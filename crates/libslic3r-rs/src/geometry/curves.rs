//! Curve fitting utilities.
//!
//! C++ Reference:
//! - Geometry/Curves.hpp  (header-only templates)
//!
//! Faithful 1:1 port of `Slic3r::Geometry` curve fitting:
//! `PolynomialCurve`, `fit_polynomial`, `PiecewiseFittedCurve`, `fit_curve`,
//! and the `fit_linear_spline` / `fit_cubic_bspline` / `fit_catmul_rom_spline`
//! convenience wrappers.
//!
//! The C++ is templated over `<int Dimension, typename NumberType>` and the
//! kernel type. All Eigen matrices in this file are `Eigen::MatrixXf` (i.e.
//! always `f32`) regardless of `NumberType`; only the observation points,
//! weights and kernel evaluation use `NumberType`. The single concrete
//! instantiation in BambuStudio (SeamPlacer.cpp) is `Dimension = 2`,
//! `NumberType = float`, so we model `NumberType` as `f32` here and keep the
//! coefficient matrices as `f32` to mirror `Eigen::MatrixXf` exactly.

use nalgebra::DMatrix;

use crate::geometry::bicubic::{
    CubicBSplineKernel, CubicCatmulRomKernel, CubicKernelWrapper, KernelCoefficients, LinearKernel,
};

// Geometry/Curves.hpp:14
// template<int Dimension, typename NumberType>
// struct PolynomialCurve {
//     Eigen::MatrixXf coefficients;
/// A polynomial curve represented by its coefficients matrix.
///
/// The coefficients matrix has `Dimension` rows and `order + 1` columns, where
/// each column corresponds to a power of the parameter (x^0, x^1, ..., x^n).
/// Mirrors `Eigen::MatrixXf` (column-major, `f32`).
#[derive(Debug, Clone)]
pub struct PolynomialCurve {
    /// `Eigen::MatrixXf coefficients;`
    pub coefficients: DMatrix<f32>,
}

impl PolynomialCurve {
    // Geometry/Curves.hpp:18
    //     Vec<Dimension, NumberType> get_fitted_value(const NumberType& value) const {
    /// Evaluate the fitted curve at a given parameter value.
    ///
    /// Returns a `Dimension`-element column vector.
    pub fn get_fitted_value(&self, value: f32) -> Vec<f32> {
        // Geometry/Curves.hpp:19
        // Vec<Dimension, NumberType> result = Vec<Dimension, NumberType>::Zero();
        let dimension = self.coefficients.nrows();
        let mut result = vec![0.0f32; dimension];
        // Geometry/Curves.hpp:20
        // size_t order = this->coefficients.rows() - 1;
        //
        // NOTE: in C++ `coefficients.rows()` is `Dimension`, NOT the polynomial
        // order (the polynomial order is `cols - 1`). This is a quirk of the
        // upstream code: it derives `order` from the *row* count and then loops
        // `order + 1` (= `Dimension`) times indexing `coefficients.col(index)`.
        // For the only real instantiation (Dimension == 2 == cols, i.e. order 1)
        // these coincide. We faithfully reproduce the C++ by driving the loop by
        // `nrows()` (= `Dimension`), and we additionally guard the column access
        // (`index < ncols()`) so that, unlike the C++ which would read out of
        // bounds when `Dimension > cols`, we stay memory-safe while producing the
        // same values whenever the C++ access is in bounds.
        let order = self.coefficients.nrows().saturating_sub(1);
        // Geometry/Curves.hpp:21
        // auto x = NumberType(1.);
        let mut x = 1.0f32;
        // Geometry/Curves.hpp:22
        // for (size_t index = 0; index < order + 1; ++index, x *= value)
        for index in 0..(order + 1) {
            // Geometry/Curves.hpp:23
            // result += x * this->coefficients.col(index);
            if index < self.coefficients.ncols() {
                for dim in 0..dimension {
                    result[dim] += x * self.coefficients[(dim, index)];
                }
            }
            x *= value;
        }
        // Geometry/Curves.hpp:24
        // return result;
        result
    }
}

// Geometry/Curves.hpp:28
// https://towardsdatascience.com/least-square-polynomial-CURVES-using-c-eigen-package-c0673728bd01
// Geometry/Curves.hpp:29
// template<int Dimension, typename NumberType>
// PolynomialCurve<Dimension, NumberType> fit_polynomial(...)
/// Fit a polynomial curve to `observations` using weighted least squares.
///
/// - `observations`: data points to fit (each is a `dimension`-length vector)
/// - `observation_points`: parameter values where observations were made
/// - `weights`: importance weight for each observation
/// - `order`: polynomial order
/// - `dimension`: number of spatial dimensions (the template `Dimension`)
pub fn fit_polynomial(
    observations: &[Vec<f32>],
    observation_points: &[f32],
    weights: &[f32],
    order: usize,
    dimension: usize,
) -> PolynomialCurve {
    // Geometry/Curves.hpp:34
    // check to make sure inputs are correct
    // size_t cols = order + 1;
    let cols = order + 1;
    // Geometry/Curves.hpp:35-37
    // assert(observation_points.size() >= cols);
    // assert(observation_points.size() == weights.size());
    // assert(observations.size() == weights.size());
    debug_assert!(observation_points.len() >= cols);
    debug_assert!(observation_points.len() == weights.len());
    debug_assert!(observations.len() == weights.len());

    // Geometry/Curves.hpp:39
    // Eigen::MatrixXf data_points(Dimension, observations.size());
    let mut data_points = DMatrix::<f32>::zeros(dimension, observations.len());
    // Geometry/Curves.hpp:40
    // Eigen::MatrixXf T(observations.size(), cols);
    let mut t = DMatrix::<f32>::zeros(observations.len(), cols);
    // Geometry/Curves.hpp:41
    // for (size_t i = 0; i < weights.size(); ++i) {
    for i in 0..weights.len() {
        // Geometry/Curves.hpp:42
        // auto squared_weight = sqrt(weights[i]);
        let squared_weight = weights[i].sqrt();
        // Geometry/Curves.hpp:43
        // data_points.col(i) = observations[i] * squared_weight;
        for dim in 0..dimension {
            data_points[(dim, i)] = observations[i][dim] * squared_weight;
        }
        // Geometry/Curves.hpp:44-46
        // Populate the matrix
        // auto x = squared_weight;
        // auto c = observation_points[i];
        let mut x = squared_weight;
        let c = observation_points[i];
        // Geometry/Curves.hpp:47
        // for (size_t j = 0; j < cols; ++j, x *= c)
        for j in 0..cols {
            // Geometry/Curves.hpp:48
            // T(i, j) = x;
            t[(i, j)] = x;
            x *= c;
        }
    }

    // Geometry/Curves.hpp:51
    // const auto QR = T.householderQr();
    let qr = t.qr();
    // Geometry/Curves.hpp:52
    // Eigen::MatrixXf coefficients(Dimension, cols);
    let mut coefficients = DMatrix::<f32>::zeros(dimension, cols);
    // Geometry/Curves.hpp:53-56
    // Solve for linear least square fit
    // for (size_t dim = 0; dim < Dimension; ++dim) {
    //     coefficients.row(dim) = QR.solve(data_points.row(dim).transpose());
    // }
    for dim in 0..dimension {
        let b = data_points.row(dim).transpose();
        let solution = solve_least_squares_qr(&qr, &b);
        for j in 0..cols {
            coefficients[(dim, j)] = solution[j];
        }
    }

    // Geometry/Curves.hpp:58
    // return {std::move(coefficients)};
    PolynomialCurve { coefficients }
}

// Geometry/Curves.hpp:61
// template<size_t Dimension, typename NumberType, typename KernelType>
// struct PiecewiseFittedCurve {
//     using Kernel = KernelType;
/// A piecewise curve fitted using a kernel function (B-spline, Catmul-Rom, ...).
///
/// The curve is defined by coefficients at evenly-spaced control points; the
/// kernel function blends nearby coefficients during evaluation. The kernel
/// type is carried as a const generic via `kernel_span` plus the generic
/// `get_fitted_value::<K>` method, mirroring the C++ `Kernel` typedef.
#[derive(Debug, Clone)]
pub struct PiecewiseFittedCurve {
    /// `Eigen::MatrixXf coefficients;` — `Dimension` rows, `parameters_count` cols.
    pub coefficients: DMatrix<f32>,
    /// `NumberType start;`
    pub start: f32,
    /// `NumberType segment_size;`
    pub segment_size: f32,
    /// `size_t endpoints_level_of_freedom;`
    pub endpoints_level_of_freedom: usize,
}

impl PiecewiseFittedCurve {
    // Geometry/Curves.hpp:70
    //     Vec<Dimension, NumberType> get_fitted_value(const NumberType &observation_point) const {
    /// Evaluate the fitted curve at `observation_point`.
    ///
    /// `K` is the kernel type used to fit the curve (e.g. `CubicBSplineKernel`).
    /// Returns a `Dimension`-element column vector.
    pub fn get_fitted_value<K: KernelCoefficients>(&self, observation_point: f32) -> Vec<f32> {
        // Geometry/Curves.hpp:71
        // Vec<Dimension, NumberType> result = Vec<Dimension, NumberType>::Zero();
        let dimension = self.coefficients.nrows();
        let mut result = vec![0.0f32; dimension];

        // Geometry/Curves.hpp:73-74
        // find corresponding segment index; expects kernels to be centered
        // int middle_right_segment_index = floor((observation_point - start) / segment_size);
        let middle_right_segment_index =
            ((observation_point - self.start) / self.segment_size).floor() as i32;
        // Geometry/Curves.hpp:75-76
        // find index of first segment that is affected by the point i; this can be deduced from kernel_span
        // int start_segment_idx = middle_right_segment_index - Kernel::kernel_span / 2 + 1;
        let kernel_span = CubicKernelWrapper::<K>::KERNEL_SPAN as i32;
        let start_segment_idx = middle_right_segment_index - kernel_span / 2 + 1;
        // Geometry/Curves.hpp:77-78
        // for (int segment_index = start_segment_idx; segment_index < int(start_segment_idx + Kernel::kernel_span); segment_index++) {
        let mut segment_index = start_segment_idx;
        while segment_index < start_segment_idx + kernel_span {
            // Geometry/Curves.hpp:79
            // NumberType segment_start = start + segment_index * segment_size;
            // one expression at the pin: fmaf(float(segment_index), segment_size, start)
            // (R937 gate CURVES_FMA; =0 gives the earlier unfused arm)
            let segment_start = if crate::faithful_gate("CURVES_FMA") {
                (segment_index as f32).mul_add(self.segment_size, self.start)
            } else {
                self.start + segment_index as f32 * self.segment_size
            };
            // Geometry/Curves.hpp:80
            // NumberType normalized_segment_distance = (segment_start - observation_point) / segment_size;
            let normalized_segment_distance =
                (segment_start - observation_point) / self.segment_size;

            // Geometry/Curves.hpp:82
            // int parameter_index = segment_index + endpoints_level_of_freedom;
            let mut parameter_index = segment_index + self.endpoints_level_of_freedom as i32;
            // Geometry/Curves.hpp:83
            // parameter_index = std::clamp(parameter_index, 0, int(coefficients.cols()) - 1);
            parameter_index = parameter_index.clamp(0, self.coefficients.ncols() as i32 - 1);
            // Geometry/Curves.hpp:84
            // result += Kernel::kernel(normalized_segment_distance) * coefficients.col(parameter_index);
            let k = cubic_kernel_value::<K>(normalized_segment_distance);
            let col = parameter_index as usize;
            for dim in 0..dimension {
                result[dim] += k * self.coefficients[(dim, col)];
            }
            segment_index += 1;
        }
        // Geometry/Curves.hpp:86
        // return result;
        result
    }
}

/// `Kernel::kernel(x)` as `fit_curve` / `get_fitted_value` call it: `CubicKernelWrapper<...<float>>`,
/// so the pin evaluates the polynomial in FLOAT, and clang (-ffp-contract=on) fuses each
/// `+ a * x^n` of the one return expression into an fmaf (Geometry/Bicubic.hpp:198-213):
/// `fmaf(a13, x3, fmaf(a12, x2, fmaf(a11, x, a10)))`, `x2 = x * x`, `x3 = x2 * x` rounded apart.
/// The coefficients are `T(c / 6.)`, i.e. the double quotient rounded to float.
/// R938 gate CURVES_KERNEL_F32: opt_in (off unless `=1`) because on the fixtures it moved Benchy arachne
/// 241,857 -> 241,855 in order (classic 190,197 -> 190,198); its counterpart is the solver
/// (`fullPivHouseholderQr`, Curves.hpp:166-169), which this file did with a thin Householder QR.
/// R951: faithful (on unless `=0`) now that CURVES_FULLPIV_QR solves as the pin: with both, the fits
/// equal the pin's (tests/curves_fullpiv_qr_native_parity.rs) and Benchy classic is the oracle's bytes.
pub fn cubic_kernel_value<K: KernelCoefficients>(x: f32) -> f32 {
    if !crate::faithful_gate("CURVES_KERNEL_F32") {
        return CubicKernelWrapper::<K>::kernel(x as f64) as f32;
    }
    cubic_kernel_f32::<K>(x)
}

/// Geometry/Bicubic.hpp:198-213 `CubicKernelWrapper<Kernel>::kernel` with `FloatType = float`.
pub fn cubic_kernel_f32<K: KernelCoefficients>(x: f32) -> f32 {
    // Bicubic.hpp:200 x = fabs(x);
    let x = x.abs();
    // Bicubic.hpp:201-202
    if x >= 2.0f32 {
        return 0.0f32;
    }
    // Bicubic.hpp:203-207
    if x <= 1.0f32 {
        let x2 = x * x;
        let x3 = x2 * x;
        return (K::a13() as f32).mul_add(
            x3,
            (K::a12() as f32).mul_add(x2, (K::a11() as f32).mul_add(x, K::a10() as f32)),
        );
    }
    // Bicubic.hpp:209-212
    let x = x - 1.0f32;
    let x2 = x * x;
    let x3 = x2 * x;
    (K::a03() as f32).mul_add(
        x3,
        (K::a02() as f32).mul_add(x2, (K::a01() as f32).mul_add(x, K::a00() as f32)),
    )
}

// Geometry/Curves.hpp:90-95
// observations: data to be fitted by the curve
// observation points: growing sequence of points where the observations were made.
//      In other words, for function f(x) = y, observations are y0...yn, and observation points are x0...xn
// weights: how important the observation is
// segments_count: number of segments inside the valid length of the curve
// endpoints_level_of_freedom: number of additional parameters at each end; reasonable values depend on the kernel span
// Geometry/Curves.hpp:96
// template<typename Kernel, int Dimension, typename NumberType>
// PiecewiseFittedCurve<Dimension, NumberType, Kernel> fit_curve(...)
/// Fit a piecewise kernel curve to `observations` using weighted least squares.
///
/// `K` is the kernel type (e.g. `CubicBSplineKernel`).
pub fn fit_curve<K: KernelCoefficients>(
    observations: &[Vec<f32>],
    observation_points: &[f32],
    weights: &[f32],
    segments_count: usize,
    endpoints_level_of_freedom: usize,
    dimension: usize,
) -> PiecewiseFittedCurve {
    // Geometry/Curves.hpp:104-109
    // check to make sure inputs are correct
    // assert(segments_count > 0);
    // assert(observations.size() > 0);
    // assert(observation_points.size() == observations.size());
    // assert(observation_points.size() == weights.size());
    // assert(segments_count <= observations.size());
    debug_assert!(segments_count > 0);
    debug_assert!(!observations.is_empty());
    debug_assert!(observation_points.len() == observations.len());
    debug_assert!(observation_points.len() == weights.len());
    debug_assert!(segments_count <= observations.len());

    // Geometry/Curves.hpp:111-116
    // prepare sqrt of weights, which will then be applied to both matrix T and observed data:
    //   https://en.wikipedia.org/wiki/Weighted_least_squares
    // std::vector<NumberType> sqrt_weights(weights.size());
    // for (size_t index = 0; index < weights.size(); ++index) {
    //     assert(weights[index] > 0);
    //     sqrt_weights[index] = sqrt(weights[index]);
    // }
    let mut sqrt_weights = vec![0.0f32; weights.len()];
    for index in 0..weights.len() {
        debug_assert!(weights[index] > 0.0);
        sqrt_weights[index] = weights[index].sqrt();
    }

    // Geometry/Curves.hpp:118-125
    // prepare result and compute metadata
    // PiecewiseFittedCurve<Dimension, NumberType, Kernel> result { };
    // NumberType valid_length = observation_points.back() - observation_points.front();
    // NumberType segment_size = valid_length / NumberType(segments_count);
    // result.start = observation_points.front();
    // result.segment_size = segment_size;
    // result.endpoints_level_of_freedom = endpoints_level_of_freedom;
    let valid_length =
        observation_points[observation_points.len() - 1] - observation_points[0];
    let segment_size = valid_length / segments_count as f32;
    let start = observation_points[0];

    // Geometry/Curves.hpp:127-132
    // prepare observed data
    // Eigen defaults to column major memory layout.
    // Eigen::MatrixXf data_points(Dimension, observations.size());
    // for (size_t index = 0; index < observations.size(); ++index) {
    //     data_points.col(index) = observations[index] * sqrt_weights[index];
    // }
    let mut data_points = DMatrix::<f32>::zeros(dimension, observations.len());
    for index in 0..observations.len() {
        for dim in 0..dimension {
            data_points[(dim, index)] = observations[index][dim] * sqrt_weights[index];
        }
    }
    // Geometry/Curves.hpp:133-135
    // parameters count is always increased by one to make the parametric space of the curve symmetric.
    // without this fix, the end of the curve is less flexible than the beginning
    // size_t parameters_count = segments_count + 1 + 2 * endpoints_level_of_freedom;
    let parameters_count = segments_count + 1 + 2 * endpoints_level_of_freedom;
    // Geometry/Curves.hpp:136-138
    // Create weight matrix T for each point and each segment;
    // Eigen::MatrixXf T(observation_points.size(), parameters_count);
    // T.setZero();
    let mut t = DMatrix::<f32>::zeros(observation_points.len(), parameters_count);
    let kernel_span = CubicKernelWrapper::<K>::KERNEL_SPAN as i32;
    // Geometry/Curves.hpp:139-140
    // Fill the weight matrix
    // for (size_t i = 0; i < observation_points.size(); ++i) {
    for i in 0..observation_points.len() {
        // Geometry/Curves.hpp:141
        // NumberType observation_point = observation_points[i];
        let observation_point = observation_points[i];
        // Geometry/Curves.hpp:142-143
        // find corresponding segment index; expects kernels to be centered
        // int middle_right_segment_index = floor((observation_point - result.start) / result.segment_size);
        let middle_right_segment_index =
            ((observation_point - start) / segment_size).floor() as i32;
        // Geometry/Curves.hpp:144-145
        // find index of first segment that is affected by the point i; this can be deduced from kernel_span
        // int start_segment_idx = middle_right_segment_index - int(Kernel::kernel_span / 2) + 1;
        let start_segment_idx = middle_right_segment_index - kernel_span / 2 + 1;
        // Geometry/Curves.hpp:146-147
        // for (int segment_index = start_segment_idx; segment_index < int(start_segment_idx + Kernel::kernel_span); segment_index++) {
        let mut segment_index = start_segment_idx;
        while segment_index < start_segment_idx + kernel_span {
            // Geometry/Curves.hpp:148
            // NumberType segment_start = result.start + segment_index * result.segment_size;
            // fused at the pin (R937 gate CURVES_FMA; =0 gives the earlier unfused arm)
            let segment_start = if crate::faithful_gate("CURVES_FMA") {
                (segment_index as f32).mul_add(segment_size, start)
            } else {
                start + segment_index as f32 * segment_size
            };
            // Geometry/Curves.hpp:149
            // NumberType normalized_segment_distance = (segment_start - observation_point) / result.segment_size;
            let normalized_segment_distance = (segment_start - observation_point) / segment_size;

            // Geometry/Curves.hpp:151
            // int parameter_index = segment_index + endpoints_level_of_freedom;
            let mut parameter_index = segment_index + endpoints_level_of_freedom as i32;
            // Geometry/Curves.hpp:152
            // parameter_index = std::clamp(parameter_index, 0, int(parameters_count) - 1);
            parameter_index = parameter_index.clamp(0, parameters_count as i32 - 1);
            // Geometry/Curves.hpp:153
            // T(i, parameter_index) += Kernel::kernel(normalized_segment_distance) * sqrt_weights[i];
            let k = cubic_kernel_value::<K>(normalized_segment_distance);
            // `T(i, j) += kernel * sqrt_weights[i]` is fused at the pin (R937 gate CURVES_FMA)
            if crate::faithful_gate("CURVES_FMA") {
                t[(i, parameter_index as usize)] =
                    k.mul_add(sqrt_weights[i], t[(i, parameter_index as usize)]);
            } else {
                t[(i, parameter_index as usize)] += k * sqrt_weights[i];
            }
            segment_index += 1;
        }
    }

    // Geometry/Curves.hpp:157-166 (#ifdef LSQR_DEBUG) — debug-only print, omitted.

    // Geometry/Curves.hpp:168-173
    // Solve for linear least square fit
    // result.coefficients.resize(Dimension, parameters_count);
    // const auto QR = T.fullPivHouseholderQr();
    // for (size_t dim = 0; dim < Dimension; ++dim) {
    //     result.coefficients.row(dim) = QR.solve(data_points.row(dim).transpose());
    // }
    //
    // NOTE: C++ uses `fullPivHouseholderQr()`. nalgebra has no full-pivoting
    // Householder QR. Its column-pivoting `ColPivQR::solve_mut` is only
    // implemented for *square* systems (it asserts `is_square()` and panics on
    // a non-square matrix), and `T` here is tall/rectangular
    // (observation_points.len() x parameters_count), so it cannot be used.
    // We therefore use the plain Householder QR thin decomposition and solve
    // the overdetermined least-squares system manually via `Q^T b` plus
    // upper-triangular back substitution against the thin `R` (the same path as
    // `fit_polynomial`). This is exactly Eigen's `HouseholderQR::solve` and
    // yields the same mathematical least-squares minimizer as
    // `fullPivHouseholderQr().solve()`; the solver choice can introduce
    // floating-point divergence vs Eigen but the minimizer is identical.
    let mut coefficients = DMatrix::<f32>::zeros(dimension, parameters_count);
    // CURVES_FULLPIV_QR (faithful, R951): `T.fullPivHouseholderQr()` and its `solve` as the pin's
    // build runs them (Eigen 5.0.1, float, NEON 4-lane packets with FMA, -ffp-contract=on):
    // FullPivHouseholderQR.h computeInPlace / _solve_impl, ported in `FullPivHouseholderQrF32`.
    // =0 keeps the thin Householder QR below.
    if crate::faithful_gate("CURVES_FULLPIV_QR") {
        let qr = FullPivHouseholderQrF32::new(&t);
        for dim in 0..dimension {
            let b: Vec<f32> = (0..observation_points.len()).map(|i| data_points[(dim, i)]).collect();
            let solution = qr.solve(&b);
            for j in 0..parameters_count {
                coefficients[(dim, j)] = solution[j];
            }
        }
        return PiecewiseFittedCurve {
            coefficients,
            start,
            segment_size,
            endpoints_level_of_freedom,
        };
    }
    let qr = t.qr();
    for dim in 0..dimension {
        let b = data_points.row(dim).transpose();
        let solution = solve_least_squares_qr(&qr, &b);
        for j in 0..parameters_count {
            coefficients[(dim, j)] = solution[j];
        }
    }

    // Geometry/Curves.hpp:175
    // return result;
    PiecewiseFittedCurve {
        coefficients,
        start,
        segment_size,
        endpoints_level_of_freedom,
    }
}

// Geometry/Curves.hpp:179-189
// template<int Dimension, typename NumberType>
// PiecewiseFittedCurve<Dimension, NumberType, LinearKernel<NumberType>>
// fit_linear_spline(...)
/// Fit a piecewise linear spline (uses `LinearKernel`).
pub fn fit_linear_spline(
    observations: &[Vec<f32>],
    observation_points: &[f32],
    weights: &[f32],
    segments_count: usize,
    endpoints_level_of_freedom: usize,
    dimension: usize,
) -> PiecewiseFittedCurve {
    fit_curve::<LinearKernel>(
        observations,
        observation_points,
        weights,
        segments_count,
        endpoints_level_of_freedom,
        dimension,
    )
}

// Geometry/Curves.hpp:191-201
// template<int Dimension, typename NumberType>
// PiecewiseFittedCurve<Dimension, NumberType, CubicBSplineKernel<NumberType>>
// fit_cubic_bspline(...)
/// Fit a piecewise cubic B-spline (uses `CubicBSplineKernel`).
pub fn fit_cubic_bspline(
    observations: &[Vec<f32>],
    observation_points: &[f32],
    weights: &[f32],
    segments_count: usize,
    endpoints_level_of_freedom: usize,
    dimension: usize,
) -> PiecewiseFittedCurve {
    fit_curve::<CubicBSplineKernel>(
        observations,
        observation_points,
        weights,
        segments_count,
        endpoints_level_of_freedom,
        dimension,
    )
}

// Geometry/Curves.hpp:203-213
// template<int Dimension, typename NumberType>
// PiecewiseFittedCurve<Dimension, NumberType, CubicCatmulRomKernel<NumberType>>
// fit_catmul_rom_spline(...)
/// Fit a piecewise Catmul-Rom spline (uses `CubicCatmulRomKernel`).
pub fn fit_catmul_rom_spline(
    observations: &[Vec<f32>],
    observation_points: &[f32],
    weights: &[f32],
    segments_count: usize,
    endpoints_level_of_freedom: usize,
    dimension: usize,
) -> PiecewiseFittedCurve {
    fit_curve::<CubicCatmulRomKernel>(
        observations,
        observation_points,
        weights,
        segments_count,
        endpoints_level_of_freedom,
        dimension,
    )
}

/// Solve an overdetermined least-squares system `T x = b` from a Householder
/// `QR` decomposition of the (tall) matrix `T` (m x c).
///
/// Mirrors Eigen's `HouseholderQR::solve`: for `T = Q R` (thin), the minimizer
/// of `||T x - b||` is `x = R^{-1} (Q^T b)`. nalgebra's `QR::solve` is only
/// implemented for square matrices, so we form `Q^T b` and back-substitute
/// against the upper-triangular `R` ourselves.
///
/// The returned vector always has length `c` (the column count of `T`), so the
/// caller can index columns `0..c` directly. For the tall/square case (m >= c,
/// the only case that arises in the BambuStudio caller) this is the exact
/// thin-QR least-squares solution. For a wide `T` (m < c) the thin `R` is only
/// `m x c`, so we solve the leading `m x m` block and leave the trailing
/// `c - m` entries at zero — a best-effort result that keeps the column count
/// stable without panicking.
fn solve_least_squares_qr(
    qr: &nalgebra::QR<f32, nalgebra::Dyn, nalgebra::Dyn>,
    b: &nalgebra::DVector<f32>,
) -> nalgebra::DVector<f32> {
    // For tall/square T (m x c, m >= c): Q is m x c (thin), R is c x c.
    // For wide T (m x c, m < c): Q is m x m, R is m x c; rank <= m.
    let q = qr.q();
    let r = qr.r();
    // Number of solvable unknowns = leading square block of R = min(m, c).
    let rank_dim = r.nrows().min(r.ncols());
    let cols = r.ncols();
    // Q^T b  (length = min(m, c))
    let qtb = q.transpose() * b;
    // Solve R[0..rank, 0..rank] x = (Q^T b)[0..rank] by back substitution.
    let mut x_lead = qtb.rows(0, rank_dim).into_owned();
    let r_lead = r.view((0, 0), (rank_dim, rank_dim)).into_owned();
    if !r_lead.solve_upper_triangular_mut(&mut x_lead) {
        // Singular R: leave x as-is (best effort), matching Eigen returning
        // whatever the triangular solve produces.
    }
    // Pad to full column count so the caller can index columns 0..c.
    let mut x = nalgebra::DVector::<f32>::zeros(cols);
    for i in 0..rank_dim {
        x[i] = x_lead[i];
    }
    x
}

// ---------------------------------------------------------------------------------------------
// Eigen 5.0.1 `FullPivHouseholderQR<MatrixXf>` as the pin's build runs it (R951).
//
// The pin compiles Curves.hpp inside SeamPlacer.cpp with clang, arm64, -O3, -ffp-contract=on,
// against the Eigen the oracle was built with (5.0.1, docs/rust-engine/README.md). Eigen
// vectorizes float with NEON 4-lane packets (Packet4f, half packet Packet2f) and has
// EIGEN_VECTORIZE_FMA, so `pmadd` is a fused multiply-add per lane; the kernels below reproduce
// the order of every rounding the vectorized code makes.
// ---------------------------------------------------------------------------------------------

/// `predux<Packet4f>` on aarch64: `vaddvq_f32`, i.e. `(a0 + a1) + (a2 + a3)` (NEON/PacketMath.h:3347-3349).
#[inline]
fn eigen_predux4(p: [f32; 4]) -> f32 {
    (p[0] + p[1]) + (p[2] + p[3])
}

/// `predux<Packet2f>` on aarch64: `vaddv_f32` (NEON/PacketMath.h:3343-3345).
#[inline]
fn eigen_predux2(p: [f32; 2]) -> f32 {
    p[0] + p[1]
}

/// `redux_impl<scalar_sum_op, Evaluator, LinearVectorizedTraversal, NoUnrolling>::run` (Redux.h:275-324)
/// over an expression without direct access (`first_default_aligned` is 0): two packet
/// accumulators, `padd` of the packets of `coeff`, `predux`, then the tail one scalar add at a time.
/// `coeff(i)` is the rounded coefficient (a square for `squaredNorm`, a product for a lazy
/// coefficient-based product). `DenseBase::sum` of an empty expression is 0 (Redux.h).
fn eigen_redux_sum(size: usize, coeff: impl Fn(usize) -> f32) -> f32 {
    if size == 0 {
        return 0.0;
    }
    const PS: usize = 4;
    let packet = |index: usize| -> [f32; 4] {
        [coeff(index), coeff(index + 1), coeff(index + 2), coeff(index + 3)]
    };
    let padd = |a: [f32; 4], b: [f32; 4]| -> [f32; 4] {
        [a[0] + b[0], a[1] + b[1], a[2] + b[2], a[3] + b[3]]
    };
    let aligned_start = 0usize;
    let aligned_size2 = ((size - aligned_start) / (2 * PS)) * (2 * PS);
    let aligned_size = ((size - aligned_start) / PS) * PS;
    let aligned_end2 = aligned_start + aligned_size2;
    let aligned_end = aligned_start + aligned_size;
    if aligned_size > 0 {
        let mut packet_res0 = packet(aligned_start);
        if aligned_size > PS {
            let mut packet_res1 = packet(aligned_start + PS);
            let mut index = aligned_start + 2 * PS;
            while index < aligned_end2 {
                packet_res0 = padd(packet_res0, packet(index));
                packet_res1 = padd(packet_res1, packet(index + PS));
                index += 2 * PS;
            }
            packet_res0 = padd(packet_res0, packet_res1);
            if aligned_end > aligned_end2 {
                packet_res0 = padd(packet_res0, packet(aligned_end2));
            }
        }
        let mut res = eigen_predux4(packet_res0);
        for index in aligned_end..size {
            res = res + coeff(index);
        }
        res
    } else {
        let mut res = coeff(0);
        for index in 1..size {
            res = res + coeff(index);
        }
        res
    }
}

/// `MatrixBase::dot` (Dot.h:52-54 -> InnerProduct.h `inner_product_impl<Evaluator, true>::run`):
/// four packet accumulators, `pmul` for the first packet of each, `pmadd` (fused) after, the
/// accumulators added pairwise, `predux`, and the tail through the scalar `pmadd`, which is
/// `numext::madd` = `x * y + z` contracted to an fma (MathFunctions.h:1001-1005). Fewer than four
/// coefficients take the scalar loop (`pmul` then `pmadd`).
fn eigen_dot(a: &[f32], b: &[f32]) -> f32 {
    let size = a.len();
    const PS: usize = 4;
    if size < PS {
        if size == 0 {
            return 0.0;
        }
        let mut result = a[0] * b[0];
        for k in 1..size {
            result = a[k].mul_add(b[k], result);
        }
        return result;
    }
    let pmul = |k: usize| -> [f32; 4] { [a[k] * b[k], a[k + 1] * b[k + 1], a[k + 2] * b[k + 2], a[k + 3] * b[k + 3]] };
    let pmadd = |acc: [f32; 4], k: usize| -> [f32; 4] {
        [
            a[k].mul_add(b[k], acc[0]),
            a[k + 1].mul_add(b[k + 1], acc[1]),
            a[k + 2].mul_add(b[k + 2], acc[2]),
            a[k + 3].mul_add(b[k + 3], acc[3]),
        ]
    };
    let padd = |x: [f32; 4], y: [f32; 4]| -> [f32; 4] { [x[0] + y[0], x[1] + y[1], x[2] + y[2], x[3] + y[3]] };
    let packet_end = size / PS * PS;
    let quad_end = size / (4 * PS) * (4 * PS);
    let num_packets = size / PS;
    let num_rem_packets = (packet_end - quad_end) / PS;
    let mut presult0 = pmul(0);
    let mut presult1 = [0.0f32; 4];
    let mut presult2 = [0.0f32; 4];
    if num_packets >= 2 {
        presult1 = pmul(PS);
    }
    if num_packets >= 3 {
        presult2 = pmul(2 * PS);
    }
    if num_packets >= 4 {
        let mut presult3 = pmul(3 * PS);
        let mut k = 4 * PS;
        while k < quad_end {
            presult0 = pmadd(presult0, k);
            presult1 = pmadd(presult1, k + PS);
            presult2 = pmadd(presult2, k + 2 * PS);
            presult3 = pmadd(presult3, k + 3 * PS);
            k += 4 * PS;
        }
        if num_rem_packets >= 1 {
            presult0 = pmadd(presult0, quad_end);
        }
        if num_rem_packets >= 2 {
            presult1 = pmadd(presult1, quad_end + PS);
        }
        if num_rem_packets == 3 {
            presult2 = pmadd(presult2, quad_end + 2 * PS);
        }
        presult2 = padd(presult2, presult3);
    }
    if num_packets >= 3 {
        presult1 = padd(presult1, presult2);
    }
    if num_packets >= 2 {
        presult0 = padd(presult0, presult1);
    }
    let mut result = eigen_predux4(presult0);
    for k in packet_end..size {
        result = a[k].mul_add(b[k], result);
    }
    result
}

/// `general_matrix_vector_product<..., RowMajor, ...>::run` (GeneralMatrixVector.h:300-470) with
/// `alpha = 1` into a zeroed destination: `res[r] = sum_c lhs(r, c) * rhs[c]`. Rows in groups of
/// 8, 4 and 2 take full packets (fused per lane) and a scalar tail with a separate product and add;
/// a last single row also takes a half packet. `res += alpha * cc` is contracted (fma(1, cc, 0)).
fn eigen_gemv_row_major(rows: usize, cols: usize, lhs: impl Fn(usize, usize) -> f32, rhs: &[f32]) -> Vec<f32> {
    let full_col_block_end = 4 * (cols / 4);
    let half_col_block_end = 2 * (cols / 2);
    let mut res = vec![0.0f32; rows];
    let grouped_rows = rows - rows % 2;
    for r in 0..rows {
        let mut c = [0.0f32; 4];
        let mut j = 0;
        while j < full_col_block_end {
            for l in 0..4 {
                c[l] = lhs(r, j + l).mul_add(rhs[j + l], c[l]);
            }
            j += 4;
        }
        let mut cc = eigen_predux4(c);
        let mut tail_start = full_col_block_end;
        if r >= grouped_rows {
            // The single-row loop: half packets (Packet2f), no quarter packets for float on NEON.
            let mut ch = [0.0f32; 2];
            let mut j = full_col_block_end;
            while j < half_col_block_end {
                for l in 0..2 {
                    ch[l] = lhs(r, j + l).mul_add(rhs[j + l], ch[l]);
                }
                j += 2;
            }
            cc = cc + eigen_predux2(ch);
            tail_start = half_col_block_end;
        }
        for j in tail_start..cols {
            cc = cc + lhs(r, j) * rhs[j];
        }
        res[r] = 1.0f32.mul_add(cc, res[r]);
    }
    res
}

/// `general_matrix_vector_product<..., ColMajor, ...>::run` (GeneralMatrixVector.h:107-262) with
/// `alpha = -1`, as the triangular solve calls it: `res[r] -= sum_c lhs(r, c) * rhs[c]`. Rows in
/// full and half packets accumulate fused per lane; a last odd row accumulates with a separate
/// product and add. `pmadd(c, palpha, res)` / `res += alpha * c` with alpha -1 is `res - c`.
fn eigen_gemv_col_major_minus(rows: usize, cols: usize, lhs: impl Fn(usize, usize) -> f32, rhs: &[f32], res: &mut [f32]) {
    let packet_rows = rows - rows % 2;
    for r in 0..rows {
        let mut c = 0.0f32;
        if r < packet_rows {
            for j in 0..cols {
                c = lhs(r, j).mul_add(rhs[j], c);
            }
            res[r] = c.mul_add(-1.0, res[r]);
        } else {
            for j in 0..cols {
                c = c + lhs(r, j) * rhs[j];
            }
            res[r] = (-1.0f32).mul_add(c, res[r]);
        }
    }
}

/// `Eigen::FullPivHouseholderQR<Eigen::MatrixXf>` (QR/FullPivHouseholderQR.h), the decomposition
/// `fit_curve` solves with (Curves.hpp:168-173). Column-major storage as `MatrixXf`.
#[derive(Debug, Clone)]
pub struct FullPivHouseholderQrF32 {
    /// `m_qr`, column-major `rows x cols`.
    qr: Vec<f32>,
    rows: usize,
    cols: usize,
    /// `m_hCoeffs`
    h_coeffs: Vec<f32>,
    /// `m_rows_transpositions`
    rows_transpositions: Vec<usize>,
    /// `m_cols_transpositions`
    cols_transpositions: Vec<usize>,
    /// `m_cols_permutation.indices()`
    cols_permutation: Vec<usize>,
    /// `m_nonzero_pivots`
    nonzero_pivots: usize,
    /// `m_maxpivot`
    maxpivot: f32,
}

impl FullPivHouseholderQrF32 {
    /// `FullPivHouseholderQR(const EigenBase<InputType>& matrix)` + `computeInPlace()`
    /// (FullPivHouseholderQR.h, computeInPlace).
    pub fn new(matrix: &DMatrix<f32>) -> Self {
        let rows = matrix.nrows();
        let cols = matrix.ncols();
        // nalgebra's DMatrix is column-major, as MatrixXf.
        let qr: Vec<f32> = matrix.as_slice().to_vec();
        let mut me = FullPivHouseholderQrF32 {
            qr,
            rows,
            cols,
            h_coeffs: Vec::new(),
            rows_transpositions: Vec::new(),
            cols_transpositions: Vec::new(),
            cols_permutation: Vec::new(),
            nonzero_pivots: 0,
            maxpivot: 0.0,
        };
        me.compute_in_place();
        me
    }

    #[inline]
    fn at(&self, i: usize, j: usize) -> f32 {
        self.qr[i + j * self.rows]
    }

    #[inline]
    fn at_mut(&mut self, i: usize, j: usize) -> &mut f32 {
        let rows = self.rows;
        &mut self.qr[i + j * rows]
    }

    /// `FullPivHouseholderQR::computeInPlace()`.
    fn compute_in_place(&mut self) {
        let rows = self.rows;
        let cols = self.cols;
        let size = rows.min(cols);
        self.h_coeffs = vec![0.0; size];
        // m_precision = NumTraits<Scalar>::epsilon() * RealScalar(size);
        let precision = f32::EPSILON * size as f32;
        self.rows_transpositions = vec![0; size];
        self.cols_transpositions = vec![0; size];
        let mut biggest = 0.0f32;
        self.nonzero_pivots = size;
        self.maxpivot = 0.0;
        for k in 0..size {
            // Score score = m_qr.bottomRightCorner(rows - k, cols - k).unaryExpr(Scoring())
            //                   .maxCoeff(&row_of_biggest_in_corner, &col_of_biggest_in_corner);
            // FindCoeff.h: the first coefficient of the largest |x| in column-major order (strict >;
            // the vectorized loop re-scans the winning packet, which gives the same position).
            let mut score = self.at(k, k).abs();
            let mut row_of_biggest_in_corner = k;
            let mut col_of_biggest_in_corner = k;
            for j in k..cols {
                for i in k..rows {
                    let v = self.at(i, j).abs();
                    if v > score {
                        score = v;
                        row_of_biggest_in_corner = i;
                        col_of_biggest_in_corner = j;
                    }
                }
            }
            let biggest_in_corner = score;
            if k == 0 {
                biggest = biggest_in_corner;
            }
            // if (internal::isMuchSmallerThan(biggest_in_corner, biggest, m_precision))
            if biggest_in_corner.abs() <= biggest.abs() * precision {
                self.nonzero_pivots = k;
                for i in k..size {
                    self.rows_transpositions[i] = i;
                    self.cols_transpositions[i] = i;
                    self.h_coeffs[i] = 0.0;
                }
                break;
            }
            self.rows_transpositions[k] = row_of_biggest_in_corner;
            self.cols_transpositions[k] = col_of_biggest_in_corner;
            if k != row_of_biggest_in_corner {
                // m_qr.row(k).tail(cols - k).swap(m_qr.row(row_of_biggest_in_corner).tail(cols - k));
                for j in k..cols {
                    self.qr.swap(k + j * rows, row_of_biggest_in_corner + j * rows);
                }
            }
            if k != col_of_biggest_in_corner {
                // m_qr.col(k).swap(m_qr.col(col_of_biggest_in_corner));
                for i in 0..rows {
                    self.qr.swap(i + k * rows, i + col_of_biggest_in_corner * rows);
                }
            }
            // m_qr.col(k).tail(rows - k).makeHouseholderInPlace(m_hCoeffs.coeffRef(k), beta);
            let (tau, beta) = self.make_householder_in_place(k);
            self.h_coeffs[k] = tau;
            *self.at_mut(k, k) = beta;
            if beta.abs() > self.maxpivot {
                self.maxpivot = beta.abs();
            }
            // m_qr.bottomRightCorner(rows - k, cols - k - 1)
            //     .applyHouseholderOnTheLeft(m_qr.col(k).tail(rows - k - 1), m_hCoeffs.coeffRef(k), ..);
            self.apply_householder_on_the_left_corner(k, tau);
        }
        // m_cols_permutation.setIdentity(cols);
        // for (Index k = 0; k < size; ++k) m_cols_permutation.applyTranspositionOnTheRight(k, m_cols_transpositions.coeff(k));
        self.cols_permutation = (0..cols).collect();
        for k in 0..size {
            self.cols_permutation.swap(k, self.cols_transpositions[k]);
        }
    }

    /// `MatrixBase::makeHouseholder` (Householder/Householder.h) on `m_qr.col(k).tail(rows - k)`:
    /// returns (tau, beta) and writes the essential part into the tail.
    fn make_householder_in_place(&mut self, k: usize) -> (f32, f32) {
        let rows = self.rows;
        let n = rows - k;
        let base = k + k * rows;
        // RealScalar tailSqNorm = size() == 1 ? RealScalar(0) : tail.squaredNorm();
        let tail_sq_norm = if n == 1 {
            0.0
        } else {
            let qr = &self.qr;
            eigen_redux_sum(n - 1, |i| {
                let v = qr[base + 1 + i];
                v * v
            })
        };
        let c0 = self.qr[base];
        let tol = f32::MIN_POSITIVE;
        if tail_sq_norm <= tol {
            for i in 1..n {
                self.qr[base + i] = 0.0;
            }
            (0.0, c0)
        } else {
            // beta = numext::sqrt(numext::abs2(c0) + tailSqNorm): abs2 is a call, no contraction.
            let mut beta = (c0 * c0 + tail_sq_norm).sqrt();
            if c0 >= 0.0 {
                beta = -beta;
            }
            // essential = tail / (c0 - beta);
            let d = c0 - beta;
            for i in 1..n {
                self.qr[base + i] = self.qr[base + i] / d;
            }
            // tau = conj((beta - c0) / beta);
            ((beta - c0) / beta, beta)
        }
    }

    /// `MatrixBase::applyHouseholderOnTheLeft` (Householder/Householder.h) on
    /// `m_qr.bottomRightCorner(rows - k, cols - k - 1)` with `essential = m_qr.col(k).tail(rows - k - 1)`.
    fn apply_householder_on_the_left_corner(&mut self, k: usize, tau: f32) {
        let rows = self.rows;
        let cols = self.cols;
        let nr = rows - k;
        let nc = cols - k - 1;
        if nr == 1 {
            // *this *= Scalar(1) - tau;
            let f = 1.0 - tau;
            for j in 0..nc {
                *self.at_mut(k, k + 1 + j) *= f;
            }
            return;
        }
        if tau == 0.0 || nc == 0 {
            return;
        }
        let ne = nr - 1;
        let essential: Vec<f32> = (0..ne).map(|i| self.at(k + 1 + i, k)).collect();
        // tmp.noalias() = essential.adjoint() * bottom;  (GemvProduct; a runtime 1x1 result falls
        // back to `dst(0,0) += alpha * lhs.row(0).dot(rhs.col(0))`, ProductEvaluators.h:381-387)
        let mut tmp: Vec<f32> = if nc == 1 {
            let col: Vec<f32> = (0..ne).map(|i| self.at(k + 1 + i, k + 1)).collect();
            vec![1.0f32.mul_add(eigen_dot(&essential, &col), 0.0)]
        } else {
            let qr = &self.qr;
            eigen_gemv_row_major(nc, ne, |r, c| qr[(k + 1 + c) + (k + 1 + r) * rows], &essential)
        };
        // tmp += this->row(0);
        for j in 0..nc {
            tmp[j] = tmp[j] + self.at(k, k + 1 + j);
        }
        // this->row(0) -= tau * tmp;
        for j in 0..nc {
            let v = self.at(k, k + 1 + j) - tau * tmp[j];
            *self.at_mut(k, k + 1 + j) = v;
        }
        // bottom.noalias() -= tau * essential * tmp;  (OuterProduct: `tau * essential` evaluated
        // into a temporary, then `dst.col(j) -= tmp(j) * actual_lhs`, ProductEvaluators.h:259-268)
        let te: Vec<f32> = essential.iter().map(|e| tau * e).collect();
        for j in 0..nc {
            let tj = tmp[j];
            for i in 0..ne {
                let v = self.at(k + 1 + i, k + 1 + j) - tj * te[i];
                *self.at_mut(k + 1 + i, k + 1 + j) = v;
            }
        }
    }

    /// `FullPivHouseholderQR::threshold()` with no prescribed threshold:
    /// `NumTraits<Scalar>::epsilon() * RealScalar(m_qr.diagonalSize())`.
    fn threshold(&self) -> f32 {
        f32::EPSILON * self.rows.min(self.cols) as f32
    }

    /// `FullPivHouseholderQR::rank()`.
    pub fn rank(&self) -> usize {
        let premultiplied_threshold = self.maxpivot.abs() * self.threshold();
        let mut result = 0;
        for i in 0..self.nonzero_pivots {
            if self.at(i, i).abs() > premultiplied_threshold {
                result += 1;
            }
        }
        result
    }

    /// `FullPivHouseholderQR::_solve_impl(rhs, dst)` for one right-hand side (`rows` values),
    /// as `QR.solve(data_points.row(dim).transpose())` calls it (Curves.hpp:171-173).
    pub fn solve(&self, rhs: &[f32]) -> Vec<f32> {
        let rows = self.rows;
        let cols = self.cols;
        let l_rank = self.rank();
        let mut dst = vec![0.0f32; cols];
        if l_rank == 0 {
            return dst;
        }
        let mut c: Vec<f32> = rhs.to_vec();
        for k in 0..l_rank {
            let remaining_size = rows - k;
            // c.row(k).swap(c.row(m_rows_transpositions.coeff(k)));
            c.swap(k, self.rows_transpositions[k]);
            // c.bottomRightCorner(remainingSize, rhs.cols()).applyHouseholderOnTheLeft(
            //     m_qr.col(k).tail(remainingSize - 1), m_hCoeffs.coeff(k), &temp.coeffRef(0));
            let tau = self.h_coeffs[k];
            if remaining_size == 1 {
                c[k] *= 1.0 - tau;
                continue;
            }
            if tau == 0.0 {
                continue;
            }
            let ne = remaining_size - 1;
            // tmp.noalias() = essential.adjoint() * bottom: the block of a vector has one column at
            // most, so the product is coefficient-based (GeneralProduct.h product_type_selector<1,
            // Small, Large>) and its coefficient is `(lhs.row(0).transpose().cwiseProduct(rhs.col(0))).sum()`
            // (ProductEvaluators.h:607-609): products rounded, then the vectorized sum.
            let mut tmp = {
                let qr = &self.qr;
                let cc = &c;
                eigen_redux_sum(ne, |i| qr[(k + 1 + i) + k * rows] * cc[k + 1 + i])
            };
            // tmp += this->row(0);
            tmp = tmp + c[k];
            // this->row(0) -= tau * tmp;
            c[k] = c[k] - tau * tmp;
            // bottom.noalias() -= tau * essential * tmp;  (lazy: (tau * e_i) * tmp)
            for i in 0..ne {
                let te = tau * self.at(k + 1 + i, k);
                c[k + 1 + i] = c[k + 1 + i] - te * tmp;
            }
        }
        // m_qr.topLeftCorner(l_rank, l_rank).template triangularView<Upper>().solveInPlace(c.topRows(l_rank));
        self.triangular_solve_upper_in_place(l_rank, &mut c);
        // for (Index i = 0; i < l_rank; ++i) dst.row(m_cols_permutation.indices().coeff(i)) = c.row(i);
        // for (Index i = l_rank; i < cols(); ++i) dst.row(m_cols_permutation.indices().coeff(i)).setZero();
        for i in 0..l_rank {
            dst[self.cols_permutation[i]] = c[i];
        }
        for i in l_rank..cols {
            dst[self.cols_permutation[i]] = 0.0;
        }
        dst
    }

    /// `triangular_solve_vector<float, float, Index, OnTheLeft, Upper, false, ColMajor>::run`
    /// (products/TriangularSolverVector.h:77-116), panel width EIGEN_TUNE_TRIANGULAR_PANEL_WIDTH = 8
    /// (arch/Default/Settings.h:37).
    fn triangular_solve_upper_in_place(&self, size: usize, rhs: &mut [f32]) {
        const PANEL_WIDTH: usize = 8;
        let rows = self.rows;
        let mut pi = size;
        while pi > 0 {
            let actual_panel_width = pi.min(PANEL_WIDTH);
            let start_block = pi - actual_panel_width;
            for k in 0..actual_panel_width {
                let i = pi - k - 1;
                if rhs[i] != 0.0 {
                    rhs[i] = rhs[i] / self.at(i, i);
                    let r = actual_panel_width - k - 1;
                    let s = i - r;
                    // Map(rhs + s, r) -= rhs[i] * cjLhs.col(i).segment(s, r);
                    let xi = rhs[i];
                    for t in 0..r {
                        rhs[s + t] = rhs[s + t] - xi * self.at(s + t, i);
                    }
                }
            }
            let r = start_block;
            if r > 0 {
                let x: Vec<f32> = rhs[start_block..start_block + actual_panel_width].to_vec();
                let qr = &self.qr;
                eigen_gemv_col_major_minus(
                    r,
                    actual_panel_width,
                    |rr, cc| qr[rr + (start_block + cc) * rows],
                    &x,
                    &mut rhs[0..r],
                );
            }
            pi -= actual_panel_width;
        }
    }
}
