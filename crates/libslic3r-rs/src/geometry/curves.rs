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
        // order. The number of columns is `order + 1`. The loop below iterates
        // `order + 1` times, i.e. once per column, multiplying the running power
        // `x` by the matched column. We therefore drive the loop by the column
        // count to reproduce the C++ behaviour faithfully.
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
            let segment_start = self.start + segment_index as f32 * self.segment_size;
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
            let k =
                CubicKernelWrapper::<K>::kernel(normalized_segment_distance as f64) as f32;
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
            let segment_start = start + segment_index as f32 * segment_size;
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
            let k =
                CubicKernelWrapper::<K>::kernel(normalized_segment_distance as f64) as f32;
            t[(i, parameter_index as usize)] += k * sqrt_weights[i];
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
