//! Bicubic interpolation kernels and utilities.
//!
//! C++ Reference:
//! - Geometry/Bicubic.hpp
//!
//! Provides linear, Catmul-Rom, and B-spline interpolation kernels
//! for 1D and 2D interpolation of sampled data.

/// Trait for interpolation kernel coefficients.
///
/// Each kernel defines a 4x4 coefficient matrix where
/// a{row}{col} returns the coefficient at that position.
/// Rows correspond to segments (0-3), columns to polynomial powers (0-3).
///
/// In C++ this is expressed structurally: each `BicubicInternal::*Kernel<T>`
/// struct exposes 16 static `aRC()` accessors plus a `FloatType` typedef.
/// Geometry/Bicubic.hpp:14  namespace BicubicInternal {
pub trait KernelCoefficients {
    fn a00() -> f64;
    fn a01() -> f64;
    fn a02() -> f64;
    fn a03() -> f64;
    fn a10() -> f64;
    fn a11() -> f64;
    fn a12() -> f64;
    fn a13() -> f64;
    fn a20() -> f64;
    fn a21() -> f64;
    fn a22() -> f64;
    fn a23() -> f64;
    fn a30() -> f64;
    fn a31() -> f64;
    fn a32() -> f64;
    fn a33() -> f64;
}

/// Linear kernel (hat function), for testing cubic methods with linear interpolation.
///
/// Geometry/Bicubic.hpp:15  // Linear kernel, to be able to test cubic methods with hat kernels.
/// Geometry/Bicubic.hpp:16-69  template<typename T> struct LinearKernel
#[derive(Debug, Clone)]
pub struct LinearKernel;

impl KernelCoefficients for LinearKernel {
    fn a00() -> f64 {
        0.0
    }
    fn a01() -> f64 {
        0.0
    }
    fn a02() -> f64 {
        0.0
    }
    fn a03() -> f64 {
        0.0
    }
    fn a10() -> f64 {
        1.0
    }
    fn a11() -> f64 {
        -1.0
    }
    fn a12() -> f64 {
        0.0
    }
    fn a13() -> f64 {
        0.0
    }
    fn a20() -> f64 {
        0.0
    }
    fn a21() -> f64 {
        1.0
    }
    fn a22() -> f64 {
        0.0
    }
    fn a23() -> f64 {
        0.0
    }
    fn a30() -> f64 {
        0.0
    }
    fn a31() -> f64 {
        0.0
    }
    fn a32() -> f64 {
        0.0
    }
    fn a33() -> f64 {
        0.0
    }
}

/// Catmul-Rom interpolation kernel (also known as Keyes kernel).
///
/// Geometry/Bicubic.hpp:71  // Interpolation kernel aka Catmul-Rom aka Keyes kernel.
/// Geometry/Bicubic.hpp:72-125  template<typename T> struct CubicCatmulRomKernel
#[derive(Debug, Clone)]
pub struct CubicCatmulRomKernel;

impl KernelCoefficients for CubicCatmulRomKernel {
    fn a00() -> f64 {
        0.0
    }
    fn a01() -> f64 {
        -0.5
    }
    fn a02() -> f64 {
        1.0
    }
    fn a03() -> f64 {
        -0.5
    }
    fn a10() -> f64 {
        1.0
    }
    fn a11() -> f64 {
        0.0
    }
    fn a12() -> f64 {
        -5.0 / 2.0
    }
    fn a13() -> f64 {
        3.0 / 2.0
    }
    fn a20() -> f64 {
        0.0
    }
    fn a21() -> f64 {
        0.5
    }
    fn a22() -> f64 {
        2.0
    }
    fn a23() -> f64 {
        -3.0 / 2.0
    }
    fn a30() -> f64 {
        0.0
    }
    fn a31() -> f64 {
        0.0
    }
    fn a32() -> f64 {
        -0.5
    }
    fn a33() -> f64 {
        0.5
    }
}

/// Cubic B-spline kernel.
///
/// Geometry/Bicubic.hpp:127  // B-spline kernel
/// Geometry/Bicubic.hpp:128-181  template<typename T> struct CubicBSplineKernel
#[derive(Debug, Clone)]
pub struct CubicBSplineKernel;

impl KernelCoefficients for CubicBSplineKernel {
    fn a00() -> f64 {
        1.0 / 6.0
    }
    fn a01() -> f64 {
        -3.0 / 6.0
    }
    fn a02() -> f64 {
        3.0 / 6.0
    }
    fn a03() -> f64 {
        -1.0 / 6.0
    }
    fn a10() -> f64 {
        4.0 / 6.0
    }
    fn a11() -> f64 {
        0.0
    }
    fn a12() -> f64 {
        -6.0 / 6.0
    }
    fn a13() -> f64 {
        3.0 / 6.0
    }
    fn a20() -> f64 {
        1.0 / 6.0
    }
    fn a21() -> f64 {
        3.0 / 6.0
    }
    fn a22() -> f64 {
        3.0 / 6.0
    }
    fn a23() -> f64 {
        -3.0 / 6.0
    }
    fn a30() -> f64 {
        0.0
    }
    fn a31() -> f64 {
        0.0
    }
    fn a32() -> f64 {
        0.0
    }
    fn a33() -> f64 {
        1.0 / 6.0
    }
}

/// Wrapper around a kernel that provides evaluation and interpolation methods.
///
/// Geometry/Bicubic.hpp:191-192  template<typename Kernel> struct CubicKernelWrapper
#[derive(Debug, Clone)]
pub struct CubicKernelWrapper<K: KernelCoefficients> {
    _phantom: std::marker::PhantomData<K>,
}

impl<K: KernelCoefficients> CubicKernelWrapper<K> {
    /// The span of the kernel (number of samples it touches).
    ///
    /// Geometry/Bicubic.hpp:196  static constexpr size_t kernel_span = 4;
    pub const KERNEL_SPAN: usize = 4;

    /// Evaluate the kernel function at position x.
    ///
    /// Geometry/Bicubic.hpp:198  static FloatType kernel(FloatType x)
    pub fn kernel(x: f64) -> f64 {
        // Geometry/Bicubic.hpp:200  x = fabs(x);
        let x = x.abs();
        // Geometry/Bicubic.hpp:201-202  if (x >= (FloatType) 2.) return 0.0f;
        if x >= 2.0 {
            return 0.0;
        }
        // Geometry/Bicubic.hpp:203  if (x <= (FloatType) 1.) {
        if x <= 1.0 {
            // Geometry/Bicubic.hpp:204  FloatType x2 = x * x;
            let x2 = x * x;
            // Geometry/Bicubic.hpp:205  FloatType x3 = x2 * x;
            let x3 = x2 * x;
            // Geometry/Bicubic.hpp:206  return Kernel::a10() + Kernel::a11() * x + Kernel::a12() * x2 + Kernel::a13() * x3;
            return K::a10() + K::a11() * x + K::a12() * x2 + K::a13() * x3;
        }
        // Geometry/Bicubic.hpp:208  assert(x > (FloatType )1. && x < (FloatType )2.);
        // Geometry/Bicubic.hpp:209  x -= (FloatType) 1.;
        let x = x - 1.0;
        // Geometry/Bicubic.hpp:210  FloatType x2 = x * x;
        let x2 = x * x;
        // Geometry/Bicubic.hpp:211  FloatType x3 = x2 * x;
        let x3 = x2 * x;
        // Geometry/Bicubic.hpp:212  return Kernel::a00() + Kernel::a01() * x + Kernel::a02() * x2 + Kernel::a03() * x3;
        K::a00() + K::a01() * x + K::a02() * x2 + K::a03() * x3
    }

    /// Interpolate between four evenly-spaced sample values at position x (0..1).
    ///
    /// Geometry/Bicubic.hpp:215  static FloatType interpolate(FloatType f0, FloatType f1, FloatType f2, FloatType f3, FloatType x)
    pub fn interpolate(f0: f64, f1: f64, f2: f64, f3: f64, x: f64) -> f64 {
        // Geometry/Bicubic.hpp:217  const FloatType x2 = x * x;
        let x2 = x * x;
        // Geometry/Bicubic.hpp:218  const FloatType x3 = x * x * x;
        let x3 = x * x * x;
        // Geometry/Bicubic.hpp:219-222  return f0 * (...) + f1 * (...) + f2 * (...) + f3 * (...);
        f0 * (K::a00() + K::a01() * x + K::a02() * x2 + K::a03() * x3)
            + f1 * (K::a10() + K::a11() * x + K::a12() * x2 + K::a13() * x3)
            + f2 * (K::a20() + K::a21() * x + K::a22() * x2 + K::a23() * x3)
            + f3 * (K::a30() + K::a31() * x + K::a32() * x2 + K::a33() * x3)
    }
}

// Geometry/Bicubic.hpp:226-236  Kernel wrapper aliases.
// The C++ source declares (in namespace Geometry) the following `using`
// aliases, each wrapping a `BicubicInternal` coefficient struct:
//
//   Geometry/Bicubic.hpp:227-228  using LinearKernel<NumberType>        = CubicKernelWrapper<BicubicInternal::LinearKernel<NumberType>>;
//   Geometry/Bicubic.hpp:231-232  using CubicCatmulRomKernel<NumberType>= CubicKernelWrapper<BicubicInternal::CubicCatmulRomKernel<NumberType>>;
//   Geometry/Bicubic.hpp:235-236  using CubicBSplineKernel<NumberType>  = CubicKernelWrapper<BicubicInternal::CubicBSplineKernel<NumberType>>;
//
// In this port the coefficient structs above are named `LinearKernel`,
// `CubicCatmulRomKernel` and `CubicBSplineKernel` (mirroring the
// `BicubicInternal::*` structs), and the wrapper is `CubicKernelWrapper<K>`.
// Callers therefore spell the C++ alias `Geometry::LinearKernel<f64>` as
// `CubicKernelWrapper::<LinearKernel>`. The aliases are not introduced as
// separate Rust type aliases to avoid name collisions with the coefficient
// structs (Rust has no per-namespace shadowing as C++ does).

/// Clamp a value to the range [lower, upper].
///
/// Geometry/Bicubic.hpp:183  template<class T> inline T clamp(T a, T lower, T upper)
/// Geometry/Bicubic.hpp: BicubicInternal::clamp
pub fn clamp<T: PartialOrd>(a: T, lower: T, upper: T) -> T {
    if a < lower {
        lower
    } else if a > upper {
        upper
    } else {
        a
    }
}

/// Perform 1D cubic interpolation on a data array at a given position.
///
/// The position is in array index space (floating point).
/// Values outside the array are clamped to boundary values.
///
/// The C++ source takes an `Eigen::ArrayBase<...> &F`; here `data` is the
/// equivalent flat array (`F.size()` -> `data.len()`).
///
/// Geometry/Bicubic.hpp:238  template<typename KernelWrapper>
/// Geometry/Bicubic.hpp:239  static typename KernelWrapper::FloatType cubic_interpolate(const Eigen::ArrayBase<...> &F,
/// Geometry/Bicubic.hpp:240          const typename KernelWrapper::FloatType pt) {
pub fn cubic_interpolate<K: KernelCoefficients>(data: &[f64], pt: f64) -> f64 {
    // Geometry/Bicubic.hpp:242  const int w = int(F.size());
    let w = data.len() as i32;
    // Geometry/Bicubic.hpp:243  const int ix = (int) floor(pt);
    let ix = pt.floor() as i32;
    // Geometry/Bicubic.hpp:244  const T s = pt - T( ix);
    let s = pt - ix as f64;

    // Geometry/Bicubic.hpp:246  if (ix > 1 && ix + 2 < w) {
    if ix > 1 && ix + 2 < w {
        // Geometry/Bicubic.hpp:247  // Inside the fully interpolated region.
        // Geometry/Bicubic.hpp:248  return KernelWrapper::interpolate(F[ix - 1], F[ix], F[ix + 1], F[ix + 2], s);
        return CubicKernelWrapper::<K>::interpolate(
            data[ix as usize - 1],
            data[ix as usize],
            data[ix as usize + 1],
            data[ix as usize + 2],
            s,
        );
    }
    // Geometry/Bicubic.hpp:250  // Transition region. Extend with a constant function.
    // Geometry/Bicubic.hpp:251-253  auto f = [&F, w](T x) { return F[clamp(x, 0, w - 1)]; };
    let f = |i: i32| -> f64 { data[clamp(i, 0, w - 1) as usize] };
    // Geometry/Bicubic.hpp:254  return KernelWrapper::interpolate(f(ix - 1), f(ix), f(ix + 1), f(ix + 2), s);
    CubicKernelWrapper::<K>::interpolate(f(ix - 1), f(ix), f(ix + 1), f(ix + 2), s)
}

/// Perform 2D bicubic interpolation on a row-major 2D grid.
///
/// `data` is a row-major grid of size `rows x cols`.
/// `pt` is the position in grid index space (col, row) as (x, y).
///
/// The C++ source operates on an `Eigen::MatrixBase<Derived> &F` accessed as
/// `F(col, row)` with `F.cols()`/`F.rows()`. Here `data` is the equivalent
/// row-major storage with `data[row * cols + col]`.
///
/// Geometry/Bicubic.hpp:257  template<typename Kernel, typename Derived>
/// Geometry/Bicubic.hpp:258  static float bicubic_interpolate(const Eigen::MatrixBase<Derived> &F,
/// Geometry/Bicubic.hpp:259          const Eigen::Matrix<typename Kernel::FloatType, 2, 1, Eigen::DontAlign> &pt) {
pub fn bicubic_interpolate<K: KernelCoefficients>(
    data: &[f64],
    cols: usize,
    rows: usize,
    pt_x: f64,
    pt_y: f64,
) -> f64 {
    // Geometry/Bicubic.hpp:261  const int w = F.cols();
    let w = cols as i32;
    // Geometry/Bicubic.hpp:262  const int h = F.rows();
    let h = rows as i32;
    // Geometry/Bicubic.hpp:263  const int ix = (int) floor(pt[0]);
    let ix = pt_x.floor() as i32;
    // Geometry/Bicubic.hpp:264  const int iy = (int) floor(pt[1]);
    let iy = pt_y.floor() as i32;
    // Geometry/Bicubic.hpp:265  const T s = pt[0] - T( ix);
    let s = pt_x - ix as f64;
    // Geometry/Bicubic.hpp:266  const T t = pt[1] - T( iy);
    let t = pt_y - iy as f64;

    // Geometry/Bicubic.hpp:268  if (ix > 1 && ix + 2 < w && iy > 1 && iy + 2 < h) {
    if ix > 1 && ix + 2 < w && iy > 1 && iy + 2 < h {
        // Geometry/Bicubic.hpp:269  // Inside the fully interpolated region.
        // Direct (unclamped) access, matching the C++ fast path.
        let g = |x: i32, y: i32| -> f64 { data[y as usize * cols + x as usize] };
        // Geometry/Bicubic.hpp:270-274  return Kernel::interpolate( ... , t);
        return CubicKernelWrapper::<K>::interpolate(
            CubicKernelWrapper::<K>::interpolate(
                g(ix - 1, iy - 1),
                g(ix, iy - 1),
                g(ix + 1, iy - 1),
                g(ix + 2, iy - 1),
                s,
            ),
            CubicKernelWrapper::<K>::interpolate(
                g(ix - 1, iy),
                g(ix, iy),
                g(ix + 1, iy),
                g(ix + 2, iy),
                s,
            ),
            CubicKernelWrapper::<K>::interpolate(
                g(ix - 1, iy + 1),
                g(ix, iy + 1),
                g(ix + 1, iy + 1),
                g(ix + 2, iy + 1),
                s,
            ),
            CubicKernelWrapper::<K>::interpolate(
                g(ix - 1, iy + 2),
                g(ix, iy + 2),
                g(ix + 1, iy + 2),
                g(ix + 2, iy + 2),
                s,
            ),
            t,
        );
    }
    // Geometry/Bicubic.hpp:276  // Transition region. Extend with a constant function.
    // Geometry/Bicubic.hpp:277-279  auto f = [&F, w, h](int x, int y) { return F(clamp(x,0,w-1), clamp(y,0,h-1)); };
    let f = |x: i32, y: i32| -> f64 {
        let cx = clamp(x, 0, w - 1) as usize;
        let cy = clamp(y, 0, h - 1) as usize;
        data[cy * cols + cx]
    };
    // Geometry/Bicubic.hpp:280-284  return Kernel::interpolate( ... , t);
    CubicKernelWrapper::<K>::interpolate(
        CubicKernelWrapper::<K>::interpolate(
            f(ix - 1, iy - 1),
            f(ix, iy - 1),
            f(ix + 1, iy - 1),
            f(ix + 2, iy - 1),
            s,
        ),
        CubicKernelWrapper::<K>::interpolate(
            f(ix - 1, iy),
            f(ix, iy),
            f(ix + 1, iy),
            f(ix + 2, iy),
            s,
        ),
        CubicKernelWrapper::<K>::interpolate(
            f(ix - 1, iy + 1),
            f(ix, iy + 1),
            f(ix + 1, iy + 1),
            f(ix + 2, iy + 1),
            s,
        ),
        CubicKernelWrapper::<K>::interpolate(
            f(ix - 1, iy + 2),
            f(ix, iy + 2),
            f(ix + 1, iy + 2),
            f(ix + 2, iy + 2),
            s,
        ),
        t,
    )
}
