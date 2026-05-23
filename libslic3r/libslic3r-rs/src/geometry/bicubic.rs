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
/// Geometry/Bicubic.hpp: BicubicInternal::LinearKernel
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
/// Geometry/Bicubic.hpp: BicubicInternal::CubicCatmulRomKernel
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
/// Geometry/Bicubic.hpp: BicubicInternal::CubicBSplineKernel
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
/// Geometry/Bicubic.hpp: CubicKernelWrapper
#[derive(Debug, Clone)]
pub struct CubicKernelWrapper<K: KernelCoefficients> {
    _phantom: std::marker::PhantomData<K>,
}

impl<K: KernelCoefficients> CubicKernelWrapper<K> {
    /// The span of the kernel (number of samples it touches).
    pub const KERNEL_SPAN: usize = 4;

    /// Evaluate the kernel function at position x.
    ///
    /// Geometry/Bicubic.hpp: CubicKernelWrapper::kernel
    pub fn kernel(x: f64) -> f64 {
        let x = x.abs();
        if x >= 2.0 {
            return 0.0;
        }
        if x <= 1.0 {
            let x2 = x * x;
            let x3 = x2 * x;
            return K::a10() + K::a11() * x + K::a12() * x2 + K::a13() * x3;
        }
        // 1 < x < 2
        let x = x - 1.0;
        let x2 = x * x;
        let x3 = x2 * x;
        K::a00() + K::a01() * x + K::a02() * x2 + K::a03() * x3
    }

    /// Interpolate between four evenly-spaced sample values at position x (0..1).
    ///
    /// Geometry/Bicubic.hpp: CubicKernelWrapper::interpolate
    pub fn interpolate(f0: f64, f1: f64, f2: f64, f3: f64, x: f64) -> f64 {
        let x2 = x * x;
        let x3 = x * x * x;
        f0 * (K::a00() + K::a01() * x + K::a02() * x2 + K::a03() * x3)
            + f1 * (K::a10() + K::a11() * x + K::a12() * x2 + K::a13() * x3)
            + f2 * (K::a20() + K::a21() * x + K::a22() * x2 + K::a23() * x3)
            + f3 * (K::a30() + K::a31() * x + K::a32() * x2 + K::a33() * x3)
    }
}

/// Clamp a value to the range [lower, upper].
///
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
/// Geometry/Bicubic.hpp: cubic_interpolate
pub fn cubic_interpolate<K: KernelCoefficients>(data: &[f64], pt: f64) -> f64 {
    let w = data.len() as i32;
    let ix = pt.floor() as i32;
    let s = pt - ix as f64;

    let f = |i: i32| -> f64 { data[clamp(i, 0, w - 1) as usize] };

    if ix > 1 && ix + 2 < w {
        // Inside the fully interpolated region
        return CubicKernelWrapper::<K>::interpolate(
            data[ix as usize - 1],
            data[ix as usize],
            data[ix as usize + 1],
            data[ix as usize + 2],
            s,
        );
    }
    // Transition region: extend with constant function (clamped boundary)
    CubicKernelWrapper::<K>::interpolate(f(ix - 1), f(ix), f(ix + 1), f(ix + 2), s)
}

/// Perform 2D bicubic interpolation on a row-major 2D grid.
///
/// `data` is a row-major grid of size `rows x cols`.
/// `pt` is the position in grid index space (col, row) as (x, y).
///
/// Geometry/Bicubic.hpp: bicubic_interpolate
pub fn bicubic_interpolate<K: KernelCoefficients>(
    data: &[f64],
    cols: usize,
    rows: usize,
    pt_x: f64,
    pt_y: f64,
) -> f64 {
    let w = cols as i32;
    let h = rows as i32;
    let ix = pt_x.floor() as i32;
    let iy = pt_y.floor() as i32;
    let s = pt_x - ix as f64;
    let t = pt_y - iy as f64;

    let f = |x: i32, y: i32| -> f64 {
        let cx = clamp(x, 0, w - 1) as usize;
        let cy = clamp(y, 0, h - 1) as usize;
        data[cy * cols + cx]
    };

    let row = |dy: i32| -> f64 {
        CubicKernelWrapper::<K>::interpolate(
            f(ix - 1, iy + dy),
            f(ix, iy + dy),
            f(ix + 1, iy + dy),
            f(ix + 2, iy + dy),
            s,
        )
    };

    CubicKernelWrapper::<K>::interpolate(row(-1), row(0), row(1), row(2), t)
}
