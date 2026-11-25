//! Port of `SLA/bicubic.h` (header only).
//!
//! C++ reference: `references/BambuStudio/src/libslic3r/SLA/bicubic.h`.
//!
//! The SLA copy is an older twin of `Geometry/Bicubic.hpp` (ported in [`crate::geometry::bicubic`]):
//! the kernel wrapper is called `CubicKernel` and the interpolation functions take an unused `dx`.
//! At the pin nothing includes it, and it does not compile as it stands: bicubic.h:96
//! (`typedef typename KERNEL KernelInternal;`), :152 (a lambda parameter without a type) and :176
//! (a lambda that captures itself and not `F`). The kernels (bicubic.h:12-91) and `CubicKernel`'s
//! `kernel` / `interpolate` (bicubic.h:93-129) are ported as written; `cubic_interpolate` /
//! `bicubic_interpolate` (bicubic.h:142-185) follow the evident meaning of the broken lambdas (the
//! sample at the index clamped to the grid), with `int` indices and the result returned as `float`.
//!
//! Floating point: the kernels' accessors are inline constants, so clang (`-ffp-contract=on`) fuses
//! each `a0 + a1 * x + a2 * x2 + a3 * x3` into a chain of fused multiply-adds, and the four-term sum
//! of `interpolate` as `f0 * p0 + f1 * p1` (the left product fused, the right one rounded) followed by
//! two more fused terms.

use std::marker::PhantomData;

/// `typename KERNEL::FloatType`: the kernels are instantiated with `float` and `double`
/// (bicubic.h:131-140).
pub trait BicubicFloat:
    Copy + PartialOrd + std::ops::Add<Output = Self> + std::ops::Sub<Output = Self> + std::ops::Mul<Output = Self>
{
    /// `(T) v` of a double constant.
    fn from_f64(v: f64) -> Self;
    /// `(T) i` of an int.
    fn from_i32(v: i32) -> Self;
    /// `fabs(x)`.
    fn fabs(self) -> Self;
    /// `floor(x)`.
    fn floor_(self) -> Self;
    /// `(int) x` (truncation).
    fn to_i32(self) -> i32;
    /// `(float) x`.
    fn to_f32(self) -> f32;
    /// `a * b + c`, contracted by clang into one fused multiply-add.
    fn fma(a: Self, b: Self, c: Self) -> Self;
}

impl BicubicFloat for f32 {
    fn from_f64(v: f64) -> Self {
        v as f32
    }
    fn from_i32(v: i32) -> Self {
        v as f32
    }
    fn fabs(self) -> Self {
        // `fabs(float)` resolves to the float overload (<cmath>).
        self.abs()
    }
    fn floor_(self) -> Self {
        self.floor()
    }
    fn to_i32(self) -> i32 {
        self as i32
    }
    fn to_f32(self) -> f32 {
        self
    }
    fn fma(a: Self, b: Self, c: Self) -> Self {
        a.mul_add(b, c)
    }
}

impl BicubicFloat for f64 {
    fn from_f64(v: f64) -> Self {
        v
    }
    fn from_i32(v: i32) -> Self {
        v as f64
    }
    fn fabs(self) -> Self {
        self.abs()
    }
    fn floor_(self) -> Self {
        self.floor()
    }
    fn to_i32(self) -> i32 {
        self as i32
    }
    fn to_f32(self) -> f32 {
        self as f32
    }
    fn fma(a: Self, b: Self, c: Self) -> Self {
        a.mul_add(b, c)
    }
}

/// The sixteen `aRC()` accessors a kernel struct offers to `CubicKernel<KERNEL>` (bicubic.h:12-91;
/// in C++ a structural requirement of the template). Implemented by delegating to the kernels'
/// inherent functions of the C++ names.
pub trait KernelCoeffs {
    /// `typedef T FloatType;`
    type FloatType: BicubicFloat;
    fn k00() -> Self::FloatType;
    fn k01() -> Self::FloatType;
    fn k02() -> Self::FloatType;
    fn k03() -> Self::FloatType;
    fn k10() -> Self::FloatType;
    fn k11() -> Self::FloatType;
    fn k12() -> Self::FloatType;
    fn k13() -> Self::FloatType;
    fn k20() -> Self::FloatType;
    fn k21() -> Self::FloatType;
    fn k22() -> Self::FloatType;
    fn k23() -> Self::FloatType;
    fn k30() -> Self::FloatType;
    fn k31() -> Self::FloatType;
    fn k32() -> Self::FloatType;
    fn k33() -> Self::FloatType;
}

/// bicubic.h:12 `namespace BicubicInternal`.
pub mod bicubic_internal {
    use super::{BicubicFloat, KernelCoeffs};
    use std::marker::PhantomData;

    macro_rules! kernel_struct {
        ($(#[$m:meta])* $name:ident, [$a00:expr, $a01:expr, $a02:expr, $a03:expr, $a10:expr, $a11:expr, $a12:expr, $a13:expr,
          $a20:expr, $a21:expr, $a22:expr, $a23:expr, $a30:expr, $a31:expr, $a32:expr, $a33:expr]) => {
            $(#[$m])*
            #[derive(Debug, Clone, Copy, Default)]
            pub struct $name<T>(PhantomData<T>);

            impl<T: BicubicFloat> $name<T> {
                pub fn a00() -> T { T::from_f64($a00) }
                pub fn a01() -> T { T::from_f64($a01) }
                pub fn a02() -> T { T::from_f64($a02) }
                pub fn a03() -> T { T::from_f64($a03) }
                pub fn a10() -> T { T::from_f64($a10) }
                pub fn a11() -> T { T::from_f64($a11) }
                pub fn a12() -> T { T::from_f64($a12) }
                pub fn a13() -> T { T::from_f64($a13) }
                pub fn a20() -> T { T::from_f64($a20) }
                pub fn a21() -> T { T::from_f64($a21) }
                pub fn a22() -> T { T::from_f64($a22) }
                pub fn a23() -> T { T::from_f64($a23) }
                pub fn a30() -> T { T::from_f64($a30) }
                pub fn a31() -> T { T::from_f64($a31) }
                pub fn a32() -> T { T::from_f64($a32) }
                pub fn a33() -> T { T::from_f64($a33) }
            }

            impl<T: BicubicFloat> KernelCoeffs for $name<T> {
                type FloatType = T;
                fn k00() -> T { Self::a00() }
                fn k01() -> T { Self::a01() }
                fn k02() -> T { Self::a02() }
                fn k03() -> T { Self::a03() }
                fn k10() -> T { Self::a10() }
                fn k11() -> T { Self::a11() }
                fn k12() -> T { Self::a12() }
                fn k13() -> T { Self::a13() }
                fn k20() -> T { Self::a20() }
                fn k21() -> T { Self::a21() }
                fn k22() -> T { Self::a22() }
                fn k23() -> T { Self::a23() }
                fn k30() -> T { Self::a30() }
                fn k31() -> T { Self::a31() }
                fn k32() -> T { Self::a32() }
                fn k33() -> T { Self::a33() }
            }
        };
    }

    // bicubic.h:13-35 `template<typename T> struct LinearKernel`: `T(0.)`, `T(1.)`, `T(-1.)`.
    kernel_struct!(
        /// bicubic.h:13-35: linear kernel, to test the cubic methods with hat kernels.
        LinearKernel,
        [0., 0., 0., 0., 1., -1., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0.]
    );

    // bicubic.h:37-60 `template<typename T> struct CubicCatmulRomKernel`: `(T)-5./2.` is
    // `((T)-5.) / 2.`, a double division of an exact value, then converted to T on return.
    kernel_struct!(
        /// bicubic.h:37-60: the interpolation kernel aka Catmul-Rom aka Keyes kernel.
        CubicCatmulRomKernel,
        [0., -0.5, 1., -0.5, 1., 0., -5. / 2., 3. / 2., 0., 0.5, 2., -3. / 2., 0., 0., -0.5, 0.5]
    );

    // bicubic.h:62-85 `template<typename T> struct CubicBSplineKernel`: `(T) 1./6.` is the double
    // `1. / 6.` converted to T on return.
    kernel_struct!(
        /// bicubic.h:62-85: the B-spline kernel.
        CubicBSplineKernel,
        [1. / 6., -3. / 6., 3. / 6., -1. / 6., 4. / 6., 0., -6. / 6., 3. / 6., 1. / 6., 3. / 6., 3. / 6., -3. / 6., 0., 0., 0., 1. / 6.]
    );

    /// bicubic.h:87-92 `template<class T> inline T clamp(T a, T lower, T upper)`
    pub fn clamp<T: PartialOrd>(a: T, lower: T, upper: T) -> T {
        if a < lower {
            lower
        } else if a > upper {
            upper
        } else {
            a
        }
    }
}

/// bicubic.h:94-129 `template<typename KERNEL> struct CubicKernel`.
#[derive(Debug, Clone, Copy, Default)]
pub struct CubicKernel<K: KernelCoeffs>(PhantomData<K>);

impl<K: KernelCoeffs> CubicKernel<K> {
    /// bicubic.h:99-114 `static FloatType kernel(FloatType x)`
    pub fn kernel(x: K::FloatType) -> K::FloatType {
        type F<K> = <K as KernelCoeffs>::FloatType;
        // bicubic.h:101 `x = fabs(x);`
        let mut x = x.fabs();
        // bicubic.h:102-103
        if x >= F::<K>::from_f64(2.) {
            return F::<K>::from_f64(0.0);
        }
        // bicubic.h:104-108
        if x <= F::<K>::from_f64(1.) {
            let x2 = x * x;
            let x3 = x2 * x;
            // `a10 + a11 * x + a12 * x2 + a13 * x3`: three fused multiply-adds.
            return F::<K>::fma(K::k13(), x3, F::<K>::fma(K::k12(), x2, F::<K>::fma(K::k11(), x, K::k10())));
        }
        // bicubic.h:109-113
        x = x - F::<K>::from_f64(1.);
        let x2 = x * x;
        let x3 = x2 * x;
        F::<K>::fma(K::k03(), x3, F::<K>::fma(K::k02(), x2, F::<K>::fma(K::k01(), x, K::k00())))
    }

    /// bicubic.h:116-124 `static FloatType interpolate(FloatType f0, FloatType f1, FloatType f2, FloatType f3, FloatType x)`
    pub fn interpolate(
        f0: K::FloatType,
        f1: K::FloatType,
        f2: K::FloatType,
        f3: K::FloatType,
        x: K::FloatType,
    ) -> K::FloatType {
        type F<K> = <K as KernelCoeffs>::FloatType;
        // bicubic.h:118-119
        let x2 = x * x;
        let x3 = x * x * x;
        let p0 = F::<K>::fma(K::k03(), x3, F::<K>::fma(K::k02(), x2, F::<K>::fma(K::k01(), x, K::k00())));
        let p1 = F::<K>::fma(K::k13(), x3, F::<K>::fma(K::k12(), x2, F::<K>::fma(K::k11(), x, K::k10())));
        let p2 = F::<K>::fma(K::k23(), x3, F::<K>::fma(K::k22(), x2, F::<K>::fma(K::k21(), x, K::k20())));
        let p3 = F::<K>::fma(K::k33(), x3, F::<K>::fma(K::k32(), x2, F::<K>::fma(K::k31(), x, K::k30())));
        // bicubic.h:120-123 `f0*(p0) + f1*(p1) + f2*(p2) + f3*(p3)`
        let s = F::<K>::fma(f0, p0, f1 * p1);
        let s = F::<K>::fma(f2, p2, s);
        F::<K>::fma(f3, p3, s)
    }
}

/// bicubic.h:131-132 `LinearKernelf` / `LinearKerneld`.
pub type LinearKernelf = CubicKernel<bicubic_internal::LinearKernel<f32>>;
pub type LinearKerneld = CubicKernel<bicubic_internal::LinearKernel<f64>>;
/// bicubic.h:134-137 Catmul-Rom splines (also called the interpolation kernel).
pub type CubicCatmulRomKernelf = CubicKernel<bicubic_internal::CubicCatmulRomKernel<f32>>;
pub type CubicCatmulRomKerneld = CubicKernel<bicubic_internal::CubicCatmulRomKernel<f64>>;
pub type CubicInterpolationKernelf = CubicKernel<bicubic_internal::CubicCatmulRomKernel<f32>>;
pub type CubicInterpolationKerneld = CubicKernel<bicubic_internal::CubicCatmulRomKernel<f64>>;
/// bicubic.h:139-140 cubic B-splines.
pub type CubicBSplineKernelf = CubicKernel<bicubic_internal::CubicBSplineKernel<f32>>;
pub type CubicBSplineKerneld = CubicKernel<bicubic_internal::CubicBSplineKernel<f64>>;

// Keep the existing Geometry interpolation API available to callers of this module.
pub use crate::geometry::bicubic::{
    bicubic_interpolate, clamp, cubic_interpolate, CubicBSplineKernel, CubicCatmulRomKernel,
    CubicKernelWrapper, KernelCoefficients, LinearKernel,
};

/// SLA/bicubic.h grid interpolation overloads, including its extra spacing argument.
pub mod grid {
    use super::{bicubic_internal, BicubicFloat, CubicKernel, KernelCoeffs};

    /// bicubic.h:142-157 `template<typename KERNEL, typename Derived> static float cubic_interpolate(const
    /// Eigen::ArrayBase<Derived> &F, const typename KERNEL::FloatType pt, const typename KERNEL::FloatType
    /// dx)`; `dx` is unused. The transition-region lambda (bicubic.h:152, not valid C++) reads the
    /// sample at the index clamped to `[0, w - 1]`.
    pub fn cubic_interpolate<K: KernelCoeffs>(f: &[K::FloatType], pt: K::FloatType, _dx: K::FloatType) -> f32 {
        type T<K> = <K as KernelCoeffs>::FloatType;
        // bicubic.h:146-148
        let w = f.len() as i32;
        let ix = pt.floor_().to_i32();
        let s = pt - T::<K>::from_i32(ix);

        if ix > 1 && ix + 2 < w {
            // bicubic.h:150-153 Inside the fully interpolated region.
            let ix = ix as usize;
            return CubicKernel::<K>::interpolate(f[ix - 1], f[ix], f[ix + 1], f[ix + 2], s).to_f32();
        }
        // bicubic.h:155-156 Transition region. Extend with a constant function.
        let at = |x: i32| f[bicubic_internal::clamp(x, 0, w - 1) as usize];
        CubicKernel::<K>::interpolate(at(ix - 1), at(ix), at(ix + 1), at(ix + 2), s).to_f32()
    }

    /// bicubic.h:159-183 `template<typename KERNEL, typename Derived> static float bicubic_interpolate(const
    /// Eigen::MatrixBase<Derived> &F, const Eigen::Matrix<FloatType, 2, 1, Eigen::DontAlign> &pt, const
    /// FloatType dx)`; `dx` is unused. `F` is a column-major matrix of `rows` x `cols` samples
    /// (`F(r, c)` is `f[c * rows + r]`), indexed as the C++ does: `F(ix, iy)` with `w = F.cols()` and
    /// `h = F.rows()`. The transition-region lambda (bicubic.h:176, not valid C++) reads the sample at
    /// the indices clamped to `[0, w - 1]` and `[0, h - 1]`.
    pub fn bicubic_interpolate<K: KernelCoeffs>(
        f: &[K::FloatType],
        rows: usize,
        cols: usize,
        pt: [K::FloatType; 2],
        _dx: K::FloatType,
    ) -> f32 {
        type T<K> = <K as KernelCoeffs>::FloatType;
        // bicubic.h:163-168
        let w = cols as i32;
        let h = rows as i32;
        let ix = pt[0].floor_().to_i32();
        let iy = pt[1].floor_().to_i32();
        let s = pt[0] - T::<K>::from_i32(ix);
        let t = pt[1] - T::<K>::from_i32(iy);
        let at = |r: i32, c: i32| f[c as usize * rows + r as usize];
        let interp = CubicKernel::<K>::interpolate;

        if ix > 1 && ix + 2 < w && iy > 1 && iy + 2 < h {
            // bicubic.h:170-177 Inside the fully interpolated region.
            return interp(
                interp(at(ix - 1, iy - 1), at(ix, iy - 1), at(ix + 1, iy - 1), at(ix + 2, iy - 1), s),
                interp(at(ix - 1, iy), at(ix, iy), at(ix + 1, iy), at(ix + 2, iy), s),
                interp(at(ix - 1, iy + 1), at(ix, iy + 1), at(ix + 1, iy + 1), at(ix + 2, iy + 1), s),
                interp(at(ix - 1, iy + 2), at(ix, iy + 2), at(ix + 1, iy + 2), at(ix + 2, iy + 2), s),
                t,
            )
            .to_f32();
        }
        // bicubic.h:178-183 Transition region. Extend with a constant function.
        let fc = |x: i32, y: i32| at(bicubic_internal::clamp(x, 0, w - 1), bicubic_internal::clamp(y, 0, h - 1));
        interp(
            interp(fc(ix - 1, iy - 1), fc(ix, iy - 1), fc(ix + 1, iy - 1), fc(ix + 2, iy - 1), s),
            interp(fc(ix - 1, iy), fc(ix, iy), fc(ix + 1, iy), fc(ix + 2, iy), s),
            interp(fc(ix - 1, iy + 1), fc(ix, iy + 1), fc(ix + 1, iy + 1), fc(ix + 2, iy + 1), s),
            interp(fc(ix - 1, iy + 2), fc(ix, iy + 2), fc(ix + 1, iy + 2), fc(ix + 2, iy + 2), s),
            t,
        )
        .to_f32()
    }
}
