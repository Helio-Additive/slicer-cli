//! What `clipper.rs` is compiled with as `crate::clipper_lib_z`, the C++
//! `namespace ClipperLib_Z`: clipper.cpp with `CLIPPERLIB_USE_XYZ` (clipper_z.cpp:3-7). See
//! `clipper_build_xy.rs`, which defines the same names for the build without the macro.

#![allow(non_camel_case_types)]

/// clipper.hpp:88 `typedef int32_t cInt;`
pub type cInt = i32;

/// clipper.hpp:44-45 `CLIPPERLIB_USE_XYZ`: defined in this build (clipper_z.hpp:10-11).
pub const CLIPPERLIB_USE_XYZ: bool = true;

/// clipper.hpp:99-107 `IntPoint`: `Eigen::Matrix<cInt, 3, 1, Eigen::DontAlign>` under
/// `CLIPPERLIB_USE_XYZ`. `==` and `!=` are Eigen's and compare all three coefficients, so
/// two points at one place with different Z are not equal, in every `==` of clipper.cpp.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Hash)]
pub struct IntPoint {
    pub x: cInt,
    pub y: cInt,
    pub z: cInt,
}

impl IntPoint {
    /// clipper.cpp:108-115
    /// C++: inline IntPoint IntPoint2d(cInt x, cInt y)
    /// The point with z = 0.
    pub fn new(x: cInt, y: cInt) -> Self {
        Self { x, y, z: 0 }
    }

    /// `IntPoint(x, y, z)`, the constructor of the Eigen vector of three.
    pub fn new_xyz(x: cInt, y: cInt, z: cInt) -> Self {
        Self { x, y, z }
    }
}

/// `pt.z()`
#[inline(always)]
pub(crate) fn point_z(pt: &IntPoint) -> cInt {
    pt.z
}

/// `pt.z() = z`
#[inline(always)]
pub(crate) fn point_set_z(pt: &mut IntPoint, z: cInt) {
    pt.z = z;
}

/// clipper.hpp:125-127
/// C++: typedef std::function<void(const IntPoint& e1bot, const IntPoint& e1top, const IntPoint& e2bot, const IntPoint& e2top, IntPoint& pt)> ZFillCallback;
/// A `std::function` is copied by value and may be empty; here it is a shared pointer
/// to the function, or `None`.
#[derive(Clone, Default)]
pub struct ZFillCallback(pub Option<std::rc::Rc<dyn Fn(&IntPoint, &IntPoint, &IntPoint, &IntPoint, &mut IntPoint)>>);

impl std::fmt::Debug for ZFillCallback {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(if self.0.is_some() { "ZFillCallback(set)" } else { "ZFillCallback(empty)" })
    }
}

impl ZFillCallback {
    /// A callback that holds `f`.
    pub fn new(f: impl Fn(&IntPoint, &IntPoint, &IntPoint, &IntPoint, &mut IntPoint) + 'static) -> Self {
        ZFillCallback(Some(std::rc::Rc::new(f)))
    }

    /// clipper.cpp:1061 `m_ZFill = 0;`
    #[inline(always)]
    pub(crate) fn none() -> Self {
        ZFillCallback(None)
    }

    /// `!m_ZFill` is false for a callback that is set.
    #[inline(always)]
    pub(crate) fn is_set(&self) -> bool {
        self.0.is_some()
    }

    /// `m_ZFill(e1bot, e1top, e2bot, e2top, pt)`
    #[inline(always)]
    pub(crate) fn call(&self, e1bot: &IntPoint, e1top: &IntPoint, e2bot: &IntPoint, e2top: &IntPoint, pt: &mut IntPoint) {
        (self.0.as_ref().expect("SetZ calls m_ZFill only when it is set"))(e1bot, e1top, e2bot, e2top, pt)
    }
}
