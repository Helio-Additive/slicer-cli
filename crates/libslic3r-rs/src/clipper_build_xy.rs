//! What `clipper.rs` is compiled with as `crate::clipper`, the C++ `namespace ClipperLib`:
//! clipper.cpp without `CLIPPERLIB_USE_XYZ`. The engine compiles clipper.cpp twice, once
//! without the macro and once with it (`clipper_z.cpp`); `clipper.rs` is compiled twice the
//! same way, and this file and `clipper_build_xyz.rs` are what the macro stands for. Each
//! defines the same names; `clipper.rs` takes them from `super::clipperlib_build`.

#![allow(non_camel_case_types)]

/// clipper.hpp:88 `typedef int32_t cInt;`
pub type cInt = i32;

/// clipper.hpp:44-45 `CLIPPERLIB_USE_XYZ`: not defined in this build.
pub const CLIPPERLIB_USE_XYZ: bool = false;

/// clipper.hpp:99-107 `IntPoint`: `Eigen::Matrix<cInt, 2, 1, Eigen::DontAlign>` without
/// `CLIPPERLIB_USE_XYZ`. `==` compares x and y.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Hash)]
pub struct IntPoint {
    pub x: cInt,
    pub y: cInt,
}

impl IntPoint {
    /// clipper.cpp:108-115
    /// C++: inline IntPoint IntPoint2d(cInt x, cInt y)
    pub fn new(x: cInt, y: cInt) -> Self {
        Self { x, y }
    }
}

/// `pt.z()` in an arm under `CLIPPERLIB_USE_XYZ`; this build has no such arm, so nothing
/// that runs calls it.
#[inline(always)]
pub(crate) fn point_z(_pt: &IntPoint) -> cInt {
    0
}

/// `pt.z() = z` in an arm under `CLIPPERLIB_USE_XYZ`; see `point_z`.
#[inline(always)]
pub(crate) fn point_set_z(_pt: &mut IntPoint, _z: cInt) {}

/// clipper.hpp:448-450 `ZFillCallback m_ZFill;` is a member under `CLIPPERLIB_USE_XYZ`
/// only: this build's `Clipper` holds nothing for it.
#[derive(Debug, Clone, Default)]
pub struct ZFillCallback;

impl ZFillCallback {
    /// clipper.cpp:1061 `m_ZFill = 0;`
    #[inline(always)]
    pub(crate) fn none() -> Self {
        ZFillCallback
    }

    /// `!m_ZFill` is false for a callback that is set.
    #[inline(always)]
    pub(crate) fn is_set(&self) -> bool {
        false
    }

    /// `m_ZFill(e1bot, e1top, e2bot, e2top, pt)`
    #[inline(always)]
    pub(crate) fn call(&self, _e1bot: &IntPoint, _e1top: &IntPoint, _e2bot: &IntPoint, _e2top: &IntPoint, _pt: &mut IntPoint) {}
}
