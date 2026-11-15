//! What `clipper_core.rs`, `clipper_engine.rs`, `clipper_offset.rs` and `clipper_h.rs` are compiled with as `crate::clipper2`,
//! the C++ `namespace Clipper2Lib`: the sources without `USINGZ`. `build_xyz.rs` defines
//! the same names for the build with the macro; the two files are what the macro stands for.

/// `#ifdef USINGZ`: not defined in this build.
pub const USINGZ: bool = false;

/// clipper.core.h:119-243 `template <typename T> struct Point` with `T = int64_t`
/// (`using Point64 = Point<int64_t>`, clipper.core.h:246), the `#else` arm of `USINGZ`
/// (clipper.core.h:178-214): `x` and `y`.
#[derive(Debug, Clone, Copy, Default)]
pub struct Point64 {
    pub x: i64,
    pub y: i64,
}

impl Point64 {
    /// clipper.core.h:198-199
    /// C++: Point(const T2 x_, const T2 y_) { Init(x_, y_); }
    pub const fn new(x: i64, y: i64) -> Self {
        Self { x, y }
    }
}

/// `pt.z` in an arm under `USINGZ`; this build has no such arm, so nothing that runs reads
/// the value.
#[inline(always)]
pub(crate) fn point_z(_pt: &Point64) -> i64 {
    0
}

/// `pt.z = z` in an arm under `USINGZ`; see `point_z`.
#[inline(always)]
pub(crate) fn point_set_z(_pt: &mut Point64, _z: i64) {}

/// `Point64(x, y, z)` in an arm under `USINGZ`, `Point64(x, y)` in the other arm.
#[inline(always)]
pub(crate) fn point_xyz(x: i64, y: i64, _z: i64) -> Point64 {
    Point64 { x, y }
}

/// clipper.engine.h:167-173 `ZCallback64` and clipper.engine.h:274-277 `zCallback_` exist
/// under `USINGZ` only: this build's `ClipperBase` holds nothing for it.
#[derive(Debug, Clone, Default)]
pub struct ZCallback64;

impl ZCallback64 {
    /// clipper.engine.h:275 `ZCallback64 zCallback_ = nullptr;`
    #[inline(always)]
    pub(crate) fn none() -> Self {
        ZCallback64
    }

    /// `if (zCallback_)`
    #[inline(always)]
    pub(crate) fn is_set(&self) -> bool {
        false
    }

    /// `zCallback_(e1bot, e1top, e2bot, e2top, pt)`
    #[inline(always)]
    pub(crate) fn call(&self, _e1bot: &Point64, _e1top: &Point64, _e2bot: &Point64, _e2top: &Point64, _pt: &mut Point64) {}
}

/// clipper.core.h:119-243 `template <typename T> struct Point` with `T = double`
/// (`using PointD = Point<double>`, clipper.core.h:247), the `#else` arm of `USINGZ`.
#[derive(Debug, Clone, Copy, Default)]
pub struct PointD {
    pub x: f64,
    pub y: f64,
}

/// `PointD(x, y, z)` in an arm under `USINGZ`, `PointD(x, y)` in the other arm.
#[inline(always)]
pub(crate) fn pointd_xyz(x: f64, y: f64, _z: i64) -> PointD {
    PointD { x, y }
}

/// `pt.z` of a `PointD` in an arm under `USINGZ`; see `point_z`.
#[inline(always)]
pub(crate) fn pointd_z(_pt: &PointD) -> i64 {
    0
}

/// clipper.offset.cpp:609-614, under `USINGZ` only:
/// C++: auto fp = std::bind(&ClipperOffset::ZCB, this, _1, _2, _3, _4, _5); c.SetZCallback(fp);
/// This build has no such arm: the clipper of `ClipperOffset::ExecuteInternal` gets no
/// callback.
#[inline(always)]
pub(crate) fn bind_zcb(
    _zcb: fn(&ZCallback64, &Point64, &Point64, &Point64, &Point64, &mut Point64),
    _z_callback64: &ZCallback64,
) -> ZCallback64 {
    ZCallback64
}

/// clipper.engine.h:171-172 `ZCallbackD` and clipper.engine.h:535 `zCallbackD_` exist under
/// `USINGZ` only: this build's `ClipperD` holds nothing for it.
#[derive(Debug, Clone, Default)]
pub struct ZCallbackD;

impl ZCallbackD {
    /// clipper.engine.h:535 `ZCallbackD zCallbackD_ = nullptr;`
    #[inline(always)]
    pub(crate) fn none() -> Self {
        ZCallbackD
    }

    /// `if (zCallbackD_)`
    #[inline(always)]
    pub(crate) fn is_set(&self) -> bool {
        false
    }

    /// `zCallbackD_(e1b, e1t, e2b, e2t, tmp)`
    #[inline(always)]
    pub(crate) fn call(&self, _e1bot: &PointD, _e1top: &PointD, _e2bot: &PointD, _e2top: &PointD, _pt: &mut PointD) {}
}

/// clipper.engine.h:569-580 `ClipperD::CheckCallback`, under `USINGZ` only:
/// C++: ClipperBase::zCallback_ = std::bind(&ClipperD::ZCB, this, _1, _2, _3, _4, _5);
/// This build has no such member function: `ClipperD::Execute` sets no callback.
#[inline(always)]
pub(crate) fn bind_zcb_d(
    _zcb: fn(&ZCallbackD, f64, &Point64, &Point64, &Point64, &Point64, &mut Point64),
    _z_callback_d: &ZCallbackD,
    _inv_scale: f64,
) -> ZCallback64 {
    ZCallback64
}
