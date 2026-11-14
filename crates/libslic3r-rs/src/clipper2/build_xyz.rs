//! What `clipper_core.rs`, `clipper_engine.rs`, `clipper_offset.rs` and `clipper_h.rs` are compiled with as
//! `crate::clipper2::clipper2_z`, the C++ `namespace Clipper2Lib_Z`: the sources with
//! `USINGZ` (clipper2_z.cpp:1-8). See `build_xy.rs`, which defines the same names for the
//! build without the macro.

/// `#ifdef USINGZ`: defined in this build (clipper2_z.cpp:3).
pub const USINGZ: bool = true;

/// clipper.core.h:119-243 `template <typename T> struct Point` with `T = int64_t`
/// (`using Point64 = Point<int64_t>`, clipper.core.h:246), the `USINGZ` arm
/// (clipper.core.h:123-176): `x`, `y` and `z_type z` (`using z_type = int64_t`,
/// clipper.core.h:113-115). `==` compares x and y only (clipper.core.h:216-224), so it is
/// written out below instead of derived.
#[derive(Debug, Clone, Copy, Default)]
pub struct Point64 {
    pub x: i64,
    pub y: i64,
    pub z: i64,
}

impl Point64 {
    /// clipper.core.h:146-151
    /// C++: Point(const T2 x_, const T2 y_, const z_type z_ = 0)
    /// The point with z = 0.
    pub const fn new(x: i64, y: i64) -> Self {
        Self { x, y, z: 0 }
    }

    /// clipper.core.h:146-151 with `z_` given.
    pub const fn new_xyz(x: i64, y: i64, z: i64) -> Self {
        Self { x, y, z }
    }

    /// clipper.core.h:170
    /// C++: void SetZ(const z_type z_value) { z = z_value; }
    pub fn set_z(&mut self, z_value: i64) {
        self.z = z_value;
    }
}

/// `pt.z`
#[inline(always)]
pub(crate) fn point_z(pt: &Point64) -> i64 {
    pt.z
}

/// `pt.z = z`
#[inline(always)]
pub(crate) fn point_set_z(pt: &mut Point64, z: i64) {
    pt.z = z;
}

/// `Point64(x, y, z)`
#[inline(always)]
pub(crate) fn point_xyz(x: i64, y: i64, z: i64) -> Point64 {
    Point64 { x, y, z }
}

/// clipper.engine.h:168-169
/// C++: typedef std::function<void(const Point64& e1bot, const Point64& e1top, const Point64& e2bot, const Point64& e2top, Point64& pt)> ZCallback64;
/// A `std::function` is copied by value and may be empty; here it is a shared pointer to
/// the function, or `None`.
#[derive(Clone, Default)]
pub struct ZCallback64(pub Option<std::rc::Rc<dyn Fn(&Point64, &Point64, &Point64, &Point64, &mut Point64)>>);

impl std::fmt::Debug for ZCallback64 {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(if self.0.is_some() { "ZCallback64(set)" } else { "ZCallback64(empty)" })
    }
}

impl ZCallback64 {
    /// A callback that holds `f`.
    pub fn new(f: impl Fn(&Point64, &Point64, &Point64, &Point64, &mut Point64) + 'static) -> Self {
        ZCallback64(Some(std::rc::Rc::new(f)))
    }

    /// clipper.engine.h:275 `ZCallback64 zCallback_ = nullptr;`
    #[inline(always)]
    pub(crate) fn none() -> Self {
        ZCallback64(None)
    }

    /// `if (zCallback_)`
    #[inline(always)]
    pub(crate) fn is_set(&self) -> bool {
        self.0.is_some()
    }

    /// `zCallback_(e1bot, e1top, e2bot, e2top, pt)`
    #[inline(always)]
    pub(crate) fn call(&self, e1bot: &Point64, e1top: &Point64, e2bot: &Point64, e2top: &Point64, pt: &mut Point64) {
        (self.0.as_ref().expect("the callback is called only when it is set"))(e1bot, e1top, e2bot, e2top, pt)
    }
}

/// clipper.core.h:119-243 `template <typename T> struct Point` with `T = double`
/// (`using PointD = Point<double>`, clipper.core.h:247), the `USINGZ` arm: `z_type z` is
/// `int64_t` for a `PointD` as well (clipper.core.h:113-115, 126).
#[derive(Debug, Clone, Copy, Default)]
pub struct PointD {
    pub x: f64,
    pub y: f64,
    pub z: i64,
}

/// `PointD(x, y, z)`
#[inline(always)]
pub(crate) fn pointd_xyz(x: f64, y: f64, z: i64) -> PointD {
    PointD { x, y, z }
}

/// `pt.z` of a `PointD`
#[inline(always)]
pub(crate) fn pointd_z(pt: &PointD) -> i64 {
    pt.z
}

/// clipper.offset.cpp:609-614
/// C++: auto fp = std::bind(&ClipperOffset::ZCB, this, _1, _2, _3, _4, _5); c.SetZCallback(fp);
/// `ZCB` reads one member of `this`, `zCallback64_`: the callback made here holds a copy of
/// it (a `std::function` copy shares nothing the offsetter changes while the clipper runs).
#[inline(always)]
pub(crate) fn bind_zcb(
    zcb: fn(&ZCallback64, &Point64, &Point64, &Point64, &Point64, &mut Point64),
    z_callback64: &ZCallback64,
) -> ZCallback64 {
    let user = z_callback64.clone();
    ZCallback64::new(move |bot1, top1, bot2, top2, ip| zcb(&user, bot1, top1, bot2, top2, ip))
}
