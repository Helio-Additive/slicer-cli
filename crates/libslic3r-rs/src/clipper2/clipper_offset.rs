//! `include/clipper2/clipper.offset.h` and `src/clipper.offset.cpp` of Clipper2 as the
//! engine pins it (version 1.5.2, `src/clipper2/Clipper2Lib`): Path Offset (Inflate/Shrink).
//! Compiled twice, see `mod.rs`; `super::clipper2_build` is the build file that stands for
//! `USINGZ`.
//!
//! Floating point follows the reference build (Apple clang, arm64, `-ffp-contract=on`): a
//! product that feeds a sum or a difference inside one expression is fused, and is a
//! `mul_add` here; a product passed as an argument, kept in a variable or returned by a
//! function is rounded first. `std::sin(a)` and `std::cos(a)` of one argument are one call
//! of `__sincos_stret` in an optimized build of the C++ (`crate::clipper::sin_cos`).

use super::clipper2_build::{bind_zcb, point_set_z, point_xyz, point_z, pointd_xyz};
use super::clipper_core::*;
use super::clipper_engine::*;
use super::clipper_h::ellipse;
use std::rc::Rc;

/// clipper.offset.cpp:19
const FLOATING_POINT_TOLERANCE: f64 = 1e-12;

// Clipper2 approximates arcs by using series of relatively short straight
//line segments. And logically, shorter line segments will produce better arc
// approximations. But very short segments can degrade performance, usually
// with little or no discernable improvement in curve quality. Very short
// segments can even detract from curve quality, due to the effects of integer
// rounding. Since there isn't an optimal number of line segments for any given
// arc radius (that perfectly balances curve approximation with performance),
// arc tolerance is user defined. Nevertheless, when the user doesn't define
// an arc tolerance (ie leaves alone the 0 default value), the calculated
// default arc tolerance (offset_radius / 500) generally produces good (smooth)
// arc approximations without producing excessively small segment lengths.
// See also: https://www.angusj.com/clipper2/Docs/Trigonometry.htm
/// clipper.offset.cpp:33
const ARC_CONST: f64 = 0.002; // <-- 1/500

/// clipper.offset.h:23
/// C++: enum class JoinType { Square, Bevel, Round, Miter };
//Square : Joins are 'squared' at exactly the offset distance (more complex code)
//Bevel  : Similar to Square, but the offset distance varies with angle (simple code & faster)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum JoinType {
    Square,
    Bevel,
    Round,
    Miter,
}

/// clipper.offset.h:27
/// C++: enum class EndType {Polygon, Joined, Butt, Square, Round};
//Butt   : offsets both sides of a path, with square blunt ends
//Square : offsets both sides of a path, with square extended ends
//Round  : offsets both sides of a path, with round extended ends
//Joined : offsets both sides of a path, with joined ends
//Polygon: offsets only one side of a closed path
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EndType {
    Polygon,
    Joined,
    Butt,
    Square,
    Round,
}

/// clipper.offset.h:34
/// C++: typedef std::function<double(const Path64& path, const PathD& path_normals, size_t curr_idx, size_t prev_idx)> DeltaCallback64;
/// A `std::function` is copied by value and may be empty; here it is a shared pointer to
/// the function, or `None`.
#[derive(Clone, Default)]
pub struct DeltaCallback64(pub Option<Rc<dyn Fn(&Path64, &PathD, usize, usize) -> f64>>);

impl std::fmt::Debug for DeltaCallback64 {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(if self.0.is_some() { "DeltaCallback64(set)" } else { "DeltaCallback64(empty)" })
    }
}

impl DeltaCallback64 {
    /// A callback that holds `f`.
    pub fn new(f: impl Fn(&Path64, &PathD, usize, usize) -> f64 + 'static) -> Self {
        DeltaCallback64(Some(Rc::new(f)))
    }

    /// `deltaCallback64_ = nullptr`
    pub fn none() -> Self {
        DeltaCallback64(None)
    }

    /// `if (deltaCallback64_)`
    #[inline]
    fn is_set(&self) -> bool {
        self.0.is_some()
    }

    /// `deltaCallback64_(path, norms, curr_idx, prev_idx)`
    #[inline]
    fn call(&self, path: &Path64, path_normals: &PathD, curr_idx: usize, prev_idx: usize) -> f64 {
        (self.0.as_ref().expect("the callback is called only when it is set"))(path, path_normals, curr_idx, prev_idx)
    }
}

/// How often one `ClipperOffset` went through each of its routines and their branches.
/// Diagnostics the parity tests read to know their inputs reach them; they change nothing.
#[derive(Debug, Clone, Copy, Default)]
pub struct OffsetStats {
    /// `DoBevel` calls, and those of them at a line end (`j == k`).
    pub do_bevel: u64,
    pub do_bevel_end: u64,
    /// `DoSquare` calls, those at a line end, and those `OffsetPoint` made because a miter
    /// went over the limit.
    pub do_square: u64,
    pub do_square_end: u64,
    pub miter_exceeded: u64,
    /// `DoMiter` calls, and those of them `OffsetPoint` made for an almost straight join.
    pub do_miter: u64,
    pub almost_straight: u64,
    /// `DoRound` calls, those at a line end, and the points their arcs added.
    pub do_round: u64,
    pub do_round_end: u64,
    pub round_steps: u64,
    /// Concave joins of `OffsetPoint` (three points).
    pub concave: u64,
    /// `OffsetPoint` calls that returned at once because the two points are one.
    pub same_point: u64,
    /// Vertices whose delta was within the tolerance of zero (the point itself goes out).
    pub zero_delta: u64,
    /// `DoSquare` calls in which `GetSegmentIntersectPt` found the lines parallel.
    pub square_parallel: u64,
    /// `GetAvgUnitVector` calls whose sum was almost zero (`NormalizeVector` gave (0, 0)).
    pub avg_vector_zero: u64,
    /// Single points made a circle, made a square, and skipped for a delta below 1.
    pub point_circle: u64,
    pub point_square: u64,
    pub point_skipped: u64,
    /// `OffsetPolygon`, `OffsetOpenJoined` and `OffsetOpenPath` calls.
    pub offset_polygon: u64,
    pub offset_open_joined: u64,
    pub offset_open_path: u64,
    /// Two-point `Joined` paths whose end type became `Round` or `Square`.
    pub joined_two_points: u64,
    /// Groups that were reversed (their lowest path has a negative area).
    pub groups_reversed: u64,
    /// Polygon groups with no lowest path (no point at all), which make `delta_` positive.
    pub no_lowest_path: u64,
    /// `ExecuteInternal` calls with |delta| < 0.5 (the paths go out as they are).
    pub insignificant: u64,
    /// `ZCB` calls, and those of them that came down to the user's callback.
    pub zcb: u64,
    pub zcb_user: u64,
    /// Delta callback calls.
    pub delta_callbacks: u64,
}

//------------------------------------------------------------------------------
// Miscellaneous methods
//------------------------------------------------------------------------------

/// clipper.offset.cpp:40-57
/// C++: std::optional<size_t> GetLowestClosedPathIdx(const Paths64& paths)
pub fn get_lowest_closed_path_idx(paths: &Paths64) -> Option<usize> {
    let mut result: Option<usize> = None;
    let mut bot_pt = Point64::new(i64::MAX, i64::MIN);
    for (i, path) in paths.iter().enumerate() {
        for pt in path {
            if (pt.y < bot_pt.y) || ((pt.y == bot_pt.y) && (pt.x >= bot_pt.x)) {
                continue;
            }
            result = Some(i);
            bot_pt.x = pt.x;
            bot_pt.y = pt.y;
        }
    }
    result
}

/// clipper.offset.cpp of 1.5.4, :36-58, and what `Group::Group` (:148-155) makes of it
/// (not the pin; taken under the test probe only, see `version_154_probe`)
/// C++: void GetLowestClosedPathInfo(const Paths64& paths, std::optional<size_t>& idx, bool& is_neg_area)
/// C++: is_reversed = lowest_path_idx.has_value() && is_neg_area;
/// `is_neg_area` is not initialised in the C++; it is read only where `idx` has a value,
/// and then it has been set.
fn get_lowest_closed_path_info_v154(paths: &Paths64) -> (Option<usize>, bool) {
    let mut idx: Option<usize> = None;
    let mut is_neg_area = false;
    let mut bot_pt = Point64::new(i64::MAX, i64::MIN);
    for (i, path) in paths.iter().enumerate() {
        let mut a = MAX_DBL;
        for pt in path {
            if (pt.y < bot_pt.y) || ((pt.y == bot_pt.y) && (pt.x >= bot_pt.x)) {
                continue;
            }
            if a == MAX_DBL {
                a = area(path);
                if a == 0.0 {
                    break; // invalid closed path, so break from inner loop
                }
                is_neg_area = a < 0.0;
            }
            idx = Some(i);
            bot_pt.x = pt.x;
            bot_pt.y = pt.y;
        }
    }
    (idx, idx.is_some() && is_neg_area)
}

/// clipper.offset.cpp:59-67
/// C++: inline double Hypot(double x, double y)
/// C++: return std::sqrt(x * x + y * y);
/// `x * x + y * y` in one expression: fma(x, x, y * y).
#[inline]
fn hypot(x: f64, y: f64) -> f64 {
    // given that this is an internal function, and given the x and y parameters
    // will always be coordinate values (or the difference between coordinate values),
    // x and y should always be within INT64_MIN to INT64_MAX. Consequently,
    // there should be no risk that the following computation will overflow
    // see https://stackoverflow.com/a/32436148/359538
    x.mul_add(x, y * y).sqrt()
}

/// clipper.offset.cpp:69-78
/// C++: static PointD GetUnitNormal(const Point64& pt1, const Point64& pt2)
fn get_unit_normal(pt1: &Point64, pt2: &Point64) -> PointD {
    if pt1 == pt2 {
        return PointD::new(0.0, 0.0);
    }
    // `pt2.x - pt1.x` is a subtraction of int64_t: with coordinates in the library's range
    // it cannot overflow.
    let mut dx = pt2.x.wrapping_sub(pt1.x) as f64;
    let mut dy = pt2.y.wrapping_sub(pt1.y) as f64;
    let inverse_hypot = 1.0 / hypot(dx, dy);
    dx *= inverse_hypot;
    dy *= inverse_hypot;
    PointD::new(dy, -dx)
}

/// clipper.offset.cpp:80-83
/// C++: inline bool AlmostZero(double value, double epsilon = 0.001)
#[inline]
fn almost_zero(value: f64) -> bool {
    value.abs() < 0.001
}

/// clipper.offset.cpp:85-91
/// C++: inline PointD NormalizeVector(const PointD& vec)
#[inline]
fn normalize_vector(vec: &PointD) -> PointD {
    let h = hypot(vec.x, vec.y);
    if almost_zero(h) {
        return PointD::new(0.0, 0.0);
    }
    let inverse_hypot = 1.0 / h;
    PointD::new(vec.x * inverse_hypot, vec.y * inverse_hypot)
}

/// clipper.offset.cpp:93-96
/// C++: inline PointD GetAvgUnitVector(const PointD& vec1, const PointD& vec2)
#[inline]
fn get_avg_unit_vector(vec1: &PointD, vec2: &PointD) -> PointD {
    normalize_vector(&PointD::new(vec1.x + vec2.x, vec1.y + vec2.y))
}

/// clipper.offset.cpp:98-101
/// C++: inline bool IsClosedPath(EndType et)
#[inline]
pub fn is_closed_path(et: EndType) -> bool {
    et == EndType::Polygon || et == EndType::Joined
}

/// clipper.offset.cpp:103-110
/// C++: static inline Point64 GetPerpendic(const Point64& pt, const PointD& norm, double delta)
/// C++: return Point64(pt.x + norm.x * delta, pt.y + norm.y * delta, pt.z);
/// `pt.x + norm.x * delta`: fma(norm.x, delta, pt.x), then rounded to the integer.
#[inline]
fn get_perpendic(pt: &Point64, norm: &PointD, delta: f64) -> Point64 {
    point64_from_doubles(norm.x.mul_add(delta, pt.x as f64), norm.y.mul_add(delta, pt.y as f64), point_z(pt))
}

/// clipper.offset.cpp:112-119
/// C++: inline PointD GetPerpendicD(const Point64& pt, const PointD& norm, double delta)
#[inline]
fn get_perpendic_d(pt: &Point64, norm: &PointD, delta: f64) -> PointD {
    pointd_xyz(norm.x.mul_add(delta, pt.x as f64), norm.y.mul_add(delta, pt.y as f64), point_z(pt))
}

/// clipper.offset.cpp:121-131
/// C++: inline void NegatePath(PathD& path)
#[inline]
fn negate_path(path: &mut PathD) {
    for pt in path.iter_mut() {
        pt.x = -pt.x;
        pt.y = -pt.y;
        // #ifdef USINGZ: pt.z = pt.z;
    }
}

/// `std::min(a, b)` of doubles: `(b < a) ? b : a`.
#[inline]
fn std_min(a: f64, b: f64) -> f64 {
    if b < a {
        b
    } else {
        a
    }
}

//------------------------------------------------------------------------------
// ClipperOffset::Group methods
//------------------------------------------------------------------------------

/// clipper.offset.h:39-47
/// C++: class Group
#[derive(Debug, Clone)]
pub struct Group {
    paths_in: Paths64,
    lowest_path_idx: Option<usize>,
    is_reversed: bool,
    join_type: JoinType,
    end_type: EndType,
}

impl Group {
    /// clipper.offset.cpp:138-159
    /// C++: ClipperOffset::Group::Group(const Paths64& _paths, JoinType _join_type, EndType _end_type)
    fn new(paths: &Paths64, join_type: JoinType, end_type: EndType) -> Self {
        let mut paths_in = paths.clone();
        let is_joined = (end_type == EndType::Polygon) || (end_type == EndType::Joined);
        for p in paths_in.iter_mut() {
            strip_duplicates(p, is_joined);
        }

        let lowest_path_idx;
        let is_reversed;
        if end_type == EndType::Polygon && v154(V154_LOWEST_CLOSED_PATH) {
            (lowest_path_idx, is_reversed) = get_lowest_closed_path_info_v154(&paths_in);
        } else if end_type == EndType::Polygon {
            lowest_path_idx = get_lowest_closed_path_idx(&paths_in);
            // the lowermost path must be an outer path, so if its orientation is negative,
            // then flag the whole group is 'reversed' (will negate delta etc.)
            // as this is much more efficient than reversing every path.
            is_reversed = match lowest_path_idx {
                Some(idx) => area(&paths_in[idx]) < 0.0,
                None => false,
            };
        } else {
            lowest_path_idx = None;
            is_reversed = false;
        }
        Group { paths_in, lowest_path_idx, is_reversed, join_type, end_type }
    }
}

//------------------------------------------------------------------------------
// ClipperOffset methods
//------------------------------------------------------------------------------

/// clipper.offset.h:36-126
/// C++: class ClipperOffset
///
/// `Paths64* solution` points at the caller's paths, or at paths `Execute` makes for the
/// run that fills a tree; here the offsetter holds the paths while it runs (`solution`) and
/// hands them back. `PolyTree64* solution_tree` is the argument of `execute_internal`.
#[derive(Debug)]
pub struct ClipperOffset {
    error_code_: i32,
    delta_: f64,
    group_delta_: f64,
    temp_lim_: f64,
    steps_per_rad_: f64,
    step_sin_: f64,
    step_cos_: f64,
    norms: PathD,
    path_out: Path64,
    solution: Paths64,
    groups_: Vec<Group>,
    join_type_: JoinType,
    end_type_: EndType,

    miter_limit_: f64,
    arc_tolerance_: f64,
    preserve_collinear_: bool,
    reverse_solution_: bool,

    /// `#ifdef USINGZ`: `ZCallback64 zCallback64_ = nullptr;`
    z_callback64_: ZCallback64,
    delta_callback64_: DeltaCallback64,

    pub stats: OffsetStats,
}

impl Default for ClipperOffset {
    fn default() -> Self {
        ClipperOffset::new(2.0, 0.0, false, false)
    }
}

thread_local! {
    /// What `ZCB` counted on this thread: the callback the clipper holds cannot reach the
    /// offsetter's `stats` while the offsetter is borrowed, so `execute_internal` takes the
    /// counts from here.
    static ZCB_COUNTS: std::cell::Cell<(u64, u64)> = const { std::cell::Cell::new((0, 0)) };
}

/// clipper.offset.cpp:544-553 (`#ifdef USINGZ`)
/// C++: void ClipperOffset::ZCB(const Point64& bot1, const Point64& top1, const Point64& bot2, const Point64& top2, Point64& ip)
/// `z_callback64` is the member `zCallback64_`, the one thing of `this` the method reads.
fn zcb(z_callback64: &ZCallback64, bot1: &Point64, top1: &Point64, bot2: &Point64, top2: &Point64, ip: &mut Point64) {
    ZCB_COUNTS.with(|c| c.set((c.get().0 + 1, c.get().1)));
    if point_z(bot1) != 0 && ((point_z(bot1) == point_z(bot2)) || (point_z(bot1) == point_z(top2))) {
        point_set_z(ip, point_z(bot1));
    } else if point_z(bot2) != 0 && (point_z(bot2) == point_z(top1)) {
        point_set_z(ip, point_z(bot2));
    } else if point_z(top1) != 0 && (point_z(top1) == point_z(top2)) {
        point_set_z(ip, point_z(top1));
    } else if z_callback64.is_set() {
        ZCB_COUNTS.with(|c| c.set((c.get().0, c.get().1 + 1)));
        z_callback64.call(bot1, top1, bot2, top2, ip);
    }
}

impl ClipperOffset {
    /// clipper.offset.h:91-97
    /// C++: explicit ClipperOffset(double miter_limit = 2.0, double arc_tolerance = 0.0, bool preserve_collinear = false, bool reverse_solution = false)
    pub fn new(miter_limit: f64, arc_tolerance: f64, preserve_collinear: bool, reverse_solution: bool) -> Self {
        ClipperOffset {
            error_code_: 0,
            delta_: 0.0,
            group_delta_: 0.0,
            temp_lim_: 0.0,
            steps_per_rad_: 0.0,
            step_sin_: 0.0,
            step_cos_: 0.0,
            norms: PathD::new(),
            path_out: Path64::new(),
            solution: Paths64::new(),
            groups_: Vec::new(),
            join_type_: JoinType::Bevel,
            end_type_: EndType::Polygon,
            miter_limit_: miter_limit,
            arc_tolerance_: arc_tolerance,
            preserve_collinear_: preserve_collinear,
            reverse_solution_: reverse_solution,
            z_callback64_: ZCallback64::none(),
            delta_callback64_: DeltaCallback64::none(),
            stats: OffsetStats::default(),
        }
    }

    /// clipper.offset.h:101
    /// C++: int ErrorCode() const { return error_code_; };
    pub fn error_code(&self) -> i32 {
        self.error_code_
    }

    /// clipper.offset.cpp:165-168
    /// C++: void ClipperOffset::AddPath(const Path64& path, JoinType jt_, EndType et_)
    pub fn add_path(&mut self, path: &Path64, jt_: JoinType, et_: EndType) {
        self.groups_.push(Group::new(&vec![path.clone()], jt_, et_));
    }

    /// clipper.offset.cpp:170-174
    /// C++: void ClipperOffset::AddPaths(const Paths64 &paths, JoinType jt_, EndType et_)
    pub fn add_paths(&mut self, paths: &Paths64, jt_: JoinType, et_: EndType) {
        if paths.is_empty() {
            return;
        }
        self.groups_.push(Group::new(paths, jt_, et_));
    }

    /// clipper.offset.h:104
    /// C++: void Clear() { groups_.clear(); norms.clear(); };
    pub fn clear(&mut self) {
        self.groups_.clear();
        self.norms.clear();
    }

    /// clipper.offset.h:110
    /// C++: double MiterLimit() const { return miter_limit_; }
    pub fn miter_limit(&self) -> f64 {
        self.miter_limit_
    }

    /// clipper.offset.h:111
    /// C++: void MiterLimit(double miter_limit) { miter_limit_ = miter_limit; }
    pub fn set_miter_limit(&mut self, miter_limit: f64) {
        self.miter_limit_ = miter_limit;
    }

    //ArcTolerance: needed for rounded offsets (See offset_triginometry2.svg)
    /// clipper.offset.h:114
    /// C++: double ArcTolerance() const { return arc_tolerance_; }
    pub fn arc_tolerance(&self) -> f64 {
        self.arc_tolerance_
    }

    /// clipper.offset.h:115
    /// C++: void ArcTolerance(double arc_tolerance) { arc_tolerance_ = arc_tolerance; }
    pub fn set_arc_tolerance(&mut self, arc_tolerance: f64) {
        self.arc_tolerance_ = arc_tolerance;
    }

    /// clipper.offset.h:117
    /// C++: bool PreserveCollinear() const { return preserve_collinear_; }
    pub fn preserve_collinear(&self) -> bool {
        self.preserve_collinear_
    }

    /// clipper.offset.h:118
    /// C++: void PreserveCollinear(bool preserve_collinear){preserve_collinear_ = preserve_collinear;}
    pub fn set_preserve_collinear(&mut self, preserve_collinear: bool) {
        self.preserve_collinear_ = preserve_collinear;
    }

    /// clipper.offset.h:120
    /// C++: bool ReverseSolution() const { return reverse_solution_; }
    pub fn reverse_solution(&self) -> bool {
        self.reverse_solution_
    }

    /// clipper.offset.h:121
    /// C++: void ReverseSolution(bool reverse_solution) {reverse_solution_ = reverse_solution;}
    pub fn set_reverse_solution(&mut self, reverse_solution: bool) {
        self.reverse_solution_ = reverse_solution;
    }

    /// clipper.offset.h:123-125 (`#ifdef USINGZ`)
    /// C++: void SetZCallback(ZCallback64 cb) { zCallback64_ = cb; }
    /// (in the build without `USINGZ`, `ZCallback64` holds nothing.)
    pub fn set_z_callback(&mut self, cb: ZCallback64) {
        self.z_callback64_ = cb;
    }

    /// clipper.offset.h:126
    /// C++: void SetDeltaCallback(DeltaCallback64 cb) { deltaCallback64_ = cb; }
    pub fn set_delta_callback(&mut self, cb: DeltaCallback64) {
        self.delta_callback64_ = cb;
    }

    /// `deltaCallback64_(path, norms, j, k)`
    #[inline]
    fn delta_callback(&mut self, path: &Path64, j: usize, k: usize) -> f64 {
        self.stats.delta_callbacks += 1;
        self.delta_callback64_.call(path, &self.norms, j, k)
    }

    /// clipper.offset.cpp:176-185
    /// C++: void ClipperOffset::BuildNormals(const Path64& path)
    fn build_normals(&mut self, path: &Path64) {
        self.norms.clear();
        self.norms.reserve(path.len());
        if path.is_empty() {
            return;
        }
        let path_stop_iter = path.len() - 1;
        for path_iter in 0..path_stop_iter {
            self.norms.push(get_unit_normal(&path[path_iter], &path[path_iter + 1]));
        }
        self.norms.push(get_unit_normal(&path[path_stop_iter], &path[0]));
    }

    /// clipper.offset.cpp:187-212
    /// C++: void ClipperOffset::DoBevel(const Path64& path, size_t j, size_t k)
    fn do_bevel(&mut self, path: &Path64, j: usize, k: usize) {
        self.stats.do_bevel += 1;
        let pt1;
        let pt2;
        let (px, py, pz) = (path[j].x as f64, path[j].y as f64, point_z(&path[j]));
        if j == k {
            self.stats.do_bevel_end += 1;
            let abs_delta = self.group_delta_.abs();
            // `path[j].x - abs_delta * norms[j].x`: fma(-abs_delta, norms[j].x, path[j].x)
            pt1 = pointd_xyz((-abs_delta).mul_add(self.norms[j].x, px), (-abs_delta).mul_add(self.norms[j].y, py), pz);
            // `path[j].x + abs_delta * norms[j].x`: fma(abs_delta, norms[j].x, path[j].x)
            pt2 = pointd_xyz(abs_delta.mul_add(self.norms[j].x, px), abs_delta.mul_add(self.norms[j].y, py), pz);
        } else {
            pt1 = pointd_xyz(self.group_delta_.mul_add(self.norms[k].x, px), self.group_delta_.mul_add(self.norms[k].y, py), pz);
            pt2 = pointd_xyz(self.group_delta_.mul_add(self.norms[j].x, px), self.group_delta_.mul_add(self.norms[j].y, py), pz);
        }
        self.path_out.push(point64_from_pointd(&pt1));
        self.path_out.push(point64_from_pointd(&pt2));
    }

    /// clipper.offset.cpp:214-253
    /// C++: void ClipperOffset::DoSquare(const Path64& path, size_t j, size_t k)
    fn do_square(&mut self, path: &Path64, j: usize, k: usize) {
        self.stats.do_square += 1;
        let vec;
        if j == k {
            self.stats.do_square_end += 1;
            vec = PointD::new(self.norms[j].y, -self.norms[j].x);
        } else {
            vec = get_avg_unit_vector(&PointD::new(-self.norms[k].y, self.norms[k].x), &PointD::new(self.norms[j].y, -self.norms[j].x));
            if vec.x == 0.0 && vec.y == 0.0 {
                self.stats.avg_vector_zero += 1;
            }
        }

        let abs_delta = self.group_delta_.abs();

        // now offset the original vertex delta units along unit vector
        let mut pt_q = pointd_from_point64(&path[j]);
        pt_q = translate_point_d(&pt_q, abs_delta * vec.x, abs_delta * vec.y);
        // get perpendicular vertices
        let pt1 = translate_point_d(&pt_q, self.group_delta_ * vec.y, self.group_delta_ * -vec.x);
        let pt2 = translate_point_d(&pt_q, self.group_delta_ * -vec.y, self.group_delta_ * vec.x);
        // get 2 vertices along one edge offset
        let pt3 = get_perpendic_d(&path[k], &self.norms[k], self.group_delta_);
        if j == k {
            // `PointD(pt3.x + vec.x * group_delta_, pt3.y + vec.y * group_delta_)`:
            // fma(vec.x, group_delta_, pt3.x); z = 0.
            let pt4 = PointD::new(vec.x.mul_add(self.group_delta_, pt3.x), vec.y.mul_add(self.group_delta_, pt3.y));
            let mut pt = pt_q;
            if !get_segment_intersect_pt_d(&pt1, &pt2, &pt3, &pt4, &mut pt) {
                self.stats.square_parallel += 1;
            }
            //get the second intersect point through reflecion
            self.path_out.push(point64_from_pointd(&reflect_point_d(&pt, &pt_q)));
            self.path_out.push(point64_from_pointd(&pt));
        } else {
            let pt4 = get_perpendic_d(&path[j], &self.norms[k], self.group_delta_);
            let mut pt = pt_q;
            if !get_segment_intersect_pt_d(&pt1, &pt2, &pt3, &pt4, &mut pt) {
                self.stats.square_parallel += 1;
            }
            self.path_out.push(point64_from_pointd(&pt));
            //get the second intersect point through reflecion
            self.path_out.push(point64_from_pointd(&reflect_point_d(&pt, &pt_q)));
        }
    }

    /// clipper.offset.cpp:255-269
    /// C++: void ClipperOffset::DoMiter(const Path64& path, size_t j, size_t k, double cos_a)
    fn do_miter(&mut self, path: &Path64, j: usize, k: usize, cos_a: f64) {
        self.stats.do_miter += 1;
        let q = self.group_delta_ / (cos_a + 1.0);
        // `path[j].x + (norms[k].x + norms[j].x) * q`: fma(norms[k].x + norms[j].x, q, path[j].x)
        self.path_out.push(point64_from_doubles(
            (self.norms[k].x + self.norms[j].x).mul_add(q, path[j].x as f64),
            (self.norms[k].y + self.norms[j].y).mul_add(q, path[j].y as f64),
            point_z(&path[j]),
        ));
    }

    /// clipper.offset.cpp:271-308
    /// C++: void ClipperOffset::DoRound(const Path64& path, size_t j, size_t k, double angle)
    fn do_round(&mut self, path: &Path64, j: usize, k: usize, angle: f64) {
        self.stats.do_round += 1;
        if j == k {
            self.stats.do_round_end += 1;
        }
        if self.delta_callback64_.is_set() {
            // when deltaCallback64_ is assigned, group_delta_ won't be constant,
            // so we'll need to do the following calculations for *every* vertex.
            let abs_delta = self.group_delta_.abs();
            let arc_tol =
                if self.arc_tolerance_ > FLOATING_POINT_TOLERANCE { std_min(abs_delta, self.arc_tolerance_) } else { abs_delta * ARC_CONST };
            let steps_per_360 = std_min(PI / (1.0 - arc_tol / abs_delta).acos(), abs_delta * PI);
            // `std::sin(2 * PI / steps_per_360)` and `std::cos(2 * PI / steps_per_360)`
            (self.step_sin_, self.step_cos_) = crate::clipper::sin_cos(2.0 * PI / steps_per_360);
            if self.group_delta_ < 0.0 {
                self.step_sin_ = -self.step_sin_;
            }
            self.steps_per_rad_ = steps_per_360 / (2.0 * PI);
        }

        let pt = path[j];
        let mut offset_vec = PointD::new(self.norms[k].x * self.group_delta_, self.norms[k].y * self.group_delta_);

        if j == k {
            offset_vec.negate();
        }
        self.path_out.push(point64_from_doubles(pt.x as f64 + offset_vec.x, pt.y as f64 + offset_vec.y, point_z(&pt)));
        // `static_cast<int>(std::ceil(steps_per_rad_ * std::abs(angle)))` // #448, #456
        // FIDELITY-NOTE: the cast of a double out of the range of `int` (or of a NaN) is
        // undefined in the C++; the reference build saturates (NaN gives 0), as `as i32` does.
        let steps = (self.steps_per_rad_ * angle.abs()).ceil() as i32;
        for _ in 1..steps {
            // ie 1 less than steps
            // `offsetVec.x * step_cos_ - step_sin_ * offsetVec.y`: fma(x, cos, -(sin * y));
            // `offsetVec.x * step_sin_ + offsetVec.y * step_cos_`: fma(x, sin, y * cos).
            offset_vec = PointD::new(
                offset_vec.x.mul_add(self.step_cos_, -(self.step_sin_ * offset_vec.y)),
                offset_vec.x.mul_add(self.step_sin_, offset_vec.y * self.step_cos_),
            );
            self.stats.round_steps += 1;
            self.path_out.push(point64_from_doubles(pt.x as f64 + offset_vec.x, pt.y as f64 + offset_vec.y, point_z(&pt)));
        }
        self.path_out.push(get_perpendic(&path[j], &self.norms[j], self.group_delta_));
    }

    /// clipper.offset.cpp:310-370
    /// C++: void ClipperOffset::OffsetPoint(Group& group, const Path64& path, size_t j, size_t k)
    fn offset_point(&mut self, group_is_reversed: bool, path: &Path64, j: usize, k: usize) {
        // Let A = change in angle where edges join
        // A == 0: ie no change in angle (flat join)
        // A == PI: edges 'spike'
        // sin(A) < 0: right turning
        // cos(A) < 0: change in angle is more than 90 degree

        if path[j] == path[k] {
            self.stats.same_point += 1;
            return;
        }

        let mut sin_a = cross_product_vec(&self.norms[j], &self.norms[k]);
        let cos_a = dot_product_vec(&self.norms[j], &self.norms[k]);
        if sin_a > 1.0 {
            sin_a = 1.0;
        } else if sin_a < -1.0 {
            sin_a = -1.0;
        }

        if self.delta_callback64_.is_set() {
            self.group_delta_ = self.delta_callback(path, j, k);
            if group_is_reversed {
                self.group_delta_ = -self.group_delta_;
            }
        }
        if self.group_delta_.abs() <= FLOATING_POINT_TOLERANCE {
            self.stats.zero_delta += 1;
            self.path_out.push(path[j]);
            return;
        }

        if cos_a > -0.999 && (sin_a * self.group_delta_ < 0.0) {
            // test for concavity first (#593)
            // is concave
            // by far the simplest way to construct concave joins, especially those joining very
            // short segments, is to insert 3 points that produce negative regions. These regions
            // will be removed later by the finishing union operation. This is also the best way
            // to ensure that path reversals (ie over-shrunk paths) are removed.
            self.stats.concave += 1;
            // #ifdef USINGZ: `path_out.emplace_back(GetPerpendic(...), path[j].z)`, the z
            // `GetPerpendic` gave the point already.
            let p = get_perpendic(&path[j], &self.norms[k], self.group_delta_);
            self.path_out.push(point_xyz(p.x, p.y, point_z(&path[j])));
            self.path_out.push(path[j]); // (#405, #873, #916)
            let p = get_perpendic(&path[j], &self.norms[j], self.group_delta_);
            self.path_out.push(point_xyz(p.x, p.y, point_z(&path[j])));
        } else if cos_a > 0.999 && self.join_type_ != JoinType::Round {
            // almost straight - less than 2.5 degree (#424, #482, #526 & #724)
            self.stats.almost_straight += 1;
            self.do_miter(path, j, k, cos_a);
        } else if self.join_type_ == JoinType::Miter {
            // miter unless the angle is sufficiently acute to exceed ML
            if cos_a > self.temp_lim_ - 1.0 {
                self.do_miter(path, j, k, cos_a);
            } else {
                self.stats.miter_exceeded += 1;
                self.do_square(path, j, k);
            }
        } else if self.join_type_ == JoinType::Round {
            self.do_round(path, j, k, sin_a.atan2(cos_a));
        } else if self.join_type_ == JoinType::Bevel {
            self.do_bevel(path, j, k);
        } else {
            self.do_square(path, j, k);
        }
    }

    /// clipper.offset.cpp:372-378
    /// C++: void ClipperOffset::OffsetPolygon(Group& group, const Path64& path)
    fn offset_polygon(&mut self, group_is_reversed: bool, path: &Path64) {
        self.stats.offset_polygon += 1;
        self.path_out.clear();
        // for (j = 0, k = path.size() - 1; j < path.size(); k = j, ++j)
        let mut k = path.len().wrapping_sub(1);
        for j in 0..path.len() {
            self.offset_point(group_is_reversed, path, j, k);
            k = j;
        }
        self.solution.push(self.path_out.clone());
    }

    /// clipper.offset.cpp:380-393
    /// C++: void ClipperOffset::OffsetOpenJoined(Group& group, const Path64& path)
    ///
    /// FIDELITY-NOTE: on a path without points the C++ reads `norms[0]` of an empty vector
    /// (undefined). Such a path gives its two empty paths here and nothing is read.
    fn offset_open_joined(&mut self, group_is_reversed: bool, path: &Path64) {
        self.stats.offset_open_joined += 1;
        self.offset_polygon(group_is_reversed, path);
        let mut reverse_path = path.clone();
        reverse_path.reverse();

        //rebuild normals
        self.norms.reverse();
        if !self.norms.is_empty() {
            let first = self.norms[0];
            self.norms.push(first);
            self.norms.remove(0);
        }
        negate_path(&mut self.norms);

        self.offset_polygon(group_is_reversed, &reverse_path);
    }

    /// clipper.offset.cpp:395-455
    /// C++: void ClipperOffset::OffsetOpenPath(Group& group, const Path64& path)
    ///
    /// FIDELITY-NOTE: on a path without points the C++ reads `path[0]` of an empty vector
    /// (undefined). Such a path gives one empty path here.
    fn offset_open_path(&mut self, group_is_reversed: bool, path: &Path64) {
        self.stats.offset_open_path += 1;
        if path.is_empty() {
            self.solution.push(self.path_out.clone());
            return;
        }
        // do the line start cap
        if self.delta_callback64_.is_set() {
            self.group_delta_ = self.delta_callback(path, 0, 0);
        }

        if self.group_delta_.abs() <= FLOATING_POINT_TOLERANCE {
            self.stats.zero_delta += 1;
            self.path_out.push(path[0]);
        } else {
            match self.end_type_ {
                EndType::Butt => self.do_bevel(path, 0, 0),
                EndType::Round => self.do_round(path, 0, 0, PI),
                _ => self.do_square(path, 0, 0),
            }
        }

        let high_i = path.len() - 1;
        // offset the left side going forward
        let mut k = 0;
        for j in 1..high_i {
            self.offset_point(group_is_reversed, path, j, k);
            k = j;
        }

        // reverse normals
        for i in (1..=high_i).rev() {
            self.norms[i] = PointD::new(-self.norms[i - 1].x, -self.norms[i - 1].y);
        }
        self.norms[0] = self.norms[high_i];

        // do the line end cap
        if self.delta_callback64_.is_set() {
            self.group_delta_ = self.delta_callback(path, high_i, high_i);
        }

        if self.group_delta_.abs() <= FLOATING_POINT_TOLERANCE {
            self.stats.zero_delta += 1;
            self.path_out.push(path[high_i]);
        } else {
            match self.end_type_ {
                EndType::Butt => self.do_bevel(path, high_i, high_i),
                EndType::Round => self.do_round(path, high_i, high_i, PI),
                _ => self.do_square(path, high_i, high_i),
            }
        }

        // for (size_t j = highI -1, k = highI; j > 0; k = j, --j)
        let mut k = high_i;
        let mut j = high_i.wrapping_sub(1);
        while j > 0 && j != usize::MAX {
            self.offset_point(group_is_reversed, path, j, k);
            k = j;
            j -= 1;
        }
        self.solution.push(self.path_out.clone());
    }

    /// clipper.offset.cpp:457-542
    /// C++: void ClipperOffset::DoGroupOffset(Group& group)
    fn do_group_offset(&mut self, group: &Group) {
        if group.end_type == EndType::Polygon {
            // a straight path (2 points) can now also be 'polygon' offset
            // where the ends will be treated as (180 deg.) joins
            if group.lowest_path_idx.is_none() {
                self.stats.no_lowest_path += 1;
                self.delta_ = self.delta_.abs();
            }
            self.group_delta_ = if group.is_reversed { -self.delta_ } else { self.delta_ };
        } else {
            self.group_delta_ = self.delta_.abs(); // *0.5;
        }
        if group.is_reversed {
            self.stats.groups_reversed += 1;
        }

        let mut abs_delta = self.group_delta_.abs();
        self.join_type_ = group.join_type;
        self.end_type_ = group.end_type;

        if group.join_type == JoinType::Round || group.end_type == EndType::Round {
            // calculate the number of steps required to approximate a circle
            // (see https://www.angusj.com/clipper2/Docs/Trigonometry.htm)
            // arcTol - when arc_tolerance_ is undefined (0) then curve imprecision
            // will be relative to the size of the offset (delta). Obviously very
            //large offsets will almost always require much less precision.
            let arc_tol =
                if self.arc_tolerance_ > FLOATING_POINT_TOLERANCE { std_min(abs_delta, self.arc_tolerance_) } else { abs_delta * ARC_CONST };

            let steps_per_360 = std_min(PI / (1.0 - arc_tol / abs_delta).acos(), abs_delta * PI);
            // `std::sin(2 * PI / steps_per_360)` and `std::cos(2 * PI / steps_per_360)`
            (self.step_sin_, self.step_cos_) = crate::clipper::sin_cos(2.0 * PI / steps_per_360);
            if self.group_delta_ < 0.0 {
                self.step_sin_ = -self.step_sin_;
            }
            self.steps_per_rad_ = steps_per_360 / (2.0 * PI);
        }

        //double min_area = PI * Sqr(group_delta_);
        for path_in in group.paths_in.iter() {
            let path_len = path_in.len();
            self.path_out.clear();

            if path_len == 1 {
                // single point
                if self.delta_callback64_.is_set() {
                    self.group_delta_ = self.delta_callback(path_in, 0, 0);
                    if group.is_reversed {
                        self.group_delta_ = -self.group_delta_;
                    }
                    abs_delta = self.group_delta_.abs();
                }

                // `if (group_delta_ < 1) continue;` (a NaN goes on)
                if self.group_delta_ < 1.0 {
                    self.stats.point_skipped += 1;
                    continue;
                }
                let pt = path_in[0];
                //single vertex so build a circle or square ...
                if group.join_type == JoinType::Round {
                    self.stats.point_circle += 1;
                    let radius = abs_delta;
                    // `steps_per_rad_ * 2 * PI` is `(steps_per_rad_ * 2) * PI`. //#617
                    let steps: usize = if self.steps_per_rad_ > 0.0 { (self.steps_per_rad_ * 2.0 * PI).ceil() as usize } else { 0 };
                    self.path_out = ellipse(&pt, radius, radius, steps);
                    // #ifdef USINGZ
                    for p in self.path_out.iter_mut() {
                        point_set_z(p, point_z(&pt));
                    }
                } else {
                    self.stats.point_square += 1;
                    // `int d = (int)std::ceil(abs_delta);`
                    let d = abs_delta.ceil() as i32 as i64;
                    let r = Rect64::new(pt.x.wrapping_sub(d), pt.y.wrapping_sub(d), pt.x.wrapping_add(d), pt.y.wrapping_add(d));
                    self.path_out = r.as_path();
                    // #ifdef USINGZ
                    for p in self.path_out.iter_mut() {
                        point_set_z(p, point_z(&pt));
                    }
                }

                self.solution.push(self.path_out.clone());
                continue;
            } // end of offsetting a single point

            if (path_len == 2) && (group.end_type == EndType::Joined) {
                self.stats.joined_two_points += 1;
                self.end_type_ = if group.join_type == JoinType::Round { EndType::Round } else { EndType::Square };
            }

            self.build_normals(path_in);
            if self.end_type_ == EndType::Polygon {
                self.offset_polygon(group.is_reversed, path_in);
            } else if self.end_type_ == EndType::Joined {
                self.offset_open_joined(group.is_reversed, path_in);
            } else {
                self.offset_open_path(group.is_reversed, path_in);
            }
        }
    }

    /// clipper.offset.cpp:555-561
    /// C++: size_t ClipperOffset::CalcSolutionCapacity()
    fn calc_solution_capacity(&self) -> usize {
        let mut result = 0;
        for g in &self.groups_ {
            result += if g.end_type == EndType::Joined { g.paths_in.len() * 2 } else { g.paths_in.len() };
        }
        result
    }

    /// clipper.offset.cpp:563-574
    /// C++: bool ClipperOffset::CheckReverseOrientation()
    fn check_reverse_orientation(&self) -> bool {
        // nb: this assumes there's consistency in orientation between groups
        let mut is_reversed_orientation = false;
        for g in &self.groups_ {
            if g.end_type == EndType::Polygon {
                is_reversed_orientation = g.is_reversed;
                break;
            }
        }
        is_reversed_orientation
    }

    /// clipper.offset.cpp:576-631
    /// C++: void ClipperOffset::ExecuteInternal(double delta)
    /// `solution_tree` is the member of that name: the tree to fill, or `None` for the
    /// paths (`self.solution`).
    fn execute_internal(&mut self, delta: f64, solution_tree: Option<&mut PolyTree64>) {
        self.error_code_ = 0;
        if self.groups_.is_empty() {
            return;
        }
        self.solution.reserve(self.calc_solution_capacity());

        if delta.abs() < 0.5 {
            // ie: offset is insignificant
            self.stats.insignificant += 1;
            let mut sol_size = 0;
            for group in &self.groups_ {
                sol_size += group.paths_in.len();
            }
            self.solution.reserve(sol_size);
            for group in &self.groups_ {
                self.solution.extend(group.paths_in.iter().cloned());
            }
        } else {
            self.temp_lim_ = if self.miter_limit_ <= 1.0 { 2.0 } else { 2.0 / (self.miter_limit_ * self.miter_limit_) };

            self.delta_ = delta;
            // `DoGroupOffset(*git)` changes no group; the groups are taken out while the
            // offsetter is borrowed and put back.
            let groups = std::mem::take(&mut self.groups_);
            for group in &groups {
                self.do_group_offset(group);
                if self.error_code_ == 0 {
                    continue; // all OK
                }
                self.solution.clear();
            }
            self.groups_ = groups;
        }

        if self.solution.is_empty() {
            return;
        }

        let paths_reversed = self.check_reverse_orientation();
        //clean up self-intersections ...
        let mut c = Clipper64::new();
        c.set_preserve_collinear(false);
        if v154(V154_OFFSET_PRESERVE_COLLINEAR) {
            // clipper.offset.cpp of 1.5.4, :610 (under the test probe only, see
            // `version_154_probe`): `c.PreserveCollinear(preserve_collinear_);`
            c.set_preserve_collinear(self.preserve_collinear_);
        }
        //the solution should retain the orientation of the input
        c.set_reverse_solution(self.reverse_solution_ != paths_reversed);
        if USINGZ {
            ZCB_COUNTS.with(|n| n.set((0, 0)));
            c.set_z_callback(bind_zcb(zcb, &self.z_callback64_));
        }
        c.add_subject(&self.solution);
        if let Some(solution_tree) = solution_tree {
            if paths_reversed {
                c.execute_tree(ClipType::Union, FillRule::Negative, solution_tree);
            } else {
                c.execute_tree(ClipType::Union, FillRule::Positive, solution_tree);
            }
        } else if paths_reversed {
            c.execute(ClipType::Union, FillRule::Negative, &mut self.solution);
        } else {
            c.execute(ClipType::Union, FillRule::Positive, &mut self.solution);
        }
        if USINGZ {
            let (calls, user) = ZCB_COUNTS.with(|n| n.get());
            self.stats.zcb += calls;
            self.stats.zcb_user += user;
        }
    }

    /// clipper.offset.cpp:633-639
    /// C++: void ClipperOffset::Execute(double delta, Paths64& paths64)
    pub fn execute(&mut self, delta: f64, paths64: &mut Paths64) {
        paths64.clear();
        // solution = &paths64;
        self.solution = std::mem::take(paths64);
        // solution_tree = nullptr;
        self.execute_internal(delta, None);
        *paths64 = std::mem::take(&mut self.solution);
    }

    /// clipper.offset.cpp:642-650
    /// C++: void ClipperOffset::Execute(double delta, PolyTree64& polytree)
    pub fn execute_tree(&mut self, delta: f64, polytree: &mut PolyTree64) {
        polytree.clear();
        // solution_tree = &polytree;
        // solution = new Paths64();
        self.solution = Paths64::new();
        self.execute_internal(delta, Some(polytree));
        // delete solution; solution = nullptr;
        self.solution = Paths64::new();
    }

    /// clipper.offset.cpp:652-656
    /// C++: void ClipperOffset::Execute(DeltaCallback64 delta_cb, Paths64& paths)
    pub fn execute_delta_callback(&mut self, delta_cb: DeltaCallback64, paths: &mut Paths64) {
        self.delta_callback64_ = delta_cb;
        self.execute(1.0, paths);
    }
}
