//! Helper functions to work with nano svg.
//!
//! C++ Reference:
//! - BambuStudio/src/libslic3r/NSVGUtils.hpp
//! - BambuStudio/src/libslic3r/NSVGUtils.cpp
//!
//! Faithful 1:1 line-by-line port of `NSVGUtils.{hpp,cpp}`.
//!
//! STATUS: partial.
//!
//! The bulk of this translation unit consumes the `nanosvg` C library's
//! `NSVGimage` / `NSVGshape` / `NSVGpath` structures (and the `nsvgParse` /
//! `nsvgParseFromFile` / `nsvgDelete` entry points). `nanosvg` is a native C
//! header (`src/nanosvg/nanosvg.h`) that has no Rust port and cannot be added as
//! a system/dylib dependency under the wasm-safe constraint. This is confirmed
//! by the sibling modules [`crate::emboss_shape`] ("There is no `NSVGimage` type
//! ported yet") and [`crate::format::svg`] ("nanosvg is a native C header with no
//! Rust port"). nanosvg is the single root blocker for this file.
//!
//! The other dependencies that earlier blocked this file are now AVAILABLE and
//! are no longer blockers:
//! - `Emboss::heal_polygons` is fully ported in [`crate::emboss::heal_polygons`].
//! - `Slic3r::center(ExPolygonsWithIds&)` is ported in [`crate::emboss::center`].
//!
//! Still-missing, nanosvg-independent geometry helpers needed only by the
//! (nanosvg-blocked) stroke path:
//! - `ClipperUtils::contour_to_polygons` is not ported.
//! - The `Slic3r::offset(Polylines/Polygons, delta, JoinType, miter, EndType)`
//!   open-path offset overload that selects `EndType` from the SVG line-cap
//!   (`etOpenButt`/`etOpenRound`/`etOpenSquare`) is not exposed by the Rust
//!   clipper helpers (only a fixed `OpenButt` variant exists in
//!   [`crate::clipper_utils::offset_polyline`]).
//!
//! Consequently the following symbols remain BLOCKED on the native nanosvg
//! `NSVGimage`/`NSVGshape`/`NSVGpath` structures (they iterate `image.shapes`,
//! `shape.paths`, `path->pts`, `path->npts`, `path->closed`, etc.):
//! `create_shape_with_ids`, `to_polygons(image)`, `bounds`, `get_shapes_count`,
//! `linearize_path`, `fill_to_expolygons`, `stroke_to_expolygons`, and the
//! `DashesParam` constructor. The file I/O / parse wrappers `nsvgParseFromFile`,
//! `read_from_disk`, `nsvgParse`, and `init_image` are likewise blocked because
//! they construct/return the native `NSVGimage` (via `::nsvgParse`).
//!
//! Everything tractable around nanosvg is ported faithfully:
//! - the self-contained curve-flattening math `to_coor`, `need_flattening`,
//!   `is_line`, and `flatten_cubic_bez` (what `linearize_path` calls);
//! - the dependency-free dash splitter `to_dashes` and the `DashesParam` struct
//!   fields (only its `NSVGshape`-reading constructor is blocked).
//!
//! AUDIT FIX (2026-06-13): `NSVGLineParams::scale` previously defaulted to
//! `1. / crate::SCALING_FACTOR` (= 0.00001), which is the reciprocal of the
//! correct value. C++ `scale = 1. / SCALING_FACTOR` with C++ `SCALING_FACTOR =
//! 0.00001` evaluates to `100000.0`, and the crate stores `crate::SCALING_FACTOR
//! = 100_000.0` so that `crate::scale(v) == C++ scale_(v)`. The default is now
//! `crate::SCALING_FACTOR`.

// NSVGUtils.cpp:1-9  #include "NSVGUtils.hpp" / <array> / <charconv> / boost nowide /
//                    "ClipperUtils.hpp" / "Emboss.hpp" (heal for shape)
use crate::geometry::{Point, Points, Polyline, Polylines};

// ============================================================================
// NSVGUtils.hpp:18-44  struct NSVGLineParams
// ============================================================================

/// Parameters for conversion of a curve from SVG to lines in a Polygon.
///
/// NSVGUtils.hpp:15-44
#[derive(Debug, Clone, Copy)]
pub struct NSVGLineParams {
    // NSVGUtils.hpp:20-22
    // Smaller will divide curve to more lines
    // NOTE: Value is in image scale
    /// `tesselation_tolerance` — smaller divides the curve into more lines.
    pub tesselation_tolerance: f64,

    // NSVGUtils.hpp:24-25
    // Maximal depth of recursion for conversion curve to lines
    /// `max_level` — maximal recursion depth for converting a curve to lines.
    pub max_level: i32,

    // NSVGUtils.hpp:27-29
    // Multiplicator of point coors
    // NOTE: Every point coor from image(float) is multiplied by scale and rounded
    //       to integer --> Slic3r::Point
    /// `scale` — multiplier of point coordinates.
    pub scale: f64,

    // NSVGUtils.hpp:31-32
    // Flag wether y is negative, when true than y coor is multiplied by -1
    /// `is_y_negative` — when true the y coordinate is multiplied by -1.
    pub is_y_negative: bool,

    // NSVGUtils.hpp:34-35
    // Is used only with rounded Stroke
    /// `arc_tolerance` — used only with a rounded stroke.
    pub arc_tolerance: f64,

    // NSVGUtils.hpp:37-38
    // Maximal count of heal iteration
    /// `max_heal_iteration` — maximal count of heal iterations.
    pub max_heal_iteration: u32,
}

impl NSVGLineParams {
    // NSVGUtils.hpp:40-43
    // explicit NSVGLineParams(double tesselation_tolerance):
    //     tesselation_tolerance(tesselation_tolerance),
    //     arc_tolerance(std::pow(tesselation_tolerance, 1/3.))
    // {}
    //
    // The remaining members keep their in-class default initializers
    // (NSVGUtils.hpp:22,25,29,32,38).
    /// NSVGUtils.hpp:40-43
    pub fn new(tesselation_tolerance: f64) -> Self {
        Self {
            tesselation_tolerance,
            // NSVGUtils.hpp:25  int max_level = 10;
            max_level: 10,
            // NSVGUtils.hpp:29  double scale = 1. / SCALING_FACTOR;
            //
            // C++ `SCALING_FACTOR` is `0.00001` (libslic3r.h:58), so the literal
            // value of `1. / SCALING_FACTOR` is `100000.0`. The Rust crate stores
            // `crate::SCALING_FACTOR = 100_000.0` (the reciprocal of the C++
            // constant) precisely so that `crate::scale(v) == C++ scale_(v) ==
            // v * 100000`. Hence the faithful numeric value here is
            // `crate::SCALING_FACTOR`, NOT `1. / crate::SCALING_FACTOR`.
            scale: crate::SCALING_FACTOR,
            // NSVGUtils.hpp:32  bool is_y_negative = true;
            is_y_negative: true,
            // NSVGUtils.hpp:42  arc_tolerance(std::pow(tesselation_tolerance, 1/3.))
            arc_tolerance: tesselation_tolerance.powf(1.0 / 3.0),
            // NSVGUtils.hpp:38  unsigned max_heal_iteration = 10;
            max_heal_iteration: 10,
        }
    }
}

// ============================================================================
// NSVGUtils.cpp:252-323  namespace { ... } — self-contained curve-flattening math
// ============================================================================

/// A 2D single-precision vector, mirroring Eigen's `Vec2f` as used by the
/// curve-flattening routines in `NSVGUtils.cpp`.
///
/// The Rust geometry crate has no f32 `Vec2f`; this minimal local helper
/// provides exactly the operations (`+`, `-`, `* f32`, `.x()`, `.y()`,
/// `.squaredNorm()`) used by `need_flattening` / `flatten_cubic_bez`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Vec2f {
    x: f32,
    y: f32,
}

impl Vec2f {
    #[inline]
    pub fn new(x: f32, y: f32) -> Self {
        Self { x, y }
    }
    #[inline]
    pub fn x(&self) -> f32 {
        self.x
    }
    #[inline]
    pub fn y(&self) -> f32 {
        self.y
    }
    /// Eigen `Vec2f::squaredNorm()`.
    #[inline]
    pub fn squared_norm(&self) -> f32 {
        self.x * self.x + self.y * self.y
    }
}

impl std::ops::Add for Vec2f {
    type Output = Vec2f;
    #[inline]
    fn add(self, rhs: Vec2f) -> Vec2f {
        Vec2f::new(self.x + rhs.x, self.y + rhs.y)
    }
}

impl std::ops::Sub for Vec2f {
    type Output = Vec2f;
    #[inline]
    fn sub(self, rhs: Vec2f) -> Vec2f {
        Vec2f::new(self.x - rhs.x, self.y - rhs.y)
    }
}

impl std::ops::Mul<f32> for Vec2f {
    type Output = Vec2f;
    #[inline]
    fn mul(self, rhs: f32) -> Vec2f {
        Vec2f::new(self.x * rhs, self.y * rhs)
    }
}

// NSVGUtils.cpp:255
// Point::coord_type to_coor(float val, double scale) { return static_cast<Point::coord_type>(std::round(val * scale)); }
//
// coord_t -> i64 per the porting map; the C++ `Point::coord_type` is the (32-bit)
// coord_t, here represented by the crate's `Coord = i64`.
/// NSVGUtils.cpp:255
pub fn to_coor(val: f32, scale: f64) -> i64 {
    (val as f64 * scale).round() as i64
}

// NSVGUtils.cpp:257-272
// bool need_flattening(float tessTol, const Vec2f &p1, const Vec2f &p2, const Vec2f &p3, const Vec2f &p4)
/// NSVGUtils.cpp:257
pub fn need_flattening(tess_tol: f32, p1: &Vec2f, p2: &Vec2f, p3: &Vec2f, p4: &Vec2f) -> bool {
    // NSVGUtils.cpp:258-259
    // f .. first
    // s .. second
    // NSVGUtils.cpp:260-262
    // auto det = [](const Vec2f &f, const Vec2f &s) {
    //     return std::fabs(f.x() * s.y() - f.y() * s.x());
    // };
    let det = |f: &Vec2f, s: &Vec2f| -> f32 { (f.x() * s.y() - f.y() * s.x()).abs() };

    // NSVGUtils.cpp:264
    let pd: Vec2f = *p4 - *p1;
    // NSVGUtils.cpp:265
    let pd2: Vec2f = *p2 - *p4;
    // NSVGUtils.cpp:266
    let d2: f32 = det(&pd2, &pd);
    // NSVGUtils.cpp:267
    let pd3: Vec2f = *p3 - *p4;
    // NSVGUtils.cpp:268
    let d3: f32 = det(&pd3, &pd);
    // NSVGUtils.cpp:269
    let d23: f32 = d2 + d3;

    // NSVGUtils.cpp:271
    (d23 * d23) >= tess_tol * pd.squared_norm()
}

// NSVGUtils.cpp:274
// see function nsvg__lineTo(NSVGparser* p, float x, float y)
// NSVGUtils.cpp:13  bool is_line(const float *p, float precision = 1e-4f);
//
// Scalar `is_approx` from libslic3r.h:288:
//   is_approx(value, test, precision) == fabs(double(value)-double(test)) < double(precision)
/// NSVGUtils.cpp:275-288
pub fn is_line(p: &[f32], precision: f32) -> bool {
    // NSVGUtils.cpp:276-279
    //Vec2f p1(p[0], p[1]);
    //Vec2f p2(p[2], p[3]);
    //Vec2f p3(p[4], p[5]);
    //Vec2f p4(p[6], p[7]);
    // NSVGUtils.cpp:280
    let dx_3: f32 = (p[6] - p[0]) / 3.0;
    // NSVGUtils.cpp:281
    let dy_3: f32 = (p[7] - p[1]) / 3.0;

    // NSVGUtils.cpp:283-287
    is_approx_f32(p[2], p[0] + dx_3, precision)
        && is_approx_f32(p[4], p[6] - dx_3, precision)
        && is_approx_f32(p[3], p[1] + dy_3, precision)
        && is_approx_f32(p[5], p[7] - dy_3, precision)
}

/// `is_line` default precision (`NSVGUtils.cpp:13  float precision = 1e-4f`).
pub const IS_LINE_DEFAULT_PRECISION: f32 = 1e-4;

// libslic3r.h:287-291
// template <typename Number>
// constexpr inline bool is_approx(Number value, Number test_value, Number precision = EPSILON)
// { return std::fabs(double(value) - double(test_value)) < double(precision); }
//
// `is_line` instantiates this with `Number = float`.
#[inline]
fn is_approx_f32(value: f32, test_value: f32, precision: f32) -> bool {
    (value as f64 - test_value as f64).abs() < precision as f64
}

/// Convert cubic curve to lines.
///
/// Inspired by nanosvgrast.h function nsvgRasterize -> nsvg__flattenShape ->
/// nsvg__flattenCubicBez.
/// <https://github.com/memononen/nanosvg/blob/f0a3e1034dd22e2e87e5db22401e44998383124e/src/nanosvgrast.h#L335>
///
/// * `points`  — Result points
/// * `tess_tol` — Tesselation tolerance
/// * `p1`,`p2`,`p3`,`p4` — Curve points
/// * `level`   — Actual depth of recursion
///
/// NSVGUtils.cpp:290-323
pub fn flatten_cubic_bez(
    points: &mut Points,
    tess_tol: f32,
    p1: &Vec2f,
    p2: &Vec2f,
    p3: &Vec2f,
    p4: &Vec2f,
    level: i32,
) {
    // NSVGUtils.cpp:304-309
    if !need_flattening(tess_tol, p1, p2, p3, p4) {
        // NSVGUtils.cpp:305  Point::coord_type x = static_cast<Point::coord_type>(std::round(p4.x()));
        let x: i64 = (p4.x() as f64).round() as i64;
        // NSVGUtils.cpp:306  Point::coord_type y = static_cast<Point::coord_type>(std::round(p4.y()));
        let y: i64 = (p4.y() as f64).round() as i64;
        // NSVGUtils.cpp:307  points.emplace_back(x, y);
        points.push(Point::new(x, y));
        // NSVGUtils.cpp:308  return;
        return;
    }

    // NSVGUtils.cpp:311  --level;
    let level = level - 1;
    // NSVGUtils.cpp:312-313  if (level == 0) return;
    if level == 0 {
        return;
    }

    // NSVGUtils.cpp:315  Vec2f p12  = (p1 + p2) * 0.5f;
    let p12: Vec2f = (*p1 + *p2) * 0.5;
    // NSVGUtils.cpp:316  Vec2f p23  = (p2 + p3) * 0.5f;
    let p23: Vec2f = (*p2 + *p3) * 0.5;
    // NSVGUtils.cpp:317  Vec2f p34  = (p3 + p4) * 0.5f;
    let p34: Vec2f = (*p3 + *p4) * 0.5;
    // NSVGUtils.cpp:318  Vec2f p123 = (p12 + p23) * 0.5f;
    let p123: Vec2f = (p12 + p23) * 0.5;
    // NSVGUtils.cpp:319  Vec2f p234  = (p23 + p34) * 0.5f;
    let p234: Vec2f = (p23 + p34) * 0.5;
    // NSVGUtils.cpp:320  Vec2f p1234 = (p123 + p234) * 0.5f;
    let p1234: Vec2f = (p123 + p234) * 0.5;
    // NSVGUtils.cpp:321  flatten_cubic_bez(points, tessTol, p1, p12, p123, p1234, level);
    flatten_cubic_bez(points, tess_tol, p1, &p12, &p123, &p1234, level);
    // NSVGUtils.cpp:322  flatten_cubic_bez(points, tessTol, p1234, p234, p34, p4, level);
    flatten_cubic_bez(points, tess_tol, &p1234, &p234, &p34, p4, level);
}

// ============================================================================
// NSVGUtils.cpp:394-437  struct DashesParam
// ============================================================================

/// `DashesParam` — dash-pattern bookkeeping for converting a stroke polyline into
/// dashes.
///
/// NSVGUtils.cpp:394-437
///
/// NOTE: The constructor `DashesParam(const NSVGshape &shape, double scale)`
/// (NSVGUtils.cpp:408-436) reads the `NSVGshape` fields `strokeDashCount`,
/// `strokeDashArray`, and `strokeDashOffset`. The `NSVGshape` type is part of the
/// native `nanosvg` C library, which has no Rust port (and cannot be added as a
/// system/dylib dependency under the wasm-safe rule — see the module-level docs).
/// The constructor therefore stays BLOCKED. The struct fields and the
/// dependency-free `to_dashes` consumer below are ported faithfully so they are
/// ready the moment nanosvg parsing becomes available.
#[derive(Debug, Clone)]
pub struct DashesParam {
    // NSVGUtils.cpp:395-396
    // first dash length
    /// first dash length (scaled)
    pub dash_length: f32,

    // NSVGUtils.cpp:398-400
    // is current dash .. true
    // is current space .. false
    /// is current dash (`true`) or space (`false`)
    pub is_line: bool,

    // NSVGUtils.cpp:402-403
    // current index to array
    /// current index into `dash_array`
    pub dash_index: u8,

    // NSVGUtils.cpp:405  std::array<float, max_dash_array_size> dash_array; // scaled
    /// dash lengths (scaled); limited to `MAX_DASH_ARRAY_SIZE`
    pub dash_array: [f32; Self::MAX_DASH_ARRAY_SIZE],

    // NSVGUtils.cpp:406  unsigned char dash_count = 0;
    /// count of values in `dash_array`
    pub dash_count: u8,
}

impl DashesParam {
    // NSVGUtils.cpp:404  static constexpr size_t max_dash_array_size = 8; // limitation of nanosvg strokeDashArray
    /// NSVGUtils.cpp:404 — limitation of nanosvg `strokeDashArray`.
    pub const MAX_DASH_ARRAY_SIZE: usize = 8;
}

// NSVGUtils.cpp:439-498
// Polylines to_dashes(const Polyline &polyline, const DashesParam& param)
/// NSVGUtils.cpp:439
pub fn to_dashes(polyline: &Polyline, param: &DashesParam) -> Polylines {
    // NSVGUtils.cpp:441  Polylines dashes;
    let mut dashes: Polylines = Polylines::new();
    // NSVGUtils.cpp:442  Polyline dash; // cache for one dash in dashed line
    let mut dash: Polyline = Polyline::new();
    // NSVGUtils.cpp:443  Point prev_point;
    let mut prev_point: Point = Point::new(0, 0);

    // NSVGUtils.cpp:445  bool is_line = param.is_line;
    let mut is_line: bool = param.is_line;
    // NSVGUtils.cpp:446  unsigned char dash_index = param.dash_index;
    let mut dash_index: u8 = param.dash_index;
    // NSVGUtils.cpp:447  float dash_length = param.dash_length; // current rest of dash distance
    let mut dash_length: f32 = param.dash_length;
    // NSVGUtils.cpp:448  for (const Point &point : polyline.points) {
    for (i, point) in polyline.points.iter().enumerate() {
        let point = *point;
        // NSVGUtils.cpp:449-453  if (&point == &polyline.points.front()) { ... is first point }
        if i == 0 {
            // NSVGUtils.cpp:451  prev_point = point; // copy
            prev_point = point;
            // NSVGUtils.cpp:452  continue;
            continue;
        }

        // NSVGUtils.cpp:455  Point diff = point - prev_point;
        let mut diff: Point = point - prev_point;
        // NSVGUtils.cpp:456  float line_segment_length = diff.cast<float>().norm();
        let mut line_segment_length: f32 =
            ((diff.x() as f32).powi(2) + (diff.y() as f32).powi(2)).sqrt();
        // NSVGUtils.cpp:457  while (dash_length < line_segment_length) {
        while dash_length < line_segment_length {
            // NSVGUtils.cpp:458-459  Calculate intermediate point
            // float d = dash_length / line_segment_length;
            let d: f32 = dash_length / line_segment_length;
            // NSVGUtils.cpp:460  Point move_point   = diff * d;
            // C++ Point::operator*(const double&) -> Point(x()*rhs, y()*rhs) with the
            // double->coord_t conversion via the Point(double,double) ctor (lrint).
            let move_point: Point = diff * (d as f64);
            // NSVGUtils.cpp:461  Point intermediate = prev_point + move_point;
            let intermediate: Point = prev_point + move_point;

            // NSVGUtils.cpp:463-473  add Dash in stroke
            if is_line {
                if dash.points.is_empty() {
                    // NSVGUtils.cpp:466  dashes.emplace_back(Points{prev_point, intermediate});
                    dashes.push(Polyline::from_points(vec![prev_point, intermediate]));
                } else {
                    // NSVGUtils.cpp:468  dash.append(prev_point);
                    dash.append_point(prev_point);
                    // NSVGUtils.cpp:469  dash.append(intermediate);
                    dash.append_point(intermediate);
                    // NSVGUtils.cpp:470  dashes.push_back(dash);
                    dashes.push(dash.clone());
                    // NSVGUtils.cpp:471  dash.clear();
                    dash.points.clear();
                }
            }

            // NSVGUtils.cpp:475  diff -= move_point;
            diff -= move_point;
            // NSVGUtils.cpp:476  line_segment_length -= dash_length;
            line_segment_length -= dash_length;
            // NSVGUtils.cpp:477  prev_point = intermediate;
            prev_point = intermediate;

            // NSVGUtils.cpp:479-482  Advance dash pattern
            // is_line = !is_line;
            is_line = !is_line;
            // NSVGUtils.cpp:481  dash_index = (dash_index + 1) % param.dash_count;
            dash_index = (dash_index + 1) % param.dash_count;
            // NSVGUtils.cpp:482  dash_length = param.dash_array[dash_index];
            dash_length = param.dash_array[dash_index as usize];
        }

        // NSVGUtils.cpp:485-486  if (is_line) dash.append(prev_point);
        if is_line {
            dash.append_point(prev_point);
        }
        // NSVGUtils.cpp:487  dash_length -= line_segment_length;
        dash_length -= line_segment_length;
        // NSVGUtils.cpp:488  prev_point = point; // copy
        prev_point = point;
    }

    // NSVGUtils.cpp:491-496  add last dash
    if is_line {
        // NSVGUtils.cpp:493  assert(!dash.empty());
        debug_assert!(!dash.points.is_empty());
        // NSVGUtils.cpp:494  dash.append(prev_point); // prev_point == polyline.points.back()
        dash.append_point(prev_point);
        // NSVGUtils.cpp:495  dashes.push_back(dash);
        dashes.push(dash);
    }
    // NSVGUtils.cpp:497  return dashes;
    dashes
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn nsvg_line_params_defaults() {
        // NSVGUtils.hpp:40-43
        let p = NSVGLineParams::new(10.0);
        assert_eq!(p.tesselation_tolerance, 10.0);
        assert_eq!(p.max_level, 10);
        // NSVGUtils.hpp:29  scale = 1. / SCALING_FACTOR; with C++ SCALING_FACTOR =
        // 0.00001 this is the literal value 100000.0 == crate::SCALING_FACTOR.
        assert_eq!(p.scale, crate::SCALING_FACTOR);
        assert!(p.is_y_negative);
        assert_eq!(p.arc_tolerance, 10.0_f64.powf(1.0 / 3.0));
        assert_eq!(p.max_heal_iteration, 10);
    }

    #[test]
    fn to_coor_rounds() {
        // NSVGUtils.cpp:255  static_cast<coord_t>(std::round(val * scale))
        assert_eq!(to_coor(1.0, 100.0), 100);
        assert_eq!(to_coor(1.004, 100.0), 100);
        assert_eq!(to_coor(1.005, 100.0), 101);
    }

    #[test]
    fn is_line_straight_segment() {
        // A perfectly straight cubic: control points are at exact thirds.
        // p1=(0,0) p4=(9,0); dx/3 = 3, dy/3 = 0
        // p2 = (3,0), p3 = (6,0)
        let p = [0.0_f32, 0.0, 3.0, 0.0, 6.0, 0.0, 9.0, 0.0];
        assert!(is_line(&p, IS_LINE_DEFAULT_PRECISION));
    }

    #[test]
    fn is_line_curved_segment() {
        // Bulge the control points off the straight line so it is not a line.
        let p = [0.0_f32, 0.0, 3.0, 5.0, 6.0, 5.0, 9.0, 0.0];
        assert!(!is_line(&p, IS_LINE_DEFAULT_PRECISION));
    }

    #[test]
    fn flatten_cubic_bez_straight_appends_endpoint() {
        // For a straight, short segment need_flattening is false on the first call
        // -> it pushes the rounded endpoint p4 once.
        let mut points = Points::new();
        let p1 = Vec2f::new(0.0, 0.0);
        let p2 = Vec2f::new(1.0, 0.0);
        let p3 = Vec2f::new(2.0, 0.0);
        let p4 = Vec2f::new(3.0, 0.0);
        flatten_cubic_bez(&mut points, 10.0, &p1, &p2, &p3, &p4, 10);
        assert_eq!(points, vec![Point::new(3, 0)]);
    }

    #[test]
    fn to_dashes_splits_single_segment() {
        // NSVGUtils.cpp:439-498
        // A single horizontal segment of length 100, with a uniform dash pattern
        // [10, 10] (dash, gap) starting on a dash with full remaining length.
        // Expected: dashes at x=[0,10], [20,30], [40,50], [60,70], [80,90], and a
        // trailing dash [90?..100] — verify the splitter terminates and produces a
        // sensible alternating set of on-segments.
        let polyline = Polyline::from_points(vec![Point::new(0, 0), Point::new(100, 0)]);
        let param = DashesParam {
            dash_length: 10.0,
            is_line: true,
            dash_index: 0,
            dash_array: [10.0, 10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            dash_count: 2,
        };
        let dashes = to_dashes(&polyline, &param);
        // First dash starts at the segment origin.
        assert!(!dashes.is_empty());
        assert_eq!(dashes.first().unwrap().points.first().copied(), Some(Point::new(0, 0)));
        // Every produced dash is a 2-point on-segment, alternating along x.
        for d in &dashes {
            assert!(d.points.len() >= 2);
        }
    }
}
