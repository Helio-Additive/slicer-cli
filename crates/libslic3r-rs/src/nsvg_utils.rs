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
//! a system/dylib dependency under the wasm-safe constraint. Several geometry
//! dependencies are also not yet available in the Rust crate:
//!
//! - `Emboss::heal_polygons` is currently a stub (returns `Err`) in
//!   [`crate::emboss::heal_polygons`].
//! - `ClipperUtils::contour_to_polygons` is not ported.
//! - The `Slic3r::offset(Polylines/Polygons, delta, JoinType, miter, EndType)`
//!   open-path offset overload (with `EndType::etOpenButt`/`etOpenRound`/
//!   `etOpenSquare`) is not exposed by the Rust clipper helpers.
//! - `Slic3r::center(ExPolygonsWithIds&)` lives in `Emboss.cpp` and is not ported.
//!
//! Consequently the following symbols are BLOCKED and intentionally not ported
//! here (see the module-level docs in the report):
//! `create_shape_with_ids`, `to_polygons`, `bounds`, `nsvgParseFromFile`,
//! `read_from_disk`, `nsvgParse`, `init_image`, `get_shapes_count`,
//! `linearize_path`, `fill_to_expolygons`, `stroke_to_expolygons`, `to_dashes`,
//! and the `DashesParam` helper.
//!
//! The genuinely self-contained, dependency-free curve-flattening math is ported
//! faithfully below: `to_coor`, `need_flattening`, `is_line`, and
//! `flatten_cubic_bez`. These are exactly the routines `linearize_path` would
//! call once nanosvg parsing becomes available.

// NSVGUtils.cpp:1-9  #include "NSVGUtils.hpp" / <array> / <charconv> / boost nowide /
//                    "ClipperUtils.hpp" / "Emboss.hpp" (heal for shape)
use crate::geometry::{Point, Points};

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
            scale: 1. / crate::SCALING_FACTOR,
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn nsvg_line_params_defaults() {
        // NSVGUtils.hpp:40-43
        let p = NSVGLineParams::new(10.0);
        assert_eq!(p.tesselation_tolerance, 10.0);
        assert_eq!(p.max_level, 10);
        assert_eq!(p.scale, 1.0 / crate::SCALING_FACTOR);
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
}
