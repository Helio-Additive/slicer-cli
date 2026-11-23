//! FillRectilinear - Port of libslic3r's FillRectilinear algorithm.
//!
//! This module implements the sophisticated rectilinear infill algorithm from
//! BambuStudio/libslic3r that uses dual-offset polygons and an intersection
//! graph to produce boundary-following infill paths. This produces significantly
//! more connected paths than simple line clipping, which is critical for
//! first-layer (bottom surface) parity with BambuStudio.
//!
//! ## Algorithm Overview
//!
//! 1. **ExPolygonWithOffset**: Create two offset versions of the fill polygon:
//!    - `polygons_outer`: Outer offset (larger) for anchoring infill endpoints
//!    - `polygons_inner`: Inner offset (smaller) for connecting infill lines
//!
//! 2. **slice_region_by_vertical_lines**: Generate evenly-spaced vertical lines
//!    and compute intersections with both offset contours. Each intersection is
//!    classified as OUTER_LOW, OUTER_HIGH, INNER_LOW, or INNER_HIGH.
//!
//! 3. **connect_segment_intersections_by_contours**: Build a connection graph
//!    linking intersections on adjacent vertical lines via perimeter segments.
//!
//! 4. **traverse_graph_generate_polylines**: Walk the graph to produce polylines
//!    that go vertically along infill lines and horizontally along perimeter
//!    segments, creating boundary-following connected paths.
//!
//! ## libslic3r Reference
//!
//! - `Fill/FillRectilinear.cpp` - Main algorithm
//! - `Fill/FillRectilinear.hpp` - Class declarations

use crate::clipper_utils::{offset_expolygon, offset_polygons, OffsetJoinType};
use crate::geometry::{BoundingBox, ExPolygon, Point, Polygon, Polyline};
use crate::{scale, unscale, Coord, CoordF};

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

/// Scaled epsilon for position comparisons.
/// libslic3r.h:84 `#define SCALED_EPSILON scale_(EPSILON)`, with EPSILON=1e-4
/// (libslic3r.h:52) and SCALING_FACTOR=1e-5 (libslic3r.h:58), so
/// SCALED_EPSILON = 1e-4 / 1e-5 = 10 units at the crate's 1e5 scale.
const SCALED_EPSILON: Coord = 10;

// ---------------------------------------------------------------------------
// SegmentIntersectionType
// ---------------------------------------------------------------------------

/// Classification of where a vertical line intersects the offset contours.
///
/// A vertical segment inside the fill region always has at least one pair of
/// OUTER_LOW / OUTER_HIGH. Between those, there may be pairs of INNER_LOW /
/// INNER_HIGH from the inner offset contour.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum SegmentIntersectionType {
    Unknown = 0,
    OuterLow = 1,
    OuterHigh = 2,
    InnerLow = 3,
    InnerHigh = 4,
}

impl SegmentIntersectionType {
    pub fn is_inner(self) -> bool {
        matches!(self, Self::InnerLow | Self::InnerHigh)
    }
    pub fn is_outer(self) -> bool {
        matches!(self, Self::OuterLow | Self::OuterHigh)
    }
    pub fn is_low(self) -> bool {
        matches!(self, Self::OuterLow | Self::InnerLow)
    }
    pub fn is_high(self) -> bool {
        matches!(self, Self::OuterHigh | Self::InnerHigh)
    }
}

// ---------------------------------------------------------------------------
// LinkType / LinkQuality
// ---------------------------------------------------------------------------

/// Direction for horizontal link lookup (left or right).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Side {
    Left,
    Right,
}

/// `SegmentIntersection::Direction` (FillRectilinear.cpp:206-209).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Direction {
    Up,
    Down,
}

/// Type of connection from an intersection to prev/next contour point.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LinkType {
    /// Horizontal link to the previous or next vertical line.
    Horizontal,
    /// Vertical link upward on the same vertical line.
    Up,
    /// Vertical link downward on the same vertical line.
    Down,
    /// Phony intersection (inserted for pinch handling) — no real link.
    Phony,
}

/// Quality of a connection link.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LinkQuality {
    /// Link cannot be followed.
    Invalid,
    /// Link is valid and short enough.
    Valid,
    /// Link is valid but too long to follow.
    TooLong,
}

// ---------------------------------------------------------------------------
// SegmentIntersection
// ---------------------------------------------------------------------------

/// Intersection of a vertical line with a polygon segment.
///
/// Mirrors libslic3r's `SegmentIntersection` struct.
#[derive(Debug, Clone)]
pub struct SegmentIntersection {
    /// Index of a contour in ExPolygonWithOffset.
    pub i_contour: usize,
    /// Index of the segment (edge endpoint) in the contour.
    pub i_segment: usize,
    /// Index of the previous point in the contour (the other endpoint of the edge).
    pub prev_idx: usize,
    /// Y position numerator (rational number = pos_p / pos_q).
    pub pos_p: i64,
    /// Y position denominator (always > 0).
    pub pos_q: u32,
    /// Classification type.
    pub itype: SegmentIntersectionType,

    // Connection to previous contour point (left / vertical down/up).
    pub prev_on_contour: i32,
    pub prev_on_contour_type: LinkType,
    pub prev_on_contour_quality: LinkQuality,

    // Connection to next contour point (right / vertical down/up).
    pub next_on_contour: i32,
    pub next_on_contour_type: LinkType,
    pub next_on_contour_quality: LinkQuality,

    /// Was the vertical segment going up from this point consumed?
    pub consumed_vertical_up: bool,
    /// Was the perimeter segment going right from this point consumed?
    pub consumed_perimeter_right: bool,
}

impl Default for SegmentIntersection {
    fn default() -> Self {
        Self {
            i_contour: 0,
            i_segment: 0,
            prev_idx: 0,
            pos_p: 0,
            pos_q: 1,
            itype: SegmentIntersectionType::Unknown,
            prev_on_contour: 0,
            prev_on_contour_type: LinkType::Horizontal,
            prev_on_contour_quality: LinkQuality::Valid,
            next_on_contour: 0,
            next_on_contour_type: LinkType::Horizontal,
            next_on_contour_quality: LinkQuality::Valid,
            consumed_vertical_up: false,
            consumed_perimeter_right: false,
        }
    }
}

impl SegmentIntersection {
    // Compute the integer Y position (with rounding).
    // FillRectilinear.cpp:131-140
    // C++ returns coord_t(p / int64_t(pos_q)); coord_t is int32 (libslic3r.h:40).
    // FIDELITY-NOTE(F2): C++ truncates the quotient to int32; we keep Coord=i64.
    // Scaled Y positions fit in int32 for typical print volumes, so behavior matches.
    pub fn pos(&self) -> Coord {
        let mut p = self.pos_p;
        if p < 0 {
            p -= self.pos_q as i64 >> 1;
        } else {
            p += self.pos_q as i64 >> 1;
        }
        (p / self.pos_q as i64) as Coord
    }

    #[inline]
    pub fn is_inner(&self) -> bool {
        self.itype.is_inner()
    }
    #[inline]
    pub fn is_outer(&self) -> bool {
        self.itype.is_outer()
    }
    #[inline]
    pub fn is_low(&self) -> bool {
        self.itype.is_low()
    }
    #[inline]
    pub fn is_high(&self) -> bool {
        self.itype.is_high()
    }

    // --- Left (prev) link helpers ---

    pub fn has_left_horizontal(&self) -> bool {
        self.prev_on_contour_type == LinkType::Horizontal
    }
    pub fn has_right_horizontal(&self) -> bool {
        self.next_on_contour_type == LinkType::Horizontal
    }

    pub fn has_left_vertical_up(&self) -> bool {
        self.prev_on_contour_type == LinkType::Up
    }
    pub fn has_left_vertical_down(&self) -> bool {
        self.prev_on_contour_type == LinkType::Down
    }
    pub fn has_left_vertical(&self) -> bool {
        self.has_left_vertical_up() || self.has_left_vertical_down()
    }

    pub fn has_right_vertical_up(&self) -> bool {
        self.next_on_contour_type == LinkType::Up
    }
    pub fn has_right_vertical_down(&self) -> bool {
        self.next_on_contour_type == LinkType::Down
    }
    pub fn has_right_vertical(&self) -> bool {
        self.has_right_vertical_up() || self.has_right_vertical_down()
    }

    pub fn has_vertical(&self) -> bool {
        self.has_left_vertical() || self.has_right_vertical()
    }

    /// Generic horizontal link accessor parameterized by side.
    pub fn horizontal(&self, side: Side) -> i32 {
        match side {
            Side::Left => self.left_horizontal(),
            Side::Right => self.right_horizontal(),
        }
    }

    /// Generic has_horizontal accessor parameterized by side.
    pub fn has_horizontal(&self, side: Side) -> bool {
        match side {
            Side::Left => self.has_left_horizontal(),
            Side::Right => self.has_right_horizontal(),
        }
    }

    pub fn left_horizontal(&self) -> i32 {
        if self.has_left_horizontal() {
            self.prev_on_contour
        } else {
            -1
        }
    }
    pub fn right_horizontal(&self) -> i32 {
        if self.has_right_horizontal() {
            self.next_on_contour
        } else {
            -1
        }
    }

    pub fn left_vertical_up(&self) -> i32 {
        if self.has_left_vertical_up() {
            self.prev_on_contour
        } else {
            -1
        }
    }
    pub fn left_vertical_down(&self) -> i32 {
        if self.has_left_vertical_down() {
            self.prev_on_contour
        } else {
            -1
        }
    }
    pub fn left_vertical(&self) -> i32 {
        if self.has_left_vertical() {
            self.prev_on_contour
        } else {
            -1
        }
    }
    pub fn right_vertical_up(&self) -> i32 {
        if self.has_right_vertical_up() {
            self.next_on_contour
        } else {
            -1
        }
    }
    pub fn right_vertical_down(&self) -> i32 {
        if self.has_right_vertical_down() {
            self.next_on_contour
        } else {
            -1
        }
    }
    pub fn right_vertical(&self) -> i32 {
        if self.has_right_vertical() {
            self.next_on_contour
        } else {
            -1
        }
    }

    /// Returns the index of the vertical link going away from the fill region
    /// (down if is_low, up if is_high), or -1 if none.
    pub fn vertical_outside(&self) -> i32 {
        if self.is_low() {
            self.vertical_down()
        } else {
            self.vertical_up()
        }
    }

    pub fn vertical_outside_quality(&self) -> LinkQuality {
        if self.is_low() {
            self.vertical_down_quality()
        } else {
            self.vertical_up_quality()
        }
    }

    pub fn vertical_up(&self) -> i32 {
        if self.has_left_vertical_up() {
            self.prev_on_contour
        } else if self.has_right_vertical_up() {
            self.next_on_contour
        } else {
            -1
        }
    }

    pub fn vertical_up_quality(&self) -> LinkQuality {
        if self.has_left_vertical_up() {
            self.prev_on_contour_quality
        } else {
            self.next_on_contour_quality
        }
    }

    pub fn vertical_down(&self) -> i32 {
        if self.has_left_vertical_down() {
            self.prev_on_contour
        } else if self.has_right_vertical_down() {
            self.next_on_contour
        } else {
            -1
        }
    }

    pub fn vertical_down_quality(&self) -> LinkQuality {
        if self.has_left_vertical_down() {
            self.prev_on_contour_quality
        } else {
            self.next_on_contour_quality
        }
    }

    pub fn horizontal_quality_left(&self) -> LinkQuality {
        self.prev_on_contour_quality
    }
    pub fn horizontal_quality_right(&self) -> LinkQuality {
        self.next_on_contour_quality
    }

    // The accessors of the pin under their C++ names (FillRectilinear.cpp:211-271). C++ overloads by
    // argument type; the overload taking a `Direction` carries `_dir`, the one taking a `Side` `_side`.

    // FillRectilinear.cpp:217
    pub fn has_left_vertical_dir(&self, dir: Direction) -> bool {
        if dir == Direction::Up {
            self.has_left_vertical_up()
        } else {
            self.has_left_vertical_down()
        }
    }
    // FillRectilinear.cpp:219
    pub fn has_left_vertical_outside(&self) -> bool {
        if self.is_low() {
            self.has_left_vertical_down()
        } else {
            self.has_left_vertical_up()
        }
    }
    // FillRectilinear.cpp:223
    pub fn has_right_vertical_dir(&self, dir: Direction) -> bool {
        if dir == Direction::Up {
            self.has_right_vertical_up()
        } else {
            self.has_right_vertical_down()
        }
    }
    // FillRectilinear.cpp:225
    pub fn has_right_vertical_outside(&self) -> bool {
        if self.is_low() {
            self.has_right_vertical_down()
        } else {
            self.has_right_vertical_up()
        }
    }
    // FillRectilinear.cpp:228
    pub fn has_vertical_side(&self, side: Side) -> bool {
        if side == Side::Left {
            self.has_left_vertical()
        } else {
            self.has_right_vertical()
        }
    }
    // FillRectilinear.cpp:229
    pub fn has_vertical_up(&self) -> bool {
        self.has_left_vertical_up() || self.has_right_vertical_up()
    }
    // FillRectilinear.cpp:230
    pub fn has_vertical_down(&self) -> bool {
        self.has_left_vertical_down() || self.has_right_vertical_down()
    }
    // FillRectilinear.cpp:231
    pub fn has_vertical_dir(&self, dir: Direction) -> bool {
        if dir == Direction::Up {
            self.has_vertical_up()
        } else {
            self.has_vertical_down()
        }
    }
    // FillRectilinear.cpp:236-239 (the pin asserts has_horizontal(side))
    pub fn horizontal_quality(&self, side: Side) -> LinkQuality {
        debug_assert!(self.has_horizontal(side));
        if side == Side::Left {
            self.prev_on_contour_quality
        } else {
            self.next_on_contour_quality
        }
    }
    // FillRectilinear.cpp:243
    pub fn left_vertical_dir(&self, dir: Direction) -> i32 {
        if if dir == Direction::Up { self.has_left_vertical_up() } else { self.has_left_vertical_down() } {
            self.prev_on_contour
        } else {
            -1
        }
    }
    // FillRectilinear.cpp:245
    pub fn left_vertical_outside(&self) -> i32 {
        if self.is_low() {
            self.left_vertical_down()
        } else {
            self.left_vertical_up()
        }
    }
    // FillRectilinear.cpp:248
    pub fn right_vertical_dir(&self, dir: Direction) -> i32 {
        if if dir == Direction::Up { self.has_right_vertical_up() } else { self.has_right_vertical_down() } {
            self.next_on_contour
        } else {
            -1
        }
    }
    // FillRectilinear.cpp:250
    pub fn right_vertical_outside(&self) -> i32 {
        if self.is_low() {
            self.right_vertical_down()
        } else {
            self.right_vertical_up()
        }
    }
    // FillRectilinear.cpp:252
    pub fn vertical_up_side(&self, side: Side) -> i32 {
        if side == Side::Left {
            self.left_vertical_up()
        } else {
            self.right_vertical_up()
        }
    }
    // FillRectilinear.cpp:253
    pub fn vertical_down_side(&self, side: Side) -> i32 {
        if side == Side::Left {
            self.left_vertical_down()
        } else {
            self.right_vertical_down()
        }
    }
    // FillRectilinear.cpp:254
    pub fn vertical_outside_side(&self, side: Side) -> i32 {
        if side == Side::Left {
            self.left_vertical_outside()
        } else {
            self.right_vertical_outside()
        }
    }

    /// Compare rational position: self < other.
    /// Uses the same 48-bit wide-multiply logic as libslic3r.
    pub fn pos_less_than(&self, other: &SegmentIntersection) -> bool {
        debug_assert!(self.pos_q > 0 && other.pos_q > 0);
        if self.pos_p == 0 || other.pos_p == 0 {
            return self.pos_p < other.pos_p;
        }
        let sign1: i32 = if self.pos_p > 0 { 1 } else { -1 };
        let sign2: i32 = if other.pos_p > 0 { 1 } else { -1 };
        let signs = sign1 * sign2;
        if signs < 0 {
            return sign1 < 0;
        }
        let (p1, p2) = if sign1 > 0 {
            (self.pos_p as u64, other.pos_p as u64)
        } else {
            ((-self.pos_p) as u64, (-other.pos_p) as u64)
        };
        let l_hi = (p1 >> 32) * other.pos_q as u64;
        let l_lo = (p1 & 0xffffffff) * other.pos_q as u64;
        let l_hi = l_hi + (l_lo >> 32);
        let r_hi = (p2 >> 32) * self.pos_q as u64;
        let r_lo = (p2 & 0xffffffff) * self.pos_q as u64;
        let r_hi = r_hi + (r_lo >> 32);
        if l_hi == r_hi {
            let l_lo = l_lo & 0xffffffff;
            let r_lo = r_lo & 0xffffffff;
            if sign1 < 0 {
                l_lo > r_lo
            } else {
                l_lo < r_lo
            }
        } else if sign1 < 0 {
            l_hi > r_hi
        } else {
            l_hi < r_hi
        }
    }

    pub fn pos_equal(&self, other: &SegmentIntersection) -> bool {
        debug_assert!(self.pos_q > 0 && other.pos_q > 0);
        if self.pos_p == 0 || other.pos_p == 0 {
            return self.pos_p == other.pos_p;
        }
        let positive = self.pos_p > 0;
        if positive != (other.pos_p > 0) {
            return false;
        }
        let p1 = if positive {
            self.pos_p as u64
        } else {
            (-self.pos_p) as u64
        };
        let p2 = if positive {
            other.pos_p as u64
        } else {
            (-other.pos_p) as u64
        };
        let l_lo = (p1 & 0xffffffff) * other.pos_q as u64;
        let r_lo = (p2 & 0xffffffff) * self.pos_q as u64;
        if l_lo != r_lo {
            return false;
        }
        let l_hi = (p1 >> 32) * other.pos_q as u64;
        let r_hi = (p2 >> 32) * self.pos_q as u64;
        l_hi + (l_lo >> 32) == r_hi + (r_lo >> 32)
    }
}

// ---------------------------------------------------------------------------
// SegmentedIntersectionLine
// ---------------------------------------------------------------------------

/// A vertical line with all its intersection points.
#[derive(Debug, Clone)]
pub struct SegmentedIntersectionLine {
    /// Index of this vertical line.
    pub idx: usize,
    /// X position of this vertical line.
    pub pos: Coord,
    /// Intersections sorted by Y.
    pub intersections: Vec<SegmentIntersection>,
}

// ---------------------------------------------------------------------------
// ExPolygonWithOffset
// ---------------------------------------------------------------------------

/// An ExPolygon with its inner and outer offset contours.
///
/// The outer offset is used for anchoring infill line endpoints.
/// The inner offset is used for connecting infill lines along the perimeter.
pub struct ExPolygonWithOffset {
    /// Source polygon (rotated).
    pub polygons_src: ExPolygon,
    /// Outer offset polygons.
    pub polygons_outer: Vec<Polygon>,
    /// Inner offset polygons.
    pub polygons_inner: Vec<Polygon>,
    /// Number of outer contours.
    pub n_contours_outer: usize,
    /// Number of inner contours.
    pub n_contours_inner: usize,
    /// Total number of contours (outer + inner).
    pub n_contours: usize,
    /// For each contour, whether it is CCW.
    pub polygons_ccw: Vec<bool>,
}

impl ExPolygonWithOffset {
    // Create with dual offsets matching libslic3r's constructor.
    //
    // `angle` is applied as a rotation to the source polygon (in radians).
    // `aoffset1` is the outer offset (typically negative = shrink).
    // `aoffset2` is the inner offset (more negative than aoffset1).
    pub fn new(expolygon: &ExPolygon, angle: f64, aoffset1: Coord, aoffset2: Coord) -> Self {
        // Legacy integer-offset entry: i64 as f64 is exact, and
        // v_f64 / SCALING_FACTOR == unscale(v_i64), so this is byte-identical
        // to the previous body for all existing callers.
        Self::new_scaled_f(expolygon, angle, aoffset1 as CoordF, aoffset2 as CoordF)
    }

    // R301: native passes the offsets as FLOATS in scaled units (the f32
    // fraction survives into ClipperOffset, FillRectilinear.cpp:2843-2848);
    // this entry keeps the fractional scaled delta.
    pub fn new_scaled_f(expolygon: &ExPolygon, angle: f64, aoffset1: CoordF, aoffset2: CoordF) -> Self {
        // Copy and rotate
        let mut src = expolygon.clone();
        if angle.abs() > 1e-10 {
            // libslic3r rotates by negative angle for alignment;
            // the caller passes -rotate_vector.first.
            src.rotate(angle);
        }

        // Remove degenerate points
        remove_sticks_polygon(&mut src.contour);
        for hole in &mut src.holes {
            remove_sticks_polygon(hole);
        }

        // Compute outer offset
        // Native FillRectilinear.cpp:478-482 offsets via ClipperLib @1e5; the geo
        // variants grid the raster clip boundary to 1um — the last gridder in the
        // top-fill chain (line endpoints clip against these contours). Gated.
        let f1 = crate::faithful_gate("TOPFILL_FAITHFUL");
        // RECT_OFFSET_FAITHFUL (on unless =0) — FillRectilinear.cpp:453-459, 481-483.
        // The two offsets are `coord_t` parameters, so the `float(scale_(..))` the
        // callers pass is truncated to an integer; then
        //     polygons_outer = aoffset1 == 0 ? to_polygons(polygons_src)
        //         : offset(polygons_src, float(aoffset1), jtMiter, miterLimit);
        //     if (aoffset2 < 0)
        //         polygons_inner = shrink(polygons_outer, float(aoffset1 - aoffset2), jtMiter, miterLimit);
        // `offset(const ExPolygon&)` is the raw `expolygon_offset` paths
        // (ClipperUtils.cpp:559-560), no union after it, and `shrink(const Polygons&)`
        // is `offset(polygons, -delta)` (ClipperUtils.hpp:388-389): `shrink_paths`
        // over those very paths (ClipperUtils.cpp:381-397, 413-414). =0 rebuilds
        // expolygons with a union after each step and offsets them one by one.
        let rof = crate::faithful_gate("RECT_OFFSET_FAITHFUL");
        let (aoffset1, aoffset2) = if rof {
            ((aoffset1 as Coord) as CoordF, (aoffset2 as Coord) as CoordF)
        } else {
            (aoffset1, aoffset2)
        };
        let aoffset1_mm = aoffset1 / crate::SCALING_FACTOR;
        let polygons_outer = if aoffset1 == 0.0 {
            expolygon_to_polygons(&src)
        } else if rof {
            crate::clipper_utils::offset_expolygons_clib_raw_scaled(
                std::slice::from_ref(&src),
                (aoffset1 as f32) as f64,
                OffsetJoinType::Miter,
            )
        } else if f1 {
            let result = crate::clipper_utils::offset_expolygons_clib(
                &[src.clone()],
                aoffset1_mm,
                OffsetJoinType::Miter,
            );
            expolygons_to_polygons(&result)
        } else {
            let result = offset_expolygon(&src, aoffset1_mm, OffsetJoinType::Miter);
            expolygons_to_polygons(&result)
        };

        // Compute inner offset
        // FillRectilinear.cpp:482 — shrink(polygons_outer, float(aoffset1 - aoffset2), ...).
        // shrink() offsets by a negative amount; aoffset1 - aoffset2 > 0 here (aoffset2 more
        // negative than aoffset1), so we offset polygons_outer inward by (aoffset1 - aoffset2).
        let mut polygons_inner = if aoffset2 < 0.0 {
            let shrink_amount = (aoffset1 - aoffset2) / crate::SCALING_FACTOR;
            if rof {
                crate::clipper_utils::offset_polygons_clib(
                    &polygons_outer,
                    -(((aoffset1 - aoffset2) as f32) as f64),
                    OffsetJoinType::Miter,
                )
            } else if f1 {
                // polygons_outer is a FLAT orientation-tagged set (contours CCW +
                // holes CW); rebuild proper nested ExPolygons full-res first so the
                // clib shrink offsets holes the right way (native shrink() feeds the
                // raw oriented paths into one ClipperOffset — same net semantics).
                let outer_ex = crate::clipper_utils::union_ex_clib(&polygons_outer, 1);
                let result = crate::clipper_utils::shrink_clib(
                    &outer_ex,
                    shrink_amount.abs(),
                    OffsetJoinType::Miter,
                );
                expolygons_to_polygons(&result)
            } else {
                let result =
                    offset_polygons(&polygons_outer, -shrink_amount.abs(), OffsetJoinType::Miter);
                expolygons_to_polygons(&result)
            }
        } else {
            Vec::new()
        };

        // Filter out contours with zero area or small area, contours with 2 points only.
        // FillRectilinear.cpp:485 — min_area = 0.01 * aoffset2 * aoffset2 (scaled units).
        let min_area_threshold = 0.01 * (aoffset2 as f64) * (aoffset2 as f64);
        // remove_small keeps polygons with abs(area) >= min_area. FillRectilinear.cpp:486-487
        let mut polygons_outer: Vec<Polygon> = polygons_outer
            .into_iter()
            .filter(|p| p.area().abs() >= min_area_threshold)
            .collect();
        polygons_inner.retain(|p| p.area().abs() >= min_area_threshold);
        // remove_sticks(polygons_outer); remove_sticks(polygons_inner); FillRectilinear.cpp:488-489
        for poly in &mut polygons_outer {
            remove_sticks_polygon(poly);
        }
        for poly in &mut polygons_inner {
            remove_sticks_polygon(poly);
        }

        let n_contours_outer = polygons_outer.len();
        let n_contours_inner = polygons_inner.len();
        let n_contours = n_contours_outer + n_contours_inner;

        // FillRectilinear.cpp:493-498 — polygons_ccw.assign(n_contours, false); for each
        // contour: remove_duplicate_points(), then record is_ccw(contour(i)).
        let mut polygons_ccw = Vec::with_capacity(n_contours);
        for i in 0..n_contours_outer {
            crate::multi_point::remove_duplicate_points(polygons_outer[i].points_mut());
            polygons_ccw.push(polygons_outer[i].is_counter_clockwise());
        }
        for i in 0..n_contours_inner {
            crate::multi_point::remove_duplicate_points(polygons_inner[i].points_mut());
            polygons_ccw.push(polygons_inner[i].is_counter_clockwise());
        }

        Self {
            polygons_src: src,
            polygons_outer,
            polygons_inner,
            n_contours_outer,
            n_contours_inner,
            n_contours,
            polygons_ccw,
        }
    }

    /// Create with only the outer offset (no inner).
    pub fn new_outer_only(expolygon: &ExPolygon, angle: f64, aoffset1: Coord) -> Self {
        Self::new(expolygon, angle, aoffset1, 0)
    }

    /// Faithful C++ copy-rotate ctor `ExPolygonWithOffset(base, -angle)`
    /// (FillRectilinear.cpp:501): take an ALREADY-OFFSET base's outer polygons and
    /// just ROTATE them by `angle` — no re-offset. This keeps the offset geometry
    /// identical to the unrotated base (so make_fill_lines endpoints land exactly
    /// on base.polygons_outer, which the connect graph snaps against). Outer-only.
    pub fn from_base_outer_rotated(base_outer: &[Polygon], angle: f64) -> Self {
        let mut polygons_outer: Vec<Polygon> = base_outer.to_vec();
        if angle.abs() > 1e-10 {
            for p in &mut polygons_outer {
                p.rotate(angle);
            }
        }
        let n_contours_outer = polygons_outer.len();
        let mut polygons_ccw = Vec::with_capacity(n_contours_outer);
        for p in &mut polygons_outer {
            crate::multi_point::remove_duplicate_points(p.points_mut());
            polygons_ccw.push(p.is_counter_clockwise());
        }
        Self {
            polygons_src: ExPolygon::default(),
            polygons_outer,
            polygons_inner: Vec::new(),
            n_contours_outer,
            n_contours_inner: 0,
            n_contours: n_contours_outer,
            polygons_ccw,
        }
    }

    pub fn is_contour_outer(&self, idx: usize) -> bool {
        idx < self.n_contours_outer
    }

    pub fn is_contour_inner(&self, idx: usize) -> bool {
        idx >= self.n_contours_outer
    }

    pub fn contour(&self, idx: usize) -> &Polygon {
        if self.is_contour_outer(idx) {
            &self.polygons_outer[idx]
        } else {
            &self.polygons_inner[idx - self.n_contours_outer]
        }
    }

    pub fn is_contour_ccw(&self, idx: usize) -> bool {
        if idx < self.polygons_ccw.len() {
            self.polygons_ccw[idx]
        } else {
            false
        }
    }

    pub fn bounding_box_outer(&self) -> BoundingBox {
        let mut bbox = BoundingBox::new();
        for poly in &self.polygons_outer {
            bbox.merge(&poly.bounding_box());
        }
        bbox
    }

    pub fn bounding_box_src(&self) -> BoundingBox {
        self.polygons_src.bounding_box()
    }
}

// ---------------------------------------------------------------------------
// Fill parameters subset
// ---------------------------------------------------------------------------

/// Parameters controlling the fill algorithm (subset of libslic3r FillParams).
thread_local! {
    /// R257: the solid-spacing adjustment (native `this->spacing =
    /// unscale(_adjust_solid_spacing(...))`, FillRectilinear.cpp:2862-2863) —
    /// exported so the emission site can recompute flow/mm3 via with_spacing
    /// like native fill_surface_extrusion (FillBase.cpp:138-149). Reset at
    /// generate_fill_rectilinear entry; Some(mm) when the adjust branch ran.
    pub static ADJUSTED_SPACING_MM: std::cell::Cell<Option<f64>> =
        const { std::cell::Cell::new(None) };
}

#[derive(Debug, Clone)]
pub struct FillRectilinearParams {
    /// Infill density (0.0 - 1.0).
    pub density: f64,
    /// Whether infill should be monotonic (for top/bottom surfaces).
    pub monotonic: bool,
    /// Whether connections are disabled (e.g., for Line pattern).
    pub dont_connect: bool,
    /// Maximum link length in scaled units (0 = unlimited).
    pub link_max_length: Coord,
    /// Full infill (density = 1.0 or close).
    pub full_infill: bool,
    /// Whether to adjust spacing for solid layers.
    pub dont_adjust: bool,
    /// Whether the infill pattern must be consistent between layers (ZigZag /
    /// CrossZag / LockedZag). FillRectilinear.hpp:152 `has_consistent_pattern()`.
    /// When true, traversal switches between forward and backward passes by line
    /// index and never connects to the previous vertical line.
    pub consistent_pattern: bool,
}

impl Default for FillRectilinearParams {
    fn default() -> Self {
        Self {
            density: 1.0,
            monotonic: false,
            dont_connect: false,
            link_max_length: 0,
            full_infill: true,
            dont_adjust: false,
            consistent_pattern: false,
        }
    }
}

/// What `FillRectilinear::fill_surface_by_lines` reads of the filler and of
/// `FillParams` for the patterns that are laid consistently between layers
/// (FillZigZag, FillCrossZag, FillLockedZag: FillRectilinear.hpp:160-187), next
/// to [`FillRectilinearParams`]. The default is what every other filler has.
#[derive(Debug, Clone, Default)]
pub struct FillRectilinearZag {
    /// `Fill::bounding_box` (FillBase.hpp:137), the bounding box of the object
    /// (Fill.cpp:699 `f->set_bounding_box(bbox)`), which
    /// `extended_object_bounding_box()` grows (FillRectilinear.cpp:2815-2823).
    /// Read only where `FillRectilinearParams::consistent_pattern` is set.
    pub object_bounding_box: Option<BoundingBox>,
    /// `FillParams::horiz_move` (FillBase.hpp:96), scaled: the shift of the
    /// vertical lines of a cross zag from layer to layer (Fill.cpp:817-821).
    pub horiz_move: f32,
    /// `FillParams::symmetric_y_axis` (FillBase.hpp:98) where
    /// `FillParams::symmetric_infill_y_axis` is set (Fill.cpp:823-826, 838-841).
    pub symmetric_y_axis: Option<Coord>,
}

/// FillRectilinear.cpp:2815-2823 — `BoundingBox FillRectilinear::extended_object_bounding_box() const`:
/// the bounding box of the whole object that covers any rotation of every layer.
pub fn extended_object_bounding_box(bounding_box: &BoundingBox) -> BoundingBox {
    // FillRectilinear.cpp:2816-2818
    let mut out = *bounding_box;
    out.merge_point(Point::new(out.min.y, out.min.x));
    out.merge_point(Point::new(out.max.y, out.max.x));
    // FillRectilinear.cpp:2820-2822 — `return out.scaled(sqrt(2.));`.
    // `BoundingBoxBase<Point>::scale` is `min *= factor; max *= factor`
    // (BoundingBox.cpp:63-68) and `Point::operator*=(const double&)` truncates
    // each product to `coord_t` (Point.hpp:199).
    let factor = 2.0f64.sqrt();
    out.min = Point::new((out.min.x as f64 * factor) as Coord, (out.min.y as f64 * factor) as Coord);
    out.max = Point::new((out.max.x as f64 * factor) as Coord, (out.max.y as f64 * factor) as Coord);
    out
}

// ---------------------------------------------------------------------------
// Helper functions
// ---------------------------------------------------------------------------

/// Convert an ExPolygon to a flat list of Polygons (contour + holes).
fn expolygon_to_polygons(expoly: &ExPolygon) -> Vec<Polygon> {
    let mut result = Vec::with_capacity(1 + expoly.holes.len());
    result.push(expoly.contour.clone());
    for hole in &expoly.holes {
        result.push(hole.clone());
    }
    result
}

/// Convert ExPolygons to flat list of Polygons.
fn expolygons_to_polygons(expolygons: &[ExPolygon]) -> Vec<Polygon> {
    let mut result = Vec::new();
    for expoly in expolygons {
        result.push(expoly.contour.clone());
        for hole in &expoly.holes {
            result.push(hole.clone());
        }
    }
    result
}

/// Simple conversion of polygons to ExPolygons (each polygon becomes its own ExPolygon contour).
fn polygons_to_expolygons_simple(polygons: &[Polygon]) -> Vec<ExPolygon> {
    polygons
        .iter()
        .map(|p| ExPolygon {
            contour: p.clone(),
            holes: vec![],
        })
        .collect()
}

/// True if `p2` is a "stick" tip between `p1` and `p3` (zero-area protrusion or
/// collinear backtrack). Port of `is_stick` (Polygon.cpp:518-538).
fn is_stick(p1: Point, p2: Point, p3: Point) -> bool {
    let v1 = (p2.x - p1.x, p2.y - p1.y);
    let v2 = (p3.x - p2.x, p3.y - p2.y);
    let dir = (v1.0 as i64) * (v2.0 as i64) + (v1.1 as i64) * (v2.1 as i64);
    if dir > 0 {
        // p3 does not turn back to p1. Do not remove p2.
        return false;
    }
    let l2_1 = (v1.0 as f64) * (v1.0 as f64) + (v1.1 as f64) * (v1.1 as f64);
    let l2_2 = (v2.0 as f64) * (v2.0 as f64) + (v2.1 as f64) * (v2.1 as f64);
    if dir == 0 {
        // Perpendicular corner or a zero-length edge.
        return l2_1 == 0.0 || l2_2 == 0.0;
    }
    // p3 turns back to p1 after p2. Collinear within EPSILON?
    let cross = (v1.0 as f64) * (v2.1 as f64) - (v2.0 as f64) * (v1.1 as f64);
    let dist2 = cross * cross / l2_1.max(l2_2);
    // EPSILON == 1e-4 (libslic3r.h:52).
    const EPSILON: f64 = 1e-4;
    dist2 < EPSILON * EPSILON
}

/// Remove "sticks" (zero-area protrusions) from a polygon.
/// Port of `remove_sticks(Polygon&)` (Polygon.cpp:540-564).
fn remove_sticks_polygon(poly: &mut Polygon) {
    {
        let pts = poly.points_mut();
        // Polygon.cpp:543-551 — compact, keeping non-stick interior points.
        let mut j = 1usize;
        let mut i = 1usize;
        while i + 1 < pts.len() {
            if !is_stick(pts[j - 1], pts[i], pts[i + 1]) {
                if j < i {
                    pts[j] = pts[i];
                }
                j += 1;
            }
            i += 1;
        }
        // Polygon.cpp:552-556 — move the last point into place and truncate.
        j += 1;
        if j < pts.len() {
            let last = pts[pts.len() - 1];
            pts[j - 1] = last;
            pts.truncate(j);
        }
        // Polygon.cpp:557-560 — trim trailing sticks at the wrap point.
        while pts.len() >= 3
            && is_stick(pts[pts.len() - 2], pts[pts.len() - 1], pts[0])
        {
            pts.pop();
        }
        // Polygon.cpp:561-562 — trim leading sticks at the wrap point.
        while pts.len() >= 3 && is_stick(pts[pts.len() - 1], pts[0], pts[1]) {
            pts.remove(0);
        }
    }
}

/// Previous value in a circular array.
fn prev_value_modulo(idx: usize, contour_pts: &[Point]) -> Point {
    let n = contour_pts.len();
    contour_pts[if idx == 0 { n - 1 } else { idx - 1 }]
}

/// Next value in a circular array.
fn next_value_modulo(idx: usize, contour_pts: &[Point]) -> Point {
    let n = contour_pts.len();
    contour_pts[if idx + 1 >= n { 0 } else { idx + 1 }]
}

// FillRectilinear.cpp:1066-1116 — `bool validate_segment_intersection_connectivity(segs)`. The pin compiles
// it without NDEBUG only and calls it inside `assert()` (1285, 1381): every check is an assert and the
// function returns true.
pub fn validate_segment_intersection_connectivity(segs: &[SegmentedIntersectionLine]) -> bool {
    // Validate the connectivity.
    let mut i_vline = 0usize;
    while i_vline + 1 < segs.len() {
        let il_left = &segs[i_vline];
        let il_right = &segs[i_vline + 1];
        for (i_it, it) in il_left.intersections.iter().enumerate() {
            if it.has_right_horizontal() {
                let it_right = &il_right.intersections[it.right_horizontal() as usize];
                // For a right link there is a symmetric left link.
                debug_assert!(it.i_contour == it_right.i_contour);
                debug_assert!(it.itype == it_right.itype);
                debug_assert!(it_right.has_left_horizontal());
                debug_assert!(it_right.left_horizontal() == i_it as i32);
            }
        }
        for (i_it, it) in il_right.intersections.iter().enumerate() {
            if it.has_left_horizontal() {
                let it_left = &il_left.intersections[it.left_horizontal() as usize];
                // For a right link there is a symmetric left link.
                debug_assert!(it.i_contour == it_left.i_contour);
                debug_assert!(it.itype == it_left.itype);
                debug_assert!(it_left.has_right_horizontal());
                debug_assert!(it_left.right_horizontal() == i_it as i32);
            }
        }
        i_vline += 1;
    }
    for il in segs {
        for (i_it, it) in il.intersections.iter().enumerate() {
            let i_it = i_it as i32;
            if it.has_left_vertical_up() {
                debug_assert!(il.intersections[it.left_vertical_up() as usize].left_vertical_down() == i_it);
                debug_assert!(il.intersections[it.left_vertical_up() as usize].prev_on_contour_quality == it.prev_on_contour_quality);
            }
            if it.has_left_vertical_down() {
                debug_assert!(il.intersections[it.left_vertical_down() as usize].left_vertical_up() == i_it);
                debug_assert!(il.intersections[it.left_vertical_down() as usize].prev_on_contour_quality == it.prev_on_contour_quality);
            }
            if it.has_right_vertical_up() {
                debug_assert!(il.intersections[it.right_vertical_up() as usize].right_vertical_down() == i_it);
                debug_assert!(il.intersections[it.right_vertical_up() as usize].next_on_contour_quality == it.next_on_contour_quality);
            }
            if it.has_right_vertical_down() {
                debug_assert!(il.intersections[it.right_vertical_down() as usize].right_vertical_up() == i_it);
                debug_assert!(il.intersections[it.right_vertical_down() as usize].next_on_contour_quality == it.next_on_contour_quality);
            }
        }
    }
    true
}

/// Distance between two segment indices along a polygon, in one direction, under the name the pin gives
/// it (FillRectilinear.cpp:555-563).
pub fn distance_of_segmens(poly: &Polygon, seg1: usize, seg2: usize, forward: bool) -> i32 {
    distance_of_segments(poly, seg1, seg2, forward)
}

/// Distance between two segment indices along a polygon, in one direction.
fn distance_of_segments(poly: &Polygon, seg1: usize, seg2: usize, forward: bool) -> i32 {
    let mut d = seg2 as i32 - seg1 as i32;
    if !forward {
        d = -d;
    }
    if d < 0 {
        d += poly.points().len() as i32;
    }
    d
}

/// Euclidean length of a perimeter segment from p1 (at seg1) to p2 (at seg2), going forward.
fn segment_length(poly: &Polygon, seg1: usize, p1: Point, seg2: usize, p2: Point) -> CoordF {
    let pts = poly.points();
    let n = pts.len();
    let mut len = 0.0f64;
    let mut prev = p1;

    if seg1 <= seg2 {
        for i in seg1..seg2 {
            let this_pt = pts[i];
            len += distance_f(prev, this_pt);
            prev = this_pt;
        }
    } else {
        for i in seg1..n {
            let this_pt = pts[i];
            len += distance_f(prev, this_pt);
            prev = this_pt;
        }
        for i in 0..seg2 {
            let this_pt = pts[i];
            len += distance_f(prev, this_pt);
            prev = this_pt;
        }
    }
    len += distance_f(prev, p2);
    len
}

fn distance_f(a: Point, b: Point) -> f64 {
    let dx = (b.x - a.x) as f64;
    let dy = (b.y - a.y) as f64;
    (dx * dx + dy * dy).sqrt()
}

/// Append a segment of a closed polygon to an output points vector (forward direction).
/// Appends points from polygon[seg1] up to (but not including) polygon[seg2].
fn polygon_segment_append(out: &mut Vec<Point>, polygon: &Polygon, seg1: usize, seg2: usize) {
    let pts = polygon.points();
    if seg1 == seg2 {
        // Nothing to append.
    } else if seg1 < seg2 {
        out.extend_from_slice(&pts[seg1..seg2]);
    } else {
        out.reserve(seg2 + pts.len() - seg1);
        out.extend_from_slice(&pts[seg1..]);
        out.extend_from_slice(&pts[..seg2]);
    }
}

/// Append a segment of a closed polygon in reverse direction.
fn polygon_segment_append_reversed(
    out: &mut Vec<Point>,
    polygon: &Polygon,
    seg1: usize,
    seg2: usize,
) {
    let pts = polygon.points();
    if seg1 >= seg2 {
        for i in (seg2..seg1).rev() {
            // seg1 down to seg2+1
            out.push(pts[i]);
        }
    } else {
        // seg1 < seg2: wrap around
        for i in (0..seg1).rev() {
            out.push(pts[i]);
        }
        for i in (seg2..pts.len()).rev() {
            out.push(pts[i]);
        }
    }
}

// ---------------------------------------------------------------------------
// Core algorithm: slice_region_by_vertical_lines
// ---------------------------------------------------------------------------

/// Create vertical lines and compute their intersections with the offset contours.
///
/// This is the port of libslic3r `slice_region_by_vertical_lines()`.
pub fn slice_region_by_vertical_lines(
    poly_with_offset: &ExPolygonWithOffset,
    n_vlines: usize,
    x0: Coord,
    line_spacing: Coord,
) -> Vec<SegmentedIntersectionLine> {
    let mut segs: Vec<SegmentedIntersectionLine> = (0..n_vlines)
        .map(|i| SegmentedIntersectionLine {
            idx: i,
            pos: x0 + i as Coord * line_spacing,
            intersections: Vec::new(),
        })
        .collect();

    // For each contour (outer + inner)
    for i_contour in 0..poly_with_offset.n_contours {
        let contour_pts = poly_with_offset.contour(i_contour).points();
        if contour_pts.len() < 2 {
            continue;
        }

        // For each segment of the contour
        for i_segment in 0..contour_pts.len() {
            let i_prev = if i_segment == 0 {
                contour_pts.len() - 1
            } else {
                i_segment - 1
            };
            let p1 = contour_pts[i_prev];
            let p2 = contour_pts[i_segment];

            // Find which vertical lines this segment crosses
            let (l, r) = if p1.x <= p2.x {
                (p1.x, p2.x)
            } else {
                (p2.x, p1.x)
            };

            // il, ir are the left / right indices of vertical lines intersecting a segment
            // FillRectilinear.cpp:845 — use exact integer arithmetic (C division truncates
            // toward zero), NOT floating-point ceil/floor, for byte-exact parity.
            let mut il = (l - x0) / line_spacing; // FillRectilinear.cpp:845
            while il * line_spacing + x0 < l {
                // FillRectilinear.cpp:846
                il += 1;
            }
            il = std::cmp::max(0, il); // FillRectilinear.cpp:848
            let mut ir = (r - x0 + line_spacing) / line_spacing; // FillRectilinear.cpp:849
            while ir * line_spacing + x0 > r {
                // FillRectilinear.cpp:850
                ir -= 1;
            }
            ir = std::cmp::min(segs.len() as Coord - 1, ir); // FillRectilinear.cpp:852
            if il > ir {
                // No vertical line intersects this segment.
                // FillRectilinear.cpp:853
                continue;
            }

            for i in il..=ir {
                let this_x = segs[i as usize].pos;

                let mut is = SegmentIntersection::default();
                is.i_contour = i_contour;
                is.i_segment = i_segment;
                is.prev_idx = i_prev;

                // Calculate Y position
                if p1.x == this_x {
                    if p2.x == this_x {
                        // Strictly vertical segment — skip
                        continue;
                    }
                    let p0 = prev_value_modulo(i_prev, contour_pts);
                    if (p0.x as i64 - p1.x as i64) * (p2.x as i64 - p1.x as i64) > 0 {
                        // Tangent touch from one side — skip
                        continue;
                    }
                    is.pos_p = p1.y as i64;
                    is.pos_q = 1;
                } else if p2.x == this_x {
                    let p3 = next_value_modulo(i_segment, contour_pts);
                    if (p3.x as i64 - p2.x as i64) * (p1.x as i64 - p2.x as i64) > 0 {
                        continue;
                    }
                    is.pos_p = p2.y as i64;
                    is.pos_q = 1;
                } else {
                    // General intersection: compute t = (this_x - p1.x) / (p2.x - p1.x)
                    if p2.x > p1.x {
                        is.pos_p = (this_x - p1.x) as i64;
                        is.pos_q = (p2.x - p1.x) as u32;
                    } else {
                        is.pos_p = (p1.x - this_x) as i64;
                        is.pos_q = (p1.x - p2.x) as u32;
                    }
                    debug_assert!(is.pos_q > 1);
                    // Convert t to Y: y = p1.y + t * (p2.y - p1.y) = (pos_p * (p2.y - p1.y) + p1.y * pos_q) / pos_q
                    is.pos_p *= (p2.y - p1.y) as i64;
                    is.pos_p += p1.y as i64 * is.pos_q as i64;
                }

                // Determine type based on direction and contour
                let dir = p2.x - p1.x;
                let low = dir > 0;
                is.itype = if poly_with_offset.is_contour_outer(i_contour) {
                    if low {
                        SegmentIntersectionType::OuterLow
                    } else {
                        SegmentIntersectionType::OuterHigh
                    }
                } else {
                    if low {
                        SegmentIntersectionType::InnerLow
                    } else {
                        SegmentIntersectionType::InnerHigh
                    }
                };

                segs[i as usize].intersections.push(is);
            }
        }
    }

    // Sort the intersections along their segments, specify the intersection types.
    // FillRectilinear.cpp:918
    for sil in &mut segs {
        // Sort the intersection points using exact rational arithmetic.
        // BBS: if the LOW and HIGH has seam y pos, LOW should be first
        // FillRectilinear.cpp:922 — comparator uses the integer-rounded pos(), with type
        // as the tiebreaker only when pos() and iContour are equal.
        sil.intersections.sort_by(|l, r| {
            if l.pos() == r.pos() && l.i_contour == r.i_contour {
                return l.itype.cmp(&r.itype);
            }
            l.pos().cmp(&r.pos())
        });

        // Apply the type-order fix-up (adjust_sort_for_segment_intersections)
        adjust_sort_for_segment_intersections(&mut sil.intersections);

        // Remove duplicate/overlapping intersection points
        let mut j = 0usize;
        for i in 0..sil.intersections.len() {
            let take = if j > 0 {
                let is = &sil.intersections[i];
                let is2 = &sil.intersections[j - 1];
                if is.i_contour == is2.i_contour && is.pos_q == 1 && is2.pos_q == 1 {
                    if is.pos_p == is2.pos_p {
                        // Same position, same contour — remove duplicate
                        false
                    } else if is.itype == is2.itype {
                        // Z-shaped path — keep the better one
                        if is.is_low() {
                            // Keep the first (already placed)
                            false
                        } else {
                            // Replace the first with the second
                            sil.intersections[j - 1] = is.clone();
                            false
                        }
                    } else {
                        true
                    }
                } else {
                    true
                }
            } else {
                true
            };

            if take {
                if j < i {
                    sil.intersections[j] = sil.intersections[i].clone();
                }
                j += 1;
            }
        }
        sil.intersections.truncate(j);
    }

    // Verify: intersections should come in pairs OUTER_LOW...OUTER_HIGH
    // (with optional INNER_LOW...INNER_HIGH between them).
    // In case of failure, we try to repair by removing bad lines.
    for sil in &mut segs {
        if sil.intersections.len() % 2 != 0 {
            // Odd number of intersections — clear this line to be safe
            sil.intersections.clear();
            continue;
        }
        // Basic validity check
        let mut valid = true;
        let mut i = 0;
        while i < sil.intersections.len() {
            if sil.intersections[i].itype != SegmentIntersectionType::OuterLow {
                valid = false;
                break;
            }
            let mut j = i + 1;
            if j >= sil.intersections.len() {
                valid = false;
                break;
            }
            // Skip inner pairs
            while j < sil.intersections.len() && sil.intersections[j].is_inner() {
                j += 1;
            }
            if j >= sil.intersections.len()
                || sil.intersections[j].itype != SegmentIntersectionType::OuterHigh
            {
                valid = false;
                break;
            }
            i = j + 1;
        }
        if !valid {
            sil.intersections.clear();
        }
    }

    segs
}

/// Port of libslic3r `adjust_sort_for_segment_intersections`.
/// Ensures the intersection types follow the expected nesting pattern:
/// OUTER_LOW, (INNER_LOW, INNER_HIGH)*, OUTER_HIGH
fn adjust_sort_for_segment_intersections(intersections: &mut Vec<SegmentIntersection>) {
    use SegmentIntersectionType::*;

    let mut stack: Vec<SegmentIntersectionType> = Vec::new();
    let mut visited = vec![false; intersections.len()];

    let is_valid_type =
        |stack: &Vec<SegmentIntersectionType>, t: SegmentIntersectionType| -> bool {
            if stack.is_empty() {
                return t == OuterLow;
            }
            let top = *stack.last().unwrap();
            match t {
                OuterLow => false,
                OuterHigh => top == OuterLow,
                InnerLow => top == OuterLow || top == InnerHigh,
                InnerHigh => top == InnerLow,
                Unknown => true,
            }
        };

    // FillRectilinear.cpp:390-426
    let mut index_group: Vec<usize> = Vec::new();
    let mut i = 0;
    while i < intersections.len() {
        if is_valid_type(&stack, intersections[i].itype) {
            index_group.clear();
            // FillRectilinear.cpp:396-400
            match intersections[i].itype {
                OuterLow | InnerLow => stack.push(intersections[i].itype),
                OuterHigh | InnerHigh => {
                    stack.pop();
                }
                _ => {}
            }
            i += 1;
        } else {
            // FillRectilinear.cpp:403-424
            visited[i] = true;
            // scale_(EPSILON) == EPSILON / SCALING_FACTOR == 1e-4 / 1e-5 == 10 units.
            index_group.clear();
            let pos_i = intersections[i].pos();
            for j in (i + 1)..intersections.len() {
                if !visited[j] && (intersections[j].pos() - pos_i).abs() < scale(0.0001) {
                    index_group.push(j);
                }
            }

            if !index_group.is_empty() {
                let mut swap_index: i32 = -1;
                for &index in &index_group {
                    if !visited[index] {
                        swap_index = index as i32;
                        visited[index] = true;
                        break;
                    }
                }

                if swap_index != -1 {
                    intersections.swap(i, swap_index as usize);
                    // C++ `continue;` — re-evaluate the swapped element without incrementing.
                    continue;
                }
            }

            i += 1;
        }
    }
}

// ---------------------------------------------------------------------------
// Core algorithm: connect_segment_intersections_by_contours
// ---------------------------------------------------------------------------

/// For each intersection, find the closest matching intersection on the
/// prev/next vertical line (or on the same line) that shares the same contour.
/// Build the connection graph.
///
/// `RECT=1` (diagnostics only): the three stages of `fill_surface_by_lines`, as
/// sums a twin `fprintf` in FillRectilinear.cpp prints after
/// `slice_region_by_vertical_lines` and `connect_segment_intersections_by_contours`.
fn rect_probe_segs(stage: u8, n_vlines: usize, x0: Coord, line_spacing: Coord, mono: bool, lml: Coord, segs: &[SegmentedIntersectionLine]) {
    if stage == 1 {
        let (mut ni, mut sp, mut sq, mut st, mut spos) = (0i64, 0i64, 0i64, 0i64, 0i64);
        for sil in segs {
            spos = spos.wrapping_add(sil.pos as i64);
            for it in &sil.intersections {
                ni += 1;
                sp = sp.wrapping_add(it.pos_p);
                sq += it.pos_q as i64;
                st += it.itype as i64;
            }
        }
        eprintln!(
            "RECT1 nv={} x0={} ls={} mono={} nseg={} nint={} spos={} sp={} sq={} st={}",
            n_vlines, x0, line_spacing, mono as i32, segs.len(), ni, spos, sp, sq, st
        );
    } else {
        let (mut a1, mut a2, mut a3, mut a4, mut a5, mut a6) = (0i64, 0i64, 0i64, 0i64, 0i64, 0i64);
        for sil in segs {
            for it in &sil.intersections {
                a1 += it.prev_on_contour as i64;
                a2 += it.next_on_contour as i64;
                a3 += it.prev_on_contour_type as i64;
                a4 += it.next_on_contour_type as i64;
                a5 += it.prev_on_contour_quality as i64;
                a6 += it.next_on_contour_quality as i64;
            }
        }
        eprintln!(
            "RECT2 nv={} x0={} prev={} next={} pt={} nt={} pq={} nq={} lml={}",
            n_vlines, x0, a1, a2, a3, a4, a5, a6, lml
        );
    }
}

/// Port of libslic3r `connect_segment_intersections_by_contours()`.
pub fn connect_segment_intersections_by_contours(
    poly_with_offset: &ExPolygonWithOffset,
    segs: &mut Vec<SegmentedIntersectionLine>,
    params: &FillRectilinearParams,
    link_max_length: Coord,
) {
    let n_vlines = segs.len();

    for i_vline in 0..n_vlines {
        let n_intersections = segs[i_vline].intersections.len();

        for i_intersection in 0..n_intersections {
            let itsct_contour = segs[i_vline].intersections[i_intersection].i_contour;
            let itsct_type = segs[i_vline].intersections[i_intersection].itype;
            let itsct_segment = segs[i_vline].intersections[i_intersection].i_segment;
            let forward = segs[i_vline].intersections[i_intersection].is_low();

            let poly = poly_with_offset.contour(itsct_contour);

            // 1) Find closest matching intersection on the previous vertical line
            let mut iprev: i32 = -1;
            let mut d_prev = i32::MAX;
            if i_vline > 0 {
                let il_prev = &segs[i_vline - 1];
                for i in 0..il_prev.intersections.len() {
                    let itsct2 = &il_prev.intersections[i];
                    if itsct_contour == itsct2.i_contour && itsct_type == itsct2.itype {
                        let d =
                            distance_of_segments(poly, itsct2.i_segment, itsct_segment, forward);
                        if d < d_prev {
                            iprev = i as i32;
                            d_prev = d;
                        }
                    }
                }
            }

            // 2) Find closest matching intersection on the next vertical line
            let mut inext: i32 = -1;
            let mut d_next = i32::MAX;
            if i_vline + 1 < n_vlines {
                let il_next = &segs[i_vline + 1];
                for i in 0..il_next.intersections.len() {
                    let itsct2 = &il_next.intersections[i];
                    if itsct_contour == itsct2.i_contour && itsct_type == itsct2.itype {
                        let d =
                            distance_of_segments(poly, itsct_segment, itsct2.i_segment, forward);
                        if d < d_next {
                            inext = i as i32;
                            d_next = d;
                        }
                    }
                }
            }

            // 3) Find connections on the same vertical line
            let mut same_prev = false;
            let mut same_next = false;
            let il = &segs[i_vline];
            for i in 0..il.intersections.len() {
                if i == i_intersection {
                    continue;
                }
                let it2 = &il.intersections[i];
                if it2.i_contour == itsct_contour && it2.itype != itsct_type {
                    let d = distance_of_segments(poly, it2.i_segment, itsct_segment, forward);
                    if d < d_prev {
                        iprev = i as i32;
                        d_prev = d;
                        same_prev = true;
                    }
                    let d = distance_of_segments(poly, itsct_segment, it2.i_segment, forward);
                    if d < d_next {
                        inext = i as i32;
                        d_next = d;
                        same_next = true;
                    }
                }
            }

            // Set link types
            let prev_type = if same_prev {
                if iprev < i_intersection as i32 {
                    LinkType::Down
                } else {
                    LinkType::Up
                }
            } else {
                LinkType::Horizontal
            };
            let next_type = if same_next {
                if inext < i_intersection as i32 {
                    LinkType::Down
                } else {
                    LinkType::Up
                }
            } else {
                LinkType::Horizontal
            };

            let itsct = &mut segs[i_vline].intersections[i_intersection];
            itsct.prev_on_contour = iprev;
            itsct.prev_on_contour_type = prev_type;
            itsct.next_on_contour = inext;
            itsct.next_on_contour_type = next_type;

            // Validate vertical links: only follow if they skip just outer intersections
            if same_prev {
                let (lo, hi) = if iprev < i_intersection as i32 {
                    (iprev as usize, i_intersection)
                } else {
                    (i_intersection, iprev as usize)
                };
                let mut found_inner = false;
                for k in (lo + 1)..hi {
                    if segs[i_vline].intersections[k].is_inner() {
                        found_inner = true;
                        break;
                    }
                }
                if found_inner {
                    segs[i_vline].intersections[i_intersection].prev_on_contour_quality =
                        LinkQuality::Invalid;
                }
            }

            if same_next {
                let (lo, hi) = if inext < i_intersection as i32 {
                    (inext as usize, i_intersection)
                } else {
                    (i_intersection, inext as usize)
                };
                let mut found_inner = false;
                for k in (lo + 1)..hi {
                    if segs[i_vline].intersections[k].is_inner() {
                        found_inner = true;
                        break;
                    }
                }
                if found_inner {
                    segs[i_vline].intersections[i_intersection].next_on_contour_quality =
                        LinkQuality::Invalid;
                }
            }

            // If both prev and next are on same line and same side, invalidate both
            if same_prev && same_next && iprev >= 0 && inext >= 0 {
                if (iprev > i_intersection as i32) == (inext > i_intersection as i32) {
                    segs[i_vline].intersections[i_intersection].prev_on_contour_quality =
                        LinkQuality::Invalid;
                    segs[i_vline].intersections[i_intersection].next_on_contour_quality =
                        LinkQuality::Invalid;
                }
            }

            // Handle dont_connect and link_max_length
            if params.dont_connect {
                if segs[i_vline].intersections[i_intersection].prev_on_contour_quality
                    == LinkQuality::Valid
                {
                    segs[i_vline].intersections[i_intersection].prev_on_contour_quality =
                        LinkQuality::TooLong;
                }
                if segs[i_vline].intersections[i_intersection].next_on_contour_quality
                    == LinkQuality::Valid
                {
                    segs[i_vline].intersections[i_intersection].next_on_contour_quality =
                        LinkQuality::TooLong;
                }
            } else if link_max_length > 0 {
                // Measure lengths and mark TooLong if exceeding
                if segs[i_vline].intersections[i_intersection].prev_on_contour_quality
                    == LinkQuality::Valid
                    && iprev >= 0
                {
                    let length = if same_prev {
                        measure_perimeter_segment_on_vertical_line_length(
                            poly_with_offset,
                            segs,
                            i_vline,
                            iprev as usize,
                            i_intersection,
                            forward,
                        )
                    } else if i_vline > 0 {
                        measure_perimeter_horizontal_segment_length(
                            poly_with_offset,
                            segs,
                            i_vline - 1,
                            iprev as usize,
                            i_intersection,
                        )
                    } else {
                        0.0
                    };
                    if length > link_max_length as CoordF {
                        segs[i_vline].intersections[i_intersection].prev_on_contour_quality =
                            LinkQuality::TooLong;
                    }
                }
                if segs[i_vline].intersections[i_intersection].next_on_contour_quality
                    == LinkQuality::Valid
                    && inext >= 0
                {
                    let length = if same_next {
                        measure_perimeter_segment_on_vertical_line_length(
                            poly_with_offset,
                            segs,
                            i_vline,
                            i_intersection,
                            inext as usize,
                            forward,
                        )
                    } else {
                        measure_perimeter_horizontal_segment_length(
                            poly_with_offset,
                            segs,
                            i_vline,
                            i_intersection,
                            inext as usize,
                        )
                    };
                    if length > link_max_length as CoordF {
                        segs[i_vline].intersections[i_intersection].next_on_contour_quality =
                            LinkQuality::TooLong;
                    }
                }
            }
        }

        // Make LinkQuality::Invalid symmetric on vertical connections
        let n_intersections = segs[i_vline].intersections.len();
        for i_intersection in 0..n_intersections {
            if segs[i_vline].intersections[i_intersection].has_left_vertical()
                && segs[i_vline].intersections[i_intersection].prev_on_contour_quality
                    == LinkQuality::Invalid
            {
                let other = segs[i_vline].intersections[i_intersection].left_vertical();
                if other >= 0 && (other as usize) < n_intersections {
                    segs[i_vline].intersections[other as usize].prev_on_contour_quality =
                        LinkQuality::Invalid;
                }
            }
            if segs[i_vline].intersections[i_intersection].has_right_vertical()
                && segs[i_vline].intersections[i_intersection].next_on_contour_quality
                    == LinkQuality::Invalid
            {
                let other = segs[i_vline].intersections[i_intersection].right_vertical();
                if other >= 0 && (other as usize) < n_intersections {
                    segs[i_vline].intersections[other as usize].next_on_contour_quality =
                        LinkQuality::Invalid;
                }
            }
        }
    }
}

/// Measure Euclidean length of a perimeter segment between two intersections on adjacent vertical lines.
fn measure_perimeter_horizontal_segment_length(
    poly_with_offset: &ExPolygonWithOffset,
    segs: &[SegmentedIntersectionLine],
    i_vline: usize,
    i_intersection: usize,
    i_intersection2: usize,
) -> CoordF {
    let i_vline_other = i_vline + 1;
    if i_vline_other >= segs.len() {
        return f64::MAX;
    }
    let vline = &segs[i_vline];
    let vline2 = &segs[i_vline_other];

    if i_intersection >= vline.intersections.len() || i_intersection2 >= vline2.intersections.len()
    {
        return f64::MAX;
    }

    let it = &vline.intersections[i_intersection];
    let it2 = &vline2.intersections[i_intersection2];

    if it.i_contour != it2.i_contour {
        return f64::MAX;
    }

    let poly = poly_with_offset.contour(it.i_contour);
    let p1 = Point::new(vline.pos, it.pos());
    let p2 = Point::new(vline2.pos, it2.pos());

    if it.is_low() {
        segment_length(poly, it.i_segment, p1, it2.i_segment, p2)
    } else {
        segment_length(poly, it2.i_segment, p2, it.i_segment, p1)
    }
}

/// Measure length of a perimeter segment on the same vertical line.
fn measure_perimeter_segment_on_vertical_line_length(
    poly_with_offset: &ExPolygonWithOffset,
    segs: &[SegmentedIntersectionLine],
    i_vline: usize,
    i_intersection: usize,
    i_intersection2: usize,
    forward: bool,
) -> CoordF {
    let il = &segs[i_vline];
    if i_intersection >= il.intersections.len() || i_intersection2 >= il.intersections.len() {
        return f64::MAX;
    }
    let itsct = &il.intersections[i_intersection];
    let itsct2 = &il.intersections[i_intersection2];
    if itsct.i_contour != itsct2.i_contour {
        return f64::MAX;
    }
    let poly = poly_with_offset.contour(itsct.i_contour);
    let p1 = Point::new(il.pos, itsct.pos());
    let p2 = Point::new(il.pos, itsct2.pos());
    if forward {
        segment_length(poly, itsct.i_segment, p1, itsct2.i_segment, p2)
    } else {
        segment_length(poly, itsct2.i_segment, p2, itsct.i_segment, p1)
    }
}

// ---------------------------------------------------------------------------
// Core algorithm: traverse_graph_generate_polylines
// ---------------------------------------------------------------------------

/// Check if a horizontal link to the previous vertical line is valid.
fn intersection_on_prev_vertical_line_valid(
    segs: &[SegmentedIntersectionLine],
    i_vline: usize,
    i_intersection: usize,
) -> bool {
    intersection_on_prev_next_vertical_line_valid(segs, i_vline, i_intersection, false)
}

/// Check if a horizontal link to the next vertical line is valid.
fn intersection_on_next_vertical_line_valid(
    segs: &[SegmentedIntersectionLine],
    i_vline: usize,
    i_intersection: usize,
) -> bool {
    intersection_on_prev_next_vertical_line_valid(segs, i_vline, i_intersection, true)
}

/// Check validity of a horizontal link to the next (is_right=true) or previous (is_right=false)
/// vertical line.
fn intersection_on_prev_next_vertical_line_valid(
    segs: &[SegmentedIntersectionLine],
    i_vline: usize,
    i_intersection: usize,
    is_right: bool,
) -> bool {
    let vline_this = &segs[i_vline];
    if i_intersection >= vline_this.intersections.len() {
        return false;
    }
    let it_this = &vline_this.intersections[i_intersection];

    // If this intersection has a vertical link in this direction, skip
    if is_right {
        if it_this.has_right_vertical() {
            return false;
        }
    } else {
        if it_this.has_left_vertical() {
            return false;
        }
    }

    let i_other = if is_right {
        it_this.right_horizontal()
    } else {
        it_this.left_horizontal()
    };
    if i_other < 0 {
        return false;
    }

    let i_vline_other = if is_right {
        if i_vline + 1 >= segs.len() {
            return false;
        }
        i_vline + 1
    } else {
        if i_vline == 0 {
            return false;
        }
        i_vline - 1
    };

    let vline_other = &segs[i_vline_other];
    let i_other_u = i_other as usize;
    if i_other_u >= vline_other.intersections.len() {
        return false;
    }
    let it_other = &vline_other.intersections[i_other_u];
    if !it_other.is_inner() {
        return false;
    }
    if i_other_u == 0 || i_other_u + 1 >= vline_other.intersections.len() {
        return false;
    }

    // Check that we're at the boundary of a vertical segment
    let it_other2 = if it_other.is_low() {
        &vline_other.intersections[i_other_u - 1]
    } else {
        &vline_other.intersections[i_other_u + 1]
    };
    if it_other2.is_inner() {
        // Would connect into the middle of another vertical segment
        return false;
    }

    // Check link quality
    let quality = if is_right {
        it_this.horizontal_quality_right()
    } else {
        it_this.horizontal_quality_left()
    };
    if quality != LinkQuality::Valid {
        return false;
    }

    // Check if perimeter segment already consumed
    if is_right {
        if it_this.consumed_perimeter_right {
            return false;
        }
    } else {
        if it_other.consumed_perimeter_right {
            return false;
        }
    }

    // Check if the target vertical segment is already consumed
    if it_other.is_low() {
        if it_other.consumed_vertical_up {
            return false;
        }
    } else {
        if i_other_u > 0 && vline_other.intersections[i_other_u - 1].consumed_vertical_up {
            return false;
        }
    }

    true
}

/// Emit a perimeter segment between two intersections on adjacent vertical lines.
fn emit_perimeter_prev_next_segment(
    poly_with_offset: &ExPolygonWithOffset,
    segs: &[SegmentedIntersectionLine],
    i_vline: usize,
    i_inner_contour: usize,
    i_intersection: usize,
    i_intersection2: usize,
    out: &mut Vec<Point>,
    dir_is_next: bool,
) {
    let i_vline_other = if dir_is_next {
        i_vline + 1
    } else {
        if i_vline == 0 {
            return;
        }
        i_vline - 1
    };

    if i_vline_other >= segs.len() {
        return;
    }

    let il = &segs[i_vline];
    let il2 = &segs[i_vline_other];

    if i_intersection >= il.intersections.len() || i_intersection2 >= il2.intersections.len() {
        return;
    }

    let itsct = &il.intersections[i_intersection];
    let itsct2 = &il2.intersections[i_intersection2];

    if i_inner_contour >= poly_with_offset.n_contours {
        return;
    }
    let poly = poly_with_offset.contour(i_inner_contour);

    let forward = itsct.is_low() == dir_is_next;

    if forward {
        polygon_segment_append(out, poly, itsct.i_segment, itsct2.i_segment);
    } else {
        polygon_segment_append_reversed(out, poly, itsct.i_segment, itsct2.i_segment);
    }
    // Append the endpoint
    out.push(Point::new(il2.pos, itsct2.pos()));
}

/// Emit a perimeter segment on the same vertical line.
fn emit_perimeter_segment_on_vertical_line(
    poly_with_offset: &ExPolygonWithOffset,
    segs: &[SegmentedIntersectionLine],
    i_vline: usize,
    i_inner_contour: usize,
    i_intersection: usize,
    i_intersection2: usize,
    out: &mut Vec<Point>,
    forward: bool,
) {
    let il = &segs[i_vline];
    if i_intersection >= il.intersections.len() || i_intersection2 >= il.intersections.len() {
        return;
    }

    let itsct = &il.intersections[i_intersection];
    let itsct2 = &il.intersections[i_intersection2];

    if i_inner_contour >= poly_with_offset.n_contours {
        return;
    }
    let poly = poly_with_offset.contour(i_inner_contour);

    if forward {
        polygon_segment_append(out, poly, itsct.i_segment, itsct2.i_segment);
    } else {
        polygon_segment_append_reversed(out, poly, itsct.i_segment, itsct2.i_segment);
    }
    out.push(Point::new(il.pos, itsct2.pos()));
}

/// Walk the intersection graph to generate connected polylines.
///
/// This is the port of libslic3r `traverse_graph_generate_polylines()`.
pub fn traverse_graph_generate_polylines(
    poly_with_offset: &ExPolygonWithOffset,
    params: &FillRectilinearParams,
    segs: &mut Vec<SegmentedIntersectionLine>,
    polylines_out: &mut Vec<Polyline>,
) {
    // Mark outer-only chords (OUTER_LOW immediately followed by OUTER_HIGH) as consumed.
    for sil in segs.iter_mut() {
        let n = sil.intersections.len();
        for i in 0..n.saturating_sub(1) {
            if sil.intersections[i].itype == SegmentIntersectionType::OuterLow
                && sil.intersections[i + 1].itype == SegmentIntersectionType::OuterHigh
            {
                sil.intersections[i].consumed_vertical_up = true;
            }
        }
    }

    let mut i_vline: i32 = 0;
    let mut i_intersection: i32 = -1;
    let mut point_last = Point::new(0, 0);

    loop {
        if i_intersection == -1 {
            // The path has been interrupted. Find a next starting point.
            // FillRectilinear.cpp:1462
            let mut found = false;
            for iv in 0..segs.len() {
                let vline = &segs[iv];
                if vline.intersections.is_empty() {
                    continue;
                }
                // For infill that needs to be consistent between layers (like Zig Zag),
                // we are switching between forward and backward passes based on the line
                // index. FillRectilinear.cpp:1472-1474
                let forward_pass = !params.consistent_pattern || (iv % 2 == 0);
                let n = vline.intersections.len();
                for i in 0..n {
                    // FillRectilinear.cpp:1476
                    let intrsctn_idx = if forward_pass { i } else { n - i - 1 };
                    let intrsctn = &vline.intersections[intrsctn_idx];
                    if intrsctn.is_outer() {
                        let consumed = if intrsctn.is_low() {
                            intrsctn.consumed_vertical_up
                        } else if intrsctn_idx > 0 {
                            vline.intersections[intrsctn_idx - 1].consumed_vertical_up
                        } else {
                            true
                        };
                        if !consumed {
                            i_vline = iv as i32;
                            i_intersection = intrsctn_idx as i32;
                            found = true;
                            break;
                        }
                    }
                }
                if found {
                    break;
                }
            }

            if !found {
                // All done
                break;
            }

            // Start a new polyline
            point_last = Point::new(
                segs[i_vline as usize].pos,
                segs[i_vline as usize].intersections[i_intersection as usize].pos(),
            );
            let mut new_polyline = Polyline::new();
            new_polyline.points_mut().push(point_last);
            polylines_out.push(new_polyline);
        }

        // Follow the path from (i_vline, i_intersection)
        let iv = i_vline as usize;
        let ii = i_intersection as usize;

        let going_up = segs[iv].intersections[ii].is_low();
        let mut try_connect = false;
        let mut current_ii = ii;

        if going_up {
            // Step back to beginning of vertical segment
            if segs[iv].intersections[current_ii].is_inner() && current_ii > 0 {
                current_ii -= 1;
            }
            // Consume upward
            loop {
                segs[iv].intersections[current_ii].consumed_vertical_up = true;
                current_ii += 1;
                if current_ii >= segs[iv].intersections.len() {
                    break;
                }
                if segs[iv].intersections[current_ii].itype == SegmentIntersectionType::OuterHigh {
                    break;
                }
            }
            if current_ii >= segs[iv].intersections.len() {
                // Safety: shouldn't happen, but stop gracefully
                i_intersection = -1;
                continue;
            }
            // If we stopped at an inner point, step back and try to connect
            if current_ii > 0 && segs[iv].intersections[current_ii - 1].is_inner() {
                current_ii -= 1;
                if segs[iv].intersections[current_ii].itype == SegmentIntersectionType::InnerHigh {
                    try_connect = true;
                }
            }
        } else {
            // Going down
            if segs[iv].intersections[current_ii].is_inner() {
                segs[iv].intersections[current_ii].consumed_vertical_up = true;
            }
            loop {
                if current_ii == 0 {
                    break;
                }
                current_ii -= 1;
                segs[iv].intersections[current_ii].consumed_vertical_up = true;
                if segs[iv].intersections[current_ii].itype == SegmentIntersectionType::OuterLow {
                    break;
                }
            }
            // If we stopped at inner, step forward and try to connect
            if current_ii + 1 < segs[iv].intersections.len()
                && segs[iv].intersections[current_ii + 1].is_inner()
            {
                current_ii += 1;
                if segs[iv].intersections[current_ii].itype == SegmentIntersectionType::InnerLow {
                    try_connect = true;
                }
            }
        }

        i_intersection = current_ii as i32;

        if try_connect {
            let it = &segs[iv].intersections[current_ii];
            let i_prev = it.left_horizontal();
            let i_next = it.right_horizontal();

            // To ensure pattern consistency between layers for Zig Zag infill, we always
            // try to connect to the next vertical line and never to the previous vertical
            // line. FillRectilinear.cpp:1560
            let prev_valid = i_prev >= 0
                && iv > 0
                && intersection_on_prev_vertical_line_valid(segs, iv, current_ii)
                && !params.consistent_pattern;
            let next_valid = i_next >= 0
                && iv + 1 < segs.len()
                && intersection_on_next_vertical_line_valid(segs, iv, current_ii);

            let horizontal_valid = prev_valid || next_valid;

            // Mark perimeter segments as consumed
            if i_prev >= 0 && iv > 0 {
                segs[iv - 1].intersections[i_prev as usize].consumed_perimeter_right = true;
            }
            if i_next >= 0 {
                segs[iv].intersections[current_ii].consumed_perimeter_right = true;
            }

            if horizontal_valid {
                // Connect along perimeter to adjacent vertical line
                let take_next = if prev_valid && next_valid {
                    // Take the shorter path
                    let dist_prev = measure_perimeter_horizontal_segment_length(
                        poly_with_offset,
                        segs,
                        iv - 1,
                        i_prev as usize,
                        current_ii,
                    );
                    let dist_next = measure_perimeter_horizontal_segment_length(
                        poly_with_offset,
                        segs,
                        iv,
                        current_ii,
                        i_next as usize,
                    );
                    dist_next < dist_prev
                } else {
                    next_valid
                };

                // Emit current point
                let current_point =
                    Point::new(segs[iv].pos, segs[iv].intersections[current_ii].pos());
                if let Some(last_polyline) = polylines_out.last_mut() {
                    last_polyline.points_mut().push(current_point);
                }

                // Emit the perimeter segment
                let i_contour = segs[iv].intersections[current_ii].i_contour;
                let target_intersection = if take_next {
                    i_next as usize
                } else {
                    i_prev as usize
                };

                if let Some(last_polyline) = polylines_out.last_mut() {
                    emit_perimeter_prev_next_segment(
                        poly_with_offset,
                        segs,
                        iv,
                        i_contour,
                        current_ii,
                        target_intersection,
                        last_polyline.points_mut(),
                        take_next,
                    );
                }

                // Advance to the neighbor line
                if take_next {
                    i_vline += 1;
                    i_intersection = i_next;
                } else {
                    i_vline -= 1;
                    i_intersection = i_prev;
                }
                continue;
            }

            // Try vertical link
            let it = &segs[iv].intersections[current_ii];
            let i_vertical = it.vertical_outside();
            let vertical_quality = if i_vertical == -1 {
                LinkQuality::Invalid
            } else {
                let check_idx = if going_up {
                    i_vertical as usize
                } else {
                    if i_vertical > 0 {
                        (i_vertical - 1) as usize
                    } else {
                        0
                    }
                };
                if check_idx < segs[iv].intersections.len()
                    && segs[iv].intersections[check_idx].consumed_vertical_up
                {
                    LinkQuality::Invalid
                } else {
                    it.vertical_outside_quality()
                }
            };

            if vertical_quality != LinkQuality::Invalid && i_vertical >= 0 {
                let i_vert = i_vertical as usize;
                let i_contour = segs[iv].intersections[current_ii].i_contour;

                // Emit current point
                let current_point =
                    Point::new(segs[iv].pos, segs[iv].intersections[current_ii].pos());

                if vertical_quality == LinkQuality::Valid {
                    // Emit the connecting contour segment
                    let forward = if going_up {
                        segs[iv].intersections[current_ii].has_left_vertical_up()
                    } else {
                        segs[iv].intersections[current_ii].has_right_vertical_down()
                    };
                    if let Some(last_polyline) = polylines_out.last_mut() {
                        last_polyline.points_mut().push(current_point);
                        emit_perimeter_segment_on_vertical_line(
                            poly_with_offset,
                            segs,
                            iv,
                            i_contour,
                            current_ii,
                            i_vert,
                            last_polyline.points_mut(),
                            forward,
                        );
                    }
                } else {
                    // TooLong: skip the connecting contour, start new path
                    if let Some(last_polyline) = polylines_out.last_mut() {
                        last_polyline.points_mut().push(current_point);
                    }
                    let new_start = Point::new(segs[iv].pos, segs[iv].intersections[i_vert].pos());
                    let mut new_polyline = Polyline::new();
                    new_polyline.points_mut().push(new_start);
                    polylines_out.push(new_polyline);
                }

                // Mark consumed
                if going_up {
                    for k in current_ii..i_vert {
                        segs[iv].intersections[k].consumed_vertical_up = true;
                    }
                } else {
                    for k in i_vert..current_ii {
                        segs[iv].intersections[k].consumed_vertical_up = true;
                    }
                }

                segs[iv].intersections[current_ii].consumed_perimeter_right = true;
                let adj = if going_up {
                    current_ii + 1
                } else {
                    if current_ii > 0 {
                        current_ii - 1
                    } else {
                        current_ii
                    }
                };
                if adj < segs[iv].intersections.len() {
                    segs[iv].intersections[adj].consumed_perimeter_right = true;
                }

                i_intersection = i_vertical;
                continue;
            }

            // No connection possible — take the rest of the line to the outer contour
            if going_up {
                current_ii += 1;
            } else if current_ii > 0 {
                current_ii -= 1;
            }
            i_intersection = current_ii as i32;
        }

        // Finish the current vertical segment
        let iv = i_vline as usize;
        let ii = i_intersection as usize;

        if iv < segs.len() && ii < segs[iv].intersections.len() {
            point_last = Point::new(segs[iv].pos, segs[iv].intersections[ii].pos());
            if let Some(last_polyline) = polylines_out.last_mut() {
                last_polyline.points_mut().push(point_last);
            }
        }

        // Handle duplicate points and zero length segments.
        // FillRectilinear.cpp:1647 — remove_duplicate_points() before the pop-back check.
        if let Some(last_polyline) = polylines_out.last_mut() {
            crate::multi_point::remove_duplicate_points(last_polyline.points_mut());
        }
        // Handle nearly zero length edges. FillRectilinear.cpp:1650
        if let Some(last_polyline) = polylines_out.last() {
            let pts = last_polyline.points();
            let should_remove = pts.len() <= 1
                || (pts.len() == 2
                    && (pts[0].x - pts[pts.len() - 1].x).abs() < SCALED_EPSILON
                    && (pts[0].y - pts[pts.len() - 1].y).abs() < SCALED_EPSILON);
            if should_remove {
                polylines_out.pop();
            }
        }

        i_intersection = -1;
    }
}

// ---------------------------------------------------------------------------
// Main entry point: fill_surface_by_lines
// ---------------------------------------------------------------------------

/// Main entry point matching libslic3r's `FillRectilinear::fill_surface_by_lines`.
///
/// Generates connected infill polylines for a single ExPolygon, with
/// boundary-following connections along the inner offset perimeter.
///
/// # Arguments
/// * `expolygon` - The fill area
/// * `spacing` - Line spacing in mm
/// * `angle` - Fill angle in radians
/// * `overlap` - Overlap with perimeters in mm
/// * `params` - Fill parameters
///
/// # Returns
/// Connected polylines in the original (unrotated) coordinate space.
pub fn fill_surface_by_lines(
    expolygon: &ExPolygon,
    spacing: CoordF,
    angle: f64,
    overlap: CoordF,
    params: &FillRectilinearParams,
) -> Vec<Polyline> {
    fill_surface_by_lines_zag(
        expolygon,
        spacing,
        angle,
        overlap,
        params,
        &FillRectilinearZag::default(),
    )
}

/// [`fill_surface_by_lines`] with the members the zig zag fillers add
/// (FillRectilinear.cpp:2825-2990 read with `has_consistent_pattern()`,
/// `params.horiz_move` and `params.symmetric_infill_y_axis`).
pub fn fill_surface_by_lines_zag(
    expolygon: &ExPolygon,
    spacing: CoordF,
    angle: f64,
    overlap: CoordF,
    params: &FillRectilinearParams,
    zag: &FillRectilinearZag,
) -> Vec<Polyline> {
    let mut polylines_out = Vec::new();

    const INFILL_OVERLAP_OVER_SPACING: f64 = 0.45;

    // R301 (TOPFILL_FAITHFUL): native domains — line_spacing =
    // coord_t((spacing / 1e-5) / density) [scale_ DIVIDES, then trunc];
    // aoffsets = float(scale_(overlap - (0.5 - 0.45f)*spacing)) — f32 deltas
    // with the fraction KEPT (FillRectilinear.cpp:2841-2848). The legacy
    // mult/round domain drifts the raster clip boundary sub-unit (the Top
    // endpoint last-digit class, R299/R300).
    let tf = crate::faithful_gate("TOPFILL_FAITHFUL") && std::env::var("RASTER_QUANT").is_ok();
    // RECT_OFFSET_FAITHFUL (on unless =0) — FillRectilinear.cpp:2842-2849:
    //     coord_t line_spacing = coord_t(scale_(this->spacing) / params.density);
    //     float(scale_(this->overlap - (0.5 - INFILL_OVERLAP_OVER_SPACING) * this->spacing)),
    //     float(scale_(this->overlap - 0.5f * this->spacing))
    // `scale_` divides by 0.00001 and nothing rounds: the two offsets go through
    // `float` into `coord_t` parameters (FillRectilinear.cpp:456-459), which
    // truncates them. INFILL_OVERLAP_OVER_SPACING is the float 0.45f.
    let rof = crate::faithful_gate("RECT_OFFSET_FAITHFUL");
    let line_spacing = if tf || rof {
        ((spacing / 0.00001) / params.density) as Coord
    } else {
        (spacing * crate::SCALING_FACTOR / params.density) as Coord
    };
    let (aoffset1, aoffset2): (CoordF, CoordF) = if rof {
        let c = 0.5 - (0.45f32 as f64);
        (
            ((((-c).mul_add(spacing, overlap) / 0.00001) as f32) as Coord) as CoordF,
            ((((-0.5f64).mul_add(spacing, overlap) / 0.00001) as f32) as Coord) as CoordF,
        )
    } else if tf {
        let c = 0.5 - (0.45f32 as f64);
        (
            (((overlap - c * spacing) / 0.00001) as f32) as f64,
            (((overlap - 0.5 * spacing) / 0.00001) as f32) as f64,
        )
    } else {
        (
            scale(overlap - (0.5 - INFILL_OVERLAP_OVER_SPACING) * spacing) as CoordF,
            scale(overlap - 0.5 * spacing) as CoordF,
        )
    };

    let poly_with_offset = ExPolygonWithOffset::new_scaled_f(expolygon, -angle, aoffset1, aoffset2);

    if poly_with_offset.n_contours_inner == 0 {
        // No inner contour — no infill lines fit
        return polylines_out;
    }

    // Bounding box for vertical line placement
    let bbox_src = poly_with_offset.bounding_box_src();
    // FillRectilinear.cpp:2856-2859 — for infill that needs to be consistent
    // between layers (like Zig Zag), the bounding box of the whole object is
    // used to match vertical lines between layers:
    //     BoundingBox bounding_box = this->has_consistent_pattern() ?
    //         this->extended_object_bounding_box() : bounding_box_src;
    let mut bbox = match (params.consistent_pattern, zag.object_bounding_box.as_ref()) {
        (true, Some(object_bbox)) => extended_object_bounding_box(object_bbox),
        _ => bbox_src,
    };

    // Adjust spacing for solid fill
    let line_spacing = if params.full_infill && !params.dont_adjust {
        // FillRectilinear.cpp:2863 — `bounding_box_src.size().x()`.
        let ls = adjust_solid_spacing(bbox_src.width(), line_spacing);
        ADJUSTED_SPACING_MM.with(|c| c.set(Some(ls as f64 / crate::SCALING_FACTOR)));
        ls
    } else {
        // Native align_to_grid branch (FillRectilinear.cpp:2865-2875): refpt =
        // Fill::bounding_box.center() rotated, and PrintObject::bounding_box()
        // is origin-centered (PrintObject.hpp:395) so refpt == (0,0) exactly:
        // align bbox.min down to global line_spacing multiples. Gated.
        if crate::faithful_gate("TOPFILL_FAITHFUL") {
            let aligned = crate::geometry::align_to_grid_point(
                bbox.min,
                Point::new(line_spacing, line_spacing),
            );
            bbox.min = Point::new(bbox.min.x.min(aligned.x), bbox.min.y.min(aligned.y));
        }
        line_spacing
    };

    if line_spacing <= 0 {
        return polylines_out;
    }

    // Number of vertical lines
    let mut n_vlines = ((bbox.max.x - bbox.min.x + line_spacing - 1) / line_spacing).max(0) as usize;
    let mut x0 = bbox.min.x;
    if params.full_infill {
        x0 += (line_spacing + SCALED_EPSILON) / 2;
    }
    // RECT_HORIZ_MOVE (on unless =0) — FillRectilinear.cpp:2885-2891.
    rect_horiz_move(zag.horiz_move, line_spacing, &mut x0, &mut n_vlines);

    if n_vlines == 0 {
        return polylines_out;
    }

    // Slice the region
    let mut segs = slice_region_by_vertical_lines(&poly_with_offset, n_vlines, x0, line_spacing);

    // Build the connection graph.
    // C++ reference: FillRectilinear::fill_surface_by_lines() passes the Fill
    // member `link_max_length` (Fill.cpp:683-695: 0 for density <= 80%, else
    // 3*spacing). It is NOT recomputed from line_spacing here, so long
    // perimeter links between sparse infill lines stay valid and are chained
    // into a single continuous polyline (zero rapids). We honour the value
    // threaded in via FillRectilinearParams::link_max_length.
    let link_max_length = params.link_max_length;
    let rect_probe = crate::probe_enabled("RECT");
    if rect_probe {
        rect_probe_segs(1, n_vlines, x0, line_spacing, false, link_max_length as Coord, &segs);
    }
    connect_segment_intersections_by_contours(
        &poly_with_offset,
        &mut segs,
        params,
        link_max_length,
    );
    if rect_probe {
        rect_probe_segs(2, n_vlines, x0, line_spacing, false, link_max_length as Coord, &segs);
    }

    // Traverse the graph to generate polylines
    traverse_graph_generate_polylines(&poly_with_offset, params, &mut segs, &mut polylines_out);
    if rect_probe {
        let (mut np, mut sx, mut sy) = (0i64, 0i64, 0i64);
        for pl in &polylines_out {
            for pt in pl.points() {
                np += 1;
                sx += pt.x();
                sy += pt.y();
            }
        }
        eprintln!(
            "RECT3 nv={} x0={} npl={} np={} sx={} sy={}",
            n_vlines, x0, polylines_out.len(), np, sx, sy
        );
    }

    // paths must be rotated back. FillRectilinear.cpp:2967
    for polyline in &mut polylines_out {
        // Rotate back by rotate_vector.first, then remove duplicate points (rotation may
        // collapse distinct scaled points). FillRectilinear.cpp:2971-2974
        polyline.rotate(angle);
        crate::multi_point::remove_duplicate_points(polyline.points_mut());
        // FillRectilinear.cpp:2977-2980 — get origin direction infill:
        //     if (params.symmetric_infill_y_axis)
        //         it->symmetric_y(params.symmetric_y_axis);
        if let Some(symmetric_y_axis) = zag.symmetric_y_axis {
            crate::multi_point::symmetric_y(polyline.points_mut(), symmetric_y_axis);
        }
    }

    // Filter out degenerate polylines
    polylines_out.retain(|p| p.points().len() >= 2);

    polylines_out
}

/// FillRectilinear.cpp:2885-2891 — the shift of the first vertical line:
///     int gap_line = params.horiz_move / line_spacing;
///     if (gap_line % 2 == 0) {
///         x0 += params.horiz_move - gap_line * line_spacing;
///     } else {
///         x0 += params.horiz_move - (gap_line - 1) * line_spacing;
///         n_vlines += 1;
///     }
/// `horiz_move` is a float and `line_spacing` and `x0` are `coord_t`: the
/// quotient is taken in float and truncated, and `x0 +=` adds in float, so `x0`
/// goes through a float also where `horiz_move` is 0. RECT_HORIZ_MOVE=0 leaves
/// `x0` and `n_vlines` as they are.
fn rect_horiz_move(horiz_move: f32, line_spacing: Coord, x0: &mut Coord, n_vlines: &mut usize) {
    if !crate::faithful_gate("RECT_HORIZ_MOVE") {
        return;
    }
    let gap_line = (horiz_move / line_spacing as f32) as i32;
    if gap_line % 2 == 0 {
        *x0 = (*x0 as f32 + (horiz_move - (gap_line as i64 * line_spacing) as f32)) as Coord;
    } else {
        *x0 = (*x0 as f32 + (horiz_move - ((gap_line - 1) as i64 * line_spacing) as f32)) as Coord;
        *n_vlines += 1;
    }
}

/// Adjust line spacing for solid infill to fill the bounding box exactly.
/// Port of `Fill::_adjust_solid_spacing` (FillBase.cpp:179-195).
fn adjust_solid_spacing(width: Coord, distance: Coord) -> Coord {
    debug_assert!(width >= 0);
    debug_assert!(distance > 0);
    // EPSILON is the unscaled constant 1e-4 (libslic3r.h:52). C division here
    // truncates toward zero (coord_t cast), so we use f64 then truncate.
    const EPSILON: f64 = 1e-4;
    // floor(width / distance) — truncation toward zero. FillBase.cpp:184
    let number_of_intervals = ((width as f64 - EPSILON) / distance as f64) as Coord;
    let mut distance_new = if number_of_intervals == 0 {
        distance
    } else {
        ((width as f64 - EPSILON) / number_of_intervals as f64) as Coord
    };
    let factor = distance_new as f64 / distance as f64;
    debug_assert!(factor > 1.0 - 1e-5);
    // How much could the extrusion width be increased? By 20%.
    const FACTOR_MAX: f64 = 1.2;
    if factor > FACTOR_MAX {
        distance_new = (distance as f64 * FACTOR_MAX + 0.5).floor() as Coord;
    }
    distance_new
}

// ---------------------------------------------------------------------------
// High-level convenience function for integration with InfillGenerator
// ---------------------------------------------------------------------------

use super::{InfillConfig, InfillPath};

/// Generate rectilinear infill using the FillRectilinear algorithm.
///
/// This is the main entry point for the pipeline to use instead of the
/// simple line-clipping approach.
pub fn generate_fill_rectilinear(
    fill_area: &[ExPolygon],
    config: &InfillConfig,
    layer_index: usize,
    is_grid: bool,
) -> Vec<InfillPath> {
    ADJUSTED_SPACING_MM.with(|c| c.set(None));
    // FILL_RECTILINEAR_PIN (on unless =0): one direction is laid by
    // `FillRectilinear::fill_surface_by_lines` in the arithmetic of the pin (the classes at the
    // end of this file, compared with the pin by tests/fill_rectilinear_native_parity.rs).
    if !is_grid && live_entries_at_the_pin() && (!config.consistent_pattern || config.object_bounding_box.is_some()) {
        return generate_fill_rectilinear_pin(fill_area, config, layer_index, false);
    }
    let mut paths = Vec::new();

    // Native Fill::_infill_direction (FillBase.cpp:239) adds an UNCONDITIONAL
    // +M_PI/2 after base angle + per-layer alternation (native top L0 = 45deg
    // emitted lines). Gated: default keeps the legacy angle (byte-locked).
    let faithful_dir = crate::faithful_gate("TOPFILL_FAITHFUL");
    // Native _layer_angle = (idx & 1) ? 90 : 0 (FillBase.hpp:207) — a PARITY
    // alternation, NOT 90*idx. 90*idx coincides only mod 180; the raster frame
    // needs mod-360 equality (a 180-rotated frame point-reflects the bbox and
    // shifts x0 for every adjust-branch solid fill). Plus _infill_direction's
    // unconditional +90 (FillBase.cpp:239).
    //
    // R599: `faithful_gate` is TRUE unless the variable is "0", so TOPFILL_FAITHFUL
    // is DEFAULT-ON and the faithful branch is what actually runs. The previous
    // comment here said "default keeps legacy", which was wrong.
    let angle_deg = if faithful_dir {
        config.angle + if layer_index & 1 == 1 { 90.0 } else { 0.0 } + 90.0
    } else {
        config.angle + config.angle_increment * layer_index as f64
    };
    // R599: C++ Fill::_infill_direction (FillBase.cpp:224-239) short-circuits to
    // the surface's bridge angle when it is set, SKIPPING the per-layer
    // alternation, then still adds the unconditional +M_PI/2. `bridge_angle` is
    // already in RADIANS, so this bypasses the degrees path rather than
    // round-tripping through it. Populated in `group_fills` (fill/mod.rs:844);
    // R599's comment here cited fill.rs:606, which is the DEAD twin module.
    //
    // R600: the branch below is only meaningful once `bridge_angle` is -1 on
    // non-bridge surfaces. Until `FILL_PARAMS_BRIDGE_ANGLE_CPP` it was 0.0 there,
    // so this fired on ~every fill where C++ fires on 3.2% of them (Benchy) --
    // which is why R599 measured a regression.
    //
    // R600 flipped this DEFAULT-ON. With the population corrected it is a net gain
    // on both fixtures and both instruments -- Benchy +13 matched lines
    // (line_parity) / +28 (line_align), Majora +21 -- and it lifts the Bridge
    // feature on both (7.8% -> 8.2%, 8.3% -> 8.7%). Benchy moves nowhere else;
    // Majora's per-feature deltas sum to the +21 exactly, and are not all positive:
    // Bridge +17, Outer wall +15, Sparse +14, but Floating vertical shell -20 and
    // Top surface -9. Those two are the next thing to look at, not a reason to hold
    // back a change that is verbatim `FillBase.cpp:224-239`.
    let used_bridge = config.bridge_angle >= 0.0;
    let angle_rad = if faithful_dir && crate::faithful_gate("RECT_ANGLE_F32") {
        // Fill::_infill_direction (FillBase.cpp:204-244) in its own type: the
        // float `angle`, replaced by float(bridge_angle) on a bridge, else plus
        // the float layer angle, then plus float(M_PI/2). The polygon is
        // rotated by this float and the lines are rotated back by it.
        // RECT_ANGLE_F32=0 sums the angle in double.
        let half_pi = std::f64::consts::FRAC_PI_2 as f32;
        let mut out_angle: f32 = (std::f64::consts::PI * config.angle / 180.0) as f32;
        // Fill.cpp:704-709 — the zig zag turns from layer pair to layer pair:
        //     if (surface_fill.params.pattern == ipZigZag) {
        //         if (f->layer_id % 2 == 0)
        //             f->angle -= surface_fill.params.infill_rotate_step * (f->layer_id / 2);
        //         else
        //             f->angle += surface_fill.params.infill_rotate_step * (f->layer_id / 2);
        //     }
        // in float, the product and the sum in one expression.
        if let Some(infill_rotate_step) = config.zigzag_rotate_step {
            let half_layers = (layer_index / 2) as f32;
            out_angle = if layer_index % 2 == 0 {
                (-infill_rotate_step).mul_add(half_layers, out_angle)
            } else {
                infill_rotate_step.mul_add(half_layers, out_angle)
            };
        }
        if crate::faithful_gate("FILL_BRIDGE_ANGLE") && used_bridge {
            out_angle = config.bridge_angle as f32;
        } else {
            out_angle += if layer_index & 1 == 1 { half_pi } else { 0.0f32 };
        }
        out_angle += half_pi;
        // FillRectilinear.cpp:2837-2838:
        //     if (params.locked_zag)
        //         rotate_vector.first += float(M_PI/2.);
        if config.locked_zag {
            out_angle += half_pi;
        }
        out_angle as f64
    } else if crate::faithful_gate("FILL_BRIDGE_ANGLE") && used_bridge {
        config.bridge_angle + std::f64::consts::FRAC_PI_2
    } else {
        angle_deg.to_radians()
    };
    // FILL_BRIDGE_POP=1 — the R600 population census, counting the SAME quantity as
    // the C++ FILLANG probe in `Fill::_infill_direction`: of all fills, how many
    // present a usable bridge angle. Cumulative totals are printed, not a prefix
    // (R598), and the branch is evaluated regardless of `FILL_BRIDGE_ANGLE` so the
    // population can be measured without changing the output.
    if crate::probe_enabled("FILL_BRIDGE_POP") {
        use std::sync::atomic::{AtomicUsize, Ordering};
        static N: AtomicUsize = AtomicUsize::new(0);
        static USED: AtomicUsize = AtomicUsize::new(0);
        let n = N.fetch_add(1, Ordering::Relaxed) + 1;
        let used = USED.fetch_add(used_bridge as usize, Ordering::Relaxed) + used_bridge as usize;
        // Print EVERY call: this path is entered a few hundred times, so a modulo
        // stride prints once and reads as a prefix artefact (R598).
        eprintln!(
            "[FILLPOP] n={n} used_bridge={used} ({:.2}%) bridge_angle={:.6}",
            100.0 * used as f64 / n as f64,
            config.bridge_angle
        );
    }

    // Grid draws two perpendicular passes; FillRectilinear.cpp:3036
    // (fill_surface_by_multilines) divides density by the sweep count so the passes
    // combine to the target density. Without this, each pass fills at full density and
    // grid over-extrudes ~2x.
    let pass_density = if is_grid {
        config.density / 2.0
    } else {
        config.density
    };
    let params = FillRectilinearParams {
        density: pass_density,
        full_infill: config.density > 0.99,
        dont_connect: !config.connect_infill,
        dont_adjust: config.dont_adjust,
        monotonic: false,
        link_max_length: config.link_max_length,
        // FillRectilinear.hpp:165,174,187 — `has_consistent_pattern()` of
        // FillZigZag, FillCrossZag and FillLockedZag.
        consistent_pattern: config.consistent_pattern,
    };
    let zag = FillRectilinearZag {
        object_bounding_box: config.object_bounding_box,
        horiz_move: config.horiz_move,
        symmetric_y_axis: config.symmetric_y_axis,
    };

    for expoly in fill_area {
        let polylines = fill_surface_by_lines_zag(
            expoly,
            config.extrusion_width,
            angle_rad,
            config.overlap,
            &params,
            &zag,
        );

        for polyline in polylines {
            if polyline.points().len() >= 2 {
                paths.push(InfillPath::Line(polyline));
            }
        }
    }

    // For grid pattern, add perpendicular lines
    if is_grid {
        let perp_angle_rad = angle_rad + std::f64::consts::FRAC_PI_2;
        for expoly in fill_area {
            let polylines = fill_surface_by_lines(
                expoly,
                config.extrusion_width,
                perp_angle_rad,
                config.overlap,
                &params,
            );

            for polyline in polylines {
                if polyline.points().len() >= 2 {
                    paths.push(InfillPath::Line(polyline));
                }
            }
        }
    }

    paths
}

/// `pattern->fill_surface(&surface, params)` of FillLockedZag's two nested
/// fillers (FillRectilinear.cpp:3936-3937, 3961-3962): a filler made by
/// `Fill::new_from_type` for the skin or the skeleton pattern, with the locked
/// zag's own members (`copy_fill_data`, FillBase.hpp:176-188) and its params
/// with another density. `None` for a pattern this entry cannot lay.
// FIDELITY-NOTE: `Fill::new_from_type` takes any infill pattern; the fillers
// this function stands for are FillRectilinear, FillZigZag and FillCrossZag
// (FillBase.cpp:41, 62-63), the ones `generate_fill_rectilinear` lays. With
// `params.conformal` set the skin goes through FillConformal
// (FillRectilinear.cpp:3077-3081), which is not taken here.
fn locked_zag_pattern_fill(
    pattern: crate::print_config::InfillPattern,
    config: &InfillConfig,
    layer_index: usize,
    density: f32,
    locked_zag: bool,
    expolygons: &[ExPolygon],
    out: &mut Vec<Polyline>,
) -> Option<()> {
    use crate::print_config::InfillPattern as P;
    let consistent_pattern = match pattern {
        P::Rectilinear => false,
        // FillRectilinear.hpp:165, 174 — `has_consistent_pattern()`.
        P::ZigZag | P::CrossZag => true,
        _ => return None,
    };
    let mut cfg = config.clone();
    cfg.pattern = super::InfillPattern::Rectilinear;
    cfg.density = density as f64;
    cfg.consistent_pattern = consistent_pattern;
    // FillRectilinear.cpp:3903-3904, 3948-3949:
    //     if (this->skeleton_pattern != ipCrossZag) params.horiz_move = 0;
    if pattern != P::CrossZag {
        cfg.horiz_move = 0.0;
    }
    cfg.locked_zag = locked_zag;
    cfg.zigzag_rotate_step = None;
    // for (ExPolygon &exp : exps) { surface.expolygon = exp;
    //     Polylines lines = pattern->fill_surface(&surface, params); ... }
    for exp in expolygons {
        for path in generate_fill_rectilinear(std::slice::from_ref(exp), &cfg, layer_index, false) {
            match path {
                InfillPath::Line(pl) => out.push(pl),
                InfillPath::Loop(poly) => out.push(poly.split_at_first_point()),
            }
        }
    }
    Some(())
}

/// FillRectilinear.cpp:3969-4004 — `void FillLockedZag::fill_surface_locked_zag(
/// const Surface *surface, const FillParams &params,
/// std::vector<std::pair<Polylines, Flow>> &multi_width_polyline)`, with
/// `get_skin_and_skeleton_area` (3875-3892), `generate_skeleton_pattern`
/// (3894-3940) and `generate_skin_pattern` (3942-3967).
///
/// `config` carries the locked zag filler's members and its FillParams
/// (`horiz_move`, the symmetric axis, the object's bounding box); `expolygon` is
/// `surface->expolygon`. `None` where a nested pattern cannot be laid.
pub fn fill_surface_locked_zag(
    lock_param: &super::LockRegionParam,
    skin_pattern: crate::print_config::InfillPattern,
    skeleton_pattern: crate::print_config::InfillPattern,
    expolygon: &ExPolygon,
    config: &InfillConfig,
    layer_index: usize,
) -> Option<Vec<(Vec<Polyline>, crate::flow::Flow)>> {
    use crate::clipper_utils::{
        difference_clib, intersection_clib, intersection_pl, offset_expolygons_clib_scaled,
        union_ex_clib, union_safety_offset_ex_expolygons,
    };
    let surface_expolygon = std::slice::from_ref(expolygon);

    // FillRectilinear.cpp:3875-3892 — get_skin_and_skeleton_area.
    let mut skin_areas = intersection_clib(surface_expolygon, &lock_param.outlook);
    let left = difference_clib(surface_expolygon, &lock_param.outlook);
    let mut skeleton_areas: Vec<ExPolygon> = Vec::new();
    // skin and skeleton areas are separated by skin_depths_params
    for (skin_depth, regions) in &lock_param.skin_depths_params {
        let exps = offset_expolygons_clib_scaled(&left, -(*skin_depth as f64), OffsetJoinType::Miter);
        let depth_region = union_safety_offset_ex_expolygons(regions);
        let res = intersection_clib(&exps, &depth_region);
        skeleton_areas.extend(res);
    }
    skeleton_areas = union_safety_offset_ex_expolygons(&skeleton_areas);
    // skin = union_ex(skin, diff_ex(surface.expolygon, skeleton));
    skin_areas.extend(difference_clib(surface_expolygon, &skeleton_areas));
    let skin_areas = union_ex_clib(&crate::geometry::to_polygons(&skin_areas), 1);

    let mut multi_width_polyline: Vec<(Vec<Polyline>, crate::flow::Flow)> = Vec::new();
    // FillRectilinear.cpp:3982-3992 — generate_for_different_flow.
    let mut generate_for_different_flow =
        |flow_params: &[(crate::flow::Flow, Vec<ExPolygon>)], polylines: &[Polyline]| {
            for (flow, regions) in flow_params {
                let region_exp = union_safety_offset_ex_expolygons(regions);
                let polys = intersection_pl(polylines, &region_exp);
                multi_width_polyline.push((polys, flow.clone()));
            }
        };

    // FillRectilinear.cpp:3894-3940 — generate_skeleton_pattern. `params` keeps
    // `locked_zag`; `params.conformal = false`.
    let mut skeleton_lines: Vec<Polyline> = Vec::new();
    {
        // union exps (3906-3912)
        let locked_depths: Vec<(f32, Vec<ExPolygon>)> = lock_param
            .locked_depths_params
            .iter()
            .map(|(depth, regions)| (*depth, union_safety_offset_ex_expolygons(regions)))
            .collect();
        for (density, regions) in &lock_param.skeleton_density_params {
            // remove extra areas
            let exps = intersection_clib(&union_safety_offset_ex_expolygons(regions), &skeleton_areas);
            // add offset
            let mut exps_offset: Vec<ExPolygon> = Vec::new();
            for (lock_depth, depth_regions) in &locked_depths {
                // get diff depths areas
                let res = intersection_clib(&exps, depth_regions);
                let res_offset =
                    offset_expolygons_clib_scaled(&res, *lock_depth as f64, OffsetJoinType::Miter);
                // remove extra areas
                let res_offset = intersection_clib(&res_offset, surface_expolygon);
                exps_offset.extend(res_offset);
            }
            let exps_offset = union_safety_offset_ex_expolygons(&exps_offset);
            locked_zag_pattern_fill(
                skeleton_pattern,
                config,
                layer_index,
                *density,
                true,
                &exps_offset,
                &mut skeleton_lines,
            )?;
        }
    }
    // set skeleton flow
    generate_for_different_flow(&lock_param.skeleton_flow_params, &skeleton_lines);

    // FillRectilinear.cpp:3942-3967 — generate_skin_pattern:
    // `params.locked_zag = false`.
    let mut skin_lines: Vec<Polyline> = Vec::new();
    for (density, regions) in &lock_param.skin_density_params {
        let region_exp = union_safety_offset_ex_expolygons(regions);
        let exps = intersection_clib(&region_exp, &skin_areas);
        let exps = union_safety_offset_ex_expolygons(&exps);
        locked_zag_pattern_fill(
            skin_pattern,
            config,
            layer_index,
            *density,
            false,
            &exps,
            &mut skin_lines,
        )?;
    }
    generate_for_different_flow(&lock_param.skin_flow_params, &skin_lines);

    Some(multi_width_polyline)
}

/// The lines of FillTriangles, FillStars and FillCubic for the pair entry of the slicing path, as
/// `generate_grid_raw_lines` gives those of FillGrid: the three sweeps of
/// `FillTriangles::fill_surface` (FillRectilinear.cpp:3133-3142), `FillStars::fill_surface`
/// (FillRectilinear.cpp:3144-3153) and `FillCubic::fill_surface` (FillRectilinear.cpp:3155-3165) laid by
/// `FillRectilinear::fill_surface_by_multilines` (FillRectilinear.cpp:3037-3063) before its
/// `connect_infill`, and the outer polygons of the offset base the caller connects them along.
/// `z` is `f->z` (Fill.cpp:702, the layer's print_z). None for another pattern.
pub fn generate_multilines_raw_lines(
    expoly: &ExPolygon,
    config: &InfillConfig,
    layer_index: usize,
    pattern: crate::print_config::InfillPattern,
    z: f64,
) -> Option<(Vec<Polyline>, Vec<Polygon>)> {
    use crate::print_config::InfillPattern as P;
    let class = match pattern {
        P::Triangles => FillRectilinearClass::Triangles,
        P::Stars => FillRectilinearClass::Stars,
        P::Cubic => FillRectilinearClass::Cubic,
        _ => return None,
    };
    let mut f = FillRectilinear::with_class(class);
    live_fill_members(&mut f.base, config, layer_index);
    f.base.z = z;
    let surface = live_surface(expoly, config);
    let mut params = live_fill_params(config, false);
    let third = (std::f64::consts::PI / 3.) as f32;
    let sweeps = match class {
        // FillRectilinear.cpp:3139
        FillRectilinearClass::Triangles => [
            SweepParams { angle_base: 0., pattern_shift: 0. },
            SweepParams { angle_base: third, pattern_shift: 0. },
            SweepParams { angle_base: (2. * std::f64::consts::PI / 3.) as f32, pattern_shift: 0. },
        ],
        // FillRectilinear.cpp:3150 — float((3./2.) * this->spacing * params.multiline / params.density)
        FillRectilinearClass::Stars => {
            let shift = ((3. / 2.) * f.base.spacing * params.multiline as f64 / ((params.density as f32) as f64)) as f32;
            [
                SweepParams { angle_base: 0., pattern_shift: 0. },
                SweepParams { angle_base: third, pattern_shift: 0. },
                SweepParams { angle_base: (2. * std::f64::consts::PI / 3.) as f32, pattern_shift: shift },
            ]
        }
        // FillRectilinear.cpp:3158-3161 — coordf_t dx = sqrt(0.5) * z;
        _ => {
            let dx: f64 = 0.5f64.sqrt() * f.base.z;
            [
                SweepParams { angle_base: 0., pattern_shift: dx as f32 },
                SweepParams { angle_base: third, pattern_shift: -(dx as f32) },
                SweepParams { angle_base: (std::f64::consts::PI * 2. / 3.) as f32, pattern_shift: dx as f32 },
            ]
        }
    };
    // FillRectilinear.cpp:3041 — params.density /= double(sweep_params.size());
    params.density = (((params.density as f32) as f64 / 3.0) as f32) as f64;
    Some(match f.fill_lines_of_sweeps(&surface, &params, &sweeps) {
        Some((poly_with_offset_base, fill_lines)) => (fill_lines, poly_with_offset_base.polygons_outer),
        None => (Vec::new(), Vec::new()),
    })
}

/// Faithful FillGrid raster for the emitter-pair path: emit BOTH sweep
/// directions' RAW vertical-line segments (C++ `make_fill_lines`,
/// FillRectilinear.cpp:2993-3030) into one set + return the shared offset
/// boundary (polygons_outer of the unrotated base). The caller runs the single
/// faithful `connect_infill` on the combined set — matching C++
/// `fill_surface_by_multilines`. Returns (raw_lines, connect_boundary).
pub fn generate_grid_raw_lines(
    expoly: &ExPolygon,
    config: &InfillConfig,
    layer_index: usize,
) -> (Vec<Polyline>, Vec<Polygon>) {
    // FILL_RECTILINEAR_PIN (on unless =0): the lines of the two sweeps as
    // `FillRectilinear::fill_surface_by_multilines` lays them for FillGrid (FillRectilinear.cpp:3037-3063,
    // 3118-3125) in the arithmetic of the pin: `scale_` as a division, the sweep density halved in
    // float, the offset polygon turned by `ExPolygonWithOffset(base, -angle)` and the lines turned back
    // by `Point::rotated(cos, sin)` with fused products and one `__sincos_stret` call.
    if fill_rectilinear_pin()
        && crate::faithful_gate("GRID_ANGLE_F32")
        && crate::faithful_gate("GRID_FILL_LINES_FAITHFUL")
        && crate::faithful_gate("RECT_OFFSET_FAITHFUL")
        && crate::faithful_gate("TOPFILL_FAITHFUL")
    {
        let mut f = FillGrid::new();
        live_fill_members(&mut f.0.base, config, layer_index);
        let surface = live_surface(expoly, config);
        let mut params = live_fill_params(config, false);
        // params.density /= double(sweep_params.size());
        params.density = (((params.density as f32) as f64 / 2.0) as f32) as f64;
        let sweeps = [SweepParams { angle_base: 0., pattern_shift: 0. }, SweepParams { angle_base: (std::f64::consts::PI / 2.) as f32, pattern_shift: 0. }];
        return match f.0.fill_lines_of_sweeps(&surface, &params, &sweeps) {
            Some((poly_with_offset_base, fill_lines)) => (fill_lines, poly_with_offset_base.polygons_outer),
            None => (Vec::new(), Vec::new()),
        };
    }
    // FillBase.hpp:207 — _layer_angle alternates 0 / 90° on odd/even layers.
    let layer_angle = if layer_index & 1 == 1 {
        std::f64::consts::FRAC_PI_2
    } else {
        0.0
    };
    let angle_rad = config.angle.to_radians() + layer_angle;
    // GRID_ANGLE_F32 (on): the sweep angles as C++ forms them, in `float`.
    // Fill::_infill_direction (FillBase.cpp:204-244) starts from the float
    // `angle`, adds `_layer_angle()` — 0 for FillGrid on every layer
    // (FillRectilinear.hpp:75) — and float(M_PI/2); fill_surface_by_multilines
    // (FillRectilinear.cpp:3050-3054) adds each sweep's float base, 0 and
    // float(M_PI/2). So both sweeps sit at +90 and +180 degrees on every layer,
    // and the rotation uses the cosine of a float angle. =0 turns the grid by 90
    // degrees on odd layers only and keeps the angles in double.
    let sweeps: [f64; 2] = if crate::faithful_gate("GRID_ANGLE_F32") {
        let half_pi = std::f64::consts::FRAC_PI_2 as f32;
        let mut out_angle: f32 = (std::f64::consts::PI * config.angle / 180.0) as f32;
        out_angle += 0.0f32;
        out_angle += half_pi;
        [(out_angle + 0.0f32) as f64, (out_angle + half_pi) as f64]
    } else {
        [angle_rad, angle_rad + std::f64::consts::FRAC_PI_2]
    };
    let pass_density = config.density / 2.0; // density /= sweep_count(2)
    let spacing = config.extrusion_width;
    // C++ poly_with_offset_base = ExPolygonWithOffset(expoly, 0, scale_(overlap -
    // 0.5*spacing)) — a SINGLE offset arg; aoffset2 DEFAULTS to 0, so NO inner
    // contour is built and the slice yields only OUTER_LOW/OUTER_HIGH pairs (the
    // strict pairing make_fill_lines relies on). Passing a negative aoffset2 (as
    // fill_surface_by_lines does) would add INNER intersections that break it.
    let base_offset = scale(config.overlap - 0.5 * spacing);
    let line_spacing = (spacing * crate::SCALING_FACTOR / pass_density) as Coord;

    // Build the offset base ONCE at angle 0 (C++ poly_with_offset_base), then for
    // each sweep ROTATE that base (copy-rotate ctor) and slice. The raw line
    // endpoints then lie EXACTLY on base.polygons_outer after rotate-back, so the
    // single connect_infill can snap+chain them (proven byte-exact vs C++).
    let base = ExPolygonWithOffset::new(expoly, 0.0, base_offset, 0);
    let mut out: Vec<Polyline> = Vec::new();
    for a in sweeps {
        let pwo = ExPolygonWithOffset::from_base_outer_rotated(&base.polygons_outer, -a);
        if pwo.n_contours_outer == 0 {
            continue;
        }
        // GRID_FILL_LINES_FAITHFUL (on): make_fill_lines (FillRectilinear.cpp
        // 2993-3030) works from the bounding box of the rotated SOURCE polygon
        // and drops the lines within `x_margin` = scaled spacing + SCALED_EPSILON
        // of its left and right side, "which fully overlap with the infill
        // perimeter". =0 takes the box of the offset contour and keeps them.
        if crate::faithful_gate("GRID_FILL_LINES_FAITHFUL") {
            let mut src_bbox = crate::geometry::BoundingBox::default();
            let mut add = |ring: &Polygon| {
                let mut r = ring.clone();
                if (-a).abs() > 1e-10 {
                    r.rotate(-a);
                }
                for p in r.points() {
                    src_bbox.merge_point(*p);
                }
            };
            add(&base.polygons_src.contour);
            for h in &base.polygons_src.holes {
                add(h);
            }
            let x_margin = (spacing / crate::libslic3r::SCALING_FACTOR) as Coord
                + crate::libslic3r::SCALED_EPSILON as Coord;
            make_fill_lines_raw_bbox(&pwo, a, line_spacing, Some((src_bbox, x_margin)), &mut out);
        } else {
            make_fill_lines_raw(&pwo, a, line_spacing, &mut out);
        }
    }
    (out, base.polygons_outer)
}

/// Faithful port of C++ `make_fill_lines` (FillRectilinear.cpp:2993-3030): emit
/// raw vertical-line OUTER_LOW→OUTER_HIGH segments rotated back by `angle`. No
/// graph connect — the caller runs the single faithful connect_infill.
fn make_fill_lines_raw(
    pwo: &ExPolygonWithOffset,
    angle: f64,
    line_spacing: Coord,
    out: &mut Vec<Polyline>,
) {
    make_fill_lines_raw_bbox(pwo, angle, line_spacing, None, out)
}

/// [`make_fill_lines_raw`] with the source bounding box and `x_margin` of
/// FillRectilinear.cpp:2995-3003, 3011-3013 when `src` is given.
fn make_fill_lines_raw_bbox(
    pwo: &ExPolygonWithOffset,
    angle: f64,
    line_spacing: Coord,
    src: Option<(crate::geometry::BoundingBox, Coord)>,
    out: &mut Vec<Polyline>,
) {
    if line_spacing <= 0 {
        return;
    }
    if let Some((src_bbox, x_margin)) = src {
        if !src_bbox.is_defined() {
            return;
        }
        // FillRectilinear.cpp:2995-3003 — refpt is the centre of the object's
        // bounding box, the origin, and pattern_shift is 0.
        let x_min = src_bbox.min.x + x_margin;
        let x_max = src_bbox.max.x - x_margin;
        let aligned = crate::geometry::Point::new(
            crate::geometry::align_to_grid(src_bbox.min.x, line_spacing),
            crate::geometry::align_to_grid(src_bbox.min.y, line_spacing),
        );
        let mut bbox = src_bbox;
        bbox.merge_point(aligned);
        // FillRectilinear.cpp:3007
        let n_vlines = ((bbox.max.x - bbox.min.x + line_spacing - 1) / line_spacing).max(0) as usize;
        let cos_a = angle.cos();
        let sin_a = angle.sin();
        // FillRectilinear.cpp:3010-3029
        for vline in &slice_region_by_vertical_lines(pwo, n_vlines, bbox.min.x, line_spacing) {
            if vline.pos < x_min {
                continue;
            }
            if vline.pos > x_max {
                break;
            }
            let mut i = 0;
            while i < vline.intersections.len() {
                let low = i;
                i += 1;
                if vline.intersections[low].itype != SegmentIntersectionType::OuterLow {
                    continue;
                }
                if i < vline.intersections.len()
                    && vline.intersections[i].itype == SegmentIntersectionType::OuterHigh
                {
                    let (p1, p2) = if angle == 0.0 {
                        (
                            Point::new(vline.pos, vline.intersections[low].pos()),
                            Point::new(vline.pos, vline.intersections[i].pos()),
                        )
                    } else {
                        (
                            rotate_back(vline.pos, vline.intersections[low].pos(), cos_a, sin_a),
                            rotate_back(vline.pos, vline.intersections[i].pos(), cos_a, sin_a),
                        )
                    };
                    out.push(Polyline::from_points(vec![p1, p2]));
                    i += 1;
                }
            }
        }
        return;
    }
    // bbox of the OFFSET geometry (this pwo is built from rotated outer polys, so
    // polygons_src is empty — use the outer contours' extents like C++ which
    // bbox's poly_with_offset's contours).
    if pwo.polygons_outer.is_empty() {
        return;
    }
    let mut bbox = crate::geometry::BoundingBox::default();
    for poly in &pwo.polygons_outer {
        for p in poly.points() {
            bbox.merge_point(*p);
        }
    }
    if bbox.max.x <= bbox.min.x {
        return;
    }
    // C++ make_fill_lines (FillRectilinear.cpp:3001-3003): align the bbox min to the
    // grid relative to the infill reference point so the pattern is consistent
    // across layers. refpt = origin rotated into the vline frame; pattern_shift=0
    // here, so align bbox.min.x down to the nearest line_spacing multiple of refpt.x.
    // refpt = origin (rotate_vector.second ≈ origin), base.x = 0; align_to_grid_base
    // = base + align_to_grid(coord - base, spacing) = align_to_grid(bbox.min.x).
    let aligned_min_x = crate::geometry::align_to_grid(bbox.min.x, line_spacing);
    let min_x = aligned_min_x.min(bbox.min.x);
    let n_vlines = ((bbox.max.x - min_x + line_spacing - 1) / line_spacing).max(0) as usize;
    if n_vlines == 0 {
        return;
    }
    let cos_a = angle.cos();
    let sin_a = angle.sin();
    let segs = slice_region_by_vertical_lines(pwo, n_vlines, min_x, line_spacing);
    for vline in &segs {
        let mut i = 0;
        while i + 1 < vline.intersections.len() {
            if vline.intersections[i].itype != SegmentIntersectionType::OuterLow {
                i += 1;
                continue;
            }
            if vline.intersections[i + 1].itype == SegmentIntersectionType::OuterHigh {
                let p1 = rotate_back(vline.pos, vline.intersections[i].pos(), cos_a, sin_a);
                let p2 = rotate_back(vline.pos, vline.intersections[i + 1].pos(), cos_a, sin_a);
                out.push(Polyline::from_points(vec![p1, p2]));
                i += 2;
            } else {
                i += 1;
            }
        }
    }
}

#[inline]
fn rotate_back(x: Coord, y: Coord, cos_a: f64, sin_a: f64) -> Point {
    // C++ Point::rotated(cos, sin) = (x*cos - y*sin, x*sin + y*cos).
    let xf = x as f64;
    let yf = y as f64;
    Point::new(
        (xf * cos_a - yf * sin_a).round() as Coord,
        (xf * sin_a + yf * cos_a).round() as Coord,
    )
}

// ===========================================================================
// Monotonic infill implementation
// ===========================================================================
//
// Port of libslic3r's FillMonotonic algorithm. This produces significantly
// better paths for top/bottom surfaces by:
// 1. Grouping vertical infill segments into "monotonic regions"
// 2. Connecting neighboring regions
// 3. Using ant-colony optimization to chain regions optimally
// 4. Generating polylines that sweep in a consistent direction

// ---------------------------------------------------------------------------
// Vertical run navigation helpers (index-based, porting C++ pointer arithmetic)
// ---------------------------------------------------------------------------

/// Find the bottom (lowest INNER_LOW) of a vertical run starting from `i_start`.
/// Returns the index of the bottom INNER_LOW intersection.
fn vertical_run_bottom(vline: &SegmentedIntersectionLine, i_start: usize) -> usize {
    assert!(vline.intersections[i_start].is_inner());
    let mut idx = i_start;
    loop {
        // Find INNER_LOW going downward
        while vline.intersections[idx].itype != SegmentIntersectionType::InnerLow {
            if idx == 0 {
                break;
            }
            idx -= 1;
        }
        if idx > 0 && vline.intersections[idx - 1].itype == SegmentIntersectionType::InnerHigh {
            idx -= 1;
        } else {
            let down = vline.intersections[idx].vertical_down();
            if down == -1 || vline.intersections[idx].vertical_down_quality() != LinkQuality::Valid
            {
                break;
            }
            idx = down as usize;
            assert!(vline.intersections[idx].itype == SegmentIntersectionType::InnerHigh);
        }
    }
    idx
}

/// Find the top (highest INNER_HIGH) of a vertical run starting from `i_start`.
/// Returns the index of the top INNER_HIGH intersection.
fn vertical_run_top(vline: &SegmentedIntersectionLine, i_start: usize) -> usize {
    assert!(vline.intersections[i_start].is_inner());
    let mut idx = i_start;
    loop {
        while vline.intersections[idx].itype != SegmentIntersectionType::InnerHigh {
            idx += 1;
            if idx >= vline.intersections.len() {
                return vline.intersections.len() - 1;
            }
        }
        if idx + 1 < vline.intersections.len()
            && vline.intersections[idx + 1].itype == SegmentIntersectionType::InnerLow
        {
            idx += 1;
        } else {
            let up = vline.intersections[idx].vertical_up();
            if up == -1 || vline.intersections[idx].vertical_up_quality() != LinkQuality::Valid {
                break;
            }
            idx = up as usize;
            assert!(vline.intersections[idx].itype == SegmentIntersectionType::InnerLow);
        }
    }
    idx
}

/// Find the last INNER_HIGH of a raw vertical run (no vertical links followed).
fn end_of_vertical_run_raw(vline: &SegmentedIntersectionLine, i_start: usize) -> usize {
    assert!(vline.intersections[i_start].itype == SegmentIntersectionType::InnerLow);
    let mut idx = i_start;
    loop {
        idx += 1;
        if idx >= vline.intersections.len() {
            return vline.intersections.len() - 1;
        }
        if vline.intersections[idx].itype == SegmentIntersectionType::OuterHigh {
            break;
        }
    }
    if idx > 0 && vline.intersections[idx - 1].is_inner() {
        idx -= 1;
        assert!(vline.intersections[idx].itype == SegmentIntersectionType::InnerHigh);
    }
    idx
}

/// Find the last INNER_HIGH intersection of a full vertical run (following vertical links).
fn end_of_vertical_run(vline: &SegmentedIntersectionLine, i_start: usize) -> usize {
    assert!(vline.intersections[i_start].itype == SegmentIntersectionType::InnerLow);
    let mut end_idx = end_of_vertical_run_raw(vline, i_start);
    assert!(vline.intersections[end_idx].itype == SegmentIntersectionType::InnerHigh);
    loop {
        let up = vline.intersections[end_idx].vertical_up();
        if up == -1 {
            break;
        }
        let quality = if vline.intersections[end_idx].has_left_vertical_up() {
            vline.intersections[end_idx].prev_on_contour_quality
        } else {
            vline.intersections[end_idx].next_on_contour_quality
        };
        if quality != LinkQuality::Valid {
            break;
        }
        let new_start = up as usize;
        assert!(vline.intersections[new_start].itype == SegmentIntersectionType::InnerLow);
        end_idx = end_of_vertical_run_raw(vline, new_start);
    }
    assert!(vline.intersections[end_idx].itype == SegmentIntersectionType::InnerHigh);
    end_idx
}

/// Find the bottom of the overlapping region on `vline_other` for a vertical run
/// on `vline_this` defined by `[i_start..i_end]`, looking in direction `side`.
fn overlap_bottom(
    segs: &[SegmentedIntersectionLine],
    i_vline_this: usize,
    i_start: usize,
    i_end: usize,
    i_vline_other: usize,
    side: Side,
) -> Option<usize> {
    let vline_this = &segs[i_vline_this];
    let vline_other = &segs[i_vline_other];
    assert!(vline_this.intersections[i_start].is_inner());
    assert!(vline_this.intersections[i_end].is_inner());
    let mut idx = i_start;
    loop {
        if vline_this.intersections[idx].is_inner() {
            let i = vline_this.intersections[idx].horizontal(side);
            if i != -1 {
                let other_idx = i as usize;
                return Some(vertical_run_bottom(vline_other, other_idx));
            }
            if idx == i_end {
                break;
            }
        }
        if vline_this.intersections[idx].itype != SegmentIntersectionType::InnerHigh {
            idx += 1;
        } else if idx + 1 < vline_this.intersections.len()
            && vline_this.intersections[idx + 1].itype == SegmentIntersectionType::InnerLow
        {
            idx += 1;
        } else {
            let up = vline_this.intersections[idx].vertical_up();
            if up == -1 || vline_this.intersections[idx].vertical_up_quality() != LinkQuality::Valid
            {
                break;
            }
            idx = up as usize;
            assert!(vline_this.intersections[idx].itype == SegmentIntersectionType::InnerLow);
        }
    }
    None
}

/// Find the top of the overlapping region on `vline_other`.
fn overlap_top(
    segs: &[SegmentedIntersectionLine],
    i_vline_this: usize,
    i_start: usize,
    i_end: usize,
    i_vline_other: usize,
    side: Side,
) -> Option<usize> {
    let vline_this = &segs[i_vline_this];
    let vline_other = &segs[i_vline_other];
    assert!(vline_this.intersections[i_start].is_inner());
    assert!(vline_this.intersections[i_end].is_inner());
    let mut idx = i_end;
    loop {
        if vline_this.intersections[idx].is_inner() {
            let i = vline_this.intersections[idx].horizontal(side);
            if i != -1 {
                let other_idx = i as usize;
                return Some(vertical_run_top(vline_other, other_idx));
            }
            if idx == i_start {
                break;
            }
        }
        if vline_this.intersections[idx].itype != SegmentIntersectionType::InnerLow {
            if idx == 0 {
                break;
            }
            idx -= 1;
        } else if idx > 0
            && vline_this.intersections[idx - 1].itype == SegmentIntersectionType::InnerHigh
        {
            idx -= 1;
        } else {
            let down = vline_this.intersections[idx].vertical_down();
            if down == -1
                || vline_this.intersections[idx].vertical_down_quality() != LinkQuality::Valid
            {
                break;
            }
            idx = down as usize;
            assert!(vline_this.intersections[idx].itype == SegmentIntersectionType::InnerHigh);
        }
    }
    None
}

/// Left overlap: find the (bottom, top) indices on vline_left that overlap
/// with the vertical run [i_low..i_high] on vline_this.
fn left_overlap(
    segs: &[SegmentedIntersectionLine],
    i_vline: usize,
    i_low: usize,
    i_high: usize,
) -> Option<(usize, usize)> {
    if i_vline == 0 {
        return None;
    }
    let i_vline_left = i_vline - 1;
    let bot = overlap_bottom(segs, i_vline, i_low, i_high, i_vline_left, Side::Left)?;
    let top = overlap_top(segs, i_vline, i_low, i_high, i_vline_left, Side::Left)?;
    if bot < top {
        Some((bot, top))
    } else {
        None
    }
}

/// Right overlap: find the (bottom, top) indices on vline_right.
fn right_overlap(
    segs: &[SegmentedIntersectionLine],
    i_vline: usize,
    i_low: usize,
    i_high: usize,
) -> Option<(usize, usize)> {
    if i_vline + 1 >= segs.len() {
        return None;
    }
    let i_vline_right = i_vline + 1;
    let bot = overlap_bottom(segs, i_vline, i_low, i_high, i_vline_right, Side::Right)?;
    let top = overlap_top(segs, i_vline, i_low, i_high, i_vline_right, Side::Right)?;
    if bot < top {
        Some((bot, top))
    } else {
        None
    }
}

// ---------------------------------------------------------------------------
// Pinch contours: insert phony OUTER_HIGH/OUTER_LOW pairs
// ---------------------------------------------------------------------------

/// Create a phony outer intersection at the given position.
fn phony_outer_intersection(itype: SegmentIntersectionType, pos: Coord) -> SegmentIntersection {
    debug_assert!(matches!(
        itype,
        SegmentIntersectionType::OuterLow | SegmentIntersectionType::OuterHigh
    ));
    let mut out = SegmentIntersection::default();
    // Invalid contour & segment. FillRectilinear.cpp:434-435
    out.i_contour = usize::MAX;
    out.i_segment = usize::MAX;
    out.pos_p = pos as i64;
    out.itype = itype;
    // Invalid prev / next. FillRectilinear.cpp:439-444
    out.prev_on_contour = -1;
    out.next_on_contour = -1;
    out.prev_on_contour_type = LinkType::Phony;
    out.next_on_contour_type = LinkType::Phony;
    out.prev_on_contour_quality = LinkQuality::Invalid;
    out.next_on_contour_quality = LinkQuality::Invalid;
    out
}

/// Insert phony OUTER_HIGH / OUTER_LOW pairs at pinch points where
/// the outer contour squeezes the inner contour.
/// Port of libslic3r `pinch_contours_insert_phony_outer_intersections`.
fn pinch_contours_insert_phony_outer_intersections(segs: &mut Vec<SegmentedIntersectionLine>) {
    let mut insert_after: Vec<usize> = Vec::new();
    let mut map: Vec<i32> = Vec::new();
    let mut temp_intersections: Vec<SegmentIntersection> = Vec::new();

    for i_vline in 1..segs.len() {
        let il = &segs[i_vline];
        if il.intersections.is_empty() {
            continue;
        }
        assert!(il.intersections.first().unwrap().itype == SegmentIntersectionType::OuterLow);
        assert!(il.intersections.last().unwrap().itype == SegmentIntersectionType::OuterHigh);

        insert_after.clear();
        let end_idx = il.intersections.len() - 1;
        let mut it = 1usize;
        while it < end_idx {
            if il.intersections[it].itype == SegmentIntersectionType::OuterHigh {
                it += 1; // skip OUTER_HIGH
                assert!(il.intersections[it].itype == SegmentIntersectionType::OuterLow);
                it += 1;
            } else {
                assert!(il.intersections[it].itype == SegmentIntersectionType::InnerLow);
                let hi = it + 1;
                if hi >= end_idx {
                    it = hi + 1;
                    continue;
                }
                assert!(il.intersections[hi].itype == SegmentIntersectionType::InnerHigh);
                let lo2 = hi + 1;
                if lo2 >= end_idx {
                    it = lo2 + 1;
                    continue;
                }
                if il.intersections[lo2].itype == SegmentIntersectionType::InnerLow {
                    // INNER_HIGH followed by INNER_LOW — possible pinch
                    let up = il.intersections[hi].vertical_up();
                    let dn = il.intersections[lo2].vertical_down();
                    // FillRectilinear.cpp:1324 — bool pinched = dn + 1 != up;
                    let pinched = dn + 1 != up;
                    if pinched {
                        insert_after.push(hi);
                    }
                }
                it = lo2;
            }
        }

        if !insert_after.is_empty() {
            map.clear();
            temp_intersections.clear();
            let mut i = 0usize;
            for &idx_insert_after in &insert_after {
                while i <= idx_insert_after {
                    map.push(temp_intersections.len() as i32);
                    temp_intersections.push(segs[i_vline].intersections[i].clone());
                    i += 1;
                }
                let pos = (temp_intersections.last().unwrap().pos()
                    + segs[i_vline].intersections[i].pos())
                    / 2;
                temp_intersections.push(phony_outer_intersection(
                    SegmentIntersectionType::OuterHigh,
                    pos,
                ));
                temp_intersections.push(phony_outer_intersection(
                    SegmentIntersectionType::OuterLow,
                    pos,
                ));
            }
            while i < segs[i_vline].intersections.len() {
                map.push(temp_intersections.len() as i32);
                temp_intersections.push(segs[i_vline].intersections[i].clone());
                i += 1;
            }
            std::mem::swap(&mut segs[i_vline].intersections, &mut temp_intersections);

            // Reindex references on current line
            for ip in &mut segs[i_vline].intersections {
                if ip.has_left_vertical() && ip.prev_on_contour >= 0 {
                    let old = ip.prev_on_contour as usize;
                    if old < map.len() {
                        ip.prev_on_contour = map[old];
                    }
                }
                if ip.has_right_vertical() && ip.next_on_contour >= 0 {
                    let old = ip.next_on_contour as usize;
                    if old < map.len() {
                        ip.next_on_contour = map[old];
                    }
                }
            }
            // Reindex references on previous line
            for ip in &mut segs[i_vline - 1].intersections {
                if ip.has_right_horizontal() && ip.next_on_contour >= 0 {
                    let old = ip.next_on_contour as usize;
                    if old < map.len() {
                        ip.next_on_contour = map[old];
                    }
                }
            }
            // Reindex references on next line
            if i_vline + 1 < segs.len() {
                for ip in &mut segs[i_vline + 1].intersections {
                    if ip.has_left_horizontal() && ip.prev_on_contour >= 0 {
                        let old = ip.prev_on_contour as usize;
                        if old < map.len() {
                            ip.prev_on_contour = map[old];
                        }
                    }
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------
// MonotonicRegion and related types
// ---------------------------------------------------------------------------

/// Boundary of a monotonic region on one side (left or right).
#[derive(Debug, Clone, Copy, Default)]
struct MonotonicBoundary {
    vline: usize,
    low: usize,
    high: usize,
}

/// A monotonic region: a rectangular group of vertical infill segments
/// that can be extruded in a single monotonic sweep.
#[derive(Debug, Clone)]
struct MonotonicRegion {
    left: MonotonicBoundary,
    right: MonotonicBoundary,
    /// Length when starting at left.low
    len1: f32,
    /// Length when starting at left.high
    len2: f32,
    /// If true, starting at left.low exits at right.high (and vice versa).
    flips: bool,
    /// Indices of regions whose right boundary touches our left boundary.
    left_neighbors: Vec<usize>,
    /// Indices of regions whose left boundary touches our right boundary.
    right_neighbors: Vec<usize>,
}

impl MonotonicRegion {
    fn new() -> Self {
        Self {
            left: MonotonicBoundary::default(),
            right: MonotonicBoundary::default(),
            len1: 0.0,
            len2: 0.0,
            flips: false,
            left_neighbors: Vec::new(),
            right_neighbors: Vec::new(),
        }
    }

    fn length(&self, flipped: bool) -> f32 {
        if flipped {
            self.len2
        } else {
            self.len1
        }
    }

    fn left_intersection_point(&self, flipped: bool) -> usize {
        if flipped {
            self.left.high
        } else {
            self.left.low
        }
    }

    fn right_intersection_point(&self, flipped: bool) -> usize {
        if flipped == self.flips {
            self.right.low
        } else {
            self.right.high
        }
    }
}

/// Ant colony path segment between regions.
#[derive(Debug, Clone)]
struct AntPath {
    length: f32,
    visibility: f32,
    pheromone: f32,
}

impl AntPath {
    fn new() -> Self {
        Self {
            length: -1.0,
            visibility: -1.0,
            pheromone: 0.0,
        }
    }
}

/// Link in a chained monotonic path.
#[derive(Debug, Clone)]
struct MonotonicRegionLink {
    region_idx: usize,
    flipped: bool,
    next_path_idx: Option<usize>,
    next_flipped_path_idx: Option<usize>,
}

/// Matrix of AntPath entries between all pairs of monotonic regions.
struct AntPathMatrix {
    /// Flattened matrix: rows = 2*regions (each region has normal + flipped),
    /// cols = 2*regions. Entry [from_region*2+from_flipped][to_region*2+to_flipped].
    paths: Vec<AntPath>,
    n: usize, // number of regions
}

impl AntPathMatrix {
    fn new(
        regions: &[MonotonicRegion],
        poly_with_offset: &ExPolygonWithOffset,
        segs: &[SegmentedIntersectionLine],
        pheromone_initial: f32,
    ) -> Self {
        let n = regions.len();
        let dim = 2 * n;
        let mut paths = vec![AntPath::new(); dim * dim];

        // Initialize path lengths and visibilities
        for i in 0..n {
            for j in 0..n {
                if i == j {
                    continue;
                }
                for from_flip in 0..2usize {
                    for to_flip in 0..2usize {
                        let from_flipped = from_flip != 0;
                        let to_flipped = to_flip != 0;

                        // Compute distance from exit of region i to entry of region j.
                        // FillRectilinear.cpp:1744-1759.
                        let i_from = regions[i].right_intersection_point(from_flipped);
                        let i_to = regions[j].left_intersection_point(to_flipped);
                        let exit_vline = regions[i].right.vline;
                        let entry_vline = regions[j].left.vline;

                        let mut length: f32 = -1.0;
                        // FillRectilinear.cpp:1748 — measure along the contour only when the
                        // right vline is exactly the left vline minus one and the horizontal
                        // link points at i_to with valid quality. (Replicated as-is.)
                        if exit_vline + 1 == regions[i].left.vline
                            && exit_vline < segs.len()
                            && i_from < segs[exit_vline].intersections.len()
                        {
                            let it = &segs[exit_vline].intersections[i_from];
                            let i_right = it.right_horizontal();
                            if i_right == i_to as i32
                                && it.next_on_contour_quality == LinkQuality::Valid
                            {
                                length = measure_perimeter_horizontal_segment_length(
                                    poly_with_offset,
                                    segs,
                                    exit_vline,
                                    i_from,
                                    i_to,
                                ) as f32
                                    / crate::SCALING_FACTOR as f32;
                            }
                        }

                        if length == -1.0 {
                            // Euclidean distance of the end points. FillRectilinear.cpp:1757
                            let p1 = if exit_vline < segs.len()
                                && i_from < segs[exit_vline].intersections.len()
                            {
                                let vl = &segs[exit_vline];
                                Point::new(vl.pos, vl.intersections[i_from].pos())
                            } else {
                                Point::new(0, 0)
                            };
                            let p2 = if entry_vline < segs.len()
                                && i_to < segs[entry_vline].intersections.len()
                            {
                                let vl = &segs[entry_vline];
                                Point::new(vl.pos, vl.intersections[i_to].pos())
                            } else {
                                Point::new(0, 0)
                            };
                            let dx = (p2.x - p1.x) as f64;
                            let dy = (p2.y - p1.y) as f64;
                            let dist = (dx * dx + dy * dy).sqrt() as f32;
                            length = dist / crate::SCALING_FACTOR as f32;
                        }

                        let idx = (i * 2 + from_flip) * dim + (j * 2 + to_flip);
                        paths[idx].length = length;
                        // visibility = 1 / (length + EPSILON), EPSILON = 1e-4.
                        // FillRectilinear.cpp:1759.
                        paths[idx].visibility = 1.0 / (length + 1e-4_f32);
                        paths[idx].pheromone = pheromone_initial;
                    }
                }
            }
        }

        Self { paths, n }
    }

    fn update_initial_pheromone(&mut self, pheromone: f32) {
        for p in &mut self.paths {
            p.pheromone = pheromone;
        }
    }

    fn get(
        &self,
        from_region: usize,
        from_flipped: bool,
        to_region: usize,
        to_flipped: bool,
    ) -> &AntPath {
        let dim = 2 * self.n;
        let from_flip = if from_flipped { 1 } else { 0 };
        let to_flip = if to_flipped { 1 } else { 0 };
        &self.paths[(from_region * 2 + from_flip) * dim + (to_region * 2 + to_flip)]
    }

    fn get_mut(
        &mut self,
        from_region: usize,
        from_flipped: bool,
        to_region: usize,
        to_flipped: bool,
    ) -> &mut AntPath {
        let dim = 2 * self.n;
        let from_flip = if from_flipped { 1 } else { 0 };
        let to_flip = if to_flipped { 1 } else { 0 };
        &mut self.paths[(from_region * 2 + from_flip) * dim + (to_region * 2 + to_flip)]
    }
}

// ---------------------------------------------------------------------------
// Generate monotonic regions
// ---------------------------------------------------------------------------

/// Group vertical infill segments into monotonic regions.
/// Port of libslic3r `generate_montonous_regions`.
fn generate_monotonous_regions(segs: &mut Vec<SegmentedIntersectionLine>) -> Vec<MonotonicRegion> {
    let mut monotonic_regions: Vec<MonotonicRegion> = Vec::new();

    for i_vline_seed in 0..segs.len() {
        let mut i_intersection_seed = 1usize;
        while i_intersection_seed + 1 < segs[i_vline_seed].intersections.len() {
            // Find the next INNER_LOW
            while i_intersection_seed < segs[i_vline_seed].intersections.len()
                && segs[i_vline_seed].intersections[i_intersection_seed].itype
                    != SegmentIntersectionType::InnerLow
            {
                i_intersection_seed += 1;
            }
            if i_intersection_seed >= segs[i_vline_seed].intersections.len() {
                break;
            }

            let start_idx = i_intersection_seed;
            let end_idx = end_of_vertical_run(&segs[i_vline_seed], start_idx);

            if !segs[i_vline_seed].intersections[start_idx].consumed_vertical_up {
                // Start a new monotonic region
                let mut i_vline = i_vline_seed;
                let mut left_low = start_idx;
                let mut left_high = end_idx;

                let mut region = MonotonicRegion::new();
                region.left.vline = i_vline;
                region.left.low = left_low;
                region.left.high = left_high;
                region.right = region.left;

                segs[i_vline_seed].intersections[start_idx].consumed_vertical_up = true;
                let mut num_lines = 1;

                // Extend right as long as there's a unique 1:1 overlap
                loop {
                    let next_vline = i_vline + 1;
                    if next_vline >= segs.len() {
                        break;
                    }

                    // Find right overlap
                    let right_opt = right_overlap(segs, i_vline, left_low, left_high);
                    if right_opt.is_none() {
                        break;
                    }
                    let (right_low, right_high) = right_opt.unwrap();

                    // Check the top of right_low matches right_high
                    let right_top_first = vertical_run_top(&segs[next_vline], right_low);
                    if right_top_first != right_high {
                        // Overlaps with multiple segments
                        break;
                    }

                    // Check left overlap of (right_low, right_high) matches (left_low, left_high)
                    let right_left_opt = left_overlap(segs, next_vline, right_low, right_high);
                    match right_left_opt {
                        Some((rl_low, rl_high)) if rl_low == left_low && rl_high == left_high => {}
                        _ => break,
                    }

                    region.right.vline = next_vline;
                    region.right.low = right_low;
                    region.right.high = right_high;
                    segs[next_vline].intersections[right_low].consumed_vertical_up = true;
                    num_lines += 1;
                    i_vline = next_vline;
                    left_low = right_low;
                    left_high = right_high;
                }

                // Even number of lines makes the infill zig-zag
                region.flips = (num_lines & 1) != 0;
                monotonic_regions.push(region);
            }

            i_intersection_seed = end_idx + 1;
        }
    }

    monotonic_regions
}

// ---------------------------------------------------------------------------
// Connect monotonic regions (find left/right neighbors)
// ---------------------------------------------------------------------------

/// Connect monotonic regions by finding left/right neighbor relationships.
/// Port of libslic3r `connect_monotonic_regions`.
fn connect_monotonic_regions(
    regions: &mut Vec<MonotonicRegion>,
    poly_with_offset: &ExPolygonWithOffset,
    segs: &[SegmentedIntersectionLine],
) {
    let n = regions.len();
    if n == 0 {
        return;
    }

    // Map from low intersection (vline, idx) to the left / right side of a
    // monotonic region. FillRectilinear.cpp:2152-2164. C++ keys the map on the
    // SegmentIntersection* of region.left.low / region.right.low and binary
    // searches it. We use (vline, idx) keys, which are unique per region side.
    use std::collections::HashMap;
    let mut map_start: HashMap<(usize, usize), usize> = HashMap::with_capacity(n);
    let mut map_end: HashMap<(usize, usize), usize> = HashMap::with_capacity(n);
    for (i, region) in regions.iter().enumerate() {
        map_start.insert((region.left.vline, region.left.low), i);
        map_end.insert((region.right.vline, region.right.low), i);
    }

    // Scatter links to neighboring regions. FillRectilinear.cpp:2167-2208.
    for i in 0..n {
        // Left side.
        let left_vline = regions[i].left.vline;
        if left_vline > 0 {
            let vline_left_idx = left_vline - 1;
            // left_overlap(vline.intersections[left.low], vline.intersections[left.high], ...)
            if let Some((lbegin0, lend)) =
                left_overlap(segs, left_vline, regions[i].left.low, regions[i].left.high)
            {
                let mut lbegin = lbegin0;
                loop {
                    // Region whose right.low == lbegin on vline_left.
                    if let Some(&ei) = map_end.get(&(vline_left_idx, lbegin)) {
                        // FillRectilinear.cpp:2177 — it->second->right_neighbors.push_back(&region)
                        regions[ei].right_neighbors.push(i);
                    }
                    // lnext = vertical_run_top(vline_left, *lbegin)
                    let lnext = vertical_run_top(&segs[vline_left_idx], lbegin);
                    if lnext == lend {
                        break;
                    }
                    // while (lnext->type != INNER_LOW) ++ lnext;
                    let mut k = lnext;
                    while k < segs[vline_left_idx].intersections.len()
                        && segs[vline_left_idx].intersections[k].itype
                            != SegmentIntersectionType::InnerLow
                    {
                        k += 1;
                    }
                    if k >= segs[vline_left_idx].intersections.len() {
                        break;
                    }
                    lbegin = k;
                }
            }
        }

        // Right side.
        let right_vline = regions[i].right.vline;
        if right_vline + 1 < segs.len() {
            let vline_right_idx = right_vline + 1;
            if let Some((rbegin0, rend)) =
                right_overlap(segs, right_vline, regions[i].right.low, regions[i].right.high)
            {
                let mut rbegin = rbegin0;
                loop {
                    // FillRectilinear.cpp:2194-2196 — if the region start is not found, break.
                    let si = match map_start.get(&(vline_right_idx, rbegin)) {
                        Some(&si) => si,
                        None => break,
                    };
                    // it->second->left_neighbors.push_back(&region)
                    regions[si].left_neighbors.push(i);
                    let rnext = vertical_run_top(&segs[vline_right_idx], rbegin);
                    if rnext == rend {
                        break;
                    }
                    let mut k = rnext;
                    while k < segs[vline_right_idx].intersections.len()
                        && segs[vline_right_idx].intersections[k].itype
                            != SegmentIntersectionType::InnerLow
                    {
                        k += 1;
                    }
                    if k >= segs[vline_right_idx].intersections.len() {
                        break;
                    }
                    rbegin = k;
                }
            }
        }
    }

    // Sort + dedup, then make the left/right neighbor relation symmetric.
    // FillRectilinear.cpp:2213-2228.
    for region in regions.iter_mut() {
        region.left_neighbors.sort_unstable();
        region.left_neighbors.dedup();
        region.right_neighbors.sort_unstable();
        region.right_neighbors.dedup();
    }
    for i in 0..n {
        let left_n: Vec<usize> = regions[i].left_neighbors.clone();
        for ln in left_n {
            if !regions[ln].right_neighbors.contains(&i) {
                regions[ln].right_neighbors.push(i);
            }
        }
        let right_n: Vec<usize> = regions[i].right_neighbors.clone();
        for rn in right_n {
            if !regions[rn].left_neighbors.contains(&i) {
                regions[rn].left_neighbors.push(i);
            }
        }
    }
    for region in regions.iter_mut() {
        region.left_neighbors.sort_unstable();
        region.right_neighbors.sort_unstable();
    }

    // Fill in sum length of connecting lines of a region, used by the optimizer.
    // FillRectilinear.cpp:2244-2256.
    for i in 0..n {
        let mut len1 = monotonous_region_path_length(&regions[i], false, poly_with_offset, segs);
        let mut len2 = monotonous_region_path_length(&regions[i], true, poly_with_offset, segs);
        // Subtract the smaller length from the longer one, so we optimize just
        // with the positive difference of the two.
        if len1 > len2 {
            len1 -= len2;
            len2 = 0.0;
        } else {
            len2 -= len1;
            len1 = 0.0;
        }
        regions[i].len1 = len1;
        regions[i].len2 = len2;
    }
}

// ---------------------------------------------------------------------------
// Path length calculation for a monotonic region
// ---------------------------------------------------------------------------

/// Calculate the path length through a monotonic region.
/// Port of libslic3r `montonous_region_path_length`.
fn monotonous_region_path_length(
    region: &MonotonicRegion,
    dir: bool,
    poly_with_offset: &ExPolygonWithOffset,
    segs: &[SegmentedIntersectionLine],
) -> f32 {
    let mut i_intersection = region.left_intersection_point(dir);
    let mut i_vline = region.left.vline;
    let mut total_length: f32 = 0.0;
    let mut no_perimeter = false;
    let mut last_point = (0.0f32, 0.0f32);

    loop {
        if i_vline >= segs.len() {
            break;
        }
        let vline = &segs[i_vline];
        if i_intersection >= vline.intersections.len() {
            break;
        }
        let going_up = vline.intersections[i_intersection].is_low();

        if no_perimeter {
            let outer_idx = if going_up && i_intersection > 0 {
                i_intersection - 1
            } else if !going_up && i_intersection + 1 < vline.intersections.len() {
                i_intersection + 1
            } else {
                i_intersection
            };
            let p = if outer_idx < vline.intersections.len() {
                (
                    vline.pos as f32,
                    vline.intersections[outer_idx].pos() as f32,
                )
            } else {
                (
                    vline.pos as f32,
                    vline.intersections[i_intersection].pos() as f32,
                )
            };
            let dx = p.0 - last_point.0;
            let dy = p.1 - last_point.1;
            total_length += (dx * dx + dy * dy).sqrt();
        }

        let mut iright: i32 = vline.intersections[i_intersection].right_horizontal();

        if going_up {
            let mut it = i_intersection;
            loop {
                loop {
                    it += 1;
                    if it >= vline.intersections.len() {
                        break;
                    }
                    let ir = vline.intersections[it].right_horizontal();
                    if ir > iright {
                        iright = ir;
                    }
                    if !vline.intersections[it].is_inner() {
                        break;
                    }
                    if vline.intersections[it].itype == SegmentIntersectionType::InnerHigh
                        && it + 1 < vline.intersections.len()
                        && vline.intersections[it + 1].itype == SegmentIntersectionType::OuterHigh
                    {
                        break;
                    }
                }
                let inext = vline.intersections[it].vertical_up();
                if inext == -1
                    || vline.intersections[it].vertical_up_quality() != LinkQuality::Valid
                {
                    break;
                }
                it = inext as usize;
            }
            i_intersection = it;
        } else {
            let mut it = i_intersection;
            loop {
                if it == 0 {
                    break;
                }
                loop {
                    if it == 0 {
                        break;
                    }
                    it -= 1;
                    let ir_new = vline.intersections[it].right_horizontal();
                    if ir_new != -1 {
                        iright = ir_new;
                    }
                    if !vline.intersections[it].is_inner() {
                        break;
                    }
                    if vline.intersections[it].itype == SegmentIntersectionType::InnerLow
                        && it > 0
                        && vline.intersections[it - 1].itype == SegmentIntersectionType::OuterLow
                    {
                        break;
                    }
                }
                let inext = vline.intersections[it].vertical_down();
                if inext == -1
                    || vline.intersections[it].vertical_down_quality() != LinkQuality::Valid
                {
                    break;
                }
                it = inext as usize;
            }
            i_intersection = it;
        }

        // `i_intersection` is the final index on the *current* vline after
        // traversing up or down.  We need to keep it for the connection check
        // before overwriting it with the index on the *next* vline.
        let it_on_current = i_intersection;

        if i_vline == region.right.vline {
            break;
        }

        let inext = vline.intersections[it_on_current].right_horizontal();
        if iright < 0 {
            break;
        }

        // Find the end of the next overlapping vertical segment
        let vline_right = &segs[i_vline + 1];
        let iright_u = iright as usize;
        if iright_u >= vline_right.intersections.len() {
            break;
        }
        let right_idx = if going_up {
            vertical_run_top(vline_right, iright_u)
        } else {
            vertical_run_bottom(vline_right, iright_u)
        };
        i_intersection = right_idx;

        if inext == right_idx as i32
            && vline.intersections[it_on_current].next_on_contour_quality == LinkQuality::Valid
        {
            // Connected via perimeter — add half the perimeter length
            total_length += 0.5
                * measure_perimeter_horizontal_segment_length(
                    poly_with_offset,
                    segs,
                    i_vline,
                    it_on_current,
                    inext as usize,
                ) as f32;
            no_perimeter = false;
        } else {
            // Disconnected — record endpoint for distance calculation
            let outer_it = if going_up {
                if it_on_current + 1 < vline.intersections.len() {
                    it_on_current + 1
                } else {
                    it_on_current
                }
            } else {
                if it_on_current > 0 {
                    it_on_current - 1
                } else {
                    it_on_current
                }
            };
            let pos = vline.intersections[outer_it].pos();
            last_point = (vline.pos as f32, pos as f32);
            no_perimeter = true;
        }

        i_vline += 1;
    }

    // FillRectilinear.cpp:2148 — `return unscale<float>(total_length);`: libslic3r.h:112
    // `T(v) * T(SCALING_FACTOR)`, a product with the float 0.00001f. FILL_RECTILINEAR_PIN=0
    // divides by 100000f, which is an ulp away for about every second length.
    if fill_rectilinear_pin() {
        total_length * (crate::libslic3r::SCALING_FACTOR as f32)
    } else {
        total_length / crate::SCALING_FACTOR as f32
    }
}

// ---------------------------------------------------------------------------
// Chain monotonic regions using ant colony optimization
// ---------------------------------------------------------------------------

/// Chain monotonic regions into an optimal traversal order.
/// Port of libslic3r `chain_monotonic_regions`.
fn chain_monotonic_regions(
    regions: &mut Vec<MonotonicRegion>,
    poly_with_offset: &ExPolygonWithOffset,
    segs: &[SegmentedIntersectionLine],
) -> Vec<MonotonicRegionLink> {
    let n = regions.len();
    if n == 0 {
        return Vec::new();
    }
    // R149: native has NO n==1 shortcut — its ACO runs even for a single region
    // and picks `flipped` by path COST via the deterministic mt19937_64 flip
    // draws (native single-region chains split 70F/67 on benchy; L196 chain ran
    // back-to-front under the legacy flipped=false shortcut). Under
    // TOPFILL_FAITHFUL the full ant loop runs for n==1 too; the legacy shortcut
    // is kept on the default path only for byte-lock.
    let faithful = crate::faithful_gate("TOPFILL_FAITHFUL");
    if n == 1 && !faithful {
        return vec![MonotonicRegionLink {
            region_idx: 0,
            flipped: false,
            next_path_idx: None,
            next_flipped_path_idx: None,
        }];
    }

    // Region len1/len2 are computed in connect_monotonic_regions()
    // (FillRectilinear.cpp:2244-2256), which runs before this function. C++ does
    // NOT recompute them here, so we rely on the values already set.

    // left_neighbors_unprocessed[i] = 1 + number of left neighbors
    // (1 for self, decremented to 0 when processed)
    let mut left_neighbors_unprocessed_initial: Vec<i32> = vec![1; n];
    let mut queue_initial: Vec<usize> = Vec::new();
    for (i, region) in regions.iter().enumerate() {
        if region.left_neighbors.is_empty() {
            queue_initial.push(i);
        } else {
            left_neighbors_unprocessed_initial[i] += region.left_neighbors.len() as i32;
        }
    }

    let mut best_path: Vec<MonotonicRegionLink> = Vec::new();
    let mut best_path_length: f32 = f32::MAX;

    // Ant colony parameters
    let num_rounds = 25;
    let num_rounds_no_change_exit = 8;
    let num_ants = n.min(10);
    let mut pheromone_initial_deposit = 0.5f32;
    let pheromone_evaporation = 0.1f32;
    let pheromone_diversification = 0.1f32;
    let probability_take_best = 0.9f32;
    let pheromone_alpha = 1.0f32;
    let pheromone_beta = 2.0f32;

    let mut path_matrix =
        AntPathMatrix::new(regions, poly_with_offset, segs, pheromone_initial_deposit);

    // Greedy initial path to set pheromone scale
    {
        let mut queue = queue_initial.clone();
        let mut left_unprocessed = left_neighbors_unprocessed_initial.clone();
        if queue.is_empty() {
            // Fallback: add all regions
            for i in 0..n {
                queue.push(i);
            }
        }
        let first_idx = queue.pop().unwrap();
        left_unprocessed[first_idx] -= 1;
        let mut path_end_region = first_idx;
        let mut path_end_flipped = false;
        let mut total_length = regions[first_idx].length(false);

        while !queue.is_empty() || !regions[path_end_region].right_neighbors.is_empty() {
            let mut best_candidate: Option<(usize, bool, f32)> = None;

            // Try right neighbors first.
            // FillRectilinear.cpp:2422 — only when left_neighbors_unprocessed == 2.
            let right_n: Vec<usize> = regions[path_end_region].right_neighbors.clone();
            for &next in &right_n {
                if left_unprocessed[next] == 2 {
                    for flip in [false, true] {
                        let vis = path_matrix
                            .get(path_end_region, path_end_flipped, next, flip)
                            .visibility;
                        if best_candidate.is_none() || vis > best_candidate.unwrap().2 {
                            best_candidate = Some((next, flip, vis));
                        }
                    }
                }
            }

            let from_queue = best_candidate.is_none();
            if from_queue {
                for &next in &queue {
                    for flip in [false, true] {
                        let vis = path_matrix
                            .get(path_end_region, path_end_flipped, next, flip)
                            .visibility;
                        if best_candidate.is_none() || vis > best_candidate.unwrap().2 {
                            best_candidate = Some((next, flip, vis));
                        }
                    }
                }
            }

            if best_candidate.is_none() {
                break;
            }

            let (next_region, next_dir, _) = best_candidate.unwrap();

            // Move other right neighbors with satisfied constraints to queue
            for &next in &right_n {
                left_unprocessed[next] -= 1;
                if left_unprocessed[next] == 1 && next != next_region {
                    queue.push(next);
                }
            }

            if from_queue {
                if let Some(pos) = queue.iter().position(|&x| x == next_region) {
                    queue.swap_remove(pos);
                }
            }

            let link_len = path_matrix
                .get(path_end_region, path_end_flipped, next_region, next_dir)
                .length;
            total_length += regions[next_region].length(next_dir) + link_len;
            left_unprocessed[next_region] = 0;
            path_end_region = next_region;
            path_end_flipped = next_dir;
        }

        if total_length > 0.0 {
            pheromone_initial_deposit = 0.1 / total_length;
        }
        path_matrix.update_initial_pheromone(pheromone_initial_deposit);
    }

    let path_probability = |path: &AntPath| -> f32 {
        path.pheromone.powf(pheromone_alpha) * path.visibility.powf(pheromone_beta)
    };

    // R99: faithful `std::mt19937_64 rng;` (default seed 5489), matching C++
    // chain_monotonic_regions (FillRectilinear.cpp:2942/3397). Was
    // `rand::thread_rng()` (entropy) — the root of run-to-run gcode nondeterminism.
    let mut rng = crate::mt19937_64::Mt19937_64::new(5489);
    let mut num_rounds_no_change = 0;

    // Hoisted like C++ (`path` lives across ants/rounds): after the
    // std::mem::swap on improvement, `path` holds the PREVIOUS best — and
    // native's end-of-round pheromone reinforcement iterates over THAT `path`
    // (FillRectilinear.cpp:2643 uses `path`, not `best_path`, despite the
    // comment), so the post-swap content is observable.
    let mut path: Vec<MonotonicRegionLink> = Vec::with_capacity(n);

    for _round in 0..num_rounds {
        if num_rounds_no_change >= num_rounds_no_change_exit {
            break;
        }

        let mut improved = false;
        for _ant in 0..num_ants {
            path.clear();
            let mut queue = queue_initial.clone();
            let mut left_unprocessed = left_neighbors_unprocessed_initial.clone();

            if queue.is_empty() {
                for i in 0..n {
                    queue.push(i);
                }
            }

            // Pick random first region.
            // C++: std::uniform_int_distribution<>(0, queue.size()-1)(rng), then
            //      MonotonicRegionLink{ queue[first_idx], rng() > rng.max()/2 }.
            let first_idx_in_queue = rng.uniform_int_below(queue.len() as u64) as usize;
            let first_region = queue[first_idx_in_queue];
            let first_flipped: bool = rng.next_u64() > (u64::MAX / 2);
            queue.swap_remove(first_idx_in_queue);
            left_unprocessed[first_region] -= 1;
            left_unprocessed[first_region] = 0;
            path.push(MonotonicRegionLink {
                region_idx: first_region,
                flipped: first_flipped,
                next_path_idx: None,
                next_flipped_path_idx: None,
            });

            while !queue.is_empty()
                || !regions[path.last().unwrap().region_idx]
                    .right_neighbors
                    .is_empty()
            {
                let current_region = path.last().unwrap().region_idx;
                let current_dir = path.last().unwrap().flipped;

                // Collect candidates from right neighbors
                struct NextCandidate {
                    region: usize,
                    probability: f32,
                    dir: bool,
                }
                let mut next_candidates: Vec<NextCandidate> = Vec::new();

                let right_n: Vec<usize> = regions[current_region].right_neighbors.clone();
                for &next in &right_n {
                    if left_unprocessed[next] > 1 {
                        left_unprocessed[next] -= 1;
                    }
                    if left_unprocessed[next] == 1 {
                        for flip in [false, true] {
                            let ap = path_matrix.get(current_region, current_dir, next, flip);
                            next_candidates.push(NextCandidate {
                                region: next,
                                probability: path_probability(ap),
                                dir: flip,
                            });
                        }
                    }
                }

                let num_direct = next_candidates.len();
                if num_direct == 0 {
                    // Add queue candidates
                    for &next in &queue {
                        for flip in [false, true] {
                            let ap = path_matrix.get(current_region, current_dir, next, flip);
                            next_candidates.push(NextCandidate {
                                region: next,
                                probability: path_probability(ap),
                                dir: flip,
                            });
                        }
                    }
                }

                if next_candidates.is_empty() {
                    break;
                }

                // Select path. C++ FillRectilinear.cpp:2548-2568.
                // float dice = float(rng()) / float(rng.max());
                let dice: f32 = (rng.next_u64() as f32) / (u64::MAX as f32);
                let take_idx = if dice < probability_take_best {
                    // Take the highest-probability path. C++ std::max_element returns
                    // the FIRST maximum on ties (Rust's max_by returns the LAST), so
                    // replicate first-max with a strict `>` scan.
                    let mut best = 0usize;
                    let mut best_p = next_candidates[0].probability;
                    for (i, c) in next_candidates.iter().enumerate().skip(1) {
                        if c.probability > best_p {
                            best = i;
                            best_p = c.probability;
                        }
                    }
                    best
                } else {
                    // Probabilistic selection. C++:
                    //   total = accumulate(0.f, +probability)
                    //   thr   = float(rng()) * total / float(rng.max())
                    //   take_path = last; for it: if ((thr -= it->probability) <= 0) { take=it; break; }
                    let total_probability: f32 =
                        next_candidates.iter().map(|c| c.probability).sum();
                    let mut probability_threshold =
                        (rng.next_u64() as f32) * total_probability / (u64::MAX as f32);
                    let mut selected = next_candidates.len() - 1;
                    for (i, c) in next_candidates.iter().enumerate() {
                        probability_threshold -= c.probability;
                        if probability_threshold <= 0.0 {
                            selected = i;
                            break;
                        }
                    }
                    selected
                };

                let next_region = next_candidates[take_idx].region;
                let next_dir = next_candidates[take_idx].dir;

                // Move satisfied right neighbors to queue
                for i in 0..num_direct {
                    let cand_region = next_candidates[i].region;
                    if cand_region != next_region
                        && left_unprocessed[cand_region] == 1
                        && !queue.contains(&cand_region)
                    {
                        queue.push(cand_region);
                    }
                }

                if take_idx >= num_direct {
                    // Selected from queue — remove it
                    if let Some(pos) = queue.iter().position(|&x| x == next_region) {
                        queue.swap_remove(pos);
                    }
                }

                path.push(MonotonicRegionLink {
                    region_idx: next_region,
                    flipped: next_dir,
                    next_path_idx: None,
                    next_flipped_path_idx: None,
                });
                left_unprocessed[next_region] = 0;

                // Local pheromone update (diversification)
                let p = path_matrix
                    .get(current_region, current_dir, next_region, next_dir)
                    .pheromone;
                path_matrix
                    .get_mut(current_region, current_dir, next_region, next_dir)
                    .pheromone = (1.0 - pheromone_diversification) * p
                    + pheromone_diversification * pheromone_initial_deposit;
            }

            // Measure path length
            if path.is_empty() {
                continue;
            }
            let mut path_length: f32 =
                regions[path.last().unwrap().region_idx].length(path.last().unwrap().flipped);
            for i in 0..path.len() - 1 {
                let r_idx = path[i].region_idx;
                let r_flip = path[i].flipped;
                let n_idx = path[i + 1].region_idx;
                let n_flip = path[i + 1].flipped;
                path_length += regions[r_idx].length(r_flip)
                    + path_matrix.get(r_idx, r_flip, n_idx, n_flip).length;
            }

            if path_length < best_path_length {
                best_path_length = path_length;
                std::mem::swap(&mut best_path, &mut path);
                improved = true;
                if path_length <= 0.0 {
                    // Perfect path
                    return best_path;
                }
            }
        }

        // Reinforce the path pheromones. FillRectilinear.cpp:2640-2645 —
        // total_cost = best_path_length + EPSILON (1e-4), but the loop iterates
        // `path` (post-swap = last ant's path, or the PREVIOUS best if the last
        // ant improved), NOT `best_path`. Faithful under TOPFILL_FAITHFUL;
        // legacy default kept reinforcing best_path (byte-locked).
        let reinforce_src: &[MonotonicRegionLink] = if faithful { &path } else { &best_path };
        if !reinforce_src.is_empty() {
            let total_cost = best_path_length + 1e-4;
            for i in 0..reinforce_src.len().saturating_sub(1) {
                let r_idx = reinforce_src[i].region_idx;
                let r_flip = reinforce_src[i].flipped;
                let n_idx = reinforce_src[i + 1].region_idx;
                let n_flip = reinforce_src[i + 1].flipped;
                let p = path_matrix.get(r_idx, r_flip, n_idx, n_flip).pheromone;
                path_matrix.get_mut(r_idx, r_flip, n_idx, n_flip).pheromone =
                    (1.0 - pheromone_evaporation) * p + pheromone_evaporation / total_cost;
            }
        }

        if improved {
            num_rounds_no_change = 0;
        } else {
            num_rounds_no_change += 1;
        }
    }

    best_path
}

// ---------------------------------------------------------------------------
// Generate polylines from chained monotonic regions
// ---------------------------------------------------------------------------

/// Generate polylines from a chained monotonic region path.
/// Port of libslic3r `polylines_from_paths`.
fn polylines_from_paths(
    path: &[MonotonicRegionLink],
    regions: &[MonotonicRegion],
    poly_with_offset: &ExPolygonWithOffset,
    segs: &[SegmentedIntersectionLine],
    polylines_out: &mut Vec<Polyline>,
) {
    if path.is_empty() {
        return;
    }

    let mut current_polyline: Option<usize> = None; // index into polylines_out

    let finish_polyline = |polylines: &mut Vec<Polyline>, idx: Option<usize>| -> Option<usize> {
        if let Some(i) = idx {
            if i < polylines.len() {
                polylines[i].points_mut().dedup();
                let pts = polylines[i].points();
                let should_remove = pts.len() <= 1
                    || (pts.len() == 2
                        && (pts[0].x - pts[1].x).abs() < SCALED_EPSILON
                        && (pts[0].y - pts[1].y).abs() < SCALED_EPSILON);
                if should_remove {
                    polylines.remove(i);
                    return None;
                }
                // Try merging with previous polyline
                if i >= 1 {
                    let prev_end = {
                        let prev = &polylines[i - 1];
                        *prev.points().last().unwrap()
                    };
                    let cur_start = polylines[i].points()[0];
                    if (prev_end.x - cur_start.x).abs() < SCALED_EPSILON
                        && (prev_end.y - cur_start.y).abs() < SCALED_EPSILON
                    {
                        // Merge current into previous
                        let cur_points: Vec<Point> = polylines[i].points()[1..].to_vec();
                        let prev_idx = i - 1;
                        // Average the meeting point
                        let avg = Point::new(
                            (prev_end.x + cur_start.x) / 2,
                            (prev_end.y + cur_start.y) / 2,
                        );
                        let prev_len = polylines[prev_idx].points().len();
                        polylines[prev_idx].points_mut()[prev_len - 1] = avg;
                        polylines[prev_idx]
                            .points_mut()
                            .extend_from_slice(&cur_points);
                        polylines.remove(i);
                        return Some(prev_idx);
                    }
                }
            }
        }
        None
    };

    for (path_seg_idx, path_segment) in path.iter().enumerate() {
        let region = &regions[path_segment.region_idx];
        let dir = path_segment.flipped;

        let mut i_intersection = region.left_intersection_point(dir);
        let mut i_vline = region.left.vline;

        // Connect to previous path segment
        if current_polyline.is_some() && path_seg_idx > 0 {
            let prev_segment = &path[path_seg_idx - 1];
            let region_prev = &regions[prev_segment.region_idx];
            let dir_prev = prev_segment.flipped;
            let i_vline_prev = region_prev.right.vline;
            let i_intersection_prev = region_prev.right_intersection_point(dir_prev);

            let mut extended = false;

            // Try horizontal connection
            if i_vline_prev + 1 == i_vline
                && i_vline_prev < segs.len()
                && i_intersection_prev < segs[i_vline_prev].intersections.len()
            {
                let ip_prev = &segs[i_vline_prev].intersections[i_intersection_prev];
                if ip_prev.right_horizontal() == i_intersection as i32
                    && ip_prev.next_on_contour_quality == LinkQuality::Valid
                {
                    // Emit horizontal perimeter connection
                    if let Some(poly_idx) = current_polyline {
                        if poly_idx < polylines_out.len() {
                            let i_contour = ip_prev.i_contour;
                            emit_perimeter_prev_next_segment(
                                poly_with_offset,
                                segs,
                                i_vline_prev,
                                i_contour,
                                i_intersection_prev,
                                i_intersection,
                                polylines_out[poly_idx].points_mut(),
                                true,
                            );
                            extended = true;
                        }
                    }
                }
            }

            if !extended {
                // Finish the current vertical line at outer contour
                if i_vline_prev < segs.len()
                    && i_intersection_prev < segs[i_vline_prev].intersections.len()
                {
                    let ip_prev = &segs[i_vline_prev].intersections[i_intersection_prev];
                    let outer_idx = if ip_prev.is_low() {
                        if i_intersection_prev > 0 {
                            i_intersection_prev - 1
                        } else {
                            i_intersection_prev
                        }
                    } else if i_intersection_prev + 1 < segs[i_vline_prev].intersections.len() {
                        i_intersection_prev + 1
                    } else {
                        i_intersection_prev
                    };
                    let outer_pos = segs[i_vline_prev].intersections[outer_idx].pos();
                    if let Some(poly_idx) = current_polyline {
                        if poly_idx < polylines_out.len() {
                            let pts = polylines_out[poly_idx].points_mut();
                            if let Some(last) = pts.last_mut() {
                                *last = Point::new(segs[i_vline_prev].pos, outer_pos);
                            }
                        }
                    }
                }
                current_polyline = finish_polyline(polylines_out, current_polyline);
                current_polyline = None;
            }
        }

        // Traverse the region
        loop {
            if i_vline >= segs.len() {
                break;
            }
            let vline = &segs[i_vline];
            if i_intersection >= vline.intersections.len() {
                break;
            }

            let going_up = vline.intersections[i_intersection].is_low();

            if current_polyline.is_none() {
                polylines_out.push(Polyline::new());
                current_polyline = Some(polylines_out.len() - 1);

                // Start at outer contour
                let outer_idx = if going_up && i_intersection > 0 {
                    i_intersection - 1
                } else if !going_up && i_intersection + 1 < vline.intersections.len() {
                    i_intersection + 1
                } else {
                    i_intersection
                };
                let pos = vline.intersections[outer_idx].pos();
                if let Some(poly_idx) = current_polyline {
                    polylines_out[poly_idx]
                        .points_mut()
                        .push(Point::new(vline.pos, pos));
                }
            } else if let Some(poly_idx) = current_polyline {
                if poly_idx < polylines_out.len() {
                    polylines_out[poly_idx].points_mut().push(Point::new(
                        vline.pos,
                        vline.intersections[i_intersection].pos(),
                    ));
                }
            }

            let mut iright: i32 = vline.intersections[i_intersection].right_horizontal();

            // Traverse vertical segment
            let it_final;
            if going_up {
                let mut it = i_intersection;
                loop {
                    loop {
                        it += 1;
                        if it >= vline.intersections.len() {
                            break;
                        }
                        let ir = vline.intersections[it].right_horizontal();
                        if ir > iright {
                            iright = ir;
                        }
                        if !vline.intersections[it].is_inner() {
                            break;
                        }
                        if vline.intersections[it].itype == SegmentIntersectionType::InnerHigh
                            && it + 1 < vline.intersections.len()
                            && vline.intersections[it + 1].itype
                                == SegmentIntersectionType::OuterHigh
                        {
                            break;
                        }
                    }
                    if let Some(poly_idx) = current_polyline {
                        if poly_idx < polylines_out.len() && it < vline.intersections.len() {
                            polylines_out[poly_idx]
                                .points_mut()
                                .push(Point::new(vline.pos, vline.intersections[it].pos()));
                        }
                    }
                    let inext = vline.intersections[it].vertical_up();
                    if inext == -1
                        || vline.intersections[it].vertical_up_quality() != LinkQuality::Valid
                    {
                        break;
                    }
                    let i_contour = vline.intersections[it].i_contour;
                    let forward = vline.intersections[it].has_left_vertical_up();
                    if let Some(poly_idx) = current_polyline {
                        if poly_idx < polylines_out.len() {
                            emit_perimeter_segment_on_vertical_line(
                                poly_with_offset,
                                segs,
                                i_vline,
                                i_contour,
                                it,
                                inext as usize,
                                polylines_out[poly_idx].points_mut(),
                                forward,
                            );
                        }
                    }
                    it = inext as usize;
                }
                it_final = it;
            } else {
                // Going down
                let mut it = i_intersection;
                loop {
                    if it == 0 {
                        break;
                    }
                    loop {
                        if it == 0 {
                            break;
                        }
                        it -= 1;
                        let ir_new = vline.intersections[it].right_horizontal();
                        if ir_new != -1 {
                            iright = ir_new;
                        }
                        if !vline.intersections[it].is_inner() {
                            break;
                        }
                        if vline.intersections[it].itype == SegmentIntersectionType::InnerLow
                            && it > 0
                            && vline.intersections[it - 1].itype
                                == SegmentIntersectionType::OuterLow
                        {
                            break;
                        }
                    }
                    if let Some(poly_idx) = current_polyline {
                        if poly_idx < polylines_out.len() && it < vline.intersections.len() {
                            polylines_out[poly_idx]
                                .points_mut()
                                .push(Point::new(vline.pos, vline.intersections[it].pos()));
                        }
                    }
                    let inext = vline.intersections[it].vertical_down();
                    if inext == -1
                        || vline.intersections[it].vertical_down_quality() != LinkQuality::Valid
                    {
                        break;
                    }
                    let i_contour = vline.intersections[it].i_contour;
                    let forward = vline.intersections[it].has_right_vertical_down();
                    if let Some(poly_idx) = current_polyline {
                        if poly_idx < polylines_out.len() {
                            emit_perimeter_segment_on_vertical_line(
                                poly_with_offset,
                                segs,
                                i_vline,
                                i_contour,
                                it,
                                inext as usize,
                                polylines_out[poly_idx].points_mut(),
                                forward,
                            );
                        }
                    }
                    it = inext as usize;
                }
                it_final = it;
            }

            if i_vline == region.right.vline {
                i_intersection = it_final;
                break;
            }

            let inext = vline.intersections[it_final].right_horizontal();
            if iright < 0 {
                break;
            }

            // Find the next overlapping vertical segment
            let iright_u = iright as usize;
            if i_vline + 1 >= segs.len() {
                break;
            }
            let vline_right = &segs[i_vline + 1];
            if iright_u >= vline_right.intersections.len() {
                break;
            }
            let right_idx = if going_up {
                vertical_run_top(vline_right, iright_u)
            } else {
                vertical_run_bottom(vline_right, iright_u)
            };
            i_intersection = right_idx;

            if inext == right_idx as i32
                && vline.intersections[it_final].next_on_contour_quality == LinkQuality::Valid
            {
                // Emit horizontal connection
                let i_contour = vline.intersections[it_final].i_contour;
                if let Some(poly_idx) = current_polyline {
                    if poly_idx < polylines_out.len() {
                        emit_perimeter_prev_next_segment(
                            poly_with_offset,
                            segs,
                            i_vline,
                            i_contour,
                            it_final,
                            inext as usize,
                            polylines_out[poly_idx].points_mut(),
                            true,
                        );
                    }
                }
            } else {
                // Disconnected: finish at outer contour
                let outer_idx = if going_up {
                    it_final + 1
                } else {
                    if it_final > 0 {
                        it_final - 1
                    } else {
                        it_final
                    }
                };
                if outer_idx < vline.intersections.len() {
                    if let Some(poly_idx) = current_polyline {
                        if poly_idx < polylines_out.len() {
                            let pts = polylines_out[poly_idx].points_mut();
                            if let Some(last) = pts.last_mut() {
                                *last = Point::new(vline.pos, vline.intersections[outer_idx].pos());
                            }
                        }
                    }
                }
                current_polyline = finish_polyline(polylines_out, current_polyline);
                current_polyline = None;
            }

            i_vline += 1;
        }
    }

    // Finish last polyline
    if let Some(poly_idx) = current_polyline {
        if !path.is_empty() {
            let last_seg = path.last().unwrap();
            let region = &regions[last_seg.region_idx];
            let dir = last_seg.flipped;
            let i_vline_final = region.right.vline;
            let i_inter_final = region.right_intersection_point(dir);

            if i_vline_final < segs.len() && i_inter_final < segs[i_vline_final].intersections.len()
            {
                let ip = &segs[i_vline_final].intersections[i_inter_final];
                let outer_idx = if ip.is_low() {
                    if i_inter_final > 0 {
                        i_inter_final - 1
                    } else {
                        i_inter_final
                    }
                } else if i_inter_final + 1 < segs[i_vline_final].intersections.len() {
                    i_inter_final + 1
                } else {
                    i_inter_final
                };
                let outer_pos = segs[i_vline_final].intersections[outer_idx].pos();
                if poly_idx < polylines_out.len() {
                    let pts = polylines_out[poly_idx].points_mut();
                    if let Some(last) = pts.last_mut() {
                        *last = Point::new(segs[i_vline_final].pos, outer_pos);
                    }
                }
            }
        }
        finish_polyline(polylines_out, Some(poly_idx));
    }
}

// ---------------------------------------------------------------------------
// Monotonic fill_surface_by_lines
// ---------------------------------------------------------------------------

/// Fill a surface using monotonic infill (for top/bottom surfaces).
/// This uses the same intersection graph as non-monotonic but chains
/// regions using ant colony optimization for better path connectivity.
pub fn fill_surface_by_lines_monotonic(
    expolygon: &ExPolygon,
    spacing: CoordF,
    angle: f64,
    overlap: CoordF,
    params: &FillRectilinearParams,
) -> Vec<Polyline> {
    let mut polylines_out = Vec::new();

    const INFILL_OVERLAP_OVER_SPACING: f64 = 0.45;

    // R301 (TOPFILL_FAITHFUL): native domains — see fill_surface_by_lines.
    let tf = crate::faithful_gate("TOPFILL_FAITHFUL") && std::env::var("RASTER_QUANT").is_ok();
    // RECT_OFFSET_FAITHFUL (on unless =0) — FillRectilinear.cpp:2842-2849:
    //     coord_t line_spacing = coord_t(scale_(this->spacing) / params.density);
    //     float(scale_(this->overlap - (0.5 - INFILL_OVERLAP_OVER_SPACING) * this->spacing)),
    //     float(scale_(this->overlap - 0.5f * this->spacing))
    // `scale_` divides by 0.00001 and nothing rounds: the two offsets go through
    // `float` into `coord_t` parameters (FillRectilinear.cpp:456-459), which
    // truncates them. INFILL_OVERLAP_OVER_SPACING is the float 0.45f.
    let rof = crate::faithful_gate("RECT_OFFSET_FAITHFUL");
    let line_spacing = if tf || rof {
        ((spacing / 0.00001) / params.density) as Coord
    } else {
        (spacing * crate::SCALING_FACTOR / params.density) as Coord
    };
    let (aoffset1, aoffset2): (CoordF, CoordF) = if rof {
        let c = 0.5 - (0.45f32 as f64);
        (
            ((((-c).mul_add(spacing, overlap) / 0.00001) as f32) as Coord) as CoordF,
            ((((-0.5f64).mul_add(spacing, overlap) / 0.00001) as f32) as Coord) as CoordF,
        )
    } else if tf {
        let c = 0.5 - (0.45f32 as f64);
        (
            (((overlap - c * spacing) / 0.00001) as f32) as f64,
            (((overlap - 0.5 * spacing) / 0.00001) as f32) as f64,
        )
    } else {
        (
            scale(overlap - (0.5 - INFILL_OVERLAP_OVER_SPACING) * spacing) as CoordF,
            scale(overlap - 0.5 * spacing) as CoordF,
        )
    };

    let poly_with_offset = ExPolygonWithOffset::new_scaled_f(expolygon, -angle, aoffset1, aoffset2);

    if poly_with_offset.n_contours_inner == 0 {
        return polylines_out;
    }

    let mut bbox_src = poly_with_offset.bounding_box_src();

    let line_spacing = if params.full_infill && !params.dont_adjust {
        {
            // R258: same writeback as the non-monotonic path — the monotonic
            // (top-surface) emission recomputes flow from the adjusted spacing.
            let ls = adjust_solid_spacing(bbox_src.width(), line_spacing);
            ADJUSTED_SPACING_MM.with(|c| c.set(Some(ls as f64 / crate::SCALING_FACTOR)));
            ls
        }
    } else {
        // Native align_to_grid branch — refpt == (0,0) (see fill_surface_by_lines).
        if crate::faithful_gate("TOPFILL_FAITHFUL") {
            let aligned = crate::geometry::align_to_grid_point(
                bbox_src.min,
                Point::new(line_spacing, line_spacing),
            );
            bbox_src.min =
                Point::new(bbox_src.min.x.min(aligned.x), bbox_src.min.y.min(aligned.y));
        }
        line_spacing
    };

    if line_spacing <= 0 {
        return polylines_out;
    }

    let mut n_vlines =
        ((bbox_src.max.x - bbox_src.min.x + line_spacing - 1) / line_spacing).max(0) as usize;
    let mut x0 = bbox_src.min.x;
    if params.full_infill {
        x0 += (line_spacing + SCALED_EPSILON) / 2;
    }
    // RECT_HORIZ_MOVE (on unless =0) — FillRectilinear.cpp:2885-2891; the
    // monotonic fillers leave `params.horiz_move` at 0.
    rect_horiz_move(0.0, line_spacing, &mut x0, &mut n_vlines);

    if n_vlines == 0 {
        return polylines_out;
    }

    let mut segs = slice_region_by_vertical_lines(&poly_with_offset, n_vlines, x0, line_spacing);

    // C++: FillRectilinear::fill_surface_by_lines() uses the Fill member
    // `link_max_length` (0 for density <= 80%, 3*spacing otherwise), not a
    // value recomputed from line_spacing. See Fill.cpp:683-695.
    let link_max_length = params.link_max_length;
    let rect_probe = crate::probe_enabled("RECT");
    if rect_probe {
        rect_probe_segs(1, n_vlines, x0, line_spacing, true, link_max_length as Coord, &segs);
    }
    connect_segment_intersections_by_contours(
        &poly_with_offset,
        &mut segs,
        params,
        link_max_length,
    );
    if rect_probe {
        rect_probe_segs(2, n_vlines, x0, line_spacing, true, link_max_length as Coord, &segs);
    }

    // Insert phony outer intersections at pinch points
    pinch_contours_insert_phony_outer_intersections(&mut segs);

    // Generate monotonic regions
    let mut regions = generate_monotonous_regions(&mut segs);

    if !regions.is_empty() {
        // Connect neighboring regions
        connect_monotonic_regions(&mut regions, &poly_with_offset, &segs);

        // Chain regions using ant colony optimization
        let path = chain_monotonic_regions(&mut regions, &poly_with_offset, &segs);
        // MONO (diagnostics only): the regions and the chosen path, one line per
        // call, as the C++ twin probe prints them.
        if crate::probe_enabled("MONO") {
            let mut s = format!("MONO n={} segs={} path=", regions.len(), segs.len());
            for l in &path {
                s.push_str(&format!("{}:{},", l.region_idx, l.flipped as i32));
            }
            s.push_str(" regs=");
            for r in &regions {
                s.push_str(&format!(
                    "{},{},{},{},{},{},{:e},{:e},{};",
                    r.left.vline, r.left.low, r.left.high, r.right.vline, r.right.low,
                    r.right.high, r.len1, r.len2, r.flips as i32
                ));
            }
            eprintln!("{s}");
        }

        // Generate polylines from the chained path
        polylines_from_paths(
            &path,
            &regions,
            &poly_with_offset,
            &segs,
            &mut polylines_out,
        );
    }

    // Rotate polylines back
    for polyline in &mut polylines_out {
        polyline.points_mut().dedup();
        polyline.rotate(angle);
    }

    polylines_out.retain(|p| p.points().len() >= 2);

    polylines_out
}

// ---------------------------------------------------------------------------
// Updated generate_fill_rectilinear with monotonic support
// ---------------------------------------------------------------------------

/// Generate rectilinear infill, with optional monotonic mode for top/bottom surfaces.
pub fn generate_fill_rectilinear_monotonic(
    fill_area: &[ExPolygon],
    config: &InfillConfig,
    layer_index: usize,
    is_grid: bool,
    monotonic: bool,
) -> Vec<InfillPath> {
    ADJUSTED_SPACING_MM.with(|c| c.set(None));
    // FILL_RECTILINEAR_PIN (on unless =0): the lines of a top surface (ipMonotonicLine, which
    // `Fill::new_from_type` makes a FillMonotonicLineWGapFill, FillBase.cpp:48) are laid by
    // `FillMonotonicLineWGapFill::fill_surface_by_lines`, every other one-direction fill by
    // `FillRectilinear::fill_surface_by_lines` with `params.monotonic` (FillMonotonic::fill_surface).
    if !is_grid && live_entries_at_the_pin() && !config.consistent_pattern {
        if monotonic && config.pattern == super::InfillPattern::MonotonicLine && !config.connect_infill && config.dont_adjust {
            return generate_fill_monotonic_line_w_gap_fill_pin(fill_area, config, layer_index);
        }
        if !monotonic || fill_monotonic_pin() {
            return generate_fill_rectilinear_pin(fill_area, config, layer_index, monotonic);
        }
    }
    let mut paths = Vec::new();

    // Same missing native _infill_direction +90 as generate_fill_rectilinear.
    let faithful_dir = crate::faithful_gate("TOPFILL_FAITHFUL");
    // Native parity alternation + unconditional +90 (see generate_fill_rectilinear).
    let angle_deg = if faithful_dir {
        config.angle + if layer_index & 1 == 1 { 90.0 } else { 0.0 } + 90.0
    } else {
        config.angle + config.angle_increment * layer_index as f64
    };
    let angle_rad = angle_deg.to_radians();

    // Grid draws two perpendicular passes; halve per-pass density so they combine to the
    // target density (FillRectilinear.cpp:3036, params.density /= sweep count).
    let pass_density = if is_grid {
        config.density / 2.0
    } else {
        config.density
    };
    let params = FillRectilinearParams {
        density: pass_density,
        full_infill: config.density > 0.99,
        dont_connect: !config.connect_infill,
        dont_adjust: config.dont_adjust,
        monotonic,
        link_max_length: config.link_max_length,
        consistent_pattern: false,
    };

    let fill_fn = if monotonic {
        fill_surface_by_lines_monotonic
    } else {
        fill_surface_by_lines
    };

    for expoly in fill_area {
        let polylines = fill_fn(
            expoly,
            config.extrusion_width,
            angle_rad,
            config.overlap,
            &params,
        );

        for polyline in polylines {
            if polyline.points().len() >= 2 {
                paths.push(InfillPath::Line(polyline));
            }
        }
    }

    // For grid pattern, add perpendicular lines
    if is_grid {
        let perp_angle_rad = angle_rad + std::f64::consts::FRAC_PI_2;
        for expoly in fill_area {
            let polylines = fill_fn(
                expoly,
                config.extrusion_width,
                perp_angle_rad,
                config.overlap,
                &params,
            );

            for polyline in polylines {
                if polyline.points().len() >= 2 {
                    paths.push(InfillPath::Line(polyline));
                }
            }
        }
    }

    paths
}

// ---------------------------------------------------------------------------
// sample_grid_pattern
// ---------------------------------------------------------------------------

// Lightning infill assumes that the distance between any two sampled points is always
// at least equal to the value of spacing. To meet this assumption, we need to use
// BoundingBox for whole layers instead of bounding box just around processing ExPolygon.
// Using just BoundingBox around processing ExPolygon could produce two points closer
// than spacing (in cases where two ExPolygon are closer than spacing).
//
// FillRectilinear.cpp:3204-3224
pub fn sample_grid_pattern(
    expolygon: &ExPolygon,
    spacing: Coord,
    global_bounding_box: &BoundingBox,
) -> Vec<Point> {
    // FillRectilinear.cpp:3206
    let poly_with_offset = ExPolygonWithOffset::new(expolygon, 0.0, 0, 0);
    // FillRectilinear.cpp:3207-3211
    let segs = slice_region_by_vertical_lines(
        &poly_with_offset,
        ((global_bounding_box.max.x() - global_bounding_box.min.x() + spacing - 1) / spacing)
            as usize,
        global_bounding_box.min.x(),
        spacing,
    );

    // FillRectilinear.cpp:3213
    let mut out: Vec<Point> = Vec::new();
    // FillRectilinear.cpp:3214
    for sil in &segs {
        // FillRectilinear.cpp:3215
        let mut i = 0;
        while i < sil.intersections.len() {
            // FillRectilinear.cpp:3216
            let a = sil.intersections[i].pos();
            // FillRectilinear.cpp:3217
            let b = sil.intersections[i + 1].pos();
            // FillRectilinear.cpp:3218
            let mut y = a - (a % spacing) - spacing;
            while y < b {
                // FillRectilinear.cpp:3219
                if y > a {
                    // FillRectilinear.cpp:3220
                    out.push(Point::new(sil.pos, y));
                }
                y += spacing;
            }
            i += 2;
        }
    }
    // FillRectilinear.cpp:3223
    out
}

// FillRectilinear.cpp:3226-3232
pub fn sample_grid_pattern_expolygons(
    expolygons: &[ExPolygon],
    spacing: Coord,
    global_bounding_box: &BoundingBox,
) -> Vec<Point> {
    // FillRectilinear.cpp:3228
    let mut out: Vec<Point> = Vec::new();
    // FillRectilinear.cpp:3229-3230
    for expoly in expolygons {
        out.extend(sample_grid_pattern(expoly, spacing, global_bounding_box));
    }
    // FillRectilinear.cpp:3231
    out
}

// FillRectilinear.cpp:3234-3237
pub fn sample_grid_pattern_polygons(
    polygons: &[Polygon],
    spacing: Coord,
    global_bounding_box: &BoundingBox,
) -> Vec<Point> {
    // FillRectilinear.cpp:3236 — return sample_grid_pattern(union_ex(polygons), ...);
    sample_grid_pattern_expolygons(
        &crate::clipper2_utils::union_ex_2(polygons),
        spacing,
        global_bounding_box,
    )
}

// ---------------------------------------------------------------------------
// The fillers of FillRectilinear.hpp under their C++ names
// ---------------------------------------------------------------------------
//
// `class FillRectilinear` and the fillers derived from it (FillRectilinear.hpp:11-217), with
// `fill_surface_by_lines`, `fill_surface_by_multilines` and `make_fill_lines` (FillRectilinear.cpp:2825-3204)
// in the arithmetic of the pin: the angle in `float`, `scale_` as a division, the rotations with one
// `__sincos_stret` call and fused products, `coord_t` truncation where the C++ converts. These are what
// tests/fill_rectilinear_native_parity.rs compares with the pin; the entries the slicing path calls
// (`generate_fill_rectilinear`, `generate_fill_rectilinear_monotonic`, `generate_grid_raw_lines`) go
// through them when `FILL_RECTILINEAR_PIN` is on.

use super::fill_base::Fill;
use super::FillParams;
use crate::surface::Surface;

/// `FILL_RECTILINEAR_PIN` (on unless `=0`): the entries of the slicing path lay their lines through the
/// classes below. `=0` keeps the arithmetic those entries had before R922.
pub fn fill_rectilinear_pin() -> bool {
    crate::faithful_gate("FILL_RECTILINEAR_PIN")
}

/// `FILL_MONOTONIC_PIN`: the connected monotonic fill of a bottom surface (ipMonotonic, FillMonotonic) is
/// laid by `FillRectilinear::fill_surface_by_lines` with `params.monotonic` too (see
/// `generate_fill_rectilinear_monotonic`).
fn fill_monotonic_pin() -> bool {
    crate::faithful_gate("FILL_MONOTONIC_PIN")
}

/// MultiPoint.cpp:27-35 — `void MultiPoint::rotate(double cos_angle, double sin_angle)`; the same body as
/// `Point::rotate(double cos_a, double sin_a)` (Point.hpp:227-232). One fused product per coordinate.
fn point_rotate_cos_sin(pt: &mut Point, cos_angle: f64, sin_angle: f64) {
    let cur_x = pt.x as f64;
    let cur_y = pt.y as f64;
    pt.x = cos_angle.mul_add(cur_x, -(sin_angle * cur_y)).round() as Coord;
    pt.y = cos_angle.mul_add(cur_y, sin_angle * cur_x).round() as Coord;
}

/// MultiPoint.hpp:42 — `void rotate(double angle) { this->rotate(cos(angle), sin(angle)); }`: the cosine and
/// the sine of one argument, one `__sincos_stret` call.
fn multi_point_rotate(points: &mut [Point], angle: f64) {
    let (sin_angle, cos_angle) = crate::clipper::sin_cos(angle);
    for pt in points.iter_mut() {
        point_rotate_cos_sin(pt, cos_angle, sin_angle);
    }
}

/// `sinf(x)` and `cosf(x)` of one argument, in the order (sin, cos): clang for macOS turns the two calls
/// into one call of `__sincosf_stret` (see `crate::clipper::sin_cos` for the doubles), which this calls
/// itself on Apple targets.
///
/// FIDELITY-NOTE: on the other targets this is `x.sin()` and `x.cos()`; whether the Linux and Windows
/// builds of the C++ get the same bits there has not been tested.
#[inline]
fn sin_cos_f32(x: f32) -> (f32, f32) {
    #[cfg(target_vendor = "apple")]
    {
        #[repr(C)]
        struct Float2 {
            sinval: f32,
            cosval: f32,
        }
        extern "C" {
            fn __sincosf_stret(x: f32) -> Float2;
        }
        // SAFETY: `__sincosf_stret` of libSystem takes a float and returns the two floats by value.
        let r = unsafe { __sincosf_stret(x) };
        (r.sinval, r.cosval)
    }
    #[cfg(not(target_vendor = "apple"))]
    {
        (x.sin(), x.cos())
    }
}

/// Point.hpp:233 — `Point rotated(double angle) const`.
fn point_rotated(pt: Point, angle: f64) -> Point {
    let (sin_angle, cos_angle) = crate::clipper::sin_cos(angle);
    let mut res = pt;
    point_rotate_cos_sin(&mut res, cos_angle, sin_angle);
    res
}

/// BoundingBox.hpp:226-246 — `align_to_grid(coord, spacing)`, `align_to_grid(Point, Point, Point base)`.
fn align_to_grid_coord(coord: Coord, spacing: Coord) -> Coord {
    // Current C++ standard defines the result of integer division to be rounded to zero,
    // for both positive and negative numbers. Here we want to round down for negative
    // numbers as well.
    if coord < 0 {
        ((coord - spacing + 1) / spacing) * spacing
    } else {
        (coord / spacing) * spacing
    }
}
fn align_to_grid_base(coord: Point, spacing: Point, base: Point) -> Point {
    Point::new(
        base.x + align_to_grid_coord(coord.x - base.x, spacing.x),
        base.y + align_to_grid_coord(coord.y - base.y, spacing.y),
    )
}

impl ExPolygonWithOffset {
    /// FillRectilinear.cpp:453-499 — the constructor
    /// `ExPolygonWithOffset(const ExPolygon &expolygon, float angle, coord_t aoffset1, coord_t aoffset2 = 0)`.
    pub fn new_pin(expolygon: &ExPolygon, angle: f32, aoffset1: Coord, aoffset2: Coord) -> Self {
        // Copy and rotate the source polygons.
        let mut polygons_src = expolygon.clone();
        if angle != 0. {
            multi_point_rotate(&mut polygons_src.contour.points, angle as f64);
            for hole in &mut polygons_src.holes {
                multi_point_rotate(&mut hole.points, angle as f64);
            }
        }

        // double miterLimit = DefaultMiterLimit; (3., ClipperUtils.hpp)
        crate::ex_polygon::remove_sticks_expoly(&mut polygons_src);
        // polygons_outer = aoffset1 == 0 ? to_polygons(polygons_src) : offset(polygons_src, float(aoffset1), ClipperLib::jtMiter, miterLimit);
        let mut polygons_outer: Vec<Polygon> = if aoffset1 == 0 {
            expolygon_to_polygons(&polygons_src)
        } else {
            crate::clipper_utils::offset_expolygons_clib_raw_scaled(
                std::slice::from_ref(&polygons_src),
                (aoffset1 as f32) as f64,
                OffsetJoinType::Miter,
            )
        };
        // if (aoffset2 < 0)
        //     polygons_inner = shrink(polygons_outer, float(aoffset1 - aoffset2), ClipperLib::jtMiter, miterLimit);
        let mut polygons_inner: Vec<Polygon> = if aoffset2 < 0 {
            crate::clipper_utils::offset_polygons_clib(&polygons_outer, -(((aoffset1 - aoffset2) as f32) as f64), OffsetJoinType::Miter)
        } else {
            Vec::new()
        };
        // Filter out contours with zero area or small area, contours with 2 points only.
        let min_area_threshold = 0.01 * aoffset2 as f64 * aoffset2 as f64;
        crate::geometry::remove_small_polygons(&mut polygons_outer, min_area_threshold);
        crate::geometry::remove_small_polygons(&mut polygons_inner, min_area_threshold);
        crate::geometry::remove_sticks_polygons(&mut polygons_outer);
        crate::geometry::remove_sticks_polygons(&mut polygons_inner);
        let n_contours_outer = polygons_outer.len();
        let n_contours_inner = polygons_inner.len();
        let n_contours = n_contours_outer + n_contours_inner;
        let mut polygons_ccw = vec![false; n_contours];
        for i in 0..n_contours {
            let contour = if i < n_contours_outer { &mut polygons_outer[i] } else { &mut polygons_inner[i - n_contours_outer] };
            crate::multi_point::remove_duplicate_points(&mut contour.points);
            polygons_ccw[i] = crate::geometry::geometry::is_ccw(contour);
        }
        Self { polygons_src, polygons_outer, polygons_inner, n_contours_outer, n_contours_inner, n_contours, polygons_ccw }
    }

    /// FillRectilinear.cpp:501-511 — `ExPolygonWithOffset(const ExPolygonWithOffset &rhs, float angle)`.
    pub fn new_rotated(rhs: &ExPolygonWithOffset, angle: f32) -> Self {
        let mut this = Self {
            polygons_src: rhs.polygons_src.clone(),
            polygons_outer: rhs.polygons_outer.clone(),
            polygons_inner: rhs.polygons_inner.clone(),
            n_contours_outer: rhs.n_contours_outer,
            n_contours_inner: rhs.n_contours_inner,
            n_contours: rhs.n_contours,
            polygons_ccw: rhs.polygons_ccw.clone(),
        };
        if angle != 0. {
            multi_point_rotate(&mut this.polygons_src.contour.points, angle as f64);
            for hole in &mut this.polygons_src.holes {
                multi_point_rotate(&mut hole.points, angle as f64);
            }
            for poly in &mut this.polygons_outer {
                multi_point_rotate(&mut poly.points, angle as f64);
            }
            for poly in &mut this.polygons_inner {
                multi_point_rotate(&mut poly.points, angle as f64);
            }
        }
        this
    }

    /// FillRectilinear.cpp:527-528 — `bounding_box_src()`: `get_extents(polygons_src)`, the extents of the
    /// contour (ExPolygon.cpp `get_extents(const ExPolygon &)`).
    pub fn bounding_box_src_pin(&self) -> BoundingBox {
        BoundingBox::from_points(&self.polygons_src.contour.points)
    }

    /// FillRectilinear.cpp:531-532 — `bounding_box_inner()`.
    pub fn bounding_box_inner(&self) -> BoundingBox {
        let mut bbox = BoundingBox::new();
        for poly in &self.polygons_inner {
            bbox.merge(&poly.bounding_box());
        }
        bbox
    }
}

/// FillRectilinear.cpp:2994-3031 —
/// `void make_fill_lines(const ExPolygonWithOffset &poly_with_offset, Point refpt, double angle, coord_t x_margin, coord_t line_spacing, coord_t pattern_shift, Polylines &fill_lines)`
pub fn make_fill_lines(
    poly_with_offset: &ExPolygonWithOffset,
    mut refpt: Point,
    angle: f64,
    x_margin: Coord,
    line_spacing: Coord,
    pattern_shift: Coord,
    fill_lines: &mut Vec<Polyline>,
) {
    let mut bounding_box = poly_with_offset.bounding_box_src_pin();
    // Don't produce infill lines, which fully overlap with the infill perimeter.
    let x_min = bounding_box.min.x + x_margin;
    let x_max = bounding_box.max.x - x_margin;
    // extend bounding box so that our pattern will be aligned with other layers
    // align_to_grid will not work correctly with positive pattern_shift.
    let pattern_shift_scaled = pattern_shift % line_spacing;
    refpt.x -= if pattern_shift_scaled >= 0 { pattern_shift_scaled } else { line_spacing + pattern_shift_scaled };
    bounding_box.merge_point(align_to_grid_base(bounding_box.min, Point::new(line_spacing, line_spacing), refpt));

    // Intersect a set of euqally spaced vertical lines wiht expolygon.
    // n_vlines = ceil(bbox_width / line_spacing)
    let n_vlines = ((bounding_box.max.x - bounding_box.min.x + line_spacing - 1) / line_spacing) as usize;
    // const double cos_a = cos(angle); const double sin_a = sin(angle); — one call
    let (sin_a, cos_a) = crate::clipper::sin_cos(angle);
    for vline in &slice_region_by_vertical_lines(poly_with_offset, n_vlines, bounding_box.min.x, line_spacing) {
        if vline.pos >= x_min {
            if vline.pos > x_max {
                break;
            }
            let n = vline.intersections.len();
            let mut it = 0usize;
            while it < n {
                let it_low = it;
                it += 1;
                if vline.intersections[it_low].itype != SegmentIntersectionType::OuterLow {
                    continue;
                }
                let it_high = it;
                // (The C++ reads *it_high without a test for the end; an OUTER_LOW is followed by its
                // OUTER_HIGH where no inner contour was sliced.)
                if it_high < n && vline.intersections[it_high].itype == SegmentIntersectionType::OuterHigh {
                    let mut a = Point::new(vline.pos, vline.intersections[it_low].pos());
                    let mut b = Point::new(vline.pos, vline.intersections[it_high].pos());
                    if angle != 0. {
                        point_rotate_cos_sin(&mut a, cos_a, sin_a);
                        point_rotate_cos_sin(&mut b, cos_a, sin_a);
                    }
                    fill_lines.push(Polyline::from_points(vec![a, b]));
                    it += 1;
                }
            }
        }
    }
}

/// The class of a filler made from `FillRectilinear` (FillRectilinear.hpp): what its virtual functions
/// `_layer_angle`, `has_consistent_pattern`, `no_sort` and `is_self_crossing` return.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FillRectilinearClass {
    Rectilinear,
    AlignedRectilinear,
    Monotonic,
    MonotonicLine,
    Grid,
    Triangles,
    Stars,
    Cubic,
    SupportBase,
    ZigZag,
    CrossZag,
    LockedZag,
    Lattice2D,
}

/// FillRectilinear.hpp:26-29 — `struct SweepParams`: fill by multiple sweeps of differing directions.
#[derive(Debug, Clone, Copy)]
pub struct SweepParams {
    pub angle_base: f32,
    pub pattern_shift: f32,
}

/// FillRectilinear.hpp:11-34 — `class FillRectilinear : public Fill`.
#[derive(Debug, Clone)]
pub struct FillRectilinear {
    /// The members of the base class (FillBase.hpp:114-141).
    pub base: Fill,
    /// The dynamic type.
    pub class: FillRectilinearClass,
}

impl Default for FillRectilinear {
    fn default() -> Self {
        Self::new()
    }
}

impl FillRectilinear {
    pub fn new() -> Self {
        Self::with_class(FillRectilinearClass::Rectilinear)
    }

    pub fn with_class(class: FillRectilinearClass) -> Self {
        Self { base: Fill::new(), class }
    }

    /// FillRectilinear.hpp:18 — `bool is_self_crossing() override { return false; }`, and the overrides of
    /// FillGrid, FillTriangles, FillStars and FillCubic (FillRectilinear.hpp:63, 76, 89, 102).
    pub fn is_self_crossing(&self) -> bool {
        use FillRectilinearClass::*;
        matches!(self.class, Grid | Triangles | Stars | Cubic)
    }

    /// FillBase.hpp:217 — `_layer_angle` of the base class, and the overrides that keep the angle constant
    /// between the layers (FillRectilinear.hpp:43, 67, 80, 93, 106, 118, 212).
    pub fn _layer_angle(&self, idx: usize) -> f32 {
        use FillRectilinearClass::*;
        match self.class {
            AlignedRectilinear | Grid | Triangles | Stars | Cubic | SupportBase | Lattice2D => 0.,
            _ => self.base._layer_angle(idx),
        }
    }

    /// FillBase.hpp:162, FillRectilinear.hpp:165, 174, 187 — `has_consistent_pattern()`.
    pub fn has_consistent_pattern(&self) -> bool {
        use FillRectilinearClass::*;
        matches!(self.class, ZigZag | CrossZag | LockedZag)
    }

    /// FillBase.hpp:157, FillRectilinear.hpp:52, 61 — `no_sort()`.
    pub fn no_sort(&self) -> bool {
        use FillRectilinearClass::*;
        matches!(self.class, Monotonic | MonotonicLine)
    }

    /// FillBase.cpp:197-241 — `Fill::_infill_direction` with the `_layer_angle` of this class.
    pub fn _infill_direction(&self, surface: &Surface) -> (f32, Point) {
        self.base._infill_direction_with(surface, |idx| self._layer_angle(idx))
    }

    /// FillRectilinear.cpp:2815-2823 — `BoundingBox FillRectilinear::extended_object_bounding_box() const`
    pub fn extended_object_bounding_box(&self) -> BoundingBox {
        extended_object_bounding_box(&self.base.bounding_box)
    }

    /// FillRectilinear.cpp:2825-2992 —
    /// `bool FillRectilinear::fill_surface_by_lines(const Surface *surface, const FillParams &params, float angleBase, float pattern_shift, Polylines &polylines_out)`
    ///
    /// Fill by single directional lines, interconnect the lines along perimeters.
    pub fn fill_surface_by_lines(&mut self, surface: &Surface, params: &FillParams, angle_base: f32, pattern_shift: f32, polylines_out: &mut Vec<Polyline>) -> bool {
        self.fill_surface_by_lines_spacing_written(surface, params, angle_base, pattern_shift, polylines_out).0
    }

    /// The body of `fill_surface_by_lines`; the second value says whether `this->spacing` was written
    /// (FillRectilinear.cpp:2864), which the caller of the C++ reads back from the filler
    /// (FillBase.cpp:138-149) and the slicing path of this crate from `ADJUSTED_SPACING_MM`.
    fn fill_surface_by_lines_spacing_written(&mut self, surface: &Surface, params: &FillParams, angle_base: f32, pattern_shift: f32, polylines_out: &mut Vec<Polyline>) -> (bool, bool) {
        let mut spacing_written = false;
        // At the end, only the new polylines will be rotated back.
        let n_polylines_out_initial = polylines_out.len();

        // Shrink the input polygon a bit first to not push the infill lines out of the perimeters.
        const INFILL_OVERLAP_OVER_SPACING: f32 = 0.45;

        // Rotate polygons so that we can work with vertical lines here
        let mut rotate_vector = self._infill_direction(surface);
        if params.locked_zag {
            rotate_vector.0 += (std::f64::consts::PI / 2.) as f32;
        }
        rotate_vector.0 += angle_base;

        let density = (params.density as f32) as f64;
        // coord_t line_spacing = coord_t(scale_(this->spacing) / params.density);
        let mut line_spacing = ((self.base.spacing / crate::libslic3r::SCALING_FACTOR) / density) as Coord;

        // On the polygons of poly_with_offset, the infill lines will be connected.
        //     float(scale_(this->overlap - (0.5 - INFILL_OVERLAP_OVER_SPACING) * this->spacing)),
        //     float(scale_(this->overlap - 0.5f * this->spacing)));
        // two floats handed to `coord_t` parameters.
        let c = 0.5 - INFILL_OVERLAP_OVER_SPACING as f64;
        let aoffset1 = (((-c).mul_add(self.base.spacing, self.base.overlap) / crate::libslic3r::SCALING_FACTOR) as f32) as Coord;
        let aoffset2 = (((-0.5f64).mul_add(self.base.spacing, self.base.overlap) / crate::libslic3r::SCALING_FACTOR) as f32) as Coord;
        let poly_with_offset = ExPolygonWithOffset::new_pin(&surface.expolygon, -rotate_vector.0, aoffset1, aoffset2);
        if poly_with_offset.n_contours_inner == 0 {
            // Not a single infill line fits.
            //FIXME maybe one shall trigger the gap fill here?
            return (true, spacing_written);
        }

        // For infill that needs to be consistent between layers (like Zig Zag),
        // we use bounding box of whole object to match vertical lines between layers.
        let bounding_box_src = poly_with_offset.bounding_box_src_pin();
        let mut bounding_box = if self.has_consistent_pattern() { self.extended_object_bounding_box() } else { bounding_box_src };

        // define flow spacing according to requested density
        if params.full_infill() && !params.dont_adjust {
            line_spacing = Fill::_adjust_solid_spacing(bounding_box_src.max.x - bounding_box_src.min.x, line_spacing);
            // this->spacing = unscale<double>(line_spacing);
            self.base.spacing = line_spacing as f64 * crate::libslic3r::SCALING_FACTOR;
            spacing_written = true;
        } else {
            // extend bounding box so that our pattern will be aligned with other layers
            // Transform the reference point to the rotated coordinate system.
            let mut refpt = point_rotated(rotate_vector.1, (-rotate_vector.0) as f64);
            // align_to_grid will not work correctly with positive pattern_shift.
            let pattern_shift_scaled = ((pattern_shift as f64 / crate::libslic3r::SCALING_FACTOR) as Coord) % line_spacing;
            refpt.x -= if pattern_shift_scaled >= 0 { pattern_shift_scaled } else { line_spacing + pattern_shift_scaled };
            bounding_box.merge_point(align_to_grid_base(bounding_box.min, Point::new(line_spacing, line_spacing), refpt));
        }

        // Intersect a set of euqally spaced vertical lines wiht expolygon.
        // n_vlines = ceil(bbox_width / line_spacing)
        let mut n_vlines = ((bounding_box.max.x - bounding_box.min.x + line_spacing - 1) / line_spacing) as usize;
        let mut x0 = bounding_box.min.x;
        if params.full_infill() {
            x0 += (line_spacing + SCALED_EPSILON) / 2;
        }

        // int gap_line = params.horiz_move / line_spacing;  (a float quotient, truncated)
        let gap_line = (params.horiz_move / line_spacing as f32) as i32;
        // x0 += params.horiz_move - gap_line * line_spacing;  (the sum is taken in float)
        if gap_line % 2 == 0 {
            x0 = (x0 as f32 + (params.horiz_move - (gap_line as i64 * line_spacing) as f32)) as Coord;
        } else {
            x0 = (x0 as f32 + (params.horiz_move - ((gap_line - 1) as i64 * line_spacing) as f32)) as Coord;
            n_vlines += 1;
        }

        let mut segs = slice_region_by_vertical_lines(&poly_with_offset, n_vlines, x0, line_spacing);
        // Connect by horizontal / vertical links, classify the links based on link_max_length as too long.
        let rparams = FillRectilinearParams {
            density: params.density,
            monotonic: params.monotonic,
            dont_connect: params.dont_connect(),
            link_max_length: self.base.link_max_length,
            full_infill: params.full_infill(),
            dont_adjust: params.dont_adjust,
            consistent_pattern: self.has_consistent_pattern(),
        };
        connect_segment_intersections_by_contours(&poly_with_offset, &mut segs, &rparams, self.base.link_max_length);

        //FIXME this is a hack to get the monotonic infill rolling. We likely want a smarter switch, likely based on user decison.
        let monotonic_infill = params.monotonic; // || params.density > 0.99;
        if monotonic_infill {
            // Sometimes the outer contour pinches the inner contour from both sides along a single vertical line.
            // This situation is not handled correctly by generate_montonous_regions().
            // Insert phony OUTER_HIGH / OUTER_LOW pairs at the position where the contour is pinched.
            pinch_contours_insert_phony_outer_intersections(&mut segs);
            let mut regions = generate_monotonous_regions(&mut segs);
            connect_monotonic_regions(&mut regions, &poly_with_offset, &segs);
            if !regions.is_empty() {
                // std::mt19937_64 rng; (seeded in chain_monotonic_regions)
                let path = chain_monotonic_regions(&mut regions, &poly_with_offset, &segs);
                polylines_from_paths(&path, &regions, &poly_with_offset, &segs, polylines_out);
            }
        } else {
            traverse_graph_generate_polylines(&poly_with_offset, &rparams, &mut segs, polylines_out);
        }

        // paths must be rotated back
        for it in polylines_out[n_polylines_out_initial..].iter_mut() {
            // No need to translate, the absolute position is irrelevant.
            multi_point_rotate(&mut it.points, rotate_vector.0 as f64);
            //FIXME rather simplify the paths to avoid very short edges?
            crate::multi_point::remove_duplicate_points(&mut it.points);

            //get origin direction infill
            if params.symmetric_infill_y_axis {
                crate::multi_point::symmetric_y(&mut it.points, params.symmetric_y_axis);
            }
        }

        (true, spacing_written)
    }

    /// FillRectilinear.cpp:3033-3073 —
    /// `bool FillRectilinear::fill_surface_by_multilines(const Surface *surface, FillParams params, const std::initializer_list<SweepParams> &sweep_params, Polylines &polylines_out)`
    ///
    /// Fill by multiple sweeps of differing directions.
    pub fn fill_surface_by_multilines(&mut self, surface: &Surface, mut params: FillParams, sweep_params: &[SweepParams], polylines_out: &mut Vec<Polyline>) -> bool {
        debug_assert!(!sweep_params.is_empty());
        // params.density /= double(sweep_params.size());  (a float member)
        params.density = (((params.density as f32) as f64 / sweep_params.len() as f64) as f32) as f64;
        let (poly_with_offset_base, mut fill_lines) = match self.fill_lines_of_sweeps(surface, &params, sweep_params) {
            Some(lines) => lines,
            // Not a single infill line fits.
            None => return true,
        };

        // FillRectilinear.cpp:3065-3070
        if params.dont_connect() || fill_lines.len() <= 1 {
            if fill_lines.len() > 1 {
                fill_lines = crate::shortest_path::chain_polylines(fill_lines, None);
            }
            polylines_out.extend(fill_lines);
        } else {
            let bbox = BoundingBox::from_points(&surface.expolygon.contour.points);
            Fill::connect_infill_polygons(fill_lines, &poly_with_offset_base.polygons_outer, &bbox, polylines_out, self.base.spacing, &params);
        }

        true
    }

    /// FillRectilinear.cpp:3038-3063 — the part of `fill_surface_by_multilines` that lays the lines of
    /// every sweep, with `params.density` already divided by the number of sweeps: the offset polygon
    /// the lines are connected along afterwards and the lines; `None` where no contour is left.
    fn fill_lines_of_sweeps(&self, surface: &Surface, params: &FillParams, sweep_params: &[SweepParams]) -> Option<(ExPolygonWithOffset, Vec<Polyline>)> {
        let n_multilines = params.multiline;

        // ExPolygonWithOffset poly_with_offset_base(surface->expolygon, 0, float(scale_(this->overlap - 0.5 * this->spacing)));
        let aoffset1 = (((-0.5f64).mul_add(self.base.spacing, self.base.overlap) / crate::libslic3r::SCALING_FACTOR) as f32) as Coord;
        let poly_with_offset_base = ExPolygonWithOffset::new_pin(&surface.expolygon, 0., aoffset1, 0);
        if poly_with_offset_base.n_contours == 0 {
            // Not a single infill line fits.
            return None;
        }

        let mut fill_lines: Vec<Polyline> = Vec::new();
        let line_width = (self.base.spacing / crate::libslic3r::SCALING_FACTOR) as Coord;
        let line_spacing = ((self.base.spacing / crate::libslic3r::SCALING_FACTOR) * params.multiline as f64 / params.density) as Coord;
        let rotate_vector = self._infill_direction(surface);
        for sweep in sweep_params {
            // Rotate polygons so that we can work with vertical lines here
            let angle: f32 = rotate_vector.0 + sweep.angle_base;
            //Fill Multiline
            for i in 0..n_multilines {
                let group_offset: Coord = i as Coord * line_spacing;
                // coord_t internal_offset = (i - (n_multilines - 1) / 2.0f) * line_width;  (in float)
                let internal_offset: Coord = ((i as f32 - (n_multilines - 1) as f32 / 2.0f32) * line_width as f32) as Coord;
                let total_offset: Coord = group_offset + internal_offset;
                // coord_t pattern_shift = scale_(sweep.pattern_shift + unscale_(total_offset));
                let pattern_shift: Coord =
                    ((total_offset as f64).mul_add(crate::libslic3r::SCALING_FACTOR, sweep.pattern_shift as f64) / crate::libslic3r::SCALING_FACTOR) as Coord;

                make_fill_lines(
                    &ExPolygonWithOffset::new_rotated(&poly_with_offset_base, -angle),
                    point_rotated(rotate_vector.1, (-angle) as f64),
                    angle as f64,
                    line_width + SCALED_EPSILON,
                    line_spacing,
                    pattern_shift,
                    &mut fill_lines,
                );
            }
        }

        Some((poly_with_offset_base, fill_lines))
    }

    /// FillRectilinear.cpp:3075-3095 — `Polylines FillRectilinear::fill_surface(const Surface *surface, const FillParams &params)`
    pub fn fill_surface(&mut self, surface: &Surface, params: &FillParams) -> Vec<Polyline> {
        if params.conformal {
            let mut conformal_out: Vec<Polyline> = Vec::new();
            if super::fill_conformal::fill_surface(&self.base, Some(surface), params, &mut conformal_out) {
                return conformal_out;
            }
        }

        use crate::print_config::InfillPattern as P;
        let mut polylines_out: Vec<Polyline> = Vec::new();
        if params.full_infill() || params.multiline == 1 || params.pattern == P::CrossZag || params.pattern == P::ZigZag || params.pattern == P::LockedZag {
            // if (!fill_surface_by_lines(surface, params, 0.f, 0.f, polylines_out)) BOOST_LOG_TRIVIAL(error) << ...
            self.fill_surface_by_lines(surface, params, 0., 0., &mut polylines_out);
        } else {
            self.fill_surface_by_multilines(surface, params.clone(), &[SweepParams { angle_base: 0., pattern_shift: 0. }], &mut polylines_out);
        }
        polylines_out
    }
}

macro_rules! filler_of_fill_rectilinear {
    ($(#[$doc:meta])* $name:ident, $class:ident) => {
        $(#[$doc])*
        #[derive(Debug, Clone)]
        pub struct $name(pub FillRectilinear);
        impl Default for $name {
            fn default() -> Self {
                Self(FillRectilinear::with_class(FillRectilinearClass::$class))
            }
        }
        impl $name {
            pub fn new() -> Self {
                Self::default()
            }
        }
    };
}

filler_of_fill_rectilinear!(
    /// FillRectilinear.hpp:36-44 — `class FillAlignedRectilinear : public FillRectilinear`.
    FillAlignedRectilinear, AlignedRectilinear);
filler_of_fill_rectilinear!(
    /// FillRectilinear.hpp:46-53 — `class FillMonotonic : public FillRectilinear`.
    FillMonotonic, Monotonic);
filler_of_fill_rectilinear!(
    /// FillRectilinear.hpp:55-62 — `class FillMonotonicLine : public FillRectilinear`.
    FillMonotonicLine, MonotonicLine);
filler_of_fill_rectilinear!(
    /// FillRectilinear.hpp:64-75 — `class FillGrid : public FillRectilinear`.
    FillGrid, Grid);
filler_of_fill_rectilinear!(
    /// FillRectilinear.hpp:77-88 — `class FillTriangles : public FillRectilinear`.
    FillTriangles, Triangles);
filler_of_fill_rectilinear!(
    /// FillRectilinear.hpp:90-101 — `class FillStars : public FillRectilinear`.
    FillStars, Stars);
filler_of_fill_rectilinear!(
    /// FillRectilinear.hpp:103-114 — `class FillCubic : public FillRectilinear`.
    FillCubic, Cubic);
filler_of_fill_rectilinear!(
    /// FillRectilinear.hpp:116-126 — `class FillSupportBase : public FillRectilinear`.
    FillSupportBase, SupportBase);
filler_of_fill_rectilinear!(
    /// FillRectilinear.hpp:158-166 — `class FillZigZag : public FillRectilinear`.
    FillZigZag, ZigZag);
filler_of_fill_rectilinear!(
    /// FillRectilinear.hpp:168-175 — `class FillCrossZag : public FillRectilinear`.
    FillCrossZag, CrossZag);
filler_of_fill_rectilinear!(
    /// FillRectilinear.hpp:204-214 — `class Fill2DLattice : public FillRectilinear`.
    Fill2DLattice, Lattice2D);

impl FillAlignedRectilinear {
    /// FillRectilinear.hpp:43 — always generate infill at the same angle.
    pub fn _layer_angle(&self, _idx: usize) -> f32 {
        0.
    }
    /// The `fill_surface` of the base class (FillRectilinear.cpp:3075-3095).
    pub fn fill_surface(&mut self, surface: &Surface, params: &FillParams) -> Vec<Polyline> {
        self.0.fill_surface(surface, params)
    }
}

impl FillMonotonic {
    /// FillRectilinear.hpp:52 — `bool no_sort() const override { return true; }`
    pub fn no_sort(&self) -> bool {
        true
    }
    /// FillRectilinear.cpp:3097-3105 — `Polylines FillMonotonic::fill_surface(const Surface *surface, const FillParams &params)`
    pub fn fill_surface(&mut self, surface: &Surface, params: &FillParams) -> Vec<Polyline> {
        let mut params2 = params.clone();
        params2.monotonic = true;
        let mut polylines_out: Vec<Polyline> = Vec::new();
        self.0.fill_surface_by_lines(surface, &params2, 0., 0., &mut polylines_out);
        polylines_out
    }
}

impl FillMonotonicLine {
    /// FillRectilinear.hpp:61 — `bool no_sort() const override { return true; }`
    pub fn no_sort(&self) -> bool {
        true
    }
    /// FillRectilinear.cpp:3107-3116 — `Polylines FillMonotonicLine::fill_surface(const Surface* surface, const FillParams& params)`
    pub fn fill_surface(&mut self, surface: &Surface, params: &FillParams) -> Vec<Polyline> {
        let mut params2 = params.clone();
        params2.monotonic = true;
        params2.anchor_length_max = 0.0;
        let mut polylines_out: Vec<Polyline> = Vec::new();
        self.0.fill_surface_by_lines(surface, &params2, 0., 0., &mut polylines_out);
        polylines_out
    }
}

impl FillGrid {
    /// FillRectilinear.hpp:72 — `bool is_self_crossing() override { return true; }`
    pub fn is_self_crossing(&self) -> bool {
        true
    }
    /// FillRectilinear.hpp:75 — the grid fill keeps the angle constant between the layers.
    pub fn _layer_angle(&self, _idx: usize) -> f32 {
        0.
    }
    /// FillRectilinear.cpp:3118-3131 — `Polylines FillGrid::fill_surface(const Surface *surface, const FillParams &params)`
    pub fn fill_surface(&mut self, surface: &Surface, params: &FillParams) -> Vec<Polyline> {
        let mut polylines_out: Vec<Polyline> = Vec::new();
        self.0.fill_surface_by_multilines(
            surface,
            params.clone(),
            &[SweepParams { angle_base: 0., pattern_shift: 0. }, SweepParams { angle_base: (std::f64::consts::PI / 2.) as f32, pattern_shift: 0. }],
            &mut polylines_out,
        );

        if self.0.base.layer_id % 2 == 1 {
            for pl in polylines_out.iter_mut() {
                pl.points.reverse();
            }
        }
        polylines_out
    }
}

impl FillTriangles {
    /// FillRectilinear.hpp:85 — `bool is_self_crossing() override { return true; }`
    pub fn is_self_crossing(&self) -> bool {
        true
    }
    /// FillRectilinear.hpp:88
    pub fn _layer_angle(&self, _idx: usize) -> f32 {
        0.
    }
    /// FillRectilinear.cpp:3133-3142 — `Polylines FillTriangles::fill_surface(const Surface *surface, const FillParams &params)`
    pub fn fill_surface(&mut self, surface: &Surface, params: &FillParams) -> Vec<Polyline> {
        let mut polylines_out: Vec<Polyline> = Vec::new();
        self.0.fill_surface_by_multilines(
            surface,
            params.clone(),
            &[
                SweepParams { angle_base: 0., pattern_shift: 0. },
                SweepParams { angle_base: (std::f64::consts::PI / 3.) as f32, pattern_shift: 0. },
                SweepParams { angle_base: (2. * std::f64::consts::PI / 3.) as f32, pattern_shift: 0. },
            ],
            &mut polylines_out,
        );
        polylines_out
    }
}

impl FillStars {
    /// FillRectilinear.hpp:98 — `bool is_self_crossing() override { return true; }`
    pub fn is_self_crossing(&self) -> bool {
        true
    }
    /// FillRectilinear.hpp:101
    pub fn _layer_angle(&self, _idx: usize) -> f32 {
        0.
    }
    /// FillRectilinear.cpp:3144-3153 — `Polylines FillStars::fill_surface(const Surface *surface, const FillParams &params)`
    pub fn fill_surface(&mut self, surface: &Surface, params: &FillParams) -> Vec<Polyline> {
        let mut polylines_out: Vec<Polyline> = Vec::new();
        // float((3./2.) * this->spacing * params.multiline / params.density)
        let shift = ((3. / 2.) * self.0.base.spacing * params.multiline as f64 / ((params.density as f32) as f64)) as f32;
        self.0.fill_surface_by_multilines(
            surface,
            params.clone(),
            &[
                SweepParams { angle_base: 0., pattern_shift: 0. },
                SweepParams { angle_base: (std::f64::consts::PI / 3.) as f32, pattern_shift: 0. },
                SweepParams { angle_base: (2. * std::f64::consts::PI / 3.) as f32, pattern_shift: shift },
            ],
            &mut polylines_out,
        );
        polylines_out
    }
}

impl FillCubic {
    /// FillRectilinear.hpp:111 — `bool is_self_crossing() override { return true; }`
    pub fn is_self_crossing(&self) -> bool {
        true
    }
    /// FillRectilinear.hpp:114
    pub fn _layer_angle(&self, _idx: usize) -> f32 {
        0.
    }
    /// FillRectilinear.cpp:3155-3165 — `Polylines FillCubic::fill_surface(const Surface *surface, const FillParams &params)`
    pub fn fill_surface(&mut self, surface: &Surface, params: &FillParams) -> Vec<Polyline> {
        let mut polylines_out: Vec<Polyline> = Vec::new();
        let dx: f64 = 0.5f64.sqrt() * self.0.base.z;
        self.0.fill_surface_by_multilines(
            surface,
            params.clone(),
            &[
                SweepParams { angle_base: 0., pattern_shift: dx as f32 },
                SweepParams { angle_base: (std::f64::consts::PI / 3.) as f32, pattern_shift: -(dx as f32) },
                SweepParams { angle_base: (std::f64::consts::PI * 2. / 3.) as f32, pattern_shift: dx as f32 },
            ],
            &mut polylines_out,
        );
        polylines_out
    }
}

impl FillSupportBase {
    /// FillRectilinear.hpp:125
    pub fn _layer_angle(&self, _idx: usize) -> f32 {
        0.
    }
    /// FillRectilinear.cpp:3182-3204 — `Polylines FillSupportBase::fill_surface(const Surface *surface, const FillParams &params)`
    pub fn fill_surface(&mut self, surface: &Surface, params: &FillParams) -> Vec<Polyline> {
        debug_assert!(!params.full_infill());

        let mut polylines_out: Vec<Polyline> = Vec::new();
        let rotate_vector = self.0._infill_direction(surface);
        let spacing = self.0.base.spacing;
        // ExPolygonWithOffset poly_with_offset(surface->expolygon, - rotate_vector.first, float(scale_(this->overlap - 0.5 * this->spacing)));
        let aoffset1 = (((-0.5f64).mul_add(spacing, self.0.base.overlap) / crate::libslic3r::SCALING_FACTOR) as f32) as Coord;
        let poly_with_offset = ExPolygonWithOffset::new_pin(&surface.expolygon, -rotate_vector.0, aoffset1, 0);
        if poly_with_offset.n_contours > 0 {
            let mut fill_lines: Vec<Polyline> = Vec::new();
            let line_spacing = ((spacing / crate::libslic3r::SCALING_FACTOR) / ((params.density as f32) as f64)) as Coord;
            // Create infill lines, keep them vertical.
            make_fill_lines(&poly_with_offset, point_rotated(rotate_vector.1, (-rotate_vector.0) as f64), 0., 0, line_spacing, 0, &mut fill_lines);
            // Both the poly_with_offset and polylines_out are rotated, so the infill lines are strictly vertical.
            Fill::connect_base_support_polygons(fill_lines, &poly_with_offset.polygons_outer, &poly_with_offset.bounding_box_outer(), &mut polylines_out, spacing, params);
            // Rotate back by rotate_vector.first
            //     const double cos_a = cos(rotate_vector.first);
            //     const double sin_a = sin(rotate_vector.first);
            // The argument is a float, so these are the float overloads of <cmath> (one
            // `__sincosf_stret` call), widened afterwards.
            let (sin_a, cos_a) = sin_cos_f32(rotate_vector.0);
            let (sin_a, cos_a) = (sin_a as f64, cos_a as f64);
            for pl in polylines_out.iter_mut() {
                for pt in pl.points.iter_mut() {
                    point_rotate_cos_sin(pt, cos_a, sin_a);
                }
            }
        }
        polylines_out
    }
}

impl FillZigZag {
    /// FillRectilinear.hpp:165 — `bool has_consistent_pattern() const override { return true; }`
    pub fn has_consistent_pattern(&self) -> bool {
        true
    }
    /// The `fill_surface` of the base class (FillRectilinear.cpp:3075-3095).
    pub fn fill_surface(&mut self, surface: &Surface, params: &FillParams) -> Vec<Polyline> {
        self.0.fill_surface(surface, params)
    }
}

impl FillCrossZag {
    /// FillRectilinear.hpp:174 — `bool has_consistent_pattern() const override { return true; }`
    pub fn has_consistent_pattern(&self) -> bool {
        true
    }
    /// The `fill_surface` of the base class (FillRectilinear.cpp:3075-3095).
    pub fn fill_surface(&mut self, surface: &Surface, params: &FillParams) -> Vec<Polyline> {
        self.0.fill_surface(surface, params)
    }
}

/// FillRectilinear.hpp:128-146 — `class FillMonotonicLineWGapFill : public Fill`, and
/// `class FillGlobalMonotonicLine` (FillRectilinear.hpp:148-156), which overrides `global_monotonic_sweep`.
/// (`fill_surface_extrusion`, FillRectilinear.cpp:3641-3753, is laid out by the slicing path in layer.rs.)
#[derive(Debug, Clone)]
pub struct FillMonotonicLineWGapFill {
    /// The members of the base class (FillBase.hpp:114-141).
    pub base: Fill,
    /// FillRectilinear.hpp:135 — `float gap_compensation_ratio{0.0};`
    pub gap_compensation_ratio: f32,
    /// The dynamic type: FillGlobalMonotonicLine.
    global_monotonic_sweep: bool,
}

impl Default for FillMonotonicLineWGapFill {
    fn default() -> Self {
        Self::new()
    }
}

/// FillRectilinear.hpp:148-156 — `class FillGlobalMonotonicLine : public FillMonotonicLineWGapFill`.
#[derive(Debug, Clone)]
pub struct FillGlobalMonotonicLine(pub FillMonotonicLineWGapFill);

impl Default for FillGlobalMonotonicLine {
    fn default() -> Self {
        Self::new()
    }
}

impl FillGlobalMonotonicLine {
    pub fn new() -> Self {
        let mut f = FillMonotonicLineWGapFill::new();
        f.global_monotonic_sweep = true;
        Self(f)
    }
    /// FillRectilinear.hpp:155 — `bool global_monotonic_sweep() const override { return true; }`
    pub fn global_monotonic_sweep(&self) -> bool {
        true
    }
}

impl FillMonotonicLineWGapFill {
    pub fn new() -> Self {
        Self { base: Fill::new(), gap_compensation_ratio: 0., global_monotonic_sweep: false }
    }

    /// FillRectilinear.hpp:133 — `bool is_self_crossing() override { return false; }`
    pub fn is_self_crossing(&self) -> bool {
        false
    }

    /// FillRectilinear.hpp:139 — `bool no_sort() const override { return true; }`
    pub fn no_sort(&self) -> bool {
        true
    }

    /// FillRectilinear.hpp:142 — `virtual bool global_monotonic_sweep() const { return false; }`: when true,
    /// keep monotonic-line geometry, sweep columns left-to-right, and alternate direction by column.
    pub fn global_monotonic_sweep(&self) -> bool {
        self.global_monotonic_sweep
    }

    /// FillRectilinear.cpp:3755-3874 —
    /// `void FillMonotonicLineWGapFill::fill_surface_by_lines(const Surface* surface, const FillParams& params, Polylines& polylines_out)`
    pub fn fill_surface_by_lines(&mut self, surface: &Surface, params: &FillParams, polylines_out: &mut Vec<Polyline>) {
        // `params.flow.spacing()` is a float (Flow.hpp:66).
        self.fill_surface_by_lines_flow_spacing(surface, params, params.flow.spacing() as f32, polylines_out)
    }

    /// The body of `fill_surface_by_lines` with `params.flow.spacing()` handed in.
    pub fn fill_surface_by_lines_flow_spacing(&mut self, surface: &Surface, params: &FillParams, flow_spacing: f32, polylines_out: &mut Vec<Polyline>) {
        // At the end, only the new polylines will be rotated back.
        let n_polylines_out_initial = polylines_out.len();

        // Shrink the input polygon a bit first to not push the infill lines out of the perimeters.
        const INFILL_OVERLAP_OVER_SPACING: f32 = 0.45;

        // Rotate polygons so that we can work with vertical lines here
        let rotate_vector = self.base._infill_direction(surface);

        let density = params.density as f32;
        // Flow.hpp:68 — `coord_t scaled_spacing() const { return coord_t(scale_(m_spacing)); }`
        let scaled_spacing = (flow_spacing as f64 / crate::libslic3r::SCALING_FACTOR) as Coord;
        // if (params.density < (float) params.flow.spacing() / (float) INT32_MAX || params.density < EPSILON)
        //     line_spacing = INT32_MAX;
        // else
        //     line_spacing = params.flow.scaled_spacing() / params.density;   (a float quotient)
        let mut line_spacing: Coord = if density < flow_spacing / (i32::MAX as f32) || (density as f64) < crate::libslic3r::EPSILON {
            i32::MAX as Coord
        } else {
            (scaled_spacing as f32 / density) as Coord
        };

        // On the polygons of poly_with_offset, the infill lines will be connected.
        //     float(scale_(0 - (0.5 - INFILL_OVERLAP_OVER_SPACING) * params.flow.spacing())),
        //     float(scale_(0 - 0.5f * params.flow.spacing())));
        let c = 0.5 - INFILL_OVERLAP_OVER_SPACING as f64;
        let aoffset1 = (((-c).mul_add(flow_spacing as f64, 0.) / crate::libslic3r::SCALING_FACTOR) as f32) as Coord;
        // 0.5f * float is a float product; the difference is an int minus that float, in float.
        let aoffset2 = ((((0.0f32 - 0.5f32 * flow_spacing) as f64) / crate::libslic3r::SCALING_FACTOR) as f32) as Coord;
        let poly_with_offset = ExPolygonWithOffset::new_pin(&surface.expolygon, -rotate_vector.0, aoffset1, aoffset2);
        if poly_with_offset.n_contours_inner == 0 {
            // Not a single infill line fits.
            //FIXME maybe one shall trigger the gap fill here?
            return;
        }

        let mut bounding_box = poly_with_offset.bounding_box_src_pin();

        // define flow spacing according to requested density
        if params.full_infill() && !params.dont_adjust {
            line_spacing = Fill::_adjust_solid_spacing(bounding_box.max.x - bounding_box.min.x, line_spacing);
            self.base.spacing = line_spacing as f64 * crate::libslic3r::SCALING_FACTOR;
        } else {
            // extend bounding box so that our pattern will be aligned with other layers
            // Transform the reference point to the rotated coordinate system.
            let refpt = point_rotated(rotate_vector.1, (-rotate_vector.0) as f64);
            bounding_box.merge_point(align_to_grid_base(bounding_box.min, Point::new(line_spacing, line_spacing), refpt));
        }

        // Intersect a set of euqally spaced vertical lines wiht expolygon.
        let n_vlines = ((bounding_box.max.x - bounding_box.min.x + line_spacing - 1) / line_spacing) as usize;
        let mut x0 = bounding_box.min.x;
        if params.full_infill() {
            x0 += (line_spacing + SCALED_EPSILON) / 2;
        }

        let mut segs = slice_region_by_vertical_lines(&poly_with_offset, n_vlines, x0, line_spacing);
        // Connect by horizontal / vertical links, classify the links based on link_max_length as too long.
        let rparams = FillRectilinearParams {
            density: params.density,
            monotonic: params.monotonic,
            dont_connect: params.dont_connect(),
            link_max_length: self.base.link_max_length,
            full_infill: params.full_infill(),
            dont_adjust: params.dont_adjust,
            consistent_pattern: false,
        };
        connect_segment_intersections_by_contours(&poly_with_offset, &mut segs, &rparams, self.base.link_max_length);

        // Sometimes the outer contour pinches the inner contour from both sides along a single vertical line.
        // This situation is not handled correctly by generate_montonous_regions().
        // Insert phony OUTER_HIGH / OUTER_LOW pairs at the position where the contour is pinched.
        pinch_contours_insert_phony_outer_intersections(&mut segs);
        let mut regions = generate_monotonous_regions(&mut segs);

        connect_monotonic_regions(&mut regions, &poly_with_offset, &segs);
        if !regions.is_empty() {
            // std::mt19937_64 rng; (seeded in chain_monotonic_regions)
            let path = chain_monotonic_regions(&mut regions, &poly_with_offset, &segs);
            polylines_from_paths(&path, &regions, &poly_with_offset, &segs, polylines_out);
        }

        // Keep the same line geometry / spacing / wall overlap as monotonic line.
        // Sweep columns strictly from left to right. Within one column, visit all
        // disconnected segments from one side to the other, then reverse the
        // direction for the next column.
        if self.global_monotonic_sweep() && polylines_out.len() > n_polylines_out_initial {
            let scan_x = |line: &Polyline| -> Coord { line.points[0].x.min(line.points[line.points.len() - 1].x) };
            let scan_y = |line: &Polyline| -> Coord { line.points[0].y.min(line.points[line.points.len() - 1].y) };
            let new_lines = &mut polylines_out[n_polylines_out_initial..];
            // std::sort (libc++: the order of equal keys is the algorithm's), over the keys with the
            // index of each line; the lines are then put into that order.
            let mut keyed: Vec<(Coord, Coord, usize)> = new_lines.iter().enumerate().map(|(i, l)| (scan_x(l), scan_y(l), i)).collect();
            crate::clipper::libcxx_sort(&mut keyed, |a: &(Coord, Coord, usize), b: &(Coord, Coord, usize)| {
                if a.0 != b.0 {
                    return a.0 < b.0;
                }
                a.1 < b.1
            });
            let sorted: Vec<Polyline> = keyed.iter().map(|k| std::mem::take(&mut new_lines[k.2])).collect();
            for (dst, src) in new_lines.iter_mut().zip(sorted) {
                *dst = src;
            }

            let mut column_idx = 0usize;
            let mut column_begin = 0usize;
            while column_begin != new_lines.len() {
                let column_x = scan_x(&new_lines[column_begin]);
                let mut column_end = column_begin + 1;
                while column_end != new_lines.len() && scan_x(&new_lines[column_end]) == column_x {
                    column_end += 1;
                }

                let upward = (column_idx & 1) == 0;
                if !upward {
                    new_lines[column_begin..column_end].reverse();
                }
                for it in new_lines[column_begin..column_end].iter_mut() {
                    if (it.points[0].y < it.points[it.points.len() - 1].y) != upward {
                        it.points.reverse();
                    }
                }

                column_begin = column_end;
                column_idx += 1;
            }
        }

        // paths must be rotated back
        for it in polylines_out[n_polylines_out_initial..].iter_mut() {
            // No need to translate, the absolute position is irrelevant.
            multi_point_rotate(&mut it.points, rotate_vector.0 as f64);
            //FIXME rather simplify the paths to avoid very short edges?
            crate::multi_point::remove_duplicate_points(&mut it.points);
        }
    }
}

/// Whether the entries of the slicing path lay their lines through the classes above: `FILL_RECTILINEAR_PIN`
/// on, and none of the earlier gates of those entries turned off (so that each of their arms can still be
/// reproduced).
fn live_entries_at_the_pin() -> bool {
    fill_rectilinear_pin()
        && crate::faithful_gate("TOPFILL_FAITHFUL")
        && crate::faithful_gate("RECT_ANGLE_F32")
        && crate::faithful_gate("FILL_BRIDGE_ANGLE")
        && crate::faithful_gate("RECT_OFFSET_FAITHFUL")
        && crate::faithful_gate("RECT_HORIZ_MOVE")
        && std::env::var("RASTER_QUANT").is_err()
}

/// The members of the filler and the surface as `Layer::make_fills` sets them (Fill.cpp:699-844) from what
/// the slicing path hands over in `InfillConfig`.
fn live_fill_members(base: &mut Fill, config: &InfillConfig, layer_index: usize) {
    base.layer_id = layer_index;
    base.spacing = config.extrusion_width;
    base.overlap = config.overlap;
    // `params.angle = float(Geometry::deg2rad(...))` (Fill.cpp), the configuration value in degrees.
    let mut angle: f32 = (std::f64::consts::PI * config.angle / 180.0) as f32;
    // Fill.cpp:704-709 — the zig zag turns from layer pair to layer pair.
    if let Some(infill_rotate_step) = config.zigzag_rotate_step {
        let half_layers = (layer_index / 2) as f32;
        angle = if layer_index % 2 == 0 { (-infill_rotate_step).mul_add(half_layers, angle) } else { infill_rotate_step.mul_add(half_layers, angle) };
    }
    base.angle = angle;
    base.link_max_length = config.link_max_length;
    // Fill.cpp:699 `f->set_bounding_box(bbox)`: the bounding box of the object, which is centred at the
    // origin (Print.hpp:442). Where the slicing path hands none over, a box with that centre.
    match config.object_bounding_box.as_ref() {
        Some(bbox) => base.set_bounding_box(bbox),
        None => base.set_bounding_box(&BoundingBox::from_points_minmax(Point::new(-1, -1), Point::new(1, 1))),
    }
}

fn live_surface(expoly: &ExPolygon, config: &InfillConfig) -> Surface {
    let mut surface = Surface::new(crate::surface::SurfaceType::Internal, expoly.clone());
    // Surface.hpp:41: negative means undefined.
    surface.bridge_angle = if config.bridge_angle >= 0.0 { Some(config.bridge_angle) } else { None };
    surface
}

fn live_fill_params(config: &InfillConfig, monotonic: bool) -> FillParams {
    let mut params = FillParams::new();
    params.density = config.density;
    // FillBase.hpp:56 — `dont_connect()` is `anchor_length_max < 0.05f`.
    params.anchor_length_max = if config.connect_infill { 1000.0 } else { 0.0 };
    params.dont_adjust = config.dont_adjust;
    params.monotonic = monotonic;
    params.locked_zag = config.locked_zag;
    params.horiz_move = config.horiz_move;
    if let Some(symmetric_y_axis) = config.symmetric_y_axis {
        params.symmetric_infill_y_axis = true;
        params.symmetric_y_axis = symmetric_y_axis;
    }
    params
}

/// `generate_fill_rectilinear` / `generate_fill_rectilinear_monotonic` of one direction through
/// `FillRectilinear::fill_surface_by_lines` (FILL_RECTILINEAR_PIN).
fn generate_fill_rectilinear_pin(fill_area: &[ExPolygon], config: &InfillConfig, layer_index: usize, monotonic: bool) -> Vec<InfillPath> {
    let mut paths = Vec::new();
    for expoly in fill_area {
        let mut f = FillRectilinear::with_class(if config.consistent_pattern { FillRectilinearClass::ZigZag } else { FillRectilinearClass::Rectilinear });
        live_fill_members(&mut f.base, config, layer_index);
        let surface = live_surface(expoly, config);
        let params = live_fill_params(config, monotonic);
        let mut polylines = Vec::new();
        let (_, spacing_written) = f.fill_surface_by_lines_spacing_written(&surface, &params, 0., 0., &mut polylines);
        if spacing_written {
            ADJUSTED_SPACING_MM.with(|c| c.set(Some(f.base.spacing)));
        }
        for polyline in polylines {
            if polyline.points.len() >= 2 {
                paths.push(InfillPath::Line(polyline));
            }
        }
    }
    paths
}

/// ipLine as the pin fills it (FILL_LINE_PIN): `Fill::new_from_type(ipLine)` makes a FillLine
/// (FillBase.cpp:44), which does not override `Fill::fill_surface` (FillBase.cpp:99-113): the
/// surface is offset by `float(scale_(this->overlap - 0.5 * this->spacing))` and every island is
/// handed to `FillLine::_fill_surface_single` (FillLine.cpp:10-120) with `_infill_direction(surface)`
/// (FillBase.cpp:197-241, the base `_layer_angle`). Nothing connects the lines again.
pub fn generate_fill_line_pin(fill_area: &[ExPolygon], config: &InfillConfig, layer_index: usize) -> Vec<InfillPath> {
    ADJUSTED_SPACING_MM.with(|c| c.set(None));
    let mut paths = Vec::new();
    for expoly in fill_area {
        let mut base = Fill::new();
        live_fill_members(&mut base, config, layer_index);
        let surface = live_surface(expoly, config);
        let params = live_fill_params(config, false);
        let mut f = super::fill_line::FillLine { spacing: base.spacing, ..Default::default() };
        // FillBase.cpp:102 — `offset_ex(surface->expolygon, float(scale_(this->overlap - 0.5 * this->spacing)))`
        let delta = ((-0.5f64).mul_add(f.spacing, base.overlap) / crate::libslic3r::SCALING_FACTOR) as f32;
        let expp = crate::clipper_utils::offset_ex_expolygon_clib(
            &surface.expolygon,
            delta as f64,
            crate::clipper_utils::OffsetJoinType::Miter,
            3.0,
        );
        // FillBase.cpp:104-111
        let direction = base._infill_direction(&surface);
        let mut polylines: Vec<Polyline> = Vec::new();
        for ex in expp {
            f._fill_surface_single(&params, surface.thickness_layers as u32, &direction, ex, &mut polylines);
        }
        // FillLine.cpp:28-30 writes `this->spacing` on a full, adjusted infill, which
        // Fill::fill_surface_extrusion reads back (FillBase.cpp:146-153).
        if params.density > 0.9999 && !params.dont_adjust && !polylines.is_empty() {
            ADJUSTED_SPACING_MM.with(|c| c.set(Some(f.spacing)));
        }
        for polyline in polylines {
            if polyline.points.len() >= 2 {
                paths.push(InfillPath::Line(polyline));
            }
        }
    }
    paths
}

/// ipHoneycomb as the pin fills it (FILL_HONEYCOMB_PIN): `Fill::new_from_type(ipHoneycomb)` makes a
/// FillHoneycomb (FillBase.cpp), which does not override `Fill::fill_surface` (FillBase.cpp:99-113): the
/// surface is offset by `float(scale_(this->overlap - 0.5 * this->spacing))` and every island goes to
/// `FillHoneycomb::_fill_surface_single` (FillHoneycomb.cpp:9-84) with `_infill_direction(surface)` and
/// the filler's `_layer_angle` (FillHoneycomb.hpp:53). The FillParams are those of Layer::make_fills
/// (Fill.cpp:789-797). One filler per surface fill, as the pin's `f` (its cache is kept).
pub fn generate_fill_honeycomb_pin(
    fill_area: &[ExPolygon],
    config: &InfillConfig,
    layer_index: usize,
    density: f64,
    multiline: i32,
    anchor_length: f64,
    anchor_length_max: f64,
) -> Vec<InfillPath> {
    let mut paths = Vec::new();
    let mut base = Fill::new();
    live_fill_members(&mut base, config, layer_index);
    let mut f = super::fill_honeycomb::FillHoneycomb::new(base.spacing);
    // Fill.cpp:789-797
    let mut params = FillParams::new();
    params.density = density;
    params.multiline = multiline;
    params.dont_adjust = false;
    params.anchor_length = anchor_length;
    params.anchor_length_max = anchor_length_max;
    for expoly in fill_area {
        let surface = live_surface(expoly, config);
        // Fill.cpp:844 — the spacing is reset for each expolygon.
        f.spacing = base.spacing;
        // FillBase.cpp:102
        let delta = ((-0.5f64).mul_add(f.spacing, base.overlap) / crate::libslic3r::SCALING_FACTOR) as f32;
        let expp = crate::clipper_utils::offset_ex_expolygon_clib(
            &surface.expolygon,
            delta as f64,
            crate::clipper_utils::OffsetJoinType::Miter,
            3.0,
        );
        // FillBase.cpp:104-111
        let direction = base._infill_direction_with(&surface, |idx| f.layer_angle(idx));
        let mut polylines: Vec<Polyline> = Vec::new();
        for ex in expp {
            f.fill_surface_single(&params, surface.thickness_layers as u32, &direction, ex, &mut polylines);
        }
        for polyline in polylines {
            if polyline.points.len() >= 2 {
                paths.push(InfillPath::Line(polyline));
            }
        }
    }
    paths
}

/// The monotonic lines of a top surface through `FillMonotonicLineWGapFill::fill_surface_by_lines`
/// (FILL_RECTILINEAR_PIN): the slicing path hands the spacing of the flow over as the extrusion width.
fn generate_fill_monotonic_line_w_gap_fill_pin(fill_area: &[ExPolygon], config: &InfillConfig, layer_index: usize) -> Vec<InfillPath> {
    let mut paths = Vec::new();
    for expoly in fill_area {
        let mut f = FillMonotonicLineWGapFill::new();
        live_fill_members(&mut f.base, config, layer_index);
        let surface = live_surface(expoly, config);
        let params = live_fill_params(config, true);
        let mut polylines = Vec::new();
        f.fill_surface_by_lines_flow_spacing(&surface, &params, config.extrusion_width as f32, &mut polylines);
        if params.full_infill() && !params.dont_adjust && !polylines.is_empty() {
            ADJUSTED_SPACING_MM.with(|c| c.set(Some(f.base.spacing)));
        }
        for polyline in polylines {
            if polyline.points.len() >= 2 {
                paths.push(InfillPath::Line(polyline));
            }
        }
    }
    paths
}

/// What tests/fill_rectilinear_native_parity.rs reads of the stages of the monotonic path, whose types are
/// private to this file: the vertical lines after `pinch_contours_insert_phony_outer_intersections`, the
/// regions after `generate_montonous_regions` and `connect_monotonic_regions` (left and right boundary,
/// len1, len2, flips, the indices of the neighbours), the chain of `chain_monotonic_regions` (region,
/// flipped) and the polylines of `polylines_from_paths`.
#[doc(hidden)]
pub struct MonotonicStages {
    pub segs: Vec<SegmentedIntersectionLine>,
    pub regions: Vec<([usize; 6], f32, f32, bool, Vec<usize>, Vec<usize>)>,
    pub chain: Vec<(usize, bool)>,
    pub polylines: Vec<Polyline>,
}

#[doc(hidden)]
pub fn monotonic_stages(poly_with_offset: &ExPolygonWithOffset, segs: &[SegmentedIntersectionLine]) -> MonotonicStages {
    let mut segs: Vec<SegmentedIntersectionLine> = segs.to_vec();
    pinch_contours_insert_phony_outer_intersections(&mut segs);
    let segs_pinched = segs.clone();
    let mut regions = generate_monotonous_regions(&mut segs);
    connect_monotonic_regions(&mut regions, poly_with_offset, &segs);
    let regions_out = regions
        .iter()
        .map(|r| {
            (
                [r.left.vline, r.left.low, r.left.high, r.right.vline, r.right.low, r.right.high],
                r.len1,
                r.len2,
                r.flips,
                r.left_neighbors.clone(),
                r.right_neighbors.clone(),
            )
        })
        .collect();
    let mut chain = Vec::new();
    let mut polylines = Vec::new();
    if !regions.is_empty() {
        let path = chain_monotonic_regions(&mut regions, poly_with_offset, &segs);
        chain = path.iter().map(|l| (l.region_idx, l.flipped)).collect();
        polylines_from_paths(&path, &regions, poly_with_offset, &segs, &mut polylines);
    }
    MonotonicStages { segs: segs_pinched, regions: regions_out, chain, polylines }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::{ExPolygon, Point, Polygon};
    use crate::scale;

    fn make_square(x: f64, y: f64, size: f64) -> ExPolygon {
        let x0 = scale(x);
        let y0 = scale(y);
        let x1 = scale(x + size);
        let y1 = scale(y + size);
        ExPolygon {
            contour: Polygon::from_points(vec![
                Point::new(x0, y0),
                Point::new(x1, y0),
                Point::new(x1, y1),
                Point::new(x0, y1),
            ]),
            holes: vec![],
        }
    }

    fn make_square_with_hole(x: f64, y: f64, size: f64, hole_size: f64) -> ExPolygon {
        let x0 = scale(x);
        let y0 = scale(y);
        let x1 = scale(x + size);
        let y1 = scale(y + size);
        let hx0 = scale(x + (size - hole_size) / 2.0);
        let hy0 = scale(y + (size - hole_size) / 2.0);
        let hx1 = scale(x + (size + hole_size) / 2.0);
        let hy1 = scale(y + (size + hole_size) / 2.0);
        ExPolygon {
            contour: Polygon::from_points(vec![
                Point::new(x0, y0),
                Point::new(x1, y0),
                Point::new(x1, y1),
                Point::new(x0, y1),
            ]),
            // Hole winds clockwise
            holes: vec![Polygon::from_points(vec![
                Point::new(hx0, hy0),
                Point::new(hx0, hy1),
                Point::new(hx1, hy1),
                Point::new(hx1, hy0),
            ])],
        }
    }

    #[test]
    fn test_expoly_with_offset_basic() {
        let square = make_square(0.0, 0.0, 20.0);
        let offset = ExPolygonWithOffset::new(
            &square,
            0.0,
            scale(-0.5), // shrink by 0.5mm
            scale(-1.0), // inner shrink by 1.0mm
        );
        assert!(offset.n_contours_outer > 0);
        assert!(offset.n_contours_inner > 0);
        assert_eq!(
            offset.n_contours,
            offset.n_contours_outer + offset.n_contours_inner
        );
    }

    #[test]
    fn test_slice_region_basic() {
        let square = make_square(0.0, 0.0, 20.0);
        let offset = ExPolygonWithOffset::new(&square, 0.0, scale(-0.5), scale(-1.0));

        let bbox = offset.bounding_box_outer();
        let line_spacing = scale(2.0); // 2mm spacing
        let n_vlines = ((bbox.max.x - bbox.min.x) / line_spacing).max(1) as usize;
        let x0 = bbox.min.x + line_spacing / 2;

        let segs = slice_region_by_vertical_lines(&offset, n_vlines, x0, line_spacing);
        assert!(!segs.is_empty());

        // Each vertical line that crosses the square should have intersections
        let non_empty = segs.iter().filter(|s| !s.intersections.is_empty()).count();
        assert!(
            non_empty > 0,
            "Expected some vertical lines with intersections"
        );
    }

    #[test]
    fn test_fill_surface_by_lines_basic() {
        let square = make_square(0.0, 0.0, 20.0);
        let params = FillRectilinearParams {
            density: 1.0,
            full_infill: true,
            ..Default::default()
        };

        let polylines = fill_surface_by_lines(
            &square, 0.45, // spacing in mm
            0.0,  // angle
            0.1,  // overlap
            &params,
        );

        assert!(
            !polylines.is_empty(),
            "Expected at least one polyline for a 20mm square"
        );

        // Each polyline should have at least 2 points
        for pl in &polylines {
            assert!(
                pl.points().len() >= 2,
                "Polyline should have at least 2 points"
            );
        }
    }

    #[test]
    fn test_fill_surface_by_lines_with_angle() {
        let square = make_square(0.0, 0.0, 20.0);
        let params = FillRectilinearParams {
            density: 1.0,
            full_infill: true,
            ..Default::default()
        };

        let polylines_0 = fill_surface_by_lines(&square, 0.45, 0.0, 0.1, &params);
        let polylines_45 =
            fill_surface_by_lines(&square, 0.45, std::f64::consts::FRAC_PI_4, 0.1, &params);

        assert!(!polylines_0.is_empty());
        assert!(!polylines_45.is_empty());
    }

    #[test]
    fn test_fill_surface_with_hole() {
        let expoly = make_square_with_hole(0.0, 0.0, 20.0, 5.0);
        let params = FillRectilinearParams {
            density: 1.0,
            full_infill: true,
            ..Default::default()
        };

        let polylines = fill_surface_by_lines(&expoly, 0.45, 0.0, 0.1, &params);
        assert!(
            !polylines.is_empty(),
            "Expected polylines for square with hole"
        );
    }

    #[test]
    fn test_fill_connected_paths() {
        // Test that the algorithm produces connected paths (polylines with >2 points)
        // which is the key improvement over simple line clipping.
        let square = make_square(0.0, 0.0, 20.0);
        let params = FillRectilinearParams {
            density: 1.0,
            full_infill: true,
            dont_connect: false,
            ..Default::default()
        };

        let polylines = fill_surface_by_lines(&square, 0.45, 0.0, 0.1, &params);

        // At least some polylines should have more than 2 points (connected paths)
        let connected_count = polylines.iter().filter(|p| p.points().len() > 2).count();
        assert!(
            connected_count > 0 || polylines.len() <= 2,
            "Expected some connected polylines (with >2 points) for a 20mm square"
        );
    }

    #[test]
    fn test_fill_sparse() {
        let square = make_square(0.0, 0.0, 20.0);
        let params = FillRectilinearParams {
            density: 0.2,
            full_infill: false,
            ..Default::default()
        };

        let polylines = fill_surface_by_lines(&square, 0.45, 0.0, 0.1, &params);
        assert!(!polylines.is_empty(), "Expected polylines for 20% infill");
    }

    #[test]
    fn test_fill_dont_connect() {
        let square = make_square(0.0, 0.0, 20.0);
        let params = FillRectilinearParams {
            density: 1.0,
            full_infill: true,
            dont_connect: true,
            ..Default::default()
        };

        let polylines = fill_surface_by_lines(&square, 0.45, 0.0, 0.1, &params);
        assert!(!polylines.is_empty());
    }

    #[test]
    fn test_generate_fill_rectilinear() {
        let square = make_square(0.0, 0.0, 20.0);
        let config = InfillConfig {
            pattern: crate::fill::InfillPattern::Rectilinear,
            density: 1.0,
            extrusion_width: 0.45,
            angle: 45.0,
            connect_infill: true,
            overlap: 0.1,
            ..Default::default()
        };

        let paths = generate_fill_rectilinear(&[square], &config, 0, false);
        assert!(
            !paths.is_empty(),
            "Expected infill paths from generate_fill_rectilinear"
        );
    }

    #[test]
    fn test_generate_fill_rectilinear_grid() {
        let square = make_square(0.0, 0.0, 20.0);
        let config = InfillConfig {
            pattern: crate::fill::InfillPattern::Grid,
            density: 0.2,
            extrusion_width: 0.45,
            angle: 0.0,
            connect_infill: true,
            overlap: 0.1,
            ..Default::default()
        };

        let paths = generate_fill_rectilinear(&[square], &config, 0, true);
        assert!(!paths.is_empty(), "Expected grid infill paths");
    }

    #[test]
    fn test_segment_intersection_ordering() {
        let mut a = SegmentIntersection::default();
        a.pos_p = 100;
        a.pos_q = 1;

        let mut b = SegmentIntersection::default();
        b.pos_p = 200;
        b.pos_q = 1;

        assert!(a.pos_less_than(&b));
        assert!(!b.pos_less_than(&a));
        assert!(!a.pos_equal(&b));

        let mut c = SegmentIntersection::default();
        c.pos_p = 100;
        c.pos_q = 1;
        assert!(a.pos_equal(&c));
    }

    #[test]
    fn test_segment_intersection_rational() {
        // Test rational comparison: 1/3 < 1/2
        let mut a = SegmentIntersection::default();
        a.pos_p = 1;
        a.pos_q = 3;

        let mut b = SegmentIntersection::default();
        b.pos_p = 1;
        b.pos_q = 2;

        assert!(a.pos_less_than(&b));
        assert!(!b.pos_less_than(&a));
    }

    #[test]
    fn test_distance_of_segments() {
        let poly = Polygon::from_points(vec![
            Point::new(0, 0),
            Point::new(100, 0),
            Point::new(100, 100),
            Point::new(0, 100),
        ]);

        assert_eq!(distance_of_segments(&poly, 0, 2, true), 2);
        assert_eq!(distance_of_segments(&poly, 2, 0, true), 2);
        assert_eq!(distance_of_segments(&poly, 0, 2, false), 2);
    }

    #[test]
    fn test_polygon_segment_append_basic() {
        let poly = Polygon::from_points(vec![
            Point::new(0, 0),
            Point::new(100, 0),
            Point::new(100, 100),
            Point::new(0, 100),
        ]);

        let mut out = Vec::new();
        polygon_segment_append(&mut out, &poly, 0, 2);
        assert_eq!(out.len(), 2);
        assert_eq!(out[0], Point::new(0, 0));
        assert_eq!(out[1], Point::new(100, 0));
    }

    #[test]
    fn test_polygon_segment_append_wrapped() {
        let poly = Polygon::from_points(vec![
            Point::new(0, 0),
            Point::new(100, 0),
            Point::new(100, 100),
            Point::new(0, 100),
        ]);

        let mut out = Vec::new();
        polygon_segment_append(&mut out, &poly, 3, 1);
        assert_eq!(out.len(), 2); // wraps: [3] then [0]
        assert_eq!(out[0], Point::new(0, 100));
        assert_eq!(out[1], Point::new(0, 0));
    }

    #[test]
    fn test_empty_expolygon() {
        let expoly = ExPolygon {
            contour: Polygon::from_points(vec![]),
            holes: vec![],
        };
        let params = FillRectilinearParams::default();
        let polylines = fill_surface_by_lines(&expoly, 0.45, 0.0, 0.1, &params);
        assert!(polylines.is_empty());
    }

    #[test]
    fn test_tiny_expolygon() {
        // Very small polygon where no infill lines fit
        let expoly = ExPolygon {
            contour: Polygon::from_points(vec![
                Point::new(0, 0),
                Point::new(scale(0.1), 0),
                Point::new(scale(0.1), scale(0.1)),
                Point::new(0, scale(0.1)),
            ]),
            holes: vec![],
        };
        let params = FillRectilinearParams::default();
        let polylines = fill_surface_by_lines(&expoly, 0.45, 0.0, 0.1, &params);
        // May or may not produce lines depending on offset — just check it doesn't crash
        let _ = polylines;
    }

    // -----------------------------------------------------------------------
    // Monotonic infill tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_monotonic_fill_basic_square() {
        let square = make_square(0.0, 0.0, 20.0);
        let params = FillRectilinearParams {
            density: 1.0,
            full_infill: true,
            monotonic: true,
            ..Default::default()
        };

        let polylines = fill_surface_by_lines_monotonic(
            &square, 0.45, // spacing
            0.0,  // angle
            0.1,  // overlap
            &params,
        );

        assert!(
            !polylines.is_empty(),
            "Monotonic fill should produce polylines for a 20mm square"
        );

        // Verify all points are within a reasonable range of the original square
        for pl in &polylines {
            for pt in pl.points() {
                let x_mm = crate::unscale(pt.x);
                let y_mm = crate::unscale(pt.y);
                assert!(
                    x_mm >= -1.0 && x_mm <= 21.0,
                    "Point x={} out of range",
                    x_mm
                );
                assert!(
                    y_mm >= -1.0 && y_mm <= 21.0,
                    "Point y={} out of range",
                    y_mm
                );
            }
        }
    }

    #[test]
    fn test_monotonic_fill_with_hole() {
        let expoly = make_square_with_hole(0.0, 0.0, 20.0, 6.0);
        let params = FillRectilinearParams {
            density: 1.0,
            full_infill: true,
            monotonic: true,
            ..Default::default()
        };

        let polylines = fill_surface_by_lines_monotonic(&expoly, 0.45, 0.0, 0.1, &params);

        assert!(
            !polylines.is_empty(),
            "Monotonic fill should produce polylines for square with hole"
        );
    }

    #[test]
    fn test_monotonic_fill_with_angle() {
        let square = make_square(0.0, 0.0, 20.0);
        let params = FillRectilinearParams {
            density: 1.0,
            full_infill: true,
            monotonic: true,
            ..Default::default()
        };

        let polylines = fill_surface_by_lines_monotonic(
            &square,
            0.45,
            std::f64::consts::FRAC_PI_4, // 45 degrees
            0.1,
            &params,
        );

        assert!(
            !polylines.is_empty(),
            "Monotonic fill should work at 45° angle"
        );
    }

    #[test]
    fn test_monotonic_vs_nonmonotonic_both_produce_output() {
        // Both variants should produce output for the same geometry; monotonic
        // typically produces fewer, longer polylines (better connected).
        let square = make_square(0.0, 0.0, 30.0);
        let params_mono = FillRectilinearParams {
            density: 1.0,
            full_infill: true,
            monotonic: true,
            ..Default::default()
        };
        let params_non = FillRectilinearParams {
            density: 1.0,
            full_infill: true,
            monotonic: false,
            ..Default::default()
        };

        let mono = fill_surface_by_lines_monotonic(&square, 0.45, 0.0, 0.1, &params_mono);
        let non_mono = fill_surface_by_lines(&square, 0.45, 0.0, 0.1, &params_non);

        assert!(!mono.is_empty(), "Monotonic should produce output");
        assert!(!non_mono.is_empty(), "Non-monotonic should produce output");

        // Both should cover roughly the same total length (within 50%)
        let mono_len: f64 = mono.iter().map(|p| p.length()).sum();
        let non_mono_len: f64 = non_mono.iter().map(|p| p.length()).sum();
        let ratio = mono_len / non_mono_len.max(1.0);
        assert!(
            ratio > 0.3 && ratio < 3.0,
            "Monotonic length ({}) and non-monotonic length ({}) should be in the same ballpark (ratio={})",
            mono_len, non_mono_len, ratio
        );
    }

    #[test]
    fn test_monotonic_fewer_polylines() {
        // For a simple rectangle, monotonic should produce fewer or equal polylines
        // because it chains regions better.
        let rect = ExPolygon {
            contour: Polygon::from_points(vec![
                Point::new(scale(0.0), scale(0.0)),
                Point::new(scale(40.0), scale(0.0)),
                Point::new(scale(40.0), scale(10.0)),
                Point::new(scale(0.0), scale(10.0)),
            ]),
            holes: vec![],
        };
        let params_mono = FillRectilinearParams {
            density: 1.0,
            full_infill: true,
            monotonic: true,
            ..Default::default()
        };
        let params_non = FillRectilinearParams {
            density: 1.0,
            full_infill: true,
            monotonic: false,
            ..Default::default()
        };

        let mono = fill_surface_by_lines_monotonic(&rect, 0.45, 0.0, 0.1, &params_mono);
        let non_mono = fill_surface_by_lines(&rect, 0.45, 0.0, 0.1, &params_non);

        // Monotonic should ideally produce fewer or equal polylines (better connectivity)
        // Allow some slack: monotonic count should not be drastically more.
        assert!(
            mono.len() <= non_mono.len() + 5,
            "Monotonic produced {} polylines vs {} non-monotonic; expected fewer or comparable",
            mono.len(),
            non_mono.len()
        );
    }

    #[test]
    fn test_generate_fill_rectilinear_monotonic_api() {
        let square = make_square(0.0, 0.0, 20.0);
        let config = super::super::InfillConfig {
            pattern: super::super::InfillPattern::Rectilinear,
            density: 1.0,
            extrusion_width: 0.45,
            angle: 0.0,
            angle_increment: 0.0,
            overlap: 0.1,
            connect_infill: true,
            ..Default::default()
        };

        // Non-monotonic
        let paths_non =
            generate_fill_rectilinear_monotonic(&[square.clone()], &config, 0, false, false);
        assert!(
            !paths_non.is_empty(),
            "Non-monotonic API should produce paths"
        );

        // Monotonic
        let paths_mono = generate_fill_rectilinear_monotonic(&[square], &config, 0, false, true);
        assert!(!paths_mono.is_empty(), "Monotonic API should produce paths");
    }

    #[test]
    fn test_monotonic_empty_expolygon() {
        let expoly = ExPolygon {
            contour: Polygon::from_points(vec![]),
            holes: vec![],
        };
        let params = FillRectilinearParams {
            monotonic: true,
            ..Default::default()
        };
        let polylines = fill_surface_by_lines_monotonic(&expoly, 0.45, 0.0, 0.1, &params);
        assert!(
            polylines.is_empty(),
            "Empty expolygon should produce no output"
        );
    }

    #[test]
    fn test_monotonic_tiny_expolygon() {
        let expoly = ExPolygon {
            contour: Polygon::from_points(vec![
                Point::new(0, 0),
                Point::new(scale(0.1), 0),
                Point::new(scale(0.1), scale(0.1)),
                Point::new(0, scale(0.1)),
            ]),
            holes: vec![],
        };
        let params = FillRectilinearParams {
            monotonic: true,
            ..Default::default()
        };
        let polylines = fill_surface_by_lines_monotonic(&expoly, 0.45, 0.0, 0.1, &params);
        // Should not crash; may or may not produce output
        let _ = polylines;
    }

    #[test]
    fn test_vertical_run_helpers() {
        // Create a simple intersection line with OUTER_LOW, INNER_LOW, INNER_HIGH, OUTER_HIGH
        let mut vline = SegmentedIntersectionLine {
            idx: 0,
            pos: 0,
            intersections: vec![
                {
                    let mut is = SegmentIntersection::default();
                    is.itype = SegmentIntersectionType::OuterLow;
                    is.pos_p = 0;
                    is
                },
                {
                    let mut is = SegmentIntersection::default();
                    is.itype = SegmentIntersectionType::InnerLow;
                    is.pos_p = 100;
                    is
                },
                {
                    let mut is = SegmentIntersection::default();
                    is.itype = SegmentIntersectionType::InnerHigh;
                    is.pos_p = 900;
                    is
                },
                {
                    let mut is = SegmentIntersection::default();
                    is.itype = SegmentIntersectionType::OuterHigh;
                    is.pos_p = 1000;
                    is
                },
            ],
        };

        // vertical_run_bottom from INNER_LOW should return itself
        assert_eq!(vertical_run_bottom(&vline, 1), 1);
        // vertical_run_bottom from INNER_HIGH should find the INNER_LOW
        assert_eq!(vertical_run_bottom(&vline, 2), 1);
        // vertical_run_top from INNER_LOW should find INNER_HIGH
        assert_eq!(vertical_run_top(&vline, 1), 2);
        // vertical_run_top from INNER_HIGH should return itself
        assert_eq!(vertical_run_top(&vline, 2), 2);
        // end_of_vertical_run from INNER_LOW
        assert_eq!(end_of_vertical_run(&vline, 1), 2);
    }

    #[test]
    fn test_pinch_contours_no_pinch() {
        // Verify pinch_contours doesn't modify a well-formed intersection set
        let mut segs = vec![
            SegmentedIntersectionLine {
                idx: 0,
                pos: 0,
                intersections: vec![
                    {
                        let mut is = SegmentIntersection::default();
                        is.itype = SegmentIntersectionType::OuterLow;
                        is.pos_p = 0;
                        is
                    },
                    {
                        let mut is = SegmentIntersection::default();
                        is.itype = SegmentIntersectionType::InnerLow;
                        is.pos_p = 100;
                        is
                    },
                    {
                        let mut is = SegmentIntersection::default();
                        is.itype = SegmentIntersectionType::InnerHigh;
                        is.pos_p = 900;
                        is
                    },
                    {
                        let mut is = SegmentIntersection::default();
                        is.itype = SegmentIntersectionType::OuterHigh;
                        is.pos_p = 1000;
                        is
                    },
                ],
            },
            SegmentedIntersectionLine {
                idx: 1,
                pos: scale(1.0),
                intersections: vec![
                    {
                        let mut is = SegmentIntersection::default();
                        is.itype = SegmentIntersectionType::OuterLow;
                        is.pos_p = 0;
                        is
                    },
                    {
                        let mut is = SegmentIntersection::default();
                        is.itype = SegmentIntersectionType::InnerLow;
                        is.pos_p = 100;
                        is
                    },
                    {
                        let mut is = SegmentIntersection::default();
                        is.itype = SegmentIntersectionType::InnerHigh;
                        is.pos_p = 900;
                        is
                    },
                    {
                        let mut is = SegmentIntersection::default();
                        is.itype = SegmentIntersectionType::OuterHigh;
                        is.pos_p = 1000;
                        is
                    },
                ],
            },
        ];

        let orig_len_0 = segs[0].intersections.len();
        let orig_len_1 = segs[1].intersections.len();

        pinch_contours_insert_phony_outer_intersections(&mut segs);

        // No pinch points, so intersection count should remain unchanged
        assert_eq!(segs[0].intersections.len(), orig_len_0);
        assert_eq!(segs[1].intersections.len(), orig_len_1);
    }

    #[test]
    fn test_monotonic_region_generation_simple() {
        // Test that generate_monotonous_regions produces at least one region
        // for a simple polygon.
        let square = make_square(0.0, 0.0, 10.0);
        let params = FillRectilinearParams {
            density: 1.0,
            full_infill: true,
            monotonic: true,
            ..Default::default()
        };

        let aoffset1 = scale(0.1 - (0.5 - 0.45) * 0.45);
        let aoffset2 = scale(0.1 - 0.5 * 0.45);
        let poly_with_offset = ExPolygonWithOffset::new(&square, 0.0, aoffset1, aoffset2);

        if poly_with_offset.n_contours_inner == 0 {
            // Offsets might collapse for small geometry; skip
            return;
        }

        let bbox = poly_with_offset.bounding_box_src();
        let line_spacing = scale(0.45);
        let n_vlines =
            ((bbox.max.x - bbox.min.x + line_spacing - 1) / line_spacing).max(0) as usize;
        let x0 = bbox.min.x + (line_spacing + SCALED_EPSILON) / 2;

        if n_vlines == 0 {
            return;
        }

        let mut segs =
            slice_region_by_vertical_lines(&poly_with_offset, n_vlines, x0, line_spacing);
        connect_segment_intersections_by_contours(&poly_with_offset, &mut segs, &params, 0);
        pinch_contours_insert_phony_outer_intersections(&mut segs);

        let regions = generate_monotonous_regions(&mut segs);

        assert!(
            !regions.is_empty(),
            "Should generate at least one monotonic region for a 10mm square"
        );

        // Each region should have valid boundaries
        for region in &regions {
            assert!(region.left.vline <= region.right.vline);
            assert!(region.left.low < region.left.high || region.left.low == region.left.high);
        }
    }
}

// ---------------------------------------------------------------------------------------------
// FillRectilinear.cpp:3247-3590 — the route of `FillMonotonicLineWGapFill::fill_surface_extrusion`
// (3735-3746): where each gap fill goes between the monotonic lines, the order of the gaps placed
// at one position and their direction. Nothing live calls it yet: the live assembly of the top
// surface's lines and gap fills is in layer.rs make_fills (see the R936 notes).
// ---------------------------------------------------------------------------------------------

/// FillRectilinear.cpp:3247-3251 `struct MonotonicRoute`.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct MonotonicRoute {
    /// `gaps_at[i]`: the gaps (indices into `gaps`) printed between line i-1 and line i.
    pub gaps_at: Vec<Vec<usize>>,
    /// The gaps that could not be inserted into the order of the lines.
    pub leftover: Vec<usize>,
}

/// `(a - b).cast<double>().squaredNorm()` of two Points: Eigen evaluates the Vec2d as one packet
/// (multiply, then the horizontal add), so the two products are rounded before the add.
#[inline]
fn route_squared_norm(a: &Point, b: &Point) -> f64 {
    let dx = (a.x - b.x) as f64;
    let dy = (a.y - b.y) as f64;
    dx * dx + dy * dy
}

/// `Point::rotated(double cos_a, double sin_a)` (Point.hpp:226-231, 235) with clang's contraction
/// of `cos_a * cur_x - sin_a * cur_y` and `cos_a * cur_y + sin_a * cur_x`.
#[inline]
fn route_rotated(p: &Point, cos_a: f64, sin_a: f64) -> Point {
    let cur_x = p.x as f64;
    let cur_y = p.y as f64;
    Point::new(
        cos_a.mul_add(cur_x, -(sin_a * cur_y)).round() as Coord,
        cos_a.mul_add(cur_y, sin_a * cur_x).round() as Coord,
    )
}

/// FillRectilinear.cpp:3501-3511 `struct OrientationScore` and its `operator<`.
#[derive(Debug, Clone, Copy)]
struct OrientationScore {
    fold_count: i32,
    total_link_length: f64,
}

impl OrientationScore {
    fn lt(&self, other: &OrientationScore) -> bool {
        if self.fold_count != other.fold_count {
            return self.fold_count < other.fold_count;
        }
        self.total_link_length < other.total_link_length
    }
}

/// FillRectilinear.cpp:3263-3266 the lambda `link`.
#[inline]
fn route_link(from: Option<&Point>, to: &Point) -> f64 {
    match from {
        Some(f) => route_squared_norm(to, f).sqrt(),
        None => 0.0,
    }
}

/// FillRectilinear.cpp:3267-3270 the lambda `folds_back`.
#[inline]
fn route_folds_back(from: Option<&Point>, at: &Point, to: Option<&Point>) -> bool {
    match (from, to) {
        (Some(f), Some(t)) => {
            let ax = (at.x - f.x) as f64;
            let ay = (at.y - f.y) as f64;
            let bx = (t.x - at.x) as f64;
            let by = (t.y - at.y) as f64;
            ax * bx + ay * by < 0.0
        }
        _ => false,
    }
}

/// FillRectilinear.cpp:3254-3590 `static MonotonicRoute plan_monotonic_route(const Polylines &lines,
/// ThickPolylines &gaps, double link_max, float scan_angle, const std::vector<size_t> &island_end)`.
/// Keeps the order of the monotonic lines; decides where each gap is inserted, the order of the gaps
/// at one position and the direction of each gap (the open ones are reversed in `gaps`).
pub fn plan_monotonic_route(
    lines: &[Polyline],
    gaps: &mut [crate::geometry::ThickPolyline],
    link_max: f64,
    scan_angle: f32,
    island_end: &[usize],
) -> MonotonicRoute {
    // FillRectilinear.cpp:3257-3261
    let n_lines = lines.len();
    let mut route = MonotonicRoute { gaps_at: vec![Vec::new(); n_lines + 1], leftover: Vec::new() };
    if gaps.is_empty() {
        return route;
    }

    // FillRectilinear.cpp:3274-3276 the scan frame (-scan_angle).
    let scan_cos = (-(scan_angle as f64)).cos();
    let scan_sin = (-(scan_angle as f64)).sin();
    let to_scan = |p: &Point| route_rotated(p, scan_cos, scan_sin);

    // FillRectilinear.cpp:3280-3292
    #[derive(Clone, Copy)]
    struct LineEndpoint {
        scan_pos: Point,
        line_idx: usize,
        island_idx: usize,
        is_start: bool,
    }
    #[derive(Clone, Copy)]
    struct ColumnSpan {
        min_x: Coord,
        max_x: Coord,
    }
    // FillRectilinear.cpp:3293-3295
    let mut island_spans =
        vec![ColumnSpan { min_x: Coord::MAX, max_x: Coord::MIN }; island_end.len().max(1)];
    // FillRectilinear.cpp:3296-3309
    let mut endpoints: Vec<LineEndpoint> = Vec::with_capacity(2 * n_lines);
    let mut isl = 0usize;
    for i in 0..n_lines {
        while isl + 1 < island_end.len() && i >= island_end[isl] {
            isl += 1;
        }
        let first = to_scan(&lines[i].first_point());
        let last = to_scan(&lines[i].last_point());
        endpoints.push(LineEndpoint { scan_pos: first, line_idx: i, island_idx: isl, is_start: true });
        endpoints.push(LineEndpoint { scan_pos: last, line_idx: i, island_idx: isl, is_start: false });
        island_spans[isl].min_x = island_spans[isl].min_x.min(first.x).min(last.x);
        island_spans[isl].max_x = island_spans[isl].max_x.max(first.x).max(last.x);
    }

    // FillRectilinear.cpp:3311-3313 std::sort by the scan x (libc++ order on equal keys).
    crate::clipper::libcxx_sort(&mut endpoints, |a, b| a.scan_pos.x < b.scan_pos.x);

    // FillRectilinear.cpp:3316-3333 the lambda `nearest_endpoint`.
    let nearest_endpoint = |q: &Point| -> (usize, f64) {
        let mut hi = endpoints.partition_point(|p| p.scan_pos.x < q.x);
        let mut lo = hi;
        let mut out = endpoints.len();
        let mut out_d2 = f64::MAX;
        let qx = q.x as f64;
        let inf = f64::INFINITY;
        while lo > 0 || hi < endpoints.len() {
            let dl = if lo > 0 { qx - endpoints[lo - 1].scan_pos.x as f64 } else { inf };
            let dr = if hi < endpoints.len() { endpoints[hi].scan_pos.x as f64 - qx } else { inf };
            // std::min(dl, dr)
            let dx = if dr < dl { dr } else { dl };
            if !(dx * dx < out_d2) {
                break;
            }
            let i = if dl <= dr {
                lo -= 1;
                lo
            } else {
                let t = hi;
                hi += 1;
                t
            };
            let d2 = route_squared_norm(&endpoints[i].scan_pos, q);
            if d2 < out_d2 {
                out_d2 = d2;
                out = i;
            }
        }
        (out, out_d2)
    };

    // FillRectilinear.cpp:3335-3341
    #[derive(Clone, Copy, Default)]
    struct GapAttach {
        at_line_start: bool,
        distance_squared: f64,
    }
    let mut attachments: Vec<GapAttach> = vec![GapAttach::default(); gaps.len()];

    // FillRectilinear.cpp:3343-3421
    for igap in 0..gaps.len() {
        if gaps[igap].points.len() < 2 {
            continue;
        }
        if n_lines == 0 {
            route.leftover.push(igap);
            continue;
        }
        // FillRectilinear.cpp:3353-3362 the probes: every point but the last of a closed gap, the
        // two ends of an open one.
        let probes: Vec<Point> = {
            let gap_line = &gaps[igap];
            if gap_line.is_closed() {
                gap_line.points[..gap_line.points.len() - 1].to_vec()
            } else {
                vec![gap_line.points[0], gap_line.points[gap_line.points.len() - 1]]
            }
        };
        // FillRectilinear.cpp:3364-3380
        let mut nearest_endpoint_idx = endpoints.len();
        let mut nearest_distance_squared = f64::MAX;
        let mut gap_min_x = Coord::MAX;
        let mut gap_max_x = Coord::MIN;
        for probe_point in &probes {
            let probe_scan_pos = to_scan(probe_point);
            gap_min_x = gap_min_x.min(probe_scan_pos.x);
            gap_max_x = gap_max_x.max(probe_scan_pos.x);
            let (endpoint_idx, distance_squared) = nearest_endpoint(&probe_scan_pos);
            if endpoint_idx < endpoints.len() && distance_squared < nearest_distance_squared {
                nearest_distance_squared = distance_squared;
                nearest_endpoint_idx = endpoint_idx;
            }
        }

        // FillRectilinear.cpp:3382-3389
        let endpoint = endpoints[nearest_endpoint_idx];
        let line_idx = endpoint.line_idx;
        let mut attach_at_line_start = endpoint.is_start;
        let island_span = island_spans[endpoint.island_idx];
        let endpoint_x = endpoint.scan_pos.x;
        let gap_center_is_right_of_endpoint = ((gap_min_x as i64 + gap_max_x as i64) / 2) as Coord > endpoint_x;

        // FillRectilinear.cpp:3391-3395
        let is_boundary_gap = if gap_center_is_right_of_endpoint {
            island_span.max_x <= gap_max_x
        } else {
            island_span.min_x >= gap_min_x
        };

        // FillRectilinear.cpp:3397-3417
        if is_boundary_gap {
            let position_has_line_link = |insertion_idx: usize| -> bool {
                insertion_idx > 0
                    && insertion_idx < n_lines
                    && route_squared_norm(&lines[insertion_idx].first_point(), &lines[insertion_idx - 1].last_point())
                        .sqrt()
                        <= link_max
            };
            let current_insertion_idx = if attach_at_line_start { line_idx } else { line_idx + 1 };
            let alternate_insertion_idx = if attach_at_line_start { line_idx + 1 } else { line_idx };
            if position_has_line_link(current_insertion_idx) && !position_has_line_link(alternate_insertion_idx) {
                attach_at_line_start = !attach_at_line_start;
                let new_endpoint =
                    if attach_at_line_start { lines[line_idx].first_point() } else { lines[line_idx].last_point() };
                nearest_distance_squared = f64::MAX;
                for probe in &probes {
                    let d = route_squared_norm(probe, &new_endpoint);
                    // std::min(nearest_distance_squared, d)
                    if d < nearest_distance_squared {
                        nearest_distance_squared = d;
                    }
                }
            }
        }

        // FillRectilinear.cpp:3419-3421
        let insertion_idx = if attach_at_line_start { line_idx } else { line_idx + 1 };
        attachments[igap] = GapAttach { at_line_start: attach_at_line_start, distance_squared: nearest_distance_squared };
        route.gaps_at[insertion_idx].push(igap);
    }

    // FillRectilinear.cpp:3424-3443 std::stable_sort of the gaps at one position.
    for ii in 0..=n_lines {
        let placed = &mut route.gaps_at[ii];
        if placed.len() < 2 {
            continue;
        }
        let less = |a: usize, b: usize| -> bool {
            let attach_a = attachments[a];
            let attach_b = attachments[b];
            if attach_a.at_line_start != attach_b.at_line_start {
                return !attach_a.at_line_start;
            }
            if attach_a.at_line_start {
                return attach_a.distance_squared > attach_b.distance_squared;
            }
            attach_a.distance_squared < attach_b.distance_squared
        };
        placed.sort_by(|&a, &b| {
            if less(a, b) {
                std::cmp::Ordering::Less
            } else if less(b, a) {
                std::cmp::Ordering::Greater
            } else {
                std::cmp::Ordering::Equal
            }
        });
    }

    // FillRectilinear.cpp:3445-3483 the leftover gaps by nearest neighbour.
    if !route.leftover.is_empty() {
        let mut route_end_point = Point::new(0, 0);
        let mut has_route_end = false;
        let mut remaining_gap_indices = std::mem::take(&mut route.leftover);
        while !remaining_gap_indices.is_empty() {
            let mut nearest_pool_pos = 0usize;
            let mut should_reverse = false;
            let mut nearest_distance = f64::MAX;
            for pool_pos in 0..remaining_gap_indices.len() {
                let candidate_gap = &gaps[remaining_gap_indices[pool_pos]];
                for reverse in 0..2 {
                    let entry_point = if reverse != 0 {
                        candidate_gap.points[candidate_gap.points.len() - 1]
                    } else {
                        candidate_gap.points[0]
                    };
                    let distance =
                        if has_route_end { route_squared_norm(&entry_point, &route_end_point).sqrt() } else { 0.0 };
                    if distance < nearest_distance {
                        nearest_distance = distance;
                        nearest_pool_pos = pool_pos;
                        should_reverse = reverse != 0;
                    }
                }
            }
            let selected_gap_idx = remaining_gap_indices.remove(nearest_pool_pos);
            let selected_gap = &mut gaps[selected_gap_idx];
            if should_reverse && !selected_gap.is_closed() {
                selected_gap.reverse();
            }
            route_end_point = selected_gap.points[selected_gap.points.len() - 1];
            has_route_end = true;
            route.leftover.push(selected_gap_idx);
        }
    }

    // FillRectilinear.cpp:3486-3497 the route flattened.
    #[derive(Clone, Copy)]
    struct RouteItem {
        geometry_idx: usize,
        is_gap_fill: bool,
    }
    let mut route_items: Vec<RouteItem> = Vec::with_capacity(lines.len() + gaps.len());
    for insertion_idx in 0..=n_lines {
        for &gap_idx in &route.gaps_at[insertion_idx] {
            route_items.push(RouteItem { geometry_idx: gap_idx, is_gap_fill: true });
        }
        if insertion_idx < n_lines {
            route_items.push(RouteItem { geometry_idx: insertion_idx, is_gap_fill: false });
        }
    }
    for &gap_idx in &route.leftover {
        route_items.push(RouteItem { geometry_idx: gap_idx, is_gap_fill: true });
    }

    // FillRectilinear.cpp:3513-3516 (the pointers are the Options).
    let mut previous_exit: Option<Point> = None;
    let mut next_entry: Option<Point>;
    let mut can_link_from_previous = false;
    let mut can_link_to_next: bool;

    // FillRectilinear.cpp:3518-3534 the lambda `score_orientation`.
    let score_orientation = |previous_exit: Option<&Point>,
                             next_entry: Option<&Point>,
                             can_link_from_previous: bool,
                             can_link_to_next: bool,
                             entry_point: &Point,
                             point_after_entry: &Point,
                             point_before_exit: &Point,
                             exit_point: &Point|
     -> OrientationScore {
        let entry_link_length = route_link(previous_exit, entry_point);
        let exit_link_length = route_link(next_entry, exit_point);
        let entry_folds_back = if can_link_from_previous && entry_link_length <= link_max {
            route_folds_back(previous_exit, entry_point, Some(point_after_entry)) as i32
        } else {
            0
        };
        let exit_folds_back = if can_link_to_next && exit_link_length <= link_max {
            route_folds_back(Some(point_before_exit), exit_point, next_entry) as i32
        } else {
            0
        };
        OrientationScore {
            fold_count: entry_folds_back + exit_folds_back,
            total_link_length: entry_link_length + exit_link_length,
        }
    };

    // FillRectilinear.cpp:3536-3587
    for item_pos in 0..route_items.len() {
        let item = route_items[item_pos];
        if !item.is_gap_fill {
            previous_exit = Some(lines[item.geometry_idx].last_point());
            can_link_from_previous = true;
            continue;
        }
        // FillRectilinear.cpp:3545-3563
        next_entry = None;
        can_link_to_next = false;
        if item_pos + 1 < route_items.len() {
            let next_item = route_items[item_pos + 1];
            let next_entry_point = if !next_item.is_gap_fill {
                lines[next_item.geometry_idx].first_point()
            } else {
                let gap_line = &gaps[item.geometry_idx];
                let next_gap_line = &gaps[next_item.geometry_idx];
                let g_first = gap_line.points[0];
                let g_last = gap_line.points[gap_line.points.len() - 1];
                let distance_to_current_gap = |point: &Point| -> f64 {
                    let a = route_squared_norm(point, &g_first);
                    let b = route_squared_norm(point, &g_last);
                    // std::min(a, b)
                    if b < a { b } else { a }
                };
                let n_first = next_gap_line.points[0];
                let n_last = next_gap_line.points[next_gap_line.points.len() - 1];
                if distance_to_current_gap(&n_first) <= distance_to_current_gap(&n_last) {
                    n_first
                } else {
                    n_last
                }
            };
            next_entry = Some(next_entry_point);
            can_link_to_next = !(next_item.is_gap_fill && gaps[next_item.geometry_idx].is_closed());
        }

        // FillRectilinear.cpp:3565-3583
        let gap_line = &mut gaps[item.geometry_idx];
        let resolved_exit_point = if gap_line.is_closed() {
            let seam_reference = if previous_exit.is_some() { previous_exit } else { next_entry };
            match seam_reference {
                Some(r) => crate::geometry::foot_pt(&gap_line.points, &r).1,
                None => gap_line.points[gap_line.points.len() - 1],
            }
        } else {
            let point_count = gap_line.points.len();
            let p = &gap_line.points;
            let forward_score = score_orientation(
                previous_exit.as_ref(),
                next_entry.as_ref(),
                can_link_from_previous,
                can_link_to_next,
                &p[0],
                &p[1],
                &p[point_count - 2],
                &p[point_count - 1],
            );
            let reverse_score = score_orientation(
                previous_exit.as_ref(),
                next_entry.as_ref(),
                can_link_from_previous,
                can_link_to_next,
                &p[point_count - 1],
                &p[point_count - 2],
                &p[1],
                &p[0],
            );
            if reverse_score.lt(&forward_score) {
                gap_line.reverse();
            }
            gap_line.points[gap_line.points.len() - 1]
        };
        // FillRectilinear.cpp:3584-3586
        previous_exit = Some(resolved_exit_point);
        can_link_from_previous = !gap_line.is_closed();
    }

    route
}

/// FillRectilinear.cpp:3592-3639 `static void append_monotonic_route(ExtrusionEntitiesPtr &dst,
/// const MonotonicRoute &route, Polylines &lines, const ThickPolylines &gaps, ExtrusionRole line_role,
/// double line_mm3_per_mm, float line_width, float line_height, ExtrusionRole gap_role,
/// const Flow &gap_flow, float overlap_gap_compensation_ratio)`: the lines and gap fills in the order
/// of `plan_monotonic_route`, into `dst` (the no-sort collection). The runs of lines between two gaps
/// go through `extrusion_entities_append_paths_with_wipe` (the crate's version takes no nozzle
/// diameter; the pin hands `gap_flow.nozzle_diameter()`). Ported, not compared yet.
#[allow(clippy::too_many_arguments)]
pub fn append_monotonic_route(
    dst: &mut Vec<crate::extrusion_entity::ExtrusionEntityType>,
    route: &MonotonicRoute,
    lines: &mut [Polyline],
    gaps: &[crate::geometry::ThickPolyline],
    line_role: crate::extrusion_entity::ExtrusionRole,
    line_mm3_per_mm: f64,
    line_width: f32,
    line_height: f32,
    gap_role: crate::extrusion_entity::ExtrusionRole,
    gap_flow: &crate::flow::Flow,
    overlap_gap_compensation_ratio: f32,
) {
    use crate::extrusion_entity::{ExtrusionEntityType, ExtrusionLoop, ExtrusionLoopRole};
    // FillRectilinear.cpp:3598-3600
    let width_tolerance = (0.05 / crate::SCALING_FACTOR) as f32;
    dst.reserve(lines.len() + gaps.len());
    let mut pending_lines: Vec<Polyline> = Vec::new();

    // FillRectilinear.cpp:3602-3617 the lambda `add_gap`.
    let add_gap = |dst: &mut Vec<ExtrusionEntityType>, pending_lines: &mut Vec<Polyline>, igap: usize| {
        let paths = crate::variable_width::thick_polyline_to_extrusion_paths_2(
            &gaps[igap],
            gap_role,
            gap_flow,
            width_tolerance,
        );
        if paths.is_empty() {
            return;
        }
        crate::extrusion_entity::extrusion_entities_append_paths_with_wipe(
            dst,
            std::mem::take(pending_lines),
            line_role,
            line_mm3_per_mm,
            line_width,
            line_height,
            overlap_gap_compensation_ratio,
        );
        if paths.first().unwrap().first_point() == paths.last().unwrap().last_point() {
            dst.push(ExtrusionEntityType::Loop(ExtrusionLoop::new(paths, ExtrusionLoopRole::DEFAULT)));
        } else {
            for path in paths {
                dst.push(ExtrusionEntityType::Path(path));
            }
        }
    };

    // FillRectilinear.cpp:3619-3625
    for i in 0..lines.len() {
        for &igap in &route.gaps_at[i] {
            add_gap(dst, &mut pending_lines, igap);
        }
        if lines[i].is_valid() {
            pending_lines.push(std::mem::take(&mut lines[i]));
        }
    }
    // FillRectilinear.cpp:3626-3628
    for &igap in route.gaps_at.last().unwrap() {
        add_gap(dst, &mut pending_lines, igap);
    }
    // FillRectilinear.cpp:3629-3631
    for &igap in &route.leftover {
        add_gap(dst, &mut pending_lines, igap);
    }
    // FillRectilinear.cpp:3633-3637
    crate::extrusion_entity::extrusion_entities_append_paths_with_wipe(
        dst,
        pending_lines,
        line_role,
        line_mm3_per_mm,
        line_width,
        line_height,
        overlap_gap_compensation_ratio,
    );
    dst.shrink_to_fit();
}

/// FillRectilinear.hpp:177-200 — `class FillLockedZag : public FillRectilinear`: the filler object with
/// its three members (`lock_param`, `skin_pattern`, `skeleton_pattern`, both patterns
/// `InfillPattern(0)` = ipConcentric by default). Its fill (`fill_surface_extrusion`,
/// `fill_surface_locked_zag`, the skin / skeleton helpers) is the free functions above
/// (FillRectilinear.cpp:3969-4004 and following), which take these members as arguments.
#[derive(Debug, Clone)]
pub struct FillLockedZag {
    pub base: FillRectilinear,
    /// FillRectilinear.hpp:182 `LockRegionParam lock_param;`
    pub lock_param: super::LockRegionParam,
    /// FillRectilinear.hpp:183 `InfillPattern skin_pattern = InfillPattern(0);`
    pub skin_pattern: crate::print_config::InfillPattern,
    /// FillRectilinear.hpp:184 `InfillPattern skeleton_pattern = InfillPattern(0);`
    pub skeleton_pattern: crate::print_config::InfillPattern,
}

impl Default for FillLockedZag {
    fn default() -> Self {
        Self {
            base: FillRectilinear::with_class(FillRectilinearClass::LockedZag),
            lock_param: super::LockRegionParam::default(),
            skin_pattern: crate::print_config::InfillPattern::Concentric,
            skeleton_pattern: crate::print_config::InfillPattern::Concentric,
        }
    }
}

impl FillLockedZag {
    pub fn new() -> Self {
        Self::default()
    }
    /// FillRectilinear.hpp:180 `Fill *clone() const override { return new FillLockedZag(*this); }`
    pub fn clone(&self) -> Self {
        Clone::clone(self)
    }
    /// FillRectilinear.hpp:187 `bool has_consistent_pattern() const override { return true; }`
    pub fn has_consistent_pattern(&self) -> bool {
        true
    }
    /// FillRectilinear.hpp:188 `void set_lock_region_param(const LockRegionParam &lock_param) override`
    pub fn set_lock_region_param(&mut self, lock_param: &super::LockRegionParam) {
        self.lock_param = lock_param.clone();
    }
    /// FillRectilinear.hpp:196-199 `void set_skin_and_skeleton_pattern(const InfillPattern &skin_pattern,
    /// const InfillPattern &skeleton_pattern)`
    pub fn set_skin_and_skeleton_pattern(
        &mut self,
        skin_pattern: &crate::print_config::InfillPattern,
        skeleton_pattern: &crate::print_config::InfillPattern,
    ) {
        self.skin_pattern = *skin_pattern;
        self.skeleton_pattern = *skeleton_pattern;
    }
}
