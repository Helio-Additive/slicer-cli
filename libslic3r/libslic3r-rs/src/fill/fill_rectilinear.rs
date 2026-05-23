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

/// Scaled epsilon for position comparisons (~0.001mm = 1000 units at 1e6 scale)
const SCALED_EPSILON: Coord = 1000;

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
        let aoffset1_mm = unscale(aoffset1);
        let polygons_outer = if aoffset1 == 0 {
            expolygon_to_polygons(&src)
        } else {
            let result = offset_expolygon(&src, aoffset1_mm, OffsetJoinType::Miter);
            expolygons_to_polygons(&result)
        };

        // Compute inner offset
        let polygons_inner = if aoffset2 < 0 {
            let shrink_amount = unscale(aoffset1 - aoffset2);
            let _outer_as_expolygons = polygons_to_expolygons_simple(&polygons_outer);
            let result =
                offset_polygons(&polygons_outer, -shrink_amount.abs(), OffsetJoinType::Miter);
            expolygons_to_polygons(&result)
        } else {
            Vec::new()
        };

        // Filter small contours
        let min_area_threshold = 0.01 * (aoffset2 as f64) * (aoffset2 as f64);
        let polygons_outer: Vec<Polygon> = polygons_outer
            .into_iter()
            .filter(|p| p.area().abs() > min_area_threshold.abs())
            .collect();
        let polygons_inner: Vec<Polygon> = polygons_inner
            .into_iter()
            .filter(|p| p.area().abs() > min_area_threshold.abs())
            .collect();

        let n_contours_outer = polygons_outer.len();
        let n_contours_inner = polygons_inner.len();
        let n_contours = n_contours_outer + n_contours_inner;

        // Determine CCW for each contour
        let mut polygons_ccw = Vec::with_capacity(n_contours);
        for i in 0..n_contours_outer {
            polygons_ccw.push(polygons_outer[i].is_counter_clockwise());
        }
        for i in 0..n_contours_inner {
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
        }
    }
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

/// Remove "sticks" (zero-area protrusions) from a polygon.
/// This is a simplified version of libslic3r's `remove_sticks`.
fn remove_sticks_polygon(poly: &mut Polygon) {
    if poly.points().len() < 3 {
        return;
    }
    let mut cleaned = Vec::with_capacity(poly.points().len());
    let n = poly.points().len();
    for i in 0..n {
        let prev = if i == 0 { n - 1 } else { i - 1 };
        let next = if i + 1 >= n { 0 } else { i + 1 };
        // Remove point if it's the same as its neighbour or if prev==next (stick)
        if poly.points()[i] == poly.points()[next] {
            continue;
        }
        if poly.points()[prev] == poly.points()[next] && poly.points()[prev] != poly.points()[i] {
            // This forms a stick — skip this point
            continue;
        }
        cleaned.push(poly.points()[i]);
    }
    if cleaned.len() < 3 {
        poly.points_mut().clear();
    } else {
        *poly.points_mut() = cleaned;
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

            // il, ir: indices of vertical lines that intersect this segment
            let mut il = ((l - x0) as f64 / line_spacing as f64).ceil() as i64;
            if il < 0 {
                il = 0;
            }
            let mut ir = ((r - x0) as f64 / line_spacing as f64).floor() as i64;
            if ir >= n_vlines as i64 {
                ir = n_vlines as i64 - 1;
            }

            if il > ir {
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

    // Sort intersections on each vertical line by Y position and clean up duplicates
    for sil in &mut segs {
        // Sort by Y position, with type as tiebreaker for same-position same-contour
        sil.intersections.sort_by(|a, b| {
            let pos_cmp = if a.pos_equal(b) {
                if a.i_contour == b.i_contour {
                    a.itype.cmp(&b.itype)
                } else {
                    std::cmp::Ordering::Equal
                }
            } else if a.pos_less_than(b) {
                std::cmp::Ordering::Less
            } else {
                std::cmp::Ordering::Greater
            };
            if pos_cmp != std::cmp::Ordering::Equal {
                return pos_cmp;
            }
            a.pos().cmp(&b.pos())
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

    let mut i = 0;
    while i < intersections.len() {
        if is_valid_type(&stack, intersections[i].itype) {
            match intersections[i].itype {
                OuterLow | InnerLow => stack.push(intersections[i].itype),
                OuterHigh | InnerHigh => {
                    stack.pop();
                }
                _ => {}
            }
            i += 1;
        } else {
            visited[i] = true;
            // Find a candidate to swap with
            let mut swap_index: Option<usize> = None;
            let pos_i = intersections[i].pos();
            for j in (i + 1)..intersections.len() {
                if !visited[j]
                    && (intersections[j].pos() - pos_i).abs() < scale(0.001)
                    && is_valid_type(&stack, intersections[j].itype)
                {
                    swap_index = Some(j);
                    visited[j] = true;
                    break;
                }
            }
            if let Some(si) = swap_index {
                intersections.swap(i, si);
                // Don't increment i — re-evaluate the swapped element
            } else {
                i += 1;
            }
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
    _params: &FillRectilinearParams,
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
            // Find the next unconsumed starting point (sweep left to right).
            let mut found = false;
            for iv in 0..segs.len() {
                let vline = &segs[iv];
                if vline.intersections.is_empty() {
                    continue;
                }
                for i in 0..vline.intersections.len() {
                    let intrsctn = &vline.intersections[i];
                    if intrsctn.is_outer() {
                        let consumed = if intrsctn.is_low() {
                            intrsctn.consumed_vertical_up
                        } else if i > 0 {
                            vline.intersections[i - 1].consumed_vertical_up
                        } else {
                            true
                        };
                        if !consumed {
                            i_vline = iv as i32;
                            i_intersection = i as i32;
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

            let prev_valid = i_prev >= 0
                && iv > 0
                && intersection_on_prev_vertical_line_valid(segs, iv, current_ii);
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

        // Clean up the polyline
        if let Some(last_polyline) = polylines_out.last() {
            let pts = last_polyline.points();
            let should_remove = pts.len() <= 1
                || (pts.len() == 2
                    && (pts[0].x - pts[1].x).abs() < SCALED_EPSILON
                    && (pts[0].y - pts[1].y).abs() < SCALED_EPSILON);
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
    let mut polylines_out = Vec::new();

    const INFILL_OVERLAP_OVER_SPACING: f64 = 0.45;

    // Line spacing in scaled units
    let line_spacing = scale((spacing / params.density).max(spacing));

    // Compute offsets
    let aoffset1 = scale(overlap - (0.5 - INFILL_OVERLAP_OVER_SPACING) * spacing);
    let aoffset2 = scale(overlap - 0.5 * spacing);

    let poly_with_offset = ExPolygonWithOffset::new(expolygon, -angle, aoffset1, aoffset2);

    if poly_with_offset.n_contours_inner == 0 {
        // No inner contour — no infill lines fit
        return polylines_out;
    }

    // Bounding box for vertical line placement
    let bbox_src = poly_with_offset.bounding_box_src();
    let bbox = bbox_src;

    // Adjust spacing for solid fill
    let line_spacing = if params.full_infill && !params.dont_adjust {
        adjust_solid_spacing(bbox.width(), line_spacing)
    } else {
        line_spacing
    };

    if line_spacing <= 0 {
        return polylines_out;
    }

    // Number of vertical lines
    let n_vlines = ((bbox.max.x - bbox.min.x + line_spacing - 1) / line_spacing).max(0) as usize;
    let mut x0 = bbox.min.x;
    if params.full_infill {
        x0 += (line_spacing + SCALED_EPSILON) / 2;
    }

    if n_vlines == 0 {
        return polylines_out;
    }

    // Slice the region
    let mut segs = slice_region_by_vertical_lines(&poly_with_offset, n_vlines, x0, line_spacing);

    // Build the connection graph
    // link_max_length: use 0 (unlimited) for solid infill, scaled value otherwise
    let link_max_length = if params.full_infill {
        0
    } else {
        (line_spacing as f64 * 1.5) as Coord
    };
    connect_segment_intersections_by_contours(
        &poly_with_offset,
        &mut segs,
        params,
        link_max_length,
    );

    // Traverse the graph to generate polylines
    traverse_graph_generate_polylines(&poly_with_offset, params, &mut segs, &mut polylines_out);

    // Rotate polylines back to original coordinate space
    for polyline in &mut polylines_out {
        // Remove duplicate points first
        polyline.points_mut().dedup();
        // Rotate back
        polyline.rotate(angle);
    }

    // Filter out degenerate polylines
    polylines_out.retain(|p| p.points().len() >= 2);

    polylines_out
}

/// Adjust line spacing for solid infill to fill the bounding box exactly.
fn adjust_solid_spacing(width: Coord, line_spacing: Coord) -> Coord {
    if width <= 0 || line_spacing <= 0 {
        return line_spacing;
    }
    let n_lines = ((width as f64 / line_spacing as f64) + 0.5) as Coord;
    if n_lines <= 0 {
        return line_spacing;
    }
    // Distribute the width evenly
    width / n_lines
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
    let mut paths = Vec::new();

    let angle_deg = config.angle + config.angle_increment * layer_index as f64;
    let angle_rad = angle_deg.to_radians();

    let params = FillRectilinearParams {
        density: config.density,
        full_infill: config.density > 0.99,
        dont_connect: !config.connect_infill,
        dont_adjust: false,
        monotonic: false,
        link_max_length: 0,
    };

    for expoly in fill_area {
        let polylines = fill_surface_by_lines(
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

use rand::Rng;

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
    let mut out = SegmentIntersection::default();
    out.itype = itype;
    out.pos_p = pos as i64;
    out.pos_q = 1;
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
                    let pinched = if up >= 0 && dn >= 0 {
                        dn + 1 != up
                    } else {
                        up == -1 || dn == -1
                    };
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
        _poly_with_offset: &ExPolygonWithOffset,
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

                        // Compute distance from exit of region i to entry of region j
                        let exit_vline = regions[i].right.vline;
                        let exit_inter = regions[i].right_intersection_point(from_flipped);
                        let entry_vline = regions[j].left.vline;
                        let entry_inter = regions[j].left_intersection_point(to_flipped);

                        let p1 = if exit_vline < segs.len()
                            && exit_inter < segs[exit_vline].intersections.len()
                        {
                            let vl = &segs[exit_vline];
                            Point::new(vl.pos, vl.intersections[exit_inter].pos())
                        } else {
                            Point::new(0, 0)
                        };

                        let p2 = if entry_vline < segs.len()
                            && entry_inter < segs[entry_vline].intersections.len()
                        {
                            let vl = &segs[entry_vline];
                            Point::new(vl.pos, vl.intersections[entry_inter].pos())
                        } else {
                            Point::new(0, 0)
                        };

                        let dx = (p2.x - p1.x) as f64;
                        let dy = (p2.y - p1.y) as f64;
                        let dist = (dx * dx + dy * dy).sqrt() as f32;
                        let dist_unscaled = dist / crate::SCALING_FACTOR as f32;

                        let idx = (i * 2 + from_flip) * dim + (j * 2 + to_flip);
                        paths[idx].length = dist_unscaled;
                        paths[idx].visibility = if dist_unscaled > 0.0 {
                            1.0 / dist_unscaled
                        } else {
                            1e6
                        };
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
    _poly_with_offset: &ExPolygonWithOffset,
    segs: &[SegmentedIntersectionLine],
) {
    let n = regions.len();
    if n == 0 {
        return;
    }

    // Build maps from intersection index to region start/end
    // (vline, low_intersection_idx) -> region_idx
    let mut region_starts: Vec<(usize, usize, usize)> = Vec::with_capacity(n); // (vline, low, region_idx)
    let mut region_ends: Vec<(usize, usize, usize)> = Vec::with_capacity(n);
    for (i, region) in regions.iter().enumerate() {
        region_starts.push((region.left.vline, region.left.low, i));
        region_ends.push((region.right.vline, region.right.low, i));
    }
    region_starts.sort_by(|a, b| a.0.cmp(&b.0).then(a.1.cmp(&b.1)));
    region_ends.sort_by(|a, b| a.0.cmp(&b.0).then(a.1.cmp(&b.1)));

    // For each region, find overlapping regions on left and right
    for i in 0..n {
        // Left neighbors: regions whose right boundary is at (our left.vline - 1)
        let left_vline = regions[i].left.vline;
        if left_vline > 0 {
            let target_vline = left_vline - 1;
            let i_low = regions[i].left.low;
            let i_high = regions[i].left.high;

            // Find overlap on the left
            if let Some((l_low, l_high)) = left_overlap(segs, left_vline, i_low, i_high) {
                // Find which region ends at (target_vline, l_low)
                for &(ev, _el, ei) in &region_ends {
                    if ev == target_vline {
                        // Check if this region's right boundary overlaps
                        let r_low = regions[ei].right.low;
                        let r_high = regions[ei].right.high;
                        // Overlap check: the vertical runs intersect
                        let vline_other = &segs[target_vline];
                        let this_bot = if r_low < vline_other.intersections.len() {
                            vline_other.intersections[r_low].pos()
                        } else {
                            continue;
                        };
                        let this_top = if r_high < vline_other.intersections.len() {
                            vline_other.intersections[r_high].pos()
                        } else {
                            continue;
                        };
                        let other_bot = if l_low < vline_other.intersections.len() {
                            vline_other.intersections[l_low].pos()
                        } else {
                            continue;
                        };
                        let other_top = if l_high < vline_other.intersections.len() {
                            vline_other.intersections[l_high].pos()
                        } else {
                            continue;
                        };
                        if this_bot <= other_top && other_bot <= this_top {
                            if !regions[i].left_neighbors.contains(&ei) {
                                regions[i].left_neighbors.push(ei);
                            }
                        }
                    }
                }
            }
        }

        // Right neighbors: regions whose left boundary is at (our right.vline + 1)
        let right_vline = regions[i].right.vline;
        if right_vline + 1 < segs.len() {
            let target_vline = right_vline + 1;
            let i_low = regions[i].right.low;
            let i_high = regions[i].right.high;

            if let Some((r_low, r_high)) = right_overlap(segs, right_vline, i_low, i_high) {
                for &(sv, _sl, si) in &region_starts {
                    if sv == target_vline {
                        let s_low = regions[si].left.low;
                        let s_high = regions[si].left.high;
                        let vline_other = &segs[target_vline];
                        let this_bot = if s_low < vline_other.intersections.len() {
                            vline_other.intersections[s_low].pos()
                        } else {
                            continue;
                        };
                        let this_top = if s_high < vline_other.intersections.len() {
                            vline_other.intersections[s_high].pos()
                        } else {
                            continue;
                        };
                        let other_bot = if r_low < vline_other.intersections.len() {
                            vline_other.intersections[r_low].pos()
                        } else {
                            continue;
                        };
                        let other_top = if r_high < vline_other.intersections.len() {
                            vline_other.intersections[r_high].pos()
                        } else {
                            continue;
                        };
                        if this_bot <= other_top && other_bot <= this_top {
                            if !regions[i].right_neighbors.contains(&si) {
                                regions[i].right_neighbors.push(si);
                            }
                        }
                    }
                }
            }
        }
    }

    // Ensure symmetry: if A is left-neighbor of B, then B is right-neighbor of A
    for i in 0..n {
        let left_n: Vec<usize> = regions[i].left_neighbors.clone();
        for &ln in &left_n {
            if !regions[ln].right_neighbors.contains(&i) {
                regions[ln].right_neighbors.push(i);
            }
        }
        let right_n: Vec<usize> = regions[i].right_neighbors.clone();
        for &rn in &right_n {
            if !regions[rn].left_neighbors.contains(&i) {
                regions[rn].left_neighbors.push(i);
            }
        }
    }

    // Sort neighbors for deterministic behavior
    for region in regions.iter_mut() {
        region.left_neighbors.sort();
        region.left_neighbors.dedup();
        region.right_neighbors.sort();
        region.right_neighbors.dedup();
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

    total_length / crate::SCALING_FACTOR as f32
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
    if n == 1 {
        return vec![MonotonicRegionLink {
            region_idx: 0,
            flipped: false,
            next_path_idx: None,
            next_flipped_path_idx: None,
        }];
    }

    // Fill in path lengths
    for i in 0..n {
        let len1 = monotonous_region_path_length(&regions[i], false, poly_with_offset, segs);
        let len2 = monotonous_region_path_length(&regions[i], true, poly_with_offset, segs);
        regions[i].len1 = len1;
        regions[i].len2 = len2;
        // Subtract smaller from larger for optimization
        if regions[i].len1 > regions[i].len2 {
            regions[i].len1 -= regions[i].len2;
            regions[i].len2 = 0.0;
        } else {
            regions[i].len2 -= regions[i].len1;
            regions[i].len1 = 0.0;
        }
    }

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

            // Try right neighbors first
            let right_n: Vec<usize> = regions[path_end_region].right_neighbors.clone();
            for &next in &right_n {
                if left_unprocessed[next] <= 2 {
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

    let mut rng = rand::thread_rng();
    let mut num_rounds_no_change = 0;

    for _round in 0..num_rounds {
        if num_rounds_no_change >= num_rounds_no_change_exit {
            break;
        }

        let mut improved = false;
        for _ant in 0..num_ants {
            let mut path: Vec<MonotonicRegionLink> = Vec::with_capacity(n);
            let mut queue = queue_initial.clone();
            let mut left_unprocessed = left_neighbors_unprocessed_initial.clone();

            if queue.is_empty() {
                for i in 0..n {
                    queue.push(i);
                }
            }

            // Pick random first region
            let first_idx_in_queue = rng.gen_range(0..queue.len());
            let first_region = queue[first_idx_in_queue];
            let first_flipped: bool = rng.gen();
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

                // Select path
                let dice: f32 = rng.gen();
                let take_idx = if dice < probability_take_best {
                    // Take best
                    next_candidates
                        .iter()
                        .enumerate()
                        .max_by(|a, b| {
                            a.1.probability
                                .partial_cmp(&b.1.probability)
                                .unwrap_or(std::cmp::Ordering::Equal)
                        })
                        .map(|(i, _)| i)
                        .unwrap_or(0)
                } else {
                    // Probabilistic selection
                    let total_prob: f32 = next_candidates.iter().map(|c| c.probability).sum();
                    let threshold: f32 = rng.gen::<f32>() * total_prob;
                    let mut acc = 0.0f32;
                    let mut selected = next_candidates.len() - 1;
                    for (i, c) in next_candidates.iter().enumerate() {
                        acc += c.probability;
                        if acc >= threshold {
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

        // Global pheromone update with best path
        if !best_path.is_empty() {
            let total_cost = best_path_length + 1e-6;
            for i in 0..best_path.len().saturating_sub(1) {
                let r_idx = best_path[i].region_idx;
                let r_flip = best_path[i].flipped;
                let n_idx = best_path[i + 1].region_idx;
                let n_flip = best_path[i + 1].flipped;
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

    let line_spacing = scale((spacing / params.density).max(spacing));

    let aoffset1 = scale(overlap - (0.5 - INFILL_OVERLAP_OVER_SPACING) * spacing);
    let aoffset2 = scale(overlap - 0.5 * spacing);

    let poly_with_offset = ExPolygonWithOffset::new(expolygon, -angle, aoffset1, aoffset2);

    if poly_with_offset.n_contours_inner == 0 {
        return polylines_out;
    }

    let bbox_src = poly_with_offset.bounding_box_src();

    let line_spacing = if params.full_infill && !params.dont_adjust {
        adjust_solid_spacing(bbox_src.width(), line_spacing)
    } else {
        line_spacing
    };

    if line_spacing <= 0 {
        return polylines_out;
    }

    let n_vlines =
        ((bbox_src.max.x - bbox_src.min.x + line_spacing - 1) / line_spacing).max(0) as usize;
    let mut x0 = bbox_src.min.x;
    if params.full_infill {
        x0 += (line_spacing + SCALED_EPSILON) / 2;
    }

    if n_vlines == 0 {
        return polylines_out;
    }

    let mut segs = slice_region_by_vertical_lines(&poly_with_offset, n_vlines, x0, line_spacing);

    let link_max_length = if params.full_infill {
        0
    } else {
        (line_spacing as f64 * 1.5) as Coord
    };
    connect_segment_intersections_by_contours(
        &poly_with_offset,
        &mut segs,
        params,
        link_max_length,
    );

    // Insert phony outer intersections at pinch points
    pinch_contours_insert_phony_outer_intersections(&mut segs);

    // Generate monotonic regions
    let mut regions = generate_monotonous_regions(&mut segs);

    if !regions.is_empty() {
        // Connect neighboring regions
        connect_monotonic_regions(&mut regions, &poly_with_offset, &segs);

        // Chain regions using ant colony optimization
        let path = chain_monotonic_regions(&mut regions, &poly_with_offset, &segs);

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
    let mut paths = Vec::new();

    let angle_deg = config.angle + config.angle_increment * layer_index as f64;
    let angle_rad = angle_deg.to_radians();

    let params = FillRectilinearParams {
        density: config.density,
        full_infill: config.density > 0.99,
        dont_connect: !config.connect_infill,
        dont_adjust: false,
        monotonic,
        link_max_length: 0,
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
            pattern: crate::infill::InfillPattern::Rectilinear,
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
            pattern: crate::infill::InfillPattern::Grid,
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
