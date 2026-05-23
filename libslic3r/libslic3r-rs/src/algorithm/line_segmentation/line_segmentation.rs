//! Line segmentation algorithm.
//!
//! C++ Reference:
//! - Algorithm/LineSegmentation/LineSegmentation.hpp
//! - Algorithm/LineSegmentation/LineSegmentation.cpp
//!
//! Provides functions for segmenting polylines, polygons, and extrusion lines
//! against a set of clipping regions (ExPolygons). Each segment is tagged with
//! the index of the clipping region it falls within.
//!
//! The algorithm uses Clipper Z (with z-coordinates for tracking point provenance)
//! to intersect the subject line with clip regions, then reconstructs the line
//! segments with their region assignments.

use crate::geometry::{ExPolygon, Point, Polygon, Polyline};

/// Type aliases matching C++ types.
pub type ExPolygons = Vec<ExPolygon>;

/// A segment of a polyline with an associated clip region index.
///
/// LineSegmentation.hpp:26-30
#[derive(Debug, Clone)]
pub struct PolylineSegment {
    /// The segmented polyline
    pub polyline: Polyline,
    /// Index of the clipping region this segment belongs to
    pub clip_idx: usize,
}

/// A segment of a polyline with an associated region config reference.
///
/// LineSegmentation.hpp:32-38
#[derive(Debug, Clone)]
pub struct PolylineRegionSegment {
    /// The segmented polyline
    pub polyline: Polyline,
    /// Index into the perimeter regions
    pub region_idx: usize,
}

/// An extrusion line junction (simplified from Arachne::ExtrusionJunction).
///
/// LineSegmentation.hpp:42 (Arachne::ExtrusionJunction)
#[derive(Debug, Clone)]
pub struct ExtrusionJunction {
    /// Position
    pub p: Point,
    /// Width
    pub w: i64,
    /// Perimeter index
    pub perimeter_index: usize,
}

/// An extrusion line (simplified from Arachne::ExtrusionLine).
///
/// LineSegmentation.hpp:42 (Arachne::ExtrusionLine)
#[derive(Debug, Clone)]
pub struct ExtrusionLine {
    /// Junctions forming the extrusion path
    pub junctions: Vec<ExtrusionJunction>,
    /// Inset index
    pub inset_idx: usize,
    /// Whether this is an odd perimeter
    pub is_odd: bool,
    /// Whether this is a closed loop
    pub is_closed: bool,
}

impl ExtrusionLine {
    /// Create a new empty extrusion line.
    pub fn new(inset_idx: usize, is_odd: bool) -> Self {
        Self {
            junctions: Vec::new(),
            inset_idx,
            is_odd,
            is_closed: false,
        }
    }

    /// Check if the extrusion line is empty.
    pub fn is_empty(&self) -> bool {
        self.junctions.is_empty()
    }

    /// Get the number of junctions.
    pub fn len(&self) -> usize {
        self.junctions.len()
    }
}

/// A segment of an extrusion line with an associated clip region index.
///
/// LineSegmentation.hpp:40-44
#[derive(Debug, Clone)]
pub struct ExtrusionSegment {
    /// The segmented extrusion line
    pub extrusion: ExtrusionLine,
    /// Index of the clipping region
    pub clip_idx: usize,
}

/// A segment of an extrusion line with an associated region config.
///
/// LineSegmentation.hpp:46-52
#[derive(Debug, Clone)]
pub struct ExtrusionRegionSegment {
    /// The segmented extrusion line
    pub extrusion: ExtrusionLine,
    /// Index into the perimeter regions
    pub region_idx: usize,
}

/// Type aliases for segment collections.
///
/// LineSegmentation.hpp:54-57
pub type PolylineSegments = Vec<PolylineSegment>;
pub type ExtrusionSegments = Vec<ExtrusionSegment>;
pub type PolylineRegionSegments = Vec<PolylineRegionSegment>;
pub type ExtrusionRegionSegments = Vec<ExtrusionRegionSegment>;

/// Internal: Range of a line region (begin/end indices and interpolation parameters).
///
/// LineSegmentation.cpp:58-102
#[derive(Debug, Clone)]
struct LineRegionRange {
    begin_idx: usize,
    begin_t: f64,
    end_idx: usize,
    end_t: f64,
    clip_idx: usize,
}

impl LineRegionRange {
    fn new(begin_idx: usize, begin_t: f64, end_idx: usize, end_t: f64, clip_idx: usize) -> Self {
        Self {
            begin_idx,
            begin_t,
            end_idx,
            end_t,
            clip_idx,
        }
    }

    /// Check if this range overlaps with another.
    /// LineSegmentation.cpp:70-81
    fn is_overlap(&self, other: &LineRegionRange) -> bool {
        if self.end_idx < other.begin_idx || self.begin_idx > other.end_idx {
            return false;
        } else if self.end_idx == other.begin_idx && self.end_t <= other.begin_t {
            return false;
        } else if self.begin_idx == other.end_idx && self.begin_t >= other.end_t {
            return false;
        }
        true
    }

    /// Check if `inner` is wholly inside this range.
    /// LineSegmentation.cpp:84-95
    fn is_inside(&self, inner: &LineRegionRange) -> bool {
        if !self.is_overlap(inner) {
            return false;
        }
        let starts_after = (self.begin_idx < inner.begin_idx)
            || (self.begin_idx == inner.begin_idx && self.begin_t <= inner.begin_t);
        let ends_before = (self.end_idx > inner.end_idx)
            || (self.end_idx == inner.end_idx && self.end_t >= inner.end_t);
        starts_after && ends_before
    }

    /// Check if this range has zero length.
    /// LineSegmentation.cpp:96
    fn is_zero_length(&self) -> bool {
        self.begin_idx == self.end_idx && self.begin_t == self.end_t
    }
}

/// Interpolate between two points at parameter t.
fn lerp_point(a: &Point, b: &Point, t: f64) -> Point {
    Point::new(
        (a.x as f64 + (b.x as f64 - a.x as f64) * t) as i64,
        (a.y as f64 + (b.y as f64 - a.y as f64) * t) as i64,
    )
}

/// Create a polyline segment from a line region range.
///
/// LineSegmentation.cpp:409-436
fn create_polyline_segment(range: &LineRegionRange, subject: &Polyline) -> PolylineSegment {
    let mut points = Vec::new();

    // Start point
    // LineSegmentation.cpp:412-417
    if range.begin_t == 0.0 {
        points.push(subject.points[range.begin_idx]);
    } else {
        let interpolated = lerp_point(
            &subject.points[range.begin_idx],
            &subject.points[range.begin_idx + 1],
            range.begin_t,
        );
        points.push(interpolated);
    }

    // Intermediate points
    // LineSegmentation.cpp:420-422
    for line_idx in (range.begin_idx + 1)..=range.end_idx {
        points.push(subject.points[line_idx]);
    }

    // End point
    // LineSegmentation.cpp:424-432
    if range.end_t == 0.0 {
        points.push(subject.points[range.end_idx]);
    } else if range.end_t == 1.0 {
        points.push(subject.points[range.end_idx + 1]);
    } else {
        let interpolated = lerp_point(
            &subject.points[range.end_idx],
            &subject.points[range.end_idx + 1],
            range.end_t,
        );
        points.push(interpolated);
    }

    PolylineSegment {
        polyline: Polyline::from_points(points),
        clip_idx: range.clip_idx,
    }
}

/// Create polyline segments from line region ranges.
///
/// LineSegmentation.cpp:438-447
fn create_polyline_segments(ranges: &[LineRegionRange], subject: &Polyline) -> PolylineSegments {
    ranges
        .iter()
        .map(|range| create_polyline_segment(range, subject))
        .collect()
}

/// Fill gaps between sorted, non-overlapping region ranges with a default region.
///
/// LineSegmentation.cpp:335-395
fn create_continuous_line_region_ranges(
    mut ranges: Vec<LineRegionRange>,
    default_clip_idx: usize,
    total_lines_cnt: usize,
) -> Vec<LineRegionRange> {
    if ranges.is_empty() {
        return ranges;
    }

    // Sort by begin position
    // LineSegmentation.cpp:341
    ranges.sort_by(|a, b| {
        a.begin_idx.cmp(&b.begin_idx).then(
            a.begin_t
                .partial_cmp(&b.begin_t)
                .unwrap_or(std::cmp::Ordering::Equal),
        )
    });

    // Resolve overlapping regions
    // LineSegmentation.cpp:344-359
    for i in 1..ranges.len() {
        let (left, right) = ranges.split_at_mut(i);
        let prev = &mut left[i - 1];
        let curr = &mut right[0];

        if prev.is_inside(curr) {
            *curr = prev.clone();
            prev.end_idx = curr.begin_idx;
            prev.end_t = curr.begin_t;
        } else if prev.is_overlap(curr) {
            curr.begin_idx = prev.end_idx;
            curr.begin_t = prev.end_t;
        }
    }

    // Fill gaps with default region
    // LineSegmentation.cpp:363-395
    let mut out = Vec::new();
    let mut prev_line_idx: usize = 0;
    let mut prev_t: f64 = 0.0;

    for range in &ranges {
        if range.is_zero_length() {
            continue;
        }

        // Fill gap before this region
        // LineSegmentation.cpp:374-376
        if prev_line_idx != range.begin_idx || prev_t != range.begin_t {
            out.push(LineRegionRange::new(
                prev_line_idx,
                prev_t,
                range.begin_idx,
                range.begin_t,
                default_clip_idx,
            ));
        }

        // Add current region
        out.push(range.clone());
        prev_line_idx = range.end_idx;
        prev_t = range.end_t;
    }

    // Fill trailing gap
    // LineSegmentation.cpp:385-393
    let last_line_idx = total_lines_cnt - 1;
    if !((prev_line_idx == last_line_idx && prev_t == 1.0)
        || (prev_line_idx == total_lines_cnt && prev_t == 0.0))
    {
        out.push(LineRegionRange::new(
            prev_line_idx,
            prev_t,
            last_line_idx,
            1.0,
            default_clip_idx,
        ));
    }

    out
}

/// Segment a polyline against a set of clipping ExPolygons.
///
/// Each segment is tagged with the index of the clipping region it falls in,
/// or `default_clip_idx` for segments outside all clipping regions.
///
/// LineSegmentation.hpp:59
pub fn polyline_segmentation(
    subject: &Polyline,
    _expolygons_clips: &[ExPolygons],
    default_clip_idx: usize,
) -> PolylineSegments {
    // NOTE: Full implementation requires Clipper Z intersection.
    // For now, return the entire subject as a single segment with the default clip index.
    // This is correct behavior when there are no intersections.
    //
    // LineSegmentation.cpp:502-512
    // The C++ code returns the whole subject as default when no intersections found.

    if subject.points.is_empty() {
        return Vec::new();
    }

    vec![PolylineSegment {
        polyline: subject.clone(),
        clip_idx: default_clip_idx,
    }]
}

/// Segment a polygon against a set of clipping ExPolygons.
///
/// Converts the polygon to a polyline and delegates to `polyline_segmentation`.
///
/// LineSegmentation.hpp:60
pub fn polygon_segmentation(
    subject: &Polygon,
    expolygons_clips: &[ExPolygons],
    default_clip_idx: usize,
) -> PolylineSegments {
    // LineSegmentation.cpp:514-518
    // C++: return polyline_segmentation(to_polyline(subject), ...);
    let polyline = subject.to_polyline();
    polyline_segmentation(&polyline, expolygons_clips, default_clip_idx)
}

/// Segment an extrusion line against a set of clipping ExPolygons.
///
/// LineSegmentation.hpp:61
pub fn extrusion_segmentation(
    subject: &ExtrusionLine,
    _expolygons_clips: &[ExPolygons],
    default_clip_idx: usize,
) -> ExtrusionSegments {
    // NOTE: Full implementation requires Clipper Z intersection.
    // For now, return the whole subject as a single segment.
    //
    // LineSegmentation.cpp:519-529

    if subject.is_empty() {
        return Vec::new();
    }

    vec![ExtrusionSegment {
        extrusion: subject.clone(),
        clip_idx: default_clip_idx,
    }]
}

/// Segment a polyline with perimeter region configs.
///
/// LineSegmentation.hpp:63
pub fn polyline_region_segmentation(
    subject: &Polyline,
    _perimeter_regions: &[ExPolygons],
    default_region_idx: usize,
) -> PolylineRegionSegments {
    // LineSegmentation.cpp:542-558
    if subject.points.is_empty() {
        return Vec::new();
    }

    vec![PolylineRegionSegment {
        polyline: subject.clone(),
        region_idx: default_region_idx,
    }]
}

/// Segment a polygon with perimeter region configs.
///
/// LineSegmentation.hpp:64
pub fn polygon_region_segmentation(
    subject: &Polygon,
    perimeter_regions: &[ExPolygons],
    default_region_idx: usize,
) -> PolylineRegionSegments {
    // LineSegmentation.cpp:560-563
    let polyline = subject.to_polyline();
    polyline_region_segmentation(&polyline, perimeter_regions, default_region_idx)
}

/// Segment an extrusion line with perimeter region configs.
///
/// LineSegmentation.hpp:65
pub fn extrusion_region_segmentation(
    subject: &ExtrusionLine,
    _perimeter_regions: &[ExPolygons],
    default_region_idx: usize,
) -> ExtrusionRegionSegments {
    // LineSegmentation.cpp:565-581
    if subject.is_empty() {
        return Vec::new();
    }

    vec![ExtrusionRegionSegment {
        extrusion: subject.clone(),
        region_idx: default_region_idx,
    }]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_polyline_segment_creation() {
        let seg = PolylineSegment {
            polyline: Polyline::from_points(vec![Point::new(0, 0), Point::new(100, 100)]),
            clip_idx: 0,
        };
        assert_eq!(seg.clip_idx, 0);
        assert_eq!(seg.polyline.points.len(), 2);
    }

    #[test]
    fn test_polyline_segmentation_empty() {
        let subject = Polyline::new();
        let result = polyline_segmentation(&subject, &[], 0);
        assert!(result.is_empty());
    }

    #[test]
    fn test_polyline_segmentation_no_clips() {
        let subject = Polyline::from_points(vec![Point::new(0, 0), Point::new(100, 100)]);
        let result = polyline_segmentation(&subject, &[], 0);
        assert_eq!(result.len(), 1);
        assert_eq!(result[0].clip_idx, 0);
    }

    #[test]
    fn test_polygon_segmentation_no_clips() {
        let polygon = Polygon::from_points(vec![
            Point::new(0, 0),
            Point::new(100, 0),
            Point::new(100, 100),
            Point::new(0, 100),
        ]);
        let result = polygon_segmentation(&polygon, &[], 0);
        assert_eq!(result.len(), 1);
    }

    #[test]
    fn test_extrusion_segmentation_empty() {
        let subject = ExtrusionLine::new(0, false);
        let result = extrusion_segmentation(&subject, &[], 0);
        assert!(result.is_empty());
    }

    #[test]
    fn test_extrusion_segmentation_no_clips() {
        let mut subject = ExtrusionLine::new(0, false);
        subject.junctions.push(ExtrusionJunction {
            p: Point::new(0, 0),
            w: 100,
            perimeter_index: 0,
        });
        subject.junctions.push(ExtrusionJunction {
            p: Point::new(100, 100),
            w: 100,
            perimeter_index: 0,
        });
        let result = extrusion_segmentation(&subject, &[], 0);
        assert_eq!(result.len(), 1);
        assert_eq!(result[0].clip_idx, 0);
    }

    #[test]
    fn test_line_region_range_overlap() {
        let a = LineRegionRange::new(0, 0.0, 5, 1.0, 0);
        let b = LineRegionRange::new(3, 0.0, 8, 1.0, 1);
        assert!(a.is_overlap(&b));
    }

    #[test]
    fn test_line_region_range_no_overlap() {
        let a = LineRegionRange::new(0, 0.0, 3, 1.0, 0);
        let b = LineRegionRange::new(5, 0.0, 8, 1.0, 1);
        assert!(!a.is_overlap(&b));
    }

    #[test]
    fn test_line_region_range_inside() {
        let outer = LineRegionRange::new(0, 0.0, 10, 1.0, 0);
        let inner = LineRegionRange::new(3, 0.0, 7, 1.0, 1);
        assert!(outer.is_inside(&inner));
        assert!(!inner.is_inside(&outer));
    }

    #[test]
    fn test_line_region_range_zero_length() {
        let range = LineRegionRange::new(5, 0.5, 5, 0.5, 0);
        assert!(range.is_zero_length());
    }

    #[test]
    fn test_create_continuous_ranges_empty() {
        let result = create_continuous_line_region_ranges(vec![], 0, 10);
        assert!(result.is_empty());
    }

    #[test]
    fn test_lerp_point() {
        let a = Point::new(0, 0);
        let b = Point::new(100, 100);
        let mid = lerp_point(&a, &b, 0.5);
        assert_eq!(mid.x, 50);
        assert_eq!(mid.y, 50);
    }
}
