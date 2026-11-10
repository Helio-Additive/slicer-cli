//! Internal bridge detection for optimal bridge angle calculation.
//!
//! 1:1 line-by-line port of:
//! - InternalBridgeDetector.hpp
//! - InternalBridgeDetector.cpp
//!
//! BBS: InternalBridgeDetector is used to detect bridge angle for internal bridge.
//! this step may enlarge internal bridge area for a little(only occupy sparse infill
//! area) for better anchoring.

// InternalBridgeDetector.cpp:1-4
use crate::clipper_utils::{difference, offset_expolygons, OffsetJoinType};
use crate::geometry::{
    directions_parallel, expolygons_contain, to_lines, BoundingBox, ExPolygon, ExPolygons, Line,
    Lines, Point,
};
use crate::{clipper_utils, Coord};
use std::f64::consts::PI;

// InternalBridgeDetector.hpp:29-45
struct InternalBridgeDirection {
    // InternalBridgeDetector.hpp:42
    angle: f64,
    // InternalBridgeDetector.hpp:43
    coverage: f64,
    // InternalBridgeDetector.hpp:44
    max_length: f64,
}

impl InternalBridgeDirection {
    // InternalBridgeDetector.hpp:30
    fn new(a: f64) -> Self {
        Self {
            angle: a,
            coverage: 0.,
            max_length: 0.,
        }
    }

    // the best direction is the one causing most lines to be bridged and the span is short
    // InternalBridgeDetector.hpp:32-41 `bool operator<(const InternalBridgeDirection &other) const`
    fn less(&self, other: &InternalBridgeDirection) -> bool {
        let delta = self.coverage - other.coverage;
        if delta > 0.001 {
            true
        } else if delta < -0.001 {
            false
        } else {
            // coverage is almost same, then compare span
            self.max_length < other.max_length
        }
    }
}

// InternalBridgeDetector.hpp:11-50
pub struct InternalBridgeDetector {
    // input: all fill area in LayerRegion without overlap with perimeter.
    // InternalBridgeDetector.hpp:15
    pub fill_no_overlap: ExPolygons,
    // input: internal bridge infill area.
    // InternalBridgeDetector.hpp:17
    pub internal_bridge_infill: ExPolygons,
    // input: scaled extrusion width of the infill.
    // InternalBridgeDetector.hpp:19
    pub spacing: Coord,
    // output: the final optimal angle.
    // InternalBridgeDetector.hpp:21
    pub angle: f64,

    // InternalBridgeDetector.hpp:48
    resolution: f64,
    // InternalBridgeDetector.hpp:49
    m_anchor_regions: ExPolygons,
}

impl InternalBridgeDetector {
    // InternalBridgeDetector.cpp:8-15
    pub fn new(internal_bridge: ExPolygon, fill_no_overlap: &ExPolygons, spacing: Coord) -> Self {
        let mut detector = Self {
            // InternalBridgeDetector.cpp:10
            fill_no_overlap: fill_no_overlap.clone(),
            internal_bridge_infill: Vec::new(),
            // InternalBridgeDetector.cpp:11
            spacing,
            // InternalBridgeDetector.hpp:21
            angle: -1.,
            // InternalBridgeDetector.hpp:48
            resolution: PI / 36.0,
            m_anchor_regions: Vec::new(),
        };
        // InternalBridgeDetector.cpp:13
        detector.internal_bridge_infill.push(internal_bridge);
        // InternalBridgeDetector.cpp:14
        detector.initialize();
        detector
    }

    // InternalBridgeDetector.cpp:19-42
    fn initialize(&mut self) {
        // InternalBridgeDetector.cpp:21
        // C++ `offset(ExPolygons, float)` returns `Polygons`; the crate variant returns
        // `ExPolygons`. The geometric region is the same; representation differs only.
        // FIDELITY-NOTE(F1): geo-clipper approximation vs C++ ClipperLib (offset/diff at coord_t precision).
        // FIDELITY-NOTE(F2): `spacing` is i64 here but coord_t=int32 in C++; `float(spacing)` cast magnitude is unaffected at logic level.
        let grown = offset_expolygons(
            &self.internal_bridge_infill,
            self.spacing as f64,
            OffsetJoinType::Miter,
        );
        // InternalBridgeDetector.cpp:22
        self.m_anchor_regions = difference(
            &grown,
            &offset_expolygons(&self.fill_no_overlap, 10.0, OffsetJoinType::Miter),
        );

        // InternalBridgeDetector.cpp:24-41 (INTERNAL_BRIDGE_DETECTOR_DEBUG_TO_SVG) omitted.
    }

    // InternalBridgeDetector.cpp:44-111
    pub fn detect_angle(&mut self) -> bool {
        // InternalBridgeDetector.cpp:46-47
        if self.m_anchor_regions.is_empty() {
            return false;
        }

        // InternalBridgeDetector.cpp:49-53
        let mut candidates: Vec<InternalBridgeDirection> = Vec::new();
        let angles = self.bridge_direction_candidates();
        candidates.reserve(angles.len());
        for i in 0..angles.len() {
            candidates.push(InternalBridgeDirection::new(angles[i]));
        }

        // InternalBridgeDetector.cpp:55
        let clip_area = offset_expolygons(
            &self.internal_bridge_infill,
            0.5 * self.spacing as f64,
            OffsetJoinType::Miter,
        );

        // InternalBridgeDetector.cpp:57
        let mut have_coverage = false;
        // InternalBridgeDetector.cpp:58
        for i_angle in 0..candidates.len() {
            // InternalBridgeDetector.cpp:60
            let angle = candidates[i_angle].angle;

            // InternalBridgeDetector.cpp:62
            let mut lines: Lines = Vec::new();
            {
                // InternalBridgeDetector.cpp:64
                let bbox = get_extents_rotated(&self.m_anchor_regions, -angle);
                // Cover the region with line segments.
                // InternalBridgeDetector.cpp:66
                lines.reserve(
                    ((bbox.max.y - bbox.min.y + self.spacing) / self.spacing) as usize,
                );
                // InternalBridgeDetector.cpp:67
                let s = angle.sin();
                // InternalBridgeDetector.cpp:68
                let c = angle.cos();

                // InternalBridgeDetector.cpp:70
                let mut y = bbox.min.y;
                while y <= bbox.max.y {
                    // InternalBridgeDetector.cpp:71-73
                    lines.push(Line::new(
                        Point::new(
                            (c * bbox.min.x as f64 - s * y as f64).round() as Coord,
                            (c * y as f64 + s * bbox.min.x as f64).round() as Coord,
                        ),
                        Point::new(
                            (c * bbox.max.x as f64 - s * y as f64).round() as Coord,
                            (c * y as f64 + s * bbox.max.x as f64).round() as Coord,
                        ),
                    ));
                    y += self.spacing;
                }
            }

            // InternalBridgeDetector.cpp:76
            let mut total_length: f64 = 0.;
            // InternalBridgeDetector.cpp:77
            let mut anchored_length: f64 = 0.;
            // InternalBridgeDetector.cpp:78
            let mut max_length: f64 = 0.;
            {
                // InternalBridgeDetector.cpp:80
                // FIDELITY-NOTE(F1): geo-clipper approximation vs C++ ClipperLib — the
                // underlying `intersection_pl` primitive clips by segment sampling rather
                // than exact ClipperLib `_clipper_pl_open`, so clipped line endpoints may
                // differ slightly. The front/back-of-polyline reduction matches `_clipper_ln`.
                let clipped_lines = intersection_ln(&lines, &clip_area);
                // InternalBridgeDetector.cpp:81
                for i in 0..clipped_lines.len() {
                    // InternalBridgeDetector.cpp:82
                    let line = &clipped_lines[i];
                    // InternalBridgeDetector.cpp:83
                    let len = line.length();
                    // InternalBridgeDetector.cpp:84
                    total_length += len;
                    // InternalBridgeDetector.cpp:85
                    if expolygons_contain(&self.m_anchor_regions, line.a)
                        && expolygons_contain(&self.m_anchor_regions, line.b)
                    {
                        // This line could be anchored.
                        // InternalBridgeDetector.cpp:87
                        anchored_length += len;
                        // InternalBridgeDetector.cpp:88
                        max_length = max_length.max(len);
                    }
                }
            }
            // InternalBridgeDetector.cpp:92-93
            if anchored_length == 0. {
                continue;
            }

            // InternalBridgeDetector.cpp:95
            have_coverage = true;

            // InternalBridgeDetector.cpp:97
            candidates[i_angle].coverage = anchored_length / total_length;
            // InternalBridgeDetector.cpp:98
            candidates[i_angle].max_length = max_length;
        }

        // InternalBridgeDetector.cpp:101-102
        if !have_coverage {
            return false;
        }

        // InternalBridgeDetector.cpp:104 `std::sort(candidates.begin(), candidates.end());`
        // operator< returns true when `a` is the better candidate, so the best is sorted first.
        candidates.sort_by(|a, b| {
            if a.less(b) {
                std::cmp::Ordering::Less
            } else if b.less(a) {
                std::cmp::Ordering::Greater
            } else {
                std::cmp::Ordering::Equal
            }
        });
        // InternalBridgeDetector.cpp:105
        let i_best = 0;
        // InternalBridgeDetector.cpp:106
        self.angle = candidates[i_best].angle;
        // InternalBridgeDetector.cpp:107-108
        if self.angle >= PI {
            self.angle -= PI;
        }

        // InternalBridgeDetector.cpp:110
        true
    }

    // InternalBridgeDetector.cpp:113-140
    fn bridge_direction_candidates(&self) -> Vec<f64> {
        // InternalBridgeDetector.cpp:115
        let mut angles: Vec<f64> = Vec::new();
        // InternalBridgeDetector.cpp:116-117
        // `for (int i = 0; i <= PI/this->resolution; ++i)` — integer index compared
        // against the floating-point bound PI/resolution.
        let mut i: i32 = 0;
        while (i as f64) <= PI / self.resolution {
            angles.push(i as f64 * self.resolution);
            i += 1;
        }

        // we also test angles of each bridge contour
        // InternalBridgeDetector.cpp:120-124
        {
            let lines = to_lines(&self.internal_bridge_infill);
            for line in lines.iter() {
                angles.push(line_direction(line));
            }
        }

        // remove duplicates
        // InternalBridgeDetector.cpp:127
        let min_resolution = PI / 180.0;
        // InternalBridgeDetector.cpp:128
        angles.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        // InternalBridgeDetector.cpp:129-134
        // C++: `for (size_t i = 1; i < angles.size(); ++i) { if (parallel) { erase(i); --i; } }`.
        // The `--i` followed by the for-loop's `++i` leaves `i` unchanged after an
        // erase, so the next comparison is angles[i] (the shifted element) vs
        // angles[i-1]. Mirror that net effect: hold `i` on removal, advance otherwise.
        let mut i = 1usize;
        while i < angles.len() {
            if directions_parallel(angles[i], angles[i - 1], min_resolution) {
                angles.remove(i);
            } else {
                i += 1;
            }
        }

        // InternalBridgeDetector.cpp:136-137
        if directions_parallel(angles[0], *angles.last().unwrap(), min_resolution) {
            angles.pop();
        }

        // InternalBridgeDetector.cpp:139
        angles
    }
}

// Faithful port of `Line::direction()` (Line.cpp:60-66) used at
// InternalBridgeDetector.cpp:123.
fn line_direction(line: &Line) -> f64 {
    // Line.hpp:176 `atan2_() { return atan2(b(1) - a(1), b(0) - a(0)); }`
    let atan2 = ((line.b.y - line.a.y) as f64).atan2((line.b.x - line.a.x) as f64);
    // Line.cpp:63-65
    if (atan2 - PI).abs() < crate::libslic3r::EPSILON {
        0.
    } else if atan2 < 0. {
        atan2 + PI
    } else {
        atan2
    }
}

// Faithful port of `get_extents_rotated(const ExPolygons&, double)`
// (ExPolygon.cpp:511-519) which only considers each ExPolygon's contour, and
// `get_extents_rotated(const Points&, double)` (MultiPoint.cpp:441) for the
// rotation/rounding.
fn get_extents_rotated(expolygons: &ExPolygons, angle: f64) -> BoundingBox {
    // MultiPoint.cpp:443
    let mut bbox = BoundingBox::new();
    // ExPolygon.cpp:514-517
    let s = angle.sin();
    let c = angle.cos();
    for expoly in expolygons.iter() {
        // ExPolygon.cpp:508 uses expolygon.contour only.
        for point in expoly.contour.points() {
            // MultiPoint.cpp:446-447, 451-452
            let cur_x = point.x as f64;
            let cur_y = point.y as f64;
            let x = (c * cur_x - s * cur_y).round() as Coord;
            let y = (c * cur_y + s * cur_x).round() as Coord;
            bbox.merge_point(Point::new(x, y));
        }
    }
    bbox
}

// Faithful port of `intersection_ln(const Lines&, const Polygons&)`
// (ClipperUtils.hpp:536-539 -> ClipperUtils.cpp:940-958 `_clipper_ln`).
// Converts each Line to a 2-point Polyline, clips the open polylines against
// the polygons (even-odd, so holes are honoured), then takes the front/back of
// each surviving polyline as a Line.
fn intersection_ln(subject: &Lines, clip: &ExPolygons) -> Lines {
    // ClipperUtils.cpp:942-946 convert Lines to Polylines
    let mut polylines: Vec<crate::geometry::Polyline> = Vec::with_capacity(subject.len());
    for line in subject.iter() {
        polylines.push(crate::geometry::Polyline::from_points(vec![line.a, line.b]));
    }

    // ClipperUtils.cpp:948-949 perform operation
    let polylines = clipper_utils::intersection_pl(&polylines, clip);

    // ClipperUtils.cpp:951-957 convert Polylines to Lines
    let mut retval: Lines = Vec::new();
    for polyline in polylines.iter() {
        if polyline.len() >= 2 {
            retval.push(Line::new(polyline.first_point(), polyline.last_point()));
        }
    }
    retval
}

// NOTE: the C++ `intersection_ln` takes `Polygons` for the clip; the crate's
// `intersection_pl` takes `ExPolygons`. `clip_area` here is the offset of
// `internal_bridge_infill` (an ExPolygons), passed directly. The clipper backend
// honours holes via the even-odd fill rule, matching ClipperLib's behaviour.

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::Polygon;

    fn create_test_bridge() -> ExPolygon {
        let points = vec![
            Point::new(0, 0),
            Point::new(1000, 0),
            Point::new(1000, 500),
            Point::new(0, 500),
        ];
        ExPolygon::new(Polygon::from_points(points))
    }

    fn create_test_fill() -> ExPolygons {
        let left = ExPolygon::new(Polygon::from_points(vec![
            Point::new(-500, -100),
            Point::new(200, -100),
            Point::new(200, 600),
            Point::new(-500, 600),
        ]));

        let right = ExPolygon::new(Polygon::from_points(vec![
            Point::new(800, -100),
            Point::new(1500, -100),
            Point::new(1500, 600),
            Point::new(800, 600),
        ]));

        vec![left, right]
    }

    #[test]
    fn test_detector_creation() {
        let bridge = create_test_bridge();
        let fill = create_test_fill();
        let spacing = 100;

        let detector = InternalBridgeDetector::new(bridge, &fill, spacing);

        assert_eq!(detector.spacing, spacing);
        assert_eq!(detector.internal_bridge_infill.len(), 1);
        assert_eq!(detector.angle, -1.0);
    }

    #[test]
    fn test_detector_initialization() {
        let bridge = create_test_bridge();
        let fill = create_test_fill();
        let spacing = 100;

        let detector = InternalBridgeDetector::new(bridge, &fill, spacing);

        // Should have computed anchor regions
        assert!(!detector.m_anchor_regions.is_empty());
    }

    #[test]
    fn test_detect_angle_no_anchors() {
        // Bridge with no anchor regions (should fail)
        let bridge = create_test_bridge();
        let fill = Vec::new(); // No fill regions
        let spacing = 100;

        let mut detector = InternalBridgeDetector::new(bridge, &fill, spacing);
        // With no fill, grown bridge minus empty -> anchor regions are the whole
        // grown bridge, so detection may succeed; only assert it does not panic.
        let _ = detector.detect_angle();
    }

    #[test]
    fn test_candidate_generation() {
        let bridge = create_test_bridge();
        let fill = create_test_fill();
        let spacing = 100;

        let detector = InternalBridgeDetector::new(bridge, &fill, spacing);
        let candidates = detector.bridge_direction_candidates();

        assert!(!candidates.is_empty());

        // Should be sorted
        for i in 1..candidates.len() {
            assert!(candidates[i] >= candidates[i - 1]);
        }
    }

    #[test]
    fn test_line_direction_normalizes() {
        // Horizontal line -> 0
        let l = Line::new(Point::new(0, 0), Point::new(100, 0));
        assert!((line_direction(&l)).abs() < 1e-9);

        // Vertical line -> PI/2
        let l = Line::new(Point::new(0, 0), Point::new(0, 100));
        assert!((line_direction(&l) - PI / 2.0).abs() < 1e-9);

        // Pointing left (atan2 == PI) -> 0 per C++ special case
        let l = Line::new(Point::new(0, 0), Point::new(-100, 0));
        assert!((line_direction(&l)).abs() < 1e-6);
    }
}
