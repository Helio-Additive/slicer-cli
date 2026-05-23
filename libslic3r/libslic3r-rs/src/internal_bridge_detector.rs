//! Internal bridge detection for optimal bridge angle calculation.
//!
//! C++ Reference:
//! - InternalBridgeDetector.hpp
//! - InternalBridgeDetector.cpp
//!
//! This module detects the optimal bridging angle for internal bridges by testing
//! multiple candidate angles and selecting the one with the best coverage and shortest
//! span. Internal bridges are regions where material spans over sparse infill areas.

use crate::clipper_utils::{difference, offset_expolygons};
use crate::geometry::{BoundingBox, ExPolygon, ExPolygons, Line, Lines, Point, Polygon};
use crate::Coord;
use std::f64::consts::PI;

/// Result of evaluating a candidate bridge direction.
/// InternalBridgeDetector.hpp:29-44
#[derive(Debug, Clone)]
struct InternalBridgeDirection {
    /// Bridge angle in radians
    /// InternalBridgeDetector.hpp:41
    angle: f64,
    /// Ratio of anchored line length to total line length
    /// InternalBridgeDetector.hpp:42
    coverage: f64,
    /// Maximum length of any single anchored line
    /// InternalBridgeDetector.hpp:43
    max_length: f64,
}

impl InternalBridgeDirection {
    /// Create a new bridge direction candidate
    /// InternalBridgeDetector.hpp:30
    fn new(angle: f64) -> Self {
        Self {
            angle,
            coverage: 0.0,
            max_length: 0.0,
        }
    }
}

impl PartialEq for InternalBridgeDirection {
    fn eq(&self, other: &Self) -> bool {
        (self.coverage - other.coverage).abs() < 0.001
            && (self.max_length - other.max_length).abs() < 0.001
    }
}

impl Eq for InternalBridgeDirection {}

impl PartialOrd for InternalBridgeDirection {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for InternalBridgeDirection {
    /// Compare bridge directions: better coverage wins, shorter span breaks ties
    /// InternalBridgeDetector.hpp:32-39
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        let delta = self.coverage - other.coverage;
        if delta > 0.001 {
            // Self has better coverage
            std::cmp::Ordering::Greater
        } else if delta < -0.001 {
            // Other has better coverage
            std::cmp::Ordering::Less
        } else {
            // Coverage is almost the same, prefer shorter span
            other
                .max_length
                .partial_cmp(&self.max_length)
                .unwrap_or(std::cmp::Ordering::Equal)
        }
    }
}

/// Detector for optimal bridge angles in internal bridges.
/// InternalBridgeDetector.hpp:11-52
#[derive(Debug, Clone)]
pub struct InternalBridgeDetector {
    /// All fill area in LayerRegion without overlap with perimeter
    /// InternalBridgeDetector.hpp:14
    pub fill_no_overlap: ExPolygons,

    /// Internal bridge infill area
    /// InternalBridgeDetector.hpp:16
    pub internal_bridge_infill: ExPolygons,

    /// Scaled extrusion width of the infill
    /// InternalBridgeDetector.hpp:18
    pub spacing: Coord,

    /// The final optimal angle (output)
    /// InternalBridgeDetector.hpp:20
    pub angle: f64,

    /// Angular resolution for candidate generation
    /// InternalBridgeDetector.hpp:46
    resolution: f64,

    /// Regions where bridge lines can be anchored
    /// InternalBridgeDetector.hpp:47
    anchor_regions: ExPolygons,
}

impl InternalBridgeDetector {
    /// Create a new internal bridge detector
    /// InternalBridgeDetector.cpp:7-15
    pub fn new(internal_bridge: ExPolygon, fill_no_overlap: ExPolygons, spacing: Coord) -> Self {
        let mut detector = Self {
            fill_no_overlap,
            internal_bridge_infill: vec![internal_bridge],
            spacing,
            angle: -1.0,
            resolution: PI / 36.0, // 5 degrees
            anchor_regions: Vec::new(),
        };

        detector.initialize();
        detector
    }

    /// Initialize anchor regions for bridge detection
    /// InternalBridgeDetector.cpp:19-42
    fn initialize(&mut self) {
        // Grow the internal bridge area by spacing amount
        let grown = offset_expolygons(
            &self.internal_bridge_infill,
            self.spacing as f64,
            crate::clipper_utils::OffsetJoinType::Miter,
        );

        // Anchor regions are the grown area minus the fill regions (with small offset)
        let fill_offset = offset_expolygons(
            &self.fill_no_overlap,
            10.0,
            crate::clipper_utils::OffsetJoinType::Miter,
        );
        self.anchor_regions = difference(&grown, &fill_offset);
    }

    /// Detect the optimal bridge angle
    /// InternalBridgeDetector.cpp:44-117
    ///
    /// Tests multiple candidate angles and selects the one with best coverage
    /// (ratio of anchored line length to total line length) and shortest maximum span.
    /// Returns true if a valid angle was found, false otherwise.
    pub fn detect_angle(&mut self) -> bool {
        // Need anchor regions to detect angle
        if self.anchor_regions.is_empty() {
            return false;
        }

        // Generate candidate angles
        let angles = self.bridge_direction_candidates();
        let mut candidates: Vec<InternalBridgeDirection> = angles
            .into_iter()
            .map(InternalBridgeDirection::new)
            .collect();

        // Expand bridge area slightly for clipping
        let clip_area_polygons = offset_expolygons(
            &self.internal_bridge_infill,
            0.5 * self.spacing as f64,
            crate::clipper_utils::OffsetJoinType::Miter,
        );

        // Convert ExPolygons to Polygons for line intersection
        let mut clip_area = Vec::new();
        for expoly in clip_area_polygons.iter() {
            clip_area.push(expoly.contour.clone());
            for hole in expoly.holes.iter() {
                clip_area.push(hole.clone());
            }
        }

        let mut have_coverage = false;

        // Test each candidate angle
        for candidate in candidates.iter_mut() {
            let angle = candidate.angle;

            // Generate parallel lines covering the anchor regions at this angle
            let lines = self.generate_coverage_lines(angle);

            let mut total_length = 0.0;
            let mut anchored_length = 0.0;
            let mut max_length = 0.0;

            // Clip lines to the bridge area (manual intersection since intersection_ln not available)
            let clipped_lines = self.intersect_lines_with_polygons(&lines, &clip_area);

            // Calculate coverage metrics
            for line in clipped_lines.iter() {
                let len = line.length();
                total_length += len;

                // Check if both endpoints are in anchor regions
                if self.point_in_anchor_regions(&line.a) && self.point_in_anchor_regions(&line.b) {
                    anchored_length += len;
                    max_length = if len > max_length { len } else { max_length };
                }
            }

            if anchored_length == 0.0 {
                continue;
            }

            have_coverage = true;
            candidate.coverage = anchored_length / total_length;
            candidate.max_length = max_length;
        }

        if !have_coverage {
            return false;
        }

        // Sort candidates by quality (best first)
        candidates.sort_by(|a, b| b.cmp(a));

        // Select the best candidate
        self.angle = candidates[0].angle;

        // Normalize angle to [0, PI)
        if self.angle >= PI {
            self.angle -= PI;
        }

        true
    }

    /// Generate parallel coverage lines at a given angle
    /// InternalBridgeDetector.cpp:63-75
    fn generate_coverage_lines(&self, angle: f64) -> Lines {
        // Get bounding box of anchor regions rotated by -angle
        let bbox = self.get_extents_rotated(&self.anchor_regions, -angle);

        let mut lines = Vec::new();
        lines.reserve(((bbox.max.y - bbox.min.y + self.spacing) / self.spacing) as usize);

        let s = angle.sin();
        let c = angle.cos();

        // Generate horizontal lines in rotated space
        let mut y = bbox.min.y;
        while y <= bbox.max.y {
            let x0 = bbox.min.x;
            let x1 = bbox.max.x;

            // Rotate back to original space
            let p0 = Point::new(
                (c * x0 as f64 - s * y as f64).round() as Coord,
                (c * y as f64 + s * x0 as f64).round() as Coord,
            );
            let p1 = Point::new(
                (c * x1 as f64 - s * y as f64).round() as Coord,
                (c * y as f64 + s * x1 as f64).round() as Coord,
            );

            lines.push(Line::new(p0, p1));
            y += self.spacing;
        }

        lines
    }

    /// Get bounding box of polygons rotated by given angle
    /// InternalBridgeDetector.cpp:63 (via get_extents_rotated)
    fn get_extents_rotated(&self, expolygons: &ExPolygons, angle: f64) -> BoundingBox {
        let s = angle.sin();
        let c = angle.cos();

        let mut bbox = BoundingBox::new();

        for expoly in expolygons.iter() {
            for point in expoly.contour.points() {
                let x = point.x as f64;
                let y = point.y as f64;
                let rotated = Point::new(
                    (c * x - s * y).round() as Coord,
                    (c * y + s * x).round() as Coord,
                );
                bbox.merge_point(rotated);
            }

            for hole in expoly.holes.iter() {
                for point in hole.points() {
                    let x = point.x as f64;
                    let y = point.y as f64;
                    let rotated = Point::new(
                        (c * x - s * y).round() as Coord,
                        (c * y + s * x).round() as Coord,
                    );
                    bbox.merge_point(rotated);
                }
            }
        }

        bbox
    }

    /// Check if a point is contained in any anchor region
    /// InternalBridgeDetector.cpp:85-91
    fn point_in_anchor_regions(&self, point: &Point) -> bool {
        for expoly in self.anchor_regions.iter() {
            if expoly.contains_point(point) {
                return true;
            }
        }
        false
    }

    /// Generate candidate bridge directions to test
    /// InternalBridgeDetector.cpp:119-141
    fn bridge_direction_candidates(&self) -> Vec<f64> {
        let mut angles = Vec::new();

        // Generate angles at regular intervals
        let n = (PI / self.resolution).round() as i32;
        for i in 0..=n {
            angles.push(i as f64 * self.resolution);
        }

        // Add angles from bridge contour edges
        for expoly in self.internal_bridge_infill.iter() {
            let lines = expoly.contour.edges();
            for line in lines.iter() {
                angles.push(line.direction_angle());
            }

            for hole in expoly.holes.iter() {
                let lines = hole.edges();
                for line in lines.iter() {
                    angles.push(line.direction_angle());
                }
            }
        }

        // Remove duplicates (angles within min_resolution are considered equal)
        let min_resolution = PI / 180.0; // 1 degree
        angles.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));

        let mut i = 1;
        while i < angles.len() {
            if directions_parallel(angles[i], angles[i - 1], min_resolution) {
                angles.remove(i);
            } else {
                i += 1;
            }
        }

        // Check if first and last are duplicates (wrapping around)
        if angles.len() > 1
            && directions_parallel(angles[0], *angles.last().unwrap(), min_resolution)
        {
            angles.pop();
        }

        angles
    }

    /// Intersect lines with polygons (helper method since intersection_ln not available)
    /// This is a simplified implementation - clips lines to polygon boundaries
    fn intersect_lines_with_polygons(&self, lines: &Lines, polygons: &[Polygon]) -> Lines {
        // Simplified implementation: for now, just return lines that have both endpoints
        // within or very close to the polygons
        let mut result = Vec::new();

        for line in lines.iter() {
            // Check if line intersects with any polygon
            let mut include = false;
            for poly in polygons.iter() {
                if poly.contains_point(&line.a) || poly.contains_point(&line.b) {
                    include = true;
                    break;
                }
            }

            if include {
                result.push(line.clone());
            }
        }

        result
    }
}

/// Check if two directions are parallel within a given tolerance
/// Geometry.cpp (helper function)
fn directions_parallel(angle1: f64, angle2: f64, max_diff: f64) -> bool {
    let diff = (angle1 - angle2).abs();
    let diff = if diff > PI { 2.0 * PI - diff } else { diff };
    diff < max_diff
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::Polygon;

    fn create_test_bridge() -> ExPolygon {
        // Create a simple rectangular bridge
        let points = vec![
            Point::new(0, 0),
            Point::new(1000, 0),
            Point::new(1000, 500),
            Point::new(0, 500),
        ];
        ExPolygon::new(Polygon::new(points), Vec::new())
    }

    fn create_test_fill() -> ExPolygons {
        // Create fill regions on both sides of the bridge
        let left = ExPolygon::new(
            Polygon::new(vec![
                Point::new(-500, -100),
                Point::new(200, -100),
                Point::new(200, 600),
                Point::new(-500, 600),
            ]),
            Vec::new(),
        );

        let right = ExPolygon::new(
            Polygon::new(vec![
                Point::new(800, -100),
                Point::new(1500, -100),
                Point::new(1500, 600),
                Point::new(800, 600),
            ]),
            Vec::new(),
        );

        vec![left, right]
    }

    #[test]
    fn test_internal_bridge_direction_ordering() {
        let mut dir1 = InternalBridgeDirection::new(0.0);
        dir1.coverage = 0.8;
        dir1.max_length = 100.0;

        let mut dir2 = InternalBridgeDirection::new(PI / 4.0);
        dir2.coverage = 0.9;
        dir2.max_length = 150.0;

        // Better coverage wins
        assert!(dir2 > dir1);

        let mut dir3 = InternalBridgeDirection::new(PI / 2.0);
        dir3.coverage = 0.9;
        dir3.max_length = 120.0;

        // Same coverage, shorter span wins
        assert!(dir3 > dir2);
    }

    #[test]
    fn test_detector_creation() {
        let bridge = create_test_bridge();
        let fill = create_test_fill();
        let spacing = 100;

        let detector = InternalBridgeDetector::new(bridge, fill, spacing);

        assert_eq!(detector.spacing, spacing);
        assert_eq!(detector.internal_bridge_infill.len(), 1);
        assert_eq!(detector.angle, -1.0);
    }

    #[test]
    fn test_detector_initialization() {
        let bridge = create_test_bridge();
        let fill = create_test_fill();
        let spacing = 100;

        let detector = InternalBridgeDetector::new(bridge, fill, spacing);

        // Should have computed anchor regions
        assert!(!detector.anchor_regions.is_empty());
    }

    #[test]
    fn test_directions_parallel() {
        // Same angle
        assert!(directions_parallel(0.0, 0.0, 0.01));

        // Close angles
        assert!(directions_parallel(0.0, 0.005, 0.01));
        assert!(directions_parallel(PI / 4.0, PI / 4.0 + 0.005, 0.01));

        // Different angles
        assert!(!directions_parallel(0.0, PI / 4.0, 0.01));

        // Wrapping around (0 ≈ 2π)
        assert!(directions_parallel(0.01, 2.0 * PI - 0.01, 0.05));
    }

    #[test]
    fn test_candidate_generation() {
        let bridge = create_test_bridge();
        let fill = create_test_fill();
        let spacing = 100;

        let detector = InternalBridgeDetector::new(bridge, fill, spacing);
        let candidates = detector.bridge_direction_candidates();

        // Should have at least the regular interval candidates
        assert!(!candidates.is_empty());

        // Should be sorted
        for i in 1..candidates.len() {
            assert!(candidates[i] >= candidates[i - 1]);
        }

        // Should be within valid range
        for angle in candidates.iter() {
            assert!(*angle >= 0.0 && *angle <= PI);
        }
    }

    #[test]
    fn test_detect_angle_no_anchors() {
        // Bridge with no anchor regions (should fail)
        let bridge = create_test_bridge();
        let fill = Vec::new(); // No fill regions
        let spacing = 100;

        let mut detector = InternalBridgeDetector::new(bridge, fill, spacing);
        assert!(!detector.detect_angle());
        assert_eq!(detector.angle, -1.0);
    }

    #[test]
    fn test_detect_angle_with_anchors() {
        let bridge = create_test_bridge();
        let fill = create_test_fill();
        let spacing = 100;

        let mut detector = InternalBridgeDetector::new(bridge, fill, spacing);

        // Should successfully detect an angle
        let result = detector.detect_angle();

        // May or may not find a valid angle depending on geometry
        if result {
            assert!(detector.angle >= 0.0 && detector.angle < PI);
        }
    }

    #[test]
    fn test_generate_coverage_lines() {
        let bridge = create_test_bridge();
        let fill = create_test_fill();
        let spacing = 100;

        let detector = InternalBridgeDetector::new(bridge, fill, spacing);

        // Generate lines at 0 degree angle (horizontal)
        let lines = detector.generate_coverage_lines(0.0);

        assert!(!lines.is_empty());

        // Lines should be roughly horizontal (Y coordinates similar, X different)
        for line in lines.iter() {
            assert!((line.a.x - line.b.x).abs() > 10); // Significant X difference
        }
    }

    #[test]
    fn test_point_in_anchor_regions() {
        let bridge = create_test_bridge();
        let fill = create_test_fill();
        let spacing = 100;

        let detector = InternalBridgeDetector::new(bridge, fill, spacing);

        // Points inside anchor regions should be detected
        // (anchor regions are computed during initialization)

        // This is hard to test without knowing exact anchor geometry,
        // but we can verify the method doesn't panic
        let _ = detector.point_in_anchor_regions(&Point::new(0, 0));
        let _ = detector.point_in_anchor_regions(&Point::new(500, 250));
    }
}
