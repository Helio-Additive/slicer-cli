//! Travel path planning module.
//!
//! This module provides travel path optimization, including the AvoidCrossingPerimeters
//! algorithm that routes travel moves around perimeter walls to avoid crossing them.
//!
//! # libslic3r Mapping
//!
//! This module corresponds to `GCode/AvoidCrossingPerimeters.cpp` and
//! `GCode/AvoidCrossingPerimeters.hpp` in BambuStudio/libslic3r.
//!
//! # Overview
//!
//! When the print head travels from one point to another without extruding, crossing
//! over already-printed perimeter walls can leave visible marks on the surface.
//! The AvoidCrossingPerimeters algorithm finds alternative travel paths that go
//! around perimeters instead of through them.
//!
//! # Algorithm
//!
//! 1. Build boundary polygons from layer perimeters (offset inward slightly)
//! 2. For each travel move, check if it crosses any boundaries
//! 3. If crossing detected, find entry/exit points on the boundaries
//! 4. Route around the boundary using shortest path (forward or backward)
//! 5. Simplify the resulting path to remove unnecessary points
//!
//! # Example
//!
//! ```ignore
//! use slicer::travel::{AvoidCrossingPerimeters, TravelConfig};
//! use slicer::geometry::{Point, Polygon, Polyline};
//!
//! let boundaries = vec![/* layer perimeter polygons */];
//! let config = TravelConfig::default();
//! let mut avoid = AvoidCrossingPerimeters::new(config);
//!
//! avoid.init_layer(&boundaries, 200_000); // 0.2mm perimeter spacing
//!
//! let start = Point::new(100_000, 100_000);
//! let end = Point::new(900_000, 900_000);
//!
//! let travel_path = avoid.travel_to(&start, &end);
//! ```

use crate::edge_grid::{EdgeGrid, Intersection};
use crate::geometry::{BoundingBox, Point, Polygon, Polyline};

/// Configuration for travel path planning.
#[derive(Clone, Debug)]
pub struct TravelConfig {
    /// Whether avoid crossing perimeters is enabled.
    pub enabled: bool,
    /// Maximum detour as a percentage of direct travel distance.
    /// If the detour exceeds this, fall back to direct travel.
    pub max_detour_percent: f64,
    /// Maximum absolute detour distance (in scaled units).
    /// If set to 0, only percentage limit is used.
    pub max_detour_absolute: i64,
    /// Resolution for the edge grid (in scaled units).
    pub grid_resolution: i64,
    /// Epsilon for offsetting points inside boundaries.
    pub boundary_offset: i64,
}

impl Default for TravelConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            max_detour_percent: 200.0, // 2x direct distance
            max_detour_absolute: 0,
            grid_resolution: 100_000, // 0.1mm
            boundary_offset: 1_000,   // 1 micron
        }
    }
}

impl TravelConfig {
    // Create a new travel config with defaults.
    pub fn new() -> Self {
        Self::default()
    }

    /// Enable or disable avoid crossing perimeters.
    pub fn with_enabled(mut self, enabled: bool) -> Self {
        self.enabled = enabled;
        self
    }

    /// Set the maximum detour percentage.
    pub fn with_max_detour_percent(mut self, percent: f64) -> Self {
        self.max_detour_percent = percent;
        self
    }

    /// Set the maximum absolute detour.
    pub fn with_max_detour_absolute(mut self, distance: i64) -> Self {
        self.max_detour_absolute = distance;
        self
    }

    /// Set the grid resolution.
    pub fn with_grid_resolution(mut self, resolution: i64) -> Self {
        self.grid_resolution = resolution;
        self
    }
}

/// A point along the travel path with metadata.
#[derive(Clone, Debug)]
struct TravelPoint {
    /// The point location.
    point: Point,
    /// Index of the boundary polygon this point is on (-1 if not on boundary).
    boundary_idx: i32,
    /// Whether this point should not be removed during simplification.
    do_not_remove: bool,
}

impl TravelPoint {
    fn new(point: Point, boundary_idx: i32) -> Self {
        Self {
            point,
            boundary_idx,
            do_not_remove: false,
        }
    }

    fn with_do_not_remove(mut self, do_not_remove: bool) -> Self {
        self.do_not_remove = do_not_remove;
        self
    }
}

/// Direction for walking around a boundary polygon.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Direction {
    Forward,
    Backward,
}

/// Boundary data for avoid crossing perimeters.
#[derive(Clone, Debug)]
struct Boundary {
    /// The boundary polygons.
    polygons: Vec<Polygon>,
    /// Bounding box of all boundaries.
    bbox: BoundingBox,
    /// Pre-computed cumulative distances along each polygon.
    polygon_params: Vec<Vec<f64>>,
    /// Edge grid for fast intersection testing.
    grid: EdgeGrid,
}

impl Boundary {
    fn new() -> Self {
        Self {
            polygons: Vec::new(),
            bbox: BoundingBox::new(),
            polygon_params: Vec::new(),
            grid: EdgeGrid::new(),
        }
    }

    fn clear(&mut self) {
        self.polygons.clear();
        self.polygon_params.clear();
        self.bbox = BoundingBox::new();
        self.grid = EdgeGrid::new();
    }

    fn is_empty(&self) -> bool {
        self.polygons.is_empty()
    }

    /// Initialize the boundary from polygons.
    fn init(&mut self, polygons: Vec<Polygon>, resolution: i64) {
        self.polygons = polygons;

        // Compute bounding box
        self.bbox = BoundingBox::new();
        for poly in &self.polygons {
            for point in poly.points() {
                self.bbox.merge_point(*point);
            }
        }

        // Pre-compute cumulative distances along each polygon
        self.polygon_params.clear();
        for poly in &self.polygons {
            let mut params = Vec::with_capacity(poly.points().len());
            let mut cumulative = 0.0;
            params.push(cumulative);

            let points = poly.points();
            for i in 0..points.len() {
                let next_i = if i + 1 >= points.len() { 0 } else { i + 1 };
                let dx = (points[next_i].x - points[i].x) as f64;
                let dy = (points[next_i].y - points[i].y) as f64;
                cumulative += (dx * dx + dy * dy).sqrt();
                params.push(cumulative);
            }
            self.polygon_params.push(params);
        }

        // Create edge grid
        self.grid = EdgeGrid::from_polygons(&self.polygons, resolution);
    }

    /// Get the total perimeter length of a polygon.
    fn polygon_length(&self, poly_idx: usize) -> f64 {
        if poly_idx >= self.polygon_params.len() {
            return 0.0;
        }
        let params = &self.polygon_params[poly_idx];
        if params.is_empty() {
            return 0.0;
        }
        *params.last().unwrap()
    }

    /// Get the distance along a polygon from start to a segment index.
    fn distance_to_segment(&self, poly_idx: usize, seg_idx: usize) -> f64 {
        if poly_idx >= self.polygon_params.len() {
            return 0.0;
        }
        let params = &self.polygon_params[poly_idx];
        if seg_idx >= params.len() {
            return 0.0;
        }
        params[seg_idx]
    }
}

/// Result of travel planning.
#[derive(Clone, Debug)]
pub struct TravelResult {
    /// The travel path as a polyline.
    pub path: Polyline,
    /// Number of boundary crossings in the original direct path.
    pub original_crossings: usize,
    /// Whether the path was modified to avoid crossings.
    pub path_modified: bool,
    /// Whether wipe should be disabled for this travel.
    pub wipe_disabled: bool,
}

impl TravelResult {
    // Create a simple direct travel result.
    pub fn direct(start: Point, end: Point) -> Self {
        Self {
            path: Polyline::from_points(vec![start, end]),
            original_crossings: 0,
            path_modified: false,
            wipe_disabled: false,
        }
    }
}

/// Avoid Crossing Perimeters travel planner.
///
/// Routes travel moves around perimeter boundaries to avoid crossing them.
pub struct AvoidCrossingPerimeters {
    /// Configuration.
    config: TravelConfig,
    /// Use external (between objects) boundaries.
    use_external: bool,
    /// Use external boundaries for next move only.
    use_external_once: bool,
    /// Disable avoid crossing for next move only.
    disabled_once: bool,
    /// Internal boundaries (within object).
    internal: Boundary,
    /// External boundaries (between objects).
    external: Boundary,
    /// Current perimeter spacing.
    perimeter_spacing: i64,
}

impl AvoidCrossingPerimeters {
    // Create a new avoid crossing perimeters planner.
    pub fn new(config: TravelConfig) -> Self {
        Self {
            config,
            use_external: false,
            use_external_once: false,
            disabled_once: true, // Disabled for first move
            internal: Boundary::new(),
            external: Boundary::new(),
            perimeter_spacing: 400_000, // Default 0.4mm
        }
    }

    /// Create with default configuration.
    pub fn with_defaults() -> Self {
        Self::new(TravelConfig::default())
    }

    /// Check if avoid crossing is enabled.
    pub fn is_enabled(&self) -> bool {
        self.config.enabled
    }

    /// Set whether to use external boundaries.
    pub fn use_external_mp(&mut self, use_external: bool) {
        self.use_external = use_external;
    }

    /// Set to use external boundaries for next move only.
    pub fn use_external_mp_once(&mut self) {
        self.use_external_once = true;
    }

    /// Disable avoid crossing for next move only.
    pub fn disable_once(&mut self) {
        self.disabled_once = true;
    }

    /// Reset per-move flags.
    pub fn reset_once_modifiers(&mut self) {
        self.use_external_once = false;
        self.disabled_once = false;
    }

    /// Initialize for a new layer with internal boundaries.
    pub fn init_layer(&mut self, boundaries: &[Polygon], perimeter_spacing: i64) {
        self.internal.clear();
        self.external.clear();
        self.perimeter_spacing = perimeter_spacing;

        if !self.config.enabled || boundaries.is_empty() {
            return;
        }

        // Offset boundaries inward by half perimeter spacing
        // For now, use boundaries directly (proper offset requires clipper)
        self.internal
            .init(boundaries.to_vec(), self.config.grid_resolution);
    }

    /// Initialize external boundaries (for multi-object prints).
    pub fn init_external_boundaries(&mut self, boundaries: &[Polygon]) {
        if !self.config.enabled || boundaries.is_empty() {
            return;
        }

        self.external
            .init(boundaries.to_vec(), self.config.grid_resolution);
    }

    /// Plan a travel move from start to end.
    pub fn travel_to(&mut self, start: &Point, end: &Point) -> TravelResult {
        // Check if disabled
        if !self.config.enabled || self.disabled_once {
            self.reset_once_modifiers();
            return TravelResult::direct(*start, *end);
        }

        let use_external = self.use_external || self.use_external_once;
        self.reset_once_modifiers();

        // Select appropriate boundary
        let boundary = if use_external {
            &self.external
        } else {
            &self.internal
        };

        if boundary.is_empty() {
            return TravelResult::direct(*start, *end);
        }

        // Find intersections with boundaries
        let intersections = boundary.grid.find_intersections(start, end);

        if intersections.is_empty() {
            // No crossings, use direct path
            return TravelResult::direct(*start, *end);
        }

        // Avoid perimeters
        let (result_path, crossing_count) =
            self.avoid_perimeters_inner(boundary, start, end, &intersections);

        // Check if detour is acceptable
        let direct_length = Self::distance(start, end);
        let path_length = result_path.length();
        let detour = path_length - direct_length;

        let max_detour = if self.config.max_detour_absolute > 0 {
            (self.config.max_detour_absolute as f64)
                .min(direct_length * self.config.max_detour_percent / 100.0)
        } else {
            direct_length * self.config.max_detour_percent / 100.0
        };

        if detour > max_detour {
            // Detour too long, use direct path
            return TravelResult {
                path: Polyline::from_points(vec![*start, *end]),
                original_crossings: crossing_count,
                path_modified: false,
                wipe_disabled: false,
            };
        }

        TravelResult {
            path: result_path,
            original_crossings: crossing_count,
            path_modified: true,
            wipe_disabled: crossing_count == 0,
        }
    }

    /// Internal: avoid perimeters and build path.
    fn avoid_perimeters_inner(
        &self,
        boundary: &Boundary,
        start: &Point,
        end: &Point,
        intersections: &[Intersection],
    ) -> (Polyline, usize) {
        let mut result: Vec<TravelPoint> = Vec::new();
        result.push(TravelPoint::new(*start, -1));

        let mut it_first_idx = 0;

        while it_first_idx < intersections.len() {
            let intersection_first = &intersections[it_first_idx];

            // Find the last intersection with the same boundary
            let mut it_second_idx = None;
            for j in (it_first_idx + 1..intersections.len()).rev() {
                if intersections[j].contour_idx == intersection_first.contour_idx {
                    it_second_idx = Some(j);
                    break;
                }
            }

            // Add entry point (offset inward)
            let entry_point = self.offset_point_inward(
                boundary,
                intersection_first.contour_idx,
                intersection_first.segment_idx,
                &intersection_first.point,
            );
            result.push(TravelPoint::new(
                entry_point,
                intersection_first.contour_idx as i32,
            ));

            if let Some(second_idx) = it_second_idx {
                let intersection_second = &intersections[second_idx];

                // Determine shortest direction around the polygon
                let direction = self.get_shortest_direction(
                    boundary,
                    intersection_first.contour_idx,
                    intersection_first.segment_idx,
                    intersection_second.segment_idx,
                );

                // Walk around the polygon boundary
                let poly = &boundary.polygons[intersection_first.contour_idx];
                let poly_len = poly.points().len();

                match direction {
                    Direction::Forward => {
                        let mut seg_idx = intersection_first.segment_idx;
                        while seg_idx != intersection_second.segment_idx {
                            seg_idx = (seg_idx + 1) % poly_len;
                            let _point = poly.points()[seg_idx];
                            let offset_point = self.offset_vertex(
                                boundary,
                                intersection_first.contour_idx,
                                seg_idx,
                            );
                            result.push(TravelPoint::new(
                                offset_point,
                                intersection_first.contour_idx as i32,
                            ));
                            // Avoid infinite loop
                            if result.len() > poly_len + 10 {
                                break;
                            }
                        }
                    }
                    Direction::Backward => {
                        let mut seg_idx = intersection_first.segment_idx;
                        while seg_idx != intersection_second.segment_idx {
                            let _point = poly.points()[seg_idx];
                            let offset_point = self.offset_vertex(
                                boundary,
                                intersection_first.contour_idx,
                                seg_idx,
                            );
                            result.push(TravelPoint::new(
                                offset_point,
                                intersection_first.contour_idx as i32,
                            ));
                            seg_idx = if seg_idx == 0 {
                                poly_len - 1
                            } else {
                                seg_idx - 1
                            };
                            // Avoid infinite loop
                            if result.len() > poly_len + 10 {
                                break;
                            }
                        }
                    }
                }

                // Add exit point
                let exit_point = self.offset_point_inward(
                    boundary,
                    intersection_second.contour_idx,
                    intersection_second.segment_idx,
                    &intersection_second.point,
                );
                result.push(TravelPoint::new(
                    exit_point,
                    intersection_second.contour_idx as i32,
                ));

                // Skip to after the last intersection we processed
                it_first_idx = second_idx + 1;
            } else {
                it_first_idx += 1;
            }
        }

        result.push(TravelPoint::new(*end, -1));

        // Simplify the path
        let simplified = self.simplify_travel(boundary, &result);

        // Convert to polyline
        let points: Vec<Point> = simplified.iter().map(|tp| tp.point).collect();

        (Polyline::from_points(points), intersections.len())
    }

    /// Get the shortest direction around a polygon between two segments.
    fn get_shortest_direction(
        &self,
        boundary: &Boundary,
        poly_idx: usize,
        seg1: usize,
        seg2: usize,
    ) -> Direction {
        let total_length = boundary.polygon_length(poly_idx);
        if total_length <= 0.0 {
            return Direction::Forward;
        }

        let dist1 = boundary.distance_to_segment(poly_idx, seg1);
        let dist2 = boundary.distance_to_segment(poly_idx, seg2);

        let forward_dist = if dist2 >= dist1 {
            dist2 - dist1
        } else {
            total_length - dist1 + dist2
        };

        let backward_dist = total_length - forward_dist;

        if forward_dist <= backward_dist {
            Direction::Forward
        } else {
            Direction::Backward
        }
    }

    /// Offset a point inward from a polygon edge.
    fn offset_point_inward(
        &self,
        boundary: &Boundary,
        poly_idx: usize,
        seg_idx: usize,
        point: &Point,
    ) -> Point {
        if poly_idx >= boundary.polygons.len() {
            return *point;
        }

        let poly = &boundary.polygons[poly_idx];
        let points = poly.points();
        if points.len() < 2 {
            return *point;
        }

        let p1 = &points[seg_idx];
        let p2 = &points[(seg_idx + 1) % points.len()];

        // Compute inward normal
        let dx = (p2.x - p1.x) as f64;
        let dy = (p2.y - p1.y) as f64;
        let len = (dx * dx + dy * dy).sqrt();

        if len < 1.0 {
            return *point;
        }

        // Normal pointing inward (assuming CCW winding for outer contour)
        let nx = dy / len;
        let ny = -dx / len;

        let offset = self.config.boundary_offset as f64;
        Point::new(
            (point.x as f64 + nx * offset) as i64,
            (point.y as f64 + ny * offset) as i64,
        )
    }

    /// Offset a polygon vertex inward.
    fn offset_vertex(&self, boundary: &Boundary, poly_idx: usize, vertex_idx: usize) -> Point {
        if poly_idx >= boundary.polygons.len() {
            return Point::new(0, 0);
        }

        let poly = &boundary.polygons[poly_idx];
        let points = poly.points();
        let n = points.len();

        if n < 3 {
            return points.get(vertex_idx).copied().unwrap_or(Point::new(0, 0));
        }

        let prev_idx = if vertex_idx == 0 {
            n - 1
        } else {
            vertex_idx - 1
        };
        let next_idx = (vertex_idx + 1) % n;

        let prev = &points[prev_idx];
        let curr = &points[vertex_idx];
        let next = &points[next_idx];

        // Compute inward normal as average of edge normals
        let v1 = ((curr.x - prev.x) as f64, (curr.y - prev.y) as f64);
        let v2 = ((next.x - curr.x) as f64, (next.y - curr.y) as f64);

        let len1 = (v1.0 * v1.0 + v1.1 * v1.1).sqrt();
        let len2 = (v2.0 * v2.0 + v2.1 * v2.1).sqrt();

        if len1 < 1.0 || len2 < 1.0 {
            return *curr;
        }

        // Normals (rotated 90 degrees)
        let n1 = (v1.1 / len1, -v1.0 / len1);
        let n2 = (v2.1 / len2, -v2.0 / len2);

        // Average normal
        let avg_nx = (n1.0 + n2.0) / 2.0;
        let avg_ny = (n1.1 + n2.1) / 2.0;
        let avg_len = (avg_nx * avg_nx + avg_ny * avg_ny).sqrt();

        if avg_len < 0.01 {
            return *curr;
        }

        let offset = self.config.boundary_offset as f64;
        Point::new(
            (curr.x as f64 + avg_nx / avg_len * offset) as i64,
            (curr.y as f64 + avg_ny / avg_len * offset) as i64,
        )
    }

    /// Simplify the travel path by removing unnecessary points.
    fn simplify_travel(&self, boundary: &Boundary, path: &[TravelPoint]) -> Vec<TravelPoint> {
        if path.len() <= 2 {
            return path.to_vec();
        }

        let mut result: Vec<TravelPoint> = Vec::with_capacity(path.len());
        result.push(path[0].clone());

        let mut current_idx = 0;

        while current_idx < path.len() - 1 {
            let current = &path[current_idx];

            // Try to skip to the furthest point that doesn't cause new crossings
            let mut best_next_idx = current_idx + 1;

            for try_idx in (current_idx + 2)..path.len() {
                let try_point = &path[try_idx];

                // Check if direct path causes crossings
                if !boundary
                    .grid
                    .line_intersects_any(&current.point, &try_point.point)
                {
                    // No crossings, we can skip to this point
                    best_next_idx = try_idx;
                } else if path[try_idx].do_not_remove {
                    // Can't skip past do_not_remove points
                    break;
                }
            }

            result.push(path[best_next_idx].clone());
            current_idx = best_next_idx;
        }

        result
    }

    /// Calculate distance between two points.
    fn distance(p1: &Point, p2: &Point) -> f64 {
        let dx = (p2.x - p1.x) as f64;
        let dy = (p2.y - p1.y) as f64;
        (dx * dx + dy * dy).sqrt()
    }
}

impl Default for AvoidCrossingPerimeters {
    fn default() -> Self {
        Self::with_defaults()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_square_boundary() -> Vec<Polygon> {
        vec![Polygon::from_points(vec![
            Point::new(100_000, 100_000),
            Point::new(900_000, 100_000),
            Point::new(900_000, 900_000),
            Point::new(100_000, 900_000),
        ])]
    }

    #[test]
    fn test_travel_config_default() {
        let config = TravelConfig::default();
        assert!(config.enabled);
        assert_eq!(config.max_detour_percent, 200.0);
    }

    #[test]
    fn test_travel_config_builder() {
        let config = TravelConfig::new()
            .with_enabled(false)
            .with_max_detour_percent(150.0)
            .with_grid_resolution(50_000);

        assert!(!config.enabled);
        assert_eq!(config.max_detour_percent, 150.0);
        assert_eq!(config.grid_resolution, 50_000);
    }

    #[test]
    fn test_avoid_crossing_disabled() {
        let config = TravelConfig::new().with_enabled(false);
        let mut avoid = AvoidCrossingPerimeters::new(config);

        let boundaries = make_square_boundary();
        avoid.init_layer(&boundaries, 400_000);

        let start = Point::new(0, 500_000);
        let end = Point::new(1_000_000, 500_000);

        let result = avoid.travel_to(&start, &end);
        assert!(!result.path_modified);
        assert_eq!(result.path.points().len(), 2);
    }

    #[test]
    fn test_avoid_crossing_no_intersection() {
        let mut avoid = AvoidCrossingPerimeters::with_defaults();

        let boundaries = make_square_boundary();
        avoid.init_layer(&boundaries, 400_000);

        // Travel outside the square boundary
        let start = Point::new(0, 50_000);
        let end = Point::new(1_000_000, 50_000);

        let result = avoid.travel_to(&start, &end);
        assert!(!result.path_modified);
        assert_eq!(result.path.points().len(), 2);
    }

    #[test]
    fn test_avoid_crossing_with_intersection() {
        let mut avoid = AvoidCrossingPerimeters::with_defaults();

        let boundaries = make_square_boundary();
        avoid.init_layer(&boundaries, 400_000);

        // First travel is disabled by default (disabled_once = true for first move)
        // so we need to reset or do a first move
        avoid.reset_once_modifiers();

        // Travel through the square boundary
        let start = Point::new(0, 500_000);
        let end = Point::new(1_000_000, 500_000);

        let result = avoid.travel_to(&start, &end);
        assert!(result.original_crossings > 0);
        // Path may or may not be modified depending on detour limits
    }

    #[test]
    fn test_disable_once() {
        let mut avoid = AvoidCrossingPerimeters::with_defaults();

        let boundaries = make_square_boundary();
        avoid.init_layer(&boundaries, 400_000);

        avoid.disable_once();

        let start = Point::new(0, 500_000);
        let end = Point::new(1_000_000, 500_000);

        let result = avoid.travel_to(&start, &end);
        assert!(!result.path_modified); // Should be disabled for this move

        // Next move should work normally
        let result2 = avoid.travel_to(&start, &end);
        // original_crossings should be detected if crossing occurs
    }

    #[test]
    fn test_empty_boundaries() {
        let mut avoid = AvoidCrossingPerimeters::with_defaults();
        avoid.init_layer(&[], 400_000);

        let start = Point::new(0, 500_000);
        let end = Point::new(1_000_000, 500_000);

        let result = avoid.travel_to(&start, &end);
        assert!(!result.path_modified);
        assert_eq!(result.path.points().len(), 2);
    }

    #[test]
    fn test_travel_result_direct() {
        let start = Point::new(0, 0);
        let end = Point::new(100, 100);

        let result = TravelResult::direct(start, end);
        assert!(!result.path_modified);
        assert_eq!(result.original_crossings, 0);
        assert_eq!(result.path.points().len(), 2);
    }

    #[test]
    fn test_boundary_init() {
        let mut boundary = Boundary::new();
        assert!(boundary.is_empty());

        let polygons = make_square_boundary();
        boundary.init(polygons, 100_000);

        assert!(!boundary.is_empty());
        assert_eq!(boundary.polygons.len(), 1);
        assert!(!boundary.bbox.is_empty());
    }

    #[test]
    fn test_direction_shortest() {
        let mut avoid = AvoidCrossingPerimeters::with_defaults();
        let boundaries = make_square_boundary();
        avoid.init_layer(&boundaries, 400_000);

        // The direction should be Forward when seg1 < seg2 and difference is small
        let dir = avoid.get_shortest_direction(&avoid.internal, 0, 0, 1);
        assert_eq!(dir, Direction::Forward);
    }

    #[test]
    fn test_simplify_empty_path() {
        let avoid = AvoidCrossingPerimeters::with_defaults();
        let boundary = Boundary::new();

        let path: Vec<TravelPoint> = vec![];
        let simplified = avoid.simplify_travel(&boundary, &path);
        assert!(simplified.is_empty());
    }

    #[test]
    fn test_simplify_two_point_path() {
        let avoid = AvoidCrossingPerimeters::with_defaults();
        let boundary = Boundary::new();

        let path = vec![
            TravelPoint::new(Point::new(0, 0), -1),
            TravelPoint::new(Point::new(100, 100), -1),
        ];
        let simplified = avoid.simplify_travel(&boundary, &path);
        assert_eq!(simplified.len(), 2);
    }

    #[test]
    fn test_polygon_length() {
        let mut boundary = Boundary::new();
        let square = Polygon::from_points(vec![
            Point::new(0, 0),
            Point::new(1_000_000, 0),
            Point::new(1_000_000, 1_000_000),
            Point::new(0, 1_000_000),
        ]);
        boundary.init(vec![square], 100_000);

        let length = boundary.polygon_length(0);
        // Square perimeter = 4 * 1mm = 4mm = 4_000_000 scaled units
        assert!((length - 4_000_000.0).abs() < 1000.0);
    }
}
