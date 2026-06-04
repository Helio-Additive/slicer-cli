//! Retract when crossing perimeters.
//!
//! This module determines whether to perform retraction when a travel move
//! crosses perimeter boundaries. It helps reduce stringing by retracting
//! when the nozzle travels outside internal regions or crosses perimeters.
//!
//! # Overview
//!
//! The module provides two main checks:
//! 1. Whether the travel is entirely within internal regions (no retraction needed)
//! 2. Whether the travel crosses any perimeter lines (retraction needed)
//!
//! If a travel move stays within internal regions AND doesn't cross perimeters,
//! retraction can be skipped since any stringing will be hidden inside the print.
//!
//! # BambuStudio Reference
//!
//! This module corresponds to:
//! - `src/libslic3r/GCode/RetractWhenCrossingPerimeters.cpp`
//! - `src/libslic3r/GCode/RetractWhenCrossingPerimeters.hpp`

use crate::geometry::{BoundingBox, ExPolygon, Line, Point, Polyline};
use crate::layer::Layer;
use crate::{scale, CoordF};

/// Result of checking whether retraction is needed for a travel move.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RetractDecision {
    /// Retraction is needed (travel crosses perimeters or exits internal regions).
    Retract,
    /// No retraction needed (travel stays within internal regions without crossing perimeters).
    NoRetract,
}

impl RetractDecision {
    // Returns true if retraction should be performed.
    #[inline]
    pub fn should_retract(&self) -> bool {
        matches!(self, RetractDecision::Retract)
    }
}

/// Determines whether to retract when crossing perimeters during travel moves.
///
/// This struct caches information about internal regions and perimeter lines
/// for efficient repeated queries on the same layer.
pub struct RetractWhenCrossingPerimeters {
    /// The layer currently cached (by layer ID).
    cached_layer_id: Option<usize>,

    /// Internal islands (regions where stringing is not visible).
    /// These are references to internal surfaces' ExPolygons.
    internal_islands: Vec<ExPolygon>,

    /// Bounding boxes for internal islands (for fast rejection).
    internal_islands_bboxes: Vec<BoundingBox>,

    /// Perimeter and internal surface boundary lines for intersection testing.
    perimeter_lines: Vec<Line>,

    /// Bounding box of all perimeter lines.
    perimeter_lines_bbox: BoundingBox,

    /// Flag indicating perimeter lines have been cached for current layer.
    perimeters_cached: bool,
}

impl Default for RetractWhenCrossingPerimeters {
    fn default() -> Self {
        Self::new()
    }
}

impl RetractWhenCrossingPerimeters {
    // Create a new instance.
    pub fn new() -> Self {
        Self {
            cached_layer_id: None,
            internal_islands: Vec::new(),
            internal_islands_bboxes: Vec::new(),
            perimeter_lines: Vec::new(),
            perimeter_lines_bbox: BoundingBox::new(),
            perimeters_cached: false,
        }
    }

    /// Clear all cached data.
    pub fn clear(&mut self) {
        self.cached_layer_id = None;
        self.internal_islands.clear();
        self.internal_islands_bboxes.clear();
        self.perimeter_lines.clear();
        self.perimeter_lines_bbox = BoundingBox::new();
        self.perimeters_cached = false;
    }

    /// Check if a travel move should skip retraction because it stays inside
    /// internal regions and doesn't cross any perimeters.
    ///
    /// Returns `NoRetract` if the travel is entirely within an internal region
    /// AND doesn't cross any perimeter lines.
    ///
    /// # Arguments
    ///
    /// * `layer` - The current layer being printed
    /// * `travel` - The travel path (polyline from current position to destination)
    ///
    /// # Returns
    ///
    /// * `RetractDecision::NoRetract` if retraction can be skipped
    /// * `RetractDecision::Retract` if retraction is recommended
    pub fn check_travel(&mut self, layer: &Layer, travel: &Polyline) -> RetractDecision {
        // Empty travel doesn't need retraction
        if travel.is_empty() || travel.points().len() < 2 {
            return RetractDecision::NoRetract;
        }

        // First check if travel is inside internal regions
        if !self.travel_inside_internal_regions(layer, travel) {
            return RetractDecision::Retract;
        }

        // Then check if travel crosses any perimeters
        if self.travel_crosses_perimeters(layer, travel) {
            return RetractDecision::Retract;
        }

        // Travel is inside internal regions and doesn't cross perimeters
        RetractDecision::NoRetract
    }

    /// Check if the travel path is entirely contained within internal regions.
    ///
    /// Internal regions are areas where sparse infill will be printed, so any
    /// stringing in these areas will be hidden by the infill.
    fn travel_inside_internal_regions(&mut self, layer: &Layer, travel: &Polyline) -> bool {
        // Update cache if layer changed
        self.update_internal_islands_cache(layer);

        if self.internal_islands.is_empty() {
            return false;
        }

        // Get travel bounding box for quick rejection
        let travel_bbox = travel.bounding_box();

        // Check each internal island
        for (island, island_bbox) in self
            .internal_islands
            .iter()
            .zip(self.internal_islands_bboxes.iter())
        {
            // Quick bounding box rejection
            if !travel_bbox.intersects(island_bbox) {
                continue;
            }

            // Check if travel is completely inside this island
            if self.polyline_inside_expolygon(travel, island) {
                return true;
            }
        }

        false
    }

    /// Check if a polyline is completely inside an ExPolygon.
    fn polyline_inside_expolygon(&self, polyline: &Polyline, expolygon: &ExPolygon) -> bool {
        // All points must be inside
        for point in polyline.points() {
            if !expolygon.contains_point(point) {
                return false;
            }
        }

        // Additionally, check that no line segment exits and re-enters
        // (could happen even if endpoints are inside)
        for edge in polyline.edges() {
            // Check midpoint as a simple heuristic
            let mid = edge.midpoint();
            if !expolygon.contains_point(&mid) {
                return false;
            }
        }

        true
    }

    /// Check if the travel path crosses any perimeter lines.
    fn travel_crosses_perimeters(&mut self, layer: &Layer, travel: &Polyline) -> bool {
        // Update perimeter cache if needed
        self.update_perimeter_lines_cache(layer);

        if self.perimeter_lines.is_empty() {
            return false;
        }

        // Get travel bounding box
        let travel_bbox = travel.bounding_box();

        // Quick rejection: check if travel bbox intersects perimeter bbox
        if !travel_bbox.intersects(&self.perimeter_lines_bbox) {
            return false;
        }

        // Check each travel segment against perimeter lines
        for travel_line in travel.edges() {
            let line_bbox = BoundingBox::from_points(&[travel_line.a, travel_line.b]);

            for perimeter_line in &self.perimeter_lines {
                // Quick bbox check
                let perimeter_bbox =
                    BoundingBox::from_points(&[perimeter_line.a, perimeter_line.b]);
                if !line_bbox.intersects(&perimeter_bbox) {
                    continue;
                }

                // Check for actual intersection
                if lines_intersect(&travel_line, perimeter_line) {
                    return true;
                }
            }
        }

        false
    }

    /// Update the internal islands cache for the given layer.
    fn update_internal_islands_cache(&mut self, layer: &Layer) {
        // Check if cache is still valid
        if self.cached_layer_id == Some(layer.id()) {
            return;
        }

        // Clear old cache
        self.internal_islands.clear();
        self.internal_islands_bboxes.clear();
        self.perimeters_cached = false;
        self.cached_layer_id = Some(layer.id());

        // Collect internal surfaces from all regions
        for region in layer.regions() {
            for surface in &region.fill_surfaces {
                if surface.surface_type.is_internal() && !surface.expolygon.is_empty() {
                    let bbox = surface.expolygon.bounding_box();
                    self.internal_islands.push(surface.expolygon.clone());
                    self.internal_islands_bboxes.push(bbox);
                }
            }
        }
    }

    /// Update the perimeter lines cache for the given layer.
    fn update_perimeter_lines_cache(&mut self, layer: &Layer) {
        // Ensure internal islands are cached first (this sets cached_layer_id)
        self.update_internal_islands_cache(layer);

        // Check if perimeters already cached
        if self.perimeters_cached {
            return;
        }

        self.perimeter_lines.clear();
        self.perimeter_lines_bbox = BoundingBox::new();

        // Collect lines from internal surfaces
        for island in &self.internal_islands {
            // Add contour edges
            let contour_edges = island.contour.edges();
            for edge in contour_edges {
                self.perimeter_lines_bbox.merge_point(edge.a);
                self.perimeter_lines_bbox.merge_point(edge.b);
                self.perimeter_lines.push(edge);
            }

            // Add hole edges
            for hole in &island.holes {
                let hole_edges = hole.edges();
                for edge in hole_edges {
                    self.perimeter_lines_bbox.merge_point(edge.a);
                    self.perimeter_lines_bbox.merge_point(edge.b);
                    self.perimeter_lines.push(edge);
                }
            }
        }

        // Also add perimeter lines from the layer regions
        // TODO: Implement edges() method for ExtrusionEntityType
        // This needs to recursively extract Line segments from Path/Loop/Collection variants
        // for region in layer.regions() {
        //     for perimeter in &region.perimeters {
        //         let edges = perimeter.edges();
        //         for edge in edges {
        //             self.perimeter_lines_bbox.merge_point(edge.a);
        //             self.perimeter_lines_bbox.merge_point(edge.b);
        //             self.perimeter_lines.push(edge);
        //         }
        //     }
        // }

        // Expand bbox slightly for numerical stability
        self.perimeter_lines_bbox.expand(scale(0.001)); // 1 micron

        self.perimeters_cached = true;
    }
}

/// Check if two line segments intersect.
///
/// Uses the cross product method to determine intersection.
fn lines_intersect(line1: &Line, line2: &Line) -> bool {
    let p1 = line1.a;
    let p2 = line1.b;
    let p3 = line2.a;
    let p4 = line2.b;

    // Direction vectors
    let d1x = p2.x - p1.x;
    let d1y = p2.y - p1.y;
    let d2x = p4.x - p3.x;
    let d2y = p4.y - p3.y;

    // Cross product of directions
    let cross = d1x as i128 * d2y as i128 - d1y as i128 * d2x as i128;

    // Parallel lines (including collinear)
    if cross == 0 {
        return false;
    }

    // Vector from p1 to p3
    let dx = p3.x - p1.x;
    let dy = p3.y - p1.y;

    // Parameters for intersection point
    let t_num = dx as i128 * d2y as i128 - dy as i128 * d2x as i128;
    let u_num = dx as i128 * d1y as i128 - dy as i128 * d1x as i128;

    // Check if intersection is within both segments [0, 1]
    // We need: 0 <= t_num/cross <= 1 and 0 <= u_num/cross <= 1
    // Accounting for sign of cross:
    if cross > 0 {
        t_num >= 0 && t_num <= cross && u_num >= 0 && u_num <= cross
    } else {
        t_num <= 0 && t_num >= cross && u_num <= 0 && u_num >= cross
    }
}

/// Configuration for retraction when crossing perimeters.
#[derive(Debug, Clone)]
pub struct RetractCrossingConfig {
    /// Whether this feature is enabled.
    pub enabled: bool,

    /// Whether to check for perimeter crossings (more strict).
    pub check_perimeter_crossings: bool,

    /// Minimum travel distance to consider for this check.
    /// Very short travels don't need this optimization.
    pub min_travel_distance: CoordF,
}

impl Default for RetractCrossingConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            check_perimeter_crossings: true,
            min_travel_distance: 1.0, // 1mm minimum
        }
    }
}

impl RetractCrossingConfig {
    // Create a new configuration with defaults.
    pub fn new() -> Self {
        Self::default()
    }

    /// Set whether the feature is enabled.
    pub fn with_enabled(mut self, enabled: bool) -> Self {
        self.enabled = enabled;
        self
    }

    /// Set whether to check perimeter crossings.
    pub fn with_perimeter_crossings(mut self, check: bool) -> Self {
        self.check_perimeter_crossings = check;
        self
    }

    /// Set minimum travel distance.
    pub fn with_min_travel_distance(mut self, distance: CoordF) -> Self {
        self.min_travel_distance = distance;
        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::{ExPolygon, Point, Polygon};
    use crate::slice::{Layer, LayerRegion, Surface, SurfaceType};
    use crate::Coord;

    fn make_square(x: Coord, y: Coord, size: Coord) -> Polygon {
        Polygon::rectangle(Point::new(x, y), Point::new(x + size, y + size))
    }

    fn make_layer_with_internal_surface() -> Layer {
        let mut layer = Layer::new_f(0, 0.0, 0.2, 0.1);
        let mut region = LayerRegion::new();

        // Add an internal surface
        let square = make_square(0, 0, scale(10.0));
        let expolygon = ExPolygon::new(square);
        let surface = Surface::new(SurfaceType::Internal, expolygon);
        region.fill_surfaces.push(surface);

        layer.add_region(region);
        layer
    }

    #[test]
    fn test_retract_decision() {
        assert!(RetractDecision::Retract.should_retract());
        assert!(!RetractDecision::NoRetract.should_retract());
    }

    #[test]
    fn test_new_instance() {
        let checker = RetractWhenCrossingPerimeters::new();
        assert!(checker.cached_layer_id.is_none());
        assert!(checker.internal_islands.is_empty());
        assert!(checker.perimeter_lines.is_empty());
    }

    #[test]
    fn test_empty_travel() {
        let mut checker = RetractWhenCrossingPerimeters::new();
        let layer = Layer::new_f(0, 0.0, 0.2, 0.1);
        let travel = Polyline::new();

        let result = checker.check_travel(&layer, &travel);
        assert_eq!(result, RetractDecision::NoRetract);
    }

    #[test]
    fn test_single_point_travel() {
        let mut checker = RetractWhenCrossingPerimeters::new();
        let layer = Layer::new_f(0, 0.0, 0.2, 0.1);
        let mut travel = Polyline::new();
        travel.push(Point::new(0, 0));

        let result = checker.check_travel(&layer, &travel);
        assert_eq!(result, RetractDecision::NoRetract);
    }

    #[test]
    fn test_travel_outside_internal_regions() {
        let mut checker = RetractWhenCrossingPerimeters::new();
        let layer = make_layer_with_internal_surface();

        // Travel completely outside the internal region
        let travel = Polyline::from_points(vec![
            Point::new(scale(20.0), scale(20.0)),
            Point::new(scale(30.0), scale(30.0)),
        ]);

        let result = checker.check_travel(&layer, &travel);
        assert_eq!(result, RetractDecision::Retract);
    }

    #[test]
    fn test_travel_inside_internal_region() {
        let mut checker = RetractWhenCrossingPerimeters::new();
        let layer = make_layer_with_internal_surface();

        // Travel completely inside the internal region (10mm square at origin)
        let travel = Polyline::from_points(vec![
            Point::new(scale(2.0), scale(2.0)),
            Point::new(scale(8.0), scale(8.0)),
        ]);

        let result = checker.check_travel(&layer, &travel);
        assert_eq!(result, RetractDecision::NoRetract);
    }

    #[test]
    fn test_travel_crossing_boundary() {
        let mut checker = RetractWhenCrossingPerimeters::new();
        let layer = make_layer_with_internal_surface();

        // Travel that starts inside and ends outside
        let travel = Polyline::from_points(vec![
            Point::new(scale(5.0), scale(5.0)),
            Point::new(scale(15.0), scale(15.0)),
        ]);

        let result = checker.check_travel(&layer, &travel);
        assert_eq!(result, RetractDecision::Retract);
    }

    #[test]
    fn test_layer_caching() {
        let mut checker = RetractWhenCrossingPerimeters::new();
        let layer0 = make_layer_with_internal_surface();
        let mut layer1 = Layer::new_f(1, 0.2, 0.4, 0.3);
        layer1.add_region(LayerRegion::new());

        let travel = Polyline::from_points(vec![
            Point::new(scale(2.0), scale(2.0)),
            Point::new(scale(3.0), scale(3.0)),
        ]);

        // Check first layer
        checker.check_travel(&layer0, &travel);
        assert_eq!(checker.cached_layer_id, Some(0));

        // Check second layer - cache should update
        checker.check_travel(&layer1, &travel);
        assert_eq!(checker.cached_layer_id, Some(1));
    }

    #[test]
    fn test_clear() {
        let mut checker = RetractWhenCrossingPerimeters::new();
        let layer = make_layer_with_internal_surface();
        let travel = Polyline::from_points(vec![
            Point::new(scale(2.0), scale(2.0)),
            Point::new(scale(3.0), scale(3.0)),
        ]);

        checker.check_travel(&layer, &travel);
        assert!(checker.cached_layer_id.is_some());

        checker.clear();
        assert!(checker.cached_layer_id.is_none());
        assert!(checker.internal_islands.is_empty());
    }

    #[test]
    fn test_lines_intersect_crossing() {
        let line1 = Line::new(Point::new(0, 0), Point::new(100, 100));
        let line2 = Line::new(Point::new(0, 100), Point::new(100, 0));

        assert!(lines_intersect(&line1, &line2));
    }

    #[test]
    fn test_lines_intersect_parallel() {
        let line1 = Line::new(Point::new(0, 0), Point::new(100, 0));
        let line2 = Line::new(Point::new(0, 10), Point::new(100, 10));

        assert!(!lines_intersect(&line1, &line2));
    }

    #[test]
    fn test_lines_intersect_no_intersection() {
        let line1 = Line::new(Point::new(0, 0), Point::new(50, 50));
        let line2 = Line::new(Point::new(60, 60), Point::new(100, 100));

        assert!(!lines_intersect(&line1, &line2));
    }

    #[test]
    fn test_lines_intersect_endpoint() {
        let line1 = Line::new(Point::new(0, 0), Point::new(50, 50));
        let line2 = Line::new(Point::new(50, 50), Point::new(100, 0));

        assert!(lines_intersect(&line1, &line2));
    }

    #[test]
    fn test_config_default() {
        let config = RetractCrossingConfig::default();
        assert!(config.enabled);
        assert!(config.check_perimeter_crossings);
        assert!((config.min_travel_distance - 1.0).abs() < 0.001);
    }

    #[test]
    fn test_config_builder() {
        let config = RetractCrossingConfig::new()
            .with_enabled(false)
            .with_perimeter_crossings(false)
            .with_min_travel_distance(2.5);

        assert!(!config.enabled);
        assert!(!config.check_perimeter_crossings);
        assert!((config.min_travel_distance - 2.5).abs() < 0.001);
    }

    #[test]
    fn test_empty_layer() {
        let mut checker = RetractWhenCrossingPerimeters::new();
        let layer = Layer::new_f(0, 0.0, 0.2, 0.1);

        let travel = Polyline::from_points(vec![
            Point::new(scale(0.0), scale(0.0)),
            Point::new(scale(10.0), scale(10.0)),
        ]);

        // With no internal regions, should retract
        let result = checker.check_travel(&layer, &travel);
        assert_eq!(result, RetractDecision::Retract);
    }

    #[test]
    fn test_multiple_internal_regions() {
        let mut layer = Layer::new_f(0, 0.0, 0.2, 0.1);
        let mut region = LayerRegion::new();

        // Add two separate internal regions
        let square1 = make_square(0, 0, scale(5.0));
        let square2 = make_square(scale(10.0), scale(10.0), scale(5.0));

        region
            .fill_surfaces
            .push(Surface::new(SurfaceType::Internal, ExPolygon::new(square1)));
        region
            .fill_surfaces
            .push(Surface::new(SurfaceType::Internal, ExPolygon::new(square2)));

        layer.add_region(region);

        let mut checker = RetractWhenCrossingPerimeters::new();

        // Travel inside first region - no retract
        let travel1 = Polyline::from_points(vec![
            Point::new(scale(1.0), scale(1.0)),
            Point::new(scale(4.0), scale(4.0)),
        ]);
        assert_eq!(
            checker.check_travel(&layer, &travel1),
            RetractDecision::NoRetract
        );

        // Travel inside second region - no retract
        let travel2 = Polyline::from_points(vec![
            Point::new(scale(11.0), scale(11.0)),
            Point::new(scale(14.0), scale(14.0)),
        ]);
        assert_eq!(
            checker.check_travel(&layer, &travel2),
            RetractDecision::NoRetract
        );

        // Travel between regions (outside both) - retract
        let travel3 = Polyline::from_points(vec![
            Point::new(scale(4.0), scale(4.0)),
            Point::new(scale(11.0), scale(11.0)),
        ]);
        assert_eq!(
            checker.check_travel(&layer, &travel3),
            RetractDecision::Retract
        );
    }
}
