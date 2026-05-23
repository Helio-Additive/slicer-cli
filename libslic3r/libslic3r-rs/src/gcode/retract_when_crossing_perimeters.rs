//! Smart retraction when crossing perimeters.
//!
//! C++ Reference:
//! - GCode/RetractWhenCrossingPerimeters.hpp
//! - GCode/RetractWhenCrossingPerimeters.cpp
//!
//! This module decides whether to retract when the travel path crosses
//! perimeter walls, using AABB tree searches over internal island boundaries.

use crate::geometry::{ExPolygon, Point, Polyline};

/// Retract-when-crossing-perimeters logic.
/// Decides whether a travel move needs retraction based on whether it crosses
/// any perimeter boundaries.
///
/// Corresponds to C++ RetractWhenCrossingPerimeters.
#[derive(Debug, Clone)]
pub struct RetractWhenCrossingPerimeters {
    /// Cached layer index for invalidation.
    cached_layer_index: Option<usize>,
    /// Bounding box of internal islands (min_x, min_y, max_x, max_y).
    internal_islands_bbox: Option<(i64, i64, i64, i64)>,
    /// Lines forming the internal island boundaries.
    internal_islands_lines: Vec<(Point, Point)>,
    /// Whether cross-perimeters flag was detected.
    cross_perimeters_flag: bool,
    /// Cached internal island polygons.
    internal_islands: Vec<ExPolygon>,
}

impl RetractWhenCrossingPerimeters {
    pub fn new() -> Self {
        Self {
            cached_layer_index: None,
            internal_islands_bbox: None,
            internal_islands_lines: Vec::new(),
            cross_perimeters_flag: false,
            internal_islands: Vec::new(),
        }
    }

    /// Check if a travel move stays inside internal regions without crossing walls.
    ///
    /// This is the main entry point. Returns true if the travel path does NOT cross
    /// any external perimeters and stays within internal fill regions.
    ///
    /// If true, retraction can be skipped (the nozzle stays "inside").
    pub fn travel_inside_internal_regions_no_wall_crossing(
        &mut self,
        layer_index: usize,
        internal_islands: &[ExPolygon],
        travel: &Polyline,
    ) -> bool {
        if travel.points().len() < 2 {
            return true;
        }

        // Rebuild cache if layer changed
        if self.cached_layer_index != Some(layer_index) {
            self.rebuild_cache(layer_index, internal_islands);
        }

        // If no internal islands, always retract
        if self.internal_islands.is_empty() {
            return false;
        }

        // Check if the travel crosses any perimeter
        if self.travel_cross_perimeters(travel) {
            return false;
        }

        // Check if the travel stays inside internal regions
        self.travel_inside_internal_regions(travel)
    }

    /// Check if the travel path crosses any perimeter boundary.
    fn travel_cross_perimeters(&self, travel: &Polyline) -> bool {
        let points = travel.points();
        if points.len() < 2 {
            return false;
        }

        // Check each travel segment against cached boundary lines
        for i in 0..points.len() - 1 {
            let p1 = &points[i];
            let p2 = &points[i + 1];

            // Quick AABB rejection
            if let Some((min_x, min_y, max_x, max_y)) = self.internal_islands_bbox {
                let seg_min_x = p1.x.min(p2.x);
                let seg_max_x = p1.x.max(p2.x);
                let seg_min_y = p1.y.min(p2.y);
                let seg_max_y = p1.y.max(p2.y);

                if seg_max_x < min_x || seg_min_x > max_x || seg_max_y < min_y || seg_min_y > max_y
                {
                    continue;
                }
            }

            for (a, b) in &self.internal_islands_lines {
                if segments_intersect(p1, p2, a, b) {
                    return true;
                }
            }
        }

        false
    }

    /// Check if the entire travel path stays inside internal regions.
    fn travel_inside_internal_regions(&self, travel: &Polyline) -> bool {
        let points = travel.points();
        // Check that start and end points are inside some internal island
        if points.is_empty() {
            return true;
        }

        let start = &points[0];
        let end = &points[points.len() - 1];

        let start_inside = self
            .internal_islands
            .iter()
            .any(|island| point_inside_expolygon(island, start));
        let end_inside = self
            .internal_islands
            .iter()
            .any(|island| point_inside_expolygon(island, end));

        start_inside && end_inside
    }

    /// Rebuild the internal boundary cache for a new layer.
    fn rebuild_cache(&mut self, layer_index: usize, internal_islands: &[ExPolygon]) {
        self.cached_layer_index = Some(layer_index);
        self.internal_islands = internal_islands.to_vec();
        self.internal_islands_lines.clear();
        self.cross_perimeters_flag = false;

        if internal_islands.is_empty() {
            self.internal_islands_bbox = None;
            return;
        }

        let mut min_x = i64::MAX;
        let mut min_y = i64::MAX;
        let mut max_x = i64::MIN;
        let mut max_y = i64::MIN;

        for island in internal_islands {
            let contour_pts = island.contour.points();
            for i in 0..contour_pts.len() {
                let j = (i + 1) % contour_pts.len();
                let p1 = contour_pts[i];
                let p2 = contour_pts[j];

                min_x = min_x.min(p1.x).min(p2.x);
                min_y = min_y.min(p1.y).min(p2.y);
                max_x = max_x.max(p1.x).max(p2.x);
                max_y = max_y.max(p1.y).max(p2.y);

                self.internal_islands_lines.push((p1, p2));
            }

            for hole in &island.holes {
                let hole_pts = hole.points();
                for i in 0..hole_pts.len() {
                    let j = (i + 1) % hole_pts.len();
                    self.internal_islands_lines.push((hole_pts[i], hole_pts[j]));
                }
            }
        }

        self.internal_islands_bbox = Some((min_x, min_y, max_x, max_y));
    }
}

impl Default for RetractWhenCrossingPerimeters {
    fn default() -> Self {
        Self::new()
    }
}

/// Check if two line segments intersect using cross products.
fn segments_intersect(p1: &Point, p2: &Point, p3: &Point, p4: &Point) -> bool {
    let d1 = cross(p3, p4, p1);
    let d2 = cross(p3, p4, p2);
    let d3 = cross(p1, p2, p3);
    let d4 = cross(p1, p2, p4);

    if ((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) && ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0)) {
        return true;
    }

    // Collinear cases
    if d1 == 0 && on_segment(p3, p4, p1) {
        return true;
    }
    if d2 == 0 && on_segment(p3, p4, p2) {
        return true;
    }
    if d3 == 0 && on_segment(p1, p2, p3) {
        return true;
    }
    if d4 == 0 && on_segment(p1, p2, p4) {
        return true;
    }

    false
}

/// Cross product of vectors (b-a) x (c-a) using i128 to prevent overflow.
fn cross(a: &Point, b: &Point, c: &Point) -> i128 {
    let abx = b.x as i128 - a.x as i128;
    let aby = b.y as i128 - a.y as i128;
    let acx = c.x as i128 - a.x as i128;
    let acy = c.y as i128 - a.y as i128;
    abx * acy - aby * acx
}

/// Check if point c is on segment [a, b] (assuming collinear).
fn on_segment(a: &Point, b: &Point, c: &Point) -> bool {
    c.x >= a.x.min(b.x) && c.x <= a.x.max(b.x) && c.y >= a.y.min(b.y) && c.y <= a.y.max(b.y)
}

/// Simple point-in-polygon test using ray casting.
fn point_inside_expolygon(expoly: &ExPolygon, point: &Point) -> bool {
    if !point_inside_polygon_points(expoly.contour.points(), point) {
        return false;
    }
    // Check that point is not inside any hole
    for hole in &expoly.holes {
        if point_inside_polygon_points(hole.points(), point) {
            return false;
        }
    }
    true
}

/// Ray-casting point-in-polygon test.
fn point_inside_polygon_points(points: &[Point], test: &Point) -> bool {
    if points.len() < 3 {
        return false;
    }

    let mut inside = false;
    let n = points.len();
    let mut j = n - 1;

    for i in 0..n {
        let pi = &points[i];
        let pj = &points[j];

        if ((pi.y > test.y) != (pj.y > test.y))
            && (test.x as i128)
                < (pj.x as i128 - pi.x as i128) * (test.y as i128 - pi.y as i128)
                    / (pj.y as i128 - pi.y as i128)
                    + pi.x as i128
        {
            inside = !inside;
        }
        j = i;
    }

    inside
}

/// Check if travel stays inside internal regions without crossing walls (standalone function).
pub fn travel_inside_internal_regions_no_wall_crossing(
    checker: &mut RetractWhenCrossingPerimeters,
    layer_index: usize,
    internal_islands: &[ExPolygon],
    travel: &Polyline,
) -> bool {
    checker.travel_inside_internal_regions_no_wall_crossing(layer_index, internal_islands, travel)
}

/// Check if travel path crosses perimeter boundaries (standalone function).
pub fn travel_cross_perimeters(checker: &RetractWhenCrossingPerimeters, travel: &Polyline) -> bool {
    checker.travel_cross_perimeters(travel)
}

/// Check if travel stays inside internal regions (standalone function).
pub fn travel_inside_internal_regions(
    checker: &RetractWhenCrossingPerimeters,
    travel: &Polyline,
) -> bool {
    checker.travel_inside_internal_regions(travel)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::Polygon;

    #[test]
    fn test_new() {
        let checker = RetractWhenCrossingPerimeters::new();
        assert!(checker.cached_layer_index.is_none());
        assert!(checker.internal_islands.is_empty());
    }

    #[test]
    fn test_segments_intersect() {
        let p1 = Point::new(0, 0);
        let p2 = Point::new(10, 10);
        let p3 = Point::new(0, 10);
        let p4 = Point::new(10, 0);
        assert!(segments_intersect(&p1, &p2, &p3, &p4));

        let p5 = Point::new(20, 20);
        let p6 = Point::new(30, 30);
        assert!(!segments_intersect(&p1, &p2, &p5, &p6));
    }

    #[test]
    fn test_point_inside_polygon() {
        let points = vec![
            Point::new(0, 0),
            Point::new(100, 0),
            Point::new(100, 100),
            Point::new(0, 100),
        ];
        assert!(point_inside_polygon_points(&points, &Point::new(50, 50)));
        assert!(!point_inside_polygon_points(&points, &Point::new(150, 50)));
    }

    #[test]
    fn test_empty_travel() {
        let mut checker = RetractWhenCrossingPerimeters::new();
        let travel = Polyline::from_points(vec![]);
        assert!(checker.travel_inside_internal_regions_no_wall_crossing(0, &[], &travel));
    }
}
