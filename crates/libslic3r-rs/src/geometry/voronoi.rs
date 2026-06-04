//! Voronoi diagram utilities.
//!
//! Voronoi diagrams partition space into regions based on distance to
//! a set of seed points. Each region contains all points closer to its
//! seed than to any other.
//!
//! Mirroring BambuStudio's Geometry/Voronoi*.cpp

use crate::geometry::{BoundingBox, Line, Point, PointF, Polygon};
use crate::CoordF;
use std::collections::HashMap;

/// A cell in the Voronoi diagram.
#[derive(Debug, Clone)]
pub struct VoronoiCell {
    /// Index of the seed point
    pub site_index: usize,
    /// Vertices of the cell polygon
    pub vertices: Vec<PointF>,
}

/// Voronoi diagram generator.
#[derive(Debug, Clone)]
pub struct VoronoiDiagram {
    /// The seed points
    pub sites: Vec<PointF>,
    /// The Voronoi cells
    pub cells: Vec<VoronoiCell>,
    /// Bounding box of the diagram
    pub bbox: BoundingBox,
}

impl VoronoiDiagram {
    // Create an empty Voronoi diagram.
    pub fn new() -> Self {
        Self {
            sites: Vec::new(),
            cells: Vec::new(),
            bbox: BoundingBox::empty(),
        }
    }

    /// Generate a Voronoi diagram from a set of points.
    ///
    /// Uses a simple brute-force approach for demonstration.
    /// For production, consider using a proper Voronoi library.
    pub fn from_points(points: &[PointF]) -> Self {
        if points.len() < 2 {
            return Self::new();
        }

        let mut diagram = Self::new();
        diagram.sites = points.to_vec();

        // Compute bounding box
        for p in points {
            diagram.bbox.merge_point(Point::new(
                (p.x * 1_000_000.0) as i64,
                (p.y * 1_000_000.0) as i64,
            ));
        }

        // Grow bbox slightly
        diagram
            .bbox
            .grow((diagram.bbox.size_x().max(diagram.bbox.size_y()) / 10) as i64);

        // Generate cells using brute-force sampling
        // For a proper implementation, use Fortune's algorithm
        let cell_count = points.len();
        for i in 0..cell_count {
            let cell = Self::compute_cell_brute_force(i, points, &diagram.bbox);
            diagram.cells.push(cell);
        }

        diagram
    }

    /// Compute a single Voronoi cell using brute-force sampling.
    fn compute_cell_brute_force(
        site_idx: usize,
        sites: &[PointF],
        bbox: &BoundingBox,
    ) -> VoronoiCell {
        let site = sites[site_idx];

        // Sample points and find those closest to this site
        let sample_count = 50;
        let min_x = bbox.min.x as CoordF / 1_000_000.0;
        let max_x = bbox.max.x as CoordF / 1_000_000.0;
        let min_y = bbox.min.y as CoordF / 1_000_000.0;
        let max_y = bbox.max.y as CoordF / 1_000_000.0;

        let mut cell_points: Vec<PointF> = Vec::new();

        for i in 0..=sample_count {
            for j in 0..=sample_count {
                let x = min_x + (max_x - min_x) * i as CoordF / sample_count as CoordF;
                let y = min_y + (max_y - min_y) * j as CoordF / sample_count as CoordF;
                let p = PointF::new(x, y);

                // Check if this point is closest to our site
                let dist_to_site = ((p.x - site.x).powi(2) + (p.y - site.y).powi(2)).sqrt();

                let mut is_closest = true;
                for (k, other_site) in sites.iter().enumerate() {
                    if k == site_idx {
                        continue;
                    }
                    let dist_to_other =
                        ((p.x - other_site.x).powi(2) + (p.y - other_site.y).powi(2)).sqrt();
                    if dist_to_other < dist_to_site {
                        is_closest = false;
                        break;
                    }
                }

                if is_closest {
                    // Check if this is on the boundary (rough approximation)
                    let is_boundary = i == 0 || i == sample_count || j == 0 || j == sample_count;
                    if is_boundary || cell_points.is_empty() {
                        cell_points.push(p);
                    }
                }
            }
        }

        // Sort points to form a polygon
        cell_points = Self::sort_points_ccw(cell_points, site);

        VoronoiCell {
            site_index: site_idx,
            vertices: cell_points,
        }
    }

    /// Sort points counter-clockwise around a center point.
    fn sort_points_ccw(points: Vec<PointF>, center: PointF) -> Vec<PointF> {
        let mut points: Vec<(PointF, CoordF)> = points
            .into_iter()
            .map(|p| {
                let angle = (p.y - center.y).atan2(p.x - center.x);
                (p, angle)
            })
            .collect();

        points.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap());
        points.into_iter().map(|(p, _)| p).collect()
    }

    /// Get the cell for a specific site.
    pub fn cell_for_site(&self, site_idx: usize) -> Option<&VoronoiCell> {
        self.cells.iter().find(|c| c.site_index == site_idx)
    }

    /// Check if the diagram is empty.
    pub fn is_empty(&self) -> bool {
        self.cells.is_empty()
    }

    /// Get the number of cells.
    pub fn cell_count(&self) -> usize {
        self.cells.len()
    }
}

impl Default for VoronoiDiagram {
    fn default() -> Self {
        Self::new()
    }
}

/// Find the closest site index for a given point.
pub fn find_closest_site(point: PointF, sites: &[PointF]) -> Option<usize> {
    if sites.is_empty() {
        return None;
    }

    let mut closest_idx = 0;
    let mut min_dist = CoordF::INFINITY;

    for (i, site) in sites.iter().enumerate() {
        let dist = ((point.x - site.x).powi(2) + (point.y - site.y).powi(2)).sqrt();
        if dist < min_dist {
            min_dist = dist;
            closest_idx = i;
        }
    }

    Some(closest_idx)
}

/// Compute the distance between two sites.
pub fn site_distance(a: PointF, b: PointF) -> CoordF {
    ((a.x - b.x).powi(2) + (a.y - b.y).powi(2)).sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_voronoi_empty() {
        let diagram = VoronoiDiagram::new();
        assert!(diagram.is_empty());
    }

    #[test]
    fn test_voronoi_two_points() {
        let points = vec![PointF::new(0.0, 0.0), PointF::new(10.0, 0.0)];
        let diagram = VoronoiDiagram::from_points(&points);
        assert_eq!(diagram.cell_count(), 2);
    }

    #[test]
    fn test_find_closest_site() {
        let sites = vec![
            PointF::new(0.0, 0.0),
            PointF::new(10.0, 0.0),
            PointF::new(5.0, 10.0),
        ];

        assert_eq!(find_closest_site(PointF::new(1.0, 1.0), &sites), Some(0));
        assert_eq!(find_closest_site(PointF::new(9.0, 1.0), &sites), Some(1));
        assert_eq!(find_closest_site(PointF::new(5.0, 9.0), &sites), Some(2));
    }

    #[test]
    fn test_site_distance() {
        let a = PointF::new(0.0, 0.0);
        let b = PointF::new(3.0, 4.0);
        assert!((site_distance(a, b) - 5.0).abs() < 0.001);
    }
}
