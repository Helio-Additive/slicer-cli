//! 2D Honeycomb infill pattern.
//!
//! C++ Reference:
//! - Fill/FillHoneycomb.hpp
//! - Fill/FillHoneycomb.cpp
//!
//! Generates a 2D honeycomb (hexagonal) infill pattern. The pattern tiles
//! hexagons across the fill region with zigzag polylines that form
//! the honeycomb walls.

use crate::geometry::{BoundingBox, ExPolygon, Point, Polyline};
use crate::{Coord, CoordF};

/// Cached honeycomb geometry for a given density/spacing pair.
/// FillHoneycomb.hpp: CacheData
#[derive(Debug, Clone)]
pub struct CacheData {
    pub distance: Coord,
    pub hex_side: Coord,
    pub hex_width: Coord,
    pub pattern_height: Coord,
    pub y_short: Coord,
    pub x_offset: Coord,
    pub y_offset: Coord,
    pub hex_center: Point,
}

impl CacheData {
    /// Compute honeycomb geometry from spacing and density.
    /// FillHoneycomb.cpp:22-32
    pub fn new(spacing: CoordF, density: CoordF) -> Self {
        let min_spacing = (spacing * 1e6) as Coord; // scale to internal units
        let distance = if density > 0.0 {
            (min_spacing as f64 / density) as Coord
        } else {
            min_spacing * 10
        };
        let hex_side = (distance as f64 / (3.0_f64.sqrt() / 2.0)) as Coord;
        let hex_width = distance * 2;
        let hex_height = hex_side * 2;
        let pattern_height = hex_height + hex_side;
        let y_short = (distance as f64 * 3.0_f64.sqrt() / 3.0) as Coord;
        let x_offset = min_spacing / 2;
        let y_offset = (x_offset as f64 * 3.0_f64.sqrt() / 3.0) as Coord;
        let hex_center = Point::new(hex_width / 2, hex_side);

        Self {
            distance,
            hex_side,
            hex_width,
            pattern_height,
            y_short,
            x_offset,
            y_offset,
            hex_center,
        }
    }
}

/// Cache key for honeycomb geometry.
/// FillHoneycomb.hpp: CacheID
#[derive(Debug, Clone, PartialEq)]
pub struct CacheID {
    pub density: f32,
    pub spacing: CoordF,
}

impl CacheID {
    pub fn new(density: f32, spacing: CoordF) -> Self {
        Self { density, spacing }
    }
}

/// FillHoneycomb pattern generator.
/// FillHoneycomb.hpp
#[derive(Debug, Clone, Default)]
pub struct FillHoneycomb {
    /// Spacing between lines.
    pub spacing: CoordF,
    /// Fill density (0.0..1.0).
    pub density: CoordF,
}

impl FillHoneycomb {
    pub fn new(spacing: CoordF, density: CoordF) -> Self {
        Self { spacing, density }
    }

    /// Generate honeycomb infill polylines for a single expolygon.
    /// FillHoneycomb.cpp: _fill_surface_single
    pub fn fill_surface(&self, expolygon: &ExPolygon) -> Vec<Polyline> {
        if self.spacing <= 0.0 || self.density <= 0.0 {
            return Vec::new();
        }

        let m = CacheData::new(self.spacing, self.density);
        if m.distance <= 0 || m.hex_side <= 0 {
            return Vec::new();
        }

        // Compute bounding box of the contour
        let mut bb = BoundingBox::default();
        for pt in &expolygon.contour.points {
            bb.merge_point(*pt);
        }

        // Align bounding box to pattern grid
        if m.hex_width > 0 {
            bb.min.x = bb.min.x - (bb.min.x.rem_euclid(m.hex_width));
        }
        if m.pattern_height > 0 {
            bb.min.y = bb.min.y - (bb.min.y.rem_euclid(m.pattern_height));
        }

        let mut all_polylines = Vec::new();
        let mut x = bb.min.x;
        while x <= bb.max.x {
            let mut p = Polyline { points: Vec::new() };
            let mut ax = [x + m.x_offset, x + m.distance - m.x_offset];

            for i in 0..2usize {
                if i > 0 {
                    p.points.reverse();
                }
                let mut y = bb.min.y;
                while y <= bb.max.y {
                    p.points.push(Point::new(ax[1], y + m.y_offset));
                    p.points.push(Point::new(ax[0], y + m.y_short - m.y_offset));
                    p.points
                        .push(Point::new(ax[0], y + m.y_short + m.hex_side + m.y_offset));
                    p.points.push(Point::new(
                        ax[1],
                        y + m.y_short + m.hex_side + m.y_short - m.y_offset,
                    ));
                    p.points.push(Point::new(
                        ax[1],
                        y + m.y_short + m.hex_side + m.y_short + m.hex_side + m.y_offset,
                    ));
                    y += m.y_short + m.hex_side + m.y_short + m.hex_side;
                }
                ax[0] += m.distance;
                ax[1] += m.distance;
                ax.swap(0, 1);
                x += m.distance;
            }

            if !p.points.is_empty() {
                all_polylines.push(p);
            }
        }

        // TODO: Clip polylines to the expolygon boundary (intersection_pl)
        // For now return unclipped polylines; the caller clips to the fill region.
        all_polylines
    }
}
