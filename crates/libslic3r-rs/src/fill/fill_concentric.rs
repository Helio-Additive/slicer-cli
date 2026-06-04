//! Concentric infill pattern.
//!
//! C++ Reference:
//! - Fill/FillConcentric.hpp
//! - Fill/FillConcentric.cpp
//!
//! Concentric infill creates concentric offset loops of the fill region boundary,
//! shrinking inward by the line spacing each time. This produces rings of material
//! that follow the part outline, providing good top/bottom surface quality.

use crate::clipper_utils::{offset_expolygon, OffsetJoinType};
use crate::geometry::{ExPolygon, Polyline};
use crate::{Coord, CoordF};

/// Configuration for concentric fill.
/// FillConcentric.hpp
#[derive(Debug, Clone)]
pub struct FillConcentric {
    /// Spacing between concentric loops (scaled units).
    pub spacing: Coord,
    /// Minimum loop length to keep (scaled units).
    pub min_loop_length: Coord,
}

impl FillConcentric {
    /// Create a new FillConcentric with given spacing.
    pub fn new(spacing: Coord) -> Self {
        Self {
            spacing,
            min_loop_length: 0,
        }
    }
}

impl Default for FillConcentric {
    fn default() -> Self {
        Self {
            spacing: 0,
            min_loop_length: 0,
        }
    }
}

/// Generate concentric infill polylines for a set of expolygons.
///
/// Repeatedly offsets the boundary inward by `spacing` until no area remains,
/// converting each offset contour into a polyline.
///
/// FillConcentric.cpp: fill_surface_by_lines equivalent
pub fn generate_concentric_infill(fill_area: &[ExPolygon], spacing: CoordF) -> Vec<Polyline> {
    let mut result = Vec::new();

    for expoly in fill_area {
        let mut current = vec![expoly.clone()];
        loop {
            // Offset inward (negative offset)
            let mut next = Vec::new();
            for ep in &current {
                let shrunk = offset_expolygon(ep, -spacing, OffsetJoinType::Miter);
                next.extend(shrunk);
            }
            if next.is_empty() {
                break;
            }
            // Convert each contour to a polyline (closed loop)
            for ep in &next {
                if ep.contour.points.len() >= 3 {
                    let mut pts = ep.contour.points.clone();
                    // Close the loop by appending the first point
                    pts.push(pts[0]);
                    result.push(Polyline { points: pts });
                }
                // Also add hole contours as separate polylines
                for hole in &ep.holes {
                    if hole.points.len() >= 3 {
                        let mut pts = hole.points.clone();
                        pts.push(pts[0]);
                        result.push(Polyline { points: pts });
                    }
                }
            }
            current = next;
        }
    }

    result
}
