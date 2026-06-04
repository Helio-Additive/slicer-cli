//! GCodePrintExtents.rs - Computes print extents for visualization.
//!
//! This module calculates the spatial extents of G-code toolpaths,
//! mirroring BambuStudio's GCode/PrintExtents.cpp.
//!
//! Used for:
//! - Bed visualization
//! - Print preview bounds
//! - Collision detection
//! - Camera framing

use crate::gcode::{ExtrusionRole, GCodeMove, ParsedGCode};
use crate::geometry::{BoundingBox3F, Point3F};

/// Print extent information.
#[derive(Debug, Clone, Default)]
pub struct PrintExtents {
    /// Overall bounding box
    pub bbox: BoundingBox3F,
    /// Extents by extrusion role
    pub extents_by_role: Vec<(ExtrusionRole, BoundingBox3F)>,
    /// Layer count
    pub layer_count: usize,
    /// Min and max Z
    pub z_range: (f64, f64),
    /// Estimated print time (seconds)
    pub estimated_time: f64,
}

impl PrintExtents {
    // Create new empty extents.
    pub fn new() -> Self {
        Self {
            bbox: BoundingBox3F::empty(),
            extents_by_role: Vec::new(),
            layer_count: 0,
            z_range: (f64::INFINITY, f64::NEG_INFINITY),
            estimated_time: 0.0,
        }
    }

    /// Check if extents are valid (non-empty).
    pub fn is_valid(&self) -> bool {
        self.bbox.is_valid()
    }

    /// Get print width (X dimension).
    pub fn width(&self) -> f64 {
        self.bbox.size_x()
    }

    /// Get print depth (Y dimension).
    pub fn depth(&self) -> f64 {
        self.bbox.size_y()
    }

    /// Get print height (Z dimension).
    pub fn height(&self) -> f64 {
        self.bbox.size_z()
    }

    /// Get center point of the print.
    pub fn center(&self) -> Point3F {
        self.bbox.center()
    }

    /// Get extent for a specific role.
    pub fn extent_for_role(&self, role: ExtrusionRole) -> Option<&BoundingBox3F> {
        self.extents_by_role
            .iter()
            .find(|(r, _)| *r == role)
            .map(|(_, bbox)| bbox)
    }
}

/// Calculates print extents from parsed G-code.
pub struct PrintExtentsCalculator;

impl PrintExtentsCalculator {
    // Calculate extents from G-code.
    pub fn calculate(gcode: &ParsedGCode) -> PrintExtents {
        let mut extents = PrintExtents::new();
        let mut role_extents: std::collections::HashMap<ExtrusionRole, BoundingBox3F> =
            std::collections::HashMap::new();

        let mut current_z = 0.0_f64;
        let mut current_feedrate = 0.0_f64;
        let mut total_time = 0.0_f64;

        for move_ in &gcode.moves {
            let start = Point3F::new(move_.x, move_.y, move_.z);
            let end = Point3F::new(
                move_.x + move_.dx.unwrap_or(0.0),
                move_.y + move_.dy.unwrap_or(0.0),
                move_.z,
            );

            // Update Z range
            extents.z_range.0 = extents.z_range.0.min(move_.z);
            extents.z_range.1 = extents.z_range.1.max(move_.z);
            current_z = move_.z;

            // Update overall bbox
            extents.bbox.merge_point(&start);
            extents.bbox.merge_point(&end);

            // Track by role
            if let Some(ref role) = move_.role {
                let entry = role_extents.entry(role.clone()).or_default();
                entry.merge_point(&start);
                entry.merge_point(&end);
            }

            // Estimate time based on feedrate
            if move_.feedrate > 0.0 {
                current_feedrate = move_.feedrate;
            }

            if current_feedrate > 0.0 {
                let distance = move_.dx.unwrap_or(0.0).hypot(move_.dy.unwrap_or(0.0));
                let time = distance / current_feedrate * 60.0; // convert mm/min to mm/s
                total_time += time;
            }
        }

        // Count unique Z values as layers
        let mut z_values: Vec<f64> = gcode
            .moves
            .iter()
            .map(|m| m.z)
            .collect::<std::collections::HashSet<_>>()
            .into_iter()
            .collect();
        z_values.sort_by(|a, b| a.partial_cmp(b).unwrap());
        extents.layer_count = z_values.len();

        // Convert role extents to vec
        extents.extents_by_role = role_extents
            .into_iter()
            .map(|(role, bbox)| (role, bbox))
            .collect();

        extents.estimated_time = total_time;

        extents
    }

    /// Calculate extents for a subset of moves.
    pub fn calculate_for_moves(moves: &[GCodeMove]) -> PrintExtents {
        let mut extents = PrintExtents::new();
        let mut role_extents: std::collections::HashMap<ExtrusionRole, BoundingBox3F> =
            std::collections::HashMap::new();

        for move_ in moves {
            let start = Point3F::new(move_.x, move_.y, move_.z);
            let end = Point3F::new(
                move_.x + move_.dx.unwrap_or(0.0),
                move_.y + move_.dy.unwrap_or(0.0),
                move_.z,
            );

            extents.z_range.0 = extents.z_range.0.min(move_.z);
            extents.z_range.1 = extents.z_range.1.max(move_.z);

            extents.bbox.merge_point(&start);
            extents.bbox.merge_point(&end);

            if let Some(ref role) = move_.role {
                let entry = role_extents.entry(role.clone()).or_default();
                entry.merge_point(&start);
                entry.merge_point(&end);
            }
        }

        extents.extents_by_role = role_extents
            .into_iter()
            .map(|(role, bbox)| (role, bbox))
            .collect();

        extents
    }
}

/// Convenience function to calculate print extents.
pub fn calculate_extents(gcode: &ParsedGCode) -> PrintExtents {
    PrintExtentsCalculator::calculate(gcode)
}

/// Calculate extents for specific roles only.
pub fn calculate_extents_for_roles(gcode: &ParsedGCode, roles: &[ExtrusionRole]) -> PrintExtents {
    let filtered_moves: Vec<_> = gcode
        .moves
        .iter()
        .filter(|m| m.role.as_ref().map(|r| roles.contains(r)).unwrap_or(false))
        .cloned()
        .collect();

    PrintExtentsCalculator::calculate_for_moves(&filtered_moves)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_print_extents_new() {
        let extents = PrintExtents::new();
        assert!(!extents.is_valid());
        assert_eq!(extents.layer_count, 0);
    }

    #[test]
    fn test_calculate_empty_gcode() {
        let gcode = ParsedGCode::default();
        let extents = calculate_extents(&gcode);
        assert!(!extents.is_valid());
    }

    #[test]
    fn test_extents_dimensions() {
        let mut extents = PrintExtents::new();
        extents.bbox =
            BoundingBox3F::new(Point3F::new(0.0, 0.0, 0.0), Point3F::new(100.0, 50.0, 20.0));

        assert!(extents.is_valid());
        assert_eq!(extents.width(), 100.0);
        assert_eq!(extents.depth(), 50.0);
        assert_eq!(extents.height(), 20.0);
    }
}
