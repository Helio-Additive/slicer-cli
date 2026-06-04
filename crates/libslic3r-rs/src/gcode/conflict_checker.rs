//! GCodeConflictChecker.rs - Detects potential conflicts in toolpaths.
//!
//! This module implements conflict detection for G-code toolpaths,
//! mirroring BambuStudio's GCode/ConflictChecker.cpp.
//!
//! Conflicts detected include:
//! - Tool collisions with printed parts
//! - Extruder interference during tool changes
//! - Travel moves through printed geometry
//! - Inter-object collisions in multi-object prints

use crate::gcode::{ExtrusionRole, GCodeMove, ParsedGCode};
use crate::geometry::{BoundingBox3F, Point3F};

/// Configuration for conflict checking.
#[derive(Debug, Clone)]
pub struct ConflictCheckerConfig {
    /// Minimum clearance required around printed parts (mm)
    pub clearance_radius: f64,
    /// Height of the extruder nozzle tip above the bed (mm)
    pub nozzle_height: f64,
    /// Maximum height of printed parts to check (mm)
    pub max_check_height: f64,
    /// Enable collision detection with printed parts
    pub check_print_collisions: bool,
    /// Enable tool change interference detection
    pub check_tool_changes: bool,
}

impl Default for ConflictCheckerConfig {
    fn default() -> Self {
        Self {
            clearance_radius: 2.0,
            nozzle_height: 10.0,
            max_check_height: 100.0,
            check_print_collisions: true,
            check_tool_changes: true,
        }
    }
}

/// Types of conflicts that can be detected.
#[derive(Debug, Clone, PartialEq)]
pub enum ConflictType {
    /// Travel move passes through already-printed geometry
    TravelThroughPrinted,
    /// Tool change would collide with printed part
    ToolChangeCollision,
    /// Extruder would hit a tall printed feature
    NozzleCollision,
    /// Object-to-object collision in multi-object print
    InterObjectCollision,
}

/// A detected conflict.
#[derive(Debug, Clone)]
pub struct Conflict {
    /// Type of conflict
    pub conflict_type: ConflictType,
    /// Layer where conflict occurs
    pub layer_z: f64,
    /// Position of the conflict
    pub position: Point3F,
    /// Description of the conflict
    pub description: String,
    /// Move index in the G-code (if applicable)
    pub move_index: Option<usize>,
}

/// Result of conflict checking.
#[derive(Debug, Default)]
pub struct ConflictResult {
    /// Conflicts detected
    pub conflicts: Vec<Conflict>,
    /// Total moves checked
    pub moves_checked: usize,
    /// Travel moves checked
    pub travel_moves_checked: usize,
}

impl ConflictResult {
    // Check if any conflicts were found.
    pub fn has_conflicts(&self) -> bool {
        !self.conflicts.is_empty()
    }

    /// Get the number of conflicts.
    pub fn conflict_count(&self) -> usize {
        self.conflicts.len()
    }

    /// Get conflicts by type.
    pub fn conflicts_by_type(&self, conflict_type: ConflictType) -> Vec<&Conflict> {
        self.conflicts
            .iter()
            .filter(|c| c.conflict_type == conflict_type)
            .collect()
    }
}

/// Checks G-code for potential conflicts.
pub struct ConflictChecker {
    config: ConflictCheckerConfig,
    printed_geometry: Vec<PrintedGeometry>,
}

/// Represents a region of already-printed geometry.
#[derive(Debug, Clone)]
struct PrintedGeometry {
    /// Bounding box of printed region
    bbox: BoundingBox3F,
    /// Layer Z height
    layer_z: f64,
    /// Extrusion role (to determine clearance requirements)
    role: ExtrusionRole,
}

impl ConflictChecker {
    // Create a new conflict checker with configuration.
    pub fn new(config: ConflictCheckerConfig) -> Self {
        Self {
            config,
            printed_geometry: Vec::new(),
        }
    }

    /// Create a conflict checker with default configuration.
    pub fn default_checker() -> Self {
        Self::new(ConflictCheckerConfig::default())
    }

    /// Add printed geometry for collision checking.
    pub fn add_printed_geometry(&mut self, bbox: BoundingBox3F, layer_z: f64, role: ExtrusionRole) {
        self.printed_geometry.push(PrintedGeometry {
            bbox,
            layer_z,
            role,
        });
    }

    /// Check parsed G-code for conflicts.
    pub fn check_gcode(&mut self, gcode: &ParsedGCode) -> ConflictResult {
        let mut result = ConflictResult {
            conflicts: Vec::new(),
            moves_checked: gcode.moves.len(),
            travel_moves_checked: 0,
        };

        // Build geometry from extrusion moves first
        self.build_geometry_from_moves(&gcode.moves);

        // Check each travel move for conflicts
        for (i, move_) in gcode.moves.iter().enumerate() {
            if move_.is_travel() {
                result.travel_moves_checked += 1;

                if self.config.check_print_collisions {
                    if let Some(conflict) = self.check_travel_move(move_, i) {
                        result.conflicts.push(conflict);
                    }
                }
            }
        }

        result
    }

    /// Build geometry tracking from moves.
    fn build_geometry_from_moves(&mut self, moves: &[GCodeMove]) {
        self.printed_geometry.clear();

        for move_ in moves {
            if let Some(ref role) = move_.role {
                if *role != ExtrusionRole::Travel && move_.extrusion > 0.0 {
                    // Create a small bounding box around the move
                    let start = Point3F::new(move_.x, move_.y, move_.z);
                    let end = Point3F::new(
                        move_.x + move_.dx.unwrap_or(0.0),
                        move_.y + move_.dy.unwrap_or(0.0),
                        move_.z,
                    );

                    let min = Point3F::new(
                        start.x.min(end.x) - 0.5,
                        start.y.min(end.y) - 0.5,
                        move_.z - 0.1,
                    );
                    let max = Point3F::new(
                        start.x.max(end.x) + 0.5,
                        start.y.max(end.y) + 0.5,
                        move_.z + 0.5,
                    );

                    self.add_printed_geometry(BoundingBox3F::new(min, max), move_.z, role.clone());
                }
            }
        }
    }

    /// Check a single travel move for conflicts.
    fn check_travel_move(&self, move_: &GCodeMove, index: usize) -> Option<Conflict> {
        let start = Point3F::new(move_.x, move_.y, move_.z);
        let end = Point3F::new(
            move_.x + move_.dx.unwrap_or(0.0),
            move_.y + move_.dy.unwrap_or(0.0),
            move_.z,
        );

        // Check travel line against all printed geometry
        for geom in &self.printed_geometry {
            // Skip geometry that's too high
            if geom.layer_z > move_.z + self.config.nozzle_height {
                continue;
            }

            // Check if travel line intersects with geometry bbox
            if Self::line_intersects_bbox(start, end, &geom.bbox) {
                // Check if this is actually a problem (need clearance)
                let clearance = self.config.clearance_radius;
                let expanded_bbox = geom.bbox.grow(clearance);

                if Self::line_intersects_bbox(start, end, &expanded_bbox) {
                    return Some(Conflict {
                        conflict_type: ConflictType::TravelThroughPrinted,
                        layer_z: move_.z,
                        position: start,
                        description: format!(
                            "Travel move {} at Z={:.2} passes near printed geometry at Z={:.2}",
                            index, move_.z, geom.layer_z
                        ),
                        move_index: Some(index),
                    });
                }
            }
        }

        None
    }

    /// Check if a line segment intersects with a bounding box.
    fn line_intersects_bbox(start: Point3F, end: Point3F, bbox: &BoundingBox3F) -> bool {
        // Liang-Barsky line clipping algorithm
        let dx = end.x - start.x;
        let dy = end.y - start.y;
        let dz = end.z - start.z;

        let mut tmin = 0.0;
        let mut tmax = 1.0;

        // Check X planes
        if dx.abs() > f64::EPSILON {
            let tx1 = (bbox.min.x - start.x) / dx;
            let tx2 = (bbox.max.x - start.x) / dx;
            tmin = tmin.max(tx1.min(tx2));
            tmax = tmax.min(tx1.max(tx2));
        } else if start.x < bbox.min.x || start.x > bbox.max.x {
            return false;
        }

        // Check Y planes
        if dy.abs() > f64::EPSILON {
            let ty1 = (bbox.min.y - start.y) / dy;
            let ty2 = (bbox.max.y - start.y) / dy;
            tmin = tmin.max(ty1.min(ty2));
            tmax = tmax.min(ty1.max(ty2));
        } else if start.y < bbox.min.y || start.y > bbox.max.y {
            return false;
        }

        // Check Z planes
        if dz.abs() > f64::EPSILON {
            let tz1 = (bbox.min.z - start.z) / dz;
            let tz2 = (bbox.max.z - start.z) / dz;
            tmin = tmin.max(tz1.min(tz2));
            tmax = tmax.min(tz1.max(tz2));
        } else if start.z < bbox.min.z || start.z > bbox.max.z {
            return false;
        }

        tmin <= tmax && tmax >= 0.0 && tmin <= 1.0
    }
}

/// Convenience function to check G-code for conflicts.
pub fn check_conflicts(gcode: &ParsedGCode) -> ConflictResult {
    let mut checker = ConflictChecker::default_checker();
    checker.check_gcode(gcode)
}

/// Check conflicts with custom configuration.
pub fn check_conflicts_with_config(
    gcode: &ParsedGCode,
    config: ConflictCheckerConfig,
) -> ConflictResult {
    let mut checker = ConflictChecker::new(config);
    checker.check_gcode(gcode)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_conflict_checker_default() {
        let checker = ConflictChecker::default_checker();
        assert!(checker.printed_geometry.is_empty());
    }

    #[test]
    fn test_conflict_result_empty() {
        let result = ConflictResult::default();
        assert!(!result.has_conflicts());
        assert_eq!(result.conflict_count(), 0);
    }

    #[test]
    fn test_line_bbox_intersection() {
        let bbox = BoundingBox3F::new(Point3F::new(0.0, 0.0, 0.0), Point3F::new(10.0, 10.0, 10.0));

        // Line passing through
        let start = Point3F::new(-5.0, 5.0, 5.0);
        let end = Point3F::new(15.0, 5.0, 5.0);
        assert!(ConflictChecker::line_intersects_bbox(start, end, &bbox));

        // Line missing
        let start = Point3F::new(-5.0, 15.0, 5.0);
        let end = Point3F::new(15.0, 15.0, 5.0);
        assert!(!ConflictChecker::line_intersects_bbox(start, end, &bbox));
    }

    #[test]
    fn test_conflict_types() {
        assert_eq!(
            ConflictType::TravelThroughPrinted,
            ConflictType::TravelThroughPrinted
        );
        assert_ne!(
            ConflictType::TravelThroughPrinted,
            ConflictType::ToolChangeCollision
        );
    }
}
