//! Auto-arrange algorithm for placing objects on the print bed.
//!
//! Implements algorithms for automatically arranging multiple 3D objects
//! on the print bed to maximize space utilization and minimize print time.
//! Based on BambuStudio's Arrange.cpp/hpp.

use crate::geometry::{BoundingBox, Point};
use crate::model::{Instance, Model, ModelObject};
use crate::Coord;

/// Configuration for auto-arrangement.
#[derive(Debug, Clone)]
pub struct ArrangeConfig {
    /// Minimum distance between objects (in scaled units).
    pub min_distance: Coord,
    /// Bed width (in scaled units).
    pub bed_width: Coord,
    /// Bed depth (in scaled units).
    pub bed_depth: Coord,
    /// Whether to allow rotation for better packing.
    pub allow_rotation: bool,
    /// Maximum number of iterations.
    pub max_iterations: usize,
}

impl Default for ArrangeConfig {
    fn default() -> Self {
        Self {
            min_distance: 10_000, // 10mm in scaled units
            bed_width: 250_000,   // 250mm
            bed_depth: 250_000,   // 250mm
            allow_rotation: true,
            max_iterations: 100,
        }
    }
}

/// Result of an arrangement operation.
#[derive(Debug, Clone)]
pub struct ArrangeResult {
    /// New positions for each instance.
    pub positions: Vec<(usize, usize, Point)>, // (object_idx, instance_idx, new_position)
    /// Whether all objects could be placed.
    pub all_placed: bool,
    /// Number of objects that didn't fit.
    pub unplaced_count: usize,
}

/// Auto-arrange objects on the print bed.
pub struct Arrange;

impl Arrange {
    // Arrange all instances in a model on the print bed.
    pub fn arrange_model(model: &mut Model, config: &ArrangeConfig) -> ArrangeResult {
        let mut positions = Vec::new();
        let mut current_x = config.min_distance;
        let mut current_y = config.min_distance;
        let mut row_height = 0;
        let mut unplaced_count = 0;

        for (obj_idx, obj) in model.objects.iter_mut().enumerate() {
            if !obj.printable {
                continue;
            }

            for (inst_idx, instance) in obj.instances.iter_mut().enumerate() {
                if !instance.printable {
                    continue;
                }

                let bbox = obj.bounding_box();
                let width = (bbox.max.x - bbox.min.x) as Coord;
                let height = (bbox.max.y - bbox.min.y) as Coord;

                if current_x + width + config.min_distance > config.bed_width {
                    current_x = config.min_distance;
                    current_y += row_height + config.min_distance;
                    row_height = 0;
                }

                if current_y + height + config.min_distance > config.bed_depth {
                    unplaced_count += 1;
                    continue;
                }

                let new_position = Point::new(current_x + width / 2, current_y + height / 2);

                instance.position.x = new_position.x as f64;
                instance.position.y = new_position.y as f64;

                positions.push((obj_idx, inst_idx, new_position));

                current_x += width + config.min_distance;
                row_height = row_height.max(height);
            }
        }

        ArrangeResult {
            positions,
            all_placed: unplaced_count == 0,
            unplaced_count,
        }
    }

    /// Simple grid-based arrangement.
    pub fn arrange_grid(objects: &[&ModelObject], spacing: Coord) -> Vec<Point> {
        let mut positions = Vec::new();
        let grid_size = (objects.len() as f64).sqrt().ceil() as usize;
        let mut max_width = 0;
        let mut max_depth = 0;

        for obj in objects {
            let bbox = obj.bounding_box();
            let width = (bbox.max.x - bbox.min.x) as Coord;
            let depth = (bbox.max.y - bbox.min.y) as Coord;
            max_width = max_width.max(width);
            max_depth = max_depth.max(depth);
        }

        for (i, _) in objects.iter().enumerate() {
            let row = i / grid_size;
            let col = i % grid_size;

            let x = col as Coord * (max_width + spacing);
            let y = row as Coord * (max_depth + spacing);

            positions.push(Point::new(x, y));
        }

        positions
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::triangle_mesh::{ModelObject, TriangleMesh};

    #[test]
    fn test_arrange_grid() {
        let mesh = TriangleMesh::new();
        let obj = ModelObject::new("test", mesh);
        let objects = vec![&obj, &obj, &obj, &obj];

        let positions = Arrange::arrange_grid(&objects, 10_000);
        assert_eq!(positions.len(), 4);
    }
}
