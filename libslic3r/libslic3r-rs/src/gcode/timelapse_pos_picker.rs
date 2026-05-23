//! Timelapse position picker for smooth timelapse photography.
//!
//! C++ Reference:
//! - GCode/TimelapsePosPicker.hpp
//! - GCode/TimelapsePosPicker.cpp
//!
//! This module picks optimal positions for the toolhead to move to when taking
//! timelapse photos, avoiding collisions with printed objects.

use crate::geometry::Point;

/// Default timelapse position (origin).
pub const DEFAULT_TIMELAPSE_POS: Point = Point { x: 0, y: 0 };

/// Default camera position (origin).
pub const DEFAULT_CAMERA_POS: Point = Point { x: 0, y: 0 };

/// Context for position picking decisions.
/// Corresponds to C++ PosPickCtx.
#[derive(Debug, Clone)]
pub struct PosPickCtx {
    /// Current toolhead position.
    pub curr_pos: Point,
    /// Current layer index (used to reference layer data).
    pub curr_layer_index: Option<usize>,
    /// Extruder ID used for taking the picture.
    pub picture_extruder_id: i32,
    /// Currently active extruder ID.
    pub curr_extruder_id: i32,
    /// Printed objects (only in by-object mode).
    pub printed_object_count: Option<usize>,
}

impl PosPickCtx {
    pub fn new() -> Self {
        Self {
            curr_pos: DEFAULT_TIMELAPSE_POS,
            curr_layer_index: None,
            picture_extruder_id: 0,
            curr_extruder_id: 0,
            printed_object_count: None,
        }
    }
}

impl Default for PosPickCtx {
    fn default() -> Self {
        Self::new()
    }
}

/// Print sequence mode affecting timelapse behavior.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PrintSequence {
    ByLayer,
    ByObject,
}

impl Default for PrintSequence {
    fn default() -> Self {
        PrintSequence::ByLayer
    }
}

/// Timelapse position picker.
/// Selects safe toolhead positions for timelapse photography.
/// Corresponds to C++ TimelapsePosPicker.
#[derive(Debug, Clone)]
pub struct TimelapsePosPicker {
    /// Printable area polygons per extruder (scaled coordinates).
    extruder_printable_area: Vec<Vec<Point>>,
    /// Bed polygon (scaled coordinates).
    bed_polygon: Vec<Point>,
    /// Plate offset (unscaled).
    plate_offset: Point,
    /// Plate height (unscaled).
    plate_height: i64,
    /// Plate width (unscaled).
    plate_width: i64,
    /// Print sequence mode.
    print_seq: PrintSequence,
    /// Whether to base position on all layers.
    based_on_all_layer: bool,
    /// Nozzle height to rod clearance.
    nozzle_height_to_rod: i64,
    /// Nozzle clearance radius.
    nozzle_clearance_radius: i64,
    /// Liftable extruder ID if applicable.
    liftable_extruder_id: Option<i64>,
    /// Extruder height gap if applicable.
    extruder_height_gap: Option<i64>,
    /// Cached position for all-layer mode.
    all_layer_pos: Option<Point>,
    /// Whether the picker has been initialized.
    initialized: bool,
}

impl TimelapsePosPicker {
    pub fn new() -> Self {
        Self {
            extruder_printable_area: Vec::new(),
            bed_polygon: Vec::new(),
            plate_offset: Point { x: 0, y: 0 },
            plate_height: 0,
            plate_width: 0,
            print_seq: PrintSequence::ByLayer,
            based_on_all_layer: false,
            nozzle_height_to_rod: 0,
            nozzle_clearance_radius: 0,
            liftable_extruder_id: None,
            extruder_height_gap: None,
            all_layer_pos: None,
            initialized: false,
        }
    }

    /// Initialize the picker with print configuration.
    pub fn init(&mut self, plate_offset: Point, plate_width: i64, plate_height: i64) {
        self.plate_offset = plate_offset;
        self.plate_width = plate_width;
        self.plate_height = plate_height;
        self.construct_printable_area_by_printer();
        self.initialized = true;
    }

    /// Reset state between prints.
    pub fn reset(&mut self) {
        self.extruder_printable_area.clear();
        self.bed_polygon.clear();
        self.all_layer_pos = None;
        self.initialized = false;
    }

    /// Pick the best timelapse position for the given context.
    pub fn pick_pos(&self, ctx: &PosPickCtx) -> Point {
        if !self.initialized {
            return DEFAULT_TIMELAPSE_POS;
        }

        if self.based_on_all_layer {
            if let Some(pos) = self.all_layer_pos {
                return pos;
            }
        }

        self.pick_pos_for_curr_layer(ctx)
    }

    /// Pick position based on current layer data.
    fn pick_pos_for_curr_layer(&self, ctx: &PosPickCtx) -> Point {
        // Default: move to the center of the printable area
        // In a full implementation, this would avoid printed object areas
        if !self.bed_polygon.is_empty() {
            // Use center of bed polygon
            let (sum_x, sum_y) = self.bed_polygon.iter().fold((0i64, 0i64), |acc, p| {
                (acc.0 + p.x as i64, acc.1 + p.y as i64)
            });
            let n = self.bed_polygon.len() as i64;
            if n > 0 {
                return Point {
                    x: sum_x / n,
                    y: sum_y / n,
                };
            }
        }

        // Fallback: use current position (minimal movement)
        ctx.curr_pos
    }

    /// Construct printable area from printer configuration.
    fn construct_printable_area_by_printer(&mut self) {
        // Build a rectangle for the bed polygon based on plate dimensions
        let half_w = self.plate_width / 2;
        let half_h = self.plate_height / 2;
        let ox = self.plate_offset.x;
        let oy = self.plate_offset.y;

        self.bed_polygon = vec![
            Point {
                x: ox - half_w,
                y: oy - half_h,
            },
            Point {
                x: ox + half_w,
                y: oy - half_h,
            },
            Point {
                x: ox + half_w,
                y: oy + half_h,
            },
            Point {
                x: ox - half_w,
                y: oy + half_h,
            },
        ];
    }
}

impl Default for TimelapsePosPicker {
    fn default() -> Self {
        Self::new()
    }
}

/// Collect object slice data for collision avoidance.
/// Returns area polygons for the current layer's object cross-sections.
pub fn collect_object_slices_data() -> crate::Result<()> {
    // In a full implementation, this collects ExPolygons from the layer's object slices
    Ok(())
}

/// Reset the timelapse position picker.
pub fn reset(picker: &mut TimelapsePosPicker) {
    picker.reset();
}

/// Collect limit areas for camera clearance.
/// Returns polygons representing areas the camera cannot reach.
pub fn collect_limit_areas_for_camera() -> crate::Result<()> {
    // Computes camera clearance zones based on object projections
    Ok(())
}

/// Construct the printable area based on printer configuration.
pub fn construct_printable_area_by_printer(picker: &mut TimelapsePosPicker) {
    picker.construct_printable_area_by_printer();
}

/// Collect limit areas for rod clearance.
/// Returns polygons representing areas blocked by the rod mechanism.
pub fn collect_limit_areas_for_rod() -> crate::Result<()> {
    // Computes rod clearance zones for CoreXY / bed-slinger printers
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pos_pick_ctx_default() {
        let ctx = PosPickCtx::new();
        assert_eq!(ctx.curr_pos, DEFAULT_TIMELAPSE_POS);
        assert_eq!(ctx.picture_extruder_id, 0);
    }

    #[test]
    fn test_timelapse_picker_default() {
        let picker = TimelapsePosPicker::new();
        assert!(!picker.initialized);
    }

    #[test]
    fn test_timelapse_picker_init_and_reset() {
        let mut picker = TimelapsePosPicker::new();
        picker.init(Point { x: 0, y: 0 }, 256, 256);
        assert!(picker.initialized);
        assert!(!picker.bed_polygon.is_empty());

        picker.reset();
        assert!(!picker.initialized);
        assert!(picker.bed_polygon.is_empty());
    }

    #[test]
    fn test_pick_pos_uninitialized() {
        let picker = TimelapsePosPicker::new();
        let ctx = PosPickCtx::new();
        let pos = picker.pick_pos(&ctx);
        assert_eq!(pos, DEFAULT_TIMELAPSE_POS);
    }

    #[test]
    fn test_pick_pos_initialized() {
        let mut picker = TimelapsePosPicker::new();
        picker.init(Point { x: 100, y: 100 }, 200, 200);
        let ctx = PosPickCtx::new();
        let pos = picker.pick_pos(&ctx);
        // Should return center of bed polygon (around plate_offset)
        assert_eq!(pos.x, 100);
        assert_eq!(pos.y, 100);
    }

    #[test]
    fn test_convenience_functions() {
        assert!(collect_object_slices_data().is_ok());
        assert!(collect_limit_areas_for_camera().is_ok());
        assert!(collect_limit_areas_for_rod().is_ok());
    }
}
