//! WallToolPaths - generates variable-width wall toolpaths using Arachne algorithm
//!
//! C++ Reference:
//! - Arachne/WallToolPaths.hpp
//! - Arachne/WallToolPaths.cpp
//!
//! **STATUS:** 🔄 IN PROGRESS - Core structure and main methods implemented
//!
//! **TODO:** Complete implementation requires:
//! - BeadingStrategyFactory::make_strategy() implementation
//! - SkeletalTrapezoidation::new() and generate_toolpaths() implementation
//! - PolylineStitcher for stitch_tool_paths()

// TODO: Uncomment when BeadingStrategyFactory is implemented
// use crate::arachne::beading_strategy::beading_strategy_factory::BeadingStrategyFactory;
// TODO: Uncomment when SkeletalTrapezoidation is implemented
// use crate::arachne::skeletal_trapezoidation::SkeletalTrapezoidation;
use crate::arachne::utils::extrusion_line::VariableWidthLines;
use crate::geometry::{Coord, CoordF, Polygons};

// Constants from WallToolPaths.hpp:17-21
// WallToolPaths.hpp:17
pub const FILL_OUTLINE_GAPS: bool = true;

// WallToolPaths.hpp:18
pub const MESHFIX_MAXIMUM_RESOLUTION: Coord = 500_000; // scale(0.5)

// WallToolPaths.hpp:19
pub const MESHFIX_MAXIMUM_DEVIATION: Coord = 25_000; // scale(0.025)

// WallToolPaths.hpp:20
pub const MESHFIX_MAXIMUM_EXTRUSION_AREA_DEVIATION: Coord = 2_000_000; // scale(2.0)

/// Parameters for WallToolPaths generation
/// WallToolPaths.hpp:23-32
#[derive(Debug, Clone, Copy)]
pub struct WallToolPathsParams {
    /// Minimum bead width (in mm)
    /// WallToolPaths.hpp:26
    pub min_bead_width: f32,

    /// Minimum feature size (in mm)
    /// WallToolPaths.hpp:27
    pub min_feature_size: f32,

    /// Wall transition length (in mm)
    /// WallToolPaths.hpp:28
    pub wall_transition_length: f32,

    /// Wall transition angle (in radians)
    /// WallToolPaths.hpp:29
    pub wall_transition_angle: f32,

    /// Wall transition filter deviation (in mm)
    /// WallToolPaths.hpp:30
    pub wall_transition_filter_deviation: f32,

    /// Wall distribution count
    /// WallToolPaths.hpp:31
    pub wall_distribution_count: i32,
}

impl Default for WallToolPathsParams {
    fn default() -> Self {
        Self {
            min_bead_width: 0.34,
            min_feature_size: 0.1,
            wall_transition_length: 0.4,
            wall_transition_angle: 0.174533, // ~10 degrees
            wall_transition_filter_deviation: 0.025,
            wall_distribution_count: 1,
        }
    }
}

/// Main class for generating variable-width wall toolpaths
/// WallToolPaths.hpp:34-150
pub struct WallToolPaths {
    /// Reference to the outline polygon
    /// WallToolPaths.hpp:143
    outline: Polygons,

    /// First wall bead width
    /// WallToolPaths.hpp:144
    bead_width_0: Coord,

    /// Inner walls bead width
    /// WallToolPaths.hpp:145
    bead_width_x: Coord,

    /// Maximum number of walls
    /// WallToolPaths.hpp:146
    inset_count: usize,

    /// Outer wall inset distance
    /// WallToolPaths.hpp:147
    wall_0_inset: Coord,

    /// Layer height
    /// WallToolPaths.hpp:148
    layer_height: CoordF,

    /// Whether to print thin walls
    /// WallToolPaths.hpp:149
    print_thin_walls: bool,

    /// Minimum feature size (scaled)
    /// WallToolPaths.hpp:150
    min_feature_size: Coord,

    /// Minimum bead width (scaled)
    /// WallToolPaths.hpp:151
    min_bead_width: Coord,

    /// Small area length threshold
    /// WallToolPaths.hpp:152
    small_area_length: f64,

    /// Wall transition filter deviation (scaled)
    /// WallToolPaths.hpp:153
    wall_transition_filter_deviation: Coord,

    /// Whether toolpaths have been generated
    /// WallToolPaths.hpp:154
    toolpaths_generated: bool,

    /// Generated toolpaths
    /// WallToolPaths.hpp:155
    toolpaths: Vec<VariableWidthLines>,

    /// Inner contour of walls
    /// WallToolPaths.hpp:156
    inner_contour: Polygons,

    /// First wall contour
    /// WallToolPaths.hpp:157
    first_wall_contour: Polygons,

    /// Parameters
    /// WallToolPaths.hpp:158
    params: WallToolPathsParams,

    /// Hole compensation enabled
    /// WallToolPaths.hpp:160
    enable_hole_compensation: bool,

    /// Hole indices for compensation
    /// WallToolPaths.hpp:161
    hole_indices: Vec<i32>,
}

impl WallToolPaths {
    /// Create a new WallToolPaths generator
    /// WallToolPaths.cpp:26-42
    pub fn new(
        outline: Polygons,
        bead_width_0: Coord,
        bead_width_x: Coord,
        inset_count: usize,
        wall_0_inset: Coord,
        layer_height: CoordF,
        params: WallToolPathsParams,
    ) -> Self {
        Self {
            outline,
            bead_width_0,
            bead_width_x,
            inset_count,
            wall_0_inset,
            layer_height,
            print_thin_walls: FILL_OUTLINE_GAPS,
            min_feature_size: (params.min_feature_size as f64 * 1_000_000.0) as Coord,
            min_bead_width: (params.min_bead_width as f64 * 1_000_000.0) as Coord,
            small_area_length: bead_width_0 as f64 / 2.0,
            wall_transition_filter_deviation: (params.wall_transition_filter_deviation as f64
                * 1_000_000.0) as Coord,
            toolpaths_generated: false,
            toolpaths: Vec::new(),
            inner_contour: Polygons::new(),
            first_wall_contour: Polygons::new(),
            params,
            enable_hole_compensation: false,
            hole_indices: Vec::new(),
        }
    }

    /// Enable hole compensation for specified holes
    /// WallToolPaths.cpp:44-48
    pub fn enable_hole_compensation(&mut self, enable: bool, hole_indices: Vec<i32>) {
        self.enable_hole_compensation = enable;
        self.hole_indices = hole_indices;
    }

    /// Generate the toolpaths
    /// WallToolPaths.cpp:441-550
    pub fn generate(&mut self) -> &Vec<VariableWidthLines> {
        if self.toolpaths_generated {
            return &self.toolpaths;
        }

        // Mark as generated even if we return early
        self.toolpaths_generated = true;

        // Prepare outline by simplifying
        let _prepared_outline = self.outline.clone();

        // TODO: Apply simplification, hole compensation, and other preprocessing
        // This is a complex section (lines 441-491 in C++)

        // TODO: Create beading strategy and skeletal trapezoidation
        // This requires full implementation of BeadingStrategyFactory and SkeletalTrapezoidation
        // For now, return empty toolpaths
        // let beading_strategy = BeadingStrategyFactory::make_strategy(...);
        // let mut wall_maker = SkeletalTrapezoidation::new(...);
        // wall_maker.generate_toolpaths(&mut self.toolpaths);

        // Post-process toolpaths
        Self::stitch_tool_paths(&mut self.toolpaths, self.bead_width_x);
        Self::remove_small_lines(&mut self.toolpaths);
        Self::simplify_tool_paths(&mut self.toolpaths);
        Self::remove_empty_tool_paths(&mut self.toolpaths);

        &self.toolpaths
    }

    /// Get the toolpaths (generates if needed)
    /// WallToolPaths.cpp:694-699
    pub fn get_tool_paths(&mut self) -> &Vec<VariableWidthLines> {
        if !self.toolpaths_generated {
            self.generate();
        }
        &self.toolpaths
    }

    /// Separate out the inner contour from wall toolpaths
    /// WallToolPaths.cpp:701-770
    pub fn separate_out_inner_contour(&mut self) {
        // Separate toolpaths into actual paths, wall contours, and first wall contours
        let mut actual_toolpaths = Vec::new();
        let mut wall_contour_paths = Vec::new();
        let mut first_wall_contour_paths = Vec::new();

        for toolpath in &self.toolpaths {
            let mut actual_lines = Vec::new();
            let mut wall_contour_lines = Vec::new();
            let mut first_wall_contour_lines = Vec::new();

            for line in toolpath {
                // Determine path type based on bead widths
                // WallToolPaths.cpp:716-728
                if line.junctions.is_empty() {
                    continue;
                }

                // Check if this is a contour line (marked width)
                let is_contour = line.junctions.iter().all(|j| j.w <= 1);
                let is_first_wall = line.junctions.iter().any(|j| j.w == 1);

                if is_first_wall {
                    first_wall_contour_lines.push(line.clone());
                } else if is_contour {
                    wall_contour_lines.push(line.clone());
                } else {
                    actual_lines.push(line.clone());
                }
            }

            if !actual_lines.is_empty() {
                actual_toolpaths.push(actual_lines);
            }
            if !wall_contour_lines.is_empty() {
                wall_contour_paths.push(wall_contour_lines);
            }
            if !first_wall_contour_lines.is_empty() {
                first_wall_contour_paths.push(first_wall_contour_lines);
            }
        }

        self.toolpaths = actual_toolpaths;

        // Convert contour paths to polygons
        for contour_path in &wall_contour_paths {
            for line in contour_path {
                if line.is_closed && line.junctions.len() >= 3 {
                    self.inner_contour.push(line.to_polygon());
                }
            }
        }

        for contour_path in &first_wall_contour_paths {
            for line in contour_path {
                if line.is_closed && line.junctions.len() >= 3 {
                    self.first_wall_contour.push(line.to_polygon());
                }
            }
        }
    }

    /// Get the inner contour (generates if needed)
    /// WallToolPaths.cpp:772-783
    pub fn get_inner_contour(&mut self) -> &Polygons {
        if !self.toolpaths_generated {
            self.generate();
        }

        if self.inner_contour.is_empty() {
            self.separate_out_inner_contour();
        }

        &self.inner_contour
    }

    /// Get the first wall contour (generates if needed)
    /// WallToolPaths.cpp:785-796
    pub fn get_first_wall_contour(&mut self) -> &Polygons {
        if !self.toolpaths_generated {
            self.generate();
        }

        if self.first_wall_contour.is_empty() {
            self.separate_out_inner_contour();
        }

        &self.first_wall_contour
    }

    /// Stitch toolpaths together to form closed polygons
    /// WallToolPaths.cpp:552-648
    fn stitch_tool_paths(toolpaths: &mut Vec<VariableWidthLines>, bead_width_x: Coord) {
        let _stitch_distance = bead_width_x - 1;

        for wall_lines in toolpaths.iter_mut() {
            let mut stitched_polylines = Vec::new();
            let mut closed_polygons = Vec::new();

            // Separate already-closed polygons from open polylines
            for line in wall_lines.drain(..) {
                if line.is_closed {
                    closed_polygons.push(line);
                } else {
                    stitched_polylines.push(line);
                }
            }

            // TODO: Implement polyline stitching using PolylineStitcher
            // This is complex logic from WallToolPaths.cpp:559-646
            // For now, just keep the polylines as-is

            // Combine closed polygons and stitched polylines back
            wall_lines.extend(closed_polygons);
            wall_lines.extend(stitched_polylines);
        }
    }

    /// Remove small lines that are shorter than half their minimum width
    /// WallToolPaths.cpp:663-678
    fn remove_small_lines(toolpaths: &mut Vec<VariableWidthLines>) {
        for toolpath in toolpaths.iter_mut() {
            toolpath.retain(|line| {
                if line.junctions.is_empty() {
                    return false;
                }

                let min_width = line.get_minimal_width();
                let length = line.get_length();

                // Keep line if it's longer than half its minimum width
                length >= min_width / 2
            });
        }
    }

    /// Simplify toolpaths to reduce resolution
    /// WallToolPaths.cpp:680-692
    fn simplify_tool_paths(toolpaths: &mut Vec<VariableWidthLines>) {
        let maximum_resolution = MESHFIX_MAXIMUM_RESOLUTION;
        let maximum_deviation = MESHFIX_MAXIMUM_DEVIATION;
        let maximum_extrusion_area_deviation = MESHFIX_MAXIMUM_EXTRUSION_AREA_DEVIATION;

        for toolpath in toolpaths.iter_mut() {
            for line in toolpath.iter_mut() {
                line.simplify(
                    maximum_resolution * maximum_resolution,
                    maximum_deviation * maximum_deviation,
                    maximum_extrusion_area_deviation,
                );
            }
        }
    }

    /// Remove empty toolpaths
    /// WallToolPaths.cpp:799-806
    pub fn remove_empty_tool_paths(toolpaths: &mut Vec<VariableWidthLines>) -> bool {
        toolpaths.retain(|lines| !lines.is_empty());
        !toolpaths.is_empty()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_wall_tool_paths_params_default() {
        let params = WallToolPathsParams::default();
        assert!(params.min_bead_width > 0.0);
        assert!(params.min_feature_size > 0.0);
        assert!(params.wall_transition_angle > 0.0);
    }

    #[test]
    fn test_wall_tool_paths_creation() {
        let outline = Polygons::new();
        let params = WallToolPathsParams::default();
        let wall_paths = WallToolPaths::new(
            outline, 400, // bead_width_0
            400, // bead_width_x
            3,   // inset_count
            0,   // wall_0_inset
            0.2, // layer_height
            params,
        );

        assert_eq!(wall_paths.bead_width_0, 400);
        assert_eq!(wall_paths.bead_width_x, 400);
        assert_eq!(wall_paths.inset_count, 3);
        assert!(!wall_paths.toolpaths_generated);
    }

    #[test]
    fn test_enable_hole_compensation() {
        let outline = Polygons::new();
        let params = WallToolPathsParams::default();
        let mut wall_paths = WallToolPaths::new(outline, 400, 400, 3, 0, 0.2, params);

        assert!(!wall_paths.enable_hole_compensation);

        wall_paths.enable_hole_compensation(true, vec![0, 1, 2]);
        assert!(wall_paths.enable_hole_compensation);
        assert_eq!(wall_paths.hole_indices, vec![0, 1, 2]);
    }

    #[test]
    fn test_remove_empty_tool_paths() {
        use crate::arachne::utils::extrusion_line::ExtrusionLine;
        let mut toolpaths = vec![vec![], vec![ExtrusionLine::new(0, false)], vec![]];

        let has_paths = WallToolPaths::remove_empty_tool_paths(&mut toolpaths);
        assert!(has_paths);
        assert_eq!(toolpaths.len(), 1);
    }

    #[test]
    fn test_remove_empty_tool_paths_all_empty() {
        let mut toolpaths: Vec<VariableWidthLines> = vec![vec![], vec![], vec![]];

        let has_paths = WallToolPaths::remove_empty_tool_paths(&mut toolpaths);
        assert!(!has_paths);
        assert_eq!(toolpaths.len(), 0);
    }
}
