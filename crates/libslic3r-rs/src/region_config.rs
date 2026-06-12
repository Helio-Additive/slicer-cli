//! Print region configuration.
//!
//! This module provides the PrintRegionConfig type for controlling
//! region-specific print settings, mirroring BambuStudio's PrintRegionConfig.

use crate::config::FloatOrPercent;
use crate::perimeter_generator::WallGeneratorMode;
use crate::print_config::{InfillPattern, ScarfSeamType, SeamPosition, WallSequence};
use crate::CoordF;
use serde::{Deserialize, Serialize};
use std::fmt;

#[derive(Clone, Debug, Serialize, Deserialize)]
/// Configuration for a specific print region
/// PrintConfig.hpp:939-1050
pub struct PrintRegionConfig {
    // === Perimeters ===
    /// Number of perimeters/shells
    /// PrintConfig.hpp:998
    pub perimeters: u32,

    /// External perimeter extrusion width (mm, 0 = auto)
    /// C++ name: outer_wall_line_width
    /// PrintConfig.hpp:954
    pub outer_wall_line_width: CoordF,

    /// Perimeter extrusion width (mm, 0 = auto)
    /// C++ name: inner_wall_line_width
    /// PrintConfig.hpp:996
    pub inner_wall_line_width: CoordF,

    /// External perimeter speed (mm/s)
    /// PrintConfig.hpp:956
    pub external_perimeter_speed: CoordF,

    /// Perimeter speed (mm/s)
    /// PrintConfig.hpp:997
    pub perimeter_speed: CoordF,

    /// Small perimeter speed (mm/s)
    /// PrintConfig.hpp:997
    pub small_perimeter_speed: CoordF,

    /// Enable thin walls detection
    /// PrintConfig.hpp:998
    pub thin_walls: bool,

    /// Enable detect bridging perimeters
    /// PrintConfig.hpp:989
    pub overhangs: bool,

    /// Extra perimeters if needed for vertical shells
    /// PrintConfig.hpp:949
    pub extra_perimeters: bool,

    /// Extra perimeters on overhangs
    /// PrintConfig.hpp:949
    pub extra_perimeters_on_overhangs: bool,

    // === Infill ===
    /// Infill density (0.0 - 1.0)
    /// PrintConfig.hpp:965
    pub fill_density: CoordF,

    /// Infill pattern
    /// PrintConfig.hpp:968
    pub fill_pattern: InfillPattern,

    /// Solid infill pattern (for top/bottom)
    /// PrintConfig.hpp:952
    pub solid_fill_pattern: InfillPattern,

    /// Top solid infill pattern
    /// PrintConfig.hpp:950
    pub top_fill_pattern: InfillPattern,

    /// Bottom solid infill pattern
    /// PrintConfig.hpp:948
    pub bottom_fill_pattern: InfillPattern,

    /// Infill angle (degrees)
    /// PrintConfig.hpp:955
    pub fill_angle: CoordF,

    /// Infill extrusion width (mm, 0 = auto)
    /// C++ name: sparse_infill_line_width
    /// PrintConfig.hpp:973
    pub sparse_infill_line_width: CoordF,

    /// Solid infill extrusion width (mm, 0 = auto)
    /// C++ name: internal_solid_infill_line_width
    /// PrintConfig.hpp:975
    pub internal_solid_infill_line_width: CoordF,

    /// Top solid infill extrusion width (mm, 0 = auto)
    /// C++ name: top_surface_line_width
    /// PrintConfig.hpp:974
    pub top_surface_line_width: CoordF,

    /// Infill speed (mm/s)
    /// PrintConfig.hpp:978
    pub infill_speed: CoordF,

    /// Solid infill speed (mm/s)
    /// PrintConfig.hpp:978
    pub solid_infill_speed: CoordF,

    /// Top solid infill speed (mm/s)
    /// PrintConfig.hpp:978
    pub top_solid_infill_speed: CoordF,

    /// Infill overlap with perimeters (ratio, 0.0 - 1.0)
    /// PrintConfig.hpp:972
    pub infill_overlap: CoordF,

    /// Infill anchor length (mm, or % of sparse infill line width)
    /// C++ name: sparse_infill_anchor (ConfigOptionFloatOrPercent,
    /// ratio_over = "sparse_infill_line_width"; default 400%,
    /// PrintConfig.cpp:3525-3551)
    pub infill_anchor: FloatOrPercent,

    /// Maximum infill anchor length (mm, or % of sparse infill line width)
    /// C++ name: sparse_infill_anchor_max (ConfigOptionFloatOrPercent;
    /// default 20mm, PrintConfig.cpp:3553-3579)
    pub infill_anchor_max: FloatOrPercent,

    /// Number of parallel lines drawn per sparse infill path.
    /// C++ name: fill_multiline (ConfigOptionInt, default 1,
    /// PrintConfig.cpp:2746-2752)
    pub fill_multiline: i32,

    /// Skin pattern of the locked-zag infill.
    /// C++ name: locked_skin_infill_pattern (default ipCrossZag,
    /// PrintConfig.cpp:2844)
    pub locked_skin_infill_pattern: InfillPattern,

    /// Skeleton pattern of the locked-zag infill.
    /// C++ name: locked_skeleton_infill_pattern (default ipZigZag,
    /// PrintConfig.cpp:2885)
    pub locked_skeleton_infill_pattern: InfillPattern,

    /// Per-layer infill shift step (mm) for cross-zag / locked-zag.
    /// C++ name: infill_shift_step (ConfigOptionFloat, default 0.4,
    /// PrintConfig.cpp:3423)
    pub infill_shift_step: CoordF,

    /// Per-layer infill rotate step (degrees) for zig-zag cross texture.
    /// C++ name: infill_rotate_step (ConfigOptionFloat, default 0,
    /// PrintConfig.cpp:3433)
    pub infill_rotate_step: CoordF,

    /// Mirror the infill across the object's Y axis.
    /// C++ name: symmetric_infill_y_axis (ConfigOptionBool, default false,
    /// PrintConfig.cpp:3503)
    pub symmetric_infill_y_axis: bool,

    /// First lattice angle (degrees) for the 2D-lattice infill.
    /// C++ name: sparse_infill_lattice_angle_1 (default -45,
    /// PrintConfig.cpp:3513)
    pub sparse_infill_lattice_angle_1: CoordF,

    /// Second lattice angle (degrees) for the 2D-lattice infill.
    /// C++ name: sparse_infill_lattice_angle_2 (default 45,
    /// PrintConfig.cpp:3523)
    pub sparse_infill_lattice_angle_2: CoordF,

    /// Top surface density (PERCENT value, 0-100 as in the C++
    /// ConfigOptionPercent — NOT the 0.0-1.0 ratio convention used by
    /// `fill_density`).
    /// C++ name: top_surface_density (default 100, PrintConfig.cpp:1858)
    pub top_surface_density: CoordF,

    /// Bottom surface density (PERCENT value, 0-100).
    /// C++ name: bottom_surface_density (default 100, PrintConfig.cpp:1890)
    pub bottom_surface_density: CoordF,

    /// Travel into wall ratio (PERCENT of line width) for monotonic-line fill.
    /// C++ name: monotonic_travel_into_wall (ConfigOptionPercent, default 0,
    /// PrintConfig.cpp:1868)
    pub monotonic_travel_into_wall: CoordF,

    /// Skin (outer zone) line width of the locked-zag infill (mm).
    /// C++ name: skin_infill_line_width (ConfigOptionFloat, default 0.4,
    /// PrintConfig.cpp:3486)
    pub skin_infill_line_width: CoordF,

    /// Skeleton (inner zone) line width of the locked-zag infill (mm).
    /// C++ name: skeleton_infill_line_width (ConfigOptionFloat, default 0.4,
    /// PrintConfig.cpp:3495)
    pub skeleton_infill_line_width: CoordF,

    /// Skin infill density (PERCENT value, 0-100).
    /// C++ name: skin_infill_density (ConfigOptionPercent, default 15,
    /// PrintConfig.cpp:3457)
    pub skin_infill_density: CoordF,

    /// Skeleton infill density (PERCENT value, 0-100).
    /// C++ name: skeleton_infill_density (ConfigOptionPercent, default 15,
    /// PrintConfig.cpp:3445)
    pub skeleton_infill_density: CoordF,

    /// Depth of the locked-zag skin zone (mm).
    /// C++ name: skin_infill_depth (ConfigOptionFloat, default 2.0,
    /// PrintConfig.cpp:3467)
    pub skin_infill_depth: CoordF,

    /// Locked-zag transition zone depth (mm).
    /// C++ name: infill_lock_depth (ConfigOptionFloat, default 1.0,
    /// PrintConfig.cpp:3477)
    pub infill_lock_depth: CoordF,

    /// Replace top/bottom surfaces with infill (locked-zag).
    /// C++ name: infill_instead_top_bottom_surfaces
    pub infill_instead_top_bottom_surfaces: bool,

    // === Solid Layers ===
    /// Number of solid top layers
    /// PrintConfig.hpp:944
    pub top_solid_layers: u32,

    /// Number of solid bottom layers
    /// PrintConfig.hpp:943
    pub bottom_solid_layers: u32,

    /// Minimum shell thickness (mm) for solid infill
    /// PrintConfig.hpp:944
    pub top_solid_min_thickness: CoordF,

    /// Minimum shell thickness (mm) for solid infill
    /// PrintConfig.hpp:943
    pub bottom_solid_min_thickness: CoordF,

    /// Whether to ensure vertical shell thickness near sloped walls. When !=
    /// Disabled, discover_horizontal_shells is skipped (the shell work is done by
    /// discover_vertical_shells). C++ default is Enabled. PrintConfig.hpp:83-87.
    pub ensure_vertical_shell_thickness: EnsureVerticalThicknessLevel,

    // === Bridges ===
    /// Bridge speed (mm/s)
    /// PrintConfig.hpp:947
    pub bridge_speed: CoordF,

    /// Bridge flow ratio
    /// PrintConfig.hpp:946
    pub bridge_flow_ratio: CoordF,

    /// Bridge angle (degrees, 0 = auto)
    /// PrintConfig.hpp:945
    pub bridge_angle: CoordF,

    // === Gap Fill ===
    /// Enable gap fill
    /// PrintConfig.hpp:976
    pub gap_fill_enabled: bool,

    /// Gap fill speed (mm/s)
    /// PrintConfig.hpp:976
    pub gap_fill_speed: CoordF,

    // === Seam ===
    /// Seam position preference
    /// PrintConfig.hpp:1000
    pub seam_position: SeamPosition,

    /// Seam angle cost (for seam placement algorithm)
    /// PrintConfig.hpp:1000
    pub seam_angle_cost: CoordF,

    /// Seam travel cost (for seam placement algorithm)
    /// PrintConfig.hpp:1000
    pub seam_travel_cost: CoordF,

    // === Scarf Seam (seam slope) ===
    /// Override filament scarf seam setting (so a modifier can control it).
    /// PrintConfig.hpp:1127 ((ConfigOptionBool, override_filament_scarf_seam_setting))
    pub override_filament_scarf_seam_setting: bool,

    /// Scarf seam type (C++ SeamScarfType: none/external/all).
    /// PrintConfig.hpp:1128 ((ConfigOptionEnum<SeamScarfType>, seam_slope_type))
    pub seam_slope_type: ScarfSeamType,

    /// Apply scarf joints only to smooth perimeters (conditional scarf).
    /// PrintConfig.hpp:1129 ((ConfigOptionBool, seam_slope_conditional))
    pub seam_slope_conditional: bool,

    /// Scarf start height (mm or % of layer height; ratio_over = "layer_height").
    /// PrintConfig.hpp:1130 ((ConfigOptionFloatOrPercent, seam_slope_start_height))
    pub seam_slope_start_height: FloatOrPercent,

    /// Scarf slope gap (mm or % of nozzle diameter; ratio_over = "nozzle_diameter").
    /// PrintConfig.hpp:1131 ((ConfigOptionFloatOrPercent, seam_slope_gap))
    pub seam_slope_gap: FloatOrPercent,

    /// The scarf extends to the entire length of the wall.
    /// PrintConfig.hpp:1132 ((ConfigOptionBool, seam_slope_entire_loop))
    pub seam_slope_entire_loop: bool,

    /// Length of the scarf (mm); zero disables the scarf.
    /// PrintConfig.hpp:1133 ((ConfigOptionFloat, seam_slope_min_length))
    pub seam_slope_min_length: CoordF,

    /// Minimum number of segments of each scarf.
    /// PrintConfig.hpp:1134 ((ConfigOptionInt, seam_slope_steps))
    pub seam_slope_steps: i32,

    /// Use scarf joint for inner walls as well.
    /// PrintConfig.hpp:1135 ((ConfigOptionBool, seam_slope_inner_walls))
    pub seam_slope_inner_walls: bool,

    // === Ironing ===
    /// Enable ironing (smoothing top surfaces)
    /// PrintConfig.hpp:980
    pub ironing: bool,

    /// Ironing type
    /// PrintConfig.hpp:980
    pub ironing_type: IroningType,

    /// Ironing flow rate ratio
    /// PrintConfig.hpp:982
    pub ironing_flow_rate: CoordF,

    /// Ironing spacing (mm)
    /// PrintConfig.hpp:983
    pub ironing_spacing: CoordF,

    /// Ironing speed (mm/s)
    /// PrintConfig.hpp:986
    pub ironing_speed: CoordF,

    // === Fuzzy Skin ===
    /// Enable fuzzy skin
    /// PrintConfig.hpp:969
    pub fuzzy_skin: bool,

    /// Fuzzy skin mode
    /// PrintConfig.hpp:969
    pub fuzzy_skin_mode: FuzzySkinMode,

    /// Fuzzy skin thickness (mm)
    /// PrintConfig.hpp:970
    pub fuzzy_skin_thickness: CoordF,

    /// Fuzzy skin point distance (mm)
    /// PrintConfig.hpp:971
    pub fuzzy_skin_point_distance: CoordF,

    /// Fuzzy skin type (which contours/holes/walls are fuzzified).
    /// PrintConfig.hpp:1052 (ConfigOptionEnum<FuzzySkinType> fuzzy_skin)
    pub fuzzy_skin_type: FuzzySkinType,

    /// Whether the first layer is fuzzified.
    /// PrintConfig.hpp:1055 (ConfigOptionBool fuzzy_skin_first_layer)
    pub fuzzy_skin_first_layer: bool,

    /// Noise function driving the fuzzy displacement.
    /// PrintConfig.hpp:1056 (ConfigOptionEnum<NoiseType> fuzzy_skin_noise_type)
    pub fuzzy_skin_noise_type: NoiseType,

    /// Noise scale (mm) for the procedural noise modules.
    /// PrintConfig.hpp:1057 (ConfigOptionFloat fuzzy_skin_scale)
    pub fuzzy_skin_scale: CoordF,

    /// Octave count for fractal noise modules.
    /// PrintConfig.hpp:1058 (ConfigOptionInt fuzzy_skin_octaves)
    pub fuzzy_skin_octaves: i32,

    /// Persistence for fractal noise modules.
    /// PrintConfig.hpp:1059 (ConfigOptionFloat fuzzy_skin_persistence)
    pub fuzzy_skin_persistence: CoordF,

    /// How the fuzzy displacement is applied to extrusion lines.
    /// PrintConfig.hpp:1060 (ConfigOptionEnum<FuzzySkinMode> fuzzy_skin_mode)
    pub fuzzy_skin_displacement_mode: FuzzySkinDisplacementMode,

    // === Wall Generation Mode ===
    /// Perimeter generator mode (Classic or Arachne).
    /// BambuStudio: `wall_generator` / `perimeter_generator`.
    pub wall_generator_mode: WallGeneratorMode,

    /// Wall sequence (inner/outer ordering).
    /// BambuStudio: `wall_sequence` in PrintRegionConfig.
    pub wall_sequence: WallSequence,

    // === Misc ===
    /// Region identifier/name
    /// PrintConfig.hpp:939
    pub region_id: usize,

    /// Wall filament extruder (1-based, C++ uses wall_filament)
    /// PrintConfig.hpp:993
    pub wall_filament: usize,

    /// Sparse infill filament extruder (1-based, C++ uses sparse_infill_filament)
    /// PrintConfig.hpp:977
    pub sparse_infill_filament: usize,

    /// Solid infill filament extruder (1-based, C++ uses solid_infill_filament)
    /// PrintConfig.hpp:977
    pub solid_infill_filament: usize,

    /// Filter out gap fill extrusions shorter than this (mm).
    /// PrintConfig.hpp:1122 ((ConfigOptionFloat, filter_out_gap_fill))
    pub filter_out_gap_fill: CoordF,

    /// Embed wall into infill (used by InterlockingGenerator::generate_embedding_wall).
    /// PrintConfig.hpp:1136 ((ConfigOptionBool, embedding_wall_into_infill))
    pub embedding_wall_into_infill: bool,
}

/// Implementation of PrintRegionConfig methods
/// PrintConfig.hpp:939-1050
impl PrintRegionConfig {
    // Create a new PrintRegionConfig with default values
    // PrintConfig.hpp:939
    pub fn new() -> Self {
        Self::default()
    }

    /// Create a config with a specific region ID
    /// PrintConfig.hpp:939
    pub fn with_region_id(region_id: usize) -> Self {
        Self {
            region_id,
            ..Default::default()
        }
    }

    /// Builder method: set number of perimeters
    /// PrintConfig.hpp:998
    pub fn perimeters(mut self, count: u32) -> Self {
        // PrintConfig.hpp:998
        self.perimeters = count;
        // PrintConfig.hpp:998
        self
    }

    /// Builder method: set infill density
    /// PrintConfig.hpp:965
    pub fn fill_density(mut self, density: CoordF) -> Self {
        // PrintConfig.hpp:965
        self.fill_density = density;
        // PrintConfig.hpp:965
        self
    }

    /// Builder method: set infill pattern
    /// PrintConfig.hpp:968
    pub fn fill_pattern(mut self, pattern: InfillPattern) -> Self {
        // PrintConfig.hpp:968
        self.fill_pattern = pattern;
        // PrintConfig.hpp:968
        self
    }

    /// Builder method: set top solid layers
    /// PrintConfig.hpp:944
    pub fn top_solid_layers(mut self, layers: u32) -> Self {
        // PrintConfig.hpp:944
        self.top_solid_layers = layers;
        // PrintConfig.hpp:944
        self
    }

    /// Builder method: set bottom solid layers
    /// PrintConfig.hpp:943
    pub fn bottom_solid_layers(mut self, layers: u32) -> Self {
        // PrintConfig.hpp:943
        self.bottom_solid_layers = layers;
        // PrintConfig.hpp:943
        self
    }

    /// Builder method: set wall filament extruder (1-based)
    /// PrintConfig.hpp:993
    pub fn wall_filament(mut self, extruder: usize) -> Self {
        // PrintConfig.hpp:993
        self.wall_filament = extruder;
        // PrintConfig.hpp:993
        self
    }

    /// Builder method: enable/disable ironing
    /// PrintConfig.hpp:980
    pub fn ironing(mut self, enabled: bool) -> Self {
        // PrintConfig.hpp:980
        self.ironing = enabled;
        // PrintConfig.hpp:980
        self
    }

    /// Builder method: enable/disable fuzzy skin
    /// PrintConfig.hpp:969
    pub fn fuzzy_skin(mut self, enabled: bool) -> Self {
        // PrintConfig.hpp:969
        self.fuzzy_skin = enabled;
        // PrintConfig.hpp:969
        self
    }

    /// Get the effective infill extruder (falls back to perimeter extruder)
    /// PrintConfig.hpp:977
    pub fn effective_infill_extruder(&self) -> usize {
        // PrintConfig.hpp:977
        if self.sparse_infill_filament > 0 {
            // PrintConfig.hpp:977
            self.sparse_infill_filament
        } else {
            // PrintConfig.hpp:977
            // PrintConfig.hpp:993
            self.wall_filament
        }
    }

    /// Get the effective solid infill extruder
    /// PrintConfig.hpp:977
    pub fn effective_solid_infill_extruder(&self) -> usize {
        // PrintConfig.hpp:977
        if self.solid_infill_filament > 0 {
            // PrintConfig.hpp:977
            self.solid_infill_filament
        } else {
            self.effective_infill_extruder()
        }
    }

    /// Check if this region has sparse infill
    /// PrintConfig.hpp:965
    pub fn has_sparse_infill(&self) -> bool {
        // PrintConfig.hpp:965
        self.fill_density > 0.0 && self.fill_density < 1.0
    }

    /// Check if this region has solid infill (100% density)
    /// PrintConfig.hpp:965
    pub fn is_solid(&self) -> bool {
        // PrintConfig.hpp:965
        self.fill_density >= 1.0
    }

    /// Check if this region has no infill
    /// PrintConfig.hpp:965
    pub fn is_hollow(&self) -> bool {
        // PrintConfig.hpp:965
        self.fill_density == 0.0
    }
}

impl PrintRegionConfig {
    /// Apply a key-value pair from BambuStudio project_settings JSON.
    /// Returns true if the key was recognized and applied.
    pub fn set_deserialize(&mut self, key: &str, value: &str) -> bool {
        use crate::print_config::{
            parse_bool, parse_f64, parse_float_or_percent, parse_pct, parse_u32,
        };

        match key {
            // === Perimeters ===
            "wall_loops" => {
                if let Some(v) = parse_u32(value) {
                    self.perimeters = v;
                }
                true
            }
            "outer_wall_line_width" => {
                if let Some(v) = parse_f64(value) {
                    self.outer_wall_line_width = v;
                }
                true
            }
            "inner_wall_line_width" => {
                if let Some(v) = parse_f64(value) {
                    self.inner_wall_line_width = v;
                }
                true
            }
            "outer_wall_speed" => {
                if let Some(v) = parse_f64(value) {
                    self.external_perimeter_speed = v;
                }
                true
            }
            "inner_wall_speed" => {
                if let Some(v) = parse_f64(value) {
                    self.perimeter_speed = v;
                }
                true
            }
            "small_perimeter_speed" => {
                if let Some(v) = parse_f64(value) {
                    self.small_perimeter_speed = v;
                }
                true
            }
            "detect_thin_wall" => {
                if let Some(v) = parse_bool(value) {
                    self.thin_walls = v;
                }
                true
            }
            "detect_overhang_wall" => {
                if let Some(v) = parse_bool(value) {
                    self.overhangs = v;
                }
                true
            }
            "extra_perimeters_on_overhangs" => {
                if let Some(v) = parse_bool(value) {
                    self.extra_perimeters = v;
                    self.extra_perimeters_on_overhangs = v;
                }
                true
            }

            // === Infill ===
            "sparse_infill_density" => {
                if let Some(v) = parse_pct(value) {
                    self.fill_density = v;
                }
                true
            }
            "ensure_vertical_shell_thickness" => {
                self.ensure_vertical_shell_thickness = match value {
                    "disabled" => EnsureVerticalThicknessLevel::Disabled,
                    "partial" => EnsureVerticalThicknessLevel::Partial,
                    "enabled" => EnsureVerticalThicknessLevel::Enabled,
                    _ => self.ensure_vertical_shell_thickness,
                };
                true
            }
            "sparse_infill_pattern" => {
                self.fill_pattern = InfillPattern::from_str_bambu(value);
                true
            }
            "top_surface_pattern" => {
                self.top_fill_pattern = InfillPattern::from_str_bambu(value);
                true
            }
            "bottom_surface_pattern" => {
                self.bottom_fill_pattern = InfillPattern::from_str_bambu(value);
                true
            }
            "internal_solid_infill_pattern" => {
                self.solid_fill_pattern = InfillPattern::from_str_bambu(value);
                true
            }
            "locked_skin_infill_pattern" => {
                self.locked_skin_infill_pattern = InfillPattern::from_str_bambu(value);
                true
            }
            "locked_skeleton_infill_pattern" => {
                self.locked_skeleton_infill_pattern = InfillPattern::from_str_bambu(value);
                true
            }
            "fill_multiline" => {
                if let Ok(v) = value.trim().parse::<i32>() {
                    self.fill_multiline = v;
                }
                true
            }
            "infill_shift_step" => {
                if let Some(v) = parse_f64(value) {
                    self.infill_shift_step = v;
                }
                true
            }
            "infill_rotate_step" => {
                if let Some(v) = parse_f64(value) {
                    self.infill_rotate_step = v;
                }
                true
            }
            "symmetric_infill_y_axis" => {
                if let Some(v) = parse_bool(value) {
                    self.symmetric_infill_y_axis = v;
                }
                true
            }
            "sparse_infill_lattice_angle_1" => {
                if let Some(v) = parse_f64(value) {
                    self.sparse_infill_lattice_angle_1 = v;
                }
                true
            }
            "sparse_infill_lattice_angle_2" => {
                if let Some(v) = parse_f64(value) {
                    self.sparse_infill_lattice_angle_2 = v;
                }
                true
            }
            // Percent options: the C++ ConfigOptionPercent stores the raw
            // percent number (e.g. "100" / "15%" -> 100.0 / 15.0).
            "top_surface_density" => {
                if let Some(v) = parse_f64(value) {
                    self.top_surface_density = v;
                }
                true
            }
            "bottom_surface_density" => {
                if let Some(v) = parse_f64(value) {
                    self.bottom_surface_density = v;
                }
                true
            }
            "monotonic_travel_into_wall" => {
                if let Some(v) = parse_f64(value) {
                    self.monotonic_travel_into_wall = v;
                }
                true
            }
            "skin_infill_line_width" => {
                if let Some(v) = parse_f64(value) {
                    self.skin_infill_line_width = v;
                }
                true
            }
            "skeleton_infill_line_width" => {
                if let Some(v) = parse_f64(value) {
                    self.skeleton_infill_line_width = v;
                }
                true
            }
            "skin_infill_density" => {
                if let Some(v) = parse_f64(value) {
                    self.skin_infill_density = v;
                }
                true
            }
            "skeleton_infill_density" => {
                if let Some(v) = parse_f64(value) {
                    self.skeleton_infill_density = v;
                }
                true
            }
            "skin_infill_depth" => {
                if let Some(v) = parse_f64(value) {
                    self.skin_infill_depth = v;
                }
                true
            }
            "infill_lock_depth" => {
                if let Some(v) = parse_f64(value) {
                    self.infill_lock_depth = v;
                }
                true
            }
            "infill_instead_top_bottom_surfaces" => {
                if let Some(v) = parse_bool(value) {
                    self.infill_instead_top_bottom_surfaces = v;
                }
                true
            }
            "infill_direction" => {
                if let Some(v) = parse_f64(value) {
                    self.fill_angle = v;
                }
                true
            }
            "sparse_infill_line_width" => {
                if let Some(v) = parse_f64(value) {
                    self.sparse_infill_line_width = v;
                }
                true
            }
            "internal_solid_infill_line_width" => {
                if let Some(v) = parse_f64(value) {
                    self.internal_solid_infill_line_width = v;
                }
                true
            }
            "top_surface_line_width" => {
                if let Some(v) = parse_f64(value) {
                    self.top_surface_line_width = v;
                }
                true
            }
            "sparse_infill_speed" => {
                if let Some(v) = parse_f64(value) {
                    self.infill_speed = v;
                }
                true
            }
            "internal_solid_infill_speed" => {
                if let Some(v) = parse_f64(value) {
                    self.solid_infill_speed = v;
                }
                true
            }
            "top_surface_speed" => {
                if let Some(v) = parse_f64(value) {
                    self.top_solid_infill_speed = v;
                }
                true
            }
            "infill_wall_overlap" => {
                if let Some(v) = parse_pct(value) {
                    self.infill_overlap = v;
                }
                true
            }
            // "infill_anchor" is the legacy alias handled by the C++
            // composite-key remap (PrintConfig.cpp:6873-6876).
            "sparse_infill_anchor" | "infill_anchor" => {
                if let Some(v) = parse_float_or_percent(value) {
                    self.infill_anchor = v;
                }
                true
            }
            "sparse_infill_anchor_max" | "infill_anchor_max" => {
                if let Some(v) = parse_float_or_percent(value) {
                    self.infill_anchor_max = v;
                }
                true
            }

            // === Solid Layers ===
            "top_shell_layers" => {
                if let Some(v) = parse_u32(value) {
                    self.top_solid_layers = v;
                }
                true
            }
            "bottom_shell_layers" => {
                if let Some(v) = parse_u32(value) {
                    self.bottom_solid_layers = v;
                }
                true
            }
            "top_shell_thickness" => {
                if let Some(v) = parse_f64(value) {
                    self.top_solid_min_thickness = v;
                }
                true
            }
            "bottom_shell_thickness" => {
                if let Some(v) = parse_f64(value) {
                    self.bottom_solid_min_thickness = v;
                }
                true
            }

            // === Bridges ===
            "bridge_speed" => {
                if let Some(v) = parse_f64(value) {
                    self.bridge_speed = v;
                }
                true
            }
            "bridge_flow" => {
                if let Some(v) = parse_f64(value) {
                    self.bridge_flow_ratio = v;
                }
                true
            }
            "bridge_angle" => {
                if let Some(v) = parse_f64(value) {
                    self.bridge_angle = v;
                }
                true
            }

            // === Gap Fill ===
            "gap_infill_speed" => {
                if let Some(v) = parse_f64(value) {
                    self.gap_fill_speed = v;
                }
                true
            }
            "filter_out_gap_fill" => {
                if let Some(v) = parse_f64(value) {
                    self.filter_out_gap_fill = v;
                }
                true
            }

            // === Embedding Wall (InterlockingGenerator) ===
            "embedding_wall_into_infill" => {
                if let Some(v) = parse_bool(value) {
                    self.embedding_wall_into_infill = v;
                }
                true
            }

            // === Seam ===
            "seam_position" => {
                self.seam_position = match value {
                    "aligned" => SeamPosition::Aligned,
                    "random" => SeamPosition::Random,
                    "back" | "rear" => SeamPosition::Rear,
                    "nearest" => SeamPosition::Nearest,
                    _ => self.seam_position,
                };
                true
            }

            // === Scarf Seam (seam slope) ===
            "override_filament_scarf_seam_setting" => {
                if let Some(v) = parse_bool(value) {
                    self.override_filament_scarf_seam_setting = v;
                }
                true
            }
            "seam_slope_type" => {
                self.seam_slope_type = ScarfSeamType::from_str_bambu(value);
                true
            }
            "seam_slope_conditional" => {
                if let Some(v) = parse_bool(value) {
                    self.seam_slope_conditional = v;
                }
                true
            }
            "seam_slope_start_height" => {
                if let Some(v) = parse_float_or_percent(value) {
                    self.seam_slope_start_height = v;
                }
                true
            }
            "seam_slope_gap" => {
                if let Some(v) = parse_float_or_percent(value) {
                    self.seam_slope_gap = v;
                }
                true
            }
            "seam_slope_entire_loop" => {
                if let Some(v) = parse_bool(value) {
                    self.seam_slope_entire_loop = v;
                }
                true
            }
            "seam_slope_min_length" => {
                if let Some(v) = parse_f64(value) {
                    self.seam_slope_min_length = v;
                }
                true
            }
            "seam_slope_steps" => {
                if let Ok(v) = value.trim().parse::<i32>() {
                    self.seam_slope_steps = v;
                }
                true
            }
            "seam_slope_inner_walls" => {
                if let Some(v) = parse_bool(value) {
                    self.seam_slope_inner_walls = v;
                }
                true
            }

            // === Ironing ===
            "ironing_type" => {
                self.ironing = value != "no ironing" && !value.is_empty();
                self.ironing_type = match value {
                    "top" => IroningType::TopSurfaces,
                    "topmost" => IroningType::TopmostOnly,
                    "solid" => IroningType::AllSolid,
                    _ => IroningType::TopSurfaces,
                };
                true
            }
            "ironing_flow" => {
                if let Some(v) = parse_f64(value) {
                    self.ironing_flow_rate = v;
                }
                true
            }
            "ironing_spacing" => {
                if let Some(v) = parse_f64(value) {
                    self.ironing_spacing = v;
                }
                true
            }
            "ironing_speed" => {
                if let Some(v) = parse_f64(value) {
                    self.ironing_speed = v;
                }
                true
            }

            // === Fuzzy Skin ===
            "fuzzy_skin" => {
                self.fuzzy_skin = value == "1"
                    || value == "true"
                    || value == "external"
                    || value == "allwalls"
                    || value == "all";
                self.fuzzy_skin_mode = match value {
                    "allwalls" | "all" => FuzzySkinMode::All,
                    "external" => FuzzySkinMode::External,
                    _ => FuzzySkinMode::None,
                };
                self.fuzzy_skin_type = match value {
                    "external" => FuzzySkinType::External,
                    "all" => FuzzySkinType::All,
                    "allwalls" => FuzzySkinType::AllWalls,
                    _ => FuzzySkinType::None,
                };
                true
            }
            "fuzzy_skin_thickness" => {
                if let Some(v) = parse_f64(value) {
                    self.fuzzy_skin_thickness = v;
                }
                true
            }
            "fuzzy_skin_point_distance" | "fuzzy_skin_point_dist" => {
                if let Some(v) = parse_f64(value) {
                    self.fuzzy_skin_point_distance = v;
                }
                true
            }

            // === Wall Sequence ===
            "wall_sequence" => {
                self.wall_sequence = match value {
                    "outer_wall_first" | "outer/inner" => WallSequence::OuterInner,
                    "inner_outer_inner" | "inner/outer/inner" => WallSequence::InnerOuterInner,
                    _ => WallSequence::InnerOuter, // "inner_outer_first" / "inner/outer"
                };
                true
            }

            _ => false,
        }
    }
}

/// Default trait implementation for PrintRegionConfig
/// PrintConfig.hpp:939
impl Default for PrintRegionConfig {
    // Create default PrintRegionConfig with standard values
    // PrintConfig.hpp:939
    fn default() -> Self {
        // PrintConfig.hpp:939-1050
        Self {
            // Perimeters
            perimeters: 3,
            outer_wall_line_width: 0.4,
            inner_wall_line_width: 0.4,
            external_perimeter_speed: 25.0,
            perimeter_speed: 45.0,
            small_perimeter_speed: 25.0,
            thin_walls: true,
            overhangs: true,
            extra_perimeters: true,
            extra_perimeters_on_overhangs: false,

            // Infill
            fill_density: 0.2,
            fill_pattern: InfillPattern::Grid,
            solid_fill_pattern: InfillPattern::Rectilinear,
            top_fill_pattern: InfillPattern::Rectilinear,
            bottom_fill_pattern: InfillPattern::Rectilinear,
            fill_angle: 45.0,
            sparse_infill_line_width: 0.4,
            internal_solid_infill_line_width: 0.4,
            top_surface_line_width: 0.4,
            infill_speed: 80.0,
            solid_infill_speed: 40.0,
            top_solid_infill_speed: 30.0,
            infill_overlap: 0.25,
            // C++ default: PrintConfig.cpp:3551 ConfigOptionFloatOrPercent(400, true)
            infill_anchor: FloatOrPercent::with(400.0, true),
            // C++ default: PrintConfig.cpp:3579 ConfigOptionFloatOrPercent(20, false)
            infill_anchor_max: FloatOrPercent::with(20.0, false),
            // C++ default: PrintConfig.cpp:2752 ConfigOptionInt(1)
            fill_multiline: 1,
            // C++ default: PrintConfig.cpp:2844 ipCrossZag
            locked_skin_infill_pattern: InfillPattern::CrossZag,
            // C++ default: PrintConfig.cpp:2885 ipZigZag
            locked_skeleton_infill_pattern: InfillPattern::ZigZag,
            // C++ default: PrintConfig.cpp:3423 ConfigOptionFloat(0.4)
            infill_shift_step: 0.4,
            // C++ default: PrintConfig.cpp:3433 ConfigOptionFloat(0)
            infill_rotate_step: 0.0,
            // C++ default: PrintConfig.cpp:3503 ConfigOptionBool(false)
            symmetric_infill_y_axis: false,
            // C++ default: PrintConfig.cpp:3513 ConfigOptionFloat(-45)
            sparse_infill_lattice_angle_1: -45.0,
            // C++ default: PrintConfig.cpp:3523 ConfigOptionFloat(45)
            sparse_infill_lattice_angle_2: 45.0,
            // C++ default: PrintConfig.cpp:1858 ConfigOptionPercent(100)
            top_surface_density: 100.0,
            // C++ default: PrintConfig.cpp:1890 ConfigOptionPercent(100)
            bottom_surface_density: 100.0,
            // C++ default: PrintConfig.cpp:1868 ConfigOptionPercent(0.0)
            monotonic_travel_into_wall: 0.0,
            // C++ default: PrintConfig.cpp:3486 ConfigOptionFloat(0.4)
            skin_infill_line_width: 0.4,
            // C++ default: PrintConfig.cpp:3495 ConfigOptionFloat(0.4)
            skeleton_infill_line_width: 0.4,
            // C++ default: PrintConfig.cpp:3457 ConfigOptionPercent(15)
            skin_infill_density: 15.0,
            // C++ default: PrintConfig.cpp:3445 ConfigOptionPercent(15)
            skeleton_infill_density: 15.0,
            // C++ default: PrintConfig.cpp:3467 ConfigOptionFloat(2.0)
            skin_infill_depth: 2.0,
            // C++ default: PrintConfig.cpp:3477 ConfigOptionFloat(1.0)
            infill_lock_depth: 1.0,
            // C++ default: PrintConfig.cpp:5570 ConfigOptionBool(false)
            infill_instead_top_bottom_surfaces: false,

            // Solid Layers (BambuStudio reference: top_shell_layers = 5)
            top_solid_layers: 5,
            bottom_solid_layers: 3,
            top_solid_min_thickness: 0.0,
            bottom_solid_min_thickness: 0.0,
            // C++ default is evtEnabled (PrintConfig.cpp:1804)
            ensure_vertical_shell_thickness: EnsureVerticalThicknessLevel::Enabled,

            // Bridges
            bridge_speed: 25.0,
            bridge_flow_ratio: 1.0,
            bridge_angle: 0.0,

            // Gap Fill
            gap_fill_enabled: true,
            gap_fill_speed: 20.0,

            // Seam
            seam_position: SeamPosition::Aligned,
            seam_angle_cost: 1.0,
            seam_travel_cost: 1.0,

            // Scarf Seam (seam slope)
            // C++ default: PrintConfig.cpp:4710 (ConfigOptionBool(false))
            override_filament_scarf_seam_setting: false,
            // C++ default: PrintConfig.cpp:4725 (SeamScarfType::None)
            seam_slope_type: ScarfSeamType::None,
            // C++ default: PrintConfig.cpp:4671 (ConfigOptionBool(true))
            seam_slope_conditional: true,
            // C++ default: PrintConfig.cpp:4734 (ConfigOptionFloatOrPercent{10, true})
            seam_slope_start_height: FloatOrPercent::with(10.0, true),
            // C++ default: PrintConfig.cpp:4744 (ConfigOptionFloatOrPercent{0, 0})
            seam_slope_gap: FloatOrPercent::with(0.0, false),
            // C++ default: PrintConfig.cpp:4688 (ConfigOptionBool(false))
            seam_slope_entire_loop: false,
            // C++ default: PrintConfig.cpp:4753 (ConfigOptionFloat{10})
            seam_slope_min_length: 10.0,
            // C++ default: PrintConfig.cpp:4696 (ConfigOptionInt(10))
            seam_slope_steps: 10,
            // C++ default: PrintConfig.cpp:4703 (ConfigOptionBool(true))
            seam_slope_inner_walls: true,

            // Ironing
            ironing: false,
            ironing_type: IroningType::TopSurfaces,
            ironing_flow_rate: 0.15,
            ironing_spacing: 0.1,
            ironing_speed: 15.0,

            // Fuzzy Skin
            fuzzy_skin: false,
            fuzzy_skin_mode: FuzzySkinMode::None,
            fuzzy_skin_thickness: 0.3,
            fuzzy_skin_point_distance: 0.8,
            fuzzy_skin_type: FuzzySkinType::None,
            fuzzy_skin_first_layer: false,
            fuzzy_skin_noise_type: NoiseType::Classic,
            fuzzy_skin_scale: 1.0,
            fuzzy_skin_octaves: 4,
            fuzzy_skin_persistence: 0.5,
            fuzzy_skin_displacement_mode: FuzzySkinDisplacementMode::Displacement,

            // Wall Generation Mode
            wall_generator_mode: WallGeneratorMode::Classic,
            wall_sequence: WallSequence::InnerOuter,

            // Misc
            region_id: 0,
            wall_filament: 1, // 1-based in C++
            sparse_infill_filament: 1,
            solid_infill_filament: 1,

            // C++ default: PrintConfig.cpp:3187 (ConfigOptionFloat(0))
            filter_out_gap_fill: 0.0,
            // C++ default: PrintConfig.cpp:4218 (ConfigOptionBool(false))
            embedding_wall_into_infill: false,
        }
    }
}

/// Display trait implementation for PrintRegionConfig
/// PrintConfig.hpp:939
impl fmt::Display for PrintRegionConfig {
    // Format PrintRegionConfig for display
    // PrintConfig.hpp:939
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "PrintRegionConfig(region={}, perimeters={}, infill={:.0}%)",
            self.region_id,
            self.perimeters,
            self.fill_density * 100.0
        )
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
/// Ironing type - which surfaces to iron
/// PrintConfig.hpp:980
pub enum IroningType {
    /// Iron all top surfaces
    /// PrintConfig.hpp:980
    #[default]
    TopSurfaces,
    /// Iron only the topmost surface
    /// PrintConfig.hpp:980
    TopmostOnly,
    /// Iron all solid surfaces
    /// PrintConfig.hpp:980
    AllSolid,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
/// Fuzzy skin mode
/// PrintConfig.hpp:969
pub enum FuzzySkinMode {
    /// No fuzzy skin
    /// PrintConfig.hpp:969
    #[default]
    None,
    /// Fuzzy skin on external perimeters only
    /// PrintConfig.hpp:969
    External,
    /// Fuzzy skin on all perimeters
    /// PrintConfig.hpp:969
    All,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
/// Fuzzy skin type (which walls/contours/holes get fuzzified).
/// PrintConfig.hpp:46-52 (enum class FuzzySkinType)
pub enum FuzzySkinType {
    /// PrintConfig.hpp:47
    #[default]
    None,
    /// PrintConfig.hpp:48
    External,
    /// PrintConfig.hpp:49
    All,
    /// PrintConfig.hpp:50
    AllWalls,
    /// PrintConfig.hpp:51
    DisabledFuzzy,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
/// Noise function used to drive the fuzzy-skin displacement.
/// PrintConfig.hpp:54-60 (enum class NoiseType)
pub enum NoiseType {
    /// Classic uniform random noise (backward compatible).
    /// PrintConfig.hpp:55
    #[default]
    Classic,
    /// PrintConfig.hpp:56
    Perlin,
    /// PrintConfig.hpp:57
    Billow,
    /// PrintConfig.hpp:58
    RidgedMulti,
    /// PrintConfig.hpp:59
    Voronoi,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
/// How the fuzzy displacement is applied to an extrusion line.
/// PrintConfig.hpp:62-66 (enum class FuzzySkinMode)
pub enum FuzzySkinDisplacementMode {
    /// Move the centerline perpendicular to the path.
    /// PrintConfig.hpp:63
    #[default]
    Displacement,
    /// Vary the extrusion width instead of moving the centerline.
    /// PrintConfig.hpp:64
    Extrusion,
    /// Combine displacement and width variation.
    /// PrintConfig.hpp:65
    Combined,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
/// Whether to add solid infill near sloping surfaces to guarantee the vertical
/// shell thickness (top+bottom solid layers). When != Disabled, the shell work
/// is performed by discover_vertical_shells() and discover_horizontal_shells()
/// is skipped — see PrintObject.cpp:3398.
/// PrintConfig.hpp:83-87 (EnsureVerticalThicknessLevel)
pub enum EnsureVerticalThicknessLevel {
    /// PrintConfig.hpp:84 (evtDisabled)
    Disabled,
    /// PrintConfig.hpp:85 (evtPartial)
    Partial,
    /// PrintConfig.hpp:86 (evtEnabled) — C++ default (PrintConfig.cpp:1804)
    #[default]
    Enabled,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_print_region_config_default() {
        let config = PrintRegionConfig::default();
        assert_eq!(config.perimeters, 3);
        assert!((config.fill_density - 0.2).abs() < 1e-6);
        assert_eq!(config.fill_pattern, InfillPattern::Grid);
        assert_eq!(config.top_solid_layers, 5);
        assert_eq!(config.bottom_solid_layers, 3);
    }

    #[test]
    fn test_print_region_config_builder() {
        let config = PrintRegionConfig::new()
            .perimeters(5)
            .fill_density(0.4)
            .fill_pattern(InfillPattern::Gyroid)
            .top_solid_layers(6)
            .wall_filament(1);

        assert_eq!(config.perimeters, 5);
        assert!((config.fill_density - 0.4).abs() < 1e-6);
        assert_eq!(config.fill_pattern, InfillPattern::Gyroid);
        assert_eq!(config.top_solid_layers, 6);
        assert_eq!(config.wall_filament, 1);
    }

    #[test]
    fn test_print_region_config_with_region_id() {
        let config = PrintRegionConfig::with_region_id(5);
        assert_eq!(config.region_id, 5);
    }

    #[test]
    fn test_effective_extruders() {
        let mut config = PrintRegionConfig::default();
        config.wall_filament = 1;
        config.sparse_infill_filament = 0;
        config.solid_infill_filament = 0;

        assert_eq!(config.effective_infill_extruder(), 1);
        assert_eq!(config.effective_solid_infill_extruder(), 1);

        config.sparse_infill_filament = 2;
        assert_eq!(config.effective_infill_extruder(), 2);
        assert_eq!(config.effective_solid_infill_extruder(), 2);

        config.solid_infill_filament = 3;
        assert_eq!(config.effective_solid_infill_extruder(), 3);
    }

    #[test]
    fn test_infill_classification() {
        let mut config = PrintRegionConfig::default();

        config.fill_density = 0.0;
        assert!(config.is_hollow());
        assert!(!config.has_sparse_infill());
        assert!(!config.is_solid());

        config.fill_density = 0.5;
        assert!(!config.is_hollow());
        assert!(config.has_sparse_infill());
        assert!(!config.is_solid());

        config.fill_density = 1.0;
        assert!(!config.is_hollow());
        assert!(!config.has_sparse_infill());
        assert!(config.is_solid());
    }

    #[test]
    fn test_ironing_type_default() {
        assert_eq!(IroningType::default(), IroningType::TopSurfaces);
    }

    #[test]
    fn test_fuzzy_skin_mode_default() {
        assert_eq!(FuzzySkinMode::default(), FuzzySkinMode::None);
    }
}
