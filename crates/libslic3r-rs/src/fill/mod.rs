//! Fill module - Infill pattern generation
//!
//! This module provides infill pattern generation for 3D printing,
//! corresponding to BambuStudio's Fill/ directory.
//!
//! # Overview
//!
//! Infill patterns fill the interior space of a print with various geometric
//! patterns to provide structural support while minimizing material usage.
//!
//! # C++ Reference
//!
//! - `libslic3r/Fill/` directory
//! - `libslic3r/Fill/Fill.cpp` - Base fill infrastructure
//! - `libslic3r/Fill/FillRectilinear.cpp` - Rectilinear/grid patterns
//! - `libslic3r/Fill/FillAdaptive.cpp` - Adaptive density infill
//! - `libslic3r/Fill/FillGyroid.cpp` - Gyroid mathematical surface
//! - `libslic3r/Fill/Fill3DHoneycomb.cpp` - 3D honeycomb pattern
//!
//! # Structure
//!
//! This module mirrors the C++ Fill/ directory structure exactly:
//! - Fill.cpp → fill_base.rs (core infrastructure in mod.rs for now)
//! - FillRectilinear.cpp → fill_rectilinear.rs
//! - FillAdaptive.cpp → fill_adaptive.rs
//! - Fill3DHoneycomb.cpp → fill3_d_honeycomb.rs
//! - FillCrossHatch.cpp → fill_cross_hatch.rs
//! - FillFloatingConcentric.cpp → fill_floating_concentric.rs
//! - FillPlanePath.cpp → fill_plane_path.rs
//! - FillConcentric.cpp → fill_concentric.rs (TODO)
//! - FillGyroid.cpp → fill_gyroid.rs (TODO)
//! - FillHoneycomb.cpp → fill_honeycomb.rs (TODO)
//! - FillLine.cpp → fill_line.rs
//! - FillLightning.cpp → fill_lightning.rs (delegating to lightning/ subdir)
//!
//! NOTE: The fill_base.rs content is currently in this mod.rs file for convenience.
//! It contains the main pattern generation logic from Fill.cpp and FillBase.cpp.

// C++ Fill/ directory files (exact 1:1 mapping)
pub mod fill3_d_honeycomb;
pub mod fill_adaptive;
pub mod fill_concentric;
pub mod fill_concentric_internal;
pub mod fill_cross_hatch;
pub mod fill_floating_concentric;
pub mod fill_gyroid;
pub mod fill_honeycomb;
pub mod fill_lightning;
pub mod fill_line;
pub mod fill_plane_path;
pub mod fill_rectilinear;

// Lightning subdirectory (matches C++ Fill/Lightning/)
pub mod lightning;

// Re-export from fill_adaptive
pub use fill_adaptive::{generate_adaptive_infill, AdaptiveInfillConfig, CubeProperties, Octree};

// Re-export from fill3_d_honeycomb
pub use fill3_d_honeycomb::Honeycomb3DConfig;

// Re-export from fill_gyroid
pub use fill_gyroid::{generate_gyroid_infill, GyroidConfig};

// Re-export from fill_cross_hatch
pub use fill_cross_hatch::CrossHatchConfig;

// Re-export from fill_plane_path (space-filling curves)
pub use fill_plane_path::{PlanPathConfig, PlanPathPattern};

// Re-export from fill_floating_concentric
pub use fill_floating_concentric::FloatingConcentricConfig;

use crate::extrusion_entity::ExtrusionRole;
use crate::flow::Flow;
use crate::geometry::{ExPolygon, Point, Polygon, Polyline};
use crate::surface::{Surface, SurfaceType};
use crate::{Coord, CoordF, Result};
use std::collections::BTreeSet;

/// Configuration for infill generation
/// Corresponds to Fill.hpp infill parameters
#[derive(Debug, Clone)]
pub struct InfillConfig {
    /// Pattern type
    pub pattern: InfillPattern,
    /// Line spacing in mm
    pub line_spacing: CoordF,
    /// Fill angle in degrees
    pub angle: CoordF,
    /// Angle increment per layer
    pub angle_increment: CoordF,
    /// Fill density (0.0 to 1.0)
    pub density: CoordF,
    /// Extrusion width in mm
    pub extrusion_width: CoordF,
    /// Overlap between infill and perimeters (fraction of line width)
    pub overlap: CoordF,
    /// Whether to connect infill lines
    pub connect_infill: bool,
    /// Maximum length of a perimeter segment linking two infill lines, in
    /// scaled units. 0 means unlimited. C++: Fill::link_max_length
    /// (Fill.cpp:683-695: 0 for density <= 80%, 3*spacing otherwise).
    pub link_max_length: Coord,
}

impl Default for InfillConfig {
    fn default() -> Self {
        Self {
            pattern: InfillPattern::default(),
            line_spacing: 0.4,
            angle: 45.0,
            angle_increment: 90.0,
            density: 0.2,
            extrusion_width: 0.4,
            overlap: 0.25,
            connect_infill: true,
            link_max_length: 0,
        }
    }
}

/// Single infill path - either a line or a loop
/// Corresponds to Fill.cpp path output
#[derive(Debug, Clone)]
pub enum InfillPath {
    /// Open polyline path
    Line(Polyline),
    /// Closed polygon loop
    Loop(Polygon),
}

/// Result of infill generation
/// Corresponds to Fill.cpp output structure
#[derive(Debug, Clone, Default)]
pub struct InfillResult {
    /// Generated infill paths
    pub paths: Vec<InfillPath>,
}

/// Trait for infill generators
/// Corresponds to Fill.hpp FillBase interface
pub trait InfillGenerator {
    /// Generate infill for the given regions
    fn generate(&self, regions: &[ExPolygon], config: &InfillConfig) -> Result<InfillResult>;
}

/// Infill pattern types
/// Fill.hpp:30-50
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Default)]
pub enum InfillPattern {
    /// Parallel lines in one direction
    /// Fill.hpp:31
    #[default]
    Rectilinear,

    /// Crossing lines at 90° (alternating layers)
    /// Fill.hpp:32
    Grid,

    /// Concentric loops following perimeter
    /// Fill.hpp:33
    Concentric,

    /// 2D hexagonal honeycomb
    /// Fill.hpp:34
    Honeycomb,

    /// 3D honeycomb (stacked layers)
    /// Fill.hpp:35
    Honeycomb3D,

    /// Gyroid mathematical surface
    /// Fill.hpp:36
    Gyroid,

    /// Hilbert space-filling curve
    /// Fill.hpp:37
    Hilbert,

    /// Archimedean chords
    /// Fill.hpp:38
    ArchimedeanChords,

    /// Octagram spiral
    /// Fill.hpp:39
    OctagramSpiral,

    /// Adaptive density based on geometry
    /// Fill.hpp:40
    Adaptive,

    /// Lightning tree-like pattern
    /// Fill.hpp:41
    Lightning,

    /// Cross-hatch pattern
    /// Fill.hpp:42
    CrossHatch,

    /// Floating concentric
    /// Fill.hpp:43
    FloatingConcentric,
}

/// Convert print-config infill patterns to Fill.cpp pattern family.
///
/// This keeps the translation local to the fill module, mirroring the C++
/// flow where `group_fills()` derives a `SurfaceFillParams::pattern` from the
/// region config before selecting a concrete Fill implementation.
impl From<crate::print_config::InfillPattern> for InfillPattern {
    fn from(pattern: crate::print_config::InfillPattern) -> Self {
        match pattern {
            crate::print_config::InfillPattern::Rectilinear => InfillPattern::Rectilinear,
            crate::print_config::InfillPattern::Grid => InfillPattern::Grid,
            crate::print_config::InfillPattern::Honeycomb => InfillPattern::Honeycomb,
            crate::print_config::InfillPattern::Honeycomb3D => InfillPattern::Honeycomb3D,
            crate::print_config::InfillPattern::Gyroid => InfillPattern::Gyroid,
            crate::print_config::InfillPattern::Concentric => InfillPattern::Concentric,
            crate::print_config::InfillPattern::CrossHatch => InfillPattern::CrossHatch,
            crate::print_config::InfillPattern::Lightning => InfillPattern::Lightning,
            crate::print_config::InfillPattern::AdaptiveCubic => InfillPattern::Adaptive,
            // Keep currently unsupported print-config patterns on the nearest
            // Fill.cpp-compatible fallback instead of dropping geometry.
            crate::print_config::InfillPattern::Triangles
            | crate::print_config::InfillPattern::Cubic => InfillPattern::Grid,
            // Monotonic and variants map to Rectilinear (monotonic fill order
            // is handled at a higher level, the base pattern is rectilinear).
            crate::print_config::InfillPattern::Monotonic
            | crate::print_config::InfillPattern::MonotonicLine
            | crate::print_config::InfillPattern::AlignedRectilinear => InfillPattern::Rectilinear,
            // Decorative / specialty patterns fall back to nearest equivalent
            crate::print_config::InfillPattern::HilbertCurve
            | crate::print_config::InfillPattern::ArchimedeanChords
            | crate::print_config::InfillPattern::OctagramSpiral => InfillPattern::Concentric,
            crate::print_config::InfillPattern::SupportCubic => InfillPattern::Grid,
            crate::print_config::InfillPattern::ZigZag => InfillPattern::Rectilinear,
        }
    }
}

/// Parameters for surface fill generation
/// Fill.cpp:23-140
#[derive(Debug, Clone, PartialEq)]
pub struct SurfaceFillParams {
    /// Fill.cpp:26
    /// C++: unsigned int extruder
    pub extruder: u32,

    /// Fill.cpp:28
    /// C++: InfillPattern pattern
    pub pattern: InfillPattern,

    /// Fill.cpp:30
    /// C++: InfillPattern skin_pattern
    pub skin_pattern: InfillPattern,

    /// Fill.cpp:31
    /// C++: InfillPattern skeleton_pattern
    pub skeleton_pattern: InfillPattern,

    /// Fill.cpp:35
    /// C++: coordf_t spacing
    pub spacing: f64,

    /// Fill.cpp:37
    /// C++: coordf_t overlap
    pub overlap: f64,

    /// Fill.cpp:39
    /// C++: float angle
    pub angle: f32,

    /// Fill.cpp:41
    /// C++: bool bridge
    pub bridge: bool,

    /// Fill.cpp:43
    /// C++: float bridge_angle
    pub bridge_angle: f32,

    /// Fill.cpp:46
    /// C++: float density
    pub density: f32,

    /// Fill.cpp:47
    /// C++: int multiline
    pub multiline: i32,

    /// Fill.cpp:52
    /// C++: float anchor_length
    pub anchor_length: f32,

    /// Fill.cpp:53
    /// C++: float anchor_length_max
    pub anchor_length_max: f32,

    /// Fill.cpp:57
    /// C++: Flow flow
    pub flow: Flow,

    /// Fill.cpp:60
    /// C++: ExtrusionRole extrusion_role
    pub extrusion_role: ExtrusionRole,

    /// Fill.cpp:65
    /// C++: size_t idx
    pub idx: usize,

    /// Fill.cpp:67
    /// C++: float sparse_infill_speed
    pub sparse_infill_speed: f32,

    /// Fill.cpp:68
    /// C++: float top_surface_speed
    pub top_surface_speed: f32,

    /// Fill.cpp:69
    /// C++: float solid_infill_speed
    pub solid_infill_speed: f32,

    /// Fill.cpp:70
    /// C++: float infill_shift_step
    pub infill_shift_step: f32,

    /// Fill.cpp:71
    /// C++: float infill_rotate_step
    pub infill_rotate_step: f32,

    /// Fill.cpp:72
    /// C++: bool symmetric_infill_y_axis
    pub symmetric_infill_y_axis: bool,

    /// Fill.cpp:75
    /// C++: float lattice_angle_1
    pub lattice_angle_1: f32,

    /// Fill.cpp:76
    /// C++: float lattice_angle_2
    pub lattice_angle_2: f32,
}

/// Fill.cpp:78-112
/// C++: bool operator<(const SurfaceFillParams &rhs) const
impl Ord for SurfaceFillParams {
    fn cmp(&self, rhs: &Self) -> std::cmp::Ordering {
        use std::cmp::Ordering;

        // Fill.cpp:82
        macro_rules! return_compare_non_equal {
            ($field:ident) => {
                if self.$field != rhs.$field {
                    return self.$field.cmp(&rhs.$field);
                }
            };
        }

        // Fill.cpp:83-110
        return_compare_non_equal!(extruder);

        // Pattern comparison (enum implements Ord now)
        if self.pattern != rhs.pattern {
            return self.pattern.cmp(&rhs.pattern);
        }

        return_compare_non_equal!(bridge);

        // Float comparisons
        if (self.spacing - rhs.spacing).abs() > 1e-9 {
            return if self.spacing < rhs.spacing {
                Ordering::Less
            } else {
                Ordering::Greater
            };
        }
        if (self.density - rhs.density).abs() > 1e-9 {
            return if self.density < rhs.density {
                Ordering::Less
            } else {
                Ordering::Greater
            };
        }
        if (self.angle - rhs.angle).abs() > 1e-9 {
            return if self.angle < rhs.angle {
                Ordering::Less
            } else {
                Ordering::Greater
            };
        }
        if (self.overlap - rhs.overlap).abs() > 1e-9 {
            return if self.overlap < rhs.overlap {
                Ordering::Less
            } else {
                Ordering::Greater
            };
        }

        // ExtrusionRole comparison
        if self.extrusion_role != rhs.extrusion_role {
            return (self.extrusion_role as u32).cmp(&(rhs.extrusion_role as u32));
        }

        Ordering::Equal
    }
}

impl PartialOrd for SurfaceFillParams {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl Eq for SurfaceFillParams {}

/// Surface to fill with parameters
/// Fill.cpp:142-152
#[derive(Debug, Clone)]
pub struct SurfaceFill {
    /// Fill.cpp:143
    /// C++: SurfaceFill(const SurfaceFillParams &params) : params(params) {}

    /// Fill.cpp:145
    /// C++: size_t region_id
    pub region_id: usize,

    /// Fill.cpp:146
    /// C++: Surface surface
    pub surface: Surface,

    /// Fill.cpp:147
    /// C++: ExPolygons expolygons
    pub expolygons: Vec<ExPolygon>,

    /// Fill.cpp:148
    /// C++: SurfaceFillParams params
    pub params: SurfaceFillParams,

    /// Fill.cpp:150
    /// C++: std::vector<size_t> region_id_group
    pub region_id_group: Vec<usize>,

    /// Fill.cpp:151
    /// C++: ExPolygons no_overlap_expolygons
    pub no_overlap_expolygons: Vec<ExPolygon>,
}

impl SurfaceFill {
    /// Fill.cpp:143
    /// C++: SurfaceFill(const SurfaceFillParams &params) : params(params) {}
    pub fn new(params: SurfaceFillParams) -> Self {
        Self {
            region_id: usize::MAX,
            surface: Surface::default(),
            expolygons: Vec::new(),
            params,
            region_id_group: Vec::new(),
            no_overlap_expolygons: Vec::new(),
        }
    }
}

/// Group surfaces by fill parameters
/// Fill.cpp:164-547
/// C++: std::vector<SurfaceFill> group_fills(const Layer &layer, LockRegionParam &lock_param)
pub fn group_fills(
    layer: &crate::layer::Layer,
    region_configs: &[crate::region_config::PrintRegionConfig],
    _lock_param: &mut LockRegionParam,
) -> Result<Vec<SurfaceFill>> {
    /// Fill.cpp:166
    /// C++: std::vector<SurfaceFill> surface_fills
    let mut surface_fills: Vec<SurfaceFill> = Vec::new();

    /// Fill.cpp:168
    /// C++: std::set<SurfaceFillParams> set_surface_params
    let mut set_surface_params: BTreeSet<SurfaceFillParams> = BTreeSet::new();

    /// Fill.cpp:169
    /// C++: std::vector<std::vector<const SurfaceFillParams*>> region_to_surface_params
    let mut region_to_surface_params: Vec<Vec<Option<usize>>> =
        vec![Vec::new(); layer.region_count()];

    /// Fill.cpp:170
    /// C++: bool has_internal_voids = false
    let mut _has_internal_voids = false;

    // Fill.cpp:189-329
    // Populate region_to_surface_params from layer regions
    for region_id in 0..layer.region_count() {
        let region = layer.get_region(region_id).unwrap();
        region_to_surface_params[region_id] = vec![None; region.fill_surfaces.surfaces.len()];

        // Fill.cpp:196-318
        for (surface_idx, surface) in region.fill_surfaces.surfaces.iter().enumerate() {
            // Fill.cpp:198
            if surface.surface_type == SurfaceType::InternalVoid {
                _has_internal_voids = true;
                continue;
            }

            // Build params for this surface
            // Fill.cpp:200-318
            let region_config = region_configs
                .get(region_id)
                .cloned()
                .unwrap_or_else(crate::region_config::PrintRegionConfig::default);

            let extrusion_role = if surface.is_top() {
                /// Fill.cpp:245-246
                /// C++: role = erTopSolidInfill
                ExtrusionRole::TopSolidInfill
            } else if surface.is_bottom() {
                /// Fill.cpp:247-248
                /// C++: role = erBottomSurface
                ExtrusionRole::BottomSurface
            } else if surface.is_solid() {
                /// Fill.cpp:249-250
                /// C++: role = erSolidInfill
                ExtrusionRole::SolidInfill
            } else {
                /// Fill.cpp:251-252
                /// C++: role = erInternalInfill
                ExtrusionRole::InternalInfill
            };

            let is_bridge = surface.is_bridge();

            let extruder = match extrusion_role {
                ExtrusionRole::InternalInfill => region_config.sparse_infill_filament,
                ExtrusionRole::SolidInfill | ExtrusionRole::TopSolidInfill => {
                    region_config.solid_infill_filament
                }
                _ => region_config.wall_filament,
            };

            let mut pattern = region_config.fill_pattern.into();
            let mut density = (region_config.fill_density * 100.0) as f32;

            if surface.is_solid() {
                density = 100.0;
                pattern = if surface.is_top() {
                    region_config.top_fill_pattern.into()
                } else if surface.is_bottom() {
                    region_config.bottom_fill_pattern.into()
                } else {
                    region_config.solid_fill_pattern.into()
                };
            } else if density <= 0.0 {
                continue;
            }

            let flow_role = if is_bridge {
                crate::flow::FlowRole::SolidInfill
            } else {
                match extrusion_role {
                    ExtrusionRole::InternalInfill => crate::flow::FlowRole::Infill,
                    ExtrusionRole::SolidInfill => crate::flow::FlowRole::SolidInfill,
                    ExtrusionRole::TopSolidInfill | ExtrusionRole::BottomSurface => {
                        crate::flow::FlowRole::TopSolidInfill
                    }
                    _ => crate::flow::FlowRole::Infill,
                }
            };

            let config_width = match flow_role {
                crate::flow::FlowRole::Infill => region_config.sparse_infill_line_width,
                crate::flow::FlowRole::SolidInfill => {
                    region_config.internal_solid_infill_line_width
                }
                crate::flow::FlowRole::TopSolidInfill => region_config.top_surface_line_width,
                _ => region_config.sparse_infill_line_width,
            };

            let layer_height = region
                .fill_surfaces
                .surfaces
                .get(surface_idx)
                .map(|s| if s.thickness > 0.0 { s.thickness } else { 0.2 })
                .unwrap_or(0.2);

            let flow = Flow::new_from_config_width(flow_role, config_width, 0.4, layer_height)?;

            let spacing = if surface.is_solid() || is_bridge {
                flow.spacing()
            } else {
                Flow::new_from_config_width(
                    crate::flow::FlowRole::Infill,
                    region_config.sparse_infill_line_width,
                    0.4,
                    0.2,
                )?
                .spacing()
            };

            let (anchor_length, anchor_length_max) = if surface.is_solid() || is_bridge {
                (1000.0_f32, 1000.0_f32)
            } else {
                let anchor = region_config.infill_anchor as f32;
                let anchor_max = region_config.infill_anchor_max as f32;
                (anchor.min(anchor_max), anchor_max)
            };

            let params = SurfaceFillParams {
                extruder: extruder as u32,
                pattern,
                skin_pattern: region_config.top_fill_pattern.into(),
                skeleton_pattern: region_config.solid_fill_pattern.into(),
                spacing,
                overlap: region_config.infill_overlap,
                angle: region_config.fill_angle as f32,
                bridge: is_bridge,
                bridge_angle: surface.bridge_angle.unwrap_or(region_config.bridge_angle) as f32,
                density,
                multiline: 1,
                anchor_length,
                anchor_length_max,
                flow,
                extrusion_role,
                idx: 0,
                sparse_infill_speed: region_config.infill_speed as f32,
                top_surface_speed: region_config.top_solid_infill_speed as f32,
                solid_infill_speed: region_config.solid_infill_speed as f32,
                infill_shift_step: 0.0,
                infill_rotate_step: 0.0,
                symmetric_infill_y_axis: false,
                lattice_angle_1: 0.0,
                lattice_angle_2: 0.0,
            };

            // Fill.cpp:318-325
            // Find or insert params in set
            let params_idx =
                if let Some(existing_idx) = set_surface_params.iter().position(|p| p == &params) {
                    existing_idx
                } else {
                    let idx = set_surface_params.len();
                    set_surface_params.insert(params);
                    idx
                };

            region_to_surface_params[region_id][surface_idx] = Some(params_idx);
        }
    }

    // Fill.cpp:327-330
    // Create surface_fills from params
    surface_fills.reserve(set_surface_params.len());
    for params in set_surface_params {
        let mut params = params;
        params.idx = surface_fills.len();
        surface_fills.push(SurfaceFill::new(params));
    }

    // Fill.cpp:332-358
    // Populate surface_fills with actual surfaces
    for region_id in 0..layer.region_count() {
        let region = layer.get_region(region_id).unwrap();

        for (surface_idx, surface) in region.fill_surfaces.surfaces.iter().enumerate() {
            if surface.surface_type == SurfaceType::InternalVoid {
                continue;
            }

            if let Some(Some(params_idx)) = region_to_surface_params[region_id].get(surface_idx) {
                let fill = &mut surface_fills[*params_idx];

                // Fill.cpp:338-356
                if fill.region_id == usize::MAX {
                    fill.region_id = region_id;
                    fill.surface = surface.clone();
                    fill.expolygons.push(surface.expolygon.clone());
                    fill.region_id_group.push(region_id);
                    fill.no_overlap_expolygons = region.fill_no_overlap_expolygons.clone();
                } else {
                    fill.expolygons.push(surface.expolygon.clone());
                    if !fill.region_id_group.contains(&region_id) {
                        fill.region_id_group.push(region_id);
                        // TODO: union fill.no_overlap_expolygons with region.fill_no_overlap_expolygons
                    }
                }
            }
        }
    }

    Ok(surface_fills)
}

/// Lock region parameters (for locked zag pattern)
/// FillBase.hpp:47-53
#[derive(Debug, Clone, Default)]
pub struct LockRegionParam {
    // TODO: Port full structure from FillBase.hpp:47-53
}

/// Main infill generation function
/// Fill.cpp:100-200
pub fn generate_infill(
    pattern: InfillPattern,
    fill_area: &[ExPolygon],
    line_spacing: CoordF,
    angle: CoordF,
) -> Result<Vec<Polyline>> {
    match pattern {
        InfillPattern::Rectilinear => {
            // FillRectilinear.cpp - parallel lines infill
            use crate::geometry::BoundingBox;
            let mut all_polylines = Vec::new();
            for expoly in fill_area {
                let mut bb = BoundingBox::default();
                for pt in &expoly.contour.points {
                    bb.merge_point(*pt);
                }
                let spacing_scaled = (line_spacing * 1e6) as Coord;
                if spacing_scaled <= 0 {
                    continue;
                }
                let cos_a = angle.cos();
                let sin_a = angle.sin();
                let mut x = bb.min.x;
                while x <= bb.max.x {
                    // Generate a vertical line and rotate by angle
                    let p1 = Point::new(
                        (x as f64 * cos_a - bb.min.y as f64 * sin_a) as Coord,
                        (x as f64 * sin_a + bb.min.y as f64 * cos_a) as Coord,
                    );
                    let p2 = Point::new(
                        (x as f64 * cos_a - bb.max.y as f64 * sin_a) as Coord,
                        (x as f64 * sin_a + bb.max.y as f64 * cos_a) as Coord,
                    );
                    all_polylines.push(Polyline::from_points(vec![p1, p2]));
                    x += spacing_scaled;
                }
            }
            Ok(all_polylines)
        }
        InfillPattern::Grid => {
            // Grid is rectilinear in two perpendicular directions
            let mut lines1 =
                generate_infill(InfillPattern::Rectilinear, fill_area, line_spacing, angle)?;
            let lines2 = generate_infill(
                InfillPattern::Rectilinear,
                fill_area,
                line_spacing,
                angle + std::f64::consts::FRAC_PI_2,
            )?;
            lines1.extend(lines2);
            Ok(lines1)
        }
        InfillPattern::Adaptive => {
            // FillAdaptive.cpp - adaptive density infill
            let config = fill_adaptive::AdaptiveInfillConfig {
                line_spacing,
                ..Default::default()
            };
            fill_adaptive::generate_adaptive_infill(fill_area, &config)
        }
        InfillPattern::Honeycomb3D => {
            // Fill3DHoneycomb.cpp - 3D honeycomb infill
            let mut all_polylines = Vec::new();
            for expoly in fill_area {
                let result = fill3_d_honeycomb::generate_honeycomb_3d(
                    expoly,
                    0.0, // z height - caller should provide
                    0.2, // layer height default
                    0.2, // density default
                    line_spacing,
                );
                all_polylines.extend(result.polylines);
            }
            Ok(all_polylines)
        }
        InfillPattern::CrossHatch => {
            // FillCrossHatch.cpp - cross hatch infill
            let result = fill_cross_hatch::generate_cross_hatch_with_angle(
                fill_area,
                0.0, // z height - caller should provide
                line_spacing,
                0.2, // density default
                angle,
            );
            Ok(result.polylines)
        }
        InfillPattern::FloatingConcentric => {
            // FillFloatingConcentric.cpp - concentric with floating detection
            // This pattern needs floating area info; fall back to regular concentric
            Ok(fill_concentric::generate_concentric_infill(
                fill_area,
                line_spacing,
            ))
        }
        InfillPattern::Hilbert => {
            // FillPlanePath.cpp - Hilbert curve space-filling infill
            let mut all_polylines = Vec::new();
            for expoly in fill_area {
                let result = fill_plane_path::generate_hilbert_curve(expoly, 0.2, line_spacing);
                all_polylines.extend(result.polylines);
            }
            Ok(all_polylines)
        }
        InfillPattern::ArchimedeanChords => {
            // FillPlanePath.cpp - Archimedean chords space-filling infill
            let mut all_polylines = Vec::new();
            for expoly in fill_area {
                let result =
                    fill_plane_path::generate_archimedean_chords(expoly, 0.2, line_spacing);
                all_polylines.extend(result.polylines);
            }
            Ok(all_polylines)
        }
        InfillPattern::OctagramSpiral => {
            // FillPlanePath.cpp - Octagram spiral space-filling infill
            let mut all_polylines = Vec::new();
            for expoly in fill_area {
                let result = fill_plane_path::generate_octagram_spiral(expoly, 0.2, line_spacing);
                all_polylines.extend(result.polylines);
            }
            Ok(all_polylines)
        }
        InfillPattern::Concentric => {
            // FillConcentric.cpp - concentric offset loops
            Ok(fill_concentric::generate_concentric_infill(
                fill_area,
                line_spacing,
            ))
        }
        InfillPattern::Honeycomb => {
            // FillHoneycomb.cpp - 2D hexagonal honeycomb infill
            let mut filler = fill_honeycomb::FillHoneycomb::new(line_spacing);
            let mut params = FillParams::new();
            params.density = 0.2; // default density
            // dont_connect() so we just chain the lines in this simplified helper.
            params.anchor_length_max = 0.0;
            let direction = (angle as f32, Point::new(0, 0));
            let mut all_polylines = Vec::new();
            for expoly in fill_area {
                filler.fill_surface_single(
                    &params,
                    1,
                    &direction,
                    expoly.clone(),
                    &mut all_polylines,
                );
            }
            Ok(all_polylines)
        }
        InfillPattern::Gyroid => {
            // FillGyroid.cpp - Gyroid mathematical surface infill
            use crate::geometry::BoundingBox;

            // Compute bounding box of all fill regions
            let mut bb = BoundingBox::default();
            for expoly in fill_area {
                for pt in &expoly.contour.points {
                    bb.merge_point(*pt);
                }
            }

            let config = fill_gyroid::GyroidConfig {
                z: 0.0, // caller should set Z; using 0 as default
                spacing: line_spacing,
                density: 0.2, // default density
                angle,
            };
            Ok(fill_gyroid::generate_gyroid_infill(&config, bb.min, bb.max))
        }
        InfillPattern::Lightning => {
            // FillLightning.cpp + Lightning/ - tree-based sparse infill
            // Lightning infill requires layer-by-layer tree generation context
            // that is not available in this single-layer API. Return empty
            // polylines; the full lightning pipeline should be invoked through
            // the fill_lightning::Filler (FillLightning.cpp) +
            // lightning::generator::Generator (Lightning/Generator.cpp) API.
            Ok(Vec::new())
        }
    }
}

// ============================================================================
// FillBase infrastructure functions ported from C++ FillBase.cpp
// ============================================================================

/// Parameters for fill generation, matching C++ FillParams.
/// FillBase.hpp:30-80
#[derive(Debug, Clone)]
pub struct FillParams {
    /// Fill density (0.0 to 1.0).
    pub density: f64,
    /// Whether to use bridge flow.
    pub use_bridge_flow: bool,
    /// Whether to use Arachne variable-width fill.
    pub use_arachne: bool,
    /// Whether internal flow should be used (non-solid infill).
    pub using_internal_flow: bool,
    /// Whether paths can be reversed for better connection.
    pub can_reverse: bool,
    /// Extrusion role for this fill.
    pub extrusion_role: ExtrusionRole,
    /// Flow parameters.
    pub flow: Flow,
    /// Anchor length for connecting infill to perimeters (mm, scaled).
    pub anchor_length: f64,
    /// Maximum anchor length (mm, scaled).
    pub anchor_length_max: f64,
    /// Number of parallel lines to replicate per infill path (multiline infill).
    /// FillBase.hpp:58 — `int multiline{ 1 }`.
    pub multiline: i32,
    /// Layer height for Concentric infill with Arachne (unscaled, mm).
    /// FillBase.hpp — `coordf_t layer_height { 0.f }`.
    pub layer_height: f64,
    /// Don't adjust spacing to fill the space evenly.
    /// FillBase.hpp:68 — `bool dont_adjust { true }`.
    pub dont_adjust: bool,
}

impl FillParams {
    /// Create default FillParams with a basic flow.
    pub fn new() -> Self {
        Self {
            density: 1.0,
            use_bridge_flow: false,
            use_arachne: false,
            using_internal_flow: false,
            can_reverse: true,
            extrusion_role: ExtrusionRole::InternalInfill,
            flow: Flow::new(0.4, 0.2, 0.4).unwrap_or_else(|_| {
                // Fallback: create minimal flow
                Flow::new(0.45, 0.2, 0.4).expect("basic flow creation")
            }),
            anchor_length: 1000.0,
            anchor_length_max: 1000.0,
            multiline: 1,
            layer_height: 0.0,
            dont_adjust: true,
        }
    }

    /// Don't connect the fill lines around the inner perimeter.
    /// FillBase.hpp:56 — `bool dont_connect() const { return anchor_length_max < 0.05; }`.
    pub fn dont_connect(&self) -> bool {
        self.anchor_length_max < 0.05
    }
}

impl Default for FillParams {
    fn default() -> Self {
        Self::new()
    }
}

/// Returns orientation of the infill and the reference point of the infill pattern.
///
/// C++ reference: Fill::_infill_direction()
/// FillBase.cpp:199-241
///
/// For a normal print, the reference point is the center of the bounding box of the STL.
/// The angle alternates per layer based on `_layer_angle()`.
pub fn infill_direction(
    base_angle: f32,
    layer_id: usize,
    surface_thickness_layers: u32,
    bridge_angle: f64,
    bounding_box: &crate::geometry::BoundingBox,
) -> (f32, Point) {
    // FillBase.cpp:202-208
    let mut out_angle = base_angle;
    if out_angle == f32::MAX {
        out_angle = 0.0;
    }

    // FillBase.cpp:212-214 -- reference point is center of bounding box
    let out_shift = if bounding_box.is_empty() {
        Point::new(0, 0)
    } else {
        bounding_box.center()
    };

    // FillBase.cpp:224-237
    if bridge_angle >= 0.0 {
        // Use bridge angle
        out_angle = bridge_angle as f32;
    } else if layer_id != usize::MAX {
        // Alternate fill direction per layer
        // FillBase.cpp:233: out_angle += this->_layer_angle(this->layer_id / surface->thickness_layers);
        let effective_layer = if surface_thickness_layers > 0 {
            layer_id / surface_thickness_layers as usize
        } else {
            layer_id
        };
        // Default _layer_angle alternates 0 and 90 degrees
        out_angle += layer_angle(effective_layer);
    }

    // FillBase.cpp:239: out_angle += float(M_PI/2.);
    out_angle += std::f32::consts::FRAC_PI_2;

    (out_angle, out_shift)
}

/// Default layer angle alternation.
///
/// C++ reference: Fill::_layer_angle(size_t idx) const
/// FillBase.hpp:145-147
///
/// Returns 0 for even layers, PI/2 for odd layers (90 degree rotation).
fn layer_angle(idx: usize) -> f32 {
    if idx & 1 == 0 {
        0.0
    } else {
        std::f32::consts::FRAC_PI_2
    }
}

/// Calculate a new spacing to fill width with possibly integer number of lines.
///
/// C++ reference: Fill::_adjust_solid_spacing()
/// FillBase.cpp:179-195
///
/// The first and last line being centered at the interval ends.
/// This function possibly increases the spacing, never decreases,
/// and for a narrow width the increase in spacing may become severe,
/// therefore the adjustment is limited to 20% increase.
pub fn adjust_solid_spacing(width: Coord, distance: Coord) -> Coord {
    assert!(width >= 0);
    assert!(distance > 0);

    // FillBase.cpp:184: floor(width / distance)
    let number_of_intervals = ((width as f64 - 1e-6) / distance as f64) as Coord;
    let distance_new = if number_of_intervals == 0 {
        distance
    } else {
        ((width as f64 - 1e-6) / number_of_intervals as f64) as Coord
    };

    // FillBase.cpp:188-193: limit to 20% increase
    let factor = distance_new as f64 / distance as f64;
    let factor_max = 1.2;
    if factor > factor_max {
        ((distance as f64 * factor_max) + 0.5) as Coord
    } else {
        distance_new
    }
}

/// Wrap generated fill polylines into ExtrusionEntityCollection with proper flow.
///
/// C++ reference: Fill::fill_surface_extrusion()
/// FillBase.cpp:122-172
///
/// This is the base implementation that:
/// 1. Calls fill_surface() to generate polylines
/// 2. Calculates actual flow from spacing
/// 3. Wraps polylines into ExtrusionPath entities inside a collection
/// 4. Handles Arachne variable-width output
pub fn fill_surface_extrusion(
    polylines: Vec<Polyline>,
    params: &FillParams,
    actual_spacing: f64,
    no_sort: bool,
) -> Option<crate::extrusion_entity::ExtrusionEntityCollection> {
    if polylines.is_empty() {
        return None;
    }

    // FillBase.cpp:138-149: calculate actual flow from spacing
    let flow_mm3_per_mm;
    let flow_width;
    if params.using_internal_flow {
        // If we used the internal flow we're not doing a solid infill
        // so we can safely ignore the slight variation
        flow_mm3_per_mm = params.flow.mm3_per_mm().unwrap_or(0.0);
        flow_width = params.flow.width();
    } else {
        let new_flow = params
            .flow
            .with_spacing(actual_spacing)
            .unwrap_or(params.flow.clone());
        flow_mm3_per_mm = new_flow.mm3_per_mm().unwrap_or(0.0);
        flow_width = new_flow.width();
    }

    // FillBase.cpp:151-155: create collection
    let mut eec = crate::extrusion_entity::ExtrusionEntityCollection::new();
    eec.no_sort = no_sort;

    // FillBase.cpp:162-166: append paths with flow
    crate::extrusion_entity::extrusion_entities_append_paths(
        &mut eec.entities,
        polylines,
        params.extrusion_role,
        flow_mm3_per_mm,
        flow_width as f32,
        params.flow.height() as f32,
    );

    // FillBase.cpp:167-170: mark non-reversible if needed
    // TODO: add no_reverse field to ExtrusionPath when needed
    // if !params.can_reverse { ... }

    Some(eec)
}

/// Connect separate infill polylines into continuous paths along boundary contours.
///
/// C++ reference: Fill::connect_infill()
/// FillBase.cpp:1164-1600+ (~400 lines)
///
/// This is a simplified implementation of the C++ connect_infill algorithm.
/// The full C++ version uses a BoundaryInfillGraph with ContourIntersectionPoints
/// to optimally connect infill line endpoints along perimeter contours.
///
/// This simplified version:
/// 1. Tries to connect consecutive polylines whose endpoints are close
/// 2. Falls through to just outputting disconnected polylines otherwise
pub fn connect_infill(
    mut infill_ordered: Vec<Polyline>,
    boundary: &[Polygon],
    spacing: f64,
    _params: &FillParams,
    polylines_out: &mut Vec<Polyline>,
) {
    if infill_ordered.is_empty() {
        return;
    }

    // Simplified connection: try to merge consecutive polylines whose
    // endpoints are within 2x spacing distance
    let max_connection_dist = crate::scale(spacing * 2.5);
    let max_connection_dist_sq = (max_connection_dist as f64) * (max_connection_dist as f64);

    let mut i = 0;
    while i < infill_ordered.len() {
        let mut current = infill_ordered[i].clone();
        i += 1;

        // Try to append subsequent polylines if their start is close to current end
        while i < infill_ordered.len() {
            let next = &infill_ordered[i];
            if current.points.is_empty() || next.points().is_empty() {
                break;
            }
            let end_pt = *current.points.last().unwrap();
            let start_pt = next.points()[0];
            let dx = (end_pt.x() - start_pt.x()) as f64;
            let dy = (end_pt.y() - start_pt.y()) as f64;
            let dist_sq = dx * dx + dy * dy;

            if dist_sq < max_connection_dist_sq {
                // Check if the connection path stays within the boundary
                let midpoint = Point::new(
                    (end_pt.x() + start_pt.x()) / 2,
                    (end_pt.y() + start_pt.y()) / 2,
                );
                let inside = boundary.is_empty()
                    || boundary.iter().any(|poly| poly.contains_point(&midpoint));

                if inside {
                    // Connect: append next polyline's points to current
                    for pt in next.points().iter().skip(0) {
                        current.points.push(*pt);
                    }
                    i += 1;
                    continue;
                }
            }
            break;
        }

        if !current.points.is_empty() {
            polylines_out.push(current);
        }
    }
}

/// Overload of connect_infill that takes ExPolygon boundary.
///
/// C++ reference: Fill::connect_infill() (ExPolygon overload)
/// FillBase.cpp:1164-1172
pub fn connect_infill_expolygon(
    infill_ordered: Vec<Polyline>,
    boundary: &ExPolygon,
    spacing: f64,
    params: &FillParams,
    polylines_out: &mut Vec<Polyline>,
) {
    let mut polygons: Vec<Polygon> = Vec::with_capacity(boundary.holes.len() + 1);
    polygons.push(boundary.contour.clone());
    for hole in &boundary.holes {
        polygons.push(hole.clone());
    }
    connect_infill(infill_ordered, &polygons, spacing, params, polylines_out);
}

/// Apply a multi-line offset to a set of infill polylines.
///
/// When `params.multiline > 1`, each input polyline is replicated `multiline`
/// times, offset to both sides of the original along the local normal, so that
/// a single infill path becomes a band of parallel paths.
///
/// FillBase.cpp:2615
pub fn multiline_fill(polylines: &mut Vec<Polyline>, params: &FillParams, spacing: f32) {
    // FillBase.cpp:2617
    if params.multiline > 1 {
        // FillBase.cpp:2618
        let n_lines = params.multiline;
        // FillBase.cpp:2619
        let n_polylines = polylines.len() as i32;
        // FillBase.cpp:2620-2621
        let mut all_polylines: Vec<Polyline> = Vec::with_capacity((n_lines * n_polylines) as usize);

        // FillBase.cpp:2623
        let center = (n_lines - 1) as f32 / 2.0f32;

        // FillBase.cpp:2625
        // current polyline as the center line, offset to both sides
        // FillBase.cpp:2626
        for line in 0..n_lines {
            // FillBase.cpp:2627
            let offset = (line as f32 - center) * spacing;

            // FillBase.cpp:2629
            for pl in polylines.iter() {
                // FillBase.cpp:2630
                let n = pl.points.len();
                // FillBase.cpp:2631-2634
                if n < 2 {
                    all_polylines.push(pl.clone());
                    continue;
                }

                // FillBase.cpp:2636-2637
                let mut new_points: Vec<Point> = Vec::with_capacity(n);
                // FillBase.cpp:2638
                // Offset each point along the normal direction
                // FillBase.cpp:2639
                for i in 0..n {
                    // FillBase.cpp:2640
                    let tangent: (f32, f32);
                    // FillBase.cpp:2641-2642
                    if i == 0 {
                        tangent = (
                            (pl.points[1].x() - pl.points[0].x()) as f32,
                            (pl.points[1].y() - pl.points[0].y()) as f32,
                        );
                    // FillBase.cpp:2643-2644
                    } else if i == n - 1 {
                        tangent = (
                            (pl.points[n - 1].x() - pl.points[n - 2].x()) as f32,
                            (pl.points[n - 1].y() - pl.points[n - 2].y()) as f32,
                        );
                    } else {
                        // FillBase.cpp:2647
                        tangent = (
                            (pl.points[i + 1].x() - pl.points[i - 1].x()) as f32,
                            (pl.points[i + 1].y() - pl.points[i - 1].y()) as f32,
                        );
                        // FillBase.cpp:2648-2656 (commented out in C++)
                    }
                    // FillBase.cpp:2658
                    let mut len = tangent.0.hypot(tangent.1);
                    // FillBase.cpp:2659-2660
                    if len == 0.0 {
                        len = 1.0f32;
                    }
                    // FillBase.cpp:2661
                    let tangent = (tangent.0 / len, tangent.1 / len);
                    // FillBase.cpp:2662
                    let normal = (-tangent.1, tangent.0);

                    // FillBase.cpp:2664
                    let mut p = pl.points[i];
                    // FillBase.cpp:2665
                    p.x += crate::scale((normal.0 * offset) as f64);
                    // FillBase.cpp:2666
                    p.y += crate::scale((normal.1 * offset) as f64);
                    // FillBase.cpp:2667
                    new_points.push(p);
                }

                // FillBase.cpp:2670
                all_polylines.push(Polyline::from_points(new_points));
            }
        }
        // FillBase.cpp:2673
        *polylines = all_polylines;
    }
}

