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
// Faithful 1:1 port of Fill/Fill.cpp (self-contained subset). Distinct
// namespace `crate::fill::fill` to avoid colliding with the simplified
// SurfaceFillParams/SurfaceFill/group_fills currently defined in this mod.rs.
pub mod fill;
pub mod fill_base;
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
pub use fill_adaptive::{
    build_octree, generate_infill_lines as generate_adaptive_infill_lines,
    transform_to_octree, transform_to_world, triangle_aabb_intersects, Cube, CubeProperties,
    Octree,
};

/// R456: sparse-infill (InternalInfill) fill accounting, under FILL_SURFACE_DEBUG=1.
/// Areas are in milli-mm2, lengths in micrometres. `NOGEN` counts expolygons handed
/// to the filler that produced NO polylines — the "Internal area that is never filled"
/// the R455 arithmetic predicted.
pub static SPARSE_IN_N: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
pub static SPARSE_IN_AREA: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
pub static SPARSE_EMPTY_N: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
pub static SPARSE_NOGEN_N: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
pub static SPARSE_NOGEN_AREA: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
pub static SPARSE_OK_N: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
pub static SPARSE_OK_AREA: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
pub static SPARSE_OK_LEN: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);

// Re-export from fill3_d_honeycomb
pub use fill3_d_honeycomb::Fill3DHoneycomb;

// Re-export from fill_gyroid
pub use fill_gyroid::{generate_gyroid_infill, GyroidConfig};

// Re-export from fill_cross_hatch
pub use fill_cross_hatch::CrossHatchConfig;

// Re-export from fill_plane_path (space-filling curves)
pub use fill_plane_path::{FillPlanePath, PlanPathPattern};

// Re-export from fill_floating_concentric (FillFloatingConcentric.cpp/.hpp)
pub use fill_floating_concentric::{
    FloatingPolyline, FloatingPolylines, FloatingThickPolyline, FloatingThickPolylines,
    FloatingThickline, FloatingThicklines,
};

use crate::clipper_utils::{
    clip_clipper_polygons_with_subject_bbox_expolygons, intersection_ex_expolygons_polygons,
    offset_expolygon, OffsetJoinType,
};
use crate::extrusion_entity::ExtrusionRole;
use crate::fill::fill::is_narrow_infill_area;
use crate::flow::Flow;
use crate::geometry::{
    get_extents_expoly, get_extents_polygons, ExPolygon, Point, Polygon, Polyline,
};
use crate::libslic3r::{scale, SCALED_EPSILON};
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
    /// C++ FillParams::dont_adjust. FillMonotonicLineWGapFill forces it TRUE
    /// (FillRectilinear.cpp:3247): top-surface MonotonicLine keeps the nominal
    /// flow spacing and takes the align_to_grid branch.
    pub dont_adjust: bool,
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
            dont_adjust: false,
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

    /// Monotonic rectilinear (top/bottom/solid surfaces). Same raster core as
    /// Rectilinear but lines are emitted in monotonic sweep order and NOT
    /// connected into long polylines (FillMonotonic, FillRectilinear.hpp:47-53,
    /// no_sort()==true). Fill.hpp ipMonotonic.
    Monotonic,

    /// Monotonic line variant (FillMonotonicLine / FillMonotonicLineWGapFill,
    /// FillRectilinear.hpp:56-62,129-140). Fill.hpp ipMonotonicLine.
    MonotonicLine,
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
            | crate::print_config::InfillPattern::Cubic
            | crate::print_config::InfillPattern::Stars => InfillPattern::Grid,
            // Monotonic / MonotonicLine keep their identity so the fill driver
            // can engage the monotonic raster (FillMonotonic / FillMonotonicLine,
            // FillRectilinear.hpp:47-62): same rectilinear lines, but emitted in a
            // monotonic sweep WITHOUT connect_infill (no_sort()==true). Collapsing
            // them to Rectilinear (the prior behaviour) chained the lines into long
            // polylines, the opposite of native's traveled top/solid topology.
            crate::print_config::InfillPattern::Monotonic => InfillPattern::Monotonic,
            crate::print_config::InfillPattern::MonotonicLine => InfillPattern::MonotonicLine,
            // Aligned rectilinear has no monotonic ordering; plain rectilinear core.
            crate::print_config::InfillPattern::AlignedRectilinear => InfillPattern::Rectilinear,
            // Decorative / specialty patterns fall back to nearest equivalent
            crate::print_config::InfillPattern::HilbertCurve
            | crate::print_config::InfillPattern::ArchimedeanChords
            | crate::print_config::InfillPattern::OctagramSpiral => InfillPattern::Concentric,
            crate::print_config::InfillPattern::SupportCubic
            | crate::print_config::InfillPattern::Lattice2D => InfillPattern::Grid,
            // C++ FillLine / FillSupportBase are rectilinear-family fillers
            // (FillBase.cpp:44/54); the zig-zag family shares the rectilinear
            // raster core too.
            crate::print_config::InfillPattern::Line
            | crate::print_config::InfillPattern::SupportBase
            | crate::print_config::InfillPattern::ZigZag
            | crate::print_config::InfillPattern::CrossZag
            | crate::print_config::InfillPattern::LockedZag => InfillPattern::Rectilinear,
            // BBS internal-solid / floating concentric specializations.
            crate::print_config::InfillPattern::ConcentricInternal => InfillPattern::Concentric,
            crate::print_config::InfillPattern::FloatingConcentric => {
                InfillPattern::FloatingConcentric
            }
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

        // Fill.cpp:84-86 — sort FIRST by DECREASING bridge_angle, so bridges are
        // processed with priority when trimming one layer's fills by another in the
        // group_fills post-loop diff (Fill.cpp:368 "Bridges are processed first").
        // C++: if (this->bridge_angle > rhs.bridge_angle) return true; (i.e. larger
        // bridge_angle is "less" → sorts earlier). Mirror with a reversed compare.
        if self.bridge_angle != rhs.bridge_angle {
            return rhs
                .bridge_angle
                .partial_cmp(&self.bridge_angle)
                .unwrap_or(Ordering::Equal);
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
    lower_internal_areas: &[ExPolygon],
    _lock_param: &mut LockRegionParam,
) -> Result<Vec<SurfaceFill>> {
    // TOPDBG (diagnostics only, env-gated): Top state at the entry of the
    // make_fills surface grouping (last stop before extrusion emission).
    if crate::debug::topdbg::enabled() {
        let mut all: Vec<crate::surface::Surface> = Vec::new();
        for region_id in 0..layer.region_count() {
            if let Some(region) = layer.get_region(region_id) {
                all.extend(region.fill_surfaces.surfaces.iter().cloned());
            }
        }
        crate::debug::topdbg::log_top_surfaces(layer.id(), "group_fills_entry", &all);
        crate::debug::topdbg::dump_top_surfaces(layer.id(), "d5_group_fills_top", &all);
    }

    /// Fill.cpp:166
    /// C++: std::vector<SurfaceFill> surface_fills
    let mut surface_fills: Vec<SurfaceFill> = Vec::new();

    /// Fill.cpp:168
    /// C++: std::set<SurfaceFillParams> set_surface_params
    let mut set_surface_params: BTreeSet<SurfaceFillParams> = BTreeSet::new();

    /// Fill.cpp:169
    /// C++: std::vector<std::vector<const SurfaceFillParams*>> region_to_surface_params
    /// C++ stores a pointer to the params held in the sorted `set_surface_params`,
    /// then resolves it to a surface_fills index via `params->idx` (set after the
    /// set is materialised into the sorted surface_fills vector). We cannot store a
    /// raw index at insertion time because `set_surface_params` is a sorted
    /// `BTreeSet` — inserting shifts later elements — so we keep the params *value*
    /// here and resolve it to the final sorted index below.
    let mut region_to_surface_params: Vec<Vec<Option<SurfaceFillParams>>> =
        vec![Vec::new(); layer.region_count()];

    /// Fill.cpp:170
    /// C++: bool has_internal_voids = false
    let mut _has_internal_voids = false;

    // Fill.cpp:189-329
    // Populate region_to_surface_params from layer regions
    for region_id in 0..layer.region_count() {
        let region = layer.get_region(region_id).unwrap();
        region_to_surface_params[region_id] = vec![None; region.fill_surfaces.surfaces.len()];

        // FILL_REGION_DEBUG=1 — per-region accounting of the INTERNAL (sparse-infill
        // candidate) area vs the INTERNAL_VOID area that is skipped outright, plus the
        // density/filament that decide whether it gets filled at all. Aggregate the
        // stderr lines to find a region whose sparse infill silently disappears.
        if std::env::var_os("FILL_REGION_DEBUG").is_some() {
            let rc = region.region().config();
            let mut a_int = 0.0f64;
            let mut a_void = 0.0f64;
            let mut a_solid = 0.0f64;
            for s in region.fill_surfaces.surfaces.iter() {
                let a = s.expolygon.area() / (crate::SCALING_FACTOR * crate::SCALING_FACTOR);
                match s.surface_type {
                    SurfaceType::Internal => a_int += a,
                    SurfaceType::InternalVoid => a_void += a,
                    _ => {
                        if s.is_solid() {
                            a_solid += a
                        }
                    }
                }
            }
            eprintln!(
                "FILLDBG L{} r{} filament(sparse={} solid={} wall={}) density={:.4} \
                 area(int={:.2} void={:.2} solid={:.2})",
                layer.id(),
                region_id,
                rc.sparse_infill_filament,
                rc.solid_infill_filament,
                rc.wall_filament,
                rc.fill_density,
                a_int,
                a_void,
                a_solid
            );
        }

        // Fill.cpp:196-318
        for (surface_idx, surface) in region.fill_surfaces.surfaces.iter().enumerate() {
            // Fill.cpp:198
            if surface.surface_type == SurfaceType::InternalVoid {
                _has_internal_voids = true;
                continue;
            }

            // Build params for this surface
            // Fill.cpp:200-318
            // Fill.cpp:199
            // C++: const PrintRegionConfig &region_config = layerm.region().config();
            let region_config = region.region().config();

            // Fill.cpp:201 — C++: bool is_bridge = layer.id() > 0 && surface.is_bridge();
            let is_bridge = layer.id() > 0 && surface.is_bridge();

            // Fill.cpp:237-242 — faithful role precedence:
            //   is_bridge ? erBridgeInfill
            //     : (is_solid ? (is_top ? erTopSolidInfill
            //                   : (is_bottom ? erBottomSurface
            //                     : (is_floating_vertical_shell ? erFloatingVerticalShell
            //                       : erSolidInfill)))
            //       : erInternalInfill)
            let extrusion_role = if is_bridge {
                ExtrusionRole::BridgeInfill
            } else if surface.is_solid() {
                if surface.is_top() {
                    ExtrusionRole::TopSolidInfill
                } else if surface.is_bottom() {
                    ExtrusionRole::BottomSurface
                } else if surface.is_floating_vertical_shell() {
                    ExtrusionRole::FloatingVerticalShell
                } else {
                    ExtrusionRole::SolidInfill
                }
            } else {
                ExtrusionRole::InternalInfill
            };

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

            // Fill.cpp:255
            // C++: layerm.flow(extrusion_role, (surface.thickness == -1) ? layer.height : surface.thickness)
            // `Surface::thickness` is a SENTINEL of -1 ("unset", Surface.hpp:215) — NOT
            // "<= 0" — and the fallback is the LAYER's own height, never a constant. The
            // previous `if s.thickness > 0.0 { .. } else { 0.2 }` therefore took the
            // fallback for every surface (every Surface is constructed with thickness=-1)
            // and pinned all infill flow to a 0.2mm layer. That is invisible on Benchy,
            // whose layer height IS 0.2, but on Majora (0.3) it under-extruded every
            // infill feature by the h=0.2/h=0.3 flow ratio ~0.70 (R451).
            let layer_height = region
                .fill_surfaces
                .surfaces
                .get(surface_idx)
                .map(|s| if s.thickness == -1.0 { layer.height } else { s.thickness })
                .unwrap_or(layer.height);

            // R337: bridges must use the ROUND bridging_flow (m_bridge: wider
            // spacing = dmr + BRIDGE_EXTRA_SPACING, ~1.5x mm3/mm from the round
            // cross-section) — native LayerRegion::bridging_flow (Fill.cpp:253-255)
            // and rust's own unused fill/fill.rs both do this. This active path
            // historically used a rectangular solid-infill flow for bridges,
            // laying ~2x the lines (R336). Gated under BRIDGE_FLOW so the
            // byte-locked default output is preserved while validating.
            let flow = if is_bridge && crate::faithful_gate("BRIDGE_FLOW") {
                // thick_bridge = (surface.is_bridge() && !surface.is_external())
                //                || object_config.thick_bridges   (is_bridge here)
                let thick = !surface.is_external()
                    || layer.object().config().thick_bridges;
                region.bridging_flow(crate::flow::FlowRole::SolidInfill, thick, layer_height)?
            } else if layer.id() == 0 && crate::faithful_gate("BOTTOM_FLOW") {
                // R343: first-layer fills must use initial_layer_line_width. Native
                // routes the fill flow through layerm.flow() (Fill.cpp:256), whose
                // first_layer (=id==0) branch selects initial_layer_line_width; this
                // active path used new_from_config_width with the regular width,
                // under-filling the first layer (bottom surface: h~0.16 vs native
                // ~0.19; E/mm 0.031 vs 0.038). region.flow() applies the first-layer
                // width override. Gated BOTTOM_FLOW (byte-locked default preserved).
                region.flow(flow_role, layer_height)?
            } else {
                Flow::new_from_config_width(flow_role, config_width, 0.4, layer_height)?
            };

            let spacing = if surface.is_solid() || is_bridge {
                flow.spacing()
            } else {
                // Fill.cpp:281
                // C++: params.spacing = layerm.region().flow(*layer.object(), frInfill,
                //                          layer.object()->config().layer_height, false).spacing();
                // "Calculating infill line spacing independent of the current layer height
                // and 1st layer status, so that internal infill will be aligned over all
                // layers of the current region" — hence the OBJECT's configured layer
                // height and an explicit first_layer=false, not the constants that were
                // here (nozzle 0.4 / height 0.2).
                // PrintRegion::flow() inlined against the ObjectRef's config snapshots
                // (PrintRegion.cpp:21-50) so first_layer can be forced false.
                crate::print_region::flow_from_configs(
                    crate::flow::FlowRole::Infill,
                    layer.object().config().layer_height,
                    false,
                    layer.object().print().config().initial_layer_line_width,
                    layer.object().config().line_width,
                    layer.object().print().config().nozzle_diameter,
                    region_config,
                )
                .map_err(crate::Error::Slicing)?
                .spacing()
            };

            let (anchor_length, anchor_length_max) = if surface.is_solid() || is_bridge {
                (1000.0_f32, 1000.0_f32)
            } else {
                // Fill.cpp:286-293 — percent anchors resolve over the sparse
                // infill line spacing.
                let mut anchor = region_config.infill_anchor.value as f32;
                if region_config.infill_anchor.percent {
                    anchor = (anchor as f64 * 0.01 * spacing) as f32;
                }
                let mut anchor_max = region_config.infill_anchor_max.value as f32;
                if region_config.infill_anchor_max.percent {
                    anchor_max = (anchor_max as f64 * 0.01 * spacing) as f32;
                }
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
            // C++: auto it_params = set_surface_params.find(params);
            //      if (it_params == set_surface_params.end())
            //          it_params = set_surface_params.insert(it_params, params);
            //      region_to_surface_params[region_id][...] = &(*it_params);
            // The `idx` field is excluded from SurfaceFillParams ordering, so the set
            // dedups on the geometric/extrusion params only — matching C++.
            set_surface_params.insert(params.clone());
            region_to_surface_params[region_id][surface_idx] = Some(params);
        }
    }

    // Fill.cpp:327-330
    // Create surface_fills from params, stamping each params.idx with its final
    // (sorted) position. `params_to_idx` lets the populate loop below resolve a
    // stored params value to that position — the moral equivalent of C++'s
    // `params->idx` pointer deref.
    surface_fills.reserve(set_surface_params.len());
    let mut params_to_idx: std::collections::BTreeMap<SurfaceFillParams, usize> = Default::default();
    for params in set_surface_params {
        let mut params = params;
        let idx = surface_fills.len();
        params.idx = idx;
        params_to_idx.insert(params.clone(), idx);
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

            if let Some(Some(params)) = region_to_surface_params[region_id].get(surface_idx) {
                let params_idx = match params_to_idx.get(params) {
                    Some(&i) => i,
                    None => continue,
                };
                let fill = &mut surface_fills[params_idx];

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

    // Fill.cpp:361-373 — POST-LOOP: for each fill, make a union (safety offset) of
    // its polygons and subtract the PRECEDING groups' polygons so fills don't
    // overlap. Bridges are processed first (see SurfaceFillParams::operator<, which
    // sorts by decreasing bridge_angle — mirrored in our Ord above). Rust previously
    // skipped this whole block. The union merges near-touching fragments (drops
    // bridge over-extrusion; sparse is unaffected because the grid emitter already
    // unions internally) and the diff prevents inter-group double-extrusion.
    {
        let mut all_polygons: Vec<Polygon> = Vec::new();
        let n = surface_fills.len();
        for i in 0..n {
            if surface_fills[i].expolygons.is_empty() {
                continue;
            }
            if surface_fills[i].expolygons.len() > 1 || !all_polygons.is_empty() {
                let polys = crate::geometry::to_polygons(&surface_fills[i].expolygons);
                // Native Fill.cpp:361-373 runs these through ClipperLib @1e5 with
                // the raw safety offset; the geo variants grid @1um and cut a
                // spurious hole into stTop (R138: rust 46pts/1hole vs native
                // 49/0). Gated full-res (TOPFILL_FAITHFUL).
                let tf = crate::faithful_gate("TOPFILL_FAITHFUL");
                surface_fills[i].expolygons = if all_polygons.is_empty() {
                    if tf {
                        crate::clipper_utils::union_safety_offset_ex_clib(&polys)
                    } else {
                        crate::clipper_utils::union_safety_offset_ex(&polys)
                    }
                } else if tf {
                    let subj: Vec<crate::geometry::ExPolygon> = polys
                        .iter()
                        .map(|p| crate::geometry::ExPolygon::new(p.clone()))
                        .collect();
                    let clip: Vec<crate::geometry::ExPolygon> = all_polygons
                        .iter()
                        .map(|p| crate::geometry::ExPolygon::new(p.clone()))
                        .collect();
                    crate::clipper_utils::difference_clib_safety(&subj, &clip)
                } else {
                    crate::clipper_utils::diff_ex_polygons_polygons(
                        &polys,
                        &all_polygons,
                        crate::clipper_utils::ApplySafetyOffset::Yes,
                    )
                };
                all_polygons.extend(polys);
            } else if i != n - 1 {
                all_polygons.extend(crate::geometry::to_polygons(&surface_fills[i].expolygons));
            }
        }
    }

    // BBS: detect narrow internal solid infill area and use ipConcentricInternal pattern instead
    // Fill.cpp:453-546. `lower_internal_areas` is the union of the lower layer's
    // stInternal/stInternalVoid fill-surface expolygons, gathered by the caller
    // (Fill.cpp:455-464) because `group_fills` cannot reach a sibling Layer here.
    if layer.object().config().detect_narrow_internal_solid_infill {
        let surface_fills_size = surface_fills.len();
        for i in 0..surface_fills_size {
            // Fill.cpp:467-468
            if surface_fills[i].surface.surface_type != SurfaceType::InternalSolid {
                continue;
            }

            // Fill.cpp:470-473
            let expolygons_size = surface_fills[i].expolygons.len();
            let mut narrow_expoly_idx: Vec<usize> = Vec::new();
            let mut narrow_floating_expoly_idx: Vec<usize> = Vec::new();
            // BBS: get the index list of narrow expolygon
            // Fill.cpp:475-487
            for j in 0..expolygons_size {
                // Fill.cpp:476
                let bbox = get_extents_expoly(&surface_fills[i].expolygons[j]);
                // Fill.cpp:477 — bbox.inflated(scale_(2)); expand a little.
                let clipped_internals = clip_clipper_polygons_with_subject_bbox_expolygons(
                    lower_internal_areas,
                    &bbox.expanded(scale(2.0)),
                    false,
                );
                // Fill.cpp:478
                let clipped_internal_bbox = get_extents_polygons(&clipped_internals);
                // Fill.cpp:479-486
                if is_narrow_infill_area(&surface_fills[i].expolygons[j]) {
                    // Fill.cpp:480 — offset_ex(expoly, SCALED_EPSILON); the crate's
                    // offset helpers take millimeters.
                    if !clipped_internals.is_empty()
                        && bbox.intersects(&clipped_internal_bbox)
                        && !intersection_ex_expolygons_polygons(
                            &offset_expolygon(
                                &surface_fills[i].expolygons[j],
                                SCALED_EPSILON / crate::SCALING_FACTOR,
                                OffsetJoinType::Miter,
                            ),
                            &clipped_internals,
                        )
                        .is_empty()
                    {
                        narrow_floating_expoly_idx.push(j);
                    } else {
                        narrow_expoly_idx.push(j);
                    }
                }
            }

            // Fill.cpp:489-492
            if narrow_expoly_idx.is_empty() && narrow_floating_expoly_idx.is_empty() {
                // BBS: has no narrow expolygon
                continue;
            // Fill.cpp:493-497
            } else if narrow_floating_expoly_idx.len() == expolygons_size {
                surface_fills[i].params.pattern = InfillPattern::FloatingConcentric;
                surface_fills[i].params.extrusion_role = ExtrusionRole::FloatingVerticalShell;
                surface_fills[i].surface.surface_type = SurfaceType::FloatingVerticalShell;
            // Fill.cpp:498-500 — ipConcentricInternal maps onto this crate's
            // fill::InfillPattern::Concentric (see the config→fill mapping at
            // mod.rs:268).
            } else if narrow_expoly_idx.len() == expolygons_size {
                surface_fills[i].params.pattern = InfillPattern::Concentric;
            } else {
                // BBS: some expolygons are narrow, spilit surface_fills[i] and rearrange the expolygons
                // Fill.cpp:505-518
                if !narrow_expoly_idx.is_empty() {
                    let mut params = surface_fills[i].params.clone();
                    params.pattern = InfillPattern::Concentric;
                    let region_id_i = surface_fills[i].region_id;
                    let thickness_i = surface_fills[i].surface.thickness;
                    let region_id_group_i = surface_fills[i].region_id_group.clone();
                    let no_overlap_i = surface_fills[i].no_overlap_expolygons.clone();
                    surface_fills.push(SurfaceFill::new(params));
                    let back_idx = surface_fills.len() - 1;
                    surface_fills[back_idx].region_id = region_id_i;
                    surface_fills[back_idx].surface.surface_type = SurfaceType::InternalSolid;
                    surface_fills[back_idx].surface.thickness = thickness_i;
                    surface_fills[back_idx].region_id_group = region_id_group_i;
                    surface_fills[back_idx].no_overlap_expolygons = no_overlap_i;
                    // Fill.cpp:514-517
                    for &idx in &narrow_expoly_idx {
                        let exp = std::mem::take(&mut surface_fills[i].expolygons[idx]);
                        surface_fills[back_idx].expolygons.push(exp);
                    }
                }

                // Fill.cpp:520-534
                if !narrow_floating_expoly_idx.is_empty() {
                    let mut params = surface_fills[i].params.clone();
                    params.pattern = InfillPattern::FloatingConcentric;
                    params.extrusion_role = ExtrusionRole::FloatingVerticalShell;
                    let region_id_i = surface_fills[i].region_id;
                    let thickness_i = surface_fills[i].surface.thickness;
                    let region_id_group_i = surface_fills[i].region_id_group.clone();
                    let no_overlap_i = surface_fills[i].no_overlap_expolygons.clone();
                    surface_fills.push(SurfaceFill::new(params));
                    let back_idx = surface_fills.len() - 1;
                    surface_fills[back_idx].region_id = region_id_i;
                    surface_fills[back_idx].surface.surface_type =
                        SurfaceType::FloatingVerticalShell;
                    surface_fills[back_idx].surface.thickness = thickness_i;
                    surface_fills[back_idx].region_id_group = region_id_group_i;
                    surface_fills[back_idx].no_overlap_expolygons = no_overlap_i;
                    // Fill.cpp:530-533
                    for &idx in &narrow_floating_expoly_idx {
                        let exp = std::mem::take(&mut surface_fills[i].expolygons[idx]);
                        surface_fills[back_idx].expolygons.push(exp);
                    }
                }

                // Fill.cpp:536-538
                let mut to_be_delete = narrow_floating_expoly_idx.clone();
                to_be_delete.extend(narrow_expoly_idx.iter().cloned());
                to_be_delete.sort_unstable();

                // Fill.cpp:540-543
                for j in (0..to_be_delete.len()).rev() {
                    surface_fills[i].expolygons.remove(to_be_delete[j]);
                }
            }
        }
    }

    Ok(surface_fills)
}

/// Lock region parameters (for the locked-zag pattern).
/// FillBase.hpp:38-49
///
/// The C++ struct stores `std::map<float, ExPolygons>` /
/// `std::map<Flow, ExPolygons>` members. This crate has no `Ord` floats, so
/// each map is a `Vec<(key, ExPolygons)>` KEPT SORTED ASCENDING by key —
/// the `std::map` iteration order. Insertion goes through
/// `fill::fill::append_density_param` / `append_flow_param`
/// (Fill.cpp:175-189), which reproduce `map::find` (comparator equivalence:
/// `!(a<b) && !(b<a)`; for `Flow` the C++ `operator<` compares
/// `mm3_per_mm()`, Flow.hpp:88-90) + `map::insert`.
#[derive(Debug, Clone, Default)]
pub struct LockRegionParam {
    /// FillBase.hpp:41 — std::map<float, ExPolygons> skin_density_params;
    pub skin_density_params: Vec<(f32, Vec<ExPolygon>)>,
    /// FillBase.hpp:42 — std::map<float, ExPolygons> skin_depths_params;
    pub skin_depths_params: Vec<(f32, Vec<ExPolygon>)>,
    /// FillBase.hpp:43 — std::map<float, ExPolygons> locked_depths_params;
    pub locked_depths_params: Vec<(f32, Vec<ExPolygon>)>,
    /// FillBase.hpp:45 — ExPolygons outlook;
    pub outlook: Vec<ExPolygon>,
    /// FillBase.hpp:46 — std::map<float, ExPolygons> skeleton_density_params;
    pub skeleton_density_params: Vec<(f32, Vec<ExPolygon>)>,
    /// FillBase.hpp:47 — std::map<Flow, ExPolygons> skin_flow_params;
    pub skin_flow_params: Vec<(Flow, Vec<ExPolygon>)>,
    /// FillBase.hpp:48 — std::map<Flow, ExPolygons> skeleton_flow_params;
    pub skeleton_flow_params: Vec<(Flow, Vec<ExPolygon>)>,
}

/// Helper: drive `FillPlanePath::_fill_surface_single` over each fill area for the
/// space-filling-curve patterns (Hilbert, Archimedean, Octagram).
/// FillPlanePath.cpp:69-134.
fn generate_plane_path(
    pattern: fill_plane_path::PlanPathPattern,
    fill_area: &[ExPolygon],
    line_spacing: CoordF,
    angle: CoordF,
) -> Result<Vec<Polyline>> {
    let mut fill = fill_plane_path::FillPlanePath::new(pattern);
    // `line_spacing` here is already the per-line spacing; the C++ `_fill_surface_single`
    // divides `scaled(this->spacing) / params.density`, so feed spacing*density-back via
    // density = 1.0 and `this->spacing = line_spacing` to keep the same line pitch.
    fill.spacing = line_spacing;
    let mut params = FillParams::new();
    params.density = 1.0; // solid path: snug bbox, no cross-layer object alignment needed
    params.anchor_length_max = 0.0; // dont_connect(): chain instead of connect_infill
    params.resolution = 0.0125;
    let direction = (angle as f32, Point::new(0, 0));
    let mut all_polylines = Vec::new();
    for expoly in fill_area {
        fill._fill_surface_single(&params, 1, &direction, expoly.clone(), &mut all_polylines);
    }
    Ok(all_polylines)
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
            // FillAdaptive.cpp - adaptive density infill.
            // The line-generation + hook-connection core is ported in `fill_adaptive`
            // (`build_octree` + `generate_infill_lines`), but the full
            // `Filler::_fill_surface_single` entry point requires a mesh-built octree
            // and the `Fill` base-class state (z, spacing, params) threaded from
            // Print/PrintObject, which is not available through this 2D
            // `generate_infill` shim.
            let _ = (fill_area, line_spacing, angle);
            Err(crate::Error::Slicing(String::from(
                "Adaptive cubic infill requires a mesh-built octree (FillAdaptive::build_octree) and Fill base-class state (z/spacing/params); not reachable via the 2D generate_infill shim. Use fill_adaptive::build_octree + generate_infill_lines.",
            )))
        }
        InfillPattern::Honeycomb3D => {
            // Fill3DHoneycomb.cpp - 3D honeycomb infill
            // Drive the faithful `Fill3DHoneycomb::_fill_surface_single`, mirroring
            // `generate_plane_path`. `line_spacing` is the per-line spacing; the C++
            // `_fill_surface_single` divides `scale(spacing) * ... / density`, so feed
            // density = 1.0 with `this->spacing = line_spacing` to keep the line pitch.
            let mut fill = fill3_d_honeycomb::Fill3DHoneycomb {
                angle: angle as f32,
                spacing: line_spacing,
                z: 0.0, // z height - caller should provide
            };
            let mut params = FillParams::new();
            params.density = 1.0;
            params.anchor_length_max = 0.0; // dont_connect(): chain instead of connect_infill
            let direction = (angle as f32, Point::new(0, 0));
            let mut all_polylines = Vec::new();
            for expoly in fill_area {
                fill._fill_surface_single(
                    &params,
                    1,
                    &direction,
                    expoly.clone(),
                    &mut all_polylines,
                );
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
            generate_plane_path(
                fill_plane_path::PlanPathPattern::HilbertCurve,
                fill_area,
                line_spacing,
                angle,
            )
        }
        InfillPattern::ArchimedeanChords => {
            // FillPlanePath.cpp - Archimedean chords space-filling infill
            generate_plane_path(
                fill_plane_path::PlanPathPattern::ArchimedeanChords,
                fill_area,
                line_spacing,
                angle,
            )
        }
        InfillPattern::OctagramSpiral => {
            // FillPlanePath.cpp - Octagram spiral space-filling infill
            generate_plane_path(
                fill_plane_path::PlanPathPattern::OctagramSpiral,
                fill_area,
                line_spacing,
                angle,
            )
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
        InfillPattern::Monotonic | InfillPattern::MonotonicLine => {
            // Monotonic surfaces are routed through
            // generate_fill_rectilinear_monotonic by the toolpath driver
            // (Layer::make_fills). This generic single-layer helper does not
            // carry the monotonic sweep state, so fall back to the plain
            // rectilinear raster (same lines, just not monotonically ordered).
            generate_infill(InfillPattern::Rectilinear, fill_area, line_spacing, angle)
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
    /// G-code resolution (unscaled, mm). Used by space-filling-curve infills to
    /// pick the discretization angle of the Archimedean spiral.
    /// FillBase.hpp:65 — `coordf_t resolution`.
    pub resolution: f64,
    /// Do not sort the lines, just simply connect them.
    /// FillBase.hpp:93 — `bool dont_sort{ false }`.
    pub dont_sort: bool,
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
            resolution: 0.0125,
            dont_sort: false,
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
/// R461: distance (micrometres) from each clipped gyroid endpoint to the nearest edge
/// of the expolygon it was clipped against. Buckets: <1um, <10um, <100um, <1mm, >=1mm.
/// R462: endpoints that coincide with an ORIGINAL raw-wave vertex (vs a clip crossing).
#[allow(clippy::declare_interior_mutable_const)]
pub static GEP_PTS_HIST: [std::sync::atomic::AtomicUsize; 5] =
    [const { std::sync::atomic::AtomicUsize::new(0) }; 5];
pub static GEP_RAW_SUM_UM: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
pub static GEP_CROSS_N: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
pub static GEP_CROSS_SUM_UM: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
#[allow(clippy::declare_interior_mutable_const)]
pub static GEP_CROSS_HIST: [std::sync::atomic::AtomicUsize; 4] =
    [const { std::sync::atomic::AtomicUsize::new(0) }; 4];
pub static GEP_RAWVERT: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
pub static GEP_N: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
pub static GEP_SUM_UM: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
#[allow(clippy::declare_interior_mutable_const)]
pub static GEP_HIST: [std::sync::atomic::AtomicUsize; 5] =
    [const { std::sync::atomic::AtomicUsize::new(0) }; 5];

/// R459 diagnostic (FILL_CONNECT_DEBUG=1): does the non-monotonic connect step
/// actually JOIN anything? Counts polylines in vs runs out and the length added by
/// the implicit connector segments.
pub static CONN_IN: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
pub static CONN_OUT: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
pub static CONN_LEN_IN: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
pub static CONN_LEN_OUT: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);

pub fn connect_infill(
    infill_ordered: Vec<Polyline>,
    boundary: &[Polygon],
    spacing: f64,
    _params: &FillParams,
    polylines_out: &mut Vec<Polyline>,
) {
    if infill_ordered.is_empty() {
        return;
    }
    let dbg = std::env::var_os("FILL_CONNECT_DEBUG").is_some();
    let out_start = polylines_out.len();
    if dbg {
        use std::sync::atomic::Ordering::Relaxed;
        CONN_IN.fetch_add(infill_ordered.len(), Relaxed);
        let l: f64 = infill_ordered.iter().map(|p| p.length()).sum();
        CONN_LEN_IN.fetch_add((l / crate::SCALING_FACTOR * 1000.0) as usize, Relaxed);
    }

    // Simplified connection: try to merge consecutive polylines whose
    // endpoints are within 2x spacing distance
    // R459 diagnostic knob: the 2.5x factor cannot span a gyroid wave pitch
    // (spacing/(density*2.44)*PI), so this stub never chains gyroid at all.
    // FILL_CONNECT_MULT lets a round measure the mechanism without changing default
    // behaviour.
    let mult: f64 = std::env::var("FILL_CONNECT_MULT")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(2.5);
    let max_connection_dist = crate::scale(spacing * mult);
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
    if dbg {
        use std::sync::atomic::Ordering::Relaxed;
        CONN_OUT.fetch_add(polylines_out.len() - out_start, Relaxed);
        let l: f64 = polylines_out[out_start..].iter().map(|p| p.length()).sum();
        CONN_LEN_OUT.fetch_add((l / crate::SCALING_FACTOR * 1000.0) as usize, Relaxed);
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

