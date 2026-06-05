//! # Slicer
//!
//! A Rust rewrite of the BambuStudio core slicing algorithm.
//!
//! This library provides a complete 3D printing slicing pipeline:
//! - STL mesh loading and processing
//! - Layer slicing with configurable layer heights
//! - Perimeter generation (including Arachne variable-width)
//! - Infill pattern generation
//! - Support structure generation
//! - G-code generation
//!
//! ## Example
//!
//! ```rust,ignore
//! use slicer::{Mesh, SlicingConfig, Slicer};
//!
//! let mesh = Mesh::from_stl("model.stl")?;
//! let config = SlicingConfig::default();
//! let slicer = Slicer::new(config);
//! let gcode = slicer.slice(&mesh)?;
//! gcode.write_to_file("output.gcode")?;
//! ```

// Core modules (alphabetically organized for maintainability)
pub mod a_star;
pub mod aabb_mesh;
pub mod aabb_tree_indirect;
pub mod aabb_tree_lines;
pub mod algorithm;
pub mod any_ptr;
pub mod app_config;
pub mod arachne;
pub mod arc_fitter;
pub mod blacklisted_library_check;
pub mod bounding_box;
pub mod bridge_detector;
pub mod brim;
pub mod brim_ears_point;
pub mod build_volume;
pub mod by_object_print_data;
pub mod calib;
pub mod channel;
pub mod circle;
pub mod clipper2_utils;
pub mod clipper2_z_utils;
pub mod clipper_utils;
pub mod clipper_z_utils;
pub mod clonable_ptr;
pub mod color;
pub mod color_space_convert;
pub mod common_defs;
pub mod csg_mesh;
pub mod curve_analyzer;
pub mod custom_g_code;
pub mod cut_surface;
pub mod cut_utils;
pub mod edge_grid;
pub mod elephant_foot_compensation;
pub mod emboss;
pub mod emboss_shape;
pub mod enum_bitmask;
pub mod ex_polygon_collection;
pub mod ex_polygon_serialize;
pub mod ex_polygons_index;
pub mod exception;
pub mod execution;
pub mod extruder;
pub mod extrusion_entity;
pub mod extrusion_entity_collection;
pub mod extrusion_simulator;
pub mod face_detector;
pub mod filament_group;
pub mod filament_group_utils;
pub mod file_parser_error;
pub mod fill;
pub mod flow;
pub mod flush_vol_calc;
pub mod flush_vol_predictor;
pub mod flush_volume_calc;
pub mod format;
pub mod frustum;
pub mod function_trace;
pub mod fuzzy_skin;
pub mod gcode;
pub mod gcode_reader;
pub mod gcode_sender;
pub mod geometry;
pub mod i18n;
pub mod int128;
pub mod interlocking;
pub mod internal_bridge_detector;
pub mod intersection_points;
pub mod jump_point_search;
pub mod kd_tree_indirect;
pub mod layer;
pub mod layer_region;
pub mod libslic3r;
pub mod line_segmentation;
pub mod locales_utils;
pub mod log_sink;
pub mod mac_utils;
pub mod marching_squares;
pub mod measure;
pub mod measure_utils;
pub mod mesh_boolean;
pub mod mesh_split_impl;
pub mod min_area_bounding_box;
pub mod minimum_spanning_tree;
pub mod miniz_extension;
pub mod model;
pub mod model_arrange;
pub mod mt_utils;
pub mod multi_material_segmentation;
pub mod multi_nozzle_utils;
pub mod multi_point;
pub mod mutable_polygon;
pub mod mutable_priority_queue;
pub mod normal_utils;
pub mod nsvg_utils;
pub mod obj;
pub mod obj_color_utils;
pub mod object_id;
pub mod open_vdb_utils;
pub mod optimize;
pub mod orient;
pub mod overhang_detector;
pub mod parameter_utils;
pub mod pchheader;
pub mod perimeter_generator;
pub mod placeholder_parser;
pub mod platform;
pub mod png_read_write;
pub mod polygon_trimmer;
pub mod preset;
pub mod preset_bundle;
pub mod principal_components2_d;
pub mod print;
pub mod print_apply;
pub mod print_base;
pub mod print_config;
pub mod print_object;
pub mod print_object_slice;
pub mod print_region;
pub mod project_task;
pub mod quadric_collapse;
pub mod region_config;
pub mod region_expansion;
pub mod semver;
pub mod shape;
pub mod short_edge_collapse;
pub mod shortest_path;
pub mod sla;
pub mod sla_print;
pub mod sla_print_steps;
pub mod slicer;
pub mod slices_to_mesh;
pub mod slices_to_triangle_mesh;
pub mod slicing;
pub mod slicing_adaptive;
pub mod stl;
pub mod support;
pub mod surface;
pub mod surface_collection;
pub mod surface_mesh;
pub mod svg;
pub mod technologies;
pub mod tesselate;
pub mod text_configuration;
pub mod thread;
pub mod threemf;
pub mod time;
pub mod timer;
pub mod triangle_mesh;
pub mod triangle_mesh_deal;
pub mod triangle_mesh_slicer;
pub mod triangle_selector;
pub mod triangle_set_sampling;
pub mod triangulate_wall;
pub mod triangulation;
pub mod try_catch_signal;
pub mod try_catch_signal_seh;
pub mod utils;
pub mod variable_width;
pub mod vector_formatter;
pub mod zipper;

// Re-export commonly used types

pub use flow::{
    support_material_1st_layer_flow, support_material_flow, support_material_interface_flow,
    support_transition_flow, Flow, FlowError, FlowResult, FlowRole, BRIDGE_EXTRA_SPACING,
};

pub use fill::{
    // TODO: Re-enable once fill functions are implemented
    // generate_concentric_infill, generate_grid_infill, generate_gyroid_infill,
    // generate_honeycomb_infill, generate_infill_with_density,
    // generate_lightning_infill, generate_solid_infill,
    generate_infill,
    InfillConfig,
    InfillGenerator,
    InfillPath,
    InfillPattern,
    InfillResult,
};
pub use gcode::{
    ExtrusionPath, ExtrusionRole, GCode, GCodeWriter, LayerPaths, PathConfig, PathGenerator,
    SeamPosition,
};

// Re-export brim/adhesion types
pub use brim::{
    BrimConfig, BrimGenerator, BrimResult, BrimType, RaftConfig, RaftGenerator, RaftLayer,
    RaftLayerType, RaftResult, SkirtConfig, SkirtGenerator, SkirtResult,
};

// Re-export cooling types
pub use gcode::cooling::{
    CoolingBuffer, CoolingConfig, CoolingMove, CoolingResult, GCodeEditorState,
};

// Re-export G-code validation types
pub use gcode::validation::{
    validate_gcode_files, validate_gcode_files_with_config, FeatureStats, FeatureType,
    IssueCategory, IssueSeverity, LayerValidation, ReportFormat, ScoreBreakdown, ValidationConfig,
    ValidationIssue, ValidationReport, ValidationSummary,
};

// Re-export G-code comparison types
pub use gcode::compare::{
    compare_gcode, compare_gcode_files, ComparisonConfig, ComparisonResult, GCodeComparator,
    GCodeMove, LayerComparison, LayerInfo as GCodeLayerInfo, MoveComparison, ParsedGCode,
};

// Re-export pressure equalizer types
pub use gcode::pressure_equalizer::{
    PressureEqualizer, PressureEqualizerConfig, PressureEqualizerStats,
};

// Re-export ironing types
pub use gcode::ironing::{
    generate_ironing, should_iron_layer, IroningConfig, IroningGenerator, IroningPath,
    IroningResult, IroningType,
};

// Re-export spiral vase types
pub use gcode::spiral_vase::{SpiralPoint, SpiralVase, SpiralVaseConfig};

// Re-export seam placer types
pub use gcode::seam_placer::{
    create_seam_placer, place_seam, EnforcedBlockedSeamPoint, LayerOutline, LayerSeams, Perimeter,
    PerimeterOutline, Point3f, SeamCandidate, SeamPlacer, SeamPlacerConfig, SeamPlacerStats,
    SeamPositionMode,
};

// Re-export wipe tower types
pub use gcode::wipe_tower::{
    align_ceil, align_floor, align_round, is_valid_gcode, BedShape, BlockDepthInfo, BoxCoordinates,
    Extrusion as WipeTowerExtrusion, FilamentParameters, GCodeFlavor as WipeTowerGCodeFlavor,
    LimitFlow, NozzleChangeResult, ToolChangeInfo, ToolChangeResult, Vec2f, WipeShape, WipeTower,
    WipeTowerBlock, WipeTowerConfig, WipeTowerLayerInfo, WipeTowerWriter,
};

// Re-export tool ordering types
pub use gcode::tool_ordering::{
    calculate_flush_volume, find_optimal_ordering_exhaustive, generate_all_orderings,
    optimize_extruder_sequence, CustomGCodeItem, CustomGCodeType, ExtrusionRoleType,
    FilamentChangeMode, FilamentChangeStats, FilamentMapMode, FlushMatrix, LayerTools,
    ToolOrdering, ToolOrderingConfig, WipingExtrusions,
};

// Re-export multi-material coordination types
pub use gcode::multi_material::{
    MultiMaterialConfig, MultiMaterialCoordinator, MultiMaterialLayer, MultiMaterialPlan,
    ToolChange, WipeTowerBounds,
};

// Note: profiles module was deleted - no C++ equivalent
// Configuration now handled by print_config module

// Re-export elephant foot compensation
pub use geometry::elephant_foot::{ElephantFootCompensator, ElephantFootConfig};
pub use geometry::{
    BoundingBox, BoundingBox3, ExPolygon, Line, Point, Point3, Polygon, Polyline, ThickPolyline,
    ThickPolylines,
};

// Re-export path simplification
pub use geometry::simplify::{
    douglas_peucker, douglas_peucker_polygon, douglas_peucker_polyline, simplify_comprehensive,
    simplify_polygon, simplify_polygon_comprehensive, simplify_polygons, simplify_polyline,
    simplify_polyline_comprehensive, simplify_polylines, SimplifyConfig, MESHFIX_MAXIMUM_DEVIATION,
    MESHFIX_MAXIMUM_RESOLUTION,
};

// Re-export AABB tree types
pub use geometry::{
    closest_point_on_triangle, ray_box_intersect, ray_triangle_intersect, AABBClosestPointResult,
    AABBNode, AABBTree, IndexedTriangleSet, RayHit, Vec3, AABB3,
};
pub use layer::Layer;
pub use print::{Print, PrintObject};
pub use slicer::Slicer;
pub use slicing::SlicingParams;
pub use triangle_mesh::{Triangle, TriangleMesh};

// Re-export adaptive layer heights
pub use slicing_adaptive::{
    compute_adaptive_heights, compute_adaptive_heights_with_quality, AdaptiveHeightsConfig,
    AdaptiveLayerHeight, AdaptiveSlicing, FaceZ, SlopeErrorMetric,
};

// Re-export clipper operations
pub use clipper_utils::{
    difference, grow, intersection, offset_expolygon, offset_expolygons, offset_polygon,
    offset_polygons, shrink, union, union_ex, xor, OffsetJoinType, OffsetType,
};

// Re-export perimeter generation
pub use perimeter_generator::{
    PerimeterConfig, PerimeterGenerator, PerimeterLoop, PerimeterResult,
};

// Re-export fuzzy skin
pub use fuzzy_skin::{
    apply_fuzzy_skin_extrusion, apply_fuzzy_skin_polygon, fuzzy_extrusion_line,
    fuzzy_extrusion_line_params, fuzzy_polygon, fuzzy_polygon_params, fuzzy_polyline,
    should_fuzzify, FuzzySkinConfig,
};

// Re-export Arachne variable-width perimeter generation
pub use arachne::{
    generate_arachne_walls, generate_arachne_walls_with_width, ArachneConfig, ArachneGenerator,
    ArachneResult, BeadingCalculator, BeadingResult, BeadingStrategy, ExtrusionJunction,
    ExtrusionLine, VariableWidthLines,
};

// Note: infill generation already re-exported in fill module (lines 196-202)

// Re-export adaptive infill
pub use fill::{
    // TODO: Re-enable once adaptive infill is fully implemented
    // build_octree, generate_adaptive_infill_with_density,
    // AdaptiveInfillGenerator, AdaptiveInfillResult, Vec3d,
    generate_adaptive_infill,
    AdaptiveInfillConfig,
    CubeProperties,
    Octree,
};

// Re-export 3D honeycomb infill
pub use fill::{
    // TODO: Re-enable once 3D honeycomb is fully implemented
    // generate_honeycomb_3d, Honeycomb3DGenerator, Honeycomb3DResult,
    Honeycomb3DConfig,
};

// Re-export Cross Hatch infill
pub use fill::{
    // TODO: Re-enable once cross hatch is fully implemented
    // generate_cross_hatch, generate_cross_hatch_with_angle, CrossHatchGenerator,
    // CrossHatchResult,
    CrossHatchConfig,
};

// Re-export plan path infill (space-filling curves)
pub use fill::{
    // TODO: Re-enable once plan path functions are fully implemented
    // generate_archimedean_chords, generate_hilbert_curve, generate_octagram_spiral,
    // PlanPathGenerator, PlanPathResult,
    PlanPathConfig,
    PlanPathPattern,
};

// Re-export floating concentric infill
pub use fill::{
    // TODO: Re-enable once floating concentric functions are fully implemented
    // generate_floating_concentric, generate_floating_concentric_with_config,
    // FloatingConcentricGenerator, FloatingConcentricResult,
    // FloatingThickLine, FloatingThickPolyline,
    FloatingConcentricConfig,
};

// Re-export support generation
pub use support::{
    sample_overhang_points, SupportConfig, SupportGenerator, SupportLayer, SupportPattern,
    SupportType, TreeBranch, TreeSupportGenerator,
};

// Re-export tree support 3D types
pub use support::tree_model_volumes::{
    find_nearest_safe_position, is_safe_position, point_inside_polygons, AvoidanceType,
    RadiusLayerKey, RadiusLayerPolygonCache, TreeModelVolumes, TreeModelVolumesConfig,
    COLLISION_RESOLUTION, EXPONENTIAL_FACTOR, EXPONENTIAL_THRESHOLD,
};
pub use support::tree_support_3d::{
    LayerSupportElements, LineInformation, LineInformations, SupportElements, TreeSupport3D,
    TreeSupport3DConfig, TreeSupport3DResult,
};
pub use support::tree_support_settings::{
    AreaIncreaseSettings, AvoidanceTypeCompact, InterfacePreference, LineStatus, ParentIndices,
    SupportElement, SupportElementState, SupportElementStateBits, TreeSupportMeshGroupSettings,
};

// Re-export bridge detection
pub use bridge_detector::{
    detect_bridges, detect_bridging_direction, detect_internal_bridges, generate_bridge_infill,
    Bridge, BridgeConfig, BridgeDetector, InternalBridgeConfig, InternalBridgeDetector,
};

// Re-export edge grid
pub use edge_grid::{ClosestPointResult, Contour, EdgeGrid, Intersection};

// Re-export travel planning
pub use crate::gcode::avoid_crossing_perimeters::{
    AvoidCrossingPerimeters, TravelConfig, TravelResult,
};

/// Coordinate type used throughout the slicer.
/// Using i64 for integer coordinates (scaled by SCALING_FACTOR) to avoid floating-point issues.
pub type Coord = i64;

/// Floating-point coordinate type for unscaled values.
pub type CoordF = f64;

/// Scaling factor: coordinates are stored as integers scaled by this factor
/// libslic3r.h:38-40
/// 1 unit = 10 nanometers, so 1mm = 100_000 units.
/// This matches BambuStudio/PrusaSlicer's internal scaling (SCALING_FACTOR = 0.00001).
pub const SCALING_FACTOR: f64 = 100_000.0;

#[inline]
/// Scale a floating-point coordinate to integer
/// libslic3r.h:42-45
pub fn scale(v: CoordF) -> Coord {
    (v * SCALING_FACTOR).round() as Coord
}

#[inline]
/// Unscale an integer coordinate to floating-point
/// libslic3r.h:47-50
pub fn unscale(v: Coord) -> CoordF {
    v as CoordF / SCALING_FACTOR
}

#[inline]
/// Scale a floating-point coordinate to integer (same as scale, for compatibility)
/// libslic3r.h:52-55
pub fn scaled(v: CoordF) -> Coord {
    scale(v)
}

#[inline]
/// Unscale an integer coordinate to floating-point (same as unscale, for compatibility)
/// libslic3r.h:57-60
pub fn unscaled(v: Coord) -> CoordF {
    unscale(v)
}

/// Result type used throughout the slicer.
pub type Result<T> = std::result::Result<T, Error>;

#[derive(Debug, thiserror::Error)]
/// Error type for slicer operations
/// Exception.hpp:15-35
pub enum Error {
    #[error("IO error: {0}")]
    Io(#[from] std::io::Error),

    /// IO error (alias for compatibility)
    /// Exception.hpp:23
    /// C++: SLIC3R_DERIVE_EXCEPTION(IOError, CriticalException);
    #[error("IO error: {0}")]
    IO(String),

    #[error("Mesh error: {0}")]
    Mesh(String),

    #[error("Slicing error: {0}")]
    Slicing(String),

    #[error("G-code error: {0}")]
    GCode(String),

    #[error("Configuration error: {0}")]
    Config(String),

    #[error("Invalid geometry: {0}")]
    Geometry(String),

    /// Invalid input/argument error
    /// Exception.hpp:21
    /// C++: SLIC3R_DERIVE_EXCEPTION(InvalidArgument, LogicError);
    #[error("Invalid input: {0}")]
    InvalidInput(String),

    /// Parse error
    /// Exception.hpp:26
    /// C++: SLIC3R_DERIVE_EXCEPTION(PlaceholderParserError, RuntimeError);
    #[error("Parse error: {0}")]
    ParseError(String),

    /// Flow calculation error
    /// Flow.cpp:14-17
    #[error("Flow error: {0}")]
    Flow(#[from] crate::flow::FlowError),

    #[error("Cancelled")]
    Cancelled,
}

/// Version information
/// libslic3r.h:25
pub const VERSION: &str = env!("CARGO_PKG_VERSION");

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_scaling() {
        // 1mm should scale to 1_000_000
        assert_eq!(scale(1.0), 1_000_000);

        // And back
        assert!((unscale(1_000_000) - 1.0).abs() < 1e-10);

        // Test sub-millimeter precision
        assert_eq!(scale(0.001), 1_000); // 1 micron
        assert_eq!(scale(0.0001), 100); // 100 nanometers
    }
}
