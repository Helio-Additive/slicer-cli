//! Fill infrastructure - Base types and Fill trait
//!
//! Direct port of BambuStudio's Fill.cpp and FillBase.cpp/hpp
//!
//! # C++ Reference
//! - Fill/Fill.cpp
//! - Fill/FillBase.cpp
//! - Fill/FillBase.hpp
//!
//! **STATUS:** Partial port - core infrastructure for fill generation

use crate::{
    BoundingBox, ExPolygon, ExtrusionRole, Flow, InfillPattern, Point, Polyline, Result, Surface,
    SurfaceType,
};

/// Parameters for filling a surface
/// Fill.cpp:23-118
#[derive(Debug, Clone)]
pub struct SurfaceFillParams {
    /// Zero based extruder ID.
    /// Fill.cpp:25
    pub extruder: u32,

    /// Infill pattern, adjusted for the density etc.
    /// Fill.cpp:27
    pub pattern: InfillPattern,

    /// For locked zag
    /// Fill.cpp:29-30
    pub skin_pattern: InfillPattern,
    pub skeleton_pattern: InfillPattern,

    /// In unscaled coordinates
    /// Fill.cpp:33
    pub spacing: f64,

    /// Infill / perimeter overlap, in unscaled coordinates
    /// Fill.cpp:35
    pub overlap: f64,

    /// Angle as provided by the region config, in radians.
    /// Fill.cpp:37
    pub angle: f32,

    /// Is bridging used for this fill? Bridging parameters may be used even if this->flow.bridge() is not set.
    /// Fill.cpp:39-40
    pub bridge: bool,

    /// Non-negative for a bridge.
    /// Fill.cpp:42
    pub bridge_angle: f32,

    /// Fill.cpp:45
    pub density: f32,

    /// Fill.cpp:46
    pub multiline: i32,

    /// Length of the infill anchor along the perimeter line.
    /// 1000mm is roughly the maximum length line that fits into a 32bit coord_t.
    /// Fill.cpp:49-51
    pub anchor_length: f32,
    pub anchor_length_max: f32,

    /// Width, height of extrusion, nozzle diameter, is bridge
    /// For the output, for fill generator.
    /// Fill.cpp:53-54
    pub flow: Flow,

    /// For the output
    /// Fill.cpp:56-57
    pub extrusion_role: ExtrusionRole,

    /// Index of this entry in a linear vector.
    /// Fill.cpp:61
    pub idx: usize,

    /// Infill speed settings
    /// Fill.cpp:63-65
    pub sparse_infill_speed: f32,
    pub top_surface_speed: f32,
    pub solid_infill_speed: f32,

    /// Param for cross zag
    /// Fill.cpp:66
    pub infill_shift_step: f32,

    /// Param for zig zag to get cross texture
    /// Fill.cpp:67
    pub infill_rotate_step: f32,

    /// Fill.cpp:68
    pub symmetric_infill_y_axis: bool,

    /// Params for 2Dlattice infill angles
    /// Fill.cpp:71-72
    pub lattice_angle_1: f32,
    pub lattice_angle_2: f32,
}

impl SurfaceFillParams {
    /// Create new parameters with defaults
    /// Fill.cpp:23
    pub fn new() -> Self {
        Self {
            extruder: 0,
            pattern: InfillPattern::Rectilinear,
            skin_pattern: InfillPattern::Rectilinear,
            skeleton_pattern: InfillPattern::Rectilinear,
            spacing: 0.0,
            overlap: 0.0,
            angle: 0.0,
            bridge: false,
            bridge_angle: 0.0,
            density: 0.0,
            multiline: 1,
            anchor_length: 1000.0,
            anchor_length_max: 1000.0,
            flow: Flow::default(),
            extrusion_role: ExtrusionRole::None,
            idx: 0,
            sparse_infill_speed: 0.0,
            top_surface_speed: 0.0,
            solid_infill_speed: 0.0,
            infill_shift_step: 0.0,
            infill_rotate_step: 0.0,
            symmetric_infill_y_axis: false,
            lattice_angle_1: -45.0,
            lattice_angle_2: 45.0,
        }
    }
}

impl Default for SurfaceFillParams {
    fn default() -> Self {
        Self::new()
    }
}

/// Parameters for fill generation algorithm
/// FillBase.hpp:48-110
#[derive(Debug, Clone)]
pub struct FillParams {
    /// Fill density, fraction in <0, 1>
    /// FillBase.hpp:57
    pub density: f32,

    /// FillBase.hpp:58
    pub multiline: i32,

    /// Length of an infill anchor along the perimeter.
    /// 1000mm is roughly the maximum length line that fits into a 32bit coord_t.
    /// FillBase.hpp:61-62
    pub anchor_length: f32,
    pub anchor_length_max: f32,

    /// G-code resolution.
    /// FillBase.hpp:65
    pub resolution: f64,

    /// Don't adjust spacing to fill the space evenly.
    /// FillBase.hpp:68
    pub dont_adjust: bool,

    /// Monotonic infill - strictly left to right for better surface quality of top infills.
    /// FillBase.hpp:71
    pub monotonic: bool,

    /// For Honeycomb.
    /// we were requested to complete each loop;
    /// in this case we don't try to make more continuous paths
    /// FillBase.hpp:75-76
    pub complete: bool,

    /// For Concentric infill, to switch between Classic and Arachne.
    /// FillBase.hpp:79
    pub use_arachne: bool,

    /// Layer height for Concentric infill with Arachne.
    /// FillBase.hpp:81
    pub layer_height: f64,

    /// FillBase.hpp:83
    pub pattern: InfillPattern,

    /// BBS
    /// FillBase.hpp:86-90
    pub flow: Flow,
    pub extrusion_role: ExtrusionRole,
    pub using_internal_flow: bool,
    pub no_extrusion_overlap: f32,
    pub dont_sort: bool,
    pub can_reverse: bool,

    /// Move infill to get cross zag pattern
    /// FillBase.hpp:92
    pub horiz_move: f32,

    /// FillBase.hpp:93-95
    pub symmetric_infill_y_axis: bool,
    pub symmetric_y_axis: i64,
    pub locked_zag: bool,

    /// 2D lattice angles
    /// FillBase.hpp:97-98
    pub lattice_angle_1: f32,
    pub lattice_angle_2: f32,
}

impl FillParams {
    /// Check if full infill
    /// FillBase.hpp:54
    pub fn full_infill(&self) -> bool {
        self.density > 0.9999
    }

    /// Don't connect the fill lines around the inner perimeter.
    /// FillBase.hpp:56
    pub fn dont_connect(&self) -> bool {
        self.anchor_length_max < 0.05
    }
}

impl Default for FillParams {
    fn default() -> Self {
        Self {
            density: 0.0,
            multiline: 1,
            anchor_length: 1000.0,
            anchor_length_max: 1000.0,
            resolution: 0.0125,
            dont_adjust: true,
            monotonic: false,
            complete: false,
            use_arachne: false,
            layer_height: 0.0,
            pattern: InfillPattern::Rectilinear,
            flow: Flow::default(),
            extrusion_role: ExtrusionRole::None,
            using_internal_flow: false,
            no_extrusion_overlap: 0.0,
            dont_sort: false,
            can_reverse: true,
            horiz_move: 0.0,
            symmetric_infill_y_axis: false,
            symmetric_y_axis: 0,
            locked_zag: false,
            lattice_angle_1: -45.0,
            lattice_angle_2: 45.0,
        }
    }
}

/// Grouped surface with fill parameters
/// Fill.cpp:538-544
#[derive(Debug, Clone)]
pub struct SurfaceFill {
    /// Fill.cpp:539
    pub surface: Surface,

    /// Fill.cpp:540
    pub expolygons: Vec<ExPolygon>,

    /// Fill.cpp:541
    pub params: SurfaceFillParams,

    /// Fill.cpp:542
    pub region_id: usize,

    /// Fill.cpp:543
    pub no_overlap_expolygons: Vec<ExPolygon>,
}

impl SurfaceFill {
    /// Create new SurfaceFill
    /// Fill.cpp:538
    pub fn new(surface: Surface, params: SurfaceFillParams, region_id: usize) -> Self {
        Self {
            expolygons: vec![surface.expolygon.clone()],
            surface,
            params,
            region_id,
            no_overlap_expolygons: Vec::new(),
        }
    }
}

/// Base Fill trait - corresponds to C++ Fill class
/// FillBase.hpp:138-250
pub trait Fill {
    /// Generate fill polylines for a surface
    /// FillBase.cpp:89
    fn fill_surface(&mut self, surface: &Surface, params: &FillParams) -> Result<Vec<Polyline>>;

    /// Get the spacing for this fill
    /// FillBase.hpp:150
    fn spacing(&self) -> f64;

    /// Set the spacing for this fill
    /// FillBase.hpp:151
    fn set_spacing(&mut self, spacing: f64);

    /// Get the bounding box
    /// FillBase.hpp:155
    fn bounding_box(&self) -> &BoundingBox;

    /// Set the bounding box
    /// FillBase.hpp:156
    fn set_bounding_box(&mut self, bbox: BoundingBox);

    /// Get layer ID
    /// FillBase.hpp:160
    fn layer_id(&self) -> usize;

    /// Set layer ID
    /// FillBase.hpp:161
    fn set_layer_id(&mut self, layer_id: usize);

    /// Get z coordinate
    /// FillBase.hpp:165
    fn z(&self) -> f64;

    /// Set z coordinate
    /// FillBase.hpp:166
    fn set_z(&mut self, z: f64);

    /// Get angle
    /// FillBase.hpp:170
    fn angle(&self) -> f32;

    /// Set angle
    /// FillBase.hpp:171
    fn set_angle(&mut self, angle: f32);

    /// Get link max length
    /// FillBase.hpp:175
    fn link_max_length(&self) -> i64;

    /// Set link max length
    /// FillBase.hpp:176
    fn set_link_max_length(&mut self, len: i64);

    /// Get loop clipping
    /// FillBase.hpp:180
    fn loop_clipping(&self) -> i64;

    /// Set loop clipping
    /// FillBase.hpp:181
    fn set_loop_clipping(&mut self, clip: i64);

    /// Whether this fill should be sorted
    /// FillBase.hpp:237
    fn no_sort(&self) -> bool {
        false
    }

    /// Whether this fill uses bridge flow
    /// FillBase.cpp:68
    fn use_bridge_flow(&self) -> bool {
        false
    }
}
