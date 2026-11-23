//! PrintObject - Individual object to be printed
//!
//! C++ Reference:
//! - PrintObject.hpp (316 lines, Print.hpp:378-693)
//! - PrintObject.cpp (1,000+ lines)
//! - PrintObjectSlice.cpp (843+ lines)
//!
//! This is a **1:1 port** of BambuStudio's PrintObject.cpp/hpp.
//! Every function must have exact C++ file:line references.
//!
//! ## Status
//!
//! **✅ REGION INFRASTRUCTURE COMPLETE** - PrintObject now supports regions
//!
//! ### ✅ Ported
//! - PrintObject struct definition with all core fields
//! - PrintObjectRegions struct (shared region data)
//! - Basic lifecycle methods (new, with_config, etc.)
//! - Step tracking (is_step_done, set_step_done, invalidate_all_steps)
//! - slice() - mesh slicing into layers
//! - make_perimeters() - structure complete with region loop
//! - prepare_infill() - minimal stub
//! - infill() - basic implementation
//! - generate_support_material() - stub only
//! - Region accessors: num_printing_regions(), printing_region(), all_regions()
//! - shared_regions field with Arc reference counting
//! - typed_slices field and restoration logic structure
//!
//! ### ⚠️ Partially Ported (Extra Perimeters)
//! - Extra perimeters algorithm loop structure exists (PrintObject.cpp:481-540)
//! - Currently skips execution (BambuStudio has extra_perimeters disabled by default)
//! - Algorithm body marked as TODO for future implementation if needed
//!
//! ### ❌ Still Missing
//! - Perimeter continuity calculation (z_direction_outwall_speed_continuous)
//! - Multi-region merging logic in Layer::make_perimeters
//! - Per-region config passing (currently uses default)
//! - Layer::restore_untyped_slices() implementation
//!
//! ### 🔧 Next Steps
//! 1. Implement per-region config passing in make_perimeters
//! 2. Port perimeter continuity calculation if needed
//! 3. Port multi-region merging logic
//! 4. Implement restore_untyped_slices if typed_slices gets used
//! 5. Port support material generation fully

use crate::{
    geometry::{Polygon, Polygons},
    layer::Layer,
    print_config::{PrintConfig, PrintObjectConfig},
    print_region::PrintRegion,
    surface_collection::SurfaceCollection,
    Error, Result,
};
use std::sync::{
    atomic::{AtomicBool, Ordering},
    Arc,
};

// PrintObject step IDs
// Print.hpp:101-106
/// Print.hpp:101
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum PrintObjectStep {
    /// Slice mesh into layers
    /// Print.hpp:102
    Slice,
    /// Generate perimeter shells
    /// Print.hpp:102
    Perimeters,
    /// Prepare infill regions
    /// Print.hpp:102
    PrepareInfill,
    /// Generate infill patterns
    /// Print.hpp:102
    Infill,
    /// Generate ironing paths
    /// Print.hpp:102
    Ironing,
    /// Generate support material
    /// Print.hpp:102
    SupportMaterial,
    /// Detect overhangs for Z-lift
    /// Print.hpp:104
    DetectOverhangsForLift,
    /// Simplify perimeter paths
    /// Print.hpp:105
    SimplifyWall,
    /// Simplify infill paths
    /// Print.hpp:105
    SimplifyInfill,
    /// Simplify support paths
    /// Print.hpp:105
    SimplifySupportPath,
}

/// PrintObjectRegions - Shared region data for PrintObject
/// Print.hpp:230-319
///
/// Contains all regions (material/config groups) used by this PrintObject.
/// Shared between PrintObjects created from the same ModelObject.
pub struct PrintObjectRegions {
    /// All regions used by this object
    /// Print.hpp:298
    pub all_regions: Vec<Arc<PrintRegion>>,

    /// Reference count for sharing
    ref_count: std::sync::atomic::AtomicUsize,
}

impl PrintObjectRegions {
    /// Create new empty PrintObjectRegions
    pub fn new() -> Self {
        Self {
            all_regions: Vec::new(),
            ref_count: std::sync::atomic::AtomicUsize::new(1),
        }
    }

    /// Create with regions
    pub fn with_regions(regions: Vec<Arc<PrintRegion>>) -> Self {
        Self {
            all_regions: regions,
            ref_count: std::sync::atomic::AtomicUsize::new(1),
        }
    }

    /// Increment reference count
    pub fn ref_inc(&self) {
        self.ref_count.fetch_add(1, Ordering::Relaxed);
    }

    /// Decrement reference count and return true if should be dropped
    pub fn ref_dec(&self) -> bool {
        self.ref_count.fetch_sub(1, Ordering::Relaxed) == 1
    }
}

/// Zero-cost upward view from a PrintObject to its owning Print's config,
/// preserving the C++ call shape `object->print()->config()`.
///
/// C++: `PrintType* print() { return m_print; }` (PrintBase.hpp:632). In Rust
/// the ownership tree points downward, so instead of a parent pointer the
/// PrintObject holds an `Arc<PrintConfig>` snapshot stamped at sync points
/// (Print::add_object, top of Print::process). This is faithful because in
/// C++ the print config only mutates inside Print::apply, and any diff there
/// invalidates posSlice — so between sync points the snapshot cannot diverge.
pub struct PrintRef<'a> {
    config: &'a PrintConfig,
}

impl<'a> PrintRef<'a> {
    /// Access the print-level configuration.
    /// C++: `const PrintConfig& config() const { return m_config; }` (Print.hpp:885)
    pub fn config(&self) -> &'a PrintConfig {
        self.config
    }
}

/// Zero-cost upward view from a Layer to its owning PrintObject, preserving
/// the C++ call shapes `layer->object()->config()` and
/// `layer->object()->print()->config()`.
///
/// C++: `PrintObject* object() { return m_object; }` (Layer.hpp:139). In Rust
/// the ownership tree points downward, so instead of a parent pointer each
/// Layer holds Arc snapshots of the object/print configs, stamped at sync
/// points by PrintObject::wire_layer_hierarchy.
pub struct ObjectRef<'a> {
    object_config: &'a PrintObjectConfig,
    print_config: &'a PrintConfig,
}

impl<'a> ObjectRef<'a> {
    pub(crate) fn new(
        object_config: &'a PrintObjectConfig,
        print_config: &'a PrintConfig,
    ) -> Self {
        Self {
            object_config,
            print_config,
        }
    }

    /// Access the object-level configuration.
    /// C++: `const PrintObjectConfig& config() const { return m_config; }` (Print.hpp:369)
    pub fn config(&self) -> &'a PrintObjectConfig {
        self.object_config
    }

    /// Upward view to the owning Print.
    /// C++: `PrintType* print() { return m_print; }` (PrintBase.hpp:632)
    pub fn print(&self) -> PrintRef<'a> {
        PrintRef {
            config: self.print_config,
        }
    }
}

/// PrintObject - Individual object to be printed
/// Print.hpp:378-693 (316 lines)
pub struct PrintObject {
    /// Layers of this object
    /// Print.hpp:612
    pub layers: Vec<Layer>,

    /// Support layers
    /// Print.hpp:613
    pub support_layers: Vec<Layer>,

    /// Object configuration
    /// Print.hpp:614
    pub config: PrintObjectConfig,

    /// Snapshot of the owning Print's configuration, shared via Arc.
    /// C++ reaches this through the parent pointer: `m_print->config()`
    /// (PrintBase.hpp:632 + Print.hpp:885). Stamped wholesale at sync points
    /// (Print::add_object, top of Print::process) — NEVER mutated in place
    /// (no Arc::make_mut/get_mut), which would fork the share.
    print_config: Arc<PrintConfig>,

    /// R85 slice-frame XY center_offset (mm) applied in the fused f32 slice
    /// transform (= C++ m_center_offset unscaled). Consumed by Print to set the
    /// gcode export origin so the centered slices re-align to C++'s gcode frame.
    /// (0,0) until slice() computes it. Only effective under SLICE_CENTER.
    pub slice_center_offset: (f64, f64),

    /// Reference to parent model object
    /// Print.hpp:615
    pub model_object_id: usize,

    /// BambuStudio identify_id from model_settings.config — the label used in
    /// ; OBJECT_ID comments. Corresponds to ModelInstance::loaded_id /
    /// get_labeled_id() in C++. 0 means unset (falls back to object index).
    pub label_id: usize,

    /// Processing state
    /// Print.hpp:616
    state: u32,

    /// Cancellation flag shared with Print
    canceled: Arc<AtomicBool>,

    /// Triangle mesh to slice
    /// PrintObject.hpp:380
    mesh: Option<crate::triangle_mesh::TriangleMesh>,

    /// R704 — merged NEGATIVE-volume mesh, sliced at the SAME layer z's as
    /// `mesh` and subtracted from every region's slices.
    ///
    /// C++ keeps these as `ModelVolume`s of type `NEGATIVE_VOLUME`: they ARE
    /// sliced (`model_volume_needs_slicing`, PrintObjectSlice.cpp:110) and then
    /// `slices_to_regions` (:403-421) does `diff_ex` of every preceding
    /// non-negative region against them. Holding one merged mesh is the Tier-1
    /// shape of that — the subtraction below is per layer in 2D, which is what
    /// C++ does; a 3D mesh boolean would not be faithful.
    negative_mesh: Option<crate::triangle_mesh::TriangleMesh>,

    /// Slicing parameters
    /// PrintObject.hpp:381
    slicing_params: crate::slicing::SlicingParams,

    /// Shared region data
    /// PrintObject.hpp:605
    shared_regions: Option<Arc<PrintObjectRegions>>,

    /// Whether slices have been typed (top/bottom/internal)
    /// PrintObject.hpp:613
    typed_slices: bool,

    /// Painted multi-material sub-meshes: (extruder slot 1-based, painted
    /// facets), mesh coords in mm — extracted by the loader from the 3MF
    /// `paint_color` annotation (C++: ModelVolume::mmu_segmentation_facets →
    /// TriangleSelector::get_facets, MMS.cpp:2226). Ascending extruder order;
    /// index i maps to painted print region `1 + i` (install_painted_regions).
    pub painted_submeshes: Vec<(u8, crate::normal_utils::indexed_triangle_set)>,

    /// The first layer took the multi-region Elephant foot branch in `slice()`
    /// (PrintObjectSlice.cpp:2331-2378), so `make_perimeters` must not
    /// compensate it again.
    pub l0_efc_done_in_slice: bool,

    /// R768 — strict per-state sub-meshes for the top/bottom painted-surface
    /// propagation: one `(state, mesh)` per `EnforcerBlockerType` state
    /// 0..=num_filaments (state 0 = the unpainted rest), from
    /// `TriangleSelector::get_facets_strict` (MMS.cpp:1356; strict splits
    /// T-joints so the slab slicer sees watertight neighbor topology). Empty
    /// states omitted; empty vec = no painting (or loader predates R768).
    pub painted_submeshes_strict: Vec<(u8, crate::normal_utils::indexed_triangle_set)>,

    /// R786 — the scaled XY offset stripped from the paint-frame transform
    /// (native's `center_offset`); the painted-line construction adds it back
    /// as an integer translate. None when the paint frame is inactive.
    pub paint_frame_offset: Option<(i64, i64)>,
    /// C++ `PrintInstance::shift` of the object's instance (scaled): the
    /// instance translation `Point::new_scale`d plus `m_center_offset`
    /// (PrintApply.cpp:150, PrintObject.cpp:114); `unscale` of it is the G-code
    /// origin (GCode.cpp:5873). Set by the 3MF loader.
    pub instance_shift: Option<(i64, i64)>,

    /// R788 — native's topbot volume_trafo (trafo_centered * volume matrix);
    /// when set, the strict sub-meshes are volume-LOCAL and the slab chain
    /// uses this trafo with voff (0,0,0).
    pub paint_tb_trafo: Option<[f64; 16]>,
    /// R805 — raw VOLUME-LOCAL f32 mesh of the main volume (same triangles as
    /// `mesh`), sliced through `paint_tb_trafo` under SLICE_VOLUME_TRAFO.
    pub raw_local_mesh: Option<crate::triangle_mesh::TriangleMesh>,
    /// R809 — the slice frame's offset from the mesh frame in mm, when the
    /// caller placed the object (`unscale` of the print instance's shift).
    /// Replaces the bounding-box centre that `slice()` derives otherwise.
    pub slice_center_override: Option<(f64, f64)>,
    /// R811 — the translations the seam placer's occlusion mesh goes through in
    /// C++ when the caller placed the object: the volume matrix's, then
    /// `trafo_centered()`'s (SeamPlacer.cpp 577-604). With `raw_local_mesh`.
    pub occlusion_place: Option<([f64; 3], [f64; 3])>,
    /// The object's volumes as the seam placer's occlusion mesh takes them
    /// (SeamPlacer.cpp:577-606): for each MODEL_PART and NEGATIVE_VOLUME its
    /// volume-local mesh, its matrix (rows of the linear part, then the
    /// translation) and whether it is a negative volume; then `trafo_centered()`
    /// in the same layout.
    pub occlusion_volumes: Option<(Vec<(crate::triangle_mesh::TriangleMesh, [f64; 12], bool)>, [f64; 12])>,
    /// The painted seam facets (`ModelVolume::seam_facets`), ENFORCER then
    /// BLOCKER, already through `trafo_centered() * volume matrix`
    /// (SeamPlacer.cpp:652-663).
    pub seam_facets: Option<(
        crate::triangle_mesh::indexed_triangle_set,
        crate::triangle_mesh::indexed_triangle_set,
    )>,
    /// R805 — negative volumes as (volume-local mesh, exact 4x4 slicing transform).
    pub negative_volumes_raw: Vec<(crate::triangle_mesh::TriangleMesh, [f64; 16])>,

    /// R770 — raw per-layer negative-volume slices (index-aligned with
    /// `layers`), captured during the R704 subtraction for the MM
    /// segmentation's fill + clip-back flow (cpp `neg_slices`, MMS.cpp:2217).
    /// Empty when the object has no negative volumes.
    pub mm_negative_layer_slices: Vec<crate::geometry::ExPolygons>,

    /// Total configured filament slots (PrintConfig::num_filaments at setup) —
    /// the `num_extruders` the MMU segmentation indexes by (MMS.cpp:2097).
    pub num_total_filaments: usize,
}

impl PrintObject {
    /// Create new PrintObject
    /// PrintObject.cpp:30
    pub fn new() -> Self {
        Self::with_canceled(Arc::new(AtomicBool::new(false)))
    }

    pub fn with_canceled(canceled: Arc<AtomicBool>) -> Self {
        Self {
            layers: Vec::new(),
            support_layers: Vec::new(),
            config: PrintObjectConfig::default(),
            print_config: Arc::new(PrintConfig::default()),
            slice_center_offset: (0.0, 0.0),
            model_object_id: 0,
            label_id: 0,
            state: 0,
            canceled,
            mesh: None,
            negative_mesh: None,
            slicing_params: crate::slicing::SlicingParams::default(),
            shared_regions: None,
            typed_slices: false,
            painted_submeshes: Vec::new(),
            l0_efc_done_in_slice: false,
            painted_submeshes_strict: Vec::new(),
            mm_negative_layer_slices: Vec::new(),
            paint_frame_offset: None,
            instance_shift: None,
            paint_tb_trafo: None,
            raw_local_mesh: None,
            slice_center_override: None,
            occlusion_place: None,
            occlusion_volumes: None,
            seam_facets: None,
            negative_volumes_raw: Vec::new(),
            num_total_filaments: 1,
        }
    }

    /// Create PrintObject with mesh and config
    /// PrintObject.cpp:30
    pub fn with_config(
        mesh: crate::triangle_mesh::TriangleMesh,
        config: PrintObjectConfig,
    ) -> Self {
        Self {
            layers: Vec::new(),
            support_layers: Vec::new(),
            config,
            print_config: Arc::new(PrintConfig::default()),
            slice_center_offset: (0.0, 0.0),
            model_object_id: 0,
            label_id: 0,
            state: 0,
            canceled: Arc::new(AtomicBool::new(false)),
            mesh: Some(mesh),
            negative_mesh: None,
            slicing_params: crate::slicing::SlicingParams::default(),
            shared_regions: None,
            typed_slices: false,
            painted_submeshes: Vec::new(),
            l0_efc_done_in_slice: false,
            painted_submeshes_strict: Vec::new(),
            mm_negative_layer_slices: Vec::new(),
            paint_frame_offset: None,
            instance_shift: None,
            paint_tb_trafo: None,
            raw_local_mesh: None,
            slice_center_override: None,
            occlusion_place: None,
            occlusion_volumes: None,
            seam_facets: None,
            negative_volumes_raw: Vec::new(),
            num_total_filaments: 1,
        }
    }

    /// Set shared regions
    /// Print.hpp:469-473
    pub fn set_shared_regions(&mut self, regions: Arc<PrintObjectRegions>) {
        self.shared_regions = Some(regions);
    }

    /// Set cancellation flag (called by Print when adding object)
    /// Print.cpp:440
    pub(crate) fn set_canceled(&mut self, canceled: Arc<AtomicBool>) {
        self.canceled = canceled;
    }

    /// Stamp the owning Print's config snapshot onto this object (called by
    /// Print at sync points: add_object and the top of process). The Arc is
    /// replaced wholesale, mirroring C++ where m_print is assigned once and
    /// the pointed-to config only mutates inside Print::apply.
    pub(crate) fn set_print_config(&mut self, print_config: Arc<PrintConfig>) {
        self.print_config = print_config;
    }

    /// Upward view to the owning Print, preserving the C++ call shape
    /// `object->print()->config()`.
    /// C++: `PrintType* print() { return m_print; }` (PrintBase.hpp:632)
    pub fn print(&self) -> PrintRef<'_> {
        PrintRef {
            config: &self.print_config,
        }
    }

    /// Object-level configuration, preserving the C++ call shape
    /// `object.config()`.
    /// C++: `const PrintObjectConfig& config() const { return m_config; }`
    /// (Print.hpp:369)
    pub fn config(&self) -> &PrintObjectConfig {
        &self.config
    }

    /// Collect the slicing parameters, to be used by variable layer thickness
    /// algorithm, by the interactive layer height editor and by the printing
    /// process itself.
    /// C++: `const SlicingParameters& slicing_parameters() const { return m_slicing_params; }`
    /// (Print.hpp:468)
    pub fn slicing_parameters(&self) -> &crate::slicing::SlicingParams {
        &self.slicing_params
    }

    /// Set the mesh for this object
    /// PrintObject.cpp:35
    pub fn set_mesh(&mut self, mesh: crate::triangle_mesh::TriangleMesh) {
        self.mesh = Some(mesh);
    }

    /// Get reference to the mesh
    /// PrintObject.cpp:40
    /// R704 — set the merged negative-volume mesh (see the field docs).
    pub fn set_negative_mesh(&mut self, mesh: crate::triangle_mesh::TriangleMesh) {
        self.negative_mesh = if mesh.is_empty() { None } else { Some(mesh) };
    }

    pub fn mesh(&self) -> Option<&crate::triangle_mesh::TriangleMesh> {
        self.mesh.as_ref()
    }

    /// Get reference to layers
    pub fn layers(&self) -> &[Layer] {
        &self.layers
    }

    /// Get mutable reference to layers
    pub fn layers_mut(&mut self) -> &mut [Layer] {
        &mut self.layers
    }

    /// Check if a step is done
    /// PrintObject.cpp:45
    pub fn is_step_done(&self, step: PrintObjectStep) -> bool {
        let step_bit = 1 << (step as u32);
        (self.state & step_bit) != 0
    }

    /// Mark step as done
    /// PrintObject.cpp:50
    fn set_step_done(&mut self, step: PrintObjectStep) {
        let step_bit = 1 << (step as u32);
        self.state |= step_bit;
    }

    /// Invalidate all steps
    /// PrintObject.cpp:55
    pub fn invalidate_all_steps(&mut self) {
        self.state = 0;
    }

    /// Slice mesh into layers using the real slicing algorithm
    /// PrintObjectSlice.cpp:791-843
    pub fn slice(&mut self) -> Result<()> {
        // Check if already sliced - avoid re-slicing
        // PrintObjectSlice.cpp:792-793
        if self.is_step_done(PrintObjectStep::Slice) {
            return Ok(());
        }

        // Get the mesh to slice
        // PrintObjectSlice.cpp:795
        let mesh = self
            .mesh
            .as_ref()
            .ok_or_else(|| Error::Slicing("No mesh to slice".into()))?;

        // Check for cancellation
        // PrintObjectSlice.cpp:796
        if self.canceled.load(Ordering::Relaxed) {
            return Err(Error::Cancelled);
        }

        // Clear any existing layers
        // PrintObjectSlice.cpp:800
        self.layers.clear();

        // Build slicing parameters from config
        // PrintObjectSlice.cpp:797-798
        let mut slicing_params = crate::slicing::SlicingParams::default();
        slicing_params.layer_height = self.config.layer_height;
        slicing_params.first_print_layer_height = self.config.first_layer_height;
        slicing_params.min_layer_height = self.config.layer_height * 0.75;
        slicing_params.max_layer_height = self.config.layer_height * 1.5;
        self.slicing_params = slicing_params.clone();

        // Create slicer with parameters
        // PrintObjectSlice.cpp:799
        let mut slicer = crate::slicer::Slicer::new(slicing_params);
        // PrintObjectSlice.cpp:144 — slice-contour simplification resolution:
        //   params_base.resolution = print_config.resolution <= 0.001 ? 0.0f : 0.0025;
        // (BBS: 0.0025mm safe to speed slicing; separate from arc-fit resolution.)
        let cfg_resolution = self.print_config.resolution;
        slicer.set_slice_resolution(if cfg_resolution <= 0.001 { 0.0 } else { 0.0025 });
        // R96 — thread the morphological closing radius (default 0.049) so
        // make_expolygons applies C++'s post-union offset2_ex(±scale(closing_radius)).
        // PrintObjectSlice passes params.closing_radius = print_config.slice_closing_radius.
        slicer.set_slice_closing_radius(self.config.slice_closing_radius);

        // R85 slice-frame centering: C++ m_center_offset = Point::new_scale(
        // bbox_center.x, bbox_center.y) (PrintObject.cpp:88), bbox = raw_bounding_box
        // (identity-trafo for slicer_cli STL). unscale(m_center_offset) = the mm
        // center applied by trafo_centered. new_scale truncates toward zero
        // (coord_t(x/SCALING_FACTOR)); replicate so the f64 mm value matches.
        let bb = mesh.compute_bounding_box();
        let sf = crate::libslic3r::SCALING_FACTOR;
        let bbcx = (bb.min.x as f64 + bb.max.x as f64) * 0.5;
        let bbcy = (bb.min.y as f64 + bb.max.y as f64) * 0.5;
        let cx_mm = (bbcx / sf).trunc() * sf; // unscale(new_scale(bbcx))
        let cy_mm = (bbcy / sf).trunc() * sf;
        let (cx_mm, cy_mm) = self.slice_center_override.unwrap_or((cx_mm, cy_mm));
        slicer.set_slice_center_offset(cx_mm, cy_mm);
        // Record for the export origin (consumed by Print after process): gcode =
        // unscale(centered) + center → C++'s raw frame (R81: m_origin = unscale(shift)
        // = center, instance=0). Only meaningful when SLICE_CENTER is on.
        self.slice_center_offset = (cx_mm, cy_mm);

        // Perform actual mesh slicing
        // PrintObjectSlice.cpp:801
        // R805 SLICE_VOLUME_TRAFO (faithful ON): slice the raw volume-local mesh
        // through the exact composed f32 transform (native make_trafo_for_slicing
        // of trafo_centered*get_matrix) instead of the pre-placed f64 mesh.
        let use_vol_trafo = crate::faithful_gate("SLICE_VOLUME_TRAFO")
            && self.paint_tb_trafo.is_some()
            && self.raw_local_mesh.is_some();
        if use_vol_trafo {
            slicer.set_frame_trafo(self.paint_tb_trafo);
        }
        let layers = slicer.slice_with_geometry(
            mesh,
            if use_vol_trafo { self.raw_local_mesh.as_ref() } else { None },
        )?;

        // Check for empty result
        // PrintObjectSlice.cpp:838-839
        if layers.is_empty() {
            return Err(Error::Slicing(
                "No layers were detected. You might want to repair your STL file(s) or check their size or thickness and retry.".into()
            ));
        }

        // Update layer IDs and relationships
        // PrintObjectSlice.cpp:820-836
        // Note: Layer IDs are already set by the Slicer
        // Layer references (upper/lower) are handled internally by Layer struct

        // Store the layers
        // PrintObjectSlice.cpp:802
        self.layers = layers;

        // C++ gives every Layer one LayerRegion per print-object region
        // (Layer::add_region for each region id during slicing — empty
        // LayerRegions are legal and expected; PrintObjectSlice.cpp:60-115
        // slices per region). The Rust slicer creates only region 0, so when
        // painted regions are declared (install_painted_regions) pad each
        // layer up to num_printing_regions() with empty LayerRegions — the
        // region loops (`for region_id in 0..num_printing_regions()`) index
        // `layer.regions()[region_id]` directly and would OOB otherwise.
        // apply_mm_segmentation (campaign layer 4) later moves painted
        // surfaces into these.
        let num_regions = self.num_printing_regions();
        if num_regions > 1 {
            for (layer_id, layer) in self.layers.iter_mut().enumerate() {
                while layer.regions().len() < num_regions {
                    let region_id = layer.regions().len();
                    layer.add_region(crate::layer::LayerRegion::new(layer_id, region_id));
                }
            }
        }

        // R704 — NEGATIVE VOLUMES. C++ slices them like any other volume
        // (`model_volume_needs_slicing`, PrintObjectSlice.cpp:110) and then, in
        // `slices_to_regions` (:403-421), subtracts each from every PRECEDING
        // non-negative region: `diff_ex(region2.expolygons, negative.expolygons)`
        // gated on `overlap_in_xy`. This is the Tier-1 shape of that: slice the
        // merged negative mesh at each layer's own `slice_z` — in the SAME frame
        // (R704 made `slice_at_z` honour the slice-frame center_offset) — and
        // diff it out of every region's surfaces.
        //
        // Placed BEFORE `make_slices` and before SLICEPTS bracket A, matching
        // C++ where the subtraction happens inside slice_volumes, upstream of
        // both. The acceptance test is arithmetic and pre-registered (R700):
        // bracket-A holes 45 -> 97, five 7.53532 mm2 holes on each of layers
        // 0-9 plus two on 207/208.
        // R704 — SHIPPED DEFAULT-ON (`=0` to disable). Bracket-A holes go 45 -> 97,
        // matching C++ LAYER FOR LAYER on all 36 layers. Scored on both metrics in
        // ABSOLUTE matched lines (R599 — the rate has a moving denominator):
        // content 709,080 -> 710,614 (+1,534), IN ORDER 469,489 -> 469,628 (+139),
        // and our body-line count moves 2,511,238 -> 2,525,970, narrowing the gap
        // to C++'s 2,781,977 by 14,732. Both rates dip 0.11pp purely because the
        // denominator grew faster than the matches; every absolute measure improves
        // and the geometry is now the same geometry C++ slices.
        if crate::faithful_gate("NEGATIVE_VOLUMES") {
            if let Some(neg) = self.negative_mesh.clone() {
                use crate::surface::{Surface, SurfaceType};
                use crate::surface_collection::SurfaceCollection;
                // Only layers within the negative geometry's own Z span can be
                // cut — C++ never slices a volume outside its range either. On
                // Majora this takes 656 slice_at_z calls down to ~64.
                let nbb = neg.compute_bounding_box();
                let (nz0, nz1) = (nbb.min.z as f64, nbb.max.z as f64);
                let mut n_layers_cut = 0usize;
                let mut n_pieces = 0usize;
                // R805 NEGVOL_EXACT_FRAME: slice each negative volume from its raw
                // local vertices through its exact composed f32 frame.
                let neg_exact = crate::faithful_gate("NEGVOL_EXACT_FRAME") && !self.negative_volumes_raw.is_empty();
                let neg_raw = if neg_exact { self.negative_volumes_raw.clone() } else { Vec::new() };
                let neg_res = if crate::faithful_gate("SLICE_SIMPLIFY") && self.print_config.resolution > 0.001 { 0.0025 } else { 0.0 };
                let neg_closing = self.config.slice_closing_radius as f32;
                // R770 — keep the raw per-layer negative slices for the MM
                // segmentation's fill/clip-back flow (cpp neg_slices, MMS.cpp:2217).
                self.mm_negative_layer_slices =
                    vec![crate::geometry::ExPolygons::new(); self.layers.len()];
                for (layer_i, layer) in self.layers.iter_mut().enumerate() {
                    if layer.slice_z < nz0 || layer.slice_z > nz1 {
                        continue;
                    }
                    // R804 NEGVOL_SLICE_PARAMS (faithful ON): negative volumes go
                    // through the same simplifying/closing slice parameters as
                    // the model parts (PrintObjectSlice.cpp:156-158).
                    let cut_res = if neg_exact {
                        let mut all: crate::geometry::ExPolygons = Vec::new();
                        for (m, t16) in &neg_raw {
                            let mut params = crate::triangle_mesh_slicer::MeshSlicingParamsEx::default();
                            params.resolution = neg_res;
                            params.closing_radius = neg_closing;
                            params.base.frame_trafo = Some(*t16);
                            let mut ls = crate::triangle_mesh_slicer::slice_mesh_ex(m, &[layer.slice_z as f32], &params, &|| {});
                            if !ls.is_empty() {
                                all.append(&mut ls[0]);
                            }
                        }
                        Ok(all)
                    } else if crate::faithful_gate("NEGVOL_SLICE_PARAMS") {
                        slicer.slice_at_z_faithful(&neg, layer.slice_z)
                    } else {
                        slicer.slice_at_z(&neg, layer.slice_z)
                    };
                    let cut = match cut_res {
                        Ok(c) if !c.is_empty() => c,
                        _ => continue,
                    };
                    self.mm_negative_layer_slices[layer_i] = cut.clone();
                    if crate::probe_enabled("NEGPROBE") && layer_i < 2 {
                        for (ci, ex) in cut.iter().enumerate() {
                            let bb = crate::geometry::BoundingBox::from_points(&ex.contour.points);
                            eprintln!("[NEGCUT] lid={} piece={} pts={} holes={} area={:.0} bb=({},{})-({},{}) ccw={}", layer_i, ci, ex.contour.points.len(), ex.holes.len(), ex.contour.area(), bb.min.x, bb.min.y, bb.max.x, bb.max.y, ex.contour.is_counter_clockwise());
                        }
                        if let Some(r0) = layer.regions().first() {
                            if let Some(sf) = r0.slices.surfaces.first() {
                                let bb = crate::geometry::BoundingBox::from_points(&sf.expolygon.contour.points);
                                eprintln!("[NEGCUT] lid={} src pts={} area={:.0} bb=({},{})-({},{})", layer_i, sf.expolygon.contour.points.len(), sf.expolygon.contour.area(), bb.min.x, bb.min.y, bb.max.x, bb.max.y);
                            }
                        }
                    }
                    n_layers_cut += 1;
                    n_pieces += cut.len();
                    for region in layer.regions_mut().iter_mut() {
                        if region.slices.surfaces.is_empty() {
                            continue;
                        }
                        let src: crate::geometry::ExPolygons = region
                            .slices
                            .surfaces
                            .iter()
                            .map(|sf| sf.expolygon.clone())
                            .collect();
                        let kept = crate::clipper_utils::difference(&src, &cut);
                        let mut coll = SurfaceCollection::new();
                        for ex in &kept {
                            coll.push(Surface::new(SurfaceType::Internal, ex.clone()));
                        }
                        region.set_slices(coll);
                    }
                }
                if crate::probe_enabled("NEGPROBE") {
                    eprintln!(
                        "[NEGPROBE] negative mesh {} triangles; cut {} layers, {} pieces total; neg z-span {:.4}..{:.4}; layer0 slice_z {:.4}",
                        neg.triangle_count(),
                        n_layers_cut,
                        n_pieces,
                        nz0,
                        nz1,
                        self.layers.first().map(|l| l.slice_z).unwrap_or(-1.0),
                    );
                    for (li, layer) in self.layers.iter().enumerate().take(4) {
                        let (mut nh, mut ah) = (0usize, 0.0f64);
                        for region in layer.regions() {
                            for sf in &region.slices.surfaces {
                                nh += sf.expolygon.holes.len();
                                for h in &sf.expolygon.holes {
                                    ah += h.area().abs();
                                }
                            }
                        }
                        eprintln!(
                            "[NEGPROBE] lid={} slice_z={:.4} holes={} hole_area={:.0}",
                            li, layer.slice_z, nh, ah
                        );
                    }
                }
            }
        }

        // Build lslices for each layer (union of all region slices)
        // C++: PrintObjectSlice.cpp calls layer->make_slices() which populates
        // lslices from region slices. This is needed for detect_surfaces_type()
        // to correctly diff between adjacent layers.
        let __t_ms = std::time::Instant::now();
        // R675 — bracket the slicer→region path on OUR side. R674 measured
        // 50.956 points-per-contour at `generate_arachne`'s `surface` against
        // C++'s 84.158. This counts the SAME unit at the two points before it:
        // "A region slices (pre make_slices)" is what the slicer produced and
        // stored per region; "B lslices (post make_slices)" is the per-layer
        // union. If A already reads ~51 the loss is in the SLICER itself,
        // despite R671 confirming its DP tolerance is faithful; if A reads ~84
        // the loss is in this assembly.
        if crate::probe_enabled("SLICEPTS") {
            let mut f = |tag: &str, get: &dyn Fn(&crate::layer::Layer) -> (u64, u64)| {
                let (mut c, mut p) = (0u64, 0u64);
                for layer in self.layers.iter() {
                    let (dc, dp) = get(layer);
                    c += dc;
                    p += dp;
                }
                // R676 — print the LAYER population too. R675's bracket A read 1,391
                // contours across a 4,720-layer model (0.29 per layer), which is not
                // credible; without the layer count there was no way to see that the
                // probe might be reading a field the slicer had not filled.
                let n_layers = self.layers.len();
                let n_nonempty = self
                    .layers
                    .iter()
                    .filter(|l| l.regions().iter().any(|r| !r.slices.surfaces.is_empty()))
                    .count();
                eprintln!(
                    "[SLICEPTS] {tag:34} layers={n_layers} nonempty={n_nonempty} contours={c} points={p} \
                     points_per_contour={:.3} points_per_layer={:.3}",
                    p as f64 / c.max(1) as f64,
                    p as f64 / n_layers.max(1) as f64
                );
            };
            f("A region slices (pre make_slices)", &|layer: &crate::layer::Layer| {
                let (mut c, mut p) = (0u64, 0u64);
                for r in layer.regions() {
                    for sf in r.slices.surfaces.iter() {
                        c += 1 + sf.expolygon.holes.len() as u64;
                        p += sf.expolygon.contour.points.len() as u64
                            + sf.expolygon.holes.iter().map(|h| h.points.len() as u64).sum::<u64>();
                    }
                }
                (c, p)
            });
            // R677 — the (layer, region) pair and surface populations at A, so C
            // and SURFPROBE can be checked against them instead of against a
            // call count that revisits inflate (R676).
            let (mut pairs, mut surfaces) = (0u64, 0u64);
            let mut area = 0.0f64;
            // R679 — UNITS TRAP: `crate::SCALING_FACTOR` (lib.rs:489) is 100_000.0,
            // while `crate::libslic3r::SCALING_FACTOR` (libslic3r.rs:19, the C++
            // mirror of libslic3r.h:58) is 0.00001. They are RECIPROCALS and both
            // are in scope. `area()` returns scaled units^2, so mm^2 is a DIVISION
            // by the lib.rs constant — the same convention MMS_PARTITION uses.
            let sf2 = crate::SCALING_FACTOR * crate::SCALING_FACTOR;
            for layer in self.layers.iter() {
                for r in layer.regions() {
                    if r.slices.surfaces.is_empty() {
                        continue;
                    }
                    pairs += 1;
                    surfaces += r.slices.surfaces.len() as u64;
                    for sf in r.slices.surfaces.iter() {
                        area += sf.expolygon.area() / sf2;
                    }
                }
            }
            eprintln!("[SLICEPTS] A population: pairs={pairs} surfaces={surfaces} area_mm2={area:.3}");
            // R700 — is the bracket-A hole deficit UNIFORM or CONCENTRATED?
            // Ours carries 45 holes to C++'s 97 across 656 layers at IDENTICAL
            // surface counts (1,346). 52 missing holes over 656 layers is
            // 0.08/layer, which cannot be a uniform per-layer shortfall — so the
            // histogram distinguishes "we drop small holes everywhere" (a
            // tolerance/area-filter question) from "specific layers lose their
            // interior rings" (specific geometry).
            {
                let mut per_layer: Vec<(usize, u64)> = Vec::new();
                for (li, layer) in self.layers.iter().enumerate() {
                    let mut h = 0u64;
                    for r in layer.regions() {
                        for sf in r.slices.surfaces.iter() {
                            h += sf.expolygon.holes.len() as u64;
                        }
                    }
                    if h > 0 {
                        per_layer.push((li, h));
                    }
                }
                let total: u64 = per_layer.iter().map(|(_, h)| *h).sum();
                eprintln!(
                    "[SLICEPTS] A holes: total={total} layers_with_holes={} of {} \
                     max_on_one_layer={}",
                    per_layer.len(),
                    self.layers.len(),
                    per_layer.iter().map(|(_, h)| *h).max().unwrap_or(0),
                );
                let head: Vec<String> = per_layer
                    .iter()
                    .take(40)
                    .map(|(li, h)| format!("{li}:{h}"))
                    .collect();
                eprintln!("[SLICEPTS] A holes by layer: {}", head.join(" "));
            }
        }
        for layer in &mut self.layers {
            layer.make_slices();
        }
        if crate::probe_enabled("LSLPROBE") {
            // R806 — lslices AFTER the post-MMS make_slices (the LSL dump above is pre-rebuild).
            for (li, layer) in self.layers.iter().enumerate() {
                let (mut n, mut pts, mut sx, mut sy) = (0usize, 0usize, 0i64, 0i64);
                for ex in &layer.lslices {
                    n += 1 + ex.holes.len();
                    for pt in &ex.contour.points { pts += 1; sx += pt.x(); sy += pt.y(); }
                    for h in &ex.holes { for pt in &h.points { pts += 1; sx += pt.x(); sy += pt.y(); } }
                }
                eprintln!("LSL2 li={} n={} pts={} sx={} sy={}", li, n, pts, sx, sy);
            }
        }
        if crate::probe_enabled("SLICEPTS") {
            let (mut c, mut p) = (0u64, 0u64);
            for layer in self.layers.iter() {
                for ex in &layer.lslices {
                    c += 1 + ex.holes.len() as u64;
                    p += ex.contour.points.len() as u64
                        + ex.holes.iter().map(|h| h.points.len() as u64).sum::<u64>();
                }
            }
            let n_layers = self.layers.len();
            let n_nonempty = self.layers.iter().filter(|l| !l.lslices.is_empty()).count();
            eprintln!(
                "[SLICEPTS] {:34} layers={n_layers} nonempty={n_nonempty} contours={c} points={p} \
                 points_per_contour={:.3} points_per_layer={:.3}",
                "B lslices (post make_slices)",
                p as f64 / c.max(1) as f64,
                p as f64 / n_layers.max(1) as f64
            );
        }
        if crate::probe_enabled("SLICE_PHASE_TIMING") {
            eprintln!("      slice(): make_slices {:.2}s", __t_ms.elapsed().as_secs_f64());
        }

        // Painted multi-material segmentation (campaign layer 4): compute the
        // per-layer per-extruder painted areas from the painted sub-meshes and
        // move the corresponding surfaces out of region 0 into the painted
        // regions. C++: slice_volumes → apply_mm_segmentation
        // (PrintObjectSlice.cpp:1161 / :845-925), segmentation itself in
        // MultiMaterialSegmentation.cpp:2095.
        //
        // Deliberately AFTER make_slices: the painted pieces + the region-0
        // remainder tile the pre-split region-0 area exactly, so lslices (the
        // per-layer union across regions) is geometrically identical either
        // way — but unioning the post-split regions re-walks every jagged
        // segmentation border through the slow offset path (observed >30min
        // release wall on Majora). Building lslices from the single pre-split
        // region first is byte-equivalent and cheap.
        if crate::probe_enabled("LSLPROBE") {
            // R804 pre-MMS region census (cpp twin: PRESL before apply_mm_segmentation).
            for (li, layer) in self.layers.iter().enumerate() {
                for (ri, lr) in layer.regions().iter().enumerate() {
                    let (mut rn, mut rp, mut rx, mut ry) = (0usize, 0usize, 0i64, 0i64);
                    for sf in &lr.slices.surfaces {
                        rn += 1 + sf.expolygon.holes.len();
                        for pt in &sf.expolygon.contour.points { rp += 1; rx += pt.x(); ry += pt.y(); }
                        for h in &sf.expolygon.holes { for pt in &h.points { rp += 1; rx += pt.x(); ry += pt.y(); } }
                    }
                    eprintln!("PRESL li={} r={} ns={} n={} pts={} sx={} sy={}", li, ri, lr.slices.surfaces.len(), rn, rp, rx, ry);
                }
            }
        }
        if num_regions > 1 && !self.painted_submeshes.is_empty() {
            self.apply_mm_segmentation_tier1()?;
            // R806 LSLICES_POST_MMS (faithful ON): native rebuilds lslices from the
            // post-segmentation regions (PrintObjectSlice.cpp:1355 make_slices →
            // union_safety_offset_ex over all regions). The pre-split lslices are
            // NOT byte-equivalent: the safety-offset union keeps region-boundary
            // vertices (Majora layer 2: 1,403 v 1,845 points), and lslices feed
            // overhang/bridge detection and the fills.
            if crate::faithful_gate("LSLICES_POST_MMS") {
                let __t = std::time::Instant::now();
                for layer in &mut self.layers {
                    layer.make_slices();
                }
                if crate::probe_enabled("SLICE_PHASE_TIMING") {
                    eprintln!("[SLICE_PHASE_TIMING] lslices post-MMS rebuild {:.3}s", __t.elapsed().as_secs_f64());
                }
                if crate::probe_enabled("LSLPROBE") {
                    for (li, layer) in self.layers.iter().enumerate() {
                        let (mut n, mut pts, mut sx, mut sy) = (0usize, 0usize, 0i64, 0i64);
                        for ex in &layer.lslices {
                            n += 1 + ex.holes.len();
                            for pt in &ex.contour.points { pts += 1; sx += pt.x(); sy += pt.y(); }
                            for h in &ex.holes { for pt in &h.points { pts += 1; sx += pt.x(); sy += pt.y(); } }
                        }
                        eprintln!("LSL3 li={} n={} pts={} sx={} sy={}", li, n, pts, sx, sy);
                    }
                }
            }
        }
        if crate::probe_enabled("LSLPROBE") {
            // R804 per-layer lslices + per-region slice census (cpp twin: PrintObjectSlice.cpp).
            for (li, layer) in self.layers.iter().enumerate() {
                let (mut n, mut pts, mut sx, mut sy) = (0usize, 0usize, 0i64, 0i64);
                for ex in &layer.lslices {
                    n += 1 + ex.holes.len();
                    for pt in &ex.contour.points { pts += 1; sx += pt.x(); sy += pt.y(); }
                    for h in &ex.holes { for pt in &h.points { pts += 1; sx += pt.x(); sy += pt.y(); } }
                }
                eprintln!("LSL li={} z={:.4} n={} pts={} sx={} sy={}", li, layer.print_z, n, pts, sx, sy);
                for (ri, lr) in layer.regions().iter().enumerate() {
                    let (mut rn, mut rp, mut rx, mut ry) = (0usize, 0usize, 0i64, 0i64);
                    for sf in &lr.slices.surfaces {
                        rn += 1 + sf.expolygon.holes.len();
                        for pt in &sf.expolygon.contour.points { rp += 1; rx += pt.x(); ry += pt.y(); }
                        for h in &sf.expolygon.holes { for pt in &h.points { rp += 1; rx += pt.x(); ry += pt.y(); } }
                    }
                    eprintln!("REGSL li={} r={} ns={} n={} pts={} sx={} sy={}", li, ri, lr.slices.surfaces.len(), rn, rp, rx, ry);
                    if let Ok(d) = std::env::var("LSLDUMP") {
                        if d == format!("{},{}", li, ri) {
                            for (si, sf) in lr.slices.surfaces.iter().enumerate() {
                                for (k, pt) in sf.expolygon.contour.points.iter().enumerate() {
                                    eprintln!("LSLD REGSL {} c {} {} {}", si, k, pt.x(), pt.y());
                                }
                                for (hi, h) in sf.expolygon.holes.iter().enumerate() {
                                    for (k, pt) in h.points.iter().enumerate() {
                                        eprintln!("LSLD REGSL {} h{} {} {} {}", si, hi, k, pt.x(), pt.y());
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        // R677 — bracket C. A and B are both PRE-segmentation, so neither is
        // comparable to SURFPROBE (R675: 1,391 contours become 26,122 here).
        // C counts the SAME field as A on the SAME layer population, after the
        // split, so A -> C isolates what segmentation does and C is the first
        // bracket that shares a population with `generate_arachne`'s input.
        // The (layer, region) pair count is printed so the next bracket can be
        // checked against it rather than against a raw call count (R676).
        // R679 — + total AREA. C++ conserves it exactly across this stage
        // (1,189,508.593 -> 1,189,508.418 mm2), which is what a partition must do.
        // A matching area with a mismatched piece count means the CLEANUP tolerance
        // is off; a mismatched area means the PARTITION is misplaced.
        if crate::probe_enabled("SLICEPTS") {
            let (mut c, mut p, mut pairs, mut surfaces) = (0u64, 0u64, 0u64, 0u64);
            let mut area = 0.0f64;
            let sf2 = crate::SCALING_FACTOR * crate::SCALING_FACTOR;
            for layer in self.layers.iter() {
                for r in layer.regions() {
                    if r.slices.surfaces.is_empty() {
                        continue;
                    }
                    pairs += 1;
                    for sf in r.slices.surfaces.iter() {
                        surfaces += 1;
                        c += 1 + sf.expolygon.holes.len() as u64;
                        p += sf.expolygon.contour.points.len() as u64
                            + sf.expolygon.holes.iter().map(|h| h.points.len() as u64).sum::<u64>();
                        area += sf.expolygon.area() / sf2;
                    }
                }
            }
            let n_layers = self.layers.len();
            let n_nonempty = self
                .layers
                .iter()
                .filter(|l| l.regions().iter().any(|r| !r.slices.surfaces.is_empty()))
                .count();
            eprintln!(
                "[SLICEPTS] {:34} layers={n_layers} nonempty={n_nonempty} pairs={pairs} \
                 surfaces={surfaces} contours={c} points={p} \
                 points_per_contour={:.3} points_per_layer={:.3} area_mm2={area:.3}",
                "C region slices (post segmentation)",
                p as f64 / c.max(1) as f64,
                p as f64 / n_layers.max(1) as f64
            );
        }

        // Check for cancellation after slicing
        // PrintObjectSlice.cpp:803
        if self.canceled.load(Ordering::Relaxed) {
            return Err(Error::Cancelled);
        }

        // Wire the config hierarchy onto the freshly created layers so that
        // `layer.object().print().config()` / `layerm.region().config()` work.
        // Sync point: end of PrintObject::slice — this is the load-bearing
        // stamp (make_perimeters calls self.slice() internally). C++ does not
        // need this because Layer/LayerRegion carry parent pointers from their
        // ctors (Layer.hpp:125, Layer.hpp:309).
        self.wire_layer_hierarchy();

        // L0_EFC_MULTI_REGION (on): PrintObjectSlice.cpp:2331-2378 and 2391-2408.
        // slice_volumes compensates the Elephant foot after apply_mm_segmentation
        // (:2202), so a first layer with several regions takes the multi-region
        // branch: the merged layer is compensated once and every region is
        // trimmed by the result. =0 compensates each region's slices by
        // themselves in make_perimeters (the single-region branch, :2295-2313).
        if crate::faithful_gate("L0_EFC_MULTI_REGION")
            && crate::faithful_gate("L0_EFC")
            && self.config.raft_layers == 0
            && self.config.elephant_foot_compensation > 0.0
            && self.layers.first().is_some_and(|l| l.regions().len() > 1)
        {
            use crate::clipper_utils::OffsetJoinType;
            // :2277-2279 `float(scale_(elefant_foot_compensation))`, :2368 `unscale<double>(elfoot)`.
            let elfoot = crate::scaled_f(self.config.elephant_foot_compensation) as f32;
            // :2360 `float(scale_(slice_closing_radius) * 1.5)`
            let eps = (crate::scaled_f(self.config.slice_closing_radius) * 1.5) as f32 as f64;
            let layer = &mut self.layers[0];
            let layer_height = layer.height;
            if let Ok(ext_flow) = layer.regions()[0].flow(crate::flow::FlowRole::ExternalPerimeter, layer_height) {
                // Layer::merged(eps) (Layer.cpp:150-170).
                let mut polygons: crate::geometry::Polygons = Vec::new();
                for layerm in layer.regions() {
                    let config = layerm.region().config();
                    if config.bottom_solid_layers > 0
                        || config.top_solid_layers > 0
                        || config.fill_density > 0.0
                        || config.perimeters > 0
                    {
                        // offset(Surfaces, delta): expolygons_offset (ClipperUtils.cpp:537-547).
                        let surf_ex = crate::surface::to_expolygons(&layerm.slices.surfaces);
                        let (raw, collected) =
                            crate::clipper_utils::offset_expolygons_clib_raw_scaled_counted(
                                &surf_ex, eps, OffsetJoinType::Miter);
                        if collected > 1 && eps > 0.0 {
                            polygons.extend(crate::clipper_utils::union_flat_clib(&raw));
                        } else {
                            polygons.extend(raw);
                        }
                    }
                }
                let merged = crate::clipper_utils::polytree_ex_clib(&polygons);
                // :2362 lslices_1st_layer = offset_ex(layer->merged(eps), -eps);
                let lslices_1st_layer =
                    crate::clipper_utils::offset_expolygons_clib_scaled(&merged, -eps, OffsetJoinType::Miter);
                // :2363-2364
                let trimming =
                    crate::elephant_foot_compensation::elephant_foot_compensation_expolygons_with_flow(
                        &lslices_1st_layer,
                        &ext_flow,
                        elfoot as f64 * crate::libslic3r::SCALING_FACTOR,
                    );
                let trimming_p = crate::geometry::to_polygons(&trimming);
                // :2372-2378 trim surfaces
                for region in layer.regions_mut().iter_mut() {
                    let contour_exp = crate::surface::to_expolygons(&region.slices.surfaces);
                    region.slices.surfaces =
                        crate::clipper_utils::intersection_ex_expolygons_polygons(&contour_exp, &trimming_p)
                            .into_iter()
                            .map(|ex| crate::surface::Surface {
                                expolygon: ex,
                                surface_type: crate::surface::SurfaceType::Internal,
                                thickness: -1.0,
                                thickness_layers: 1,
                                bridge_angle: None,
                                extra_perimeters: 0,
                            })
                            .collect();
                }
                // :2384
                layer.make_slices();
                // :2391-2408 the first layer keeps the uncompensated lslices,
                // sorted along the shortest path.
                let ordering_points: Vec<crate::geometry::Point> = lslices_1st_layer
                    .iter()
                    .map(|ex| ex.contour.points[0])
                    .collect();
                let order = crate::shortest_path::chain_points(&ordering_points, None);
                layer.lslices = order.into_iter().map(|i| lslices_1st_layer[i].clone()).collect();
                self.l0_efc_done_in_slice = true;
            }
        }

        // Mark step as complete
        // PrintObjectSlice.cpp:843
        // R807: PrintObjectSlice.cpp:834 `layer.backup_untyped_slices()` — the
        // per-region untyped slices native keeps for restore_untyped_slices and
        // for the first-layer tool order (ToolOrdering.cpp:539 reads raw_slices).
        // Rust never called it, so `raw_slices` stayed empty and Majora's first
        // layer found no target. `BACKUP_UNTYPED_SLICES=0` restores that.
        if crate::faithful_gate("BACKUP_UNTYPED_SLICES") {
            for layer in &mut self.layers {
                layer.backup_untyped_slices();
            }
        }
        self.set_step_done(PrintObjectStep::Slice);
        Ok(())
    }

    /// Stamp the config-hierarchy Arcs onto every Layer (and support Layer)
    /// and their LayerRegions, so that the C++ call shapes
    /// `layer->object()->print()->config()` and `layerm->region().config()`
    /// work without parent pointers.
    ///
    /// Each LayerRegion's region Arc is looked up via its region id into
    /// shared_regions.all_regions — the very same Arcs held by
    /// Print::print_regions, so the PrintRegion identity stays unified.
    ///
    /// Called at sync points (end of PrintObject::slice; support layers are
    /// created inside generate_support_material AFTER this pass and get
    /// stamped again at its end). Faithful because in C++ configs only mutate
    /// inside Print::apply and any diff there invalidates posSlice, so between
    /// sync points the snapshots cannot diverge.
    ///
    /// INVARIANT: Arcs are replaced wholesale (Arc::new + clone), NEVER
    /// mutated in place via Arc::make_mut/get_mut — that would fork the share.
    /// Split each layer's region-0 slices into the painted print regions using
    /// the MMU segmentation of the painted sub-meshes.
    ///
    /// Tier-1 shape of C++ `apply_mm_segmentation` (PrintObjectSlice.cpp:845-925)
    /// over the single merged volume: the segmentation
    /// (`multi_material_segmentation_by_painting`, MMS.cpp:2095) yields per-layer
    /// per-color ExPolygons (color 0 = unpainted); for each painted extruder we
    /// intersect its area with the layer's region-0 slices, hand that to the
    /// painted region (`1 + index` in ascending painted-extruder order —
    /// mirrors Print::install_painted_regions), and keep the remainder in
    /// region 0. C++ walks sorted painted_regions and steals via
    /// `intersection_ex` per parent region the same way; with one parent the
    /// two are equivalent.
    fn apply_mm_segmentation_tier1(&mut self) -> Result<()> {
        use crate::surface::{Surface, SurfaceType};
        use crate::surface_collection::SurfaceCollection;

        let num_layers = self.layers.len();
        // Per-layer merged slices + zs for the segmentation input.
        let mut layer_slices: Vec<crate::geometry::ExPolygons> = Vec::with_capacity(num_layers);
        let mut layer_zs: Vec<f64> = Vec::with_capacity(num_layers);
        for layer in &self.layers {
            let ex: crate::geometry::ExPolygons = layer.regions()[0]
                .slices
                .surfaces
                .iter()
                .map(|s| s.expolygon.clone())
                .collect();
            layer_slices.push(ex);
            layer_zs.push(layer.slice_z);
        }

        // MMS.cpp:2097 — num_extruders from the filament vector length.
        let num_extruders = self.num_total_filaments.max(
            self.painted_submeshes
                .iter()
                .map(|(e, _)| *e as usize)
                .max()
                .unwrap_or(1),
        );

        // mmu_segmented_region_max_width / interlocking depth: not carried in
        // the Tier-1 object config → 0.0 (cut step skipped), matching the C++
        // gate `max_width > 0 || interlocking_depth > 0` (MMS.cpp:2377-2381).
        // MMS.cpp:2291 center_offset must match the XY shift the slicer applied to the
        // slices (slicer.rs `want_center`): scaled bbox-XY center when centering, else
        // (0,0). slice_center_offset is the unscaled (mm) center (self set at slice());
        // new_scale() re-scales it to the painted-line coordinate space. Keeping this in
        // lock-step with the slicer's decision is what preserves painted-region overlap.
        // R787 — when the paint frame is active, paint_frame_offset IS native's
        // center_offset (raw-bbox center, scaled trunc); the projection does
        // line_to_test.translate(-center_offset) exactly like MMS.cpp:2373.
        let mms_center_offset = if let Some((ox, oy)) = self.paint_frame_offset {
            crate::geometry::Point::new(ox, oy)
        } else if crate::faithful_gate("SLICE_CENTER")
            && (self.slice_center_offset.0 != 0.0 || self.slice_center_offset.1 != 0.0)
        {
            crate::geometry::Point::new_scale(
                self.slice_center_offset.0,
                self.slice_center_offset.1,
            )
        } else {
            crate::geometry::Point::new(0, 0)
        };
        // R768 (gate MMS_TOPBOT) — inputs for the top/bottom painted-surface
        // propagation (`mmu_segmentation_top_and_bottom_layers`, MMS.cpp:2393).
        // The C++ calls it unconditionally; the ctx is None only when the loader
        // provided no strict sub-meshes (unpainted object) or the gate is off.
        let tb_region_cfgs: Vec<crate::multi_material_segmentation::TopBottomRegionCfg> = self
            .shared_regions
            .as_ref()
            .map(|sr| {
                sr.all_regions
                    .iter()
                    .map(|r| {
                        let c = r.config();
                        crate::multi_material_segmentation::TopBottomRegionCfg {
                            wall_filament: c.wall_filament,
                            outer_wall_line_width: c.outer_wall_line_width,
                            top_color_penetration_layers: c.top_color_penetration_layers,
                            bottom_color_penetration_layers: c.bottom_color_penetration_layers,
                            gap_fill_speed: c.gap_fill_speed,
                        }
                    })
                    .collect()
            })
            .unwrap_or_default();
        let tb_layer_heights: Vec<f64> = self.layers.iter().map(|l| l.height).collect();
        let tb_trafo = self.mesh().map(|mesh| {
            // The FRAME_UNIFY trafo derivation (see slice_mesh_its): C++ centers the
            // volume on its f32 bbox; volume_trafo = trafo_centered() * get_matrix()
            // nets to translate(quantization residue XY, bbox center Z) applied to
            // (placed - voff) — byte-matched to the object slicing path.
            let (mut xlo, mut ylo, mut zlo) = (f32::MAX, f32::MAX, f32::MAX);
            let (mut xhi, mut yhi, mut zhi) = (f32::MIN, f32::MIN, f32::MIN);
            for p in mesh.vertices() {
                let (x, y, z) = (p.x as f32, p.y as f32, p.z as f32);
                xlo = xlo.min(x);
                xhi = xhi.max(x);
                ylo = ylo.min(y);
                yhi = yhi.max(y);
                zlo = zlo.min(z);
                zhi = zhi.max(z);
            }
            let center_x = ((xlo + xhi) * 0.5) as f64;
            let center_y = ((ylo + yhi) * 0.5) as f64;
            let center_z = ((zlo + zhi) * 0.5) as f64;
            let sf = crate::libslic3r::SCALING_FACTOR;
            let quantize = |v: f64| -> f64 { ((v / sf) as i64) as f64 * sf };
            let res_x = center_x - quantize(center_x);
            let res_y = center_y - quantize(center_y);
            let trafo16: [f64; 16] = [
                1.0, 0.0, 0.0, res_x,
                0.0, 1.0, 0.0, res_y,
                0.0, 0.0, 1.0, center_z,
                0.0, 0.0, 0.0, 1.0,
            ];
            (trafo16, (center_x, center_y, center_z))
        });
        let tb_ctx = match (&tb_trafo, crate::faithful_gate("MMS_TOPBOT")) {
            (Some((trafo16, voff)), true) if !self.painted_submeshes_strict.is_empty() => {
                // R788 — with the paint frame active the strict meshes are
                // volume-LOCAL and the trafo is native's volume_trafo.
                let (t16, vo) = if let Some(t) = self.paint_tb_trafo {
                    (t, (0.0, 0.0, 0.0))
                } else {
                    (*trafo16, *voff)
                };
                Some(crate::multi_material_segmentation::TopBottomCtx {
                    strict_submeshes: &self.painted_submeshes_strict,
                    zs: layer_zs.iter().map(|&z| z as f32).collect(),
                    layer_heights: &tb_layer_heights,
                    region_configs: &tb_region_cfgs,
                    default_line_width: self.config.line_width,
                    nozzle_diameter: self.print_config.nozzle_diameter,
                    trafo16: t16,
                    voff: vo,
                })
            }
            _ => None,
        };
        // R785 FINDING (code removed as insufficient): native projects painted
        // facets through Transform3f(trafo()*get_matrix()) — for the Majora 3MF
        // this is a FULL 3D ROTATION (TRDUMP: x->-z tilt combo, translation
        // (0,0,98.46), center_offset (2248446,1698337)), not a translate. The
        // f32 rotation matmul over volume-LOCAL verts (the R85 Eigen wall)
        // cannot be reproduced from our f64-baked placed vertices — a plain
        // center round-trip moved only 9/2425 painted lines. The faithful fix
        // needs (a) the raw local painted submeshes + the instance/volume
        // matrices threaded from the 3MF loader and (b) an eigen-shim entry for
        // the plain f32 affine apply. Until then the ±1-unit painted-line
        // endpoint drift (PLDUMP lid385: 296/2425 byte-common) remains the
        // Majora colored-boundary noise floor.
        let segmented = crate::multi_material_segmentation::multi_material_segmentation_by_painting_tier1(
            &layer_slices,
            &layer_zs,
            &self.painted_submeshes,
            num_extruders,
            0.0,
            0.0,
            mms_center_offset,
            tb_ctx.as_ref(),
            if self.mm_negative_layer_slices.is_empty() {
                None
            } else {
                Some(self.mm_negative_layer_slices.as_slice())
            },
        );

        if crate::probe_enabled("LSLPROBE") {
            for (li, per_ext) in segmented.iter().enumerate() {
                for (e, exs) in per_ext.iter().enumerate() {
                    let (mut rn, mut rp, mut rx, mut ry) = (0usize, 0usize, 0i64, 0i64);
                    for ex in exs.iter() {
                        rn += 1 + ex.holes.len();
                        for pt in &ex.contour.points { rp += 1; rx += pt.x(); ry += pt.y(); }
                        for h in &ex.holes { for pt in &h.points { rp += 1; rx += pt.x(); ry += pt.y(); } }
                    }
                    if rn > 0 {
                        eprintln!("SEG li={} e={} n={} pts={} sx={} sy={}", li, e, rn, rp, rx, ry);
                    }
                }
            }
        }
        // Move painted areas out of region 0 into the painted regions.
        // PrintObjectSlice.cpp:855-925 (single-parent collapse).
        let painted_order: Vec<u8> = self.painted_submeshes.iter().map(|(e, _)| *e).collect();
        // R489: filament -> print-region index. C++ keeps the PrintRegion pointer
        // that `get_create_region` handed back (PrintApply.cpp:1071); because that
        // call deduplicates, the index cannot be derived from the painted-extruder
        // ordinal. The region a painted extruder owns is the one whose config prints
        // with that filament — first match, so a deduped extruder resolves to the
        // parent region it was merged into.
        let extruder_to_region: std::collections::HashMap<u8, usize> = {
            let mut m = std::collections::HashMap::new();
            if let Some(sr) = self.shared_regions.as_ref() {
                for (idx, r) in sr.all_regions.iter().enumerate() {
                    let c = r.config();
                    if c.wall_filament == c.solid_infill_filament
                        && c.wall_filament == c.sparse_infill_filament
                    {
                        m.entry(c.wall_filament as u8).or_insert(idx);
                    }
                }
            }
            m
        };
        if crate::probe_enabled("MMS_DEBUG") {
            let seg_nonempty = segmented
                .iter()
                .filter(|l| l.iter().any(|s| !s.is_empty()))
                .count();
            let slices_nonempty = layer_slices.iter().filter(|s| !s.is_empty()).count();
            // R455: how much painted triangle AREA faces up/down vs sideways. Contour
            // projection (all our port does) only sees side-facing paint; C++'s
            // `mmu_segmentation_top_and_bottom_layers` is what turns up/down-facing paint
            // into real area. This quantifies how much that stub is costing BEFORE
            // committing to the ~1000-line slice_mesh_slabs port it needs.
            {
                let mut side = vec![0.0f64; 16];
                let mut updown = vec![0.0f64; 16];
                for (ext, its) in self.painted_submeshes.iter() {
                    let e = (*ext as usize).min(15);
                    for tri in its.indices.iter() {
                        let (a, b, c) = (
                            its.vertices[tri[0] as usize],
                            its.vertices[tri[1] as usize],
                            its.vertices[tri[2] as usize],
                        );
                        let u = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
                        let v = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
                        let n = [
                            (u[1] * v[2] - u[2] * v[1]) as f64,
                            (u[2] * v[0] - u[0] * v[2]) as f64,
                            (u[0] * v[1] - u[1] * v[0]) as f64,
                        ];
                        let mag = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
                        if mag <= 0.0 {
                            continue;
                        }
                        let area = 0.5 * mag;
                        // |nz|/|n| > cos(45deg): the facet is closer to horizontal than vertical.
                        if (n[2].abs() / mag) > std::f64::consts::FRAC_1_SQRT_2 {
                            updown[e] += area;
                        } else {
                            side[e] += area;
                        }
                    }
                }
                let ts: f64 = side.iter().sum();
                let tu: f64 = updown.iter().sum();
                eprintln!(
                    "MMS_FACING: painted triangle area mm2 — side-facing {:.0} ({:.1}%), up/down-facing {:.0} ({:.1}%)",
                    ts, 100.0 * ts / (ts + tu).max(1e-9),
                    tu, 100.0 * tu / (ts + tu).max(1e-9)
                );
                for e in 1..16 {
                    if side[e] + updown[e] > 0.0 {
                        eprintln!(
                            "MMS_FACING:   extruder {} side={:.0} up/down={:.0} ({:.1}% up/down)",
                            e, side[e], updown[e],
                            100.0 * updown[e] / (side[e] + updown[e])
                        );
                    }
                }
            }

            // R453: the actual PARTITION, which is what the per-tool sparse-infill
            // parity metric reduces to. Total area claimed by each extruder slot over
            // all layers, against the total input slice area — the leftover is what
            // stays with the object's default filament.
            let sf2 = crate::SCALING_FACTOR * crate::SCALING_FACTOR;
            let mut per_ext = vec![0.0f64; num_extruders + 1];
            for l in segmented.iter() {
                for (e, eps) in l.iter().enumerate() {
                    if e < per_ext.len() {
                        per_ext[e] += eps.iter().map(|x| x.area()).sum::<f64>() / sf2;
                    }
                }
            }
            let total_slice: f64 = layer_slices
                .iter()
                .map(|s| s.iter().map(|x| x.area()).sum::<f64>() / sf2)
                .sum();
            let claimed: f64 = per_ext.iter().sum();
            // Per-layer claimed fraction: distinguishes "a few layers fail outright"
            // from "every layer systematically under-claims the interior".
            let mut hist = [0usize; 11];
            let mut worst: Vec<(usize, f64)> = Vec::new();
            for (li, l) in segmented.iter().enumerate() {
                let sl: f64 = layer_slices
                    .get(li)
                    .map(|s| s.iter().map(|x| x.area()).sum::<f64>() / sf2)
                    .unwrap_or(0.0);
                if sl <= 0.0 {
                    continue;
                }
                let cl: f64 = l.iter().map(|e| e.iter().map(|x| x.area()).sum::<f64>() / sf2).sum();
                let frac = (cl / sl).clamp(0.0, 1.0);
                hist[(frac * 10.0).round() as usize] += 1;
                worst.push((li, frac));
            }
            worst.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap());
            {
                use std::sync::atomic::Ordering::Relaxed;
                use crate::multi_material_segmentation::{FACES_DISCARDED, FACES_OK, FACES_RECOVERED};
                let (ok, rec, disc) = (
                    FACES_OK.load(Relaxed),
                    FACES_RECOVERED.load(Relaxed),
                    FACES_DISCARDED.load(Relaxed),
                );
                let tot = (ok + rec + disc).max(1);
                use crate::multi_material_segmentation::{
                    CONTOUR_LEN_TOTAL, PAINTED_LEN_POST, PAINTED_LEN_RAW, COLORIZED_LEN, COLOR_LEN_BUCKETS,
                };
                let (praw, ppost, ctot) = (
                    PAINTED_LEN_RAW.load(Relaxed) as f64 / 1000.0,
                    PAINTED_LEN_POST.load(Relaxed) as f64 / 1000.0,
                    CONTOUR_LEN_TOTAL.load(Relaxed) as f64 / 1000.0,
                );
                eprintln!(
                    "MMS_COLORIZE: painted length raw={:.0} ({:.1}% of contour) -> post_process={:.0} ({:.1}%)  [contour total {:.0}]",
                    praw, 100.0 * praw / ctot.max(1e-9),
                    ppost, 100.0 * ppost / ctot.max(1e-9), ctot,
                );
                let lens: Vec<f64> = (0..COLOR_LEN_BUCKETS)
                    .map(|i| COLORIZED_LEN[i].load(Relaxed) as f64 / 1000.0)
                    .collect();
                let ltot: f64 = lens.iter().sum::<f64>().max(1e-9);
                eprintln!(
                    "MMS_COLORIZE: contour length by colour (mm) — total {:.0}, colour0(unpainted) {:.0} ({:.1}%)",
                    ltot, lens[0], 100.0 * lens[0] / ltot
                );
                for (c, l) in lens.iter().enumerate().skip(1) {
                    if *l > 0.0 {
                        eprintln!("MMS_COLORIZE:   colour {} len={:.0} ({:.1}%)", c, l, 100.0 * l / ltot);
                    }
                }
                eprintln!(
                    "MMS_PARTITION: traced faces ok={} ({:.1}%) recovered={} ({:.1}%) DISCARDED={} ({:.1}%)",
                    ok, 100.0 * ok as f64 / tot as f64,
                    rec, 100.0 * rec as f64 / tot as f64,
                    disc, 100.0 * disc as f64 / tot as f64,
                );
            }
            eprintln!("MMS_PARTITION: per-layer claimed-fraction histogram (0.0..1.0 in tenths) = {:?}", hist);
            eprintln!(
                "MMS_PARTITION: worst layers = {:?}",
                worst.iter().take(6).map(|(i, f)| (*i, (f * 100.0).round() as i32)).collect::<Vec<_>>()
            );
            eprintln!(
                "MMS_PARTITION: total_slice_area={:.0} claimed_by_painting={:.0} ({:.1}%) leftover_to_default={:.0}",
                total_slice,
                claimed,
                100.0 * claimed / total_slice.max(1e-9),
                total_slice - claimed
            );
            for (e, a) in per_ext.iter().enumerate() {
                if *a > 0.0 {
                    eprintln!(
                        "MMS_PARTITION:   slot {} area={:.0} ({:.1}% of slice)",
                        e,
                        a,
                        100.0 * a / total_slice.max(1e-9)
                    );
                }
            }
            eprintln!(
                "MMS_DEBUG: center_offset={:?} mms_center_offset=({},{}) layers={} segmented={} seg_layers_nonempty={} slice_layers_nonempty={} painted_order={:?}",
                self.slice_center_offset,
                mms_center_offset.x, mms_center_offset.y,
                self.layers.len(), segmented.len(), seg_nonempty, slices_nonempty, painted_order,
            );
            // Frame check on a mid layer: bbox of the painted segmentation vs the
            // region-0 slice it must intersect.
            let li = self.layers.len() / 2;
            if li < segmented.len() && li < layer_slices.len() {
                let bb = |v: &crate::geometry::ExPolygons| -> String {
                    let (mut x0, mut x1, mut y0, mut y1) = (i64::MAX, i64::MIN, i64::MAX, i64::MIN);
                    for ex in v {
                        for p in &ex.contour.points {
                            x0 = x0.min(p.x); x1 = x1.max(p.x);
                            y0 = y0.min(p.y); y1 = y1.max(p.y);
                        }
                    }
                    if x0 == i64::MAX { "<empty>".into() } else { format!("X[{x0},{x1}] Y[{y0},{y1}]") }
                };
                for (slot, its) in self.painted_submeshes.iter() {
                    let (mut x0, mut x1, mut y0, mut y1, mut z0, mut z1) =
                        (f32::MAX, f32::MIN, f32::MAX, f32::MIN, f32::MAX, f32::MIN);
                    for v in &its.vertices {
                        x0 = x0.min(v.x); x1 = x1.max(v.x);
                        y0 = y0.min(v.y); y1 = y1.max(v.y);
                        z0 = z0.min(v.z); z1 = z1.max(v.z);
                    }
                    eprintln!(
                        "MMS_DEBUG submesh extruder{slot}: tris={} X[{x0:.2},{x1:.2}] Y[{y0:.2},{y1:.2}] Z[{z0:.2},{z1:.2}]",
                        its.indices.len()
                    );
                }
                eprintln!("MMS_DEBUG layer{li}: region0 {}", bb(&layer_slices[li]));
                for (slot, seg) in segmented[li].iter().enumerate() {
                    eprintln!("MMS_DEBUG layer{li}: seg slot{slot} {}", bb(seg));
                }
            }
        }
        for (layer_idx, layer) in self.layers.iter_mut().enumerate() {
            if layer_idx >= segmented.len() {
                break;
            }
            let region0_ex = &layer_slices[layer_idx];
            if region0_ex.is_empty() {
                continue;
            }
            // R700 — holes in the SEGMENTATION OUTPUT itself, before any region
            // assembly. R699 proved `difference(solid, island)` creates a hole
            // whenever the island is interior, so if these painted ExPolygons are
            // hole-free AND meet the region boundary, the 39.6x hole deficit at
            // bracket C follows with no primitive at fault.
            if crate::probe_enabled("SEGHOLES") {
                use std::sync::atomic::{AtomicU64, Ordering::Relaxed};
                static PIECES: AtomicU64 = AtomicU64::new(0);
                static HOLES: AtomicU64 = AtomicU64::new(0);
                static LAYERS: AtomicU64 = AtomicU64::new(0);
                let mut p = 0u64;
                let mut h = 0u64;
                for slot in segmented[layer_idx].iter() {
                    for ex in slot.iter() {
                        p += 1;
                        h += ex.holes.len() as u64;
                    }
                }
                PIECES.fetch_add(p, Relaxed);
                HOLES.fetch_add(h, Relaxed);
                let n = LAYERS.fetch_add(1, Relaxed) + 1;
                if n % 200 == 0 {
                    eprintln!(
                        "[SEGHOLES] layers={n} painted_pieces={} holes={}",
                        PIECES.load(Relaxed),
                        HOLES.load(Relaxed),
                    );
                }
            }
            let mut stolen_total: crate::geometry::ExPolygons = Vec::new();
            for (i, &extruder) in painted_order.iter().enumerate() {
                let _ = i;
                // Faithful merge output shape: [layer][num_extruders], 0-based
                // extruder slots (slot j == filament j+1); the default color is
                // dropped by merge_segmented_layers — matches the C++ consumer
                // indexing (PrintObjectSlice.cpp:848,877-879).
                let slot = extruder as usize - 1;
                let seg = match segmented[layer_idx].get(slot) {
                    Some(s) if !s.is_empty() => s,
                    _ => continue,
                };
                // Painted area limited to what region 0 actually owns.
                // MMSEG_STOLEN_SUBJECT (on): PrintObjectSlice.cpp:1921
                // `intersection_ex(layerm.slices.surfaces, segmented.expolygons)` —
                // the region's slices are the subject and the segmentation the clip.
                // With the operands the other way round Clipper rounds the crossing
                // points differently (=0).
                let stolen = if crate::faithful_gate("MMSEG_STOLEN_SUBJECT") {
                    crate::clipper_utils::intersection(region0_ex, seg)
                } else {
                    crate::clipper_utils::intersection(seg, region0_ex)
                };
                if stolen.is_empty() {
                    continue;
                }
                // R489: `1 + i` hard-codes "painted extruder at index i lives in
                // print region 1+i", which only holds while
                // Print::install_painted_regions appends one region per painted
                // extruder unconditionally. C++ resolves the region through the
                // pointer `get_create_region` returned (PrintApply.cpp:1071), which
                // may be an EXISTING region — so the index is not derivable from the
                // painted-extruder ordinal. Look it up by filament instead, which is
                // what actually identifies the region, and is correct whether or not
                // the dedup is active. Tied to the same gate so the default path is
                // byte-unchanged.
                let region_id = if crate::faithful_gate("PAINTED_REGION_DEDUP") {
                    match extruder_to_region.get(&extruder) {
                        // The painted extruder resolved to the PARENT region, i.e.
                        // `get_create_region` reused it because the parent already
                        // prints this filament. Then there is nothing to steal: the
                        // area is already in the region that will print it. Stealing
                        // it would also be destructive here, because the remainder
                        // write below re-sets region 0 and would clobber it (measured:
                        // object-only 0.9798 -> 0.8200 when this case steals).
                        Some(&0) => continue,
                        Some(&r) => r,
                        None => continue,
                    }
                } else {
                    1 + i
                };
                if region_id >= layer.regions().len() {
                    continue;
                }
                // R557 / PrintObjectSlice.cpp:962-964 — C++ accumulates into
                // `by_region[region_id]` and, if a region received MORE THAN ONE
                // contribution (`needs_merge`), applies
                //   src.expolygons = closing_ex(src.expolygons, float(scale_(10 * EPSILON)));
                // before `slices.set(...)`. Accumulate the same way so a region fed by
                // two painted extruders (possible under PAINTED_REGION_DEDUP) is merged
                // rather than overwritten.
                // R678 — SHIPPED ON. Inert on Majora (byte-identical output, bracket C
                // unchanged at 26,620 surfaces): `prev` is non-empty only when an
                // earlier painted extruder in the SAME layer already wrote to the SAME
                // region_id, which PAINTED_REGION_DEDUP's one-region-per-filament map
                // never produces here. Kept ON because it is the faithful C++ shape and
                // costs nothing until a config does hit the multi-contribution case.
                let merged = if crate::faithful_gate("MMSEG_CLOSING") {
                    let prev = layer.regions()[region_id]
                        .slices
                        .surfaces
                        .iter()
                        .map(|s| s.expolygon.clone())
                        .collect::<crate::geometry::ExPolygons>();
                    if prev.is_empty() {
                        stolen.clone()
                    } else {
                        let mut acc = prev;
                        acc.extend(stolen.iter().cloned());
                        // needs_merge == true
                        crate::clipper_utils::closing(
                            &acc,
                            10.0 * crate::libslic3r::EPSILON,
                            crate::clipper_utils::OffsetJoinType::Miter,
                        )
                    }
                } else {
                    stolen.clone()
                };
                let mut coll = SurfaceCollection::new();
                for ex in &merged {
                    coll.push(Surface::new(SurfaceType::Internal, ex.clone()));
                }
                layer.regions_mut()[region_id].set_slices(coll);
                stolen_total.extend(stolen);
            }
            if !stolen_total.is_empty() {
                // Remainder stays in region 0.
                // R806 MMSEG_REMAINDER_SEQ (faithful ON) — PrintObjectSlice.cpp:948-976:
                // native builds the unpainted remainder on flat Polygons, subtracting
                // each extruder's SEGMENTED pieces one ctDifference at a time (extruder
                // order, own extruder skipped, bbox-gated), then
                // opening(union_ex(mine), 5*EPSILON) via the Paths offset variants,
                // then union_ex. The one-shot ExPolygon difference rounds its
                // intersection vertices differently (457 Majora layers diverged in
                // region 0 alone).
                let seq_remainder: Option<crate::geometry::ExPolygons> = if crate::faithful_gate("MMSEG_REMAINDER_SEQ") {
                    use crate::clipper_utils::OffsetJoinType;
                    let pts0: Vec<crate::Point> = region0_ex.iter().flat_map(|ex| ex.contour.points.iter().copied()).collect();
                    let bb0 = crate::geometry::BoundingBox::from_points(&pts0);
                    let mut mine: Vec<crate::geometry::Polygon> = crate::geometry::to_polygons(region0_ex);
                    for slot in 0..segmented[layer_idx].len() {
                        let extruder = (slot + 1) as u8;
                        let seg = &segmented[layer_idx][slot];
                        if seg.is_empty() {
                            continue;
                        }
                        let is_self = crate::faithful_gate("PAINTED_REGION_DEDUP")
                            && extruder_to_region.get(&extruder) == Some(&0);
                        if is_self {
                            continue;
                        }
                        let ptss: Vec<crate::Point> = seg.iter().flat_map(|ex| ex.contour.points.iter().copied()).collect();
                        let bbs = crate::geometry::BoundingBox::from_points(&ptss);
                        if !(bb0.min.x <= bbs.max.x && bb0.max.x >= bbs.min.x && bb0.min.y <= bbs.max.y && bb0.max.y >= bbs.min.y) {
                            continue;
                        }
                        mine = crate::clipper_utils::difference_polygons_clib(&mine, seg);
                        if mine.is_empty() {
                            break;
                        }
                    }
                    if mine.is_empty() {
                        Some(Vec::new())
                    } else {
                        let u = crate::clipper_utils::union_ex_clib(&mine, 1);
                        let rings = crate::geometry::to_polygons(&u);
                        let d = (crate::scaled_f(5.0 * crate::libslic3r::EPSILON) as f32) as f64;
                        let shr = crate::clipper_utils::offset_polygons_clib(&rings, -d, OffsetJoinType::Miter);
                        let exp = crate::clipper_utils::offset_polygons_clib(&shr, d, OffsetJoinType::Miter);
                        Some(crate::clipper_utils::union_ex_clib(&exp, 1))
                    }
                } else {
                    None
                };
                // R698 — HOLE LOSS. C++ builds this remainder with
                // `diff(mine, segmented.expolygons)` + `union_ex(mine)`
                // (PrintObjectSlice.cpp:937/951), and subtracting painted islands
                // from the base region is exactly what CREATES holes: C++ carries
                // 1,740 holes at bracket C where we carry 44 (39.6x), with our area
                // 0.14% LARGER — both symptoms of holes being filled in rather than
                // kept.
                //
                // The asymmetry is in the primitives: `intersection` routes to the
                // vertex-exact `intersection_clib` under the default-ON CLIPPER_INT
                // gate, but `difference` has NO such routing — it always takes the
                // geo-clipper path at a 1um grid, even though `difference_clib`
                // exists and is faithful. Gated here (not inside `difference`) to
                // keep the blast radius to this one call site.
                // R739 STATUS: this gate is INERT. Re-scored on all four fixtures
                // against today's default and the output hash was IDENTICAL to the
                // ungated build on every one, majora included — it changes nothing.
                // Its premise (R698) predates the `_clib` conversions that have
                // since shipped (CLIPPER_DIFF_GEO R728, CLIPPER_UNION_GEO R738),
                // which already route this difference through ClipperLib. Do not
                // spend another parity run on it; delete it with the next cleanup
                // of the geo/clib gate family.
                let mut remaining = if let Some(r) = seq_remainder {
                    r
                } else if crate::opt_in_gate("MMSEG_DIFF_CLIB") {
                    crate::clipper_utils::difference_clib(region0_ex, &stolen_total)
                } else {
                    crate::clipper_utils::difference(region0_ex, &stolen_total)
                };
                // R678 — SHIPPED ON, reversing R557's parking decision. R557 rejected
                // this on a WALL-LINES IoU metric that is no longer the bar; R677 then
                // showed by bracketing BOTH engines around this exact stage that it is
                // the sole source of the long-hunted contour-coarseness deficit.
                // Bracket C (post-segmentation LayerRegion slices, Majora, 656 layers):
                //     OFF   26,620 surfaces   50.884 points/contour
                //     ON    14,924 surfaces   88.736 points/contour
                //     C++   16,728 surfaces   86.384 points/contour
                // i.e. the 1.698x deficit becomes 1.027x. Scored on BOTH current
                // metrics: Majora in-order +919 lines (18.65% -> 18.70%), concentrated
                // in the two features it targets (Outer wall +802, Inner wall +407);
                // Majora order-blind content -2,457, in those same walls. Structure
                // right, coordinates shifted — and the order-blind metric is the one
                // R651/R652 established is insufficient. Benchy and cube are
                // byte-unchanged (single-material: this function never runs).
                // Both prior parkings (R654, R656) were driven by the ORDER metric
                // getting worse; here it improves. Disable with MMSEG_OPENING=0.
                // R557: SHIPS OPT-IN (not default-ON). The port is faithful and moves
                // our internal geometry decisively toward C++ (surfaces per
                // layer-region 7.78 -> 4.40 against C++'s 4.97; make_perimeters call
                // count 3,400 -> 3,200, exactly C++'s), but it does NOT move the
                // `; LINE_WIDTH:` metric it was aimed at (1.19, unchanged) and costs
                // 0.06pp of wall-lines IoU — a real regression, stable across three
                // independent C++ references.
                // R557 / PrintObjectSlice.cpp:946-947 — C++ filters the remainder:
                //   if (! mine.empty())
                //       mine = opening(union_ex(mine), float(scale_(5 * EPSILON)),
                //                                      float(scale_(5 * EPSILON)));
                // with the comment: "subtraction from layerm.region() could produce a
                // huge number of small unprintable regions for the model's base
                // extruder". `difference` already returns unioned ExPolygons, so only
                // the opening is needed. NOTE the distance unit: `opening_ex` takes mm
                // (the scale is applied inside `offset_expolygons`), so C++'s
                // `scale_(5 * EPSILON)` is simply `5 * EPSILON` here.
                if !crate::faithful_gate("MMSEG_REMAINDER_SEQ") && crate::faithful_gate("MMSEG_OPENING") && !remaining.is_empty() {
                    // R697 — DIAGNOSTIC sweep knob. The nominal value is C++'s
                    // `scale_(5 * EPSILON)`; EPSILON is 1e-4 on both engines and
                    // `opening_ex` takes mm, so 5.0 * EPSILON is the faithful
                    // translation and is the DEFAULT. The multiplier exists only to
                    // locate where our surface count crosses C++'s 16,732: opening
                    // OFF gives 26,620 and 5x gives 14,924, so the target is
                    // bracketed and monotonic. Not a shipping knob.
                    let mult = crate::env_f64("MMSEG_OPEN_MULT").unwrap_or(5.0);
                    remaining = crate::clipper_utils::opening_ex(
                        &remaining,
                        mult * crate::libslic3r::EPSILON,
                    );
                }
                let mut coll = SurfaceCollection::new();
                for ex in &remaining {
                    coll.push(Surface::new(SurfaceType::Internal, ex.clone()));
                }
                layer.regions_mut()[0].set_slices(coll);
            }
        }
        Ok(())
    }

    pub fn wire_layer_hierarchy(&mut self) {
        let object_config = Arc::new(self.config.clone());
        let print_config = self.print_config.clone();
        let all_regions: Vec<Arc<PrintRegion>> = self
            .shared_regions
            .as_ref()
            .map(|regions| regions.all_regions.clone())
            .unwrap_or_default();
        // Print.hpp:442 — `bounding_box()`, which Layer::make_fills hands its
        // fillers (Fill.cpp:599, 699).
        let object_bounding_box = self
            .mesh
            .as_ref()
            .and_then(crate::layer::print_object_bounding_box);
        for layer in &mut self.layers {
            layer.wire_config_hierarchy(&object_config, &print_config, &all_regions);
            layer.object_bounding_box = object_bounding_box;
        }
        for layer in &mut self.support_layers {
            layer.wire_config_hierarchy(&object_config, &print_config, &all_regions);
        }
    }

    /// Generate perimeters for all layers
    /// PrintObject.cpp:453-620
    ///
    /// ⚠️ WARNING: INCOMPLETE PORT - Missing critical algorithm!
    ///
    /// This implementation is missing:
    /// - Extra perimeters calculation (C++ lines 481-540)
    /// - Region loop and per-region config handling
    /// - Typed slices restoration (C++ lines 464-472)
    /// - Perimeter continuity calculation (C++ lines 583-615)
    ///
    /// C++ Structure:
    /// 1. Call this->slice() (line 456)
    /// 2. Restore untyped slices if needed (lines 464-472)
    /// 3. Loop over regions, calculate extra_perimeters (lines 481-540)
    /// 4. Parallel loop: call Layer::make_perimeters() (lines 557-566)
    /// 5. Calculate perimeter continuity (lines 583-615)
    pub fn make_perimeters(&mut self) -> Result<()> {
        // Prerequisites: slice must run first
        // PrintObject.cpp:456
        let __mp_t = crate::probe_enabled("SLICE_PHASE_TIMING");
        let __t_slice = std::time::Instant::now();
        self.slice()?;
        let __slice_s = __t_slice.elapsed().as_secs_f64();
        let __t_peri = std::time::Instant::now();

        crate::stage_dump::pp_slices(&self.layers);
        if let Some(sd_key) = crate::stage_dump::stagedump_key() {
            if sd_key < self.layers.len() {
                let ly = &self.layers[sd_key];
                crate::stage_dump::dump("lslices", sd_key, &ly.lslices);
                if let Some(lrm) = ly.regions().first() {
                    let eps: Vec<crate::geometry::ExPolygon> =
                        lrm.slices.surfaces.iter().map(|s| s.expolygon.clone()).collect();
                    crate::stage_dump::dump("rslices", sd_key, &eps);
                }
            }
        }

        // Check if already done
        // PrintObject.cpp:457-458
        if self.is_step_done(PrintObjectStep::Perimeters) {
            return Ok(());
        }

        // Check for cancellation
        // PrintObject.cpp:456
        if self.canceled.load(Ordering::Relaxed) {
            return Err(Error::Cancelled);
        }

        // PrintObject.cpp:461
        // m_print->set_status(15, L("Generating walls"));

        // Restore untyped slices if needed
        // PrintObject.cpp:464-472
        if self.typed_slices {
            for layer in &mut self.layers {
                // PrintObject.cpp:467
                // C++: layer->restore_untyped_slices();
                layer.restore_untyped_slices();
                // PrintObject.cpp:468
                // C++: m_print->throw_if_canceled();
                if self.canceled.load(Ordering::Relaxed) {
                    return Err(Error::Cancelled);
                }
            }
            self.typed_slices = false;
        }

        let num_regions = self.num_printing_regions();

        // Extra perimeters calculation
        // PrintObject.cpp:481-540
        // Loop over each region to calculate extra perimeters
        for region_id in 0..num_regions {
            let _region = match self.printing_region(region_id) {
                Some(r) => r,
                None => continue,
            };

            // BBS: extra_perimeters removed, always skip
            // PrintObject.cpp:482-483
            // if (!region.config().extra_perimeters || region.config().wall_loops == 0 ||
            //     region.config().sparse_infill_density == 0 || this->layer_count() < 2)
            //     continue;
            continue; // Always skip for now since extra_perimeters is disabled in BambuStudio

            // TODO: Port the extra_perimeters algorithm when needed
            // This is the code that was at C++ lines 485-540
            // It analyzes layer-to-layer differences and marks surfaces needing extra perimeters
        }

        // Generate perimeters for each layer
        // PrintObject.cpp:557-566
        // C++: tbb::parallel_for(0, m_layers.size(), [this](...) { m_layers[layer_idx]->make_perimeters(); })
        //
        // Per-region configs are no longer threaded from here: each
        // LayerRegion reads its own stored Arc<PrintRegion> (C++:
        // (*layerm)->region().config(), Layer.cpp:218 / LayerRegion.cpp:137),
        // and the Classic/Arachne dispatch reads the object config Arc
        // (C++: this->layer()->object()->config().wall_generator,
        // LayerRegion.cpp:176).

        // Collect lslices from each layer for overhang detection (previous layer comparison)
        let all_lslices: Vec<Vec<crate::geometry::ExPolygon>> =
            self.layers.iter().map(|l| l.lslices.clone()).collect();

        // C++: tbb::parallel_for(tbb::blocked_range<size_t>(0, m_layers.size()),
        // C++:     [this](const tbb::blocked_range<size_t> &range) {
        // C++:         for (size_t layer_idx = range.begin(); layer_idx < range.end(); ++layer_idx) {
        // C++:             m_print->throw_if_canceled();
        // C++:             m_layers[layer_idx]->make_perimeters();
        // rayon stands in for tbb: `par_iter_mut().enumerate()` is the
        // blocked_range over layer indices, `try_for_each` propagates the
        // cancellation Err exactly like throw_if_canceled unwinds tbb.
        use rayon::prelude::*;
        let canceled = self.canceled.clone();
        let object_config = &self.config;
        let l0_efc_done_in_slice = self.l0_efc_done_in_slice;
        self.layers
            .par_iter_mut()
            .enumerate()
            .try_for_each(|(idx, layer)| -> Result<()> {
            // Check for cancellation
            // PrintObject.cpp:559
            if canceled.load(Ordering::Relaxed) {
                return Err(Error::Cancelled);
            }

            // FIDELITY-NOTE: elephant-foot compensation has no counterpart in C++
            // PrintObject::make_perimeters — in BambuStudio it is applied during
            // slicing (apply_first_layer_compensation in PrintObjectSlice.cpp). It is
            // kept here as this crate's integration point; retained to avoid silently
            // dropping the feature, not because the C++ runs it from make_perimeters.
            // R764: default ON. The faithful route (R316) was parked behind a
            // presence-only env; the legacy step left layer-0 slices at 1277
            // verts vs native 1452 — the R759/R763 layer-0 structural root.
            // With this ON the layer-0 surface matches native's structure
            // exactly (1452 pts / 8 holes; the 76-pt island byte-exact).
            // L0_EFC=0 restores the legacy arm.
            if idx == 0 && l0_efc_done_in_slice {
                // The multi-region branch already ran in slice().
            } else if idx == 0
                && crate::faithful_gate("L0_EFC")
                && object_config.elephant_foot_compensation > 0.0
            {
                // R316: native L0 (PrintObjectSlice.cpp:1228-46): slices.set(
                // union_ex(Slic3r::elephant_foot_compensation(raw, ext_perimeter
                // flow, elfoot))) while lslices keep the UNCOMPENSATED backup —
                // the SDN-attributed bisect (R315) shows native rsl = 1528-pt
                // COMPENSATED contours (EFC adds vertices) vs lslices 1368 raw;
                // rust never compensated its surfaces. Route through the
                // faithful elephant_foot_compensation port.
                let layer_height = layer.height;
                for region in layer.regions_mut().iter_mut() {
                    let raw: crate::geometry::ExPolygons =
                        region.slices.surfaces.iter().map(|s| s.expolygon.clone()).collect();
                    if let Ok(ext_flow) =
                        region.flow(crate::flow::FlowRole::ExternalPerimeter, layer_height)
                    {
                        let comp: crate::geometry::ExPolygons = raw
                            .iter()
                            .map(|ex| {
                                crate::elephant_foot_compensation::elephant_foot_compensation_with_flow(
                                    ex,
                                    &ext_flow,
                                    object_config.elephant_foot_compensation,
                                )
                            })
                            .collect();
                        let merged = crate::clipper_utils::union_ex(&comp);
                        region.slices.surfaces = merged
                            .into_iter()
                            .map(|ex| crate::surface::Surface {
                                expolygon: ex,
                                surface_type: crate::surface::SurfaceType::Internal,
                                thickness: -1.0,
                                thickness_layers: 1,
                                bridge_angle: None,
                                extra_perimeters: 0,
                            })
                            .collect();
                    }
                }
            } else if idx == 0 && object_config.elephant_foot_compensation > 0.0 {
                for region in layer.regions_mut().iter_mut() {
                    region.elephant_foot_compensation_step(self.config.elephant_foot_compensation);
                }
            }

            // (LayerRegion::flow reads print_config.initial_layer_line_width
            // off its stored Arc on the first layer — PrintRegion.cpp:27-28 —
            // so no per-region width stamping is needed here.)

            // Pass previous layer's lslices for overhang detection
            let lower_slices = if idx > 0 {
                Some(&all_lslices[idx - 1])
            } else {
                None
            };
            // Pass next layer's lslices for top-surface detection (top_fills).
            // PerimeterGenerator.cpp:1118 uses this->upper_slices.
            let upper_slices = if idx + 1 < all_lslices.len() {
                Some(&all_lslices[idx + 1])
            } else {
                None
            };

            // Call Layer::make_perimeters() which orchestrates perimeter generation
            // This will call LayerRegion::make_perimeters() for each region
            // PrintObject.cpp:560
            layer.make_perimeters_with_neighbors(lower_slices, upper_slices)
            })?;

        // PrintObject.cpp:583-615 — perimeter continuity + cooling-node ids
        // (z-direction outwall speed smoothing). Native runs the continuity
        // and record passes as parallel_for with a `layer_idx > 1` guard;
        // ported sequentially (layer order is irrelevant for continuity —
        // each pass only touches layer_idx and layer_idx-1).
        if std::env::var("NODEDBG").is_ok() {
            eprintln!(
                "NODEDBG flag={}",
                self.print_config.z_direction_outwall_speed_continuous
            );
        }
        if self.print_config.z_direction_outwall_speed_continuous {
            // PrintObject.cpp:583-588 — calculate_perimeter_continuity
            for layer_idx in 2..self.layers.len() {
                let (before, after) = self.layers.split_at_mut(layer_idx);
                let prev = &mut before[layer_idx - 1];
                after[0].calculate_perimeter_continuity(&mut prev.loop_nodes);
            }

            // PrintObject.cpp:595-600 — merge_layer_node
            let mut max_merged_id: i32 = -1;
            let mut node_record: std::collections::BTreeMap<i32, Vec<(i32, i32)>> =
                std::collections::BTreeMap::new();
            for layer_idx in 1..self.layers.len() {
                self.merge_layer_node(layer_idx, &mut max_merged_id, &mut node_record);
            }

            // PrintObject.cpp:606-615 — record cooling node for each extrusion
            // (native guard: layer_idx > 1)
            for layer_idx in 2..self.layers.len() {
                self.layers[layer_idx].record_cooling_node_for_each_extrusion();
            }

            if std::env::var("NODEDBG").is_ok() {
                let total_nodes: usize = self.layers.iter().map(|l| l.loop_nodes.len()).sum();
                let linked: usize = self
                    .layers
                    .iter()
                    .flat_map(|l| l.loop_nodes.iter())
                    .filter(|n| !n.lower_node_ids.is_empty())
                    .count();
                let (mut ents, mut ents_with_node, mut regions_ranged) = (0usize, 0usize, 0usize);
                for l in self.layers.iter() {
                    for r in l.regions() {
                        ents += r.perimeters.entities.len();
                        ents_with_node += r.entity_cooling_nodes.iter().filter(|&&v| v != -1).count();
                        if r.perimeters.loop_node_range.1 > r.perimeters.loop_node_range.0 { regions_ranged += 1; }
                    }
                }
                eprintln!("NODEDBG entities={} with_node={} regions_ranged={}", ents, ents_with_node, regions_ranged);
                eprintln!(
                    "NODEDBG layers={} nodes={} linked={} max_merged_id={}",
                    self.layers.len(),
                    total_nodes,
                    linked,
                    max_merged_id
                );
            }
        }

        if __mp_t {
            eprintln!(
                "      make_perimeters split: slice() {:.2}s  perimeter_gen {:.2}s",
                __slice_s,
                __t_peri.elapsed().as_secs_f64()
            );
        }

        // Mark step as complete
        // PrintObject.cpp:620
        self.set_step_done(PrintObjectStep::Perimeters);
        Ok(())
    }

    /// Prepare infill - detect surface types and prepare fill surfaces
    /// PrintObject.cpp:623-750
    /// C++: detect_surfaces_type(), prepare_fill_surfaces(), discover_vertical_shells(), etc.
    pub fn prepare_infill(&mut self) -> Result<()> {
        /// PrintObject.cpp:624
        if self.is_step_done(PrintObjectStep::PrepareInfill) {
            return Ok(());
        }


        /// PrintObject.cpp:626-633
        /// C++: m_print->set_status(25, L("Generating infill regions"));
        // TODO: Port status callback

        /// PrintObject.cpp:626-640
        if self.typed_slices {
            /// PrintObject.cpp:633
            for layer in &mut self.layers {
                /// PrintObject.cpp:634
                /// C++: layer->restore_untyped_slices_no_extra_perimeters();
                layer.restore_untyped_slices_no_extra_perimeters();

                /// PrintObject.cpp:635
                if self.canceled.load(std::sync::atomic::Ordering::Relaxed) {
                    return Err(crate::Error::Cancelled);
                }
            }
        }

        // SLICE_PHASE_TIMING (R392): time prepare_infill's sub-steps to locate the
        // Majora bottleneck (the ~35s is here, not in MMS segmentation).
        let __pi_t = crate::probe_enabled("SLICE_PHASE_TIMING");
        let mut __pi: Vec<(&'static str, f64)> = Vec::new();
        macro_rules! pi_phase {
            ($name:expr, $body:expr) => {{
                let __t = std::time::Instant::now();
                let __r = $body;
                if __pi_t {
                    __pi.push(($name, __t.elapsed().as_secs_f64()));
                }
                __r
            }};
        }

        /// PrintObject.cpp:642-647
        /// C++: std::vector<std::vector<SurfaceCollection>> slice_surfaces_cpy;
        /// C++: this->detect_surfaces_type(slice_surfaces_cpy);
        /// C++: m_print->throw_if_canceled();
        let mut slice_surfaces_cpy: Vec<Vec<SurfaceCollection>> = Vec::new();
        pi_phase!("detect_surfaces_type", self.detect_surfaces_type(&mut slice_surfaces_cpy)?);
        crate::stage_dump::pp_typed("a_detect", &self.layers);

        /// PrintObject.cpp:649-655
        /// C++: for (auto *layer : m_layers)
        /// C++:     for (auto *region : layer->m_regions) {
        /// C++:         region->prepare_fill_surfaces();
        /// C++:         m_print->throw_if_canceled();
        /// C++:     }
        for layer in &mut self.layers {
            for region in layer.regions_mut().iter_mut() {
                region.prepare_fill_surfaces();
                if self.canceled.load(std::sync::atomic::Ordering::Relaxed) {
                    return Err(crate::Error::Cancelled);
                }
            }
        }
        // TOPDBG (diagnostics only, env-gated): Top state after prepare_fill_surfaces.
        if crate::debug::topdbg::enabled() {
            for (idx_layer, layer) in self.layers.iter().enumerate() {
                for region in layer.regions().iter() {
                    crate::debug::topdbg::log_top_surfaces(
                        idx_layer,
                        "prepare_fill_surfaces",
                        &region.fill_surfaces.surfaces,
                    );
                }
            }
        }

        /// PrintObject.cpp:657-658
        /// C++: this->discover_vertical_shells();
        /// C++: m_print->throw_if_canceled();
        crate::stage_dump::pp_typed("b_prepfill", &self.layers);
        pi_phase!("discover_vertical_shells", self.discover_vertical_shells()?);
        crate::stage_dump::pp_typed("c_vshell", &self.layers);
        // TOPDBG (diagnostics only, env-gated): Top state after discover_vertical_shells.
        if crate::debug::topdbg::enabled() {
            for (idx_layer, layer) in self.layers.iter().enumerate() {
                for region in layer.regions().iter() {
                    crate::debug::topdbg::log_top_surfaces(
                        idx_layer,
                        "discover_vertical_shells",
                        &region.fill_surfaces.surfaces,
                    );
                    crate::debug::topdbg::dump_top_surfaces(
                        idx_layer,
                        "d2_vshell_top",
                        &region.fill_surfaces.surfaces,
                    );
                }
            }
        }

        /// PrintObject.cpp:669-670
        /// C++: this->process_external_surfaces();
        /// C++: m_print->throw_if_canceled();
        pi_phase!("process_external_surfaces", self.process_external_surfaces()?);
        crate::stage_dump::pp_typed("d_external", &self.layers);
        // TOPDBG (diagnostics only, env-gated): Top state after process_external_surfaces.
        if crate::debug::topdbg::enabled() {
            for (idx_layer, layer) in self.layers.iter().enumerate() {
                for region in layer.regions().iter() {
                    crate::debug::topdbg::log_top_surfaces(
                        idx_layer,
                        "process_external_surfaces",
                        &region.fill_surfaces.surfaces,
                    );
                    crate::debug::topdbg::dump_top_surfaces(
                        idx_layer,
                        "d3_process_external_top",
                        &region.fill_surfaces.surfaces,
                    );
                }
            }
        }

        /// PrintObject.cpp:689-691
        /// C++: this->discover_horizontal_shells();
        /// C++: m_print->throw_if_canceled();
        pi_phase!("discover_horizontal_shells", self.discover_horizontal_shells()?);
        crate::stage_dump::pp_typed("e_hshell", &self.layers);
        // TOPDBG (diagnostics only, env-gated): Top state after discover_horizontal_shells.
        if crate::debug::topdbg::enabled() {
            for (idx_layer, layer) in self.layers.iter().enumerate() {
                for region in layer.regions().iter() {
                    crate::debug::topdbg::log_top_surfaces(
                        idx_layer,
                        "discover_horizontal_shells",
                        &region.fill_surfaces.surfaces,
                    );
                    crate::debug::topdbg::dump_top_surfaces(
                        idx_layer,
                        "d4_horizontal_shells_top",
                        &region.fill_surfaces.surfaces,
                    );
                }
            }
        }

        /// PrintObject.cpp:701-702
        /// C++: if (m_config.interlocking_beam.value)
        /// C++:     discover_shell_for_perimeters();
        // TODO: Port interlocking_beam check and discover_shell_for_perimeters()

        /// PrintObject.cpp:703
        /// C++: reset_slice_surfaces(slice_surfaces_cpy);
        // TODO: Port reset_slice_surfaces()

        /// PrintObject.cpp:709-711
        /// C++: this->clip_fill_surfaces();
        /// C++: m_print->throw_if_canceled();
        // clip_fill_surfaces() (PrintObject.cpp:3309) is intentionally NOT ported: it is
        // DEAD CODE in BambuStudio. Its body is gated `if (! infill_only_where_needed) return;`
        // and `PrintObject::infill_only_where_needed` is a static member hard-initialized to
        // `false` (PrintObjectSlice.cpp:21) that is never assigned `true` anywhere in libslic3r.
        // So it always returns immediately and porting it would be a behavioral no-op.

        /// PrintObject.cpp:721-723
        /// C++: this->bridge_over_infill();
        /// C++: m_print->throw_if_canceled();
        pi_phase!("bridge_over_infill", self.bridge_over_infill()?);
        crate::stage_dump::pp_typed("g_bridge", &self.layers);
        if self.canceled.load(std::sync::atomic::Ordering::Relaxed) {
            return Err(crate::Error::Cancelled);
        }

        /// PrintObject.cpp:725-727
        /// C++: this->combine_infill();
        /// C++: m_print->throw_if_canceled();
        // TODO: Port combine_infill()

        /// PrintObject.cpp:1051-1052
        /// C++: this->discover_sub_top_surfaces();
        /// C++: m_print->throw_if_canceled();
        // SUB_TOP_SURFACES=0 leaves the solid layer under a top surface as
        // stInternalSolid, as before the 02.09 engine.
        if crate::faithful_gate("SUB_TOP_SURFACES") {
            pi_phase!("discover_sub_top_surfaces", self.discover_sub_top_surfaces()?);
        }
        crate::stage_dump::pp_typed("i_subtop", &self.layers);
        if self.canceled.load(std::sync::atomic::Ordering::Relaxed) {
            return Err(crate::Error::Cancelled);
        }

        if __pi_t {
            let __tot: f64 = __pi.iter().map(|(_, s)| s).sum();
            eprintln!("    prepare_infill sub-steps (s), total {__tot:.2}:");
            for (name, secs) in &__pi {
                eprintln!("      {name:<28} {secs:7.3}  ({:4.1}%)", 100.0 * secs / __tot);
            }
        }

        /// PrintObject.cpp:748
        /// C++: this->set_done(posPrepareInfill);
        self.set_step_done(PrintObjectStep::PrepareInfill);
        Ok(())
    }

    /// Port of `PrintObject::discover_sub_top_surfaces()` (PrintObject.cpp:5839-5920).
    ///
    /// Retypes to `stSubTop` every `stInternalSolid` fill surface that lies
    /// under a top fill surface of the layer above which is wide enough to
    /// matter. The surface keeps its geometry and every other field.
    fn discover_sub_top_surfaces(&mut self) -> Result<()> {
        use crate::clipper_utils::{
            clip_clipper_polygons_with_subject_bbox_expolygon, intersection_clib_safety,
            offset2_ex_clib, offset_expolygons_clib_scaled, union_ex_clib, OffsetJoinType,
        };
        use crate::flow::FlowRole;
        use crate::geometry::{BoundingBox, ExPolygon, ExPolygons, Polygon};
        use crate::surface::SurfaceType;

        // PrintObject.cpp:5837 — a top island that an erosion of this radius
        // wipes out is too narrow to justify a sub-top band underneath it.
        const SUB_TOP_MIN_TOP_EROSION_MM: f64 = 1.5;

        // PrintObject.cpp:5842-5843
        if self.layers.len() < 2 {
            return Ok(());
        }

        fn get_extents(expolygon: &ExPolygon) -> BoundingBox {
            BoundingBox::from_points(expolygon.contour.points())
        }

        // PrintObject.cpp:5903-5917 — a layer reads the tops of the layer above
        // and retypes its own fill surfaces only, so the order of the layers
        // does not change the result; they are visited in sequence here.
        for idx_layer in 0..self.layers.len() - 1 {
            // PrintObject.cpp:5850-5856
            let mut top_mask: ExPolygons = Vec::new();
            for upper_region in self.layers[idx_layer + 1].regions() {
                for s in &upper_region.fill_surfaces.surfaces {
                    if s.surface_type == SurfaceType::Top {
                        top_mask.push(s.expolygon.clone());
                    }
                }
            }
            if top_mask.is_empty() {
                continue;
            }

            // PrintObject.cpp:5858-5872
            let erosion: f32 = (SUB_TOP_MIN_TOP_EROSION_MM / crate::libslic3r::SCALING_FACTOR) as f32;
            let min_extent: crate::Coord =
                (2.0 * SUB_TOP_MIN_TOP_EROSION_MM / crate::libslic3r::SCALING_FACTOR) as crate::Coord;
            let mut large_tops: ExPolygons = Vec::new();
            let top_polygons: Vec<Polygon> = top_mask.iter().flat_map(|e| e.to_polygons()).collect();
            for top in union_ex_clib(&top_polygons, 1) {
                // An island narrower than the erosion diameter cannot survive it.
                let extents = get_extents(&top).size();
                if extents.x < min_extent || extents.y < min_extent {
                    continue;
                }
                if !offset_expolygons_clib_scaled(
                    std::slice::from_ref(&top),
                    -(erosion as f64),
                    OffsetJoinType::Miter,
                )
                .is_empty()
                {
                    large_tops.push(top);
                }
            }
            if large_tops.is_empty() {
                continue;
            }
            let top_mask = large_tops;

            // PrintObject.cpp:5874-5880
            let top_extents: Vec<BoundingBox> = top_mask.iter().map(get_extents).collect();

            // PrintObject.cpp:5882-5901
            let layer_height = self.layers[idx_layer].height;
            for layerm in self.layers[idx_layer].regions_mut() {
                let min_width: f32 = layerm.flow(FlowRole::SolidInfill, layer_height)?.scaled_spacing() as f32;
                for surface in layerm.fill_surfaces.surfaces.iter_mut() {
                    if surface.surface_type != SurfaceType::InternalSolid {
                        continue;
                    }
                    // Trim the mask to the island first. Opening then drops what
                    // only grazes the mask: an island is claimed on real
                    // overlap, not on a shared edge.
                    let island_bbox = get_extents(&surface.expolygon)
                        .expanded(crate::libslic3r::SCALED_EPSILON as crate::Coord);
                    let mut local_mask: Vec<Polygon> = Vec::new();
                    for i in 0..top_mask.len() {
                        if top_extents[i].intersects(&island_bbox) {
                            local_mask.extend(clip_clipper_polygons_with_subject_bbox_expolygon(
                                &top_mask[i],
                                &island_bbox,
                                false,
                            ));
                        }
                    }
                    if local_mask.is_empty() {
                        continue;
                    }
                    let local_mask: ExPolygons =
                        local_mask.into_iter().map(ExPolygon::new).collect();
                    let overlap = intersection_clib_safety(
                        std::slice::from_ref(&surface.expolygon),
                        &local_mask,
                    );
                    // opening_ex(…, 0.5f * min_width) = offset2_ex(-d, +d).
                    let d_mm = (0.5f32 * min_width) as f64 * crate::libslic3r::SCALING_FACTOR;
                    if !overlap.is_empty()
                        && !offset2_ex_clib(&overlap, -d_mm, d_mm, OffsetJoinType::Miter).is_empty()
                    {
                        surface.surface_type = SurfaceType::SubTop;
                    }
                }
            }
        }
        Ok(())
    }

    /// Find internal sparse/solid-infill regions that bridge over voids in the
    /// layer below, mark them `stInternalBridge`, determine the bridging angle,
    /// and adjust fill_surfaces accordingly.
    ///
    /// 1:1 (faithful) port of `PrintObject::bridge_over_infill()`
    /// (PrintObject.cpp:2167-3025).
    ///
    /// FIDELITY NOTES (no-adaptive / no-lightning simplifications):
    /// - The anchoring sparse-infill polylines (PrintObject.cpp:2372-2407 via
    ///   `Layer::generate_sparse_infill_polylines_for_anchoring`) ARE ported and
    ///   populate `infill_lines`, so `anchors` are real for grid / rectilinear /
    ///   line / concentric / gyroid sparse infill — this bounds bridge expansion
    ///   to match native. The adaptive-cubic octree and lightning generator
    ///   (`prepare_adaptive_infill_data`, `prepare_lightning_infill_data`) are
    ///   not ported, so those patterns emit no anchor lines and fall back to the
    ///   boundary path (PrintObject.cpp:2894); this is faithful for the common
    ///   case (Benchy uses grid infill).
    /// - The lightning-infill expansion section (PrintObject.cpp:2281-2370) is
    ///   skipped (no-op) when no region uses `ipLightning`, which is faithful.
    ///
    /// `bridge_over_infill` operates on `fill_surfaces` after
    /// `discover_horizontal_shells`. It does not depend on `clip_fill_surfaces`
    /// (PrintObject.cpp:709) having run — that step only trims fill surfaces
    /// against the slices, which is orthogonal to the void-detection here.
    fn bridge_over_infill(&mut self) -> Result<()> {
        use crate::aabb_tree_lines::LinesDistancer;
        use crate::clipper_utils::{
            diff_ex, diff_polygons, expand_polygons, intersection_ex_expolygons_polygons,
            intersection_ex_polygons_polygons, intersection_pl, to_polygons, union_polygons_ex,
            union_safety_offset_ex, union_safety_offset_ex_expolygons, ApplySafetyOffset,
        };
        use crate::flow::FlowRole;
        use crate::geometry::polygon::to_lines_polygons;
        use crate::geometry::{
            get_extents_lines, get_extents_polygons, polygons_rotate, to_lines_polylines,
            to_polygons as ex_to_polygons, to_polylines as ex_to_polylines, ExPolygon, Line, Point,
            Polygon, Polyline,
        };
        use crate::libslic3r::{EPSILON, SCALED_EPSILON};
        use crate::print_config::InfillPattern;
        use crate::surface::{Surface, SurfaceType, Surfaces};
        use std::collections::BTreeMap;
        use std::f64::consts::PI;

        // --- Local geometry helpers mirroring the C++ ClipperUtils `Polygons`
        // overloads. The crate's offset/boolean backend operates on ExPolygons
        // and unscales/rescales internally, so deltas are passed in mm
        // (= scaled_delta unscaled => use Flow::spacing()). We convert
        // Polygons<->ExPolygons at the boundaries via union_polygons_ex /
        // to_polygons, which is faithful to ClipperLib's implicit union.

        // C++ `union_(Polygons a, Polygons b)` -> Polygons
        let clib = crate::clipper_utils::polygons_ops_clib();
        let boi_pp = |tag: &str, l: usize, pp: &[Polygon], extra: f64| {
            if !crate::probe_enabled("BOI") {
                return;
            }
            let mut np = 0usize;
            let (mut sx, mut sy) = (0i64, 0i64);
            for p in pp {
                for q in p.points() {
                    np += 1;
                    sx += q.x;
                    sy += q.y;
                }
            }
            eprintln!("BOI {} L={} n={} np={} sx={} sy={} x={:.9}", tag, l, pp.len(), np, sx, sy, extra);
        };
        let boi_pl = |tag: &str, l: usize, pp: &[Polyline]| {
            if !crate::probe_enabled("BOI") {
                return;
            }
            let mut np = 0usize;
            let (mut sx, mut sy) = (0i64, 0i64);
            for p in pp {
                for q in p.points() {
                    np += 1;
                    sx += q.x;
                    sy += q.y;
                }
            }
            eprintln!("BOI {} L={} n={} np={} sx={} sy={} x=0", tag, l, pp.len(), np, sx, sy);
            if std::env::var("BOIP").ok().as_deref() == Some(&format!("{}:{}", tag, l)) {
                for (k, p) in pp.iter().enumerate() {
                    for q in p.points() {
                        eprintln!("BOIP {} {} {}", k, q.x, q.y);
                    }
                }
            }
        };
        let union2 = |a: &[Polygon], b: &[Polygon]| -> Vec<Polygon> {
            let mut all = a.to_vec();
            all.extend_from_slice(b);
            if clib {
                return crate::clipper_utils::union_flat_clib(&all);
            }
            crate::geometry::to_polygons(&union_polygons_ex(&all))
        };
        // C++ `intersection(Polygons, Polygons)` -> Polygons
        let intersect_polys = |a: &[Polygon], b: &[Polygon]| -> Vec<Polygon> {
            if clib {
                return crate::clipper_utils::intersection_polygons_raw_clib(a, b);
            }
            crate::geometry::to_polygons(&intersection_ex_polygons_polygons(
                a,
                b,
                ApplySafetyOffset::No,
            ))
        };
        // C++ `expand(Polygons, scaled_delta)` -> Polygons. delta_mm = scaled/scale.
        let expand_p = |a: &[Polygon], delta_mm: f64| -> Vec<Polygon> { expand_polygons(a, delta_mm) };
        // C++ `shrink(Polygons, scaled_delta)` -> Polygons.
        let shrink_p = |a: &[Polygon], delta_mm: f64| -> Vec<Polygon> {
            if a.is_empty() {
                return Vec::new();
            }
            if clib {
                return crate::clipper_utils::shrink_polygons_clib(a, delta_mm);
            }
            crate::geometry::to_polygons(&crate::clipper_utils::shrink(
                &union_polygons_ex(a),
                delta_mm,
                crate::clipper_utils::OffsetJoinType::Miter,
            ))
        };
        // C++ `closing(Polygons, delta)` -> Polygons = expand then shrink.
        let closing_p = |a: &[Polygon], delta_mm: f64| -> Vec<Polygon> {
            if a.is_empty() {
                return Vec::new();
            }
            if clib {
                return crate::clipper_utils::closing_polygons_clib(a, delta_mm, delta_mm);
            }
            let grown = crate::clipper_utils::grow(
                &union_polygons_ex(a),
                delta_mm,
                crate::clipper_utils::OffsetJoinType::Miter,
            );
            crate::geometry::to_polygons(&crate::clipper_utils::shrink(
                &grown,
                delta_mm,
                crate::clipper_utils::OffsetJoinType::Miter,
            ))
        };
        // C++ `opening(Polygons, delta)` -> Polygons = shrink then expand.
        let opening_p = |a: &[Polygon], delta_mm: f64| -> Vec<Polygon> {
            if a.is_empty() {
                return Vec::new();
            }
            if clib {
                return crate::clipper_utils::opening_polygons_clib(a, delta_mm, delta_mm);
            }
            let shrunk = crate::clipper_utils::shrink(
                &union_polygons_ex(a),
                delta_mm,
                crate::clipper_utils::OffsetJoinType::Miter,
            );
            crate::geometry::to_polygons(&crate::clipper_utils::grow(
                &shrunk,
                delta_mm,
                crate::clipper_utils::OffsetJoinType::Miter,
            ))
        };
        // C++ `to_polygons(const SurfacesPtr&)` — collect contours + holes from
        // borrowed surfaces. (clipper_utils::to_polygons only takes &[Surface].)
        fn surfaces_ptr_to_polygons(surfaces: &[&Surface]) -> Vec<Polygon> {
            let mut polygons = Vec::new();
            for s in surfaces {
                polygons.push(s.expolygon.contour.clone());
                for hole in &s.expolygon.holes {
                    polygons.push(hole.clone());
                }
            }
            polygons
        }

        // ====================================================================
        // CandidateSurface: a region that needs an internal bridge.
        // PrintObject.cpp:2172-2190
        // We cannot keep a raw `&Surface` pointer like the C++ does
        // (`original_surface`) because of Rust borrow rules across the mutating
        // apply phase; instead we identify the originating surface by
        // (layer_idx, region_idx, surface_idx) into the pre-bridge snapshot of
        // stInternalSolid surfaces. The matching at apply-time
        // (PrintObject.cpp:2986 `cs.original_surface == surface`) is reproduced
        // by comparing those indices.
        // ====================================================================
        #[derive(Clone)]
        struct CandidateSurface {
            // index of the stInternalSolid surface (within the region) on the
            // candidate's own layer that this bridge originates from.
            original_solid_idx: usize,
            // index of the region on the candidate's layer.
            region_idx: usize,
            // layer index of the surface to be bridged.
            layer_index: usize,
            // polygons to be bridged / (after expansion) the bridged area.
            new_polys: Vec<Polygon>,
            // bridge angle (radians).
            bridge_angle: f64,
        }

        let n_layers = self.layers.len();
        if n_layers == 0 {
            return Ok(());
        }

        // surfaces_by_layer (ordered) — PrintObject.cpp:2193
        let mut surfaces_by_layer: BTreeMap<usize, Vec<CandidateSurface>> = BTreeMap::new();

        // ====================================================================
        let __bt = crate::probe_enabled("SLICE_PHASE_TIMING");
        let __b0 = std::time::Instant::now();
        // SECTION: gather and filter surfaces for expanding, cluster by layer.
        // PrintObject.cpp:2196-2279
        // ====================================================================
        // R394: candidate extraction is independent per layer (reads own +
        // lower layer immutably, emits candidates for its own map key). Compute
        // in parallel, then insert into the ordered map serially — byte-identical.
        let layers_ref = &self.layers;
        let per_layer_cands: Result<Vec<Vec<CandidateSurface>>> = {
            use rayon::prelude::*;
            (0..n_layers)
                .into_par_iter()
                .map(|lidx| -> Result<Vec<CandidateSurface>> {
            let mut cands: Vec<CandidateSurface> = Vec::new();
            // PrintObject.cpp:2208 — skip first layer (no lower layer).
            if layers_ref[lidx].lower_layer_id.is_none() {
                return Ok(cands);
            }
            let lower_idx = layers_ref[lidx].lower_layer_id.unwrap();
            let layer_height = layers_ref[lidx].height;

            // PrintObject.cpp:2211 — spacing from solid-infill flow of first region.
            // PrintObject.cpp:2794 — `double spacing = …flow(frSolidInfill)
            // .scaled_spacing()`: the truncated coord_t, which every delta below
            // is a multiple of.
            let spacing = {
                let region0 = &layers_ref[lidx].regions()[0];
                let f = region0.flow(FlowRole::SolidInfill, layer_height)?;
                if clib {
                    f.scaled_spacing() as f64 / crate::SCALING_FACTOR
                } else {
                    f.spacing()
                }
            };

            // PrintObject.cpp:2213-2226 — gather lower-layer fill & solids.
            let mut unsupported_area: Vec<Polygon> = Vec::new();
            let mut lower_layer_solids: Vec<Polygon> = Vec::new();
            for region in layers_ref[lower_idx].regions() {
                // PrintObject.cpp:2217-2219 — whole lower fill considered unsupported.
                let fill_polys = ex_to_polygons(&region.fill_expolygons);
                unsupported_area.extend(fill_polys);
                // PrintObject.cpp:2220-2225 — gather solid (supporting) areas.
                let dense = region.region().config().fill_density >= 1.0;
                for surface in &region.fill_surfaces.surfaces {
                    if surface.surface_type != SurfaceType::Internal || dense {
                        lower_layer_solids.extend(surface.expolygon.to_polygons());
                    }
                }
            }
            // PrintObject.cpp:2227 — close unsupported.
            unsupported_area = closing_p(&unsupported_area, crate::unscale(SCALED_EPSILON as i64));
            // PrintObject.cpp:2230-2231 — opening of solids (remove thin, expand back +3).
            lower_layer_solids = shrink_p(&lower_layer_solids, 1.0 * spacing);
            lower_layer_solids = expand_p(&lower_layer_solids, (1.0 + 3.0) * spacing);
            // PrintObject.cpp:2233-2234 — shrink unsupported, subtract solids.
            unsupported_area = shrink_p(&unsupported_area, 3.0 * spacing);
            unsupported_area = diff_polygons(&unsupported_area, &lower_layer_solids);
            // PrintObject.cpp:2236-2268 — per-region candidate extraction.
            let n_regions = layers_ref[lidx].regions().len();
            for region_idx in 0..n_regions {
                // Snapshot the stInternalSolid surfaces of this region. We track
                // their index so the apply phase can match them.
                let solid_exs_indexed: Vec<(usize, ExPolygon)> = {
                    let region = &layers_ref[lidx].regions()[region_idx];
                    region
                        .fill_surfaces
                        .surfaces
                        .iter()
                        .enumerate()
                        .filter(|(_, s)| s.surface_type == SurfaceType::InternalSolid)
                        .map(|(i, s)| (i, s.expolygon.clone()))
                        .collect()
                };

                for (solid_idx, s_ex) in &solid_exs_indexed {
                    let s_polys = s_ex.to_polygons();
                    // PrintObject.cpp:2239 — unsupported part of this solid.
                    let unsupported = intersect_polys(&s_polys, &unsupported_area);
                    // PrintObject.cpp:2242 — partially_supported test.
                    let area_unsupported = crate::geometry::area_polygons(&unsupported);
                    let area_solid = crate::geometry::area_polygons(&s_polys);
                    let partially_supported = area_unsupported < area_solid - EPSILON;
                    // PrintObject.cpp:2243 — `area(unsupported) > 3*3*spacing*spacing`.
                    // C++ `spacing` is scaled_spacing() (scaled coord units) and
                    // `area(...)` is in scaled^2 units, so the threshold must be
                    // built from SCALED spacing. `area_unsupported` here is
                    // `area_polygons` over scaled polygons => scaled^2; using the
                    // unscaled mm `spacing` made the threshold ~1.8 (effectively
                    // zero) and never filtered partially-supported overhangs.
                    let spacing_scaled = crate::scale(spacing) as f64;
                    if !unsupported.is_empty()
                        && (!partially_supported
                            || area_unsupported > 3.0 * 3.0 * spacing_scaled * spacing_scaled)
                    {
                        // PrintObject.cpp:2244 — worth_bridging.
                        let mut worth_bridging =
                            intersect_polys(&s_polys, &expand_p(&unsupported, 4.0 * spacing));
                        // PrintObject.cpp:2246-2251 — merge tiny leftovers back.
                        let leftovers = diff_polygons(
                            &s_polys,
                            &expand_p(&worth_bridging, spacing),
                        );
                        for p in &leftovers {
                            let area = p.area();
                            // scale_(12.0) = scale(12.0); spacing here is mm so
                            // compare with scaled metrics as in C++ (which uses
                            // scaled spacing). Convert spacing->scaled.
                            let spacing_scaled = crate::scale(spacing) as f64;
                            if area < spacing_scaled * (crate::scale(12.0) as f64)
                                && area > spacing_scaled * spacing_scaled
                            {
                                worth_bridging.push(p.clone());
                            }
                        }
                        // PrintObject.cpp:2252 — closing & clip to solid.
                        let closed = closing_p(&worth_bridging, crate::unscale(SCALED_EPSILON as i64));
                        worth_bridging = if clib {
                            // intersection(Polygons, ExPolygon) — raw paths out.
                            crate::clipper_utils::intersection_polygons_raw_clib(&closed, &s_polys)
                        } else {
                            crate::geometry::to_polygons(&crate::clipper_utils::intersection(
                                &union_polygons_ex(&closed),
                                std::slice::from_ref(s_ex),
                            ))
                        };

                        // PrintObject.cpp:2254 — record candidate.
                        cands.push(CandidateSurface {
                            original_solid_idx: *solid_idx,
                            region_idx,
                            layer_index: lidx,
                            new_polys: worth_bridging,
                            bridge_angle: 0.0,
                        });
                    }
                }
            }
            Ok(cands)
                })
                .collect()
        };
        // Serial insert into the ordered map (per-layer order preserved).
        for (lidx, cands) in per_layer_cands?.into_iter().enumerate() {
            if !cands.is_empty() {
                for c in &cands {
                    boi_pp("cand", lidx, &c.new_polys, 0.0);
                }
                surfaces_by_layer.insert(lidx, cands);
            }
        }

        // ====================================================================
        let __b1 = std::time::Instant::now();
        // LIGHTNING INFILL SECTION — PrintObject.cpp:2281-2370.
        // Skipped: no region uses ipLightning in the common path. Faithful
        // no-op when has_lightning_infill == false.
        // ====================================================================
        let has_lightning_infill = (0..self.num_printing_regions())
            .any(|i| self.printing_region(i).unwrap().config().fill_pattern == InfillPattern::Lightning);
        if has_lightning_infill {
            // The lightning expansion path is not ported (depends on the
            // lightning generator). Surfaces are left as gathered.
        }

        // ====================================================================
        // SECTION to generate infill polylines — PrintObject.cpp:2372-2407.
        // For each layer that hosts a bridge candidate (key `lidx` in
        // `surfaces_by_layer`), populate `infill_lines[lidx - 1]` with the
        // sparse-infill anchor polylines of the layer below, via
        // `Layer::generate_sparse_infill_polylines_for_anchoring`
        // (PrintObject.cpp:2384-2401). These anchors are intersected with the
        // shrunk expansion area (PrintObject.cpp:2845) and inserted into the
        // anchor/wall AABB tree (PrintObject.cpp:2621) that bounds bridge
        // expansion; without them the bridge over-expands.
        //
        // Adaptive-cubic / lightning octrees are not ported: the anchoring
        // function emits no lines for those patterns, which is faithful to the
        // common grid/rectilinear sparse-infill case.
        let mut infill_lines: BTreeMap<usize, Vec<Polyline>> = BTreeMap::new();
        {
            // PrintObject.cpp:2384-2389 — collect the lower layer indices that
            // need anchoring infill (lidx-1 for each candidate layer lidx>0).
            let mut layers_to_generate_infill: Vec<usize> = surfaces_by_layer
                .keys()
                .filter(|&&lidx| lidx > 0)
                .map(|&lidx| lidx - 1)
                .collect();
            layers_to_generate_infill.sort_unstable();
            layers_to_generate_infill.dedup();
            // PrintObject.cpp:2391-2401 — per-layer anchoring polylines.
            // R395: generate_sparse_infill_polylines_for_anchoring(&self) is a pure
            // per-layer read → compute in parallel, insert into the ordered map
            // serially. Byte-identical (each lower_idx independent).
            let layers_ref = &self.layers;
            let anchor_results: Result<Vec<(usize, Vec<Polyline>)>> = {
                use rayon::prelude::*;
                layers_to_generate_infill
                    .into_par_iter()
                    .map(|lower_idx| {
                        Ok((
                            lower_idx,
                            // Fill.cpp:228-229: layers().front()->bottom_z() (FILL_GROUP_PIN reads it).
                            layers_ref[lower_idx].generate_sparse_infill_polylines_for_anchoring_at_object_base(
                                layers_ref.first().map_or(0., |l| l.bottom_z()),
                            )?,
                        ))
                    })
                    .collect()
            };
            for (lower_idx, lines) in anchor_results? {
                boi_pl("lines", lower_idx, &lines);
                infill_lines.insert(lower_idx, lines);
            }
        }

        // ====================================================================
        // Cluster layers by depth for thick bridges. PrintObject.cpp:2409-2463.
        // ====================================================================
        let target_flow_height_factor: f64 = 0.9;
        let mut clustered_layers_for_threads: Vec<Vec<usize>> = Vec::new();
        {
            // PrintObject.cpp:2414-2434 — inflated AABB union per layer.
            let mut layer_area_covered_by_candidates: BTreeMap<usize, Vec<Polygon>> =
                BTreeMap::new();
            for (&lidx, cands) in &surfaces_by_layer {
                let mut cover: Vec<Polygon> = Vec::new();
                for candidate in cands {
                    // PrintObject.cpp:2429 — inflated AABB polygon (inflate scale_(7)).
                    let mut bb = get_extents_polygons(&candidate.new_polys);
                    // BoundingBox::inflated(delta) — geometry's BoundingBox has no
                    // such method; inflate the corners manually (scaled units).
                    let delta = crate::scale(7.0);
                    bb.min.x -= delta;
                    bb.min.y -= delta;
                    bb.max.x += delta;
                    bb.max.y += delta;
                    let inflated = bb.polygon();
                    cover = union2(&cover, std::slice::from_ref(&inflated));
                }
                layer_area_covered_by_candidates.insert(lidx, cover);
            }

            // PrintObject.cpp:2437-2451 — z-proximity + overlap clustering.
            for (&lidx, _) in &surfaces_by_layer {
                let new_group = if clustered_layers_for_threads.is_empty() {
                    true
                } else {
                    let back_layer = *clustered_layers_for_threads.last().unwrap().last().unwrap();
                    let cur_print_z = self.layers[lidx].print_z;
                    let bridging_h = {
                        let r0 = &self.layers[lidx].regions()[0];
                        r0.bridging_flow(FlowRole::SolidInfill, true, self.layers[lidx].height)?
                            .height()
                    };
                    let z_far = self.layers[back_layer].print_z
                        < cur_print_z - bridging_h * target_flow_height_factor - EPSILON;
                    let no_overlap = intersect_polys(
                        &layer_area_covered_by_candidates[&back_layer],
                        &layer_area_covered_by_candidates[&lidx],
                    )
                    .is_empty();
                    z_far || no_overlap
                };
                if new_group {
                    clustered_layers_for_threads.push(vec![lidx]);
                } else {
                    clustered_layers_for_threads.last_mut().unwrap().push(lidx);
                }
            }
        }

        // ====================================================================
        // determine_bridging_angle lambda — PrintObject.cpp:2497-2580.
        // ====================================================================
        let determine_bridging_angle =
            |bridged_area: &[Polygon], anchors: &[Line], dominant_pattern: InfillPattern, infill_direction: f64| -> f64 {
                let lines_tree = LinesDistancer::new(anchors.to_vec());
                // PrintObject.cpp:2501-2505 — fixed-angle patterns.
                match dominant_pattern {
                    InfillPattern::Honeycomb3D | InfillPattern::CrossHatch => {
                        return (infill_direction + 45.0) * 2.0 * PI / 360.0;
                    }
                    _ => {}
                }

                // PrintObject.cpp:2508-2534 — accumulate directions.
                let mut counted_directions: BTreeMap<i64, i32> = BTreeMap::new();
                // We key by quantized angle to mirror std::map<double,int>;
                // use a fine quantization to avoid collisions while remaining
                // ordered. (1e9 buckets over the angle range.)
                let quant = 1.0e9_f64;
                if !anchors.is_empty() {
                    for p in bridged_area {
                        let mut acc_distance = 0.0_f64;
                        let pts = &p.points;
                        for point_idx in 0..(pts.len().saturating_sub(1)) {
                            let start = [pts[point_idx].x as f64, pts[point_idx].y as f64];
                            let next = [pts[point_idx + 1].x as f64, pts[point_idx + 1].y as f64];
                            let mut v = [next[0] - start[0], next[1] - start[1]];
                            let dist_to_next = (v[0] * v[0] + v[1] * v[1]).sqrt();
                            acc_distance += dist_to_next;
                            if acc_distance > crate::scale(2.0) as f64 {
                                acc_distance = 0.0;
                                if dist_to_next > 0.0 {
                                    v[0] /= dist_to_next;
                                    v[1] /= dist_to_next;
                                }
                                let lines_count =
                                    ((dist_to_next / crate::scale(2.0) as f64).ceil()) as i32;
                                let lines_count = lines_count.max(1);
                                let step_size = dist_to_next / lines_count as f64;
                                for i in 0..lines_count {
                                    let ax = start[0] + v[0] * (i as f64 * step_size);
                                    let ay = start[1] + v[1] * (i as f64 * step_size);
                                    let a = Point::new(ax.round() as i64, ay.round() as i64);
                                    let (_d, index, _np) =
                                        lines_tree.distance_from_lines_extra::<false>(a);
                                    let mut angle = lines_tree.get_line(index).orientation();
                                    if angle > PI {
                                        angle -= PI;
                                    }
                                    angle += PI * 0.5;
                                    *counted_directions
                                        .entry((angle * quant).round() as i64)
                                        .or_insert(0) += 1;
                                }
                            }
                        }
                    }
                }

                // PrintObject.cpp:2536-2568 — sliding-window best direction.
                let mut best_dir: (f64, i32) = (0.0, 0);
                let keys: Vec<(f64, i32)> = counted_directions
                    .iter()
                    .map(|(&k, &c)| (k as f64 / quant, c))
                    .collect();
                for &(dir_angle, _) in &keys {
                    let mut score_acc = 0_i32;
                    let mut dir_acc = 0.0_f64;
                    let window_start_angle = dir_angle - PI * 0.1;
                    let window_end_angle = dir_angle + PI * 0.1;
                    for &(a, c) in &keys {
                        if a >= window_start_angle && a < window_end_angle {
                            dir_acc += a * c as f64;
                            score_acc += c;
                        }
                    }
                    if window_start_angle < 0.5 * PI {
                        let lb = 1.5 * PI - (0.5 * PI - window_start_angle);
                        for &(a, c) in &keys {
                            if a >= lb {
                                dir_acc += a * c as f64;
                                score_acc += c;
                            }
                        }
                    }
                    if window_start_angle > 1.5 * PI {
                        let ub = window_start_angle - 1.5 * PI;
                        for &(a, c) in &keys {
                            if a < ub {
                                dir_acc += a * c as f64;
                                score_acc += c;
                            }
                        }
                    }
                    if score_acc > best_dir.1 {
                        best_dir = (dir_acc / score_acc as f64, score_acc);
                    }
                }
                // PrintObject.cpp:2569-2577
                let mut bridging_angle = best_dir.0;
                if bridging_angle == 0.0 {
                    bridging_angle = 0.001;
                }
                match dominant_pattern {
                    InfillPattern::HilbertCurve => bridging_angle += 0.25 * PI,
                    InfillPattern::OctagramSpiral => bridging_angle += (1.0 / 16.0) * PI,
                    _ => {}
                }
                bridging_angle
            };

        // ====================================================================
        // construct_anchored_polygon lambda — PrintObject.cpp:2584-2752.
        // ====================================================================
        let construct_anchored_polygon = |bridged_area_in: &[Polygon],
                                          anchors_in: &[Line],
                                          flow_scaled_spacing: i64,
                                          flow_scaled_width: i64,
                                          bridging_angle: f64|
         -> Vec<Polygon> {
            let scaled_spacing = flow_scaled_spacing;
            let scaled_width = flow_scaled_width;
            let mut expanded_bridged_area: Vec<Polygon> = Vec::new();
            // PrintObject.cpp:2604
            let aligning_angle = -bridging_angle + PI * 0.5;

            // PrintObject.cpp:2606-2607 — rotate inputs into alignment.
            let mut bridged_area = bridged_area_in.to_vec();
            polygons_rotate(&mut bridged_area, aligning_angle);
            let cos_a = aligning_angle.cos();
            let sin_a = aligning_angle.sin();
            let mut anchors = anchors_in.to_vec();
            for l in &mut anchors {
                let ax = l.a.x as f64;
                let ay = l.a.y as f64;
                l.a.x = (cos_a * ax - sin_a * ay).round() as i64;
                l.a.y = (cos_a * ay + sin_a * ax).round() as i64;
                let bx = l.b.x as f64;
                let by = l.b.y as f64;
                l.b.x = (cos_a * bx - sin_a * by).round() as i64;
                l.b.y = (cos_a * by + sin_a * bx).round() as i64;
            }

            // PrintObject.cpp:2608-2619 — build vertical scan lines.
            let bb_x = get_extents_polygons(&bridged_area);
            let bb_y = get_extents_lines(&anchors);
            if scaled_spacing <= 0 {
                return Vec::new();
            }
            let n_vlines = ((bb_x.max.x - bb_x.min.x + scaled_spacing - 1) / scaled_spacing).max(0)
                as usize;
            let mut vertical_lines: Vec<Line> = Vec::with_capacity(n_vlines);
            for i in 0..n_vlines {
                let x = bb_x.min.x + i as i64 * scaled_spacing;
                let y_min = bb_y.min.y - scaled_spacing;
                let y_max = bb_y.max.y + scaled_spacing;
                vertical_lines.push(Line::new(Point::new(x, y_min), Point::new(x, y_max)));
            }

            // PrintObject.cpp:2621-2622
            let anchors_and_walls_tree = LinesDistancer::new(anchors.clone());
            let bridged_area_tree = LinesDistancer::new(to_lines_polygons(&bridged_area));

            // PrintObject.cpp:2624-2671 — per scan line, build sections.
            let mut polygon_sections: Vec<Vec<Line>> = vec![Vec::new(); n_vlines];
            for i in 0..n_vlines {
                let area_intersections =
                    bridged_area_tree.intersections_with_line::<true>(&vertical_lines[i]);
                for ii in 0..(area_intersections.len().saturating_sub(1)) {
                    let mid = Point::new(
                        (area_intersections[ii].0.x + area_intersections[ii + 1].0.x) / 2,
                        (area_intersections[ii].0.y + area_intersections[ii + 1].0.y) / 2,
                    );
                    if bridged_area_tree.outside(mid) < 0 {
                        polygon_sections[i].push(Line::new(
                            area_intersections[ii].0,
                            area_intersections[ii + 1].0,
                        ));
                    }
                }
                let anchors_intersections =
                    anchors_and_walls_tree.intersections_with_line::<true>(&vertical_lines[i]);

                for section in polygon_sections[i].iter_mut() {
                    // PrintObject.cpp:2637-2644 — extend low end to anchor below.
                    // C++: maybe_below_anchor = std::upper_bound(rbegin, rend, section.a,
                    //          [](Point a, pair b){ return a.y() > b.first.y(); });
                    // upper_bound returns the first element (scanning the reversed —
                    // i.e. descending-y — range) for which the predicate is TRUE, i.e.
                    // the nearest anchor strictly below section.a. Replicated as the first
                    // `section.a.y > ai.y` in the reversed scan (NOT its negation — the
                    // negation picked the nearest anchor *above*, extending the section the
                    // wrong way and over-widening the bridge).
                    let mut chosen_below: Option<Point> = None;
                    for ai in anchors_intersections.iter().rev() {
                        if section.a.y > ai.0.y {
                            chosen_below = Some(ai.0);
                            break;
                        }
                    }
                    if let Some(p) = chosen_below {
                        section.a = p;
                        section.a.y -= scaled_width; // (0.5 + 0.5) * scaled_width
                    }
                    // PrintObject.cpp:2646-2653 — extend high end to anchor above.
                    // C++: maybe_upper_anchor = std::upper_bound(begin, end, section.b,
                    //          [](Point a, pair b){ return a.y() < b.first.y(); });
                    // First element (ascending-y scan) for which the predicate is TRUE =
                    // nearest anchor strictly above section.b.
                    let mut chosen_above: Option<Point> = None;
                    for ai in anchors_intersections.iter() {
                        if section.b.y < ai.0.y {
                            chosen_above = Some(ai.0);
                            break;
                        }
                    }
                    if let Some(p) = chosen_above {
                        section.b = p;
                        section.b.y += scaled_width;
                    }
                }

                // PrintObject.cpp:2656-2664 — merge overlapping sections.
                let len = polygon_sections[i].len();
                for sidx in 0..len.saturating_sub(1) {
                    let (a_a, a_b) = (polygon_sections[i][sidx].a, polygon_sections[i][sidx].b);
                    let (b_a, b_b) = (
                        polygon_sections[i][sidx + 1].a,
                        polygon_sections[i][sidx + 1].b,
                    );
                    let alow = a_a.y;
                    let ahigh = a_b.y;
                    let blow = b_a.y;
                    let bhigh = b_b.y;
                    let overlap = (alow >= blow && alow <= bhigh)
                        || (ahigh >= blow && ahigh <= bhigh)
                        || (blow >= alow && blow <= ahigh)
                        || (bhigh >= alow && bhigh <= ahigh);
                    if overlap {
                        let new_b_a = if a_a.y < b_a.y { a_a } else { b_a };
                        let new_b_b = if a_b.y < b_b.y { b_b } else { a_b };
                        polygon_sections[i][sidx + 1].a = new_b_a;
                        polygon_sections[i][sidx + 1].b = new_b_b;
                        polygon_sections[i][sidx].a = polygon_sections[i][sidx].b;
                    }
                }
                // PrintObject.cpp:2666-2670 — drop degenerate, sort.
                polygon_sections[i].retain(|s| s.a != s.b);
                polygon_sections[i].sort_by(|a, b| a.a.y.cmp(&b.b.y));
            }

            // PrintObject.cpp:2674-2746 — reconstruct polygons from sections.
            struct TracedPoly {
                lows: Vec<Point>,
                highs: Vec<Point>,
            }
            let mut current_traced_polys: Vec<TracedPoly> = Vec::new();
            let ss_sq = 36.0 * scaled_spacing as f64 * scaled_spacing as f64;
            for polygon_slice in &polygon_sections {
                let mut used_segments: std::collections::HashSet<usize> =
                    std::collections::HashSet::new();
                for traced_poly in current_traced_polys.iter_mut() {
                    // candidate range — PrintObject.cpp:2684-2687.
                    let last_low = *traced_poly.lows.last().unwrap();
                    let last_high = *traced_poly.highs.last().unwrap();
                    // candidates_begin: first seg with seg.b.y > low.y
                    // candidates_end: first seg with seg.a.y > high.y
                    let mut begin_idx = polygon_slice.len();
                    for (idx, seg) in polygon_slice.iter().enumerate() {
                        if seg.b.y > last_low.y {
                            begin_idx = idx;
                            break;
                        }
                    }
                    let mut end_idx = polygon_slice.len();
                    for (idx, seg) in polygon_slice.iter().enumerate() {
                        if seg.a.y > last_high.y {
                            end_idx = idx;
                            break;
                        }
                    }

                    let mut segment_added = false;
                    let mut cand_idx = begin_idx;
                    while cand_idx < end_idx && !segment_added {
                        if used_segments.contains(&cand_idx) {
                            cand_idx += 1;
                            continue;
                        }
                        let candidate = &polygon_slice[cand_idx];
                        // lows
                        let dx = (last_low.x - candidate.a.x) as f64;
                        let dy = (last_low.y - candidate.a.y) as f64;
                        if dx * dx + dy * dy < ss_sq {
                            traced_poly.lows.push(candidate.a);
                        } else {
                            let lb = *traced_poly.lows.last().unwrap();
                            traced_poly
                                .lows
                                .push(Point::new(lb.x + scaled_spacing / 2, lb.y));
                            traced_poly
                                .lows
                                .push(Point::new(candidate.a.x - scaled_spacing / 2, candidate.a.y));
                            traced_poly.lows.push(candidate.a);
                        }
                        // highs
                        let dxh = (last_high.x - candidate.b.x) as f64;
                        let dyh = (last_high.y - candidate.b.y) as f64;
                        if dxh * dxh + dyh * dyh < ss_sq {
                            traced_poly.highs.push(candidate.b);
                        } else {
                            let hb = *traced_poly.highs.last().unwrap();
                            traced_poly
                                .highs
                                .push(Point::new(hb.x + scaled_spacing / 2, hb.y));
                            traced_poly.highs.push(Point::new(
                                candidate.b.x - scaled_spacing / 2,
                                candidate.b.y,
                            ));
                            traced_poly.highs.push(candidate.b);
                        }
                        segment_added = true;
                        used_segments.insert(cand_idx);
                        cand_idx += 1;
                    }

                    if !segment_added {
                        // PrintObject.cpp:2716-2724 — close polygon.
                        let lb = *traced_poly.lows.last().unwrap();
                        traced_poly
                            .lows
                            .push(Point::new(lb.x + scaled_spacing / 2, lb.y));
                        let hb = *traced_poly.highs.last().unwrap();
                        traced_poly
                            .highs
                            .push(Point::new(hb.x + scaled_spacing / 2, hb.y));
                        let mut pts = std::mem::take(&mut traced_poly.lows);
                        pts.extend(traced_poly.highs.iter().rev().cloned());
                        expanded_bridged_area.push(Polygon::from_points(pts));
                        traced_poly.highs.clear();
                    }
                }
                // PrintObject.cpp:2727-2729 — drop emptied traced polys.
                current_traced_polys.retain(|tp| !tp.lows.is_empty());

                // PrintObject.cpp:2731-2739 — start new traced polys.
                for (idx, segment) in polygon_slice.iter().enumerate() {
                    if !used_segments.contains(&idx) {
                        current_traced_polys.push(TracedPoly {
                            lows: vec![
                                Point::new(segment.a.x - scaled_spacing / 2, segment.a.y),
                                segment.a,
                            ],
                            highs: vec![
                                Point::new(segment.b.x - scaled_spacing / 2, segment.b.y),
                                segment.b,
                            ],
                        });
                    }
                }
            }

            // PrintObject.cpp:2743-2746 — add not-closed polys.
            for mut traced_poly in current_traced_polys {
                let mut pts = std::mem::take(&mut traced_poly.lows);
                pts.extend(traced_poly.highs.iter().rev().cloned());
                expanded_bridged_area.push(Polygon::from_points(pts));
            }
            // PrintObject.cpp:2747
            let mut out = crate::geometry::to_polygons(&union_safety_offset_ex(&expanded_bridged_area));
            // PrintObject.cpp:2750 — rotate back.
            polygons_rotate(&mut out, -aligning_angle);
            out
        };

        // ====================================================================
        // gather_areas_w_depth — PrintObject.cpp:2466-2494. Implemented inline
        // in the cluster loop below (needs &self).
        // ====================================================================

        // ====================================================================
        let __b2 = std::time::Instant::now();
        // Main expand loop — PrintObject.cpp:2754-2942.
        // C++: tbb::parallel_for(tbb::blocked_range<size_t>(0, clustered_layers_for_threads.size()),
        // C++:     [po = static_cast<const PrintObject *>(this), target_flow_height_factor,
        // C++:      &surfaces_by_layer, &clustered_layers_for_threads, gather_areas_w_depth,
        // C++:      &infill_lines, determine_bridging_angle, construct_anchored_polygon](
        // C++:         tbb::blocked_range<size_t> r) {
        // C++:             for (size_t cluster_idx = r.begin(); ...) for (size_t job_idx = 0; ...)
        // C++ (2765-2766): "this thread has exclusive access to all surfaces in layers
        // enumerated in clustered_layers_for_threads[cluster_idx]" — clusters were built
        // (PrintObject.cpp:2437-2451) so no candidate layer is in two clusters and every
        // within-cluster lower-layer lookup stays inside the cluster; that is why C++ shares
        // `surfaces_by_layer` across threads without locking. Rust makes the disjointness
        // explicit: move each cluster's candidate entries OUT of the shared map into a
        // per-cluster owned job list, run the clusters in parallel over their private jobs,
        // then reinsert into the map for the untouched Apply loop below. rayon stands in for
        // tbb; `try_for_each` propagates cancellation like throw_if_canceled.
        // ====================================================================
        use rayon::prelude::*;
        let mut cluster_jobs: Vec<Vec<(usize, Vec<CandidateSurface>)>> =
            Vec::with_capacity(clustered_layers_for_threads.len());
        for cluster in &clustered_layers_for_threads {
            let mut jobs: Vec<(usize, Vec<CandidateSurface>)> = Vec::with_capacity(cluster.len());
            for &lidx in cluster {
                let cands = surfaces_by_layer
                    .remove(&lidx)
                    .expect("clustered candidate layer must be present in surfaces_by_layer");
                jobs.push((lidx, cands));
            }
            cluster_jobs.push(jobs);
        }
        // Immutable captures mirroring the C++ lambda captures at PrintObject.cpp:2754-2759
        // (m_layers read through `po`; helper lambdas + infill_lines shared by reference).
        let layers = &self.layers;
        let canceled = self.canceled.clone();
        cluster_jobs
            .par_iter_mut()
            .try_for_each(|jobs| -> Result<()> {
            // Cancellation guard once per cluster (C++ throw_if_canceled).
            if canceled.load(Ordering::Relaxed) {
                return Err(Error::Cancelled);
            }
            for job_idx in 0..jobs.len() {
                let lidx = jobs[job_idx].0;
                let layer_height = layers[lidx].height;
                let print_z = layers[lidx].print_z;

                // PrintObject.cpp:2770-2790 — presort candidates.
                {
                    let cands = &mut jobs[job_idx].1;
                    cands.sort_by(|left, right| {
                        let a = get_extents_polygons(&left.new_polys);
                        let b = get_extents_polygons(&right.new_polys);
                        if a.min.x == b.min.x {
                            a.min.y.cmp(&b.min.y)
                        } else {
                            a.min.x.cmp(&b.min.x)
                        }
                    });
                    if cands.len() > 2 {
                        let origin = get_extents_polygons(&cands[0].new_polys).max;
                        let ox = origin.x as f64;
                        let oy = origin.y as f64;
                        cands[1..].sort_by(|left, right| {
                            let a = get_extents_polygons(&left.new_polys).min;
                            let b = get_extents_polygons(&right.new_polys).min;
                            let da = (ox - a.x as f64).powi(2) + (oy - a.y as f64).powi(2);
                            let db = (ox - b.x as f64).powi(2) + (oy - b.y as f64).powi(2);
                            da.partial_cmp(&db).unwrap_or(std::cmp::Ordering::Equal)
                        });
                    }
                }

                // PrintObject.cpp:2793-2795 — bridging flow / target height.
                let (front_region_idx, bridging_flow) = {
                    let cand0 = &jobs[job_idx].1[0];
                    let r = &layers[lidx].regions()[cand0.region_idx];
                    (
                        cand0.region_idx,
                        r.bridging_flow(FlowRole::SolidInfill, true, layer_height)?,
                    )
                };
                let _ = front_region_idx;
                let spacing_scaled = bridging_flow.scaled_spacing();
                let spacing_mm = if clib {
                    spacing_scaled as f64 / crate::SCALING_FACTOR
                } else {
                    bridging_flow.spacing()
                };
                let target_flow_height = bridging_flow.height() * target_flow_height_factor;

                // gather_areas_w_depth — PrintObject.cpp:2466-2494 / called 2797.
                let mut deep_infill_area = {
                    let bottom_z = print_z - target_flow_height * target_flow_height_factor
                        - EPSILON;
                    let mut layers_sparse_infill: Vec<ExPolygon> = Vec::new();
                    let mut not_sparse_infill: Vec<ExPolygon> = Vec::new();
                    let mut i = lidx as isize - 1;
                    while i >= 0 {
                        let li = i as usize;
                        if layers[li].print_z < bottom_z && li < lidx - 1 {
                            break;
                        }
                        for region in layers[li].regions() {
                            let has_low_density = region.region().config().fill_density < 1.0;
                            for surface in &region.fill_surfaces.surfaces {
                                if (surface.surface_type == SurfaceType::Internal && has_low_density)
                                    || surface.surface_type == SurfaceType::InternalVoid
                                {
                                    layers_sparse_infill.push(surface.expolygon.clone());
                                } else {
                                    not_sparse_infill.push(surface.expolygon.clone());
                                }
                            }
                        }
                        i -= 1;
                    }
                    if clib {
                        // PrintObject.cpp:3073-3077: union_ex, closing_ex over the
                        // raw polygons, and diff() with raw paths out.
                        let close = |exs: &[ExPolygon]| -> Vec<Polygon> {
                            let u = crate::clipper_utils::union_ex_clib(&ex_to_polygons(exs), 1);
                            let grown = crate::clipper_utils::offset_polygons_clib(
                                &ex_to_polygons(&u),
                                SCALED_EPSILON as f32 as f64,
                                crate::clipper_utils::OffsetJoinType::Miter,
                            );
                            ex_to_polygons(&crate::clipper_utils::offset_ex_polygons_clib(
                                &grown,
                                -(SCALED_EPSILON as f32 as f64),
                                crate::clipper_utils::OffsetJoinType::Miter,
                            ))
                        };
                        crate::clipper_utils::diff_polygons_raw_clib(
                            &close(&layers_sparse_infill),
                            &close(&not_sparse_infill),
                        )
                    } else {
                    let sparse = crate::clipper_utils::union_ex(&layers_sparse_infill);
                    let sparse = crate::clipper_utils::closing(
                        &sparse,
                        crate::unscale(SCALED_EPSILON as i64),
                        crate::clipper_utils::OffsetJoinType::Miter,
                    );
                    let not_sparse = crate::clipper_utils::union_ex(&not_sparse_infill);
                    let not_sparse = crate::clipper_utils::closing(
                        &not_sparse,
                        crate::unscale(SCALED_EPSILON as i64),
                        crate::clipper_utils::OffsetJoinType::Miter,
                    );
                    crate::geometry::to_polygons(&crate::clipper_utils::difference(&sparse, &not_sparse))
                    }
                };

                // PrintObject.cpp:2799-2820 — subtract lower-layer fills (this cluster).
                {
                    let bottom_z = print_z - target_flow_height - EPSILON;
                    let mut filled_on_lower: Vec<Polygon> = Vec::new();
                    if job_idx > 0 {
                        let mut lower_job_idx = job_idx as isize - 1;
                        while lower_job_idx >= 0 {
                            // jobs is built in cluster order, so jobs[lower_job_idx].0 ==
                            // clustered_layers_for_threads[cluster_idx][lower_job_idx] and
                            // jobs[lower_job_idx].1 holds that layer's (already-expanded)
                            // surfaces, exactly as surfaces_by_layer[&lower_layer_idx] did.
                            let lower_layer_idx = jobs[lower_job_idx as usize].0;
                            if layers[lower_layer_idx].print_z >= bottom_z {
                                for c in &jobs[lower_job_idx as usize].1 {
                                    filled_on_lower.extend(c.new_polys.iter().cloned());
                                }
                            } else {
                                break;
                            }
                            lower_job_idx -= 1;
                        }
                    }
                    deep_infill_area = diff_polygons(&deep_infill_area, &filled_on_lower);
                }
                // PrintObject.cpp:2822 — expand by 1.5*spacing.
                deep_infill_area = expand_p(&deep_infill_area, 1.5 * spacing_mm);

                // PrintObject.cpp:2824-2841 — gather expansion / total / top / lightning.
                let mut expansion_area: Vec<Polygon> = Vec::new();
                let mut total_fill_area: Vec<Polygon> = Vec::new();
                let mut top_area: Vec<Polygon> = Vec::new();
                let mut lightning_area: Vec<Polygon> = Vec::new();
                for region in layers[lidx].regions() {
                    let internal_polys = surfaces_ptr_to_polygons(&region.fill_surfaces.filter_by_types(&[SurfaceType::Internal, SurfaceType::InternalSolid]));
                    expansion_area.extend(internal_polys);
                    total_fill_area.extend(ex_to_polygons(&region.fill_expolygons));
                    let top_polys = surfaces_ptr_to_polygons(&region.fill_surfaces.filter_by_type(SurfaceType::Top));
                    top_area.extend(top_polys);
                    if region.region().config().fill_pattern == InfillPattern::Lightning {
                        let l = surfaces_ptr_to_polygons(&region.fill_surfaces.filter_by_type(SurfaceType::Internal));
                        lightning_area.extend(l);
                    }
                }
                // PrintObject.cpp:2842-2846
                total_fill_area = closing_p(&total_fill_area, crate::unscale(SCALED_EPSILON as i64));
                expansion_area = closing_p(&expansion_area, crate::unscale(SCALED_EPSILON as i64));
                expansion_area = intersect_polys(&expansion_area, &deep_infill_area);
                // anchors empty (infill_lines empty); kept for fidelity.
                let anchors: Vec<Polyline> = infill_lines
                    .get(&(lidx - 1))
                    .map(|lines| {
                        if clib {
                            crate::clipper_utils::intersection_pl(
                                lines,
                                &crate::clipper_utils::polygons_as_paths(&shrink_p(
                                    &expansion_area,
                                    spacing_mm,
                                )),
                            )
                        } else {
                        intersection_pl(
                            lines,
                            &union_polygons_ex(&shrink_p(&expansion_area, spacing_mm)),
                        )
                        }
                    })
                    .unwrap_or_default();
                let internal_unsupported_area = shrink_p(&deep_infill_area, spacing_mm * 4.5);
                boi_pp("deep2", lidx, &deep_infill_area, spacing_scaled as f64);
                boi_pp("exp", lidx, &expansion_area, 0.0);
                boi_pp("tot", lidx, &total_fill_area, 0.0);
                boi_pp("iua", lidx, &internal_unsupported_area, 0.0);

                // PrintObject.cpp:2853-2937 — per-candidate expansion.
                let cands_snapshot = jobs[job_idx].1.clone();
                let mut expanded_surfaces: Vec<CandidateSurface> =
                    Vec::with_capacity(cands_snapshot.len());
                for candidate in &cands_snapshot {
                    let flow = {
                        let r = &layers[lidx].regions()[candidate.region_idx];
                        r.bridging_flow(FlowRole::SolidInfill, true, layer_height)?
                    };
                    let flow_scaled_spacing = flow.scaled_spacing();
                    let flow_spacing_mm = if clib {
                        flow_scaled_spacing as f64 / crate::SCALING_FACTOR
                    } else {
                        flow.spacing()
                    };
                    let flow_scaled_width = flow.scaled_width();

                    // PrintObject.cpp:2857-2866 — area_to_be_bridge.
                    let mut area_to_be_bridge = expand_p(&candidate.new_polys, flow_spacing_mm);
                    area_to_be_bridge = intersect_polys(&area_to_be_bridge, &deep_infill_area);
                    let mut area_ex = if clib {
                        crate::clipper_utils::union_ex_clib(&area_to_be_bridge, 1)
                    } else {
                        crate::clipper_utils::union_ex(&union_polygons_ex(&area_to_be_bridge))
                    };
                    area_ex.retain(|p| {
                        !intersection_ex_expolygons_polygons(
                            std::slice::from_ref(p),
                            &internal_unsupported_area,
                        )
                        .is_empty()
                    });
                    area_to_be_bridge = ex_to_polygons(&area_ex);

                    // PrintObject.cpp:2868
                    let limiting_area = union2(&area_to_be_bridge, &expansion_area);

                    // PrintObject.cpp:2870-2871
                    if area_to_be_bridge.is_empty() {
                        continue;
                    }

                    // PrintObject.cpp:2873-2877 — boundary polylines.
                    // to_polylines(const Polygons &): every ring as a closed
                    // polyline. The second expand is by `0.3 * flow.spacing()`,
                    // the unscaled spacing taken as a scaled delta (0.1 units).
                    let closed_plines = |polys: &[Polygon]| -> Vec<Polyline> {
                        polys
                            .iter()
                            .map(|p| {
                                let mut pts = p.points().to_vec();
                                if let Some(f) = pts.first().copied() {
                                    pts.push(f);
                                }
                                Polyline::from_points(pts)
                            })
                            .collect()
                    };
                    let mut boundary_plines: Vec<Polyline> = if clib {
                        closed_plines(&expand_p(&total_fill_area, 1.3 * flow_spacing_mm))
                    } else {
                        ex_to_polylines(&union_polygons_ex(&expand_p(
                            &total_fill_area,
                            1.3 * flow_spacing_mm,
                        )))
                    };
                    {
                        let limiting_plines = if clib {
                            closed_plines(&crate::clipper_utils::offset_polygons_clib(
                                &limiting_area,
                                (0.3 * flow.spacing()) as f32 as f64,
                                crate::clipper_utils::OffsetJoinType::Miter,
                            ))
                        } else {
                            ex_to_polylines(&union_polygons_ex(&expand_p(
                                &limiting_area,
                                0.3 * flow_spacing_mm,
                            )))
                        };
                        boundary_plines.extend(limiting_plines);
                    }

                    // PrintObject.cpp:2886-2895 — bridging angle.
                    let region_cfg_pattern;
                    let region_cfg_dir;
                    {
                        let r = &layers[lidx].regions()[candidate.region_idx];
                        region_cfg_pattern = r.region().config().fill_pattern;
                        region_cfg_dir = r.region().config().fill_angle;
                    }
                    let mut bridging_angle = if !anchors.is_empty() {
                        determine_bridging_angle(
                            &area_to_be_bridge,
                            &to_lines_polylines(&anchors),
                            region_cfg_pattern,
                            region_cfg_dir,
                        )
                    } else {
                        determine_bridging_angle(
                            &area_to_be_bridge,
                            &to_lines_polylines(&boundary_plines),
                            InfillPattern::Line,
                            0.0,
                        )
                    };

                    boi_pp("a2b", lidx, &area_to_be_bridge, bridging_angle);
                    boi_pl("anchors", lidx, &anchors);
                    boi_pl("bplines", lidx, &boundary_plines);
                    // PrintObject.cpp:2897-2900
                    boundary_plines.extend(anchors.iter().cloned());
                    if !lightning_area.is_empty()
                        && !intersect_polys(&area_to_be_bridge, &lightning_area).is_empty()
                    {
                        boundary_plines = intersection_pl(
                            &boundary_plines,
                            &union_polygons_ex(&expand_p(&area_to_be_bridge, 10.0)),
                        );
                    }
                    // PrintObject.cpp:2901
                    let mut bridging_area = construct_anchored_polygon(
                        &area_to_be_bridge,
                        &to_lines_polylines(&boundary_plines),
                        flow_scaled_spacing,
                        flow_scaled_width,
                        bridging_angle,
                    );

                    // PrintObject.cpp:2903-2917 — collision check with other expanded.
                    {
                        let mut reconstruct = false;
                        let tmp_expanded =
                            expand_p(&bridging_area, 3.0 * flow_spacing_mm);
                        for s in &expanded_surfaces {
                            if !intersect_polys(&s.new_polys, &tmp_expanded).is_empty() {
                                bridging_angle = s.bridge_angle;
                                reconstruct = true;
                                break;
                            }
                        }
                        if reconstruct {
                            bridging_area = construct_anchored_polygon(
                                &area_to_be_bridge,
                                &to_lines_polylines(&boundary_plines),
                                flow_scaled_spacing,
                                flow_scaled_width,
                                bridging_angle,
                            );
                        }
                    }

                    boi_pp("constr", lidx, &bridging_area, bridging_angle);
                    // PrintObject.cpp:2919-2928 — clean & clip.
                    bridging_area = opening_p(&bridging_area, flow_spacing_mm);
                    bridging_area = closing_p(&bridging_area, flow_spacing_mm);
                    bridging_area = intersect_polys(&bridging_area, &limiting_area);
                    bridging_area = intersect_polys(&bridging_area, &total_fill_area);
                    bridging_area = diff_polygons(&bridging_area, &top_area);
                    bridging_area = opening_p(&bridging_area, flow_spacing_mm);
                    bridging_area = closing_p(&bridging_area, flow_spacing_mm);
                    boi_pp("final", lidx, &bridging_area, bridging_angle);
                    expansion_area = diff_polygons(&expansion_area, &bridging_area);

                    // PrintObject.cpp:2935
                    expanded_surfaces.push(CandidateSurface {
                        original_solid_idx: candidate.original_solid_idx,
                        region_idx: candidate.region_idx,
                        layer_index: candidate.layer_index,
                        new_polys: bridging_area,
                        bridge_angle: bridging_angle,
                    });
                }
                // PrintObject.cpp:2938 — store expanded polys back into this
                // cluster's private job (reinserted into surfaces_by_layer below).
                jobs[job_idx].1 = expanded_surfaces;
            }
            Ok(())
            })?;

        // Reinsert every cluster's (now expanded) candidate entries back into the
        // shared map so the Apply loop (PrintObject.cpp:2946+) is untouched; the
        // per-index writes above make the parallel result order-independent.
        for jobs in cluster_jobs {
            for (lidx, cands) in jobs {
                surfaces_by_layer.insert(lidx, cands);
            }
        }

        // ====================================================================
        let __b3 = std::time::Instant::now();
        // Apply loop — PrintObject.cpp:2946-3021.
        // ====================================================================
        // R397: the apply step is independent per layer (reads its own layer +
        // the lidx+1 solid-infill spacing + surfaces_by_layer, writes only its
        // own layer). Snapshot per-(layer,region) solid-infill spacing (the only
        // fallible/neighbour read), compute the new surfaces in parallel, then
        // apply the remove_types+append serially in order — byte-identical.
        let solid_spacings: Vec<Vec<f64>> = {
            let mut v: Vec<Vec<f64>> = Vec::with_capacity(n_layers);
            for l in 0..n_layers {
                let h = self.layers[l].height;
                let mut row = Vec::with_capacity(self.layers[l].regions().len());
                for r in 0..self.layers[l].regions().len() {
                    // flow(frSolidInfill).scaled_spacing(), the truncated coord_t.
                    let f = self.layers[l].regions()[r].flow(FlowRole::SolidInfill, h)?;
                    row.push(if clib {
                        f.scaled_spacing() as f64 / crate::SCALING_FACTOR
                    } else {
                        f.spacing()
                    });
                }
                v.push(row);
            }
            v
        };
        let layers_ref = &self.layers;
        let sbl = &surfaces_by_layer;
        let apply_results: Vec<Option<Vec<(usize, Surfaces)>>> = {
            use rayon::prelude::*;
            (0..n_layers)
                .into_par_iter()
                .map(|lidx| -> Option<Vec<(usize, Surfaces)>> {
            // PrintObject.cpp:2949
            let has_this = sbl.contains_key(&lidx);
            let has_next = sbl.contains_key(&(lidx + 1));
            if !has_this && !has_next {
                return None;
            }
            let mut region_results: Vec<(usize, Surfaces)> = Vec::new();

            // PrintObject.cpp:2953-2958 — cut_from_infill.
            let mut cut_from_infill: Vec<Polygon> = Vec::new();
            if has_this {
                for surface in &sbl[&lidx] {
                    cut_from_infill.extend(surface.new_polys.iter().cloned());
                }
            }

            // PrintObject.cpp:2960-2967 — additional_ensuring_areas.
            let mut additional_ensuring_areas: Vec<Polygon> = Vec::new();
            if has_next {
                let next_cands = sbl[&(lidx + 1)].clone();
                for surface in &next_cands {
                    let next_region_spacing_mm = solid_spacings[lidx + 1][surface.region_idx];
                    let additional_area = diff_polygons(
                        &surface.new_polys,
                        &shrink_p(&surface.new_polys, next_region_spacing_mm),
                    );
                    additional_ensuring_areas.extend(additional_area);
                }
            }

            // PrintObject.cpp:2969-3019 — per region rebuild surfaces.
            let n_regions = layers_ref[lidx].regions().len();
            for region_idx in 0..n_regions {
                let region_spacing_mm = solid_spacings[lidx][region_idx];
                // PrintObject.cpp:2972-2974 — near_perimeters / additional_ensuring.
                let (all_surface_polys, fill_expolys, internal_exs, internal_solid_exs, solid_indices) = {
                    let r = &layers_ref[lidx].regions()[region_idx];
                    let all_polys = surfaces_ptr_to_polygons(&r.fill_surfaces.surfaces.iter().collect::<Vec<_>>());
                    let fe = r.fill_expolygons.clone();
                    let int_exs: Vec<ExPolygon> = r
                        .fill_surfaces
                        .surfaces
                        .iter()
                        .filter(|s| s.surface_type == SurfaceType::Internal)
                        .map(|s| s.expolygon.clone())
                        .collect();
                    let solid_idx_ex: Vec<(usize, ExPolygon)> = r
                        .fill_surfaces
                        .surfaces
                        .iter()
                        .enumerate()
                        .filter(|(_, s)| s.surface_type == SurfaceType::InternalSolid)
                        .map(|(i, s)| (i, s.expolygon.clone()))
                        .collect();
                    let solid_exs: Vec<ExPolygon> =
                        solid_idx_ex.iter().map(|(_, e)| e.clone()).collect();
                    let solid_idx: Vec<usize> = solid_idx_ex.iter().map(|(i, _)| *i).collect();
                    (all_polys, fe, int_exs, solid_exs, solid_idx)
                };

                let mut near_perimeters =
                    crate::geometry::to_polygons(&union_safety_offset_ex(&all_surface_polys));
                near_perimeters = diff_polygons(
                    &near_perimeters,
                    &shrink_p(&near_perimeters, region_spacing_mm),
                );
                let additional_ensuring = intersection_ex_polygons_polygons(
                    &additional_ensuring_areas,
                    &near_perimeters,
                    ApplySafetyOffset::No,
                );

                let mut new_surfaces: Surfaces = Vec::new();

                // PrintObject.cpp:2976-2981 — new internal infills.
                // PrintObject.cpp:3560-3562 — diff_ex(SurfacesPtr, Polygons), then
                // diff_ex(ExPolygons, ExPolygons).
                let mut new_internal_infills = if clib {
                    crate::clipper_utils::diff_ex_polygons_polygons(
                        &ex_to_polygons(&internal_exs),
                        &cut_from_infill,
                        ApplySafetyOffset::No,
                    )
                } else {
                    crate::clipper_utils::difference(&internal_exs, &union_polygons_ex(&cut_from_infill))
                };
                new_internal_infills = if clib {
                    crate::clipper_utils::difference_clib(&new_internal_infills, &additional_ensuring)
                } else {
                    crate::clipper_utils::difference(&new_internal_infills, &additional_ensuring)
                };
                for ep in new_internal_infills {
                    new_surfaces.push(Surface::new(SurfaceType::Internal, ep));
                }

                // PrintObject.cpp:2983-2998 — mark bridges from matching solids.
                if has_this {
                    for cs in &sbl[&lidx] {
                        if cs.region_idx != region_idx {
                            continue;
                        }
                        // match by original solid index against current solids.
                        if !solid_indices.contains(&cs.original_solid_idx) {
                            continue;
                        }
                        // intersection_ex(union_ex(cs.new_polys), region->fill_expolygons)
                        let clipped = if clib {
                            crate::clipper_utils::intersection_clib(
                                &crate::clipper_utils::union_ex_clib(&cs.new_polys, 1),
                                &fill_expolys,
                            )
                        } else {
                            let cs_union = union_polygons_ex(&cs.new_polys);
                            crate::clipper_utils::intersection(&cs_union, &fill_expolys)
                        };
                        for ep in clipped {
                            let mut s = Surface::new(SurfaceType::InternalBridge, ep);
                            s.bridge_angle = Some(cs.bridge_angle);
                            new_surfaces.push(s);
                        }
                    }
                }

                // PrintObject.cpp:3000-3006 — new internal solids.
                let mut new_internal_solids = internal_solid_exs.clone();
                new_internal_solids.extend(additional_ensuring.iter().cloned());
                let mut new_internal_solids = if clib {
                    crate::clipper_utils::diff_ex_polygons_polygons(
                        &ex_to_polygons(&new_internal_solids),
                        &cut_from_infill,
                        ApplySafetyOffset::No,
                    )
                } else {
                    crate::clipper_utils::difference(&new_internal_solids, &union_polygons_ex(&cut_from_infill))
                };
                new_internal_solids = union_safety_offset_ex_expolygons(&new_internal_solids);
                for ep in new_internal_solids {
                    new_surfaces.push(Surface::new(SurfaceType::InternalSolid, ep));
                }

                // PrintObject.cpp:3017-3018 — deferred to the serial apply below.
                region_results.push((region_idx, new_surfaces));
            }
            Some(region_results)
                })
                .collect()
        };

        // Serial apply (preserves lidx→region_idx order; cheap mutation only).
        for (lidx, out) in apply_results.into_iter().enumerate() {
            if let Some(region_results) = out {
                for (region_idx, new_surfaces) in region_results {
                    let region = self.layers[lidx].get_region_mut(region_idx).unwrap();
                    region
                        .fill_surfaces
                        .remove_types(&[SurfaceType::InternalSolid, SurfaceType::Internal]);
                    region.fill_surfaces.append_surfaces(new_surfaces);
                }
            }
        }

        if __bt {
            let __b4 = std::time::Instant::now();
            eprintln!(
                "      bridge_over_infill sub (s): candidate_extract {:.2}  anchor+cluster {:.2}  main_expand {:.2}  apply {:.2}",
                (__b1 - __b0).as_secs_f64(),
                (__b2 - __b1).as_secs_f64(),
                (__b3 - __b2).as_secs_f64(),
                (__b4 - __b3).as_secs_f64(),
            );
        }

        Ok(())
    }

    /// Detect surface types (top/bottom/internal) by comparing layers
    /// PrintObject.cpp:1447-1695
    /// C++: void PrintObject::detect_surfaces_type(std::vector<std::vector<SurfaceCollection>> &slice_surfaces_cpy)
    fn detect_surfaces_type(
        &mut self,
        slice_surfaces_cpy: &mut Vec<Vec<SurfaceCollection>>,
    ) -> Result<()> {
        use crate::clipper_utils::{
            diff_ex, diff_ex_polygons_surfaces, diff_ex_surfaces_expolygons,
            intersection_surfaces_expolygons, opening_ex, to_polygons, ApplySafetyOffset,
        };
        use crate::surface::{surfaces_append, Surface, SurfaceType, Surfaces};

        /// PrintObject.cpp:1449
        /// C++: BOOST_LOG_TRIVIAL(info) << "Detecting solid surfaces..." << log_memory_info();

        /// PrintObject.cpp:1451-1457
        /// C++: bool spiral_mode = this->print()->config().spiral_mode.value;
        /// C++: bool interface_shells = ! spiral_mode && m_config.interface_shells.value;
        /// C++: size_t num_layers = spiral_mode ? std::min(size_t(this->printing_region(0).config().bottom_shell_layers), m_layers.size()) : m_layers.size();
        let spiral_mode = self.config.spiral_vase;
        let interface_shells = !spiral_mode && self.config.interface_shells;

        let bottom_shell_layers = self
            .shared_regions
            .as_ref()
            .and_then(|r| r.all_regions.first())
            .map(|r| r.config().bottom_solid_layers as usize)
            .unwrap_or(2);
        let num_layers = if spiral_mode {
            std::cmp::min(bottom_shell_layers, self.layers.len())
        } else {
            self.layers.len()
        };

        /// PrintObject.cpp:1459
        for region_id in 0..self.num_printing_regions() {
            /// PrintObject.cpp:1460
            /// C++: BOOST_LOG_TRIVIAL(debug) << "Detecting solid surfaces for region " << region_id << " in parallel - start";
            // C++: const PrintRegionConfig &region_config = layerm->region().config();
            // (identical for every layer of a region). Cloning the Arc (not the
            // config) keeps the shared snapshot alive across the mutable layer
            // accesses below.
            let region = self
                .shared_regions
                .as_ref()
                .and_then(|r| r.all_regions.get(region_id))
                .cloned()
                .expect("shared_regions covers all printing regions");
            let region_config = region.config();

            /// PrintObject.cpp:1466-1469
            /// C++: std::vector<Surfaces> surfaces_new;
            /// C++: if (interface_shells)
            /// C++:     surfaces_new.assign(num_layers, Surfaces());
            let mut surfaces_new: Vec<Surfaces> = if interface_shells {
                vec![Surfaces::new(); num_layers]
            } else {
                Vec::new()
            };

            /// PrintObject.cpp:1471
            /// C++: slice_surfaces_cpy.resize(m_layers.size());
            slice_surfaces_cpy.resize(self.layers.len(), Vec::new());

            /// PrintObject.cpp:1473-1597
            /// C++: tbb::parallel_for(tbb::blocked_range<size_t>(0, ...), [&](const tbb::blocked_range<size_t> &range) {
            // rayon two-phase (R379 shape): C++ runs the surface-typing body under
            // tbb::parallel_for, reading the neighbour upper/lower layers through
            // pointers while it rebuilds the current layer's region slices. The
            // borrow checker wants the split into (1) a parallel READ pass over
            // &self.layers computing each layer's top/bottom/internal surface sets
            // (all the heavy diff_ex/opening_ex geometry), then (2) the apply pass
            // writing each layer's own region (or its surfaces_new slot). The
            // neighbour reads — lslices, and in interface_shells mode the sibling
            // region slices — are never mutated by this pass, so the layers are
            // independent exactly as the C++ parallel_for assumes.
            let range_end = if spiral_mode && num_layers > 1 {
                num_layers - 1
            } else {
                self.layers.len()
            };

            /// PrintObject.cpp:1483-1485 — slice_surfaces_cpy[idx_layer] is resized to
            /// the layer's region count (order-independent, reads no neighbours);
            /// hoisted out of the parallel body so phase 1 can borrow &self.layers.
            /// PrintObject.cpp:1486-1490 — TODO: Port infill_instead_top_bottom_surfaces
            /// (ipLockedZag) copy into slice_surfaces_cpy[idx_layer][region_id].
            for idx_layer in 0..range_end {
                slice_surfaces_cpy[idx_layer].resize(
                    self.layers[idx_layer].regions().len(),
                    SurfaceCollection::new(),
                );
            }

            // Phase 1 — parallel READ pass computing (top, bottom, internal) per layer.
            use rayon::prelude::*;
            let canceled = self.canceled.clone();
            let layers = &self.layers;
            let layer_typings: Vec<(Surfaces, Surfaces, Vec<crate::ExPolygon>)> = (0..range_end)
                .into_par_iter()
                .map(|idx_layer| -> Result<(Surfaces, Surfaces, Vec<crate::ExPolygon>)> {
                /// PrintObject.cpp:1481
                if canceled.load(std::sync::atomic::Ordering::Relaxed) {
                    return Err(crate::Error::Cancelled);
                }

                /// PrintObject.cpp:1491-1494
                /// C++: Layer *upper_layer = (idx_layer + 1 < this->layer_count()) ? m_layers[idx_layer + 1] : nullptr;
                /// C++: Layer *lower_layer = (idx_layer > 0) ? m_layers[idx_layer - 1] : nullptr;
                let has_upper_layer = idx_layer + 1 < layers.len();
                let has_lower_layer = idx_layer > 0;

                /// PrintObject.cpp:1495-1496
                /// C++: float offset = layerm->flow(frExternalPerimeter).scaled_width() / 10.f;
                // This crate's clipper primitives (opening_ex/shrink/grow) operate in
                // UNSCALED (mm) space — they `unscale()` the polygon coords and pass the
                // delta through verbatim — so the faithful equivalent of C++
                // `scaled_width()/10` is `width()/10` (mm).
                let layer_height = layers[idx_layer].height;
                let offset = {
                    let w = layers[idx_layer].regions()[region_id]
                        .flow(crate::flow::FlowRole::ExternalPerimeter, layer_height)?
                        .width();
                    if crate::faithful_gate("TOPFILL_FAITHFUL") {
                        // R283: native = float(scaled_width())/10.f — the coord_t
                        // TRUNCATION comes first (41999 → 4199.8999f), then f32
                        // division; width()/10 skips the trunc (4199.9999) — a
                        // 0.1-unit delta error on every detect opening_ex.
                        ((((w / 0.00001).trunc() as f32) / 10.0f32) as f64) / crate::SCALING_FACTOR
                    } else {
                        w / 10.0
                    }
                };

                /// PrintObject.cpp:1498-1499
                /// C++: bool detect_top = spiral_mode || layerm->region().config().top_shell_layers;
                /// C++: bool detect_bottom = spiral_mode || layerm->region().config().bottom_shell_layers;
                let detect_top = spiral_mode || region_config.top_solid_layers > 0;
                let detect_bottom = spiral_mode || region_config.bottom_solid_layers > 0;

                if crate::probe_enabled("LSDBG")
                    && region_id == 0
                                    {
                    let ls = &layers[idx_layer].lslices;
                    let mut a = 0.0f64;
                    let mut ha = 0.0f64;
                    let mut nh = 0usize;
                    for e in ls {
                        a += e.contour.area().abs() / 1e10;
                        nh += e.holes.len();
                        for h in &e.holes {
                            ha += h.area().abs() / 1e10;
                        }
                    }
                    let mut rnh = 0usize;
                    let mut rha = 0.0f64;
                    for reg in layers[idx_layer].regions() {
                        for sf in &reg.slices.surfaces {
                            rnh += sf.expolygon.holes.len();
                            for h in &sf.expolygon.holes {
                                rha += h.area().abs() / 1e10;
                            }
                        }
                    }
                    eprintln!(
                        "LSDBG-RGN z={:.2} rnholes={} rholes={:.4}",
                        layers[idx_layer].print_z, rnh, rha
                    );
                    eprintln!(
                        "LSDBG-R z={:.2} n={} outer={:.4} nholes={} holes={:.4}",
                        layers[idx_layer].print_z,
                        ls.len(),
                        a,
                        nh,
                        ha
                    );
                }

                /// PrintObject.cpp:1501-1520
                /// C++: Surfaces top;
                /// C++: if (detect_top) { ... }
                let mut top = Surfaces::new();
                if detect_top {
                    if has_upper_layer {
                        /// PrintObject.cpp:1503-1506
                        /// C++: ExPolygons upper_slices = interface_shells ?
                        /// C++:     diff_ex(layerm->slices.surfaces, upper_layer->m_regions[region_id]->slices.surfaces, ApplySafetyOffset::Yes) :
                        /// C++:     diff_ex(layerm->slices.surfaces, upper_layer->lslices, ApplySafetyOffset::Yes);
                        let current_slices =
                            &layers[idx_layer].regions()[region_id].slices.surfaces;
                        let upper_diff = if interface_shells {
                            let upper_slices = &layers[idx_layer + 1].regions()[region_id]
                                .slices
                                .surfaces;
                            diff_ex(current_slices, upper_slices, ApplySafetyOffset::Yes)
                        } else {
                            let upper_lslices = &layers[idx_layer + 1].lslices;
                            diff_ex_surfaces_expolygons(
                                current_slices,
                                upper_lslices,
                                ApplySafetyOffset::Yes,
                            )
                        };

                        /// PrintObject.cpp:1507
                        /// C++: surfaces_append(top, opening_ex(upper_slices, offset), stTop);
                        if std::env::var("TSDBG").is_ok()
                            && (layers[idx_layer].print_z - 4.80).abs() < 0.001
                        {
                            let sc2 = crate::SCALING_FACTOR * crate::SCALING_FACTOR;
                            let mut tiny = 0usize;
                            let mut atot = 0.0f64;
                            for sf in &layers[idx_layer].regions()[region_id].slices.surfaces {
                                let a = sf.expolygon.area().abs() / sc2;
                                atot += a;
                                if a < 0.05 { tiny += 1; }
                            }
                            eprintln!(
                                "TSDBG-R reg={} nsurf={} tiny={} area={:.4} nregions={}",
                                region_id,
                                layers[idx_layer].regions()[region_id].slices.surfaces.len(),
                                tiny, atot, layers[idx_layer].regions().len()
                            );
                        }
                        if std::env::var("TSDBG_FULL").is_ok()
                            && (layers[idx_layer].print_z - 4.80).abs() < 0.001
                        {
                            let sc = crate::SCALING_FACTOR * crate::SCALING_FACTOR;
                            for sf in &layers[idx_layer].regions()[region_id].slices.surfaces {
                                eprintln!("TSDBG-R in_slice npts={} nholes={} a={:.4}", sf.expolygon.contour.points.len(), sf.expolygon.holes.len(), sf.expolygon.area().abs()/sc);
                            }
                            for e in &layers[idx_layer + 1].lslices {
                                eprintln!("TSDBG-R in_upper npts={} nholes={} a={:.4}", e.contour.points.len(), e.holes.len(), e.area().abs()/sc);
                            }
                            for e in &upper_diff {
                                eprintln!("TSDBG-R post_diff npts={} nholes={} a={:.4}", e.contour.points.len(), e.holes.len(), e.area().abs()/sc);
                            }
                            for e in &opening_ex(&upper_diff, offset) {
                                eprintln!("TSDBG-R post_open npts={} nholes={} a={:.4} (offset={:.3})", e.contour.points.len(), e.holes.len(), e.area().abs()/sc, offset);
                            }
                        }
                        surfaces_append(
                            &mut top,
                            opening_ex(&upper_diff, offset),
                            SurfaceType::Top,
                        );
                    } else {
                        // PrintObject.cpp:1509-1515
                        // C++: else {
                        // C++:     top = layerm->slices.surfaces;
                        // C++:     for (Surface& surface : top)
                        // C++:         surface.surface_type = stTop;
                        // C++: }
                        top = layers[idx_layer].regions()[region_id]
                            .slices
                            .surfaces
                            .clone();
                        for surface in &mut top {
                            surface.surface_type = SurfaceType::Top;
                        }
                    }
                }

                /// PrintObject.cpp:1522-1567
                /// C++: Surfaces bottom;
                /// C++: if (detect_bottom) { ... }
                let mut bottom = Surfaces::new();
                if detect_bottom {
                    if has_lower_layer {
                        /// PrintObject.cpp:1531-1546
                        /// C++: surfaces_append(
                        /// C++:     bottom,
                        /// C++:     opening_ex(
                        /// C++:         diff_ex(layerm->slices.surfaces, lower_layer->lslices, ApplySafetyOffset::Yes),
                        /// C++:         offset),
                        /// C++:     surface_type_bottom_other);
                        let current_slices =
                            &layers[idx_layer].regions()[region_id].slices.surfaces;
                        let lower_lslices = &layers[idx_layer - 1].lslices;
                        let bottom_diff = diff_ex_surfaces_expolygons(
                            current_slices,
                            lower_lslices,
                            ApplySafetyOffset::Yes,
                        );
                        surfaces_append(
                            &mut bottom,
                            opening_ex(&bottom_diff, offset),
                            SurfaceType::BottomBridge,
                        );
                        if crate::probe_enabled("BOTPROBE") && idx_layer == 2 {
                            let (mut six, mut siy, mut sn) = (0i64, 0i64, 0i64);
                            for sf in current_slices.iter() {
                                sn += sf.expolygon.contour.points.len() as i64;
                                for pt in &sf.expolygon.contour.points {
                                    six += pt.x();
                                    siy += pt.y();
                                }
                            }
                            let (mut lix, mut liy, mut ln) = (0i64, 0i64, 0i64);
                            for e in lower_lslices.iter() {
                                ln += e.contour.points.len() as i64;
                                for pt in &e.contour.points {
                                    lix += pt.x();
                                    liy += pt.y();
                                }
                            }
                            eprintln!(
                                "BOTPROBE in slices={}:{},{} lsl={}:{},{} off={:.4}",
                                sn,
                                six,
                                siy,
                                ln,
                                lix,
                                liy,
                                offset * crate::SCALING_FACTOR
                            );
                            for sf in bottom.iter() {
                                let ix: i64 =
                                    sf.expolygon.contour.points.iter().map(|p| p.x()).sum();
                                let iy: i64 =
                                    sf.expolygon.contour.points.iter().map(|p| p.y()).sum();
                                eprintln!(
                                    "BOTPROBE out n={} ix={} iy={} a={:.1}",
                                    sf.expolygon.contour.points.len(),
                                    ix,
                                    iy,
                                    sf.expolygon.area().abs()
                                );
                            }
                        }

                        /// PrintObject.cpp:1547-1559
                        /// C++: if (interface_shells) {
                        /// C++:     surfaces_append(
                        /// C++:         bottom,
                        /// C++:         opening_ex(
                        /// C++:             diff_ex(
                        /// C++:                 intersection(layerm->slices.surfaces, lower_layer->lslices),
                        /// C++:                 lower_layer->m_regions[region_id]->slices.surfaces,
                        /// C++:                 ApplySafetyOffset::Yes),
                        /// C++:             offset),
                        /// C++:         stBottom);
                        /// C++: }
                        if interface_shells {
                            let intersection_result =
                                intersection_surfaces_expolygons(current_slices, lower_lslices);
                            let intersection_surfaces_vec: Vec<Surface> = intersection_result
                                .into_iter()
                                .map(|ex| Surface::new(SurfaceType::Bottom, ex))
                                .collect();
                            let lower_region_slices = &layers[idx_layer - 1].regions()
                                [region_id]
                                .slices
                                .surfaces;
                            let interface_diff = diff_ex(
                                &intersection_surfaces_vec,
                                lower_region_slices,
                                ApplySafetyOffset::Yes,
                            );

                            surfaces_append(
                                &mut bottom,
                                opening_ex(&interface_diff, offset),
                                SurfaceType::Bottom,
                            );
                        }
                    } else {
                        // PrintObject.cpp:1562-1567
                        // C++: else {
                        // C++:     bottom = layerm->slices.surfaces;
                        // C++:     for (Surface& surface : bottom)
                        // C++:         surface.surface_type = stBottom;
                        // C++: }
                        bottom = layers[idx_layer].regions()[region_id]
                            .slices
                            .surfaces
                            .clone();
                        for surface in &mut bottom {
                            surface.surface_type = SurfaceType::Bottom;
                        }
                    }
                }

                /// PrintObject.cpp:1569-1575
                /// C++: if (! top.empty() && ! bottom.empty()) {
                /// C++:     Polygons top_polygons = to_polygons(std::move(top));
                /// C++:     top.clear();
                /// C++:     surfaces_append(top, diff_ex(top_polygons, bottom), stTop);
                /// C++: }
                if !top.is_empty() && !bottom.is_empty() {
                    let top_polygons = to_polygons(&top);
                    top.clear();
                    surfaces_append(
                        &mut top,
                        diff_ex_polygons_surfaces(&top_polygons, &bottom, ApplySafetyOffset::No),
                        SurfaceType::Top,
                    );
                }

                if crate::stage_dump::stagedump_key() == Some(idx_layer) {
                    let eps = |ss: &[crate::surface::Surface]| -> Vec<crate::geometry::ExPolygon> {
                        ss.iter().map(|s| s.expolygon.clone()).collect()
                    };
                    crate::stage_dump::dump("detect_top", idx_layer, &eps(&top));
                    crate::stage_dump::dump("detect_bottom", idx_layer, &eps(&bottom));
                }

                /// PrintObject.cpp:1584-1591 — surfaces_prev is the layer's ORIGINAL
                /// region slices. C++ selects it via `interface_shells ? layerm->slices
                /// : surfaces_backup`, where surfaces_backup is the pre-clear copy in the
                /// non-interface branch; phase 1 only reads, so clone it here and let the
                /// apply pass below perform the clear + rebuild.
                let surfaces_prev = layers[idx_layer].regions()[region_id]
                    .slices
                    .surfaces
                    .clone();

                /// PrintObject.cpp:1593-1597
                /// C++: {
                /// C++:     Polygons topbottom = to_polygons(top);
                /// C++:     polygons_append(topbottom, to_polygons(bottom));
                /// C++:     surfaces_append(surfaces_out, diff_ex(surfaces_prev, topbottom), stInternal);
                /// C++: }
                // C++ builds `topbottom = to_polygons(top) ++ to_polygons(bottom)` then
                // `diff_ex(surfaces_prev, topbottom)`. We keep the top/bottom ExPolygon
                // structure (contour+holes) for the clip, which yields the same diff set.
                let surfaces_prev_expolygons: Vec<crate::ExPolygon> =
                    surfaces_prev.iter().map(|s| s.expolygon.clone()).collect();
                let topbottom_expolygons: Vec<crate::ExPolygon> = top
                    .iter()
                    .chain(bottom.iter())
                    .map(|s| s.expolygon.clone())
                    .collect();
                let internal_surfaces = if crate::faithful_gate("TOPFILL_FAITHFUL") {
                    crate::clipper_utils::difference_clib(
                        &surfaces_prev_expolygons,
                        &topbottom_expolygons,
                    )
                } else {
                    crate::clipper_utils::difference(
                        &surfaces_prev_expolygons,
                        &topbottom_expolygons,
                    )
                };

                Ok((top, bottom, internal_surfaces))
                })
                .collect::<Result<Vec<_>>>()?;

            // Phase 2 — apply pass writing each layer's typed surfaces back.
            // PrintObject.cpp:1584-1597: surfaces_out = interface_shells ?
            // surfaces_new[idx_layer] : layerm->slices.surfaces.
            if interface_shells {
                // surfaces_out == surfaces_new[idx_layer] (a local, per-index vec);
                // the appends are trivial, so this write-back stays sequential.
                for (idx_layer, (mut top, mut bottom, internal_surfaces)) in
                    layer_typings.into_iter().enumerate()
                {
                    surfaces_append(
                        &mut surfaces_new[idx_layer],
                        internal_surfaces,
                        SurfaceType::Internal,
                    );
                    surfaces_new[idx_layer].append(&mut top);
                    surfaces_new[idx_layer].append(&mut bottom);
                }
            } else {
                // surfaces_out == layerm->slices.surfaces — each layer clears &
                // rebuilds its OWN region, so par_iter_mut applies disjointly (rayon
                // for tbb, mirroring the C++ parallel_for's per-layer writes).
                self.layers[..range_end]
                    .par_iter_mut()
                    .zip(layer_typings)
                    .for_each(|(layer, (mut top, mut bottom, internal_surfaces))| {
                        let region = &mut layer.regions_mut()[region_id];
                        region.slices.surfaces.clear();
                        // Convert internal_surfaces (ExPolygons) to Surfaces
                        for expoly in internal_surfaces {
                            region
                                .slices
                                .surfaces
                                .push(Surface::new(SurfaceType::Internal, expoly));
                        }
                        region.slices.surfaces.append(&mut top);
                        region.slices.surfaces.append(&mut bottom);
                    });
            }

            /// PrintObject.cpp:1600
            if self.canceled.load(std::sync::atomic::Ordering::Relaxed) {
                return Err(crate::Error::Cancelled);
            }

            /// PrintObject.cpp:1602-1606
            /// C++: if (interface_shells) {
            /// C++:     for (size_t idx_layer = 0; idx_layer < num_layers; ++ idx_layer)
            /// C++:         m_layers[idx_layer]->m_regions[region_id]->slices.surfaces = std::move(surfaces_new[idx_layer]);
            /// C++: }
            if interface_shells {
                for idx_layer in 0..num_layers {
                    self.layers[idx_layer].regions_mut()[region_id]
                        .slices
                        .surfaces = surfaces_new[idx_layer].clone();
                }
            }

            /// PrintObject.cpp:1608-1615
            /// C++: if (spiral_mode) {
            /// C++:     if (num_layers > 1)
            /// C++:         m_layers[num_layers - 1]->m_regions[region_id]->slices.set_type(stTop);
            /// C++:     for (size_t i = num_layers; i < m_layers.size(); ++ i)
            /// C++:         m_layers[i]->m_regions[region_id]->slices.set_type(stInternal);
            /// C++: }
            if spiral_mode {
                if num_layers > 1 {
                    self.layers[num_layers - 1].regions_mut()[region_id]
                        .slices
                        .set_type(SurfaceType::Top);
                }
                for i in num_layers..self.layers.len() {
                    self.layers[i].regions_mut()[region_id]
                        .slices
                        .set_type(SurfaceType::Internal);
                }
            }

            /// PrintObject.cpp:1617
            /// C++: BOOST_LOG_TRIVIAL(debug) << "Detecting solid surfaces for region " << region_id << " - clipping in parallel - start";

            /// PrintObject.cpp:1618-1643
            /// C++: tbb::parallel_for(
            /// C++:     tbb::blocked_range<size_t>(0, m_layers.size()),
            /// C++:     [this, region_id](const tbb::blocked_range<size_t>& range) {
            /// C++:         for (size_t idx_layer = range.begin(); idx_layer < range.end(); ++ idx_layer) {
            /// C++:             m_print->throw_if_canceled();
            /// C++:             LayerRegion *layerm = m_layers[idx_layer]->m_regions[region_id];
            /// C++:             layerm->slices_to_fill_surfaces_clipped();
            /// C++:         }
            /// C++:     });
            // TOPDBG (diagnostics only, env-gated, not part of the C++ port):
            // Top state after detect typing, before the fill_expolygons clip.
            if crate::debug::topdbg::enabled() {
                for idx_layer in 0..self.layers.len() {
                    let region = &self.layers[idx_layer].regions()[region_id];
                    crate::debug::topdbg::log_top_surfaces(
                        idx_layer,
                        "detect_surfaces_type",
                        &region.slices.surfaces,
                    );
                    crate::debug::topdbg::dump_top_surfaces(
                        idx_layer,
                        "a_detect_top_slices",
                        &region.slices.surfaces,
                    );
                    crate::debug::topdbg::dump_expolygons(
                        idx_layer,
                        "c_fill_expolygons",
                        &region.fill_expolygons,
                    );
                }
            }
            // C++ parallel_for (PrintObject.cpp:1618-1643): each layer clips its OWN
            // region's fill surfaces (slices_to_fill_surfaces_clipped reads no
            // neighbour), so this is a directly-independent per-layer pass. rayon
            // stands in for tbb; try_for_each carries the per-layer cancellation
            // (throw_if_canceled). `canceled` is the clone made for phase 1 above.
            self.layers
                .par_iter_mut()
                .enumerate()
                .try_for_each(|(idx_layer, layer)| -> Result<()> {
                    if canceled.load(std::sync::atomic::Ordering::Relaxed) {
                        return Err(crate::Error::Cancelled);
                    }
                    layer.regions_mut()[region_id].slices_to_fill_surfaces_clipped();
                    // TOPDBG: Top state after the clip by fill_expolygons.
                    if crate::debug::topdbg::enabled() {
                        let region = &layer.regions()[region_id];
                        crate::debug::topdbg::log_top_surfaces(
                            idx_layer,
                            "slices_to_fill_surfaces_clipped",
                            &region.fill_surfaces.surfaces,
                        );
                        crate::debug::topdbg::dump_top_surfaces(
                            idx_layer,
                            "d1_clip_top",
                            &region.fill_surfaces.surfaces,
                        );
                    }
                    Ok(())
                })?;

            /// PrintObject.cpp:1644
            if self.canceled.load(std::sync::atomic::Ordering::Relaxed) {
                return Err(crate::Error::Cancelled);
            }

            // PrintObject.cpp:1645
            // C++: BOOST_LOG_TRIVIAL(debug) << "Detecting solid surfaces for region " << region_id << " - clipping in parallel - end";
        }

        // PrintObject.cpp:1648
        // C++: m_typed_slices = true;
        self.typed_slices = true;

        Ok(())
    }

    /// Generate infill by calling Layer::make_fills() on all layers
    /// PrintObject.cpp:751-780
    /// C++: tbb::parallel_for(...) { m_layers[layer_idx]->make_fills(...); }
    pub fn infill(&mut self) -> Result<()> {
        // Prerequisites - prepare infill first
        // PrintObject.cpp:754
        // SLICE_PHASE_TIMING splits prepare_infill (incl. MMS segmentation) from
        // the parallel fill loop below — the two have very different perf
        // profiles on multicolour models.
        let __timing = crate::probe_enabled("SLICE_PHASE_TIMING");
        let __t_prep = std::time::Instant::now();
        self.prepare_infill()?;
        let __prep_s = __t_prep.elapsed().as_secs_f64();
        let __t_fill = std::time::Instant::now();

        // Check if step needs to be done
        // PrintObject.cpp:756
        if !self.is_step_done(PrintObjectStep::Infill) {
            // Status update
            // PrintObject.cpp:757
            // (Status callback handled by Print::process)

            // PrintObject.cpp:1166-1223 — pin the radial scan count and the pole of
            // every conformal region before the layers are filled in parallel.
            // C++: bool   want_conformal = false;
            //      double chart_spacing  = 0.4;
            //      for (const PrintRegion &region : this->all_regions())
            //          if (region.config().conformal_infill) { ...; break; }
            let mut want_conformal = false;
            let mut chart_spacing = 0.4_f64;
            for region in self.all_regions() {
                if region.config().conformal_infill {
                    want_conformal = true;
                    if region.config().sparse_infill_line_width > 0.0 {
                        chart_spacing = region.config().sparse_infill_line_width;
                    }
                    break;
                }
            }

            if want_conformal && !self.layers.is_empty() {
                // PrintObject.cpp:1180-1186
                let lid0 = self.layers[0].id();
                let mut nlid = 0usize;
                for layer in &self.layers {
                    nlid = nlid.max(layer.id() - lid0 + 1);
                }
                let mut zs: Vec<f64> = vec![0.0; nlid];
                for layer in &self.layers {
                    zs[layer.id() - lid0] = layer.print_z;
                }

                // PrintObject.cpp:1188-1220
                crate::fill::fill_radial_zig_zag::reset_n_lock();
                let nreg = self.num_printing_regions();
                for region_id in 0..nreg {
                    let rcfg = match self.printing_region(region_id) {
                        Some(r) => r.config().clone(),
                        None => continue,
                    };
                    if !rcfg.conformal_infill {
                        continue;
                    }
                    // `0.01 * rcfg.sparse_infill_density`: the option is a percentage
                    // in the C++; this crate keeps the fraction, and group_fills
                    // takes the percentage back the same way (fill/mod.rs).
                    let density = (0.05_f64).max(0.01 * (rcfg.fill_density * 100.0));
                    let mut line_w = chart_spacing;
                    if rcfg.sparse_infill_line_width > 0.0 {
                        line_w = rcfg.sparse_infill_line_width;
                    }
                    // FIDELITY-NOTE: blocked on conformal_modifier_islands
                    // (PrintObject.cpp:1083-1154), which needs
                    // PrintObject::firstLayerObjSlice() and the volume regions of
                    // PrintObjectRegions::layer_ranges. It returns islands only for
                    // conformal MODIFIER volumes (`is_modifier()` with a parent), so
                    // for a region that is not a modifier its rows are all empty,
                    // `n_cast` is 0 and the fill surfaces below are what is pinned,
                    // as here. A conformal modifier is pinned from its fill surfaces
                    // instead of from its uncarved islands.
                    let mut islands_by_layer: Vec<Vec<crate::geometry::ExPolygon>> = vec![Vec::new(); nlid];
                    let mut n_cast = 0usize;
                    for row in &islands_by_layer {
                        if !row.is_empty() {
                            n_cast += 1;
                        }
                    }
                    if n_cast < 3 {
                        islands_by_layer = vec![Vec::new(); nlid];
                        for layer in &self.layers {
                            if region_id >= layer.regions().len() {
                                continue;
                            }
                            let lr = &layer.regions()[region_id];
                            if !lr.region().config().conformal_infill {
                                continue;
                            }
                            let lid = layer.id() - lid0;
                            for surface in &lr.fill_surfaces.surfaces {
                                if surface.surface_type == crate::surface::SurfaceType::Internal {
                                    islands_by_layer[lid].push(surface.expolygon.clone());
                                }
                            }
                        }
                    }
                    crate::fill::fill_radial_zig_zag::pin_scan_counts(
                        &islands_by_layer,
                        line_w / density,
                        &zs,
                        region_id,
                        rcfg.conformal_ray_count,
                    );
                }
            } else {
                // PrintObject.cpp:1222
                crate::fill::fill_radial_zig_zag::reset_n_lock();
            }

            // Adaptive fill octrees (currently None - TODO: implement adaptive fill)
            // PrintObject.cpp:763-770
            // Per-region configs are no longer threaded from here: group_fills
            // reads each LayerRegion's own Arc<PrintRegion> (C++:
            // layerm.region().config(), Fill.cpp:199).

            // Iterate through all layers and generate fills
            // PrintObject.cpp:763-770
            // C++: tbb::parallel_for(tbb::blocked_range<size_t>(0, m_layers.size()),
            // C++:     [this, ...](const tbb::blocked_range<size_t> &range) {
            // C++:         for (size_t layer_idx = range.begin(); layer_idx < range.end(); ++layer_idx)
            // C++:             m_layers[layer_idx]->make_fills(...);
            // rayon stands in for tbb. C++ reads the sibling lower layer through
            // pointers inside the parallel body; the borrow checker wants that
            // split into (1) a parallel READ pass over &self.layers building the
            // per-layer lower-layer snapshots, then (2) the parallel MUTATE pass
            // handing each layer its snapshot — same work, same order.
            use rayon::prelude::*;

            // FLPROF (R738, default OFF): per-layer census of BOTH fill passes.
            // `fill_loop` is 0.230 s with CLIPPER_UNION_GEO off and 0.866 s with it
            // on (+0.636 s) — the one blocker on that gate's +3,183 in-order. The
            // stage total cannot say whether that delta is spread over the 300
            // layers or concentrated in a few, and the last three findings
            // (R736, R737, R737's union census) were each ONE layer of 300.
            let flprof = crate::probe_enabled("FLPROF");
            let fl_snap: std::sync::Mutex<Vec<(usize, f64)>> =
                std::sync::Mutex::new(Vec::new());
            let fl_make: std::sync::Mutex<Vec<(usize, f64)>> =
                std::sync::Mutex::new(Vec::new());

            let lower_snapshots: Vec<(
                Vec<crate::geometry::ExPolygon>,
                Vec<crate::geometry::Polygon>,
                Option<Vec<crate::geometry::ExPolygon>>,
            )> = (0..self.layers.len())
                .into_par_iter()
                .map(|layer_idx| {
                let __t_snap = std::time::Instant::now();
                // BBS Fill.cpp:455-464 — gather the lower layer's stInternal /
                // stInternalVoid fill-surface expolygons (the floating-vertical-shell
                // detection in group_fills needs them). `group_fills` runs per-Layer
                // and cannot reach a sibling Layer, so collect here while we still
                // hold the whole `layers` slice, then hand the snapshot to make_fills.
                let lower_internal_areas: Vec<crate::geometry::ExPolygon> = self.layers
                    [layer_idx]
                    .lower_layer_id
                    .and_then(|lid| self.layers.get(lid))
                    .map(|lower| {
                        let mut areas: Vec<crate::geometry::ExPolygon> = Vec::new();
                        for layerm in lower.regions() {
                            for surface in layerm.fill_surfaces.filter_by_types(&[
                                crate::surface::SurfaceType::Internal,
                                crate::surface::SurfaceType::InternalVoid,
                            ]) {
                                areas.push(surface.expolygon.clone());
                            }
                        }
                        areas
                    })
                    .unwrap_or_default();
                if crate::probe_enabled("FVS_DEBUG") {
                    use std::sync::atomic::Ordering::Relaxed;
                    crate::layer::FVS_LAYERS.fetch_add(1, Relaxed);
                    if self.layers[layer_idx].lower_layer_id.is_none() {
                        crate::layer::FVS_NO_LOWER.fetch_add(1, Relaxed);
                    } else if lower_internal_areas.is_empty() {
                        crate::layer::FVS_LOWER_NOSURF.fetch_add(1, Relaxed);
                    }
                }

                // Fill.cpp:638-673 — for ipFloatingConcentric, the filler also needs
                // the lower layer's SPARSE-infill anchor polygons. C++ computes
                // lower_sparse_polys = union_(offset(lower_layer->
                // generate_sparse_infill_polylines_for_anchoring(...),
                // internal_infill_width/2)). Gather it here while the lower layer is
                // reachable (group_fills/make_fills run per-Layer), mirroring the
                // lower_internal_areas snapshot above. The detect_lower_sparse_lines
                // guard (skip for adaptive/lightning/support-cubic lower infill) is
                // satisfied for the non-adaptive Benchy case; the anchor generator
                // already skips those patterns.
                let lower_sparse_polys: Vec<crate::geometry::Polygon> = self.layers
                    [layer_idx]
                    .lower_layer_id
                    .and_then(|lid| self.layers.get(lid))
                    .map(|lower| {
                        let __t_anch = std::time::Instant::now();
                        let lines = lower
                            .generate_sparse_infill_polylines_for_anchoring()
                            .unwrap_or_default();
                        if flprof && __t_anch.elapsed().as_secs_f64() > 0.005 {
                            eprintln!(
                                "[FLPROF]   anchoring-gen for layer={layer_idx} \
(on lower layer) lines={} {:.4}s",
                                lines.len(), __t_anch.elapsed().as_secs_f64());
                        }
                        if lines.is_empty() {
                            return Vec::new();
                        }
                        // internal_infill_width = mean of layerm.flow(frInfill).scaled_width().
                        let mut sum_w = 0i64;
                        let mut nregions = 0usize;
                        for layerm in lower.regions() {
                            if let Ok(f) = layerm.flow(crate::flow::FlowRole::Infill, lower.height) {
                                sum_w += f.scaled_width();
                            }
                            nregions += 1;
                        }
                        let internal_infill_width = if nregions > 0 {
                            sum_w as f64 / nregions as f64
                        } else {
                            0.0
                        };
                        // offset(lower_sparse_lines, internal_infill_width/2) then union_.
                        if crate::faithful_gate("FFC_LOWER_SPARSE_CLIB") {
                            // Fill.cpp:756-762: `float internal_infill_width`, halved
                            // in float; `offset(Polylines, delta)` is one ClipperOffset
                            // per path (jtSquare, etOpenButt) and one
                            // `clipper_union<Paths>` over all of them
                            // (ClipperUtils.cpp:420-421, :273-300), then `union_`
                            // (:721-722), flat paths in Clipper's order.
                            let mut width = 0.0f32;
                            for layerm in lower.regions() {
                                if let Ok(f) =
                                    layerm.flow(crate::flow::FlowRole::Infill, lower.height)
                                {
                                    width += f.scaled_width() as f32;
                                }
                            }
                            width /= nregions as f32;
                            let delta = (width / 2.0f32) as f64;
                            let mut grown: Vec<crate::geometry::Polygon> = Vec::new();
                            for pl in &lines {
                                grown.extend(crate::extrusion_entity::offset_polyline_clib(
                                    pl, delta,
                                ));
                            }
                            let grown = crate::clipper_utils::union_flat_clib(&grown);
                            return crate::clipper_utils::union_flat_clib(&grown);
                        }
                        let delta = internal_infill_width / 2.0;
                        let mut grown: Vec<crate::geometry::Polygon> = Vec::new();
                        for pl in &lines {
                            grown.extend(crate::clipper_utils::offset_polyline(pl, delta));
                        }
                        if flprof {
                            let nv: usize = grown.iter().map(|g| g.points.len()).sum();
                            eprintln!(
                                "[FLPROF]   snapshot-union layer={layer_idx} \
lines={} grown_polys={} grown_verts={}",
                                lines.len(), grown.len(), nv);
                        }
                        // union_(grown): collapse to non-overlapping polygons.
                        crate::clipper_utils::union_polygons_ex(&grown)
                            .into_iter()
                            .flat_map(|ex| {
                                std::iter::once(ex.contour).chain(ex.holes.into_iter())
                            })
                            .collect()
                    })
                    .unwrap_or_default();

                if flprof {
                    fl_snap.lock().unwrap()
                        .push((layer_idx, __t_snap.elapsed().as_secs_f64()));
                }
                // Fill.cpp:734-745 — `lower_unsuporrt_expolys` of the floating
                // concentric filler: per region of the lower layer
                // `union_ex(stInternal + stInternalVoid)`, accumulated with
                // `union_ex(acc, region)` (ClipperUtils.cpp:825-831: the two lists
                // concatenated, one union), then `shrink_ex(acc, SCALED_EPSILON)`
                // (ClipperUtils.hpp:392: `offset_ex(ExPolygons, -delta)`, each
                // expolygon offset on its own and the paths united through a
                // PolyTree, ClipperUtils.cpp:552-571).
                let lower_unsupport: Option<Vec<crate::geometry::ExPolygon>> =
                    if crate::faithful_gate("FFC_LOWER_UNSUPPORT") {
                        self.layers[layer_idx]
                            .lower_layer_id
                            .and_then(|lid| self.layers.get(lid))
                            .map(|lower| {
                                let mut acc: Vec<crate::geometry::ExPolygon> = Vec::new();
                                for layerm in lower.regions() {
                                    let sexpolys: Vec<crate::geometry::ExPolygon> = layerm
                                        .fill_surfaces
                                        .filter_by_types(&[
                                            crate::surface::SurfaceType::Internal,
                                            crate::surface::SurfaceType::InternalVoid,
                                        ])
                                        .into_iter()
                                        .map(|surface| surface.expolygon.clone())
                                        .collect();
                                    let sexpolys = crate::clipper_utils::union_ex_clib(
                                        &crate::geometry::to_polygons(&sexpolys),
                                        1,
                                    );
                                    if !sexpolys.is_empty() {
                                        acc.extend(sexpolys);
                                        acc = crate::clipper_utils::union_ex_clib(
                                            &crate::geometry::to_polygons(&acc),
                                            1,
                                        );
                                    }
                                }
                                crate::clipper_utils::offset_expolygons_clib_scaled(
                                    &acc,
                                    -(crate::libslic3r::SCALED_EPSILON as f32) as f64,
                                    crate::clipper_utils::OffsetJoinType::Miter,
                                )
                            })
                    } else {
                        None
                    };
                (lower_internal_areas, lower_sparse_polys, lower_unsupport)
                })
                .collect();

            // Call Layer::make_fills() on each layer
            // PrintObject.cpp:768
            // Fill.cpp:228-229: layers().front()->bottom_z(), 0. for no layers (FILL_GROUP_PIN reads it).
            let fill_object_base_z: f64 = self.layers.first().map_or(0., |l| l.bottom_z());
            let canceled = self.canceled.clone();
            self.layers
                .par_iter_mut()
                .enumerate()
                .try_for_each(|(layer_idx, layer)| -> Result<()> {
                    if canceled.load(Ordering::Relaxed) {
                        return Err(Error::Cancelled);
                    }
                    let (lower_internal_areas, lower_sparse_polys, lower_unsupport) =
                        &lower_snapshots[layer_idx];
                    let __t_mk = std::time::Instant::now();
                    let r = layer.make_fills_at_object_base(
                        lower_internal_areas,
                        lower_sparse_polys,
                        lower_unsupport.as_deref(),
                        fill_object_base_z,
                    );
                    if flprof {
                        fl_make.lock().unwrap()
                            .push((layer_idx, __t_mk.elapsed().as_secs_f64()));
                    }
                    r
                })?;
            // PrintObject.cpp:1236
            crate::fill::fill_radial_zig_zag::reset_n_lock();

            if flprof {
                let report = |name: &str, m: std::sync::Mutex<Vec<(usize, f64)>>| {
                    let mut v = m.into_inner().unwrap();
                    let total: f64 = v.iter().map(|x| x.1).sum();
                    v.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap());
                    let top10: f64 = v.iter().take(10).map(|x| x.1).sum();
                    eprintln!(
                        "[FLPROF] {name} layers={} sum={:.3}s top10={:.3}s ({:.1}%)",
                        v.len(), total, top10,
                        if total > 0.0 { 100.0 * top10 / total } else { 0.0 });
                    for (l, t) in v.iter().take(8) {
                        eprintln!("[FLPROF]   {name} layer={l} {:.4}s", t);
                    }
                };
                report("snapshot", fl_snap);
                report("make_fills", fl_make);
            }

            if crate::probe_enabled("GYROID_ENDPOINT_DEBUG") {
                use crate::fill::{
                    GEP_CROSS_HIST, GEP_CROSS_N, GEP_CROSS_SUM_UM, GEP_HIST, GEP_N, GEP_PTS_HIST,
                    GEP_RAWVERT, GEP_RAW_SUM_UM, GEP_SUM_UM,
                };
                use std::sync::atomic::Ordering::Relaxed as R3;
                let n = GEP_N.load(R3).max(1);
                eprintln!(
                    "GYROID_ENDPOINTS: n={} mean dist {:.1} um | <1um {} ({:.1}%)  <10um {}  <100um {}  <1mm {}  >=1mm {}",
                    GEP_N.load(R3), GEP_SUM_UM.load(R3) as f64 / n as f64,
                    GEP_HIST[0].load(R3), 100.0 * GEP_HIST[0].load(R3) as f64 / n as f64,
                    GEP_HIST[1].load(R3), GEP_HIST[2].load(R3),
                    GEP_HIST[3].load(R3), GEP_HIST[4].load(R3),
                );
                eprintln!(
                    "GYROID_ENDPOINTS: of those, {} ({:.1}%) coincide with an ORIGINAL raw-wave vertex (legitimately interior); {} are clip CROSSINGS (must be on the boundary)",
                    GEP_RAWVERT.load(R3), 100.0 * GEP_RAWVERT.load(R3) as f64 / n as f64,
                    GEP_N.load(R3).saturating_sub(GEP_RAWVERT.load(R3)),
                );
                eprintln!(
                    "GYROID_ENDPOINTS: points-per-output-polyline  2pt={}  3-4={}  5-8={}  9-16={}  17+={}",
                    GEP_PTS_HIST[0].load(R3), GEP_PTS_HIST[1].load(R3), GEP_PTS_HIST[2].load(R3),
                    GEP_PTS_HIST[3].load(R3), GEP_PTS_HIST[4].load(R3),
                );
                {
                    use crate::fill::{SRC_CROSS_BAD, SRC_CROSS_N, SRC_CROSS_SUM};
                    let n = SRC_CROSS_N.load(R3).max(1);
                    eprintln!(
                        "GYROID_ENDPOINTS: AT SOURCE — crossing points emitted n={} mean dist {:.3} um, >1um: {} ({:.2}%)",
                        SRC_CROSS_N.load(R3),
                        SRC_CROSS_SUM.load(R3) as f64 / 1000.0 / n as f64,
                        SRC_CROSS_BAD.load(R3),
                        100.0 * SRC_CROSS_BAD.load(R3) as f64 / n as f64,
                    );
                }
                let cn = GEP_CROSS_N.load(R3).max(1);
                let rn = GEP_RAWVERT.load(R3).max(1);
                eprintln!(
                    "GYROID_ENDPOINTS: BY CLASS — crossings n={} mean {:.1} um (<1um {} <10um {} <100um {} >=100um {}) | raw-wave vertices n={} mean {:.1} um",
                    GEP_CROSS_N.load(R3), GEP_CROSS_SUM_UM.load(R3) as f64 / cn as f64,
                    GEP_CROSS_HIST[0].load(R3), GEP_CROSS_HIST[1].load(R3),
                    GEP_CROSS_HIST[2].load(R3), GEP_CROSS_HIST[3].load(R3),
                    GEP_RAWVERT.load(R3), GEP_RAW_SUM_UM.load(R3) as f64 / rn as f64,
                );
            }
            if crate::probe_enabled("FILL_CONNECT_DEBUG") {
                use crate::fill::fill_base::{
                    FB_ARCHES, FB_ENDS, FB_ENDS_UNCONNECTED, FB_IN, FB_LEN_IN, FB_LEN_OUT, FB_OUT,
                };
                use std::sync::atomic::Ordering::Relaxed as R2;
                let f2 = |x: &std::sync::atomic::AtomicUsize| x.load(R2) as f64 / 1000.0;
                eprintln!(
                    "FILL_CONNECT_FAITHFUL: polylines in={} out={} (joins={})  ends={} UNCONNECTED={} ({:.1}%)  arches={}  length in={:.0} out={:.0} mm (added {:.0})",
                    FB_IN.load(R2), FB_OUT.load(R2),
                    FB_IN.load(R2).saturating_sub(FB_OUT.load(R2)),
                    FB_ENDS.load(R2), FB_ENDS_UNCONNECTED.load(R2),
                    100.0 * FB_ENDS_UNCONNECTED.load(R2) as f64 / FB_ENDS.load(R2).max(1) as f64,
                    FB_ARCHES.load(R2),
                    f2(&FB_LEN_IN), f2(&FB_LEN_OUT), f2(&FB_LEN_OUT) - f2(&FB_LEN_IN),
                );
                use crate::fill::fill_base::{
                    FB_ARC_JOIN_LEN, FB_ARC_JOIN_N, FB_ARC_STUB_LEN, FB_ARC_STUB_N,
                    FB_TAIL_JOIN_LEN, FB_TAIL_JOIN_N, FB_TAIL_STUB_LEN, FB_TAIL_STUB_N,
                };
                eprintln!(
                    "FILL_CONNECT_WHERE: arch-loop join n={} len={:.0} mm | arch-loop stub n={} len={:.0} | tail join n={} len={:.0} | tail stub n={} len={:.0}",
                    FB_ARC_JOIN_N.load(R2), f2(&FB_ARC_JOIN_LEN),
                    FB_ARC_STUB_N.load(R2), f2(&FB_ARC_STUB_LEN),
                    FB_TAIL_JOIN_N.load(R2), f2(&FB_TAIL_JOIN_LEN),
                    FB_TAIL_STUB_N.load(R2), f2(&FB_TAIL_STUB_LEN),
                );
            }
            if crate::probe_enabled("VSHELL_DEBUG") {
                use std::sync::atomic::Ordering::Relaxed as R5;
                let n = crate::layer::VS_REG_IN.load(R5);
                let d = crate::layer::VS_REG_DROP.load(R5);
                eprintln!(
                    "VSHELL_REG: regularized pieces={} DROPPED={} ({:.1}%)  dropped area={:.1} mm2",
                    n, d, 100.0 * d as f64 / n.max(1) as f64,
                    crate::layer::VS_REG_DROP_AREA.load(R5) as f64 / 1000.0,
                );
                {
                    let f = |x: &std::sync::atomic::AtomicUsize| x.load(R5) as f64 / 1000.0;
                    eprintln!(
                        "VSHELL_TERMS (per region-layer, n={}): internal_all={:.0} mm2 | shell&internal={:.0} | internal-holes={:.0} | existing solid={:.0} | holes={:.0} (empty on {} of {}), shell&internal empty on {}",
                        crate::layer::VS_T_N.load(R5),
                        f(&crate::layer::VS_T_INTERNAL), f(&crate::layer::VS_T_SHELLINT),
                        f(&crate::layer::VS_T_DIFF), f(&crate::layer::VS_T_SOLID),
                        f(&crate::layer::VS_T_HOLES),
                        crate::layer::VS_T_HOLES_EMPTY.load(R5), crate::layer::VS_T_N.load(R5),
                        crate::layer::VS_T_SHELLINT_EMPTY.load(R5),
                    );
                }
                eprintln!(
                    "VSHELL_SHELL: shell pieces IN={} area={:.0} mm2  ->  regularized OUT={} area={:.0} mm2",
                    crate::layer::VS_SHELL_IN.load(R5),
                    crate::layer::VS_SHELL_IN_AREA.load(R5) as f64 / 1000.0,
                    crate::layer::VS_REG_OUT.load(R5),
                    crate::layer::VS_REG_OUT_AREA.load(R5) as f64 / 1000.0,
                );
                eprintln!(
                    "VSHELL_REG: cond1(small)={} of which area<thr15={} | cond2(does-not-cover-an-internal)={} | cond2 SAVED {} pieces cond1 wanted dropped",
                    crate::layer::VS_C1.load(R5), crate::layer::VS_C1_TINY.load(R5),
                    crate::layer::VS_C2.load(R5), crate::layer::VS_SAVED.load(R5),
                );
            }
            if crate::probe_enabled("FVS_DEBUG") {
                use std::sync::atomic::Ordering::Relaxed as R4;
                eprintln!(
                    "FVS_NO_OVERLAP: concentric/floating expolygons={}  with EMPTY no_overlap={} ({:.1}%)",
                    crate::layer::FVS_EXPOLYS.load(R4),
                    crate::layer::FVS_EMPTY_NOOVERLAP.load(R4),
                    100.0 * crate::layer::FVS_EMPTY_NOOVERLAP.load(R4) as f64
                        / crate::layer::FVS_EXPOLYS.load(R4).max(1) as f64,
                );
                eprintln!(
                    "FVS_LOWER: layers={} with NO lower_layer_id={} | lower present but NO Internal/InternalVoid fill_surfaces={}",
                    crate::layer::FVS_LAYERS.load(R4), crate::layer::FVS_NO_LOWER.load(R4),
                    crate::layer::FVS_LOWER_NOSURF.load(R4),
                );
                eprintln!(
                    "FVS_CLASSIFY (real make_fills pass only): InternalSolid expolygons tested={} narrow={} | clipped_internals EMPTY for {} (-> not floating)",
                    crate::layer::FVS_CAND.load(R4), crate::layer::FVS_NARROW.load(R4),
                    crate::layer::FVS_CLIP_EMPTY.load(R4),
                );
                eprintln!(
                    "FVS_SPLIT: beads={} flag_off={} self_intersect={} no_floating_areas={} closed_dup={} closed_nodup={}",
                    crate::layer::FVS_LINES.load(R4), crate::layer::FVS_FLAG_OFF.load(R4),
                    crate::layer::FVS_SELF_INT.load(R4), crate::layer::FVS_NO_AREAS.load(R4),
                    crate::layer::FVS_CLOSED_DUP.load(R4), crate::layer::FVS_CLOSED_NODUP.load(R4),
                );
                eprintln!(
                    "FVS_NO_OVERLAP: group_fills saw {} first-surface regions, {} with an EMPTY LayerRegion::fill_no_overlap_expolygons",
                    crate::layer::FVS_REGION_SEEN.load(R4),
                    crate::layer::FVS_REGION_EMPTY.load(R4),
                );
            }
            if crate::probe_enabled("FILL_CONNECT_DEBUG") {
                use crate::fill::{CONN_IN, CONN_LEN_IN, CONN_LEN_OUT, CONN_OUT};
                use std::sync::atomic::Ordering::Relaxed;
                let f = |x: &std::sync::atomic::AtomicUsize| x.load(Relaxed) as f64 / 1000.0;
                eprintln!(
                    "FILL_CONNECT: polylines in={} out={} (joins={})  length in={:.0} out={:.0} mm (added {:.0})",
                    CONN_IN.load(Relaxed), CONN_OUT.load(Relaxed),
                    CONN_IN.load(Relaxed).saturating_sub(CONN_OUT.load(Relaxed)),
                    f(&CONN_LEN_IN), f(&CONN_LEN_OUT), f(&CONN_LEN_OUT) - f(&CONN_LEN_IN),
                );
            }

            // R456: sparse-infill fill accounting — how much Internal AREA reached the
            // filler and how much of it produced no polylines at all.
            if crate::probe_enabled("FILL_SURFACE_DEBUG") {
                use crate::fill::*;
                use std::sync::atomic::Ordering::Relaxed;
                let a = |x: &std::sync::atomic::AtomicUsize| x.load(Relaxed) as f64 / 1000.0;
                let n = |x: &std::sync::atomic::AtomicUsize| x.load(Relaxed);
                let (ia, oa, na) = (a(&SPARSE_IN_AREA), a(&SPARSE_OK_AREA), a(&SPARSE_NOGEN_AREA));
                eprintln!(
                    "FILL_SPARSE: expolygons handed to filler n={} area={:.0} mm2 | filled n={} area={:.0} ({:.1}%) len={:.0} mm | NO-FILL n={} area={:.0} ({:.1}%) | empty n={}",
                    n(&SPARSE_IN_N), ia,
                    n(&SPARSE_OK_N), oa, 100.0 * oa / ia.max(1e-9), a(&SPARSE_OK_LEN),
                    n(&SPARSE_NOGEN_N), na, 100.0 * na / ia.max(1e-9),
                    n(&SPARSE_EMPTY_N),
                );
            }

            // Mark step as complete
            // PrintObject.cpp:776
            self.set_step_done(PrintObjectStep::Infill);
        }
        if __timing {
            eprintln!(
                "    infill split: prepare_infill(+MMS) {:.3}s  fill_loop {:.3}s",
                __prep_s,
                __t_fill.elapsed().as_secs_f64()
            );
        }

        Ok(())
    }

    /// Optimize (simplify) every extrusion toolpath of the object.
    ///
    /// PrintObject.cpp:902-938 — `PrintObject::simplify_extrusion_path()`.
    /// C++ runs two phases (posSimplifyWall, posSimplifyInfill); each iterates
    /// all layers and calls `Layer::simplify_wall_extrusion_path()` /
    /// `simplify_infill_extrusion_path()`, which fan out per LayerRegion to
    /// `simplify_entity_collection(&perimeters)` / `(&fills)`
    /// (Layer.hpp:113-114, 221-222). With arc-fitting enabled this DP-simplifies
    /// and arc-fits each path at `scaled(resolution)` (LayerRegion.cpp:785-801).
    ///
    /// This was previously a no-op TODO in the Rust pipeline (Print.cpp:2231),
    /// which left every toolpath at full medial-axis / fill vertex density — most
    /// visibly inflating gap-fill G1 move count ~3.5x vs native. We also simplify
    /// `thin_fills` here: BambuStudio copies thin_fills into `fills` before the
    /// infill simplify pass (Fill.cpp:752-761) so they get simplified there,
    /// whereas this crate keeps and exports gap-fill from `thin_fills`.
    ///
    /// PrintObject.cpp:902-938
    pub fn simplify_extrusion_path(&mut self) {
        // posSimplifyWall + posSimplifyInfill (PrintObject.cpp:904-938).
        if !self.is_step_done(PrintObjectStep::SimplifyWall) {
            for layer in self.layers.iter_mut() {
                for region in layer.regions_mut() {
                    region.simplify_wall_extrusion_entity();
                }
            }
            self.set_step_done(PrintObjectStep::SimplifyWall);
        }

        if !self.is_step_done(PrintObjectStep::SimplifyInfill) {
            for layer in self.layers.iter_mut() {
                for region in layer.regions_mut() {
                    region.simplify_infill_extrusion_entity();
                    // LayerRegion.hpp simplify_infill_extrusion_path simplifies
                    // `fills` only, which hold the thin-fill clones
                    // (THIN_FILLS_CLONE); =0 also simplifies `thin_fills`.
                    if !crate::faithful_gate("THIN_FILLS_CLONE") {
                        region.simplify_thin_fill_extrusion_entity();
                    }
                }
            }
            self.set_step_done(PrintObjectStep::SimplifyInfill);
        }
    }

    /// PrintObject.cpp:801-810 `PrintObject::clear_overhangs_for_lift`.
    fn clear_overhangs_for_lift(&mut self) {
        for l in &mut self.layers {
            l.loverhangs.clear();
        }
    }

    /// PrintObject.cpp:814-853 `PrintObject::detect_overhangs_for_lift`.
    ///
    /// Fills `Layer::loverhangs` — the avoidance region the Auto z-hop consults.
    /// C++'s `is_through_overhang` (GCode.cpp:6972-7027) tests each travel against
    /// these polygons and promotes the lift to a spiral when the travel crosses
    /// one; otherwise the lift is a slope. R630 measured what happens without
    /// this: the lift type collapses to a single constant and Majora emits 35,751
    /// object `G17` against C++'s 6,062. Until R631 the field was declared and
    /// initialised empty at layer.rs:1274/1327 and read by nothing.
    ///
    /// The overhang of a layer is what it adds beyond its lower layer, allowing
    /// `g_min_overhang_percent_for_lift` (0.3) of a line width of overlap, then
    /// opened by ±0.1 line widths to drop slivers.
    pub fn detect_overhangs_for_lift(&mut self) {
        use crate::clipper_utils::{
            difference_clib, offset2_ex_clib, offset_expolygons_clib, OffsetJoinType,
        };

        // PrintObject.cpp:816-817
        const G_MIN_OVERHANG_PERCENT_FOR_LIFT: crate::CoordF = 0.3;
        let min_overlap = self.config.line_width * G_MIN_OVERHANG_PERCENT_FOR_LIFT;
        let open_delta = 0.1 * self.config.line_width;
        let num_raft_layers = self.slicing_params.raft_layers();

        // PrintObject.cpp:823
        self.clear_overhangs_for_lift();

        // PrintObject.cpp:826 — `blocked_range(num_raft_layers + 1, num_layers)`:
        // layer 0 (and any raft layer) has no lower layer, so it is skipped.
        for layer_id in (num_raft_layers + 1)..self.layers.len() {
            // PrintObject.cpp:831-832
            // The `_clib` variants are the C++-exact Clipper ports; the `geo`
            // ones diverge enough here to matter — R631 measured the geo path
            // opening 275 overhang polygons across Benchy's 299 layers, most
            // layers ending empty, and `is_through_overhang` fired 6x too rarely
            // as a result.
            let overhangs = difference_clib(
                &self.layers[layer_id].lslices,
                &offset_expolygons_clib(
                    &self.layers[layer_id - 1].lslices,
                    min_overlap,
                    OffsetJoinType::Miter,
                ),
            );
            let opened =
                offset2_ex_clib(&overhangs, -open_delta, open_delta, OffsetJoinType::Miter);

            // PrintObject.cpp:849
            let bbox = crate::geometry::get_extents(&opened);
            if crate::probe_enabled("OVERHANG_LIFT_CENSUS") {
                // R632: AREA is the discriminator, not the polygon count — a
                // count of 0 after the opening says nothing about how much
                // overhang was there to begin with. Areas are in scaled units^2;
                // divide by SCALING_FACTOR^2 for mm^2.
                let sf = crate::SCALING_FACTOR * crate::SCALING_FACTOR;
                eprintln!(
                    "OHLIFT layer={} z={:.3} lslices={} lslices_area={:.4} lower_lslices={} raw={} raw_area={:.4} opened={} opened_area={:.4} lw={:.4}",
                    layer_id,
                    self.layers[layer_id].print_z,
                    self.layers[layer_id].lslices.len(),
                    crate::geometry::area_expolygons(&self.layers[layer_id].lslices) / sf,
                    self.layers[layer_id - 1].lslices.len(),
                    overhangs.len(),
                    crate::geometry::area_expolygons(&overhangs) / sf,
                    opened.len(),
                    crate::geometry::area_expolygons(&opened) / sf,
                    self.config.line_width
                );
            }
            let layer = &mut self.layers[layer_id];
            layer.loverhangs = opened;
            layer.loverhangs_bbox = bbox;
        }

        // PrintObject.cpp:836-847 REGISTER_SUPPORTS_FOR_LIFT appends the support
        // islands (or the spacing-covered support fills) to the same region. Not
        // ported yet — neither Benchy nor Majora prints support, so it cannot
        // move either fixture; port it before trusting a supported model.
    }

    /// Generate support material using SupportGenerator from support/
    /// PrintObject.cpp:856-901
    pub fn generate_support_material(&mut self) -> Result<()> {
        use crate::support::{SupportConfig, SupportGenerator, SupportType as SupportGenType};

        // PrintObject.cpp:1332
        // C++: if (this->set_started(posSupportMaterial)) {
        // set_started throws CanceledException before it looks at the step
        // state (PrintBase.hpp:154-157, :171-172). Gate PRINT_OBJECT_STARTED_CANCEL
        // (on unless =0); the slicing path never cancels, so G-code is unchanged.
        if crate::faithful_gate("PRINT_OBJECT_STARTED_CANCEL") && self.canceled.load(Ordering::Relaxed) {
            return Err(Error::Cancelled);
        }
        if self.is_step_done(PrintObjectStep::SupportMaterial) {
            return Ok(());
        }

        // PrintObject.cpp:858
        // C++: this->clear_support_layers();
        self.clear_support_layers();

        // PrintObject.cpp:860-889
        // C++: if (!has_support() && !m_print->get_no_check_flag()) { ... is_support_necessary() warnings ... }
        // FIDELITY-NOTE: the is_support_necessary() overhang-warning path is part of
        // the (unported) support necessity subsystem; only the warning is emitted in
        // C++, no slicing state changes, so its omission does not alter geometry.

        // PrintObject.cpp:891
        // C++: if ((this->has_support() && m_layers.size() > 1) || (this->has_raft() && !m_layers.empty()))
        // (has_raft() <=> raft_layers > 0 here.)
        let do_generate = (self.has_support() && self.layers.len() > 1)
            || (self.config.raft_layers > 0 && !self.layers.is_empty());
        if !do_generate {
            self.set_step_done(PrintObjectStep::SupportMaterial);
            return Ok(());
        }

        // PrintObject.cpp:894
        // C++: this->_generate_support_material();
        // The remainder reproduces _generate_support_material via this crate's
        // SupportGenerator (the C++ support subsystem is not 1:1 ported).

        // Build layer slice data: (z_height, layer_height, expolygons)
        let layer_slices: Vec<(f64, f64, Vec<crate::geometry::ExPolygon>)> = self
            .layers
            .iter()
            .map(|layer| (layer.print_z, layer.height, layer.lslices.clone()))
            .collect();

        // Map PrintObjectConfig support_type to support module's SupportType
        // Tree support is not yet fully ported — fall back to Normal to avoid crash
        let support_type = match self.config.support_type {
            crate::print_config::SupportType::Tree => {
                log::warn!("Tree support not yet stable, falling back to Normal support");
                SupportGenType::Normal
            }
            crate::print_config::SupportType::Hybrid => {
                log::warn!("Hybrid support not yet stable, falling back to Normal support");
                SupportGenType::Normal
            }
            crate::print_config::SupportType::Normal => SupportGenType::Normal,
        };

        // Build SupportConfig from PrintObjectConfig
        let mut support_config = SupportConfig::enabled();
        support_config.support_type = support_type;
        support_config.overhang_angle = self.config.support_threshold_angle;
        support_config.tree_branch_angle = self.config.tree_support_branch_angle;
        support_config.tree_branch_diameter = self.config.tree_support_branch_diameter;

        // Generate support layers
        let generator = SupportGenerator::new(support_config);
        let support_layers = generator.generate(&layer_slices);

        // Convert support::SupportLayer results into Layer structs and store
        self.support_layers.clear();
        for sl in &support_layers {
            if sl.is_empty() {
                continue;
            }
            let mut layer = Layer::new(sl.layer_id, self.model_object_id, sl.height, sl.z, sl.z);
            layer.lslices = sl.support_regions.clone();
            self.support_layers.push(layer);
        }

        // The support layers above are created AFTER the end-of-slice
        // wire_layer_hierarchy pass — stamp their config Arcs now so that
        // `layer.object()`/`.print()` work on them. They carry no
        // LayerRegions, so `LayerRegion::region` stays None (support layers
        // are region-less; only LayerRegion::region() is fail-fast).
        let object_config = Arc::new(self.config.clone());
        let print_config = self.print_config.clone();
        for layer in &mut self.support_layers {
            layer.wire_config_hierarchy(&object_config, &print_config, &[]);
        }

        self.set_step_done(PrintObjectStep::SupportMaterial);
        Ok(())
    }

    /// Gate `TREE_SUPPORT_ORGANIC_PIN` (opt_in_gate, R932; print.rs calls this before `generate_support_material`
    /// only with the gate on): the tree support types go through the pin's support step instead of the mapping to
    /// Normal in `generate_support_material`: PrintObject.cpp:891 (the same `do_generate` test), :5926-5930
    /// (`TreeSupport tree_support(*this, m_slicing_params); tree_support.generate();`) on the faithful PrintObject of
    /// this object's ModelObject in `print` (`support::tree_support_3d::assemble_faithful_print`, PrintApply.cpp),
    /// with this object's layers handed over and the support layers handed back
    /// (`support::tree_support_3d::generate_live_tree_support`), configs wired as `generate_support_material` does.
    /// Returns false (nothing done) for a non-tree support type, when there is nothing to generate or when `print`
    /// holds no object of this ModelObject, so the caller runs `generate_support_material`.
    pub fn generate_support_material_tree_pin(&mut self, print: &mut crate::print::faithful::Print) -> Result<bool> {
        if self.is_step_done(PrintObjectStep::SupportMaterial) {
            return Ok(false);
        }
        if self.config.support_type != crate::print_config::SupportType::Tree {
            return Ok(false);
        }
        // PrintObject.cpp:891
        let do_generate = (self.has_support() && self.layers.len() > 1)
            || (self.config.raft_layers > 0 && !self.layers.is_empty());
        if !do_generate {
            return Ok(false);
        }
        let Some(res) = crate::support::tree_support_3d::generate_live_tree_support(print, self.model_object_id, &self.layers) else {
            return Ok(false);
        };
        // PrintObject.cpp:858 `this->clear_support_layers();`
        self.clear_support_layers();
        self.support_layers = res?;
        let object_config = Arc::new(self.config.clone());
        let print_config = self.print_config.clone();
        for layer in &mut self.support_layers {
            layer.wire_config_hierarchy(&object_config, &print_config, &[]);
        }
        self.set_step_done(PrintObjectStep::SupportMaterial);
        Ok(true)
    }

    /// Generate ironing paths for top surfaces on each layer.
    /// PrintObject.cpp:840-855 (Layer::make_ironing)
    pub fn ironing(&mut self) -> Result<()> {
        use crate::gcode::ironing::{
            IroningConfig as GcIroningConfig, IroningGenerator, IroningType as GcIroningType,
        };
        use crate::print_config::IroningType;

        // PrintObject.cpp:1251  `if (this->set_started(posIroning)) {`: set_started
        // throws CanceledException first (PrintBase.hpp:154-157), and the layer
        // loop and its end check the print again (PrintObject.cpp:1258, :1263).
        // Gate PRINT_OBJECT_STARTED_CANCEL (on unless =0); the slicing path never
        // cancels, so G-code is unchanged.
        let cancel_checks = crate::faithful_gate("PRINT_OBJECT_STARTED_CANCEL");
        if cancel_checks && self.canceled.load(Ordering::Relaxed) {
            return Err(Error::Cancelled);
        }

        // Map print_config IroningType to gcode ironing IroningType
        let ironing_type = match self.config.ironing_type {
            IroningType::NoIroning => {
                self.set_step_done(PrintObjectStep::Ironing);
                return Ok(());
            }
            IroningType::TopSurfaces => GcIroningType::TopSurfaces,
            IroningType::TopmostOnly => GcIroningType::TopmostOnly,
            IroningType::AllSolid => GcIroningType::AllSolid,
        };

        let ironing_config = GcIroningConfig {
            ironing_type,
            flow_percent: self.config.ironing_flow * 100.0,
            speed: self.config.ironing_speed,
            line_spacing: self.config.ironing_spacing,
            direction: self.config.ironing_direction,
            ..GcIroningConfig::default()
        };

        let generator = IroningGenerator::new(ironing_config);
        let num_layers = self.layers.len();

        for layer_idx in 0..num_layers {
            // PrintObject.cpp:1258  m_print->throw_if_canceled();
            if cancel_checks && self.canceled.load(Ordering::Relaxed) {
                return Err(Error::Cancelled);
            }
            let has_layer_above = layer_idx + 1 < num_layers;

            // Collect top surface expolygons from all regions in this layer
            let top_expolygons: Vec<crate::geometry::ExPolygon> = self.layers[layer_idx]
                .regions()
                .iter()
                .flat_map(|region| {
                    region
                        .get_slices()
                        .top_surfaces()
                        .into_iter()
                        .map(|s| s.expolygon.clone())
                })
                .collect();

            if top_expolygons.is_empty() {
                continue;
            }

            let result = generator.generate(&top_expolygons, None, layer_idx, has_layer_above);
            if result.is_empty() {
                continue;
            }

            // Convert ironing paths to extrusion paths and add to the layer
            for ironing_path in &result.paths {
                let mut ext_path = crate::extrusion_entity::ExtrusionPath::new(
                    crate::extrusion_entity::ExtrusionRole::Ironing,
                );
                ext_path.polyline =
                    crate::geometry::Polyline::from_points(ironing_path.points.clone());
                ext_path.mm3_per_mm = ironing_path.flow;
                ext_path.width = ironing_path.width;
                ext_path.height = ironing_path.height;
                // Add ironing fills to the first region of this layer
                if let Some(region) = self.layers[layer_idx].regions_mut().first_mut() {
                    region
                        .fills
                        .entities
                        .push(crate::extrusion_entity::ExtrusionEntityType::Path(ext_path));
                }
            }
        }

        // PrintObject.cpp:1263  m_print->throw_if_canceled();
        if cancel_checks && self.canceled.load(Ordering::Relaxed) {
            return Err(Error::Cancelled);
        }
        self.set_step_done(PrintObjectStep::Ironing);
        Ok(())
    }

    /// Check if object needs support
    /// PrintObject.cpp:858
    pub fn has_support(&self) -> bool {
        self.config.enable_support || self.config.enforce_support_layers > 0
    }

    /// Get number of print regions
    /// Print.hpp:469
    /// C++: size_t num_printing_regions() const throw() { return m_shared_regions->all_regions.size(); }
    pub fn num_printing_regions(&self) -> usize {
        self.shared_regions
            .as_ref()
            .map(|r| r.all_regions.len())
            .unwrap_or(0)
    }

    /// Get reference to a printing region by index
    /// Print.hpp:470
    /// C++: const PrintRegion& printing_region(size_t idx) const throw() { return *m_shared_regions->all_regions[idx].get(); }
    pub fn printing_region(&self, idx: usize) -> Option<&PrintRegion> {
        self.shared_regions
            .as_ref()
            .and_then(|r| r.all_regions.get(idx).map(|arc| arc.as_ref()))
    }

    /// Get all regions
    /// PrintObject.cpp:127-133
    pub fn all_regions(&self) -> Vec<&PrintRegion> {
        self.shared_regions
            .as_ref()
            .map(|r| r.all_regions.iter().map(|arc| arc.as_ref()).collect())
            .unwrap_or_default()
    }

    /// Merge perimeter loop nodes of a layer into connected cooling components.
    /// PrintObject.cpp:135-194
    /// C++: void PrintObject::merge_layer_node(const size_t layer_id, int &max_merged_id,
    ///          std::map<int, std::vector<std::pair<int, int>>> &node_record)
    ///
    /// `merged_id` is stored as Option<usize> in Rust's LoopNode (C++ `int merged_id = -1`):
    /// `None` == the C++ sentinel `-1`. Every id assigned by this routine is >= 1, so the
    /// min comparison `min_merged_id == -1 || min_merged_id > id` maps cleanly onto Option
    /// ordering (`None < Some(_)`).
    pub fn merge_layer_node(
        &mut self,
        layer_id: usize,
        max_merged_id: &mut i32,
        node_record: &mut std::collections::BTreeMap<i32, Vec<(i32, i32)>>,
    ) {
        // PrintObject.cpp:137-138
        // C++: Layer *this_layer = m_layers[layer_id];
        // C++: std::vector<LoopNode> &loop_nodes = this_layer->loop_nodes;
        let loop_nodes_len = self.layers[layer_id].loop_nodes.len();
        // PrintObject.cpp:139
        for idx in 0..loop_nodes_len {
            // PrintObject.cpp:140-148
            // C++: //new cool node
            // C++: if (loop_nodes[idx].lower_node_id.empty()) {
            if self.layers[layer_id].loop_nodes[idx].lower_node_ids.is_empty() {
                // PrintObject.cpp:142
                *max_merged_id += 1;
                // PrintObject.cpp:143
                self.layers[layer_id].loop_nodes[idx].merged_id = Some(*max_merged_id as usize);
                // PrintObject.cpp:144-146
                // C++: std::vector<std::pair<int, int>> node_pos;
                // C++: node_pos.emplace_back(layer_id, idx);
                // C++: node_record.emplace(max_merged_id, node_pos);
                let node_pos = vec![(layer_id as i32, idx as i32)];
                node_record.insert(*max_merged_id, node_pos);
                // PrintObject.cpp:147
                continue;
            }

            // PrintObject.cpp:150-155
            // C++: //it should finds key in map
            // C++: if (loop_nodes[idx].lower_node_id.size() == 1) {
            if self.layers[layer_id].loop_nodes[idx].lower_node_ids.len() == 1 {
                // PrintObject.cpp:152
                // C++: loop_nodes[idx].merged_id = m_layers[layer_id - 1]->loop_nodes[loop_nodes[idx].lower_node_id.front()].merged_id;
                let lower = self.layers[layer_id].loop_nodes[idx].lower_node_ids[0];
                let merged_id = self.layers[layer_id - 1].loop_nodes[lower].merged_id;
                self.layers[layer_id].loop_nodes[idx].merged_id = merged_id;
                // PrintObject.cpp:153
                // C++: node_record[loop_nodes[idx].merged_id].emplace_back(layer_id, idx);
                if let Some(id) = merged_id {
                    node_record
                        .entry(id as i32)
                        .or_default()
                        .push((layer_id as i32, idx as i32));
                }
                // PrintObject.cpp:154
                continue;
            }

            // PrintObject.cpp:157-165
            // C++: //min index
            // C++: int min_merged_id = -1;
            // C++: std::vector<int> appear_id;
            let mut min_merged_id: Option<usize> = None;
            let mut appear_id: Vec<Option<usize>> = Vec::new();
            let lower_node_ids = self.layers[layer_id].loop_nodes[idx].lower_node_ids.clone();
            for &lower in &lower_node_ids {
                // PrintObject.cpp:161
                // C++: int id = m_layers[layer_id - 1]->loop_nodes[loop_nodes[idx].lower_node_id[lower_idx]].merged_id;
                let id = self.layers[layer_id - 1].loop_nodes[lower].merged_id;
                // PrintObject.cpp:162-163
                // C++: if (min_merged_id == -1 || min_merged_id > id)
                // C++:     min_merged_id = id;
                if min_merged_id.is_none() || min_merged_id > id {
                    min_merged_id = id;
                }
                // PrintObject.cpp:164
                appear_id.push(id);
            }

            // PrintObject.cpp:167-168
            // C++: loop_nodes[idx].merged_id = min_merged_id;
            // C++: node_record[min_merged_id].emplace_back(layer_id, idx);
            self.layers[layer_id].loop_nodes[idx].merged_id = min_merged_id;
            if let Some(min_id) = min_merged_id {
                node_record
                    .entry(min_id as i32)
                    .or_default()
                    .push((layer_id as i32, idx as i32));
            }

            // PrintObject.cpp:170-192
            // C++: //update other node merged id
            // C++: for (size_t appear_node_idx = 0; appear_node_idx < appear_id.size(); ++appear_node_idx) {
            for &appear in &appear_id {
                // PrintObject.cpp:172-173
                // C++: if (appear_id[appear_node_idx] == min_merged_id)
                // C++:     continue;
                if appear == min_merged_id {
                    continue;
                }

                // PrintObject.cpp:175-178
                // C++: auto it = node_record.find(appear_id[appear_node_idx]);
                // C++: //protect
                // C++: if (it == node_record.end())
                // C++:     continue;
                let appear_key = match appear {
                    Some(a) => a as i32,
                    None => -1,
                };
                let appear_node_pos = match node_record.get(&appear_key) {
                    Some(v) => v.clone(),
                    None => continue,
                };

                // PrintObject.cpp:182-190
                // C++: for (size_t node_idx = 0; node_idx < appear_node_pos.size(); ++node_idx) {
                for &(node_layer, node_pos) in &appear_node_pos {
                    // PrintObject.cpp:186-188
                    // C++: LoopNode &node = m_layers[node_layer]->loop_nodes[node_pos];
                    // C++: node.merged_id = min_merged_id;
                    self.layers[node_layer as usize].loop_nodes[node_pos as usize].merged_id =
                        min_merged_id;
                    // PrintObject.cpp:189
                    // C++: node_record[min_merged_id].emplace_back(node_layer, node_pos);
                    if let Some(min_id) = min_merged_id {
                        node_record
                            .entry(min_id as i32)
                            .or_default()
                            .push((node_layer, node_pos));
                    }
                }
                // PrintObject.cpp:191
                // C++: node_record.erase(it);
                node_record.remove(&appear_key);
            }
        }
    }

    /// Clear all layers.
    /// PrintObject.cpp:1007-1014
    /// C++: void PrintObject::clear_layers() — guarded by `if (!m_shared_object)`.
    /// The Rust port stores layers by value (no shared-object aliasing), so the
    /// guard is unconditionally true here.
    pub fn clear_layers(&mut self) {
        // PrintObject.cpp:1010-1012
        self.layers.clear();
    }

    /// Append a new (empty) layer and return its index.
    /// PrintObject.cpp:1016-1020
    /// C++: Layer* PrintObject::add_layer(int id, coordf_t height, coordf_t print_z, coordf_t slice_z)
    /// returns Layer*; the Rust port returns the index into `layers` (which is the
    /// stable handle equivalent given layers are stored by value).
    pub fn add_layer(&mut self, id: i32, height: f64, print_z: f64, slice_z: f64) -> usize {
        // PrintObject.cpp:1018
        // C++: m_layers.emplace_back(new Layer(id, this, height, print_z, slice_z));
        self.layers
            .push(Layer::new(id as usize, self.model_object_id, height, print_z, slice_z));
        // PrintObject.cpp:1019
        // C++: return m_layers.back();
        self.layers.len() - 1
    }

    /// Get the support layer (index) approximately at `print_z` within `epsilon`.
    /// PrintObject.cpp:1022-1027
    /// C++: const SupportLayer* PrintObject::get_support_layer_at_printz(coordf_t print_z, coordf_t epsilon) const
    pub fn get_support_layer_at_printz(&self, print_z: f64, epsilon: f64) -> Option<usize> {
        // PrintObject.cpp:1024
        let limit = print_z - epsilon;
        // PrintObject.cpp:1025
        // C++: auto it = lower_bound_by_predicate(..., [limit](const SupportLayer* layer) { return layer->print_z < limit; });
        let it = self
            .support_layers
            .iter()
            .position(|layer| !(layer.print_z < limit));
        // PrintObject.cpp:1026
        // C++: return (it == end || (*it)->print_z > print_z + epsilon) ? nullptr : *it;
        match it {
            Some(i) if self.support_layers[i].print_z <= print_z + epsilon => Some(i),
            _ => None,
        }
    }

    /// Clear all support layers.
    /// PrintObject.cpp:1034-1046
    /// C++: void PrintObject::clear_support_layers() — guarded by `if (!m_shared_object)`.
    pub fn clear_support_layers(&mut self) {
        // PrintObject.cpp:1037-1039
        self.support_layers.clear();
        // PrintObject.cpp:1040-1044
        // C++: for (auto l : m_layers) { l->sharp_tails.clear(); l->sharp_tails_height.clear(); l->cantilevers.clear(); }
        for l in &mut self.layers {
            l.sharp_tails.clear();
            l.sharp_tails_height = 0.0;
            l.cantilevers.clear();
        }
    }

    /// Get a layer index approximately at `print_z` (default epsilon = EPSILON).
    /// PrintObject.cpp:4086, 4091-4095
    /// C++: const Layer* PrintObject::get_layer_at_printz(coordf_t print_z, coordf_t epsilon) const
    pub fn get_layer_at_printz(&self, print_z: f64, epsilon: f64) -> Option<usize> {
        // PrintObject.cpp:4092
        let limit = print_z - epsilon;
        // PrintObject.cpp:4093
        // C++: auto it = lower_bound_by_predicate(..., [limit](const Layer *layer) { return layer->print_z < limit; });
        let it = self.layers.iter().position(|layer| !(layer.print_z < limit));
        // PrintObject.cpp:4094
        // C++: return (it == end || (*it)->print_z > print_z + epsilon) ? nullptr : *it;
        match it {
            Some(i) if self.layers[i].print_z <= print_z + epsilon => Some(i),
            _ => None,
        }
    }

    /// Get the first layer index strictly below `print_z` (within epsilon).
    /// PrintObject.cpp:4101-4106
    /// C++: const Layer *PrintObject::get_first_layer_bellow_printz(coordf_t print_z, coordf_t epsilon) const
    pub fn get_first_layer_bellow_printz(&self, print_z: f64, epsilon: f64) -> Option<usize> {
        // PrintObject.cpp:4103
        let limit = print_z + epsilon;
        // PrintObject.cpp:4104
        let it = self.layers.iter().position(|layer| !(layer.print_z < limit));
        // PrintObject.cpp:4105
        // C++: return (it == begin) ? nullptr : *(--it);
        match it {
            Some(0) => None,
            Some(i) => Some(i - 1),
            // it == end(): the C++ `--it` yields the last element.
            None => {
                if self.layers.is_empty() {
                    None
                } else {
                    Some(self.layers.len() - 1)
                }
            }
        }
    }

    /// Get the layer index near `print_z`, or -1 if it would be the first layer.
    /// PrintObject.cpp:4107-4111
    /// C++: int PrintObject::get_layer_idx_get_printz(coordf_t print_z, coordf_t epsilon)
    pub fn get_layer_idx_get_printz(&self, print_z: f64, epsilon: f64) -> i32 {
        // PrintObject.cpp:4108
        let limit = print_z + epsilon;
        // PrintObject.cpp:4109
        let it = self.layers.iter().position(|layer| !(layer.print_z < limit));
        // PrintObject.cpp:4110
        // C++: return (it == begin) ? -1 : std::distance(begin, it);
        match it {
            Some(0) => -1,
            Some(i) => i as i32,
            // it == end(): std::distance(begin, end) == size().
            None => self.layers.len() as i32,
        }
    }

    /// Get a layer index whose bottom_z is approximately `bottom_z` (within epsilon).
    /// PrintObject.cpp:4113-4123
    /// C++: const Layer* PrintObject::get_layer_at_bottomz(coordf_t bottom_z, coordf_t epsilon) const
    pub fn get_layer_at_bottomz(&self, bottom_z: f64, epsilon: f64) -> Option<usize> {
        // PrintObject.cpp:4114-4115
        let limit_upper = bottom_z + epsilon;
        let limit_lower = bottom_z - epsilon;
        // PrintObject.cpp:4117-4121
        for (i, layer) in self.layers.iter().enumerate() {
            if layer.bottom_z() > limit_lower {
                return if layer.bottom_z() < limit_upper {
                    Some(i)
                } else {
                    None
                };
            }
        }
        // PrintObject.cpp:4122
        None
    }

    /// Discover vertical shells — ensure minimum solid shell thickness near sloped walls
    /// by projecting each layer's top/bottom surfaces across the shell-layer window and
    /// converting the matching internal regions to internal-solid.
    /// Faithful port of C++ PrintObject::discover_vertical_shells (PrintObject.cpp:1739-2110),
    /// including both the single-region per-region cache path and the multi-material
    /// `top_bottom_surfaces_all_regions` path (PrintObject.cpp:1756-1820), which builds
    /// the cache ONCE over every region and shares it across all region_id iterations.
    fn discover_vertical_shells(&mut self) -> Result<()> {
        use crate::clipper_utils::OffsetJoinType;
        use crate::flow::FlowRole;
        // Gated shadows of the geo primitives: under TOPFILL_FAITHFUL every
        // classification op routes through the vertex-exact vendored ClipperLib
        // @1e5 (native semantics); default keeps geo @1um (byte-locked). This is
        // the R100 gridding class applied to the whole vertical-shell chain.
        fn faithful() -> bool {
            crate::faithful_gate("TOPFILL_FAITHFUL")
        }
        fn grow(
            e: &[crate::geometry::ExPolygon],
            d: crate::CoordF,
            j: OffsetJoinType,
        ) -> crate::geometry::ExPolygons {
            if faithful() && std::env::var("VSHELL_RAW").is_ok() {
                // R297: native expand()/offset() here are RAW Polygons offsets —
                // per-expolygon ClipperOffset paths, NO union reconstruction
                // (Class-5). Keep each output path as its own ExPolygon; the
                // downstream union_/intersection consume them like native's
                // appended Polygons. Delta mirrors the float param.
                let d_sc = ((d / 0.00001) as f32) as f64;
                crate::clipper_utils::offset_expolygons_clib_raw_scaled(e, d_sc, j)
                    .into_iter()
                    .map(crate::geometry::ExPolygon::new)
                    .collect()
            } else if faithful() {
                crate::clipper_utils::offset_expolygons_clib(e, d, j)
            } else {
                crate::clipper_utils::grow(e, d, j)
            }
        }
        fn shrink(
            e: &[crate::geometry::ExPolygon],
            d: crate::CoordF,
            j: OffsetJoinType,
        ) -> crate::geometry::ExPolygons {
            if faithful() {
                crate::clipper_utils::shrink_clib(e, d, j)
            } else {
                crate::clipper_utils::shrink(e, d, j)
            }
        }
        fn closing(
            e: &[crate::geometry::ExPolygon],
            d: crate::CoordF,
            j: OffsetJoinType,
        ) -> crate::geometry::ExPolygons {
            if faithful() {
                crate::clipper_utils::offset2_ex_clib(e, d, -d, j)
            } else {
                crate::clipper_utils::closing(e, d, j)
            }
        }
        fn offset2(
            e: &[crate::geometry::ExPolygon],
            shrink_amount: crate::CoordF,
            grow_amount: crate::CoordF,
            j: OffsetJoinType,
        ) -> crate::geometry::ExPolygons {
            if faithful() {
                crate::clipper_utils::offset2_ex_clib(e, -shrink_amount, grow_amount, j)
            } else {
                crate::clipper_utils::offset2(e, shrink_amount, grow_amount, j)
            }
        }
        fn union_ex(e: &[crate::geometry::ExPolygon]) -> crate::geometry::ExPolygons {
            if faithful() {
                crate::clipper_utils::union_ex_clib(&crate::geometry::to_polygons(e), 1)
            } else {
                crate::clipper_utils::union_ex(e)
            }
        }
        fn intersection(
            a: &[crate::geometry::ExPolygon],
            b: &[crate::geometry::ExPolygon],
        ) -> crate::geometry::ExPolygons {
            if faithful() {
                crate::clipper_utils::intersection_clib(a, b)
            } else {
                crate::clipper_utils::intersection(a, b)
            }
        }
        fn difference(
            a: &[crate::geometry::ExPolygon],
            b: &[crate::geometry::ExPolygon],
        ) -> crate::geometry::ExPolygons {
            if faithful() {
                crate::clipper_utils::difference_clib(a, b)
            } else {
                crate::clipper_utils::difference(a, b)
            }
        }
        use crate::geometry::ExPolygons;
        use crate::region_config::EnsureVerticalThicknessLevel;
        use crate::surface::SurfaceType;

        const TOP_BOTTOM_EXPANSION_COEFF: f64 = 0.05; // PrintObject.cpp:1758
        let sf = crate::SCALING_FACTOR; // mm -> scaled (100_000); area is scaled^2
        let eps_mm = crate::libslic3r::EPSILON; // 1e-4

        // PerimeterGenerator/Layer feed this; spiral mode clamps the layer count.
        let num_layers = self.layers.len();
        if num_layers == 0 {
            return Ok(());
        }

        // Collected polygons, offsetted. PrintObject.cpp:1740-1745
        struct Cache {
            top: ExPolygons,
            bottom: ExPolygons,
            holes: ExPolygons,
        }

        // PrintObject.cpp:1759 — for a multi-material print with interface_shells disabled
        // the vertical shell thickness is calculated over ALL materials merged, so the
        // per-layer cache is built ONCE (top/bottom unioned across every region, plus the
        // merged perimeter shadow) and reused unchanged by every region_id below.
        // Single-region prints (and interface_shells) keep the per-region cache path.
        let top_bottom_surfaces_all_regions =
            self.num_printing_regions() > 1 && !self.config.interface_shells;
        let shared_cache: Option<Vec<Cache>> = if top_bottom_surfaces_all_regions {
            // PrintObject.cpp:1763-1772 — "ensure vertical wall thickness" applies to no
            // region at all: quit.
            let has_extra_layers = (0..self.num_printing_regions()).any(|rid| {
                self.shared_regions
                    .as_ref()
                    .and_then(|r| r.all_regions.get(rid))
                    .map(|r| {
                        r.config().ensure_vertical_shell_thickness
                            != EnsureVerticalThicknessLevel::Disabled
                    })
                    .unwrap_or(false)
            });
            if !has_extra_layers {
                return Ok(());
            }
            // VSHELL_ALL_REGIONS_F32 (on): the offsets of the all-regions cache are
            // `float` expressions over the scaled integer widths and spacings
            // (PrintObject.cpp:2351-2382), as the per-region cache already has them
            // (VSHELL_REG_QUANT), and the perimeter shadow is one `offset2` of the
            // lslices. =0 computes them in double millimetres and offsets in two steps.
            let all_regions_f32 = faithful() && crate::faithful_gate("VSHELL_ALL_REGIONS_F32");
            // `Flow::scaled_width()` / `scaled_spacing()`: coord_t(scale_(v)).
            let scaled_int = |v: f64| -> f32 { (v / 0.00001).trunc() as f32 };
            // PrintObject.cpp:1777-1820 — per layer, over all regions.
            let mut out: Vec<Cache> = Vec::with_capacity(num_layers);
            for layer in self.layers.iter() {
                if self.canceled.load(std::sync::atomic::Ordering::Relaxed) {
                    return Err(crate::Error::Cancelled);
                }
                let lh = layer.height;
                let mut top: ExPolygons = Vec::new();
                let mut bottom: ExPolygons = Vec::new();
                let mut holes: ExPolygons = Vec::new();
                // Simulate a single set of perimeters over all merged regions.
                // PrintObject.cpp:1782-1783
                let mut perimeter_offset: f64 = 0.0;
                let mut perimeter_min_spacing: f64 = f64::MAX;
                // The same two in `float`, scaled (PrintObject.cpp:2351-2352).
                let mut perimeter_offset_f: f32 = 0.0;
                let mut perimeter_min_spacing_f: f32 = f32::MAX;
                for (rid, lr) in layer.regions().iter().enumerate() {
                    // PrintObject.cpp:1786
                    let exp = if all_regions_f32 {
                        // :2355 float(scaled_spacing()) * top_bottom_expansion_coeff
                        ((scaled_int(lr.flow(FlowRole::SolidInfill, lh)?.spacing()) * 0.05f32) as f64)
                            / crate::SCALING_FACTOR
                    } else {
                        lr.flow(FlowRole::SolidInfill, lh)?.spacing() * TOP_BOTTOM_EXPANSION_COEFF
                    };
                    // PrintObject.cpp:1788-1790 — top / bottom surfaces, APPENDED (not
                    // overwritten as in the per-region path).
                    let top_eps: ExPolygons = lr
                        .slices
                        .filter_by_type(SurfaceType::Top)
                        .iter()
                        .map(|s| s.expolygon.clone())
                        .collect();
                    let bot_eps: ExPolygons = lr
                        .slices
                        .filter_by_types(&[SurfaceType::Bottom, SurfaceType::BottomBridge])
                        .iter()
                        .map(|s| s.expolygon.clone())
                        .collect();
                    if !top_eps.is_empty() {
                        top.extend(grow(&top_eps, exp, OffsetJoinType::Miter));
                    }
                    if !bot_eps.is_empty() {
                        bottom.extend(grow(&bot_eps, exp, OffsetJoinType::Miter));
                    }
                    // PrintObject.cpp:1791-1794 — perimeters = max(extra_perimeters over
                    // this region's slices) + wall_loops.
                    let mut perimeters: u32 = 0;
                    for s in lr.slices.surfaces.iter() {
                        perimeters = perimeters.max(s.extra_perimeters as u32);
                    }
                    perimeters += self
                        .shared_regions
                        .as_ref()
                        .and_then(|r| r.all_regions.get(rid))
                        .map(|r| r.config().perimeters)
                        .unwrap_or(0);
                    // PrintObject.cpp:1795-1802 — widest simulated perimeter band.
                    if perimeters > 0 {
                        let extflow = lr.flow(FlowRole::ExternalPerimeter, lh)?;
                        let pflow = lr.flow(FlowRole::Perimeter, lh)?;
                        perimeter_offset = perimeter_offset.max(
                            0.5 * (extflow.width() + extflow.spacing())
                                + (perimeters as f64 - 1.0) * pflow.spacing(),
                        );
                        perimeter_min_spacing =
                            perimeter_min_spacing.min(extflow.spacing().min(pflow.spacing()));
                        // :2374-2376
                        perimeter_offset_f = perimeter_offset_f.max(
                            0.5f32 * (scaled_int(extflow.width()) + scaled_int(extflow.spacing()))
                                + (perimeters as f32 - 1.0f32) * scaled_int(pflow.spacing()),
                        );
                        perimeter_min_spacing_f = perimeter_min_spacing_f
                            .min(scaled_int(extflow.spacing()).min(scaled_int(pflow.spacing())));
                    }
                    // PrintObject.cpp:1803
                    holes.extend(lr.fill_expolygons.iter().cloned());
                }
                // PrintObject.cpp:1805-1816 — simulate the perimeter / infill split as if
                // only a single extruder had printed the whole layer: grow lslices to force
                // the per-region islands to merge, then shrink by the widest perimeter band.
                // NOTE: C++ `offset2(a, +d1, -d2)` is GROW-then-SHRINK, the opposite order
                // to `clipper_utils::offset2`, hence the explicit grow()/shrink() pair.
                if all_regions_f32 {
                    if perimeter_offset_f > 0.0 {
                        // :2382 offset2(layer.lslices, 0.3f * perimeter_min_spacing,
                        //               -perimeter_offset - 0.3f * perimeter_min_spacing)
                        let d1 = 0.3f32 * perimeter_min_spacing_f;
                        let d2 = -perimeter_offset_f - 0.3f32 * perimeter_min_spacing_f;
                        holes.extend(crate::clipper_utils::offset2_ex_clib(
                            &layer.lslices,
                            d1 as f64 / crate::SCALING_FACTOR,
                            d2 as f64 / crate::SCALING_FACTOR,
                            OffsetJoinType::Miter,
                        ));
                    }
                } else if perimeter_offset > 0.0 {
                    let pad = 0.3 * perimeter_min_spacing;
                    let grown = grow(&layer.lslices, pad, OffsetJoinType::Miter);
                    if !grown.is_empty() {
                        holes.extend(shrink(&grown, perimeter_offset + pad, OffsetJoinType::Miter));
                    }
                }
                // PrintObject.cpp:1818-1820 — reduce the polygon count.
                out.push(Cache {
                    top: if top.is_empty() { top } else { union_ex(&top) },
                    bottom: if bottom.is_empty() { bottom } else { union_ex(&bottom) },
                    holes: if holes.is_empty() { holes } else { union_ex(&holes) },
                });
            }
            Some(out)
        } else {
            None
        };

        if crate::probe_enabled("VSHELL_DEBUG") {
            eprintln!("VSHELL_REGIONS: num_printing_regions={} layers={}",
                      self.num_printing_regions(), self.layers.len());
        }
        // PrintObject.cpp:1827 — per region.
        for region_id in 0..self.num_printing_regions() {
            // PrintObject.cpp:1830
            // C++: const PrintRegionConfig &region_config = this->printing_region(region_id).config();
            // Cloning the Arc (not the config) keeps the shared snapshot alive
            // across the mutable layer accesses below.
            let region = self
                .shared_regions
                .as_ref()
                .and_then(|r| r.all_regions.get(region_id))
                .cloned()
                .expect("shared_regions covers all printing regions");
            let rc = region.config();
            // PrintObject.cpp:1830-1832 — evtDisabled regions are handled by discover_horizontal_shells.
            if rc.ensure_vertical_shell_thickness == EnsureVerticalThicknessLevel::Disabled {
                continue;
            }
            let n_top = rc.top_solid_layers as usize;
            let n_bottom = rc.bottom_solid_layers as usize;
            let top_thick = rc.top_solid_min_thickness;
            let bot_thick = rc.bottom_solid_min_thickness;
            let is_partial = rc.ensure_vertical_shell_thickness == EnsureVerticalThicknessLevel::Partial;

            // --- Build per-layer cache (top_surfaces, bottom_surfaces, holes) from SLICES + fill_expolygons.
            // PrintObject.cpp:1831-1859 — skipped entirely when the shared all-regions
            // cache was already built above (C++ `if (! top_bottom_surfaces_all_regions)`).
            let build_owned = shared_cache.is_none();
            let mut cache_owned: Vec<Cache> =
                Vec::with_capacity(if build_owned { num_layers } else { 0 });
            let mut lslices_all: Vec<ExPolygons> = Vec::with_capacity(num_layers);
            let mut solid_spacing_mm: Vec<f64> = Vec::with_capacity(num_layers);
            let mut ext_spacing_mm: Vec<f64> = Vec::with_capacity(num_layers);
            for layer in self.layers.iter() {
                lslices_all.push(layer.lslices.clone());
                let lh = layer.height;
                let lr = match layer.regions().get(region_id) {
                    Some(r) => r,
                    None => {
                        if build_owned {
                            cache_owned.push(Cache { top: vec![], bottom: vec![], holes: vec![] });
                        }
                        solid_spacing_mm.push(0.45);
                        ext_spacing_mm.push(0.45);
                        continue;
                    }
                };
                // C++: layerm->flow(frSolidInfill) / flow(frExternalPerimeter)
                // (region/object/print configs read off the LayerRegion's Arcs;
                // lr.layer_id supplies the first-layer gate internally).
                let solid_flow = lr.flow(FlowRole::SolidInfill, lh)?;
                let ext_flow = lr.flow(FlowRole::ExternalPerimeter, lh)?;
                let sp = solid_flow.spacing();
                solid_spacing_mm.push(sp);
                // R280 (faithful): native deltas are f32 exprs on f32(coord_t
                // scaled_spacing) — trunc(v/1e-5) int → f32 arithmetic
                // (PrintObject.cpp:1793/1850 top_bottom_expansion = f32(S)*0.05f;
                // :1945 expand-by-ext = f32(S_ext)).
                if faithful() && crate::faithful_gate("VSHELL_REG_QUANT") {
                    let e_int = (ext_flow.spacing() / 0.00001).trunc();
                    ext_spacing_mm.push((e_int as f32 as f64) / crate::SCALING_FACTOR);
                } else {
                    ext_spacing_mm.push(ext_flow.spacing());
                }
                if !build_owned {
                    // The all-regions cache built before this loop already covers every
                    // region; C++ likewise skips this whole per-region cache pass.
                    continue;
                }
                // PrintObject.cpp:1850 — top_bottom_expansion = scaled_spacing * 0.05 (mm here).
                let exp = if faithful() && crate::faithful_gate("VSHELL_REG_QUANT") {
                    (((sp / 0.00001).trunc() as f32 * 0.05f32) as f64) / crate::SCALING_FACTOR
                } else {
                    sp * TOP_BOTTOM_EXPANSION_COEFF
                };
                let top_eps: ExPolygons = lr
                    .slices
                    .filter_by_type(SurfaceType::Top)
                    .iter()
                    .map(|s| s.expolygon.clone())
                    .collect();
                let bot_eps: ExPolygons = lr
                    .slices
                    .filter_by_types(&[SurfaceType::Bottom, SurfaceType::BottomBridge])
                    .iter()
                    .map(|s| s.expolygon.clone())
                    .collect();
                let top = if top_eps.is_empty() { vec![] } else { grow(&top_eps, exp, OffsetJoinType::Miter) };
                let bottom = if bot_eps.is_empty() { vec![] } else { grow(&bot_eps, exp, OffsetJoinType::Miter) };
                // holes = union of all regions' fill_expolygons on this layer (PrintObject.cpp:1852-1856).
                // The perimeter-shadow offset2(lslices,...) holes term (PrintObject.cpp:1815) lives ONLY
                // in the multi-material `top_bottom_surfaces_all_regions` cache loop above, which is
                // gated `if (num_printing_regions() > 1 && !interface_shells)` (PrintObject.cpp:1759) —
                // so on this path holes = fill_expolygons only, exactly as C++ does.
                let mut holes: ExPolygons = Vec::new();
                for r in layer.regions() {
                    holes.extend(r.fill_expolygons.iter().cloned());
                }
                // Native cache.holes are RAW appended polygons (no union at build,
                // PO.cpp:1859-64); the union here merges/snaps razor vertices the
                // native combine_holes intersection chain would see raw. Gated.
                let holes = if holes.is_empty() || std::env::var("VSHELL_RAW").is_ok() {
                    holes
                } else {
                    union_ex(&holes)
                };
                cache_owned.push(Cache { top, bottom, holes });
            }
            // C++ keeps one `cache_top_botom_regions` vector, filled either by the
            // all-regions pre-pass or by the per-region pass above.
            let cache: &[Cache] = shared_cache.as_deref().unwrap_or(&cache_owned);

            // --- Per layer: project, trim, regularize, convert. PrintObject.cpp:1880-2091
            // R393: the per-layer shell projection is independent across layers
            // (it reads only the pre-built `cache`/`lslices_all` snapshots and the
            // neighbour print_z/bottom_z, and writes only its own layer's region).
            // Snapshot the neighbour z's, compute all layers in parallel (rayon),
            // then apply the cheap surface reassignment serially. Same math, same
            // per-layer order → byte-identical output; ~8-core instead of 1.
            let print_zs: Vec<f64> = self.layers.iter().map(|l| l.print_z).collect();
            let bottom_zs: Vec<f64> = self.layers.iter().map(|l| l.bottom_z()).collect();
            let canceled = self.canceled.clone();
            let layers_ref = &self.layers;
            let vshell_outputs: Result<Vec<Option<(ExPolygons, ExPolygons, ExPolygons)>>> = {
                use rayon::prelude::*;
                (0..num_layers)
                    .into_par_iter()
                    .map(|idx| -> Result<Option<(ExPolygons, ExPolygons, ExPolygons)>> {
                if canceled.load(std::sync::atomic::Ordering::Relaxed) {
                    return Err(crate::Error::Cancelled);
                }
                // Native: min_perimeter_infill_spacing = float(infill_line_spacing) * 1.05f
                // (PrintObject.cpp:1905) — f32 on the scaled int; keep the f32 for the
                // radius/threshold exprs below.
                let min_pis_sc: f32 = ((solid_spacing_mm[idx] / 0.00001).trunc() as f32) * 1.05f32;
                let min_pis = if faithful() && crate::faithful_gate("VSHELL_REG_QUANT") {
                    (min_pis_sc as f64) / crate::SCALING_FACTOR
                } else {
                    solid_spacing_mm[idx] * 1.05 // min_perimeter_infill_spacing (mm)
                };

                let mut shell: ExPolygons = Vec::new();
                let mut holes: ExPolygons = cache[idx].holes.clone();

                // combine_shells: union accumulate. combine_holes: intersect (empty either -> empty).
                macro_rules! combine_shells {
                    ($s2:expr) => {{
                        let s2 = $s2;
                        if shell.is_empty() {
                            shell = s2;
                        } else if !s2.is_empty() {
                            shell.extend(s2);
                            shell = union_ex(&shell);
                        }
                    }};
                }
                macro_rules! combine_holes {
                    ($h2:expr) => {{
                        let h2 = $h2;
                        if holes.is_empty() || h2.is_empty() {
                            holes = vec![];
                        } else {
                            holes = intersection(&holes, &h2);
                        }
                    }};
                }

                // TOP projection (PrintObject.cpp:1924-1954)
                if n_top > 0 {
                    let print_z = print_zs[idx];
                    let itop = idx + n_top;
                    let mut i = idx + 1;
                    let mut any = false;
                    while i < cache.len()
                        && (i < itop || print_zs[i] - print_z < top_thick - eps_mm)
                    {
                        any = true;
                        if !is_partial {
                            combine_holes!(cache[i].holes.clone());
                        }
                        combine_shells!(cache[i].top.clone());
                        i += 1;
                    }
                    if !any && i < cache.len() {
                        // anchor special-case (PrintObject.cpp:1940-1948)
                        let grown = grow(&cache[idx].top, ext_spacing_mm[idx], OffsetJoinType::Miter);
                        combine_shells!(intersection(&grown, &lslices_all[i]));
                    }
                }
                // BOTTOM projection (PrintObject.cpp:1955-1983)
                if n_bottom > 0 {
                    let bottom_z = bottom_zs[idx];
                    let ibottom = idx as i64 - n_bottom as i64;
                    let mut i = idx as i64 - 1;
                    let mut any = false;
                    while i >= 0
                        && (i > ibottom
                            || bottom_z - bottom_zs[i as usize] < bot_thick - eps_mm)
                    {
                        any = true;
                        if !is_partial {
                            combine_holes!(cache[i as usize].holes.clone());
                        }
                        combine_shells!(cache[i as usize].bottom.clone());
                        i -= 1;
                    }
                    if !any && i >= 0 {
                        let grown = grow(&cache[idx].bottom, ext_spacing_mm[idx], OffsetJoinType::Miter);
                        combine_shells!(intersection(&grown, &lslices_all[i as usize]));
                    }
                }

                // VSHELLDBG (diagnostics only, env-gated)
                if std::env::var("VSHELLDBG").is_ok() {
                    let area = |e: &ExPolygons| e.iter().map(|p| p.area().abs()).sum::<f64>();
                    eprintln!(
                        "VSHELLDBG L{} own_holes n={} a={:.3e} comb_holes n={} a={:.3e} shell n={} a={:.3e}",
                        idx,
                        cache[idx].holes.len(),
                        area(&cache[idx].holes),
                        holes.len(),
                        area(&holes),
                        shell.len(),
                        area(&shell),
                    );
                }

                // polygonsInternal = fill_surfaces filtered to {Internal, InternalVoid, InternalSolid}
                // (PrintObject.cpp:1992). Read the current region's fill_surfaces (immutable).
                let (internal_all, internal_only, void_only, solid_only) = {
                    let fs = &layers_ref[idx].regions()[region_id].fill_surfaces;
                    let pick = |types: &[SurfaceType]| -> ExPolygons {
                        fs.filter_by_types(types).iter().map(|s| s.expolygon.clone()).collect()
                    };
                    (
                        pick(&[SurfaceType::Internal, SurfaceType::InternalVoid, SurfaceType::InternalSolid]),
                        pick(&[SurfaceType::Internal]),
                        pick(&[SurfaceType::InternalVoid]),
                        pick(&[SurfaceType::InternalSolid]),
                    )
                };

                // PrintObject.cpp:1993-1996 — trim shell to internal, plus internal-not-holes.
                let shell_int = if shell.is_empty() || internal_all.is_empty() {
                    vec![]
                } else if faithful() {
                    // R296: native is intersection(shell, polygonsInternal,
                    // ApplySafetyOffset::Yes) (PO.cpp:1993) — the CLIP gets the
                    // +10-unit raw safety offset before clipping. Rust's plain
                    // intersection was the exact +10-unit piece-1 signature
                    // (SHDBG R295).
                    crate::clipper_utils::intersection_clib_safety(&shell, &internal_all)
                } else {
                    intersection(&shell, &internal_all)
                };
                // PrintObject.cpp:1994 — polygons_append(shell, diff(polygonsInternal, holes)).
                // FORENSIC (vshells-holes, /tmp/holes_findings.md): at li=70/Z14.2 this produces a
                // 54 mm^2 central/right blob (dih#0, x=[5.36,26.77]) that native does NOT have, which
                // becomes a spurious InternalSolid and is reclassified to an over-wide Bridge
                // downstream. The op itself is faithful (verified: routing through difference_clib at
                // ClipperLib precision yields the identical 54 mm^2 — so this is NOT an F1-local diff
                // bug). The blob is REAL given the inputs: it is internal_all minus the window-combined
                // `holes`, and `holes` is over-carved because rust's WINDOW fill_expolygons (L71..L74)
                // are shape-divergent/fragmented vs native (L74 own holes fragment to n=9 with sub-mm
                // gaps). Root is UPSTREAM fill_expolygons shape (detect_surfaces_type / perimeter->infill
                // partition / upstream union F1 fragmentation), not this op — see findings doc.
                //
                // R485 NARROWS THAT: the "upstream fill_expolygons fragmentation" root does
                // NOT hold globally on Majora. Counting per-layer islands in the emitted
                // gcode (3mm centroid clustering) against the C++ reference:
                //     Sparse infill            rust 10,180 vs bambu 10,313 = 0.987
                //     Internal solid infill    rust  5,401 vs bambu  5,825 = 0.927
                //     Floating vertical shell  rust  5,117 vs bambu  6,363 = 0.804
                // Sparse is filled from the SAME fill region these `holes` derive from, and
                // it matches to 1.3%. If fill_expolygons were broadly shape-divergent or
                // over-fragmented, sparse would diverge too. It does not. What diverges is
                // only the SOLID/SPARSE SPLIT — how much Internal gets promoted to
                // InternalSolid — and it diverges WORST on the narrow (FVS) subset, i.e. we
                // create ~1,534 fewer, not smaller, solid pieces (mean island size matches:
                // 18.21 vs 17.80mm, R482). The scattered-drop filter below is not
                // responsible either (R483: disabling it entirely leaves FVS at 0.863).
                // So the remaining suspect is the size of `shell` / `holes` themselves in
                // this promotion step, not the fill region that feeds them.
                let diff_int_holes = if holes.is_empty() {
                    internal_all.clone()
                } else {
                    difference(&internal_all, &holes)
                };
                if crate::probe_enabled("VSHELL_DEBUG") {
                    use std::sync::atomic::Ordering::Relaxed;
                    let a = |v: &ExPolygons| -> usize {
                        (v.iter().map(|e| e.area().abs()).sum::<f64>() / (sf * sf) * 1000.0) as usize
                    };
                    crate::layer::VS_T_SHELLINT.fetch_add(a(&shell_int), Relaxed);
                    crate::layer::VS_T_DIFF.fetch_add(a(&diff_int_holes), Relaxed);
                    crate::layer::VS_T_SOLID.fetch_add(a(&solid_only), Relaxed);
                    crate::layer::VS_T_INTERNAL.fetch_add(a(&internal_all), Relaxed);
                    crate::layer::VS_T_N.fetch_add(1, Relaxed);
                    if holes.is_empty() { crate::layer::VS_T_HOLES_EMPTY.fetch_add(1, Relaxed); }
                    else { crate::layer::VS_T_HOLES.fetch_add(a(&holes), Relaxed); }
                    if shell_int.is_empty() { crate::layer::VS_T_SHELLINT_EMPTY.fetch_add(1, Relaxed); }
                }
                let mut new_shell = shell_int;
                new_shell.extend(diff_int_holes);
                if new_shell.is_empty() {
                    return Ok(None);
                }
                // PrintObject.cpp:1999 — append existing internal-solid so they merge.
                new_shell.extend(solid_only.clone());
                let shell_u = union_ex(&new_shell);
                // PrintObject.cpp:2007-2055 — regularize (open then close), then drop scattered tiny bits.
                // Native radii are f32 chains on min_pis_sc (PrintObject.cpp:2012-2017).
                let (narrow_wall_r, narrow_sparse_r, tiny_overlap_r) = if faithful()
                    && crate::faithful_gate("VSHELL_REG_QUANT")
                {
                    (
                        ((0.5f32 * 0.65f32 * min_pis_sc) as f64) / crate::SCALING_FACTOR,
                        ((0.5f32 * 1.2f32 * min_pis_sc) as f64) / crate::SCALING_FACTOR,
                        ((0.2f32 * min_pis_sc) as f64) / crate::SCALING_FACTOR,
                    )
                } else {
                    (0.5 * 0.65 * min_pis, 0.5 * 1.2 * min_pis, 0.2 * min_pis)
                };
                // R239 (gated): native regularized_shell = shrink_ex(offset2_ex(
                // union_ex(shell), -r1, r1+r2, jtSquare), r2-tiny, jtSquare)
                // (PrintObject.cpp:2018-2024) — run it at ClipperLib precision;
                // the geo route re-grids and reshapes the ISI contour.
                let regularized0 = if crate::faithful_gate("TOPFILL_FAITHFUL") {
                    let opened = crate::clipper_utils::offset2_ex_clib(
                        &shell_u,
                        -narrow_wall_r,
                        narrow_wall_r + narrow_sparse_r,
                        OffsetJoinType::Square,
                    );
                    crate::clipper_utils::offset_expolygons_clib_scaled(
                        &opened,
                        -(narrow_sparse_r - tiny_overlap_r) * crate::SCALING_FACTOR,
                        OffsetJoinType::Square,
                    )
                } else {
                    let opened = offset2(
                        &shell_u,
                        narrow_wall_r,
                        narrow_wall_r + narrow_sparse_r,
                        OffsetJoinType::Square,
                    );
                    shrink(&opened, narrow_sparse_r - tiny_overlap_r, OffsetJoinType::Square)
                };

                if crate::probe_enabled("VSHELL_DEBUG") {
                    use std::sync::atomic::Ordering::Relaxed;
                    let ain: f64 = shell_u.iter().map(|e| e.area().abs()).sum::<f64>() / (sf * sf);
                    let aout: f64 = regularized0.iter().map(|e| e.area().abs()).sum::<f64>() / (sf * sf);
                    crate::layer::VS_SHELL_IN.fetch_add(shell_u.len(), Relaxed);
                    crate::layer::VS_SHELL_IN_AREA.fetch_add((ain * 1000.0) as usize, Relaxed);
                    crate::layer::VS_REG_OUT.fetch_add(regularized0.len(), Relaxed);
                    crate::layer::VS_REG_OUT_AREA.fetch_add((aout * 1000.0) as usize, Relaxed);
                }

                // object_volume = intersection(lslices[idx-1], lslices[idx+1]); internal_volume = closing(internal).
                let object_volume = if idx > 0 && idx + 1 < lslices_all.len() {
                    intersection(&lslices_all[idx - 1], &lslices_all[idx + 1])
                } else {
                    vec![]
                };
                let internal_volume = if internal_all.is_empty() {
                    vec![]
                } else {
                    closing(&internal_all, eps_mm, OffsetJoinType::Miter)
                };
                // Native: p.area() < min_pis_f32 * scaled(1.5) — f32 * coord_t(150000)
                // in FLOAT, promoted to double for the compare (PrintObject.cpp:2047-2048).
                let (thr15, thr8) = if faithful() && crate::faithful_gate("VSHELL_REG_QUANT") {
                    (
                        (min_pis_sc * 150_000f32) as f64,
                        (min_pis_sc * 800_000f32) as f64,
                    )
                } else {
                    (1.5 * min_pis * sf * sf, 8.0 * min_pis * sf * sf)
                };
                let regularized: ExPolygons = regularized0
                    .into_iter()
                    .filter(|p| {
                        let a = p.area().abs();
                        let p1 = vec![p.clone()];
                        let cond1 = a < thr15
                            || (a < thr8 && difference(&p1, &object_volume).is_empty());
                        let grown_p = grow(&p1, min_pis, OffsetJoinType::Miter);
                        let cond2 =
                            difference(&internal_volume, &grown_p).len() >= internal_volume.len();
                        if crate::probe_enabled("VSHELL_DEBUG") {
                            use std::sync::atomic::Ordering::Relaxed;
                            crate::layer::VS_REG_IN.fetch_add(1, Relaxed);
                            if cond1 { crate::layer::VS_C1.fetch_add(1, Relaxed); }
                            if a < thr15 { crate::layer::VS_C1_TINY.fetch_add(1, Relaxed); }
                            if cond2 { crate::layer::VS_C2.fetch_add(1, Relaxed); }
                            // cond2 SAVED the piece: cond1 wanted it gone, cond2 vetoed.
                            if cond1 && !cond2 { crate::layer::VS_SAVED.fetch_add(1, Relaxed); }
                            if cond1 && cond2 {
                                crate::layer::VS_REG_DROP.fetch_add(1, Relaxed);
                                crate::layer::VS_REG_DROP_AREA
                                    .fetch_add((a / (sf * sf) * 1000.0) as usize, Relaxed);
                            }
                        }
                        // R483 EXPERIMENT: VSHELL_DROP_FILTER=0 keeps every piece, to
                        // measure how much of the solid deficit this filter accounts for.
                        if !crate::faithful_gate("VSHELL_DROP_FILTER") {
                            return true;
                        }
                        !(cond1 && cond2) // keep if NOT removed
                    })
                    .collect();
                if regularized.is_empty() {
                    return Ok(None);
                }

                // PrintObject.cpp:2060,2075-2090 — reassign surfaces.
                let new_solid = intersection(&internal_all, &regularized);
                crate::stage_dump::dump("vshell_reg", idx, &regularized);
                crate::stage_dump::dump("vshell_solid", idx, &new_solid);
                let new_internal = difference(&internal_only, &regularized);
                let new_void = difference(&void_only, &regularized);

                Ok(Some((new_internal, new_void, new_solid)))
                    })
                    .collect()
            };

            // Serial apply of the per-layer results (cheap; preserves order).
            for (idx, out) in vshell_outputs?.into_iter().enumerate() {
                if let Some((new_internal, new_void, new_solid)) = out {
                    let fs = &mut self.layers[idx].regions_mut()[region_id].fill_surfaces;
                    fs.keep_types(&[SurfaceType::Top, SurfaceType::Bottom, SurfaceType::BottomBridge]);
                    fs.append(new_internal, SurfaceType::Internal);
                    fs.append(new_void, SurfaceType::InternalVoid);
                    fs.append(new_solid, SurfaceType::InternalSolid);
                }
            }
        }
        Ok(())
    }

    /// Discover horizontal shells - propagate top/bottom surfaces to neighbor layers
    /// PrintObject.cpp:3385-3560
    /// C++: void PrintObject::discover_horizontal_shells()
    fn discover_horizontal_shells(&mut self) -> Result<()> {
        use crate::clipper_utils::{
            difference, grow, opening, union_ex, OffsetJoinType,
        };
        use crate::geometry::{to_polygons, ExPolygon, ExPolygons, Polygons};
        use crate::surface::{Surface, SurfaceType};

        /// PrintObject.cpp:3387
        /// C++: BOOST_LOG_TRIVIAL(trace) << "discover_horizontal_shells()";

        /// PrintObject.cpp:3389-3556
        /// C++: for (size_t region_id = 0; region_id < this->num_printing_regions(); ++region_id) {
        for region_id in 0..self.num_printing_regions() {
            // PrintObject.cpp:3394: const PrintRegionConfig& region_config = layerm->region().config();
            // Threaded via shared_regions (indexed by region_id) instead of a
            // LayerRegion->region() back-pointer; identical for all layers of a
            // region. Cloning the Arc (not the config) keeps the shared
            // snapshot alive across the mutable layer accesses below.
            let region = self
                .shared_regions
                .as_ref()
                .and_then(|r| r.all_regions.get(region_id))
                .cloned()
                .expect("shared_regions covers all printing regions");
            let region_config = region.config();

            // PrintObject.cpp:3397-3399: if ensure_vertical_shell_thickness != evtDisabled,
            // the shell work was already performed by discover_vertical_shells(); skip the
            // whole region here (C++ does `continue;` per layer — equivalent, config is
            // identical across a region's layers).
            if region_config.ensure_vertical_shell_thickness
                != crate::region_config::EnsureVerticalThicknessLevel::Disabled
            {
                continue;
            }
            /// PrintObject.cpp:3390-3555
            /// C++: for (size_t i = 0; i < m_layers.size(); ++i) {
            for i in 0..self.layers.len() {
                /// PrintObject.cpp:3391
                /// C++: m_print->throw_if_canceled();
                if self.canceled.load(std::sync::atomic::Ordering::Relaxed) {
                    return Err(crate::Error::Cancelled);
                }

                /// PrintObject.cpp:3392-3395
                /// C++: Layer* layer = m_layers[i];
                /// C++: LayerRegion* layerm = layer->regions()[region_id];
                /// C++: const PrintRegionConfig& region_config = layerm->region().config();
                let print_z = self.layers[i].print_z;
                let bottom_z = self.layers[i].bottom_z();

                // Get region config - check if ensure_vertical_shell_thickness is disabled
                /// PrintObject.cpp:3396-3397
                /// C++: if (region_config.ensure_vertical_shell_thickness.value!=EnsureVerticalThicknessLevel::evtDisabled)
                /// C++:     continue;
                // TODO: Check ensure_vertical_shell_thickness from region config
                // For now, assume it's disabled so we process all regions

                /// PrintObject.cpp:3401-3554
                /// C++: for (size_t idx_surface_type = 0; idx_surface_type < 3; ++idx_surface_type) {
                for idx_surface_type in 0..3 {
                    /// PrintObject.cpp:3402
                    /// C++: m_print->throw_if_canceled();
                    if self.canceled.load(std::sync::atomic::Ordering::Relaxed) {
                        return Err(crate::Error::Cancelled);
                    }

                    /// PrintObject.cpp:3403-3404
                    /// C++: SurfaceType type = (idx_surface_type == 0) ? stTop : (idx_surface_type == 1) ? stBottom : stBottomBridge;
                    let surface_type = match idx_surface_type {
                        0 => SurfaceType::Top,
                        1 => SurfaceType::Bottom,
                        _ => SurfaceType::BottomBridge,
                    };

                    /// PrintObject.cpp:3405
                    /// C++: int num_solid_layers = (type == stTop) ? region_config.top_shell_layers.value : region_config.bottom_shell_layers.value;
                    let num_solid_layers = if surface_type == SurfaceType::Top {
                        region_config.top_solid_layers as i32
                    } else {
                        region_config.bottom_solid_layers as i32
                    };

                    /// PrintObject.cpp:3406-3407
                    /// C++: if (num_solid_layers == 0)
                    /// C++:     continue;
                    if num_solid_layers == 0 {
                        continue;
                    }

                    /// PrintObject.cpp:3421-3428
                    /// C++: Polygons solid;
                    /// C++: for (const Surface& surface : layerm->slices.surfaces)
                    /// C++:     if (surface.surface_type == type)
                    /// C++:         polygons_append(solid, to_polygons(surface.expolygon));
                    /// C++: for (const Surface& surface : layerm->fill_surfaces.surfaces)
                    /// C++:     if (surface.surface_type == type)
                    /// C++:         polygons_append(solid, to_polygons(surface.expolygon));
                    let mut solid = Polygons::new();

                    // Collect from slices
                    let mut slice_count = 0;
                    for surface in &self.layers[i].regions()[region_id].slices.surfaces {
                        if surface.surface_type == surface_type {
                            let polys = to_polygons(&[surface.expolygon.clone()]);
                            solid.extend(polys);
                            slice_count += 1;
                        }
                    }

                    // Collect from fill_surfaces
                    let mut fill_count = 0;
                    for surface in &self.layers[i].regions()[region_id].fill_surfaces.surfaces {
                        if surface.surface_type == surface_type {
                            let polys = to_polygons(&[surface.expolygon.clone()]);
                            solid.extend(polys);
                            fill_count += 1;
                        }
                    }

                    /// PrintObject.cpp:3429-3430
                    /// C++: if (solid.empty())
                    /// C++:     continue;
                    if solid.is_empty() {
                        continue;
                    }

                    /// PrintObject.cpp:3433-3439
                    /// C++: for (int n = (type == stTop) ? int(i) - 1 : int(i) + 1;
                    /// C++:     (type == stTop) ?
                    /// C++:     (n >= 0 && (int(i) - n < num_solid_layers ||
                    /// C++:         print_z - m_layers[n]->print_z < region_config.top_shell_thickness.value - EPSILON)) :
                    /// C++:     (n < int(m_layers.size()) && (n - int(i) < num_solid_layers ||
                    /// C++:         m_layers[n]->bottom_z() - bottom_z < region_config.bottom_shell_thickness.value - EPSILON));
                    /// C++:     (type == stTop) ? --n : ++n)
                    // C++: region_config.top_shell_thickness / bottom_shell_thickness
                    // (Rust field names are the legacy top/bottom_solid_min_thickness aliases)
                    let top_shell_thickness = region_config.top_solid_min_thickness;
                    let bottom_shell_thickness = region_config.bottom_solid_min_thickness;
                    let epsilon = 1e-4;

                    let mut n: i32 = if surface_type == SurfaceType::Top {
                        i as i32 - 1
                    } else {
                        i as i32 + 1
                    };

                    let mut neighbor_count = 0;
                    let _initial_n = if surface_type == SurfaceType::Top {
                        i as i32 - 1
                    } else {
                        i as i32 + 1
                    };

                    while if surface_type == SurfaceType::Top {
                        n >= 0
                            && ((i as i32 - n) < num_solid_layers as i32
                                || print_z - self.layers[n as usize].print_z
                                    < top_shell_thickness - epsilon)
                    } else {
                        n < self.layers.len() as i32
                            && ((n - i as i32) < num_solid_layers as i32
                                || self.layers[n as usize].bottom_z() - bottom_z
                                    < bottom_shell_thickness - epsilon)
                    } {
                        let n_usize = n as usize;
                        neighbor_count += 1;

                        /// PrintObject.cpp:3443
                        /// C++: LayerRegion* neighbor_layerm = m_layers[n]->regions()[region_id];

                        /// PrintObject.cpp:3455-3461
                        /// C++: Polygons new_internal_solid;
                        /// C++: {
                        /// C++:     Polygons internal;
                        /// C++:     for (const Surface& surface : neighbor_layerm->fill_surfaces.surfaces)
                        /// C++:         if (surface.surface_type == stInternal || surface.surface_type == stInternalSolid)
                        /// C++:             polygons_append(internal, to_polygons(surface.expolygon));
                        let mut internal = Polygons::new();
                        for surface in &self.layers[n_usize].regions()[region_id]
                            .fill_surfaces
                            .surfaces
                        {
                            if surface.surface_type == SurfaceType::Internal
                                || surface.surface_type == SurfaceType::InternalSolid
                            {
                                let polys = to_polygons(&[surface.expolygon.clone()]);
                                internal.extend(polys);
                            }
                        }

                        /// PrintObject.cpp:3462
                        /// C++: new_internal_solid = intersection(solid, internal, ApplySafetyOffset::Yes);
                        // R238 (gated): native applies ApplySafetyOffset::Yes —
                        // ctIntersection vs safety_offset(clip) (+10u).
                        let mut new_internal_solid = if crate::faithful_gate("TOPFILL_FAITHFUL") {
                            let subj_ex: Vec<crate::geometry::ExPolygon> =
                                solid.iter().map(|p| crate::geometry::ExPolygon::new(p.clone())).collect();
                            let clip_ex: Vec<crate::geometry::ExPolygon> =
                                internal.iter().map(|p| crate::geometry::ExPolygon::new(p.clone())).collect();
                            crate::clipper_utils::intersection_clib_safety(&subj_ex, &clip_ex)
                                .iter()
                                .flat_map(|ep| crate::geometry::to_polygons(&[ep.clone()]))
                                .collect()
                        } else {
                            polygons_intersection(&solid, &internal)
                        };

                        /// PrintObject.cpp:3464-3478
                        /// C++: if (new_internal_solid.empty()) {
                        if new_internal_solid.is_empty() {
                            /// PrintObject.cpp:3468-3477
                            /// C++: if (region_config.sparse_infill_density.value == 0 || region_config.ensure_vertical_shell_thickness.value == EnsureVerticalThicknessLevel::evtDisabled) {
                            /// C++:     goto EXTERNAL;
                            /// C++: } else {
                            /// C++:     continue;
                            /// C++: }
                            // C++ sparse_infill_density.value==0 ⟺ fill_density==0 (Rust stores a 0-1 fraction)
                        let sparse_infill_density = region_config.fill_density;
                            if sparse_infill_density == 0.0 {
                                // Hollow object - stop propagation
                                break; // equivalent to goto EXTERNAL
                            } else {
                                // Has infill - continue searching
                                n = if surface_type == SurfaceType::Top {
                                    n - 1
                                } else {
                                    n + 1
                                };
                                continue;
                            }
                        }

                        /// PrintObject.cpp:3480-3490
                        /// C++: if (region_config.sparse_infill_density.value == 0) {
                        /// C++:     float margin = float(neighbor_layerm->flow(frExternalPerimeter).scaled_width());
                        /// C++:     Polygons too_narrow = diff(
                        /// C++:         new_internal_solid,
                        /// C++:         opening(new_internal_solid, margin, margin + ClipperSafetyOffset, jtMiter, 5));
                        /// C++:     if (!too_narrow.empty())
                        /// C++:         new_internal_solid = solid = diff(new_internal_solid, too_narrow);
                        /// C++: }
                        // C++ sparse_infill_density.value==0 ⟺ fill_density==0 (Rust stores a 0-1 fraction)
                        let sparse_infill_density = region_config.fill_density;
                        if sparse_infill_density == 0.0 {
                            // PrintObject.cpp:3496
                            // C++: float margin = float(neighbor_layerm->flow(frExternalPerimeter).scaled_width());
                            // (opening() works in mm here; scaled_width()/scale == width().)
                            let neighbor_h = self.layers[n_usize].height;
                            let margin = self.layers[n_usize].regions()[region_id]
                                .flow(crate::flow::FlowRole::ExternalPerimeter, neighbor_h)?
                                .width();
                            let _clipper_safety = crate::libslic3r::SCALED_EPSILON; // ClipperSafetyOffset (unused: symmetric opening)

                            // Convert to ExPolygons for opening operation
                            let new_solid_expolys: Vec<ExPolygon> = new_internal_solid
                                .iter()
                                .map(|p| ExPolygon::new(p.clone()))
                                .collect();
                            let opened = if crate::faithful_gate("TOPFILL_FAITHFUL") {
                                // R238: native opening(x, margin, margin+ClipperSafetyOffset,
                                // jtMiter, 5) — ASYMMETRIC grow-back, limit 5.
                                crate::clipper_utils::offset2_ex_clib_miter(
                                    &new_solid_expolys,
                                    -margin,
                                    margin + 10.0 * crate::SCALING_FACTOR.recip(),
                                    OffsetJoinType::Miter,
                                    5.0,
                                )
                            } else {
                                opening(&new_solid_expolys, margin, OffsetJoinType::Miter)
                            };

                            // Convert back to polygons for diff
                            let opened_polys: Polygons = opened
                                .iter()
                                .flat_map(|ep| to_polygons(&[ep.clone()]))
                                .collect();
                            let too_narrow = polygons_diff(&new_internal_solid, &opened_polys);

                            if !too_narrow.is_empty() {
                                new_internal_solid =
                                    polygons_diff(&new_internal_solid, &too_narrow);
                                solid = new_internal_solid.clone();
                            }
                        }

                        /// PrintObject.cpp:3494-3524
                        /// C++: {
                        /// C++:     float margin = 3.f * layerm->flow(frSolidInfill).scaled_width();
                        /// C++:     Polygons too_narrow = diff(
                        /// C++:         new_internal_solid,
                        /// C++:         opening(new_internal_solid, margin, margin + ClipperSafetyOffset, ClipperLib::jtMiter, 5));
                        /// C++:     if (!too_narrow.empty()) {
                        /// C++:         Polygons internal;
                        /// C++:         for (const Surface& surface : neighbor_layerm->fill_surfaces.surfaces)
                        /// C++:             if (surface.is_internal() && !surface.is_bridge())
                        /// C++:                 polygons_append(internal, to_polygons(surface.expolygon));
                        /// C++:         polygons_append(new_internal_solid,
                        /// C++:             intersection(
                        /// C++:                 expand(too_narrow, +margin),
                        /// C++:                 internal));
                        /// C++:     }
                        /// C++: }
                        {
                            // PrintObject.cpp:3509
                            // C++: float margin = 3.f * layerm->flow(frSolidInfill).scaled_width();
                            // (opening()/grow() work in mm here; scaled_width()/scale == width().)
                            let layer_h = self.layers[i].height;
                            let margin = 3.0
                                * self.layers[i].regions()[region_id]
                                    .flow(crate::flow::FlowRole::SolidInfill, layer_h)?
                                    .width();

                            // Convert to ExPolygons for opening operation
                            let new_solid_expolys: Vec<ExPolygon> = new_internal_solid
                                .iter()
                                .map(|p| ExPolygon::new(p.clone()))
                                .collect();
                            let opened = if crate::faithful_gate("TOPFILL_FAITHFUL") {
                                // R238: asymmetric native opening, jtMiter limit 5
                                // (PrintObject.cpp:3512-3514).
                                crate::clipper_utils::offset2_ex_clib_miter(
                                    &new_solid_expolys,
                                    -margin,
                                    margin + 10.0 * crate::SCALING_FACTOR.recip(),
                                    OffsetJoinType::Miter,
                                    5.0,
                                )
                            } else {
                                opening(&new_solid_expolys, margin, OffsetJoinType::Miter)
                            };

                            // Convert back to polygons for diff
                            let opened_polys: Polygons = opened
                                .iter()
                                .flat_map(|ep| to_polygons(&[ep.clone()]))
                                .collect();
                            let too_narrow = polygons_diff(&new_internal_solid, &opened_polys);

                            if !too_narrow.is_empty() {
                                let mut internal_for_expand = Polygons::new();
                                for surface in &self.layers[n_usize].regions()[region_id]
                                    .fill_surfaces
                                    .surfaces
                                {
                                    if surface.is_internal() && !surface.is_bridge() {
                                        let polys = to_polygons(&[surface.expolygon.clone()]);
                                        internal_for_expand.extend(polys);
                                    }
                                }

                                // Expand too_narrow polygons
                                let too_narrow_expolys: Vec<ExPolygon> = too_narrow
                                    .iter()
                                    .map(|p| ExPolygon::new(p.clone()))
                                    .collect();
                                let expanded_expolys =
                                    grow(&too_narrow_expolys, margin, OffsetJoinType::Miter);
                                let expanded: Polygons = expanded_expolys
                                    .iter()
                                    .flat_map(|ep| to_polygons(&[ep.clone()]))
                                    .collect();

                                // Intersection
                                let intersection_result =
                                    polygons_intersection(&expanded, &internal_for_expand);
                                new_internal_solid.extend(intersection_result);
                            }
                        }

                        /// PrintObject.cpp:3527-3551
                        /// C++: SurfaceCollection backup = std::move(neighbor_layerm->fill_surfaces);
                        /// C++: polygons_append(new_internal_solid, to_polygons(backup.filter_by_type(stInternalSolid)));
                        /// C++: ExPolygons internal_solid = union_ex(new_internal_solid);
                        /// C++: neighbor_layerm->fill_surfaces.set(internal_solid, stInternalSolid);
                        /// C++: Polygons polygons_internal = to_polygons(std::move(internal_solid));
                        /// C++: ExPolygons internal = diff_ex(backup.filter_by_type(stInternal), polygons_internal, ApplySafetyOffset::Yes);
                        /// C++: neighbor_layerm->fill_surfaces.append(internal, stInternal);
                        /// C++: polygons_append(polygons_internal, to_polygons(std::move(internal)));
                        /// C++: backup.keep_types({ stTop, stBottom, stBottomBridge });
                        /// C++: std::vector<SurfacesPtr> top_bottom_groups;
                        /// C++: backup.group(&top_bottom_groups);
                        /// C++: for (SurfacesPtr& group : top_bottom_groups)
                        /// C++:     neighbor_layerm->fill_surfaces.append(
                        /// C++:         diff_ex(group, polygons_internal),
                        /// C++:         *group.front());
                        // Backup current fill_surfaces
                        let backup = self.layers[n_usize].regions()[region_id]
                            .fill_surfaces
                            .clone();

                        // Add existing internal solid to new_internal_solid
                        for surface in &backup.surfaces {
                            if surface.surface_type == SurfaceType::InternalSolid {
                                let polys = to_polygons(&[surface.expolygon.clone()]);
                                new_internal_solid.extend(polys);
                            }
                        }

                        // Union to create final internal_solid
                        let new_solid_expolys: Vec<ExPolygon> = new_internal_solid
                            .iter()
                            .map(|p| ExPolygon::new(p.clone()))
                            .collect();
                        let internal_solid_expolys = union_ex(&new_solid_expolys);

                        // Clear and rebuild fill_surfaces
                        self.layers[n_usize].regions_mut()[region_id]
                            .fill_surfaces
                            .surfaces
                            .clear();

                        // Add new internal solid surfaces
                        for expolygon in &internal_solid_expolys {
                            self.layers[n_usize].regions_mut()[region_id]
                                .fill_surfaces
                                .surfaces
                                .push(Surface::new(SurfaceType::InternalSolid, expolygon.clone()));
                        }

                        // Convert internal_solid back to polygons for subtraction
                        let mut polygons_internal = Polygons::new();
                        for ep in &internal_solid_expolys {
                            let polys = to_polygons(&[ep.clone()]);
                            polygons_internal.extend(polys);
                        }

                        // Create new internal surfaces (old internal minus new internal_solid)
                        let mut old_internal_expolys = Vec::new();
                        for surface in &backup.surfaces {
                            if surface.surface_type == SurfaceType::Internal {
                                old_internal_expolys.push(surface.expolygon.clone());
                            }
                        }
                        let internal_expolys =
                            difference(&old_internal_expolys, &internal_solid_expolys);

                        // Add new internal surfaces
                        for expolygon in &internal_expolys {
                            self.layers[n_usize].regions_mut()[region_id]
                                .fill_surfaces
                                .surfaces
                                .push(Surface::new(SurfaceType::Internal, expolygon.clone()));
                        }

                        // Update polygons_internal to include new internal
                        for ep in &internal_expolys {
                            let polys = to_polygons(&[ep.clone()]);
                            polygons_internal.extend(polys);
                        }

                        // PrintObject.cpp:3553-3561 — trim top/bottom/bottom-bridge surfaces
                        // by `polygons_internal`, which is the union of the new internal-solid
                        // AND the new internal surfaces (both appended above), not just the
                        // internal-solid. Build the ExPolygon clip set from polygons_internal.
                        let polygons_internal_ex: ExPolygons = polygons_internal
                            .iter()
                            .map(|p| ExPolygon::new(p.clone()))
                            .collect();
                        for surface in &backup.surfaces {
                            if surface.surface_type == SurfaceType::Top
                                || surface.surface_type == SurfaceType::Bottom
                                || surface.surface_type == SurfaceType::BottomBridge
                            {
                                let trimmed = difference(
                                    &[surface.expolygon.clone()],
                                    &polygons_internal_ex,
                                );
                                for expolygon in trimmed {
                                    let mut new_surface = surface.clone();
                                    new_surface.expolygon = expolygon;
                                    self.layers[n_usize].regions_mut()[region_id]
                                        .fill_surfaces
                                        .surfaces
                                        .push(new_surface);
                                }
                            }
                        }

                        // Move to next neighbor layer
                        n = if surface_type == SurfaceType::Top {
                            n - 1
                        } else {
                            n + 1
                        };
                    } // while neighbor layers
                } // for each surface type
            } // for each layer
        } // for each region

        Ok(())
    }

    /// Process external surfaces across all printing regions / layers.
    /// PrintObject.cpp:1661-1737
    /// C++: void PrintObject::process_external_surfaces()
    ///
    /// Faithful structure: iterate every printing region, and for each layer call
    /// `LayerRegion::process_external_surfaces`. The C++ also precomputes
    /// `surfaces_covered` (the void-trim mask for sparse-infill regions,
    /// PrintObject.cpp:1666-1718) and threads it as `lower_layer_covered`; the
    /// wave-expansion port does not yet consume that mask, so the covered-surface
    /// precompute is omitted here (the per-region member runs without it).
    fn process_external_surfaces(&mut self) -> Result<()> {
        // PrintObject.cpp:1661-1718 — pre-pass that builds `surfaces_covered`, the
        // per-layer extrusion-covered regions over which the surfaces one layer up
        // may expand. It is only ever non-empty when some printing region has
        // `sparse_infill_density == 0` (C++ `has_voids`); otherwise C++ passes
        // `nullptr` for the covered-surfaces argument on every layer.
        //
        // FIDELITY-NOTE: the `surfaces_covered`/`has_voids` machinery and the
        // `lower_layer` argument to LayerRegion::process_external_surfaces are not
        // threaded here because this crate's LayerRegion::process_external_surfaces
        // (region_expansion port) does not yet accept those arguments. For any
        // region with non-zero infill density this is exact (C++ would pass
        // nullptr); zero-infill regions lose the void-supported expansion clamp.
        let has_voids = (0..self.num_printing_regions())
            .any(|region_id| self.printing_region(region_id)
                .map(|r| r.config().fill_density == 0.0)
                .unwrap_or(false));
        let _ = has_voids; // see FIDELITY-NOTE above

        // PrintObject.cpp:1728 — one layer's work. Shared by the parallel and the
        // serial arm below so the two can never drift apart.
        fn one_layer(layer: &mut Layer, region_id: usize) -> Result<()> {
            // m_layers[layer_idx]->get_region(region_id)->process_external_surfaces(...)
            // layer.height mirrors C++ m_layer->height threaded into LayerRegion::flow.
            let layer_height = layer.height;
            if let Some(region) = layer.regions_mut().get_mut(region_id) {
                region.process_external_surfaces(layer_height)?;
            }
            Ok(())
        }

        // PESPROF (R736, default OFF): per-layer wall-clock for the loop below.
        // The stage total alone cannot say whether the cost is SPREAD over layers
        // (parallelism helps) or CONCENTRATED in a few (it cannot) — and the A/B
        // said parallelising changed nothing, so the distribution is the finding.
        let pesprof = crate::probe_enabled("PESPROF");
        let pes_times: std::sync::Mutex<Vec<(usize, usize, f64)>> =
            std::sync::Mutex::new(Vec::new());

        // R736: native wraps this layer loop in
        // `tbb::parallel_for(tbb::blocked_range<size_t>(0, m_layers.size()))`
        // (PrintObject.cpp:1722). This port kept the loop body and dropped the
        // parallelism — the same porting move that cost 41% of the slice in the
        // seam raycast (R735). Serial, it was 0.662 s of prepare_infill's 0.750 s
        // (87.9%) and 39% of benchy's whole 1.68 s slice.
        //
        // Chunking across layers is bit-identical by construction: `one_layer`
        // writes only `layers[i].regions[region_id].fill_surfaces` and reads only
        // that same region plus the Arc'd, immutable config hierarchy. There is no
        // reduction, no ordering, and no shared mutable state — `region_expansion`
        // holds no statics, no RNG and no interior mutability. Note this port is
        // MORE independent than native, which reads `m_layers[layer_idx - 1]` for
        // the void-trim path we do not implement (see the FIDELITY-NOTE above).
        //
        // PES_PARALLEL=0 restores the serial loop for A/B.
        let canceled = self.canceled.clone();
        let parallel = crate::faithful_gate("PES_PARALLEL");
        // PrintObject.cpp:1720 — for each printing region.
        for region_id in 0..self.num_printing_regions() {
            // PrintObject.cpp:1722-1731 — for each layer, drive the LayerRegion member.
            if parallel {
                use rayon::prelude::*;
                self.layers.par_iter_mut().enumerate().try_for_each(|(li, layer)| {
                    // PrintObject.cpp:1726 — C++: m_print->throw_if_canceled();
                    if canceled.load(Ordering::Relaxed) {
                        return Err(Error::Cancelled);
                    }
                    let t = std::time::Instant::now();
                    let r = one_layer(layer, region_id);
                    if pesprof {
                        pes_times.lock().unwrap().push(
                            (region_id, li, t.elapsed().as_secs_f64()));
                    }
                    r
                })?;
            } else {
                for (li, layer) in self.layers.iter_mut().enumerate() {
                    if canceled.load(Ordering::Relaxed) {
                        return Err(Error::Cancelled);
                    }
                    let t = std::time::Instant::now();
                    one_layer(layer, region_id)?;
                    if pesprof {
                        pes_times.lock().unwrap().push(
                            (region_id, li, t.elapsed().as_secs_f64()));
                    }
                }
            }
        }
        if pesprof {
            let mut v = pes_times.into_inner().unwrap();
            let total: f64 = v.iter().map(|x| x.2).sum();
            v.sort_by(|a, b| b.2.partial_cmp(&a.2).unwrap());
            let top: f64 = v.iter().take(10).map(|x| x.2).sum();
            eprintln!(
                "[PESPROF] records={} sum={:.3}s top10={:.3}s ({:.1}%) parallel={}",
                v.len(), total, top,
                if total > 0.0 { 100.0 * top / total } else { 0.0 }, parallel);
            for (r, l, t) in v.iter().take(10) {
                eprintln!("[PESPROF]   region={r} layer={l} {:.4}s", t);
            }
        }
        Ok(())
    }
}

// Helper functions for polygon operations that work with Vec<Polygon>
fn polygons_diff(subject: &[Polygon], clip: &[Polygon]) -> Polygons {
    use crate::clipper_utils::difference;
    use crate::geometry::{to_polygons, ExPolygon};

    let subject_expolys: Vec<ExPolygon> =
        subject.iter().map(|p| ExPolygon::new(p.clone())).collect();
    let clip_expolys: Vec<ExPolygon> = clip.iter().map(|p| ExPolygon::new(p.clone())).collect();

    let result_expolys = difference(&subject_expolys, &clip_expolys);
    result_expolys
        .iter()
        .flat_map(|ep| to_polygons(&[ep.clone()]))
        .collect()
}

fn polygons_intersection(subject: &[Polygon], clip: &[Polygon]) -> Polygons {
    use crate::clipper_utils::intersection;
    use crate::geometry::{to_polygons, ExPolygon};

    let subject_expolys: Vec<ExPolygon> =
        subject.iter().map(|p| ExPolygon::new(p.clone())).collect();
    let clip_expolys: Vec<ExPolygon> = clip.iter().map(|p| ExPolygon::new(p.clone())).collect();

    let result_expolys = intersection(&subject_expolys, &clip_expolys);
    result_expolys
        .iter()
        .flat_map(|ep| to_polygons(&[ep.clone()]))
        .collect()
}

impl Default for PrintObject {
    fn default() -> Self {
        Self::new()
    }
}


// =================================================================================================
// PrintObject.cpp on the Print.hpp classes of `print::faithful` (the C++ shape of PrintObject, its
// Print, its instances and regions). The live `PrintObject` above keeps the pipeline's own shape.
// =================================================================================================

/// PrintObject.cpp functions on [`crate::print::faithful::PrintObject`], in the order of the C++
/// file. A function that reaches the Print through `m_print` takes the Print (or, when the object
/// is borrowed out of the Print, the Print's step state and model).
pub mod faithful {
    use crate::config::{ConfigBase, ConfigOptionResolver};
    use crate::geometry::{ExPolygon, Point};
    use crate::libslic3r::{EPSILON, SCALING_FACTOR};
    use crate::model::{Model, ModelObject};
    use crate::print::faithful::{
        psCount, static_config_defaults, Print, PrintInstances, PrintObject, PrintObjectConfig, PrintObjectStep, PrintRegion, PrintRegionConfig,
        PrintStep,
    };
    use crate::print_base::{ApplyStatus, PrintBaseWithState};
    use crate::print_config::config_def::DynamicPrintConfig;
    use crate::print_config::enums::{BrimType, SupportType};
    use crate::slicing::SlicingParams;

    /// The Print a PrintObject reaches through `m_print` when it invalidates steps: the Print's
    /// step state, its full configuration and its cancel callback.
    pub type PrintStepState = PrintBaseWithState<PrintStep, { psCount }>;

    fn cfg_int(config: &DynamicPrintConfig, key: &str) -> i32 {
        config.option(key).and_then(|o| o.get_int().ok()).unwrap_or(0)
    }
    fn cfg_float(config: &DynamicPrintConfig, key: &str) -> f64 {
        *config.opt_float(key)
    }
    fn cfg_floats_or_empty(config: &DynamicPrintConfig, key: &str) -> Vec<f64> {
        if config.option(key).is_some() {
            crate::gcode::wipe_tower::cfg_floats(config, key)
        } else {
            Vec::new()
        }
    }
    fn sort_remove_duplicates<T: Ord>(v: &mut Vec<T>) {
        v.sort();
        v.dedup();
    }
    /// libslic3r.h `is_approx(value, test_value)` with EPSILON.
    fn is_approx(value: f64, test_value: f64) -> bool {
        (value - test_value).abs() < EPSILON
    }

    /// Print.cpp:459-466 `bool Print::invalidate_step(PrintStep step)` on the Print's step state:
    /// the step and, for every other step, the G-code export.
    pub fn print_invalidate_step(print: &mut PrintStepState, step: PrintStep) -> bool {
        let mut invalidated = print.invalidate_step(step);
        // Propagate to dependent steps.
        if step != PrintStep::psGCodeExport {
            invalidated |= print.invalidate_step(PrintStep::psGCodeExport);
        }
        invalidated
    }

    impl PrintObject {
        /// PrintObject.cpp:68-101 `PrintObject::PrintObject(Print* print, ModelObject* model_object, const
        /// Transform3d& trafo, PrintInstances&& instances)`: the centering offset, the size and the top
        /// of the object from the 1st instance, then `set_instances`.
        ///
        /// FIDELITY-NOTE: the instances point at their object by its index in `Print::m_objects`,
        /// which the caller knows once the object is placed (`usize::MAX` until then).
        pub fn new(print: &mut Print, model_object: usize, trafo: crate::geometry::geometry::Transform3d, instances: PrintInstances) -> PrintObject {
            use crate::geometry::geometry::eigen;
            let mut object = PrintObject::with_model_object(model_object, trafo, Vec::new());
            {
                let mo = &print.m_model.objects[model_object];
                // Snug bounding box of a rotated and scaled object by the 1st instantion, without the instance translation applied.
                let mut bbox = mo.raw_bounding_box().expect("PrintObject of a ModelObject without instances");
                let c = bbox.center();
                let mut bbox_center = eigen::Vec3d::new(c.x, c.y, c.z);

                // We may need to rotate the bbox / bbox_center from the original instance to the current instance.
                let z_diff = crate::geometry::geometry::rotation_diff_z(
                    &mo.instances[0].get_rotation(),
                    &mo.instances[instances[0].model_instance].get_rotation(),
                );
                if z_diff.abs() > EPSILON {
                    let z_rot = eigen::AngleAxisd::new(z_diff, eigen::Vec3d::new(0., 0., 1.)).to_rotation_matrix();
                    let t = eigen::transform_from_linear(&z_rot);
                    let mut rows = [[0.0f64; 4]; 4];
                    for (r, row) in rows.iter_mut().enumerate() {
                        for (col, v) in row.iter_mut().enumerate() {
                            *v = t[(r, col)];
                        }
                    }
                    bbox = bbox.transformed(&rows);
                    bbox_center = eigen::mat3_mul_vec(&z_rot, &bbox_center);
                }

                // Center of the transformed mesh (without translation): Point::new_scale, a truncation.
                object.m_center_offset = Point::new((bbox_center.x / SCALING_FACTOR) as i64, (bbox_center.y / SCALING_FACTOR) as i64);
                // Size of the transformed mesh. This bounding may not be snug in XY plane, but it is snug in Z.
                let size = bbox.size();
                let k = 1. / SCALING_FACTOR;
                object.m_size = [(size.x * k) as i64, (size.y * k) as i64, (size.z * k) as i64];
                // scaled<coord_t>(): a truncation, kept in the double member.
                object.m_max_z = ((mo.instance_bounding_box(0, false).max.z / SCALING_FACTOR) as i64) as f64;
            }
            object.set_instances(&mut print.base, instances, usize::MAX);
            object
        }

        /// PrintObject.cpp:110-130 `PrintBase::ApplyStatus PrintObject::set_instances(PrintInstances &&instances)`.
        /// `self_index` is the object's index in `Print::m_objects` (`i.print_object = this`).
        pub fn set_instances(&mut self, print: &mut PrintStepState, mut instances: PrintInstances, self_index: usize) -> ApplyStatus {
            for i in instances.iter_mut() {
                // Add the center offset, which will be subtracted from the mesh when slicing.
                i.shift = i.shift + self.m_center_offset;
            }
            // Invalidate and set copies.
            let mut status = ApplyStatus::ApplyStatusUnchanged;
            let equal_length = instances.len() == self.m_instances.len();
            let equal = equal_length
                && instances.iter().zip(self.m_instances.iter()).all(|(lhs, rhs)| lhs.model_instance == rhs.model_instance && lhs.shift == rhs.shift);
            if !equal {
                status = ApplyStatus::ApplyStatusChanged;
                if print.invalidate_steps([PrintStep::psWipeTower, PrintStep::psSkirtBrim, PrintStep::psGCodeExport])
                    || (!equal_length && print_invalidate_step(print, PrintStep::psWipeTower))
                {
                    status = ApplyStatus::ApplyStatusInvalidated;
                }
                self.m_instances = instances;
                for i in self.m_instances.iter_mut() {
                    i.print_object = self_index;
                }
            }
            status
        }

        /// PrintObject.cpp:1549-1877 `bool PrintObject::invalidate_state_by_config_options(const
        /// ConfigOptionResolver &old_config, const ConfigOptionResolver &new_config, const
        /// std::vector<t_config_option_key> &opt_keys)`. `model` is the Print's model (for
        /// `is_mm_painted()`).
        pub fn invalidate_state_by_config_options(
            &mut self,
            print: &mut PrintStepState,
            model: &Model,
            old_config: &DynamicPrintConfig,
            new_config: &DynamicPrintConfig,
            opt_keys: &[String],
        ) -> bool {
            use PrintObjectStep::*;
            if opt_keys.is_empty() {
                return false;
            }

            let mut steps: Vec<PrintObjectStep> = Vec::new();
            let mut invalidated = false;
            for opt_key in opt_keys {
                let k = opt_key.as_str();
                if matches!(
                    k,
                    "brim_width"
                        | "brim_object_gap"
                        | "brim_type"
                        // BBS: brim generation depends on printing speed
                        | "outer_wall_speed"
                        | "sparse_infill_speed"
                        | "inner_wall_speed"
                        | "support_speed"
                        | "internal_solid_infill_speed"
                        | "top_surface_speed"
                ) {
                    // Brim is printed below supports, support invalidates brim and skirt.
                    steps.push(posSupportMaterial);
                    if k == "brim_type" {
                        let old_brim_type = cfg_int(old_config, k);
                        let new_brim_type = cfg_int(new_config, k);
                        //BBS: When switch to manual brim, the object must have brim, then re-generate perimeter
                        //to make the wall order of first layer to be outer-first
                        if old_brim_type == BrimType::BtOuterOnly as i32 || new_brim_type == BrimType::BtOuterOnly as i32 {
                            steps.push(posPerimeters);
                        }
                    }
                } else if matches!(k, "periodic_modifier" | "periodic_modifier_skip_layers" | "periodic_modifier_apply_layers" | "modifier_ignore_infill") {
                    // Region assignment changes per layer when a modifier is skipped.
                    steps.push(posSlice);
                } else if matches!(
                    k,
                    "wall_loops"
                        | "top_one_wall_type"
                        | "top_area_threshold"
                        | "only_one_wall_first_layer"
                        | "initial_layer_line_width"
                        | "inner_wall_line_width"
                        | "infill_wall_overlap"
                        | "apply_scarf_seam_on_circles"
                ) {
                    steps.push(posPerimeters);
                } else if k == "gap_infill_speed" || k == "filter_out_gap_fill" {
                    // Return true if gap-fill speed has changed from zero value to non-zero or from non-zero value to zero.
                    let is_gap_fill_changed_state_due_to_speed = || -> bool {
                        if k == "gap_infill_speed" {
                            let old_gap_fill_speed = cfg_floats_or_empty(old_config, k);
                            let new_gap_fill_speed = cfg_floats_or_empty(new_config, k);
                            return old_gap_fill_speed.len() != new_gap_fill_speed.len() || old_gap_fill_speed != new_gap_fill_speed;
                        }
                        false
                    };
                    // Filtering of unprintable regions in multi-material segmentation depends on if gap-fill is enabled or not.
                    if model.objects[self.m_model_object].is_mm_painted()
                        && ((k == "gap_infill_speed" || k == "filter_out_gap_fill") && is_gap_fill_changed_state_due_to_speed())
                    {
                        steps.push(posSlice);
                    }
                    steps.push(posPerimeters);
                } else if matches!(
                    k,
                    "layer_height"
                        | "mmu_segmented_region_max_width"
                        | "mmu_segmented_region_interlocking_depth"
                        | "raft_layers"
                        | "raft_contact_distance"
                        | "slice_closing_radius"
                        | "slicing_mode"
                        | "interlocking_beam"
                        | "interlocking_orientation"
                        | "interlocking_beam_layer_count"
                        | "interlocking_depth"
                        | "interlocking_boundary_avoidance"
                        | "interlocking_beam_width"
                ) {
                    steps.push(posSlice);
                } else if matches!(
                    k,
                    "elefant_foot_compensation" | "support_top_z_distance" | "support_bottom_z_distance" | "xy_hole_compensation" | "xy_contour_compensation"
                ) {
                    steps.push(posSlice);
                } else if k == "enable_support" {
                    steps.push(posSupportMaterial);
                    if cfg_float(&self.m_config, "support_top_z_distance") == 0. {
                        // Enabling / disabling supports while soluble support interface is enabled.
                        // This changes the bridging logic (bridging enabled without supports, disabled with supports).
                        // Reset everything.
                        steps.push(posSlice);
                    }
                } else if matches!(
                    k,
                    "support_type"
                        | "support_angle"
                        | "support_on_build_plate_only"
                        | "support_critical_regions_only"
                        | "support_remove_small_overhang"
                        | "enforce_support_layers"
                        | "support_filament"
                        | "support_line_width"
                        | "support_interface_top_layers"
                        | "support_interface_bottom_layers"
                        | "support_interface_pattern"
                        | "support_interface_loop_pattern"
                        | "support_interface_filament"
                        | "support_interface_not_for_body"
                        | "support_interface_spacing"
                        | "support_bottom_interface_spacing"
                        | "support_base_pattern"
                        | "support_style"
                        | "support_object_xy_distance"
                        | "support_object_first_layer_gap"
                        | "support_base_pattern_spacing"
                        | "support_expansion"
                        | "support_threshold_angle"
                        | "enable_support_ironing"
                        | "support_ironing_pattern"
                        | "support_ironing_flow"
                        | "support_ironing_spacing"
                        | "support_ironing_inset"
                        | "support_ironing_direction"
                        | "support_ironing_speed"
                        | "raft_expansion"
                        | "raft_first_layer_density"
                        | "raft_first_layer_expansion"
                        | "bridge_no_support"
                        | "max_bridge_length"
                        | "tree_support_branch_distance"
                        | "tree_support_branch_diameter"
                        | "tree_support_branch_angle"
                        | "tree_support_branch_diameter_angle"
                        | "tree_support_tip_diameter"
                        | "tree_support_wall_count"
                ) {
                    steps.push(posSupportMaterial);
                    if cfg_float(&self.m_config, "support_top_z_distance") == 0. {
                        // Zero-gap contact refinement in prepare_infill reads these support settings.
                        steps.push(posPrepareInfill);
                    }
                } else if matches!(
                    k,
                    "bottom_shell_layers" | "top_shell_layers" | "top_color_penetration_layers" | "bottom_color_penetration_layers" | "infill_instead_top_bottom_surfaces"
                ) {
                    steps.push(posSlice);
                } else if matches!(
                    k,
                    "interface_shells"
                        | "fill_multiline"
                        | "infill_combination"
                        | "bottom_shell_thickness"
                        | "top_shell_thickness"
                        | "minimum_sparse_infill_area"
                        | "sparse_infill_filament"
                        | "solid_infill_filament"
                        | "sparse_infill_line_width"
                        | "skin_infill_line_width"
                        | "skeleton_infill_line_width"
                        | "infill_direction"
                        | "ensure_vertical_shell_thickness"
                        | "bridge_angle"
                ) {
                    steps.push(posPrepareInfill);
                } else if matches!(
                    k,
                    "top_surface_pattern"
                        | "top_surface_density"
                        | "monotonic_travel_into_wall"
                        | "bottom_surface_pattern"
                        | "bottom_surface_density"
                        | "internal_solid_infill_pattern"
                        | "sub_top_surface_pattern"
                        | "external_fill_link_max_length"
                        | "sparse_infill_anchor"
                        | "sparse_infill_anchor_max"
                        | "top_surface_line_width"
                        | "detect_floating_vertical_shell"
                ) {
                    steps.push(posInfill);
                } else if matches!(
                    k,
                    "sparse_infill_pattern"
                        | "conformal_infill"
                        | "conformal_stagger"
                        | "conformal_link_keep_layers"
                        | "conformal_link_flip_layers"
                        | "conformal_pole"
                        | "conformal_ray_count"
                        | "conformal_hub_radius"
                        | "symmetric_infill_y_axis"
                        | "infill_shift_step"
                        | "sparse_infill_lattice_angle_1"
                        | "sparse_infill_lattice_angle_2"
                        | "infill_rotate_step"
                        | "skeleton_infill_density"
                        | "skin_infill_density"
                        | "infill_lock_depth"
                        | "skin_infill_depth"
                        | "locked_skin_infill_pattern"
                        | "locked_skeleton_infill_pattern"
                ) {
                    steps.push(posPrepareInfill);
                } else if k == "sparse_infill_density" {
                    // One likely wants to reslice only when switching between zero infill to simulate boolean difference (subtracting volumes),
                    // normal infill and 100% (solid) infill.
                    let old_density = cfg_float(old_config, k);
                    let new_density = cfg_float(new_config, k);
                    if is_approx(old_density, 0.) || is_approx(old_density, 100.) || is_approx(new_density, 0.) || is_approx(new_density, 100.) {
                        steps.push(posPerimeters);
                    }
                    steps.push(posPrepareInfill);
                } else if k == "internal_solid_infill_line_width" {
                    // This value is used for calculating perimeter - infill overlap, thus perimeters need to be recalculated.
                    steps.push(posPerimeters);
                    steps.push(posPrepareInfill);
                } else if matches!(
                    k,
                    "outer_wall_line_width"
                        | "wall_filament"
                        | "fuzzy_skin"
                        | "fuzzy_skin_thickness"
                        | "fuzzy_skin_point_distance"
                        | "fuzzy_skin_first_layer"
                        | "fuzzy_skin_noise_type"
                        | "fuzzy_skin_scale"
                        | "fuzzy_skin_octaves"
                        | "fuzzy_skin_persistence"
                        | "fuzzy_skin_mode"
                        | "detect_overhang_wall"
                        | "enable_overhang_speed"
                        | "detect_thin_wall"
                        | "precise_outer_wall"
                ) {
                    // filament_shrink is applied to slice contours in slice_volumes() (posSlice) using wall_filament.
                    if k == "wall_filament" {
                        steps.push(posSlice);
                    }
                    steps.push(posPerimeters);
                    steps.push(posSupportMaterial);
                } else if k == "bridge_flow" {
                    steps.push(posPerimeters);
                    steps.push(posInfill);
                    steps.push(posSupportMaterial);
                } else if matches!(
                    k,
                    "wall_generator"
                        | "wall_transition_length"
                        | "wall_transition_filter_deviation"
                        | "wall_transition_angle"
                        | "wall_distribution_count"
                        | "min_feature_size"
                        | "min_bead_width"
                ) {
                    steps.push(posSlice);
                } else if matches!(
                    k,
                    "seam_position"
                        | "seam_placement_away_from_overhangs"
                        | "seam_slope_conditional"
                        | "scarf_angle_threshold"
                        | "seam_slope_entire_loop"
                        | "seam_slope_steps"
                        | "seam_slope_inner_walls"
                        | "seam_gap"
                        | "wipe_speed"
                        | "role_base_wipe_speed"
                        | "support_interface_speed"
                        | "smooth_speed_discontinuity_area"
                        | "smooth_coefficient"
                        | "overhang_1_4_speed"
                        | "overhang_2_4_speed"
                        | "overhang_3_4_speed"
                        | "overhang_4_4_speed"
                        | "overhang_totally_speed"
                        | "bridge_speed"
                        | "small_perimeter_speed"
                        | "small_perimeter_threshold"
                        | "vertical_shell_speed"
                        | "enable_height_slowdown"
                        | "slowdown_start_height"
                        | "slowdown_start_speed"
                        | "slowdown_start_acc"
                        | "slowdown_end_height"
                        | "slowdown_end_speed"
                        | "slowdown_end_acc"
                ) {
                    invalidated |= print_invalidate_step(print, PrintStep::psGCodeExport);
                } else if matches!(k, "flush_into_infill" | "flush_into_objects" | "flush_into_support") {
                    invalidated |= print_invalidate_step(print, PrintStep::psWipeTower);
                    invalidated |= print_invalidate_step(print, PrintStep::psGCodeExport);
                } else {
                    // for legacy, if we can't handle this option let's invalidate all steps
                    self.invalidate_all_steps(print);
                    invalidated = true;
                }
            }

            sort_remove_duplicates(&mut steps);
            for step in steps {
                invalidated |= self.invalidate_step(print, step);
            }
            invalidated
        }

        /// PrintObject.cpp:1879-1914 `bool PrintObject::invalidate_step(PrintObjectStep step)`
        pub fn invalidate_step(&mut self, print: &mut PrintStepState, step: PrintObjectStep) -> bool {
            use PrintObjectStep::*;
            let cancel = print.base.cancel_callback();
            let mut invalidated = self.m_state.invalidate(step, || cancel());

            // propagate to dependent steps
            if step == posPerimeters {
                invalidated |= self.m_state.invalidate_multiple([posPrepareInfill, posInfill, posIroning, posSimplifyWall, posSimplifyInfill], || cancel());
                invalidated |= print.invalidate_steps([PrintStep::psSkirtBrim]);
            } else if step == posPrepareInfill {
                invalidated |= self.m_state.invalidate_multiple([posInfill, posIroning, posSimplifyWall, posSimplifyInfill], || cancel());
                invalidated |= print.invalidate_steps([PrintStep::psSkirtBrim]);
            } else if step == posInfill {
                invalidated |= self.m_state.invalidate_multiple([posIroning, posSimplifyInfill], || cancel());
                invalidated |= print.invalidate_steps([PrintStep::psSkirtBrim]);
            } else if step == posSlice {
                invalidated |= self.m_state.invalidate_multiple(
                    [posPerimeters, posPrepareInfill, posInfill, posIroning, posSupportMaterial, posSimplifyWall, posSimplifyInfill],
                    || cancel(),
                );
                invalidated |= print.invalidate_steps([PrintStep::psSkirtBrim]);
                self.m_slicing_params.valid = false;
            } else if step == posSupportMaterial {
                invalidated |= self.m_state.invalidate_multiple([posSimplifySupportPath], || cancel());
                invalidated |= print.invalidate_steps([PrintStep::psSkirtBrim]);
                self.m_slicing_params.valid = false;
            }
            // BBS multi-diameter: the support merge plan reads which filaments the walls and fills of each layer extrude.
            if (step == posPerimeters || step == posPrepareInfill || step == posInfill)
                && crate::print_config::config_def::multi_nozzle_diameter_mode_enabled(&print.base.m_full_print_config)
            {
                invalidated |= self.invalidate_step(print, posSupportMaterial);
            }

            // Wipe tower depends on the ordering of extruders, which in turn depends on everything.
            invalidated |= print_invalidate_step(print, PrintStep::psWipeTower);
            // Invalidate G-code export in any case.
            invalidated |= print_invalidate_step(print, PrintStep::psGCodeExport);
            invalidated
        }

        /// PrintObject.cpp:1916-1923 `bool PrintObject::invalidate_all_steps()`
        pub fn invalidate_all_steps(&mut self, print: &mut PrintStepState) -> bool {
            // First call the "invalidate" functions, which may cancel background processing.
            let cancel = print.base.cancel_callback();
            let result = self.m_state.invalidate_all(|| cancel()) | print.invalidate_all_steps();
            // Then reset some of the depending values.
            self.m_slicing_params.valid = false;
            result
        }

        /// PrintObject.cpp:3840-3845 `void PrintObject::update_slicing_parameters()`, on the object
        /// `object_idx` of the print.
        pub fn update_slicing_parameters(print: &mut Print, object_idx: usize) {
            if print.m_objects[object_idx].m_slicing_params.valid {
                return;
            }
            let params = print.m_objects[object_idx].compute_slicing_parameters(print, print.single_diameter_layering_nozzle());
            print.m_objects[object_idx].m_slicing_params = params;
        }

        /// PrintObject.cpp:3847-3860 `SlicingParameters PrintObject::compute_slicing_parameters(double
        /// single_diameter_layering_nozzle) const`
        pub fn compute_slicing_parameters(&self, print: &Print, single_diameter_layering_nozzle: f64) -> SlicingParams {
            use crate::print_config::config_def::{layer_grid_nozzle_diameter, layer_grid_subdivision};
            let full_config = print.full_print_config();
            let subdivision = layer_grid_subdivision(full_config);
            // BBS multi-diameter: a plate printing with one diameter only stacks that nozzle's band of grid steps into
            // one sliced layer, which is the height the merge would have given every band.
            let mut multiplier = 1;
            if single_diameter_layering_nozzle > EPSILON {
                multiplier = crate::adaptive_layer_merge::adaptive_layer_band_size(
                    single_diameter_layering_nozzle,
                    layer_grid_nozzle_diameter(full_config),
                    cfg_float(&self.m_config, "layer_height") / (1.max(subdivision) as f64),
                ) as i32;
            }
            SlicingParams::create_from_config(
                print.config(),
                &self.m_config,
                self.model_object(print).bounding_box().max.z,
                &self.object_extruders(print),
                subdivision,
                multiplier,
                single_diameter_layering_nozzle,
            )
        }

        /// PrintObject.cpp:3902-3923 `std::vector<unsigned int> PrintObject::object_extruders() const`:
        /// 0-based indices of the extruders used to print the object (without brim, support and other
        /// helper extrusions), the `#else` branch: the extruders of the object's volumes.
        pub fn object_extruders(&self, print: &Print) -> Vec<u32> {
            let mut extruders: Vec<u32> = Vec::with_capacity(self.all_regions().len() * 3);
            let mo = self.model_object(print);
            let global_features = crate::model::GlobalFeatureFilaments::from_config(
                print.default_region_config(),
                crate::gcode::wipe_tower::cfg_floats(print.config(), "filament_diameter").len(),
            );
            for mv in &mo.volumes {
                let volume_extruders = mv.get_extruders(mo, &global_features);
                for extruder in volume_extruders {
                    debug_assert!(extruder > 0);
                    extruders.push((extruder - 1) as u32);
                }
            }
            sort_remove_duplicates(&mut extruders);
            extruders
        }

        /// PrintObject.cpp:3925-3967 `bool PrintObject::update_layer_height_profile(const ModelObject
        /// &model_object, const SlicingParameters &slicing_parameters, std::vector<coordf_t>
        /// &layer_height_profile, bool &out_nozzle_range_reset)`
        pub fn update_layer_height_profile(
            model_object: &ModelObject,
            slicing_parameters: &SlicingParams,
            layer_height_profile: &mut Vec<f64>,
            out_nozzle_range_reset: &mut bool,
        ) -> bool {
            let mut updated = false;
            *out_nozzle_range_reset = false;

            if layer_height_profile.is_empty() {
                *layer_height_profile = model_object.layer_height_profile.get();
                let mut i = 1;
                while i < layer_height_profile.len() {
                    layer_height_profile[i] = slicing_parameters.to_base_layer_height(layer_height_profile[i]);
                    i += 2;
                }
                updated = true;
            }

            // Verify the layer_height_profile.
            if !layer_height_profile.is_empty()
                // Must not be of even length.
                && ((layer_height_profile.len() & 1) != 0
                    // Last entry must be at the top of the object.
                    || (layer_height_profile[layer_height_profile.len() - 2] - slicing_parameters.object_print_z_max + slicing_parameters.object_print_z_min).abs() > 1e-3)
            {
                layer_height_profile.clear();
            }

            // Verify that all layer height values are within the allowed range for the current nozzle.
            if !layer_height_profile.is_empty() {
                let mut i = 1;
                while i < layer_height_profile.len() {
                    if layer_height_profile[i] < slicing_parameters.min_layer_height - EPSILON
                        || layer_height_profile[i] > slicing_parameters.max_layer_height + EPSILON
                    {
                        *out_nozzle_range_reset = true;
                        layer_height_profile.clear();
                        break;
                    }
                    i += 2;
                }
            }

            let mut not_match_flag = !slicing_parameters.has_raft(); // if there is raft layer_height_profile[1] could also be adaptive
            not_match_flag &= !layer_height_profile.is_empty() && (layer_height_profile[1] != slicing_parameters.first_object_layer_height);
            if layer_height_profile.is_empty() || not_match_flag {
                *layer_height_profile = crate::slicing::layer_height_profile_from_ranges(slicing_parameters, &model_object.layer_config_ranges);
                updated = true;
            }

            updated
        }

        /// PrintObject.cpp:3969-3983 `void PrintObject::get_certain_layers(float start, float end,
        /// std::vector<LayerPtrs> &out, std::vector<BoundingBox> &boundingbox_objects)`. A layer is the
        /// object's index `self_index` in `Print::m_objects` and the layer's index in `m_layers`.
        pub fn get_certain_layers(
            &self,
            self_index: usize,
            start: f32,
            end: f32,
            out: &mut Vec<Vec<(usize, usize)>>,
            boundingbox_objects: &mut Vec<crate::geometry::BoundingBox>,
        ) {
            let mut temp = crate::geometry::BoundingBox::new();
            let mut out_temp: Vec<(usize, usize)> = Vec::new();
            for (layer_idx, layer) in self.layers().iter().enumerate() {
                if layer.print_z < start as f64 {
                    continue;
                }

                if layer.print_z > end as f64 + EPSILON {
                    break;
                }
                temp.merge(&layer.loverhangs_bbox);
                out_temp.push((self_index, layer_idx));
            }
            boundingbox_objects.push(temp);
            out.push(out_temp);
        }

        /// PrintObject.cpp:3985-3993 `std::vector<Point> PrintObject::get_instances_shift_without_plate_offset() const`
        pub fn get_instances_shift_without_plate_offset(&self, print: &Print) -> Vec<Point> {
            let mut out: Vec<Point> = Vec::with_capacity(self.m_instances.len());
            for instance in &self.m_instances {
                out.push(instance.shift_without_plate_offset(print));
            }
            out
        }
    }

    /// PrintObject.cpp:3862-3900 `SlicingParameters PrintObject::slicing_parameters(const DynamicPrintConfig&
    /// full_config, const ModelObject& model_object, float object_max_z, std::vector<int> variant_index)`,
    /// the static overload (the member one is `PrintObject::slicing_parameters()`). `model` is the
    /// model of `model_object` (for the materials of its volumes).
    pub fn slicing_parameters(
        full_config: &DynamicPrintConfig,
        model_object: &ModelObject,
        model: Option<&Model>,
        mut object_max_z: f32,
        mut variant_index: Vec<i32>,
    ) -> SlicingParams {
        let static_apply = |class_name: &str| {
            let mut config = static_config_defaults(class_name);
            let keys = config.keys();
            let _ = config.apply_only(full_config, &keys, true);
            config
        };
        let print_config = static_apply("PrintConfig");
        let mut object_config: PrintObjectConfig = static_apply("PrintObjectConfig");
        let default_region_config: PrintRegionConfig = static_apply("PrintRegionConfig");
        // BBS
        let filament_extruders = crate::gcode::wipe_tower::cfg_floats(&print_config, "filament_diameter").len();
        object_config = crate::print_apply::object_config_from_model_object(&object_config, model_object, filament_extruders, &mut variant_index);

        let has_brim = cfg_int(&object_config, "brim_type") != BrimType::BtNoBrim as i32 && cfg_float(&object_config, "brim_width") > 0.;
        let mut object_extruders: Vec<u32> = Vec::new();
        for model_volume in &model_object.volumes {
            if model_volume.is_model_part() {
                PrintRegion::collect_object_printing_extruders_static(
                    &print_config,
                    &crate::print_apply::region_config_from_model_volume(
                        &default_region_config,
                        None,
                        model_volume,
                        model_object,
                        model,
                        filament_extruders,
                        &mut variant_index,
                    ),
                    has_brim,
                    &mut object_extruders,
                );
                for (_, range_config) in model_object.layer_config_ranges.iter() {
                    if range_config.has("wall_filament") || range_config.has("sparse_infill_filament") || range_config.has("solid_infill_filament") {
                        PrintRegion::collect_object_printing_extruders_static(
                            &print_config,
                            &crate::print_apply::region_config_from_model_volume(
                                &default_region_config,
                                Some(range_config.get()),
                                model_volume,
                                model_object,
                                model,
                                filament_extruders,
                                &mut variant_index,
                            ),
                            has_brim,
                            &mut object_extruders,
                        );
                    }
                }
            }
        }
        sort_remove_duplicates(&mut object_extruders);
        //FIXME add painting extruders

        if object_max_z <= 0.0 {
            object_max_z = model_object.raw_bounding_box().map(|b| b.size().z).unwrap_or(0.) as f32;
        }
        // The interactive variable-layer editor works in the authored preset's height domain.
        SlicingParams::create_from_config(&print_config, &object_config, object_max_z as f64, &object_extruders, 1, 1, 0.)
    }

    /// PrintObject.cpp:655 `const coord_t zero_gap_length_thresh = scale_(2);`
    pub fn zero_gap_length_thresh() -> i64 {
        (2.0 / SCALING_FACTOR) as i64
    }

    /// PrintObject.cpp:657-663 `double zero_gap_threshold_rad(const PrintObjectConfig &config)`: the
    /// same thresh_angle convention as SupportMaterial::detect_overhangs: +1, clamp 89.
    pub fn zero_gap_threshold_rad(config: &PrintObjectConfig) -> f64 {
        let support_threshold_angle = cfg_int(config, "support_threshold_angle") as f64;
        let mut thresh_angle = if support_threshold_angle > 0. { support_threshold_angle + 1. } else { 0. };
        thresh_angle = thresh_angle.min(89.);
        // Geometry::deg2rad: T(PI) * angle / T(180.0)
        std::f64::consts::PI * thresh_angle / 180.0
    }

    /// PrintObject.cpp:675-694 `bool should_refine_zero_gap_contact(const PrintObjectConfig &config)`
    pub fn should_refine_zero_gap_contact(config: &PrintObjectConfig) -> bool {
        config.opt_bool("enable_support")
            && cfg_float(config, "support_top_z_distance") == 0.
            && cfg_int(config, "support_type") == SupportType::StNormalAuto as i32
            && !config.opt_bool("support_on_build_plate_only")
            && !config.opt_bool("support_critical_regions_only")
            // Without interface layers the contact is printed with the sparse base pattern.
            && cfg_int(config, "support_interface_top_layers") > 0
            // Keep legacy Bridge classification whenever bridge_no_support is on.
            && !config.opt_bool("bridge_no_support")
    }

    /// PrintObject.cpp:696-706 `bool is_significant_contact_piece(const ExPolygon &piece, float erode_r)`
    pub fn is_significant_contact_piece(piece: &ExPolygon, erode_r: f32) -> bool {
        if erode_r <= 0.0 {
            return false;
        }
        let eroded =
            crate::clipper_utils::offset_ex_expolygon_clib(piece, -(erode_r as f64), crate::clipper_utils::OffsetJoinType::Miter, 3.0);
        if eroded.is_empty() {
            return false;
        }
        let bbox_sz = crate::geometry::get_extents(&eroded).size();
        bbox_sz.x > zero_gap_length_thresh() || bbox_sz.y > zero_gap_length_thresh()
    }

    /// PrintObject.cpp:708-716 `ExPolygons filter_significant_contact_pieces(ExPolygons pieces, float erode_r)`
    pub fn filter_significant_contact_pieces(pieces: Vec<ExPolygon>, erode_r: f32) -> Vec<ExPolygon> {
        let mut out: Vec<ExPolygon> = Vec::with_capacity(pieces.len());
        for piece in pieces {
            if is_significant_contact_piece(&piece, erode_r) {
                out.push(piece);
            }
        }
        out
    }

    /// PrintObject.cpp:2254-2258 `struct EffectiveShellParameters`
    #[derive(Debug, Clone, Copy, PartialEq)]
    pub struct EffectiveShellParameters {
        pub layers: i32,
        pub thickness: f64,
    }

    /// PrintObject.cpp:2260-2268 `static int shell_band_size(const PrintObject &object, double
    /// nozzle_diameter, double base_layer_height)`: the base layers one extrusion of this nozzle covers.
    pub fn shell_band_size(print: &Print, nozzle_diameter: f64, base_layer_height: f64) -> i32 {
        if print.single_diameter_layering_nozzle() > EPSILON {
            return 1;
        }
        crate::adaptive_layer_merge::adaptive_layer_band_size(
            nozzle_diameter,
            crate::print_config::config_def::layer_grid_nozzle_diameter(print.full_print_config()),
            base_layer_height,
        ) as i32
    }

    /// PrintObject.cpp:2270-2283 `static EffectiveShellParameters effective_top_shell_parameters(const
    /// PrintObject &object, const PrintRegionConfig &config, double base_layer_height)`
    pub fn effective_top_shell_parameters(print: &Print, config: &PrintRegionConfig, base_layer_height: f64) -> EffectiveShellParameters {
        use crate::print_config::config_def::{multi_nozzle_diameter_mode_enabled, multi_nozzle_top_shell_layers, multi_nozzle_top_shell_thickness};
        let mut out = EffectiveShellParameters { layers: cfg_int(config, "top_shell_layers"), thickness: cfg_float(config, "top_shell_thickness") };
        if !multi_nozzle_diameter_mode_enabled(print.full_print_config()) {
            return out;
        }
        let nozzle_diameter =
            crate::print::faithful::nozzle_diameter_for_filament(print.config(), cfg_int(config, "solid_infill_filament") as u32) as f64;
        out.layers = multi_nozzle_top_shell_layers(nozzle_diameter, out.layers);
        out.layers *= shell_band_size(print, nozzle_diameter, base_layer_height);
        out.thickness = multi_nozzle_top_shell_thickness(nozzle_diameter, out.thickness);
        out
    }

    /// PrintObject.cpp:2285-2298 `static EffectiveShellParameters effective_bottom_shell_parameters(const
    /// PrintObject &object, const PrintRegionConfig &config, double base_layer_height)`
    pub fn effective_bottom_shell_parameters(print: &Print, config: &PrintRegionConfig, base_layer_height: f64) -> EffectiveShellParameters {
        use crate::print_config::config_def::{multi_nozzle_bottom_shell_layers, multi_nozzle_bottom_shell_thickness, multi_nozzle_diameter_mode_enabled};
        let mut out =
            EffectiveShellParameters { layers: cfg_int(config, "bottom_shell_layers"), thickness: cfg_float(config, "bottom_shell_thickness") };
        if !multi_nozzle_diameter_mode_enabled(print.full_print_config()) {
            return out;
        }
        let nozzle_diameter =
            crate::print::faithful::nozzle_diameter_for_filament(print.config(), cfg_int(config, "solid_infill_filament") as u32) as f64;
        out.layers = multi_nozzle_bottom_shell_layers(nozzle_diameter, out.layers);
        out.layers *= shell_band_size(print, nozzle_diameter, base_layer_height);
        out.thickness = multi_nozzle_bottom_shell_thickness(nozzle_diameter, out.thickness);
        out
    }

    // ---------------------------------------------------------------------------------------------
    // PrintObject.cpp functions over the object's layers (`m_layers`, the crate's Layer and
    // LayerRegion) and its shared regions (the PrintRegionConfig of `print::faithful`).
    // ---------------------------------------------------------------------------------------------

    /// ClipperLib `pftNonZero`
    const PFT_NON_ZERO: i32 = 1;

    fn expolygons_of_polygons(polygons: &[crate::geometry::Polygon]) -> Vec<ExPolygon> {
        polygons.iter().map(|p| ExPolygon::new(p.clone())).collect()
    }

    /// PrintObject.cpp:665-673 `float scaled_external_perimeter_width(const Layer &layer)`: the real
    /// external-perimeter flow width rather than config.line_width (which may be 0). Max across regions is
    /// conservative: a larger guard_r only shrinks the Bottom mask.
    pub fn scaled_external_perimeter_width(layer: &crate::layer::Layer) -> f32 {
        let mut fw: f32 = 0.0;
        for layerm in layer.regions() {
            let w = layerm.flow(crate::flow::FlowRole::ExternalPerimeter, layer.height).map(|f| f.scaled_width()).unwrap_or(0);
            fw = fw.max(w as f32);
        }
        fw
    }

    /// PrintObject.cpp:719-723 `struct ZeroGapContactMask`
    #[derive(Debug, Clone, Default)]
    pub struct ZeroGapContactMask {
        pub polygons: Vec<ExPolygon>,
        /// leftover / inset sliver filter, <= the opening radius
        pub sliver_r: f32,
    }

    /// PrintObject.cpp:725-811 `ZeroGapContactMask compute_zero_gap_contact_mask(const PrintObject &object, const
    /// Layer &layer, const Layer &lower_layer, size_t layer_id, const std::vector<Polygons> &blockers, const
    /// std::vector<Polygons> &enforcers)`: lightweight zero-gap top-contact candidate, intentionally
    /// conservative vs classic detect_overhangs.
    pub fn compute_zero_gap_contact_mask(
        object: &PrintObject,
        layer: &crate::layer::Layer,
        lower_layer: &crate::layer::Layer,
        layer_id: usize,
        blockers: &[Vec<crate::geometry::Polygon>],
        enforcers: &[Vec<crate::geometry::Polygon>],
    ) -> ZeroGapContactMask {
        use crate::clipper_utils::{
            difference_clib, intersection_clib, offset2_ex_clib, offset_expolygons_clib_scaled, offset_polygons_clib_ml, union_ex_clib,
            union_flat_clib, OffsetJoinType,
        };
        let mut result = ZeroGapContactMask::default();
        let extrusion_width_scaled = scaled_external_perimeter_width(layer);
        if extrusion_width_scaled <= 0.0 {
            return result;
        }

        let config = object.config();
        let threshold_rad = zero_gap_threshold_rad(config);
        let angle_offset: f32 = if threshold_rad > 0. {
            ((lower_layer.height / threshold_rad.tan()) / SCALING_FACTOR) as f32
        } else {
            0.5 * extrusion_width_scaled
        };
        // Opening radius covering both the threshold angle and the small-overhang lower bound.
        let guard_r = (2.5f32 * extrusion_width_scaled).max(angle_offset);
        result.sliver_r = 2.5 * extrusion_width_scaled;
        if guard_r <= 0.0 {
            return result;
        }

        let lower_polygons = crate::geometry::to_polygons(&lower_layer.lslices);

        // Automatic overhangs, approximating detect_overhangs: the opening drops what the
        // threshold angle and small-overhang removal would have dropped.
        let mut mask: Vec<ExPolygon> = Vec::new();
        {
            let lower_expanded =
                if angle_offset > 0.0 { offset_polygons_clib_ml(&lower_polygons, angle_offset as f64, OffsetJoinType::Square, 0.) } else { lower_polygons.clone() };
            let raw = difference_clib(&layer.lslices, &expolygons_of_polygons(&lower_expanded));
            if !raw.is_empty() {
                // opening_ex(raw, guard_r) = offset2_ex(raw, -guard_r, guard_r)
                let guard_mm = guard_r as f64 * SCALING_FACTOR;
                mask = intersection_clib(&offset2_ex_clib(&raw, -guard_mm, guard_mm, OffsetJoinType::Miter), &raw);
                mask = filter_significant_contact_pieces(mask, guard_r);
            }
        }

        // Support enforcers, mirroring detect_contacts: they enforce support as if the slope were
        // 90 degrees, so neither the threshold angle nor small-overhang removal applies.
        if layer_id < enforcers.len() && !enforcers[layer_id].is_empty() {
            let mut enforced = difference_clib(
                &intersection_clib(&layer.lslices, &expolygons_of_polygons(&enforcers[layer_id])),
                &expolygons_of_polygons(&offset_polygons_clib_ml(&lower_polygons, (0.05f32 * extrusion_width_scaled) as f64, OffsetJoinType::Square, 0.)),
            );
            enforced = filter_significant_contact_pieces(enforced, result.sliver_r);
            if !enforced.is_empty() {
                mask = if mask.is_empty() {
                    enforced
                } else {
                    let mut all = crate::geometry::to_polygons(&mask);
                    all.extend(crate::geometry::to_polygons(&enforced));
                    union_ex_clib(&all, PFT_NON_ZERO)
                };
            }
        }
        if mask.is_empty() {
            return result;
        }

        // Support blockers (expand slightly, matching SupportMaterial). Classic support lets an
        // enforcer win over a blocker; subtracting from the whole mask is stricter on purpose.
        if layer_id < blockers.len() && !blockers[layer_id].is_empty() {
            // expand(union_(blockers), float(1000. * SCALED_EPSILON)): jtMiter, miter limit 3.
            let blocker = offset_polygons_clib_ml(
                &union_flat_clib(&blockers[layer_id]),
                ((1000. * crate::libslic3r::SCALED_EPSILON) as f32) as f64,
                OffsetJoinType::Miter,
                3.,
            );
            mask = difference_clib(&mask, &expolygons_of_polygons(&blocker));
            if mask.is_empty() {
                return result;
            }
        }

        // Negative support_expansion shrinks contacts in classic support; mirror that.
        let xy_expansion = (cfg_float(config, "support_expansion") / SCALING_FACTOR) as f32;
        if (xy_expansion as f64) < -crate::libslic3r::SCALED_EPSILON {
            mask = offset_expolygons_clib_scaled(&mask, xy_expansion as f64, OffsetJoinType::Miter);
            if mask.is_empty() {
                return result;
            }
        }

        // Lateral gap between support and object wall is not covered by the opening;
        // erode so the unsupported rim stays Bridge.
        let xy_gap = (cfg_float(config, "support_object_xy_distance") / SCALING_FACTOR) as f32;
        if xy_gap as f64 > crate::libslic3r::SCALED_EPSILON {
            mask = offset_expolygons_clib_scaled(&mask, -(xy_gap as f64), OffsetJoinType::Miter);
            if mask.is_empty() {
                return result;
            }
        }

        // Re-filter leftover slivers after shrinks. Use sliver_r, not guard_r.
        result.polygons = filter_significant_contact_pieces(mask, result.sliver_r);
        result
    }

    /// PrintObject.cpp:813-858 `void refine_fill_surfaces_for_zero_gap_contact(LayerRegion &layerm, const
    /// ExPolygons &mask, float sliver_r)`: split each stBottomBridge fill surface into supported (stBottom) /
    /// unsupported (stBottomBridge).
    pub fn refine_fill_surfaces_for_zero_gap_contact(layerm: &mut crate::layer::LayerRegion, mask: &[ExPolygon], sliver_r: f32) {
        use crate::clipper_utils::{difference_clib, intersection_clib};
        use crate::surface::SurfaceType;
        if mask.is_empty() || sliver_r <= 0.0 {
            return;
        }

        let mut out: Vec<crate::surface::Surface> = Vec::with_capacity(layerm.fill_surfaces.surfaces.len());

        for surface in &layerm.fill_surfaces.surfaces {
            if surface.surface_type != SurfaceType::BottomBridge {
                out.push(surface.clone());
                continue;
            }

            // fill_surfaces is already inset by perimeters; do not re-apply the full
            // overhang opening radius here or a valid contact would fail the bbox test.
            let candidates = intersection_clib(std::slice::from_ref(&surface.expolygon), mask);
            let mut promoted: Vec<ExPolygon> = Vec::with_capacity(candidates.len());
            for expoly in candidates {
                if is_significant_contact_piece(&expoly, sliver_r) {
                    promoted.push(expoly);
                }
            }

            if promoted.is_empty() {
                out.push(surface.clone());
                continue;
            }

            // Remainder must be computed before moving promoted polygons.
            let remainder = difference_clib(std::slice::from_ref(&surface.expolygon), &promoted);
            for expoly in promoted {
                let mut refined = surface.clone();
                refined.expolygon = expoly;
                refined.surface_type = SurfaceType::Bottom;
                out.push(refined);
            }
            // Original minus promoted: rim, true overhang, and rejected slivers stay Bridge together.
            for expoly in remainder {
                let mut s = surface.clone();
                s.expolygon = expoly;
                out.push(s);
            }
        }

        layerm.fill_surfaces.surfaces = out;
    }

    /// PrintObject.cpp:862-900 `static void type_bridges_over_suppressed_orphans(const LayerPtrs &layers)`:
    /// BBS multi-diameter: a suppressed orphan is sliced but never extruded, so the step above it rests on
    /// nothing there; type it as the bridge it really is, in the outline of the face that was dropped.
    /// `adaptive_merge_patches[layer][region]` stands for `LayerRegion::adaptive_merge_patches`, which the
    /// crate's LayerRegion does not carry.
    pub fn type_bridges_over_suppressed_orphans(
        layers: &mut [crate::layer::Layer],
        adaptive_merge_patches: &[Vec<Vec<crate::adaptive_layer_merge::AdaptiveMergePatch>>],
    ) {
        use crate::adaptive_layer_merge::{adaptive_suppressed_orphan_area, AdaptiveSuppressedOrphanRegion};
        use crate::clipper_utils::{difference_clib, intersection_clib};
        use crate::surface::SurfaceType;
        let no_patches: Vec<crate::adaptive_layer_merge::AdaptiveMergePatch> = Vec::new();
        for layer_idx in 1..layers.len() {
            let dropped = {
                let lower = &layers[layer_idx - 1];
                let regions: Vec<AdaptiveSuppressedOrphanRegion<'_>> = lower
                    .regions()
                    .iter()
                    .enumerate()
                    .map(|(region_id, layerm)| AdaptiveSuppressedOrphanRegion {
                        adaptive_merge_patches: adaptive_merge_patches
                            .get(layer_idx - 1)
                            .and_then(|l| l.get(region_id))
                            .map(|p| p.as_slice())
                            .unwrap_or(&no_patches),
                        slices: &layerm.slices.surfaces,
                    })
                    .collect();
                adaptive_suppressed_orphan_area(&regions)
            };
            if dropped.is_empty() {
                continue;
            }
            let retype = |surfaces: &mut crate::surface_collection::SurfaceCollection| {
                let mut out: Vec<crate::surface::Surface> = Vec::new();
                let mut changed = false;
                for surface in &surfaces.surfaces {
                    let mut bridge: Vec<ExPolygon> = Vec::new();
                    if surface.surface_type == SurfaceType::Internal || surface.surface_type == SurfaceType::InternalSolid {
                        bridge = intersection_clib(std::slice::from_ref(&surface.expolygon), &dropped);
                    }
                    if bridge.is_empty() {
                        out.push(surface.clone());
                        continue;
                    }
                    changed = true;
                    for expolygon in difference_clib(std::slice::from_ref(&surface.expolygon), &dropped) {
                        let mut s = surface.clone();
                        s.expolygon = expolygon;
                        out.push(s);
                    }
                    for expolygon in bridge {
                        let mut s = surface.clone();
                        s.expolygon = expolygon;
                        s.surface_type = SurfaceType::BottomBridge;
                        s.bridge_angle = None;
                        out.push(s);
                    }
                }
                if changed {
                    surfaces.surfaces = out;
                }
            };
            for layerm in layers[layer_idx].regions_mut().iter_mut() {
                retype(&mut layerm.slices);
                retype(&mut layerm.fill_surfaces);
            }
        }
    }

    /// PrintObject.cpp:1075-1081 `static const VolumeSlices *volume_slices_by_id(const std::vector<VolumeSlices>
    /// &all, const ObjectID id)`
    pub fn volume_slices_by_id(all: &[crate::print::faithful::VolumeSlices], id: crate::object_id::ObjectID) -> Option<&crate::print::faithful::VolumeSlices> {
        all.iter().find(|vs| vs.volume_id == id)
    }

    /// PrintConfig.hpp:1303-1312 `inline bool periodic_modifier_active(const PrintRegionConfig &cfg, int layer_id)`
    ///
    /// FIDELITY-NOTE: PrintConfig.hpp's inline function, kept here until print_config.rs carries it.
    pub fn periodic_modifier_active(cfg: &PrintRegionConfig, layer_id: i32) -> bool {
        if !cfg.opt_bool("periodic_modifier") || cfg_int(cfg, "periodic_modifier_apply_layers") < 1 || layer_id < 0 {
            return true;
        }
        let m = cfg_int(cfg, "periodic_modifier_apply_layers");
        let mut n = cfg_int(cfg, "periodic_modifier_skip_layers");
        if n < 0 {
            n = 0;
        }
        (layer_id % (m + n)) < m
    }

    /// PrintObject.cpp:1083-1152 `static std::vector<std::vector<ExPolygon>> conformal_modifier_islands(const
    /// PrintObject &po, size_t region_id, size_t nlid, size_t lid0)`: the island of a conformal modifier after
    /// earlier modifiers/model, before later volumes carve it.
    pub fn conformal_modifier_islands(po: &PrintObject, print: &Print, region_id: usize, nlid: usize, lid0: usize) -> Vec<Vec<ExPolygon>> {
        use crate::clipper_utils::{difference_clib, intersection_clib, union_ex_clib};
        let mut out: Vec<Vec<ExPolygon>> = vec![Vec::new(); nlid];
        let regions = po.shared_regions();
        let vol_slices = po.firstLayerObjSlice();
        if regions.is_none() || vol_slices.is_empty() || po.layer_count() == 0 {
            return out;
        }
        let regions = regions.unwrap();
        let regions = regions.borrow();
        let mo = po.model_object(print);

        for zi in 0..po.layer_count() {
            let layer = po.get_layer(zi as i32);
            let lid = layer.id().wrapping_sub(lid0);
            if lid >= nlid {
                continue;
            }
            let z = layer.slice_z;
            let mut it = 0usize;
            while it < regions.layer_ranges.len() {
                let r = regions.layer_ranges[it].layer_height_range;
                if r.0 <= z + EPSILON && z <= r.1 + EPSILON {
                    break;
                }
                it += 1;
            }
            if it == regions.layer_ranges.len() {
                continue;
            }
            if (z - regions.layer_ranges[it].layer_height_range.1).abs() < EPSILON {
                let nxt = it + 1;
                if nxt != regions.layer_ranges.len() && (regions.layer_ranges[nxt].layer_height_range.0 - z).abs() < EPSILON {
                    it = nxt;
                }
            }
            let layer_range = &regions.layer_ranges[it];
            if layer_range.volume_regions.is_empty() {
                continue;
            }

            let mut temp: Vec<Vec<ExPolygon>> = vec![Vec::new(); layer_range.volume_regions.len()];
            for i in 0..layer_range.volume_regions.len() {
                let mv = &mo.volumes[layer_range.volume_regions[i].model_volume.unwrap()];
                if let Some(vs) = volume_slices_by_id(vol_slices, mv.id()) {
                    if zi < vs.slices.len() {
                        temp[i] = vs.slices[zi].clone();
                    }
                }
            }

            for idx in 0..layer_range.volume_regions.len() as i32 {
                if temp[idx as usize].is_empty() {
                    continue;
                }
                let vr = &layer_range.volume_regions[idx as usize];
                let model_volume = &mo.volumes[vr.model_volume.unwrap()];
                if !model_volume.is_modifier() || vr.parent < 0 {
                    continue;
                }
                let next_same = idx + 1 < layer_range.volume_regions.len() as i32
                    && layer_range.volume_regions[(idx + 1) as usize].model_volume == vr.model_volume;
                let source = temp[idx as usize].clone();
                let skip_periodic = match &vr.region {
                    Some(r) => {
                        let r = r.borrow();
                        r.config().opt_bool("periodic_modifier") && !periodic_modifier_active(r.config(), zi as i32)
                    }
                    None => false,
                };
                let parent = vr.parent as usize;
                if skip_periodic || temp[parent].is_empty() {
                    temp[idx as usize].clear();
                } else {
                    temp[idx as usize] = intersection_clib(&temp[parent], &source);
                    temp[parent] = difference_clib(&temp[parent], &source);
                }
                if next_same {
                    temp[(idx + 1) as usize] = source;
                }
                if let Some(r) = &vr.region {
                    let r = r.borrow();
                    if r.print_object_region_id() as usize == region_id && r.config().opt_bool("conformal_infill") && !temp[idx as usize].is_empty() {
                        out[lid].extend(temp[idx as usize].iter().cloned());
                    }
                }
            }
        }
        for row in out.iter_mut() {
            if row.len() > 1 {
                *row = union_ex_clib(&crate::geometry::to_polygons(row), PFT_NON_ZERO);
            }
        }
        out
    }

    /// PrintObject.cpp:2245-2253 `static bool layer_skips_periodic_modifier(const PrintObject &object, int layer_id)`
    pub fn layer_skips_periodic_modifier(object: &PrintObject, layer_id: i32) -> bool {
        for i in 0..object.num_printing_regions() {
            let region = object.printing_region(i);
            let region = region.borrow();
            let cfg = region.config();
            if cfg.opt_bool("periodic_modifier") && !periodic_modifier_active(cfg, layer_id) {
                return true;
            }
        }
        false
    }

    /// PrintObject.cpp:4283-4306 `static double band_reference_nozzle(const PrintObject &object)`
    pub fn band_reference_nozzle(object: &PrintObject, print: &Print) -> f64 {
        let primary = crate::print_config::config_def::layer_grid_nozzle_diameter(print.full_print_config());
        if primary > EPSILON {
            return primary;
        }
        // No printer preset to read the variant from. Fall back to the finest nozzle laying material down.
        let print_config = print.config();
        let mut min_nozzle = 0.0;
        for region_id in 0..object.num_printing_regions() {
            let mut prints_anything = false;
            for layer in object.layers() {
                if !layer.get_region(region_id).map(|r| r.raw_slices.is_empty()).unwrap_or(true) {
                    prints_anything = true;
                    break;
                }
            }
            if !prints_anything {
                continue;
            }
            let nozzle = crate::print::faithful::nozzle_diameter_for_filament(
                print_config,
                cfg_int(object.printing_region(region_id).borrow().config(), "wall_filament") as u32,
            ) as f64;
            if nozzle > EPSILON && (min_nozzle <= EPSILON || nozzle < min_nozzle) {
                min_nozzle = nozzle;
            }
        }
        min_nozzle
    }

    /// PrintObject.cpp:5008-5030 `static Surfaces collect_span_bridges(const LayerPtrs &layers, size_t region_id,
    /// size_t bottom_idx, size_t top_idx, const ExPolygons &merge_area)`
    pub fn collect_span_bridges(
        layers: &[crate::layer::Layer],
        region_id: usize,
        bottom_idx: usize,
        top_idx: usize,
        merge_area: &[ExPolygon],
    ) -> Vec<crate::surface::Surface> {
        use crate::clipper_utils::{difference_clib, intersection_clib, union_ex_clib};
        let mut out: Vec<crate::surface::Surface> = Vec::new();
        let mut claimed: Vec<crate::geometry::Polygon> = Vec::new();

        let mut claim = |templ: &crate::surface::Surface, mut area: Vec<ExPolygon>| {
            if !claimed.is_empty() {
                area = difference_clib(&area, &union_ex_clib(&claimed, PFT_NON_ZERO));
            }
            for expolygon in area {
                let mut s = templ.clone();
                s.expolygon = expolygon;
                claimed.extend(crate::geometry::to_polygons_expoly(&s.expolygon));
                out.push(s);
            }
        };

        for layer_idx in bottom_idx..=top_idx {
            if let Some(layerm) = layers[layer_idx].get_region(region_id) {
                for surface in &layerm.fill_surfaces.surfaces {
                    if surface.is_bridge() {
                        claim(surface, intersection_clib(std::slice::from_ref(&surface.expolygon), merge_area));
                    }
                }
            }
        }
        out
    }

    impl PrintObject {
        /// PrintObject.cpp:1530-1534 `SupportLayer* PrintObject::add_tree_support_layer(int id, coordf_t height,
        /// coordf_t print_z, coordf_t slice_z)`
        pub fn add_tree_support_layer(&mut self, id: i32, height: f64, print_z: f64, slice_z: f64) -> &mut crate::print::faithful::SupportLayer {
            let object_id = self.m_model_object;
            self.m_support_layers.push(crate::print::faithful::SupportLayer::new(id as usize, 0, object_id, height, print_z, slice_z));
            self.m_support_layers.last_mut().unwrap()
        }

        /// PrintObject.cpp:1536-1540 `SupportLayer* PrintObject::add_support_layer(int id, int interface_id, coordf_t
        /// height, coordf_t print_z)`
        pub fn add_support_layer(&mut self, id: i32, interface_id: i32, height: f64, print_z: f64) -> &mut crate::print::faithful::SupportLayer {
            let object_id = self.m_model_object;
            self.m_support_layers.push(crate::print::faithful::SupportLayer::new(id as usize, interface_id as usize, object_id, height, print_z, -1.0));
            self.m_support_layers.last_mut().unwrap()
        }

        /// PrintObject.cpp:1542-1545 `SupportLayerPtrs::iterator PrintObject::insert_support_layer(SupportLayerPtrs::iterator
        /// pos, size_t id, size_t interface_id, coordf_t height, coordf_t print_z, coordf_t slice_z)`: the position
        /// is an index, the result the index of the new layer.
        pub fn insert_support_layer(&mut self, pos: usize, id: usize, interface_id: usize, height: f64, print_z: f64, slice_z: f64) -> usize {
            let object_id = self.m_model_object;
            self.m_support_layers.insert(pos, crate::print::faithful::SupportLayer::new(id, interface_id, object_id, height, print_z, slice_z));
            pos
        }

        /// PrintObject.cpp:202-324 `std::vector<std::set<int>> PrintObject::detect_extruder_geometric_unprintables() const`.
        /// The region of a layer region is the object's printing region of its index. The C++ runs the
        /// printable-area check over the layers in parallel with concurrent sets; running the layers in order is
        /// one of its schedules.
        pub fn detect_extruder_geometric_unprintables(&self, print: &Print) -> Vec<std::collections::BTreeSet<i32>> {
            use crate::clipper_utils::{difference_clib, intersection_clib, offset_polygons_clib, OffsetJoinType};
            use std::collections::BTreeSet;
            let extruder_size = crate::gcode::wipe_tower::cfg_floats(print.config(), "nozzle_diameter").len();
            if extruder_size == 1 {
                return vec![BTreeSet::new(); 1];
            }

            let mut geometric_unprintables: Vec<BTreeSet<i32>> = vec![BTreeSet::new(); extruder_size]; // the container to return

            let printable_height_per_extruder = crate::gcode::wipe_tower::cfg_floats(print.config(), "extruder_printable_height");
            debug_assert!(printable_height_per_extruder.len() == extruder_size);
            let filaments_of = |region_id: usize| {
                let region = self.printing_region(region_id);
                let region = region.borrow();
                (
                    cfg_int(region.config(), "wall_filament"),
                    cfg_int(region.config(), "solid_infill_filament"),
                    cfg_int(region.config(), "sparse_infill_filament"),
                )
            };

            // check unprintable filaments caused by printable height limit
            for extruder_id in 0..printable_height_per_extruder.len() {
                let printable_height = printable_height_per_extruder[extruder_id];
                for layer in &self.m_layers {
                    if layer.print_z <= printable_height {
                        continue;
                    }
                    for layerm in layer.regions() {
                        let (wall_filament, solid_infill_filament, sparse_infill_filament) = filaments_of(layerm.region_id());

                        if !layerm.fills.entities.is_empty() {
                            if solid_infill_filament > 0 {
                                geometric_unprintables[extruder_id].insert(solid_infill_filament - 1);
                            }
                            if sparse_infill_filament > 0 {
                                geometric_unprintables[extruder_id].insert(sparse_infill_filament - 1);
                            }
                        }
                        if !layerm.perimeters.entities.is_empty() && wall_filament > 0 {
                            geometric_unprintables[extruder_id].insert(wall_filament - 1);
                        }
                    }
                }
            }

            let mut tbb_geometric_unprintables: Vec<BTreeSet<i32>> = vec![BTreeSet::new(); extruder_size]; // the container used in tbb

            let mut unprintable_area_in_obj_coord = print.get_extruder_unprintable_polygons();
            let mut unprintable_area_bbox: Vec<crate::geometry::BoundingBox> = Vec::new();

            // transform the unprintable areas to obj coord is cheaper than thransform obj into world coord
            let shift = self.m_instances[0].shift_without_plate_offset(print);
            for polys in unprintable_area_in_obj_coord.iter_mut() {
                for poly in polys.iter_mut() {
                    poly.translate(Point::new(-shift.x, -shift.y));
                }
                // shrink(polys, float(SCALED_EPSILON)): the exclusion area shrunk slightly, so a slice only sharing an
                // edge with it does not count as intersecting.
                *polys = offset_polygons_clib(polys, -((crate::libslic3r::SCALED_EPSILON as f32) as f64), OffsetJoinType::Miter);
                unprintable_area_bbox.push(crate::geometry::BoundingBox::from_points(&polys.iter().flat_map(|p| p.points.iter().copied()).collect::<Vec<_>>()));
            }
            // BoundingBoxBase::overlap
            let overlap = |a: &crate::geometry::BoundingBox, b: &crate::geometry::BoundingBox| {
                !(a.max.x < b.min.x || a.min.x > b.max.x || a.max.y < b.min.y || a.min.y > b.max.y)
            };
            let extents = |expolys: &[ExPolygon]| crate::geometry::get_extents(expolys);

            // check unprintbale filaments caused by printable area limit
            for layer in &self.m_layers {
                for layerm in layer.regions() {
                    let (wall_filament, solid_infill_filament, sparse_infill_filament) = filaments_of(layerm.region_id());
                    let mut fill_expolys: Option<Vec<ExPolygon>> = None;
                    let mut fill_bbox = crate::geometry::BoundingBox::new();
                    let mut wall_expolys: Option<Vec<ExPolygon>> = None;
                    let mut wall_bbox = crate::geometry::BoundingBox::new();

                    for idx in 0..unprintable_area_in_obj_coord.len() {
                        let set = &mut tbb_geometric_unprintables[idx];
                        let do_infill_filament_detect = (solid_infill_filament > 0 && !set.contains(&(solid_infill_filament - 1)))
                            || (sparse_infill_filament > 0 && !set.contains(&(sparse_infill_filament - 1)));

                        let mut infill_unprintable = !layerm.fills.entities.is_empty()
                            && ((solid_infill_filament > 0 && set.contains(&(solid_infill_filament - 1)))
                                || (sparse_infill_filament > 0 && set.contains(&(sparse_infill_filament - 1))));

                        let area = expolygons_of_polygons(&unprintable_area_in_obj_coord[idx]);
                        if !layerm.fills.entities.is_empty() && do_infill_filament_detect {
                            if fill_expolys.is_none() {
                                fill_expolys = Some(layerm.fill_expolygons.clone());
                                fill_bbox = extents(fill_expolys.as_ref().unwrap());
                            }
                            if overlap(&fill_bbox, &unprintable_area_bbox[idx]) && !intersection_clib(fill_expolys.as_ref().unwrap(), &area).is_empty() {
                                if solid_infill_filament > 0 {
                                    set.insert(solid_infill_filament - 1);
                                }
                                if sparse_infill_filament > 0 {
                                    set.insert(sparse_infill_filament - 1);
                                }
                                infill_unprintable = true;
                            }
                        }

                        let do_wall_filament_detect = wall_filament > 0 && !set.contains(&(wall_filament - 1));
                        if !layerm.perimeters.entities.is_empty() && do_wall_filament_detect {
                            // if infill is unprintable, no need to check wall since wall contour surrounds infill contour
                            if infill_unprintable {
                                set.insert(wall_filament - 1);
                                continue;
                            }

                            if wall_expolys.is_none() {
                                if fill_expolys.is_none() {
                                    fill_expolys = Some(layerm.fill_expolygons.clone());
                                    fill_bbox = extents(fill_expolys.as_ref().unwrap());
                                }
                                wall_expolys = Some(difference_clib(&crate::surface::to_expolygons(&layerm.raw_slices.surfaces), fill_expolys.as_ref().unwrap()));
                                wall_bbox = extents(wall_expolys.as_ref().unwrap());
                            }

                            if overlap(&wall_bbox, &unprintable_area_bbox[idx]) && !intersection_clib(wall_expolys.as_ref().unwrap(), &area).is_empty() {
                                set.insert(wall_filament - 1);
                            }
                        }
                    }
                }
            }

            // add the elems in tbb container to final contianer
            for idx in 0..extruder_size {
                let elems: Vec<i32> = tbb_geometric_unprintables[idx].iter().copied().collect();
                geometric_unprintables[idx].extend(elems);
            }

            geometric_unprintables
        }

        /// PrintObjectSlice.cpp:21 `bool PrintObject::infill_only_where_needed = false;`
        pub const infill_only_where_needed: bool = false;

        /// PrintObject.cpp:4003-4088 `void PrintObject::clip_fill_surfaces()`: only active if
        /// infill_only_where_needed (never set at the pin). Trims the sparse infill, so it acts as an internal
        /// support; all other infill types are kept intact.
        pub fn clip_fill_surfaces(&mut self) {
            use crate::clipper_utils::{
                diff_polygons_raw_clib, difference_clib_safety, intersection_clib_safety, intersection_polygons_raw_clib, offset_polygons_clib,
                offset_polygons_clib_ml, OffsetJoinType,
            };
            use crate::surface::SurfaceType;
            if !PrintObject::infill_only_where_needed {
                return;
            }
            let mut has_infill = false;
            for i in 0..self.num_printing_regions() {
                if cfg_float(self.printing_region(i).borrow().config(), "sparse_infill_density") > 0. {
                    has_infill = true;
                    break;
                }
            }
            if !has_infill {
                return;
            }

            // We only want infill under ceilings; this is almost like an
            // internal support material.
            // Proceed top-down, skipping the bottom layer.
            let mut upper_internal: Vec<crate::geometry::Polygon> = Vec::new();
            let mut layer_id = self.m_layers.len() as i32 - 1;
            while layer_id > 0 {
                let (lid, llid) = (layer_id as usize, layer_id as usize - 1);
                // Detect things that we need to support.
                // Cummulative fill surfaces.
                let mut fill_surfaces: Vec<crate::geometry::Polygon> = Vec::new();
                // Solid surfaces to be supported.
                let mut overhangs: Vec<crate::geometry::Polygon> = Vec::new();
                for layerm in self.m_layers[lid].regions() {
                    for surface in &layerm.fill_surfaces.surfaces {
                        let polygons = crate::geometry::to_polygons_expoly(&surface.expolygon);
                        if surface.is_solid() {
                            overhangs.extend(polygons.iter().cloned());
                        }
                        fill_surfaces.extend(polygons);
                    }
                }
                let mut lower_layer_fill_surfaces: Vec<crate::geometry::Polygon> = Vec::new();
                let mut lower_layer_internal_surfaces: Vec<crate::geometry::Polygon> = Vec::new();
                for layerm in self.m_layers[llid].regions() {
                    for surface in &layerm.fill_surfaces.surfaces {
                        let polygons = crate::geometry::to_polygons_expoly(&surface.expolygon);
                        if surface.surface_type == SurfaceType::Internal || surface.surface_type == SurfaceType::InternalVoid {
                            lower_layer_internal_surfaces.extend(polygons.iter().cloned());
                        }
                        lower_layer_fill_surfaces.extend(polygons);
                    }
                }
                // We also need to support perimeters when there's at least one full unsupported loop
                {
                    let layer = &self.m_layers[lid];
                    // Get perimeters area as the difference between slices and fill_surfaces
                    // Only consider the area that is not supported by lower perimeters
                    let perimeters = intersection_polygons_raw_clib(
                        &diff_polygons_raw_clib(&crate::geometry::to_polygons(&layer.lslices), &fill_surfaces),
                        &lower_layer_fill_surfaces,
                    );
                    // Only consider perimeter areas that are at least one extrusion width thick.
                    let mut pw = f32::MAX;
                    for layerm in layer.regions() {
                        pw = pw.min(layerm.flow(crate::flow::FlowRole::Perimeter, layer.height).map(|f| f.scaled_width()).unwrap_or(0) as f32);
                    }
                    // Append such thick perimeters to the areas that need support: opening(perimeters, pw).
                    let shrunk = offset_polygons_clib(&perimeters, -(pw as f64), OffsetJoinType::Miter);
                    overhangs.extend(offset_polygons_clib(&shrunk, pw as f64, OffsetJoinType::Miter));
                }
                // Merge the new overhangs, find new internal infill.
                upper_internal.extend(overhangs);
                // static constexpr const auto closing_radius = scaled<float>(2.f);
                let closing_radius = (2.0f32 as f64 / SCALING_FACTOR) as f32 as f64;
                // closing(upper_internal, closing_radius, jtSquare, 0.)
                let closed = offset_polygons_clib_ml(
                    &offset_polygons_clib_ml(&upper_internal, closing_radius, OffsetJoinType::Square, 0.),
                    -closing_radius,
                    OffsetJoinType::Square,
                    0.,
                );
                upper_internal = intersection_polygons_raw_clib(
                    // Regularize the overhang regions, so that the infill areas will not become excessively jagged.
                    &crate::mutable_polygon::smooth_outward_polygons(closed, (0.1 / SCALING_FACTOR) as i64),
                    &lower_layer_internal_surfaces,
                );
                // Apply new internal infill to regions.
                let upper = expolygons_of_polygons(&upper_internal);
                for region_id in 0..self.m_layers[llid].regions().len() {
                    let density = cfg_float(self.printing_region(self.m_layers[llid].regions()[region_id].region_id()).borrow().config(), "sparse_infill_density");
                    let layerm = &mut self.m_layers[llid].regions_mut()[region_id];
                    if density == 0. {
                        continue;
                    }
                    let mut internal: Vec<ExPolygon> = Vec::new();
                    for surface in &layerm.fill_surfaces.surfaces {
                        if surface.surface_type == SurfaceType::Internal || surface.surface_type == SurfaceType::InternalVoid {
                            internal.push(surface.expolygon.clone());
                        }
                    }
                    layerm.fill_surfaces.remove_types(&[SurfaceType::Internal, SurfaceType::InternalVoid]);
                    layerm.fill_surfaces.append(intersection_clib_safety(&internal, &upper), SurfaceType::Internal);
                    layerm.fill_surfaces.append(difference_clib_safety(&internal, &upper), SurfaceType::InternalVoid);
                    // If there are voids it means that our internal infill is not adjacent to
                    // perimeters. In this case it would be nice to add a loop around infill to
                    // make it more robust and nicer. TODO.
                }
                layer_id -= 1;
            }
        }

        /// PrintObject.cpp:328-465 `std::unordered_map<int, std::unordered_map<int, double>>
        /// PrintObject::calc_estimated_filament_print_time() const`. The region of a layer region is the object's
        /// printing region of its index.
        pub fn calc_estimated_filament_print_time(&self, print: &Print) -> std::collections::HashMap<i32, std::collections::HashMap<i32, f64>> {
            use crate::extrusion_entity::{ExtrusionEntity, ExtrusionEntityType, ExtrusionLoop, ExtrusionMultiPath, ExtrusionPath, ExtrusionRole};
            use crate::gcode::wipe_tower::{cfg_bools, cfg_floats, cfg_ints, cfg_strings};
            use std::collections::HashMap;
            let full_print_config = &print.base.base.m_ori_full_print_config;
            let extruder_variant_list = cfg_strings(print.config(), "printer_extruder_variant");

            if !full_print_config.has("filament_extruder_variant")
                || !full_print_config.has("filament_self_index")
                || !full_print_config.has("filament_max_volumetric_speed")
                || !full_print_config.has("filament_flow_ratio")
            {
                log::warn!("filament_* config option missing, skip estimation");
                return HashMap::new();
            }
            let filament_variant_list = cfg_strings(full_print_config, "filament_extruder_variant");
            let filament_self_idx = cfg_ints(full_print_config, "filament_self_index");
            let filament_max_volumetric_speed = cfg_floats(full_print_config, "filament_max_volumetric_speed");
            let filament_flow_ratio = cfg_floats(full_print_config, "filament_flow_ratio");

            let get_limit_from_volumetric_speed = |filament_idx: i32, extruder_idx: i32, width: f64, height: f64| -> f64 {
                let extruder_variant = &extruder_variant_list[extruder_idx as usize];

                let mut idx_for_filament = 0usize;
                for idx in 0..filament_self_idx.len() {
                    if filament_self_idx[idx] - 1 == filament_idx && *extruder_variant == filament_variant_list[idx] {
                        idx_for_filament = idx;
                        break;
                    }
                }

                let max_volumetric_speed = filament_max_volumetric_speed[idx_for_filament];
                let flow_ratio = filament_flow_ratio[idx_for_filament];

                // height * (width - height * (1 - PI / 4)) * flow_ratio: the inner difference contracts to an fma.
                let mm3_per_mm = height * (-height).mul_add(1.0 - std::f64::consts::PI / 4.0, width) * flow_ratio;
                max_volumetric_speed / mm3_per_mm
            };

            let print_config = print.config();
            let get_speed_from_role_with_filament = |role: ExtrusionRole, filament_id: i32, region_config: &PrintRegionConfig, extruder_id: i32| -> f64 {
                let e = extruder_id as usize;
                match role {
                    ExtrusionRole::Perimeter => cfg_floats(region_config, "inner_wall_speed")[e],
                    ExtrusionRole::ExternalPerimeter => cfg_floats(region_config, "outer_wall_speed")[e],
                    ExtrusionRole::OverhangPerimeter | ExtrusionRole::BridgeInfill | ExtrusionRole::SupportTransition => {
                        let use_filament_bridge_speed = crate::gcode::wipe_tower::get_at(&cfg_bools(print_config, "filament_enable_overhang_speed"), filament_id as usize);

                        if use_filament_bridge_speed {
                            cfg_floats(print_config, "filament_bridge_speed")[filament_id as usize]
                        } else {
                            cfg_floats(region_config, "bridge_speed")[e]
                        }
                    }
                    ExtrusionRole::InternalInfill => cfg_floats(region_config, "sparse_infill_speed")[e],
                    ExtrusionRole::FloatingVerticalShell | ExtrusionRole::SolidInfill => cfg_floats(region_config, "internal_solid_infill_speed")[e],
                    ExtrusionRole::TopSolidInfill => cfg_floats(region_config, "top_surface_speed")[e],
                    ExtrusionRole::BottomSurface => cfg_floats(print_config, "initial_layer_speed")[e],
                    ExtrusionRole::GapFill => cfg_floats(region_config, "gap_infill_speed")[e],
                    _ => cfg_floats(region_config, "internal_solid_infill_speed")[e],
                }
            };

            let mut filament_print_time: HashMap<i32, HashMap<i32, f64>> = HashMap::new();

            fn process_paths(
                paths: &[ExtrusionPath],
                filament: i32,
                eidx: i32,
                speed_from_config: f64,
                limit: &dyn Fn(i32, i32, f64, f64) -> f64,
                time: &mut HashMap<i32, HashMap<i32, f64>>,
            ) {
                for path in paths {
                    let speed_from_filament = limit(filament, eidx, path.width, path.height);
                    let speed_applied = speed_from_filament.min(speed_from_config);
                    *time.entry(filament).or_default().entry(eidx).or_insert(0.0) += path.length() * SCALING_FACTOR / speed_applied;
                }
            }
            fn process_entity(
                filament: i32,
                eidx: i32,
                entity: &ExtrusionEntityType,
                speed_from_config: f64,
                limit: &dyn Fn(i32, i32, f64, f64) -> f64,
                time: &mut HashMap<i32, HashMap<i32, f64>>,
            ) {
                match entity {
                    // 如果是集合类型，则递归处理其子entity
                    ExtrusionEntityType::Collection(collection) => {
                        for sub_entity in &collection.entities {
                            process_entity(filament, eidx, sub_entity, speed_from_config, limit, time);
                        }
                    }
                    ExtrusionEntityType::Path(path) => process_paths(std::slice::from_ref(path), filament, eidx, speed_from_config, limit, time),
                    ExtrusionEntityType::Loop(eloop) => process_paths(&eloop.paths, filament, eidx, speed_from_config, limit, time),
                    ExtrusionEntityType::MultiPath(empath) => process_paths(&empath.paths, filament, eidx, speed_from_config, limit, time),
                }
            }
            // ExtrusionEntityCollection.hpp:54-61 `ExtrusionRole role() const` for a collection, the entity's own role otherwise.
            fn entity_role(entity: &ExtrusionEntityType) -> ExtrusionRole {
                match entity {
                    ExtrusionEntityType::Path(p) => p.role,
                    ExtrusionEntityType::MultiPath(m) => ExtrusionEntity::role(m as &ExtrusionMultiPath),
                    ExtrusionEntityType::Loop(l) => ExtrusionEntity::role(l as &ExtrusionLoop),
                    ExtrusionEntityType::Collection(c) => {
                        let mut out = ExtrusionRole::None;
                        for ee in &c.entities {
                            let er = entity_role(ee);
                            out = if out == ExtrusionRole::None || out == er { er } else { ExtrusionRole::Mixed };
                        }
                        out
                    }
                }
            }

            // Iterate through all layers and regions to calculate the print area for each filament.
            let extruder_num = cfg_floats(print.config(), "nozzle_diameter").len() as i32;
            for layer in &self.m_layers {
                for layerm in layer.regions() {
                    let region = self.printing_region(layerm.region_id());
                    let region = region.borrow();
                    let region_config = region.config();

                    let entity_list: [&Vec<ExtrusionEntityType>; 3] = [&layerm.fills.entities, &layerm.thin_fills.entities, &layerm.perimeters.entities];
                    for eidx in 0..extruder_num {
                        for entities in entity_list.iter() {
                            for entity in entities.iter() {
                                let role = entity_role(entity);
                                let filament: i32;
                                if crate::extrusion_entity::is_perimeter(role) {
                                    filament = cfg_int(region_config, "wall_filament") - 1;
                                } else if crate::extrusion_entity::is_solid_infill(role) {
                                    filament = cfg_int(region_config, "solid_infill_filament") - 1;
                                } else if crate::extrusion_entity::is_infill(role) {
                                    filament = cfg_int(region_config, "sparse_infill_filament") - 1;
                                } else {
                                    continue;
                                }

                                let speed_from_config = get_speed_from_role_with_filament(role, filament, region_config, eidx);
                                process_entity(filament, eidx, entity, speed_from_config, &get_limit_from_volumetric_speed, &mut filament_print_time);
                            }
                        }
                    }
                }
            }
            filament_print_time
        }

        /// PrintObject.cpp:1925-1940 `void PrintObject::reset_slice_surfaces(const
        /// std::vector<std::vector<SurfaceCollection>> &slice_surfaces_cpy)`: reset infill surface and slice
        /// data back.
        pub fn reset_slice_surfaces(&mut self, slice_surfaces_cpy: &[Vec<crate::surface_collection::SurfaceCollection>]) {
            use crate::print_config::enums::InfillPattern;
            use crate::surface::SurfaceType;
            for region_id in 0..self.num_printing_regions() {
                let (instead, locked_zag) = {
                    let region = self.printing_region(region_id);
                    let region = region.borrow();
                    (
                        region.config().opt_bool("infill_instead_top_bottom_surfaces"),
                        cfg_int(region.config(), "sparse_infill_pattern") == InfillPattern::IpLockedZag as i32,
                    )
                };
                for idx_layer in 0..self.m_layers.len() {
                    let layerm = &mut self.m_layers[idx_layer].regions_mut()[region_id];
                    if instead && locked_zag {
                        layerm.slices = slice_surfaces_cpy[idx_layer][region_id].clone();
                        let mut exps: Vec<ExPolygon> = Vec::new();
                        layerm.fill_surfaces.keep_type_collect_exps(SurfaceType::Internal, &mut exps);
                        layerm.fill_surfaces.append(exps, SurfaceType::Top);
                    }
                }
            }
        }

        /// PrintObject.cpp:2690-2696 `bool PrintObject::has_interlocking_beam() const`
        pub fn has_interlocking_beam(&self) -> bool {
            for region_id in 0..self.num_printing_regions() {
                if self.printing_region(region_id).borrow().config().opt_bool("interlocking_beam") {
                    return true;
                }
            }
            false
        }

        /// PrintObject.cpp:2698-2749 `void PrintObject::discover_shell_for_perimeters()`. The C++ runs the layers
        /// in parallel; each one reads its own `fill_expolygons` and writes the `fill_surfaces` of the layer
        /// below, so running them in order gives the same result.
        pub fn discover_shell_for_perimeters(&mut self) {
            use crate::clipper_utils::{difference_clib, intersection_clib, offset_expolygons_clib_scaled, union_ex_clib, OffsetJoinType};
            use crate::surface::SurfaceType;
            let num_regions = self.num_printing_regions();

            for idx_layer in 0..self.m_layers.len() {
                let lower_idx = match self.m_layers[idx_layer].lower_layer_id {
                    Some(i) => i,
                    None => continue,
                };

                let mut infill_areas: Vec<ExPolygon> = Vec::new();
                let mut max_line_width: f32 = 0.0;
                let perimeter_areas = {
                    let layer = &self.m_layers[idx_layer];
                    for region_id in 0..num_regions {
                        let layerm = &layer.regions()[region_id];
                        let extflow = layerm.flow(crate::flow::FlowRole::ExternalPerimeter, layer.height);
                        infill_areas.extend(layerm.fill_expolygons.iter().cloned());
                        if let Ok(extflow) = extflow {
                            max_line_width = max_line_width.max(0.5f32 * (extflow.scaled_width() + extflow.scaled_spacing()) as f32);
                        }
                    }
                    infill_areas = union_ex_clib(&crate::geometry::to_polygons(&infill_areas), PFT_NON_ZERO);
                    offset_expolygons_clib_scaled(&difference_clib(&layer.lslices, &infill_areas), max_line_width as f64, OffsetJoinType::Miter)
                };

                for region_id in 0..num_regions {
                    let lower_layerm = &mut self.m_layers[lower_idx].regions_mut()[region_id];

                    let mut new_perimeter_solid = intersection_clib(&perimeter_areas, &lower_layerm.fill_expolygons);
                    new_perimeter_solid.retain(|expoly| {
                        !crate::ex_polygon::is_narrow_expolygon(expoly, 3.0 * max_line_width as f64, (1.0 / SCALING_FACTOR) * (1.0 / SCALING_FACTOR), 0.1)
                    });
                    if new_perimeter_solid.is_empty() {
                        continue;
                    }

                    let old_internal = crate::surface::to_expolygons_ptr(&lower_layerm.fill_surfaces.filter_by_type(SurfaceType::Internal));
                    let old_internal_void = crate::surface::to_expolygons_ptr(&lower_layerm.fill_surfaces.filter_by_type(SurfaceType::InternalVoid));
                    let old_internal_solid = crate::surface::to_expolygons_ptr(&lower_layerm.fill_surfaces.filter_by_type(SurfaceType::InternalSolid));
                    let bottom_area = crate::surface::to_expolygons_ptr(&lower_layerm.fill_surfaces.filter_by_type(SurfaceType::Bottom));

                    lower_layerm.fill_surfaces.remove_types(&[SurfaceType::Internal, SurfaceType::InternalVoid, SurfaceType::InternalSolid]);

                    let mut solid_and_new = crate::geometry::to_polygons(&old_internal_solid);
                    solid_and_new.extend(crate::geometry::to_polygons(&new_perimeter_solid));
                    let new_internal_solid = difference_clib(&union_ex_clib(&solid_and_new, PFT_NON_ZERO), &bottom_area);
                    let new_internal = difference_clib(&old_internal, &new_perimeter_solid);
                    let new_internal_void = difference_clib(&old_internal_void, &new_perimeter_solid);
                    lower_layerm.fill_surfaces.append(new_internal, SurfaceType::Internal);
                    lower_layerm.fill_surfaces.append(new_internal_void, SurfaceType::InternalVoid);
                    lower_layerm.fill_surfaces.append(new_internal_solid, SurfaceType::InternalSolid);
                }
            }
        }

        /// PrintObject.cpp:5724-5832 `void PrintObject::combine_infill()`: combine fill surfaces to honor the
        /// "infill every N layers" option.
        pub fn combine_infill(&mut self, print: &Print) {
            use crate::clipper_utils::{difference_clib, intersection_clib, offset_expolygons_clib_scaled, OffsetJoinType};
            use crate::print_config::enums::InfillPattern;
            use crate::surface::SurfaceType;
            // Work on each region separately.
            for region_id in 0..self.num_printing_regions() {
                let region = self.printing_region(region_id);
                let region = region.borrow();
                //BBS
                let enable_combine_infill = region.config().opt_bool("infill_combination");
                if !enable_combine_infill || cfg_float(region.config(), "sparse_infill_density") == 0. {
                    continue;
                }
                // Limit the number of combined layers to the sparse-infill nozzle diameter. Solid fill is not
                // combined here and must not constrain the sparse-infill band size.
                //FIXME limit the layer height to max_layer_height
                let nozzle_diameter =
                    crate::print::faithful::nozzle_diameter_for_filament(print.config(), cfg_int(region.config(), "sparse_infill_filament") as u32) as f64;
                // define the combinations
                let mut combine: Vec<usize> = vec![0; self.m_layers.len()];
                {
                    let mut current_height = 0.0;
                    let mut num_layers = 0usize;
                    for layer_idx in 0..self.m_layers.len() {
                        let layer = &self.m_layers[layer_idx];
                        if layer.id() == 0 {
                            // Skip first print layer (which may not be first layer in array because of raft).
                            continue;
                        }
                        // Check whether the combination of this layer with the lower layers' buffer
                        // would exceed max layer height or max combined layer count.
                        // BBS: automatically calculate how many layers should be combined
                        if current_height + layer.height > nozzle_diameter + EPSILON {
                            // Append combination to lower layer.
                            combine[layer_idx - 1] = num_layers;
                            current_height = 0.;
                            num_layers = 0;
                        }
                        current_height += layer.height;
                        num_layers += 1;
                    }

                    // Append lower layers (if any) to uppermost layer.
                    let last = self.m_layers.len() - 1;
                    combine[last] = num_layers;
                }

                let pattern = cfg_int(region.config(), "sparse_infill_pattern");
                // loop through layers to which we have assigned layers to combine
                for layer_idx in 0..self.m_layers.len() {
                    let num_layers = combine[layer_idx];
                    if num_layers <= 1 {
                        continue;
                    }
                    // Get all the LayerRegion objects to be combined: the layers layer_idx + 1 - num_layers ..= layer_idx.
                    let first = layer_idx + 1 - num_layers;
                    // We need to perform a multi-layer intersection, so let's split it in pairs.
                    // Initialize the intersection with the candidates of the lowest layer.
                    let mut intersection =
                        crate::surface::to_expolygons_ptr(&self.m_layers[first].regions()[region_id].fill_surfaces.filter_by_type(SurfaceType::Internal));
                    // Start looping from the second layer and intersect the current intersection with it.
                    for i in (first + 1)..=layer_idx {
                        intersection = intersection_clib(
                            &crate::surface::to_expolygons_ptr(&self.m_layers[i].regions()[region_id].fill_surfaces.filter_by_type(SurfaceType::Internal)),
                            &intersection,
                        );
                    }
                    let area_threshold = self.m_layers[first].regions()[region_id].infill_area_threshold();
                    if !intersection.is_empty() && area_threshold > 0. {
                        intersection.retain(|expoly| !(expoly.area() <= area_threshold));
                    }
                    if intersection.is_empty() {
                        continue;
                    }
                    // intersection now contains the regions that can be combined across the full amount of layers,
                    // so let's remove those areas from all layers.
                    let (perimeter_w, solid_w) = {
                        let top = &self.m_layers[layer_idx];
                        let layerm = &top.regions()[region_id];
                        (
                            layerm.flow(crate::flow::FlowRole::Perimeter, top.height).map(|f| f.scaled_width()).unwrap_or(0) as f32,
                            layerm.flow(crate::flow::FlowRole::SolidInfill, top.height).map(|f| f.scaled_width()).unwrap_or(0) as f32,
                        )
                    };
                    let grows = pattern == InfillPattern::IpRectilinear as i32
                        || pattern == InfillPattern::IpMonotonic as i32
                        || pattern == InfillPattern::IpGrid as i32
                        || pattern == InfillPattern::Ip2DLattice as i32
                        || pattern == InfillPattern::IpLine as i32
                        || pattern == InfillPattern::IpHoneycomb as i32;
                    let clearance_offset: f32 = 0.5f32 * perimeter_w
                        // Because fill areas for rectilinear and honeycomb are grown
                        // later to overlap perimeters, we need to counteract that too.
                        + (if grows { 1.5f32 } else { 0.5f32 }) * solid_w;
                    let mut intersection_with_clearance: Vec<crate::geometry::Polygon> = Vec::with_capacity(intersection.len());
                    for expoly in &intersection {
                        // offset(const ExPolygon&, float delta): jtMiter, miter limit 3, as polygons.
                        for e in offset_expolygons_clib_scaled(std::slice::from_ref(expoly), clearance_offset as f64, OffsetJoinType::Miter) {
                            intersection_with_clearance.extend(crate::geometry::to_polygons_expoly(&e));
                        }
                    }
                    let clearance = expolygons_of_polygons(&intersection_with_clearance);
                    let mut thickness = 0.0;
                    for i in first..=layer_idx {
                        thickness += self.m_layers[i].height;
                    }
                    for i in first..=layer_idx {
                        let layerm = &mut self.m_layers[i].regions_mut()[region_id];
                        let internal = crate::surface::to_expolygons_ptr(&layerm.fill_surfaces.filter_by_type(SurfaceType::Internal));
                        layerm.fill_surfaces.remove_type(SurfaceType::Internal);
                        layerm.fill_surfaces.append(difference_clib(&internal, &clearance), SurfaceType::Internal);
                        if i == layer_idx {
                            // Apply surfaces back with adjusted depth to the uppermost layer.
                            let mut templ = crate::surface::Surface::new(SurfaceType::Internal, ExPolygon::default());
                            templ.thickness = thickness;
                            templ.thickness_layers = num_layers;
                            layerm.fill_surfaces.append_expolygons_templ(intersection.clone(), &templ);
                        } else {
                            // Save void surfaces.
                            layerm.fill_surfaces.append(intersection_clib(&internal, &clearance), SurfaceType::InternalVoid);
                        }
                    }
                }
            }
        }
    }
}
