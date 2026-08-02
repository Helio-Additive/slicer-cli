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
            slicing_params: crate::slicing::SlicingParams::default(),
            shared_regions: None,
            typed_slices: false,
            painted_submeshes: Vec::new(),
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
            slicing_params: crate::slicing::SlicingParams::default(),
            shared_regions: None,
            typed_slices: false,
            painted_submeshes: Vec::new(),
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
        slicer.set_slice_center_offset(cx_mm, cy_mm);
        // Record for the export origin (consumed by Print after process): gcode =
        // unscale(centered) + center → C++'s raw frame (R81: m_origin = unscale(shift)
        // = center, instance=0). Only meaningful when SLICE_CENTER is on.
        self.slice_center_offset = (cx_mm, cy_mm);

        // Perform actual mesh slicing
        // PrintObjectSlice.cpp:801
        let layers = slicer.slice(mesh)?;

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

        // Build lslices for each layer (union of all region slices)
        // C++: PrintObjectSlice.cpp calls layer->make_slices() which populates
        // lslices from region slices. This is needed for detect_surfaces_type()
        // to correctly diff between adjacent layers.
        let __t_ms = std::time::Instant::now();
        for layer in &mut self.layers {
            layer.make_slices();
        }
        if std::env::var_os("SLICE_PHASE_TIMING").is_some() {
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
        if num_regions > 1 && !self.painted_submeshes.is_empty() {
            self.apply_mm_segmentation_tier1()?;
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

        // Mark step as complete
        // PrintObjectSlice.cpp:843
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
        let mms_center_offset = if crate::faithful_gate("SLICE_CENTER")
            && (self.slice_center_offset.0 != 0.0 || self.slice_center_offset.1 != 0.0)
        {
            crate::geometry::Point::new_scale(
                self.slice_center_offset.0,
                self.slice_center_offset.1,
            )
        } else {
            crate::geometry::Point::new(0, 0)
        };
        let segmented = crate::multi_material_segmentation::multi_material_segmentation_by_painting_tier1(
            &layer_slices,
            &layer_zs,
            &self.painted_submeshes,
            num_extruders,
            0.0,
            0.0,
            mms_center_offset,
        );

        // Move painted areas out of region 0 into the painted regions.
        // PrintObjectSlice.cpp:855-925 (single-parent collapse).
        let painted_order: Vec<u8> = self.painted_submeshes.iter().map(|(e, _)| *e).collect();
        if std::env::var_os("MMS_DEBUG").is_some() {
            let seg_nonempty = segmented
                .iter()
                .filter(|l| l.iter().any(|s| !s.is_empty()))
                .count();
            let slices_nonempty = layer_slices.iter().filter(|s| !s.is_empty()).count();
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
            let mut stolen_total: crate::geometry::ExPolygons = Vec::new();
            for (i, &extruder) in painted_order.iter().enumerate() {
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
                let stolen = crate::clipper_utils::intersection(seg, region0_ex);
                if stolen.is_empty() {
                    continue;
                }
                let region_id = 1 + i;
                if region_id >= layer.regions().len() {
                    continue;
                }
                let mut coll = SurfaceCollection::new();
                for ex in &stolen {
                    coll.push(Surface::new(SurfaceType::Internal, ex.clone()));
                }
                layer.regions_mut()[region_id].set_slices(coll);
                stolen_total.extend(stolen);
            }
            if !stolen_total.is_empty() {
                // Remainder stays in region 0.
                let remaining = crate::clipper_utils::difference(region0_ex, &stolen_total);
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
        for layer in &mut self.layers {
            layer.wire_config_hierarchy(&object_config, &print_config, &all_regions);
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
        let __mp_t = std::env::var_os("SLICE_PHASE_TIMING").is_some();
        let __t_slice = std::time::Instant::now();
        self.slice()?;
        let __slice_s = __t_slice.elapsed().as_secs_f64();
        let __t_peri = std::time::Instant::now();

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
            if idx == 0
                && std::env::var("L0_EFC").is_ok()
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
        let __pi_t = std::env::var_os("SLICE_PHASE_TIMING").is_some();
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
        pi_phase!("discover_vertical_shells", self.discover_vertical_shells()?);
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
        if self.canceled.load(std::sync::atomic::Ordering::Relaxed) {
            return Err(crate::Error::Cancelled);
        }

        /// PrintObject.cpp:725-727
        /// C++: this->combine_infill();
        /// C++: m_print->throw_if_canceled();
        // TODO: Port combine_infill()

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
        let union2 = |a: &[Polygon], b: &[Polygon]| -> Vec<Polygon> {
            let mut all = a.to_vec();
            all.extend_from_slice(b);
            crate::geometry::to_polygons(&union_polygons_ex(&all))
        };
        // C++ `intersection(Polygons, Polygons)` -> Polygons
        let intersect_polys = |a: &[Polygon], b: &[Polygon]| -> Vec<Polygon> {
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
        let __bt = std::env::var_os("SLICE_PHASE_TIMING").is_some();
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
            let spacing = {
                let region0 = &layers_ref[lidx].regions()[0];
                region0.flow(FlowRole::SolidInfill, layer_height)?.spacing()
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
                        worth_bridging =
                            crate::geometry::to_polygons(&crate::clipper_utils::intersection(
                                &union_polygons_ex(&closed),
                                std::slice::from_ref(s_ex),
                            ));

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
                            layers_ref[lower_idx].generate_sparse_infill_polylines_for_anchoring()?,
                        ))
                    })
                    .collect()
            };
            for (lower_idx, lines) in anchor_results? {
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
                let spacing_mm = bridging_flow.spacing();
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
                        intersection_pl(
                            lines,
                            &union_polygons_ex(&shrink_p(&expansion_area, spacing_mm)),
                        )
                    })
                    .unwrap_or_default();
                let internal_unsupported_area = shrink_p(&deep_infill_area, spacing_mm * 4.5);

                // PrintObject.cpp:2853-2937 — per-candidate expansion.
                let cands_snapshot = jobs[job_idx].1.clone();
                let mut expanded_surfaces: Vec<CandidateSurface> =
                    Vec::with_capacity(cands_snapshot.len());
                for candidate in &cands_snapshot {
                    let flow = {
                        let r = &layers[lidx].regions()[candidate.region_idx];
                        r.bridging_flow(FlowRole::SolidInfill, true, layer_height)?
                    };
                    let flow_spacing_mm = flow.spacing();
                    let flow_scaled_spacing = flow.scaled_spacing();
                    let flow_scaled_width = flow.scaled_width();

                    // PrintObject.cpp:2857-2866 — area_to_be_bridge.
                    let mut area_to_be_bridge = expand_p(&candidate.new_polys, flow_spacing_mm);
                    area_to_be_bridge = intersect_polys(&area_to_be_bridge, &deep_infill_area);
                    let mut area_ex = crate::clipper_utils::union_ex(
                        &union_polygons_ex(&area_to_be_bridge),
                    );
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
                    let mut boundary_plines: Vec<Polyline> =
                        ex_to_polylines(&union_polygons_ex(&expand_p(
                            &total_fill_area,
                            1.3 * flow_spacing_mm,
                        )));
                    {
                        let limiting_plines = ex_to_polylines(&union_polygons_ex(&expand_p(
                            &limiting_area,
                            0.3 * flow_spacing_mm,
                        )));
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

                    // PrintObject.cpp:2919-2928 — clean & clip.
                    bridging_area = opening_p(&bridging_area, flow_spacing_mm);
                    bridging_area = closing_p(&bridging_area, flow_spacing_mm);
                    bridging_area = intersect_polys(&bridging_area, &limiting_area);
                    bridging_area = intersect_polys(&bridging_area, &total_fill_area);
                    bridging_area = diff_polygons(&bridging_area, &top_area);
                    bridging_area = opening_p(&bridging_area, flow_spacing_mm);
                    bridging_area = closing_p(&bridging_area, flow_spacing_mm);
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
                    row.push(self.layers[l].regions()[r].flow(FlowRole::SolidInfill, h)?.spacing());
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
                let mut new_internal_infills =
                    crate::clipper_utils::difference(&internal_exs, &union_polygons_ex(&cut_from_infill));
                new_internal_infills =
                    crate::clipper_utils::difference(&new_internal_infills, &additional_ensuring);
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
                        let cs_union = union_polygons_ex(&cs.new_polys);
                        let clipped = crate::clipper_utils::intersection(&cs_union, &fill_expolys);
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
                let mut new_internal_solids =
                    crate::clipper_utils::difference(&new_internal_solids, &union_polygons_ex(&cut_from_infill));
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
        let __timing = std::env::var_os("SLICE_PHASE_TIMING").is_some();
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
            let lower_snapshots: Vec<(
                Vec<crate::geometry::ExPolygon>,
                Vec<crate::geometry::Polygon>,
            )> = (0..self.layers.len())
                .into_par_iter()
                .map(|layer_idx| {
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
                        let lines = lower
                            .generate_sparse_infill_polylines_for_anchoring()
                            .unwrap_or_default();
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
                        let delta = internal_infill_width / 2.0;
                        let mut grown: Vec<crate::geometry::Polygon> = Vec::new();
                        for pl in &lines {
                            grown.extend(crate::clipper_utils::offset_polyline(pl, delta));
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

                (lower_internal_areas, lower_sparse_polys)
                })
                .collect();

            // Call Layer::make_fills() on each layer
            // PrintObject.cpp:768
            let canceled = self.canceled.clone();
            self.layers
                .par_iter_mut()
                .enumerate()
                .try_for_each(|(layer_idx, layer)| -> Result<()> {
                    if canceled.load(Ordering::Relaxed) {
                        return Err(Error::Cancelled);
                    }
                    let (lower_internal_areas, lower_sparse_polys) =
                        &lower_snapshots[layer_idx];
                    layer.make_fills(lower_internal_areas, lower_sparse_polys)
                })?;

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
                    region.simplify_thin_fill_extrusion_entity();
                }
            }
            self.set_step_done(PrintObjectStep::SimplifyInfill);
        }
    }

    /// Generate support material using SupportGenerator from support/
    /// PrintObject.cpp:856-901
    pub fn generate_support_material(&mut self) -> Result<()> {
        use crate::support::{SupportConfig, SupportGenerator, SupportType as SupportGenType};

        // PrintObject.cpp:857
        // C++: if (this->set_started(posSupportMaterial)) {
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

    /// Generate ironing paths for top surfaces on each layer.
    /// PrintObject.cpp:840-855 (Layer::make_ironing)
    pub fn ironing(&mut self) -> Result<()> {
        use crate::gcode::ironing::{
            IroningConfig as GcIroningConfig, IroningGenerator, IroningType as GcIroningType,
        };
        use crate::print_config::IroningType;

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
                for (rid, lr) in layer.regions().iter().enumerate() {
                    // PrintObject.cpp:1786
                    let exp = lr.flow(FlowRole::SolidInfill, lh)?.spacing()
                        * TOP_BOTTOM_EXPANSION_COEFF;
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
                    }
                    // PrintObject.cpp:1803
                    holes.extend(lr.fill_expolygons.iter().cloned());
                }
                // PrintObject.cpp:1805-1816 — simulate the perimeter / infill split as if
                // only a single extruder had printed the whole layer: grow lslices to force
                // the per-region islands to merge, then shrink by the widest perimeter band.
                // NOTE: C++ `offset2(a, +d1, -d2)` is GROW-then-SHRINK, the opposite order
                // to `clipper_utils::offset2`, hence the explicit grow()/shrink() pair.
                if perimeter_offset > 0.0 {
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
                if faithful() && std::env::var("VSHELL_REG_QUANT").is_ok() {
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
                let exp = if faithful() && std::env::var("VSHELL_REG_QUANT").is_ok() {
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
                let min_pis = if faithful() && std::env::var("VSHELL_REG_QUANT").is_ok() {
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
                let diff_int_holes = if holes.is_empty() {
                    internal_all.clone()
                } else {
                    difference(&internal_all, &holes)
                };
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
                    && std::env::var("VSHELL_REG_QUANT").is_ok()
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
                let (thr15, thr8) = if faithful() && std::env::var("VSHELL_REG_QUANT").is_ok() {
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

        // PrintObject.cpp:1720 — for each printing region.
        for region_id in 0..self.num_printing_regions() {
            // PrintObject.cpp:1722-1731 — for each layer, drive the LayerRegion member.
            for layer in &mut self.layers {
                // PrintObject.cpp:1726
                // C++: m_print->throw_if_canceled();
                if self.canceled.load(std::sync::atomic::Ordering::Relaxed) {
                    return Err(crate::Error::Cancelled);
                }
                // PrintObject.cpp:1728 — m_layers[layer_idx]->get_region(region_id)->process_external_surfaces(...)
                // layer.height mirrors C++ m_layer->height threaded into LayerRegion::flow.
                let layer_height = layer.height;
                if let Some(region) = layer.regions_mut().get_mut(region_id) {
                    region.process_external_surfaces(layer_height)?;
                }
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

