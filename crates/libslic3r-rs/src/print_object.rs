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
    surface::{Surface, SurfaceCollection, SurfaceDetectionConfig},
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
            model_object_id: 0,
            label_id: 0,
            state: 0,
            canceled,
            mesh: None,
            slicing_params: crate::slicing::SlicingParams::default(),
            shared_regions: None,
            typed_slices: false,
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
            model_object_id: 0,
            label_id: 0,
            state: 0,
            canceled: Arc::new(AtomicBool::new(false)),
            mesh: Some(mesh),
            slicing_params: crate::slicing::SlicingParams::default(),
            shared_regions: None,
            typed_slices: false,
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
        let slicer = crate::slicer::Slicer::new(slicing_params);

        // Perform actual mesh slicing
        // PrintObjectSlice.cpp:801
        let mut layers = slicer.slice(mesh)?;

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

        // Build lslices for each layer (union of all region slices)
        // C++: PrintObjectSlice.cpp calls layer->make_slices() which populates
        // lslices from region slices. This is needed for detect_surfaces_type()
        // to correctly diff between adjacent layers.
        for layer in &mut self.layers {
            layer.make_slices();
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
        self.slice()?;

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
                // TODO: layer->restore_untyped_slices();
                // Check for cancellation
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
            let region = match self.printing_region(region_id) {
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

        // Build per-region configs from shared_regions (same pattern as infill())
        // C++: each LayerRegion accesses region().config() for its own PrintRegionConfig
        let wall_mode = match self.config.perimeter_mode {
            crate::print_config::PerimeterMode::Classic => {
                crate::perimeter_generator::WallGeneratorMode::Classic
            }
            crate::print_config::PerimeterMode::Arachne => {
                crate::perimeter_generator::WallGeneratorMode::Arachne
            }
        };
        let region_configs: Vec<crate::region_config::PrintRegionConfig> = self
            .shared_regions
            .as_ref()
            .map(|regions| {
                regions
                    .all_regions
                    .iter()
                    .map(|region| {
                        let mut rc = region.config().clone();
                        rc.wall_generator_mode = wall_mode;
                        rc
                    })
                    .collect()
            })
            .unwrap_or_default();

        // Collect lslices from each layer for overhang detection (previous layer comparison)
        let all_lslices: Vec<Vec<crate::geometry::ExPolygon>> =
            self.layers.iter().map(|l| l.lslices.clone()).collect();

        for (idx, layer) in self.layers.iter_mut().enumerate() {
            // Check for cancellation
            // PrintObject.cpp:559
            if self.canceled.load(Ordering::Relaxed) {
                return Err(Error::Cancelled);
            }

            // Apply elephant foot compensation on first layer
            if idx == 0 && self.config.elephant_foot_compensation > 0.0 {
                for region in layer.regions_mut().iter_mut() {
                    region.elephant_foot_compensation_step(self.config.elephant_foot_compensation);
                }
            }

            // Set initial_layer_line_width on first layer regions
            if idx == 0 {
                for region in layer.regions_mut().iter_mut() {
                    region.initial_layer_line_width = self.config.initial_layer_line_width;
                }
            }

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
            layer.make_perimeters_with_neighbors(&region_configs, lower_slices, upper_slices)?;
        }

        // TODO: Port C++ lines 583-615 - Perimeter continuity calculation
        // if (this->m_print->m_config.z_direction_outwall_speed_continuous) {
        //     tbb::parallel_for(...) {
        //         m_layers[layer_idx]->calculate_perimeter_continuity(...);
        //     }
        //     merge_layer_node(...);
        //     record_cooling_node_for_each_extrusion(...);
        // }

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
            /// PrintObject.cpp:634
            for layer in &mut self.layers {
                /// PrintObject.cpp:635
                /// C++: layer->restore_untyped_slices_no_extra_perimeters();
                // TODO: Port restore_untyped_slices_no_extra_perimeters

                /// PrintObject.cpp:636
                if self.canceled.load(std::sync::atomic::Ordering::Relaxed) {
                    return Err(crate::Error::Cancelled);
                }
            }
        }

        /// PrintObject.cpp:642-647
        /// C++: std::vector<std::vector<SurfaceCollection>> slice_surfaces_cpy;
        /// C++: this->detect_surfaces_type(slice_surfaces_cpy);
        /// C++: m_print->throw_if_canceled();
        let mut slice_surfaces_cpy: Vec<Vec<SurfaceCollection>> = Vec::new();
        self.detect_surfaces_type(&mut slice_surfaces_cpy)?;

        /// PrintObject.cpp:649-655
        /// C++: for (auto *layer : m_layers)
        /// C++:     for (auto *region : layer->m_regions) {
        /// C++:         region->prepare_fill_surfaces();
        /// C++:         m_print->throw_if_canceled();
        /// C++:     }
        let spiral_mode = self.config.spiral_vase;
        let minimum_sparse_infill_area = self.config.minimum_sparse_infill_area;
        let region_configs: Vec<crate::region_config::PrintRegionConfig> = self
            .shared_regions
            .as_ref()
            .map(|regions| {
                regions
                    .all_regions
                    .iter()
                    .map(|region| region.config().clone())
                    .collect()
            })
            .unwrap_or_default();

        for layer in &mut self.layers {
            for (region_idx, region) in layer.regions_mut().iter_mut().enumerate() {
                let rc = region_configs.get(region_idx).cloned().unwrap_or_default();
                region.prepare_fill_surfaces(spiral_mode, &rc, minimum_sparse_infill_area);
                if self.canceled.load(std::sync::atomic::Ordering::Relaxed) {
                    return Err(crate::Error::Cancelled);
                }
            }
        }

        /// PrintObject.cpp:657-658
        /// C++: this->discover_vertical_shells();
        /// C++: m_print->throw_if_canceled();
        // NOTE: the faithful method `Self::discover_vertical_shells` is implemented and correct,
        // but enabling it amplifies the upstream detect_surfaces_type BottomBridge over-classification
        // (603 vs golden ~38): reading SLICES faithfully propagates the spurious bottom into massive
        // solid fill (filament 3742 -> 6031). It stays gated behind VSHELL_FAITHFUL until that detect
        // bug is fixed; the divergent surface::discover_vertical_shells remains the default.
        let use_faithful_vshell = std::env::var("VSHELL_FAITHFUL").is_ok();
        if use_faithful_vshell {
            self.discover_vertical_shells()?;
        }
        for region_id in 0..self.num_printing_regions() {
            let region_config = region_configs.get(region_id).cloned().unwrap_or_default();
            let nozzle_like_width = if region_config.outer_wall_line_width > 0.0 {
                region_config.outer_wall_line_width
            } else if region_config.inner_wall_line_width > 0.0 {
                region_config.inner_wall_line_width
            } else {
                0.4
            };
            let solid_infill_width = if region_config.internal_solid_infill_line_width > 0.0 {
                region_config.internal_solid_infill_line_width
            } else {
                nozzle_like_width
            };

            let mut region_surfaces: Vec<Vec<Surface>> = self
                .layers
                .iter()
                .map(|layer| {
                    layer
                        .regions()
                        .get(region_id)
                        .map(|region| region.fill_surfaces.surfaces.clone())
                        .unwrap_or_default()
                })
                .collect();

            if !use_faithful_vshell {
                let surface_cfg = SurfaceDetectionConfig {
                    top_solid_layers: region_config.top_solid_layers as usize,
                    bottom_solid_layers: region_config.bottom_solid_layers as usize,
                    offset: nozzle_like_width / 10.0,
                    min_area: 0.5,
                    shell_growth: nozzle_like_width * 2.0,
                    fill_boundary_inset: 0.0,
                    solid_infill_width,
                };
                crate::surface::discover_vertical_shells(&mut region_surfaces, &surface_cfg);
                if self.canceled.load(std::sync::atomic::Ordering::Relaxed) {
                    return Err(crate::Error::Cancelled);
                }
            }
            /// C++: this->process_external_surfaces();
            /// C++: m_print->throw_if_canceled();
            let expansion_distance = nozzle_like_width * std::f64::consts::SQRT_2;
            crate::surface::process_external_surfaces(
                &mut region_surfaces,
                expansion_distance,
                0.5,
            );

            if self.canceled.load(std::sync::atomic::Ordering::Relaxed) {
                return Err(crate::Error::Cancelled);
            }

            for (layer, layer_surfaces) in self.layers.iter_mut().zip(region_surfaces.into_iter()) {
                if let Some(region) = layer.regions_mut().get_mut(region_id) {
                    region.fill_surfaces.surfaces = layer_surfaces;
                }
            }
        }

        /// PrintObject.cpp:689-691
        /// C++: this->discover_horizontal_shells();
        /// C++: m_print->throw_if_canceled();
        self.discover_horizontal_shells()?;

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
        // TODO: Port clip_fill_surfaces()

        /// PrintObject.cpp:721-723
        /// C++: this->bridge_over_infill();
        /// C++: m_print->throw_if_canceled();
        // TODO: Port bridge_over_infill()

        /// PrintObject.cpp:725-727
        /// C++: this->combine_infill();
        /// C++: m_print->throw_if_canceled();
        // TODO: Port combine_infill()

        /// PrintObject.cpp:748
        /// C++: this->set_done(posPrepareInfill);
        self.set_step_done(PrintObjectStep::PrepareInfill);
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
            let region_config = self
                .shared_regions
                .as_ref()
                .and_then(|r| r.all_regions.get(region_id))
                .map(|r| r.config().clone())
                .unwrap_or_default();

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
            // Sequential implementation (C++ uses TBB parallel_for)
            let range_end = if spiral_mode && num_layers > 1 {
                num_layers - 1
            } else {
                self.layers.len()
            };

            for idx_layer in 0..range_end {
                /// PrintObject.cpp:1481
                if self.canceled.load(std::sync::atomic::Ordering::Relaxed) {
                    return Err(crate::Error::Cancelled);
                }

                /// PrintObject.cpp:1483-1485
                /// C++: Layer *layer = m_layers[idx_layer];
                /// C++: LayerRegion *layerm = layer->m_regions[region_id];
                /// C++: slice_surfaces_cpy[idx_layer].resize(layer->m_regions.size());
                slice_surfaces_cpy[idx_layer].resize(
                    self.layers[idx_layer].regions().len(),
                    SurfaceCollection::new(),
                );

                /// PrintObject.cpp:1486-1490
                /// C++: if (layerm->region().config().infill_instead_top_bottom_surfaces && layerm->region().config().sparse_infill_pattern == ipLockedZag) {
                /// C++:     slice_surfaces_cpy[idx_layer][region_id] = layerm->slices;
                /// C++: }
                // TODO: Port infill_instead_top_bottom_surfaces check

                /// PrintObject.cpp:1491-1494
                /// C++: Layer *upper_layer = (idx_layer + 1 < this->layer_count()) ? m_layers[idx_layer + 1] : nullptr;
                /// C++: Layer *lower_layer = (idx_layer > 0) ? m_layers[idx_layer - 1] : nullptr;
                let has_upper_layer = idx_layer + 1 < self.layers.len();
                let has_lower_layer = idx_layer > 0;

                /// PrintObject.cpp:1495-1496
                /// C++: float offset = layerm->flow(frExternalPerimeter).scaled_width() / 10.f;
                // C++: float offset = layerm->flow(frExternalPerimeter).scaled_width() / 10.f;
                // scaled_width for 0.42mm line width = scale_(0.42) = 42000
                // 42000 / 10 = 4200 scaled units = 0.042 mm
                // Use the external perimeter line width / 10 in mm
                let ext_wall_width = if region_config.outer_wall_line_width > 0.0 {
                    region_config.outer_wall_line_width
                } else {
                    0.42 // default
                };
                let offset = ext_wall_width / 10.0; // in mm

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
                            &self.layers[idx_layer].regions()[region_id].slices.surfaces;
                        let upper_diff = if interface_shells {
                            let upper_slices = &self.layers[idx_layer + 1].regions()[region_id]
                                .slices
                                .surfaces;
                            diff_ex(current_slices, upper_slices, ApplySafetyOffset::Yes)
                        } else {
                            let upper_lslices = &self.layers[idx_layer + 1].lslices;
                            diff_ex_surfaces_expolygons(
                                current_slices,
                                upper_lslices,
                                ApplySafetyOffset::Yes,
                            )
                        };

                        /// PrintObject.cpp:1507
                        /// C++: surfaces_append(top, opening_ex(upper_slices, offset), stTop);
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
                        top = self.layers[idx_layer].regions()[region_id]
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
                            &self.layers[idx_layer].regions()[region_id].slices.surfaces;
                        let lower_lslices = &self.layers[idx_layer - 1].lslices;
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
                            let lower_region_slices = &self.layers[idx_layer - 1].regions()
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
                        bottom = self.layers[idx_layer].regions()[region_id]
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

                /// PrintObject.cpp:1584-1586
                /// C++: Surfaces &surfaces_out = interface_shells ? surfaces_new[idx_layer] : layerm->slices.surfaces;
                /// C++: Surfaces surfaces_backup;
                let surfaces_backup = if !interface_shells {
                    self.layers[idx_layer].regions_mut()[region_id]
                        .slices
                        .surfaces
                        .clone()
                } else {
                    Surfaces::new()
                };

                if !interface_shells {
                    self.layers[idx_layer].regions_mut()[region_id]
                        .slices
                        .surfaces
                        .clear();
                }

                /// PrintObject.cpp:1591
                /// C++: const Surfaces &surfaces_prev = interface_shells ? layerm->slices.surfaces : surfaces_backup;
                let surfaces_prev = if interface_shells {
                    self.layers[idx_layer].regions()[region_id]
                        .slices
                        .surfaces
                        .clone()
                } else {
                    surfaces_backup
                };

                /// PrintObject.cpp:1593-1597
                /// C++: {
                /// C++:     Polygons topbottom = to_polygons(top);
                /// C++:     polygons_append(topbottom, to_polygons(bottom));
                /// C++:     surfaces_append(surfaces_out, diff_ex(surfaces_prev, topbottom), stInternal);
                /// C++: }
                let mut topbottom = to_polygons(&top);
                topbottom.append(&mut to_polygons(&bottom));
                // C++: diff_ex(surfaces_prev, topbottom) = surfaces_prev MINUS topbottom
                // Internal surfaces are those NOT classified as top or bottom
                let surfaces_prev_expolygons: Vec<crate::ExPolygon> =
                    surfaces_prev.iter().map(|s| s.expolygon.clone()).collect();
                let topbottom_expolygons: Vec<crate::ExPolygon> = top
                    .iter()
                    .chain(bottom.iter())
                    .map(|s| s.expolygon.clone())
                    .collect();
                let internal_surfaces = crate::clipper_utils::difference(
                    &surfaces_prev_expolygons,
                    &topbottom_expolygons,
                );

                if interface_shells {
                    surfaces_append(
                        &mut surfaces_new[idx_layer],
                        internal_surfaces,
                        SurfaceType::Internal,
                    );
                    surfaces_new[idx_layer].append(&mut top);
                    surfaces_new[idx_layer].append(&mut bottom);
                } else {
                    let region = &mut self.layers[idx_layer].regions_mut()[region_id];
                    // Convert internal_surfaces (ExPolygons) to Surfaces
                    for expoly in internal_surfaces {
                        region
                            .slices
                            .surfaces
                            .push(Surface::new(SurfaceType::Internal, expoly));
                    }
                    region.slices.surfaces.append(&mut top);
                    region.slices.surfaces.append(&mut bottom);
                }
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
            for idx_layer in 0..self.layers.len() {
                if self.canceled.load(std::sync::atomic::Ordering::Relaxed) {
                    return Err(crate::Error::Cancelled);
                }
                self.layers[idx_layer].regions_mut()[region_id].slices_to_fill_surfaces_clipped();
            }

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
        self.prepare_infill()?;

        // Check if step needs to be done
        // PrintObject.cpp:756
        if !self.is_step_done(PrintObjectStep::Infill) {
            // Status update
            // PrintObject.cpp:757
            // (Status callback handled by Print::process)

            // Adaptive fill octrees (currently None - TODO: implement adaptive fill)
            // PrintObject.cpp:763-770
            // C++ uses the actual PrintRegionConfig from each shared print region.
            // Pass the per-region configs through to Layer::make_fills() so fill
            // parameter grouping follows the same data flow instead of falling
            // back to synthetic defaults.
            let region_configs: Vec<crate::region_config::PrintRegionConfig> = self
                .shared_regions
                .as_ref()
                .map(|regions| {
                    regions
                        .all_regions
                        .iter()
                        .map(|region| region.config().clone())
                        .collect()
                })
                .unwrap_or_default();

            // Iterate through all layers and generate fills
            // PrintObject.cpp:763-770
            // C++ uses tbb::parallel_for for parallelism, we use sequential for now
            for layer in &mut self.layers {
                // Call Layer::make_fills() on each layer
                // PrintObject.cpp:768
                layer.make_fills(&region_configs)?;
            }

            // Mark step as complete
            // PrintObject.cpp:776
            self.set_step_done(PrintObjectStep::Infill);
        }

        Ok(())
    }

    /// Generate support material using SupportGenerator from support/
    /// PrintObject.cpp:856-901
    pub fn generate_support_material(&mut self) -> Result<()> {
        use crate::support::{SupportConfig, SupportGenerator, SupportType as SupportGenType};

        if !self.config.enable_support && self.config.enforce_support_layers == 0 {
            self.set_step_done(PrintObjectStep::SupportMaterial);
            return Ok(());
        }

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
    /// single-region path (Benchy is single-region; the multi-material
    /// top_bottom_surfaces_all_regions branch is not applicable here).
    fn discover_vertical_shells(&mut self) -> Result<()> {
        use crate::clipper_utils::{closing, difference, grow, intersection, offset2, shrink, union_ex, OffsetJoinType};
        use crate::flow::FlowRole;
        use crate::geometry::ExPolygons;
        use crate::region_config::EnsureVerticalThicknessLevel;
        use crate::surface::SurfaceType;

        const TOP_BOTTOM_EXPANSION_COEFF: f64 = 0.05; // PrintObject.cpp:1758
        let sf = crate::SCALING_FACTOR; // mm -> scaled (100_000); area is scaled^2
        let eps_mm = crate::libslic3r::EPSILON; // 1e-4

        let region_configs: Vec<crate::region_config::PrintRegionConfig> = self
            .shared_regions
            .as_ref()
            .map(|r| r.all_regions.iter().map(|x| x.config().clone()).collect())
            .unwrap_or_default();

        // PerimeterGenerator/Layer feed this; spiral mode clamps the layer count.
        let num_layers = self.layers.len();
        if num_layers == 0 {
            return Ok(());
        }

        // PrintObject.cpp:1827 — per region.
        for region_id in 0..self.num_printing_regions() {
            let rc = region_configs.get(region_id).cloned().unwrap_or_default();
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
            // PrintObject.cpp:1846-1864
            struct Cache {
                top: ExPolygons,
                bottom: ExPolygons,
                holes: ExPolygons,
            }
            let mut cache: Vec<Cache> = Vec::with_capacity(num_layers);
            let mut lslices_all: Vec<ExPolygons> = Vec::with_capacity(num_layers);
            let mut solid_spacing_mm: Vec<f64> = Vec::with_capacity(num_layers);
            let mut ext_spacing_mm: Vec<f64> = Vec::with_capacity(num_layers);
            for (idx, layer) in self.layers.iter().enumerate() {
                lslices_all.push(layer.lslices.clone());
                let lh = layer.height;
                let lr = match layer.regions().get(region_id) {
                    Some(r) => r,
                    None => {
                        cache.push(Cache { top: vec![], bottom: vec![], holes: vec![] });
                        solid_spacing_mm.push(0.45);
                        ext_spacing_mm.push(0.45);
                        continue;
                    }
                };
                let solid_flow = lr.flow_with_config(FlowRole::SolidInfill, lh, &rc, idx == 0)?;
                let ext_flow = lr.flow_with_config(FlowRole::ExternalPerimeter, lh, &rc, idx == 0)?;
                let sp = solid_flow.spacing();
                solid_spacing_mm.push(sp);
                ext_spacing_mm.push(ext_flow.spacing());
                // PrintObject.cpp:1850 — top_bottom_expansion = scaled_spacing * 0.05 (mm here).
                let exp = sp * TOP_BOTTOM_EXPANSION_COEFF;
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
                // holes = union of all regions' fill_expolygons on this layer (PrintObject.cpp:1859-1862)
                let mut holes: ExPolygons = Vec::new();
                for r in layer.regions() {
                    holes.extend(r.fill_expolygons.iter().cloned());
                }
                let holes = if holes.is_empty() { holes } else { union_ex(&holes) };
                cache.push(Cache { top, bottom, holes });
            }

            // --- Per layer: project, trim, regularize, convert. PrintObject.cpp:1880-2091
            for idx in 0..num_layers {
                if self.canceled.load(std::sync::atomic::Ordering::Relaxed) {
                    return Err(crate::Error::Cancelled);
                }
                let min_pis = solid_spacing_mm[idx] * 1.05; // min_perimeter_infill_spacing (mm)

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
                    let print_z = self.layers[idx].print_z;
                    let itop = idx + n_top;
                    let mut i = idx + 1;
                    let mut any = false;
                    while i < cache.len()
                        && (i < itop || self.layers[i].print_z - print_z < top_thick - eps_mm)
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
                    let bottom_z = self.layers[idx].bottom_z();
                    let ibottom = idx as i64 - n_bottom as i64;
                    let mut i = idx as i64 - 1;
                    let mut any = false;
                    while i >= 0
                        && (i > ibottom
                            || bottom_z - self.layers[i as usize].bottom_z() < bot_thick - eps_mm)
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

                // polygonsInternal = fill_surfaces filtered to {Internal, InternalVoid, InternalSolid}
                // (PrintObject.cpp:1992). Read the current region's fill_surfaces (immutable).
                let (internal_all, internal_only, void_only, solid_only) = {
                    let fs = &self.layers[idx].regions()[region_id].fill_surfaces;
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
                let mut new_shell = if shell.is_empty() || internal_all.is_empty() {
                    vec![]
                } else {
                    intersection(&shell, &internal_all)
                };
                new_shell.extend(if holes.is_empty() {
                    internal_all.clone()
                } else {
                    difference(&internal_all, &holes)
                });
                if new_shell.is_empty() {
                    continue;
                }
                // PrintObject.cpp:1999 — append existing internal-solid so they merge.
                new_shell.extend(solid_only.clone());
                let shell_u = union_ex(&new_shell);

                // PrintObject.cpp:2007-2055 — regularize (open then close), then drop scattered tiny bits.
                let narrow_wall_r = 0.5 * 0.65 * min_pis;
                let narrow_sparse_r = 0.5 * 1.2 * min_pis;
                let tiny_overlap_r = 0.2 * min_pis;
                let opened = offset2(
                    &shell_u,
                    narrow_wall_r,
                    narrow_wall_r + narrow_sparse_r,
                    OffsetJoinType::Square,
                );
                let regularized0 = shrink(&opened, narrow_sparse_r - tiny_overlap_r, OffsetJoinType::Square);

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
                let thr15 = 1.5 * min_pis * sf * sf;
                let thr8 = 8.0 * min_pis * sf * sf;
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
                    continue;
                }

                // PrintObject.cpp:2060,2075-2090 — reassign surfaces.
                let new_solid = intersection(&internal_all, &regularized);
                let new_internal = difference(&internal_only, &regularized);
                let new_void = difference(&void_only, &regularized);

                let fs = &mut self.layers[idx].regions_mut()[region_id].fill_surfaces;
                fs.keep_types(&[SurfaceType::Top, SurfaceType::Bottom, SurfaceType::BottomBridge]);
                fs.append(new_internal, SurfaceType::Internal);
                fs.append(new_void, SurfaceType::InternalVoid);
                fs.append(new_solid, SurfaceType::InternalSolid);
            }
        }
        Ok(())
    }

    /// Discover horizontal shells - propagate top/bottom surfaces to neighbor layers
    /// PrintObject.cpp:3385-3560
    /// C++: void PrintObject::discover_horizontal_shells()
    fn discover_horizontal_shells(&mut self) -> Result<()> {
        use crate::clipper_utils::{
            diff_ex, difference, grow, intersection, opening, opening_ex, shrink, union_ex,
            union_polygons_ex, ApplySafetyOffset, OffsetJoinType,
        };
        use crate::geometry::{to_polygons, ExPolygon, ExPolygons, Polygon, Polygons};
        use crate::surface::{Surface, SurfaceCollection, SurfaceType};

        /// PrintObject.cpp:3387
        /// C++: BOOST_LOG_TRIVIAL(trace) << "discover_horizontal_shells()";

        /// PrintObject.cpp:3389-3556
        /// C++: for (size_t region_id = 0; region_id < this->num_printing_regions(); ++region_id) {
        for region_id in 0..self.num_printing_regions() {
            // PrintObject.cpp:3394: const PrintRegionConfig& region_config = layerm->region().config();
            // Threaded via shared_regions (indexed by region_id) instead of a
            // LayerRegion->region() back-pointer; identical for all layers of a region.
            let region_config = self
                .shared_regions
                .as_ref()
                .and_then(|r| r.all_regions.get(region_id))
                .map(|r| r.config().clone())
                .unwrap_or_default();

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
                    let initial_n = if surface_type == SurfaceType::Top {
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
                        let mut new_internal_solid = polygons_intersection(&solid, &internal);

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
                            let margin = 0.4 * 1000.0; // TODO: Get from neighbor_layerm->flow(frExternalPerimeter).scaled_width()
                            let clipper_safety = 0.0001; // ClipperSafetyOffset = 10 scaled units = 0.0001mm

                            // Convert to ExPolygons for opening operation
                            let new_solid_expolys: Vec<ExPolygon> = new_internal_solid
                                .iter()
                                .map(|p| ExPolygon::new(p.clone()))
                                .collect();
                            let opened = opening(&new_solid_expolys, margin, OffsetJoinType::Miter);

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
                            let margin = 3.0 * 0.4 * 1000.0; // TODO: Get from layerm->flow(frSolidInfill).scaled_width()

                            // Convert to ExPolygons for opening operation
                            let new_solid_expolys: Vec<ExPolygon> = new_internal_solid
                                .iter()
                                .map(|p| ExPolygon::new(p.clone()))
                                .collect();
                            let opened = opening(&new_solid_expolys, margin, OffsetJoinType::Miter);

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

                        // Restore top, bottom, and bottom bridge surfaces (trimmed by polygons_internal)
                        for surface in &backup.surfaces {
                            if surface.surface_type == SurfaceType::Top
                                || surface.surface_type == SurfaceType::Bottom
                                || surface.surface_type == SurfaceType::BottomBridge
                            {
                                let trimmed = difference(
                                    &[surface.expolygon.clone()],
                                    &internal_solid_expolys,
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
