//! Layer and LayerRegion structures
//!
//! Direct port of BambuStudio's Layer.cpp and Layer.hpp
//!
//! # C++ Reference
//! - Layer.cpp
//! - Layer.hpp
//!
//! # Overview
//! A Layer represents a horizontal slice through a PrintObject at a specific Z height.
//! Each layer contains:
//! - Collection of LayerRegions (one per PrintRegion)
//! - Perimeter paths (outer walls)
//! - Fill/infill paths (interior)
//! - References to upper/lower layers

use crate::clipper_utils::{union_ex, OffsetJoinType};
use crate::extrusion_entity::{
    extrusion_entities_append_paths, ExtrusionEntityCollection, ExtrusionLoop, ExtrusionPath,
    ExtrusionRole,
};
use crate::fill::fill_rectilinear::{
    generate_fill_rectilinear, generate_fill_rectilinear_monotonic,
};
use crate::fill::{generate_infill, InfillConfig, InfillPath, InfillPattern};
use crate::flow::{Flow, FlowRole};
use crate::geometry::{
    BoundingBox, ExPolygons, Point, Polygon, Polygons, Polyline, Polylines,
};
use crate::perimeter_generator::WallGeneratorMode;
use crate::region_config::PrintRegionConfig;
use crate::surface::{Surface, SurfaceType};
use crate::surface_collection::SurfaceCollection;
use crate::{scale, Result};

use std::sync::Arc;

/// Represents a region within a layer, tied to a specific print configuration.
/// Layer.hpp:35
#[derive(Debug, Clone)]
pub struct LayerRegion {
    /// Layer.hpp:36
    layer_id: usize,
    /// Layer.hpp:37
    region_id: usize,

    // Surfaces
    /// Layer.hpp:40
    pub slices: SurfaceCollection,

    /// Layer.hpp:44
    pub raw_slices: SurfaceCollection,

    /// Layer.hpp:47-48
    pub raw_counter_circle_compensation: Polygons,
    pub raw_holes_circle_compensation: Polygons,

    // Thin fills
    /// Layer.hpp:53
    pub thin_fills: ExtrusionEntityCollection,

    // Fill regions
    /// Layer.hpp:57
    pub fill_expolygons: ExPolygons,
    /// Layer.hpp:60
    pub fill_surfaces: SurfaceCollection,
    /// Layer.hpp:63
    pub fill_no_overlap_expolygons: ExPolygons,

    /// Layer.hpp:66
    pub unsupported_bridge_edges: Polylines,

    // Extrusions
    /// Layer.hpp:70
    pub perimeters: ExtrusionEntityCollection,

    /// Layer.hpp:74
    pub fills: ExtrusionEntityCollection,

    /// Lower layer slices for overhang detection (set before make_perimeters)
    pub lower_slices: Option<Vec<crate::geometry::ExPolygon>>,

    /// Upper layer slices for top-surface detection (set before make_perimeters)
    pub upper_slices: Option<Vec<crate::geometry::ExPolygon>>,

    /// Owning PrintRegion, shared via Arc with Print::print_regions and
    /// PrintObjectRegions::all_regions (the very same Arc identity).
    /// C++: `const PrintRegion *m_region;` (Layer.hpp:131), reached through
    /// `LayerRegion::region()` (Layer.hpp:38). Stamped by
    /// PrintObject::wire_layer_hierarchy at sync points; None until wired.
    pub(crate) region: Option<Arc<crate::print_region::PrintRegion>>,

    /// Snapshot of the owning PrintObject's config, shared via Arc.
    /// C++ reaches it via the parent pointers `m_layer->m_object->config()`
    /// (Layer.hpp:130 + Print.hpp:369). Stamped at sync points; None until wired.
    pub(crate) object_config: Option<Arc<crate::print_config::PrintObjectConfig>>,

    /// Snapshot of the owning Print's config, shared via Arc.
    /// C++: `m_layer->m_object->print()->config()` (PrintBase.hpp:632 +
    /// Print.hpp:885). Stamped at sync points; None until wired.
    pub(crate) print_config: Option<Arc<crate::print_config::PrintConfig>>,
}

impl LayerRegion {
    /// Create a new LayerRegion
    /// Layer.cpp:15
    pub fn new(layer_id: usize, region_id: usize) -> Self {
        Self {
            layer_id,
            region_id,
            slices: SurfaceCollection::new(),
            raw_slices: SurfaceCollection::new(),
            raw_counter_circle_compensation: Polygons::new(),
            raw_holes_circle_compensation: Polygons::new(),
            thin_fills: ExtrusionEntityCollection::new(),
            fill_expolygons: ExPolygons::new(),
            fill_surfaces: SurfaceCollection::new(),
            fill_no_overlap_expolygons: ExPolygons::new(),
            unsupported_bridge_edges: Polylines::new(),
            perimeters: ExtrusionEntityCollection::new(),
            fills: ExtrusionEntityCollection::new(),
            lower_slices: None,
            upper_slices: None,
            region: None,
            object_config: None,
            print_config: None,
        }
    }

    /// Get layer ID
    /// Layer.hpp:78
    pub fn layer_id(&self) -> usize {
        self.layer_id
    }

    /// Get region ID
    /// Layer.hpp:79
    pub fn region_id(&self) -> usize {
        self.region_id
    }

    /// Get the owning PrintRegion.
    /// C++: `const PrintRegion& region() const { return *m_region; }` (Layer.hpp:38)
    ///
    /// Fails fast when the config hierarchy has not been stamped yet —
    /// support layers carry no regions, so only true printing regions reach
    /// this accessor.
    pub fn region(&self) -> &crate::print_region::PrintRegion {
        self.region
            .as_deref()
            .expect("config hierarchy not wired — call wire_layer_hierarchy")
    }

    /// Get slices
    /// Layer.hpp:81
    pub fn get_slices(&self) -> &SurfaceCollection {
        &self.slices
    }

    /// Calculate flow for a given role.
    /// LayerRegion.cpp:21-29
    /// C++: Flow LayerRegion::flow(FlowRole role) const
    ///          { return this->flow(role, m_layer->height); }
    /// C++: Flow LayerRegion::flow(FlowRole role, double layer_height) const
    ///
    /// This crate's LayerRegion has no Layer back-pointer, so `layer_height`
    /// (C++ `m_layer->height`) is threaded by the caller and the two C++
    /// overloads collapse into this single method.
    pub fn flow(&self, role: FlowRole, layer_height: f64) -> Result<Flow> {
        // LayerRegion.cpp:28
        // C++: return m_region->flow(*m_layer->object(), role, layer_height, m_layer->id() == 0);
        // The object/print configs are reached through the Arc snapshots
        // stamped by wire_layer_hierarchy instead of the C++ parent pointers;
        // `m_layer->id()` is mirrored by this->layer_id (stamped at creation).
        let print_config = self
            .print_config
            .as_deref()
            .expect("config hierarchy not wired — call wire_layer_hierarchy");
        let object_config = self
            .object_config
            .as_deref()
            .expect("config hierarchy not wired — call wire_layer_hierarchy");
        crate::print_region::flow_from_configs(
            role,
            layer_height,
            self.layer_id == 0,
            print_config.initial_layer_line_width,
            object_config.line_width,
            print_config.nozzle_diameter,
            self.region().config(),
        )
        .map_err(crate::Error::Config)
    }

    /// Calculate bridging flow.
    /// LayerRegion.cpp:31-46
    /// C++: Flow LayerRegion::bridging_flow(FlowRole role, bool thick_bridge) const
    ///
    /// The non-thick branch calls C++ `this->flow(role)` which reads
    /// `m_layer->height`; `layer_height` is threaded by the caller for the
    /// same reason as in [`LayerRegion::flow`].
    pub fn bridging_flow(&self, role: FlowRole, thick_bridge: bool, layer_height: f64) -> Result<Flow> {
        // LayerRegion.cpp:33
        // C++: const PrintRegion &region = this->region();
        let region = self.region();
        // LayerRegion.cpp:34
        // C++: const PrintRegionConfig &region_config = region.config();
        let region_config = region.config();
        // LayerRegion.cpp:35
        // C++: const PrintObject &print_object = *this->layer()->object();
        // (only used to reach print()->config(); read off the stored Arc below)
        if thick_bridge {
            // The old Slic3r way (different from all other slicers): Use rounded extrusions.
            // Get the configured nozzle_diameter for the extruder associated to the flow role requested.
            // Here this->extruder(role) - 1 may underflow to MAX_INT, but then the get_at() will follback to zero'th element, so everything is all right.
            // LayerRegion.cpp:40
            // C++: auto nozzle_diameter = float(print_object.print()->config().nozzle_diameter.get_at(region.extruder(role) - 1));
            // This crate's PrintConfig nozzle_diameter is a single-extruder
            // scalar, so get_at(extruder - 1) collapses to a direct read;
            // region.extruder(role) is still evaluated for its Unknown-role
            // error semantics.
            let _ = region.extruder(role).map_err(crate::Error::Config)?;
            let nozzle_diameter = self
                .print_config
                .as_deref()
                .expect("config hierarchy not wired — call wire_layer_hierarchy")
                .nozzle_diameter;
            // Applies default bridge spacing.
            // LayerRegion.cpp:42
            // C++: return Flow::bridging_flow(float(sqrt(region_config.bridge_flow)) * nozzle_diameter, nozzle_diameter);
            Ok(Flow::bridging_flow(
                region_config.bridge_flow_ratio.sqrt() * nozzle_diameter,
                nozzle_diameter,
            ))
        } else {
            // The same way as other slicers: Use normal extrusions. Apply bridge_flow while maintaining the original spacing.
            // LayerRegion.cpp:45
            // C++: return this->flow(role).with_flow_ratio(region_config.bridge_flow);
            self.flow(role, layer_height)?
                .with_flow_ratio(region_config.bridge_flow_ratio)
                .map_err(|e| e.into())
        }
    }

    // Placeholder methods

    /// Clip slices to fill surfaces
    /// LayerRegion.cpp:50-66
    /// C++: void LayerRegion::slices_to_fill_surfaces_clipped()
    pub fn slices_to_fill_surfaces_clipped(&mut self) {
        use crate::clipper_utils::intersection_ex;
        use crate::surface::SurfaceType;

        /// LayerRegion.cpp:52-56
        /// C++: std::array<SurfacesPtr, size_t(stCount)> by_surface;
        /// C++: for (Surface &surface : this->slices.surfaces)
        /// C++:     by_surface[size_t(surface.surface_type)].emplace_back(&surface);
        const ST_COUNT: usize = crate::surface::SurfaceType::COUNT; // stCount from C++ (Surface.hpp:29)
        let mut by_surface: Vec<Vec<usize>> = vec![Vec::new(); ST_COUNT];
        for (idx, surface) in self.slices.surfaces.iter().enumerate() {
            let surface_type_idx = surface.surface_type as usize;
            by_surface[surface_type_idx].push(idx);
        }

        /// LayerRegion.cpp:57-58
        /// C++: this->fill_surfaces.surfaces.clear();
        self.fill_surfaces.surfaces.clear();

        /// LayerRegion.cpp:59-65
        /// C++: for (size_t surface_type = 0; surface_type < size_t(stCount); ++ surface_type) {
        /// C++:     const SurfacesPtr &this_surfaces = by_surface[surface_type];
        /// C++:     if (! this_surfaces.empty())
        /// C++:         this->fill_surfaces.append(intersection_ex(this_surfaces, this->fill_expolygons), SurfaceType(surface_type));
        /// C++: }
        for surface_type_idx in 0..ST_COUNT {
            let this_surfaces_indices = &by_surface[surface_type_idx];
            if !this_surfaces_indices.is_empty() {
                let this_surfaces: Vec<crate::surface::Surface> = this_surfaces_indices
                    .iter()
                    .map(|&idx| self.slices.surfaces[idx].clone())
                    .collect();
                let intersected = intersection_ex(&this_surfaces, &self.fill_expolygons);
                self.fill_surfaces
                    .append(intersected, SurfaceType::from_u8(surface_type_idx as u8));
            }
        }

    }

    /// Prepare fill surfaces
    /// LayerRegion.cpp:645-689
    /// C++: void LayerRegion::prepare_fill_surfaces()
    pub fn prepare_fill_surfaces(&mut self) {
        /// LayerRegion.cpp:650-652
        /// C++: Note: in order to make the psPrepareInfill step idempotent, we should never
        /// C++: alter fill_surfaces boundaries on which our idempotency relies since that's
        /// C++: the only meaningful information returned by psPerimeters.

        /// First clip slices to fill boundaries
        /// LayerRegion.cpp:50-66 (called before this function in the flow)
        self.slices_to_fill_surfaces_clipped();

        /// LayerRegion.cpp:657
        /// C++: bool spiral_mode = this->layer()->object()->print()->config().spiral_mode;
        // Reached through the LayerRegion's own print-config Arc (stamped by
        // wire_layer_hierarchy); field mapping: spiral_mode -> spiral_vase.
        let spiral_mode = self
            .print_config
            .as_deref()
            .expect("config hierarchy not wired — call wire_layer_hierarchy")
            .spiral_vase;

        /// LayerRegion.cpp:658-672 (commented out with #if 0 in C++)
        /// The top/bottom demotion logic is disabled in BambuStudio

        /// LayerRegion.cpp:675-682
        /// C++: turn too small internal regions into solid regions according to the user setting
        /// C++: if (! spiral_mode && this->region().config().sparse_infill_density.value > 0) {
        /// C++:     double min_area = scale_(scale_(this->region().config().minimum_sparse_infill_area.value));
        /// C++:     for (Surface &surface : this->fill_surfaces.surfaces)
        /// C++:         if (surface.surface_type == stInternal && surface.area() <= min_area)
        /// C++:             surface.surface_type = stInternalSolid;
        /// C++: }
        // Field mapping: sparse_infill_density -> fill_density on the wired
        // region Arc. NOTE: this crate keeps minimum_sparse_infill_area on
        // PrintObjectConfig (print_config.rs) rather than PrintRegionConfig,
        // so it is read through the object-config Arc — same resolved value.
        if !spiral_mode && self.region().config().fill_density > 0.0 {
            let minimum_sparse_infill_area = self
                .object_config
                .as_deref()
                .expect("config hierarchy not wired — call wire_layer_hierarchy")
                .minimum_sparse_infill_area;
            let scale_factor = crate::SCALING_FACTOR as f64;
            let min_area = (minimum_sparse_infill_area * scale_factor * scale_factor) as i64;

            for surface in &mut self.fill_surfaces.surfaces {
                if surface.surface_type == crate::surface::SurfaceType::Internal
                    && surface.area() <= min_area as f64
                {
                    surface.surface_type = crate::surface::SurfaceType::InternalSolid;
                }
            }
        }
    }

    /// Generate perimeters for this region
    /// LayerRegion.cpp:131-172
    /// C++: void LayerRegion::make_perimeters(const SurfaceCollection &slices, const PerimeterRegions &perimeter_regions, SurfaceCollection *fill_surfaces, ExPolygons *fill_no_overlap, std::vector<LoopNode> &loop_nodes)
    pub fn make_perimeters(
        &mut self,
        surface_fill: &SurfaceCollection,
        layer_height: f64,
        layer_id: usize,
        print_z: f64,
    ) -> Result<()> {
        use crate::perimeter_generator::{PerimeterConfig, PerimeterGenerator};

        // LayerRegion.cpp:133-134
        // C++: this->perimeters.clear();
        // C++: this->thin_fills.clear();
        self.perimeters.entities.clear();
        self.thin_fills.entities.clear();

        // LayerRegion.cpp:136-138
        // C++: const PrintConfig &print_config = this->layer()->object()->print()->config();
        // C++: const PrintRegionConfig &region_config = this->region().config();
        // C++: const PrintObjectConfig& object_config = this->layer()->object()->config();
        // All three are reached through the Arc snapshots stamped by
        // wire_layer_hierarchy; the Arcs are cloned into locals up front so no
        // borrow of self is held while building PerimeterConfig / mutating
        // self below. layer_id/print_z replace the C++ Layer back-pointer
        // reads this->layer()->id() / this->layer()->print_z.
        let print_config = self
            .print_config
            .clone()
            .expect("config hierarchy not wired — call wire_layer_hierarchy");
        let region = self
            .region
            .clone()
            .expect("config hierarchy not wired — call wire_layer_hierarchy");
        let config = region.config();
        let object_config = self
            .object_config
            .clone()
            .expect("config hierarchy not wired — call wire_layer_hierarchy");

        // LayerRegion.cpp:139-148
        // C++: // This needs to be in sync with PrintObject::_slice() slicing_mode_normal_below_layer!
        // C++: bool spiral_mode = print_config.spiral_mode &&
        // C++:     //FIXME account for raft layers.
        // C++:     (this->layer()->id() >= size_t(region_config.bottom_shell_layers.value) &&
        // C++:      this->layer()->print_z >= region_config.bottom_shell_thickness - EPSILON);
        // Rust field mapping: spiral_mode -> PrintConfig::spiral_vase,
        // bottom_shell_layers -> PrintRegionConfig::bottom_solid_layers,
        // bottom_shell_thickness -> PrintRegionConfig::bottom_solid_min_thickness.
        let spiral_mode = print_config.spiral_vase
            && (layer_id >= config.bottom_solid_layers as usize
                && print_z >= config.bottom_solid_min_thickness - crate::libslic3r::EPSILON);
        // C++ feeds spiral_mode into the PerimeterGenerator ctor and the
        // arachne/classic dispatch (LayerRegion.cpp:150-179). The Rust
        // PerimeterConfig does not carry a spiral field yet, so the gate is
        // computed faithfully here but not yet consumed downstream.
        let _ = spiral_mode;

        // LayerRegion.cpp:150-172
        // C++: PerimeterGenerator g(
        // C++:     &slices,
        // C++:     this->layer()->height,
        // C++:     this->flow(frPerimeter),
        // C++:     &region_config,
        // C++:     &this->layer()->object()->config(),
        // C++:     &print_config,
        // C++:     spiral_mode,
        // C++:     &this->perimeters,
        // C++:     &this->thin_fills,
        // C++:     fill_surfaces,
        // C++:     fill_no_overlap,
        // C++:     &loop_nodes
        // C++: );

        // Get flows for different perimeter types
        // C++: this->flow(frPerimeter) (LayerRegion.cpp:153) — the LayerRegion
        // reads region/object/print configs off its stored Arcs; layer_height
        // stands in for the missing m_layer->height back-pointer read.
        let perimeter_flow = self.flow(FlowRole::Perimeter, layer_height)?;
        let external_perimeter_flow = self.flow(FlowRole::ExternalPerimeter, layer_height)?;

        // Build PerimeterConfig from PrintRegionConfig
        let perimeter_config = PerimeterConfig {
            perimeter_count: config.perimeters as usize,
            perimeter_extrusion_width: perimeter_flow.width(),
            external_perimeter_extrusion_width: external_perimeter_flow.width(),
            smaller_external_perimeter_width: external_perimeter_flow.width() * 0.9, // TODO: Get from config
            perimeter_spacing: perimeter_flow.spacing(),
            external_perimeter_spacing: external_perimeter_flow.spacing(),
            smaller_external_perimeter_spacing: external_perimeter_flow.spacing() * 0.9,
            external_to_internal_spacing: (external_perimeter_flow.spacing()
                + perimeter_flow.spacing())
                / 2.0,
            layer_height,
            perimeter_flow: perimeter_flow.clone(),
            ext_perimeter_flow: external_perimeter_flow.clone(),
            smaller_ext_perimeter_flow: external_perimeter_flow.clone(), // TODO: Create proper smaller flow
            join_type: crate::clipper_utils::OffsetJoinType::Miter,
            gap_fill_threshold: if config.gap_fill_enabled {
                config.gap_fill_speed.max(1.0)
            } else {
                0.0
            },
            // PerimeterGenerator.cpp:1185 — this->config->sparse_infill_density.value.
            // PrintRegionConfig stores the percent option as the fraction fill_density;
            // the consumer only performs the == 0 comparison, which is equivalent.
            sparse_infill_density: config.fill_density,
            detect_thin_wall: config.thin_walls,
            // PerimeterGenerator.cpp:911 m_scaled_resolution = scaled<double>(print_config.resolution).
            // This is fed to ExPolygon::simplify_p(), which calls
            // `geometry::simplify::douglas_peucker` — that variant RE-SCALES the
            // tolerance internally (`tolerance_sq = scale(tolerance)^2`,
            // geometry/simplify.rs), so it expects the UNSCALED value in **mm**.
            // Read from the print config (C++: print_config->resolution).
            // DO NOT pre-scale here: passing scaled units would be squared again and
            // collapse every contour to a point (empty perimeters). NOTE the contrast
            // with `multi_point::douglas_peucker`, which does NOT scale and is the one
            // used by the arc fitter / Polyline::simplify — different unit convention.
            surface_simplify_resolution: print_config.resolution,
            // PerimeterGenerator.cpp:914 uses print_config->enable_arc_fitting to pick the
            // surface simplify resolution factor (0.2x when arc fitting + no fuzzy skin).
            // Mirrors PrintConfig default enable_arc_fitting = true / resolved config "1".
            arc_fitting_enabled: true,
            // LayerRegion.cpp:176
            // C++: if (this->layer()->object()->config().wall_generator.value == PerimeterGeneratorType::Arachne && !spiral_mode)
            // C++:     g.process_arachne();
            // C++: else
            // C++:     g.process_classic();
            // The Classic/Arachne dispatch lives on the OBJECT config in C++;
            // mapped here onto PerimeterConfig::wall_generator_mode, which the
            // generator dispatches on internally. (The !spiral_mode gate is
            // not consumed yet — see the spiral_mode note above.)
            wall_generator_mode: match object_config.perimeter_mode {
                crate::print_config::PerimeterMode::Classic => WallGeneratorMode::Classic,
                crate::print_config::PerimeterMode::Arachne => WallGeneratorMode::Arachne,
            },
            fuzzy_skin_mode: config.fuzzy_skin_mode,
            fuzzy_skin_thickness: config.fuzzy_skin_thickness,
            fuzzy_skin_point_distance: config.fuzzy_skin_point_distance,
            wall_sequence: config.wall_sequence,
            detect_overhang_wall: config.overhangs,
            lower_slices: self.lower_slices.clone(),
            upper_slices: self.upper_slices.clone(),
            // BambuStudio default top_one_wall_type == Alltop (config key only_one_wall_top);
            // gates the top_fills top-surface detection (PerimeterGenerator.cpp:1118).
            top_one_wall: true,
            // PrintConfig.cpp:1288 default top_area_threshold = 200%.
            top_area_threshold: 200.0,
            // PerimeterGenerator.cpp:874 — solid_infill_flow.scaled_spacing()
            // (LayerRegion.cpp:173: g.solid_infill_flow = this->flow(frSolidInfill)).
            solid_infill_spacing: self
                .flow(FlowRole::SolidInfill, layer_height)
                .map(|f| f.spacing())
                .unwrap_or_else(|_| perimeter_flow.spacing()),
            // PerimeterGenerator.cpp:1167 — config->sparse_infill_line_width.value.
            sparse_infill_line_width: config.sparse_infill_line_width,
            // PerimeterGenerator.cpp:1392 — config->infill_wall_overlap (percent -> fraction).
            infill_wall_overlap: object_config.infill_wall_overlap,
            layer_id: layer_id,
            raft_layers: 0,
            // LayerRegion.cpp:172 — g.overhang_flow = this->bridging_flow(frPerimeter, thick_bridges);
            // Used by detect_bridge_wall for the 100%-overhang (erOverhangPerimeter) walls.
            overhang_flow: self
                .bridging_flow(FlowRole::Perimeter, object_config.thick_bridges, layer_height)
                .ok(),
            // LayerRegion.cpp:173 — g.solid_infill_flow = this->flow(frSolidInfill).
            // Used by the in-generator gap-fill variable_width (PerimeterGenerator.cpp:1364)
            // so the gap-fill covered_ex subtraction happens BEFORE the infill opening.
            solid_infill_flow: self.flow(FlowRole::SolidInfill, layer_height).ok(),
        };

        // Create PerimeterGenerator
        let generator = PerimeterGenerator::new(perimeter_config);

        // Extract ExPolygons from surface_fill for processing
        let slices: Vec<crate::ExPolygon> = surface_fill
            .surfaces
            .iter()
            .map(|s| s.expolygon.clone())
            .collect();

        // LayerRegion.cpp:174
        // C++: g.process_classic();
        // Generate perimeters
        let result = generator.generate(&slices);

        // LayerRegion.cpp:171
        // C++: The PerimeterGenerator writes directly to this->perimeters via pointer
        // In Rust, we copy from the result

        // Store perimeter entities (ExtrusionLoops)
        // PerimeterGenerator.cpp:1242-1420 - traverse_loops creates these
        self.perimeters = result.entities;

        // Store infill area
        for infill_expoly in result.infill_area {
            let fill_surface = Surface {
                expolygon: infill_expoly,
                surface_type: SurfaceType::Internal,
                thickness: layer_height,
                thickness_layers: 1,
                bridge_angle: None,
                extra_perimeters: 0,
            };
            self.fill_surfaces.surfaces.push(fill_surface.clone());
            self.fill_expolygons.push(fill_surface.expolygon);
        }

        // Store the no-overlap infill area. PerimeterGenerator.cpp:1429
        // (this->fill_no_overlap->insert(... polyWithoutOverlap ...)). Consumed by
        // group_fills -> SurfaceFill::no_overlap_expolygons and the
        // FillMonotonicLineWGapFill top-surface gap-fill in make_fills.
        self.fill_no_overlap_expolygons
            .extend(result.no_overlap_area);

        // Gap fill — PerimeterGenerator.cpp:1326-1375 now runs INSIDE the
        // perimeter generator (process_classic), so the
        //   last = diff_ex(last, gap_fill.polygons_covered_by_width(10))
        // subtraction (C++ line 1373) happens BEFORE the infill-boundary opening
        // (C++ line 1403). result.infill_area / fill_expolygons therefore already
        // exclude the gap-fill footprint with the C++ ordering. Here we only
        // append the variable-width gap-fill extrusion paths to thin_fills, as
        // C++ does via this->gap_fill->append (PerimeterGenerator.cpp:1374).
        for path in result.gap_fill_paths {
            self.thin_fills
                .entities
                .push(crate::extrusion_entity::ExtrusionEntityType::Path(path));
        }

        Ok(())
    }

    /// Process external surfaces
    /// LayerRegion.cpp:518-640
    /// C++: void LayerRegion::process_external_surfaces(const Layer *lower_layer, const Polygons *lower_layer_covered)
    ///
    /// Faithful driver: builds the flow-derived expansion config exactly as the
    /// C++ member does (shell_width / expansion_min from external-perimeter and
    /// perimeter flows, closing_radius from the solid-infill flow), then runs the
    /// wave-expansion port [`crate::region_expansion::process_external_surfaces_wave`]
    /// over this region's `fill_surfaces` in place.
    ///
    /// The C++ `lower_layer` / `lower_layer_covered` parameters feed the void-trim
    /// path in `PrintObject::process_external_surfaces`; the wave port does not yet
    /// consume them, so they are omitted here. `layer_height` (C++
    /// `m_layer->height`) is threaded by the caller, matching [`LayerRegion::flow`].
    pub fn process_external_surfaces(&mut self, layer_height: f64) -> Result<()> {
        use crate::region_expansion::{process_external_surfaces_wave, ExternalSurfaceConfig};

        // LayerRegion.cpp:526-539 — width of the perimeters.
        // C++: int num_perimeters = this->region().config().wall_loops;
        let num_perimeters = self.region().config().perimeters as usize;

        // LayerRegion.cpp:529-534
        // C++: Flow external_perimeter_flow = this->flow(frExternalPerimeter);
        // C++: Flow perimeter_flow          = this->flow(frPerimeter);
        // ExternalSurfaceConfig::from_flows reproduces the shell_width /
        // expansion_min computation (LayerRegion.cpp:532-534) and the
        // num_perimeters == 0 SCALED_EPSILON fallback (LayerRegion.cpp:536-538).
        // Note: from_flows works in mm (unscaled) widths/spacings; the wave port
        // scales internally, so feed the unscaled flow values.
        let (external_perimeter_width, external_perimeter_spacing) = if num_perimeters > 0 {
            let f = self.flow(FlowRole::ExternalPerimeter, layer_height)?;
            (f.width(), f.spacing())
        } else {
            (0.0, 0.0)
        };
        let perimeter_spacing = if num_perimeters > 0 {
            self.flow(FlowRole::Perimeter, layer_height)?.spacing()
        } else {
            0.0
        };

        // LayerRegion.cpp:550
        // C++: const float closing_radius = ... this->flow(frSolidInfill).scaled_spacing();
        let solid_infill_spacing = self.flow(FlowRole::SolidInfill, layer_height)?.spacing();

        let mut config = ExternalSurfaceConfig::from_flows(
            external_perimeter_width,
            external_perimeter_spacing,
            perimeter_spacing,
            solid_infill_spacing,
            num_perimeters,
        );

        // LayerRegion.cpp:569 — custom bridge angle (degrees).
        // C++: const double custom_angle = this->region().config().bridge_angle.value;
        config.custom_bridge_angle = self.region().config().bridge_angle;

        // LayerRegion.cpp:599-602 — minimum sparse infill area logic guards.
        // C++: if (!...->config().spiral_mode && this->region().config().sparse_infill_density.value > 0)
        // C++:     double min_area = scale_(scale_(...minimum_sparse_infill_area.value));
        config.spiral_mode = self
            .print_config
            .as_deref()
            .expect("config hierarchy not wired — call wire_layer_hierarchy")
            .spiral_vase;
        config.sparse_infill_density = self.region().config().fill_density;
        // minimum_sparse_infill_area lives on PrintObjectConfig in this crate.
        config.minimum_sparse_infill_area = self
            .object_config
            .as_deref()
            .expect("config hierarchy not wired — call wire_layer_hierarchy")
            .minimum_sparse_infill_area;

        // LayerRegion.cpp:518 — operate on this region's fill_surfaces in place.
        let mut surfaces = vec![std::mem::take(&mut self.fill_surfaces.surfaces)];
        process_external_surfaces_wave(&mut surfaces, &config);
        self.fill_surfaces.surfaces = surfaces.into_iter().next().unwrap_or_default();
        Ok(())
    }

    /// Calculate infill area threshold
    /// LayerRegion.cpp:690-694
    /// C++: double LayerRegion::infill_area_threshold() const
    /// C++: {
    /// C++:     double ss = this->flow(frSolidInfill).scaled_spacing();
    /// C++:     return ss*ss;
    /// C++: }
    ///
    /// C++ reads `this->flow(frSolidInfill)`; this crate has no `Print` back-pointer
    /// on `LayerRegion`, so the solid-infill flow is supplied by the caller.
    pub fn infill_area_threshold_with_flow(&self, solid_infill_flow: &Flow) -> f64 {
        // LayerRegion.cpp:692
        let ss = solid_infill_flow.scaled_spacing() as f64;
        // LayerRegion.cpp:693
        ss * ss
    }

    /// Calculate infill area threshold (legacy 1mm² fallback retained for callers
    /// that have no flow handy; the faithful version is
    /// `infill_area_threshold_with_flow`).
    /// LayerRegion.cpp:690-694
    pub fn infill_area_threshold(&self) -> f64 {
        scale(1.0) as f64 // 1mm² threshold
    }

    /// Trim surfaces by trimming polygons. Used by the elephant foot compensation at the 1st layer.
    /// LayerRegion.cpp:696-703
    /// C++: void LayerRegion::trim_surfaces(const Polygons &trimming_polygons)
    pub fn trim_surfaces(&mut self, trimming_polygons: &Polygons) {
        // LayerRegion.cpp:698-701
        // #ifndef NDEBUG
        //     for (const Surface &surface : this->slices.surfaces)
        //         assert(surface.surface_type == stInternal);
        // #endif
        debug_assert!(self
            .slices
            .surfaces
            .iter()
            .all(|s| s.surface_type == SurfaceType::Internal));

        // LayerRegion.cpp:702
        // this->slices.set(intersection_ex(this->slices.surfaces, trimming_polygons), stInternal);
        let clip: ExPolygons = trimming_polygons
            .iter()
            .map(|p| crate::geometry::ExPolygon::from(p.clone()))
            .collect();
        let trimmed = crate::clipper_utils::intersection_ex(&self.slices.surfaces, &clip);
        self.slices.set_expolygons(trimmed, SurfaceType::Internal);
    }

    /// Apply elephant foot compensation
    /// LayerRegion.cpp:537-570
    /// Apply elephant foot compensation to the first layer slices.
    /// LayerRegion.cpp:704-713
    /// Shrinks perimeters inward by compensation_xy to reduce elephant foot effect.
    pub fn elephant_foot_compensation_step(&mut self, compensation_xy: f64) {
        if compensation_xy <= 0.0 || self.slices.surfaces.is_empty() {
            return;
        }

        // C++ reference: PrintObjectSlice.cpp:1228-1244
        // Slic3r::elephant_foot_compensation(lslices, ext_perimeter_flow, compensation)
        // For simple shapes this is equivalent to shrinking the polygon by compensation_xy mm.
        let slices_polys: ExPolygons = self
            .slices
            .surfaces
            .iter()
            .map(|s| s.expolygon.clone())
            .collect();

        // Shrink polygon by compensation amount (inward offset)
        let result = crate::clipper_utils::shrink(
            &slices_polys,
            compensation_xy,
            crate::clipper_utils::OffsetJoinType::Miter,
        );

        // Replace slices with compensated geometry
        let surface_type = self
            .slices
            .surfaces
            .first()
            .map(|s| s.surface_type)
            .unwrap_or(crate::surface::SurfaceType::Internal);
        let thickness = self
            .slices
            .surfaces
            .first()
            .map(|s| s.thickness)
            .unwrap_or(0.2);

        self.slices.surfaces.clear();
        for ep in result {
            self.slices.surfaces.push(crate::surface::Surface {
                expolygon: ep,
                surface_type,
                thickness,
                thickness_layers: 1,
                bridge_angle: None,
                extra_perimeters: 0,
            });
        }
    }

    /// Check if region has any extrusions
    /// Layer.hpp:140
    pub fn has_extrusions(&self) -> bool {
        !self.perimeters.entities.is_empty() || !self.fills.entities.is_empty()
    }
}

// NOTE: the canonical PrintRegion lives in crate::print_region (shared via
// Arc into LayerRegion::region); a dead local shadow struct was removed here.

/// Convert a perimeter loop to an extrusion loop
/// Layer.cpp:200-240
fn convert_polygon_to_extrusion_loop(
    polygon: &Polygon,
    flow: &Flow,
    role: ExtrusionRole,
) -> ExtrusionLoop {
    let mut paths = Vec::new();

    // Create a single path from the polygon
    // Layer.cpp:242-260
    let mut path = ExtrusionPath::new(role);
    path.polyline.points = polygon.points.clone();
    path.mm3_per_mm = flow.mm3_per_mm().unwrap_or(0.0);
    path.width = flow.width();
    path.height = flow.height();

    paths.push(path);

    ExtrusionLoop::new(paths, crate::extrusion_entity::ExtrusionLoopRole::DEFAULT)
}

/// Convert a polyline to an extrusion path
/// Layer.cpp:242-260
fn convert_polyline_to_extrusion_path(
    polyline: &Polyline,
    flow: &Flow,
    role: ExtrusionRole,
) -> ExtrusionPath {
    // Layer.cpp:242-260
    let mut path = ExtrusionPath::new(role);
    path.polyline = polyline.clone();
    path.mm3_per_mm = flow.mm3_per_mm().unwrap_or(0.0);
    path.width = flow.width();
    path.height = flow.height();
    path
}

/// Extract and union surfaces of given types
/// Layer.cpp:262-285
fn extract_and_union_surfaces(surfaces: &[Surface], types: &[SurfaceType]) -> ExPolygons {
    let mut result = ExPolygons::new();

    for surface in surfaces {
        if types.contains(&surface.surface_type) {
            result.push(surface.expolygon.clone());
        }
    }

    // Union overlapping expolygons
    union_ex(&result)
}

/// Node for loop hierarchy
/// Layer.cpp:287-310
#[derive(Debug, Clone)]
pub struct LoopNode {
    pub node_id: usize,
    pub loop_id: usize,
    pub bbox: BoundingBox,
    pub merged_id: Option<usize>,
    pub upper_node_ids: Vec<usize>,
    pub lower_node_ids: Vec<usize>,
}

impl LoopNode {
    pub fn new(node_id: usize, loop_id: usize, bbox: BoundingBox) -> Self {
        Self {
            node_id,
            loop_id,
            bbox,
            merged_id: None,
            upper_node_ids: Vec::new(),
            lower_node_ids: Vec::new(),
        }
    }
}

impl Default for LoopNode {
    fn default() -> Self {
        Self::new(0, 0, BoundingBox::empty())
    }
}

/// Represents a layer in the print
/// Layer.hpp:150
#[derive(Debug, Clone)]
pub struct Layer {
    /// Layer.hpp:152
    id: usize,
    /// Object ID this layer belongs to
    /// Layer.hpp:153
    object_id: usize,

    // Layer connectivity
    /// Layer.hpp:156-157
    pub upper_layer_id: Option<usize>,
    pub lower_layer_id: Option<usize>,

    /// Layer.hpp:160
    pub slicing_errors: bool,

    // Z coordinates
    /// Layer.hpp:163
    pub slice_z: f64,
    /// Layer.hpp:166
    pub print_z: f64,
    /// Layer.hpp:169
    pub height: f64,

    /// Layer.hpp:172-174
    pub sharp_tails: Polygons,
    pub cantilevers: Polygons,
    pub sharp_tails_height: f64,

    // Slices
    /// Layer.hpp:178
    pub lslices: ExPolygons,
    /// Layer.hpp:181
    pub lslices_extrudable: ExPolygons,
    /// Layer.hpp:184
    pub lslices_bboxes: Vec<BoundingBox>,

    /// Layer.hpp:187-189
    pub loverhangs: Polygons,
    pub loverhangs_with_type: Vec<(Polygon, u32)>,
    pub loverhangs_bbox: BoundingBox,

    /// Layer.hpp:192
    pub loop_nodes: Vec<LoopNode>,

    // Regions
    /// Layer.hpp:196
    regions: Vec<LayerRegion>,

    /// Support material fills for this layer.
    /// C++ SupportLayer::support_fills
    pub support_fills: Option<ExtrusionEntityCollection>,

    /// Snapshot of the owning PrintObject's config, shared via Arc.
    /// C++ reaches it via the parent pointer: `PrintObject *m_object;`
    /// (Layer.hpp:309) + `object()->config()` (Print.hpp:369). Stamped by
    /// PrintObject::wire_layer_hierarchy at sync points; None until wired.
    /// Present on support layers too (stamped at the end of
    /// generate_support_material).
    pub(crate) object_config: Option<Arc<crate::print_config::PrintObjectConfig>>,

    /// Snapshot of the owning Print's config, shared via Arc.
    /// C++: `object()->print()->config()` (PrintBase.hpp:632 + Print.hpp:885).
    /// Stamped at sync points; None until wired.
    pub(crate) print_config: Option<Arc<crate::print_config::PrintConfig>>,
}

// Marker traits for thread safety
// Layer.hpp:200-201
unsafe impl Send for Layer {}
unsafe impl Sync for Layer {}

impl Layer {
    /// Create a new layer
    /// Layer.cpp:312-345
    pub fn new(id: usize, object_id: usize, height: f64, print_z: f64, slice_z: f64) -> Self {
        Self {
            id,
            object_id,
            upper_layer_id: None,
            lower_layer_id: None,
            slicing_errors: false,
            slice_z,
            print_z,
            height,
            sharp_tails: Polygons::new(),
            cantilevers: Polygons::new(),
            sharp_tails_height: 0.0,
            lslices: ExPolygons::new(),
            lslices_extrudable: ExPolygons::new(),
            lslices_bboxes: Vec::new(),
            loverhangs: Polygons::new(),
            loverhangs_with_type: Vec::new(),
            loverhangs_bbox: BoundingBox::empty(),
            loop_nodes: Vec::new(),
            regions: Vec::new(),
            support_fills: None,
            object_config: None,
            print_config: None,
        }
    }

    /// Get layer ID
    /// Layer.hpp:204
    pub fn id(&self) -> usize {
        self.id
    }

    /// Set layer ID
    /// Layer.hpp:205
    pub fn set_id(&mut self, id: usize) {
        self.id = id;
    }

    /// Get object ID
    /// Layer.hpp:206
    pub fn object_id(&self) -> usize {
        self.object_id
    }

    /// Upward view to the owning PrintObject, preserving the C++ call shapes
    /// `layer->object()->config()` and `layer->object()->print()->config()`.
    /// C++: `PrintObject* object() { return m_object; }` (Layer.hpp:139)
    ///
    /// Works on region-less support layers too — they get their config Arcs
    /// stamped at the end of generate_support_material; only
    /// LayerRegion::region() is restricted to true printing regions.
    pub fn object(&self) -> crate::print_object::ObjectRef<'_> {
        crate::print_object::ObjectRef::new(
            self.object_config
                .as_deref()
                .expect("config hierarchy not wired — call wire_layer_hierarchy"),
            self.print_config
                .as_deref()
                .expect("config hierarchy not wired — call wire_layer_hierarchy"),
        )
    }

    /// Stamp the config-hierarchy Arcs onto this layer and its LayerRegions.
    /// Called by PrintObject::wire_layer_hierarchy at sync points (end of
    /// PrintObject::slice; end of generate_support_material for the support
    /// layers created there).
    ///
    /// The region Arc is looked up via each LayerRegion's region id into
    /// `all_regions`, mirroring the C++ ctor wiring
    /// `LayerRegion(Layer *layer, const PrintRegion *region)` (Layer.hpp:125).
    /// Support layers have no LayerRegions, so their `region` stays None.
    ///
    /// INVARIANT: the Arcs are cloned/replaced wholesale — NEVER mutated in
    /// place via Arc::make_mut/get_mut, which would fork the share. Faithful
    /// because C++ configs only mutate inside Print::apply and any diff there
    /// invalidates posSlice.
    pub(crate) fn wire_config_hierarchy(
        &mut self,
        object_config: &Arc<crate::print_config::PrintObjectConfig>,
        print_config: &Arc<crate::print_config::PrintConfig>,
        all_regions: &[Arc<crate::print_region::PrintRegion>],
    ) {
        self.object_config = Some(object_config.clone());
        self.print_config = Some(print_config.clone());
        for layerm in &mut self.regions {
            layerm.region = all_regions.get(layerm.region_id).cloned();
            layerm.object_config = Some(object_config.clone());
            layerm.print_config = Some(print_config.clone());
        }
    }

    /// Get bottom Z coordinate
    /// Layer.hpp:209
    pub fn bottom_z(&self) -> f64 {
        self.print_z - self.height
    }

    /// Get number of regions
    /// Layer.hpp:212
    pub fn region_count(&self) -> usize {
        self.regions.len()
    }

    /// Get a region by index
    /// Layer.hpp:215
    pub fn get_region(&self, idx: usize) -> Option<&LayerRegion> {
        self.regions.get(idx)
    }

    /// Get a mutable region by index
    /// Layer.hpp:218
    pub fn get_region_mut(&mut self, idx: usize) -> Option<&mut LayerRegion> {
        self.regions.get_mut(idx)
    }

    /// Get all regions
    /// Layer.hpp:221
    pub fn regions(&self) -> &[LayerRegion] {
        &self.regions
    }

    /// Get all regions mutably
    /// Layer.cpp:347
    pub fn regions_mut(&mut self) -> &mut [LayerRegion] {
        &mut self.regions
    }

    /// Add a new region
    /// Layer.cpp:349-355
    pub fn add_region(&mut self, region: LayerRegion) {
        self.regions.push(region);
    }

    // Test whether whether there are any slices assigned to this layer.
    // Layer.cpp:25
    pub fn empty(&self) -> bool {
        // Layer.cpp:27-31
        // C++: for (const LayerRegion *layerm : m_regions)
        // C++:     if (layerm != nullptr && ! layerm->slices.empty())
        // C++:         // Non empty layer.
        // C++:         return false;
        // C++: return true;
        for layerm in &self.regions {
            if !layerm.slices.is_empty() {
                // Non empty layer.
                return false;
            }
        }
        true
    }

    // If there is any incompatibility, separate LayerRegions have to be created.
    // Layer.cpp:173
    // C++: bool Layer::has_compatible_layer_regions(const PrintRegionConfig &config, const PrintRegionConfig &other_config)
    //
    // Porting notes (Rust PrintRegionConfig field-name mapping):
    //   wall_loops           -> perimeters
    //   inner_wall_speed     -> perimeter_speed (C++ per-extruder .get_at(get_process_config_idx(wall_filament)); Rust scalar)
    //   outer_wall_speed     -> external_perimeter_speed (same per-extruder note)
    //   gap_infill_speed     -> gap_fill_speed (same per-extruder note)
    //   detect_overhang_wall -> overhangs
    //   detect_thin_wall     -> thin_walls
    //   infill_wall_overlap  -> infill_overlap
    //   opt_serialize("inner_wall_line_width"/"outer_wall_line_width") -> direct float compare
    //   override_filament_scarf_seam_setting / seam_slope_* keep their C++ names
    pub fn has_compatible_layer_regions(
        &self,
        config: &PrintRegionConfig,
        other_config: &PrintRegionConfig,
    ) -> bool {
        // Layer.cpp:175
        config.wall_filament == other_config.wall_filament
            // Layer.cpp:176
            && config.perimeters == other_config.perimeters
            // Layer.cpp:177
            && config.wall_sequence == other_config.wall_sequence
            // Layer.cpp:178
            && config.perimeter_speed == other_config.perimeter_speed
            // Layer.cpp:179
            && config.external_perimeter_speed == other_config.external_perimeter_speed
            // Layer.cpp:180
            && config.gap_fill_speed == other_config.gap_fill_speed
            // Layer.cpp:181
            && config.overhangs == other_config.overhangs
            // Layer.cpp:182
            && config.filter_out_gap_fill == other_config.filter_out_gap_fill
            // Layer.cpp:183
            && config.inner_wall_line_width == other_config.inner_wall_line_width
            // Layer.cpp:184
            && config.outer_wall_line_width == other_config.outer_wall_line_width
            // Layer.cpp:185
            && config.thin_walls == other_config.thin_walls
            // Layer.cpp:186
            && config.infill_overlap == other_config.infill_overlap
            // Layer.cpp:187
            && config.override_filament_scarf_seam_setting == other_config.override_filament_scarf_seam_setting
            // Layer.cpp:188
            && config.seam_slope_type == other_config.seam_slope_type
            // Layer.cpp:189
            && config.seam_slope_start_height == other_config.seam_slope_start_height
            // Layer.cpp:190
            && config.seam_slope_gap == other_config.seam_slope_gap
            // Layer.cpp:191
            && config.seam_slope_min_length == other_config.seam_slope_min_length
            // Layer.cpp:192
            && config.seam_slope_conditional == other_config.seam_slope_conditional
            // Layer.cpp:193
            && config.seam_slope_entire_loop == other_config.seam_slope_entire_loop
            // Layer.cpp:194
            && config.seam_slope_steps == other_config.seam_slope_steps
            // Layer.cpp:195
            && config.seam_slope_inner_walls == other_config.seam_slope_inner_walls
    }

    /// Generate perimeters for all regions
    /// Layer.cpp:201-304
    /// Make perimeters with lower layer data for overhang detection
    pub fn make_perimeters_with_lower(
        &mut self,
        lower_slices: Option<&Vec<crate::geometry::ExPolygon>>,
    ) -> Result<()> {
        self.make_perimeters_with_neighbors(lower_slices, None)
    }

    /// Make perimeters with both lower- and upper-layer slices.
    /// upper_slices feeds the top-surface detection (top_fills, PerimeterGenerator.cpp:1118).
    pub fn make_perimeters_with_neighbors(
        &mut self,
        lower_slices: Option<&Vec<crate::geometry::ExPolygon>>,
        upper_slices: Option<&Vec<crate::geometry::ExPolygon>>,
    ) -> Result<()> {
        // Store neighbor slices in each region (used by LayerRegion::make_perimeters)
        if let Some(ls) = lower_slices {
            for region in &mut self.regions {
                region.lower_slices = Some(ls.clone());
            }
        }
        if let Some(us) = upper_slices {
            for region in &mut self.regions {
                region.upper_slices = Some(us.clone());
            }
        }
        self.make_perimeters(None)
    }

    // Here the perimeters are created cummulatively for all layer regions
    // sharing the same parameters influencing the perimeters.
    // Layer.cpp:201
    pub fn make_perimeters(
        &mut self,
        _perimeter_generator_options: Option<&()>,
    ) -> Result<()> {
        let print_z = self.print_z;
        let height = self.height;
        let id = self.id;

        // Layer.cpp:205
        // C++: std::vector<unsigned char> done(m_regions.size(), false);
        let mut done = vec![false; self.regions.len()];

        // Layer.cpp:207-208
        // C++: for (LayerRegionPtrs::iterator layerm = m_regions.begin(); layerm != m_regions.end(); ++ layerm)
        //      if ((*layerm)->slices.empty()) {
        for region_idx in 0..self.regions.len() {
            if self.regions[region_idx].slices.surfaces.is_empty() {
                // Layer.cpp:209-211
                // C++: (*layerm)->perimeters.clear();
                //      (*layerm)->fills.clear();
                //      (*layerm)->thin_fills.clear();
                self.regions[region_idx].perimeters.entities.clear();
                self.regions[region_idx].fills.entities.clear();
                self.regions[region_idx].thin_fills.entities.clear();
            } else {
                // Layer.cpp:213-215
                // C++: size_t region_id = layerm - m_regions.begin();
                //      if (done[region_id]) continue;
                if done[region_idx] {
                    continue;
                }
                // Layer.cpp:217
                // C++: done[region_id] = true;
                done[region_idx] = true;
                // Layer.cpp:218
                // C++: const PrintRegionConfig &config = (*layerm)->region().config();
                let config = self.regions[region_idx].region().config().clone();

                // Layer.cpp:221-222
                // find compatible regions
                // C++: LayerRegionPtrs layerms;  layerms.push_back(*layerm);
                let mut layerms: Vec<usize> = vec![region_idx];

                // Layer.cpp:224
                // C++: PerimeterRegions perimeter_regions;
                // FIDELITY-NOTE: PerimeterRegion::has_compatible_perimeter_regions
                // / merge_compatible_perimeter_regions (Layer.cpp:242-244, 270-272)
                // and the LayerRegion::make_perimeters(slices, perimeter_regions,
                // ...) overload that consumes them are not ported in this crate;
                // perimeter_regions is therefore not accumulated and the reduced
                // LayerRegion::make_perimeters is used.

                // Layer.cpp:225-245
                // C++: for (it = layerm + 1; it != m_regions.end(); ++it) { ... }
                for it in (region_idx + 1)..self.regions.len() {
                    // Layer.cpp:226-227
                    if self.regions[it].slices.surfaces.is_empty() {
                        continue;
                    }
                    // Layer.cpp:231-233
                    let other_config = self.regions[it].region().config().clone();
                    if !self.has_compatible_layer_regions(&config, &other_config) {
                        continue;
                    }
                    // Layer.cpp:235-237
                    self.regions[it].perimeters.entities.clear();
                    self.regions[it].fills.entities.clear();
                    self.regions[it].thin_fills.entities.clear();
                    // Layer.cpp:239-240
                    layerms.push(it);
                    done[it] = true;
                    // Layer.cpp:242-244: perimeter_regions.emplace_back — see note above.
                }

                if layerms.len() == 1 {
                    // Layer.cpp:247-252  (optimization)
                    // C++: (*layerm)->fill_surfaces.surfaces.clear();
                    //      (*layerm)->fill_no_overlap_expolygons.clear();
                    //      (*layerm)->make_perimeters((*layerm)->slices, perimeter_regions,
                    //          &(*layerm)->fill_surfaces, &(*layerm)->fill_no_overlap_expolygons, this->loop_nodes);
                    let surface_fill = self.regions[region_idx].slices.clone();
                    self.regions[region_idx].fill_surfaces.surfaces.clear();
                    self.regions[region_idx].fill_no_overlap_expolygons.clear();
                    self.regions[region_idx].make_perimeters(&surface_fill, height, id, print_z)?;
                    // Layer.cpp:254
                    // C++: (*layerm)->fill_expolygons = to_expolygons((*layerm)->fill_surfaces.surfaces);
                    // Already done inside LayerRegion::make_perimeters.
                } else {
                    // Layer.cpp:254-267
                    // C++: SurfaceCollection new_slices;
                    //      group slices according to number of extra perimeters,
                    //      merge each group with offset_ex(.., ClipperSafetyOffset).
                    let mut new_slices = SurfaceCollection::new();
                    {
                        // Layer.cpp:259-263
                        // C++: std::map<unsigned short, Surfaces> slices;
                        //      for (LayerRegion *layerm : layerms)
                        //          for (const Surface &surface : layerm->slices.surfaces)
                        //              slices[surface.extra_perimeters].emplace_back(surface);
                        use std::collections::BTreeMap;
                        let mut slices: BTreeMap<usize, Vec<Surface>> = BTreeMap::new();
                        for &lid in &layerms {
                            for surface in &self.regions[lid].slices.surfaces {
                                slices
                                    .entry(surface.extra_perimeters)
                                    .or_default()
                                    .push(surface.clone());
                            }
                        }
                        // Layer.cpp:264-266
                        // C++: for (.. surfaces_with_extra_perimeters : slices)
                        //          new_slices.append(offset_ex(surfaces_with_extra_perimeters.second,
                        //              ClipperSafetyOffset), surfaces_with_extra_perimeters.second.front());
                        for (_extra, group) in slices {
                            // offset_ex(surfaces, ClipperSafetyOffset) == union_safety_offset_ex.
                            let group_polys = crate::surface::to_polygons(&group);
                            let merged = crate::clipper_utils::union_safety_offset_ex(&group_polys);
                            // append(expolygons, template surface = group.front())
                            new_slices.append_expolygons_templ(merged, &group[0]);
                        }
                    }

                    // Layer.cpp:269-272: PerimeterRegion merge — see FIDELITY-NOTE above.

                    // Layer.cpp:275-278
                    // C++: SurfaceCollection fill_surfaces;  ExPolygons fill_no_overlap;
                    //      (*layerm)->make_perimeters(new_slices, perimeter_regions,
                    //          &fill_surfaces, &fill_no_overlap, this->loop_nodes);
                    // The reduced LayerRegion::make_perimeters writes into the
                    // region's own fill_surfaces / fill_no_overlap_expolygons; we
                    // snapshot those into the local fill_surfaces / fill_no_overlap
                    // used by the split below.
                    self.regions[region_idx].fill_surfaces.surfaces.clear();
                    self.regions[region_idx].fill_no_overlap_expolygons.clear();
                    self.regions[region_idx].make_perimeters(&new_slices, height, id, print_z)?;
                    let fill_surfaces = self.regions[region_idx].fill_surfaces.clone();
                    let fill_no_overlap = self.regions[region_idx].fill_no_overlap_expolygons.clone();

                    // Layer.cpp:281-290
                    // C++: if (!fill_surfaces.surfaces.empty()) {
                    //          for (l : layerms) {
                    //              ExPolygons expp = intersection_ex(fill_surfaces.surfaces, (*l)->slices.surfaces);
                    //              (*l)->fill_expolygons = expp;
                    //              (*l)->fill_surfaces.set(std::move(expp), fill_surfaces.surfaces.front());
                    //              (*l)->fill_no_overlap_expolygons = intersection_ex((*l)->slices.surfaces, fill_no_overlap);
                    //          }
                    //      }
                    if !fill_surfaces.surfaces.is_empty() {
                        let fs_templ = fill_surfaces.surfaces[0].clone();
                        for &lid in &layerms {
                            let slice_expp =
                                crate::surface::to_expolygons(&self.regions[lid].slices.surfaces);
                            // intersection_ex(fill_surfaces.surfaces, (*l)->slices.surfaces)
                            let expp = crate::clipper_utils::intersection_ex(
                                &fill_surfaces.surfaces,
                                &slice_expp,
                            );
                            // intersection_ex((*l)->slices.surfaces, fill_no_overlap)
                            let no_overlap = crate::clipper_utils::intersection_ex(
                                &self.regions[lid].slices.surfaces,
                                &fill_no_overlap,
                            );
                            self.regions[lid].fill_expolygons = expp.clone();
                            self.regions[lid].fill_surfaces.set_expolygons_templ(expp, &fs_templ);
                            self.regions[lid].fill_no_overlap_expolygons = no_overlap;
                        }
                    }
                }
            }
        }

        Ok(())
    }

    // merge all regions' slices to get islands
    // Layer.cpp:47
    pub fn make_slices(&mut self) {
        // Layer.cpp:49-58
        let slices: ExPolygons = if self.regions.len() == 1 {
            // Layer.cpp:50-52
            // optimization: if we only have one region, take its slices
            // C++: slices = to_expolygons(m_regions.front()->slices.surfaces);
            crate::surface::to_expolygons(&self.regions[0].slices.surfaces)
        } else {
            // Layer.cpp:54-57
            // C++: Polygons slices_p;
            //      for (LayerRegion *layerm : m_regions)
            //          polygons_append(slices_p, to_polygons(layerm->slices.surfaces));
            //      slices = union_safety_offset_ex(slices_p);
            let mut slices_p: Polygons = Polygons::new();
            for layerm in &self.regions {
                slices_p.extend(crate::surface::to_polygons(&layerm.slices.surfaces));
            }
            crate::clipper_utils::union_safety_offset_ex(&slices_p)
        };

        // Layer.cpp:60-61
        // C++: this->lslices.clear();
        //      this->lslices.reserve(slices.size());
        self.lslices.clear();
        self.lslices.reserve(slices.len());

        // Layer.cpp:63-67
        // prepare ordering points
        // C++: Points ordering_points;
        //      ordering_points.reserve(slices.size());
        //      for (const ExPolygon &ex : slices)
        //          ordering_points.push_back(ex.contour.first_point());
        let mut ordering_points: Vec<Point> = Vec::with_capacity(slices.len());
        for ex in &slices {
            ordering_points.push(ex.contour.first_point());
        }

        // Layer.cpp:69-70
        // sort slices
        // C++: std::vector<Points::size_type> order = chain_points(ordering_points);
        let order = chain_points(&ordering_points);

        // Layer.cpp:72-74
        // populate slices vector
        // C++: for (size_t i : order)
        //          this->lslices.emplace_back(std::move(slices[i]));
        for i in order {
            self.lslices.push(slices[i].clone());
        }
    }

    // Layer.cpp:77
    // C++: static inline bool layer_needs_raw_backup(const Layer *layer)
    //
    // GATE-NOTE (verified against BambuStudio Layer.cpp:77-82): the
    // elefant_foot_compensation read is COMMENTED OUT in the C++ source —
    // BambuStudio backs up raw slices unconditionally ("BBS: backup raw slice
    // for generating support"). The faithful port therefore keeps `true` as
    // the live return; the commented expression below is the disabled C++
    // line rendered with this crate's config-hierarchy reads.
    fn layer_needs_raw_backup(&self) -> bool {
        // BBS: backup raw slice for generating support
        // Layer.cpp:80 (disabled in C++):
        //return !(self.regions().len() == 1
        //    && (self.id() > 0 || self.object().config().elephant_foot_compensation == 0.0));
        // Layer.cpp:81
        true
    }

    // Layer.cpp:84
    pub fn backup_untyped_slices(&mut self) {
        // Layer.cpp:86 — layer_needs_raw_backup(this) is always true (Layer.cpp:81).
        if self.layer_needs_raw_backup() {
            // Layer.cpp:87-95
            // C++: for (LayerRegion *layerm : m_regions) {
            //          layerm->raw_slices = to_expolygons(layerm->slices.surfaces);
            //          layerm->raw_counter_circle_compensation.clear();
            //          layerm->raw_holes_circle_compensation.clear();
            //          for (Surface &surface : layerm->slices.surfaces) {
            //              layerm->raw_counter_circle_compensation.push_back(surface.counter_circle_compensation);
            //              layerm->raw_holes_circle_compensation.push_back(surface.holes_circle_compensation);
            //          }
            //      }
            // raw_slices stores the untyped expolygons of the current slices.
            for region in &mut self.regions {
                region.raw_slices = region.slices.clone();
                region.raw_counter_circle_compensation.clear();
                region.raw_holes_circle_compensation.clear();
                // FIDELITY-NOTE: Surface in this crate does not model the
                // per-surface counter_circle_compensation (bool) /
                // holes_circle_compensation (Vec<int>) flags (Surface.hpp:41-42);
                // the per-surface flag push_back loop is therefore a no-op here.
            }
        } else {
            // Layer.cpp:96-101
            // C++: assert(m_regions.size() == 1);
            //      m_regions.front()->raw_slices.clear();
            //      m_regions.front()->raw_counter_circle_compensation.clear();
            //      m_regions.front()->raw_holes_circle_compensation.clear();
            debug_assert_eq!(self.regions.len(), 1);
            if let Some(front) = self.regions.first_mut() {
                front.raw_slices.surfaces.clear();
                front.raw_counter_circle_compensation.clear();
                front.raw_holes_circle_compensation.clear();
            }
        }
    }

    // Layer.cpp:104
    pub fn restore_untyped_slices(&mut self) {
        // Layer.cpp:106 — layer_needs_raw_backup(this) is always true.
        if self.layer_needs_raw_backup() {
            // Layer.cpp:107-117
            // C++: for (LayerRegion *layerm : m_regions) {
            //          layerm->slices.set(layerm->raw_slices, stInternal);
            //          ... (per-surface circle_compensation copy)
            //      }
            for region in &mut self.regions {
                let raw = region.raw_slices.to_expolygons();
                region.slices.set(&raw, SurfaceType::Internal);
                // FIDELITY-NOTE: per-surface counter/holes circle_compensation
                // copy (Layer.cpp:110-115) is omitted — Surface does not model
                // those flags in this crate.
            }
        } else {
            // Layer.cpp:118-127
            // C++: assert(m_regions.size() == 1);
            //      m_regions.front()->slices.set(this->lslices, stInternal);
            debug_assert_eq!(self.regions.len(), 1);
            let lslices = self.lslices.clone();
            if let Some(front) = self.regions.first_mut() {
                front.slices.set(&lslices, SurfaceType::Internal);
            }
        }
    }

    // Similar to Layer::restore_untyped_slices()
    // Layer.cpp:134
    pub fn restore_untyped_slices_no_extra_perimeters(&mut self) {
        // Layer.cpp:136 — layer_needs_raw_backup(this) is always true.
        if self.layer_needs_raw_backup() {
            // Layer.cpp:137-140
            // C++: for (LayerRegion *layerm : m_regions)
            //          //if (! layerm->region().config().extra_perimeters.value)  // always false
            //              layerm->slices.set(layerm->raw_slices, stInternal);
            for region in &mut self.regions {
                let raw = region.raw_slices.to_expolygons();
                region.slices.set(&raw, SurfaceType::Internal);
            }
        } else {
            // Layer.cpp:141-146
            // C++: assert(m_regions.size() == 1);
            //      LayerRegion *layerm = m_regions.front();
            //      //if (! layerm->region().config().extra_perimeters.value)
            //          layerm->slices.set(this->lslices, stInternal);
            debug_assert_eq!(self.regions.len(), 1);
            let lslices = self.lslices.clone();
            if let Some(front) = self.regions.first_mut() {
                front.slices.set(&lslices, SurfaceType::Internal);
            }
        }
    }

    // Layer.cpp:150
    // C++: ExPolygons Layer::merged(float offset_scaled) const
    pub fn merged(&self, offset_scaled: f64) -> ExPolygons {
        // Layer.cpp:152
        debug_assert!(offset_scaled >= 0.0);
        // Layer.cpp:153-158
        // If no offset is set, apply EPSILON offset before union, and revert it afterwards.
        let mut offset_scaled = offset_scaled;
        let mut offset_scaled2 = 0.0_f64;
        if offset_scaled == 0.0 {
            offset_scaled = crate::libslic3r::EPSILON;
            offset_scaled2 = -crate::libslic3r::EPSILON;
        }
        // Layer.cpp:159-165
        // C++: Polygons polygons;
        //      for (LayerRegion *layerm : m_regions) {
        //          const PrintRegionConfig &config = layerm->region().config();
        //          if (config.bottom_shell_layers > 0 || config.top_shell_layers > 0 ||
        //              config.sparse_infill_density > 0. || config.wall_loops > 0)
        //              append(polygons, offset(layerm->slices.surfaces, offset_scaled));
        //      }
        let mut polygons: ExPolygons = Vec::new();
        for layerm in &self.regions {
            let config = layerm.region().config();
            if config.bottom_solid_layers > 0
                || config.top_solid_layers > 0
                || config.fill_density > 0.0
                || config.perimeters > 0
            {
                let surf_ex = crate::surface::to_expolygons(&layerm.slices.surfaces);
                polygons.extend(crate::clipper_utils::offset_expolygons(
                    &surf_ex,
                    offset_scaled,
                    OffsetJoinType::Miter,
                ));
            }
        }
        // Layer.cpp:166
        // C++: ExPolygons out = union_ex(polygons);
        let mut out = union_ex(&polygons);
        // Layer.cpp:167-168
        // C++: if (offset_scaled2 != 0.f)
        //          out = offset_ex(out, offset_scaled2);
        if offset_scaled2 != 0.0 {
            out = crate::clipper_utils::offset_expolygons(&out, offset_scaled2, OffsetJoinType::Miter);
        }
        // Layer.cpp:169
        out
    }

    // Layer.cpp:39
    pub fn apply_auto_circle_compensation(&mut self) {
        // Layer.cpp:41-43
        // C++: for (LayerRegion *layerm : m_regions) {
        //          layerm->auto_circle_compensation(layerm->slices,
        //              this->object()->get_auto_circle_compenstaion_params(),
        //              scale_(this->object()->config().circle_compensation_manual_offset));
        //      }
        // FIDELITY-NOTE: LayerRegion::auto_circle_compensation,
        // PrintObject::get_auto_circle_compenstaion_params, and the
        // circle_compensation_manual_offset object-config field are not yet
        // ported (LayerRegion.cpp / PrintObject), so the per-region call body
        // is a no-op. The region-iteration skeleton mirrors C++ Layer.cpp:41.
        for _layerm in &mut self.regions {
            // layerm.auto_circle_compensation(...);  // not ported
        }
    }

    // Layer.cpp:354
    // C++: void Layer::calculate_perimeter_continuity(std::vector<LoopNode> &prev_nodes)
    //
    // FIDELITY-NOTE: not faithfully portable in this crate yet. The C++ body
    // (Layer.cpp:355-431) walks `loop_nodes[*].node_contour.{pts,widths,is_loop}`
    // and queries a `ContinuitiousDistancer` (AABBTreeLines squared-distance,
    // Layer.cpp:298-340) to thread continuity edges between this layer's nodes
    // and `prev_nodes`. This crate's `layer::LoopNode` (used by print_object.rs)
    // does NOT model `node_contour`, and there is no AABBTreeLines distancer nor
    // `BoundingBox::overlap`. The outer node-iteration skeleton is reproduced;
    // the distance-driven edge linking is omitted pending that infrastructure.
    pub fn calculate_perimeter_continuity(&mut self, _prev_nodes: &mut [LoopNode]) {
        // Layer.cpp:355
        // C++: for (size_t node_pos = 0; node_pos < loop_nodes.size(); ++node_pos) {
        for _node_pos in 0..self.loop_nodes.len() {
            // Layer.cpp:356-430: node_contour distancer + prev_nodes bbox-overlap
            // loop + continuity polyline accumulation — omitted (see note above).
        }
    }

    // Layer.cpp:579
    // C++: coordf_t Layer::get_sparse_infill_max_void_area()
    pub fn get_sparse_infill_max_void_area(&self) -> Result<f64> {
        use crate::print_config::InfillPattern as Ip;
        // Layer.cpp:581
        let mut max_void_area = 0.0_f64;
        // Layer.cpp:582
        // C++: for (auto layerm : m_regions) {
        for layerm in &self.regions {
            // Layer.cpp:583
            // C++: Flow flow = layerm->flow(frInfill);  (uses m_layer->height)
            let flow = layerm.flow(FlowRole::Infill, self.height)?;
            // Layer.cpp:584
            // C++: float density = layerm->region().config().sparse_infill_density;
            let density = layerm.region().config().fill_density;
            // Layer.cpp:585
            // C++: InfillPattern pattern = layerm->region().config().sparse_infill_pattern;
            let pattern = layerm.region().config().fill_pattern;
            // Layer.cpp:586-587
            // C++: if (density == 0.) return -1;
            if density == 0.0 {
                return Ok(-1.0);
            }

            // Layer.cpp:589-590
            // BBS: rough estimation and need to be optimized
            // C++: double spacing = flow.scaled_spacing() * (100 - density) / density;
            let spacing = flow.scaled_spacing() as f64 * (100.0 - density) / density;
            // Layer.cpp:591-619
            match pattern {
                // Layer.cpp:592-603
                Ip::Concentric
                | Ip::Rectilinear
                | Ip::Line
                | Ip::Gyroid
                | Ip::AlignedRectilinear
                | Ip::OctagramSpiral
                | Ip::HilbertCurve
                | Ip::Honeycomb3D
                | Ip::ArchimedeanChords
                | Ip::Lattice2D => {
                    max_void_area = max_void_area.max(spacing * spacing);
                }
                // Layer.cpp:604-608
                Ip::Grid | Ip::Honeycomb | Ip::Lightning => {
                    max_void_area = max_void_area.max(4.0 * spacing * spacing);
                }
                // Layer.cpp:609-615
                Ip::Cubic | Ip::AdaptiveCubic | Ip::Triangles | Ip::Stars | Ip::SupportCubic => {
                    max_void_area = max_void_area.max(4.5 * spacing * spacing);
                }
                // Layer.cpp:616-618
                _ => {
                    max_void_area = max_void_area.max(spacing * spacing);
                }
            }
        }
        // Layer.cpp:621
        Ok(max_void_area)
    }

    /// Generate fill patterns for all regions
    /// Fill.cpp:586-768
    pub fn make_fills(
        &mut self,
        lower_internal_areas: &[crate::geometry::ExPolygon],
    ) -> Result<()> {
        // Fill.cpp:600
        // C++: const auto resolution = this->object()->print()->config().resolution.value;
        // Read up front through the layer's own print-config Arc (stamped by
        // wire_config_hierarchy) so no shared borrow overlaps the region
        // mutation below. Still underscore-bound: the downstream consumer
        // (params.resolution, Fill.cpp:705, used for path simplification) is
        // not ported yet.
        let resolution = self.object().print().config().resolution;

        // Fill.cpp:588-590
        // C++: for (LayerRegion *layerm : m_regions)
        //          layerm->fills.clear();
        for region in &mut self.regions {
            region.fills.entities.clear();
        }

        // Fill.cpp:594-596
        // C++: LockRegionParam lock_param;
        //      set_outlook_range(lock_param);
        //      std::vector<SurfaceFill> surface_fills = group_fills(*this, lock_param);
        let mut lock_param = crate::fill::LockRegionParam::default();
        let surface_fills = crate::fill::group_fills(self, lower_internal_areas, &mut lock_param)?;

        // Fill.cpp:597-598
        // C++: const Slic3r::BoundingBox bbox = this->object()->bounding_box();
        //      const auto resolution = this->object()->print()->config().resolution.value;
        let _bbox = crate::geometry::BoundingBox::empty(); // TODO: get from object

        // Fill.cpp:605-750
        // C++: for (SurfaceFill &surface_fill : surface_fills)
        for surface_fill in surface_fills {
            // Fill.cpp:607-632
            // Create the filler object
            // C++: std::unique_ptr<Fill> f = std::unique_ptr<Fill>(Fill::new_from_type(surface_fill.params.pattern));
            // TODO: Implement Fill factory (Fill::new_from_type)

            let fill_pattern = surface_fill.params.pattern;

            // Fill.cpp:678-693
            // Calculate spacing and link_max_length
            // C++: bool using_internal_flow = ! surface_fill.surface.is_solid() && ! surface_fill.params.bridge;
            let _using_internal_flow =
                !surface_fill.surface.is_solid() && !surface_fill.params.bridge;

            // C++: double link_max_length = 0.;
            // Fill.cpp:683-695 computes link_max_length in mm, then
            // f->link_max_length = scale_(link_max_length).
            let link_max_length_mm = if !surface_fill.params.bridge {
                // Fill.cpp:687-693
                if surface_fill.params.density > 80.0 {
                    // C++: link_max_length = 3. * f->spacing;
                    3.0 * surface_fill.params.spacing
                } else {
                    0.0
                }
            } else {
                0.0
            };
            let link_max_length = crate::scale(link_max_length_mm);

            let density = (0.01 * surface_fill.params.density) as f64;
            let is_grid = fill_pattern == InfillPattern::Grid;
            // C++: FillParams::dont_connect() == (anchor_length_max < 0.05f)
            // (FillBase.hpp). The infill traversal connects lines whenever
            // dont_connect() is false; link_max_length only bounds the length
            // of an individual perimeter link, it does NOT disable connection.
            // Solid/bridge infill has anchor_length_max == 1000 (never
            // disconnected) and sparse infill uses infill_anchor_max (default
            // 12mm), so both connect here. Patterns like Line set
            // anchor_length_max == 0 to force dont_connect == true.
            let dont_connect = surface_fill.params.anchor_length_max < 0.05;
            let infill_config = InfillConfig {
                pattern: fill_pattern,
                line_spacing: surface_fill.params.spacing,
                angle: surface_fill.params.angle as f64,
                angle_increment: 90.0,
                density,
                extrusion_width: surface_fill.params.spacing,
                overlap: surface_fill.params.spacing * 0.15,
                connect_infill: !dont_connect,
                link_max_length,
            };

            // Fill.cpp:735-747
            // C++: LayerRegion* layerm = this->m_regions[surface_fill.region_id];
            //      for (ExPolygon& expoly : surface_fill.expolygons)
            let region_id = surface_fill.region_id;

            // FillMonotonicLineWGapFill (ipMonotonicLine) lays the monotonic
            // lines and then medial-axis gap-fills the band the lines left
            // uncovered (FillRectilinear.cpp:3260-3320). Capture the per-
            // surface no_overlap area and accumulate the footprint of the laid
            // lines so the gap-fill can run once, after the expoly loop.
            let is_monotonic_line = fill_pattern == InfillPattern::MonotonicLine;
            // C++ (Fill.cpp:740) intersects no_overlap with EACH expoly before
            // filling: f->no_overlap = intersection_ex(surface_fill.no_overlap, {expoly}).
            // Aggregate that here by intersecting with the surface's expolygons.
            let mono_no_overlap: ExPolygons = if is_monotonic_line
                && !surface_fill.no_overlap_expolygons.is_empty()
                && !surface_fill.expolygons.is_empty()
            {
                crate::clipper_utils::intersection(
                    &surface_fill.no_overlap_expolygons,
                    &surface_fill.expolygons,
                )
            } else {
                Vec::new()
            };
            let mono_flow = surface_fill.params.flow.clone();
            let mono_density = surface_fill.params.density;
            // Footprint (covered-by-spacing) of all monotonic lines emitted for
            // this surface_fill — C++ coll_nosort->polygons_covered_by_spacing(10).
            let mut mono_covered: Vec<crate::geometry::Polygon> = Vec::new();

            for expoly in surface_fill.expolygons {
                // Skip empty polygons
                if expoly.contour.points.is_empty() {
                    continue;
                }

                // Save contour for connect_infill boundary before expoly may be moved
                let boundary_contour = expoly.contour.clone();

                let generated = match fill_pattern {
                    InfillPattern::Rectilinear | InfillPattern::Grid => generate_fill_rectilinear(
                        &[expoly],
                        &infill_config,
                        self.id as usize,
                        is_grid,
                    ),
                    // Monotonic top/bottom/solid surfaces: FillMonotonic /
                    // FillMonotonicLine (FillRectilinear.hpp:47-62; their
                    // fill_surface, FillRectilinear.cpp:3090-3109). Both set
                    // params.monotonic = true; FillMonotonicLine *also* sets
                    // anchor_length_max = 0 => dont_connect()==true, so its lines
                    // are emitted SEPARATELY (native travels between them), whereas
                    // FillMonotonic keeps the perimeter links (anchor default).
                    // no_sort()==true for both => we skip the post connect_infill.
                    InfillPattern::Monotonic | InfillPattern::MonotonicLine => {
                        // FillMonotonicLine forces dont_connect (anchor_length_max=0).
                        let mut mono_config = infill_config.clone();
                        if fill_pattern == InfillPattern::MonotonicLine {
                            mono_config.connect_infill = false;
                        }
                        generate_fill_rectilinear_monotonic(
                            &[expoly],
                            &mono_config,
                            self.id as usize,
                            is_grid,
                            true,
                        )
                    }
                    InfillPattern::Gyroid => {
                        // Use gyroid fill — generate paths for bounding box then clip to fill area
                        use crate::fill::fill_gyroid::{generate_gyroid_infill, GyroidConfig};
                        use crate::geometry::BoundingBox;
                        let mut bb = BoundingBox::empty();
                        for pt in &expoly.contour.points {
                            bb.merge_point(*pt);
                        }
                        // C++ FillGyroid uses raw line spacing (not density-adjusted)
                        // and applies density via DENSITY_ADJUST internally.
                        let raw_line_width = surface_fill.params.flow.width();
                        let gyroid_config = GyroidConfig {
                            z: self.print_z,
                            spacing: raw_line_width,
                            density: density,
                            angle: surface_fill.params.angle as f64,
                        };
                        let raw_polylines = generate_gyroid_infill(&gyroid_config, bb.min, bb.max);
                        // Clip polylines to the fill expolygon
                        let mut clipped = Vec::new();
                        for pl in raw_polylines {
                            let mut current_segment = Vec::new();
                            for pt in &pl.points {
                                if expoly.contour.contains_point(pt) {
                                    current_segment.push(*pt);
                                } else {
                                    if current_segment.len() >= 2 {
                                        clipped.push(InfillPath::Line(
                                            crate::geometry::Polyline::from_points(current_segment),
                                        ));
                                    }
                                    current_segment = Vec::new();
                                }
                            }
                            if current_segment.len() >= 2 {
                                clipped.push(InfillPath::Line(
                                    crate::geometry::Polyline::from_points(current_segment),
                                ));
                            }
                        }
                        clipped
                    }
                    // All other patterns: dispatch through generate_infill which
                    // routes to the correct fill implementation
                    _ => {
                        let polylines = generate_infill(
                            fill_pattern,
                            &[expoly.clone()],
                            infill_config.line_spacing,
                            infill_config.angle,
                        )?;
                        polylines.into_iter().map(InfillPath::Line).collect()
                    }
                };

                if generated.is_empty() {
                    continue;
                }

                let mut polylines = Vec::new();
                for path in generated {
                    match path {
                        InfillPath::Line(pl) => polylines.push(pl),
                        InfillPath::Loop(poly) => polylines.push(poly.split_at_first_point()),
                    }
                }

                if polylines.is_empty() {
                    continue;
                }

                // Connect separate infill lines into continuous paths
                // C++: Fill::connect_infill() - called after fill generation.
                // Monotonic fillers (FillMonotonic / FillMonotonicLine) set
                // no_sort()==true and emit their lines already in the final
                // sweep order WITHOUT cross-line connection; running
                // connect_infill on them would re-chain the lines into long
                // polylines (the divergence from native, which travels between
                // monotonic lines). Skip the connect step for those patterns.
                let is_monotonic = matches!(
                    fill_pattern,
                    InfillPattern::Monotonic | InfillPattern::MonotonicLine
                );
                if infill_config.connect_infill && !is_monotonic {
                    let boundary = vec![boundary_contour.clone()];
                    let fill_params = crate::fill::FillParams::new();
                    let mut connected = Vec::new();
                    crate::fill::connect_infill(
                        polylines,
                        &boundary,
                        infill_config.line_spacing,
                        &fill_params,
                        &mut connected,
                    );
                    polylines = connected;
                }

                if polylines.is_empty() {
                    continue;
                }

                // Convert to extrusion paths
                let mm3_per_mm = surface_fill.params.flow.mm3_per_mm()?;
                let mut collection = ExtrusionEntityCollection::new();
                extrusion_entities_append_paths(
                    &mut collection.entities,
                    polylines,
                    surface_fill.params.extrusion_role,
                    mm3_per_mm,
                    surface_fill.params.flow.width() as f32,
                    surface_fill.params.flow.height() as f32,
                );

                // FillMonotonicLineWGapFill: record the footprint of the laid
                // monotonic lines so the post-loop gap-fill can subtract it from
                // the no_overlap area (FillRectilinear.cpp:3279
                // coll_nosort->polygons_covered_by_spacing(10)).
                if is_monotonic_line {
                    for ent in &collection.entities {
                        if let crate::extrusion_entity::ExtrusionEntityType::Path(path) = ent {
                            path.polygons_covered_by_spacing(&mut mono_covered, 10.0);
                        }
                    }
                }

                if !collection.entities.is_empty() {
                    self.regions[region_id].fills.entities.push(
                        crate::extrusion_entity::ExtrusionEntityType::Collection(Box::new(
                            collection,
                        )),
                    );
                }
            }

            // FillMonotonicLineWGapFill gap-fill tail (FillRectilinear.cpp:3260-3320).
            // After the monotonic lines are laid, medial-axis gap-fill the band of
            // the no_overlap area the lines left uncovered, emitting variable-width
            // erGapFill runs. These curved runs are what the arc-fitter turns into
            // G2/G3 on the top surface (native: 113 arcs; without this Rust got 4).
            if is_monotonic_line && mono_density >= 1.0 && !mono_no_overlap.is_empty() {
                use crate::clipper_utils::{
                    difference_clib, intersection_ex_expolygons_polygons, offset2_clib,
                    union_polygons_ex, OffsetJoinType,
                };
                use crate::extrusion_entity::{ExtrusionEntityType, ExtrusionRole};
                use crate::perimeter_generator::convert_thin_walls_to_extrusion_paths;

                // C++: ExPolygons unextruded_areas = diff_ex(no_overlap,
                //          union_ex(coll_nosort->polygons_covered_by_spacing(10)));
                // diff_ex via the vertex-exact vendored ClipperLib (difference_clib).
                let unextruded_areas: ExPolygons = if mono_covered.is_empty() {
                    mono_no_overlap.clone()
                } else {
                    let covered_ex = union_polygons_ex(&mono_covered);
                    difference_clib(&mono_no_overlap, &covered_ex)
                };

                // C++: gapfill_areas = union_ex(unextruded_areas);
                //      gapfill_areas = intersection_ex(gapfill_areas, no_overlap);
                let gapfill_areas = intersection_ex_expolygons_polygons(
                    &unextruded_areas,
                    &mono_no_overlap
                        .iter()
                        .flat_map(|ex| {
                            std::iter::once(ex.contour.clone()).chain(ex.holes.iter().cloned())
                        })
                        .collect::<Vec<_>>(),
                );

                if !gapfill_areas.is_empty() {
                    // C++: new_flow = params.flow.with_spacing(this->spacing) for
                    // solid fill; the top surface uses the surface flow directly.
                    let new_flow = mono_flow.clone();
                    // C++ works in SCALED units (new_flow.scaled_spacing()); the Rust
                    // clipper_utils offset/opening helpers operate in mm, so use the
                    // UNSCALED spacing here (mirrors the perimeter gap-fill block above,
                    // layer.rs:566-567 which uses flow.width()/spacing() in mm).
                    let spacing_mm = new_flow.spacing();
                    // C++: min = 0.2 * scaled_spacing * (1 - INSET_OVERLAP_TOLERANCE)
                    //      max = 2. * scaled_spacing   (INSET_OVERLAP_TOLERANCE = 0.45)
                    let min = 0.2 * spacing_mm * (1.0 - 0.45);
                    let max = 2.0 * spacing_mm;
                    const CLIPPER_SAFETY_OFFSET: f64 = 0.0001;

                    // C++: gaps_ex = diff_ex(opening_ex(gapfill_areas, min/2),
                    //          offset2_ex(gapfill_areas, -max/2, max/2 + ClipperSafetyOffset));
                    // opening_ex(g, d) == offset2_ex(g, -d, +d); both opening_ex and
                    // offset2_ex here take no explicit join type, so DefaultJoinType =
                    // jtMiter, DefaultMiterLimit = 3.0 (ClipperUtils.hpp:31,37). Route
                    // the offsets + diff_ex through the vertex-exact vendored ClipperLib
                    // (offset2_clib / difference_clib) so the gap-band contours feeding
                    // medial_axis are not over-segmented (vs geo-clipper @ 1µm).
                    let opened_min =
                        offset2_clib(&gapfill_areas, min / 2.0, min / 2.0, OffsetJoinType::Miter);
                    let wide_part = offset2_clib(
                        &gapfill_areas,
                        max / 2.0,
                        max / 2.0 + CLIPPER_SAFETY_OFFSET,
                        OffsetJoinType::Miter,
                    );
                    let mut gaps_ex = difference_clib(&opened_min, &wide_part);

                    if !gaps_ex.is_empty() {
                        // C++: ex.douglas_peucker(SCALED_RESOLUTION * 0.1) then
                        //      ex.medial_axis(min, max, &polylines).
                        let scaled_resolution = crate::scale(resolution) as f64;
                        let simplify_resolution = scaled_resolution * 0.1;
                        let mut polylines: crate::geometry::ThickPolylines = Vec::new();
                        for ex in &mut gaps_ex {
                            ex.douglas_peucker(simplify_resolution);
                            ex.medial_axis(min, max, &mut polylines);
                        }

                        if !polylines.is_empty() {
                            // C++: variable_width(polylines, erGapFill, params.flow, ...)
                            let paths = convert_thin_walls_to_extrusion_paths(
                                &polylines,
                                ExtrusionRole::GapFill,
                                &mono_flow,
                            );
                            if !paths.is_empty() {
                                let mut coll = ExtrusionEntityCollection::new();
                                coll.no_sort = true;
                                for path in paths {
                                    coll.entities.push(ExtrusionEntityType::Path(path));
                                }
                                self.regions[region_id]
                                    .fills
                                    .entities
                                    .push(ExtrusionEntityType::Collection(Box::new(coll)));
                            }
                        }
                    }
                }
            }
        }

        // Fill.cpp:753-763
        // Add thin fill regions
        // C++: for (LayerRegion *layerm : m_regions)
        //          for (const ExtrusionEntity *thin_fill : layerm->thin_fills.entities)
        for region in &mut self.regions {
            for _thin_fill in &region.thin_fills.entities {
                // Fill.cpp:757-760
                // C++: ExtrusionEntityCollection &collection = *(new ExtrusionEntityCollection());
                //      layerm->fills.entities.push_back(&collection);
                //      collection.entities.push_back(thin_fill->clone());
                let collection = ExtrusionEntityCollection::new();
                region.fills.entities.push(
                    crate::extrusion_entity::ExtrusionEntityType::Collection(Box::new(collection)),
                );
            }
        }

        Ok(())
    }

    /// Generate the sparse-infill polylines used to anchor internal bridges.
    ///
    /// 1:1 (faithful) port of
    /// `Layer::generate_sparse_infill_polylines_for_anchoring`
    /// (Fill/Fill.cpp:772-875). Consumed by `PrintObject::bridge_over_infill`
    /// (PrintObject.cpp:2391-2401 producer, :2845 consumer) where the returned
    /// polylines become the anchor lines that bound internal-bridge expansion.
    ///
    /// This is a const operation (it does not mutate `fill_surfaces` or
    /// `fills`): it re-derives the surface-fill groups via `group_fills`,
    /// processes ONLY `stInternal` surfaces, runs the same per-pattern fill
    /// dispatch as `make_fills`, and returns the raw polylines (no
    /// extrusion-entity conversion, no thin fills).
    ///
    /// Adaptive-cubic / lightning patterns are not ported; for those patterns
    /// no anchor lines are produced (the C++ filler would consult an octree /
    /// lightning generator we do not have). For the common grid / rectilinear /
    /// line / concentric / gyroid sparse-infill patterns this produces the same
    /// anchor polylines as native.
    pub fn generate_sparse_infill_polylines_for_anchoring(&self) -> Result<Vec<Polyline>> {
        // NOTE: `surface_fill.params.pattern` is `crate::fill::InfillPattern`
        // (group_fills' type), which is the `InfillPattern` already imported at
        // the top of this module (layer.rs:23). Do NOT import
        // `crate::print_config::InfillPattern` here — it is a distinct enum.
        use crate::surface::SurfaceType;

        // Fill.cpp:774-775
        // C++: LockRegionParam skin_inner_param;
        //      std::vector<SurfaceFill> surface_fills = group_fills(*this, skin_inner_param);
        // group_fills only consults `lower_internal_areas` for stInternalSolid
        // (narrow-solid -> ipConcentricInternal) surfaces; this function emits
        // only stInternal surfaces, so an empty slice is faithful here.
        let mut lock_param = crate::fill::LockRegionParam::default();
        let surface_fills = crate::fill::group_fills(self, &[], &mut lock_param)?;

        // Fill.cpp:779
        // C++: Polylines sparse_infill_polylines{};
        let mut sparse_infill_polylines: Vec<Polyline> = Vec::new();

        // Fill.cpp:781
        // C++: for (SurfaceFill &surface_fill : surface_fills)
        for surface_fill in surface_fills {
            // Fill.cpp:782-784
            // C++: if (surface_fill.surface.surface_type != stInternal) continue;
            if surface_fill.surface.surface_type != SurfaceType::Internal {
                continue;
            }

            let fill_pattern = surface_fill.params.pattern;

            // Fill.cpp:786-812
            // C++: switch over pattern; ipCount / ipSupportBase -> continue.
            // The remaining patterns all break (fall through to generation).
            // Adaptive-cubic / support-cubic / lightning need an octree or
            // lightning generator that is not ported here, so they cannot
            // produce anchor lines -> skip them (no anchors, faithful to
            // "no adaptive/lightning" Benchy case). `fill::InfillPattern`
            // collapses ipAdaptiveCubic/ipSupportCubic into `Adaptive`.
            match fill_pattern {
                InfillPattern::Adaptive | InfillPattern::Lightning => continue,
                _ => {}
            }

            // Fill.cpp:826-836
            // C++: link_max_length = 0; if (!bridge && density > 80%) link_max_length = 3*spacing
            let link_max_length_mm = if !surface_fill.params.bridge {
                if surface_fill.params.density > 80.0 {
                    3.0 * surface_fill.params.spacing
                } else {
                    0.0
                }
            } else {
                0.0
            };
            let link_max_length = crate::scale(link_max_length_mm);

            // Fill.cpp:846-854 — FillParams. params.density = 0.01 * density.
            let density = (0.01 * surface_fill.params.density) as f64;
            let is_grid = fill_pattern == InfillPattern::Grid;
            // Mirror make_fills' dont_connect derivation
            // (anchor_length_max < 0.05 disables connection; FillBase.hpp).
            let dont_connect = surface_fill.params.anchor_length_max < 0.05;
            let infill_config = InfillConfig {
                pattern: fill_pattern,
                line_spacing: surface_fill.params.spacing,
                angle: surface_fill.params.angle as f64,
                angle_increment: 90.0,
                density,
                extrusion_width: surface_fill.params.spacing,
                overlap: surface_fill.params.spacing * 0.15,
                connect_infill: !dont_connect,
                link_max_length,
            };

            // Fill.cpp:862-871
            // C++: for (ExPolygon &expoly : surface_fill.expolygons)
            //          polylines = f->fill_surface(&surface_fill.surface, params);
            //          sparse_infill_polylines.insert(..., polylines);
            // We mirror make_fills' polyline dispatch exactly, but accumulate
            // the polylines instead of converting them to extrusion entities.
            for expoly in surface_fill.expolygons {
                if expoly.contour.points.is_empty() {
                    continue;
                }

                let boundary_contour = expoly.contour.clone();

                let generated = match fill_pattern {
                    InfillPattern::Rectilinear | InfillPattern::Grid => generate_fill_rectilinear(
                        &[expoly],
                        &infill_config,
                        self.id as usize,
                        is_grid,
                    ),
                    InfillPattern::Gyroid => {
                        use crate::fill::fill_gyroid::{generate_gyroid_infill, GyroidConfig};
                        use crate::geometry::BoundingBox;
                        let mut bb = BoundingBox::empty();
                        for pt in &expoly.contour.points {
                            bb.merge_point(*pt);
                        }
                        let raw_line_width = surface_fill.params.flow.width();
                        let gyroid_config = GyroidConfig {
                            z: self.print_z,
                            spacing: raw_line_width,
                            density,
                            angle: surface_fill.params.angle as f64,
                        };
                        let raw_polylines = generate_gyroid_infill(&gyroid_config, bb.min, bb.max);
                        let mut clipped = Vec::new();
                        for pl in raw_polylines {
                            let mut current_segment = Vec::new();
                            for pt in &pl.points {
                                if expoly.contour.contains_point(pt) {
                                    current_segment.push(*pt);
                                } else {
                                    if current_segment.len() >= 2 {
                                        clipped.push(InfillPath::Line(
                                            crate::geometry::Polyline::from_points(current_segment),
                                        ));
                                    }
                                    current_segment = Vec::new();
                                }
                            }
                            if current_segment.len() >= 2 {
                                clipped.push(InfillPath::Line(
                                    crate::geometry::Polyline::from_points(current_segment),
                                ));
                            }
                        }
                        clipped
                    }
                    _ => {
                        let polylines = generate_infill(
                            fill_pattern,
                            &[expoly.clone()],
                            infill_config.line_spacing,
                            infill_config.angle,
                        )?;
                        polylines.into_iter().map(InfillPath::Line).collect()
                    }
                };

                if generated.is_empty() {
                    continue;
                }

                let mut polylines = Vec::new();
                for path in generated {
                    match path {
                        InfillPath::Line(pl) => polylines.push(pl),
                        InfillPath::Loop(poly) => polylines.push(poly.split_at_first_point()),
                    }
                }

                if polylines.is_empty() {
                    continue;
                }

                // C++: Fill::connect_infill() inside fill_surface joins lines.
                if infill_config.connect_infill {
                    let boundary = vec![boundary_contour.clone()];
                    let fill_params = crate::fill::FillParams::new();
                    let mut connected = Vec::new();
                    crate::fill::connect_infill(
                        polylines,
                        &boundary,
                        infill_config.line_spacing,
                        &fill_params,
                        &mut connected,
                    );
                    polylines = connected;
                }

                sparse_infill_polylines.extend(polylines);
            }
        }

        // Fill.cpp:874
        // C++: return sparse_infill_polylines;
        Ok(sparse_infill_polylines)
    }

    /// Generate ironing paths
    /// Layer.cpp:667-1250
    pub fn make_ironing(&mut self) -> Result<()> {
        // Layer.cpp:672-1245
        // Ironing parameters
        // Layer.cpp:678-695
        #[derive(Debug, Clone, PartialEq)]
        struct IroningParams {
            pattern: InfillPattern,
            extruder: u32,
            just_infill: bool,
            line_spacing: f64,
            height: f64,
            speed: f64,
            angle: f64,
            inset: f64,
            layerm_idx: usize,
        }

        impl IroningParams {
            fn new(
                pattern: InfillPattern,
                extruder: u32,
                just_infill: bool,
                line_spacing: f64,
                height: f64,
                speed: f64,
                angle: f64,
                inset: f64,
                layerm_idx: usize,
            ) -> Self {
                Self {
                    pattern,
                    extruder,
                    just_infill,
                    line_spacing,
                    height,
                    speed,
                    angle,
                    inset,
                    layerm_idx,
                }
            }
        }

        impl PartialOrd for IroningParams {
            fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
                Some(self.cmp(other))
            }
        }

        impl Eq for IroningParams {}

        impl Ord for IroningParams {
            // Layer.cpp:710-755
            fn cmp(&self, other: &Self) -> std::cmp::Ordering {
                use std::cmp::Ordering;

                // Compare extruder
                if self.extruder != other.extruder {
                    return self.extruder.cmp(&other.extruder);
                }

                // Compare pattern
                if self.pattern != other.pattern {
                    return (self.pattern as u32).cmp(&(other.pattern as u32));
                }

                // Compare other fields
                if self.just_infill != other.just_infill {
                    return self.just_infill.cmp(&other.just_infill);
                }

                // Compare floats carefully
                if (self.line_spacing - other.line_spacing).abs() > 1e-9 {
                    return if self.line_spacing < other.line_spacing {
                        Ordering::Less
                    } else {
                        Ordering::Greater
                    };
                }

                if (self.height - other.height).abs() > 1e-9 {
                    return if self.height < other.height {
                        Ordering::Less
                    } else {
                        Ordering::Greater
                    };
                }

                if (self.speed - other.speed).abs() > 1e-9 {
                    return if self.speed < other.speed {
                        Ordering::Less
                    } else {
                        Ordering::Greater
                    };
                }

                if (self.angle - other.angle).abs() > 1e-9 {
                    return if self.angle < other.angle {
                        Ordering::Less
                    } else {
                        Ordering::Greater
                    };
                }

                if (self.inset - other.inset).abs() > 1e-9 {
                    return if self.inset < other.inset {
                        Ordering::Less
                    } else {
                        Ordering::Greater
                    };
                }

                self.layerm_idx.cmp(&other.layerm_idx)
            }
        }

        // TODO: Implement full ironing generation
        // Layer.cpp:757-1240
        Ok(())
    }

    /// Generate solid infill for a surface (helper)
    /// Layer.cpp:1252-1260
    fn generate_solid_infill(&mut self, _region_idx: usize, _surface: &Surface) -> Result<()> {
        // This is now handled by make_fills()
        Ok(())
    }

    /// Generate sparse infill for a surface (helper)
    /// Layer.cpp:1262-1270
    fn generate_sparse_infill(&mut self, _region_idx: usize, _surface: &Surface) -> Result<()> {
        // This is now handled by make_fills()
        Ok(())
    }

    // Additional helper methods

    /// Create new layer (floating point variant)
    /// Layer.cpp:1272-1285
    pub fn new_f(
        id: usize,
        object_id: usize,
        height_mm: f64,
        print_z_mm: f64,
        slice_z_mm: f64,
    ) -> Self {
        Self::new(id, object_id, height_mm, print_z_mm, slice_z_mm)
    }

    /// Get bottom Z in millimeters
    /// Layer.cpp:1287-1292
    pub fn bottom_z_mm(&self) -> f64 {
        self.bottom_z()
    }

    /// Get height in millimeters
    /// Layer.cpp:1294-1299
    pub fn height_mm(&self) -> f64 {
        self.height
    }

    /// Set lower layer reference
    /// Layer.cpp:1301-1306
    pub fn set_lower_layer(&mut self, lower_layer_id: Option<usize>) {
        self.lower_layer_id = lower_layer_id;
    }

    /// Set upper layer reference
    /// Layer.cpp:1308-1313
    pub fn set_upper_layer(&mut self, upper_layer_id: Option<usize>) {
        self.upper_layer_id = upper_layer_id;
    }

    /// Get all slices (combined)
    /// Layer.cpp:1315-1325
    pub fn all_slices(&self) -> ExPolygons {
        let result = self.lslices.clone();
        result
    }
}

impl LayerRegion {
    /// Set slices for this region
    /// Layer.cpp:1327-1332
    pub fn set_slices(&mut self, slices: SurfaceCollection) {
        self.slices = slices;
    }
}

// NOTE: the canonical PrintObject lives in crate::print_object; a dead local
// placeholder struct (empty bounding_box stub) was removed here.

/// Chain points by finding nearest neighbors
/// Layer.cpp:1334-1385
fn chain_points(points: &[Point]) -> Vec<usize> {
    if points.is_empty() {
        return Vec::new();
    }

    let mut result = Vec::with_capacity(points.len());
    let mut used = vec![false; points.len()];

    // Start with first point
    result.push(0);
    used[0] = true;

    // Chain remaining points
    for _ in 1..points.len() {
        let last_idx = *result.last().unwrap();
        let last_pt = &points[last_idx];

        // Find nearest unused point
        let mut nearest_idx = 0;
        let mut nearest_dist = f64::MAX;

        for (i, pt) in points.iter().enumerate() {
            if !used[i] {
                let dx = (pt.x - last_pt.x) as f64;
                let dy = (pt.y - last_pt.y) as f64;
                let dist = dx * dx + dy * dy;

                if dist < nearest_dist {
                    nearest_dist = dist;
                    nearest_idx = i;
                }
            }
        }

        result.push(nearest_idx);
        used[nearest_idx] = true;
    }

    result
}

// Layer.cpp:635
// C++: BoundingBox get_extents(const LayerRegion &layer_region)
pub fn get_extents_layer_region(layer_region: &LayerRegion) -> BoundingBox {
    use crate::geometry::get_extents_expoly;
    // Layer.cpp:637
    let mut bbox = BoundingBox::empty();
    // Layer.cpp:638-642
    // C++: if (!layer_region.slices.surfaces.empty()) {
    // C++:     bbox = get_extents(layer_region.slices.surfaces.front());
    // C++:     for (auto it = layer_region.slices.surfaces.cbegin() + 1; it != layer_region.slices.surfaces.cend(); ++it)
    // C++:         bbox.merge(get_extents(*it));
    // C++: }
    if !layer_region.slices.surfaces.is_empty() {
        // get_extents(const Surface&) returns the extents of surface.expolygon.
        bbox = get_extents_expoly(&layer_region.slices.surfaces[0].expolygon);
        for surface in &layer_region.slices.surfaces[1..] {
            bbox.merge(&get_extents_expoly(&surface.expolygon));
        }
    }
    // Layer.cpp:643
    bbox
}

// Layer.cpp:646
// C++: BoundingBox get_extents(const LayerRegionPtrs &layer_regions)
pub fn get_extents_layer_regions(layer_regions: &[LayerRegion]) -> BoundingBox {
    // Layer.cpp:648
    let mut bbox = BoundingBox::empty();
    // Layer.cpp:649-654
    // C++: if (!layer_regions.empty()) {
    // C++:     bbox = get_extents(*layer_regions.front());
    // C++:     for (auto it = layer_regions.begin() + 1; it != layer_regions.end(); ++it)
    // C++:         bbox.merge(get_extents(**it));
    // C++: }
    if !layer_regions.is_empty() {
        bbox = get_extents_layer_region(&layer_regions[0]);
        for layer_region in &layer_regions[1..] {
            bbox.merge(&get_extents_layer_region(layer_region));
        }
    }
    // Layer.cpp:655
    bbox
}
