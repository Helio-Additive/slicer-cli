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
use crate::fill::fill_rectilinear::generate_fill_rectilinear;
use crate::fill::{generate_infill, InfillConfig, InfillPath, InfillPattern};
use crate::flow::{Flow, FlowRole};
use crate::geometry::{
    BoundingBox, ExPolygon, ExPolygons, Point, Polygon, Polygons, Polyline, Polylines,
};
use crate::perimeter_generator::{PerimeterConfig, PerimeterGenerator, WallGeneratorMode};
use crate::region_config::PrintRegionConfig;
use crate::region_expansion::{
    expand_bridges_detect_orientations, expand_merge_surfaces, ExpansionZone,
    RegionExpansionParameters,
};
use crate::surface::{Surface, SurfaceCollection, SurfaceType};
use crate::{scale, unscale, Coord, CoordF, Result};

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
            // This is fed to ExPolygon::simplify_p() which, in this crate, scales the
            // tolerance internally (geometry::douglas_peucker), so it expects the value in
            // **mm**. Read from the print config (C++: print_config->resolution).
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
            layer_id: layer_id,
            raft_layers: 0,
            overhang_flow: None,
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

        // Gap fill — faithful port of PerimeterGenerator.cpp:1327-1374.
        // The previous version traced each gap polygon's CONTOUR at full perimeter
        // width, which over-extruded ~4.6x (contour ~2x length, full width vs thin).
        // C++ instead: collapse the gaps to the truly-thin band, run medial_axis to get
        // variable-width centerlines, and emit those via variable_width().
        if !result.gap_fills.is_empty() {
            use crate::clipper_utils::{difference, offset2, opening_ex, OffsetJoinType};
            use crate::extrusion_entity::{ExtrusionEntityType, ExtrusionRole};
            use crate::perimeter_generator::convert_thin_walls_to_extrusion_paths;

            let perimeter_width = perimeter_flow.width();
            let perimeter_spacing = perimeter_flow.spacing();
            // PerimeterGenerator.cpp:1329-1330 (INSET_OVERLAP_TOLERANCE = 0.45)
            let min = 0.2 * perimeter_width * (1.0 - 0.45);
            let max = 2.0 * perimeter_spacing;
            // ClipperSafetyOffset = 10 scaled units = 1e-5 mm
            const CLIPPER_SAFETY_OFFSET: f64 = 0.00001;

            // PerimeterGenerator.cpp:1331-1334 — keep only the band wider than `min`
            // (opening by min/2) and narrower than `max` (subtract the max-opening).
            let opened_min = opening_ex(&result.gap_fills, min / 2.0);
            let wide_part = offset2(
                &result.gap_fills,
                max / 2.0,
                max / 2.0 + CLIPPER_SAFETY_OFFSET,
                OffsetJoinType::Square,
            );
            let gaps_ex = difference(&opened_min, &wide_part);

            // PerimeterGenerator.cpp:1335-1340 — medial axis of each thin gap region.
            let mut polylines: crate::geometry::ThickPolylines = Vec::new();
            for ex in &gaps_ex {
                ex.medial_axis(min, max, &mut polylines);
            }

            // PerimeterGenerator.cpp:1357-1360 — filter tiny gap fills
            // (config.filter_out_gap_fill default 0.0 in this profile -> no extra filter).

            // PerimeterGenerator.cpp:1364 — variable_width(polylines, erGapFill, solid_infill_flow)
            if !polylines.is_empty() {
                let gap_fill_flow = self
                    .flow(FlowRole::SolidInfill, layer_height)
                    .unwrap_or_else(|_| perimeter_flow.clone());
                let paths = convert_thin_walls_to_extrusion_paths(
                    &polylines,
                    ExtrusionRole::GapFill,
                    &gap_fill_flow,
                );
                for path in paths {
                    self.thin_fills
                        .entities
                        .push(ExtrusionEntityType::Path(path));
                }
            }
        }

        Ok(())
    }

    /// Process external surfaces
    /// LayerRegion.cpp:518-640
    ///
    /// The full wave-expansion port of this LayerRegion member lives as the free
    /// function `crate::surface::process_external_surfaces` (re-exported from
    /// `crate::layer_region`), and is driven from `print_object.rs`. This thin
    /// member shim forwards to it so the `LayerRegion`-method spelling still works.
    pub fn process_external_surfaces(
        &mut self,
        expansion_distance: f64,
        min_area_mm2: f64,
    ) -> Result<()> {
        // LayerRegion.cpp:518 — operate on this region's fill_surfaces in place.
        let mut surfaces = vec![std::mem::take(&mut self.fill_surfaces.surfaces)];
        crate::surface::process_external_surfaces(&mut surfaces, expansion_distance, min_area_mm2);
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
    // BLOCKED comparisons (fields not yet in the Rust PrintRegionConfig):
    //   override_filament_scarf_seam_setting, seam_slope_type, seam_slope_start_height,
    //   seam_slope_gap, seam_slope_min_length, seam_slope_conditional, seam_slope_entire_loop,
    //   seam_slope_steps, seam_slope_inner_walls (Layer.cpp:186-195)
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
        // Layer.cpp:187-195: scarf/seam-slope comparisons blocked (see note above)
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

    pub fn make_perimeters(
        &mut self,
        _perimeter_generator_options: Option<&()>,
    ) -> Result<()> {
        // Layer.cpp:204-206
        // C++: std::vector<unsigned char> done(m_regions.size(), false);
        let mut done = vec![false; self.regions.len()];

        // Layer.cpp:208-209
        // C++: for (LayerRegionPtrs::iterator layerm = m_regions.begin(); layerm != m_regions.end(); ++ layerm)
        for region_idx in 0..self.regions.len() {
            if done[region_idx] {
                continue;
            }

            // Layer.cpp:210-212
            // C++: if ((*layerm)->slices.empty()) {
            //          (*layerm)->perimeters.clear();
            //          (*layerm)->fills.clear();
            //          (*layerm)->thin_fills.clear();
            if self.regions[region_idx].slices.surfaces.is_empty() {
                self.regions[region_idx].perimeters.entities.clear();
                self.regions[region_idx].fills.entities.clear();
                self.regions[region_idx].thin_fills.entities.clear();
                done[region_idx] = true;
                continue;
            }

            // Layer.cpp:217-218
            done[region_idx] = true;

            // Layer.cpp:218
            // C++: const PrintRegionConfig &config = (*layerm)->region().config();
            // C++ only uses it to find compatible regions to merge
            // (Layer.cpp:221-245); the single-region Rust path skips merging,
            // and LayerRegion::make_perimeters reads its own stored
            // Arc<PrintRegion> directly, so no config is threaded from here.

            // Layer.cpp:248-251
            // For now, single region optimization (skip multi-region merging)
            // C++: if (layerms.size() == 1) {
            //          (*layerm)->fill_surfaces.surfaces.clear();
            //          (*layerm)->fill_no_overlap_expolygons.clear();
            let surface_fill = self.regions[region_idx].slices.clone();
            self.regions[region_idx].fill_surfaces.surfaces.clear();
            self.regions[region_idx].fill_no_overlap_expolygons.clear();

            // Layer.cpp:252
            // C++: (*layerm)->make_perimeters((*layerm)->slices, perimeter_regions, &(*layerm)->fill_surfaces, &(*layerm)->fill_no_overlap_expolygons, this->loop_nodes);
            // print_z is threaded explicitly because the Rust LayerRegion has no
            // Layer back-pointer (C++ reads this->layer()->print_z, LayerRegion.cpp:148).
            // Reads of self's plain fields are hoisted to locals before the
            // &mut borrow of self.regions[region_idx] below.
            let print_z = self.print_z;
            let height = self.height;
            let id = self.id;
            self.regions[region_idx].make_perimeters(&surface_fill, height, id, print_z)?;

            // Layer.cpp:254
            // C++: (*layerm)->fill_expolygons = to_expolygons((*layerm)->fill_surfaces.surfaces);
            // Already done in make_perimeters
        }

        Ok(())
    }

    /// Create slices from regions
    /// Layer.cpp:492-540
    pub fn make_slices(&mut self) {
        // Layer.cpp:497-510
        self.lslices.clear();

        // Collect all region slices
        // Layer.cpp:512-530
        for region in &self.regions {
            for surface in &region.slices.surfaces {
                self.lslices.push(surface.expolygon.clone());
            }
        }

        // Union overlapping slices
        // Layer.cpp:532-538
        self.lslices = union_ex(&self.lslices);
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

    /// Backup untyped slices
    /// Layer.cpp:550-580
    pub fn backup_untyped_slices(&mut self) {
        // Layer.cpp:555-575
        for region in &mut self.regions {
            region.raw_slices = region.slices.clone();
        }
    }

    /// Restore untyped slices
    /// Layer.cpp:582-615
    pub fn restore_untyped_slices(&mut self) {
        // Layer.cpp:590-610
        for region in &mut self.regions {
            region.slices = region.raw_slices.clone();
        }
    }

    /// Restore untyped slices without extra perimeters
    /// Layer.cpp:617-635
    pub fn restore_untyped_slices_no_extra_perimeters(&mut self) {
        // Layer.cpp:622-632
        for region in &mut self.regions {
            region.slices = region.raw_slices.clone();
        }
    }

    /// Get merged slices
    /// Layer.cpp:637-645
    pub fn merged(&self, _offset: f64) -> Polygons {
        // TODO: Implement
        Polygons::new()
    }

    /// Apply auto circle compensation
    /// Layer.cpp:647-655
    pub fn apply_auto_circle_compensation(&mut self) {
        // TODO: Implement
    }

    /// Calculate perimeter continuity
    /// Layer.cpp:657-665
    pub fn calculate_perimeter_continuity(&mut self) {
        // TODO: Implement
    }

    /// Generate fill patterns for all regions
    /// Fill.cpp:586-768
    pub fn make_fills(&mut self) -> Result<()> {
        // Fill.cpp:600
        // C++: const auto resolution = this->object()->print()->config().resolution.value;
        // Read up front through the layer's own print-config Arc (stamped by
        // wire_config_hierarchy) so no shared borrow overlaps the region
        // mutation below. Still underscore-bound: the downstream consumer
        // (params.resolution, Fill.cpp:705, used for path simplification) is
        // not ported yet.
        let _resolution = self.object().print().config().resolution;

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
        let surface_fills = crate::fill::group_fills(self, &mut lock_param)?;

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
                // C++: Fill::connect_infill() - called after fill generation
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

                if !collection.entities.is_empty() {
                    self.regions[region_id].fills.entities.push(
                        crate::extrusion_entity::ExtrusionEntityType::Collection(Box::new(
                            collection,
                        )),
                    );
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
        let mut result = self.lslices.clone();
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
