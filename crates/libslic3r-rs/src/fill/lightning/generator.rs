//Copyright (c) 2021 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.
//
//! Lightning infill generator.
//!
//! C++ Reference:
//! - Fill/Lightning/Generator.hpp
//! - Fill/Lightning/Generator.cpp
//!
//! Faithful 1:1 port of `Slic3r::FillLightning::Generator`.
//!
//! The generator pre-computes the per-layer overhang areas and orchestrates the
//! layer-by-layer construction of lightning infill trees. It processes layers
//! from top to bottom, growing trees from overhang points toward the build
//! plate and propagating them downward.
//!
//! PORTING STATUS (partial): the constructor for the supports (`Generator::new_for_support`,
//! Generator.cpp:100-143) and `generate_trees_for_support` (Generator.cpp:238-278) are ported over the
//! PrintObject of `crate::print::faithful` and compared with the pin through the tree support
//! (tests/tree_support_native_parity.rs, the "lightning_trees" lines). The constructor for the infill of an
//! object and `generate_trees` are still the commented references below: they read the layers of the
//! old-world `crate::print_object::PrintObject`, and `generate_initial_internal_overhangs` runs on the `geo`
//! clipper (see its FIDELITY-NOTE).

// Generator.cpp:4-11
//   #include "Generator.hpp"
//   #include "TreeNode.hpp"
//   #include "../../ClipperUtils.hpp"
//   #include "../../Layer.hpp"
//   #include "../../Print.hpp"
//   #include "ExPolygon.hpp"
use super::layer::Layer;
use crate::bounding_box::BoundingBox;
use crate::clipper_utils::{diff_polygons, offset_polygons, OffsetJoinType};
use crate::geometry::{to_polygons as expolygons_to_polygons, Polygon};
use crate::print_object::PrintObject;
use crate::surface::SurfaceType;
use crate::Coord;

/* Generator.cpp:13-25 — Possible future tasks/optimizations,etc.:
 * - Improve connecting heuristic to favor connecting to shorter trees
 * - Change which node of a tree is the root when that would be better in reconnectRoots.
 * - (For implementation in Infill classes & elsewhere): Outline offset, infill-overlap & perimeter gaps.
 * - Allow for polylines, i.e. merge Tims PR about polyline fixes
 * - Unit Tests?
 * - Optimization: let the square grid store the closest point on boundary
 * - Optimization: only compute the closest dist to / point on boundary for the outer cells and flood-fill the rest
 * - Make a pass with Arachne over the output. Somehow.
 * - Generate all to-be-supported points at once instead of sequentially: See branch interlocking_gen PolygonUtils::spreadDots (Or work with sparse grids.)
 * - Lots of magic values ... to many to parameterize. But are they the best?
 * - Move more complex computations from Generator constructor to elsewhere.
 */

// Generator.cpp:29-60 — `get_svg_filename` and `draw_two_overhangs_to_svg` are
// debugging-only SVG helpers (guarded by commented-out call sites in the C++).
// They are intentionally NOT ported: they pull in `Slic3r::SVG`, `srand`/`time`,
// and filesystem writes, none of which participate in G-code parity and the SVG
// backend is not wasm-safe.

// Generator.cpp:64 — namespace Slic3r::FillLightning

/// Generates the Lightning Infill pattern.
///
/// Generator.hpp:37 — `class Generator  // "Just like Nicola used to make!"`
///
/// The lightning infill pattern is designed to use a minimal amount of material
/// to support the top skin of the print, while still printing with reasonably
/// consistently flowing lines. It sacrifices strength completely in favour of
/// top surface quality and reduced print time / material usage.
#[derive(Debug, Clone, Default)]
pub struct Generator {
    /// Generator.hpp:83 — `float m_infill_extrusion_width;`
    pub m_infill_extrusion_width: f32,

    /// How far each piece of infill can support skin in the layer above.
    /// Generator.hpp:88 — `coord_t m_supporting_radius;`
    pub m_supporting_radius: Coord,

    /// How far a wall can support the wall above it. If a wall completely
    /// supports the wall above it, no infill needs to support that.
    ///
    /// This is similar to the overhang distance calculated for support. It is
    /// determined by the lightning_infill_overhang_angle setting.
    /// Generator.hpp:97 — `coord_t m_wall_supporting_radius;`
    pub m_wall_supporting_radius: Coord,

    /// How far each piece of infill can support other infill in the layer above.
    ///
    /// This may be different than `supporting_radius`, because the infill is
    /// printed with one end floating in mid-air. This endpoint will sag more, so
    /// an infill line may need to be supported more than a skin line.
    /// Generator.hpp:106 — `coord_t m_prune_length;`
    pub m_prune_length: Coord,

    /// How far a line may be shifted in order to straighten the line out.
    ///
    /// Straightening the line reduces material and time usage and reduces
    /// accelerations needed to print the pattern. However it makes the infill
    /// weak if lines are partially suspended next to the line on the previous
    /// layer.
    /// Generator.hpp:116 — `coord_t m_straightening_max_distance;`
    pub m_straightening_max_distance: Coord,

    /// For each layer, the overhang that needs to be supported by the pattern.
    /// This is generated by `generate_initial_internal_overhangs`.
    /// Generator.hpp:123 — `std::vector<Polygons> m_overhang_per_layer;`
    pub m_overhang_per_layer: Vec<Vec<Polygon>>,

    /// For each layer, the generated lightning paths.
    /// This is generated by `generate_trees`.
    /// Generator.hpp:130 — `std::vector<Layer> m_lightning_layers;`
    pub m_lightning_layers: Vec<Layer>,

    /// Generator.hpp:132 — `std::vector<BoundingBox> bboxs;`
    pub bboxs: Vec<BoundingBox>,
}

impl Generator {
    // Generator.cpp:66-92 — `Generator::Generator(const PrintObject &print_object,
    //   const std::function<void()> &throw_on_cancel_callback)`
    //
    // BLOCKED: this primary constructor cannot be ported faithfully yet.
    //   * Generator.cpp:90-91 `generateInitialInternalOverhangs` /
    //     `generateTrees` — the config reads and `generateInitialInternalOverhangs`
    //     are unblocked (config hierarchy is wired; `nozzle_diameter` scalar is
    //     equivalent to max-element over a single-nozzle vector; the C++
    //     `region_config.sparse_infill_density` (ConfigOptionPercent, e.g. 15)
    //     maps to the Rust `PrintRegionConfig::fill_density` (a 0..1 fraction, so
    //     `0.15`)), but `generateTrees` is still blocked on
    //     `Layer::generateNewTrees` / `reconnectRoots` (Fill/Lightning/Layer.cpp
    //     not ported). Until those land the whole constructor remains commented out.
    //
    // Faithful reference (mapping C++ `region_config` accessor through the wired
    // Rust hierarchy: `print_object.shared_regions()->all_regions.front()->config()`
    // == `print_object.all_regions()[0].config()`):
    //
    //     let print_config         = print_object.print().config();
    //     let object_config        = print_object.config();
    //     let region_config        = print_object.all_regions()[0].config();
    //     // C++ `nozzle_diameter.values` is a vector; Rust models a single nozzle
    //     // as a scalar, so `*max_element(values)` == that scalar.
    //     let max_nozzle_diameter  = print_config.nozzle_diameter;
    //     // const int infill_extruder = region_config.infill_extruder.value; (commented in C++)
    //     let default_infill_extrusion_width =
    //         Flow::auto_extrusion_width(FlowRole::Infill, max_nozzle_diameter);
    //     // Note: There's not going to be a layer below the first one, so the
    //     // 'initial layer height' doesn't have to be taken into account.
    //     let layer_thickness = crate::scaled(object_config.layer_height) as f64;
    //
    //     // m_infill_extrusion_width = scaled<float>(region_config.infill_extrusion_width.percent ? ... : ...);
    //     // m_supporting_radius = coord_t(m_infill_extrusion_width) * 100 / region_config.sparse_infill_density;
    //     self.m_infill_extrusion_width = crate::scaled(region_config.sparse_infill_line_width) as f32;
    //     // C++ divides by the integer percent (e.g. 15). Rust `fill_density` is the
    //     // equivalent fraction (0.15), so `* 100 / percent` == `/ fill_density`.
    //     self.m_supporting_radius =
    //         ((self.m_infill_extrusion_width as f64) / region_config.fill_density) as Coord;
    //
    //     let lightning_infill_overhang_angle      = std::f64::consts::PI / 4.0; // 45 degrees
    //     let lightning_infill_prune_angle         = std::f64::consts::PI / 4.0; // 45 degrees
    //     let lightning_infill_straightening_angle = std::f64::consts::PI / 4.0; // 45 degrees
    //     self.m_wall_supporting_radius     = (layer_thickness * lightning_infill_overhang_angle.tan()) as Coord;
    //     self.m_prune_length               = (layer_thickness * lightning_infill_prune_angle.tan()) as Coord;
    //     self.m_straightening_max_distance = (layer_thickness * lightning_infill_straightening_angle.tan()) as Coord;
    //
    //     self.generate_initial_internal_overhangs(print_object, throw_on_cancel_callback);
    //     self.generate_trees(print_object, throw_on_cancel_callback);

    // Generator.cpp:94-131 — `Generator::Generator(PrintObject* m_object,
    //   std::vector<Polygons>& contours, std::vector<Polygons>& overhangs,
    //   const std::function<void()> &throw_on_cancel_callback, float density)`
    //
    // BLOCKED for the same reason as the primary constructor: config reads are
    // unblocked (config hierarchy wired), but `generateTreesforSupport` calls
    // `Layer::generateNewTrees` / `reconnectRoots` (Fill/Lightning/Layer.cpp not
    // ported). See `generate_trees_for_support` below.
    //
    // Faithful reference (note the divergent magic-value formulas vs. the
    // primary ctor):
    //
    //     ... (same config reads as above) ...
    //     self.m_infill_extrusion_width = crate::scaled(region_config.sparse_infill_line_width) as f32;
    //     // m_supporting_radius: against to the density of lightning, failures may
    //     // happen if set to high density. higher density lightning makes support
    //     // harder, more time-consuming on computing and printing, but more reliable
    //     // on supporting overhangs; lower density lightning performs opposite.
    //     // TODO: decide whether enable density controller in advanced options or not
    //     let density = density.max(0.15f32);
    //     // C++ `coord_t(m_infill_extrusion_width) / density` is integer-by-float
    //     // division promoted to float, then truncated back to coord_t. Casting
    //     // `density` to Coord first would truncate 0.15 -> 0 (div-by-zero), so the
    //     // division MUST stay in float: `(coord / density as f32) as Coord`.
    //     self.m_supporting_radius =
    //         ((self.m_infill_extrusion_width as Coord) as f32 / density) as Coord;
    //
    //     let lightning_infill_overhang_angle      = std::f64::consts::PI / 4.0; // 45 degrees
    //     let lightning_infill_prune_angle         = std::f64::consts::PI / 4.0; // 45 degrees
    //     let lightning_infill_straightening_angle = std::f64::consts::PI / 4.0; // 45 degrees
    //     self.m_wall_supporting_radius     = (layer_thickness * lightning_infill_overhang_angle.tan()) as Coord;
    //     self.m_prune_length               = (layer_thickness * lightning_infill_prune_angle.tan()) as Coord;
    //     self.m_straightening_max_distance = (layer_thickness * lightning_infill_straightening_angle.tan()) as Coord;
    //
    //     self.m_overhang_per_layer = overhangs;
    //     self.generate_trees_for_support(contours, throw_on_cancel_callback);

    /// Generator.cpp:133-153 — `void Generator::generateInitialInternalOverhangs(
    ///   const PrintObject &print_object, const std::function<void()> &throw_on_cancel_callback)`
    ///
    /// Calculate the overhangs above the infill areas that need to be supported
    /// by infill. Normally, overhangs are only generated for the outside of the
    /// model and only when support is generated. For this pattern, we also need
    /// to generate overhang areas for the inside of the model.
    pub fn generate_initial_internal_overhangs(
        &mut self,
        print_object: &PrintObject,
        throw_on_cancel_callback: &dyn Fn(),
    ) {
        // Generator.cpp:135
        self.m_overhang_per_layer
            .resize(print_object.layers().len(), Vec::new());

        // Generator.cpp:137
        let mut infill_area_above: Vec<Polygon> = Vec::new();
        // Generator.cpp:138-139 — Iterate from top to bottom, to subtract the
        // overhang areas above from the overhang areas on the layer below, to get
        // only overhang in the top layer where it is overhanging.
        for layer_nr in (0..print_object.layers().len() as i64).rev() {
            // Generator.cpp:140
            throw_on_cancel_callback();
            // Generator.cpp:141
            let mut infill_area_here: Vec<Polygon> = Vec::new();
            // Generator.cpp:142-145
            for layerm in print_object.layers()[layer_nr as usize].regions() {
                for surface in &layerm.fill_surfaces.surfaces {
                    if surface.surface_type == SurfaceType::Internal
                        || surface.surface_type == SurfaceType::InternalVoid
                    {
                        append(&mut infill_area_here, surface.expolygon.to_polygons());
                    }
                }
            }

            // Generator.cpp:147-148 — Remove the part of the infill area that is
            // already supported by the walls.
            //   Polygons overhang = diff(offset(infill_area_here, -float(m_wall_supporting_radius)), infill_area_above);
            // `offset(Polygons, delta)` and `diff(Polygons, Polygons)` are Clipper
            // wrappers that take/return `Polygons`. `offset_polygons` mirrors
            // `offset(...)` and `diff_polygons` mirrors `diff(Polygons, Polygons)`.
            //
            // FIDELITY-NOTE(F1): geo-clipper approximation vs C++ ClipperLib.
            // `offset_polygons` runs through the `geo` crate at the fixed
            // GEO_CLIPPER_SCALE (1000) and expects its `delta` in unscaled mm,
            // whereas C++ `offset` works directly at coord_t (scaled) precision.
            // The delta is therefore unscaled here so the *physical* offset
            // distance matches C++ (`-float(m_wall_supporting_radius)` coord_t ==
            // `-unscale(m_wall_supporting_radius)` mm); the resulting geometry is
            // a ClipperLib approximation, not bit-identical.
            let offset_here: Vec<Polygon> = expolygons_to_polygons(&offset_polygons(
                &infill_area_here,
                -crate::unscale(self.m_wall_supporting_radius),
                OffsetJoinType::Miter,
            ));
            let overhang: Vec<Polygon> = diff_polygons(&offset_here, &infill_area_above);

            // Generator.cpp:150
            self.m_overhang_per_layer[layer_nr as usize] = overhang;
            // Generator.cpp:151
            infill_area_above = std::mem::take(&mut infill_area_here);
        }
    }

    /// Generator.cpp:155-159 — `const Layer& Generator::getTreesForLayer(
    ///   const size_t& layer_id) const`
    ///
    /// Get a tree of paths generated for a certain layer of the mesh. This tree
    /// represents the paths that must be traced to print the infill.
    pub fn get_trees_for_layer(&self, layer_id: usize) -> &Layer {
        // Generator.cpp:157 — assert(layer_id < m_lightning_layers.size());
        debug_assert!(layer_id < self.m_lightning_layers.len());
        // Generator.cpp:158
        &self.m_lightning_layers[layer_id]
    }

    // Generator.cpp:161-215 — `void Generator::generateTrees(const PrintObject
    //   &print_object, const std::function<void()> &throw_on_cancel_callback)`
    //
    // BLOCKED. The sole blocker is:
    //   * Generator.cpp:193 `current_lightning_layer.generateNewTrees(...)` and
    //     Generator.cpp:194 `reconnectRoots(...)` — `lightning::layer::Layer`
    //     does not implement these (Fill/Lightning/Layer.cpp not ported).
    //
    // All other call targets are resolved:
    //   - `EdgeGrid::create_from_polygons`, `EdgeGrid::set_bbox`, and
    //     `BoundingBox::inflated`/`.defined` all exist in Rust.
    //   - `geometry::BoundingBox` and `bounding_box::BoundingBox` are the same type.
    //   - `LOCATOR_CELL_SIZE` (`scaled<coord_t>(4.)`) is `tree_node::LOCATOR_CELL_SIZE`.
    //   - `get_extents(tree_roots)` is `tree_node::get_extents_roots(...)`.
    //   - `tree->propagateToNextLayer(...)` is `tree_node::propagate_to_next_layer(...)`.
    //
    // Faithful reference:
    //
    //     self.m_lightning_layers.resize(print_object.layers().len(), Layer::default());
    //     self.bboxs.resize(print_object.layers().len(), BoundingBox::default());
    //     let mut infill_outlines: Vec<Vec<Polygon>> = vec![Vec::new(); print_object.layers().len()];
    //
    //     // For-each layer from top to bottom:
    //     for layer_id in (0..print_object.layers().len() as i64).rev() {
    //         throw_on_cancel_callback();
    //         for layerm in print_object.layers()[layer_id as usize].regions() {
    //             for surface in &layerm.fill_surfaces.surfaces {
    //                 if surface.surface_type == SurfaceType::Internal
    //                     || surface.surface_type == SurfaceType::InternalVoid {
    //                     append(&mut infill_outlines[layer_id as usize], surface.expolygon.to_polygons());
    //                 }
    //             }
    //         }
    //     }
    //
    //     // For various operations its beneficial to quickly locate nearby features on the polygon:
    //     // EdgeGrid::Grid outlines_locator(bbox) — set bbox then create.
    //     let top_layer_id = print_object.layers().len() - 1;
    //     let mut outlines_locator = EdgeGrid::new();
    //     outlines_locator.set_bbox(
    //         crate::geometry::polygon::get_extents_polygons(&infill_outlines[top_layer_id])
    //             .inflated(crate::SCALED_EPSILON));
    //     outlines_locator.create_from_polygons(
    //         &infill_outlines[top_layer_id], super::tree_node::LOCATOR_CELL_SIZE);
    //
    //     // For-each layer from top to bottom:
    //     for layer_id in (0..=top_layer_id as i64).rev() {
    //         throw_on_cancel_callback();
    //         let current_outlines      = &infill_outlines[layer_id as usize];
    //         let current_outlines_bbox =
    //             crate::geometry::polygon::get_extents_polygons(current_outlines);
    //         self.bboxs[layer_id as usize] = current_outlines_bbox;
    //
    //         // register all trees propagated from the previous layer as to-be-reconnected
    //         let to_be_reconnected_tree_roots =
    //             self.m_lightning_layers[layer_id as usize].tree_roots.clone();
    //
    //         // BLOCKED: Layer::generateNewTrees / reconnectRoots not ported.
    //         // self.m_lightning_layers[layer_id as usize].generate_new_trees(
    //         //     &self.m_overhang_per_layer[layer_id as usize], current_outlines,
    //         //     &current_outlines_bbox, &outlines_locator,
    //         //     self.m_supporting_radius, self.m_wall_supporting_radius,
    //         //     throw_on_cancel_callback);
    //         // self.m_lightning_layers[layer_id as usize].reconnect_roots(
    //         //     &to_be_reconnected_tree_roots, current_outlines,
    //         //     &current_outlines_bbox, &outlines_locator,
    //         //     self.m_supporting_radius, self.m_wall_supporting_radius);
    //
    //         // Initialize trees for next lower layer from the current one.
    //         if layer_id == 0 { return; }
    //
    //         let below_outlines = &infill_outlines[(layer_id - 1) as usize];
    //         let mut below_outlines_bbox =
    //             crate::geometry::polygon::get_extents_polygons(below_outlines)
    //                 .inflated(crate::SCALED_EPSILON);
    //         if outlines_locator.bbox().defined {
    //             below_outlines_bbox.merge_bb(outlines_locator.bbox());
    //         }
    //         if !self.m_lightning_layers[layer_id as usize].tree_roots.is_empty() {
    //             below_outlines_bbox.merge_bb(
    //                 &super::tree_node::get_extents_roots(
    //                     &self.m_lightning_layers[layer_id as usize].tree_roots)
    //                     .inflated(crate::SCALED_EPSILON));
    //         }
    //         outlines_locator.set_bbox(below_outlines_bbox);
    //         outlines_locator.create_from_polygons(
    //             below_outlines, super::tree_node::LOCATOR_CELL_SIZE);
    //
    //         for tree in &self.m_lightning_layers[layer_id as usize].tree_roots {
    //             super::tree_node::propagate_to_next_layer(
    //                 tree,
    //                 &mut self.m_lightning_layers[(layer_id - 1) as usize].tree_roots,
    //                 below_outlines, &outlines_locator, self.m_prune_length,
    //                 self.m_straightening_max_distance,
    //                 super::tree_node::LOCATOR_CELL_SIZE / 2);
    //         }
    //     }

    // Generator.cpp:217-263 — `void Generator::generateTreesforSupport(
    //   std::vector<Polygons>& contours, const std::function<void()> &throw_on_cancel_callback)`
    //
    // BLOCKED for the same sole reason as `generate_trees`:
    //   * `Layer::generateNewTrees` / `reconnectRoots` not ported in
    //     `lightning::layer::Layer` (Fill/Lightning/Layer.cpp).
    // Its body is `generateTrees` minus the surface-extraction prelude (it takes
    // pre-computed `contours` directly):
    //
    //     if contours.is_empty() { return; }
    //     self.m_lightning_layers.resize(contours.len(), Layer::default());
    //     self.bboxs.resize(contours.len(), BoundingBox::default());
    //     ... (identical locator + per-layer loop as generate_trees, using
    //          `contours[layer_id]` in place of `infill_outlines[layer_id]`) ...
}

impl Generator {
    /// Generator.cpp:100-143 — `Generator::Generator(PrintObject* m_object, std::vector<Polygons>& contours,
    ///   std::vector<Polygons>& overhangs, const std::function<void()> &throw_on_cancel_callback, float density)`,
    /// the constructor the tree support makes the generator of its lightning infill with (TreeSupport.cpp
    /// draw_circles()), over the PrintObject of `crate::print::faithful`.
    pub fn new_for_support(
        m_object: &crate::print::faithful::PrintObject,
        print: &crate::print::faithful::Print,
        contours: &mut Vec<Vec<Polygon>>,
        overhangs: &mut Vec<Vec<Polygon>>,
        throw_on_cancel_callback: &dyn Fn(),
        density: f32,
    ) -> Self {
        use crate::config::{ConfigOptionFloat, ConfigOptionFloats, ConfigOptionFloatsNullable, ConfigOptionResolver};
        let mut this = Generator::default();
        // Generator.cpp:102-104
        let print_config = print.config();
        let object_config = m_object.config();
        let shared_regions = m_object.m_shared_regions.as_ref().expect("PrintObject::shared_regions").borrow();
        let region = shared_regions.all_regions.first().expect("all_regions.front()").borrow();
        let region_config = region.config();
        let cfg_float = |config: &crate::print_config::config_def::DynamicPrintConfig, key: &str| -> f64 {
            config.option(key).and_then(|o| o.get_float().ok()).unwrap_or(0.0)
        };
        // Generator.cpp:105-106
        let nozzle_diameters: Vec<f64> = match print_config.option("nozzle_diameter") {
            Some(opt) => {
                if let Some(o) = opt.as_any().downcast_ref::<ConfigOptionFloats>() {
                    o.values.clone()
                } else if let Some(o) = opt.as_any().downcast_ref::<ConfigOptionFloatsNullable>() {
                    o.values.clone()
                } else {
                    Vec::new()
                }
            }
            None => Vec::new(),
        };
        // `*std::max_element(begin, end)`: the first of the largest values
        let mut max_nozzle_diameter: f64 = nozzle_diameters[0];
        for &d in &nozzle_diameters[1..] {
            if max_nozzle_diameter < d {
                max_nozzle_diameter = d;
            }
        }
        // Generator.cpp:108: `float Flow::auto_extrusion_width(FlowRole, float)` (Flow.cpp:22-37), `1.125f *
        // nozzle_diameter` of floats for the infill
        let default_infill_extrusion_width: f64 = (1.125f32 * (max_nozzle_diameter as f32)) as f64;
        // Note: There's not going to be a layer below the first one, so the 'initial layer height' doesn't have to be taken into account.
        // Generator.cpp:110: `scaled<double>(v)` is `v / SCALING_FACTOR` (Point.hpp:527-530)
        let layer_thickness: f64 = crate::scaled_f(cfg_float(object_config, "layer_height"));

        // Generator.cpp:112-116: `scaled<float>(double)` is `float(v / SCALING_FACTOR)`
        let sparse_infill_filament = region_config.option("sparse_infill_filament").and_then(|o| o.get_int().ok()).unwrap_or(0);
        this.m_infill_extrusion_width = crate::scaled_f(crate::flow::Flow::extrusion_width_for_nozzle(
            &ConfigOptionFloat::with_value(cfg_float(region_config, "sparse_infill_line_width")),
            crate::print_config::config_def::primary_nozzle_diameter(print.full_print_config()) as f32,
            crate::print::faithful::nozzle_diameter_for_filament(print_config, sparse_infill_filament as u32),
            crate::print_config::config_def::multi_nozzle_diameter_mode_enabled(print.full_print_config()),
        )) as f32;
        // sparse_infill_line_width may be 0 (auto) in a support-only context; fall back to the
        // nozzle-derived default so the supporting radius (and thus the DistanceField cell size)
        // never collapses to 0, which would make sample_grid_pattern loop forever.
        // Generator.cpp:120-121: `SCALED_EPSILON` is `scale_(1e-4)`, a double the float is compared with as a float
        if this.m_infill_extrusion_width < (crate::scaled_f(1e-4) as f32) {
            this.m_infill_extrusion_width = crate::scaled_f(default_infill_extrusion_width) as f32;
        }
        //m_supporting_radius: against to the density of lightning, failures may happen if set to high density
        //higher density lightning makes support harder, more time-consuming on computing and printing, but more reliable on supporting overhangs
        //lower density lightning performs opposite
        //TODO: decide whether enable density controller in advanced options or not
        // Generator.cpp:126-127: a float quotient cut to a coord_t (an int at the pin); `scaled<coord_t>(0.05)`
        // is `coord_t(0.05 / SCALING_FACTOR)`
        let density = 0.15f32.max(density);
        this.m_supporting_radius = (((this.m_infill_extrusion_width / density) as i32) as Coord).max(crate::scaled_f(0.05) as i32 as Coord);

        // Generator.cpp:129-134: doubles assigned to coord_t members
        let lightning_infill_overhang_angle: f64 = std::f64::consts::PI / 4.0; // 45 degrees
        let lightning_infill_prune_angle: f64 = std::f64::consts::PI / 4.0; // 45 degrees
        let lightning_infill_straightening_angle: f64 = std::f64::consts::PI / 4.0; // 45 degrees
        this.m_wall_supporting_radius = (layer_thickness * lightning_infill_overhang_angle.tan()) as i32 as Coord;
        this.m_prune_length = (layer_thickness * lightning_infill_prune_angle.tan()) as i32 as Coord;
        this.m_straightening_max_distance = (layer_thickness * lightning_infill_straightening_angle.tan()) as i32 as Coord;

        // Generator.cpp:136
        this.m_overhang_per_layer = overhangs.clone();

        // Generator.cpp:138
        this.generate_trees_for_support(contours, throw_on_cancel_callback);
        this
    }

    /// Generator.cpp:238-278 — `void Generator::generateTreesforSupport(std::vector<Polygons>& contours,
    ///   const std::function<void()> &throw_on_cancel_callback)`
    pub fn generate_trees_for_support(&mut self, contours: &mut Vec<Vec<Polygon>>, throw_on_cancel_callback: &dyn Fn()) {
        use super::tree_node::LOCATOR_CELL_SIZE;
        use crate::edge_grid::EdgeGrid;
        use crate::geometry::polygon::get_extents_polygons;
        // The bounding boxes of the locator are those of `crate::geometry` (what EdgeGrid and the tree nodes take).
        // `bbox.inflated(SCALED_EPSILON)` (BoundingBox.hpp:48, BoundingBox.cpp `offset(coordf_t delta)`: `PointClass
        // v(delta, delta)`, a Point made of doubles, lrint): `scale_(1e-4)` rounds to 10.
        let scaled_epsilon: Coord = crate::scaled_f(1e-4).round_ties_even() as Coord;
        type GeoBoundingBox = crate::geometry::BoundingBox;
        // Generator.cpp:240
        if contours.is_empty() {
            return;
        }

        // Generator.cpp:242-243
        self.m_lightning_layers.resize(contours.len(), Layer::default());
        self.bboxs.resize(contours.len(), BoundingBox::default());

        // For various operations its beneficial to quickly locate nearby features on the polygon:
        // Generator.cpp:246-248
        let top_layer_id: usize = contours.len() - 1;
        let mut outlines_locator = EdgeGrid::new();
        outlines_locator.set_bbox(get_extents_polygons(&contours[top_layer_id]).expanded(scaled_epsilon));
        outlines_locator.create_from_polygons(&contours[top_layer_id], LOCATOR_CELL_SIZE);

        // For-each layer from top to bottom:
        // Generator.cpp:251-277
        let callback = || throw_on_cancel_callback();
        for layer_id in (0..=top_layer_id).rev() {
            throw_on_cancel_callback();
            let current_outlines: &Vec<Polygon> = &contours[layer_id];
            let current_outlines_bbox: GeoBoundingBox = get_extents_polygons(current_outlines);

            self.bboxs[layer_id] = if current_outlines_bbox.is_defined() {
                BoundingBox { min: current_outlines_bbox.min, max: current_outlines_bbox.max, defined: true }
            } else {
                BoundingBox::default()
            };

            // register all trees propagated from the previous layer as to-be-reconnected
            let to_be_reconnected_tree_roots: Vec<super::tree_node::NodeSPtr> = self.m_lightning_layers[layer_id].tree_roots.clone();

            self.m_lightning_layers[layer_id].generate_new_trees(
                &self.m_overhang_per_layer[layer_id],
                current_outlines,
                &current_outlines_bbox,
                &outlines_locator,
                self.m_supporting_radius,
                self.m_wall_supporting_radius,
                &callback,
            );
            self.m_lightning_layers[layer_id].reconnect_roots(
                &to_be_reconnected_tree_roots,
                current_outlines,
                &current_outlines_bbox,
                &outlines_locator,
                self.m_supporting_radius,
                self.m_wall_supporting_radius,
            );

            // Initialize trees for next lower layer from the current one.
            if layer_id == 0 {
                return;
            }

            let below_outlines: &Vec<Polygon> = &contours[layer_id - 1];
            let mut below_outlines_bbox: GeoBoundingBox = get_extents_polygons(below_outlines).expanded(scaled_epsilon);
            let outlines_locator_bbox = *outlines_locator.bbox();
            if outlines_locator_bbox.is_defined() {
                below_outlines_bbox.merge(&outlines_locator_bbox);
            }

            if !self.m_lightning_layers[layer_id].tree_roots.is_empty() {
                below_outlines_bbox.merge(&super::tree_node::get_extents_roots(&self.m_lightning_layers[layer_id].tree_roots).expanded(scaled_epsilon));
            }

            outlines_locator.set_bbox(below_outlines_bbox);
            outlines_locator.create_from_polygons(below_outlines, LOCATOR_CELL_SIZE);

            let (lower, upper) = self.m_lightning_layers.split_at_mut(layer_id);
            let lower_trees: &mut Vec<super::tree_node::NodeSPtr> = &mut lower[layer_id - 1].tree_roots;
            for tree in &upper[0].tree_roots {
                super::tree_node::propagate_to_next_layer(
                    tree,
                    lower_trees,
                    below_outlines,
                    &outlines_locator,
                    self.m_prune_length,
                    self.m_straightening_max_distance,
                    LOCATOR_CELL_SIZE / 2,
                );
            }
        }
    }
}

// Generator.cpp:265 — } // namespace Slic3r::FillLightning

/// `append(dst, src)` — Slic3r helper (libslic3r.h) that moves all elements of
/// `src` onto the end of `dst`. Used at Generator.cpp:145.
#[inline]
fn append(dst: &mut Vec<Polygon>, mut src: Vec<Polygon>) {
    if dst.is_empty() {
        *dst = src;
    } else {
        dst.append(&mut src);
    }
}
