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
//! PORTING STATUS (partial): the tractable surface — the class data layout, the
//! constructor magic-value math, `get_trees_for_layer`, and
//! `generate_initial_internal_overhangs` — is ported faithfully against existing
//! crate primitives. The two `Generator(...)` constructors and the
//! `generate_trees` / `generate_trees_for_support` orchestration loops are
//! BLOCKED because they delegate all real work to symbols that are not yet
//! ported / not yet threaded through the Rust pipeline:
//!   * `PrintObject::print()->config()` — the Rust `PrintObject` has no `Print`
//!     back-reference and `PrintConfig::nozzle_diameter` is a scalar, not the
//!     `std::vector<double>` the C++ `*std::max_element(...)` consumes.
//!   * `PrintRegionConfig::sparse_infill_density` — not present as a config
//!     field in the Rust `PrintRegionConfig`.
//!   * `Layer::generateNewTrees`, `Layer::reconnectRoots` (Fill/Lightning/Layer.cpp)
//!     — the sibling `lightning::layer::Layer` is still a stub and lacks these.
//!   * `Node::propagateToNextLayer`, `get_extents(const NodeSPtr&)`,
//!     `locator_cell_size`, the `shared_ptr<Node>` (`NodeSPtr`) tree mutation
//!     semantics (Fill/Lightning/TreeNode.cpp) — `lightning::tree_node::Node` is
//!     a stub.
//! These are listed faithfully below where they are called.

// Generator.cpp:4-11
//   #include "Generator.hpp"
//   #include "TreeNode.hpp"
//   #include "../../ClipperUtils.hpp"
//   #include "../../Layer.hpp"
//   #include "../../Print.hpp"
//   #include "ExPolygon.hpp"
use super::layer::Layer;
use crate::bounding_box::BoundingBox;
use crate::clipper_utils::{difference, offset_polygons, OffsetJoinType};
use crate::geometry::{to_polygons as expolygons_to_polygons, ExPolygon, Polygon};
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
    //   * Generator.cpp:68 `print_object.print()->config()` — the Rust
    //     `PrintObject` carries no back-reference to its owning `Print`, so the
    //     `PrintConfig` is unreachable.
    //   * Generator.cpp:71-72 `print_config.nozzle_diameter.values` +
    //     `*std::max_element(...)` — Rust `PrintConfig::nozzle_diameter` is a
    //     single `CoordF`, not the per-extruder `std::vector<double>` the C++
    //     reduces over.
    //   * Generator.cpp:70/80-81 `region_config.sparse_infill_line_width` /
    //     `region_config.sparse_infill_density` — `sparse_infill_density` is not
    //     a field of the Rust `PrintRegionConfig`.
    //   * Generator.cpp:90-91 `generateInitialInternalOverhangs` /
    //     `generateTrees` — the latter is blocked (see `generate_trees`).
    //
    // Faithful reference for when the config thread + siblings land:
    //
    //     let print_config         = print_object.print().config();
    //     let object_config        = print_object.config();
    //     let region_config        = print_object.shared_regions().all_regions.front().config();
    //     let nozzle_diameters     = &print_config.nozzle_diameter.values;
    //     let max_nozzle_diameter  = *nozzle_diameters.iter().max();
    //     // const int infill_extruder = region_config.infill_extruder.value; (commented in C++)
    //     let default_infill_extrusion_width =
    //         Flow::auto_extrusion_width(FlowRole::Infill, max_nozzle_diameter as f32);
    //     // Note: There's not going to be a layer below the first one, so the
    //     // 'initial layer height' doesn't have to be taken into account.
    //     let layer_thickness = scaled(object_config.layer_height) as f64;
    //
    //     // m_infill_extrusion_width = scaled<float>(region_config.infill_extrusion_width.percent ? ... : ...);
    //     // m_supporting_radius = coord_t(m_infill_extrusion_width) * 100 / coord_t(region_config.fill_density.value);
    //     self.m_infill_extrusion_width = scaled(region_config.sparse_infill_line_width) as f32;
    //     self.m_supporting_radius =
    //         (self.m_infill_extrusion_width as Coord) * 100 / region_config.sparse_infill_density;
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
    // BLOCKED for the same reasons as the primary constructor (needs
    // `print_object.print()->config()`, the nozzle-diameter vector, and the
    // sparse-infill region config), plus `generateTreesforSupport` (see
    // `generate_trees_for_support`).
    //
    // Faithful reference (note the divergent magic-value formulas vs. the
    // primary ctor):
    //
    //     ... (same config reads as above) ...
    //     self.m_infill_extrusion_width = scaled(region_config.sparse_infill_line_width) as f32;
    //     // m_supporting_radius: against to the density of lightning, failures may
    //     // happen if set to high density. higher density lightning makes support
    //     // harder, more time-consuming on computing and printing, but more reliable
    //     // on supporting overhangs; lower density lightning performs opposite.
    //     // TODO: decide whether enable density controller in advanced options or not
    //     let density = density.max(0.15f32);
    //     self.m_supporting_radius = (self.m_infill_extrusion_width as Coord) / (density as Coord);
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
            // wrappers producing `Polygons`; here composed via the ExPolygon-based
            // crate primitives (`offset_polygons` -> `difference` -> `to_polygons`)
            // which perform the identical Clipper boolean.
            let offset_here: Vec<Polygon> = expolygons_to_polygons(&offset_polygons(
                &infill_area_here,
                -(self.m_wall_supporting_radius as f64),
                OffsetJoinType::Miter,
            ));
            let above_ex: Vec<ExPolygon> = crate::clipper_utils::union_polygons_ex(&infill_area_above);
            let offset_here_ex: Vec<ExPolygon> = crate::clipper_utils::union_polygons_ex(&offset_here);
            let overhang: Vec<Polygon> = expolygons_to_polygons(&difference(&offset_here_ex, &above_ex));

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
    // BLOCKED. The structure below is the faithful control flow, but every
    // load-bearing call targets a symbol that is not yet ported:
    //   * Generator.cpp:178-179 `EdgeGrid::Grid::create(polygons, locator_cell_size)`
    //     — Rust `EdgeGrid::create_from_polygons` exists, but `locator_cell_size`
    //     (`scaled<coord_t>(4.)`, TreeNode.hpp:21) and the
    //     `BoundingBox::inflated`/`.defined` API the C++ uses live on the
    //     `bounding_box::BoundingBox`, whereas `get_extents(Polygons)` returns the
    //     `geometry::BoundingBox` which lacks `inflated`. Reconciling the two
    //     BoundingBox types is a separate fix.
    //   * Generator.cpp:193 `current_lightning_layer.generateNewTrees(...)` and
    //     Generator.cpp:194 `reconnectRoots(...)` — `lightning::layer::Layer`
    //     does not implement these (Fill/Lightning/Layer.cpp not ported).
    //   * Generator.cpp:206/254 `get_extents(current_lightning_layer.tree_roots)`
    //     — `get_extents(const NodeSPtr&)` (TreeNode.hpp:286) not ported.
    //   * Generator.cpp:213 `tree->propagateToNextLayer(...)` — `Node` is a stub
    //     (Fill/Lightning/TreeNode.cpp not ported), and the whole tree model uses
    //     `shared_ptr<Node>` (`NodeSPtr`) aliasing the Rust port has not adopted.
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
    //     let top_layer_id = print_object.layers().len() - 1;
    //     let mut outlines_locator =
    //         EdgeGrid::new_with_bbox(get_extents(&infill_outlines[top_layer_id]).inflated(SCALED_EPSILON));
    //     outlines_locator.create(&infill_outlines[top_layer_id], locator_cell_size);
    //
    //     // For-each layer from top to bottom:
    //     for layer_id in (0..=top_layer_id as i64).rev() {
    //         throw_on_cancel_callback();
    //         let current_outlines      = &infill_outlines[layer_id as usize];
    //         let current_outlines_bbox = get_extents(current_outlines);
    //         self.bboxs[layer_id as usize] = get_extents(current_outlines);
    //
    //         // register all trees propagated from the previous layer as to-be-reconnected
    //         let to_be_reconnected_tree_roots = self.m_lightning_layers[layer_id as usize].tree_roots.clone();
    //
    //         self.m_lightning_layers[layer_id as usize].generate_new_trees(
    //             &self.m_overhang_per_layer[layer_id as usize], current_outlines, &current_outlines_bbox,
    //             &outlines_locator, self.m_supporting_radius, self.m_wall_supporting_radius, throw_on_cancel_callback);
    //         self.m_lightning_layers[layer_id as usize].reconnect_roots(
    //             &to_be_reconnected_tree_roots, current_outlines, &current_outlines_bbox,
    //             &outlines_locator, self.m_supporting_radius, self.m_wall_supporting_radius);
    //
    //         // Initialize trees for next lower layer from the current one.
    //         if layer_id == 0 { return; }
    //
    //         let below_outlines = &infill_outlines[(layer_id - 1) as usize];
    //         let mut below_outlines_bbox = get_extents(below_outlines).inflated(SCALED_EPSILON);
    //         if outlines_locator.bbox().defined { below_outlines_bbox.merge(outlines_locator.bbox()); }
    //         if !self.m_lightning_layers[layer_id as usize].tree_roots.is_empty() {
    //             below_outlines_bbox.merge(&get_extents(&self.m_lightning_layers[layer_id as usize].tree_roots).inflated(SCALED_EPSILON));
    //         }
    //         outlines_locator.set_bbox(below_outlines_bbox);
    //         outlines_locator.create(below_outlines, locator_cell_size);
    //
    //         for tree in &self.m_lightning_layers[layer_id as usize].tree_roots {
    //             tree.propagate_to_next_layer(
    //                 &mut self.m_lightning_layers[(layer_id - 1) as usize].tree_roots,
    //                 below_outlines, &outlines_locator, self.m_prune_length,
    //                 self.m_straightening_max_distance, locator_cell_size / 2);
    //         }
    //     }

    // Generator.cpp:217-263 — `void Generator::generateTreesforSupport(
    //   std::vector<Polygons>& contours, const std::function<void()> &throw_on_cancel_callback)`
    //
    // BLOCKED for the same set of reasons as `generate_trees`
    // (`Layer::generateNewTrees`/`reconnectRoots`, `Node::propagateToNextLayer`,
    // `get_extents(NodeSPtr)`, `locator_cell_size`, the two-BoundingBox split).
    // Its body is `generateTrees` minus the surface-extraction prelude (it takes
    // pre-computed `contours` directly):
    //
    //     if contours.is_empty() { return; }
    //     self.m_lightning_layers.resize(contours.len(), Layer::default());
    //     self.bboxs.resize(contours.len(), BoundingBox::default());
    //     ... (identical locator + per-layer loop as generate_trees, using
    //          `contours[layer_id]` in place of `infill_outlines[layer_id]`) ...
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
