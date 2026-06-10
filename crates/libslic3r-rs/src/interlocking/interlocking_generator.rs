//! Interlocking structure generator.
//!
//! Faithful 1:1 port of BambuStudio's
//! `src/libslic3r/Interlocking/InterlockingGenerator.{cpp,hpp}`.
//!
//! InterlockingGenerator.hpp:12-41
//! Class for generating an interlocking structure between two adjacent models
//! of a different extruder.
//!
//! The structure consists of horizontal beams of the two materials interlaced.
//! In the z direction the direction of these beams is alternated with 90*.
//!
//! Example with two materials # and O
//! Even beams:      Odd beams:
//! ######           ##OO##OO
//! OOOOOO           ##OO##OO
//! ######           ##OO##OO
//! OOOOOO           ##OO##OO
//!
//! We set up a voxel grid of (2*beam_w,2*beam_w,2*beam_h) and mark all the
//! voxels which contain both meshes. We then remove all voxels which also
//! contain air, so that the interlocking pattern will not be visible from the
//! outside. We then generate and combine the polygons for each voxel and apply
//! those areas to the outlines of the meshes.
//!
//! Porting notes (divergences forced by the current Rust crate state):
//! * C++ holds `PrintObject& print_object` as a member and mutates it from
//!   `const` methods (the reference target is not const-qualified). Rust
//!   stores `&mut PrintObject` and the mutating methods take `&mut self`.
//! * The C++ `PrintRegion::flow(const PrintObject&, ...)` reaches the print
//!   level config (`nozzle_diameter`) through `object.print()->config()`.
//!   The Rust `PrintObject` has no back-reference to `Print`, so the
//!   per-extruder nozzle diameters are threaded in explicitly
//!   (`nozzle_diameters` parameter / field).
//! * The Rust clipper layer (`crate::clipper_utils`) takes offsets in mm
//!   (`CoordF`), so the scaled `coord_t` distances of the C++ are `unscale`d
//!   at the call sites. The integer arithmetic that produces those distances
//!   is kept in `Coord` (i64) exactly as in C++.
//! * C++ `Polygons` used in `handleThinAreas` (`near_interlock_per_layer`)
//!   are represented as `ExPolygons` because the crate's boolean ops operate
//!   on `ExPolygon`s; the stored cell squares are hole-free so the semantics
//!   are identical.
//! * `tbb::parallel_for` in `generate_embedding_wall` is executed as a
//!   sequential loop (each layer's work is independent, so the result is
//!   identical).

use std::cell::RefCell;
use std::collections::HashSet;

use super::voxel_utils::{DilationKernel, DilationKernelType, GridPoint3, VoxelUtils};
use crate::clipper_utils::{
    closing, difference, intersection, offset_expolygons, opening_ex, union, union_ex, xor,
    OffsetJoinType,
};
use crate::flow::FlowRole;
use crate::geometry::{
    deg2rad, expolygons_append, expolygons_rotate, ExPolygon, ExPolygons, Point, Polygon,
};
use crate::libslic3r::EPSILON;
use crate::print_object::PrintObject;
use crate::surface::{to_expolygons, SurfaceType};
use crate::{scaled, unscale, Coord, CoordF};

/// Hash for GridPoint3.
///
/// InterlockingGenerator.cpp:10-23
/// C++: template<> struct std::hash<Slic3r::GridPoint3>
///
/// In C++ this specialization is required so that `std::unordered_set<GridPoint3>`
/// compiles; it only influences the (unspecified) iteration order of the set.
/// The Rust `HashSet<GridPoint3>` uses its default hasher, which is the
/// equivalent "unordered" behaviour. The function is kept for reference.
#[allow(dead_code)]
fn grid_point_hash(pp: &GridPoint3) -> usize {
    // InterlockingGenerator.cpp:15
    let prime: i32 = 31;
    // InterlockingGenerator.cpp:16
    let mut result: i32 = 89;
    // InterlockingGenerator.cpp:17
    result = result.wrapping_mul(prime).wrapping_add(pp[0] as i32);
    // InterlockingGenerator.cpp:18
    result = result.wrapping_mul(prime).wrapping_add(pp[1] as i32);
    // InterlockingGenerator.cpp:19
    result = result.wrapping_mul(prime).wrapping_add(pp[2] as i32);
    // InterlockingGenerator.cpp:20
    result as usize
}

/// Mimic C++ `std::unordered_set::merge(source)`: elements of `source` that
/// are NOT yet present in `target` are moved into `target`; elements already
/// present in `target` REMAIN in `source`. After the call `target` holds the
/// union and `source` holds the intersection — this is the
/// "perform union and intersection simultaneously" trick of
/// InterlockingGenerator.cpp:200 / 227.
fn unordered_set_merge(target: &mut HashSet<GridPoint3>, source: &mut HashSet<GridPoint3>) {
    let mut kept: HashSet<GridPoint3> = HashSet::new();
    for p in source.drain() {
        // `insert` returns false when the value was already present.
        if !target.insert(p) {
            kept.insert(p);
        }
    }
    *source = kept;
}

/// Class for generating an interlocking structure between two adjacent models
/// of a different extruder.
///
/// InterlockingGenerator.hpp:42-170
pub struct InterlockingGenerator<'a> {
    /// InterlockingGenerator.hpp:154 `PrintObject& print_object;`
    print_object: &'a mut PrintObject,
    /// InterlockingGenerator.hpp:155 `const size_t region_a_index;`
    region_a_index: usize,
    /// InterlockingGenerator.hpp:156 `const size_t region_b_index;`
    region_b_index: usize,
    /// InterlockingGenerator.hpp:157 `const coord_t beam_width;`
    beam_width: Coord,
    /// InterlockingGenerator.hpp:158 `const coord_t boundary_avoidance;`
    boundary_avoidance: Coord,
    /// InterlockingGenerator.hpp:160 `const VoxelUtils vu;`
    vu: VoxelUtils,
    /// InterlockingGenerator.hpp:162 `const float rotation;`
    rotation: f32,
    /// InterlockingGenerator.hpp:163 `const Vec3crd cell_size;`
    cell_size: [Coord; 3],
    /// InterlockingGenerator.hpp:164 `const coord_t beam_layer_count;`
    beam_layer_count: Coord,
    /// InterlockingGenerator.hpp:165 `const DilationKernel interface_dilation;`
    interface_dilation: DilationKernel,
    /// InterlockingGenerator.hpp:166 `const DilationKernel air_dilation;`
    air_dilation: DilationKernel,
    // Whether to fully remove all of the interlocking cells which would be visible on the outside. If no air filtering then those cells
    // will be cut off midway in a beam.
    /// InterlockingGenerator.hpp:169 `const bool air_filtering;`
    air_filtering: bool,
    /// Rust-only: per-extruder nozzle diameters (mm). C++ `PrintRegion::flow`
    /// reads these via `object.print()->config().nozzle_diameter`; the Rust
    /// `PrintObject` has no back-reference to the print, so they are threaded
    /// in by the caller (empty slice is fine for the embedding-wall path,
    /// which never computes a flow).
    nozzle_diameters: &'a [CoordF],
}

impl<'a> InterlockingGenerator<'a> {
    /// Distance between models to be considered next to each other so that an
    /// interlocking structure will be generated there.
    ///
    /// InterlockingGenerator.hpp:152 `static const coord_t ignored_gap_ = 100u;`
    const IGNORED_GAP: Coord = 100;

    /// Generate an interlocking structure between each two adjacent meshes.
    ///
    /// InterlockingGenerator.cpp:27
    /// C++: void InterlockingGenerator::generate_embedding_wall(PrintObject* print_object)
    pub fn generate_embedding_wall(print_object: &mut PrintObject) {
        // params
        // InterlockingGenerator.cpp:29-32
        let interface_depth: i32 = 2;
        let boundary_avoidance: i32 = 2;
        // C++: constexpr coord_t DEFAULT_BEAM_WIDTH = scaled(0.2); // 例如默认2mm
        let default_beam_width: Coord = scaled(0.2);
        let beam_width: Coord = default_beam_width;

        // InterlockingGenerator.cpp:34
        let interface_dilation = DilationKernel::new(
            [
                interface_depth as Coord,
                interface_depth as Coord,
                interface_depth as Coord,
            ],
            DilationKernelType::Prism,
        );
        // InterlockingGenerator.cpp:35
        let air_filtering: bool = boundary_avoidance > 0;
        // InterlockingGenerator.cpp:36
        let air_dilation = DilationKernel::new(
            [
                boundary_avoidance as Coord,
                boundary_avoidance as Coord,
                boundary_avoidance as Coord,
            ],
            DilationKernelType::Prism,
        );

        // InterlockingGenerator.cpp:38-39
        let cell_width: Coord = beam_width + beam_width;
        let cell_size: [Coord; 3] = [cell_width, cell_width, 2];

        // generator
        // InterlockingGenerator.cpp:42-44
        // C++: tbb::parallel_for over the layer range. Each layer's work is
        // independent; executed sequentially here (identical result).
        for i in 0..print_object.layers.len() {
            // InterlockingGenerator.cpp:46-48
            if print_object.layers[i].id() % 2 == 0 {
                continue;
            }
            // InterlockingGenerator.cpp:49
            let region_count = print_object.layers[i].region_count();
            for region_a_index in 0..region_count {
                // InterlockingGenerator.cpp:50
                // C++: const PrintRegionConfig& config = layer->get_region(region_a_index)->region().config();
                let config = {
                    let region_id = print_object.layers[i]
                        .get_region(region_a_index)
                        .expect("layer region out of range")
                        .region_id();
                    print_object
                        .printing_region(region_id)
                        .expect("printing region out of range")
                        .config()
                        .clone()
                };

                // InterlockingGenerator.cpp:52
                for region_b_index in (region_a_index + 1)..region_count {
                    // InterlockingGenerator.cpp:53
                    let config_other = {
                        let region_id = print_object.layers[i]
                            .get_region(region_b_index)
                            .expect("layer region out of range")
                            .region_id();
                        print_object
                            .printing_region(region_id)
                            .expect("printing region out of range")
                            .config()
                            .clone()
                    };
                    // wall to infill
                    // InterlockingGenerator.cpp:55-59
                    // (Rust PrintRegionConfig names `wall_loops` as `perimeters`.)
                    if !(config_other.embedding_wall_into_infill || config.embedding_wall_into_infill)
                        || print_object.layers[i].has_compatible_layer_regions(&config, &config_other)
                        || (config.perimeters == 0 && config_other.perimeters == 0)
                        || (config.perimeters > 0 && config_other.perimeters > 0)
                    {
                        continue;
                    }
                    // has embedding part
                    // InterlockingGenerator.cpp:61-62
                    let mut gen = InterlockingGenerator::new(
                        &mut *print_object,
                        region_a_index,
                        region_b_index,
                        beam_width,
                        boundary_avoidance as Coord,
                        0.0,
                        cell_size,
                        1,
                        interface_dilation.clone(),
                        air_dilation.clone(),
                        air_filtering,
                        // The embedding-wall path never computes a flow.
                        &[],
                    );
                    // InterlockingGenerator.cpp:63
                    gen.generate_interlockingwall(i);
                }
            }
        }
    }

    /// Generate an interlocking structure between each two adjacent meshes.
    ///
    /// InterlockingGenerator.cpp:70
    /// C++: void InterlockingGenerator::generate_interlocking_structure(PrintObject* print_object)
    ///
    /// `nozzle_diameters` is the per-extruder nozzle diameter list of the
    /// print config (see the struct-level porting notes).
    pub fn generate_interlocking_structure(
        print_object: &mut PrintObject,
        nozzle_diameters: &[CoordF],
    ) -> crate::Result<()> {
        // InterlockingGenerator.cpp:72
        let config = &print_object.config;
        // Check if interlocking is enabled, and avoid errors like division by zero due to invalid configuration.
        // InterlockingGenerator.cpp:74-76
        if !config.interlocking_beam
            || config.interlocking_beam_layer_count < 1
            || config.interlocking_depth < 1
            || config.interlocking_beam_width < EPSILON
        {
            return Ok(());
        }

        // InterlockingGenerator.cpp:78-82
        let rotation: f32 = deg2rad(config.interlocking_orientation) as f32;
        let beam_layer_count: Coord = config.interlocking_beam_layer_count as Coord;
        let interface_depth: i32 = config.interlocking_depth;
        let boundary_avoidance: i32 = config.interlocking_boundary_avoidance;
        let beam_width: Coord = scaled(config.interlocking_beam_width);

        // Zero width would cause divide-by-zero in VoxelUtils (cell_size used as divisor). Treat as disabled.
        // InterlockingGenerator.cpp:84-86
        if beam_width <= 0 {
            return Ok(());
        }

        // InterlockingGenerator.cpp:88
        let interface_dilation = DilationKernel::new(
            [
                interface_depth as Coord,
                interface_depth as Coord,
                interface_depth as Coord,
            ],
            DilationKernelType::Prism,
        );

        // InterlockingGenerator.cpp:90-91
        let air_filtering: bool = boundary_avoidance > 0;
        let air_dilation = DilationKernel::new(
            [
                boundary_avoidance as Coord,
                boundary_avoidance as Coord,
                boundary_avoidance as Coord,
            ],
            DilationKernelType::Prism,
        );

        // InterlockingGenerator.cpp:93-94
        let cell_width: Coord = beam_width + beam_width;
        let cell_size: [Coord; 3] = [cell_width, cell_width, 2 * beam_layer_count];

        // InterlockingGenerator.cpp:96-98
        for region_a_index in 0..print_object.num_printing_regions() {
            let extruder_nr_a = print_object
                .printing_region(region_a_index)
                .expect("printing region out of range")
                .extruder(FlowRole::ExternalPerimeter)
                .map_err(crate::Error::Config)?;

            // InterlockingGenerator.cpp:100-105
            for region_b_index in (region_a_index + 1)..print_object.num_printing_regions() {
                let extruder_nr_b = print_object
                    .printing_region(region_b_index)
                    .expect("printing region out of range")
                    .extruder(FlowRole::ExternalPerimeter)
                    .map_err(crate::Error::Config)?;
                if extruder_nr_a == extruder_nr_b {
                    continue;
                }

                // InterlockingGenerator.cpp:107-108
                let mut gen = InterlockingGenerator::new(
                    &mut *print_object,
                    region_a_index,
                    region_b_index,
                    beam_width,
                    boundary_avoidance as Coord,
                    rotation,
                    cell_size,
                    beam_layer_count,
                    interface_dilation.clone(),
                    air_dilation.clone(),
                    air_filtering,
                    nozzle_diameters,
                );
                // InterlockingGenerator.cpp:109
                gen.generate_interlocking_structure_impl()?;
            }
        }
        Ok(())
    }

    /// Private constructor for storing some variables used in the computation
    /// of the interlocking structure between two meshes.
    ///
    /// InterlockingGenerator.hpp:68-91
    #[allow(clippy::too_many_arguments)]
    fn new(
        print_object: &'a mut PrintObject,
        region_a_index: usize,
        region_b_index: usize,
        beam_width: Coord,
        boundary_avoidance: Coord,
        rotation: f32,
        cell_size: [Coord; 3],
        beam_layer_count: Coord,
        interface_dilation: DilationKernel,
        air_dilation: DilationKernel,
        air_filtering: bool,
        nozzle_diameters: &'a [CoordF],
    ) -> Self {
        // InterlockingGenerator.hpp:79-91 (member initializer list)
        Self {
            print_object,
            region_a_index,
            region_b_index,
            beam_width,
            boundary_avoidance,
            vu: VoxelUtils::new(cell_size),
            rotation,
            cell_size,
            beam_layer_count,
            interface_dilation,
            air_dilation,
            air_filtering,
            nozzle_diameters,
        }
    }

    /// Compute the scaled external-perimeter flow width of a printing region.
    ///
    /// This is the repeated C++ sub-expression
    /// `print_object.printing_region(idx).flow(print_object, frExternalPerimeter, 0.1).scaled_width()`
    /// of InterlockingGenerator.cpp:117-118 and 146-147.
    fn printing_region_flow_scaled_width(&self, region_index: usize) -> crate::Result<Coord> {
        Ok(self
            .print_object
            .printing_region(region_index)
            .expect("printing region out of range")
            .flow(
                FlowRole::ExternalPerimeter,
                0.1,
                // C++ default argument `first_layer = false`
                false,
                // initial_layer_line_width: only read when first_layer == true
                // (PrintRegion.cpp:27), so the value is irrelevant here.
                0.0,
                // C++ PrintRegion::flow falls back to object.config().line_width
                self.print_object.config.line_width,
                self.nozzle_diameters,
            )
            .map_err(crate::Error::Config)?
            .scaled_width())
    }

    /// Given two polygons, return the parts that border on air, and grow
    /// 'perpendicular' up to 'detect' distance.
    ///
    /// InterlockingGenerator.cpp:114
    /// C++: std::pair<ExPolygons, ExPolygons> InterlockingGenerator::growBorderAreasPerpendicular(
    ///          const ExPolygons& a, const ExPolygons& b, const coord_t& detect) const
    fn grow_border_areas_perpendicular(
        &self,
        a: &ExPolygons,
        b: &ExPolygons,
        detect: Coord,
    ) -> crate::Result<(ExPolygons, ExPolygons)> {
        // InterlockingGenerator.cpp:116-118
        let min_line: Coord = std::cmp::min(
            self.printing_region_flow_scaled_width(self.region_a_index)?,
            self.printing_region_flow_scaled_width(self.region_b_index)?,
        );

        // InterlockingGenerator.cpp:120
        let total_shrunk = offset_expolygons(
            &union(
                &offset_expolygons(a, unscale(min_line), OffsetJoinType::Miter),
                &offset_expolygons(b, unscale(min_line), OffsetJoinType::Miter),
            ),
            unscale(2 * -min_line),
            OffsetJoinType::Miter,
        );

        // InterlockingGenerator.cpp:122-123
        let mut from_border_a = difference(a, &total_shrunk);
        let mut from_border_b = difference(b, &total_shrunk);

        // InterlockingGenerator.cpp:125-131
        for _i in 0..((detect / min_line) + 2) {
            let temp_a = offset_expolygons(&from_border_a, unscale(min_line), OffsetJoinType::Miter);
            let temp_b = offset_expolygons(&from_border_b, unscale(min_line), OffsetJoinType::Miter);
            from_border_a = difference(&temp_a, &temp_b);
            from_border_b = difference(&temp_b, &temp_a);
        }

        // InterlockingGenerator.cpp:133
        Ok((from_border_a, from_border_b))
    }

    /// Special handling for thin strips of material.
    ///
    /// Expand the meshes into each other where they need it, namely when a thin
    /// strip of material needs to be attached.
    ///
    /// InterlockingGenerator.cpp:136
    /// C++: void InterlockingGenerator::handleThinAreas(const std::unordered_set<GridPoint3>& has_all_meshes) const
    fn handle_thin_areas(&mut self, has_all_meshes: &HashSet<GridPoint3>) -> crate::Result<()> {
        // InterlockingGenerator.cpp:138-140
        let number_of_beams_detect: Coord = self.boundary_avoidance;
        let number_of_beams_expand: Coord = self.boundary_avoidance - 1;
        // C++: constexpr coord_t rounding_errors = 5;
        const ROUNDING_ERRORS: Coord = 5;

        // InterlockingGenerator.cpp:142-144
        let max_beam_width: Coord = self.beam_width;
        let detect: Coord = (max_beam_width * number_of_beams_detect) + ROUNDING_ERRORS;
        let expand: Coord = (max_beam_width * number_of_beams_expand) + ROUNDING_ERRORS;
        // InterlockingGenerator.cpp:145-147
        let close_gaps: Coord = std::cmp::min(
            self.printing_region_flow_scaled_width(self.region_a_index)?,
            self.printing_region_flow_scaled_width(self.region_b_index)?,
        ) / 4;

        // Make an inclusionary polygon, to only actually handle thin areas near actual microstructures (so not in skin for example).
        // InterlockingGenerator.cpp:150-151
        let mut near_interlock_per_layer: Vec<ExPolygons> =
            vec![ExPolygons::new(); self.print_object.layers.len()];
        // InterlockingGenerator.cpp:152-158
        for cell in has_all_meshes {
            let bottom_corner = self.vu.to_lower_corner(*cell);
            let mut layer_nr: Coord = bottom_corner[2];
            while layer_nr < bottom_corner[2] + self.cell_size[2]
                && layer_nr < near_interlock_per_layer.len() as Coord
            {
                // C++ pushes the cell's Polygon into a Polygons vector; the
                // squares are hole-free so wrapping in ExPolygon is identical.
                near_interlock_per_layer[layer_nr as usize]
                    .push(ExPolygon::new(self.vu.to_polygon(*cell)));
                layer_nr += 1;
            }
        }
        // InterlockingGenerator.cpp:159-162
        for near_interlock in &mut near_interlock_per_layer {
            // C++: near_interlock = offset(union_(closing(near_interlock, rounding_errors)), detect);
            *near_interlock = offset_expolygons(
                &union_ex(&closing(
                    near_interlock,
                    unscale(ROUNDING_ERRORS),
                    OffsetJoinType::Miter,
                )),
                unscale(detect),
                OffsetJoinType::Miter,
            );
            // C++: polygons_rotate(near_interlock, rotation);
            expolygons_rotate(near_interlock, self.rotation as CoordF);
        }

        // Only alter layers when they are present in both meshes, zip should take care if that.
        // InterlockingGenerator.cpp:165
        for layer_nr in 0..self.print_object.layers.len() {
            // InterlockingGenerator.cpp:166-168
            let polys_a = to_expolygons(
                &self.print_object.layers[layer_nr]
                    .get_region(self.region_a_index)
                    .expect("layer region out of range")
                    .slices
                    .surfaces,
            );
            let polys_b = to_expolygons(
                &self.print_object.layers[layer_nr]
                    .get_region(self.region_b_index)
                    .expect("layer region out of range")
                    .slices
                    .surfaces,
            );

            // InterlockingGenerator.cpp:170
            let (from_border_a, from_border_b) =
                self.grow_border_areas_perpendicular(&polys_a, &polys_b, detect)?;

            // Get the areas of each mesh that are _not_ thin (large), by performing a morphological open.
            // InterlockingGenerator.cpp:173-174
            let large_a = opening_ex(&polys_a, unscale(detect));
            let large_b = opening_ex(&polys_b, unscale(detect));

            // Derive the area that the thin areas need to expand into (so the added areas to the thin strips) from the information we already have.
            // InterlockingGenerator.cpp:177-181
            let thin_expansion_a = offset_expolygons(
                &intersection(
                    &intersection(
                        &intersection(
                            &large_b,
                            &offset_expolygons(
                                &difference(&polys_a, &large_a),
                                unscale(expand),
                                OffsetJoinType::Miter,
                            ),
                        ),
                        &near_interlock_per_layer[layer_nr],
                    ),
                    &from_border_a,
                ),
                unscale(ROUNDING_ERRORS),
                OffsetJoinType::Miter,
            );
            // InterlockingGenerator.cpp:182-186
            let thin_expansion_b = offset_expolygons(
                &intersection(
                    &intersection(
                        &intersection(
                            &large_a,
                            &offset_expolygons(
                                &difference(&polys_b, &large_b),
                                unscale(expand),
                                OffsetJoinType::Miter,
                            ),
                        ),
                        &near_interlock_per_layer[layer_nr],
                    ),
                    &from_border_b,
                ),
                unscale(ROUNDING_ERRORS),
                OffsetJoinType::Miter,
            );

            // Expanded thin areas of the opposing polygon should 'eat into' the larger areas of the polygon,
            // and conversely, add the expansions to their own thin areas.
            // InterlockingGenerator.cpp:190-191
            let new_a = closing(
                &difference(&union(&polys_a, &thin_expansion_a), &thin_expansion_b),
                unscale(close_gaps),
                OffsetJoinType::Miter,
            );
            let new_b = closing(
                &difference(&union(&polys_b, &thin_expansion_b), &thin_expansion_a),
                unscale(close_gaps),
                OffsetJoinType::Miter,
            );
            self.print_object.layers[layer_nr]
                .get_region_mut(self.region_a_index)
                .expect("layer region out of range")
                .slices
                .set(&new_a, SurfaceType::Internal);
            self.print_object.layers[layer_nr]
                .get_region_mut(self.region_b_index)
                .expect("layer region out of range")
                .slices
                .set(&new_b, SurfaceType::Internal);
        }
        Ok(())
    }

    /// Generate an interlocking embedding wall on a single layer.
    ///
    /// InterlockingGenerator.cpp:194
    /// C++: void InterlockingGenerator::generateInterlockingwall(Layer* layer) const
    /// (Rust takes the layer index into `print_object.layers` instead of a pointer.)
    fn generate_interlockingwall(&mut self, layer_idx: usize) {
        // get shell shape
        // InterlockingGenerator.cpp:196
        let mut voxels_per_mesh = self.get_layer_shell_voxels(&self.interface_dilation.clone(), layer_idx);

        // InterlockingGenerator.cpp:198-200
        let (any_slot, all_slot) = voxels_per_mesh.split_at_mut(1);
        let has_any_mesh = &mut any_slot[0];
        let has_all_meshes = &mut all_slot[0];
        // perform union and intersection simultaneously. Cannibalizes voxels_per_mesh
        unordered_set_merge(has_any_mesh, has_all_meshes);

        // InterlockingGenerator.cpp:202-204
        if has_all_meshes.is_empty() {
            return;
        }

        // InterlockingGenerator.cpp:206-208
        let mut layer_regions = ExPolygons::new();
        expolygons_append(
            &mut layer_regions,
            &to_expolygons(
                &self.print_object.layers[layer_idx]
                    .get_region(self.region_a_index)
                    .expect("layer region out of range")
                    .slices
                    .surfaces,
            ),
        );
        expolygons_append(
            &mut layer_regions,
            &to_expolygons(
                &self.print_object.layers[layer_idx]
                    .get_region(self.region_b_index)
                    .expect("layer region out of range")
                    .slices
                    .surfaces,
            ),
        );
        // InterlockingGenerator.cpp:209
        // Morphological close to merge meshes into single volume
        layer_regions = closing(
            &layer_regions,
            unscale(Self::IGNORED_GAP),
            OffsetJoinType::Miter,
        );

        // InterlockingGenerator.cpp:211-212
        let mut air_cells: HashSet<GridPoint3> = HashSet::new();
        let layer_id = self.print_object.layers[layer_idx].id();
        self.add_layer_boundary_cells(
            &layer_regions,
            layer_id as i32,
            &self.air_dilation.clone(),
            &mut air_cells,
        );

        // InterlockingGenerator.cpp:214-216
        for p in &air_cells {
            has_all_meshes.remove(p);
        }

        // InterlockingGenerator.cpp:218
        self.apply_embedding_to_outlines(has_all_meshes, &layer_regions, layer_idx, layer_id as i32);
    }

    /// Generate an interlocking structure between two meshes.
    ///
    /// InterlockingGenerator.cpp:221
    /// C++: void InterlockingGenerator::generateInterlockingStructure() const
    /// (named `_impl` because the snake_case of the C++ camelCase collides with
    /// the public static entry point `generate_interlocking_structure`.)
    fn generate_interlocking_structure_impl(&mut self) -> crate::Result<()> {
        // InterlockingGenerator.cpp:223
        let mut voxels_per_mesh = self.get_shell_voxels(&self.interface_dilation.clone());

        // InterlockingGenerator.cpp:225-227
        let (any_slot, all_slot) = voxels_per_mesh.split_at_mut(1);
        let has_any_mesh = &mut any_slot[0];
        let has_all_meshes = &mut all_slot[0];
        // perform union and intersection simultaneously. Cannibalizes voxels_per_mesh
        unordered_set_merge(has_any_mesh, has_all_meshes);

        // InterlockingGenerator.cpp:229-231
        if has_all_meshes.is_empty() {
            return Ok(());
        }

        // InterlockingGenerator.cpp:233
        let layer_regions = self.compute_unioned_volume_regions();

        // InterlockingGenerator.cpp:235-244
        if self.air_filtering {
            // InterlockingGenerator.cpp:236-237
            let mut air_cells: HashSet<GridPoint3> = HashSet::new();
            self.add_boundary_cells(&layer_regions, &self.air_dilation.clone(), &mut air_cells);

            // InterlockingGenerator.cpp:239-241
            for p in &air_cells {
                has_all_meshes.remove(p);
            }

            // InterlockingGenerator.cpp:243
            self.handle_thin_areas(has_all_meshes)?;
        }

        // InterlockingGenerator.cpp:246
        self.apply_microstructure_to_outlines(has_all_meshes, &layer_regions);
        Ok(())
    }

    /// Compute the voxels overlapping with the shell of both models on a
    /// single layer.
    ///
    /// InterlockingGenerator.cpp:248
    /// C++: std::vector<std::unordered_set<GridPoint3>> InterlockingGenerator::getLayerShellVoxels(
    ///          const DilationKernel& kernel, Layer* layer) const
    fn get_layer_shell_voxels(
        &self,
        kernel: &DilationKernel,
        layer_idx: usize,
    ) -> Vec<HashSet<GridPoint3>> {
        // InterlockingGenerator.cpp:249
        let mut voxels_per_mesh: Vec<HashSet<GridPoint3>> = vec![HashSet::new(), HashSet::new()];

        // mark all cells which contain some boundary
        // InterlockingGenerator.cpp:252-257
        for region_idx in 0..2usize {
            let region = if region_idx == 0 {
                self.region_a_index
            } else {
                self.region_b_index
            };
            let layer = &self.print_object.layers[layer_idx];
            let rotated_polygons_per_layer = to_expolygons(
                &layer
                    .get_region(region)
                    .expect("layer region out of range")
                    .slices
                    .surfaces,
            );
            let layer_id = layer.id();
            self.add_layer_boundary_cells(
                &rotated_polygons_per_layer,
                layer_id as i32,
                kernel,
                &mut voxels_per_mesh[region_idx],
            );
        }

        // InterlockingGenerator.cpp:259
        voxels_per_mesh
    }

    /// Compute the voxels overlapping with the shell of both models.
    /// This includes the walls, but also top/bottom skin.
    ///
    /// InterlockingGenerator.cpp:262
    /// C++: std::vector<std::unordered_set<GridPoint3>> InterlockingGenerator::getShellVoxels(const DilationKernel& kernel) const
    fn get_shell_voxels(&self, kernel: &DilationKernel) -> Vec<HashSet<GridPoint3>> {
        // InterlockingGenerator.cpp:264
        let mut voxels_per_mesh: Vec<HashSet<GridPoint3>> = vec![HashSet::new(), HashSet::new()];

        // mark all cells which contain some boundary
        // InterlockingGenerator.cpp:267-269
        for region_idx in 0..2usize {
            let region = if region_idx == 0 {
                self.region_a_index
            } else {
                self.region_b_index
            };

            // InterlockingGenerator.cpp:272-278
            let mut rotated_polygons_per_layer: Vec<ExPolygons> =
                vec![ExPolygons::new(); self.print_object.layers.len()];
            for layer_nr in 0..self.print_object.layers.len() {
                let layer = &self.print_object.layers[layer_nr];
                rotated_polygons_per_layer[layer_nr] = to_expolygons(
                    &layer
                        .get_region(region)
                        .expect("layer region out of range")
                        .slices
                        .surfaces,
                );
                expolygons_rotate(
                    &mut rotated_polygons_per_layer[layer_nr],
                    self.rotation as CoordF,
                );
            }

            // InterlockingGenerator.cpp:280
            self.add_boundary_cells(
                &rotated_polygons_per_layer,
                kernel,
                &mut voxels_per_mesh[region_idx],
            );
        }

        // InterlockingGenerator.cpp:283
        voxels_per_mesh
    }

    /// Compute the voxels overlapping with the shell of a single layer.
    ///
    /// InterlockingGenerator.cpp:285
    /// C++: void InterlockingGenerator::addLayerBoundaryCells(const ExPolygons& layers, const int& layer_cnt,
    ///          const DilationKernel& kernel, std::unordered_set<GridPoint3>& cells) const
    fn add_layer_boundary_cells(
        &self,
        layers: &ExPolygons,
        layer_cnt: i32,
        kernel: &DilationKernel,
        cells: &mut HashSet<GridPoint3>,
    ) {
        // InterlockingGenerator.cpp:290-296
        let cells_cell = RefCell::new(cells);
        let voxel_emplacer = |p: GridPoint3| -> bool {
            if p[2] < 0 {
                return true;
            }
            cells_cell.borrow_mut().insert(p);
            true
        };

        // InterlockingGenerator.cpp:298-299
        let z: Coord = layer_cnt as Coord;
        self.vu
            .walk_dilated_polygons_multi(layers, z, kernel, &voxel_emplacer);
        // InterlockingGenerator.cpp:300
        let skin = ExPolygons::new();
        // InterlockingGenerator.cpp:301 (commented out in C++)
        // skin = xor_ex(skin, layers[layer_nr - 1]);

        // InterlockingGenerator.cpp:303 (commented out in C++)
        // skin = opening_ex(skin, cell_size.x() / 2.f); // remove superfluous small areas, which would anyway be included because of walkPolygons
        // InterlockingGenerator.cpp:304
        self.vu
            .walk_dilated_areas_multi(&skin, z, kernel, &voxel_emplacer);
    }

    /// Compute the voxels overlapping with the shell of some layers.
    /// This includes the walls, but also top/bottom skin.
    ///
    /// InterlockingGenerator.cpp:309
    /// C++: void InterlockingGenerator::addBoundaryCells(const std::vector<ExPolygons>& layers,
    ///          const DilationKernel& kernel, std::unordered_set<GridPoint3>& cells) const
    fn add_boundary_cells(
        &self,
        layers: &[ExPolygons],
        kernel: &DilationKernel,
        cells: &mut HashSet<GridPoint3>,
    ) {
        // InterlockingGenerator.cpp:313-319
        let cells_cell = RefCell::new(cells);
        let voxel_emplacer = |p: GridPoint3| -> bool {
            if p[2] < 0 {
                return true;
            }
            cells_cell.borrow_mut().insert(p);
            true
        };

        // InterlockingGenerator.cpp:321-330
        for layer_nr in 0..layers.len() {
            let z: Coord = layer_nr as Coord;
            self.vu
                .walk_dilated_polygons_multi(&layers[layer_nr], z, kernel, &voxel_emplacer);
            // InterlockingGenerator.cpp:324
            let mut skin = layers[layer_nr].clone();
            // InterlockingGenerator.cpp:325-327
            if layer_nr > 0 {
                skin = xor(&skin, &layers[layer_nr - 1]);
            }
            // InterlockingGenerator.cpp:328
            // remove superfluous small areas, which would anyway be included because of walkPolygons
            skin = opening_ex(&skin, unscale(self.cell_size[0]) / 2.0);
            // InterlockingGenerator.cpp:329
            self.vu
                .walk_dilated_areas_multi(&skin, z, kernel, &voxel_emplacer);
        }
    }

    /// Compute the regions occupied by both models.
    ///
    /// A morphological close is performed so that we don't register small gaps
    /// between the two models as being separate.
    ///
    /// InterlockingGenerator.cpp:333
    /// C++: std::vector<ExPolygons> InterlockingGenerator::computeUnionedVolumeRegions() const
    fn compute_unioned_volume_regions(&self) -> Vec<ExPolygons> {
        // InterlockingGenerator.cpp:335-336
        // introduce ghost layer on top for correct skin computation of topmost layer.
        let max_layer_count: usize = self.print_object.layers.len() + 1;
        // InterlockingGenerator.cpp:337
        let mut layer_regions: Vec<ExPolygons> = vec![ExPolygons::new(); max_layer_count];

        // InterlockingGenerator.cpp:339-347
        for layer_nr in 0..(max_layer_count - 1) {
            for region_idx in [self.region_a_index, self.region_b_index] {
                // InterlockingGenerator.cpp:342-343
                let layer = &self.print_object.layers[layer_nr];
                let polys = to_expolygons(
                    &layer
                        .get_region(region_idx)
                        .expect("layer region out of range")
                        .slices
                        .surfaces,
                );
                expolygons_append(&mut layer_regions[layer_nr], &polys);
            }
            // InterlockingGenerator.cpp:345
            // Morphological close to merge meshes into single volume
            layer_regions[layer_nr] = closing(
                &layer_regions[layer_nr],
                unscale(Self::IGNORED_GAP),
                OffsetJoinType::Miter,
            );
            // InterlockingGenerator.cpp:346
            expolygons_rotate(&mut layer_regions[layer_nr], self.rotation as CoordF);
        }
        // InterlockingGenerator.cpp:348
        layer_regions
    }

    /// Generate the polygons for the beams of a single cell (single-layer
    /// embedding variant).
    ///
    /// InterlockingGenerator.cpp:352
    /// C++: ExPolygons InterlockingGenerator::generateLayerMicrostructure() const
    fn generate_layer_microstructure(&self) -> ExPolygons {
        // InterlockingGenerator.cpp:354
        let mut cell_area_per_mesh_per_layer = ExPolygons::new();
        // InterlockingGenerator.cpp:355-356
        let middle: Coord = self.cell_size[0] / 2;
        let width: [Coord; 2] = [middle, self.cell_size[0] - middle];
        // InterlockingGenerator.cpp:357-367
        for mesh_idx in 0..2usize {
            // InterlockingGenerator.cpp:358-359
            let offset = Point::new(if mesh_idx != 0 { middle } else { 0 }, 0);
            let area_size = Point::new(width[mesh_idx], self.cell_size[1]);

            // InterlockingGenerator.cpp:361-365
            let poly = Polygon::from_points(vec![
                offset,
                Point::new(offset.x + area_size.x, offset.y),
                Point::new(offset.x + area_size.x, offset.y + area_size.y),
                Point::new(offset.x, offset.y + area_size.y),
            ]);
            // InterlockingGenerator.cpp:366
            cell_area_per_mesh_per_layer.push(ExPolygon::new(poly));
        }
        // InterlockingGenerator.cpp:368
        cell_area_per_mesh_per_layer
    }

    /// Generate the polygons for the beams of a single cell.
    ///
    /// InterlockingGenerator.cpp:371
    /// C++: std::vector<std::vector<ExPolygons>> InterlockingGenerator::generateMicrostructure() const
    fn generate_microstructure(&self) -> Vec<Vec<ExPolygons>> {
        // InterlockingGenerator.cpp:373-375
        let mut cell_area_per_mesh_per_layer: Vec<Vec<ExPolygons>> = Vec::new();
        cell_area_per_mesh_per_layer.resize(2, Vec::new());
        cell_area_per_mesh_per_layer[0].resize(2, ExPolygons::new());
        // InterlockingGenerator.cpp:376-377 (commented out in C++)
        // const coord_t beam_w_sum = beam_width + beam_width;
        // const coord_t middle     = cell_size.x() * beam_width / beam_w_sum;
        // InterlockingGenerator.cpp:378-379
        let middle: Coord = self.cell_size[0] / 2;
        let width: [Coord; 2] = [middle, self.cell_size[0] - middle];
        // InterlockingGenerator.cpp:380-390
        for mesh_idx in 0..2usize {
            // InterlockingGenerator.cpp:381-382
            let offset = Point::new(if mesh_idx != 0 { middle } else { 0 }, 0);
            let area_size = Point::new(width[mesh_idx], self.cell_size[1]);

            // InterlockingGenerator.cpp:384-388
            let poly = Polygon::from_points(vec![
                offset,
                Point::new(offset.x + area_size.x, offset.y),
                Point::new(offset.x + area_size.x, offset.y + area_size.y),
                Point::new(offset.x, offset.y + area_size.y),
            ]);
            // InterlockingGenerator.cpp:389
            cell_area_per_mesh_per_layer[0][mesh_idx].push(ExPolygon::new(poly));
        }
        // InterlockingGenerator.cpp:391
        cell_area_per_mesh_per_layer[1] = cell_area_per_mesh_per_layer[0].clone();
        // InterlockingGenerator.cpp:392-398
        for polys in &mut cell_area_per_mesh_per_layer[1] {
            for poly in polys.iter_mut() {
                for p in &mut poly.contour.points {
                    std::mem::swap(&mut p.x, &mut p.y);
                }
            }
        }
        // InterlockingGenerator.cpp:399
        cell_area_per_mesh_per_layer
    }

    /// Change the outlines of the meshes with the computed embedding structure
    /// (single-layer variant).
    ///
    /// InterlockingGenerator.cpp:402
    /// C++: void InterlockingGenerator::applyEmbeddingToOutlines(const std::unordered_set<GridPoint3>& cells,
    ///          const ExPolygons& layer_regions, Layer* layer, const int& idx) const
    /// (Rust takes the layer index into `print_object.layers` instead of a pointer;
    /// `_idx` is unused in the C++ body as well.)
    fn apply_embedding_to_outlines(
        &mut self,
        cells: &HashSet<GridPoint3>,
        layer_regions: &ExPolygons,
        layer_idx: usize,
        _idx: i32,
    ) {
        // InterlockingGenerator.cpp:403
        let cell_area_per_mesh_per_layer = self.generate_layer_microstructure();

        // InterlockingGenerator.cpp:405 for each mesh the structure on each layer
        let mut structure = ExPolygons::new();
        // Every `beam_layer_count` number of layers are combined to an interlocking beam layer
        // to store these we need ceil(max_layer_count / beam_layer_count) of these layers
        // the formula is rewritten as (max_layer_count + beam_layer_count - 1) / beam_layer_count, so it works for integer division
        // Only compute cell structure for half the layers, because since our beams are two layers high, every odd layer of the structure will
        // be the same as the layer below.
        // InterlockingGenerator.cpp:411-418
        for grid_loc in cells {
            let bottom_corner = self.vu.to_lower_corner(*grid_loc);
            let mut areas_here = cell_area_per_mesh_per_layer.clone();
            for here in &mut areas_here {
                // C++: here.translate(bottom_corner.x(), bottom_corner.y());
                here.translate(Point::new(bottom_corner[0], bottom_corner[1]));
            }
            expolygons_append(&mut structure, &areas_here);
        }

        // InterlockingGenerator.cpp:420 (unused in the C++ body as well)
        let _layer_outlines = layer_regions.clone();
        // InterlockingGenerator.cpp:421
        let areas_here = intersection(&structure, layer_regions);
        // InterlockingGenerator.cpp:422-432
        for region_idx in 0..2usize {
            let region = if region_idx == 0 {
                self.region_a_index
            } else {
                self.region_b_index
            };
            // C++: layer->get_region(region)->region().config().wall_loops
            // (Rust PrintRegionConfig names `wall_loops` as `perimeters`.)
            let region_id = self.print_object.layers[layer_idx]
                .get_region(region)
                .expect("layer region out of range")
                .region_id();
            let wall_loops = self
                .print_object
                .printing_region(region_id)
                .expect("printing region out of range")
                .config()
                .perimeters;
            // InterlockingGenerator.cpp:425
            let polys = to_expolygons(
                &self.print_object.layers[layer_idx]
                    .get_region(region)
                    .expect("layer region out of range")
                    .slices
                    .surfaces,
            );
            // InterlockingGenerator.cpp:426-431
            let new_slices = if wall_loops > 0 {
                // reduce layer areas inward with beams from other mesh /
                // extend layer areas outward with newly added beams
                union(&polys, &areas_here)
            } else {
                difference(&polys, &areas_here)
            };
            self.print_object.layers[layer_idx]
                .get_region_mut(region)
                .expect("layer region out of range")
                .slices
                .set(&new_slices, SurfaceType::Internal);
        }
    }

    /// Change the outlines of the meshes with the computed interlocking structure.
    ///
    /// InterlockingGenerator.cpp:435
    /// C++: void InterlockingGenerator::applyMicrostructureToOutlines(const std::unordered_set<GridPoint3>& cells,
    ///          const std::vector<ExPolygons>& layer_regions) const
    fn apply_microstructure_to_outlines(
        &mut self,
        cells: &HashSet<GridPoint3>,
        layer_regions: &[ExPolygons],
    ) {
        // InterlockingGenerator.cpp:438
        let cell_area_per_mesh_per_layer = self.generate_microstructure();

        // InterlockingGenerator.cpp:440-441
        let unapply_rotation: f32 = -self.rotation;
        let max_layer_count: usize = self.print_object.layers.len();

        // InterlockingGenerator.cpp:443 for each mesh the structure on each layer
        let mut structure_per_layer: [Vec<ExPolygons>; 2] = [Vec::new(), Vec::new()];

        // Every `beam_layer_count` number of layers are combined to an interlocking beam layer
        // to store these we need ceil(max_layer_count / beam_layer_count) of these layers
        // the formula is rewritten as (max_layer_count + beam_layer_count - 1) / beam_layer_count, so it works for integer division
        // InterlockingGenerator.cpp:448-451
        let num_interlocking_layers: usize =
            (max_layer_count + self.beam_layer_count as usize - 1) / self.beam_layer_count as usize;
        structure_per_layer[0].resize(num_interlocking_layers, ExPolygons::new());
        structure_per_layer[1].resize(num_interlocking_layers, ExPolygons::new());

        // Only compute cell structure for half the layers, because since our beams are two layers high, every odd layer of the structure will
        // be the same as the layer below.
        // InterlockingGenerator.cpp:455-468
        for grid_loc in cells {
            let bottom_corner = self.vu.to_lower_corner(*grid_loc);
            for mesh_idx in 0..2usize {
                let mut layer_nr: Coord = bottom_corner[2];
                while layer_nr < bottom_corner[2] + self.cell_size[2]
                    && layer_nr < max_layer_count as Coord
                {
                    // InterlockingGenerator.cpp:460-461
                    let mut areas_here = cell_area_per_mesh_per_layer[(layer_nr
                        / self.beam_layer_count)
                        as usize
                        % cell_area_per_mesh_per_layer.len()][mesh_idx]
                        .clone();
                    // InterlockingGenerator.cpp:462-464
                    for here in &mut areas_here {
                        here.translate(Point::new(bottom_corner[0], bottom_corner[1]));
                    }
                    // InterlockingGenerator.cpp:465
                    expolygons_append(
                        &mut structure_per_layer[mesh_idx]
                            [(layer_nr / self.beam_layer_count) as usize],
                        &areas_here,
                    );
                    layer_nr += self.beam_layer_count;
                }
            }
        }

        // InterlockingGenerator.cpp:470-476
        for mesh_idx in 0..2usize {
            for layer_nr in 0..structure_per_layer[mesh_idx].len() {
                let layer_structure = &mut structure_per_layer[mesh_idx][layer_nr];
                // InterlockingGenerator.cpp:473
                *layer_structure = union_ex(layer_structure);
                // InterlockingGenerator.cpp:474
                expolygons_rotate(layer_structure, unapply_rotation as CoordF);
            }
        }

        // InterlockingGenerator.cpp:478-494
        for region_idx in 0..2usize {
            let region = if region_idx == 0 {
                self.region_a_index
            } else {
                self.region_b_index
            };
            for layer_nr in 0..max_layer_count {
                // InterlockingGenerator.cpp:481-482
                let mut layer_outlines = layer_regions[layer_nr].clone();
                expolygons_rotate(&mut layer_outlines, unapply_rotation as CoordF);

                // InterlockingGenerator.cpp:484-485
                let areas_here = intersection(
                    &structure_per_layer[region_idx][layer_nr / self.beam_layer_count as usize],
                    &layer_outlines,
                );
                let areas_other = &structure_per_layer[1 - region_idx]
                    [layer_nr / self.beam_layer_count as usize];

                // InterlockingGenerator.cpp:487-489
                let polys = to_expolygons(
                    &self.print_object.layers[layer_nr]
                        .get_region(region)
                        .expect("layer region out of range")
                        .slices
                        .surfaces,
                );
                // InterlockingGenerator.cpp:490-492
                // reduce layer areas inward with beams from other mesh /
                // extend layer areas outward with newly added beams
                let new_slices = union(&difference(&polys, areas_other), &areas_here);
                self.print_object.layers[layer_nr]
                    .get_region_mut(region)
                    .expect("layer region out of range")
                    .slices
                    .set(&new_slices, SurfaceType::Internal);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_print_object() -> PrintObject {
        PrintObject::new()
    }

    fn make_generator(print_object: &mut PrintObject) -> InterlockingGenerator<'_> {
        let beam_width: Coord = 200;
        let cell_width = beam_width + beam_width;
        InterlockingGenerator::new(
            print_object,
            0,
            1,
            beam_width,
            2,
            0.0,
            [cell_width, cell_width, 2],
            1,
            DilationKernel::new([2, 2, 2], DilationKernelType::Prism),
            DilationKernel::new([2, 2, 2], DilationKernelType::Prism),
            true,
            &[],
        )
    }

    #[test]
    fn test_generator_creation() {
        let mut po = make_print_object();
        let gen = make_generator(&mut po);
        assert_eq!(gen.region_a_index, 0);
        assert_eq!(gen.region_b_index, 1);
        assert_eq!(gen.beam_width, 200);
        assert_eq!(InterlockingGenerator::IGNORED_GAP, 100);
    }

    #[test]
    fn test_generate_layer_microstructure() {
        let mut po = make_print_object();
        let gen = make_generator(&mut po);
        let areas = gen.generate_layer_microstructure();
        // Two rectangles, one per mesh (InterlockingGenerator.cpp:357-367).
        assert_eq!(areas.len(), 2);
        for area in &areas {
            assert_eq!(area.contour.points.len(), 4);
        }
    }

    #[test]
    fn test_generate_microstructure() {
        let mut po = make_print_object();
        let gen = make_generator(&mut po);
        let micro = gen.generate_microstructure();
        // Two beam orientations, each with two meshes.
        assert_eq!(micro.len(), 2);
        assert_eq!(micro[0].len(), 2);
        assert_eq!(micro[1].len(), 2);
        // The second orientation is the first with x/y swapped
        // (InterlockingGenerator.cpp:391-398).
        for mesh_idx in 0..2 {
            let p0 = &micro[0][mesh_idx][0].contour.points;
            let p1 = &micro[1][mesh_idx][0].contour.points;
            for (a, b) in p0.iter().zip(p1.iter()) {
                assert_eq!(a.x, b.y);
                assert_eq!(a.y, b.x);
            }
        }
    }

    #[test]
    fn test_unordered_set_merge_union_and_intersection() {
        let mut a: HashSet<GridPoint3> = HashSet::new();
        a.insert([0, 0, 0]);
        a.insert([1, 0, 0]);
        a.insert([0, 1, 0]);

        let mut b: HashSet<GridPoint3> = HashSet::new();
        b.insert([1, 0, 0]);
        b.insert([0, 1, 0]);
        b.insert([1, 1, 0]);

        unordered_set_merge(&mut a, &mut b);
        // a holds the union, b the intersection (InterlockingGenerator.cpp:200/227).
        assert_eq!(a.len(), 4);
        assert_eq!(b.len(), 2);
        assert!(b.contains(&[1, 0, 0]));
        assert!(b.contains(&[0, 1, 0]));
    }

    #[test]
    fn test_add_boundary_cells_empty() {
        let mut po = make_print_object();
        let gen = make_generator(&mut po);
        let mut cells: HashSet<GridPoint3> = HashSet::new();
        let layers: Vec<ExPolygons> = vec![vec![]];
        let kernel = DilationKernel::new([2, 2, 2], DilationKernelType::Prism);
        gen.add_boundary_cells(&layers, &kernel, &mut cells);
        assert!(cells.is_empty());
    }

    #[test]
    fn test_generate_interlocking_structure_disabled_is_noop() {
        let mut po = make_print_object();
        // interlocking_beam defaults to false (PrintConfig.cpp:3670).
        assert!(!po.config.interlocking_beam);
        InterlockingGenerator::generate_interlocking_structure(&mut po, &[0.4]).unwrap();
    }
}
