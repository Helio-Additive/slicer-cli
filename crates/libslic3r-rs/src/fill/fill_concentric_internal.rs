//! Faithful 1:1 line-by-line port of BambuStudio
//! `src/libslic3r/Fill/FillConcentricInternal.cpp` (+ `.hpp`).
//!
//! C++ Reference:
//! - Fill/FillConcentricInternal.hpp
//! - Fill/FillConcentricInternal.cpp
//!
//! `coord_t` -> `i64` (`Coord`), `coordf_t` -> `f64` (`CoordF`).
//!
//! Modelling notes (see also the per-line comments):
//! - The C++ class derives from `Fill`. There is no shared `Fill` base struct in
//!   the Rust port (the fill module is otherwise procedural), so the members this
//!   method reads from the base — `no_overlap_expolygons`, `spacing`, `loop_clipping`
//!   — and the `FillConcentricInternal`-specific `print_config`/`print_object_config`
//!   pointers (FillConcentricInternal.hpp:19-20) are carried directly on this struct.
//! - C++ `print_config->nozzle_diameter.values` is a `ConfigOptionFloats` (a vector,
//!   one per extruder). The Rust `PrintConfig::nozzle_diameter` is a single scalar
//!   (`CoordF`), so `std::min_element(...)` over that single value is just the value
//!   itself. See the `min_nozzle_diameter` line.

// FillConcentricInternal.cpp:1-8
// #include "../ClipperUtils.hpp"
// #include "../ExPolygon.hpp"
// #include "../Surface.hpp"
// #include "../VariableWidth.hpp"
// #include "Arachne/WallToolPaths.hpp"
// #include "FillConcentricInternal.hpp"
// #include <libslic3r/ShortestPath.hpp>
use crate::arachne::utils::extrusion_line::{to_thick_polyline, ExtrusionLine, VariableWidthLines};
use crate::arachne::wall_tool_paths::{WallToolPaths, WallToolPathsParams};
use crate::extrusion_entity::{ExtrusionEntityCollection, ExtrusionEntityType};
use crate::flow::Flow;
use crate::geometry::{to_polygons_expoly, ExPolygon, ExPolygons, Point, Polygons, ThickPolylines};
use crate::print_config::{PrintConfig, PrintObjectConfig};
use crate::shortest_path::reorder_by_shortest_traverse;
use crate::surface::Surface;
use crate::variable_width::variable_width;
use crate::Coord;

use super::FillParams;

// FillConcentricInternal.cpp:10
// namespace Slic3r {

/// FillConcentricInternal.hpp:8-23  class FillConcentricInternal : public Fill
///
/// Carries the `Fill` base members this fill reads plus the
/// `FillConcentricInternal`-specific config pointers (FillConcentricInternal.hpp:19-20).
pub struct FillConcentricInternal<'a> {
    // Fill base (FillBase.hpp:111): `size_t layer_id;` — index of the layer,
    // assigned by the caller at Fill.cpp:611. This filler does not read it for
    // output; it is carried so the ARWTP probe can key on the same value the
    // C++ probe prints.
    pub layer_id: usize,
    // Fill base (FillBase.hpp): `coordf_t spacing;` — in unscaled coordinates.
    pub spacing: f64,
    // Fill base (FillBase.hpp): `coord_t loop_clipping;` — in scaled coordinates.
    pub loop_clipping: Coord,
    // Fill base (FillBase.hpp): `ExPolygons no_overlap_expolygons;`
    pub no_overlap_expolygons: ExPolygons,
    // FillConcentricInternal.hpp:19  const PrintConfig *print_config = nullptr;
    pub print_config: Option<&'a PrintConfig>,
    // FillConcentricInternal.hpp:20  const PrintObjectConfig *print_object_config = nullptr;
    pub print_object_config: Option<&'a PrintObjectConfig>,
}

impl<'a> FillConcentricInternal<'a> {
    /// FillConcentricInternal.hpp:17  bool no_sort() const override { return true; }
    pub fn no_sort(&self) -> bool {
        true
    }

    /// FillConcentricInternal.hpp:13  bool is_self_crossing() override { return false; }
    pub fn is_self_crossing(&self) -> bool {
        false
    }

    /// FillConcentricInternal.cpp:12-99
    /// void FillConcentricInternal::fill_surface_extrusion(const Surface* surface, const FillParams& params, ExtrusionEntitiesPtr& out)
    pub fn fill_surface_extrusion(
        &mut self,
        _surface: &Surface,
        params: &FillParams,
        out: &mut Vec<ExtrusionEntityType>,
    ) {
        // FillConcentricInternal.cpp:14
        assert!(self.print_config.is_some() && self.print_object_config.is_some());
        let print_config = self.print_config.unwrap();

        // FillConcentricInternal.cpp:16
        let mut thick_polylines_out: ThickPolylines = ThickPolylines::new();

        // FillConcentricInternal.cpp:18
        for i in 0..self.no_overlap_expolygons.len() {
            // FillConcentricInternal.cpp:19
            let expolygon: &ExPolygon = &self.no_overlap_expolygons[i];

            // no rotation is supported for this infill pattern
            // FillConcentricInternal.cpp:22
            let bbox_size: Point = expolygon.contour.bounding_box().size();
            // FillConcentricInternal.cpp:23
            let min_spacing: Coord = params.flow.scaled_spacing();

            // FillConcentricInternal.cpp:25
            let loops_count: Coord = std::cmp::max(bbox_size.x(), bbox_size.y()) / min_spacing + 1;
            // FillConcentricInternal.cpp:26
            let polygons: Polygons = to_polygons_expoly(expolygon);

            // FillConcentricInternal.cpp:28
            // double min_nozzle_diameter = *std::min_element(nozzle_diameter.values.begin(), ...);
            // The Rust PrintConfig holds a single nozzle diameter, so the min over the
            // (one-element) `values` is that value itself.
            let min_nozzle_diameter: f64 = print_config.nozzle_diameter;
            // FillConcentricInternal.cpp:29
            let mut input_params = WallToolPathsParams::default();
            // FillConcentricInternal.cpp:30
            input_params.min_bead_width = (0.85 * min_nozzle_diameter) as f32;
            // FillConcentricInternal.cpp:31
            input_params.min_feature_size = (0.25 * min_nozzle_diameter) as f32;
            // FillConcentricInternal.cpp:32
            input_params.wall_transition_length = 0.4;
            // FillConcentricInternal.cpp:33
            input_params.wall_transition_angle = 10.0;
            // FillConcentricInternal.cpp:34
            input_params.wall_transition_filter_deviation = (0.25 * min_nozzle_diameter) as f32;
            // FillConcentricInternal.cpp:35
            input_params.wall_distribution_count = 1;

            // R721 — the ARWTP probe reads the INPUT before `polygons` is moved
            // into the constructor. Gated, so nothing is computed when off.
            let probe_arwtp = crate::probe_enabled("ARWTP");
            let (probe_np, probe_ia, probe_iv, probe_ix, probe_iy) = if probe_arwtp {
                let mut iv = 0usize;
                let mut ix: i64 = 0;
                let mut iy: i64 = 0;
                for pg in polygons.iter() {
                    iv += pg.points.len();
                    for pt in pg.points.iter() {
                        ix = ix.wrapping_add(pt.x() as i64);
                        iy = iy.wrapping_add(pt.y() as i64);
                    }
                }
                (
                    polygons.len(),
                    polygons.iter().map(|p| p.area()).sum(),
                    iv,
                    ix,
                    iy,
                )
            } else {
                (0usize, 0.0f64, 0usize, 0i64, 0i64)
            };

            // FillConcentricInternal.cpp:37
            let mut wall_tool_paths = WallToolPaths::new(
                polygons,
                min_spacing,
                min_spacing,
                loops_count as usize,
                0,
                params.layer_height,
                input_params,
            );

            // FillConcentricInternal.cpp:39
            let loops: Vec<VariableWidthLines> = wall_tool_paths.get_tool_paths().clone();

            // R721 — probe the Arachne output the filler actually consumes.
            // Mirrors the C++ ARWTP probe injected at the same statement.
            if probe_arwtp {
                let mut el = 0usize;
                let mut jn = 0usize;
                let mut len = 0.0f64;
                let mut wsum = 0.0f64;
                let mut ox: i64 = 0;
                let mut oy: i64 = 0;
                for lp in loops.iter() {
                    for w in lp.iter() {
                        el += 1;
                        jn += w.junctions.len();
                        for k in 0..w.junctions.len() {
                            wsum += w.junctions[k].w as f64;
                            ox = ox.wrapping_add(w.junctions[k].p.x() as i64);
                            oy = oy.wrapping_add(w.junctions[k].p.y() as i64);
                            if k > 0 {
                                let dx = (w.junctions[k].p.x() - w.junctions[k - 1].p.x()) as f64;
                                let dy = (w.junctions[k].p.y() - w.junctions[k - 1].p.y()) as f64;
                                len += (dx * dx + dy * dy).sqrt();
                            }
                        }
                    }
                }
                eprintln!(
                    "[ARWTP] lid={} np={} ia={:.0} iv={} ix={} iy={} lcnt={} loops={} el={} jn={} len={:.0} wsum={:.0} ox={} oy={}",
                    self.layer_id,
                    probe_np,
                    probe_ia,
                    probe_iv,
                    probe_ix,
                    probe_iy,
                    loops_count,
                    loops.len(),
                    el,
                    jn,
                    len,
                    wsum,
                    ox,
                    oy
                );
            }

            // FillConcentricInternal.cpp:40
            let mut all_extrusions: Vec<&ExtrusionLine> = Vec::new();
            // FillConcentricInternal.cpp:41
            for loop_ in loops.iter() {
                // FillConcentricInternal.cpp:42-43
                if loop_.is_empty() {
                    continue;
                }
                // FillConcentricInternal.cpp:44-45
                for wall in loop_.iter() {
                    all_extrusions.push(wall);
                }
            }

            // Split paths using a nearest neighbor search.
            // FillConcentricInternal.cpp:49
            let firts_poly_idx: usize = thick_polylines_out.len();
            // FillConcentricInternal.cpp:50
            let last_pos: Point = Point::new(0, 0);
            // FillConcentricInternal.cpp:51
            for extrusion in all_extrusions.iter() {
                // FillConcentricInternal.cpp:52-53
                if extrusion.is_empty() {
                    continue;
                }

                // FillConcentricInternal.cpp:55
                let mut thick_polyline = to_thick_polyline(extrusion);
                // FillConcentricInternal.cpp:56
                if extrusion.is_closed
                    && thick_polyline.points.first() == thick_polyline.points.last()
                    && thick_polyline.widths.first() == thick_polyline.widths.last()
                {
                    // FillConcentricInternal.cpp:57
                    thick_polyline.points.pop();
                    // FillConcentricInternal.cpp:58
                    debug_assert!(thick_polyline.points.len() * 2 == thick_polyline.widths.len());
                    // FillConcentricInternal.cpp:59
                    let nearest_idx: i32 = last_pos.nearest_point_index(&thick_polyline.points);
                    // FillConcentricInternal.cpp:60  std::rotate(points.begin(), points.begin() + nearest_idx, points.end());
                    thick_polyline.points.rotate_left(nearest_idx as usize);
                    // FillConcentricInternal.cpp:61  std::rotate(width.begin(), width.begin() + 2 * nearest_idx, width.end());
                    thick_polyline.widths.rotate_left(2 * nearest_idx as usize);
                    // FillConcentricInternal.cpp:62  points.emplace_back(points.front());
                    let front = thick_polyline.points[0];
                    thick_polyline.points.push(front);
                }
                // FillConcentricInternal.cpp:64
                thick_polylines_out.push(thick_polyline);
            }

            // clip the paths to prevent the extruder from getting exactly on the first point of the loop
            // Keep valid paths only.
            // FillConcentricInternal.cpp:69
            let mut j: usize = firts_poly_idx;
            // FillConcentricInternal.cpp:70
            for i in firts_poly_idx..thick_polylines_out.len() {
                // FillConcentricInternal.cpp:71
                thick_polylines_out[i].clip_end(self.loop_clipping as f64);
                // FillConcentricInternal.cpp:72
                if thick_polylines_out[i].is_valid() {
                    // FillConcentricInternal.cpp:73-74
                    if j < i {
                        thick_polylines_out[j] = std::mem::take(&mut thick_polylines_out[i]);
                    }
                    // FillConcentricInternal.cpp:75
                    j += 1;
                }
            }
            // FillConcentricInternal.cpp:78-79
            if j < thick_polylines_out.len() {
                thick_polylines_out.truncate(j);
            }

            // FillConcentricInternal.cpp:81
            reorder_by_shortest_traverse(&mut thick_polylines_out);
        }

        // FillConcentricInternal.cpp:84
        let mut coll_nosort = ExtrusionEntityCollection::new();
        // FillConcentricInternal.cpp:85  can be sorted inside the pass
        coll_nosort.no_sort = self.no_sort();

        // FillConcentricInternal.cpp:87
        if !thick_polylines_out.is_empty() {
            // FillConcentricInternal.cpp:88
            let new_flow: Flow = params
                .flow
                .with_spacing(self.spacing as f32 as f64)
                .expect("with_spacing");
            // FillConcentricInternal.cpp:89
            let mut gap_fill = ExtrusionEntityCollection::new();
            // FillConcentricInternal.cpp:90
            variable_width(
                &thick_polylines_out,
                params.extrusion_role,
                &new_flow,
                &mut gap_fill.entities,
            );
            // FillConcentricInternal.cpp:91  coll_nosort->append(std::move(gap_fill.entities));
            for entity in gap_fill.entities.drain(..) {
                coll_nosort.append(entity);
            }
        }

        // FillConcentricInternal.cpp:94-97
        if !coll_nosort.entities.is_empty() {
            out.push(ExtrusionEntityType::Collection(Box::new(coll_nosort)));
        }
        // (else branch: C++ `delete coll_nosort` — the Rust owned value is dropped here.)
    }
}

// FillConcentricInternal.cpp:102
// } // namespace Slic3r
