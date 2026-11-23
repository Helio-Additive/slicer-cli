//! Native parity of `ContinuitiousDistancer`, `Layer::calculate_perimeter_continuity`, `Layer::make_slices`,
//! `get_extents`, `Layer::backup_untyped_slices` / `restore_untyped_slices*` and `Layer::merged`
//! (`slicer::layer`) against vectors made by the C++ at the pin (tests/data/layer/gen.cpp).
//!
//! Every line of the vectors is "<what> <inputs> => <results>": this test reads the inputs back, calls
//! the port and has to print the same results. `LAYER_PARITY_VECTORS` names a file of the generator to
//! read instead of the committed one.

use slicer::geometry::{BoundingBox, ExPolygon, Point, Polygon};
use slicer::layer::{
    get_extents, get_extents_layer_region, get_extents_layer_regions, ContinuitiousDistancer, Layer, LayerRegion, LoopNode, NodeContour,
};
use slicer::surface::{Surface, SurfaceType};
use std::io::Read;

/// The tokens of the inputs of a line.
struct Tok<'a> {
    t: std::str::SplitWhitespace<'a>,
}
impl<'a> Tok<'a> {
    fn new(s: &'a str) -> Self {
        Tok { t: s.split_whitespace() }
    }
    fn s(&mut self) -> &'a str {
        self.t.next().expect("token")
    }
    fn i(&mut self) -> i64 {
        self.s().parse().expect("integer")
    }
    fn f(&mut self) -> f32 {
        f32::from_bits(u32::from_str_radix(self.s(), 16).expect("float"))
    }
    fn b(&mut self) -> bool {
        self.s() == "1"
    }
    fn point(&mut self) -> Point {
        let x = self.i();
        let y = self.i();
        Point::new(x, y)
    }
    /// gen.cpp `EX(ex)`
    fn expolygon(&mut self) -> ExPolygon {
        let n = self.i();
        let contour = Polygon { points: (0..n).map(|_| self.point()).collect() };
        let nh = self.i();
        let mut holes = Vec::new();
        for _ in 0..nh {
            let n = self.i();
            holes.push(Polygon { points: (0..n).map(|_| self.point()).collect() });
        }
        ExPolygon { contour, holes }
    }
    /// gen.cpp `NODE(n)`
    fn node(&mut self) -> LoopNode {
        let node_id = self.i() as usize;
        let min = self.point();
        let max = self.point();
        let is_loop = self.b();
        let np = self.i();
        let pts: Vec<Point> = (0..np).map(|_| self.point()).collect();
        let nw = self.i();
        let widths: Vec<i64> = (0..nw).map(|_| self.i()).collect();
        let mut node = LoopNode::new(node_id, node_id, BoundingBox::from_points_minmax(min, max));
        node.node_contour = NodeContour { pts, widths, is_loop };
        node
    }
}

fn read_vectors() -> Vec<String> {
    if let Ok(path) = std::env::var("LAYER_PARITY_VECTORS") {
        let text = std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
        return text.lines().map(|l| l.to_string()).collect();
    }
    let path = format!("{}/tests/data/layer/vectors.txt.gz", env!("CARGO_MANIFEST_DIR"));
    let file = std::fs::File::open(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
    let mut text = String::new();
    flate2::read::GzDecoder::new(file).read_to_string(&mut text).unwrap_or_else(|e| panic!("{path}: {e}"));
    text.lines().map(|l| l.to_string()).collect()
}

/// gen.cpp `EX(ex)`
fn ex_str(ex: &ExPolygon) -> String {
    let mut o: Vec<String> = vec![ex.contour.points.len().to_string()];
    o.extend(ex.contour.points.iter().map(|p| format!("{} {}", p.x, p.y)));
    o.push(ex.holes.len().to_string());
    for h in &ex.holes {
        o.push(h.points.len().to_string());
        o.extend(h.points.iter().map(|p| format!("{} {}", p.x, p.y)));
    }
    o.join(" ")
}

/// gen.cpp `BB(bb)`: a box that is not defined has both corners at the origin at the pin (BoundingBox.hpp).
fn bb_str(bb: &BoundingBox) -> String {
    if bb.is_defined() {
        format!("1 {} {} {} {}", bb.min.x, bb.min.y, bb.max.x, bb.max.y)
    } else {
        "0 0 0 0 0".to_string()
    }
}

/// gen.cpp `SLICES(layer)`
fn slices_str(layer: &Layer, o: &mut Vec<String>) {
    for layerm in layer.regions() {
        o.push(layerm.slices.surfaces.len().to_string());
        for surface in &layerm.slices.surfaces {
            o.push(if surface.surface_type == SurfaceType::Internal { "1" } else { "0" }.to_string());
            o.push(ex_str(&surface.expolygon));
        }
    }
}

#[test]
fn layer_native_parity() {
    let expected = read_vectors();
    let mut mismatches = 0usize;
    // Lines on which the body `Layer::merged` had before R928 gives another result than the pin (reported).
    let mut legacy_merged_differs = 0usize;
    let mut by_kind: std::collections::BTreeMap<String, (usize, usize)> = std::collections::BTreeMap::new();
    for (k, line) in expected.iter().enumerate() {
        let (head, want) = line.split_once(" => ").unwrap_or_else(|| panic!("no result in {line}"));
        let (what, inputs) = head.split_once(' ').unwrap_or((head, ""));
        let mut t = Tok::new(inputs);
        let result: String = match what {
            "dist" => {
                let np = t.i();
                let pts: Vec<Point> = (0..np).map(|_| t.point()).collect();
                let distancer = ContinuitiousDistancer::new(&pts);
                let nq = t.i();
                let mut o: Vec<String> = Vec::new();
                for _ in 0..nq {
                    let x = t.f();
                    let y = t.f();
                    o.push(format!("{:08x}", distancer.distance_from_perimeter([x, y]).to_bits()));
                }
                o.join(" ")
            }
            "continuity" => {
                let n = t.i();
                let mut layer = Layer::new(1, 0, 0.2, 0.4, 0.3);
                for _ in 0..n {
                    let node = t.node();
                    layer.loop_nodes.push(node);
                }
                let m = t.i();
                let mut prev_nodes: Vec<LoopNode> = (0..m).map(|_| t.node()).collect();
                layer.calculate_perimeter_continuity(&mut prev_nodes);
                let mut o: Vec<String> = Vec::new();
                for node in &layer.loop_nodes {
                    o.push(node.lower_node_ids.len().to_string());
                    o.extend(node.lower_node_ids.iter().map(|id| id.to_string()));
                }
                for node in &prev_nodes {
                    o.push(node.upper_node_ids.len().to_string());
                    o.extend(node.upper_node_ids.iter().map(|id| id.to_string()));
                }
                o.join(" ")
            }
            // Layer.cpp:47-148, 750-770
            "slices" => {
                let nreg = t.i();
                let mut layer = Layer::new(1, 0, 0.2, 0.4, 0.3);
                for r in 0..nreg {
                    let mut layerm = LayerRegion::new(1, r as usize);
                    let nsurf = t.i();
                    for _ in 0..nsurf {
                        let ex = t.expolygon();
                        layerm.slices.surfaces.push(Surface::new(SurfaceType::Internal, ex));
                    }
                    layer.add_region(layerm);
                }
                layer.make_slices();
                let mut o: Vec<String> = vec![layer.lslices.len().to_string()];
                o.extend(layer.lslices.iter().map(ex_str));
                for layerm in layer.regions() {
                    o.push(bb_str(&get_extents(layerm)));
                    assert_eq!(bb_str(&get_extents_layer_region(layerm)), bb_str(&get_extents(layerm)));
                }
                o.push(bb_str(&get_extents(layer.regions())));
                assert_eq!(bb_str(&get_extents_layer_regions(layer.regions())), bb_str(&get_extents(layer.regions())));
                layer.backup_untyped_slices();
                for layerm in layer.regions_mut() {
                    for surface in layerm.slices.surfaces.iter_mut() {
                        surface.surface_type = SurfaceType::Top;
                    }
                    if layerm.slices.surfaces.len() > 1 {
                        layerm.slices.surfaces.pop();
                    }
                }
                layer.restore_untyped_slices();
                slices_str(&layer, &mut o);
                for layerm in layer.regions_mut() {
                    for surface in layerm.slices.surfaces.iter_mut() {
                        surface.surface_type = SurfaceType::Bottom;
                    }
                    if !layerm.slices.surfaces.is_empty() {
                        layerm.slices.surfaces.remove(0);
                    }
                }
                layer.restore_untyped_slices_no_extra_perimeters();
                slices_str(&layer, &mut o);
                o.join(" ")
            }
            // Layer.cpp:150-170
            "merged" => {
                let offset_scaled = t.f();
                let nreg = t.i();
                let mut layer = Layer::new(1, 0, 0.2, 0.4, 0.3);
                for r in 0..nreg {
                    let mut cfg = slicer::region_config::PrintRegionConfig::default();
                    cfg.bottom_solid_layers = t.i() as u32;
                    cfg.top_solid_layers = t.i() as u32;
                    cfg.fill_density = t.i() as f64;
                    cfg.perimeters = t.i() as u32;
                    let mut layerm = LayerRegion::new(1, r as usize);
                    layerm.set_config_hierarchy(
                        std::sync::Arc::new(slicer::print_region::PrintRegion::new(cfg)),
                        std::sync::Arc::new(slicer::print_config::PrintObjectConfig::default()),
                        std::sync::Arc::new(slicer::print_config::PrintConfig::default()),
                    );
                    let nsurf = t.i();
                    for _ in 0..nsurf {
                        let ex = t.expolygon();
                        layerm.slices.surfaces.push(Surface::new(SurfaceType::Internal, ex));
                    }
                    layer.add_region(layerm);
                }
                let merged = layer.merged_f32(offset_scaled);
                // `merged` is the same function behind the double argument older callers pass.
                assert_eq!(layer.merged(offset_scaled as f64), merged);
                if layer.merged_legacy(offset_scaled as f64) != merged {
                    legacy_merged_differs += 1;
                }
                let mut o: Vec<String> = vec![merged.len().to_string()];
                o.extend(merged.iter().map(ex_str));
                o.join(" ")
            }
            // Layer.cpp:173-198: one option of the second config differs from the first's.
            "compat" => {
                use slicer::print_config::{ScarfSeamType, WallSequence};
                let layer_id = t.i();
                let key = t.i();
                let order = t.i();
                let layer = Layer::new(layer_id as usize, 0, 0.2, 0.2 * (layer_id + 1) as f64, 0.2 * layer_id as f64 + 0.1);
                let mut base = slicer::region_config::PrintRegionConfig::default();
                base.wall_filament = 1;
                base.perimeters = 2;
                base.wall_sequence = WallSequence::InnerOuter;
                base.perimeter_speed = 60.;
                base.external_perimeter_speed = 30.;
                base.gap_fill_speed = 50.;
                base.bridge_speed = 25.;
                base.overhangs = true;
                base.filter_out_gap_fill = 0.;
                base.inner_wall_line_width = 0.45;
                base.outer_wall_line_width = 0.42;
                base.thin_walls = false;
                base.infill_overlap = 15.;
                base.override_filament_scarf_seam_setting = false;
                base.seam_slope_type = ScarfSeamType::None;
                base.seam_slope_start_height.value = 10.;
                base.seam_slope_start_height.percent = false;
                base.seam_slope_gap.value = 0.;
                base.seam_slope_gap.percent = false;
                base.seam_slope_min_length = 10.;
                base.seam_slope_conditional = false;
                base.seam_slope_entire_loop = false;
                base.seam_slope_steps = 10;
                base.seam_slope_inner_walls = false;
                base.initial_layer_flow_ratio = 1.;
                base.fill_density = 15.;
                base.top_solid_layers = 3;
                let mut other = base.clone();
                match key {
                    0 => {}
                    1 => other.wall_filament = 2,
                    2 => other.perimeters = 3,
                    3 => other.wall_sequence = WallSequence::OuterInner,
                    4 => other.perimeter_speed = 55.,
                    5 => other.external_perimeter_speed = 35.,
                    6 => other.gap_fill_speed = 45.,
                    7 => other.bridge_speed = 20.,
                    8 => other.overhangs = false,
                    9 => other.filter_out_gap_fill = 0.5,
                    10 => other.inner_wall_line_width = 0.5,
                    11 => other.outer_wall_line_width = 0.45,
                    12 => other.thin_walls = true,
                    13 => other.infill_overlap = 25.,
                    14 => other.override_filament_scarf_seam_setting = true,
                    15 => other.seam_slope_type = ScarfSeamType::External,
                    16 => other.seam_slope_start_height.value = 20.,
                    17 => other.seam_slope_gap.value = 0.1,
                    18 => other.seam_slope_min_length = 20.,
                    19 => other.seam_slope_conditional = true,
                    20 => other.seam_slope_entire_loop = true,
                    21 => other.seam_slope_steps = 12,
                    22 => other.seam_slope_inner_walls = true,
                    23 => other.initial_layer_flow_ratio = 0.9,
                    24 => other.fill_density = 30.,
                    25 => other.top_solid_layers = 7,
                    26 => other.seam_slope_start_height.percent = true,
                    k => panic!("unknown compat key {k}"),
                }
                let res = if order == 0 { layer.has_compatible_layer_regions(&base, &other) } else { layer.has_compatible_layer_regions(&other, &base) };
                (res as u8).to_string()
            }
            // Layer.cpp:379-380: the merge of one group of slices, as the live Layer::make_perimeters does it
            // (union_safety_offset_ex of the surfaces' polygons; the pin: offset_ex(Surfaces, ClipperSafetyOffset)).
            "group" => {
                let nsurf = t.i();
                let group: Vec<Surface> = (0..nsurf).map(|_| Surface::new(SurfaceType::Internal, t.expolygon())).collect();
                let group_polys = slicer::surface::to_polygons(&group);
                let merged = slicer::clipper_utils::union_safety_offset_ex(&group_polys);
                let mut o: Vec<String> = vec![merged.len().to_string()];
                o.extend(merged.iter().map(ex_str));
                o.join(" ")
            }
            // Layer.cpp:395-403: the fill surfaces of a shared pass split back among the regions, with the
            // calls the live Layer::make_perimeters makes.
            "split" => {
                let nreg = t.i();
                let mut region_slices: Vec<Vec<Surface>> = Vec::new();
                for _ in 0..nreg {
                    let nsurf = t.i();
                    region_slices.push((0..nsurf).map(|_| Surface::new(SurfaceType::Internal, t.expolygon())).collect());
                }
                let nfill = t.i();
                let fill_surfaces: Vec<Surface> = (0..nfill).map(|_| Surface::new(SurfaceType::Internal, t.expolygon())).collect();
                let nno = t.i();
                let fill_no_overlap: Vec<ExPolygon> = (0..nno).map(|_| t.expolygon()).collect();
                let mut o: Vec<String> = Vec::new();
                for slices in &region_slices {
                    let slice_expp = slicer::surface::to_expolygons(slices);
                    let expp = slicer::clipper_utils::intersection_ex(&fill_surfaces, &slice_expp);
                    let no_overlap = slicer::clipper_utils::intersection_ex(slices, &fill_no_overlap);
                    o.push(expp.len().to_string());
                    o.extend(expp.iter().map(ex_str));
                    o.push(no_overlap.len().to_string());
                    o.extend(no_overlap.iter().map(ex_str));
                }
                o.join(" ")
            }
            other => panic!("unknown line {other}"),
        };
        let entry = by_kind.entry(what.to_string()).or_insert((0, 0));
        entry.0 += 1;
        if result != want {
            entry.1 += 1;
            mismatches += 1;
            if mismatches <= 10 {
                let cut = |s: &str| if s.len() > 500 { format!("{}...", &s[..500]) } else { s.to_string() };
                eprintln!("line {} {}:\n  C++:  {}\n  Rust: {}", k + 1, cut(head), cut(want), cut(&result));
            }
        }
    }
    for (kind, (n, bad)) in &by_kind {
        eprintln!("{kind}: {n} lines, {bad} differ");
    }
    eprintln!("merged_legacy (not the pin's function, reported only): {legacy_merged_differs} lines differ");
    eprintln!("layer parity: {} lines, {} mismatches", expected.len(), mismatches);
    assert_eq!(mismatches, 0, "{mismatches} of {} lines differ from the pin", expected.len());
}
