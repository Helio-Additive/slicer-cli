//! Native parity of the PrintObjectSlice.cpp port (`slicer::print_object_slice`) against vectors made
//! by the C++ at the pin (tests/data/print_object_slice/gen.cpp).
//!
//! The generator puts small Models with a full print config through `Print::apply`, runs the helpers of
//! PrintObjectSlice.cpp on every object (new_layers, slice_volumes_inner, both slice_volume, slices_to_regions),
//! then the step itself (`PrintObject::slice()`) and writes the layers, and last what reads the sliced
//! object (support_shrinkage_scale, should_skip_filament_shrink, slice_support_volumes). The other helpers
//! run on synthetic inputs. This test builds the same models, assembles the Print as `Print::apply` does
//! for a fresh Print (as tests/print_native_parity.rs) and has to reproduce every line.
//!
//! The lines of the vectors that start with "in" are inputs, which this test reads back: the mode of
//! the file, the arguments of the synthetic cases, and what the two segmentations by painting returned
//! in the C++ (they are handed to the port through `SliceVolumesCores`). The long part of a line is in
//! the vectors as "#" and the FNV-1a hash of its text; `PRINT_OBJECT_SLICE_PARITY_VECTORS` names a file
//! the generator wrote with every line in full (`gen <file> full`), for finding the first token that
//! differs.

use slicer::config::{
    ConfigBase, ConfigOptionFloats, ConfigOptionInts, ConfigOptionResolver, ConfigOptionStrings, ConfigSubstitutionContext,
    ForwardCompatibilitySubstitutionRule,
};
use slicer::geometry::geometry::Vec3d;
use slicer::geometry::{BoundingBox, ExPolygon, ExPolygons, Point, Polygon, Polygons};
use slicer::layer::{zs_from_layers, Layer};
use slicer::model::*;
use slicer::normal_utils::indexed_triangle_set;
use slicer::object_id::ObjectID;
use slicer::print::faithful::{groupedVolumeSlices, AlignedBox3f, LayerRangeRegions, Print, PrintObject, PrintObjectStep, VolumeSlices};
use slicer::print_apply::*;
use slicer::print_base::CanceledException;
use slicer::print_config::config_def::DynamicPrintConfig;
use slicer::print_object_slice::*;
use slicer::surface::Surface;
use slicer::triangle_mesh::{its_make_cube, its_make_cylinder, its_make_prism, its_make_pyramid, its_make_sphere, Vec3f};
use std::io::Read;

fn hd(v: f64) -> String {
    format!("{:016x}", v.to_bits())
}
fn hb(v: bool) -> &'static str {
    if v {
        "1"
    } else {
        "0"
    }
}
fn pt(p: &Point) -> String {
    format!("{} {}", p.x, p.y)
}
fn poly_s(p: &Polygon) -> String {
    let mut o = format!("{}", p.points.len());
    for q in &p.points {
        o += &format!(" {}", pt(q));
    }
    o
}
fn polys_s(ps: &[Polygon]) -> String {
    let mut o = format!("{}", ps.len());
    for p in ps {
        o += &format!(" {}", poly_s(p));
    }
    o
}
fn expoly_s(e: &ExPolygon) -> String {
    let mut o = format!("{} {}", poly_s(&e.contour), e.holes.len());
    for h in &e.holes {
        o += &format!(" {}", poly_s(h));
    }
    o
}
fn expolys_s(es: &[ExPolygon]) -> String {
    let mut o = format!("{}", es.len());
    for e in es {
        o += &format!(" {}", expoly_s(e));
    }
    o
}
/// gen.cpp `BBOX(b)`
fn bbox_s(b: &BoundingBox) -> String {
    format!("{} {} {}", hb(b.is_defined()), pt(&b.min), pt(&b.max))
}
/// gen.cpp `SURFACES(ss)`
fn surfaces_s(ss: &[Surface]) -> String {
    let mut o = format!("{}", ss.len());
    for s in ss {
        o += &format!(" {} {}", s.surface_type as i32, expoly_s(&s.expolygon));
    }
    o
}

/// The tokens of an input line.
struct Tok<'a> {
    t: std::str::SplitWhitespace<'a>,
}
#[allow(dead_code)]
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
    fn d(&mut self) -> f64 {
        f64::from_bits(u64::from_str_radix(self.s(), 16).expect("double"))
    }
    fn b(&mut self) -> bool {
        self.s() == "1"
    }
    fn f(&mut self) -> f32 {
        f32::from_bits(u32::from_str_radix(self.s(), 16).expect("float"))
    }
    fn poly(&mut self) -> Polygon {
        let n = self.i();
        let mut p = Polygon::new();
        for _ in 0..n {
            let x = self.i();
            let y = self.i();
            p.points.push(Point::new(x, y));
        }
        p
    }
    fn expoly(&mut self) -> ExPolygon {
        let contour = self.poly();
        let nh = self.i();
        let mut holes = Vec::new();
        for _ in 0..nh {
            holes.push(self.poly());
        }
        ExPolygon::with_holes(contour, holes)
    }
    fn expolys(&mut self) -> ExPolygons {
        let n = self.i();
        (0..n).map(|_| self.expoly()).collect()
    }
}

/// The vectors, read in order: an input line is handed out and echoed, a result line is produced.
struct Run {
    expected: Vec<String>,
    pos: usize,
    lines: Vec<String>,
    surplus: usize,
    /// The vectors carry every line in full ("in mode full").
    full: bool,
}

/// gen.cpp `fnv(s)`
fn fnv(s: &str) -> String {
    let mut h: u64 = 1469598103934665603;
    for c in s.bytes() {
        h ^= c as u64;
        h = h.wrapping_mul(1099511628211);
    }
    format!("{h:016x}")
}
impl Run {
    /// The next input line when it is the input `prefix`; it is echoed. The results produced since the
    /// input before it are aligned with the vectors: missing ones are padded, surplus ones counted.
    fn input(&mut self, prefix: &str) -> Option<String> {
        let j = (self.pos..self.expected.len()).find(|&k| self.expected[k].starts_with("in "))?;
        let line = self.expected[j].clone();
        if !(line.starts_with(prefix) && line[prefix.len()..].starts_with(' ')) {
            return None;
        }
        while self.lines.len() < j {
            self.lines.push("<missing>".into());
        }
        if self.lines.len() > j {
            self.surplus += self.lines.len() - j;
            self.lines.truncate(j);
        }
        self.lines.push(line.clone());
        self.pos = j + 1;
        Some(line[prefix.len() + 1..].to_string())
    }
    fn out(&mut self, s: String) {
        self.lines.push(s);
    }
    /// gen.cpp `outb(head, body)`
    fn outb(&mut self, head: String, body: String) {
        if self.full {
            self.lines.push(format!("{head} {body}"));
        } else {
            self.lines.push(format!("{head} #{}", fnv(&body)));
        }
    }
}

fn box_its(sx: f64, sy: f64, sz: f64, ox: f32, oy: f32, oz: f32) -> indexed_triangle_set {
    moved(its_make_cube(sx, sy, sz), ox, oy, oz)
}
/// gen.cpp `moved(its, ox, oy, oz)`
fn moved(mut its: indexed_triangle_set, ox: f32, oy: f32, oz: f32) -> indexed_triangle_set {
    for v in &mut its.vertices {
        *v += Vec3f::new(ox, oy, oz);
    }
    its
}
fn mesh(its: indexed_triangle_set) -> ModelMesh {
    ModelMesh::from_its(its)
}

fn deser(config: &mut DynamicPrintConfig, key: &str, value: &str) {
    let mut ctxt = ConfigSubstitutionContext::new(ForwardCompatibilitySubstitutionRule::Disable);
    config.set_deserialize(key, value, &mut ctxt, false).unwrap_or_else(|e| panic!("{key}={value}: {e:?}"));
}

/// gen.cpp `N_SCENARIOS`
const N_SCENARIOS: i32 = 26;

/// gen.cpp `scenario_config(scenario)`
fn scenario_config(scenario: i32) -> Vec<(&'static str, &'static str)> {
    match scenario {
        3 => vec![("enable_order_independent_overlap_carving", "1")],
        5 => vec![("xy_hole_compensation", "0.1"), ("xy_contour_compensation", "-0.05"), ("elefant_foot_compensation", "0.15")],
        6 => vec![("xy_hole_compensation", "-0.08"), ("xy_contour_compensation", "0.12"), ("elefant_foot_compensation", "0")],
        7 => vec![("xy_hole_compensation", "0.1"), ("xy_contour_compensation", "0.07"), ("elefant_foot_compensation", "0.2")],
        8 => vec![("xy_hole_compensation", "-0.05"), ("xy_contour_compensation", "0.1"), ("raft_layers", "2")],
        12 => vec![("filament_shrink", "98%,97%,100%")],
        15 => vec![("spiral_mode", "1")],
        16 => vec![("slicing_mode", "even_odd")],
        17 => vec![("slicing_mode", "close_holes")],
        18 => vec![("resolution", "0.0005"), ("slice_closing_radius", "0.02"), ("layer_height", "0.12")],
        20 => vec![("xy_hole_compensation", "-0.1"), ("xy_contour_compensation", "-0.07"), ("elefant_foot_compensation", "0.1")],
        22 => vec![("xy_hole_compensation", "0.1"), ("xy_contour_compensation", "0.05"), ("elefant_foot_compensation", "0")],
        23 => vec![("filament_shrink", "98%,97%,100%")],
        24 => vec![
            ("multi_nozzle_diameter_mode", "1"),
            ("extruder_nozzle_count", "3"),
            ("extruder_nozzle_diameter", "0.2,0.4,0.8"),
            ("filament_diameter_map", "0.2;0.8;0.4"),
        ],
        25 => vec![
            ("multi_nozzle_diameter_mode", "1"),
            ("extruder_nozzle_count", "3"),
            ("extruder_nozzle_diameter", "0.2,0.4,0.8"),
            ("filament_diameter_map", "0.4;0.2;0.8"),
        ],
        _ => Vec::new(),
    }
}

/// gen.cpp `add_multi(o)`
fn add_multi(o: &mut ModelObject) {
    o.add_volume_mesh(mesh(box_its(10.0, 10.0, 6.0, -5.0, -5.0, 0.0)), ModelVolumeType::ModelPart, true);
    o.add_volume_mesh(mesh(box_its(10.0, 10.0, 6.0, 0.0, -2.0, 0.0)), ModelVolumeType::ModelPart, true).config.set("extruder", 2);
    o.add_volume_mesh(mesh(box_its(6.0, 6.0, 4.0, -8.0, -8.0, 0.0)), ModelVolumeType::ModelPart, true);
    o.add_volume_mesh(mesh(box_its(4.0, 4.0, 6.0, 2.0, 2.0, 0.0)), ModelVolumeType::ModelPart, true).config.set("extruder", 3);
    o.add_volume_mesh(mesh(box_its(3.0, 3.0, 3.0, -1.0, -1.0, 1.5)), ModelVolumeType::NegativeVolume, true);
}
/// gen.cpp `add_holed(o)`
fn add_holed(o: &mut ModelObject) {
    o.add_volume_mesh(mesh(box_its(20.0, 20.0, 4.0, -10.0, -10.0, 0.0)), ModelVolumeType::ModelPart, true);
    o.add_volume_mesh(
        mesh(moved(its_make_cylinder(3., 10., 2. * std::f64::consts::PI / 24.), 1.0, 0.5, -1.0)),
        ModelVolumeType::NegativeVolume,
        true,
    );
}

fn range_key(lo: f64, hi: f64) -> (ordered_float::OrderedFloat<f64>, ordered_float::OrderedFloat<f64>) {
    (ordered_float::OrderedFloat(lo), ordered_float::OrderedFloat(hi))
}

/// gen.cpp `build_model(scenario)`
fn build_model(scenario: i32) -> Model {
    let mut model = Model::new();
    fn object<'a>(model: &'a mut Model, name: &str) -> &'a mut ModelObject {
        let o = model.add_object_new();
        o.name = name.into();
        o
    }
    match scenario {
        0 => {
            let o = object(&mut model, "cube");
            o.add_volume_mesh(mesh(box_its(20.0, 20.0, 6.0, -10.0, -10.0, 0.0)), ModelVolumeType::ModelPart, true);
            o.add_instance().set_offset(&Vec3d::new(100., 100., 0.));
        }
        1 | 14 => {
            let o = object(&mut model, "mods");
            o.config.set("extruder", 2);
            o.add_volume_mesh(mesh(box_its(10.0, 12.0, 6.0, -5.0, -6.0, 0.0)), ModelVolumeType::ModelPart, true).config.set("wall_filament", 3);
            {
                let v1 = o.add_volume_mesh(mesh(box_its(4.0, 4.0, 4.0, 0.0, 0.0, 2.0)), ModelVolumeType::ParameterModifier, true);
                v1.config.set("sparse_infill_filament", 1);
                v1.set_offset(&Vec3d::new(1., -1., 0.5));
            }
            o.add_volume_mesh(mesh(box_its(3.0, 3.0, 3.0, 20.0, 20.0, 0.0)), ModelVolumeType::ParameterModifier, true).config.set("wall_loops", 5);
            o.add_volume_mesh(mesh(box_its(2.0, 2.0, 10.0, -1.0, -1.0, 0.0)), ModelVolumeType::NegativeVolume, true);
            {
                let i0 = o.add_instance();
                i0.set_offset(&Vec3d::new(60., 60., 0.));
                if scenario == 14 {
                    i0.set_mirror(&Vec3d::new(-1., 1., 1.));
                }
            }
            {
                let i1 = o.add_instance();
                i1.set_offset(&Vec3d::new(80., 40., 0.));
                i1.set_rotation(&Vec3d::new(0., 0., 0.5));
                if scenario == 14 {
                    i1.set_mirror(&Vec3d::new(1., -1., 1.));
                }
            }
        }
        2 | 3 | 7 | 8 | 20 => {
            let o = object(&mut model, "multi");
            add_multi(o);
            o.add_instance().set_offset(&Vec3d::new(120., 80., 0.));
        }
        4 | 18 => {
            {
                let o = object(&mut model, "prism");
                o.add_volume_mesh(mesh(its_make_prism(20., 10., 8.)), ModelVolumeType::ModelPart, true);
                let i = o.add_instance();
                i.set_offset(&Vec3d::new(70., 70., 0.));
                i.set_rotation(&Vec3d::new(0., 0., 0.3));
            }
            {
                let o2 = object(&mut model, "cylinder");
                o2.add_volume_mesh(mesh(its_make_cylinder(4., 5., 2. * std::f64::consts::PI / 20.)), ModelVolumeType::ModelPart, true);
                o2.add_instance().set_offset(&Vec3d::new(110., 70., 0.));
            }
            {
                let o3 = object(&mut model, "sphere");
                o3.add_volume_mesh(mesh(moved(its_make_sphere(5., 2. * std::f64::consts::PI / 16.), 0.0, 0.0, 5.0)), ModelVolumeType::ModelPart, true);
                let i3 = o3.add_instance();
                i3.set_offset(&Vec3d::new(70., 110., 0.));
                i3.set_rotation(&Vec3d::new(0., 0., 1.1));
            }
            {
                let o4 = object(&mut model, "pyramid");
                o4.add_volume_mesh(mesh(its_make_pyramid(12., 7.)), ModelVolumeType::ModelPart, true);
                o4.add_instance().set_offset(&Vec3d::new(110., 110., 0.));
            }
        }
        5 | 6 | 15 | 17 => {
            let o = object(&mut model, "holed");
            add_holed(o);
            o.add_instance().set_offset(&Vec3d::new(90., 90., 0.));
        }
        9 => {
            let o = object(&mut model, "needle");
            o.add_volume_mesh(mesh(box_its(0.1, 0.1, 0.6, -0.05, -0.05, 0.0)), ModelVolumeType::ModelPart, true);
            o.add_volume_mesh(mesh(box_its(10.0, 10.0, 3.0, -5.0, -5.0, 0.6)), ModelVolumeType::ModelPart, true);
            o.add_instance().set_offset(&Vec3d::new(100., 100., 0.));
        }
        10 => {
            let o = object(&mut model, "floating");
            o.add_volume_mesh(mesh(box_its(10.0, 10.0, 3.0, -5.0, -5.0, 1.5)), ModelVolumeType::ModelPart, true);
            o.add_instance().set_offset(&Vec3d::new(100., 100., 0.));
        }
        11 => {
            let o = object(&mut model, "enforced");
            o.add_volume_mesh(mesh(box_its(10.0, 10.0, 4.0, -5.0, -5.0, 0.0)), ModelVolumeType::ModelPart, true);
            o.add_volume_mesh(mesh(box_its(4.0, 4.0, 2.0, -2.0, -2.0, 0.0)), ModelVolumeType::SupportEnforcer, true);
            o.add_volume_mesh(mesh(box_its(4.0, 4.0, 2.0, 0.0, 0.0, 1.0)), ModelVolumeType::SupportEnforcer, true);
            o.add_volume_mesh(mesh(box_its(3.0, 3.0, 3.0, 3.0, 3.0, 0.0)), ModelVolumeType::SupportBlocker, true);
            o.add_instance().set_offset(&Vec3d::new(100., 100., 0.));
        }
        12 => {
            let o = object(&mut model, "shrink");
            add_multi(o);
            o.add_volume_mesh(mesh(box_its(4.0, 4.0, 2.0, -2.0, -2.0, 0.0)), ModelVolumeType::SupportEnforcer, true);
            o.add_volume_mesh(mesh(box_its(4.0, 4.0, 2.0, 0.0, 0.0, 1.0)), ModelVolumeType::SupportEnforcer, true);
            o.add_instance().set_offset(&Vec3d::new(120., 80., 0.));
        }
        13 => {
            let o = object(&mut model, "ranges");
            o.add_volume_mesh(mesh(box_its(15.0, 15.0, 6.0, -7.5, -7.5, 0.0)), ModelVolumeType::ModelPart, true);
            o.add_volume_mesh(mesh(box_its(6.0, 6.0, 5.0, 4.0, 4.0, 0.0)), ModelVolumeType::ModelPart, true).config.set("extruder", 2);
            o.add_volume_mesh(mesh(box_its(5.0, 5.0, 6.0, -6.0, -6.0, 0.0)), ModelVolumeType::ParameterModifier, true).config.set("wall_loops", 4);
            let mut ctxt = ConfigSubstitutionContext::new(ForwardCompatibilitySubstitutionRule::Disable);
            {
                let r = o.layer_config_ranges.entry(range_key(1.0, 2.0)).or_default();
                r.set("extruder", 3);
                r.set_deserialize("layer_height", "0.2", &mut ctxt, false).unwrap();
            }
            {
                let r = o.layer_config_ranges.entry(range_key(3.0, 4.0)).or_default();
                r.set("wall_filament", 2);
                r.set_deserialize("layer_height", "0.2", &mut ctxt, false).unwrap();
            }
            o.add_instance().set_offset(&Vec3d::new(150., 50., 0.));
        }
        16 => {
            let o = object(&mut model, "evenodd");
            o.add_volume_mesh(mesh(box_its(12.0, 12.0, 4.0, -6.0, -6.0, 0.0)), ModelVolumeType::ModelPart, true);
            o.add_volume_mesh(mesh(box_its(12.0, 12.0, 4.0, 0.0, 0.0, 0.0)), ModelVolumeType::ModelPart, true);
            o.add_volume_mesh(mesh(box_its(4.0, 4.0, 4.0, -4.0, -4.0, 0.0)), ModelVolumeType::ModelPart, true);
            o.add_instance().set_offset(&Vec3d::new(100., 100., 0.));
        }
        24 | 25 => {
            let o = object(&mut model, "diameters");
            {
                let v = o.add_volume_mesh(mesh(box_its(10.0, 10.0, 4.0, -5.0, -5.0, 0.0)), ModelVolumeType::ModelPart, true);
                v.mmu_segmentation_facets.set_triangle_from_string(2, "8");
                v.mmu_segmentation_facets.set_triangle_from_string(3, "8");
                v.mmu_segmentation_facets.set_triangle_from_string(4, "0C");
                v.mmu_segmentation_facets.set_triangle_from_string(7, "8");
                v.mmu_segmentation_facets.set_triangle_from_string(10, "0C");
            }
            {
                let v2 = o.add_volume_mesh(mesh(box_its(6.0, 6.0, 3.0, 2.0, 2.0, 0.0)), ModelVolumeType::ModelPart, true);
                v2.config.set("extruder", 2);
                v2.mmu_segmentation_facets.set_triangle_from_string(5, "0C");
            }
            o.add_instance().set_offset(&Vec3d::new(100., 100., 0.));
        }
        19 => {
            let o = object(&mut model, "painted");
            {
                let v = o.add_volume_mesh(mesh(box_its(10.0, 10.0, 4.0, -5.0, -5.0, 0.0)), ModelVolumeType::ModelPart, true);
                v.mmu_segmentation_facets.set_triangle_from_string(2, "8");
                v.mmu_segmentation_facets.set_triangle_from_string(3, "8");
                v.mmu_segmentation_facets.set_triangle_from_string(4, "C");
                v.mmu_segmentation_facets.set_triangle_from_string(7, "8");
                v.mmu_segmentation_facets.set_triangle_from_string(10, "C");
            }
            o.add_instance().set_offset(&Vec3d::new(100., 100., 0.));
        }
        21 => {
            let o = object(&mut model, "fuzzy");
            {
                let v = o.add_volume_mesh(mesh(box_its(10.0, 10.0, 4.0, -5.0, -5.0, 0.0)), ModelVolumeType::ModelPart, true);
                v.fuzzy_skin_facets.set_triangle_from_string(4, "4");
                v.fuzzy_skin_facets.set_triangle_from_string(5, "4");
                v.fuzzy_skin_facets.set_triangle_from_string(8, "4");
            }
            o.add_instance().set_offset(&Vec3d::new(100., 100., 0.));
        }
        22 | 23 => {
            let o = object(&mut model, "painted2");
            {
                let v = o.add_volume_mesh(mesh(box_its(10.0, 10.0, 4.0, -5.0, -5.0, 0.0)), ModelVolumeType::ModelPart, true);
                v.mmu_segmentation_facets.set_triangle_from_string(2, "8");
                v.mmu_segmentation_facets.set_triangle_from_string(3, "8");
                v.mmu_segmentation_facets.set_triangle_from_string(6, "4");
                v.mmu_segmentation_facets.set_triangle_from_string(9, "C");
                v.fuzzy_skin_facets.set_triangle_from_string(4, "4");
                v.fuzzy_skin_facets.set_triangle_from_string(7, "4");
            }
            {
                let v2 = o.add_volume_mesh(mesh(box_its(6.0, 6.0, 3.0, 2.0, 2.0, 0.0)), ModelVolumeType::ModelPart, true);
                v2.config.set("extruder", 2);
                v2.mmu_segmentation_facets.set_triangle_from_string(5, "4");
                v2.fuzzy_skin_facets.set_triangle_from_string(10, "4");
            }
            o.add_instance().set_offset(&Vec3d::new(100., 100., 0.));
        }
        _ => {}
    }
    model
}

/// The Print `Print::apply(model, config)` makes on a fresh Print, as tests/print_native_parity.rs
/// assembles it.
fn assemble(model: &Model, config: &DynamicPrintConfig) -> Print {
    let mut print = Print::new();
    let mut new_full_config = config.clone();
    // PrintApply.cpp:1305-1307
    for key in ["print_settings_id", "filament_settings_id", "printer_settings_id"] {
        let _ = new_full_config.option_mut(key, true);
    }
    // PrintApply.cpp:1323-1325: no objects on the fresh Print, no used filaments.
    new_full_config.normalize_fdm_1().unwrap();
    let mut changed_keys_ori_values = slicer::config::DynamicConfig::new();
    let _ = new_full_config.normalize_fdm_2(0, 0, Some(&mut changed_keys_ori_values)).unwrap();
    // PrintApply.cpp:1438-1456: one extruder.
    let mut print_variant_index: Vec<i32> = vec![0];
    print.base.base.m_ori_full_print_config = new_full_config.clone();
    let filament_maps: Vec<i32> = new_full_config
        .option("filament_map")
        .and_then(|o| o.as_any().downcast_ref::<ConfigOptionInts>())
        .map(|o| o.values.clone())
        .unwrap_or_else(|| vec![1]);
    // PrintApply.cpp:1495-1506
    let mut filament_overrides = DynamicPrintConfig::new();
    let print_diff = print_config_diffs(&print.m_config, &new_full_config, &mut filament_overrides, 0, &filament_maps);
    let full_config_diff = full_print_config_diffs(&print.base.base.m_full_print_config, &new_full_config, 0);
    let object_diff = print.m_default_object_config.diff(&new_full_config).unwrap();
    let region_diff = print.m_default_region_config.diff(&new_full_config).unwrap();
    // PrintApply.cpp:1644-1678
    if !full_config_diff.is_empty() {
        let _ = print.m_config.apply_only(&new_full_config, &print_diff, true);
        let keys = filament_overrides.keys();
        let _ = print.m_config.apply_only(&filament_overrides, &keys, true);
        let _ = print.m_default_object_config.apply_only(&new_full_config, &object_diff, true);
        let _ = print.m_default_region_config.apply_only(&new_full_config, &region_diff, true);
        print.base.base.m_full_print_config = new_full_config.clone();
        print.update_filament_self_index_cache();
    }
    let num_extruders = print.m_config.option("filament_diameter").and_then(|o| o.as_vector_base()).map(|v| v.size()).unwrap_or(0);
    print.m_model = model.clone();
    // PrintApply.cpp:1958-1990: the PrintObjects of every ModelObject, in the order of the model.
    let mut statuses: Vec<(usize, Vec<PrintObjectTrafoAndInstances>)> = Vec::new();
    for (mo_idx, model_object) in print.m_model.objects.iter().enumerate() {
        let print_instances = print_objects_from_model_object(model_object);
        statuses.push((mo_idx, print_instances));
    }
    for (mo_idx, print_instances) in &statuses {
        let model_object = &print.m_model.objects[*mo_idx];
        let config = object_config_from_model_object(&print.m_default_object_config, model_object, num_extruders, &mut print_variant_index);
        for pi in print_instances {
            // PrintApply.cpp:1983 `new PrintObject(this, model_object, trafo, std::move(instances))`
            let mut po = PrintObject::new(&mut print, *mo_idx, pi.trafo, pi.instances.clone());
            po.m_config = config.clone();
            print.m_objects.push(po);
        }
    }
    let n_objects = print.m_objects.len();
    for i in 0..n_objects {
        for inst in print.m_objects[i].m_instances.iter_mut() {
            inst.print_object = i;
        }
    }
    // PrintApply.cpp:2116-2272: the regions, shared by the PrintObjects of one ModelObject.
    let mut i = 0;
    while i < n_objects {
        let mo_idx = print.m_objects[i].m_model_object;
        let mut end = i + 1;
        while end < n_objects && print.m_objects[end].m_model_object == mo_idx {
            end += 1;
        }
        let model_object = &print.m_model.objects[mo_idx];
        let trafo = statuses.iter().find(|s| s.0 == mo_idx).unwrap().1[0].trafo;
        let xy = *print.m_objects[i].m_config.opt_float("xy_contour_compensation") as f32;
        // PrintApply.cpp:2132-2139: every filament when a volume of the object is painted.
        let mut painting_extruders: Vec<u32> = Vec::new();
        if num_extruders > 1 && model_object.volumes.iter().any(|v| !v.mmu_segmentation_facets.is_empty()) {
            painting_extruders = (1..=num_extruders as u32).collect();
        }
        let regions = generate_print_object_regions(
            None,
            model_object,
            Some(&print.m_model),
            &LayerRanges::new(&model_object.layer_config_ranges),
            &print.m_default_region_config,
            &trafo,
            num_extruders,
            if model_object.is_mm_painted() { 0.0 } else { xy },
            &painting_extruders,
            &mut print_variant_index,
            model_object.is_fuzzy_skin_painted(),
            &[],
        );
        for k in i..end {
            regions.borrow_mut().ref_cnt_inc();
            print.m_objects[k].m_shared_regions = Some(regions.clone());
        }
        i = end;
    }
    print.update_print_regions_from_objects();
    // PrintApply.cpp:2053-2090: the config checked again with the objects in place.
    let new_used_filaments = print.extruders(true).len() as i32;
    let new_changed_keys =
        new_full_config.normalize_fdm_2(print.m_objects.len() as i32, new_used_filaments, Some(&mut changed_keys_ori_values)).unwrap();
    if !new_changed_keys.is_empty() {
        print.invalidate_state_by_config_options(&new_changed_keys);
        print.invalidate_step(slicer::print::faithful::PrintStep::psGCodeExport);
        let _ = print.m_config.apply_only(&new_full_config, &new_changed_keys, true);
        let _ = print.m_default_object_config.apply_only(&new_full_config, &new_changed_keys, true);
        let _ = print.m_default_region_config.apply_only(&new_full_config, &new_changed_keys, true);
        print.base.base.m_full_print_config = new_full_config.clone();
        print.update_filament_self_index_cache();
    }
    // PrintApply.cpp:2294-2306: the single-diameter layering nozzle (0 off multi-diameter mode).
    let single_nozzle = print.resolve_single_diameter_layering_nozzle();
    if (single_nozzle - print.m_single_diameter_layering_nozzle).abs() > slicer::libslic3r::EPSILON {
        print.m_single_diameter_layering_nozzle = single_nozzle;
    }
    // PrintApply.cpp:2326-2331
    for i in 0..print.m_objects.len() {
        PrintObject::update_slicing_parameters(&mut print, i);
        print.m_support_used |= print.m_objects[i].m_config.opt_bool("enable_support");
    }
    print
}

/// gen.cpp `volume_index(mo, id)`
fn volume_index(mo: &ModelObject, id: ObjectID) -> i64 {
    mo.volumes.iter().position(|v| v.id() == id).map(|i| i as i64).unwrap_or(-1)
}

/// gen.cpp `dump_volume_slices(tag, mo, vss)`
fn dump_volume_slices(r: &mut Run, tag: &str, mo: &ModelObject, vss: &[VolumeSlices]) {
    r.out(format!("{tag} {}", vss.len()));
    for vs in vss {
        let body: Vec<String> = vs.slices.iter().map(|e| expolys_s(e)).collect();
        r.outb(format!("{tag} volume {} {}", volume_index(mo, vs.volume_id), vs.slices.len()), body.join(" "));
    }
}

/// gen.cpp `dump_by_layer(tag, layers)`
fn dump_by_layer(r: &mut Run, tag: &str, layers: &[ExPolygons]) {
    let body: Vec<String> = layers.iter().map(|e| expolys_s(e)).collect();
    r.outb(format!("{tag} {}", layers.len()), body.join(" "));
}

/// gen.cpp `dump_layers(tag, layers)`
fn dump_layers(r: &mut Run, tag: &str, layers: &[Layer]) {
    r.out(format!("{tag} layers {}", layers.len()));
    for (i, l) in layers.iter().enumerate() {
        let mut body = format!("lslices {} bboxes {}", expolys_s(&l.lslices), l.lslices_bboxes.len());
        for b in &l.lslices_bboxes {
            body += &format!(" {}", bbox_s(b));
        }
        body += &format!(" regions {}", l.regions().len());
        for region in l.regions() {
            let raw: ExPolygons = region.raw_slices.surfaces.iter().map(|s| s.expolygon.clone()).collect();
            body += &format!(" slices {} raw {}", surfaces_s(&region.slices.surfaces), expolys_s(&raw));
        }
        let link = |id: Option<usize>| id.map(|k| layers[k].id() as i64).unwrap_or(-1);
        r.outb(
            format!(
                "{tag} layer {i} {} {} {} {} {} {} {}",
                l.id(),
                hd(l.print_z),
                hd(l.height),
                hd(l.slice_z),
                hb(l.slicing_errors),
                link(l.lower_layer_id),
                link(l.upper_layer_id)
            ),
            body,
        );
    }
}

/// gen.cpp `dump_groups(tag, mo, groups)`
fn dump_groups(r: &mut Run, tag: &str, mo: &ModelObject, groups: &[groupedVolumeSlices]) {
    r.out(format!("{tag} {}", groups.len()));
    for g in groups {
        let mut head = format!("{tag} group {} {}", g.groupId, g.volume_ids.len());
        for id in &g.volume_ids {
            head += &format!(" {}", volume_index(mo, *id));
        }
        r.outb(head, expolys_s(&g.slices));
    }
}

/// What the two segmentations by painting returned in the C++ (the generator's hooks).
struct Cores {
    mms: Option<Vec<Vec<ExPolygons>>>,
    fuzzy: Option<Vec<Vec<ExPolygons>>>,
}

impl SliceVolumesCores for Cores {
    fn multi_material_segmentation_by_painting(&mut self, _print: &Print, _print_object: &PrintObject) -> Vec<Vec<ExPolygons>> {
        self.mms.take().expect("the vectors have no multi-material segmentation for this object")
    }
    fn fuzzy_skin_segmentation_by_painting(&mut self, _print: &Print, _print_object: &PrintObject) -> Vec<Vec<ExPolygons>> {
        self.fuzzy.take().expect("the vectors have no fuzzy skin segmentation for this object")
    }
    fn generate_interlocking_structure(&mut self, _print: &Print, _print_object: &mut PrintObject) {}
    fn generate_embedding_wall(&mut self, _print: &Print, _print_object: &mut PrintObject) {}
}

/// gen.cpp `dump_segmentation(what, segmentation)`, read back.
fn read_segmentation(r: &mut Run, tag: &str, what: &str) -> Option<Vec<Vec<ExPolygons>>> {
    let line = r.input(&format!("in {tag} {what}"))?;
    let n = Tok::new(&line).i();
    let mut segmentation = Vec::new();
    for _ in 0..n {
        let line = r.input(&format!("in {tag} {what}_layer")).expect("a layer of the segmentation");
        let mut t = Tok::new(&line);
        let m = t.i();
        segmentation.push((0..m).map(|_| t.expolys()).collect());
    }
    Some(segmentation)
}

fn run_scenario(r: &mut Run, scenario: i32) {
    let s = format!("s{scenario}");
    let model = build_model(scenario);
    let mut config = DynamicPrintConfig::full_print_config();
    config.set_key_value("filament_diameter", Box::new(ConfigOptionFloats::from_values(vec![1.75, 1.75, 1.75])));
    config.set_key_value(
        "filament_colour",
        Box::new(ConfigOptionStrings::from_values(vec!["#FF0000".into(), "#00FF00".into(), "#0000FF".into()])),
    );
    config.set_key_value("filament_map", Box::new(ConfigOptionInts::from_values(vec![1, 1, 1])));
    for (key, value) in scenario_config(scenario) {
        deser(&mut config, key, value);
    }
    let mut print = assemble(&model, &config);
    r.out(format!("{s} objects {} {}", print.m_objects.len(), hb(print.uses_multiple_nozzle_diameters())));
    let no_cancel = || -> Result<(), CanceledException> { Ok(()) };

    for obj_idx in 0..print.m_objects.len() {
        let mut obj = print.m_objects.remove(obj_idx);
        let t = format!("{s} o{obj_idx}");
        {
            let mo = obj.model_object(&print);

            // new_layers over the layers `PrintObject::slice()` makes.
            let mut layer_height_profile: Vec<f64> = Vec::new();
            let mut nozzle_range_reset = false;
            PrintObject::update_layer_height_profile(mo, &obj.m_slicing_params, &mut layer_height_profile, &mut nozzle_range_reset);
            let layers = new_layers(
                &obj,
                &slicer::slicing::generate_object_layers(&obj.m_slicing_params, &layer_height_profile, obj.m_config.opt_bool("precise_z_height")),
            );
            {
                let mut body = String::new();
                let link = |id: Option<usize>| id.map(|k| layers[k].id() as i64).unwrap_or(-1);
                for l in &layers {
                    body += &format!(
                        " {} {} {} {} {} {}",
                        l.id(),
                        hd(l.print_z),
                        hd(l.height),
                        hd(l.slice_z),
                        link(l.lower_layer_id),
                        link(l.upper_layer_id)
                    );
                }
                r.outb(format!("{t} new_layers {}", layers.len()), body);
            }
            let zs: Vec<f32> = zs_from_layers(&layers);
            drop(layers);

            // slice_volumes_inner
            let shared = obj.m_shared_regions.clone().expect("shared regions");
            let by_volume = slice_volumes_inner(
                print.config(),
                obj.config(),
                &obj.trafo_centered(),
                &mo.volumes,
                &shared.borrow().layer_ranges,
                &zs,
                &no_cancel,
                Some(&print),
            )
            .unwrap();
            dump_volume_slices(r, &format!("{t} slice_volumes_inner"), mo, &by_volume);

            // slice_volume of every volume: all of the layers, every third layer, and layer ranges.
            {
                let mut params = MeshSlicingParamsEx::default();
                params.trafo = obj.trafo_centered();
                params.closing_radius = 0.049;
                let zs3: Vec<f32> = zs.iter().step_by(3).copied().collect();
                let top: f64 = if zs.is_empty() { 0. } else { zs[zs.len() - 1] as f64 };
                let ranges1 = vec![(0., top + 1.)];
                let ranges2 = vec![(0.3, 0.9), (1.1, 1.1 + 0.25 * top), (0.75 * top, top)];
                let ranges3 = vec![(top + 5., top + 6.)];
                for (iv, mv) in mo.volumes.iter().enumerate() {
                    let v = format!("{t} slice_volume {iv}");
                    r.out(format!("{v} needs_slicing {}", hb(model_volume_needs_slicing(mv))));
                    dump_by_layer(r, &format!("{v} all"), &slice_volume(mv, &zs, &params, &no_cancel).unwrap());
                    dump_by_layer(r, &format!("{v} third"), &slice_volume(mv, &zs3, &params, &no_cancel).unwrap());
                    dump_by_layer(r, &format!("{v} ranges1"), &slice_volume_ranges(mv, &zs, &ranges1, &params, &no_cancel).unwrap());
                    dump_by_layer(r, &format!("{v} ranges2"), &slice_volume_ranges(mv, &zs, &ranges2, &params, &no_cancel).unwrap());
                    dump_by_layer(r, &format!("{v} ranges3"), &slice_volume_ranges(mv, &zs, &ranges3, &params, &no_cancel).unwrap());
                }
            }

            // slices_to_regions: the four combinations of its two flags.
            for flags in 0..4 {
                let by_region =
                    slices_to_regions(&mo.volumes, &shared.borrow(), &zs, by_volume.clone(), (flags & 1) != 0, (flags & 2) != 0, &no_cancel).unwrap();
                r.out(format!("{t} slices_to_regions {flags} {}", by_region.len()));
                for (k, layers) in by_region.iter().enumerate() {
                    dump_by_layer(r, &format!("{t} slices_to_regions {flags} region {k}"), layers);
                }
            }
        }

        // The step.
        let mut cores = Cores { mms: read_segmentation(r, &t, "mms"), fuzzy: read_segmentation(r, &t, "fuzzy") };
        let result = obj.slice(&print, &mut cores);
        if let Err(e) = &result {
            eprintln!("{t}: slice: {e}");
        }
        r.out(format!("{t} slice {} {}", hb(result.is_ok()), hb(obj.m_state.is_done_unguarded(PrintObjectStep::posSlice))));
        dump_layers(r, &t, &obj.m_layers);
        {
            let mo = obj.model_object(&print);
            dump_volume_slices(r, &format!("{t} first_layer_by_volume"), mo, &obj.firstLayerObjSliceByVolume);
            dump_groups(r, &format!("{t} first_layer_groups"), mo, &obj.firstLayerObjSliceByGroups);
        }

        // What reads the sliced object.
        r.out(format!("{t} shrink {} {}", hd(obj.support_shrinkage_scale(&print)), hb(should_skip_filament_shrink(&print, &obj))));
        for type_ in [ModelVolumeType::SupportEnforcer, ModelVolumeType::SupportBlocker] {
            let slices: Vec<Polygons> = obj.slice_support_volumes(&print, type_).unwrap();
            let body: Vec<String> = slices.iter().map(|p| polys_s(p)).collect();
            r.outb(format!("{t} slice_support_volumes {} {}", type_ as i32, slices.len()), body.join(" "));
        }

        // normalize_mm_segmentation_to_layer_bands over the synthetic segmentations of the vectors.
        let mut c = 0;
        while let Some(mut segmentation) = read_segmentation(r, &t, "normalize") {
            normalize_mm_segmentation_to_layer_bands(&print, &obj, &mut segmentation, &no_cancel).unwrap();
            for (l, layer) in segmentation.iter().enumerate() {
                let body: Vec<String> = layer.iter().map(|e| expolys_s(e)).collect();
                r.outb(format!("{t} normalized {c} layer {l}"), body.join(" "));
            }
            c += 1;
        }
        print.m_objects.insert(obj_idx, obj);
    }
}

/// gen.cpp `run_synthetic()`: the helpers on the inputs the vectors carry.
fn run_synthetic(r: &mut Run) {
    // overlap_in_xy
    while let Some(line) = r.input("in overlap_in_xy") {
        let mut t = Tok::new(&line);
        let mut f3 = || [t.f(), t.f(), t.f()];
        let l = AlignedBox3f { min: f3(), max: f3() };
        let rr = AlignedBox3f { min: f3(), max: f3() };
        r.out(format!("overlap_in_xy {}", hb(overlap_in_xy(&l, &rr))));
    }

    // layer_range_first, layer_range_next
    while let Some(line) = r.input("in layer_ranges") {
        let mut t = Tok::new(&line);
        let n = t.i();
        let ranges: Vec<LayerRangeRegions> = (0..n)
            .map(|_| {
                let mut lr = LayerRangeRegions::default();
                lr.layer_height_range = (t.d(), t.d());
                lr
            })
            .collect();
        loop {
            if let Some(line) = r.input("in layer_range_first") {
                let z = Tok::new(&line).d();
                r.out(format!("layer_range_first {}", layer_range_first(&ranges, z)));
            } else if let Some(line) = r.input("in layer_range_next") {
                let mut t = Tok::new(&line);
                let from = t.i() as usize;
                let z = t.d();
                r.out(format!("layer_range_next {}", layer_range_next(&ranges, from, z)));
            } else {
                break;
            }
        }
    }

    // groupingVolumes
    while let Some(line) = r.input("in grouping") {
        let mut t = Tok::new(&line);
        let n = t.i();
        let resolution = t.d();
        let replaced = t.i() as i32;
        let mut vss: Vec<VolumeSlices> = Vec::new();
        for _ in 0..n {
            let line = r.input("in grouping_volume").expect("a volume of the grouping");
            let mut t = Tok::new(&line);
            let id = t.i() as usize;
            let nl = t.i();
            vss.push(VolumeSlices { volume_id: ObjectID::new(id), slices: (0..nl).map(|_| t.expolys()).collect() });
        }
        let mut groups: Vec<groupedVolumeSlices> = Vec::new();
        let ok = groupingVolumes(vss, &mut groups, resolution, replaced);
        r.out(format!("grouping ok {}", hb(ok)));
        r.out(format!("grouping {}", groups.len()));
        for g in &groups {
            let mut head = format!("grouping group {} {}", g.groupId, g.volume_ids.len());
            for id in &g.volume_ids {
                head += &format!(" {}", id.id);
            }
            r.outb(head, expolys_s(&g.slices));
        }
    }

    // findPartVolumes, groupingVolumes, applyNegtiveVolumes and reGroupingLayerPolygons over the volumes
    // of an object.
    {
        let mut model = Model::new();
        {
            let o = model.add_object_new();
            for type_ in [
                ModelVolumeType::ModelPart,
                ModelVolumeType::NegativeVolume,
                ModelVolumeType::ModelPart,
                ModelVolumeType::ParameterModifier,
                ModelVolumeType::ModelPart,
                ModelVolumeType::NegativeVolume,
                ModelVolumeType::SupportEnforcer,
                ModelVolumeType::ModelPart,
            ] {
                o.add_volume_mesh(mesh(box_its(1.0, 1.0, 1.0, 0.0, 0.0, 0.0)), type_, true);
            }
        }
        let o = &model.objects[0];
        while let Some(line) = r.input("in brim_groups") {
            let mut t = Tok::new(&line);
            let n = t.i();
            let resolution = t.d();
            let replaced = t.i() as i32;
            let mut vss: Vec<VolumeSlices> = Vec::new();
            for _ in 0..n {
                let line = r.input("in brim_groups_volume").expect("a volume of the brim groups");
                let mut t = Tok::new(&line);
                let idx = t.i() as usize;
                let nl = t.i();
                vss.push(VolumeSlices { volume_id: o.volumes[idx].id(), slices: (0..nl).map(|_| t.expolys()).collect() });
            }
            let parts = findPartVolumes(&vss, &o.volumes);
            {
                let mut line = format!("brim_groups parts {}", parts.len());
                for vs in &parts {
                    line += &format!(" {}", volume_index(o, vs.volume_id));
                }
                r.out(line);
            }
            let mut groups: Vec<groupedVolumeSlices> = Vec::new();
            groupingVolumes(parts, &mut groups, resolution, replaced);
            dump_groups(r, "brim_groups grouped", o, &groups);
            applyNegtiveVolumes(&o.volumes, &vss, &mut groups, resolution);
            dump_groups(r, "brim_groups negative", o, &groups);
            let line = r.input("in brim_groups_eps").expect("the islands of the layer");
            let eps = Tok::new(&line).expolys();
            reGroupingLayerPolygons(&mut groups, &eps, resolution);
            dump_groups(r, "brim_groups regrouped", o, &groups);
        }
    }

    // PrintObject::_shrink_contour_holes, on the object of the first scenario.
    {
        let model = build_model(0);
        let config = DynamicPrintConfig::full_print_config();
        let print = assemble(&model, &config);
        let obj = &print.m_objects[0];
        while let Some(line) = r.input("in shrink_contour_holes") {
            let mut t = Tok::new(&line);
            let contour_delta = t.d();
            let hole_delta = t.d();
            let polys = t.expolys();
            r.outb("shrink_contour_holes".into(), expolys_s(&obj._shrink_contour_holes(contour_delta, hole_delta, &polys)));
        }
    }
}

fn read_vectors() -> Vec<String> {
    // A diagnostic: a file of the generator with every line in full.
    if let Ok(path) = std::env::var("PRINT_OBJECT_SLICE_PARITY_VECTORS") {
        let text = std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
        return text.lines().map(|l| l.to_string()).collect();
    }
    let path = format!("{}/tests/data/print_object_slice/vectors.txt.gz", env!("CARGO_MANIFEST_DIR"));
    let file = std::fs::File::open(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
    let mut text = String::new();
    flate2::read::GzDecoder::new(file).read_to_string(&mut text).unwrap_or_else(|e| panic!("{path}: {e}"));
    text.lines().map(|l| l.to_string()).collect()
}

#[test]
fn print_object_slice_native_parity() {
    let mut r = Run { expected: read_vectors(), pos: 0, lines: Vec::new(), surplus: 0, full: false };
    r.full = r.input("in mode").expect("mode") == "full";
    // A diagnostic: the scenarios PRINT_OBJECT_SLICE_PARITY_ONLY names ("3" or "3,7"), of a file the
    // generator wrote for those scenarios.
    let only: Option<Vec<i32>> =
        std::env::var("PRINT_OBJECT_SLICE_PARITY_ONLY").ok().map(|v| v.split(',').filter_map(|s| s.trim().parse().ok()).collect());
    for scenario in 0..N_SCENARIOS {
        if only.as_ref().map(|o| o.contains(&scenario)).unwrap_or(true) {
            run_scenario(&mut r, scenario);
        }
    }
    if only.is_none() {
        run_synthetic(&mut r);
    }
    // A diagnostic: the lines this test produced, written where PRINT_OBJECT_SLICE_PARITY_DUMP names a file.
    if let Ok(path) = std::env::var("PRINT_OBJECT_SLICE_PARITY_DUMP") {
        let _ = std::fs::write(path, r.lines.join("\n"));
    }
    let mut mismatches = r.surplus;
    let mut by_kind: std::collections::BTreeMap<String, (usize, usize)> = std::collections::BTreeMap::new();
    for k in 0..r.expected.len().max(r.lines.len()) {
        let e = r.expected.get(k).map(|s| s.as_str()).unwrap_or("<missing>");
        let l = r.lines.get(k).map(|s| s.as_str()).unwrap_or("<missing>");
        // The kind of a line: the scenario and the stage ("s3 slices_to_regions").
        let words: Vec<&str> = e.split(' ').collect();
        let kind = if e.starts_with('s') && words.len() > 2 {
            let stage = if words[1].starts_with('o') { words[2] } else { words[1] };
            format!("{} {}", words[0], stage)
        } else {
            words[..words.len().min(2)].join(" ")
        };
        let entry = by_kind.entry(kind).or_insert((0, 0));
        entry.0 += 1;
        if e != l {
            entry.1 += 1;
            mismatches += 1;
            if mismatches <= 12 {
                let cut = |s: &str| if s.len() > 700 { format!("{}...", &s[..700]) } else { s.to_string() };
                eprintln!("line {}:\n  C++:  {}\n  Rust: {}", k + 1, cut(e), cut(l));
            }
        }
    }
    for (kind, (n, bad)) in &by_kind {
        if *bad > 0 {
            eprintln!("{kind}: {n} lines, {bad} mismatches");
        }
    }
    eprintln!("{} lines, {} mismatches", r.expected.len(), mismatches);
    // Every line has to be equal; the surplus lines (a stage that wrote more lines than the pin) count.
    assert_eq!(mismatches, 0);
}

