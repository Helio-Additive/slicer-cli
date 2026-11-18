//! Shared by the native parity tests of the support units (support_parameters, support_common,
//! support_material): the formatting of the vectors, their reader, and `Print::apply` on a fresh Print as
//! tests/print_native_parity.rs and tests/tree_support_native_parity.rs assemble it.
#![allow(dead_code)]

use slicer::config::{ConfigBase, ConfigOptionInts, ConfigOptionResolver, ConfigSubstitutionContext, ForwardCompatibilitySubstitutionRule};
use slicer::geometry::{ExPolygon, ExPolygons, Point, Polygon};
use slicer::model::*;
use slicer::normal_utils::indexed_triangle_set;
use slicer::print::faithful::{Print, PrintObject};
use slicer::print_apply::*;
use slicer::print_config::config_def::DynamicPrintConfig;
use slicer::triangle_mesh::{its_make_cube, Vec3f};

pub fn hd(v: f64) -> String {
    format!("{:016x}", v.to_bits())
}
pub fn hf(v: f32) -> String {
    format!("{:08x}", v.to_bits())
}
pub fn hb(v: bool) -> &'static str {
    if v {
        "1"
    } else {
        "0"
    }
}
pub fn pt(p: &Point) -> String {
    format!("{} {}", p.x, p.y)
}
pub fn poly_s(p: &Polygon) -> String {
    let mut o = format!("{}", p.points.len());
    for q in &p.points {
        o += &format!(" {}", pt(q));
    }
    o
}
pub fn polys_s(ps: &[Polygon]) -> String {
    let mut o = format!("{}", ps.len());
    for p in ps {
        o += &format!(" {}", poly_s(p));
    }
    o
}
pub fn expoly_s(e: &ExPolygon) -> String {
    let mut o = format!("{} {}", poly_s(&e.contour), e.holes.len());
    for h in &e.holes {
        o += &format!(" {}", poly_s(h));
    }
    o
}
pub fn expolys_s(es: &[ExPolygon]) -> String {
    let mut o = format!("{}", es.len());
    for e in es {
        o += &format!(" {}", expoly_s(e));
    }
    o
}

/// The tokens of an input line.
pub struct Tok<'a> {
    pub t: std::str::SplitWhitespace<'a>,
}
impl<'a> Tok<'a> {
    pub fn new(s: &'a str) -> Self {
        Tok { t: s.split_whitespace() }
    }
    pub fn s(&mut self) -> &'a str {
        self.t.next().expect("token")
    }
    pub fn i(&mut self) -> i64 {
        self.s().parse().expect("integer")
    }
    pub fn d(&mut self) -> f64 {
        f64::from_bits(u64::from_str_radix(self.s(), 16).expect("double"))
    }
    pub fn b(&mut self) -> bool {
        self.s() == "1"
    }
    pub fn poly(&mut self) -> Polygon {
        let n = self.i();
        let mut p = Polygon::new();
        for _ in 0..n {
            let x = self.i();
            let y = self.i();
            p.points.push(Point::new(x, y));
        }
        p
    }
    pub fn expoly(&mut self) -> ExPolygon {
        let contour = self.poly();
        let nh = self.i();
        let mut holes = Vec::new();
        for _ in 0..nh {
            holes.push(self.poly());
        }
        ExPolygon::with_holes(contour, holes)
    }
    pub fn expolys(&mut self) -> ExPolygons {
        let n = self.i();
        (0..n).map(|_| self.expoly()).collect()
    }
}

/// The vectors, read in order: an input line is handed out and echoed, a result line is produced.
pub struct Run {
    pub expected: Vec<String>,
    pub pos: usize,
    pub lines: Vec<String>,
    pub surplus: usize,
}
impl Run {
    /// The next input line when it is the input `prefix`; it is echoed. The results produced since the
    /// input before it are aligned with the vectors: missing ones are padded, surplus ones counted.
    pub fn input(&mut self, prefix: &str) -> Option<String> {
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
    pub fn out(&mut self, s: String) {
        self.lines.push(s);
    }
}

pub fn box_its(sx: f64, sy: f64, sz: f64, ox: f32, oy: f32, oz: f32) -> indexed_triangle_set {
    let mut its = its_make_cube(sx, sy, sz);
    for v in &mut its.vertices {
        *v += Vec3f::new(ox, oy, oz);
    }
    its
}

pub fn deser(config: &mut DynamicPrintConfig, key: &str, value: &str) {
    let mut ctxt = ConfigSubstitutionContext::new(ForwardCompatibilitySubstitutionRule::Disable);
    config.set_deserialize(key, value, &mut ctxt, false).unwrap_or_else(|e| panic!("{key}={value}: {e:?}"));
}

/// `Print::apply` on a fresh Print, as tests/print_native_parity.rs assembles it.
pub fn assemble(model: &Model, config: &DynamicPrintConfig) -> Print {
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
        let regions = generate_print_object_regions(
            None,
            model_object,
            Some(&print.m_model),
            &LayerRanges::new(&model_object.layer_config_ranges),
            &print.m_default_region_config,
            &trafo,
            num_extruders,
            if model_object.is_mm_painted() { 0.0 } else { xy },
            &[],
            &mut print_variant_index,
            false,
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

impl<'a> Tok<'a> {
    pub fn f(&mut self) -> f32 {
        f32::from_bits(u32::from_str_radix(self.s(), 16).expect("float"))
    }
    pub fn polys(&mut self) -> Vec<Polygon> {
        let n = self.i();
        (0..n).map(|_| self.poly()).collect()
    }
    pub fn pt(&mut self) -> Point {
        let x = self.i();
        let y = self.i();
        Point::new(x, y)
    }
}

impl Run {
    /// The vectors of `data/<unit>/vectors.txt` (or `vectors.txt.gz`).
    pub fn load(unit: &str) -> Run {
        use std::io::Read;
        let dir = format!("{}/tests/data/{}", env!("CARGO_MANIFEST_DIR"), unit);
        let mut text = String::new();
        if let Ok(f) = std::fs::File::open(format!("{dir}/vectors.txt.gz")) {
            flate2::read::GzDecoder::new(f).read_to_string(&mut text).expect("vectors.txt.gz");
        } else {
            text = std::fs::read_to_string(format!("{dir}/vectors.txt")).expect("vectors.txt");
        }
        Run { expected: text.lines().map(|l| l.to_string()).collect(), pos: 0, lines: Vec::new(), surplus: 0 }
    }
    /// Every line produced against the vectors: the number of lines that differ, the first few printed.
    pub fn finish(&mut self) -> usize {
        while self.lines.len() < self.expected.len() {
            self.lines.push("<missing>".into());
        }
        let mut bad = self.surplus;
        if self.lines.len() > self.expected.len() {
            bad += self.lines.len() - self.expected.len();
        }
        let mut shown = 0;
        for (i, e) in self.expected.iter().enumerate() {
            if &self.lines[i] != e {
                bad += 1;
                if shown < 12 {
                    shown += 1;
                    let cut = |s: &str| if s.len() > 700 { format!("{}...", &s[..700]) } else { s.to_string() };
                    eprintln!("line {}:\n  expected {}\n  got      {}", i + 1, cut(e), cut(&self.lines[i]));
                }
            }
        }
        bad
    }
    /// The number of result lines (not inputs) of the vectors whose first tokens are `prefix`, and how many
    /// of them the run reproduced.
    pub fn tally(&self, prefix: &str) -> (usize, usize) {
        let mut n = 0;
        let mut ok = 0;
        for (i, e) in self.expected.iter().enumerate() {
            if e.starts_with(prefix) {
                n += 1;
                if self.lines.get(i) == Some(e) {
                    ok += 1;
                }
            }
        }
        (n, ok)
    }
}
