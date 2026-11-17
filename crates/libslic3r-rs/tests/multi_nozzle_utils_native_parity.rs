//! `slicer::multi_nozzle_utils` (the port of libslic3r/MultiNozzleUtils.cpp) against the C++ at the pin.
//!
//! `tests/data/multi_nozzle_utils/vectors.txt.gz` is the output of `gen.cpp` beside it (gzip -9n),
//! which compiles the pin's MultiNozzleUtils.cpp into itself and drives its free functions and the
//! member functions of LayeredNozzleGroupResult, StaticNozzleGroupResult, NozzleStatusRecorder,
//! NozzleInfo and NozzleGroupInfo on random nozzle lists, filament change sequences, configurations
//! and slice results. The header of gen.cpp describes the lines. Every operation is repeated here on
//! the Rust port and its outputs are written in the generator's format; the two texts have to be the
//! same (floats and doubles bit for bit, integers and texts equal).

use slicer::config::ConfigBase;
use slicer::extruder::NozzleVolumeType;
use slicer::gcode::g_code_processor::{ETimeMode, GCodeProcessorResult};
use slicer::multi_nozzle_utils::*;
use slicer::print_config::config_def::DynamicPrintConfig;
use slicer::project_task::FilamentInfo;
use std::collections::BTreeMap;
use std::io::Read;

fn read_vectors() -> String {
    let path = format!("{}/tests/data/multi_nozzle_utils/vectors.txt.gz", env!("CARGO_MANIFEST_DIR"));
    let file = std::fs::File::open(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
    let mut text = String::new();
    flate2::read::GzDecoder::new(file).read_to_string(&mut text).expect("gzip");
    text
}

fn hex(s: &[u8]) -> String {
    if s.is_empty() {
        return "-".to_string();
    }
    s.iter().map(|c| format!("{:02x}", c)).collect()
}
fn unhex(t: &str) -> String {
    if t == "-" {
        return String::new();
    }
    let b: Vec<u8> = (0..t.len() / 2).map(|i| u8::from_str_radix(&t[2 * i..2 * i + 2], 16).expect("hex")).collect();
    String::from_utf8(b).expect("the vectors are UTF-8")
}

/// A cursor over the input tokens of one line.
struct Toks<'a> {
    t: Vec<&'a str>,
    i: usize,
}

impl<'a> Toks<'a> {
    fn new(s: &'a str) -> Self {
        Toks { t: s.split(' ').filter(|x| !x.is_empty()).collect(), i: 0 }
    }
    fn s(&mut self) -> &'a str {
        let v = self.t[self.i];
        self.i += 1;
        v
    }
    fn i(&mut self) -> i64 {
        self.s().parse().expect("integer")
    }
    fn u(&mut self) -> usize {
        self.i() as usize
    }
    fn b(&mut self) -> bool {
        self.i() != 0
    }
    fn d(&mut self) -> f64 {
        f64::from_bits(u64::from_str_radix(self.s(), 16).expect("double"))
    }
    fn f(&mut self) -> f32 {
        f32::from_bits(u32::from_str_radix(self.s(), 16).expect("float"))
    }
    fn text(&mut self) -> String {
        unhex(self.s())
    }
    fn list_i32(&mut self) -> Vec<i32> {
        let n = self.u();
        (0..n).map(|_| self.i() as i32).collect()
    }
    fn list_u32(&mut self) -> Vec<u32> {
        let n = self.u();
        (0..n).map(|_| self.i() as u32).collect()
    }
    fn list_f32(&mut self) -> Vec<f32> {
        let n = self.u();
        (0..n).map(|_| self.f()).collect()
    }
    fn list_f64(&mut self) -> Vec<f64> {
        let n = self.u();
        (0..n).map(|_| self.d()).collect()
    }
    fn list_bool(&mut self) -> Vec<bool> {
        let n = self.u();
        (0..n).map(|_| self.b()).collect()
    }
    fn nozzle(&mut self) -> NozzleInfo {
        let mut ni = NozzleInfo::default();
        ni.diameter = self.text();
        ni.volume_type = NozzleVolumeType::from_i32(self.i() as i32);
        ni.extruder_id = self.i() as i32;
        ni.group_id = self.i() as i32;
        ni.installed = self.b();
        ni
    }
    fn nozzles(&mut self) -> Vec<NozzleInfo> {
        let n = self.u();
        (0..n).map(|_| self.nozzle()).collect()
    }
    fn params(&mut self) -> FilamentChangeTimeParams {
        FilamentChangeTimeParams {
            selector_load_time: self.f(),
            selector_unload_time: self.f(),
            standard_load_time: self.f(),
            standard_unload_time: self.f(),
        }
    }
    fn filament_infos(&mut self) -> Vec<FilamentInfo> {
        let n = self.u();
        (0..n)
            .map(|_| {
                let mut f = FilamentInfo::default();
                f.id = self.i() as i32;
                f.group_id = self.list_i32();
                f.nozzle_diameter = self.d();
                f.nozzle_volume_type = self.text();
                f
            })
            .collect()
    }
    /// "C": on top of a full configuration; "D": on top of an empty DynamicPrintConfig.
    fn cfg(&mut self) -> DynamicPrintConfig {
        let tag = self.s();
        let mut cfg = match tag {
            "C" => DynamicPrintConfig::full_print_config(),
            "D" => DynamicPrintConfig::new(),
            _ => panic!("configuration tag {tag}"),
        };
        let n = self.u();
        for _ in 0..n {
            let k = self.text();
            let v = self.text();
            // As the generator: an enum vector is given the definition's key map before names are
            // deserialized into it (its default value has none, Config.hpp:2135).
            if let Some(def) = slicer::print_config::config_def::print_config_def().base.get(&k) {
                if def.r#type == slicer::config::ConfigOptionType::CoEnums {
                    if let Ok(Some(o)) = cfg.option_mut(&k, true) {
                        if let Some(e) = o.as_any_mut().downcast_mut::<slicer::config::ConfigOptionEnumsGeneric>() {
                            e.keys_map = def.enum_keys_map;
                        }
                    }
                }
            }
            cfg.set_deserialize_strict(&k, &v, false).unwrap_or_else(|e| panic!("{k}={v}: {e:?}"));
        }
        cfg
    }
    fn arrow(&mut self) {
        assert_eq!(self.s(), "=>");
    }
    fn rest(&self) -> String {
        self.t[self.i..].join(" ")
    }
}

/// The outputs of one line, in the generator's format.
#[derive(Default)]
struct Out(Vec<String>);

impl Out {
    fn tok(&mut self, s: &str) {
        self.0.push(s.to_string());
    }
    fn i(&mut self, v: i64) {
        self.0.push(v.to_string());
    }
    fn b(&mut self, v: bool) {
        self.i(v as i64);
    }
    fn d(&mut self, v: f64) {
        self.0.push(format!("{:016x}", v.to_bits()));
    }
    fn f(&mut self, v: f32) {
        self.0.push(format!("{:08x}", v.to_bits()));
    }
    fn s(&mut self, v: &str) {
        self.0.push(hex(v.as_bytes()));
    }
    fn list<T: Copy + Into<i64>>(&mut self, v: &[T]) {
        self.i(v.len() as i64);
        for &x in v {
            self.i(x.into());
        }
    }
    fn nozzle(&mut self, n: &NozzleInfo) {
        self.s(&n.diameter);
        self.i(n.volume_type as i64);
        self.i(n.extruder_id as i64);
        self.i(n.group_id as i64);
        self.b(n.installed);
    }
    fn nozzles(&mut self, v: &[NozzleInfo]) {
        self.i(v.len() as i64);
        for n in v {
            self.nozzle(n);
        }
    }
    fn opt(&mut self, n: &Option<NozzleInfo>) {
        match n {
            None => self.tok("-"),
            Some(n) => {
                self.tok("+");
                self.nozzle(n);
            }
        }
    }
    fn text(&self) -> String {
        self.0.join(" ")
    }
}

struct SimInput {
    logical: Vec<i32>,
    nozzles: Vec<NozzleInfo>,
    fseq: Vec<i32>,
    nseq: Vec<i32>,
    groups: Vec<i32>,
    params: FilamentChangeTimeParams,
    preload: Vec<bool>,
}

fn read_sim(t: &mut Toks) -> SimInput {
    SimInput {
        logical: t.list_i32(),
        nozzles: t.nozzles(),
        fseq: t.list_i32(),
        nseq: t.list_i32(),
        groups: t.list_i32(),
        params: t.params(),
        preload: t.list_bool(),
    }
}

fn dump_layered(o: &mut Out, r: &Option<LayeredNozzleGroupResult>, filaments: i32, layers: i32, nozzles: i32) {
    let Some(r) = r else {
        o.i(0);
        return;
    };
    o.i(1);
    o.b(r.is_support_dynamic_nozzle_map());
    o.b(r.is_handle_nozzle_as_extruder());
    o.i(r.get_extruder_count() as i64);
    o.i(r.get_layer_count() as i64);
    o.list(&r.get_used_filaments());
    o.list(&r.get_used_extruders());
    for e in -1..=2 {
        o.nozzles(&r.get_used_nozzles_in_extruder(e));
    }
    for l in -1..=layers {
        o.list(r.get_layer_filament_nozzle_map(l));
        o.list(&r.get_used_filaments_layer(l));
        o.list(&r.get_used_extruders_layer(l));
        for e in -1..=2 {
            o.nozzles(&r.get_used_nozzles_in_extruder_layer(e, l));
        }
        o.list(&r.get_extruder_map(true, l));
        o.list(&r.get_extruder_map(false, l));
        o.list(&r.get_nozzle_map(l));
        o.list(&r.get_volume_map(l));
        for f in -1..=filaments {
            o.opt(&r.get_nozzle_for_filament(f, l));
            o.i(r.get_extruder_id(f, l) as i64);
            o.i(r.get_nozzle_id(f, l) as i64);
            for g in 0..filaments {
                o.b(r.are_filaments_same_extruder(f, g, l));
                o.b(r.are_filaments_same_nozzle(f, g, l));
            }
        }
    }
    for f in -1..=filaments {
        o.nozzles(&r.get_nozzles_for_filament(f));
        o.opt(&r.get_first_nozzle_for_filament(f));
    }
    for n in -1..=nozzles {
        o.opt(&r.get_nozzle_from_id(n));
    }
    let seqs = r.get_layer_filament_sequences();
    o.list(if seqs.is_empty() { &[][..] } else { &seqs[0][..] });
}

fn dump_recorder(o: &mut Out, r: &NozzleStatusRecorder) {
    for n in -1..=12 {
        o.b(r.is_nozzle_empty(n));
        o.i(r.get_filament_in_nozzle(n) as i64);
    }
    for e in -1..=4 {
        o.i(r.get_nozzle_in_extruder(e) as i64);
    }
    o.i(r.get_current_extruder_id() as i64);
    let m = r.get_nozzle_filament_map();
    o.i(m.len() as i64);
    for (k, &v) in m.iter() {
        o.i(k as i64);
        o.i(v as i64);
    }
    let m = r.get_extruder_nozzle_map();
    o.i(m.len() as i64);
    for (k, &v) in m.iter() {
        o.i(k as i64);
        o.i(v as i64);
    }
}

/// Runs one line on the port; returns the outputs in the generator's format.
fn run(op: &str, t: &mut Toks) -> String {
    let mut o = Out::default();
    match op {
        "sim" => {
            let s = read_sim(t);
            let sliced = t.b();
            t.arrow();
            let r = simulate_filament_change_time(&s.logical, &s.nozzles, &s.fseq, &s.nseq, &s.groups, &s.params, &s.preload, sliced);
            o.d(r.actual_time);
            o.d(r.sliced_time);
        }
        "gap" => {
            let s = read_sim(t);
            t.arrow();
            o.f(calc_filament_change_gap_for_assignment(&s.logical, &s.nozzles, &s.fseq, &s.nseq, &s.groups, &s.params, &s.preload));
        }
        "opt" => {
            let logical = t.list_i32();
            let nozzles = t.nozzles();
            let fseq = t.list_i32();
            let nseq = t.list_i32();
            let group_count = t.i() as i32;
            let params = t.params();
            t.arrow();
            o.list(&find_optimal_physical_assignment(&logical, &nozzles, &fseq, &nseq, group_count, &params, 100000));
        }
        "bnl" => {
            let n = t.u();
            let groups: Vec<NozzleGroupInfo> = (0..n)
                .map(|_| {
                    let d = t.text();
                    let vt = NozzleVolumeType::from_i32(t.i() as i32);
                    let e = t.i() as i32;
                    let c = t.i() as i32;
                    NozzleGroupInfo::new(d, vt, e, c)
                })
                .collect();
            t.arrow();
            o.nozzles(&build_nozzle_list(groups));
        }
        "bnl2" => {
            let d = t.d();
            let nozzle_map = t.list_i32();
            let volume_map = t.list_i32();
            let filament_map = t.list_i32();
            t.arrow();
            o.nozzles(&build_nozzle_list_from_maps(d, &nozzle_map, &volume_map, &filament_map));
        }
        "bxn" => {
            let cfg = t.cfg();
            t.arrow();
            o.nozzles(&build_nozzle_list_from_extruder_nozzles(&cfg));
        }
        "rmn" => {
            let cfg = t.cfg();
            let used = t.list_u32();
            let fmap = t.list_i32();
            let vmap = t.list_i32();
            let kmap = t.list_i32();
            t.arrow();
            o.list(&resolve_manual_nozzle_map_to_index(&cfg, &used, &fmap, &vmap, &kmap));
        }
        "lnc" => {
            let nozzles = t.nozzles();
            let infos = t.filament_infos();
            let fmap = t.list_i32();
            let vts: Vec<NozzleVolumeType> = t.list_i32().into_iter().map(NozzleVolumeType::from_i32).collect();
            let nd = t.list_f64();
            t.arrow();
            o.nozzles(&load_nozzle_infos_with_compatibility(&nozzles, &infos, &fmap, &vts, &nd));
        }
        "lc1" => {
            let map = t.list_i32();
            let nozzles = t.nozzles();
            let used = t.list_u32();
            let handle = t.b();
            let filaments = t.i() as i32;
            t.arrow();
            let r = <LayeredNozzleGroupResult as CreateFromNozzleMap>::create(&map, &nozzles, &used, handle);
            dump_layered(&mut o, &r, filaments, 0, nozzles.len() as i32);
        }
        "lc2" => {
            let nm = t.u();
            let maps: Vec<Vec<i32>> = (0..nm).map(|_| t.list_i32()).collect();
            let nozzles = t.nozzles();
            let used = t.list_u32();
            let ns = t.u();
            let seqs: Vec<Vec<u32>> = (0..ns).map(|_| t.list_u32()).collect();
            let handle = t.b();
            let filaments = t.i() as i32;
            t.arrow();
            let r = <LayeredNozzleGroupResult as CreateFromLayerMaps>::create(&maps, &nozzles, &used, &seqs, handle);
            dump_layered(&mut o, &r, filaments, maps.len() as i32, nozzles.len() as i32);
        }
        "lc3" => {
            let used = t.list_u32();
            let fmap = t.list_i32();
            let vmap = t.list_i32();
            let nmap = t.list_i32();
            let extruders = t.u();
            let mut counts: Vec<BTreeMap<NozzleVolumeType, i32>> = Vec::new();
            let mut total = 0;
            for _ in 0..extruders {
                let n = t.u();
                let mut m = BTreeMap::new();
                for _ in 0..n {
                    let vt = NozzleVolumeType::from_i32(t.i() as i32);
                    let c = t.i() as i32;
                    total += c;
                    m.insert(vt, c);
                }
                counts.push(m);
            }
            let diameter = t.f();
            let handle = t.b();
            let filaments = t.i() as i32;
            t.arrow();
            let r = <LayeredNozzleGroupResult as CreateFromNozzleCount>::create(&used, &fmap, &vmap, &nmap, &counts, diameter, handle);
            dump_layered(&mut o, &r, filaments, 0, total);
        }
        "esw" => {
            let map = t.list_i32();
            let nozzles = t.nozzles();
            let used = t.list_u32();
            let ne = t.u();
            let fm: Vec<Vec<Vec<f32>>> = (0..ne)
                .map(|_| {
                    let rows = t.u();
                    (0..rows).map(|_| t.list_f32()).collect()
                })
                .collect();
            let seq = t.list_i32();
            t.arrow();
            let r = LayeredNozzleGroupResult::create(&map, &nozzles, &used);
            o.i(match &r {
                Some(r) => r.estimate_seq_flush_weight(&fm, &seq) as i64,
                None => -999999,
            });
        }
        "sc" => {
            let infos = t.filament_infos();
            let nozzles = t.nozzles();
            let fseq = t.list_i32();
            let nseq = t.list_i32();
            let support = t.b();
            t.arrow();
            match StaticNozzleGroupResult::create(&infos, &nozzles, &fseq, &nseq, support) {
                None => o.i(0),
                Some(r) => {
                    o.i(1);
                    o.b(r.is_support_dynamic_nozzle_map());
                    o.i(r.get_extruder_count() as i64);
                    for e in -1..=2 {
                        o.nozzles(&r.get_used_nozzles_in_extruder(e));
                    }
                    o.list(&r.get_used_extruders());
                    o.list(&r.get_used_filaments());
                    for n in -1..=7 {
                        o.opt(&r.get_nozzle_from_id(n));
                    }
                    for f in -1..=6 {
                        o.nozzles(&r.get_nozzles_for_filament(f));
                        o.opt(&r.get_first_nozzle_for_filament(f));
                    }
                }
            }
        }
        "rec" => {
            let mut r = [NozzleStatusRecorder::new(), NozzleStatusRecorder::new()];
            let nops = t.u();
            for _ in 0..nops {
                match t.s() {
                    "s" => {
                        let w = t.u();
                        let n = t.i() as i32;
                        let f = t.i() as i32;
                        let e = t.i() as i32;
                        r[w].set_nozzle_status(n, f, e);
                    }
                    "c" => {
                        let w = t.u();
                        let n = t.i() as i32;
                        r[w].clear_nozzle_status(n);
                    }
                    "x" => {
                        let w = t.u();
                        let e = t.i() as i32;
                        r[w].set_current_extruder_id(e);
                    }
                    "A" => {
                        let (a, b) = r.split_at_mut(1);
                        a[0].assign_from(&b[0]);
                    }
                    "K" => {
                        r[1] = r[0].clone();
                    }
                    other => panic!("recorder op {other}"),
                }
            }
            t.arrow();
            dump_recorder(&mut o, &r[0]);
            dump_recorder(&mut o, &r[1]);
        }
        "umap" => {
            let mut m = [UnorderedMapInt::<i32>::new(), UnorderedMapInt::<i32>::new()];
            let nops = t.u();
            for _ in 0..nops {
                match t.s() {
                    "o" => {
                        let w = t.u();
                        let k = t.i() as i32;
                        let v = t.i() as i32;
                        *m[w].entry(k) = v;
                    }
                    "i" => {
                        let w = t.u();
                        let k = t.i() as i32;
                        let v = t.i() as i32;
                        m[w].insert(k, v);
                    }
                    "e" => {
                        let w = t.u();
                        let k = t.i() as i32;
                        m[w].erase(k);
                    }
                    "r" => {
                        let w = t.u();
                        let n = t.u();
                        m[w].reserve(n);
                    }
                    "c" => {
                        let w = t.u();
                        m[w].clear();
                    }
                    "A" => {
                        let (a, b) = m.split_at_mut(1);
                        a[0].assign_from(&b[0]);
                    }
                    "K" => {
                        m[1] = m[0].clone();
                    }
                    "B" => {
                        let n = t.u();
                        m[1] = UnorderedMapInt::with_bucket_count(n);
                    }
                    other => panic!("map op {other}"),
                }
            }
            t.arrow();
            for mm in &m {
                o.i(mm.len() as i64);
                o.i(mm.bucket_count() as i64);
                for (k, &v) in mm.iter() {
                    o.i(k as i64);
                    o.i(v as i64);
                }
            }
        }
        "nis" => {
            let n = t.nozzle();
            t.arrow();
            o.s(&n.serialize());
        }
        "ngs" => {
            let d = t.text();
            let vt = NozzleVolumeType::from_i32(t.i() as i32);
            let e = t.i() as i32;
            let c = t.i() as i32;
            t.arrow();
            o.s(&NozzleGroupInfo::new(d, vt, e, c).serialize());
        }
        "ngd" => {
            let s = t.text();
            t.arrow();
            match NozzleGroupInfo::deserialize(&s) {
                None => o.i(0),
                Some(g) => {
                    o.i(1);
                    o.s(&g.diameter);
                    o.i(g.volume_type as i64);
                    o.i(g.extruder_id as i64);
                    o.i(g.nozzle_count as i64);
                }
            }
        }
        "ams" => {
            let loads = t.list_f64();
            let unloads = t.list_f64();
            let ty = t.i() as i32;
            let ll = t.f();
            let lu = t.f();
            t.arrow();
            let (ok, ol, ou) = resolve_ams_load_unload(&loads, &unloads, ty, ll, lu);
            o.b(ok);
            o.f(ol);
            o.f(ou);
        }
        "emp" => {
            let cfg = t.cfg();
            let time = t.f();
            let seq = t.list_u32();
            let types = t.list_i32();
            let null_flags = t.i();
            t.arrow();
            let mut res = GCodeProcessorResult::default();
            res.print_statistics.modes[ETimeMode::Normal as usize].time = time;
            let r = estimate_mapped_print_time(
                if null_flags & 1 != 0 { None } else { Some(&res) },
                &seq,
                if null_flags & 2 != 0 { None } else { Some(&cfg) },
                &types,
            );
            o.f(r.total_seconds);
            o.f(r.gap_seconds);
            o.i(r.reason as i64);
        }
        _ => panic!("unknown op {op}"),
    }
    o.text()
}

#[test]
fn multi_nozzle_utils_matches_the_pin() {
    let text = read_vectors();
    let mut cases: BTreeMap<String, usize> = BTreeMap::new();
    let mut failures: BTreeMap<String, usize> = BTreeMap::new();
    let mut first: Vec<String> = Vec::new();
    for (ln, line) in text.lines().enumerate() {
        let mut t = Toks::new(line);
        let op = t.s().to_string();
        let got = run(&op, &mut t);
        let want = t.rest();
        *cases.entry(op.clone()).or_default() += 1;
        if got != want {
            *failures.entry(op.clone()).or_default() += 1;
            if first.len() < 8 {
                let cut = |s: &str| s.chars().take(600).collect::<String>();
                first.push(format!("line {} ({op}):\n  input {}\n  want  {}\n  got   {}", ln + 1, cut(line.split(" => ").next().unwrap_or("")), cut(&want), cut(&got)));
            }
        }
    }
    eprintln!("cases per operation: {cases:?}");
    assert!(failures.is_empty(), "mismatches per operation: {failures:?}\n{}", first.join("\n"));
}
