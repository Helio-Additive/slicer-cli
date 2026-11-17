//! `slicer::filament_group` (the port of libslic3r/FilamentGroup.cpp) against the C++ at the pin.
//!
//! `tests/data/filament_group/vectors.txt.gz` is the output of `gen.cpp` beside it (gzip -9n), which
//! compiles the pin's FilamentGroup.cpp into itself and drives its file-static functions, the members
//! of FilamentGroup, KMediods, FlushDistanceEvaluator, TimeEvaluator and FilamentGroupContext and the
//! free grouping functions on random grouping contexts. The header of gen.cpp describes the lines.
//! Every operation is repeated here on the Rust port and what it returns has to be the same: floats
//! and doubles bit for bit, integers and texts equal, unordered maps in the same iteration order.

use slicer::extruder::NozzleVolumeType;
use slicer::filament_group::*;
use slicer::filament_group_utils::*;
use slicer::multi_nozzle_utils::NozzleInfo;
use std::collections::{BTreeMap, BTreeSet, HashMap};
use std::io::Read;
use std::rc::Rc;

fn read_vectors() -> String {
    let path = format!("{}/tests/data/filament_group/vectors.txt.gz", env!("CARGO_MANIFEST_DIR"));
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

/// A cursor over the tokens of one line.
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
        let n = self.i() as usize;
        (0..n).map(|_| self.i() as i32).collect()
    }
    fn list_u32(&mut self) -> Vec<u32> {
        let n = self.i() as usize;
        (0..n).map(|_| self.i() as u32).collect()
    }
    fn set_i32(&mut self) -> BTreeSet<i32> {
        self.list_i32().into_iter().collect()
    }
}

/// The output tokens of one operation.
#[derive(Default)]
struct Out {
    t: Vec<String>,
}

impl Out {
    fn i(&mut self, v: i64) {
        self.t.push(v.to_string());
    }
    fn u(&mut self, v: u64) {
        self.t.push(v.to_string());
    }
    fn d(&mut self, v: f64) {
        self.t.push(format!("{:016x}", v.to_bits()));
    }
    fn f(&mut self, v: f32) {
        self.t.push(format!("{:08x}", v.to_bits()));
    }
    fn s(&mut self, v: &str) {
        self.t.push(hex(v.as_bytes()));
    }
    fn list<T: Copy + Into<i64>>(&mut self, v: &[T]) {
        self.i(v.len() as i64);
        for &x in v {
            self.i(x.into());
        }
    }
    fn exc(&mut self, e: &FilamentGroupError) {
        self.t.push("EXC".to_string());
        match e {
            FilamentGroupError::Group(g) => {
                self.i(0);
                self.s(g.what());
            }
            FilamentGroupError::Runtime(r) => {
                self.i(1);
                self.s(&r.message);
            }
            FilamentGroupError::OutOfRange(m) => {
                self.i(2);
                self.s(m);
            }
        }
    }
    fn join(&self) -> String {
        self.t.join(" ")
    }
}

fn usage_type(v: i64) -> FilamentUsageType {
    match v {
        0 => FilamentUsageType::SupportOnly,
        1 => FilamentUsageType::ModelOnly,
        _ => FilamentUsageType::Hybrid,
    }
}

fn read_finfo(t: &mut Toks) -> FilamentInfo {
    let r = t.i() as u8;
    let g = t.i() as u8;
    let b = t.i() as u8;
    let a = t.i() as u8;
    FilamentInfo { color: Color::new(r, g, b, a), type_: t.text(), is_support: t.b(), usage_type: usage_type(t.i()), diameter: t.f() }
}

fn read_ctx(t: &mut Toks) -> FilamentGroupContext {
    let mut c = FilamentGroupContext::default();
    // model
    let nfm = t.i() as usize;
    for _ in 0..nfm {
        let rows = t.i() as usize;
        let mut fm = Vec::new();
        for _ in 0..rows {
            let n = t.i() as usize;
            fm.push((0..n).map(|_| t.f()).collect::<Vec<f32>>());
        }
        c.model_info.flush_matrix.push(fm);
    }
    let nl = t.i() as usize;
    for _ in 0..nl {
        c.model_info.layer_filaments.push(t.list_u32());
    }
    let nf = t.i() as usize;
    for _ in 0..nf {
        c.model_info.filament_info.push(read_finfo(t));
    }
    let n = t.i() as usize;
    for _ in 0..n {
        c.model_info.filament_ids.push(t.text());
    }
    let n = t.i() as usize;
    for _ in 0..n {
        c.model_info.selected_nozzle_diameters.push(t.text());
    }
    let n = t.i() as usize;
    for _ in 0..n {
        c.model_info.unprintable_filaments.push(t.set_i32());
    }
    let n = t.i() as usize;
    for _ in 0..n {
        let k = t.i() as i32;
        let m = t.i() as usize;
        let s: BTreeSet<NozzleVolumeType> = (0..m).map(|_| NozzleVolumeType::from_i32(t.i() as i32)).collect();
        c.model_info.unprintable_volumes.insert(k, s);
    }
    // group
    c.group_info.total_filament_num = t.i() as i32;
    c.group_info.max_gap_threshold = t.d();
    c.group_info.mode = if t.i() == 0 { FGMode::FlushMode } else { FGMode::MatchMode };
    c.group_info.strategy = if t.i() == 0 { FGStrategy::BestCost } else { FGStrategy::BestFit };
    c.group_info.ignore_ext_filament = t.b();
    c.group_info.has_filament_switcher = t.b();
    c.group_info.filament_volume_map = t.list_i32();
    // machine
    c.machine_info.max_group_size = t.list_i32();
    let n = t.i() as usize;
    for _ in 0..n {
        let m = t.i() as usize;
        let mut v = Vec::new();
        for _ in 0..m {
            let f = read_finfo(t);
            v.push(MachineFilamentInfo {
                color: f.color,
                type_: f.type_,
                is_support: f.is_support,
                usage_type: f.usage_type,
                diameter: f.diameter,
                extruder_id: t.i() as i32,
                is_extended: t.b(),
                filament_id: t.text(),
            });
        }
        c.machine_info.machine_filament_info.push(v);
    }
    let n = t.i() as usize;
    for _ in 0..n {
        let base = read_finfo(t);
        c.machine_info.nozzle_loaded_filaments.push(NozzleLoadedFilamentInfo {
            base,
            nozzle_id: t.i() as i32,
            filament_id: t.text(),
            valid: t.b(),
        });
    }
    let n = t.i() as usize;
    c.machine_info.prefer_non_model_filament = (0..n).map(|_| t.b()).collect();
    c.machine_info.master_extruder_id = t.i() as i32;
    // speed, in the insertion order of the C++
    let n = t.i() as usize;
    for _ in 0..n {
        let fid = t.i() as i32;
        let m = t.i() as usize;
        for _ in 0..m {
            let key = t.i() as i32;
            let v = t.d();
            *c.speed_info.filament_print_time.entry(fid).entry(key) = v;
        }
    }
    c.speed_info.extruder_change_time = t.d();
    c.speed_info.filament_change_time = t.d();
    c.speed_info.group_with_time = t.b();
    c.speed_info.change_time_params.selector_load_time = t.f();
    c.speed_info.change_time_params.selector_unload_time = t.f();
    c.speed_info.change_time_params.standard_load_time = t.f();
    c.speed_info.change_time_params.standard_unload_time = t.f();
    let n = t.i() as usize;
    c.speed_info.ams_preload_enabled = (0..n).map(|_| t.b()).collect();
    // nozzle
    let n = t.i() as usize;
    for _ in 0..n {
        let mut ni = NozzleInfo::default();
        ni.diameter = t.text();
        ni.volume_type = NozzleVolumeType::from_i32(t.i() as i32);
        ni.extruder_id = t.i() as i32;
        ni.group_id = t.i() as i32;
        ni.installed = t.b();
        c.nozzle_info.nozzle_list.push(ni);
    }
    let n = t.i() as usize;
    for _ in 0..n {
        let k = t.i() as i32;
        let v = t.list_i32();
        c.nozzle_info.extruder_nozzle_list.insert(k, v);
    }
    let n = t.i() as usize;
    for _ in 0..n {
        let k = t.i() as i32;
        let v = t.i() as i32;
        *c.nozzle_info.nozzle_status.entry(k) = v;
    }
    c.nozzle_info.per_nozzle_capacity = t.list_i32();
    assert_eq!(build_extruder_nozzle_list(&c.nozzle_info.nozzle_list), c.nozzle_info.extruder_nozzle_list);
    c
}

fn write_nozzle(o: &mut Out, n: &NozzleInfo) {
    o.s(&n.diameter);
    o.i(n.volume_type as i64);
    o.i(n.extruder_id as i64);
    o.i(n.group_id as i64);
    o.i(n.installed as i64);
}

fn write_ctx_state(o: &mut Out, c: &FilamentGroupContext) {
    o.i(c.nozzle_info.nozzle_list.len() as i64);
    for n in &c.nozzle_info.nozzle_list {
        write_nozzle(o, n);
    }
    o.i(c.nozzle_info.nozzle_status.len() as i64);
    for (k, &v) in c.nozzle_info.nozzle_status.iter() {
        o.i(k as i64);
        o.i(v as i64);
    }
    o.list(&c.nozzle_info.per_nozzle_capacity);
    o.i(c.nozzle_info.extruder_nozzle_list.len() as i64);
    for (&k, v) in &c.nozzle_info.extruder_nozzle_list {
        o.i(k as i64);
        o.list(v);
    }
    o.i(c.machine_info.master_extruder_id as i64);
    o.i(c.machine_info.prefer_non_model_filament.len() as i64);
    for &b in &c.machine_info.prefer_non_model_filament {
        o.i(b as i64);
    }
    o.i(c.speed_info.filament_print_time.len() as i64);
    for (k, inner) in c.speed_info.filament_print_time.iter() {
        o.i(k as i64);
        o.i(inner.len() as i64);
        for (kk, &v) in inner.iter() {
            o.i(kk as i64);
            o.d(v);
        }
    }
    o.i(c.model_info.unprintable_volumes.len() as i64);
    for (&k, v) in &c.model_info.unprintable_volumes {
        o.i(k as i64);
        o.i(v.len() as i64);
    }
}

fn write_heap(o: &mut Out, h: &MemoryedGroupHeap) {
    o.i(h.c().len() as i64);
    for m in h.c() {
        o.d(m.cost);
        o.i(m.prefer_level as i64);
        o.list(&m.group);
    }
}

fn extract_hash(c: &FilamentGroupContext, used: &[u32]) -> HashMap<i32, Vec<i32>> {
    let mut lim: HashMap<i32, Vec<i32>> = HashMap::new();
    extract_unprintable_limit_indices(&c.model_info.unprintable_filaments, used, &mut lim);
    lim
}

#[test]
fn filament_group_matches_the_pin() {
    let text = read_vectors();
    let mut ctx = FilamentGroupContext::default();
    let mut km: Option<(KMediods, FilamentGroup)> = None;
    let mut counts: BTreeMap<String, usize> = BTreeMap::new();
    let mut failures: Vec<String> = Vec::new();

    for (lineno, line) in text.lines().enumerate() {
        let (lhs, expected) = match line.find(" => ") {
            Some(p) => (&line[..p], line[p + 4..].to_string()),
            None => (line, String::new()),
        };
        let lhs = if line.ends_with(" =>") { &line[..line.len() - 3] } else { lhs };
        let expected = if line.ends_with(" =>") { String::new() } else { expected };
        let mut t = Toks::new(lhs);
        let op = t.s().to_string();
        let mut o = Out::default();
        let used = collect_sorted_used_filaments(&ctx.model_info.layer_filaments);
        match op.as_str() {
            "ctx" => {
                ctx = read_ctx(&mut t);
                km = None;
                continue;
            }
            "cmc" => {
                let cd = t.f();
                let a = t.text();
                let b = t.text();
                let n = t.i() as i32;
                o.i(calc_match_mode_cost(cd, &a, &b, n) as i64);
            }
            "fnv" => {
                let d = t.text();
                let vt = t.i() as i32;
                let r = t.i() as i32;
                let lf = t.i() as i32;
                let cap = t.i() as i32;
                o.u(fnv_hash_nozzle(&d, vt, r, lf, cap));
            }
            "evs" => {
                let f = t.d();
                let tt = t.d();
                let w = t.b();
                o.d(evaluate_score(f, tt, w));
            }
            "fdm" => {
                let dia = t.f();
                let vt = t.i() as i32;
                let fi = FilamentInfo { diameter: dia, ..FilamentInfo::default() };
                o.i(filament_diameter_matches_nozzle_volume(&fi, NozzleVolumeType::from_i32(vt)) as i64);
            }
            "umg" => {
                let budget = t.d();
                let thr = t.d();
                let n = t.i();
                let mut heap = MemoryedGroupHeap::new();
                for _ in 0..n {
                    let cost = t.d();
                    let lvl = t.i() as i32;
                    let grp = t.list_i32();
                    update_memoryed_groups(&MemoryedGroup::new(grp, cost, lvl), budget, thr, &mut heap);
                }
                write_heap(&mut o, &heap);
                let mut arrs = Vec::new();
                change_memoryed_heaps_to_arrays(&mut heap, 5, &[0, 2, 3], &mut arrs);
                o.i(arrs.len() as i64);
                for a in &arrs {
                    o.list(a);
                }
            }
            "csu" => {
                let l = t.i();
                let lf: Vec<Vec<u32>> = (0..l).map(|_| t.list_u32()).collect();
                o.list(&collect_sorted_used_filaments(&lf));
            }
            "ded" => {
                let k = t.i() as i32;
                let n = t.i() as i32;
                o.i(estimate_dedup_enum_count(k, n, &ctx));
            }
            "ocb" => {
                let fg = FilamentGroup::new(&ctx);
                o.d(fg.calc_one_change_budget(&used));
            }
            "fe" => {
                let m = t.list_i32();
                let mut flush = -7;
                let s = full_evaluate(&used, &m, &ctx, None, Some(&mut flush));
                o.d(s);
                o.i(flush as i64);
            }
            "te" => {
                let m = t.list_i32();
                let te = TimeEvaluator::new(&ctx.speed_info);
                o.d(te.get_estimated_time(&m));
            }
            "fde" => {
                let p = t.d();
                let fde = FlushDistanceEvaluator::new(&ctx.model_info.flush_matrix, &used, &ctx.model_info.layer_filaments, p);
                for e in 0..ctx.model_info.flush_matrix.len() {
                    for a in 0..used.len() {
                        for b in 0..used.len() {
                            o.d(fde.get_distance(a as i32, b as i32, e as i32));
                        }
                    }
                }
            }
            "ao" => {
                let mut full_ids = Vec::new();
                let actx = ctx.available_only(&mut full_ids);
                o.list(&full_ids);
                write_ctx_state(&mut o, &actx);
            }
            "eul" => {
                let mut fg = FilamentGroup::new(&ctx);
                let mut lim: BTreeMap<i32, i32> = BTreeMap::new();
                extract_unprintable_limit_indices(&ctx.model_info.unprintable_filaments, &used, &mut lim);
                o.i(lim.len() as i64);
                for (&k, &v) in &lim {
                    o.i(k as i64);
                    o.i(v as i64);
                }
                let r = fg.rebuild_unprintables(&used, &lim);
                o.i(r.len() as i64);
                for (&k, &v) in &r {
                    o.i(k as i64);
                    o.i(v as i64);
                }
                write_ctx_state(&mut o, &fg.ctx);
            }
            "rnu" => {
                let mut full_ids = Vec::new();
                let actx = ctx.available_only(&mut full_ids);
                let mut fg = FilamentGroup::new(&actx);
                let lim = extract_hash(&actx, &used);
                let r = fg.rebuild_nozzle_unprintables(&used, &lim, &actx.group_info.filament_volume_map);
                o.i(r.len() as i64);
                for (k, v) in r.iter() {
                    o.i(k as i64);
                    o.list(v);
                }
            }
            "tmf" => {
                let m = t.list_i32();
                let mut fg = FilamentGroup::new(&ctx);
                let merged = fg.try_merge_filaments();
                let sorted: BTreeMap<i32, Vec<i32>> = merged.iter().map(|(&k, v)| (k, v.clone())).collect();
                let sep = fg.seperate_merged_filaments(&m, &merged);
                fg.rebuild_context(&merged);
                o.i(sorted.len() as i64);
                for (&k, v) in &sorted {
                    o.i(k as i64);
                    o.list(v);
                }
                o.list(&sep);
                o.i(fg.ctx.model_info.layer_filaments.len() as i64);
                for lf in &fg.ctx.model_info.layer_filaments {
                    o.list(lf);
                }
            }
            "cfg" => {
                let mut fg = FilamentGroup::new(&ctx);
                let mut cost = -7;
                match fg.calc_filament_group(Some(&mut cost)) {
                    Ok(r) => {
                        o.list(&r);
                        o.i(cost as i64);
                        o.i(fg.m_memoryed_groups.len() as i64);
                        for a in &fg.m_memoryed_groups {
                            o.list(a);
                        }
                    }
                    Err(e) => o.exc(&e),
                }
            }
            "kmd" => {
                let mut full_ids = Vec::new();
                let actx = ctx.available_only(&mut full_ids);
                let mut fg = FilamentGroup::new(&actx);
                let k = actx.nozzle_info.nozzle_list.len() as i32;
                let lim = extract_hash(&actx, &used);
                let ul = fg.rebuild_nozzle_unprintables(&used, &lim, &actx.group_info.filament_volume_map);
                let mut cost = -7;
                match fg.calc_group_by_kmedoids(k, &used, &ul, Some(&mut cost), 1000000000) {
                    Ok(r) => {
                        o.list(&r);
                        o.i(cost as i64);
                        write_heap(&mut o, &fg.m_memoryed_heap);
                    }
                    Err(e) => o.exc(&e),
                }
            }
            "kmm" => {
                let cs = t.list_i32();
                let seed = t.i() as i32;
                let mut full_ids = Vec::new();
                let actx = ctx.available_only(&mut full_ids);
                let k = actx.nozzle_info.nozzle_list.len() as i32;
                let ev = Rc::new(FlushDistanceEvaluator::new(&actx.model_info.flush_matrix, &used, &actx.model_info.layer_filaments, 0.65));
                let mut kmed = KMediods::new(k, used.len() as i32, ev, actx.machine_info.master_extruder_id);
                let lim = extract_hash(&actx, &used);
                let mut fg = FilamentGroup::new(&actx);
                let ul = fg.rebuild_nozzle_unprintables(&used, &lim, &actx.group_info.filament_volume_map);
                kmed.set_unplacable_limits(&ul);
                let mut csl: Vec<(BTreeSet<i32>, i32)> = Vec::new();
                for (&e, nozzles) in &actx.nozzle_info.extruder_nozzle_list {
                    let cap = if (e as usize) < actx.machine_info.max_group_size.len() { actx.machine_info.max_group_size[e as usize] } else { 1 };
                    csl.push((nozzles.iter().copied().collect(), cap));
                }
                kmed.set_cluster_group_size(&csl);
                let center = kmed.init_cluster_center(&kmed.m_placeable_limits, &kmed.m_unplaceable_limits, &cs, &csl, seed);
                let labels = kmed.assign_cluster_label(&center, &kmed.m_placeable_limits, &kmed.m_unplaceable_limits, &cs, &csl);
                o.list(&center);
                o.list(&labels);
                o.i(kmed.have_enough_size(&cs, &csl, used.len() as i32) as i64);
                o.i(kmed.calc_cost(&labels, &center, -1) as i64);
                o.i(kmed.calc_cost(&labels, &center, if labels.is_empty() { 0 } else { labels[0] }) as i64);
                km = Some((kmed, fg));
            }
            "kdc" => {
                let retry = t.i() as i32;
                let mut full_ids = Vec::new();
                let actx = ctx.available_only(&mut full_ids);
                let (kmed, fg) = km.as_mut().expect("kdc follows kmm");
                kmed.set_memory_threshold(0.01);
                kmed.set_one_change_budget(fg.calc_one_change_budget(&used));
                kmed.do_clustering(&actx, 1000000000, retry);
                o.list(&kmed.get_cluster_labels());
                write_heap(&mut o, &kmed.get_memoryed_groups());
                o.i(kmed.m_unplaceable_limits.len() as i64);
                for (k, v) in kmed.m_unplaceable_limits.iter() {
                    o.i(k as i64);
                    o.list(v);
                }
            }
            "cfm" => {
                let mut fg = FilamentGroup::new(&ctx);
                match fg.calc_filament_group_for_match(None) {
                    Ok(r) => o.list(&r),
                    Err(e) => o.exc(&e),
                }
            }
            "sbg" => {
                let m = t.i();
                let maps: Vec<Vec<i32>> = (0..m).map(|_| t.list_i32()).collect();
                let sw = t.b();
                let thr = t.d();
                let info: Vec<FilamentInfo> = used.iter().map(|&f| ctx.model_info.filament_info[f as usize].clone()).collect();
                o.list(&select_best_group_for_ams(
                    &maps,
                    &ctx.nozzle_info.nozzle_list,
                    &used,
                    &info,
                    &ctx.machine_info.machine_filament_info,
                    sw,
                    thr,
                ));
            }
            "mmn" => match calc_filament_group_for_match_multi_nozzle(&ctx) {
                Ok(r) => o.list(&r),
                Err(e) => o.exc(&e),
            },
            "man" => {
                let man = t.list_i32();
                match calc_filament_group_for_manual_multi_nozzle(&man, &ctx) {
                    Ok(r) => o.list(&r),
                    Err(e) => o.exc(&e),
                }
            }
            "cnm" => {
                let mut c2 = ctx.clone();
                match calc_filament_group_for_nozzle_match(&mut c2) {
                    Ok(r) => {
                        o.list(&r);
                        write_ctx_state(&mut o, &c2);
                    }
                    Err(e) => o.exc(&e),
                }
            }
            "pfn" => match plan_filament_nozzle_mapping_and_order(&ctx) {
                Ok(res) => {
                    o.i(res.len() as i64);
                    for r in &res {
                        o.list(&r.fil_order);
                        o.list(&r.fil_nozzle_match);
                    }
                }
                Err(e) => o.exc(&e),
            },
            "tpu" => {
                let mut fg = FilamentGroup::new(&ctx);
                let r = fg.calc_filament_group_for_tpu(None);
                o.list(&r);
                write_ctx_state(&mut o, &fg.ctx);
            }
            other => panic!("line {}: unknown op {other}", lineno + 1),
        }
        *counts.entry(op.clone()).or_insert(0) += 1;
        let got = o.join();
        if got != expected {
            failures.push(format!("line {} {op}:\n  lhs {}\n  C++  {}\n  Rust {}", lineno + 1, &lhs[..lhs.len().min(300)], expected, got));
        }
    }
    eprintln!("cases per op: {:?}", counts);
    if !failures.is_empty() {
        let mut per_op: BTreeMap<String, usize> = BTreeMap::new();
        for f in &failures {
            let op = f.split_whitespace().nth(2).unwrap_or("").trim_end_matches(':').to_string();
            *per_op.entry(op).or_insert(0) += 1;
        }
        for f in failures.iter().take(12) {
            eprintln!("{f}");
        }
        if let Ok(path) = std::env::var("FILAMENT_GROUP_FAILURES") {
            std::fs::write(&path, failures.join("\n")).expect("write the failures");
        }
        panic!("{} of the cases differ: {:?}", failures.len(), per_op);
    }
}
