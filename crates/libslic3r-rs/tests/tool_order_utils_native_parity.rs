//! `slicer::gcode::tool_order_utils` (the port of libslic3r/GCode/ToolOrderUtils.cpp) against the
//! C++ at the pin.
//!
//! `tests/data/tool_order_utils/vectors.txt.gz` is the output of `gen.cpp` beside it (gzip -9n),
//! which compiles the pin's ToolOrderUtils.cpp into itself, with the `DEBUG_MULTI_NOZZLE_MCMF`
//! block switched on, and drives the flow solvers, the order solvers of one nozzle and the
//! reorder functions on random flush matrices, layers, custom sequences and nozzle groupings. The
//! header of gen.cpp describes the lines. Every operation is repeated here on the Rust port, and
//! what it returns has to be the same: floats bit for bit, integers equal.

use slicer::gcode::tool_order_utils::*;
use slicer::multi_nozzle_utils::{CreateFromNozzleMap, LayeredNozzleGroupResult, NozzleInfo, NozzleStatusRecorder};
use std::collections::{BTreeMap, BTreeSet, HashMap};
use std::io::Read;

fn read_vectors() -> String {
    let path = format!("{}/tests/data/tool_order_utils/vectors.txt.gz", env!("CARGO_MANIFEST_DIR"));
    let file = std::fs::File::open(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
    let mut text = String::new();
    flate2::read::GzDecoder::new(file).read_to_string(&mut text).expect("gzip");
    text
}

fn ff(v: f32) -> String {
    format!("{:08x}", v.to_bits())
}

/// The cost a solver leaves alone is the value it had (gen.cpp, COST_UNSET).
const COST_UNSET: f32 = -1.0;

type Limits = HashMap<i32, Vec<i32>>;
type CustomSeq = BTreeMap<i32, Vec<i32>>;

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
    fn i32(&mut self) -> i32 {
        self.i() as i32
    }
    fn u(&mut self) -> usize {
        self.i() as usize
    }
    fn b(&mut self) -> bool {
        self.i() != 0
    }
    fn f(&mut self) -> f32 {
        f32::from_bits(u32::from_str_radix(self.s(), 16).expect("float"))
    }
    fn list_u32(&mut self) -> Vec<u32> {
        let n = self.u();
        (0..n).map(|_| self.i() as u32).collect()
    }
    fn list_i32(&mut self) -> Vec<i32> {
        let n = self.u();
        (0..n).map(|_| self.i() as i32).collect()
    }
    fn mat(&mut self) -> FlushMatrix {
        let n = self.u();
        (0..n)
            .map(|_| {
                let m = self.u();
                (0..m).map(|_| self.f()).collect()
            })
            .collect()
    }
    fn mats(&mut self) -> Vec<FlushMatrix> {
        let n = self.u();
        (0..n).map(|_| self.mat()).collect()
    }
    fn limits(&mut self) -> Limits {
        let n = self.u();
        (0..n)
            .map(|_| {
                let k = self.i32();
                (k, self.list_i32())
            })
            .collect()
    }
    fn group_capacity(&mut self) -> Vec<(BTreeSet<i32>, i32)> {
        let n = self.u();
        (0..n)
            .map(|_| {
                let s: BTreeSet<i32> = self.list_i32().into_iter().collect();
                (s, self.i32())
            })
            .collect()
    }
    fn layers(&mut self) -> Vec<Vec<u32>> {
        let n = self.u();
        (0..n).map(|_| self.list_u32()).collect()
    }
    fn start(&mut self) -> Option<u32> {
        let v = self.i();
        (v >= 0).then_some(v as u32)
    }
    /// `None`: no function.
    fn custom(&mut self) -> Option<CustomSeq> {
        assert_eq!(self.s(), "Q");
        let n = self.i();
        if n < 0 {
            return None;
        }
        Some(
            (0..n)
                .map(|_| {
                    let l = self.i32();
                    (l, self.list_i32())
                })
                .collect(),
        )
    }
    fn arrow(&mut self) {
        assert_eq!(self.s(), "=>");
    }
    fn rest(&mut self) -> Vec<String> {
        let r = self.t[self.i..].iter().map(|s| s.to_string()).collect();
        self.i = self.t.len();
        r
    }
}

/// What the C++ writes, token by token.
#[derive(Default)]
struct Out(Vec<String>);

impl Out {
    fn i(&mut self, v: i64) {
        self.0.push(v.to_string());
    }
    fn f(&mut self, v: f32) {
        self.0.push(ff(v));
    }
    fn list<T: Copy + Into<i64>>(&mut self, v: &[T]) {
        self.i(v.len() as i64);
        for &x in v {
            self.i(x.into());
        }
    }
    fn layers(&mut self, ls: &[Vec<u32>]) {
        self.i(ls.len() as i64);
        for l in ls {
            self.list(l);
        }
    }
    fn matching(&mut self, m: &[Vec<i32>]) {
        self.i(m.len() as i64);
        for l in m {
            self.list(l);
        }
    }
}

/// The function gen.cpp's custom_fn() builds.
fn custom_fn(c: &CustomSeq) -> impl Fn(i32, &mut Vec<i32>) -> bool + '_ {
    move |layer: i32, seq: &mut Vec<i32>| match c.get(&layer) {
        None => false,
        Some(s) => {
            *seq = s.clone();
            true
        }
    }
}

/// Runs every line of `op` through `run` and compares what it writes with the line's outputs.
fn check(op: &str, mut run: impl FnMut(&mut Toks, &mut Out)) -> usize {
    let text = read_vectors();
    let mut n = 0usize;
    let mut bad = 0usize;
    let mut first = String::new();
    for (ln, line) in text.lines().enumerate() {
        if line.split(' ').next() != Some(op) {
            continue;
        }
        let mut t = Toks::new(line);
        t.s();
        let mut out = Out::default();
        run(&mut t, &mut out);
        let want = t.rest();
        n += 1;
        if want != out.0 {
            bad += 1;
            if first.is_empty() {
                first = format!("line {}: C++ {:?}\n         Rust {:?}", ln + 1, want, out.0);
            }
        }
    }
    assert!(n > 0, "{op}: no vectors");
    assert_eq!(bad, 0, "{op}: {bad} of {n} differ; first: {first}");
    n
}

#[test]
fn max_flow_solver() {
    let n = check("max_flow_solver", |t, out| {
        let (u, v) = (t.list_i32(), t.list_i32());
        let (link, unlink) = (t.limits(), t.limits());
        let (ucap, vcap) = (t.list_i32(), t.list_i32());
        let gcap = t.group_capacity();
        t.arrow();
        let mut s = MaxFlowSolver::new(&u, &v, &link, &unlink, &ucap, &vcap, &gcap);
        out.list(&s.solve());
    });
    assert_eq!(n, 800);
}

#[test]
fn general_min_cost_solver() {
    let n = check("general_min_cost_solver", |t, out| {
        let m = t.mat();
        let (u, v) = (t.list_i32(), t.list_i32());
        t.arrow();
        let mut s = GeneralMinCostSolver::new(&m, &u, &v);
        out.list(&s.solve());
    });
    assert_eq!(n, 600);
}

struct GroupIn {
    ms: Vec<FlushMatrix>,
    u: Vec<i32>,
    v: Vec<i32>,
    vg: Vec<i32>,
    link: Limits,
    unlink: Limits,
}
fn group_in(t: &mut Toks) -> GroupIn {
    let ms = t.mats();
    let (u, v, vg) = (t.list_i32(), t.list_i32(), t.list_i32());
    let (link, unlink) = (t.limits(), t.limits());
    t.arrow();
    GroupIn { ms, u, v, vg, link, unlink }
}

#[test]
fn general_min_cost_lower_bounds_solver() {
    let n = check("general_min_cost_lower_bounds_solver", |t, out| {
        let g = group_in(t);
        let mut s = GeneralMinCostLowerBoundsSolver::new(&g.ms, &g.u, &g.v, &g.vg, &g.link, &g.unlink);
        out.list(&s.solve());
    });
    assert_eq!(n, 600);
}

#[test]
fn group_min_cost_flow_solver() {
    let n = check("group_min_cost_flow_solver", |t, out| {
        let g = group_in(t);
        let mut s = GroupMinCostFlowSolver::new(&g.ms, &g.u, &g.v, &g.vg, &g.link, &g.unlink);
        out.list(&s.solve());
    });
    assert_eq!(n, 600);
}

#[test]
fn min_flush_flow_solver() {
    let n = check("min_flush_flow_solver", |t, out| {
        let m = t.mat();
        let (u, v) = (t.list_i32(), t.list_i32());
        let (link, unlink) = (t.limits(), t.limits());
        let (ucap, vcap) = (t.list_i32(), t.list_i32());
        let gcap = t.group_capacity();
        t.arrow();
        let mut s = MinFlushFlowSolver::new(&m, &u, &v, &link, &unlink, &ucap, &vcap, &gcap);
        out.list(&s.solve());
    });
    assert_eq!(n, 800);
}

#[test]
fn match_mode_group_solver() {
    let n = check("match_mode_group_solver", |t, out| {
        let m = t.mat();
        let (u, v, vcap) = (t.list_i32(), t.list_i32(), t.list_i32());
        let unlink = t.limits();
        t.arrow();
        let mut s = MatchModeGroupSolver::new(&m, &u, &v, &vcap, &unlink);
        out.list(&s.solve());
    });
    assert_eq!(n, 600);
}

#[test]
fn solve_extruder_order_with_greedy_() {
    let n = check("solve_extruder_order_with_greedy", |t, out| {
        let m = t.mat();
        let curr = t.list_u32();
        let start = t.start();
        t.arrow();
        let mut cost = COST_UNSET;
        out.list(&solve_extruder_order_with_greedy(&m, &curr, start, Some(&mut cost)));
        out.f(cost);
    });
    assert_eq!(n, 600);
}

#[test]
fn solve_extruder_order_with_forcast_() {
    let n = check("solve_extruder_order_with_forcast", |t, out| {
        let m = t.mat();
        let (curr, next) = (t.list_u32(), t.list_u32());
        let start = t.start();
        t.arrow();
        let mut cost = COST_UNSET;
        out.list(&solve_extruder_order_with_forcast(&m, curr, next, start, Some(&mut cost)));
        out.f(cost);
    });
    assert_eq!(n, 500);
}

#[test]
fn solve_extruder_order_() {
    let n = check("solve_extruder_order", |t, out| {
        let m = t.mat();
        let curr = t.list_u32();
        let start = t.start();
        t.arrow();
        let mut cost = COST_UNSET;
        out.list(&solve_extruder_order(&m, curr, start, Some(&mut cost)));
        out.f(cost);
    });
    assert_eq!(n, 500);
}

#[test]
fn get_extruders_order_() {
    let n = check("get_extruders_order", |t, out| {
        let m = t.mat();
        let (curr, next) = (t.list_u32(), t.list_u32());
        let start = t.start();
        let (forcast, with_cost) = (t.b(), t.b());
        t.arrow();
        let mut cost = COST_UNSET;
        out.list(&get_extruders_order(&m, &curr, &next, start, forcast, with_cost.then_some(&mut cost)));
        out.f(cost);
    });
    assert_eq!(n, 700);
}

#[test]
fn reorder_filaments_for_minimum_flush_volume_base_() {
    let n = check("reorder_filaments_for_minimum_flush_volume_base", |t, out| {
        let list = t.list_u32();
        let layers = t.layers();
        let m = t.mat();
        let custom = t.custom();
        let initial = t.start();
        let with_seq = t.b();
        t.arrow();
        let mut seqs: Vec<Vec<u32>> = Vec::new();
        let f = custom.as_ref().map(custom_fn);
        let cost = reorder_filaments_for_minimum_flush_volume_base(
            &list,
            &layers,
            &m,
            f.as_ref().map(|f| f as &dyn Fn(i32, &mut Vec<i32>) -> bool),
            with_seq.then_some(&mut seqs),
            initial,
        );
        out.i(cost as i64);
        out.layers(&seqs);
    });
    assert_eq!(n, 500);
}

#[test]
fn reorder_filaments_for_minimum_flush_volume_() {
    let n = check("reorder_filaments_for_minimum_flush_volume", |t, out| {
        let list = t.list_u32();
        let maps = t.list_i32();
        let layers = t.layers();
        let ms = t.mats();
        let custom = t.custom();
        let ns = t.u();
        let status: HashMap<i32, i32> = (0..ns)
            .map(|_| {
                let g = t.i32();
                (g, t.i32())
            })
            .collect();
        let with_seq = t.b();
        t.arrow();
        let mut seqs: Vec<Vec<u32>> = Vec::new();
        let f = custom.as_ref().map(custom_fn);
        let cost = reorder_filaments_for_minimum_flush_volume(
            &list,
            &maps,
            &layers,
            &ms,
            f.as_ref().map(|f| f as &dyn Fn(i32, &mut Vec<i32>) -> bool),
            with_seq.then_some(&mut seqs),
            &status,
        );
        out.i(cost as i64);
        out.layers(&seqs);
    });
    assert_eq!(n, 700);
}

#[test]
fn reorder_filaments_for_multi_nozzle_extruder_() {
    let n = check("reorder_filaments_for_multi_nozzle_extruder", |t, out| {
        let list = t.list_u32();
        let map = t.list_i32();
        let nn = t.u();
        let nozzles: Vec<NozzleInfo> = (0..nn)
            .map(|_| {
                let mut ni = NozzleInfo::default();
                ni.diameter = "0.4".to_string();
                ni.volume_type = slicer::extruder::NozzleVolumeType::from_i32(t.i32());
                ni.extruder_id = t.i32();
                ni.group_id = t.i32();
                ni
            })
            .collect();
        let used = t.list_u32();
        let as_extruder = t.b();
        let layers = t.layers();
        let ms = t.mats();
        let custom = t.custom();
        let mut recorder = NozzleStatusRecorder::new();
        for _ in 0..t.u() {
            let (nozzle, filament, extruder) = (t.i32(), t.i32(), t.i32());
            recorder.set_nozzle_status(nozzle, filament, extruder);
        }
        let rec_ext = t.i32();
        if rec_ext != -2 {
            recorder.set_current_extruder_id(rec_ext);
        }
        let with_seq = t.b();
        t.arrow();
        let group = <LayeredNozzleGroupResult as CreateFromNozzleMap>::create(&map, &nozzles, &used, as_extruder)
            .expect("the generator's grouping exists");
        let mut seqs: Vec<Vec<u32>> = Vec::new();
        let f = custom.as_ref().map(custom_fn);
        let cost = reorder_filaments_for_multi_nozzle_extruder(
            &list,
            &group,
            &layers,
            &ms,
            f.as_ref().map(|f| f as &dyn Fn(i32, &mut Vec<i32>) -> bool),
            with_seq.then_some(&mut seqs),
            &recorder,
        );
        out.i(cost as i64);
        out.layers(&seqs);
    });
    assert_eq!(n, 700);
}

#[test]
fn debug_solve_extruder_order_with_mcmf() {
    let n = check("solve_extruder_order_with_mcmf", |t, out| {
        let m = t.mat();
        let (curr, state) = (t.list_i32(), t.list_i32());
        let by_name = t.b();
        t.arrow();
        let mut cost = COST_UNSET;
        let ret = if by_name {
            debug_multi_nozzle_mcmf::get_extruders_order(&m, &curr, &state, Some(&mut cost))
        } else {
            debug_multi_nozzle_mcmf::solve_extruder_order_with_mcmf(&m, &curr, &state, Some(&mut cost))
        };
        out.matching(&ret);
        out.f(cost);
    });
    assert_eq!(n, 500);
}

#[test]
fn debug_get_nozzle_match_in_given_order() {
    let n = check("get_nozzle_match_in_given_order", |t, out| {
        let m = t.mat();
        let seq = t.list_u32();
        let state = t.list_i32();
        t.arrow();
        let mut cost = COST_UNSET;
        out.matching(&debug_multi_nozzle_mcmf::get_nozzle_match_in_given_order(&m, &seq, &state, Some(&mut cost)));
        out.f(cost);
    });
    assert_eq!(n, 500);
}

#[test]
fn debug_reorder_filaments_for_multi_nozzle_extruder() {
    let n = check("debug_reorder_filaments_for_multi_nozzle_extruder", |t, out| {
        let list = t.list_u32();
        let maps = t.list_i32();
        let layers = t.layers();
        let ms = t.mats();
        let custom = t.custom();
        let (multi_id, multi_num) = (t.i32(), t.i32());
        let (with_seq, with_match) = (t.b(), t.b());
        t.arrow();
        let mut seqs: Vec<Vec<u32>> = Vec::new();
        let mut matches: Vec<Vec<Vec<i32>>> = Vec::new();
        let f = custom.as_ref().map(custom_fn);
        let cost = debug_multi_nozzle_mcmf::reorder_filaments_for_multi_nozzle_extruder(
            &list,
            &maps,
            &layers,
            &ms,
            f.as_ref().map(|f| f as &dyn Fn(i32, &mut Vec<i32>) -> bool),
            multi_id,
            multi_num,
            with_seq.then_some(&mut seqs),
            with_match.then_some(&mut matches),
        );
        out.i(cost as i64);
        out.layers(&seqs);
        out.i(matches.len() as i64);
        for m in &matches {
            out.matching(m);
        }
    });
    assert_eq!(n, 500);
}
