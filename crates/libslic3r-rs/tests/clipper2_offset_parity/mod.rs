//! What `clipper2_offset_native_parity.rs` (Clipper2 without Z) and
//! `clipper2_offset_z_native_parity.rs` (with `USINGZ`) share: the deterministic generator of
//! the cases, the calls into the C++ reference (`cz2_ref_xy_*` / `cz2_ref_z_*` of
//! `clipper2-z-sys`, built from the Clipper2 the engine pins), the comparison, and the tests
//! themselves (`offset_parity_tests!`, which each of the two files expands for its build).
//!
//! The shapes come from the generator of the engine tests (`clipper2_engine_parity/mod.rs`).
//! A case is made from its own seed, so a failure names the case and prints its input. The
//! environment variables of the engine tests work here too: `CLIPPER2_PARITY_LOG=<file>`
//! appends every failing case with its input and both outputs, `CLIPPER2_PARITY_SUMMARY=<file>`
//! appends the counts of every category, `CLIPPER2_PARITY_ONLY=<category>#<index>` runs one
//! case, `CLIPPER2_PARITY_SCALE=<k>` runs k times as many cases.
#![allow(dead_code)]

#[path = "../clipper2_engine_parity/mod.rs"]
pub mod engine;

pub use engine::{below, between, case_seed, chance, mix, r, xorshift, Flat, Path, Paths, Pt, HASH_START, MAX_COORD};

use clipper2_z_sys as sys;
use std::fmt::Write as _;
use std::io::Write as _;

/// The paths of one `AddPaths` (or `AddPath`), with their JoinType (0 Square, 1 Bevel,
/// 2 Round, 3 Miter; clipper.offset.h:23) and EndType (0 Polygon, 1 Joined, 2 Butt,
/// 3 Square, 4 Round; clipper.offset.h:27).
#[derive(Clone, Debug)]
pub struct Group {
    pub paths: Paths,
    pub join: i32,
    pub end: i32,
}

#[derive(Clone, Debug)]
pub struct OffsetCase {
    pub name: String,
    pub groups: Vec<Group>,
    pub delta: f64,
    pub delta2: f64,
    pub miter_limit: f64,
    pub arc_tolerance: f64,
    pub preserve_collinear: bool,
    pub reverse_solution: bool,
    pub use_tree: bool,
    /// See `cz2_ref_*_offset` (clipper2_ref_shim.cpp): bit 0 AddPath for a group of one,
    /// bit 1 an Execute(delta2) before, bit 2 InflatePaths, bit 3 a second stage with delta2.
    pub mode: i32,
    /// 0 none, 1..4 the delta callbacks of clipper2_ref_shim.cpp; bit 4: through
    /// Execute(DeltaCallback64, paths).
    pub delta_cb: i32,
    /// 0 none, 2 one that hashes all it is given.
    pub z_callback: i32,
}

/// The number of (JoinType, EndType, paths or tree) combinations; case `i` of a category
/// takes combination `i % COMBOS`.
pub const COMBOS: u64 = 4 * 5 * 2;

/// The largest extent of the paths along an axis.
fn extent(paths: &Paths) -> f64 {
    let (mut lo, mut hi) = ([i64::MAX; 2], [i64::MIN; 2]);
    for p in paths {
        for q in p {
            for a in 0..2 {
                lo[a] = lo[a].min(q[a]);
                hi[a] = hi[a].max(q[a]);
            }
        }
    }
    if lo[0] > hi[0] {
        return 1.0;
    }
    ((hi[0] as f64 - lo[0] as f64).max(hi[1] as f64 - lo[1] as f64)).max(1.0)
}

/// A delta for paths of extent `size`: zero, the insignificant ones around 0.5, whole and
/// fractional ones from a hundredth of the extent to twice the extent, of both signs.
fn pick_delta(size: f64, s: &mut u64) -> f64 {
    let f = 0.5 + below(1000, s) as f64 / 1000.0;
    let d = match below(18, s) {
        0 => 0.0,
        1 => 0.3,
        2 => 0.5,
        3 => 1.0,
        4 => 1.0 + below(9000, s) as f64 / 1000.0,
        5 | 6 => size * 0.01 * f,
        7 | 8 | 9 => size * 0.1 * f,
        10 | 11 => size * 0.5 * f,
        12 => size * 2.0 * f,
        13 => (size * 0.05 * f).round(),
        14 => size * 1e-3 * f,
        15 => [0.49999, 0.50001, 0.9999, 1.0001][below(4, s) as usize],
        16 => (size * 0.3 * f).round() + 0.5,
        _ => 2.0 + below(40, s) as f64,
    };
    if chance(1, 2, s) {
        -d
    } else {
        d
    }
}

fn pick_miter_limit(s: &mut u64) -> f64 {
    [2.0, 2.0, 0.0, 1.0, 1.2, 1.5, 3.0, 10.0, 100.0][below(9, s) as usize]
}

/// An arc tolerance for `delta`: none, below the tolerance at which the library takes its
/// own, a fixed one, fractions of the delta and more than the delta. One that is used is
/// kept above a ten-thousandth of the delta (222 steps to the circle).
fn pick_arc_tolerance(delta: f64, s: &mut u64) -> f64 {
    let d = delta.abs();
    let a = [0.0, 0.0, 0.0, 1e-13, 0.25, d * 0.05, d * 0.001, d * 2.0, 1.0, d * 0.3][below(10, s) as usize];
    if a > 1e-12 && a < d * 1e-4 {
        d * 1e-4
    } else {
        a
    }
}

fn base(name: &str, index: u64, paths: Paths, size_cap: f64, s: &mut u64) -> OffsetCase {
    let c = index % COMBOS;
    let size = extent(&paths).min(size_cap);
    let delta = pick_delta(size, s);
    OffsetCase {
        name: format!("{name}#{index}"),
        groups: vec![Group { paths, join: (c % 4) as i32, end: ((c / 4) % 5) as i32 }],
        delta,
        delta2: pick_delta(size, s),
        miter_limit: pick_miter_limit(s),
        arc_tolerance: pick_arc_tolerance(delta, s),
        preserve_collinear: chance(1, 4, s),
        reverse_solution: chance(1, 4, s),
        use_tree: (c / 20) % 2 == 1,
        mode: if chance(1, 2, s) { 0 } else { below(2, s) as i32 },
        delta_cb: 0,
        z_callback: 0,
    }
}

/// The closed and open paths of a case of the engine tests, without the empty ones (an
/// open path without points is undefined behaviour in the C++ offsetter).
fn all_paths(case: &engine::Case) -> Paths {
    case.subj.iter().chain(case.clip.iter()).chain(case.open.iter()).filter(|p| !p.is_empty()).cloned().collect()
}

/// Closed polygons at the magnitudes 1e2 .. 1e9; one case in eight goes through InflatePaths.
pub fn gen_closed(index: u64, s: &mut u64) -> OffsetCase {
    let paths = all_paths(&engine::gen_random(index, s));
    let mut case = base("closed_polygons", index, paths, 1e18, s);
    if index / COMBOS % 8 == 7 {
        case.mode = 4;
        case.use_tree = false;
        case.preserve_collinear = false;
        case.reverse_solution = false;
    }
    case
}

/// Outers with holes with outers in them.
pub fn gen_holes(index: u64, s: &mut u64) -> OffsetCase {
    let paths = all_paths(&engine::gen_nested(index, s));
    base("holes_nested", index, paths, 1e18, s)
}

/// Open paths: lines, polylines, loops given as open paths, paths that go back on
/// themselves.
pub fn gen_open(index: u64, s: &mut u64) -> OffsetCase {
    let e = engine::gen_open(index, s);
    let mut paths: Paths = e.open.iter().filter(|p| !p.is_empty()).cloned().collect();
    if paths.is_empty() {
        paths.push(vec![[0, 0, 0], [100, 30, 0]]);
    }
    base("open_paths", index, paths, 1e18, s)
}

/// One- and two-point paths, duplicate points, collinear runs, spikes, and (for the
/// Polygon end type) paths without points.
pub fn gen_degenerate(index: u64, s: &mut u64) -> OffsetCase {
    let mag = [3i64, 20, 1000, 1_000_000][(index / COMBOS % 4) as usize];
    let end = (index % COMBOS / 4) % 5;
    let mut paths = Paths::new();
    for _ in 0..between(1, 4, s) {
        let p = |s: &mut u64| -> Pt { [r(mag, s), r(mag, s), 0] };
        let path: Path = match below(12, s) {
            0 => vec![p(s)],
            1 => {
                let a = p(s);
                vec![a, a, a]
            }
            2 => vec![p(s), p(s)],
            3 => {
                let (a, b) = (p(s), p(s));
                vec![a, a, b, b, a]
            }
            // a collinear run with points that go back
            4 | 5 => {
                let (a, d) = (p(s), [r(5, s), r(5, s)]);
                (0..between(2, 7, s))
                    .map(|_| {
                        let t = r(6, s);
                        [a[0] + t * d[0], a[1] + t * d[1], 0]
                    })
                    .collect()
            }
            // a spike: out and back along one line
            6 => {
                let (a, b, c) = (p(s), p(s), p(s));
                vec![a, b, c, b]
            }
            // a polygon with every point twice, and the first again at the end
            7 => {
                let n = between(3, 6, s);
                let mut v = Path::new();
                for _ in 0..n {
                    let a = p(s);
                    v.push(a);
                    v.push(a);
                }
                v.push(v[0]);
                v
            }
            // axis-parallel segments with collinear points in between
            8 => {
                let (x0, y0, w, h) = (r(mag, s), r(mag, s), between(1, mag.max(2), s), between(1, mag.max(2), s));
                vec![[x0, y0, 0], [x0 + w / 2, y0, 0], [x0 + w, y0, 0], [x0 + w, y0 + h, 0], [x0 + w / 3, y0 + h, 0], [x0, y0 + h, 0], [x0, y0 + h / 2, 0]]
            }
            // nothing at all (with the Polygon end type only)
            9 if end == 0 => Path::new(),
            // nearly straight joins
            10 => {
                let (x0, y0) = (r(mag, s), r(mag, s));
                (0..between(3, 8, s)).map(|i| [x0 + i * mag.max(50), y0 + r(1, s), 0]).collect()
            }
            _ => (0..between(3, 7, s)).map(|_| p(s)).collect(),
        };
        paths.push(path);
    }
    let mut case = base("degenerate", index, paths, 1e18, s);
    // single points want deltas of 1 and more to give anything
    if chance(1, 3, s) {
        case.delta = [1.0, 1.5, 0.99, 7.25, -3.0, 40.0, 2.5][below(7, s) as usize];
        case.arc_tolerance = pick_arc_tolerance(case.delta, s);
    }
    case
}

/// Coordinates up to 2^60, with deltas up to 2^40.
pub fn gen_huge(index: u64, s: &mut u64) -> OffsetCase {
    let e = engine::gen_huge(index, s);
    let mut paths = all_paths(&e);
    for p in paths.iter_mut() {
        for q in p.iter_mut() {
            q[0] /= 2;
            q[1] /= 2;
        }
    }
    base("huge_coordinates", index, paths, (1u64 << 40) as f64, s)
}

/// Shapes a few units across, where every rounding shows.
pub fn gen_tiny(index: u64, s: &mut u64) -> OffsetCase {
    let k = [2i64, 4, 8, 30][(index / COMBOS % 4) as usize];
    let mut paths = Paths::new();
    for _ in 0..between(1, 3, s) {
        paths.push((0..between(2, 8, s)).map(|_| [r(k, s), r(k, s), 0]).collect());
    }
    let mut case = base("tiny", index, paths, 1e18, s);
    if chance(1, 2, s) {
        case.delta = [0.5, 0.75, 1.0, 1.25, 2.0, 3.5, -0.5, -1.0, -2.0, 0.6][below(10, s) as usize];
        case.arc_tolerance = pick_arc_tolerance(case.delta, s);
    }
    case
}

/// Coincident and collinear edges, touching and overlapping polygons, horizontal edges.
pub fn gen_coincident(index: u64, s: &mut u64) -> OffsetCase {
    let e = match index / COMBOS % 3 {
        0 => engine::gen_coincident(index, s),
        1 => engine::gen_touching(index, s),
        _ => engine::gen_horizontal(index, s),
    };
    base("coincident_and_collinear", index, all_paths(&e), 1e18, s)
}

/// Self-intersecting polygons and dense intersections.
pub fn gen_self_intersecting(index: u64, s: &mut u64) -> OffsetCase {
    let e = if index / COMBOS % 4 == 3 { engine::gen_dense(index, s) } else { engine::gen_self_intersecting(index, s) };
    base("self_intersecting", index, all_paths(&e), 1e18, s)
}

/// Two or three groups, each with its own join and end type and orientation, on one
/// offsetter; an Execute before the one asked for.
pub fn gen_multi_group(index: u64, s: &mut u64) -> OffsetCase {
    let e = match index / COMBOS % 3 {
        0 => engine::gen_random(index, s),
        1 => engine::gen_nested(index, s),
        _ => engine::gen_open(index, s),
    };
    let paths = all_paths(&e);
    let mut case = base("multi_group", index, paths.clone(), 1e18, s);
    let first = case.groups[0].clone();
    case.groups.clear();
    let n_groups = between(2, 3, s) as usize;
    for g in 0..n_groups {
        let mut ps: Paths = paths.iter().enumerate().filter(|(i, _)| i % n_groups == g).map(|(_, p)| p.clone()).collect();
        if ps.is_empty() && chance(1, 2, s) {
            ps.push(vec![[r(1000, s), r(1000, s), 0], [r(1000, s), r(1000, s), 0], [r(1000, s), r(1000, s), 0]]);
        }
        if chance(1, 3, s) {
            for p in ps.iter_mut() {
                p.reverse();
            }
        }
        let (join, end) = if g == 0 { (first.join, first.end) } else { (below(4, s) as i32, below(5, s) as i32) };
        case.groups.push(Group { paths: ps, join, end });
    }
    case.mode = below(4, s) as i32;
    case
}

/// A delta that changes from vertex to vertex.
pub fn gen_delta_callback(index: u64, s: &mut u64) -> OffsetCase {
    let e = match index / COMBOS % 3 {
        0 => engine::gen_random(index, s),
        1 => engine::gen_open(index, s),
        _ => engine::gen_nested(index, s),
    };
    let mut paths = all_paths(&e);
    if chance(1, 6, s) {
        paths.push(vec![[r(1000, s), r(1000, s), 0]]);
    }
    let mut case = base("delta_callback", index, paths, 1e18, s);
    case.delta_cb = 1 + (index / COMBOS % 4) as i32;
    if !case.use_tree && chance(1, 3, s) {
        case.delta_cb |= 16;
    }
    if case.delta == 0.0 {
        case.delta = 5.0;
    }
    case.arc_tolerance = pick_arc_tolerance(case.delta * 0.5, s);
    case
}

/// Out and in again, as `offset2_ex_2` of the engine does: the solution of the first run
/// (the paths of its tree) goes back into the cleared offsetter.
pub fn gen_two_stage(index: u64, s: &mut u64) -> OffsetCase {
    let e = if index / COMBOS % 2 == 0 { engine::gen_random(index, s) } else { engine::gen_nested(index, s) };
    let mut case = base("two_stage", index, all_paths(&e), 1e18, s);
    case.mode = 8 | (case.mode & 1);
    if case.delta == 0.0 {
        case.delta = 3.0;
    }
    case.delta2 = -case.delta * (0.3 + below(1200, s) as f64 / 1000.0);
    case
}

/// Gives the input its z and the offsetter a callback.
pub fn assign_z(case: &mut OffsetCase, index: u64, s: &mut u64) {
    let mode = index / COMBOS % 5;
    case.z_callback = if (index / (5 * COMBOS)) % 2 == 0 { 2 } else { 0 };
    let mut path_no = 0i64;
    for g in case.groups.iter_mut() {
        for path in g.paths.iter_mut() {
            path_no += 1;
            for (vi, p) in path.iter_mut().enumerate() {
                p[2] = match mode {
                    0 => 0,
                    // every vertex its own
                    1 => (path_no << 32) | vi as i64,
                    // one per path, as the engine gives a contour its index
                    2 => path_no,
                    // a few small values, zero among them: runs of equal z along a path
                    3 => r(2, s),
                    _ => xorshift(s) as i64,
                };
            }
        }
    }
}

// -------------------------------------------------------------------------------------------
// The reference
// -------------------------------------------------------------------------------------------

fn flatten(paths: &Paths) -> (Vec<i64>, Vec<i32>) {
    let mut xyz = Vec::new();
    let mut lens = Vec::new();
    for p in paths {
        lens.push(p.len() as i32);
        for q in p {
            xyz.extend_from_slice(q);
        }
    }
    (xyz, lens)
}

fn read_result(res: sys::Cz2RefResult, tree: bool, z: bool) -> Flat {
    let slice = |p: *mut i64, n: usize| if p.is_null() || n == 0 { Vec::new() } else { unsafe { std::slice::from_raw_parts(p, n) }.to_vec() };
    let slice32 = |p: *mut i32, n: usize| if p.is_null() || n == 0 { Vec::new() } else { unsafe { std::slice::from_raw_parts(p, n) }.to_vec() };
    let paths = (res.num_closed + res.num_open) as usize;
    let nodes = if tree { res.num_closed as usize } else { 0 };
    let out = Flat {
        coords: slice(res.coords, 3 * res.total_points as usize),
        lens: slice32(res.path_lens, paths),
        parents: slice32(res.tree_parent, nodes),
        holes: slice32(res.tree_is_hole, nodes),
        num_closed: res.num_closed,
        num_open: res.num_open,
        succeeded: res.succeeded == 1,
        cb_calls: res.cb_calls,
        cb_hash: res.cb_hash,
    };
    unsafe {
        if z {
            sys::cz2_ref_z_free(res)
        } else {
            sys::cz2_ref_xy_free(res)
        }
    };
    out
}

/// `ClipperOffset` of the C++ reference, with `USINGZ` (`z`) or without.
pub fn cpp_offset(case: &OffsetCase, z: bool) -> Flat {
    let all: Paths = case.groups.iter().flat_map(|g| g.paths.iter().cloned()).collect();
    let (xyz, lens) = flatten(&all);
    let sizes: Vec<i32> = case.groups.iter().map(|g| g.paths.len() as i32).collect();
    let joins: Vec<i32> = case.groups.iter().map(|g| g.join).collect();
    let ends: Vec<i32> = case.groups.iter().map(|g| g.end).collect();
    let f = if z { sys::cz2_ref_z_offset } else { sys::cz2_ref_xy_offset };
    let res = unsafe {
        f(
            xyz.as_ptr(),
            lens.as_ptr(),
            sizes.as_ptr(),
            joins.as_ptr(),
            ends.as_ptr(),
            sizes.len() as i32,
            case.delta,
            case.delta2,
            case.miter_limit,
            case.arc_tolerance,
            case.preserve_collinear as i32,
            case.reverse_solution as i32,
            case.use_tree as i32,
            case.mode,
            case.delta_cb,
            case.z_callback,
        )
    };
    read_result(res, case.use_tree && case.mode & 4 == 0, z)
}

/// One helper of clipper.h on one path through the C++ reference: the path that comes
/// back (if the helper gives one) and the four doubles of op 5, as bits.
pub fn cpp_path_op(op: i32, path: &Path, a: f64, b: f64, flag: i64, z: bool) -> (Flat, [u64; 4]) {
    let (xyz, _) = flatten(&vec![path.clone()]);
    let mut d = [0.0f64; 4];
    let f = if z { sys::cz2_ref_z_path_op } else { sys::cz2_ref_xy_path_op };
    let res = unsafe { f(op, xyz.as_ptr(), path.len() as i32, a, b, flag, d.as_mut_ptr()) };
    (read_result(res, false, z), d.map(f64::to_bits))
}

/// `BooleanOp` and its wrappers of clipper.h through the C++ reference.
pub fn cpp_boolean_op(op: i32, clip_type: i32, fill_rule: i32, subj: &Paths, clip: &Paths, z: bool) -> Flat {
    let (sx, sl) = flatten(subj);
    let (cx, cl) = flatten(clip);
    let f = if z { sys::cz2_ref_z_boolean_op } else { sys::cz2_ref_xy_boolean_op };
    let res = unsafe { f(op, clip_type, fill_rule, sx.as_ptr(), sl.as_ptr(), sl.len() as i32, cx.as_ptr(), cl.as_ptr(), cl.len() as i32) };
    read_result(res, op == 1, z)
}

/// sin and cos of `x` as the reference's offsetter takes them, then each from a call of
/// its own, as bits.
pub fn cpp_sin_cos(x: f64, z: bool) -> [u64; 4] {
    let mut d = [0.0f64; 4];
    unsafe {
        if z {
            sys::cz2_ref_z_sin_cos(x, d.as_mut_ptr())
        } else {
            sys::cz2_ref_xy_sin_cos(x, d.as_mut_ptr())
        }
    };
    d.map(f64::to_bits)
}

pub fn dump_case(case: &OffsetCase) -> String {
    let mut out = String::new();
    let _ = writeln!(
        out,
        "case {}: delta {:?} delta2 {:?} miter_limit {:?} arc_tolerance {:?} preserve_collinear {} reverse_solution {} use_tree {} mode {} delta_cb {} z_callback {}",
        case.name,
        case.delta,
        case.delta2,
        case.miter_limit,
        case.arc_tolerance,
        case.preserve_collinear,
        case.reverse_solution,
        case.use_tree,
        case.mode,
        case.delta_cb,
        case.z_callback
    );
    for (i, g) in case.groups.iter().enumerate() {
        let _ = writeln!(out, "  group {i}: join {} end {}, {} paths", g.join, g.end, g.paths.len());
        for p in &g.paths {
            let _ = write!(out, "    [");
            for q in p {
                let _ = write!(out, "({},{},{}) ", q[0], q[1], q[2]);
            }
            let _ = writeln!(out, "]");
        }
    }
    out
}

pub fn dump_flat(out: &mut String, name: &str, f: &Flat) {
    let _ = writeln!(out, "  {name}: succeeded {} closed {} open {} cb_calls {} cb_hash {:#x}", f.succeeded, f.num_closed, f.num_open, f.cb_calls, f.cb_hash);
    let mut at = 0usize;
    for (i, l) in f.lens.iter().enumerate() {
        let _ = write!(out, "    path {i}");
        if i < f.parents.len() {
            let _ = write!(out, " (parent {}, hole {})", f.parents[i], f.holes[i]);
        }
        let _ = write!(out, ": [");
        for v in 0..*l as usize {
            let _ = write!(out, "({},{},{}) ", f.coords[at + 3 * v], f.coords[at + 3 * v + 1], f.coords[at + 3 * v + 2]);
        }
        let _ = writeln!(out, "]");
        at += 3 * *l as usize;
    }
}

pub fn append(env: &str, text: &str) {
    if let Ok(path) = std::env::var(env) {
        if let Ok(mut f) = std::fs::OpenOptions::new().create(true).append(true).open(path) {
            let _ = f.write_all(text.as_bytes());
        }
    }
}

// -------------------------------------------------------------------------------------------
// The counters of the Rust side
// -------------------------------------------------------------------------------------------

pub const STAT_NAMES: [&str; 29] = [
    "do_bevel",
    "do_bevel_end",
    "do_square",
    "do_square_end",
    "miter_exceeded",
    "do_miter",
    "almost_straight",
    "do_round",
    "do_round_end",
    "round_steps",
    "concave",
    "same_point",
    "zero_delta",
    "square_parallel",
    "avg_vector_zero",
    "point_circle",
    "point_square",
    "point_skipped",
    "offset_polygon",
    "offset_open_joined",
    "offset_open_path",
    "joined_two_points",
    "groups_reversed",
    "no_lowest_path",
    "insignificant",
    "zcb",
    "zcb_user",
    "delta_callbacks",
    "inflate_paths",
];
pub type Stats = [u64; 29];

#[derive(Debug, Default, Clone)]
pub struct Totals {
    pub cases: u64,
    pub mismatches: u64,
    pub stats: Stats,
    pub nonempty: u64,
    pub paths: u64,
    pub output_points: u64,
    pub tree_cases: u64,
    pub tree_holes: u64,
    pub cb_calls: u64,
    pub nonzero_z_out: u64,
    pub combos: std::collections::BTreeSet<u64>,
    pub positive: u64,
    pub negative: u64,
    pub zero: u64,
}

impl Totals {
    pub fn stat(&self, name: &str) -> u64 {
        self.stats[STAT_NAMES.iter().position(|n| *n == name).unwrap()]
    }
}

/// Runs `n` cases of one category through both sides and compares. Panics at the end with
/// the first differences if there were any. With `expect_mismatches` (the mutation checks)
/// it does not panic and writes nothing to the log.
pub fn run_category(
    name: &str,
    test_id: u64,
    n: u64,
    gen: fn(u64, &mut u64) -> OffsetCase,
    z: bool,
    rust: &dyn Fn(&OffsetCase) -> (Flat, Stats),
    expect_mismatches: bool,
) -> Totals {
    let mut t = Totals::default();
    let mut messages: Vec<String> = Vec::new();
    let only: Option<u64> = std::env::var("CLIPPER2_PARITY_ONLY").ok().and_then(|v| v.strip_prefix(&format!("{name}#")).and_then(|i| i.parse().ok()));
    let n = if expect_mismatches { n } else { n * std::env::var("CLIPPER2_PARITY_SCALE").ok().and_then(|v| v.parse::<u64>().ok()).unwrap_or(1) };
    for index in 0..n {
        if only.is_some_and(|o| o != index) {
            continue;
        }
        let mut s = case_seed(test_id, index);
        let mut case = gen(index, &mut s);
        if z {
            assign_z(&mut case, index, &mut s);
        }
        let cpp = cpp_offset(&case, z);
        let rust_out = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| rust(&case)));
        t.cases += 1;
        t.combos.insert(index % COMBOS);
        if case.delta > 0.0 {
            t.positive += 1;
        } else if case.delta < 0.0 {
            t.negative += 1;
        } else {
            t.zero += 1;
        }
        let difference = match &rust_out {
            Err(e) => {
                let what = e.downcast_ref::<String>().cloned().or_else(|| e.downcast_ref::<&str>().map(|m| m.to_string())).unwrap_or_default();
                Some(format!("the Rust side panicked: {what}"))
            }
            Ok((flat, stats)) => {
                for i in 0..stats.len() {
                    t.stats[i] += stats[i];
                }
                engine::first_difference(flat, &cpp)
            }
        };
        t.paths += cpp.lens.len() as u64;
        t.output_points += (cpp.coords.len() / 3) as u64;
        t.cb_calls += cpp.cb_calls;
        if cpp.coords.chunks(3).any(|c| c[2] != 0) {
            t.nonzero_z_out += 1;
        }
        if !cpp.lens.is_empty() {
            t.nonempty += 1;
        }
        if !cpp.parents.is_empty() {
            t.tree_cases += 1;
            t.tree_holes += cpp.holes.iter().filter(|h| **h != 0).count() as u64;
        }
        if let Some(d) = difference {
            t.mismatches += 1;
            if !expect_mismatches {
                let mut text = format!("MISMATCH {}: {d}\n{}", case.name, dump_case(&case));
                if let Ok((flat, _)) = &rust_out {
                    dump_flat(&mut text, "Rust", flat);
                }
                dump_flat(&mut text, "C++", &cpp);
                append("CLIPPER2_PARITY_LOG", &text);
                if messages.len() < 2 {
                    messages.push(text);
                }
            }
        }
    }
    let mut summary = format!(
        "{name} ({}{}): {} cases, {} mismatches; deltas: {} positive, {} negative, {} zero; {} non-empty results, {} paths, {} points; {} trees with nodes, {} holes; Z callback calls {}, results with a non-zero z {}; {} of {} combinations of join type, end type, paths or tree\n ",
        if z { "USINGZ" } else { "no Z" },
        if expect_mismatches { ", MUTATED" } else { "" },
        t.cases,
        t.mismatches,
        t.positive,
        t.negative,
        t.zero,
        t.nonempty,
        t.paths,
        t.output_points,
        t.tree_cases,
        t.tree_holes,
        t.cb_calls,
        t.nonzero_z_out,
        t.combos.len(),
        COMBOS
    );
    for (i, n) in STAT_NAMES.iter().enumerate() {
        let _ = write!(summary, " {n}={}", t.stats[i]);
    }
    summary.push('\n');
    eprint!("{summary}");
    append("CLIPPER2_PARITY_SUMMARY", &summary);
    if !expect_mismatches {
        assert!(t.mismatches == 0, "{name}: {} of {} cases differ from the C++. The first:\n{}", t.mismatches, t.cases, messages.join("\n"));
        if only.is_none() {
            assert_eq!(t.combos.len() as u64, COMBOS.min(n), "{name}: not every combination of the options was run");
        }
    }
    t
}

/// A path for the helpers of clipper.h: random, nearly collinear, with duplicates, closed
/// by its first point, at one of several magnitudes.
pub fn helper_path(index: u64, s: &mut u64) -> Path {
    let mag = [4i64, 100, 100_000, 1_000_000_000, 1 << 45][(index % 5) as usize];
    let n = match index / 5 % 6 {
        0 => between(0, 5, s),
        1 => between(3, 9, s),
        _ => between(4, 40, s),
    } as usize;
    let mut path: Path = match index / 30 % 5 {
        // anything
        0 => (0..n).map(|_| [r(mag, s), r(mag, s), 0]).collect(),
        // a noisy line
        1 => {
            let (dx, dy) = (r(mag, s) / 8, r(mag, s) / 8);
            (0..n as i64).map(|i| [i * dx + r((mag / 1000).max(1), s), i * dy + r((mag / 1000).max(1), s), 0]).collect()
        }
        // a noisy circle
        2 => (0..n)
            .map(|i| {
                let a = i as f64 / n.max(1) as f64 * std::f64::consts::TAU;
                [(mag as f64 * a.cos()).round() as i64 + r((mag / 500).max(1), s), (mag as f64 * a.sin()).round() as i64 + r((mag / 500).max(1), s), 0]
            })
            .collect(),
        // exactly collinear runs and corners on a grid
        3 => {
            let (mut x, mut y) = (r(mag, s), r(mag, s));
            let step = (mag / 16).max(1);
            let (mut dx, mut dy) = (step, 0);
            (0..n)
                .map(|_| {
                    if chance(1, 3, s) {
                        (dx, dy) = [(step, 0), (0, step), (-step, 0), (0, -step), (step, step)][below(5, s) as usize];
                    }
                    x += dx;
                    y += dy;
                    [x, y, 0]
                })
                .collect()
        }
        // duplicates
        _ => {
            let pts: Vec<Pt> = (0..4).map(|_| [r(mag, s), r(mag, s), 0]).collect();
            (0..n).map(|_| pts[below(4, s) as usize]).collect()
        }
    };
    if !path.is_empty() && chance(1, 5, s) {
        path.push(path[0]);
    }
    for (i, p) in path.iter_mut().enumerate() {
        p[2] = i as i64 + 1;
    }
    path
}

/// The tests of one build. The file that expands this has `c2` (the module of the build:
/// `slicer::clipper2` or `slicer::clipper2::clipper2_z`), `Z`, `mk` (a point of the build
/// from x, y, z), `zof` (the z of a point of the build, 0 without Z) and `install_zcb` (gives
/// the offsetter its Z callback; nothing without Z).
#[macro_export]
macro_rules! offset_parity_tests {
    () => {
        use common::{Flat, OffsetCase, Stats};
        use std::cell::{Cell, RefCell};
        use std::rc::Rc;

        fn to_path(p: &common::Path) -> c2::Path64 {
            p.iter().map(mk).collect()
        }

        fn to_paths(ps: &common::Paths) -> c2::Paths64 {
            ps.iter().map(to_path).collect()
        }

        fn from_path(p: &c2::Path64) -> common::Path {
            p.iter().map(|q| [q.x, q.y, zof(q)]).collect()
        }

        fn walk(node: &c2::PolyPath64, parent: i32, flat: &mut Flat) {
            for i in 0..node.count() {
                let child = node.child(i);
                let me = flat.parents.len() as i32;
                flat.parents.push(parent);
                flat.holes.push(child.is_hole() as i32);
                flat.add(&from_path(child.polygon()));
                walk(child, me, flat);
            }
        }

        const JOINS: [c2::JoinType; 4] = [c2::JoinType::Square, c2::JoinType::Bevel, c2::JoinType::Round, c2::JoinType::Miter];
        const ENDS: [c2::EndType; 5] = [c2::EndType::Polygon, c2::EndType::Joined, c2::EndType::Butt, c2::EndType::Square, c2::EndType::Round];

        thread_local! {
            /// A mutation of the Rust side the mutation checks switch on: 0 none.
            static MUTATION: Cell<u32> = const { Cell::new(0) };
        }

        /// The delta callbacks of `cz2_ref_*_offset` (clipper2_ref_shim.cpp).
        fn make_delta_callback(kind: i32, delta: f64) -> Option<c2::DeltaCallback64> {
            match kind & 15 {
                1 => Some(c2::DeltaCallback64::new(move |_path: &c2::Path64, _norms: &c2::PathD, curr: usize, prev: usize| {
                    let f = 0.5 + 0.25 * ((curr + 2 * prev) % 5) as f64;
                    delta * f
                })),
                2 => Some(c2::DeltaCallback64::new(move |_path: &c2::Path64, norms: &c2::PathD, curr: usize, _prev: usize| {
                    let t = 0.25 * norms[curr].x;
                    let f = 1.0 + t;
                    delta * f
                })),
                3 => Some(c2::DeltaCallback64::new(move |_path: &c2::Path64, _norms: &c2::PathD, curr: usize, _prev: usize| {
                    if curr % 3 == 0 {
                        0.0
                    } else {
                        delta
                    }
                })),
                4 => Some(c2::DeltaCallback64::new(move |path: &c2::Path64, norms: &c2::PathD, curr: usize, prev: usize| {
                    let v = (path[curr].x ^ path[prev].y) & 7;
                    let f = 0.75 + 0.125 * v as f64;
                    let g = if norms.len() == path.len() { 1.0 } else { 2.0 };
                    let d = delta * f;
                    d * g
                })),
                _ => None,
            }
        }

        /// The case through the Rust port, as `cz2_ref_*_offset` (clipper2_ref_shim.cpp) runs
        /// it through the library.
        fn rust_offset(case: &OffsetCase) -> (Flat, Stats) {
            let mutation = MUTATION.with(|m| m.get());
            let mut flat = Flat { cb_hash: common::HASH_START, succeeded: true, ..Flat::default() };
            let mut stats: Stats = [0; 29];
            if case.mode & 4 != 0 {
                let g = &case.groups[0];
                let sol = c2::inflate_paths(&to_paths(&g.paths), case.delta, JOINS[g.join as usize], ENDS[g.end as usize], case.miter_limit, case.arc_tolerance);
                for p in &sol {
                    flat.add(&from_path(p));
                }
                flat.num_closed = sol.len() as i32;
                stats[28] = 1;
                return (flat, stats);
            }
            let mut co = c2::ClipperOffset::new(case.miter_limit, case.arc_tolerance, case.preserve_collinear, case.reverse_solution);
            let state = Rc::new(RefCell::new((0u64, common::HASH_START)));
            install_zcb(&mut co, case.z_callback, state.clone());
            for g in &case.groups {
                let (mut jt, et) = (JOINS[g.join as usize], ENDS[g.end as usize]);
                // mutation 1: a Miter join where a Square one is asked for
                if mutation == 1 && jt == c2::JoinType::Square {
                    jt = c2::JoinType::Miter;
                }
                if case.mode & 1 != 0 && g.paths.len() == 1 {
                    co.add_path(&to_path(&g.paths[0]), jt, et);
                } else {
                    co.add_paths(&to_paths(&g.paths), jt, et);
                }
            }
            if case.mode & 2 != 0 {
                let mut dropped = c2::Paths64::new();
                co.execute(case.delta2, &mut dropped);
            }
            let cb = make_delta_callback(case.delta_cb, case.delta);
            let run = |co: &mut c2::ClipperOffset, d: f64, flat: &mut Flat| -> c2::Paths64 {
                flat.coords.clear();
                flat.lens.clear();
                flat.parents.clear();
                flat.holes.clear();
                // mutation 2: the delta one unit in the last place off
                let d = if mutation == 2 { f64::from_bits(d.to_bits() + 1) } else { d };
                if case.use_tree {
                    let mut tree = c2::PolyTree64::new();
                    if let Some(cb) = &cb {
                        co.set_delta_callback(cb.clone());
                    }
                    co.execute_tree(d, &mut tree);
                    walk(&tree, -1, flat);
                    flat.num_closed = flat.parents.len() as i32;
                    return c2::poly_tree_to_paths64(&tree);
                }
                let mut sol = c2::Paths64::new();
                match &cb {
                    Some(cb) if case.delta_cb & 16 != 0 => co.execute_delta_callback(cb.clone(), &mut sol),
                    _ => {
                        if let Some(cb) = &cb {
                            co.set_delta_callback(cb.clone());
                        }
                        co.execute(d, &mut sol);
                    }
                }
                // mutation 3: the solution from its second path on
                if mutation == 3 && sol.len() > 1 {
                    sol.rotate_left(1);
                }
                for p in &sol {
                    flat.add(&from_path(p));
                }
                flat.num_closed = sol.len() as i32;
                sol
            };
            let sol = run(&mut co, case.delta, &mut flat);
            if case.mode & 8 != 0 {
                co.clear();
                let g = &case.groups[0];
                co.add_paths(&sol, JOINS[g.join as usize], ENDS[g.end as usize]);
                run(&mut co, case.delta2, &mut flat);
            }
            flat.succeeded = co.error_code() == 0;
            let st = state.borrow();
            flat.cb_calls = st.0;
            flat.cb_hash = st.1;
            let o = co.stats;
            stats = [
                o.do_bevel,
                o.do_bevel_end,
                o.do_square,
                o.do_square_end,
                o.miter_exceeded,
                o.do_miter,
                o.almost_straight,
                o.do_round,
                o.do_round_end,
                o.round_steps,
                o.concave,
                o.same_point,
                o.zero_delta,
                o.square_parallel,
                o.avg_vector_zero,
                o.point_circle,
                o.point_square,
                o.point_skipped,
                o.offset_polygon,
                o.offset_open_joined,
                o.offset_open_path,
                o.joined_two_points,
                o.groups_reversed,
                o.no_lowest_path,
                o.insignificant,
                o.zcb,
                o.zcb_user,
                o.delta_callbacks,
                0,
            ];
            (flat, stats)
        }

        fn run(name: &str, id: u64, n: u64, gen: fn(u64, &mut u64) -> OffsetCase) -> common::Totals {
            common::run_category(name, id, n, gen, Z, &rust_offset, false)
        }

        #[test]
        fn reference_is_clipper2_of_the_pin() {
            let (from_pin, version) = common::engine::cpp_source(Z);
            eprintln!("reference: Clipper2 {version}, {}", if from_pin == 1 { "built from references/BambuStudio" } else { "built from the vendored copy" });
            assert_eq!(version, "1.5.2");
        }

        #[test]
        fn closed_polygons() {
            let t = run("closed_polygons", 201, 24_000, common::gen_closed);
            assert!(t.nonempty > 12_000);
            for name in ["do_bevel", "do_square", "do_miter", "do_round", "concave", "almost_straight", "miter_exceeded", "round_steps", "groups_reversed", "insignificant", "inflate_paths"] {
                assert!(t.stat(name) > 100, "{name} was reached {} times", t.stat(name));
            }
            assert!(t.positive > 8_000 && t.negative > 8_000 && t.zero > 500);
        }

        #[test]
        fn holes_nested() {
            let t = run("holes_nested", 202, 12_000, common::gen_holes);
            assert!(t.tree_holes > 1_000, "no holes came out: {}", t.tree_holes);
            assert!(t.stat("concave") > 10_000);
        }

        #[test]
        fn open_paths() {
            let t = run("open_paths", 203, 20_000, common::gen_open);
            // Three branches no input reaches, by construction: `path[j] == path[k]` in
            // OffsetPoint (StripDuplicates has taken the equal neighbours out, and a single
            // point never gets there), the zero vector of GetAvgUnitVector (the normals of a
            // join that is squared are at least 2.5 degrees apart) and parallel lines in
            // DoSquare's GetSegmentIntersectPt (the two lines are perpendicular by
            // construction). The counters show it.
            for name in ["same_point", "avg_vector_zero", "square_parallel"] {
                assert_eq!(t.stat(name), 0, "{name} is reached after all: give it a test");
            }
            for name in ["do_bevel_end", "do_square_end", "do_round_end", "offset_open_joined", "offset_open_path", "joined_two_points"] {
                assert!(t.stat(name) > 100, "{name} was reached {} times", t.stat(name));
            }
        }

        #[test]
        fn degenerate() {
            let t = run("degenerate", 204, 16_000, common::gen_degenerate);
            for name in ["point_circle", "point_square", "point_skipped", "joined_two_points", "almost_straight"] {
                assert!(t.stat(name) > 50, "{name} was reached {} times", t.stat(name));
            }
            // a Polygon group without a single point
            assert!(t.stat("no_lowest_path") > 10, "no_lowest_path was reached {} times", t.stat("no_lowest_path"));
        }

        #[test]
        fn huge_coordinates() {
            let t = run("huge_coordinates", 205, 8_000, common::gen_huge);
            assert!(t.nonempty > 3_000);
        }

        #[test]
        fn tiny() {
            let t = run("tiny", 206, 10_000, common::gen_tiny);
            assert!(t.nonempty > 3_000);
            assert!(t.stat("do_round") > 1_000);
        }

        #[test]
        fn coincident_and_collinear() {
            let t = run("coincident_and_collinear", 207, 12_000, common::gen_coincident);
            assert!(t.stat("almost_straight") > 1_000);
        }

        #[test]
        fn self_intersecting() {
            let t = run("self_intersecting", 208, 6_000, common::gen_self_intersecting);
            assert!(t.nonempty > 3_000);
        }

        #[test]
        fn multi_group() {
            let t = run("multi_group", 209, 10_000, common::gen_multi_group);
            assert!(t.stat("groups_reversed") > 1_000);
        }

        #[test]
        fn delta_callback() {
            let t = run("delta_callback", 210, 10_000, common::gen_delta_callback);
            assert!(t.stat("delta_callbacks") > 100_000);
            assert!(t.stat("zero_delta") > 1_000, "no vertex had a delta of zero");
            assert!(t.stat("do_round") > 1_000);
        }

        #[test]
        fn two_stage() {
            let t = run("two_stage", 211, 6_000, common::gen_two_stage);
            assert!(t.nonempty > 1_000);
        }

        /// The Z arms: the z of the input goes to every point made from it, `ZCB` takes its
        /// arms and the user's callback is reached.
        #[test]
        fn z_arms() {
            if !Z {
                return;
            }
            let t = run("closed_polygons", 201, 4_000, common::gen_closed);
            assert!(t.stat("zcb") > 10_000, "ZCB ran {} times", t.stat("zcb"));
            assert!(t.stat("zcb_user") > 1_000, "the user's Z callback ran {} times", t.stat("zcb_user"));
            assert!(t.cb_calls > 1_000 && t.nonzero_z_out > 1_000);
        }

        /// Sensitivity: each change of the Rust side below makes cases differ.
        #[test]
        fn mutation_checks() {
            let mut found = Vec::new();
            for (mutation, what) in [(1u32, "Miter for Square"), (2, "delta one ulp off"), (3, "solution rotated by one path")] {
                MUTATION.with(|m| m.set(mutation));
                let t = common::run_category("closed_polygons", 201, 2_000, common::gen_closed, Z, &rust_offset, true);
                MUTATION.with(|m| m.set(0));
                eprintln!("mutation {mutation} ({what}): {} of {} cases differ", t.mismatches, t.cases);
                found.push((what, t.mismatches));
            }
            // sin and cos from two calls in the place of the one paired call
            let mut ulp = 0u64;
            let n = 200_000u64;
            for i in 0..n {
                let x = 2.0 * c2::PI / (3.0 + i as f64 * 0.37);
                let c = common::cpp_sin_cos(x, Z);
                let (s_r, c_r) = slicer::clipper::sin_cos(x);
                assert_eq!([s_r.to_bits(), c_r.to_bits()], [c[0], c[1]], "sin_cos({x}) is not what the reference's paired call gives");
                if x.sin().to_bits() != c[0] || x.cos().to_bits() != c[1] {
                    ulp += 1;
                }
            }
            eprintln!("sin/cos: {ulp} of {n} step angles where two separate calls give other bits than the paired call");
            for (what, n) in &found {
                assert!(*n > 0, "the mutation '{what}' changed nothing");
            }
        }

        fn flat_of(paths: &c2::Paths64) -> Flat {
            let mut flat = Flat { succeeded: true, ..Flat::default() };
            for p in paths {
                flat.add(&from_path(p));
            }
            flat.num_closed = paths.len() as i32;
            flat
        }

        /// SimplifyPath, RamerDouglasPeucker, TrimCollinear, Ellipse, TranslatePath, Length,
        /// Distance, NearCollinear of clipper.h.
        #[test]
        fn path_helpers() {
            const N: u64 = 60_000;
            let mut differ = 0u64;
            let mut first = String::new();
            let mut counts = [0u64; 6];
            let mut removed = [0u64; 3];
            let check = |what: String, rust: Flat, cpp: Flat, differ: &mut u64, first: &mut String| {
                if rust.coords != cpp.coords || rust.lens != cpp.lens {
                    *differ += 1;
                    if first.is_empty() {
                        *first = format!("{what}:\n  Rust {:?}\n  C++  {:?}", rust.coords, cpp.coords);
                    }
                }
            };
            for index in 0..N {
                let mut s = common::case_seed(301, index);
                let path = common::helper_path(index, &mut s);
                let p = to_path(&path);
                let scale = path.iter().map(|q| q[0].abs().max(q[1].abs())).max().unwrap_or(1).max(1) as f64;
                let eps = [0.0, 1.0, scale * 1e-4, scale * 1e-3, scale * 0.01, scale * 0.1, scale][common::below(7, &mut s) as usize];
                let closed = common::chance(1, 2, &mut s);
                // 0 SimplifyPath
                let rust = c2::simplify_path(&p, eps, closed);
                removed[0] += (p.len() - rust.len()) as u64;
                check(format!("SimplifyPath #{index} eps {eps:?} closed {closed} {path:?}"), flat_of(&vec![rust]), common::cpp_path_op(0, &path, eps, 0.0, closed as i64, Z).0, &mut differ, &mut first);
                counts[0] += 1;
                // 1 RamerDouglasPeucker
                let rust = c2::ramer_douglas_peucker(&p, eps);
                removed[1] += (p.len() - rust.len()) as u64;
                check(format!("RamerDouglasPeucker #{index} eps {eps:?} {path:?}"), flat_of(&vec![rust]), common::cpp_path_op(1, &path, eps, 0.0, 0, Z).0, &mut differ, &mut first);
                counts[1] += 1;
                // 2 TrimCollinear
                let rust = c2::trim_collinear(&p, closed);
                removed[2] += (p.len() - rust.len()) as u64;
                check(format!("TrimCollinear #{index} open {closed} {path:?}"), flat_of(&vec![rust]), common::cpp_path_op(2, &path, 0.0, 0.0, closed as i64, Z).0, &mut differ, &mut first);
                counts[2] += 1;
                if !path.is_empty() {
                    // 3 Ellipse
                    let rx = [0.0, -1.0, 0.4, 1.0, 2.5, 10.0, scale * 0.37, 1e6][common::below(8, &mut s) as usize];
                    let ry = [0.0, rx, rx * 0.5, 3.0, -2.0][common::below(5, &mut s) as usize];
                    let steps = [0usize, 0, 1, 2, 3, 4, 7, 16, 100][common::below(9, &mut s) as usize];
                    let rust = c2::ellipse(&p[0], rx, ry, steps);
                    check(format!("Ellipse #{index} center {:?} rx {rx:?} ry {ry:?} steps {steps}", path[0]), flat_of(&vec![rust]), common::cpp_path_op(3, &path, rx, ry, steps as i64, Z).0, &mut differ, &mut first);
                    counts[3] += 1;
                    // 4 TranslatePath
                    let d = common::r(1_000_000, &mut s);
                    let rust = c2::translate_path(&p, d, -d);
                    check(format!("TranslatePath #{index} {d} {path:?}"), flat_of(&vec![rust]), common::cpp_path_op(4, &path, 0.0, 0.0, d, Z).0, &mut differ, &mut first);
                    counts[4] += 1;
                    // 5 Length, Distance, NearCollinear
                    let sin2 = [1e-6, 1e-3, 0.1][common::below(3, &mut s) as usize];
                    let rust = [
                        c2::length(&p, false),
                        c2::length(&p, true),
                        if p.len() >= 2 { c2::distance(p[0], p[1]) } else { 0.0 },
                        if p.len() >= 3 { c2::near_collinear(&p[0], &p[1], &p[2], sin2) as u8 as f64 } else { 0.0 },
                    ]
                    .map(f64::to_bits);
                    let cpp = common::cpp_path_op(5, &path, sin2, 0.0, 0, Z).1;
                    if rust != cpp {
                        differ += 1;
                        if first.is_empty() {
                            first = format!("Length/Distance/NearCollinear #{index} {path:?}: Rust {rust:?}, C++ {cpp:?}");
                        }
                    }
                    counts[5] += 1;
                }
            }
            let total: u64 = counts.iter().sum();
            eprintln!(
                "path_helpers ({}): {total} calls: SimplifyPath {} (removed {} points), RamerDouglasPeucker {} (removed {}), TrimCollinear {} (removed {}), Ellipse {}, TranslatePath {}, Length/Distance/NearCollinear {}; {differ} differ",
                if Z { "USINGZ" } else { "no Z" },
                counts[0],
                removed[0],
                counts[1],
                removed[1],
                counts[2],
                removed[2],
                counts[3],
                counts[4],
                counts[5]
            );
            assert!(differ == 0, "{differ} of {total} helper calls differ. The first: {first}");
            assert!(removed.iter().all(|r| *r > 10_000), "a helper removed next to nothing: {removed:?}");
        }

        /// BooleanOp to paths and to a tree, PolyTreeToPaths64,
        /// CheckPolytreeFullyContainsChildren, Union of subjects alone, Intersect, Union,
        /// Difference, Xor of clipper.h.
        #[test]
        fn boolean_helpers() {
            const N: u64 = 12_000;
            let mut differ = 0u64;
            let mut first = String::new();
            let mut nonempty = 0u64;
            let mut not_contained = 0u64;
            let cts = [c2::ClipType::NoClip, c2::ClipType::Intersection, c2::ClipType::Union, c2::ClipType::Difference, c2::ClipType::Xor];
            let frs = [c2::FillRule::EvenOdd, c2::FillRule::NonZero, c2::FillRule::Positive, c2::FillRule::Negative];
            for index in 0..N {
                let mut s = common::case_seed(302, index);
                let e = match index % 3 {
                    0 => common::engine::gen_random(index, &mut s),
                    1 => common::engine::gen_nested(index, &mut s),
                    _ => common::engine::gen_touching(index, &mut s),
                };
                let (ct, fr, op) = (1 + (index % 4) as i32, (index / 4 % 4) as i32, (index / 16 % 5) as i32);
                let (subj, clip) = (to_paths(&e.subj), to_paths(&e.clip));
                let (ctr, frr) = (cts[ct as usize], frs[fr as usize]);
                let mut rust = Flat { succeeded: true, ..Flat::default() };
                match op {
                    0 => rust = flat_of(&c2::boolean_op(ctr, frr, &subj, &clip)),
                    1 => {
                        let mut tree = c2::PolyTree64::new();
                        c2::boolean_op_tree(ctr, frr, &subj, &clip, &mut tree);
                        walk(&tree, -1, &mut rust);
                        rust.num_closed = rust.parents.len() as i32;
                    }
                    2 => {
                        let mut tree = c2::PolyTree64::new();
                        c2::boolean_op_tree(ctr, frr, &subj, &clip, &mut tree);
                        rust = flat_of(&c2::poly_tree_to_paths64(&tree));
                        rust.succeeded = c2::check_polytree_fully_contains_children(&tree);
                    }
                    3 => rust = flat_of(&c2::union_subjects(&subj, frr)),
                    _ => {
                        rust = flat_of(&match ct {
                            1 => c2::intersect(&subj, &clip, frr),
                            2 => c2::union(&subj, &clip, frr),
                            3 => c2::difference(&subj, &clip, frr),
                            _ => c2::xor(&subj, &clip, frr),
                        })
                    }
                }
                let cpp = common::cpp_boolean_op(op, ct, fr, &e.subj, &e.clip, Z);
                if !cpp.lens.is_empty() {
                    nonempty += 1;
                }
                if op == 2 && !cpp.succeeded {
                    not_contained += 1;
                }
                if let Some(d) = common::engine::first_difference(&rust, &cpp) {
                    differ += 1;
                    if first.is_empty() {
                        first = format!("{} op {op} clip type {ct} fill rule {fr}: {d}", e.name);
                    }
                }
            }
            eprintln!("boolean_helpers ({}): {N} calls, {nonempty} non-empty results, {not_contained} trees not fully containing their children; {differ} differ", if Z { "USINGZ" } else { "no Z" });
            assert!(differ == 0, "{differ} of {N} calls differ. The first: {first}");
            assert!(nonempty > N / 2);
        }
    };
}
