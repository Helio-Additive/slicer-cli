//! `ClipperOffset` of `src/clipper.rs` (clipper.cpp:3315-3751) against the library itself,
//! compiled in `clipper-z-sys`: `Execute(Paths&, delta)` gives the same paths in the same
//! order, each with the same points from the same start vertex, and `Execute(PolyTree&,
//! delta)` the same tree (nodes in pre-order with contour, parent, `IsHole()`, `IsOpen()`,
//! number of children, and `Total()`).
//!
//! Inputs: convex, concave (stars), rectangles, self-intersecting, outers with holes (the
//! opposite orientation), tiny polygons, slivers, collinear and coincident points, rectilinear
//! walks, and open paths down to a single point; every `JoinType` and every `EndType`, the same
//! for all paths of a case or mixed; deltas of both signs from 1e-25 to twice the size of the
//! shape (so that it collapses); miter limits from 0.5 (the code clamps below 2) to 8; arc
//! tolerances absolute, relative to the delta, zero, negative and huge; `ShortestEdgeLength`
//! off and on; coordinates from tens to 1e8.
//!
//! Every case is made from its own seed, so a failure names the case and prints its input.
//! `CLIPPER_OFFSET_PARITY_LOG=<file>` writes every failing input to that file.

use slicer::clipper::{ClipperOffset, EndType, IntPoint, JoinType, Path, Paths, PolyTree, POLY_TREE_ROOT};
use std::fmt::Write as _;

const JOIN_TYPES: [JoinType; 3] = [JoinType::jtSquare, JoinType::jtRound, JoinType::jtMiter];
const END_TYPES: [EndType; 5] =
    [EndType::etClosedPolygon, EndType::etClosedLine, EndType::etOpenButt, EndType::etOpenSquare, EndType::etOpenRound];
const LIMIT: i64 = 400_000_000;
/// Nodes the library may write to the node buffer.
const NODE_CAP: usize = 8192;

fn xorshift(s: &mut u64) -> u64 {
    *s ^= *s << 13;
    *s ^= *s >> 7;
    *s ^= *s << 17;
    *s
}

/// The seed of case `index` of a test: splitmix64 of the two, never zero.
fn case_seed(test: u64, index: u64) -> u64 {
    let mut z = test.wrapping_mul(0x9E37_79B9_7F4A_7C15).wrapping_add(index).wrapping_add(0x9E37_79B9_7F4A_7C15);
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    (z ^ (z >> 31)) | 1
}

/// A value in `-m..=m`.
fn r(m: i64, s: &mut u64) -> i64 {
    (xorshift(s) % (2 * m as u64 + 1)) as i64 - m
}

/// A value in `0..n`.
fn below(n: u64, s: &mut u64) -> u64 {
    xorshift(s) % n
}

fn pt(x: i64, y: i64, grid: i64) -> IntPoint {
    let snap = |v: i64| if grid > 1 { v.div_euclid(grid) * grid } else { v };
    IntPoint::new(snap(x).clamp(-LIMIT, LIMIT) as i32, snap(y).clamp(-LIMIT, LIMIT) as i32)
}


#[derive(Clone)]
struct Case {
    name: String,
    paths: Paths,
    /// Per path: index into `JOIN_TYPES` and into `END_TYPES`.
    join: Vec<usize>,
    end: Vec<usize>,
    miter_limit: f64,
    arc_tolerance: f64,
    shortest_edge_length: f64,
    delta: f64,
}

fn show_paths(ps: &Paths) -> String {
    let mut out = String::from("vec![");
    for p in ps {
        out.push_str("vec![");
        for q in p {
            let _ = write!(out, "({},{}),", q.x, q.y);
        }
        out.push_str("],");
    }
    out.push(']');
    out
}

impl Case {
    fn describe(&self) -> String {
        format!(
            "{}: delta={:?} miter_limit={:?} arc_tolerance={:?} shortest_edge_length={:?}\n  join: {:?}\n  end: {:?}\n  paths: {}",
            self.name,
            self.delta,
            self.miter_limit,
            self.arc_tolerance,
            self.shortest_edge_length,
            self.join.iter().map(|&j| JOIN_TYPES[j]).collect::<Vec<_>>(),
            self.end.iter().map(|&e| END_TYPES[e]).collect::<Vec<_>>(),
            show_paths(&self.paths),
        )
    }

    fn offsetter(&self) -> ClipperOffset {
        let mut co = ClipperOffset::new(self.miter_limit, self.arc_tolerance, self.shortest_edge_length);
        for (i, p) in self.paths.iter().enumerate() {
            co.add_path(p, JOIN_TYPES[self.join[i]], END_TYPES[self.end[i]]);
        }
        co
    }
}

/// A tree as the shim lists it: contours in pre-order and, per node, the parent's index in
/// the listing (-1 for the tree), IsHole, IsOpen, ChildCount.
#[derive(Debug, PartialEq, Eq)]
struct Listing {
    contours: Paths,
    info: Vec<[i32; 4]>,
    total: i32,
}

fn list_node(tree: &PolyTree, node: usize, parent: i32, contours: &mut Paths, info: &mut Vec<[i32; 4]>) {
    for &child in &tree.node(node).childs {
        let me = contours.len() as i32;
        let n = tree.node(child);
        contours.push(n.contour.clone());
        info.push([parent, tree.is_hole(child) as i32, n.is_open() as i32, n.child_count()]);
        list_node(tree, child, me, contours, info);
    }
}

fn rust_paths(c: &Case) -> Paths {
    let mut solution = Paths::new();
    c.offsetter().execute(&mut solution, c.delta);
    solution
}

fn rust_tree(c: &Case) -> Listing {
    let mut tree = PolyTree::new();
    c.offsetter().execute_poly_tree(&mut tree, c.delta);
    let mut contours = Paths::new();
    let mut info = Vec::new();
    list_node(&tree, POLY_TREE_ROOT, -1, &mut contours, &mut info);
    let total = if contours.is_empty() && tree.child_count() == 0 { 0 } else { tree.total() };
    Listing { contours, info, total }
}

unsafe fn take_paths(out: clipper_z_sys::CzZPaths) -> Paths {
    let mut paths = Paths::new();
    let mut k = 0usize;
    for i in 0..out.num_paths as usize {
        let len = *out.path_lens.add(i) as usize;
        let mut p = Path::with_capacity(len);
        for _ in 0..len {
            p.push(IntPoint::new(*out.coords.add(3 * k), *out.coords.add(3 * k + 1)));
            k += 1;
        }
        paths.push(p);
    }
    assert_eq!(k, out.total_points as usize);
    clipper_z_sys::cz_free_zpaths(out);
    paths
}

fn run_library(c: &Case, use_polytree: bool) -> Listing {
    let xy: Vec<i32> = c.paths.iter().flatten().flat_map(|p| [p.x, p.y]).collect();
    let lens: Vec<i32> = c.paths.iter().map(|p| p.len() as i32).collect();
    let join: Vec<i32> = c.join.iter().map(|&j| j as i32).collect();
    let end: Vec<i32> = c.end.iter().map(|&e| e as i32).collect();
    let mut node_info = vec![0i32; 4 * NODE_CAP];
    let (mut num_nodes, mut total) = (0i32, 0i32);
    let contours = unsafe {
        take_paths(clipper_z_sys::cz_ref_offset(
            xy.as_ptr(),
            lens.as_ptr(),
            join.as_ptr(),
            end.as_ptr(),
            lens.len() as i32,
            c.miter_limit,
            c.arc_tolerance,
            c.shortest_edge_length,
            c.delta,
            use_polytree as i32,
            node_info.as_mut_ptr(),
            NODE_CAP as i32,
            &mut num_nodes,
            &mut total,
        ))
    };
    let mut info = Vec::new();
    if use_polytree {
        assert!(num_nodes as usize <= NODE_CAP, "the node buffer is too small: {num_nodes}");
        assert_eq!(num_nodes as usize, contours.len());
        info = node_info[..4 * num_nodes as usize].chunks(4).map(|q| [q[0], q[1], q[2], q[3]]).collect();
    }
    Listing { contours, info, total }
}

/// Where two path lists first differ, for the failure message.
fn first_difference(want: &Paths, got: &Paths) -> String {
    if want.len() != got.len() {
        return format!("{} paths in the library, {} in the port", want.len(), got.len());
    }
    for (i, (w, g)) in want.iter().zip(got).enumerate() {
        if w.len() != g.len() {
            return format!("path {i}: {} points in the library, {} in the port", w.len(), g.len());
        }
        for (j, (a, b)) in w.iter().zip(g).enumerate() {
            if a != b {
                return format!("path {i} point {j} of {}: library ({},{}) port ({},{})", w.len(), a.x, a.y, b.x, b.y);
            }
        }
    }
    String::from("equal")
}

/// What became of the comparisons of one test.
#[derive(Default)]
struct Tally {
    cases: usize,
    nonempty: usize,
    paths: usize,
    points: usize,
    trees: usize,
    tree_holes: usize,
    failures: Vec<String>,
    /// Failures by the join type and by the end type of the first path, and by the sign of delta.
    by_join: [usize; 3],
    by_end: [usize; 5],
    by_sign: [usize; 2],
}

impl Tally {
    fn check(&mut self, c: &Case) {
        self.cases += 1;
        let want = run_library(c, false).contours;
        if !want.is_empty() {
            self.nonempty += 1;
            self.paths += want.len();
            self.points += want.iter().map(|p| p.len()).sum::<usize>();
        }
        let got = std::panic::catch_unwind(|| rust_paths(c));
        let mut what = match got {
            Ok(got) if got == want => String::new(),
            Ok(got) => format!("  Execute(Paths): {}\n  library: {:?}\n  port:    {:?}", first_difference(&want, &got), want, got),
            Err(_) => format!("  Execute(Paths)\n  library: {want:?}\n  port:    panicked"),
        };
        // The PolyTree overload on every third case.
        if what.is_empty() && self.cases % 3 == 0 {
            let want = run_library(c, true);
            self.trees += 1;
            self.tree_holes += want.info.iter().map(|q| q[1] as usize).sum::<usize>();
            match std::panic::catch_unwind(|| rust_tree(c)) {
                Ok(got) if got == want => {}
                Ok(got) => what = format!("  Execute(PolyTree)\n  library: {want:?}\n  port:    {got:?}"),
                Err(_) => what = format!("  Execute(PolyTree)\n  library: {want:?}\n  port:    panicked"),
            }
        }
        if what.is_empty() {
            return;
        }
        if let (Some(&j), Some(&e)) = (c.join.first(), c.end.first()) {
            self.by_join[j] += 1;
            self.by_end[e] += 1;
        }
        self.by_sign[(c.delta > 0.0) as usize] += 1;
        self.failures.push(format!("{}\n{}", c.describe(), what));
    }

    fn finish(self, test: &str) {
        if let Some(path) = std::env::var_os("CLIPPER_OFFSET_PARITY_LOG") {
            use std::io::Write as _;
            let mut f = std::fs::OpenOptions::new().create(true).append(true).open(path).expect("the log file opens");
            writeln!(
                f,
                "{test}: {} cases, {} with a non-empty solution ({} paths, {} points), {} trees ({} hole nodes), {} differ",
                self.cases,
                self.nonempty,
                self.paths,
                self.points,
                self.trees,
                self.tree_holes,
                self.failures.len()
            )
            .unwrap();
            for failure in &self.failures {
                writeln!(f, "{failure}\n").unwrap();
            }
        }
        assert!(
            self.failures.is_empty(),
            "{test}: {} of {} cases differ from the library; by join type of the first path (square, round, miter) {:?}; \
             by its end type (closed polygon, closed line, open butt, open square, open round) {:?}; by delta (<= 0, > 0) {:?}; the first:\n{}",
            self.failures.len(),
            self.cases,
            self.by_join,
            self.by_end,
            self.by_sign,
            self.failures[0]
        );
        assert!(self.nonempty * 4 > self.cases, "{test}: only {} of {} cases have a solution", self.nonempty, self.cases);
    }
}

fn circle(cx: i64, cy: i64, radius: f64, n: u64, phase: f64) -> Path {
    (0..n)
        .map(|i| {
            let a = phase + i as f64 * std::f64::consts::TAU / n as f64;
            pt(cx + (radius * a.cos()) as i64, cy + (radius * a.sin()) as i64, 1)
        })
        .collect()
}

/// One shape of a case: one path, or an outer with its holes.
fn shape(s: &mut u64, scale: i64, out: &mut Paths) {
    let (cx, cy) = (r(scale, s), r(scale, s));
    let phase = below(628, s) as f64 / 100.0;
    let radius = (1 + below(scale as u64, s)) as f64;
    let mut poly = Path::new();
    match below(13, s) {
        // Convex.
        0 => poly = circle(cx, cy, radius, 3 + below(10, s), phase),
        // A star: concave, sharp, and with a low inner radius nearly self-touching.
        1 | 2 => {
            let inner = radius * (below(95, s) as f64 / 100.0);
            let n = 2 * (3 + below(6, s));
            for i in 0..n {
                let a = phase + i as f64 * std::f64::consts::TAU / n as f64;
                let rad = if i % 2 == 0 { radius } else { inner };
                poly.push(pt(cx + (rad * a.cos()) as i64, cy + (rad * a.sin()) as i64, 1));
            }
        }
        // Axis-aligned rectangle.
        3 => {
            let (w, h) = (1 + below(scale as u64, s) as i64, 1 + below(scale as u64, s) as i64);
            poly.extend([pt(cx, cy, 1), pt(cx + w, cy, 1), pt(cx + w, cy + h, 1), pt(cx, cy + h, 1)]);
        }
        // Random points: mostly self-intersecting.
        4 => {
            for _ in 0..3 + below(7, s) {
                poly.push(pt(cx + r(scale, s), cy + r(scale, s), 1));
            }
        }
        // An outer with one to three holes, given in the opposite orientation.
        5 | 6 => {
            let n = 3 + below(9, s);
            out.push(circle(cx, cy, radius, n, phase));
            let holes = 1 + below(3, s);
            for h in 0..holes {
                let hr = radius * (0.1 + below(60, s) as f64 / 100.0) / holes as f64;
                let off = if holes == 1 { 0.0 } else { radius * 0.5 };
                let a = h as f64 * std::f64::consts::TAU / holes as f64;
                let mut hole = circle(cx + (off * a.cos()) as i64, cy + (off * a.sin()) as i64, hr, 3 + below(7, s), phase);
                hole.reverse();
                if hole.len() >= 3 {
                    out.push(hole);
                }
            }
            if below(2, s) == 0 {
                // The whole shape the other way round: FixOrientations turns it back.
                let k = out.len();
                for p in out[k - holes as usize - 1..].iter_mut() {
                    p.reverse();
                }
            }
            return;
        }
        // Tiny: a few units across, wherever the centre is.
        7 => {
            for _ in 0..3 + below(4, s) {
                poly.push(pt(cx + r(3, s), cy + r(3, s), 1));
            }
        }
        // A sliver: long and one or two units wide.
        8 => {
            let (dx, dy) = (r(scale, s), r(scale, s));
            poly.extend([pt(cx, cy, 1), pt(cx + dx, cy + dy, 1), pt(cx + dx + r(2, s), cy + dy + r(2, s), 1)]);
            if below(2, s) == 0 {
                poly.push(pt(cx + r(2, s), cy + r(2, s), 1));
            }
        }
        // Collinear points: a rectangle with points on its edges, exactly on them.
        9 => {
            let (w, h) = (4 * (1 + below(scale as u64 / 4 + 1, s) as i64), 4 * (1 + below(scale as u64 / 4 + 1, s) as i64));
            poly.extend([
                pt(cx, cy, 1),
                pt(cx + w / 4, cy, 1),
                pt(cx + w / 2, cy, 1),
                pt(cx + w, cy, 1),
                pt(cx + w, cy + h / 2, 1),
                pt(cx + w, cy + h, 1),
                pt(cx + w / 4, cy + h, 1),
                pt(cx, cy + h, 1),
                pt(cx, cy + h / 4, 1),
            ]);
        }
        // Rectilinear walk: right angles, spikes, overlaps.
        10 => {
            let (mut x, mut y) = (cx, cy);
            let mut horizontal = below(2, s) == 0;
            for _ in 0..4 + below(8, s) {
                poly.push(pt(x, y, 1));
                if horizontal {
                    x += r(scale / 2, s);
                } else {
                    y += r(scale / 2, s);
                }
                if below(5, s) != 0 {
                    horizontal = !horizontal;
                }
            }
        }
        // Nearly collinear points along a line.
        11 => {
            let (dx, dy) = (r(scale / 4 + 1, s), r(scale / 4 + 1, s));
            for i in 0..3 + below(4, s) as i64 {
                poly.push(pt(cx + i * dx + r(1, s), cy + i * dy + r(1, s), 1));
            }
        }
        // One or two points.
        _ => {
            poly.push(pt(cx, cy, 1));
            if below(2, s) == 0 {
                poly.push(pt(cx + r(scale, s), cy + r(scale, s), 1));
            }
        }
    }
    // Coincident points: a vertex twice, the first one again at the end, a vertex in the
    // middle of an edge; another start vertex; the other direction.
    if below(4, s) == 0 && !poly.is_empty() {
        let i = below(poly.len() as u64, s) as usize;
        poly.insert(i, poly[i]);
    }
    if below(5, s) == 0 && !poly.is_empty() {
        poly.push(poly[0]);
        if below(2, s) == 0 {
            poly.push(poly[0]);
        }
    }
    if below(5, s) == 0 && poly.len() >= 2 {
        let i = below(poly.len() as u64, s) as usize;
        let (a, b) = (poly[i], poly[(i + 1) % poly.len()]);
        let mid = IntPoint::new(((a.x as i64 + b.x as i64) / 2) as i32, ((a.y as i64 + b.y as i64) / 2) as i32);
        poly.insert(i + 1, mid);
    }
    if below(3, s) == 0 && !poly.is_empty() {
        let k = below(poly.len() as u64, s) as usize;
        poly.rotate_left(k);
    }
    if below(2, s) == 0 {
        poly.reverse();
    }
    out.push(poly);
}

const MITER_LIMITS: [f64; 9] = [0.5, 1.0, 1.5, 2.0, 2.0000001, 2.5, 3.0, 5.0, 8.0];

fn delta_for(s: &mut u64, scale: i64) -> f64 {
    let magnitude = match below(16, s) {
        0 => 1e-25,
        1 => 0.0,
        2 => 0.3,
        3 => 0.5,
        4 => 1.0,
        5 => 2.5,
        6 => scale as f64 * 0.001,
        7 | 8 => scale as f64 * 0.01,
        9 | 10 => scale as f64 * 0.05,
        11 | 12 => scale as f64 * 0.2,
        13 => scale as f64 * 0.5,
        14 => scale as f64 * 1.0,
        _ => scale as f64 * 2.0,
    };
    // An offset with a fraction, now and then.
    let magnitude = if below(3, s) == 0 { magnitude * (1.0 + below(1000, s) as f64 / 3000.0) } else { magnitude };
    let magnitude = magnitude.min(50_000_000.0);
    if below(2, s) == 0 { -magnitude } else { magnitude }
}

fn arc_tolerance_for(s: &mut u64, delta: f64) -> f64 {
    let d = delta.abs();
    match below(12, s) {
        // Absolute tolerances make a circle of about pi * sqrt(2 * delta / tolerance) points:
        // with a large delta they are taken relative to it instead, most of the time.
        0 | 1 => if d <= 1e6 || below(40, s) == 0 { 0.0 } else { d * 0.02 },
        2 => if d <= 1e6 || below(40, s) == 0 { -1.0 } else { d * 0.07 },
        3 => if d <= 1e6 { 0.25 } else { d * 0.25 },
        4 => if d <= 1e5 { 0.01 } else { d * 0.003 },
        5 => if d <= 1e7 { 3.0 } else { d * 0.2 },
        6 => d * 0.25,
        7 => d * 0.1,
        8 => d * 0.01,
        9 => d * 0.002,
        10 => 1e12,
        _ => d * 0.26,
    }
}

fn random_case(test: u64, index: u64) -> Case {
    let mut s = case_seed(test, index);
    let s = &mut s;
    let scale = [30i64, 500, 20_000, 1_000_000, 100_000_000][below(5, s) as usize];
    let mut paths = Paths::new();
    for _ in 0..1 + below(3, s) {
        shape(s, scale, &mut paths);
    }
    let n = paths.len();
    let mixed = below(4, s) == 0;
    let (j0, e0) = (below(3, s) as usize, [0, 0, 0, 1, 2, 3, 4][below(7, s) as usize]);
    let (mut join, mut end) = (vec![j0; n], vec![e0; n]);
    if mixed {
        for i in 0..n {
            join[i] = below(3, s) as usize;
            end[i] = below(5, s) as usize;
        }
    }
    let delta = delta_for(s, scale);
    Case {
        name: format!("case {index} of test {test:#x} (scale {scale})"),
        paths,
        join,
        end,
        miter_limit: MITER_LIMITS[below(9, s) as usize],
        arc_tolerance: arc_tolerance_for(s, delta),
        shortest_edge_length: match below(9, s) {
            0 => 1.0,
            1 => 1.5,
            2 => scale as f64 * 0.005,
            3 => scale as f64 * 0.05,
            4 => scale as f64 * 0.5,
            _ => 0.0,
        },
        delta,
    }
}

fn run_random(test: u64, cases: u64, name: &str) {
    let mut tally = Tally::default();
    for index in 0..cases {
        tally.check(&random_case(test, index));
    }
    tally.finish(name);
}

// The random cases, in six tests so that they run side by side.
#[test]
fn random_offsets_a() {
    run_random(0x7A1, 12_000, "random_offsets_a");
}

#[test]
fn random_offsets_b() {
    run_random(0x7B2, 12_000, "random_offsets_b");
}

#[test]
fn random_offsets_c() {
    run_random(0x7C3, 12_000, "random_offsets_c");
}

#[test]
fn random_offsets_d() {
    run_random(0x7D4, 12_000, "random_offsets_d");
}

#[test]
fn random_offsets_e() {
    run_random(0x7E5, 12_000, "random_offsets_e");
}

#[test]
fn random_offsets_f() {
    run_random(0x7F6, 12_000, "random_offsets_f");
}

/// Every join type with every end type, miter limit and a ladder of deltas of both signs, on
/// the same inputs.
#[test]
fn every_configuration() {
    let mut tally = Tally::default();
    for index in 0..40u64 {
        let base = random_case(0x8A1, index);
        let size = base.paths.iter().flatten().map(|p| p.x as i64).max().unwrap_or(0)
            - base.paths.iter().flatten().map(|p| p.x as i64).min().unwrap_or(0);
        let size = (size as f64).clamp(4.0, 50_000_000.0);
        for join in 0..3 {
            for end in 0..5 {
                for (m, miter_limit) in [1.0, 2.0, 3.0, 6.0].into_iter().enumerate() {
                    for (d, factor) in [0.004, 0.03, 0.11, 0.26, 0.5, 1.0].into_iter().enumerate() {
                        for sign in [-1.0, 1.0] {
                            let mut c = base.clone();
                            c.join = vec![join; c.paths.len()];
                            c.end = vec![end; c.paths.len()];
                            c.miter_limit = miter_limit;
                            c.delta = sign * size * factor;
                            c.arc_tolerance = [0.25 * c.delta.abs(), 0.05 * c.delta.abs(), 0.004 * c.delta.abs()][(m + d) % 3];
                            c.shortest_edge_length = 0.0;
                            tally.check(&c);
                        }
                    }
                }
            }
        }
    }
    tally.finish("every_configuration");
}

/// The smallest inputs and the ones with nothing to offset.
#[test]
fn degenerate_inputs() {
    let mut tally = Tally::default();
    let p = |v: &[(i32, i32)]| -> Path { v.iter().map(|q| IntPoint::new(q.0, q.1)).collect() };
    let inputs: Vec<Paths> = vec![
        vec![],
        vec![p(&[])],
        vec![p(&[(3, 4)])],
        vec![p(&[(3, 4), (3, 4)])],
        vec![p(&[(0, 0), (10, 0)])],
        vec![p(&[(0, 0), (10, 0), (0, 0)])],
        vec![p(&[(0, 0), (10, 0), (20, 0)])],
        vec![p(&[(0, 0), (10, 0), (5, 0)])],
        vec![p(&[(0, 0), (10, 0), (10, 10), (0, 10)])],
        vec![p(&[(0, 10), (10, 10), (10, 0), (0, 0)])],
        vec![p(&[(0, 0), (10, 0), (10, 10), (0, 10), (0, 0), (0, 0)])],
        vec![p(&[(0, 0), (100, 0), (100, 100), (0, 100)]), p(&[(20, 20), (20, 80), (80, 80), (80, 20)])],
        vec![p(&[(0, 0), (100, 0), (100, 100), (0, 100)]), p(&[(20, 20), (80, 20), (80, 80), (20, 80)])],
        vec![p(&[(0, 0), (1, 0), (1, 1)])],
        vec![p(&[(0, 0), (0, 0), (0, 0)])],
    ];
    for (i, paths) in inputs.into_iter().enumerate() {
        for join in 0..3 {
            for end in 0..5 {
                for delta in [-30.0, -5.0, -1.0, -0.4, -1e-30, 0.0, 1e-21, 0.4, 1.0, 2.5, 5.0, 30.0, 1000.0] {
                    for miter_limit in [1.0, 4.0] {
                        for shortest_edge_length in [0.0, 2.0] {
                            tally.check(&Case {
                                name: format!("degenerate input {i}"),
                                join: vec![join; paths.len()],
                                end: vec![end; paths.len()],
                                paths: paths.clone(),
                                miter_limit,
                                arc_tolerance: if join == 1 { 0.0 } else { 0.1 },
                                shortest_edge_length,
                                delta,
                            });
                        }
                    }
                }
            }
        }
    }
    assert!(tally.failures.is_empty(), "{} of {} degenerate cases differ; the first:\n{}", tally.failures.len(), tally.cases, tally.failures[0]);
}

/// `m_sin` and `m_cos` are `std::sin` and `std::cos` of one argument (clipper.cpp:3529-3530),
/// which the C++ compiler may turn into one call that returns both. On macOS that call
/// (`__sincos_stret`) gives a sine one unit in the last place away from `sin` for about one
/// argument in 500. These are circles around a single point, with a delta large enough for
/// that unit to move a point of the circle: found by search, they differ when the port takes
/// `sin` and `cos` separately there.
#[test]
fn sin_and_cos_of_one_call() {
    let mut tally = Tally::default();
    let witnesses: [(f64, u64); 6] = [
        (500000000.0, 0x41211e6a7690c4b6),
        (1000000000.0, 0x418e2bb2bb1f1ecb),
        (1000000000.0, 0x4187073648f6ebc8),
        (1000000000.0, 0x4185cb201d735d55),
        (750000000.0, 0x4169bff6b234462d),
        (750000000.0, 0x4175c8b95fa728dc),
    ];
    for (i, (delta, arc_bits)) in witnesses.into_iter().enumerate() {
        tally.check(&Case {
            name: format!("circle {i}"),
            paths: vec![vec![IntPoint::new(0, 0)]],
            join: vec![1],
            end: vec![4],
            miter_limit: 2.0,
            arc_tolerance: f64::from_bits(arc_bits),
            shortest_edge_length: 0.0,
            delta,
        });
    }
    tally.finish("sin_and_cos_of_one_call");
}
