//! The polygon tree of `src/clipper.rs` (`PolyNode`, `PolyTree`, `Clipper::execute_poly_tree`,
//! `Clipper::build_result2`, clipper.cpp:122-171, :1098-1110, :2791-2843, and the functions
//! that turn a tree into paths, :4093-4132) against the library itself, compiled in
//! `clipper-z-sys`: the same nodes in the same pre-order, each with the same contour from the
//! same start vertex, the same parent, `IsHole()`, `IsOpen()` and number of children, the same
//! `Total()` and the same return value of `Execute`.
//!
//! The closed inputs are those of `clipper_execute_native_parity.rs` (its generators are copied
//! here); on top of them come nests (holes inside outers inside holes) and open subject paths,
//! which only the `PolyTree` `Execute` takes. Every clip type, every pair of fill types,
//! StrictlySimple, PreserveCollinear and ReverseSolution on and off, paths added one by one
//! (`AddPath`) or together (`AddPaths`).
//!
//! Every case is made from its own seed, so a failure names the case and prints its input.
//! `CLIPPER_POLYTREE_PARITY_LOG=<file>` writes every failing input to that file.
//! `CLIPPER_POLYTREE_TRACE=<file>` writes every case to that file before it runs, to find an
//! input on which the library itself dies (see `drop_horizontal_reversals`).

use slicer::clipper::{
    closed_paths_from_poly_tree, open_paths_from_poly_tree, poly_tree_to_paths, simplify_polygon, simplify_polygons,
    ClipType, Clipper, IntPoint, Path, Paths, PolyFillType, PolyTree, PolyType, POLY_TREE_ROOT,
};
use std::fmt::Write as _;

const CLIP_TYPES: [ClipType; 4] =
    [ClipType::ctIntersection, ClipType::ctUnion, ClipType::ctDifference, ClipType::ctXor];
const FILL_TYPES: [PolyFillType; 4] =
    [PolyFillType::pftEvenOdd, PolyFillType::pftNonZero, PolyFillType::pftPositive, PolyFillType::pftNegative];
const LIMIT: i64 = 1_000_000_000;
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
    subj: Paths,
    /// One per subject: closed or open.
    subj_closed: Vec<bool>,
    clip: Paths,
    clip_type: usize,
    subj_fill: usize,
    clip_fill: usize,
    strictly_simple: bool,
    preserve_collinear: bool,
    reverse_solution: bool,
    /// Paths go in through `AddPaths` instead of `AddPath`.
    add_paths: bool,
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
            "{}: clip_type={:?} subj_fill={:?} clip_fill={:?} strictly_simple={} preserve_collinear={} reverse_solution={} add_paths={}\n  subj: {}\n  subj_closed: {:?}\n  clip: {}",
            self.name,
            CLIP_TYPES[self.clip_type],
            FILL_TYPES[self.subj_fill],
            FILL_TYPES[self.clip_fill],
            self.strictly_simple,
            self.preserve_collinear,
            self.reverse_solution,
            self.add_paths,
            show_paths(&self.subj),
            self.subj_closed,
            show_paths(&self.clip),
        )
    }
}

/// A tree as the shim lists it: contours in pre-order and, per node, the parent's index in
/// the listing (-1 for the tree), IsHole, IsOpen, ChildCount.
#[derive(Debug, PartialEq, Eq)]
struct Listing {
    ok: bool,
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

fn list_tree(ok: bool, tree: &PolyTree) -> Listing {
    let mut contours = Paths::new();
    let mut info = Vec::new();
    list_node(tree, POLY_TREE_ROOT, -1, &mut contours, &mut info);
    // GetFirst / GetNext walk the same nodes in the same order.
    let mut walked = Paths::new();
    let mut node = tree.get_first();
    while let Some(n) = node {
        walked.push(tree.node(n).contour.clone());
        node = tree.get_next(n);
    }
    assert_eq!(walked, contours, "GetFirst/GetNext do not walk the tree in pre-order");
    assert_eq!(tree.get_next(POLY_TREE_ROOT), tree.get_first());
    let total = if contours.is_empty() && tree.child_count() == 0 { 0 } else { tree.total() };
    Listing { ok, contours, info, total }
}

fn rust_tree(c: &Case) -> (bool, PolyTree) {
    let mut clipper = Clipper::new(0);
    clipper.set_strictly_simple(c.strictly_simple);
    clipper.set_preserve_collinear(c.preserve_collinear);
    clipper.set_reverse_solution(c.reverse_solution);
    if !c.add_paths {
        for (p, &closed) in c.subj.iter().zip(&c.subj_closed) {
            clipper.add_path(p, PolyType::ptSubject, closed);
        }
        for p in &c.clip {
            clipper.add_path(p, PolyType::ptClip, true);
        }
    } else {
        let (mut closed, mut open) = (Paths::new(), Paths::new());
        for (p, &is_closed) in c.subj.iter().zip(&c.subj_closed) {
            if is_closed { closed.push(p.clone()) } else { open.push(p.clone()) }
        }
        clipper.add_paths(&closed, PolyType::ptSubject, true);
        clipper.add_paths(&open, PolyType::ptSubject, false);
        clipper.add_paths(&c.clip, PolyType::ptClip, true);
    }
    let mut tree = PolyTree::new();
    let ok = clipper.execute_poly_tree(CLIP_TYPES[c.clip_type], &mut tree, FILL_TYPES[c.subj_fill], FILL_TYPES[c.clip_fill]);
    (ok, tree)
}

fn flat(ps: &Paths) -> (Vec<i32>, Vec<i32>) {
    (ps.iter().flatten().flat_map(|p| [p.x, p.y]).collect(), ps.iter().map(|p| p.len() as i32).collect())
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

/// `output`: 0 the tree, 1 PolyTreeToPaths, 2 ClosedPathsFromPolyTree, 3 OpenPathsFromPolyTree.
fn run_library(c: &Case, output: i32) -> Listing {
    let (subj_xy, subj_lens) = flat(&c.subj);
    let (clip_xy, clip_lens) = flat(&c.clip);
    let subj_closed: Vec<i32> = c.subj_closed.iter().map(|&b| b as i32).collect();
    let mut node_info = vec![0i32; 4 * NODE_CAP];
    let (mut num_nodes, mut total, mut succeeded) = (0i32, 0i32, -1i32);
    let contours = unsafe {
        take_paths(clipper_z_sys::cz_ref_execute_polytree(
            subj_xy.as_ptr(),
            subj_lens.as_ptr(),
            subj_closed.as_ptr(),
            subj_lens.len() as i32,
            clip_xy.as_ptr(),
            clip_lens.as_ptr(),
            clip_lens.len() as i32,
            c.clip_type as i32,
            c.subj_fill as i32,
            c.clip_fill as i32,
            c.strictly_simple as i32,
            c.preserve_collinear as i32,
            c.reverse_solution as i32,
            c.add_paths as i32,
            output,
            node_info.as_mut_ptr(),
            NODE_CAP as i32,
            &mut num_nodes,
            &mut total,
            &mut succeeded,
        ))
    };
    let mut info = Vec::new();
    if output == 0 {
        assert!(num_nodes as usize <= NODE_CAP, "the node buffer is too small: {num_nodes}");
        assert_eq!(num_nodes as usize, contours.len());
        info = node_info[..4 * num_nodes as usize].chunks(4).map(|q| [q[0], q[1], q[2], q[3]]).collect();
    }
    Listing { ok: succeeded == 1, contours, info, total }
}

/// What became of the comparisons of one test.
#[derive(Default)]
struct Tally {
    cases: usize,
    nonempty: usize,
    nodes: usize,
    /// Nodes that are holes, nodes below the second level, open nodes.
    holes: usize,
    deep: usize,
    open: usize,
    failed_runs: usize,
    failures: Vec<String>,
    by_clip_type: [usize; 4],
    by_flags: [usize; 8],
}

impl Tally {
    fn check(&mut self, c: &Case) {
        self.cases += 1;
        if let Some(path) = std::env::var_os("CLIPPER_POLYTREE_TRACE") {
            std::fs::write(path, c.describe()).unwrap();
        }
        let want = run_library(c, 0);
        if !want.ok {
            self.failed_runs += 1;
        }
        if !want.contours.is_empty() {
            self.nonempty += 1;
            self.nodes += want.contours.len();
            for (i, q) in want.info.iter().enumerate() {
                self.holes += q[1] as usize;
                self.open += q[2] as usize;
                // A node whose parent has a parent that is not the tree.
                if q[0] >= 0 && want.info[q[0] as usize][0] >= 0 {
                    self.deep += 1;
                }
                let _ = i;
            }
        }
        let got = std::panic::catch_unwind(|| {
            let (ok, tree) = rust_tree(c);
            let listing = list_tree(ok, &tree);
            let (mut all, mut closed, mut open) = (Paths::new(), Paths::new(), Paths::new());
            poly_tree_to_paths(&tree, &mut all);
            closed_paths_from_poly_tree(&tree, &mut closed);
            open_paths_from_poly_tree(&tree, &mut open);
            (listing, all, closed, open)
        });
        let what = match got {
            Ok((listing, all, closed, open)) => {
                if listing != want {
                    format!("  library: {want:?}\n  port:    {listing:?}")
                } else {
                    // The three conversions of the tree to paths, on every fourth case.
                    if self.cases % 4 != 0 {
                        return;
                    }
                    let (want_all, want_closed, want_open) =
                        (run_library(c, 1).contours, run_library(c, 2).contours, run_library(c, 3).contours);
                    if all == want_all && closed == want_closed && open == want_open {
                        return;
                    }
                    format!(
                        "  the tree is equal, its paths are not:\n  library: all {want_all:?}\n    closed {want_closed:?}\n    open {want_open:?}\n  port:    all {all:?}\n    closed {closed:?}\n    open {open:?}"
                    )
                }
            }
            Err(_) => format!("  library: {want:?}\n  port:    panicked"),
        };
        self.by_clip_type[c.clip_type] += 1;
        self.by_flags[c.strictly_simple as usize * 4 + c.preserve_collinear as usize * 2 + c.reverse_solution as usize] += 1;
        self.failures.push(format!("{}\n{}", c.describe(), what));
    }

    fn finish(self, test: &str) {
        if let Some(path) = std::env::var_os("CLIPPER_POLYTREE_PARITY_LOG") {
            use std::io::Write as _;
            let mut f = std::fs::OpenOptions::new().create(true).append(true).open(path).expect("the log file opens");
            writeln!(
                f,
                "{test}: {} cases, {} with a non-empty tree ({} nodes: {} holes, {} below the second level, {} open), {} where Execute returned false, {} differ",
                self.cases,
                self.nonempty,
                self.nodes,
                self.holes,
                self.deep,
                self.open,
                self.failed_runs,
                self.failures.len()
            )
            .unwrap();
            for failure in &self.failures {
                writeln!(f, "{failure}\n").unwrap();
            }
        }
        assert!(
            self.failures.is_empty(),
            "{test}: {} of {} cases differ from the library; by clip type (intersection, union, difference, xor) {:?}; \
             by flags (index = strictly_simple*4 + preserve_collinear*2 + reverse_solution) {:?}; the first:\n{}",
            self.failures.len(),
            self.cases,
            self.by_clip_type,
            self.by_flags,
            self.failures[0]
        );
        // The inputs must reach the sweep: most cases have something to output.
        assert!(self.nonempty * 4 > self.cases, "{test}: only {} of {} cases have a tree", self.nonempty, self.cases);
    }
}

/// Zigzag with `teeth` downward points, all at Y = `base` (Clipper's Y grows downwards: these
/// are the local minima), closed over the top. `flat` gives every other tooth a horizontal
/// bottom, `ragged` varies the height of the peaks between the teeth.
fn comb(x0: i64, base: i64, teeth: i64, w: i64, h: i64, flat: bool, ragged: bool, s: &mut u64) -> Path {
    let mut p = Path::new();
    p.push(pt(x0, base - h, 1));
    for i in 0..teeth {
        let x = x0 + i * w;
        if flat && i % 2 == 1 && w >= 4 {
            p.push(pt(x + w / 4, base, 1));
            p.push(pt(x + w / 2 + w / 4, base, 1));
        } else {
            p.push(pt(x + w / 2, base, 1));
        }
        let peak = if ragged { h - below(h.max(2) as u64 / 2 + 1, s) as i64 } else { h };
        p.push(pt(x + w, base - peak.max(1), 1));
    }
    p.push(pt(x0 + teeth * w, base - 2 * h - 1, 1));
    p.push(pt(x0, base - 2 * h - 1, 1));
    p
}

fn polygon(s: &mut u64, scale: i64, grid: i64, earlier: &[Path]) -> Path {
    let mut poly = Path::new();
    match below(12, s) {
        // Random points: mostly self-intersecting.
        0 | 1 => {
            for _ in 0..3 + below(7, s) {
                poly.push(pt(r(scale, s), r(scale, s), grid));
            }
        }
        // Convex.
        2 => {
            let (cx, cy) = (r(scale / 2, s), r(scale / 2, s));
            let radius = (1 + below(scale as u64 / 2 + 1, s)) as f64;
            let n = 3 + below(9, s);
            let phase = below(628, s) as f64 / 100.0;
            for i in 0..n {
                let a = phase + i as f64 * std::f64::consts::TAU / n as f64;
                poly.push(pt(cx + (radius * a.cos()) as i64, cy + (radius * a.sin()) as i64, grid));
            }
        }
        // A star: concave, and with a low inner radius self-touching.
        3 => {
            let (cx, cy) = (r(scale / 2, s), r(scale / 2, s));
            let outer = (1 + below(scale as u64 / 2 + 1, s)) as f64;
            let inner = outer * (below(90, s) as f64 / 100.0);
            let n = 2 * (3 + below(5, s));
            let phase = below(628, s) as f64 / 100.0;
            for i in 0..n {
                let a = phase + i as f64 * std::f64::consts::TAU / n as f64;
                let rad = if i % 2 == 0 { outer } else { inner };
                poly.push(pt(cx + (rad * a.cos()) as i64, cy + (rad * a.sin()) as i64, grid));
            }
        }
        // Axis-aligned rectangle: horizontal edges, shared and overlapping on the grid.
        4 | 5 => {
            let (x0, y0) = (r(scale, s), r(scale, s));
            let (w, h) = (1 + below(scale as u64 + 1, s) as i64, 1 + below(scale as u64 + 1, s) as i64);
            let g = grid.max(1);
            poly.extend([pt(x0, y0, grid), pt(x0 + w + g, y0, grid), pt(x0 + w + g, y0 + h + g, grid), pt(x0, y0 + h + g, grid)]);
        }
        // Rectilinear walk: runs of horizontals and verticals, spikes, overlaps.
        6 => {
            let (mut x, mut y) = (r(scale / 2, s), r(scale / 2, s));
            let mut horizontal = below(2, s) == 0;
            for _ in 0..4 + below(8, s) {
                poly.push(pt(x, y, grid));
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
        // A small comb: several local minima at one Y, on the grid's rows.
        7 => {
            let teeth = 2 + below(6, s) as i64;
            let w = (scale / (teeth + 1)).max(2);
            let h = (1 + below(scale as u64, s) as i64).min(scale);
            let base = pt(0, r(scale / 2, s), grid).y as i64;
            poly = comb(r(scale / 2, s), base, teeth, w, h, below(2, s) == 0, below(2, s) == 0, s);
        }
        // A copy of an earlier polygon: in place, moved by one unit, by one grid step, or by
        // its own width or height so that the two touch.
        8 | 9 if !earlier.is_empty() => {
            let src = &earlier[below(earlier.len() as u64, s) as usize];
            let (min_x, max_x) = (src.iter().map(|p| p.x).min().unwrap() as i64, src.iter().map(|p| p.x).max().unwrap() as i64);
            let (min_y, max_y) = (src.iter().map(|p| p.y).min().unwrap() as i64, src.iter().map(|p| p.y).max().unwrap() as i64);
            let (dx, dy) = match below(8, s) {
                0 => (0, 0),
                1 => (1, 0),
                2 => (0, 1),
                3 => (grid, 0),
                4 => (0, grid),
                5 => (max_x - min_x, 0),
                6 => (0, max_y - min_y),
                _ => ((max_x - min_x) / 2, (max_y - min_y) / 2),
            };
            for p in src {
                poly.push(pt(p.x as i64 + dx, p.y as i64 + dy, 1));
            }
        }
        // Nearly collinear: thin slivers and their rounding.
        10 => {
            let (x0, y0) = (r(scale, s), r(scale, s));
            let (dx, dy) = (r(scale / 4 + 1, s), r(scale / 4 + 1, s));
            for i in 0..3 + below(4, s) as i64 {
                poly.push(pt(x0 + i * dx + r(1, s), y0 + i * dy + r(1, s), 1));
            }
        }
        // Random points off the grid.
        _ => {
            for _ in 0..3 + below(5, s) {
                poly.push(pt(r(scale, s), r(scale, s), 1));
            }
        }
    }
    // Duplicate vertices, vertices in the middle of an edge, another start vertex, the
    // other direction.
    if below(4, s) == 0 && !poly.is_empty() {
        let i = below(poly.len() as u64, s) as usize;
        poly.insert(i, poly[i]);
    }
    if below(4, s) == 0 && poly.len() >= 2 {
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
    poly
}

fn configure(name: String, subj: Paths, subj_closed: Vec<bool>, clip: Paths, s: &mut u64) -> Case {
    Case {
        name,
        subj,
        subj_closed,
        clip,
        clip_type: below(4, s) as usize,
        subj_fill: below(4, s) as usize,
        clip_fill: below(4, s) as usize,
        strictly_simple: below(2, s) == 0,
        preserve_collinear: below(2, s) == 0,
        reverse_solution: below(2, s) == 0,
        add_paths: below(3, s) == 0,
    }
}

fn scale_and_grid(s: &mut u64) -> (i64, i64) {
    let scale = [8i64, 60, 2_000, 400_000, 60_000_000, 900_000_000][below(6, s) as usize];
    let grid = match below(4, s) {
        0 => 1,
        1 => (scale / 4).max(1),
        2 => (scale / 16).max(1),
        _ => (scale / 64).max(1),
    };
    (scale, grid)
}

/// The closed cases of `clipper_execute_native_parity.rs`.
fn random_case(test: u64, index: u64) -> Case {
    let mut s = case_seed(test, index);
    let s = &mut s;
    let (scale, grid) = scale_and_grid(s);
    let mut all: Paths = Vec::new();
    let n_subj = 1 + below(4, s) as usize;
    let n_clip = below(4, s) as usize;
    for _ in 0..n_subj + n_clip {
        let p = polygon(s, scale, grid, &all);
        all.push(p);
    }
    let clip = all.split_off(n_subj);
    let closed = vec![true; all.len()];
    configure(format!("case {index} of test {test:#x} (scale {scale}, grid {grid})"), all, closed, clip, s)
}

/// An open path: random points, a walk on the grid, a horizontal or vertical line through
/// everything, or an earlier polygon taken as a polyline (so that it runs along its edges).
fn polyline(s: &mut u64, scale: i64, grid: i64, earlier: &[Path]) -> Path {
    let mut line = Path::new();
    match below(8, s) {
        0 | 1 => {
            for _ in 0..2 + below(7, s) {
                line.push(pt(r(scale, s), r(scale, s), grid));
            }
        }
        2 => {
            for _ in 0..2 + below(5, s) {
                line.push(pt(r(scale, s), r(scale, s), 1));
            }
        }
        3 => {
            let y = r(scale, s);
            line.extend([pt(-scale, y, grid), pt(scale, y, grid)]);
            if below(2, s) == 0 {
                line.insert(1, pt(r(scale, s), y, grid));
            }
        }
        4 => {
            let x = r(scale, s);
            line.extend([pt(x, -scale, grid), pt(x, scale, grid)]);
        }
        5 | 6 if !earlier.is_empty() => {
            line = earlier[below(earlier.len() as u64, s) as usize].clone();
            if below(2, s) == 0 && !line.is_empty() {
                // Closed back onto its start.
                line.push(line[0]);
            }
        }
        _ => {
            let (mut x, mut y) = (r(scale / 2, s), r(scale / 2, s));
            let mut horizontal = below(2, s) == 0;
            for _ in 0..2 + below(8, s) {
                line.push(pt(x, y, grid));
                if horizontal {
                    x += r(scale / 2, s);
                } else {
                    y += r(scale / 2, s);
                }
                if below(4, s) != 0 {
                    horizontal = !horizontal;
                }
            }
        }
    }
    if below(5, s) == 0 && !line.is_empty() {
        let i = below(line.len() as u64, s) as usize;
        line.insert(i, line[i]);
    }
    if below(2, s) == 0 {
        line.reverse();
    }
    drop_horizontal_reversals(&mut line);
    line
}

/// Removes the points at which an open path turns back along a horizontal line.
///
/// The library cannot take such a path: `Clipper::ProcessHorizontal` (clipper.cpp:2227-2246)
/// sets `maxIt` or `maxRit`, whichever the direction of the first horizontal of a run needs,
/// and leaves the other default-constructed; when the run turns round (only an open path can
/// do that: `AddPath` removes such a spike from a closed one) it dereferences that null
/// iterator as soon as `m_Maxima` is not empty, and the process dies. There is nothing to
/// compare the port with on these inputs.
fn drop_horizontal_reversals(line: &mut Path) {
    loop {
        // The points that differ from their predecessor: AddPath drops the others.
        let mut distinct: Vec<usize> = Vec::new();
        for i in 0..line.len() {
            if distinct.last().map_or(true, |&l| line[l] != line[i]) {
                distinct.push(i);
            }
        }
        let turn = distinct.windows(3).find(|w| {
            let (a, b, c) = (line[w[0]], line[w[1]], line[w[2]]);
            a.y == b.y && b.y == c.y && (b.x > a.x) != (c.x > b.x)
        });
        match turn {
            Some(w) => {
                line.remove(w[1]);
            }
            None => break,
        }
    }
}

/// Open subjects, with or without closed subjects, against closed clips.
fn open_case(test: u64, index: u64) -> Case {
    let mut s = case_seed(test, index);
    let s = &mut s;
    let (scale, grid) = scale_and_grid(s);
    let mut all: Paths = Vec::new();
    let n_clip = 1 + below(3, s) as usize;
    for _ in 0..n_clip {
        let p = polygon(s, scale, grid, &all);
        all.push(p);
    }
    let mut subj = Paths::new();
    let mut subj_closed = Vec::new();
    for _ in 0..1 + below(4, s) {
        if below(4, s) == 0 {
            subj.push(polygon(s, scale, grid, &all));
            subj_closed.push(true);
        } else {
            subj.push(polyline(s, scale, grid, &all));
            subj_closed.push(false);
        }
    }
    configure(format!("open case {index} of test {test:#x} (scale {scale}, grid {grid})"), subj, subj_closed, all, s)
}

/// A ring around (cx, cy): a rectangle or a convex polygon of the given radius.
fn ring(cx: i64, cy: i64, radius: i64, kind: u64, phase: f64, grid: i64) -> Path {
    let mut p = Path::new();
    if kind == 0 {
        p.extend([
            pt(cx - radius, cy - radius, grid),
            pt(cx + radius, cy - radius, grid),
            pt(cx + radius, cy + radius, grid),
            pt(cx - radius, cy + radius, grid),
        ]);
    } else {
        let n = 3 + kind;
        for i in 0..n {
            let a = phase + i as f64 * std::f64::consts::TAU / n as f64;
            p.push(pt(cx + (radius as f64 * a.cos()) as i64, cy + (radius as f64 * a.sin()) as i64, grid));
        }
    }
    p
}

/// Nests: rings inside rings, so that the result has holes inside outers inside holes. One
/// to three nests, apart, touching or overlapping; rings alternate in direction, all turn
/// the same way, or turn at random; some rings are clips.
fn nested_case(test: u64, index: u64) -> Case {
    let mut s = case_seed(test, index);
    let s = &mut s;
    let scale = [40i64, 600, 20_000, 4_000_000, 300_000_000][below(5, s) as usize];
    let grid = if below(3, s) == 0 { (scale / 32).max(1) } else { 1 };
    let (mut subj, mut clip) = (Paths::new(), Paths::new());
    let nests = 1 + below(3, s);
    let direction = below(3, s);
    let clip_share = below(4, s);
    for nest in 0..nests {
        let levels = 2 + below(6, s) as i64;
        let outer = scale / 2 + below(scale as u64 / 2 + 1, s) as i64;
        let (cx, cy) = match (nest, below(4, s)) {
            (0, _) => (r(scale / 4, s), r(scale / 4, s)),
            // Beside the first: apart, touching (rectangles share an edge) or overlapping.
            (_, 0) => (2 * scale + scale / 2, r(scale / 4, s)),
            (_, 1) => (2 * outer, 0),
            (_, 2) => (outer, r(scale / 2, s)),
            _ => (r(scale, s), r(scale, s)),
        };
        let kind = if below(2, s) == 0 { 0 } else { 1 + below(6, s) };
        let phase = below(628, s) as f64 / 100.0;
        for level in 0..levels {
            let radius = outer * (levels - level) / levels;
            if radius < 1 {
                break;
            }
            // Now and then a ring off centre, so that it touches or crosses its parent.
            let (ox, oy) = if below(6, s) == 0 { (r(outer / levels + 1, s), r(outer / levels + 1, s)) } else { (0, 0) };
            let mut p = ring(cx + ox, cy + oy, radius, kind, phase, grid);
            let reverse = match direction {
                0 => level % 2 == 1,
                1 => false,
                _ => below(2, s) == 0,
            };
            if reverse {
                p.reverse();
            }
            if clip_share != 0 && below(4, s) < clip_share {
                clip.push(p);
            } else {
                subj.push(p);
            }
        }
    }
    if below(4, s) == 0 {
        let extra = polygon(s, scale, grid, &subj);
        subj.push(extra);
    }
    let mut subj_closed = vec![true; subj.len()];
    // Now and then an open path across the nests.
    if below(4, s) == 0 {
        let line = polyline(s, 2 * scale, grid, &subj);
        subj.push(line);
        subj_closed.push(false);
    }
    configure(format!("nested case {index} of test {test:#x} (scale {scale}, grid {grid})"), subj, subj_closed, clip, s)
}

fn run(test: u64, cases: u64, name: &str, make: fn(u64, u64) -> Case) {
    let mut tally = Tally::default();
    for index in 0..cases {
        tally.check(&make(test, index));
    }
    tally.finish(name);
}

#[test]
fn random_polygons_a() {
    run(0x1A1, 15_000, "random_polygons_a", random_case);
}

#[test]
fn random_polygons_b() {
    run(0x1B2, 15_000, "random_polygons_b", random_case);
}

#[test]
fn random_polygons_c() {
    run(0x1C3, 15_000, "random_polygons_c", random_case);
}

#[test]
fn open_paths_a() {
    run(0x2A1, 15_000, "open_paths_a", open_case);
}

#[test]
fn open_paths_b() {
    run(0x2B2, 15_000, "open_paths_b", open_case);
}

#[test]
fn nested_polygons_a() {
    run(0x3A1, 10_000, "nested_polygons_a", nested_case);
}

#[test]
fn nested_polygons_b() {
    run(0x3B2, 10_000, "nested_polygons_b", nested_case);
}

/// The nests must give deep trees and holes, the open cases open nodes: the generators reach
/// what they are for.
#[test]
fn the_generators_reach_deep_trees_holes_and_open_nodes() {
    let mut tally = Tally::default();
    for index in 0..1_500 {
        tally.check(&nested_case(0x3C3, index));
    }
    assert!(tally.holes > 500 && tally.deep > 500, "holes {} deep {}", tally.holes, tally.deep);
    tally.finish("nested (reach)");
    let mut tally = Tally::default();
    for index in 0..1_500 {
        tally.check(&open_case(0x2C3, index));
    }
    assert!(tally.open > 300, "open nodes {}", tally.open);
    tally.finish("open (reach)");
}

/// Every combination of clip type, fill types and the three options, on the same few inputs:
/// closed, nested and with open paths.
#[test]
fn every_configuration() {
    let mut tally = Tally::default();
    for index in 0..18u64 {
        let base = match index % 3 {
            0 => random_case(0x4E5, index),
            1 => nested_case(0x4E5, index),
            _ => open_case(0x4E5, index),
        };
        for clip_type in 0..4 {
            for subj_fill in 0..4 {
                for clip_fill in 0..4 {
                    for flags in 0..8 {
                        let mut c = base.clone();
                        c.clip_type = clip_type;
                        c.subj_fill = subj_fill;
                        c.clip_fill = clip_fill;
                        c.strictly_simple = flags & 4 != 0;
                        c.preserve_collinear = flags & 2 != 0;
                        c.reverse_solution = flags & 1 != 0;
                        c.add_paths = (clip_type + subj_fill + flags) % 2 == 0;
                        tally.check(&c);
                    }
                }
            }
        }
    }
    tally.finish("every_configuration");
}

/// Inputs with nothing to clip, and the smallest ones.
#[test]
fn degenerate_inputs() {
    let mut tally = Tally::default();
    let p = |v: &[(i32, i32)]| -> Path { v.iter().map(|q| IntPoint::new(q.0, q.1)).collect() };
    let square = p(&[(0, 0), (10, 0), (10, 10), (0, 10)]);
    let inputs: Vec<(Paths, Vec<bool>, Paths)> = vec![
        (vec![], vec![], vec![]),
        (vec![p(&[(0, 0), (5, 5)])], vec![true], vec![]),
        (vec![p(&[(0, 0), (5, 5)])], vec![false], vec![]),
        (vec![p(&[(-5, 5), (15, 5)])], vec![false], vec![square.clone()]),
        (vec![p(&[(-5, 0), (15, 0)])], vec![false], vec![square.clone()]),
        (vec![p(&[(5, 5)])], vec![false], vec![square.clone()]),
        (vec![p(&[(5, 5), (5, 5)])], vec![false], vec![square.clone()]),
        (vec![p(&[(2, 2), (8, 8)]), square.clone()], vec![false, true], vec![]),
        (vec![square.clone()], vec![true], vec![]),
        (vec![], vec![], vec![square.clone()]),
        (vec![p(&[(0, 0), (0, 0), (0, 0)])], vec![true], vec![p(&[(1, 1), (1, 1), (2, 2)])]),
        (vec![square.clone(), p(&[(2, 2), (2, 8), (8, 8), (8, 2)]), p(&[(4, 4), (6, 4), (6, 6), (4, 6)])], vec![true; 3], vec![]),
    ];
    for (i, (subj, subj_closed, clip)) in inputs.into_iter().enumerate() {
        for clip_type in 0..4 {
            for fill in 0..4 {
                for flags in 0..16 {
                    tally.check(&Case {
                        name: format!("degenerate input {i}"),
                        subj: subj.clone(),
                        subj_closed: subj_closed.clone(),
                        clip: clip.clone(),
                        clip_type,
                        subj_fill: fill,
                        clip_fill: (fill + flags) % 4,
                        strictly_simple: flags & 4 != 0,
                        preserve_collinear: flags & 2 != 0,
                        reverse_solution: flags & 1 != 0,
                        add_paths: flags & 8 != 0,
                    });
                }
            }
        }
    }
    assert!(tally.failures.is_empty(), "{} degenerate cases differ; the first:\n{}", tally.failures.len(), tally.failures[0]);
}

/// `SimplifyPolygon` and `SimplifyPolygons` (clipper.cpp:3842-3850, clipper.hpp:558-566).
#[test]
fn simplify_polygon_and_polygons() {
    let mut failures = Vec::new();
    let mut nonempty = 0;
    let cases = 12_000u64;
    for index in 0..cases {
        let mut s = case_seed(0x5F6, index);
        let s = &mut s;
        let (scale, grid) = scale_and_grid(s);
        let mut all = Paths::new();
        for _ in 0..1 + below(4, s) {
            let p = polygon(s, scale, grid, &all);
            all.push(p);
        }
        let fill = below(4, s) as usize;
        let which = below(2, s) as i32;
        let (xy, lens) = flat(&all);
        let want = unsafe {
            take_paths(clipper_z_sys::cz_ref_simplify_polygons(xy.as_ptr(), lens.as_ptr(), lens.len() as i32, fill as i32, which))
        };
        let got = std::panic::catch_unwind(|| {
            if which == 0 { simplify_polygon(&all[0], FILL_TYPES[fill]) } else { simplify_polygons(&all, FILL_TYPES[fill]) }
        });
        nonempty += !want.is_empty() as usize;
        if got.as_ref().ok() != Some(&want) {
            failures.push(format!(
                "case {index}: which={which} fill={:?}\n  in: {}\n  library: {want:?}\n  port:    {got:?}",
                FILL_TYPES[fill],
                show_paths(&all)
            ));
        }
    }
    assert!(failures.is_empty(), "{} of {cases} cases differ; the first:\n{}", failures.len(), failures[0]);
    assert!(nonempty * 4 > cases as usize, "only {nonempty} of {cases} cases have a result");
}
