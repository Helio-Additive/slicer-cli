//! `Clipper::execute` of `src/clipper.rs` (the port of the Clipper class, clipper.cpp:1048-3313
//! and :3762-3826) against the library itself, compiled in `clipper-z-sys`: the same number of
//! paths, in the same order, each with the same points from the same start vertex, and the same
//! return value.
//!
//! The inputs are made to put the sweep on its decisions: random, convex, star-shaped,
//! rectilinear and comb-shaped polygons, snapped to a grid so that edges are shared, collinear
//! and overlapping and vertices coincide; copies of polygons, moved to touch or not at all;
//! duplicate and collinear vertices; magnitudes from 8 to 9e8; every clip type, every pair of
//! fill types, and StrictlySimple, PreserveCollinear and ReverseSolution on and off. The comb
//! test gives the local minima list far more than 23 entries at one Y, where libc++'s
//! `std::sort` stops being stable.
//!
//! Every case is made from its own seed, so a failure names the case and prints its input.
//! `CLIPPER_EXECUTE_PARITY_LOG=<file>` writes every failing input to that file.

use slicer::clipper::{ClipType, Clipper, IntPoint, Path, Paths, PolyFillType, PolyType};
use std::fmt::Write as _;

const CLIP_TYPES: [ClipType; 4] =
    [ClipType::ctIntersection, ClipType::ctUnion, ClipType::ctDifference, ClipType::ctXor];
const FILL_TYPES: [PolyFillType; 4] =
    [PolyFillType::pftEvenOdd, PolyFillType::pftNonZero, PolyFillType::pftPositive, PolyFillType::pftNegative];
const LIMIT: i64 = 1_000_000_000;

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
    clip: Paths,
    clip_type: usize,
    subj_fill: usize,
    clip_fill: usize,
    strictly_simple: bool,
    preserve_collinear: bool,
    reverse_solution: bool,
}

impl Case {
    fn describe(&self) -> String {
        let paths = |ps: &Paths| -> String {
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
        };
        format!(
            "{}: clip_type={:?} subj_fill={:?} clip_fill={:?} strictly_simple={} preserve_collinear={} reverse_solution={}\n  subj: {}\n  clip: {}",
            self.name,
            CLIP_TYPES[self.clip_type],
            FILL_TYPES[self.subj_fill],
            FILL_TYPES[self.clip_fill],
            self.strictly_simple,
            self.preserve_collinear,
            self.reverse_solution,
            paths(&self.subj),
            paths(&self.clip),
        )
    }
}

fn run_rust(c: &Case) -> (bool, Paths) {
    let mut clipper = Clipper::new(0);
    clipper.set_strictly_simple(c.strictly_simple);
    clipper.set_preserve_collinear(c.preserve_collinear);
    clipper.set_reverse_solution(c.reverse_solution);
    for p in &c.subj {
        clipper.add_path(p, PolyType::ptSubject, true);
    }
    for p in &c.clip {
        clipper.add_path(p, PolyType::ptClip, true);
    }
    let mut solution = Paths::new();
    let ok = clipper.execute(CLIP_TYPES[c.clip_type], &mut solution, FILL_TYPES[c.subj_fill], FILL_TYPES[c.clip_fill]);
    (ok, solution)
}

fn run_library(c: &Case) -> (bool, Paths) {
    let flat = |ps: &Paths| -> (Vec<i32>, Vec<i32>) {
        (ps.iter().flatten().flat_map(|p| [p.x, p.y]).collect(), ps.iter().map(|p| p.len() as i32).collect())
    };
    let (subj_xy, subj_lens) = flat(&c.subj);
    let (clip_xy, clip_lens) = flat(&c.clip);
    let mut succeeded: i32 = -1;
    let out = unsafe {
        clipper_z_sys::cz_ref_execute(
            subj_xy.as_ptr(),
            subj_lens.as_ptr(),
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
            &mut succeeded,
        )
    };
    let mut paths = Paths::new();
    unsafe {
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
    }
    (succeeded == 1, paths)
}

/// What became of the comparisons of one test.
#[derive(Default)]
struct Tally {
    cases: usize,
    nonempty: usize,
    paths: usize,
    failed_runs: usize,
    failures: Vec<String>,
    /// Failures per clip type, then per (strictly_simple, preserve_collinear, reverse_solution).
    by_clip_type: [usize; 4],
    by_flags: [usize; 8],
}

impl Tally {
    fn check(&mut self, c: &Case) {
        self.cases += 1;
        let want = run_library(c);
        let got = std::panic::catch_unwind(|| run_rust(c));
        if !want.0 {
            self.failed_runs += 1;
        }
        if !want.1.is_empty() {
            self.nonempty += 1;
            self.paths += want.1.len();
        }
        let what = match got {
            Ok(got) if got == want => return,
            Ok(got) => format!("  library: ok={} {:?}\n  port:    ok={} {:?}", want.0, want.1, got.0, got.1),
            Err(_) => format!("  library: ok={} {:?}\n  port:    panicked", want.0, want.1),
        };
        self.by_clip_type[c.clip_type] += 1;
        self.by_flags[c.strictly_simple as usize * 4 + c.preserve_collinear as usize * 2 + c.reverse_solution as usize] += 1;
        self.failures.push(format!("{}\n{}", c.describe(), what));
    }

    fn finish(self, test: &str) {
        if let Some(path) = std::env::var_os("CLIPPER_EXECUTE_PARITY_LOG") {
            use std::io::Write as _;
            let mut f = std::fs::OpenOptions::new().create(true).append(true).open(path).expect("the log file opens");
            writeln!(
                f,
                "{test}: {} cases, {} with a non-empty solution ({} paths), {} where Execute returned false, {} differ",
                self.cases,
                self.nonempty,
                self.paths,
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
        assert!(self.nonempty * 4 > self.cases, "{test}: only {} of {} cases have a solution", self.nonempty, self.cases);
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

fn configure(name: String, subj: Paths, clip: Paths, s: &mut u64) -> Case {
    Case {
        name,
        subj,
        clip,
        clip_type: below(4, s) as usize,
        subj_fill: below(4, s) as usize,
        clip_fill: below(4, s) as usize,
        strictly_simple: below(2, s) == 0,
        preserve_collinear: below(2, s) == 0,
        reverse_solution: below(2, s) == 0,
    }
}

fn random_case(test: u64, index: u64) -> Case {
    let mut s = case_seed(test, index);
    let s = &mut s;
    let scale = [8i64, 60, 2_000, 400_000, 60_000_000, 900_000_000][below(6, s) as usize];
    let grid = match below(4, s) {
        0 => 1,
        1 => (scale / 4).max(1),
        2 => (scale / 16).max(1),
        _ => (scale / 64).max(1),
    };
    let mut all: Paths = Vec::new();
    let n_subj = 1 + below(4, s) as usize;
    let n_clip = below(4, s) as usize;
    for _ in 0..n_subj + n_clip {
        let p = polygon(s, scale, grid, &all);
        all.push(p);
    }
    let clip = all.split_off(n_subj);
    configure(format!("case {index} of test {test:#x} (scale {scale}, grid {grid})"), all, clip, s)
}

fn run_random(test: u64, cases: u64, name: &str) {
    let mut tally = Tally::default();
    for index in 0..cases {
        tally.check(&random_case(test, index));
    }
    tally.finish(name);
}

// The random cases, in four tests so that they run side by side.
#[test]
fn random_polygons_a() {
    run_random(0xA1, 20_000, "random_polygons_a");
}

#[test]
fn random_polygons_b() {
    run_random(0xB2, 20_000, "random_polygons_b");
}

#[test]
fn random_polygons_c() {
    run_random(0xC3, 20_000, "random_polygons_c");
}

#[test]
fn random_polygons_d() {
    run_random(0xD4, 20_000, "random_polygons_d");
}

/// Every combination of clip type, fill types and the three options, on the same few inputs.
#[test]
fn every_configuration() {
    let mut tally = Tally::default();
    for index in 0..24u64 {
        let base = random_case(0xE5, index);
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
                        tally.check(&c);
                    }
                }
            }
        }
    }
    tally.finish("every_configuration");
}

/// Combs with their teeth at one Y: from a handful to several hundred local minima with equal
/// keys in `ClipperBase::Reset`'s sort, from several paths, and as many intersections at one Y
/// in `FixupIntersectionOrder`'s where the combs cross.
#[test]
fn many_local_minima_at_one_y() {
    let mut tally = Tally::default();
    for index in 0..3_000u64 {
        let mut s = case_seed(0xF6, index);
        let s = &mut s;
        let scale = [40i64, 3_000, 500_000, 80_000_000][below(4, s) as usize];
        let base = r(scale, s);
        // The total number of teeth: around libc++'s thresholds (24, 128) and well past them.
        let total = match below(6, s) {
            0 => 6 + below(18, s) as i64,
            1 => 20 + below(10, s) as i64,
            2 => 24 + below(60, s) as i64,
            3 => 120 + below(20, s) as i64,
            4 => 130 + below(200, s) as i64,
            _ => 300 + below(400, s) as i64,
        };
        let n_paths = 1 + below(5, s) as i64;
        let mut all = Paths::new();
        let mut left = total;
        for k in 0..n_paths {
            let teeth = if k == n_paths - 1 { left.max(1) } else { (1 + below(left.max(2) as u64 - 1, s) as i64).min(left.max(1)) };
            left -= teeth;
            let w = (2 + below((scale / 20).max(1) as u64, s) as i64).max(2);
            let h = 1 + below(scale as u64, s) as i64;
            // Most combs share the base Y; some sit one unit or one tooth height off it.
            let y = match below(6, s) {
                0 => base + 1,
                1 => base - h,
                _ => base,
            };
            all.push(comb(r(scale, s), y, teeth, w, h, below(3, s) == 0, below(2, s) == 0, s));
            if left <= 0 {
                break;
            }
        }
        if below(3, s) == 0 {
            let extra = polygon(s, scale, 1, &all);
            all.push(extra);
        }
        // Shuffle which path is added first, then split into subjects and clips.
        for i in (1..all.len()).rev() {
            let j = below(i as u64 + 1, s) as usize;
            all.swap(i, j);
        }
        let n_subj = 1 + below(all.len() as u64, s) as usize;
        let clip = all.split_off(n_subj);
        tally.check(&configure(format!("comb case {index} (scale {scale}, {total} teeth)"), all, clip, s));
    }
    tally.finish("many_local_minima_at_one_y");
}

/// Inputs with nothing to clip.
#[test]
fn degenerate_inputs() {
    let mut tally = Tally::default();
    let p = |v: &[(i32, i32)]| -> Path { v.iter().map(|q| IntPoint::new(q.0, q.1)).collect() };
    let inputs: Vec<(Paths, Paths)> = vec![
        (vec![], vec![]),
        (vec![p(&[(0, 0), (5, 5)])], vec![]),
        (vec![p(&[(0, 3), (5, 3), (9, 3)])], vec![p(&[(0, 0), (10, 0), (10, 10), (0, 10)])]),
        (vec![p(&[(0, 0), (10, 0), (10, 10), (0, 10)])], vec![]),
        (vec![], vec![p(&[(0, 0), (10, 0), (10, 10), (0, 10)])]),
        (vec![p(&[(0, 0), (0, 0), (0, 0)])], vec![p(&[(1, 1), (1, 1), (2, 2)])]),
    ];
    for (i, (subj, clip)) in inputs.into_iter().enumerate() {
        for clip_type in 0..4 {
            for fill in 0..4 {
                for flags in 0..8 {
                    tally.check(&Case {
                        name: format!("degenerate input {i}"),
                        subj: subj.clone(),
                        clip: clip.clone(),
                        clip_type,
                        subj_fill: fill,
                        clip_fill: (fill + flags) % 4,
                        strictly_simple: flags & 4 != 0,
                        preserve_collinear: flags & 2 != 0,
                        reverse_solution: flags & 1 != 0,
                    });
                }
            }
        }
    }
    assert!(tally.failures.is_empty(), "{} degenerate cases differ; the first:\n{}", tally.failures.len(), tally.failures[0]);
}
