//! The two arms of the gate `CLIPPER2_RUST`, compared at the level of the engine's wrappers:
//! every public function of `clipper2_utils.rs`, `clipper2_z.rs` and the two of
//! `clipper_utils.rs` that used the `clipper2` crate, through its `*_impl(rust, ...)` twin,
//! on generated inputs. `rust = true` is `crate::clipper2`, the Rust port of the Clipper2
//! the engine pins (1.5.2); `rust = false` is the C++ library the function called before.
//!
//! Which C++ library that is decides what is expected:
//!  - `clipper2_z.rs` (`offset_z`, `intersect_open_z`) and, with `ZSMOOTH_FAITHFUL` on as it
//!    is by default, `intersection_pl_2` / `diff_pl_2` of `clipper2_utils.rs` call
//!    `clipper2-z-sys`, which is Clipper2 1.5.2 with `USINGZ`: the two arms have to be equal,
//!    always.
//!  - `intersection_pl_2` / `diff_pl_2` of `clipper_utils.rs` call the `clipper2` crate
//!    (1.5.4), which hands back the closed paths of an open-subject clip, of which there are
//!    none: the two arms have to be equal, always.
//!  - The others of `clipper2_utils.rs` call `clipper2c-sys`, which is Clipper2 1.5.4, and
//!    the offset functions call it in another sequence than Clipper2Utils.cpp does. See
//!    `version_154` below for how every difference between the arms is accounted for.
//!
//! `rust_arm_is_the_pin` holds the Rust arm of the functions of Clipper2Utils.cpp equal to
//! the calls that file makes, made in C++ on the pin's Clipper2 (`cz2_ref_xy_utils`).

#[path = "clipper2_engine_parity/mod.rs"]
mod engine;

use engine::{below, between, case_seed, chance, r};
use slicer::clipper2 as c2;
use slicer::clipper2_utils as cu;
use slicer::geometry::{ExPolygon, ExPolygons, Point, Polygon, Polyline};
use std::collections::BTreeMap;
use std::io::Write as _;

fn polygon(p: &engine::Path) -> Polygon {
    Polygon::from_points(p.iter().map(|q| Point::new(q[0], q[1])).collect())
}

fn polyline(p: &engine::Path) -> Polyline {
    Polyline::from_points(p.iter().map(|q| Point::new(q[0], q[1])).collect())
}

type Flat = Vec<Vec<(i64, i64)>>;

fn flat_polylines(pls: &[Polyline]) -> Flat {
    pls.iter().map(|pl| pl.points.iter().map(|p| (p.x, p.y)).collect()).collect()
}

fn flat_polygons(pgs: &[Polygon]) -> Flat {
    pgs.iter().map(|pg| pg.points.iter().map(|p| (p.x, p.y)).collect()).collect()
}

/// Every expolygon as its contour and then its holes, with an empty path between two
/// expolygons: equal lists are equal expolygons in the same order with the same holes.
fn flat_expolygons(eps: &[ExPolygon]) -> Flat {
    let mut out = Flat::new();
    for ep in eps {
        out.push(ep.contour.points.iter().map(|p| (p.x, p.y)).collect());
        for h in &ep.holes {
            out.push(h.points.iter().map(|p| (p.x, p.y)).collect());
        }
        out.push(Vec::new());
    }
    out
}

fn report(text: &str) {
    eprint!("{text}");
    if let Ok(path) = std::env::var("CLIPPER2_PARITY_SUMMARY") {
        if let Ok(mut f) = std::fs::OpenOptions::new().create(true).append(true).open(path) {
            let _ = f.write_all(text.as_bytes());
        }
    }
}

/// The polygons of a case of the engine tests, as the slicer would have them: at most
/// `i32` coordinates (the variable offset rounds through `int32_t`).
fn case_polygons(index: u64, s: &mut u64) -> Vec<Polygon> {
    let e = match index % 5 {
        0 | 1 => engine::gen_random(index, s),
        2 => engine::gen_nested(index, s),
        3 => engine::gen_touching(index, s),
        _ => engine::gen_coincident(index, s),
    };
    e.subj.iter().chain(e.clip.iter()).filter(|p| !p.is_empty()).map(polygon).collect()
}

/// Shapes as slices of a print are: a few rings of 4 to 40 points some millimetres across
/// (1 mm = 1e6), outers counter-clockwise with clockwise holes inside, some of them
/// overlapping their neighbours.
fn slicer_like(s: &mut u64) -> ExPolygons {
    let ring = |cx: f64, cy: f64, rad: f64, n: usize, ccw: bool, s: &mut u64| -> Polygon {
        let mut pts: Vec<Point> = (0..n)
            .map(|i| {
                let a = i as f64 / n as f64 * std::f64::consts::TAU;
                let rr = rad * (0.75 + 0.25 * (below(1000, s) as f64 / 1000.0));
                Point::new((cx + rr * a.cos()).round() as i64, (cy + rr * a.sin()).round() as i64)
            })
            .collect();
        if !ccw {
            pts.reverse();
        }
        Polygon::from_points(pts)
    };
    let mut out = ExPolygons::new();
    for _ in 0..between(1, 3, s) {
        let (cx, cy) = (r(40_000_000, s) as f64, r(40_000_000, s) as f64);
        let rad = between(2_000_000, 30_000_000, s) as f64;
        let contour = ring(cx, cy, rad, between(4, 40, s) as usize, true, s);
        let mut holes = Vec::new();
        for _ in 0..between(0, 2, s) {
            let hr = rad * (0.1 + 0.2 * (below(1000, s) as f64 / 1000.0));
            let (hx, hy) = (cx + (r(1000, s) as f64 / 1000.0) * rad * 0.4, cy + (r(1000, s) as f64 / 1000.0) * rad * 0.4);
            holes.push(ring(hx, hy, hr, between(4, 20, s) as usize, false, s));
        }
        out.push(ExPolygon::with_holes(contour, holes));
    }
    out
}

/// Expolygons for the offset functions: slicer-like ones, the union of generated polygons
/// (valid, with holes), or every generated polygon as a contour of its own (not valid:
/// overlapping, self-intersecting, of either orientation).
fn case_expolygons(index: u64, s: &mut u64) -> ExPolygons {
    match index % 4 {
        0 | 1 => slicer_like(s),
        2 => cu::union_ex_2_impl(true, &case_polygons(index / 4, s)),
        _ => case_polygons(index / 4, s).into_iter().map(ExPolygon::new).collect(),
    }
}

fn extent(eps: &ExPolygons) -> f64 {
    let (mut lo, mut hi) = (i64::MAX, i64::MIN);
    for ep in eps {
        for p in ep.contour.points.iter().chain(ep.holes.iter().flat_map(|h| h.points.iter())) {
            lo = lo.min(p.x).min(p.y);
            hi = hi.max(p.x).max(p.y);
        }
    }
    if lo > hi {
        1.0
    } else {
        (hi as f64 - lo as f64).max(1.0)
    }
}

fn pick_delta(size: f64, s: &mut u64) -> f64 {
    let f = 0.5 + below(1000, s) as f64 / 1000.0;
    let d = [0.0, 0.4, 1.0, 7.5, size * 0.002 * f, size * 0.01 * f, size * 0.01 * f, size * 0.05 * f, size * 0.05 * f, size * 0.2 * f, (size * 0.03 * f).round()][below(11, s) as usize];
    if chance(1, 2, s) {
        -d
    } else {
        d
    }
}

// -------------------------------------------------------------------------------------------
// Arms that have to be equal, always
// -------------------------------------------------------------------------------------------

/// `intersection_pl_2` and `diff_pl_2` of `clipper2_utils.rs` (Clipper2Utils.cpp:119-141):
/// open polylines against closed polygons. The C++ arm is `cz2_pl_open` of `clipper2-z-sys`
/// (Clipper2 1.5.2) while `ZSMOOTH_FAITHFUL` is on, which it is unless the environment of
/// the test says otherwise.
#[test]
fn pl_open_2() {
    if std::env::var("ZSMOOTH_FAITHFUL").is_ok_and(|v| v == "0") {
        eprintln!("pl_open_2: skipped, ZSMOOTH_FAITHFUL=0 puts Clipper2 1.5.4 in the C++ arm");
        return;
    }
    const N: u64 = 30_000;
    let (mut differ, mut nonempty, mut pieces) = (0u64, 0u64, 0u64);
    let mut first = String::new();
    for index in 0..N {
        let mut s = case_seed(401, index);
        let e = if index % 3 == 2 { engine::gen_random(index, &mut s) } else { engine::gen_open(index, &mut s) };
        let mut subject: Vec<Polyline> = e.open.iter().chain(if index % 3 == 2 { e.subj.iter() } else { [].iter() }).map(polyline).collect();
        if index % 7 == 0 {
            subject.push(Polyline::from_points(Vec::new()));
        }
        let clip: Vec<Polygon> = e.clip.iter().map(polygon).collect();
        for diff in [false, true] {
            let run = |rust: bool| if diff { cu::diff_pl_2_impl(rust, &subject, &clip) } else { cu::intersection_pl_2_impl(rust, &subject, &clip) };
            let (a, b) = (flat_polylines(&run(true)), flat_polylines(&run(false)));
            nonempty += !b.is_empty() as u64;
            pieces += b.len() as u64;
            if a != b {
                differ += 1;
                if first.is_empty() {
                    first = format!("{} diff {diff}:\n  Rust {a:?}\n  C++  {b:?}", e.name);
                }
            }
        }
    }
    report(&format!("pl_open_2: {} calls of intersection_pl_2 and diff_pl_2, {nonempty} non-empty results, {pieces} polylines; {differ} differ\n", 2 * N));
    assert!(differ == 0, "{differ} calls differ. The first: {first}");
    assert!(pieces > N);
}

/// `offset_z` and `intersect_open_z` of `clipper2_z.rs` (RegionExpansion.cpp:108-136 and
/// 301-322), as `wave_seeds` uses them: every contour of the source offset outwards (holes
/// inwards) with the index of its expolygon as z, the results closed again and cut as open
/// paths by the boundary, whose contours have z of their own.
#[test]
fn clipper2_z_wrappers() {
    use slicer::clipper2_z::{intersect_open_z_impl, offset_z_impl};
    use slicer::clipper_z_utils::{ZPath, ZPaths};
    const N: u64 = 20_000;
    let (mut differ, mut offsets, mut offset_paths, mut clips, mut segments, mut intersections) = (0u64, 0u64, 0u64, 0u64, 0u64, 0u64);
    let mut first = String::new();
    for index in 0..N {
        let mut s = case_seed(402, index);
        let src = case_expolygons(index, &mut s);
        let boundary = case_expolygons(index + 1, &mut s);
        let size = extent(&src);
        let expansion = [1.0, 10.0, 0.6, size * 0.001, size * 0.01, size * 0.1, 0.0, -size * 0.01][below(8, &mut s) as usize];
        let mut idx = 1i64;
        let mut zboundary = ZPaths::new();
        for ep in &boundary {
            for pg in std::iter::once(&ep.contour).chain(ep.holes.iter()) {
                // one case in eight gives every vertex a z of its own: more than two
                // sources at an intersection, which the callback leaves alone
                let own = index % 8 == 7;
                zboundary.push(pg.points.iter().enumerate().map(|(i, p)| (p.x, p.y, if own { idx * 1000 + i as i64 } else { idx })).collect());
            }
            idx += 1;
        }
        let mut zsrc = ZPaths::new();
        let mut same = true;
        for ep in &src {
            for (icontour, pg) in std::iter::once(&ep.contour).chain(ep.holes.iter()).enumerate() {
                let contour: ZPath = pg.points.iter().map(|p| (p.x, p.y, idx)).collect();
                let delta = if icontour == 0 { expansion } else { -expansion };
                let (a, b) = (offset_z_impl(true, &contour, delta), offset_z_impl(false, &contour, delta));
                offsets += 1;
                offset_paths += b.len() as u64;
                if a != b {
                    same = false;
                    if first.is_empty() {
                        first = format!("offset_z #{index} delta {delta:?} {contour:?}:\n  Rust {a:?}\n  C++  {b:?}");
                    }
                }
                // to_zpaths64<true>: the path closed by its first point, with the index
                for mut path in b {
                    if let Some(f) = path.first().copied() {
                        path.push(f);
                    }
                    zsrc.push(path);
                }
            }
            idx += 1;
        }
        let (a, b) = (intersect_open_z_impl(true, &zsrc, &zboundary), intersect_open_z_impl(false, &zsrc, &zboundary));
        clips += 1;
        segments += b.segments.len() as u64;
        intersections += b.intersections.len() as u64;
        if a.segments != b.segments || a.num_closed != b.num_closed || a.intersections != b.intersections {
            same = false;
            if first.is_empty() {
                first = format!(
                    "intersect_open_z #{index}: {} segments ({} closed), {} intersections in Rust; {} ({}), {} in C++\n  src {zsrc:?}\n  boundary {zboundary:?}",
                    a.segments.len(),
                    a.num_closed,
                    a.intersections.len(),
                    b.segments.len(),
                    b.num_closed,
                    b.intersections.len()
                );
            }
        }
        differ += !same as u64;
    }
    report(&format!(
        "clipper2_z_wrappers: {N} cases: {offsets} calls of offset_z ({offset_paths} paths), {clips} of intersect_open_z ({segments} segments, {intersections} recorded intersections); {differ} cases differ\n"
    ));
    assert!(differ == 0, "{differ} cases differ. The first: {first}");
    assert!(segments > N && intersections > N);
}

/// `intersection_pl_2` and `diff_pl_2` of `clipper_utils.rs`, which went through the
/// `clipper2` crate: its coordinates are `Centi` (a value goes in and out through
/// `(v / 100.0) * 100.0` and a rounding) and its `boolean_operation` returns the closed
/// paths of the solution only, so what comes back is what the early returns of the two
/// functions give and, past them, nothing.
#[test]
fn clipper2_crate_wrappers() {
    use slicer::clipper_utils::{diff_pl_2_impl, intersection_pl_2_impl};
    const N: u64 = 12_000;
    let (mut differ, mut nonempty) = (0u64, 0u64);
    let mut first = String::new();
    for index in 0..N {
        let mut s = case_seed(403, index);
        let e = engine::gen_open(index, &mut s);
        let subject: Vec<Polyline> = e.open.iter().map(polyline).collect();
        let clip: ExPolygons = if index % 5 == 4 { ExPolygons::new() } else { cu::union_ex_2_impl(true, &e.clip.iter().map(polygon).collect::<Vec<_>>()) };
        for diff in [false, true] {
            let run = |rust: bool| if diff { diff_pl_2_impl(rust, &subject, &clip) } else { intersection_pl_2_impl(rust, &subject, &clip) };
            let (a, b) = (flat_polylines(&run(true)), flat_polylines(&run(false)));
            nonempty += !b.is_empty() as u64;
            if a != b {
                differ += 1;
                if first.is_empty() {
                    first = format!("{} diff {diff}:\n  Rust {a:?}\n  C++  {b:?}", e.name);
                }
            }
        }
    }
    report(&format!("clipper2_crate_wrappers: {} calls of intersection_pl_2 and diff_pl_2 of clipper_utils.rs, {nonempty} non-empty results; {differ} differ\n", 2 * N));
    assert!(differ == 0, "{differ} calls differ. The first: {first}");
}

// -------------------------------------------------------------------------------------------
// The Rust arm against the calls of Clipper2Utils.cpp on the pin
// -------------------------------------------------------------------------------------------

/// `cz2_ref_xy_utils` (clipper2_ref_shim.cpp): what the function of Clipper2Utils.cpp
/// calls, on the Clipper2 the engine pins. For the functions that return a tree, the
/// expolygons `PolyTreeToExPolygons` (Clipper2Utils.cpp:46-80) makes of it.
fn pin_utils(op: i32, subject: &[Vec<(i64, i64)>], clip: &[Vec<(i64, i64)>], delta1: f64, delta2: f64) -> Flat {
    let flatten = |paths: &[Vec<(i64, i64)>]| -> (Vec<i64>, Vec<i32>) {
        let mut xyz = Vec::new();
        let mut lens = Vec::new();
        for p in paths {
            lens.push(p.len() as i32);
            for q in p {
                xyz.extend_from_slice(&[q.0, q.1, 0]);
            }
        }
        (xyz, lens)
    };
    let (sx, sl) = flatten(subject);
    let (cx, cl) = flatten(clip);
    let res = unsafe { clipper2_z_sys::cz2_ref_xy_utils(op, sx.as_ptr(), sl.as_ptr(), sl.len() as i32, cx.as_ptr(), cl.as_ptr(), cl.len() as i32, delta1, delta2) };
    let n_paths = (res.num_closed + res.num_open) as usize;
    let lens: Vec<i32> = if n_paths == 0 { Vec::new() } else { unsafe { std::slice::from_raw_parts(res.path_lens, n_paths) }.to_vec() };
    let coords: Vec<i64> = if res.total_points == 0 { Vec::new() } else { unsafe { std::slice::from_raw_parts(res.coords, 3 * res.total_points as usize) }.to_vec() };
    let parents: Vec<i32> = if op > 2 || n_paths == 0 { Vec::new() } else { unsafe { std::slice::from_raw_parts(res.tree_parent, n_paths) }.to_vec() };
    unsafe { clipper2_z_sys::cz2_ref_xy_free(res) };
    let mut paths = Flat::new();
    let mut at = 0usize;
    for l in &lens {
        paths.push((0..*l as usize).map(|v| (coords[at + 3 * v], coords[at + 3 * v + 1])).collect());
        at += 3 * *l as usize;
    }
    if op > 2 {
        return paths;
    }
    // PolyTreeToExPolygons on the nodes: `childs[n]` in the order of the tree.
    let mut childs: Vec<Vec<usize>> = vec![Vec::new(); n_paths];
    let mut roots = Vec::new();
    for (i, p) in parents.iter().enumerate() {
        if *p < 0 {
            roots.push(i);
        } else {
            childs[*p as usize].push(i);
        }
    }
    fn rec(node: usize, childs: &[Vec<usize>], paths: &Flat, out: &mut Vec<Vec<usize>>) {
        let at = out.len();
        out.push(vec![node]);
        for &hole in &childs[node] {
            out[at].push(hole);
            for &inner in &childs[hole] {
                rec(inner, childs, paths, out);
            }
        }
    }
    let mut expolygons: Vec<Vec<usize>> = Vec::new();
    for root in roots {
        rec(root, &childs, &paths, &mut expolygons);
    }
    let mut out = Flat::new();
    for ep in expolygons {
        for node in ep {
            out.push(paths[node].clone());
        }
        out.push(Vec::new());
    }
    out
}

fn pairs_of_expolygons(eps: &ExPolygons) -> Flat {
    eps.iter().flat_map(|ep| std::iter::once(&ep.contour).chain(ep.holes.iter())).map(|pg| pg.points.iter().map(|p| (p.x, p.y)).collect()).collect()
}

/// The Rust arm of `union_ex_2`, `union_ex_2_expolygons`, `offset_ex_2`, `offset2_ex_2`,
/// `intersection_pl_2` and `diff_pl_2` against the calls Clipper2Utils.cpp makes, made in
/// C++ on the Clipper2 the engine pins (`cz2_ref_xy_utils`): equal, always. This is what the
/// engine's C++ gives for the same input.
#[test]
fn rust_arm_is_the_pin() {
    const N: u64 = 16_000;
    let mut differ = [0u64; 6];
    let mut nonempty = [0u64; 6];
    let mut first = String::new();
    let mut check = |slot: usize, what: String, rust: Flat, pin: Flat, first: &mut String| {
        nonempty[slot] += !pin.is_empty() as u64;
        if rust != pin {
            differ[slot] += 1;
            if first.is_empty() {
                *first = format!("{what}:\n  Rust arm {rust:?}\n  the pin  {pin:?}");
            }
        }
    };
    for index in 0..N {
        let mut s = case_seed(404, index);
        let polygons = case_polygons(index, &mut s);
        check(0, format!("union_ex_2 #{index}"), flat_expolygons(&cu::union_ex_2_impl(true, &polygons)), pin_utils(0, &flat_polygons(&polygons), &[], 0.0, 0.0), &mut first);
        let eps = case_expolygons(index, &mut s);
        let paths = pairs_of_expolygons(&eps);
        check(1, format!("union_ex_2_expolygons #{index}"), flat_expolygons(&cu::union_ex_2_expolygons_impl(true, &eps)), pin_utils(0, &paths, &[], 0.0, 0.0), &mut first);
        let size = extent(&eps);
        let delta1 = pick_delta(size, &mut s);
        check(2, format!("offset_ex_2 #{index} delta {delta1:?}"), flat_expolygons(&cu::offset_ex_2_impl(true, &eps, delta1)), pin_utils(1, &paths, &[], delta1, 0.0), &mut first);
        let delta2 = if chance(1, 4, &mut s) { pick_delta(size, &mut s) } else { -delta1 * (0.5 + below(1000, &mut s) as f64 / 1000.0) };
        check(
            3,
            format!("offset2_ex_2 #{index} deltas {delta1:?} {delta2:?}"),
            flat_expolygons(&cu::offset2_ex_2_impl(true, &eps, delta1, delta2)),
            pin_utils(2, &paths, &[], delta1, delta2),
            &mut first,
        );
        let e = engine::gen_open(index, &mut s);
        let subject: Vec<Polyline> = e.open.iter().map(polyline).collect();
        let clip: Vec<Polygon> = e.clip.iter().map(polygon).collect();
        let (sp, cp) = (flat_polylines(&subject), flat_polygons(&clip));
        check(4, format!("intersection_pl_2 #{index}"), flat_polylines(&cu::intersection_pl_2_impl(true, &subject, &clip)), pin_utils(3, &sp, &cp, 0.0, 0.0), &mut first);
        check(5, format!("diff_pl_2 #{index}"), flat_polylines(&cu::diff_pl_2_impl(true, &subject, &clip)), pin_utils(4, &sp, &cp, 0.0, 0.0), &mut first);
    }
    report(&format!(
        "rust_arm_is_the_pin: {N} cases each of union_ex_2, union_ex_2_expolygons, offset_ex_2, offset2_ex_2, intersection_pl_2, diff_pl_2; non-empty {nonempty:?}; differ {differ:?}\n"
    ));
    assert!(differ.iter().all(|d| *d == 0), "the Rust arm differs from the calls of Clipper2Utils.cpp on the pin: {differ:?}. The first: {first}");
    assert!(nonempty.iter().all(|n| *n > N / 2));
}

// -------------------------------------------------------------------------------------------
// The arms that meet Clipper2 1.5.4
// -------------------------------------------------------------------------------------------

/// What one comparison of the two arms came to.
#[derive(Default)]
struct Tally {
    cases: u64,
    nonempty: u64,
    /// Rust arm != C++ arm.
    differ: u64,
    /// By what accounts for the difference.
    kinds: BTreeMap<String, u64>,
    unexplained: u64,
    first_unexplained: String,
}

impl Tally {
    fn line(&self, name: &str) -> String {
        let kinds: Vec<String> = self.kinds.iter().map(|(k, n)| format!("{k}: {n}")).collect();
        format!(
            "{name}: {} cases, {} non-empty; the arms differ in {} [{}]; unexplained {}\n",
            self.cases,
            self.nonempty,
            self.differ,
            kinds.join("; "),
            self.unexplained
        )
    }
}

fn tree_to_expolygons(tree: &c2::PolyTree64) -> ExPolygons {
    fn rec(node: &c2::PolyPath64, out: &mut ExPolygons) {
        let pts = |n: &c2::PolyPath64| Polygon::from_points(n.polygon().iter().map(|p| Point::new(p.x, p.y)).collect());
        let at = out.len();
        out.push(ExPolygon::new(pts(node)));
        for i in 0..node.count() {
            let hole = pts(node.child(i));
            out[at].holes.push(hole);
            for j in 0..node.child(i).count() {
                rec(node.child(i).child(j), out);
            }
        }
    }
    let mut out = ExPolygons::new();
    for i in 0..tree.count() {
        rec(tree.child(i), &mut out);
    }
    out
}

fn to_paths(eps: &ExPolygons) -> c2::Paths64 {
    let mut out = c2::Paths64::new();
    for ep in eps {
        for pg in std::iter::once(&ep.contour).chain(ep.holes.iter()) {
            out.push(pg.points.iter().map(|p| c2::Point64::new(p.x, p.y)).collect());
        }
    }
    out
}

/// The sequence of calls the C++ arm of `offset_ex_2` and `offset2_ex_2` makes
/// (`clipper_offset_execute`, `simplify_poly_tree_paths`, `offset_paths_to_expolygons` of
/// clipper2_utils.rs), on the Rust port. It is not the sequence of Clipper2Utils.cpp:
///  - the offsetter fills paths, not a tree, and a second clipper makes the tree from them
///    by a NonZero union (`ClipperOffset::Execute(delta, tree)` is not in the C API);
///  - between the two offsets the paths are simplified by `clipper_paths64_simplify(paths,
///    epsilon, 0)`, whose third argument the C API calls `is_open_paths` and hands to
///    `SimplifyPaths` as `isClosedPath`: the closed contours are simplified as OPEN paths
///    (the two ends kept), where Clipper2Utils.cpp simplifies them as closed ones.
fn ffi_sequence_on_rust(eps: &ExPolygons, delta1: f64, delta2: Option<f64>) -> ExPolygons {
    let offset = |paths: &c2::Paths64, delta: f64| -> c2::Paths64 {
        let mut co = c2::ClipperOffset::new(2.0, 0.0, false, false);
        co.add_paths(paths, c2::JoinType::Round, c2::EndType::Polygon);
        let mut out = c2::Paths64::new();
        co.execute(delta, &mut out);
        out
    };
    let mut paths = offset(&to_paths(eps), delta1);
    if let Some(delta2) = delta2 {
        paths = offset(&c2::simplify_paths(&paths, 10.0, false), delta2);
    }
    let mut c = c2::Clipper64::new();
    c.add_subject(&paths);
    let mut tree = c2::PolyTree64::new();
    c.execute_tree(c2::ClipType::Union, c2::FillRule::NonZero, &mut tree);
    tree_to_expolygons(&tree)
}

/// Runs `f` with the engine's Clipper2 behaving as version 1.5.4 in the respects `bits`
/// names (`c2::V154_*`), and as the pin again afterwards.
fn with_154<T>(bits: u32, f: impl FnOnce() -> T) -> T {
    let before = c2::version_154_probe(bits);
    let out = f();
    c2::version_154_probe(before);
    out
}

/// Accounts for one difference between the Rust arm (`rust`) and the C++ arm (`cpp`) of a
/// function whose C++ arm is Clipper2 1.5.4. `run` runs what the C++ arm runs — the same
/// sequence of calls — on the Rust port. A difference is explained when
///  1. `run` under the pin's behaviour gives what the Rust arm gave, or the difference to
///     it is the sequence of calls alone (`sequence`, for the offset functions), and
///  2. `run` with every change of 1.5.4 switched on gives what the C++ arm gave, exactly.
/// The changes that, switched on alone, move the result away from the pin's are named.
fn account(t: &mut Tally, what: &str, rust: &Flat, cpp: &Flat, sequence: bool, run: &dyn Fn() -> Flat) {
    t.cases += 1;
    t.nonempty += !cpp.is_empty() as u64;
    if rust == cpp {
        return;
    }
    t.differ += 1;
    let pin = run();
    let as_154 = with_154(c2::V154_ALL, run);
    if as_154 != *cpp || (!sequence && pin != *rust) {
        t.unexplained += 1;
        if t.first_unexplained.is_empty() {
            t.first_unexplained = format!("{what}:\n  Rust arm {rust:?}\n  C++ arm  {cpp:?}\n  the C++ arm's calls on the port as the pin {pin:?}\n  and as 1.5.4 {as_154:?}");
        }
        return;
    }
    let mut kind = Vec::new();
    if sequence && pin != *rust {
        kind.push("the C++ arm's sequence of calls".to_string());
    }
    if pin != as_154 {
        let mut alone = Vec::new();
        for (bit, name) in c2::V154_NAMES {
            if with_154(bit, run) != pin {
                alone.push(name);
            }
        }
        kind.push(if alone.is_empty() { "1.5.4: several changes together".to_string() } else { format!("1.5.4: {}", alone.join(" + ")) });
    }
    *t.kinds.entry(kind.join(", and ")).or_default() += 1;
}

/// The functions of `clipper2_utils.rs` whose C++ arm is Clipper2 1.5.4 (`clipper2c-sys`):
/// `union_ex_2`, `union_ex_2_expolygons`, `offset_ex_2`, `offset2_ex_2` and the four
/// `variable_offset_*`. The Rust arm follows the pin (1.5.2), so the arms may differ, and
/// every difference has to be accounted for by `account`: the port, made to behave as 1.5.4
/// (`c2::version_154_probe`, which switches on the changes between the two versions that
/// are listed in `clipper_engine.rs` and `clipper_offset.rs`) and to make the calls the C++
/// arm makes, has to give the C++ arm's result exactly. A difference that is not reproduced
/// that way fails the test.
#[test]
fn version_154() {
    let n: u64 = 16_000 * std::env::var("CLIPPER2_PARITY_SCALE").ok().and_then(|v| v.parse::<u64>().ok()).unwrap_or(1);
    let mut tallies: Vec<(&str, Tally)> =
        ["union_ex_2", "union_ex_2_expolygons", "offset_ex_2", "offset2_ex_2", "variable_offset_inner", "variable_offset_outer", "variable_offset_inner_ex", "variable_offset_outer_ex"]
            .into_iter()
            .map(|n| (n, Tally::default()))
            .collect();
    let mut panics = 0u64;
    for index in 0..n {
        let mut s = case_seed(404, index);

        // union_ex_2
        let polygons = case_polygons(index, &mut s);
        let (a, b) = (flat_expolygons(&cu::union_ex_2_impl(true, &polygons)), flat_expolygons(&cu::union_ex_2_impl(false, &polygons)));
        account(&mut tallies[0].1, &format!("union_ex_2 #{index}"), &a, &b, false, &|| flat_expolygons(&cu::union_ex_2_impl(true, &polygons)));

        // union_ex_2_expolygons
        let eps = case_expolygons(index, &mut s);
        let (a, b) = (flat_expolygons(&cu::union_ex_2_expolygons_impl(true, &eps)), flat_expolygons(&cu::union_ex_2_expolygons_impl(false, &eps)));
        account(&mut tallies[1].1, &format!("union_ex_2_expolygons #{index}"), &a, &b, false, &|| flat_expolygons(&cu::union_ex_2_expolygons_impl(true, &eps)));

        // offset_ex_2
        let size = extent(&eps);
        let delta1 = pick_delta(size, &mut s);
        let (a, b) = (flat_expolygons(&cu::offset_ex_2_impl(true, &eps, delta1)), flat_expolygons(&cu::offset_ex_2_impl(false, &eps, delta1)));
        account(&mut tallies[2].1, &format!("offset_ex_2 #{index} delta {delta1:?}"), &a, &b, true, &|| flat_expolygons(&ffi_sequence_on_rust(&eps, delta1, None)));

        // offset2_ex_2
        let delta2 = if chance(1, 4, &mut s) { pick_delta(size, &mut s) } else { -delta1 * (0.5 + below(1000, &mut s) as f64 / 1000.0) };
        let (a, b) =
            (flat_expolygons(&cu::offset2_ex_2_impl(true, &eps, delta1, delta2)), flat_expolygons(&cu::offset2_ex_2_impl(false, &eps, delta1, delta2)));
        account(&mut tallies[3].1, &format!("offset2_ex_2 #{index} deltas {delta1:?} {delta2:?}"), &a, &b, true, &|| {
            flat_expolygons(&ffi_sequence_on_rust(&eps, delta1, Some(delta2)))
        });

        // variable_offset_*: a slicer-like expolygon, one delta per point, all of one sign
        let ep = slicer_like(&mut s).remove(0);
        let amount = between(20_000, 600_000, &mut s) as f32;
        let mut deltas = |sign: f32, s: &mut u64| -> Vec<Vec<f32>> {
            std::iter::once(&ep.contour).chain(ep.holes.iter()).map(|pg| pg.points.iter().map(|_| sign * amount * (0.2 + 0.8 * below(1000, s) as f32 / 1000.0)).collect()).collect()
        };
        let miter_limit = [2.0, 3.0, 1.5][below(3, &mut s) as usize];
        let (inner, outer) = (deltas(-1.0, &mut s), deltas(1.0, &mut s));
        // The functions carry the debug assertions of ClipperUtils.cpp; an input that
        // trips one (in either arm) is left out.
        type Run<'a> = Box<dyn Fn(bool) -> Flat + 'a>;
        let runs: [(usize, Run); 4] = [
            (4, Box::new(|rust| flat_polygons(&cu::variable_offset_inner_impl(rust, &ep, &inner, miter_limit)))),
            (5, Box::new(|rust| flat_polygons(&cu::variable_offset_outer_impl(rust, &ep, &outer, miter_limit)))),
            (6, Box::new(|rust| flat_expolygons(&cu::variable_offset_inner_ex_impl(rust, &ep, &inner, miter_limit)))),
            (7, Box::new(|rust| flat_expolygons(&cu::variable_offset_outer_ex_impl(rust, &ep, &outer, miter_limit)))),
        ];
        for (slot, run) in &runs {
            let both = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| (run(true), run(false))));
            match both {
                Ok((a, b)) => {
                    let what = format!("{} #{index}", tallies[*slot].0);
                    account(&mut tallies[*slot].1, &what, &a, &b, false, &|| run(true))
                }
                Err(_) => panics += 1,
            }
        }
    }
    let mut text = String::new();
    for (name, t) in &tallies {
        text.push_str(&t.line(name));
    }
    text.push_str(&format!("variable_offset_*: {panics} calls left out for a debug assertion of the function\n"));
    report(&text);
    for (name, t) in &tallies {
        assert!(t.unexplained == 0, "{name}: {} differences between the arms are not accounted for. The first:\n{}", t.unexplained, t.first_unexplained);
        assert!(t.nonempty > t.cases / 3, "{name}: {} of {} results are empty", t.cases - t.nonempty, t.cases);
    }
    assert!(panics < n, "the variable offsets were hardly run: {panics} of {} calls tripped an assertion", 4 * n);
}

/// Sensitivity of `version_154`: with one change of 1.5.4 left out of the probe, the port
/// no longer gives what the C++ arm of `union_ex_2` gives; and as the pin (no change
/// switched on) it differs from it wherever the arms differ.
#[test]
fn version_154_mutations() {
    const N: u64 = 16_000;
    let engine_changes = [c2::V154_PATH2_CONTAINS_PATH1, c2::V154_IS_VALID_AEL_ORDER, c2::V154_FIX_SELF_INTERSECTS, c2::V154_ADJUST_CURR_X];
    let mut not_reproduced = [0u64; 5];
    let mut all = 0u64;
    for index in 0..N {
        let mut s = case_seed(404, index);
        let polygons = case_polygons(index, &mut s);
        let cpp = flat_expolygons(&cu::union_ex_2_impl(false, &polygons));
        let run = || flat_expolygons(&cu::union_ex_2_impl(true, &polygons));
        all += (with_154(c2::V154_ALL, run) != cpp) as u64;
        not_reproduced[4] += (run() != cpp) as u64;
        for (i, bit) in engine_changes.iter().enumerate() {
            not_reproduced[i] += (with_154(c2::V154_ALL & !bit, run) != cpp) as u64;
        }
    }
    report(&format!(
        "version_154_mutations: union_ex_2, {N} cases: not reproduced with every change on: {all}; without Path2ContainsPath1: {}, without IsValidAelOrder's CrossProductSign: {}, without FixSelfIntersects: {}, without AdjustCurrXAndCopyToSEL: {}; as the pin: {}\n",
        not_reproduced[0], not_reproduced[1], not_reproduced[2], not_reproduced[3], not_reproduced[4]
    ));
    assert_eq!(all, 0);
    assert!(not_reproduced.iter().all(|n| *n > 0), "a change of 1.5.4 left out of the probe went unnoticed: {not_reproduced:?}");
}
