//! The engine's Clipper2 wrappers against the C++ they stand for, on generated inputs:
//! every function of Clipper2Utils.cpp in `clipper2_utils.rs` and the two of `clipper2_z.rs`.
//! They run on `crate::clipper2`, the Rust port of the Clipper2 the engine pins (1.5.2); the
//! C++ side is `clipper2-z-sys`, built from the same sources:
//!  - `rust_arm_is_the_pin`: `union_ex_2`, `union_ex_2_expolygons`, `offset_ex_2`,
//!    `offset2_ex_2`, `intersection_pl_2`, `diff_pl_2` against the calls Clipper2Utils.cpp
//!    makes, made in C++ on the pin's Clipper2 (`cz2_ref_xy_utils`);
//!  - `pl_open_2`: `intersection_pl_2` / `diff_pl_2` against `cz2_pl_open` (the `USINGZ`
//!    build, the C++ these two ran on until R905);
//!  - `clipper2_z_wrappers`: `offset_z` / `intersect_open_z` against `cz2_offset_z` /
//!    `cz2_intersect_open_z`.
//! Equal, always. What Clipper2 1.5.4 does otherwise (the version the engine reached through
//! `clipper2c-sys` until R905) is recorded in docs/rust-engine/parity/not-ported.md.

#[path = "clipper2_engine_parity/mod.rs"]
mod engine;

use engine::{below, between, case_seed, chance, r};
use slicer::clipper2_utils as cu;
use slicer::geometry::{ExPolygon, ExPolygons, Point, Polygon, Polyline};
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
        2 => cu::union_ex_2(&case_polygons(index / 4, s)),
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
// The C++ library through `clipper2-z-sys`
// -------------------------------------------------------------------------------------------

/// (x, y, z) triples and the length of every path, as the shim takes them.
fn flatten_z(paths: &[Vec<(i64, i64, i64)>]) -> (Vec<i64>, Vec<i32>) {
    let mut flat: Vec<i64> = Vec::new();
    let mut lens: Vec<i32> = Vec::with_capacity(paths.len());
    for path in paths {
        lens.push(path.len() as i32);
        for &(x, y, z) in path {
            flat.extend_from_slice(&[x, y, z]);
        }
    }
    (flat, lens)
}

fn ptr_or_null<T>(v: &[T]) -> *const T {
    if v.is_empty() {
        std::ptr::null()
    } else {
        v.as_ptr()
    }
}

/// A `Cz2ZPaths` (flat triples and the length of every path) as owned paths.
unsafe fn read_cz2_zpaths(raw: &clipper2_z_sys::Cz2ZPaths) -> Vec<Vec<(i64, i64, i64)>> {
    let mut out = Vec::with_capacity(raw.num_paths.max(0) as usize);
    if raw.num_paths > 0 && !raw.coords.is_null() && !raw.path_lens.is_null() {
        let path_lens = std::slice::from_raw_parts(raw.path_lens, raw.num_paths as usize);
        let coords = std::slice::from_raw_parts(raw.coords, (raw.total_points * 3) as usize);
        let mut cursor = 0usize;
        for &len in path_lens {
            let mut path = Vec::with_capacity(len.max(0) as usize);
            for _ in 0..len.max(0) {
                path.push((coords[cursor * 3], coords[cursor * 3 + 1], coords[cursor * 3 + 2]));
                cursor += 1;
            }
            out.push(path);
        }
    }
    out
}

/// `cz2_pl_open` (clipper2_z_shim.cpp): `_clipper2_pl_open` of Clipper2Utils.cpp:119-136 on
/// Clipper2 1.5.2. `diff`: `ClipType::Difference`, else `Intersection`. Paths without
/// points are not handed over.
fn cpp_pl_open(diff: bool, subject: &[Polyline], clip: &[Polygon]) -> Vec<Polyline> {
    let triples = |pts: &[Point]| -> Vec<(i64, i64, i64)> { pts.iter().map(|p| (p.x, p.y, 0)).collect() };
    let src: Vec<Vec<(i64, i64, i64)>> = subject.iter().filter(|pl| !pl.points.is_empty()).map(|pl| triples(&pl.points)).collect();
    let clp: Vec<Vec<(i64, i64, i64)>> = clip.iter().filter(|pg| !pg.points.is_empty()).map(|pg| triples(&pg.points)).collect();
    let (src_xyz, src_lens) = flatten_z(&src);
    let (clip_xyz, clip_lens) = flatten_z(&clp);
    let raw = unsafe {
        clipper2_z_sys::cz2_pl_open(diff as i32, src_xyz.as_ptr(), src_lens.as_ptr(), src_lens.len() as i32, clip_xyz.as_ptr(), clip_lens.as_ptr(), clip_lens.len() as i32)
    };
    let paths = unsafe { read_cz2_zpaths(&raw) };
    unsafe { clipper2_z_sys::cz2_free_zpaths(raw) };
    paths.into_iter().map(|path| Polyline::from_points(path.into_iter().map(|(x, y, _)| Point::new(x, y)).collect())).collect()
}

/// `cz2_offset_z` (clipper2_z_shim.cpp); a contour without points is not handed over.
fn cpp_offset_z(contour: &[(i64, i64, i64)], delta: f64) -> Vec<Vec<(i64, i64, i64)>> {
    if contour.is_empty() {
        return Vec::new();
    }
    let (flat, _) = flatten_z(std::slice::from_ref(&contour.to_vec()));
    let raw = unsafe { clipper2_z_sys::cz2_offset_z(flat.as_ptr(), contour.len() as i32, delta) };
    let out = unsafe { read_cz2_zpaths(&raw) };
    unsafe { clipper2_z_sys::cz2_free_zpaths(raw) };
    out
}

/// `cz2_intersect_open_z` (clipper2_z_shim.cpp): the segments, how many of them are closed,
/// and the intersections the callback recorded.
fn cpp_intersect_open_z(src: &[Vec<(i64, i64, i64)>], boundary: &[Vec<(i64, i64, i64)>]) -> (Vec<Vec<(i64, i64, i64)>>, usize, Vec<(i64, i64)>) {
    let (src_flat, src_lens) = flatten_z(src);
    let (clip_flat, clip_lens) = flatten_z(boundary);
    let raw = unsafe {
        clipper2_z_sys::cz2_intersect_open_z(ptr_or_null(&src_flat), ptr_or_null(&src_lens), src.len() as i32, ptr_or_null(&clip_flat), ptr_or_null(&clip_lens), boundary.len() as i32)
    };
    let segments = unsafe { read_cz2_zpaths(&raw.segs) };
    let num_closed = raw.num_closed.max(0) as usize;
    let mut intersections = Vec::with_capacity(raw.num_is.max(0) as usize);
    if raw.num_is > 0 && !raw.is_a.is_null() && !raw.is_b.is_null() {
        let a = unsafe { std::slice::from_raw_parts(raw.is_a, raw.num_is as usize) };
        let b = unsafe { std::slice::from_raw_parts(raw.is_b, raw.num_is as usize) };
        for i in 0..raw.num_is as usize {
            intersections.push((a[i], b[i]));
        }
    }
    unsafe { clipper2_z_sys::cz2_free_wave_clip(raw) };
    (segments, num_closed, intersections)
}

/// `intersection_pl_2` and `diff_pl_2` of `clipper2_utils.rs` (Clipper2Utils.cpp:119-141):
/// open polylines against closed polygons, against `cz2_pl_open` of `clipper2-z-sys`
/// (Clipper2 1.5.2).
#[test]
fn pl_open_2() {
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
            let rust = if diff { cu::diff_pl_2(&subject, &clip) } else { cu::intersection_pl_2(&subject, &clip) };
            let (a, b) = (flat_polylines(&rust), flat_polylines(&cpp_pl_open(diff, &subject, &clip)));
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
    use slicer::clipper2_z::{intersect_open_z, offset_z};
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
                let (a, b) = (offset_z(&contour, delta), cpp_offset_z(&contour, delta));
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
        let (a, b) = (intersect_open_z(&zsrc, &zboundary), cpp_intersect_open_z(&zsrc, &zboundary));
        clips += 1;
        segments += b.0.len() as u64;
        intersections += b.2.len() as u64;
        if a.segments != b.0 || a.num_closed != b.1 || a.intersections != b.2 {
            same = false;
            if first.is_empty() {
                first = format!(
                    "intersect_open_z #{index}: {} segments ({} closed), {} intersections in Rust; {} ({}), {} in C++\n  src {zsrc:?}\n  boundary {zboundary:?}",
                    a.segments.len(),
                    a.num_closed,
                    a.intersections.len(),
                    b.0.len(),
                    b.1,
                    b.2.len()
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

// -------------------------------------------------------------------------------------------
// Against the calls of Clipper2Utils.cpp on the pin
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

/// `union_ex_2`, `union_ex_2_expolygons`, `offset_ex_2`, `offset2_ex_2`,
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
        check(0, format!("union_ex_2 #{index}"), flat_expolygons(&cu::union_ex_2(&polygons)), pin_utils(0, &flat_polygons(&polygons), &[], 0.0, 0.0), &mut first);
        let eps = case_expolygons(index, &mut s);
        let paths = pairs_of_expolygons(&eps);
        check(1, format!("union_ex_2_expolygons #{index}"), flat_expolygons(&cu::union_ex_2_expolygons(&eps)), pin_utils(0, &paths, &[], 0.0, 0.0), &mut first);
        let size = extent(&eps);
        let delta1 = pick_delta(size, &mut s);
        check(2, format!("offset_ex_2 #{index} delta {delta1:?}"), flat_expolygons(&cu::offset_ex_2(&eps, delta1)), pin_utils(1, &paths, &[], delta1, 0.0), &mut first);
        let delta2 = if chance(1, 4, &mut s) { pick_delta(size, &mut s) } else { -delta1 * (0.5 + below(1000, &mut s) as f64 / 1000.0) };
        check(
            3,
            format!("offset2_ex_2 #{index} deltas {delta1:?} {delta2:?}"),
            flat_expolygons(&cu::offset2_ex_2(&eps, delta1, delta2)),
            pin_utils(2, &paths, &[], delta1, delta2),
            &mut first,
        );
        let e = engine::gen_open(index, &mut s);
        let subject: Vec<Polyline> = e.open.iter().map(polyline).collect();
        let clip: Vec<Polygon> = e.clip.iter().map(polygon).collect();
        let (sp, cp) = (flat_polylines(&subject), flat_polygons(&clip));
        check(4, format!("intersection_pl_2 #{index}"), flat_polylines(&cu::intersection_pl_2(&subject, &clip)), pin_utils(3, &sp, &cp, 0.0, 0.0), &mut first);
        check(5, format!("diff_pl_2 #{index}"), flat_polylines(&cu::diff_pl_2(&subject, &clip)), pin_utils(4, &sp, &cp, 0.0, 0.0), &mut first);
    }
    report(&format!(
        "rust_arm_is_the_pin: {N} cases each of union_ex_2, union_ex_2_expolygons, offset_ex_2, offset2_ex_2, intersection_pl_2, diff_pl_2; non-empty {nonempty:?}; differ {differ:?}\n"
    ));
    assert!(differ.iter().all(|d| *d == 0), "the Rust arm differs from the calls of Clipper2Utils.cpp on the pin: {differ:?}. The first: {first}");
    assert!(nonempty.iter().all(|n| *n > N / 2));
}
