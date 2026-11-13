//! The ClipperUtils.cpp routines of `src/clipper_utils.rs` (and the wave propagation of
//! `src/region_expansion.rs`) that run on the Rust port of ClipperLib, against the shim
//! routines of `clipper-z-sys` they replace under the gate `CLIPPER_RUST`: the same inputs go
//! through the Rust routine and through the `cz_*` function, and the two results must be the
//! same paths in the same order, each with the same points from the same start vertex (and,
//! for the routines that return ExPolygons, the same contours with the same holes).
//!
//! Inputs: rectangles, convex polygons, stars, random (self-intersecting) polygons,
//! rectilinear outlines with collinear points, tiny polygons, degenerate and empty paths,
//! polygons with holes, several shapes overlapping or touching (coordinates snapped to a
//! grid), from tens of units to 2e7; offsets with the three join types, both signs, f32-rounded
//! and not, from a fraction of a unit to more than the shape (so that it collapses); open
//! polylines that cross the clip polygons, run along their edges and through their vertices.
//! The variable offset takes one offset per point, constant, smooth or arbitrary along a ring.
//!
//! Every case is made from its own seed; a failure prints the case and its input.
//!
//! The last test goes through the wrappers the engine calls (`union_ex_clib`, ...), which
//! take the Rust routines unless `CLIPPER_RUST=0`, and compares them with the shim.

use clipper_z_sys as cz;
use slicer::clipper::{self as cl, ClipType, IntPoint, Path, Paths, PolyFillType, PolyTree};
use slicer::clipper_utils as cu;
use slicer::geometry::{ExPolygon, Point, Polygon, Polyline};
use std::fmt::Write as _;

// ---------------------------------------------------------------------------
// Random inputs
// ---------------------------------------------------------------------------

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

/// A value in `0.0..1.0`.
fn unit(s: &mut u64) -> f64 {
    (xorshift(s) >> 11) as f64 / (1u64 << 53) as f64
}

/// The size of a scene and the grid its coordinates are snapped to.
#[derive(Clone, Copy)]
struct Scene {
    size: i64,
    grid: i64,
}

fn scene(s: &mut u64) -> Scene {
    let size = [40, 400, 5_000, 60_000, 800_000, 6_000_000, 20_000_000][below(7, s) as usize];
    let grid = match below(6, s) {
        0 | 1 | 2 => 1,
        3 => (size / 200).max(1),
        4 => (size / 20).max(1),
        _ => (size / 6).max(1),
    };
    Scene { size, grid }
}

fn pt(x: i64, y: i64, sc: Scene) -> IntPoint {
    let snap = |v: i64| if sc.grid > 1 { v.div_euclid(sc.grid) * sc.grid } else { v };
    IntPoint::new(snap(x) as i32, snap(y) as i32)
}

/// One closed shape around (cx, cy) of about the given radius.
fn shape(cx: i64, cy: i64, radius: i64, sc: Scene, s: &mut u64) -> Path {
    let radius = radius.max(2);
    let mut p = Path::new();
    match below(16, s) {
        // rectangle
        0 | 1 | 2 => {
            let (w, h) = (1 + below(radius as u64, s) as i64, 1 + below(radius as u64, s) as i64);
            p.push(pt(cx - w, cy - h, sc));
            p.push(pt(cx + w, cy - h, sc));
            p.push(pt(cx + w, cy + h, sc));
            p.push(pt(cx - w, cy + h, sc));
        }
        // convex polygon
        3 | 4 | 5 => {
            let n = 3 + below(10, s) as i64;
            let phase = unit(s) * 6.283185307179586;
            for i in 0..n {
                let a = phase + 6.283185307179586 * i as f64 / n as f64;
                p.push(pt(cx + (radius as f64 * a.cos()) as i64, cy + (radius as f64 * a.sin()) as i64, sc));
            }
        }
        // star: concave
        6 | 7 | 8 => {
            let n = 2 * (3 + below(6, s) as i64);
            let inner = 0.2 + 0.6 * unit(s);
            let phase = unit(s) * 6.283185307179586;
            for i in 0..n {
                let a = phase + 6.283185307179586 * i as f64 / n as f64;
                let rr = if i % 2 == 0 { radius as f64 } else { radius as f64 * inner };
                p.push(pt(cx + (rr * a.cos()) as i64, cy + (rr * a.sin()) as i64, sc));
            }
        }
        // random points: self-intersecting
        9 | 10 => {
            let n = 3 + below(8, s);
            for _ in 0..n {
                p.push(pt(cx + r(radius, s), cy + r(radius, s), sc));
            }
        }
        // rectilinear outline with collinear and repeated points
        11 | 12 => {
            let (w, h) = (1 + below(radius as u64, s) as i64, 1 + below(radius as u64, s) as i64);
            let corners = [(cx - w, cy - h), (cx + w, cy - h), (cx + w, cy + h), (cx - w, cy + h)];
            for k in 0..4 {
                let (a, b) = (corners[k], corners[(k + 1) % 4]);
                p.push(pt(a.0, a.1, sc));
                for _ in 0..below(3, s) {
                    let t = unit(s);
                    p.push(pt(a.0 + ((b.0 - a.0) as f64 * t) as i64, a.1 + ((b.1 - a.1) as f64 * t) as i64, sc));
                }
                if below(6, s) == 0 {
                    p.push(pt(b.0, b.1, sc));
                }
            }
        }
        // tiny
        13 => {
            let n = 3 + below(4, s);
            for _ in 0..n {
                p.push(pt(cx + r(3, s), cy + r(3, s), Scene { size: sc.size, grid: 1 }));
            }
        }
        // sliver
        14 => {
            p.push(pt(cx - radius, cy, sc));
            p.push(pt(cx + radius, cy + r(2, s), sc));
            p.push(pt(cx + radius, cy + 1 + r(2, s), sc));
            p.push(pt(cx - radius, cy + 1, sc));
        }
        // degenerate: no point, one point, two points
        _ => {
            for _ in 0..below(3, s) {
                p.push(pt(cx + r(radius, s), cy + r(radius, s), sc));
            }
        }
    }
    if below(2, s) == 0 {
        p.reverse();
    }
    p
}

fn oriented(mut p: Path, ccw: bool) -> Path {
    if cl::orientation(&p) != ccw {
        p.reverse();
    }
    p
}

/// A contour (counter-clockwise) and up to three holes (clockwise) inside or across it.
fn shape_with_holes(cx: i64, cy: i64, radius: i64, sc: Scene, s: &mut u64) -> (Path, Paths) {
    let contour = oriented(shape(cx, cy, radius, sc, s), true);
    let mut holes = Paths::new();
    if below(3, s) != 0 {
        for _ in 0..1 + below(3, s) {
            let hr = (radius / (2 + below(5, s) as i64)).max(2);
            let spread = if below(5, s) == 0 { radius } else { radius / 2 };
            holes.push(oriented(shape(cx + r(spread.max(1), s), cy + r(spread.max(1), s), hr, sc, s), false));
        }
    }
    (contour, holes)
}

/// Several shapes, overlapping, touching or apart, some with holes: raw closed paths.
fn soup(sc: Scene, s: &mut u64) -> Paths {
    let mut out = Paths::new();
    let n = match below(10, s) {
        0 => 0,
        1 | 2 | 3 => 1,
        4 | 5 | 6 => 2,
        _ => 3 + below(4, s),
    };
    for _ in 0..n {
        let radius = (sc.size / (1 + below(6, s) as i64)).max(2);
        let (cx, cy) = (r(sc.size, s), r(sc.size, s));
        if below(3, s) == 0 {
            let (c, h) = shape_with_holes(cx, cy, radius, sc, s);
            out.push(c);
            out.extend(h);
        } else {
            out.push(shape(cx, cy, radius, sc, s));
        }
    }
    out
}

/// Expolygons: contours with their holes.
fn expolygon_soup(sc: Scene, s: &mut u64) -> Vec<(Path, Paths)> {
    let n = 1 + below(4, s);
    (0..n)
        .map(|_| {
            let radius = (sc.size / (1 + below(6, s) as i64)).max(2);
            shape_with_holes(r(sc.size, s), r(sc.size, s), radius, sc, s)
        })
        .collect()
}

/// Open polylines over a set of clip polygons: crossing them, along their edges, through
/// their vertices, and elsewhere.
fn polylines(clip: &Paths, sc: Scene, min_points: usize, s: &mut u64) -> Paths {
    let mut out = Paths::new();
    for _ in 0..1 + below(4, s) {
        let mut p = Path::new();
        let poly = if clip.is_empty() { None } else { Some(&clip[below(clip.len() as u64, s) as usize]) };
        match (below(6, s), poly) {
            // along the edges of a clip polygon, from vertex to vertex
            (0, Some(poly)) if poly.len() >= 2 => {
                let start = below(poly.len() as u64, s) as usize;
                for k in 0..=1 + below(poly.len() as u64, s) as usize {
                    p.push(poly[(start + k) % poly.len()]);
                }
            }
            // on an edge of a clip polygon, between two points of it, and beyond
            (1, Some(poly)) if poly.len() >= 2 => {
                let i = below(poly.len() as u64, s) as usize;
                let (a, b) = (poly[i], poly[(i + 1) % poly.len()]);
                for _ in 0..2 + below(3, s) {
                    let t = unit(s) * 1.6 - 0.3;
                    p.push(IntPoint::new(
                        a.x + ((b.x - a.x) as f64 * t) as i32,
                        a.y + ((b.y - a.y) as f64 * t) as i32,
                    ));
                }
            }
            // through vertices of clip polygons
            (2, Some(_)) => {
                for _ in 0..2 + below(4, s) {
                    let q = &clip[below(clip.len() as u64, s) as usize];
                    if q.is_empty() {
                        p.push(pt(r(sc.size, s), r(sc.size, s), sc));
                    } else {
                        p.push(q[below(q.len() as u64, s) as usize]);
                    }
                }
            }
            // a closed loop: the first point again at the end
            (3, _) => {
                for _ in 0..3 + below(4, s) {
                    p.push(pt(r(2 * sc.size, s), r(2 * sc.size, s), sc));
                }
                let first = p[0];
                p.push(first);
            }
            // anywhere
            _ => {
                for _ in 0..min_points as u64 + below(6, s) {
                    p.push(pt(r(2 * sc.size, s), r(2 * sc.size, s), sc));
                }
            }
        }
        if p.len() >= min_points {
            out.push(p);
        }
    }
    out
}

/// An offset for a scene: from a fraction of a unit to more than the scene, either sign,
/// most of the time rounded to f32 as the callers of the shim do.
fn delta(sc: Scene, s: &mut u64) -> f64 {
    let magnitude = match below(10, s) {
        0 => 10.0,
        1 => 0.3 + unit(s),
        2 => 1.0 + unit(s) * 30.0,
        3 | 4 => sc.size as f64 * (0.001 + 0.02 * unit(s)),
        5 | 6 => sc.size as f64 * (0.02 + 0.2 * unit(s)),
        7 => sc.size as f64 * (0.2 + 0.5 * unit(s)),
        8 => sc.size as f64 * (0.7 + 1.5 * unit(s)),
        _ => (sc.grid * (1 + below(4, s) as i64)) as f64,
    };
    let d = if below(2, s) == 0 { magnitude } else { -magnitude };
    match below(12, s) {
        0 => 0.0,
        1 => d,
        _ => d as f32 as f64,
    }
}

/// The shim's code of a join type (0=miter, 1=round, 2=square) and a `miterLimit` for it:
/// the miter limit for jtMiter and jtSquare, the arc tolerance for jtRound.
fn join(delta: f64, s: &mut u64) -> (i32, f64) {
    let code = below(3, s) as i32;
    let limit = if code == 1 {
        match below(5, s) {
            0 | 1 => 3.0,
            2 => (delta.abs() * 0.01).max(0.3),
            3 => delta.abs() * 0.3,
            _ => 0.25,
        }
    } else {
        [3.0, 3.0, 5.0, 2.0, 1.2, 0.0][below(6, s) as usize]
    };
    // Keep the arcs of a round join within a few hundred points.
    let limit = if code == 1 { limit.max(delta.abs() * 2e-5) } else { limit };
    (code, limit)
}

// ---------------------------------------------------------------------------
// The shim
// ---------------------------------------------------------------------------

fn flat(paths: &Paths) -> (Vec<i32>, Vec<i32>) {
    let mut xy = Vec::new();
    let mut lens = Vec::new();
    for p in paths {
        lens.push(p.len() as i32);
        for q in p {
            xy.push(q.x);
            xy.push(q.y);
        }
    }
    (xy, lens)
}

fn ptr_or_null(v: &[i32]) -> *const i32 {
    if v.is_empty() {
        std::ptr::null()
    } else {
        v.as_ptr()
    }
}

/// The paths of a `CzZPaths` with the z of each point; frees it.
fn take(raw: cz::CzZPaths) -> Vec<Vec<(i32, i32, i32)>> {
    let mut out = Vec::new();
    if raw.num_paths > 0 && !raw.path_lens.is_null() {
        let lens = unsafe { std::slice::from_raw_parts(raw.path_lens, raw.num_paths as usize) };
        let coords: &[i32] = if raw.coords.is_null() {
            &[]
        } else {
            unsafe { std::slice::from_raw_parts(raw.coords, (raw.total_points * 3) as usize) }
        };
        let mut cursor = 0usize;
        for &len in lens {
            let mut path = Vec::new();
            for _ in 0..len {
                path.push((coords[cursor * 3], coords[cursor * 3 + 1], coords[cursor * 3 + 2]));
                cursor += 1;
            }
            out.push(path);
        }
    }
    unsafe { cz::cz_free_zpaths(raw) };
    out
}

/// A flat shim result as `Paths`.
fn take_paths(raw: cz::CzZPaths) -> Paths {
    take(raw).into_iter().map(|p| p.into_iter().map(|(x, y, _)| IntPoint::new(x, y)).collect()).collect()
}

/// An expolygon as plain data: the contour and the holes.
type Ex = (Vec<(i64, i64)>, Vec<Vec<(i64, i64)>>);

/// A grouped shim result (`marshal_grouped`: the z of the first point of a path says whether
/// it is a hole) decoded as the wrappers of the engine decode it: a contour starts an
/// expolygon, a hole goes to the last one.
fn take_grouped(raw: cz::CzZPaths) -> Vec<Ex> {
    let mut out: Vec<Ex> = Vec::new();
    for path in take(raw) {
        let is_hole = path.first().map(|q| q.2).unwrap_or(0);
        let ring: Vec<(i64, i64)> = path.iter().map(|q| (q.0 as i64, q.1 as i64)).collect();
        if is_hole == 0 {
            out.push((ring, Vec::new()));
        } else if let Some(last) = out.last_mut() {
            last.1.push(ring);
        }
    }
    out
}

fn ex_data(expolygons: &[ExPolygon]) -> Vec<Ex> {
    expolygons
        .iter()
        .map(|e| {
            (
                e.contour.points().iter().map(|p| (p.x, p.y)).collect(),
                e.holes.iter().map(|h| h.points().iter().map(|p| (p.x, p.y)).collect()).collect(),
            )
        })
        .collect()
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

fn fill(code: i32) -> PolyFillType {
    cu::clib_fill_type(code)
}

/// What a test went through: how many cases, how many of them with a result that is not
/// empty, how many points in the results. `CLIPPER_UTILS_PARITY_STATS=<file>` appends a line
/// per test to that file. A test whose inputs hardly ever give a result compares nothing, so
/// one case in ten at least has to give one.
struct Tally {
    name: String,
    cases: u64,
    non_empty: u64,
    points: u64,
}

impl Tally {
    fn new(name: &str) -> Self {
        Self { name: name.to_string(), cases: 0, non_empty: 0, points: 0 }
    }

    fn add(&mut self, points: usize) {
        self.cases += 1;
        if points > 0 {
            self.non_empty += 1;
        }
        self.points += points as u64;
    }
}

impl Drop for Tally {
    fn drop(&mut self) {
        if std::thread::panicking() {
            return;
        }
        if let Ok(path) = std::env::var("CLIPPER_UTILS_PARITY_STATS") {
            use std::io::Write as _;
            if let Ok(mut f) = std::fs::OpenOptions::new().create(true).append(true).open(path) {
                let _ = writeln!(
                    f,
                    "{}: {} cases compared, {} with a non-empty result, {} points",
                    self.name, self.cases, self.non_empty, self.points
                );
            }
        }
        assert!(
            self.cases < 1000 || self.non_empty * 10 >= self.cases,
            "{}: only {} of {} cases gave a result",
            self.name,
            self.non_empty,
            self.cases
        );
    }
}

fn points(paths: &Paths) -> usize {
    paths.iter().map(|p| p.len()).sum()
}

fn ex_points(expolygons: &[Ex]) -> usize {
    expolygons.iter().map(|e| e.0.len() + e.1.iter().map(|h| h.len()).sum::<usize>()).sum()
}

/// Number of cases of a test: `CLIPPER_UTILS_PARITY_CASES` or the given default.
fn cases(default: u64) -> u64 {
    std::env::var("CLIPPER_UTILS_PARITY_CASES").ok().and_then(|v| v.parse().ok()).unwrap_or(default)
}

// ---------------------------------------------------------------------------
// Unions
// ---------------------------------------------------------------------------

/// `cz_union_flat` against `clipper_union<ClipperLib::Paths>`.
#[test]
fn union_flat_matches_shim() {
    let mut tally = Tally::new("cz_union_flat");
    for i in 0..cases(30_000) {
        let mut s = case_seed(1, i);
        let subject = soup(scene(&mut s), &mut s);
        let (xy, lens) = flat(&subject);
        let want = take_paths(unsafe { cz::cz_union_flat(xy.as_ptr(), lens.as_ptr(), lens.len() as i32) });
        tally.add(points(&want));
        let got: Paths = cu::clipper_union::<Paths>(&subject, PolyFillType::pftNonZero);
        assert_eq!(got, want, "cz_union_flat case {i}\n  subject: {}", show_paths(&subject));
    }
}

/// `cz_union_polytree` against `PolyTreeToExPolygons(clipper_union<ClipperLib::PolyTree>)`.
#[test]
fn union_polytree_matches_shim() {
    let mut tally = Tally::new("cz_union_polytree");
    for i in 0..cases(30_000) {
        let mut s = case_seed(2, i);
        let subject = soup(scene(&mut s), &mut s);
        let (xy, lens) = flat(&subject);
        let want = take_grouped(unsafe { cz::cz_union_polytree(xy.as_ptr(), lens.as_ptr(), lens.len() as i32) });
        tally.add(ex_points(&want));
        let got = ex_data(&cu::poly_tree_to_ex_polygons(&cu::clipper_union::<PolyTree>(&subject, PolyFillType::pftNonZero)));
        assert_eq!(got, want, "cz_union_polytree case {i}\n  subject: {}", show_paths(&subject));
    }
}

/// `cz_union_ex` against `PolyTreeToExPolygons(clipper_do_polytree(ctUnion, ...))`, every
/// fill type code.
#[test]
fn union_ex_matches_shim() {
    let mut tally = Tally::new("cz_union_ex");
    for i in 0..cases(30_000) {
        let mut s = case_seed(3, i);
        let subject = soup(scene(&mut s), &mut s);
        let fill_type = below(6, &mut s) as i32 - 1;
        let (xy, lens) = flat(&subject);
        let want = take_grouped(unsafe { cz::cz_union_ex(xy.as_ptr(), lens.as_ptr(), lens.len() as i32, fill_type) });
        tally.add(ex_points(&want));
        let got = ex_data(&cu::poly_tree_to_ex_polygons(&cu::clipper_do_polytree(
            ClipType::ctUnion,
            &subject,
            &Paths::new(),
            fill(fill_type),
        )));
        assert_eq!(got, want, "cz_union_ex case {i} fill_type={fill_type}\n  subject: {}", show_paths(&subject));
    }
}

/// `cz_union_ex_safety` against `union_safety_offset_ex_paths`.
#[test]
fn union_ex_safety_matches_shim() {
    let mut tally = Tally::new("cz_union_ex_safety");
    for i in 0..cases(30_000) {
        let mut s = case_seed(4, i);
        let subject = soup(scene(&mut s), &mut s);
        let (xy, lens) = flat(&subject);
        let want = take_grouped(unsafe { cz::cz_union_ex_safety(xy.as_ptr(), lens.as_ptr(), lens.len() as i32) });
        tally.add(ex_points(&want));
        let got = ex_data(&cu::union_safety_offset_ex_paths(&subject));
        assert_eq!(got, want, "cz_union_ex_safety case {i}\n  subject: {}", show_paths(&subject));
    }
}

// ---------------------------------------------------------------------------
// Booleans on closed paths
// ---------------------------------------------------------------------------

type CzBoolean = unsafe extern "C" fn(*const i32, *const i32, i32, *const i32, *const i32, i32) -> cz::CzZPaths;

fn closed_boolean(test: u64, name: &str, shim: CzBoolean, clip_type: ClipType, safety: bool, n: u64) {
    let mut tally = Tally::new(name);
    for i in 0..n {
        let mut s = case_seed(test, i);
        let sc = scene(&mut s);
        let subject = soup(sc, &mut s);
        let clip = soup(sc, &mut s);
        let (sxy, slens) = flat(&subject);
        let (cxy, clens) = flat(&clip);
        // The wrappers hand over a null pointer for an empty clip.
        let want = take_paths(unsafe {
            shim(sxy.as_ptr(), slens.as_ptr(), slens.len() as i32, ptr_or_null(&cxy), ptr_or_null(&clens), clens.len() as i32)
        });
        tally.add(points(&want));
        let clip_read = cu::clib_read_closed_paths_or_null(&cxy, &clens);
        let got: Paths = if safety {
            cu::clipper_do_safety_offset::<Paths>(
                clip_type,
                &subject,
                &clip_read,
                PolyFillType::pftNonZero,
                cu::ApplySafetyOffset::Yes,
            )
        } else {
            cu::clipper_do::<Paths>(clip_type, &subject, &clip_read, PolyFillType::pftNonZero)
        };
        assert_eq!(
            got,
            want,
            "{name} case {i}\n  subject: {}\n  clip: {}",
            show_paths(&subject),
            show_paths(&clip)
        );
    }
}

/// `cz_difference_closed` against `clipper_do<ClipperLib::Paths>(ctDifference, ...)`.
#[test]
fn difference_closed_matches_shim() {
    closed_boolean(5, "cz_difference_closed", cz::cz_difference_closed, ClipType::ctDifference, false, cases(30_000));
}

/// `cz_intersection_closed` against `clipper_do<ClipperLib::Paths>(ctIntersection, ...)`.
#[test]
fn intersection_closed_matches_shim() {
    closed_boolean(6, "cz_intersection_closed", cz::cz_intersection_closed, ClipType::ctIntersection, false, cases(30_000));
}

/// `cz_difference_closed_safety` against `clipper_do` with `ApplySafetyOffset::Yes`.
#[test]
fn difference_closed_safety_matches_shim() {
    closed_boolean(
        7,
        "cz_difference_closed_safety",
        cz::cz_difference_closed_safety,
        ClipType::ctDifference,
        true,
        cases(30_000),
    );
}

/// `cz_intersection_closed_safety` against `clipper_do` with `ApplySafetyOffset::Yes`.
#[test]
fn intersection_closed_safety_matches_shim() {
    closed_boolean(
        8,
        "cz_intersection_closed_safety",
        cz::cz_intersection_closed_safety,
        ClipType::ctIntersection,
        true,
        cases(30_000),
    );
}

// ---------------------------------------------------------------------------
// Open polylines against closed polygons
// ---------------------------------------------------------------------------

fn open_boolean(test: u64, name: &str, shim: CzBoolean, clip_type: ClipType, min_points: usize, n: u64) {
    let mut tally = Tally::new(name);
    for i in 0..n {
        let mut s = case_seed(test, i);
        let sc = scene(&mut s);
        let clip = soup(sc, &mut s);
        let subject = polylines(&clip, sc, min_points, &mut s);
        let (sxy, slens) = flat(&subject);
        let (cxy, clens) = flat(&clip);
        let want = take_paths(unsafe {
            shim(sxy.as_ptr(), slens.as_ptr(), slens.len() as i32, ptr_or_null(&cxy), ptr_or_null(&clens), clens.len() as i32)
        });
        tally.add(points(&want));
        let got = cu::_clipper_pl_open(clip_type, &subject, &cu::clib_read_closed_paths_or_null(&cxy, &clens));
        assert_eq!(
            got,
            want,
            "{name} case {i}\n  subject: {}\n  clip: {}",
            show_paths(&subject),
            show_paths(&clip)
        );
    }
}

/// `cz_intersection_pl` against `_clipper_pl_open(ctIntersection, ...)`.
#[test]
fn intersection_pl_matches_shim() {
    open_boolean(9, "cz_intersection_pl", cz::cz_intersection_pl, ClipType::ctIntersection, 2, cases(40_000));
}

/// `cz_diff_pl` against `_clipper_pl_open(ctDifference, ...)`; the wrapper lets a polyline of
/// one point through.
#[test]
fn diff_pl_matches_shim() {
    open_boolean(10, "cz_diff_pl", cz::cz_diff_pl, ClipType::ctDifference, 1, cases(40_000));
}

// ---------------------------------------------------------------------------
// Offsets
// ---------------------------------------------------------------------------

/// `cz_offset_paths` against `offset_paths<ClipperLib::Paths>`.
#[test]
fn offset_paths_matches_shim() {
    let mut tally = Tally::new("cz_offset_paths");
    for i in 0..cases(30_000) {
        let mut s = case_seed(11, i);
        let sc = scene(&mut s);
        let paths = soup(sc, &mut s);
        let d = delta(sc, &mut s);
        let (jt, limit) = join(d, &mut s);
        let (xy, lens) = flat(&paths);
        let want = take_paths(unsafe { cz::cz_offset_paths(xy.as_ptr(), lens.as_ptr(), lens.len() as i32, d, jt, limit) });
        tally.add(points(&want));
        let got: Paths = cu::offset_paths::<Paths>(&paths, d, cu::clib_join_type(jt), limit);
        assert_eq!(
            got,
            want,
            "cz_offset_paths case {i} delta={d:?} join_type={jt} miter_limit={limit:?}\n  paths: {}",
            show_paths(&paths)
        );
    }
}

/// `cz_offset_ex_paths` against `PolyTreeToExPolygons(offset_paths<ClipperLib::PolyTree>)`.
#[test]
fn offset_ex_paths_matches_shim() {
    let mut tally = Tally::new("cz_offset_ex_paths");
    for i in 0..cases(30_000) {
        let mut s = case_seed(12, i);
        let sc = scene(&mut s);
        let paths = soup(sc, &mut s);
        let d = delta(sc, &mut s);
        let (jt, limit) = join(d, &mut s);
        let (xy, lens) = flat(&paths);
        let want =
            take_grouped(unsafe { cz::cz_offset_ex_paths(xy.as_ptr(), lens.as_ptr(), lens.len() as i32, d, jt, limit) });
        tally.add(ex_points(&want));
        let got = ex_data(&cu::poly_tree_to_ex_polygons(&cu::offset_paths::<PolyTree>(
            &paths,
            d,
            cu::clib_join_type(jt),
            limit,
        )));
        assert_eq!(
            got,
            want,
            "cz_offset_ex_paths case {i} delta={d:?} join_type={jt} miter_limit={limit:?}\n  paths: {}",
            show_paths(&paths)
        );
    }
}

/// `cz_offset_expolygon` against `expolygon_offset`.
#[test]
fn offset_expolygon_matches_shim() {
    let mut tally = Tally::new("cz_offset_expolygon");
    for i in 0..cases(30_000) {
        let mut s = case_seed(13, i);
        let sc = scene(&mut s);
        let radius = (sc.size / (1 + below(4, &mut s) as i64)).max(2);
        let (contour, holes) = shape_with_holes(r(sc.size, &mut s), r(sc.size, &mut s), radius, sc, &mut s);
        if contour.is_empty() {
            // The wrapper returns before the call.
            continue;
        }
        let d = delta(sc, &mut s);
        let (jt, limit) = join(d, &mut s);
        let (cxy, _) = flat(&vec![contour.clone()]);
        let (hxy, hlens) = flat(&holes);
        let want = take_paths(unsafe {
            cz::cz_offset_expolygon(
                cxy.as_ptr(),
                contour.len() as i32,
                ptr_or_null(&hxy),
                ptr_or_null(&hlens),
                holes.len() as i32,
                d,
                jt,
                limit,
            )
        });
        tally.add(points(&want));
        let got = cu::expolygon_offset(&contour, &holes, d, cu::clib_join_type(jt), limit);
        assert_eq!(
            got,
            want,
            "cz_offset_expolygon case {i} delta={d:?} join_type={jt} miter_limit={limit:?}\n  contour: {}\n  holes: {}",
            show_paths(&vec![contour.clone()]),
            show_paths(&holes)
        );
    }
}

/// The layout `cz_offset2_ex` takes: the paths of the expolygons with a hole flag each.
fn grouped_layout(expolygons: &[(Path, Paths)]) -> (Vec<i32>, Vec<i32>, Vec<i32>) {
    let mut all = Paths::new();
    let mut is_hole = Vec::new();
    for (contour, holes) in expolygons {
        // The wrapper leaves out an expolygon without contour points.
        if contour.is_empty() {
            continue;
        }
        all.push(contour.clone());
        is_hole.push(0);
        for h in holes {
            all.push(h.clone());
            is_hole.push(1);
        }
    }
    let (xy, lens) = flat(&all);
    (xy, lens, is_hole)
}

/// `cz_offset2_ex` against `offset2_ex_paths`.
#[test]
fn offset2_ex_matches_shim() {
    let mut tally = Tally::new("cz_offset2_ex");
    for i in 0..cases(30_000) {
        let mut s = case_seed(14, i);
        let sc = scene(&mut s);
        let expolygons = expolygon_soup(sc, &mut s);
        let d1 = delta(sc, &mut s);
        // Most of the time the second offset undoes the first, as the callers do.
        let d2 = match below(4, &mut s) {
            0 => delta(sc, &mut s),
            1 => -d1 * (0.5 + unit(&mut s)),
            _ => -d1,
        };
        let (jt, limit) = join(d1.abs().max(d2.abs()), &mut s);
        let (xy, lens, is_hole) = grouped_layout(&expolygons);
        let want = take_grouped(unsafe {
            cz::cz_offset2_ex(xy.as_ptr(), lens.as_ptr(), is_hole.as_ptr(), lens.len() as i32, d1, d2, jt, limit)
        });
        tally.add(ex_points(&want));
        let got = ex_data(&cu::offset2_ex_paths(&xy, &lens, &is_hole, d1, d2, cu::clib_join_type(jt), limit));
        assert_eq!(
            got,
            want,
            "cz_offset2_ex case {i} delta1={d1:?} delta2={d2:?} join_type={jt} miter_limit={limit:?}\n  xy: {xy:?}\n  lens: {lens:?}\n  is_hole: {is_hole:?}"
        );
    }
}

/// `cz_offset_polyline` against `offset_polyline_paths`.
#[test]
fn offset_polyline_matches_shim() {
    let mut tally = Tally::new("cz_offset_polyline");
    for i in 0..cases(40_000) {
        let mut s = case_seed(15, i);
        let sc = scene(&mut s);
        let clip = soup(sc, &mut s);
        let mut lines = polylines(&clip, sc, 0, &mut s);
        let polyline = lines.pop().unwrap_or_default();
        // The offset of a polyline is a width: positive.
        let d = delta(sc, &mut s).abs().max(0.05);
        let limit = [0.0, 0.0, 3.0, 2.0][below(4, &mut s) as usize];
        let (xy, _) = flat(&vec![polyline.clone()]);
        let want = take_paths(unsafe { cz::cz_offset_polyline(xy.as_ptr(), polyline.len() as i32, d, limit) });
        tally.add(points(&want));
        let got = cu::offset_polyline_paths(&polyline, d, limit);
        assert_eq!(
            got,
            want,
            "cz_offset_polyline case {i} delta={d:?} miter_limit={limit:?}\n  polyline: {}",
            show_paths(&vec![polyline.clone()])
        );
    }
}

/// `cz_simplify_polygons` against `ClipperLib::SimplifyPolygons` of the port.
#[test]
fn simplify_polygons_matches_shim() {
    let mut tally = Tally::new("cz_simplify_polygons");
    for i in 0..cases(30_000) {
        let mut s = case_seed(16, i);
        let subject = soup(scene(&mut s), &mut s);
        let fill_type = below(6, &mut s) as i32 - 1;
        let (xy, lens) = flat(&subject);
        let want =
            take_paths(unsafe { cz::cz_simplify_polygons(xy.as_ptr(), lens.as_ptr(), lens.len() as i32, fill_type) });
        tally.add(points(&want));
        let got = cl::simplify_polygons(&subject, fill(fill_type));
        assert_eq!(
            got,
            want,
            "cz_simplify_polygons case {i} fill_type={fill_type}\n  subject: {}",
            show_paths(&subject)
        );
    }
}

/// `cz_variable_offset_inner_ex` against `variable_offset_inner_ex_paths`: an expolygon with
/// one offset per point, none of them positive, from zero to more than the shape; constant
/// along a ring, varying smoothly, or anything.
#[test]
fn variable_offset_inner_ex_matches_shim() {
    let mut tally = Tally::new("cz_variable_offset_inner_ex");
    for i in 0..cases(40_000) {
        let mut s = case_seed(20, i);
        let sc = scene(&mut s);
        let radius = (sc.size / (1 + below(4, &mut s) as i64)).max(2);
        let (contour, holes) = shape_with_holes(r(sc.size, &mut s), r(sc.size, &mut s), radius, sc, &mut s);
        if contour.is_empty() {
            continue;
        }
        let mut rings = vec![contour];
        rings.extend(holes);
        let reach = match below(5, &mut s) {
            0 => 30.0,
            1 => radius as f64 * 0.02,
            2 => radius as f64 * 0.2,
            3 => radius as f64 * 0.6,
            _ => radius as f64 * 1.5,
        };
        let mut deltas: Vec<f32> = Vec::new();
        for ring in &rings {
            let kind = below(4, &mut s);
            let base = unit(&mut s);
            let phase = unit(&mut s) * 6.283185307179586;
            for k in 0..ring.len() {
                let f = match kind {
                    0 => base,
                    1 => 0.5 + 0.5 * (phase + 6.283185307179586 * k as f64 / ring.len() as f64).sin(),
                    2 => unit(&mut s),
                    _ => {
                        if below(3, &mut s) == 0 {
                            0.0
                        } else {
                            base
                        }
                    }
                };
                deltas.push(-(reach * f) as f32);
            }
        }
        let miter_limit = [2.0, 2.0, 3.0, 1.0, 5.0][below(5, &mut s) as usize];
        let (xy, lens) = flat(&rings);
        let want = take_grouped(unsafe {
            cz::cz_variable_offset_inner_ex(xy.as_ptr(), lens.as_ptr(), lens.len() as i32, deltas.as_ptr(), miter_limit)
        });
        tally.add(ex_points(&want));
        let got = ex_data(&cu::variable_offset_inner_ex_paths(&rings, &deltas, miter_limit));
        assert_eq!(
            got,
            want,
            "cz_variable_offset_inner_ex case {i} miter_limit={miter_limit:?}\n  rings: {}\n  deltas (bits): {:?}",
            show_paths(&rings),
            deltas.iter().map(|d| d.to_bits()).collect::<Vec<_>>()
        );

        // The wrapper of the engine, which takes the Rust routine while the gate is on.
        if cu::clipper_rust() {
            let mut expolygon = ExPolygon::new(polygons(&vec![rings[0].clone()]).remove(0));
            expolygon.holes = polygons(&rings[1..].to_vec());
            let mut per_ring: Vec<Vec<f32>> = Vec::new();
            let mut at = 0usize;
            for ring in &rings {
                per_ring.push(deltas[at..at + ring.len()].to_vec());
                at += ring.len();
            }
            assert_eq!(
                ex_data(&cu::variable_offset_inner_ex_clib(&expolygon, &per_ring, miter_limit)),
                want,
                "variable_offset_inner_ex_clib case {i} miter_limit={miter_limit:?}\n  rings: {}\n  deltas (bits): {:?}",
                show_paths(&rings),
                deltas.iter().map(|d| d.to_bits()).collect::<Vec<_>>()
            );
        }
    }
}

// ---------------------------------------------------------------------------
// Wave propagation (RegionExpansion.cpp)
// ---------------------------------------------------------------------------

/// `cz_propagate_wave` against `propagate_wave_from_boundary`.
#[test]
fn propagate_wave_matches_shim() {
    let mut tally = Tally::new("cz_propagate_wave");
    for i in 0..cases(20_000) {
        let mut s = case_seed(17, i);
        let sc = scene(&mut s);
        let radius = (sc.size / (1 + below(3, &mut s) as i64)).max(2);
        let (contour, holes) = shape_with_holes(r(sc.size / 2, &mut s), r(sc.size / 2, &mut s), radius, sc, &mut s);
        let mut boundary = vec![contour];
        boundary.extend(holes);
        let seed = polylines(&boundary, sc, 2, &mut s);
        if seed.is_empty() {
            continue;
        }
        let initial_step = (sc.size as f64 * (0.005 + 0.1 * unit(&mut s))).max(0.5) as f32 as f64;
        let other_step = if below(3, &mut s) == 0 {
            (sc.size as f64 * (0.005 + 0.1 * unit(&mut s))).max(0.5) as f32 as f64
        } else {
            initial_step
        };
        let num_other_steps = below(4, &mut s) as i32;
        let arc_tolerance = match below(3, &mut s) {
            0 => 10_000.0,
            1 => initial_step * 0.05,
            _ => (initial_step * 0.002).max(0.3),
        };
        let shortest_edge_length = if below(4, &mut s) == 0 { 0.0 } else { initial_step * 0.005 };
        let (sxy, slens) = flat(&seed);
        let (bxy, blens) = flat(&boundary);
        let want = take_paths(unsafe {
            cz::cz_propagate_wave(
                sxy.as_ptr(),
                slens.as_ptr(),
                slens.len() as i32,
                bxy.as_ptr(),
                blens.as_ptr(),
                blens.len() as i32,
                initial_step,
                other_step,
                num_other_steps,
                arc_tolerance,
                shortest_edge_length,
            )
        });
        tally.add(points(&want));
        let mut co = cl::ClipperOffset::default();
        co.arc_tolerance = arc_tolerance;
        co.shortest_edge_length = shortest_edge_length;
        let got = slicer::region_expansion::propagate_wave_from_boundary(
            &mut co,
            &seed,
            &boundary,
            initial_step,
            other_step,
            num_other_steps,
        );
        assert_eq!(
            got,
            want,
            "cz_propagate_wave case {i} initial_step={initial_step:?} other_step={other_step:?} num_other_steps={num_other_steps} arc_tolerance={arc_tolerance:?} shortest_edge_length={shortest_edge_length:?}\n  seed: {}\n  boundary: {}",
            show_paths(&seed),
            show_paths(&boundary)
        );
    }
}

// ---------------------------------------------------------------------------
// The wrappers of the engine
// ---------------------------------------------------------------------------

fn polygons(paths: &Paths) -> Vec<Polygon> {
    paths.iter().map(|p| Polygon::from_points(p.iter().map(|q| Point::new(q.x as i64, q.y as i64)).collect())).collect()
}

fn polygon_data(polygons: &[Polygon]) -> Vec<Vec<(i64, i64)>> {
    polygons.iter().map(|p| p.points().iter().map(|q| (q.x, q.y)).collect()).collect()
}

fn path_data(paths: &Paths) -> Vec<Vec<(i64, i64)>> {
    paths.iter().map(|p| p.iter().map(|q| (q.x as i64, q.y as i64)).collect()).collect()
}

/// The paths the wrappers flatten out of polygons: the ones with points.
fn non_empty(paths: &Paths) -> Paths {
    paths.iter().filter(|p| !p.is_empty()).cloned().collect()
}

/// The wrappers the engine calls, which take the Rust routines while the gate is on, against
/// the shim called the way the wrappers call it and decoded the way they decode it.
#[test]
fn wrappers_match_shim() {
    if !cu::clipper_rust() {
        eprintln!("CLIPPER_RUST=0: the wrappers call the shim; nothing to compare");
        return;
    }
    for i in 0..cases(8_000) {
        let mut s = case_seed(18, i);
        let sc = scene(&mut s);
        let subject = soup(sc, &mut s);
        let clip = soup(sc, &mut s);
        let subject_polygons = polygons(&subject);
        let clip_polygons = polygons(&clip);
        let (sxy, slens) = flat(&non_empty(&subject));
        let (cxy, clens) = flat(&non_empty(&clip));
        let snum = slens.len() as i32;
        let cnum = clens.len() as i32;
        let input = format!("case {i}\n  subject: {}\n  clip: {}", show_paths(&subject), show_paths(&clip));

        // union_flat_clib: cz_union_flat
        let want = if snum == 0 { vec![] } else { path_data(&take_paths(unsafe { cz::cz_union_flat(sxy.as_ptr(), slens.as_ptr(), snum) })) };
        assert_eq!(polygon_data(&cu::union_flat_clib(&subject_polygons)), want, "union_flat_clib {input}");

        // union_ex_clib: cz_union_ex
        let fill_type = below(4, &mut s) as i32;
        let want = if snum == 0 { vec![] } else { take_grouped(unsafe { cz::cz_union_ex(sxy.as_ptr(), slens.as_ptr(), snum, fill_type) }) };
        assert_eq!(ex_data(&cu::union_ex_clib(&subject_polygons, fill_type)), want, "union_ex_clib fill_type={fill_type} {input}");

        // polytree_ex_clib: cz_union_polytree (unless CLIB_POLYTREE_ONE_PASS=0)
        if std::env::var("CLIB_POLYTREE_ONE_PASS").map(|v| v != "0").unwrap_or(true) {
            let want = if snum == 0 { vec![] } else { take_grouped(unsafe { cz::cz_union_polytree(sxy.as_ptr(), slens.as_ptr(), snum) }) };
            assert_eq!(ex_data(&cu::polytree_ex_clib(&subject_polygons)), want, "polytree_ex_clib {input}");
        }

        // union_safety_offset_ex_clib: cz_union_ex_safety
        let want = if snum == 0 { vec![] } else { take_grouped(unsafe { cz::cz_union_ex_safety(sxy.as_ptr(), slens.as_ptr(), snum) }) };
        assert_eq!(ex_data(&cu::union_safety_offset_ex_clib(&subject_polygons)), want, "union_safety_offset_ex_clib {input}");

        // simplify_polygons_clib: cz_simplify_polygons
        let want = if snum == 0 { vec![] } else { path_data(&take_paths(unsafe { cz::cz_simplify_polygons(sxy.as_ptr(), slens.as_ptr(), snum, fill_type) })) };
        assert_eq!(polygon_data(&cu::simplify_polygons_clib(&subject_polygons, fill_type)), want, "simplify_polygons_clib fill_type={fill_type} {input}");

        // diff_polygons_raw_clib: cz_difference_closed
        let want = if snum == 0 {
            vec![]
        } else {
            path_data(&take_paths(unsafe {
                cz::cz_difference_closed(sxy.as_ptr(), slens.as_ptr(), snum, ptr_or_null(&cxy), ptr_or_null(&clens), cnum)
            }))
        };
        assert_eq!(polygon_data(&cu::diff_polygons_raw_clib(&subject_polygons, &clip_polygons)), want, "diff_polygons_raw_clib {input}");

        // intersection_polygons_raw_clib: cz_intersection_closed
        let want = if snum == 0 || clip_polygons.is_empty() {
            vec![]
        } else {
            path_data(&take_paths(unsafe {
                cz::cz_intersection_closed(sxy.as_ptr(), slens.as_ptr(), snum, ptr_or_null(&cxy), ptr_or_null(&clens), cnum)
            }))
        };
        assert_eq!(
            polygon_data(&cu::intersection_polygons_raw_clib(&subject_polygons, &clip_polygons)),
            want,
            "intersection_polygons_raw_clib {input}"
        );

        // offset_polygons_clib_ml: cz_offset_paths; offset_ex_polygons_clib: cz_offset_ex_paths
        let d = delta(sc, &mut s);
        let (jt, limit) = join(d, &mut s);
        let join_type = match jt {
            1 => cu::OffsetJoinType::Round,
            2 => cu::OffsetJoinType::Square,
            _ => cu::OffsetJoinType::Miter,
        };
        let want = if snum == 0 { vec![] } else { path_data(&take_paths(unsafe { cz::cz_offset_paths(sxy.as_ptr(), slens.as_ptr(), snum, d, jt, limit) })) };
        assert_eq!(
            polygon_data(&cu::offset_polygons_clib_ml(&subject_polygons, d, join_type, limit)),
            want,
            "offset_polygons_clib_ml delta={d:?} join_type={jt} miter_limit={limit:?} {input}"
        );
        let want = if snum == 0 { vec![] } else { take_grouped(unsafe { cz::cz_offset_ex_paths(sxy.as_ptr(), slens.as_ptr(), snum, d, jt, 3.0) }) };
        assert_eq!(
            ex_data(&cu::offset_ex_polygons_clib(&subject_polygons, d, join_type)),
            want,
            "offset_ex_polygons_clib delta={d:?} join_type={jt} {input}"
        );

        // offset_expolygons_clib_raw_scaled: cz_offset_expolygon per expolygon, and
        // offset2_ex_clib_miter: cz_offset2_ex
        let expolygons = expolygon_soup(sc, &mut s);
        let engine_expolygons: Vec<ExPolygon> = expolygons
            .iter()
            .map(|(c, h)| {
                let mut e = ExPolygon::new(polygons(&vec![c.clone()]).remove(0));
                e.holes = polygons(h);
                e
            })
            .collect();
        let mut want: Vec<Vec<(i64, i64)>> = Vec::new();
        for (c, h) in &expolygons {
            if c.is_empty() {
                continue;
            }
            let (cxy2, _) = flat(&vec![c.clone()]);
            let (hxy, hlens) = flat(h);
            want.extend(path_data(&take_paths(unsafe {
                cz::cz_offset_expolygon(cxy2.as_ptr(), c.len() as i32, ptr_or_null(&hxy), ptr_or_null(&hlens), h.len() as i32, d, jt, 3.0)
            })));
        }
        assert_eq!(
            polygon_data(&cu::offset_expolygons_clib_raw_scaled(&engine_expolygons, d, join_type)),
            want,
            "offset_expolygons_clib_raw_scaled delta={d:?} join_type={jt} case {i}\n  expolygons: {expolygons:?}"
        );
        let (d1_mm, d2_mm) = (d * 0.00001, -d * 0.00001);
        let (xy, lens, is_hole) = grouped_layout(&expolygons);
        let want = if lens.is_empty() {
            vec![]
        } else {
            take_grouped(unsafe {
                cz::cz_offset2_ex(
                    xy.as_ptr(),
                    lens.as_ptr(),
                    is_hole.as_ptr(),
                    lens.len() as i32,
                    ((d1_mm / 0.00001_f64) as f32) as f64,
                    ((d2_mm / 0.00001_f64) as f32) as f64,
                    jt,
                    limit,
                )
            })
        };
        assert_eq!(
            ex_data(&cu::offset2_ex_clib_miter(&engine_expolygons, d1_mm, d2_mm, join_type, limit)),
            want,
            "offset2_ex_clib_miter delta1_mm={d1_mm:?} delta2_mm={d2_mm:?} join_type={jt} miter_limit={limit:?} case {i}\n  expolygons: {expolygons:?}"
        );

        // intersection_pl_clib (clip: expolygons) and diff_pl_clib (clip: polygons)
        let lines = polylines(&clip, sc, 1, &mut s);
        let engine_lines: Vec<Polyline> = lines
            .iter()
            .map(|p| Polyline::from_points(p.iter().map(|q| Point::new(q.x as i64, q.y as i64)).collect()))
            .collect();
        let mut ex_paths = Paths::new();
        for (c, h) in &expolygons {
            ex_paths.push(c.clone());
            ex_paths.extend(h.iter().cloned());
        }
        let (lxy2, llens2) = flat(&lines.iter().filter(|p| p.len() >= 2).cloned().collect());
        let (exy, elens) = flat(&ex_paths);
        let want: Vec<Vec<(i64, i64)>> = if llens2.is_empty() || elens.is_empty() {
            vec![]
        } else {
            path_data(&take_paths(unsafe {
                cz::cz_intersection_pl(lxy2.as_ptr(), llens2.as_ptr(), llens2.len() as i32, exy.as_ptr(), elens.as_ptr(), elens.len() as i32)
            }))
            .into_iter()
            .filter(|p| p.len() >= 2)
            .collect()
        };
        let got: Vec<Vec<(i64, i64)>> = cu::intersection_pl_clib(&engine_lines, &engine_expolygons)
            .iter()
            .map(|p| p.points().iter().map(|q| (q.x, q.y)).collect())
            .collect();
        assert_eq!(got, want, "intersection_pl_clib case {i}\n  lines: {}\n  expolygons: {expolygons:?}", show_paths(&lines));

        let (lxy, llens) = flat(&lines);
        let want: Vec<Vec<(i64, i64)>> = if llens.is_empty() {
            vec![]
        } else {
            path_data(&take_paths(unsafe {
                cz::cz_diff_pl(lxy.as_ptr(), llens.as_ptr(), llens.len() as i32, ptr_or_null(&cxy), ptr_or_null(&clens), cnum)
            }))
        };
        let got: Vec<Vec<(i64, i64)>> = cu::diff_pl_clib(&engine_lines, &clip_polygons)
            .iter()
            .map(|p| p.points().iter().map(|q| (q.x, q.y)).collect())
            .collect();
        assert_eq!(got, want, "diff_pl_clib lines: {} {input}", show_paths(&lines));
    }
}
