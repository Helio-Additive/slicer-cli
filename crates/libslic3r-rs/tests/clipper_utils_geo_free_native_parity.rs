//! The GEO_CLIPPER_CLIB arms of `src/clipper_utils.rs` (R910) against the ClipperUtils.cpp
//! functions they follow: the helpers that ran on geo-clipper until R910 (`union`, `xor`,
//! `offset_polyline`, `offset_polyline_miter`, `offset_polygon`, `offset_polygons`,
//! `offset_polygons_round`, `offset_expolygon`, `offset_expolygons`,
//! `offset_expolygons_miter_limit`, `offset_expolygons_round`, `polygons_overlap` through
//! `intersection_base`) and the geo arms of `union_ex`, `union_polygons_ex`, `intersection` and
//! `difference`, which now take their ClipperLib arm.
//!
//! The reference is the pin's ClipperUtils.cpp text, copied into the shim of `clipper-z-sys`
//! (namespace `cu_pin` of clipper_z_shim.cpp) and compiled on the pin's clipper.cpp: the
//! `cz_ref_cu_*` exports. The same input goes through the engine's helper and through the C++
//! function; the two results must be the same polygons in the same order, each with the same
//! points from the same start vertex (for ExPolygons, the same contours with the same holes).
//!
//! The closed offsets of the engine take millimetres; the C++ receives `float(delta_mm *
//! SCALING_FACTOR)`, which is what the shim's `float` parameter makes of the double handed to
//! it. The polyline offsets take scaled units, as the C++ does. Round joins take an arc
//! tolerance in millimetres, which the C++ receives scaled, as a double.
//!
//! Inputs: rectangles, convex polygons, stars, random (self-intersecting) polygons, rectilinear
//! outlines with collinear and repeated points, tiny polygons, slivers, degenerate and empty
//! rings, polygons with holes (inside the contour and across it), several shapes overlapping,
//! touching or apart (coordinates snapped to a grid), an operand equal to the other one or
//! shifted by one grid step, empty operands; from tens of units to 2e7. Offsets of both signs
//! and zero, f32-rounded and not, from a fraction of a unit to more than the shape, with the
//! three join types, several miter limits and arc tolerances. Polylines of no, one, two and more
//! points, crossing themselves, along a line, closed.
//!
//! Every case is made from its own seed; a failure prints the case and its input.
//! `GEO_FREE_PARITY_CASES=<n>` sets the number of cases of every test.

use clipper_z_sys as cz;
use slicer::clipper::{self as cl, IntPoint, Path, Paths};
use slicer::clipper_utils::{self as cu, OffsetJoinType};
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
    match below(17, s) {
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
        // all points on one line (no area)
        15 => {
            let (dx, dy) = (r(radius, s), r(radius, s));
            for _ in 0..3 + below(3, s) {
                let t = unit(s) * 2.0 - 1.0;
                p.push(pt(cx + (dx as f64 * t) as i64, cy + (dy as f64 * t) as i64, Scene { size: sc.size, grid: 1 }));
            }
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

/// A contour (counter-clockwise most of the time) and up to three holes (clockwise most of
/// the time) inside or across it.
fn shape_with_holes(cx: i64, cy: i64, radius: i64, sc: Scene, s: &mut u64) -> (Path, Paths) {
    let raw = shape(cx, cy, radius, sc, s);
    let contour = if below(8, s) == 0 { raw } else { oriented(raw, true) };
    let mut holes = Paths::new();
    if below(3, s) != 0 {
        for _ in 0..1 + below(3, s) {
            let hr = (radius / (2 + below(5, s) as i64)).max(2);
            let spread = if below(5, s) == 0 { radius } else { radius / 2 };
            let raw = shape(cx + r(spread.max(1), s), cy + r(spread.max(1), s), hr, sc, s);
            holes.push(if below(8, s) == 0 { raw } else { oriented(raw, false) });
        }
    }
    (contour, holes)
}

/// Expolygons: contours with their holes, overlapping, touching or apart; sometimes none.
fn expolygon_soup(sc: Scene, s: &mut u64) -> Vec<(Path, Paths)> {
    let n = match below(12, s) {
        0 => 0,
        1 | 2 | 3 | 4 => 1,
        5 | 6 | 7 => 2,
        _ => 3 + below(3, s),
    };
    (0..n)
        .map(|_| {
            let radius = (sc.size / (1 + below(6, s) as i64)).max(2);
            shape_with_holes(r(sc.size, s), r(sc.size, s), radius, sc, s)
        })
        .collect()
}

/// A second operand for a boolean: another soup, the same one, or the same one shifted by a
/// grid step (edges on edges).
fn second_operand(first: &[(Path, Paths)], sc: Scene, s: &mut u64) -> Vec<(Path, Paths)> {
    match below(8, s) {
        0 => first.to_vec(),
        1 => {
            let (dx, dy) = if below(2, s) == 0 { (sc.grid as i32, 0) } else { (0, sc.grid as i32) };
            let shift = |p: &Path| p.iter().map(|q| IntPoint::new(q.x + dx, q.y + dy)).collect::<Path>();
            first.iter().map(|(c, h)| (shift(c), h.iter().map(shift).collect())).collect()
        }
        _ => expolygon_soup(sc, s),
    }
}

/// Raw closed polygons (each ring of a soup of expolygons, as a polygon).
fn polygon_soup(sc: Scene, s: &mut u64) -> Paths {
    let mut out = Paths::new();
    for (c, h) in expolygon_soup(sc, s) {
        out.push(c);
        if below(2, s) == 0 {
            out.extend(h);
        }
    }
    out
}

/// An open polyline: random, along a line (collinear points), closed, short.
fn polyline(sc: Scene, s: &mut u64) -> Path {
    let mut p = Path::new();
    match below(8, s) {
        0 => {
            for _ in 0..below(3, s) {
                p.push(pt(r(sc.size, s), r(sc.size, s), sc));
            }
        }
        1 => {
            let (ax, ay) = (r(sc.size, s), r(sc.size, s));
            let (dx, dy) = (r(sc.size, s), r(sc.size, s));
            for _ in 0..3 + below(4, s) {
                let t = unit(s) * 2.0 - 0.5;
                p.push(pt(ax + (dx as f64 * t) as i64, ay + (dy as f64 * t) as i64, sc));
            }
        }
        2 => {
            for _ in 0..3 + below(4, s) {
                p.push(pt(r(sc.size, s), r(sc.size, s), sc));
            }
            let first = p[0];
            p.push(first);
        }
        3 => {
            let c = shape(r(sc.size, s), r(sc.size, s), (sc.size / 2).max(2), sc, s);
            p = c;
        }
        _ => {
            for _ in 0..2 + below(8, s) {
                p.push(pt(r(sc.size, s), r(sc.size, s), sc));
            }
        }
    }
    p
}

/// A scaled offset for a scene: from a fraction of a unit to more than the scene, either sign
/// or zero, f32-rounded most of the time. `max` caps its magnitude.
fn delta_scaled(sc: Scene, max: f64, s: &mut u64) -> f64 {
    let magnitude = match below(11, s) {
        0 => 10.0,
        1 => 0.3 + unit(s),
        2 => 1.0 + unit(s) * 30.0,
        3 | 4 => sc.size as f64 * (0.001 + 0.02 * unit(s)),
        5 | 6 => sc.size as f64 * (0.02 + 0.2 * unit(s)),
        7 => sc.size as f64 * (0.2 + 0.5 * unit(s)),
        8 => sc.size as f64 * (0.7 + 1.5 * unit(s)),
        9 => [5_000.0, 10_000.0, 20_000.0, 45_000.0, 100_000.0][below(5, s) as usize],
        _ => (sc.grid * (1 + below(4, s) as i64)) as f64,
    };
    let magnitude = magnitude.min(max);
    let d = if below(2, s) == 0 { magnitude } else { -magnitude };
    match below(12, s) {
        0 => 0.0,
        1 => d,
        _ => d as f32 as f64,
    }
}

/// A millimetre offset, as the engine passes them: a scaled offset over SCALING_FACTOR, or a
/// round number of millimetres.
fn delta_mm(sc: Scene, join: OffsetJoinType, s: &mut u64) -> f64 {
    // Round joins with the default arc tolerance of 3 units: keep the arcs within a few
    // hundred points.
    let max = if join == OffsetJoinType::Round { 2.0e5 } else { 1.0e9 };
    if below(6, s) == 0 {
        let mm: f64 = [0.0001, 0.01, 0.05, 0.1, 0.2, 0.45, 1.0, 2.0][below(8, s) as usize];
        let mm = mm.min(max / 1e5);
        return if below(2, s) == 0 { mm } else { -mm };
    }
    delta_scaled(sc, max, s) / 1e5
}

fn join_type(s: &mut u64) -> OffsetJoinType {
    [OffsetJoinType::Miter, OffsetJoinType::Round, OffsetJoinType::Square][below(3, s) as usize]
}

/// The shim's code of a join type: 0=jtMiter, 1=jtRound, 2=jtSquare.
fn join_code(j: OffsetJoinType) -> i32 {
    match j {
        OffsetJoinType::Miter => 0,
        OffsetJoinType::Round => 1,
        OffsetJoinType::Square => 2,
    }
}

fn miter_limit(s: &mut u64) -> f64 {
    [3.0, 1.2, 2.0, 5.0, 1.0, 0.0, 2.5][below(7, s) as usize]
}

/// An arc tolerance in millimetres that keeps the arcs of the given offset within a few
/// hundred points.
fn arc_tolerance_mm(delta_mm: f64, s: &mut u64) -> f64 {
    let a: f64 = [0.01, 0.00003, 0.001, 0.0001, 0.05][below(5, s) as usize];
    a.max(delta_mm.abs() * 2e-5)
}

// ---------------------------------------------------------------------------
// The engine's types and the shim's layout
// ---------------------------------------------------------------------------

fn polygon(p: &Path) -> Polygon {
    Polygon::from_points(p.iter().map(|q| Point::new(q.x as i64, q.y as i64)).collect())
}

fn expolygon(e: &(Path, Paths)) -> ExPolygon {
    ExPolygon::with_holes(polygon(&e.0), e.1.iter().map(polygon).collect())
}

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

/// Expolygons in the shim's layout: the paths and, per path, 0 for a contour, 1 for a hole.
fn flat_ex(expolygons: &[(Path, Paths)]) -> (Vec<i32>, Vec<i32>, Vec<i32>) {
    let mut paths = Paths::new();
    let mut is_hole = Vec::new();
    for (c, h) in expolygons {
        paths.push(c.clone());
        is_hole.push(0);
        for hole in h {
            paths.push(hole.clone());
            is_hole.push(1);
        }
    }
    let (xy, lens) = flat(&paths);
    (xy, lens, is_hole)
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

type Ring = Vec<(i64, i64)>;
/// An expolygon as plain data: the contour and the holes.
type Ex = (Ring, Vec<Ring>);

/// A grouped shim result: z = 0 on a contour, which starts an expolygon; z = 1 on a hole of
/// the expolygon before it.
fn take_grouped(raw: cz::CzZPaths) -> Vec<Ex> {
    let mut out: Vec<Ex> = Vec::new();
    for path in take(raw) {
        let is_hole = path.first().map(|q| q.2).unwrap_or(0);
        let ring: Ring = path.iter().map(|q| (q.0 as i64, q.1 as i64)).collect();
        if is_hole == 0 {
            out.push((ring, Vec::new()));
        } else {
            out.last_mut().expect("a hole before any contour").1.push(ring);
        }
    }
    out
}

fn take_rings(raw: cz::CzZPaths) -> Vec<Ring> {
    take(raw).into_iter().map(|p| p.into_iter().map(|q| (q.0 as i64, q.1 as i64)).collect()).collect()
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

fn rings_of(polygons: &[Polygon]) -> Vec<Ring> {
    polygons.iter().map(|p| p.points().iter().map(|q| (q.x, q.y)).collect()).collect()
}

fn show_path(p: &Path) -> String {
    let mut out = String::from("vec![");
    for q in p {
        let _ = write!(out, "({},{}),", q.x, q.y);
    }
    out.push(']');
    out
}

fn show_paths(ps: &Paths) -> String {
    let mut out = String::from("vec![");
    for p in ps {
        out.push_str(&show_path(p));
        out.push(',');
    }
    out.push(']');
    out
}

fn show_ex(exs: &[(Path, Paths)]) -> String {
    let mut out = String::from("vec![");
    for (c, h) in exs {
        let _ = write!(out, "({}, {}),", show_path(c), show_paths(h));
    }
    out.push(']');
    out
}

fn ex_points(expolygons: &[Ex]) -> usize {
    expolygons.iter().map(|e| e.0.len() + e.1.iter().map(|h| h.len()).sum::<usize>()).sum()
}

/// Number of cases of a test: `GEO_FREE_PARITY_CASES` or the given default.
fn cases(default: u64) -> u64 {
    std::env::var("GEO_FREE_PARITY_CASES").ok().and_then(|v| v.parse().ok()).unwrap_or(default)
}

/// The arms under test are the gate's on arm; with `GEO_CLIPPER_CLIB=0` the helpers run on
/// geo-clipper and there is nothing to compare, unless `GEO_FREE_PARITY_COMPARE_GEO=1` asks
/// to compare the geo arms all the same (they fail: that is what the gate fixes).
fn gate_on() -> bool {
    if std::env::var("GEO_FREE_PARITY_COMPARE_GEO").map(|v| v == "1").unwrap_or(false) {
        return true;
    }
    if std::env::var("GEO_CLIPPER_CLIB").map(|v| v == "0").unwrap_or(false) {
        eprintln!("GEO_CLIPPER_CLIB=0: the helpers run on geo-clipper; nothing compared");
        return false;
    }
    true
}

/// How many cases a test compared and how many of them had a result that is not empty; a
/// test whose inputs hardly ever give a result compares nothing, so one case in ten at least
/// has to give one. `GEO_FREE_PARITY_STATS=<file>` appends a line per test to that file.
struct Tally {
    name: &'static str,
    cases: u64,
    non_empty: u64,
    points: u64,
}

impl Tally {
    fn new(name: &'static str) -> Self {
        Self { name, cases: 0, non_empty: 0, points: 0 }
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
        if let Ok(path) = std::env::var("GEO_FREE_PARITY_STATS") {
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

// ---------------------------------------------------------------------------
// Booleans
// ---------------------------------------------------------------------------

/// `cz_ref_cu_boolean_ex`: 0 intersection_ex, 1 union_ex(poly1, poly2), 2 diff_ex, 3 xor_ex.
fn cpp_boolean(subject: &[(Path, Paths)], clip: &[(Path, Paths)], op: i32) -> Vec<Ex> {
    let (sxy, slens, sholes) = flat_ex(subject);
    let (cxy, clens, choles) = flat_ex(clip);
    take_grouped(unsafe {
        cz::cz_ref_cu_boolean_ex(
            sxy.as_ptr(),
            slens.as_ptr(),
            sholes.as_ptr(),
            slens.len() as i32,
            cxy.as_ptr(),
            clens.as_ptr(),
            choles.as_ptr(),
            clens.len() as i32,
            op,
        )
    })
}

/// `union` against `union_ex(const ExPolygons&, const ExPolygons&)` (ClipperUtils.cpp:824-831).
#[test]
fn union_matches_cpp() {
    if !gate_on() {
        return;
    }
    let mut tally = Tally::new("union");
    for i in 0..cases(12_000) {
        let mut s = case_seed(101, i);
        let sc = scene(&mut s);
        let a = expolygon_soup(sc, &mut s);
        let b = second_operand(&a, sc, &mut s);
        let want = cpp_boolean(&a, &b, 1);
        tally.add(ex_points(&want));
        let ea: Vec<ExPolygon> = a.iter().map(expolygon).collect();
        let eb: Vec<ExPolygon> = b.iter().map(expolygon).collect();
        let got = ex_data(&cu::union(&ea, &eb));
        assert_eq!(got, want, "union case {i}\n  subject: {}\n  clip: {}", show_ex(&a), show_ex(&b));
    }
}

/// `xor` against `xor_ex(const ExPolygons&, const ExPolygons&)` (ClipperUtils.cpp:836-838).
#[test]
fn xor_matches_cpp() {
    if !gate_on() {
        return;
    }
    let mut tally = Tally::new("xor");
    for i in 0..cases(12_000) {
        let mut s = case_seed(102, i);
        let sc = scene(&mut s);
        let a = expolygon_soup(sc, &mut s);
        let b = second_operand(&a, sc, &mut s);
        let want = cpp_boolean(&a, &b, 3);
        tally.add(ex_points(&want));
        let ea: Vec<ExPolygon> = a.iter().map(expolygon).collect();
        let eb: Vec<ExPolygon> = b.iter().map(expolygon).collect();
        let got = ex_data(&cu::xor(&ea, &eb));
        assert_eq!(got, want, "xor case {i}\n  subject: {}\n  clip: {}", show_ex(&a), show_ex(&b));
    }
}

/// `intersection` against `intersection_ex(const ExPolygons&, const ExPolygons&)`
/// (ClipperUtils.cpp:802-803), and `polygons_overlap` (`intersection_base`) against whether
/// that is empty.
#[test]
fn intersection_matches_cpp() {
    if !gate_on() {
        return;
    }
    let mut tally = Tally::new("intersection");
    for i in 0..cases(12_000) {
        let mut s = case_seed(103, i);
        let sc = scene(&mut s);
        let a = expolygon_soup(sc, &mut s);
        let b = second_operand(&a, sc, &mut s);
        let want = cpp_boolean(&a, &b, 0);
        tally.add(ex_points(&want));
        let ea: Vec<ExPolygon> = a.iter().map(expolygon).collect();
        let eb: Vec<ExPolygon> = b.iter().map(expolygon).collect();
        let got = ex_data(&cu::intersection(&ea, &eb));
        assert_eq!(got, want, "intersection case {i}\n  subject: {}\n  clip: {}", show_ex(&a), show_ex(&b));
        assert_eq!(
            cu::polygons_overlap(&ea, &eb),
            !want.is_empty(),
            "polygons_overlap case {i}\n  subject: {}\n  clip: {}",
            show_ex(&a),
            show_ex(&b)
        );
    }
}

/// `difference` against `diff_ex(const ExPolygons&, const ExPolygons&)`
/// (ClipperUtils.cpp:755-756), for a clip that is not empty: with an empty clip the wrapper
/// returns its subject as it is (an early return of every arm), where the C++ unions it.
#[test]
fn difference_matches_cpp() {
    if !gate_on() {
        return;
    }
    let mut tally = Tally::new("difference");
    for i in 0..cases(12_000) {
        let mut s = case_seed(104, i);
        let sc = scene(&mut s);
        let a = expolygon_soup(sc, &mut s);
        let b = second_operand(&a, sc, &mut s);
        if b.is_empty() {
            continue;
        }
        let want = cpp_boolean(&a, &b, 2);
        tally.add(ex_points(&want));
        let ea: Vec<ExPolygon> = a.iter().map(expolygon).collect();
        let eb: Vec<ExPolygon> = b.iter().map(expolygon).collect();
        let got = ex_data(&cu::difference(&ea, &eb));
        assert_eq!(got, want, "difference case {i}\n  subject: {}\n  clip: {}", show_ex(&a), show_ex(&b));
    }
}

/// `union_ex` against `union_ex(const ExPolygons&)` (ClipperUtils.cpp:815-816) for two
/// expolygons or more (a single one is returned as it is, an early return of every arm), and
/// `union_polygons_ex` against `union_ex(const Polygons&)` (ClipperUtils.cpp:813-814), whose
/// PolygonsProvider yields the paths the ExPolygonsProvider of hole-less expolygons does.
#[test]
fn union_ex_matches_cpp() {
    if !gate_on() {
        return;
    }
    let mut tally = Tally::new("union_ex");
    for i in 0..cases(12_000) {
        let mut s = case_seed(105, i);
        let sc = scene(&mut s);
        let a = expolygon_soup(sc, &mut s);
        if a.len() >= 2 {
            let want = cpp_boolean(&a, &[], 1);
            tally.add(ex_points(&want));
            let ea: Vec<ExPolygon> = a.iter().map(expolygon).collect();
            let got = ex_data(&cu::union_ex(&ea));
            assert_eq!(got, want, "union_ex case {i}\n  subject: {}", show_ex(&a));
        }
        let polys = polygon_soup(sc, &mut s);
        let as_ex: Vec<(Path, Paths)> = polys.iter().map(|p| (p.clone(), Paths::new())).collect();
        let want = cpp_boolean(&as_ex, &[], 1);
        let got = ex_data(&cu::union_polygons_ex(&polys.iter().map(polygon).collect::<Vec<_>>()));
        assert_eq!(got, want, "union_polygons_ex case {i}\n  subject: {}", show_paths(&polys));
    }
}

// ---------------------------------------------------------------------------
// Offsets
// ---------------------------------------------------------------------------

/// `cz_ref_cu_offset_polyline`: offset(const Polyline&, ...), end type etOpenButt.
fn cpp_offset_polyline(path: &Path, delta: f64, join: i32, miter_limit: f64) -> Vec<Ring> {
    let (xy, _) = flat(&vec![path.clone()]);
    take_rings(unsafe { cz::cz_ref_cu_offset_polyline(xy.as_ptr(), path.len() as i32, delta, join, miter_limit, 2) })
}

/// A positive scaled offset for a polyline (the C++ asserts `delta > 0`).
fn polyline_delta(sc: Scene, s: &mut u64) -> f64 {
    loop {
        let d = delta_scaled(sc, 1.0e9, s).abs();
        if d > 0.0 {
            return d;
        }
    }
}

/// `offset_polyline` against `offset(const Polyline&, delta)` with its defaults
/// (ClipperUtils.cpp:418-419, ClipperUtils.hpp:346: jtSquare, miter limit 0, etOpenButt).
#[test]
fn offset_polyline_matches_cpp() {
    if !gate_on() {
        return;
    }
    let mut tally = Tally::new("offset_polyline");
    for i in 0..cases(12_000) {
        let mut s = case_seed(106, i);
        let sc = scene(&mut s);
        let path = polyline(sc, &mut s);
        let delta = polyline_delta(sc, &mut s);
        let want = cpp_offset_polyline(&path, delta, 2, 0.0);
        tally.add(want.iter().map(|r| r.len()).sum());
        let pl = Polyline::from_points(path.iter().map(|q| Point::new(q.x as i64, q.y as i64)).collect());
        let got = rings_of(&cu::offset_polyline(&pl, delta));
        assert_eq!(got, want, "offset_polyline case {i} delta {delta}\n  polyline: {}", show_path(&path));
    }
}

/// `offset_polyline_miter` against `offset(const Polyline&, delta, jtMiter, miterLimit)`
/// (ClipperUtils.cpp:418-419, etOpenButt).
#[test]
fn offset_polyline_miter_matches_cpp() {
    if !gate_on() {
        return;
    }
    let mut tally = Tally::new("offset_polyline_miter");
    for i in 0..cases(12_000) {
        let mut s = case_seed(107, i);
        let sc = scene(&mut s);
        let path = polyline(sc, &mut s);
        let delta = polyline_delta(sc, &mut s);
        let ml = miter_limit(&mut s);
        let want = cpp_offset_polyline(&path, delta, 0, ml);
        tally.add(want.iter().map(|r| r.len()).sum());
        let pl = Polyline::from_points(path.iter().map(|q| Point::new(q.x as i64, q.y as i64)).collect());
        let got = rings_of(&cu::offset_polyline_miter(&pl, delta, ml));
        assert_eq!(got, want, "offset_polyline_miter case {i} delta {delta} ml {ml}\n  polyline: {}", show_path(&path));
    }
}

/// `cz_ref_cu_offset_ex_polygons`: offset_ex(const Polygons&, ...).
fn cpp_offset_ex_polygons(paths: &Paths, delta_scaled: f64, join: i32, miter_limit: f64) -> Vec<Ex> {
    let (xy, lens) = flat(paths);
    take_grouped(unsafe {
        cz::cz_ref_cu_offset_ex_polygons(xy.as_ptr(), lens.as_ptr(), lens.len() as i32, delta_scaled, join, miter_limit)
    })
}

/// `cz_ref_cu_offset_ex_expolygons`: offset_ex(const ExPolygon&, ...) of the first (which 0) or
/// offset_ex(const ExPolygons&, ...) (which 1).
fn cpp_offset_ex_expolygons(expolygons: &[(Path, Paths)], delta_scaled: f64, join: i32, miter_limit: f64, which: i32) -> Vec<Ex> {
    let (xy, lens, is_hole) = flat_ex(expolygons);
    take_grouped(unsafe {
        cz::cz_ref_cu_offset_ex_expolygons(
            xy.as_ptr(),
            lens.as_ptr(),
            is_hole.as_ptr(),
            lens.len() as i32,
            delta_scaled,
            join,
            miter_limit,
            which,
        )
    })
}

/// `offset_polygon` against `offset_ex(const Polygon&, delta, joinType)` (ClipperUtils.hpp:358-364).
#[test]
fn offset_polygon_matches_cpp() {
    if !gate_on() {
        return;
    }
    let mut tally = Tally::new("offset_polygon");
    for i in 0..cases(12_000) {
        let mut s = case_seed(108, i);
        let sc = scene(&mut s);
        let path = shape(r(sc.size, &mut s), r(sc.size, &mut s), (sc.size / (1 + below(4, &mut s) as i64)).max(2), sc, &mut s);
        let join = join_type(&mut s);
        let d = delta_mm(sc, join, &mut s);
        let want = cpp_offset_ex_polygons(&vec![path.clone()], d * 1e5, join_code(join), 3.0);
        tally.add(ex_points(&want));
        let got = ex_data(&cu::offset_polygon(&polygon(&path), d, join));
        assert_eq!(got, want, "offset_polygon case {i} delta_mm {d} {join:?}\n  polygon: {}", show_path(&path));
    }
}

/// `offset_polygons` against `offset_ex(const Polygons&, delta, joinType)` (ClipperUtils.cpp:415-416).
#[test]
fn offset_polygons_matches_cpp() {
    if !gate_on() {
        return;
    }
    let mut tally = Tally::new("offset_polygons");
    for i in 0..cases(12_000) {
        let mut s = case_seed(109, i);
        let sc = scene(&mut s);
        let paths = polygon_soup(sc, &mut s);
        let join = join_type(&mut s);
        let d = delta_mm(sc, join, &mut s);
        let want = cpp_offset_ex_polygons(&paths, d * 1e5, join_code(join), 3.0);
        tally.add(ex_points(&want));
        let polys: Vec<Polygon> = paths.iter().map(polygon).collect();
        let got = ex_data(&cu::offset_polygons(&polys, d, join));
        assert_eq!(got, want, "offset_polygons case {i} delta_mm {d} {join:?}\n  polygons: {}", show_paths(&paths));
    }
}

/// `offset_polygons_round` against `offset_ex(const Polygons&, delta, jtRound, arc tolerance)`
/// (ClipperUtils.cpp:415-416).
#[test]
fn offset_polygons_round_matches_cpp() {
    if !gate_on() {
        return;
    }
    let mut tally = Tally::new("offset_polygons_round");
    for i in 0..cases(12_000) {
        let mut s = case_seed(110, i);
        let sc = scene(&mut s);
        let paths = polygon_soup(sc, &mut s);
        let d = delta_mm(sc, OffsetJoinType::Miter, &mut s);
        let arc = arc_tolerance_mm(d, &mut s);
        let want = cpp_offset_ex_polygons(&paths, d * 1e5, 1, arc * 100_000.0);
        tally.add(ex_points(&want));
        let polys: Vec<Polygon> = paths.iter().map(polygon).collect();
        let got = ex_data(&cu::offset_polygons_round(&polys, d, arc));
        assert_eq!(got, want, "offset_polygons_round case {i} delta_mm {d} arc_mm {arc}\n  polygons: {}", show_paths(&paths));
    }
}

/// `offset_expolygon` against `offset_ex(const ExPolygon&, delta, joinType)` (ClipperUtils.cpp:567-569).
#[test]
fn offset_expolygon_matches_cpp() {
    if !gate_on() {
        return;
    }
    let mut tally = Tally::new("offset_expolygon");
    for i in 0..cases(12_000) {
        let mut s = case_seed(111, i);
        let sc = scene(&mut s);
        let radius = (sc.size / (1 + below(4, &mut s) as i64)).max(2);
        let e = shape_with_holes(r(sc.size, &mut s), r(sc.size, &mut s), radius, sc, &mut s);
        let join = join_type(&mut s);
        let d = delta_mm(sc, join, &mut s);
        let want = cpp_offset_ex_expolygons(std::slice::from_ref(&e), d * 1e5, join_code(join), 3.0, 0);
        tally.add(ex_points(&want));
        let got = ex_data(&cu::offset_expolygon(&expolygon(&e), d, join));
        assert_eq!(got, want, "offset_expolygon case {i} delta_mm {d} {join:?}\n  expolygon: {}", show_ex(std::slice::from_ref(&e)));
    }
}

/// `offset_expolygons` against `offset_ex(const ExPolygons&, delta, joinType)` (ClipperUtils.cpp:570-571).
#[test]
fn offset_expolygons_matches_cpp() {
    if !gate_on() {
        return;
    }
    let mut tally = Tally::new("offset_expolygons");
    for i in 0..cases(12_000) {
        let mut s = case_seed(112, i);
        let sc = scene(&mut s);
        let exs = expolygon_soup(sc, &mut s);
        let join = join_type(&mut s);
        let d = delta_mm(sc, join, &mut s);
        let want = cpp_offset_ex_expolygons(&exs, d * 1e5, join_code(join), 3.0, 1);
        tally.add(ex_points(&want));
        let ee: Vec<ExPolygon> = exs.iter().map(expolygon).collect();
        let got = ex_data(&cu::offset_expolygons(&ee, d, join));
        assert_eq!(got, want, "offset_expolygons case {i} delta_mm {d} {join:?}\n  expolygons: {}", show_ex(&exs));
    }
}

/// `offset_expolygons_miter_limit` against `offset_ex(const ExPolygons&, delta, jtMiter,
/// miterLimit)` (ClipperUtils.cpp:570-571).
#[test]
fn offset_expolygons_miter_limit_matches_cpp() {
    if !gate_on() {
        return;
    }
    let mut tally = Tally::new("offset_expolygons_miter_limit");
    for i in 0..cases(12_000) {
        let mut s = case_seed(113, i);
        let sc = scene(&mut s);
        let exs = expolygon_soup(sc, &mut s);
        let d = delta_mm(sc, OffsetJoinType::Miter, &mut s);
        let ml = miter_limit(&mut s);
        let want = cpp_offset_ex_expolygons(&exs, d * 1e5, 0, ml, 1);
        tally.add(ex_points(&want));
        let ee: Vec<ExPolygon> = exs.iter().map(expolygon).collect();
        let got = ex_data(&cu::offset_expolygons_miter_limit(&ee, d, ml));
        assert_eq!(got, want, "offset_expolygons_miter_limit case {i} delta_mm {d} ml {ml}\n  expolygons: {}", show_ex(&exs));
    }
}

/// `offset_expolygons_round` against `offset_ex(const ExPolygons&, delta, jtRound, arc
/// tolerance)` (ClipperUtils.cpp:570-571).
#[test]
fn offset_expolygons_round_matches_cpp() {
    if !gate_on() {
        return;
    }
    let mut tally = Tally::new("offset_expolygons_round");
    for i in 0..cases(12_000) {
        let mut s = case_seed(114, i);
        let sc = scene(&mut s);
        let exs = expolygon_soup(sc, &mut s);
        let d = delta_mm(sc, OffsetJoinType::Miter, &mut s);
        let arc = arc_tolerance_mm(d, &mut s);
        let want = cpp_offset_ex_expolygons(&exs, d * 1e5, 1, arc * 100_000.0, 1);
        tally.add(ex_points(&want));
        let ee: Vec<ExPolygon> = exs.iter().map(expolygon).collect();
        let got = ex_data(&cu::offset_expolygons_round(&ee, d, arc));
        assert_eq!(got, want, "offset_expolygons_round case {i} delta_mm {d} arc_mm {arc}\n  expolygons: {}", show_ex(&exs));
    }
}

/// `grow` and `shrink`, which are `offset_expolygons` with `+|d|` and `-|d|`.
#[test]
fn grow_shrink_match_cpp() {
    if !gate_on() {
        return;
    }
    let mut tally = Tally::new("grow_shrink");
    for i in 0..cases(6_000) {
        let mut s = case_seed(115, i);
        let sc = scene(&mut s);
        let exs = expolygon_soup(sc, &mut s);
        let join = join_type(&mut s);
        let d = delta_mm(sc, join, &mut s).abs();
        let ee: Vec<ExPolygon> = exs.iter().map(expolygon).collect();
        let want = cpp_offset_ex_expolygons(&exs, d * 1e5, join_code(join), 3.0, 1);
        tally.add(ex_points(&want));
        assert_eq!(ex_data(&cu::grow(&ee, d, join)), want, "grow case {i} d {d} {join:?}\n  expolygons: {}", show_ex(&exs));
        let want = cpp_offset_ex_expolygons(&exs, -d * 1e5, join_code(join), 3.0, 1);
        assert_eq!(ex_data(&cu::shrink(&ee, d, join)), want, "shrink case {i} d {d} {join:?}\n  expolygons: {}", show_ex(&exs));
    }
}
