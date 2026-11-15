//! Medial chart of the conformal infill.
//!
//! C++ Reference:
//! - Fill/FillConformalChart.hpp
//! - Fill/FillConformalChart.cpp
//!
//! Line-by-line port of `Slic3r::FillConformal::MedialChart` and the per-layer
//! generator extraction (`extract_generator`): an open spine or a closed
//! mid-loop of one island, from the medial axis or from an inset, and the 3D
//! SDF-ridge sheet over a stack of islands.
//!
//! Floating point: the C++ program is built with `-ffp-contract=on`, so a
//! scalar `a * b + c` of ONE source expression is a fused multiply-add; those
//! places are written with `mul_add` here, and nowhere else.

// FillConformalChart.cpp:1-18
//   #include "FillConformalChart.hpp"
//   #include "../BoundingBox.hpp"
//   #include "../ClipperUtils.hpp"
//   #include "../Geometry.hpp"
//   #include "../Point.hpp"
//   #include "../Polygon.hpp"
//   #include "../TriangleMesh.hpp"
use std::collections::{BTreeMap, VecDeque};

use crate::clipper_utils::{offset_expolygons_clib_scaled, raw_offset, OffsetJoinType};
use crate::geometry::MedialAxisThickPolyline;
use crate::geometry::{align_to_grid, BoundingBox, ExPolygon, Point, Polygon, Polyline};
use crate::normal_utils::{indexed_triangle_set, Vec3crd, Vec3f};
use crate::Coord;

// FillConformalChart.cpp:20-21 — namespace Slic3r { namespace FillConformal {

/// libslic3r.h:52 — `static constexpr double SCALING_FACTOR = 0.00001;`
const SCALING_FACTOR: f64 = 0.00001;

/// libslic3r.h:60 — `#define scale_(val) ((val) / SCALING_FACTOR)`: a double.
#[inline]
fn scale_(val: f64) -> f64 {
    val / SCALING_FACTOR
}

/// Point.hpp:540 — `scaled<coord_t>(v)`: `coord_t(v / SCALING_FACTOR)`, truncating.
#[inline]
fn scaled_coord(v: f64) -> Coord {
    (v / SCALING_FACTOR) as Coord
}

/// libslic3r.h — `unscale<double>(v)`: `double(v) * double(SCALING_FACTOR)`.
#[inline]
fn unscale_d(v: f64) -> f64 {
    v * SCALING_FACTOR
}

/// Point.hpp:185 — `Point::new_scale(x, y)`: `Point(coord_t(scale_(x)), coord_t(scale_(y)))`.
#[inline]
fn point_new_scale(x: f64, y: f64) -> Point {
    Point::new(scale_(x) as Coord, scale_(y) as Coord)
}

/// `(a - b).cast<double>().norm()` — Eigen, not fused.
#[inline]
fn diff_norm(a: Point, b: Point) -> f64 {
    let dx = (a.x - b.x) as f64;
    let dy = (a.y - b.y) as f64;
    (dx * dx + dy * dy).sqrt()
}

/// `get_extents(const Polygon&)` / `get_extents(const ExPolygon&)` (the contour's box).
#[inline]
fn get_extents_points(points: &[Point]) -> BoundingBox {
    BoundingBox::from_points(points)
}

/// ClipperUtils.cpp:567-569 — `offset_ex(const ExPolygon&, const float delta)`.
fn offset_ex(expolygon: &ExPolygon, delta: f32) -> Vec<ExPolygon> {
    offset_expolygons_clib_scaled(std::slice::from_ref(expolygon), delta as f64, OffsetJoinType::Miter)
}

/// ClipperUtils.cpp:410-411 — `offset(const Polygon&, const float delta)`:
/// `to_polygons(raw_offset(SinglePathProvider(polygon.points), delta, jtMiter, 3.))`.
fn offset_polygon(polygon: &Polygon, delta: f32) -> Vec<Polygon> {
    let path: crate::clipper::Path = polygon
        .points
        .iter()
        .map(|p| crate::clipper::IntPoint::new(p.x as i32, p.y as i32))
        .collect();
    let paths: crate::clipper::Paths = vec![path];
    raw_offset(
        &paths,
        delta as f64,
        crate::clipper::JoinType::jtMiter,
        3.0,
        crate::clipper::EndType::etClosedPolygon,
    )
    .iter()
    .map(|p| Polygon::from_points(p.iter().map(|q| Point::new(q.x as i64, q.y as i64)).collect()))
    .collect()
}

/// One layer's 2D generator: an open spine (sole) or a closed mid-loop (collar).
/// FillConformalChart.hpp:16-22 — `struct LayerGenerator`
#[derive(Debug, Clone, Default)]
pub struct LayerGenerator {
    /// FillConformalChart.hpp:18 — `double z { 0. };`
    pub z: f64,
    /// FillConformalChart.hpp:19 — `bool closed { false };`
    pub closed: bool,
    /// FillConformalChart.hpp:20 — `Polyline polyline;`
    pub polyline: Polyline,
    /// FillConformalChart.hpp:21 — `double length { 0. }; // scaled`
    pub length: f64,
}

/// FillConformalChart.cpp:23-27 — `static Point quantize_pt(const Point &p)`
fn quantize_pt(p: &Point) -> Point {
    // FillConformalChart.cpp:25
    let q: Coord = scaled_coord(0.1);
    // FillConformalChart.cpp:26
    Point::new(align_to_grid(p.x, q), align_to_grid(p.y, q))
}

/// FillConformalChart.cpp:46-51 — `struct Edge` of `stitch_centerline`.
#[derive(Debug, Clone, Copy)]
struct Edge {
    a: i32,
    b: i32,
    idx: usize,
}

/// FillConformalChart.cpp:35-44 — the lambda `id_of` of `stitch_centerline`.
fn id_of(node_id: &mut BTreeMap<(Coord, Coord), i32>, p: &Point) -> i32 {
    let q = quantize_pt(p);
    match node_id.get(&(q.x, q.y)) {
        None => {
            let id = node_id.len() as i32;
            node_id.insert((q.x, q.y), id);
            id
        }
        Some(&id) => id,
    }
}

/// FillConformalChart.cpp:72 — the lambda `other` of `stitch_centerline`.
#[inline]
fn other(e: &Edge, v: i32) -> i32 {
    if e.a == v {
        e.b
    } else {
        e.a
    }
}

/// FillConformalChart.cpp:75-95 — the lambda `walk` of `stitch_centerline`.
fn walk(edges: &[Edge], adj: &[Vec<i32>], used: &mut [u8], start: i32, closed_walk: bool) -> Vec<i32> {
    let mut chain: Vec<i32> = Vec::new();
    let mut v = start;
    loop {
        let mut next_e: i32 = -1;
        for &ei in &adj[v as usize] {
            if used[ei as usize] == 0 {
                next_e = ei;
                break;
            }
        }
        if next_e < 0 {
            break;
        }
        used[next_e as usize] = 1;
        chain.push(next_e);
        v = other(&edges[next_e as usize], v);
        if closed_walk && v == start {
            break;
        }
    }
    chain
}

/// FillConformalChart.cpp:97-114 — the lambda `concat` of `stitch_centerline`.
fn concat(edges: &[Edge], pieces: &[Polyline], chain: &[i32], start: i32, closed: bool) -> Polyline {
    let mut acc = Polyline::new();
    let mut v = start;
    for &ei in chain {
        let e = &edges[ei as usize];
        let mut pl = pieces[e.idx].clone();
        if e.a != v {
            pl.reverse();
        }
        if acc.points.is_empty() {
            acc.points = std::mem::take(&mut pl.points);
        } else if pl.points.len() > 1 {
            acc.points.extend_from_slice(&pl.points[1..]);
        }
        v = other(e, v);
    }
    if closed && !acc.points.is_empty() && acc.points[0] != acc.points[acc.points.len() - 1] {
        let front = acc.points[0];
        acc.points.push(front);
    }
    acc
}

/// FillConformalChart.cpp:140-167 — the lambda `farthest` of `stitch_centerline`.
fn farthest(edges: &[Edge], adj: &[Vec<i32>], elen: &[f64], n: i32, src: i32) -> (i32, Vec<i32>, Vec<i32>) {
    let mut dist: Vec<f64> = vec![-1.0; n as usize];
    let mut parent: Vec<i32> = vec![-1; n as usize];
    let mut parent_e: Vec<i32> = vec![-1; n as usize];
    let mut q: VecDeque<i32> = VecDeque::new();
    dist[src as usize] = 0.0;
    q.push_back(src);
    let mut best_v = src;
    let mut best_d = 0.0_f64;
    while let Some(v) = q.pop_front() {
        if dist[v as usize] > best_d {
            best_d = dist[v as usize];
            best_v = v;
        }
        for &ei in &adj[v as usize] {
            let u = other(&edges[ei as usize], v);
            if dist[u as usize] >= 0.0 {
                continue;
            }
            dist[u as usize] = dist[v as usize] + elen[ei as usize];
            parent[u as usize] = v;
            parent_e[u as usize] = ei;
            q.push_back(u);
        }
    }
    (best_v, parent, parent_e)
}

/// FillConformalChart.cpp:29-199 — `static bool stitch_centerline(const Polylines &pieces, LayerGenerator &out)`
fn stitch_centerline(pieces: &[Polyline], out: &mut LayerGenerator) -> bool {
    // FillConformalChart.cpp:31-32
    if pieces.is_empty() {
        return false;
    }

    // FillConformalChart.cpp:34 — std::map<Point, int> node_id; only find/emplace/size
    // are used, so its iteration order never reaches the output.
    let mut node_id: BTreeMap<(Coord, Coord), i32> = BTreeMap::new();

    // FillConformalChart.cpp:52-53
    let mut edges: Vec<Edge> = Vec::new();
    let mut adj: Vec<Vec<i32>> = Vec::new();

    // FillConformalChart.cpp:55-67
    for i in 0..pieces.len() {
        let pl = &pieces[i];
        if pl.points.len() < 2 {
            continue;
        }
        let a = id_of(&mut node_id, &pl.points[0]);
        let b = id_of(&mut node_id, &pl.points[pl.points.len() - 1]);
        if (adj.len() as i32) <= a.max(b) {
            adj.resize((a.max(b) + 1) as usize, Vec::new());
        }
        let e = edges.len() as i32;
        edges.push(Edge { a, b, idx: i });
        adj[a as usize].push(e);
        adj[b as usize].push(e);
    }
    // FillConformalChart.cpp:68-69
    if edges.is_empty() {
        return false;
    }

    // FillConformalChart.cpp:71
    let n = adj.len() as i32;

    // FillConformalChart.cpp:74
    let mut used: Vec<u8> = vec![0; edges.len()];

    // FillConformalChart.cpp:116-119
    let mut all_deg2 = true;
    for v in 0..n {
        if adj[v as usize].len() != 2 && !adj[v as usize].is_empty() {
            all_deg2 = false;
        }
    }

    // FillConformalChart.cpp:121-133
    if all_deg2 && n > 0 {
        let mut start = 0;
        while start < n && adj[start as usize].is_empty() {
            start += 1;
        }
        if start < n {
            let chain = walk(&edges, &adj, &mut used, start, true);
            if chain.len() >= 2 {
                out.polyline = concat(&edges, pieces, &chain, start, true);
                out.closed = true;
                return out.polyline.points.len() >= 4;
            }
        }
    }

    // Open generator: longest path in the skeleton forest (drop short branches).
    // FillConformalChart.cpp:136-138
    let mut elen: Vec<f64> = vec![0.0; edges.len()];
    for i in 0..edges.len() {
        elen[i] = pieces[edges[i].idx].length();
    }

    // FillConformalChart.cpp:169-181
    let mut src: i32 = 0;
    for v in 0..n {
        if adj[v as usize].len() == 1 {
            src = v;
            break;
        }
    }
    if src >= n || adj[src as usize].is_empty() {
        src = 0;
        while src < n && adj[src as usize].is_empty() {
            src += 1;
        }
        if src >= n {
            return false;
        }
    }

    // FillConformalChart.cpp:183-186
    let (u, _parent0, _parent_e0) = farthest(&edges, &adj, &elen, n, src);
    let (w, parent, parent_e) = farthest(&edges, &adj, &elen, n, u);
    // FillConformalChart.cpp:187-192
    let mut chain: Vec<i32> = Vec::new();
    let mut x = w;
    while x != u && x >= 0 {
        if parent_e[x as usize] < 0 {
            break;
        }
        chain.push(parent_e[x as usize]);
        x = parent[x as usize];
    }
    // FillConformalChart.cpp:193-198
    chain.reverse();
    if chain.is_empty() {
        return false;
    }
    out.polyline = concat(&edges, pieces, &chain, u, false);
    out.closed = false;
    out.polyline.points.len() >= 2
}

/// FillConformalChart.cpp:201-210 — `static bool assign_closed_loop(const Polygon &loop, LayerGenerator &out)`
fn assign_closed_loop(loop_: &Polygon, out: &mut LayerGenerator) -> bool {
    // FillConformalChart.cpp:203-204
    if loop_.points.len() < 3 {
        return false;
    }
    // FillConformalChart.cpp:205-206
    out.polyline = Polyline::from_points(loop_.points.clone());
    out.closed = true;
    // FillConformalChart.cpp:207-208
    if !out.polyline.points.is_empty()
        && out.polyline.points[0] != out.polyline.points[out.polyline.points.len() - 1]
    {
        let front = out.polyline.points[0];
        out.polyline.points.push(front);
    }
    // FillConformalChart.cpp:209
    out.polyline.points.len() >= 4
}

/// FillConformalChart.cpp:223-256 — the lambda `try_inset` of `generator_from_inset`.
fn try_inset(expoly: &ExPolygon, out: &mut LayerGenerator, inset_mm: f64, require_hole: bool) -> bool {
    // FillConformalChart.cpp:224-225
    if inset_mm < 0.35 {
        return false;
    }
    // FillConformalChart.cpp:226-253
    if !expoly.holes.is_empty() {
        let mid = offset_ex(expoly, (-scale_(inset_mm)) as f32);
        if mid.is_empty() {
            return false;
        }
        let mut best: Option<&Polygon> = None;
        let mut best_l = 0.0_f64;
        for ex in &mid {
            for h in &ex.holes {
                let l = h.length();
                if l > best_l {
                    best_l = l;
                    best = Some(h);
                }
            }
        }
        if let Some(b) = best {
            return assign_closed_loop(b, out);
        }
        if require_hole {
            return false;
        }
        for ex in &mid {
            let l = ex.contour.length();
            if l > best_l {
                best_l = l;
                best = Some(&ex.contour);
            }
        }
        return match best {
            Some(b) => assign_closed_loop(b, out),
            None => false,
        };
    }
    // FillConformalChart.cpp:254-255
    let mid = offset_polygon(&expoly.contour, (-scale_(inset_mm)) as f32);
    mid.len() == 1 && assign_closed_loop(&mid[0], out)
}

/// FillConformalChart.cpp:212-266 — `static bool generator_from_inset(const ExPolygon &expoly, LayerGenerator &out)`
fn generator_from_inset(expoly: &ExPolygon, out: &mut LayerGenerator) -> bool {
    // FillConformalChart.cpp:214-216
    let ob = get_extents_points(&expoly.contour.points);
    let min_side = unscale_d(ob.size().x.min(ob.size().y) as f64);
    let mut max_inset = 0.25 * min_side;
    // FillConformalChart.cpp:217-222
    if !expoly.holes.is_empty() {
        let ib = get_extents_points(&expoly.holes[0].points);
        let outer_span = unscale_d(ob.size().x.min(ob.size().y) as f64);
        let inner_span = unscale_d(ib.size().x.max(ib.size().y) as f64);
        max_inset = 0.35 * (0.2_f64).max(0.5 * (outer_span - inner_span));
    }
    // FillConformalChart.cpp:257-261
    if !expoly.holes.is_empty() {
        let mut inset = max_inset;
        while inset >= 0.4 {
            if try_inset(expoly, out, inset, true) {
                return true;
            }
            inset *= 0.5;
        }
    }
    // FillConformalChart.cpp:262-264
    let mut inset = max_inset;
    while inset >= 0.4 {
        if try_inset(expoly, out, inset, false) {
            return true;
        }
        inset *= 0.5;
    }
    // FillConformalChart.cpp:265
    try_inset(expoly, out, 0.5, false)
}

/// ExPolygon.cpp:373-380 — `ExPolygon::medial_axis(double min_width, double max_width, Polylines*)`:
/// the thick polylines of ExPolygon.cpp:263-371 with the widths dropped, appended.
fn medial_axis_polylines(expoly: &ExPolygon, min_width: f64, max_width: f64, polylines: &mut Vec<Polyline>) {
    let mut tp: Vec<MedialAxisThickPolyline> = Vec::new();
    expoly.medial_axis_thick_polylines(min_width, max_width, &mut tp);
    polylines.reserve(tp.len());
    for pl in tp {
        polylines.push(Polyline::from_points(pl.points));
    }
}

/// FillConformalChart.cpp:268-279 — `static bool extract_medial_axis(const ExPolygon &expoly, double spacing, LayerGenerator &out)`
fn extract_medial_axis(expoly: &ExPolygon, spacing: f64, out: &mut LayerGenerator) -> bool {
    // FillConformalChart.cpp:270-272
    let bbox = get_extents_points(&expoly.contour.points);
    let min_width = scale_(spacing.max(1e-3));
    let max_width = bbox.size().x.max(bbox.size().y) as f64 + scale_(1.0);

    // FillConformalChart.cpp:274-277
    let mut pieces: Vec<Polyline> = Vec::new();
    medial_axis_polylines(expoly, min_width, max_width, &mut pieces);
    if pieces.is_empty() {
        medial_axis_polylines(expoly, min_width * 0.25, max_width, &mut pieces);
    }
    // FillConformalChart.cpp:278
    !pieces.is_empty() && stitch_centerline(&pieces, out) && out.polyline.points.len() >= 2
}

/// 2D medial-axis / inset generator for a single island (also used without a loft).
/// FillConformalChart.cpp:281-294 — `bool extract_generator(const ExPolygon &expoly, double spacing, LayerGenerator &out)`
pub fn extract_generator(expoly: &ExPolygon, spacing: f64, out: &mut LayerGenerator) -> bool {
    // FillConformalChart.cpp:283-284
    *out = LayerGenerator::default();
    #[allow(unused_assignments)]
    let mut ok = false;
    // FillConformalChart.cpp:285-289
    if !expoly.holes.is_empty() {
        ok = generator_from_inset(expoly, out) || extract_medial_axis(expoly, spacing, out);
    } else {
        ok = extract_medial_axis(expoly, spacing, out) || generator_from_inset(expoly, out);
    }
    // FillConformalChart.cpp:290-291
    if !ok {
        return false;
    }
    // FillConformalChart.cpp:292-293
    out.length = out.polyline.length();
    out.polyline.points.len() >= 2 && out.length > scale_(0.5)
}

/// FillConformalChart.cpp:296-329 — `static bool sample_at_s(const Polyline &pl, bool closed, double s, Point &p)`
fn sample_at_s(pl: &Polyline, closed: bool, mut s: f64, p: &mut Point) -> bool {
    // FillConformalChart.cpp:298-300
    let pts = &pl.points;
    if pts.len() < 2 {
        return false;
    }
    // FillConformalChart.cpp:301-306
    let nseg = pts.len() - 1;
    let mut l = 0.0_f64;
    for i in 0..nseg {
        l += diff_norm(pts[i + 1], pts[i]);
    }
    if l < 1.0 {
        return false;
    }
    // FillConformalChart.cpp:307-313
    if closed {
        s %= l; // std::fmod
        if s < 0.0 {
            s += l;
        }
    } else {
        s = (0.0_f64).max(s.min(l));
    }
    // FillConformalChart.cpp:314-326
    let mut acc = 0.0_f64;
    for i in 0..nseg {
        let dx = (pts[i + 1].x - pts[i].x) as f64;
        let dy = (pts[i + 1].y - pts[i].y) as f64;
        let len = (dx * dx + dy * dy).sqrt();
        if len < 1.0 {
            continue;
        }
        if acc + len >= s - 1e-3 {
            let t = (0.0_f64).max((1.0_f64).min((s - acc) / len));
            // std::lround: half away from zero.
            *p = Point::new(pts[i].x + (dx * t).round() as Coord, pts[i].y + (dy * t).round() as Coord);
            return true;
        }
        acc += len;
    }
    // FillConformalChart.cpp:327-328
    *p = pts[nseg];
    true
}

/// FillConformalChart.cpp:331-348 — `static Points resample(const Polyline &pl, bool closed, int n)`
fn resample(pl: &Polyline, closed: bool, n: i32) -> Vec<Point> {
    // FillConformalChart.cpp:333-335
    let mut out: Vec<Point> = Vec::new();
    if pl.points.len() < 2 || n < 2 {
        return out;
    }
    // FillConformalChart.cpp:336-338
    let l = pl.length();
    if l < 1.0 {
        return out;
    }
    // FillConformalChart.cpp:339-346
    out.reserve(n as usize);
    for i in 0..n {
        let s = if closed {
            (i as f64 / n as f64) * l
        } else if n <= 1 {
            0.0
        } else {
            i as f64 / (n - 1) as f64 * l
        };
        let mut p = Point::new(0, 0);
        if sample_at_s(pl, closed, s, &mut p) {
            out.push(p);
        }
    }
    out
}

/// FillConformalChart.cpp:350-357 — `static double sum_point_dist(const Points &a, const Points &b)`
fn sum_point_dist(a: &[Point], b: &[Point]) -> f64 {
    let n = a.len().min(b.len());
    let mut s = 0.0_f64;
    for i in 0..n {
        s += diff_norm(a[i], b[i]);
    }
    s
}

/// FillConformalChart.cpp:359-370 — `static void rotate_closed(Polyline &pl, size_t k)`
fn rotate_closed(pl: &mut Polyline, mut k: usize) {
    // FillConformalChart.cpp:361-362
    if pl.points.len() < 4 {
        return;
    }
    // FillConformalChart.cpp:363-365
    let drop_last = if pl.points[0] == pl.points[pl.points.len() - 1] { 1 } else { 0 };
    let mut pts: Vec<Point> = pl.points[..pl.points.len() - drop_last].to_vec();
    if pts.is_empty() {
        return;
    }
    // FillConformalChart.cpp:366-369
    k %= pts.len();
    pts.rotate_left(k);
    let front = pts[0];
    pts.push(front);
    *pl = Polyline::from_points(pts);
}

/// FillConformalChart.cpp:372-387 — `static size_t nearest_vertex(const Polyline &pl, const Point &q)`
fn nearest_vertex(pl: &Polyline, q: &Point) -> usize {
    // FillConformalChart.cpp:374-376
    let mut n = pl.points.len();
    if n >= 2 && pl.points[0] == pl.points[n - 1] {
        n -= 1;
    }
    // FillConformalChart.cpp:377-386
    let mut best = 0usize;
    let mut best_d = f64::MAX;
    for i in 0..n {
        let d = diff_norm(pl.points[i], *q);
        if d < best_d {
            best_d = d;
            best = i;
        }
    }
    best
}

/// FillConformalChart.cpp:395-401 — the lambda `cost` of `align_open`.
fn align_open_cost(cur: &LayerGenerator, pa: &[Point], n: i32, rev: bool) -> f64 {
    let mut b = cur.polyline.clone();
    if rev {
        b.reverse();
    }
    let pb = resample(&b, false, n);
    sum_point_dist(pa, &pb)
}

/// FillConformalChart.cpp:389-404 — `static void align_open(LayerGenerator &cur, const LayerGenerator &prev)`
fn align_open(cur: &mut LayerGenerator, prev: &LayerGenerator) {
    // FillConformalChart.cpp:391-392
    if cur.polyline.points.len() < 2 || prev.polyline.points.len() < 2 {
        return;
    }
    // FillConformalChart.cpp:393-394
    const N: i32 = 32;
    let pa = resample(&prev.polyline, false, N);
    // FillConformalChart.cpp:402-403
    if align_open_cost(cur, &pa, N, true) + scale_(0.2) < align_open_cost(cur, &pa, N, false) {
        cur.polyline.reverse();
    }
}

/// FillConformalChart.cpp:415-436 — the lambda `try_orient` of `align_closed`.
fn try_orient(cur: &LayerGenerator, pa: &[Point], n: i32, rev: bool) -> (f64, usize) {
    let mut cand = cur.polyline.clone();
    if rev {
        cand.reverse();
    }
    let mut nvert = cand.points.len();
    if nvert >= 2 && cand.points[0] == cand.points[nvert - 1] {
        nvert -= 1;
    }
    let mut best = f64::MAX;
    let mut bestk = 0usize;
    let step = std::cmp::max(1usize, nvert / 48);
    let mut k = 0usize;
    while k < nvert {
        let mut t = cand.clone();
        rotate_closed(&mut t, k);
        let pb = resample(&t, true, n);
        let c = sum_point_dist(pa, &pb);
        if c < best {
            best = c;
            bestk = k;
        }
        k += step;
    }
    (best, bestk)
}

/// FillConformalChart.cpp:406-446 — `static void align_closed(LayerGenerator &cur, const LayerGenerator &prev)`
fn align_closed(cur: &mut LayerGenerator, prev: &LayerGenerator) {
    // FillConformalChart.cpp:408-409
    if cur.polyline.points.len() < 4 || prev.polyline.points.len() < 2 {
        return;
    }
    // FillConformalChart.cpp:410-413
    const N: i32 = 32;
    let pa = resample(&prev.polyline, prev.closed, N);
    if pa.is_empty() {
        return;
    }

    // FillConformalChart.cpp:438-445
    let (c0, k0) = try_orient(cur, &pa, N, false);
    let (c1, k1) = try_orient(cur, &pa, N, true);
    if c1 < c0 {
        cur.polyline.reverse();
        rotate_closed(&mut cur.polyline, k1);
    } else {
        rotate_closed(&mut cur.polyline, k0);
    }
}

/// FillConformalChart.cpp:448-466 — `static void align_open_to_closed(LayerGenerator &cur, const LayerGenerator &prev)`
fn align_open_to_closed(cur: &mut LayerGenerator, prev: &LayerGenerator) {
    // FillConformalChart.cpp:450-451
    if cur.polyline.points.len() < 4 || prev.polyline.points.len() < 2 {
        return;
    }
    // FillConformalChart.cpp:452
    let k = nearest_vertex(&cur.polyline, &prev.polyline.points[0]);
    rotate_closed(&mut cur.polyline, k);
    // Match walking direction to the previous open spine.
    // FillConformalChart.cpp:454-456
    let mut p1 = Point::new(0, 0);
    if !sample_at_s(&prev.polyline, false, prev.length.min(scale_(2.0)), &mut p1) {
        return;
    }
    // FillConformalChart.cpp:457
    let want = (
        (p1.x - prev.polyline.points[0].x) as f64,
        (p1.y - prev.polyline.points[0].y) as f64,
    );
    // FillConformalChart.cpp:458-460
    let mut q1 = Point::new(0, 0);
    if !sample_at_s(&cur.polyline, true, scale_(2.0), &mut q1) {
        return;
    }
    // FillConformalChart.cpp:461
    let got = (
        (q1.x - cur.polyline.points[0].x) as f64,
        (q1.y - cur.polyline.points[0].y) as f64,
    );
    // FillConformalChart.cpp:462-465 — Eigen dot, not fused.
    if want.0 * got.0 + want.1 * got.1 < 0.0 {
        cur.polyline.reverse();
        let k = nearest_vertex(&cur.polyline, &prev.polyline.points[0]);
        rotate_closed(&mut cur.polyline, k);
    }
}

/// FillConformalChart.cpp:468-476 — `static void align_closed_to_open(LayerGenerator &cur, const LayerGenerator &prev)`
fn align_closed_to_open(cur: &mut LayerGenerator, prev: &LayerGenerator) {
    // FillConformalChart.cpp:470-471
    if cur.polyline.points.len() < 2 || prev.polyline.points.len() < 2 {
        return;
    }
    // FillConformalChart.cpp:472-475
    let df = diff_norm(cur.polyline.points[0], prev.polyline.points[0]);
    let db = diff_norm(cur.polyline.points[cur.polyline.points.len() - 1], prev.polyline.points[0]);
    if db < df {
        cur.polyline.reverse();
    }
}

/// FillConformalChart.cpp:478-490 — `static void align_to_prev(LayerGenerator &cur, const LayerGenerator &prev)`
fn align_to_prev(cur: &mut LayerGenerator, prev: &LayerGenerator) {
    // FillConformalChart.cpp:480-481
    if prev.polyline.points.len() < 2 || cur.polyline.points.len() < 2 {
        return;
    }
    // FillConformalChart.cpp:482-489
    if !cur.closed && !prev.closed {
        align_open(cur, prev);
    } else if cur.closed && prev.closed {
        align_closed(cur, prev);
    } else if cur.closed && !prev.closed {
        align_open_to_closed(cur, prev);
    } else {
        align_closed_to_open(cur, prev);
    }
}

/// FillConformalChart.cpp:492-526 — `static void edt_1d(const float *f, int n, float *d, int *v, float *z)`
fn edt_1d(f: &[f32], n: i32, d: &mut [f32], v: &mut [i32], z: &mut [f32]) {
    // FillConformalChart.cpp:494-495
    if n <= 0 {
        return;
    }
    // FillConformalChart.cpp:496-500
    let inf: f32 = 1e20;
    let mut k: i32 = 0;
    v[0] = 0;
    z[0] = -inf;
    z[1] = inf;
    // FillConformalChart.cpp:501-518
    for q in 1..n {
        let mut s: f32;
        loop {
            let vk = v[k as usize];
            // (f[q] + q*q) and (f[vk] + vk*vk) are each one fused multiply-add (float).
            let qf = q as f32;
            let vkf = vk as f32;
            s = (qf.mul_add(qf, f[q as usize]) - vkf.mul_add(vkf, f[vk as usize])) / (2.0f32 * (q - vk) as f32);
            if s > z[k as usize] {
                break;
            }
            k -= 1;
            if k < 0 {
                k = 0;
                s = -inf;
                break;
            }
        }
        k += 1;
        v[k as usize] = q;
        z[k as usize] = s;
        z[(k + 1) as usize] = inf;
    }
    // FillConformalChart.cpp:519-525
    k = 0;
    for q in 0..n {
        while z[(k + 1) as usize] < q as f32 {
            k += 1;
        }
        let diff = (q - v[k as usize]) as f32;
        // diff * diff + f[v[k]]: one fused multiply-add (float).
        d[q as usize] = diff.mul_add(diff, f[v[k as usize] as usize]);
    }
}

/// FillConformalChart.cpp:533-555 — the lambda `add_edges` of `rasterize_expoly`.
fn add_edges(poly: &Polygon, hits: &mut [Vec<f32>], ny: i32, h: f64, origin: (f64, f64)) {
    let pts = &poly.points;
    let n = pts.len();
    for i in 0..n {
        let a = pts[i];
        let b = pts[(i + 1) % n];
        let y0 = unscale_d(a.y as f64);
        let y1 = unscale_d(b.y as f64);
        if (y1 - y0).abs() < 1e-9 {
            continue;
        }
        let x0 = unscale_d(a.x as f64);
        let x1 = unscale_d(b.x as f64);
        let ymin = std::cmp::max(0, ((y0.min(y1) - origin.1) / h - 1e-9).ceil() as i32);
        let ymax = std::cmp::min(ny - 1, ((y0.max(y1) - origin.1) / h - 1e-9).floor() as i32);
        let mut y = ymin;
        while y <= ymax {
            // origin.y() + (double(y) + 0.5) * h: fused.
            let py = (y as f64 + 0.5).mul_add(h, origin.1);
            let t = (py - y0) / (y1 - y0);
            if !(t < 0.0 || t > 1.0) {
                // float(x0 + t * (x1 - x0)): fused.
                hits[y as usize].push(t.mul_add(x1 - x0, x0) as f32);
            }
            y += 1;
        }
    }
}

/// FillConformalChart.cpp:528-572 — `static void rasterize_expoly(const ExPolygon &ex, int nx, int ny, double h, const Vec2d &origin, std::vector<char> &img)`
fn rasterize_expoly(ex: &ExPolygon, nx: i32, ny: i32, h: f64, origin: (f64, f64), img: &mut Vec<u8>) {
    // FillConformalChart.cpp:530-532
    img.clear();
    img.resize(nx as usize * ny as usize, 0);
    if ex.contour.points.len() < 3 {
        return;
    }
    // FillConformalChart.cpp:556-559
    let mut hits: Vec<Vec<f32>> = vec![Vec::new(); ny as usize];
    add_edges(&ex.contour, &mut hits, ny, h, origin);
    for hpoly in &ex.holes {
        add_edges(hpoly, &mut hits, ny, h, origin);
    }
    // FillConformalChart.cpp:560-571
    for y in 0..ny {
        let row = &mut hits[y as usize];
        if row.len() < 2 {
            continue;
        }
        // std::sort of floats: equal keys are indistinguishable, so stability is moot.
        row.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let mut i = 0usize;
        while i + 1 < row.len() {
            let x0 = std::cmp::max(0, ((row[i] as f64 - origin.0) / h).floor() as i32);
            let x1 = std::cmp::min(nx - 1, ((row[i + 1] as f64 - origin.0) / h).floor() as i32);
            let mut x = x0;
            while x <= x1 {
                img[x as usize + y as usize * nx as usize] = 1;
                x += 1;
            }
            i += 2;
        }
    }
}

/// FillConformalChart.cpp:574-602 — `static void add_axis_quad(::indexed_triangle_set &its, const Vec3f &c, int axis, float h)`
fn add_axis_quad(its: &mut indexed_triangle_set, c: &Vec3f, axis: i32, h: f32) {
    // FillConformalChart.cpp:576-590
    let s = 0.55f32 * h;
    let (a, b, d): (Vec3f, Vec3f, Vec3f);
    if axis == 0 {
        a = Vec3f::new(0.0, -s, -s);
        b = Vec3f::new(0.0, s, -s);
        d = Vec3f::new(0.0, s, s);
    } else if axis == 1 {
        a = Vec3f::new(-s, 0.0, -s);
        b = Vec3f::new(s, 0.0, -s);
        d = Vec3f::new(s, 0.0, s);
    } else {
        a = Vec3f::new(-s, -s, 0.0);
        b = Vec3f::new(s, -s, 0.0);
        d = Vec3f::new(s, s, 0.0);
    }
    // FillConformalChart.cpp:591-594
    let p0 = c + a;
    let p1 = c + b;
    let p2 = c + d;
    let p3 = c + (a + d - b);
    // FillConformalChart.cpp:595-601
    let i = its.vertices.len() as i32;
    its.vertices.push(p0);
    its.vertices.push(p1);
    its.vertices.push(p2);
    its.vertices.push(p3);
    its.indices.push(Vec3crd::new(i, i + 1, i + 2));
    its.indices.push(Vec3crd::new(i, i + 2, i + 3));
}

/// FillConformalChart.cpp:610-619 — the lambda `id` of `polyline_from_pixels`.
fn pixel_id(id_of: &mut BTreeMap<(i32, i32), i32>, nodes: &mut Vec<(i32, i32)>, x: i32, y: i32) -> i32 {
    let key = (x, y);
    if let Some(&it) = id_of.get(&key) {
        return it;
    }
    let n = nodes.len() as i32;
    id_of.insert(key, n);
    nodes.push((x, y));
    n
}

/// FillConformalChart.cpp:624-629 — the lambda `try_link` of `polyline_from_pixels`.
fn try_link(id_of: &BTreeMap<(i32, i32), i32>, adj: &mut [Vec<i32>], a: i32, bx: i32, by: i32) {
    match id_of.get(&(bx, by)) {
        None => {}
        Some(&it) => {
            if it == a {
                return;
            }
            adj[a as usize].push(it);
        }
    }
}

/// FillConformalChart.cpp:638-658 — the lambda `farthest` of `polyline_from_pixels`.
fn pixel_farthest(adj: &[Vec<i32>], n: i32, src: i32) -> (i32, Vec<i32>) {
    let mut dist: Vec<i32> = vec![-1; n as usize];
    let mut parent: Vec<i32> = vec![-1; n as usize];
    let mut q: VecDeque<i32> = VecDeque::new();
    dist[src as usize] = 0;
    q.push_back(src);
    let mut best = src;
    while let Some(v) = q.pop_front() {
        if dist[v as usize] > dist[best as usize] {
            best = v;
        }
        for &u in &adj[v as usize] {
            if dist[u as usize] >= 0 {
                continue;
            }
            dist[u as usize] = dist[v as usize] + 1;
            parent[u as usize] = v;
            q.push_back(u);
        }
    }
    (best, parent)
}

/// FillConformalChart.cpp:604-732 — `static bool polyline_from_pixels(const std::vector<Vec2i> &pix, double h, const Vec2d &origin, LayerGenerator &out)`
fn polyline_from_pixels(pix: &[(i32, i32)], h: f64, origin: (f64, f64), out: &mut LayerGenerator) -> bool {
    // FillConformalChart.cpp:606-607
    if pix.len() < 2 {
        return false;
    }
    // FillConformalChart.cpp:608-609 — std::map<std::pair<int,int>,int>: find/emplace only.
    let mut id_of: BTreeMap<(i32, i32), i32> = BTreeMap::new();
    let mut nodes: Vec<(i32, i32)> = Vec::new();
    // FillConformalChart.cpp:620-621
    for p in pix {
        pixel_id(&mut id_of, &mut nodes, p.0, p.1);
    }
    // FillConformalChart.cpp:622-623
    let n = nodes.len() as i32;
    let mut adj: Vec<Vec<i32>> = vec![Vec::new(); n as usize];
    // FillConformalChart.cpp:630-637
    for i in 0..n {
        let x = nodes[i as usize].0;
        let y = nodes[i as usize].1;
        try_link(&id_of, &mut adj, i, x + 1, y);
        try_link(&id_of, &mut adj, i, x - 1, y);
        try_link(&id_of, &mut adj, i, x, y + 1);
        try_link(&id_of, &mut adj, i, x, y - 1);
    }
    // FillConformalChart.cpp:659-666
    let mut all_deg2 = true;
    let mut start: i32 = 0;
    for i in 0..n {
        if adj[i as usize].len() == 1 {
            start = i;
        }
        if adj[i as usize].len() != 2 && !adj[i as usize].is_empty() {
            all_deg2 = false;
        }
    }
    // origin + (double(node) + 0.5) * h: fused; Point::new_scale truncates.
    let pixel_point = |idx: i32| -> Point {
        let px = (nodes[idx as usize].0 as f64 + 0.5).mul_add(h, origin.0);
        let py = (nodes[idx as usize].1 as f64 + 0.5).mul_add(h, origin.1);
        point_new_scale(px, py)
    };
    // FillConformalChart.cpp:669-711
    if all_deg2 && n >= 4 {
        let mut used: Vec<u8> = vec![0; n as usize];
        let mut chain: Vec<i32> = Vec::new();
        let mut v: i32 = 0;
        while v < n && adj[v as usize].is_empty() {
            v += 1;
        }
        let mut prev: i32 = -1;
        let mut step = 0;
        while step < n + 2 && v >= 0 && v < n {
            chain.push(v);
            used[v as usize] = 1;
            let mut nxt: i32 = -1;
            for &u in &adj[v as usize] {
                if u != prev && used[u as usize] == 0 {
                    nxt = u;
                    break;
                }
            }
            if nxt < 0 {
                for &u in &adj[v as usize] {
                    if u != prev {
                        nxt = u;
                        break;
                    }
                }
            }
            prev = v;
            v = nxt;
            if v == chain[0] {
                break;
            }
            step += 1;
        }
        if chain.len() < 4 {
            return false;
        }
        out.closed = true;
        let mut points: Vec<Point> = Vec::new();
        for &idx in &chain {
            points.push(pixel_point(idx));
        }
        if points[0] != points[points.len() - 1] {
            let front = points[0];
            points.push(front);
        }
        out.polyline = Polyline::from_points(points);
        out.length = out.polyline.length();
        return out.polyline.points.len() >= 4 && out.length > scale_(0.5);
    }
    // FillConformalChart.cpp:712-722
    let (a, _parent0) = pixel_farthest(&adj, n, start);
    let (b, parent) = pixel_farthest(&adj, n, a);
    let mut chain: Vec<i32> = Vec::new();
    let mut x = b;
    while x >= 0 {
        chain.push(x);
        if x == a {
            break;
        }
        x = parent[x as usize];
    }
    chain.reverse();
    if chain.len() < 2 {
        return false;
    }
    // FillConformalChart.cpp:723-731
    out.closed = false;
    let mut points: Vec<Point> = Vec::new();
    for &idx in &chain {
        points.push(pixel_point(idx));
    }
    out.polyline = Polyline::from_points(points);
    out.length = out.polyline.length();
    out.polyline.points.len() >= 2 && out.length > scale_(0.5)
}

/// FillConformalChart.cpp:734-918 — `static bool build_sdf_sheet(std::vector<LayerGenerator> &layers, std::unique_ptr<::indexed_triangle_set> &sheet, const std::vector<ExPolygon> &islands, const std::vector<double> &zs, double spacing)`
fn build_sdf_sheet(
    layers: &mut Vec<LayerGenerator>,
    sheet: &mut Option<Box<indexed_triangle_set>>,
    islands: &[ExPolygon],
    zs: &[f64],
    spacing: f64,
) -> bool {
    // FillConformalChart.cpp:740-748
    let nlayer = islands.len().min(zs.len());
    if nlayer < 8 {
        return false;
    }
    let mut bb = BoundingBox::new();
    for ex in islands {
        if ex.contour.points.len() >= 3 {
            bb.merge(&get_extents_points(&ex.contour.points));
        }
    }
    if !bb.is_defined() {
        return false;
    }

    // FillConformalChart.cpp:750-756
    let origin = (unscale_d(bb.min.x as f64) - 1.0, unscale_d(bb.min.y as f64) - 1.0);
    let span = (
        unscale_d(bb.max.x as f64) - origin.0 + 1.0,
        unscale_d(bb.max.y as f64) - origin.1 + 1.0,
    );
    let z0 = zs[0];
    let z1 = zs[zs.len() - 1];
    let mut h = 0.8_f64;
    let (mut nx, mut ny, mut nz) = (0i32, 0i32, 0i32);
    // FillConformalChart.cpp:757-764
    for _iter in 0..6 {
        nx = std::cmp::max(8, (span.0 / h).ceil() as i32);
        ny = std::cmp::max(8, (span.1 / h).ceil() as i32);
        nz = std::cmp::max(4, ((z1 - z0).max(h) / h).ceil() as i32 + 1);
        if nx as usize * ny as usize * nz as usize <= 9_000_000usize {
            break;
        }
        h *= 1.35;
    }
    // FillConformalChart.cpp:765-769
    if nz < 8 || (z1 - z0) < 4.0 {
        return false;
    }
    let nxyz = nx as usize * ny as usize * nz as usize;
    if nxyz < 64 {
        return false;
    }

    // FillConformalChart.cpp:771-774
    let mut occ: Vec<u8> = vec![0; nxyz];
    let at = |x: i32, y: i32, z: i32| -> usize {
        x as usize + nx as usize * (y as usize + ny as usize * z as usize)
    };
    // FillConformalChart.cpp:775-793
    let mut slice: Vec<u8> = Vec::new();
    for li in 0..nlayer {
        if islands[li].contour.points.len() < 3 {
            continue;
        }
        rasterize_expoly(&islands[li], nx, ny, h, origin, &mut slice);
        let mut iz0 = ((zs[li] - z0) / h).round() as i32;
        let mut iz1 = iz0;
        if li + 1 < nlayer {
            iz1 = ((zs[li + 1] - z0) / h).round() as i32;
        }
        iz0 = std::cmp::max(0, std::cmp::min(nz - 1, iz0));
        iz1 = std::cmp::max(0, std::cmp::min(nz - 1, iz1));
        if iz1 < iz0 {
            std::mem::swap(&mut iz0, &mut iz1);
        }
        for iz in iz0..=iz1 {
            for y in 0..ny {
                for x in 0..nx {
                    if slice[x as usize + y as usize * nx as usize] != 0 {
                        occ[at(x, y, iz)] = 1;
                    }
                }
            }
        }
    }

    // FillConformalChart.cpp:795-798
    let inf: f32 = 1e12;
    let mut f: Vec<f32> = vec![0.0; nxyz];
    let mut dt: Vec<f32> = vec![0.0; nxyz];
    let mut tmp: Vec<f32> = vec![0.0; nxyz];
    for i in 0..nxyz {
        f[i] = if occ[i] != 0 { inf } else { 0.0 };
    }

    // FillConformalChart.cpp:800-803
    let dim = std::cmp::max(nx, std::cmp::max(ny, nz)) as usize;
    let mut line: Vec<f32> = vec![0.0; dim];
    let mut outl: Vec<f32> = vec![0.0; dim];
    let mut vbuf: Vec<i32> = vec![0; dim];
    let mut zbuf: Vec<f32> = vec![0.0; dim + 1];
    // FillConformalChart.cpp:804-811
    for z in 0..nz {
        for y in 0..ny {
            for x in 0..nx {
                line[x as usize] = f[at(x, y, z)];
            }
            edt_1d(&line, nx, &mut outl, &mut vbuf, &mut zbuf);
            for x in 0..nx {
                tmp[at(x, y, z)] = outl[x as usize];
            }
        }
    }
    // FillConformalChart.cpp:812-819
    for z in 0..nz {
        for x in 0..nx {
            for y in 0..ny {
                line[y as usize] = tmp[at(x, y, z)];
            }
            edt_1d(&line, ny, &mut outl, &mut vbuf, &mut zbuf);
            for y in 0..ny {
                tmp[at(x, y, z)] = outl[y as usize];
            }
        }
    }
    // FillConformalChart.cpp:820-827
    for y in 0..ny {
        for x in 0..nx {
            for z in 0..nz {
                line[z as usize] = tmp[at(x, y, z)];
            }
            edt_1d(&line, nz, &mut outl, &mut vbuf, &mut zbuf);
            for z in 0..nz {
                dt[at(x, y, z)] = (0.0f32).max(outl[z as usize]).sqrt() * h as f32;
            }
        }
    }

    // FillConformalChart.cpp:829-833 — the lambda `sample_dt`.
    let sample_dt = |x: i32, y: i32, z: i32| -> f32 {
        if x < 0 || y < 0 || z < 0 || x >= nx || y >= ny || z >= nz {
            return 0.0;
        }
        if occ[at(x, y, z)] != 0 {
            dt[at(x, y, z)]
        } else {
            0.0
        }
    };

    // FillConformalChart.cpp:835-866
    let mut ridge: Vec<u8> = vec![0; nxyz];
    let mut axis_of: Vec<i8> = vec![0; nxyz];
    let min_r = (1.15 * h) as f32;
    let mut n_ridge: usize = 0;
    for z in 1..nz - 1 {
        for y in 1..ny - 1 {
            for x in 1..nx - 1 {
                let i = at(x, y, z);
                if occ[i] == 0 || dt[i] < min_r {
                    continue;
                }
                let d = dt[i];
                // d - 0.5f * (a + b): one fused multiply-add (float).
                let dx = (-0.5f32).mul_add(sample_dt(x - 1, y, z) + sample_dt(x + 1, y, z), d);
                let dy = (-0.5f32).mul_add(sample_dt(x, y - 1, z) + sample_dt(x, y + 1, z), d);
                let dz = (-0.5f32).mul_add(sample_dt(x, y, z - 1) + sample_dt(x, y, z + 1), d);
                let mut best = dx;
                let mut ax = 0;
                if dy > best {
                    best = dy;
                    ax = 1;
                }
                if dz > best {
                    best = dz;
                    ax = 2;
                }
                if best < 0.12f32 * h as f32 {
                    continue;
                }
                ridge[i] = 1;
                axis_of[i] = ax as i8;
                n_ridge += 1;
            }
        }
    }
    if n_ridge < 16 {
        return false;
    }

    // FillConformalChart.cpp:868-881
    let mut its = Box::new(indexed_triangle_set::default());
    its.vertices.reserve(n_ridge * 4);
    its.indices.reserve(n_ridge * 2);
    for z in 1..nz - 1 {
        for y in 1..ny - 1 {
            for x in 1..nx - 1 {
                let i = at(x, y, z);
                if ridge[i] == 0 {
                    continue;
                }
                // origin + (double(x) + 0.5) * h and z0 + (double(z) + 0.5) * h: fused.
                let c = Vec3f::new(
                    (x as f64 + 0.5).mul_add(h, origin.0) as f32,
                    (y as f64 + 0.5).mul_add(h, origin.1) as f32,
                    (z as f64 + 0.5).mul_add(h, z0) as f32,
                );
                add_axis_quad(&mut its, &c, axis_of[i] as i32, h as f32);
            }
        }
    }
    *sheet = Some(its);

    // FillConformalChart.cpp:883-916
    layers.clear();
    layers.resize(nlayer, LayerGenerator::default());
    // `const LayerGenerator *prev`: the index of the last layer that got a generator.
    let mut prev: Option<usize> = None;
    let mut ok_n = 0;
    for li in 0..nlayer {
        let mut g = LayerGenerator::default();
        g.z = zs[li];
        let iz = std::cmp::max(0, std::cmp::min(nz - 1, ((zs[li] - z0) / h).round() as i32));
        let mut pix: Vec<(i32, i32)> = Vec::new();
        let mut n_pix: usize = 0;
        for y in 0..ny {
            for x in 0..nx {
                if ridge[at(x, y, iz)] != 0 {
                    pix.push((x, y));
                    n_pix += 1;
                }
            }
        }
        let island_a = if islands[li].contour.points.len() >= 3 {
            unscale_d(unscale_d(islands[li].area().abs()))
        } else {
            0.0
        };
        let ridge_a = n_pix as f64 * h * h;
        let region_like = island_a > 1.0 && ridge_a > 0.22 * island_a;
        let mut got = false;
        if !region_like && n_pix >= 4 {
            got = polyline_from_pixels(&pix, h, origin, &mut g);
        }
        if !got && islands[li].contour.points.len() >= 3 {
            got = extract_generator(&islands[li], spacing, &mut g);
        }
        if got {
            if let Some(pi) = prev {
                let prev_g = layers[pi].clone();
                align_to_prev(&mut g, &prev_g);
            }
            g.length = g.polyline.length();
            prev = Some(li);
            ok_n += 1;
        } else {
            g = LayerGenerator::default();
            g.z = zs[li];
        }
        layers[li] = g;
    }
    // FillConformalChart.cpp:917
    ok_n >= 2 && sheet.as_ref().map(|s| !s.indices.is_empty()).unwrap_or(false)
}

/// 3D SDF-ridge sheet (when available), plus per-layer curves G = sheet ∩ layer
/// (or 2D MAT fallback). u is tracked across height after alignment.
/// FillConformalChart.hpp:26-49 — `class MedialChart`
///
/// The move constructor and move assignment (FillConformalChart.cpp:922-923) are
/// Rust moves; the deleted copies are the absence of `Clone`; the destructor
/// (FillConformalChart.cpp:921) is the default drop.
#[derive(Debug, Default)]
pub struct MedialChart {
    /// FillConformalChart.hpp:47 — `std::vector<LayerGenerator> m_layers;`
    m_layers: Vec<LayerGenerator>,
    /// FillConformalChart.hpp:48 — `std::unique_ptr<::indexed_triangle_set> m_sheet;`
    m_sheet: Option<Box<indexed_triangle_set>>,
}

impl MedialChart {
    /// FillConformalChart.cpp:920 — `MedialChart::MedialChart() = default;`
    pub fn new() -> Self {
        Self::default()
    }

    /// FillConformalChart.cpp:925-933 — `const LayerGenerator *MedialChart::layer(size_t layer_id) const`
    pub fn layer(&self, layer_id: usize) -> Option<&LayerGenerator> {
        if layer_id >= self.m_layers.len() {
            return None;
        }
        let g = &self.m_layers[layer_id];
        if g.polyline.points.len() < 2 {
            return None;
        }
        Some(g)
    }

    /// FillConformalChart.hpp:42 — `size_t layer_count() const { return m_layers.size(); }`
    pub fn layer_count(&self) -> usize {
        self.m_layers.len()
    }

    /// FillConformalChart.hpp:43 — `bool empty() const { return m_layers.empty(); }`
    pub fn empty(&self) -> bool {
        self.m_layers.is_empty()
    }

    /// FillConformalChart.cpp:935-940 — `const ::indexed_triangle_set *MedialChart::sheet_mesh() const`
    pub fn sheet_mesh(&self) -> Option<&indexed_triangle_set> {
        match &self.m_sheet {
            Some(s) if !s.indices.is_empty() => Some(s.as_ref()),
            _ => None,
        }
    }

    /// FillConformalChart.cpp:942-975 — `std::unique_ptr<MedialChart> MedialChart::build(const std::vector<ExPolygon> &islands, const std::vector<double> &zs, double spacing)`
    pub fn build(islands: &[ExPolygon], zs: &[f64], spacing: f64) -> Option<Box<MedialChart>> {
        // FillConformalChart.cpp:946-948
        let mut chart = Box::new(MedialChart::new());
        if build_sdf_sheet(&mut chart.m_layers, &mut chart.m_sheet, islands, zs, spacing) {
            return Some(chart);
        }

        // FillConformalChart.cpp:950-967
        let n = islands.len().min(zs.len());
        chart.m_layers.clear();
        chart.m_layers.resize(n, LayerGenerator::default());
        let mut prev: Option<usize> = None;
        for i in 0..n {
            let mut g = LayerGenerator::default();
            g.z = zs[i];
            if islands[i].contour.points.len() >= 3 && extract_generator(&islands[i], spacing, &mut g) {
                if let Some(pi) = prev {
                    let prev_g = chart.m_layers[pi].clone();
                    align_to_prev(&mut g, &prev_g);
                } else if !g.closed
                    && g.polyline.points.len() >= 2
                    && g.polyline.points[g.polyline.points.len() - 1].x > g.polyline.points[0].x
                {
                    g.polyline.reverse();
                }
                g.length = g.polyline.length();
                prev = Some(i);
            } else {
                g = LayerGenerator::default();
                g.z = zs[i];
            }
            chart.m_layers[i] = g;
        }
        // FillConformalChart.cpp:968-974
        let mut any = false;
        for g in &chart.m_layers {
            if g.polyline.points.len() >= 2 {
                any = true;
            }
        }
        if !any {
            return None;
        }
        Some(chart)
    }
}

// FillConformalChart.cpp:977-978 — } // namespace FillConformal } // namespace Slic3r

#[cfg(test)]
mod tests {
    use super::*;

    fn mm(v: f64) -> Coord {
        (v * 100000.0).round() as Coord
    }

    fn rect(x0: f64, y0: f64, x1: f64, y1: f64) -> Polygon {
        Polygon::from_points(vec![
            Point::new(mm(x0), mm(y0)),
            Point::new(mm(x1), mm(y0)),
            Point::new(mm(x1), mm(y1)),
            Point::new(mm(x0), mm(y1)),
        ])
    }

    fn square_with_hole() -> ExPolygon {
        let mut hole = rect(-4.0, -4.0, 4.0, 4.0);
        hole.points.reverse();
        ExPolygon::with_holes(rect(-10.0, -10.0, 10.0, 10.0), vec![hole])
    }

    fn pl(points: &[(f64, f64)]) -> Polyline {
        Polyline::from_points(points.iter().map(|&(x, y)| Point::new(mm(x), mm(y))).collect())
    }

    #[test]
    fn quantize_pt_aligns_down_to_the_tenth_of_a_millimetre() {
        // scaled<coord_t>(0.1) is the truncated quotient 0.1 / 0.00001.
        let q = (0.1_f64 / 0.00001_f64) as Coord;
        assert!(q == 10000 || q == 9999);
        let p = quantize_pt(&Point::new(25_000, -1));
        assert_eq!(p.x, (25_000 / q) * q);
        assert_eq!(p.y, -q);
        // Never bigger than the input.
        assert!(p.x <= 25_000 && p.y <= -1);
    }

    #[test]
    fn stitch_centerline_joins_an_open_chain_and_drops_the_short_branch() {
        // A spine of two pieces with a short side branch at the joint.
        let pieces = vec![
            pl(&[(0.0, 0.0), (5.0, 0.0)]),
            pl(&[(10.0, 0.0), (5.0, 0.0)]),
            pl(&[(5.0, 0.0), (5.0, 1.0)]),
        ];
        let mut g = LayerGenerator::default();
        assert!(stitch_centerline(&pieces, &mut g));
        assert!(!g.closed);
        assert_eq!(g.polyline.points.len(), 3);
        assert_eq!(g.polyline.length(), mm(10.0) as f64);
        let (a, b) = (g.polyline.points[0], g.polyline.points[2]);
        assert!((a.x == 0 && b.x == mm(10.0)) || (a.x == mm(10.0) && b.x == 0));
    }

    #[test]
    fn stitch_centerline_closes_a_ring_of_pieces() {
        let pieces = vec![
            pl(&[(0.0, 0.0), (5.0, 0.0)]),
            pl(&[(5.0, 0.0), (5.0, 5.0)]),
            pl(&[(0.0, 5.0), (5.0, 5.0)]),
            pl(&[(0.0, 5.0), (0.0, 0.0)]),
        ];
        let mut g = LayerGenerator::default();
        assert!(stitch_centerline(&pieces, &mut g));
        assert!(g.closed);
        assert_eq!(g.polyline.points.len(), 5);
        assert_eq!(g.polyline.points[0], g.polyline.points[4]);
        assert_eq!(g.polyline.length(), mm(20.0) as f64);
        assert!(!stitch_centerline(&[], &mut g));
    }

    #[test]
    fn sample_at_s_walks_the_polyline() {
        let line = pl(&[(0.0, 0.0), (10.0, 0.0), (10.0, 10.0)]);
        let mut p = Point::new(0, 0);
        assert!(sample_at_s(&line, false, mm(4.0) as f64, &mut p));
        assert_eq!(p, Point::new(mm(4.0), 0));
        assert!(sample_at_s(&line, false, mm(13.0) as f64, &mut p));
        assert_eq!(p, Point::new(mm(10.0), mm(3.0)));
        // Open: clamped to the ends.
        assert!(sample_at_s(&line, false, -5.0, &mut p));
        assert_eq!(p, Point::new(0, 0));
        assert!(sample_at_s(&line, false, 1e12, &mut p));
        assert_eq!(p, Point::new(mm(10.0), mm(10.0)));
        // Closed: wraps by the length.
        assert!(sample_at_s(&line, true, mm(24.0) as f64, &mut p));
        assert_eq!(p, Point::new(mm(4.0), 0));
        assert!(!sample_at_s(&pl(&[(0.0, 0.0)]), false, 0.0, &mut p));
    }

    #[test]
    fn resample_and_sum_point_dist() {
        let line = pl(&[(0.0, 0.0), (10.0, 0.0)]);
        let pts = resample(&line, false, 5);
        assert_eq!(pts.len(), 5);
        for (i, p) in pts.iter().enumerate() {
            assert_eq!(*p, Point::new(mm(2.5 * i as f64), 0));
        }
        let shifted: Vec<Point> = pts.iter().map(|p| Point::new(p.x, p.y + 300)).collect();
        assert_eq!(sum_point_dist(&pts, &shifted), 1500.0);
        assert!(resample(&line, false, 1).is_empty());
    }

    #[test]
    fn rotate_closed_and_nearest_vertex() {
        let mut ring = pl(&[(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0), (0.0, 0.0)]);
        assert_eq!(nearest_vertex(&ring, &Point::new(mm(0.9), mm(1.2))), 2);
        rotate_closed(&mut ring, 2);
        assert_eq!(ring.points.len(), 5);
        assert_eq!(ring.points[0], Point::new(mm(1.0), mm(1.0)));
        assert_eq!(ring.points[4], ring.points[0]);
        assert_eq!(ring.points[1], Point::new(0, mm(1.0)));
    }

    #[test]
    fn edt_1d_is_the_squared_distance_transform() {
        let inf = 1e12f32;
        let f = [inf, inf, 0.0, inf, inf, inf, 0.0, inf];
        let n = f.len() as i32;
        let mut d = vec![0.0f32; f.len()];
        let mut v = vec![0i32; f.len()];
        let mut z = vec![0.0f32; f.len() + 1];
        edt_1d(&f, n, &mut d, &mut v, &mut z);
        assert_eq!(d, vec![4.0, 1.0, 0.0, 1.0, 4.0, 1.0, 0.0, 1.0]);
    }

    #[test]
    fn rasterize_expoly_fills_the_ring_and_leaves_the_hole() {
        let ex = square_with_hole();
        let mut img = Vec::new();
        // 24 x 24 cells of 1 mm, the origin at (-12, -12).
        rasterize_expoly(&ex, 24, 24, 1.0, (-12.0, -12.0), &mut img);
        let at = |x: usize, y: usize| img[x + y * 24];
        assert_eq!(at(3, 12), 1); // x in [-9, -8): the ring
        assert_eq!(at(12, 12), 0); // the hole
        assert_eq!(at(0, 0), 0); // outside
        assert_eq!(at(12, 3), 1);
        let filled: usize = img.iter().map(|&c| c as usize).sum();
        // 400 mm2 less the 64 mm2 hole, to within the boundary cells.
        assert!((300..=380).contains(&filled), "filled {filled}");
    }

    #[test]
    fn add_axis_quad_adds_two_triangles() {
        let mut its = indexed_triangle_set::default();
        add_axis_quad(&mut its, &Vec3f::new(1.0, 2.0, 3.0), 2, 2.0);
        assert_eq!(its.vertices.len(), 4);
        assert_eq!(its.indices.len(), 2);
        let s = 0.55f32 * 2.0;
        assert_eq!(its.vertices[0], Vec3f::new(1.0 - s, 2.0 - s, 3.0));
        assert_eq!(its.vertices[2], Vec3f::new(1.0 + s, 2.0 + s, 3.0));
        assert_eq!(its.vertices[3], Vec3f::new(1.0 - s, 2.0 + s, 3.0));
        assert_eq!(its.indices[1], Vec3crd::new(0, 2, 3));
    }

    #[test]
    fn polyline_from_pixels_open_and_closed() {
        let row: Vec<(i32, i32)> = (0..6).map(|x| (x, 2)).collect();
        let mut g = LayerGenerator::default();
        assert!(polyline_from_pixels(&row, 1.0, (0.0, 0.0), &mut g));
        assert!(!g.closed);
        assert_eq!(g.polyline.points.len(), 6);
        assert!((g.length - 500000.0).abs() < 8.0);

        // The border of a 4 x 4 block of pixels: every pixel has two neighbours.
        let mut ring: Vec<(i32, i32)> = Vec::new();
        for x in 0..4 {
            for y in 0..4 {
                if x == 0 || y == 0 || x == 3 || y == 3 {
                    ring.push((x, y));
                }
            }
        }
        let mut g = LayerGenerator::default();
        assert!(polyline_from_pixels(&ring, 1.0, (0.0, 0.0), &mut g));
        assert!(g.closed);
        assert_eq!(g.polyline.points.len(), 13);
        assert_eq!(g.polyline.points[0], g.polyline.points[12]);
    }

    #[test]
    fn extract_generator_of_a_strip_is_its_open_spine() {
        let strip = ExPolygon::new(rect(0.0, 0.0, 40.0, 4.0));
        let mut g = LayerGenerator::default();
        assert!(extract_generator(&strip, 0.45, &mut g));
        assert!(!g.closed);
        assert!(g.length > scale_(20.0), "length {}", g.length);
        for p in &g.polyline.points {
            // The spine runs along the middle of the strip.
            assert!(p.x >= 0 && p.x <= mm(40.0));
            assert!(p.y >= 0 && p.y <= mm(4.0));
        }
        let mut g2 = LayerGenerator::default();
        assert!(extract_generator(&strip, 0.45, &mut g2));
        assert_eq!(g.polyline.points, g2.polyline.points);
    }

    #[test]
    fn extract_generator_of_a_collar_is_a_closed_mid_loop() {
        let ex = square_with_hole();
        let mut g = LayerGenerator::default();
        assert!(extract_generator(&ex, 0.45, &mut g));
        assert!(g.closed);
        assert_eq!(g.polyline.points[0], g.polyline.points[g.polyline.points.len() - 1]);
        // The inset is 0.35 * max(0.2, 0.5 * (20 - 8)) = 2.1 mm: the hole of the
        // shrunk island is the 8 mm hole grown by 2.1 mm, so a loop between the rims.
        for p in &g.polyline.points {
            let r = p.x.abs().max(p.y.abs());
            assert!(r > mm(4.0) && r < mm(10.0), "r {r}");
        }
        assert!((g.length - 4.0 * mm(12.2) as f64).abs() < mm(0.5) as f64, "length {}", g.length);
    }

    #[test]
    fn medial_chart_build_stacks_generators() {
        let islands: Vec<ExPolygon> = (0..5).map(|_| square_with_hole()).collect();
        let zs: Vec<f64> = (0..5).map(|i| 0.2 * (i + 1) as f64).collect();
        let chart = MedialChart::build(&islands, &zs, 0.45).expect("chart");
        assert_eq!(chart.layer_count(), 5);
        assert!(!chart.empty());
        assert!(chart.sheet_mesh().is_none()); // fewer than 8 layers: the 2D fallback
        let g0 = chart.layer(0).expect("layer 0");
        assert!(g0.closed);
        // `extract_generator` starts with `out = LayerGenerator{}` (FillConformalChart.cpp:283),
        // which clears the z that `build` had just set (FillConformalChart.cpp:955-956).
        assert_eq!(g0.z, 0.0);
        // The same island on every layer: aligned generators coincide.
        for i in 1..5 {
            let g = chart.layer(i).expect("layer");
            assert_eq!(g.polyline.points[0], g0.polyline.points[0]);
            assert_eq!(g.length, g0.length);
        }
        assert!(chart.layer(5).is_none());
        assert!(MedialChart::build(&[], &[], 0.45).is_none());
    }

    #[test]
    fn medial_chart_build_makes_a_sheet_over_a_tall_stack() {
        // A 6 mm tall wall, 30 layers: the SDF ridge is the mid-plane of the wall.
        let islands: Vec<ExPolygon> = (0..30).map(|_| ExPolygon::new(rect(0.0, 0.0, 40.0, 8.0))).collect();
        let zs: Vec<f64> = (0..30).map(|i| 0.2 * (i + 1) as f64).collect();
        let chart = MedialChart::build(&islands, &zs, 0.45).expect("chart");
        assert_eq!(chart.layer_count(), 30);
        let sheet = chart.sheet_mesh().expect("sheet");
        assert_eq!(sheet.vertices.len() % 4, 0);
        assert_eq!(sheet.indices.len() * 2, sheet.vertices.len());
        let mut n = 0;
        for i in 0..30 {
            if let Some(g) = chart.layer(i) {
                n += 1;
                assert!(g.length > scale_(0.5));
            }
        }
        assert!(n >= 2);
    }
}
