//! `src/boost_polygon_voronoi.rs` against `boost::polygon::construct_voronoi` itself, compiled in
//! `boost-voronoi-sys`: every vertex bit for bit and in the same order, every half-edge with its
//! links and flags, every cell. The inputs are what the engine feeds the builder (closed contours
//! with holes) and what makes it take its rare paths: exactly and nearly cocircular sites, grids,
//! collinear runs, slivers, at four coordinate magnitudes up to 1e8.
//!
//! `BOOST_VORONOI_PARITY_LOG=<file>` writes every mismatch there (the input and the first element
//! that differs); `BOOST_VORONOI_PARITY_CASES=<n>` changes the number of generated diagrams.

use boost_voronoi_sys::TestSite;
use slicer::boost_polygon_voronoi::{
    construct_voronoi, counters, distance_predicate, event_comparison_ss, fast_ps, find_distance_to_point_arc,
    find_distance_to_segment_arc, node_comparison_predicate, BeachLineNodeKey, CircleEvent, CircleFormationPredicate,
    KPredicateResult, LazyCircleFormationFunctor, MpCircleFormationFunctor, Point2D, SiteEvent, VoronoiDiagram,
};
use std::sync::Mutex;

/// The counters of the port are process-wide: the tests run one at a time, so that what
/// `against_boost` reads is what its own diagrams did.
static ONE_AT_A_TIME: Mutex<()> = Mutex::new(());

type Seg = [i32; 4];
type Pt = (i32, i32);

struct Rng(u64);

impl Rng {
    /// A generator for one case: the state is a splitmix of the case number, so a case does
    /// not depend on the ones before it (nor on the thread that runs it).
    fn for_case(category: u64, case: u64) -> Rng {
        let mut z = (category << 40 ^ case).wrapping_add(0x9E37_79B9_7F4A_7C15).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z ^= z >> 30;
        z = z.wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^= z >> 31;
        Rng(z | 1)
    }

    fn xorshift(&mut self) -> u64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        self.0
    }

    fn below(&mut self, n: u64) -> u64 {
        self.xorshift() % n
    }

    /// In `lo..=hi`.
    fn range(&mut self, lo: i64, hi: i64) -> i64 {
        lo + (self.xorshift() % (hi - lo + 1) as u64) as i64
    }

    /// In `[0, 1)`.
    fn unit(&mut self) -> f64 {
        (self.xorshift() >> 11) as f64 / (1u64 << 53) as f64
    }
}

// ---------------------------------------------------------------------------
// Validity: boost wants segments that meet at endpoints only.
// ---------------------------------------------------------------------------

fn orient(a: Pt, b: Pt, c: Pt) -> i128 {
    (b.0 as i128 - a.0 as i128) * (c.1 as i128 - a.1 as i128) - (b.1 as i128 - a.1 as i128) * (c.0 as i128 - a.0 as i128)
}

/// `p` on the closed segment `a b`, given that the three are collinear.
fn within(a: Pt, b: Pt, p: Pt) -> bool {
    p.0 >= a.0.min(b.0) && p.0 <= a.0.max(b.0) && p.1 >= a.1.min(b.1) && p.1 <= a.1.max(b.1)
}

/// Whether two segments meet anywhere else than in a common endpoint.
fn conflict(s: &Seg, t: &Seg) -> bool {
    let (a, b, c, d) = ((s[0], s[1]), (s[2], s[3]), (t[0], t[1]), (t[2], t[3]));
    if (a == c && b == d) || (a == d && b == c) {
        return true;
    }
    let (o1, o2, o3, o4) = (orient(a, b, c), orient(a, b, d), orient(c, d, a), orient(c, d, b));
    if ((o1 > 0 && o2 < 0) || (o1 < 0 && o2 > 0)) && ((o3 > 0 && o4 < 0) || (o3 < 0 && o4 > 0)) {
        return true;
    }
    (o1 == 0 && within(a, b, c) && c != a && c != b)
        || (o2 == 0 && within(a, b, d) && d != a && d != b)
        || (o3 == 0 && within(c, d, a) && a != c && a != d)
        || (o4 == 0 && within(c, d, b) && b != c && b != d)
}

fn valid(segs: &[Seg]) -> bool {
    if segs.is_empty() || segs.iter().any(|s| s[0] == s[2] && s[1] == s[3]) {
        return false;
    }
    let mut order: Vec<usize> = (0..segs.len()).collect();
    order.sort_by_key(|&i| segs[i][0].min(segs[i][2]));
    for (k, &i) in order.iter().enumerate() {
        let xmax = segs[i][0].max(segs[i][2]);
        for &j in &order[k + 1..] {
            if segs[j][0].min(segs[j][2]) > xmax {
                break;
            }
            if conflict(&segs[i], &segs[j]) {
                return false;
            }
        }
    }
    true
}

// ---------------------------------------------------------------------------
// Generators
// ---------------------------------------------------------------------------

/// The closed contour through `pts` as segments; consecutive equal points are dropped.
fn ring(pts: &[Pt], out: &mut Vec<Seg>) -> bool {
    let mut p: Vec<Pt> = Vec::with_capacity(pts.len());
    for &q in pts {
        if p.last() != Some(&q) {
            p.push(q);
        }
    }
    while p.len() > 1 && p.first() == p.last() {
        p.pop();
    }
    if p.len() < 3 {
        return false;
    }
    for i in 0..p.len() {
        let (a, b) = (p[i], p[(i + 1) % p.len()]);
        out.push([a.0, a.1, b.0, b.1]);
    }
    true
}

/// The open chain through `pts`.
fn chain(pts: &[Pt], out: &mut Vec<Seg>) {
    for w in pts.windows(2) {
        if w[0] != w[1] {
            out.push([w[0].0, w[0].1, w[1].0, w[1].1]);
        }
    }
}

fn magnitude(r: &mut Rng) -> i64 {
    [100i64, 10_000, 1_000_000, 100_000_000][r.below(4) as usize]
}

/// A centre such that a shape of radius `rad` stays within `m`.
fn centre(r: &mut Rng, m: i64, rad: i64) -> (i64, i64) {
    let room = (m - rad).max(0);
    (r.range(-room, room), r.range(-room, room))
}

fn polar(c: (i64, i64), rad: f64, ang: f64) -> Pt {
    ((c.0 as f64 + rad * ang.cos()).round() as i32, (c.1 as f64 + rad * ang.sin()).round() as i32)
}

fn sorted_angles(r: &mut Rng, n: usize) -> Vec<f64> {
    let mut a: Vec<f64> = (0..n).map(|_| r.unit() * std::f64::consts::TAU).collect();
    a.sort_by(|x, y| x.partial_cmp(y).unwrap());
    a
}

/// Points on an ellipse at random angles: convex up to the rounding.
fn convex(r: &mut Rng, out: &mut Vec<Seg>) {
    let m = magnitude(r);
    let rad = r.range(m / 4, m / 2).max(3);
    let c = centre(r, m, rad);
    let n = 3 + r.below(14) as usize;
    let squash = 0.3 + 0.7 * r.unit();
    let pts: Vec<Pt> = sorted_angles(r, n)
        .iter()
        .map(|&a| ((c.0 as f64 + rad as f64 * a.cos()).round() as i32, (c.1 as f64 + rad as f64 * squash * a.sin()).round() as i32))
        .collect();
    ring(&pts, out);
}

/// A star-shaped contour: random radii at sorted angles.
fn star(r: &mut Rng, m: i64, c: (i64, i64), rad: i64, n: usize, out: &mut Vec<Seg>) {
    let lo = 0.25 + 0.5 * r.unit();
    let angles = sorted_angles(r, n);
    let pts: Vec<Pt> = angles.iter().map(|&a| polar(c, rad as f64 * (lo + (1.0 - lo) * r.unit()), a)).collect();
    let _ = m;
    ring(&pts, out);
}

fn non_convex(r: &mut Rng, out: &mut Vec<Seg>) {
    let m = magnitude(r);
    let rad = r.range(m / 4, m / 2).max(4);
    let c = centre(r, m, rad);
    let n = 4 + r.below(20) as usize;
    star(r, m, c, rad, n, out);
}

fn rect(x0: i64, y0: i64, x1: i64, y1: i64, out: &mut Vec<Seg>) {
    ring(&[(x0 as i32, y0 as i32), (x1 as i32, y0 as i32), (x1 as i32, y1 as i32), (x0 as i32, y1 as i32)], out);
}

/// Axis-aligned rectangles: one with rectangular holes, or a grid of them. Squares and equal
/// gaps make four and more sites cocircular.
fn rectangles(r: &mut Rng, out: &mut Vec<Seg>) {
    let m = magnitude(r);
    match r.below(3) {
        0 => {
            let (w, h) = (r.range(m / 8, m).max(8), r.range(m / 8, m).max(8));
            let (x0, y0) = (r.range(-m, m - w), r.range(-m, m - h));
            rect(x0, y0, x0 + w, y0 + h, out);
            // Holes on a grid inside.
            let k = 1 + r.below(3) as i64;
            let (cw, ch) = (w / k, h / k);
            for i in 0..k {
                for j in 0..k {
                    if r.below(3) == 0 || cw < 6 || ch < 6 {
                        continue;
                    }
                    let (mx, my) = (r.range(1, cw / 3), r.range(1, ch / 3));
                    rect(x0 + i * cw + mx, y0 + j * ch + my, x0 + (i + 1) * cw - mx, y0 + (j + 1) * ch - my, out);
                }
            }
        }
        1 => {
            // A grid of equal squares with equal gaps.
            let k = 1 + r.below(4) as i64;
            let side = r.range(2, (m / (2 * k)).max(2));
            let gap = r.range(1, side);
            let pitch = side + gap;
            let (x0, y0) = (r.range(-m, m - k * pitch), r.range(-m, m - k * pitch));
            for i in 0..k {
                for j in 0..k {
                    if r.below(5) != 0 {
                        rect(x0 + i * pitch, y0 + j * pitch, x0 + i * pitch + side, y0 + j * pitch + side, out);
                    }
                }
            }
        }
        _ => {
            // A square with a concentric square hole.
            let half = r.range(4, m / 2);
            let inner = r.range(1, half - 1);
            let c = centre(r, m, half);
            rect(c.0 - half, c.1 - half, c.0 + half, c.1 + half, out);
            rect(c.0 - inner, c.1 - inner, c.0 + inner, c.1 + inner, out);
        }
    }
}

/// Regular n-gons, rounded to the grid, alone or concentric: nearly cocircular sites.
fn regular(r: &mut Rng, out: &mut Vec<Seg>) {
    let m = magnitude(r);
    let rad = r.range(m / 8, m / 2).max(4);
    let c = centre(r, m, rad);
    let n = 3 + r.below(22) as usize;
    let phase = if r.below(2) == 0 { 0.0 } else { r.unit() };
    let gon = |rad: f64, n: usize, phase: f64| -> Vec<Pt> {
        (0..n).map(|i| polar(c, rad, (i as f64 + phase) * std::f64::consts::TAU / n as f64)).collect()
    };
    ring(&gon(rad as f64, n, phase), out);
    if r.below(2) == 0 && rad >= 16 {
        let inner = rad as f64 * (0.2 + 0.6 * r.unit());
        let n2 = if r.below(2) == 0 { n } else { 3 + r.below(22) as usize };
        ring(&gon(inner, n2, if r.below(2) == 0 { phase } else { r.unit() }), out);
    }
}

/// Lattice points exactly on a circle (`x^2 + y^2 = R^2` for an `R` with many representations),
/// scaled: exactly cocircular endpoints, so the vertices of many circle events coincide.
fn lattice_circle(r: &mut Rng, out: &mut Vec<Seg>) {
    let m = magnitude(r);
    let base = [5i64, 25, 65, 85, 325, 425, 1105][r.below(7) as usize];
    let base = if base * 2 > m { 5 } else { base };
    let mut pts: Vec<(i64, i64)> = Vec::new();
    for x in -base..=base {
        let y2 = base * base - x * x;
        let y = (y2 as f64).sqrt().round() as i64;
        if y * y == y2 {
            pts.push((x, y));
            if y != 0 {
                pts.push((x, -y));
            }
        }
    }
    pts.sort_by(|a, b| (a.1 as f64).atan2(a.0 as f64).partial_cmp(&(b.1 as f64).atan2(b.0 as f64)).unwrap());
    // Keep a random subset, at least three.
    let keep = 3 + r.below(pts.len() as u64 - 2) as usize;
    while pts.len() > keep {
        let i = r.below(pts.len() as u64) as usize;
        pts.remove(i);
    }
    let scale = r.range(1, (m / (2 * base)).max(1));
    let c = centre(r, m, base * scale);
    let p: Vec<Pt> = pts.iter().map(|&(x, y)| ((c.0 + x * scale) as i32, (c.1 + y * scale) as i32)).collect();
    ring(&p, out);
    if r.below(3) == 0 && scale >= 4 {
        // The same points on a smaller concentric circle.
        let s2 = r.range(1, scale / 2);
        let p: Vec<Pt> = pts.iter().map(|&(x, y)| ((c.0 + x * s2) as i32, (c.1 + y * s2) as i32)).collect();
        ring(&p, out);
    }
}

/// A contour whose edges are cut at lattice points on them: consecutive collinear edges.
fn collinear(r: &mut Rng, out: &mut Vec<Seg>) {
    let m = magnitude(r);
    let unit = r.range(1, (m / 40).max(1));
    let n = 3 + r.below(6) as usize;
    let rad = 12i64;
    let c = centre(r, m, rad * unit * 2);
    let lo = 0.4 + 0.3 * r.unit();
    // Corners on a coarse lattice, then scaled: every edge has lattice points on it.
    let corners: Vec<(i64, i64)> = sorted_angles(r, n)
        .iter()
        .map(|&a| {
            let rr = rad as f64 * (lo + (1.0 - lo) * r.unit());
            ((rr * a.cos()).round() as i64, (rr * a.sin()).round() as i64)
        })
        .collect();
    let mut pts: Vec<Pt> = Vec::new();
    for i in 0..corners.len() {
        let (a, b) = (corners[i], corners[(i + 1) % corners.len()]);
        let cuts = 1 + r.below(4) as i64;
        for k in 0..cuts {
            // a + (b - a) * k / cuts, in units of `unit / cuts` when that is whole.
            let (x, y) = (a.0 * cuts + (b.0 - a.0) * k, a.1 * cuts + (b.1 - a.1) * k);
            if unit % cuts == 0 {
                pts.push(((c.0 + x * (unit / cuts)) as i32, (c.1 + y * (unit / cuts)) as i32));
            } else if k == 0 {
                pts.push(((c.0 + a.0 * unit) as i32, (c.1 + a.1 * unit) as i32));
            }
        }
    }
    ring(&pts, out);
}

/// Long thin triangles and quadrilaterals.
fn sliver(r: &mut Rng, out: &mut Vec<Seg>) {
    let m = magnitude(r);
    let len = r.range(m / 2, m).max(8);
    let thick = r.range(1, (len / 1000).max(2));
    let ang = r.unit() * std::f64::consts::TAU;
    let c = centre(r, m, len);
    let (dx, dy) = (ang.cos(), ang.sin());
    let at = |t: f64, off: f64| -> Pt {
        ((c.0 as f64 + t * dx - off * dy).round() as i32, (c.1 as f64 + t * dy + off * dx).round() as i32)
    };
    let (l, th) = (len as f64, thick as f64);
    let pts = match r.below(3) {
        0 => vec![at(-l, 0.0), at(l, -th), at(l, th)],
        1 => vec![at(-l, -th), at(l, -th), at(l, th), at(-l, th)],
        _ => vec![at(-l, 0.0), at(0.0, -th), at(l, 0.0), at(0.0, th)],
    };
    ring(&pts, out);
}

/// A star-shaped contour with star-shaped holes.
fn with_holes(r: &mut Rng, out: &mut Vec<Seg>) {
    let m = magnitude(r);
    let rad = r.range(m / 3, m / 2).max(40);
    let c = centre(r, m, rad);
    // The outer contour stays outside 0.6 rad.
    let n = 5 + r.below(16) as usize;
    let pts: Vec<Pt> = sorted_angles(r, n).iter().map(|&a| polar(c, rad as f64 * (0.62 + 0.38 * r.unit()), a)).collect();
    // Enough corners, spread enough, to keep the 0.5 rad disc inside.
    let dense: Vec<Pt> = (0..24)
        .map(|i| polar(c, rad as f64 * (0.62 + 0.38 * r.unit()), i as f64 * std::f64::consts::TAU / 24.0))
        .collect();
    ring(if r.below(2) == 0 { &dense } else { &pts }, out);
    let holes = 1 + r.below(4) as usize;
    for _ in 0..holes {
        let hr = (rad as f64 * (0.03 + 0.1 * r.unit())) as i64;
        if hr < 3 {
            continue;
        }
        let d = rad as f64 * 0.3 * r.unit();
        let a = r.unit() * std::f64::consts::TAU;
        let hc = (c.0 + (d * a.cos()) as i64, c.1 + (d * a.sin()) as i64);
        let hn = 3 + r.below(8) as usize;
        star(r, m, hc, hr, hn, out);
    }
}

/// Segments that do not touch, and open chains.
fn loose(r: &mut Rng, out: &mut Vec<Seg>) {
    let m = magnitude(r);
    if r.below(2) == 0 {
        let n = 1 + r.below(8) as usize;
        for _ in 0..n {
            let s = [r.range(-m, m) as i32, r.range(-m, m) as i32, r.range(-m, m) as i32, r.range(-m, m) as i32];
            if (s[0], s[1]) != (s[2], s[3]) && !out.iter().any(|t| conflict(&s, t) || touches(&s, t)) {
                out.push(s);
            }
        }
    } else {
        // An x-monotone chain, possibly with vertical steps.
        let n = 2 + r.below(10) as usize;
        let mut x = r.range(-m, 0);
        let step = (m / n as i64).max(1);
        let mut pts: Vec<Pt> = Vec::new();
        for _ in 0..n {
            pts.push((x as i32, r.range(-m, m) as i32));
            x += if r.below(5) == 0 { 0 } else { r.range(1, step) };
        }
        chain(&pts, out);
    }
}

fn touches(s: &Seg, t: &Seg) -> bool {
    let (a, b, c, d) = ((s[0], s[1]), (s[2], s[3]), (t[0], t[1]), (t[2], t[3]));
    a == c || a == d || b == c || b == d
}

/// Staircases and combs on a small lattice: many equal distances.
fn lattice_shape(r: &mut Rng, out: &mut Vec<Seg>) {
    let m = magnitude(r);
    let steps = 1 + r.below(6) as i64;
    let unit = r.range(1, (m / (2 * (steps + 2))).max(1));
    let c = centre(r, m, unit * (steps + 2));
    let mut pts: Vec<(i64, i64)> = Vec::new();
    match r.below(3) {
        0 => {
            // A staircase.
            pts.push((0, 0));
            for i in 0..steps {
                pts.push((i + 1, i));
                pts.push((i + 1, i + 1));
            }
            pts.push((0, steps));
        }
        1 => {
            // A comb.
            pts.push((0, 0));
            pts.push((2 * steps + 1, 0));
            for i in (0..=steps).rev() {
                pts.push((2 * i + 1, 3));
                pts.push((2 * i, 3));
                if i > 0 {
                    pts.push((2 * i, 1));
                    pts.push((2 * i - 1, 1));
                }
            }
        }
        _ => {
            // A plus.
            let a = 1 + r.below(3) as i64;
            pts.extend([(a, 0), (a + 1, 0), (a + 1, a), (2 * a + 1, a), (2 * a + 1, a + 1), (a + 1, a + 1)]);
            pts.extend([(a + 1, 2 * a + 1), (a, 2 * a + 1), (a, a + 1), (0, a + 1), (0, a), (a, a)]);
        }
    }
    // One of the eight symmetries of the lattice.
    let sym = r.below(8);
    let p: Vec<Pt> = pts
        .iter()
        .map(|&(x, y)| {
            let (x, y) = if sym & 1 != 0 { (y, x) } else { (x, y) };
            let (x, y) = (if sym & 2 != 0 { -x } else { x }, if sym & 4 != 0 { -y } else { y });
            ((c.0 + x * unit) as i32, (c.1 + y * unit) as i32)
        })
        .collect();
    ring(&p, out);
}

/// A contour symmetric about both axes through its centre.
fn symmetric(r: &mut Rng, out: &mut Vec<Seg>) {
    let m = magnitude(r);
    let rad = r.range(m / 8, m / 2).max(6);
    let c = centre(r, m, rad);
    let n = 1 + r.below(5) as usize;
    let mut q: Vec<(i64, i64)> = sorted_angles(r, n)
        .iter()
        .map(|&a| {
            let a = a / 4.0; // first quadrant
            let rr = rad as f64 * (0.5 + 0.5 * r.unit());
            ((rr * a.cos()).round() as i64, (rr * a.sin()).round() as i64)
        })
        .filter(|&(x, y)| x > 0 && y > 0)
        .collect();
    q.dedup();
    let mut pts: Vec<Pt> = Vec::new();
    let axis = r.below(2) == 0;
    if axis {
        pts.push(((c.0 + rad) as i32, c.1 as i32));
    }
    pts.extend(q.iter().map(|&(x, y)| ((c.0 + x) as i32, (c.1 + y) as i32)));
    if axis {
        pts.push((c.0 as i32, (c.1 + rad) as i32));
    }
    pts.extend(q.iter().rev().map(|&(x, y)| ((c.0 - x) as i32, (c.1 + y) as i32)));
    if axis {
        pts.push(((c.0 - rad) as i32, c.1 as i32));
    }
    pts.extend(q.iter().map(|&(x, y)| ((c.0 - x) as i32, (c.1 - y) as i32)));
    if axis {
        pts.push((c.0 as i32, (c.1 - rad) as i32));
    }
    pts.extend(q.iter().rev().map(|&(x, y)| ((c.0 + x) as i32, (c.1 - y) as i32)));
    ring(&pts, out);
}

/// A slicer-like contour: a smooth closed curve of 50 to 2000 vertices with small noise, and
/// sometimes a hole of the same kind.
fn smooth(r: &mut Rng, out: &mut Vec<Seg>) {
    let m = [1_000_000i64, 100_000_000][r.below(2) as usize];
    let rad = r.range(m / 3, m / 2);
    let c = centre(r, m, rad);
    let curve = |r: &mut Rng, rad: f64, n: usize, out: &mut Vec<Seg>| {
        let (k1, k2) = (2 + r.below(5), 5 + r.below(9));
        let (a1, a2) = (0.15 * r.unit(), 0.06 * r.unit());
        let (p1, p2) = (r.unit() * 6.0, r.unit() * 6.0);
        let noise = rad * 1e-4 * r.unit();
        let pts: Vec<Pt> = (0..n)
            .map(|i| {
                let t = i as f64 * std::f64::consts::TAU / n as f64;
                let rr = rad * (1.0 + a1 * (k1 as f64 * t + p1).sin() + a2 * (k2 as f64 * t + p2).sin()) + noise * (r.unit() - 0.5);
                polar(c, rr, t)
            })
            .collect();
        ring(&pts, out);
    };
    let n = 50 + r.below(1951) as usize;
    curve(r, rad as f64 * 0.75, n, out);
    if r.below(2) == 0 {
        let n = 50 + r.below(500) as usize;
        let inner = rad as f64 * (0.1 + 0.3 * r.unit());
        curve(r, inner, n, out);
    }
}

const CATEGORIES: [(&str, fn(&mut Rng, &mut Vec<Seg>)); 11] = [
    ("convex", convex),
    ("non-convex star-shaped", non_convex),
    ("axis-aligned rectangles", rectangles),
    ("regular n-gons", regular),
    ("lattice circles", lattice_circle),
    ("collinear consecutive edges", collinear),
    ("thin slivers", sliver),
    ("contour with holes", with_holes),
    ("loose segments and chains", loose),
    ("staircases, combs, pluses", lattice_shape),
    ("doubly symmetric", symmetric),
];

/// The input of one case: a valid set of segments, the directions and the order shuffled now
/// and then (the builder sorts them, but not stably, and records the direction).
fn generate(category: usize, case: u64, gen: fn(&mut Rng, &mut Vec<Seg>)) -> Vec<Seg> {
    let mut r = Rng::for_case(category as u64, case);
    for _ in 0..50 {
        let mut segs = Vec::new();
        gen(&mut r, &mut segs);
        if !valid(&segs) {
            continue;
        }
        if r.below(2) == 0 {
            for s in segs.iter_mut() {
                if r.below(2) == 0 {
                    *s = [s[2], s[3], s[0], s[1]];
                }
            }
        }
        if r.below(3) == 0 {
            for i in (1..segs.len()).rev() {
                segs.swap(i, r.below(i as u64 + 1) as usize);
            }
        }
        return segs;
    }
    vec![[0, 0, 10, 0], [10, 0, 0, 10], [0, 10, 0, 0]]
}

// ---------------------------------------------------------------------------
// The comparison
// ---------------------------------------------------------------------------

fn idx(o: Option<usize>) -> i32 {
    o.map_or(-1, |i| i as i32)
}

/// `None` when the port's diagram is the library's; otherwise the first element that differs.
fn difference(segs: &[Seg]) -> Option<String> {
    let want = boost_voronoi_sys::construct_segments(segs);
    let got = match std::panic::catch_unwind(|| {
        let mut vd = VoronoiDiagram::new();
        construct_voronoi(segs, &mut vd);
        vd
    }) {
        Ok(vd) => vd,
        Err(e) => {
            let msg = e.downcast_ref::<String>().cloned().or_else(|| e.downcast_ref::<&str>().map(|s| s.to_string()));
            return Some(format!("the port panicked: {}", msg.unwrap_or_default()));
        }
    };
    if got.num_vertices() != want.vertices.len() || got.num_edges() != want.edges.len() || got.num_cells() != want.cells.len() {
        return Some(format!(
            "sizes: {} vertices, {} edges, {} cells; boost has {}, {}, {}",
            got.num_vertices(),
            got.num_edges(),
            got.num_cells(),
            want.vertices.len(),
            want.edges.len(),
            want.cells.len()
        ));
    }
    for (i, (g, w)) in got.vertices().iter().zip(&want.vertices).enumerate() {
        if g.x().to_bits() != w.0.to_bits() || g.y().to_bits() != w.1.to_bits() {
            return Some(format!(
                "vertex {i}: ({:?}, {:?}) [{:#018x}, {:#018x}]; boost has ({:?}, {:?}) [{:#018x}, {:#018x}]",
                g.x(),
                g.y(),
                g.x().to_bits(),
                g.y().to_bits(),
                w.0,
                w.1,
                w.0.to_bits(),
                w.1.to_bits()
            ));
        }
    }
    for (i, (g, w)) in got.edges().iter().zip(&want.edges).enumerate() {
        let g = (
            idx(g.vertex0()),
            idx(got.vertex1(i)),
            idx(g.twin()),
            idx(g.next()),
            idx(g.prev()),
            idx(g.cell()),
            g.is_primary(),
            g.is_curved(),
            got.is_finite(i),
        );
        let w = (w.vertex0, w.vertex1, w.twin, w.next, w.prev, w.cell, w.is_primary, w.is_curved, w.is_finite);
        if g != w {
            return Some(format!("edge {i} (vertex0, vertex1, twin, next, prev, cell, primary, curved, finite): {g:?}; boost has {w:?}"));
        }
    }
    for (i, (g, w)) in got.cells().iter().zip(&want.cells).enumerate() {
        let g = (g.source_index(), idx(g.incident_edge()), g.source_category() as i32, g.contains_point(), g.contains_segment(), g.is_degenerate());
        let w = (w.source_index, w.incident_edge, w.source_category, w.contains_point, w.contains_segment, w.is_degenerate);
        if g != w {
            return Some(format!("cell {i} (source_index, incident_edge, source_category, point, segment, degenerate): {g:?}; boost has {w:?}"));
        }
    }
    None
}

#[derive(Default)]
struct Tally {
    cases: usize,
    segments: usize,
    mismatches: usize,
    first: Option<String>,
}

fn record(tally: &Mutex<Vec<Tally>>, category: usize, name: &str, case: u64, segs: &[Seg], log: &Mutex<Option<std::fs::File>>) {
    let diff = difference(segs);
    let mut t = tally.lock().unwrap();
    let t = &mut t[category];
    t.cases += 1;
    t.segments += segs.len();
    if let Some(d) = diff {
        t.mismatches += 1;
        let shown = if segs.len() <= 64 { format!("{segs:?}") } else { format!("{} segments, the first {:?}", segs.len(), &segs[..8]) };
        let text = format!("{name}, case {case}: {d}\n  input: {shown}");
        if let Some(f) = log.lock().unwrap().as_mut() {
            use std::io::Write;
            let _ = writeln!(f, "{name}, case {case}: {d}\n  input: {segs:?}");
        }
        if t.first.is_none() {
            t.first = Some(text);
        }
    }
}

// ---------------------------------------------------------------------------
// Real contours: slices of the Benchy of the parity fixtures.
// ---------------------------------------------------------------------------

fn load_stl(path: &std::path::Path) -> Vec<[[f32; 3]; 3]> {
    let bytes = std::fs::read(path).expect("the Benchy fixture");
    let n = u32::from_le_bytes(bytes[80..84].try_into().unwrap()) as usize;
    let f = |o: usize| f32::from_le_bytes(bytes[o..o + 4].try_into().unwrap());
    (0..n)
        .map(|t| {
            let o = 84 + 50 * t + 12;
            [[f(o), f(o + 4), f(o + 8)], [f(o + 12), f(o + 16), f(o + 20)], [f(o + 24), f(o + 28), f(o + 32)]]
        })
        .collect()
}

/// The contours of the mesh at height `z`, in units of `1 / scale` mm. An edge of the mesh
/// gives the same point to both of its triangles, so the loops close exactly.
fn slice(tris: &[[[f32; 3]; 3]], z: f32, scale: f64) -> Vec<Seg> {
    let cut = |a: [f32; 3], b: [f32; 3]| -> Pt {
        // Always from the lower end, so that both triangles compute the same point.
        let (a, b) = if (a[2], a[0], a[1]) <= (b[2], b[0], b[1]) { (a, b) } else { (b, a) };
        let t = (z as f64 - a[2] as f64) / (b[2] as f64 - a[2] as f64);
        (
            ((a[0] as f64 + t * (b[0] as f64 - a[0] as f64)) * scale).round() as i32,
            ((a[1] as f64 + t * (b[1] as f64 - a[1] as f64)) * scale).round() as i32,
        )
    };
    let mut segs: Vec<Seg> = Vec::new();
    for t in tris {
        let above = [t[0][2] >= z, t[1][2] >= z, t[2][2] >= z];
        let mut pts: Vec<Pt> = Vec::new();
        for i in 0..3 {
            let j = (i + 1) % 3;
            if above[i] != above[j] {
                pts.push(cut(t[i], t[j]));
            }
        }
        if pts.len() == 2 && pts[0] != pts[1] {
            segs.push([pts[0].0, pts[0].1, pts[1].0, pts[1].1]);
        }
    }
    segs
}

#[test]
fn against_boost() {
    let _one = ONE_AT_A_TIME.lock().unwrap_or_else(|e| e.into_inner());
    let cases: usize = std::env::var("BOOST_VORONOI_PARITY_CASES").ok().and_then(|s| s.parse().ok()).unwrap_or(110_000);
    let large = (cases / 250).max(1);
    let log = Mutex::new(std::env::var("BOOST_VORONOI_PARITY_LOG").ok().map(|p| std::fs::File::create(p).expect("the log file")));
    let names: Vec<&str> = CATEGORIES.iter().map(|c| c.0).chain(["smooth contours of 50-2000 vertices", "Benchy slices"]).collect();
    let tally = Mutex::new((0..names.len()).map(|_| Tally::default()).collect::<Vec<_>>());
    let before = counters();

    // The panics of the port are caught and reported with their input; keep them quiet.
    let hook = std::panic::take_hook();
    std::panic::set_hook(Box::new(|_| {}));

    let threads = std::thread::available_parallelism().map_or(4, |n| n.get()).min(16);
    std::thread::scope(|scope| {
        for thread in 0..threads {
            let (tally, log, names) = (&tally, &log, &names);
            scope.spawn(move || {
                // The generated small diagrams, the categories in turn.
                for case in (thread..cases).step_by(threads) {
                    let category = case % CATEGORIES.len();
                    let segs = generate(category, case as u64, CATEGORIES[category].1);
                    record(tally, category, names[category], case as u64, &segs, log);
                }
                for case in (thread..large).step_by(threads) {
                    let segs = generate(CATEGORIES.len(), case as u64, smooth);
                    record(tally, CATEGORIES.len(), names[CATEGORIES.len()], case as u64, &segs, log);
                }
            });
        }
    });

    // Real contours.
    let stl = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../tests/rust-engine/fixtures/3DBenchy.stl");
    let tris = load_stl(&stl);
    let (zmin, zmax) = tris.iter().flatten().fold((f32::MAX, f32::MIN), |(lo, hi), v| (lo.min(v[2]), hi.max(v[2])));
    let layers = (cases / 2000).clamp(4, 60);
    let mut skipped = 0;
    for layer in 0..layers {
        let z = zmin + (zmax - zmin) * (layer as f32 + 0.37) / layers as f32;
        for scale in [1_000.0, 1_000_000.0] {
            let segs = slice(&tris, z, scale);
            if !valid(&segs) {
                skipped += 1;
                continue;
            }
            record(&tally, names.len() - 1, names[names.len() - 1], (layer * 2) as u64 + (scale > 1e4) as u64, &segs, &log);
        }
    }
    std::panic::set_hook(hook);

    let after = counters();
    let tally = tally.into_inner().unwrap();
    let total: usize = tally.iter().map(|t| t.cases).sum();
    let mut report = String::new();
    for (name, t) in names.iter().zip(&tally) {
        report += &format!("{name}: {} diagrams, {} segments, {} differ\n", t.cases, t.segments, t.mismatches);
    }
    report += &format!("Benchy slices skipped as not valid input after rounding: {skipped}\n");
    report += &format!(
        "exact fallback calls: ppp {}, pps {}, pss {}, sss {}; conjugate branches of the sqrt expressions {}\n",
        after.exact_ppp - before.exact_ppp,
        after.exact_pps - before.exact_pps,
        after.exact_pss - before.exact_pss,
        after.exact_sss - before.exact_sss,
        after.sqrt_expr_conjugate - before.sqrt_expr_conjugate
    );
    report += &format!(
        "degenerate edges removed {}, vertices merged away {}; beach line hints missed {}, keys found {}; circle event ties {}; fast_ps undefined {}; collinear starts {}\n",
        after.degenerate_edges_removed - before.degenerate_edges_removed,
        after.degenerate_vertices_removed - before.degenerate_vertices_removed,
        after.beach_line_hint_missed - before.beach_line_hint_missed,
        after.beach_line_key_found - before.beach_line_key_found,
        after.circle_event_ties - before.circle_event_ties,
        after.fast_ps_undefined - before.fast_ps_undefined,
        after.collinear_init - before.collinear_init
    );
    println!("{total} diagrams\n{report}");
    if let Some(f) = log.lock().unwrap().as_mut() {
        use std::io::Write;
        let _ = writeln!(f, "{total} diagrams\n{report}");
    }

    let failures: Vec<&str> = tally.iter().filter_map(|t| t.first.as_deref()).collect();
    assert!(failures.is_empty(), "the port differs from boost::polygon.\n{report}\nThe first of each category:\n{}", failures.join("\n"));
    assert!(total >= cases, "{total} diagrams compared, {cases} wanted");
    assert!(tally[names.len() - 1].cases >= layers, "only {} Benchy slices were valid input", tally[names.len() - 1].cases);

    // The rare paths were taken.
    if cases >= 100_000 {
        assert!(after.exact_ppp > before.exact_ppp, "mp_circle_formation_functor::ppp was never reached\n{report}");
        assert!(after.exact_pps > before.exact_pps, "mp_circle_formation_functor::pps was never reached\n{report}");
        assert!(after.exact_pss > before.exact_pss, "mp_circle_formation_functor::pss was never reached\n{report}");
        assert!(after.exact_sss > before.exact_sss, "mp_circle_formation_functor::sss was never reached\n{report}");
        assert!(after.degenerate_edges_removed > before.degenerate_edges_removed, "no degenerate edge was removed\n{report}");
        assert!(after.degenerate_vertices_removed > before.degenerate_vertices_removed, "no vertex was merged away\n{report}");
        assert!(after.fast_ps_undefined > before.fast_ps_undefined, "fast_ps always decided\n{report}");
        assert!(after.collinear_init > before.collinear_init, "init_beach_line_collinear_sites was never reached\n{report}");
        assert!(after.sqrt_expr_conjugate > before.sqrt_expr_conjugate, "the sqrt expressions never took the conjugate branch\n{report}");
        assert!(after.circle_event_ties > before.circle_event_ties, "no two circle events were ever tied\n{report}");
    }
}

#[test]
fn smallest_inputs() {
    let _one = ONE_AT_A_TIME.lock().unwrap_or_else(|e| e.into_inner());
    for segs in [
        vec![],
        vec![[0, 0, 10, 0]],
        vec![[0, 0, 0, 10]],
        vec![[0, 0, 0, 10], [0, 10, 0, 25]],
        vec![[0, 0, 0, 10], [0, 20, 0, 30], [0, 40, 0, 50]],
        vec![[0, 0, 10, 0], [20, 0, 30, 0]],
        vec![[0, 0, 10, 10], [0, 0, 10, 10]],
        vec![[0, 0, 1000, 0], [1000, 0, 1000, 1000], [1000, 1000, 0, 1000], [0, 1000, 0, 0]],
    ] {
        let d = difference(&segs);
        assert!(d.is_none(), "{segs:?}: {}", d.unwrap());
    }
}

// ---------------------------------------------------------------------------
// The predicates and the circle functors one call at a time. A diagram only shows a
// rounding difference of a predicate when it flips a decision; here the doubles themselves
// are compared.
// ---------------------------------------------------------------------------

fn site(s: &TestSite) -> SiteEvent {
    let (p0, p1) = (Point2D::new(s[0], s[1]), Point2D::new(s[2], s[3]));
    let mut e = if s[5] != 0 {
        let mut e = SiteEvent::from_points(p1, p0);
        e.inverse();
        e
    } else {
        SiteEvent::from_points(p0, p1)
    };
    e.set_sorted_index(s[4] as usize);
    e
}

/// A frame for one configuration: a centre and the extent of what is placed around it, so
/// that the coordinates are large and their differences small as often as not.
struct Frame {
    c: (i64, i64),
    ext: i64,
}

impl Frame {
    fn new(r: &mut Rng) -> Frame {
        let m = magnitude(r);
        let ext = match r.below(4) {
            0 => m,
            1 => (m / 100).max(2),
            2 => (m / 10_000).max(2),
            _ => r.range(2, m),
        };
        let room = m - ext.min(m);
        Frame { c: (r.range(-room, room), r.range(-room, room)), ext }
    }

    fn xy(&self, r: &mut Rng) -> (i64, i64) {
        (self.c.0 + r.range(-self.ext, self.ext), self.c.1 + r.range(-self.ext, self.ext))
    }
}

fn point_site(p: (i64, i64), r: &mut Rng) -> TestSite {
    [p.0 as i32, p.1 as i32, p.0 as i32, p.1 as i32, r.below(40) as i32, 0]
}

/// A segment site as the builder holds one: `point0 < point1` unless it is inverse.
fn segment_site(a: (i64, i64), b: (i64, i64), r: &mut Rng) -> TestSite {
    let b = if a == b { (b.0 + 1, b.1) } else { b };
    let (lo, hi) = if a < b { (a, b) } else { (b, a) };
    if r.below(2) == 0 {
        [lo.0 as i32, lo.1 as i32, hi.0 as i32, hi.1 as i32, r.below(40) as i32, 0]
    } else {
        [hi.0 as i32, hi.1 as i32, lo.0 as i32, lo.1 as i32, r.below(40) as i32, 1]
    }
}

/// Three sites of the kinds `kind` asks for (0 = ppp, 1 = pps, 2 = pss, 3 = sss; the points
/// first), often in the positions where the functors branch: collinear points, parallel
/// segments, a segment parallel to the two points, a point where the two lines meet.
fn triple(kind: i32, r: &mut Rng) -> [TestSite; 3] {
    let f = Frame::new(r);
    let special = r.below(4) == 0;
    let small = |r: &mut Rng| if r.below(3) == 0 { r.range(-2, 2) } else { 0 };
    match kind {
        0 => {
            let (a, b) = (f.xy(r), f.xy(r));
            let c = if special {
                let (num, den) = (r.range(-8, 8), r.range(1, 8));
                (a.0 + (b.0 - a.0) * num / den + small(r), a.1 + (b.1 - a.1) * num / den + small(r))
            } else {
                f.xy(r)
            };
            [point_site(a, r), point_site(b, r), point_site(c, r)]
        }
        1 => {
            let (a, b) = (f.xy(r), f.xy(r));
            let s0 = f.xy(r);
            let s1 = if special {
                // Parallel to a b.
                let (num, den) = (r.range(-4, 4), r.range(1, 4));
                (s0.0 + (b.0 - a.0) * num / den + small(r), s0.1 + (b.1 - a.1) * num / den + small(r))
            } else {
                f.xy(r)
            };
            [point_site(a, r), point_site(b, r), segment_site(s0, s1, r)]
        }
        2 => {
            let (s0, s1) = (f.xy(r), f.xy(r));
            let t0 = f.xy(r);
            let (p, t1);
            match if special { r.below(3) } else { 3 } {
                0 => {
                    // Parallel segments.
                    let (num, den) = (r.range(-4, 4), r.range(1, 4));
                    t1 = (t0.0 + (s1.0 - s0.0) * num / den, t0.1 + (s1.1 - s0.1) * num / den);
                    p = f.xy(r);
                }
                1 => {
                    // The point where the two lines meet: both segments point away from it.
                    p = f.xy(r);
                    let (u, v) = ((r.range(-9, 9), r.range(-9, 9)), (r.range(-9, 9), r.range(-9, 9)));
                    let (k1, k2, k3, k4) = (r.range(1, 5), r.range(6, 12), r.range(1, 5), r.range(6, 12));
                    let s = segment_site((p.0 + u.0 * k1, p.1 + u.1 * k1), (p.0 + u.0 * k2, p.1 + u.1 * k2), r);
                    let t = segment_site((p.0 + v.0 * k3, p.1 + v.1 * k3), (p.0 + v.0 * k4, p.1 + v.1 * k4), r);
                    return [point_site(p, r), s, t];
                }
                2 => {
                    // The point is an endpoint of one of the segments.
                    t1 = f.xy(r);
                    p = if r.below(2) == 0 { s0 } else { t1 };
                }
                _ => {
                    t1 = f.xy(r);
                    p = f.xy(r);
                }
            }
            [point_site(p, r), segment_site(s0, s1, r), segment_site(t0, t1, r)]
        }
        _ => {
            let (a0, a1, b0, c0) = (f.xy(r), f.xy(r), f.xy(r), f.xy(r));
            let b1 = if special {
                let (num, den) = (r.range(-4, 4), r.range(1, 4));
                (b0.0 + (a1.0 - a0.0) * num / den, b0.1 + (a1.1 - a0.1) * num / den)
            } else {
                f.xy(r)
            };
            let c1 = if special && r.below(2) == 0 { a0 } else { f.xy(r) };
            [segment_site(a0, a1, r), segment_site(b0, b1, r), segment_site(c0, c1, r)]
        }
    }
}

/// The bits agree, or boost's value is not finite and the port's is not either (a circle
/// that does not exist: the functors divide by zero, and what `frexp` stores for an infinity
/// is not specified).
fn same(got: &[f64], want: &[f64]) -> bool {
    got.iter().zip(want).all(|(g, w)| g.to_bits() == w.to_bits() || (!w.is_finite() && !g.is_finite()))
}

fn circle(c: &CircleEvent) -> [f64; 3] {
    [c.x(), c.y(), c.lower_x()]
}

#[test]
fn single_predicates_against_boost() {
    let _one = ONE_AT_A_TIME.lock().unwrap_or_else(|e| e.into_inner());
    let cases: u64 = std::env::var("BOOST_VORONOI_PARITY_CASES").ok().and_then(|s| s.parse().ok()).unwrap_or(110_000) * 4;
    let threads = std::thread::available_parallelism().map_or(4, |n| n.get()).min(16) as u64;
    let failures: Mutex<Vec<String>> = Mutex::new(Vec::new());
    let not_finite = std::sync::atomic::AtomicU64::new(0);
    let fail = |text: String| {
        let mut f = failures.lock().unwrap();
        if f.len() < 20 {
            f.push(text);
        }
    };
    std::thread::scope(|scope| {
        for thread in 0..threads {
            let (fail, not_finite) = (&fail, &not_finite);
            scope.spawn(move || {
                let mut case = thread;
                while case < cases {
                    let mut r = Rng::for_case(1000, case);
                    let kind = (case % 4) as i32;
                    let index = 1 + r.below(3) as i32;
                    let t = triple(kind, &mut r);
                    let (s1, s2, s3) = (site(&t[0]), site(&t[1]), site(&t[2]));

                    // The lazy functor (with its fall-back to the exact one).
                    let want = boost_voronoi_sys::test_lazy_circle(kind, index, &t[0], &t[1], &t[2]);
                    if want.iter().any(|v| !v.is_finite()) {
                        not_finite.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
                    }
                    let mut c = CircleEvent::new(0.0, 0.0, 0.0);
                    let mut lazy = LazyCircleFormationFunctor::default();
                    match kind {
                        0 => lazy.ppp(&s1, &s2, &s3, &mut c),
                        1 => lazy.pps(&s1, &s2, &s3, index, &mut c),
                        2 => lazy.pss(&s1, &s2, &s3, index, &mut c),
                        _ => lazy.sss(&s1, &s2, &s3, &mut c),
                    }
                    if !same(&circle(&c), &want) {
                        fail(format!("lazy functor, kind {kind}, index {index}, sites {t:?}: {:?}; boost has {want:?}", circle(&c)));
                    }

                    // The exact functor, every coordinate.
                    let want = boost_voronoi_sys::test_mp_circle(kind, index, &t[0], &t[1], &t[2]);
                    let mut c = CircleEvent::new(0.0, 0.0, 0.0);
                    let mut mp = MpCircleFormationFunctor::default();
                    match kind {
                        0 => mp.ppp(&s1, &s2, &s3, &mut c, true, true, true),
                        1 => mp.pps(&s1, &s2, &s3, index, &mut c, true, true, true),
                        2 => mp.pss(&s1, &s2, &s3, index, &mut c, true, true, true),
                        _ => mp.sss(&s1, &s2, &s3, &mut c, true, true, true),
                    }
                    if !same(&circle(&c), &want) {
                        fail(format!("exact functor, kind {kind}, index {index}, sites {t:?}: {:?}; boost has {want:?}", circle(&c)));
                    }

                    // The predicate, the sites in any order of kinds.
                    let mut o = [t[0], t[1], t[2]];
                    for i in (1..3).rev() {
                        o.swap(i, r.below(i as u64 + 1) as usize);
                    }
                    let (want_exists, want) = boost_voronoi_sys::test_circle_formation(&o[0], &o[1], &o[2]);
                    let mut c = CircleEvent::new(0.0, 0.0, 0.0);
                    let exists = CircleFormationPredicate::default().call(&site(&o[0]), &site(&o[1]), &site(&o[2]), &mut c);
                    if exists != want_exists || !same(&circle(&c), &want) {
                        fail(format!(
                            "circle_formation_predicate, sites {o:?}: {exists} {:?}; boost has {want_exists} {want:?}",
                            circle(&c)
                        ));
                    }

                    // The distance predicate and its distances, for a point right of the sites.
                    let f = Frame::new(&mut r);
                    let mk = |r: &mut Rng| {
                        if r.below(2) == 0 {
                            point_site(f.xy(r), r)
                        } else {
                            let (a, b) = (f.xy(r), f.xy(r));
                            segment_site(a, if r.below(6) == 0 { (a.0, b.1) } else { b }, r)
                        }
                    };
                    let (left, right) = (mk(&mut r), mk(&mut r));
                    let p = f.xy(&mut r);
                    let p = [if r.below(4) == 0 { p.0 } else { p.0.max(f.c.0 + f.ext) } as i32, p.1 as i32];
                    let reverse = r.below(2) == 0;
                    let (want_less, want_fast, want_d1, want_d2) = boost_voronoi_sys::test_distance(&left, &right, p, reverse);
                    let (l, rt, pt) = (site(&left), site(&right), Point2D::new(p[0], p[1]));
                    let d1 = find_distance_to_point_arc(&l, pt);
                    let d2 = find_distance_to_segment_arc(&rt, pt);
                    let fast = if !l.is_segment() && rt.is_segment() {
                        Some(match fast_ps(&l, &rt, pt, reverse) {
                            KPredicateResult::Less => -1,
                            KPredicateResult::Undefined => 0,
                            KPredicateResult::More => 1,
                        })
                    } else {
                        None
                    };
                    if distance_predicate(&l, &rt, pt) != want_less || fast != want_fast || !same(&[d1, d2], &[want_d1, want_d2]) {
                        fail(format!(
                            "distance_predicate, left {left:?}, right {right:?}, point {p:?}, reverse {reverse}: {} {fast:?} {d1:?} {d2:?}; boost has {want_less} {want_fast:?} {want_d1:?} {want_d2:?}",
                            distance_predicate(&l, &rt, pt)
                        ));
                    }

                    // The order of the beach line and of the site events.
                    let (l2, r2) = (mk(&mut r), mk(&mut r));
                    let want = boost_voronoi_sys::test_node_comparison(&left, &right, &l2, &r2);
                    let got = node_comparison_predicate(
                        &BeachLineNodeKey::new2(l, rt),
                        &BeachLineNodeKey::new2(site(&l2), site(&r2)),
                    );
                    if got != want {
                        fail(format!("node_comparison_predicate, ({left:?}, {right:?}) and ({l2:?}, {r2:?}): {got}; boost has {want}"));
                    }
                    let (a, b) = (left, if r.below(4) == 0 { [left[0], left[1], r2[2], r2[3], r2[4], 0] } else { l2 });
                    let (a, b) = (if a[5] != 0 { [a[2], a[3], a[0], a[1], a[4], 0] } else { a }, if b[5] != 0 { [b[2], b[3], b[0], b[1], b[4], 0] } else { b });
                    let want = boost_voronoi_sys::test_event_comparison(&a, &b);
                    let got = event_comparison_ss(&site(&a), &site(&b));
                    if got != want {
                        fail(format!("event_comparison_predicate, {a:?} and {b:?}: {got}; boost has {want}"));
                    }

                    case += threads;
                }
            });
        }
    });
    let failures = failures.into_inner().unwrap();
    println!(
        "{cases} configurations of each predicate; boost's circle was not finite in {}",
        not_finite.load(std::sync::atomic::Ordering::Relaxed)
    );
    if let Ok(path) = std::env::var("BOOST_VORONOI_PARITY_LOG") {
        let _ = std::fs::write(format!("{path}.predicates"), failures.join("\n"));
    }
    assert!(failures.is_empty(), "the port's predicates differ from boost::polygon's:\n{}", failures.join("\n"));
}
