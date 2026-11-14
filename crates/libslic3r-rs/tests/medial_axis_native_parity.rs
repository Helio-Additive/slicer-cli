//! `ExPolygon::medial_axis_thick_polylines` (ExPolygon.cpp:263-371, Geometry/MedialAxis.cpp:445-690,
//! Geometry/VoronoiOffset.cpp:640-967, over `src/boost_polygon_voronoi.rs`) against the C++ it
//! replaces, `clipper_z_sys::medial_axis_native` (crates/clipper-z-sys/shim/medial_axis_shim.cpp,
//! over boost::polygon): the same polylines in the same order, every point, every width bit for
//! bit, both endpoint flags.
//!
//! The inputs are what gap fill and thin walls hand it: thin rectangles, wedges, rings, curved
//! bands, junctions of strips, combs; polygons with holes; and the thin regions of the slices of
//! the Benchy of the parity fixtures.
//!
//! `MEDIAL_AXIS_PARITY_LOG=<file>` writes every mismatch there (the input and the first element
//! that differs); `MEDIAL_AXIS_PARITY_CASES=<n>` changes the number of generated cases per
//! category.

use slicer::clipper_utils::{difference_clib, offset2_ex_clib, OffsetJoinType};
use slicer::geometry::{medial_axis_counters, ExPolygon, MedialAxisThickPolyline, Point, Polygon};
use std::io::Write;
use std::sync::Mutex;

/// The counters of the port are process-wide: the tests run one at a time.
static ONE_AT_A_TIME: Mutex<()> = Mutex::new(());

type Pt = (i64, i64);
type Seg = [i64; 4];

struct Rng(u64);

impl Rng {
    /// A generator for one case: the state is a splitmix of the case number, so a case does
    /// not depend on the ones before it.
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

    /// In `[0, 1)`.
    fn unit(&mut self) -> f64 {
        (self.xorshift() >> 11) as f64 / (1u64 << 53) as f64
    }

    /// In `[lo, hi)`.
    fn f(&mut self, lo: f64, hi: f64) -> f64 {
        lo + (hi - lo) * self.unit()
    }
}

// ---------------------------------------------------------------------------
// Validity: boost wants segments that meet at endpoints only. The shim is not
// fed anything else.
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

fn signed_area2(p: &[Pt]) -> i128 {
    let mut a: i128 = 0;
    for i in 0..p.len() {
        let (q, r) = (p[i], p[(i + 1) % p.len()]);
        a += q.0 as i128 * r.1 as i128 - r.0 as i128 * q.1 as i128;
    }
    a
}

/// A vertex of the ring `p` that every other ring of the ExPolygon must be on one side of:
/// `q` strictly inside the ring `p` (crossing number; the rings do not cross each other).
fn inside(p: &[Pt], q: Pt) -> bool {
    let mut c = false;
    for i in 0..p.len() {
        let (a, b) = (p[i], p[(i + 1) % p.len()]);
        if (a.1 > q.1) != (b.1 > q.1) {
            let o = orient(a, b, q);
            if (b.1 > a.1 && o > 0) || (b.1 < a.1 && o < 0) {
                c = !c;
            }
        }
    }
    c
}

/// One input: the contour (counter-clockwise), the holes (clockwise), the widths.
#[derive(Clone, Debug)]
struct Case {
    contour: Vec<Pt>,
    holes: Vec<Vec<Pt>>,
    min_width: f64,
    max_width: f64,
}

/// The rings in millimetres to a valid ExPolygon in scaled units: turned by `angle`, moved,
/// rounded, the contour made counter-clockwise and the holes clockwise. `None` where rounding
/// left something that is not a simple polygon with its holes inside.
fn to_case(rings: &[Vec<(f64, f64)>], scale: f64, angle: f64, shift: (f64, f64), min_width: f64, max_width: f64) -> Option<Case> {
    let (sn, cs) = angle.sin_cos();
    let mut out: Vec<Vec<Pt>> = Vec::new();
    let mut segs: Vec<Seg> = Vec::new();
    for ring in rings {
        let mut p: Vec<Pt> = Vec::with_capacity(ring.len());
        for &(x, y) in ring {
            let q = (
                ((x * cs - y * sn + shift.0) * scale).round() as i64,
                ((x * sn + y * cs + shift.1) * scale).round() as i64,
            );
            if p.last() != Some(&q) {
                p.push(q);
            }
        }
        while p.len() > 1 && p.first() == p.last() {
            p.pop();
        }
        if p.len() < 3 {
            return None;
        }
        for i in 0..p.len() {
            let (a, b) = (p[i], p[(i + 1) % p.len()]);
            segs.push([a.0, a.1, b.0, b.1]);
        }
        out.push(p);
    }
    if !valid(&segs) {
        return None;
    }
    let mut it = out.into_iter();
    let mut contour = it.next()?;
    if signed_area2(&contour) < 0 {
        contour.reverse();
    }
    let mut holes: Vec<Vec<Pt>> = Vec::new();
    for mut h in it {
        if !inside(&contour, h[0]) || holes.iter().any(|o| inside(o, h[0])) {
            return None;
        }
        if signed_area2(&h) > 0 {
            h.reverse();
        }
        holes.push(h);
    }
    Some(Case { contour, holes, min_width, max_width })
}

// ---------------------------------------------------------------------------
// Generators (millimetres)
// ---------------------------------------------------------------------------

const CATEGORIES: [&str; 8] =
    ["thin rectangle", "tapered wedge", "thin ring", "curved band", "junction", "comb", "polygon with holes", "strip chain"];

/// The points of `a b` with `k` more in between, each pushed sideways by up to `jitter`.
fn subdivided(r: &mut Rng, a: (f64, f64), b: (f64, f64), k: u64, jitter: f64, out: &mut Vec<(f64, f64)>) {
    out.push(a);
    let (dx, dy) = (b.0 - a.0, b.1 - a.1);
    let len = (dx * dx + dy * dy).sqrt().max(1e-9);
    for i in 1..=k {
        let t = i as f64 / (k + 1) as f64;
        let j = r.f(-jitter, jitter);
        out.push((a.0 + t * dx - dy / len * j, a.1 + t * dy + dx / len * j));
    }
}

fn rings_for(r: &mut Rng, category: usize) -> Vec<Vec<(f64, f64)>> {
    match category {
        // A long thin rectangle, its long sides in one piece or in several nearly collinear ones.
        0 => {
            let (l, w) = (r.f(1.0, 30.0), r.f(0.05, 1.5));
            let k = r.below(6);
            let jitter = if r.below(2) == 0 { 0.0 } else { r.f(0.0, 0.03) };
            let mut p = Vec::new();
            subdivided(r, (0.0, 0.0), (l, 0.0), k, jitter, &mut p);
            let k2 = r.below(6);
            subdivided(r, (l, w), (0.0, w), k2, jitter, &mut p);
            vec![p]
        }
        // A wedge: one width at one end, another at the other (down to a point).
        1 => {
            let l = r.f(1.0, 25.0);
            let w0 = r.f(0.1, 2.0);
            let w1 = if r.below(3) == 0 { 0.0 } else { r.f(0.0, 1.0) };
            let skew = r.f(-0.5, 0.5);
            let mut p = Vec::new();
            let k = r.below(4);
            subdivided(r, (0.0, -w0 / 2.0), (l, skew - w1 / 2.0), k, 0.0, &mut p);
            let k2 = r.below(4);
            subdivided(r, (l, skew + w1 / 2.0), (0.0, w0 / 2.0), k2, 0.0, &mut p);
            vec![p]
        }
        // A thin ring: a polygonal circle and a hole, concentric or not.
        2 => {
            let rad = r.f(1.0, 10.0);
            let w = r.f(0.1, 1.2).min(rad * 0.8);
            let (n, m) = (4 + r.below(36), 4 + r.below(36));
            let m = if r.below(2) == 0 { n } else { m };
            let ph = if r.below(2) == 0 { 0.0 } else { r.f(0.0, 6.283) };
            let ecc = if r.below(2) == 0 { 0.0 } else { r.f(0.0, 0.6) * w };
            let outer: Vec<(f64, f64)> = (0..n)
                .map(|i| {
                    let a = 6.283185307179586 * i as f64 / n as f64;
                    (rad * a.cos(), rad * a.sin())
                })
                .collect();
            let inner: Vec<(f64, f64)> = (0..m)
                .map(|i| {
                    let a = ph + 6.283185307179586 * i as f64 / m as f64;
                    (ecc + (rad - w) * a.cos(), (rad - w) * a.sin())
                })
                .collect();
            vec![outer, inner]
        }
        // A thin band along an arc, its width changing along the way.
        3 => {
            let rad = r.f(3.0, 20.0);
            let span = r.f(0.3, 3.0);
            let n = 3 + r.below(30);
            let (w0, w1) = (r.f(0.1, 1.2), r.f(0.1, 1.2));
            let mut outer = Vec::new();
            let mut inner = Vec::new();
            for i in 0..=n {
                let t = i as f64 / n as f64;
                let a = span * t;
                let w = w0 + (w1 - w0) * t;
                outer.push(((rad + w / 2.0) * a.cos(), (rad + w / 2.0) * a.sin()));
                inner.push(((rad - w / 2.0) * a.cos(), (rad - w / 2.0) * a.sin()));
            }
            inner.reverse();
            outer.extend(inner);
            vec![outer]
        }
        // L, T and + junctions of thin strips.
        4 => {
            let (a, b) = (r.f(2.0, 20.0), r.f(2.0, 20.0));
            let (w1, w2) = (r.f(0.1, 1.2), r.f(0.1, 1.2));
            match r.below(3) {
                0 => vec![vec![(0.0, 0.0), (a, 0.0), (a, w1), (w2, w1), (w2, b + w1), (0.0, b + w1)]],
                1 => {
                    let c = r.f(0.3, (a - w2 - 0.3).max(0.4));
                    vec![vec![
                        (0.0, 0.0),
                        (a, 0.0),
                        (a, w1),
                        (c + w2, w1),
                        (c + w2, w1 + b),
                        (c, w1 + b),
                        (c, w1),
                        (0.0, w1),
                    ]]
                }
                _ => {
                    let c = r.f(0.3, (a - w2 - 0.3).max(0.4));
                    let d = r.f(1.0, 10.0);
                    vec![vec![
                        (0.0, 0.0),
                        (c, 0.0),
                        (c, -d),
                        (c + w2, -d),
                        (c + w2, 0.0),
                        (a, 0.0),
                        (a, w1),
                        (c + w2, w1),
                        (c + w2, w1 + b),
                        (c, w1 + b),
                        (c, w1),
                        (0.0, w1),
                    ]]
                }
            }
        }
        // A comb: a strip with teeth.
        5 => {
            let teeth = 2 + r.below(7);
            let pitch = r.f(0.8, 4.0);
            let wt = r.f(0.1, 0.9).min(pitch * 0.8);
            let wb = r.f(0.1, 1.5);
            let mut p = vec![(0.0, 0.0), (teeth as f64 * pitch, 0.0), (teeth as f64 * pitch, wb)];
            for i in (0..teeth).rev() {
                let x0 = i as f64 * pitch + r.f(0.0, pitch - wt);
                let h = r.f(0.5, 8.0);
                let taper = if r.below(3) == 0 { r.f(0.0, wt * 0.45) } else { 0.0 };
                p.push((x0 + wt, wb));
                p.push((x0 + wt - taper, wb + h));
                p.push((x0 + taper, wb + h));
                p.push((x0, wb));
            }
            p.push((0.0, wb));
            vec![p]
        }
        // A star-shaped polygon with zero to three holes, some of them close to the contour.
        6 => {
            let n = 5 + r.below(30);
            let base = r.f(1.0, 10.0);
            let rough = r.f(0.0, 0.5);
            let radii: Vec<f64> = (0..n).map(|_| base * (1.0 - rough * r.unit())).collect();
            let rmin = radii.iter().cloned().fold(f64::MAX, f64::min) * (3.141592653589793 / n as f64).cos();
            let outer: Vec<(f64, f64)> = radii
                .iter()
                .enumerate()
                .map(|(i, &q)| {
                    let a = 6.283185307179586 * i as f64 / n as f64;
                    (q * a.cos(), q * a.sin())
                })
                .collect();
            let mut rings = vec![outer];
            let m = r.below(4);
            for k in 0..m {
                let (c, hr) = if m == 1 {
                    ((0.0, 0.0), rmin * r.f(0.2, 0.97))
                } else {
                    let a = 6.283185307179586 * k as f64 / m as f64 + 0.3;
                    let d = rmin * 0.5;
                    ((d * a.cos(), d * a.sin()), d * (3.141592653589793 / m as f64).sin() * r.f(0.3, 0.95))
                };
                let hn = 3 + r.below(16);
                let ph = r.f(0.0, 6.283);
                rings.push(
                    (0..hn)
                        .map(|i| {
                            let a = ph + 6.283185307179586 * i as f64 / hn as f64;
                            (c.0 + hr * a.cos(), c.1 + hr * a.sin())
                        })
                        .collect(),
                );
            }
            rings
        }
        // A strip that turns: a chain of segments thickened to a varying width, mitred by hand.
        _ => {
            let n = 2 + r.below(8);
            let mut centre = vec![(0.0, 0.0)];
            let mut dir: f64 = r.f(-0.3, 0.3);
            for _ in 0..n {
                let l = r.f(0.8, 8.0);
                let last = *centre.last().unwrap();
                centre.push((last.0 + l * dir.cos(), last.1 + l * dir.sin()));
                dir += r.f(-0.7, 0.7);
            }
            let mut left = Vec::new();
            let mut right = Vec::new();
            for i in 0..centre.len() {
                let a = centre[i.saturating_sub(1)];
                let b = centre[(i + 1).min(centre.len() - 1)];
                let (dx, dy) = (b.0 - a.0, b.1 - a.1);
                let len = (dx * dx + dy * dy).sqrt();
                let w = r.f(0.08, 0.9);
                left.push((centre[i].0 - dy / len * w / 2.0, centre[i].1 + dx / len * w / 2.0));
                right.push((centre[i].0 + dy / len * w / 2.0, centre[i].1 - dx / len * w / 2.0));
            }
            left.reverse();
            right.extend(left);
            vec![right]
        }
    }
}

/// One generated case of a category, or `None` where rounding made it invalid.
fn generate(category: usize, case: u64) -> Option<Case> {
    let mut r = Rng::for_case(category as u64, case);
    let rings = rings_for(&mut r, category);
    // 1 mm is 1e5 units in the engine (SCALING_FACTOR 1e-5); 1e6 and 1e4 are there for the
    // arithmetic at other magnitudes.
    let scale = [1e5, 1e5, 1e5, 1e6, 1e4][r.below(5) as usize];
    // Axis-aligned inputs make cocircular sites; turned ones do not.
    let angle = match r.below(4) {
        0 => 0.0,
        1 => std::f64::consts::FRAC_PI_2 * r.below(4) as f64,
        _ => r.f(0.0, 6.283),
    };
    let shift = if r.below(3) == 0 { (0.0, 0.0) } else { (r.f(-100.0, 100.0), r.f(-100.0, 100.0)) };
    // The widths gap fill and thin walls pass: min 0.1..0.3 mm, max 0.5..2 mm; and others.
    let (min_width, max_width) = match r.below(6) {
        0 => (0.0, r.f(0.3, 3.0)),
        1 => (r.f(0.01, 0.1), r.f(1.0, 50.0)),
        2 => (r.f(0.2, 0.8), r.f(0.8, 1.6)),
        _ => (r.f(0.1, 0.3), r.f(0.5, 2.0)),
    };
    // Whole numbers of units, as the engine's callers compute them from coord_t; sometimes not.
    let (mut min_width, mut max_width) = (min_width * scale, max_width * scale);
    if r.below(4) != 0 {
        min_width = min_width.trunc();
        max_width = max_width.trunc();
    }
    to_case(&rings, scale, angle, shift, min_width, max_width)
}

// ---------------------------------------------------------------------------
// The comparison
// ---------------------------------------------------------------------------

fn to_expolygon(c: &Case) -> ExPolygon {
    let poly = |p: &[Pt]| Polygon { points: p.iter().map(|&(x, y)| Point::new(x, y)).collect() };
    ExPolygon { contour: poly(&c.contour), holes: c.holes.iter().map(|h| poly(h)).collect() }
}

#[derive(Default)]
struct Tally {
    cases: u64,
    skipped: u64,
    non_empty: u64,
    polylines: u64,
    multi_polyline: u64,
    endpoint_true: u64,
    endpoint_false: u64,
    loops: u64,
    mismatches: u64,
    first: Option<String>,
}

/// The first element in which the port's result differs from the shim's, if any.
fn first_difference(got: &[MedialAxisThickPolyline], want: &[clipper_z_sys::MaThickPolyline]) -> Option<String> {
    if got.len() != want.len() {
        return Some(format!("polyline count: rust {} c++ {}", got.len(), want.len()));
    }
    for (k, (g, w)) in got.iter().zip(want).enumerate() {
        if g.points.len() != w.points.len() {
            return Some(format!("polyline {k}: point count: rust {} c++ {}", g.points.len(), w.points.len()));
        }
        for (i, (p, q)) in g.points.iter().zip(&w.points).enumerate() {
            if (p.x, p.y) != *q {
                return Some(format!("polyline {k} point {i}: rust ({}, {}) c++ ({}, {})", p.x, p.y, q.0, q.1));
            }
        }
        if g.width.len() != w.width.len() {
            return Some(format!("polyline {k}: width count: rust {} c++ {}", g.width.len(), w.width.len()));
        }
        for (i, (a, b)) in g.width.iter().zip(&w.width).enumerate() {
            if a.to_bits() != b.to_bits() {
                return Some(format!(
                    "polyline {k} width {i}: rust {a:?} ({:#018x}) c++ {b:?} ({:#018x})",
                    a.to_bits(),
                    b.to_bits()
                ));
            }
        }
        if g.endpoints != w.endpoints {
            return Some(format!("polyline {k} endpoints: rust {:?} c++ {:?}", g.endpoints, w.endpoints));
        }
    }
    None
}

fn compare(c: &Case, what: &str, tally: &mut Tally, log: &mut Option<std::fs::File>) {
    let want = clipper_z_sys::medial_axis_native(&c.contour, &c.holes, c.min_width, c.max_width);
    let mut got: Vec<MedialAxisThickPolyline> = Vec::new();
    to_expolygon(c).medial_axis_thick_polylines(c.min_width, c.max_width, &mut got);
    tally.cases += 1;
    if !want.is_empty() {
        tally.non_empty += 1;
    }
    if want.len() > 1 {
        tally.multi_polyline += 1;
    }
    tally.polylines += want.len() as u64;
    for w in &want {
        for e in [w.endpoints.0, w.endpoints.1] {
            if e {
                tally.endpoint_true += 1;
            } else {
                tally.endpoint_false += 1;
            }
        }
        if w.points.first() == w.points.last() {
            tally.loops += 1;
        }
    }
    if let Some(d) = first_difference(&got, &want) {
        tally.mismatches += 1;
        let msg = format!("{what}: {d}\n  input: {c:?}");
        if let Some(f) = log.as_mut() {
            let _ = writeln!(f, "{msg}");
        }
        if tally.first.is_none() {
            tally.first = Some(msg);
        }
    }
}

fn open_log() -> Option<std::fs::File> {
    std::env::var_os("MEDIAL_AXIS_PARITY_LOG")
        .map(|p| std::fs::OpenOptions::new().create(true).append(true).open(p).expect("MEDIAL_AXIS_PARITY_LOG"))
}

fn report(log: &mut Option<std::fs::File>, line: String) {
    println!("{line}");
    if let Some(f) = log.as_mut() {
        let _ = writeln!(f, "{line}");
    }
}

/// The shim resets the endpoint flags in `clear()` where the engine's C++ does not
/// (Polyline.hpp:265-268; `MEDIAL_AXIS_CLEAR_CPP`, on by default since R900). The comparison
/// with the shim is made with that one gate off: everything else is the default path. Set
/// before the first gate is read (the crate snapshots the environment once).
fn shim_behaviour() {
    std::env::set_var("MEDIAL_AXIS_CLEAR_CPP", "0");
}

#[test]
fn generated_expolygons_against_the_shim() {
    shim_behaviour();
    let _guard = ONE_AT_A_TIME.lock().unwrap_or_else(|e| e.into_inner());
    let per_category: u64 =
        std::env::var("MEDIAL_AXIS_PARITY_CASES").ok().and_then(|v| v.parse().ok()).unwrap_or(7_000);
    let mut log = open_log();
    let before = medial_axis_counters();
    let mut total = Tally::default();
    for (category, name) in CATEGORIES.iter().enumerate() {
        let mut t = Tally::default();
        for case in 0..per_category {
            match generate(category, case) {
                Some(c) => compare(&c, &format!("{name} #{case}"), &mut t, &mut log),
                None => t.skipped += 1,
            }
        }
        report(
            &mut log,
            format!(
                "{name}: {} cases ({} invalid skipped), {} non-empty, {} polylines, {} multi-polyline, endpoint flags {} true / {} false, {} loops, {} mismatches",
                t.cases, t.skipped, t.non_empty, t.polylines, t.multi_polyline, t.endpoint_true, t.endpoint_false, t.loops, t.mismatches
            ),
        );
        assert!(t.cases * 10 >= per_category * 8, "{name}: only {} of {per_category} cases were valid", t.cases);
        assert!(t.non_empty * 4 >= t.cases, "{name}: only {} of {} cases gave a medial axis", t.non_empty, t.cases);
        total.cases += t.cases;
        total.skipped += t.skipped;
        total.non_empty += t.non_empty;
        total.polylines += t.polylines;
        total.multi_polyline += t.multi_polyline;
        total.endpoint_true += t.endpoint_true;
        total.endpoint_false += t.endpoint_false;
        total.loops += t.loops;
        total.mismatches += t.mismatches;
        if total.first.is_none() {
            total.first = t.first;
        }
    }
    let after = medial_axis_counters();
    let (front, back, short, joined) = (
        after.front_extended - before.front_extended,
        after.back_extended - before.back_extended,
        after.short_removed - before.short_removed,
        after.joined - before.joined,
    );
    report(
        &mut log,
        format!(
            "total: {} cases, {} non-empty, {} polylines, {} multi-polyline, endpoint flags {} true / {} false, {} loops; \
             post-processing: {front} first points extended, {back} last points extended, {short} short polylines removed, {joined} joined; {} mismatches",
            total.cases, total.non_empty, total.polylines, total.multi_polyline, total.endpoint_true, total.endpoint_false, total.loops, total.mismatches
        ),
    );
    if let Some(first) = &total.first {
        panic!("{} of {} cases differ from the shim; the first: {first}", total.mismatches, total.cases);
    }
    if per_category >= 7_000 {
        assert!(total.cases >= 50_000, "{} cases", total.cases);
    }
    assert!(total.non_empty * 2 >= total.cases, "{} of {} cases gave a medial axis", total.non_empty, total.cases);
    assert!(total.multi_polyline > 0 && total.endpoint_true > 0 && total.endpoint_false > 0 && total.loops > 0);
    assert!(front > 0 && back > 0 && short > 0, "post-processing branches: {front} {back} {short} {joined}");
    if per_category >= 7_000 {
        // Joining after a removal is rare: about one case in two thousand.
        assert!(joined > 0, "no polylines were joined after a removal");
    }
}

/// The thin regions of real slices, found as PerimeterGenerator finds its thin walls
/// (PerimeterGenerator.cpp:960-975): what an opening by half a width takes away from the slice,
/// opened again by half the minimum width.
#[test]
fn benchy_thin_regions_against_the_shim() {
    shim_behaviour();
    let _guard = ONE_AT_A_TIME.lock().unwrap_or_else(|e| e.into_inner());
    let stl = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../tests/rust-engine/fixtures/3DBenchy.stl");
    let mesh = slicer::stl::read_stl_file(&stl).expect("fixture STL");
    let zs: Vec<f64> = (0..96).map(|i| 0.1 + 0.5 * i as f64).collect();
    let layers = slicer::triangle_mesh_slicer::slice_mesh(&mesh, &zs);
    let mut log = open_log();
    let before = medial_axis_counters();
    let mut t = Tally::default();
    let mut slices = 0;
    for (li, layer) in layers.iter().enumerate() {
        if layer.is_empty() {
            continue;
        }
        slices += 1;
        for (wi, &(width, min_width)) in [(0.45, 0.133), (0.9, 0.2), (1.6, 0.133), (3.0, 0.3)].iter().enumerate() {
            let opened = offset2_ex_clib(layer, -width / 2.0, width / 2.0, OffsetJoinType::Miter);
            let thin = difference_clib(layer, &opened);
            let expp = offset2_ex_clib(&thin, -min_width / 2.0, min_width / 2.0, OffsetJoinType::Miter);
            for (ei, ex) in expp.iter().enumerate() {
                let ring = |p: &Polygon| p.points.iter().map(|q| (q.x, q.y)).collect::<Vec<Pt>>();
                let c = Case {
                    contour: ring(&ex.contour),
                    holes: ex.holes.iter().map(ring).collect(),
                    min_width: (min_width / 0.00001f64).trunc(),
                    max_width: (2.0 * width / 0.00001f64).trunc(),
                };
                let mut segs: Vec<Seg> = Vec::new();
                for p in std::iter::once(&c.contour).chain(&c.holes) {
                    for i in 0..p.len() {
                        let (a, b) = (p[i], p[(i + 1) % p.len()]);
                        segs.push([a.0, a.1, b.0, b.1]);
                    }
                }
                if c.contour.len() < 3 || !valid(&segs) {
                    t.skipped += 1;
                    continue;
                }
                compare(&c, &format!("Benchy z {:.1} width {wi} region {ei} (layer {li})", zs[li]), &mut t, &mut log);
            }
        }
    }
    let after = medial_axis_counters();
    report(
        &mut log,
        format!(
            "Benchy: {slices} slices, {} thin regions ({} not simple, skipped), {} non-empty, {} polylines, {} multi-polyline, \
             endpoint flags {} true / {} false, {} loops; post-processing: {} first points extended, {} last points extended, \
             {} short polylines removed, {} joined; {} mismatches",
            t.cases,
            t.skipped,
            t.non_empty,
            t.polylines,
            t.multi_polyline,
            t.endpoint_true,
            t.endpoint_false,
            t.loops,
            after.front_extended - before.front_extended,
            after.back_extended - before.back_extended,
            after.short_removed - before.short_removed,
            after.joined - before.joined,
            t.mismatches
        ),
    );
    if let Some(first) = &t.first {
        panic!("{} of {} thin regions differ from the shim; the first: {first}", t.mismatches, t.cases);
    }
    assert!(slices >= 80, "{slices} slices");
    assert!(t.cases >= 500, "{} thin regions", t.cases);
    assert!(t.non_empty * 4 >= t.cases, "{} of {} thin regions gave a medial axis", t.non_empty, t.cases);
}
