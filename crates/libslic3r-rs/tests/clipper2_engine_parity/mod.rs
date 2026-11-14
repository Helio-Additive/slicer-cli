//! What `clipper2_engine_native_parity.rs` (Clipper2 without Z) and
//! `clipper2_engine_z_native_parity.rs` (with `USINGZ`) share: the deterministic generator of
//! the cases, the call into the C++ reference (`cz2_ref_xy_*` / `cz2_ref_z_*` of
//! `clipper2-z-sys`, built from the Clipper2 the engine pins) and the comparison.
//!
//! A case is made from its own seed, so a failure names the case and prints its input.
//! `CLIPPER2_PARITY_LOG=<file>` appends every failing case with its input and both outputs.
//! `CLIPPER2_PARITY_TRACE=<file>` writes every case before it runs, to find an input on which
//! one side dies. `CLIPPER2_PARITY_SUMMARY=<file>` appends the counts of every category.
//! `CLIPPER2_PARITY_ONLY=<category>#<index>` runs one case of one category.
//! `CLIPPER2_PARITY_SCALE=<k>` runs k times as many cases of every category.
#![allow(dead_code)]

use clipper2_z_sys as sys;
use std::fmt::Write as _;
use std::io::Write as _;

/// clipper.core.h:68 `MAX_COORD = INT64_MAX >> 2`: the largest coordinate the library takes.
pub const MAX_COORD: i64 = i64::MAX >> 2;

/// x, y, z. z is 0 in the build without Z.
pub type Pt = [i64; 3];
pub type Path = Vec<Pt>;
pub type Paths = Vec<Path>;

pub fn xorshift(s: &mut u64) -> u64 {
    *s ^= *s << 13;
    *s ^= *s >> 7;
    *s ^= *s << 17;
    *s
}

/// The seed of case `index` of a test: splitmix64 of the two, never zero.
pub fn case_seed(test: u64, index: u64) -> u64 {
    let mut z = test.wrapping_mul(0x9E37_79B9_7F4A_7C15).wrapping_add(index).wrapping_add(0x9E37_79B9_7F4A_7C15);
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    (z ^ (z >> 31)) | 1
}

/// A value in `-m..=m`.
pub fn r(m: i64, s: &mut u64) -> i64 {
    if m <= 0 {
        return 0;
    }
    (xorshift(s) % (2 * m as u64 + 1)) as i64 - m
}

/// A value in `0..n`.
pub fn below(n: u64, s: &mut u64) -> u64 {
    xorshift(s) % n
}

/// A value in `lo..=hi`.
pub fn between(lo: i64, hi: i64, s: &mut u64) -> i64 {
    lo + below((hi - lo + 1) as u64, s) as i64
}

pub fn chance(num: u64, den: u64, s: &mut u64) -> bool {
    below(den, s) < num
}

// -------------------------------------------------------------------------------------------
// Shapes, in a small space of whole numbers; `Space::put` takes them to the coordinates.
// -------------------------------------------------------------------------------------------

type Shape = Vec<(i64, i64)>;

/// `n` points anywhere in `-k..=k`: usually self-intersecting.
fn rand_poly(n: usize, k: i64, s: &mut u64) -> Shape {
    (0..n).map(|_| (r(k, s), r(k, s))).collect()
}

/// `n` points around the origin in the order of their angle: usually simple.
fn ring(n: usize, k: i64, s: &mut u64) -> Shape {
    let mut angles: Vec<f64> = (0..n).map(|_| below(1 << 20, s) as f64 / (1u64 << 20) as f64 * std::f64::consts::TAU).collect();
    angles.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let reverse = chance(1, 2, s);
    let mut out: Shape = angles
        .iter()
        .map(|a| {
            let rad = k as f64 * (0.4 + 0.6 * (below(1000, s) as f64 / 1000.0));
            ((rad * a.cos()).round() as i64, (rad * a.sin()).round() as i64)
        })
        .collect();
    if reverse {
        out.reverse();
    }
    out
}

/// A star of `n` points joined every `step`-th: self-intersecting, winding number above 1.
fn star(n: usize, step: usize, k: i64) -> Shape {
    (0..n)
        .map(|i| {
            let a = (i * step % n) as f64 / n as f64 * std::f64::consts::TAU;
            ((k as f64 * a.cos()).round() as i64, (k as f64 * a.sin()).round() as i64)
        })
        .collect()
}

fn rect(x0: i64, y0: i64, x1: i64, y1: i64, ccw: bool) -> Shape {
    let mut v = vec![(x0, y0), (x1, y0), (x1, y1), (x0, y1)];
    if !ccw {
        v.reverse();
    }
    v
}

/// A rectilinear polygon: steps to the right and up, then back along the top and the left.
fn staircase(steps: usize, k: i64, s: &mut u64) -> Shape {
    let mut v = vec![(-k, -k)];
    let (mut x, mut y) = (-k, -k);
    for _ in 0..steps {
        x += between(0, (2 * k / steps as i64).max(1), s);
        v.push((x, y));
        y += between(0, (2 * k / steps as i64).max(1), s);
        v.push((x, y));
    }
    v.push((-k, y));
    if chance(1, 2, s) {
        v.reverse();
    }
    v
}

/// Points whose y is one of a few levels: many horizontal edges, some back over themselves.
fn flat_poly(n: usize, k: i64, levels: i64, s: &mut u64) -> Shape {
    (0..n).map(|_| (r(k, s), between(-levels, levels, s) * (k / levels.max(1)).max(1))).collect()
}

/// A zigzag between y = -k and y = k of `teeth` teeth, leaning by `slant`: every vertex is on
/// one of two scanlines, so all its crossings with another comb are in one scanbeam.
fn comb(teeth: usize, k: i64, pitch: i64, slant: i64, x0: i64) -> Shape {
    let mut v = Vec::new();
    for i in 0..teeth as i64 {
        v.push((x0 + i * pitch, -k));
        v.push((x0 + i * pitch + slant, k));
    }
    v
}

/// `depth` shapes one inside the other, alternating or not in orientation.
fn nest(depth: usize, k: i64, s: &mut u64) -> Vec<Shape> {
    let mut out = Vec::new();
    let mut rad = k;
    let alternate = chance(3, 4, s);
    let mut ccw = chance(1, 2, s);
    for _ in 0..depth {
        if rad < 2 {
            break;
        }
        if chance(1, 2, s) {
            out.push(rect(-rad, -rad, rad, rad, ccw));
        } else {
            // A diamond, or a polygon with more corners, inside the square of `rad`.
            let n = between(4, 8, s) as usize;
            let mut sh: Shape = (0..n)
                .map(|i| {
                    let a = i as f64 / n as f64 * std::f64::consts::TAU;
                    ((rad as f64 * a.cos()).round() as i64, (rad as f64 * a.sin()).round() as i64)
                })
                .collect();
            if !ccw {
                sh.reverse();
            }
            out.push(sh);
        }
        if alternate {
            ccw = !ccw;
        }
        // Sometimes the next one touches this one.
        rad = if chance(1, 6, s) { rad } else { rad * between(40, 80, s) / 100 };
        if chance(1, 8, s) {
            rad -= 1;
        }
    }
    out
}

fn shift(sh: &mut Shape, dx: i64, dy: i64) {
    for p in sh.iter_mut() {
        p.0 += dx;
        p.1 += dy;
    }
}

/// Repeats of a vertex, vertices one unit off, points on an edge, spikes that go out and back.
fn decorate(sh: &Shape, amount: u64, s: &mut u64) -> Shape {
    // In 128 bits: with the largest shapes the sums below leave 64.
    let lim = |v: i128| v.clamp(-(2 * MAX_COORD as i128), 2 * MAX_COORD as i128) as i64;
    let mut out = Shape::new();
    let n = sh.len();
    for i in 0..n {
        let p = sh[i];
        let q = sh[(i + 1) % n];
        let (px, py, qx, qy) = (p.0 as i128, p.1 as i128, q.0 as i128, q.1 as i128);
        out.push(p);
        if amount == 0 {
            continue;
        }
        match below(40 / amount.min(8), s) {
            0 => out.push(p),
            1 => {
                out.push(p);
                out.push(p);
            }
            2 => out.push((lim(px + r(1, s) as i128), lim(py + r(1, s) as i128))),
            3 => out.push((lim((px + qx) / 2), lim((py + qy) / 2))),
            4 => {
                // a point on the edge where the edge allows one, else next to it
                let t = between(1, 3, s) as i128;
                out.push((lim(px + (qx - px) * t / 4), lim(py + (qy - py) * t / 4)));
            }
            5 => {
                // a spike: out and back
                let far = (lim(px + r(5, s) as i128), lim(py + r(5, s) as i128));
                out.push(far);
                out.push(p);
            }
            6 => {
                // a spike along the edge, beyond its end
                out.push((lim(qx + (qx - px)), lim(qy + (qy - py))));
            }
            _ => {}
        }
    }
    out
}

/// Where a shape goes: `x * f + jitter + offset`, kept inside the coordinates the library
/// takes.
#[derive(Clone, Copy, Debug)]
struct Space {
    f: i64,
    jit: i64,
    ox: i64,
    oy: i64,
}

impl Space {
    fn put(&self, sh: &Shape, s: &mut u64) -> Path {
        let c = |v: i64, o: i64, s: &mut u64| -> i64 {
            let w = v as i128 * self.f as i128 + r(self.jit, s) as i128 + o as i128;
            w.clamp(-(MAX_COORD as i128), MAX_COORD as i128) as i64
        };
        sh.iter().map(|&(x, y)| [c(x, self.ox, s), c(y, self.oy, s), 0]).collect()
    }
}

// -------------------------------------------------------------------------------------------
// Cases
// -------------------------------------------------------------------------------------------

#[derive(Clone, Debug)]
pub struct Case {
    pub name: String,
    pub subj: Paths,
    pub open: Paths,
    pub clip: Paths,
    /// 0 NoClip, 1 Intersection, 2 Union, 3 Difference, 4 Xor (clipper.engine.h:33).
    pub clip_type: i32,
    /// 0 EvenOdd, 1 NonZero, 2 Positive, 3 Negative (clipper.core.h:111).
    pub fill_rule: i32,
    pub preserve_collinear: bool,
    pub reverse_solution: bool,
    pub use_tree: bool,
    /// See `cz2_ref_*_execute` (clipper2_ref_shim.cpp): bit 0 clips first, bit 1 an `Execute`
    /// before this one on the same clipper, bit 2 the paths go in one by one.
    pub add_order: i32,
    /// 0 none, 1 the engine's (Clipper2ZIntersectionVisitor), 2 one that hashes all it is given.
    pub z_callback: i32,
    pub default_z: i64,
}

/// The number of (clip type, fill rule, PreserveCollinear, ReverseSolution, paths or tree)
/// combinations; case `i` of a category takes combination `i % COMBOS`.
pub const COMBOS: u64 = 4 * 4 * 2 * 2 * 2;

fn options(name: &str, index: u64, s: &mut u64) -> Case {
    let c = index % COMBOS;
    Case {
        name: format!("{name}#{index}"),
        subj: Paths::new(),
        open: Paths::new(),
        clip: Paths::new(),
        // One case in 257 is NoClip.
        clip_type: if index % 257 == 256 { 0 } else { 1 + (c % 4) as i32 },
        fill_rule: ((c / 4) % 4) as i32,
        preserve_collinear: (c / 16) % 2 == 1,
        reverse_solution: (c / 32) % 2 == 1,
        use_tree: (c / 64) % 2 == 1,
        add_order: if chance(1, 2, s) { 0 } else { below(8, s) as i32 },
        z_callback: 0,
        default_z: 0,
    }
}

/// The scale and jitter that take a shape of size `k` to coordinates of size `mag`.
fn space_for(k: i64, mag: i64, s: &mut u64) -> Space {
    let f = (mag / k).max(1);
    let jit = if f > 1 {
        match below(4, s) {
            0 => 0,
            1 => 1,
            2 => f / 3,
            _ => f / 100,
        }
    } else {
        0
    };
    Space { f, jit, ox: 0, oy: 0 }
}

fn any_shape(k: i64, s: &mut u64) -> Shape {
    let mut sh = match below(8, s) {
        0 | 1 => rand_poly(between(3, 10, s) as usize, k, s),
        2 | 3 => ring(between(3, 12, s) as usize, k, s),
        4 => {
            let n = between(5, 11, s) as usize;
            star(n, between(2, (n as i64 - 1) / 2 + 1, s) as usize, k)
        }
        5 => {
            let (a, b) = (r(k, s), r(k, s));
            let (c, d) = (r(k, s), r(k, s));
            rect(a.min(c), b.min(d), a.max(c), b.max(d), chance(1, 2, s))
        }
        6 => staircase(between(1, 5, s) as usize, k, s),
        _ => flat_poly(between(4, 10, s) as usize, k, between(1, 3, s), s),
    };
    shift(&mut sh, r(k / 2, s), r(k / 2, s));
    sh
}

/// Random polygons at the magnitudes 1e2 .. 1e9.
pub fn gen_random(index: u64, s: &mut u64) -> Case {
    let mut case = options("random_polygons", index, s);
    let mag = 10i64.pow(2 + (index % 8) as u32);
    let k = [8i64, 50, 1000, mag][below(4, s) as usize].min(mag);
    let sp = space_for(k, mag, s);
    for _ in 0..between(1, 3, s) {
        let sh = any_shape(k, s);
        let sh = decorate(&sh, below(3, s), s);
        case.subj.push(sp.put(&sh, s));
    }
    for _ in 0..between(0, 3, s) {
        let sh = any_shape(k, s);
        let sh = decorate(&sh, below(3, s), s);
        case.clip.push(sp.put(&sh, s));
    }
    case
}

/// Coordinates near the largest the library accepts: differences up to 2^62, so the
/// products of `ProductsAreEqual` need 128 bits and the doubles lose the low bits.
pub fn gen_huge(index: u64, s: &mut u64) -> Case {
    let mut case = options("huge_coordinates", index, s);
    let variant = index / COMBOS % 5;
    let (k, sp) = match variant {
        // the whole range, with noise in the low bits
        0 => {
            let k = 1i64 << between(3, 20, s);
            let f = MAX_COORD / (2 * k);
            (k, Space { f, jit: [0, 1, f / 2, 1000][below(4, s) as usize], ox: 0, oy: 0 })
        }
        // every coordinate anywhere in the range
        1 => (MAX_COORD / 2, Space { f: 1, jit: 0, ox: 0, oy: 0 }),
        // small shapes at the edge of the range
        2 => {
            let k = [10i64, 1000, 1_000_000][below(3, s) as usize];
            let far = MAX_COORD - 4 * k;
            let (sx, sy) = (between(-1, 1, s), between(-1, 1, s));
            (k, Space { f: 1, jit: 0, ox: sx * far, oy: if sx == 0 && sy == 0 { far } else { sy * far } })
        }
        // a tiny grid blown up: collinear and coincident points at huge coordinates
        3 => {
            let k = between(2, 6, s);
            (k, Space { f: MAX_COORD / (2 * k), jit: [0, 0, 1, 3][below(4, s) as usize], ox: 0, oy: 0 })
        }
        // the whole range in x, a narrow band in y, and the other way round
        _ => (1 << 20, Space { f: MAX_COORD / (1 << 21), jit: 5, ox: 0, oy: 0 }),
    };
    let squash = variant == 4;
    let squash_x = chance(1, 2, s);
    let put = |sh: Shape, s: &mut u64| -> Path {
        let mut p = sp.put(&sh, s);
        if squash {
            for q in p.iter_mut() {
                if squash_x {
                    q[0] /= 1 << 30;
                } else {
                    q[1] /= 1 << 30;
                }
            }
        }
        p
    };
    for _ in 0..between(1, 3, s) {
        let sh = any_shape(k, s);
        let sh = decorate(&sh, below(3, s), s);
        case.subj.push(put(sh, s));
    }
    for _ in 0..between(0, 2, s) {
        let sh = any_shape(k, s);
        let sh = decorate(&sh, below(3, s), s);
        case.clip.push(put(sh, s));
    }
    if chance(1, 5, s) {
        let sh = rand_poly(between(2, 6, s) as usize, k, s);
        case.open.push(put(sh, s));
    }
    case
}

/// A tiny grid: many coincident, collinear and near-duplicate vertices.
pub fn gen_coincident(index: u64, s: &mut u64) -> Case {
    let mut case = options("coincident_and_collinear", index, s);
    let k = between(1, 6, s);
    let sp = Space { f: [1i64, 1, 2, 3, 1000, 1_000_000][below(6, s) as usize], jit: [0i64, 0, 0, 1][below(4, s) as usize], ox: 0, oy: 0 };
    for _ in 0..between(1, 4, s) {
        let sh = rand_poly(between(3, 12, s) as usize, k, s);
        let sh = decorate(&sh, between(1, 8, s) as u64, s);
        case.subj.push(sp.put(&sh, s));
    }
    for _ in 0..between(0, 3, s) {
        let sh = rand_poly(between(3, 12, s) as usize, k, s);
        let sh = decorate(&sh, between(0, 8, s) as u64, s);
        case.clip.push(sp.put(&sh, s));
    }
    if chance(1, 6, s) {
        let sh = rand_poly(between(2, 7, s) as usize, k, s);
        case.open.push(sp.put(&decorate(&sh, 3, s), s));
    }
    case
}

/// Rectangles, staircases and polygons on a few levels: horizontal edges that touch,
/// overlap and continue each other.
pub fn gen_horizontal(index: u64, s: &mut u64) -> Case {
    let mut case = options("horizontal_edges", index, s);
    let k = [4i64, 6, 10, 20, 100][below(5, s) as usize];
    let sp = Space { f: [1i64, 1, 10, 100_000][below(4, s) as usize], jit: 0, ox: 0, oy: 0 };
    let one = |s: &mut u64| -> Shape {
        let mut sh = match below(6, s) {
            0 | 1 | 2 => {
                let (a, b) = (r(k, s), r(k, s));
                let (c, d) = (a + between(1, k, s), b + between(1, k, s));
                rect(a, b, c, d, chance(1, 2, s))
            }
            3 => staircase(between(1, 6, s) as usize, k, s),
            4 => flat_poly(between(4, 12, s) as usize, k, between(1, 3, s), s),
            _ => {
                // a rectangle with points along its horizontal edges
                let (a, b) = (r(k, s), r(k, s));
                let (c, d) = (a + between(2, 2 * k, s), b + between(1, k, s));
                let m1 = between(a, c, s);
                let m2 = between(a, c, s);
                vec![(a, b), (m1, b), (c, b), (c, d), (m2, d), (a, d)]
            }
        };
        if chance(1, 3, s) {
            sh = decorate(&sh, between(1, 4, s) as u64, s);
        }
        sh
    };
    for _ in 0..between(1, 4, s) {
        let sh = one(s);
        case.subj.push(sp.put(&sh, s));
    }
    for _ in 0..between(0, 3, s) {
        let sh = one(s);
        case.clip.push(sp.put(&sh, s));
    }
    if chance(1, 8, s) {
        let y = r(k, s);
        let sh = vec![(r(2 * k, s), y), (r(2 * k, s), y), (r(2 * k, s), if chance(1, 2, s) { y } else { r(k, s) })];
        case.open.push(sp.put(&sh, s));
    }
    case
}

/// Polygons that share edges and vertices, copies of each other, copies moved by one unit.
pub fn gen_touching(index: u64, s: &mut u64) -> Case {
    let mut case = options("touching_and_overlapping", index, s);
    let mag = 10i64.pow(1 + (index / COMBOS % 9) as u32);
    let k = [6i64, 30, 1000][below(3, s) as usize].min(mag);
    let sp = space_for(k, mag, s);
    let sp = Space { jit: if chance(3, 4, s) { 0 } else { sp.jit.min(1) }, ..sp };
    let base = if chance(1, 2, s) { ring(between(3, 9, s) as usize, k, s) } else { any_shape(k, s) };
    let n = base.len();
    case.subj.push(sp.put(&base, s));
    match below(7, s) {
        // the same polygon as clip, perhaps from another start vertex, perhaps reversed
        0 => {
            let rot = below(n as u64, s) as usize;
            let mut c: Shape = (0..n).map(|i| base[(i + rot) % n]).collect();
            if chance(1, 2, s) {
                c.reverse();
            }
            case.clip.push(sp.put(&c, s));
        }
        // a copy moved by a little
        1 => {
            let mut c = base.clone();
            shift(&mut c, r(1, s), r(1, s));
            case.clip.push(sp.put(&c, s));
        }
        // a polygon on the other side of one edge
        2 | 3 => {
            let i = below(n as u64, s) as usize;
            let (p, q) = (base[i], base[(i + 1) % n]);
            let third = (r(k, s), r(k, s));
            let c = vec![q, p, third];
            if chance(1, 2, s) {
                case.clip.push(sp.put(&c, s));
            } else {
                case.subj.push(sp.put(&c, s));
            }
            if chance(1, 2, s) {
                // and one that shares only part of that edge
                let mid = ((p.0 + q.0) / 2, (p.1 + q.1) / 2);
                case.clip.push(sp.put(&vec![mid, q, (r(k, s), r(k, s))], s));
            }
        }
        // polygons that meet in one vertex
        4 => {
            let p = base[below(n as u64, s) as usize];
            for _ in 0..between(1, 3, s) {
                let c = vec![p, (r(k, s), r(k, s)), (r(k, s), r(k, s))];
                if chance(1, 2, s) {
                    case.clip.push(sp.put(&c, s));
                } else {
                    case.subj.push(sp.put(&c, s));
                }
            }
        }
        // rectangles side by side, sharing all or part of a side
        5 => {
            case.subj.clear();
            let (w, h) = (between(1, k, s), between(1, k, s));
            case.subj.push(sp.put(&rect(0, 0, w, h, chance(1, 2, s)), s));
            let dy = between(-h, h, s);
            let other = rect(w, dy, w + between(1, k, s), dy + between(1, 2 * h, s), chance(1, 2, s));
            if chance(1, 2, s) {
                case.clip.push(sp.put(&other, s));
            } else {
                case.subj.push(sp.put(&other, s));
            }
            let dx = between(-w, w, s);
            let above = rect(dx, h, dx + between(1, 2 * w, s), h + between(1, k, s), chance(1, 2, s));
            if chance(1, 2, s) {
                case.clip.push(sp.put(&above, s));
            } else if chance(1, 2, s) {
                case.subj.push(sp.put(&above, s));
            }
        }
        // the polygon twice in the subject, and a clip through it
        _ => {
            case.subj.push(sp.put(&base, s));
            case.clip.push(sp.put(&any_shape(k, s), s));
        }
    }
    case
}

/// Stars, polygons of many random vertices, paths that run over themselves.
pub fn gen_self_intersecting(index: u64, s: &mut u64) -> Case {
    let mut case = options("self_intersecting", index, s);
    let mag = 10i64.pow(2 + (index / COMBOS % 8) as u32);
    let k = [10i64, 40, 1000][below(3, s) as usize].min(mag);
    let sp = space_for(k, mag, s);
    let one = |s: &mut u64| -> Shape {
        match below(5, s) {
            0 => {
                let n = between(5, 17, s) as usize;
                star(n, between(2, n as i64 - 2, s) as usize, k)
            }
            1 | 2 => rand_poly(between(5, 40, s) as usize, k, s),
            3 => {
                // a path that goes round twice
                let mut sh = ring(between(3, 8, s) as usize, k, s);
                let again = sh.clone();
                sh.extend(again.iter().map(|&(x, y)| (x + r(2, s), y + r(2, s))));
                sh
            }
            _ => decorate(&rand_poly(between(4, 12, s) as usize, k, s), 6, s),
        }
    };
    for _ in 0..between(1, 2, s) {
        let sh = one(s);
        case.subj.push(sp.put(&sh, s));
    }
    for _ in 0..between(0, 2, s) {
        let sh = one(s);
        case.clip.push(sp.put(&sh, s));
    }
    case
}

/// Open subject paths against closed clips, with and without closed subjects beside them.
pub fn gen_open(index: u64, s: &mut u64) -> Case {
    let mut case = options("open_subjects", index, s);
    let mag = 10i64.pow(1 + (index / COMBOS % 9) as u32);
    let k = [5i64, 20, 1000][below(3, s) as usize].min(mag);
    let sp = space_for(k, mag, s);
    let sp = Space { jit: if chance(1, 2, s) { 0 } else { sp.jit }, ..sp };
    let mut clips: Vec<Shape> = Vec::new();
    for _ in 0..between(1, 3, s) {
        clips.push(any_shape(k, s));
    }
    for _ in 0..between(1, 4, s) {
        let c = &clips[below(clips.len() as u64, s) as usize];
        let n = c.len();
        let sh: Shape = match below(9, s) {
            0 | 1 | 2 => rand_poly(between(2, 8, s) as usize, k + k / 2, s),
            // horizontal
            3 => {
                let y = r(k, s);
                (0..between(2, 4, s)).map(|_| (r(2 * k, s), y)).collect()
            }
            // vertical
            4 => {
                let x = r(k, s);
                (0..between(2, 4, s)).map(|_| (x, r(2 * k, s))).collect()
            }
            // along an edge of a clip, and beyond it
            5 => {
                let i = below(n as u64, s) as usize;
                let (p, q) = (c[i], c[(i + 1) % n]);
                vec![(p.0 - (q.0 - p.0), p.1 - (q.1 - p.1)), p, q, (q.0 + (q.0 - p.0), q.1 + (q.1 - p.1))]
            }
            // from a vertex of a clip
            6 => {
                let p = c[below(n as u64, s) as usize];
                vec![p, (r(2 * k, s), r(2 * k, s)), (r(2 * k, s), r(2 * k, s))]
            }
            // through two vertices of a clip
            7 => {
                let p = c[below(n as u64, s) as usize];
                let q = c[below(n as u64, s) as usize];
                vec![(r(2 * k, s), r(2 * k, s)), p, q, (r(2 * k, s), r(2 * k, s))]
            }
            // a closed loop given as an open path, a single point, a path that goes back
            _ => match below(3, s) {
                0 => {
                    let mut sh = ring(between(3, 6, s) as usize, k, s);
                    sh.push(sh[0]);
                    sh
                }
                1 => vec![(r(k, s), r(k, s))],
                _ => {
                    let (p, q) = ((r(k, s), r(k, s)), (r(k, s), r(k, s)));
                    vec![p, q, p, q]
                }
            },
        };
        let sh = if chance(1, 4, s) { decorate(&sh, 3, s) } else { sh };
        case.open.push(sp.put(&sh, s));
    }
    // The clips go in after the open paths were made from them, so that both see the same
    // jitter-free coordinates where there is no jitter.
    for c in &clips {
        case.clip.push(sp.put(c, s));
    }
    if chance(1, 3, s) {
        case.subj.push(sp.put(&any_shape(k, s), s));
    }
    if chance(1, 12, s) {
        case.clip.clear();
    }
    case
}

/// Holes inside outers inside holes, and several such side by side.
pub fn gen_nested(index: u64, s: &mut u64) -> Case {
    let mut case = options("nested", index, s);
    // three of four are trees
    if index / COMBOS % 4 != 0 {
        case.use_tree = true;
    }
    let mag = 10i64.pow(2 + (index / COMBOS % 8) as u32);
    let k = [20i64, 100, 1000][below(3, s) as usize].min(mag);
    let sp = space_for(k, mag, s);
    let sp = Space { jit: if chance(2, 3, s) { 0 } else { sp.jit.min(2) }, ..sp };
    for _ in 0..between(1, 3, s) {
        let (dx, dy) = (r(2 * k, s), r(2 * k, s));
        for mut sh in nest(between(1, 6, s) as usize, k, s) {
            shift(&mut sh, dx, dy);
            let sh = if chance(1, 6, s) { decorate(&sh, 2, s) } else { sh };
            if chance(1, 5, s) {
                case.clip.push(sp.put(&sh, s));
            } else {
                case.subj.push(sp.put(&sh, s));
            }
        }
    }
    if chance(1, 2, s) {
        case.clip.push(sp.put(&any_shape(2 * k, s), s));
    }
    if chance(1, 3, s) {
        // a self-intersecting path through the nest: splits whose owners have to be found
        let n = between(5, 9, s) as usize;
        case.subj.push(sp.put(&star(n, 2, 2 * k), s));
    }
    case
}

/// Two combs that lean against each other: tens to hundreds of intersections in one
/// scanbeam, for the sort of the intersection list.
pub fn gen_dense(index: u64, s: &mut u64) -> Case {
    let mut case = options("dense_intersections", index, s);
    let k = [50i64, 1000, 100_000][below(3, s) as usize];
    let sp = Space { f: [1i64, 1, 7, 1000][below(4, s) as usize], jit: 0, ox: 0, oy: 0 };
    let t1 = between(2, 14, s) as usize;
    let t2 = between(2, 14, s) as usize;
    let pitch = between(1, k / 8, s).max(1);
    let a = comb(t1, k, pitch, between(k / 4, 2 * k, s), -k);
    let b = comb(t2, k, between(1, k / 8, s).max(1), -between(k / 4, 2 * k, s), between(0, k, s));
    case.subj.push(sp.put(&a, s));
    if chance(1, 2, s) {
        case.clip.push(sp.put(&b, s));
    } else {
        case.subj.push(sp.put(&b, s));
    }
    if chance(1, 3, s) {
        // the same comb again, one unit off: intersections at (nearly) the same points
        let mut c = a.clone();
        shift(&mut c, between(-1, 1, s), 0);
        case.clip.push(sp.put(&c, s));
    }
    if chance(1, 4, s) {
        case.open.push(sp.put(&comb(between(2, 6, s) as usize, k, pitch, -k, 0), s));
    }
    case
}

/// Long edges that are nearly parallel, nearly horizontal or nearly vertical: joins of
/// edges, intersection points that fall outside their scanbeam.
pub fn gen_nearly_parallel(index: u64, s: &mut u64) -> Case {
    let mut case = options("nearly_parallel", index, s);
    let len = 10i64.pow(3 + (index / COMBOS % 7) as u32);
    let thin = [1i64, 2, 3, 10, 100][below(5, s) as usize];
    let lying = chance(2, 3, s);
    let one = |s: &mut u64| -> Path {
        let n = between(3, 8, s) as usize;
        (0..n)
            .map(|_| {
                let long = r(len, s);
                let short = r(thin, s);
                if lying {
                    [long, short, 0]
                } else {
                    [short, long, 0]
                }
            })
            .collect()
    };
    for _ in 0..between(1, 3, s) {
        let p = one(s);
        case.subj.push(p);
    }
    for _ in 0..between(0, 2, s) {
        let p = one(s);
        case.clip.push(p);
    }
    if chance(1, 3, s) {
        // two edges along one line, with other edges ending on it
        let y = r(thin, s);
        case.subj.push(vec![[-len, y, 0], [len, y, 0], [r(len, s), y + between(1, thin + 1, s), 0]]);
        case.clip.push(vec![[-len / 2, y, 0], [len / 2, y, 0], [r(len, s), y - between(1, thin + 1, s), 0]]);
    }
    case
}

// -------------------------------------------------------------------------------------------
// Z
// -------------------------------------------------------------------------------------------

/// Gives the input its z, the clipper a callback and a `DefaultZ`.
pub fn assign_z(case: &mut Case, index: u64, s: &mut u64) {
    let mode = index / COMBOS % 6;
    case.z_callback = (index / (6 * COMBOS) % 3) as i32;
    // Every pair of (z mode, callback) is met in each run of 18 * COMBOS cases; the rest is
    // drawn.
    if index >= 18 * COMBOS && chance(1, 3, s) {
        case.z_callback = below(3, s) as i32;
    }
    case.default_z = [0i64, -1, 12345, i64::MIN / 3][below(4, s) as usize];
    let mut path_no = 0i64;
    for (kind, paths) in [(0i64, &mut case.subj), (1, &mut case.open), (2, &mut case.clip)] {
        for path in paths.iter_mut() {
            path_no += 1;
            let per_path = r(1_000_000, s);
            for (vi, p) in path.iter_mut().enumerate() {
                p[2] = match mode {
                    // all zero
                    0 => 0,
                    // every vertex its own
                    1 => (path_no << 32) | vi as i64,
                    // one per path, as the engine gives a contour its index
                    2 => path_no,
                    // one per path, drawn
                    3 => per_path,
                    // a few small values, negative too: runs of equal z along a path
                    4 => r(2, s),
                    // anything
                    _ => xorshift(s) as i64,
                };
            }
            let _ = kind;
        }
    }
}

// -------------------------------------------------------------------------------------------
// The result of one Execute, and the reference
// -------------------------------------------------------------------------------------------

#[derive(Clone, Debug, PartialEq, Eq, Default)]
pub struct Flat {
    /// x, y, z of every point of every path: the closed paths (the nodes of the tree in
    /// depth-first pre-order), then the open paths.
    pub coords: Vec<i64>,
    pub lens: Vec<i32>,
    /// With a tree: the index of each node's parent, -1 for a child of the root.
    pub parents: Vec<i32>,
    /// With a tree: `IsHole()` of each node.
    pub holes: Vec<i32>,
    pub num_closed: i32,
    pub num_open: i32,
    pub succeeded: bool,
    pub cb_calls: u64,
    pub cb_hash: u64,
}

impl Flat {
    pub fn add(&mut self, path: &[Pt]) {
        self.lens.push(path.len() as i32);
        for p in path {
            self.coords.extend_from_slice(p);
        }
    }
}

fn flatten(paths: &Paths) -> (Vec<i64>, Vec<i32>) {
    let mut xyz = Vec::new();
    let mut lens = Vec::new();
    for p in paths {
        lens.push(p.len() as i32);
        for q in p {
            xyz.extend_from_slice(q);
        }
    }
    (xyz, lens)
}

/// `Clipper64::Execute` of the C++ reference, with `USINGZ` (`z`) or without.
pub fn cpp_execute(case: &Case, z: bool) -> Flat {
    let (sx, sl) = flatten(&case.subj);
    let (ox, ol) = flatten(&case.open);
    let (cx, cl) = flatten(&case.clip);
    let f = if z { sys::cz2_ref_z_execute } else { sys::cz2_ref_xy_execute };
    let res = unsafe {
        f(
            sx.as_ptr(),
            sl.as_ptr(),
            sl.len() as i32,
            ox.as_ptr(),
            ol.as_ptr(),
            ol.len() as i32,
            cx.as_ptr(),
            cl.as_ptr(),
            cl.len() as i32,
            case.clip_type,
            case.fill_rule,
            case.preserve_collinear as i32,
            case.reverse_solution as i32,
            case.use_tree as i32,
            case.add_order,
            case.z_callback,
            case.default_z,
        )
    };
    let slice = |p: *mut i64, n: usize| if p.is_null() || n == 0 { Vec::new() } else { unsafe { std::slice::from_raw_parts(p, n) }.to_vec() };
    let slice32 = |p: *mut i32, n: usize| if p.is_null() || n == 0 { Vec::new() } else { unsafe { std::slice::from_raw_parts(p, n) }.to_vec() };
    let paths = (res.num_closed + res.num_open) as usize;
    let nodes = if case.use_tree { res.num_closed as usize } else { 0 };
    let out = Flat {
        coords: slice(res.coords, 3 * res.total_points as usize),
        lens: slice32(res.path_lens, paths),
        parents: slice32(res.tree_parent, nodes),
        holes: slice32(res.tree_is_hole, nodes),
        num_closed: res.num_closed,
        num_open: res.num_open,
        succeeded: res.succeeded != 0,
        cb_calls: res.cb_calls,
        cb_hash: res.cb_hash,
    };
    unsafe {
        if z {
            sys::cz2_ref_z_free(res)
        } else {
            sys::cz2_ref_xy_free(res)
        }
    };
    out
}

/// `h = (h ^ v) * 0x100000001b3`, the `mix` of clipper2_ref_shim.cpp.
pub fn mix(h: &mut u64, v: i64) {
    *h = (*h ^ v as u64).wrapping_mul(0x0000_0100_0000_01b3);
}

pub const HASH_START: u64 = 0xcbf2_9ce4_8422_2325;

/// The first place two results differ, `None` where they are equal.
pub fn first_difference(rust: &Flat, cpp: &Flat) -> Option<String> {
    if rust.succeeded != cpp.succeeded {
        return Some(format!("Execute returned {} in Rust, {} in C++", rust.succeeded, cpp.succeeded));
    }
    if rust.num_closed != cpp.num_closed {
        return Some(format!("{} closed paths in Rust, {} in C++", rust.num_closed, cpp.num_closed));
    }
    if rust.num_open != cpp.num_open {
        return Some(format!("{} open paths in Rust, {} in C++", rust.num_open, cpp.num_open));
    }
    let mut at = 0usize;
    for (i, (lr, lc)) in rust.lens.iter().zip(cpp.lens.iter()).enumerate() {
        let what = if (i as i32) < rust.num_closed { format!("closed path {i}") } else { format!("open path {}", i as i32 - rust.num_closed) };
        if lr != lc {
            return Some(format!("{what}: {lr} points in Rust, {lc} in C++"));
        }
        for v in 0..*lr as usize {
            let a = &rust.coords[at + 3 * v..at + 3 * v + 3];
            let b = &cpp.coords[at + 3 * v..at + 3 * v + 3];
            if a != b {
                return Some(format!("{what}, point {v}: ({}, {}, z {}) in Rust, ({}, {}, z {}) in C++", a[0], a[1], a[2], b[0], b[1], b[2]));
            }
        }
        at += 3 * *lr as usize;
    }
    if rust.lens.len() != cpp.lens.len() {
        return Some(format!("{} paths in Rust, {} in C++", rust.lens.len(), cpp.lens.len()));
    }
    for (i, (a, b)) in rust.parents.iter().zip(cpp.parents.iter()).enumerate() {
        if a != b {
            return Some(format!("tree node {i}: parent {a} in Rust, {b} in C++"));
        }
    }
    if rust.parents.len() != cpp.parents.len() {
        return Some(format!("{} tree nodes in Rust, {} in C++", rust.parents.len(), cpp.parents.len()));
    }
    for (i, (a, b)) in rust.holes.iter().zip(cpp.holes.iter()).enumerate() {
        if a != b {
            return Some(format!("tree node {i}: IsHole {a} in Rust, {b} in C++"));
        }
    }
    if rust.cb_calls != cpp.cb_calls {
        return Some(format!("the Z callback ran {} times in Rust, {} in C++", rust.cb_calls, cpp.cb_calls));
    }
    if rust.cb_hash != cpp.cb_hash {
        return Some(format!("the Z callback saw other arguments: hash {:#x} in Rust, {:#x} in C++", rust.cb_hash, cpp.cb_hash));
    }
    None
}

fn dump_paths(out: &mut String, name: &str, paths: &Paths) {
    let _ = writeln!(out, "  {name}: {} paths", paths.len());
    for p in paths {
        let _ = write!(out, "    [");
        for q in p {
            let _ = write!(out, "({},{},{}) ", q[0], q[1], q[2]);
        }
        let _ = writeln!(out, "]");
    }
}

pub fn dump_case(case: &Case) -> String {
    let mut out = String::new();
    let _ = writeln!(
        out,
        "case {}: clip_type {} fill_rule {} preserve_collinear {} reverse_solution {} use_tree {} add_order {} z_callback {} default_z {}",
        case.name,
        case.clip_type,
        case.fill_rule,
        case.preserve_collinear,
        case.reverse_solution,
        case.use_tree,
        case.add_order,
        case.z_callback,
        case.default_z
    );
    dump_paths(&mut out, "subject", &case.subj);
    dump_paths(&mut out, "open subject", &case.open);
    dump_paths(&mut out, "clip", &case.clip);
    out
}

fn dump_flat(out: &mut String, name: &str, f: &Flat) {
    let _ = writeln!(out, "  {name}: succeeded {} closed {} open {} cb_calls {} cb_hash {:#x}", f.succeeded, f.num_closed, f.num_open, f.cb_calls, f.cb_hash);
    let mut at = 0usize;
    for (i, l) in f.lens.iter().enumerate() {
        let _ = write!(out, "    {i}");
        if i < f.parents.len() {
            let _ = write!(out, " parent {} hole {}", f.parents[i], f.holes[i]);
        }
        let _ = write!(out, ": [");
        for v in 0..*l as usize {
            let _ = write!(out, "({},{},{}) ", f.coords[at + 3 * v], f.coords[at + 3 * v + 1], f.coords[at + 3 * v + 2]);
        }
        let _ = writeln!(out, "]");
        at += 3 * *l as usize;
    }
}

fn append(env: &str, text: &str) {
    if let Ok(path) = std::env::var(env) {
        if let Ok(mut f) = std::fs::OpenOptions::new().create(true).append(true).open(path) {
            let _ = f.write_all(text.as_bytes());
        }
    }
}

// -------------------------------------------------------------------------------------------
// The counters of the Rust side
// -------------------------------------------------------------------------------------------

pub const STAT_NAMES: [&str; 16] = [
    "horz_joins",
    "horz_join_splits",
    "split_ops",
    "split_new_outrecs",
    "edge_splits",
    "edge_joins",
    "intersect_corrections",
    "path1_inside_path2_midpoint",
    "split_owners_found",
    "failures",
    "collinear_removed",
    "splits_changed_in_loop",
    "intersect_list_max",
    "intersect_reorders",
    "horizontals",
    "products_128",
];
/// The one counter that is a maximum and not a sum.
pub const STAT_MAX: usize = 12;
pub type Stats = [u64; 16];

/// What one category came to.
#[derive(Debug, Default, Clone)]
pub struct Totals {
    pub cases: u64,
    pub mismatches: u64,
    pub stats: Stats,
    pub closed_paths: u64,
    pub open_paths: u64,
    pub tree_cases: u64,
    pub tree_nodes: u64,
    pub tree_holes: u64,
    pub tree_deepest: u64,
    pub nonempty: u64,
    pub output_points: u64,
    pub not_succeeded: u64,
    pub cb_calls: u64,
    pub combos: std::collections::BTreeSet<u64>,
    pub z_pairs: std::collections::BTreeSet<(i32, u64)>,
    pub nonzero_z_out: u64,
}

impl Totals {
    pub fn stat(&self, name: &str) -> u64 {
        self.stats[STAT_NAMES.iter().position(|n| *n == name).unwrap()]
    }
}

/// Runs `n` cases of one category through both sides and compares. Panics at the end with
/// the first differences if there were any.
pub fn run_category(
    name: &str,
    test_id: u64,
    n: u64,
    gen: fn(u64, &mut u64) -> Case,
    z: bool,
    rust: fn(&Case) -> (Flat, Stats),
) -> Totals {
    let mut t = Totals::default();
    let mut messages: Vec<String> = Vec::new();
    let trace = std::env::var("CLIPPER2_PARITY_TRACE").ok();
    // `CLIPPER2_PARITY_ONLY=<category>#<index>` runs that one case of that category.
    let only: Option<u64> = std::env::var("CLIPPER2_PARITY_ONLY").ok().and_then(|v| v.strip_prefix(&format!("{name}#")).and_then(|i| i.parse().ok()));
    // `CLIPPER2_PARITY_SCALE=<k>` runs k times as many cases of every category.
    let n = n * std::env::var("CLIPPER2_PARITY_SCALE").ok().and_then(|v| v.parse::<u64>().ok()).unwrap_or(1);
    for index in 0..n {
        if only.is_some_and(|o| o != index) {
            continue;
        }
        let mut s = case_seed(test_id, index);
        let mut case = gen(index, &mut s);
        if z {
            assign_z(&mut case, index, &mut s);
        }
        if let Some(path) = &trace {
            let _ = std::fs::write(path, dump_case(&case));
        }
        let cpp = cpp_execute(&case, z);
        let rust_out = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| rust(&case)));
        t.cases += 1;
        if case.clip_type != 0 {
            t.combos.insert(index % COMBOS);
        }
        t.z_pairs.insert((case.z_callback, index / COMBOS % 6));
        let difference = match &rust_out {
            Err(e) => {
                let what = e.downcast_ref::<String>().cloned().or_else(|| e.downcast_ref::<&str>().map(|m| m.to_string())).unwrap_or_default();
                Some(format!("the Rust side panicked: {what}"))
            }
            Ok((flat, stats)) => {
                for i in 0..stats.len() {
                    if i == STAT_MAX {
                        t.stats[i] = t.stats[i].max(stats[i]);
                    } else {
                        t.stats[i] += stats[i];
                    }
                }
                first_difference(flat, &cpp)
            }
        };
        t.closed_paths += cpp.num_closed as u64;
        t.open_paths += cpp.num_open as u64;
        t.output_points += (cpp.coords.len() / 3) as u64;
        t.cb_calls += cpp.cb_calls;
        if cpp.coords.chunks(3).any(|c| c[2] != 0) {
            t.nonzero_z_out += 1;
        }
        if !cpp.succeeded {
            t.not_succeeded += 1;
        }
        if !cpp.lens.is_empty() {
            t.nonempty += 1;
        }
        if case.use_tree {
            t.tree_cases += 1;
            t.tree_nodes += cpp.parents.len() as u64;
            t.tree_holes += cpp.holes.iter().filter(|h| **h != 0).count() as u64;
            let mut depth = vec![0u64; cpp.parents.len()];
            for (i, p) in cpp.parents.iter().enumerate() {
                depth[i] = if *p < 0 { 1 } else { depth[*p as usize] + 1 };
                t.tree_deepest = t.tree_deepest.max(depth[i]);
            }
        }
        if let Some(d) = difference {
            t.mismatches += 1;
            let mut text = format!("MISMATCH {}: {d}\n{}", case.name, dump_case(&case));
            if let Ok((flat, _)) = &rust_out {
                dump_flat(&mut text, "Rust", flat);
            }
            dump_flat(&mut text, "C++", &cpp);
            append("CLIPPER2_PARITY_LOG", &text);
            if messages.len() < 3 {
                messages.push(text);
            }
        }
    }
    let mut summary = format!(
        "{name} ({}): {} cases, {} mismatches; {} non-empty results, {} closed paths, {} open paths, {} points; {} tree cases with {} nodes, {} holes, deepest {}; Execute false {}; Z callback calls {}, results with a non-zero z {}; {} of {} option combinations\n ",
        if z { "USINGZ" } else { "no Z" },
        t.cases,
        t.mismatches,
        t.nonempty,
        t.closed_paths,
        t.open_paths,
        t.output_points,
        t.tree_cases,
        t.tree_nodes,
        t.tree_holes,
        t.tree_deepest,
        t.not_succeeded,
        t.cb_calls,
        t.nonzero_z_out,
        t.combos.len(),
        COMBOS
    );
    for (i, n) in STAT_NAMES.iter().enumerate() {
        let _ = write!(summary, " {n}={}", t.stats[i]);
    }
    summary.push('\n');
    eprint!("{summary}");
    append("CLIPPER2_PARITY_SUMMARY", &summary);
    assert!(t.mismatches == 0, "{name}: {} of {} cases differ from the C++. The first:\n{}", t.mismatches, t.cases, messages.join("\n"));
    if only.is_none() {
        assert_eq!(t.combos.len() as u64, COMBOS.min(n), "{name}: not every combination of the options was run");
    }
    t
}

// -------------------------------------------------------------------------------------------
// The helpers of clipper.core.h
// -------------------------------------------------------------------------------------------

/// What `cz2_ref_*_core` gives for four points a, b, c, d (see clipper2_ref_shim.cpp).
#[derive(Debug, Clone, PartialEq)]
pub struct CoreOut {
    /// ip of GetSegmentIntersectPt, GetClosestPointOnSegment(a, b, c), MidPoint(a, b): x, y, z each.
    pub ints: [i64; 9],
    /// CrossProduct(a, b, c), DotProduct(a, b, c), PerpendicDistFromLineSqrd(a, b, c), DistanceSqr(a, b), as bits.
    pub doubles: [u64; 4],
    /// GetSegmentIntersectPt, SegmentsIntersect exclusive and inclusive, IsCollinear(a, b, c),
    /// ProductsAreEqual(a.x, a.y, b.x, b.y), a == b.
    pub bools: [bool; 6],
}

pub fn cpp_core(p: &[i64; 12], z: bool) -> CoreOut {
    let mut ints = [0i64; 9];
    let mut doubles = [0f64; 4];
    let mut bools = [0i32; 6];
    unsafe {
        if z {
            sys::cz2_ref_z_core(p.as_ptr(), ints.as_mut_ptr(), doubles.as_mut_ptr(), bools.as_mut_ptr())
        } else {
            sys::cz2_ref_xy_core(p.as_ptr(), ints.as_mut_ptr(), doubles.as_mut_ptr(), bools.as_mut_ptr())
        }
    }
    CoreOut { ints, doubles: doubles.map(f64::to_bits), bools: bools.map(|b| b != 0) }
}

pub fn cpp_point_in_polygon(x: i64, y: i64, poly: &Path, z: bool) -> i32 {
    let xyz: Vec<i64> = poly.iter().flat_map(|p| p.iter().copied()).collect();
    unsafe {
        if z {
            sys::cz2_ref_z_point_in_polygon(x, y, xyz.as_ptr(), poly.len() as i32)
        } else {
            sys::cz2_ref_xy_point_in_polygon(x, y, xyz.as_ptr(), poly.len() as i32)
        }
    }
}

pub fn cpp_area(poly: &Path, z: bool) -> u64 {
    let xyz: Vec<i64> = poly.iter().flat_map(|p| p.iter().copied()).collect();
    unsafe {
        if z {
            sys::cz2_ref_z_area(xyz.as_ptr(), poly.len() as i32)
        } else {
            sys::cz2_ref_xy_area(xyz.as_ptr(), poly.len() as i32)
        }
    }
    .to_bits()
}

/// 1 where the reference is built from the engine's pin, and its `CLIPPER2_VERSION`.
pub fn cpp_source(z: bool) -> (i32, String) {
    unsafe {
        let (from_pin, v) = if z { (sys::cz2_ref_z_from_pin(), sys::cz2_ref_z_version()) } else { (sys::cz2_ref_xy_from_pin(), sys::cz2_ref_xy_version()) };
        (from_pin, std::ffi::CStr::from_ptr(v).to_string_lossy().into_owned())
    }
}

/// Four points for the helpers, of a kind chosen by `index`: the inputs sit on the places
/// where the helpers decide (collinear and one unit off, parallel, shared end points,
/// intersections at and next to whole numbers and halves, zero-length segments), at every
/// magnitude up to the largest coordinate.
pub fn core_input(index: u64, s: &mut u64) -> [i64; 12] {
    let mag = match index % 12 {
        0 => 3,
        1 => 10,
        2 => 100,
        3 => 10_000,
        4 => 1_000_000,
        5 => 1_000_000_000,
        6 => 1 << 40,
        7 => 1 << 52,
        8 => 1 << 53,
        9 => 1 << 57,
        _ => MAX_COORD / 2,
    };
    let p = |s: &mut u64| (r(mag, s), r(mag, s));
    let clamp = |v: i128| v.clamp(-(MAX_COORD as i128), MAX_COORD as i128) as i64;
    let along = |a: (i64, i64), b: (i64, i64), num: i64, den: i64| -> (i64, i64) {
        (
            clamp(a.0 as i128 + (b.0 as i128 - a.0 as i128) * num as i128 / den as i128),
            clamp(a.1 as i128 + (b.1 as i128 - a.1 as i128) * num as i128 / den as i128),
        )
    };
    let nudge = |q: (i64, i64), s: &mut u64| (clamp(q.0 as i128 + r(1, s) as i128), clamp(q.1 as i128 + r(1, s) as i128));
    let (mut a, mut b, mut c, mut d) = (p(s), p(s), p(s), p(s));
    match index / 12 % 14 {
        // anything
        0 => {}
        // c on the line a-b, inside, at the ends and beyond
        1 => c = along(a, b, between(-4, 8, s), 4),
        // and one unit next to it
        2 => {
            c = along(a, b, between(-4, 8, s), 4);
            c = nudge(c, s);
        }
        // c-d parallel to a-b
        3 => {
            let (dx, dy) = (b.0 as i128 - a.0 as i128, b.1 as i128 - a.1 as i128);
            let m = between(-2, 2, s) as i128;
            d = (clamp(c.0 as i128 + dx * m), clamp(c.1 as i128 + dy * m));
        }
        // c-d on the line of a-b
        4 => {
            c = along(a, b, between(-8, 16, s), 8);
            d = along(a, b, between(-8, 16, s), 8);
        }
        // a shared end point
        5 => match below(4, s) {
            0 => c = a,
            1 => c = b,
            2 => d = a,
            _ => d = b,
        },
        // c on a-b, d anywhere: the segments touch
        6 => {
            c = along(a, b, between(0, 8, s), 8);
            if chance(1, 2, s) {
                std::mem::swap(&mut c, &mut d);
            }
        }
        // a on c-d: t = 0 of the intersection, and b on c-d: t = 1
        7 => {
            if chance(1, 2, s) {
                a = along(c, d, between(0, 8, s), 8);
            } else {
                b = along(c, d, between(0, 8, s), 8);
            }
            if chance(1, 3, s) {
                a = nudge(a, s);
            }
        }
        // zero-length segments
        8 => match below(3, s) {
            0 => b = a,
            1 => d = c,
            _ => {
                b = a;
                d = c;
            }
        },
        // horizontal and vertical segments
        9 => {
            if chance(1, 2, s) {
                b.1 = a.1;
            } else {
                b.0 = a.0;
            }
            if chance(1, 2, s) {
                d.1 = c.1;
            } else {
                d.0 = c.0;
            }
        }
        // a diagonal cross: the intersection is at a whole number or at a half
        10 => {
            let (cx, cy) = p(s);
            let h = between(0, mag.min(1 << 20), s);
            let odd = between(0, 1, s);
            a = (clamp(cx as i128 - h as i128), clamp(cy as i128 - h as i128));
            b = (clamp(cx as i128 + h as i128 + odd as i128), clamp(cy as i128 + h as i128 + odd as i128));
            c = (clamp(cx as i128 - h as i128), clamp(cy as i128 + h as i128 + odd as i128));
            d = (clamp(cx as i128 + h as i128 + odd as i128), clamp(cy as i128 - h as i128));
        }
        // the closest point is at a half: c at the same distance from two lattice points
        11 => {
            b = (clamp(a.0 as i128 + 2 * between(-50, 50, s) as i128 + 1), a.1);
            c = (clamp((a.0 as i128 + b.0 as i128) / 2), clamp(a.1 as i128 + r(5, s) as i128));
            if chance(1, 2, s) {
                // and turned by 90 degrees
                a = (a.1, a.0);
                b = (b.1, b.0);
                c = (c.1, c.0);
            }
        }
        // everything within one unit
        12 => {
            b = nudge(a, s);
            c = nudge(a, s);
            d = nudge(b, s);
        }
        // the corners of the range
        _ => {
            let corner = |s: &mut u64| {
                let e = |s: &mut u64| match below(3, s) {
                    0 => MAX_COORD - between(0, 2, s),
                    1 => -MAX_COORD + between(0, 2, s),
                    _ => between(-2, 2, s),
                };
                (e(s), e(s))
            };
            a = corner(s);
            b = corner(s);
            c = corner(s);
            d = corner(s);
        }
    }
    [a.0, a.1, r(9, s), b.0, b.1, r(9, s), c.0, c.1, r(9, s), d.0, d.1, r(9, s)]
}

/// A polygon and points to ask `PointInPolygon` about: its vertices, points on its edges,
/// points at the height of its vertices, points anywhere.
pub fn point_in_polygon_input(index: u64, s: &mut u64) -> (Path, Vec<(i64, i64)>) {
    let mag = [4i64, 20, 1000, 1_000_000, 1_000_000_000, 1 << 50, MAX_COORD / 2][(index % 7) as usize];
    let k = [4i64, 20, 1000][below(3, s) as usize].min(mag);
    let sp = Space { f: mag / k, jit: 0, ox: 0, oy: 0 };
    let sh = match index / 7 % 6 {
        0 => rand_poly(between(1, 9, s) as usize, k, s),
        1 => ring(between(3, 9, s) as usize, k, s),
        2 => flat_poly(between(3, 10, s) as usize, k, between(1, 2, s), s),
        3 => staircase(between(1, 4, s) as usize, k, s),
        4 => decorate(&rand_poly(between(3, 8, s) as usize, k, s), 6, s),
        // every vertex at one height
        _ => (0..between(3, 6, s)).map(|_| (r(k, s), 0)).collect(),
    };
    let mut pts: Vec<(i64, i64)> = Vec::new();
    let n = sh.len();
    for i in 0..n {
        let (p, q) = (sh[i], sh[(i + 1) % n]);
        pts.push(p);
        pts.push(((p.0 + q.0) / 2, (p.1 + q.1) / 2));
        pts.push((r(2 * k, s), p.1));
        pts.push((p.0, r(2 * k, s)));
        pts.push((p.0 + r(1, s), p.1 + r(1, s)));
    }
    for _ in 0..6 {
        pts.push((r(2 * k, s), r(2 * k, s)));
    }
    let poly = sp.put(&sh, s);
    let pts = pts.iter().map(|&(x, y)| (x.saturating_mul(sp.f).clamp(-MAX_COORD, MAX_COORD), y.saturating_mul(sp.f).clamp(-MAX_COORD, MAX_COORD))).collect();
    (poly, pts)
}
