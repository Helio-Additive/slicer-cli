//! What `clipper2_rectclip_native_parity.rs`, `clipper2_d_native_parity.rs` and their `_z`
//! twins share: the deterministic generators of the cases, the call into the C++ reference
//! (`cz2_ref_xy_ext` / `cz2_ref_z_ext` of `clipper2-z-sys`, built from the Clipper2 the engine
//! pins; shim/clipper2_ref_ext_shim.cpp), the same operations on the Rust port
//! (`ext_rust_impl!`) and the tests (`rectclip_parity_tests!`, `d_parity_tests!`), which each
//! file expands for its build.
//!
//! An operation takes a stream of `i64` and one of `f64` and answers with two such streams
//! (the encodings are at the top of clipper2_ref_ext_shim.cpp). The two answers have to be
//! equal value for value: every path, in order, from the same start vertex, with the same
//! coordinates and z; a double by its bits. A case is made from its own seed, so a failure
//! names the case and prints its input.
//!
//! `CLIPPER2_PARITY_SCALE=<k>` runs k times as many cases; `CLIPPER2_PARITY_TRACE=1` names
//! every case, with its input, before each side runs it.
#![allow(dead_code)]

use clipper2_z_sys as sys;

pub type P64 = [i64; 3];
pub type PD = (f64, f64, i64);

/// The two streams of an operation's input or answer.
#[derive(Clone, Default, Debug)]
pub struct Enc {
    pub ints: Vec<i64>,
    pub dbls: Vec<f64>,
}

impl Enc {
    pub fn i(&mut self, v: i64) {
        self.ints.push(v);
    }
    pub fn d(&mut self, v: f64) {
        self.dbls.push(v);
    }
    pub fn p64(&mut self, p: &P64) {
        self.ints.extend_from_slice(p);
    }
    pub fn path64(&mut self, p: &[P64]) {
        self.i(p.len() as i64);
        for q in p {
            self.p64(q);
        }
    }
    pub fn paths64(&mut self, pp: &[Vec<P64>]) {
        self.i(pp.len() as i64);
        for p in pp {
            self.path64(p);
        }
    }
    pub fn pd(&mut self, p: &PD) {
        self.d(p.0);
        self.d(p.1);
        self.i(p.2);
    }
    pub fn pathd(&mut self, p: &[PD]) {
        self.i(p.len() as i64);
        for q in p {
            self.pd(q);
        }
    }
    pub fn pathsd(&mut self, pp: &[Vec<PD>]) {
        self.i(pp.len() as i64);
        for p in pp {
            self.pathd(p);
        }
    }
    pub fn rect64(&mut self, r: &[i64; 4]) {
        self.ints.extend_from_slice(r);
    }
    pub fn rectd(&mut self, r: &[f64; 4]) {
        self.dbls.extend_from_slice(r);
    }
}

/// Equal value for value, the doubles by their bits.
pub fn same(a: &Enc, b: &Enc) -> bool {
    a.ints == b.ints && a.dbls.len() == b.dbls.len() && a.dbls.iter().zip(&b.dbls).all(|(x, y)| x.to_bits() == y.to_bits())
}

/// The operation on the C++ reference of the build.
pub fn cpp_ext(op: i32, input: &Enc, z: bool) -> Enc {
    unsafe {
        let r = if z { sys::cz2_ref_z_ext(op, input.ints.as_ptr(), input.dbls.as_ptr()) } else { sys::cz2_ref_xy_ext(op, input.ints.as_ptr(), input.dbls.as_ptr()) };
        let out = Enc {
            ints: if r.n_ints > 0 { std::slice::from_raw_parts(r.ints, r.n_ints as usize).to_vec() } else { Vec::new() },
            dbls: if r.n_dbls > 0 { std::slice::from_raw_parts(r.dbls, r.n_dbls as usize).to_vec() } else { Vec::new() },
        };
        if z {
            sys::cz2_ref_z_ext_free(r)
        } else {
            sys::cz2_ref_xy_ext_free(r)
        }
        out
    }
}

/// 1 where the reference was built from `references/BambuStudio` (the pin), 0 from the
/// vendored copy of it.
pub fn cpp_from_pin(z: bool) -> i32 {
    unsafe {
        if z {
            sys::cz2_ref_z_from_pin()
        } else {
            sys::cz2_ref_xy_from_pin()
        }
    }
}

// ---------------------------------------------------------------------------------------
// The generator
// ---------------------------------------------------------------------------------------

pub fn xorshift(s: &mut u64) -> u64 {
    let mut x = *s;
    x ^= x << 13;
    x ^= x >> 7;
    x ^= x << 17;
    *s = x;
    x
}

pub fn case_seed(test: u64, index: u64) -> u64 {
    let mut s = 0x9E37_79B9_7F4A_7C15u64 ^ test.wrapping_mul(0xD6E8_FEB8_6659_FD93) ^ index.wrapping_mul(0xA24B_AED4_963E_E407);
    if s == 0 {
        s = 1;
    }
    for _ in 0..4 {
        xorshift(&mut s);
    }
    s
}

pub fn below(n: u64, s: &mut u64) -> u64 {
    xorshift(s) % n
}

pub fn between(lo: i64, hi: i64, s: &mut u64) -> i64 {
    lo + (xorshift(s) % ((hi - lo + 1) as u64)) as i64
}

pub fn chance(num: u64, den: u64, s: &mut u64) -> bool {
    below(den, s) < num
}

pub fn pick<T: Copy>(v: &[T], s: &mut u64) -> T {
    v[below(v.len() as u64, s) as usize]
}

pub fn scale_factor() -> u64 {
    std::env::var("CLIPPER2_PARITY_SCALE").ok().and_then(|v| v.parse().ok()).unwrap_or(1)
}

/// A rectangle: mostly small, some of one or two units, some empty or inverted, some thin,
/// some with coordinates up to 10^15.
pub fn gen_rect(s: &mut u64) -> [i64; 4] {
    match below(20, s) {
        0..=11 => {
            let (l, t) = (between(-40, 40, s), between(-40, 40, s));
            [l, t, l + between(1, 60, s), t + between(1, 60, s)]
        }
        12 => {
            let (l, t) = (between(-5, 5, s), between(-5, 5, s));
            [l, t, l + between(1, 2, s), t + between(1, 2, s)]
        }
        13 => {
            let (l, t) = (between(-10, 10, s), between(-10, 10, s));
            if chance(1, 2, s) {
                [l, t, l, t + between(0, 20, s)]
            } else {
                [l, t, l + between(0, 20, s), t]
            }
        }
        14 => {
            let (l, t) = (between(-10, 10, s), between(-10, 10, s));
            [l, t, l - between(1, 20, s), t + between(-20, 20, s)]
        }
        15..=17 => {
            let m = 10i64.pow(between(4, 15, s) as u32);
            let (l, t) = (between(-m, m, s), between(-m, m, s));
            [l, t, l + between(1, m, s), t + between(1, m, s)]
        }
        18 => {
            let (l, t) = (between(-20, 20, s), between(-20, 20, s));
            if chance(1, 2, s) {
                [l, t, l + 1, t + 100]
            } else {
                [l, t, l + 100, t + 1]
            }
        }
        _ => [0, 0, 100, 100],
    }
}

fn lattice(lo: i64, hi: i64, s: &mut u64) -> i64 {
    let w = (hi - lo).abs().max(1);
    let d = between(1, w, s);
    pick(&[lo - w, lo - d, lo - 2, lo - 1, lo, lo, lo + 1, lo + d.min(w - 1).max(0), (lo + hi) / 2, hi - 1, hi, hi, hi + 1, hi + 2, hi + d, hi + w], s)
}

/// A path about the rectangle `rc`: on, inside, outside, around, along and across it.
pub fn gen_rc_path(rc: &[i64; 4], s: &mut u64) -> Vec<P64> {
    let [l, t, r, b] = *rc;
    let w = (r - l).abs().max(1);
    let h = (b - t).abs().max(1);
    let (lo_x, hi_x, lo_y, hi_y) = (l.min(r), l.max(r), t.min(b), t.max(b));
    let mut p: Vec<P64> = Vec::new();
    match below(13, s) {
        0..=2 => {
            for _ in 0..between(1, 14, s) {
                p.push([lattice(l, r, s), lattice(t, b, s), 0]);
            }
        }
        3 => {
            for _ in 0..between(3, 12, s) {
                p.push([between(lo_x - w, hi_x + w, s), between(lo_y - h, hi_y + h, s), 0]);
            }
        }
        4 => {
            for _ in 0..between(3, 10, s) {
                p.push([between(lo_x, hi_x, s), between(lo_y, hi_y, s), 0]);
            }
        }
        5 => {
            let side = below(4, s);
            for _ in 0..between(3, 8, s) {
                let (x, y) = match side {
                    0 => (between(hi_x + 1, hi_x + w + 5, s), between(lo_y - h, hi_y + h, s)),
                    1 => (between(lo_x - w - 5, lo_x - 1, s), between(lo_y - h, hi_y + h, s)),
                    2 => (between(lo_x - w, hi_x + w, s), between(hi_y + 1, hi_y + h + 5, s)),
                    _ => (between(lo_x - w, hi_x + w, s), between(lo_y - h - 5, lo_y - 1, s)),
                };
                p.push([x, y, 0]);
            }
        }
        6 => {
            // around the rectangle, at a distance or on its edges
            let m = |s: &mut u64, w: i64| pick(&[0, 0, 1, 5, w, 2 * w], s);
            let (x0, y0, x1, y1) = (lo_x - m(s, w), lo_y - m(s, h), hi_x + m(s, w), hi_y + m(s, h));
            let corners = [[x0, y0, 0], [x1, y0, 0], [x1, y1, 0], [x0, y1, 0]];
            for k in 0..4 {
                p.push(corners[k]);
                if chance(1, 3, s) {
                    let n = corners[(k + 1) % 4];
                    p.push([(corners[k][0] + n[0]) / 2, (corners[k][1] + n[1]) / 2, 0]);
                }
            }
            if chance(1, 2, s) {
                p.reverse();
            }
            let k = below(p.len() as u64, s) as usize;
            p.rotate_left(k);
        }
        7 => {
            // along the boundary, with excursions
            for _ in 0..between(3, 12, s) {
                if chance(1, 4, s) {
                    p.push([lattice(l, r, s), lattice(t, b, s), 0]);
                } else {
                    let q = match below(4, s) {
                        0 => [between(lo_x, hi_x, s), t, 0],
                        1 => [r, between(lo_y, hi_y, s), 0],
                        2 => [between(lo_x, hi_x, s), b, 0],
                        _ => [l, between(lo_y, hi_y, s), 0],
                    };
                    p.push(q);
                }
            }
        }
        8 => {
            // across, many times
            let d = pick(&[0, 1, 3, w], s);
            let n = between(4, 16, s);
            let horizontal = chance(1, 2, s);
            for k in 0..n {
                let far = if k % 2 == 0 { -1 } else { 1 };
                let inside = chance(1, 5, s);
                if horizontal {
                    let x = if inside { (l + r) / 2 } else if far < 0 { lo_x - d } else { hi_x + d };
                    p.push([x, lo_y - 2 + k * (h + 4) / n, 0]);
                } else {
                    let y = if inside { (t + b) / 2 } else if far < 0 { lo_y - d } else { hi_y + d };
                    p.push([lo_x - 2 + k * (w + 4) / n, y, 0]);
                }
            }
        }
        9 => {
            let (x0, x1, y0, y1) = (lattice(l, r, s), lattice(l, r, s), lattice(t, b, s), lattice(t, b, s));
            p = vec![[x0, y0, 0], [x1, y0, 0], [x1, y1, 0], [x0, y1, 0]];
            if chance(1, 2, s) {
                p.reverse();
            }
        }
        10 => {
            // on the line of an edge, then off it
            let on = below(4, s);
            for _ in 0..between(2, 6, s) {
                p.push(match on {
                    0 => [lattice(l, r, s), t, 0],
                    1 => [lattice(l, r, s), b, 0],
                    2 => [l, lattice(t, b, s), 0],
                    _ => [r, lattice(t, b, s), 0],
                });
            }
            for _ in 0..between(0, 3, s) {
                p.push([lattice(l, r, s), lattice(t, b, s), 0]);
            }
        }
        11 => {
            for _ in 0..between(2, 6, s) {
                p.push([between(lo_x - 3 * w, hi_x + 3 * w, s), between(lo_y - 3 * h, hi_y + 3 * h, s), 0]);
            }
        }
        _ => {
            // a polygon of many vertices around the middle of the rectangle
            let n = between(5, 24, s);
            let (cx, cy) = ((l + r) / 2, (t + b) / 2);
            for k in 0..n {
                let a = k as f64 / n as f64 * std::f64::consts::TAU;
                let rad = if k % 2 == 0 { 1.0 } else { pick(&[0.3, 0.7, 1.0, 1.6], s) };
                let (rx, ry) = (w as f64 * pick(&[0.4, 0.5, 0.75, 1.2], s) * rad, h as f64 * pick(&[0.4, 0.5, 0.75, 1.2], s) * rad);
                p.push([cx + (a.cos() * rx) as i64, cy + (a.sin() * ry) as i64, 0]);
            }
        }
    }
    if !p.is_empty() && chance(3, 20, s) {
        let k = below(p.len() as u64, s) as usize;
        let q = p[k];
        p.insert(k, q);
    }
    if !p.is_empty() && chance(1, 10, s) {
        let q = p[0];
        p.push(q);
    }
    let zmode = below(3, s);
    for q in p.iter_mut() {
        q[2] = match zmode {
            0 => 0,
            1 => between(-3, 3, s),
            _ => xorshift(s) as i64 >> 8,
        };
    }
    p
}

pub fn gen_rc_paths(rc: &[i64; 4], s: &mut u64) -> Vec<Vec<P64>> {
    let n = pick(&[1u64, 1, 1, 1, 2, 2, 3, 4, 0], s);
    let mut pp = Vec::new();
    for _ in 0..n {
        if chance(1, 30, s) {
            pp.push(Vec::new());
        } else {
            pp.push(gen_rc_path(rc, s));
        }
    }
    pp
}

/// `RectClip` / `RectClipLines` on paths (ops 1 and 3) or one path (2 and 4), or one
/// `RectClip64` / `RectClipLines64` executed on two sets of paths (5 and 6).
pub fn gen_rectclip(op: i32, s: &mut u64) -> Enc {
    let mut e = Enc::default();
    let rc = gen_rect(s);
    e.rect64(&rc);
    match op {
        1 | 3 => e.paths64(&gen_rc_paths(&rc, s)),
        2 | 4 => {
            let p = if chance(1, 40, s) { Vec::new() } else { gen_rc_path(&rc, s) };
            e.path64(&p)
        }
        _ => {
            e.paths64(&gen_rc_paths(&rc, s));
            e.paths64(&gen_rc_paths(&rc, s));
        }
    }
    e
}

/// `GetSegmentIntersection` (op 9): two segments on a small lattice (collinear, touching,
/// sharing an end, crossing), the second often an edge of a rectangle, or far apart with
/// large coordinates.
pub fn gen_segment_intersection(s: &mut u64) -> Enc {
    let mut e = Enc::default();
    let big = chance(1, 4, s);
    let m = if big { 10i64.pow(between(3, 12, s) as u32) } else { pick(&[2, 4, 8, 20], s) };
    let mut pt = |s: &mut u64| -> P64 { [between(-m, m, s), between(-m, m, s), between(-9, 9, s)] };
    let (p1, p2) = (pt(s), pt(s));
    let (mut p3, mut p4) = (pt(s), pt(s));
    match below(4, s) {
        0 => p4[1] = p3[1],
        1 => p4[0] = p3[0],
        2 => {
            // p3 on the line of p1 p2
            p3 = [p1[0] + (p2[0] - p1[0]) * between(-1, 2, s), p1[1] + (p2[1] - p1[1]) * between(-1, 2, s), p3[2]];
            if chance(1, 2, s) {
                p4 = [p1[0] + (p2[0] - p1[0]) * between(-2, 3, s), p1[1] + (p2[1] - p1[1]) * between(-2, 3, s), p4[2]];
            }
        }
        _ => {}
    }
    for p in [p1, p2, p3, p4, [7, 8, 9]] {
        e.p64(&p);
    }
    e
}

pub fn gen_start_locs(s: &mut u64) -> Enc {
    let mut e = Enc::default();
    let n = below(9, s) as i64;
    e.i(n);
    for _ in 0..n {
        e.i(below(5, s) as i64);
    }
    e
}

/// A precision for the `PathsD` interface: mostly the usual ones, the whole range -8..8,
/// and some outside it (the error path).
pub fn gen_precision(s: &mut u64) -> i64 {
    pick(&[2, 2, 2, 2, 0, 1, 3, 4, 5, 6, 7, 8, -1, -2, -3, -4, -5, -6, -7, -8, 9, -9, 100, -100, 12], s)
}

/// A coordinate for a precision: a number with two decimals, a whole one, one a half unit
/// of the scaled grid off (the rounding ties), a binary fraction, or anything; times
/// 10^-precision where the precision is negative, so that the scaled path keeps its shape.
pub fn dval(precision: i64, s: &mut u64) -> f64 {
    let p = precision.clamp(-8, 8);
    let grow = if p < 0 { 10f64.powi(-p as i32) } else { 1.0 };
    let v = match below(8, s) {
        0 | 1 => between(-20000, 20000, s) as f64 / 100.0,
        2 => between(-200, 200, s) as f64,
        3 => (between(-2000, 2000, s) as f64 + 0.5) / 10f64.powi(p.max(0) as i32),
        4 => between(-4096, 4096, s) as f64 / 64.0,
        5 => between(-40, 40, s) as f64 * 0.25,
        6 => (xorshift(s) >> 11) as f64 / (1u64 << 53) as f64 * 400.0 - 200.0,
        _ => between(-12, 12, s) as f64,
    };
    v * grow
}

pub fn gen_pathd(n: i64, precision: i64, s: &mut u64) -> Vec<PD> {
    let mut p = Vec::new();
    let zmode = below(3, s);
    let shape = below(6, s);
    for k in 0..n {
        let z = match zmode {
            0 => 0,
            1 => between(-3, 3, s),
            _ => xorshift(s) as i64 >> 8,
        };
        if shape == 0 && n >= 4 {
            // a rectangle, then anything
            if k == 0 {
                let (x0, y0, x1, y1) = (dval(precision, s), dval(precision, s), dval(precision, s), dval(precision, s));
                p.extend_from_slice(&[(x0, y0, z), (x1, y0, z), (x1, y1, z), (x0, y1, z)]);
            } else if k >= 4 {
                p.push((dval(precision, s), dval(precision, s), z));
            }
        } else {
            p.push((dval(precision, s), dval(precision, s), z));
        }
    }
    if !p.is_empty() && chance(1, 8, s) {
        let k = below(p.len() as u64, s) as usize;
        let q = p[k];
        p.insert(k, q);
    }
    p
}

pub fn gen_pathsd(lo: i64, hi: i64, precision: i64, s: &mut u64) -> Vec<Vec<PD>> {
    let mut pp = Vec::new();
    for _ in 0..between(lo, hi, s) {
        let n = if chance(1, 25, s) { between(0, 2, s) } else { between(3, 10, s) };
        pp.push(gen_pathd(n, precision, s));
    }
    pp
}

fn gen_path64(n: i64, m: i64, s: &mut u64) -> Vec<P64> {
    let mut p = Vec::new();
    let zmode = below(3, s);
    for _ in 0..n {
        let z = match zmode {
            0 => 0,
            1 => between(-3, 3, s),
            _ => xorshift(s) as i64 >> 8,
        };
        p.push([between(-m, m, s), between(-m, m, s), z]);
    }
    if !p.is_empty() && chance(1, 8, s) {
        let k = below(p.len() as u64, s) as usize;
        let q = p[k];
        p.insert(k, q);
    }
    p
}

fn gen_paths64(lo: i64, hi: i64, s: &mut u64) -> Vec<Vec<P64>> {
    let mut pp = Vec::new();
    let m = pick(&[4, 10, 10, 100, 100, 1000, 1_000_000], s);
    for _ in 0..between(lo, hi, s) {
        let n = if chance(1, 25, s) { between(0, 2, s) } else { between(3, 10, s) };
        if chance(1, 5, s) && n >= 4 {
            let (x0, y0, x1, y1) = (between(-m, m, s), between(-m, m, s), between(-m, m, s), between(-m, m, s));
            pp.push(vec![[x0, y0, 1], [x1, y0, 2], [x1, y1, 3], [x0, y1, 4]]);
        } else {
            pp.push(gen_path64(n, m, s));
        }
    }
    pp
}

/// `RectClip` / `RectClipLines` on doubles (ops 7 and 8).
pub fn gen_rectclip_d(s: &mut u64) -> Enc {
    let mut e = Enc::default();
    let precision = gen_precision(s);
    e.i(precision);
    e.i(below(2, s) as i64);
    let (l, t) = (dval(precision, s), dval(precision, s));
    let rect = match below(10, s) {
        0 => [l, t, l, t + 3.0],
        1 => [l, t, l - 1.0, t + 3.0],
        _ => [l, t, l + dval(precision, s).abs() + 0.25, t + dval(precision, s).abs() + 0.25],
    };
    e.rectd(&rect);
    let mut paths = gen_pathsd(0, 3, precision, s);
    if chance(1, 40, s) && !paths.is_empty() && !paths[0].is_empty() {
        paths[0][0].0 = 1e19; // out of the coordinate range once scaled
    }
    e.pathsd(&paths);
    e
}

/// `ClipperD` (op 20).
pub fn gen_clipper_d(s: &mut u64) -> Enc {
    let mut e = Enc::default();
    let precision = gen_precision(s);
    e.i(precision);
    e.i(if chance(1, 30, s) { 0 } else { between(1, 4, s) });
    e.i(below(4, s) as i64);
    e.i(below(2, s) as i64);
    e.i(below(2, s) as i64);
    e.i(below(4, s) as i64);
    e.i(below(2, s) as i64);
    e.i(between(-5, 5, s));
    e.i(between(1, 2, s));
    e.i(between(1, 4, s));
    e.i(below(4, s) as i64);
    let mut subj = gen_pathsd(1, 3, precision, s);
    let open = if chance(1, 2, s) { Vec::new() } else { gen_pathsd(1, 2, precision, s) };
    let mut clip = gen_pathsd(0, 2, precision, s);
    if chance(1, 40, s) && !subj[0].is_empty() {
        subj[0][0].1 = -1e19;
    }
    if chance(1, 60, s) && !clip.is_empty() && !clip[0].is_empty() {
        clip[0][0].0 = 4e18;
    }
    e.pathsd(&subj);
    e.pathsd(&open);
    e.pathsd(&clip);
    e
}

/// `BooleanOp` and its wrappers on `PathsD` (op 21).
pub fn gen_boolean_d(s: &mut u64) -> Enc {
    let mut e = Enc::default();
    let precision = gen_precision(s);
    e.i(below(7, s) as i64);
    e.i(between(1, 4, s));
    e.i(below(4, s) as i64);
    e.i(precision);
    e.pathsd(&gen_pathsd(1, 3, precision, s));
    e.pathsd(&gen_pathsd(0, 2, precision, s));
    e
}

/// `InflatePaths` on `PathsD` (op 22).
pub fn gen_inflate_d(s: &mut u64) -> Enc {
    let mut e = Enc::default();
    let precision = pick(&[2, 2, 2, 1, 0, 3, 4, -1, 9, -9], s);
    e.i(below(4, s) as i64);
    e.i(below(5, s) as i64);
    e.i(precision);
    let grow = if precision < 0 { 10f64.powi(-precision as i32) } else { 1.0 };
    e.d(pick(&[0.0, 0.5, -0.5, 1.5, -1.5, 5.25, -5.25, 20.0, -20.0, 0.004], s) * grow);
    e.d(pick(&[2.0, 2.0, 1.0, 3.0, 0.0], s));
    e.d(pick(&[0.0, 0.0, 0.25, 0.01], s));
    // Without the empty paths: `ClipperOffset` reads the first point of a path it was given
    // (clipper.offset.cpp, `OffsetOpenPath` and `DoGroupOffset`), which an empty one has not.
    let mut paths = gen_pathsd(0, 2, precision, s);
    paths.retain(|p| !p.is_empty());
    e.pathsd(&paths);
    e
}

/// The helpers on `PathD` (op 23), sub-operation `which`.
pub fn gen_path_d(which: i64, s: &mut u64) -> Enc {
    let mut e = Enc::default();
    let precision = if which == 2 { gen_precision(s) } else { 2 };
    let flag = if which == 8 || which == 9 { pick(&[0, 0, 1, 3, 7, 12, 40], s) } else { below(2, s) as i64 };
    e.i(which);
    e.i(flag);
    e.i(precision);
    // the paths: half of them on a grid of quarters, where points are equal, collinear and
    // on each other's edges
    let grid = chance(1, 2, s);
    let mut coord = |s: &mut u64| if grid { between(-32, 32, s) as f64 * 0.25 } else { dval(precision, s) };
    let mut paths: Vec<Vec<PD>> = Vec::new();
    for k in 0..between(1, 3, s) {
        let n = if which == 14 && k == 0 { between(4, 6, s) } else if chance(1, 20, s) { between(0, 2, s) } else { between(3, 12, s) };
        let mut p = Vec::new();
        for _ in 0..n {
            p.push((coord(s), coord(s), between(-3, 3, s)));
            if chance(1, 10, s) {
                let q = *p.last().unwrap();
                p.push(q);
            }
        }
        if which == 14 && k == 0 {
            match below(6, s) {
                0 => p[1] = p[0],
                1 => p[2] = ((p[0].0 + p[1].0) / 2.0, (p[0].1 + p[1].1) / 2.0, 5),
                2 => p[3] = p[2],
                3 => p[2] = p[1],
                _ => {}
            }
        }
        paths.push(p);
    }
    let first = paths[0].clone();
    let (a, b, c, d) = match which {
        0 | 1 => (coord(s), coord(s), 0.0, 0.0),
        4..=7 => (pick(&[0.0, 0.25, 0.5, 1.0, 2.5, 10.0, 100.0], s), 0.0, 0.0, 0.0),
        8 => (pick(&[-1.0, 0.0, 0.5, 3.0, 10.25, 100.0, 1e-3], s), pick(&[0.0, -1.0, 2.0, 7.5, 0.0], s), coord(s), coord(s)),
        9 => {
            let (l, t) = (coord(s), coord(s));
            (l, t, l + pick(&[-1.0, 0.0, 2.0, 9.5, 30.25], s), t + pick(&[0.0, 3.0, 9.5, 12.125], s))
        }
        11 => {
            // a point of the path, the middle of an edge, or anywhere
            if first.len() >= 2 && chance(2, 3, s) {
                let k = below(first.len() as u64, s) as usize;
                let n = (k + 1) % first.len();
                if chance(1, 2, s) {
                    (first[k].0, first[k].1, 0.0, 0.0)
                } else {
                    ((first[k].0 + first[n].0) / 2.0, (first[k].1 + first[n].1) / 2.0, 0.0, 0.0)
                }
            } else {
                (coord(s), coord(s), 0.0, 0.0)
            }
        }
        13 => (pick(&[0.0, 0.01, 0.0625, 1.0, 4.0, 100.0], s), 0.0, 0.0, 0.0),
        14 => (pick(&[0.001, 0.5, 2.0, -1.0, 0.0625, 1e-9], s), coord(s), 0.0, 0.0),
        _ => (0.0, 0.0, 0.0, 0.0),
    };
    for v in [a, b, c, d] {
        e.d(v);
    }
    e.pathsd(&paths);
    e
}

/// The helpers on `Path64` (op 24).
pub fn gen_path_64(s: &mut u64) -> Enc {
    let mut e = Enc::default();
    let which = below(3, s) as i64;
    e.i(which);
    e.i(if which == 0 { pick(&[0, 0, 1, 3, 7, 12, 40], s) } else { below(2, s) as i64 });
    e.d(pick(&[0.0, 0.5, 1.5, -2.5, 100.25, 4.0, 1e6], s));
    e.d(pick(&[0.5, -0.5, 1.5, 2.5, -1000.5, 3.0], s));
    let mut paths = gen_paths64(1, 3, s);
    if which == 0 {
        let m = pick(&[4, 100, 30000, 1_000_000], s);
        paths[0] = gen_path64(between(4, 6, s), m, s);
        while paths[0].len() < 4 {
            paths[0].push([1, 2, 3]);
        }
        if chance(1, 4, s) {
            paths[0][1] = paths[0][0];
        }
    }
    e.paths64(&paths);
    e
}

/// `ScalePath` and `ScalePaths` (op 25).
pub fn gen_scale(s: &mut u64) -> Enc {
    let mut e = Enc::default();
    let variant = below(4, s) as i64;
    e.i(variant);
    e.i(below(2, s) as i64);
    e.i(below(2, s) as i64);
    let scales = [1.0, 100.0, 0.01, 0.0, -1.0, 2.5, 1e-3, 0.5, 1e18, 134217728.0, 7.450580596923828e-9, 0.0, 3.0, -100.0];
    e.d(pick(&scales, s));
    e.d(pick(&scales, s));
    if variant == 0 || variant == 3 {
        let mut paths = gen_pathsd(0, 3, 2, s);
        if chance(1, 12, s) && !paths.is_empty() && !paths[0].is_empty() {
            paths[0][0].0 = pick(&[1e17, -1e17, 4.7e18, 2.305843009213694e16], s);
        }
        e.pathsd(&paths);
    } else {
        let mut paths = gen_paths64(0, 3, s);
        if chance(1, 12, s) && !paths.is_empty() && !paths[0].is_empty() {
            paths[0][0][1] = pick(&[100_000_000_000_000_000, -100_000_000_000_000_000, 2_305_843_009_213_693_951, 23_058_430_092_136_939], s);
        }
        e.paths64(&paths);
    }
    e
}

/// `TransformPath` and `TransformPaths` (op 26).
pub fn gen_transform(s: &mut u64) -> Enc {
    let mut e = Enc::default();
    let variant = below(2, s) as i64;
    e.i(variant);
    if variant == 0 {
        e.pathsd(&gen_pathsd(0, 3, 2, s));
    } else {
        e.paths64(&gen_paths64(0, 3, s));
    }
    e
}

/// `CheckPrecisionRange` (op 27).
pub fn gen_check_precision(s: &mut u64) -> Enc {
    let mut e = Enc::default();
    let p = |s: &mut u64| if chance(1, 6, s) { pick(&[100, -100, 1 << 20, -(1 << 20)], s) } else { between(-12, 12, s) };
    e.i(p(s));
    e.i(pick(&[0, 2, 64, 1], s));
    e.i(p(s));
    e
}

/// `Rect<double>`, `Rect<int64_t>::Scale` and `ScaleRect` (op 28).
pub fn gen_rect_d(s: &mut u64) -> Enc {
    let mut e = Enc::default();
    let mut rect = |s: &mut u64| -> [f64; 4] {
        let (l, t) = (dval(2, s), dval(2, s));
        match below(8, s) {
            0 => [l, t, l, t + 1.0],
            1 => [l, t, l - 2.0, t - 1.0],
            2 => [f64::MAX, f64::MAX, f64::MIN, f64::MIN],
            _ => [l, t, l + dval(2, s).abs(), t + dval(2, s).abs()],
        }
    };
    let r = rect(s);
    let r2 = if chance(1, 6, s) { r } else { rect(s) };
    e.rectd(&r);
    e.rectd(&r2);
    e.d(pick(&[0.5, 100.0, -1.0, 0.0, 3.25, 1.0, 0.01], s));
    if chance(1, 3, s) {
        // on the rectangle's edge or corner
        e.d(pick(&[r[0], r[2], (r[0] + r[2]) / 2.0], s));
        e.d(pick(&[r[1], r[3], (r[1] + r[3]) / 2.0], s));
    } else {
        e.d(dval(2, s));
        e.d(dval(2, s));
    }
    let m = pick(&[10, 1000, 1_000_000_000], s);
    e.rect64(&[between(-m, m, s), between(-m, m, s), between(-m, m, s), between(-m, m, s)]);
    e
}

/// `MinkowskiSum` and `MinkowskiDiff` (ops 30 and 31).
pub fn gen_minkowski(op: i32, s: &mut u64) -> Enc {
    let mut e = Enc::default();
    e.i(below(2, s) as i64);
    e.i(below(2, s) as i64);
    if op == 30 {
        let m = pick(&[3, 10, 50], s);
        e.path64(&gen_path64(between(0, 6, s), m, s));
        e.path64(&gen_path64(between(0, 8, s), 100, s));
    } else {
        let decimals = pick(&[2, 2, 2, 0, 1, 3, 4, 6, -1, -2, -400], s);
        e.i(decimals);
        let small = |s: &mut u64| -> Vec<PD> { (0..between(0, 6, s)).map(|_| (between(-40, 40, s) as f64 * 0.125, between(-40, 40, s) as f64 * 0.125, between(-2, 2, s))).collect() };
        e.pathd(&small(s));
        e.pathd(&gen_pathd(between(0, 8, s), decimals.max(-2), s));
    }
    e
}

/// `MakePath`, `MakePathD`, `MakePathZ`, `MakePathZD` (op 32).
pub fn gen_make_path(s: &mut u64) -> Enc {
    let mut e = Enc::default();
    let n = below(11, s) as i64;
    e.i(n);
    for _ in 0..n {
        e.i(between(-1000, 1000, s));
    }
    for _ in 0..n {
        e.d(dval(2, s));
    }
    e
}

/// `ReuseableDataContainer64` (op 33).
pub fn gen_reuseable(s: &mut u64) -> Enc {
    let mut e = Enc::default();
    e.i(between(1, 4, s));
    e.i(below(4, s) as i64);
    e.i(between(1, 4, s));
    e.i(below(4, s) as i64);
    e.i(below(2, s) as i64);
    e.i(below(64, s) as i64);
    e.i(below(2, s) as i64);
    e.i(below(2, s) as i64);
    e.paths64(&gen_paths64(1, 3, s));
    e.paths64(&if chance(1, 2, s) { Vec::new() } else { gen_paths64(1, 2, s) });
    e.paths64(&gen_paths64(0, 2, s));
    e.paths64(&gen_paths64(0, 1, s));
    e
}

/// A `PolyPathD` made by hand (op 34).
pub fn gen_polypath_d(s: &mut u64) -> Enc {
    let mut e = Enc::default();
    e.d(pick(&[1.0, 0.5, 0.0, 100.0, 0.01, 0.25], s));
    e.d(pick(&[2.0, 0.0, 1.0, 0.125], s));
    let p64 = vec![gen_path64(between(0, 6, s), 100, s), gen_path64(between(0, 6, s), 100, s)];
    e.paths64(&p64);
    e.pathsd(&[gen_pathd(between(0, 6, s), 2, s), gen_pathd(between(0, 6, s), 2, s)]);
    e
}

/// `operator<<` of a `PolyTree64` (op 35): nested rectangles, so that the tree has holes
/// and polygons in them, and anything.
pub fn gen_tree_string(s: &mut u64) -> Enc {
    let mut e = Enc::default();
    e.i(between(1, 4, s));
    e.i(below(2, s) as i64);
    let mut subj: Vec<Vec<P64>> = Vec::new();
    for g in 0..between(1, 3, s) {
        let (cx, cy) = (g * 300, between(-20, 20, s));
        for k in 0..between(1, 5, s) {
            let h = 100 - 15 * k;
            let mut p = vec![[cx - h, cy - h, 0], [cx + h, cy - h, 0], [cx + h, cy + h, 0], [cx - h, cy + h, 0]];
            if k % 2 == 1 {
                p.reverse();
            }
            subj.push(p);
        }
    }
    e.paths64(&subj);
    e.paths64(&gen_paths64(0, 2, s));
    e
}

pub fn dump(op: i32, input: &Enc) -> String {
    format!("op {op}\n  ints {:?}\n  dbls {:?}", input.ints, input.dbls)
}

pub fn first_difference(rust: &Enc, cpp: &Enc) -> String {
    if rust.ints != cpp.ints {
        let k = rust.ints.iter().zip(&cpp.ints).position(|(a, b)| a != b).unwrap_or(rust.ints.len().min(cpp.ints.len()));
        return format!(
            "ints differ at {k} (rust has {}, c++ has {}): rust {:?} c++ {:?}",
            rust.ints.len(),
            cpp.ints.len(),
            &rust.ints[k.saturating_sub(4)..(k + 6).min(rust.ints.len())],
            &cpp.ints[k.saturating_sub(4)..(k + 6).min(cpp.ints.len())]
        );
    }
    let k = rust.dbls.iter().zip(&cpp.dbls).position(|(a, b)| a.to_bits() != b.to_bits()).unwrap_or(rust.dbls.len().min(cpp.dbls.len()));
    format!(
        "doubles differ at {k} (rust has {}, c++ has {}): rust {:?} c++ {:?}",
        rust.dbls.len(),
        cpp.dbls.len(),
        rust.dbls.get(k).map(|v| (v, v.to_bits())),
        cpp.dbls.get(k).map(|v| (v, v.to_bits()))
    )
}

/// What a set of cases came to.
#[derive(Default, Debug, Clone)]
pub struct Totals {
    pub cases: u64,
    pub mismatches: u64,
    /// cases in which the library threw
    pub exceptions: u64,
    /// cases whose answer holds a point
    pub with_output: u64,
    pub out_ints: u64,
    pub out_dbls: u64,
}

impl Totals {
    pub fn add(&mut self, o: &Totals) {
        self.cases += o.cases;
        self.mismatches += o.mismatches;
        self.exceptions += o.exceptions;
        self.with_output += o.with_output;
        self.out_ints += o.out_ints;
        self.out_dbls += o.out_dbls;
    }
}

/// The operations on the Rust port, for the build the file names `c2` (with its `Z`, `mk`,
/// `zof` and `install_zcb_d`), and the runner of a set of cases.
#[macro_export]
macro_rules! ext_rust_impl {
    () => {
        use std::cell::Cell;

        thread_local! {
            /// A mutation of the Rust side the mutation checks switch on: 0 none.
            static MUTATION: Cell<u32> = const { Cell::new(0) };
            /// The `RectClipStats` of the `RectClip64`s of op 5, added up.
            static RC_STATS: std::cell::RefCell<[u64; 10]> = const { std::cell::RefCell::new([0; 10]) };
            /// The calls of the `ZCallbackD` of op 20, added up.
            static ZCB_CALLS: Cell<u64> = const { Cell::new(0) };
            /// The first four of the `RectClipLines64`s of op 6.
            static RCL_STATS: std::cell::RefCell<[u64; 4]> = const { std::cell::RefCell::new([0; 4]) };
        }

        const RC_STAT_NAMES: [&str; 10] =
            ["all inside", "entering", "exiting", "passing through", "passing through at one point", "path contains the rectangle", "corners added", "collinear vertices unlinked", "splits", "rejoins"];

        struct Rd<'a> {
            e: &'a common::Enc,
            ni: usize,
            nd: usize,
        }

        impl<'a> Rd<'a> {
            fn i(&mut self) -> i64 {
                self.ni += 1;
                self.e.ints[self.ni - 1]
            }
            fn d(&mut self) -> f64 {
                self.nd += 1;
                self.e.dbls[self.nd - 1]
            }
            fn point64(&mut self) -> c2::Point64 {
                let q = [self.i(), self.i(), self.i()];
                mk(&q)
            }
            fn pointd(&mut self) -> c2::PointD {
                let (x, y) = (self.d(), self.d());
                let z = self.i();
                c2::PointD::new_xyz(x, y, z)
            }
            fn path64(&mut self) -> c2::Path64 {
                (0..self.i()).map(|_| self.point64()).collect()
            }
            fn paths64(&mut self) -> c2::Paths64 {
                (0..self.i()).map(|_| self.path64()).collect()
            }
            fn pathd(&mut self) -> c2::PathD {
                (0..self.i()).map(|_| self.pointd()).collect()
            }
            fn pathsd(&mut self) -> c2::PathsD {
                (0..self.i()).map(|_| self.pathd()).collect()
            }
            fn rect64(&mut self) -> c2::Rect64 {
                let (l, t, r, b) = (self.i(), self.i(), self.i(), self.i());
                c2::Rect64::new(l, t, r, b)
            }
            fn rectd(&mut self) -> c2::RectD {
                let (l, t, r, b) = (self.d(), self.d(), self.d(), self.d());
                c2::RectD::new(l, t, r, b)
            }
        }

        #[derive(Default)]
        struct Wr {
            e: common::Enc,
        }

        impl Wr {
            fn i(&mut self, v: i64) {
                self.e.ints.push(v);
            }
            fn d(&mut self, v: f64) {
                self.e.dbls.push(v);
            }
            fn point64(&mut self, p: &c2::Point64) {
                self.i(p.x);
                self.i(p.y);
                self.i(zof(p));
            }
            fn pointd(&mut self, p: &c2::PointD) {
                self.d(p.x);
                self.d(p.y);
                self.i(p.z_or_zero());
            }
            fn path64(&mut self, p: &c2::Path64) {
                self.i(p.len() as i64);
                for q in p {
                    self.point64(q);
                }
            }
            fn paths64(&mut self, pp: &c2::Paths64) {
                self.i(pp.len() as i64);
                for p in pp {
                    self.path64(p);
                }
            }
            fn pathd(&mut self, p: &c2::PathD) {
                self.i(p.len() as i64);
                for q in p {
                    self.pointd(q);
                }
            }
            fn pathsd(&mut self, pp: &c2::PathsD) {
                self.i(pp.len() as i64);
                for p in pp {
                    self.pathd(p);
                }
            }
            fn rect64(&mut self, r: &c2::Rect64) {
                for v in [r.left, r.top, r.right, r.bottom] {
                    self.i(v);
                }
            }
            fn rectd(&mut self, r: &c2::RectD) {
                for v in [r.left, r.top, r.right, r.bottom] {
                    self.d(v);
                }
            }
            fn str(&mut self, s: &str) {
                self.i(s.len() as i64);
                for c in s.bytes() {
                    self.i(c as i64);
                }
            }
            fn tree64(&mut self, node: &c2::PolyPath64) {
                self.i(node.count() as i64);
                for child in node.iter() {
                    self.i(child.is_hole() as i64);
                    self.i(child.level() as i64);
                    self.path64(child.polygon());
                    self.tree64(child);
                }
            }
            fn treed(&mut self, node: &c2::PolyPathD) {
                self.i(node.count() as i64);
                for child in node.iter() {
                    self.i(child.is_hole() as i64);
                    self.i(child.level() as i64);
                    self.d(child.scale());
                    self.pathd(child.polygon());
                    self.treed(child);
                }
            }
            fn thrown(&mut self, e: &c2::Clipper2Exception) {
                self.i(-778);
                self.i(code_of(e));
            }
        }

        /// The number of an exception, as `code_of` of clipper2_ref_ext_shim.cpp.
        fn code_of(e: &c2::Clipper2Exception) -> i64 {
            match e.what() {
                "Precision exceeds the permitted range" => c2::PRECISION_ERROR_I as i64,
                "Invalid scale (either 0 or too large)" => c2::SCALE_ERROR_I as i64,
                "There must be 2 values for each coordinate" => c2::NON_PAIR_ERROR_I as i64,
                "There is an undefined error in Clipper2" => c2::UNDEFINED_ERROR_I as i64,
                "Values exceed permitted range" => c2::RANGE_ERROR_I as i64,
                _ => -1,
            }
        }

        fn clip_type(v: i64) -> c2::ClipType {
            [c2::ClipType::NoClip, c2::ClipType::Intersection, c2::ClipType::Union, c2::ClipType::Difference, c2::ClipType::Xor][v as usize]
        }

        fn fill_rule(v: i64) -> c2::FillRule {
            [c2::FillRule::EvenOdd, c2::FillRule::NonZero, c2::FillRule::Positive, c2::FillRule::Negative][v as usize]
        }

        fn location(v: i64) -> c2::Location {
            [c2::Location::Left, c2::Location::Top, c2::Location::Right, c2::Location::Bottom, c2::Location::Inside][v as usize]
        }

        /// `run` of clipper2_ref_ext_shim.cpp on the Rust port.
        fn rust_run(op: i32, rd: &mut Rd, out: &mut Wr) -> Result<(), c2::Clipper2Exception> {
            let mutation = MUTATION.with(|m| m.get());
            match op {
                1 | 3 => {
                    let mut rect = rd.rect64();
                    let paths = rd.paths64();
                    // mutation 1: the rectangle one unit wider
                    if mutation == 1 {
                        rect.right += 1;
                    }
                    let mut res = if op == 1 { c2::rect_clip(&rect, &paths) } else { c2::rect_clip_lines(&rect, &paths) };
                    // mutation 2: the first path from its second vertex
                    if mutation == 2 && !res.is_empty() && res[0].len() > 1 {
                        res[0].rotate_left(1);
                    }
                    out.paths64(&res);
                }
                2 | 4 => {
                    let rect = rd.rect64();
                    let path = rd.path64();
                    out.paths64(&if op == 2 { c2::rect_clip_path(&rect, &path) } else { c2::rect_clip_lines_path(&rect, &path) });
                }
                5 => {
                    let rect = rd.rect64();
                    let (a, b) = (rd.paths64(), rd.paths64());
                    let mut rc = c2::RectClip64::new(&rect);
                    out.paths64(&rc.execute(&a));
                    out.paths64(&rc.execute(&b));
                    RC_STATS.with(|t| {
                        let mut t = t.borrow_mut();
                        let st = rc.stats;
                        for (k, v) in [st.all_inside, st.entering, st.exiting, st.passing_through, st.passing_through_one_point, st.contains_rect, st.corners, st.collinear_unlinked, st.splits, st.rejoins].into_iter().enumerate() {
                            t[k] += v;
                        }
                    });
                }
                6 => {
                    let rect = rd.rect64();
                    let (a, b) = (rd.paths64(), rd.paths64());
                    let mut rc = c2::RectClipLines64::new(&rect);
                    out.paths64(&rc.execute(&a));
                    out.paths64(&rc.execute(&b));
                    RCL_STATS.with(|t| {
                        let mut t = t.borrow_mut();
                        let st = rc.stats();
                        for (k, v) in [st.all_inside, st.entering, st.exiting, st.passing_through].into_iter().enumerate() {
                            t[k] += v;
                        }
                    });
                }
                7 | 8 => {
                    let mut precision = rd.i() as i32;
                    let single = rd.i();
                    let rect = rd.rectd();
                    let paths = rd.pathsd();
                    // mutation 4: one more decimal
                    if mutation == 4 && precision < 8 && precision >= -8 {
                        precision += 1;
                    }
                    let res = if single != 0 {
                        let first = paths.first().cloned().unwrap_or_default();
                        if op == 7 {
                            c2::rect_clip_path_d(&rect, &first, precision)?
                        } else {
                            c2::rect_clip_lines_path_d(&rect, &first, precision)?
                        }
                    } else if op == 7 {
                        c2::rect_clip_d(&rect, &paths, precision)?
                    } else {
                        c2::rect_clip_lines_d(&rect, &paths, precision)?
                    };
                    out.pathsd(&res);
                }
                9 => {
                    let (p1, p2, mut p3, mut p4) = (rd.point64(), rd.point64(), rd.point64(), rd.point64());
                    let mut ip = rd.point64();
                    // mutation 3: the second segment the other way round
                    if mutation == 3 {
                        std::mem::swap(&mut p3, &mut p4);
                    }
                    let res = c2::get_segment_intersection(&p1, &p2, &p3, &p4, &mut ip);
                    out.i(res as i64);
                    out.point64(&ip);
                }
                10 => {
                    let locs: Vec<c2::Location> = (0..rd.i()).map(|_| location(rd.i())).collect();
                    out.i(c2::start_locs_are_clockwise(&locs) as i64);
                }

                20 => {
                    let mut precision = rd.i() as i32;
                    let (ct, fr) = (rd.i(), rd.i());
                    let (preserve_collinear, reverse_solution) = (rd.i(), rd.i());
                    let (mode, zcb, default_z) = (rd.i(), rd.i(), rd.i());
                    let (execs, ct2, fr2) = (rd.i(), rd.i(), rd.i());
                    let (subj, open, clip) = (rd.pathsd(), rd.pathsd(), rd.pathsd());
                    if mutation == 4 && precision < 8 && precision >= -8 {
                        precision += 1;
                    }
                    let state = std::rc::Rc::new(std::cell::RefCell::new((0u64, HASH_START)));
                    let mut clipper = c2::ClipperD::new(precision)?;
                    clipper.set_preserve_collinear(preserve_collinear != 0);
                    clipper.set_reverse_solution(reverse_solution != 0);
                    out.i(clipper.preserve_collinear() as i64);
                    out.i(clipper.reverse_solution() as i64);
                    if Z {
                        clipper.default_z = default_z;
                        if zcb == 1 {
                            install_zcb_d(&mut clipper, state.clone());
                        }
                    }
                    for kind in 0..3 {
                        let added = match kind {
                            0 => clipper.add_subject(&subj),
                            1 => clipper.add_open_subject(&open),
                            _ => clipper.add_clip(&clip),
                        };
                        if let Err(e) = added {
                            out.thrown(&e);
                        }
                        out.i(clipper.error_code() as i64);
                    }
                    for e in 0..execs {
                        let ct = clip_type(if e == 0 { ct } else { ct2 });
                        let fr = fill_rule(if e == 0 { fr } else { fr2 });
                        if mode < 2 {
                            let (mut closed, mut opened) = (c2::PathsD::new(), c2::PathsD::new());
                            let ok = if mode == 0 { clipper.execute(ct, fr, &mut closed) } else { clipper.execute_open(ct, fr, &mut closed, &mut opened) };
                            out.i(ok as i64);
                            out.pathsd(&closed);
                            out.pathsd(&opened);
                        } else {
                            let mut tree = c2::PolyTreeD::new();
                            let mut opened = c2::PathsD::new();
                            let ok = if mode == 2 { clipper.execute_tree(ct, fr, &mut tree) } else { clipper.execute_tree_open(ct, fr, &mut tree, &mut opened) };
                            out.i(ok as i64);
                            out.treed(&tree);
                            out.pathsd(&opened);
                            out.pathsd(&c2::poly_tree_to_paths_d(&tree));
                            out.d(tree.area());
                            out.d(tree.scale());
                            out.str(&c2::poly_tree_d_to_string(&tree));
                        }
                        out.i(clipper.error_code() as i64);
                    }
                    let st = state.borrow();
                    out.i(st.0 as i64);
                    out.i(st.1 as i64);
                    ZCB_CALLS.with(|n| n.set(n.get() + st.0));
                }

                21 => {
                    let (which, ct, fr) = (rd.i(), rd.i(), rd.i());
                    let mut precision = rd.i() as i32;
                    let (subj, clip) = (rd.pathsd(), rd.pathsd());
                    if mutation == 4 && precision < 8 && precision >= -8 {
                        precision += 1;
                    }
                    let fr = fill_rule(fr);
                    match which {
                        0 => out.pathsd(&c2::boolean_op_d(clip_type(ct), fr, &subj, &clip, precision)?),
                        1 => {
                            let mut tree = c2::PolyTreeD::new();
                            tree.add_child_d(&vec![c2::PointD::new_xyz(1.0, 2.0, 3)]); // BooleanOp clears the tree first
                            c2::boolean_op_tree_d(clip_type(ct), fr, &subj, &clip, &mut tree, precision)?;
                            out.treed(&tree);
                        }
                        2 => out.pathsd(&c2::intersect_d(&subj, &clip, fr, precision)?),
                        3 => out.pathsd(&c2::union_d(&subj, &clip, fr, precision)?),
                        4 => out.pathsd(&c2::union_subjects_d(&subj, fr, precision)?),
                        5 => out.pathsd(&c2::difference_d(&subj, &clip, fr, precision)?),
                        _ => out.pathsd(&c2::xor_d(&subj, &clip, fr, precision)?),
                    }
                }
                22 => {
                    let (jt, et) = (rd.i(), rd.i());
                    let precision = rd.i() as i32;
                    let (delta, miter_limit, arc_tolerance) = (rd.d(), rd.d(), rd.d());
                    let paths = rd.pathsd();
                    let jt = [c2::JoinType::Square, c2::JoinType::Bevel, c2::JoinType::Round, c2::JoinType::Miter][jt as usize];
                    let et = [c2::EndType::Polygon, c2::EndType::Joined, c2::EndType::Butt, c2::EndType::Square, c2::EndType::Round][et as usize];
                    out.pathsd(&c2::inflate_paths_d(&paths, delta, jt, et, miter_limit, precision, arc_tolerance)?);
                }
                23 => {
                    let (which, flag) = (rd.i(), rd.i());
                    let precision = rd.i() as i32;
                    let (a, b, c, d) = (rd.d(), rd.d(), rd.d(), rd.d());
                    let paths = rd.pathsd();
                    let first = paths.first().cloned().unwrap_or_default();
                    match which {
                        0 => out.pathd(&c2::translate_path_d(&first, a, b)),
                        1 => out.pathsd(&c2::translate_paths_d(&paths, a, b)),
                        2 => out.pathd(&c2::trim_collinear_d(&first, precision, flag != 0)?),
                        3 => out.d(c2::length_d(&first, flag != 0)),
                        4 => out.pathd(&c2::simplify_path_d(&first, a, flag != 0)),
                        5 => out.pathsd(&c2::simplify_paths_d(&paths, a, flag != 0)),
                        6 => out.pathd(&c2::ramer_douglas_peucker_d(&first, a)),
                        7 => out.pathsd(&c2::ramer_douglas_peucker_paths_d(&paths, a)),
                        8 => out.pathd(&c2::ellipse_d(&c2::PointD::new(c, d), a, b, flag as usize)),
                        9 => out.pathd(&c2::ellipse_rect_d(&c2::RectD::new(a, b, c, d), flag as usize)),
                        10 => {
                            out.d(c2::area_d(&first));
                            out.d(c2::area_paths_d(&paths));
                            out.i(c2::is_positive_d(&first) as i64);
                        }
                        11 => out.i(match c2::point_in_polygon_d(&c2::PointD::new(a, b), &first) {
                            c2::PointInPolygonResult::IsOn => 0,
                            c2::PointInPolygonResult::IsInside => 1,
                            c2::PointInPolygonResult::IsOutside => 2,
                        }),
                        12 => {
                            out.rectd(&c2::get_bounds_d(&first));
                            out.rectd(&c2::get_bounds_paths_d(&paths));
                            out.rect64(&c2::get_bounds_64_from_d(&first));
                            out.rect64(&c2::get_bounds_paths_64_from_d(&paths));
                            out.rectd(&c2::get_bounds_paths_d(&paths));
                        }
                        13 => {
                            out.pathd(&c2::strip_near_equal_d(&first, a, flag != 0));
                            out.pathsd(&c2::strip_near_equal_paths_d(&paths, a, flag != 0));
                            let mut p = first.clone();
                            c2::strip_duplicates_d(&mut p, flag != 0);
                            out.pathd(&p);
                            let mut pp = paths.clone();
                            c2::strip_duplicates_paths_d(&mut pp, flag != 0);
                            out.pathsd(&pp);
                        }
                        _ => {
                            if first.len() >= 4 {
                                let (p0, p1, p2, p3) = (first[0], first[1], first[2], first[3]);
                                out.i(c2::near_collinear_d(&p0, &p1, &p2, a) as i64);
                                out.d(c2::distance_d(p0, p1));
                                out.d(c2::distance_sqr_d(p0, p1));
                                out.d(c2::cross_product_d(&p0, &p1, &p2));
                                out.d(c2::dot_product_d(&p0, &p1, &p2));
                                out.d(c2::cross_product_vec(&p0, &p1));
                                out.d(c2::dot_product_vec(&p0, &p1));
                                out.d(c2::perpendic_dist_from_line_sqrd_d(&p0, &p1, &p2));
                                out.pointd(&c2::mid_point_d(&p0, &p1));
                                out.i(c2::near_equal_d(&p0, &p1, a) as i64);
                                out.pointd(&c2::get_closest_point_on_segment_d(&p0, &p1, &p2));
                                let mut ip = p3;
                                out.i(c2::get_segment_intersect_pt_d(&p0, &p1, &p2, &p3, &mut ip) as i64);
                                out.pointd(&ip);
                                out.pointd(&c2::translate_point_d(&p0, a, b));
                                out.pointd(&c2::reflect_point_d(&p0, &p1));
                                out.pointd(&(p0 * a));
                                out.pointd(&(-p0));
                                out.pointd(&(p0 + p1));
                                out.pointd(&(p0 - p1));
                                out.point64(&c2::point64_from_pointd(&p0));
                                out.i((p0 == p1) as i64);
                                out.point64(&if Z { c2::Point64::from_pointd_z(&p0, 77) } else { c2::point64_from_pointd(&p0) });
                                let mut q = p0;
                                q.negate();
                                out.pointd(&q);
                            }
                        }
                    }
                }
                24 => {
                    let (which, flag) = (rd.i(), rd.i());
                    let (a, b) = (rd.d(), rd.d());
                    let paths = rd.paths64();
                    let first = paths.first().cloned().unwrap_or_default();
                    match which {
                        0 => {
                            if first.len() >= 4 {
                                let (p0, p1, p2) = (first[0], first[1], first[2]);
                                out.i(c2::near_equal(&p0, &p1, a) as i64);
                                out.point64(&c2::translate_point(&p0, a, b));
                                out.point64(&c2::reflect_point(&p0, &p1));
                                out.d(c2::cross_product_vec64(&p0, &p1));
                                out.d(c2::dot_product_vec64(&p0, &p1));
                                out.point64(&(p0 * a));
                                out.pointd(&c2::pointd_from_point64(&p0));
                                out.pointd(&if Z { c2::PointD::from_point64_z(&p0, 77) } else { c2::pointd_from_point64(&p0) });
                                let r = c2::Rect64::new(p0.x, p0.y, p2.x, p2.y);
                                out.path64(&c2::ellipse_rect(&r, flag as usize));
                            }
                        }
                        1 => {
                            out.path64(&c2::strip_near_equal(&first, a, flag != 0));
                            out.paths64(&c2::strip_near_equal_paths(&paths, a, flag != 0));
                        }
                        _ => {
                            out.rectd(&c2::get_bounds_d_from_64(&first));
                            out.rectd(&c2::get_bounds_paths_d_from_64(&paths));
                        }
                    }
                }
                25 => {
                    let (variant, uniform, whole) = (rd.i(), rd.i() != 0, rd.i() != 0);
                    let (mut sx, sy) = (rd.d(), rd.d());
                    // mutation 5: the scale one unit in the last place off
                    if mutation == 5 {
                        sx = f64::from_bits(sx.to_bits() + 1);
                    }
                    let mut ec = 0;
                    let done: Result<(), c2::Clipper2Exception> = (|| {
                        if variant == 0 || variant == 3 {
                            let paths = rd.pathsd();
                            let first = paths.first().cloned().unwrap_or_default();
                            if variant == 0 {
                                if whole {
                                    out.paths64(&if uniform { c2::scale_paths64_from_d_uniform(&paths, sx, &mut ec)? } else { c2::scale_paths64_from_d(&paths, sx, sy, &mut ec)? });
                                } else {
                                    out.path64(&if uniform { c2::scale_path64_from_d_uniform(&first, sx, &mut ec)? } else { c2::scale_path64_from_d(&first, sx, sy, &mut ec)? });
                                }
                            } else if whole {
                                out.pathsd(&if uniform { c2::scale_pathsd_from_d_uniform(&paths, sx, &mut ec)? } else { c2::scale_pathsd_from_d(&paths, sx, sy, &mut ec)? });
                            } else {
                                out.pathd(&if uniform { c2::scale_pathd_from_d_uniform(&first, sx, &mut ec)? } else { c2::scale_pathd_from_d(&first, sx, sy, &mut ec)? });
                            }
                        } else {
                            let paths = rd.paths64();
                            let first = paths.first().cloned().unwrap_or_default();
                            if variant == 1 {
                                if whole {
                                    out.pathsd(&if uniform { c2::scale_pathsd_from_64_uniform(&paths, sx, &mut ec)? } else { c2::scale_pathsd_from_64(&paths, sx, sy, &mut ec)? });
                                } else {
                                    out.pathd(&if uniform { c2::scale_pathd_from_64_uniform(&first, sx, &mut ec)? } else { c2::scale_pathd_from_64(&first, sx, sy, &mut ec)? });
                                }
                            } else if whole {
                                out.paths64(&if uniform { c2::scale_paths64_from_64_uniform(&paths, sx, &mut ec)? } else { c2::scale_paths64_from_64(&paths, sx, sy, &mut ec)? });
                            } else {
                                out.path64(&if uniform { c2::scale_path64_from_64_uniform(&first, sx, &mut ec)? } else { c2::scale_path64_from_64(&first, sx, sy, &mut ec)? });
                            }
                        }
                        Ok(())
                    })();
                    if let Err(e) = done {
                        out.thrown(&e);
                    }
                    out.i(ec as i64);
                }
                26 => {
                    if rd.i() == 0 {
                        let paths = rd.pathsd();
                        out.path64(&c2::transform_path64_from_d(&paths.first().cloned().unwrap_or_default()));
                        out.paths64(&c2::transform_paths64_from_d(&paths));
                    } else {
                        let paths = rd.paths64();
                        out.pathd(&c2::transform_pathd_from_64(&paths.first().cloned().unwrap_or_default()));
                        out.pathsd(&c2::transform_pathsd_from_64(&paths));
                    }
                }
                27 => {
                    let mut precision = rd.i() as i32;
                    let mut ec = rd.i() as i32;
                    if let Err(e) = c2::check_precision_range(&mut precision, &mut ec) {
                        out.thrown(&e);
                    }
                    out.i(precision as i64);
                    out.i(ec as i64);
                    let mut p2 = rd.i() as i32;
                    if let Err(e) = c2::check_precision_range_1(&mut p2) {
                        out.thrown(&e);
                    }
                    out.i(p2 as i64);
                }
                28 => {
                    let (r, r2) = (rd.rectd(), rd.rectd());
                    let (a, px, py) = (rd.d(), rd.d(), rd.d());
                    let q = rd.rect64();
                    out.i(r.is_valid() as i64);
                    out.d(r.width());
                    out.d(r.height());
                    out.pointd(&r.mid_point());
                    out.pathd(&r.as_path());
                    out.i(r.contains_point(&c2::PointD::new(px, py)) as i64);
                    out.i(r.contains(&r2) as i64);
                    out.i(r.is_empty() as i64);
                    out.i(r.intersects(&r2) as i64);
                    out.i((r == r2) as i64);
                    out.rectd(&r.add(&r2));
                    let mut s = r;
                    s.add_assign(&r2);
                    out.rectd(&s);
                    s = r;
                    s.scale(a);
                    out.rectd(&s);
                    s = r;
                    s.set_width(a);
                    s.set_height(px);
                    out.rectd(&s);
                    out.rectd(&c2::RectD::new_valid(true));
                    out.rectd(&c2::RectD::new_valid(false));
                    out.rectd(&c2::RectD::invalid_rect());
                    out.i(c2::RectD::invalid_rect().is_valid() as i64);
                    out.rectd(&c2::INVALID_RECTD);
                    out.pointd(&c2::invalid_point_d());
                    out.rect64(&c2::scale_rect64_from_d(&r, a));
                    out.rectd(&c2::scale_rectd_from_64(&q, a));
                    out.rect64(&c2::scale_rect64_from_64(&q, a));
                    out.rectd(&c2::scale_rectd_from_d(&r, a));
                    let mut qs = q;
                    qs.scale(a);
                    out.rect64(&qs);
                }

                30 => {
                    let (is_sum, is_closed) = (rd.i() != 0, rd.i() != 0);
                    let (pattern, path) = (rd.path64(), rd.path64());
                    out.paths64(&if is_sum { c2::minkowski_sum(&pattern, &path, is_closed) } else { c2::minkowski_diff(&pattern, &path, is_closed) });
                }
                31 => {
                    let (is_sum, is_closed) = (rd.i() != 0, rd.i() != 0);
                    let decimals = rd.i() as i32;
                    let (pattern, path) = (rd.pathd(), rd.pathd());
                    out.pathsd(&if is_sum { c2::minkowski_sum_d(&pattern, &path, is_closed, decimals)? } else { c2::minkowski_diff_d(&pattern, &path, is_closed, decimals)? });
                }

                32 => {
                    let n = rd.i();
                    let list: Vec<i64> = (0..n).map(|_| rd.i()).collect();
                    let listd: Vec<f64> = (0..n).map(|_| rd.d()).collect();
                    match c2::make_path(&list) {
                        Ok(p) => out.path64(&p),
                        Err(e) => out.thrown(&e),
                    }
                    match c2::make_path_d(&listd) {
                        Ok(p) => out.pathd(&p),
                        Err(e) => out.thrown(&e),
                    }
                    if n >= 6 {
                        let arr: [i64; 6] = list[..6].try_into().unwrap();
                        let arrd: [f64; 6] = listd[..6].try_into().unwrap();
                        out.path64(&c2::make_path_array(&arr));
                        out.pathd(&c2::make_path_d_array(&arrd));
                        out.path64(&c2::make_path_z(&arr));
                        out.pathd(&c2::make_path_zd(&arrd));
                    }
                }

                33 => {
                    let (ct, fr, ct2, fr2) = (rd.i(), rd.i(), rd.i(), rd.i());
                    let (use_tree, mode) = (rd.i() != 0, rd.i());
                    let (preserve_collinear, reverse_solution) = (rd.i() != 0, rd.i() != 0);
                    let (subj, open, clip, extra) = (rd.paths64(), rd.paths64(), rd.paths64(), rd.paths64());
                    let mut exec = |c: &mut c2::Clipper64, ct: i64, fr: i64, out: &mut Wr| {
                        if use_tree {
                            let mut tree = c2::PolyTree64::new();
                            let mut opened = c2::Paths64::new();
                            out.i(c.execute_tree_open(clip_type(ct), fill_rule(fr), &mut tree, &mut opened) as i64);
                            out.tree64(&tree);
                            out.paths64(&opened);
                        } else {
                            let (mut closed, mut opened) = (c2::Paths64::new(), c2::Paths64::new());
                            out.i(c.execute_open(clip_type(ct), fill_rule(fr), &mut closed, &mut opened) as i64);
                            out.paths64(&closed);
                            out.paths64(&opened);
                        }
                    };
                    let mut rdc = c2::ReuseableDataContainer64::new();
                    let mut rdc2 = c2::ReuseableDataContainer64::new();
                    rdc.add_paths(&subj, c2::PathType::Subject, false);
                    rdc.add_paths(&open, c2::PathType::Subject, true);
                    if mode & 1 != 0 {
                        rdc2.add_paths(&clip, c2::PathType::Clip, false);
                    } else {
                        rdc.add_paths(&clip, c2::PathType::Clip, false);
                    }
                    let mut c1 = c2::Clipper64::new();
                    c1.set_preserve_collinear(preserve_collinear);
                    c1.set_reverse_solution(reverse_solution);
                    if mode & 2 != 0 {
                        c1.add_subject(&extra);
                    }
                    c1.add_reuseable_data(&rdc);
                    if mode & 1 != 0 {
                        c1.add_reuseable_data(&rdc2);
                    }
                    if mode & 4 != 0 {
                        c1.add_clip(&extra);
                    }
                    // the container grows after the first clipper took it
                    if mode & 8 != 0 {
                        rdc.add_paths(&extra, c2::PathType::Clip, false);
                    }
                    exec(&mut c1, ct, fr, out);
                    let mut c2_ = c2::Clipper64::new();
                    c2_.add_reuseable_data(&rdc);
                    if mode & 1 != 0 {
                        c2_.add_reuseable_data(&rdc2);
                    }
                    exec(&mut c2_, ct2, fr2, out);
                    if mode & 16 != 0 {
                        c1.clear();
                        c1.add_reuseable_data(&rdc);
                        exec(&mut c1, ct2, fr, out);
                    }
                    if mode & 32 != 0 {
                        // the container emptied and filled again, for a third clipper
                        rdc.clear();
                        rdc.add_paths(&clip, c2::PathType::Subject, false);
                        rdc.add_paths(&subj, c2::PathType::Clip, false);
                        let mut c3 = c2::Clipper64::new();
                        c3.add_reuseable_data(&rdc);
                        exec(&mut c3, ct, fr2, out);
                    }
                }

                34 => {
                    let (a, b) = (rd.d(), rd.d());
                    let p64 = rd.paths64();
                    let pd = rd.pathsd();
                    let mut root = c2::PolyTreeD::new();
                    let made: Result<(), c2::Clipper2Exception> = (|| {
                        root.set_scale(a);
                        let c1 = root.add_child(&p64[0])?;
                        let c2_ = c1.add_child_d(&pd[0]);
                        c2_.add_child(&p64[1])?;
                        root.add_child_d(&pd[1]);
                        let c3 = root.child_mut(0).child_mut(0).child_mut(0);
                        c3.set_scale(b);
                        c3.add_child(&p64[0])?;
                        out.i(root.child(0).count() as i64);
                        out.i(root.child(1).count() as i64);
                        Ok(())
                    })();
                    if let Err(e) = made {
                        out.thrown(&e);
                    }
                    out.treed(&root);
                    out.d(root.area());
                    out.d(c2::PolyPathD::clipper2_lib_area(&pd[0]));
                    out.d(c2::PolyPath64::clipper2_lib_area(&p64[0]));
                    out.pathsd(&c2::poly_tree_to_paths_d(&root));
                    out.str(&c2::poly_tree_d_to_string(&root));
                    root.clear();
                    out.i(root.count() as i64);
                }
                35 => {
                    let (ct, fr) = (rd.i(), rd.i());
                    let (subj, clip) = (rd.paths64(), rd.paths64());
                    let mut tree = c2::PolyTree64::new();
                    c2::boolean_op_tree(clip_type(ct), fill_rule(fr), &subj, &clip, &mut tree);
                    out.str(&c2::poly_tree64_to_string(&tree));
                    out.d(tree.area());
                }
                _ => out.i(-1),
            }
            Ok(())
        }

        const HASH_START: u64 = 0xcbf2_9ce4_8422_2325;

        fn mix(h: &mut u64, v: i64) {
            *h = (*h ^ v as u64).wrapping_mul(0x0000_0100_0000_01b3);
        }

        /// The operation on the Rust port: `cz2_ref_*_ext` of clipper2_ref_ext_shim.cpp.
        fn rust_ext(op: i32, input: &common::Enc) -> common::Enc {
            let mut rd = Rd { e: input, ni: 0, nd: 0 };
            let mut out = Wr::default();
            if let Err(e) = rust_run(op, &mut rd, &mut out) {
                out.e.ints.clear();
                out.e.dbls.clear();
                out.i(-777);
                out.i(code_of(&e));
            }
            let mut e = out.e;
            // mutation 6: the last double of the answer one unit in the last place off
            if MUTATION.with(|m| m.get()) == 6 {
                if let Some(v) = e.dbls.last_mut() {
                    *v = f64::from_bits(v.to_bits() ^ 1);
                }
            }
            e
        }

        /// `n` cases (times `CLIPPER2_PARITY_SCALE`) of `gen`, each from its own seed, through
        /// both sides. With `expect_mismatches` (the mutation checks) nothing is printed.
        fn run_set(name: &str, test: u64, n: u64, gen: &dyn Fn(u64, &mut u64) -> (i32, common::Enc), expect_mismatches: bool) -> common::Totals {
            let mut t = common::Totals::default();
            let n = n * common::scale_factor();
            let mut shown = 0;
            // `CLIPPER2_PARITY_TRACE=1` names every case before each side runs it.
            let trace = std::env::var("CLIPPER2_PARITY_TRACE").is_ok();
            for index in 0..n {
                let mut s = common::case_seed(test, index);
                let (op, input) = gen(index, &mut s);
                if trace {
                    eprintln!("{name}#{index}: c++\n{}", common::dump(op, &input));
                }
                let cpp = common::cpp_ext(op, &input, Z);
                if trace {
                    eprintln!("{name}#{index}: rust");
                }
                let rust = match std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| rust_ext(op, &input))) {
                    Ok(r) => r,
                    Err(_) => {
                        t.mismatches += 1;
                        if !expect_mismatches {
                            eprintln!("{name}#{index}: the Rust side panicked\n{}", common::dump(op, &input));
                        }
                        t.cases += 1;
                        continue;
                    }
                };
                t.cases += 1;
                t.out_ints += cpp.ints.len() as u64;
                t.out_dbls += cpp.dbls.len() as u64;
                if cpp.ints.contains(&-777) && cpp.ints.len() == 2 || cpp.ints.windows(2).any(|w| w[0] == -778 && [1, 2, 4, 32, 64, -1].contains(&w[1])) {
                    t.exceptions += 1;
                }
                if cpp.ints.len() + cpp.dbls.len() > 3 {
                    t.with_output += 1;
                }
                if !common::same(&rust, &cpp) {
                    t.mismatches += 1;
                    if !expect_mismatches && shown < 5 {
                        shown += 1;
                        eprintln!("{name}#{index}: {}\n{}", common::first_difference(&rust, &cpp), common::dump(op, &input));
                    }
                }
            }
            if !expect_mismatches {
                eprintln!(
                    "{name}: {} cases, {} differ; {} threw, {} with output, {} ints and {} doubles compared",
                    t.cases, t.mismatches, t.exceptions, t.with_output, t.out_ints, t.out_dbls
                );
            }
            t
        }

        fn finish(what: &str, total: &common::Totals, at_least: u64) {
            eprintln!(
                "{what} (Z = {Z}, reference from the pin: {}): {} cases, {} differ; {} threw, {} with output, {} ints and {} doubles compared",
                common::cpp_from_pin(Z),
                total.cases,
                total.mismatches,
                total.exceptions,
                total.with_output,
                total.out_ints,
                total.out_dbls
            );
            assert!(total.cases >= at_least, "{what}: {} cases, fewer than {at_least}", total.cases);
            assert_eq!(total.mismatches, 0, "{what}: {} of {} cases differ from the C++", total.mismatches, total.cases);
        }
    };
}

/// The tests of clipper.rectclip.cpp and its wrappers of clipper.h.
#[macro_export]
macro_rules! rectclip_parity_tests {
    () => {
        $crate::ext_rust_impl!();

        fn rectclip_sets() -> Vec<(&'static str, u64, u64, Box<dyn Fn(u64, &mut u64) -> (i32, common::Enc)>)> {
            vec![
                ("rect_clip_paths", 1, 40_000, Box::new(|_, s| (1, common::gen_rectclip(1, s)))),
                ("rect_clip_path", 2, 10_000, Box::new(|_, s| (2, common::gen_rectclip(2, s)))),
                ("rect_clip_lines_paths", 3, 30_000, Box::new(|_, s| (3, common::gen_rectclip(3, s)))),
                ("rect_clip_lines_path", 4, 6_000, Box::new(|_, s| (4, common::gen_rectclip(4, s)))),
                ("rect_clip64_twice", 5, 12_000, Box::new(|_, s| (5, common::gen_rectclip(5, s)))),
                ("rect_clip_lines64_twice", 6, 10_000, Box::new(|_, s| (6, common::gen_rectclip(6, s)))),
            ]
        }

        #[test]
        fn rect_clip_matches_the_library() {
            let mut total = common::Totals::default();
            for (name, test, n, gen) in rectclip_sets() {
                total.add(&run_set(name, test, n, &*gen, false));
            }
            finish("RectClip and RectClipLines", &total, 100_000);
            // what the RectClip64s of `rect_clip64_twice` went through
            let stats = RC_STATS.with(|t| *t.borrow());
            for (k, (name, v)) in RC_STAT_NAMES.iter().zip(stats).enumerate() {
                eprintln!("  RectClip64: {name}: {v}");
                // (`RectClip64::Execute` takes a path that is all on the rectangle whole,
                // before `ExecuteInternal`: that arm of it cannot be reached.)
                assert!(v > 0 || k == 0, "no case of rect_clip64_twice reached '{name}'");
            }
            let stats = RCL_STATS.with(|t| *t.borrow());
            for (name, v) in RC_STAT_NAMES.iter().zip(stats) {
                eprintln!("  RectClipLines64: {name}: {v}");
                assert!(v > 0, "no case of rect_clip_lines64_twice reached '{name}'");
            }
        }

        #[test]
        fn rect_clip_d_matches_the_library() {
            let mut total = common::Totals::default();
            total.add(&run_set("rect_clip_d", 7, 12_000, &|_, s| (7, common::gen_rectclip_d(s)), false));
            total.add(&run_set("rect_clip_lines_d", 8, 12_000, &|_, s| (8, common::gen_rectclip_d(s)), false));
            finish("RectClip and RectClipLines on doubles", &total, 24_000);
            assert!(total.exceptions > 0, "no case reached the error path");
        }

        #[test]
        fn rect_clip_helpers_match_the_library() {
            let mut total = common::Totals::default();
            total.add(&run_set("get_segment_intersection", 9, 40_000, &|_, s| (9, common::gen_segment_intersection(s)), false));
            total.add(&run_set("start_locs_are_clockwise", 10, 4_000, &|_, s| (10, common::gen_start_locs(s)), false));
            finish("GetSegmentIntersection and StartLocsAreClockwise", &total, 44_000);
        }

        #[test]
        fn rect_clip_mutation_checks() {
            let checks: [(u32, &str, i32, u64); 4] = [
                (1, "the rectangle one unit wider", 1, 1),
                (2, "the first path from its second vertex", 3, 3),
                (3, "the second segment the other way round", 9, 9),
                (4, "one more decimal", 7, 7),
            ];
            for (mutation, what, op, test) in checks {
                MUTATION.with(|m| m.set(mutation));
                let t = run_set(
                    what,
                    test,
                    2_000,
                    &|_, s| match op {
                        9 => (9, common::gen_segment_intersection(s)),
                        7 => (7, common::gen_rectclip_d(s)),
                        _ => (op, common::gen_rectclip(op, s)),
                    },
                    true,
                );
                MUTATION.with(|m| m.set(0));
                eprintln!("mutation {mutation} ({what}): {} of {} cases differ", t.mismatches, t.cases);
                assert!(t.mismatches > 0, "the mutation '{what}' changed nothing");
            }
        }
    };
}

/// The tests of `ClipperD`, the `PathsD` interface of clipper.h, clipper.minkowski.h,
/// `ReuseableDataContainer64` and the templates of clipper.core.h.
#[macro_export]
macro_rules! d_parity_tests {
    () => {
        $crate::ext_rust_impl!();

        #[test]
        fn clipper_d_matches_the_library() {
            let mut total = common::Totals::default();
            total.add(&run_set("clipper_d", 20, 16_000, &|_, s| (20, common::gen_clipper_d(s)), false));
            total.add(&run_set("boolean_op_d", 21, 8_000, &|_, s| (21, common::gen_boolean_d(s)), false));
            total.add(&run_set("polypath_d", 34, 1_000, &|_, s| (34, common::gen_polypath_d(s)), false));
            total.add(&run_set("polytree64_stream", 35, 1_000, &|_, s| (35, common::gen_tree_string(s)), false));
            finish("ClipperD, PolyPathD and BooleanOp on doubles", &total, 26_000);
            let calls = ZCB_CALLS.with(|n| n.get());
            eprintln!("  ZCallbackD calls: {calls}");
            assert!(!Z || calls > 0, "the ZCallbackD was never called");
            assert!(total.exceptions > 0, "no case reached the error path");
        }

        #[test]
        fn paths_d_helpers_match_the_library() {
            let mut total = common::Totals::default();
            total.add(&run_set("inflate_paths_d", 22, 4_000, &|_, s| (22, common::gen_inflate_d(s)), false));
            for which in 0..15u64 {
                total.add(&run_set(&format!("path_d_{which}"), 2300 + which, 1_200, &move |_, s| (23, common::gen_path_d(which as i64, s)), false));
            }
            total.add(&run_set("path_64", 24, 4_000, &|_, s| (24, common::gen_path_64(s)), false));
            total.add(&run_set("minkowski", 30, 2_500, &|_, s| (30, common::gen_minkowski(30, s)), false));
            total.add(&run_set("minkowski_d", 31, 2_500, &|_, s| (31, common::gen_minkowski(31, s)), false));
            total.add(&run_set("make_path", 32, 1_500, &|_, s| (32, common::gen_make_path(s)), false));
            finish("the PathsD helpers of clipper.h and clipper.minkowski.h", &total, 32_000);
            assert!(total.exceptions > 0, "no case reached the error path");
        }

        #[test]
        fn core_templates_match_the_library() {
            let mut total = common::Totals::default();
            total.add(&run_set("scale_path", 25, 8_000, &|_, s| (25, common::gen_scale(s)), false));
            total.add(&run_set("transform_path", 26, 1_500, &|_, s| (26, common::gen_transform(s)), false));
            total.add(&run_set("check_precision_range", 27, 1_000, &|_, s| (27, common::gen_check_precision(s)), false));
            total.add(&run_set("rect_d", 28, 3_000, &|_, s| (28, common::gen_rect_d(s)), false));
            finish("ScalePath, ScalePaths, TransformPath, CheckPrecisionRange, Rect", &total, 13_500);
            assert!(total.exceptions > 0, "no case reached the error path");
        }

        #[test]
        fn reuseable_data_matches_the_library() {
            let mut total = common::Totals::default();
            total.add(&run_set("reuseable_data", 33, 8_000, &|_, s| (33, common::gen_reuseable(s)), false));
            finish("ReuseableDataContainer64", &total, 8_000);
        }

        #[test]
        fn d_mutation_checks() {
            let checks: [(u32, &str, i32, u64); 4] = [
                (4, "one more decimal (ClipperD)", 20, 20),
                (4, "one more decimal (BooleanOp)", 21, 21),
                (5, "the scale one unit in the last place off", 25, 25),
                (6, "the last double one unit in the last place off", 20, 20),
            ];
            for (mutation, what, op, test) in checks {
                MUTATION.with(|m| m.set(mutation));
                let t = run_set(
                    what,
                    test,
                    2_000,
                    &|_, s| match op {
                        20 => (20, common::gen_clipper_d(s)),
                        21 => (21, common::gen_boolean_d(s)),
                        _ => (25, common::gen_scale(s)),
                    },
                    true,
                );
                MUTATION.with(|m| m.set(0));
                eprintln!("mutation {mutation} ({what}): {} of {} cases differ", t.mismatches, t.cases);
                assert!(t.mismatches > 0, "the mutation '{what}' changed nothing");
            }
        }
    };
}
