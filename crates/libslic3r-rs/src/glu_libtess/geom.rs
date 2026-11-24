//! Port of `src/glu-libtess/src/geom.c` / `geom.h`.
//!
//! The library is C compiled by clang with `-ffp-contract=on`: a product added
//! to or subtracted from another term in one expression is fused
//! (`a*b + c` = `fma(a, b, c)`, `a*b + c*d` = `fma(a, b, c*d)`).

use super::mesh::GluVertex;

/// The (s, t) of a vertex: geom.c reads nothing else.
#[derive(Clone, Copy, Debug, Default)]
pub struct St {
    pub s: f64,
    pub t: f64,
}

impl From<&GluVertex> for St {
    fn from(v: &GluVertex) -> Self {
        St { s: v.s, t: v.t }
    }
}

/// geom.h `VertEq`.
#[inline]
pub fn vert_eq(u: St, v: St) -> bool {
    u.s == v.s && u.t == v.t
}

/// geom.h `VertLeq`.
#[inline]
pub fn vert_leq(u: St, v: St) -> bool {
    (u.s < v.s) || (u.s == v.s && u.t <= v.t)
}

/// geom.h `TransLeq`.
#[inline]
pub fn trans_leq(u: St, v: St) -> bool {
    (u.t < v.t) || (u.t == v.t && u.s <= v.s)
}

/// geom.h `ABS` + `VertL1dist`.
#[inline]
pub fn vert_l1dist(u: St, v: St) -> f64 {
    let a = u.s - v.s;
    let b = u.t - v.t;
    (if a < 0.0 { -a } else { a }) + (if b < 0.0 { -b } else { b })
}

/// `__gl_vertLeq` (geom.c:41-45).
pub fn gl_vert_leq(u: St, v: St) -> bool {
    vert_leq(u, v)
}

/// `__gl_edgeEval` (geom.c:59-78).
pub fn edge_eval(u: St, v: St, w: St) -> f64 {
    let gap_l = v.s - u.s;
    let gap_r = w.s - v.s;
    if gap_l + gap_r > 0.0 {
        if gap_l < gap_r {
            // (v->t - u->t) + (u->t - w->t) * (gapL / (gapL + gapR))
            return (u.t - w.t).mul_add(gap_l / (gap_l + gap_r), v.t - u.t);
        } else {
            return (w.t - u.t).mul_add(gap_r / (gap_l + gap_r), v.t - w.t);
        }
    }
    // vertical line
    0.0
}

/// `__gl_edgeSign` (geom.c:80-97).
pub fn edge_sign(u: St, v: St, w: St) -> f64 {
    let gap_l = v.s - u.s;
    let gap_r = w.s - v.s;
    if gap_l + gap_r > 0.0 {
        // (v->t - w->t) * gapL + (v->t - u->t) * gapR
        return (v.t - w.t).mul_add(gap_l, (v.t - u.t) * gap_r);
    }
    // vertical line
    0.0
}

/// `__gl_transEval` (geom.c:104-123).
pub fn trans_eval(u: St, v: St, w: St) -> f64 {
    let gap_l = v.t - u.t;
    let gap_r = w.t - v.t;
    if gap_l + gap_r > 0.0 {
        if gap_l < gap_r {
            return (u.s - w.s).mul_add(gap_l / (gap_l + gap_r), v.s - u.s);
        } else {
            return (w.s - u.s).mul_add(gap_r / (gap_l + gap_r), v.s - w.s);
        }
    }
    // vertical line
    0.0
}

/// `__gl_transSign` (geom.c:125-142).
pub fn trans_sign(u: St, v: St, w: St) -> f64 {
    let gap_l = v.t - u.t;
    let gap_r = w.t - v.t;
    if gap_l + gap_r > 0.0 {
        return (v.s - w.s).mul_add(gap_l, (v.s - u.s) * gap_r);
    }
    // vertical line
    0.0
}

/// `__gl_vertCCW` (geom.c:145-155).
pub fn vert_ccw(u: St, v: St, w: St) -> bool {
    // u->s*(v->t - w->t) + v->s*(w->t - u->t) + w->s*(u->t - v->t)
    let a = u.s.mul_add(v.t - w.t, v.s * (w.t - u.t));
    w.s.mul_add(u.t - v.t, a) >= 0.0
}

/// geom.c `RealInterpolate` / `Interpolate` (geom.c:162-166): a and b are
/// clamped to non-negative values first.
#[inline]
fn interpolate(a: f64, x: f64, b: f64, y: f64) -> f64 {
    let a = if a < 0.0 { 0.0 } else { a };
    let b = if b < 0.0 { 0.0 } else { b };
    if a <= b {
        if b == 0.0 {
            (x + y) / 2.0
        } else {
            (y - x).mul_add(a / (a + b), x)
        }
    } else {
        (x - y).mul_add(b / (a + b), y)
    }
}

/// `__gl_edgeIntersect` (geom.c:186-264): returns the intersection's (s, t).
pub fn edge_intersect(o1: St, d1: St, o2: St, d2: St) -> St {
    let (mut o1, mut d1, mut o2, mut d2) = (o1, d1, o2, d2);
    let mut v = St::default();
    let (mut z1, mut z2);
    if !vert_leq(o1, d1) {
        std::mem::swap(&mut o1, &mut d1);
    }
    if !vert_leq(o2, d2) {
        std::mem::swap(&mut o2, &mut d2);
    }
    if !vert_leq(o1, o2) {
        std::mem::swap(&mut o1, &mut o2);
        std::mem::swap(&mut d1, &mut d2);
    }
    if !vert_leq(o2, d1) {
        // Technically, no intersection -- do our best
        v.s = (o2.s + d1.s) / 2.0;
    } else if vert_leq(d1, d2) {
        // Interpolate between o2 and d1
        z1 = edge_eval(o1, o2, d1);
        z2 = edge_eval(o2, d1, d2);
        if z1 + z2 < 0.0 {
            z1 = -z1;
            z2 = -z2;
        }
        v.s = interpolate(z1, o2.s, z2, d1.s);
    } else {
        // Interpolate between o2 and d2
        z1 = edge_sign(o1, o2, d1);
        z2 = -edge_sign(o1, d2, d1);
        if z1 + z2 < 0.0 {
            z1 = -z1;
            z2 = -z2;
        }
        v.s = interpolate(z1, o2.s, z2, d2.s);
    }

    // Now repeat the process for t
    if !trans_leq(o1, d1) {
        std::mem::swap(&mut o1, &mut d1);
    }
    if !trans_leq(o2, d2) {
        std::mem::swap(&mut o2, &mut d2);
    }
    if !trans_leq(o1, o2) {
        std::mem::swap(&mut o1, &mut o2);
        std::mem::swap(&mut d1, &mut d2);
    }
    if !trans_leq(o2, d1) {
        // Technically, no intersection -- do our best
        v.t = (o2.t + d1.t) / 2.0;
    } else if trans_leq(d1, d2) {
        // Interpolate between o2 and d1
        z1 = trans_eval(o1, o2, d1);
        z2 = trans_eval(o2, d1, d2);
        if z1 + z2 < 0.0 {
            z1 = -z1;
            z2 = -z2;
        }
        v.t = interpolate(z1, o2.t, z2, d1.t);
    } else {
        // Interpolate between o2 and d2
        z1 = trans_sign(o1, o2, d1);
        z2 = -trans_sign(o1, d2, d1);
        if z1 + z2 < 0.0 {
            z1 = -z1;
            z2 = -z2;
        }
        v.t = interpolate(z1, o2.t, z2, d2.t);
    }
    v
}
