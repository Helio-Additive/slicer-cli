//! `MultiPoint::_douglas_peucker` (MultiPoint.cpp:179-230) with `line_alg::distance_to_squared`
//! (Line.hpp:43-68): the Rust port against the C++, compiled with Eigen in `eigen-transform-sys`.
//!
//! The inputs are made so that rounding decides: families of points at exactly the same distance
//! from the anchor-floater line (a point and its translates along the line), near-duplicate points
//! one unit apart, and tolerances on both sides of the distances. Coordinates stay within the range
//! of scaled coordinates (a bed of 400 mm is 4e7 units).

use slicer::geometry::Point;

fn xorshift(s: &mut u64) -> u64 {
    *s ^= *s << 13;
    *s ^= *s >> 7;
    *s ^= *s << 17;
    *s
}

fn r(m: i64, x: u64) -> i64 {
    (x % (2 * m as u64 + 1)) as i64 - m
}

fn check(pts: &[(i64, i64)], tol: f64) {
    let xy: Vec<i64> = pts.iter().flat_map(|p| [p.0, p.1]).collect();
    let keep = eigen_transform_sys::douglas_peucker_keep(&xy, tol);
    let want: Vec<(i64, i64)> =
        pts.iter().zip(keep.iter()).filter(|(_, k)| **k != 0).map(|(p, _)| *p).collect();
    let points: Vec<Point> = pts.iter().map(|p| Point::new(p.0, p.1)).collect();
    let got: Vec<(i64, i64)> = slicer::multi_point::douglas_peucker_faithful(&points, tol)
        .iter()
        .map(|p| (p.x, p.y))
        .collect();
    assert_eq!(got, want, "points {pts:?} tolerance {tol}");
    // The function named after the C++ one (with DP_TRUE_DISTANCE on, the default).
    let got: Vec<(i64, i64)> =
        slicer::multi_point::douglas_peucker(&points, tol).iter().map(|p| (p.x, p.y)).collect();
    assert_eq!(got, want, "multi_point::douglas_peucker, points {pts:?} tolerance {tol}");
}

#[test]
fn equal_distance_families() {
    let mut s: u64 = 0x9E37_79B9_7F4A_7C15;
    for _ in 0..200_000 {
        let a = (r(20_000_000, xorshift(&mut s)), r(20_000_000, xorshift(&mut s)));
        let (ux, uy) = (r(900, xorshift(&mut s)), r(900, xorshift(&mut s)));
        let g = 200 + (xorshift(&mut s) % 20_000) as i64;
        let b = (a.0 + ux * g, a.1 + uy * g);
        let mut pts = vec![a];
        let base = (
            a.0 + ux * (g / 7) + r(5_000, xorshift(&mut s)),
            a.1 + uy * (g / 7) + r(5_000, xorshift(&mut s)),
        );
        let n = 2 + (xorshift(&mut s) % 4) as i64;
        for j in 0..n {
            let k = j * (g / 8).max(1);
            pts.push((base.0 + ux * k, base.1 + uy * k));
        }
        pts.push(b);
        check(&pts, [0.5f64, 30.0, 2_000.0][(xorshift(&mut s) % 3) as usize]);
    }
}

#[test]
fn near_duplicates_and_random_polylines() {
    let mut s: u64 = 0xD1B5_4A32_D192_ED03;
    for _ in 0..200_000 {
        let a = (r(40_000_000, xorshift(&mut s)), r(40_000_000, xorshift(&mut s)));
        let b = (a.0 + r(3_000_000, xorshift(&mut s)), a.1 + r(3_000_000, xorshift(&mut s)));
        let n = 3 + (xorshift(&mut s) % 6) as usize;
        let mut pts = vec![a];
        for j in 0..n {
            let t = (j as f64 + 1.0) / (n as f64 + 1.0);
            let off = [3i64, 40, 700, 20_000][(xorshift(&mut s) % 4) as usize];
            let px = a.0 + ((b.0 - a.0) as f64 * t) as i64 + r(off, xorshift(&mut s));
            let py = a.1 + ((b.1 - a.1) as f64 * t) as i64 + r(off, xorshift(&mut s));
            pts.push((px, py));
            if xorshift(&mut s) % 4 != 0 {
                pts.push((px + r(1, xorshift(&mut s)), py + r(1, xorshift(&mut s))));
            }
        }
        pts.push(b);
        let tol = [0.5f64, 2.0, 30.0, 500.0, 30_000.0][(xorshift(&mut s) % 5) as usize]
            * ((xorshift(&mut s) % 1000) as f64 / 500.0);
        check(&pts, tol);
    }
}

#[test]
fn degenerate_inputs() {
    check(&[(5, 5)], 10.0);
    check(&[(5, 5), (5, 5)], 10.0);
    check(&[(0, 0), (10, 0), (0, 0)], 1.0);
    check(&[(0, 0), (0, 0), (0, 0), (7, 7)], 0.0);
    check(&[(0, 0), (5, 1), (10, 0), (15, -1), (20, 0)], 0.999);
}
