//! The free functions of ClipperLib ported in `src/clipper.rs` (`Area`, `PointInPolygon`,
//! `CleanPolygon`, clipper.cpp:177-257 and :3854-3981) against the library itself, compiled in
//! `clipper-z-sys`. The inputs put the float decisions on their boundaries: points on and next to
//! edges, near-duplicate vertices, nearly collinear runs, at four magnitudes up to 1e9.

use slicer::clipper::{area, clean_polygon, point_in_polygon, IntPoint, Path};

fn xorshift(s: &mut u64) -> u64 {
    *s ^= *s << 13;
    *s ^= *s >> 7;
    *s ^= *s << 17;
    *s
}

fn r(m: i64, x: u64) -> i32 {
    ((x % (2 * m as u64 + 1)) as i64 - m) as i32
}

fn polygon(s: &mut u64, scale: i64) -> Path {
    let n = 3 + (xorshift(s) % 12) as usize;
    let mut poly = Path::new();
    let (mut x, mut y) = (r(scale, xorshift(s)), r(scale, xorshift(s)));
    let (dx, dy) = (r(scale / 8 + 3, xorshift(s)), r(scale / 8 + 3, xorshift(s)));
    for _ in 0..n {
        match xorshift(s) % 5 {
            0 => {
                x = x.wrapping_add(r(2, xorshift(s)));
                y = y.wrapping_add(r(2, xorshift(s)));
            }
            1 | 2 => {
                x = x.saturating_add(dx).clamp(-1_000_000_000, 1_000_000_000) + r(1, xorshift(s));
                y = y.saturating_add(dy).clamp(-1_000_000_000, 1_000_000_000) + r(1, xorshift(s));
            }
            _ => {
                x = r(scale, xorshift(s));
                y = r(scale, xorshift(s));
            }
        }
        poly.push(IntPoint::new(x, y));
    }
    poly
}

#[test]
fn area_point_in_polygon_and_clean_polygon() {
    let mut s: u64 = 0x9E37_79B9_7F4A_7C15;
    for round in 0..120_000 {
        let scale = [2_000i64, 400_000, 60_000_000, 900_000_000][round % 4];
        let poly = polygon(&mut s, scale);
        let xy: Vec<i32> = poly.iter().flat_map(|p| [p.x, p.y]).collect();
        let n = poly.len() as i32;

        let want = unsafe { clipper_z_sys::cz_ref_area(xy.as_ptr(), n) };
        assert_eq!(area(&poly).to_bits(), want.to_bits(), "area of {poly:?}");

        for _ in 0..6 {
            let i = (xorshift(&mut s) % poly.len() as u64) as usize;
            let (a, b) = (poly[i], poly[(i + 1) % poly.len()]);
            let t = (xorshift(&mut s) % 1001) as f64 / 1000.0;
            let pt = IntPoint::new(
                (a.x as f64 + (b.x as f64 - a.x as f64) * t).round() as i32 + r(1, xorshift(&mut s)),
                (a.y as f64 + (b.y as f64 - a.y as f64) * t).round() as i32 + r(1, xorshift(&mut s)),
            );
            let want = unsafe { clipper_z_sys::cz_ref_point_in_polygon(pt.x, pt.y, xy.as_ptr(), n) };
            assert_eq!(point_in_polygon(&pt, &poly), want, "{pt:?} in {poly:?}");
        }

        let distance = [1.415f64, 0.5, 3.0, 40.0][(xorshift(&mut s) % 4) as usize];
        let mut out = vec![0i32; xy.len()];
        let m = unsafe { clipper_z_sys::cz_ref_clean_polygon(xy.as_ptr(), n, distance, out.as_mut_ptr()) } as usize;
        let want: Path = (0..m).map(|i| IntPoint::new(out[2 * i], out[2 * i + 1])).collect();
        let mut got = Path::new();
        clean_polygon(&poly, &mut got, distance);
        assert_eq!(got, want, "clean_polygon({poly:?}, {distance})");
    }
}

#[test]
fn degenerate_inputs() {
    let empty = Path::new();
    assert_eq!(area(&empty), 0.0);
    assert_eq!(point_in_polygon(&IntPoint::new(0, 0), &empty), 0);
    let mut out = vec![IntPoint::new(1, 1)];
    clean_polygon(&empty, &mut out, 1.415);
    assert!(out.is_empty());
    let two = vec![IntPoint::new(0, 0), IntPoint::new(5, 5)];
    clean_polygon(&two, &mut out, 1.415);
    assert!(out.is_empty());
}
