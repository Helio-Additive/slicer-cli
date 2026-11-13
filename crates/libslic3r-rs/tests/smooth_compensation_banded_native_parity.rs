//! `smooth_compensation_banded` (ElephantFootCompensation.cpp:465-532) in Rust against the same
//! function compiled as C++ in `clipper-z-sys` (`cz_smooth_compensation_banded`), bit for bit.
//! Contours have short and long edges around the band so that both branches of each walk, the
//! interpolation and the wrap of the walk around the whole contour are taken.

use slicer::elephant_foot_compensation::smooth_compensation_banded;
use slicer::geometry::Point;

fn xorshift(s: &mut u64) -> u64 {
    *s ^= *s << 13;
    *s ^= *s >> 7;
    *s ^= *s << 17;
    *s
}

#[test]
fn smooth_compensation_banded_matches_cpp() {
    let mut s: u64 = 0xD1B5_4A32_D192_ED03;
    let mut cases = 0usize;
    let mut interpolated = 0usize;
    for round in 0..40_000 {
        let n = 3 + (xorshift(&mut s) % 60) as usize;
        let band = [20_000f32, 40_000.0, 77_777.0, 5_000.0][round % 4];
        let step = [band as f64 * 0.05, band as f64 * 0.3, band as f64 * 0.9, band as f64 * 3.0][(xorshift(&mut s) % 4) as usize];
        let (mut x, mut y) = ((xorshift(&mut s) % 20_000_000) as f64, (xorshift(&mut s) % 20_000_000) as f64);
        let mut contour: Vec<Point> = Vec::with_capacity(n);
        for _ in 0..n {
            let a = (xorshift(&mut s) % 10_000) as f64 / 10_000.0 * std::f64::consts::TAU;
            let l = step * (0.2 + (xorshift(&mut s) % 1000) as f64 / 500.0);
            x += l * a.cos();
            y += l * a.sin();
            if xorshift(&mut s) % 23 == 0 && !contour.is_empty() {
                // a repeated point: an edge of no length
                let p = *contour.last().unwrap();
                contour.push(p);
            } else {
                contour.push(Point::new(x as i64, y as i64));
            }
        }
        let comp: Vec<f32> = (0..n).map(|_| -((xorshift(&mut s) % 30_000) as f32) * 0.37).collect();
        let strength = [0.8f32, 0.5, 1.0, 0.13][(xorshift(&mut s) % 4) as usize];
        let iters = 1 + (xorshift(&mut s) % 12) as usize;

        let xy: Vec<i32> = contour.iter().flat_map(|p| [p.x as i32, p.y as i32]).collect();
        let mut want = comp.clone();
        unsafe {
            clipper_z_sys::cz_smooth_compensation_banded(xy.as_ptr(), n as i32, want.as_mut_ptr(), band, strength, iters as i32);
        }
        let mut got = comp.clone();
        smooth_compensation_banded(&contour, band, &mut got, strength, iters);
        let w: Vec<u32> = want.iter().map(|v| v.to_bits()).collect();
        let g: Vec<u32> = got.iter().map(|v| v.to_bits()).collect();
        assert_eq!(g, w, "case {round}: n {n} band {band} strength {strength} iters {iters}");
        cases += 1;
        if got != comp {
            interpolated += 1;
        }
    }
    assert!(interpolated > cases / 2, "the cases do not exercise the solver: {interpolated} of {cases}");
}
