//! `project_line_on_line` (MultiMaterialSegmentation.cpp:493-514): the Rust port against the C++,
//! compiled with Eigen in `eigen-transform-sys`. The endpoints are truncated to coordinates, so a
//! last-bit difference in `t * v` shows as a unit.

use slicer::geometry::{Line, Point};

fn xorshift(s: &mut u64) -> u64 {
    *s ^= *s << 13;
    *s ^= *s >> 7;
    *s ^= *s << 17;
    *s
}

fn r(m: i64, x: u64) -> i64 {
    (x % (2 * m as u64 + 1)) as i64 - m
}

fn check(pa: (i64, i64), pb: (i64, i64), qa: (i64, i64), qb: (i64, i64)) {
    let want = eigen_transform_sys::project_line_on_line(pa, pb, qa, qb);
    let mut out = Line::default();
    let ok = slicer::multi_material_segmentation::project_line_on_line(
        &Line::new(Point::new(pa.0, pa.1), Point::new(pb.0, pb.1)),
        &Line::new(Point::new(qa.0, qa.1), Point::new(qb.0, qb.1)),
        &mut out,
    );
    let got = if ok { Some([out.a.x, out.a.y, out.b.x, out.b.y]) } else { None };
    assert_eq!(got, want, "projection {pa:?}-{pb:?}, projected {qa:?}-{qb:?}");
}

#[test]
fn random_lines_at_three_magnitudes() {
    let mut s: u64 = 0x9E37_79B9_7F4A_7C15;
    for scale in [40_000_000i64, 400_000_000, 3_000_000_000] {
        for _ in 0..300_000 {
            let pa = (r(scale, xorshift(&mut s)), r(scale, xorshift(&mut s)));
            let len = [50i64, 2_000, 100_000, 5_000_000][(xorshift(&mut s) % 4) as usize];
            let pb = (pa.0 + r(len, xorshift(&mut s)), pa.1 + r(len, xorshift(&mut s)));
            let qa = (pa.0 + r(len, xorshift(&mut s)), pa.1 + r(len, xorshift(&mut s)));
            let qb = (
                pb.0 + r(len / 10 + 2, xorshift(&mut s)),
                pb.1 + r(len / 10 + 2, xorshift(&mut s)),
            );
            check(pa, pb, qa, qb);
        }
    }
}

#[test]
fn degenerate_and_clamped() {
    check((0, 0), (0, 0), (5, 5), (9, 9));
    check((0, 0), (100, 0), (-50, 7), (500, -7));
    check((0, 0), (100, 0), (50, 0), (50, 0));
    check((10, 10), (-90, 35), (10, 10), (-90, 35));
}
