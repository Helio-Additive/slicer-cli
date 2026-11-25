//! Geometry.hpp / Geometry.cpp free functions under their C++ names (geometry/geometry.rs) against the pin:
//! directions_parallel, directions_perpendicular (Geometry.cpp:28-40), rad2deg_dir (52-57), linint (72-75),
//! segments_intersect (Geometry.hpp:117-167), foot_pt x2 (169-180), ray_point_distance_squared x2,
//! ray_point_distance x2 (182-200), liang_barsky_line_clipping_interval (202-252),
//! liang_barsky_line_clipping x2 (254-288), is_rotation_ninety_degrees x2 (512-524). Vectors from
//! `tests/data/geometry_cpp_names/gen.cpp` (16 groups of 3,000 cases, gzip -9n); results bit-equal.

use slicer::bounding_box::BoundingBoxf;
use slicer::geometry::geometry::{
    directions_parallel, directions_perpendicular, foot_pt, liang_barsky_line_clipping,
    liang_barsky_line_clipping_interval, linint, rad2deg_dir, ray_point_distance, ray_point_distance_squared,
    segments_intersect, GeometryLinePoint, IsRotationNinetyDegrees, LiangBarskyLineClippingSrc, Vec3d,
};
use slicer::geometry::{Line, Point, PointF};
use std::io::Read;

fn hexf(s: &str) -> f64 {
    let (neg, s) = match s.strip_prefix('-') {
        Some(r) => (true, r),
        None => (false, s),
    };
    let v = if s == "inf" {
        f64::INFINITY
    } else if s == "nan" {
        f64::NAN
    } else if s == "0x0p+0" {
        0.0
    } else {
        let s = s.strip_prefix("0x").expect("hex float");
        let (mant, exp) = s.split_once('p').expect("exponent");
        let exp: i32 = exp.parse().expect("exp");
        let (int_part, frac_part) = mant.split_once('.').unwrap_or((mant, ""));
        let digits = format!("{int_part}{frac_part}");
        let m = u64::from_str_radix(&digits, 16).expect("mantissa");
        (m as f64) * 2f64.powi(exp - 4 * frac_part.len() as i32)
    };
    if neg {
        -v
    } else {
        v
    }
}

fn same(a: f64, b: f64) -> bool {
    a.to_bits() == b.to_bits() || (a.is_nan() && b.is_nan())
}

#[test]
fn geometry_free_functions_match_the_pin() {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/geometry_cpp_names/vectors.txt.gz");
    let mut text = String::new();
    flate2::read::GzDecoder::new(std::fs::File::open(path).unwrap()).read_to_string(&mut text).unwrap();
    let mut counts = std::collections::BTreeMap::<&str, (usize, usize)>::new();
    let mut tally = |name: &'static str, ok: bool| {
        let e = counts.entry(name).or_insert((0, 0));
        e.0 += 1;
        if !ok {
            e.1 += 1;
        }
    };
    for line in text.lines() {
        let t: Vec<&str> = line.split_whitespace().collect();
        let f = |i: usize| hexf(t[i]);
        let n = |i: usize| t[i].parse::<i64>().unwrap();
        let b = |i: usize| t[i] == "1";
        match t[0] {
            "ANG" => {
                tally("directions_parallel", directions_parallel(f(1), f(2), f(3)) == b(4));
                tally("directions_perpendicular", directions_perpendicular(f(1), f(2), f(3)) == b(5));
                tally("rad2deg_dir", same(rad2deg_dir(f(6)), f(7)));
            }
            "LIN" => tally("linint", same(linint(f(1), f(2), f(3), f(4), f(5)), f(6))),
            "SEG" => {
                let p = |i: usize| Point::new(n(i), n(i + 1));
                tally("segments_intersect", segments_intersect(&p(1), &p(3), &p(5), &p(7)) == b(9));
            }
            "FPD" => {
                let v = |i: usize| PointF::new(f(i), f(i + 1));
                let fp = foot_pt(&v(1), &v(3), &v(5));
                tally("foot_pt(Vec2d)", same(fp.x, f(7)) && same(fp.y, f(8)));
                tally("ray_point_distance_squared(Vec2d)", same(ray_point_distance_squared(&v(1), &v(3), &v(5)), f(9)));
                tally("ray_point_distance(Vec2d)", same(ray_point_distance(&v(1), &v(3), &v(5)), f(10)));
            }
            "FPL" => {
                let ln = Line::new(Point::new(n(1), n(2)), Point::new(n(3), n(4)));
                let ipt = Point::new(n(5), n(6));
                let fp = GeometryLinePoint::foot_pt(&ln, &ipt);
                tally("foot_pt(Line)", same(fp.x, f(7)) && same(fp.y, f(8)));
                tally("ray_point_distance_squared(Line)", same(GeometryLinePoint::ray_point_distance_squared(&ln, &ipt), f(9)));
                tally("ray_point_distance(Line)", same(GeometryLinePoint::ray_point_distance(&ln, &ipt), f(10)));
            }
            "LB" => {
                let bb = BoundingBoxf::new_from_points(PointF::new(f(1), f(2)), PointF::new(f(3), f(4)));
                let x0 = PointF::new(f(5), f(6));
                let x1 = PointF::new(f(7), f(8));
                let mut iv = (-1.0, -1.0);
                let ok_i = liang_barsky_line_clipping_interval(&x0, &PointF::new(x1.x - x0.x, x1.y - x0.y), &bb, &mut iv);
                tally(
                    "liang_barsky_line_clipping_interval",
                    ok_i == b(9) && same(iv.0, f(10)) && same(iv.1, f(11)),
                );
                let (mut c0, mut c1) = (PointF::new(0.0, 0.0), PointF::new(0.0, 0.0));
                let ok_c = (x0, x1).liang_barsky_line_clipping(&bb, &mut c0, &mut c1);
                tally(
                    "liang_barsky_line_clipping(src, clip)",
                    ok_c == b(12) && (!ok_c || (same(c0.x, f(13)) && same(c0.y, f(14)) && same(c1.x, f(15)) && same(c1.y, f(16)))),
                );
                let (mut m0, mut m1) = (x0, x1);
                let ok_m = liang_barsky_line_clipping(&mut m0, &mut m1, &bb);
                tally(
                    "liang_barsky_line_clipping(in place)",
                    ok_m == b(17) && same(m0.x, f(18)) && same(m0.y, f(19)) && same(m1.x, f(20)) && same(m1.y, f(21)),
                );
            }
            "ROT" => {
                let r = Vec3d::new(f(1), f(2), f(3));
                tally("is_rotation_ninety_degrees(double)", f(1).is_rotation_ninety_degrees() == b(4));
                tally("is_rotation_ninety_degrees(Vec3d)", r.is_rotation_ninety_degrees() == b(5));
            }
            _ => panic!("bad line {line}"),
        }
    }
    let mut bad = 0;
    for (k, (c, m)) in &counts {
        eprintln!("{k}: {c} cases, {m} mismatches");
        bad += m;
    }
    assert_eq!(counts.len(), 16);
    assert_eq!(bad, 0);
}
