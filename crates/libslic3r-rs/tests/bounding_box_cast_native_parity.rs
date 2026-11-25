//! Native parity (R951) for `cast<Tout>` of a bounding box (BoundingBox.hpp:265-277): BoundingBox3 to
//! doubles, BoundingBoxf3 to coord_t (truncating) and the `BoundingBoxBase<Vec3crd>` instance (x/y
//! `defined` only), on random corners, some with `min >= max` on an axis.
//! Vectors: tests/data/bounding_box_cpp_names/gen_cast.cpp (cast_vectors.txt.gz).

use slicer::bounding_box::{BoundingBox3, BoundingBoxCast, BoundingBoxCastBase, BoundingBoxf3};
use slicer::geometry::{Point3, Vec3d};

fn hexf(s: &str) -> f64 {
    let (neg, s) = match s.strip_prefix('-') {
        Some(r) => (true, r),
        None => (false, s),
    };
    if s == "0x0p+0" {
        return if neg { -0.0 } else { 0.0 };
    }
    let s = s.strip_prefix("0x").expect("hex float");
    let (mant, exp) = s.split_once('p').expect("exponent");
    let exp: i32 = exp.parse().expect("exp");
    let (int_part, frac_part) = mant.split_once('.').unwrap_or((mant, ""));
    let digits = format!("{int_part}{frac_part}");
    let m = u64::from_str_radix(&digits, 16).expect("mantissa");
    let v = (m as f64) * 2f64.powi(exp - 4 * frac_part.len() as i32);
    if neg {
        -v
    } else {
        v
    }
}

#[test]
fn bounding_box_cast_native_parity() {
    let text = {
        use std::io::Read;
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/bounding_box_cpp_names/cast_vectors.txt.gz");
        let mut s = String::new();
        flate2::read::GzDecoder::new(std::fs::File::open(path).expect("cast_vectors.txt.gz")).read_to_string(&mut s).expect("gzip");
        s
    };
    let mut it = text.lines();
    let (mut cases, mut bad) = (0usize, [0usize; 5]);
    while let Some(l) = it.next() {
        let i: Vec<i64> = l.split_whitespace().skip(1).map(|v| v.parse().unwrap()).collect();
        let ci: Vec<&str> = it.next().unwrap().split_whitespace().skip(1).collect();
        let d: Vec<&str> = it.next().unwrap().split_whitespace().skip(1).collect();
        let cf: Vec<i64> = it.next().unwrap().split_whitespace().skip(1).map(|v| v.parse().unwrap()).collect();
        let cb: Vec<&str> = it.next().unwrap().split_whitespace().skip(1).collect();
        let bi = BoundingBox3::new_from_points(Point3::new(i[0], i[1], i[2]), Point3::new(i[3], i[4], i[5]));
        if bi.defined as i64 != i[6] {
            bad[0] += 1;
        }
        let r: BoundingBoxf3 = BoundingBoxCast::<BoundingBoxf3>::cast(&bi);
        let want: Vec<f64> = ci[..6].iter().map(|v| hexf(v)).collect();
        let got = [r.min.x, r.min.y, r.min.z, r.max.x, r.max.y, r.max.z];
        if got.iter().zip(&want).any(|(a, b)| a.to_bits() != b.to_bits()) || r.defined as i64 != ci[6].parse::<i64>().unwrap() {
            bad[1] += 1;
        }
        let dv: Vec<f64> = d[..6].iter().map(|v| hexf(v)).collect();
        let bf = BoundingBoxf3::new_from_points(Vec3d::new(dv[0], dv[1], dv[2]), Vec3d::new(dv[3], dv[4], dv[5]));
        if bf.defined as i64 != d[6].parse::<i64>().unwrap() {
            bad[2] += 1;
        }
        let r3: BoundingBox3 = BoundingBoxCast::<BoundingBox3>::cast(&bf);
        let got3 = [r3.min.x, r3.min.y, r3.min.z, r3.max.x, r3.max.y, r3.max.z, r3.defined as i64];
        if got3[..] != cf[..] {
            bad[3] += 1;
        }
        let rb: BoundingBoxf3 = BoundingBoxCastBase::<BoundingBoxf3>::cast(&bi);
        let wantb: Vec<f64> = cb[..6].iter().map(|v| hexf(v)).collect();
        let gotb = [rb.min.x, rb.min.y, rb.min.z, rb.max.x, rb.max.y, rb.max.z];
        if gotb.iter().zip(&wantb).any(|(a, b)| a.to_bits() != b.to_bits()) || rb.defined as i64 != cb[6].parse::<i64>().unwrap() {
            bad[4] += 1;
        }
        cases += 1;
    }
    eprintln!("bounding_box_cast: {cases} cases; mismatches (ctor3, cast3<double>, ctorf3, cast3<coord_t>, cast_base<double>) {bad:?}");
    assert!(cases >= 1000);
    assert_eq!(bad, [0; 5]);
}
