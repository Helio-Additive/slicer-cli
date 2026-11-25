//! BoundingBox.hpp:248-263 under the C++ names against the pin (tests/data/bounding_box_cpp_names,
//! gen.cpp): empty() of BoundingBoxf, BoundingBoxf3, BoundingBox, BoundingBox3; scaled(BoundingBoxf),
//! scaled(BoundingBoxf3); unscaled(BoundingBox), unscaled(BoundingBox3). Bit-equal results required.

use slicer::bounding_box::{
    BoundingBox, BoundingBox3, BoundingBoxEmpty, BoundingBoxScaled, BoundingBoxUnscaled, BoundingBoxf,
    BoundingBoxf3,
};
use slicer::geometry::{Point, Point3, Vec2d, Vec3d};
use std::collections::BTreeMap;
use std::io::Read;

fn vectors() -> String {
    let path = format!("{}/tests/data/bounding_box_cpp_names/vectors.txt.gz", env!("CARGO_MANIFEST_DIR"));
    let raw = std::fs::read(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
    let mut s = String::new();
    flate2::read::GzDecoder::new(&raw[..]).read_to_string(&mut s).unwrap();
    s
}

fn d(s: &str) -> f64 {
    f64::from_bits(u64::from_str_radix(s, 16).unwrap())
}
fn i(s: &str) -> i64 {
    s.parse().unwrap()
}
fn b(s: &str) -> bool {
    s == "1"
}

#[test]
fn bounding_box_cpp_names_match_the_pin() {
    let text = vectors();
    let mut counts = BTreeMap::<&'static str, (usize, usize)>::new();
    let mut check = |name: &'static str, ok: bool| {
        let e = counts.entry(name).or_insert((0, 0));
        e.0 += 1;
        if !ok {
            e.1 += 1;
        }
    };
    for line in text.lines() {
        let t: Vec<&str> = line.split_whitespace().collect();
        assert_eq!(t[0], "BB");
        let a: Vec<f64> = (1..7).map(|k| d(t[k])).collect();
        let c: Vec<i64> = (7..13).map(|k| i(t[k])).collect();
        let bf = BoundingBoxf { min: Vec2d::new(a[0], a[1]), max: Vec2d::new(a[3], a[4]), defined: b(t[13]) };
        let bf3 = BoundingBoxf3 { min: Vec3d::new(a[0], a[1], a[2]), max: Vec3d::new(a[3], a[4], a[5]), defined: b(t[14]) };
        let bi = BoundingBox { min: Point::new(c[0], c[1]), max: Point::new(c[3], c[4]), defined: b(t[15]) };
        let bi3 = BoundingBox3 { min: Point3::new(c[0], c[1], c[2]), max: Point3::new(c[3], c[4], c[5]), defined: b(t[16]) };
        assert_eq!(t[17], "|");
        check("empty(const BoundingBoxBase<Vec2d>&)", BoundingBoxEmpty::empty(&bf) == b(t[18]));
        check("empty(const BoundingBox3Base<Vec3d>&)", BoundingBoxEmpty::empty(&bf3) == b(t[19]));
        check("empty(const BoundingBoxBase<Point>&)", BoundingBoxEmpty::empty(&bi) == b(t[20]));
        check("empty(const BoundingBox3Base<Vec3crd>&)", BoundingBoxEmpty::empty(&bi3) == b(t[21]));
        let s2: BoundingBox = bf.scaled();
        check(
            "scaled(const BoundingBoxf&)",
            [s2.min.x, s2.min.y, s2.max.x, s2.max.y] == [i(t[22]), i(t[23]), i(t[24]), i(t[25])] && s2.defined == b(t[26]),
        );
        let s3: BoundingBox3 = bf3.scaled();
        check(
            "scaled(const BoundingBoxf3&)",
            [s3.min.x, s3.min.y, s3.min.z, s3.max.x, s3.max.y, s3.max.z]
                == [i(t[27]), i(t[28]), i(t[29]), i(t[30]), i(t[31]), i(t[32])]
                && s3.defined == b(t[33]),
        );
        let u2: BoundingBoxf = bi.unscaled();
        check(
            "unscaled(const BoundingBox&)",
            [u2.min.x, u2.min.y, u2.max.x, u2.max.y].map(f64::to_bits)
                == [d(t[34]), d(t[35]), d(t[36]), d(t[37])].map(f64::to_bits)
                && u2.defined == b(t[38]),
        );
        let u3: BoundingBoxf3 = bi3.unscaled();
        check(
            "unscaled(const BoundingBox3&)",
            [u3.min.x, u3.min.y, u3.min.z, u3.max.x, u3.max.y, u3.max.z].map(f64::to_bits)
                == [d(t[39]), d(t[40]), d(t[41]), d(t[42]), d(t[43]), d(t[44])].map(f64::to_bits)
                && u3.defined == b(t[45]),
        );
    }
    let mut bad = 0;
    for (name, (n, m)) in &counts {
        eprintln!("{name}: {n} cases, {m} mismatches");
        bad += m;
    }
    assert_eq!(counts.len(), 8);
    assert_eq!(bad, 0);
}
