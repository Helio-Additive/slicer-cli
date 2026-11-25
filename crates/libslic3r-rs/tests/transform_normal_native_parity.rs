//! Geometry::transform_normal (Geometry.hpp:382-387 with a Vec3d normal, 390-393 with a Vec3f one)
//! against the pin, compiled with the engine's flags, on 4,000 transforms of
//! `tests/data/transform_normal/gen.cpp` (assemble_transform with uniform and non-uniform scales and
//! mirrors, raw random linear parts, singular ones; gzip -9n). The intermediate
//! `transform.linear().inverse().transpose() * normal` and both results must be bit-equal.

use slicer::geometry::geometry::{eigen, transform_normal, Transform3d, Vec3d};
use std::io::Read;

struct Tok<'a>(std::str::SplitWhitespace<'a>);

impl<'a> Tok<'a> {
    fn s(&mut self) -> &'a str {
        self.0.next().expect("token")
    }
    fn d(&mut self) -> f64 {
        f64::from_bits(u64::from_str_radix(self.s(), 16).expect("double"))
    }
    fn f(&mut self) -> f32 {
        f32::from_bits(u32::from_str_radix(self.s(), 16).expect("float"))
    }
    fn trafo(&mut self) -> Transform3d {
        let mut t = Transform3d::zeros();
        for r in 0..4 {
            for c in 0..4 {
                t[(r, c)] = self.d();
            }
        }
        t
    }
    fn v3(&mut self) -> Vec3d {
        Vec3d::new(self.d(), self.d(), self.d())
    }
    fn arrow(&mut self) {
        assert_eq!(self.s(), "->");
    }
}

fn bits(v: &Vec3d) -> [u64; 3] {
    [v.x.to_bits(), v.y.to_bits(), v.z.to_bits()]
}

#[test]
fn transform_normal_matches_the_pin() {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/transform_normal/vectors.txt.gz");
    let mut text = String::new();
    flate2::read::GzDecoder::new(std::fs::File::open(path).unwrap()).read_to_string(&mut text).unwrap();
    let (mut nd, mut nf) = (0usize, 0usize);
    let (mut bad_n, mut bad_d, mut bad_f) = (0usize, 0usize, 0usize);
    for line in text.lines() {
        let mut t = Tok(line.split_whitespace());
        match t.s() {
            "d" => {
                let trafo = t.trafo();
                let normal = t.v3();
                t.arrow();
                let (want_n, want) = (t.v3(), t.v3());
                // Geometry.hpp:384, as transform_normal evaluates it.
                let inv = eigen::mat3_inverse(&eigen::linear(&trafo));
                let row = |i: usize| (inv[(0, i)] * normal.x + inv[(1, i)] * normal.y) + inv[(2, i)] * normal.z;
                let n = Vec3d::new(row(0), row(1), row(2));
                nd += 1;
                // NaN results (a singular linear part) compare by bits as well.
                bad_n += (bits(&n) != bits(&want_n)) as usize;
                bad_d += (bits(&transform_normal(&trafo, &normal)) != bits(&want)) as usize;
            }
            "f" => {
                let trafo = t.trafo();
                let normal = nalgebra::Vector3::<f32>::new(t.f(), t.f(), t.f());
                t.arrow();
                let want = t.v3();
                nf += 1;
                bad_f += (bits(&transform_normal(&trafo, &normal)) != bits(&want)) as usize;
            }
            other => panic!("unknown tag {other}"),
        }
    }
    eprintln!("inverse().transpose() * normal: {nd} cases, {bad_n} mismatches");
    eprintln!("transform_normal(Vec3d): {nd} cases, {bad_d} mismatches");
    eprintln!("transform_normal(Vec3f): {nf} cases, {bad_f} mismatches");
    assert_eq!((nd, nf), (4000, 4000));
    assert_eq!((bad_n, bad_d, bad_f), (0, 0, 0));
}
