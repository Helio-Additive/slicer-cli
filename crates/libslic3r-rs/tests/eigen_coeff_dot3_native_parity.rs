//! Every call site of `geometry::eigen::coeff_dot3` (the third row of a lazy product evaluated alone,
//! `e0 + (e1 + e2)`) against the pin's Eigen expression, compiled with the engine's flags, on 1,200
//! random cases of `tests/data/eigen_coeff_dot3/gen.cpp` (every coefficient random, so the order of the
//! three-term sums shows; gzip -9n):
//!
//! | site (Rust) | pin expression |
//! |-|-|
//! | geometry.rs `eigen::mat3_mul` | `Matrix3d * Matrix3d` |
//! | geometry.rs `eigen::mat3_mul_vec`, model.rs `eigen_matrix3_mul_vec` | `Matrix3d * Vec3d` |
//! | geometry.rs `eigen::transform_mul`, model.rs `eigen_mul_affine` | `Transform3d * Transform3d` |
//! | geometry.rs `eigen::transform_inverse` | `Transform3d::inverse()` |
//! | geometry.rs `eigen::translate` | `Transform3d::translate(Vec3d)` |
//! | geometry.rs `eigen::rotate` (through `mat3_mul`) | `Transform3d::rotate(Matrix3d)` |
//! | line.rs `transform` | Line.cpp:10-20 `transform(Linef3, Transform3d)` |
//! | bounding_box.rs `transformed_eigen`, model.rs `eigen_bbox_transformed` | BoundingBox.cpp:265-294 |
//!
//! All of them multiply a column-major left-hand side (a `Matrix3d` or the linear block of a
//! `Transform3d`), where the pin's row 2 is `coeff()` over a strided row, `e0 + (e1 + e2)`; the test
//! also counts how often the other order, `(e0 + e1) + e2`, would differ there (it must, or the cases
//! would not tell the two apart). The transposed product `Matrix3d.transpose() * Vec3d` is the
//! opposite: a contiguous row, `(e0 + e1) + e2` on every row, which `coeff_dot3` does not give
//! (Geometry.hpp:384's `transform_normal` evaluates it so, R953).

use slicer::geometry::geometry::eigen::{
    coeff_dot3, mat3_mul, mat3_mul_vec, rotate, transform_inverse, transform_mul, translate, Matrix3d,
    Transform3d, Vec3d,
};
use std::io::Read;

struct Tok<'a>(std::str::SplitWhitespace<'a>);

impl<'a> Tok<'a> {
    fn s(&mut self) -> &'a str {
        self.0.next().expect("token")
    }
    fn d(&mut self) -> f64 {
        f64::from_bits(u64::from_str_radix(self.s(), 16).expect("double"))
    }
    fn m3(&mut self) -> Matrix3d {
        let mut m = Matrix3d::zeros();
        for r in 0..3 {
            for c in 0..3 {
                m[(r, c)] = self.d();
            }
        }
        m
    }
    fn v3(&mut self) -> Vec3d {
        Vec3d::new(self.d(), self.d(), self.d())
    }
    fn t(&mut self) -> Transform3d {
        let mut t = Transform3d::zeros();
        for r in 0..4 {
            for c in 0..4 {
                t[(r, c)] = self.d();
            }
        }
        t
    }
}

fn eq_m3(a: &Matrix3d, b: &Matrix3d) -> bool {
    (0..3).all(|r| (0..3).all(|c| a[(r, c)].to_bits() == b[(r, c)].to_bits()))
}
fn eq_v3(a: &Vec3d, b: &Vec3d) -> bool {
    (0..3).all(|i| a[i].to_bits() == b[i].to_bits())
}
fn eq_t(a: &Transform3d, b: &Transform3d) -> bool {
    (0..4).all(|r| (0..4).all(|c| a[(r, c)].to_bits() == b[(r, c)].to_bits()))
}

/// `(e0 + e1) + e2`, unfused: one coefficient of a product whose left row is contiguous.
fn seq_dot3(a0: f64, a1: f64, a2: f64, x0: f64, x1: f64, x2: f64) -> f64 {
    (a0 * x0 + a1 * x1) + a2 * x2
}

#[derive(Default, Debug)]
struct Bad {
    mat3_mul: usize,
    mat3_mul_vec: usize,
    transform_mul: usize,
    transform_inverse: usize,
    translate: usize,
    rotate: usize,
    line_transform: usize,
    bbox_transformed: usize,
    model_bbox_transformed: usize,
    model_wrappers: usize,
    transposed: usize,
}

#[test]
fn coeff_dot3_sites_match_the_pin() {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/eigen_coeff_dot3/vectors.txt.gz");
    let mut text = String::new();
    flate2::read::GzDecoder::new(std::fs::File::open(path).unwrap()).read_to_string(&mut text).unwrap();
    let mut bad = Bad::default();
    // How often the other order would differ from the pin at each kind of site (must be > 0).
    let (mut alt_row2, mut alt_transposed) = (0usize, 0usize);
    let mut cases = 0usize;
    for line in text.lines() {
        let mut t = Tok(line.split_whitespace());
        assert_eq!(t.s(), "c");
        let (a, b, v, w, tr, u) = (t.m3(), t.m3(), t.v3(), t.v3(), t.t(), t.t());
        assert_eq!(t.s(), "->");
        let (want_ab, want_av, want_tu, want_ti, want_tt, want_tr) = (t.m3(), t.v3(), t.t(), t.t(), t.t(), t.t());
        let (want_la, want_lb, want_bmin, want_bmax, want_atv) = (t.v3(), t.v3(), t.v3(), t.v3(), t.v3());
        cases += 1;

        // Matrix3d * Matrix3d
        let ab = mat3_mul(&a, &b);
        bad.mat3_mul += usize::from(!eq_m3(&ab, &want_ab));
        for c in 0..3 {
            let alt = seq_dot3(a[(2, 0)], a[(2, 1)], a[(2, 2)], b[(0, c)], b[(1, c)], b[(2, c)]);
            alt_row2 += usize::from(alt.to_bits() != want_ab[(2, c)].to_bits());
        }
        // Matrix3d * Vec3d
        let av = mat3_mul_vec(&a, &v);
        bad.mat3_mul_vec += usize::from(!eq_v3(&av, &want_av));
        let alt = seq_dot3(a[(2, 0)], a[(2, 1)], a[(2, 2)], v.x, v.y, v.z);
        alt_row2 += usize::from(alt.to_bits() != want_av.z.to_bits());
        // Transform3d * Transform3d
        let tu = transform_mul(&tr, &u);
        bad.transform_mul += usize::from(!eq_t(&tu, &want_tu));
        // Transform3d::inverse()
        bad.transform_inverse += usize::from(!eq_t(&transform_inverse(&tr), &want_ti));
        // Transform3d::translate(v)
        let mut tt = tr;
        translate(&mut tt, &v);
        bad.translate += usize::from(!eq_t(&tt, &want_tt));
        let alt = seq_dot3(tr[(2, 0)], tr[(2, 1)], tr[(2, 2)], v.x, v.y, v.z) + tr[(2, 3)];
        alt_row2 += usize::from(alt.to_bits() != want_tt[(2, 3)].to_bits());
        // Transform3d::rotate(a)
        let mut trr = tr;
        rotate(&mut trr, &a);
        bad.rotate += usize::from(!eq_t(&trr, &want_tr));
        // model.rs's names for the same products
        let mw = !eq_m3(&slicer::model::eigen_matrix3_mul(&a, &b), &want_ab)
            || !eq_v3(&slicer::model::eigen_matrix3_mul_vec(&a, &v), &want_av)
            || !eq_t(&slicer::model::eigen_mul_affine(&tr, &u), &want_tu);
        bad.model_wrappers += usize::from(mw);

        // Line.cpp:10-20
        let mut col = [0f64; 16];
        for c in 0..4 {
            for r in 0..4 {
                col[c * 4 + r] = tr[(r, c)];
            }
        }
        let p3 = |p: &Vec3d| slicer::geometry::Point3F::new(p.x, p.y, p.z);
        let l = slicer::geometry::line_transform(
            &slicer::geometry::Linef3::new(p3(&v), p3(&w)),
            &slicer::geometry::Transform3D { matrix: col },
        );
        let lv = |p: &slicer::geometry::Point3F| Vec3d::new(p.x, p.y, p.z);
        bad.line_transform += usize::from(!eq_v3(&lv(&l.a), &want_la) || !eq_v3(&lv(&l.b), &want_lb));
        let alt = seq_dot3(tr[(2, 0)], tr[(2, 1)], tr[(2, 2)], v.x, v.y, v.z) + tr[(2, 3)];
        alt_row2 += usize::from(alt.to_bits() != want_la.z.to_bits());

        // BoundingBox.cpp:265-294 (bounding_box.rs and model.rs)
        let gv = |p: &Vec3d| slicer::geometry::Vec3d::new(p.x, p.y, p.z);
        let bb = slicer::bounding_box::BoundingBoxf3::new_from_points(
            gv(&Vec3d::new(v.x.min(w.x), v.y.min(w.y), v.z.min(w.z))),
            gv(&Vec3d::new(v.x.max(w.x), v.y.max(w.y), v.z.max(w.z))),
        );
        let mut m4 = [[0f64; 4]; 4];
        for (r, row) in m4.iter_mut().enumerate() {
            for (c, x) in row.iter_mut().enumerate() {
                *x = tr[(r, c)];
            }
        }
        let tb = bb.transformed(&m4);
        let bv = |p: &slicer::geometry::Vec3d| Vec3d::new(p.x, p.y, p.z);
        bad.bbox_transformed += usize::from(!eq_v3(&bv(&tb.min), &want_bmin) || !eq_v3(&bv(&tb.max), &want_bmax));
        let mb = slicer::model::eigen_bbox_transformed(&bb, &tr);
        bad.model_bbox_transformed += usize::from(!eq_v3(&bv(&mb.min), &want_bmin) || !eq_v3(&bv(&mb.max), &want_bmax));

        // Matrix3d.transpose() * Vec3d: every row (e0 + e1) + e2 (a contiguous row); coeff_dot3 is not it.
        let atv = Vec3d::new(
            seq_dot3(a[(0, 0)], a[(1, 0)], a[(2, 0)], v.x, v.y, v.z),
            seq_dot3(a[(0, 1)], a[(1, 1)], a[(2, 1)], v.x, v.y, v.z),
            seq_dot3(a[(0, 2)], a[(1, 2)], a[(2, 2)], v.x, v.y, v.z),
        );
        bad.transposed += usize::from(!eq_v3(&atv, &want_atv));
        for i in 0..3 {
            let alt = coeff_dot3(a[(0, i)], a[(1, i)], a[(2, i)], v.x, v.y, v.z);
            alt_transposed += usize::from(alt.to_bits() != want_atv[i].to_bits());
        }
    }
    eprintln!(
        "eigen coeff_dot3 sites: {cases} cases; mismatches {bad:?}; the other order would differ on {alt_row2} row-2 \
         coefficients, coeff_dot3 on {alt_transposed} transposed-product coefficients"
    );
    assert_eq!(cases, 1200);
    assert!(alt_row2 > 0 && alt_transposed > 0, "the cases do not tell the two orders apart");
    let total = bad.mat3_mul
        + bad.mat3_mul_vec
        + bad.transform_mul
        + bad.transform_inverse
        + bad.translate
        + bad.rotate
        + bad.line_transform
        + bad.bbox_transformed
        + bad.model_bbox_transformed
        + bad.model_wrappers
        + bad.transposed;
    assert_eq!(total, 0, "{bad:?}");
}
