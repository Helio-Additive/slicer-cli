//! The free functions of libslic3r/Point.hpp that `point_native_parity` does not reach, under their C++
//! names in `slicer::geometry` (point.rs), against the C++ at the pin.
//!
//! `tests/data/point_rest/vectors.txt.gz` is the output of `gen.cpp` beside it; the header of gen.cpp
//! describes the line. Integers have to be equal, doubles and floats bit-equal.

use slicer::geometry::{
    point_angle, point_cross2, point_has_reflection, point_lerp, point_to_3d, shorter_then, Cross2, GetBase,
    IsApprox, Perp, Point, Point3, Point3F, PointF, Transform3D, Unscale,
};
use std::collections::BTreeMap;
use std::io::Read;

fn read_vectors() -> String {
    let path = std::env::var("POINT_REST_VECTORS")
        .unwrap_or_else(|_| format!("{}/tests/data/point_rest/vectors.txt.gz", env!("CARGO_MANIFEST_DIR")));
    let file = std::fs::File::open(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
    let mut text = String::new();
    if path.ends_with(".gz") {
        flate2::read::GzDecoder::new(file).read_to_string(&mut text).expect("gzip");
    } else {
        std::io::BufReader::new(file).read_to_string(&mut text).expect("read");
    }
    text
}

struct Tok<'a> {
    t: Vec<&'a str>,
    k: usize,
}
impl<'a> Tok<'a> {
    fn s(&mut self) -> &'a str {
        self.k += 1;
        self.t[self.k - 1]
    }
    fn i(&mut self) -> i64 {
        self.s().parse().expect("integer")
    }
    fn d(&mut self) -> f64 {
        f64::from_bits(u64::from_str_radix(self.s(), 16).expect("double"))
    }
    fn f(&mut self) -> f32 {
        f32::from_bits(u32::from_str_radix(self.s(), 16).expect("float"))
    }
    fn pt(&mut self) -> Point {
        let x = self.i();
        let y = self.i();
        Point::new(x, y)
    }
}

#[test]
fn point_rest_matches_the_pin() {
    let text = read_vectors();
    let mut calls: BTreeMap<&'static str, (usize, usize)> = BTreeMap::new();
    let mut first: Vec<String> = Vec::new();
    let mut cases = 0usize;
    let mut reflections = 0usize;
    let mut approx_true = [0usize; 4];
    for (ln, line) in text.lines().enumerate() {
        let mut t = Tok { t: line.split(' ').filter(|x| !x.is_empty()).collect(), k: 0 };
        assert_eq!(t.s(), "c");
        cases += 1;
        let v1 = PointF::new(t.d(), t.d());
        let v2 = PointF::new(t.d(), t.d());
        let z = t.d();
        let a = t.pt();
        let b = t.pt();
        let ia = [t.i(), t.i()];
        let ib = [t.i(), t.i()];
        let p3 = Point3::new(t.i(), t.i(), t.i());
        let w3 = Point3F::new(t.d(), t.d(), t.d());
        let eps = t.d();
        let f2a = [t.f(), t.f()];
        let f2b = [t.f(), t.f()];
        let f3a = [t.f(), t.f(), t.f()];
        let f3b = [t.f(), t.f(), t.f()];
        let feps = t.f();
        let tt = t.d();
        let len = t.i();
        let mut m = [0f64; 16];
        for v in m.iter_mut() {
            *v = t.d();
        }
        assert_eq!(t.s(), "->");
        let tr = Transform3D { matrix: m };
        let lin = slicer::geometry::geometry::eigen::Matrix3d::from_fn(|r, c| m[c * 4 + r]);

        let mut check = |name: &'static str, ok: bool, what: String| {
            let e = calls.entry(name).or_insert((0, 0));
            e.0 += 1;
            if !ok {
                e.1 += 1;
                if first.len() < 12 {
                    first.push(format!("line {}: {name}: {what}", ln + 1));
                }
            }
        };
        macro_rules! dbl_eq {
            ($name:expr, $got:expr) => {{
                let want = t.d();
                let got: f64 = $got;
                check($name, got.to_bits() == want.to_bits(), format!("got {got:e} want {want:e}"));
            }};
        }
        macro_rules! int_eq {
            ($name:expr, $got:expr) => {{
                let want = t.i();
                let got: i64 = $got;
                check($name, got == want, format!("got {got} want {want}"));
            }};
        }
        macro_rules! p2_eq {
            ($name:expr, $got:expr) => {{
                let got: PointF = $got;
                dbl_eq!($name, got.x);
                dbl_eq!($name, got.y);
            }};
        }
        macro_rules! p3_eq {
            ($name:expr, $got:expr) => {{
                let got: Point3F = $got;
                dbl_eq!($name, got.x);
                dbl_eq!($name, got.y);
                dbl_eq!($name, got.z);
            }};
        }

        dbl_eq!("cross2(Vec2d)", point_cross2(&v1, &v2));
        int_eq!("cross2(Vec2i64)", Cross2::cross2(&ia, &ib));
        p2_eq!("perp(Vec2d)", Perp::perp(&v1));
        {
            let q = Perp::perp(&a);
            int_eq!("perp(Vec2crd)", q.x);
            int_eq!("perp(Vec2crd)", q.y);
        }
        dbl_eq!("angle(Vec2d)", point_angle(&v1, &v2));
        // Point.hpp:115-116: v1.template cast<double>() of a Vec2crd
        dbl_eq!("angle(Point)", point_angle(&PointF::new(a.x as f64, a.y as f64), &PointF::new(b.x as f64, b.y as f64)));
        p3_eq!("to_3d", point_to_3d(&v1, z));
        p2_eq!("unscale(x, y)", (a.x, a.y).unscale());
        p2_eq!("unscale(Vec2crd)", a.unscale());
        p2_eq!("unscale(Vec2d)", v1.unscale());
        p3_eq!("unscale(x, y, z)", (p3.x, p3.y, p3.z).unscale());
        p3_eq!("unscale(Vec3crd)", p3.unscale());
        p3_eq!("unscale(Vec3d)", w3.unscale());
        {
            // The second point of each pair as gen.cpp builds it (Eigen sums and products of one
            // coefficient each, unfused).
            let q2 = PointF::new(v1.x + w3.x * eps, v1.y + w3.y * eps);
            let r = v1.is_approx(&q2, eps);
            approx_true[0] += r as usize;
            int_eq!("is_approx(Vec2d)", r as i64);
            let drange = [1., 300., 1e5, 1e8][(ln / 4) % 4];
            let s = 2. * eps / drange;
            let p = Point3F::new(w3.x * drange, w3.y * drange, w3.z * drange);
            let q3 = Point3F::new(w3.x * drange + v1.x * s, w3.y * drange + v1.y * s, w3.z * drange + z * s);
            let r = p.is_approx(&q3, eps);
            approx_true[1] += r as usize;
            int_eq!("is_approx(Vec3d)", r as i64);
            let r = f2a.is_approx(&f2b, feps);
            approx_true[2] += r as usize;
            int_eq!("is_approx(Vec2f)", r as i64);
            let r = f3a.is_approx(&f3b, feps);
            approx_true[3] += r as usize;
            int_eq!("is_approx(Vec3f)", r as i64);
        }
        let refl = point_has_reflection(&tr);
        reflections += refl as usize;
        int_eq!("has_reflection", refl as i64);
        for i in 0..3u32 {
            p3_eq!("get_base(Transform3d)", tr.get_base(i));
        }
        p3_eq!("get_x_base(Transform3d)", tr.get_x_base());
        p3_eq!("get_y_base(Transform3d)", tr.get_y_base());
        p3_eq!("get_z_base(Transform3d)", tr.get_z_base());
        for i in 0..3u32 {
            p3_eq!("get_base(LinearPart)", lin.get_base(i));
        }
        p3_eq!("get_x_base(LinearPart)", lin.get_x_base());
        p3_eq!("get_y_base(LinearPart)", lin.get_y_base());
        p3_eq!("get_z_base(LinearPart)", lin.get_z_base());
        {
            let q = point_lerp(&a, &b, tt);
            int_eq!("lerp", q.x);
            int_eq!("lerp", q.y);
        }
        int_eq!("shorter_then", shorter_then(&(a - b), len) as i64);
        assert_eq!(t.k, t.t.len(), "line {}: tokens left", ln + 1);
    }
    let mut total = 0usize;
    let mut bad = 0usize;
    for (name, (n, m)) in &calls {
        println!("{name}: {n} calls, {m} mismatches");
        total += n;
        bad += m;
    }
    println!(
        "point_rest: {cases} cases, {total} calls, {bad} mismatches; has_reflection true {reflections}; is_approx true {approx_true:?}"
    );
    for f in &first {
        println!("{f}");
    }
    assert_eq!(cases, 4000);
    assert_eq!(bad, 0, "{bad} of {total} results differ from the pin");
}
