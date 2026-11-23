//! `slicer::bounding_box::BoundingBoxf3` (the port of `BoundingBox3Base<Vec3d>` / `BoundingBoxf3` of
//! libslic3r/BoundingBox.hpp:105-161, 240-258 and BoundingBox.cpp:117-255) against the C++ at the pin.
//!
//! `tests/data/bounding_box3/vectors.txt.gz` is the output of `gen.cpp` beside it; the header of gen.cpp
//! describes the line. Every case carries its inputs and the pin's results; the same calls run here and
//! every result has to be the pin's (integers equal, doubles bit-equal). Boxes that are not defined (the
//! default constructor, points sharing a coordinate, an inverted (pmin, pmax)) are included.
//!
//! `slicer::geometry::BoundingBox3F` is run on the same cases and REPORTED, not asserted: it also stands
//! for `Eigen::AlignedBox` in aabb_tree_indirect.rs (empty box with MAX / lowest corners, `intersects` and
//! `contains` with `<=`), so where `BoundingBox3Base` reads differently (strict `intersects`, no `defined`
//! guard, zero corners of the default box) it keeps the AlignedBox reading. Its counts are printed under
//! names ending in "reported)". `polygon` is asserted where the footprint fits the pin's int32 coord_t
//! (libslic3r.h:38-44); beyond it the C++ conversion is undefined and the count is reported.

use slicer::bounding_box::BoundingBoxf3;
use slicer::geometry::{BoundingBox3F, Point, Point3F, Vec3d};
use std::collections::BTreeMap;
use std::io::Read;

fn read_vectors() -> String {
    let path = std::env::var("BOUNDING_BOX3_VECTORS")
        .unwrap_or_else(|_| format!("{}/tests/data/bounding_box3/vectors.txt.gz", env!("CARGO_MANIFEST_DIR")));
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
    fn v3(&mut self) -> Vec3d {
        let x = self.d();
        let y = self.d();
        let z = self.d();
        Vec3d::new(x, y, z)
    }
}

fn bits3(v: &Vec3d) -> [u64; 3] {
    [v.x.to_bits(), v.y.to_bits(), v.z.to_bits()]
}
fn p3f(v: &Vec3d) -> Point3F {
    Point3F::new(v.x, v.y, v.z)
}
fn v3(p: &Point3F) -> Vec3d {
    Vec3d::new(p.x, p.y, p.z)
}

#[test]
fn bounding_box3_matches_the_pin() {
    let text = read_vectors();
    let mut calls: BTreeMap<&'static str, (usize, usize)> = BTreeMap::new();
    let mut first: Vec<String> = Vec::new();
    let mut cases = 0usize;
    for (ln, line) in text.lines().enumerate() {
        let mut t = Tok { t: line.split(' ').filter(|x| !x.is_empty()).collect(), k: 0 };
        assert_eq!(t.s(), "d");
        cases += 1;
        let kind = t.i();
        let n = t.i() as usize;
        let pts: Vec<Vec3d> = (0..n).map(|_| t.v3()).collect();
        let m = t.i() as usize;
        let pts2: Vec<Vec3d> = (0..m).map(|_| t.v3()).collect();
        let q = t.v3();
        let delta = t.d();
        let v = t.v3();
        assert_eq!(t.s(), "->");

        let mut check = |name: &'static str, ok: bool, what: String| {
            let e = calls.entry(name).or_insert((0, 0));
            e.0 += 1;
            if !ok {
                e.1 += 1;
                if first.len() < 16 && !name.ends_with("reported)") {
                    first.push(format!("line {}: {name}: {what}", ln + 1));
                }
            }
        };
        // Reads the pin's value and checks the port's; returns the pin's value for the reported family.
        macro_rules! v3_eq {
            ($name:expr, $got:expr) => {{
                let want = t.v3();
                let got: Vec3d = $got;
                check($name, bits3(&got) == bits3(&want), format!("got {got:?} want {want:?}"));
                want
            }};
        }
        macro_rules! int_eq {
            ($name:expr, $got:expr) => {{
                let want = t.i();
                let got: i64 = $got;
                check($name, got == want, format!("got {got} want {want}"));
                want
            }};
        }
        macro_rules! dbl_eq {
            ($name:expr, $got:expr) => {{
                let want = t.d();
                let got: f64 = $got;
                check($name, got.to_bits() == want.to_bits(), format!("got {got:e} want {want:e}"));
                want
            }};
        }
        macro_rules! box_eq {
            ($name:expr, $got:expr) => {{
                let b: BoundingBoxf3 = $got;
                let d = int_eq!($name, b.defined as i64);
                let mn = v3_eq!($name, b.min);
                let mx = v3_eq!($name, b.max);
                (d, mn, mx)
            }};
        }

        // BoundingBox.hpp:116-130 (iterator constructor), 110-112 (pmin, pmax), 20 (default).
        let bb = if kind == 4 { BoundingBoxf3::new() } else { BoundingBoxf3::new_from_points_slice(&pts) };
        let bb2 = match kind {
            2 => BoundingBoxf3::new_from_points(pts2[0], *pts2.last().unwrap()),
            3 => BoundingBoxf3::new(),
            _ => BoundingBoxf3::new_from_points_slice(&pts2),
        };
        let (d1, mn1, mx1) = box_eq!("BoundingBoxf3(points) / (pmin, pmax) / ()", bb);
        let (d2, mn2, mx2) = box_eq!("BoundingBoxf3(points) / (pmin, pmax) / ()", bb2);
        // The pin-shaped constructors added to geometry BoundingBox3F in R932 (asserted).
        let same = |g: &BoundingBox3F, d: i64, mn: &Vec3d, mx: &Vec3d| {
            g.is_defined() as i64 == d && bits3(&v3(&g.min)) == bits3(mn) && bits3(&v3(&g.max)) == bits3(mx)
        };
        if kind != 4 {
            let g = BoundingBox3F::from_points(&pts.iter().map(p3f).collect::<Vec<_>>());
            check("geometry BoundingBox3F::from_points", same(&g, d1, &mn1, &mx1), format!("{g:?}"));
        }
        match kind {
            2 => {
                let g = BoundingBox3F::new_from_points(p3f(&pts2[0]), p3f(pts2.last().unwrap()));
                check("geometry BoundingBox3F::new_from_points", same(&g, d2, &mn2, &mx2), format!("{g:?}"));
            }
            3 => {}
            _ => {
                let g = BoundingBox3F::from_points(&pts2.iter().map(p3f).collect::<Vec<_>>());
                check("geometry BoundingBox3F::from_points", same(&g, d2, &mn2, &mx2), format!("{g:?}"));
            }
        }
        // A geometry box with the pin's corners, for the methods that do not read `defined`.
        let gp = BoundingBox3F::from_points_minmax(p3f(&mn1), p3f(&mx1));
        let gp2 = BoundingBox3F::from_points_minmax(p3f(&mn2), p3f(&mx2));

        // The live 3D box of the geometry module on the same inputs (reported).
        let gbb = if kind == 4 {
            BoundingBox3F::new()
        } else {
            BoundingBox3F::from_points(&pts.iter().map(p3f).collect::<Vec<_>>())
        };
        let gbb2 = match kind {
            2 => BoundingBox3F::from_points_minmax(p3f(&pts2[0]), p3f(pts2.last().unwrap())),
            3 => BoundingBox3F::new(),
            _ => BoundingBox3F::from_points(&pts2.iter().map(p3f).collect::<Vec<_>>()),
        };
        check(
            "geometry BoundingBox3F constructors (reported)",
            gbb.is_defined() == bb.defined
                && bits3(&v3(&gbb.min)) == bits3(&bb.min)
                && bits3(&v3(&gbb.max)) == bits3(&bb.max)
                && gbb2.is_defined() == bb2.defined,
            String::new(),
        );

        let size = v3_eq!("size", bb.size());
        check("geometry BoundingBox3F size (reported)", bits3(&v3(&gbb.size())) == bits3(&size), String::new());
        let r = dbl_eq!("radius", bb.radius());
        check("geometry BoundingBox3F::radius", gp.radius().to_bits() == r.to_bits(), format!("{}", gp.radius()));
        let center = v3_eq!("center", bb.center());
        check(
            "geometry BoundingBox3F center (reported)",
            bits3(&v3(&gbb.center())) == bits3(&center),
            String::new(),
        );
        let ms = dbl_eq!("max_size", bb.max_size());
        check("geometry BoundingBox3F::max_size", gp.max_size().to_bits() == ms.to_bits(), String::new());
        let volume = dbl_eq!("volume", bb.volume());
        check(
            "geometry BoundingBox3F volume (reported)",
            gbb.volume().to_bits() == volume.to_bits(),
            String::new(),
        );
        int_eq!("empty", bb.is_empty() as i64);
        let c = int_eq!("contains(point)", bb.contains_point(q) as i64);
        check(
            "geometry BoundingBox3F contains_point (reported)",
            gbb.contains_point(&p3f(&q)) as i64 == c,
            String::new(),
        );
        let c = int_eq!("contains(box)", bb.contains_bb(&bb2) as i64);
        check("geometry BoundingBox3F::contains", gp.contains(&gp2) as i64 == c, String::new());
        let c = int_eq!("intersects", bb.intersects(&bb2) as i64);
        check("geometry BoundingBox3F intersects (reported)", gbb.intersects(&gbb2) as i64 == c, String::new());
        {
            let mut b = bb;
            b.merge_point(q);
            let (d, mn, mx) = box_eq!("merge(point)", b);
            let mut g = gbb;
            g.merge_point(p3f(&q));
            check(
                "geometry BoundingBox3F merge_point (reported)",
                g.is_defined() as i64 == d && bits3(&v3(&g.min)) == bits3(&mn) && bits3(&v3(&g.max)) == bits3(&mx),
                String::new(),
            );
        }
        {
            let mut b = bb;
            b.merge_bb(&bb2);
            box_eq!("merge(box)", b);
        }
        {
            let mut b = bb;
            b.merge_points(&pts2);
            box_eq!("merge(points)", b);
        }
        {
            let mut b = bb;
            b.offset(delta);
            let mn = v3_eq!("offset", b.min);
            let mx = v3_eq!("offset", b.max);
            let mut g = gp;
            g.offset(delta);
            check("geometry BoundingBox3F::offset", same(&g, 1, &mn, &mx), String::new());
        }
        {
            let b = bb.inflated(delta);
            let mn = v3_eq!("inflated", b.min);
            let mx = v3_eq!("inflated", b.max);
            check("geometry BoundingBox3F::inflated", same(&gp.inflated(delta), 1, &mn, &mx), String::new());
        }
        {
            let mut b = bb;
            b.translate_vec(v);
            let mn = v3_eq!("translate", b.min);
            let mx = v3_eq!("translate", b.max);
            let mut g = gp;
            g.translate(p3f(&v));
            check("geometry BoundingBox3F::translate", same(&g, 1, &mn, &mx), String::new());
        }
        for (name, scaled) in [("polygon(false)", false), ("polygon(true)", true)] {
            let poly = bb.polygon(scaled);
            let got: Vec<Point> = poly.points().to_vec();
            let want: Vec<Point> = (0..4)
                .map(|_| {
                    let x = t.i();
                    let y = t.i();
                    Point::new(x, y)
                })
                .collect();
            // coord_t is int32_t at the pin (libslic3r.h:38-44) and the crate's Coord is i64: a footprint
            // beyond the int32 range is an out-of-range conversion (undefined) in the C++, reported only.
            let lim = |v: f64| {
                let s = v * (1.0 / if scaled { slicer::libslic3r::SCALING_FACTOR } else { 1.0 });
                s >= -2147483648.0 && s < 2147483648.0
            };
            let in_range = lim(bb.min.x) && lim(bb.min.y) && lim(bb.max.x) && lim(bb.max.y);
            let name = match (name, in_range) {
                (_, true) => name,
                ("polygon(false)", false) => "polygon(false) (beyond int32 coord_t; reported)",
                _ => "polygon(true) (beyond int32 coord_t; reported)",
            };
            let ggot: Vec<Point> = gp.polygon(scaled).points().to_vec();
            check(name, got == want, format!("got {got:?} want {want:?}"));
            let gname = if in_range { "geometry BoundingBox3F::polygon" } else { "geometry BoundingBox3F::polygon (beyond int32 coord_t; reported)" };
            check(gname, ggot == want, format!("got {ggot:?} want {want:?}"));
        }
        assert_eq!(t.k, t.t.len(), "line {}: tokens left", ln + 1);
    }
    let mut total = 0usize;
    let mut bad = 0usize;
    for (name, (n, m)) in &calls {
        println!("{name}: {n} calls, {m} mismatches");
        if !name.ends_with("reported)") {
            total += n;
            bad += m;
        }
    }
    println!("bounding_box3: {cases} cases, {total} asserted calls, {bad} mismatches");
    for f in &first {
        println!("{f}");
    }
    assert_eq!(cases, 4000);
    assert_eq!(bad, 0, "{bad} of {total} results differ from the pin");
}
