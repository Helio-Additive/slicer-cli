//! TriangleSelector.cpp's file-static helpers and ClippingPlane's members against the pin, on the
//! random inputs of `tests/data/triangle_selector/gen.cpp free` (vectors_free.txt.gz, gzip -9n):
//! test_line_inside_sphere (TriangleSelector.cpp:52-87), test_line_inside_cylinder (91-142),
//! test_line_inside_capsule (145-154), is_point_inside_triangle (224-243),
//! is_mesh_point_not_clipped (2430-2434), is_circle_pointer_inside_triangle (2509-2529),
//! line_plane_intersection (2543-2558), ClippingPlane::is_active / is_mesh_point_clipped
//! (TriangleSelector.hpp:40-42). Results and the intersection point (bit for bit) must be equal.

use slicer::triangle_mesh::Vec3f;
use slicer::triangle_selector::{
    is_circle_pointer_inside_triangle, is_mesh_point_not_clipped, is_point_inside_triangle, line_plane_intersection,
    test_line_inside_capsule, test_line_inside_cylinder, test_line_inside_sphere, ClippingPlane, Transform3f,
};
use std::io::Read;

struct Tok<'a>(std::str::SplitWhitespace<'a>);

impl<'a> Tok<'a> {
    fn s(&mut self) -> &'a str {
        self.0.next().expect("token")
    }
    fn i(&mut self) -> i32 {
        self.s().parse().expect("integer")
    }
    fn b(&mut self) -> bool {
        self.i() != 0
    }
    fn f(&mut self) -> f32 {
        f32::from_bits(u32::from_str_radix(self.s(), 16).expect("float"))
    }
    fn v3(&mut self) -> Vec3f {
        Vec3f::new(self.f(), self.f(), self.f())
    }
    fn trafof(&mut self) -> Transform3f {
        let mut t = Transform3f::zeros();
        for r in 0..4 {
            for c in 0..4 {
                t[(r, c)] = self.f();
            }
        }
        t
    }
    fn arrow(&mut self) {
        assert_eq!(self.s(), "->");
    }
}

fn bits3(v: &Vec3f) -> [u32; 3] {
    [v.x.to_bits(), v.y.to_bits(), v.z.to_bits()]
}

#[test]
fn triangle_selector_free_functions_match_the_pin() {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/triangle_selector/vectors_free.txt.gz");
    let mut text = String::new();
    flate2::read::GzDecoder::new(std::fs::File::open(path).unwrap()).read_to_string(&mut text).unwrap();
    let mut counts = std::collections::BTreeMap::<&'static str, (usize, usize)>::new();
    let mut tally = |name: &'static str, ok: bool| {
        let e = counts.entry(name).or_insert((0, 0));
        e.0 += 1;
        if !ok {
            e.1 += 1;
        }
    };
    let mut skipped = 0usize;
    for line in text.lines() {
        let mut t = Tok(line.split_whitespace());
        match t.s() {
            "sphere" => {
                let (a, b, p, r) = (t.v3(), t.v3(), t.v3(), t.f());
                t.arrow();
                tally("test_line_inside_sphere", test_line_inside_sphere(&a, &b, &p, r) == t.b());
            }
            "cylinder" => {
                let (a, b, p, q, r) = (t.v3(), t.v3(), t.v3(), t.v3(), t.f());
                t.arrow();
                // TriangleSelector.cpp:93 asserts P != Q (compiled out at the pin, a debug_assert in
                // the port): the generator's degenerate axes are outside the contract.
                if p == q {
                    skipped += 1;
                    continue;
                }
                tally("test_line_inside_cylinder", test_line_inside_cylinder(&a, &b, &p, &q, r) == t.b());
            }
            "capsule" => {
                let (a, b, p, q, r) = (t.v3(), t.v3(), t.v3(), t.v3(), t.f());
                t.arrow();
                if p == q {
                    skipped += 1;
                    continue;
                }
                tally("test_line_inside_capsule", test_line_inside_capsule(&a, &b, &p, &q, r) == t.b());
            }
            "inside_triangle" => {
                let (pt, p1, p2, p3) = (t.v3(), t.v3(), t.v3(), t.v3());
                t.arrow();
                tally("is_point_inside_triangle", is_point_inside_triangle(&pt, &p1, &p2, &p3) == t.b());
            }
            "clip" => {
                let normal = t.v3();
                let offset = t.f();
                let clp = ClippingPlane { normal, offset };
                let pt = t.v3();
                t.arrow();
                tally("ClippingPlane::is_active", clp.is_active() == t.b());
                tally("ClippingPlane::is_mesh_point_clipped", clp.is_mesh_point_clipped(&pt) == t.b());
                tally("is_mesh_point_not_clipped", is_mesh_point_not_clipped(&pt, &clp) == t.b());
            }
            "circle_pointer" => {
                let (p1, p2, p3, center, dir) = (t.v3(), t.v3(), t.v3(), t.v3(), t.v3());
                let uniform = t.b();
                let trafo = t.trafof();
                t.arrow();
                tally(
                    "is_circle_pointer_inside_triangle",
                    is_circle_pointer_inside_triangle(&p1, &p2, &p3, &center, &dir, uniform, &trafo) == t.b(),
                );
            }
            "line_plane" => {
                let (a, b, o, n) = (t.v3(), t.v3(), t.v3(), t.v3());
                t.arrow();
                let mut out = Vec3f::new(0.0, 0.0, 0.0);
                let hit = line_plane_intersection(&a, &b, &o, &n, &mut out);
                let want_hit = t.b();
                let want = t.v3();
                tally("line_plane_intersection", hit == want_hit && bits3(&out) == bits3(&want));
            }
            other => panic!("unknown tag {other}"),
        }
    }
    for (name, (n, bad)) in &counts {
        eprintln!("{name}: {n} cases, {bad} mismatches");
    }
    eprintln!("cylinder/capsule cases with P == Q skipped: {skipped}");
    assert_eq!(counts.len(), 9);
    assert!(counts.values().all(|&(_, bad)| bad == 0), "{counts:?}");
}

/// TriangleSelector::Triangle's members (TriangleSelector.hpp:386-422, TriangleSelector.cpp:196-222):
/// the constructor, set_division, set_state / get_state, select_by_seed_fill /
/// unselect_by_seed_fill / is_selected_by_seed_fill, valid, is_split, number_of_split_sides and
/// special_side, replayed on the 2,000 call sequences of `gen.cpp triangle` (vectors_triangle.txt.gz).
#[test]
fn triangle_members_match_the_pin() {
    use slicer::triangle_selector::{EnforcerBlockerType, Triangle};
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/triangle_selector/vectors_triangle.txt.gz");
    let mut text = String::new();
    flate2::read::GzDecoder::new(std::fs::File::open(path).unwrap()).read_to_string(&mut text).unwrap();
    let (mut cases, mut steps, mut bad) = (0usize, 0usize, 0usize);
    for line in text.lines() {
        let mut t = Tok(line.split_whitespace());
        assert_eq!(t.s(), "triangle");
        let (a, b, c, src, st) = (t.i(), t.i(), t.i(), t.i(), t.i());
        t.arrow();
        let mut tr = Triangle::new(a, b, c, src, EnforcerBlockerType(st as i8));
        let mut ok = tr.verts_idxs == [t.i(), t.i(), t.i()] && tr.source_triangle == t.i();
        let after = |tr: &Triangle, t: &mut Tok| -> bool {
            let mut same = tr.valid() == t.b() && tr.is_split() == t.b() && tr.number_of_split_sides() == t.i();
            if tr.is_split() {
                same &= tr.special_side() == t.i();
            } else {
                same &= tr.get_state().0 as i32 == t.i();
                same &= tr.is_selected_by_seed_fill() == t.b();
            }
            same
        };
        ok &= after(&tr, &mut t);
        while let Some(op) = t.0.next() {
            match op {
                "D" => {
                    let (sides, special) = (t.i(), t.i());
                    tr.set_division(sides, special);
                }
                "N" => {}
                "T" => tr.set_state(EnforcerBlockerType(t.i() as i8)),
                "F" => tr.select_by_seed_fill(),
                "U" => tr.unselect_by_seed_fill(),
                other => panic!("unknown op {other}"),
            }
            ok &= after(&tr, &mut t);
            steps += 1;
        }
        cases += 1;
        bad += (!ok) as usize;
    }
    eprintln!("Triangle members: {cases} sequences, {steps} calls, {bad} mismatches");
    assert_eq!((cases, steps), (2000, 16000));
    assert_eq!(bad, 0);
}
