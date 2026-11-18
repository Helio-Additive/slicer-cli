//! The functions of GCode/AvoidCrossingPerimeters.cpp that need neither a Layer nor a GCode,
//! against the pin.
//!
//! `tests/data/avoid_crossing_perimeters/vectors.txt.gz` is the output of `gen.cpp` beside it, which
//! includes the pin's AvoidCrossingPerimeters.cpp (its functions are `static`) and calls them on
//! generated polygons; the header comment there describes the lines.

use slicer::gcode::avoid_crossing_perimeters::native_test_api as acp;
use slicer::geometry::{ExPolygon, ExPolygons, Point, Polygon};
use std::io::Read;

fn read_vectors() -> String {
    let path = format!("{}/tests/data/avoid_crossing_perimeters/vectors.txt.gz", env!("CARGO_MANIFEST_DIR"));
    let file = std::fs::File::open(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
    let mut text = String::new();
    flate2::read::GzDecoder::new(file).read_to_string(&mut text).expect("gzip");
    text
}

struct Tokens<'a>(std::str::SplitWhitespace<'a>);

impl<'a> Tokens<'a> {
    fn tok(&mut self) -> &'a str {
        self.0.next().expect("token")
    }
    fn int(&mut self) -> i64 {
        self.tok().parse().expect("integer")
    }
    fn f32(&mut self) -> f32 {
        f32::from_bits(u32::from_str_radix(self.tok(), 16).expect("float bits"))
    }
    fn f64(&mut self) -> f64 {
        f64::from_bits(u64::from_str_radix(self.tok(), 16).expect("double bits"))
    }
    fn point(&mut self) -> Point {
        Point::new(self.int(), self.int())
    }
    fn ring(&mut self) -> Polygon {
        let n = self.int() as usize;
        Polygon::from_points((0..n).map(|_| self.point()).collect())
    }
    fn polygons(&mut self) -> Vec<Polygon> {
        let n = self.int() as usize;
        (0..n).map(|_| self.ring()).collect()
    }
    fn expolygons(&mut self) -> ExPolygons {
        let n = self.int() as usize;
        let mut out = ExPolygons::new();
        for _ in 0..n {
            let mut e = ExPolygon::new(self.ring());
            let holes = self.int() as usize;
            for _ in 0..holes {
                e.holes.push(self.ring());
            }
            out.push(e);
        }
        out
    }
}

fn ring_text(out: &mut String, p: &Polygon) {
    out.push_str(&format!(" {}", p.points.len()));
    for q in &p.points {
        out.push_str(&format!(" {} {}", q.x, q.y));
    }
}

fn expolygons_text(areas: &[ExPolygon]) -> String {
    let mut out = format!("{}", areas.len());
    for e in areas {
        ring_text(&mut out, &e.contour);
        out.push_str(&format!(" {}", e.holes.len()));
        for h in &e.holes {
            ring_text(&mut out, h);
        }
    }
    out
}

/// Runs every line of `kind` through `f` (left side tokens in, the right side's text out) and
/// returns the number of cases; fails on the first differences.
fn run(kind: &str, mut f: impl FnMut(&mut Tokens<'_>) -> String) -> usize {
    let text = read_vectors();
    let (mut cases, mut bad) = (0usize, 0usize);
    for line in text.lines() {
        let (lhs, want) = line.split_once(" -> ").expect("arrow");
        let mut it = Tokens(lhs.split_whitespace());
        if it.tok() != kind {
            continue;
        }
        let got = f(&mut it);
        assert!(it.0.next().is_none(), "{kind} case {cases}: input tokens left over");
        cases += 1;
        if got != want {
            bad += 1;
            if bad <= 3 {
                let cut = |s: &str| s.chars().take(700).collect::<String>();
                eprintln!("{kind} case {cases}:\n  got  {}\n  want {}\n  in   {}", cut(&got), cut(want), cut(lhs));
            }
        }
    }
    assert_eq!(bad, 0, "{kind}: {bad} of {cases} cases differ");
    cases
}

// AvoidCrossingPerimeters.cpp:1125-1151 (inner_offset), with resample_expolygons,
// precompute_expolygon_distances, contour_distance, offset_ex_poly_by_contour_widths and
// remove_too_small_holes under it.
#[test]
fn inner_offset_matches_the_pin() {
    let n = run("INNER", |it| {
        let offset_dis = it.f64();
        let input = it.expolygons();
        expolygons_text(&acp::inner_offset(&input, offset_dis))
    });
    assert_eq!(n, 260);
}

// AvoidCrossingPerimeters.cpp:1312-1331 (init_boundary, init_boundary_distances,
// precompute_polygon_distances).
#[test]
fn boundary_params_match_the_pin() {
    let n = run("PARAMS", |it| {
        let polys = it.polygons();
        let params = acp::boundary_params(&polys);
        let mut out = format!("{}", params.len());
        for p in &params {
            out.push_str(&format!(" {}", p.len()));
            for v in p {
                out.push_str(&format!(" {:08x}", v.to_bits()));
            }
        }
        out
    });
    assert_eq!(n, 100);
}

// AvoidCrossingPerimeters.cpp:118-190 (MinDistanceVisitor, get_closest_lines_in_radius).
#[test]
fn get_closest_lines_in_radius_matches_the_pin() {
    let mut found = 0usize;
    let n = run("CLOSE", |it| {
        let center = it.point();
        let radius = it.f32();
        let polys = it.polygons();
        let lines = acp::get_closest_lines_in_radius(&polys, &center, radius);
        found += lines.len();
        let mut out = format!("{}", lines.len());
        for (border, line, p) in &lines {
            out.push_str(&format!(" {border} {line} {} {}", p.x, p.y));
        }
        out
    });
    assert_eq!(n, 500);
    assert!(found > 200, "{found}");
}

// AvoidCrossingPerimeters.cpp:192-296 (extend_for_closest_lines), after AllIntersectionsVisitor
// (:46-90) and the sort of avoid_perimeters_inner (:585-590).
#[test]
fn extend_for_closest_lines_matches_the_pin() {
    let mut found = 0usize;
    let n = run("EXTEND", |it| {
        let start = it.point();
        let end = it.point();
        let radius = it.f32();
        let polys = it.polygons();
        let res = acp::extend_for_closest_lines(&polys, &start, &end, radius);
        found += res.len();
        let mut out = format!("{}", res.len());
        for (border, line, p, distance, do_not_remove) in &res {
            out.push_str(&format!(" {border} {line} {} {} {:08x} {}", p.x, p.y, distance.to_bits(), *do_not_remove as i32));
        }
        out
    });
    assert_eq!(n, 500);
    assert!(found > 500, "{found}");
}

// AvoidCrossingPerimeters.cpp:434-477 (simplify_travel, FirstIntersectionVisitor :92-116).
#[test]
fn simplify_travel_matches_the_pin() {
    let mut removed = 0usize;
    let n = run("SIMPL", |it| {
        let polys = it.polygons();
        let k = it.int() as usize;
        let travel: Vec<(Point, i32, bool)> = (0..k).map(|_| (it.point(), it.int() as i32, it.int() != 0)).collect();
        let res = acp::simplify_travel(&polys, &travel);
        removed += travel.len() - res.len();
        let mut out = format!("{}", res.len());
        for (p, border, do_not_remove) in &res {
            out.push_str(&format!(" {} {} {border} {}", p.x, p.y, *do_not_remove as i32));
        }
        out
    });
    assert_eq!(n, 500);
    assert!(removed > 500, "{removed}");
}

// AvoidCrossingPerimeters.cpp:718-770 (any_expolygon_contains, both overloads).
#[test]
fn any_expolygon_contains_matches_the_pin() {
    let mut yes = 0usize;
    let n = run("ANY", |it| {
        let ex_polygons = it.expolygons();
        let k = it.int() as usize;
        let pts: Vec<Point> = (0..k).map(|_| it.point()).collect();
        let res = acp::any_expolygon_contains(&ex_polygons, &pts);
        yes += res as usize;
        format!("{}", res as i32)
    });
    assert_eq!(n, 1500);
    assert!(yes > 50 && yes < 1450, "{yes}");
}
