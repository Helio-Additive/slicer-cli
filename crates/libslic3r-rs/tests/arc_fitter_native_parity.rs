//! ArcFitter::do_arc_fitting_and_simplify (ArcFitter.cpp:9-152, with Circle.cpp's
//! try_create_circle / try_create_arc) against the pin, bit for bit. Vectors from
//! tests/data/arc_fitter/gen.cpp: 1,500 polylines of arcs and straight runs with up to 40
//! scaled units of noise, tolerance 1000 / 2500 / 5000 / 10000 scaled units (1,169 arcs); ArcSegment::is_point_inside / clip_start / clip_end / reverse /
//! split_at (Circle.cpp:203-254) on every fitted arc at a point on its circle (764 inside, 739
//! clips); Circle::try_create_circle (Circle.cpp:15-55) on 1,476 point triples (666 found); and
//! ArcSegment::calc_arc_radian / calc_arc_radius / calc_arc_length (Circle.cpp:479-514) on 2,000
//! float cases.
use slicer::arc_fitter::{ArcFitter, EMovePathType, PathFittingData};
use slicer::circle::{ArcDirection, ArcSegment, Circle};
use slicer::geometry::Point;
use std::io::Read;

fn read_vectors() -> String {
    let path = format!("{}/tests/data/arc_fitter/vectors.txt.gz", env!("CARGO_MANIFEST_DIR"));
    let file = std::fs::File::open(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
    let mut text = String::new();
    flate2::read::GzDecoder::new(file).read_to_string(&mut text).expect("gzip");
    text
}

fn points(t: &[&str]) -> Vec<Point> {
    t.chunks(2)
        .map(|c| Point::new(c[0].parse().unwrap(), c[1].parse().unwrap()))
        .collect()
}

fn path_type(t: EMovePathType) -> i32 {
    match t {
        EMovePathType::NoopMove => 0,
        EMovePathType::LinearMove => 1,
        EMovePathType::ArcMoveCw => 2,
        EMovePathType::ArcMoveCcw => 3,
    }
}

fn direction(d: ArcDirection) -> i32 {
    match d {
        ArcDirection::Unknown => 0,
        ArcDirection::CounterClockwise => 1,
        ArcDirection::Clockwise => 2,
    }
}

fn format(work: &[Point], result: &[PathFittingData]) -> String {
    let mut s = format!("{}", work.len());
    for p in work {
        s += &format!(" {} {}", p.x(), p.y());
    }
    s += &format!(" | {}", result.len());
    for d in result {
        s += &format!(" {} {} {}", d.start_point_index, d.end_point_index, path_type(d.path_type));
        if d.is_arc_move() {
            let a = &d.arc_data;
            s += &format!(
                " {} {} {:016x} {:016x} {:016x} {:016x} {:016x} {}",
                a.circle.center.x(),
                a.circle.center.y(),
                a.circle.radius.to_bits(),
                a.length.to_bits(),
                a.angle_radians.to_bits(),
                a.polar_start_theta.to_bits(),
                a.polar_end_theta.to_bits(),
                direction(a.direction)
            );
        }
    }
    s
}

fn hx(t: &str) -> f64 {
    f64::from_bits(u64::from_str_radix(t, 16).unwrap())
}

fn arc_in(t: &[&str]) -> ArcSegment {
    let i = |k: usize| t[k].parse::<i64>().unwrap();
    let mut a = ArcSegment::default();
    a.is_arc = t[0] == "1";
    a.circle = Circle { center: Point::new(i(1), i(2)), radius: hx(t[3]) };
    a.start_point = Point::new(i(4), i(5));
    a.end_point = Point::new(i(6), i(7));
    a.direction = match t[8] {
        "0" => ArcDirection::Unknown,
        "1" => ArcDirection::CounterClockwise,
        _ => ArcDirection::Clockwise,
    };
    a.length = hx(t[9]);
    a.angle_radians = hx(t[10]);
    a.polar_start_theta = hx(t[11]);
    a.polar_end_theta = hx(t[12]);
    a
}

fn arc_out(a: &ArcSegment) -> String {
    format!(
        " {} {} {} {:016x} {} {} {} {} {} {:016x} {:016x} {:016x} {:016x}",
        i32::from(a.is_arc),
        a.circle.center.x(),
        a.circle.center.y(),
        a.circle.radius.to_bits(),
        a.start_point.x(),
        a.start_point.y(),
        a.end_point.x(),
        a.end_point.y(),
        direction(a.direction),
        a.length.to_bits(),
        a.angle_radians.to_bits(),
        a.polar_start_theta.to_bits(),
        a.polar_end_theta.to_bits()
    )
}

#[test]
fn do_arc_fitting_and_simplify_matches_the_pin() {
    let text = read_vectors();
    let (mut cases, mut arcs, mut bad) = (0usize, 0usize, 0usize);
    let (mut triples, mut circles, mut segments, mut radians) = (0usize, 0usize, 0usize, 0usize);
    for line in text.lines() {
        let (lhs, want) = line.split_once(" -> ").expect("arrow");
        let a: Vec<&str> = lhs.split_whitespace().collect();
        if a[0] == "R" {
            // ArcSegment::calc_arc_radian / calc_arc_radius / calc_arc_length, Circle.cpp:479-514.
            let f = |k: usize| f32::from_bits(u32::from_str_radix(a[k], 16).unwrap());
            let st = nalgebra::Vector3::new(f(1), f(2), f(3));
            let en = nalgebra::Vector3::new(f(4), f(5), f(6));
            let ce = nalgebra::Vector3::new(f(7), f(8), f(9));
            let ccw = a[10] == "1";
            let got = format!(
                "{:08x} {:08x} {:08x}",
                ArcSegment::calc_arc_radian(st, en, ce, ccw).to_bits(),
                ArcSegment::calc_arc_radius(st, ce).to_bits(),
                ArcSegment::calc_arc_length(st, en, ce, ccw).to_bits()
            );
            radians += 1;
            if got != want {
                bad += 1;
                if bad <= 5 {
                    eprintln!("{lhs}\n   want {want}\n   got  {got}");
                }
            }
            continue;
        }
        if a[0] == "S" {
            // ArcSegment::is_point_inside / clip_start / clip_end / reverse / split_at, Circle.cpp:203-254.
            let arc = arc_in(&a[1..14]);
            let p = Point::new(a[14].parse().unwrap(), a[15].parse().unwrap());
            let mut got = format!("{}", i32::from(arc.is_point_inside(p)));
            let mut cs = arc.clone();
            let ok = cs.clip_start(p);
            got += &format!(" {}{}", i32::from(ok), arc_out(&cs));
            let mut ce = arc.clone();
            let ok = ce.clip_end(p);
            got += &format!(" {}{}", i32::from(ok), arc_out(&ce));
            let mut rv = arc.clone();
            let ok = rv.reverse();
            got += &format!(" {}{}", i32::from(ok), arc_out(&rv));
            match arc.split_at(p) {
                Some((s1, s2)) => got += &format!(" 1{}{}", arc_out(&s1), arc_out(&s2)),
                None => got += " 0",
            }
            segments += 1;
            if got != want {
                bad += 1;
                if bad <= 5 {
                    eprintln!("{lhs}\n   want {want}\n   got  {got}");
                }
            }
            continue;
        }
        if a[0] == "T" {
            // Circle::try_create_circle(p1, p2, p3, max_radius, new_circle), Circle.cpp:15-55.
            let p = points(&a[1..7]);
            let max_radius = f64::from_bits(u64::from_str_radix(a[7], 16).unwrap());
            let mut c = Circle::default();
            let ok = Circle::try_create_circle(p[0], p[1], p[2], max_radius, &mut c);
            let got = if ok {
                format!("1 {} {} {:016x}", c.center.x(), c.center.y(), c.radius.to_bits())
            } else {
                "0".to_string()
            };
            triples += 1;
            circles += usize::from(ok);
            if got != want {
                bad += 1;
                if bad <= 5 {
                    eprintln!("{lhs}\n   want {want}\n   got  {got}");
                }
            }
            continue;
        }
        assert_eq!(a[0], "C");
        let tol: f64 = a[1].parse().unwrap();
        let n: usize = a[2].parse().unwrap();
        let mut work = points(&a[3..3 + 2 * n]);
        let mut result = Vec::new();
        ArcFitter::do_arc_fitting_and_simplify(&mut work, &mut result, tol).expect("fit");
        arcs += result.iter().filter(|d| d.is_arc_move()).count();
        cases += 1;
        let got = format(&work, &result);
        if got != want {
            bad += 1;
            if bad <= 5 {
                eprintln!("case {cases} tol {tol} n {n}\n   want {want}\n   got  {got}");
            }
        }
    }
    eprintln!("cases {cases}, arcs {arcs}, segments {segments}, triples {triples}, circles {circles}, calc_arc {radians}, mismatches {bad}");
    assert_eq!(cases, 1500);
    assert!(arcs > 1100, "{arcs}");
    assert!(triples > 1400 && circles > 500, "{triples} {circles}");
    assert_eq!(segments, arcs);
    assert_eq!(radians, 2000);
    assert_eq!(bad, 0);
}
