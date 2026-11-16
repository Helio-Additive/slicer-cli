//! `get_intersections` against IntersectionPoints.cpp at the pin.
//!
//! `tests/data/intersection_points/vectors.txt.gz` is the output of `gen.cpp` beside it, which
//! links the engine's libslic3r and calls the five overloads on generated polygons, on grids
//! coarse enough that end points coincide and segments overlap (the header comment there says
//! how, and describes the lines). Each result is compared whole: count, order, line indices and
//! the point's doubles bit for bit. The SELF cases check Triangulation.cpp's
//! has_self_intersection, which is `!get_intersections(lines).empty()` over the edges.

use slicer::geometry::{ExPolygon, ExPolygons, Line, Lines, Point, Polygon, Polygons};
use slicer::intersection_points::{get_intersections, IntersectionsLines};
use std::io::Read;

fn read_vectors() -> String {
    let path = format!("{}/tests/data/intersection_points/vectors.txt.gz", env!("CARGO_MANIFEST_DIR"));
    let file = std::fs::File::open(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
    let mut text = String::new();
    flate2::read::GzDecoder::new(file).read_to_string(&mut text).expect("gzip");
    text
}

fn res(tag: &str, r: &IntersectionsLines) -> String {
    let mut s = format!(" {} {}", tag, r.len());
    for i in r {
        s += &format!(
            " {} {} {:016x} {:016x}",
            i.line_index1,
            i.line_index2,
            i.intersection.x.to_bits(),
            i.intersection.y.to_bits()
        );
    }
    s
}

fn to_lines(polys: &Polygons) -> Lines {
    let mut lines = Lines::new();
    for p in polys {
        lines.extend(p.lines());
    }
    lines
}

#[test]
fn get_intersections_matches_the_pin() {
    let text = read_vectors();
    let (mut cases, mut selfs, mut found, mut bad) = (0, 0, 0, 0);
    for line in text.lines() {
        let (lhs, want) = line.split_once(" ->").expect("arrow");
        let mut it = lhs.split_whitespace();
        let tag = it.next().unwrap();
        let mut num = || -> i64 { it.next().expect("number").parse().expect("integer") };
        let got = match tag {
            "CASE" => {
                cases += 1;
                let k = num() as usize;
                let mut polys = Polygons::new();
                for _ in 0..k {
                    let n = num() as usize;
                    let pts: Vec<Point> = (0..n).map(|_| Point::new(num(), num())).collect();
                    polys.push(Polygon::from_points(pts));
                }
                let mut ex = ExPolygon::new(polys[0].clone());
                ex.holes = polys[1..].to_vec();
                let exs: ExPolygons = polys.iter().map(|p| ExPolygon::new(p.clone())).collect();
                let l = get_intersections(&to_lines(&polys));
                found += l.len();
                res("L", &l)
                    + &res("P", &get_intersections(&polys[0]))
                    + &res("PS", &get_intersections(&polys))
                    + &res("E", &get_intersections(&ex))
                    + &res("ES", &get_intersections(&exs))
            }
            "SELF" => {
                selfs += 1;
                let n = num() as usize;
                let pts: Vec<Point> = (0..n).map(|_| Point::new(num(), num())).collect();
                let e = num() as usize;
                let lines: Lines = (0..e)
                    .map(|_| {
                        let (i, j) = (num() as usize, num() as usize);
                        Line::new(pts[i], pts[j])
                    })
                    .collect();
                format!(" {}", (!get_intersections(&lines).is_empty()) as i32)
            }
            t => panic!("unknown tag {t}"),
        };
        if got != want {
            bad += 1;
            if bad <= 5 {
                eprintln!("{lhs}\n   want{want}\n   got {got}");
            }
        }
    }
    assert_eq!((cases, selfs), (400, 200));
    assert!(found > 2000, "{found}");
    assert_eq!(bad, 0);
}
