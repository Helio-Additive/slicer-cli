//! Polyline::split_at (Polyline.cpp:223-265, under its C++ name through `PolylineSplitAt`),
//! split_at_index (268-298) and split_at_length (300-344) with the fitting result they carry
//! (split_fitting_result_before_index / after_index, ArcSegment::clip_start / clip_end),
//! Polyline::simplify_by_fitting_arc (152-156), clip_end, clip_start, reverse, append, extend_end,
//! extend_start and the free
//! remove_same_neighbor(Polyline&) / remove_same_neighbor(Polylines&) (534-556, through
//! `RemoveSameNeighbor`) against the pin, bit for bit. Vectors from tests/data/polyline_split/gen.cpp:
//! 1,500 polylines of arcs and straight runs with up to 30 scaled units of noise and repeated points,
//! three quarters of them through simplify_by_fitting_arc (tolerance 1,000 / 2,500 / 5,000 scaled
//! units); per polyline four split_at points (a vertex, two near the input, one far away), three
//! split_at_index, four split_at_length, two clip_end and two clip_start (Polyline.cpp:45-108), one
//! reverse (34-50), one append(const Polyline&) and one append(Polyline&&) of the halves of a split
//! (358-404, append_fitting_result_after_append_polyline), one extend_end, one extend_start (111-116) and
//! one remove_same_neighbor; then 300 vectors of short polylines for remove_same_neighbor(Polylines&) and
//! polylines_merge (Polyline.hpp:230-247) of their first two, in the four flag combinations.
use slicer::arc_fitter::{EMovePathType, PathFittingData};
use slicer::circle::{ArcDirection, ArcSegment};
use slicer::geometry::{polylines_merge, Point, Polyline, PolylineSplitAt, RemoveSameNeighbor};
use std::io::Read;
use std::str::SplitWhitespace;

fn read_vectors() -> String {
    let path = std::env::var("POLYLINE_SPLIT_VECTORS")
        .unwrap_or_else(|_| format!("{}/tests/data/polyline_split/vectors.txt.gz", env!("CARGO_MANIFEST_DIR")));
    let file = std::fs::File::open(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
    let mut text = String::new();
    if path.ends_with(".gz") {
        flate2::read::GzDecoder::new(file).read_to_string(&mut text).expect("gzip");
    } else {
        std::io::BufReader::new(file).read_to_string(&mut text).expect("read");
    }
    text
}

fn path_type_code(t: EMovePathType) -> i32 {
    match t {
        EMovePathType::NoopMove => 0,
        EMovePathType::LinearMove => 1,
        EMovePathType::ArcMoveCw => 2,
        EMovePathType::ArcMoveCcw => 3,
    }
}

fn direction_code(d: ArcDirection) -> i32 {
    match d {
        ArcDirection::Unknown => 0,
        ArcDirection::CounterClockwise => 1,
        ArcDirection::Clockwise => 2,
    }
}

/// The generator's ` P n x y ... | k (s e type [arc])...` for a polyline.
fn fmt_pl(pl: &Polyline) -> String {
    let mut s = format!(" P {}", pl.points.len());
    for p in &pl.points {
        s += &format!(" {} {}", p.x(), p.y());
    }
    s += &format!(" | {}", pl.fitting_result.len());
    for d in &pl.fitting_result {
        s += &format!(" {} {} {}", d.start_point_index, d.end_point_index, path_type_code(d.path_type));
        if d.is_arc_move() {
            let a = &d.arc_data;
            s += &format!(
                " {} {} {} {} {} {} {} {:016x} {:016x} {:016x} {:016x} {:016x} {}",
                a.is_arc as i32,
                a.start_point.x(),
                a.start_point.y(),
                a.end_point.x(),
                a.end_point.y(),
                a.circle.center.x(),
                a.circle.center.y(),
                a.circle.radius.to_bits(),
                a.length.to_bits(),
                a.angle_radians.to_bits(),
                a.polar_start_theta.to_bits(),
                a.polar_end_theta.to_bits(),
                direction_code(a.direction)
            );
        }
    }
    s
}

fn bits(t: &str) -> f64 {
    f64::from_bits(u64::from_str_radix(t, 16).unwrap())
}

/// Reads one ` P ...` polyline back WITH its fitting result, every field as the pin printed it.
fn parse_pl(t: &mut SplitWhitespace<'_>) -> Polyline {
    assert_eq!(t.next(), Some("P"));
    let n: usize = t.next().unwrap().parse().unwrap();
    let mut pts = Vec::with_capacity(n);
    for _ in 0..n {
        let x: i64 = t.next().unwrap().parse().unwrap();
        let y: i64 = t.next().unwrap().parse().unwrap();
        pts.push(Point::new(x, y));
    }
    assert_eq!(t.next(), Some("|"));
    let k: usize = t.next().unwrap().parse().unwrap();
    let mut fitting_result = Vec::with_capacity(k);
    for _ in 0..k {
        let s_idx: usize = t.next().unwrap().parse().unwrap();
        let e_idx: usize = t.next().unwrap().parse().unwrap();
        let ty: i32 = t.next().unwrap().parse().unwrap();
        let path_type = match ty {
            0 => EMovePathType::NoopMove,
            1 => EMovePathType::LinearMove,
            2 => EMovePathType::ArcMoveCw,
            _ => EMovePathType::ArcMoveCcw,
        };
        let mut arc = ArcSegment::default();
        if ty == 2 || ty == 3 {
            let v: Vec<&str> = (0..13).map(|_| t.next().unwrap()).collect();
            arc.is_arc = v[0] == "1";
            arc.start_point = Point::new(v[1].parse().unwrap(), v[2].parse().unwrap());
            arc.end_point = Point::new(v[3].parse().unwrap(), v[4].parse().unwrap());
            arc.circle.center = Point::new(v[5].parse().unwrap(), v[6].parse().unwrap());
            arc.circle.radius = bits(v[7]);
            arc.length = bits(v[8]);
            arc.angle_radians = bits(v[9]);
            arc.polar_start_theta = bits(v[10]);
            arc.polar_end_theta = bits(v[11]);
            arc.direction = match v[12] {
                "1" => ArcDirection::CounterClockwise,
                "2" => ArcDirection::Clockwise,
                _ => ArcDirection::Unknown,
            };
        }
        fitting_result.push(PathFittingData::new(s_idx, e_idx, path_type, arc));
    }
    let mut pl = Polyline::from_points(pts);
    pl.fitting_result = fitting_result;
    pl
}

#[test]
fn polyline_split_native_parity() {
    let text = read_vectors();
    let tols = [1000.0, 2500.0, 5000.0, 10000.0];
    let mut cur = Polyline::new();
    let mut raw = Polyline::new();
    let mut counts = [0usize; 13]; // cases, fit, split_at, split_at_index, split_at_length, extend_end, rsn, rsn(Polylines), clip_end/start, reverse, append, extend_start, polylines_merge
    let mut bad: Vec<String> = Vec::new();
    let mut check = |bad: &mut Vec<String>, what: &str, got: String, want: &str| {
        if got != want {
            bad.push(format!("{what}\n  pin  {want}\n  rust {got}"));
        }
    };
    for line in text.lines() {
        let (head, rest) = line.split_once(' ').unwrap_or((line, ""));
        match head {
            "C" => {
                cur = parse_pl(&mut rest.split_whitespace());
                counts[0] += 1;
            }
            "R" => {
                raw = parse_pl(&mut rest.split_whitespace());
                // Polyline.cpp:152-156 on the input, for the three quarters the generator fitted
                let c = counts[0] - 1;
                if c % 4 != 3 {
                    let mut fit = Polyline::from_points(raw.points.clone());
                    fit.simplify_by_fitting_arc(tols[c % 4]);
                    counts[1] += 1;
                    check(&mut bad, &format!("case {c} simplify_by_fitting_arc"), fmt_pl(&fit), &fmt_pl(&cur));
                }
            }
            "SP" => {
                let (q, want) = rest.split_once(" -> ").unwrap();
                let mut qi = q.split_whitespace().map(|v| v.parse::<i64>().unwrap());
                let mut io = Point::new(qi.next().unwrap(), qi.next().unwrap());
                let (mut p1, mut p2) = (Polyline::new(), Polyline::new());
                PolylineSplitAt::split_at(&cur, &mut io, &mut p1, &mut p2);
                counts[2] += 1;
                let got = format!("{} {}{}{}", io.x(), io.y(), fmt_pl(&p1), fmt_pl(&p2));
                check(&mut bad, &format!("case {} split_at {q}", counts[0] - 1), got, want);
            }
            "SI" => {
                let (q, want) = rest.split_once(" -> ").unwrap();
                let idx: usize = q.trim().parse().unwrap();
                let (mut p1, mut p2) = (Polyline::new(), Polyline::new());
                let ok = cur.split_at_index(idx, &mut p1, &mut p2);
                counts[3] += 1;
                let got = format!("{}{}{}", ok as i32, fmt_pl(&p1), fmt_pl(&p2));
                check(&mut bad, &format!("case {} split_at_index {q}", counts[0] - 1), got, want);
            }
            "SL" => {
                let (q, want) = rest.split_once(" -> ").unwrap();
                let (mut p1, mut p2) = (Polyline::new(), Polyline::new());
                let ok = cur.split_at_length(bits(q.trim()), &mut p1, &mut p2);
                counts[4] += 1;
                let got = format!("{}{}{}", ok as i32, fmt_pl(&p1), fmt_pl(&p2));
                check(&mut bad, &format!("case {} split_at_length {q}", counts[0] - 1), got, want);
            }
            "EE" => {
                let (q, want) = rest.split_once(" ->").unwrap();
                let mut e = Polyline::from_points(raw.points.clone());
                e.extend_end(bits(q.trim()));
                counts[5] += 1;
                check(&mut bad, &format!("case {} extend_end {q}", counts[0] - 1), fmt_pl(&Polyline::from_points(e.points)), want);
            }
            "RN" => {
                let want = rest.strip_prefix("-> ").unwrap();
                let mut r = Polyline::from_points(raw.points.clone());
                let erased = RemoveSameNeighbor::remove_same_neighbor(&mut r);
                counts[6] += 1;
                let got = format!("{}{}", erased as i32, fmt_pl(&Polyline::from_points(r.points)));
                check(&mut bad, &format!("case {} remove_same_neighbor", counts[0] - 1), got, want);
            }
            "RNS" => {
                let (q, want) = rest.split_once(" -> ").unwrap();
                let mut t = q.split_whitespace();
                let n: usize = t.next().unwrap().parse().unwrap();
                let mut pls: Vec<Polyline> = (0..n).map(|_| parse_pl(&mut t)).collect();
                let erased = RemoveSameNeighbor::remove_same_neighbor(&mut pls);
                counts[7] += 1;
                let mut got = format!("{} {}", erased as i32, pls.len());
                for p in &pls {
                    got += &fmt_pl(p);
                }
                check(&mut bad, &format!("remove_same_neighbor(Polylines) {q}"), got, want);
            }
            "CE" | "CS" => {
                let (q, want) = rest.split_once(" ->").unwrap();
                let mut c = cur.clone();
                if head == "CE" {
                    c.clip_end(bits(q.trim()));
                } else {
                    c.clip_start(bits(q.trim()));
                }
                counts[8] += 1;
                check(&mut bad, &format!("case {} {head} {q}", counts[0] - 1), fmt_pl(&c), want);
            }
            "RV" => {
                let want = rest.strip_prefix("->").unwrap();
                let mut c = cur.clone();
                c.reverse();
                counts[9] += 1;
                check(&mut bad, &format!("case {} reverse", counts[0] - 1), fmt_pl(&c), want);
            }
            "AP" | "AM" => {
                let (q, want) = rest.split_once(" ->").unwrap();
                let mut t = q.split_whitespace();
                let mut a = parse_pl(&mut t);
                let b = parse_pl(&mut t);
                if head == "AP" {
                    a.append(&b);
                } else {
                    a.append_move(b);
                }
                counts[10] += 1;
                check(&mut bad, &format!("case {} {head}", counts[0] - 1), fmt_pl(&a), want);
            }
            "ES" => {
                let (q, want) = rest.split_once(" ->").unwrap();
                let mut e = Polyline::from_points(raw.points.clone());
                e.extend_start(bits(q.trim()));
                counts[11] += 1;
                check(&mut bad, &format!("case {} extend_start {q}", counts[0] - 1), fmt_pl(&Polyline::from_points(e.points)), want);
            }
            "PM" => {
                let (q, want) = rest.split_once(" ->").unwrap();
                let mut t = q.split_whitespace();
                let dst_first = t.next() == Some("1");
                let src_first = t.next() == Some("1");
                let mut dst = parse_pl(&mut t).points;
                let src = parse_pl(&mut t).points;
                polylines_merge(&mut dst, dst_first, src, src_first);
                counts[12] += 1;
                check(&mut bad, &format!("polylines_merge {q}"), fmt_pl(&Polyline::from_points(dst)), want);
            }
            _ => panic!("unknown vector line {head}"),
        }
    }
    eprintln!(
        "polyline_split_native_parity: {} polylines; simplify_by_fitting_arc {}, split_at {}, split_at_index {}, split_at_length {}, extend_end {}, remove_same_neighbor {}, remove_same_neighbor(Polylines) {}, clip_end + clip_start {}, reverse {}, append (copy + move) {}, extend_start {}, polylines_merge {}; {} mismatches",
        counts[0], counts[1], counts[2], counts[3], counts[4], counts[5], counts[6], counts[7], counts[8], counts[9], counts[10], counts[11], counts[12], bad.len()
    );
    for b in bad.iter().take(6) {
        eprintln!("{b}");
    }
    assert!(counts.iter().all(|&c| c > 0), "missing vectors: {counts:?}");
    assert!(bad.is_empty(), "{} mismatches", bad.len());
}
