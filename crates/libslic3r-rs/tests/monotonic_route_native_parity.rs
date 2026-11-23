//! Native parity of `plan_monotonic_route` (FillRectilinear.cpp:3247-3590), the route of
//! `FillMonotonicLineWGapFill::fill_surface_extrusion`: where each gap fill is inserted between the
//! monotonic lines, the order at one position, the leftover order and the direction of every gap.
//! Vectors: tests/data/monotonic_route/gen.cpp (the pin's static function, its FillRectilinear.cpp included).

use slicer::fill::fill_rectilinear::{plan_monotonic_route, MonotonicRoute};
use slicer::geometry::{Point, Polyline, ThickPolyline};

fn pts(tok: &[&str]) -> Vec<Point> {
    let n: usize = tok[1].parse().unwrap();
    (0..n)
        .map(|i| Point::new(tok[2 + 2 * i].parse().unwrap(), tok[3 + 2 * i].parse().unwrap()))
        .collect()
}

fn hex(s: &str) -> f64 {
    // %a of a double: [-]0x1.<hex>p<exp> or 0x0p+0
    let neg = s.starts_with('-');
    let s = s.trim_start_matches('-').trim_start_matches("0x");
    let (mant, exp) = s.split_once('p').unwrap();
    let (int_part, frac) = mant.split_once('.').unwrap_or((mant, ""));
    let mut v = u64::from_str_radix(int_part, 16).unwrap() as f64;
    let mut scale = 1.0 / 16.0;
    for c in frac.chars() {
        v += c.to_digit(16).unwrap() as f64 * scale;
        scale /= 16.0;
    }
    let e: i32 = exp.parse().unwrap();
    let r = v * 2f64.powi(e);
    if neg { -r } else { r }
}

#[test]
fn plan_monotonic_route_native_parity() {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/monotonic_route/vectors.txt");
    let text = std::fs::read_to_string(path).expect("vectors");
    let mut lines_it = text.lines().peekable();
    let (mut cases, mut equal, mut gaps_total) = (0usize, 0usize, 0usize);
    let mut first_bad: Vec<String> = Vec::new();
    while let Some(head) = lines_it.next() {
        let h: Vec<&str> = head.split_whitespace().collect();
        assert_eq!(h[0], "case");
        let n_lines: usize = h[2].parse().unwrap();
        let n_gaps: usize = h[3].parse().unwrap();
        let link_max = hex(h[4]);
        let scan_angle = hex(h[5]) as f32;
        let n_ie: usize = h[6].parse().unwrap();
        let island_end: Vec<usize> = (0..n_ie).map(|i| h[7 + i].parse().unwrap()).collect();
        let mut lines: Vec<Polyline> = Vec::new();
        for _ in 0..n_lines {
            let t: Vec<&str> = lines_it.next().unwrap().split_whitespace().collect();
            assert_eq!(t[0], "L");
            lines.push(Polyline::from_points(pts(&t)));
        }
        let mut gaps: Vec<ThickPolyline> = Vec::new();
        for _ in 0..n_gaps {
            let t: Vec<&str> = lines_it.next().unwrap().split_whitespace().collect();
            assert_eq!(t[0], "G");
            let p = pts(&t);
            let w = vec![0.0; p.len()];
            gaps.push(ThickPolyline::from_points_and_widths(p, w));
        }
        let mut expected = MonotonicRoute { gaps_at: vec![Vec::new(); n_lines + 1], leftover: Vec::new() };
        let mut expected_gaps: Vec<Vec<Point>> = Vec::new();
        loop {
            let l = lines_it.next().unwrap();
            let t: Vec<&str> = l.split_whitespace().collect();
            match t[0] {
                "S" => {
                    let i: usize = t[1].parse().unwrap();
                    let k: usize = t[2].parse().unwrap();
                    expected.gaps_at[i] = (0..k).map(|j| t[3 + j].parse().unwrap()).collect();
                }
                "O" => {
                    let k: usize = t[1].parse().unwrap();
                    expected.leftover = (0..k).map(|j| t[2 + j].parse().unwrap()).collect();
                }
                "P" => expected_gaps.push(pts(&t)),
                "E" => break,
                other => panic!("unexpected {other}"),
            }
        }
        if n_gaps == 0 {
            // FillRectilinear.cpp:3260-3261: no gaps, an empty route.
            expected.gaps_at = vec![Vec::new(); n_lines + 1];
        }
        let got = plan_monotonic_route(&lines, &mut gaps, link_max, scan_angle, &island_end);
        let got_gaps: Vec<Vec<Point>> = gaps.iter().map(|g| g.points.clone()).collect();
        cases += 1;
        gaps_total += n_gaps;
        if got == expected && got_gaps == expected_gaps {
            equal += 1;
        } else if first_bad.len() < 5 {
            first_bad.push(format!("{head}\n got {:?} / {:?}\n pin {:?} / {:?}", got.gaps_at, got.leftover, expected.gaps_at, expected.leftover));
        }
    }
    eprintln!("plan_monotonic_route: {equal} of {cases} cases equal ({gaps_total} gaps)");
    for b in &first_bad {
        eprintln!("{b}");
    }
    assert_eq!(equal, cases);
}

/// Native parity of `append_monotonic_route` (FillRectilinear.cpp:3592-3639): the entities it assembles from
/// the route (the lines through extrusion_entities_append_paths_with_wipe, the gaps through
/// thick_polyline_to_extrusion_paths_2, a closed gap as a loop). Vectors: tests/data/monotonic_route/gen.cpp
/// run as `gen append` (the same cases with widths on the gaps), gzip -9n.
#[test]
fn append_monotonic_route_native_parity() {
    use slicer::extrusion_entity::{ExtrusionEntityType, ExtrusionPath, ExtrusionRole};
    use std::io::Read;
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/monotonic_route/vectors_append.txt.gz");
    let file = std::fs::File::open(path).expect("vectors");
    let mut text = String::new();
    flate2::read::GzDecoder::new(file).read_to_string(&mut text).expect("gzip");
    let flow = slicer::flow::Flow::new(0.42f32 as f64, 0.2f32 as f64, 0.4f32 as f64).unwrap();
    let mm3 = flow.mm3_per_mm().unwrap();
    let fmt_path = |p: &ExtrusionPath| -> String {
        let mut s = format!("p {:?} {:?} {:?} {}", p.mm3_per_mm, p.width as f32, p.height as f32, p.polyline.points.len());
        for q in &p.polyline.points {
            s += &format!(" {} {}", q.x, q.y);
        }
        s
    };
    let mut lines_it = text.lines();
    let (mut cases, mut equal, mut entities_total) = (0usize, 0usize, 0usize);
    let mut first_bad: Vec<String> = Vec::new();
    while let Some(head) = lines_it.next() {
        let h: Vec<&str> = head.split_whitespace().collect();
        assert_eq!(h[0], "case");
        let n_lines: usize = h[2].parse().unwrap();
        let n_gaps: usize = h[3].parse().unwrap();
        let link_max = hex(h[4]);
        let scan_angle = hex(h[5]) as f32;
        let n_ie: usize = h[6].parse().unwrap();
        let island_end: Vec<usize> = (0..n_ie).map(|i| h[7 + i].parse().unwrap()).collect();
        let mut lines: Vec<Polyline> = Vec::new();
        for _ in 0..n_lines {
            let t: Vec<&str> = lines_it.next().unwrap().split_whitespace().collect();
            lines.push(Polyline::from_points(pts(&t)));
        }
        let mut gaps: Vec<ThickPolyline> = Vec::new();
        for _ in 0..n_gaps {
            let t: Vec<&str> = lines_it.next().unwrap().split_whitespace().collect();
            assert_eq!(t[0], "G");
            let p = pts(&t);
            let wi = 2 + 2 * p.len();
            // the C++ width array (two per segment, scaled) in the pair layout of `widths`
            let w: Vec<f64> = if t.len() > wi && t[wi] == "W" {
                let k: usize = t[wi + 1].parse().unwrap();
                (0..k).map(|j| hex(t[wi + 2 + j])).collect()
            } else {
                Vec::new()
            };
            let mut tp = ThickPolyline::new();
            tp.points = p;
            tp.widths = w;
            gaps.push(tp);
        }
        // the pin's entities, formatted as the Rust ones below
        let mut expected: Vec<String> = Vec::new();
        loop {
            let l = lines_it.next().unwrap();
            if l == "E" {
                break;
            }
            let t: Vec<&str> = l.split_whitespace().collect();
            match t[0] {
                "X" => expected.push(format!("X {} {}", t[1], t[2])),
                "p" => {
                    let n: usize = t[4].parse().unwrap();
                    let mut s = format!("p {:?} {:?} {:?} {}", hex(t[1]), hex(t[2]) as f32, hex(t[3]) as f32, n);
                    for i in 0..n {
                        s += &format!(" {} {}", t[5 + 2 * i], t[6 + 2 * i]);
                    }
                    expected.push(s);
                }
                other => panic!("unexpected {other}"),
            }
        }
        let route = plan_monotonic_route(&lines, &mut gaps, link_max, scan_angle, &island_end);
        let mut dst: Vec<ExtrusionEntityType> = Vec::new();
        slicer::fill::fill_rectilinear::append_monotonic_route(
            &mut dst, &route, &mut lines, &gaps, ExtrusionRole::TopSolidInfill, mm3, 0.42f32, 0.2f32,
            ExtrusionRole::GapFill, &flow, 0.45f32,
        );
        let mut got: Vec<String> = Vec::new();
        for e in &dst {
            match e {
                ExtrusionEntityType::Path(p) => {
                    got.push("X path 1".to_string());
                    got.push(fmt_path(p));
                }
                ExtrusionEntityType::MultiPath(mp) => {
                    got.push(format!("X multi {}", mp.paths.len()));
                    got.extend(mp.paths.iter().map(|p| fmt_path(p)));
                }
                ExtrusionEntityType::Loop(l) => {
                    got.push(format!("X loop {}", l.paths.len()));
                    got.extend(l.paths.iter().map(|p| fmt_path(p)));
                }
                _ => got.push("X other 0".to_string()),
            }
        }
        cases += 1;
        entities_total += expected.iter().filter(|s| s.starts_with('X')).count();
        if got == expected {
            equal += 1;
        } else if first_bad.len() < 3 {
            let k = got.iter().zip(expected.iter()).position(|(a, b)| a != b).unwrap_or(got.len().min(expected.len()));
            first_bad.push(format!(
                "{head}\n first difference at item {k} of {} / {}\n got {:?}\n pin {:?}",
                got.len(),
                expected.len(),
                got.get(k),
                expected.get(k)
            ));
        }
    }
    eprintln!("append_monotonic_route: {equal} of {cases} cases equal ({entities_total} entities)");
    for b in &first_bad {
        eprintln!("{b}");
    }
    assert_eq!(equal, cases);
}
