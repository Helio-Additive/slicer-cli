//! Native parity for EdgeGrid::Grid under its C++ names (R948): the `create` overloads (EdgeGrid.cpp:28-140,
//! EdgeGrid.hpp:111) through `create_from_m_contours` (141-335) and `visit_intersect_line_impl`
//! (EdgeGrid.hpp:172-289), compared on the grid they leave (bbox, rows, cols, every cell's begin/end and the
//! cell data, by hash); `intersecting_edges` (1414-1450); `cell_inside_or_crossing` (EdgeGrid.hpp:420-431)
//! before and after `calculate_sdf` (672-980); the distance field; `closest_point_signed_distance`
//! (1047-1176). Vectors: tests/data/edge_grid/gen.cpp (vectors.txt.gz); EDGE_GRID_VECTORS overrides the path.

use slicer::edge_grid::{EdgeGrid, Grid, GridCreate};
use slicer::geometry::{BoundingBox, ExPolygon, Point, Polygon, Polyline};
use std::io::Read;

fn hexf(s: &str) -> f64 {
    // "%a" of the C++ printf: [-]0xh.hhhp[+-]d
    let (neg, s) = if let Some(r) = s.strip_prefix('-') { (true, r) } else { (false, s) };
    let s = s.strip_prefix("0x").expect("hex float");
    let (mant, exp) = s.split_once('p').expect("exponent");
    let exp: i32 = exp.parse().unwrap();
    let (ip, fp) = mant.split_once('.').unwrap_or((mant, ""));
    let mut m: u64 = u64::from_str_radix(ip, 16).unwrap();
    for c in fp.chars() {
        m = (m << 4) | c.to_digit(16).unwrap() as u64;
    }
    let e = exp - 4 * fp.len() as i32;
    let v = if m == 0 { 0.0 } else { (m as f64) * 2f64.powi(e) };
    // values with more than 53 significant bits do not occur ("%a" prints a double exactly)
    let v = if e < -1000 { (m as f64) * 2f64.powi(e + 100) * 2f64.powi(-100) } else { v };
    if neg { -v } else { v }
}

fn fnv(h: u64, v: u64) -> u64 {
    (h ^ v).wrapping_mul(0x100000001b3)
}
const FNV0: u64 = 0xcbf29ce484222325;

fn vectors() -> String {
    let path = std::env::var("EDGE_GRID_VECTORS")
        .unwrap_or_else(|_| format!("{}/tests/data/edge_grid/vectors.txt.gz", env!("CARGO_MANIFEST_DIR")));
    let raw = std::fs::read(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
    if path.ends_with(".gz") {
        let mut s = String::new();
        flate2::read::GzDecoder::new(&raw[..]).read_to_string(&mut s).unwrap();
        s
    } else {
        String::from_utf8(raw).unwrap()
    }
}

fn nums(line: &str, tag: &str) -> Vec<i64> {
    let mut it = line.split_whitespace();
    assert_eq!(it.next(), Some(tag), "line {line}");
    it.map(|v| v.parse().unwrap()).collect()
}

#[test]
fn edge_grid_native_parity() {
    let text = vectors();
    let mut it = text.lines().peekable();
    let mut cases = 0usize;
    // grid, intersecting_edges, cic0, sdf, cic1, closest point
    let mut ne = [0usize; 6];
    let mut counted = [0usize; 6];
    let mut first_bad: Vec<String> = Vec::new();
    while let Some(line) = it.next() {
        let h: Vec<&str> = line.split_whitespace().collect();
        assert_eq!(h[0], "case");
        let k: usize = h[1].parse().unwrap();
        let mode: usize = h[3].parse().unwrap();
        let res: i64 = h[5].parse().unwrap();
        let preset = h[7] == "1";
        let mut g: Grid = EdgeGrid::new();
        if preset {
            let b = nums(it.next().unwrap(), "bbox");
            g.set_bbox(BoundingBox::from_points_minmax(Point::new(b[0], b[1]), Point::new(b[2], b[3])));
        }
        let nl = nums(it.next().unwrap(), "lists")[0] as usize;
        let mut lists: Vec<(i64, Vec<Point>)> = Vec::new();
        for _ in 0..nl {
            let v = nums(it.next().unwrap(), "pl");
            let n = v[1] as usize;
            let pts: Vec<Point> = (0..n).map(|i| Point::new(v[2 + 2 * i], v[3 + 2 * i])).collect();
            lists.push((v[0], pts));
        }
        match mode {
            0 => {
                let polys: Vec<Polygon> = lists.iter().map(|l| Polygon::from_points(l.1.clone())).collect();
                <Grid as GridCreate<&[Polygon]>>::create(&mut g, &polys, res);
            }
            1 => {
                let polys: Vec<Polygon> = lists.iter().map(|l| Polygon::from_points(l.1.clone())).collect();
                let ptrs: Vec<&Polygon> = polys.iter().collect();
                <Grid as GridCreate<&[&Polygon]>>::create(&mut g, &ptrs, res);
            }
            2 => {
                let v: Vec<Vec<Point>> = lists.iter().map(|l| l.1.clone()).collect();
                <Grid as GridCreate<(&[Vec<Point>], bool)>>::create(&mut g, (&v, true), res);
            }
            3 => {
                let v: Vec<Vec<Point>> = lists.iter().map(|l| l.1.clone()).collect();
                <Grid as GridCreate<&[Vec<Point>]>>::create(&mut g, &v, res);
            }
            4 => {
                let mut polys = Vec::new();
                let mut pls = Vec::new();
                for l in &lists {
                    if l.0 != 0 {
                        pls.push(Polyline::from_points(l.1.clone()));
                    } else {
                        polys.push(Polygon::from_points(l.1.clone()));
                    }
                }
                <Grid as GridCreate<(&[Polygon], &[Polyline])>>::create(&mut g, (&polys, &pls), res);
            }
            5 => {
                let mut e = ExPolygon::new(Polygon::new());
                for l in &lists {
                    if l.0 == 0 {
                        e.contour = Polygon::from_points(l.1.clone());
                    } else {
                        e.holes.push(Polygon::from_points(l.1.clone()));
                    }
                }
                <Grid as GridCreate<&ExPolygon>>::create(&mut g, &e, res);
            }
            6 => {
                let mut es: Vec<ExPolygon> = Vec::new();
                for l in &lists {
                    if l.0 == 0 || es.is_empty() {
                        es.push(ExPolygon::new(Polygon::from_points(l.1.clone())));
                    } else {
                        es.last_mut().unwrap().holes.push(Polygon::from_points(l.1.clone()));
                    }
                }
                <Grid as GridCreate<&[ExPolygon]>>::create(&mut g, &es, res);
            }
            _ => unreachable!(),
        }
        let mut bad = |i: usize, what: String, ne: &mut [usize; 6]| {
            ne[i] += 1;
            if first_bad.len() < 12 {
                first_bad.push(format!("case {k} mode {mode}: {what}"));
            }
        };
        // grid
        let w: Vec<String> = it.next().unwrap().split_whitespace().map(String::from).collect();
        assert_eq!(w[0], "grid");
        let cells = g.cells_begin_end();
        let hc = cells.iter().fold(FNV0, |h, c| fnv(fnv(h, c.0 as u64), c.1 as u64));
        let hd = g.cell_data().iter().fold(FNV0, |h, d| fnv(fnv(h, d.0 as u64), d.1 as u64));
        let got = format!(
            "{} {} {} {} {} {} {} {} {} {}",
            g.bbox().min.x, g.bbox().min.y, g.bbox().max.x, g.bbox().max.y, g.rows(), g.cols(), cells.len(), g.cell_data().len(), hc, hd
        );
        counted[0] += 1;
        if got != w[1..].join(" ") {
            bad(0, format!("grid got {got} want {}", w[1..].join(" ")), &mut ne);
        }
        // intersecting_edges
        let w: Vec<&str> = it.next().unwrap().split_whitespace().collect();
        assert_eq!(w[0], "ie");
        let ie = g.intersecting_edges();
        let hie = ie.iter().fold(FNV0, |h, e| fnv(fnv(fnv(fnv(h, e.0 .0 as u64), e.0 .1 as u64), e.1 .0 as u64), e.1 .1 as u64));
        counted[1] += 1;
        if format!("{} {}", ie.len(), hie) != format!("{} {}", w[1], w[2]) {
            bad(1, format!("intersecting_edges got {} {hie} want {} {}", ie.len(), w[1], w[2]), &mut ne);
        }
        // cell_inside_or_crossing before calculate_sdf
        let check_cic = |line: &str, tag: &str, g: &Grid| -> bool {
            let v = nums(line, tag);
            v.chunks(3).all(|q| g.cell_inside_or_crossing(q[0] as i32, q[1] as i32) == (q[2] == 1))
        };
        counted[2] += 1;
        if !check_cic(it.next().unwrap(), "cic0", &g) {
            bad(2, "cell_inside_or_crossing before sdf".into(), &mut ne);
        }
        if it.peek().map_or(false, |l| l.starts_with("sdf ")) {
            g.calculate_sdf();
            let w: Vec<&str> = it.next().unwrap().split_whitespace().collect();
            let sdf = g.signed_distance_field();
            let hs = sdf.iter().fold(FNV0, |h, f| fnv(h, f.to_bits() as u64));
            counted[3] += 1;
            if format!("{} {}", sdf.len(), hs) != format!("{} {}", w[1], w[2]) {
                bad(3, format!("sdf got {} {hs} want {} {}", sdf.len(), w[1], w[2]), &mut ne);
            }
            counted[4] += 1;
            if !check_cic(it.next().unwrap(), "cic1", &g) {
                bad(4, "cell_inside_or_crossing after sdf".into(), &mut ne);
            }
            let w: Vec<&str> = it.next().unwrap().split_whitespace().collect();
            assert_eq!(w[0], "cp");
            for q in w[1..].chunks(7) {
                let pt = Point::new(q[0].parse().unwrap(), q[1].parse().unwrap());
                let radius: i64 = q[2].parse().unwrap();
                let r = g.closest_point_signed_distance(&pt, radius);
                let (ci, si) = if r.is_valid() { (r.contour_idx as i64, r.start_point_idx as i64) } else { (-1, -1) };
                counted[5] += 1;
                let (wd, wt) = (hexf(q[5]), hexf(q[6]));
                let ok = ci == q[3].parse::<i64>().unwrap()
                    && si == q[4].parse::<i64>().unwrap()
                    && r.distance.to_bits() == wd.to_bits()
                    && r.t.to_bits() == wt.to_bits();
                if !ok {
                    bad(5, format!("closest_point_signed_distance({pt:?}, {radius}) got {ci} {si} {:e} {:e} want {} {} {:e} {:e}", r.distance, r.t, q[3], q[4], wd, wt), &mut ne);
                }
            }
        }
        cases += 1;
    }
    eprintln!(
        "edge_grid_native_parity: {cases} cases; differ / compared: create (grid) {}/{}, intersecting_edges {}/{}, cell_inside_or_crossing {}/{} (+ after sdf {}/{}), calculate_sdf {}/{}, closest_point_signed_distance {}/{}",
        ne[0], counted[0], ne[1], counted[1], ne[2], counted[2], ne[4], counted[4], ne[3], counted[3], ne[5], counted[5]
    );
    for b in &first_bad {
        eprintln!("  {b}");
    }
    assert!(cases > 0);
    // calculate_sdf follows the pin with EDGE_GRID_SDF_PIN (on by default); =0 is the earlier arm (reported).
    if !slicer::faithful_gate("EDGE_GRID_SDF_PIN") {
        ne[3] = 0;
    }
    assert_eq!(ne, [0usize; 6], "first differences: {first_bad:#?}");
}
