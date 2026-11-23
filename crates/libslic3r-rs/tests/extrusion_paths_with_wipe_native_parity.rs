//! extrusion_entities_append_paths_with_wipe (ExtrusionEntity.hpp:671-724) against the pin:
//! tests/data/extrusion_paths_with_wipe/gen.cpp calls the pin's function on LCG polylines (hops
//! around 3 * width, gap compensation ratios 0 / 0.3 / 0.45 / 0.75, some invalid polylines) and
//! prints the multipaths it appends; the Rust (EXTRUSION_PATHS_WITH_WIPE_PIN and
//! PATHS_WITH_WIPE_SCALED_PIN on, their defaults) must append the same.
use slicer::extrusion_entity::{extrusion_entities_append_paths_with_wipe, ExtrusionEntityType, ExtrusionRole};
use slicer::geometry::{Point, Polyline};

fn hex_f32(t: &str) -> f32 {
    // %a of a float promoted to double
    let t = t.trim();
    let neg = t.starts_with('-');
    let t = t.trim_start_matches('-').trim_start_matches("0x");
    let (mant, exp) = t.split_once('p').unwrap();
    let (ip, fp) = mant.split_once('.').unwrap_or((mant, ""));
    let mut m = u64::from_str_radix(ip, 16).unwrap() as f64;
    let mut scale = 1.0 / 16.0;
    for ch in fp.chars() {
        m += ch.to_digit(16).unwrap() as f64 * scale;
        scale /= 16.0;
    }
    let v = m * 2f64.powi(exp.parse::<i32>().unwrap());
    (if neg { -v } else { v }) as f32
}

#[test]
fn append_paths_with_wipe_matches_the_pin() {
    let path = format!("{}/tests/data/extrusion_paths_with_wipe/vectors.txt", env!("CARGO_MANIFEST_DIR"));
    let text = std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
    let lines: Vec<&str> = text.lines().collect();
    let mut cases = 0;
    let mut mismatches = 0;
    for pair in lines.chunks(2) {
        let c: Vec<&str> = pair[0].split_whitespace().collect();
        assert_eq!(c[0], "C");
        let w = hex_f32(c[1]);
        let r = hex_f32(c[2]);
        let n: usize = c[3].parse().unwrap();
        let mut k = 4;
        let mut pls = Vec::new();
        for _ in 0..n {
            let np: usize = c[k].parse().unwrap();
            k += 1;
            let mut pts = Vec::new();
            for _ in 0..np {
                pts.push(Point::new(c[k].parse().unwrap(), c[k + 1].parse().unwrap()));
                k += 2;
            }
            pls.push(Polyline::from_points(pts));
        }
        let mut dst = Vec::new();
        extrusion_entities_append_paths_with_wipe(&mut dst, pls, ExtrusionRole::TopSolidInfill, 0.02, w, 0.2, r);
        let mut got = format!("R {}", dst.len());
        for e in &dst {
            match e {
                ExtrusionEntityType::MultiPath(mp) => {
                    got += &format!(" M {}", mp.paths.len());
                    for p in &mp.paths {
                        got += &format!(" {} {}", p.no_extrusion as i32, p.polyline.points().len());
                        for q in p.polyline.points() {
                            got += &format!(" {} {}", q.x, q.y);
                        }
                    }
                }
                _ => got += " NOT_A_MULTIPATH",
            }
        }
        cases += 1;
        if got != pair[1] {
            mismatches += 1;
            if mismatches <= 3 {
                eprintln!("case {cases}:\n  pin  {}\n  rust {}", pair[1], got);
            }
        }
    }
    eprintln!("append_paths_with_wipe: {cases} cases, {mismatches} mismatches");
    assert!(cases >= 400);
    assert_eq!(mismatches, 0);
}
