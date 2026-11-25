//! Native parity of the speed smoothing of discontinuity areas: `GCode::mapping_speed`,
//! `GCode::get_speed_coor_x` (GCode.cpp:7157-7170), `GCode::split_and_mapping_speed`
//! (GCode.cpp:7212-7359), `GCode::set_speed_transition` (GCode.cpp:7402-7497),
//! `GCode::merge_same_speed_paths` (GCode.cpp:7362-7400) and `GCode::smooth_speed_discontinuity_area`
//! (GCode.cpp:7499-7512). The port takes the per-path speed (`GCode::get_path_speed`) as a function;
//! the test hands it the pin's `get_path_speed` values the generator printed for each path, and checks
//! the walls' values against the port's `overhang_degree_corr_speed` (`GCode::get_overhang_degree_corr_speed`,
//! GCode.cpp:7172-7202) with the volumetric cap of `get_path_speed` (GCode.cpp:6653-6658).
//!
//! `tests/data/g_code_smooth/g_code_smooth.txt.gz` is the output of `gen.cpp` beside it (gzip -9n),
//! which runs the pin's GCode.cpp over LCG-made wall paths. The pin keeps `m_smooth_coefficient` in a
//! float (GCode.hpp:659); the port takes it as the double that float widens to.

use slicer::extrusion_entity::{ExtrusionPath, ExtrusionRole};
use slicer::g_code::{
    get_speed_coor_x, mapping_speed, merge_same_speed_paths, overhang_degree_corr_speed, set_speed_transition,
    smooth_speed_discontinuity_area, split_and_mapping_speed,
};
use std::collections::HashMap;
use slicer::geometry::{Point, Polyline};
use std::io::Read;

fn read_vectors() -> String {
    let path = format!("{}/tests/data/g_code_smooth/g_code_smooth.txt.gz", env!("CARGO_MANIFEST_DIR"));
    let file = std::fs::File::open(&path).expect("vectors");
    let mut text = String::new();
    flate2::read::GzDecoder::new(file).read_to_string(&mut text).expect("gzip");
    text
}

fn f32of(t: &str) -> f32 {
    f32::from_bits(u32::from_str_radix(t, 16).expect("f32 hex"))
}

fn f64of(t: &str) -> f64 {
    f64::from_bits(u64::from_str_radix(t, 16).expect("f64 hex"))
}

fn f64h(v: f64) -> String {
    format!("{:016x}", v.to_bits())
}

fn role_of(c: &str) -> ExtrusionRole {
    match c {
        "0" => ExtrusionRole::Perimeter,
        "1" => ExtrusionRole::ExternalPerimeter,
        _ => ExtrusionRole::OverhangPerimeter,
    }
}

fn role_code(r: ExtrusionRole) -> &'static str {
    match r {
        ExtrusionRole::Perimeter => "0",
        ExtrusionRole::ExternalPerimeter => "1",
        _ => "2",
    }
}

/// Reads `<n> {<role> <smooth_speed> <np> x y ...}` from `t[*i..]`.
fn read_paths(t: &[&str], i: &mut usize) -> Vec<ExtrusionPath> {
    let n: usize = t[*i].parse().unwrap();
    *i += 1;
    let mut out = Vec::with_capacity(n);
    for _ in 0..n {
        let mut p = ExtrusionPath::new(role_of(t[*i]));
        p.mm3_per_mm = 0.05;
        p.width = 0.45f32 as f64;
        p.height = 0.2f32 as f64;
        p.smooth_speed = f64of(t[*i + 1]);
        let np: usize = t[*i + 2].parse().unwrap();
        *i += 3;
        let mut pl = Polyline::new();
        for _ in 0..np {
            let x: i64 = t[*i].parse().unwrap();
            let y: i64 = t[*i + 1].parse().unwrap();
            pl.push(Point::new(x, y));
            *i += 2;
        }
        p.polyline = pl;
        out.push(p);
    }
    out
}

/// The same text the generator prints for a list of paths.
fn put_paths(ps: &[ExtrusionPath]) -> String {
    let mut s = format!("{}", ps.len());
    for p in ps {
        s += &format!(" {} {} {}", role_code(p.role), f64h(p.smooth_speed), p.polyline.points.len());
        for pt in p.polyline.points.iter() {
            s += &format!(" {} {}", pt.x(), pt.y());
        }
    }
    s
}

fn expect_bar(t: &[&str], i: &mut usize) {
    assert_eq!(t[*i], "|");
    *i += 1;
}

#[test]
fn g_code_smooth_native_parity() {
    let text = read_vectors();
    let (mut n_map, mut n_st, mut n_sm, mut n_mg) = (0usize, 0usize, 0usize, 0usize);
    let mut bad: Vec<String> = Vec::new();
    // The generator's speeds for the 'mg' cases (filament_max_volumetric_speed 40, inner 200, outer 120).
    let mut ocfg = slicer::print_config::PrintObjectConfig::default();
    ocfg.overhang_1_4_speed = 100.0;
    ocfg.overhang_2_4_speed = 70.0;
    ocfg.overhang_3_4_speed = 40.0;
    ocfg.overhang_4_4_speed = 15.0;
    ocfg.overhang_totally_speed = 10.0;
    ocfg.bridge_speed = 25.0;
    let mut n_corr = 0usize;
    for (ln, line) in text.lines().enumerate() {
        let t: Vec<&str> = line.split(' ').collect();
        match t[0] {
            "map" => {
                n_map += 1;
                let coeff = f32of(t[1]) as f64;
                let dist = f64of(t[2]);
                let speed = f64of(t[4]);
                let ms = mapping_speed(coeff, dist);
                let gx = get_speed_coor_x(coeff, speed);
                if ms.to_bits() != f64of(t[3]).to_bits() || gx.to_bits() != f64of(t[5]).to_bits() {
                    bad.push(format!("line {} map: port {} {} pin {} {}", ln + 1, f64h(ms), f64h(gx), t[3], t[5]));
                }
            }
            "st" => {
                n_st += 1;
                let coeff = f32of(t[1]) as f64;
                let ng: usize = t[2].parse().unwrap();
                let mut i = 3;
                let mut groups: Vec<Vec<ExtrusionPath>> = Vec::with_capacity(ng);
                for _ in 0..ng {
                    groups.push(read_paths(&t, &mut i));
                }
                expect_bar(&t, &mut i);
                let want = t[i..].join(" ");
                let got = put_paths(&set_speed_transition(coeff, &mut groups));
                if got != want {
                    bad.push(format!("line {} st:\n  port {}\n  pin  {}", ln + 1, got, want));
                }
            }
            "sm" => {
                n_sm += 1;
                let coeff = f32of(t[1]) as f64;
                let other_v = f64of(t[2]);
                let final_v = f64of(t[3]);
                let max_len = f64of(t[4]);
                let left = t[5] == "1";
                let mut i = 6;
                let mut ps = read_paths(&t, &mut i);
                expect_bar(&t, &mut i);
                let rest = t[i..].join(" ");
                let mut interp = Vec::new();
                split_and_mapping_speed(coeff, other_v, final_v, &mut ps, max_len, &mut interp, left);
                let got = format!("{} | {}", put_paths(&interp), put_paths(&ps));
                if got != rest {
                    bad.push(format!("line {} sm:\n  port {}\n  pin  {}", ln + 1, got, rest));
                }
            }
            "mg" => {
                n_mg += 1;
                let coeff = f32of(t[1]) as f64;
                let n: usize = t[2].parse().unwrap();
                let mut i = 3;
                let mut ps: Vec<ExtrusionPath> = Vec::with_capacity(n);
                // get_path_speed is a function of (role, overhang_degree, mm3_per_mm) here (one filament,
                // no layer), so the pin's values are looked up by those.
                let mut speed: HashMap<(&str, u64, u64), f64> = HashMap::new();
                for _ in 0..n {
                    let mut p = ExtrusionPath::new(role_of(t[i]));
                    p.overhang_degree = f64of(t[i + 1]);
                    p.mm3_per_mm = f64of(t[i + 2]);
                    p.width = 0.45f32 as f64;
                    p.height = 0.2f32 as f64;
                    let v = f64of(t[i + 3]);
                    if t[i] != "2" {
                        // GCode.cpp:6620-6633, 6653-6658 for a wall path (no layer: nothing touches the bed).
                        n_corr += 1;
                        let normal = if t[i] == "0" { 200.0 } else { 120.0 };
                        let new_speed = overhang_degree_corr_speed(&ocfg, normal, p.overhang_degree);
                        let speed = if new_speed == 0.0 { normal } else { new_speed };
                        let speed = speed.min(40.0 / p.mm3_per_mm);
                        if speed.to_bits() != v.to_bits() {
                            bad.push(format!(
                                "line {} get_path_speed degree {:?} mm3 {}: port {:?} pin {:?}",
                                ln + 1,
                                p.overhang_degree,
                                p.mm3_per_mm,
                                speed,
                                v
                            ));
                        }
                    }
                    if let Some(old) = speed.insert((t[i], p.overhang_degree.to_bits(), p.mm3_per_mm.to_bits()), v) {
                        assert_eq!(old.to_bits(), v.to_bits(), "get_path_speed is not a function of the key");
                    }
                    let np: usize = t[i + 4].parse().unwrap();
                    i += 5;
                    let mut pl = Polyline::new();
                    for _ in 0..np {
                        pl.push(Point::new(t[i].parse().unwrap(), t[i + 1].parse().unwrap()));
                        i += 2;
                    }
                    p.polyline = pl;
                    ps.push(p);
                }
                let speed_of = |p: &ExtrusionPath| -> f64 {
                    speed[&(role_code(p.role), p.overhang_degree.to_bits(), p.mm3_per_mm.to_bits())]
                };
                expect_bar(&t, &mut i);
                let ng: usize = t[i].parse().unwrap();
                let mut j = i + 1;
                let mut want_groups = Vec::with_capacity(ng);
                for _ in 0..ng {
                    let start = j;
                    let _ = read_paths(&t, &mut j);
                    want_groups.push(t[start..j].join(" "));
                }
                expect_bar(&t, &mut j);
                let want_smooth = t[j..].join(" ");
                let groups = merge_same_speed_paths(&ps, speed_of);
                let got_groups: Vec<String> = groups.iter().map(|g| put_paths(g)).collect();
                if got_groups != want_groups {
                    bad.push(format!("line {} mg merge:\n  port {:?}\n  pin  {:?}", ln + 1, got_groups, want_groups));
                }
                smooth_speed_discontinuity_area(coeff, &mut ps, speed_of);
                let got_smooth = put_paths(&ps);
                if got_smooth != want_smooth {
                    bad.push(format!("line {} mg smooth:\n  port {}\n  pin  {}", ln + 1, got_smooth, want_smooth));
                }
            }
            "" => {}
            other => panic!("unknown tag {other} on line {}", ln + 1),
        }
    }
    eprintln!(
        "g_code_smooth: map {n_map}, set_speed_transition {n_st}, split_and_mapping_speed {n_sm}, \
         merge_same_speed_paths + smooth_speed_discontinuity_area {n_mg}, wall get_path_speed {n_corr}, mismatches {}",
        bad.len()
    );
    for b in bad.iter().take(5) {
        eprintln!("{b}");
    }
    assert!(n_map > 0 && n_st > 0 && n_sm > 0 && n_mg > 0);
    assert!(bad.is_empty(), "{} mismatches", bad.len());
}
