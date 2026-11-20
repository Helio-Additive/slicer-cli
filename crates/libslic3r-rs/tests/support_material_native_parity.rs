//! Native parity of the Support/SupportMaterial.cpp port (`slicer::support::support_material`) against
//! vectors made by the C++ at the pin (tests/data/support_material/gen.cpp).
//!
//! The first part ("grid") is the support grid: rasterize_polygons, contours_simplified, SupportGridParams
//! and SupportGridPattern (the constructor, extract_support, dilate_trimming_region, seed_fill_block,
//! island_samples) on polygons and rasters the generator made. The lines of the vectors that start with
//! "in" are the inputs, which this test reads back; every other line it has to reproduce.

mod support_shared;

use slicer::flow::Flow;
use slicer::geometry::{Point, Polygons};
use slicer::print_config::config_def::DynamicPrintConfig;
use slicer::support::support_material::grid::{contours_simplified, rasterize_polygons, SupportGridParams, SupportGridPattern, SMS_GRID};
use support_shared::*;

/// gen.cpp `HEX`
fn hex(d: &[u8]) -> String {
    if d.is_empty() {
        return "-".into();
    }
    const DIGITS: &[u8; 16] = b"0123456789abcdef";
    let mut o = String::with_capacity(d.len() * 2);
    for c in d {
        o.push(DIGITS[(c >> 4) as usize] as char);
        o.push(DIGITS[(c & 15) as usize] as char);
    }
    o
}
fn unhex(s: &str) -> Vec<u8> {
    if s == "-" {
        return Vec::new();
    }
    let b = s.as_bytes();
    let v = |c: u8| if c <= b'9' { c - b'0' } else { c - b'a' + 10 };
    (0..b.len() / 2).map(|i| (v(b[2 * i]) << 4) | v(b[2 * i + 1])).collect()
}
fn pts_s(pts: &[Point]) -> String {
    let mut o = format!("{}", pts.len());
    for q in pts {
        o += &format!(" {}", pt(q));
    }
    o
}
/// gen.cpp `PARAMS`
fn params_s(p: &SupportGridParams) -> String {
    format!(
        "{} {} {} {} {} {} {}",
        p.style,
        hd(p.grid_resolution),
        hd(p.support_angle),
        hd(p.extrusion_width),
        hd(p.support_closing_radius),
        p.expansion_to_slice,
        p.expansion_to_propagate
    )
}
fn read_params(t: &mut Tok) -> SupportGridParams {
    SupportGridParams {
        style: t.i() as i32,
        grid_resolution: t.d(),
        support_angle: t.d(),
        extrusion_width: t.d(),
        support_closing_radius: t.d(),
        expansion_to_slice: t.i() as i32,
        expansion_to_propagate: t.i() as i32,
    }
}

/// gen.cpp `run_grid`
fn run_grid(r: &mut Run) {
    while let Some(line) = r.input("in rast") {
        let mut t = Tok::new(&line);
        let grid_size = (t.i() as i32, t.i() as i32);
        let pixel_size = t.d();
        let left_bottom = t.pt();
        let polygons: Polygons = t.polys();
        r.out(format!("rast {}", hex(&rasterize_polygons(grid_size, pixel_size, &left_bottom, &polygons))));
    }
    while let Some(line) = r.input("in dil") {
        let mut t = Tok::new(&line);
        let grid_size = (t.i() as i32, t.i() as i32);
        let trimming = unhex(t.s());
        r.out(format!("dil {}", hex(&SupportGridPattern::dilate_trimming_region(&trimming, grid_size))));
    }
    while let Some(line) = r.input("in seed") {
        let mut t = Tok::new(&line);
        let grid_size = (t.i() as i32, t.i() as i32);
        let grid_blocks = (t.i() as i32, t.i() as i32);
        let oversampling = t.i() as i32;
        let mut grid = unhex(t.s());
        let trimming = unhex(t.s());
        SupportGridPattern::seed_fill_block(&mut grid, grid_size, &trimming, grid_blocks, oversampling);
        r.out(format!("seed {}", hex(&grid)));
    }
    while let Some(line) = r.input("in cs") {
        let mut t = Tok::new(&line);
        let grid_size = (t.i() as i32, t.i() as i32);
        let pixel_size = t.d();
        let left_bottom = t.pt();
        let offset = t.i() as i32;
        let fill_holes = t.b();
        let grid = unhex(t.s());
        r.out(format!("cs {}", polys_s(&contours_simplified(grid_size, pixel_size, left_bottom, &grid, offset, fill_holes))));
    }
    while let Some(line) = r.input("in isl") {
        let mut t = Tok::new(&line);
        let expolygons = t.expolys();
        r.out(format!("isl {}", pts_s(&SupportGridPattern::island_samples(&expolygons))));
    }
    while let Some(line) = r.input("in par") {
        let mut t = Tok::new(&line);
        let style = t.i();
        let spacing = t.d();
        let angle = t.d();
        let (w, h, n) = (t.d(), t.d(), t.d());
        let mut config = DynamicPrintConfig::new();
        let style_name = ["default", "grid", "snug", "tree_slim", "tree_strong", "tree_hybrid", "tree_organic"][style as usize];
        deser(&mut config, "support_style", style_name);
        deser(&mut config, "support_base_pattern_spacing", &format!("{spacing}"));
        deser(&mut config, "support_angle", &format!("{angle}"));
        let flow = Flow::new(w, h, n).expect("flow");
        r.out(format!("par {}", params_s(&SupportGridParams::new(&config, &flow))));
    }
    while let Some(line) = r.input("in pat") {
        let mut t = Tok::new(&line);
        let params = read_params(&mut t);
        let support_polygons: Polygons = t.polys();
        let trimming_polygons: Polygons = t.polys();
        let pattern = SupportGridPattern::new(&support_polygons, &trimming_polygons, &params);
        if pattern.style() == SMS_GRID {
            let (grid_size, pixel_size, min, max, grid) = pattern.grid_state();
            r.out(format!("pat {} {} {} {} {} {}", grid_size.0, grid_size.1, hd(pixel_size), pt(&min), pt(&max), hex(grid)));
        } else {
            r.out(format!("pat {}", pattern.style()));
        }
        while let Some(line) = r.input("in ext") {
            let mut t = Tok::new(&line);
            let offset = t.i() as i32;
            let fill_holes = t.b();
            r.out(format!("ext {}", polys_s(&pattern.extract_support(offset, fill_holes))));
        }
    }
}

#[test]
fn support_material_native_parity() {
    let mut r = Run::load("support_material");
    run_grid(&mut r);
    let n = r.expected.iter().filter(|l| !l.starts_with("in ")).count();
    for tag in ["rast ", "dil ", "seed ", "cs ", "isl ", "par ", "pat ", "ext "] {
        let (total, ok) = r.tally(tag);
        eprintln!("support_material: {tag}: {ok} of {total}");
    }
    let bad = r.finish();
    eprintln!("support_material: {} result lines, {} differ", n, bad);
    assert_eq!(bad, 0);
}
