//! Native parity of the Fill/FillBase.cpp port (`slicer::fill::fill_base`) against vectors made by the C++
//! at the pin (tests/data/fill_base/gen.cpp).
//!
//! Groups: "asp" (Fill::_adjust_solid_spacing), "idir" (Fill::_infill_direction, _layer_angle,
//! use_bridge_flow, no_sort, extended_object_bounding_box of the base class), "ml" (multiline_fill), "ci"
//! (Fill::connect_infill over an expolygon), "cip" (Fill::connect_infill over polygons and a bounding box)
//! and "cbs" (Fill::connect_base_support). The lines of the vectors that start with "in" are the inputs,
//! which this test reads back; every other line it has to reproduce bit for bit.
//!
//! The connectors of the live slicing path keep their earlier arithmetic unless `FILL_BASE_PIN` is on (see
//! `slicer::fill::fill_base`); this test runs with the gate as the environment sets it and says which arm
//! it compared.

mod support_shared;

use slicer::fill::fill_base::{multiline_fill, Fill};
use slicer::fill::FillParams;
use slicer::geometry::{BoundingBox, Point, Polygons, Polyline};
use slicer::surface::{Surface, SurfaceType};
use support_shared::*;

/// gen.cpp `PLS`
fn pls_s(ps: &[Polyline]) -> String {
    let mut o = format!("{}", ps.len());
    for p in ps {
        o += &format!(" {}", p.points.len());
        for q in &p.points {
            o += &format!(" {}", pt(q));
        }
    }
    o
}
fn tok_pls(t: &mut Tok) -> Vec<Polyline> {
    let n = t.i();
    (0..n)
        .map(|_| {
            let m = t.i();
            Polyline::from_points((0..m).map(|_| t.pt()).collect())
        })
        .collect()
}
/// gen.cpp `PARAMS`: density, anchor_length, anchor_length_max (floats), multiline.
fn tok_params(t: &mut Tok) -> FillParams {
    let mut params = FillParams::new();
    params.density = t.f() as f64;
    params.anchor_length = t.f() as f64;
    params.anchor_length_max = t.f() as f64;
    params.multiline = t.i() as i32;
    params
}

fn run_asp(r: &mut Run) {
    while let Some(line) = r.input("in asp") {
        let mut t = Tok::new(&line);
        let (width, distance) = (t.i(), t.i());
        r.out(format!("asp {}", Fill::_adjust_solid_spacing(width, distance)));
    }
}

fn run_idir(r: &mut Run) {
    while let Some(line) = r.input("in idir") {
        let mut t = Tok::new(&line);
        let mut f = Fill::new();
        f.angle = t.f();
        let layer_id = t.i();
        f.layer_id = if layer_id < 0 { usize::MAX } else { layer_id as usize };
        let has_bbox = t.b();
        let (min, max) = (t.pt(), t.pt());
        if has_bbox {
            f.set_bounding_box(&BoundingBox::from_points_minmax(min, max));
        }
        let bridge_angle = t.d();
        let thickness_layers = t.i() as usize;
        let mut surface = Surface::new(SurfaceType::Internal, t.expoly());
        // Surface.hpp:41: negative means undefined.
        surface.bridge_angle = if bridge_angle >= 0. { Some(bridge_angle) } else { None };
        surface.thickness_layers = thickness_layers;
        let d = f._infill_direction(&surface);
        let e = f.extended_object_bounding_box();
        r.out(format!(
            "idir {} {} {} {} {} {} {}",
            hf(d.0),
            pt(&d.1),
            hf(f._layer_angle(if f.layer_id == usize::MAX { 0 } else { f.layer_id })),
            hb(f.use_bridge_flow()),
            hb(f.no_sort()),
            pt(&e.min),
            pt(&e.max)
        ));
    }
}

fn run_ml(r: &mut Run) {
    while let Some(line) = r.input("in ml") {
        let mut t = Tok::new(&line);
        let mut params = FillParams::new();
        params.multiline = t.i() as i32;
        let spacing = t.f();
        let mut pls = tok_pls(&mut t);
        multiline_fill(&mut pls, &params, spacing);
        r.out(format!("ml {}", pls_s(&pls)));
    }
}

fn run_ci(r: &mut Run) {
    while let Some(line) = r.input("in ci") {
        let mut t = Tok::new(&line);
        let spacing = t.d();
        let params = tok_params(&mut t);
        let boundary = t.expoly();
        let lines = tok_pls(&mut t);
        let mut result = vec![Polyline::from_points(vec![Point::new(7, 8), Point::new(9, 10)])];
        Fill::connect_infill_expolygon(lines, &boundary, &mut result, spacing, &params);
        r.out(format!("ci {}", pls_s(&result)));
    }
}

fn run_cip(r: &mut Run) {
    while let Some(line) = r.input("in cip") {
        let mut t = Tok::new(&line);
        let spacing = t.d();
        let params = tok_params(&mut t);
        let (min, max) = (t.pt(), t.pt());
        let polygons: Polygons = t.polys();
        let lines = tok_pls(&mut t);
        let mut result = Vec::new();
        Fill::connect_infill_polygons(lines, &polygons, &BoundingBox::from_points_minmax(min, max), &mut result, spacing, &params);
        r.out(format!("cip {}", pls_s(&result)));
    }
}

fn run_cbs(r: &mut Run) {
    while let Some(line) = r.input("in cbs") {
        let mut t = Tok::new(&line);
        let spacing = t.d();
        let params = tok_params(&mut t);
        let (min, max) = (t.pt(), t.pt());
        let polygons: Polygons = t.polys();
        let lines = tok_pls(&mut t);
        let mut result = Vec::new();
        Fill::connect_base_support_polygons(lines, &polygons, &BoundingBox::from_points_minmax(min, max), &mut result, spacing, &params);
        r.out(format!("cbs {}", pls_s(&result)));
    }
}

#[test]
fn fill_base_native_parity() {
    let mut r = Run::load("fill_base");
    run_asp(&mut r);
    run_idir(&mut r);
    run_ml(&mut r);
    run_ci(&mut r);
    run_cip(&mut r);
    run_cbs(&mut r);
    let n = r.expected.iter().filter(|l| !l.starts_with("in ")).count();
    let mut not_equal: Vec<String> = Vec::new();
    for tag in ["asp ", "idir ", "ml ", "ci ", "cip ", "cbs "] {
        let (total, ok) = r.tally(tag);
        eprintln!("fill_base: {tag}: {ok} of {total}");
        if ok != total {
            not_equal.push(format!("{tag}{ok}/{total}"));
        }
    }
    let bad = r.finish();
    eprintln!("fill_base: {} result lines, {} differ", n, bad);
    assert_eq!(bad, 0, "not equal: {not_equal:?}");
}
