//! `slicer::arrange` (the port of libslic3r/Arrange.cpp around the nester) against the C++ at the
//! pin.
//!
//! `tests/data/arrange/vectors.txt` is the output of `gen.cpp` beside it, which includes the pin's
//! Arrange.cpp (most of the helpers are static or templates of it) and runs them over eleven point
//! sets, six `ArrangeParams` and four print configs; the header of gen.cpp describes the lines.
//! The same calls run here and have to print the same lines: doubles and floats bit for bit,
//! scaled coordinates, strings.
//!
//! Not covered, because the crate has no port of libnest2d's nester, NFP placer, selection and
//! optimizer: `AutoArranger`, `_arrange` and the `arrange` entry points (see the header of
//! `src/arrange.rs`).

use slicer::arrange::libnest2d::{NfpPConfig, _Box, _Circle, _Item};
use slicer::arrange::*;
use slicer::config::{ConfigBase, ConfigSubstitutionContext, ForwardCompatibilitySubstitutionRule};
use slicer::geometry::{BoundingBox, ExPolygon, Point, Polygon};
use slicer::print_config::config_def::DynamicPrintConfig;
use std::collections::{BTreeMap, HashMap};

fn hd(v: f64) -> String {
    format!("{:016x}", v.to_bits())
}
fn hf(v: f32) -> String {
    format!("{:08x}", v.to_bits())
}
fn d(hex: &str) -> f64 {
    f64::from_bits(u64::from_str_radix(hex, 16).expect("double bits"))
}
fn b(v: bool) -> &'static str {
    if v {
        "1"
    } else {
        "0"
    }
}
fn pts(p: &[Point]) -> String {
    let mut s = p.len().to_string();
    for q in p {
        s += &format!(" {} {}", q.x, q.y);
    }
    s
}
fn bx(x: &_Box) -> String {
    format!("{} {} {} {}", x.min_corner().x, x.min_corner().y, x.max_corner().x, x.max_corner().y)
}

/// gen.cpp `make_ap`
fn make_ap(points: &[Point], k: i32) -> ArrangePolygon {
    let mut ap = ArrangePolygon::default();
    ap.poly.contour.points = points.to_vec();
    ap.translation = Point::new((k * 1234567 % 5000000) as i64, (-k * 7654321 % 3000000) as i64);
    ap.rotation = 0.37 * k as f64;
    ap.inflation = (k * 100000 - 150000) as i64;
    ap.bed_idx = k - 1;
    ap.priority = k % 4;
    ap.itemid = k + 10;
    ap.is_virt_object = (k % 5) == 3;
    ap.is_extrusion_cali_object = (k % 7) == 3;
    ap.is_wipe_tower = (k % 6) == 4;
    ap.has_tree_support = (k % 4) == 2;
    ap.extrude_id_filament_types.insert(k % 3, "PLA".to_string());
    ap.extrude_id_filament_types.insert(k % 3 + 2, "PETG".to_string());
    ap.filament_temp_type = k % 3;
    ap.bed_temp = 55 + k;
    ap.print_temp = 210 + k;
    ap.first_bed_temp = 60 + k;
    ap.first_print_temp = 215 + k;
    ap.vitrify_temp = 50 + 3 * k;
    ap.height = 10.5 * k as f64;
    ap.brim_width = 0.75 * (k % 4) as f64;
    ap.name = format!("obj{k}");
    ap
}

/// gen.cpp `make_params`
fn make_params(k: i32) -> ArrangeParams {
    let mut p = ArrangeParams::default();
    if k == 0 {
        return p;
    }
    p.min_obj_distance = if k % 2 != 0 { (k * 250000) as i64 } else { 0 };
    p.min_inflation_floor = if k % 3 == 2 { 120000 } else { 0 };
    p.accuracy = 1.0f32 / k as f32;
    p.parallel = (k % 2) == 0;
    p.allow_rotations = (k % 2) == 1;
    p.do_final_align = (k % 3) != 0;
    p.allow_multi_materials_on_same_plate = (k % 4) != 1;
    p.avoid_extrusion_cali_region = (k % 4) != 2;
    p.is_seq_print = (k % 3) == 1;
    p.save_svg = (k % 5) == 1;
    p.bed_shrink_x = 0.4f32 * k as f32;
    p.bed_shrink_y = 0.3f32 * (k % 3) as f32;
    p.clearance_height_to_rod = 25.0f32 + k as f32;
    p.clearance_height_to_lid = 90.0f32 + k as f32;
    p.cleareance_radius = 30.0f32 + 2.5f32 * k as f32;
    p.nozzle_height = 4.0f32 * k as f32;
    p.printable_height = 250.0f32 + k as f32;
    p
}

/// gen.cpp `make_config`
fn make_config(k: i32) -> DynamicPrintConfig {
    let mut c = DynamicPrintConfig::full_print_config();
    let mut set = |key: &str, value: &str| {
        let mut ctxt = ConfigSubstitutionContext::new(ForwardCompatibilitySubstitutionRule::Disable);
        c.set_deserialize(key, value, &mut ctxt, false).unwrap_or_else(|e| panic!("{key}={value}: {e:?}"));
    };
    set("printable_area", "0x0,256x0,256x256,0x256");
    if k == 1 {
        set("skirt_loops", "2");
        set("skirt_distance", "3.5");
        set("skirt_height", "1");
    }
    if k == 2 {
        set("skirt_loops", "3");
        set("skirt_distance", "2");
        set("skirt_height", "2");
        set("printable_area", "10x-5,190x-5,190x215,10x215");
        set("initial_layer_line_width", "0.6");
    }
    if k == 3 {
        set("draft_shield", "enabled");
        set("skirt_distance", "6");
        set("printable_area", "-90x-90,90x-90,90x90,-90x90");
    }
    c
}

fn params_text(p: &ArrangeParams) -> String {
    format!(
        "{} {} {} {} {} {} {}",
        p.min_obj_distance,
        p.min_inflation_floor,
        hf(p.bed_shrink_x),
        hf(p.bed_shrink_y),
        hf(p.brim_skirt_distance),
        hd(p.brim_max),
        b(p.plate_has_tree_support)
    )
}

fn item_text(it: &_Item) -> String {
    let mut s = format!(
        "{} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {}",
        it.bin_id(),
        it.priority(),
        it.item_id(),
        hd(it.rotation()),
        it.translation().x,
        it.translation().y,
        it.inflation(),
        hd(it.height),
        it.name,
        b(it.is_virt_object),
        b(it.is_wipe_tower),
        b(it.is_extrusion_cali_object),
        hd(it.bed_temp),
        hd(it.print_temp),
        hd(it.vitrify_temp),
        it.filament_temp_type,
        b(it.is_fixed())
    );
    s += &format!(" {}", it.extrude_id_filament_types.len());
    for (k, v) in &it.extrude_id_filament_types {
        s += &format!(" {k} {v}");
    }
    s += &format!(" {} {}", it.vertex_count(), hd(it.area()));
    s += &format!(" | {}", pts(&it.transformed_shape().contour.points));
    s += &format!(" | {}", bx(&it.bounding_box()));
    s
}

fn bed_text(points: &[Point]) -> String {
    call_with_bed(&points.to_vec(), |bed| match bed {
        ArrangeBed::Infinite(i) => format!("infinite {} {} {}", i.center.x, i.center.y, bx(&i.to_nestbin())),
        ArrangeBed::Box(bb) => {
            let n = bb.to_nestbin();
            format!("box {} {} {} {}", bx(&n), n.width(), n.height(), n.area())
        }
        ArrangeBed::Circle(c) => {
            let n: _Circle = c.to_nestbin();
            format!("circle {} {} {}", n.center().x, n.center().y, hd(n.radius()))
        }
        ArrangeBed::Polygon(p) => {
            let e: ExPolygon = p.to_nestbin();
            format!("polygon {} {}", pts(&e.contour.points), e.holes.len())
        }
    })
}

#[test]
fn arrange_helpers_match_the_pin() {
    let path = format!("{}/tests/data/arrange/vectors.txt", env!("CARGO_MANIFEST_DIR"));
    let text = std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{path}: {e}"));

    let mut sets: HashMap<String, Vec<Point>> = HashMap::new();
    let mut names: Vec<String> = Vec::new();
    let mut by_op: BTreeMap<String, (usize, usize)> = BTreeMap::new();
    let configs: Vec<DynamicPrintConfig> = (0..4).map(make_config).collect();
    // The state of one (config, params, variant) block of gen.cpp, carried over its five lines.
    let mut block: Option<(ArrangeParams, ArrangePolygons, ArrangePolygons)> = None;

    for line in text.lines() {
        if let Some(rest) = line.strip_prefix("P ") {
            let tok: Vec<&str> = rest.split(' ').collect();
            let n: usize = tok[1].parse().unwrap();
            let p: Vec<Point> = (0..n).map(|i| Point::new(tok[2 + 2 * i].parse().unwrap(), tok[3 + 2 * i].parse().unwrap())).collect();
            sets.insert(tok[0].to_string(), p);
            names.push(tok[0].to_string());
            continue;
        }
        let (lhs, want) = line.split_once(" -> ").expect("arrow");
        let a: Vec<&str> = lhs.split(' ').collect();
        let op = a[0];
        let all_items = |sets: &HashMap<String, Vec<Point>>| -> Vec<_Item> {
            let mut all = Vec::new();
            for (i, name) in names.iter().enumerate() {
                process_arrangeable(&make_ap(&sets[name], i as i32 + 1), &mut all);
            }
            all
        };
        let got: String = match op {
            "unscaled" => {
                let u = unscaled(&Point::new(a[1].parse().unwrap(), a[2].parse().unwrap()));
                format!("{} {}", hd(u.x), hd(u.y))
            }
            "to_json" => make_params(a[1].parse().unwrap()).to_json(),
            "measures" => {
                let p = &sets[a[1]];
                let bb = BoundingBox::from_points(p);
                format!(
                    "{} {} {} {} {}",
                    width(&bb),
                    height(&bb),
                    hd(area(&bb)),
                    hd(poly_area(p)),
                    hd(distance_to(&p[0], p.last().unwrap()))
                )
            }
            "to_circle" => {
                let p = &sets[a[1]];
                let bb = BoundingBox::from_points(p);
                let c = to_circle(&bb.center(), p);
                format!("{} {} {}", c.center().x, c.center().y, if c.radius().is_nan() { "nan".to_string() } else { hd(c.radius()) })
            }
            "call_with_bed" => bed_text(&sets[a[1]]),
            "call_with_bed_empty" => bed_text(&[]),
            "call_with_bed_one" => bed_text(&[Point::new(700000, 300000)]),
            "min_area_rotation" | "fit_into_box" => {
                let mut sh = ExPolygon::default();
                sh.contour = Polygon::from_points(sets[a[1]].clone());
                if sh.contour.is_counter_clockwise() {
                    sh.contour.reverse();
                }
                if op == "min_area_rotation" {
                    hd(min_area_boundingbox_rotation(&sh))
                } else {
                    match fit_into_box_rotation(&sh, &_Box::with_size(a[2].parse().unwrap(), a[3].parse().unwrap(), Point::new(0, 0))) {
                        Ok(r) => hd(r),
                        Err(_) => "throws".to_string(),
                    }
                }
            }
            "process_arrangeable" => {
                let mut items = Vec::new();
                process_arrangeable(&make_ap(&sets[a[1]], a[2].parse().unwrap()), &mut items);
                let mut s = items.len().to_string();
                for it in &items {
                    s += " # ";
                    s += &item_text(it);
                }
                s
            }
            "remove_large_items_box" => {
                let mut items = all_items(&sets);
                let bin = _Box::new(
                    Point::new(a[1].parse().unwrap(), a[2].parse().unwrap()),
                    Point::new(a[3].parse().unwrap(), a[4].parse().unwrap()),
                );
                remove_large_items(&mut items, &bin);
                let mut s = items.len().to_string();
                for it in &items {
                    s += &format!(" {}", it.item_id());
                }
                s
            }
            "remove_large_items_circle" => {
                let mut items = all_items(&sets);
                let bin = _Circle::new(Point::new(a[1].parse().unwrap(), a[2].parse().unwrap()), d(a[3]));
                remove_large_items(&mut items, &bin);
                let mut s = items.len().to_string();
                for it in &items {
                    s += &format!(" {}", it.item_id());
                }
                s
            }
            "fixed_overfit" => {
                let pile = _Box::new(
                    Point::new(a[2].parse().unwrap(), a[3].parse().unwrap()),
                    Point::new(a[4].parse().unwrap(), a[5].parse().unwrap()),
                );
                let binbb = _Box::new(Point::new(0, 0), Point::new(25600000, 25600000));
                let excluded = vec![
                    _Box::new(Point::new(0, 0), Point::new(1800000, 2800000)),
                    _Box::new(Point::new(20000000, 24000000), Point::new(25600000, 25600000)),
                ];
                let res = (d(a[1]), pile);
                format!(
                    "{} {} {}",
                    hd(fixed_overfit(&res, &binbb)),
                    hd(fixed_overfit_topright_sliding(&res, &binbb, &excluded)),
                    hd(fixed_overfit_topright_sliding(&res, &binbb, &[]))
                )
            }
            "fill_config" => {
                let k: i32 = a[1].parse().unwrap();
                let mut p = make_params(k);
                for (i, name) in names.iter().enumerate() {
                    let j = i as i32 + 1;
                    if (j + k) % 3 == 0 {
                        p.excluded_regions.push(make_ap(&sets[name], j));
                    }
                    if (j + k) % 4 == 1 {
                        p.nonprefered_regions.push(make_ap(&sets[name], j + 1));
                    }
                }
                let mut pcfg = NfpPConfig::default();
                fill_config(&mut pcfg, &p);
                let mut s = format!("{} {} {}", pcfg.starting_point as i32, pcfg.alignment as i32, pcfg.rotations.len());
                for r in &pcfg.rotations {
                    s += &format!(" {}", hd(*r));
                }
                s += &format!(
                    " {} {} {} {} {}",
                    pcfg.bed_shrink.x,
                    pcfg.bed_shrink.y,
                    hf(pcfg.accuracy),
                    b(pcfg.parallel),
                    b(pcfg.save_svg)
                );
                s += &format!(" {}", pcfg.m_excluded_regions.len());
                for it in &pcfg.m_excluded_regions {
                    s += " # ";
                    s += &item_text(it);
                }
                s += &format!(" ## {}", pcfg.m_nonprefered_regions.len());
                for it in &pcfg.m_nonprefered_regions {
                    s += " # ";
                    s += &item_text(it);
                }
                s
            }
            "shrink_bedpts_before" => {
                let (c, k, variant): (usize, i32, i32) = (a[1].parse().unwrap(), a[2].parse().unwrap(), a[3].parse().unwrap());
                let p = make_params(k);
                let (mut selected, mut unselected) = (Vec::new(), Vec::new());
                for (i, name) in names.iter().enumerate() {
                    let j = i as i32 + 1;
                    if sets[name].len() < 3 {
                        continue;
                    }
                    let mut ap = make_ap(&sets[name], j);
                    if variant == 1 {
                        ap.height = 0.5 * j as f64;
                        ap.has_tree_support = false;
                    }
                    if j % 2 != 0 {
                        selected.push(ap);
                    } else {
                        unselected.push(ap);
                    }
                }
                let s = pts(&get_shrink_bedpts(&configs[c], &p));
                block = Some((p, selected, unselected));
                s
            }
            "update_arrange_params" => {
                let c: usize = a[1].parse().unwrap();
                let (p, selected, _) = block.as_mut().unwrap();
                update_arrange_params(p, &configs[c], selected);
                params_text(p)
            }
            "shrink_bedpts" => {
                let c: usize = a[1].parse().unwrap();
                let (p, _, _) = block.as_ref().unwrap();
                pts(&get_shrink_bedpts(&configs[c], p))
            }
            "update_selected_items_inflation" => {
                let c: usize = a[1].parse().unwrap();
                let (p, selected, _) = block.as_mut().unwrap();
                update_selected_items_inflation(selected, &configs[c], p);
                let mut s = params_text(p);
                for ap in selected.iter() {
                    s += &format!(" {}", ap.inflation);
                }
                s
            }
            "update_unselected_items_inflation" => {
                let c: usize = a[1].parse().unwrap();
                let (p, _, unselected) = block.as_mut().unwrap();
                update_unselected_items_inflation(unselected, &configs[c], p);
                let mut s = String::new();
                for ap in unselected.iter() {
                    s += &format!("{} {} {} ", ap.inflation, ap.poly.contour.points[0].x, ap.poly.contour.points[0].y);
                }
                s + "end"
            }
            other => panic!("unknown line {other}"),
        };
        let e = by_op.entry(op.to_string()).or_insert((0, 0));
        e.0 += 1;
        if got != want {
            e.1 += 1;
            let (gw, ww): (Vec<&str>, Vec<&str>) = (got.split(' ').collect(), want.split(' ').collect());
            let first = (0..gw.len().min(ww.len())).find(|&k| gw[k] != ww[k]);
            eprintln!(
                "{lhs}: rust {} tokens, c++ {}; first difference at token {:?}: rust {:?} c++ {:?}",
                gw.len(),
                ww.len(),
                first,
                first.map(|k| gw[k]),
                first.map(|k| ww[k])
            );
        }
    }
    let total: usize = by_op.values().map(|v| v.0).sum();
    println!("lines compared: {total}; by operation (compared, differing): {by_op:?}");
    let bad: Vec<(&String, &(usize, usize))> = by_op.iter().filter(|(_, v)| v.1 != 0).collect();
    assert!(bad.is_empty(), "lines differ from the C++: {bad:?}");
}
