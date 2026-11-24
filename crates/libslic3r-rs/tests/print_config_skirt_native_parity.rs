//! Native parity of the skirt helpers and the bed shape with its excluded areas of PrintConfig.cpp
//! (`slicer::print_config::config_def`): `has_skirt`, the three `get_real_skirt_dist` overloads and, through
//! them, `skirt_expansion_params`, `skirt_nozzle_diameter_for_filament` and `skirt_flow_from_config`;
//! `get_bed_shape_with_excluded_area`; `get_extruder_index`, `nozzle_diameter_for_filament`,
//! `get_filament_config_idx`, `get_process_config_idx`. Vectors from the C++ at the pin (tests/data/print_config_skirt/gen.cpp):
//! every line has to be equal.

use slicer::config::{ConfigBase, ConfigSubstitutionContext, ForwardCompatibilitySubstitutionRule};
use slicer::print_config::config_def::get_real_skirt_dist_overloads;
use slicer::print_config::config_def::{
    get_bed_shape_with_excluded_area, get_extruder_index, get_filament_config_idx, get_process_config_idx, get_real_skirt_dist, has_skirt,
    nozzle_diameter_for_filament, DynamicPrintConfig,
};

fn hf(v: f32) -> String {
    format!("{:08x}", v.to_bits())
}
fn hb(v: bool) -> &'static str {
    if v {
        "1"
    } else {
        "0"
    }
}
fn poly_s(p: &slicer::geometry::Polygon) -> String {
    let mut o = format!("{}", p.points.len());
    for pt in &p.points {
        o += &format!(" {} {}", pt.x, pt.y);
    }
    o
}

fn deser(config: &mut DynamicPrintConfig, key: &str, value: &str) {
    let mut ctxt = ConfigSubstitutionContext::new(ForwardCompatibilitySubstitutionRule::Disable);
    config.set_deserialize(key, value, &mut ctxt, false).unwrap_or_else(|e| panic!("{key}={value}: {e:?}"));
}

/// gen.cpp `scenario_config(s)`
fn scenario_config(s: i32) -> DynamicPrintConfig {
    let mut c = DynamicPrintConfig::full_print_config();
    let mut d = |k: &str, v: &str| deser(&mut c, k, v);
    match s {
        1 => {
            d("skirt_per_object", "1");
            d("skirt_loops", "2");
            d("skirt_height", "1");
            d("skirt_distance", "3.5");
        }
        2 => {
            d("skirt_per_object", "1");
            d("skirt_loops", "0");
            d("draft_shield", "enabled");
            d("skirt_distance", "2");
        }
        3 => {
            d("skirt_per_object", "1");
            d("skirt_loops", "3");
            d("skirt_height", "2");
            d("initial_layer_line_width", "0");
            d("line_width", "0.45");
            d("initial_layer_print_height", "0.25");
            d("nozzle_diameter", "0.4,0.6");
            d("filament_diameter_map", "\"\";\"0.8\";\"0.4;0.6\"");
            d("support_filament", "2");
        }
        4 => {
            d("skirt_per_object", "1");
            d("skirt_loops", "1");
            d("skirt_height", "0");
        }
        5 => {
            d("skirt_per_object", "1");
            d("skirt_loops", "2");
            d("skirt_height", "1");
            d("initial_layer_line_width", "0");
            d("line_width", "0");
            d("nozzle_diameter", "0.2,0.8");
            d("multi_nozzle_diameter_mode", "1");
        }
        6 => {
            d("skirt_per_object", "1");
            d("skirt_loops", "4");
            d("skirt_height", "1");
            d("skirt_distance", "0.7");
            d("initial_layer_line_width", "120%");
            d("nozzle_diameter", "0.6");
            d("filament_diameter_map", "\"0.6\";\"0\"");
            d("printable_area", "0x0,180x0,180x180,0x180");
            d("bed_exclude_area", "0x0,20x0,20x30,0x30,160x0,180x0,180x15,160x15");
        }
        7 => {
            d("printable_area", "0x0,256x0,256x256,0x256");
            d("bed_exclude_area", "0x0,18x0,18x28,0x28");
            d("enable_wrapping_detection", "1");
            d("wrapping_exclude_area", "240.5x240.25,256x240.25,256x256,240.5x256");
        }
        8 => {
            d("printable_area", "0x0,350x0,350x320,0x320");
            d("extruder_printable_area", "0x0,325x0,325x320,0x320#25x0,350x0,350x320,25x320");
            d("bed_exclude_area", "30x30,60x30,60x60,30x60");
        }
        9 => {
            // gen.cpp: extruder_type keeps its default (see there).
            drop(d);
            c.set_num_filaments(4).unwrap();
            let mut d = |k: &str, v: &str| deser(&mut c, k, v);
            d("nozzle_diameter", "0.4,0.6");
            d("filament_map", "1,2,0,2");
            d("filament_volume_map", "0,1,0,1");
            d("printer_extruder_id", "1,1,2,2");
            d("printer_extruder_variant", "Direct Drive Standard,Direct Drive High Flow,Bowden Standard,Bowden High Flow");
            d("filament_self_index", "1,1,2,2,3,4,4");
            d(
                "filament_extruder_variant",
                "Direct Drive Standard,Direct Drive High Flow,Bowden Standard,Bowden High Flow,Direct Drive Standard,Bowden Standard,Bowden High Flow",
            );
            d("filament_diameter_map", "\"\";\"0.8\";\"0.4;0.6\";\"0\"");
        }
        _ => {}
    }
    c
}

#[test]
fn skirt_helpers_and_bed_shape_match_the_pin() {
    let mut lines: Vec<String> = Vec::new();
    for s in 0..10 {
        let c = scenario_config(s);
        let sc = format!("k{}", s);
        lines.push(format!("{} has_skirt {}", sc, hb(has_skirt(&c))));
        for w in [0.42f32, 0.5] {
            for h in [0.2f32, 0.28] {
                // Flow(float width, float height, float nozzle_diameter)
                let flow = slicer::flow::Flow::new(w as f64, h as f64, 0.4f32 as f64).unwrap();
                lines.push(format!("{} dist_flow {} {} {}", sc, hf(w), hf(h), hf(get_real_skirt_dist(&c, &flow))));
            }
        }
        let ids: [&[u32]; 6] = [&[], &[1], &[2], &[3], &[1, 2, 3], &[2, 1]];
        for v in ids {
            let mut l = format!("{} dist_ids {}", sc, v.len());
            for f in v {
                l += &format!(" {}", f);
            }
            lines.push(format!("{} {}", l, hf(get_real_skirt_dist_overloads::get_real_skirt_dist(&c, v))));
        }
        lines.push(format!("{} dist {}", sc, hf(get_real_skirt_dist_overloads::no_filaments::get_real_skirt_dist(&c))));
        // nozzle_diameter_for_filament and get_extruder_index everywhere, the config indices on k9 (gen.cpp).
        let mut ix = format!("{} idx", sc);
        for f in 0..6u32 {
            ix += &format!(" {} {}", get_extruder_index(&c, f), hf(nozzle_diameter_for_filament(&c, f)));
        }
        if s == 9 {
            for f in 0..4u32 {
                ix += &format!(" {} {}", get_filament_config_idx(&c, f), get_process_config_idx(&c, f));
            }
        }
        lines.push(ix);
        // PrintConfig pc; pc.apply(c, true); get_bed_shape(pc, share)
        let mut bs = format!("{} bed_shape", sc);
        for share in [false, true] {
            let pts = slicer::print_config::config_def::get_bed_shape_overloads::get_bed_shape(&c, share);
            bs += &format!(" {}", pts.len());
            for pt in &pts {
                bs += &format!(" {} {}", pt.x, pt.y);
            }
        }
        lines.push(bs);
        lines.push(format!(
            "{} bed_excluded {} share {}",
            sc,
            poly_s(&get_bed_shape_with_excluded_area(&c, false)),
            poly_s(&get_bed_shape_with_excluded_area(&c, true))
        ));
    }
    let path = format!("{}/tests/data/print_config_skirt/vectors.txt", env!("CARGO_MANIFEST_DIR"));
    let expected = std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
    let expected: Vec<&str> = expected.lines().collect();
    let mut mismatches = 0usize;
    for k in 0..expected.len().max(lines.len()) {
        let e = expected.get(k).copied().unwrap_or("<missing>");
        let r = lines.get(k).map(|s| s.as_str()).unwrap_or("<missing>");
        if e != r {
            mismatches += 1;
            if mismatches <= 40 {
                eprintln!("line {}:\n  C++:  {}\n  Rust: {}", k + 1, e, r);
            }
        }
    }
    eprintln!("{} lines, {} mismatches", expected.len(), mismatches);
    assert_eq!(mismatches, 0);
}

/// The live copy of the skirt helpers (arrange.rs `print_config_skirt`, what Print's sequential clearance
/// and the arrange code call) gives the same results as the paired port on every configuration above.
#[test]
fn arrange_copy_of_the_skirt_helpers_matches() {
    use slicer::arrange::print_config_skirt as live;
    for s in 0..10 {
        let c = scenario_config(s);
        assert_eq!(live::has_skirt(&c), has_skirt(&c), "k{s} has_skirt");
        for w in [0.42f32, 0.5] {
            for h in [0.2f32, 0.28] {
                let flow = slicer::flow::Flow::new(w as f64, h as f64, 0.4f32 as f64).unwrap();
                assert_eq!(live::get_real_skirt_dist(&c, &flow).to_bits(), get_real_skirt_dist(&c, &flow).to_bits(), "k{s} flow");
            }
        }
        let ids: [&[u32]; 6] = [&[], &[1], &[2], &[3], &[1, 2, 3], &[2, 1]];
        for v in ids {
            assert_eq!(
                live::overloads::get_real_skirt_dist(&c, v).to_bits(),
                get_real_skirt_dist_overloads::get_real_skirt_dist(&c, v).to_bits(),
                "k{s} ids {v:?}"
            );
        }
        assert_eq!(
            live::overloads::no_filaments::get_real_skirt_dist(&c).to_bits(),
            get_real_skirt_dist_overloads::no_filaments::get_real_skirt_dist(&c).to_bits(),
            "k{s}"
        );
    }
}
