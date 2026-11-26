//! `slicer::g_code` (the port of libslic3r/GCode.cpp) against the C++ at the pin.
//!
//! `tests/data/g_code/g_code.txt.gz` is the output of `gen.cpp` beside it (gzip -9n), which includes
//! the pin's GCode.cpp, so its file-static functions and the private members of `GCode` are
//! reachable: `custom_gcode_sets_temperature`, `encodeBase64`, `infer_timelapse_final_plate_pos`,
//! `check_add_eol`, `custom_gcode_changes_tool`, `transform_gcode`, `get_wipe_avoid_pos_x`,
//! `has_overhang_path_on_slope`, the `GCode` methods `calc_max_volumetric_speed`,
//! `_encode_label_ids_to_base64`, the remaining-objects bitmap, `set_origin`, `append_full_config`,
//! `mass_load_limited_machine_acceleration`, `print_machine_envelope`, `DoExport::validate_custom_gcode`
//! and the `Skirt` helpers. The header of gen.cpp describes the lines.

use slicer::config::ConfigBase;
use slicer::g_code::gcode_class::*;
use slicer::geometry::{Point, Polyline};
use slicer::print_config::config_def::DynamicPrintConfig;
use std::io::Read;

fn read_vectors() -> String {
    let path = format!("{}/tests/data/g_code/g_code.txt.gz", env!("CARGO_MANIFEST_DIR"));
    let file = std::fs::File::open(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
    let mut text = String::new();
    flate2::read::GzDecoder::new(file).read_to_string(&mut text).expect("gzip");
    text
}

fn unhex(t: &str) -> String {
    if t == "-" {
        return String::new();
    }
    let b: Vec<u8> = (0..t.len() / 2).map(|i| u8::from_str_radix(&t[2 * i..2 * i + 2], 16).expect("hex")).collect();
    String::from_utf8(b).expect("the vectors are UTF-8")
}

fn hex(s: &str) -> String {
    if s.is_empty() {
        return "-".to_string();
    }
    s.bytes().map(|c| format!("{:02x}", c)).collect()
}

fn f32h(v: f32) -> String {
    format!("{:08x}", v.to_bits())
}

fn f64h(v: f64) -> String {
    format!("{:016x}", v.to_bits())
}

fn f32of(t: &str) -> f32 {
    f32::from_bits(u32::from_str_radix(t, 16).expect("f32 bits"))
}

fn f64of(t: &str) -> f64 {
    f64::from_bits(u64::from_str_radix(t, 16).expect("f64 bits"))
}

/// Reads `<n> (<key> <hex value>)...` at `t[i..]`: the pairs and the index after them.
fn read_kv(t: &[&str], i: usize) -> (Vec<(String, String)>, usize) {
    let n: usize = t[i].parse().unwrap();
    let mut kv = Vec::new();
    for k in 0..n {
        kv.push((t[i + 1 + 2 * k].to_string(), unhex(t[i + 2 + 2 * k])));
    }
    (kv, i + 1 + 2 * n)
}

fn kv_str(kv: &[(String, String)]) -> String {
    let mut s = kv.len().to_string();
    for (k, v) in kv {
        s += &format!(" {} {}", k, hex(v));
    }
    s
}

/// The configuration of the pairs over nothing (`DynamicPrintConfig` + `set_deserialize_strict`).
fn kv_cfg(kv: &[(String, String)]) -> DynamicPrintConfig {
    let mut c = DynamicPrintConfig::new();
    for (k, v) in kv {
        c.set_deserialize_strict(k, v, false).unwrap_or_else(|e| panic!("{k}: {e:?}"));
    }
    c
}

/// `print.config()` of a `Print` whose configuration the pairs were applied to: the defaults with
/// the pairs over them.
fn kv_print_cfg(kv: &[(String, String)]) -> DynamicPrintConfig {
    let mut c = DynamicPrintConfig::full_print_config();
    for (k, v) in kv {
        c.set_deserialize_strict(k, v, false).unwrap_or_else(|e| panic!("{k}: {e:?}"));
    }
    c
}

fn line_of(t: &[&str]) -> Option<String> {
    let tag = t[0];
    Some(match tag {
        "cgst" => {
            let g = unhex(t[1]);
            let (a, b, g10): (i32, i32, bool) = (t[2].parse().unwrap(), t[3].parse().unwrap(), t[4] == "1");
            let mut temp = 0;
            let r = custom_gcode_sets_temperature(&g, a, b, g10, &mut temp);
            format!("cgst {} {} {} {} {} {}", t[1], a, b, g10 as i32, r as i32, temp)
        }
        "b64" => {
            let v = u64::from_str_radix(t[1], 16).unwrap();
            format!("b64 {} {}", t[1], encode_base64(v))
        }
        "infer" => {
            let g = unhex(t[1]);
            let safe = Point::new(t[2].parse().unwrap(), t[3].parse().unwrap());
            let r = infer_timelapse_final_plate_pos(&g, &safe, f32of(t[4]));
            let p = r.unwrap_or([0.0; 3]);
            format!("infer {} {} {} {} {} {} {} {}", t[1], t[2], t[3], t[4], r.is_some() as i32, f32h(p[0]), f32h(p[1]), f32h(p[2]))
        }
        "eol" => {
            let mut s = unhex(t[1]);
            slicer::g_code::check_add_eol(&mut s);
            format!("eol {} {}", t[1], hex(&s))
        }
        "cgct" => {
            let g = unhex(t[1]);
            let prefix = unhex(t[2]);
            let next: u32 = t[3].parse().unwrap();
            format!("cgct {} {} {} {}", t[1], t[2], next, slicer::g_code::custom_gcode_changes_tool(&g, &prefix, next) as i32)
        }
        "tg" => {
            use slicer::g_code::Vec2f;
            let g = unhex(t[1]);
            let pos = Vec2f::new(f32of(t[2]), f32of(t[3]));
            let tr = Vec2f::new(f32of(t[4]), f32of(t[5]));
            let out = slicer::g_code::transform_gcode(&g, pos, &tr, f32of(t[6]));
            format!("tg {} {} {} {} {} {} {}", t[1], t[2], t[3], t[4], t[5], t[6], hex(&out))
        }
        "wapx" => {
            use slicer::g_code::Vec2f;
            let r = slicer::g_code::get_wipe_avoid_pos_x(
                &Vec2f::new(f32of(t[1]), f32of(t[2])),
                &Vec2f::new(f32of(t[3]), f32of(t[4])),
                f32of(t[5]),
            );
            format!("wapx {} {} {} {} {} {}", t[1], t[2], t[3], t[4], t[5], f32h(r))
        }
        "ovs" => {
            use slicer::extrusion_entity::{ExtrusionLoop, ExtrusionPath, ExtrusionRole};
            let slope = f64of(t[1]);
            let n: usize = t[2].parse().unwrap();
            let mut paths = Vec::new();
            let mut x: i64 = 0;
            let mut s = format!("ovs {} {}", t[1], n);
            for k in 0..n {
                let len: i64 = t[3 + 2 * k].parse().unwrap();
                let deg = f64of(t[4 + 2 * k]);
                let mut path = ExtrusionPath::new(ExtrusionRole::ExternalPerimeter);
                path.polyline = Polyline::from_points(vec![Point::new(x, 0), Point::new(x + len, 0)]);
                x += len;
                path.overhang_degree = deg;
                paths.push(path);
                s += &format!(" {} {}", len, t[4 + 2 * k]);
            }
            let l = ExtrusionLoop::new(paths, slicer::extrusion_entity::ExtrusionLoopRole::DEFAULT);
            format!("{} {}", s, has_overhang_path_on_slope(&l, slope) as i32)
        }
        "cmvs" => {
            let g = GCode::new();
            let co = unhex(t[3]);
            let r = g.calc_max_volumetric_speed(f64of(t[1]), f64of(t[2]), &co);
            format!("cmvs {} {} {} {}", t[1], t[2], t[3], f64h(r))
        }
        "label" => {
            let n: usize = t[1].parse().unwrap();
            let ids: Vec<usize> = (0..n).map(|k| t[2 + k].parse().unwrap()).collect();
            let m: usize = t[2 + n].parse().unwrap();
            let q: Vec<usize> = (0..m).map(|k| t[3 + n + k].parse().unwrap()).collect();
            let mut g = GCode::new();
            g.m_label_objects_ids = ids.clone();
            let (ok, out) = match g._encode_label_ids_to_base64(&q) {
                Ok(s) => (1, s),
                Err(e) => (0, e),
            };
            let mut s = format!("label {}", n);
            for v in &ids {
                s += &format!(" {}", v);
            }
            s += &format!(" {}", m);
            for v in &q {
                s += &format!(" {}", v);
            }
            format!("{} {} {}", s, ok, hex(&out))
        }
        "remain" => {
            let n: usize = t[1].parse().unwrap();
            let ids: Vec<usize> = (0..n).map(|k| t[2 + k].parse().unwrap()).collect();
            let m: usize = t[2 + n].parse().unwrap();
            let clears: Vec<usize> = (0..m).map(|k| t[3 + n + k].parse().unwrap()).collect();
            let mut g = GCode::new();
            g.m_label_objects_ids = ids.clone();
            g._init_remaining_objects_bitmap();
            for &c in &clears {
                g._clear_remaining_object(c);
            }
            let out = g._remaining_objects_gcode();
            let mut s = format!("remain {}", n);
            for v in &ids {
                s += &format!(" {}", v);
            }
            s += &format!(" {}", m);
            for v in &clears {
                s += &format!(" {}", v);
            }
            format!("{} {}", s, hex(&out))
        }
        "origin" => {
            let last = Point::new(t[1].parse().unwrap(), t[2].parse().unwrap());
            let n: usize = t[3].parse().unwrap();
            let pts: Vec<Point> = (0..n).map(|k| Point::new(t[4 + 2 * k].parse().unwrap(), t[5 + 2 * k].parse().unwrap())).collect();
            let i = 4 + 2 * n;
            let (ox, oy, nx, ny) = (f64of(t[i]), f64of(t[i + 1]), f64of(t[i + 2]), f64of(t[i + 3]));
            let mut g = GCode::new();
            g.m_origin = slicer::geometry::Vec2d::new(ox, oy);
            g.set_last_pos(last);
            g.m_wipe.path = Polyline::from_points(pts);
            g.set_origin(&slicer::geometry::Vec2d::new(nx, ny));
            let mut s = t[..i + 4].join(" ");
            s += &format!(" {} {}", g.last_pos().x, g.last_pos().y);
            for p in &g.m_wipe.path.points {
                s += &format!(" {} {}", p.x, p.y);
            }
            s
        }
        "afc" => {
            let (kv, _) = read_kv(t, 1);
            let mut s = String::new();
            GCode::append_full_config(&kv_cfg(&kv), &mut s);
            format!("afc {} {}", kv_str(&kv), hex(&s))
        }
        "vcg" => {
            let (kv, _) = read_kv(t, 1);
            let r = validate_custom_gcode(&kv_print_cfg(&kv));
            let mut s = format!("vcg {} {}", kv_str(&kv), r.len());
            for (a, b) in &r {
                s += &format!(" {} {}", hex(a), hex(b));
            }
            s
        }
        "mass" => {
            let weight = f64of(t[1]);
            let (kv, _) = read_kv(t, 2);
            let mut g = GCode::new();
            g.m_print_config = kv_print_cfg(&kv);
            let (mut y, mut mass) = (-1.0f64, -1.0f64);
            g.mass_load_limited_machine_acceleration(weight, &mut y, &mut mass);
            format!("mass {} {} {} {}", t[1], kv_str(&kv), f64h(y), f64h(mass))
        }
        "env" => {
            let extruder_id: i32 = t[1].parse().unwrap();
            let (kv, _) = read_kv(t, 2);
            let mut g = GCode::new();
            g.m_print_config = kv_print_cfg(&kv);
            g.m_config = kv_print_cfg(&kv);
            let mut proc_ = slicer::gcode::g_code_processor::GCodeProcessor::new();
            let path = std::env::temp_dir().join(format!("g_code_native_parity_env_{}.gcode", std::process::id()));
            {
                let f = std::fs::File::create(&path).unwrap();
                let mut file = GCodeOutputStream::new(Some(f), &mut proc_);
                g.print_machine_envelope(&mut file, extruder_id);
                file.flush();
                file.close();
            }
            let text = std::fs::read_to_string(&path).unwrap();
            let _ = std::fs::remove_file(&path);
            format!("env {} {} {}", extruder_id, kv_str(&kv), hex(&text))
        }
        "skirt" => {
            // skirt <fn> <loops> <has_skirt> <has_inf> <seq> <min_len> <kv...> <print_z> <lt_has_skirt>
            //       <n ext> <ext...> <n done> <done...> <last> -> ...
            let fnid: i32 = t[1].parse().unwrap();
            let loops: usize = t[2].parse().unwrap();
            let (kv, mut i) = read_kv(t, 7);
            let cfg = kv_print_cfg(&kv);
            let print = SkirtPrint {
                skirt_loops: loops,
                has_skirt: t[3] == "1",
                has_infinite_skirt: t[4] == "1",
                is_sequential_print: t[5] == "1",
                min_skirt_length: f32of(t[6]) as f64,
                config: &cfg,
            };
            let print_z = f64of(t[i]);
            let lt_has_skirt = t[i + 1] == "1";
            let n: usize = t[i + 2].parse().unwrap();
            let ext: Vec<u32> = (0..n).map(|k| t[i + 3 + k].parse().unwrap()).collect();
            i += 3 + n;
            let m: usize = t[i].parse().unwrap();
            let mut done: Vec<f64> = (0..m).map(|k| f64of(t[i + 1 + k])).collect();
            i += 1 + m;
            let mut last: u32 = t[i].parse().unwrap();
            let mut lt = slicer::gcode::tool_ordering::LayerTools::new(print_z);
            lt.has_skirt = lt_has_skirt;
            lt.extruders = ext;
            let mut out = std::collections::BTreeMap::new();
            match fnid {
                0 => skirt_loops_per_extruder_all_printing(&print, &lt, &mut out),
                1 => out = make_skirt_loops_multi_diameter(&print, &lt, &mut done, &mut last),
                2 => out = make_skirt_loops_per_extruder_1st_layer(&print, &lt, &mut done),
                _ => out = make_skirt_loops_per_extruder_other_layers(&print, &lt, &mut done),
            }
            let mut s = format!("{} -> {}", t[..i + 1].join(" "), out.len());
            for (e, (a, b)) in &out {
                s += &format!(" {} {} {}", e, a, b);
            }
            s += &format!(" {}", done.len());
            for d in &done {
                s += &format!(" {}", f64h(*d));
            }
            s += &format!(" {}", last);
            s
        }
        "est" => {
            // est <normal> <stealth> <enabled> <kv> <6 results> | <n> (<id> <volume>)* <n> (<id> <volume>)*
            let (normal, stealth, enabled) = (f32of(t[1]), f32of(t[2]), t[3] == "1");
            let (kv, i) = read_kv(t, 4);
            let j = i + 7;
            let n: usize = t[j].parse().unwrap();
            let total: std::collections::BTreeMap<usize, f64> =
                (0..n).map(|k| (t[j + 1 + 2 * k].parse().unwrap(), f64of(t[j + 2 + 2 * k]))).collect();
            let k0 = j + 1 + 2 * n;
            let m: usize = t[k0].parse().unwrap();
            let model: std::collections::BTreeMap<usize, f64> =
                (0..m).map(|k| (t[k0 + 1 + 2 * k].parse().unwrap(), f64of(t[k0 + 2 + 2 * k]))).collect();
            let mut proc_ = slicer::gcode::g_code_processor::GCodeProcessor::new();
            proc_.result_mut().print_statistics.modes[0].time = normal;
            proc_.result_mut().print_statistics.modes[1].time = stealth;
            proc_.result_mut().print_statistics.total_volumes_per_extruder = total.clone();
            proc_.result_mut().print_statistics.model_volumes_per_extruder = model;
            proc_.enable_stealth_time_estimator(enabled);
            let mut w = slicer::g_code_writer::GCodeWriter::new();
            w.apply_print_config(&kv_print_cfg(&kv));
            w.set_extruders(vec![0, 1, 2]);
            let mut ps = slicer::print::faithful::PrintStatistics::new();
            update_print_estimated_stats(&proc_, w.extruders(), &mut ps);
            let mut s = format!(
                "est {} {} {} {} {} {} {} {} {} {} | {}",
                t[1],
                t[2],
                t[3],
                kv_str(&kv),
                hex(&ps.estimated_normal_print_time),
                hex(&ps.estimated_silent_print_time),
                f64h(ps.total_extruded_volume),
                f64h(ps.total_used_filament),
                f64h(ps.total_weight),
                f64h(ps.total_cost),
                total.len()
            );
            for (id, v) in &total {
                s += &format!(" {} {}", id, f64h(*v));
            }
            s += &format!(" {}", ps.filament_stats.len());
            for (id, v) in &ps.filament_stats {
                s += &format!(" {} {}", id, f64h(*v));
            }
            s
        }
        "stats" => {
            // stats <has> <toolchanges> <n> <used>* <kv> <n> <extrude>* -> <out> <7 results>
            let has = t[1] == "1";
            let toolchanges: i32 = t[2].parse().unwrap();
            let n: usize = t[3].parse().unwrap();
            let used: Vec<f32> = (0..n).map(|k| f32of(t[4 + k])).collect();
            let (kv, i) = read_kv(t, 4 + n);
            let m: usize = t[i].parse().unwrap();
            let extrude: Vec<f64> = (0..m).map(|k| f64of(t[i + 1 + k])).collect();
            let mut w = slicer::g_code_writer::GCodeWriter::new();
            w.apply_print_config(&kv_print_cfg(&kv));
            w.set_extruders(vec![0, 1, 2]);
            for (k, x) in extrude.iter().enumerate() {
                w.toolchange(k as u32, 0);
                w.extrude_to_xy(slicer::geometry::Vec2d::new(10.0 * k as f64, 5.0), *x, "", false);
                w.retract(false);
            }
            let mut wtd = slicer::print::faithful::WipeTowerData::new();
            wtd.number_of_toolchanges = toolchanges;
            wtd.used_filament = used;
            let mut ps = slicer::print::faithful::PrintStatistics::new();
            ps.total_weight = 99.0;
            let out = update_print_stats_and_format_filament_stats(has, &wtd, w.extruders(), &mut ps);
            format!(
                "{} {} {} {} {} {} {} {} {}",
                t[..i + 1 + m].join(" "),
                hex(&out),
                ps.total_toolchanges,
                f64h(ps.total_weight),
                f64h(ps.total_cost),
                f64h(ps.total_used_filament),
                f64h(ps.total_extruded_volume),
                f64h(ps.total_wipe_tower_filament),
                f64h(ps.total_wipe_tower_cost)
            )
        }
        "pts" => {
            let (ox, oy) = (f64of(t[1]), f64of(t[2]));
            let n: usize = t[3].parse().unwrap();
            let pts: Vec<Point> = (0..n).map(|k| Point::new(t[4 + 2 * k].parse().unwrap(), t[5 + 2 * k].parse().unwrap())).collect();
            let mut print = slicer::print::faithful::Print::new();
            print.set_plate_origin([ox, oy, 0.0]);
            let poly = slicer::geometry::Polygon::from_points(pts);
            format!("{} {}", t[..4 + 2 * n].join(" "), hex(&polygon_to_string(&poly, &print)))
        }
        "apc" => {
            let (kv, _) = read_kv(t, 1);
            let mut g = GCode::new();
            g.apply_print_config(&kv_print_cfg(&kv));
            let m = g.m_writer.parity_members();
            let flavor = (0..13).find(|&c| slicer::extruder::gcode_flavor_from_int(c) == g.m_writer.config.gcode_flavor).unwrap();
            format!(
                "apc {} {} {} {} {} {} {} {} {}",
                kv_str(&kv),
                f64h(g.m_scaled_resolution),
                m.max_acceleration,
                f64h(m.max_jerk),
                m.single_extruder_multi_material as i32,
                flavor,
                hex(&g.m_config.opt_serialize("z_hop").unwrap()),
                hex(&g.m_config.opt_serialize("resolution").unwrap()),
                hex(&g.m_config.opt_serialize("printable_area").unwrap())
            )
        }
        "ctor" => {
            // GCode() (GCode.hpp:175-203, :544 `FullPrintConfig m_config`): object and region
            // options at their defaults, before and after apply_print_config.
            let applied = t[1] == "1";
            let mut g = GCode::new();
            if applied {
                let kv = vec![("travel_speed".to_string(), "321".to_string()), ("resolution".to_string(), "0.02".to_string())];
                g.apply_print_config(&kv_print_cfg(&kv));
            }
            let mut s = format!("ctor {} {}", t[1], g.m_config.keys().len());
            let mut i = 3;
            while i + 1 < t.len() {
                let k = t[i];
                s += &format!(" {} {}", k, hex(&g.m_config.opt_serialize(k).unwrap_or_default()));
                i += 2;
            }
            s
        }
        "flp" => {
            let n: usize = t[1].parse().unwrap();
            let pts: Vec<Point> = (0..n).map(|k| Point::new(t[2 + 2 * k].parse().unwrap(), t[3 + 2 * k].parse().unwrap())).collect();
            let mut print = slicer::print::faithful::Print::new();
            print.m_first_layer_convex_hull = slicer::geometry::Polygon::from_points(pts);
            let g = GCode::new();
            let b = g.first_layer_projection(&print);
            format!(
                "{} {} {} {} {} {}",
                t[..2 + 2 * n].join(" "),
                b.is_defined() as i32,
                f64h(b.min.x),
                f64h(b.min.y),
                f64h(b.max.x),
                f64h(b.max.y)
            )
        }
        "wipe" => {
            // Wipe::wipe (GCode.cpp:440-525), not a tool change.
            let (kv, i) = read_kv(t, 1);
            let cur = f64of(t[i]);
            let n: usize = t[i + 1].parse().unwrap();
            let pts: Vec<Point> = (0..n).map(|k| Point::new(t[i + 2 + 2 * k].parse().unwrap(), t[i + 3 + 2 * k].parse().unwrap())).collect();
            // GCode.hpp:544 `FullPrintConfig m_config`: GCode::new starts with every option at its
            // default, so the region options Wipe::wipe reads (`wipe_speed`,
            // `role_base_wipe_speed`, not PrintConfig's) have their defaults after
            // apply_print_config, as at the pin.
            let mut g = GCode::new();
            g.apply_print_config(&kv_print_cfg(&kv));
            g.m_writer.set_extruders(vec![0]);
            g.m_writer.toolchange(0, 0);
            if cur > 0.0 {
                g.m_writer.set_speed(cur, "", "");
            }
            g.set_last_pos(pts[0]);
            g.m_wipe.path = Polyline::from_points(pts);
            let out = Wipe::wipe(&mut g, false, false);
            let mut s = t[..i + 2 + 2 * n].join(" ");
            s += &format!(" {} {} {} {}", hex(&out), g.last_pos().x, g.last_pos().y, g.m_wipe.has_path() as i32);
            s
        }
        _ => return None,
    })
}

#[test]
fn g_code_functions() {
    let text = read_vectors();
    let mut n = 0usize;
    let mut bad: Vec<String> = Vec::new();
    let mut per_tag: std::collections::BTreeMap<String, usize> = std::collections::BTreeMap::new();
    for line in text.lines() {
        let t: Vec<&str> = line.split(' ').collect();
        if let Some(got) = line_of(&t) {
            n += 1;
            *per_tag.entry(t[0].to_string()).or_default() += 1;
            if got != line {
                bad.push(format!("  want {line}\n  got  {got}"));
            }
        }
    }
    assert!(n >= 300, "only {n} lines: {per_tag:?}");
    assert!(bad.is_empty(), "{} of {} lines differ:\n{}", bad.len(), n, bad.join("\n"));
}

// GCode.cpp:6884-6889, 7869-7902. Compare consecutive multipaths emitted
// by one exporter, including the first paths after two tower visits.
#[test]
fn multipath_tower_analyzer_tags_match_native() {
    use slicer::extrusion_entity::{
        ExtrusionEntityCollection, ExtrusionEntityType, ExtrusionMultiPath, ExtrusionPath, ExtrusionRole,
    };
    use slicer::g_code::extrude_collection;
    use slicer::GCodeWriter;
    use slicer::print_config::PrintObjectConfig;

    let mut writer = GCodeWriter::new();
    let mut x = 0;
    for row in include_str!("data/g_code/multipath_tower_tags.txt").lines() {
        let fields: Vec<_> = row.split_whitespace().collect();
        if fields[0] == "1" {
            writer.set_force_analyzer_tags();
        }
        let count: usize = fields[1].parse().unwrap();
        let mut paths = ExtrusionMultiPath::new();
        for width in &fields[2..2 + count] {
            let mut path = ExtrusionPath::new(ExtrusionRole::InternalInfill);
            path.mm3_per_mm = 0.1;
            path.width = f64::from(f32of(width));
            path.height = f64::from(0.3_f32);
            path.polyline = Polyline::from_points(vec![
                Point::new(x, 0),
                Point::new(x + slicer::scale(1.0), 0),
            ]);
            x += slicer::scale(1.0);
            paths.paths.push(path);
        }
        let mut collection = ExtrusionEntityCollection::new();
        collection.no_sort = true;
        collection.entities.push(ExtrusionEntityType::MultiPath(paths));
        let mark = writer.get_gcode().content().len();
        extrude_collection(&collection, &mut writer, &PrintObjectConfig::default(), false).unwrap();
        let content = writer.get_gcode().content();
        let gcode = &content[mark..];
        let tags: Vec<_> = gcode
            .lines()
            .filter(|s| s.starts_with("; LINE_WIDTH: ") || s.starts_with("; LAYER_HEIGHT: "))
            .collect();
        let expected = unhex(fields[2 + count]);
        assert_eq!(tags, expected.lines().collect::<Vec<_>>(), "{row}");
    }
}

// GCodeWriter.cpp:881-946, Extruder.cpp:101-146. Compare each move and
// outstanding length, including an equal or shorter request before repayment.
#[test]
fn repeated_retractions_match_native() {
    use slicer::print_config::PrintConfig;
    use slicer::GCodeWriter;

    for line in include_str!("data/g_code/repeated_retractions.txt").lines() {
        let mut fields = line.split_whitespace();
        let config = PrintConfig {
            use_relative_e: fields.next().unwrap() == "1",
            retract_length: fields.next().unwrap().parse().unwrap(),
            retract_length_toolchange: fields.next().unwrap().parse().unwrap(),
            retract_speed: 30.0,
            deretract_speed: 25.0,
            retract_lift: 0.0,
            ..PrintConfig::default()
        };
        let mut writer = GCodeWriter::with_config(config);
        for step in fields.next().unwrap().chars() {
            let start = writer.get_gcode().content().len();
            match step {
                'N' => writer.retract(),
                'T' => writer.retract_for_toolchange(),
                'U' => writer.unretract(),
                _ => panic!("unknown retraction step {step}"),
            }
            let expected_gcode = unhex(fields.next().unwrap());
            let expected_length = f64of(fields.next().unwrap());
            assert_eq!(&writer.get_gcode().content()[start..], expected_gcode, "{step} in {line}");
            assert_eq!(writer.retracted_length_now(), expected_length, "{step} in {line}");
        }
        assert!(fields.next().is_none());
    }
}

// GCode.hpp 295–327 and GCode.cpp 7106–7148. Verify the bytes delivered
// to the output file and the moves received by its borrowed processor.
#[test]
fn output_stream_matches_native_file_and_processor() {
    use std::io::{Seek, SeekFrom};
    use slicer::gcode::g_code_processor::GCodeProcessor;

    let mut failures = Vec::new();
    let mut cases = 0;
    for row in include_str!("data/g_code/output_streams.txt").lines() {
        let fields: Vec<_> = row.split_ascii_whitespace().collect();
        let input = unhex(fields[2]);
        let mut output = tempfile::tempfile().unwrap();
        let mut processor = GCodeProcessor::new();
        let (open, error, still_open);
        {
            let mut stream = GCodeOutputStream::new(Some(output.try_clone().unwrap()), &mut processor);
            match fields[1] {
                "W" => stream.write(&input),
                "L" => stream.writeln(&input),
                "F" => stream.write_format(format_args!("{input}")),
                op => panic!("unknown stream operation {op}"),
            }
            stream.writeln("G1 X1 Y2 F900");
            stream.flush();
            (open, error) = (stream.is_open(), stream.is_error());
            stream.close();
            still_open = stream.is_open();
        }
        output.seek(SeekFrom::Start(0)).unwrap();
        let mut file = String::new();
        output.read_to_string(&mut file).unwrap();
        let mut actual = format!("{} {} {} {}", hex(&file), u8::from(open), u8::from(error), u8::from(still_open));
        for m in &processor.result().moves {
            actual += &format!(" {}:{}:{}", m.gcode_id, f32h(m.position[0]), f32h(m.position[1]));
        }
        if actual != fields[3..].join(" ") {
            failures.push((cases, fields[1], actual));
        }
        cases += 1;
    }
    assert_eq!(cases, 27);
    assert!(failures.is_empty(), "{failures:?}");
}
