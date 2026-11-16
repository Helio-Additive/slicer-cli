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
                // SAFETY: the processor outlives the stream.
                let mut file = unsafe { GCodeOutputStream::new(Some(f), &mut proc_) };
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
