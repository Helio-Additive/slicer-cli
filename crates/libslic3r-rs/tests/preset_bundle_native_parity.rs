//! `slicer::preset_bundle` (the port of libslic3r/PresetBundle.hpp and PresetBundle.cpp) against the
//! C++ at the pin.
//!
//! `tests/data/preset_bundle/preset_bundle.txt.gz` is the output of `gen.cpp` beside it (gzip -9n),
//! which links the pin's PresetBundle.cpp and runs one scenario: the free functions of the unit,
//! then a PresetBundle loading a small vendor bundle and user presets from a scratch directory
//! (load_presets), its full configs, the helpers, selections exported to an AppConfig, AMS counts,
//! selections and syncs, filament count changes, a printer switch, the nozzle-diameter functions,
//! default replacement, external (project) configs, import and export of presets, cloud user
//! presets, a copy and a reset. This test runs the same scenario on the port; every line has to be
//! the same, byte for byte. The header of gen.cpp describes the lines.

use slicer::app_config::AppConfig;
use slicer::config::*;
use slicer::preset::*;
use slicer::preset_bundle::*;
use slicer::print_config::config_def::{save_extruder_ams_count_to_string, DynamicPrintConfig};
use slicer::print_config::enums::NozzleVolumeType;
use slicer::semver::Semver;
use std::collections::{BTreeMap, BTreeSet};
use std::io::Read;

fn read_vectors() -> String {
    let path = format!("{}/tests/data/preset_bundle/preset_bundle.txt.gz", env!("CARGO_MANIFEST_DIR"));
    let file = std::fs::File::open(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
    let mut text = String::new();
    flate2::read::GzDecoder::new(file).read_to_string(&mut text).expect("gzip");
    text
}

fn hex(s: &str) -> String {
    if s.is_empty() {
        return "-".to_string();
    }
    let mut o = String::with_capacity(s.len() * 2);
    for c in s.as_bytes() {
        o.push_str(&format!("{:02x}", c));
    }
    o
}
fn b(v: bool) -> String {
    (v as i32).to_string()
}
fn num(v: i64) -> String {
    v.to_string()
}
fn s(v: &str) -> String {
    v.to_string()
}
fn idx(i: usize) -> String {
    if i == usize::MAX {
        "-1".to_string()
    } else {
        i.to_string()
    }
}
fn norm_time(t: i64) -> String {
    if t > 1_500_000_000 {
        "NOW".to_string()
    } else {
        t.to_string()
    }
}
/// printf("%.17g", v)
fn dbl(v: f64) -> String {
    if v == 0.0 {
        return if v.is_sign_negative() { "-0".to_string() } else { "0".to_string() };
    }
    if !v.is_finite() {
        return if v.is_nan() { "nan".to_string() } else if v > 0.0 { "inf".to_string() } else { "-inf".to_string() };
    }
    const P: i32 = 17;
    let e = format!("{:.*e}", (P - 1) as usize, v);
    let (mant, exp) = e.split_once('e').unwrap();
    let x: i32 = exp.parse().unwrap();
    let strip = |t: String| -> String {
        if t.contains('.') {
            t.trim_end_matches('0').trim_end_matches('.').to_string()
        } else {
            t
        }
    };
    if x < -4 || x >= P {
        let m = strip(mant.to_string());
        format!("{}e{}{:02}", m, if x < 0 { '-' } else { '+' }, x.abs())
    } else {
        strip(format!("{:.*}", (P - 1 - x) as usize, v))
    }
}

/// The scenario's state: the scratch directory and the lines.
struct Out {
    d: String,
    lines: Vec<String>,
    files: BTreeMap<String, String>,
}

impl Out {
    fn emit(&mut self, t: Vec<String>) {
        self.lines.push(t.join(" "));
    }
    fn rel(&self, p: &str) -> String {
        let r = p.replace(&self.d, "<D>");
        // The vectors were made on a POSIX system.
        if cfg!(windows) {
            r.replace('\\', "/")
        } else {
            r
        }
    }
    fn err(&mut self, what: &str, e: &ConfigurationError) {
        let t = self.rel(&format!("!{}", e.what()));
        self.emit(vec![s(what), hex(&t)]);
    }
}

fn push_list(l: &mut Vec<String>, v: &[String]) {
    l.push(v.len().to_string());
    for x in v {
        l.push(hex(x));
    }
}
fn ser(opt: &dyn ConfigOption) -> String {
    match opt.serialize() {
        Ok(v) => v,
        Err(e) => format!("!{}", e.what()),
    }
}
fn push_cfg(o: &Out, l: &mut Vec<String>, c: &dyn ConfigBase) {
    let keys = c.keys();
    l.push(keys.len().to_string());
    for k in &keys {
        l.push(hex(k));
        l.push(hex(&o.rel(&ser(c.option(k).unwrap()))));
    }
}
/// gen.cpp cfg_hash(): FNV-1a 64 of the keys and serialized values.
fn cfg_hash(o: &Out, c: &dyn ConfigBase) -> String {
    let mut h: u64 = 0xcbf29ce484222325;
    let mut add = |s: &[u8]| {
        for &ch in s {
            h ^= ch as u64;
            h = h.wrapping_mul(0x100000001b3);
        }
    };
    for k in c.keys() {
        add(k.as_bytes());
        add(b"\x01");
        add(o.rel(&ser(c.option(&k).unwrap())).as_bytes());
        add(b"\x02");
    }
    format!("{:016x}", h)
}
fn inherits_of(p: &Preset) -> String {
    match p.config.option("inherits").and_then(|o| o.as_any().downcast_ref::<ConfigOptionString>()) {
        Some(o) => o.value.clone(),
        None => "<none>".to_string(),
    }
}
fn push_preset(o: &Out, l: &mut Vec<String>, p: &Preset) {
    l.push(hex(&p.name));
    l.push(hex(&o.rel(&p.file)));
    l.push(num(p.r#type as i64));
    l.push(b(p.is_default));
    l.push(b(p.is_external));
    l.push(b(p.is_system));
    l.push(b(p.is_visible));
    l.push(b(p.is_dirty));
    l.push(b(p.is_compatible));
    l.push(b(p.is_project_embedded));
    l.push(b(p.loaded));
    l.push(hex(&p.alias));
    l.push(hex(&inherits_of(p)));
    l.push(hex(&p.filament_id));
    l.push(hex(&p.setting_id));
    l.push(hex(&p.base_id));
    l.push(hex(&p.user_id));
    l.push(hex(&p.sync_info));
    l.push(hex(&p.description));
    l.push(hex(&p.version.to_string()));
    l.push(norm_time(p.updated_time));
    l.push(hex(&p.vendor.as_ref().map(|v| v.id.clone()).unwrap_or_default()));
    push_list(l, &p.renamed_from);
    l.push(num(p.config.keys().len() as i64));
    l.push(cfg_hash(o, &p.config));
}
fn dump_preset(o: &mut Out, tag: &str, p: &Preset) {
    let mut l = vec![s("p"), hex(tag)];
    push_preset(o, &mut l, p);
    o.emit(l);
}
fn dump_cfg(o: &mut Out, tag: &str, c: &dyn ConfigBase) {
    let mut l = vec![s("cfg"), hex(tag)];
    push_cfg(o, &mut l, c);
    o.emit(l);
}
fn dump_coll(o: &mut Out, tag: &str, c: &PresetCollection) {
    let path = o.rel(&c.path_from_name("x", false));
    o.emit(vec![
        s("coll"),
        hex(tag),
        num(c.size() as i64),
        num(c.num_default_presets() as i64),
        idx(c.get_selected_idx()),
        b(c.is_default_suppressed()),
        hex(&path),
        num(c.num_visible() as i64),
        b(c.has_defaults_only()),
    ]);
    for (i, p) in c.get_presets().iter().enumerate() {
        dump_preset(o, &format!("{}#{}", tag, i), p);
    }
    dump_preset(o, &format!("{}#edited", tag), c.get_edited_preset());
}
fn dump_vendors<'a>(o: &mut Out, tag: &str, vendors: impl Iterator<Item = (&'a String, &'a VendorProfile)>) {
    for (k, v) in vendors {
        let mut l = vec![
            s("vendor"),
            hex(tag),
            hex(k),
            hex(&v.name),
            hex(&v.id),
            hex(&v.config_version.to_string()),
            hex(&v.config_update_url),
            num(v.models.len() as i64),
        ];
        for m in &v.models {
            l.push(hex(&m.id));
            l.push(hex(&m.name));
            l.push(hex(&m.model_id));
            l.push(num(m.technology as i64));
            l.push(hex(&m.family));
            l.push(hex(&m.bed_model));
            l.push(hex(&m.bed_texture));
            l.push(num(m.variants.len() as i64));
            for var in &m.variants {
                l.push(hex(&var.name));
            }
            push_list(&mut l, &m.default_materials);
        }
        o.emit(l);
    }
}
fn dump_bundle(o: &mut Out, tag: &str, pb: &PresetBundle) {
    let mut l = vec![s("bundle"), hex(tag)];
    push_list(&mut l, &pb.filament_presets);
    push_list(&mut l, &save_extruder_ams_count_to_string(&pb.extruder_ams_counts));
    l.push(num(pb.calibrate_filaments.len() as i64));
    l.push(num(pb.incompatible_nozzle_diameter_slots.len() as i64));
    for s_ in &pb.incompatible_nozzle_diameter_slots {
        l.push(num(*s_ as i64));
    }
    l.push(num(pb.ams_multi_color_filment.len() as i64));
    for v in &pb.ams_multi_color_filment {
        push_list(&mut l, v);
    }
    l.push(num(pb.extruder_ams_selections.len() as i64));
    for sel in &pb.extruder_ams_selections {
        l.push(b(sel.initialized));
        l.push(hex(&sel.device_id));
        push_list(&mut l, &sel.ams4);
        push_list(&mut l, &sel.ams1);
    }
    push_list(&mut l, &pb.obsolete_presets.prints);
    push_list(&mut l, &pb.obsolete_presets.filaments);
    push_list(&mut l, &pb.obsolete_presets.printers);
    l.push(num(pb.ams_sync_substitutions.len() as i64));
    for sub in &pb.ams_sync_substitutions {
        l.push(num(sub.slot as i64));
        l.push(hex(&sub.tray));
        l.push(hex(&sub.wanted));
        l.push(hex(&sub.applied));
    }
    l.push(b(pb.has_defauls_only()));
    o.emit(l);
    dump_coll(o, &format!("{}:prints", tag), &pb.prints);
    dump_coll(o, &format!("{}:filaments", tag), &pb.filaments);
    dump_coll(o, &format!("{}:printers", tag), &pb.printers);
    dump_cfg(o, &format!("{}:project", tag), &pb.project_config);
    dump_vendors(o, tag, pb.vendors.iter().map(|(k, v)| (k, &**v)));
}
fn dump_app(o: &mut Out, tag: &str, config: &AppConfig) {
    for section in ["presets", "filaments", "nozzle_volume_types", "nozzle_slots"] {
        let mut l = vec![s("app"), hex(tag), hex(section), b(config.has_section(section))];
        if config.has_section(section) {
            let m = config.get_section(section);
            l.push(num(m.len() as i64));
            for (k, v) in m {
                l.push(hex(k));
                l.push(hex(v));
            }
        }
        o.emit(l);
    }
    o.emit(vec![
        s("app_keys"),
        hex(tag),
        hex(&config.get_key("flush_multiplier")),
        hex(&config.get_key("flush_multiplier_fast")),
        hex(&config.get_key("preset_folder")),
    ]);
}
/// gen.cpp norm_content()
fn norm_content(s: &str) -> String {
    let mut out = String::new();
    let mut pos = 0usize;
    while pos < s.len() {
        let e = s[pos..].find('\n').map(|e| e + pos);
        let mut line = s[pos..e.unwrap_or(s.len())].to_string();
        if let Some(rest) = line.strip_prefix("updated_time = ") {
            let t: i64 = c_atoll(rest);
            line = format!("updated_time = {}", norm_time(t));
        }
        out.push_str(&line);
        match e {
            None => break,
            Some(e) => {
                out.push('\n');
                pos = e + 1;
            }
        }
    }
    out
}
fn c_atoll(s: &str) -> i64 {
    let b = s.as_bytes();
    let mut i = 0;
    while i < b.len() && (b[i] == b' ' || b[i] == b'\t') {
        i += 1;
    }
    let mut neg = false;
    if i < b.len() && (b[i] == b'-' || b[i] == b'+') {
        neg = b[i] == b'-';
        i += 1;
    }
    let mut v: i64 = 0;
    while i < b.len() && b[i].is_ascii_digit() {
        v = v * 10 + (b[i] - b'0') as i64;
        i += 1;
    }
    if neg {
        -v
    } else {
        v
    }
}
fn write_file(path: &str, content: &str) {
    std::fs::create_dir_all(std::path::Path::new(path).parent().unwrap()).unwrap();
    std::fs::write(path, content).unwrap();
}
fn walk(dir: &std::path::Path, out: &mut Vec<String>) {
    if let Ok(rd) = std::fs::read_dir(dir) {
        for e in rd {
            let e = e.unwrap();
            let p = e.path();
            if p.is_dir() {
                walk(&p, out);
            } else if p.is_file() {
                out.push(p.to_string_lossy().into_owned());
            }
        }
    }
}
fn dump_files(o: &mut Out, tag: &str) {
    let mut files = Vec::new();
    walk(std::path::Path::new(&o.d), &mut files);
    files.sort();
    o.emit(vec![s("files"), hex(tag), num(files.len() as i64)]);
    let mut now: BTreeMap<String, String> = BTreeMap::new();
    for f in &files {
        let r = o.rel(f);
        let c = norm_content(&String::from_utf8_lossy(&std::fs::read(f).unwrap()));
        now.insert(r.clone(), c.clone());
        if o.files.get(&r) != Some(&c) {
            let rc = o.rel(&c);
            o.emit(vec![s("file"), hex(tag), hex(&r), hex(&rc)]);
        }
    }
    let gone: Vec<String> = o.files.keys().filter(|k| !now.contains_key(*k)).cloned().collect();
    for g in gone {
        o.emit(vec![s("gone"), hex(tag), hex(&g)]);
    }
    o.files = now;
}
fn strs(v: &[&str]) -> Vec<String> {
    v.iter().map(|x| x.to_string()).collect()
}
fn strs_opt(v: &[&str]) -> Box<dyn ConfigOption> {
    Box::new(ConfigOptionStrings::from_values(strs(v)))
}
fn tray(filament_id: &str, colour: &str, tray_name: &str, ty: &str, ams_id: &str, slot_id: &str, changed: bool) -> DynamicPrintConfig {
    let mut c = DynamicPrintConfig::new();
    c.set_key_value("filament_id", strs_opt(&[filament_id]));
    c.set_key_value("filament_colour", strs_opt(&[colour]));
    c.set_key_value("filament_colour_type", strs_opt(&["1"]));
    c.set_key_value("tray_name", strs_opt(&[tray_name]));
    c.set_key_value("filament_type", strs_opt(&[ty]));
    c.set_key_value("ams_id", strs_opt(&[ams_id]));
    c.set_key_value("slot_id", strs_opt(&[slot_id]));
    c.set_key_value("filament_multi_colour", strs_opt(&[colour]));
    c.set_key_value("filament_changed", Box::new(ConfigOptionBool::with_value(changed)));
    c
}
fn dump_maps(o: &mut Out, tag: &str, maps: &BTreeMap<i32, AMSMapInfo>) {
    let mut l = vec![s("maps"), hex(tag), num(maps.len() as i64)];
    for (k, v) in maps {
        l.push(num(*k as i64));
        l.push(hex(&v.ams_id));
        l.push(hex(&v.slot_id));
        l.push(hex(&v.nozzle_diameter));
        l.push(match v.nozzle_volume_type {
            Some(t) => num(t as i64),
            None => s("none"),
        });
    }
    o.emit(l);
}
fn map_info(ams_id: &str, slot_id: &str) -> AMSMapInfo {
    AMSMapInfo { ams_id: ams_id.to_string(), slot_id: slot_id.to_string(), ..Default::default() }
}
fn preset_at(c: &PresetCollection, i: usize) -> &Preset {
    if i == EDITED_PRESET_IDX {
        c.get_edited_preset()
    } else {
        &c.m_presets[i]
    }
}

// ---------------------------------------------------------------------------------------------
// The vendor bundle and the user presets (gen.cpp write_vendor / write_user).

const VERSION: &str = "01.02.03.04";

fn write_vendor(sys: &str) {
    write_file(
        &format!("{}/Acme.json", sys),
        r#"{
    "name": "Acme",
    "version": "01.02.03.04",
    "force_update": "0",
    "description": "Acme test vendor",
    "url": "https://example.invalid/acme",
    "machine_model_list": [{"name": "Acme X1", "sub_path": "machine/Acme X1.json"}, {"name": "Missing", "sub_path": "machine/none.json"}],
    "process_list": [{"name": "fdm_process_common", "sub_path": "process/fdm_process_common.json"},
                     {"name": "0.20mm Standard @Acme X1", "sub_path": "process/0.20mm Standard @Acme X1.json"},
                     {"name": "0.12mm Fine @Acme X1", "sub_path": "process/0.12mm Fine @Acme X1.json"}],
    "filament_list": [{"name": "fdm_filament_common", "sub_path": "filament/fdm_filament_common.json"},
                      {"name": "Acme PLA @Acme X1", "sub_path": "filament/Acme PLA @Acme X1.json"},
                      {"name": "Acme PETG @Acme X1", "sub_path": "filament/Acme PETG @Acme X1.json"},
                      {"name": "Acme TPU @Acme X1 0.4", "sub_path": "filament/Acme TPU @Acme X1 0.4.json"},
                      {"name": "Generic PLA @Acme X1", "sub_path": "filament/Generic PLA @Acme X1.json"}],
    "machine_list": [{"name": "fdm_machine_common", "sub_path": "machine/fdm_machine_common.json"},
                     {"name": "Acme X1 0.4 nozzle", "sub_path": "machine/Acme X1 0.4 nozzle.json"},
                     {"name": "Acme X1 0.6 nozzle", "sub_path": "machine/Acme X1 0.6 nozzle.json"},
                     {"name": "Acme X1 0.4 rack", "sub_path": "machine/Acme X1 0.4 rack.json"}]
}"#,
    );
    let a = format!("{}/Acme", sys);
    write_file(
        &format!("{}/machine/Acme X1.json", a),
        r#"{"type": "machine_model", "name": "Acme X1", "model_id": "AX1", "nozzle_diameter": "0.4;0.6",
        "machine_tech": "FFF", "family": "Acme", "bed_model": "acme_bed.stl", "bed_texture": "acme_tex.svg",
        "default_materials": "Acme PLA @Acme X1;Acme PETG @Acme X1;"}"#,
    );
    write_file(
        &format!("{}/machine/fdm_machine_common.json", a),
        r#"{"type": "machine", "name": "fdm_machine_common", "from": "system", "instantiation": "false",
        "printer_technology": "FFF", "gcode_flavor": "marlin", "printable_height": "240"}"#,
    );
    write_file(
        &format!("{}/machine/Acme X1 0.4 nozzle.json", a),
        r#"{"type": "machine", "name": "Acme X1 0.4 nozzle", "inherits": "fdm_machine_common",
        "from": "system", "setting_id": "AM001", "instantiation": "true", "printer_model": "Acme X1", "printer_variant": "0.4",
        "nozzle_diameter": ["0.4"], "default_print_profile": "0.20mm Standard @Acme X1", "default_filament_profile": ["Acme PLA @Acme X1"],
        "printable_area": ["0x0", "250x0", "250x250", "0x250"]}"#,
    );
    write_file(
        &format!("{}/machine/Acme X1 0.6 nozzle.json", a),
        r#"{"type": "machine", "name": "Acme X1 0.6 nozzle", "inherits": "fdm_machine_common",
        "from": "system", "setting_id": "AM002", "instantiation": "true", "printer_model": "Acme X1", "printer_variant": "0.6",
        "nozzle_diameter": ["0.6", "0.6"], "extruder_type": ["Direct Drive", "Direct Drive"],
        "extruder_variant_list": ["Direct Drive Standard,Direct Drive High Flow", "Direct Drive Standard,Direct Drive High Flow"],
        "printer_extruder_id": ["1", "1", "2", "2"],
        "printer_extruder_variant": ["Direct Drive Standard", "Direct Drive High Flow", "Direct Drive Standard", "Direct Drive High Flow"],
        "retraction_length": ["0.8", "1", "0.9", "1.1"], "nozzle_volume": ["100", "110", "120", "130"],
        "extruder_printable_area": ["0x0,250x0,250x250,0x250", "10x0,260x0,260x250,10x250"],
        "default_print_profile": "0.20mm Standard @Acme X1", "default_filament_profile": ["Acme PLA @Acme X1", "Acme PETG @Acme X1"]}"#,
    );
    write_file(
        &format!("{}/machine/Acme X1 0.4 rack.json", a),
        r#"{"type": "machine", "name": "Acme X1 0.4 rack", "inherits": "fdm_machine_common",
        "from": "system", "setting_id": "AM003", "instantiation": "true", "printer_model": "Acme X1", "printer_variant": "0.4",
        "nozzle_diameter": ["0.4"], "handle_hotend_as_extruder": "1", "extruder_max_nozzle_count": ["4"], "default_nozzle_volume_type": ["Standard"],
        "default_print_profile": "0.20mm Standard @Acme X1", "default_filament_profile": ["Acme PLA @Acme X1"]}"#,
    );
    write_file(
        &format!("{}/process/fdm_process_common.json", a),
        r#"{"type": "process", "name": "fdm_process_common", "from": "system", "instantiation": "false",
        "layer_height": "0.2", "print_extruder_id": ["1", "1", "2", "2"],
        "print_extruder_variant": ["Direct Drive Standard", "Direct Drive High Flow", "Direct Drive Standard", "Direct Drive High Flow"],
        "outer_wall_speed": ["200", "210", "220", "230"], "description": "common process"}"#,
    );
    write_file(
        &format!("{}/process/0.20mm Standard @Acme X1.json", a),
        r#"{"type": "process", "name": "0.20mm Standard @Acme X1", "inherits": "fdm_process_common",
        "from": "system", "setting_id": "AP001", "instantiation": "true", "wall_loops": "3",
        "compatible_printers": ["Acme X1 0.4 nozzle", "Acme X1 0.6 nozzle", "Acme X1 0.4 rack"]}"#,
    );
    write_file(
        &format!("{}/process/0.12mm Fine @Acme X1.json", a),
        r#"{"type": "process", "name": "0.12mm Fine @Acme X1", "inherits": "fdm_process_common",
        "from": "system", "setting_id": "AP002", "instantiation": "true", "layer_height": "0.12",
        "compatible_printers": ["Acme X1 0.4 nozzle"]}"#,
    );
    write_file(
        &format!("{}/filament/fdm_filament_common.json", a),
        r#"{"type": "filament", "name": "fdm_filament_common", "from": "system", "instantiation": "false",
        "filament_type": ["PLA"], "filament_extruder_variant": ["Direct Drive Standard", "Direct Drive High Flow"],
        "nozzle_temperature": ["210", "215"], "filament_flow_ratio": ["0.98", "0.97"]}"#,
    );
    write_file(
        &format!("{}/filament/Acme PLA @Acme X1.json", a),
        r#"{"type": "filament", "name": "Acme PLA @Acme X1", "inherits": "fdm_filament_common",
        "from": "system", "setting_id": "AF001", "filament_id": "AP1", "instantiation": "true", "filament_vendor": ["Acme"],
        "required_nozzle_HRC": ["3"], "compatible_printers": ["Acme X1 0.4 nozzle", "Acme X1 0.6 nozzle", "Acme X1 0.4 rack"]}"#,
    );
    write_file(
        &format!("{}/filament/Acme PETG @Acme X1.json", a),
        r#"{"type": "filament", "name": "Acme PETG @Acme X1", "inherits": "fdm_filament_common",
        "from": "system", "setting_id": "AF002", "filament_id": "AG1", "instantiation": "true", "filament_type": ["PETG"],
        "nozzle_temperature": ["240", "245"], "compatible_printers": ["Acme X1 0.4 nozzle", "Acme X1 0.6 nozzle", "Acme X1 0.4 rack"]}"#,
    );
    write_file(
        &format!("{}/filament/Acme TPU @Acme X1 0.4.json", a),
        r#"{"type": "filament", "name": "Acme TPU @Acme X1 0.4", "inherits": "fdm_filament_common",
        "from": "system", "setting_id": "AF003", "filament_id": "AT1", "instantiation": "true", "filament_type": ["TPU"],
        "filament_extruder_variant": ["Direct Drive Standard"], "nozzle_temperature": ["220"], "filament_flow_ratio": ["1"],
        "compatible_printers": ["Acme X1 0.4 nozzle"]}"#,
    );
    write_file(
        &format!("{}/filament/Generic PLA @Acme X1.json", a),
        r#"{"type": "filament", "name": "Generic PLA @Acme X1", "inherits": "fdm_filament_common",
        "from": "system", "setting_id": "AF004", "filament_id": "GP1", "instantiation": "true", "filament_vendor": ["Generic"],
        "compatible_printers": ["Acme X1 0.4 nozzle", "Acme X1 0.6 nozzle", "Acme X1 0.4 rack"]}"#,
    );
}

fn write_user(user: &str) {
    write_file(
        &format!("{}/filament/My PLA.json", user),
        &format!(
            r#"{{"version": "{}", "name": "My PLA", "from": "User", "inherits": "Acme PLA @Acme X1", "nozzle_temperature": ["225", "235"], "filament_settings_id": ["My PLA"]}}"#,
            VERSION
        ),
    );
    write_file(&format!("{}/filament/My PLA.info", user), "sync_info = \nuser_id = \nsetting_id = US1\nbase_id = AF001\nupdated_time = 1000\n");
    write_file(
        &format!("{}/process/My Process.json", user),
        &format!(
            r#"{{"version": "{}", "name": "My Process", "from": "User", "inherits": "0.20mm Standard @Acme X1", "layer_height": "0.16", "print_settings_id": "My Process"}}"#,
            VERSION
        ),
    );
    write_file(
        &format!("{}/machine/My Printer.json", user),
        &format!(
            r#"{{"version": "{}", "name": "My Printer", "from": "User", "inherits": "Acme X1 0.4 nozzle", "printer_settings_id": "My Printer", "printable_height": "200"}}"#,
            VERSION
        ),
    );
}

// ---------------------------------------------------------------------------------------------

fn gen_free_functions(o: &mut Out) {
    for t in 0..=6 {
        o.emit(vec![s("nvt_str"), num(t), hex(&GetNozzleVolumeTypeStr(NozzleVolumeType::from_i32(t as i32)))]);
    }
    {
        let mut info = FilamentBaseInfo::default();
        info.set_filament_extruder_compatibility(0x3a7);
        let mut l = vec![s("fbi"), num(info.get_filament_extruder_compatibility() as i64)];
        for e in -1..=11 {
            l.push(num(info.get_extruder_compatibility(e) as i64));
        }
        o.emit(l);
    }
    {
        let mut st = ExtruderNozzleStat::default();
        st.set_extruder_nozzle_count(1, NozzleVolumeType::NvtHighFlow, 2, false);
        st.set_extruder_nozzle_count(1, NozzleVolumeType::NvtStandard, 3, false);
        let mut l = vec![s("ens")];
        for e in -1..=2 {
            for t in [-1, 0, 1, 2, 3] {
                let vt = if t < 0 { None } else { Some(NozzleVolumeType::from_i32(t)) };
                l.push(num(st.get_extruder_nozzle_count(e, vt) as i64));
            }
        }
        st.on_volume_type_switch(1, NozzleVolumeType::NvtHighFlow);
        st.on_volume_type_switch(3, NozzleVolumeType::NvtStandard);
        st.on_volume_type_switch(0, NozzleVolumeType::NvtHybrid);
        for e in 0..=3 {
            l.push(num(st.get_extruder_nozzle_count(e, None) as i64));
        }
        st.on_printer_model_change_cli(&[1], &[2, 4]);
        for e in 0..=2 {
            for t in [0, 1] {
                l.push(num(st.get_extruder_nozzle_count(e, Some(NozzleVolumeType::from_i32(t))) as i64));
            }
        }
        o.emit(l);
        let full = build_extruder_full_stat_from_nozzle_stat(&st, &[3, 1], "0.4");
        let mut l2 = vec![s("full_stat"), num(full.get_extruder_count() as i64)];
        for e in 0..3usize {
            l2.push(num(full.get_extruder_nozzle_count(e) as i64));
            for p in full.get_extruder_stats(e) {
                l2.push(hex(&p.0));
                l2.push(hex(&p.1));
            }
        }
        o.emit(l2);
        let mut c = DynamicPrintConfig::new();
        o.emit(vec![s("per_slot_consistent"), b(per_slot_nozzle_options_consistent(&c))]);
        sync_per_slot_nozzle_options_from_full_stat(&mut c, &full).unwrap();
        dump_cfg(o, "per_slot", &c);
        o.emit(vec![s("per_slot_consistent"), b(per_slot_nozzle_options_consistent(&c))]);
        c.option_typed_mut::<ConfigOptionInts>("extruder_nozzle_count", false).unwrap().unwrap().values = vec![1, -1];
        o.emit(vec![s("per_slot_consistent"), b(per_slot_nozzle_options_consistent(&c))]);
    }
    {
        let mut c = DynamicPrintConfig::new();
        c.set_deserialize_strict("default_nozzle_volume_type", "Standard,High Flow", false).unwrap();
        c.set_deserialize_strict("extruder_max_nozzle_count", "3,1", false).unwrap();
        let mut l = vec![s("count_default_nvt")];
        for e in 0..3 {
            for n in [0, 1, 3] {
                let m = count_default_nozzle_volume_types(&c, e, n);
                l.push(num(m.len() as i64));
                for (k, v) in &m {
                    l.push(num(*k as i64));
                    l.push(num(*v as i64));
                }
            }
        }
        o.emit(l);
    }
    {
        // filament_name_map.json is read once.
        for (m, f) in [("Acme X1 0.6 nozzle", "Old PLA"), ("Acme X1 0.6 nozzle", "Other"), ("Nope", "Old PLA")] {
            let mut out = s("unchanged");
            let r = find_filament_preset_map_name(m, f, &mut out).unwrap();
            let mut f2 = f.to_string();
            convert_filament_preset_name(m, &mut f2).unwrap();
            o.emit(vec![s("filament_map_name"), hex(m), hex(f), b(r), hex(&out), hex(&f2)]);
        }
    }
}

type R = Result<(), ConfigurationError>;

fn gen_more(o: &mut Out, pb: &mut PresetBundle) {
    let d = o.d.clone();
    let res = format!("{}/resources", d);

    // temperatures of an AMS tray against a user root filament
    {
        write_file(
            &format!("{}/import2/Imported Root2.json", d),
            &format!(
                r#"{{"version": "{}", "name": "Imported Root2", "from": "User", "inherits": "", "filament_id": "IR2", "filament_settings_id": ["Imported Root2"], "compatible_printers": ["Acme X1 0.6 nozzle"], "nozzle_temperature_range_low": ["190"], "nozzle_temperature_range_high": ["250"]}}"#,
                VERSION
            ),
        );
        let mut files = vec![format!("{}/import2/Imported Root2.json", d)];
        let subs = pb.import_presets(&mut files, &mut |_| 1, ForwardCompatibilitySubstitutionRule::EnableSilent);
        let mut l = vec![s("import2"), num(subs.len() as i64)];
        for f in &files {
            l.push(hex(&o.rel(f)));
        }
        o.emit(l);
    }
    for (sid, tag, mn, mx) in [
        ("IR2", "1234", "190", "250"),
        ("IR2", "1234", "180", "250"),
        ("IR2", "0", "180", "250"),
        ("IR2", "123456789012010A", "180", "260"),
        ("ZZ", "1234", "1", "2"),
        ("IR2", "1234", "abc", "250"),
    ] {
        let mut nozzle = s("0.6");
        let mut sid_ = s(sid);
        let mut tag_ = s(tag);
        let mut mn_ = s(mn);
        let mut mx_ = s(mx);
        let mut psid = String::new();
        match pb.check_filament_temp_equation_by_printer_type_and_nozzle_for_mas_tray("Acme X1", &mut nozzle, &mut sid_, &mut tag_, &mut mn_, &mut mx_, &mut psid) {
            Ok(r) => o.emit(vec![s("temp_equation"), hex(sid), hex(tag), b(r), hex(&mn_), hex(&mx_), hex(&psid)]),
            Err(e) => o.err("temp_equation", &e),
        }
    }

    // the vendor bundle of the resources
    write_vendor(&format!("{}/profiles", res));
    {
        let mut other = PresetBundle::new();
        match other.load_system_models_from_json(ForwardCompatibilitySubstitutionRule::EnableSystemSilent) {
            Ok(r) => {
                let e = o.rel(&r.1);
                o.emit(vec![s("system_models"), num(r.0.len() as i64), hex(&e)]);
                dump_vendors(o, "system_models", other.vendors.iter().map(|(k, v)| (k, &**v)));
                dump_coll(o, "system_models:printers", &other.printers);
            }
            Err(e) => o.err("system_models", &e),
        }
    }
    {
        let mut other = PresetBundle::new();
        match other.load_system_filaments_json(ForwardCompatibilitySubstitutionRule::EnableSilentDisableSystem) {
            Ok(r) => {
                let e = o.rel(&r.1);
                o.emit(vec![s("system_filaments"), num(r.0.len() as i64), hex(&e)]);
                dump_vendors(o, "system_filaments", other.vendors.iter().map(|(k, v)| (k, &**v)));
                dump_coll(o, "system_filaments:filaments", &other.filaments);
                dump_coll(o, "system_filaments:prints", &other.prints);
            }
            Err(e) => o.err("system_filaments", &e),
        }
    }

    // model files and vendor errors
    write_file(&format!("{}/models/bad.json", d), "{ nope");
    write_file(&format!("{}/models/num.json", d), r#"{"name": 5}"#);
    write_file(
        &format!("{}/models/good.json", d),
        r#"{"name": "Good", "model_id": "G1", "nozzle_diameter": "0.4;0.4;0.8", "machine_tech": "SLA", "family": "F",
        "bed_model": "b.stl", "bed_texture": "t.svg", "hotend_model": "h.stl", "default_bed_type": "Cool Plate", "use_rect_grid": "true",
        "default_materials": "B;A;A;", "not_support_bed_type": ";X", "bottom_texture_rect_longer_ignore_list": "Z;Y", "support_side_panel_fan": "false",
        "resource_bind_name": "R", "version": "1", "url": "u"}"#,
    );
    for f in ["none.json", "bad.json", "num.json", "good.json"] {
        match pb.load_vendor_configs_from_json_2(&format!("{}/models/{}", d, f)) {
            Ok(m) => {
                let mut l = vec![
                    s("model"),
                    hex(f),
                    hex(&m.id),
                    hex(&m.name),
                    hex(&m.model_id),
                    num(m.technology as i64),
                    hex(&m.family),
                    hex(&m.bed_model),
                    hex(&m.bed_texture),
                    hex(&m.hotend_model),
                    hex(&m.default_bed_type),
                    hex(&m.use_rect_grid),
                    hex(&m.support_side_panel_fan),
                    hex(&m.resource_bind_name),
                    num(m.variants.len() as i64),
                ];
                for v in &m.variants {
                    l.push(hex(&v.name));
                }
                push_list(&mut l, &m.default_materials);
                push_list(&mut l, &m.not_support_bed_types);
                push_list(&mut l, &m.bottom_texture_rect_longer_ignore_list);
                o.emit(l);
            }
            Err(e) => o.err("model", &e),
        }
    }
    write_file(&format!("{}/badv/Bad.json", d), r#"{"name": "Bad", "version": "x"}"#);
    write_file(&format!("{}/badp/Badp.json", d), r#"{"name": "Badp", "version": "1.0.0.0", "process_list": [{"name": "P", "sub_path": "p.json"}]}"#);
    write_file(&format!("{}/badp/Badp/p.json", d), r#"{"name": "P", "inherits": "Nope", "instantiation": "true"}"#);
    for (dir, vendor) in [("none_dir", "Acme"), ("badv", "Bad"), ("badp", "Badp")] {
        let mut other = PresetBundle::new();
        match other.load_vendor_configs_from_json(
            &format!("{}/{}", d, dir),
            vendor,
            LoadConfigBundleAttributes::new(LoadConfigBundleAttribute::LoadSystem),
            ForwardCompatibilitySubstitutionRule::Disable,
        ) {
            Ok(r) => o.emit(vec![s("vendor_load"), hex(vendor), num(r.0.len() as i64), num(r.1 as i64)]),
            Err(e) => o.err("vendor_load", &e),
        }
    }

    // a preset bundle file
    {
        let zp = format!("{}/bundle.bbscfg", d);
        {
            use std::io::Write;
            let f = std::fs::File::create(&zp).unwrap();
            let mut zip = zip::ZipWriter::new(f);
            let opts = zip::write::FileOptions::default();
            let a = format!(
                r#"{{"version": "{}", "name": "Zip PLA", "from": "User", "inherits": "Acme PETG @Acme X1", "filament_settings_id": ["Zip PLA"], "nozzle_temperature": ["250", "251"]}}"#,
                VERSION
            );
            zip.start_file("bundle/Zip PLA.json", opts).unwrap();
            zip.write_all(a.as_bytes()).unwrap();
            zip.start_file("bundle_structure.json", opts).unwrap();
            zip.write_all(b"{}").unwrap();
            zip.add_directory("bundle/", opts).unwrap();
            zip.finish().unwrap();
        }
        let mut files = vec![zp.clone()];
        let subs = pb.import_presets(&mut files, &mut |_| 1, ForwardCompatibilitySubstitutionRule::EnableSilent);
        let mut l = vec![s("import_zip"), num(subs.len() as i64)];
        for f in &files {
            l.push(hex(&o.rel(f)));
        }
        o.emit(l);
        let _ = std::fs::remove_file(&zp);
        dump_files(o, "import_zip");
    }

    // a printer with a nozzle rack in multi-diameter mode
    let r: R = (|| {
        // The vendor section of the AppConfig does not install this variant: make it selectable.
        pb.printers.find_preset_mut("Acme X1 0.4 rack", false, true).unwrap().is_visible = true;
        pb.printers.select_preset_by_name("Acme X1 0.4 rack", true, false);
        pb.update_compatible(PresetSelectCompatibleType::Always)?;
        pb.set_num_filaments(2, "")?;
        pb.filament_presets = strs(&["Acme PLA @Acme X1", "Acme PETG @Acme X1"]);
        pb.project_config.option_typed_mut::<ConfigOptionBool>("multi_nozzle_diameter_mode", false)?.unwrap().value = true;
        let mut file = DynamicPrintConfig::new();
        file.set_deserialize_strict("extruder_nozzle_count", "3", false)?;
        file.set_deserialize_strict("extruder_nozzle_volume_type", "Standard,High Flow,Standard", false)?;
        file.set_deserialize_strict("extruder_nozzle_diameter", "0.4,0.6,0.6", false)?;
        file.set_deserialize_strict("extruder_nozzle_slot", "0,1,2", false)?;
        pb.nozzle_open_project(&file)?;
        dump_cfg(o, "rack:project", &pb.project_config);
        let mut l = vec![s("rack_helpers")];
        push_list(&mut l, &pb.compute_filament_supported_nozzle_diameters(2));
        push_list(&mut l, &pb.get_filament_nozzle_diameter_options(0));
        push_list(&mut l, &pb.get_filament_nozzle_diameter_options(1));
        for v in pb.get_installed_nozzle_variants() {
            l.push(hex(&v.0));
            l.push(num(v.1 as i64));
        }
        push_list(&mut l, &pb.get_extra_compatible_printers_by_full_stat());
        l.push(b(pb.is_filament_nozzle_diameter_valid(1, "0.6")));
        o.emit(l);
        pb.update_compatible(PresetSelectCompatibleType::Always)?;
        dump_bundle(o, "rack", pb);
        let mut dm: BTreeMap<i32, AMSMapInfo> = BTreeMap::new();
        dm.insert(0, map_info("0", "0"));
        dm.get_mut(&0).unwrap().nozzle_diameter = s("0.6");
        dm.insert(1, map_info("0", "3"));
        pb.validate_filament_nozzle_diameter_selections(Some(&dm), true)?;
        dump_bundle(o, "rack_validated", pb);
        pb.seed_filament_nozzles_for_multi_diameter("0.6")?;
        dump_cfg(o, "rack_seeded", &pb.project_config);
        let mut sel = String::new();
        let r1 = pb.set_filament_nozzle_diameter(0, "0.4", Some(&mut sel), false)?;
        let r2 = pb.set_filament_nozzle_diameter(1, "0.8", None, false)?;
        let r3 = pb.set_filament_nozzle_diameter(5, "0.4", None, false)?;
        o.emit(vec![s("rack_set_diameter"), b(r1), hex(&sel), b(r2), b(r3)]);
        pb.incompatible_nozzle_diameter_slots = vec![1];
        pb.validate_filament_nozzle_diameter_selections(None, false)?;
        dump_bundle(o, "rack_revalidated", pb);
        let c = pb.full_config(true, None, None)?;
        dump_cfg(o, "rack_full", &c);
        pb.stamp_printer_nozzle_diameter_mode()?;
        let edited = pb.printers.get_edited_preset().clone();
        dump_preset(o, "rack_stamp", &edited);
        o.emit(vec![s("rack_multi_user"), b(pb.is_multi_nozzle_user_printer_preset(&edited))]);
        Ok(())
    })();
    if let Err(e) = r {
        o.err("rack", &e);
    }

    // wizard, saving, copies, project embedded presets
    let r: R = (|| {
        let cfg = pb.full_config(true, None, None)?;
        pb.load_config_from_wizard("Wizard Set", cfg, Semver::new())?;
        dump_bundle(o, "wizard", pb);
        dump_files(o, "wizard");
        pb.filaments.get_edited_preset_mut().config.option_typed_mut::<ConfigOptionStrings>("filament_vendor", true)?.unwrap().values = strs(&["Saved"]);
        pb.save_changes_for_preset("Saved PLA", Type::TypeFilament, &strs(&["nozzle_temperature"]), false)?;
        dump_bundle(o, "saved", pb);
        dump_files(o, "saved");
        Ok(())
    })();
    if let Err(e) = r {
        o.err("misc", &e);
    }
    write_file(&format!("{}/old/process/old.json", d), "{}");
    write_file(&format!("{}/old/filament/sub/f.json", d), "{\"a\": 1}");
    for _pass in 0..2 {
        match pb.copy_files(&format!("{}/old", d)) {
            Ok(()) => o.emit(vec![s("copy_files"), s("ok")]),
            Err(e) => o.err("copy_files", &e),
        }
        std::fs::create_dir_all(format!("{}/data/old", d)).unwrap();
    }
    dump_files(o, "copied");
    let r: R = (|| {
        let mut e1 = Preset::new(Type::TypeFilament, "Embedded PLA", false);
        e1.is_project_embedded = true;
        e1.config.set_key_value("inherits", Box::new(ConfigOptionString::with_value("Acme PLA @Acme X1")));
        e1.config.set_deserialize_strict("nozzle_temperature", "222,224", false)?;
        let mut e2 = Preset::new(Type::TypePrint, "Embedded Process", false);
        e2.is_project_embedded = true;
        e2.config.set_key_value("inherits", Box::new(ConfigOptionString::with_value("Nope")));
        let mut embedded = vec![e1, e2];
        let subs = pb.load_project_embedded_presets(&mut embedded, ForwardCompatibilitySubstitutionRule::EnableSilent)?;
        let n = pb.get_current_project_embedded_presets()?.len();
        o.emit(vec![s("load_embedded"), num(subs.len() as i64), num(n as i64)]);
        dump_preset(o, "embedded:e1", &embedded[0]);
        pb.filament_presets = strs(&["Embedded PLA", "Acme PETG @Acme X1"]);
        pb.reset_project_embedded_presets()?;
        dump_bundle(o, "embedded_reset", pb);
        Ok(())
    })();
    if let Err(e) = r {
        o.err("misc", &e);
    }
}


fn gen_bundle(o: &mut Out) {
    let d = o.d.clone();
    let data = format!("{}/data", d);
    let res = format!("{}/resources", d);
    slicer::log_sink::set_data_dir(&data);
    slicer::utils::set_resources_dir(&res);
    write_file(
        &format!("{}/profiles/BBL/filament/filament_name_map.json", res),
        r#"{"Acme X1 0.6 nozzle": {"Old PLA": "Acme PLA @Acme X1", "Old PETG": "Acme PETG @Acme X1"}}"#,
    );

    gen_free_functions(o);

    let mut pb = Box::new(PresetBundle::new());
    dump_bundle(o, "new", &pb);
    match pb.setup_directories() {
        Ok(()) => o.emit(vec![s("setup_directories"), s("ok")]),
        Err(e) => o.err("setup_directories", &e),
    }
    write_vendor(&format!("{}/system", data));
    write_user(&format!("{}/user/default", data));
    dump_files(o, "written");

    let mut config = AppConfig::new();
    config.set_app("version", "02.09.01.70");
    config.set("presets", "machine", "Acme X1 0.6 nozzle");
    config.set("presets", "process", "My Process");
    config.set("presets", "filament", "My PLA");
    config.set("presets", "filament_01", "Acme PETG @Acme X1.ini");
    config.set("presets", "filament_colors", "#FF0000,#00FF00");
    config.set("presets", "flush_volumes_vector", "140|150|160|170");
    config.set("presets", "extruder_ams_count", "4#1|1#0,4#2");
    config.set("presets", "filament_is_mixed", "0,0");
    config.set("presets", "filament_mixed_gradient_curve", "0,0.2|1,0.8;0,1");
    config.set_app("flush_multiplier", "1|1.5");
    match pb.load_presets(&mut config, ForwardCompatibilitySubstitutionRule::EnableSilent, &PresetPreferences::default()) {
        Ok(r) => {
            let e = o.rel(&r.1);
            o.emit(vec![s("load_presets"), num(r.0.len() as i64), hex(&e)]);
        }
        Err(e) => o.err("load_presets", &e),
    }
    dump_bundle(o, "loaded", &pb);
    dump_app(o, "loaded", &config);
    dump_files(o, "loaded");

    // full configs
    let r: R = (|| {
        let c = pb.full_config(true, None, None)?;
        dump_cfg(o, "full", &c);
        let c = pb.full_config(false, None, None)?;
        dump_cfg(o, "full_noext", &c);
        let sec = pb.full_config_secure(None)?;
        let h = cfg_hash(o, &sec);
        o.emit(vec![s("full_secure"), num(sec.keys().len() as i64), h]);
        let c = pb.full_config(true, Some(vec![2, 1]), Some(vec![1, 0]))?;
        dump_cfg(o, "full_maps", &c);
        Ok(())
    })();
    if let Err(e) = r {
        o.err("full_config", &e);
    }
    let r: R = (|| {
        let mut printer = pb.printers.get_edited_preset().clone();
        let mut print = pb.prints.get_edited_preset().clone();
        let mut fils: Vec<Preset> = Vec::new();
        for n in &pb.filament_presets {
            fils.push(pb.filaments.find_preset(n, true).unwrap().clone());
        }
        let c = PresetBundle::construct_full_config(&mut printer, &mut print, &pb.project_config, &mut fils, true, None, None)?;
        dump_cfg(o, "construct", &c);
        let mut one = vec![fils[0].clone()];
        let c = PresetBundle::construct_full_config(&mut printer, &mut print, &pb.project_config, &mut one, false, Some(vec![2]), None)?;
        dump_cfg(o, "construct1", &c);
        let c = PresetBundle::construct_full_config(&mut printer, &mut print, &pb.project_config, &mut fils, false, None, None)?;
        dump_cfg(o, "construct_noext", &c);
        dump_preset(o, "construct:print", &print);
        Ok(())
    })();
    if let Err(e) = r {
        o.err("construct_full_config", &e);
    }
    let r: R = (|| {
        for m in [true, false] {
            let mat = pb.get_full_flush_matrix(m)?;
            let mut l = vec![s("flush_matrix"), b(m), num(mat.len() as i64)];
            for e in &mat {
                l.push(num(e.len() as i64));
                for r in e {
                    for v in r {
                        l.push(dbl(*v as f64));
                    }
                }
            }
            o.emit(l);
        }
        Ok(())
    })();
    if let Err(e) = r {
        o.err("flush_matrix", &e);
    }

    // helpers
    o.emit(vec![s("extruder_count"), num(pb.get_printer_extruder_count() as i64), b(pb.support_different_extruders())]);
    {
        let mut l = vec![s("nozzle_volume_list")];
        for t in pb.get_printer_nozzle_volume_list() {
            l.push(num(t as i64));
        }
        o.emit(l);
        let fm = vec![1, 2, 3, 0];
        let mut l2 = vec![s("default_nvt_for_filaments")];
        for v in pb.get_default_nozzle_volume_types_for_filaments(&fm) {
            l2.push(num(v as i64));
        }
        o.emit(l2);
    }
    for (id, printer, sys) in [
        ("AP1", "", false),
        ("AP1", "Acme X1 0.6 nozzle", false),
        ("AT1", "Acme X1 0.6 nozzle", false),
        ("AP1", "", true),
        ("ZZ", "", false),
        ("", "", false),
    ] {
        match pb.get_filament_by_filament_id(id, printer, sys) {
            None => o.emit(vec![s("filament_by_id"), hex(id), hex(printer), b(sys), s("none")]),
            Some(r) => o.emit(vec![
                s("filament_by_id"),
                hex(id),
                hex(printer),
                b(sys),
                hex(&r.filament_name),
                hex(&r.filament_id),
                hex(&r.filament_type),
                hex(&r.vendor),
                num(r.nozzle_temp_range_low as i64),
                num(r.nozzle_temp_range_high as i64),
                num(r.temperature_vitrification as i64),
                b(r.is_support),
                b(r.is_system),
                num(r.filament_printable as i64),
                hex(&r.setting_id),
                num(r.get_filament_extruder_compatibility() as i64),
            ]),
        }
    }
    for (t, n) in [("Acme X1", "0.6"), ("Acme X1", ""), ("Acme X1", "0.0"), ("", "0.4"), ("Nope", "0.4")] {
        let mut l = vec![s("printer_names"), hex(t), hex(n)];
        for x in pb.get_printer_names_by_printer_type_and_nozzle(t, n, true) {
            l.push(hex(&x));
        }
        o.emit(l);
    }
    for (m, v) in [("", ""), ("Acme X1", "0.4"), ("", "0.4"), ("Nope", "")] {
        let p = pb.get_similar_printer_preset(m, v);
        let name = match p {
            Some(i) => pb.printers.m_presets[i].name.clone(),
            None => s("<null>"),
        };
        o.emit(vec![s("similar_printer"), hex(m), hex(v), hex(&name)]);
    }
    {
        let a = o.rel(&pb.get_texture_for_printer_model("Acme X1"));
        let bb = o.rel(&pb.get_stl_model_for_printer_model("Acme X1"));
        let c = o.rel(&pb.get_hotend_model_for_printer_model("Acme X1"));
        let dd = o.rel(&pb.get_texture_for_printer_model("Nope"));
        o.emit(vec![s("model_files"), hex(&a), hex(&bb), hex(&c), hex(&dd)]);
    }
    {
        let custom = pb.get_custom_vendor_models();
        let key = s("Custom");
        dump_vendors(o, "custom", std::iter::once((&key, &custom)));
    }
    o.emit(vec![
        s("vendor_version"),
        hex(&pb.get_vendor_profile_version("Acme").to_string()),
        hex(&pb.get_vendor_profile_version("Nope").to_string()),
    ]);
    o.emit(vec![
        s("hrc"),
        num(pb.get_required_hrc_by_filament_id("AP1") as i64),
        num(pb.get_required_hrc_by_filament_id("AG1") as i64),
        num(pb.get_required_hrc_by_filament_id("ZZ") as i64),
    ]);
    o.emit(vec![
        s("by_alias"),
        hex(&pb.get_preset_name_by_alias(Type::TypeFilament, "Acme PLA")),
        hex(&pb.get_preset_name_by_alias(Type::TypePrinter, "Acme PLA")),
        hex(&pb.get_preset_name_by_alias(Type::TypePrint, "0.20mm Standard")),
    ]);
    o.emit(vec![
        s("only_edited"),
        b(pb.is_the_only_edited_filament(0)),
        b(pb.is_the_only_edited_filament(1)),
        b(pb.is_the_only_edited_filament(5)),
    ]);
    {
        let mut l = vec![s("tpu")];
        for v in pb.get_used_tpu_filaments(&[1, 2]) {
            l.push(num(v as i64));
        }
        o.emit(l);
    }
    {
        let mut l = vec![s("mixed"), num(pb.physical_filament_count() as i64), num(pb.mixed_filament_count() as i64), b(pb.is_mixed_filament(0))];
        for i in pb.physical_filament_config_indices() {
            l.push(num(i as i64));
        }
        o.emit(l);
    }
    {
        let mut l = vec![s("supported_diameters")];
        push_list(&mut l, &pb.compute_filament_supported_nozzle_diameters(3));
        push_list(&mut l, &pb.get_filament_nozzle_diameter_options(0));
        l.push(b(pb.is_filament_nozzle_diameter_valid(0, "0.6")));
        l.push(b(pb.is_filament_nozzle_diameter_valid(0, "0.4")));
        l.push(b(pb.is_filament_nozzle_diameter_valid(0, "")));
        for v in pb.get_installed_nozzle_variants() {
            l.push(hex(&v.0));
            l.push(num(v.1 as i64));
        }
        l.push(hex(&pb.tray_diameter_for_filament_slot(0)));
        push_list(&mut l, &pb.get_extra_compatible_printers_by_full_stat());
        l.push(b(pb.is_multi_nozzle_user_printer_preset(pb.printers.get_edited_preset())));
        o.emit(l);
    }
    let r: R = (|| {
        let mut full = pb.full_config(true, None, None)?;
        let mut diffs: BTreeSet<String> = BTreeSet::new();
        let r = pb.validate_presets("proj.3mf", &mut full, &mut diffs)?;
        let mut l = vec![s("validate"), num(r as i64)];
        for x in &diffs {
            l.push(hex(x));
        }
        o.emit(l);
        let mut full2 = pb.full_config(true, None, None)?;
        full2.option_typed_mut::<ConfigOptionStrings>("inherits_group", true)?.unwrap().values =
            strs(&["0.20mm Standard @Acme X1", "Acme PLA @Acme X1", "Acme PETG @Acme X1", "Acme X1 0.6 nozzle"]);
        full2.option_typed_mut::<ConfigOptionStrings>("filament_settings_id", true)?.unwrap().values = strs(&["Acme PLA @Acme X1", "Acme PETG @Acme X1"]);
        full2.option_typed_mut::<ConfigOptionString>("printer_settings_id", true)?.unwrap().value = s("Acme X1 0.6 nozzle");
        full2.option_typed_mut::<ConfigOptionStrings>("different_settings_to_system", true)?.unwrap().values =
            strs(&["", "machine_start_gcode;layer_height", "", "filament_start_gcode"]);
        diffs.clear();
        let r = pb.validate_presets("proj2.3mf", &mut full2, &mut diffs)?;
        let mut l2 = vec![s("validate"), num(r as i64)];
        for x in &diffs {
            l2.push(hex(x));
        }
        o.emit(l2);
        Ok(())
    })();
    if let Err(e) = r {
        o.err("validate", &e);
    }
    {
        let edited = pb.filaments.get_edited_preset().clone();
        let p = pb.get_preset_differed_for_save(&edited).unwrap();
        let (name, h) = match &p {
            Some(p) => (p.name.clone(), cfg_hash(o, &p.config)),
            None => (s("<null>"), s("-")),
        };
        o.emit(vec![s("differed_for_save"), hex(&name), h]);
        let mut kv: BTreeMap<String, String> = BTreeMap::new();
        let r = pb.get_differed_values_to_update(&edited, &mut kv).unwrap();
        let mut l = vec![s("differed_values"), num(r as i64), num(kv.len() as i64)];
        for (k, v) in &kv {
            l.push(hex(k));
            l.push(hex(v));
        }
        o.emit(l);
    }

    pb.export_selections(&mut config);
    dump_app(o, "export", &config);

    // AMS counts and selections
    pb.set_ams_count(0, 2, 1);
    pb.set_ams_count(1, 1, 3);
    pb.set_ams_count(5, 1, 1);
    {
        let c0 = pb.get_ams_count(0);
        let c1 = pb.get_ams_count(1);
        let c5 = pb.get_ams_count(5);
        o.emit(vec![
            s("ams_count"),
            num(c0.0 as i64),
            num(c0.1 as i64),
            num(c1.0 as i64),
            num(c1.1 as i64),
            num(c5.0 as i64),
            num(c5.1 as i64),
        ]);
    }
    pb.initialize_ams_selection(0, "dev1", &strs(&["A", "B", "C"]), &strs(&["X"]));
    pb.initialize_ams_selection(0, "dev1", &strs(&["Q"]), &[]);
    pb.resize_ams_selection(1, "dev1", &strs(&["A", "B", "C"]), &strs(&["X", "Y"]), 3, 1);
    pb.resize_ams_selection(0, "dev2", &strs(&["C", "A", "D"]), &strs(&["X", "Z"]), 4, 2);
    pb.replace_ams_selection(1, "dev3", &strs(&["R"]), &strs(&["S", "T"]));
    let none = pb.get_ams_selection(7).is_none();
    o.emit(vec![s("ams_selection"), b(none)]);
    dump_bundle(o, "ams", &pb);

    // AMS sync
    pb.filament_ams_list.clear();
    pb.filament_ams_list.insert(0, tray("AG1", "#00FF00FF", "A1", "PETG", "0", "0", true));
    pb.filament_ams_list.insert(1, tray("ZZ9", "#0000FFFF", "A2", "PLA", "0", "1", true));
    pb.filament_ams_list.insert(2, tray("", "#FFFFFFFF", "A3", "", "0", "2", true));
    pb.filament_ams_list.insert(3, tray("AT1", "#111111FF", "A4", "TPU", "0", "3", true));
    {
        let mut t = tray("AP1", "#FF0000FF", "B1", "PLA", "1", "0", false);
        t.set_key_value("tray_nozzle_diameter", Box::new(ConfigOptionFloats::with_size(1, 0.6)));
        t.set_key_value("tray_nozzle_volume_type", Box::new(ConfigOptionInts::from_values(vec![1])));
        pb.filament_ams_list.insert(0x10000, t);
    }
    {
        let mut info = AMSComboInfo::default();
        pb.get_ams_cobox_infos(&mut info, false);
        let mut l = vec![s("cobox")];
        push_list(&mut l, &info.ams_filament_presets);
        push_list(&mut l, &info.ams_filament_colors);
        push_list(&mut l, &info.ams_names);
        l.push(b(info.empty()));
        let mut info2 = AMSComboInfo::default();
        pb.get_ams_cobox_infos(&mut info2, true);
        push_list(&mut l, &info2.ams_filament_presets);
        o.emit(l);
        let fi = pb.get_extruder_filament_info();
        let mut l2 = vec![s("extruder_filament_info"), num(fi.len() as i64)];
        for v in &fi {
            l2.push(num(v.len() as i64));
        }
        o.emit(l2);
    }
    {
        let mut unknowns: Vec<(i32, String)> = Vec::new();
        let mut maps: BTreeMap<i32, AMSMapInfo> = BTreeMap::new();
        let mut merge = MergeFilamentInfo::default();
        let n = pb.sync_ams_list(&mut unknowns, false, &mut maps, false, &mut merge, false).unwrap();
        let mut l = vec![s("sync_ams"), num(n as i64), num(unknowns.len() as i64)];
        for u in &unknowns {
            l.push(num(u.0 as i64));
            l.push(hex(&u.1));
        }
        l.push(num(merge.merges.len() as i64));
        o.emit(l);
        dump_maps(o, "sync1", &maps);
        dump_bundle(o, "sync1", &pb);
    }
    {
        let mut unknowns: Vec<(i32, String)> = Vec::new();
        let mut maps: BTreeMap<i32, AMSMapInfo> = BTreeMap::new();
        maps.insert(0, map_info("0", "1"));
        maps.insert(1, map_info("1", "0"));
        maps.insert(4, map_info("1", "0"));
        let mut merge = MergeFilamentInfo::default();
        let n = pb.sync_ams_list(&mut unknowns, true, &mut maps, true, &mut merge, true).unwrap();
        let mut l = vec![s("sync_ams"), num(n as i64), num(unknowns.len() as i64)];
        for u in &unknowns {
            l.push(num(u.0 as i64));
            l.push(hex(&u.1));
        }
        l.push(num(merge.merges.len() as i64));
        for m in &merge.merges {
            l.push(num(m.len() as i64));
            for i in m {
                l.push(num(*i as i64));
            }
        }
        o.emit(l);
        dump_maps(o, "sync2", &maps);
        dump_bundle(o, "sync2", &pb);
    }

    // filament count changes
    let r: R = (|| {
        pb.set_num_filaments(4, "#123456")?;
        dump_bundle(o, "num4", &pb);
        pb.update_num_filaments(1)?;
        dump_bundle(o, "del1", &pb);
        pb.set_filament_preset(1, "Acme PETG @Acme X1 (modified)");
        pb.set_filament_preset(9, "x");
        pb.set_filament_preset(2, "Nope");
        dump_bundle(o, "set_filament", &pb);
        Ok(())
    })();
    if let Err(e) = r {
        o.err("num_filaments", &e);
    }

    // printer switch
    let r: R = (|| {
        pb.printers.select_preset_by_name("Acme X1 0.4 nozzle", true, false);
        pb.update_compatible(PresetSelectCompatibleType::Always)?;
        dump_bundle(o, "switch04", &pb);
        pb.on_extruders_count_changed(1, true)?;
        dump_bundle(o, "extruders1", &pb);
        let c = pb.full_config(true, None, None)?;
        dump_cfg(o, "full04", &c);
        pb.set_calibrate_printer("Acme X1 0.6 nozzle");
        let name = match pb.calibrate_printer {
            Some(i) => preset_at(&pb.printers, i).name.clone(),
            None => s("<null>"),
        };
        o.emit(vec![s("calibrate"), num(pb.calibrate_filaments.len() as i64), hex(&name)]);
        pb.set_calibrate_printer("");
        o.emit(vec![s("calibrate"), num(pb.calibrate_filaments.len() as i64)]);
        pb.stamp_printer_nozzle_diameter_mode()?;
        let edited = pb.printers.get_edited_preset().clone();
        dump_preset(o, "stamp", &edited);
        pb.validate_filament_nozzle_diameter_selections(None, false)?;
        pb.seed_filament_nozzles_for_multi_diameter("0.4")?;
        let mut sel = String::new();
        let r1 = pb.set_filament_nozzle_diameter(0, "0.4", Some(&mut sel), false)?;
        let r2 = pb.set_filament_nozzle_diameter(0, "0.6", None, true)?;
        o.emit(vec![s("set_nozzle_diameter"), b(r1), hex(&sel), b(r2)]);
        dump_bundle(o, "nozzle", &pb);
        Ok(())
    })();
    if let Err(e) = r {
        o.err("switch", &e);
    }

    // defaults replaced
    let r: R = (|| {
        pb.prints.select_preset(0);
        pb.filament_presets = strs(&["Default Filament", "Acme PETG @Acme X1", "Default Filament"]);
        let r = pb.replace_default_presets()?;
        o.emit(vec![s("replace_default_presets"), b(r)]);
        dump_bundle(o, "replaced", &pb);
        Ok(())
    })();
    if let Err(e) = r {
        o.err("replace_default_presets", &e);
    }

    // external config (a project)
    let r: R = (|| {
        pb.printers.select_preset_by_name("Acme X1 0.6 nozzle", true, false);
        pb.update_compatible(PresetSelectCompatibleType::Always)?;
        pb.set_num_filaments(2, "")?;
        pb.filament_presets = strs(&["My PLA", "Acme PETG @Acme X1"]);
        let mut cfg = pb.full_config(true, None, None)?;
        cfg.option_typed_mut::<ConfigOptionFloat>("layer_height", false)?.unwrap().value = 0.24;
        pb.load_config_model("project.3mf", cfg, Semver::new())?;
        dump_bundle(o, "model", &pb);
        let mut cfg1 = pb.full_config(false, None, None)?;
        cfg1.option_typed_mut::<ConfigOptionStrings>("filament_colour", false)?.unwrap().values = strs(&["#AABBCC"]);
        pb.load_config_model("one.3mf", cfg1, Semver::new())?;
        dump_bundle(o, "model1", &pb);
        if let Err(e) = pb.load_config_file(&format!("{}/none.ini", d), ForwardCompatibilitySubstitutionRule::Disable) {
            o.err("load_config_file", &e);
        }
        if let Err(e) = pb.load_config_model("empty.3mf", DynamicPrintConfig::new(), Semver::new()) {
            o.err("load_config_model", &e);
        }
        let embedded = pb.get_current_project_embedded_presets()?;
        o.emit(vec![s("embedded"), num(embedded.len() as i64)]);
        pb.reset_project_embedded_presets()?;
        dump_bundle(o, "reset_embedded", &pb);
        Ok(())
    })();
    if let Err(e) = r {
        o.err("model", &e);
    }

    // import / export
    let r: R = (|| {
        let exp = format!("{}/export", d);
        std::fs::create_dir_all(&exp).unwrap();
        let mut asked = 0;
        let r = pb.export_current_configs(
            &exp,
            &mut |_| {
                asked += 1;
                3
            },
            true,
            false,
        )?;
        let mut l = vec![s("export_current_configs"), num(asked as i64)];
        for f in &r {
            l.push(hex(&o.rel(f)));
        }
        o.emit(l);
        dump_files(o, "exported");
        write_file(
            &format!("{}/import/Imported PLA.json", d),
            &format!(
                r#"{{"version": "{}", "name": "Imported PLA", "from": "User", "inherits": "Acme PLA @Acme X1", "filament_id": "IP1", "filament_settings_id": ["Imported PLA"], "nozzle_temperature": ["230", "232"]}}"#,
                VERSION
            ),
        );
        write_file(
            &format!("{}/import/Imported Root.json", d),
            &format!(
                r#"{{"version": "{}", "name": "Imported Root", "from": "User", "inherits": "", "filament_settings_id": ["Imported Root"], "filament_type": ["ABS"]}}"#,
                VERSION
            ),
        );
        write_file(&format!("{}/import/Bad.json", d), r#"{"version": "x", "name": "Bad", "filament_settings_id": ["Bad"]}"#);
        write_file(&format!("{}/import/Unknown.json", d), &format!(r#"{{"version": "{}", "name": "Unknown", "layer_height": "0.1"}}"#, VERSION));
        let mut files = vec![
            format!("{}/import/Imported PLA.json", d),
            format!("{}/import/Imported Root.json", d),
            format!("{}/import/Bad.json", d),
            format!("{}/import/Unknown.json", d),
            format!("{}/import/missing.zip", d),
        ];
        let mut confirms = 0;
        let subs = pb.import_presets(
            &mut files,
            &mut |_| {
                confirms += 1;
                1
            },
            ForwardCompatibilitySubstitutionRule::EnableSilent,
        );
        let mut l2 = vec![s("import_presets"), num(subs.len() as i64), num(confirms as i64)];
        for f in &files {
            l2.push(hex(&o.rel(f)));
        }
        o.emit(l2);
        dump_bundle(o, "imported", &pb);
        dump_files(o, "imported");
        Ok(())
    })();
    if let Err(e) = r {
        o.err("import_export", &e);
    }

    // user presets from the cloud, removal, directories
    let r: R = (|| {
        let kv = |v: &[(&str, &str)]| -> BTreeMap<String, String> { v.iter().map(|(a, bb)| (a.to_string(), bb.to_string())).collect() };
        let mut my_presets: BTreeMap<String, BTreeMap<String, String>> = BTreeMap::new();
        my_presets.insert(
            s("Cloud PLA"),
            kv(&[
                ("type", "filament"),
                ("name", "Cloud PLA"),
                ("inherits", "Acme PLA @Acme X1"),
                ("version", VERSION),
                ("setting_id", "CS1"),
                ("updated_time", "1200"),
                ("nozzle_temperature", "218,219"),
                ("filament_settings_id", "Cloud PLA"),
            ]),
        );
        my_presets.insert(
            s("Cloud Process"),
            kv(&[
                ("type", "print"),
                ("name", "Cloud Process"),
                ("inherits", "0.20mm Standard @Acme X1"),
                ("version", VERSION),
                ("setting_id", "CS2"),
                ("updated_time", "1300"),
                ("layer_height", "0.28"),
                ("print_settings_id", "Cloud Process"),
            ]),
        );
        my_presets.insert(s("No Type"), kv(&[("name", "No Type")]));
        my_presets.insert(s("Odd Type"), kv(&[("type", "odd")]));
        let subs = pb.load_user_presets_2(&mut config, &mut my_presets, ForwardCompatibilitySubstitutionRule::EnableSilent)?;
        o.emit(vec![s("load_user_presets_cloud"), num(subs.len() as i64)]);
        dump_bundle(o, "cloud", &pb);
        let mut sys: BTreeMap<String, BTreeMap<String, String>> = BTreeMap::new();
        sys.insert(s("Acme PETG @Acme X1"), kv(&[("type", "filament"), ("setting_id", "NEW")]));
        sys.insert(s("Acme PLA @Acme X1"), kv(&[("type", "filament")]));
        sys.insert(s("Nope"), kv(&[("type", "print"), ("setting_id", "X")]));
        pb.update_system_preset_setting_ids(&sys);
        let mut del: Vec<String> = Vec::new();
        pb.save_user_presets(&config, &mut del)?;
        pb.update_user_presets_directory("u2")?;
        pb.remove_user_presets_directory("")?;
        pb.remove_user_presets_directory("u2")?;
        let keep: BTreeMap<String, BTreeMap<String, String>> = BTreeMap::new();
        pb.remove_users_preset(&config, Some(&keep))?;
        dump_bundle(o, "users", &pb);
        dump_files(o, "users");
        Ok(())
    })();
    if let Err(e) = r {
        o.err("users", &e);
    }

    gen_more(o, &mut pb);

    // copy, reset
    {
        let copy = pb.clone();
        dump_bundle(o, "copy", &copy);
        pb.set_default_suppressed(false);
        pb.reset(false);
        dump_bundle(o, "reset", &pb);
    }
    drop(pb);
}

#[test]
fn preset_bundle_matches_cpp() {
    let expected: Vec<String> = read_vectors().lines().map(|l| l.to_string()).collect();

    let d = std::env::temp_dir().join(format!("preset_bundle_native_parity_{}", std::process::id()));
    let d = d.to_string_lossy().into_owned();
    let _ = std::fs::remove_dir_all(&d);
    std::fs::create_dir_all(&d).unwrap();
    let mut o = Out { d: d.clone(), lines: Vec::new(), files: BTreeMap::new() };
    // As gen.cpp: the scenario runs in the scratch directory. This test binary has one test, so
    // changing the directory of the process is safe.
    let cwd = std::env::current_dir().unwrap();
    std::env::set_current_dir(&d).unwrap();
    gen_bundle(&mut o);
    std::env::set_current_dir(&cwd).unwrap();
    let _ = std::fs::remove_dir_all(&d);

    let mut bad = 0usize;
    for (i, (e, g)) in expected.iter().zip(o.lines.iter()).enumerate() {
        if e != g {
            bad += 1;
            if bad <= 30 {
                let ee: String = e.chars().take(600).collect();
                let gg: String = g.chars().take(600).collect();
                eprintln!("line {i}:\n  C++:  {ee}\n  Rust: {gg}");
            }
        }
    }
    if expected.len() != o.lines.len() {
        eprintln!("C++ has {} lines, Rust {}", expected.len(), o.lines.len());
    }
    assert_eq!(bad, 0, "{bad} of {} lines differ", expected.len());
    assert_eq!(expected.len(), o.lines.len(), "number of lines");
}
