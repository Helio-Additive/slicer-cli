//! `slicer::preset` (the port of libslic3r/Preset.hpp and Preset.cpp) against the C++ at the pin.
//!
//! `tests/data/preset/preset.txt.gz` is the output of `gen.cpp` beside it (gzip -9n), which links
//! the pin's Preset.cpp and runs one scenario: the option-key lists, the free functions, the static
//! helpers of Preset, then PresetCollection / PrinterPresetCollection operations on presets written
//! to a scratch directory (loading user presets with inherits chains, system and default presets,
//! compatibility, dirty tracking, saving, external, cloud and project presets, clones, deletion,
//! merging) and PhysicalPrinterCollection operations. This test runs the same scenario on the port;
//! every line has to be the same, byte for byte. The header of gen.cpp describes the lines.

use slicer::app_config::AppConfig;
use slicer::config::*;
use slicer::preset::*;
use slicer::print_config::config_def::{is_filament_extruder_override_key, static_print_config_defaults, DynamicPrintConfig};
use slicer::semver::Semver;
use std::collections::{BTreeMap, BTreeSet};
use std::io::Read;
use std::sync::Arc;

fn read_vectors() -> String {
    let path = format!("{}/tests/data/preset/preset.txt.gz", env!("CARGO_MANIFEST_DIR"));
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
fn bang(e: &ConfigurationError) -> String {
    hex(&format!("!{}", e.what()))
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
fn push_cfg(l: &mut Vec<String>, c: &dyn ConfigBase) {
    let keys = c.keys();
    l.push(keys.len().to_string());
    for k in &keys {
        l.push(hex(k));
        l.push(hex(&ser(c.option(k).unwrap())));
    }
}
/// gen.cpp cfg_hash(): FNV-1a 64 of the keys and serialized values.
fn cfg_hash(c: &dyn ConfigBase) -> String {
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
        add(ser(c.option(&k).unwrap()).as_bytes());
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
    l.push(cfg_hash(&p.config));
}
fn dump_preset(o: &mut Out, tag: &str, p: &Preset) {
    let mut l = vec![s("p"), hex(tag)];
    push_preset(o, &mut l, p);
    o.emit(l);
}
fn dump_cfg(o: &mut Out, tag: &str, c: &dyn ConfigBase) {
    let mut l = vec![s("cfg"), hex(tag)];
    push_cfg(&mut l, c);
    o.emit(l);
}
fn dump_coll(o: &mut Out, tag: &str, c: &PresetCollection) {
    let path = o.rel(&c.path_from_name("x", false));
    o.emit(vec![
        s("coll"),
        s(tag),
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
    o.emit(vec![s("files"), s(tag), num(files.len() as i64)]);
    let mut now: BTreeMap<String, String> = BTreeMap::new();
    for f in &files {
        let r = o.rel(f);
        let c = norm_content(&String::from_utf8_lossy(&std::fs::read(f).unwrap()));
        now.insert(r.clone(), c.clone());
        if o.files.get(&r) != Some(&c) {
            o.emit(vec![s("file"), s(tag), hex(&r), hex(&c)]);
        }
    }
    let gone: Vec<String> = o.files.keys().filter(|k| !now.contains_key(*k)).cloned().collect();
    for g in gone {
        o.emit(vec![s("gone"), s(tag), hex(&g)]);
    }
    o.files = now;
}
fn make_cfg(kvs: &[(&str, &str)]) -> DynamicPrintConfig {
    let mut c = DynamicPrintConfig::new();
    for (k, v) in kvs {
        c.set_deserialize_strict(k, v, false).unwrap_or_else(|e| panic!("{k}={v}: {}", e.what()));
    }
    c
}
fn ty(t: i32) -> Type {
    [
        Type::TypeInvalid,
        Type::TypePrint,
        Type::TypeSlaPrint,
        Type::TypeFilament,
        Type::TypeSlaMaterial,
        Type::TypePrinter,
        Type::TypeCount,
        Type::TypePhysicalPrinter,
        Type::TypePlate,
        Type::TypeModel,
    ][t as usize]
}
fn full_defaults() -> &'static DynamicPrintConfig {
    static_print_config_defaults("FullPrintConfig")
}
fn strs(v: &[&str]) -> Vec<String> {
    v.iter().map(|x| x.to_string()).collect()
}

// ---------------------------------------------------------------------------------------------

fn gen_keys(o: &mut Out) {
    let mut k = |name: &str, v: &[String]| {
        let mut l = vec![s("keys"), s(name)];
        push_list(&mut l, v);
        o.emit(l);
    };
    k("print", Preset::print_options());
    k("filament", Preset::filament_options());
    k("machine_limits", Preset::machine_limits_options());
    k("nozzle", Preset::nozzle_options());
    k("printer", Preset::printer_options());
    k("sla_print", Preset::sla_print_options());
    k("sla_material", Preset::sla_material_options());
    k("sla_printer", Preset::sla_printer_options());
    k("physical_printer", PhysicalPrinter::printer_options());
}

fn gen_filament_list_less(o: &mut Out) {
    let raw: &[(&str, &str, &str)] = &[
        ("Bambu PLA Basic", "Bambu Lab", "PLA"),
        ("Bambu PLA Matte", "Bambu", "PLA"),
        ("Bambu PETG HF", "Bambu Lab", "PETG"),
        ("Bambu Zeta", "Bambu Lab", "PLA"),
        ("Bambu Alpha", "Bambu Lab", "ABS"),
        ("Generic PLA", "Generic", "PLA"),
        ("Generic TPU", "Generic", "TPU"),
        ("Generic PC", "Generic", "PC"),
        ("PolyLite PLA", "Polymaker", "PLA"),
        ("Fiberon PA6-CF", "Polymaker", "PA"),
        ("Fiberon PET-CF", "Polymaker", "PET"),
        ("PolyMax PLA", "Polymaker", "PLA"),
        ("Overture PLA", "Overture", "PLA"),
        ("eSUN PLA+", "eSUN", "PLA"),
        ("My Custom", "", "PLA"),
        ("Another", "", "PETG"),
        ("Acme PLA", "Acme", "PLA"),
        ("Acme PETG", "Acme", "PETG"),
        ("Acme ASA", "Acme", "ASA"),
        ("Zed PLA", "Zed", "PLA"),
        ("Acme ABS", "Acme", "ABS"),
        ("Bambu TPU 95A", "Bambu Lab", "TPU"),
        ("Bambu Support G", "Bambu Lab", "PA"),
        ("Fiberon", "Polymaker", "PA"),
    ];
    let keys: Vec<FilamentListKey> =
        raw.iter().map(|(a, v, t)| FilamentListKey { alias: a.to_string(), vendor: v.to_string(), r#type: t.to_string() }).collect();
    for i in 0..keys.len() {
        let mut bits = String::new();
        for j in 0..keys.len() {
            bits.push(if filament_list_less(&keys[i], &keys[j]) { '1' } else { '0' });
        }
        o.emit(vec![s("fll"), num(i as i64), bits]);
    }
}

fn ini_texts() -> Vec<&'static str> {
    vec![
        "nozzle_diameter = 0.4\nfilament_diameter = 1.75\n",
        "[print:abc]\nlayer_height = 0.2\n[filament:x]\na = 1\n",
        "last_export_path = /tmp\n[presets]\nprint = x\n[recent]\nsettings_folder = a\nlast_opened_folder = b\n",
        "[settings]\nx=1\nnozzle_diameter=0.4\n",
        "; comment\n# c2\n\n  key  =  value with spaces  \n[empty]\n[s]\nk=v\n",
        "[a]\nx=1\n[a]\ny=2\n",
        "k=1\nk=2\n",
        "novalue\n",
        "=x\n",
        "[unterminated\n",
        "a=1\r\nb = 2\r\n\r\n[sec]\r\nc = 3",
        "[vendor]\nname = Acme\nconfig_version = 1.2.3\n[printer_model:MK3]\nname = Original\nvariants = 0.4; 0.6\n",
        "",
    ]
}

fn gen_ini(o: &mut Out) {
    for (i, t) in ini_texts().iter().enumerate() {
        let mut l = vec![s("ini"), num(i as i64)];
        let tree = match read_ini(t.as_bytes()) {
            Ok(tree) => tree,
            Err(e) => {
                l.push(bang(&e));
                o.emit(l);
                continue;
            }
        };
        l.push(num(guess_config_file_type(&tree) as i64));
        l.push(num(tree.children.len() as i64));
        for (k, v) in &tree.children {
            l.push(hex(k));
            l.push(hex(&v.data));
            l.push(num(v.children.len() as i64));
            for (k2, v2) in &v.children {
                l.push(hex(k2));
                l.push(hex(&v2.data));
            }
        }
        o.emit(l);
    }
}

fn gen_json(o: &mut Out) {
    let dir = format!("{}/json", o.d);
    write_file(&format!("{dir}/v_ok.json"), r#"{"version": "02.01.00.05", "name": "x", "a": "1", "b": "two"}"#);
    write_file(&format!("{dir}/v_bad.json"), r#"{"version": "abc", "a": "x"}"#);
    write_file(&format!("{dir}/v_missing.json"), r#"{"name": "x", "c": "3"}"#);
    write_file(&format!("{dir}/v_num.json"), r#"{"version": 3, "a": "1"}"#);
    write_file(&format!("{dir}/v_parse.json"), r#"{ nope"#);
    write_file(&format!("{dir}/v_array.json"), r#"[1, 2]"#);
    write_file(&format!("{dir}/v_trailing.json"), r#"{"version": "1.2.3.4"} trailing"#);
    for f in ["v_ok.json", "v_bad.json", "v_missing.json", "v_num.json", "v_parse.json", "v_array.json", "v_trailing.json", "v_none.json"] {
        let path = format!("{dir}/{f}");
        match get_version_from_json(&path) {
            Ok(v) => o.emit(vec![s("ver"), s(f), hex(&v.to_string())]),
            Err(e) => o.emit(vec![s("ver"), s(f), bang(&e)]),
        }
        let keys = strs(&["a", "b", "c", "version"]);
        let mut kv: BTreeMap<String, String> = BTreeMap::new();
        kv.insert(s("c"), s("pre"));
        match get_values_from_json(&path, &keys, &mut kv) {
            Ok(n) => {
                let mut l = vec![s("vals"), s(f), num(n as i64), num(kv.len() as i64)];
                for (k, v) in &kv {
                    l.push(hex(k));
                    l.push(hex(v));
                }
                o.emit(l);
            }
            Err(e) => o.emit(vec![s("vals"), s(f), bang(&e)]),
        }
    }
    std::fs::remove_dir_all(&dir).unwrap();
}

fn gen_extend(o: &mut Out) {
    let inherit = make_cfg(&[
        ("filament_extruder_variant", "\"Direct Drive Standard\";\"Direct Drive High Flow\""),
        ("print_extruder_variant", "\"Direct Drive Standard\";\"Direct Drive High Flow\""),
        ("print_extruder_id", "1,2"),
        ("printer_extruder_variant", "\"Direct Drive Standard\";\"Direct Drive High Flow\""),
        ("printer_extruder_id", "1,2"),
    ]);
    let defaults = make_cfg(&[("nozzle_temperature", "200"), ("filament_retraction_length", "0.8"), ("filament_flow_ratio", "0.95")]);
    let cases: Vec<Vec<(&str, &str)>> = vec![
        vec![("nozzle_temperature", "220"), ("filament_flow_ratio", "0.9"), ("filament_type", "PLA")],
        vec![("print_extruder_variant", "\"Direct Drive Standard\""), ("wall_loops", "3"), ("inner_wall_speed", "100")],
        vec![("nozzle_temperature", "nil"), ("filament_retraction_length", "nil"), ("filament_flow_ratio", "0.9,0.8,0.7")],
        vec![
            ("printer_extruder_variant", "\"Direct Drive Standard\";\"Direct Drive High Flow\";\"X\""),
            ("printer_extruder_id", "1,2,3"),
            ("nozzle_diameter", "0.4"),
            ("max_layer_height", "0.28"),
        ],
    ];
    let empty = DynamicPrintConfig::new();
    for (i, case) in cases.iter().enumerate() {
        for mode in 0..3 {
            let mut c = make_cfg(case);
            let mut l = vec![s("ext"), num(i as i64), num(mode)];
            let r = match mode {
                0 => extend_default_config_length(&mut c, &inherit, false, &empty),
                1 => extend_default_config_length(&mut c, &empty, true, &defaults),
                _ => extend_default_config_length(&mut c, &inherit, true, &defaults),
            };
            match r {
                Ok(()) => push_cfg(&mut l, &c),
                Err(e) => l.push(bang(&e)),
            }
            o.emit(l);
        }
    }
}

fn gen_preset_statics(o: &mut Out) {
    for t in 0..=9 {
        o.emit(vec![s("type"), num(t as i64), hex(&Preset::get_type_string(ty(t))), hex(&Preset::get_iot_type_string(ty(t)))]);
    }
    for x in ["process", "print", "filament", "machine", "printer", "bogus", ""] {
        o.emit(vec![s("type_from"), hex(x), num(Preset::get_type_from_string(x) as i64)]);
    }
    for x in [" (modified)foo", "foo (modified)", "foo", " (modified)", ""] {
        o.emit(vec![s("remove_suffix"), hex(x), hex(&Preset::remove_suffix_modified(x))]);
    }
    Preset::update_suffix_modified("* ");
    o.emit(vec![s("remove_suffix2"), hex(&Preset::remove_suffix_modified("* bar")), hex(&PresetCollection::get_suffix_modified())]);
    {
        let mut p = Preset::new(Preset::TYPE_FILAMENT, "Name A", false);
        p.is_dirty = true;
        o.emit(vec![s("label_star"), hex(&p.label(true))]);
    }
    Preset::update_suffix_modified(" (modified)");
    {
        let mut p = Preset::new(Preset::TYPE_PRINT, "MyPreset", false);
        o.emit(vec![s("label"), hex(&p.label(true)), hex(&p.label(false))]);
        p.alias = s("Alias");
        o.emit(vec![s("label"), hex(&p.label(true)), hex(&p.label(false))]);
        p.is_dirty = true;
        o.emit(vec![s("label"), hex(&p.label(true)), hex(&p.label(false))]);
    }
    for (name, alias, vendor) in [
        ("Bambu PLA Basic @BBL X1C", "", "Bambu Lab"),
        ("  Foo Bar  @x", "", ""),
        ("Acme ABS @BBL A1", "Acme ABS", "Acme"),
        ("Acme", "", "Acme"),
        ("Generic PLA", "Generic PLA", "Generic"),
        ("NoAt", "", "Bambu"),
        ("@lead", "", ""),
    ] {
        let mut p = Preset::new(Preset::TYPE_FILAMENT, name, false);
        p.alias = s(alias);
        if !vendor.is_empty() {
            p.config.set_deserialize_strict("filament_vendor", &format!("\"{}\"", vendor), false).unwrap();
        }
        o.emit(vec![s("display_name"), hex(name), hex(&p.display_name())]);
    }
    {
        let mut c = make_cfg(&[
            ("filament_diameter", "1.75,2.85,1.75"),
            ("nozzle_temperature", "200"),
            ("filament_type", "PLA"),
            ("filament_settings_id", "\"a\""),
            ("compatible_printers", "\"x\""),
            ("filament_dev_ams_drying_time", "8"),
            ("filament_density", "1.24"),
            ("relative_correction", "1,0.5"),
            ("filament_cost", "20,30"),
        ]);
        Preset::normalize(&mut c).unwrap();
        dump_cfg(o, "normalize", &c);
        let mut c2 = make_cfg(&[("nozzle_temperature", "200"), ("filament_settings_id", "\"a\"")]);
        Preset::normalize(&mut c2).unwrap();
        dump_cfg(o, "normalize2", &c2);
        let def = make_cfg(&[("nozzle_temperature", "200"), ("filament_type", "PLA")]);
        let mut c3 = make_cfg(&[("nozzle_temperature", "210"), ("layer_height", "0.2"), ("filament_type", "PETG"), ("wall_loops", "2")]);
        let r = Preset::remove_invalid_keys(&mut c3, &def);
        o.emit(vec![s("remove_invalid_keys"), hex(&r)]);
        dump_cfg(o, "remove_invalid_keys", &c3);
    }
    for t in 0..=9 {
        let mut id_name = String::new();
        let mut variant_name = String::new();
        let mut s1: &'static BTreeSet<String> = &slicer::print_config::config_def::empty_options;
        let mut s2: &'static BTreeSet<String> = &slicer::print_config::config_def::empty_options;
        Preset::get_extruder_names_and_keysets(ty(t), &mut id_name, &mut variant_name, &mut s1, &mut s2);
        o.emit(vec![s("keysets"), num(t as i64), hex(&id_name), hex(&variant_name), num(s1.len() as i64), num(s2.len() as i64)]);
    }
    {
        let mut p = Preset::new(Preset::TYPE_PRINTER, "pt", false);
        o.emit(vec![s("printer_technology"), num(p.printer_technology().to_int() as i64)]);
        p.config.set_deserialize_strict("printer_technology", "SLA", false).unwrap();
        o.emit(vec![
            s("printer_technology"),
            num(p.printer_technology().to_int() as i64),
            num(Preset::printer_technology_cfg(&p.config).to_int() as i64),
        ]);
        *p.printer_technology_ref() = PrinterTechnology::PtFFF.to_int();
        o.emit(vec![s("printer_technology"), num(p.printer_technology().to_int() as i64)]);
    }
}

fn push_vendor(l: &mut Vec<String>, v: &VendorProfile) {
    l.push(hex(&v.name));
    l.push(hex(&v.id));
    l.push(hex(&v.config_version.to_string()));
    l.push(hex(&v.config_update_url));
    l.push(hex(&v.changelog_url));
    l.push(num(v.models.len() as i64));
    for m in &v.models {
        l.push(hex(&m.id));
        l.push(hex(&m.name));
        l.push(num(m.technology.to_int() as i64));
        l.push(hex(&m.family));
        l.push(num(m.variants.len() as i64));
        for var in &m.variants {
            l.push(hex(&var.name));
        }
        push_list(l, &m.default_materials);
        l.push(hex(&m.bed_model));
        l.push(hex(&m.bed_texture));
    }
    push_list(l, &v.default_filaments.iter().cloned().collect::<Vec<_>>());
    push_list(l, &v.default_sla_materials.iter().cloned().collect::<Vec<_>>());
    l.push(num(v.num_variants() as i64));
    push_list(l, &v.families());
    l.push(b(v.valid()));
}

fn gen_vendor(o: &mut Out) {
    let dir = format!("{}/vendor", o.d);
    write_file(
        &format!("{dir}/Acme.ini"),
        concat!(
            "[vendor]\nname = Acme\nconfig_version = 1.2.3\nconfig_update_url = https://example.invalid/acme\n",
            "changelog_url = https://example.invalid/log\n\n",
            "[printer_model:MK3]\nname = Original MK3\nvariants = 0.4; 0.6; 0.4\ntechnology = FFF\n",
            "default_materials = \"PLA @Acme\"; \"PETG @Acme\"; \"\"; \"PLA @Acme\"\nbed_model = mk3_bed.stl\nbed_texture = mk3.svg\n\n",
            "[printer_model:SL1]\nvariants = default\nfamily = SLA\n\n",
            "[printer_model:NOVAR]\nname = No variants\n\n",
            "[printer_model:BADTECH]\ntechnology = LASER\nvariants = 0.4\ndefault_filaments = \"X\"\n\n",
            "[printer_model:BADVAR]\nvariants = \"unterminated\n\n",
            "[default_filaments]\nPLA @Acme = 1\nPETG @Acme = 0\n\n[default_sla_materials]\nResin = 1\n",
        ),
    );
    write_file(
        &format!("{dir}/BBL.ini"),
        "[vendor]\nname = BBL\nconfig_version = 2.0.0\n[printer_model:MK3]\nvariants = 0.4\n[printer_model:MK2S]\nvariants=0.4\nfamily = Custom\n[printer_model:X9]\nvariants=0.4\n",
    );
    write_file(&format!("{dir}/NoVendor.ini"), "[other]\na = 1\n");
    write_file(&format!("{dir}/NoName.ini"), "[vendor]\nconfig_version = 1.0.0\n");
    write_file(&format!("{dir}/BadVersion.ini"), "[vendor]\nname = X\nconfig_version = one\n");
    write_file(&format!("{dir}/BadIni.ini"), "[vendor\n");
    for (f, all) in [
        ("Acme.ini", true),
        ("Acme.ini", false),
        ("BBL.ini", true),
        ("NoVendor.ini", true),
        ("NoName.ini", true),
        ("BadVersion.ini", true),
        ("BadIni.ini", true),
        ("Missing.ini", true),
    ] {
        let mut l = vec![s("vendor"), hex(f), b(all)];
        match VendorProfile::from_ini(&format!("{dir}/{f}"), all) {
            Ok(v) => push_vendor(&mut l, &v),
            Err(e) => {
                let m = o.rel(&format!("!{}", e.what()));
                l.push(hex(&m));
            }
        }
        o.emit(l);
    }
    let mut pm = PrinterModel::default();
    pm.bottom_texture_end_name = s("end");
    pm.middle_texture_rect = s("1,2,3,4");
    pm.use_double_extruder_default_texture = s("true");
    let mut l = vec![s("bed_texture_maps")];
    for (k, v) in pm.get_bed_texture_maps() {
        l.push(hex(&k));
        l.push(hex(&v));
    }
    l.push(b(pm.variant("x").is_none()));
    l.push(hex(&pm.support_side_panel_fan));
    o.emit(l);
    std::fs::remove_dir_all(&dir).unwrap();
}

fn list(o: &mut Out, tag: &str, v: &[String]) {
    let mut l = vec![s(tag)];
    push_list(&mut l, v);
    o.emit(l);
}
fn name_or_null(p: Option<&Preset>) -> String {
    hex(&p.map(|p| p.name.clone()).unwrap_or_else(|| s("<null>")))
}
fn compat_line(o: &mut Out, tag: &str, f: &PresetCollection, with_edited: bool) {
    let mut l = vec![s(tag), idx(f.get_selected_idx())];
    for p in f.get_presets() {
        l.push(b(p.is_compatible));
    }
    if with_edited {
        l.push(b(f.get_edited_preset().is_compatible));
    }
    o.emit(l);
}

#[allow(clippy::too_many_arguments)]
fn ext(
    o: &mut Out,
    filaments: &mut PresetCollection,
    tag: &str,
    path: &str,
    name: &str,
    original: &str,
    cfg: &DynamicPrintConfig,
    d: &BTreeSet<String>,
    sel: LoadAndSelect,
    fid: &str,
) {
    match filaments.load_external_preset(path, name, original, cfg, d, sel, Semver::parse("2.2.0.0").unwrap(), fid) {
        Ok((p, modified)) => {
            let pname = p.name.clone();
            let l = vec![s("external"), s(tag), hex(&pname), b(modified), idx(filaments.get_selected_idx()), hex(&filaments.get_edited_preset().name)];
            o.emit(l);
        }
        Err(e) => o.emit(vec![s("external"), s(tag), bang(&e)]),
    }
}

fn user(o: &mut Out, filaments: &mut PresetCollection, tag: &str, name: &str, values: &BTreeMap<String, String>) {
    let mut subs: PresetsConfigSubstitutions = Vec::new();
    match filaments.load_user_preset(name, values.clone(), &mut subs, ForwardCompatibilitySubstitutionRule::EnableSilent) {
        Ok(r) => o.emit(vec![s("load_user_preset"), s(tag), b(r), num(subs.len() as i64), idx(filaments.get_selected_idx())]),
        Err(e) => o.emit(vec![s("load_user_preset"), s(tag), bang(&e)]),
    }
}

fn gen_collections(o: &mut Out) {
    let user_dir = format!("{}/user", o.d);

    // Vendors.
    let mut vendors: VendorMap = BTreeMap::new();
    {
        let mut bbl = VendorProfile::with_id(s("BBL"));
        bbl.name = s("BBL");
        bbl.config_version = Semver::parse("02.00.00.01").unwrap();
        let mut x1 = PrinterModel::default();
        x1.id = s("X1C");
        x1.name = s("Bambu Lab X1 Carbon");
        x1.model_id = s("BL-P001");
        x1.variants.push(PrinterVariant::with_name("0.4"));
        x1.variants.push(PrinterVariant::with_name("0.6"));
        x1.bed_model = s("x1_bed.stl");
        let mut a1 = PrinterModel::default();
        a1.id = s("A1");
        a1.name = s("Bambu Lab A1");
        a1.model_id = s("N2S");
        a1.variants.push(PrinterVariant::with_name("0.4"));
        bbl.models = vec![x1, a1];
        vendors.insert(s("BBL"), Arc::new(bbl));
        let mut custom = VendorProfile::with_id(s("Custom"));
        custom.name = s("Custom");
        let mut mk = PrinterModel::default();
        mk.id = s("MK1");
        mk.name = s("MyKlipper");
        mk.model_id = s("C11");
        custom.models = vec![mk];
        vendors.insert(s("Custom"), Arc::new(custom));
    }
    let bbl = vendors["BBL"].clone();

    // The printers of a PresetBundle (PresetBundle.cpp:359-431).
    let mut printers =
        PrinterPresetCollection::new(Preset::TYPE_PRINTER, Preset::printer_options(), full_defaults(), "Default Printer").unwrap();
    {
        let preset = printers.default_preset_mut(0);
        for key in ["printer_settings_id", "printer_model", "printer_variant", "thumbnail_size"] {
            preset.config.option_mut(key, true).unwrap();
        }
        preset.config.option_mut("default_print_profile", true).unwrap();
        preset.config.option_typed_mut::<ConfigOptionStrings>("default_filament_profile", true).unwrap();
        preset.inherits_mut();
    }
    printers.select_preset(0);
    {
        let mut add_printer = |name: &str, model: &str, variant: &str, nozzle: &str, system: bool, inherits: &str| {
            let c = make_cfg(&[("printer_model", model), ("printer_variant", variant), ("nozzle_diameter", nozzle), ("inherits", inherits)]);
            let p = printers.load_preset(&format!("<sys>/{}.json", name), name, &c, false, Semver::new()).unwrap();
            p.is_system = system;
            if system {
                p.vendor = Some(bbl.clone());
            }
        };
        add_printer("Bambu Lab X1 Carbon 0.4 nozzle", "Bambu Lab X1 Carbon", "0.4", "0.4", true, "");
        add_printer("Bambu Lab X1 Carbon 0.6 nozzle", "Bambu Lab X1 Carbon", "0.6", "0.6", true, "");
        add_printer("Bambu Lab A1 0.4 nozzle", "Bambu Lab A1", "0.4", "0.4,0.4", true, "");
        add_printer("My X1C", "Bambu Lab X1 Carbon", "0.4", "0.4", false, "Bambu Lab X1 Carbon 0.4 nozzle");
        add_printer("Klipper Box", "MyKlipper", "0.4", "0.4", false, "");
    }
    let r = printers.select_preset_by_name("Bambu Lab X1 Carbon 0.4 nozzle", false, false);
    o.emit(vec![s("select_printer"), b(r)]);
    dump_coll(o, "printers", &printers);

    // PrinterPresetCollection
    {
        let p = printers.find_system_preset_by_model_and_variant("Bambu Lab X1 Carbon", "0.6");
        o.emit(vec![s("find_system_preset"), name_or_null(p)]);
        let p = printers.find_system_preset_by_model_and_variant("Bambu Lab X1 Carbon", "");
        o.emit(vec![s("find_system_preset"), name_or_null(p)]);
        let p = printers.find_system_preset_by_model_and_variant("MyKlipper", "");
        o.emit(vec![s("find_system_preset"), name_or_null(p)]);
        let p = printers.find_custom_preset_by_model_and_variant("MyKlipper", "0.4");
        o.emit(vec![s("find_custom_preset"), name_or_null(p)]);
        let p = printers.find_custom_preset_by_model_and_variant("", "0.4");
        o.emit(vec![s("find_custom_preset"), name_or_null(p)]);
        for so in [true, false] {
            let mut l = vec![s("find_all_presets_by_model"), b(so)];
            for q in printers.find_all_presets_by_model("Bambu Lab X1 Carbon", so) {
                l.push(hex(&q.name));
            }
            o.emit(l);
        }
        o.emit(vec![s("only_default_printers"), b(printers.only_default_printers())]);
        let n = printers.default_preset_for(&make_cfg(&[("printer_technology", "SLA")])).name.clone();
        o.emit(vec![s("default_preset_for"), hex(&n)]);
        let mut generic = DynamicPrintConfig::new();
        generic.set_key_value(
            "printer_technology",
            Box::new(ConfigOptionEnumGeneric::with_value(
                Some(ConfigOptionEnum::<PrinterTechnology>::get_enum_values()),
                PrinterTechnology::PtSLA.to_int(),
            )),
        );
        let n = printers.default_preset_for(&generic).name.clone();
        o.emit(vec![s("default_preset_for"), hex(&n)]);
        let d = printers.diameters_of_selected_printer();
        list(o, "diameters", &d);
        let h = PhysicalPrinter::presets_with_print_host_information(&printers);
        list(o, "presets_with_print_host_information", &h);
    }

    // Prints.
    let mut prints = PresetCollection::new(Preset::TYPE_PRINT, Preset::print_options(), full_defaults(), "Default Setting").unwrap();
    prints.default_preset_mut(0).config.option_mut("print_settings_id", true).unwrap();
    prints.default_preset_mut(0).compatible_printers_condition_mut();
    prints.default_preset_mut(0).inherits_mut();
    prints.select_preset(0);
    for (nm, lh) in [("0.20mm Standard @BBL X1C", "0.2"), ("0.16mm Optimal @BBL X1C", "0.16")] {
        let c = make_cfg(&[("layer_height", lh), ("compatible_printers", "\"Bambu Lab X1 Carbon 0.4 nozzle\"")]);
        let p = prints.load_preset("<sys>/p.json", nm, &c, false, Semver::new()).unwrap();
        p.is_system = true;
        p.vendor = Some(bbl.clone());
    }
    prints.update_map_alias_to_profile_name();
    prints.select_preset_by_name("0.16mm Optimal @BBL X1C", false, false);

    // Filaments, as PresetBundle::PresetBundle() makes them.
    let mut filaments = PresetCollection::new(Preset::TYPE_FILAMENT, Preset::filament_options(), full_defaults(), "Default Filament").unwrap();
    filaments.default_preset_mut(0).config.option_typed_mut::<ConfigOptionStrings>("filament_settings_id", true).unwrap().unwrap().values =
        vec![String::new()];
    filaments.default_preset_mut(0).compatible_printers_condition_mut();
    filaments.default_preset_mut(0).inherits_mut();
    {
        let default_config = &mut filaments.default_preset_mut(0).config;
        for opt_key in default_config.keys() {
            let opt = default_config.option_mut(&opt_key, false).unwrap().unwrap();
            let is_override_key = is_filament_extruder_override_key(&opt_key);
            if !is_override_key || !opt.nullable() {
                continue;
            }
            let _ = opt.deserialize("nil", false);
        }
    }
    filaments.select_preset(0);
    let dc = filaments.default_preset(0).config.clone();
    dump_cfg(o, "default_filament", &dc);

    let x1c04 = "\"Bambu Lab X1 Carbon 0.4 nozzle\"";
    let x1c_both = format!("{};\"Bambu Lab X1 Carbon 0.6 nozzle\"", x1c04);
    let sys: Vec<(&str, &str, &str, &str, Vec<(&str, &str)>)> = vec![
        (
            "Bambu PLA Basic @BBL X1C",
            "Bambu PLA Basic",
            "GFA00",
            "",
            vec![("filament_type", "\"PLA\""), ("filament_vendor", "\"Bambu Lab\""), ("compatible_printers", &x1c_both), ("nozzle_temperature", "220")],
        ),
        (
            "Bambu PLA Basic @BBL A1",
            "Bambu PLA Basic",
            "GFA00",
            "",
            vec![("filament_type", "\"PLA\""), ("filament_vendor", "\"Bambu Lab\""), ("compatible_printers", "\"Bambu Lab A1 0.4 nozzle\"")],
        ),
        (
            "Bambu PLA Basic @BBL X1C 0.6",
            "Bambu PLA Basic",
            "GFA00",
            "",
            vec![("filament_type", "\"PLA\""), ("filament_vendor", "\"Bambu Lab\""), ("compatible_printers", "\"Bambu Lab X1 Carbon 0.6 nozzle\"")],
        ),
        (
            "Generic PETG @BBL X1C",
            "Generic PETG",
            "GFG99",
            "Generic PETG Old",
            vec![("filament_type", "\"PETG\""), ("filament_vendor", "\"Generic\""), ("compatible_printers", x1c04)],
        ),
        ("Generic ABS", "Generic ABS", "GFB99", "", vec![("filament_type", "\"ABS\""), ("filament_vendor", "\"Generic\"")]),
        (
            "Special For Optimal",
            "Special",
            "GFS00",
            "",
            vec![
                ("filament_type", "\"PLA\""),
                ("compatible_prints", "\"0.16mm Optimal @BBL X1C\""),
                ("compatible_printers_condition", "nozzle_diameter[0]==0.4"),
            ],
        ),
        ("Conditional", "Conditional", "GFC00", "", vec![("filament_type", "\"PLA\""), ("compatible_printers_condition", "nozzle_diameter[0]==0.8")]),
    ];
    for (name, alias, fid, renamed, kvs) in &sys {
        let c = make_cfg(kvs);
        let p = filaments.load_preset(&format!("<sys>/{}.json", name), name, &c, false, Semver::new()).unwrap();
        p.is_system = true;
        p.vendor = Some(bbl.clone());
        p.alias = s(alias);
        p.filament_id = s(fid);
        if !renamed.is_empty() {
            p.renamed_from.push(s(renamed));
        }
    }
    {
        // A two-extruder system preset: the variant options extended to two values.
        let mut c = filaments.default_preset(0).config.clone();
        c.apply(
            &make_cfg(&[
                ("filament_type", "\"PLA\""),
                ("filament_vendor", "\"Bambu Lab\""),
                ("filament_extruder_variant", "\"Direct Drive Standard\";\"Direct Drive High Flow\""),
                ("compatible_printers", "\"Bambu Lab H2D 0.4 nozzle\""),
                ("nozzle_temperature", "220,230"),
                ("filament_max_volumetric_speed", "20,22"),
            ]),
            false,
        )
        .unwrap();
        extend_default_config_length(&mut c, &DynamicPrintConfig::new(), false, &DynamicPrintConfig::new()).unwrap();
        let p = filaments.load_preset_move("<sys>/h2d.json", "Bambu PLA Basic @BBL H2D", c, false, Semver::new()).unwrap();
        p.is_system = true;
        p.vendor = Some(bbl.clone());
        p.alias = s("Bambu PLA Basic");
        p.filament_id = s("GFA00");
    }
    filaments.update_map_alias_to_profile_name();
    filaments.update_map_system_profile_renamed();
    dump_coll(o, "sys", &filaments);

    // User presets on disk.
    let fd = format!("{}/filament", user_dir);
    write_file(
        &format!("{fd}/base/Custom Root @Bambu Lab X1 Carbon 0.4 nozzle.json"),
        r#"{"version": "2.0.0.0", "name": "Custom Root @Bambu Lab X1 Carbon 0.4 nozzle", "from": "User", "inherits": "", "filament_type": ["PETG"], "filament_vendor": ["Acme"], "nozzle_temperature": ["245"]}"#,
    );
    write_file(
        &format!("{fd}/My PLA.json"),
        r#"{"version": "2.1.0.0", "inherits": "Bambu PLA Basic @BBL X1C", "nozzle_temperature": ["225"], "filament_settings_id": ["My PLA"]}"#,
    );
    write_file(&format!("{fd}/My PLA.info"), "sync_info = update\nuser_id = u1\nsetting_id = S1\nbase_id = B1\nupdated_time = 1000\n");
    write_file(&format!("{fd}/My PLA 2.json"), r#"{"version": "2.1.0.0", "inherits": "My PLA"}"#);
    write_file(&format!("{fd}/Broken.json"), r#"{ "version": "#);
    write_file(&format!("{fd}/Broken.info"), "updated_time = 5\n");
    write_file(&format!("{fd}/Future.json"), r#"{"version": "3.0.0.0", "inherits": "Generic ABS"}"#);
    write_file(&format!("{fd}/NoVersion.json"), r#"{"inherits": "Generic ABS"}"#);
    write_file(
        &format!("{fd}/Invisible.json"),
        r#"{"version": "2.0.0.0", "inherits": "Generic ABS", "instantiation": "false", "description": "hidden one", "filament_id": "FX1"}"#,
    );
    write_file(&format!("{fd}/SameIds.json"), r#"{"version": "2.0.0.0", "inherits": "Generic ABS", "filament_cost": ["30"]}"#);
    write_file(&format!("{fd}/SameIds.info"), "setting_id = X\nbase_id = X\nuser_id = null\nupdated_time = 2000\n");
    write_file(&format!("{fd}/Orphan.json"), r#"{"version": "2.0.0.0", "inherits": "Nonexistent"}"#);
    write_file(&format!("{fd}/Bambu PLA Basic @BBL X1C.json"), r#"{"version": "2.0.0.0"}"#);
    write_file(
        &format!("{fd}/Root2 @Bambu Lab A1 0.4 nozzle.json"),
        r#"{"version": "2.0.0.0", "inherits": "", "filament_type": ["PLA"], "filament_id": "R2"}"#,
    );
    write_file(&format!("{fd}/Root2 @Bambu Lab A1 0.4 nozzle.info"), "sync_info = \nuser_id = \nsetting_id = null\nbase_id = null\nupdated_time = 0\n");
    write_file(&format!("{fd}/Dual.json"), r#"{"version": "2.0.0.0", "inherits": "Bambu PLA Basic @BBL H2D", "nozzle_temperature": ["225", "235"]}"#);
    write_file(&format!("{fd}/Single.json"), r#"{"version": "2.0.0.0", "inherits": "Bambu PLA Basic @BBL H2D", "nozzle_temperature": ["228"]}"#);
    write_file(&format!("{fd}/BadInfo.json"), r#"{"version": "2.0.0.0", "inherits": "Generic ABS"}"#);
    write_file(&format!("{fd}/BadInfo.info"), "this is not an ini line\n");
    write_file(&format!("{fd}/readme.txt"), "ignored");
    dump_files(o, "before_load");

    let mut subs: PresetsConfigSubstitutions = Vec::new();
    match filaments.load_presets(&user_dir, "filament", &mut subs, ForwardCompatibilitySubstitutionRule::EnableSilent) {
        Ok(()) => o.emit(vec![s("load_presets"), s("ok"), num(subs.len() as i64)]),
        Err(e) => o.emit(vec![s("load_presets"), bang(&e)]),
    }
    dump_coll(o, "loaded", &filaments);
    for n in ["My PLA", "Dual", "Single", "Custom Root @Bambu Lab X1 Carbon 0.4 nozzle", "Root2 @Bambu Lab A1 0.4 nozzle"] {
        let c = filaments.find_preset(n, false).unwrap().config.clone();
        dump_cfg(o, &format!("loaded:{}", n), &c);
    }
    dump_files(o, "after_load");

    // Lookups.
    for n in ["My PLA", "Generic PETG Old", "Nope", "Default Filament", "Invisible", ""] {
        let p = name_or_null(filaments.find_preset(n, false));
        let p2 = name_or_null(filaments.find_preset(n, true));
        o.emit(vec![s("find_preset"), hex(n), p, p2]);
        let pr = name_or_null(filaments.find_preset_mut(n, false, true).map(|p| &*p));
        o.emit(vec![s("find_preset_real"), hex(n), pr]);
    }
    for n in ["Bambu PLA Basic", "Generic PETG", "Custom Root", "Root2", "Nope", ""] {
        let a = filaments.get_preset_name_by_alias(n);
        o.emit(vec![s("alias"), hex(n), hex(&a)]);
    }
    for n in ["Generic PETG Old", "Nope"] {
        let r = filaments.get_preset_name_renamed(n).cloned().unwrap_or_else(|| s("<null>"));
        o.emit(vec![s("renamed"), hex(n), hex(&r)]);
    }
    for i in 0..filaments.get_presets().len() {
        let p = &filaments.get_presets()[i];
        let parent = name_or_null(filaments.get_preset_parent(p));
        let base = name_or_null(filaments.get_preset_base(p));
        let pwv = filaments.get_preset_with_vendor_profile(p);
        let vid = pwv.vendor.map(|v| v.id.clone()).unwrap_or_else(|| s("<null>"));
        let path = o.rel(&filaments.path_for_preset(p));
        let l = vec![
            s("lineage"),
            num(i as i64),
            parent,
            base,
            hex(&vid),
            b(filaments.is_bbl_brand_filament(p)),
            b(filaments.is_base_preset(p)),
            hex(&path),
        ];
        o.emit(l);
    }
    let spn = filaments.system_preset_names();
    list(o, "system_preset_names", &spn);
    o.emit(vec![
        s("names"),
        hex(&filaments.name()),
        hex(&filaments.section_name()),
        hex(&prints.name()),
        hex(&printers.name()),
        hex(&printers.section_name()),
    ]);
    for (a, inh) in [("Generic ABS", ""), ("Generic PETG Old", ""), ("My PLA", ""), ("My PLA", "Generic ABS"), ("Nope", "Nope2"), ("Default Filament", "")] {
        o.emit(vec![s("validate_preset"), hex(a), hex(inh), b(filaments.validate_preset(a, inh))]);
    }
    for a in ["Bambu PLA Basic", "Custom Root", "Nope"] {
        o.emit(vec![s("is_alias_exist"), hex(a), b(filaments.is_alias_exist(a, None))]);
        let mut q = Preset::new(Preset::TYPE_FILAMENT, "Q", false);
        q.config = filaments.default_preset(0).config.clone();
        q.config.set_deserialize_strict("compatible_printers", x1c04, false).unwrap();
        o.emit(vec![s("is_alias_exist_p"), hex(a), b(filaments.is_alias_exist(a, Some(&q)))]);
    }
    let p1 = o.rel(&filaments.path_from_name("abc", false));
    let p2 = o.rel(&filaments.path_from_name("abc.JSON", true));
    o.emit(vec![s("path_from_name"), hex(&p1), hex(&p2)]);
    {
        let mut q = filaments.find_preset("Root2 @Bambu Lab A1 0.4 nozzle", false).unwrap().clone();
        q.alias.clear();
        q.name = s("Another Root @Bambu Lab A1 0.4 nozzle");
        let a = filaments.get_preset_alias(&mut q, false);
        o.emit(vec![s("get_preset_alias"), hex(&a), hex(&q.alias)]);
        let mut q2 = q.clone();
        q2.alias.clear();
        q2.name = s("Third Root @Bambu Lab X1 Carbon 0.4 nozzle");
        q2.config.set_deserialize_strict("compatible_printers", x1c04, false).unwrap();
        q2.config.set_deserialize_strict("inherits", "Generic ABS", false).unwrap();
        let a = filaments.get_preset_alias(&mut q2, true);
        o.emit(vec![s("get_preset_alias"), hex(&a), hex(&q2.alias)]);
        let a = filaments.get_preset_alias(&mut q2, false);
        o.emit(vec![s("get_preset_alias"), hex(&a), hex(&q2.alias)]);
    }

    // Selection, edits and dirty tracking.
    o.emit(vec![s("first_visible_idx"), num(filaments.first_visible_idx() as i64), num(filaments.first_compatible_idx() as i64)]);
    let r = filaments.select_preset_by_name("My PLA", false, false);
    o.emit(vec![s("select_by_name"), b(r)]);
    let r = filaments.select_preset_by_name("My PLA", false, false);
    o.emit(vec![s("select_by_name"), b(r)]);
    let r = filaments.select_preset_by_name(" (modified)My PLA", true, false);
    o.emit(vec![s("select_by_name"), b(r)]);
    o.emit(vec![
        s("selected"),
        idx(filaments.get_selected_idx()),
        hex(&filaments.get_selected_preset_name()),
        b(filaments.current_is_dirty().unwrap()),
        b(filaments.saved_is_dirty().unwrap()),
    ]);
    filaments.get_edited_preset_mut().config.set_deserialize_strict("nozzle_temperature", "231", false).unwrap();
    filaments.get_edited_preset_mut().config.set_deserialize_strict("filament_settings_id", "\"Other\"", false).unwrap();
    o.emit(vec![s("dirty"), b(filaments.current_is_dirty().unwrap()), b(filaments.saved_is_dirty().unwrap())]);
    let v = filaments.current_dirty_options(false).unwrap();
    list(o, "current_dirty_options", &v);
    let v = filaments.current_dirty_options(true).unwrap();
    list(o, "current_dirty_options_deep", &v);
    let v = filaments.current_different_from_parent_options(false).unwrap();
    list(o, "different_from_parent", &v);
    let v = filaments.current_different_from_parent_options(true).unwrap();
    list(o, "different_from_parent_deep", &v);
    {
        let parent = name_or_null(filaments.get_selected_preset_parent());
        o.emit(vec![s("selected_parent"), parent]);
    }
    let ign: BTreeSet<String> = [s("nozzle_temperature")].into_iter().collect();
    let v = PresetCollection::dirty_options_without_option_list(Some(filaments.get_edited_preset()), Some(filaments.get_selected_preset()), &ign, false)
        .unwrap();
    list(o, "dirty_without", &v);
    let ign: BTreeSet<String> = [s("nozzle_temperature#0")].into_iter().collect();
    let v = PresetCollection::dirty_options_without_option_list(Some(filaments.get_edited_preset()), Some(filaments.get_selected_preset()), &ign, true)
        .unwrap();
    list(o, "dirty_without_deep", &v);
    let u1 = filaments.update_dirty().unwrap();
    let u2 = filaments.update_dirty().unwrap();
    o.emit(vec![s("update_dirty"), b(u1), b(u2), b(filaments.get_selected_preset().is_dirty)]);
    {
        let a = filaments.get_edited_preset().clone();
        let mut r = filaments.get_selected_preset().clone();
        r.config.erase("compatible_prints");
        o.emit(vec![
            s("is_dirty_optional"),
            b(PresetCollection::is_dirty(Some(&a), Some(&r)).unwrap()),
            b(PresetCollection::is_dirty(Some(&a), None).unwrap()),
        ]);
        let v = PresetCollection::dirty_options(Some(&a), Some(&r), false).unwrap();
        list(o, "dirty_optional", &v);
        let v = PresetCollection::dirty_options(Some(&r), Some(&a), true).unwrap();
        list(o, "dirty_optional_deep", &v);
    }

    // Save as a new preset, then over it, then detached.
    filaments.save_current_preset("My PLA Saved", false, false, None, None).unwrap();
    dump_coll(o, "saved1", &filaments);
    filaments.get_edited_preset_mut().config.set_deserialize_strict("nozzle_temperature", "232", false).unwrap();
    filaments.save_current_preset("My PLA Saved", false, false, None, None).unwrap();
    filaments.select_preset_by_name("Dual", false, false);
    filaments.get_edited_preset_mut().config.set_deserialize_strict("nozzle_temperature", "225,240", false).unwrap();
    filaments.save_current_preset("Dual Saved", false, false, None, None).unwrap();
    filaments.select_preset_by_name("Generic ABS", false, false);
    filaments.get_edited_preset_mut().config.set_deserialize_strict("filament_cost", "40", false).unwrap();
    filaments.save_current_preset("ABS Detached", true, false, None, None).unwrap();
    filaments.select_preset_by_name("Custom Root @Bambu Lab X1 Carbon 0.4 nozzle", false, false);
    filaments.save_current_preset("Custom Child @Bambu Lab X1 Carbon 0.4 nozzle", false, false, None, None).unwrap();
    filaments.select_preset_by_name("Custom Root @Bambu Lab X1 Carbon 0.4 nozzle", false, false);
    filaments.get_edited_preset_mut().config.set_deserialize_strict("filament_cost", "55", false).unwrap();
    filaments.save_current_preset("Custom Root @Bambu Lab X1 Carbon 0.4 nozzle", false, false, None, None).unwrap();
    filaments.save_current_preset("Generic ABS", false, false, None, None).unwrap();
    dump_coll(o, "saved2", &filaments);
    let c = filaments.find_preset("Custom Child @Bambu Lab X1 Carbon 0.4 nozzle", false).unwrap().config.clone();
    dump_cfg(o, "custom_child_reloaded", &c);
    dump_files(o, "after_save");

    // Compatibility.
    {
        let print = prints.get_edited_preset_with_vendor_profile();
        filaments.select_preset_by_name("Generic PETG @BBL X1C", false, false);
        {
            let printer = printers.get_edited_preset_with_vendor_profile();
            filaments.update_compatible(&printer, Some(&print), PresetSelectCompatibleType::OnlyIfWasCompatible);
        }
        compat_line(o, "compat1", &filaments, true);
        printers.select_preset_by_name("My X1C", false, false);
        {
            let printer2 = printers.get_edited_preset_with_vendor_profile();
            filaments.update_compatible(&printer2, None, PresetSelectCompatibleType::Always);
        }
        compat_line(o, "compat2", &filaments, false);
        printers.select_preset_by_name("Bambu Lab X1 Carbon 0.4 nozzle", false, false);
        filaments.select_preset_by_name("Bambu PLA Basic @BBL X1C 0.6", false, false);
        filaments.set_extra_compatible_printers(&strs(&["Bambu Lab X1 Carbon 0.6 nozzle"]));
        {
            let printer3 = printers.get_edited_preset_with_vendor_profile();
            filaments.update_compatible_with(&printer3, None, PresetSelectCompatibleType::Always, |p: &Preset| if p.is_system { 1 } else { 0 });
        }
        filaments.clear_extra_compatible_printers();
        compat_line(o, "compat3", &filaments, false);
        let a = filaments.get_preset_name_by_alias("Bambu PLA Basic");
        o.emit(vec![
            s("alias_after_compat"),
            hex(&a),
            num(filaments.first_compatible_idx() as i64),
            num(filaments.first_compatible_idx_with(|p: &Preset| if p.name.contains("Generic") { 5 } else { 0 }) as i64),
        ]);
        printers.select_preset_by_name("Bambu Lab A1 0.4 nozzle", false, false);
        {
            let printer4 = printers.get_edited_preset_with_vendor_profile();
            filaments.update_compatible(&printer4, None, PresetSelectCompatibleType::Never);
        }
        compat_line(o, "compat4", &filaments, false);
        let a = filaments.get_preset_name_by_alias("Bambu PLA Basic");
        o.emit(vec![s("alias_after_compat4"), hex(&a)]);
        {
            let printer4 = printers.get_edited_preset_with_vendor_profile();
            filaments.update_compatible(&printer4, None, PresetSelectCompatibleType::OnlyIfWasCompatible);
        }
        o.emit(vec![s("compat5"), idx(filaments.get_selected_idx()), hex(&filaments.get_selected_preset_name())]);
        // The free functions.
        printers.select_preset_by_name("My X1C", false, false);
        {
            let pla = filaments.find_preset("Bambu PLA Basic @BBL X1C", false).unwrap();
            let pv = filaments.get_preset_with_vendor_profile(pla);
            let myx1c = printers.get_edited_preset_with_vendor_profile();
            let l = vec![
                s("free"),
                b(is_compatible_with_printer_2(&pv, &myx1c)),
                b(is_compatible_with_parent_printer(&pv, &myx1c)),
                b(is_compatible_with_printer_names(&pv, &strs(&["x", "Bambu Lab X1 Carbon 0.6 nozzle"]))),
                b(is_compatible_with_printer_names(&pv, &[])),
                b(is_compatible_with_print(&pv, &print, &myx1c)),
            ];
            o.emit(l);
        }
        printers.select_preset_by_name("Bambu Lab X1 Carbon 0.4 nozzle", false, false);
    }
    dump_coll(o, "compat", &filaments);

    // External presets.
    {
        let none: BTreeSet<String> = BTreeSet::new();
        let diff: BTreeSet<String> = [s("nozzle_temperature")].into_iter().collect();
        let pla = filaments.find_preset("Bambu PLA Basic @BBL X1C", false).unwrap().config.clone();
        ext(o, &mut filaments, "same", "/x/a.3mf", "a.3mf", "Bambu PLA Basic @BBL X1C", &pla, &none, LoadAndSelect::Always, "");
        let mut pla2 = pla.clone();
        pla2.set_deserialize_strict("nozzle_temperature", "199", false).unwrap();
        ext(o, &mut filaments, "modified_never", "/x/a.3mf", "a.3mf", "Bambu PLA Basic @BBL X1C", &pla2, &none, LoadAndSelect::Never, "");
        ext(o, &mut filaments, "modified_again", "/x/a.3mf", "a.3mf", "Bambu PLA Basic @BBL X1C", &pla2, &none, LoadAndSelect::Never, "");
        let mut abs = filaments.find_preset("Generic ABS", false).unwrap().config.clone();
        abs.set_deserialize_strict("inherits", "Generic ABS", false).unwrap();
        abs.set_deserialize_strict("filament_cost", "77", false).unwrap();
        ext(o, &mut filaments, "inherits_always", "/x/b.json", "b.json", "", &abs, &none, LoadAndSelect::Always, "");
        ext(o, &mut filaments, "inherits_again", "/x/b.json", "b.json", "", &abs, &none, LoadAndSelect::OnlyIfModified, "FID");
        let mut abs2 = abs.clone();
        abs2.set_deserialize_strict("filament_cost", "78", false).unwrap();
        ext(o, &mut filaments, "inherits_new", "/x/b.json", "b.json", "", &abs2, &none, LoadAndSelect::OnlyIfModified, "");
        let mut petg = filaments.find_preset("Generic PETG @BBL X1C", false).unwrap().config.clone();
        petg.set_deserialize_strict("nozzle_temperature", "250", false).unwrap();
        ext(o, &mut filaments, "renamed", "/x/c.3mf", "c.3mf", "Generic PETG Old", &petg, &diff, LoadAndSelect::Always, "");
        let mut petg2 = petg.clone();
        petg2.set_deserialize_strict("inherits", "Generic PETG @BBL X1C", false).unwrap();
        petg2.set_deserialize_strict("filament_cost", "11", false).unwrap();
        ext(o, &mut filaments, "diff_settings", "/x/d.3mf", "d.3mf", "Unknown", &petg2, &diff, LoadAndSelect::OnlyIfModified, "");
        let mut root = filaments.find_preset("Root2 @Bambu Lab A1 0.4 nozzle", false).unwrap().config.clone();
        root.set_deserialize_strict("filament_cost", "12", false).unwrap();
        ext(o, &mut filaments, "noname", "/x/e.json", "e.json", "", &root, &none, LoadAndSelect::Always, "");
        ext(o, &mut filaments, "noname2", "/x/e.json", "e.json", "", &root, &none, LoadAndSelect::Never, "");
    }
    dump_coll(o, "external", &filaments);
    dump_files(o, "after_external");

    // Cloud presets.
    {
        let v: BTreeMap<String, String> = [
            ("version", "2.0.0.0"),
            ("setting_id", "SX"),
            ("updated_time", "500"),
            ("user_id", "U"),
            ("base_id", "B"),
            ("inherits", "Generic ABS"),
            ("nozzle_temperature", "240"),
            ("filament_id", "null"),
            ("name", "Cloud ABS"),
        ]
        .iter()
        .map(|(a, b)| (s(a), s(b)))
        .collect();
        user(o, &mut filaments, "new", "Cloud ABS", &v);
        user(o, &mut filaments, "same", "Cloud ABS", &v);
        let mut v2 = v.clone();
        v2.insert(s("updated_time"), s("900"));
        v2.insert(s("nozzle_temperature"), s("241"));
        user(o, &mut filaments, "newer", "Cloud ABS", &v2);
        let mut v3 = v.clone();
        v3.insert(s("updated_time"), s("100"));
        user(o, &mut filaments, "older", "Cloud ABS", &v3);
        let mut v4 = v.clone();
        v4.remove("version");
        user(o, &mut filaments, "no_version", "Cloud X", &v4);
        let mut v5 = v.clone();
        v5.remove("setting_id");
        user(o, &mut filaments, "no_setting", "Cloud X", &v5);
        let mut v6 = v.clone();
        v6.remove("base_id");
        user(o, &mut filaments, "no_base", "Cloud X", &v6);
        let mut v7 = v.clone();
        v7.insert(s("inherits"), s("Missing Parent"));
        user(o, &mut filaments, "missing_parent", "Cloud X", &v7);
        let mut v8 = v.clone();
        v8.insert(s("inherits"), s(""));
        v8.insert(s("filament_type"), s("PLA"));
        user(o, &mut filaments, "root", "Cloud Root @Bambu Lab X1 Carbon 0.4 nozzle", &v8);
        let mut v9 = v.clone();
        v9.insert(s("version"), s("9.0.0.0"));
        user(o, &mut filaments, "future", "Cloud X", &v9);
        let mut v10 = v.clone();
        v10.insert(s("filament_type"), s("\"PLA\";\"PETG\""));
        v10.insert(s("filament_scarf_seam_type"), s("bogus"));
        user(o, &mut filaments, "bad_enum", "A Cloud Enum", &v10);
        filaments.update_after_user_presets_loaded();
        o.emit(vec![s("after_user"), idx(filaments.get_selected_idx()), hex(&filaments.get_selected_preset_name())]);
    }
    o.emit(vec![
        s("need_sync"),
        b(filaments.need_sync("Cloud ABS", "SX", 900)),
        b(filaments.need_sync("Cloud ABS", "SX", 901)),
        b(filaments.need_sync("Cloud ABS", "SY", 1)),
        b(filaments.need_sync("Nope", "SX", 1)),
    ]);
    {
        let mut r: Vec<Preset> = Vec::new();
        let n = filaments.get_user_presets(None, &mut r);
        let mut l = vec![s("get_user_presets"), num(n as i64)];
        for p in &r {
            l.push(hex(&p.name));
        }
        o.emit(l);
    }
    for n in ["My PLA", "Custom Root @Bambu Lab X1 Carbon 0.4 nozzle", "Cloud ABS", "Generic ABS", "Dual Saved"] {
        let mut kv: BTreeMap<String, String> = BTreeMap::new();
        let p = filaments.find_preset_real(n, false, true).unwrap();
        let r = filaments.get_differed_values_to_update(p, &mut kv).unwrap();
        let mut l = vec![s("differed_values"), hex(n), num(r as i64), num(kv.len() as i64)];
        for (k, v) in &kv {
            l.push(hex(k));
            l.push(hex(&if k == "updated_time" { norm_time(c_atoll(v)) } else { v.clone() }));
        }
        o.emit(l);
        let d = filaments.get_preset_differed_for_save(p).unwrap();
        match d {
            Some(d) => {
                dump_preset(o, &format!("differed:{}", n), &d);
                dump_cfg(o, &format!("differed:{}", n), &d.config);
            }
            None => o.emit(vec![s("differed"), hex(n), s("null")]),
        }
    }
    filaments.set_sync_info_and_save("Custom Root @Bambu Lab X1 Carbon 0.4 nozzle", "SR", "update", 3000).unwrap();
    filaments.set_sync_info_and_save("My PLA", "S2", "", 0).unwrap();
    filaments.set_sync_info_and_save("Cloud ABS", "S3", "hold", 0).unwrap();
    {
        let mut del: Vec<String> = Vec::new();
        filaments.save_user_presets(&user_dir, "filament", &mut del).unwrap();
    }
    dump_coll(o, "cloud", &filaments);
    dump_files(o, "after_cloud");

    // Project presets.
    {
        let mut project = filaments.get_project_embedded_presets().unwrap();
        o.emit(vec![s("project_presets"), num(project.len() as i64)]);
        for p in &project {
            dump_preset(o, "project", p);
        }
        let r = filaments.reset_project_embedded_presets();
        o.emit(vec![s("reset_project"), b(r), idx(filaments.get_selected_idx())]);
        filaments.select_preset_by_name("Generic ABS", false, false);
        let mut subs: PresetsConfigSubstitutions = Vec::new();
        for p in project.iter_mut() {
            p.config.set_deserialize_strict("filament_cost", "99", false).unwrap();
        }
        let mut extra = Preset::new(Preset::TYPE_FILAMENT, "Orphan Project", false);
        extra.is_project_embedded = true;
        extra.config = make_cfg(&[("inherits", "Nope")]);
        project.push(extra);
        let mut wrong = Preset::new(Preset::TYPE_PRINT, "Wrong Type", false);
        wrong.is_project_embedded = true;
        project.push(wrong);
        filaments.load_project_embedded_presets(&mut project, "filament", &mut subs, ForwardCompatibilitySubstitutionRule::Enable).unwrap();
        o.emit(vec![s("load_project"), num(subs.len() as i64), idx(filaments.get_selected_idx())]);
    }
    dump_coll(o, "project", &filaments);

    // Clones (the templates are copied first: clone_presets copies them before it changes the collection).
    {
        let templates = |f: &PresetCollection| -> Vec<Preset> {
            vec![f.find_preset("Generic ABS", false).unwrap().clone(), f.find_preset("My PLA", false).unwrap().clone()]
        };
        let mut failures: Vec<String> = Vec::new();
        let t = templates(&filaments);
        let r = filaments.clone_presets_for_printer(&t.iter().collect::<Vec<_>>(), &mut failures, "Klipper Box", |p: String| format!("P{}", p), false).unwrap();
        let mut l = vec![s("clone_for_printer"), b(r)];
        push_list(&mut l, &failures);
        o.emit(l);
        failures.clear();
        let t = templates(&filaments);
        let r = filaments.clone_presets_for_printer(&t.iter().collect::<Vec<_>>(), &mut failures, "Klipper Box", |p: String| format!("Q{}", p), false).unwrap();
        let mut l = vec![s("clone_for_printer_again"), b(r)];
        push_list(&mut l, &failures);
        o.emit(l);
        failures.clear();
        let t = templates(&filaments);
        let r = filaments.clone_presets_for_printer(&t.iter().collect::<Vec<_>>(), &mut failures, "Klipper Box", |p: String| format!("R{}", p), true).unwrap();
        let mut l = vec![s("clone_for_printer_force"), b(r)];
        push_list(&mut l, &failures);
        o.emit(l);
        failures.clear();
        let mut dc = DynamicConfig::new();
        dc.set_key_value("filament_vendor", Box::new(ConfigOptionStrings::from_values(vec![s("Acme")])));
        dc.set_key_value("filament_type", Box::new(ConfigOptionStrings::from_values(vec![s("PETG")])));
        dc.set_key_value("compatible_printers", Box::new(ConfigOptionStrings::new()));
        let abs = filaments.find_preset("Generic ABS", false).unwrap().clone();
        let r = filaments.clone_presets_for_filament(&abs, &mut failures, "Acme Blue", "AC01", &dc, "Klipper Box", false).unwrap();
        let mut l = vec![s("clone_for_filament"), b(r)];
        push_list(&mut l, &failures);
        o.emit(l);
        let fp = filaments.get_filament_presets();
        let mut l = vec![s("filament_presets"), num(fp.len() as i64)];
        for (k, v) in &fp {
            l.push(hex(k));
            l.push(num(v.len() as i64));
            for i in v {
                l.push(hex(&filaments.get_presets()[*i].name));
            }
        }
        o.emit(l);
    }
    dump_coll(o, "clones", &filaments);

    // Visibility from the application configuration.
    {
        let mut app = AppConfig::new();
        app.set_variant("BBL", "Bambu Lab X1 Carbon", "0.4", true);
        app.set(AppConfig::SECTION_FILAMENTS, "Bambu PLA Basic @BBL A1", "1");
        app.set(AppConfig::SECTION_FILAMENTS, "Generic PETG Old", "1");
        app.set(AppConfig::SECTION_FILAMENTS, "Generic ABS", "");
        let mut l = vec![s("visible_from_appconfig")];
        for n in ["Bambu PLA Basic @BBL X1C", "Bambu PLA Basic @BBL A1", "Generic PETG @BBL X1C", "Generic ABS", "My PLA"] {
            let mut p = filaments.find_preset(n, false).unwrap().clone();
            p.set_visible_from_appconfig(&app);
            l.push(b(p.is_visible));
        }
        for n in ["Bambu Lab X1 Carbon 0.4 nozzle", "Bambu Lab X1 Carbon 0.6 nozzle", "My X1C"] {
            let mut p = printers.find_preset(n, false).unwrap().clone();
            p.set_visible_from_appconfig(&app);
            l.push(b(p.is_visible));
        }
        o.emit(l);
    }

    // Bundle lookups.
    for pn in ["Bambu Lab X1 Carbon 0.4 nozzle", "Bambu Lab A1 0.4 nozzle", "Klipper Box", "My X1C"] {
        printers.select_preset_by_name(pn, false, false);
        let bundle = PresetBundleView { printers: &printers, vendors: &vendors };
        let edited = printers.get_edited_preset();
        let f = filaments.find_preset("Generic PETG @BBL X1C", false).unwrap().clone();
        let g = filaments.find_preset("Generic ABS", false).unwrap().clone();
        let mut disp = String::new();
        let ft = f.get_filament_type(&mut disp);
        let spm = preset_utils::system_printer_model(edited).map(|m| m.id.clone()).unwrap_or_else(|| s("<null>"));
        let l = vec![
            s("bundle"),
            hex(pn),
            hex(&edited.get_printer_type(Some(&bundle))),
            hex(&edited.get_printer_name(Some(&bundle))),
            hex(&edited.get_current_printer_type(Some(&bundle))),
            b(edited.has_lidar(Some(&bundle))),
            b(edited.is_bbl_vendor_preset(Some(&bundle))),
            b(f.is_bbl_vendor_preset(Some(&bundle))),
            b(g.is_bbl_vendor_preset(Some(&bundle))),
            num(edited.get_default_bed_type(Some(&bundle)) as i64),
            b(edited.has_cali_lines(Some(&bundle))),
            hex(&ft),
            hex(&disp),
            hex(&edited.get_printer_id()),
            hex(&spm),
        ];
        o.emit(l);
        let l = vec![
            s("bundle_null"),
            hex(&edited.get_printer_type(None)),
            b(edited.has_lidar(None)),
            b(edited.is_bbl_vendor_preset(None)),
            num(edited.get_default_bed_type(None) as i64),
            b(edited.has_cali_lines(None)),
        ];
        o.emit(l);
    }

    // System printer models, extruder counts, the user directory.
    {
        let mut x = printers.find_preset("Bambu Lab X1 Carbon 0.4 nozzle", false).unwrap().clone();
        x.config.set_deserialize_strict("printer_model", "X1C", false).unwrap();
        let pm = preset_utils::system_printer_model(&x).map(|m| m.id.clone()).unwrap_or_else(|| s("<null>"));
        o.emit(vec![
            s("system_printer"),
            hex(&pm),
            hex(&preset_utils::system_printer_bed_model(&x)),
            hex(&preset_utils::system_printer_bed_texture(&x)),
            hex(&preset_utils::system_printer_hotend_model(&x)),
        ]);
        x.set_num_extruders(2).unwrap();
        dump_cfg(o, "set_num_extruders", &x.config);
        prints.update_user_presets_directory(&user_dir, "process").unwrap();
        let p = o.rel(&prints.path_from_name("p", false));
        o.emit(vec![s("user_presets_directory"), hex(&p)]);
    }

    // Deletion and defaults.
    filaments.select_preset_by_name("My PLA Saved", false, false);
    let r = filaments.delete_current_preset();
    o.emit(vec![s("delete_current"), b(r), idx(filaments.get_selected_idx()), hex(&filaments.get_selected_preset_name())]);
    filaments.select_preset_by_name("Custom Root @Bambu Lab X1 Carbon 0.4 nozzle", false, false);
    let r = filaments.delete_current_preset();
    o.emit(vec![s("delete_current_root"), b(r), idx(filaments.get_selected_idx())]);
    filaments.select_preset_by_name("Default Filament", false, false);
    let r = filaments.delete_current_preset();
    o.emit(vec![s("delete_current_default"), b(r)]);
    let r = filaments.delete_preset("Invisible");
    o.emit(vec![s("delete_preset"), b(r), idx(filaments.get_selected_idx())]);
    let r = filaments.delete_preset("Default Filament");
    o.emit(vec![s("delete_preset"), b(r)]);
    let r = filaments.delete_preset("Acme Blue @Klipper Box");
    o.emit(vec![s("delete_preset"), b(r), idx(filaments.get_selected_idx())]);
    filaments.set_default_suppressed(false);
    o.emit(vec![
        s("default_suppressed"),
        b(filaments.is_default_suppressed()),
        num(filaments.first_visible_idx() as i64),
        num(filaments.num_visible() as i64),
    ]);
    filaments.set_default_suppressed(true);
    let r = filaments.select_preset_by_name_strict("Generic ABS");
    o.emit(vec![s("strict"), b(r), idx(filaments.get_selected_idx())]);
    let r = filaments.select_preset_by_name_strict("Nope");
    o.emit(vec![s("strict"), b(r), idx(filaments.get_selected_idx())]);
    o.emit(vec![s("selected_name_invalid"), hex(&filaments.get_selected_preset_name())]);
    filaments.get_selected_preset_mut();
    o.emit(vec![s("selected_fixed"), idx(filaments.get_selected_idx())]);
    filaments.select_preset(1000);
    o.emit(vec![s("select_out_of_range"), idx(filaments.get_selected_idx())]);
    filaments.discard_current_changes();
    dump_coll(o, "deleted", &filaments);
    dump_files(o, "after_delete");

    // Copies and merges.
    {
        let mut other = PresetCollection::new(Preset::TYPE_FILAMENT, Preset::filament_options(), full_defaults(), "Default Filament").unwrap();
        {
            let a = other.load_preset_move("<sys>/m.json", "Merged A", make_cfg(&[("filament_type", "\"PVA\"")]), false, Semver::new()).unwrap();
            a.vendor = Some(bbl.clone());
            a.is_system = true;
        }
        other.load_preset_move("<sys>/m.json", "Generic ABS", make_cfg(&[("filament_type", "\"ABS\"")]), false, Semver::new()).unwrap();
        {
            let e = other.load_preset_move("<sys>/m.json", "Ext", make_cfg(&[("filament_type", "\"ABS\"")]), false, Semver::new()).unwrap();
            e.is_external = true;
        }
        let newv: VendorMap = vendors.iter().map(|(k, v)| (k.clone(), Arc::new((**v).clone()))).collect();
        let dup = filaments.merge_presets(other, &newv);
        list(o, "merge", &dup);
        {
            let m = filaments.find_preset("Merged A", false).unwrap();
            o.emit(vec![s("merged_vendor"), b(Arc::ptr_eq(m.vendor.as_ref().unwrap(), &newv["BBL"]))]);
        }
        filaments.update_vendor_ptrs_after_copy(&vendors);
        {
            let m = filaments.find_preset("Merged A", false).unwrap();
            o.emit(vec![s("merged_vendor2"), b(Arc::ptr_eq(m.vendor.as_ref().unwrap(), &vendors["BBL"]))]);
        }
        let mut copy = PresetCollection::new(Preset::TYPE_PRINT, Preset::print_options(), full_defaults(), "Default Setting").unwrap();
        copy.assign(&filaments);
        dump_coll(o, "assigned", &copy);
        o.emit(vec![s("reset")]);
        copy.reset(false);
        dump_coll(o, "reset", &copy);
    }
    dump_files(o, "end_filaments");

    // Physical printers.
    {
        let pd = format!("{}/physical", user_dir);
        write_file(
            &format!("{pd}/Office.json"),
            r#"{"version": "2.0.0.0", "preset_names": ["Bambu Lab X1 Carbon 0.4 nozzle", "My X1C"], "print_host": "192.168.0.2", "host_type": "octoprint"}"#,
        );
        write_file(
            &format!("{pd}/lab.json"),
            r#"{"version": "2.0.0.0", "preset_name": "Bambu Lab A1 0.4 nozzle", "preset_names": ["Bambu Lab A1 0.4 nozzle"]}"#,
        );
        write_file(&format!("{pd}/Klipper.json"), r#"{"version": "2.0.0.0", "preset_names": ["Klipper Box"]}"#);
        write_file(&format!("{pd}/Empty.json"), r#"{"version": "2.0.0.0"}"#);
        let mut pp = PhysicalPrinterCollection::new(PhysicalPrinter::printer_options());
        let dc = pp.default_config().clone();
        dump_cfg(o, "physical_default", &dc);
        let mut subs: PresetsConfigSubstitutions = Vec::new();
        match pp.load_printers(&user_dir, "physical", &mut subs, ForwardCompatibilitySubstitutionRule::Enable) {
            Ok(()) => o.emit(vec![s("load_printers"), s("ok")]),
            Err(e) => o.emit(vec![s("load_printers"), bang(&e)]),
        }
        let dump_pp = |o: &mut Out, pp: &PhysicalPrinterCollection, tag: &str| {
            let path = o.rel(&pp.path_from_name("y"));
            o.emit(vec![
                s("pp"),
                s(tag),
                num(pp.operator_call().len() as i64),
                idx(pp.get_selected_idx()),
                hex(&pp.get_selected_printer_name()),
                hex(&pp.get_selected_full_printer_name()),
                hex(&pp.get_selected_printer_preset_name()),
                b(pp.has_selection()),
                num(pp.get_selected_printer_technology().to_int() as i64),
                hex(&path),
                b(pp.empty()),
            ]);
            for p in pp.operator_call() {
                let file = o.rel(&p.file);
                let mut l = vec![s("ppp"), s(tag), hex(&p.name), hex(&file), b(p.loaded)];
                let names: Vec<String> = p.preset_names.iter().cloned().collect();
                push_list(&mut l, &names);
                l.push(cfg_hash(&p.config));
                l.push(hex(&p.get_full_name("Z")));
                o.emit(l);
            }
        };
        dump_pp(o, &pp, "loaded");
        for n in ["Office", "office", "KLIPPER", "lab", "Nope", "Empty"] {
            let p1 = pp.find_printer(n, true).map(|p| p.name.clone()).unwrap_or_else(|| s("<null>"));
            let p2 = pp.find_printer(n, false).map(|p| p.name.clone()).unwrap_or_else(|| s("<null>"));
            o.emit(vec![s("find_printer"), hex(n), hex(&p1), hex(&p2)]);
        }
        pp.select_printer("Office * My X1C (modified)");
        dump_pp(o, &pp, "select1");
        pp.select_printer("lab");
        dump_pp(o, &pp, "select2");
        pp.select_printer_with_preset("Office", "Bambu Lab X1 Carbon 0.4 nozzle");
        o.emit(vec![
            s("is_selected"),
            b(pp.is_selected(pp.begin() + pp.get_selected_idx(), "Bambu Lab X1 Carbon 0.4 nozzle")),
            b(pp.is_selected(pp.begin(), "x")),
        ]);
        pp.select_printer("Zzz");
        dump_pp(o, &pp, "select3");
        {
            let mut l = vec![s("with_preset")];
            push_list(&mut l, &pp.get_printers_with_preset("My X1C"));
            push_list(&mut l, &pp.get_printers_with_only_preset("Klipper Box"));
            push_list(&mut l, &pp.get_printers_with_only_preset("My X1C"));
            o.emit(l);
        }
        let mut np = PhysicalPrinter::with_preset("New One", pp.default_config(), printers.find_preset("Klipper Box", false).unwrap()).unwrap();
        np.add_preset("My X1C");
        let r1 = np.add_preset("My X1C");
        let r2 = np.delete_preset("Nope");
        let r3 = np.delete_preset("My X1C");
        o.emit(vec![s("add_preset"), b(r1), b(r2), b(r3)]);
        pp.save_printer(&mut np, "").unwrap();
        dump_pp(o, &pp, "saved_new");
        let mut renamed = pp.find_printer("Office", true).unwrap().clone();
        renamed.set_name("Office 2");
        pp.save_printer(&mut renamed, "Office").unwrap();
        dump_pp(o, &pp, "renamed");
        o.emit(vec![s("renamed_config"), b(renamed.config.keys().is_empty())]);
        let mut c = pp.default_config().clone();
        c.set_deserialize_strict("preset_names", "\"My X1C\"", false).unwrap();
        pp.load_printer(&format!("{pd}/Loaded.json"), "Loaded", c, true, true).unwrap();
        dump_pp(o, &pp, "load_printer");
        pp.delete_preset_from_printers("My X1C").unwrap();
        dump_pp(o, &pp, "deleted_preset");
        let r1 = pp.delete_printer("New One");
        let r2 = pp.delete_printer("zzz");
        o.emit(vec![s("delete_printer"), b(r1), b(r2)]);
        pp.select_printer("Office 2");
        let r1 = pp.delete_selected_printer();
        let r2 = pp.delete_selected_printer();
        o.emit(vec![s("delete_selected"), b(r1), b(r2)]);
        dump_pp(o, &pp, "end");
        o.emit(vec![
            s("names"),
            hex(&PhysicalPrinter::get_short_name("A * B")),
            hex(&PhysicalPrinter::get_short_name(" * B")),
            hex(&PhysicalPrinter::get_short_name("AB")),
            hex(&PhysicalPrinter::get_preset_name("A * B (modified)")),
            hex(&PhysicalPrinter::get_preset_name("AB")),
            hex(&PhysicalPrinter::get_preset_name("A")),
            hex(&PhysicalPrinter::separator()),
        ]);
        dump_files(o, "end_physical");
    }
}

/// FIDELITY-NOTE: PlaceholderParser::evaluate_boolean_expression is not ported (see
/// `set_boolean_expression_evaluator`). The scenario's two compatibility conditions have the form
/// `key[index]==number`; this evaluates that form as the C++ grammar does (the extra config of
/// is_compatible_with_printer first, then the printer's config), so that the conditions reach the
/// collection the way the C++ results do.
fn scenario_evaluator(templ: &str, config: &DynamicConfig, config_override: Option<&DynamicConfig>) -> Result<bool, String> {
    let (lhs, rhs) = templ.split_once("==").ok_or("unsupported expression")?;
    let (key, rest) = lhs.split_once('[').ok_or("unsupported expression")?;
    let index: usize = rest.trim_end_matches(']').parse().map_err(|_| "bad index".to_string())?;
    let value: f64 = rhs.parse().map_err(|_| "bad number".to_string())?;
    let opt = config_override.and_then(|o| o.option(key)).or_else(|| config.option(key)).ok_or("unknown key")?;
    let values = opt.vector_values_any().and_then(|v| v.downcast_ref::<Vec<f64>>()).ok_or("not a vector of numbers")?;
    Ok(values.get(index).copied().ok_or("index out of range")? == value)
}

#[test]
fn preset_matches_cpp() {
    set_boolean_expression_evaluator(Some(scenario_evaluator));
    let expected: Vec<String> = read_vectors().lines().map(|l| l.to_string()).collect();

    let d = std::env::temp_dir().join(format!("preset_native_parity_{}", std::process::id()));
    let d = d.to_string_lossy().into_owned();
    let _ = std::fs::remove_dir_all(&d);
    std::fs::create_dir_all(&d).unwrap();
    let mut o = Out { d: d.clone(), lines: Vec::new(), files: BTreeMap::new() };
    // As gen.cpp: a preset without a file writes ".info" to the current directory (Preset::save_info).
    // This test binary has one test, so changing the directory of the process is safe.
    let cwd = std::env::current_dir().unwrap();
    std::env::set_current_dir(&d).unwrap();
    gen_keys(&mut o);
    gen_filament_list_less(&mut o);
    gen_ini(&mut o);
    gen_json(&mut o);
    gen_extend(&mut o);
    gen_preset_statics(&mut o);
    gen_vendor(&mut o);
    gen_collections(&mut o);
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
