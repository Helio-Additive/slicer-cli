//! `slicer::placeholder_parser` (the port of libslic3r/PlaceholderParser.hpp and .cpp) against the
//! C++ at the pin.
//!
//! `tests/data/placeholder_parser/placeholder_parser.txt.gz` is the output of `gen.cpp` beside it
//! (gzip -9n), which links the pin's PlaceholderParser.cpp. It describes three environments (a
//! synthetic configuration with options of every type, and the X1 Carbon and H2D profiles of
//! resources/profiles/BBL with the placeholders GCode.cpp sets), then runs templates through
//! `PlaceholderParser::process` and `evaluate_boolean_expression`: the custom G-code of every BBL
//! profile, hand-written templates over the grammar (operators, type mixes, vector indexing,
//! conditionals, regular expressions, functions, numbers, parse and evaluation errors), and every
//! binary operator over every pair of a set of operands. The header of gen.cpp describes the lines.
//! The Rust has to give the same text byte for byte, or an exception of the same class with the
//! same `what()`; the PlaceholderParser methods (`set`, `apply_config`, `config_diff`, ...) have to
//! leave the same configuration.

use slicer::config::*;
use slicer::placeholder_parser::*;
use slicer::print_config::config_def::DynamicPrintConfig;
use std::collections::BTreeMap;
use std::io::Read;

fn read_vectors() -> String {
    let path = format!("{}/tests/data/placeholder_parser/placeholder_parser.txt.gz", env!("CARGO_MANIFEST_DIR"));
    let file = std::fs::File::open(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
    let mut text = String::new();
    flate2::read::GzDecoder::new(file).read_to_string(&mut text).expect("gzip");
    text
}

fn hex(s: &[u8]) -> String {
    if s.is_empty() {
        return "-".to_string();
    }
    let mut o = String::with_capacity(s.len() * 2);
    for c in s {
        o.push_str(&format!("{:02x}", c));
    }
    o
}

fn unhex(t: &str) -> String {
    if t == "-" {
        return String::new();
    }
    let b: Vec<u8> = (0..t.len() / 2).map(|i| u8::from_str_radix(&t[2 * i..2 * i + 2], 16).expect("hex")).collect();
    String::from_utf8(b).expect("the vectors are UTF-8")
}

/// gen.cpp `make_opt`
fn make_opt(ty: &str) -> Box<dyn ConfigOption> {
    match ty {
        "i" => Box::new(ConfigOptionInt::new()),
        "f" => Box::new(ConfigOptionFloat::new()),
        "s" => Box::new(ConfigOptionString::new()),
        "b" => Box::new(ConfigOptionBool::new()),
        "p" => Box::new(ConfigOptionPercent::new()),
        "fp" => Box::new(ConfigOptionFloatOrPercent::new()),
        "pt" => Box::new(ConfigOptionPoint::new()),
        "is" => Box::new(ConfigOptionInts::new()),
        "isn" => Box::new(ConfigOptionIntsNullable::new()),
        "fs" => Box::new(ConfigOptionFloats::new()),
        "fsn" => Box::new(ConfigOptionFloatsNullable::new()),
        "ss" => Box::new(ConfigOptionStrings::new()),
        "bs" => Box::new(ConfigOptionBools::new()),
        "ps" => Box::new(ConfigOptionPercents::new()),
        "fps" => Box::new(ConfigOptionFloatsOrPercents::new()),
        "pts" => Box::new(ConfigOptionPoints::new()),
        _ => panic!("bad type {ty}"),
    }
}

fn typed_opt(ty: &str, text: &str) -> Box<dyn ConfigOption> {
    let mut opt = make_opt(ty);
    opt.deserialize(text, false).expect("deserialize");
    opt
}

struct Env {
    ext: DynamicPrintConfig,
    ppv: Vec<(String, String, String)>,
    ovr: DynamicConfig,
}

fn class_of(e: &PlaceholderParserException) -> &'static str {
    match e {
        PlaceholderParserException::PlaceholderParserError(_) => "PPE",
        PlaceholderParserException::FlowErrorMissingVariable(_) => "FEMV",
        PlaceholderParserException::RuntimeError(_) => "RE",
        PlaceholderParserException::ConfigurationError(_) => "CE",
    }
}

const CLOCK_KEYS: [&str; 7] = ["timestamp", "year", "month", "day", "hour", "minute", "second"];

/// gen.cpp `dump_config`
fn dump_config(name: &str, c: &DynamicConfig) -> String {
    let keys = ConfigBase::keys(c);
    let mut s = format!("api {} {}", name, keys.len());
    for k in &keys {
        let opt = c.option(k).unwrap();
        if CLOCK_KEYS.contains(&k.as_str()) {
            s.push_str(&format!(" {} {} CLOCK", hex(k.as_bytes()), opt.r#type() as i32));
            continue;
        }
        s.push_str(&format!(" {} {} {}", hex(k.as_bytes()), opt.r#type() as i32, hex(opt.serialize().unwrap().as_bytes())));
    }
    s
}

/// gen.cpp `dump_keys`
fn dump_keys(name: &str, keys: &[String]) -> String {
    let mut s = format!("api {} {}", name, keys.len());
    for k in keys {
        s.push_str(&format!(" {}", hex(k.as_bytes())));
    }
    s
}

fn opt_text(o: Option<&dyn ConfigOption>) -> String {
    match o {
        Some(o) => hex(o.serialize().unwrap().as_bytes()),
        None => "null".to_string(),
    }
}

/// gen.cpp `gen_api`
fn gen_api() -> Vec<String> {
    let mut out = Vec::new();
    let mut pp = PlaceholderParser::new(None);
    out.push(dump_config("ctor", pp.config()));
    out.push(format!("api external {}", if pp.external_config().is_none() { 0 } else { 1 }));
    pp.clear_config();
    out.push(dump_config("clear", pp.config()));
    let mut a = DynamicPrintConfig::new();
    a.set_deserialize_strict("layer_height", "0.2", false).unwrap();
    a.set_deserialize_strict("wall_loops", "3", false).unwrap();
    a.set_deserialize_strict("filament_type", "PLA;PETG", false).unwrap();
    a.set_deserialize_strict("nozzle_diameter", "0.4,0.6", false).unwrap();
    out.push(dump_keys("diff1", &pp.config_diff(&a).unwrap()));
    out.push(format!("api apply1 {}", pp.apply_config(&a).unwrap() as i32));
    out.push(dump_config("after_apply1", pp.config()));
    out.push(dump_keys("diff2", &pp.config_diff(&a).unwrap()));
    out.push(format!("api apply2 {}", pp.apply_config(&a).unwrap() as i32));
    let mut b = DynamicPrintConfig::new();
    for k in ConfigBase::keys(&a) {
        b.set_key_value(&k, ConfigOption::clone(a.option(&k).unwrap()));
    }
    b.set_deserialize_strict("layer_height", "0.3", false).unwrap();
    b.set_deserialize_strict("wall_loops", "4", false).unwrap();
    b.set_deserialize_strict("top_shell_layers", "5", false).unwrap();
    out.push(dump_keys("diff3", &pp.config_diff(&b).unwrap()));
    pp.apply_only(&b, &["layer_height".to_string(), "top_shell_layers".to_string()]);
    out.push(dump_config("after_apply_only", pp.config()));
    pp.set("s", "str".to_string());
    pp.set("i", 5i32);
    pp.set("u", 7u32);
    pp.set("bo", true);
    pp.set("d", 2.5f64);
    pp.set("v", vec!["x".to_string(), "y z".to_string()]);
    pp.set("o", Box::new(ConfigOptionPercent::with_value(12.5)) as Box<dyn ConfigOption>);
    out.push(dump_config("after_set", pp.config()));
    let mut c = DynamicPrintConfig::new();
    c.set_deserialize_strict("z_hop", "0.4,0.6", false).unwrap();
    c.set_deserialize_strict("wall_loops", "9", false).unwrap();
    pp.apply_config_move(c);
    out.push(dump_config("after_apply_move", pp.config()));
    out.push(format!("api option_i {}", opt_text(pp.option("i"))));
    out.push(format!("api option_nope {}", opt_text(pp.option("nope"))));
    pp.update_timestamp();
    out.push(dump_config("after_timestamp", pp.config()));
    let ts = pp.config().option("timestamp").and_then(|o| o.as_any().downcast_ref::<ConfigOptionString>()).map(|o| o.value.len());
    out.push(format!("api timestamp_len {}", ts.unwrap_or(0)));
    let mut tc = DynamicConfig::new();
    PlaceholderParser::update_timestamp_of(&mut tc);
    out.push(dump_config("static_timestamp", &tc));
    out.push(format!("api env {}", opt_text(pp.option("SLIC3R_PP_TEST"))));
    let pp2 = PlaceholderParser::new(None);
    out.push(format!("api env2 {}", opt_text(pp2.option("SLIC3R_PP_TEST"))));
    out.push(format!("api env_empty {}", opt_text(pp2.option("SLIC3R_PP_EMPTY"))));
    out
}

#[test]
fn placeholder_parser_matches_cpp() {
    // gen.cpp main(): the SLIC3R_* variables apply_env_variables() reads.
    std::env::set_var("SLIC3R_PP_TEST", "  hello world");
    std::env::set_var("SLIC3R_PP_EMPTY", "");

    let text = read_vectors();
    let lines: Vec<&str> = text.lines().collect();

    // The environments.
    let mut envs: BTreeMap<String, Env> = BTreeMap::new();
    let mut current: Option<String> = None;
    for l in &lines {
        let t: Vec<&str> = l.split(' ').collect();
        match t[0] {
            "env" => {
                current = Some(t[1].to_string());
                envs.insert(t[1].to_string(), Env { ext: DynamicPrintConfig::new(), ppv: Vec::new(), ovr: DynamicConfig::new() });
            }
            "ext" => {
                let e = envs.get_mut(current.as_ref().unwrap()).unwrap();
                e.ext.set_deserialize_strict(t[1], &unhex(t[2]), false).unwrap_or_else(|err| panic!("ext {}: {}", t[1], err.what()));
            }
            "ppv" => {
                let e = envs.get_mut(current.as_ref().unwrap()).unwrap();
                e.ppv.push((t[1].to_string(), t[2].to_string(), unhex(t[3])));
            }
            "ovr" => {
                let e = envs.get_mut(current.as_ref().unwrap()).unwrap();
                e.ovr.set_key_value(t[1], typed_opt(t[2], &unhex(t[3])));
            }
            "end" => current = None,
            _ => {}
        }
    }
    let mut parsers: BTreeMap<String, PlaceholderParser> = BTreeMap::new();
    for (name, e) in &envs {
        let mut pp = PlaceholderParser::new(Some(&e.ext.base));
        for (k, ty, text) in &e.ppv {
            pp.set(k, typed_opt(ty, text));
        }
        parsers.insert(name.clone(), pp);
    }

    let api = gen_api();
    let mut api_i = 0usize;
    let mut bad = 0usize;
    let mut cases = 0usize;
    let mut per_kind: BTreeMap<String, (usize, usize)> = BTreeMap::new();
    for l in &lines {
        let t: Vec<&str> = l.split(' ').collect();
        let got = match t[0] {
            "api" => {
                let g = api.get(api_i).cloned().unwrap_or_default();
                api_i += 1;
                g
            }
            "case" => {
                cases += 1;
                let env_name = t[1];
                let mode = t[2];
                let extruder: u32 = t[3].parse().unwrap();
                let use_override = t[4] == "1";
                let templ = unhex(t[5]);
                let e = &envs[env_name];
                let pp = &parsers[env_name];
                let ovr = if use_override { Some(&e.ovr) } else { None };
                let r = match mode {
                    "B" => PlaceholderParser::evaluate_boolean_expression(&templ, &e.ext.base, ovr)
                        .map(|b| if b { "true".to_string() } else { "false".to_string() }),
                    "R" => {
                        let mut cd = ContextData::default();
                        pp.process(&templ, extruder, ovr, Some(&mut cd))
                    }
                    _ => pp.process(&templ, extruder, ovr, None),
                };
                let (cls, txt) = match r {
                    Ok(s) => ("ok", s),
                    Err(e) => (class_of(&e), e.what().to_string()),
                };
                let g = format!("case {} {} {} {} {} {} {}", env_name, mode, t[3], t[4], t[5], cls, hex(txt.as_bytes()));
                let k = per_kind.entry(format!("{} {}", env_name, mode)).or_default();
                k.0 += 1;
                if g != *l {
                    k.1 += 1;
                }
                g
            }
            _ => continue,
        };
        if got != *l {
            bad += 1;
            if let Some(path) = std::env::var_os("PP_PARITY_DUMP") {
                use std::io::Write;
                let mut f = std::fs::OpenOptions::new().create(true).append(true).open(path).unwrap();
                writeln!(f, "{l}\n{got}").unwrap();
            }
            if bad <= 25 {
                let show = |s: &str| -> String {
                    let t: Vec<&str> = s.split(' ').collect();
                    if t[0] == "case" && t.len() >= 8 {
                        format!("{} {} | {:?} | {} {:?}", t[1], t[2], unhex(t[5]), t[6], unhex(t[7]))
                    } else {
                        s.chars().take(400).collect()
                    }
                };
                eprintln!("C++:  {}\nRust: {}\n", show(l), show(&got));
            }
        }
    }
    for (k, (n, b)) in &per_kind {
        eprintln!("{k}: {n} cases, {b} differ");
    }
    assert!(cases > 30000, "{cases} cases");
    assert_eq!(api_i, api.len(), "api lines");
    assert_eq!(bad, 0, "{bad} lines differ");
}
