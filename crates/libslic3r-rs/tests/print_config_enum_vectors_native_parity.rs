//! Native parity of the enum vectors (`ConfigOptionEnumsGenericTempl`, Config.hpp:2128-2227) of the
//! nine options a two-nozzle job sets by name, against the pin: `tests/data/print_config/gen.cpp`
//! `gen_enum_vectors` writes `enum_vectors.txt`.
//!
//! - `ConfigBase::set_deserialize` (Config.cpp:636-719) on a fresh `DynamicPrintConfig`: the option
//!   is made by `ConfigOptionDef::create_default_option` (Config.cpp:296-325), which hands it the
//!   definition's `enum_keys_map`; names, spaces around them (`boost::trim`), a name the map does
//!   not know (refused, or the default substituted under `EnableSilent`), `nil` (nullable only),
//!   the empty text and `append`.
//! - `DynamicPrintConfig::full_print_config()` (PrintConfig.cpp:8667-8670) clones the options of
//!   `FullPrintConfig::defaults()`, whose values come by `set()` only (PrintConfig.hpp:880-897,
//!   Config.hpp:2155-2160): their `keys_map` is null in the pin as in the port, so neither can read a
//!   name into such a clone (the pin dereferences null). A configuration is loaded into a fresh
//!   `DynamicPrintConfig` (`load_from_json`), as the port's live `full_print_config_from_settings`
//!   does.

use slicer::config::*;
use slicer::print_config::config_def::*;

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

fn unhex(s: &str) -> String {
    if s == "-" {
        return String::new();
    }
    let b: Vec<u8> = (0..s.len()).step_by(2).map(|i| u8::from_str_radix(&s[i..i + 2], 16).expect("hex")).collect();
    String::from_utf8(b).expect("utf8")
}

/// `keys_map` of an enum vector option (`Some(None)`: none; `None`: not an enum vector).
fn keys_map_of(o: &dyn ConfigOption) -> Option<Option<&'static t_config_enum_values>> {
    if let Some(a) = o.as_any().downcast_ref::<ConfigOptionEnumsGeneric>() {
        return Some(a.keys_map);
    }
    if let Some(b) = o.as_any().downcast_ref::<ConfigOptionEnumsGenericNullable>() {
        return Some(b.keys_map);
    }
    None
}

fn values_of(o: &dyn ConfigOption) -> Vec<i32> {
    if let Some(a) = o.as_any().downcast_ref::<ConfigOptionEnumsGeneric>() {
        return a.values.clone();
    }
    if let Some(b) = o.as_any().downcast_ref::<ConfigOptionEnumsGenericNullable>() {
        return b.values.clone();
    }
    panic!("not an enum vector")
}

#[test]
fn enum_vectors_deserialize_by_name_as_the_pin() {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/print_config/enum_vectors.txt");
    let text = std::fs::read_to_string(path).expect("enum_vectors.txt");
    let (mut n_km, mut n_ev) = (0, 0);
    for line in text.lines() {
        let t: Vec<&str> = line.split(' ').collect();
        match t[0] {
            "km" => {
                let key = unhex(t[1]);
                let def = print_config_def().get(&key).expect("def");
                let mut fresh = DynamicPrintConfig::new();
                let created = fresh.option_mut(&key, true).expect("create").expect("option");
                let km = keys_map_of(&*created).expect("enum vector");
                let created_is_def = match (km, def.enum_keys_map) {
                    (Some(a), Some(b)) => std::ptr::eq(a, b),
                    (None, None) => true,
                    _ => false,
                };
                let full = DynamicPrintConfig::full_print_config();
                let full_null = keys_map_of(full.option(&key).expect("full option")).expect("enum vector").is_none();
                let got = format!("km {} {} {}", t[1], created_is_def as i32, full_null as i32);
                assert_eq!(got, line, "{key}");
                n_km += 1;
            }
            "ev" => {
                let key = unhex(t[1]);
                let first = unhex(t[2]);
                let value = unhex(t[4]);
                let append = t[5] == "1";
                let rule = if t[6] == "0" {
                    ForwardCompatibilitySubstitutionRule::Disable
                } else {
                    ForwardCompatibilitySubstitutionRule::EnableSilent
                };
                let mut cfg = DynamicPrintConfig::new();
                let first_result = if first.is_empty() {
                    "-".to_string()
                } else {
                    match cfg.set_deserialize_strict(&key, &first, false) {
                        Ok(()) => "ok".to_string(),
                        Err(e) => format!("!{}", hex(e.what())),
                    }
                };
                let mut ctxt = ConfigSubstitutionContext::new(rule);
                let mut l: Vec<String> =
                    vec!["ev".into(), t[1].into(), t[2].into(), first_result, t[4].into(), t[5].into(), t[6].into(), "=>".into()];
                match cfg.set_deserialize(&key, &value, &mut ctxt, append) {
                    Ok(()) => l.push("ok".into()),
                    Err(e) => l.push(format!("!{}", hex(e.what()))),
                }
                match cfg.option(&key) {
                    None => l.push("none".into()),
                    Some(o) => {
                        let v = values_of(o);
                        l.push(v.len().to_string());
                        l.extend(v.iter().map(|x| x.to_string()));
                        match o.serialize() {
                            Ok(s) => l.push(hex(&s)),
                            Err(e) => l.push(format!("!{}", hex(e.what()))),
                        }
                    }
                }
                l.push(ctxt.substitutions.len().to_string());
                assert_eq!(l.join(" "), line, "{key} {value:?}");
                n_ev += 1;
            }
            _ => panic!("unknown line {line}"),
        }
    }
    assert_eq!((n_km, n_ev), (9, 81));
}
