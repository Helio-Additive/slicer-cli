//! Native parity of the configuration check: `DynamicPrintConfig::validate(bool under_cli)`
//! (PrintConfig.cpp:8969-9007), `validate(const FullPrintConfig&, bool under_cli)` (:10641-10916) and the
//! mixed-filament checks it calls (FilamentMixer.cpp:535-840), against the pin's own functions:
//! `tests/data/print_config_validate/gen.cpp` reads `scenarios.txt` and wrote `vectors.txt`.

use slicer::config::{ConfigBase, ConfigSubstitutionContext, ForwardCompatibilitySubstitutionRule};
use slicer::print_config::config_def::DynamicPrintConfig;

fn data(name: &str) -> String {
    let path = format!("{}/tests/data/print_config_validate/{}", env!("CARGO_MANIFEST_DIR"), name);
    std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{path}: {e}"))
}

#[test]
fn config_validate_matches_the_pin() {
    let scenarios = data("scenarios.txt");
    let mut got = String::new();
    let mut n = 0;
    let mut under_cli = false;
    let mut config = DynamicPrintConfig::new();
    for line in scenarios.lines() {
        if let Some(rest) = line.strip_prefix("CASE ") {
            config = DynamicPrintConfig::new();
            under_cli = rest == "1";
        } else if let Some(rest) = line.strip_prefix("SET ") {
            let (key, value) = rest.split_once('\t').expect("SET <key>\\t<value>");
            let mut ctxt = ConfigSubstitutionContext::new(ForwardCompatibilitySubstitutionRule::Disable);
            config.set_deserialize(key, value, &mut ctxt, false).unwrap_or_else(|e| panic!("{key}={value}: {e:?}"));
        } else if line == "END" {
            let found = config.validate(under_cli).expect("DynamicPrintConfig::validate");
            got += &format!("CASE {n}\n");
            for (key, why) in &found {
                got += &format!("E {key}\t{why}\n");
            }
            got += &format!("N {}\n", found.len());
            n += 1;
        }
    }
    let want = data("vectors.txt");
    let mut differ = 0;
    for (i, (g, w)) in got.lines().zip(want.lines()).enumerate() {
        if g != w {
            differ += 1;
            if differ <= 20 {
                eprintln!("line {}: port {:?}\n        pin  {:?}", i + 1, g, w);
            }
        }
    }
    assert_eq!(got.lines().count(), want.lines().count(), "line counts");
    assert_eq!(differ, 0, "{differ} lines differ from the pin");
    assert!(n >= 31, "{n} scenarios");
}

/// Every option of `FullPrintConfig::defaults()` with its `type()`, `nullable()`, `is_nil()` and `serialize()`
/// (Config.hpp, the ConfigOption classes), as `gen --defaults` wrote them from the pin into
/// `vectors_defaults.txt`; an enum vector's serialization is `-` there (its static default has no keys map in the
/// pin, and its serialize() would dereference it).
#[test]
fn full_print_config_defaults_serialize_as_the_pin() {
    use slicer::config::ConfigOptionResolver;
    use slicer::print_config::config_def::static_print_config_defaults;
    let defaults = static_print_config_defaults("FullPrintConfig");
    let mut keys = defaults.keys();
    keys.sort();
    let mut got = String::new();
    for key in &keys {
        let opt = defaults.option(key).expect("FullPrintConfig option");
        let ty = opt.r#type() as i32;
        let value = if ty == 16393 { "-".to_string() } else { opt.serialize().expect("serialize").replace('\n', "\u{1}") };
        got += &format!("D {}\t{}\t{}\t{}\t{}\n", key, ty, opt.nullable() as i32, opt.is_nil() as i32, value);
    }
    let want = data("vectors_defaults.txt");
    let got_lines: std::collections::BTreeSet<&str> = got.lines().collect();
    let want_lines: std::collections::BTreeSet<&str> = want.lines().collect();
    let mut differ = 0;
    for w in want_lines.difference(&got_lines) {
        differ += 1;
        if differ <= 20 {
            eprintln!("pin only : {w:?}");
        }
    }
    for g in got_lines.difference(&want_lines) {
        differ += 1;
        if differ <= 40 {
            eprintln!("port only: {g:?}");
        }
    }
    assert_eq!(differ, 0, "{differ} lines differ from the pin ({} options in the pin, {} in the port)", want_lines.len(), got_lines.len());
}
