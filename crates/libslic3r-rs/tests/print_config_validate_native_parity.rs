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

/// The typed fields of the auto circle compensation (PrintConfig.hpp:1109-1110, 1747-1758; R950): their
/// defaults serialize as the pin's FullPrintConfig defaults (`vectors_defaults.txt`, written by the pin), and
/// the typed `set_deserialize` / `apply_key_value` read a value as the configuration option of the same key
/// does (this crate's DynamicPrintConfig, whose option classes the two tests above verify against the pin).
#[test]
fn typed_circle_compensation_fields_as_the_pin() {
    use slicer::config::{ConfigOption, ConfigOptionBool, ConfigOptionFloat, ConfigOptionFloats, ConfigOptionResolver};
    use slicer::print_config::{PrintConfig, PrintObjectConfig, CIRCLE_COMPENSATION_FLOATS_KEYS};
    let want = data("vectors_defaults.txt");
    let pin_default = |key: &str| -> String {
        let head = format!("D {key}\t");
        want.lines()
            .find(|l| l.starts_with(&head))
            .map(|l| l.rsplit('\t').next().unwrap().to_string())
            .unwrap_or_else(|| panic!("{key} is not among the pin's defaults"))
    };
    let floats = |v: Vec<f64>| ConfigOptionFloats::from_values(v).serialize().unwrap();
    let pc = PrintConfig::default();
    for key in CIRCLE_COMPENSATION_FLOATS_KEYS {
        assert_eq!(floats(pc.circle_compensation_floats(key)), pin_default(key), "default of {key}");
    }
    let oc = PrintObjectConfig::default();
    assert_eq!(ConfigOptionBool { value: oc.enable_circle_compensation }.serialize().unwrap(), pin_default("enable_circle_compensation"));
    assert_eq!(
        ConfigOptionFloat { value: oc.circle_compensation_manual_offset }.serialize().unwrap(),
        pin_default("circle_compensation_manual_offset")
    );

    let dynamic = |key: &str, value: &str| -> String {
        let mut config = DynamicPrintConfig::new();
        let mut ctxt = ConfigSubstitutionContext::new(ForwardCompatibilitySubstitutionRule::Disable);
        config.set_deserialize(key, value, &mut ctxt, false).unwrap_or_else(|e| panic!("{key}={value}: {e:?}"));
        config.option(key).expect("option").serialize().unwrap()
    };
    let mut n = 0;
    for value in ["0.1,0.2,-0.3", "200", "1e-3", " 0.5 , 7", "0.025,0.025,0.025,0.025"] {
        for key in CIRCLE_COMPENSATION_FLOATS_KEYS {
            let mut a = PrintConfig::default();
            assert!(a.set_deserialize(key, value));
            let mut b = PrintConfig::default();
            assert!(b.apply_key_value(key, value));
            assert_eq!(floats(a.circle_compensation_floats(key)), dynamic(key, value), "{key}={value:?}");
            assert_eq!(a.circle_compensation_floats(key), b.circle_compensation_floats(key), "{key}={value:?}");
            n += 1;
        }
    }
    for value in ["1", "0"] {
        let mut a = PrintObjectConfig::default();
        assert!(a.set_deserialize("enable_circle_compensation", value));
        assert_eq!(ConfigOptionBool { value: a.enable_circle_compensation }.serialize().unwrap(), dynamic("enable_circle_compensation", value));
        let mut b = PrintObjectConfig::default();
        assert!(b.apply_key_value("enable_circle_compensation", value));
        assert_eq!(a.enable_circle_compensation, b.enable_circle_compensation);
        n += 1;
    }
    for value in ["0.05", "-0.1", "0", "1e-2"] {
        let mut a = PrintObjectConfig::default();
        assert!(a.set_deserialize("circle_compensation_manual_offset", value));
        assert_eq!(
            ConfigOptionFloat { value: a.circle_compensation_manual_offset }.serialize().unwrap(),
            dynamic("circle_compensation_manual_offset", value)
        );
        let mut b = PrintObjectConfig::default();
        assert!(b.apply_key_value("circle_compensation_manual_offset", value));
        assert_eq!(a.circle_compensation_manual_offset.to_bits(), b.circle_compensation_manual_offset.to_bits());
        n += 1;
    }
    assert_eq!(n, 66);
}

/// `type()` of the twenty option classes under its C++ name (`type_`, Config.hpp:829-1897; R950): equal to the
/// trait's `r#type`, which `full_print_config_defaults_serialize_as_the_pin` compares with the pin's `type()` over
/// every option of FullPrintConfig; the codes of the classes that FullPrintConfig has are looked up in the pin's
/// vectors.
#[test]
fn option_type_under_the_cpp_name() {
    use slicer::config::*;
    let want = data("vectors_defaults.txt");
    let pin_types: std::collections::BTreeSet<i32> =
        want.lines().filter_map(|l| l.split('\t').nth(1)).filter_map(|t| t.parse().ok()).collect();
    let mut got: Vec<(i32, i32)> = Vec::new();
    macro_rules! check {
        ($x:expr) => {{
            let x = $x;
            got.push((x.type_() as i32, ConfigOption::r#type(&x) as i32));
        }};
    }
    check!(ConfigOptionBool::new());
    check!(ConfigOptionBools::new());
    check!(ConfigOptionEnum::<PrinterTechnology>::new());
    check!(ConfigOptionEnumGeneric::new(None));
    check!(ConfigOptionEnumsGeneric::new(None));
    check!(ConfigOptionFloat::new());
    check!(ConfigOptionFloatOrPercent::new());
    check!(ConfigOptionFloatsOrPercents::new());
    check!(ConfigOptionFloats::new());
    check!(ConfigOptionInt::new());
    check!(ConfigOptionIntsGroups::new());
    check!(ConfigOptionInts::new());
    check!(ConfigOptionPercent::new());
    check!(ConfigOptionPercents::new());
    check!(ConfigOptionPoint::new());
    check!(ConfigOptionPoint3::new());
    check!(ConfigOptionPoints::new());
    check!(ConfigOptionPointsGroups::new());
    check!(ConfigOptionString::new());
    check!(ConfigOptionStrings::new());
    assert_eq!(got.len(), 20);
    let mut in_pin = 0;
    for (a, b) in &got {
        assert_eq!(a, b);
        in_pin += pin_types.contains(a) as usize;
    }
    // FullPrintConfig has an option of every class but ConfigOptionPoint3 (coPoint3) and ConfigOptionIntsGroups
    // (coIntsGroups).
    assert_eq!(in_pin, 18, "type codes among the pin's FullPrintConfig types {pin_types:?}");
}
