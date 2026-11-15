//! `support_recommended_params` against SupportRecommendedParams.cpp at the pin.
//!
//! `tests/data/support_recommended_params/vectors.txt` and `pin_table.txt` are the output of
//! `gen.cpp` beside them, which compiles the C++ file itself with nlohmann::json and Boost (the
//! header comment there says how, and describes the lines). `vectors.txt` was made from the table
//! under `tests/data/support_recommended_params/resources`, written to go through every branch of
//! the loader and of the lookup; `pin_table.txt` from the table the pin ships.
//!
//! Compared exactly: the loaded table (every key, entry and parameter; doubles by their bits),
//! `support_recommended_material_name`, and `evaluate_support_material_combination`.

use std::path::{Path, PathBuf};

use slicer::support_recommended_params::*;

const VECTORS: &str = include_str!("data/support_recommended_params/vectors.txt");
const PIN_TABLE: &str = include_str!("data/support_recommended_params/pin_table.txt");
const TABLE_FILE: &str = "profiles/BBL/filament/support_recommended_params.json";

fn test_resources() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/data/support_recommended_params/resources")
}

struct Tok<'a>(std::str::SplitWhitespace<'a>);

impl<'a> Tok<'a> {
    fn raw(&mut self) -> &'a str {
        self.0.next().expect("token")
    }
    fn tag(&mut self, tag: &str) {
        assert_eq!(self.raw(), tag);
    }
    fn u(&mut self) -> usize {
        self.raw().parse().expect("unsigned")
    }
    fn i(&mut self) -> i32 {
        self.raw().parse().expect("integer")
    }
    fn b(&mut self) -> bool {
        self.u() != 0
    }
    fn d(&mut self) -> f64 {
        f64::from_bits(u64::from_str_radix(self.raw(), 16).expect("double bits"))
    }
    /// A string written as the hex of its bytes, "-" when empty.
    fn s(&mut self) -> String {
        let hex = self.raw();
        if hex == "-" {
            return String::new();
        }
        let bytes: Vec<u8> =
            (0..hex.len() / 2).map(|i| u8::from_str_radix(&hex[2 * i..2 * i + 2], 16).expect("hex")).collect();
        String::from_utf8(bytes).expect("utf-8")
    }
    fn strings(&mut self) -> Vec<String> {
        let n = self.u();
        (0..n).map(|_| self.s()).collect()
    }
}

/// The table as the dump of gen.cpp describes it, with every double as its bits.
fn table_text(table: &SupportRecommendedParamsMap) -> Vec<String> {
    let mut out = vec![format!("KEYS {}", table.len())];
    for (key, entries) in table {
        out.push(format!("KEY {key:?} {}", entries.len()));
        for e in entries {
            let mut line = format!(
                "ENTRY {:?} {:?} {} {:?} {:?} {:?}",
                e.model_material, e.model_material_type, e.priority, e.support_material_type, e.support_material_list, e.printer_models
            );
            for (key, value) in &e.params.params {
                line += &match value {
                    ParamValue::Double(v) => format!(" {key:?}=0:{:x}", v.to_bits()),
                    ParamValue::Bool(v) => format!(" {key:?}=1:{v}"),
                    ParamValue::String(v) => format!(" {key:?}=2:{v:?}"),
                    ParamValue::VecDouble(v) => {
                        format!(" {key:?}=3:{:?}", v.iter().map(|d| format!("{:x}", d.to_bits())).collect::<Vec<_>>())
                    }
                };
            }
            out.push(line);
        }
    }
    out
}

/// Reads the dump of gen.cpp and returns it in the form of `table_text`.
fn read_table(t: &mut Tok) -> (Vec<String>, usize) {
    t.tag("KEYS");
    let keys = t.u();
    let mut out = vec![format!("KEYS {keys}")];
    let mut entries = 0;
    for _ in 0..keys {
        t.tag("KEY");
        let key = t.s();
        let n = t.u();
        out.push(format!("KEY {key:?} {n}"));
        for _ in 0..n {
            t.tag("ENTRY");
            let (model_material, model_material_type, priority, support_material_type) = (t.s(), t.s(), t.i(), t.s());
            let (support_material_list, printer_models) = (t.strings(), t.strings());
            let mut line = format!(
                "ENTRY {model_material:?} {model_material_type:?} {priority} {support_material_type:?} {support_material_list:?} {printer_models:?}"
            );
            for _ in 0..t.u() {
                let key = t.s();
                line += &match t.u() {
                    0 => format!(" {key:?}=0:{:x}", t.d().to_bits()),
                    1 => format!(" {key:?}=1:{}", t.b()),
                    2 => format!(" {key:?}=2:{:?}", t.s()),
                    3 => {
                        let n = t.u();
                        format!(" {key:?}=3:{:?}", (0..n).map(|_| format!("{:x}", t.d().to_bits())).collect::<Vec<_>>())
                    }
                    kind => panic!("kind {kind}"),
                };
            }
            out.push(line);
            entries += 1;
        }
    }
    (out, entries)
}

#[test]
fn loaded_table() {
    let mut t = Tok(VECTORS.split_whitespace());
    let (want, entries) = read_table(&mut t);
    assert_eq!(entries, 34);
    let got = load_support_recommended_params_map(&test_resources().join(TABLE_FILE));
    assert_eq!(table_text(&got), want);
}

#[test]
fn loaded_table_of_the_pin() {
    let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../references/BambuStudio/resources").join(TABLE_FILE);
    assert!(
        path.exists(),
        "{} is missing: run `git submodule update --init references/BambuStudio`",
        path.display()
    );
    let mut t = Tok(PIN_TABLE.split_whitespace());
    let (want, entries) = read_table(&mut t);
    assert!(t.0.next().is_none());
    assert_eq!(entries, 18);
    assert_eq!(table_text(&load_support_recommended_params_map(&path)), want);
}

#[test]
fn files_that_do_not_load() {
    let dir = std::env::temp_dir().join(format!("support_recommended_params_{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    // No file: SupportRecommendedParams.cpp:44-47.
    assert!(load_support_recommended_params_map(&dir.join("missing.json")).is_empty());
    // A file that does not parse, a document that is not an object, "combinations" that is not an
    // array: SupportRecommendedParams.cpp:59, 158-160.
    for (name, text) in [
        ("broken.json", "{ \"combinations\": [ "),
        ("empty.json", ""),
        ("array.json", "[1, 2]"),
        ("not_array.json", "{ \"combinations\": { \"model_material\": \"PLA\" } }"),
        ("overflow.json", "{ \"combinations\": [], \"x\": 1e999 }"),
    ] {
        let path = dir.join(name);
        std::fs::write(&path, text).unwrap();
        assert!(load_support_recommended_params_map(&path).is_empty(), "{name}");
    }
    // A byte order mark is skipped.
    let path = dir.join("bom.json");
    std::fs::write(
        &path,
        "\u{feff}{ \"combinations\": [ { \"model_material\": \"A\", \"support_material_list\": [\"B\"], \"printer_models\": [\"C\"], \"recommended_params\": { \"k\": 1 } } ] }",
    )
    .unwrap();
    let table = load_support_recommended_params_map(&path);
    assert_eq!(table.len(), 1);
    assert_eq!(table["B|A|C"][0].params.get::<f64>("k"), Some(1.0));
    // get<T>() answers only for the alternative held.
    assert_eq!(table["B|A|C"][0].params.get::<bool>("k"), None);
    assert_eq!(table["B|A|C"][0].params.get::<f64>("missing"), None);
    std::fs::remove_dir_all(&dir).unwrap();
}

#[test]
fn material_name() {
    let mut t = Tok(VECTORS.split_whitespace());
    read_table(&mut t);
    t.tag("NAME");
    let n = t.u();
    assert_eq!(n, 300);
    let mut changed = 0;
    for _ in 0..n {
        let name = t.s();
        t.tag("->");
        let want = t.s();
        changed += usize::from(want != name);
        assert_eq!(support_recommended_material_name(name.clone()), want, "{name:?}");
    }
    assert!(changed > 100);
}

#[test]
fn evaluate_combination() {
    // The table is loaded once per process, from the resources directory: point it at the test table
    // before the first lookup. No other test of this file reads the resources directory.
    slicer::utils::set_resources_dir(test_resources().to_str().unwrap());
    load_support_recommended_params();

    let mut t = Tok(VECTORS.split_whitespace());
    read_table(&mut t);
    t.tag("NAME");
    for _ in 0..t.u() {
        t.s();
        t.tag("->");
        t.s();
    }
    t.tag("EVAL");
    let n = t.u();
    assert_eq!(n, 3000);
    let mut found = std::collections::BTreeSet::new();
    for case in 0..n {
        let mut material =
            |t: &mut Tok| SupportMaterialProperties { name: t.s(), r#type: t.s(), soluble: t.b(), support: t.b() };
        let support = material(&mut t);
        let model = material(&mut t);
        let printer = t.s();
        t.tag("->");
        let got = evaluate_support_material_combination(&support, &model, &printer);
        let id = match got.recommended_params {
            None => -1,
            Some(entry) => entry.params.get::<f64>("id").unwrap_or(-2.0) as i32,
        };
        let rule = match got.built_in_rule {
            BuiltInSupportMaterialRule::None => 0,
            BuiltInSupportMaterialRule::Soluble => 1,
            BuiltInSupportMaterialRule::DedicatedSupport => 2,
            BuiltInSupportMaterialRule::PetgForPla => 3,
            BuiltInSupportMaterialRule::PlaForPetg => 4,
            BuiltInSupportMaterialRule::PlaForTpu => 5,
        };
        assert_eq!(
            (id, rule, got.zero_top, got.zero_bottom),
            (t.i(), t.i(), t.b(), t.b()),
            "case {case}: support {support:?} model {model:?} printer {printer:?}"
        );
        found.insert(id);
    }
    assert!(t.0.next().is_none(), "tokens left over");
    assert!(found.len() >= 15, "entries found: {found:?}");
}
