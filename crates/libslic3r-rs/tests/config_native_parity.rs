//! `slicer::config` (the port of libslic3r/Config.hpp and Config.cpp) against the C++ at the pin.
//!
//! `tests/data/config/*.txt.gz` are the output of `gen.cpp` beside them (gzip -9n), which links
//! the pin's Config.cpp and drives every class of Config.hpp: the option classes one by one and
//! pair by pair, the vector operations, the string helpers, and DynamicConfig / StaticConfig over a
//! test definition. The header of gen.cpp describes the lines. Every operation is repeated here on
//! the Rust types, and what the Rust returns has to be the same: texts byte for byte, doubles bit
//! for bit, hashes equal, a thrown exception of the same class with the same `what()`.
//!
//! The vectors were made on macOS: the "-psn_" argument of read_cli, which the C++ skips there
//! only, is not compared on another system.

use slicer::config::config_helpers::{
    enum_looks_like_true_value, looks_like_enum_value, DeserializationResult, DeserializationSubstitution,
};
use slicer::config::*;
use slicer::geometry::{Vec2d, Vec3d};
use std::collections::{BTreeMap, BTreeSet};
use std::io::Read;
use std::panic::{catch_unwind, AssertUnwindSafe};
use std::sync::OnceLock;

// ---------------------------------------------------------------------------------------------
// Tokens
// ---------------------------------------------------------------------------------------------

fn read_vectors(name: &str) -> String {
    let path = format!("{}/tests/data/config/{}.txt.gz", env!("CARGO_MANIFEST_DIR"), name);
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
fn hs(s: &str) -> String {
    hex(s.as_bytes())
}
fn unhex(t: &str) -> String {
    if t == "-" {
        return String::new();
    }
    let b: Vec<u8> = (0..t.len() / 2).map(|i| u8::from_str_radix(&t[2 * i..2 * i + 2], 16).expect("hex")).collect();
    String::from_utf8(b).expect("the vectors are UTF-8")
}
fn dbl(v: f64) -> String {
    format!("{:016x}", v.to_bits())
}
fn undbl(t: &str) -> f64 {
    f64::from_bits(u64::from_str_radix(t, 16).expect("double"))
}
fn b(v: bool) -> String {
    (v as i32).to_string()
}

struct Cur<'a> {
    t: Vec<&'a str>,
    i: usize,
}
impl<'a> Cur<'a> {
    fn new(s: &'a str) -> Self {
        Cur { t: s.split(' ').collect(), i: 0 }
    }
    fn next(&mut self) -> &'a str {
        let t = self.t[self.i];
        self.i += 1;
        t
    }
    fn int(&mut self) -> i64 {
        self.next().parse().expect("int")
    }
    fn text(&mut self) -> String {
        unhex(self.next())
    }
    fn keys(&mut self) -> Vec<String> {
        let n = self.int() as usize;
        (0..n).map(|_| self.text()).collect()
    }
    fn ints(&mut self) -> Vec<i32> {
        let n = self.int() as usize;
        (0..n).map(|_| self.int() as i32).collect()
    }
}

/// `what()` of an exception, with its class (gen.cpp exc_kind()).
fn exc(out: &mut Vec<String>, e: &ConfigurationError) {
    out.push(
        match e.kind {
            ConfigurationErrorKind::UnknownOptionException => "!U",
            ConfigurationErrorKind::NoDefinitionException => "!N",
            ConfigurationErrorKind::BadOptionTypeException => "!T",
            ConfigurationErrorKind::BadOptionValueException => "!V",
            ConfigurationErrorKind::ConfigurationError => "!C",
            ConfigurationErrorKind::RuntimeError => "!R",
            ConfigurationErrorKind::IniParserError => "!X",
        }
        .to_string(),
    );
    out.push(hs(e.what()));
}

/// gen.cpp guarded(): the tokens of `f`, or the exception it threw.
fn g(out: &mut Vec<String>, f: impl FnOnce(&mut Vec<String>) -> Result<(), ConfigurationError>) {
    let mut r = Vec::new();
    if let Err(e) = f(&mut r) {
        r.clear();
        exc(&mut r, &e);
    }
    out.extend(r);
}

fn keys_tokens(out: &mut Vec<String>, keys: &[String]) {
    out.push(keys.len().to_string());
    for k in keys {
        out.push(hs(k));
    }
}

fn no_path(s: &str, path: &str) -> String {
    s.replace(path, "<FILE>")
}

// ---------------------------------------------------------------------------------------------
// The test enum and the test definition (gen.cpp)
// ---------------------------------------------------------------------------------------------

fn enum_map(pairs: &[(&str, i32)]) -> t_config_enum_values {
    pairs.iter().map(|(k, v)| (k.to_string(), *v)).collect()
}
fn test_enum_values() -> &'static t_config_enum_values {
    static M: OnceLock<t_config_enum_values> = OnceLock::new();
    M.get_or_init(|| enum_map(&[("alpha", 0), ("beta", 1), ("gamma", 2), ("beta_alias", 1)]))
}
fn rir_values() -> &'static t_config_enum_values {
    static M: OnceLock<t_config_enum_values> = OnceLock::new();
    M.get_or_init(|| enum_map(&[("Disabled", 0), ("Auto", 1), ("Enabled", 2)]))
}
fn style_values() -> &'static t_config_enum_values {
    static M: OnceLock<t_config_enum_values> = OnceLock::new();
    M.get_or_init(|| {
        enum_map(&[
            ("default", 0),
            ("grid", 1),
            ("snug", 2),
            ("tree_slim", 3),
            ("tree_strong", 4),
            ("tree_hybrid", 5),
            ("tree_organic", 6),
        ])
    })
}

/// `enum TestEnum { teAlpha, teBeta, teGamma }`: an int, as a C++ enum may hold any value.
#[derive(Clone, Copy, PartialEq, Debug)]
struct TestEnum(i32);
impl ConfigEnum for TestEnum {
    fn get_enum_names() -> &'static t_config_enum_names {
        static N: OnceLock<t_config_enum_names> = OnceLock::new();
        N.get_or_init(|| vec!["alpha".to_string(), "beta".to_string(), "gamma".to_string()])
    }
    fn get_enum_values() -> &'static t_config_enum_values {
        test_enum_values()
    }
    fn to_int(self) -> i32 {
        self.0
    }
    fn from_int(val: i32) -> Self {
        TestEnum(val)
    }
}

fn enums_generic(values: Vec<i32>) -> ConfigOptionEnumsGeneric {
    let mut o = ConfigOptionEnumsGeneric::from_values(values);
    o.keys_map = Some(test_enum_values());
    o
}
fn enums_generic_nullable(values: Vec<i32>) -> ConfigOptionEnumsGenericNullable {
    let mut o = ConfigOptionEnumsGenericNullable::from_values(values);
    o.keys_map = Some(test_enum_values());
    o
}

fn test_def() -> &'static ConfigDef {
    static D: OnceLock<ConfigDef> = OnceLock::new();
    D.get_or_init(|| {
        use ConfigOptionType::*;
        let mut def = ConfigDef::new();
        {
            let d = def.add("f", CoFloat);
            d.label = "Float".into();
            d.tooltip = "A float value. This tooltip is long enough to be wrapped by the help printer when it is printed next to the option.".into();
            d.sidetext = "mm".into();
            d.category = "Numbers".into();
            d.set_default_value(Box::new(ConfigOptionFloat::with_value(1.5)));
        }
        {
            let d = def.add("fs", CoFloats);
            d.category = "Numbers".into();
            d.tooltip = "Floats".into();
            d.set_default_value(Box::new(ConfigOptionFloats::from_values(vec![0.4])));
        }
        def.add_nullable("fsn", CoFloats)
            .set_default_value(Box::new(ConfigOptionFloatsNullable::from_values(vec![ConfigOptionFloatsNullable::nil_value()])));
        {
            let d = def.add("i", CoInt);
            d.category = "Numbers".into();
            d.tooltip = "An int".into();
            d.cli = "int-opt|I".into();
            d.cli_params = "count".into();
            d.set_default_value(Box::new(ConfigOptionInt::with_value(3)));
        }
        def.add("is", CoInts).set_default_value(Box::new(ConfigOptionInts::from_values(vec![1])));
        def.add_nullable("isn", CoInts);
        {
            let d = def.add("s", CoString);
            d.tooltip = "A string".into();
            d.cli = "string-opt|S".into();
            d.set_default_value(Box::new(ConfigOptionString::with_value("abc")));
        }
        {
            let d = def.add("s_empty", CoString);
            d.tooltip = "A string with an empty default".into();
            d.set_default_value(Box::new(ConfigOptionString::with_value("")));
        }
        def.add("ss", CoStrings).set_default_value(Box::new(ConfigOptionStrings::new()));
        def.add("p", CoPercent).set_default_value(Box::new(ConfigOptionPercent::with_value(15.0)));
        def.add("ps", CoPercents).set_default_value(Box::new(ConfigOptionPercents::from_values(vec![50.0])));
        def.add_nullable("psn", CoPercents);
        {
            let d = def.add("fp", CoFloatOrPercent);
            d.ratio_over = "f".into();
            d.set_default_value(Box::new(ConfigOptionFloatOrPercent::with_value(50.0, true)));
        }
        {
            let d = def.add("fp2", CoFloatOrPercent);
            d.ratio_over = "fs".into();
            d.set_default_value(Box::new(ConfigOptionFloatOrPercent::with_value(25.0, true)));
        }
        def.add("fp3", CoFloatOrPercent).set_default_value(Box::new(ConfigOptionFloatOrPercent::with_value(3.0, false)));
        {
            let d = def.add("fp4", CoFloatOrPercent);
            d.ratio_over = "fp".into();
            d.set_default_value(Box::new(ConfigOptionFloatOrPercent::with_value(200.0, true)));
        }
        {
            let d = def.add("fp5", CoFloatOrPercent);
            d.ratio_over = "missing_key".into();
            d.set_default_value(Box::new(ConfigOptionFloatOrPercent::with_value(10.0, true)));
        }
        {
            let d = def.add("fp6", CoFloatOrPercent);
            d.ratio_over = "fsn".into();
            d.set_default_value(Box::new(ConfigOptionFloatOrPercent::with_value(10.0, true)));
        }
        {
            let d = def.add("fps", CoFloatsOrPercents);
            d.ratio_over = "fs".into();
            d.set_default_value(Box::new(ConfigOptionFloatsOrPercents::from_values(vec![FloatOrPercent::with(10.0, false)])));
        }
        def.add_nullable("fpsn", CoFloatsOrPercents);
        def.add("fps0", CoFloatsOrPercents)
            .set_default_value(Box::new(ConfigOptionFloatsOrPercents::from_values(vec![FloatOrPercent::with(10.0, true)])));
        def.add("pt", CoPoint).set_default_value(Box::new(ConfigOptionPoint::with_value(Vec2d::new(1.0, 2.0))));
        def.add("pts", CoPoints).set_default_value(Box::new(ConfigOptionPoints::from_values(vec![Vec2d::new(0.0, 0.0)])));
        def.add("pt3", CoPoint3);
        {
            let d = def.add("b", CoBool);
            d.tooltip = "A bool".into();
            d.category = "Switches".into();
            d.set_default_value(Box::new(ConfigOptionBool::with_value(false)));
        }
        def.add("b1", CoBool).set_default_value(Box::new(ConfigOptionBool::with_value(true)));
        {
            let d = def.add("bs", CoBools);
            d.category = "Switches".into();
            d.tooltip = "Bools".into();
            d.set_default_value(Box::new(ConfigOptionBools::from_values(vec![1])));
        }
        def.add("bs0", CoBools).set_default_value(Box::new(ConfigOptionBools::from_values(vec![0])));
        def.add("bs2", CoBools).set_default_value(Box::new(ConfigOptionBools::from_values(vec![1, 1])));
        def.add_nullable("bsn", CoBools);
        {
            let d = def.add("e", CoEnum);
            d.enum_keys_map = Some(test_enum_values());
            d.enum_values = vec!["alpha".into(), "beta".into(), "gamma".into()];
            d.tooltip = "An enum".into();
            d.category = "Switches".into();
            d.set_default_value(Box::new(ConfigOptionEnum::<TestEnum>::with_value(TestEnum(1))));
        }
        def.add("e0", CoEnum).enum_keys_map = Some(test_enum_values());
        {
            let d = def.add("es", CoEnums);
            d.enum_keys_map = Some(test_enum_values());
            d.set_default_value(Box::new(enums_generic(vec![1])));
        }
        {
            let d = def.add("esn", CoEnums);
            d.enum_keys_map = Some(test_enum_values());
            d.set_default_value(Box::new(enums_generic_nullable(vec![2, ConfigOptionInts::nil_value()])));
        }
        def.add("es0", CoEnums).enum_keys_map = Some(test_enum_values());
        def.add("ig", CoIntsGroups).set_default_value(Box::new(ConfigOptionIntsGroups::from_values(vec![vec![1, 2]])));
        def.add("ig0", CoIntsGroups);
        def.add("pg", CoPointsGroups)
            .set_default_value(Box::new(ConfigOptionPointsGroups::from_values(vec![vec![Vec2d::new(1.0, 2.0)]])));
        {
            let d = def.add("alias_target", CoInt);
            d.aliases = vec!["old_alias".into(), "older_alias".into()];
            d.set_default_value(Box::new(ConfigOptionInt::with_value(0)));
        }
        def.add("sc", CoInt).shortcut = vec!["i".into(), "alias_target".into()];
        {
            let d = def.add("nocli_opt", CoInt);
            d.cli = ConfigOptionDef::NOCLI.into();
            d.set_default_value(Box::new(ConfigOptionInt::with_value(9)));
        }
        def.add("close_fan_the_first_x_layers", CoInts).set_default_value(Box::new(ConfigOptionInts::from_values(vec![1])));
        def.add("close_additional_fan_first_x_layers", CoInts)
            .set_default_value(Box::new(ConfigOptionInts::from_values(vec![2])));
        def.add("close_optional_fan_first_x_layers", CoInts).set_default_value(Box::new(ConfigOptionInts::from_values(vec![3])));
        def.add("additional_cooling_fan_speed", CoInts).set_default_value(Box::new(ConfigOptionInts::from_values(vec![70])));
        def.add("optional_cooling_fan_speed", CoInts).set_default_value(Box::new(ConfigOptionInts::from_values(vec![80])));
        def.add("first_x_layer_fan_speed", CoFloats).set_default_value(Box::new(ConfigOptionFloats::from_values(vec![0.0])));
        def.add("first_x_layer_optional_fan_speed", CoFloats)
            .set_default_value(Box::new(ConfigOptionFloats::from_values(vec![0.0])));
        def.add("different_settings_to_system", CoStrings).set_default_value(Box::new(ConfigOptionStrings::new()));
        def.add("filament_settings_id", CoStrings).set_default_value(Box::new(ConfigOptionStrings::new()));
        def.add("reduce_infill_retraction", CoBool).set_default_value(Box::new(ConfigOptionBool::with_value(true)));
        {
            let d = def.add("reduce_infill_retraction_mode", CoEnum);
            d.enum_keys_map = Some(rir_values());
            d.set_default_value(Box::new(ConfigOptionEnumGeneric::with_value(Some(rir_values()), 1)));
        }
        def.add("support_type", CoString).set_default_value(Box::new(ConfigOptionString::with_value("normal(auto)")));
        {
            let d = def.add("support_style", CoEnum);
            d.enum_keys_map = Some(style_values());
            d.set_default_value(Box::new(ConfigOptionEnumGeneric::with_value(Some(style_values()), 0)));
        }
        def.add("wall_infill_order", CoString)
            .set_default_value(Box::new(ConfigOptionString::with_value("inner wall/outer wall/infill")));
        def.add("is_infill_first", CoBool).set_default_value(Box::new(ConfigOptionBool::with_value(false)));
        def.add("print_host", CoString).set_default_value(Box::new(ConfigOptionString::with_value("")));
        def.add("printhost_apikey", CoString).set_default_value(Box::new(ConfigOptionString::with_value("")));
        def
    })
}

/// TestConfig::handle_legacy() of gen.cpp.
fn test_handle_legacy(opt_key: &mut t_config_option_key, value: &mut String) {
    if opt_key == "legacy_removed" {
        opt_key.clear();
    } else if opt_key == "legacy_renamed" {
        *opt_key = "i".into();
    } else if opt_key == "legacy_value" {
        *opt_key = "s".into();
        value.push_str("_legacy");
    }
}

fn test_config() -> DynamicConfig {
    DynamicConfig::with_def(test_def(), Some(test_handle_legacy))
}

/// TestStaticConfig of gen.cpp: three members.
struct TestStaticConfig {
    f: ConfigOptionFloat,
    is: ConfigOptionInts,
    s: ConfigOptionString,
}
impl TestStaticConfig {
    fn new() -> Self {
        let mut c = TestStaticConfig { f: ConfigOptionFloat::new(), is: ConfigOptionInts::new(), s: ConfigOptionString::new() };
        StaticConfig::set_defaults(&mut c).expect("set_defaults");
        c
    }
}
impl ConfigOptionResolver for TestStaticConfig {
    fn optptr(&self, opt_key: &str) -> Option<&dyn ConfigOption> {
        match opt_key {
            "f" => Some(&self.f),
            "is" => Some(&self.is),
            "s" => Some(&self.s),
            _ => None,
        }
    }
}
impl ConfigBase for TestStaticConfig {
    fn def(&self) -> Option<&'static ConfigDef> {
        Some(test_def())
    }
    fn optptr_mut(&mut self, opt_key: &str, _create: bool) -> Result<Option<&mut dyn ConfigOption>, ConfigurationError> {
        Ok(match opt_key {
            "f" => Some(&mut self.f),
            "is" => Some(&mut self.is),
            "s" => Some(&mut self.s),
            _ => None,
        })
    }
    fn keys(&self) -> t_config_option_keys {
        StaticConfig::keys(self)
    }
}

// ---------------------------------------------------------------------------------------------
// Options: spec, make, dump
// ---------------------------------------------------------------------------------------------

macro_rules! is {
    ($o:expr, $t:ty) => {
        $o.as_any().is::<$t>()
    };
}
macro_rules! cast {
    ($o:expr, $t:ty) => {
        $o.as_any().downcast_ref::<$t>().expect(stringify!($t))
    };
}

fn class_name(o: &dyn ConfigOption) -> &'static str {
    if is!(o, ConfigOptionEnumsGeneric) {
        "EnumsGeneric"
    } else if is!(o, ConfigOptionEnumsGenericNullable) {
        "EnumsGenericNullable"
    } else if is!(o, ConfigOptionEnumGeneric) {
        "EnumGeneric"
    } else if is!(o, ConfigOptionEnum<TestEnum>) {
        "Enum"
    } else if is!(o, ConfigOptionFloatOrPercent) {
        "FloatOrPercent"
    } else if is!(o, ConfigOptionPercent) {
        "Percent"
    } else if is!(o, ConfigOptionFloat) {
        "Float"
    } else if is!(o, ConfigOptionPercents) {
        "Percents"
    } else if is!(o, ConfigOptionPercentsNullable) {
        "PercentsNullable"
    } else if is!(o, ConfigOptionFloats) {
        "Floats"
    } else if is!(o, ConfigOptionFloatsNullable) {
        "FloatsNullable"
    } else if is!(o, ConfigOptionInt) {
        "Int"
    } else if is!(o, ConfigOptionInts) {
        "Ints"
    } else if is!(o, ConfigOptionIntsNullable) {
        "IntsNullable"
    } else if is!(o, ConfigOptionString) {
        "String"
    } else if is!(o, ConfigOptionStrings) {
        "Strings"
    } else if is!(o, ConfigOptionFloatsOrPercents) {
        "FloatsOrPercents"
    } else if is!(o, ConfigOptionFloatsOrPercentsNullable) {
        "FloatsOrPercentsNullable"
    } else if is!(o, ConfigOptionPoint) {
        "Point"
    } else if is!(o, ConfigOptionPoints) {
        "Points"
    } else if is!(o, ConfigOptionPoint3) {
        "Point3"
    } else if is!(o, ConfigOptionPointsGroups) {
        "PointsGroups"
    } else if is!(o, ConfigOptionIntsGroups) {
        "IntsGroups"
    } else if is!(o, ConfigOptionBool) {
        "Bool"
    } else if is!(o, ConfigOptionBools) {
        "Bools"
    } else if is!(o, ConfigOptionBoolsNullable) {
        "BoolsNullable"
    } else {
        panic!("unknown class")
    }
}

fn int_values(o: &dyn ConfigOption) -> &Vec<i32> {
    o.vector_values_any().and_then(|a| a.downcast_ref::<Vec<i32>>()).expect("Vec<i32>")
}
fn float_values(o: &dyn ConfigOption) -> &Vec<f64> {
    o.vector_values_any().and_then(|a| a.downcast_ref::<Vec<f64>>()).expect("Vec<f64>")
}
fn fp_values(o: &dyn ConfigOption) -> &Vec<FloatOrPercent> {
    o.vector_values_any().and_then(|a| a.downcast_ref::<Vec<FloatOrPercent>>()).expect("Vec<FloatOrPercent>")
}
fn u8_values(o: &dyn ConfigOption) -> &Vec<u8> {
    o.vector_values_any().and_then(|a| a.downcast_ref::<Vec<u8>>()).expect("Vec<u8>")
}

/// gen.cpp spec(): <class> <n> <values>
fn spec(l: &mut Vec<String>, o: &dyn ConfigOption) {
    let cls = class_name(o);
    l.push(cls.to_string());
    let mut v: Vec<String> = Vec::new();
    match cls {
        "Float" => v.push(dbl(cast!(o, ConfigOptionFloat).value)),
        "Percent" => v.push(dbl(cast!(o, ConfigOptionPercent).value)),
        "FloatOrPercent" => {
            let p = cast!(o, ConfigOptionFloatOrPercent);
            v.push(dbl(p.value));
            v.push(b(p.percent));
        }
        "Floats" | "FloatsNullable" | "Percents" | "PercentsNullable" => v.extend(float_values(o).iter().map(|d| dbl(*d))),
        "Int" => v.push(cast!(o, ConfigOptionInt).value.to_string()),
        "EnumGeneric" => v.push(cast!(o, ConfigOptionEnumGeneric).value.to_string()),
        "Enum" => v.push(o.get_int().expect("getInt").to_string()),
        "Ints" | "IntsNullable" | "EnumsGeneric" | "EnumsGenericNullable" => v.extend(int_values(o).iter().map(|d| d.to_string())),
        "String" => v.push(hs(&cast!(o, ConfigOptionString).value)),
        "Strings" => v.extend(cast!(o, ConfigOptionStrings).values.iter().map(|s| hs(s))),
        "FloatsOrPercents" | "FloatsOrPercentsNullable" => {
            v.extend(fp_values(o).iter().map(|f| format!("{}:{}", dbl(f.value), b(f.percent))))
        }
        "Point" => {
            let p = cast!(o, ConfigOptionPoint);
            v.push(dbl(p.value.x));
            v.push(dbl(p.value.y));
        }
        "Point3" => {
            let p = cast!(o, ConfigOptionPoint3);
            v.push(dbl(p.value.x));
            v.push(dbl(p.value.y));
            v.push(dbl(p.value.z));
        }
        "Points" => v.extend(cast!(o, ConfigOptionPoints).values.iter().map(|p| format!("{}:{}", dbl(p.x), dbl(p.y)))),
        "PointsGroups" => {
            for grp in &cast!(o, ConfigOptionPointsGroups).values {
                let t: Vec<String> = grp.iter().map(|p| format!("{}:{}", dbl(p.x), dbl(p.y))).collect();
                v.push(if t.is_empty() { "-".to_string() } else { t.join(";") });
            }
        }
        "IntsGroups" => {
            for grp in &cast!(o, ConfigOptionIntsGroups).values {
                let t: Vec<String> = grp.iter().map(|d| d.to_string()).collect();
                v.push(if t.is_empty() { "-".to_string() } else { t.join(",") });
            }
        }
        "Bool" => v.push(b(cast!(o, ConfigOptionBool).value)),
        "Bools" | "BoolsNullable" => v.extend(u8_values(o).iter().map(|d| d.to_string())),
        _ => unreachable!(),
    }
    l.push(v.len().to_string());
    l.extend(v);
}

/// An option from its spec tokens.
fn read_spec(c: &mut Cur) -> Box<dyn ConfigOption> {
    let cls = c.next();
    let n = c.int() as usize;
    let v: Vec<&str> = (0..n).map(|_| c.next()).collect();
    let d = |t: &str| undbl(t);
    let i = |t: &str| t.parse::<i32>().expect("int");
    let fp = |t: &str| {
        let (x, p) = t.split_once(':').expect("fp");
        FloatOrPercent::with(undbl(x), p == "1")
    };
    let pt = |t: &str| {
        let (x, y) = t.split_once(':').expect("pt");
        Vec2d::new(undbl(x), undbl(y))
    };
    match cls {
        "Float" => Box::new(ConfigOptionFloat::with_value(d(v[0]))),
        "Percent" => Box::new(ConfigOptionPercent::with_value(d(v[0]))),
        "FloatOrPercent" => Box::new(ConfigOptionFloatOrPercent::with_value(d(v[0]), v[1] == "1")),
        "Floats" => Box::new(ConfigOptionFloats::from_values(v.iter().map(|t| d(t)).collect())),
        "FloatsNullable" => Box::new(ConfigOptionFloatsNullable::from_values(v.iter().map(|t| d(t)).collect())),
        "Percents" => Box::new(ConfigOptionPercents::from_values(v.iter().map(|t| d(t)).collect())),
        "PercentsNullable" => Box::new(ConfigOptionPercentsNullable::from_values(v.iter().map(|t| d(t)).collect())),
        "Int" => Box::new(ConfigOptionInt::with_value(i(v[0]))),
        "EnumGeneric" => Box::new(ConfigOptionEnumGeneric::with_value(Some(test_enum_values()), i(v[0]))),
        "Enum" => Box::new(ConfigOptionEnum::<TestEnum>::with_value(TestEnum(i(v[0])))),
        "Ints" => Box::new(ConfigOptionInts::from_values(v.iter().map(|t| i(t)).collect())),
        "IntsNullable" => Box::new(ConfigOptionIntsNullable::from_values(v.iter().map(|t| i(t)).collect())),
        "EnumsGeneric" => Box::new(enums_generic(v.iter().map(|t| i(t)).collect())),
        "EnumsGenericNullable" => Box::new(enums_generic_nullable(v.iter().map(|t| i(t)).collect())),
        "String" => Box::new(ConfigOptionString::with_value(unhex(v[0]))),
        "Strings" => Box::new(ConfigOptionStrings::from_values(v.iter().map(|t| unhex(t)).collect())),
        "FloatsOrPercents" => Box::new(ConfigOptionFloatsOrPercents::from_values(v.iter().map(|t| fp(t)).collect())),
        "FloatsOrPercentsNullable" => Box::new(ConfigOptionFloatsOrPercentsNullable::from_values(v.iter().map(|t| fp(t)).collect())),
        "Point" => Box::new(ConfigOptionPoint::with_value(Vec2d::new(d(v[0]), d(v[1])))),
        "Point3" => Box::new(ConfigOptionPoint3::with_value(Vec3d::new(d(v[0]), d(v[1]), d(v[2])))),
        "Points" => Box::new(ConfigOptionPoints::from_values(v.iter().map(|t| pt(t)).collect())),
        "PointsGroups" => Box::new(ConfigOptionPointsGroups::from_values(
            v.iter().map(|t| if *t == "-" { Vec::new() } else { t.split(';').map(|p| pt(p)).collect() }).collect(),
        )),
        "IntsGroups" => Box::new(ConfigOptionIntsGroups::from_values(
            v.iter().map(|t| if *t == "-" { Vec::new() } else { t.split(',').map(|p| i(p)).collect() }).collect(),
        )),
        "Bool" => Box::new(ConfigOptionBool::with_value(v[0] == "1")),
        "Bools" => Box::new(ConfigOptionBools::from_values(v.iter().map(|t| t.parse::<u8>().expect("u8")).collect())),
        "BoolsNullable" => Box::new(ConfigOptionBoolsNullable::from_values(v.iter().map(|t| t.parse::<u8>().expect("u8")).collect())),
        other => panic!("class {other}"),
    }
}

/// gen.cpp dump(): spec, type, nullable, is_nil, serialize, hash, and for a vector: size, vserialize
fn dump(l: &mut Vec<String>, o: &dyn ConfigOption) {
    spec(l, o);
    l.push((o.r#type() as i32).to_string());
    l.push(b(o.nullable()));
    l.push(b(o.is_nil()));
    g(l, |r| {
        let s = o.serialize()?;
        r.push("ok".into());
        r.push(hs(&s));
        Ok(())
    });
    l.push(o.hash().to_string());
    if let Some(v) = o.as_vector_base() {
        l.push(v.size().to_string());
        g(l, |r| {
            let vs = v.vserialize()?;
            r.push("ok".into());
            r.push(vs.len().to_string());
            r.extend(vs.iter().map(|s| hs(s)));
            Ok(())
        });
    }
}

fn vb(o: &mut Box<dyn ConfigOption>) -> &mut dyn ConfigOptionVectorBase {
    o.as_vector_base_mut().expect("ConfigOptionVectorBase")
}

/// operator< of two options of the class (gen.cpp lt()).
fn lt(cls: &str, a: &dyn ConfigOption, bo: &dyn ConfigOption) -> Option<bool> {
    macro_rules! l {
        ($t:ty) => {
            Some(cast!(a, $t).operator_lt(cast!(bo, $t)))
        };
    }
    match cls {
        "Float" => l!(ConfigOptionFloat),
        "Floats" => l!(ConfigOptionFloats),
        "FloatsNullable" => l!(ConfigOptionFloatsNullable),
        "Ints" => l!(ConfigOptionInts),
        "IntsNullable" => l!(ConfigOptionIntsNullable),
        "String" => l!(ConfigOptionString),
        "Strings" => l!(ConfigOptionStrings),
        "Percent" => l!(ConfigOptionPercent),
        "Percents" => l!(ConfigOptionPercents),
        "PercentsNullable" => l!(ConfigOptionPercentsNullable),
        "FloatOrPercent" => l!(ConfigOptionFloatOrPercent),
        "FloatsOrPercents" => l!(ConfigOptionFloatsOrPercents),
        "FloatsOrPercentsNullable" => l!(ConfigOptionFloatsOrPercentsNullable),
        "Point" => l!(ConfigOptionPoint),
        "Points" => l!(ConfigOptionPoints),
        "Point3" => l!(ConfigOptionPoint3),
        "IntsGroups" => l!(ConfigOptionIntsGroups),
        "Bool" => l!(ConfigOptionBool),
        "Bools" => l!(ConfigOptionBools),
        "BoolsNullable" => l!(ConfigOptionBoolsNullable),
        "Enum" => l!(ConfigOptionEnum<TestEnum>),
        "EnumGeneric" => l!(ConfigOptionEnumGeneric),
        // `*a < *static_cast<const ConfigOptionInts*>(b)`: b is the ConfigOptionInts it derives from.
        "EnumsGeneric" => Some(cast!(a, ConfigOptionEnumsGeneric).operator_lt(bo)),
        "EnumsGenericNullable" => Some(cast!(a, ConfigOptionEnumsGenericNullable).operator_lt(bo)),
        _ => None,
    }
}

fn idx(c: &mut Cur) -> Vec<i32> {
    c.ints()
}

// ---------------------------------------------------------------------------------------------
// Configurations
// ---------------------------------------------------------------------------------------------

fn read_cfg(c: &mut Cur) -> DynamicConfig {
    let with_def = c.next() == "D";
    let mut cfg = if with_def { test_config() } else { DynamicConfig::new() };
    let n = c.int();
    for _ in 0..n {
        let k = c.text();
        let o = read_spec(c);
        cfg.set_key_value(&k, o);
    }
    cfg
}

fn cfg_dump(l: &mut Vec<String>, c: &DynamicConfig) {
    l.push(c.size().to_string());
    for (k, o) in c.cbegin() {
        l.push(hs(k));
        dump(l, &**o);
    }
}

fn subs_dump(l: &mut Vec<String>, subs: &ConfigSubstitutions) {
    l.push(subs.len().to_string());
    for s in subs {
        l.push(hs(&s.opt_def.expect("opt_def").opt_key));
        l.push(hs(&s.old_value));
        dump(l, &**s.new_value.as_ref().expect("new_value"));
    }
}

fn ctx_dump(l: &mut Vec<String>, ctx: &ConfigSubstitutionContext) {
    subs_dump(l, &ctx.substitutions);
    keys_tokens(l, &ctx.unrecogized_keys);
}

fn rule_of(v: i64) -> ForwardCompatibilitySubstitutionRule {
    use ForwardCompatibilitySubstitutionRule::*;
    match v {
        0 => Disable,
        1 => Enable,
        2 => EnableSilent,
        3 => EnableSystemSilent,
        4 => EnableSilentDisableSystem,
        _ => panic!("rule"),
    }
}

// ---------------------------------------------------------------------------------------------
// The lines
// ---------------------------------------------------------------------------------------------

struct Tmp {
    dir: tempfile::TempDir,
}
impl Tmp {
    fn path(&self, name: &str) -> String {
        self.dir.path().join(name).to_string_lossy().into_owned()
    }
    fn write(&self, name: &str, content: &str) -> String {
        let p = self.path(name);
        std::fs::write(&p, content.as_bytes()).expect("write");
        p
    }
}

/// Runs one line: the Rust tokens of what follows "=>", or None where the line is not compared.
fn run_line(op: &str, c: &mut Cur, tmp: &Tmp) -> Option<Vec<String>> {
    let mut l: Vec<String> = Vec::new();
    match op {
        "des" => {
            let mut o = read_spec(c);
            let append = c.int() != 0;
            let s = c.text();
            g(&mut l, |r| {
                r.push(b(o.deserialize(&s, append)?));
                Ok(())
            });
            dump(&mut l, &*o);
        }
        "ser" => {
            let o = read_spec(c);
            dump(&mut l, &*o);
            match o.serialize() {
                Ok(s) => {
                    let mut o2 = ConfigOption::clone(&*o);
                    g(&mut l, |r| {
                        r.push(b(o2.deserialize(&s, false)?));
                        Ok(())
                    });
                    dump(&mut l, &*o2);
                }
                Err(_) => l.push("noser".into()),
            }
        }
        "eq" => {
            let a = read_spec(c);
            let bo = read_spec(c);
            g(&mut l, |r| {
                r.push(b(a.operator_eq(&*bo)?));
                Ok(())
            });
            g(&mut l, |r| {
                r.push(b(a.operator_ne(&*bo)?));
                Ok(())
            });
            g(&mut l, |r| {
                r.push(b(a.overriden_by(&*bo)?));
                Ok(())
            });
            let ca = class_name(&*a);
            let cb = class_name(&*bo);
            if ca == cb {
                let ok = ca != "IntsGroups"
                    || cast!(bo, ConfigOptionIntsGroups).values.len() >= cast!(a, ConfigOptionIntsGroups).values.len();
                match if ok { lt(ca, &*a, &*bo) } else { None } {
                    Some(v) => l.push(b(v)),
                    None => l.push("nolt".into()),
                }
            } else {
                l.push("nolt".into());
            }
        }
        "set" => {
            let mut a = read_spec(c);
            let bo = read_spec(c);
            g(&mut l, |r| {
                a.set(&*bo)?;
                r.push("ok".into());
                Ok(())
            });
            dump(&mut l, &*a);
        }
        "apply_override" => {
            let mut a = read_spec(c);
            let bo = read_spec(c);
            let mut ix = idx(c);
            g(&mut l, |r| {
                r.push(b(a.apply_override(&*bo, &mut ix)?));
                Ok(())
            });
            dump(&mut l, &*a);
        }
        "getters" => {
            let mut a = read_spec(c);
            let v = c.int() as i32;
            g(&mut l, |r| {
                r.push(a.get_int()?.to_string());
                Ok(())
            });
            g(&mut l, |r| {
                r.push(dbl(a.get_float()?));
                Ok(())
            });
            g(&mut l, |r| {
                r.push(b(a.get_bool()?));
                Ok(())
            });
            g(&mut l, |r| {
                a.set_int(v)?;
                r.push("ok".into());
                Ok(())
            });
            l.push(b(a.is_scalar()));
            l.push(b(a.is_vector()));
            dump(&mut l, &*a);
        }
        "set_from_options" => {
            let mut a = read_spec(c);
            let cnt = c.int();
            let owned: Vec<Box<dyn ConfigOption>> = (0..cnt).map(|_| read_spec(c)).collect();
            let rhs: Vec<&dyn ConfigOption> = owned.iter().map(|o| &**o).collect();
            g(&mut l, |r| {
                vb(&mut a).set_from_options(&rhs)?;
                r.push("ok".into());
                Ok(())
            });
            dump(&mut l, &*a);
        }
        "set_at" => {
            let mut a = read_spec(c);
            let bo = read_spec(c);
            let i = c.int() as usize;
            let j = c.int() as usize;
            g(&mut l, |r| {
                vb(&mut a).set_at(&*bo, i, j)?;
                r.push("ok".into());
                Ok(())
            });
            dump(&mut l, &*a);
        }
        "append" => {
            let mut a = read_spec(c);
            let bo = read_spec(c);
            g(&mut l, |r| {
                vb(&mut a).append(&*bo)?;
                r.push("ok".into());
                Ok(())
            });
            dump(&mut l, &*a);
        }
        "set_range" => {
            let mut a = read_spec(c);
            let bo = read_spec(c);
            let start = c.int() as usize;
            let len = c.int() as usize;
            g(&mut l, |r| {
                vb(&mut a).set_range(&*bo, start, len)?;
                r.push("ok".into());
                Ok(())
            });
            dump(&mut l, &*a);
        }
        "set_with_restore" => {
            let mut a = read_spec(c);
            let bo = read_spec(c);
            let mut ix = idx(c);
            let stride = c.int() as i32;
            g(&mut l, |r| {
                vb(&mut a).set_with_restore(&*bo, &mut ix, stride)?;
                r.push("ok".into());
                Ok(())
            });
            dump(&mut l, &*a);
        }
        "set_with_restore_2" => {
            let mut a = read_spec(c);
            let key = c.text();
            let mut bo = read_spec(c);
            let mut ix = idx(c);
            let start = c.int() as i32;
            let len = c.int() as i32;
            let skip = c.int() != 0;
            g(&mut l, |r| {
                vb(&mut a).set_with_restore_2(&key, &mut *bo, &mut ix, start, len, skip)?;
                r.push("ok".into());
                Ok(())
            });
            dump(&mut l, &*a);
            dump(&mut l, &*bo);
        }
        "set_only_diff" => {
            let mut a = read_spec(c);
            let key = c.text();
            let bo = read_spec(c);
            let mut ix = idx(c);
            let stride = c.int() as i32;
            g(&mut l, |r| {
                vb(&mut a).set_only_diff(&key, &*bo, &mut ix, stride)?;
                r.push("ok".into());
                Ok(())
            });
            dump(&mut l, &*a);
        }
        "set_to_index" => {
            let mut a = read_spec(c);
            let bo = read_spec(c);
            let mut ix = idx(c);
            let stride = c.int() as i32;
            g(&mut l, |r| {
                vb(&mut a).set_to_index(&*bo, &mut ix, stride)?;
                r.push("ok".into());
                Ok(())
            });
            dump(&mut l, &*a);
        }
        "set_with_nil" => {
            let mut a = read_spec(c);
            let rhs = read_spec(c);
            let inh = read_spec(c);
            let stride = c.int() as i32;
            g(&mut l, |r| {
                vb(&mut a).set_with_nil(&*rhs, &*inh, stride)?;
                r.push("ok".into());
                Ok(())
            });
            dump(&mut l, &*a);
        }
        "set_with_default" => {
            let mut a = read_spec(c);
            let inh = read_spec(c);
            g(&mut l, |r| {
                vb(&mut a).set_with_default(&*inh)?;
                r.push("ok".into());
                Ok(())
            });
            dump(&mut l, &*a);
        }
        "resize" => {
            let mut a = read_spec(c);
            let n = c.int() as usize;
            let d = if c.int() != 0 { Some(read_spec(c)) } else { None };
            g(&mut l, |r| {
                vb(&mut a).resize(n, d.as_deref())?;
                r.push("ok".into());
                Ok(())
            });
            dump(&mut l, &*a);
        }
        "nil_at" => {
            let mut a = read_spec(c);
            let i = c.int() as usize;
            l.push(b(vb(&mut a).is_nil_at(i)));
            l.push(b(vb(&mut a).empty()));
            vb(&mut a).set_at_to_nil(i);
            l.push(b(vb(&mut a).is_nil_at(i)));
            dump(&mut l, &*a);
            vb(&mut a).clear();
            dump(&mut l, &*a);
        }
        "override" => {
            let mut a = read_spec(c);
            let bo = read_spec(c);
            let mut ix = idx(c);
            g(&mut l, |r| {
                r.push(b(a.overriden_by(&*bo)?));
                Ok(())
            });
            g(&mut l, |r| {
                r.push(b(a.apply_override(&*bo, &mut ix)?));
                Ok(())
            });
            dump(&mut l, &*a);
        }
        "vserialize_single" => {
            let a = read_spec(c);
            let i = c.int() as i32;
            let vs = if let Some(p) = a.as_any().downcast_ref::<ConfigOptionPointsGroups>() {
                p.vserialize_single(i)
            } else {
                cast!(a, ConfigOptionIntsGroups).vserialize_single(i)
            };
            l.push(vs.len().to_string());
            l.extend(vs.iter().map(|s| hs(s)));
        }
        "bools_subst" => {
            let mut a = read_spec(c);
            let append = c.int() != 0;
            let mode = match c.int() {
                0 => DeserializationSubstitution::Disabled,
                1 => DeserializationSubstitution::DefaultsToFalse,
                _ => DeserializationSubstitution::DefaultsToTrue,
            };
            let s = c.text();
            g(&mut l, |r| {
                let res = if let Some(o) = a.as_any_mut().downcast_mut::<ConfigOptionBoolsNullable>() {
                    o.deserialize_with_substitutions(&s, append, mode)?
                } else {
                    a.as_any_mut()
                        .downcast_mut::<ConfigOptionBools>()
                        .expect("Bools")
                        .deserialize_with_substitutions(&s, append, mode)?
                };
                r.push(
                    match res {
                        DeserializationResult::Loaded => 0,
                        DeserializationResult::Substituted => 1,
                        DeserializationResult::Failed => 2,
                    }
                    .to_string(),
                );
                Ok(())
            });
            dump(&mut l, &*a);
        }
        "strings" => {
            let s = c.text();
            l.push(hs(&escape_string_cstyle(&s)));
            l.push(hs(&escape_ampersand(&s)));
            let mut out = "untouched".to_string();
            l.push(b(unescape_string_cstyle(&s, &mut out)));
            l.push(hs(&out));
            let mut v = vec!["first".to_string()];
            l.push(b(unescape_strings_cstyle(&s, &mut v)));
            keys_tokens(&mut l, &v);
            l.push(b(looks_like_enum_value(&s)));
            l.push(b(enum_looks_like_true_value(&s)));
        }
        "escape_strings" => {
            let v = c.keys();
            let e = escape_strings_cstyle(&v);
            l.push(hs(&e));
            let mut back = Vec::new();
            l.push(b(unescape_strings_cstyle(&e, &mut back)));
            keys_tokens(&mut l, &back);
        }
        "c_set_deserialize" => {
            let mut cfg = read_cfg(c);
            let rule = rule_of(c.int());
            let append = c.int() != 0;
            let key = c.text();
            let value = c.text();
            let mut ctx = ConfigSubstitutionContext::new(rule);
            ctx.unrecogized_keys = c.keys();
            g(&mut l, |r| {
                cfg.set_deserialize(&key, &value, &mut ctx, append)?;
                r.push("ok".into());
                Ok(())
            });
            ctx_dump(&mut l, &ctx);
            cfg_dump(&mut l, &cfg);
        }
        "c_set" => {
            let mut cfg = read_cfg(c);
            let key = c.text();
            let create = c.int() != 0;
            let kind = c.int();
            let v = c.next();
            g(&mut l, |r| {
                match kind {
                    0 => cfg.set_bool(&key, v == "1", create)?,
                    1 => cfg.set_int(&key, v.parse().expect("int"), create)?,
                    2 => cfg.set_double(&key, undbl(v), create)?,
                    _ => cfg.set_string(&key, &unhex(v), create)?,
                }
                r.push("ok".into());
                Ok(())
            });
            cfg_dump(&mut l, &cfg);
        }
        "c_apply" => {
            let a = read_cfg(c);
            let bc = read_cfg(c);
            let ignore = c.int() != 0;
            let mut cc = a.clone();
            g(&mut l, |r| {
                cc.apply(&bc, ignore)?;
                r.push("ok".into());
                Ok(())
            });
            cfg_dump(&mut l, &cc);
        }
        "c_compare" => {
            let a = read_cfg(c);
            let bc = read_cfg(c);
            let skipped: BTreeSet<String> = ["fs".to_string(), "i".to_string()].into_iter().collect();
            g(&mut l, |r| {
                r.push(b(<DynamicConfig as ConfigBase>::equals(&a, &bc, None)?));
                Ok(())
            });
            g(&mut l, |r| {
                r.push(b(<DynamicConfig as ConfigBase>::equals(&a, &bc, Some(&skipped))?));
                Ok(())
            });
            g(&mut l, |r| {
                keys_tokens(r, &<DynamicConfig as ConfigBase>::diff(&a, &bc)?);
                Ok(())
            });
            g(&mut l, |r| {
                keys_tokens(r, &<DynamicConfig as ConfigBase>::equal(&a, &bc)?);
                Ok(())
            });
            g(&mut l, |r| {
                r.push(b(DynamicConfig::equals(&a, &bc, None)?));
                Ok(())
            });
            g(&mut l, |r| {
                r.push(b(DynamicConfig::equals(&a, &bc, Some(&skipped))?));
                Ok(())
            });
            g(&mut l, |r| {
                keys_tokens(r, &DynamicConfig::diff(&a, &bc)?);
                Ok(())
            });
            g(&mut l, |r| {
                keys_tokens(r, &DynamicConfig::equal(&a, &bc)?);
                Ok(())
            });
            g(&mut l, |r| {
                r.push(b(a.operator_eq(&bc)?));
                Ok(())
            });
            g(&mut l, |r| {
                r.push(b(a.operator_ne(&bc)?));
                Ok(())
            });
        }
        "c_add_assign" => {
            let a = read_cfg(c);
            let bc = read_cfg(c);
            let mut cc = a.clone();
            cc.operator_add_assign(&bc);
            cfg_dump(&mut l, &cc);
            let mut c2 = a.clone();
            let mut b2 = bc.clone();
            c2.operator_add_assign_move(&mut b2);
            cfg_dump(&mut l, &c2);
            cfg_dump(&mut l, &b2);
            let mut c3 = a.clone();
            let mut b3 = bc.clone();
            c3.swap(&mut b3);
            cfg_dump(&mut l, &c3);
        }
        "c_apply_only" => {
            let mut a = read_cfg(c);
            let bc = read_cfg(c);
            let ks = c.keys();
            let ignore = c.int() != 0;
            g(&mut l, |r| {
                a.apply_only(&bc, &ks, ignore)?;
                r.push("ok".into());
                Ok(())
            });
            cfg_dump(&mut l, &a);
        }
        "c_key" => {
            let a = read_cfg(c);
            let key = c.text();
            g(&mut l, |r| {
                r.push(hs(&a.opt_serialize(&key)?));
                Ok(())
            });
            g(&mut l, |r| {
                r.push(dbl(a.get_abs_value(&key)?));
                Ok(())
            });
            g(&mut l, |r| {
                r.push(dbl(a.get_abs_value_over(&key, 80.0)?));
                Ok(())
            });
            for i in 0..3 {
                g(&mut l, |r| {
                    r.push(dbl(a.get_abs_value_at(&key, i)?));
                    Ok(())
                });
            }
            l.push(b(a.has(&key)));
            let mut cc = a.clone();
            l.push(b(cc.erase(&key)));
            l.push(b(cc.erase(&key)));
            l.push(cc.size().to_string());
        }
        "c_nodef_abs" => {
            let n = read_cfg(c);
            for key in ["f", "fp", "fps", "fs"] {
                g(&mut l, |r| {
                    r.push(if n.has(key) { dbl(n.get_abs_value(key)?) } else { "none".into() });
                    Ok(())
                });
                g(&mut l, |r| {
                    r.push(if n.has(key) { dbl(n.get_abs_value_at(key, 0)?) } else { "none".into() });
                    Ok(())
                });
            }
        }
        "c_whole" => {
            let a = read_cfg(c);
            keys_tokens(&mut l, &a.keys());
            l.push(b(a.empty()));
            {
                let mut cc = a.clone();
                g(&mut l, |r| {
                    cc.null_nullables()?;
                    r.push("ok".into());
                    Ok(())
                });
                cfg_dump(&mut l, &cc);
                l.push(cc.remove_nil_options().to_string());
                cfg_dump(&mut l, &cc);
            }
            {
                let mut cc = a.clone();
                l.push(cc.remove_nil_options().to_string());
                cfg_dump(&mut l, &cc);
                l.push(b(cc.set_key_value("f", Box::new(ConfigOptionFloat::with_value(9.0)))));
                l.push(b(cc.set_key_value("brand_new", Box::new(ConfigOptionInt::with_value(4)))));
                cfg_dump(&mut l, &cc);
                cc.clear();
                l.push(b(cc.empty()));
            }
            {
                let mut ks = a.keys();
                ks.truncate(3);
                let cc = DynamicConfig::from_config(&a, &ks);
                cfg_dump(&mut l, &cc);
            }
            g(&mut l, |r| {
                a.setenv_()?;
                r.push("ok".into());
                Ok(())
            });
            for key in a.keys() {
                let env: String = format!("SLIC3R_{}", key.to_ascii_uppercase());
                match std::env::var_os(&env) {
                    Some(v) => l.push(hs(&v.to_string_lossy())),
                    None => l.push("unset".into()),
                }
                std::env::remove_var(&env);
            }
        }
        "c_accessors" => {
            let mut a = read_cfg(c);
            {
                let ca = &a;
                l.push(hs(ca.opt_string("s")));
                for i in 0..3 {
                    l.push(hs(ca.opt_string_at("ss", i)));
                }
                l.push(dbl(*ca.opt_float("f")));
                l.push(dbl(*ca.opt_float("p")));
                l.push(dbl(*ca.opt_float("fp")));
                for i in 0..3 {
                    l.push(dbl(*ca.opt_float_at("fs", i)));
                }
                for i in 0..3 {
                    l.push(dbl(*ca.opt_float_at("fsn", i)));
                }
                for i in 0..3 {
                    l.push(dbl(*ca.opt_float_at("ps", i)));
                }
                for i in 0..3 {
                    l.push(dbl(*ca.opt_float_nullable("fsn", i)));
                }
                l.push(ca.opt_int("i").to_string());
                l.push(ca.opt_int("e").to_string());
                for i in 0..4 {
                    l.push(ca.opt_int_at("is", i).to_string());
                }
                for i in 0..3 {
                    l.push(ca.opt_int_at("es", i).to_string());
                }
                for i in 0..3 {
                    l.push(ca.opt_int_nullable("isn", i).to_string());
                }
                l.push(ca.opt_enum::<TestEnum>("e").expect("opt_enum").0.to_string());
                for i in 0..3 {
                    l.push(ca.opt_enum_at("es", i).to_string());
                }
                l.push(b(ca.opt_bool("b")));
                for i in 0..3 {
                    l.push(b(ca.opt_bool_at("bs", i)));
                }
                for i in 0..3 {
                    l.push(b(ca.opt_bool_at("bsn", i)));
                }
                for i in 0..3 {
                    l.push(b(ca.opt_bool_nullable("bsn", i)));
                }
            }
            a.opt_string_mut("s", false).push('!');
            *a.opt_string_mut("s_empty", true) = "created".into();
            a.opt_string_at_mut("ss", 5).push('?');
            *a.opt_float_mut("f") += 1.0;
            *a.opt_float_at_mut("fs", 1) = 7.0;
            *a.opt_float_at_mut("fsn", 0) = 8.0;
            *a.opt_float_nullable_mut("fsn", 9) = 9.0;
            *a.opt_int_mut("i") = 70;
            *a.opt_int_at_mut("is", 2) = 71;
            *a.opt_int_nullable_mut("isn", 1) = 72;
            cfg_dump(&mut l, &a);
            l.push(b(a.opt_mut::<ConfigOptionFloat>("f", false).expect("opt").is_some()));
            l.push(b(a.opt_mut::<ConfigOptionFloat>("i", false).expect("opt").is_some()));
            l.push(b(a.opt_mut::<ConfigOptionFloat>("nope", false).expect("opt").is_some()));
            l.push(b(a.opt_mut::<ConfigOptionInt>("alias_target", true).expect("opt").is_some()));
            l.push(b(a.option_typed_mut::<ConfigOptionFloat>("f", false).expect("option").is_some()));
            l.push(b(a.option_typed_mut::<ConfigOptionFloat>("i", false).expect("option").is_some()));
            l.push(b(a.option_typed_mut::<ConfigOptionInts>("is", false).expect("option").is_some()));
            g(&mut l, |r| {
                r.push(a.option_throw_typed_mut::<ConfigOptionInt>("i", false)?.value.to_string());
                Ok(())
            });
            g(&mut l, |r| {
                r.push(a.option_throw_typed_mut::<ConfigOptionInt>("f", false)?.value.to_string());
                Ok(())
            });
            g(&mut l, |r| {
                r.push(a.option_throw_typed_mut::<ConfigOptionInt>("nope", false)?.value.to_string());
                Ok(())
            });
            g(&mut l, |r| {
                a.option_throw_mut("nope", false)?;
                r.push("1".into());
                Ok(())
            });
            g(&mut l, |r| {
                a.option_throw("nope")?;
                r.push("1".into());
                Ok(())
            });
            g(&mut l, |r| {
                a.option_throw_typed_mut::<ConfigOptionInt>("s", false)?;
                r.push("1".into());
                Ok(())
            });
            cfg_dump(&mut l, &a);
        }
        "c_load_string_map" => {
            let mut cfg = read_cfg(c);
            let rule = rule_of(c.int());
            let n = c.int();
            let mut m = BTreeMap::new();
            for _ in 0..n {
                let k = c.text();
                let v = c.text();
                m.insert(k, v);
            }
            g(&mut l, |r| {
                let subs = cfg.load_string_map(&m, rule)?;
                r.push("ok".into());
                subs_dump(r, &subs);
                Ok(())
            });
            cfg_dump(&mut l, &cfg);
        }
        "c_ini" => {
            let mut cfg = read_cfg(c);
            let rule = rule_of(c.int());
            let ini = c.text();
            g(&mut l, |r| {
                let subs = cfg.load_from_ini_string(&ini, rule)?;
                r.push("ok".into());
                subs_dump(r, &subs);
                Ok(())
            });
            cfg_dump(&mut l, &cfg);
            let path = tmp.write("x.ini", &ini);
            let mut c2 = test_config();
            let mut r = Vec::new();
            match c2.load_from_ini(&path, rule) {
                Ok(subs) => {
                    r.push("ok".into());
                    subs_dump(&mut r, &subs);
                }
                Err(e) => {
                    let e = ConfigurationError::with_kind(e.kind, no_path(e.what(), &path));
                    exc(&mut r, &e);
                }
            }
            l.extend(r);
            cfg_dump(&mut l, &c2);
        }
        "c_ini_commented" => {
            let mut cfg = read_cfg(c);
            let ini = c.text();
            g(&mut l, |r| {
                let subs = cfg.load_from_ini_string_commented(ini.clone(), ForwardCompatibilitySubstitutionRule::Enable)?;
                r.push("ok".into());
                subs_dump(r, &subs);
                Ok(())
            });
            cfg_dump(&mut l, &cfg);
        }
        "c_gcode_legacy" => {
            let mut cfg = read_cfg(c);
            let rule = rule_of(c.int());
            let gc = c.text();
            let mut ctx = ConfigSubstitutionContext::new(rule);
            g(&mut l, |r| {
                r.push(cfg.load_from_gcode_string_legacy(Some(gc.as_bytes()), &mut ctx)?.to_string());
                Ok(())
            });
            ctx_dump(&mut l, &ctx);
            cfg_dump(&mut l, &cfg);
        }
        "c_gcode_file" => {
            let rule = rule_of(c.int());
            let via_load = c.int() != 0;
            let content = c.text();
            let path = tmp.write("x.gcode", &content);
            let mut cfg = test_config();
            let res = if via_load { cfg.load(&path, rule) } else { cfg.load_from_gcode_file(&path, rule) };
            match res {
                Ok(subs) => {
                    l.push("ok".into());
                    subs_dump(&mut l, &subs);
                }
                Err(e) => {
                    let e = ConfigurationError::with_kind(e.kind, no_path(e.what(), &path));
                    exc(&mut l, &e);
                }
            }
            cfg_dump(&mut l, &cfg);
        }
        "c_load_other" => {
            let path = tmp.write("x.ini", "f = 5\n");
            let mut cfg = test_config();
            g(&mut l, |r| {
                let subs = cfg.load(&path, ForwardCompatibilitySubstitutionRule::Enable)?;
                r.push("ok".into());
                subs_dump(r, &subs);
                Ok(())
            });
            cfg_dump(&mut l, &cfg);
        }
        "c_save" => {
            let a = read_cfg(c);
            let path = tmp.path("saved.ini");
            let _ = std::fs::remove_file(&path);
            g(&mut l, |r| {
                a.save(&path)?;
                r.push("ok".into());
                Ok(())
            });
            let content = std::fs::read(&path).unwrap_or_default();
            let rest = match content.iter().position(|&ch| ch == b'\n') {
                None => &content[..],
                Some(nl) => &content[nl + 1..],
            };
            l.push(hex(rest));
        }
        "c_save_json" => {
            let a = read_cfg(c);
            let from = c.text();
            let path = tmp.path("saved.json");
            let _ = std::fs::remove_file(&path);
            g(&mut l, |r| {
                a.save_to_json(&path, "the \"name\"", &from, "1.2.3.4")?;
                r.push("ok".into());
                Ok(())
            });
            l.push(hex(&std::fs::read(&path).unwrap_or_default()));
        }
        "c_json" => {
            let rule = rule_of(c.int());
            let inherits = c.int() != 0;
            let json = c.text();
            let path = tmp.write("x.json", &json);
            let mut cfg = test_config();
            let mut ctx = ConfigSubstitutionContext::new(rule);
            let mut key_values: BTreeMap<String, String> = [("version".to_string(), "preset".to_string())].into_iter().collect();
            let mut reason = "unset".to_string();
            let ret = cfg.load_from_json_with_context(&path, &mut ctx, inherits, &mut key_values, &mut reason);
            l.push(ret.to_string());
            l.push(hs(&no_path(&reason, &path)));
            l.push(key_values.len().to_string());
            for (k, v) in &key_values {
                l.push(hs(k));
                l.push(hs(v));
            }
            ctx_dump(&mut l, &ctx);
            cfg_dump(&mut l, &cfg);
        }
        "c_json_missing" => {
            let mut cfg = test_config();
            let mut ctx = ConfigSubstitutionContext::new(ForwardCompatibilitySubstitutionRule::Enable);
            let mut key_values = BTreeMap::new();
            let mut reason = "unset".to_string();
            l.push(cfg.load_from_json_with_context(&tmp.path("does_not_exist.json"), &mut ctx, true, &mut key_values, &mut reason).to_string());
            l.push(hs(&reason));
            let path = tmp.write("y.json", "{\"f\": \"4.5\", \"e\": \"delta\", \"inherits\": \"p\"}");
            g(&mut l, |r| {
                let subs = cfg.load(&path, ForwardCompatibilitySubstitutionRule::Enable)?;
                r.push("ok".into());
                subs_dump(r, &subs);
                Ok(())
            });
            cfg_dump(&mut l, &cfg);
        }
        "c_read_cli" => {
            let mut cfg = read_cfg(c);
            let args = c.keys();
            if !cfg!(target_os = "macos") && args.iter().any(|a| a.starts_with("-psn_")) {
                return None;
            }
            let mut extra = Vec::new();
            let mut order = Vec::new();
            g(&mut l, |r| {
                r.push(b(cfg.read_cli(&args, &mut extra, Some(&mut order))?));
                Ok(())
            });
            keys_tokens(&mut l, &extra);
            keys_tokens(&mut l, &order);
            cfg_dump(&mut l, &cfg);
        }
        "def" => {
            let key = c.text();
            let d = test_def().get(&key).expect("def");
            l.push((d.r#type as i32).to_string());
            l.push(b(d.nullable));
            keys_tokens(&mut l, &d.cli_args(&key));
            g(&mut l, |r| {
                let o = d.create_empty_option()?;
                dump(r, &*o);
                Ok(())
            });
            g(&mut l, |r| {
                let o = d.create_default_option()?;
                dump(r, &*o);
                Ok(())
            });
            l.push(b(d.has_enum_value("beta")));
            l.push(b(test_def().has(&key)));
        }
        "def_keys" => {
            keys_tokens(&mut l, &test_def().keys());
            l.push(b(test_def().has("nope")));
            l.push(b(test_def().get("nope").is_some()));
        }
        "cli_help" => {
            let show_defaults = c.int() != 0;
            let filter = c.int();
            let mut s = String::new();
            test_def()
                .print_cli_help(&mut s, show_defaults, &|d: &ConfigOptionDef| {
                    filter == 0 || (filter == 1 && !d.category.is_empty()) || (filter == 2 && d.category == "Numbers")
                })
                .expect("print_cli_help");
            l.push(hs(&s));
        }
        "c_set_items" => {
            let which = c.int();
            let mut cfg = test_config();
            let mut ctx = ConfigSubstitutionContext::new(if which == 1 {
                ForwardCompatibilitySubstitutionRule::Enable
            } else {
                ForwardCompatibilitySubstitutionRule::Disable
            });
            g(&mut l, |r| {
                use SetDeserializeItem as I;
                match which {
                    0 => cfg.set_deserialize_items(
                        &[
                            I::new("f", "2.5", false),
                            I::new("is", "4,5", true),
                            I::from_bool("b", true, false),
                            I::from_bool("b1", false, false),
                            I::from_int("i", 12, false),
                            I::from_int("alias_target", -3, false),
                            I::from_float("fp", 0.1f32, false),
                            I::from_double("p", 1e-7, false),
                            I::from_double("s", 1234567.891, false),
                            I::from_float("fs", 0.30000001f32, true),
                            I::from_double("ps", 2.5, true),
                        ],
                        &mut ctx,
                    )?,
                    1 => cfg.set_deserialize_items(
                        &[I::new("e", "delta", false), I::new("bs", "1,maybe", false), I::new("is", "6", false), I::new("is", "7", true)],
                        &mut ctx,
                    )?,
                    2 => cfg.set_deserialize_strict_items(&[I::new("f", "3", false), I::new("e", "delta", false), I::new("i", "4", false)])?,
                    _ => {
                        cfg.set_deserialize_strict("is", "1,2", false)?;
                        cfg.set_deserialize_strict("is", "3", true)?;
                        cfg.set_deserialize_strict("f", "x", false)?;
                    }
                }
                r.push("ok".into());
                Ok(())
            });
            ctx_dump(&mut l, &ctx);
            cfg_dump(&mut l, &cfg);
        }
        "misc" => {
            for v in [0, 1, 2, 3, -1] {
                l.push(b(ConfigOptionEnum::<TestEnum>::has(TestEnum(v))));
            }
            for s in ["alpha", "beta_alias", "gamma", "delta", ""] {
                let mut v = TestEnum(7);
                l.push(b(ConfigOptionEnum::<TestEnum>::from_string(s, &mut v)));
                l.push(v.0.to_string());
            }
            l.push(ConfigOptionEnum::<TestEnum>::get_enum_names().len().to_string());
            for d in [0.49999999999999994, 2.5, -2.5, -0.5, 1.4999, 1e9, -1e9, 0.5] {
                l.push(ConfigOptionInt::with_double(d).value.to_string());
            }
            let a = enums_generic(vec![0, 2]);
            let bb = enums_generic(vec![0, 2]);
            let cc = enums_generic(vec![0, 1, 2]);
            let ints = ConfigOptionInts::from_values(vec![0, 2]);
            g(&mut l, |r| {
                r.push(b(a.operator_eq_ints(&ints)?));
                Ok(())
            });
            g(&mut l, |r| {
                r.push(b(a.operator_eq_ints(&bb)?));
                Ok(())
            });
            g(&mut l, |r| {
                r.push(b(a.operator_eq_ints(&cc)?));
                Ok(())
            });
            l.push(b(a.operator_lt(&ints)));
            l.push(b(a.operator_lt(&cc)));
            l.push(b(cc.operator_lt(&a)));
        }
        "static" => {
            let mut s = TestStaticConfig::new();
            let dump_all = |l: &mut Vec<String>, s: &TestStaticConfig| {
                for key in s.keys() {
                    dump(l, s.option(&key).expect("option"));
                }
            };
            keys_tokens(&mut l, &s.keys());
            dump_all(&mut l, &s);
            let mut a = test_config();
            for (k, o) in init_a() {
                a.set_key_value(&k, o);
            }
            g(&mut l, |r| {
                s.apply(&a, true)?;
                r.push("ok".into());
                Ok(())
            });
            dump_all(&mut l, &s);
            g(&mut l, |r| {
                s.apply(&a, false)?;
                r.push("ok".into());
                Ok(())
            });
            g(&mut l, |r| {
                r.push(b(s.equals(&a, None)?));
                Ok(())
            });
            g(&mut l, |r| {
                keys_tokens(r, &s.diff(&a)?);
                Ok(())
            });
            let mut ctx = ConfigSubstitutionContext::new(ForwardCompatibilitySubstitutionRule::Disable);
            g(&mut l, |r| {
                s.set_deserialize("is", "9,8", &mut ctx, true)?;
                r.push("ok".into());
                Ok(())
            });
            g(&mut l, |r| {
                s.set_deserialize("legacy_value", "v", &mut ctx, false)?;
                r.push("ok".into());
                Ok(())
            });
            dump_all(&mut l, &s);
        }
        other => panic!("op {other}"),
    }
    Some(l)
}

/// init_a() of gen.cpp, for the StaticConfig line (which does not carry the configuration).
fn init_a() -> Vec<(String, Box<dyn ConfigOption>)> {
    let nil_f = ConfigOptionFloatsNullable::nil_value();
    let nil_i = ConfigOptionIntsNullable::nil_value();
    vec![
        ("f".into(), Box::new(ConfigOptionFloat::with_value(2.5)) as Box<dyn ConfigOption>),
        ("fs".into(), Box::new(ConfigOptionFloats::from_values(vec![0.4, 0.6]))),
        ("fsn".into(), Box::new(ConfigOptionFloatsNullable::from_values(vec![nil_f, 1.5]))),
        ("i".into(), Box::new(ConfigOptionInt::with_value(7))),
        ("is".into(), Box::new(ConfigOptionInts::from_values(vec![1, 2, 3]))),
        ("isn".into(), Box::new(ConfigOptionIntsNullable::from_values(vec![nil_i, 3]))),
        ("s".into(), Box::new(ConfigOptionString::with_value("hello world"))),
        ("ss".into(), Box::new(ConfigOptionStrings::from_values(vec!["a".into(), "b c".into()]))),
        ("p".into(), Box::new(ConfigOptionPercent::with_value(40.0))),
        ("ps".into(), Box::new(ConfigOptionPercents::from_values(vec![10.0, 20.0]))),
        ("fp".into(), Box::new(ConfigOptionFloatOrPercent::with_value(50.0, true))),
        ("fp2".into(), Box::new(ConfigOptionFloatOrPercent::with_value(25.0, true))),
        ("fp3".into(), Box::new(ConfigOptionFloatOrPercent::with_value(3.0, false))),
        ("fp4".into(), Box::new(ConfigOptionFloatOrPercent::with_value(200.0, true))),
        ("fp5".into(), Box::new(ConfigOptionFloatOrPercent::with_value(10.0, true))),
        ("fp6".into(), Box::new(ConfigOptionFloatOrPercent::with_value(10.0, true))),
        (
            "fps".into(),
            Box::new(ConfigOptionFloatsOrPercents::from_values(vec![FloatOrPercent::with(10.0, false), FloatOrPercent::with(20.0, true)])),
        ),
        ("fps0".into(), Box::new(ConfigOptionFloatsOrPercents::from_values(vec![FloatOrPercent::with(10.0, true)]))),
        ("pt".into(), Box::new(ConfigOptionPoint::with_value(Vec2d::new(1.0, 2.0)))),
        ("pts".into(), Box::new(ConfigOptionPoints::from_values(vec![Vec2d::new(0.0, 0.0), Vec2d::new(10.0, 20.0)]))),
        ("b".into(), Box::new(ConfigOptionBool::with_value(true))),
        ("bs".into(), Box::new(ConfigOptionBools::from_values(vec![1, 0]))),
        ("bsn".into(), Box::new(ConfigOptionBoolsNullable::from_values(vec![ConfigOptionBoolsNullable::nil_value(), 1]))),
        ("e".into(), Box::new(ConfigOptionEnumGeneric::with_value(Some(test_enum_values()), 1))),
        ("es".into(), Box::new(enums_generic(vec![0, 2]))),
        ("ig".into(), Box::new(ConfigOptionIntsGroups::from_values(vec![vec![1, 2], vec![3]]))),
    ]
}

fn check_file(name: &str, counts: &mut BTreeMap<String, (usize, usize)>, failures: &mut Vec<String>) {
    let text = read_vectors(name);
    let tmp = Tmp { dir: tempfile::tempdir().expect("tempdir") };
    for (no, line) in text.lines().enumerate() {
        let (input, output) = line.split_once(" => ").unwrap_or_else(|| (line.trim_end_matches(" =>"), ""));
        let mut c = Cur::new(input);
        let op = c.next().to_string();
        let expected: Vec<String> = if output.is_empty() { Vec::new() } else { output.split(' ').map(|s| s.to_string()).collect() };
        let res = catch_unwind(AssertUnwindSafe(|| run_line(&op, &mut c, &tmp)));
        let entry = counts.entry(op.clone()).or_insert((0, 0));
        match res {
            Ok(None) => {}
            Ok(Some(actual)) => {
                entry.0 += 1;
                if actual != expected {
                    entry.1 += 1;
                    let k = actual.iter().zip(expected.iter()).position(|(a, e)| a != e).unwrap_or(actual.len().min(expected.len()));
                    let lo = k.saturating_sub(3);
                    failures.push(format!(
                        "{}:{} {}: token {} differs\n  input:    {}\n  expected: {}\n  actual:   {}",
                        name,
                        no + 1,
                        op,
                        k,
                        if input.len() > 400 { &input[..400] } else { input },
                        expected[lo.min(expected.len())..(k + 8).min(expected.len())].join(" "),
                        actual[lo.min(actual.len())..(k + 8).min(actual.len())].join(" "),
                    ));
                }
            }
            Err(p) => {
                entry.0 += 1;
                entry.1 += 1;
                let msg = p
                    .downcast_ref::<String>()
                    .cloned()
                    .or_else(|| p.downcast_ref::<&str>().map(|s| s.to_string()))
                    .unwrap_or_default();
                failures.push(format!("{}:{} {}: panic: {}\n  input: {}", name, no + 1, op, msg, &input[..input.len().min(400)]));
            }
        }
    }
}

fn run(files: &[&str]) {
    let mut counts = BTreeMap::new();
    let mut failures = Vec::new();
    for f in files {
        check_file(f, &mut counts, &mut failures);
    }
    let mut summary = String::new();
    for (op, (n, bad)) in &counts {
        summary.push_str(&format!("  {op}: {n} cases, {bad} differ\n"));
    }
    eprintln!("{summary}");
    if !failures.is_empty() {
        let shown: Vec<&String> = failures.iter().take(40).collect();
        panic!(
            "{} of the lines differ from the C++:\n{}\n{}",
            failures.len(),
            summary,
            shown.iter().map(|s| s.as_str()).collect::<Vec<_>>().join("\n")
        );
    }
}

#[test]
fn options_scalar_match_the_cpp() {
    run(&["options_scalar"]);
}

#[test]
fn options_vector_match_the_cpp() {
    run(&["options_vector"]);
}

#[test]
fn option_pairs_match_the_cpp() {
    run(&["option_pairs"]);
}

#[test]
fn vector_ops_match_the_cpp() {
    run(&["vector_ops"]);
}

#[test]
fn config_matches_the_cpp() {
    run(&["config"]);
}
