//! `slicer::print_config::config_def` and `slicer::print_config::enums` (the port of the
//! definitions of libslic3r/PrintConfig.hpp and PrintConfig.cpp) against the C++ at the pin.
//!
//! `tests/data/print_config/*.txt.gz` are the output of `gen.cpp` beside them (gzip -9n), which
//! links the pin's PrintConfig.cpp and dumps the static key maps of the enums, every definition of
//! `print_config_def` with its default value, and PrintConfigDef::handle_legacy over a set of keys
//! and values that runs through every branch of it. The header of gen.cpp describes the lines.
//! What the Rust gives has to be the same line, byte for byte.

use slicer::config::*;
use slicer::print_config::config_def::{print_config_def, PrintConfigDef};
use slicer::print_config::enums;
use std::collections::BTreeMap;
use std::io::Read;

fn read_vectors(name: &str) -> String {
    let path = format!("{}/tests/data/print_config/{}.txt.gz", env!("CARGO_MANIFEST_DIR"), name);
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
fn b(v: bool) -> String {
    (v as i32).to_string()
}
fn push_list(l: &mut Vec<String>, v: &[String]) {
    l.push(v.len().to_string());
    for s in v {
        l.push(hex(s));
    }
}

/// gen.cpp class_name()
fn class_name(o: &dyn ConfigOption) -> &'static str {
    let a = o.as_any();
    if a.is::<ConfigOptionEnumsGeneric>() {
        return "EnumsGeneric";
    }
    if a.is::<ConfigOptionEnumsGenericNullable>() {
        return "EnumsGenericNullable";
    }
    if a.is::<ConfigOptionEnumGeneric>() {
        return "EnumGeneric";
    }
    if o.r#type() == ConfigOptionType::CoEnum {
        return "Enum";
    }
    macro_rules! classes {
        ($($t:ty => $n:expr),* $(,)?) => {
            $(if a.is::<$t>() { return $n; })*
        };
    }
    classes! {
        ConfigOptionFloatOrPercent => "FloatOrPercent",
        ConfigOptionPercent => "Percent",
        ConfigOptionFloat => "Float",
        ConfigOptionPercents => "Percents",
        ConfigOptionPercentsNullable => "PercentsNullable",
        ConfigOptionFloats => "Floats",
        ConfigOptionFloatsNullable => "FloatsNullable",
        ConfigOptionInt => "Int",
        ConfigOptionInts => "Ints",
        ConfigOptionIntsNullable => "IntsNullable",
        ConfigOptionString => "String",
        ConfigOptionStrings => "Strings",
        ConfigOptionFloatsOrPercents => "FloatsOrPercents",
        ConfigOptionFloatsOrPercentsNullable => "FloatsOrPercentsNullable",
        ConfigOptionPoint => "Point",
        ConfigOptionPoints => "Points",
        ConfigOptionPoint3 => "Point3",
        ConfigOptionPointsGroups => "PointsGroups",
        ConfigOptionIntsGroups => "IntsGroups",
        ConfigOptionBool => "Bool",
        ConfigOptionBools => "Bools",
        ConfigOptionBoolsNullable => "BoolsNullable",
    }
    "?"
}

/// Compares the lines; prints up to 20 that differ and fails when one does.
fn compare(what: &str, expected: &[String], got: &[String]) {
    let mut bad = 0usize;
    for (i, (e, g)) in expected.iter().zip(got.iter()).enumerate() {
        if e != g {
            bad += 1;
            if bad <= 20 {
                eprintln!("{what} line {i}:\n  C++:  {e}\n  Rust: {g}");
            }
        }
    }
    assert_eq!(expected.len(), got.len(), "{what}: number of lines");
    assert_eq!(bad, 0, "{what}: {bad} of {} lines differ", expected.len());
}

// ---------------------------------------------------------------------------------------------
// enums.txt
// ---------------------------------------------------------------------------------------------

fn enum_maps(name: &str) -> (&'static t_config_enum_values, &'static t_config_enum_names) {
    macro_rules! maps {
        ($($t:ident),* $(,)?) => {
            match name {
                $(stringify!($t) => (<enums::$t as ConfigEnum>::get_enum_values(), <enums::$t as ConfigEnum>::get_enum_names()),)*
                _ => panic!("enum {name} is not ported"),
            }
        };
    }
    maps!(
        PrinterTechnology, PrintHostType, AuthorizationType, GCodeFlavor, BedTempFormula, FuzzySkinType, NoiseType,
        FuzzySkinMode, InfillPattern, IroningType, ConformalStagger, ConformalPole, TopOneWallType, WallInfillOrder,
        WallSequence, EnsureVerticalThicknessLevel, PrintSequence, SlicingMode, SupportMaterialPattern,
        SupportMaterialStyle, SupportMaterialInterfacePattern, SupportType, SeamPosition, SeamScarfType,
        SLADisplayOrientation, SLAPillarConnectionMode, SLAMaterialSpeed, BrimType, TimelapseType, SideScrapeFace,
        DraftShield, ForwardCompatibilitySubstitutionRule, OverhangFanThreshold, OverhangThresholdParticipatingCooling,
        CoolingSlowdownLogicType, BedType, LayerSeq, NozzleType, FanDirection, PrinterStructure, PerimeterGeneratorType,
        ZHopType, ExtruderType, NozzleVolumeType, FilamentMapMode, PrimeVolumeMode, ReduceInfillRetractionMode,
        FilamentMetalStickiness, CounterboreHoleBridgingOption,
    )
}

#[test]
fn enum_static_maps() {
    let text = read_vectors("enums");
    let expected: Vec<String> = text.lines().map(|l| l.to_string()).collect();
    let mut got = Vec::new();
    for line in &expected {
        let name = line.split(' ').nth(1).expect("name");
        let (values, names) = enum_maps(name);
        let mut l = vec!["enum".to_string(), name.to_string(), values.len().to_string()];
        for (k, v) in values {
            l.push(hex(k));
            l.push(v.to_string());
        }
        push_list(&mut l, names);
        got.push(l.join(" "));
    }
    assert_eq!(expected.len(), 49);
    compare("enums", &expected, &got);
}

// ---------------------------------------------------------------------------------------------
// print_config_def.txt
// ---------------------------------------------------------------------------------------------

fn def_lines(d: &PrintConfigDef) -> Vec<String> {
    let mut out = Vec::new();
    let mut rank: BTreeMap<usize, usize> = BTreeMap::new();
    {
        let mut l = vec!["ordinals".to_string(), d.by_serialization_key_ordinal.len().to_string()];
        for (i, (ordinal, key)) in d.by_serialization_key_ordinal.iter().enumerate() {
            rank.insert(*ordinal, i);
            l.push(hex(key));
        }
        out.push(l.join(" "));
    }
    for (key, def) in &d.options {
        let mut l = vec![
            "def".to_string(),
            hex(key),
            (def.r#type as i32).to_string(),
            b(def.nullable),
            (def.gui_type as i32).to_string(),
            hex(&def.gui_flags),
            hex(&def.label),
            hex(&def.full_label),
            (def.printer_technology as i32).to_string(),
            hex(&def.category),
            hex(&def.tooltip),
            hex(&def.sidetext),
            hex(&def.cli),
            hex(&def.cli_params),
            hex(&def.ratio_over),
            b(def.multiline),
            b(def.full_width),
            b(def.is_code),
            b(def.readonly),
            def.height.to_string(),
            def.width.to_string(),
            dbl(def.min),
            dbl(def.max),
            dbl(def.max_literal),
            (def.mode as i32).to_string(),
        ];
        push_list(&mut l, &def.aliases);
        push_list(&mut l, &def.shortcut);
        push_list(&mut l, &def.enum_values);
        push_list(&mut l, &def.enum_labels);
        match def.enum_keys_map {
            None => l.push("-".to_string()),
            Some(m) => {
                l.push(m.len().to_string());
                for (k, v) in m {
                    l.push(hex(k));
                    l.push(v.to_string());
                }
            }
        }
        l.push(rank[&def.serialization_key_ordinal].to_string());
        match &def.default_value {
            None => l.push("-".to_string()),
            Some(dv) => {
                let cls = class_name(&**dv);
                l.push(cls.to_string());
                l.push((dv.r#type() as i32).to_string());
                l.push(b(dv.nullable()));
                l.push(b(dv.is_nil()));
                if cls == "EnumsGeneric" || cls == "EnumsGenericNullable" {
                    let v = dv.vector_values_any().and_then(|v| v.downcast_ref::<Vec<i32>>()).expect("ints");
                    l.push(v.len().to_string());
                    for x in v {
                        l.push(x.to_string());
                    }
                } else {
                    l.push(hex(&dv.serialize().expect("serialize")));
                }
            }
        }
        if (def.r#type == ConfigOptionType::CoEnum || def.r#type == ConfigOptionType::CoEnums) && def.enum_keys_map.is_none() {
            l.push("-".to_string());
        } else {
            let created = def.create_default_option().expect("create_default_option");
            l.push(class_name(&*created).to_string());
            l.push(hex(&created.serialize().expect("serialize")));
        }
        out.push(l.join(" "));
    }
    for (which, keys) in [
        ("extruder_option_keys", d.extruder_option_keys()),
        ("extruder_retract_keys", d.extruder_retract_keys()),
        ("filament_option_keys", d.filament_option_keys()),
        ("filament_retract_keys", d.filament_retract_keys()),
    ] {
        let mut l = vec!["keys".to_string(), which.to_string()];
        push_list(&mut l, keys);
        out.push(l.join(" "));
    }
    out
}

#[test]
fn print_config_def_options() {
    let text = read_vectors("print_config_def");
    let expected: Vec<String> = text.lines().map(|l| l.to_string()).collect();
    let got = def_lines(print_config_def());
    assert_eq!(expected.iter().filter(|l| l.starts_with("def ")).count(), 733);
    compare("print_config_def", &expected, &got);
}

// ---------------------------------------------------------------------------------------------
// functions.txt
// ---------------------------------------------------------------------------------------------

#[test]
fn handle_legacy() {
    let text = read_vectors("functions");
    let mut expected = Vec::new();
    let mut got = Vec::new();
    let mut n = 0usize;
    for line in text.lines() {
        let t: Vec<&str> = line.split(' ').collect();
        if t[0] != "legacy" {
            continue;
        }
        n += 1;
        let mut k = unhex(t[1]);
        let mut v = unhex(t[2]);
        PrintConfigDef::handle_legacy(&mut k, &mut v);
        expected.push(line.to_string());
        got.push(format!("legacy {} {} => {} {}", t[1], t[2], hex(&k), hex(&v)));
    }
    assert!(n > 900, "{n} handle_legacy cases");
    compare("handle_legacy", &expected, &got);
}

// ---------------------------------------------------------------------------------------------
// functions.txt: the free functions and DynamicPrintConfig (gen.cpp gen_functions()).
// ---------------------------------------------------------------------------------------------

mod functions {
    use super::*;
    use slicer::print_config::config_def::*;
    use slicer::print_config::enums::{
        BedType, ExtruderType, FilamentMapMode, NozzleVolumeType, SupportMaterialStyle, SupportType,
    };

    fn nvt(v: i32) -> NozzleVolumeType {
        NozzleVolumeType::from_i32(v)
    }
    fn et(v: i32) -> ExtruderType {
        <ExtruderType as ConfigEnum>::from_int(v)
    }
    fn strs(v: &[&str]) -> Vec<String> {
        v.iter().map(|s| s.to_string()).collect()
    }
    fn lst(l: &mut Vec<String>, v: &[String]) {
        push_list(l, v);
    }

    /// gen.cpp make_config()
    fn make_config(sc: &[(String, String)]) -> DynamicPrintConfig {
        let mut cfg = DynamicPrintConfig::new();
        for (k, v) in sc {
            if let Some(k) = k.strip_prefix('!') {
                let mut o = ConfigOptionStrings::new();
                o.deserialize(v, false).expect("deserialize");
                cfg.set_key_value(k, Box::new(o));
            } else {
                cfg.set_deserialize_strict(k, v, false).expect("set_deserialize_strict");
            }
        }
        cfg
    }

    /// gen.cpp push_config()
    fn push_config(l: &mut Vec<String>, cfg: &DynamicPrintConfig) {
        let keys = cfg.keys();
        l.push(keys.len().to_string());
        for k in &keys {
            l.push(hex(k));
            match cfg.option(k).unwrap().serialize() {
                Ok(s) => l.push(hex(&s)),
                Err(e) => l.push(format!("!{}", hex(e.what()))),
            }
        }
    }

    fn nvts(v: &[NozzleVolumeType]) -> String {
        let mut s = v.len().to_string();
        for t in v {
            s += &format!(",{}", *t as i32);
        }
        s
    }

    fn pure_lines(out: &mut Vec<String>) {
        let mut emit = |l: Vec<String>| out.push(l.join(" "));
        {
            let counts: Vec<Vec<i32>> = vec![vec![], vec![1], vec![2, 0], vec![1, 1], vec![-1, 3], vec![6]];
            let diams: Vec<Vec<f64>> = vec![vec![], vec![0.4], vec![0.4, 0.6], vec![0.2, 0.4, 0.6, 0.8, 0.4, 0.4], vec![0.4, 0.4, 0.4]];
            for h in 0..2 {
                for c in 0..counts.len() {
                    for d in 0..diams.len() {
                        emit(vec![
                            "fn".into(),
                            "has_authoritative_per_slot_nozzle_diameters".into(),
                            h.to_string(),
                            c.to_string(),
                            d.to_string(),
                            "=>".into(),
                            b(has_authoritative_per_slot_nozzle_diameters(h != 0, &counts[c], &diams[d])),
                        ]);
                    }
                }
            }
        }
        for d in [0.2, 0.4, 0.6, 0.8, 0.2005, 0.199, 0.401, 0.3, 1.0, 0.0, 0.4009, 0.39901, 0.8010, -0.2] {
            emit(vec![
                "fn".into(),
                "multi_nozzle_double".into(),
                dbl(d),
                "=>".into(),
                dbl(multi_nozzle_retraction_length(d, 9.5)),
                dbl(multi_nozzle_ironing_inset(d, 9.5)),
                dbl(multi_nozzle_top_shell_thickness(d, 9.5)),
                dbl(multi_nozzle_bottom_shell_thickness(d, 9.5)),
                dbl(multi_nozzle_support_z_distance(d, 9.5)),
                dbl(multi_nozzle_prime_tower_line_width(d, 9.5)),
                dbl(multi_nozzle_prime_tower_ramming_line_width(d, 9.5)),
                dbl(multi_nozzle_prime_tower_feedrate_factor(d, 9.5)),
            ]);
            emit(vec![
                "fn".into(),
                "multi_nozzle_int".into(),
                dbl(d),
                "=>".into(),
                multi_nozzle_wall_loops(d, 99).to_string(),
                multi_nozzle_top_shell_layers(d, 99).to_string(),
                multi_nozzle_bottom_shell_layers(d, 99).to_string(),
                multi_nozzle_top_color_penetration_layers(d, 99).to_string(),
                multi_nozzle_bottom_color_penetration_layers(d, 99).to_string(),
            ]);
        }
        for k in [
            "retraction_length", "min_layer_height", "max_layer_height", "ironing_inset", "wall_loops", "top_shell_layers",
            "bottom_shell_layers", "top_shell_thickness", "bottom_shell_thickness", "top_color_penetration_layers",
            "bottom_color_penetration_layers", "support_top_z_distance", "support_bottom_z_distance", "prime_tower_feedrate_factor",
            "prime_tower_line_width", "prime_tower_ramming_line_width", "wall_loops#2", "retraction_length#", "#wall_loops",
            "layer_height", "", "#", "wall_loops#1#2", "Wall_loops",
        ] {
            emit(vec!["fn".into(), "is_multi_nozzle_auto_option".into(), hex(k), "=>".into(), b(is_multi_nozzle_auto_option(k))]);
        }
        for t in 0..4 {
            let st_ = SupportType::from_i32(t);
            emit(vec!["fn".into(), "is_tree".into(), t.to_string(), "=>".into(), b(is_tree(st_)), b(is_auto(st_))]);
            for st in 0..7 {
                emit(vec![
                    "fn".into(),
                    "is_tree_slim".into(),
                    t.to_string(),
                    st.to_string(),
                    "=>".into(),
                    b(is_tree_slim(st_, SupportMaterialStyle::from_i32(st))),
                ]);
            }
        }
        for m in 0..6 {
            emit(vec!["fn".into(), "is_auto_filament_map_mode".into(), m.to_string(), "=>".into(), b(is_auto_filament_map_mode(FilamentMapMode::from_i32(m)))]);
        }
        for v in 0..=NozzleVolumeType::NvtCount as i32 {
            let mut l = vec![
                "fn".into(),
                "nozzle_volume_type".into(),
                v.to_string(),
                "=>".into(),
                (legacy_fallback_nozzle_volume_type(nvt(v)) as i32).to_string(),
                hex(&get_nozzle_volume_type_string(nvt(v))),
            ];
            for e in 0..2 {
                let name = get_extruder_variant_string(et(e), nvt(v));
                if v < NozzleVolumeType::NvtCount as i32 {
                    let volume = slicer::extruder::NozzleVolumeType::from_i32(v);
                    assert_eq!(slicer::extruder::get_extruder_variant_string(et(e), volume), name);
                }
                l.push(hex(&name));
            }
            emit(l);
        }
        {
            let mut l = vec!["fn".into(), "get_valid_nozzle_volume_type".into(), "=>".into()];
            let valid = get_valid_nozzle_volume_type();
            assert_eq!(slicer::extruder::get_valid_nozzle_volume_type(), valid.iter().map(|&v| v as i32).collect());
            for t in valid {
                l.push((t as i32).to_string());
            }
            emit(l);
        }
        for bt in 0..=BedType::BtCount as i32 {
            let t = BedType::from_i32(bt);
            emit(vec![
                "fn".into(),
                "bed_type".into(),
                bt.to_string(),
                "=>".into(),
                hex(&bed_type_to_gcode_string(t)),
                hex(&get_bed_temp_key(t)),
                hex(&get_bed_temp_1st_layer_key(t)),
            ]);
        }
        {
            let variants =
                strs(&["Direct Drive Standard", "Direct Drive High Flow", "Direct Drive Standard", "Bowden Standard", "Direct Drive High Flow"]);
            let ids = vec![1, 1, 2, 2, 3];
            for v in 0..NozzleVolumeType::NvtCount as i32 {
                for e in 0..2 {
                    for id in 0..=4 {
                        let row = get_config_index_base(nvt(v), et(e), id, &variants, &ids);
                        // The extruder and pre-cooling paths must return the same native row
                        // for every nozzle type their representation accepts.
                        if v <= slicer::extruder::NVT_MAX_NOZZLE_VOLUME_TYPE {
                            let volume = slicer::extruder::NozzleVolumeType::from_i32(v);
                            assert_eq!(slicer::extruder::get_config_index_base(volume, et(e), id, &variants, &ids),
                                       row, "volume {v}, extruder {e}, owner {id}");
                        }
                        emit(vec![
                            "fn".into(),
                            "get_config_index_base".into(),
                            v.to_string(),
                            e.to_string(),
                            id.to_string(),
                            "=>".into(),
                            row.to_string(),
                        ]);
                    }
                }
            }
        }
        for a in -1..=4 {
            emit(vec![
                "fn".into(),
                "ams".into(),
                a.to_string(),
                "=>".into(),
                hex(&get_ams_type_name(a)),
                hex(&get_ams_type_display_name(a)),
                hex(&get_ams_load_time_key(a)),
                hex(&get_ams_unload_time_key(a)),
            ]);
        }
        {
            let mut l = vec!["fn".into(), "get_ams_time_types".into(), "=>".into()];
            for t in get_ams_time_types() {
                l.push(t.to_string());
            }
            emit(l);
        }
        for names in [vec![], strs(&["AMS"]), strs(&["N3F_S", "AMS", "AMS"]), strs(&["AMS_LITE", "ams", "X", "N3F_S", "AMS_LITE"]), strs(&[""])] {
            let mut l = vec!["fn".into(), "get_supported_ams_time_types".into()];
            lst(&mut l, &names);
            l.push("=>".into());
            for t in get_supported_ams_time_types(&names) {
                l.push(t.to_string());
            }
            emit(l);
        }
        for s in [vec![], strs(&[""]), strs(&["1#0|4#1"]), strs(&["4#2|1#3", "", "2#5|2#7"]), strs(&[" 3#+4", "-1#-2|0#0"])] {
            let mut l = vec!["fn".into(), "get_extruder_ams_count".into()];
            lst(&mut l, &s);
            l.push("=>".into());
            for m in get_extruder_ams_count(&s) {
                let mut e = m.len().to_string();
                for (k, v) in &m {
                    e += &format!(",{}:{}", k, v);
                }
                l.push(e);
            }
            lst(&mut l, &save_extruder_ams_count_to_string(&get_extruder_ams_count(&s)));
            emit(l);
        }
        for s in [
            vec![],
            strs(&[""]),
            strs(&["Standard#1|High Flow#2"]),
            strs(&["TPU285#1|Bogus#2|Standard#3", "E3D High Flow#x"]),
            strs(&["Hybrid#1|malformed|#5|Standard#-1", "High Flow# 7|High Flow#2"]),
            strs(&["Standard"]),
        ] {
            let mut l = vec!["fn".into(), "get_extruder_nozzle_stats".into()];
            lst(&mut l, &s);
            l.push("=>".into());
            let stats = get_extruder_nozzle_stats(&s);
            for m in &stats {
                let mut e = m.len().to_string();
                for (k, v) in m {
                    e += &format!(",{}:{}", *k as i32, v);
                }
                l.push(e);
            }
            let saved = save_extruder_nozzle_stats_to_string(&stats);
            let extruder_stats = slicer::print_config::get_extruder_nozzle_stats(&s);
            assert_eq!(slicer::print_config::save_extruder_nozzle_stats_to_string(&extruder_stats), saved);
            lst(&mut l, &saved);
            emit(l);
        }
        for n in ["Standard", "High Flow", "TPU High Flow", "TPU285", "Hybrid", "E3D High Flow", "standard", "", "Normal"] {
            emit(vec!["fn".into(), "nozzle_volume_type_from_string".into(), hex(n), "=>".into(), (nozzle_volume_type_from_string(n) as i32).to_string()]);
        }
        let variants = strs(&[
            "Direct Drive Standard", "Bowden High Flow", " Direct Drive TPU High Flow\t", "Direct DriveTPU285", "Direct Drive Hybrid",
            "Bowden E3D High Flow", "Standard", "", "Direct Drive", "Bowden Direct Drive Standard", "Direct Drive Standard Bowden",
            "High Flow Direct Drive", "Direct Drive  Standard ", "Bowden\nStandard",
        ]);
        for v in &variants {
            let volume = convert_to_nvt_type(v) as i32;
            assert_eq!(slicer::print_config::convert_to_nvt_type(v) as i32, volume);
            emit(vec!["fn".into(), "convert_to_nvt_type".into(), hex(v), "=>".into(), volume.to_string()]);
        }
        {
            let lists = vec![
                vec![],
                strs(&["Direct Drive Standard"]),
                strs(&["Direct Drive TPU285"]),
                strs(&["Bowden High Flow", "Direct Drive TPU285"]),
                strs(&["Direct Drive E3D High Flow", "x"]),
            ];
            for v in 0..NozzleVolumeType::NvtCount as i32 {
                for i in 0..lists.len() {
                    for c in 0..2 {
                        emit(vec![
                            "fn".into(),
                            "is_nozzle_printable_for_filament".into(),
                            v.to_string(),
                            i.to_string(),
                            c.to_string(),
                            "=>".into(),
                            b(is_nozzle_printable_for_filament(nvt(v), &lists[i], c != 0)),
                        ]);
                    }
                }
            }
        }
    }

    fn scenario_lines(si: usize, sc: &[(String, String)], out: &mut Vec<String>) {
        let mut emit = |l: Vec<String>| out.push(l.join(" "));
        {
            let mut l = vec!["scenario".to_string(), si.to_string(), sc.len().to_string()];
            for (k, v) in sc {
                l.push(hex(k));
                l.push(hex(v));
            }
            emit(l);
        }
        let cfg = make_config(sc);
        let s = si.to_string();
        emit(vec![
            "fn".into(),
            "config".into(),
            s.clone(),
            "=>".into(),
            b(multi_nozzle_diameter_mode_enabled(&cfg)),
            hex(&printer_nozzle_diameter_mode_tag(&cfg)),
            b(has_multiple_configured_nozzle_diameters(&cfg)),
            dbl(layer_grid_nozzle_diameter(&cfg)),
            layer_grid_subdivision(&cfg).to_string(),
            dbl(primary_nozzle_diameter(&cfg)),
            get_effective_ams_nozzle_idx(&cfg).to_string(),
            b(is_single_ext_multi_nozzle(&cfg)),
            dbl(min_object_distance(&cfg)),
        ]);
        {
            let mut l = vec!["fn".into(), "ams_times".into(), s.clone(), "=>".into()];
            for t in get_ams_load_times(&cfg) {
                l.push(dbl(t));
            }
            l.push("|".into());
            for t in get_ams_unload_times(&cfg) {
                l.push(dbl(t));
            }
            emit(l);
        }
        for e in -1..=3 {
            let mut l = vec![
                "fn".into(),
                "default_nozzle_volume".into(),
                s.clone(),
                e.to_string(),
                "=>".into(),
                (get_extruder_default_nozzle_volume_type(&cfg, e) as i32).to_string(),
            ];
            for n in -1..=6 {
                l.push((get_default_nozzle_volume_type(&cfg, e, n) as i32).to_string());
            }
            let mut sup = String::new();
            for t in get_extruder_supported_nozzle_volume_types(&cfg, e) {
                sup += &format!("{},", t as i32);
            }
            l.push(if sup.is_empty() { "-".into() } else { sup });
            emit(l);
        }
        for v in ["Direct Drive Standard", "Direct Drive High Flow", "", "Bowden Standard"] {
            emit(vec!["fn".into(), "find_printer_variant_index".into(), s.clone(), hex(v), "=>".into(), find_printer_variant_index(&cfg, v).to_string()]);
        }
        {
            let mut l = vec!["fn".into(), "collect_aux_fan_slots".into(), s.clone(), "=>".into()];
            for a in collect_aux_fan_slots(&cfg) {
                l.push(b(a.optional));
                l.push(a.gcode_index.to_string());
                l.push(b(a.independent));
            }
            emit(l);
        }
        let c = cfg.clone();
        for k in ["retraction_length", "machine_max_speed_x", "filament_flow_ratio", "outer_wall_speed", "nozzle_diameter", "x"] {
            emit(vec!["fn".into(), "get_parameter_size".into(), s.clone(), hex(k), "=>".into(), c.get_parameter_size(k, 3).to_string()]);
        }
        for id in 0..4 {
            let mut displayed = "?".to_string();
            let t = c.get_filament_type_displayed(&mut displayed, id);
            emit(vec!["fn".into(), "get_filament_type".into(), s.clone(), id.to_string(), "=>".into(), hex(&t), hex(&displayed)]);
        }
        {
            let mut ec = -7;
            let sd = c.support_different_extruders(&mut ec);
            emit(vec![
                "fn".into(),
                "extruders".into(),
                s.clone(),
                "=>".into(),
                b(c.is_using_different_extruders()),
                b(sd),
                ec.to_string(),
                hex(&cfg.get_filament_vendor()),
                hex(&cfg.get_filament_type()),
            ]);
        }
        for ec in 1..=2 {
            let mut types: Vec<Vec<NozzleVolumeType>> = vec![vec![NozzleVolumeType::NvtHybrid]];
            let n = cfg.get_extruder_nozzle_volume_count(ec, &mut types);
            let mut l = vec!["fn".into(), "get_extruder_nozzle_volume_count".into(), s.clone(), ec.to_string(), "=>".into(), n.to_string()];
            for t in &types {
                l.push(nvts(t));
            }
            emit(l);
        }
        for vn in ["printer_extruder_variant", "print_extruder_variant", "filament_extruder_variant"] {
            for idn in ["", "printer_extruder_id", "filament_self_index"] {
                for id in 0..=2 {
                    for e in 0..2 {
                        for v in 0..NozzleVolumeType::NvtCount as i32 {
                            emit(vec![
                                "fn".into(),
                                "get_index_for_extruder".into(),
                                s.clone(),
                                hex(vn),
                                hex(idn),
                                id.to_string(),
                                e.to_string(),
                                v.to_string(),
                                "=>".into(),
                                cfg.get_index_for_extruder(id, idn, et(e), nvt(v), vn, 2).to_string(),
                            ]);
                        }
                    }
                }
            }
        }

        let mut after = |name: &str, f: &dyn Fn(&mut DynamicPrintConfig) -> Result<String, ConfigurationError>| {
            let mut c = cfg.clone();
            let mut l = vec!["fn".to_string(), name.to_string(), s.clone(), "=>".to_string()];
            match f(&mut c) {
                Ok(r) => l.push(r),
                Err(e) => l.push(format!("!{}", hex(e.what()))),
            }
            push_config(&mut l, &c);
            emit(l);
        };
        after("sync_mixed_filament_nozzle_diameters", &|c| {
            sync_mixed_filament_nozzle_diameters(c);
            Ok("-".into())
        });
        for sup in [vec![], strs(&["0.4;0.6", "", "0.8", "0.2;0.4;abc", "0.6", "0.2"])] {
            after("complete_filament_nozzle_diameter_selections", &|c| Ok(b(complete_filament_nozzle_diameter_selections(c, &sup))));
        }
        after("sync_nozzle_volume_type_to_extruder_count_0", &|c| {
            sync_nozzle_volume_type_to_extruder_count(c, false);
            Ok("-".into())
        });
        after("sync_nozzle_volume_type_to_extruder_count_1", &|c| {
            sync_nozzle_volume_type_to_extruder_count(c, true);
            Ok("-".into())
        });
        after("split_nozzle_stats_for_export", &|c| {
            split_nozzle_stats_for_export(c);
            Ok("-".into())
        });
        after("repair_invalid_filament_extrusion_parameters", &|c| {
            c.repair_invalid_filament_extrusion_parameters();
            Ok("-".into())
        });
        after("normalize_fdm", &|c| {
            c.normalize_fdm()?;
            Ok("-".into())
        });
        after("normalize_fdm_1", &|c| {
            c.normalize_fdm_1()?;
            Ok("-".into())
        });
        after("normalize_fdm_2", &|c| {
            let mut r = String::new();
            for k in c.normalize_fdm_2(1, 1, None)? {
                r += &k;
                r += ";";
            }
            r += "|";
            let mut ori = DynamicConfig::new();
            for k in c.normalize_fdm_2(2, 2, Some(&mut ori))? {
                r += &k;
                r += ";";
            }
            r += "|";
            for k in c.normalize_fdm_2(2, 1, Some(&mut ori))? {
                r += &k;
                r += ";";
            }
            r += "|";
            for k in c.normalize_fdm_2(1, 3, Some(&mut ori))? {
                r += &k;
                r += ";";
            }
            r += &format!("|{}", ori.size());
            Ok(hex(&r))
        });
        after("handle_legacy_sla", &|c| {
            handle_legacy_sla(c)?;
            Ok("-".into())
        });
    }

    /// gen.cpp push_config_classes()
    fn push_config_classes(l: &mut Vec<String>, cfg: &DynamicPrintConfig) {
        let keys = cfg.keys();
        l.push(keys.len().to_string());
        for k in &keys {
            let o = cfg.option(k).unwrap();
            let cls = class_name(o);
            l.push(hex(k));
            l.push(cls.to_string());
            let no_map = match cls {
                "EnumsGeneric" => o.as_any().downcast_ref::<ConfigOptionEnumsGeneric>().unwrap().keys_map.is_none(),
                "EnumsGenericNullable" => o.as_any().downcast_ref::<ConfigOptionEnumsGenericNullable>().unwrap().keys_map.is_none(),
                _ => false,
            };
            if no_map {
                let v = o.vector_values_any().and_then(|v| v.downcast_ref::<Vec<i32>>()).unwrap();
                let mut e = format!("*{}", v.len());
                for x in v {
                    e += &format!(",{}", x);
                }
                l.push(e);
            } else {
                match o.serialize() {
                    Ok(s) => l.push(hex(&s)),
                    Err(e) => l.push(format!("!{}", hex(e.what()))),
                }
            }
        }
    }

    /// gen.cpp gen_static_configs()
    fn static_lines(out: &mut Vec<String>) {
        for name in [
            "PrintObjectConfig", "PrintRegionConfig", "MachineEnvelopeConfig", "GCodeConfig", "PrintConfig", "FullPrintConfig",
            "SLAPrintConfig", "SLAPrintObjectConfig", "SLAMaterialConfig", "SLAPrinterConfig", "SLAFullPrintConfig",
        ] {
            let mut l = vec!["static".to_string(), name.to_string()];
            push_config_classes(&mut l, static_print_config_defaults(name));
            out.push(l.join(" "));
        }
        {
            let mut l = vec!["fn".to_string(), "full_print_config".to_string(), "=>".to_string()];
            push_config_classes(&mut l, &DynamicPrintConfig::full_print_config());
            out.push(l.join(" "));
        }
        let mut keys: Vec<String> = print_config_def().extruder_option_keys().clone();
        for k in [
            "printer_extruder_variant", "filament_extruder_variant", "print_extruder_variant", "brim_type", "printhost_apikey",
            "z_hop_types", "nozzle_volume_type", "machine_max_speed_x", "filament_flow_ratio",
        ] {
            keys.push(k.to_string());
        }
        let base = DynamicPrintConfig::new_from_defaults_keys(&keys).expect("new_from_defaults_keys");
        {
            let mut l = vec!["fn".to_string(), "new_from_defaults_keys".to_string(), "=>".to_string()];
            push_config_classes(&mut l, &base);
            out.push(l.join(" "));
        }
        for variants in ["", "\"Direct Drive Standard\";\"Direct Drive High Flow\";\"Bowden Standard\""] {
            let mut with = (*base).clone();
            if !variants.is_empty() {
                with.set_deserialize_strict("printer_extruder_variant", variants, false).unwrap();
                with.set_deserialize_strict("filament_extruder_variant", variants, false).unwrap();
                with.set_deserialize_strict("retraction_length", "1,2", false).unwrap();
                with.set_deserialize_strict("nozzle_diameter", "0.4,0.6,0.8,1.0", false).unwrap();
            }
            for n in [0u32, 1, 2, 3] {
                let mut c = with.clone();
                let mut l = vec!["fn".to_string(), "set_num_extruders".to_string(), hex(variants), n.to_string(), "=>".to_string()];
                match c.set_num_extruders(n).and_then(|_| c.set_num_filaments(n)) {
                    Ok(()) => l.push("-".to_string()),
                    Err(e) => l.push(format!("!{}", hex(e.what()))),
                }
                push_config_classes(&mut l, &c);
                out.push(l.join(" "));
            }
        }
    }

    /// gen.cpp gen_update_values()
    #[test]
    fn update_values_from_single_and_multi_to_multi() {
        let text = read_vectors("functions");
        let mut ps: Vec<Vec<(String, String)>> = Vec::new();
        let mut expected = Vec::new();
        for line in text.lines() {
            let t: Vec<&str> = line.split(' ').collect();
            if t[0] == "process" {
                let n: usize = t[2].parse().unwrap();
                ps.push((0..n).map(|i| (unhex(t[3 + 2 * i]), unhex(t[4 + 2 * i]))).collect());
            } else if line.starts_with("fn update_values_from_") {
                expected.push(line.to_string());
            }
        }
        let key_set: std::collections::BTreeSet<String> = [
            "outer_wall_speed", "inner_wall_speed", "small_perimeter_speed", "sparse_infill_acceleration", "enable_overhang_speed",
            "print_extruder_id", "print_extruder_variant", "top_solid_infill_flow_ratio", "wall_loops", "not_a_key", "filament_type",
        ]
        .iter()
        .map(|s| s.to_string())
        .collect();
        let new_variants: Vec<Vec<String>> = vec![
            strs(&["Direct Drive Standard", "Direct Drive High Flow"]),
            strs(&["Direct Drive High Flow", "Direct Drive High Flow"]),
            strs(&["Bowden Standard"]),
            vec![],
        ];
        let mut got = Vec::new();
        for a in 0..ps.len() {
            for b2 in 0..ps.len() {
                {
                    let mut c = make_config(&ps[a]);
                    let multi = make_config(&ps[b2]);
                    let mut l = vec!["fn".to_string(), "update_values_from_single_to_multi".into(), a.to_string(), b2.to_string(), "=>".into()];
                    match c.update_values_from_single_to_multi(&multi, &key_set, "print_extruder_id", "print_extruder_variant") {
                        Ok(r) => l.push(r.to_string()),
                        Err(e) => l.push(format!("!{}", hex(e.what()))),
                    }
                    push_config(&mut l, &c);
                    got.push(l.join(" "));
                }
                {
                    let mut c = make_config(&ps[a]);
                    let dst = make_config(&ps[b2]);
                    let ks: std::collections::BTreeSet<String> = [
                        "outer_wall_speed", "inner_wall_speed", "small_perimeter_speed", "sparse_infill_acceleration", "enable_overhang_speed",
                        "wall_loops", "print_extruder_id",
                    ]
                    .iter()
                    .map(|s| s.to_string())
                    .collect();
                    let src_v: Vec<String> = c.opt::<ConfigOptionStrings>("print_extruder_variant").unwrap().values.clone();
                    let dst_v: Vec<String> = dst.opt::<ConfigOptionStrings>("print_extruder_variant").unwrap().values.clone();
                    let mut l = vec!["fn".to_string(), "update_values_from_multi_to_multi_2".into(), a.to_string(), b2.to_string(), "=>".into()];
                    match c.update_values_from_multi_to_multi_2(&src_v, &dst_v, &dst, &ks) {
                        Ok(r) => l.push(r.to_string()),
                        Err(e) => l.push(format!("!{}", hex(e.what()))),
                    }
                    push_config(&mut l, &c);
                    got.push(l.join(" "));
                }
                for v in 0..new_variants.len() {
                    let mut c = make_config(&ps[a]);
                    let nc = make_config(&ps[b2]);
                    let mut l = vec![
                        "fn".to_string(),
                        "update_values_from_multi_to_multi".into(),
                        a.to_string(),
                        b2.to_string(),
                        v.to_string(),
                        "=>".into(),
                    ];
                    match c.update_values_from_multi_to_multi(&nc, &key_set, "print_extruder_id", "print_extruder_variant", &new_variants[v]) {
                        Ok(r) => l.push(r.to_string()),
                        Err(e) => l.push(format!("!{}", hex(e.what()))),
                    }
                    push_config(&mut l, &c);
                    got.push(l.join(" "));
                }
            }
        }
        assert_eq!(expected.len(), 96);
        compare("update_values", &expected, &got);
    }

    fn ints_str(v: &[i32]) -> String {
        let mut s = v.len().to_string();
        for x in v {
            s += &format!(",{}", x);
        }
        s
    }

    fn sc(pairs: &[(&str, &str)]) -> Vec<(String, String)> {
        pairs.iter().map(|(k, v)| (k.to_string(), v.to_string())).collect()
    }

    /// gen.cpp gen_overrides()
    #[test]
    fn override_helpers() {
        let text = read_vectors("functions");
        let mut ps: Vec<Vec<(String, String)>> = Vec::new();
        let mut align: Vec<Vec<(String, String)>> = Vec::new();
        let mut expected = Vec::new();
        for line in text.lines() {
            let t: Vec<&str> = line.split(' ').collect();
            if t[0] == "process" || t[0] == "overrides" {
                let n: usize = t[2].parse().unwrap();
                let v: Vec<(String, String)> = (0..n).map(|i| (unhex(t[3 + 2 * i]), unhex(t[4 + 2 * i]))).collect();
                if t[0] == "process" {
                    ps.push(v)
                } else {
                    align.push(v)
                }
            } else if line.starts_with("fn update_static_print_config_from_dynamic ")
                || line.starts_with("fn update_non_diff_values_to_base_config ")
                || line.starts_with("fn update_diff_values_to_child_config ")
                || line.starts_with("fn align_filament_variant_overrides ")
                || line.starts_with("fn filament_override_default_indices ")
                || line.starts_with("fn compute_filament_override_value ")
            {
                expected.push(line.to_string());
            }
        }
        let mut got: Vec<String> = Vec::new();
        let key_set1: std::collections::BTreeSet<String> =
            ["outer_wall_speed", "inner_wall_speed", "small_perimeter_speed", "enable_overhang_speed", "print_extruder_variant"]
                .iter()
                .map(|s| s.to_string())
                .collect();
        let vis: Vec<Vec<i32>> = vec![vec![], vec![1, 0], vec![0], vec![3, -1]];
        for a in 0..ps.len() {
            for b2 in 0..ps.len() {
                for v in 0..vis.len() {
                    let mut c = make_config(&ps[a]);
                    let dest = make_config(&ps[b2]);
                    let mut l = vec![
                        "fn".to_string(),
                        "update_static_print_config_from_dynamic".into(),
                        a.to_string(),
                        b2.to_string(),
                        v.to_string(),
                        "=>".into(),
                    ];
                    match update_static_print_config_from_dynamic(&mut c, &dest, vis[v].clone(), &key_set1, 1) {
                        Ok(()) => l.push("-".into()),
                        Err(e) => l.push(format!("!{}", hex(e.what()))),
                    }
                    push_config(&mut l, &c);
                    got.push(l.join(" "));
                }
            }
        }
        {
            let set = |v: &[&str]| -> std::collections::BTreeSet<String> { v.iter().map(|s| s.to_string()).collect() };
            let different = set(&["outer_wall_speed", "small_perimeter_speed", "enable_overhang_speed", "inner_wall_speed"]);
            let ks1 = set(&["outer_wall_speed", "small_perimeter_speed", "enable_overhang_speed"]);
            let ks2s = vec![set(&[]), set(&["inner_wall_speed"])];
            for a in 0..ps.len() {
                for b2 in 0..ps.len() {
                    for k2 in 0..ks2s.len() {
                        for idn in ["print_extruder_id", ""] {
                            {
                                let mut c = make_config(&ps[a]);
                                let mut nc = make_config(&ps[b2]);
                                let keys = nc.keys();
                                let mut l = vec![
                                    "fn".to_string(),
                                    "update_non_diff_values_to_base_config".into(),
                                    a.to_string(),
                                    b2.to_string(),
                                    k2.to_string(),
                                    hex(idn),
                                    "=>".into(),
                                ];
                                match c.update_non_diff_values_to_base_config(&mut nc, &keys, &different, idn, "print_extruder_variant", &ks1, &ks2s[k2]) {
                                    Ok(()) => l.push("-".into()),
                                    Err(e) => l.push(format!("!{}", hex(e.what()))),
                                }
                                push_config(&mut l, &c);
                                got.push(l.join(" "));
                            }
                            {
                                let mut c = make_config(&ps[a]);
                                let mut nc = make_config(&ps[b2]);
                                let mut l = vec![
                                    "fn".to_string(),
                                    "update_diff_values_to_child_config".into(),
                                    a.to_string(),
                                    b2.to_string(),
                                    k2.to_string(),
                                    hex(idn),
                                    "=>".into(),
                                ];
                                match c.update_diff_values_to_child_config(&mut nc, idn, "print_extruder_variant", &ks1, &ks2s[k2]) {
                                    Ok(()) => l.push("-".into()),
                                    Err(e) => l.push(format!("!{}", hex(e.what()))),
                                }
                                push_config(&mut l, &c);
                                got.push(l.join(" "));
                            }
                        }
                    }
                }
            }
        }
        for a in 0..align.len() {
            let mut c = make_config(&align[a]);
            let mut l = vec!["fn".to_string(), "align_filament_variant_overrides".into(), a.to_string(), "=>".into()];
            match align_filament_variant_overrides(&mut c) {
                Ok(()) => l.push("-".into()),
                Err(e) => l.push(format!("!{}", hex(e.what()))),
            }
            push_config(&mut l, &c);
            got.push(l.join(" "));
        }
        let fc = make_config(&sc(&[
            ("filament_self_index", "1,1,2,2,3"),
            (
                "filament_extruder_variant",
                "\"Direct Drive Standard\";\"Direct Drive High Flow\";\"Direct Drive Standard\";\"Bowden High Flow\";\"Direct Drive High Flow\"",
            ),
            ("printer_extruder_variant", "\"Direct Drive Standard\";\"Direct Drive High Flow\";\"Bowden Standard\""),
        ]));
        let fmaps: Vec<Vec<i32>> = vec![vec![], vec![1, 0, 2], vec![2, 2, 2, 2], vec![-1, 5]];
        for k in ["retraction_length", "z_hop", "wall_loops", "retract_restart_extra"] {
            for rows in [5usize, 3] {
                for m in 0..fmaps.len() {
                    got.push(
                        vec![
                            "fn".to_string(),
                            "filament_override_default_indices".into(),
                            hex(k),
                            rows.to_string(),
                            m.to_string(),
                            "=>".into(),
                            ints_str(&filament_override_default_indices(k, rows, &fc, &fmaps[m])),
                        ]
                        .join(" "),
                    );
                }
            }
        }
        let cases: Vec<(&str, &str, &str, &str, &str)> = vec![
            ("retraction_length", "FloatsNullable", "0.8,0.8", "0.8,1.0", "nil,1.5"),
            ("retraction_length", "FloatsNullable", "0.8,0.8", "0.8,0.8", "nil,nil"),
            ("retraction_length", "FloatsNullable", "0.4,0.8", "0.8,1.0", "0.4,nil"),
            ("z_hop", "FloatsNullable", "0.4,0.4", "0.4,0.4", "0.4,nil"),
            ("z_hop", "FloatsNullable", "0.4,0.4", "0.2,0.6", "nil,0.3"),
            ("long_retractions_when_cut", "BoolsNullable", "1,0", "1,0", "nil,1"),
            ("long_retractions_when_cut", "BoolsNullable", "0,0", "1,0", "0,nil"),
            ("retraction_distances_when_cut", "FloatsNullable", "18,18", "18,18", "10,nil"),
            ("retraction_distances_when_cut", "FloatsNullable", "18,18", "20,18", "nil,nil"),
        ];
        for longret in ["0", "2"] {
            for multi in ["0", "1"] {
                let nfc = make_config(&sc(&[
                    ("enable_long_retraction_when_cut", longret),
                    ("multi_nozzle_diameter_mode", multi),
                    ("filament_self_index", "1,2"),
                    ("filament_diameter_map", "\"0.2\";\"0.6\""),
                    ("filament_extruder_variant", "\"Direct Drive Standard\";\"Direct Drive High Flow\""),
                    ("printer_extruder_variant", "\"Direct Drive Standard\";\"Direct Drive High Flow\""),
                ]));
                for (ci, oc) in cases.iter().enumerate() {
                    let mk = || -> Box<dyn ConfigOption> {
                        if oc.1 == "FloatsNullable" {
                            Box::new(ConfigOptionFloatsNullable::new())
                        } else {
                            Box::new(ConfigOptionBoolsNullable::new())
                        }
                    };
                    let (mut om, mut nm, mut nf) = (mk(), mk(), mk());
                    om.deserialize(oc.2, false).unwrap();
                    nm.deserialize(oc.3, false).unwrap();
                    nf.deserialize(oc.4, false).unwrap();
                    let mut diff_keys: Vec<String> = Vec::new();
                    let mut overrides = DynamicPrintConfig::new();
                    let mut fmap = vec![0, 1];
                    let mut l = vec!["fn".to_string(), "compute_filament_override_value".into(), hex(longret), hex(multi), ci.to_string(), "=>".into()];
                    match compute_filament_override_value(oc.0, &*om, &*nm, &*nf, &nfc, &mut diff_keys, &mut overrides, &mut fmap) {
                        Ok(()) => l.push("-".into()),
                        Err(e) => l.push(format!("!{}", hex(e.what()))),
                    }
                    lst(&mut l, &diff_keys);
                    push_config(&mut l, &overrides);
                    l.push(ints_str(&fmap));
                    got.push(l.join(" "));
                }
            }
        }
        assert_eq!(expected.len(), 266);
        compare("overrides", &expected, &got);
    }

    #[test]
    fn static_config_defaults_and_set_num_extruders() {
        let text = read_vectors("functions");
        let expected: Vec<String> = text
            .lines()
            .filter(|l| {
                l.starts_with("static ")
                    || l.starts_with("fn full_print_config ")
                    || l.starts_with("fn new_from_defaults_keys ")
                    || l.starts_with("fn set_num_extruders ")
            })
            .map(|l| l.to_string())
            .collect();
        let mut got = Vec::new();
        static_lines(&mut got);
        assert_eq!(expected.len(), 21);
        compare("static", &expected, &got);
    }

    #[test]
    fn free_functions_and_dynamic_print_config() {
        let text = read_vectors("functions");
        let expected: Vec<String> = text
            .lines()
            .filter(|l| l.starts_with("fn ") || l.starts_with("scenario "))
            .filter(|l| {
                !l.starts_with("fn full_print_config ")
                    && !l.starts_with("fn new_from_defaults_keys ")
                    && !l.starts_with("fn set_num_extruders ")
                    && !l.starts_with("fn update_values_from_")
                    && !l.starts_with("fn update_static_print_config_from_dynamic ")
                    && !l.starts_with("fn update_non_diff_values_to_base_config ")
                    && !l.starts_with("fn update_diff_values_to_child_config ")
                    && !l.starts_with("fn align_filament_variant_overrides ")
                    && !l.starts_with("fn filament_override_default_indices ")
                    && !l.starts_with("fn compute_filament_override_value ")
            })
            .map(|l| l.to_string())
            .collect();
        let mut got = Vec::new();
        pure_lines(&mut got);
        for line in &expected {
            if !line.starts_with("scenario ") {
                continue;
            }
            let t: Vec<&str> = line.split(' ').collect();
            let si: usize = t[1].parse().unwrap();
            let n: usize = t[2].parse().unwrap();
            let sc: Vec<(String, String)> = (0..n).map(|i| (unhex(t[3 + 2 * i]), unhex(t[4 + 2 * i]))).collect();
            scenario_lines(si, &sc, &mut got);
        }
        assert!(expected.len() > 2400, "{} lines", expected.len());
        compare("functions", &expected, &got);
    }
}
