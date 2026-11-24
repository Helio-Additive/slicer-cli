//! Native parity of `Print::update_filament_maps_to_config` (Print.cpp:3241-3320),
//! `Print::update_to_config_by_nozzle_group_result` (:3347-3400) and `Print::resolve_manual_nozzle_map`
//! (:3322-3345) at Print level, against the pin: `tests/data/print_filament_maps/gen.cpp` applies
//! three configurations made from the H2D job's project settings (Majora on the Bambu Lab H2D, two
//! nozzles, 8 filaments; the same with two variants per filament; the same under fmmNozzleManual) to a
//! Print with an empty Model (`Print::apply`), writes the state it leaves (`m_config`,
//! `m_ori_full_print_config`, `m_full_print_config`, the placeholder parser's config), then for every
//! scenario makes the call on a fresh Print and writes what it threw, the options that changed,
//! `m_filament_self_index` and `m_has_auto_filament_map_result`. Scenario 0 is the call
//! ToolOrdering.cpp:3609-3614 makes on that job (`filament_map 1,2,2,1,2,2,2,1`, the volume map,
//! `filament_nozzle_map 0,1,1,0,1,1,1,0`, as slicer_cli's G-code header shows them).
//!
//! The Rust Print is given the same "pre" state (the port has no faithful `Print::apply`), the call
//! is made, and every option of the four configurations must serialize as the pin's.

use slicer::config::*;
use slicer::print::faithful::Print;
use slicer::print_config::config_def::DynamicPrintConfig;
use std::collections::BTreeMap;
use std::io::Read;

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

/// gen.cpp `value_of`: "i <n> <ints>" for an enum vector without keys map, else "s <hex>" / "! <hex>".
fn value_of(o: &dyn ConfigOption) -> String {
    let ev = if let Some(a) = o.as_any().downcast_ref::<ConfigOptionEnumsGeneric>() {
        Some((a.keys_map.is_none(), a.values.clone()))
    } else if let Some(b) = o.as_any().downcast_ref::<ConfigOptionEnumsGenericNullable>() {
        Some((b.keys_map.is_none(), b.values.clone()))
    } else {
        None
    };
    if let Some((true, vals)) = ev {
        let mut s = format!("i {}", vals.len());
        for v in vals {
            s.push_str(&format!(" {v}"));
        }
        return s;
    }
    match o.serialize() {
        Ok(s) => format!("s {}", hex(&s)),
        Err(e) => format!("! {}", hex(e.what())),
    }
}

type Values = BTreeMap<String, String>;

fn values_of<C: ConfigOptionResolver + ?Sized>(cfg: &C, keys: Vec<String>) -> Values {
    keys.into_iter().map(|k| {
        let v = value_of(cfg.option(&k).expect("option"));
        (k, v)
    }).collect()
}

fn state_of(p: &Print) -> BTreeMap<&'static str, Values> {
    let mut s = BTreeMap::new();
    s.insert("config", values_of(&p.m_config, ConfigBase::keys(&p.m_config)));
    s.insert("ori", values_of(&p.base.base.m_ori_full_print_config, ConfigBase::keys(&p.base.base.m_ori_full_print_config)));
    s.insert("full", values_of(&p.base.base.m_full_print_config, ConfigBase::keys(&p.base.base.m_full_print_config)));
    let pp = p.base.base.m_placeholder_parser.config();
    s.insert("pp", values_of(pp, ConfigBase::keys(pp)));
    s
}

/// Sets `key` of `cfg` to a value written by gen.cpp `value_of`.
fn set_value(cfg: &mut DynamicPrintConfig, key: &str, kind: &str, rest: &[&str]) {
    match kind {
        "s" => cfg.set_deserialize_strict(key, &unhex(rest[0]), false).unwrap_or_else(|e| panic!("{key}: {}", e.what())),
        "i" => {
            let vals: Vec<i32> = rest[1..].iter().map(|x| x.parse().expect("int")).collect();
            let opt = cfg.option_mut(key, true).expect("option").expect("option");
            if let Some(a) = opt.as_any_mut().downcast_mut::<ConfigOptionEnumsGeneric>() {
                a.values = vals;
            } else if let Some(b) = opt.as_any_mut().downcast_mut::<ConfigOptionEnumsGenericNullable>() {
                b.values = vals;
            } else {
                panic!("{key}: not an enum vector");
            }
        }
        _ => panic!("{key}: {kind}"),
    }
}

fn ints(t: &[&str], at: &mut usize) -> Vec<i32> {
    let n: usize = t[*at].parse().expect("n");
    let v = t[*at + 1..*at + 1 + n].iter().map(|x| x.parse().expect("int")).collect();
    *at += 1 + n;
    v
}

/// gen.cpp `FixedNozzles`: a `NozzleGroupResultBase` whose `get_nozzles_for_filament` is a table.
struct FixedNozzles(Vec<Vec<slicer::multi_nozzle_utils::NozzleInfo>>);

impl slicer::print::faithful::NozzleGroupResultNozzles for FixedNozzles {
    fn get_nozzles_for_filament(&self, filament_id: i32) -> Vec<slicer::multi_nozzle_utils::NozzleInfo> {
        self.0.get(filament_id as usize).cloned().unwrap_or_default()
    }
}

enum Call {
    Maps(Vec<i32>, Vec<i32>, Vec<i32>),
    Group(Vec<Vec<(i32, i32)>>),
    Manual(Vec<u32>),
}

#[test]
fn update_filament_maps_to_config_matches_the_pin() {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/print_filament_maps/vectors.txt.gz");
    let mut text = String::new();
    flate2::read::GzDecoder::new(std::fs::File::open(path).expect("vectors")).read_to_string(&mut text).expect("gzip");

    // The "pre" state of every configuration.
    type PreLines = BTreeMap<String, Vec<(String, String, Vec<String>)>>;
    let mut pre_lines: BTreeMap<usize, PreLines> = BTreeMap::new();
    let mut pre: BTreeMap<usize, BTreeMap<String, Values>> = BTreeMap::new();
    struct Scenario {
        cfg: usize,
        call: Call,
        result: String,
        post: BTreeMap<String, Values>,
        gone: BTreeMap<String, Vec<String>>,
        self_index: Vec<i32>,
        auto: bool,
    }
    let mut scenarios: Vec<Scenario> = Vec::new();
    let mut pre_self_index: BTreeMap<usize, Vec<i32>> = BTreeMap::new();
    for line in text.lines() {
        let t: Vec<&str> = line.split(' ').collect();
        match t[0] {
            "opt" => {
                let cfg: usize = t[1].parse().expect("cfg");
                let key = unhex(t[3]);
                pre.entry(cfg).or_default().entry(t[2].to_string()).or_default().insert(key.clone(), t[4..].join(" "));
                pre_lines
                    .entry(cfg)
                    .or_default()
                    .entry(t[2].to_string())
                    .or_default()
                    .push((key, t[4].to_string(), t[5..].iter().map(|s| s.to_string()).collect()));
            }
            "scenario" => {
                let cfg: usize = t[2].parse().expect("cfg");
                let mut at = 4;
                let call = match t[3] {
                    "maps" => {
                        let f_maps = ints(&t, &mut at);
                        let f_volume_maps = ints(&t, &mut at);
                        let f_nozzle_maps = ints(&t, &mut at);
                        Call::Maps(f_maps, f_volume_maps, f_nozzle_maps)
                    }
                    "group" => {
                        let n: usize = t[at].parse().expect("n");
                        at += 1;
                        let mut table = Vec::new();
                        for _ in 0..n {
                            let m: usize = t[at].parse().expect("m");
                            at += 1;
                            let mut row = Vec::new();
                            for _ in 0..m {
                                row.push((t[at].parse().expect("extruder"), t[at + 1].parse().expect("volume")));
                                at += 2;
                            }
                            table.push(row);
                        }
                        Call::Group(table)
                    }
                    "manual" => Call::Manual(ints(&t, &mut at).into_iter().map(|x| x as u32).collect()),
                    other => panic!("function {other}"),
                };
                scenarios.push(Scenario {
                    cfg,
                    call,
                    result: String::new(),
                    post: BTreeMap::new(),
                    gone: BTreeMap::new(),
                    self_index: Vec::new(),
                    auto: false,
                });
            }
            "result" => scenarios.last_mut().expect("scenario").result = t[2].to_string(),
            "pre_self_index" => {
                let mut at = 2;
                pre_self_index.insert(t[1].parse::<usize>().expect("cfg"), ints(&t, &mut at));
            }
            "post" => {
                let s = scenarios.last_mut().expect("scenario");
                s.post.entry(t[2].to_string()).or_default().insert(unhex(t[3]), t[4..].join(" "));
            }
            "gone" => {
                let s = scenarios.last_mut().expect("scenario");
                s.gone.entry(t[2].to_string()).or_default().push(unhex(t[3]));
            }
            "self_index" => {
                let mut at = 2;
                scenarios.last_mut().expect("scenario").self_index = ints(&t, &mut at);
            }
            "auto" => scenarios.last_mut().expect("scenario").auto = t[2] == "1",
            _ => panic!("unknown line {line}"),
        }
    }
    assert_eq!(pre.len(), 3);
    assert_eq!(scenarios.len(), 14);

    let make_print = |cfg: usize| -> Print {
        let lines = &pre_lines[&cfg];
        let mut p = Print::new();
        let load = |c: &mut DynamicPrintConfig, which: &str| {
            for (key, kind, rest) in lines.get(which).map(|v| v.as_slice()).unwrap_or(&[]) {
                let rest: Vec<&str> = rest.iter().map(|s| s.as_str()).collect();
                set_value(c, key, kind, &rest);
            }
        };
        load(&mut p.m_config, "config");
        p.m_filament_self_index = pre_self_index[&cfg].clone();
        let mut ori = DynamicPrintConfig::new();
        load(&mut ori, "ori");
        p.base.base.m_ori_full_print_config = ori;
        let mut full = DynamicPrintConfig::new();
        load(&mut full, "full");
        p.base.base.m_full_print_config = full;
        // the placeholder parser's own config as the pin's Print holds it (cleared, then the options)
        let mut pp = DynamicPrintConfig::new();
        load(&mut pp, "pp");
        p.base.base.m_placeholder_parser.clear_config();
        for k in ConfigBase::keys(&pp) {
            let opt = ConfigOption::clone(pp.option(&k).expect("pp option"));
            p.base.base.m_placeholder_parser.config_writable().set_key_value(&k, opt);
        }
        // print_preset / filament_preset / printer_preset are no options of the definition: the pin
        // sets clones of print_settings_id (a string), filament_settings_id (strings) and
        // printer_settings_id (PrintApply.cpp:1656-1659).
        for (key, kind, rest) in lines.get("pp").map(|v| v.as_slice()).unwrap_or(&[]) {
            if slicer::print_config::config_def::print_config_def().get(key).is_some() {
                continue;
            }
            assert_eq!(kind, "s", "{key}");
            let text = unhex(&rest[0]);
            let mut opt: Box<dyn ConfigOption> = if text.starts_with('"') {
                Box::new(ConfigOptionStrings::from_values(Vec::new()))
            } else {
                Box::new(ConfigOptionString::new())
            };
            assert!(opt.deserialize(&text, false).expect("deserialize"), "{key}");
            p.base.base.m_placeholder_parser.config_writable().set_key_value(key, opt);
        }
        p
    };

    // The Rust Print given the "pre" state holds the same options.
    for (cfg, state) in &pre {
        let s0 = state_of(&make_print(*cfg));
        for (which, values) in state {
            assert_eq!(&s0[which.as_str()], values, "pre state of {cfg} {which}");
        }
    }

    let mut bad = 0;
    for (id, sc) in scenarios.iter().enumerate() {
        let mut p = make_print(sc.cfg);
        let result = match &sc.call {
            Call::Maps(f_maps, f_volume_maps, f_nozzle_maps) => {
                p.update_filament_maps_to_config(f_maps.clone(), f_volume_maps.clone(), f_nozzle_maps.clone());
                "ok".to_string()
            }
            Call::Group(table) => {
                use slicer::multi_nozzle_utils::NozzleInfo;
                use slicer::extruder::NozzleVolumeType;
                let g = FixedNozzles(
                    table
                        .iter()
                        .map(|row| {
                            row.iter()
                                .map(|&(e, v)| NozzleInfo { extruder_id: e, volume_type: NozzleVolumeType::from_i32(v), ..Default::default() })
                                .collect()
                        })
                        .collect(),
                );
                p.update_to_config_by_nozzle_group_result(&g);
                "ok".to_string()
            }
            Call::Manual(used) => match p.resolve_manual_nozzle_map(used) {
                Ok(()) => "ok".to_string(),
                Err(e) => format!("!{}", hex(&e.to_string())),
            },
        };
        if result != sc.result {
            eprintln!("scenario {id}: C++ {} Rust {}", sc.result, result);
            bad += 1;
        }
        let got = state_of(&p);
        for (which, before) in &pre[&sc.cfg] {
            let mut want = before.clone();
            if let Some(post) = sc.post.get(which) {
                for (k, v) in post {
                    want.insert(k.clone(), v.clone());
                }
            }
            if let Some(gone) = sc.gone.get(which) {
                for k in gone {
                    want.remove(k);
                }
            }
            let have = &got[which.as_str()];
            for (k, v) in &want {
                if have.get(k) != Some(v) {
                    eprintln!("scenario {id} {which} {k}: C++ {} Rust {:?}", v, have.get(k));
                    bad += 1;
                }
            }
            for k in have.keys() {
                if !want.contains_key(k) {
                    eprintln!("scenario {id} {which} {k}: only in Rust");
                    bad += 1;
                }
            }
        }
        if p.m_filament_self_index != sc.self_index {
            eprintln!("scenario {id} m_filament_self_index: C++ {:?} Rust {:?}", sc.self_index, p.m_filament_self_index);
            bad += 1;
        }
        if p.m_has_auto_filament_map_result != sc.auto {
            eprintln!("scenario {id} m_has_auto_filament_map_result: C++ {} Rust {}", sc.auto, p.m_has_auto_filament_map_result);
            bad += 1;
        }
    }
    assert_eq!(bad, 0, "{bad} values differ from the pin");
}
