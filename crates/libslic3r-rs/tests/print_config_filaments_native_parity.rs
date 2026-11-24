//! Native parity of `DynamicPrintConfig::update_values_to_printer_extruders_for_multiple_filaments`
//! (PrintConfig.cpp:9996-10191) against the pin: `tests/data/print_config/gen.cpp` `gen_filaments`
//! writes `filaments.txt` (the config after the call, every key serialized, hex). The scenarios are
//! the generator's `filament_scenarios()`, repeated here.

use slicer::config::*;
use slicer::print_config::config_def::*;
use std::collections::BTreeSet;

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

/// gen.cpp `filament_scenarios()`: ((extruder_count, extruder_nozzle_volume_count), settings).
fn scenarios() -> Vec<((i32, i32), Vec<(&'static str, &'static str)>)> {
    vec![
        ((1, 1), vec![
            ("filament_map", "1,1"), ("extruder_type", "Direct Drive"), ("nozzle_volume_type", "Standard"),
            ("filament_extruder_variant", "\"Direct Drive Standard\";\"Direct Drive High Flow\";\"Direct Drive Standard\";\"Direct Drive High Flow\""),
            ("filament_self_index", "1,1,2,2"), ("filament_max_volumetric_speed", "10,20,11,21"), ("filament_flow_ratio", "0.9,0.8,0.95,0.85"),
        ]),
        ((2, 2), vec![
            ("filament_map", "1,2,2"), ("extruder_type", "Direct Drive,Bowden"), ("nozzle_volume_type", "Standard,High Flow"),
            ("filament_volume_map", "0,1,1"),
            ("filament_extruder_variant", "\"Direct Drive Standard\";\"Bowden High Flow\";\"Direct Drive Standard\";\"Bowden High Flow\";\"Bowden High Flow\""),
            ("filament_self_index", "1,1,2,2,3"), ("filament_max_volumetric_speed", "1,2,3,4,5"),
        ]),
        ((2, 3), vec![
            ("filament_map", "1,2,2"), ("extruder_type", "Direct Drive,Bowden"), ("nozzle_volume_type", "Standard,Standard"),
            ("filament_volume_map", "0,1,0"),
            ("filament_extruder_variant", "\"Direct Drive Standard\";\"Bowden High Flow\";\"Bowden Standard\";\"Bowden High Flow\";\"Bowden Standard\""),
            ("filament_self_index", "1,2,2,3,3"), ("filament_max_volumetric_speed", "1,2,3,4,5"), ("filament_flow_ratio", "0.5,0.6,0.7,0.8,0.9"),
        ]),
        ((1, 1), vec![
            ("filament_map", "1,1"), ("extruder_type", "Direct Drive"), ("nozzle_volume_type", "High Flow"),
            ("filament_extruder_variant", "\"Direct Drive Standard\";\"Direct Drive Standard\""),
            ("filament_self_index", "2,1"), ("filament_max_volumetric_speed", "7,8"),
        ]),
    ]
}

#[test]
fn update_values_to_printer_extruders_for_multiple_filaments_matches_the_pin() {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/print_config/filaments.txt");
    let text = std::fs::read_to_string(path).expect("filaments.txt");
    let expected: Vec<&str> = text
        .lines()
        .filter(|l| l.starts_with("fn update_values_to_printer_extruders_for_multiple_filaments "))
        .collect();
    let sc = scenarios();
    assert_eq!(expected.len(), sc.len());
    for (a, ((extruder_count, nvc), settings)) in sc.iter().enumerate() {
        let mut c = DynamicPrintConfig::new();
        for (k, v) in settings {
            c.set_deserialize_strict(k, v, false).expect("set_deserialize_strict");
        }
        let p = c.clone();
        let mut ks: BTreeSet<String> = (*filament_options_with_variant).clone();
        ks.insert("filament_self_index".to_string());
        c.update_values_to_printer_extruders_for_multiple_filaments(
            Some(&p),
            *extruder_count,
            *nvc,
            &ks,
            "filament_self_index",
            "filament_extruder_variant",
        )
        .expect("update");
        let mut l: Vec<String> = vec![
            "fn".into(),
            "update_values_to_printer_extruders_for_multiple_filaments".into(),
            a.to_string(),
            "=>".into(),
        ];
        let keys = c.keys();
        l.push(keys.len().to_string());
        for k in &keys {
            l.push(hex(k));
            l.push(hex(&c.option(k).unwrap().serialize().expect("serialize")));
        }
        assert_eq!(l.join(" "), expected[a], "scenario {a}");
    }
}

/// gen.cpp `gen_printer_extruders`: `update_values_to_printer_extruders` (PrintConfig.cpp:9780-9994)
/// over `printer_options_with_variant_1`, stride 1, extruder_id 0 / 1 / 2, filament volume type
/// High Flow; the printer config is the config itself.
#[test]
fn update_values_to_printer_extruders_matches_the_pin() {
    use slicer::print_config::enums::NozzleVolumeType;
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/print_config/filaments.txt");
    let text = std::fs::read_to_string(path).expect("filaments.txt");
    let expected: Vec<&str> =
        text.lines().filter(|l| l.starts_with("fn update_values_to_printer_extruders ")).collect();
    let ps: Vec<Vec<(&str, &str)>> = vec![
        vec![
            ("extruder_type", "Direct Drive"), ("nozzle_volume_type", "Standard"),
            ("printer_extruder_variant", "\"Direct Drive Standard\";\"Direct Drive High Flow\""), ("printer_extruder_id", "1,1"),
            ("retraction_length", "0.8,1.2"), ("z_hop", "0.4,0.6"),
        ],
        vec![
            ("extruder_type", "Direct Drive,Direct Drive"), ("nozzle_volume_type", "Standard,High Flow"),
            ("printer_extruder_variant", "\"Direct Drive Standard\";\"Direct Drive High Flow\";\"Direct Drive Standard\";\"Direct Drive High Flow\""),
            ("printer_extruder_id", "1,1,2,2"), ("retraction_length", "1,2,3,4"), ("z_hop", "0.1,0.2,0.3,0.4"),
        ],
        vec![
            ("extruder_type", "Direct Drive,Bowden"), ("nozzle_volume_type", "Hybrid,Standard"),
            ("printer_extruder_variant", "\"Direct Drive Standard\";\"Direct Drive High Flow\";\"Bowden Standard\""),
            ("printer_extruder_id", "1,1,2"), ("retraction_length", "1,2,3"), ("z_hop", "0.1,0.2,0.3"),
        ],
    ];
    assert_eq!(expected.len(), ps.len() * 3);
    let mut n = 0;
    for (a, settings) in ps.iter().enumerate() {
        for extruder_id in 0u32..3 {
            let mut c = DynamicPrintConfig::new();
            for (k, v) in settings {
                c.set_deserialize_strict(k, v, false).expect("set_deserialize_strict");
            }
            let mut extruder_count: i32 = 1;
            c.support_different_extruders(&mut extruder_count);
            let mut nvt: Vec<Vec<NozzleVolumeType>> = Vec::new();
            let envc = c.get_extruder_nozzle_volume_count(extruder_count, &mut nvt);
            let vi = c
                .update_values_to_printer_extruders(
                    None,
                    extruder_count,
                    envc,
                    &nvt,
                    &printer_options_with_variant_1,
                    "printer_extruder_id",
                    "printer_extruder_variant",
                    1,
                    extruder_id,
                    NozzleVolumeType::NvtHighFlow,
                )
                .expect("update");
            let mut vis = vi.len().to_string();
            for x in &vi {
                vis.push_str(&format!(",{x}"));
            }
            let mut l: Vec<String> = vec![
                "fn".into(),
                "update_values_to_printer_extruders".into(),
                a.to_string(),
                extruder_id.to_string(),
                "=>".into(),
                vis,
            ];
            let keys = c.keys();
            l.push(keys.len().to_string());
            for k in &keys {
                l.push(hex(k));
                l.push(hex(&c.option(k).unwrap().serialize().expect("serialize")));
            }
            assert_eq!(l.join(" "), expected[n], "scenario {a} extruder_id {extruder_id}");
            n += 1;
        }
    }
}
