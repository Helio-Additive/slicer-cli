//! `TimeProcessor::post_process_with_context` (the port's GCodeProcessor::TimeProcessor::post_process
//! with pre-heating, GCodeProcessor.cpp:571-1300, and the PreCoolingInjector, :6611-7202) against the
//! pin on generated two-extruder G-code.
//!
//! `tests/data/pre_cooling_injector/vectors.txt.gz` is the output of `gen.cpp` beside it (the header
//! comment there describes the lines): per scenario the context, the G-code, the moves' line ids and
//! times, and the file the pin's `post_process` leaves (no M73 time lines: the time machine has no
//! cache; `M73 E` lines, `M632 / M104 ... W ;Multi extruder pre heating / M633` and
//! `M400 / M104 ... C ;Multi extruder pre cooling` lines).

use slicer::extruder::{ExtruderType, NozzleVolumeType};
use slicer::gcode::g_code_processor::{FilamentVariantRows, MoveVertexTime, TimeProcessContext, TimeProcessor};
use slicer::multi_nozzle_utils::{LayeredNozzleGroupResult, NozzleInfo};
use std::io::Read;

fn read_vectors() -> String {
    let path = format!("{}/tests/data/pre_cooling_injector/vectors.txt.gz", env!("CARGO_MANIFEST_DIR"));
    let file = std::fs::File::open(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
    let mut text = String::new();
    flate2::read::GzDecoder::new(file).read_to_string(&mut text).expect("gzip");
    text
}

fn unhex(s: &str) -> String {
    if s == "-" {
        return String::new();
    }
    let b: Vec<u8> = (0..s.len()).step_by(2).map(|i| u8::from_str_radix(&s[i..i + 2], 16).expect("hex")).collect();
    String::from_utf8(b).expect("utf8")
}

fn hexf(s: &str) -> f64 {
    // %a: [-]0xh.hhhp[+-]d
    let (neg, s) = if let Some(r) = s.strip_prefix('-') { (true, r) } else { (false, s) };
    let s = s.strip_prefix("0x").expect("0x");
    let (mant, exp) = s.split_once('p').expect("p");
    let (ip, fp) = mant.split_once('.').unwrap_or((mant, ""));
    let mut v = u64::from_str_radix(ip, 16).expect("int") as f64;
    let mut scale = 1.0 / 16.0;
    for c in fp.chars() {
        v += c.to_digit(16).expect("digit") as f64 * scale;
        scale /= 16.0;
    }
    let e: i32 = exp.parse().expect("exp");
    let v = v * 2f64.powi(e);
    if neg { -v } else { v }
}

#[test]
fn post_process_pre_heating_matches_the_pin() {
    let text = read_vectors();
    let mut lines = text.lines();
    let (mut cases, mut bad) = (0, 0);
    let mut first_bad: Option<String> = None;
    while let Some(head) = lines.next() {
        let mut h = head.split_whitespace();
        assert_eq!(h.next(), Some("S"));
        let sc: i32 = h.next().unwrap().parse().unwrap();
        let f: usize = h.next().unwrap().parse().unwrap();
        let hotend = h.next().unwrap() == "1";
        let mut field = |tag: &str| -> Vec<String> {
            let l = lines.next().expect(tag);
            let mut it = l.split_whitespace();
            assert_eq!(it.next(), Some(tag));
            it.map(str::to_owned).collect()
        };
        let fmap: Vec<i32> = field("fmap").iter().map(|v| v.parse().unwrap()).collect();
        let nt: Vec<i32> = field("nt").iter().map(|v| v.parse().unwrap()).collect();
        let pct: Vec<i32> = field("pct").iter().map(|v| v.parse().unwrap()).collect();
        let pdelta: Vec<f64> = field("pdelta").iter().map(|v| hexf(v)).collect();
        let minec: Vec<f64> = field("minec").iter().map(|v| hexf(v)).collect();
        let ftypes: Vec<String> = field("ftypes");
        let phe: Vec<u8> = field("phe").iter().map(|v| v.parse().unwrap()).collect();
        let cool: Vec<f64> = field("cool").iter().map(|v| hexf(v)).collect();
        let rack: Vec<f64> = field("rack").iter().map(|v| hexf(v)).collect();
        let heat: Vec<f64> = field("heat").iter().map(|v| hexf(v)).collect();
        let gcode = unhex(&field("gcode")[0]);
        let mv = field("moves");
        let n: usize = mv[0].parse().unwrap();
        let moves: Vec<MoveVertexTime> = (0..n)
            .map(|k| MoveVertexTime {
                gcode_id: mv[1 + 3 * k].parse().unwrap(),
                time: [hexf(&mv[2 + 3 * k]) as f32, hexf(&mv[3 + 3 * k]) as f32],
            })
            .collect();
        let want = unhex(&field("out")[0]);

        let nozzles: Vec<NozzleInfo> = (0..2)
            .map(|k| NozzleInfo {
                diameter: "0.4".to_string(),
                volume_type: NozzleVolumeType::NvtStandard,
                extruder_id: k,
                group_id: k,
                installed: true,
            })
            .collect();
        let used: Vec<u32> = (0..f as u32).collect();
        let group = LayeredNozzleGroupResult::create(&fmap, &nozzles, &used).expect("group");
        let ctx = TimeProcessContext {
            filament_types: ftypes,
            filament_pre_heating_enabled: phe,
            physical_extruder_map: vec![1, 0],
            nozzle_group_result: group,
            cooling_rate: cool,
            cooling_rate_on_rack: rack,
            heating_rate: heat,
            filament_min_temperature_when_ec: minec,
            inject_time_threshold: 0.0,
            enable_pre_heating: true,
            handle_hotend_as_extruder: hotend,
            has_filament_switcher: false,
            extruder_max_nozzle_count: vec![1, 1],
            extruder_types: vec![ExtruderType::EtDirectDrive, ExtruderType::EtDirectDrive],
            nozzle_diameter: vec![0.4, 0.4],
            filament_variant_rows: FilamentVariantRows {
                nozzle_temp: nt,
                pre_cooling_temp: pct,
                preheat_temperature_delta: pdelta,
                extruder_variant: Vec::new(),
                self_index: Vec::new(),
                default_row: (0..f as i32).collect(),
            },
        };
        let tp = TimeProcessor::default();
        let got = tp.post_process_with_context(&gcode, &moves, &ctx, true);
        cases += 1;
        if got != want {
            bad += 1;
            if first_bad.is_none() {
                let gl: Vec<&str> = got.lines().collect();
                let wl: Vec<&str> = want.lines().collect();
                let k = (0..gl.len().max(wl.len())).find(|&k| gl.get(k) != wl.get(k)).unwrap_or(0);
                first_bad = Some(format!(
                    "scenario {sc} (hotend {hotend}): first differing line {k}: rust {:?} pin {:?}",
                    gl.get(k),
                    wl.get(k)
                ));
            }
        }
    }
    assert_eq!(cases, 48);
    assert_eq!(bad, 0, "{bad} of {cases} scenarios differ; {}", first_bad.unwrap_or_default());
}
