//! `slicer::gcode::g_code_processor` (the port of libslic3r/GCode/GCodeProcessor.cpp) against the C++
//! at the pin.
//!
//! `tests/data/g_code_processor/g_code_processor.txt.gz` is the output of `gen.cpp` beside it (gzip
//! -9n), which includes the pin's GCodeProcessor.cpp. It holds the results of the file-static
//! helpers (`remove_eols`, `parse_number`, `set_option_value` / `get_option_value`), of the static
//! queries (`parse_helioadditive_comment`, `get_last_position_from_gcode`, `get_last_z_from_gcode`,
//! `get_gcode_last_filament`, `contains_reserved_tag(s)`), and of whole synthetic programs run
//! through `apply_config`, `initialize`, `process_buffer` and `finalize(true)` with the machine
//! limits of a BBL printer under four G-code flavors: the estimated times per mode, the times per
//! role, move type and layer, the filament volumes, every move and the text after
//! `TimeProcessor::post_process`. The header of gen.cpp describes the lines. The producer tags are
//! compared by the unit tests of the module (they are private).

use slicer::config::ConfigOptionFloats;
use slicer::gcode::g_code_processor::*;
use std::io::Read;

fn read_vectors() -> String {
    let path = format!("{}/tests/data/g_code_processor/g_code_processor.txt.gz", env!("CARGO_MANIFEST_DIR"));
    let file = std::fs::File::open(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
    let mut text = String::new();
    flate2::read::GzDecoder::new(file).read_to_string(&mut text).expect("gzip");
    text
}

fn unhex(t: &str) -> String {
    if t == "-" {
        return String::new();
    }
    let b: Vec<u8> = (0..t.len() / 2).map(|i| u8::from_str_radix(&t[2 * i..2 * i + 2], 16).expect("hex")).collect();
    String::from_utf8(b).expect("the vectors are UTF-8")
}

fn hex(s: &str) -> String {
    if s.is_empty() {
        return "-".to_string();
    }
    s.bytes().map(|c| format!("{:02x}", c)).collect()
}

fn f32h(v: f32) -> String {
    format!("{:08x}", v.to_bits())
}

fn f64h(v: f64) -> String {
    format!("{:016x}", v.to_bits())
}

fn f32of(t: &str) -> f32 {
    f32::from_bits(u32::from_str_radix(t, 16).expect("f32 bits"))
}

#[test]
fn helpers() {
    let text = read_vectors();
    let mut n = 0usize;
    let mut bad: Vec<String> = Vec::new();
    for line in text.lines() {
        let t: Vec<&str> = line.split(' ').collect();
        let got: Option<String> = match t[0] {
            "eols" => {
                let s = unhex(t[1]);
                let e = remove_eols(s.as_bytes(), 0, s.len());
                Some(format!("eols {} {}", t[1], e))
            }
            "pnf" => {
                let s = unhex(t[1]);
                let mut v = 0.0f32;
                let ok = parse_number(&s, &mut v);
                Some(format!("pnf {} {} {}", t[1], ok as i32, f32h(v)))
            }
            "pni" => {
                let s = unhex(t[1]);
                let mut v = 0i32;
                let ok = parse_number(&s, &mut v);
                Some(format!("pni {} {} {}", t[1], ok as i32, v))
            }
            "optv" => {
                let id: usize = t[1].parse().unwrap();
                let value = f32of(t[2]);
                let nvals: usize = t[3].parse().unwrap();
                let mut opt = ConfigOptionFloats::from_values((0..nvals).map(|i| 10.0 * (i + 1) as f64 + 0.25).collect());
                let before = opt.get_option_value(id);
                opt.set_option_value(id, value);
                let mut s = format!("optv {} {} {} {}", id, t[2], nvals, f32h(before));
                for v in &opt.values {
                    s += &format!(" {}", f64h(*v));
                }
                Some(s)
            }
            "helio" => {
                let c = unhex(t[1]);
                let mut is_helio = false;
                let ti = parse_helioadditive_comment(&c, &mut is_helio);
                Some(format!("helio {} {} {} {} {}", t[1], is_helio as i32, f32h(ti.min), f32h(ti.max), f32h(ti.mean)))
            }
            "lastpos" => {
                let s = unhex(t[1]);
                let mut p = [f32of(t[2]), f32of(t[3]), f32of(t[4])];
                let r = get_last_position_from_gcode(&s, &mut p);
                Some(format!("lastpos {} {} {} {} {} {} {} {}", t[1], t[2], t[3], t[4], r as i32, f32h(p[0]), f32h(p[1]), f32h(p[2])))
            }
            "lastz" => {
                let s = unhex(t[1]);
                let r = get_last_z_from_gcode(&s);
                Some(format!("lastz {} {} {}", t[1], r.is_some() as i32, f64h(r.unwrap_or(-1.0))))
            }
            "lastfil" => {
                let s = unhex(t[1]);
                Some(format!("lastfil {} {}", t[1], get_gcode_last_filament(&s)))
            }
            "tag" => {
                let s = unhex(t[1]);
                let r = contains_reserved_tag(&s);
                Some(format!("tag {} {} {}", t[1], r.is_some() as i32, hex(&r.unwrap_or_default())))
            }
            "tags" => {
                let s = unhex(t[1]);
                let max: u32 = t[2].parse().unwrap();
                let mut fs: Vec<String> = Vec::new();
                let r = contains_reserved_tags(&s, max, &mut fs);
                let mut o = format!("tags {} {} {} {}", t[1], max, r as i32, fs.len());
                for f in &fs {
                    o += &format!(" {}", hex(f));
                }
                Some(o)
            }
            _ => None,
        };
        if let Some(g) = got {
            n += 1;
            if g != line {
                bad.push(format!("  want {line}\n  got  {g}"));
            }
        }
    }
    assert!(n >= 150, "only {n} helper lines");
    assert!(bad.is_empty(), "{} of {} helper lines differ:\n{}", bad.len(), n, bad.join("\n"));
}

struct Case {
    name: String,
    cfg: Vec<(String, String)>,
    stealth: bool,
    envelope: bool,
    prog: String,
    lines: Vec<String>,
}

fn cases(text: &str) -> Vec<Case> {
    let mut out = Vec::new();
    let mut cur: Option<Case> = None;
    for line in text.lines() {
        let t: Vec<&str> = line.splitn(2, ' ').collect();
        match t[0] {
            "case" => {
                cur = Some(Case {
                    name: t[1].to_string(),
                    cfg: Vec::new(),
                    stealth: false,
                    envelope: false,
                    prog: String::new(),
                    lines: Vec::new(),
                })
            }
            "end" => out.push(cur.take().expect("case")),
            _ => {
                if let Some(c) = cur.as_mut() {
                    match t[0] {
                        "cfg" => {
                            let kv: Vec<&str> = t[1].split(' ').collect();
                            c.cfg.push((kv[0].to_string(), unhex(kv[1])));
                        }
                        "stealth" => c.stealth = t[1] == "1",
                        "envelope" => c.envelope = t[1] == "1",
                        "prog" => c.prog = unhex(t[1]),
                        _ => c.lines.push(line.to_string()),
                    }
                }
            }
        }
    }
    out
}

/// The port's `PrintConfig` with the values the case gives to the C++ `PrintConfig`.
fn rust_config(cfg: &[(String, String)]) -> slicer::print_config::PrintConfig {
    use slicer::print_config::GCodeFlavor;
    let mut c = slicer::print_config::PrintConfig::new();
    for (k, v) in cfg {
        let list: Vec<f64> = v.split(',').filter_map(|x| x.parse::<f64>().ok()).collect();
        let first = list.first().copied().unwrap_or(0.0);
        match k.as_str() {
            "gcode_flavor" => {
                c.gcode_flavor = match v.as_str() {
                    "marlin" => GCodeFlavor::MarlinLegacy,
                    "marlin2" => GCodeFlavor::Marlin,
                    "klipper" => GCodeFlavor::Klipper,
                    "reprapfirmware" => GCodeFlavor::RepRapFirmware,
                    "reprap" => GCodeFlavor::RepRapSprinter,
                    "repetier" => GCodeFlavor::Repetier,
                    "sailfish" => GCodeFlavor::Sailfish,
                    "makerware" => GCodeFlavor::MakerWare,
                    "smoothie" => GCodeFlavor::Smoothie,
                    other => panic!("flavor {other}"),
                }
            }
            "machine_max_acceleration_x" => c.machine_max_acceleration_x = first,
            "machine_max_acceleration_y" => c.machine_max_acceleration_y = first,
            "machine_max_acceleration_z" => c.machine_max_acceleration_z = first,
            "machine_max_acceleration_e" => c.machine_max_acceleration_e = first,
            "machine_max_acceleration_extruding" => c.machine_max_acceleration_extruding = first,
            "machine_max_acceleration_retracting" => c.machine_max_acceleration_retracting = first,
            "machine_max_acceleration_travel" => c.machine_max_acceleration_travel = first,
            "machine_max_speed_x" => c.machine_max_speed_x = first,
            "machine_max_speed_y" => c.machine_max_speed_y = first,
            "machine_max_speed_z" => c.machine_max_speed_z = first,
            "machine_max_speed_e" => c.machine_max_speed_e = first,
            "machine_max_jerk_x" => c.machine_max_jerk_x = first,
            "machine_max_jerk_y" => c.machine_max_jerk_y = first,
            "machine_max_jerk_z" => c.machine_max_jerk_z = first,
            "machine_max_jerk_e" => c.machine_max_jerk_e = first,
            "machine_min_extruding_rate" => c.machine_min_extruding_rate = first,
            "machine_min_travel_rate" => c.machine_min_travel_rate = first,
            "filament_diameter" => {
                c.filament_diameters = list.clone();
                c.filament_diameter = first;
            }
            "filament_density" => {
                c.filament_densities = list.clone();
                c.filament_density = first;
            }
            "nozzle_volume" => c.nozzle_volumes = list.clone(),
            "machine_load_filament_time" => c.machine_load_filament_time = first,
            "machine_unload_filament_time" => c.machine_unload_filament_time = first,
            "machine_switch_extruder_time" => c.machine_switch_extruder_time = first,
            "machine_hotend_change_time" => c.machine_hotend_change_time = first,
            "machine_prepare_compensation_time" => c.machine_prepare_compensation_time = first,
            "extruder_offset" => {
                let xy: Vec<f64> = v.split('x').map(|x| x.parse().unwrap()).collect();
                c.extruder_offset_x = xy[0];
                c.extruder_offset_y = xy[1];
            }
            "filament_map" => c.filament_map = v.split(',').map(|x| x.parse().unwrap()).collect(),
            "initial_layer_print_height" => c.first_layer_height = first,
            other => panic!("config key {other}"),
        }
    }
    c
}

/// The lines of a case as the port gives them, in gen.cpp's format.
fn rust_lines(c: &Case) -> Vec<String> {
    rust_lines_with_printer(c, None)
}

fn rust_lines_with_printer(c: &Case, is_bbl: Option<bool>) -> Vec<String> {
    let config = rust_config(&c.cfg);
    let mut p = GCodeProcessor::new();
    p.reset();
    if let Some(is_bbl) = is_bbl {
        p.set_is_bbl_printer(is_bbl);
    }
    p.apply_config(&config);
    p.enable_stealth_time_estimator(c.stealth);
    p.enable_machine_envelope_processing(c.envelope);
    p.initialize(&format!("{}.gcode", c.name));
    p.process_buffer(&c.prog);
    p.finalize(false);

    let mut out = Vec::new();
    for mode in [ETimeMode::Normal, ETimeMode::Stealth] {
        let m = mode as usize;
        out.push(format!("time {} {}", m, f32h(p.get_time(mode))));
        out.push(format!("prepare {} {}", m, f32h(p.get_prepare_time(mode))));
        let roles = p.get_roles_time(mode);
        let mut s = format!("roles {} {}", m, roles.len());
        for (r, t) in roles {
            s += &format!(" {} {}", r as u8, f32h(t));
        }
        out.push(s);
        let mt = p.get_moves_time(mode);
        let mut s = format!("mtimes {} {}", m, mt.len());
        for (r, t) in mt {
            s += &format!(" {} {}", r as u8, f32h(t));
        }
        out.push(s);
        let lt = p.get_layers_time(mode);
        let mut s = format!("layers {} {}", m, lt.len());
        for t in lt {
            s += &format!(" {}", f32h(t));
        }
        out.push(s);
        let cg = p.get_custom_gcode_times(mode, true);
        let mut s = format!("cgt {} {}", m, cg.len());
        for (r, (a, b)) in cg {
            s += &format!(" {} {} {}", r as u8, f32h(a), f32h(b));
        }
        out.push(s);
    }
    let ps = &p.result().print_statistics;
    let vols = |kind: &str, m: &std::collections::BTreeMap<usize, f64>| {
        let mut s = format!("vol {} {}", kind, m.len());
        for (k, v) in m {
            s += &format!(" {} {}", k, f64h(*v));
        }
        s
    };
    out.push(vols("total", &ps.total_volumes_per_extruder));
    out.push(vols("model", &ps.model_volumes_per_extruder));
    out.push(vols("support", &ps.support_volumes_per_extruder));
    out.push(vols("wipe", &ps.wipe_tower_volumes_per_extruder));
    out.push(vols("flush", &ps.flush_per_filament));
    let moves = &p.result().moves;
    out.push(format!("moves {}", moves.len()));
    for mv in moves {
        out.push(format!(
            "m {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {}",
            mv.move_type as u8,
            mv.extrusion_role as u8,
            mv.extruder_id,
            mv.cp_color_id,
            mv.gcode_id,
            f32h(mv.position[0]),
            f32h(mv.position[1]),
            f32h(mv.position[2]),
            f32h(mv.delta_extruder),
            f32h(mv.feedrate),
            f32h(mv.width),
            f32h(mv.height),
            f32h(mv.mm3_per_mm),
            f32h(mv.fan_speed),
            f32h(mv.temperature),
            f32h(mv.layer_duration),
            f32h(mv.time[0]),
            f32h(mv.time[1]),
            f32h(mv.additional_fan_speed)
        ));
    }
    out.push(format!("post {}", hex(&p.time_processor().post_process(&c.prog))));
    out
}

/// GCodeProcessor.cpp 701–720, 5317–5325. Export's printer kind controls
/// both plain bed leveling and the time placeholder for each active mode.
#[test]
fn exported_printer_modes_match_native() {
    let inputs = cases(include_str!("data/g_code_processor/printer_mode.txt"));
    assert_eq!(inputs.len(), 8);
    for case in &inputs {
        let got = rust_lines_with_printer(case, Some(case.name.starts_with("bambu_")));
        assert_eq!(got.len(), case.lines.len());
        for (line, native) in got.iter().zip(&case.lines) {
            assert_eq!(line, native, "{}", case.name);
        }
    }
}

#[test]
fn simultaneous_processors_retain_their_printer_kind() {
    let inputs = cases(include_str!("data/g_code_processor/printer_mode.txt"));
    let mut jobs = Vec::new();
    for name in ["other_plain_one", "bambu_plain_one"] {
        let case = inputs.iter().find(|case| case.name == name).unwrap();
        let mut p = GCodeProcessor::new();
        p.apply_config(&rust_config(&case.cfg));
        p.set_is_bbl_printer(name.starts_with("bambu_"));
        jobs.push((p, case));
    }
    for (mut p, case) in jobs {
        p.process_gcode(&case.prog);
        let native = case.lines.iter().find_map(|row| row.strip_prefix("time 0 ")).unwrap();
        assert_eq!(f32h(p.get_time(ETimeMode::Normal)), native, "{}", case.name);
    }
}

/// Ordinary and virtual dispatch, G2/G3's shared handler, and T's prefix match.
/// GCodeProcessor.cpp 1699–1783, 3096–3110.
#[test]
fn command_aliases_match_native() {
    let input = include_str!("data/g_code_processor/command_aliases.txt");
    let cases = cases(input);
    assert_eq!(cases.len(), 4);
    for case in &cases {
        assert_eq!(rust_lines(case), case.lines, "{}", case.name);
    }
}

#[test]
fn programs() {
    let text = read_vectors();
    let cs = cases(&text);
    assert!(cs.len() >= 30, "only {} cases", cs.len());
    let mut report: Vec<String> = Vec::new();
    let mut total = 0usize;
    let mut differ = 0usize;
    let mut compared = 0usize;
    for c in &cs {
        compared += 1;
        let got = rust_lines(c);
        // GCP_CASE=<name>: every line of the case, the C++ one then the port's.
        if std::env::var("GCP_CASE").map(|v| v == c.name).unwrap_or(false) {
            for (i, want) in c.lines.iter().enumerate() {
                let g = got.get(i).cloned().unwrap_or_default();
                eprintln!("{} {}\n{} {}", if &g == want { "=" } else { "C" }, want, if &g == want { "=" } else { "R" }, g);
            }
        }
        let mut first: Option<String> = None;
        let mut n_bad = 0usize;
        for (i, want) in c.lines.iter().enumerate() {
            total += 1;
            let g = got.get(i).cloned().unwrap_or_default();
            if &g != want {
                n_bad += 1;
                if first.is_none() {
                    first = Some(format!("  want {want}\n  got  {g}"));
                }
            }
        }
        if got.len() != c.lines.len() {
            n_bad += 1;
        }
        if n_bad > 0 {
            differ += n_bad;
            report.push(format!("{}: {} of {} lines differ\n{}", c.name, n_bad, c.lines.len(), first.unwrap_or_default()));
        }
    }
    assert!(compared >= 90, "only {compared} cases compared");
    assert!(report.is_empty(), "{} of {} program lines differ:\n{}", differ, total, report.join("\n"));
}

/// GCodeProcessor.cpp:675-703, 1019-1032, 1316-1325. The native file pass
/// expands the two progress tags and renumbers moves by the inserted lines.
#[test]
fn progress_markers_match_native() {
    let rows = include_str!("data/g_code_processor/progress_markers.txt");
    assert_eq!(rows.lines().count(), 18);
    for row in rows.lines() {
        let mut fields = row.split_whitespace();
        let stealth = fields.next().unwrap() == "1";
        let input = unhex(fields.next().unwrap());
        let expected_text = unhex(fields.next().unwrap());
        let expected_ids: Vec<u32> = fields.map(|field| field.parse().unwrap()).collect();

        let mut processor = GCodeProcessor::new();
        processor.reset();
        processor.apply_config(&slicer::print_config::PrintConfig::default());
        processor.enable_stealth_time_estimator(stealth);
        processor.initialize("progress.gcode");
        processor.process_buffer(&input);
        processor.finalize(false);
        let mut offsets = Vec::new();
        let text = processor.time_processor().post_process_first_pass(&input, Some(&mut offsets));
        assert_eq!(text, expected_text, "stealth={stealth}, input={input:?}");
        let ids: Vec<_> = processor.result().moves.iter().map(|m| {
            let added: u32 = offsets.iter().filter(|(line, _)| *line <= m.gcode_id).map(|(_, n)| *n).sum();
            m.gcode_id + added
        }).collect();
        assert_eq!(ids, expected_ids, "stealth={stealth}, input={input:?}");
    }
}

/// GCodeProcessor.cpp:3597-3614. Compare pending and corrected option
/// positions, the order of the moves and the stop time in each estimation mode.
#[test]
fn pause_and_custom_tags_match_native() {
    let vectors = include_str!("data/g_code_processor/option_tags.txt");
    let (settings, rows) = vectors.split_once("begin\n").unwrap();
    let profile = settings.lines().map(|line| {
        let (key, value) = line.split_once(' ').unwrap();
        (key.to_owned(), unhex(value))
    }).collect::<Vec<_>>();
    assert_eq!(rows.lines().count(), 20);
    let mut config = rust_config(&profile);
    config.filament_diameters = vec![1.75; 2];
    let mut processor = GCodeProcessor::new();
    for row in rows.lines() {
        let mut fields = row.split_whitespace();
        let finish = fields.next().unwrap() == "1";
        let input = unhex(fields.next().unwrap());
        processor.reset();
        processor.apply_config(&config);
        processor.enable_stealth_time_estimator(true);
        processor.initialize("options.gcode");
        processor.process_buffer(&input);
        if finish { processor.finalize(false); }

        let result = processor.result();
        let mut actual = vec![result.custom_gcode_per_print_z.len().to_string()];
        for item in &result.custom_gcode_per_print_z {
            actual.extend([f64h(item.print_z), (item.gcode_type as i32).to_string(), item.extruder.to_string(),
                           hex(&item.color), hex(&item.extra)]);
        }
        actual.push(result.moves.len().to_string());
        for m in &result.moves {
            actual.extend([m.gcode_id.to_string(), (m.move_type as i32).to_string(),
                           f32h(m.position[0]), f32h(m.position[1]), f32h(m.position[2]), f32h(m.height)]);
        }
        for machine in &processor.time_processor().machines {
            actual.push(machine.stop_times.len().to_string());
            for stop in &machine.stop_times {
                actual.extend([stop.g1_line_id.to_string(), f32h(stop.elapsed_time)]);
            }
        }
        assert_eq!(actual, fields.collect::<Vec<_>>(), "finish={finish}, input={input:?}");
    }
}

/// GCodeProcessor.cpp 3525-3595. Exercise the preview color sequence and the
/// stop and volume recorded for the active filament, including the 255 wrap.
#[test]
fn color_tags_match_native() {
    let vectors = include_str!("data/g_code_processor/color_tags.txt");
    let (settings, rows) = vectors.split_once("begin\n").unwrap();
    let profile: Vec<_> = settings.lines().map(|line| {
        let (key, value) = line.split_once(' ').unwrap();
        (key.to_owned(), unhex(value))
    }).collect();
    let mut config = rust_config(&profile);
    config.filament_diameters = vec![1.75; 2];
    let mut processor = GCodeProcessor::new();
    assert_eq!(rows.lines().count(), 13);
    for (case, row) in rows.lines().enumerate() {
        let mut fields = row.split_whitespace();
        let input = unhex(fields.next().unwrap());
        processor.reset();
        processor.apply_config(&config);
        processor.enable_stealth_time_estimator(true);
        processor.initialize("colors.gcode");
        processor.process_buffer(&input);
        processor.finalize(false);

        let result = processor.result();
        let mut actual = vec![result.custom_gcode_per_print_z.len().to_string()];
        for item in &result.custom_gcode_per_print_z {
            actual.extend([f64h(item.print_z), (item.gcode_type as i32).to_string(),
                           item.extruder.to_string(), hex(&item.color)]);
        }
        actual.push(result.moves.len().to_string());
        for m in &result.moves {
            actual.extend([m.gcode_id.to_string(), (m.move_type as i32).to_string(), m.cp_color_id.to_string()]);
        }
        let volumes = &result.print_statistics.volumes_per_color_change;
        actual.push(volumes.len().to_string());
        actual.extend(volumes.iter().map(|&v| f64h(v)));
        for machine in &processor.time_processor().machines {
            actual.push(machine.stop_times.len().to_string());
            for stop in &machine.stop_times {
                actual.extend([stop.g1_line_id.to_string(), f32h(stop.elapsed_time)]);
            }
            let custom = &machine.gcode_time;
            actual.extend([u8::from(custom.needed).to_string(), f32h(custom.cache), custom.times.len().to_string()]);
            for &(code, duration) in &custom.times {
                actual.extend([(code as i32).to_string(), f32h(duration)]);
            }
        }
        assert_eq!(actual, fields.collect::<Vec<_>>(), "color case {case}");
    }
}

/// GCodeProcessor.cpp 513-542, 2858-2866, 5794-5811. A hardware pause
/// records the current motion and layer, then resolves its time and percentage.
#[test]
fn hardware_pause_times_match_native() {
    let vectors = include_str!("data/g_code_processor/pause_times.txt");
    let (settings, rows) = vectors.split_once("begin\n").unwrap();
    let profile: Vec<_> = settings.lines().map(|line| {
        let (key, value) = line.split_once(' ').unwrap();
        (key.to_owned(), unhex(value))
    }).collect();
    let config = rust_config(&profile);
    let mut processor = GCodeProcessor::new();
    assert_eq!(rows.lines().count(), 56);
    for (case, row) in rows.lines().enumerate() {
        let mut fields = row.split_whitespace();
        let stealth = fields.next().unwrap() == "1";
        let finalized = fields.next().unwrap() == "1";
        let input = unhex(fields.next().unwrap());
        processor.reset();
        processor.apply_config(&config);
        processor.enable_stealth_time_estimator(stealth);
        processor.initialize("pause.gcode");
        processor.process_buffer(&input);
        if finalized { processor.finalize(false); }

        let mut actual = Vec::new();
        for m in &processor.time_processor().machines {
            actual.extend([f32h(m.time), m.g1_times_cache.len().to_string()]);
            for g in &m.g1_times_cache {
                actual.extend([g.id.to_string(), f32h(g.elapsed_time)]);
            }
            actual.extend([m.stop_times.len().to_string(), m.pause_times.len().to_string()]);
            for t in &m.pause_times {
                actual.extend([t.g1_line_id.to_string(), f32h(t.elapsed_time), t.layer_id.to_string()]);
            }
        }
        let pauses = &processor.result().pause_printing;
        actual.push(pauses.len().to_string());
        for p in pauses {
            actual.extend([p.percent.to_string(), p.remaining_time.to_string(), p.layer_id.to_string()]);
        }
        assert_eq!(actual, fields.collect::<Vec<_>>(), "hardware pause case {case}, input={input:?}");
    }
}

/// GCodeProcessor.cpp 2881–2885, 6456–6507. A height visited twice forms
/// two runs when the preview slider has more ticks than the logical layer list.
#[test]
fn finalized_preview_layer_times_match_native() {
    let vectors = include_str!("data/g_code_processor/preview_layer_times.txt");
    let (settings, rows) = vectors.split_once("begin\n").unwrap();
    let profile: Vec<_> = settings.lines().map(|line| {
        let (key, value) = line.split_once(' ').unwrap();
        (key.to_owned(), unhex(value))
    }).collect();
    let mut config = rust_config(&profile);
    let mut processor = GCodeProcessor::new();
    let temp = tempfile::tempdir().unwrap();
    let filename = temp.path().join("preview.gcode");
    assert_eq!(rows.lines().count(), 120);
    for (case, row) in rows.lines().enumerate() {
        let mut fields = row.split_whitespace();
        let tags = fields.next().unwrap();
        config.spiral_vase = tags == "spiral_mode";
        config.has_scarf_joint_seam = tags == "has_scarf_joint_seam";
        let stealth = fields.next().unwrap() == "1";
        let post = fields.next().unwrap() == "1";
        let input = unhex(fields.next().unwrap());
        processor.reset();
        processor.apply_config(&config);
        processor.enable_stealth_time_estimator(stealth);
        if post { std::fs::write(&filename, &input).unwrap(); }
        processor.initialize(&filename.to_string_lossy());
        processor.process_buffer(&input);
        processor.finalize(post);

        let mut actual = Vec::new();
        for mode in &processor.result().print_statistics.modes {
            actual.extend([f32h(mode.time), mode.layers_times.len().to_string()]);
            actual.extend(mode.layers_times.iter().copied().map(f32h));
            actual.push(mode.preview_layers_times.len().to_string());
            actual.extend(mode.preview_layers_times.iter().copied().map(f32h));
        }
        assert_eq!(actual, fields.collect::<Vec<_>>(), "preview case {case}, input={input:?}");
    }
}
