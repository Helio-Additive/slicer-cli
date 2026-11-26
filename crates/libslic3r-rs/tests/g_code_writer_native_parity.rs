//! GCodeWriter.hpp / GCodeWriter.cpp (and the Extruder they drive) against the pin: the vectors of
//! tests/data/g_code_writer/gen.cpp. Every case applies a configuration to a new writer
//! (`apply_print_config`), sets its extruders and replays the generated program of writer calls;
//! every call's G-code and the writer's state after it must be byte-equal to the C++ writer's.

use slicer::config::ConfigBase;
use slicer::g_code_writer::*;
use slicer::geometry::{Vec2d, Vec3d};
use slicer::print_config::config_def::DynamicPrintConfig;
use slicer::print_config::GCodeFlavor;
use std::io::Read;

fn read_vectors() -> String {
    let path = format!("{}/tests/data/g_code_writer/g_code_writer.txt.gz", env!("CARGO_MANIFEST_DIR"));
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

fn hd(v: f64) -> String {
    format!("{:016x}", v.to_bits())
}

fn hf(v: f32) -> String {
    format!("{:08x}", v.to_bits())
}

fn d(t: &str) -> f64 {
    f64::from_bits(u64::from_str_radix(t, 16).expect("f64 bits"))
}

fn f(t: &str) -> f32 {
    f32::from_bits(u32::from_str_radix(t, 16).expect("f32 bits"))
}

fn b(t: &str) -> bool {
    t == "1"
}

const COMMENTS: [&str; 4] = ["", "travel", "move to first layer point", "wipe and retract"];
const MARKERS: [&str; 3] = ["", ";_EXTRUDE_SET_SPEED", ";_WIPE"];

fn lift_type(v: &str) -> LiftType {
    match v {
        "0" => LiftType::NormalLift,
        "1" => LiftType::SlopeLift,
        _ => LiftType::SpiralLift,
    }
}

fn lift_type_code(t: LiftType) -> i32 {
    match t {
        LiftType::NormalLift => 0,
        LiftType::SlopeLift => 1,
        LiftType::SpiralLift => 2,
    }
}

fn flavor_code(f: GCodeFlavor) -> i32 {
    (0..13).find(|&i| slicer::extruder::gcode_flavor_from_int(i) == f).expect("a flavor")
}

fn state(w: &GCodeWriter) -> String {
    let m = w.parity_members();
    let p = w.get_position();
    let mut s = format!(
        "{} {} {} {} {} {} {} {} {} {} {} {} {} {}",
        hd(p.x),
        hd(p.y),
        hd(p.z),
        hd(m.lifted),
        hd(m.to_lift),
        lift_type_code(m.to_lift_type),
        m.curr_extruder_id,
        m.current_process_config_idx,
        m.last_acceleration,
        hd(m.last_jerk),
        m.is_current_pos_clear as i32,
        hd(m.current_speed),
        m.last_bed_temperature,
        m.last_bed_temperature_reached as i32
    );
    for e in w.extruders() {
        s += &format!(
            " {} {} {} {} {}",
            hd(e.e()),
            hd(e.used_filament()),
            hd(e.extruded_volume()),
            // m_retracted of a shared extruder: never set in the C++ (see gen.cpp), not compared
            if e.is_share_extruder() { "-".to_string() } else { hd(e.get_single_retracted_length()) },
            hd(e.get_share_retracted_length())
        );
    }
    s
}

fn cfg_line(w: &GCodeWriter) -> String {
    let m = w.parity_members();
    let mut s = format!(
        "cfg {} {} {} {} {} {} {} {} {} {}",
        m.max_acceleration,
        hd(m.max_jerk),
        m.single_extruder_multi_material as i32,
        w.multiple_extruders as i32,
        m.bed_bbox_valid as i32,
        hd(m.bed_min.x),
        hd(m.bed_min.y),
        hd(m.bed_max.x),
        hd(m.bed_max.y),
        m.extruder_bed_valid.len()
    );
    for i in 0..m.extruder_bed_valid.len() {
        s += &format!(
            " {} {} {} {} {}",
            m.extruder_bed_valid[i],
            hd(m.extruder_bed_min[i].x),
            hd(m.extruder_bed_min[i].y),
            hd(m.extruder_bed_max[i].x),
            hd(m.extruder_bed_max[i].y)
        );
    }
    s += &format!(" {} {}", flavor_code(w.config.gcode_flavor), w.extruders().len());
    for e in w.extruders() {
        s += &format!(
            " {} {} {} {} {} {} {} {} {} {} {}",
            e.id(),
            e.extruder_id(),
            e.exclusive_nozzle() as i32,
            hd(e.e_per_mm3()),
            hd(e.retraction_length()),
            hd(e.retract_length_toolchange()),
            hd(e.retract_restart_extra_toolchange()),
            e.retract_speed(),
            e.deretract_speed(),
            hd(e.retract_before_wipe()),
            hd(e.retract_restart_extra())
        );
    }
    s
}

/// Runs the call of `head` (the tokens before `=>`) on the writer: its output, and the head as
/// the C++ printed it (some calls print a result into it).
fn run_op(w: &mut GCodeWriter, t: &[&str]) -> (String, String) {
    let head = t.join(" ");
    let out = match t[0] {
        "pre" => w.preamble(),
        "post" => w.postamble(),
        "init" => {
            w.init_extruder(t[1].parse().unwrap(), t[2].parse().unwrap());
            String::new()
        }
        "tc" => w.toolchange(t[1].parse().unwrap(), t[2].parse().unwrap()),
        "setext" => w.set_extruder(t[1].parse().unwrap(), t[2].parse().unwrap()),
        "temp" => {
            let ct = if t[4] == "1" { TemperatureCommandType::LayerChange } else { TemperatureCommandType::Default };
            w.set_temperature(t[1].parse().unwrap(), b(t[2]), t[3].parse().unwrap(), ct)
        }
        "bed" => w.set_bed_temperature(t[1].parse().unwrap(), b(t[2])),
        "chamber" => w.set_chamber_temperature(t[1].parse().unwrap(), b(t[2])),
        "acc" => {
            w.set_acceleration(t[1].parse().unwrap());
            String::new()
        }
        "tacc" | "tsacc" | "flacc" => {
            let n: usize = t[1].parse().unwrap();
            let v: Vec<u32> = (0..n).map(|i| t[2 + i].parse().unwrap()).collect();
            match t[0] {
                "tacc" => w.set_travel_acceleration(&v),
                "tsacc" => w.set_travel_short_acceleration(&v),
                _ => w.set_first_layer_travel_acceleration(&v),
            }
            String::new()
        }
        "first" => {
            w.set_first_layer(b(t[1]));
            String::new()
        }
        "resetacc" => {
            w.reset_last_acceleration();
            String::new()
        }
        "pa" => w.set_pressure_advance(d(t[1]), b(t[2])),
        "jerk" => w.set_jerk_xy(d(t[1])),
        "reset_e" => w.reset_e(b(t[1])),
        "prog" => w.update_progress(t[1].parse().unwrap(), t[2].parse().unwrap(), b(t[3])),
        "tcp" => w.toolchange_prefix(),
        "speed" => {
            let ci: usize = t[2].parse().unwrap();
            let mi: usize = t[3].parse().unwrap();
            w.set_speed(d(t[1]), COMMENTS[ci], MARKERS[mi])
        }
        "txy" => {
            let p = Vec2d::new(d(t[1]), d(t[2]));
            let ci: usize = t[3].parse().unwrap();
            if b(t[4]) {
                <GCodeWriter as GCodeWriterTravelToXyShort>::travel_to_xy(w, p, COMMENTS[ci], true)
            } else {
                w.travel_to_xy(p, COMMENTS[ci])
            }
        }
        "txyz" => {
            let p = Vec3d::new(d(t[1]), d(t[2]), d(t[3]));
            let ci: usize = t[4].parse().unwrap();
            if b(t[5]) {
                <GCodeWriter as GCodeWriterTravelToXyzShort>::travel_to_xyz(w, p, COMMENTS[ci], true)
            } else {
                w.travel_to_xyz(p, COMMENTS[ci])
            }
        }
        "tz" => {
            let ci: usize = t[2].parse().unwrap();
            w.travel_to_z(d(t[1]), COMMENTS[ci])
        }
        "lazy" => w.lazy_lift(lift_type(t[1]), b(t[2]), b(t[3])),
        "eager" => w.eager_lift(lift_type(t[1]), b(t[2])),
        "exy" => {
            let ci: usize = t[4].parse().unwrap();
            w.extrude_to_xy(Vec2d::new(d(t[1]), d(t[2])), d(t[3]), COMMENTS[ci], b(t[5]))
        }
        "earc" => {
            let ci: usize = t[7].parse().unwrap();
            w.extrude_arc_to_xy(
                Vec2d::new(d(t[1]), d(t[2])),
                Vec2d::new(d(t[3]), d(t[4])),
                d(t[5]),
                b(t[6]),
                COMMENTS[ci],
                b(t[8]),
            )
        }
        "exyz" => {
            let ci: usize = t[5].parse().unwrap();
            w.extrude_to_xyz(Vec3d::new(d(t[1]), d(t[2]), d(t[3])), d(t[4]), COMMENTS[ci], b(t[6]))
        }
        "retract" => w.retract(b(t[1])),
        "retracttc" => w.retract_for_toolchange(b(t[1])),
        "unretract" => w.unretract(f(t[1])),
        "unlift" => w.unlift(),
        "fan" => w.set_fan(t[1].parse().unwrap()),
        "fanf" => {
            let fl = slicer::extruder::gcode_flavor_from_int(t[1].parse().unwrap());
            <GCodeWriter as GCodeWriterSetFanFlavor>::set_fan(fl, t[2].parse().unwrap())
        }
        "afan" => GCodeWriter::set_additional_fan(t[1].parse().unwrap(), t[2].parse().unwrap()),
        "exfan" => GCodeWriter::set_exhaust_fan(t[1].parse().unwrap(), b(t[2])),
        "offset" => {
            w.set_xy_offset(d(t[1]), d(t[2]));
            String::new()
        }
        "clear" => {
            w.set_current_position_clear(b(t[1]));
            String::new()
        }
        "avoid" => {
            w.set_avoid_z_descent_travel(b(t[1]));
            String::new()
        }
        "bbl" => {
            w.set_is_bbl_printer(b(t[1]));
            String::new()
        }
        "wmz" => {
            let r = w.will_move_z(d(t[1]));
            return (String::new(), format!("wmz {} {}", t[1], r as i32));
        }
        "retlen" => {
            let r = w.get_extruder_retracted_length(t[1].parse().unwrap());
            return (String::new(), format!("retlen {} {}", t[1], hd(r)));
        }
        "labels" => {
            let which: i32 = t[1].parse().unwrap();
            let mut g = "G1 X1 ; ".to_string();
            if which == 0 {
                w.set_object_start_str("M624 AQAAAAAAAAA=\n".to_string());
                w.set_object_end_str("M625\n".to_string());
                w.add_object_change_labels(&mut g);
            } else if which == 1 {
                w.set_object_start_str("M624 AgAAAAAAAAA=\n".to_string());
                w.add_object_start_labels(&mut g);
            } else {
                w.add_object_end_labels(&mut g);
            }
            return (
                g,
                format!("labels {} {} {}", which, w.empty_object_start_str() as i32, w.empty_object_end_str() as i32),
            );
        }
        other => panic!("unknown call {other}"),
    };
    (out, head)
}

#[test]
fn g_code_writer_programs() {
    let text = read_vectors();
    let mut w: Option<GCodeWriter> = None;
    let mut n_cases = 0usize;
    let mut n_ops = 0usize;
    let mut per_op: std::collections::BTreeMap<String, usize> = std::collections::BTreeMap::new();
    let mut bad: Vec<String> = Vec::new();
    for line in text.lines() {
        let t: Vec<&str> = line.split(' ').collect();
        if t[0] == "case" {
            n_cases += 1;
            let n: usize = t[1].parse().unwrap();
            let mut cfg = DynamicPrintConfig::full_print_config();
            for k in 0..n {
                let (key, value) = (t[2 + 2 * k], unhex(t[3 + 2 * k]));
                cfg.set_deserialize_strict(key, &value, false).unwrap_or_else(|e| panic!("{key}: {e:?}"));
            }
            let mut i = 2 + 2 * n;
            let m: usize = t[i].parse().unwrap();
            let ids: Vec<u32> = (0..m).map(|k| t[i + 1 + k].parse().unwrap()).collect();
            i += 1 + m;
            let mut writer = GCodeWriter::new();
            writer.apply_print_config(&cfg);
            writer.set_extruders(ids);
            writer.set_is_bbl_printer(b(t[i]));
            w = Some(writer);
            continue;
        }
        let writer = w.as_mut().expect("a case first");
        if t[0] == "cfg" {
            let got = cfg_line(writer);
            if got != line {
                bad.push(format!("case {n_cases}:\n  want {line}\n  got  {got}"));
            }
            continue;
        }
        let arrow = t.iter().position(|&s| s == "=>").expect("=>");
        let (out, head) = run_op(writer, &t[..arrow]);
        let got = format!("{} => {} {}", head, hex(&out), state(writer));
        n_ops += 1;
        *per_op.entry(t[0].to_string()).or_default() += 1;
        if got != line {
            bad.push(format!("case {n_cases} op {}:\n  want {line}\n  got  {got}", t[0]));
            if bad.len() > 20 {
                break;
            }
        }
    }
    assert!(bad.is_empty(), "{} lines differ (of {} calls in {} cases):\n{}", bad.len(), n_ops, n_cases, bad.join("\n"));
    assert!(n_cases >= 50 && n_ops >= 5000, "only {n_cases} cases, {n_ops} calls: {per_op:?}");
    let _ = hf(0.0);
}

// PrintConfig.cpp 6872–6878 and GCodeWriter.cpp 100–125. Propagate
// the same axis mode through the profile, cooling parser and live header.
#[test]
fn export_extrusion_mode_matches_native_preamble() {
    use slicer::g_code::{GCodeHeader, GCodeStats};
    use slicer::print_config::PrintConfig;

    #[derive(serde::Deserialize)]
    struct Case {
        flavor: String,
        initial: bool,
        input: String,
        accepted: bool,
        mode: bool,
        preamble: String,
    }
    for row in include_str!("data/g_code_writer/axis_modes.jsonl").lines() {
        let case: Case = serde_json::from_str(row).unwrap();
        let mut config = PrintConfig::default().use_relative_e(case.initial);
        config.use_relative_e_distances_cooling = case.initial;
        assert!(config.set_deserialize("gcode_flavor", &case.flavor), "{row}");
        let mut restored = config.clone();
        assert_eq!(config.set_deserialize("use_relative_e_distances", &case.input), case.accepted, "{row}");
        assert_eq!(restored.apply_key_value("use_relative_e_distances", &case.input), case.accepted, "{row}");
        for cfg in [&config, &restored] {
            assert_eq!((cfg.use_relative_e, cfg.use_relative_e_distances_cooling), (case.mode, case.mode), "{row}");
        }
        let values = config.to_key_value_map();
        assert_eq!(values.get("use_relative_e_distances").unwrap(), if case.mode { "1" } else { "0" });
        let header = GCodeHeader::new(GCodeStats::default(), config).generate_executable_block_content();
        let (_, preamble) = header.rsplit_once(" ; set nozzle temperature and wait for it to be reached\n").unwrap();
        assert_eq!(preamble, case.preamble, "{row}");
    }
}
