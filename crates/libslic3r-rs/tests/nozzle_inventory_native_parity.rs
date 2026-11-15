//! `nozzle_inventory.rs` against Bambu Studio's `NozzleInventory.cpp` at the pin.
//!
//! `tests/data/nozzle_inventory/vectors.txt` is written by `gen.cpp` of that directory, which
//! compiles the C++ unit unchanged and drives it through its public interface (how to build it is
//! in its header). Each line is one operation with its arguments and the FNV-1a 64 hash of the
//! state text after it: the log records the operation wrote, the mirror config, the history and
//! every read of the inventory. The test replays the operations and requires the same hash; set
//! `NOZZLE_INVENTORY_FULL=1` to print the state text of every operation, to compare with
//! `gen full`.

use std::collections::BTreeMap;
use std::fmt::Write as _;
use std::sync::{Arc, Mutex};

use slicer::app_config::AppConfig;
use slicer::log::{flush_logs, set_log_path_and_level, LogEncOptions, LogEncType};
use slicer::nozzle_inventory::*;

const VECTORS: &str = include_str!("data/nozzle_inventory/vectors.txt");

struct Tok<'a>(std::str::SplitWhitespace<'a>);

impl<'a> Tok<'a> {
    fn s(&mut self) -> &'a str {
        self.0.next().expect("token")
    }
    fn i(&mut self) -> i32 {
        self.s().parse().expect("integer")
    }
    fn u(&mut self) -> usize {
        self.s().parse().expect("count")
    }
    /// A string token: `~` and the text, with ' ' and '%' percent-encoded.
    fn string(&mut self) -> String {
        decode(self.s())
    }
    fn config(&mut self) -> NozzlePrintConfig {
        assert_eq!(self.s(), "C");
        let mut config = NozzlePrintConfig::default();
        for _ in 0..self.u() {
            let key = self.s();
            match self.s() {
                "s" => {
                    let value = Some(self.string());
                    match key {
                        "printer_model" => config.printer_model = value,
                        "printer_variant" => config.printer_variant = value,
                        _ => panic!("string key {key}"),
                    }
                }
                "b" => {
                    let value = Some(self.i() != 0);
                    match key {
                        "handle_hotend_as_extruder" => config.handle_hotend_as_extruder = value,
                        "multi_nozzle_diameter_mode" => config.multi_nozzle_diameter_mode = value,
                        _ => panic!("bool key {key}"),
                    }
                }
                "I" => {
                    let n = self.u();
                    let value = Some((0..n).map(|_| self.i()).collect::<Vec<i32>>());
                    match key {
                        "default_nozzle_volume_type" => config.default_nozzle_volume_type = value,
                        "extruder_max_nozzle_count" => config.extruder_max_nozzle_count = value,
                        "extruder_nozzle_count" => config.extruder_nozzle_count = value,
                        "extruder_nozzle_slot" => config.extruder_nozzle_slot = value,
                        "extruder_nozzle_volume_type" => config.extruder_nozzle_volume_type = value,
                        "nozzle_volume_type" => config.nozzle_volume_type = value,
                        _ => panic!("ints key {key}"),
                    }
                }
                "F" => {
                    let n = self.u();
                    let value =
                        Some((0..n).map(|_| f64::from_bits(u64::from_str_radix(self.s(), 16).expect("bits"))).collect::<Vec<f64>>());
                    match key {
                        "extruder_nozzle_diameter" => config.extruder_nozzle_diameter = value,
                        "nozzle_diameter" => config.nozzle_diameter = value,
                        _ => panic!("floats key {key}"),
                    }
                }
                "S" => {
                    let n = self.u();
                    let value = Some((0..n).map(|_| self.string()).collect::<Vec<String>>());
                    match key {
                        "extruder_nozzle_stats" => config.extruder_nozzle_stats = value,
                        "extruder_nozzle_stats_new" => config.extruder_nozzle_stats_new = value,
                        _ => panic!("strings key {key}"),
                    }
                }
                other => panic!("type {other}"),
            }
        }
        config
    }
    fn printer(&mut self) -> PrinterPresetSelection {
        let selected_preset_name = self.string();
        let base = self.s();
        let preset_base_name = if base == "-" { None } else { Some(decode(base)) };
        PrinterPresetSelection { selected_preset_name, preset_base_name, edited_preset_config: self.config() }
    }
    fn slots(&mut self) -> ExtruderSlots {
        let n = self.u();
        (0..n)
            .map(|_| {
                let diameter = self.string();
                let volume_type = NozzleVolumeType(self.i());
                let status = match self.i() {
                    0 => NozzleSlotStatus::Empty,
                    1 => NozzleSlotStatus::Normal,
                    2 => NozzleSlotStatus::Error,
                    _ => NozzleSlotStatus::Unknown,
                };
                NozzleSlot { diameter, volume_type, status }
            })
            .collect()
    }
}

fn decode(token: &str) -> String {
    token.strip_prefix('~').expect("string token").replace("%20", " ").replace("%25", "%")
}

fn ints(values: &[i32]) -> String {
    values.iter().map(|v| v.to_string()).collect::<Vec<_>>().join(",")
}

fn floats(values: &[f64]) -> String {
    values.iter().map(|v| format!("{:016x}", v.to_bits())).collect::<Vec<_>>().join(",")
}

fn strings(values: &[String]) -> String {
    values.iter().map(|v| format!("<{v}>")).collect::<Vec<_>>().join(",")
}

/// `cfg_text` of gen.cpp: `key=values;` per key in key order.
fn cfg_text(c: &NozzlePrintConfig) -> String {
    let mut o = String::new();
    let mut put = |key: &str, value: Option<String>| {
        if let Some(value) = value {
            let _ = write!(o, "{key}={value};");
        }
    };
    put("default_nozzle_volume_type", c.default_nozzle_volume_type.as_deref().map(ints));
    put("extruder_max_nozzle_count", c.extruder_max_nozzle_count.as_deref().map(ints));
    put("extruder_nozzle_count", c.extruder_nozzle_count.as_deref().map(ints));
    put("extruder_nozzle_diameter", c.extruder_nozzle_diameter.as_deref().map(floats));
    put("extruder_nozzle_slot", c.extruder_nozzle_slot.as_deref().map(ints));
    put("extruder_nozzle_stats", c.extruder_nozzle_stats.as_deref().map(strings));
    put("extruder_nozzle_stats_new", c.extruder_nozzle_stats_new.as_deref().map(strings));
    put("extruder_nozzle_volume_type", c.extruder_nozzle_volume_type.as_deref().map(ints));
    put("handle_hotend_as_extruder", c.handle_hotend_as_extruder.map(|v| (v as i32).to_string()));
    put("multi_nozzle_diameter_mode", c.multi_nozzle_diameter_mode.map(|v| (v as i32).to_string()));
    put("nozzle_diameter", c.nozzle_diameter.as_deref().map(floats));
    put("nozzle_volume_type", c.nozzle_volume_type.as_deref().map(ints));
    put("printer_model", c.printer_model.clone());
    put("printer_variant", c.printer_variant.clone());
    o
}

fn counts_text(counts: &BTreeMap<NozzleVolumeType, i32>) -> String {
    counts.iter().map(|(type_, count)| format!("{}:{}", type_.0, count)).collect::<Vec<_>>().join(",")
}

/// The log file of the test: the records are read back from it after every operation.
struct LogFile {
    dir: std::path::PathBuf,
    read: usize,
}

impl LogFile {
    /// The records written since the last call, as `LOG <level>: <message>` lines. A record in the
    /// file is `<time> [<thread>] [<level>] [<file>:<line> <function>] <message>`; the location is
    /// that of the Rust source, so it is left out.
    fn take(&mut self) -> String {
        flush_logs();
        let mut files: Vec<_> = std::fs::read_dir(&self.dir).expect("log dir").map(|entry| entry.expect("entry").path()).collect();
        files.sort();
        let mut out = String::new();
        let Some(file) = files.first() else {
            return out;
        };
        assert_eq!(files.len(), 1, "one log file");
        let text = std::fs::read_to_string(file).expect("log file");
        for line in text[self.read..].lines() {
            let (_, rest) = line.split_once("] [").expect("thread id");
            let (level, rest) = rest.split_once("] [").expect("severity");
            let (_, message) = rest.split_once("] ").expect("location");
            let _ = writeln!(out, "LOG {}: {}", level.trim_end(), message);
        }
        self.read = text.len();
        out
    }
}

/// `state_text` of gen.cpp.
fn state_text(log: &mut LogFile, inv: &NozzleInventory, mirror: &NozzlePrintConfig, history: Option<&AppConfig>) -> String {
    let mut o = log.take();
    let _ = writeln!(o, "MIRROR {}", cfg_text(mirror));
    o += "HIST";
    match history {
        None => o += " none",
        Some(history) => {
            for section in ["nozzle_slots", "nozzle_volume_types"] {
                if history.has_section(section) {
                    for (key, value) in history.get_section(section) {
                        let _ = write!(o, " {section}/{key}={value};");
                    }
                }
            }
        }
    }
    o += "\n";
    let n = inv.extruder_count();
    let _ = writeln!(o, "EC {n}");
    for eid in -1..=n {
        let _ = write!(o, "E{eid} slots=");
        let slots = inv.slots(eid);
        for (i, slot) in slots.iter().enumerate() {
            let _ = write!(o, "{}{}/{}/{}", if i != 0 { "|" } else { "" }, slot.diameter, slot.volume_type.0, slot.status as i32);
        }
        let _ = write!(o, " counts={}", counts_text(&inv.counts(eid, None)));
        let _ = write!(o, " c04={}", counts_text(&inv.counts(eid, Some("0.4"))));
        let _ = write!(o, " c06={}", counts_text(&inv.counts(eid, Some("0.6"))));
        let _ = write!(o, " c040={}", counts_text(&inv.counts(eid, Some("0.40"))));
        let _ = write!(o, " cempty={}", counts_text(&inv.counts(eid, Some(""))));
        let _ = write!(o, " n={}", inv.count(eid, None, None));
        let _ = write!(o, " nS={}", inv.count(eid, Some(NVT_STANDARD), None));
        let _ = write!(o, " nH={}", inv.count(eid, Some(NVT_HYBRID), None));
        let _ = write!(o, " nHF04={}", inv.count(eid, Some(NVT_HIGH_FLOW), Some("0.4")));
        let _ = writeln!(o, " vt={}", inv.volume_type(eid).0);
    }
    o
}

fn fnv1a(text: &str) -> u64 {
    let mut h: u64 = 0xcbf29ce484222325;
    for byte in text.bytes() {
        h ^= byte as u64;
        h = h.wrapping_mul(0x100000001b3);
    }
    h
}

#[test]
fn nozzle_inventory_matches_cpp() {
    let dir = tempfile::tempdir().expect("temp dir");
    let enc = LogEncOptions { enc_type: LogEncType::LogEncNone, enc_key_url: String::new(), enc_key_host_env: String::new() };
    // Level 5: every record is written, as gen.cpp captures every record.
    set_log_path_and_level(dir.path().to_str().expect("utf-8 path"), 5, &enc);
    let mut log = LogFile { dir: dir.path().to_path_buf(), read: 0 };
    let full = std::env::var("NOZZLE_INVENTORY_FULL").is_ok_and(|v| v == "1");

    let mut cases = 0usize;
    let mut ops = 0usize;
    let mut mirror = Arc::new(Mutex::new(NozzlePrintConfig::default()));
    let mut history = Arc::new(Mutex::new(AppConfig::new()));
    let mut has_history = false;
    let mut inv = NozzleInventory::new(mirror.clone());

    let mut lines = VECTORS.lines().peekable();
    while let Some(line) = lines.next() {
        if line == "CASE" {
            cases += 1;
            mirror = Arc::new(Mutex::new(NozzlePrintConfig::default()));
            history = Arc::new(Mutex::new(AppConfig::new()));
            has_history = false;
            inv = NozzleInventory::new(mirror.clone());
            continue;
        }
        let (op, expected) = line.split_once(" -> ").expect("operation and hash");
        let expected = u64::from_str_radix(expected, 16).expect("hash");
        let mut t = Tok(op.split_whitespace());
        let name = t.s();
        let mut scratch: Option<(NozzleInventory, Arc<Mutex<NozzlePrintConfig>>)> = None;
        match name {
            "mirror_init" => {
                // the project config as it is when the inventory is made
                *mirror.lock().unwrap() = t.config();
            }
            "set_history" => {
                has_history = t.i() != 0;
                {
                    let mut history = history.lock().unwrap();
                    for section in ["nozzle_volume_types", "nozzle_slots"] {
                        let mut data = BTreeMap::new();
                        for _ in 0..t.u() {
                            let key = t.string();
                            data.insert(key, t.string());
                        }
                        history.set_section(section, data);
                    }
                }
                inv.set_history(if has_history { Some(history.clone()) } else { None });
            }
            "select_printer" => inv.select_printer(&t.printer()),
            "open_project" => {
                let printer = t.printer();
                inv.open_project(&printer, &t.config());
            }
            "close_project" => inv.close_project(),
            "sync_machine" => {
                let model = t.string();
                let n = t.u();
                let layout: Vec<ExtruderSlots> = (0..n).map(|_| t.slots()).collect();
                inv.sync_machine(&model, layout);
            }
            "user_set_slots" => {
                let eid = t.i();
                inv.user_set_slots(eid, t.slots());
            }
            "user_set_counts" => {
                let eid = t.i();
                let mut counts = BTreeMap::new();
                for _ in 0..t.u() {
                    let type_ = NozzleVolumeType(t.i());
                    counts.insert(type_, t.i());
                }
                inv.user_set_counts(eid, &counts);
            }
            "user_set_volume_type" => {
                let eid = t.i();
                inv.user_set_volume_type(eid, NozzleVolumeType(t.i()));
            }
            "user_unify_diameter" => inv.user_unify_diameter(&t.string()),
            "mirror_set" => {
                // a bulk copy into the mirror replaces the nozzle keys
                *mirror.lock().unwrap() = t.config();
            }
            "refresh_mirror" => inv.refresh_mirror(),
            "assign" => {
                // a scratch copy: its own mirror, no history
                let scratch_mirror = Arc::new(Mutex::new(t.config()));
                let mut copy = NozzleInventory::new(scratch_mirror.clone());
                copy.assign(&inv);
                copy.refresh_mirror();
                scratch = Some((copy, scratch_mirror));
            }
            other => panic!("operation {other}"),
        }
        assert!(t.0.next().is_none(), "tokens left in: {op}");

        let text = match scratch.as_ref() {
            Some((copy, scratch_mirror)) => {
                let mirror_now = scratch_mirror.lock().unwrap().clone();
                state_text(&mut log, copy, &mirror_now, None)
            }
            None => {
                let mirror_now = mirror.lock().unwrap().clone();
                let history = history.lock().unwrap();
                state_text(&mut log, &inv, &mirror_now, if has_history { Some(&history) } else { None })
            }
        };
        if full {
            print!("{op}\n{text}");
        }
        assert_eq!(fnv1a(&text), expected, "case {cases}, operation: {op}\n{text}");
        ops += 1;
    }
    assert_eq!(cases, 255);
    assert!(ops > 2000, "{ops} operations");
    println!("nozzle_inventory: {cases} cases, {ops} operations equal to the C++");
}
