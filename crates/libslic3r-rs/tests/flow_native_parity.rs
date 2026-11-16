//! `Flow` against Flow.cpp / Flow.hpp at the pin.
//!
//! `tests/data/flow/vectors.txt.gz` is the output of `gen.cpp` beside it, which links the engine's
//! libslic3r and calls the Flow members on generated inputs (the header comment there says how,
//! and describes the lines). Every float and double is compared bit for bit: the static helpers
//! (`auto_extrusion_width`, the rounded-rectangle spacing and width, the bridge spacing), the
//! constructors, `with_width`, `with_height`, `with_spacing`, `with_cross_section` and
//! `with_flow_ratio`, and on each result the members, `mm3_per_mm`, `scaled_width`,
//! `scaled_spacing` and `scaled_elephant_foot_spacing`.

use slicer::config::{ConfigOption, ConfigOptionFloat, ConfigOptionFloatsNullable, ConfigOptionResolver};
use slicer::flow::{Flow, FlowError, FlowRole};
use std::io::Read;

fn read_vectors() -> String {
    let path = format!("{}/tests/data/flow/vectors.txt.gz", env!("CARGO_MANIFEST_DIR"));
    let file = std::fs::File::open(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
    let mut text = String::new();
    flate2::read::GzDecoder::new(file).read_to_string(&mut text).expect("gzip");
    text
}

fn f(hex: &str) -> f64 {
    f32::from_bits(u32::from_str_radix(hex, 16).expect("float bits")) as f64
}

fn d(hex: &str) -> f64 {
    f64::from_bits(u64::from_str_radix(hex, 16).expect("double bits"))
}

fn role(n: &str) -> FlowRole {
    match n {
        "0" => FlowRole::ExternalPerimeter,
        "1" => FlowRole::Perimeter,
        "2" => FlowRole::Infill,
        "3" => FlowRole::SolidInfill,
        "4" => FlowRole::TopSolidInfill,
        "5" => FlowRole::SupportMaterial,
        "6" => FlowRole::SupportMaterialInterface,
        "7" => FlowRole::SupportTransition,
        _ => panic!("role {n}"),
    }
}

fn fbits(v: f64) -> String {
    format!("{:08x}", (v as f32).to_bits())
}

/// The "F" fields of a line (see gen.cpp) for a Rust flow, in the same text form.
fn flow_fields(flow: &Flow) -> String {
    let mm3 = match flow.mm3_per_mm() {
        Ok(v) => format!("{:016x}", v.to_bits()),
        Err(_) => "ERR".to_string(),
    };
    format!(
        "{} {} {} {} {} {} {} {} {}",
        fbits(flow.width()),
        fbits(flow.height()),
        fbits(flow.spacing()),
        fbits(flow.nozzle_diameter()),
        flow.is_bridge() as i32,
        mm3,
        flow.scaled_width(),
        flow.scaled_spacing(),
        flow.scaled_elephant_foot_spacing()
    )
}

fn result_fields(r: Result<Flow, slicer::flow::FlowError>) -> String {
    match r {
        Ok(flow) => flow_fields(&flow),
        Err(_) => "ERR".to_string(),
    }
}

/// A config holding the options a CFG line sets.
struct Cfg(Vec<(String, Box<dyn ConfigOption>)>);

impl ConfigOptionResolver for Cfg {
    fn optptr(&self, opt_key: &str) -> Option<&dyn ConfigOption> {
        self.0.iter().find(|(k, _)| k == opt_key).map(|(_, o)| o.as_ref())
    }
}

/// The three results of a CFG line (see gen.cpp): a double's bits, or "E:" and the hex of the
/// exception's message.
fn cfg_case(a: &[&str]) -> String {
    let key = a[0];
    let (has_key, v_key, has_line, v_line, has_nd) = (a[1] == "1", d(a[2]), a[3] == "1", d(a[4]), a[5] == "1");
    let n: usize = a[6].parse().unwrap();
    let nds: Vec<f64> = (0..n).map(|k| d(a[7 + k])).collect();
    let first: u32 = a[7 + n].parse().unwrap();
    let (preset_nd, nd, multi) = (f(a[8 + n]) as f32, f(a[9 + n]) as f32, a[10 + n] == "1");
    // The generator sets line_width, then the key: when the key is line_width its value replaces
    // the first one, so the key goes first here (the lookup takes the first entry of a name).
    let mut options: Vec<(String, Box<dyn ConfigOption>)> = Vec::new();
    if has_key {
        options.push((key.to_string(), Box::new(ConfigOptionFloat { value: v_key })));
    }
    if has_line {
        options.push(("line_width".to_string(), Box::new(ConfigOptionFloat { value: v_line })));
    }
    if has_nd {
        options.push(("nozzle_diameter".to_string(), Box::new(ConfigOptionFloatsNullable::from_values(nds))));
    }
    let cfg = Cfg(options);
    let out = |r: Result<f64, FlowError>| -> String {
        match r {
            Ok(v) => format!("{:016x}", v.to_bits()),
            Err(FlowError::MissingVariable(m)) | Err(FlowError::Runtime(m)) => {
                format!("E:{}", m.bytes().map(|c| format!("{:02x}", c)).collect::<String>())
            }
            Err(e) => format!("unexpected {e}"),
        }
    };
    let ew = if has_key { out(Flow::extrusion_width_by_key(key, &cfg, first)) } else { "-".to_string() };
    let res = out(Flow::resolve_extrusion_width_for_nozzle(key, &cfg, preset_nd, nd, multi));
    let ewn = format!(
        "{:016x}",
        Flow::extrusion_width_for_nozzle(&ConfigOptionFloat { value: v_key }, preset_nd, nd, multi).to_bits()
    );
    format!("{ew} {res} {ewn}")
}

#[test]
fn flow_matches_the_pin() {
    let text = read_vectors();
    let mut counts = std::collections::BTreeMap::<String, (usize, usize)>::new();
    let mut shown = 0;
    for line in text.lines() {
        let (lhs, want) = line.split_once(" -> ").expect("arrow");
        let a: Vec<&str> = lhs.split_whitespace().collect();
        let got = match a[0] {
            "AUTO" => fbits(Flow::auto_extrusion_width(role(a[1]), f(a[2]))),
            "RRS" => match Flow::rounded_rectangle_extrusion_spacing(f(a[1]), f(a[2])) {
                Ok(v) => fbits(v),
                Err(_) => "ERR".to_string(),
            },
            "RRW" => fbits(Flow::rounded_rectangle_extrusion_width_from_spacing(f(a[1]), f(a[2]))),
            "BES" => fbits(Flow::bridge_extrusion_spacing(f(a[1]))),
            "FLOW" => result_fields(Flow::new(f(a[1]), f(a[2]), f(a[3]))),
            "BRIDGE" => flow_fields(&Flow::bridging_flow(f(a[1]), f(a[2]))),
            "WW" => result_fields(Flow::new(f(a[1]), f(a[2]), f(a[3])).and_then(|fl| fl.with_width(f(a[4])))),
            "WH" => result_fields(Flow::new(f(a[1]), f(a[2]), f(a[3])).and_then(|fl| fl.with_height(f(a[4])))),
            "WSP" => {
                let base = if a[1] == "1" {
                    Ok(Flow::bridging_flow(f(a[2]), f(a[4])))
                } else {
                    Flow::new(f(a[2]), f(a[3]), f(a[4]))
                };
                result_fields(base.and_then(|fl| fl.with_spacing(f(a[5]))))
            }
            "WCS" => result_fields(Flow::new(f(a[1]), f(a[2]), f(a[3])).and_then(|fl| fl.with_cross_section(f(a[4])))),
            "WFR" => result_fields(Flow::new(f(a[1]), f(a[2]), f(a[3])).and_then(|fl| fl.with_flow_ratio(d(a[4])))),
            "CFG" => cfg_case(&a[1..]),
            tag => panic!("unknown tag {tag}"),
        };
        let e = counts.entry(a[0].to_string()).or_default();
        e.0 += 1;
        if got != want {
            e.1 += 1;
            if shown < 20 {
                shown += 1;
                eprintln!("{lhs}\n   want {want}\n   got  {got}");
            }
        }
    }
    eprintln!("{counts:?}");
    assert_eq!(counts.len(), 12);
    assert!(counts.values().all(|&(n, _)| n >= 400));
    let bad: usize = counts.values().map(|&(_, b)| b).sum();
    assert_eq!(bad, 0, "{counts:?}");
}
