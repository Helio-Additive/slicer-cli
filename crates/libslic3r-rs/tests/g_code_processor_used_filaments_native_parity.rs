//! Native parity of `GCodeProcessor::UsedFilaments` (GCodeProcessor.cpp:1380-1534): `reset`,
//! `increase_support_caches`, `increase_model_caches`, `increase_wipe_tower_caches`,
//! `process_color_change_cache`, `process_model_cache`, `process_wipe_tower_cache`,
//! `process_support_cache`, `process_total_volume_cache`, `update_flush_per_filament`,
//! `process_role_cache` and `process_caches`.
//!
//! `tests/data/g_code_processor_used_filaments/g_code_processor_used_filaments.txt.gz` is the
//! output of `gen.cpp` beside it (gzip -9n): LCG sequences of these operations run on the pin's
//! own `GCodeProcessor::m_used_filaments`, the whole state printed after every operation (doubles
//! as 64-bit patterns). What the pin's methods read from the processor (`get_filament_id()`,
//! `get_filament_id(false)`, the filament's diameter and density, `m_extrusion_role`) is printed
//! with each operation and handed to the port's methods, which take those values as arguments.

use slicer::extrusion_entity::ExtrusionRole;
use slicer::gcode::g_code_processor::UsedFilaments;
use std::collections::BTreeMap;
use std::io::Read;

fn read_vectors() -> String {
    let path = format!(
        "{}/tests/data/g_code_processor_used_filaments/g_code_processor_used_filaments.txt.gz",
        env!("CARGO_MANIFEST_DIR")
    );
    let file = std::fs::File::open(&path).expect("vectors");
    let mut text = String::new();
    flate2::read::GzDecoder::new(file).read_to_string(&mut text).expect("gzip");
    text
}

const ROLES: [ExtrusionRole; 22] = [
    ExtrusionRole::None,
    ExtrusionRole::Perimeter,
    ExtrusionRole::ExternalPerimeter,
    ExtrusionRole::OverhangPerimeter,
    ExtrusionRole::InternalInfill,
    ExtrusionRole::SolidInfill,
    ExtrusionRole::FloatingVerticalShell,
    ExtrusionRole::TopSolidInfill,
    ExtrusionRole::BottomSurface,
    ExtrusionRole::Ironing,
    ExtrusionRole::BridgeInfill,
    ExtrusionRole::GapFill,
    ExtrusionRole::Skirt,
    ExtrusionRole::Brim,
    ExtrusionRole::SupportMaterial,
    ExtrusionRole::SupportMaterialInterface,
    ExtrusionRole::SupportTransition,
    ExtrusionRole::SupportIroning,
    ExtrusionRole::WipeTower,
    ExtrusionRole::Custom,
    ExtrusionRole::Flush,
    ExtrusionRole::Mixed,
];

fn d64(v: f64) -> String {
    format!("{:016x}", v.to_bits())
}

fn pd64(t: &str) -> f64 {
    f64::from_bits(u64::from_str_radix(t, 16).unwrap())
}

fn pf32(t: &str) -> f32 {
    f32::from_bits(u32::from_str_radix(t, 16).unwrap())
}

fn dump_map(o: &mut String, name: &str, m: &BTreeMap<usize, f64>) {
    o.push(' ');
    o.push_str(name);
    o.push('=');
    if m.is_empty() {
        o.push('-');
    }
    let parts: Vec<String> = m.iter().map(|(k, v)| format!("{}:{}", k, d64(*v))).collect();
    o.push_str(&parts.join(","));
}

fn dump(u: &UsedFilaments) -> String {
    let mut o = String::from("S");
    o.push_str(&format!(
        " cc={} mc={} wc={} sc={} tc={} rc={}",
        d64(u.color_change_cache),
        d64(u.model_extrude_cache),
        d64(u.wipe_tower_cache),
        d64(u.support_volume_cache),
        d64(u.total_volume_cache),
        d64(u.role_cache)
    ));
    o.push_str(" vpcc=");
    if u.volumes_per_color_change.is_empty() {
        o.push('-');
    }
    let v: Vec<String> = u.volumes_per_color_change.iter().map(|x| d64(*x)).collect();
    o.push_str(&v.join(","));
    dump_map(&mut o, "mv", &u.model_volumes_per_filament);
    dump_map(&mut o, "wv", &u.wipe_tower_volumes_per_filament);
    dump_map(&mut o, "sv", &u.support_volumes_per_filament);
    dump_map(&mut o, "fl", &u.flush_per_filament);
    dump_map(&mut o, "tv", &u.total_volumes_per_filament);
    o.push_str(" fr=");
    if u.filaments_per_role.is_empty() {
        o.push('-');
    }
    let r: Vec<String> =
        u.filaments_per_role.iter().map(|(k, (a, b))| format!("{}:{}:{}", k, d64(*a), d64(*b))).collect();
    o.push_str(&r.join(","));
    o
}

#[test]
fn g_code_processor_used_filaments_native_parity() {
    let text = read_vectors();
    let mut u = UsedFilaments::default();
    let mut diam: Vec<f32> = Vec::new();
    let mut dens: Vec<f32> = Vec::new();
    let (mut n_cases, mut n_ops) = (0usize, 0usize);
    let mut per_op = [0usize; 14];
    let mut bad: Vec<String> = Vec::new();
    let mut last_op = String::new();
    for (ln, line) in text.lines().enumerate() {
        let t: Vec<&str> = line.split(' ').collect();
        match t[0] {
            "C" => {
                n_cases += 1;
                u = UsedFilaments::default();
                u.reset();
            }
            "F" => {
                diam.clear();
                dens.clear();
                for p in &t[1..] {
                    let (a, b) = p.split_once(':').unwrap();
                    diam.push(pf32(a));
                    dens.push(pf32(b));
                }
            }
            "O" => {
                n_ops += 1;
                let op: usize = t[1].parse().unwrap();
                per_op[op] += 1;
                let i = |k: usize| -> i32 { t[k].parse().unwrap() };
                match op {
                    0 => u.increase_model_caches(pd64(t[2])),
                    1 => u.increase_support_caches(pd64(t[2])),
                    2 => u.increase_wipe_tower_caches(pd64(t[2])),
                    3 => u.process_color_change_cache(),
                    4 => u.process_model_cache(i(2)),
                    5 => u.process_wipe_tower_cache(i(2)),
                    6 => u.process_support_cache(i(2)),
                    7 => u.process_total_volume_cache(i(2)),
                    8 => u.update_flush_per_filament(t[2].parse().unwrap(), pf32(t[3])),
                    9 => {
                        let fid = i(2) as usize;
                        u.process_role_cache(diam[fid], dens[fid], ROLES[i(3) as usize]);
                    }
                    10 => {
                        let fid = i(2) as usize;
                        u.process_caches(i(2), i(3), diam[fid], dens[fid], ROLES[i(4) as usize]);
                    }
                    11 | 12 => {}
                    13 => u.reset(),
                    other => panic!("unknown op {other} on line {}", ln + 1),
                }
                last_op = line.to_string();
            }
            "S" => {
                let got = dump(&u);
                if got != line {
                    bad.push(format!("line {} after '{}':\n  port {}\n  pin  {}", ln + 1, last_op, got, line));
                    // continue from the pin's state is not possible; restart the comparison per case
                }
            }
            other => panic!("unknown tag {other} on line {}", ln + 1),
        }
    }
    eprintln!(
        "g_code_processor_used_filaments: {n_cases} cases, {n_ops} operations (per op {:?}), mismatching states {}",
        per_op,
        bad.len()
    );
    for b in bad.iter().take(10) {
        eprintln!("{b}");
    }
    assert!(n_cases > 0 && per_op.iter().all(|&c| c > 0));
    assert!(bad.is_empty(), "{} mismatching states", bad.len());
}
