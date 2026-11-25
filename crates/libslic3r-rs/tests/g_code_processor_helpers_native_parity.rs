//! Native parity of the file-static comment helpers of GCodeProcessor.cpp: `get_object_label_id`
//! (152-165), `get_z_height` (167-177), `skip_whitespaces` (2649-2652) and `remove_eols` (2654-2657).
//!
//! `tests/data/g_code_processor_helpers/g_code_processor_helpers.txt.gz` is the output of `gen.cpp`
//! beside it (gzip -9n), which includes the pin's GCodeProcessor.cpp.

use slicer::gcode::g_code_processor::{get_object_label_id, get_z_height, remove_eols, skip_whitespaces};
use std::io::Read;

fn read_vectors() -> String {
    let path = format!(
        "{}/tests/data/g_code_processor_helpers/g_code_processor_helpers.txt.gz",
        env!("CARGO_MANIFEST_DIR")
    );
    let file = std::fs::File::open(&path).expect("vectors");
    let mut text = String::new();
    flate2::read::GzDecoder::new(file).read_to_string(&mut text).expect("gzip");
    text
}

fn unhex(t: &str) -> Vec<u8> {
    if t == "-" {
        return Vec::new();
    }
    (0..t.len() / 2).map(|i| u8::from_str_radix(&t[2 * i..2 * i + 2], 16).unwrap()).collect()
}

#[test]
fn g_code_processor_helpers_native_parity() {
    let text = read_vectors();
    let (mut n_lbl, mut n_zh, mut n_sw) = (0usize, 0usize, 0usize);
    let mut bad: Vec<String> = Vec::new();
    for (ln, line) in text.lines().enumerate() {
        let t: Vec<&str> = line.split(' ').collect();
        let bytes = unhex(t[1]);
        let s = String::from_utf8(bytes.clone()).expect("ascii input");
        match t[0] {
            "lbl" => {
                n_lbl += 1;
                let want: i32 = t[2].parse().unwrap();
                let got = get_object_label_id(&s);
                if got != want {
                    bad.push(format!("line {} lbl {:?}: port {} pin {}", ln + 1, s, got, want));
                }
            }
            "zh" => {
                n_zh += 1;
                let want = u32::from_str_radix(t[2], 16).unwrap();
                let got = get_z_height(&s);
                if got.to_bits() != want {
                    bad.push(format!("line {} zh {:?}: port {} pin {}", ln + 1, s, got, f32::from_bits(want)));
                }
            }
            "sw" => {
                n_sw += 1;
                let want_b: usize = t[2].parse().unwrap();
                let want_e: usize = t[3].parse().unwrap();
                let got_b = skip_whitespaces(&bytes, 0, bytes.len());
                let got_e = remove_eols(&bytes, 0, bytes.len());
                if got_b != want_b || got_e != want_e {
                    bad.push(format!("line {} sw {:?}: port {} {} pin {} {}", ln + 1, s, got_b, got_e, want_b, want_e));
                }
            }
            other => panic!("unknown tag {other} on line {}", ln + 1),
        }
    }
    eprintln!(
        "g_code_processor_helpers: get_object_label_id {n_lbl}, get_z_height {n_zh}, skip_whitespaces/remove_eols {n_sw}, mismatches {}",
        bad.len()
    );
    for b in bad.iter().take(20) {
        eprintln!("{b}");
    }
    assert!(n_lbl > 0 && n_zh > 0 && n_sw > 0);
    assert!(bad.is_empty(), "{} mismatches", bad.len());
}
