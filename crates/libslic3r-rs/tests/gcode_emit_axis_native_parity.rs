//! `GCodeFormatter::emit_axis` (GCodeWriter.cpp:1097-1137) against the pin: the text the live
//! writer's `format_gcode_value` (src/gcode/writer.rs) and `g_code_writer::GCodeFormatter::emit_axis`
//! write for a value and a number of digits, and the F word of the live `GCodeWriter::set_speed`
//! (GCodeWriter.cpp:387-399 `emit_f`), which goes through it with SET_SPEED_EMIT_AXIS.
//!
//! `tests/data/gcode_emit_axis/vectors.txt.gz` is the output of `gen.cpp` beside it (`gen 3000 |
//! gzip -9n`), which calls the pin's formatter on values over several magnitudes and both signs,
//! on halves of the last digit and one ulp either side of them, on feedrates (speed * 60) and on
//! fixed cases (zeros, tiny values). Each line: the double's bits in hex, the digits, the text.

use slicer::g_code_writer::GCodeFormatter;
use slicer::gcode::{format_gcode_value, GCodeWriter};
use std::io::Read;

fn read_vectors() -> String {
    let path = format!("{}/tests/data/gcode_emit_axis/vectors.txt.gz", env!("CARGO_MANIFEST_DIR"));
    let file = std::fs::File::open(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
    let mut text = String::new();
    flate2::read::GzDecoder::new(file).read_to_string(&mut text).expect("gzip");
    text
}

#[test]
fn emit_axis_matches_the_pin() {
    let text = read_vectors();
    let (mut cases, mut bad, mut f_cases, mut f_bad) = (0usize, 0usize, 0usize, 0usize);
    let mut fmt_bad = 0usize;
    for line in text.lines() {
        let t: Vec<&str> = line.split(' ').collect();
        assert_eq!(t.len(), 3, "{line}");
        let v = f64::from_bits(u64::from_str_radix(t[0], 16).expect("bits"));
        let digits: usize = t[1].parse().expect("digits");
        let want = t[2];
        cases += 1;
        let got = format_gcode_value(v, digits);
        if got != want {
            bad += 1;
            if bad <= 10 {
                eprintln!("emit_axis({v:e}, {digits}): pin {want} port {got}");
            }
        }
        // g_code_writer.rs: the formatter appends ' ' + axis + digits; string() adds '\n'.
        let mut f = GCodeFormatter::new();
        f.emit_axis(b'F', v, digits);
        let s = f.string();
        if s != format!(" F{want}\n") {
            fmt_bad += 1;
            if fmt_bad <= 10 {
                eprintln!("GCodeFormatter::emit_axis({v:e}, {digits}): pin {want} port {s:?}");
            }
        }
        // GCodeWriter::set_speed writes `G1 F` + emit_axis(F, 3); feedrates are positive.
        if digits == 3 && v > 0.0 && v < 100000.0 && slicer::faithful_gate("SET_SPEED_EMIT_AXIS") {
            f_cases += 1;
            let mut w = GCodeWriter::new();
            w.set_speed(v, "");
            let content = w.gcode().content().to_string();
            let last = content.lines().last().unwrap_or("").to_string();
            if last != format!("G1 F{want}") {
                f_bad += 1;
                if f_bad <= 10 {
                    eprintln!("set_speed({v:e}): pin G1 F{want} port {last}");
                }
            }
        }
    }
    eprintln!(
        "emit_axis: {cases} cases, format_gcode_value {bad} differ, GCodeFormatter {fmt_bad} differ; set_speed: {f_cases} feedrates, {f_bad} differ"
    );
    assert!(cases > 10000);
    assert_eq!(bad, 0);
    assert_eq!(fmt_bad, 0);
    assert_eq!(f_bad, 0);
}
