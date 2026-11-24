//! Native parity of the `determine_bridging_angle` lambda of `PrintObject::bridge_over_infill`
//! (PrintObject.cpp:3081-3160), ported as `slicer::print_object::determine_bridging_angle_pin`.
//! data/bridging_angle/gen.cpp copies the lambda verbatim and runs it on deterministic bridged areas and
//! anchors over six infill patterns.
mod support_shared;

use slicer::geometry::Line;
use slicer::print_config::InfillPattern;
use slicer::print_object::determine_bridging_angle_pin;
use support_shared::*;

/// gen.cpp `pattern_of`
fn pattern_of(k: i64) -> InfillPattern {
    match k {
        0 => InfillPattern::Rectilinear,
        1 => InfillPattern::Line,
        2 => InfillPattern::HilbertCurve,
        3 => InfillPattern::OctagramSpiral,
        4 => InfillPattern::CrossHatch,
        _ => InfillPattern::Honeycomb3D,
    }
}

#[test]
fn bridging_angle_native_parity() {
    let mut r = Run::load("bridging_angle");
    let cases = run_cases(&mut r);
    let (n, ok) = r.tally("ba ");
    eprintln!("bridging_angle: {cases} cases, {ok} of {n} equal");
    assert_eq!(r.finish(), 0);
    assert!(cases >= 400);
}

/// The two calls of the lambda on Majora's layer 187 (index from 0) as the port makes them (probe
/// BA_DUMP=187 of print_object.rs), run through the pin's lambda by `gen <file>`
/// (data/bridging_angle/vectors_majora.txt.gz). The pin's lambda gives the port's angle on the port's
/// inputs, so the bridge that prints at another angle there has other inputs (area or anchors).
#[test]
fn bridging_angle_majora_l187_native_parity() {
    use std::io::Read;
    let path = format!("{}/tests/data/bridging_angle/vectors_majora.txt.gz", env!("CARGO_MANIFEST_DIR"));
    let mut text = String::new();
    flate2::read::GzDecoder::new(std::fs::File::open(&path).expect("vectors_majora.txt.gz")).read_to_string(&mut text).expect("gz");
    let mut r = Run { expected: text.lines().map(|l| l.to_string()).collect(), pos: 0, lines: Vec::new(), surplus: 0 };
    let cases = run_cases(&mut r);
    assert_eq!(r.finish(), 0);
    assert_eq!(cases, 2);
}

fn run_cases(r: &mut Run) -> usize {
    let mut cases = 0;
    while let Some(line) = r.input("in ba") {
        let mut t = Tok::new(&line);
        let k = t.i();
        let dir = t.d();
        let area = t.polys();
        let nl = t.i();
        let mut anchors = Vec::new();
        for _ in 0..nl {
            let a = t.pt();
            let b = t.pt();
            anchors.push(Line::new(a, b));
        }
        let angle = determine_bridging_angle_pin(&area, &anchors, pattern_of(k), dir);
        r.out(format!("ba {}", hd(angle)));
        cases += 1;
    }
    cases
}
