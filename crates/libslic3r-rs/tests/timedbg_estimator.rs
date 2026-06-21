use slicer::gcode::g_code_processor::GCodeProcessor;
use slicer::print_config::PrintConfig;

#[test]
#[ignore]
fn timedbg_native_vs_rust() {
    for (name, path) in [
        ("NATIVE", "/tmp/cmp/native.gcode"),
        ("RUST", "/tmp/cmp/rust_fix.gcode"),
        ("RUST_CAP200", "/tmp/cmp/rust_cap200.gcode"),
        ("RUST_ACCEL8000", "/tmp/cmp/rust_accel8000.gcode"),
        ("RUST_BOTH", "/tmp/cmp/rust_both.gcode"),
    ] {
        let gcode = match std::fs::read_to_string(path) {
            Ok(g) => g,
            Err(_) => continue,
        };
        let mut p = GCodeProcessor::new();
        let cfg = PrintConfig::default();
        p.apply_config(&cfg);
        std::env::set_var("ESTNAME", name);
        p.process_gcode(&gcode);
        let r = p.result();
        eprintln!(
            "ESTRESULT {} print_time={:.0}s ({:.1}min)",
            name,
            r.print_time,
            r.print_time / 60.0
        );
    }
}
