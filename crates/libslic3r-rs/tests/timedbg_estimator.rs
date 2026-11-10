use slicer::gcode::g_code_processor::GCodeProcessor;
use slicer::print_config::PrintConfig;
#[test]
#[ignore]
fn timedbg_native_vs_rust() {
    for (name, path) in [("NATIVE","/tmp/cmp/native.gcode"),("RUST","/tmp/cmp/rust_fix.gcode")] {
        let g = match std::fs::read_to_string(path){Ok(g)=>g,Err(_)=>continue};
        let mut p = GCodeProcessor::new();
        p.apply_config(&PrintConfig::default());
        std::env::set_var("ESTNAME", name);
        p.process_gcode(&g);
        eprintln!("ESTRESULT {} {:.0}s ({:.1}min)", name, p.result().print_time, p.result().print_time/60.0);
    }
}
