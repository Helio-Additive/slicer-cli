//! Bambu's read-only timing command, cli_gcode_timing.cpp 188–246, 276–338.
use serde_json::{json, Value};
use slicer::gcode::g_code_processor::{ETimeMode, GCodeProcessor};
use slicer::multi_nozzle_utils::LayeredNozzleGroupResult;
use std::{fs::File, io::Read, path::Path, sync::Arc};

fn readable_input(path: &str) -> bool {
    if !Path::new(path).is_file() {
        return false;
    }
    let mut first = [0];
    File::open(path)
        .and_then(|mut f| f.read_exact(&mut first))
        .is_ok()
}

fn time_file(
    path: &str,
    seed: Option<Arc<LayeredNozzleGroupResult>>,
) -> Option<(f64, Option<Arc<LayeredNozzleGroupResult>>)> {
    let mut processor = GCodeProcessor::new();
    processor.set_quiet(true);
    if let Some(group) = seed {
        processor.initialize_from_shared_context(group);
    } else {
        processor.init_filament_maps_when_import_only_gcode();
    }
    processor.process_file(path, None).ok()?;
    let result = processor.result();
    let seconds = result.print_statistics.modes[ETimeMode::Normal as usize].time;
    if seconds.is_finite() && seconds > 0.0 {
        Some((f64::from(seconds), result.nozzle_group_result.clone()))
    } else {
        None
    }
}

pub(crate) fn run(args: &[String]) -> u8 {
    let mut document =
        json!({"schema_version": 1, "engine": "bambustudio", "engine_revision": "56a57d0"});
    let error = match args {
        [flag, original, optimized] if flag == "--gcode-timing" => {
            if !readable_input(original) || !readable_input(optimized) {
                (1, "INPUT_UNREADABLE")
            } else if let Some((first, second)) =
                time_file(original, None).and_then(|(first, group)| {
                    time_file(optimized, group).map(|(second, _)| (first, second))
                })
            {
                document["original_seconds"] = Value::from(first);
                document["optimized_seconds"] = Value::from(second);
                println!("{document}");
                return 0;
            } else {
                (1, "TIMING_UNAVAILABLE")
            }
        }
        _ => (64, "CONFLICTING_ARGUMENTS"),
    };
    document["error_code"] = Value::from(error.1);
    println!("{document}");
    error.0
}
