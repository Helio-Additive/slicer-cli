// main.cpp 13486-13498 and cli_gcode_timing.cpp 188-246, 276-338.
use std::process::Command;

#[test]
fn times_both_inputs_with_the_original_nozzle_group() {
    let dir = tempfile::tempdir().unwrap();
    let single =
        include_str!("../crates/libslic3r-rs/tests/data/g_code_processor/import_profile.gcode");
    let dual = single.replace("; filament_map = 1,1,1,1", "; filament_map = 1,2,2,1");
    let a = dir.path().join("original.gcode");
    let b = dir.path().join("优化.gcode");
    std::fs::write(&a, single).unwrap();
    std::fs::write(&b, &dual).unwrap();
    for (first, second, seconds) in [
        (&a, &b, "3.0149996280670166"),
        (&b, &a, "18.014999389648438"),
        (&b, &b, "18.014999389648438"),
    ] {
        let out = Command::new(env!("CARGO_BIN_EXE_slicer_cli-rust"))
            .env("CLI_GCODE_TIMING", "1")
            .arg("--gcode-timing")
            .arg(first)
            .arg(second)
            .output()
            .unwrap();
        assert_eq!(
            out.status.code(),
            Some(0),
            "{}",
            String::from_utf8_lossy(&out.stderr)
        );
        assert!(out.stderr.is_empty());
        assert_eq!(String::from_utf8(out.stdout).unwrap(), format!(
            "{{\"engine\":\"bambustudio\",\"engine_revision\":\"56a57d0\",\"optimized_seconds\":{seconds},\"original_seconds\":{seconds},\"schema_version\":1}}\n"
        ));
    }
    assert_eq!(std::fs::read(&a).unwrap(), single.as_bytes());
    assert_eq!(std::fs::read(&b).unwrap(), dual.as_bytes());
}

#[test]
fn timing_conflicts_unreadable_files_and_unavailable_results() {
    let dir = tempfile::tempdir().unwrap();
    std::fs::write(dir.path().join("empty"), "").unwrap();
    std::fs::write(dir.path().join("zero"), "; no movement\n").unwrap();
    std::fs::write(dir.path().join("broken"), "; BambuStudio\nG1 X20 F600\n").unwrap();
    std::fs::write(
        dir.path().join("motion"),
        "G90\nG1 X0 Y0 Z0.2 F1800\nG1 X30 Y0 F600\n",
    )
    .unwrap();
    for (args, code, reason) in [
        (&["--gcode-timing"][..], 64, "CONFLICTING_ARGUMENTS"),
        (
            &["-gcode-timing", "motion", "motion"],
            64,
            "CONFLICTING_ARGUMENTS",
        ),
        (
            &["--gcode-timing=1", "motion", "motion"],
            64,
            "CONFLICTING_ARGUMENTS",
        ),
        (
            &["--config=a", "--gcode-timing", "motion", "motion"],
            64,
            "CONFLICTING_ARGUMENTS",
        ),
        (&["--help", "--gcode-timing"], 64, "CONFLICTING_ARGUMENTS"),
        (&["--gcode-timing", "zero", "."], 1, "INPUT_UNREADABLE"),
        (
            &["--gcode-timing", "motion", "empty"],
            1,
            "INPUT_UNREADABLE",
        ),
        (
            &["--gcode-timing", "absent", "motion"],
            1,
            "INPUT_UNREADABLE",
        ),
        (
            &["--gcode-timing", "zero", "motion"],
            1,
            "TIMING_UNAVAILABLE",
        ),
        (
            &["--gcode-timing", "motion", "broken"],
            1,
            "TIMING_UNAVAILABLE",
        ),
    ] {
        let out = Command::new(env!("CARGO_BIN_EXE_slicer_cli-rust"))
            .args(args)
            .env("CLI_GCODE_TIMING", "1")
            .env("TIMEDBG", "1")
            .current_dir(dir.path())
            .output()
            .unwrap();
        assert_eq!(out.status.code(), Some(code), "{args:?}");
        assert!(
            out.stderr.is_empty(),
            "{args:?}: {}",
            String::from_utf8_lossy(&out.stderr)
        );
        assert_eq!(String::from_utf8(out.stdout).unwrap(), format!(
            "{{\"engine\":\"bambustudio\",\"engine_revision\":\"56a57d0\",\"error_code\":\"{reason}\",\"schema_version\":1}}\n"
        ));
    }
    let off = Command::new(env!("CARGO_BIN_EXE_slicer_cli-rust"))
        .args(["--gcode-timing", "motion", "motion"])
        .env("CLI_GCODE_TIMING", "0")
        .current_dir(dir.path())
        .output()
        .unwrap();
    assert_eq!(off.status.code(), Some(254));
    assert!(String::from_utf8(off.stderr)
        .unwrap()
        .starts_with("--gcode-timing is not a flag"));
}
