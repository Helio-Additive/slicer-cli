use std::{fs, path::Path};

/// main.cpp 11500–11508 selects the vendor from the machine model on each slice.
/// GCode.cpp 2004, 3363 and GCodeProcessor.cpp 701–720 consume that choice.
#[test]
fn project_exports_select_the_printer_for_each_job() {
    let model = Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/calib_base.3mf");
    let dir = tempfile::tempdir().unwrap();
    let config = dir.path().join("settings.json");
    let output = dir.path().join("part.gcode");
    for (printer, is_bbl) in [
        ("Generic CoreXY", false),
        ("Bambu Lab X1 Carbon", true),
        ("Generic CoreXY", false),
    ] {
        fs::write(
            &config,
            serde_json::json!({"printer_model": printer}).to_string(),
        )
        .unwrap();
        let args: Vec<String> = [
            "--config",
            config.to_str().unwrap(),
            "--output",
            output.to_str().unwrap(),
            model.to_str().unwrap(),
        ]
        .into_iter()
        .map(str::to_owned)
        .collect();
        assert_eq!(slicer_cli_rust::engine_cli::run(&args), 0);
        let gcode = fs::read_to_string(&output).unwrap();
        let header = gcode.lines().nth(2).unwrap();
        assert_eq!(header.starts_with("; model printing time: "), is_bbl);
        assert_eq!(
            header.starts_with("; estimated printing time (normal mode) = "),
            !is_bbl
        );
        assert_eq!(
            gcode
                .lines()
                .any(|line| line == "M981 S1 P20000 ;open spaghetti detector"),
            is_bbl
        );
    }
}
