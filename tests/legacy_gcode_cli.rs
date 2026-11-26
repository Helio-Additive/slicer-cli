//! main.cpp 521–610, 11558–11559. Inspect the real root CLI's native event and templates.
use serde_json::Value;
use std::fs;
use std::path::Path;
use std::process::{Command, Output};

fn run(model: &Path, output: &Path, opts: &[&str]) -> Output {
    Command::new(env!("CARGO_BIN_EXE_slicer_cli-rust"))
        .args(opts)
        .arg(model)
        .arg("-o")
        .arg(output)
        .output()
        .unwrap()
}

fn events(out: &Output) -> Vec<Value> {
    String::from_utf8_lossy(&out.stdout)
        .lines()
        .filter_map(|line| line.strip_prefix("[[SLICER_EVENT]] "))
        .map(|payload| serde_json::from_str(payload).unwrap())
        .collect()
}

#[test]
fn legacy_template_alias_is_a_per_slice_cli_choice() {
    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    let project = root.join("tests/fixtures/legacy_token.3mf");
    let dir = tempfile::tempdir().unwrap();
    let gcode = dir.path().join("out.gcode");
    let original = "initial_no_support_filament_id";
    for normalize in [true, false, true] {
        let options: &[&str] = if normalize {
            &[]
        } else {
            &["--no-normalize-legacy-gcode"]
        };
        let out = run(&project, &gcode, options);
        assert!(
            out.status.success(),
            "{}",
            String::from_utf8_lossy(&out.stderr)
        );
        let aliases: Vec<_> = events(&out)
            .into_iter()
            .filter(|e| e["event"] == "config_normalized")
            .collect();
        assert_eq!(aliases.len(), usize::from(normalize));
        if normalize {
            assert_eq!(aliases[0]["count"], 2);
            assert_eq!(
                aliases[0]["keys"],
                serde_json::json!(["machine_start_gcode", "filament_start_gcode"])
            );
        }
        let body = fs::read_to_string(&gcode).unwrap();
        assert_eq!(body.contains(original), !normalize);
        assert!(body
            .lines()
            .filter(|s| !s.starts_with(';'))
            .all(|s| !s.contains(original)));
    }
}

#[test]
fn repeated_driver_calls_keep_their_own_template_policy() {
    let project = Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/legacy_token.3mf");
    let dir = tempfile::tempdir().unwrap();
    let gcode = dir.path().join("job.gcode");
    for preserve in [false, true, false] {
        let mut words = vec![
            project.to_str().unwrap().to_owned(),
            "-o".into(),
            gcode.to_str().unwrap().to_owned(),
        ];
        if preserve {
            words.push("--no-normalize-legacy-gcode".into());
        }
        assert_eq!(slicer_cli_rust::engine_cli::run(&words), 0);
        let body = fs::read_to_string(&gcode).unwrap();
        assert_eq!(body.contains("initial_no_support_filament_id"), preserve);
    }
}
