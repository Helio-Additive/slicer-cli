use serde_json::Value;
use std::process::Command;

fn slicer_cli() -> Command {
    Command::new(env!("CARGO_BIN_EXE_slicer-cli"))
}

// ── list ─────────────────────────────────────────────────────────────────────

#[test]
fn list_machine_returns_json_array() {
    let out = slicer_cli().args(["presets", "list", "machine"]).output().unwrap();
    assert!(out.status.success(), "stderr: {}", String::from_utf8_lossy(&out.stderr));
    let v: Value = serde_json::from_slice(&out.stdout).expect("valid JSON");
    assert!(v.is_array());
}

#[test]
fn list_filament_returns_json_array() {
    let out = slicer_cli().args(["presets", "list", "filament"]).output().unwrap();
    assert!(out.status.success());
    let v: Value = serde_json::from_slice(&out.stdout).expect("valid JSON");
    assert!(v.is_array());
}

#[test]
fn list_process_returns_json_array() {
    let out = slicer_cli().args(["presets", "list", "process"]).output().unwrap();
    assert!(out.status.success());
    let v: Value = serde_json::from_slice(&out.stdout).expect("valid JSON");
    assert!(v.is_array());
}

#[test]
fn list_invalid_kind_fails() {
    let out = slicer_cli().args(["presets", "list", "banana"]).output().unwrap();
    assert!(!out.status.success());
}

// ── get ──────────────────────────────────────────────────────────────────────

#[test]
fn get_unknown_preset_fails_with_nonzero_exit() {
    let out = slicer_cli()
        .args(["presets", "get", "machine", "nonexistent preset xyz"])
        .output()
        .unwrap();
    assert!(!out.status.success());
}

// ── content assertions ────────────────────────────────────────────────────────

mod ffi_tests {
    use serde_json::Value;
    use slicer_cli::ffi::ffi;

    #[test]
    fn list_machine_nonempty() {
        let json = ffi::slicer_list_presets("machine");
        let v: Value = serde_json::from_str(&json).expect("valid JSON");
        assert!(v.as_array().map(|a| !a.is_empty()).unwrap_or(false));
    }

    #[test]
    fn list_filament_nonempty() {
        let json = ffi::slicer_list_presets("filament");
        let v: Value = serde_json::from_str(&json).expect("valid JSON");
        assert!(v.as_array().map(|a| !a.is_empty()).unwrap_or(false));
    }

    #[test]
    fn list_process_nonempty() {
        let json = ffi::slicer_list_presets("process");
        let v: Value = serde_json::from_str(&json).expect("valid JSON");
        assert!(v.as_array().map(|a| !a.is_empty()).unwrap_or(false));
    }

    #[test]
    fn get_machine_preset_is_object() {
        let names: Value =
            serde_json::from_str(&ffi::slicer_list_presets("machine")).unwrap();
        let first = names[0].as_str().expect("first name is a string");
        let json = ffi::slicer_get_preset("machine", first);
        let v: Value = serde_json::from_str(&json).expect("valid JSON");
        assert!(v.is_object(), "preset should be a JSON object");
    }

    #[test]
    fn get_filament_preset_is_object() {
        let names: Value =
            serde_json::from_str(&ffi::slicer_list_presets("filament")).unwrap();
        let first = names[0].as_str().expect("first name is a string");
        let json = ffi::slicer_get_preset("filament", first);
        let v: Value = serde_json::from_str(&json).expect("valid JSON");
        assert!(v.is_object());
    }

    #[test]
    fn get_missing_preset_returns_null() {
        let json = ffi::slicer_get_preset("machine", "nonexistent preset xyz");
        assert_eq!(json, "null");
    }
}
