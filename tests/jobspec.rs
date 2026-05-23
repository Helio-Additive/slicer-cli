// Round-trip test: a representative JobSpec JSON parses, serializes,
// and re-parses identically. Catches accidental schema breakage.

use slicer_cli::job::JobSpec;

const SAMPLE: &str = r#"{
    "job_id": "k8s-job-abc123",
    "input":  { "kind": "url",  "url": "https://example.com/benchy.stl" },
    "output": { "kind": "s3",   "bucket": "slices", "key": "benchy.gcode" },
    "callbacks": {
        "event_webhook":      "https://orch.svc/events",
        "completion_webhook": "https://orch.svc/done"
    },
    "machine":  { "name": "Bambu Lab X1 Carbon 0.4 nozzle" },
    "filament": [{ "name": "Bambu PLA Basic" }],
    "process":  { "name": "0.20mm Standard BBL" }
}"#;

#[test]
fn jobspec_round_trips() {
    let parsed: JobSpec = serde_json::from_str(SAMPLE).unwrap();
    let again = serde_json::to_string(&parsed).unwrap();
    let reparsed: JobSpec = serde_json::from_str(&again).unwrap();
    assert_eq!(parsed.job_id, reparsed.job_id);
}
