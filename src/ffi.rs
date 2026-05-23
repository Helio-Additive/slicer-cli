//! Rust ↔ C++ boundary. Job config crosses as one JSON string — C++ parses with nlohmann/json.

#[cxx::bridge]
pub mod ffi {
    extern "Rust" {
        type EventSink;
        fn emit_event(self: &mut EventSink, line: &str);
        fn emit_progress(self: &mut EventSink, percent: u32, message: &str);
    }

    unsafe extern "C++" {
        include!("libslic3r/bambustudio/shim.hpp");

        /// Drives one slice job. Returns 0 on success, non-zero on failure.
        fn slicer_run(job_json: &str, sink: Pin<&mut EventSink>) -> i32;

        /// Returns JSON array of preset names for `kind` ("machine"|"filament"|"process").
        fn slicer_list_presets(kind: &str) -> String;

        /// Returns JSON object for the named preset, or `null` if not found.
        fn slicer_get_preset(kind: &str, name: &str) -> String;
    }
}

/// Rust-side sink the C++ shim calls into for every slicer event.
pub struct EventSink {
    pub events: Vec<String>,
}

impl EventSink {
    pub fn new() -> Self {
        Self { events: Vec::new() }
    }

    fn emit_event(&mut self, line: &str) {
        self.events.push(line.to_string());
    }

    fn emit_progress(&mut self, percent: u32, message: &str) {
        self.events
            .push(format!(r#"{{"kind":"progress","percent":{percent},"message":"{message}"}}"#));
    }
}
