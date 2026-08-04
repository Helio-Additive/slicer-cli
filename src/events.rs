//! Structured slicer events on stdout — port of `main.cpp`'s event protocol.
//!
//! C++: `libslic3r/bambustudio/main.cpp:55-137` (the anonymous namespace) plus
//! its emission sites at :1476, :1488, :1492 and :1519-1551.
//!
//! One JSON object per line behind a fixed prefix, flushed immediately, so a
//! streaming line-reader on the host can split events without buffering and
//! sees them as the slice progresses.
//!
//! R528: the wire format here was captured from a real `--engine bambu` run,
//! not inferred. See `docs/main-cpp-correspondence.md` for what is and is not
//! wired up yet.

use serde_json::{json, Value};

/// C++: `main.cpp:57` — `constexpr const char* SLICER_EVENT_PREFIX`.
pub const SLICER_EVENT_PREFIX: &str = "[[SLICER_EVENT]] ";

/// C++: `main.cpp:59-75` — `slicing_notification_tag(int)`.
///
/// Mirrors `PrintStateBase::SlicingNotificationType`; the Rust enum lives at
/// `libslic3r_rs::print_base::SlicingNotificationType` with identical
/// discriminants.
pub fn slicing_notification_tag(t: i32) -> &'static str {
    match t {
        0 => "SlicingDefaultNotification",
        1 => "SlicingReplaceInitEmptyLayers",
        2 => "SlicingNeedSupportOn",
        3 => "SlicingEmptyGcodeLayers",
        4 => "SlicingGcodeOverlap",
        _ => "SlicingUnknown",
    }
}

/// C++: `main.cpp:76-78` — `warning_level_tag(WarningLevel)`.
pub fn warning_level_tag(critical: bool) -> &'static str {
    if critical {
        "critical"
    } else {
        "non_critical"
    }
}

/// C++: `main.cpp:80-98` — `string_exception_tag(StringExceptionType)`.
///
/// Discriminants match `libslic3r_rs::print_base::StringExceptionType`.
pub fn string_exception_tag(t: i32) -> &'static str {
    match t {
        0 => "STRING_EXCEPT_NOT_DEFINED",
        1 => "STRING_EXCEPT_FILAMENT_NOT_MATCH_BED_TYPE",
        2 => "STRING_EXCEPT_FILAMENTS_DIFFERENT_TEMP",
        3 => "STRING_EXCEPT_OBJECT_COLLISION_IN_SEQ_PRINT",
        4 => "STRING_EXCEPT_OBJECT_COLLISION_IN_LAYER_PRINT",
        5 => "STRING_EXCEPT_LAYER_HEIGHT_EXCEEDS_LIMIT",
        6 => "STRING_EXCEPT_COUNT",
        _ => "STRING_EXCEPT_UNKNOWN",
    }
}

/// C++: `main.cpp:100-106` — `emit_event(const json&)`.
///
/// One JSON object per line, flushed, so warnings that fire mid-pipeline reach
/// the host immediately.
pub fn emit_event(payload: &Value) {
    use std::io::Write;
    let stdout = std::io::stdout();
    let mut out = stdout.lock();
    let _ = writeln!(out, "{}{}", SLICER_EVENT_PREFIX, payload);
    let _ = out.flush();
}

/// C++: `main.cpp:108-121` — `emit_status_warning(const SlicingStatus&)`.
///
/// Only fires for the two warning flag bits; a plain progress update emits
/// nothing.
///
/// NOT YET WIRED (R528): our `Print::set_status_callback` carries
/// `(percent, message)` rather than C++'s full `SlicingStatus`, so `flags`,
/// `message_type`, `warning_level` and `warning_step` are not available at the
/// callback. Wiring this needs the library-side callback signature widened to
/// the full struct. The only warning our fixtures actually produce
/// (`SlicingNeedSupportOn`, on Benchy) additionally needs the unported
/// sharp-tail / cantilever detection — see the correspondence doc.
#[allow(dead_code)]
pub fn emit_status_warning(
    is_object_scope: bool,
    message_type: i32,
    critical: bool,
    text: &str,
    warning_step: i32,
) {
    let e = json!({
        "event": "warning",
        "tag": slicing_notification_tag(message_type),
        "level": warning_level_tag(critical),
        "message": text,
        "step": warning_step,
        "scope": if is_object_scope { "object" } else { "print" },
    });
    emit_event(&e);
}

/// C++: `main.cpp:123-135` — `emit_validation_event(const StringObjectException&)`.
///
/// `opt_key` / `params` / `hypertext` are omitted when empty, exactly as the
/// C++ emitter does (note C++ spells the struct field `hypetext` but the JSON
/// key `hypertext`).
pub fn emit_validation_event(
    is_warning: bool,
    exception_type: i32,
    message: &str,
    opt_key: &str,
    params: &[String],
    hypertext: &str,
) {
    let mut e = json!({
        "event": if is_warning { "validation_warning" } else { "validation_error" },
        "tag": string_exception_tag(exception_type),
        "message": message,
    });
    let obj = e.as_object_mut().expect("json object");
    if !opt_key.is_empty() {
        obj.insert("opt_key".into(), json!(opt_key));
    }
    if !params.is_empty() {
        obj.insert("params".into(), json!(params));
    }
    if !hypertext.is_empty() {
        obj.insert("hypertext".into(), json!(hypertext));
    }
    emit_event(&e);
}

/// C++: `main.cpp:1519-1551` — the `slicing_error` events raised from the
/// `process` and `export_gcode` catch blocks.
pub fn emit_slicing_error(phase: &str, kind: &str, message: &str) {
    emit_event(&json!({
        "event": "slicing_error",
        "phase": phase,
        "kind": kind,
        "message": message,
    }));
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tags_match_cpp() {
        assert_eq!(slicing_notification_tag(2), "SlicingNeedSupportOn");
        assert_eq!(slicing_notification_tag(99), "SlicingUnknown");
        assert_eq!(warning_level_tag(true), "critical");
        assert_eq!(warning_level_tag(false), "non_critical");
        assert_eq!(
            string_exception_tag(5),
            "STRING_EXCEPT_LAYER_HEIGHT_EXCEEDS_LIMIT"
        );
        assert_eq!(string_exception_tag(42), "STRING_EXCEPT_UNKNOWN");
    }
}
