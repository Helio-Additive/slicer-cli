//! Debug / parity-diagnostic tooling — NOT part of the C++ libslic3r structure.
//!
//! These modules have no counterpart in BambuStudio `src/libslic3r`; they exist
//! only to help drive and verify the C++ -> Rust port. They are grouped under
//! `debug/` so the rest of the crate mirrors the C++ file tree 1:1.
//!
//! - `topdbg`         — per-stage Top-surface tracing (env-gated, TOPDBG_*).
//! - `function_trace` — lightweight call/trace instrumentation.
//! - `compare`        — DOM-style G-code parser + comparator (gcode-diff harness).
//! - `validation`     — G-code validation built on `compare`.

pub mod compare;
pub mod function_trace;
pub mod topdbg;
pub mod validation;
