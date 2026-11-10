//! The two-engine parity harness: one job, sliced by `slicer_cli` (the C++
//! Bambu Studio engine, a subprocess) and by the Rust engine (in-process), so
//! the two G-codes can be compared.
//!
//! This is the part of the `alex/frame-unify` branch's wrapper program that the
//! parity work needs: job configs (JSON or Jsonnet), profile resolution into
//! one flat settings bundle, and the `slice` and `compare` commands. Remote
//! locations (S3, HTTP), callbacks and the profile catalogue commands were
//! left behind.

pub mod bambu;
pub mod commands;
pub mod config;
pub mod job_config;
pub mod json_utils;
pub mod locations;
pub mod profiles;
