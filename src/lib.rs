//! The command line of the Rust slicing engine.
//!
//! Two programs are built from this package:
//!
//! - `slicer_cli-rust` ([`engine_cli`]): the fourth program beside
//!   `slicer_cli`, `slicer_cli-orcaslicer` and `slicer_cli-snapmaker`. It runs
//!   the in-progress Rust port of the Bambu Studio engine
//!   (`crates/libslic3r-rs`).
//! - `slicer-parity` ([`parity`]): a development tool, not packaged. It runs
//!   one job through `slicer_cli` and through the Rust engine so the two
//!   G-codes can be scored against each other.

pub mod engine_cli;
pub mod parity;
