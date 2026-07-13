//! Staged pipeline dumper for cross-engine parity bisection (R313).
//!
//! Enable with `STAGEDUMP=<layer_index>`: every instrumented pipeline stage
//! prints one checksum line for that layer:
//!
//! `SD key=<layer> stage=<name> n=<pieces+holes> np=<points> sx=<Σx> sy=<Σy>`
//!
//! The native counterpart patch lives in `docs/native_stagedump.patch`
//! (applied to the reference BambuStudio tree per probe session and reverted
//! after, per the instrumentation guardrail). Keys are LAYER INDICES on both
//! sides — the native patch derives idx from the layer object, never from z
//! (probe-pairing protocol, R289).
//!
//! Inert unless the env var is set; safe to keep compiled in.

use crate::geometry::ExPolygon;

/// Parsed STAGEDUMP layer index, or None when disabled.
pub fn stagedump_key() -> Option<usize> {
    std::env::var("STAGEDUMP").ok().and_then(|v| v.parse().ok())
}

/// Dump one stage checksum line if STAGEDUMP matches `key`.
pub fn dump(stage: &str, key: usize, eps: &[ExPolygon]) {
    if stagedump_key() != Some(key) {
        return;
    }
    let mut sx: i64 = 0;
    let mut sy: i64 = 0;
    let mut np = 0usize;
    let mut n = 0usize;
    for ex in eps {
        n += 1 + ex.holes.len();
        for pt in &ex.contour.points {
            np += 1;
            sx = sx.wrapping_add(pt.x);
            sy = sy.wrapping_add(pt.y);
        }
        for h in &ex.holes {
            for pt in h.points() {
                np += 1;
                sx = sx.wrapping_add(pt.x);
                sy = sy.wrapping_add(pt.y);
            }
        }
    }
    eprintln!("SD key={} stage={} n={} np={} sx={} sy={}", key, stage, n, np, sx, sy);
}
