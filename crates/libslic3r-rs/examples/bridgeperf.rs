//! Bridges wave_seeds region-expansion micro-benchmark (PERF task).
//!
//! Run with:  BRIDGEPERF=1 cargo run --example bridgeperf --manifest-path crates/libslic3r-rs/Cargo.toml
//!
//! Profiles the faithful wave_seeds + propagate path in ISOLATION (no full
//! slice) to pin the bottleneck. See `slicer::region_expansion::bridgeperf`.
fn main() {
    slicer::region_expansion::run_bridgeperf_microbench();
}
