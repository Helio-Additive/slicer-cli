//! TOPDBG — env-gated per-stage Top-surface tracing instrumentation.
//!
//! NOT part of the C++ port. Pure diagnostics for the Benchy
//! surface-classification parity gap (Top surface 142 golden vs 5 Rust):
//! localizes which pipeline stage drops stTop surfaces.
//!
//! Activation (all default-off; zero effect on the default path):
//! - `TOPDBG=1` — print one line per stage per layer that has (or ever had)
//!   any Top surface:
//!   `TOPDBG layer=<id> stage=<name> count=<n> area=<a>`
//!   (area is in scaled^2 units, i.e. raw clipper coordinates squared).
//!   Once a layer has shown a Top surface at any stage, later stages keep
//!   printing for it even at count=0, so drops are visible.
//! - `TOPDBG_DUMP=<layer_id>` — additionally dump polygon coordinate text
//!   for that layer to `/tmp/topdbg/L<layer>_<name>.txt`.

use std::collections::HashSet;
use std::sync::{Mutex, OnceLock};

use crate::geometry::ExPolygon;
use crate::surface::Surface;

/// True when TOPDBG tracing is enabled.
pub fn enabled() -> bool {
    static ON: OnceLock<bool> = OnceLock::new();
    *ON.get_or_init(|| std::env::var("TOPDBG").is_ok())
}

/// Layer id selected for polygon dumps via TOPDBG_DUMP=<id>, if any.
pub fn dump_target() -> Option<usize> {
    static TGT: OnceLock<Option<usize>> = OnceLock::new();
    *TGT.get_or_init(|| std::env::var("TOPDBG_DUMP").ok().and_then(|v| v.parse().ok()))
}

/// Layers that have ever shown a Top surface at any logged stage.
fn seen() -> &'static Mutex<HashSet<usize>> {
    static SEEN: OnceLock<Mutex<HashSet<usize>>> = OnceLock::new();
    SEEN.get_or_init(|| Mutex::new(HashSet::new()))
}

/// Log one stage line for a layer. Prints when count > 0 or when the layer
/// has previously shown Top surfaces (so drops to 0 stay visible).
pub fn log_top(layer_id: usize, stage: &str, count: usize, area: f64) {
    if !enabled() {
        return;
    }
    let mut seen = seen().lock().unwrap();
    if count > 0 {
        seen.insert(layer_id);
    } else if !seen.contains(&layer_id) {
        return;
    }
    println!(
        "TOPDBG layer={} stage={} count={} area={:.6e}",
        layer_id, stage, count, area
    );
}

/// Count + total area of stTop surfaces in a slice of surfaces, then log.
pub fn log_top_surfaces(layer_id: usize, stage: &str, surfaces: &[Surface]) {
    if !enabled() {
        return;
    }
    let mut count = 0usize;
    let mut area = 0.0f64;
    for s in surfaces {
        if s.is_top() {
            count += 1;
            area += s.area();
        }
    }
    log_top(layer_id, stage, count, area);
}

/// Dump expolygons of the TOPDBG_DUMP target layer as coordinate text to
/// /tmp/topdbg/L<layer>_<name>.txt. No-op for other layers / when disabled.
pub fn dump_expolygons(layer_id: usize, name: &str, expolys: &[ExPolygon]) {
    if !enabled() || dump_target() != Some(layer_id) {
        return;
    }
    let _ = std::fs::create_dir_all("/tmp/topdbg");
    let mut out = String::new();
    out.push_str(&format!(
        "# layer={} name={} expolygons={}\n",
        layer_id,
        name,
        expolys.len()
    ));
    for (i, ep) in expolys.iter().enumerate() {
        out.push_str(&format!("EXPOLY {} area={:.6e}\n", i, ep.area()));
        out.push_str("  CONTOUR");
        for p in &ep.contour.points {
            out.push_str(&format!(" {},{}", p.x, p.y));
        }
        out.push('\n');
        for (h, hole) in ep.holes.iter().enumerate() {
            out.push_str(&format!("  HOLE {}", h));
            for p in &hole.points {
                out.push_str(&format!(" {},{}", p.x, p.y));
            }
            out.push('\n');
        }
    }
    let path = format!("/tmp/topdbg/L{}_{}.txt", layer_id, name);
    let _ = std::fs::write(path, out);
}

/// Dump only the stTop expolygons of a surface list for the target layer.
pub fn dump_top_surfaces(layer_id: usize, name: &str, surfaces: &[Surface]) {
    if !enabled() || dump_target() != Some(layer_id) {
        return;
    }
    let tops: Vec<ExPolygon> = surfaces
        .iter()
        .filter(|s| s.is_top())
        .map(|s| s.expolygon.clone())
        .collect();
    dump_expolygons(layer_id, name, &tops);
}
