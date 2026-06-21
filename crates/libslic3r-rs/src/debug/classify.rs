//! CLASSIFY_DUMP — env-gated per-stage surface-classification tracing.
//!
//! NOT part of the C++ port. Pure diagnostics for the Benchy
//! surface-classification parity gap (internal-solid 201 Rust vs 389 native,
//! sparse +143 over). Mirrors the C++ `classify_dump` helper added to the
//! BambuStudio override `PrintObject.cpp` 1:1, so the two engines emit
//! byte-comparable per-layer per-stage surface-type counts/areas.
//!
//! Activation: set env `CLASSIFY_DUMP=/path/to/file` (lines are appended).
//! Each line:
//!   `RUST\t<stage>\tL<layer>\tTop=<cnt>/<area>\tBottom=...\t...`
//! where `<area>` is the summed scaled^2 area for that type across all regions
//! of the layer (matches C++ `Surface::area()` units exactly).

use crate::surface::SurfaceType;
use crate::surface_collection::SurfaceCollection;
use std::io::Write;

/// True when CLASSIFY_DUMP tracing is enabled (path is non-empty).
pub fn path() -> Option<&'static str> {
    use std::sync::OnceLock;
    static P: OnceLock<Option<String>> = OnceLock::new();
    P.get_or_init(|| std::env::var("CLASSIFY_DUMP").ok().filter(|s| !s.is_empty()))
        .as_deref()
}

const TYPES: [(SurfaceType, &str); 7] = [
    (SurfaceType::Top, "Top"),
    (SurfaceType::Bottom, "Bottom"),
    (SurfaceType::BottomBridge, "BottomBridge"),
    (SurfaceType::Internal, "Internal"),
    (SurfaceType::InternalSolid, "InternalSolid"),
    (SurfaceType::InternalBridge, "InternalBridge"),
    (SurfaceType::InternalVoid, "InternalVoid"),
];

/// Dump per-layer per-stage surface-type counts + areas for all layers.
/// `fill_surfaces_of` yields the fill_surfaces of every region of a layer.
pub fn dump<'a, I, R>(stage: &str, layers: I)
where
    I: IntoIterator<Item = R>,
    R: IntoIterator<Item = &'a SurfaceCollection>,
{
    let Some(p) = path() else {
        return;
    };
    let Ok(mut f) = std::fs::OpenOptions::new().create(true).append(true).open(p) else {
        return;
    };
    for (li, regions) in layers.into_iter().enumerate() {
        let mut cnt = [0i64; 7];
        let mut area = [0.0f64; 7];
        for region in regions {
            for s in &region.surfaces {
                for (t, (ty, _)) in TYPES.iter().enumerate() {
                    if s.surface_type == *ty {
                        cnt[t] += 1;
                        area[t] += s.area();
                        break;
                    }
                }
            }
        }
        let mut line = format!("RUST\t{}\tL{}", stage, li);
        for (t, (_, name)) in TYPES.iter().enumerate() {
            line.push_str(&format!("\t{}={}/{:.1}", name, cnt[t], area[t]));
        }
        line.push('\n');
        let _ = f.write_all(line.as_bytes());
    }
}
