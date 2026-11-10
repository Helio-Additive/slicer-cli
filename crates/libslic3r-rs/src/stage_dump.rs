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

// ---------------------------------------------------------------------------
// R810 — `PP=1`: order-independent checksums of every layer at the step
// boundaries (after slice, after make_perimeters, after infill). One line per
// layer and stage: `PP <stage> L<idx> n=<pieces> np=<points> sx=<sum x> sy=<sum y>`.
// The C++ twin prints the same lines from PrintObject.cpp, so the first stage
// and layer where the two engines differ can be read off a diff. Diagnostics
// only; nothing here changes output.

/// One checksum accumulator.
pub struct PpAcc {
    n: usize,
    np: usize,
    sx: i64,
    sy: i64,
    /// FNV-1a over the points in the order they are stored: equal only when the
    /// pieces, their order and each ring's start vertex are the same.
    h: u64,
}

impl Default for PpAcc {
    fn default() -> Self {
        Self { n: 0, np: 0, sx: 0, sy: 0, h: 1469598103934665603 }
    }
}

impl PpAcc {
    fn add_points<'a, I: IntoIterator<Item = &'a crate::geometry::Point>>(&mut self, pts: I) {
        let mix = |h: &mut u64, v: i64| {
            *h ^= v as u64;
            *h = h.wrapping_mul(1099511628211);
        };
        self.n += 1;
        mix(&mut self.h, -1);
        for p in pts {
            self.np += 1;
            self.sx = self.sx.wrapping_add(p.x);
            self.sy = self.sy.wrapping_add(p.y);
            mix(&mut self.h, p.x);
            mix(&mut self.h, p.y);
        }
    }

    pub fn add_expolygon(&mut self, ex: &ExPolygon) {
        self.add_points(ex.contour.points.iter());
        for h in &ex.holes {
            self.add_points(h.points().iter());
        }
    }

    pub fn add_entities(&mut self, coll: &crate::extrusion_entity::ExtrusionEntityCollection) {
        for path in coll.flatten_paths() {
            self.add_points(path.polyline.points().iter());
        }
    }

    pub fn print(&self, stage: &str, layer: usize) {
        if self.n == 0 {
            return;
        }
        eprintln!(
            "PP {} L{} n={} np={} sx={} sy={} h={:x}",
            stage, layer, self.n, self.np, self.sx, self.sy, self.h
        );
    }
}

pub fn pp_enabled() -> bool {
    crate::probe_enabled("PP")
}

/// After `slice()`.
pub fn pp_slices(layers: &[crate::layer::Layer]) {
    if !pp_enabled() {
        return;
    }
    for (i, layer) in layers.iter().enumerate() {
        let mut a = PpAcc::default();
        for ex in &layer.lslices {
            a.add_expolygon(ex);
        }
        a.print("lslices", i);
        let mut b = PpAcc::default();
        for region in layer.regions() {
            for s in &region.slices.surfaces {
                b.add_expolygon(&s.expolygon);
            }
        }
        b.print("rslices", i);
    }
}

/// After `make_perimeters()`.
pub fn pp_perimeters(layers: &[crate::layer::Layer]) {
    if !pp_enabled() {
        return;
    }
    for (i, layer) in layers.iter().enumerate() {
        let (mut a, mut b, mut c) = (PpAcc::default(), PpAcc::default(), PpAcc::default());
        for region in layer.regions() {
            a.add_entities(&region.perimeters);
            c.add_entities(&region.thin_fills);
            for s in &region.fill_surfaces.surfaces {
                b.add_expolygon(&s.expolygon);
            }
        }
        a.print("perim", i);
        b.print("perim_fillsurf", i);
        c.print("perim_thin", i);
    }
}

/// After `infill()`.
pub fn pp_infill(layers: &[crate::layer::Layer]) {
    if !pp_enabled() {
        return;
    }
    if let Some(layer) = std::env::var("PPL")
        .ok()
        .and_then(|v| v.parse::<usize>().ok())
        .and_then(|l| layers.get(l))
    {
        for region in layer.regions() {
            pp_detail("fills", &region.fills, 0);
        }
    }
    use crate::surface::SurfaceType as T;
    for (i, layer) in layers.iter().enumerate() {
        let mut by_type: std::collections::BTreeMap<&'static str, PpAcc> = Default::default();
        let mut fills = PpAcc::default();
        for region in layer.regions() {
            for s in &region.fill_surfaces.surfaces {
                let name = match s.surface_type {
                    T::Top => "prep_top",
                    T::Bottom => "prep_bottom",
                    T::BottomBridge => "prep_bottombridge",
                    T::Internal => "prep_internal",
                    T::FloatingVerticalShell => "prep_fvs",
                    T::InternalSolid => "prep_internalsolid",
                    T::SubTop => "prep_subtop",
                    T::InternalBridge => "prep_internalbridge",
                    T::InternalVoid => "prep_internalvoid",
                    T::Perimeter => "prep_perimeter",
                    #[allow(unreachable_patterns)]
                    _ => "prep_other",
                };
                by_type.entry(name).or_default().add_expolygon(&s.expolygon);
            }
            fills.add_entities(&region.fills);
        }
        for name in [
            "prep_top", "prep_bottom", "prep_bottombridge", "prep_internal", "prep_fvs",
            "prep_internalsolid", "prep_subtop", "prep_internalbridge", "prep_internalvoid",
            "prep_perimeter", "prep_other",
        ] {
            if let Some(acc) = by_type.get(name) {
                acc.print(name, i);
            }
        }
        fills.print("fills", i);
    }
}

/// `PPL=<layer>` with `PP=1`: every entity of that layer's perimeters, one line
/// each (`PPD perim ...`), in the form the C++ twin prints.
fn pp_detail(tag: &str, coll: &crate::extrusion_entity::ExtrusionEntityCollection, depth: usize) {
    use crate::extrusion_entity::ExtrusionEntityType as E;
    let pad = " ".repeat(depth * 2);
    eprintln!("PPD {} {}coll n={} no_sort={}", tag, pad, coll.entities.len(), coll.no_sort as i32);
    let path_line = |p: &crate::extrusion_entity::ExtrusionPath, depth: usize| {
        let pad = " ".repeat(depth * 2);
        let q = p.polyline.points();
        let (f, l) = (q.first(), q.last());
        eprintln!(
            "PPD {} {}path role={:?} np={} od={:.3} w={:.5} mm3={:.6} first=({},{}) last=({},{})",
            tag, pad, p.role, q.len(), p.overhang_degree, p.width, p.mm3_per_mm,
            f.map_or(0, |p| p.x), f.map_or(0, |p| p.y), l.map_or(0, |p| p.x), l.map_or(0, |p| p.y)
        );
    };
    for e in &coll.entities {
        match e {
            E::Path(p) => path_line(p, depth + 1),
            E::Loop(l) => {
                eprintln!("PPD {} {}  loop paths={}", tag, pad, l.paths.len());
                for p in &l.paths {
                    path_line(p, depth + 2);
                }
            }
            E::Collection(c) => pp_detail(tag, c, depth + 1),
        }
    }
}

pub fn pp_perimeter_detail(layers: &[crate::layer::Layer]) {
    if !pp_enabled() {
        return;
    }
    let Some(l) = std::env::var("PPL").ok().and_then(|v| v.parse::<usize>().ok()) else {
        return;
    };
    if let Some(layer) = layers.get(l) {
        for region in layer.regions() {
            pp_detail("perim", &region.perimeters, 0);
        }
        // `PPF c|h x y`: every point of the fill surfaces the perimeter step left.
        for region in layer.regions() {
            for s in &region.fill_surfaces.surfaces {
                for p in &s.expolygon.contour.points {
                    eprintln!("PPF c {} {}", p.x, p.y);
                }
                for h in &s.expolygon.holes {
                    for p in h.points() {
                        eprintln!("PPF h {} {}", p.x, p.y);
                    }
                }
            }
        }
    }
}

/// `PP=1`: the typed surfaces of every layer after one step of
/// `prepare_infill` — `PP <stage>_s_<type>` for the region slices and
/// `PP <stage>_f_<type>` for the fill surfaces.
pub fn pp_typed(stage: &str, layers: &[crate::layer::Layer]) {
    if !pp_enabled() {
        return;
    }
    use crate::surface::SurfaceType as T;
    let name = |t: T| match t {
        T::Top => "top",
        T::Bottom => "bottom",
        T::BottomBridge => "bottombridge",
        T::Internal => "internal",
        T::FloatingVerticalShell => "fvs",
        T::InternalSolid => "internalsolid",
        T::SubTop => "subtop",
        T::InternalBridge => "internalbridge",
        T::InternalVoid => "internalvoid",
        T::Perimeter => "perimeter",
        #[allow(unreachable_patterns)]
        _ => "other",
    };
    for (i, layer) in layers.iter().enumerate() {
        let mut accs: std::collections::BTreeMap<String, PpAcc> = Default::default();
        for region in layer.regions() {
            for s in &region.slices.surfaces {
                accs.entry(format!("{}_s_{}", stage, name(s.surface_type)))
                    .or_default()
                    .add_expolygon(&s.expolygon);
            }
            for s in &region.fill_surfaces.surfaces {
                accs.entry(format!("{}_f_{}", stage, name(s.surface_type)))
                    .or_default()
                    .add_expolygon(&s.expolygon);
            }
        }
        for (k, acc) in &accs {
            acc.print(k, i);
        }
        // `PPL=<layer> PPS=<stage>`: every fill-surface point, `PPT <type> c|h x y`.
        let want_layer = std::env::var("PPL").ok().and_then(|v| v.parse::<usize>().ok());
        if want_layer == Some(i) && std::env::var("PPS").ok().as_deref() == Some(stage) {
            for region in layer.regions() {
                for s in &region.fill_surfaces.surfaces {
                    let t = name(s.surface_type);
                    for p in &s.expolygon.contour.points {
                        eprintln!("PPT {} c {} {}", t, p.x, p.y);
                    }
                    for h in &s.expolygon.holes {
                        for p in h.points() {
                            eprintln!("PPT {} h {} {}", t, p.x, p.y);
                        }
                    }
                }
            }
        }
    }
}
