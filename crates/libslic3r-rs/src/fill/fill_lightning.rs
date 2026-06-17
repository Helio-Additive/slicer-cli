//! Lightning infill pattern.
//!
//! C++ Reference:
//! - Fill/FillLightning.hpp
//! - Fill/FillLightning.cpp
//!
//! Faithful 1:1 port of `Slic3r::FillLightning` (FillLightning.cpp). Lightning
//! infill generates tree-like structures that branch from the top surface down
//! to the bottom, providing support with minimal material usage. The tree
//! forest is produced by the `lightning` submodule (`Lightning/Generator.cpp`
//! etc.) and this file turns the per-layer forest into infill polylines.

// FillLightning.cpp:1-6
//   #include "../Print.hpp"
//   #include "../ShortestPath.hpp"
//   #include "ClipperUtils.hpp"
//   #include "FillLightning.hpp"
//   #include "Lightning/Generator.hpp"
use super::lightning::generator::Generator;
use super::{connect_infill_expolygon, multiline_fill, FillParams};
use crate::clipper_utils::intersection_pl;
use crate::geometry::{ExPolygon, Point, Polyline};
use crate::shortest_path::chain_polylines;
use crate::{scaled, CoordF};

// FillLightning.cpp:8 — namespace Slic3r::FillLightning

/// Lightning fill pattern entry point.
///
/// FillLightning.hpp:19 — `class Filler : public Slic3r::Fill`.
///
/// The base `Slic3r::Fill` members that this filler actually reads
/// (`layer_id`, `spacing`, `overlap`) are held here directly, mirroring the
/// inherited C++ fields. `generator` is the raw pointer to the tree forest the
/// G-code pipeline owns (FillLightning.hpp:25 — `Generator *generator`).
#[derive(Debug, Clone, Default)]
pub struct Filler<'a> {
    /// Base `Fill::layer_id` (FillBase.hpp:111).
    pub layer_id: usize,
    /// Base `Fill::spacing` in unscaled coordinates (FillBase.hpp:115).
    pub spacing: CoordF,
    /// Base `Fill::overlap`, infill / perimeter overlap, unscaled (FillBase.hpp:117).
    pub overlap: CoordF,
    /// FillLightning.hpp:25 — `Generator *generator { nullptr }`.
    pub generator: Option<&'a Generator>,
}

impl<'a> Filler<'a> {
    /// FillLightning.hpp:23 — `bool is_self_crossing() override { return false; }`.
    pub fn is_self_crossing(&self) -> bool {
        false
    }

    /// FillLightning.hpp:36 — `bool no_sort() const override { return false; }`.
    /// Let the G-code export reorder the infill lines.
    pub fn no_sort(&self) -> bool {
        false
    }

    /// FillLightning.cpp:10 — `void Filler::_fill_surface_single(...)`.
    pub fn fill_surface_single(
        &self,
        params: &FillParams,
        _thickness_layers: u32,
        _direction: &(f32, Point),
        expolygon: ExPolygon,
        polylines_out: &mut Vec<Polyline>,
    ) {
        // FillLightning.cpp:17
        let generator = match self.generator {
            Some(g) => g,
            None => return,
        };
        // FillLightning.cpp:18
        let layer = generator.get_trees_for_layer(self.layer_id);
        // FillLightning.cpp:19
        //   Polylines fill_lines = layer.convertToLines(to_polygons(expolygon),
        //       scaled<coord_t>(0.5 * this->spacing - this->overlap));
        // FIDELITY-NOTE(F2): C++ uses `scaled<coord_t>` (coord_t = int32_t,
        // libslic3r.h:40) which truncates to 32-bit; the crate `scaled` returns
        // `Coord = i64`. The argument (~0.5*spacing - overlap, sub-mm) scales to
        // well within i32 range so no truncation divergence occurs in practice.
        let mut fill_lines: Vec<Polyline> = layer.convert_to_lines(
            &expolygon.to_polygons(),
            scaled(0.5 * self.spacing - self.overlap),
        );

        // FillLightning.cpp:21
        // Apply multiline offset if needed
        // FillLightning.cpp:22
        multiline_fill(&mut fill_lines, params, self.spacing as f32);
        // FillLightning.cpp:23
        //   Polylines all_polylines = intersection_pl(std::move(fill_lines), expolygon);
        // C++ uses the (Polylines, ExPolygon) overload (ClipperUtils.hpp:529); the
        // single-element `&[expolygon]` slice drives the equivalent (Polylines,
        // ExPolygons) overload (ClipperUtils.hpp:533) — same Clipper boolean.
        // FIDELITY-NOTE(F1): geo-clipper approximation vs C++ ClipperLib.
        let all_polylines: Vec<Polyline> = intersection_pl(&fill_lines, std::slice::from_ref(&expolygon));
        // FillLightning.cpp:24
        if params.dont_connect() || all_polylines.len() <= 1 {
            // FillLightning.cpp:25
            append(polylines_out, chain_polylines(all_polylines, None));
        } else {
            // FillLightning.cpp:27
            // Fill::connect_infill(std::move(all_polylines), expolygon,
            //   polylines_out, this->spacing, params)
            // `connect_infill_expolygon` (fill/mod.rs) is the ExPolygon overload
            // of Fill::connect_infill (FillBase.cpp:1164).
            connect_infill_expolygon(all_polylines, &expolygon, self.spacing, params, polylines_out);
        }
    }
}

/// Custom deleter for `Generator`.
///
/// FillLightning.hpp:14 — `struct GeneratorDeleter { void operator()(Generator *p); }`.
/// In Rust ownership/`Drop` handles destruction; this type is kept as a
/// faithful structural mirror of the C++ deleter used by `GeneratorPtr`.
#[derive(Debug, Clone, Copy, Default)]
pub struct GeneratorDeleter;

impl GeneratorDeleter {
    /// FillLightning.cpp:30 — `void GeneratorDeleter::operator()(Generator *p) { delete p; }`.
    /// Consuming the box drops (frees) the `Generator`, equivalent to `delete p`.
    pub fn call(&self, p: Box<Generator>) {
        // FillLightning.cpp:31 — delete p;
        drop(p);
    }
}

/// FillLightning.cpp:34 — `GeneratorPtr build_generator(const PrintObject &print_object, ...)`.
///
/// BLOCKED: the body `return GeneratorPtr(new Generator(print_object,
/// throw_on_cancel_callback));` requires:
///   1. `PrintObject`, which is not yet threaded through the fill pipeline, and
///   2. `Generator::generateTrees` (Lightning/Generator.cpp), the R-tree /
///      distance-field tree-growth algorithm, which is a separate not-yet-ported
///      dependency (currently only stubbed in `lightning/generator.rs`).
///
/// Once `PrintObject` and the Lightning generator are ported, this becomes:
/// `Box::new(Generator::new(print_object, throw_on_cancel_callback))`.
//
// pub fn build_generator(
//     print_object: &PrintObject,
//     throw_on_cancel_callback: &dyn Fn(),
// ) -> Box<Generator> {
//     // FillLightning.cpp:36
//     Box::new(Generator::new(print_object, throw_on_cancel_callback))
// }

// FillLightning.cpp:39 — } // namespace Slic3r::FillAdaptive

/// `append(dst, src)` — Slic3r helper that moves all elements of `src` onto the
/// end of `dst`. Used at FillLightning.cpp:25.
#[inline]
fn append(dst: &mut Vec<Polyline>, mut src: Vec<Polyline>) {
    if dst.is_empty() {
        *dst = src;
    } else {
        dst.append(&mut src);
    }
}
