# R67 — Arachne concentric fill: the deficit is the unported Voronoi graph builder, not bead under-production

Branch: `arachne-fill` (off `alex/libslic3r-parity-engine` @ e685ca7).
Job: Benchy, `tests/configs/stl-inline-config.jsonnet`.
Metric: `/tmp/feat_e2.py` (per-`;FEATURE` E summed only over moves with real XY motion).

## TL;DR / verdict

**Clean bail with a sharp, evidence-backed diagnosis that overturns the brief's premise.**

The remaining Internal-solid / Floating-vertical-shell infill deficit is **NOT** caused by
rust's Arachne `WallToolPaths` *under-producing* beads. On this branch **rust's Arachne
produces ZERO beads for every input** — the variable-width bead pipeline is a complete
no-op — because the keystone graph builder
`SkeletalTrapezoidation::construct_from_polygons` (the Voronoi-diagram → half-edge-graph
construction) is **not ported**. `WallToolPaths::generate()` explicitly **stubs out** the
wall-maker step.

The PART A wiring was implemented and proven faithful (it routes the concentric patterns
through `FillConcentricInternal` exactly as C++ Fill.cpp does), but because Arachne emits
nothing it is a **net regression** (floating 128→0, total −221), so it was **reverted**.
Working tree is clean; no C++ instrumentation was used (none needed — the gap is in rust).

## Measured per-feature table (native vs rust)

Baseline (this branch, restored after revert — the live crude `generate_concentric_infill` shim):

| FEATURE                 | native  | rust    | dE      | note |
|-------------------------|---------|---------|---------|------|
| Internal solid infill   | 494.08  | 418.12  | −75.96  | UNDER (main target) |
| Floating vertical shell | 170.76  | 128.27  | −42.50  | UNDER |
| Sparse infill           | 531.63  | 599.85  | +68.22  | OVER |
| Bridge                  | 237.87  | 254.69  | +16.82  | guardrail |
| Outer wall              | 1003.08 | 1005.21 | +2.13   | guardrail |
| Inner wall              | 995.23  | 997.98  | +2.75   | guardrail |
| Gap infill              | 230.58  | 230.79  | +0.20   | guardrail |
| TOTAL-MATERIAL          | 3858.97 | 3846.65 | −12.31  | |

PART A wiring (FillConcentricInternal / FloatingConcentric routed through the faithful
Arachne filler), measured before revert:

| FEATURE                 | native  | rust    | dE      | vs baseline |
|-------------------------|---------|---------|---------|-------------|
| Internal solid infill   | 494.08  | 337.16  | −156.92 | −81 (dropped further) |
| Floating vertical shell | 170.76  | **0.00**| −170.76 | −128 (to ZERO) |
| TOTAL-MATERIAL          | 3858.97 | 3637.41 | −221.56 | net regression |

The residual 337 ISI is **non-concentric** solid infill (rectilinear/monotonic);
floating dropping to **exactly 0** is the smoking gun.

## The root cause, pinned exactly

Instrumentation (env-gated `ARACHNE_DBG`, in the reverted PART A branch) over every
concentric `fill_surface_extrusion` call in a full Benchy slice:

```
549 concentric fill calls — ALL produced 0 entities.
```

This includes LARGE regions, not just sub-bead strips, e.g.
`no_overlap=[(3391500, 478300)]` (3.39mm × 0.48mm) with `min_spacing=37708`
(0.377mm scaled) → `loops_count = 3391500/37708 + 1 = 90` → should emit ~tens of beads.
It emits **none**.

Tracing why:

- `FillConcentricInternal::fill_surface_extrusion`
  (`crates/libslic3r-rs/src/fill/fill_concentric_internal.rs:136`) calls
  `WallToolPaths::get_tool_paths()` → `WallToolPaths::generate()`.
- `WallToolPaths::generate()`
  (`crates/libslic3r-rs/src/arachne/wall_tool_paths.rs:1010-1018`) reaches the wall-maker
  step (C++ WallToolPaths.cpp:520-532) and **stubs it out**:

  ```rust
  // BLOCKED — WallToolPaths.cpp:520-532
  //   SkeletalTrapezoidation wall_maker(prepared_outline, *beading_strat, ...);
  //   wall_maker.generateToolpaths(toolpaths);
  // ... We cannot run the wall-maker, so `toolpaths` stays empty here.
  ```

  So `self.toolpaths` is **always empty**; everything downstream (`stitch_tool_paths`,
  `separate_out_inner_contour`, …) is a faithful no-op on empty input. `get_tool_paths`
  returns `[]`, `FillConcentricInternal` produces zero `thick_polylines_out`, zero
  entities. Every concentric surface ⇒ nothing.

### Why the wall-maker is stubbed: the unported Voronoi graph builder

`SkeletalTrapezoidation::generate_toolpaths` **IS** ported and functional
(`crates/libslic3r-rs/src/arachne/skeletal_trapezoidation.rs:224`), as are its consumers
`generate_segments` (:1770), `graph.make_rib`, `graph.collapse_small_edges`. The wall-maker
comment in wall_tool_paths.rs:14 ("it has no `generate_toolpaths`") is **inaccurate** — it
does. What is missing is the **graph that `generate_toolpaths` consumes**.

The C++ ctor body runs `constructFromPolygons(polys)` which builds the half-edge graph
from a boost::polygon Voronoi diagram. In the rust port these 5 functions are **NOT
ported** (no impls exist in `skeletal_trapezoidation.rs`; the module header at
skeletal_trapezoidation.rs:22-31 lists them as BLOCKED):

| C++ fn (SkeletalTrapezoidation.cpp) | ~lines | rust status |
|-------------------------------------|--------|-------------|
| `makeNode`                          | 92–106 (~15)  | missing |
| `transferEdge`                      | 107–217 (~111)| missing |
| `discretize`                        | 218–329 (~112)| missing |
| `computePointCellRange`             | 330–390 (~61) | missing |
| `constructFromPolygons`             | 391–504 (~114)| missing |

Without them `self.graph` is empty, so even if `generate_toolpaths` were wired into
`WallToolPaths::generate` it would emit nothing.

## The brief's premise was wrong (and why R47 differed)

The brief framed PART B as: "rust WallToolPaths UNDER-produces beads (ISI-path 16031 native
vs 9730 rust); fix the bead count." That cannot be true on `arachne-fill` @ e685ca7 — rust
emits **0** beads, not 9730. The R47 ISI 414→299 figure must have come from a **different
branch** that had a (partial/working) `construct_from_polygons`. On THIS branch the Arachne
pipeline is a complete no-op; there is no "under-production" to tune — the graph builder is
simply absent.

## The next step — bounded and high-leverage (the real long-pole)

Port the 5 functions above against the crate's `boostvoronoi` (`bv`) diagram and wire the
wall-maker into `WallToolPaths::generate`. This is the **keystone that unblocks the entire
Arachne pipeline** (concentric infill here, and the Arachne perimeter path elsewhere).

The "blocked on a boost VD pointer-traversal layer the crate doesn't expose" note
(skeletal_trapezoidation.rs:22-23) is **stale/overcautious**: the `bv::Diagram` index API
that `construct_from_polygons` needs is already used elsewhere in the crate, so the
primitives exist:

- `crates/libslic3r-rs/src/geometry/voronoi_diagram.rs` — `VoronoiDiagram::construct_voronoi`
  over segment inputs; `diagram.edges()/.vertices()/.cells()`.
- `crates/libslic3r-rs/src/geometry/voronoi_utils_cgal.rs:264-268` — already navigates
  `diagram.edge_get_vertex0/1(edge_id)`, `diagram.vertices()[..]` via `bv::EdgeIndex`.
- `crates/libslic3r-rs/src/geometry/voronoi_utils.rs` — `SegmentCellRange`,
  `discretize_parabola`, `to_point`, `is_finite`, `make_rotated_vertex` ported; the
  `bv::Cell`/`bv::Diagram` `get_source_segment` / `get_source_point` /
  `compute_segment_cell_range` ports live in `voronoi_utils_cgal.rs`.
- `crates/libslic3r-rs/src/arachne/utils/polygons_segment_index.rs` /
  `polygons_point_index.rs` — the `Segment`/`PolygonsPointIndex` source-index types
  `constructFromPolygons` builds and passes through.

Concrete plan:
1. Port `makeNode`, `transferEdge`, `discretize`, `computePointCellRange`,
   `constructFromPolygons` into `skeletal_trapezoidation.rs`, building `self.graph`
   (the `vd_edge_to_he_edge` / `vd_node_to_he_node` maps key on `bv::EdgeIndex` /
   `bv::VertexIndex` instead of boost pointers). Reuse `voronoi_utils_cgal.rs`'s
   `compute_segment_cell_range` / `get_source_segment` and `voronoi_utils.rs`'s
   `discretize_parabola`. The graph mutators it calls (`graph.make_rib`,
   `separate_pointy_quad_end_nodes`, `graph.collapse_small_edges`) already exist.
2. In `WallToolPaths::generate` (wall_tool_paths.rs:1010-1018) replace the stub with:
   construct the `BeadingStrategy` (already built at :989), make a `SkeletalTrapezoidation`,
   `construct_from_polygons(&prepared_outline)`, then
   `wall_maker.generate_toolpaths(&mut self.toolpaths)`.
3. Re-apply PART A (recipe below), build, measure. EXPECT ISI/floating to move toward
   native (494 / 170) once real beads flow. Watch the sparse +68 over (it partly masks the
   under) and the outer/inner-wall guardrails.

## PART A — the wiring (faithful; reverted only because Arachne is a no-op today)

Re-apply once the graph builder lands. In `Layer::make_fills` (`crates/libslic3r-rs/src/layer.rs`),
immediately after `let fill_pattern = surface_fill.params.pattern;` (~:1843), add a branch for
`InfillPattern::Concentric | InfillPattern::FloatingConcentric` that BYPASSES the polyline
path and `continue`s. Per C++ Fill.cpp:695,706,738-749, per expoly:

- `loop_clipping = scale_(flow.nozzle_diameter() * 0.15)` (LOOP_CLIPPING const, libslic3r.h:62).
- `FillParams`: `density = 0.01*params.density`, `flow = params.flow`,
  `extrusion_role = params.extrusion_role`, `use_arachne = true`,
  `layer_height = self.height`, `using_internal_flow = !surface.is_solid() && !params.bridge`.
- `print_config` / `print_object_config` from `self.print_config` / `self.object_config`
  (deref the stamped Arcs).
- For each expoly: `no_overlap = intersection(surface_fill.no_overlap_expolygons, [expoly])`
  (plain geo-clipper intersection — matches the established `mono_no_overlap` precedent at
  layer.rs:1908; C++ uses `ApplySafetyOffset::Yes` which the f64 backend doesn't apply).
  Build `FillConcentricInternal { spacing: params.spacing, loop_clipping,
  no_overlap_expolygons: no_overlap, print_config, print_object_config }` and call
  `fill_surface_extrusion(&surface, &fp, &mut out)`; push `out` into
  `self.regions[region_id].fills.entities`.

The full reverted diff is in this branch's reflog (the change was applied, measured, then
reverted in one session) — reconstruct from the recipe above; it is ~5 lines of context plus
the per-expoly loop.

### FloatingConcentric caveat (independent second blocker)

`FillFloatingConcentric::fill_surface_extrusion` is the live C++ path (the offset2_ex
`_fill_surface_single` at FillFloatingConcentric.cpp:732-804 is dead `#if 0`). It calls
`fill_surface_arachne_floating` → `_fill_surface_single` (the **Arachne** one, :879, using
the same `WallToolPaths`) → `resplit_order_loops` → `detect_floating_line`, which needs a
**Z-aware clipper with a user `ZFillFunction`** (`ClipperLib_Z`) that this crate's Clipper2
backend does not provide (documented in `fill_floating_concentric.rs:15-31`). The bead
GEOMETRY is the same WallToolPaths output, so for MATERIAL parity FloatingConcentric can be
routed through `FillConcentricInternal` as a documented interim — only the
`FloatingVerticalShell` customize-flag tagging (the `;FEATURE:Floating vertical shell`
attribution) and the floating seam re-ordering differ. NOTE: routing it through
`FillConcentricInternal` re-tags those paths via `params.extrusion_role`, which
`group_fills` already sets to `ExtrusionRole::FloatingVerticalShell` (fill/mod.rs:832), so
the feature attribution is preserved even via the interim — the floating→0 seen above was
purely the empty-Arachne no-op, not a tagging loss.

## Files

- `crates/libslic3r-rs/src/arachne/wall_tool_paths.rs:1010-1018` — the wall-maker stub (the bug).
- `crates/libslic3r-rs/src/arachne/skeletal_trapezoidation.rs:22-31, 122-159, 224` — graph
  builder BLOCKED note, ctor (graph left empty), and the ported `generate_toolpaths`.
- `crates/libslic3r-rs/src/fill/fill_concentric_internal.rs` — faithful FillConcentricInternal (ready).
- `crates/libslic3r-rs/src/fill/fill_floating_concentric.rs` — helpers ported,
  `fill_surface_extrusion` blocked on Z-clipper.
- `crates/libslic3r-rs/src/layer.rs:1803` — `make_fills` (PART A insertion point ~:1843).
- C++: `Fill/FillConcentricInternal.cpp`, `Fill/FillFloatingConcentric.cpp`,
  `Arachne/WallToolPaths.cpp:520-532`, `Arachne/SkeletalTrapezoidation.cpp:391-504`.
