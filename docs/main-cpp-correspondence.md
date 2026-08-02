# `main.cpp` → Rust correspondence

Where each symbol of the C++ CLI driver `libslic3r/bambustudio/main.cpp` (the
BambuStudio-embedding entry point this project wraps) lives in the Rust rewrite.
Maintainers used to the C++ file can use this to find the equivalent Rust code.

The Rust driver is deliberately **split** across the thin `slicer-cli` binary
(`src/*.rs`, argument handling + job/config resolution) and the `libslic3r-rs`
library (`crates/libslic3r-rs/src/*.rs`, the actual slice). `main.cpp` keeps all
of that in one 84 KB translation unit; the split is the main structural
divergence, so this table is the bridge.

Verified against `main.cpp` @ Jun 21 2026 (84409 bytes) and the crate at R387.
Line numbers are the C++ definition sites; re-grep if `main.cpp` is re-synced.

| `main.cpp` symbol | C++ line | Rust home | Fidelity |
|---|---|---|---|
| `slicing_notification_tag` / `warning_level_tag` / `string_exception_tag` | 59 / 76 / 80 | — | **GAP** (CLI event protocol; see below) |
| `emit_event` / `emit_status_warning` / `emit_validation_event` | 100 / 108 / 123 | — (type `SlicingStatus` exists in `print_base.rs`; the stdout JSON emission does not) | **GAP** (non-gcode; progress/status) |
| `print_usage` | 139 | `src/cli.rs` (clap derive), dispatched from `src/main.rs` | reshaped (clap vs hand-rolled `--help`) |
| `load_json_config` | 171 | `src/config.rs` (`JobConfig::load` / `load_arg` / `input_config`), `src/profiles.rs` (`resolve_config_refs`) | reshaped |
| `apply_explicit_nozzle_mapping` | 211 | `src/profiles.rs::normalize_single_filament_stl_config` (see note) | **divergent** |
| `reassign_objects_to_master_nozzle` | 285 | — | **GAP** |
| `set_default_config` | 337 | `crates/libslic3r-rs/src/preset_bundle.rs` (`FullPrintConfig::defaults` / `full_fff_config`), `print_config.rs` | mirrored (library) |
| `ensure_vector_config_sizes` | 605 | — (partly subsumed by `normalize_single_filament_stl_config`) | **GAP** |
| `main` | 818 | `src/main.rs` → `src/commands.rs::slice` / `compare` → `crates/libslic3r-rs/src/app_slice.rs::slice_to_gcode` (STL) / `slice_3mf_to_gcode` / `load_3mf` (3MF) | reshaped |

## `main()` pipeline stages → Rust (the slicing pipeline itself)

`main()` (main.cpp:818-1590) is the slicing pipeline. The Rust side runs a
**streamlined, structurally divergent** version: `src/commands.rs::slice` →
`crates/libslic3r-rs/src/app_slice.rs::slice_to_gcode` / `slice_3mf_to_gcode`.
It produces correct G-code for valid single-material input (Benchy is byte- and
semantically-equivalent to C++), but it skips several of main()'s front-end
stages rather than mirroring them. Stage-by-stage (in main() order):

| # | main() stage (C++) | Rust status |
|---|--------------------|-------------|
| 1 | `set_default_config` — `FullPrintConfig::defaults()` | **implicit** — typed-struct `Default` impls in `print_config.rs`; not a distinct pipeline call |
| 2 | `load_stl` / `load_bbs_3mf` (populates config from 3MF) | ported — STL via `materialize_input`; 3MF via `app_slice::load_3mf` (Tier-1, divergent) |
| 3 | plate translation / bbox positioning | ported — `app_slice` XY-centering |
| 4 | `PresetBundle::full_config()` resolution | **divergent** — `src/profiles.rs::resolve_config_refs` (STL) / embedded config (3MF) |
| 5 | `ensure_vector_config_sizes` | **N/A (R405)** — subsumed: Rust `PrintConfig` uses typed *scalar* fields (`nozzle_diameter: CoordF`, `filament_type: String`, …), not the dynamic per-extruder vectors this defends; there are no empty-vector `.get_at(0)` panics to prevent |
| 6 | `apply_explicit_nozzle_mapping` | **divergent/GAP** — `profiles.rs` collapses multi→single for STL; general mapping absent |
| 7 | `reassign_objects_to_master_nozzle` | **GAP** — needs per-object `Model` (Tier-2) |
| 8 | prime-tower disable (multi-material detect) | **partial** — `profiles.rs` sets `enable_prime_tower=0` for single-material STL |
| 9 | `Print::apply(model, config)` | **seam ported (R406)** — `print.rs::Print::apply(config, region_config)` applies the config; called from `app_slice` so the pipeline reads `apply()→validate()→process()`. The C++ invalidation/rebuild machinery is N/A (single-slice, fresh Print) and vector sizing is subsumed (scalar config); objects are added separately (mesh→PrintObject builds in the caller) |
| 10 | `Print::validate()` | **ported (R404+R405)** — `print.rs::Print::validate()`, wired into `app_slice` before `process()`; checks: empty objects, no extrusions, spiral-vase, layer-height≤nozzle, and extrusion/line-width (`validate_extrusion_width`); remaining feature-gated checks (wipe-tower diameters/flavor, by-object sequence, organic/adaptive support sync) pending |
| 11 | `Print::process()` | ported (faithful; this is where R390-R398 perf work landed) |
| 12 | `Print::export_gcode()` | ported (`print.export_gcode`) |

**Faithful today:** the *slicing core* (stages 11-12, process + export) and the
single-material front-end result. **Port targets (ask #1), rough order:** (10)
`Print::validate()` — additive, well-bounded, no gcode change for valid input;
(5) `ensure_vector_config_sizes` — config-level, defensive; (9) a faithful
`Print::apply` seam so config sizing/validation happens the C++ way; (6/7) the
multi-nozzle trio (Tier-2, needs per-object Model). These are being ported
incrementally under the parity loop.

## The multi-nozzle config-prep gap (H2D dual physical nozzle)

`main.cpp` prepares a multi-nozzle job in three steps before slicing:
`ensure_vector_config_sizes` (normalize per-extruder vector options) →
`apply_explicit_nozzle_mapping` (derive a cross-nozzle `filament_map` from
`filament_nozzle_map` via the `physical_extruder_map` inverse, honouring
`filament_map_mode` = `NozzleManual` / `AutoForFlush`) →
`reassign_objects_to_master_nozzle` (when a cross-nozzle split was *derived* from
`AutoForFlush`, pin every object to the master-nozzle filament slot).

(Arachne overhang-wall classification — `detect_brigde_wall_arachne` — was ported
and wired in R415: Majora went from 0 to 1170 overhang wall blocks vs C++'s 1276,
material preserved; see PARITY_STATUS R411-R415. The single-material STL path is
classic-walls and unaffected.)

The Rust STL path does **not** replicate this. Instead
`src/profiles.rs::normalize_single_filament_stl_config` **collapses** a
multi-nozzle profile down to a single nozzle / single filament for STL input
(`physical_extruder_map=[0]`, `filament_map=[1]`, `extruder_max_nozzle_count=[1]`,
prime tower off). That is correct and sufficient for single-material STL (Benchy),
which is why Benchy is at parity — but the *general* nozzle mapping is absent, so
a multicolour H2D job that genuinely spreads filaments across both physical
nozzles is not prepared the way `main.cpp` prepares it. The multicolour 3MF path
runs through the separate divergent loader `app_slice.rs::load_3mf`, which is the
Tier-2 Majora work (see memory `project_rust_3mf_tier1.md`), not this trio.

**Follow-up if Majora dual-nozzle parity is pursued:** port the trio faithfully
into a shared pre-slice config-prep step reachable from both `slice_3mf_to_gcode`
and `load_3mf`, gated so the single-material STL collapse is unaffected. Blocked
on a working native-Majora comparison to verify against (no local native binary
as of R387).

## The CLI event protocol gap (non-gcode)

`main.cpp` streams newline-delimited JSON status/warning/validation events on
stdout (`emit_event` and the `*_tag` helpers). The Rust CLI does not emit this
protocol; it prints a human summary and writes the G-code. This does **not**
affect G-code parity — it only matters to a caller that consumes the machine
event stream. Port only if such a consumer is in scope.

## Multicolour / wipe-tower subsystem (added R419-R447)

`main.cpp` itself does not implement these, but a maintainer coming from the C++
tree will look for them by their BambuStudio file names. This is where they live
in the Rust port:

| C++ source | Rust home | Notes |
|---|---|---|
| `GCode/WipeTower.cpp` (generator) | `crates/libslic3r-rs/src/gcode/wipe_tower.rs` | `WipeTower::new` / `plan_toolchange` / `plan_tower` / `generate` -> `Vec<Vec<ToolChangeResult>>`. Tower gcode is emitted in tower-LOCAL coordinates. |
| `GCode.cpp::WipeTowerIntegration` (export side) | `crates/libslic3r-rs/src/gcode/wipe_tower_integration.rs` | `transform_gcode` (GCode.cpp:298) rewrites tower-local `G1` moves into bed coordinates; `substitute_change_filament` injects the evaluated tool-change block into the tower's `[change_filament_gcode]` placeholder (WipeTower.cpp:2466). |
| `PlaceholderParser.cpp` (expression engine) | `crates/libslic3r-rs/src/gcode/gcode_template.rs` | Expression-capable template evaluator: `[var]`/`{expr}`, nestable `{if}/{elsif}/{else}/{endif}`, arithmetic, comparisons, `&&`/`\|\|`, string equality, array indexing. (`gcode/placeholder_parser.rs` is the older fixed-string stub.) |
| `Print.cpp::_make_wipe_tower` variable prep (3313-3330) | `crates/libslic3r-rs/src/gcode/change_filament.rs` | `build_context()` fills the ~29 `change_filament_gcode` variables (temps, retract lengths, `flush_length_1..4` split, per-filament arrays). |
| `GCode/ToolOrderUtils.cpp` (flush optimizer) | `crates/libslic3r-rs/src/gcode/tool_order_utils.rs` | `reorder_filaments_for_minimum_flush_volume` — wired into the psWipeTower pre-pass in `print.rs`, which stores the result on `Print::optimized_layer_tools` so BOTH the tower plan and `emit_layer_by_island` follow one order. |
| `GCode/ToolOrdering.cpp::WipingExtrusions` | `crates/libslic3r-rs/src/gcode/tool_ordering.rs` | Override bookkeeping plus the `is_overriddable` / `is_obj_overriddable` / `is_support_overriddable` predicates. `mark_wiping_extrusions` itself is NOT ported (measured to divert only ~0.7% of Majora's purge, so it was not the tower gap). |
| `MultiMaterialSegmentation.cpp` | `crates/libslic3r-rs/src/multi_material_segmentation.rs` | Painted-region segmentation. `MMS_DEBUG=1` prints frames, per-slot segment bboxes and per-colour painted-line counts. |
| `EdgeGrid.cpp` | `crates/libslic3r-rs/src/edge_grid.rs` | NOTE (R447): `create_from_contours` MERGES contour points into the pre-set bbox (EdgeGrid.cpp:145-151) — it must NOT reset it, because MMS calls `set_bbox()` first with the merged adjacent-layer bbox. |
| `VariableWidth.cpp` | `crates/libslic3r-rs/src/perimeter_generator.rs` | NOTE (R444): an Arachne junction width is a SPACING; convert with `unscale(w) + height*(1-PI/4)` before building the flow (VariableWidth.cpp:66). |
| `PrintObject.cpp::discover_vertical_shells` (1739-2110) | `crates/libslic3r-rs/src/print_object.rs` | NOTE (R450): there are TWO cache paths and both are ported. When `num_printing_regions() > 1 && !interface_shells` (cpp:1759) the per-layer cache is built ONCE over ALL regions — top/bottom unioned across regions, plus the merged perimeter shadow `offset2(lslices, +0.3*min_spacing, -(perimeter_offset + 0.3*min_spacing))` — and shared by every `region_id`. Otherwise it is rebuilt per region (holes collected only once). C++ `offset2(a, +d1, -d2)` is GROW-then-SHRINK, the OPPOSITE of `clipper_utils::offset2`. |
| `Fill/Fill.cpp::_fill_surfaces` (235-295) | `crates/libslic3r-rs/src/fill/mod.rs` | NOTE (R451): infill flow height is `(surface.thickness == -1) ? layer.height : surface.thickness` (cpp:255) — `-1` is a SENTINEL and every `Surface` is constructed with it, so the fallback is taken almost always and MUST be the layer's own height. Sparse-infill *spacing* is separately computed at the OBJECT's configured layer height with `first_layer = false` (cpp:281), deliberately independent of the current layer, so sparse infill stays aligned across a region. |

### Fixtures, and why a 0.2mm-only fixture set hides flow bugs

Every long-standing single-material fixture here slices at 0.2mm. `fill/mod.rs`
used to hardcode `0.2` as the infill flow height, which is *exactly right* at
0.2mm and wrong everywhere else — it survived undetected until Majora (0.3mm)
was measured. A hardcoded height leaves walls perfect and scales every infill
feature's E-per-mm by the layer-height ratio, so it does not look like a flow
bug in aggregate.

`tests/configs/benchy-016.jsonnet` exists to close that hole: single-material
Benchy at **0.16mm**, built from the real BBL profile JSONs so the SAME config
loads in both engines (`--engine bambu` for the C++ reference, `--engine rust`
for the port). Prefer it over `stl-file-config.jsonnet` for any C++ cross-check
— that one's hand-written jsonnet has a numeric `layer_height`, which the C++
loader rejects outright ("invalid json type for layer_height"), so it can only
ever be self-compared.

### Debug/measurement entry points

Environment gates (all off by default unless stated):
`SLICE_PHASE_TIMING=1` (phase + `export_gcode` sub-phase timings), `MMS_DEBUG=1`
(painted segmentation), `FLUSH_PROBE=1` (purge demand vs divertible infill, and
the flush-order cost the optimizer achieves), `WIPE_TOWER_EMIT=0` / `FLUSH_OPT=0`
(opt OUT of the now-default tower emission and flush ordering).

G-code parity is measured with `scripts/semantic_compare.py` (see PARITY_STATUS.md).
It handles `G2`/`G3` arcs, sub-1 `E.01024` values and deretraction-priming, reports
per-feature E + path length + E-per-mm, and separates object-only material from
wipe-tower purge. Do NOT hand-roll G-code E extraction — two rounds of this port
were misled by bespoke parsers that silently dropped arcs or sub-1 extrusions.
