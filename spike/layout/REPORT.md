# Layout kernel confirmation spike — REPORT

Issue: Helio-Additive/slicer-cli#7 (delivery step 1: confirmation spike)
Baseline: tag `v0.1.0-dev-6` (`8485231a22e3a1d06108f6badd49b001022a60fb`), branch `spike/issue-7-layout-kernel`
Engine pins: BambuStudio `b506005bc4ee62124e24bf00e0f58656db3646a6`, OrcaSlicer `42cce5399cd9cee7c4b559d6947b0c6bf8455d29`
Host: macOS arm64 (Apple M1 Pro), NLopt 2.10.1, all builds Release.
Everything under `spike/layout/` is **disposable** — the fixture/output JSON is explicitly NOT `LayoutProblemV1`/`PlacementCandidateV1`.

## 1. What was called

`Slic3r::arrangement::arrange(ArrangePolygons &items, const ArrangePolygons &excludes, const Points &bed, const ArrangeParams &params)` (pinned `src/libslic3r/Arrange.hpp`), via a 300-line harness (`spike/layout/layout_spike.cpp`) linking only `libslic3r_core`. No `Model`, no 3MF, no GUI/`ArrangeJob`/`PartPlate` code, no slicing. Dataflow: fixture JSON → mm→`coord_t` scaling → `ArrangePolygon`/`ArrangeParams` → kernel → readback of mutated `translation`/`rotation`/`bed_idx` → placements JSON on stdout (logs on stderr).

## 2. Headless demonstration

Both engines built and ran the full fixture suite headless:

| gate | bambu | orca |
|-|-|-|
| `layout_spike` builds (`-DSPIKE_LAYOUT=ON`) | PASS | PASS |
| production `slicer_cli` target still builds | PASS | (not re-checked; identical CMake) |
| `scripts/smoke-clean-host.sh` (scrubbed env) | PASS (exit 2 = usage) | PASS (exit 2 = usage) |
| `otool -L` matches for wx/GLFW/OpenGL/AppKit | 0 | 0 |

Dynamic deps are the pre-existing `libslic3r_core` set (boost, tbb, NLopt, OpenCASCADE, ICU, OpenSSL, gmp/mpfr, png/z, expat). Binary ≈ 23 MB. The spike adds **zero** new dependencies. One coupling artifact: libslic3r static init prints a single boost-log trace line (`Initializing StaticPrintConfigs`) to **stdout** before `main()`; consumers must parse from the first `{` or the production adapter must install a log sink first (the existing `slicer_cli` already tolerates this pattern).

## 3. Fixed-seed repeatability (`repeat.sh`, 10 runs/fixture, seed 42)

| fixture | bambu | orca |
|-|-|-|
| two_rectangles | 10/10 identical | 10/10 identical |
| five_mixed_sizes | 10/10 identical | 10/10 identical |
| l_shaped_bed | 10/10 identical | 10/10 identical (all no_fit — see §4) |
| locked_center_item | 10/10 identical | 10/10 identical |
| exclusion_zone | 10/10 identical | 10/10 identical |
| rotations_45_90 | 10/10 identical | 10/10 identical |
| `--parallel 0` vs `--parallel 1` | identical hash | identical hash |

Determinism is **measured, not assumed**: the subplex optimizer path is deterministic; `nlopt_srand(seed)` only affects the genetic optimizer this path never invokes. Seed plumbing still belongs in the contract for future solvers.

## 4. Constraint matrix (issue #7 "Input" list)

Legend: SUPPORTED = exercised with passing evidence · DEGRADED = native path exists but loses/weakens the constraint · MISSING = no native field.

| # | contract input | bambu | orca | evidence / native anchor |
|-|-|-|-|
| 1 | opaque stable instance/bed IDs | SUPPORTED | SUPPORTED | adapter-owned mapping; `ArrangePolygon.name` carries the id; no native identity leaks |
| 2 | units / coordinate declaration | SUPPORTED | SUPPORTED | adapter-side mm→`coord_t` (`scaled<coord_t>`); transform semantics verified below |
| 3 | non-rectangular bed polygon | SUPPORTED | **DEGRADED** | bambu `call_with_bed` dispatches Points→BoundingBox/CircleBed/**Polygon** (Arrange.cpp:1071-1089); L-shaped bed packed correctly. Orca: any non-rect bed with finite circle radius becomes a **CircleBed** (Orca Arrange.cpp:1088-1104) — the Polygon branch is effectively dead; L-shaped bed → all 3 items `no_fit` |
| 4 | object footprints + current transforms | SUPPORTED | SUPPORTED | origin-model verified experimentally (both engines): `occupied(p) = R(yaw)·p + translation`, rotation about the footprint frame's origin, **not** centroid-referenced |
| 5 | concave footprints / holes | **DEGRADED** | **DEGRADED** | `process_arrangeable` uses `poly.contour` only (Arrange.cpp:1041) — holes silently dropped; GUI feeds `convex_hull_2d` (Model.cpp:4147), i.e. upstream convexifies. Contract must either declare convex-only or the adapter must convexify + report degradation |
| 6 | locked/fixed objects | **DEGRADED** | **DEGRADED** | native path works (excludes vector → `markAsFixedInBin`): anchor never moved, no overlaps (SAT-checked). **But containment becomes soft**: with a centered locked 60×60 square, bambu placed two items at x 233.7..273.7 (bed ends at 250), orca at y −18.2..+1.8. `AutoArranger<Box>::get_objfn` penalizes overfit with `miss²` instead of rejecting (Arrange.cpp:771-793, the hard-reject branch is commented out). Adapter MUST post-validate containment and demote such candidates |
| 7 | allowed yaw rotations | SUPPORTED | **MISSING** | bambu honors per-item `allowed_rotations` (evidence: both 80×20 sticks placed at exactly yaw 90° where 0°/45° cannot fit). Orca ignores the field: `_arrange` pre-rotates every item by `min_area_boundingbox_rotation` (Orca Arrange.cpp allow_rotations block) and `fill_config` uses global 45° steps — measured yaw −253.1° |
| 8 | object spacing | **DEGRADED** | **DEGRADED** | `ArrangeParams.min_obj_distance` + per-item `inflation`, floored at `MIN_SEPARATION = scale_(0.5)` (libnest2d/nester.hpp:14 → ≥1.0 mm pairwise gap). Measured with spacing=2.0: bambu gap 1.0 mm, orca gap 0.0 mm (contours touching). Semantics differ per engine; adapter must calibrate and the contract must define what spacing means (contour-to-contour) |
| 9 | generated-structure envelopes (brim/skirt) | SUPPORTED (passthrough) | SUPPORTED (passthrough) | `ArrangePolygon.brim_width`/inflation fields exist; GUI-side derivation (`update_selected_items_inflation`, `get_shrink_bedpts`, Arrange.cpp:103-167) is NOT in the kernel path — adapter ports ~60 lines or sets inflation directly |
| 10 | exclusion / obstacle polygons | SUPPORTED | SUPPORTED | `excludes` vector; both engines avoided the corner exclusion (SAT-verified). Pitfall: entries need `bed_idx = 0` — the `UNARRANGED` default is **silently ignored** by `markAsFixedInBin` (found during the spike) |
| 11 | height + sequential-print clearance | SUPPORTED | SUPPORTED | `is_seq_print` + `clearance_height_to_rod/lid` + `printable_height` + clearance radius both engines (field spelled `cleareance_radius` in bambu, `clearance_radius` in orca). Orca pitfall: seq `sortfunc` unconditionally calls `extrude_ids.front()` (Orca Arrange.cpp:815) — **SIGSEGV on empty `extrude_ids`** (reproduced, exit 139); the GUI always populates it, so must the adapter |
| 12 | per-tool reachable-region constraints | MISSING | MISSING | no native field in either fork; `excluded_regions`/`nonprefered_regions` are per-plate, not per-tool. Declare unsupported in capabilities |
| 13 | deterministic seed | SUPPORTED (measured) | SUPPORTED (measured) | §3 |
| 14 | accuracy | SUPPORTED | SUPPORTED | `ArrangeParams.accuracy` (0..1) both forks |
| 15 | bounded time budget / cancellation | SUPPORTED | SUPPORTED | `stopcondition` predicate; `--time-budget-ms 1` → `termination: "time_budget"`, all 12 ids still accounted for, both engines |
| 16 | progress | SUPPORTED | SUPPORTED | `progressind` callback (overridden to stderr; default prints to stdout — must override) |
| 17 | typed unplaced reasons | **DEGRADED** | **DEGRADED** | kernel signals unplaced only via `bed_idx == UNARRANGED`; `no_fit` vs `time_budget` is all the adapter can type. Finer reasons (containment violation, constraint conflict) are not available |
| 18 | cross-plate allocation | out of scope | out of scope | native multi-bin/virtual-bed machinery exists (`bed_idx`, `locked_plate`, `is_virt_object`); not exercised. Per the issue, milestone 1 declares it unsupported |

## 5. Dependency / coupling report

- **Dependencies**: none added. NLopt 2.10.1 was already a hard dependency (pinned libnest2d `CMakeLists.txt` unconditionally links `NLopt::nlopt` with `LIBNEST2D_OPTIMIZER_nlopt`; the top-level `find_package(NLopt QUIET)` warning understates this).
- **GUI/application-layer duplication required**: none compiled in. Behavior duplication is another matter — the desktop caller wraps the kernel in helpers the adapter would need to port for GUI-faithful results: inflation derivation + bed shrink (`update_arrange_params`, `update_selected_items_inflation`, `update_unselected_items_inflation`, `get_shrink_bedpts`, ~90 lines, Arrange.cpp:85-167), convex-hull computation per instance (Model.cpp:4129-4153), extruder-id population, and result application (`apply_arrange_result`, Model.cpp:4189). Call it ≤200 lines of straightforward ports — no orchestration machinery (no `ArrangeJob`, `PartPlate`, thumbnails, OpenGL).
- **Undocumented kernel preconditions found by the spike** (production adapter must encode): `bed_idx = 0` on items AND exclusions or they are silently skipped; Orca `extrude_ids` non-empty or seq-print UB; override `progressind` or it writes to stdout; transform = `R(yaw)·p + T` about the footprint origin; containment is a soft objective when fixed items exist — post-validate.
- **Engine drift found**: non-rect bed (polygon vs circle), per-item rotations (honored vs ignored), spacing gap (1.0 vs 0.0 mm at identical input), sequential-field spelling, starting-point alignment (CENTER-ish vs TOP_RIGHT/BOTTOM_LEFT per `fill_config`). `layout capabilities` MUST report these per engine; a single capability set for both would be false.

## 6. Example placements (disposable format, bambu engine, seed 42)

`five_mixed_sizes.json` (5 rectangles, 250×210 bed, spacing 2.0) →

```json
{"engine":"bambu","termination":"completed","elapsed_ms":4,
 "placements":[
  {"id":"r80x40","bed_id":0,"x_mm":100.49999,"y_mm":65.00001,"yaw_deg":0.0},
  {"id":"r60x60","bed_id":0,"x_mm":120.49999,"y_mm":105.99999,"yaw_deg":0.0},
  {"id":"r50x20","bed_id":0,"x_mm":130.49998,"y_mm":44.00003,"yaw_deg":0.0},
  {"id":"r30x70","bed_id":0,"x_mm":69.50001,"y_mm":95.99999,"yaw_deg":0.0},
  {"id":"r20x20","bed_id":0,"x_mm":79.50001,"y_mm":75.00001,"yaw_deg":0.0}],
 "unplaced":[],"locked":[],"warnings":[],
 "output_sha256":"a5c3bf4182d36acfac28456368a574119a48787a284283b779c2fa1ee38c1fb6"}
```

`locked_center_item.json` (locked 60×60 at bed center + three free 40×40) → locked preserved at input transform; free items SAT-verified non-overlapping — **but f1/f2 land at x 233.7..273.7, past the 250 mm bed edge** (see §4 row 6: containment is soft with fixed items; a faithful adapter must post-validate and demote).

Full outputs: `spike/layout/out/{bambu,orca}/*.json` (in the worktree, not committed).

## 7. Verdict

**PROCEED** to a production `layout plan` adapter — on the bambu engine first — with these contract-negotiation items surfaced to OhMyHelio (Bucket 17 schema freeze), none papered over:

1. **Containment is soft with fixed items** (both engines) — adapter must post-validate every candidate against the bed polygon and demote/repair violations. This is the strongest argument for OhMyHelio keeping independent validation (D-024) rather than trusting kernel output.
2. **Convex-only footprints**: kernel drops holes; GUI convexifies upstream. Contract should declare convex footprints (adapter convexifies + reports degradation) or accept the gap as `unsupported`.
3. **Per-tool reach**: MISSING in both forks — declare unsupported in capabilities from day one.
4. **Engine drift is real** (non-rect beds, rotations, spacing semantics, seq field spelling, seq UB): capabilities must be per-engine; the orca engine needs either small adapter-side compensations (e.g. adapter-enforced rotation snapping + post-validation) or a reduced capability set. Cross-checking a candidate against both engines is cheap and would make drift observable in CI.
5. **Typed no-fit reasons** are limited to `no_fit`/`time_budget`; the contract should not promise more from this backend.

Proceed criterion from the issue is met: a faithful adapter is possible without copying substantial desktop orchestration (≤200 lines of helper ports, zero GUI linkage), and every silently-lost constraint found above is now named rather than hidden.
