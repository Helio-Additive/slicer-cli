# libslic3r-rs parity rounds R472–R808

This file continues [`PARITY_STATUS.md`](../../PARITY_STATUS.md), which stops at round R471.
It is a round log for R472–R808 of the Rust port of BambuStudio's libslic3r
(`crates/libslic3r-rs`), **reconstructed from commit messages** (commit range `29e21e3..2439d16`,
329 commits, 2026-08-03 to 2026-09-05). Nothing here was re-measured: every number is copied from
the commit that reported it, and where two commits disagree both are kept and the disagreement is
listed under [Inconsistencies in the record](#inconsistencies-in-the-record).

Each round compares the G-code of the Rust engine with the G-code of the C++ engine ("native",
"C++", "bambu" in the commits) for the same job, finds one divergence, and ports or fixes it. Many
rounds change no behaviour at all; they are kept in the table because a refuted theory is as useful
to the next reader as a fix.

Round numbers with no commit in the range: R739, R741, R745–R752, R755, R766, R781, R782, R789,
R793, R795, R802 and R804 (only `R804a` exists). Some of them are cited by later commits, so the
work happened but was not committed under its own tag.

## Contents

- [Fixtures](#fixtures)
- [Instruments](#instruments)
- [Gate conventions](#gate-conventions)
- [Where it stands at R808](#where-it-stands-at-r808)
- [Score trajectory](#score-trajectory)
- [Per-round table](#per-round-table)
- [Open threads at R808](#open-threads-at-r808)
- [Gates not default at R808](#gates-not-default-at-r808)
- [Method rules and traps](#method-rules-and-traps)
- [Inconsistencies in the record](#inconsistencies-in-the-record)

## Fixtures

| Name in the commits | Job | Notes |
|-|-|-|
| Benchy, benchy classic, `benchy-016` | `tests/configs/benchy-016.jsonnet` | Single material, classic perimeters. |
| Benchy arachne, `benchy-arachne`, "arachne" | `tests/configs/benchy-016-arachne.jsonnet` | Same model with Arachne walls. Added in R705 as a cheap Arachne reproducer (2.79 s against 19.05 s for Majora); tracked as a fourth baseline from R708. |
| Majora | the multi-material painted 3MF, sliced through `tests/configs/nu3mf.jsonnet` | 8 filaments, 656 layers, prime tower, about 2,720 tool changes, Arachne walls, gyroid sparse infill, negative-part volumes. The hard fixture. |
| cube, painted cube | small painted cube | Guard fixture; named the Tier-1 gate fixture for segmentation in R765. |

The C++ engine is not byte-reproducible from run to run (R487, R547: four runs, four hashes; R736:
five Majora hashes in five runs). Scores are taken against one pinned reference file per fixture.
The Majora reference was regenerated at R494 and the references are dated 2026-08-20 from R768; the
Benchy classic reference was regenerated at R801, so Benchy deltas across R801 include native run
noise. R801 traces a likely source of the native noise to a use-after-free in the floating
vertical shell detection.

## Instruments

The headline metric changed four times in this span. Scores are not comparable across these
boundaries.

| From | Instrument | What it measures | Status |
|-|-|-|-|
| before R472 | `scripts/semantic_compare.py` | Physical equivalence: total filament, per-layer and per-feature material ratio (rust/C++), layer count, rasterised silhouette IoU and wall-line IoU. Five pass/fail checks. | Headline until R577. Both fixtures pass all five checks from R518 (Majora) and R507 (Benchy), which is why a line-level metric was introduced. The silhouette check was rescoped to the object only in R507 and its raster brush fixed in R518. |
| R578 | `scripts/line_compare.py` | Line-level: layer, then feature block, then extrude islands matched by anchor geometry, then a windowed structural walk. | Superseded at R592; its matcher mispaired (Benchy 40.39% when 75.82% of lines were byte-identical). |
| R592 | `scripts/line_align.py` | Line-level, order-respecting: `difflib` LCS over structural keys (numbers masked) per (layer, feature) block; a pair counts when every number agrees within 1e-4 relative. | Lower bound. Keys are degenerate (`G1 X# Y# E#`), so it partly measures its own alignment. Headline for one round. |
| R593 | `scripts/line_parity.py` | "Content": within each (layer, feature) group, quantise every number to 1e-3 mm and take the multiset intersection. Order-independent, reported as matched lines and % of Rust body lines. | Headline R593–R651. An upper bound: blind to ordering inside a group (found in R651). Still reported as "content" afterwards. |
| R652 | `scripts/seq_parity.py` | "In-order": same grouping and quantisation as `line_parity.py`, but a longest-matching-block walk, so a line counts only if it matches and is reachable in order. `in_order <= matched`; the difference is the "ordering loss". | Headline from R652 to R808. Uses `difflib` blocks, not a strict LCS, so it is a lower bound (R708: artifact 0 lines at classic density, 1.22% at arachne density). |

Supporting scripts added in this span: `scripts/fill_geom.py` (R594) and `scripts/fill_geometry.py`
(R711) for fill angle, phase, order and shared vertices; `scripts/seq_parity_control.py` and
`scripts/order_loss_kinds.py` (R708); `scripts/island_order.py` (R709);
`scripts/audit-config-defaults.py` (R707); `scripts/inject-arachne-probes.py` (R548), which injects
matching probes into the C++ reference for a measurement and is reverted afterwards;
`scripts/ab_template.sh` for gate A/B runs.

How scores are quoted:

- Until R651 the percentage is the `line_parity.py` content rate. From R652 rounds quote absolute
  in-order matched lines first. The absolute count is the decider: a rate can rise because the
  denominator shrank (R599, R626, R631, R647, R754).
- From R807 the headline is a Majora in-order **percentage**. The commits do not state its
  denominator, and the percentages quoted between R788 and R807 are not all on one basis (see
  Inconsistencies).

## Gate conventions

Almost every behaviour change lands behind an environment switch so that the previous arm can be
reproduced byte for byte. Since R693 there are three helpers:

- `faithful_gate`: default **ON**; `NAME=0` restores the old arm. In the table: `NAME (ON)`.
- `opt_in_gate`: default **OFF**; `NAME=1` turns it on. In the table: `NAME (opt-in)` or
  `NAME (parked)` when the commit says it is parked on a measured regression.
- `probe_enabled`: presence-only, for diagnostic probes. `NAME=0` turns a probe **on**. Before R693
  seven behaviour gates used this helper by mistake, and `FLOW_F32` and `L0_EFC` were found the same
  way later (R760, R764).

Probes (`probes: ...` in the table) print censuses and never change output. Their C++ twins are
temporary edits to the reference checkout and are not committed.

Kind tags in the table: `[fix]` behaviour change on the default path; `[port]` new faithful port on
the default path; `[gated]` landed behind a gate that is not default; `[diag]` diagnosis only, no
behaviour change; `[falsified]` a theory was tested and refuted; `[infra]` instruments, tests,
tooling.

## Where it stands at R808

Latest value reported for each metric, with the round that reported it.

| Fixture | Metric | Value | Round |
|-|-|-|-|
| Majora | in-order matched lines (`seq_parity.py`) | 1,787,698, quoted as 66.06% (from 1,718,733, 63.25%) | R808 |
| Majora | prime tower stratum, in-order | 83.0% (from 66.8%); tower extrusion lines content-matched 62,311 of 67,449 (from 0) | R808 |
| Majora | ordering loss (content-matched but out of order) | 295,093 lines (from 446,985) | R807 |
| Majora | WallToolPaths inputs byte-exact | 19,998 of 41,145 calls | R805b |
| Majora | post-MMS region pieces byte-exact | 3,248 of 3,375; post-MMS lslices exact on 591 of 656 layers | R806b, R806c |
| Majora | floating vertical shell stratum, in-order | 33.0% (from 12.6%) | R801 |
| Benchy classic | in-order matched lines | 121,714 (from 119,068); byte-identical in R808. No percentage is given after R754 (71.13% at 108,099 with a gate on); content was 86.3% at R780. | R807 |
| Benchy arachne | in-order matched lines | 138,771 (from 134,307); byte-identical in R808. Last percentage: 63.5% of a 211,590-line body at 134,299. | R807, R803e |
| Benchy arachne | surfaces with byte-identical wall output | 706 of 720 | R803 |
| all | test suite | 9/9 (one native-hash flake in R807 that passes on rerun) | R807, R808 |
| Benchy classic | slice time against C++ | 0.742x (1.418 s; C++ about 1.96 s) | R740 |
| Majora | slice time against C++ | 1.061x (17.505 s against 16.505 s) | R736 |

No slice-time ratio is reported after R743. Later rounds that state a cost: `LSLICES_POST_MMS`
adds 0.3 s on Majora (R806c).

## Score trajectory

Only rounds that report a headline number. A dash means the round gave no figure for that fixture.

### R472–R577: `semantic_compare.py` (material ratios rust/C++; 1.000 is parity)

| Round | Majora | Benchy classic |
|-|-|-|
| R472 | sparse infill 0.586 -> 1.069; object material 0.9928 -> 1.0416 (the old value was a cancelling pair) | object-only 1.0007 |
| R473 | sparse infill 1.069 -> 0.984 | - |
| R474 | floating vertical shell 1.291 -> 0.854; object-only 1.0221 -> 0.9790 | - |
| R476 | silhouette 83.81% -> 94.36% | - |
| R489 | internal solid 0.922 -> 1.001; FVS 0.856 -> 1.020; object-only 0.9798 -> 0.9960 | - |
| R492 | bridge E/mm 0.04924 -> 0.05339 (C++ 0.05433) | bridge 0.975 -> 0.991 |
| R506 | prime tower 1.045 -> 0.9947; object material 1.0193 -> 0.9959; per-layer deviation 12.42% -> 4.47% | - |
| R507 | silhouette rescoped to the object: 94.08% -> 96.67% | all five checks pass, silhouette 99.99% |
| R518 | silhouette 96.67% -> 99.37% (metric brush fix); all five checks pass | silhouette 99.99% |
| R549 | object material 0.9959 -> 0.9973; wall-line IoU 95.19% -> 95.28% | - |
| R590 | material 0.9974, silhouette 99.54% | material 1.0015, silhouette 99.99% |

### R578–R651: line-level content metrics

| Round | Instrument | Majora | Benchy classic |
|-|-|-|-|
| R578 | `line_compare.py` | essentially identical >= 2.20% | >= 36.66% |
| R579 | `line_compare.py`, fixed matcher | 4.41% | 40.38% |
| R592 | `line_align.py` | 8.19% | 52.51% |
| R593 | `line_parity.py` | 16.25% (409,270 / 2,518,598) | 74.99% (115,887 / 154,539) |
| R601 | `line_parity.py` | 409,318 -> 630,879 (25.06%); prime tower 26.9% -> 61.0% | - |
| R602 | | 646,455 (25.54%) | - |
| R603 | | 648,759 (25.60%) | - |
| R611 | | 656,934 (25.78%) | - |
| R615 | | 660,333 (25.88%) | - |
| R625 | | 668,158 (26.02%) | - |
| R627 | | 670,866 (26.15%) | - |
| R630 | | 670,865 (26.15%) | 75.03% (115,901 / 154,471) |
| R638 | | 673,583 (26.26%) | 115,900 |
| R641 | | 680,143 (26.45%) | - |
| R643 | | 692,188 (26.92%) | - |
| R645 | | 697,630 (27.04%) | - |
| R646 | | 700,352 (27.15%) | - |
| R647 | | 700,354 (28.00%; the body shrank by 78,621 lines) | 115,961 |
| R648 | | 705,796 (28.16%) | - |
| R649 | | 708,517 (28.24%) | - |
| R650 | | 711,537 (28.32%) | - |

### R652–R808: `seq_parity.py` in-order matched lines

| Round | Majora | Benchy classic | Benchy arachne |
|-|-|-|-|
| R652 | 468,570 (18.65%; content 28.32%) | 98,259 (63.61%; content 75.07%) | - |
| R678 | 469,489 (18.70%; content falls to 28.24%) | - | - |
| R704 | 469,628 (18.59%; body grew 14,732 lines) | - | - |
| R705 | 18.59% (content 28.13%) | 63.61% | 15.27% (content 23.13%), first measurement |
| R706 | - | - | 32,216 -> 32,977 (16.24%) |
| R722 | - | 98,405 | - |
| R725 | 469,005 (content +10,911) | 101,558 | 35,654 |
| R728 | 471,202 | 102,645 | 35,698 |
| R737 | 471,204 | - | - |
| R738 | 473,764 | 105,828 | 37,349 |
| R743 | 477,957 | 108,800 | 39,708 |
| R753 | 477,957 | 108,900 | 39,727 |
| R757 | 477,860 | 110,517 | 39,860 |
| R758b | 482,929 | 113,853 | 50,695 |
| R760 | 481,879 | 114,494 | 50,688 |
| R764 | 481,906 | 114,883 | 50,708 |
| R765 | 500,912 | - | - |
| R768 | 651,235 | - | - |
| R769 | 651,622 | - | - |
| R770 | 651,928 | - | - |
| R774 | 652,008 | unchanged | 51,478 |
| R775 | 1,052,148 (content 32.1% -> 48.8%) | 114,906 (content 85.5%) | 94,488 (content 33.4% -> 61.0%) |
| R780 | unchanged | 117,603 (content 86.3%) | 95,969 (content 61.3%) |
| R788 | 1,056,793 (content 49.00%) | - | unchanged |
| R792 | 1,057,745 | - | - |
| R801 | 1,116,613 (content 51.84%) | 118,951 (regenerated reference) | 97,790 |
| R803 | 1,124,799 | 119,068 | 114,757 (43.2% -> 50.6%) |
| R803b | 1,128,965 | unchanged | 115,792 |
| R803c | 1,217,995 (44.85%) | - | 124,025 (56.7%) |
| R803d | - | - | 133,818 (60.3%) |
| R803e | - | - | 134,299 (63.5%) |
| R804a | 1,276,793 -> 1,277,236 | - | - |
| R805 | 1,473,706 (53.9%) | - | - |
| R805b | 1,475,536 | - | - |
| R807 | 1,505,160 -> 1,718,733 (55.10% -> 63.25%) | 121,714 | 134,307 -> 138,771 |
| R808 | 1,787,698 (66.06%) | byte-identical | byte-identical |

The largest single steps: R775 (+400,140 on Majora, +43,010 on Benchy arachne: exact offset for the
Arachne inset and truncating scaled flow widths), R807 (1,505,160 -> 1,718,733: emission order, wipe, lift and
first-layer tool order), R805 (1,277,236 -> 1,473,706: main volume sliced through the exact f32 volume transform),
R768 (+150,323: top/bottom painted-surface propagation), R803c (+89,030: exact grow of lower slices
for overhang degrees) and R808 (1,718,733 -> 1,787,698: prime tower writer).

### Slice time against C++ (ratio rust/C++; below 1 means Rust is faster)

| Round | Benchy classic | Majora | Note |
|-|-|-|-|
| R484 | 1.21x | 1.22x | Re-measured; older notes had said 1.5x and 3.0x. |
| R520 | 1.20x | 1.15x | |
| R522 | - | 1.10x | Seam placer parallel sites. |
| R563 | - | 1.448x | Regression noticed. |
| R565 | - | 1.033x | Cause was `getenv` in hot loops from gates and probes; replaced by an environment snapshot. |
| R590 | - | 1.011x | Fixture not named in the commit. |
| R726 | 1.39x | - | Stated as the current figure. |
| R735 | 0.90x | 0.99x (retracted) | Seam occlusion raycast made parallel again. |
| R736 | 0.711x | 1.061x | Re-taken on a quieter host. |
| R737 | 0.638x | - | |
| R738 | 0.907x | - | After shipping `CLIPPER_UNION_GEO`, which costs time. |
| R740 | 0.742x | - | Last ratio reported. |

### Test suite status as reported

Eight guard tests green (R489–R615); "31 guard tests" (R616–R622); eight guard suites plus the
wall-gap geometry tests (from R623); "eight suites, seven green" with
`multi_material_integration` at 25/26, described as pre-existing (R647–R756); 9/9 from R768, with
one 8/9 at R769 attributed to native nondeterminism. No commit states when the 25/26 failure was
fixed.

## Per-round table

One row per commit. Sub-rounds (`R803b`, `R806c`) have their own rows. The Gates column lists
switches introduced or flipped in that round with the state the commit gives them; see
[Gate conventions](#gate-conventions). The Score column is `-` when the round moved no fixture score.

| Round | Date | Commit | Area | Summary | Gates | Score effect |
|-|-|-|-|-|-|-|
| R472 | 2026-08-03 | 8b7ae4d | fill | [fix] Faithful Fill::connect_infill (FillBase.cpp:1501-1733) made default for non-monotonic fillers, replacing the endpoint-merge stub. | FILL_CONNECT_FAITHFUL (ON) | Majora sparse E-rat 0.586 -> 1.069; object material 0.9928 -> 1.0416 |
| R473 | 2026-08-03 | 1aa9e58 | fill | [port] Fill::fill_surface region shrink (FillBase.cpp:97) applied to Gyroid; its fill region was half a spacing too wide. | GYROID_FILL_SHRINK (ON); probes: FILL_CONNECT_WHERE | Majora sparse E-rat 1.069 -> 0.984; object material 1.0416 -> 1.0331 |
| R474 | 2026-08-03 | 5f301dd | arachne, fill | [port] fill_no_overlap now produced on the Arachne path (PerimeterGenerator.cpp:1460); it was empty, widening three fillers' regions. | ARACHNE_FILL_NO_OVERLAP, FVS_NO_OVERLAP_FAITHFUL (default not stated); probes: FVS_DEBUG | Majora FVS 1.291 -> 0.854; int solid 1.154 -> 0.922; object material -> 1.0101 |
| R475 | 2026-08-03 | 081d960 | wipe tower | [fix] WipeTower::set_extruder widths wired (inert on Majora); tower block resets the writer's last role. Mislabelling claim retracted R476. | WT_NOZZLE_CHANGE_WIDTH, TOWER_ROLE_RESET (default not stated) | Majora tower 1.0456 -> 1.0449; object-only 0.9790 -> 0.9796 |
| R476 | 2026-08-03 | 749128f | wipe tower | [fix] Travel to the tower start before the first purge stroke (GCode.cpp:687-695); each block had drawn a stroke across the plate. | TOWER_TRAVEL_TO_START (default not stated) | Majora silhouette 83.81% -> 94.36%; tower len-rat 1.113 -> 1.065 |
| R477 | 2026-08-03 | 2e5fb3d | wipe tower | [fix] Tower transform strips X/Y instead of echoing an untransformed tower-local line (GCode.cpp:1120-1132). | TOWER_XFORM_NO_RAW_ECHO (default not stated) | Majora tower len-rat 1.065 -> 1.045; tower E/mm 0.05331 -> 0.05433 |
| R478 | 2026-08-03 | f7d7833 | wipe tower | [fix] Real tower height passed into plan_tower (WipeTower.cpp:2907); inert on Majora. Footprint and extra_spacing already match C++. | probes: WT_PLAN | - |
| R479 | 2026-08-03 | 2aa69de | wipe tower | [diag] C++ calls WipeTower::generate_new(); the port follows the dead generate(). Porting the block-based generator is what remains. | - | Majora tower E: z178-197 0.558, z0-178 1.053, all 1.045 |
| R480 | 2026-08-03 | 884786f | config, fill | [fix] detect_floating_vertical_shell was never deserialized, so floating detection was off; parity effect negligible. | - | Majora FVS 0.854 -> 0.856; benchy FVS 0.990 -> 0.991 |
| R481 | 2026-08-03 | 10afa25 | solid/sparse split | [diag] FVS deficit is fewer islands, not loops per island or narrow-solid classification; blanket config-key fallback is a known negative. | - | - |
| R482 | 2026-08-03 | c5d0360 | solid/sparse split | [diag] Solid deficit is island count at matching island size; scattered-drop filter (PrintObject.cpp:2018-2054) named prime suspect. | probes: VSHELL_DEBUG | - |
| R483 | 2026-08-03 | 310d462 | solid/sparse split | [falsified] The vertical-shell scattered-drop filter is not the cause of the solid deficit; disabling it does not overshoot. Kept on. | VSHELL_DROP_FILTER (ON) | - |
| R484 | 2026-08-03 | ec7d17f | perf | [infra] Slice time re-measured (recorded figures were stale); caching the gate env lookup measured no change and was reverted. | - | Majora 19.14s vs C++ 15.73s = 1.22x; benchy 2.38s vs 1.96s = 1.21x |
| R485 | 2026-08-03 | 47b8256 | solid/sparse split | [diag] Shell machinery reads faithful and sparse island counts match; only the solid/sparse split diverges, worst on FVS. | - | - |
| R486 | 2026-08-03 | 61f4ab9 | probes/diagnosis | [diag] Earlier rounds read a different C++ tree from the reference build; conclusions re-verified. discover_shell_for_perimeters ruled out. | probes: VSHELL_TERMS | - |
| R487 | 2026-08-03 | 2a0a44f | instruments | [diag] C++ reference is not byte-reproducible, but semantic metrics have a ~0.01% noise floor; diff(internal, holes) is 0.87 of C++. | - | C++ vs C++: silhouette 100.00%, object material 1.0000 |
| R488 | 2026-08-03 | 6b5ba90 | MMS | [gated] One extra print region (9 vs 8): C++ get_create_region (PrintApply.cpp:1004-1016) deduplicates. Dedup left off; it regresses walls. | PAINTED_REGION_DEDUP (opt-in) | - |
| R489 | 2026-08-03 | 0b53bf0 | MMS | [fix] Painted-region dedup plus region-to-filament lookup in MM segmentation (PrintApply.cpp:1071); closes the solid deficit. | PAINTED_REGION_DEDUP (ON, was opt-in) | Majora int solid 0.922 -> 1.001; FVS 0.856 -> 1.020; object-only -> 0.9960 |
| R490 | 2026-08-03 | d950122 | bridges | [diag] Bridge flow: thick/thin branch and FlowRole match C++; LINE_WIDTH-tag width lead recorded (retracted R491). | - | - |
| R491 | 2026-08-03 | 88cdcc7 | bridges | [falsified] LINE_WIDTH tag is not the flow width; C++ bridge E/mm is a continuum (per-path variable widths), the port's is bimodal. | - | - |
| R492 | 2026-08-03 | 25bc66e | bridges | [fix] with_spacing flow recompute now applied to bridges, as Fill::fill_surface_extrusion does (FillBase.cpp:140-149). | BRIDGE_WITH_SPACING (ON) | Majora bridge E/mm 0.04924 -> 0.05339 (C++ 0.05433); object-only -> 0.9969 |
| R493 | 2026-08-03 | b0c23e8 | wipe tower | [gated] finish_layer_new sparse empty grid ported (WipeTower.cpp:3585-3644); opt-in because alone it overshoots the tower. | TOWER_SPARSE_GRID (opt-in) | - |
| R494 | 2026-08-03 | b6ae0cc | wipe tower | [diag] Six tower Y-placement mechanisms eliminated; single block on every Majora layer. The -2.0 mm body shift is retracted in R495. | - | - |
| R495 | 2026-08-03 | cc850c6 | wipe tower | [fix] Per-toolchange filament start/end gcode emitted (GCode.cpp:1035-1053); needed two config keys and same-line template directives. | FILAMENT_START_END_GCODE (ON) | Majora blocks 2,722/2,721 vs C++ 2,724; material-inert |
| R496 | 2026-08-03 | 296d098 | wipe tower | [diag] Writer-only tower measurement confirms the length ratio; C++ splits fill between finish_layer_new and finish_block. | TOWER_SPARSE_GRID (opt-in); probes: WTSUM | Majora tower writer-only 1.045 |
| R497 | 2026-08-03 | fcfdc8c | wipe tower | [gated] finish_block ported (WipeTower.cpp:3733); tower error is excess fill strokes versus missing extruded connectors. | TOWER_SPARSE_GRID (opt-in) | - |
| R498 | 2026-08-03 | 23e49ac | wipe tower | [gated] Extruded purge connector ported (WipeTower.cpp:4145-4149); prime_tower_skip_points found unplumbed. | TOWER_WIPE_CONNECTOR (opt-in) | - |
| R499 | 2026-08-03 | a1d40d5 | wipe tower, config | [gated] prime_tower_skip_points plumbed; toolchange ironing block ported (WipeTower.cpp:4079-4116). R498's arithmetic retracted. | TOWER_WIPE_CONNECTOR (opt-in) | - |
| R500 | 2026-08-03 | 3ac8c81 | wipe tower | [gated] Tower fill gate split into three knobs and each sized: box inert, grid overshoots 2x, effects not additive. | TOWER_FILL_BOX, TOWER_FILL_RECT, TOWER_FILL_GRID (opt-in) | - |
| R501 | 2026-08-03 | 9350990 | wipe tower | [diag] Tower purge and fill are a cancelling pair; finish_block_solid is never called for Majora. | - | Majora tower purge 0.835, fill 6.16, total 1.058 (tcr-scoped) |
| R502 | 2026-08-03 | acf8e1c | wipe tower | [diag] Emitter-level tower split; the wall is right, ramming and nozzle_change emit nothing on Majora. | - | Majora tower purge 0.908, wall 1.036, fill 4.50 |
| R503 | 2026-08-03 | 6cd3cfe | wipe tower | [gated] finish_layer_new rarely fills (WipeTower.cpp:4746); R497's layer condition reversed. Fill-box-depth theory retracted R504. | TOWER_FILL_ONLY_TC (opt-in) | - |
| R504 | 2026-08-03 | a151c0b | wipe tower | [falsified] The tower fill box formula already equals C++'s; no per-layer allocation to port with a single block. | - | - |
| R505 | 2026-08-03 | a65a450 | wipe tower | [falsified] Tower fill call count already matches C++; with the knobs on the purge is the whole remaining gap. | probes: WTFILLCNT | Majora tower (knobs on) purge 0.908, wall 1.036, fill 0.806 |
| R506 | 2026-08-03 | 732da5d | wipe tower | [fix] Cleaning box is wipe_depth - nozzle_change_depth (WipeTower.cpp:3271), one more purge stroke per toolchange; tower set made default. | TOWER_CLEANING_BOX (ON); SPARSE_GRID, FILL_*, WIPE_CONNECTOR (ON, was opt-in) | Majora tower 1.045 -> 0.9947; object material 1.0193 -> 0.9959 |
| R507 | 2026-08-03 | d47c501 | instruments | [infra] semantic_compare silhouette check rescoped to the object; the prime tower outline is reported separately. | - | Majora silhouette 94.08% -> 96.67% (metric change) |
| R508 | 2026-08-03 | e2bc2c5 | wipe tower | [falsified] 64-layer band emits tower wall only (plan depth 0); filling no-toolchange layers and depth propagation both ruled out. | - | - |
| R509 | 2026-08-03 | a015ac9 | wipe tower, config | [gated] timelapse_type was never deserialized; C++ forces plan depth to tower depth under it (WipeTower.cpp:1742). Regresses alone. | TOWER_TIMELAPSE_DEPTH (opt-in) | - |
| R510 | 2026-08-03 | 2125380 | wipe tower | [gated] Plan depth split into depth and alloc_depth as in C++; no-op by default, timelapse path still loses to the default. | TOWER_TIMELAPSE_DEPTH (opt-in) | - |
| R511 | 2026-08-03 | dcf5ee6 | fill | [diag] Top surface excess is a region problem, not a fill problem: where top regions agree the length matches C++. | - | Majora top 1.1746; 0.9927 on agreeing layers |
| R512 | 2026-08-03 | f74c8e4 | slicing/mesh | [diag] Top excess shows as scattered slivers; the opening_ex collapse offset is right, region slices already carry tiny fragments. | - | - |
| R513 | 2026-08-03 | d7eac82 | slicing/mesh | [diag] Region slices are over-fragmented: same total area as C++ but 55 surfaces vs 30 at one layer. | - | - |
| R514 | 2026-08-03 | f469632 | MMS | [falsified] Slivers all sit in the multi-material remainder (region 0); culling them does not move top surface. Reverted. | probes: TSDBG, TSDBG_FULL | - |
| R515 | 2026-08-03 | a6bb532 | 3MF load | [diag] negative_part volumes are dropped by the 3MF path (C++ PrintObjectSlice.cpp:110/403); real gap, near-inert for the metrics. | probes: LSDBG | - |
| R516 | 2026-08-03 | 5069ff2 | instruments | [diag] Silhouette gap reported as a real one-sided boundary rim, not an arc-chord artefact; overturned by R518. | - | - |
| R517 | 2026-08-03 | 5fc4826 | instruments | [diag] Wall position and width exonerated for the silhouette gap; the excess is in the raw raster, not the closing. | - | - |
| R518 | 2026-08-03 | 24ee224 | instruments | [infra] Silhouette failure was a brush-radius cliff in semantic_compare's rasteriser; brush now tests real distance. | - | Majora silhouette 96.67% -> 99.37%; all five checks pass |
| R519 | 2026-08-03 | 121ebf8 | fill | [diag] Bottom surface gap is one first-layer feature, 13 E of the print, with matching E/mm; not pursued. | - | Majora bottom surface E 106.11 vs 118.90 (0.892) |
| R520 | 2026-08-03 | b3537e6 | perf | [diag] Profiled: the remaining time gap is serial gcode export; C++ uses a parallel pipeline over layers (GCode.cpp:3396-3400). | - | Majora 18.3s vs C++ 15.9s (1.15x); benchy 2.32s vs 1.93s (1.20x) |
| R521 | 2026-08-03 | ad0b132 | perf, seam | [port] Seam candidate visibility (SeamPlacer.cpp:955) parallelised; gather_seam_candidates (:933) measured inert and left serial. | - | Majora export generate 5.02 -> 4.13s |
| R522 | 2026-08-03 | 537c670 | perf, seam | [port] Seam overhangs/layer embedding (SeamPlacer.cpp:966) parallelised over layer ranges seeded from the previous layer. | - | Majora export generate 4.13 -> 3.73s; wall 17.47s (1.10x vs C++) |
| R523 | 2026-08-03 | 05afe9e | perf, seam | [fix] Seam KD-tree cached per layer as in C++ (SeamPlacer.cpp:944-945) instead of rebuilt at each query. | probes: KDCOUNT | Majora export generate 3.73 -> 3.02s; export total 5.11 -> 4.53s |
| R524 | 2026-08-03 | ba2a49c | perf, seam | [diag] SeamPlacer exhausted; pick_seam_point (:1430) dropped as too small. Remaining lever is the stage-overlap export pipeline. | - | - |
| R525 | 2026-08-03 | 99fb122 | perf | [diag] Export is three whole-print passes, not streaming; a layer-parallel pipeline is a multi-round refactor worth about 1.5s. Deferred. | - | - |
| R526 | 2026-08-04 | a33a844 | housekeeping | [infra] main.cpp correspondence doc re-verified; new file-layout correspondence doc mapping C++ translation units to Rust files. | - | - |
| R527 | 2026-08-04 | 5c65c03 | config | [diag] The multi-nozzle trio (main.cpp:229, :291) is dead code for all three single-nozzle fixtures; needs a dual-nozzle fixture. | - | - |
| R528 | 2026-08-04 | 556f059 | gcode emission | [port] Stdout event protocol ported (main.cpp:55-137); Benchy's one live warning is not yet reproducible, two emitters left unwired. | - | - |
| R529 | 2026-08-04 | e38041e | support | [diag] is_support_necessary declined: needs TreeSupport::detect_overhangs (653 lines) for one warning line, zero gcode change | - | - |
| R530 | 2026-08-04 | d4e8298 | wipe tower | [port] Reserved tags part 1: the four tool_change_new tags (WipeTower.cpp:3270-3341) emitted from tool_change | - | Majora CP TOOLCHANGE tags 2721 (C++ 2723); WIPE_TOWER 2721 (C++ 3655) |
| R531 | 2026-08-04 | fbb625e | wipe tower | [port] Reserved tags part 2: finish_layer_new WIPE_TOWER pair (WipeTower.cpp:3550, :3721); the -2 gap is an upstream tool-change count | - | Majora WIPE_TOWER_START/END 2721 -> 3377 (C++ 3655) |
| R532 | 2026-08-04 | f98aeeb | wipe tower | [port] Reserved tags part 3: CP_TOOLCHANGE_WIPE with its " CT<n> FL<0\|1>" suffixes (WipeTower.cpp:3960-3961); CT hard-zero | - | Majora CP_TOOLCHANGE_WIPE 0 -> 2721 (C++ 2723) |
| R533 | 2026-08-04 | 6ed809e | wipe tower | [port] Reserved tags complete: CP EMPTY GRID START/END in finish_layer; the C++ " layer #" comment value deliberately not emitted | - | Majora CP EMPTY GRID 0 -> 207 (C++ 209) |
| R534 | 2026-08-04 | b750980 | wipe tower | [port] Tower LAYER_HEIGHT + LINE_WIDTH tags at the head of every tower block (WipeTowerWriter ctor, WipeTower.cpp:637-642) | - | Majora tower LAYER_HEIGHT 3377 vs 3655; LINE_WIDTH 3383 vs 3661 |
| R535 | 2026-08-04 | 0de9d66 | perimeters | [diag] Sized the segmentation gap: it is a Majora-only outer-wall speed gap (subject only, no body) | - | - |
| R536 | 2026-08-04 | 2a4b1c2 | arachne | [diag] Majora wall-speed gap root cause: arachne path never grades overhang degree (PerimeterGenerator.cpp:703-707 unported) | probes: SMOOTHPROBE, OHSPLITPROBE | - |
| R537 | 2026-08-04 | 7a090ab | arachne | [port] Arachne overload of detect_overhang_degree (OverhangDetector.cpp:168-317); avg-width flow kept so E is unchanged | ARACHNE_OVERHANG_DEGREE (ON) | Majora outer-wall G1 F 17,525 -> 118,766 (C++ 148,936); IoU 94.64% -> 95.20% |
| R538 | 2026-08-04 | d5af846 | arachne | [diag] Outer wall structurally matched after R537; inner-wall G1 F residual open, four causes eliminated (smoothing, constants, corr speed) | probes: SMOOTHROLE, ARACHPROBE | Majora inner-wall G1 F 25,209 vs 53,474 (0.471) |
| R539 | 2026-08-04 | 0b7853b | arachne | [falsified] "detect_brigde_wall_arachne is unported" retracted (already implemented); points at overhang_flow (PerimeterGenerator.cpp:727) | probes: ARACHBRIDGE | - |
| R540 | 2026-08-04 | 84b9080 | arachne | [falsified] Overhang flow substitution inert on Majora (thick_bridges=0, bridge_flow=1); overhang closed as a lever; gap is variable width | - | Majora LINE_WIDTH changes whole file 119,622 vs C++ 215,199 |
| R541 | 2026-08-04 | f22ece8 | arachne | [fix] R414 "double-apply" blocker was stale; wired the ported-but-dead variable-width builder (extrusion_paths_append_zpaths) | ARACHNE_VARIABLE_WIDTH (ON); probes: ARACHWIDTH | Majora IoU 95.20% -> 95.19%; LINE_WIDTH 119,622 -> 120,983 (C++ 215,199) |
| R542 | 2026-08-04 | 1da35e5 | arachne | [diag] Arachne tree fully ported (seven beading strategies, factory faithful); gap restated as within-loop bead variation | - | Majora LINE_WIDTH per outer-wall block 0.25 vs C++ 4.21 |
| R543 | 2026-08-04 | 8974891 | arachne | [falsified] Beadings are not flat at compute or at junction creation; R542's propagation pointer refuted, suspects post-processing | probes: BEADPROBE | - |
| R544 | 2026-08-04 | 9df7940 | arachne | [falsified] None of the five WallToolPaths post-processing stages (WallToolPaths.cpp:534-542) flattens widths; simplify exonerated | probes: STAGEPROBE | - |
| R545 | 2026-08-04 | f74a7e0 | arachne | [falsified] connect_junctions exonerated; beadings are shared, not near-equal; R542/R543 elimination of propagation declared invalid | probes: CJPROBE | - |
| R546 | 2026-08-04 | 75d1361 | arachne | [diag] Beading propagation chain (SkeletalTrapezoidation.cpp:1608-1771) verified faithful; Rust-side probing exhausted | probes: PROPPROBE | - |
| R547 | 2026-08-04 | 1ff512f | arachne | [diag] C++ reference instrumented with matching probes: defect is at or above bead-count assignment; C++ is not byte-reproducible | probes: GRAPHPROBE | Majora stage-0 flat% ours 86.9% vs C++ 67.8% |
| R548 | 2026-08-04 | 97734ad | arachne | [diag] Deficit fully present after updateIsCentral on both-engine census; probe patch replaced by scripts/inject-arachne-probes.py | probes: CENTRALPROBE, ISCPROBE | Majora central/edge after updateIsCentral 3.212% vs C++ 11.199% |
| R549 | 2026-08-04 | c83b787 | arachne, config | [fix] wall_transition_angle default was radians in a degrees field (deg2rad applied twice, WallToolPaths.cpp:456); R538-R548 root cause | ARACHNE_WTP_ANGLE_DEG (ON); probes: GEOMPROBE2 | Majora material 0.9959 -> 0.9973; IoU 95.19% -> 95.28% |
| R550 | 2026-08-04 | 332b4c1 | arachne, config | [port] Arachne wall params read from object config (PerimeterGenerator.cpp:1537-1553); filter_deviation unit fixed; parity-neutral | ARACHNE_WTP_PARAMS (ON); probes: WTPPARAMS | Majora LINE_WIDTH per outer block 1.20 -> 1.19 (C++ 4.21) |
| R551 | 2026-08-04 | a7d3a51 | arachne | [falsified] Transition machinery faithful; getNearestBeading is dead code on Majora in both engines; gap relocated to ExtrusionLine count | probes: TRANSPROBE, GNBPROBE | Majora stage-0 flat% 75.2% (was 86.9%) vs C++ 67.8% |
| R552 | 2026-08-04 | f941c20 | arachne | [diag] The ~2x is in Arachne's input outline; prepared_outline preparation chain (WallToolPaths.cpp:461) and meshfix constants faithful | probes: POLYPROBE | - |
| R553 | 2026-08-04 | 0e74421 | perimeters | [diag] Surfaces entering process_arachne already differ (more, smaller, almost no holes); ours has a pre-union C++ :1511 lacks | probes: LASTPROBE | - |
| R554 | 2026-08-04 | 07fbbd9 | arachne | [fix] generate_arachne drops the pre-union to match PerimeterGenerator.cpp:1511; surface flattening shown 1:1; parity-neutral | ARACHNE_NO_PRE_UNION (ON) | Majora material 0.9973, IoU 95.28%, silhouette 99.53% unchanged |
| R555 | 2026-08-04 | 7013c51 | slicing/mesh | [diag] Clipper fill rule faithful; layer-region counts match, the split is inside each region's SurfaceCollection | probes: MPPROBE | Majora surfaces per layer-region 7.78 vs C++ 4.97 |
| R556 | 2026-08-04 | 8ec5d9d | MMS | [diag] apply_mm_segmentation_tier1 omits both C++ cleanup filters (opening, needs_merge closing; PrintObjectSlice.cpp:946-965) | - | - |
| R557 | 2026-08-04 | 7b2672d | MMS | [gated] Both MM-segmentation filters ported; surface count fixed but width metric unmoved, parity slightly worse; R551-R556 chain retracted | MMSEG_OPENING (opt-in); MMSEG_CLOSING (opt-in) | Majora with MMSEG_OPENING: IoU 95.28% -> 95.22%; material 0.9973 -> 0.9972 |
| R558 | 2026-08-04 | fbdf170 | arachne | [fix] No-overhang-split branch (PerimeterGenerator.cpp:772) wired to the dead extrusion_paths_append_line; LINE_WIDTH emitter faithful | ARACHNE_LINE_VARIABLE_WIDTH (ON) | Majora material 0.9973 -> 0.9974; IoU 95.28% -> 95.29% |
| R559 | 2026-08-04 | ab8f20b | arachne | [fix] filterOuterCentral no longer run (C++ never runs it, SkeletalTrapezoidation.hpp:135); R558's 2x-lines inference withdrawn | ARACHNE_NO_FILTER_OUTER_CENTRAL (ON) | Majora parity-neutral: material 0.9974, IoU 95.29%, Top 1.172 unchanged |
| R560 | 2026-08-04 | 932ff79 | arachne | [falsified] Arachne top-one-wall detection (PerimeterGenerator.cpp:1620-1719) is near-inert; port closed; Top-surface hypothesis refuted | probes: TOWPROBE | Majora (C++): detection survives on 4 of 16,500 surfaces |
| R561 | 2026-08-04 | 36a214b | arachne | [diag] Whole-run outline census: about half our generate() calls early-return; both engines re-enter generate() on failing surfaces | - | Majora early returns 25,400 (49.6%) vs C++ 9,200 (18.3%) |
| R562 | 2026-08-04 | 93af56f | arachne | [falsified] Failures are empty inputs, not killed by the preparation chain; R561's 1.85x outline gap was a denominator artefact | probes: AREAPROBE | Majora outline points/call on non-empty input 48.2 vs 52.5 (1.09x) |
| R563 | 2026-08-04 | ef017cf | perf | [diag] Empty-input generate() calls priced at 354 ms and closed; slice time vs C++ re-measured, the recorded ~1.12x was stale | - | Majora wall clock rust 25.50s vs cpp 17.60s = 1.448x |
| R564 | 2026-08-04 | ddce6ee | arachne, perf | [falsified] Run-length (path ordering) hypothesis refuted; tags-per-block mean is a tail statistic (p50 = 0 both engines); first profile | - | Majora tags per outer-wall block mean 1.333 vs 4.210; p50 0 vs 0 |
| R565 | 2026-08-05 | c60c44a | perf | [fix] getenv behind gates/probes was 18.51% of samples (process-global lock); env snapshot now backs faithful_gate and probe_enabled | - | Majora slice time vs C++ 1.448x -> 1.033x; process 19.53s -> 10.78s |
| R566 | 2026-08-05 | 1829a2b | arachne, perf | [diag] Post-fix profile confirms R565; width metric re-derived as a rate per extrude move; R564's varying-block split is confounded | - | Majora LINE_WIDTH tags per move 0.0368 vs 0.1003 (2.73x) |
| R567 | 2026-08-05 | 6be0694 | arachne | [diag] Per-loop junction width-change rate measured on both engines; scope must be outer wall only (all-loops reading was wrong) | - | Majora outer-wall change rate 0.0212 vs 0.0463 (2.18x) |
| R568 | 2026-08-05 | 2552a38 | arachne | [diag] Outer-wall LINE_WIDTH tags: ours fire after travels (inter-loop), C++'s after extrudes (intra-loop); "99.1% loss" retracted R569 | - | Majora tags after travel 99.5% vs 0.4%; after extrude 0.5% vs 94.7% |
| R569 | 2026-08-05 | ef35f4d | arachne | [falsified] thick_polyline_to_multi_path exonerated (VariableWidth.cpp:5-90; merge tolerance is 0.1um, not 50um); R568's loss claim wrong | probes: TPMPPROBE, EXPWPROBE | Majora builder input width changes 13,888 vs 39,330 (2.83x) |
| R570 | 2026-08-05 | aad6d8d | arachne, ordering | [falsified] Loop fragmentation and path adjacency/reordering both refuted; the output gap equals the builder input gap | - | Majora tags/run 0.1294 vs 0.3771 (2.91x); input gap 2.83x |
| R571 | 2026-08-05 | d049d06 | arachne | [falsified] We produce more width variety than C++, not less; "variation is lost" framing retired; R570 register tension was probe scope | probes: JUNCPROBE | Majora distinct widths at 6.4M junctions 28,001 vs 15,634 |
| R572 | 2026-08-05 | d3d33b8 | arachne | [diag] C++ creates about twice the outer-wall junctions per emitted run; causality not claimed; per-loop beading identity unmeasured | - | Majora outer-wall junctions 2,538,701 vs 5,549,046 (2.19x) |
| R573 | 2026-08-05 | b09b72a | arachne | [diag] Per-line census at end of generateToolpaths; "gap doubles between assembly and builder" (assembly figures retracted in R574) | probes: LINEPROBE2 | Majora change rate 1.42x at assembly, 2.83x at builder (retracted R574) |
| R574 | 2026-08-05 | c08886b | arachne | [diag] We split each assembled outer line into 1.88x more builder pieces; the gap is created there; R573's assembly numbers were artefacts | - | Majora builder calls per assembled line 6.366 vs C++ 3.388 |
| R575 | 2026-08-05 | c88834c | arachne | [falsified] Overhang/ZPath split site exonerated; R574's "we split 1.88x more" was a denominator effect, not extra splitting | probes: SPLITPROBE | - |
| R576 | 2026-08-05 | 946685f | arachne | [diag] add_toolpath_segment line-start conditions are faithful; the new-line gap comes from the `empty` and `odd` causes upstream | probes: NEWLINEPROBE | - |
| R577 | 2026-08-05 | 3952925 | arachne | [diag] C++ interleaves odd walls 2.19x finer; `empty` term is the invocation count. C++ counts later found contaminated (R580) | probes: ODDPROBE | - |
| R578 | 2026-08-05 | 483568a | instruments | [infra] New scripts/line_compare.py line-level parity metric (anchor-matched islands); first readings are lower bounds | - | Benchy essentially identical >=36.66%; Majora >=2.20% |
| R579 Thread A | 2026-08-05 | f84a18d | instruments | [infra] line_compare.py matcher fixed (mutual-nearest-neighbour) and now reports its own failure rate; R578 denominators were undercounts | - | Benchy essentially identical 40.38%; Majora 4.41% |
| R580 | 2026-08-05 | ad2d6c4 | arachne | [falsified] Invocation gap is a speculative one-wall pass C++ discards (PerimeterGenerator.cpp:1774); R572-R577 C++ counts contaminated | probes: WTPCALL, TOWPROBE | - |
| R581 | 2026-08-05 | 5425665 | arachne | [diag] Chain re-derived with the speculative pass excluded: supply near parity, width tags per line is the dominant term | - | - |
| R582 | 2026-08-05 | 3f6825d | arachne | [diag] Tags-per-line gap splits into a minor assembly term and a downstream doubling; width-blind reduction hypothesised | - | - |
| R583 | 2026-08-05 | 087e82e | arachne | [falsified] Width-blind reduction refuted; ZPath clip and post-processing exonerated, deficit exists at birth in SkeletalTrapezoidation | probes: REDPROBE | - |
| R584 | 2026-08-06 | 62e024e | arachne | [diag] Graph structure at parity for emitting edges; gap is in beading values at edge boundaries; R571's distinct-width claim retracted | - | - |
| R585 | 2026-08-06 | 126a6b5 | arachne | [diag] C++ neighbouring beadings differ more often than ours; width quantisation and beading object-sharing refuted | probes: BEADPAIR | - |
| R586 | 2026-08-06 | f1ae61d | arachne | [diag] We take the beading copy path too often; the copy-vs-interpolate decision itself is at parity, cause is upstream | probes: PROPCLASS | - |
| R587 | 2026-08-06 | 59ca91c | arachne | [diag] Upward propagation seeds about half as many nodes, via the `to->bead_count >= 0` guard; downward dispatcher at parity | probes: UPPROBE, DNPROBE | - |
| R588 | 2026-08-06 | 4a1d64a | arachne | [falsified] Centrality is a red herring (per-node and per-edge rates at parity); C++'s skeletal graph is about 25% denser | probes: CENSUS | - |
| R589 | 2026-08-06 | 69e3613 | arachne | [diag] Graph density gap is created converting Voronoi to half-edges; discretize exonerated | probes: GBUILD | - |
| R590 | 2026-08-06 | 46094a9 | arachne | [fix] collapse_small_edges snap distance was 2000, C++ uses 5 (SkeletalTrapezoidationGraph.hpp:84); graph density gap closed | ARACHNE_COLLAPSE_SNAP_5 (ON); probes: CONV | Majora line-count gap 12.9% -> 11.8%; line_compare 4.41% -> 4.42% |
| R591 | 2026-08-06 | 7e1c341 | arachne | [diag] R590 fix propagates only partially downstream; R583's 1.378x anchor withdrawn as a point estimate (not reproducible) | - | - |
| R592 | 2026-08-06 | c0bf547 | instruments | [infra] New scripts/line_align.py (LCS aligner); line_compare.py was the dominant error; Benchy walls score best, fill and bridge worst | - | Benchy 40.39% -> 52.51%; Majora 4.42% -> 8.19% (instrument) |
| R592 fixup | 2026-08-06 | cd5992c | housekeeping | [infra] Untrack a stray .pyc and ignore __pycache__ | - | - |
| R593 | 2026-08-06 | 202cfcb | instruments | [infra] New scripts/line_parity.py (alignment-free multiset per layer+feature); Benchy walls done, residual is fill path geometry | - | Benchy 74.99% (115,887/154,539); Majora 16.25% (409,270/2,518,598) |
| R594 | 2026-08-06 | 625829b | fill, bridges | [diag] scripts/fill_geom.py: fill features share no single cause; Bridge fill angle is wrong; Top surface differs only in order | - | - |
| R595 | 2026-08-06 | 312bc10 | bridges | [gated] detect_angle is never reached on Benchy (tie-break theory dead); Line::normal() sign fixed (Line.hpp:180), no gain | BRIDGE_NORMAL_CPP (opt-in); probes: BRIDGEPROBE | Gate ON: Majora matched 409,270 -> 409,266; Benchy unchanged |
| R596 | 2026-08-06 | 770cc7f | bridges | [fix] Bridge anchor expand epsilon was 1e5 too small (0.001 vs SCALED_EPSILON); both engines now take the principal-components branch | BRIDGE_ANCHOR_EPS_SCALED (ON); probes: BRIDGEIN | Majora matched 409,270 -> 409,276; Benchy unchanged |
| R597 | 2026-08-06 | f699131 | fill, bridges | [diag] Ported infill_direction (fill/mod.rs:1461) is dead code; the live fill path ignores bridge_angle and thickness_layers | - | - |
| R598 | 2026-08-06 | 5ed97e1 | fill, bridges | [falsified] thickness_layers divisor is cosmetic on these fixtures; R597's "bridge angle never used" was a prefix artefact | - | - |
| R599 | 2026-08-06 | 33c44ae | fill, bridges | [gated] bridge_angle short-circuit on the live fill path (FillBase.cpp:224-239); faithful but net negative, Internal solid regresses | FILL_BRIDGE_ANGLE (opt-in) | Gate ON: Benchy matched 115,887 -> 115,370; Majora 409,276 -> 408,965 |
| R600 | 2026-08-06 | afc97f2 | fill, bridges | [fix] bridge_angle fallback was 0.0 instead of -1 (Fill.cpp:243), so every surface looked like a bridge; short-circuit flipped on | FILL_PARAMS_BRIDGE_ANGLE_CPP (ON); FILL_BRIDGE_ANGLE (ON, was opt-in) | Benchy matched 115,887 -> 115,900; Majora 409,297 -> 409,318 |
| R601 | 2026-08-06 | 5ebc3c7 | wipe tower | [fix] Tag comment trio moved to the head of the tower block, so tool changes sit under ; FEATURE: Prime tower as in C++ | TOWER_FEATURE_TAG_HEAD (ON) | Majora matched 409,318 -> 630,879 (16.26% -> 25.06%) |
| R602 | 2026-08-06 | 0b0ed66 | wipe tower | [port] Tool-change header (WipeTower.cpp:3272-3281): CP TOOLCHANGE START comments, M220 speed override, tool-change counter | WT_TOOLCHANGE_HEADER_CPP (ON) | Majora matched 630,879 -> 646,455 (25.06% -> 25.54%) |
| R603 | 2026-08-06 | ec60025 | wipe tower | [port] M73 E remaining-filament-change countdown (GCodeProcessor.cpp:601) spliced after T<n> in the tower block | M73_REMAIN_FILAMENT_CHANGES (ON) | Majora matched 646,455 -> 648,759 (25.54% -> 25.60%) |
| R604 | 2026-08-06 | cb6fa65 | travel/retract | [diag] ; WIPE_START deficit: C++ installs the wipe path at four sites, we at two (extrude_multi_path and tower append missing) | - | Majora ; WIPE_START blocks C++ 44,174 vs Rust 36,394 |
| R605 | 2026-08-06 | 6f68054 | travel/retract | [gated] extrude_multi_path wipe path ported (GCode.cpp:5664-5673); reachable, wipe count unchanged, parity-neutral | WIPE_MULTIPATH_CPP (opt-in); probes: WIPE_MULTIPATH_POP | Gate ON: Benchy matched 115,900 -> 115,910; Majora 648,759 -> 648,741 |
| R606 | 2026-08-06 | a412ba2 | travel/retract | [diag] Wipe E per block matches on object features; tower wipe E differs, blamed on an unused parameter (site corrected by R607) | - | - |
| R607 | 2026-08-06 | 47f17dd | travel/retract | [falsified] R606's identity was circular and its function is dead code; the live writer retract() has no toolchange length | - | - |
| R608 | 2026-08-06 | d639827 | wipe tower | [gated] retract_for_toolchange() and tower-entry wipe ported (GCode.cpp:1083-1085); looked inert, ordering blamed (refuted R609/R610) | TOWER_ENTRY_WIPE_CPP (opt-in) | Gate ON: Majora ; WIPE_START 36,394 -> 36,396 |
| R609 | 2026-08-06 | cda82da | wipe tower | [falsified] R608's "already retracted at tower entry" cause refuted; guards pass and emit logic is sound, contradiction left open | probes: TOWER_ENTRY_WHY | - |
| R610 | 2026-08-06 | 48d7dc2 | wipe tower | [diag] Tower-entry wipe was firing all along (R608 measured a gate-off file); the gate moves wipes, C++ retracts twice and we once | probes: TOWER_ENTRY_WHY | Gate ON: Majora matched 648,759 -> 648,773 |
| R611 | 2026-08-06 | 6678950 | wipe tower | [fix] Stale retracted flag cleared after the raw tower splice (mark_unretracted_after_raw); tcr.wipe_path now consumed | TOWER_ENTRY_WIPE_CPP (ON, was opt-in); TOWER_EXIT_WIPE_CPP (ON) | Majora matched 648,759 -> 656,934 (25.60% -> 25.78%) |
| R612 | 2026-08-06 | c73fedc | wipe tower | [falsified] WIPE_MULTIPATH_CPP re-check still net negative (stays opt-in); finish_layer state-sync theory inert and reverted | WIPE_MULTIPATH_CPP (opt-in) | - |
| R613 | 2026-08-07 | c470772 | wipe tower | [diag] C++ skips no after-toolchange wipes (R612 premise was a census artefact); residual is missing finish-layer tower blocks | - | Majora finish-layer tower blocks Rust 656 vs C++ 932 |
| R614 | 2026-08-07 | 3394558 | wipe tower | [port] Finish-layer emitter iterates all tcrs (inert today); deficit is in the planner, wipe_tower_blocks is never populated | TOWER_FINISH_ALL (ON) | - |
| R615 | 2026-08-07 | ef97a01 | wipe tower | [port] Tower acceleration chain wired: M204 from wipe-tower moves (WipeTower.cpp:760-763); block-multiplicity theory refuted | TOWER_ACCEL_CPP (ON) | Majora matched 656,934 -> 660,333 (25.78% -> 25.88%) |
| R616 | 2026-08-07 | 083adfa | wipe tower | [fix] EMPTY GRID close had one blank line too many (WipeTower.cpp:3643); finish-block Y offset is a box-extent gap, not a rigid shift | - | Majora line parity unchanged 25.88% |
| R617 | 2026-08-07 | 0cd2015 | wipe tower, config | [port] Populate wipe_tower_blocks: adhesiveness category config, get_block_by_category, generate_wipe_tower_blocks steps 1-3; byte-neutral | TOWER_BLOCKS_CPP (ON) | - |
| R618 | 2026-08-07 | 11c3db7 | wipe tower | [port] Block depth layout, tool-change charge and step-4 layer_depths max (WipeTower.cpp:4316); finish_block skip predicate now fires | TOWER_BLOCK_LAYER_DEPTH_MAX (ON); probes: TOWER_FINISH_BLOCK_CENSUS | Majora finish-block census 0 -> 210 (C++ 276) |
| R619 | 2026-08-07 | 1044e55 | wipe tower | [falsified] A separate finish_block emitter would double-emit: finish_layer already emits its content in one tcr; the gap is content | - | - |
| R620 | 2026-08-07 | 956a4d2 | wipe tower | [diag] Gap-wall skip points entirely unported: wall_skip_points never populated, so the tower lacks every gap retract/travel/unretract | - | Majora `G1 E-0.8000 F1800`: C++ 5,399 vs Rust 0 |
| R621 | 2026-08-07 | 0c6a6f6 | wipe tower | [port] Wall skip-point producer get_wall_skip_points (WipeTower.cpp:3135-3186), per-layer as in C++; not yet consumed, byte-neutral | TOWER_SKIP_POINTS_CPP (ON) | - |
| R622 | 2026-08-07 | 19d619b | wipe tower | [diag] generate_support_wall (:5081) is the dead twin; live gap chain is generate_support_wall_new (:5030) -> generate_path (:1249) | - | - |
| R623 | 2026-08-07 | fb441f5 | wipe tower, geometry | [port] Wall-gap leaf helpers ray_intersetion_line, move_point_along_polygon, insert_points (WipeTower.cpp:230-399) with unit tests | - | - |
| R624 | 2026-08-07 | 92e7dd5 | wipe tower, geometry | [port] Wall-gap constructor remove_points_from_polygon (WipeTower.cpp:510-593) and helpers; a unit test caught a missing ring closure | - | - |
| R625 | 2026-08-07 | 6c5f849 | wipe tower | [port] Gap wall wired through generate_path linear branch (WipeTower.cpp:1249-1309); tower retract speed now read from config (ungated) | TOWER_WALL_GAPS_CPP (ON) | Majora line parity 25.88% -> 26.02% (660,333 -> 668,158 matched) |
| R626 | 2026-08-07 | edc60cd | wipe tower | [fix] Gap travel writes its feedrate inline as C++'s travel(dest, 600.) does (WipeTower.cpp:884); faithful but parity-neutral | - | Majora matched 668,158 -> 668,151 (flat) |
| R627 | 2026-08-07 | a090a02 | wipe tower | [fix] Gap-wall ironing travels given explicit feedrates 600/240 (WipeTower.cpp:4085-4094); no more bare unretracts per tool change | - | Majora 26.05% -> 26.15% (668,151 -> 670,866 matched) |
| R628 | 2026-08-07 | 09566c6 | wipe tower | [gated] Tower extrude_arc ported (WipeTower.cpp:798-872) but pays nothing: C++'s tower "arcs" are spiral Z-lifts, not wall arcs | tower arc-fitting gate, name not given (opt-in) | Gate on: Majora matched 670,866 unchanged, body -1,184 |
| R629 | 2026-08-07 | de150aa | travel/retract | [diag] Object spiral lifts fire far too often; blamed on a never-cleared position_known guard (later shown to be a small part) | - | Majora object G17: C++ 6,062 vs Rust 36,407 |
| R630 | 2026-08-08 | da08365 | travel/retract | [gated] Ported is_current_position_clear flag and per-travel zhtAuto lift type (GCode.cpp:7046/:7089); both regress, kept opt-in | WRITER_POS_CLEAR_CPP (opt-in); LIFT_TYPE_AUTO_CPP (opt-in) | Level: Majora 26.15% (670,865), Benchy 75.03% (115,901) |
| R631 | 2026-08-08 | cb0566f | travel/retract | [gated] Ported overhang-lift chain: detect_overhangs_for_lift (PrintObject.cpp:814-853), is_through_overhang (GCode.cpp:6972); gates off | probes: OVERHANG_LIFT_CENSUS | Gate on: Majora 670,865 -> 667,490; Benchy 115,901 -> 112,756 |
| R632 | 2026-08-08 | ad90834 | config | [fix] z_hop_type was never read from config; now resolved filament-first (GCode.cpp:1272); R631's overhang producer proven exact vs C++ | TRAVEL_G1_WITH_Z (opt-in) | Flat: Benchy 75.03% (115,900), Majora 26.15% (670,865) |
| R633 | 2026-08-08 | 8d7571b | travel/retract | [falsified] Object/instance loop is moot (Majora single-instance); region, frame, intersection_pl and clip threshold all ruled out | probes: OVERHANG_PRED_CENSUS | - |
| R634 | 2026-08-08 | 77625f2 | travel/retract | [falsified] Travel-router theory refuted (reduce_crossing_wall = 0); overhang producer matches C++ on Majora; predicate itself fires rarely | - | Majora predicate true 611 of 35,754 calls (1.7%) |
| R635 | 2026-08-08 | d50fbd6 | travel/retract | [diag] Overhang predicate was never the main spiral source; remainder attributed to auto_lift_type sites (GCode.cpp:4094-4096) | - | Majora predicate true: C++ 1,478 vs ours 611 |
| R636 | 2026-08-08 | fa10ec2 | travel/retract | [diag] Per-site census: only three C++ forced-lift sites fire (:747, :1085, :4582); object side is 656; mechanism added, not wired | - | - |
| R637 | 2026-08-08 | 189d7a3 | travel/retract | [diag] Funnel counter in C++ _spiral_travel_to_z: most spirals are eager lifts from apply_instantly retracts; we have one eager call site | - | C++ Majora spirals: eager 7,538 (85%), lazy 1,307 (15%) |
| R638 | 2026-08-08 | 4a3e39b | travel/retract | [port] Eager lift path: lift runs even when retract emits nothing (GCode.cpp:7097-7128); wipe-tower toolchange site wired (:747) | RETRACT_LIFT_ALWAYS (ON) | Majora 26.15% -> 26.26% (670,865 -> 673,583 matched) |
| R639 | 2026-08-08 | 7d36caa | travel/retract | [diag] LIFT_TYPE_AUTO_CPP re-measured on the eager path: still loses, less than before; remaining gap is object-region eager spirals | - | Gate on: Majora matched 673,583 -> 672,885 |
| R640 | 2026-08-08 | e852fe9 | travel/retract | [gated] Pre-tower toolchange retract (GCode.cpp:747) added; no gain and it undoes R638's in-tower spiral; R639's arithmetic refuted | TOWER_PRE_RETRACT (opt-in) | Gate on: Majora matched 673,583 unchanged |
| R641 | 2026-08-08 | 4c138f9 | gcode emission, config | [fix] timelapse_type was hardcoded 0 in the time_lapse_gcode template substitution; the "with wipe tower" branch now lands | - | Majora 26.26% -> 26.45% (673,583 -> 680,143 matched) |
| R642 | 2026-08-08 | 8df25b4 | gcode emission | [diag] Template branch sweep finds command lines missing their inline comment; the stated whitespace root cause is refuted in R643 | - | Majora inline-comment lines: C++ 19,731 vs ours 7,493 |
| R643 | 2026-08-08 | 4f32562 | cooling | [fix] Cooling buffer's feedrate rewrite truncated inline comments on F-bearing lines (two sites); it now keeps the rest of the line | - | Majora 26.45% -> 26.92% (680,143 -> 692,188 matched) |
| R644 | 2026-08-08 | 20ab18d | gcode emission | [fix] Dropped the ";Travel to a Wipe Tower" comment C++ never writes (full_gcode_comment is off, GCode.cpp:698-701); no parity effect | - | Majora matched flat at 692,188 |
| R645 | 2026-08-08 | b567121 | wipe tower | [port] Toolchange trailer after WIPE_TOWER_END: feedrate, flush_planner_queue, reset_extruder (WipeTower.cpp:2171-2177) | - | Majora 26.92% -> 27.04% (692,188 -> 697,630 matched) |
| R646 | 2026-08-08 | 7384df7 | wipe tower, config | [fix] WipeTowerConfig.travel_speed was never assigned (stuck at its default); now read from config (WipeTower.cpp:1739) | - | Majora 27.04% -> 27.15% (697,630 -> 700,352 matched) |
| R647 | 2026-08-08 | 8a6522f | config, cooling | [fix] Config sweep: per_extruder_cooling never populated; nine cooling keys added (GCodeEditor.cpp:402-460), removing spurious M106 lines | COOLING_CFG_KEYS (ON); MACHINE_ACCEL_KEYS (ON) | Majora 27.15% -> 28.00% (M106 89,529 -> 10,908); Benchy 115,900 -> 115,961 |
| R648 | 2026-08-08 | a45d6a5 | cooling | [port] Emit ;_FORCE_RESUME_FAN_SPEED after the tool-change template (GCode.cpp:944/:7479) so both fans re-emit per tool change | FORCE_RESUME_FAN_MARKER (ON) | Majora 28.00% -> 28.16% (700,354 -> 705,796 matched) |
| R649 | 2026-08-08 | e6b4176 | wipe tower | [port] Second flush_planner_queue after [filament_start_gcode] (WipeTower.cpp:2485); R648's trailer-ordering premise refuted | TOOLCHANGE_FLUSH_QUEUE (ON) | Majora 28.16% -> 28.24% (705,796 -> 708,517 matched) |
| R650 | 2026-08-09 | d50e418 | wipe tower | [fix] Tower travel_to_start is now followed by the bare `G1 Z` of travel_to_z (GCodeWriter.cpp:626); object-side Z deficit remains | TOWER_TRAVEL_Z (ON) | Majora 28.24% -> 28.32% (708,517 -> 711,537 matched) |
| R651 | 2026-08-09 | 8dea2bb | ordering | [fix] Pending M204 flush moved after the `; WIPE_START` marker (GCode.cpp:408); zero on content, proving line_parity.py is order-blind | WIPE_ACCEL_AFTER_MARKER (ON) | Majora matched 711,537 -> 711,537; in-order +7,740 (per R652) |
| R652 | 2026-08-09 | ca2e141 | instruments | [infra] New order-sensitive metric scripts/seq_parity.py (in-order matching within each layer/feature group); no engine change | - | Majora content 28.32%, in-order 18.65%; Benchy 75.07% / 63.61% |
| R653 | 2026-08-09 | 09ef90b | ordering | [diag] seq_parity --dump-divergence finds `; LINE_WIDTH:` / `G1 F` pairs inverted vs C++ (GCode.cpp:6607/:6663); seam order differs too | - | Majora inverted pairs: ours 148,548 vs C++ 0 |
| R654 | 2026-08-09 | 9d6b192 | ordering | [gated] Width tag now precedes the path feedrate exactly as C++; in-order gets worse while the tag count is short, so left opt-in | LINEWIDTH_BEFORE_SPEED (opt-in) | Gate on: Majora in-order 468,570 -> 442,261; content unchanged |
| R655 | 2026-08-09 | 48635ef | arachne | [falsified] LINE_WIDTH tag deficit is neither the width register nor missing paths; it is width variety, upstream in Arachne beading | - | Majora LINE_WIDTH tags: ours 154,063 vs C++ 215,199 |
| R656 | 2026-08-09 | ba56983 | gcode emission | [gated] Ported last_was_wipe_tower force-emit of Width/Height tags after a tower block (GCode.cpp:6591); parked because in-order falls | WIPE_TOWER_FORCE_TAGS (parked) | Gate on: Majora content 711,537 -> 712,741; in-order 468,570 -> 466,453 |
| R657 | 2026-08-09 | 8cad94b | arachne | [falsified] Width gap is not quantisation (constants identical); outer-wall width variety collapses per layer, partly re-attribution | - | Majora: 223 of 509 layers collapsed |
| R658 | 2026-08-09 | e524858 | arachne | [falsified] Outer-wall width loss is not feature mis-attribution; 40 FLAT layers print one width at matched run count; corrects R657 | - | - |
| R659 | 2026-08-09 | f775914 | arachne | [diag] Beading produces width variety on FLAT layers that the gcode outer wall lacks (all-inset count, qualified in R660) | probes: WTPWIDTH, WTPLAYER | - |
| R660 | 2026-08-09 | bca1cc2 | arachne | [falsified] thick_polyline_to_multi_path does not collapse widths; the inset-0 beading arrives already flat | probes: TPMPPROBE, AWIDTH | - |
| R661 | 2026-08-09 | 47ab629 | arachne | [falsified] Redistribute strategy is not the ceiling: its variable branch is unreachable; beading interpolation still 2.18x short | probes: BEADPROBE | - |
| R662 | 2026-08-09 | 578526f | arachne | [diag] Upward seed deficit 1.68x = list composition 1.17x times missing from-beading 1.44x; composition alone is not the cause | probes: UQM | - |
| R663 | 2026-08-09 | 85e7f0a | arachne | [diag] Upward pass chains from its own seeds (80%): static base 1.97%, amplification 5.07x; "chain never starts" refuted | probes: UQMSEED | - |
| R664 | 2026-08-09 | ff27428 | arachne | [diag] C++ measured: the 1.44x seed gap is base 1.198x times amplification 1.179x, split about evenly | probes: CPPUP | - |
| R665 | 2026-08-09 | 45e4953 | arachne | [diag] Seed base gap is depth mix: C++ has 2.24x more survivor nodes under 0.2 mm; C++ SCALING_FACTOR is 0.00001 (libslic3r.h:58) | probes: UQMDEPTH | - |
| R666 | 2026-08-09 | 36cc796 | arachne | [diag] isUpward filter at parity; skeletal graph has 1.62x fewer near-boundary edges than C++ | - | - |
| R667 | 2026-08-09 | aee8b71 | arachne | [falsified] collapse_small_edges removes the same fraction as C++; turning ARACHNE_COLLAPSE_SNAP_5 off worsens the deficit | probes: COLLAPSEPROBE | - |
| R668 | 2026-08-09 | e7141e5 | arachne | [falsified] Collapse does not remove the wrong edge set; near-boundary adjacency is already 1.647x short at graph construction | - | - |
| R669 | 2026-08-09 | 4423678 | arachne | [diag] Voronoi diagram and half-edge conversion at parity; we feed 1.169x fewer boundary segments; supersedes R589's target | - | - |
| R670 | 2026-08-09 | e514569 | arachne | [diag] WallToolPaths outline arrives 1.069x coarse and the prep chain adds 1.078x (WallToolPaths.cpp:455-486) | probes: POLYPROBE | - |
| R671 | 2026-08-09 | 601120d | perimeters | [diag] Slicer resolution faithful (PrintObjectSlice.cpp:144); claimed Arachne simplify_p units bug, withdrawn in R672 | - | - |
| R672 | 2026-08-09 | 2e1d2a7 | perimeters | [gated] Scaling the Arachne simplify_p tolerance empties the outline: simplify_p takes mm; its "gcode unchanged" result retracted in R673 | ARACHNE_SIMPLIFY_SCALED (opt-in) | - |
| R673 | 2026-08-09 | f5b4be4 | instruments | [falsified] R672 retracted: its A/B hashed a stale file after a panic; generate_arachne is the live wall site; A/B script hardened | - | - |
| R674 | 2026-08-09 | 5332b1a | perimeters | [diag] surface.expolygon reaches generate_arachne 1.652x coarser than C++ (PerimeterGenerator.cpp:1511), before simplify_p | probes: SURFPROBE | - |
| R675 | 2026-08-09 | b2bab0d | slicing/mesh | [falsified] The four douglas_peucker sites (PrintObjectSlice.cpp:509-613) are first-layer brim only; groupingVolumes group unported | probes: SLICEPTS | - |
| R676 | 2026-08-09 | e081f74 | slicing/mesh | [diag] Majora has 656 layers, not 4,720; claimed SURFPROBE revisit inflation (refuted in R677); no localisation | - | - |
| R677 | 2026-08-09 | 6cf440c | MMS | [diag] The coarseness is created entirely by MM segmentation fragmenting surfaces; the faithful cleanup ports were parked since R557 | - | Majora post-segmentation pts/contour 86.384 (C++) vs 50.884 |
| R678 | 2026-08-09 | 7f49548 | MMS | [fix] MM segmentation opening/closing cleanups (PrintObjectSlice.cpp:946-947, 962-964) switched on; fragmentation closed | MMSEG_OPENING (ON, was opt-in), MMSEG_CLOSING (ON, was opt-in) | Majora in-order 18.65% -> 18.70%; content 28.32% -> 28.24% |
| R679 | 2026-08-09 | dd0d06e | MMS | [diag] Segmentation areas match C++, so the cleanup tolerance is the residual; outer-wall width tags are short, not width variety | - | Majora outer-wall LINE_WIDTH tags 19,707 vs C++ 62,582 |
| R680 | 2026-08-09 | 8fafe9e | arachne | [falsified] Path-split rule is faithful (VariableWidth.cpp:80-90); the deficit is width changes entering it; corrects R679 | - | Majora width changes per outer-wall loop 0.0699 vs C++ 0.1960 |
| R681 | 2026-08-09 | 9027c47 | arachne | [diag] to_thick_polyline identical; flatness is in junction widths; edge-count decomposition later invalidated (R684) | - | - |
| R682 | 2026-08-09 | 041a90c | arachne, perimeters | [diag] Collapse exonerated per call; seperate_wall_generation branch (PerimeterGenerator.cpp:1565-1613) unported for Arachne | - | - |
| R683 | 2026-08-09 | ff70da1 | perimeters | [diag] Top-one-wall branch is live on nearly every Majora surface in C++; our port exists only in the classic path | - | - |
| R684 | 2026-08-09 | 84b89aa | perimeters | [gated] Ported seperate_wall_generation + should_enable_top_one_wall (PerimeterGenerator.cpp:1528-1613) into Arachne; output inert | ARACHNE_TOP_ONE_WALL (parked) | Majora in-order 469,489 -> 469,475 with gate on; parked |
| R685 | 2026-08-09 | 1139b5f | arachne | [diag] On matched graph populations the beading-difference rate is the dominant term (1.823x); part of the deficit unexplained | - | - |
| R686 | 2026-08-09 | 2711b9c | arachne | [falsified] Beading strategy is not coarser than C++ (BeadingStrategyFactory.cpp:35-56); re-closes R661 | - | - |
| R687 | 2026-08-09 | c5480c1 | arachne | [falsified] Equal distance_to_boundary on adjacent nodes is impossible by construction; outer width is pinned when bead_count > 2 | - | - |
| R688 | 2026-08-09 | a476df5 | arachne | [falsified] Bead-count regimes match on both engines; R687's bead-count mechanism claim disqualified; propagation structure is left. | - | - |
| R689 | 2026-08-09 | 69b0f38 | arachne | [diag] Propagation class mix is the mechanism: we copy where C++ interpolates; split at SkeletalTrapezoidation.cpp:1671. | - | fresh+interp share 0.0309 vs C++ 0.0544 (1.761) |
| R690 | 2026-08-09 | e5ce63b | arachne | [falsified] Upward-pass skip mix matches; R587's "entirely the bead_count guard" claim refuted; graph state matches. | - | - |
| R691 | 2026-08-09 | 7f44b65 | arachne | [fix] upward_quad_mids comparator made strict-weak (SkeletalTrapezoidation.cpp:1488); tie order proven inert, output unchanged. | UQM_STRICT_WEAK (ON); probes: TIEDENS | - |
| R692 | 2026-08-09 | 3bde3de | arachne | [diag] Initial beading population matches; upward seeding has ~3% leverage; the call-count gap reported here is refuted in R693. | - | - |
| R693 | 2026-08-09 | e195b3c | config | [infra] Behavioural gates moved off presence-only probe_enabled (NAME=0 meant ON) to new opt_in_gate; R692's call-count gap refuted. | ARACHNE_TOP_ONE_WALL (opt-in); +6 other behavioural gates (opt-in) | generateSegments calls 25,800 vs C++ 26,400 |
| R694 | 2026-08-09 | b64f9b2 | arachne | [diag] Voronoi faithful per outline point; the skeletal node gap is fewer outline points already at WallToolPaths entry (upstream). | - | entry points C++/ours 1.070; nodes per point 1.016 |
| R695 | 2026-08-09 | 37967d5 | MMS | [diag] Bracket A matches, bracket C has fewer points; entry gap read as a cancellation with simplify_p retention (overturned in R697). | - | - |
| R696 | 2026-08-09 | e4629e2 | clipper/geometry | [falsified] Simplify retention gap is not units, tolerance or algorithm (the faithful simplify path is retention-inert). | ARACHNE_SIMPLIFY_FAITHFUL (parked) | - |
| R697 | 2026-08-09 | 6d8d075 | MMS | [falsified] Arachne entry is invariant to the mm-segmentation opening sweep; the deficit is contour count, not vertex detail. | probes: MMSEG_OPEN_MULT | entry contours 26,791 vs C++ 29,512 |
| R698 | 2026-08-09 | 95c788b | MMS | [diag] Contour deficit is a hole deficit starting at slicing (bracket A); opening exonerated; clib remainder difference inert on holes. | MMSEG_DIFF_CLIB (parked) | holes at A 45 vs C++ 97; at C 44 vs 1,740 |
| R699 | 2026-08-09 | 699849e | clipper/geometry | [falsified] Boolean primitives and ExPolygon reconstruction keep holes (new hole_preservation test); deficit is in the boolean inputs. | - | - |
| R700 | 2026-08-09 | da680ee | slicing/mesh | [diag] Hole deficit is confined to layers 0-9 and 207-208; every other layer matches C++; segmentation output is nearly hole-free. | probes: SEGHOLES | - |
| R701 | 2026-08-09 | d37f4ec | 3MF load | [diag] Root cause: six negative_part volumes skipped at 3MF load (app_slice.rs:574-581); first-layer compensation ruled out. | - | - |
| R702 | 2026-08-09 | cb1d7b0 | 3MF load | [diag] Prices the negative-volume port from existing G-code: worth building; first estimate 10x low (segmentation amplifies holes). | - | 10,932 lines attributable, 0.46% of C++ body lines |
| R703 | 2026-08-09 | deacdf9 | 3MF load | [port] negative_part volumes parsed from model_settings.config into a separate negative mesh; inert until subtraction lands. | - | - |
| R704 | 2026-08-09 | 31362f1 | 3MF load | [port] Per-layer 2D subtraction of negative volumes (PrintObjectSlice.cpp:403-421); fixes a slice_at_z center_offset frame bug. | NEGATIVE_VOLUMES (ON); probes: NEGPROBE | Majora in-order 469,489 -> 469,628; content 709,080 -> 710,614 |
| R705 | 2026-08-10 | 2a8513c | instruments | [infra] New benchy-016-arachne fixture reproduces Majora's wall deficit single-material: the cause is Arachne, not multi-material. | - | arachne in-order 15.27%, outer wall 9.7% (Majora 18.59%, 9.0%) |
| R706 | 2026-08-10 | 734d132 | config | [fix] Arachne min_bead_width / min_feature_size / wall_transition_length defaults corrected to percent (PrintConfig.cpp:5940-6010). | - | arachne in-order 32,216 -> 32,977; content 48,798 -> 49,952 |
| R707 | 2026-08-10 | f288085 | config | [infra] New scripts/audit-config-defaults.py; 24 numeric default mismatches, all inert on the fixtures; R706 was the only live one. | - | - |
| R708 | 2026-08-10 | fac24af | gcode emission | [diag] Ordering loss is real (control script); 46% is the per-path header; LINEWIDTH_BEFORE_SPEED re-scored net negative, stays parked. | LINEWIDTH_BEFORE_SPEED (parked) | gate, not shipped: classic +1,308, arachne +339, majora -25,557 |
| R709 | 2026-08-10 | 9fee5c8 | ordering | [diag] Island set right, island visit order wrong (island_order.py); make_slices chain faithful; R708's no-op check was a stale build. | probes: MKSL | ceiling +6,599 in-order on benchy outer wall (not realised) |
| R710 | 2026-08-10 | 23e8592 | ordering | [diag] All four ordering stages faithful by probe; fills score badly even where island order matches, so their defect is geometry. | probes: PGSO, PGCH | island-order ceiling +11,911 in-order on benchy (not realised) |
| R711 | 2026-08-10 | 06f726a | fill | [falsified] Fill angle, spacing, grid phase and direction are right; line endpoints differ; new scripts/fill_geometry.py. | - | - |
| R712 | 2026-08-10 | f500f8c | fill | [falsified] Fill region matches where comparable, yet fills still mismatch there; R711's boundary inference refuted. | probes: FILLB | fills 13.6% in-order where region matches (walls 91.5%) |
| R713 | 2026-08-10 | cd77d80 | fill | [diag] Fills are at least two defects by run structure; mismatch is not at run ends; sparse_infill_anchor default mismatch is inert. | - | - |
| R714 | 2026-08-11 | fb70d1a | fill | [falsified] Fill assignment is near-identical despite a divergent live group_fills and unported make_fills; R713's class (a) withdrawn. | probes: FILLASN | - |
| R715 | 2026-08-11 | a7e5e40 | fill | [diag] FillConcentricInternal is a faithful port fed a faithful fill_no_overlap; missing cross-region no-overlap union is unreachable. | probes: FILLNOV | - |
| R716 | 2026-08-11 | b33bcfc | fill | [falsified] No WallToolPathsParams default bug (C++ has no member initialisers); concentric line exhausted; sparse is a continuum. | - | - |
| R717 | 2026-08-11 | 3fdd0c9 | fill | [diag] Sparse infill mismatch is predicted by floating vertical shell on the same layer; anchoring function is ported and live. | - | - |
| R718 | 2026-08-11 | 5474d7a | fill | [falsified] Narrow / narrow-floating classification (Fill.cpp:466-550) is faithful; FVS link recast as a marker (overturned in R719). | probes: FVSCLS | - |
| R719 | 2026-08-11 | cb76aa8 | fill | [falsified] FVS effect is not a narrowness proxy: largest on the widest regions and hits only the interior fills. | - | - |
| R720 | 2026-08-11 | a867291 | fill | [diag] Region and no-overlap clearings do cover FVS layers, yet fills still mismatch there; target moves to Arachne on the fill path. | - | internal solid 1.9% shared vertices on FVS layers |
| R721 | 2026-08-11 | f85cce9 | fill | [diag] Arachne fill-path topology faithful but fill inputs not bit-identical; ApplySafetyOffset::Yes (Fill.cpp:740) fix loses, parked. | FILL_NOOVERLAP_SAFETY (parked); probes: ARWTP | gate, not shipped: net -64 in-order across four fixtures |
| R722 | 2026-08-11 | b61c17a | fill | [fix] Fill boundary offset2_ex deltas now use truncated coord_t integers as native (PerimeterGenerator.cpp:1395-1428). | FILLBOUND_QUANT (ON, was opt-in); probes: FBIS | benchy in-order 98,259 -> 98,405; cube 10,066 -> 10,068 |
| R723 | 2026-08-11 | d719c0a | fill | [diag] 89 of 90 bit-identical fill inputs still give different Arachne output; 4-vertex reproducer; safety-offset gate stays parked. | FILL_NOOVERLAP_SAFETY (parked) | Arachne fill output bit-identical on 1/90 |
| R724 | 2026-08-11 | e5ce6e5 | arachne | [diag] Arachne fill divergence starts in prepared_outline, before the Voronoi (geo backend 1 micron snap); offset-only clib fix parked. | ARACHNE_PREP_OFFSET_CLIB (parked); probes: AGEN | - |
| R725 | 2026-08-12 | 48a1ad3 | arachne | [fix] Arachne prep chain (triple offset, SimplifyPolygons, final union) at coord_t ClipperLib precision; prepared_outline bit-identical. | ARACHNE_PREP_CLIB (ON, was opt-in as ARACHNE_PREP_OFFSET_CLIB) | benchy 98,405 -> 101,558; arachne +2,677; Majora 469,628 -> 469,005 |
| R726 | 2026-08-12 | cc1308e | clipper/geometry | [diag] Prices the geo-clipper precision class via a scale sweep; vehicle must be _clib conversion, since geo is slow at high scale. | GEO_SCALE_COORDT (opt-in); probes: GEO_SCALE | benchy in-order 101,558 -> 104,492 at coord_t scale (not shipped), 2.3x time |
| R727 | 2026-08-14 | b106347 | clipper/geometry | [infra] Five hardcoded geo scales now honour geo_clipper_scale(); class price corrected upward; _clib wrapper dispatch recurses (trap). | - | benchy in-order 101,558 -> 105,972 at coord_t scale (not shipped) |
| R728 | 2026-08-14 | 7b28a2d | clipper/geometry | [fix] difference() dispatched to coord_t _clib after a geo-body refactor breaks the recursion; union dispatch too slow, parked. | CLIPPER_DIFF_GEO (ON); CLIPPER_UNION_GEO (parked) | benchy 101,558 -> 102,645; Majora 469,005 -> 471,202; arachne +44 |
| R729 | 2026-08-14 | 41e4c45 | perf | [falsified] Union slowdown is not union_ex_clib marshalling; the cost lands in fill_loop. | probes: UNIONPROF | fill_loop 0.248s -> 1.806s with CLIPPER_UNION_GEO on (benchy) |
| R730 | 2026-08-14 | 172c0a1 | perf | [falsified] fill_loop blowup is group_fills (fill/mod.rs), not the concentric Arachne fillers. | probes: FILLPROF | group_fills 195.4ms -> 906.6ms with CLIPPER_UNION_GEO on (benchy) |
| R731 | 2026-08-14 | dbec690 | perf | [diag] CLIPPER_UNION_GEO slowdown in group_fills is the narrow-classification block; a second group_fills call site found | probes: GFPROF | benchy narrow_block 315.7ms -> 1730.2ms with union on (5.5x) |
| R732 | 2026-08-14 | 874c86f | perf, fill | [fix] is_narrow_infill_area is 99% of the narrow block; bit-identical bbox early-out added; bbox-clip-helper theory refuted | NARROW_BBOX_EARLYOUT (ON); CLIPPER_UNION_GEO (parked) | benchy union-arm wall time 1.653x -> 1.324x |
| R733 | 2026-08-14 | 7675bb9 | perf | [diag] export_gcode cost is pre-loop setup, not gcode writing; string-formatting theory refuted | probes: EXPPROF | benchy generate: pre-loop 1109.2ms (91%), layer loop 104.4ms |
| R734 | 2026-08-14 | bb1ed05 | perf, seam | [diag] SeamPlacer::init is the pre-loop cost and compute_global_occlusion is 97% of it; visibility-stage theory refuted | probes: SPPROF | benchy compute_global_occlusion 1071.0ms (~41% of the slice) |
| R735 | 2026-08-16 | 5826fc6 | perf, seam | [fix] Seam occlusion raycast shim had dropped native's tbb::parallel_for (SeamPlacer.cpp:156); restored, bit-identical | SEAM_RAYCAST_SERIAL (opt-in) | benchy slice 2.76s -> 1.83s; vs C++ 1.41x -> 0.90x |
| R736 | 2026-08-16 | b464b7f | perf, bridges | [fix] Bridge anchor grow was recomputed 7x on one layer; memoised; process_external_surfaces parallel over layers as native | BRIDGE_GROW_CACHE (ON); PES_PARALLEL (ON); probes: PESPROF | benchy slice 1.83s -> 1.42s; vs C++ 0.919x -> 0.711x |
| R737 | 2026-08-16 | 78ab28a | bridges | [fix] Bridge anchor expand routed to ClipperLib at coord_t as native (LayerRegion.cpp:323); CLIPPER_UNION_GEO re-priced, stays parked | BRIDGE_ANCHOR_GROW_CLIB (ON); CLIPPER_UNION_GEO (parked) | Majora in-order 471,202 -> 471,204; benchy vs C++ 0.711x -> 0.638x |
| R738 | 2026-08-16 | 22e32bf | fill | [fix] Exact coord_t unions shipped after a provably-empty early-out removed dead anchoring group_fills work (Fill.cpp:782) | CLIPPER_UNION_GEO (ON, was opt-in); ANCHOR_GEN_EARLYOUT (ON); probes: FLPROF | benchy 102,645 -> 105,828; arachne 35,698 -> 37,349; Majora 471,204 -> 473,764 |
| R740 | 2026-08-16 | d5f5126 | perf, fill | [fix] is_narrow_infill_area offset routed to ClipperLib as native (Fill.cpp:159); corrects R737's unit note; two old gates marked dead | NARROW_OFFSET_CLIB (ON); probes: NARROWCENSUS | benchy slice 1.764s -> 1.418s; vs C++ 0.907x -> 0.742x |
| R742 | 2026-08-16 | 6e11320 | arachne | [diag] On a 4-vertex reproducer arachne emits the same junctions as C++ with a rotated loop start; Voronoi-position theory refuted | probes: AGENPTS, AGENPTS_MAXOV | - |
| R743 | 2026-08-16 | 1a076b8 | arachne | [fix] Loop seed emulates native's ankerl::unordered_dense set (SkeletalTrapezoidation.cpp:1997), replacing R99's min-key | ARACHNE_QUADSTART_DENSE (ON) | benchy 105,828 -> 108,800; arachne 37,349 -> 39,708; Majora 473,764 -> 477,957 |
| R744 | 2026-08-16 | cb77c68 | ordering | [fix] Extruder-order seed and tie-break follow native's input-order rule (GCode/ToolOrderUtils.cpp); inert on all four fixtures | TOOLORDER_INPUT_ORDER (ON) | - |
| R753 | 2026-08-16 | ff7cf73 | seam | [fix] place_seam tries split_at_vertex before split_at, as SeamPlacer.cpp:1521-1527 | SEAM_SPLIT_AT_VERTEX (ON) | benchy 108,800 -> 108,900; arachne 39,708 -> 39,727; Majora flat |
| R754 | 2026-08-16 | 8d75fed | seam | [gated] Concave-corner seam block ported (SeamPlacer.cpp:1499-1519); fixes the reproducer but regresses via an outer-wall cascade | SEAM_CONCAVE_CORNER (opt-in) | gate on: benchy 108,900 -> 108,099 |
| R756 | 2026-08-17 | e90dbfb | cooling | [infra] Deleted the dead gcode/cooling_buffer.rs twin (live code is gcode/cooling.rs); smoothing config cleared as a cause | - | - |
| R757 | 2026-08-20 | 88feed9 | cooling, seam | [fix] set_speed always emits F (GCodeWriter.cpp:387); exact time-stretch cooling solver (CoolingBuffer.cpp:191); seam corner flipped ON | SET_SPEED_ALWAYS_EMIT, COOLING_EXACT_STRETCH (ON); SEAM_CONCAVE_CORNER (ON, was opt-in) | benchy 108,900 -> 110,517; arachne 39,717 -> 39,860; Majora 477,904 -> 477,860 |
| R758a | 2026-08-20 | 08340a6 | gcode emission | [gated] Skipping the collection-level F pre-set (native F is per-path, GCode.cpp:6663) regresses arachne; parked | NO_COLLECTION_PRESPEED (opt-in); LINEWIDTH_BEFORE_SPEED (parked) | gate on: arachne -51 |
| R758b | 2026-08-20 | 19bce57 | arachne, fill | [fix] thicklines() read the C++ pair-model width array (Polyline.hpp:282) per-vertex, flattening variable widths; now reads pairs | THICKLINES_PAIR_MODEL (ON); probes: FCIPROBE | benchy 110,517 -> 113,853; arachne 39,860 -> 50,695; Majora 477,860 -> 482,929 |
| R759 | 2026-08-20 | 8f5d4b0 | perimeters | [diag] Classic perimeter chain is byte-exact until the gap-fill footprint, which differs on all gap layers | probes: PAPROBE, LOOPI, GFPROBE +5 | - |
| R760 | 2026-08-20 | 99cba3f | flow/float | [fix] Flow width quantized through f32 as native (Flow.hpp:107); the gate had been a presence-only env | FLOW_F32 (ON, was presence-only); probes: GFPARAM, GFC1-3, GFRAW +4 | benchy 113,853 -> 114,494; arachne 50,695 -> 50,688; Majora 482,929 -> 481,879 |
| R761 | 2026-08-20 | 6acfeeb | bridges | [fix] Fully-anchored bridge direction uses the faithful area-moment PCA port with holes (LayerRegion.cpp:324); byte-inert | BRIDGE_PCA_FAITHFUL (ON); probes: PCAPROBE | - |
| R762 | 2026-08-20 | 141877e | bridges | [diag] Remaining bridge divergence is two strata: differing input surfaces on layer 2 and knife-edge sub-um anchor drift | probes: BREXP, BRIDGEIN | benchy bridges angle-exact 10/13 |
| R763 | 2026-08-20 | 79aebe8 | bridges, slicing | [diag] Bottom-bridge birth is byte-exact; no independent bridge defect remains; layer-0 input surface is structurally different | probes: BOTPROBE | - |
| R764 | 2026-08-20 | 31165e1 | slicing/mesh | [fix] Faithful layer-0 elephant-foot compensation (PrintObjectSlice.cpp:1228-46) made default; was a presence-only env since R316 | L0_EFC (ON, was presence-only) | benchy 114,494 -> 114,883; arachne 50,688 -> 50,708; Majora 481,879 -> 481,906 |
| R765 | 2026-08-20 | 2cf7ac7 | MMS | [fix] All six clipper op sites of the multi-material segmentation chain moved from the 1 um geo backend to exact ClipperLib | MMSEG_CLIB (ON) | Majora in-order 481,906 -> 500,912 |
| R767 | 2026-08-20 | f9a493c | MMS | [diag] Majora region structure matches, but painted base layers never fragment into per-color islands (top/bottom propagation missing) | probes: PACALL | Majora base-layer surfaces 9 vs native 387/420/317 |
| R768 | 2026-09-01 | 409f991 | MMS | [port] mmu_segmentation_top_and_bottom_layers (MMS.cpp:1321-1615) plus the slab slicer it needs (TriangleMeshSlicer.cpp) | MMS_TOPBOT (ON) | Majora in-order 500,912 -> 651,235 |
| R769 | 2026-09-01 | d88bb41 | MMS | [fix] mm-seg input prep lost the negative-connector holes; simplify now Douglas-Peucker + SimplifyPolygons (ExPolygon.cpp:231-261) | MMSEG_SIMPLIFY_DP (ON); probes: TBPROBE2, TBPROBE5, TBPROBE6 | Majora in-order 651,235 -> 651,622 |
| R770 | 2026-09-01 | dff6155 | MMS | [port] Negative-volume fill before segmentation and clip-back to the holed input afterwards (MMS.cpp:2202-2255, 2369-83) | MMSEG_NEGFILL (ON) | Majora in-order 651,622 -> 651,928 |
| R771 | 2026-09-01 | 41ac797 | MMS | [fix] extract_colored_segments keeps raw graph-walk pieces without union (MMS.cpp:413-481); aggregate-neutral downstream | MMSEG_RAW_EXTRACT (ON); probes: TBPROBE7, TBPROBE8, TBPROBE9 +1 | Majora outer-wall content +291 |
| R772 | 2026-09-02 | c59303d | arachne | [falsified] Voronoi-ULP theory dead: a shim around real boost::polygon voronoi is byte-identical to the rust crate | probes: AWALL | arachne AWALL: 0 exact, 392 drift, 334 structural |
| R772 | 2026-09-02 | 5c7f7ef | housekeeping | [infra] Workspace lockfile for boost-voronoi-sys | - | - |
| R773 | 2026-09-02 | 16f5324 | arachne | [diag] Arachne fork is upstream of the skeleton: the pre-WallToolPaths chain diverges although perimeter inputs are byte-exact | probes: TBARACH, POLYPROBE | - |
| R774 | 2026-09-02 | 22ce9f9 | arachne | [fix] Speculative ic=1 one-wall WallToolPaths pass (PerimeterGenerator.cpp:1896-1921) made default; call census now matches native | ARACHNE_TOP_ONE_WALL (ON, was opt-in); probes: AWIN | arachne 50,708 -> 51,478; Majora 651,928 -> 652,008 |
| R775 | 2026-09-02 | 8a6cae4 | arachne, flow/float | [fix] Exact offset_ex(Polygons) twin for the arachne inset; Flow scaled width truncates the f32 round-trip as native (0.42 -> 41999) | ARACHNE_OFFSET_CLIB (ON); FLOW_SCALED_TRUNC (ON); probes: AWIN, AWDELTA, AWINPANIC | Majora 652,008 -> 1,052,148; arachne 51,478 -> 94,488; benchy 114,883 -> 114,906 |
| R776 | 2026-09-02 | c7a3155 | arachne, ordering | [diag] Most arachne wall outputs are byte-exact per surface; island order differs with a stable permutation (gcode planner side) | - | arachne wall outputs exact 494 of 720; ordering ceiling +6,416 |
| R777 | 2026-09-02 | 66c7e44 | ordering | [diag] Ordering cascade starts at layer 2 (Bridge / Internal solid interleave); blamed on a bridge-surface split, refuted in R778 | probes: ISLPROBE, FILLSF | - |
| R778 | 2026-09-02 | cb037ba | ordering, fill | [falsified] Bridge-split theory: fill structures match in both engines; the fork is per-path role marking in the concentric solid fill | probes: SFBRIDGE, BRGROUP, BRCLOSE +1 | - |
| R779 | 2026-09-02 | 2c9e558 | ordering, fill | [gated] Region::append flattening of can_sort fill collections (GCode.cpp:7727-7735) plus no_sort flags ported; regresses, parked | ISLAND_FLATTEN (opt-in) | gate on: arachne in-order -1,998; Majora in-order -72,481 |
| R780 | 2026-09-02 | 883302d | ordering, fill | [fix] Per-filler no_sort flag stops re-chaining inside concentric/monotonic fill collections (ExtrusionEntityCollection.hpp:103) | FILL_NO_SORT (ON); ISLAND_FLATTEN (parked) | benchy 114,906 -> 117,603; arachne 94,488 -> 95,969; Majora neutral |
| R783 | 2026-09-02 | 273ebde | MMS | [diag] Colored-boundary drift (1-30 units) is not the projection lerp or rounding; it is painted-line discrete decisions | probes: TBPROBE8V | - |
| R784 | 2026-09-02 | f1f5a18 | MMS | [diag] Painted-line drift root: native projects in an f32 volume-local frame, rust in the placed f64 frame | MMS_PAINT_TRUNC (opt-in); probes: PLDUMP | Majora layer 385 painted lines byte-common 296/2,425 |
| R785 | 2026-09-02 | 2f7b5ce | MMS, 3MF load | [falsified] Majora's paint frame is a full 3D rotation; the center round-trip approximation is insufficient and was removed | probes: TRDUMP | - |
| R786 | 2026-09-02 | a8d770e | MMS, 3MF load | [gated] Paint-frame projection machinery (f32 affine shim, loader keeps raw paint verts); frame does not match native yet, parked | MMS_PAINT_FRAME (opt-in) | - |
| R787 | 2026-09-02 | d0e86bb | MMS, 3MF load | [gated] center_offset equation proven (Model.cpp:1559-73, PrintApply.cpp:148-152); selector split in the local frame; still parked | MMS_PAINT_FRAME (opt-in) | Majora layer 385 painted lines byte-common 296 -> 564/2427 |
| R788 | 2026-09-02 | f12a72a | MMS | [fix] Painted-line projection byte-exact with round conversion; topbot ctx on volume-local meshes (MMS.cpp:1345-55); trunc theory dead | MMS_PAINT_FRAME (ON, was opt-in); probes: PTDUMP | Majora in-order 1,052,148 -> 1,056,793 |
| R790 | 2026-09-02 | e3f93b5 | fill | [fix] Generic connect_infill_expolygon delegates to the faithful BoundaryInfillGraph port (FillBase.cpp:1501-1733); output unchanged | FILL_CONNECT_FAITHFUL (ON) | - |
| R791 | 2026-09-02 | b8fadcc | fill | [port] Gyroid arm wired to the faithful FillGyroid::fill_surface_single (FillGyroid.cpp:151-210); byte-neutral; two unit bugs found | GYROID_FSS (default not stated) | - |
| R792 | 2026-09-02 | 4f08c66 | fill, clipper | [fix] intersection_pl is an exact twin of ClipperLib's open-path clip (ClipperUtils.cpp:841-849) instead of a hand-rolled clipper | CLIB_INTERSECTION_PL (ON) | Majora in-order 1,056,793 -> 1,057,745 |
| R794 | 2026-09-02 | 63b607f | fill | [diag] Rust fits more arcs than native in every major stratum; G1-only censuses were blind to arc fitting | probes: FSSOUT | Majora arcs: outer wall 155,612 v 142,195; sparse 116,630 v 111,862 |
| R796 | 2026-09-02 | 85909c4 | fill | [falsified] ArcFitter port is faithful on identical inputs; the lever is FloatingVerticalShell path fragmentation | probes: ARCPROBE | Majora FVS simplify calls 81,678 vs 72,349 |
| R797 | 2026-09-02 | 5d07c37 | fill | [diag] Rust floating detection over-fires on FVS thicklines; the converter is faithful, the defect is in the detection wiring | probes: FVSPROBE | Majora floating thicklines 46,374 vs native 347 |
| R798 | 2026-09-02 | 0829a00 | fill | [fix] FVS anchor lines use flow.spacing() and the exact clip; near-inert; root narrowed to force_no_detect (FillFloatingConcentric.cpp:713) | FVS_ANCHOR_GYROID (ON) | Majora floating thicklines 46,374 -> 46,278 |
| R799 | 2026-09-02 | 5699144 | fill | [fix] has_intersecting_edges mirrors native pointer-equality adjacency and the closed-flag EdgeGrid; partial, barely moves the count | FVS_SELFX (ON) | Majora FVS self-intersections 0 -> 6 vs native 8,545 |
| R800 | 2026-09-02 | c96533c | fill | [diag] Native self-intersecting FVS loops replayed through rust's EdgeGrid do not fire (0/40); a synthetic bowtie does | probes: FVSDUMP3, FVSDUMP_FILE | - |
| R801 | 2026-09-02 | b8d8d1f | fill | [fix] Native FVS self-intersect check reads freed memory, so floating detection is effectively off; rust skips detection to match | FVS_FORCE_NO_DETECT (ON); probes: FVSDUMP4 | Majora 1,057,745 -> 1,116,613; benchy 118,951 (+1,348); arachne 97,790 (+1,821) |
| R803 | 2026-09-05 | 79f9a72 | arachne, flow/float | [fix] WallToolPaths prep offset goes through native union/shrink Paths (ring start); scaled() divides by 1e-5 and truncates | ARACHNE_PREP_OFFSET_PATHS (ON); SCALED_DIV_TRUNC (ON); probes: AWDUMP_LID, AWST_SX +1 | Majora 1,116,654 -> 1,124,799; arachne 97,790 -> 114,757; benchy -> 119,068 |
| R803b | 2026-09-05 | 99c7335 | arachne | [port] smooth_overhang_level lowpass filter of arachne wall overhang degrees (PerimeterGenerator.cpp:527-608) | ARACHNE_SMOOTH_OVERHANG (ON); probes: AWOUT, OVHDEG | arachne 114,757 -> 115,792; Majora 1,124,799 -> 1,128,965 |
| R803c | 2026-09-05 | dc8e48e | arachne | [fix] Lower slices for arachne overhang degrees grown with exact ClipperLib over all rings (PerimeterGenerator.cpp:1781) | ARACHNE_LOWER_GROW_CLIB (ON) | arachne 115,792 -> 124,025; Majora 1,128,965 -> 1,217,995 |
| R803d | 2026-09-05 | 0909dd6 | arachne | [fix] Overhang sampling never fired: segment length was measured in mm against a scaled interval (to_f64 unscales) | probes: OVHD, OVHPATH, OVHSUBJ +1 | arachne in-order 124,025 -> 133,818 |
| R803e | 2026-09-05 | d1ecf31 | arachne, cooling | [port] Arachne outer walls record LoopNodes (PerimeterGenerator.cpp:684-707/937) so the cross-layer speed smoother runs | ARACHNE_LOOP_NODES (ON); probes: NODEDBG, SSD_LID, CPOUT | arachne in-order 133,818 -> 134,299 |
| R804a | 2026-09-05 | c15bd11 | slicing/mesh | [fix] Negative volumes sliced with the model-part closing radius and DP simplification (PrintObjectSlice.cpp:156-158) | NEGVOL_SLICE_PARAMS (ON); probes: LSLPROBE, SLICEDBG, NEGCUT | Majora 1,276,793 -> 1,277,236 |
| R805 | 2026-09-05 | bbee1b3 | slicing/mesh | [fix] Main volume sliced from raw volume-local verts through the exact f32 volume transform, not the pre-placed f64 mesh | SLICE_VOLUME_TRAFO (ON) | Majora in-order 1,277,236 -> 1,473,706 (53.9%) |
| R805b | 2026-09-05 | 3b6e65c | slicing/mesh | [fix] Negative volumes sliced through their exact per-volume frame; pre-MMS region slices now byte-exact on every Majora layer | NEGVOL_EXACT_FRAME (ON) | Majora in-order 1,473,706 -> 1,475,536 |
| R806 | 2026-09-05 | cc84def | MMS, flow/float | [fix] Douglas-Peucker through a C++/Eigen twin (FMA-exact distances); Eigen project_line_on_line shim also lands | DP_SHIM (ON); MMS_PROJECT_SHIM (default not stated); probes: CPDUMP_LID +3 | Majora post-MMS region pieces exact 2,170 -> 2,749 of 3,375 |
| R806b | 2026-09-05 | e1f90f4 | MMS | [fix] Region 0's unpainted remainder built as native: per-extruder differences, opening, union_ex (PrintObjectSlice.cpp:948-976) | MMSEG_REMAINDER_SEQ (ON) | Majora post-MMS region pieces exact 2,749 -> 3,248 of 3,375 |
| R806c | 2026-09-05 | 2e597cd | MMS, slicing | [fix] lslices rebuilt after segmentation (PrintObjectSlice.cpp:1355) through the exact offset_ex twin (ClipperUtils.hpp:372) | LSLICES_POST_MMS (ON); LSLICES_OFFSET_EX (ON) | Majora post-MMS lslices byte-exact 0 -> 591 of 656 layers |
| R807 | 2026-09-05 | ecafed5 | gcode emission | [fix] LINE_WIDTH before speed, wipe path and distance, Auto Lift, layer-0 tool order (ToolOrdering.cpp:522-603), raw-slice backup | LW_BEFORE_SPEED (ON); LIFT_TYPE_AUTO_CPP (ON, was opt-in); FIRST_LAYER_TOOL_ORDER (ON); +3 | Majora 1,505,160 -> 1,718,733; arachne 134,307 -> 138,771; benchy -> 121,714 |
| R808 | 2026-09-05 | 2439d16 | wipe tower | [port] Prime tower writer (WipeTower.cpp:730), wipe (:3958), extruder offset, speeds, deretract format; template doubles as %g | TOWER_WRITER_CPP (ON); TOWER_WIPE_CPP (ON); TOWER_EXTRUDER_OFFSET (ON); +4 | Majora in-order 1,718,733 -> 1,787,698 (63.25% -> 66.06%); tower 66.8% -> 83.0% |

Gate lists shortened in the table: R807 also adds `WIPE_PATH_SSD`, `WIPE_DIST_CONFIG` and
`BACKUP_UNTYPED_SLICES`; R808 also adds `TOWER_XFORM_TC_RESET`, `TOWER_SPEEDS_CPP`,
`TOWER_DERETRACT_FMT` and `TEMPLATE_NUM_G6` (all ON). R693 moved seven gates to `opt_in_gate`:
`ARACHNE_TOP_ONE_WALL`, `ARACHNE_SIMPLIFY_SCALED`, `LINEWIDTH_BEFORE_SPEED`, `WIPE_TOWER_FORCE_TAGS`,
`WRITER_POS_CLEAR_CPP`, `LIFT_TYPE_AUTO_CPP` and `TOWER_PRE_RETRACT`.

## Open threads at R808

What the latest rounds leave open, with the rounds that say so. "No lever named" means the commit
reports the residual without proposing the next step.

### Majora

- **Ordering loss.** 295,093 content-matched lines are still out of order after R807 (was 446,985).
  No next lever is named. Earlier diagnoses that were not reported closed: island and loop visit
  order (R709, R710: ceiling +11,911 in-order on Benchy classic; R776: order differs on 108 Benchy
  arachne layers with a stable permutation), per-path role marking in the concentric internal
  filler that alternates bridge and solid runs (R778), and seam start point and direction on outer
  walls (R653; narrowed by R753, R754, R757).
- **Prime tower remainder.** 83.0% in-order; about 5,100 of 67,449 tower extrusion lines are still
  not content-matched (R808). Older tower items never reported closed: the finish-layer block
  structure (C++ emits perimeter and fill as separate tool-change results, 932 blocks against 656;
  R613–R619), the two-depth problem behind `TOWER_TIMELAPSE_DEPTH` (R509, R510, R616), the plan-depth
  rewrite and `add_solid_flag` pass (R617, R618), the tool-change count of 2,721 against C++'s 2,723
  (R531 through R650; R807's first-layer tool order may bear on it, but no commit says so), the
  avoid-crossing-perimeters detour travel before the tower (R649, not implemented), and the M204
  deficit (R651).
- **MMS (multi-material segmentation) drift.** 127 of 3,375 post-MMS region pieces are not exact
  and 31 region-0 pieces diverge (R806b); 65 of 656 layers have non-exact post-MMS lslices,
  including layer 0, which goes through the native first-layer regrouping and elephant-foot path
  (R806c). Segmentation pieces are exact on 27 of 31 probes (R805b).
- **Wall inputs.** Only 19,998 of 41,145 WallToolPaths calls receive byte-exact input (R805b). The
  remaining fork is downstream of slicing: MMS region pieces and the perimeter-generator surface
  grouping, 16,741 surfaces in Rust against 14,729 native while region surfaces match (R804a).
- **Floating vertical shell (FVS).** Rust's detection over-fired (46,374 floating-marked
  thicklines against 347 native, R797) because native's self-intersection test reads freed memory
  and so switches detection effectively off (R801). `FVS_FORCE_NO_DETECT` mirrors that observable
  behaviour; the stratum is 33.0% in-order. Exact parity here is not reproducible. Also open: Rust
  produces 16% more thicklines on equal call counts (R797).
- **Sparse infill and arcs.** A deficit of about 26k `G1` moves in sparse infill persists; the next
  probe named is the connector side (R792). Rust fits more arcs than native in every major stratum
  (R794); R796 clears the arc fitter and attributes it to micron-level input noise, parked. Native
  perimeter loops go through `simplify_loop`, which was not instrumented (R796).

### Arachne (Benchy arachne)

- 14 of 720 surfaces still differ in wall output; 821 wall-class and 450 fill-class WallToolPaths
  calls differ in their input outline (the chains ahead of WallToolPaths in the perimeter generator
  and the concentric fills); 14 of 1,431 common-input calls differ by plus or minus one unit on odd
  lines (R803).
- The cross-layer outer-wall speed smoother now runs for Arachne, but its per-node values still
  differ from native (R803e).
- Loop direction (rotation against reversal plus rotation) is unverified by the fingerprint sums
  (R743, R803).

### Benchy classic and shared

- Residual numeric drift in the medial-axis and thin-wall chain (`medial_axis_scaled`, thin walls
  to extrusion paths) and in gap-fill footprints (R759, R760); per-vertex micron drift on the
  layer-0 contour from the elephant-foot smoothing (R764).
- Seam: 223 loops (3,395 lines) differed only by seam at R754, before `SEAM_CONCAVE_CORNER` shipped
  in R757; no later census.
- Fill family on Benchy classic was the low scorer at R711–R723 (shared vertices: sparse 43.4%,
  internal solid 15.3%, FVS 4.4%, bridge 1.6%). The exact-clipper work of R725–R738 and R775–R792
  addressed the class; no later per-feature census is given.
- Bridges: no independent defect remains by R763; two residual micron-class cases (layer 142, and
  the layer-2 knife-edge case of R762).
- The geo clipper backend (a 1 micron grid) is a class-wide precision loss: 573 call sites not on
  the exact path at R726–R728. Dispatching to the exact ClipperLib path is the vehicle; a wider geo
  scale is too slow. Sites were converted piecemeal afterwards (R737, R738, R740, R765, R775, R792,
  R803c, R806c); no commit gives a remaining count.

### Structural debt named and not reported fixed

- The live fill path uses a divergent `group_fills` in `fill/mod.rs`; the faithful port in
  `fill/fill.rs` is unused and `Layer::make_fills` is unported (R714). Cross-region union of the
  no-overlap set (Fill.cpp:354) is unported (R715).
- Dead twins that can be read or called instead of the live code: `fill/fill.rs`, a second cooling
  buffer (removed in R756), the old `WipeTower::generate` path against the live `generate_new`
  (R479, R486, R620), `exporter.rs` wipe against the writer's retract (R606), a home-grown bridge
  PCA (R761).
- Unported: `discover_shell_for_perimeters` (only with `interlocking_beam`, R486);
  `is_support_necessary` and `TreeSupport::detect_overhangs` (R528, R529, declined);
  `spiral_flat_ironing` (R499); first-layer brim grouping on multi-volume objects (R675);
  avoid-crossing-perimeters router (R634, R649); modifier parts and unrecognised 3MF sub-objects are
  skipped at load (R703).
- Config: `retract_when_changing_layer` is mapped into the wrong field (R634, latent);
  `scripts/audit-config-defaults.py` covers numeric struct-literal defaults only, 235 of 402 options
  (R706, R707); the same mm-against-scaled unit trap remains in `gcode/conflict_checker.rs` (R803d,
  off the parity path).
- A multi-nozzle fixture that the C++ engine can slice does not exist, so the multi-nozzle port
  cannot be validated (R527).

### Closed questions (do not re-chase)

Each of these cost several rounds and was refuted by measurement.

- **Arachne width variety, R536–R591 and R655–R694.** The `; LINE_WIDTH:` tag deficit on Majora
  outer walls was chased through overhang grading, the beading strategies, propagation, centrality,
  the Voronoi graph, `collapse_small_edges`, the path splitter, quantisation and sort stability.
  All were measured faithful or irrelevant, apart from real fixes at R537, R541, R549, R558, R590,
  R691 and R706. C++ internal counters from R572–R577 included a speculative one-wall pass and are
  void (R580, R581). The real causes were upstream: MM segmentation fragmentation (R677, R678),
  negative volumes skipped at load (R701–R704), then exact offsets and flow truncation (R775) and
  exact overhang inputs (R803–R803e).
- **Prime tower, R475–R510.** Bounding-box and Y-shift readings were contaminated (R494, R495);
  tuning one tower component alone moves the total the wrong way because purge and fill cancel
  (R501, R506). The 2 mm Y offset is the extruder offset (R808).
- **Spiral lifts, R629–R640.** The object `G17` excess was attributed to one guard, then the
  overhang predicate, then the auto-lift sites; 85% of C++ spirals are eager lifts from
  instantly-applied retracts (R637). `z_hop_type` had never been read from config (R632).
- **Silhouette "rim", R516–R518.** A raster brush cliff in the metric, not a divergence.
- **Fill on Benchy classic, R711–R720.** Angle, spacing, phase, region, connector, filler
  assignment, defaults and classification were all cleared at tolerance; the cause was sub-micron
  boundary arithmetic, found only with exact integer fingerprints (R721–R725).
- **Painted-line endpoints, R783–R788.** FMA-fused lerps and truncation were both refuted; the
  cause was the f32 volume frame (`MMS_PAINT_FRAME`).
- **Performance suspects, R731–R736.** Per-line formatting, seam visibility raycasts, AABB tree
  builds and missing parallelism were each innocent; the costs were a lost parallel loop (R735), a
  single pathological layer (R736) and a single 5,628-vertex offset (R740). Caching the gate
  lookup alone is not a win (R484, R520); the environment snapshot is (R565).

## Gates not default at R808

As far as the commit messages say. A gate listed here is off unless set to `1`.

| Gate | Rounds | State | Why it is off |
|-|-|-|-|
| `ISLAND_FLATTEN` | R779, R780 | parked | Flattening fill collections per island regressed Majora by 72,481 in-order lines. The `no_sort` half shipped as `FILL_NO_SORT`. To flip: audit `no_sort` across all fill emission paths. |
| `NO_COLLECTION_PRESPEED` | R758a | parked | -51 on Benchy arachne; waits on the Arachne width-segmentation gap. |
| `LINEWIDTH_BEFORE_SPEED` | R654, R693, R708, R758a, R758b | parked | Majora -25,557 (R708), -16,153 (R758b). R807 ships `LW_BEFORE_SPEED` default ON for the same native ordering and does not say whether it replaces this gate. |
| `WIPE_TOWER_FORCE_TAGS` | R656, R693 | parked | +1,204 content against -2,117 in-order. |
| `FILL_NOOVERLAP_SAFETY` | R721, R723 | parked | Provably faithful (Fill.cpp:740) but net -64 in-order; something downstream compensates. |
| `MMS_PAINT_TRUNC` | R784 | opt-in | Its theory was refuted in R788. Dead. |
| `ARACHNE_SIMPLIFY_SCALED` | R672, R693, R740 | opt-in, broken | Feeds a scaled tolerance to a function that takes millimetres; empties outlines. Do not enable. |
| `ARACHNE_SIMPLIFY_FAITHFUL` | R696 | opt-in | Reachable but inert on the retention it targeted; never scored. |
| `MMSEG_DIFF_CLIB` | R698, R740 | opt-in | Inert on all four fixtures by output hash (R740). R765's `MMSEG_CLIB` covers the six segmentation sites. |
| `GEO_SCALE_COORDT`, `GEO_SCALE=<n>` | R726, R727 | instrument | Prices the geo-clipper precision class; 2.3x slice time. |
| `WRITER_POS_CLEAR_CPP` | R630, R693 | opt-in | Regressed Majora matched lines. Not re-scored after R807 flipped `LIFT_TYPE_AUTO_CPP`. |
| `TOWER_PRE_RETRACT` | R640, R693 | opt-in | Matched lines identical; undoes R638's in-tower spiral placement. |
| `TRAVEL_G1_WITH_Z` | R632 | opt-in | Not a port of anything in C++; costs 82 matched lines. |
| tower arc fitting (gate not named in the commit) | R628 | opt-in | Tower wall simplification loses lines; the tolerance probably does not transfer as a literal. |
| `WIPE_MULTIPATH_CPP` | R605, R612 | opt-in | Faithful path, net -10 lines. |
| `BRIDGE_NORMAL_CPP` | R595 | opt-in | Faithful, -4 lines on Majora, Benchy identical. |
| `TOWER_TIMELAPSE_DEPTH` | R509, R510, R616 | opt-in | Fixes the top band of the tower but regresses the tower total until the two depths are separated. |
| `SEAM_RAYCAST_SERIAL` | R735 | opt-in | Fallback to the serial raycast; not a parity gate. |
| `FVS_SELFX` | R799, R801 | unclear | Introduced as faithful ON in R799; R801 says the machinery "stays parked for a fixed native" and bypasses it with `FVS_FORCE_NO_DETECT`. |
| `GYROID_FSS`, `MMS_PROJECT_SHIM` | R791, R806 | default not stated | Both described as running; both score-neutral alone. |
| `TOWER_FILL_ONLY_TC` | R503, R506, R508 | unclear | R506 flips "the `TOWER_FILL_*` knobs" on without naming it; R508 treats `=0` as the non-default arm. |

Gates that were parked in this span and later shipped default ON, so their old "parked" notes no
longer apply: the tower fill set (`TOWER_SPARSE_GRID`, `TOWER_FILL_*`, `TOWER_WIPE_CONNECTOR`, R506),
`FILL_BRIDGE_ANGLE` (R600), `TOWER_ENTRY_WIPE_CPP` (R611), `MMSEG_OPENING` and `MMSEG_CLOSING`
(parked R557, ON R678), `CLIPPER_UNION_GEO` (parked R728 on slice time, ON R738),
`SEAM_CONCAVE_CORNER` (R754, ON R757), `FLOW_F32` (R760), `L0_EFC` (R764), `ARACHNE_TOP_ONE_WALL`
(parked R684, ON R774), `MMS_PAINT_FRAME` (R786, ON R788) and `LIFT_TYPE_AUTO_CPP` (opt-in R630, ON
R807).

Gates from R474–R477 (`ARACHNE_FILL_NO_OVERLAP`, `FVS_NO_OVERLAP_FAITHFUL`,
`WT_NOZZLE_CHANGE_WIDTH`, `TOWER_ROLE_RESET`, `TOWER_TRAVEL_TO_START`, `TOWER_XFORM_NO_RAW_ECHO`)
are named without a stated default; the commits report changed Majora output, which implies ON.

## Method rules and traps

Rules the rounds arrived at, usually after losing time to the opposite.

- **A parked verdict binds only the metric it was scored on.** `MMSEG_OPENING` was parked on
  wall-line IoU (R557) and shipped on in-order lines (R678); negative volumes were "near-inert" on
  silhouette (R515) and the whole hole deficit at line level (R701). Re-score parked gates when the
  headline metric changes (R677).
- **A locally faithful change can score worse** while a related count is still wrong (R654, R656,
  R721), and in-order is not monotone in faithfulness at low content density (R708). Majora in-order
  can fall while content rises (R725, R760).
- **Quote absolute matched lines.** Rates move with the denominator (R599, R605, R631, R647, R754).
- **Verify a gate's OFF arm by output hash.** Two arms with the same hash tested nothing (R693); an
  A/B that ignores the slicer's exit code hashes the previous run's file (R672, R673); building only
  the library crate does not rebuild the CLI binary (R709).
- **Census the artefact being judged**, per class and per site. A total moved by 2 while its
  components moved by 2,717 each way (R608–R610); stage totals hide single-item defects (R736, R740).
- **Cross-engine probe ratios.** Cumulative-prefix counters in a multithreaded pipeline are not
  reproducible point estimates (R591); matched block indices are not matched coverage (R689); C++
  runs a speculative one-wall Arachne pass that inflates its internal counters unless excluded (R580,
  R581, R684, R692); never divide one probe's numerator by another's denominator (R692). "A ratio is
  not an effect": price what share of the downstream quantity a term supplies first (R692).
- **Use exact integer fingerprints**, not area or bounding-box tolerances, to clear a stage (R721).
  Sum fingerprints are rotation-invariant and miss a ring-start rotation (R803).
- **Units.** The crate has two `SCALING_FACTOR` constants with reciprocal values (R557, R679); C++
  scales at 1e-5 mm per unit (R487, R547, R665). Geo helpers and `douglas_peucker` take millimetres,
  the `_scaled` exact helpers take scaled units; check the helper, not the parameter name (R737,
  R740, R769, R803d). Config defaults have carried radians for degrees and a fraction for a percent
  (R549, R550).
- **Config keys.** A key handled by one deserialisation path and missing from the other is a
  recurring bug class (R480, R481, R495, R509, R632, R641, R646, R647, R807). A blanket fallback
  between the two paths was measured negative; add keys one at a time (R481).
- **Ported is not wired.** Functions with zero callers, fields never read, counters never
  incremented and dead twin modules recur (R541, R558, R597, R602, R606, R614, R683, R756, R761). A
  source comment citing a C++ line range next to code that does not implement it is a lead (R645).
- **The C++ engine.** Not byte-reproducible; compare metrics, not hashes (R487, R547). The reference
  checkout is a submodule: instrument it only temporarily and rebuild the stock engine afterwards.
  Line numbers quoted before R486 may refer to a different copy of the sources.
- **Probes.** `getenv` in hot loops is not free (R565). A probe that prints nothing is a failed run,
  not a negative result (R515, R586). Thread-local "last line" attribution is unreliable under the
  thread pools (R767, R775). `G1`-only censuses are blind to arc fitting (R794). The `; LINE_WIDTH:`
  tag count is not a path count, and the `; LAYER_HEIGHT:` count is not the layer count (R676, R680).
- **Timing.** Interleave A/B runs, take the median, record host load (R523, R728, R735, R736).

## Inconsistencies in the record

Places where the commit messages disagree with each other. They are listed so that nobody treats one
side as settled.

- **Score chain breaks in the last rounds.** Majora "after" 1,217,995 (R803c) against "before"
  1,276,793 (R804a), with no Majora score in between; "after" 1,475,536 (R805b) against "before"
  1,505,160 (R807), with R806, R806b and R806c reporting no in-order score; 1,116,613 (R801) against
  1,116,654 (R803). Benchy arachne 134,299 (R803e) against 134,307 (R807).
- **Percentages on different bases.** Majora: 49.00% is a content rate at 1,056,793 in-order (R788),
  51.84% content at 1,116,613 (R801), then "44.85%" at 1,217,995 (R803c), 53.9% at 1,473,706 (R805)
  and 55.10% at 1,505,160 (R807). Benchy arachne: content 61.0% at 94,488 in-order (R775), then
  43.2% -> 50.6% for 97,790 -> 114,757 (R803).
- **`LINEWIDTH_BEFORE_SPEED` against `LW_BEFORE_SPEED`.** Parked four times on a Majora regression;
  R807 ships the same ordering under a new name without stating what removed the regression.
- **`FVS_SELFX`.** "Faithful ON" in R799, "stays parked" in R801.
- **R807 spiral lifts.** "39,603 -> ~8.8k native": the commit does not say whether 8.8k is the Rust
  result or only the native count.
- **Reused and oddly tagged rounds.** R772 has two commits (the second is a lockfile-only commit);
  `R579 Thread A` and `R592 fixup` are separate commits; R804a has no R804 or R804b.
- **Retraction chains.** Many rounds retract the previous one: R475 by R476; R490 by R491; R494 by
  R495; R498, R499, R501, R503 and R504 each by the next; R512 and R513 by R514; R516 and R517 by
  R518; R541–R545 in sequence; R551–R556 by R557; R558 by R559; R561 by R562; R563 by R564 and R565;
  R568 by R569; R570 by R571; R573 by R574; R572–R577 (C++ counters) by R580 and R581; R583 by R591;
  R597 by R598; R606 by R607; R608 and R609 by R610; R612 by R613; R622 by R628; R629 by R630–R637;
  R657 by R658; R671 by R672 by R673; R681 by R684 and R685; R692 by R693; R708 by R709; R724 by
  R725; R726 by R727; R728 by R729 by R730; R735 (Majora 0.99x) by R736; R777 by R778; R784 by R788.
  The table rows carry the later verdict where the digest could attribute it.
- **Small count drift.** Tool changes: 2,721, 2,722, 2,723 and 2,724 appear for the same quantities
  (R478, R493, R495, R506). Layer count 656 against 657 (R518, R523). C++ Majora body lines
  2,827,544 (R631) against 2,781,977 (R648). Native perimeter-surface totals 16,727 (R767) against
  16,720 (R770). R776's subject says 494/721, its body 494 of 720. R647's subject says 78,621
  spurious `M106` lines removed, its body a 73,165-line over-emission.
- **Baselines changed without a stated re-baseline.** Benchy and cube hashes change at R495 while
  described as byte-identical; the cube baseline drifted unnoticed until R600; Majora's changes at
  R638; R753 says the carried hash strings did not reproduce and re-records all four.
- **Slice time.** Majora C++ wall clock is 17.60 s in R563 and 15.53 s in R565; Benchy classic
  baseline is 3.43 s, 2.85 s and 2.72 s for the same arm across R726–R728; R727's timings are
  declared untrustworthy in the same commit.
- **Test suite wording** drifts between "eight suites unchanged", "seven green" and "eight, seven
  green" for the same state (R688–R730), and the commit that fixed the long-standing 25/26 failure is
  not identified.
- **R801** has a sentence with text missing (the object the EdgeGrid is built over).
- **R535** has a subject and no body.
