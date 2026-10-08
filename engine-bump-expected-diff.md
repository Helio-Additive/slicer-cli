# Engine bump: expected diff

The engine bump moves the two pinned engines from the Helio forks to the
untouched upstream releases:

| engine | was | is |
|-|-|-|
| OrcaSlicer | `31f6803` (upstream `42cce53`, 2.4.0-alpha, plus one Helio arrange fix) | **`8500fcd`** = upstream **v2.4.2** |
| BambuStudio | `5873b5f` (Helio fork) | **`926a719`** = upstream **v02.08.02.61** |

The fork arrange commits go away with them. The front end seeds bed 0 before
arrange instead — the desktop's own value — and a locked model is a fixed
obstacle again rather than a movable one.

**This is the candidate for the overseer's test: branch `lane-bump4` at
`78bad3a`**, which is the bump `6009651` stacked on #35's `e99b4bb`. Its own
tree is the pins, the removal of the two override-layer `Print.cpp` copies, the
front-end adjustments the new engine needs, and the doc and test updates. The
16 files it touches are listed at the end of this file.

## Where each number comes from

Every claim below cites the run that produced it. The gate, sweep and H2D runs
were made earlier in the lap on `~/bump-pkg-m3` (head `8ed665f`); the test
totals are from the candidate itself. **The engine pins are the same upstream
commits in both**, and only the front end differs between them, so the
engine-level drift those runs establish carries over unchanged.

| what it establishes | run | package and head |
|-|-|-|
| tier 1 and tier 3 outcomes, settings, print time, filament | `gateA.sh run bumpm3` (tiers 1 and 3), then `compare.py f1 bumpm3 --noise base1 base2` | `~/bump-pkg-m3` (head `8ed665f`) against `runs/f1`, noise `base1`/`base2` |
| the printer sweep, the 14 refusals, Raise3D | `~/bedsweep/sweep.sh bumpm3` | `~/bump-pkg-m3` against the `pr35` baseline |
| the official verdict on those 14 printers | `bump-official-own2.sh`: the official release AppImage, driven by its own presets | official OrcaSlicer v2.4.2 |
| H2D against the official, the locked arrange, the layout corpus | jobs `bump-m3-h2d` and `bump-m3-rest2` | `~/bump-pkg-m3` against `~/bump-pkg-old` |
| the candidate's own test totals | job `bump-m8-cases` | `~/bump-pkg-m8` (head `78bad3a`) |

The gate compares against `runs/f1`, a run that predates both this bump and
#35's conversions, so the tier-3 figures carry the two together; section 0
separates them. The first tier-3 pass filled the box's disk at 17:44 — 13
`No space left on device` lines — so two unit dirs were never created and two
lost their files; all four were re-run on the same package (`bump-m3-recheck`,
`bump-m3-gate3`), and the figures below are after those re-runs.

A version bump is expected to change G-code everywhere. This file records what
the change *is*, so a reviewer can tell expected drift from a defect.

## 0. Outcomes

Run: `gateA.sh run bumpm3` tiers 1 and 3 against `runs/f1`, judged by
`compare.py f1 bumpm3 --noise base1 base2`.

Every unit that ran kept its exit code on the Bambu build, and on the Orca
build the change is one-way — units `f1` refused now slice, and nothing that
sliced stopped:

| engine | units | rc 0 → 0 | refused → slices | still refused | slices → refused |
|-|-|-|-|-|-|
| tier 1 (Bambu) | 140 | **140** | 0 | 0 | 0 |
| tier 3 (Orca) | 141 | 8 | **71** | 62 | **0** |

The 71 tier-3 units are the Bambu projects #35's desktop-parity conversions
now carry across the `*_id` rename and the value encodings; `f1` refused them
for values this engine could not read. `13ab99508c843d4d.p1.o` — the H2D tower
unit that hung for ten hours on the previous branch, held alive by the
fork-in-handler deadlock #35 fixes — ends **rc=0 in 1292 s**, inside its
2400 s timeout.

## 1. Filament keys and the reset

`OrcaSlicer v2.4.2` numbers every feature filament from 0, where 0 is
"Default" (`outer_wall_filament_id`, `PrintConfig.cpp` 5011-5018, min 0,
default 0), and `handle_legacy` renames the old 1-based `wall_filament` /
`sparse_infill_filament` / `solid_infill_filament` to their `*_id` spelling,
mapping a legacy `"1"` to `"0"` (`PrintConfig.cpp` 8038-8058).

That makes half of #35's filament-index reset redundant — a Bambu Studio
file's own 0 is already a value this engine reads — so the front end keeps
only what the desktop still does on load (both cited at v2.4.2): an index
outside `[0, N]` falls back to 0 for the six feature filaments
(`PresetBundle.cpp` 4127-4136), and `support_filament` /
`support_interface_filament` / `wipe_tower_filament` clamp to `[0, N]`
(`PresetBundle.cpp` 4119-4125).

`PartPlate::get_extruders_under_cli` is carried to the new names the same way
(`PartPlate.cpp` 1671-1681 and 1744-1800).

## 2. Settings

`config_md5` changes on 160 units.

### Tier 1 (Bambu build): 12 keys

| key | units | value |
|-|-|-|
| `ams_filament_load_time_ams`, `ams_filament_load_time_ams_lite`, `ams_filament_load_time_n3f_s`, `ams_filament_unload_time_ams`, `ams_filament_unload_time_ams_lite`, `ams_filament_unload_time_n3f_s` | 140 | new key, `0` |
| `default_ams_type` | 140 | new key, `-1` |
| `bed_heat_soak_area` | 140 | new key, empty |
| `extruder_nozzle_stats_new` | 131 | new key, empty |
| `filament_map`, `filament_map_2`, `filament_nozzle_map` | 2 | value order changes, e.g. `1,1,1,1,1,2,1,2,1,1` → `1,1,1,1,2,1,1,2,1,1` |

The nine new keys are new BambuStudio options at their defaults; no existing
key changes value except the three filament maps on 2 units.

### Tier 3 (Orca build)

The rename, on the 8 units that name one:

| old key (gone) | new key (appears) |
|-|-|
| `wall_filament` = `1` | `outer_wall_filament_id` = `0` |
| `sparse_infill_filament` = `1` | `sparse_infill_filament_id` = `0` |
| `solid_infill_filament` = `1` | `internal_solid_filament_id` = `0` |
| | `inner_wall_filament_id`, `top_surface_filament_id`, `bottom_surface_filament_id` = `0` |

Plus the v2.4.2 options at their defaults on 8 units — `bridge_line_width`
`100%`, `chamber_minimal_temperature` `0`, `lightning_overhang_angle`,
`lightning_prune_angle`, `lightning_straightening_angle` `45`,
`parallel_printheads_bed_exclude_areas`, `parallel_printheads_count` `1`,
`relative_bridge_angle` `0`, `support_parallel_printheads` `0`, `use_3mf`
`0` — and, on the 71 units that now slice, every vector the Orca alignment
sizes to the roster (`activate_chamber_temp_control 0` → `0,0,0,0`,
`additional_fan_full_speed_layer 0` → `0,0,0,0`, `filament_change_length`
`10` → `10,10,10,10`, …), the desktop's own shape.

One existing value changes: `slowdown_for_curled_perimeters` `1` → `0` on 7
units (the engine's default for it changed between the pins).

## 3. Print time and filament

Run: the same gate run; the numbers come from each unit's own `digest.json`.

Measured on the 143 units whose footer carries both numbers.

| | units | median | min | max |
|-|-|-|-|-|
| model time | 143 | 0 s | -1130 s | +632 s |
| filament weight | 140 | 0.00 g | -0.56 g | +1.66 g |

The largest time moves are on the Orca build: `95b94b99835a2810.p1.o`
16034 s → 14904 s (-19 min) and its `.p2.o` 15827 s → 14775 s. The largest
gain is `15dcdfa8584738a0.p1.b` and `3d611b6090ed84cc.p1.b`, 2558 s →
3190 s (+10.5 min) with 2.59 g → 4.25 g, the same move on both units.

### `~/h2d-nomods.3mf`, as it is, `--slice 1`

A Snapmaker U1 project with four filaments, sliced by the Orca build with no
modifiers of any kind, and by the official OrcaSlicer v2.4.2 on the same file:

| | filament used | estimated printing time | G-code |
|-|-|-|-|
| this build | **565.92 g** | **1d 17h 37m 46s** | 376,203,089 B |
| official OrcaSlicer 2.4.2 | 564.80 g | 1d 17h 33m 45s | 375,654,060 B |

+1.12 g (+0.198 %), +4m01s (+0.16 %) and +0.146 % of G-code — the same order
as the per-unit drift section 3 describes, on a real four-filament project.

## 4. What does not change

- `--layout-plan` placements: 103 corpus units, both engines, identical
  placement or identical refusal, 0 differing.
- The product's own checks. On the candidate itself (`~/bump-pkg-m8`, head
  `78bad3a`, job `bump-m8-cases`, `TMPDIR` on the home filesystem):
  `test-complete-slice-path.sh` 239, `test_refusal_sentences.sh` 79,
  `test-pr35-findings.sh` 42, `test-inherits-inputs.sh` 11 — **371 PASS,
  0 FAIL**, plus one documented skip (`F7 orca`: OrcaSlicer orients without
  the process settings). The wider set on the earlier package (job
  `bump-m3-cases`, head `8ed665f`) was **337 PASS, 0 FAIL** across six
  scripts, the U1 end-to-end, the export-3mf case and the SIGTERM pair. The
  four suites are re-run on #35's final base before this goes up.
- The export-3mf case on the real corpus unit `40fd25289c325186` (an X1
  Carbon project that states **no `filament_ids`**) — the defect #35's
  215fb0a fixes — exits 0 and writes a ~6.69 MB project (6,686,099 /
  6,686,200 / 6,686,319 bytes on three runs; the project carries timestamps).
- SIGTERM 0.5 s and 1.5 s into the load of `44e597511c7d5abd`, both engines:
  the run is still live at the signal, exits **143** in 19–80 ms.

## 5. `--layout-plan` with a locked model

A `"locked": true` model is a real obstacle again. The Helio forks existed to
make the nester accept the items at all, and as a side effect a locked model
stopped being fixed in a bin: it was packed as a free item and its result
thrown away. Seeding bed 0 (the desktop's own value, `ModelArrange.cpp:98`)
restores the desktop behaviour; on Orca v2.4.2 the engine seeds itself
(`ModelArrange.cpp:25`).

`bump-locked-check.py` — a 40 mm locked square at the bed centre and eight
free 20 mm cubes, `oldpins` against this build:

| engine | `oldpins` places over the locked square | this build | reserves after the bump |
|-|-|-|-|
| Bambu | `f3` | none | yes |
| Orca | none | none | already correct |

The Bambu pair is the defect: the fork build packed `f3` over the locked
square, this build reserves it. Orca's old build already placed nothing over
it, so the same check reports no change on that engine. The product sends no
locked models, so its corpus results are unaffected; this is the defect it
worked around with a bed cut-out.

## 6. Printer sweep

Run: `~/bedsweep/sweep.sh bumpm3` against the `pr35` baseline — every printer
each binary lists, a 20 mm cube, `--slice 1`. This run completed on both
engines (`bump-m3-sweep`, rc=0): Bambu 56/56, Orca 1001/1001.

| engine | baseline | this build | common | new printers | rc changed |
|-|-|-|-|-|-|
| Bambu | 56 | 56 | 56 | 0 | 0 |
| Orca | 916 | 1001 | 897 | 104 | 3 |

Three Orca printers change exit code, all common with the baseline:

| printer | pr35 | this build | cause |
|-|-|-|-|
| `Creality K2 0.4 nozzle` | 0 | 238 | `retraction_distances_when_cut: 30` not in `[10,18]` |
| `Snapmaker U1 (0.4+0.6 nozzle)` | 0 | 238 | `bridge_line_width` > nozzle `0.6` |
| `Raise3D Pro3 0.4 nozzle (Dual)` | 139 | **0** | now slices (crashed on the baseline) |

Twelve printers new in Orca 2.4.2 also refuse 238: `Anycubic Kobra S1 Max`
0.25/0.4/0.6/0.8, `Creality SPARKX i7` 0.2/0.4/0.6/0.8, `SeeMeCNC BOSSdelta
500 0521` 0.4/0.5/0.7/1.0. `filament_printable` is unchanged on every common
printer, and one error text changed.

### Known upstream: all 14 refusals are the official's own

Every one of those 14 printers was run on the **official OrcaSlicer v2.4.2
Linux binary** (the release AppImage, extracted) with the same input — the
same cube, `--slice 1` — driven by its **own** machine/process/filament
presets, values verbatim (only `type`, `from: system` and `name` normalized
for the loader; the vendor files disagree on `System` vs `system`).

| printer | official | this build | official's own message |
|-|-|-|-|
| `Creality K2 0.4 nozzle` | **238** | 238 | `retraction_distances_when_cut: 30 not in range [10,18]` |
| `Snapmaker U1 (0.4+0.6 nozzle)` | **238** | 238 | `bridge_line_width: Bridge line width must not exceed nozzle diameter: 0.600000` |
| `Anycubic Kobra S1 Max` 0.25/0.4/0.6/0.8 | **238** ×4 | 238 | `invalid parameter value(s) included in the 3mf file` |
| `Creality SPARKX i7` 0.2/0.4/0.6/0.8 | **238** ×4 | 238 | `invalid parameter value(s) included in the 3mf file` |
| `SeeMeCNC BOSSdelta 500 0521` 0.4/0.5/0.7/1.0 | **238** ×4 | 238 | `invalid parameter value(s) included in the 3mf file` |

The offending values are the official's own preset data —
`retraction_distances_when_cut` is `30` on `Creality K2 0.4 nozzle`, `28` on
`Creality SPARKX i7 0.4 nozzle` and `0` on `Anycubic Kobra S1 Max 0.4 nozzle`
— so these are upstream preset values meeting upstream validation, not a
defect in this build. Our build refuses them for the same reason, with the
same code; nothing to port and nothing to fix.

## Appendix: the 71 tier-3 units that stop refusing

`f1` refuses `rc=1`, this build slices `rc=0`. The other 70 tier-3 units keep
their outcome: 62 refuse in both, 8 slice in both. Nothing slices in `f1` and
refuses here, on either tier.

```
0668e4f26885db61.p1 0ac8987b7752cad5.p1 0bd9ebcc662ed7d1.p1 0c146c45f6339874.p1
10f3db76169aaf66.p1 11baf75010b7bb8e.p1 133b2b96cbce86c0.p1 13ab99508c843d4d.p1
16074018fca42544.p1 164107857dc07674.p1 18fafc72af617173.p1 2120be8346a4d0c2.p1
269148b2b3d630ee.p1 29dca2d58a528eec.p1 2bcaf84a1f079eee.p1 38df8a7cb3ae2564.p1
4075dc2e8666b12b.p1 40fd25289c325186.p1 447ce69e8b90953b.p1 44e597511c7d5abd.p1
457bd9a6e032b36e.p1 4e7f97c1c867f8d1.p1 59e0a82c976257c6.p1 63e68e1f94b77f1f.p1
75f6cd969f1cb638.p1 7685e5d1e03aba75.p1 7685e5d1e03aba75.p2 7685e5d1e03aba75.p3
7685e5d1e03aba75.p4 7685e5d1e03aba75.p5 7685e5d1e03aba75.p6 7685e5d1e03aba75.p7
79e8a7240a0db89e.p1 83c349c1fc91d352.p1 84d898b1c175a7d7.p1 8d04ca592bf68527.p1
8e88573327bc80f8.p1 921c764ecb14c980.p1 92d5b8580685a566.p1 96b4ca075a6c2946.p1
994466ad27b144c6.p1 9b9488dd40d5d18a.p1 9e3304518440373d.p1 9ee725aac7c3a0d6.p1
abb0b218b93e0c1a.p1 aced112fffd85832.p3 ae90ff769d9dcacd.p3 b2a4e712c23fb3ff.p1
ba87d1af5cab9bd2.p1 bcaebec4528224a9.p1 c21a41f81cab52cb.p1 c35f493ce7b336c6.p1
c5178c79a385baa9.p1 c64bc16caac84404.p1 c7989cf82523da7e.p1 c9615f3c53fde3e2.p1
cc782a63669ebb70.p1 cc782a63669ebb70.p2 d750095f6f477b5a.p1 dcc02d7b07a20fdd.p1
e0c2a7603ed2e27d.p1 e203da8f17a56f73.p1 e48995025625eaf2.p1 e79f9ef79027dd9e.p1
ea3c917387914d44.p1 eaa9ec91992b5133.p1 eef459ac860086b3.p1 f0151b84da3265ea.p1
f3d18b4656b860d0.p1 f77de0d66c59506a.p1 fe4a96acf7e856a3.p1
```

Tier 1 has no outcome change at all: 140 units, `rc=0` under `f1` and here.

## The bump's own tree

`e99b4bb..78bad3a` is two commits — `6009651` (the bump) and `78bad3a` (this
file) — and touches 16 files:

| file | what it does, and why |
|-|-|
| `.gitmodules`, `references/OrcaSlicer`, `references/BambuStudio` | the pins: upstream v2.4.2 and v02.08.02.61, both untouched upstream commits |
| `libslic3r/orcaslicer/libslic3r/Print.cpp`, `libslic3r/bambustudio/libslic3r/Print.cpp` | **deleted** — the two override-layer copies. The filament-vector sizing they carried is the desktop's own rule (`Preset::normalize`, `Preset.cpp` 444-486), which reaches it without them; the Orca path now does the same in the front end |
| `main.cpp` | the front-end adjustments the new engine needs: the `*_id` rename and 0-based feature-filament indices in `arrange_plate_extruders` and the three `slice_one_plate` sites, the filament-index reset, `normalize_filament_config_vectors` just before `Print::apply`, and bed-0 seeding for the legacy `--layout` path |
| `layout_plan.cpp`, `arrange_harness.cpp` | bed-0 seeding before arrange, on both item lists |
| `CMakeLists.txt`, `ci/vcpkg-overlays/cgal/portfile.cmake`, `install_deps.sh` | version strings only (`v02.08.01.55` → `v02.08.02.61`) |
| `README.md`, `docs/building.md` | the version table and the build notes |
| `tests/test-complete-slice-path.sh` | the cases the new engine changes |
| `libslic3r/bambustudio/libslic3r/Fill/FillFloatingConcentric.cpp` | **kept as an override** (the owner-approved PR #25 fix) — upstream carries none of these fixes at any tag through v02.08.04.61 and has not touched the file since 2025-05-17, so it stays a copy of the v02.08.02.61 file plus its fixes, and keeps the file's original CRLF line endings. Its diff here is exactly four lines: the provenance header naming the upstream commit it copies |
| `engine-bump-expected-diff.md` | this file |

