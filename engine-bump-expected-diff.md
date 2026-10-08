# Engine bump: expected diff

What the pin move to `OrcaSlicer v2.4.2` (`8500fcd`) and
`BambuStudio v02.08.02.61` (`926a719`) changes on the product's own call,
measured on the box.

Branch `lane-bump2`, head `bef5cb4`: PR #35's pushed head `821c658` plus this
lap's engine bump only. Package `~/bump-pkg-onto`.

Method: `~/rewrite-gates/gateA.sh run bumpgate2 <pkg> 1` and `... 3` on the
frozen inputs, both tiers against `runs/f1` with `compare.py f1 bumpgate2
--noise base1 base2`. 281 units: tier 1 (the Bambu corpus on the Bambu build)
141, tier 3 (the same corpus on the Orca build) 140.

`f1` predates both lanes' work, so the tier-3 figures below carry this lap's
engine bump *and* #35's conversions together. Section 0 separates this lap's
own change from the pin move.

A version bump is expected to change G-code everywhere. This file records what
the change *is*, so a later reviewer can tell the expected drift from a defect.

## 0. Outcomes

Every unit that ran kept its exit code on the Bambu build, and on the Orca
build the change is one-way — units `f1` refused now slice, and nothing that
sliced stopped:

| engine | units | rc 0 → 0 | refused → slices | still refused | slices → refused |
|-|-|-|-|-|-|
| tier 1 (Bambu) | 141 | 140 | 0 | 0 | 0 |
| tier 3 (Orca) | 140 | 8 | **71** | 62 | **0** |

The 71 tier-3 units are the Bambu projects #35's desktop-parity conversions
now carry across the `*_id` rename and the value encodings; `f1` refused them
for values this engine could not read. `13ab99508c843d4d.p1.o` — the H2D tower
unit that hung for ten hours on the previous branch, held alive by the
fork-in-handler deadlock #35 fixes — ends **rc=0 in 1579 s**, inside its
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

Measured on the 143 units whose footer carries both numbers.

| | units | median | min | max |
|-|-|-|-|-|
| model time | 143 | 0 s | -1130 s | +632 s |
| filament weight | 140 | 0.00 g | -0.48 g | +1.66 g |

The largest time moves are on the Orca build: `95b94b99835a2810.p1.o`
16034 s → 14904 s (-19 min) and its `.p2.o` 15827 s → 14775 s. The largest
gain is `15dcdfa8584738a0.p1.b` and `3d611b6090ed84cc.p1.b`, 2558 s →
3190 s (+10.5 min) with 2.59 g → 4.25 g, the same move on both units.

### `~/h2d-nomods.3mf`, as it is, `--slice 1`

A Snapmaker U1 project with four filaments, sliced by the Orca build with no
modifiers of any kind:

| | filament used | estimated printing time |
|-|-|-|
| this build | **569.33 g** | **1d17h43m46s** |
| official OrcaSlicer 2.4.2 | 564.76 g | 1d17h35m32s |

+4.57 g (+0.81 %) and +8m14s (+0.33 %), the drift section 3 above describes.

## 4. What does not change

- `--layout-plan` placements: 103 corpus units, both engines, identical
  placement or identical refusal, 0 differing.
- The five scripts on both engines: **302 PASS, 0 FAIL**.
- The export-3mf case on the real corpus unit `40fd25289c325186` (an X1
  Carbon project that states **no `filament_ids`**) — the defect #35's
  215fb0a fixes — exits 0 and writes a 6,686,096-byte project.
- SIGTERM 0.5 s and 1.5 s into the load of `44e597511c7d5abd`, both engines:
  the run is still live at the signal, exits **143** in 18–56 ms.

## 5. `--layout-plan` with a locked model

A `"locked": true` model is a real obstacle again. The Helio forks existed to
make the nester accept the items at all, and as a side effect a locked model
stopped being fixed in a bin: it was packed as a free item and its result
thrown away. Seeding bed 0 (the desktop's own value, `ModelArrange.cpp:98`)
restores the desktop behaviour; on Orca v2.4.2 the engine seeds itself
(`ModelArrange.cpp:25`).

`bump-locked-check.py`, a 40 mm locked square at the bed centre and eight free
20 mm cubes: the fork build places `f3` inside the locked square, this build
places nothing over it. The product sends no locked models, so its corpus
results are unaffected; this is the defect it worked around with a bed
cut-out.

## 6. Printer sweep

`sweep.sh` over every printer each binary lists, 20 mm cube, `--slice 1`,
against the `pr35` baseline (the current pins).

**Stopped before it finished**, at the owner's request, to free the box for
the next rebase — `bump-sweep2` rc=143 and its restart `bump-sweep2b` rc=143
are both that stop, not failures.

| engine | baseline | this build | common | new failures | rc changed | code changed |
|-|-|-|-|-|-|-|
| Bambu | 56 | 56 (complete) | 56 | 0 | 0 | 0 |
| Orca | 916 | 163 (partial) | 159 | 0 | 0 | 0 |

The Bambu sweep is complete and identical to the baseline. The Orca half is
partial: 163 of 1001 printers, 159 of them common with the baseline, with no
new failure, no exit-code change and one error-text change; four printers are
new to this build. `filament_printable` is unchanged on every common printer.
