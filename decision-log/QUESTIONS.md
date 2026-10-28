# Questions for you



### From helper frontend
# frontend helper: questions for the user

1. **Temperature-mix refusal on the Snapmaker build.** Snapmaker's CLI always refuses a high+low temperature filament mix (Snapmaker_Orca.cpp 5000-5018). The desktop app only warns. Provisional answer used: refuse, as Snapmaker's CLI does; `--allow-mix-temp` turns it off.
2. **Coloured OBJ in an assemble list.** Snapmaker's CLI ignores OBJ colours there (Snapmaker_Orca.cpp 847-857). Provisional answer used: same, so the OBJ prints in its one listed filament.
3. **Which 3MF projects get rebased onto presets.** The fork's loader flags both Bambu Studio and Snapmaker Orca projects as "BBL". Provisional answer used: rebase only `BambuStudio-` projects, as the Orca build does, and let Snapmaker Orca projects keep their own settings.
4. **Version gate for Snapmaker Orca 3MF files (not changed, needs a decision).** `engine_file_version()` and `file_version_verdict()` (main.cpp ~2097-2163) only recognise the `BambuStudio-`/`OrcaSlicer-` makers. On the Snapmaker build a `Snapmaker_Orca-2.4.1` project therefore counts as "not Orca-made". It is compared with SLIC3R_VERSION 01.10.01.50, so it is only warned about and never refused. Snapmaker's own CLI compares the file version with SLIC3R_VERSION and refuses a major mismatch (Snapmaker_Orca.cpp 1431-1436), which in practice refuses its own 2.x files unless `--allow-newer-file`. Left as is; please decide the rule.

### From helper presets
# Helper "presets" questions

### Q1. Filament flow variants in a merged config: the desktop's layout or the fork CLI's?
Time: 2026-10-28 21:18
The fork's own CLI writes one value per filament (index 0 of each preset), dropping a filament's high_flow values; the desktop keeps every declared variant in per-filament segments plus filament_flow_step_size. Both slice the same standard values; only the config (and the G-code's settings block) differs.
Provisional answer used: the desktop's layout (decision "Filament merge: the fork's composed flow-variant layout").

### Q2. Plate type when the printer's default plate is not in its list
Time: 2026-10-28 21:18
The fork's desktop slices on the printer's default plate even when its plate list does not show it (it only moves the list's selection), except the U1 clamp to Textured PEI Plate. Upstream Orca moved to the list's first plate.
Provisional answer used: the fork's rule (decision "Plate type: the fork's get_default_bed_type, U1 clamp").

## Notes for the coordinator (not implemented)

### N1. renamed_from is not honoured by our preset lookup
The fork's U1 printers name a default process that no longer exists by that name: "Snapmaker U1 (0.4 nozzle).json" has default_print_profile "0.20 Standard @Snapmaker U1 (0.4 nozzle)", and the process is now "0.20mm Standard @Snapmaker U1 (0.4 nozzle)" with renamed_from "0.20 Standard @...;0.20 Strength @..." (15 Snapmaker process files carry renamed_from).
- The fork honours renamed_from in PresetCollection::find_preset_renamed / get_preset_name_renamed (Preset.cpp 2740-2745, 3396-3405): for a preset's "inherits" parent (Preset.cpp 2655-2668), for a project's preset name in load_external_preset (Preset.cpp 2110-2120), and in find_preset2 (2785-2790). The loader also adds each "@" preset's name minus "@" as an implicit renamed_from (PresetBundle.cpp 3670-3680).
- It does NOT honour it where the printer's default process is picked: update_compatible's PreferedProfileMatch compares preset.name == default_print_profile exactly (PresetBundle.cpp 4140-4185), and select_preset_by_name (Preset.cpp 3232-3265) falls back to the first visible preset. So in the fork's desktop the stale "0.20 Standard @..." default never matches by name; the pick comes from alias / layer-height scoring or the first visible preset.
- Our code: desktop_presets.cpp preset_file_in_vendor / find_system_preset / find_system_process match the vendor list's exact "name" only, so (1) a 3MF or --process-preset naming an old name ("0.20 Standard @Snapmaker U1 (0.4 nozzle)") is not found, where the fork's load_external_preset would resolve it; (2) desktop_printer_switch_process's declared_default never matches, which agrees with the fork's update_compatible. Whoever owns the named-preset path (main.cpp) should decide how the U1's default process is picked when only a printer is given, and whether old names should resolve through renamed_from (and the implicit "name without @" alias).

### N2. filament_volume_type / nozzle_volume_type
The desktop's full config always carries filament_volume_type (one per filament, PresetBundle.cpp 195-197) and the project nozzle_volume_type; the merge here does not add them. get_config_idx treats a missing filament_volume_type as standard, so slicing is unaffected; the G-code's settings block may differ. On the Snapmaker build ProjectFacts::current_nozzle_volume_type holds the fork's FilamentVolumeType values (0 standard, 1 high_flow), not upstream NozzleVolumeType; main.cpp reads it.

### N3. Fork CLI quirk not copied
Snapmaker_Orca.cpp:2848 calls get_min_flush_volumes with filament_diameter's size only; ours sizes by the colour count too (an existing deliberate difference), unchanged.

### From helper u1-tests
# u1-tests: questions for the user

1. Snapmaker Orca 2.4.1's own CLI refuses a project tagged "Snapmaker_Orca-2.4.1" (its own files) unless
   --allow-newer-file / SNAPMAKER_ORCA_ALLOW_NEWER_FILE is given: it compares major 2 with its SLIC3R_VERSION
   01.10.01.50 (Snapmaker_Orca.cpp 1431-1436). Our snapmaker build compares with 2.4.1 and would accept it.
   Should our build match the official refusal, or keep accepting? (Time: 2026-10-28 21:22)
   Provisional answer used: the test run slices the project both with and without --allow-newer-file, so
   either behaviour is visible; nothing in the source was changed.

### From helper u1-tests

2. (2026-10-28 21:34) Confirmed by running: Snapmaker's CLI refuses two_plate_snapmaker.3mf (rc 232, "File Version 2.4.0.1 not
   supported by current cli version 01.10.01.50"); our build-snapmaker slices it. Same question as 1.

### From helper snap-fixes
# snap-fixes: questions for the user

### Q1. U1 default process: Snapmaker's intent or the desktop's literal code?
Time: 2026-10-28 21:58
The fork's desktop matches default_print_profile by exact name, so on a fresh install the U1's stale "0.20 Standard @..." never matches and the desktop's code would select the first compatible process, "0.08mm Standard @Snapmaker U1 (0.4 nozzle)". The brief asked to resolve renamed_from.
Provisional answer used: "0.20mm Standard @Snapmaker U1 (0.4 nozzle)" (Snapmaker's intent), reported in defaults_replaced as a renamed_from match.

### Q2. U1 default filament
Time: 2026-10-28 21:58
The U1's stated default "Snapmaker PLA" does not suit it; what the desktop takes next depends on which filaments the setup wizard installed. Candidates: "Generic PLA" (vendor Generic, brief's rule), "Snapmaker PLA SnapSpeed @U1" (Snapmaker's SM_DEFAULT_FILAMENT, used by the wizard-cancel path only for an already-active printer), "AliZ PLA @System" (first compatible PLA if every filament were visible).
Provisional answer used: "Generic PLA".

### Q3. engine = "snapmaker" in result.json and the other JSON documents
Time: 2026-10-28 21:58
docs/cli-reference.md documents `engine` as `bambustudio` or `orcaslicer`. The Snapmaker build now writes "snapmaker" (and "snapmaker" in layout capabilities / --layout-plan, which accepts it as input on that build). The doc owner should add the value.
Provisional answer used: "snapmaker".

### Q4. Default tower for plain (non-assemble) runs
Time: 2026-10-28 21:58
For an STL or 3MF run without an assemble list the Snapmaker build keeps the engine default wipe_tower_x/y 15/220 (as Snapmaker's CLI does); the desktop sets (13, plate depth/2) when it creates a plate in GUI mode. It only matters when such a run prints a tower (e.g. smooth timelapse, or several filaments on an STL plate).
Provisional answer used: unchanged (15/220).

### Q5. Mixed filament
Time: 2026-10-28 21:58
Our output keeps the prime tower and the re-serialised mixed_filament_definitions, as the desktop's double apply and load_config_model do; Snapmaker's CLI applies once and drops the tower (607 vs 1810 mm). Confirm with the desktop app if possible.
Provisional answer used: the desktop's behaviour (no change).
Update 2026-10-29 07:15 (D-163): 20 of 21 colour-mixing tests are identical to Snapmaker's CLI. What is left is only the tower when a mixed colour is the only filament on the plate; Snapmaker-saved projects are not re-serialised.

### From the independent review (coordinator summary)
- **D-137, U1 default process.** Read literally, the fork's desktop code picks "0.08mm Standard" on a fresh U1, because the printer's default name "0.20 Standard…" now exists only as a `renamed_from` alias. Our build picks "0.20mm Standard" (Snapmaker's evident intent). This departs from the "closest to the desktop" rule because the coordinator's brief told the helper to resolve `renamed_from`. **Provisional.** Which do you want?
- **D-138, U1 default filament.** "Generic PLA" comes from a fallback heuristic; the desktop's pick depends on what its setup wizard installed. **Provisional.**
- **D-111 vs D-142.** A Snapmaker-saved project is not rebased onto system presets (it follows Snapmaker's CLI). A Bambu-tagged project is rebased (it follows the desktop). The visible effect so far: whether `mixed_filament_definitions` is re-serialised. **Provisional.**
- **D-113.** `--allow-mix-temp` keeps the Orca build's meaning. Snapmaker's CLI always refuses a high/low temperature mix; its desktop only warns.

### From helper package
# Questions: helper "package"

1. (2026-10-28 22:13) **Packaged Bambu/Orca programs delete a running slicer_cli-snapmaker's temp folder.** (needs a main.cpp change that touches the Bambu and Orca builds: coordinator/user)
   Seen: pkg slicer_cli-snapmaker --list-presets --printer "Snapmaker U1 (0.4 nozzle)" returned rc 255
   {"error": "Loading the system presets failed: boost::filesystem::create_directory: No such file or directory … /T/slicer_cli_list-…/user"}
   every time while tests/test-complete-slice-path.sh was running the packaged slicer_cli / slicer_cli-orcaslicer in the same TMPDIR.
   Same binary, same command: rc 0 with a private TMPDIR, and rc 0 from build-snapmaker/ (where it runs as "slicer_cli").
   Cause (read from code): the stale-folder sweep keeps a folder only while its owner pid runs a program named by is_engine_binary_name
   (main.cpp ~7270-7285). The Snapmaker build lists slicer_cli-snapmaker there, the Bambu and Orca builds do not, so their sweep treats a live
   Snapmaker run's slicer_cli_* folder as orphaned and removes it. This is the bug the comment above staging_owner_alive describes for
   bambu vs orca, now for the third program.
   Fix needed: add "slicer_cli-snapmaker" to is_engine_binary_name on ALL builds (outside the #ifdef ENGINE_SNAPMAKER), which changes
   the Bambu and Orca builds (HELPER-RULES rule 2), so I did not ask snap-fixes to do it on their own.
   Provisional answer used: none in code; the package is built as is and the checks were re-run with no other slicer_cli program running.
2. (2026-10-28 22:13) Snapmaker build CGAL: keep 5.6.3 (current) or switch to 5.4 like the fork's own deps (configure.sh: SNAPMAKER_CGAL=5.4, then a full rebuild of build-snapmaker)?
   Provisional answer used: keep 5.6.3.
3. (2026-10-28 22:13) manifest.json engine key for the third program: "snapmaker" (matches the file name slicer_cli-snapmaker and what the program now prints as "engine").
   Provisional answer used: "snapmaker", plus "snapmakerProfilesPath".

### From helper review-fixes
# Helper "review-fixes": questions for the user

### Which objects' wall / infill filaments count for the plate's tower and arrange?
Time: 2026-10-28 22:27
Question: On the Snapmaker build, should an object WITHOUT support count its wall_filament / sparse_infill_filament / solid_infill_filament (when not 1) as a plate filament for the tower estimate and the arrange? The fork's desktop (PartPlate::get_extruders) does; the fork's CLI (get_extruders_under_cli) only does it for objects with support.
Provisional answer used: yes, as the desktop (same choice the Orca build already makes). Decision "Arrange plate filaments: the fork's 1-based feature filament keys".

### Seen while testing, not changed (outside this brief)
Time: 2026-10-28 22:27
- On the Snapmaker build `--downward-check` needs a value (`--downward-check 1`); without one it takes the next argument ("--downward-check needs a value", or the next token read as a model file). On the Orca build it is a plain switch. The fork defines downward_check twice (PrintConfig.cpp 8542 coStrings, 8798 coBool).
- `four_colour.3mf --slice 1|0 --downward-check 1 --downward-settings "roomy.json;tiny.json"` (300 mm and 100 mm test printers) reports downward_compatible_machine ["Roomy Test Printer", "Roomy Test Printer"]: one name twice, in either file order. Not investigated; no change made.
