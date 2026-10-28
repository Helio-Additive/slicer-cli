# Decisions

Format: ID, time, who, status (final / provisional), what, why, options, evidence, undo.
Rule for choices normally the user's (overnight): closest to Snapmaker's desktop app; tie → easiest to undo. Marked provisional.

## D-000  2026-10-28 20:29  coordinator  final (user-approved plan)
What:    Start the run per ~/Projects/Snapmaker/snapmaker-build-plan.md with the plan's suggested answers:
         pin Snapmaker Orca v2.4.1 (bac6fd5); Mac only; --layout-plan may refuse on the Snapmaker build;
         download Snapmaker_Orca_Mac_universal_V2.4.1.dmg allowed; thumbnails out of scope;
         checkpoint-2 limit 150 fix sites; hard stop 07:00.
Why:     User said "please go ahead with this build" after reviewing the plan.
Undo:    n/a

## D-001  2026-10-28 20:35  coordinator  final
What:    Do not run install_deps.sh. Install the Homebrew packages it lists (no forced `brew reinstall`),
         and build libnoise / CGAL into a local prefix inside the work folder (deps-prefix/) instead of /usr/local.
Why:     install_deps.sh installs libnoise into /usr/local with sudo (install_deps.sh:295) → password prompt;
         plan forbids anything needing the user's hands. Its first-run `brew reinstall` of all packages
         would also rewrite the user's existing Homebrew kegs unnecessarily.
Options: (a) local prefix, no sudo  (b) run script and stall on sudo  (c) ask user
Evidence: install_deps.sh:258-262, 274-300
Undo:    delete deps-prefix/; `brew uninstall` the packages added (listed in RUN-LOG.md)

## D-002  2026-10-28 20:44  coordinator  final
What:    Make the macOS deployment target a cache variable (SLICER_CLI_OSX_DEPLOYMENT_TARGET, default 10.15 as before)
         and build locally with 11.0.
Why:     This Mac's SDK (macOS 27, Apple clang 21) libc++ errors on targets below 11.0
         ("The selected platform is no longer supported by libc++", promoted by Clipper2's -Werror).
         CMakeLists.txt forced 10.15 with FORCE, so it could not be overridden.
Options: (a) overridable target, local 11.0  (b) -Wno-#warnings and keep 10.15  (c) edit to 11.0 for everyone
Chosen:  (a): release behaviour unchanged; (b) hides a real incompatibility.
Evidence: decision-log/build-orca.log:545; SDK availability.h:199-204
Undo:    revert the CMakeLists.txt hunk at line 65

## D-003  2026-10-28 20:48  coordinator  final
What:    -DENGINE=snapmaker = references/SnapmakerOrca (tag v2.4.1, bac6fd5) + override layer libslic3r/snapmaker,
         compiled with BOTH ENGINE_ORCA and ENGINE_SNAPMAKER. Clipper2 is built/linked only if the engine ships it
         (the fork doesn't, and its libslic3r never uses it). The fork has no libslic3r_version.h.in; the generated
         header just includes common_func/common_func.hpp, where the fork #defines its version and app names.
Why:     Reuse every existing Orca front-end path; mark only the differences. Bambu/Orca CMake paths unchanged.
Options: (a) Orca-family + extra define  (b) a fully separate third front-end path (duplicates ~140 Orca blocks)
Evidence: CMakeLists.txt:102-128, 364-371, 428-432; references/SnapmakerOrca/src/common_func/common_func.hpp:6-17
Undo:    revert commit "Local: -DENGINE=snapmaker …"

## D-004  2026-10-28 21:05  coordinator  final
What:    The Snapmaker build uses the fork's bundled Eigen 3.3.7 (references/SnapmakerOrca/deps_src/eigen) instead of Homebrew's Eigen 5.0.1.
Why:     The fork's libigl (deps_src/libigl/igl/slice.cpp:54) uses Eigen::DynamicSparseMatrix, removed in Eigen 5, so the engine's
         MeshBoolean.cpp failed. The fork's own deps/ has no Eigen recipe, i.e. Snapmaker builds against the bundled copy too.
Options: (a) bundled Eigen for snapmaker only  (b) install an old Eigen  (c) override libigl (not covered by the override layer)
Evidence: decision-log/build-snapmaker-1.log (MeshBoolean.cpp.o); references/SnapmakerOrca/deps/ listing
Undo:    revert the Eigen hunk in CMakeLists.txt (~line 179)

## D-005  2026-10-28 21:05  coordinator  final (deviation from plan, explained)
What:    Phase 2 helpers work in this one checkout on disjoint files instead of separate git worktrees. They compile only their
         own files via decision-log/scripts/compile.sh, which holds a lock so one build runs at a time. They don't commit; the
         coordinator commits each helper's files separately after checking them.
Why:     A worktree needs its own copy of three engine checkouts and its own full builds (GBs, and the lock would serialise
         them anyway). The errors already split cleanly by file, so file ownership gives the same no-overlap guarantee.
Options: (a) shared checkout + file ownership + lock  (b) worktrees per helper as planned
Undo:    n/a (process only)

## D-100  helper "frontend", accepted by coordinator 2026-10-28 21:29
Title: Version macro on the Snapmaker build
Time: 2026-10-28 21:17
What:    SoftFever_VERSION does not exist in the fork. engine_version() (cli_run_steps.cpp) and engine_version_text() (main.cpp) return Snapmaker_VERSION ("2.4.1") on the Snapmaker build.
Why:     Compile error; the fork's own release number is Snapmaker_VERSION, SLIC3R_VERSION is the Bambu base ("01.10.01.50").
Options: (a) Snapmaker_VERSION (b) SLIC3R_VERSION
Chosen:  (a) because Snapmaker's CLI writes Snapmaker_VERSION into a saved project_settings, the same use as engine_version() at cli_run_steps.cpp:307, and the Orca build uses its own release number (SoftFever_VERSION) in the same places.
Evidence: cli_run_steps.cpp:24-33, main.cpp:2114 (engine_version_text); references/SnapmakerOrca/src/common_func/common_func.hpp:8-9; Snapmaker_Orca.cpp:4761
User-visible effect: the version written into saved project settings and the version an Orca-made 3MF is compared with (file_newer_than_engine) is 2.4.1. Snapmaker's own CLI version gate (Snapmaker_Orca.cpp:1431-1436) compares with SLIC3R_VERSION instead; not mirrored here (see questions).
Status: provisional

## D-101  helper "frontend", accepted by coordinator 2026-10-28 21:29
Title: Fork names for StringExceptionType
Time: 2026-10-28 21:17
What:    The fork names value 2 STRING_EXCEPT_FILAMENTS_MIXING_TEMP (not _DIFFERENT_TEMP) and adds COLD_PLATE_INCOMPATIBLE and FLOW_RATIO_ZERO. string_exception_tag() and the validate()-to-result-code switch use the fork's names.
Why:     Compile error; unhandled enum values would warn (-Wswitch).
Options: (a) use the fork's names (b) cast to int
Chosen:  (a) because it's a rename; MIXING_TEMP maps to CLI_FILAMENTS_DIFFERENT_TEMP exactly as Snapmaker's CLI does.
Evidence: main.cpp:159-166, main.cpp:10981-10986; references/SnapmakerOrca/src/libslic3r/PrintBase.hpp:21-31; Snapmaker_Orca.cpp:5034-5036
User-visible effect: none (event tag text for value 2 reads STRING_EXCEPT_FILAMENTS_MIXING_TEMP)
Status: final

## D-102  helper "frontend", accepted by coordinator 2026-10-28 21:29
Title: Validation event kind without StringObjectException::is_warning
Time: 2026-10-28 21:17
What:    The fork's StringObjectException has no is_warning. On the Snapmaker build emit_validation_event takes an is_warning argument (default false); the call for validate()'s warning out-parameter passes true.
Why:     Compile error.
Options: (a) caller says which (b) always "validation_error"
Chosen:  (a) because the fork's validate() returns warnings only through the out-parameter, so the caller knows exactly which one it holds; the events come out as on the Orca build.
Evidence: main.cpp:446-457, main.cpp:10956-10960; references/SnapmakerOrca/src/libslic3r/PrintBase.hpp:34-41, Print.hpp:903
User-visible effect: none
Status: final

## D-103  helper "frontend", accepted by coordinator 2026-10-28 21:29
Title: No G-code check, first-layer printable or limited-map results on the fork
Time: 2026-10-28 21:17
What:    The fork's GCodeProcessorResult has no gcode_check_result, filament_printable_reuslt or limit_filament_maps. On the Snapmaker build the gcode_check, filament_unprintable and filament_map_limited events are not emitted, and post_slice_checks() does not refuse on them (CLI_GCODE_PATH_OUTSIDE / IN_WRAPPING_DETECT_AREA / IN_UNPRINTABLE_AREA / FILAMENT_UNPRINTABLE_ON_FIRST_LAYER). Conflicts and critical slicing warnings are still refused.
Why:     Compile error; the engine produces no such data.
Options: (a) skip the checks (b) recompute them in the front end
Chosen:  (a) because Snapmaker's CLI checks only conflicts and critical warnings after slicing; (b) would invent data.
Evidence: main.cpp:248-290, 319-337, 349-356, 5095-5130; references/SnapmakerOrca/src/libslic3r/GCode/GCodeProcessor.hpp:134-260; Snapmaker_Orca.cpp:5139-5168
User-visible effect: on the Snapmaker build those events never appear and those refusals never happen; toolpath_outside and timelapse_warning events stay.
Status: provisional

## D-104  helper "frontend", accepted by coordinator 2026-10-28 21:29
Title: No points groups, per-extruder areas or expert mode in the fork
Time: 2026-10-28 21:17
What:    The fork has no coPointsGroups/coIntsGroups, no extruder_printable_area/height and no comExpert. The Snapmaker build skips these: the '#' separator for points groups (layout_plan.cpp join_profile_array, main.cpp JSON flattening), the per-extruder areas in check_objects_inside_bed (BuildVolume built from bed and height, the fork's only constructor), the shared nozzle area when moving a plate to a new bed (shared centre stays 0), the extruder-area narrowing in assemble_tower_inside_plate, and the type/mode names in the option dump.
Why:     Compile errors; the options do not exist in the fork.
Options: (a) skip (b) none: nothing to read
Chosen:  (a) because the fork's single-area behaviour is what Snapmaker's CLI does (BuildVolume(area, height); translate_models without a shared area).
Evidence: layout_plan.cpp:159-162; main.cpp:800-806, 3438-3460, 4117-4140, 4622-4632, 6238-6243, 6250; references/SnapmakerOrca/src/libslic3r/Config.hpp:158-201, BuildVolume.hpp:36; Snapmaker_Orca.cpp:3255-3330
User-visible effect: none
Status: final

## D-105  helper "frontend", accepted by coordinator 2026-10-28 21:29
Title: No filament variant tables in the fork
Time: 2026-10-28 21:17
What:    normalize_filament_config_vectors skips the filament_options_with_variant test on the Snapmaker build.
Why:     Compile error; the fork has no variant tables, so no filament option is sized by slot count.
Options: (a) skip the test
Chosen:  (a) mechanical.
Evidence: main.cpp:1962-1968; references/SnapmakerOrca/src/libslic3r/Preset.hpp (no *_options_with_variant)
User-visible effect: none
Status: final

## D-106  helper "frontend", accepted by coordinator 2026-10-28 21:29
Title: CLI result codes the fork does not have
Time: 2026-10-28 21:17
What:    CLI_GCODE_PATH_IN_UNPRINTABLE_AREA, CLI_FILAMENT_CAN_NOT_MAP and CLI_ONLY_ONE_TPU_SUPPORTED are left out of the result-sentence table on the Snapmaker build.
Why:     Compile error; the fork defines and raises none of them.
Options: (a) leave the entries out (b) define the codes locally
Chosen:  (a) because nothing on that build can produce them.
Evidence: main.cpp:3040-3043, 3073-3076; references/SnapmakerOrca/src/libslic3r/Utils.hpp:24-70
User-visible effect: none
Status: final

## D-107  helper "frontend", accepted by coordinator 2026-10-28 21:29
Title: Prime tower size estimate: the fork's own
Time: 2026-10-28 21:17
What:    arrange_estimate_wipe_tower_size follows the fork's PartPlate::estimate_wipe_tower_size on the Snapmaker build: x starts at the wipe volume, need_wipe_tower is smooth timelapse only, extra_spacing from wipe_tower_extra_spacing, filament change volume = max filament_multitool_ramming_volume (default Print: engine defaults, 0.5 when empty), volume = wipe*n + change*(n-1), rib: depth = rib_width + max(depth + extra_rib_length, depth), plain: depth = volume/(layer_height*wipe_volume)*extra_spacing. No auto tower brim (prime_tower_brim_width < 0) anywhere: the fork has none (min 0). In the arrange clamp, w is no longer replaced by the estimate's x (Snapmaker's CLI clamps with prime_tower_width).
Why:     Compile errors (filament_change_length, WipeTower::get_limit_depth_by_height, get_auto_brim_by_height do not exist); prime_tower_infill_gap does not exist either (arrange_opt would return null).
Options: (a) port the fork's estimate (b) keep the 2.4 estimate minus missing pieces
Chosen:  (a) because Snapmaker's CLI calls exactly that function before placing the tower.
Evidence: main.cpp:3828-3832, 3854-3945, 4002-4008, 4356-4372, 4644-4648, 4695-4699; references/SnapmakerOrca/src/slic3r/GUI/PartPlate.cpp:2177-2249, 2253-2262; Snapmaker_Orca.cpp:3238, 4310-4358; PrintConfig.cpp:6221-6227
User-visible effect: arrange/assemble tower placement on the Snapmaker build follows the fork's sizing.
Status: provisional

## D-108  helper "frontend", accepted by coordinator 2026-10-28 21:29
Title: Filament change count field
Time: 2026-10-28 21:17
What:    PrintEstimatedStatistics::total_filament_changes is spelt total_filamentchanges in the fork.
Why:     Compile error.
Options: (a) use the fork's field
Chosen:  (a) rename.
Evidence: main.cpp:4869-4871; references/SnapmakerOrca/src/libslic3r/GCode/GCodeProcessor.hpp:87
User-visible effect: none
Status: final

## D-109  helper "frontend", accepted by coordinator 2026-10-28 21:29
Title: PlateData filament-map fields absent
Time: 2026-10-28 21:17
What:    limit_filament_maps, layer_filaments, filament_change_sequence, nozzle_change_sequence, optimal_assignment and filament_maps are not copied into PlateData on the Snapmaker build (record_plate_for_export and the export of sliced plates). The Orca block that applies a plate's filament_map_mode / filament_maps is skipped too.
Why:     Compile errors; the fork's PlateData has none of them, it has no filament_map_mode option, and Print has no get_filament_maps().
Options: (a) skip
Chosen:  (a) the fork cannot store or apply them.
Evidence: main.cpp:4931-4941, 9793-9817, 11531-11540; references/SnapmakerOrca/src/libslic3r/Format/bbs_3mf.hpp (struct PlateData), PrintConfig.hpp:341
User-visible effect: none beyond what the fork's 3MF format holds
Status: final

## D-110  helper "frontend", accepted by coordinator 2026-10-28 21:29
Title: full_config() takes no argument
Time: 2026-10-28 21:17
What:    bundle.full_config(false) becomes bundle.full_config() on the Snapmaker build (named presets and the Bambu-project rebase).
Why:     Compile error; the fork has no extruder variants to map.
Options: (a) full_config()
Chosen:  (a) the only form the fork has.
Evidence: main.cpp:6077-6083, 8077-8081; references/SnapmakerOrca/src/libslic3r/PresetBundle.hpp:211
User-visible effect: none
Status: final

## D-111  helper "frontend", accepted by coordinator 2026-10-28 21:29
Title: load_bbs_3mf signature and which projects are rebased
Time: 2026-10-28 21:17
What:    The fork's load_bbs_3mf has the Bambu signature (no is_orca_3mf). Its is_bbl_3mf is true for both "BambuStudio-" and "Snapmaker_Orca-" projects. On the Snapmaker build is_orca_3mf is set from the 3MF's Application metadata: only an "BambuStudio-" project counts as Bambu-made, so only it is rebased onto this engine's presets.
Why:     Compile error, and without the split a Snapmaker Orca project would be rebased as if Bambu Studio had made it.
Options: (a) derive from Application metadata (b) treat every is_bbl_3mf as Bambu-made (c) never rebase
Chosen:  (a) because it keeps the Orca build's rule (rebase only Bambu Studio projects) with the fork's loader.
Evidence: main.cpp:7279-7286, 7883-7915, 8008 (bambu_made_project); references/SnapmakerOrca/src/libslic3r/Format/bbs_3mf.hpp:239-240, bbs_3mf.cpp:3827-3837
User-visible effect: a Snapmaker Orca project slices with its own settings; a Bambu Studio project is rebased as on the Orca build.
Status: provisional

## D-112  helper "frontend", accepted by coordinator 2026-10-28 21:29
Title: Type-1 tower height fix without wipe_tower_type()
Time: 2026-10-28 21:17
What:    The WipeTowerData::height = 0 workaround is applied when print.is_BBL_printer() on the Snapmaker build.
Why:     Compile error; the fork has no wipe_tower_type(). It builds the Type-1 tower exactly when is_BBL_printer(), writes height only in the WipeTower2 branch, and its clear() leaves height unset, so the same bug exists.
Options: (a) is_BBL_printer() (b) drop the workaround
Chosen:  (a) same condition in the fork's terms.
Evidence: main.cpp:10781-10793; references/SnapmakerOrca/src/libslic3r/Print.cpp:3221, 3279, 3375, 3474; Print.hpp:749-765, 1009-1010
User-visible effect: none (the U1 is not a Bambu Lab printer)
Status: final

## D-113  helper "frontend", accepted by coordinator 2026-10-28 21:29
Title: Filament temperature mix and --allow-mix-temp
Time: 2026-10-28 21:17
What:    The fork has no Print::set_check_multi_filaments_compatibility and its validate() does not test the temperature mix. On the Snapmaker build the front end does what Snapmaker's CLI does: unless --allow-mix-temp is given true, the filament types of print.extruders() go through Print::check_multi_filaments_compatibility; a high+low mix is refused as a validation error of type STRING_EXCEPT_FILAMENTS_MIXING_TEMP (CLI_FILAMENTS_DIFFERENT_TEMP) with Snapmaker's CLI sentence. It overrides whatever validate() returned, as Snapmaker's CLI stops before validate.
Why:     Compile error, and keeping the Orca build's default (mix refused, flag allows) without the engine call.
Options: (a) Snapmaker CLI's guard, flag turns it off (b) no check, flag reported unsupported (c) guard always on, ignore flag
Chosen:  (a) because it mirrors Snapmaker's CLI and keeps --allow-mix-temp meaningful.
Evidence: main.cpp:10879-10898, 10920-10925; references/SnapmakerOrca/src/Snapmaker_Orca.cpp:5000-5018; libslic3r/Print.cpp:1518-1533, 1575-1588 (check_multi_filament_valid is private and unused by validate)
User-visible effect: a run mixing high- and low-temperature filaments is refused (CLI_FILAMENTS_DIFFERENT_TEMP) unless --allow-mix-temp. The desktop app only warns about it.
Status: provisional

## D-114  helper "frontend", accepted by coordinator 2026-10-28 21:29
Title: Assembled OBJ colours
Time: 2026-10-28 21:17
What:    On the Snapmaker build an OBJ in the assemble list is loaded as a plain mesh and gets its listed filament: no colour clustering (obj_color_deal_algo does not exist in the fork), no missing-material refusal (ObjInfo has no lost_material_name), no "faces without colour" refusal. convert_obj_cluster_colors is compiled out there.
Why:     Compile errors.
Options: (a) mirror Snapmaker's CLI (plain mesh) (b) cluster with the GUI's QuantKMeans
Chosen:  (a) because Snapmaker's CLI does exactly that; (b) would recreate a feature the CLI does not have.
Evidence: cli_assemble.cpp:131-215, 399-406, 415-457; references/SnapmakerOrca/src/Snapmaker_Orca.cpp:847-857, libslic3r/Format/OBJ.hpp:12-22
User-visible effect: a coloured OBJ in an assemble list prints in its one listed filament on the Snapmaker build.
Status: provisional

## D-115  helper "frontend", accepted by coordinator 2026-10-28 21:29
Title: Profiles tree and engine resources root
Time: 2026-10-28 21:17
What:    orca_profiles_dir() searches references/SnapmakerOrca/resources/profiles (checkout) and resources/profiles-snapmaker (package); engine_resources_root() uses references/SnapmakerOrca/resources (checkout) and resources/snapmaker (package), on the Snapmaker build.
Why:     The coordinator's brief; the fork's info/ and flush/ resources and its profiles are its own.
Options: (a) as briefed
Chosen:  (a)
Evidence: main.cpp:2231-2248, 2842-2856; references/SnapmakerOrca/resources/{profiles,info}
User-visible effect: the Snapmaker build reads Snapmaker's profiles and resources.
Status: final

## D-116  helper "presets", accepted by coordinator 2026-10-28 21:29
Title: Settings merge: no per-extruder variant restore
Time: 2026-10-28 21:18
What:    update_full_config keeps a key the project changed and does nothing else on the Snapmaker build; the machine and process merges call it with no variant index, no variant key lists and variant_count_changed=false; compute_variant_index is not called.
Why:     the fork has no printer/print_extruder_variant, *_options_with_variant lists or set_with_restore; its own CLI lambda only keeps the changed keys.
Options: (a) define empty upstream-named sets and keep the upstream calls (b) Snapmaker branch calling the merge the way the fork's CLI does.
Chosen:  (b) because it is the fork's own flow and needs no upstream names.
Evidence: cli_load_settings.cpp:267, 1120, 1135, 1220, 1233; references/SnapmakerOrca/src/Snapmaker_Orca.cpp:2390-2434, 2464-2479, 2601-2616
User-visible effect: none
Status: final

## D-117  helper "presets", accepted by coordinator 2026-10-28 21:29
Title: Variant keys the fork does not define are never created
Time: 2026-10-28 21:18
What:    the reads of printer_extruder_variant / print_extruder_variant with create=true (machine count, process count, current_print_extruder_variants) and the whole nozzle-volume-type / extruder-variant step are skipped on the Snapmaker build; new_printer_extruder_variants and current_print_extruder_variants stay empty, new_printer_variant_count stays 1, ProjectFacts::current_extruder_variants stays "" per extruder.
Why:     DynamicConfig::optptr(create) returns nullptr for a key the def lacks (fork Config.cpp:1628-1648), so ->values would crash; the fork's CLI has no such step.
Options: (a) skip (b) fill with invented variant strings.
Chosen:  (a) because there is nothing in the fork to fill them with (rule: never invent data).
Evidence: cli_load_settings.cpp:468, 1183, 1241, 1254; Snapmaker_Orca.cpp:2620-2642; fork PrintConfig.cpp has no printer_extruder_variant/print_extruder_variant/extruder_type
User-visible effect: none (the fields are internal; update_object_configs_after_switch is a no-op, below)
Status: final

## D-118  helper "presets", accepted by coordinator 2026-10-28 21:29
Title: Filament merge: the fork's composed flow-variant layout
Time: 2026-10-28 21:18
What:    a loaded filament's vector values go to its index as the fork's CLI does, except filament_flow_variant_options() and filament_flow_support, which take one value per flow type the filament declares in the segment filament_flow_step_size gives it (the layout PresetBundle::full_fff_config composes); the other flow-variant values of the project follow the new segment, kept per flow type; filament_flow_step_size is written after the merge. A key the project changed is kept as in the fork's CLI; a changed flow-variant key keeps its value for each flow type the new filament still declares and takes the new value for a flow type it adds. No filament_self_index / filament_extruder_variant.
Why:     the brief says to do what the fork does when it loads presets. The fork's CLI writes only index 0 of each filament (one value per filament): the standard values slice the same, but the config (and the G-code's settings block) then differs from the desktop's, which keeps every declared variant (U1 filaments such as "Snapmaker TPU 95A HF @U1 0.4 nozzle" declare ["standard","high_flow"]).
Options: (a) the fork's CLI exactly (set_at index 0, high_flow values dropped, no filament_flow_step_size) (b) the desktop's composition (segments + filament_flow_step_size).
Chosen:  (b) because the goal is the desktop's G-code and get_config_idx (fork PrintConfig.cpp:7553-7610) reads the segments through filament_flow_step_size.
Evidence: cli_load_settings.cpp:344 (rewrite_flow_segment), 1407; fork PresetBundle.cpp:2557-2587, 2673-2722, 2905-2925; Snapmaker_Orca.cpp:2645-2765
User-visible effect: the merged settings carry the high-flow filament values and filament_flow_step_size, as the desktop's do; standard slicing reads the same values either way.
Status: provisional (the user should review)

## D-119  helper "presets", accepted by coordinator 2026-10-28 21:29
Title: Flush volumes: the fork's one-block matrix
Time: 2026-10-28 21:18
What:    the flush matrix is recomputed when colours are given or there is no matrix (not on extruder-count or nozzle-type change); it is filament x filament, flush_multiplier (a scalar in the fork) is set to 1, FlushVolCalculator(min, max) with no dataset, calc_flush_vol from the colours only. get_min_flush_volumes reads nozzle_volume as the fork's scalar.
Why:     the fork has no nozzle_flush_dataset, get_index_for_extruder, get/set_flush_volumes_matrix; flush_multiplier is coFloat; nozzle_volume is coFloat (the upstream read as ConfigOptionFloatsNullable gave 0); the third FlushVolCalculator argument is a multiplier in the fork, so passing the dataset id would have meant something else.
Options: (a) the fork's CLI block (b) keep the upstream per-nozzle block with stand-ins.
Chosen:  (a) because it is the fork's own code.
Evidence: cli_load_settings.cpp:405, 1740, 1782; Snapmaker_Orca.cpp:2781-2900; fork slic3r/GUI/Plater.cpp:1447-1500; fork FlushVolCalc.hpp:17
User-visible effect: flush volumes as the fork computes them (min flush volume now includes the U1's nozzle_volume where it states one)
Status: provisional (the user should review)

## D-120  helper "presets", accepted by coordinator 2026-10-28 21:29
Title: Printer switch leaves object settings alone
Time: 2026-10-28 21:18
What:    update_object_configs_after_switch returns at once on the Snapmaker build.
Why:     it moves per-extruder process variants (update_values_from_multi_to_multi_2), which the fork does not have; the fork's CLI does nothing to object configs on a switch.
Options: (a) no-op (b) remap something.
Chosen:  (a).
Evidence: cli_load_settings.cpp:1954; Snapmaker_Orca.cpp (no update_values_from_* call)
User-visible effect: none
Status: final

## D-121  helper "presets", accepted by coordinator 2026-10-28 21:29
Title: Downward check: no shared extruder box, no wrapping area
Time: 2026-10-28 21:18
What:    shared_extruder_box returns false; load_downward_printers reads no wrapping_exclude_area.
Why:     the fork has no extruder_printable_area / extruder_printable_height / wrapping_exclude_area (creating the last would crash), and its own downward check reads none.
Options: (a) skip (b) invent a box.
Chosen:  (a).
Evidence: cli_load_settings.cpp:2032, 2124; Snapmaker_Orca.cpp:3374-3420
User-visible effect: none (those checks only ever applied to multi-extruder Bambu printers and wrapping detection)
Status: final

## D-122  helper "presets", accepted by coordinator 2026-10-28 21:29
Title: Plate type: the fork's get_default_bed_type, U1 clamp
Time: 2026-10-28 21:18
What:    desktop_bed_type on the Snapmaker build: Preset::get_default_bed_type (U1 -> SM_U1 -> btPTE, Textured PEI Plate), then a U1 without support_multi_bed_types takes btPTE for any plate but btPTE/btPEI/btGESP. No "first plate the list offers" fallback; offered_bed_types (not_support_bed_types) is not compiled.
Why:     the fork's Sidebar::update_all_preset_comboboxes sets project curr_bed_type to that plate and only moves the list's selection; it has no not_support_bed_types and no set_bed_type_accord_combox step there.
Options: (a) mirror the fork (b) build an offered list from enum_values_u1/_ex and fall back to its first entry.
Chosen:  (a) because that is what the fork's desktop does.
Evidence: desktop_presets.cpp:40, 444; fork slic3r/GUI/Plater.cpp:3614-3706, fork Preset.cpp:892-915, fork PrintConfig.cpp:865-900
User-visible effect: the plate type a named printer slices on (U1: Textured PEI Plate); the "why" text differs.
Status: provisional (the user should review)

## D-123  helper "presets", accepted by coordinator 2026-10-28 21:29
Title: bed_type_name by the enum keys map
Time: 2026-10-28 21:18
What:    bed_type_name looks the type up in curr_bed_type's enum_keys_map on the Snapmaker build.
Why:     the fork's enum_values list omits Graphic Effect Plate, so position type-1 names btGESP "Supertack Plate" and btSuperTack nothing.
Options: (a) keys map (b) keep the position.
Chosen:  (a).
Evidence: desktop_presets.cpp:424; fork PrintConfig.hpp:282-292, PrintConfig.cpp:386-396, 865-875
User-visible effect: the plate name in result/events is right for Graphic Effect Plate and Supertack Plate
Status: final

## D-124  helper "u1-tests", accepted by coordinator 2026-10-28 21:29
Title: Project settings come from the fork's own U1 presets, not from an Orca export
Time: 2026-10-28 21:22
What:    `Metadata/project_settings.config` of every generated 3MF is the Snapmaker fork's U1 presets resolved through `inherits`: printer "Snapmaker U1 (0.4 nozzle)", process "0.20mm Standard @Snapmaker U1 (0.4 nozzle)", filament "Snapmaker PLA SnapSpeed @U1" (x N). Keys the presets don't state are left out, so each engine fills them with its own defaults.
Why:     The parity target is the fork engine. A config exported from upstream 2.4.2 would carry upstream-only keys/values (e.g. `nozzle_volume_type` "Standard", 2.4 filament-map keys) and upstream's U1 profile values, which differ from the fork's.
Options: (a) export a full config with build-orca and reuse it; (b) flatten the fork's profile JSON; (c) wait for build-snapmaker's `--export-settings`.
Chosen:  (b) because it needs no fork binary yet and is what the fork's preset system starts from (defaults + inherits chain). When build-snapmaker works, `--export-settings` could replace it with a full key set if the coordinator wants that.
Evidence: references/SnapmakerOrca/resources/profiles/Snapmaker/{machine,process,filament}; PresetBundle.cpp 2673-2722 (filament composition).
User-visible effect: none (test data only)
Status: provisional (the user should review: a desktop-saved project holds every key, ours only the preset keys)

## D-125  helper "u1-tests", accepted by coordinator 2026-10-28 21:29
Title: SnapSpeed PLA as the test filament
Time: 2026-10-28 21:22
What:    All models use "Snapmaker PLA SnapSpeed @U1".
Why:     It is the fork's default U1 filament (PresetBundle.cpp SM_DEFAULT_FILAMENT) and it declares `filament_flow_support: [standard, high_flow]`, which the high-flow variant needs. Upstream 2.4.2 ships a preset of the same name, so `--filament-preset` works for both slicer_cli builds.
Options: (a) SnapSpeed; (b) "Snapmaker PLA Basic @U1" (standard flow only); (c) Generic PLA (name differs between trees).
Chosen:  (a).
Evidence: references/SnapmakerOrca/resources/profiles/Snapmaker/filament/Snapmaker PLA SnapSpeed @U1.json; build-orca `--list-presets --printer "Snapmaker U1 (0.4 nozzle)"`.
User-visible effect: none
Status: final

## D-126  helper "u1-tests", accepted by coordinator 2026-10-28 21:29
Title: four_colour.3mf tag: Application "BambuStudio-01.10.01.50" + OrcaSlicer "2.4.1"
Time: 2026-10-28 21:22
What:    The portable projects (four_colour*, mixed_filament, two_plate_portable) carry `Application = BambuStudio-01.10.01.50`, `OrcaSlicer = 2.4.1`, `BambuStudio:3mfVersion = 1`.
Why:     Both loaders must treat the file as a project (m_is_bbl_3mf) and both CLIs' version gates must pass. The fork sets m_is_bbl_3mf for "BambuStudio-" or "Snapmaker_Orca-" only and its CLI requires file major == SLIC3R_VERSION major (01.10.01.50) -- it ignores the OrcaSlicer tag. Upstream takes the OrcaSlicer tag as the file version when present (2.4 <= 2.4.2 passes). Upstream's own export ("BambuStudio-02.06.00.51") would fail the fork CLI's gate; a Snapmaker tag is not recognised by upstream.
Options: (a) BambuStudio-01.10.01.50 + OrcaSlicer 2.4.1; (b) copy upstream's export tags; (c) Snapmaker_Orca-2.4.1.
Chosen:  (a): the only combination that satisfies both gates.
Evidence: SnapmakerOrca bbs_3mf.cpp 3818-3837, Snapmaker_Orca.cpp 1431-1436, version.inc (SLIC3R_VERSION 01.10.01.50); OrcaSlicer bbs_3mf.cpp 1425-1431, 3946-3962, OrcaSlicer.cpp 1657-1661.
User-visible effect: none
Status: final (verified on upstream; fork side read from code only)

## D-127  helper "u1-tests", accepted by coordinator 2026-10-28 21:29
Title: 3MF layout mirrors what Orca/Snapmaker write (sub-model files)
Time: 2026-10-28 21:22
What:    `3D/3dmodel.model` with component objects pointing to `3D/Objects/object_N.model` (production extension), `3D/_rels/3dmodel.model.rels`, `Metadata/model_settings.config` (per-object `extruder`, `<plate>` with `model_instance`s), `Metadata/project_settings.config`, `[Content_Types].xml`, `_rels/.rels`. No thumbnails, slice_info or G-code. Plate 2 objects are offset by 324 mm in X (bed 270 x 1.2), taken from an upstream 2-plate export. Zip dates are fixed, so regeneration is byte-identical.
Why:     It is the layout both Bambu-derived importers write and read; the inline-mesh variant has volume-range rules I did not want to depend on.
Options: (a) sub-model files; (b) inline meshes.
Chosen:  (a).
Evidence: build-orca `--export-3mf` of a 4-object and an `--load-assemble-list` 2-plate project (scratch), compared member by member.
User-visible effect: none
Status: final

## D-128  helper "u1-tests", accepted by coordinator 2026-10-28 21:29
Title: Graphic Effect Plate: serialized name "Graphic Effect Plate"
Time: 2026-10-28 21:22
What:    four_colour_gep.3mf sets `curr_bed_type = "Graphic Effect Plate"` (btGESP); bed temp keys `graphic_effect_plate_temp(_initial_layer)` = 65 in the SnapSpeed preset.
Why:     The fork's BedType map has exactly this key and forces it as the serialized name.
Evidence: SnapmakerOrca PrintConfig.cpp 386-409, 884; PrintConfig.hpp 477, 503.
User-visible effect: none
Status: final

## D-129  helper "u1-tests", accepted by coordinator 2026-10-28 21:29
Title: High flow: whole-project high_flow, built like the fork's full_config()
Time: 2026-10-28 21:22
What:    four_colour_highflow.3mf: `nozzle_volume_type = [high_flow]x4`, `filament_volume_type = [high_flow]x4`, process flow-variant options kept as [standard, high_flow] arrays with `process_flow_support`, filament flow-variant options concatenated per filament with `filament_flow_support` and `filament_flow_step_size = [2]x4`. The standard projects reduce every variant array to its standard value (`*_flow_support = [standard]`, step 1).
Why:     get_config_idx() picks the variant index from filament_volume_type + *_flow_support (+ step sizes for filaments); this is the composed form PresetBundle writes.
Options: (a) per-filament mix of flows; (b) all high flow.
Chosen:  (b): one clear variant; per-filament mixes can be added later.
Evidence: SnapmakerOrca PrintConfig.cpp 4096-4160, 7440-7605; PresetBundle.cpp 200-216, 2673-2722; Preset.cpp 405-430.
User-visible effect: none
Status: provisional (not yet sliced by a fork-based binary; I could not confirm the GUI writes exactly this shape)

## D-130  helper "u1-tests", accepted by coordinator 2026-10-28 21:29
Title: Mixed filament: done, from the fork's own parser
Time: 2026-10-28 21:22
What:    mixed_filament.3mf: 2 physical filaments, `mixed_filament_definitions = "1,2,1,1,50"` (a,b,enabled,custom,mix_b_percent), one object on filament 3, the first virtual filament.
Why:     The fork reads this project key (PresetBundle s_project_options), Print::apply loads it with load_custom_entries(), and the 3MF loader allows extruder ids up to physical + enabled mixed. Auto-generated pairs are off by default, so 3 = this row.
Evidence: SnapmakerOrca MixedFilament.cpp 33, 373-480, 1714-1760, 2328-2370, 2800-2815; PrintApply.cpp 1419-1452; bbs_3mf.cpp 619-650.
User-visible effect: none
Status: provisional (expected: tools 0 and 1 with a wipe tower; not yet sliced by a fork-based binary)

## D-131  helper "u1-tests", accepted by coordinator 2026-10-28 21:29
Title: Upstream proof run re-targets projects to upstream's own U1 presets
Time: 2026-10-28 21:22
What:    run_u1_tests.sh runs each project as-is, and for a binary whose profiles are not the fork's (process name differs) also "<model>__own_presets" with `--printer-preset/--process-preset` of that binary.
Why:     The projects carry the fork's U1 start G-code, which uses `chamber_cooling_mode`, a placeholder upstream doesn't have, so upstream cannot export G-code for them as-is. Re-targeting lets the checker be proven on real multi-colour/multi-plate output from upstream.
Options: (a) only as-is runs (all projects fail on upstream, checker untested on them); (b) strip fork G-code from the projects (then they are no longer faithful U1 projects); (c) extra re-targeted cases.
Chosen:  (c); `U1_RETARGET=0|1` overrides.
Evidence: work/u1-tests/out-orca/four_colour/run.log ("machine_start_gcode Parsing error at line 10 ... chamber_cooling_mode").
User-visible effect: none
Status: final

## D-132  helper "u1-tests", accepted by coordinator 2026-10-28 21:29
Title: Snapmaker's own CLI refuses its own 2.4.1 projects without allow_newer_file
Time: 2026-10-28 21:22
What:    The run script slices two_plate_snapmaker.3mf twice: plain and with `--allow-newer-file`.
Why:     Snapmaker_Orca.cpp compares the file version (2.4.1 from "Snapmaker_Orca-2.4.1") with SLIC3R_VERSION 01.10.01.50 and refuses a different major (CLI_FILE_VERSION_NOT_SUPPORTED) unless allow_newer_file / SNAPMAKER_ORCA_ALLOW_NEWER_FILE is set. Our CLI's ENGINE_SNAPMAKER gate compares with Snapmaker_VERSION 2.4.1 (main.cpp engine_version_text), so it would accept the file. Parity difference for the coordinator to decide; I have not run the official CLI to confirm.
Evidence: references/SnapmakerOrca/src/Snapmaker_Orca.cpp 1271-1279, 1431-1436; references/SnapmakerOrca/version.inc; main.cpp 2091-2125.
User-visible effect: possibly: our build accepts a file the official CLI refuses (or the reverse, if someone "fixes" it)
Status: provisional (the user should review)

## D-006  2026-10-28 21:34  coordinator  final
What:    (a) Snapmaker Orca's own CLI is run only through decision-log/scripts/snapmaker-cli.sh: macOS sandbox with all network
         denied, its own --datadir under work/, a 15-minute limit. (b) The flattened process JSON omits wipe_tower_filament when it is "0".
Why:     (a) It starts a Sentry crash/usage session on every run; nothing from this run should reach Snapmaker. Confirmed blocked
         ("Operation not permitted" from its HTTP transport). Note: it still writes crash dumps to
         ~/Library/Application Support/Snapmaker_Orca/SentryData/pending (outside the work folder; left in place, not deleted).
         (b) Their CLI segfaults on a process file containing wipe_tower_filament (null nozzle_diameter in normalize_fdm). "0" is the default.
Evidence: work/out-smapp/cube*.log; helper u1-tests analysis (Snapmaker_Orca.cpp 1694-1760, fork PrintConfig.cpp ~7872)
Undo:    edit the two scripts

## D-133  helper "u1-tests", accepted by coordinator 2026-10-28 21:34
Title: single.3mf: single.stl as a one-filament "portable" project (case single_project)
Time: 2026-10-28 21:34
What:    models/single.3mf = the single part, object name "single", filament 1, one filament in the project, same tag (BambuStudio-01.10.01.50 + OrcaSlicer 2.4.1) and same embedded fork presets as four_colour.3mf. Case `single_project` in run_u1_tests.sh and manifest.json; `single` (STL) unchanged.
Why:     Coordinator request: Snapmaker's own CLI crashes on the STL path (see next entry), so the single-colour part needs a project form for parity.
Options: (a) 1 filament; (b) 4 filaments with only filament 1 used.
Chosen:  (a): no wipe tower question, the simplest U1 project.
Evidence: out-smapp/single_project vs out-snap/single_project: compare_gcode.py IDENTICAL (19425 normalised lines, 17551 moves, net E 2836.429 mm, 14m46s).
User-visible effect: none
Status: final

## D-134  helper "u1-tests", accepted by coordinator 2026-10-28 21:34
Title: Why Snapmaker Orca's CLI segfaults on the flattened process JSON (code reading, not debugged)
Time: 2026-10-28 21:34
What:    `--load-settings "machine.json;process.json"` -> rc 139 in load_config_file (Snapmaker_Orca.cpp 1694-1760), which calls `config.normalize_fdm()` on each file's config by itself. The fork's DynamicPrintConfig::normalize_fdm() (PrintConfig.cpp ~7872+) does `if (has("wipe_tower_filament")) { ... opt<ConfigOptionFloats>("nozzle_diameter")->size() ... }` with no null check. A process file has `wipe_tower_filament` (our flattened U1 process: "0", from the fork's U1 process profiles) but no `nozzle_diameter`, so it dereferences a null pointer. Upstream 2.4.2's normalize_fdm has no such block.
Why:     Explains `single` rc 139 and the same crash on work/models/cube20.stl.
Options: (a) leave `single` as is and use single_project for parity; (b) strip `wipe_tower_filament` from the flattened process file (its value 0 is the default, so the settings are unchanged) -- in flatten_preset.py (the coordinator's file) or in run_u1_tests.sh's official branch.
Chosen:  (a) for now, as asked ("keep single as is"). (b) is a one-line change if the coordinator wants the STL path on the official CLI; not applied.
Evidence: references/SnapmakerOrca/src/Snapmaker_Orca.cpp 1694-1760 (normalize_fdm per file), references/SnapmakerOrca/src/libslic3r/PrintConfig.cpp normalize_fdm "wipe_tower_filament" block; work/u1-tests/out-smapp/_settings/process.json has wipe_tower_filament "0" and no nozzle_diameter.
User-visible effect: none in our build (our CLI resolves presets itself)
Status: provisional (cause inferred from code; not confirmed with a debugger)

## D-135  helper "u1-tests", accepted by coordinator 2026-10-28 21:34
Title: Fork-engine run results (out-snap = build-snapmaker, out-smapp = Snapmaker Orca CLI)
Time: 2026-10-28 21:34
What:    build-snapmaker 9/9 checks pass. Official 6/9: `single` rc 139 (above); `two_plate_snapmaker` rc 232 "Version Check: File Version 2.4.0.1 not supported by current cli version 01.10.01.50" (confirms the allow_newer_file entry: our build accepts, theirs refuses); `mixed_filament` sliced but without a wipe tower.
         compare_gcode official vs ours: IDENTICAL for single_project, four_colour, four_colour_gep, four_colour_highflow, two_plate_portable plate 1, two_plate_snapmaker__allow_newer plate 1. Plate 2 of both two-plate projects: 1 of 5270 moves differs by 0.001 mm in X (G1 X142.664 vs X142.665; extrusion and time equal) -- likely float rounding of the 324 mm plate offset. mixed_filament: DIFFERENT -- ours 21833 moves / net E 1810 mm / 17m01s with a tower, theirs 3014 moves / 607 mm / 7m43s without one; first difference at their start G-code (they emit `M104 S220 T1` before the purge line).
Why:     For the coordinator's parity table.
Evidence: work/u1-tests/out-snap/summary.json, work/u1-tests/out-smapp/summary.json.
User-visible effect: mixed-filament projects and plate-2 objects do not yet match the official CLI
Status: provisional (the user should review the mixed-filament and plate-2 differences)

## D-007  2026-10-28 21:35  coordinator  provisional (needs your review)
What:    For a bare STL (no project), parity with Snapmaker's own CLI is not a target. Parity is measured on projects.
Why:     Given the same flattened presets, their CLI slices single.stl on "Cool Plate" (the engine default; it doesn't apply the
         U1 default plate) and places the part rotated 90° compared to ours. Our build follows the fork's desktop rule
         (Preset::get_default_bed_type → Textured PEI for SM_U1). The same part as a project (single.3mf) is IDENTICAL on both.
Options: (a) match the desktop's defaults (chosen)  (b) copy their CLI's STL behaviour
Evidence: work/u1-tests/out-smapp/single vs out-snap/single (compare_gcode); single_project IDENTICAL
Undo:    n/a

## D-136  helper "snap-fixes", accepted by coordinator 2026-10-28 22:01
Title: Preset names resolve through renamed_from on the Snapmaker build
Time: 2026-10-28 21:58
What:    --printer-preset / --process-preset / --filament-preset, --list-presets --printer, the 3MF rebase check (project_presets_shipped), the settings-file lookup of a system preset by name (find_system_preset: a project's preset, an "inherits" parent) and the printer switch's default-process match all accept a former name: a name a system preset lists in "renamed_from", or the "@"-less form of an "@" preset that lists none. A name given on the command line is replaced by the current one and a new event says so (presets_resolved / PresetRenamed).
Why:     The U1 printers name default_print_profile "0.20 Standard @Snapmaker U1 (0.4 nozzle)", which survives only as renamed_from of "0.20mm Standard @Snapmaker U1 (0.4 nozzle)". The fork itself resolves renamed_from for a project's preset, for inherits and in find_preset2.
Options: (a) resolve everywhere a name is looked up (b) only for the default (c) leave exact
Chosen:  (a) because it is the fork's own map (get_preset_name_renamed) and the brief asks for lookup and default selection.
Evidence: main.cpp current_preset_name / resolve_renamed_preset_name (~5795-5830), project_presets_shipped (~5282); desktop_presets.cpp former_preset_names / renamed_system_preset (~270-350), find_system_preset (~405-416); SnapmakerOrca Preset.cpp 2114-2120, 2662-2668, 2740-2746, 2785-2791, 3394-3405; PresetBundle.cpp 3571-3576, 3670-3687
User-visible effect: an old preset name now works (with a PresetRenamed event) instead of "has no ... preset named"
Status: final

## D-137  helper "snap-fixes", accepted by coordinator 2026-10-28 22:01
Title: Default process of the U1: the renamed 0.20mm Standard
Time: 2026-10-28 21:58
What:    With only --printer-preset "Snapmaker U1 (0.4 nozzle)", the process is "0.20mm Standard @Snapmaker U1 (0.4 nozzle)" (was "0.08mm Standard ..."). defaults_replaced says: "process '0.20mm Standard @...' (the printer's default '0.20 Standard @...' is this preset's former name, its renamed_from)". The printer-switch path for projects scores the default the same way.
Why:     Snapmaker's commit "set default_print_profile to 0.20 standard in U1" shows the intent; the brief asks for renamed_from in default selection.
Options: (a) score the renamed default (b) copy the fork's update_compatible exactly
Chosen:  (a). NOTE for review: read literally, the fork's desktop does NOT do this. update_compatible's PreferedPrintProfileMatch compares preset.name == default_print_profile exactly, load_selections uses select_preset_by_name_strict (exact), so on a fresh install the stale name never matches and first_compatible_idx takes the first compatible visible process, i.e. "0.08mm Standard @Snapmaker U1 (0.4 nozzle)" -- what this build reported before. So this choice follows Snapmaker's evident intent, not the desktop's code path.
Evidence: main.cpp snapmaker_default_process (~5844), call sites in resolve_named_presets and run_list_presets; desktop_presets.cpp desktop_printer_switch_process (~775-790); SnapmakerOrca PresetBundle.cpp 1975-1990, 4140-4185, 4253; Preset.cpp 3281-3300; Preset.hpp 632-656
User-visible effect: default_process / the slice's process for a U1 with no --process-preset
Status: provisional

## D-138  helper "snap-fixes", accepted by coordinator 2026-10-28 22:01
Title: Default filament of the U1: Generic PLA
Time: 2026-10-28 21:58
What:    With no --filament-preset the Snapmaker build takes, in order: the printer's first default_filament_profile (through renamed_from) when compatible; else a compatible preset with that default's alias (the desktop's INT_MAX alias match); else a compatible filament of the printer's own vendor with the default's filament_type (PLA when the default is not shipped): "Generic <type>" first, then another "Generic ...", then the vendor's own; else the first compatible filament. U1 (0.4): "Generic PLA" (was "AliZ PA-CF @System"); defaults_replaced: "filament 'Generic PLA' (the printer's default 'Snapmaker PLA' does not suit this printer; this is Snapmaker's compatible PLA filament, Generic first)".
Why:     "Snapmaker PLA" (inherited from fdm_toolchanger) is not compatible with the U1. The desktop's pick after that is undecidable from code: update_compatible's PreferedFilamentsProfileMatch takes the first compatible VISIBLE preset with filament_type PLA, and visibility depends on which filaments the setup wizard installed (WebGuideDialog section "filaments"); the U1 model names no default_materials, and the wizard-cancel path clears the filaments section, leaving no filament visible ("Default Filament").
Options: (a) brief's fallback rule (vendor, same type, Generic first) (b) first compatible PLA in collection order ("AliZ PLA @System") (c) SM_DEFAULT_FILAMENT "Snapmaker PLA SnapSpeed @U1" (load_selections only applies it to an already-active printer)
Chosen:  (a) per the brief, after the two code-decidable steps.
Evidence: main.cpp snapmaker_default_filament (~5876); SnapmakerOrca PresetBundle.cpp 1775-1828, 2040-2052, 4190-4222, 4256-4285; WebGuideDialog.cpp 660-670, 921-929; resources/profiles/Snapmaker/machine/Snapmaker U1.json (no default_materials), fdm_toolchanger.json
User-visible effect: default_filaments / the slice's filament for a printer whose default filament is missing or incompatible
Status: provisional

## D-139  helper "snap-fixes", accepted by coordinator 2026-10-28 22:01
Title: User-facing engine names on the Snapmaker build
Time: 2026-10-28 21:58
What:    Values only, field names unchanged: result.json/--list-presets/--list-settings "engine" = "snapmaker" (doc lists bambustudio|orcaslicer); layout capabilities / --layout-plan / legacy --layout "engine" = "snapmaker", and --layout-plan input accepts "snapmaker"; --engine-info engines[].app/binary = "Snapmaker Orca"/"slicer_cli-snapmaker", and maker.app = "Snapmaker Orca" for a "Snapmaker_Orca-" file so `recommended` can name this binary; official sentences' app name (SLICER_APP) = "Snapmaker Orca", as Snapmaker_Orca.cpp 134-160 words them; preset refusals, the "cannot print this file" sentence, the per-filament and placeholder sentences, the PA-pattern refusal, --help and the flag refusals say "Snapmaker Orca" / "slicer_cli-snapmaker"; is_engine_binary_name also knows "slicer_cli-snapmaker".
Why:     The build said "orca"/"orcaslicer"/"OrcaSlicer"/"slicer_cli-orcaslicer" about itself.
Options: (a) rename values (b) keep the Orca names
Chosen:  (a) per the brief; docs/cli-reference.md documents `engine` as `bambustudio` or `orcaslicer` -- the field stays, a third value appears (docs not mine to edit; flag for the doc owner).
Evidence: main.cpp 2349, 3042, 3305, 6015, 6124, 6365, 6587, 7276, 8405, ~9300, ~9945, 11682, ~12750, 13094; layout_plan.cpp 292, 401, 568; cli_command_line.cpp 33, 648; cli_flag_support.cpp 11, 86; docs/cli-reference.md 448, 504
User-visible effect: engine strings and messages on the Snapmaker build
Status: final

## D-140  helper "snap-fixes", accepted by coordinator 2026-10-28 22:01
Title: Version gate for Snapmaker Orca projects
Time: 2026-10-28 21:58
What:    engine_file_version on the Snapmaker build reads the version after "Snapmaker_Orca-" or "BambuStudio-" in Application (the fork's loader; it reads no OrcaSlicer tag). file_version_verdict: a Snapmaker-made file (Application "Snapmaker_Orca-" or the "Snapmaker_Orca:3mfVersion" marker) is compared with Snapmaker_VERSION 2.4.1 under the existing newer-file rule (newer major.minor refused unless --allow-newer-file); any other file states a Bambu Studio version and is compared with SLIC3R_VERSION 01.10.01.50 and only warned about (FileNewerThanEngineBase), as the Orca build treats files it did not make.
Why:     Before, the OrcaSlicer tag made an upstream OrcaSlicer 2.4 file (Application "BambuStudio-02.06.00.51") count as "ours" and its Bambu-numbered 2.6 was compared with 2.4.1 and refused: that was the cause of the calib and diagnostic test failures. Snapmaker's own CLI compares everything with SLIC3R_VERSION and refuses its own 2.x files (a bug, not copied).
Options: (a) as above (b) copy the official CLI (c) compare every file with 2.4.1
Chosen:  (a) per the brief.
Evidence: main.cpp engine_file_version (~2097-2116), file_version_verdict (~2158-2190); SnapmakerOrca bbs_3mf.cpp 112-115, 3818-3837, 6645-6650; Snapmaker_Orca.cpp 1431-1436
User-visible effect: Snapmaker Orca 2.4.x projects slice without --allow-newer-file; Bambu Studio / upstream Orca projects newer than 01.10 are warned about instead of refused
Status: provisional

## D-141  helper "snap-fixes", accepted by coordinator 2026-10-28 22:01
Title: Default prime-tower position: the fork's plate middle
Time: 2026-10-28 21:58
What:    tower_default_corner on the Snapmaker build is (13, plate depth / 2) (I3: x 0), plate depth = bed depth in whole mm; assemble_tower_inside_plate with keep_existing=false writes that position unclamped. This applies wherever the CLI places a tower by default: assemble-list plates without need_arrange (desktop rule, unclamped) and with need_arrange (the corner the fork's CLI uses, then its margin and arrange clamps as before). U1 fixed plate: 13/135; arranged plate: 31/135.
Why:     The fork's PartPlateList::set_default_wipe_tower_pos_for_plate and its CLI's assemble blocks use WIPE_TOWER_DEFAULT_X_POS 13 and y = plate_depth*0.5, no clamping in the desktop function. Upstream 2.4's (165, 250) with clamping was being used.
Options: (a) mirror the fork (b) keep upstream
Chosen:  (a). A plain STL/3MF run (no assemble list) keeps the engine default wipe_tower_x/y 15/220 as before, which is what Snapmaker's CLI uses there; the desktop would set its default on plate creation in GUI mode, but that depends on the GUI's plate size at that moment, so it is not changed (see questions).
Evidence: main.cpp tower_default_corner (~4566-4592), assemble_tower_inside_plate (~4655-4668); SnapmakerOrca PartPlate.cpp 70-72, 3799, 4081-4097, 4277, 4686; Snapmaker_Orca.cpp 3940-3970, 4206-4230; PrintConfig.cpp 6190-6205
User-visible effect: prime tower position on assemble-list plates on the Snapmaker build
Status: provisional

## D-142  helper "snap-fixes", accepted by coordinator 2026-10-28 22:01
Title: Mixed filament: keep the desktop's path (tower kept, definitions re-serialised)
Time: 2026-10-28 21:58
What:    No code change. On work/u1-tests/models/mixed_filament.3mf our output differs from Snapmaker's CLI in two ways, each traced to one front-end step:
         (1) mixed_filament_definitions "1,2,1,1,50" -> "1,2,1,1,50,0,g,w,m2,z0,xa0,xb0,d0,o0,u1": the project is Bambu-tagged, so it is loaded over the system presets (ProjectRebasedOnSystemPresets: PresetBundle::load_config_model -> update_multi_material_filament_presets -> mixed_filaments.serialize_custom_entries). The desktop calls the same load_config_model for every loaded project config (Plater.cpp 12103) and slices preset_bundle->full_config(), so it gets the re-serialised string too. Snapmaker's CLI builds no PresetBundle, so it keeps the raw string.
         (2) Prime tower (and ~1200 mm of tower extrusion): Print::apply's normalize_fdm_2 turns enable_prime_tower off on the first apply because the regions do not exist yet and only the virtual filament 3 is counted (PrintApply.cpp ~1781-1800, PrintConfig.cpp 8001-8016); our second apply (PrintReapplied event) counts the regions' physical filaments and keeps the tower. The fork's desktop applies at least twice too: the background timer (Plater.cpp 10791-10795 -> update_background_process 13555) and Slice (reslice 21244 -> update_background_process(true)). Snapmaker's CLI applies once and drops the tower.
Why:     The brief says to target the desktop where the code shows it.
Options: (a) keep (desktop) (b) match Snapmaker's CLI (one apply, no rebase)
Chosen:  (a) because both steps are what the fork's desktop code runs.
Evidence: main.cpp ~8060-8120 (rebase), ~10960-10990 (second apply); SnapmakerOrca PresetBundle.cpp 3881-3917; PrintApply.cpp 1405-1460, 1781-1800; Print.cpp 984-1012; Plater.cpp 10791-10795, 12040-12110, 13535-13560, 21244-21290
User-visible effect: mixed_filament G-code still differs from Snapmaker's CLI (re-run: DIFFERENT, 3950 vs 23049 lines, unchanged by this helper)
Status: provisional (not verified against the desktop app itself)

## D-143  helper "snap-fixes", accepted by coordinator 2026-10-28 22:01
Title: Test fixes for the Snapmaker build
Time: 2026-10-28 21:58
What:    test_diagnostic_events.sh and test_excluded_features.sh know engine "snapmaker" (they read it from layout capabilities): it slices the _orca fixtures and takes the Orca expectations, the PA-pattern refusal names "Snapmaker Orca". The legacy-token checks run on a temporary copy of legacy_token_orca.3mf tagged "Snapmaker_Orca-2.4.1" (no OrcaSlicer tag) whose machine_start_gcode is cut down to its token line.
Why:     calib-* (4), profile-load / rejected-override / plate-out-of-range: real bug, the version gate (fixed above); no test change needed beyond the engine name. legacy-token-* (3): the fixture is an upstream OrcaSlicer 2.4 project; on the Snapmaker build it is a "BambuStudio-" project and is loaded over the system presets (D-111), and with no different_settings_to_system the preset's start G-code replaces the token lines; kept as its own project, its X1C start G-code uses flush_volumetric_speeds, which the fork does not bind. Both are upstream-2.4.2 data assumptions, not front-end bugs.
Options: (a) engine-aware tests (b) change D-111's rebase scope
Chosen:  (a); Orca and Bambu branches unchanged.
Evidence: tests/test_diagnostic_events.sh 64-80; tests/test_excluded_features.sh 90-95, 108-112, 143-178, 199-204, 321-334, 380-382; SnapmakerOrca GCode.cpp 2907
User-visible effect: none
Status: final

## D-144  helper "package", accepted by coordinator 2026-10-28 22:26
Title: Bambu pinned deps built into deps-prefix/ by a new local script
Time: 2026-10-28 21:45
What:    New decision-log/scripts/local-deps-bambu.sh builds the Bambu-patched CGAL 5.4 (git tag v5.4 = c58ac97e…, patch references/BambuStudio/deps/CGAL/0001-clang19.patch, header-only, CGAL_INSTALL_CMAKE_DIR=lib/cmake/CGAL) into deps-prefix/cgal-5.4 and bambulab/libnoise 7e7c98c0… into deps-prefix/libnoise-bambu, with the same marker files install_deps.sh writes. No sudo, nothing outside deps-prefix/ and deps-src/.
Why:     install_deps.sh puts CGAL 5.4 into $(brew --cellar)/cgal@5/5.4 (a fake keg inside Homebrew) and libnoise into /usr/local with sudo (D-001). local-deps.sh is the coordinator's file, so the Bambu steps live in their own script.
Options: (a) new script beside local-deps.sh (b) add the steps to local-deps.sh (not my file) (c) run install_deps.sh (sudo, D-001)
Chosen:  (a) because it is the only option inside my files and D-001.
Evidence: install_deps.sh:29-32, 41-44, 46-128, 274-300; .github/workflows/slicer-cli-ci.yml:426-457 (Linux CGAL 5.4 build flags); decision-log/local-deps-bambu.log
User-visible effect: none
Status: final

## D-145  helper "package", accepted by coordinator 2026-10-28 22:26
Title: libnoise library name check accepts liblibnoise_static.a
Time: 2026-10-28 21:45
What:    The Bambu libnoise fork installs lib/liblibnoise_static.a (as the Orca fork does), not lib/libnoise.a. local-deps-bambu.sh accepts libnoise.a, libnoise_static.a or liblibnoise_static.a plus include/{libnoise,noise}/noise.h, the same test CI uses for its libnoise prefix.
Why:     The first run failed its own install check (install_deps.sh's macOS test only looks for libnoise.a / libnoise.dylib, so on a clean Mac it would rebuild every run; CI's Linux check already allows the static names).
Options: (a) CI's three names (b) install_deps.sh's two names
Chosen:  (a) mechanical.
Evidence: .github/workflows/slicer-cli-ci.yml:104-105, 423-424; install_deps.sh:288-290; deps-prefix/libnoise-bambu/lib/
User-visible effect: none
Status: final

## D-146  helper "package", accepted by coordinator 2026-10-28 22:26
Title: configure.sh picks CGAL and libnoise per engine
Time: 2026-10-28 21:45
What:    configure.sh now selects deps-prefix/cgal-5.4 + libnoise-bambu for bambu, cgal-5.6.3 + libnoise-orca for orca and snapmaker, checks the prefixes exist, and rejects an unknown engine. The orca/snapmaker command lines are byte-for-byte what they were before (no cache change in build-orca/ build-snapmaker/).
Why:     The brief; CI passes CGAL5_DIR for bambu and the engine's own libnoise prefix.
Options: n/a
Chosen:  as CI.
Evidence: decision-log/scripts/configure.sh; .github/workflows/slicer-cli-ci.yml:1199-1207; CMakeLists.txt:227-245
User-visible effect: none
Status: final

## D-147  helper "package", accepted by coordinator 2026-10-28 22:26
Title: Snapmaker build keeps CGAL 5.6.3 although the fork pins CGAL 5.4
Time: 2026-10-28 21:45
What:    configure.sh snapmaker still passes deps-prefix/cgal-5.6.3 (as it did). SNAPMAKER_CGAL=5.4 switches it to deps-prefix/cgal-5.4.
Why:     The fork's own deps build CGAL 5.4 with the same clang19 patch as Bambu (references/SnapmakerOrca/deps/CGAL/CGAL.cmake: v5.4.zip + 0001-clang19.patch, byte-identical to Bambu's patch), so the Snapmaker desktop app is built against CGAL 5.4. But build-snapmaker is not my folder, switching would rebuild every CGAL-dependent engine file under the shared lock while snap-fixes is compiling, and the parity results so far were made with 5.6.3. libnoise needs no change: the fork pins Orca-deps-libnoise tag 1.0, which is commit f25d5331 (git ls-remote), the libnoise-orca we already use.
Options: (a) keep 5.6.3, offer the switch (b) switch snapmaker to 5.4 now (c) leave it unmentioned
Chosen:  (a) because it is the easiest to undo and does not disturb the running work; the coordinator should decide whether to reconfigure build-snapmaker with SNAPMAKER_CGAL=5.4 (CGAL is used by mesh booleans and a few geometry paths, so it can change G-code in rare cases).
Evidence: references/SnapmakerOrca/deps/CGAL/CGAL.cmake:1-14; references/SnapmakerOrca/deps/libnoise/libnoise.cmake:1-3; build-snapmaker/CMakeCache.txt:47
User-visible effect: none today; possibly rare geometry differences from the desktop app
Status: provisional (the user should review)

## D-148  helper "package", accepted by coordinator 2026-10-28 22:26
Title: Package layout and manifest key for the third program
Time: 2026-10-28 21:45
What:    scripts/package-local-macos.sh puts slicer_cli-snapmaker at the package root beside the other two, the fork's profile tree at resources/profiles-snapmaker and its resources root at resources/snapmaker (info/ only), SNAPMAKER-ORCA-LICENSE.txt from references/SnapmakerOrca/LICENSE.txt, and adds { "engine": "snapmaker", "path": "slicer-cli/slicer_cli-snapmaker" } plus "snapmakerProfilesPath": "slicer-cli/resources/profiles-snapmaker" to manifest.json. Version is 0.0.0-local-<HEAD sha12> (CI uses 0.0.0-<sha12> off a tag).
Why:     main.cpp searches those names on the Snapmaker build; CI's naming rule is slicer_cli-<engine-key>, so the key that matches the file name is "snapmaker". The app reads the manifest, so the key is user-visible.
Options: (a) "snapmaker" (b) "snapmakerorca" / "snapmaker_orca"
Chosen:  (a) because it matches the file-name rule in CI's "Stage engine binary" comment.
Evidence: main.cpp:2229-2248 (orca_profiles_dir), 2839-2856 (engine_resources_root); .github/workflows/slicer-cli-ci.yml:1360-1364, 1543-1556
User-visible effect: the package has a third engine entry "snapmaker" in manifest.json; the app would need to know that key.
Status: provisional (the user should review)

## D-149  helper "package", accepted by coordinator 2026-10-28 22:26
Title: No resources/snapmaker/flush: the fork ships none
Time: 2026-10-28 21:45
What:    The package gets resources/snapmaker/info only. The script copies resources/flush too if a later fork has one.
Why:     references/SnapmakerOrca/resources has no flush/ folder and the fork's libslic3r has no FlushVolPredictor and reads no flush/ file (it computes flush volumes in FlushVolCalc, D-119). The fork's libslic3r also reads resources_dir()/profiles/Snapmaker/filament/* (FilamentHotBedNozzleRules, AllowlistManager, FilamentColorLibrary), which resources/snapmaker/ lacks, but only GUI code calls them (Print::filament_rule_mismatch_flags, AllowlistManager::instance); nothing in the CLI path does.
Options: (a) info only (b) also make resources/snapmaker/profiles a link to ../profiles-snapmaker
Chosen:  (a) because nothing the CLI runs reads them; (b) would add a symlink to a package that has none.
Evidence: references/SnapmakerOrca/resources (listing); Print.cpp:1014-1029, 3081, 3116; FilamentHotBedNozzleRules.cpp:25-35; AllowlistManager.cpp:20-34; grep of main.cpp/cli_*.cpp
User-visible effect: none
Status: final

## D-150  helper "package", accepted by coordinator 2026-10-28 22:26
Title: Third-party records for the Snapmaker program
Time: 2026-10-28 21:45
What:    THIRD_PARTY_LICENSES/snapmaker/ holds what bundle-macos.sh records for the Snapmaker program's bundled Homebrew dylibs, exactly as CI does for bambu and orca. No copy of the fork's own third-party notices.
Why:     CI copies only bundle-macos.sh's Homebrew provenance per engine, and the engines' licence files as <ENGINE>-LICENSE.txt; it copies no THIRD_PARTY file from BambuStudio or OrcaSlicer (and the fork has none at its root).
Options: (a) as CI (b) add more
Chosen:  (a)
Evidence: .github/workflows/slicer-cli-ci.yml:1377-1384, 1488-1501, 1530-1531
User-visible effect: none
Status: final

## D-151  helper "package", accepted by coordinator 2026-10-28 22:26
Title: Package validation runs on an extracted copy outside the repository
Time: 2026-10-28 21:45
What:    The script validates the tarball extracted into $TMPDIR, not pkg-local/slicer-cli, and checks that slicer_cli-snapmaker prints "Engine resources: …/slicer-cli/resources/snapmaker".
Why:     From pkg-local/slicer-cli the binaries' ../../references/* candidates (main.cpp engine_profiles_dir, orca_profiles_dir, engine_resources_root) point at the checkout, so a package missing a resource would still pass in place. CI extracts into dist/verify, where ../.. is dist/ and has no references/.
Options: (a) validate outside the repo (b) validate in place
Chosen:  (a)
Evidence: main.cpp:2209-2215, 2236-2240, 2845-2847
User-visible effect: none (tests run by hand against pkg-local/slicer-cli can silently use the checkout's resources)
Status: final

## D-152  helper "package", accepted by coordinator 2026-10-28 22:26
Title: Observed: in-place package tests read the checkout's resources
Time: 2026-10-28 22:25
What:    tests/test-packaged-slice.sh pkg-local/slicer-cli/slicer_cli pkg-local/slicer-cli/slicer_cli-orcaslicer fails (rc 1) on the Orca half: the program prints "Engine resources: …/references/OrcaSlicer/resources" instead of ".../resources/orca". The same test on the tarball extracted outside the repo passes. test-complete-slice-path.sh passes both ways (309 PASS, 4 SKIP).
Why:     pkg-local/slicer-cli/../../ is the repo root, and main.cpp tries the checkout's references/ before the package's resources/ (see "Package validation runs on an extracted copy outside the repository").
Options: (a) validate outside the repo (b) put pkg-local outside the repo
Chosen:  (a): the brief names pkg-local/; the script and my checks validate an extracted copy.
Evidence: decision-log/pkgtest2-packaged-inplace.log, pkgtest2-packaged-extracted.log
User-visible effect: none for a real install
Status: final

## D-153  helper "review-fixes", accepted by coordinator 2026-10-28 22:28
Title: Arrange plate filaments: the fork's 1-based feature filament keys
Time: 2026-10-28 22:27
What:    arrange_plate_extruders on the Snapmaker build reads wall_filament / sparse_infill_filament / solid_infill_filament (1-based; 1 is counted through the volumes) instead of upstream 2.4's six `*_filament_id` keys, which the fork does not define, so arrange_opt returned null and `--arrange 1` crashed (rc 139). Per object: wall = object's value else global, pushed when != 1; sparse = object's value when > 0 else global (resolve_sparse_infill_filament), pushed when != 1; solid = object's value else 1, pushed when != 1, else the global when != 1. Listed with or without support, as the fork's desktop PartPlate::get_extruders does; the fork's CLI get_extruders_under_cli lists the same three only for objects with support (`if (!obj_support) continue;`).
Why:     Confirmed crash: `slicer_cli cube20.stl --printer-preset "Snapmaker U1 (0.4 nozzle)" --arrange 1 --slice 1` exited 139. Callers: arrange_on_bed and downward_plate_bbox (filament count for the tower).
Options: (a) desktop placement (with or without support) (b) CLI placement (support objects only) (c) skip feature filaments
Chosen:  (a) because the Orca block next to it already follows the desktop over the CLI on exactly this split (comment at the Orca block), and the job targets the desktop's G-code. Only the support-less case differs: an object without support whose wall/infill filament is not 1 counts that filament for the tower estimate and arrange under (a), not under (b).
Evidence: main.cpp 3651-3659, 3720-3749 (arrange_plate_extruders); SnapmakerOrca PartPlate.cpp 83-88, 1863-1865, 1899-1937 (get_extruders), 1975-1982, 2042-2081 (get_extruders_under_cli)
User-visible effect: `--arrange 1` (and --downward-check / tower sizing) works on the Snapmaker build instead of crashing; which filaments count for the tower estimate follows the desktop.
Status: provisional (placement (a) vs the fork's CLI (b) is the user's call)

## D-154  helper "review-fixes", accepted by coordinator 2026-10-28 22:28
Title: Arrange/downward box: tower width is prime_tower_width on the Snapmaker build
Time: 2026-10-28 22:27
What:    downward_plate_bbox on the Snapmaker build merges (wipe_x - brim, wipe_y) and (wipe_x + prime_tower_width + brim, wipe_y + estimated depth): the width is the setting, only the depth comes from arrange_estimate_wipe_tower_size. The brim stays on the sides only, as in the Orca branch.
Why:     The fork's CLI check_plate_wipe_tower sets wipe_width = prime_tower_width and takes only wipe_depth = estimate(1) (Snapmaker_Orca.cpp 3219, 3238-3247). The Orca branch used the estimate's x, which the fork's estimate starts from the wipe volume (D-107), not the width.
Options: (a) prime_tower_width (b) estimate x
Chosen:  (a) because it is the fork's own computation.
Evidence: main.cpp 4098-4103 (downward_plate_bbox); SnapmakerOrca Snapmaker_Orca.cpp 3219, 3238-3247
User-visible effect: --downward-check / move-to-new-bed plate box with a tower on the Snapmaker build uses the configured tower width.
Status: final

## D-155  helper "review-fixes", accepted by coordinator 2026-10-28 22:28
Title: Out-of-range feature filaments: clamp the fork's 1-based keys
Time: 2026-10-28 22:27
What:    The FilamentIndexOutOfRangeReset step on the Snapmaker build checks wall_filament / sparse_infill_filament / solid_infill_filament and sets a value below 1 or above the filament count to 1 (reported in the same config_normalized event, "reset" field, without the "0 is Default" note). support_filament / support_interface_filament / wipe_tower_filament clamp to [0, N] as before. The Orca/Bambu branch is unchanged.
Why:     The step looked up the six upstream `*_filament_id` keys, which the fork does not have, so on the Snapmaker build it did nothing. The fork's desktop (PresetBundle::full_fff_config) resets exactly these three keys to 1 when < 1 or > N.
Options: (a) the fork's rule (b) leave it to the engine
Chosen:  (a) because it is the desktop's rule on the settings it slices with.
Evidence: main.cpp 8658-8679, 8707-8711; SnapmakerOrca PresetBundle.cpp 2763-2775. Verified: a Snapmaker_Orca project with wall_filament 3, sparse_infill_filament 0, solid_infill_filament 2 on 2 filaments -> event "sparse_infill_filament 0 -> 1, wall_filament 3 -> 1", G-code header wall_filament = 1, sparse_infill_filament = 1, solid_infill_filament = 2.
User-visible effect: such a project slices with filament 1 for those features and a config_normalized / FilamentIndexOutOfRangeReset event says so.
Status: final

## D-156  helper "review-fixes", accepted by coordinator 2026-10-28 22:28
Title: Colour-list sizing after the filament roster: not on the Snapmaker build
Time: 2026-10-28 22:27
What:    The block after update_multi_material_filament_presets that resizes filament_colour / filament_multi_colour / filament_colour_type when the colour count differs from the filament count is compiled out on the Snapmaker build.
Why:     It dereferenced filament_multi_colour / filament_colour_type, which the fork does not define (latent null dereference). The block exists because upstream's update_multi_material_filament_presets adds a filament per extruder; the fork's does not (that step is under `#if 0`), and the fork's set_num_filaments, called just before, already sizes filament_colour and aligns its own lists (filament_multi_colors, filament_colour_mode). So on the fork the condition does not arise.
Options: (a) exclude on Snapmaker (b) null-guard and keep
Chosen:  (a) because the fork has no step that makes the counts differ, and (b) would still write upstream-only defaults ("1" colour type) the fork does not use.
Evidence: main.cpp 6310-6316; SnapmakerOrca PresetBundle.cpp 116-140, 2225-2250, 3787-3796
User-visible effect: none
Status: final

## D-157  helper "review-fixes", accepted by coordinator 2026-10-28 22:28
Title: filament_ids rewritten only for loaded filaments (match the fork's CLI)
Time: 2026-10-28 22:27
What:    In the Snapmaker filament merge, filament_ids is set from the loaded filament only when load_filament_count > 0; an up_config_to_date-only run keeps the project's filament_ids.
Why:     The fork's CLI writes filament_ids only inside its `if (load_filament_count > 0)` branch.
Options: (a) match the fork (b) keep rewriting in both modes
Chosen:  (a).
Evidence: cli_load_settings.cpp 1440-1451; SnapmakerOrca Snapmaker_Orca.cpp 2656-2674
User-visible effect: with --uptodate and no --load-filaments, result/G-code filament_ids stay the project's (not verified by a run).
Status: final

## D-158  helper "review-fixes", accepted by coordinator 2026-10-28 22:28
Title: Unreadable 3D/3dmodel.model when telling the project's maker: warn
Time: 2026-10-28 22:27
What:    If read_zip_member cannot re-read 3D/3dmodel.model to tell a Bambu Studio project from a Snapmaker Orca one, a `warning` event (tag ProjectMakerUnread, path = the input file) says the project is taken as Snapmaker Orca-made and not rebased. The decision itself (D-111) is unchanged.
Why:     Before, a failed read silently classified the project as Snapmaker-made.
Options: (a) warning event, keep D-111's fallback (b) refuse the file
Chosen:  (a) because the loader has just read that same member, so the failure is unexpected, and changing D-111's outcome is the user's call.
Evidence: main.cpp 8237-8248
User-visible effect: an extra warning event in that (unexpected) case only. Not exercised by a run.
Status: final

## D-159  helper "review-fixes", accepted by coordinator 2026-10-28 22:28
Title: renamed_from map: log skipped files; duplicate former name goes to the smaller preset name
Time: 2026-10-28 22:27
What:    renamed_system_preset logs (Boost.Log) every file or entry it skips: unlistable profiles folder, vendor JSON or preset file that is not readable JSON or not an object, list entry with no name or no file -> warning; vendor file without the list, preset with instantiation false -> debug. When two presets claim the same former name, the one with the smaller preset name keeps it (was: first in vendor-file order, then list order) and the clash is logged at warning, worded like the fork's error.
Why:     Review: silent skips. The fork builds the map by iterating its preset collection, which is sorted by name (Preset::operator<), and keeps the first insert.
Options: (a) smaller name wins (fork) (b) vendor/list order (before)
Chosen:  (a) because it is the fork's rule. Warning, not error, as the review allowed; warning records reach the event stream as engine_log events.
Evidence: desktop_presets.cpp 17, 291-394; SnapmakerOrca Preset.cpp 3394-3405, Preset.hpp 324. Checked with a script over references/SnapmakerOrca/resources/profiles: no former name is claimed twice and no file is skipped as malformed (BOM-prefixed files parse), so no new log lines and no change with the shipped profiles.
User-visible effect: none with the shipped profiles; engine_log warnings for a broken profiles tree.
Status: final

## D-160  2026-10-28 22:29  coordinator  final (deliberate exception to "Bambu/Orca unchanged"; please review)
What:    is_engine_binary_name (main.cpp ~7322) lists slicer_cli-snapmaker on every build, not only the Snapmaker one.
Why:     Found by helper "package": in a 3-program package, the Bambu and Orca programs' stale-temp-folder sweep deleted a
         running slicer_cli-snapmaker's folders (it failed with rc 255, "create_directory: …/slicer_cli_list-…/user").
         The only change for Bambu/Orca: they now also keep folders whose live owner is slicer_cli-snapmaker.
Options: (a) one list for all builds  (b) leave it; the package can't run the programs side by side safely
Evidence: decision-log/inbox/merged/package-questions.md; main.cpp is_engine_binary_name
Undo:    restore the #ifdef ENGINE_SNAPMAKER split

## D-161  2026-10-28 22:29  coordinator  final
What:    The Snapmaker build uses CGAL 5.4 with the clang19 patch (deps-prefix/cgal-5.4), not 5.6.3.
Why:     That is what the fork's own deps pin (references/SnapmakerOrca/deps/CGAL), so it is closest to Snapmaker's binaries.
         Raised by helper "package". Parity is re-checked after the rebuild.
Undo:    SNAPMAKER_CGAL=5.6.3 decision-log/scripts/configure.sh snapmaker build-snapmaker

## D-162  2026-10-28 22:35  coordinator  final
What:    Apply the existing CGAL 5.4 compatibility header (libslic3r/bambustudio/cgal_54_compat.hpp) to the Snapmaker build
         when its CGAL is below 5.5.
Why:     With D-161 (CGAL 5.4), the Snapmaker build failed in Triangulation.cpp (boost::mpl::if_c / CGAL::is_zero undeclared):
         the same CGAL-5.4-with-new-Boost problem the Bambu build already fixes with this header.
Evidence: decision-log/final/build-snapmaker.log
Undo:    revert the CMakeLists.txt condition (~line 703)
