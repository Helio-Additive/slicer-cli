# Porting log

| File:line | What broke | What we did | Decision |
|-|-|-|-|

### From helper frontend
| file:line | what broke | what we did | decision title |
|---|---|---|---|
| cli_run_steps.cpp:24 | SoftFever_VERSION undeclared | Snapmaker_VERSION on Snapmaker | Version macro on the Snapmaker build |
| layout_plan.cpp:159 | coPointsGroups undeclared | '#' separator branch compiled out | No points groups, per-extruder areas or expert mode in the fork |
| cli_assemble.cpp:131 | obj_color_deal_algo undeclared | convert_obj_cluster_colors compiled out | Assembled OBJ colours |
| cli_assemble.cpp:399 | ObjInfo::lost_material_name missing | missing-material and face-colour refusals compiled out | Assembled OBJ colours |
| cli_assemble.cpp:415 | (colour clustering path) | plain mesh, listed filament | Assembled OBJ colours |
| main.cpp:159 | STRING_EXCEPT_FILAMENTS_DIFFERENT_TEMP missing | fork enum names | Fork names for StringExceptionType |
| main.cpp:248 | GCodeProcessorResult::gcode_check_result missing | emit_gcode_check_result compiled out | No G-code check, first-layer printable or limited-map results on the fork |
| main.cpp:319 | filament_printable_reuslt missing | event compiled out | No G-code check, first-layer printable or limited-map results on the fork |
| main.cpp:349 | limit_filament_maps missing | event compiled out | No G-code check, first-layer printable or limited-map results on the fork |
| main.cpp:446 | StringObjectException::is_warning missing | caller passes is_warning | Validation event kind without StringObjectException::is_warning |
| main.cpp:800 | coPointsGroups missing | ',' separator only | No points groups, per-extruder areas or expert mode in the fork |
| main.cpp:1962 | filament_options_with_variant missing | test compiled out | No filament variant tables in the fork |
| main.cpp:2114 | SoftFever_VERSION undeclared | Snapmaker_VERSION | Version macro on the Snapmaker build |
| main.cpp:2231 | (non-error) Orca profiles tree | SnapmakerOrca / profiles-snapmaker | Profiles tree and engine resources root |
| main.cpp:2842 | (non-error) Orca resources root | SnapmakerOrca/resources / resources/snapmaker | Profiles tree and engine resources root |
| main.cpp:3040 | CLI_GCODE_PATH_IN_UNPRINTABLE_AREA undeclared | table entry compiled out | CLI result codes the fork does not have |
| main.cpp:3073 | CLI_FILAMENT_CAN_NOT_MAP / CLI_ONLY_ONE_TPU_SUPPORTED undeclared | table entries compiled out | CLI result codes the fork does not have |
| main.cpp:3438 | ConfigOptionPointsGroups; BuildVolume 4-arg ctor | BuildVolume(area, height) | No points groups, per-extruder areas or expert mode in the fork |
| main.cpp:3828 | (estimate) | x starts at wipe volume | Prime tower size estimate: the fork's own |
| main.cpp:3854 | filament_change_length, get_limit_depth_by_height missing | fork's estimate body | Prime tower size estimate: the fork's own |
| main.cpp:4002 | WipeTower::get_auto_brim_by_height missing | auto brim compiled out | Prime tower size estimate: the fork's own |
| main.cpp:4117 | ConfigOptionPointsGroups missing | shared area compiled out | No points groups, per-extruder areas or expert mode in the fork |
| main.cpp:4356 | get_auto_brim_by_height missing | auto brim compiled out | Prime tower size estimate: the fork's own |
| main.cpp:4362 | (clamp width) | w kept as prime_tower_width | Prime tower size estimate: the fork's own |
| main.cpp:4622 | ConfigOptionPointsGroups missing | extruder-area narrowing compiled out | No points groups, per-extruder areas or expert mode in the fork |
| main.cpp:4644 | get_auto_brim_by_height missing | auto brim compiled out | Prime tower size estimate: the fork's own |
| main.cpp:4695 | get_auto_brim_by_height missing | auto brim compiled out | Prime tower size estimate: the fork's own |
| main.cpp:4869 | total_filament_changes missing | total_filamentchanges | Filament change count field |
| main.cpp:4931 | PlateData/GCodeProcessorResult map fields, Print::get_filament_maps missing | not copied | PlateData filament-map fields absent |
| main.cpp:5095 | gcode_check_result, filament_printable_reuslt missing | post-slice refusals compiled out | No G-code check, first-layer printable or limited-map results on the fork |
| main.cpp:6077 | full_config(bool) missing | full_config() | full_config() takes no argument |
| main.cpp:6238 | coPointsGroups, coIntsGroups missing | cases compiled out | No points groups, per-extruder areas or expert mode in the fork |
| main.cpp:6250 | comExpert missing | case compiled out | No points groups, per-extruder areas or expert mode in the fork |
| main.cpp:7279 | load_bbs_3mf has no is_orca_3mf | Bambu signature | load_bbs_3mf signature and which projects are rebased |
| main.cpp:7883 | load_bbs_3mf has no is_orca_3mf | Bambu signature, is_orca_3mf from Application metadata | load_bbs_3mf signature and which projects are rebased |
| main.cpp:8077 | full_config(bool) missing | full_config() | full_config() takes no argument |
| main.cpp:9793 | PlateData::filament_maps missing | plate filament-map block compiled out | PlateData filament-map fields absent |
| main.cpp:10781 | Print::wipe_tower_type / WipeTowerType missing | is_BBL_printer() | Type-1 tower height fix without wipe_tower_type() |
| main.cpp:10879 | Print::set_check_multi_filaments_compatibility missing | Snapmaker CLI temperature-mix guard | Filament temperature mix and --allow-mix-temp |
| main.cpp:10920 | (guard result) | mix refusal overrides validate() result | Filament temperature mix and --allow-mix-temp |
| main.cpp:10956 | is_warning missing | warning passed explicitly | Validation event kind without StringObjectException::is_warning |
| main.cpp:10981 | STRING_EXCEPT_FILAMENTS_DIFFERENT_TEMP missing | MIXING_TEMP | Fork names for StringExceptionType |
| main.cpp:11531 | PlateData map fields missing | not copied | PlateData filament-map fields absent |

### From helper presets
| file:line | what broke | what we did | decision title |
|---|---|---|---|
| cli_load_settings.cpp:267 | set_with_restore | Snapmaker: keep changed key, no variant restore | Settings merge: no per-extruder variant restore |
| cli_load_settings.cpp:344 | (new) | rewrite_flow_segment helper | Filament merge: the fork's composed flow-variant layout |
| cli_load_settings.cpp:405 | nozzle_volume read as Floats (silently 0) | read fork's scalar | Flush volumes: the fork's one-block matrix |
| cli_load_settings.cpp:468 | ExtruderType, etDirectDrive, nvtStandard, get_extruder_variant_string | skip variant strings | Variant keys the fork does not define are never created |
| cli_load_settings.cpp:1120 | printer_options_with_variant_1/2 | fork-style call | Settings merge: no per-extruder variant restore |
| cli_load_settings.cpp:1135 | printer_options_with_variant_1/2 | fork-style call | Settings merge: no per-extruder variant restore |
| cli_load_settings.cpp:1183 | printer_extruder_variant create -> null (runtime) | skipped | Variant keys the fork does not define are never created |
| cli_load_settings.cpp:1220 | print_options_with_variant, empty_options | fork-style call | Settings merge: no per-extruder variant restore |
| cli_load_settings.cpp:1233 | print_options_with_variant, empty_options | fork-style call | Settings merge: no per-extruder variant restore |
| cli_load_settings.cpp:1241 | print_extruder_variant create -> null (runtime) | skipped | Variant keys the fork does not define are never created |
| cli_load_settings.cpp:1254 | nvtStandard, ExtruderType, get_extruder_variant_string | nozzle-volume/variant step skipped | Variant keys the fork does not define are never created |
| cli_load_settings.cpp:1407 | filament_options_with_variant, set_with_restore_2, filament_extruder_variant, filament_self_index (null at runtime) | fork CLI merge + desktop flow segments | Filament merge: the fork's composed flow-variant layout |
| cli_load_settings.cpp:1740 | nozzle-type comparison (vars no longer exist) | fork's condition | Flush volumes: the fork's one-block matrix |
| cli_load_settings.cpp:1782 | ExtruderType, nvtStandard, get_index_for_extruder, get/set_flush_volumes_matrix, nozzle_flush_dataset, flush_multiplier vector | fork's matrix block | Flush volumes: the fork's one-block matrix |
| cli_load_settings.cpp:1954 | update_values_from_multi_to_multi_2, print_options_with_variant | no-op | Printer switch leaves object settings alone |
| cli_load_settings.cpp:2032 | ConfigOptionPointsGroups | return false | Downward check: no shared extruder box, no wrapping area |
| cli_load_settings.cpp:2124 | wrapping_exclude_area create -> null (runtime) | empty area | Downward check: no shared extruder box, no wrapping area |
| desktop_presets.cpp:12 | (new) | include predicate.hpp | Plate type: the fork's get_default_bed_type, U1 clamp |
| desktop_presets.cpp:40 | not_support_bed_types | offered_bed_types/offered not compiled | Plate type: the fork's get_default_bed_type, U1 clamp |
| desktop_presets.cpp:424 | wrong name for btGESP/btSuperTack | keys-map lookup | bed_type_name by the enum keys map |
| desktop_presets.cpp:444 | (follows not_support_bed_types) | fork's plate rule | Plate type: the fork's get_default_bed_type, U1 clamp |

### From helper snap-fixes
| file:line | what broke | what we did | decision title |
|-|-|-|-|
| main.cpp ~2097-2116 engine_file_version | Snapmaker_Orca- tag not read; OrcaSlicer tag read (fork ignores it) | Snapmaker: Application "Snapmaker_Orca-"/"BambuStudio-" only | Version gate for Snapmaker Orca projects |
| main.cpp ~2158-2190 file_version_verdict | upstream Orca file refused (Bambu 2.6 vs 2.4.1) | Snapmaker-made vs other, SLIC3R_VERSION warn rule | Version gate for Snapmaker Orca projects |
| main.cpp 2349 engine_fit_for | names OrcaSlicer/slicer_cli-orcaslicer | Snapmaker Orca/slicer_cli-snapmaker | User-facing engine names |
| main.cpp 3042 SLICER_APP | "Orca Slicer" | "Snapmaker Orca" | User-facing engine names |
| main.cpp 3305, 6365, 6587 engine field | "orcaslicer" | "snapmaker" | User-facing engine names |
| main.cpp ~5282 project_presets_shipped | former names not found | renamed_from accepted | Preset names resolve through renamed_from |
| main.cpp ~5795-5960 new helpers | no renamed lookup, wrong defaults | current_preset_name, resolve_renamed_preset_name, snapmaker_default_process/_filament | Preset names resolve / Default process / Default filament |
| main.cpp 6015, 6124 app name | "OrcaSlicer" | "Snapmaker Orca" | User-facing engine names |
| main.cpp ~6024-6031, ~6158-6165 | old names refused | names replaced with current ones (event) | Preset names resolve through renamed_from |
| main.cpp ~6100 resolve_named_presets signature | needs to rewrite names | non-const CliOptions on Snapmaker | Preset names resolve through renamed_from |
| main.cpp ~6183-6230, ~6410-6440 | 0.08mm / AliZ PA-CF defaults | Snapmaker default picks | Default process / Default filament |
| main.cpp ~6345 run_list_presets | old printer name | current_preset_name | Preset names resolve through renamed_from |
| main.cpp 7276 is_engine_binary_name | no slicer_cli-snapmaker | added | User-facing engine names |
| main.cpp 8405, ~9300, ~9945, ~12750 messages | "OrcaSlicer" | "Snapmaker Orca" | User-facing engine names |
| main.cpp 11682 engine-info maker.app | recommended never matched | "Snapmaker Orca" for Snapmaker_Orca- files | User-facing engine names |
| main.cpp 13094 legacy --layout engine | "orca" | "snapmaker" | User-facing engine names |
| main.cpp ~4566-4592 tower_default_corner | upstream (165,250) | fork (13, depth/2) | Default prime-tower position |
| main.cpp ~4655-4668 assemble_tower_inside_plate | clamped | fork default unclamped | Default prime-tower position |
| desktop_presets.cpp ~270-350 | no renamed map | former_preset_names, renamed_system_preset | Preset names resolve through renamed_from |
| desktop_presets.cpp ~405-416 find_system_preset | former names not found | renamed fallback | Preset names resolve through renamed_from |
| desktop_presets.cpp ~775-790 desktop_printer_switch_process | stale default never matched | former names match the default | Default process of the U1 |
| layout_plan.cpp 292, 401, 568 | "orca" | "snapmaker" (input accepts it) | User-facing engine names |
| cli_command_line.cpp 33, 648 | OrcaSlicer in --help | Snapmaker Orca | User-facing engine names |
| cli_flag_support.cpp 11, 86 | OrcaSlicer in refusals | Snapmaker Orca | User-facing engine names |
| tests/test_diagnostic_events.sh 64-80 | engine "snapmaker" unknown | snapmaker case | Test fixes for the Snapmaker build |
| tests/test_excluded_features.sh 90-112, 143-178, 321-382 | engine "snapmaker" unknown; token fixture | snapmaker cases, retagged token copy | Test fixes for the Snapmaker build |

### From helper package
| file:line | what broke | what we did | decision title |
|---|---|---|---|
| decision-log/scripts/local-deps-bambu.sh (new) | no local, no-sudo recipe for Bambu's CGAL 5.4 + libnoise | build both into deps-prefix/ with install_deps.sh's markers | Bambu pinned deps built into deps-prefix/ by a new local script |
| decision-log/scripts/local-deps-bambu.sh:40-44 | libnoise install check looked for libnoise.a only | accept CI's three library names | libnoise library name check accepts liblibnoise_static.a |
| decision-log/scripts/configure.sh:14-22 | one CGAL/libnoise for every engine | per-engine prefixes, existence checks | configure.sh picks CGAL and libnoise per engine |
| decision-log/scripts/configure.sh:18 | fork pins CGAL 5.4, build uses 5.6.3 | keep 5.6.3, SNAPMAKER_CGAL=5.4 switch | Snapmaker build keeps CGAL 5.6.3 although the fork pins CGAL 5.4 |
| scripts/package-local-macos.sh (new) | no local package with the third program | CI's assemble step + slicer_cli-snapmaker, profiles-snapmaker, resources/snapmaker, licence, manifest entry | Package layout and manifest key for the third program |

### From helper review-fixes
| file:line | what broke | what we did | decision title |
|-|-|-|-|
| main.cpp 3651-3659 arrange_plate_extruders | `*_filament_id` keys absent in fork -> null deref, --arrange rc 139 | Snapmaker: global wall/sparse/solid filament keys | Arrange plate filaments: the fork's 1-based feature filament keys |
| main.cpp 3720-3749 arrange_plate_extruders | same, per object | Snapmaker: fork desktop per-object rules | Arrange plate filaments: the fork's 1-based feature filament keys |
| main.cpp 4098-4103 downward_plate_bbox | tower width taken from the estimate | Snapmaker: prime_tower_width, depth from estimate | Arrange/downward box: tower width is prime_tower_width on the Snapmaker build |
| main.cpp 6310-6316 colour sizing block | filament_multi_colour / filament_colour_type absent in fork | excluded on Snapmaker | Colour-list sizing after the filament roster: not on the Snapmaker build |
| main.cpp 8237-8248 maker detection | read failure silent | warning event ProjectMakerUnread | Unreadable 3D/3dmodel.model when telling the project's maker: warn |
| main.cpp 8658-8679, 8707-8711 FilamentIndexOutOfRangeReset | upstream keys absent -> no-op | Snapmaker: clamp 1-based keys to 1 | Out-of-range feature filaments: clamp the fork's 1-based keys |
| cli_load_settings.cpp 1440-1451 filament merge | filament_ids rewritten in up_config_to_date mode | only when load_filament_count > 0 | filament_ids rewritten only for loaded filaments (match the fork's CLI) |
| desktop_presets.cpp 17, 291-394 renamed_system_preset | silent skips; duplicate former name by vendor order | Boost.Log logging; smaller name wins | renamed_from map: log skipped files; duplicate former name goes to the smaller preset name |
