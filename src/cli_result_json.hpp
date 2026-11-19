#pragma once

#include <map>
#include <vector>
#include <string>
#include <memory>
#include <nlohmann/json.hpp>
#include "libslic3r/Utils.hpp"

struct CliOptions;
namespace Slic3r { struct PlateData; struct PlateBBoxData; }

namespace slicer_cli {

struct CliRunContext;
struct SettingsMerge;

#ifndef CLI_FILAMENT_UNPRINTABLE_ON_FIRST_LAYER
#define CLI_FILAMENT_UNPRINTABLE_ON_FIRST_LAYER -103
#endif
#ifndef CLI_GCODE_PATH_OUTSIDE
#define CLI_GCODE_PATH_OUTSIDE             -104
#endif
#ifndef CLI_GCODE_IN_WRAPPING_DETECT_AREA
#define CLI_GCODE_IN_WRAPPING_DETECT_AREA  -105
#endif

// ── Per-plate outcome ───────────────────────────────────────────────────
// The command line itself is CliOptions (cli_options.hpp), filled by
// cli_command_line.cpp.

/// What one plate's slice produced, in the shape the official CLI records per
/// plate (sliced_plate_info_t, BambuStudio.cpp 191-219 at 5873b5f).
struct PlateOutcome {
    int         plate_id   = 0;
    int         plate_index = 1;   // 1-based position in this run, for progress
    int         plate_count = 1;   // plates this run slices, for progress
    int         cli_code   = 0;    // CLI_SUCCESS, or the official CLI_* code of the failure
    std::string error_string;      // the official sentence for cli_code, plus specifics
    std::string refusal_note;      // appended to any refusal of this plate (set_outcome_failure)
    std::string error_reason;      // why it failed, beyond error_string: result.json error_reason
    std::string gcode_path;
    bool        exported   = false;

    // sliced_plate_info_t fields (BambuStudio.cpp 191-219)
    long long   sliced_time_ms = 0;
    std::string warning_message;
    float       total_predication = 0.f;
    float       main_predication  = 0.f;
    int         filament_change_times = 0;
    size_t      triangle_count = 0;
    std::vector<std::pair<std::string, float>> feature_type_times;
    nlohmann::json        objects   = nlohmann::json::array();
    nlohmann::json        filaments = nlohmann::json::array();
    nlohmann::json        warnings  = nlohmann::json::array();   // every slicing warning, not only the last
    // A printer with more than one nozzle: the nozzle (1-based logical
    // extruder) each filament printed with, and the mode that chose it.
    // Empty on a one-nozzle printer.
    std::vector<int> filament_maps;
    std::string      filament_map_mode;
    // A nozzle rack: the rack nozzle each filament printed with, as the
    // G-code's filament_nozzle_map states it (0-based). Empty without a rack.
    std::vector<int> filament_nozzle_map;
    std::vector<std::string> unknown_settings;  // keys in the file this engine has no definition for
    // Keys whose value in the project file was replaced by the preset's when
    // the project was loaded the way its desktop app loads it.
    std::vector<std::string> settings_replaced_by_presets;
    std::map<std::string, std::vector<std::string>> replaced_file_values;  // each one's value in the file
    // sliced_info_t print summary (BambuStudio.cpp 221-231)
    float       layer_height = 0.f;
    int         wall_loops   = 0;
    float       sparse_infill_density = 0.f;
    // Trusted identifiers from catalog-validated named preset selection.
    // Empty means this run did not select presets by name.
    std::string printer_preset;
    std::string process_preset;
    std::vector<std::string> filament_presets;

    // --export-3mf: what the sliced project needs from this plate.
    std::shared_ptr<Slic3r::PlateData>          plate_data;     // slice result, as the GUI stores it
    // Bambu build: routing the slice derived from a supplied filament_nozzle_map
    // (apply_explicit_nozzle_mapping); empty when nothing was derived.
    std::vector<int>                            derived_filament_map;
    std::string                                 derived_filament_map_mode;  // "Manual" or "Nozzle Manual"
    std::map<std::string, std::vector<int>>     flag_plate_maps;   // filament_volume_map / filament_nozzle_map from the flags
    // The run steps (cli_run_steps.cpp): --repetitions copies, the objects
    // --skip-objects left out (plate_N.json skipped_objects), --no-check, and
    // a run of model actions only (no G-code).
    int                                         duplicate_count = 0;
    // --downward-check: the printers checked and those this plate does not
    // fit, and whether it prints by object (BambuStudio.cpp 4785-4914).
    bool                                        downward_checked = false;
    std::vector<std::string>                    downward_printers;
    std::vector<std::string>                    downward_failed;
    bool                                        sequence_plate = false;
    // The project's filaments select more than one nozzle diameter
    // (BambuStudio.cpp 5241, 5392-5393).
    bool                                        multi_nozzle_diameters = false;
    std::vector<int>                            skipped_objects;
    bool                                        no_check = false;
    bool                                        actions_only = false;
    bool                                        run_step_failed = false;  // a model action failed: reported as plate 0
    // --slice 0 on several plates: the check pass, which stops before the
    // slice (run_slice_mode).
    bool                                        pre_check = false;
    // --slice: the pass that loads the run's one model and settings and runs
    // the steps the official CLI runs once before its plate loop (the
    // transforms, the arrange, the actions before --slice; RunState), before
    // any plate is checked or sliced.
    bool                                        prepare_pass = false;
    // --load-settings and friends: what the merge decided, for the export.
    std::shared_ptr<slicer_cli::SettingsMerge>  settings_merge;
    // --export-3mf: the plate's Metadata/plate_N.json (first-layer boxes).
    std::shared_ptr<Slic3r::PlateBBoxData>      plate_bbox;
};

std::string assemble_list_file(const CliOptions& o);
std::string cli_error_sentence(int code);
void set_outcome_failure(PlateOutcome& outcome, int code, const std::string& detail = {});
std::vector<std::string> named_input_files(CliRunContext& context, const CliOptions& o);
bool write_result_json(CliRunContext& context, const std::string& outputdir, int code, int plate_id,
                              const std::string& error_string,
                              const std::vector<PlateOutcome>& outcomes,
                              long long prepare_time_ms, long long export_time_ms);
void emit_progress(const PlateOutcome& outcome, int plate_percent, const std::string& message);
void emit_run_progress(const PlateOutcome& last, int total, const std::string& message);

void announce_noted_warning(const nlohmann::json& announcement);
void take_noted_warnings(PlateOutcome& outcome, const std::map<int, nlohmann::json>& noted, int plate_id);

} // namespace slicer_cli
