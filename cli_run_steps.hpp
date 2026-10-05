// cli_run_steps.hpp — the official CLI's transform and action steps.
//
// CLI::run works through the command line in this order (BambuStudio.cpp at
// 5873b5f; OrcaSlicer.cpp at 31f6803):
//   1. load the model files and the settings      (main.cpp, cli_model_load.cpp)
//   2. the transforms, in command-line order      apply_transforms
//        BambuStudio.cpp 4929-5208 / OrcaSlicer.cpp 4190-4470
//   3. orient the objects asked for               apply_transforms (end)
//        BambuStudio.cpp 5213-5234 / OrcaSlicer.cpp 4474-4495
//   4. arrange and duplicate                      main.cpp, cli_repetitions.cpp
//   5. ensure_on_bed                              ensure_on_bed_if_asked
//        BambuStudio.cpp 6188-6194 / OrcaSlicer.cpp 5443-5455
//   6. the actions, in command-line order         run_model_actions, then the
//        BambuStudio.cpp 6336-6437 / OrcaSlicer.cpp 5471-5561   plate loop
// Each function below is one of those steps; main.cpp calls them in that order.
#pragma once

#include <string>
#include <vector>

#include "libslic3r/Model.hpp"
#include "libslic3r/PrintConfig.hpp"

#include "cli_options.hpp"

namespace slicer_cli {

struct StepResult {
    int         code = 0;    // 0, or the official CLI_* code
    std::string message;
};

/// Step 2 and 3: the transforms in command-line order, then orient.
/// `process_config` is the process loaded with --load-settings, which the
/// official orient applies to each object first (BambuStudio.cpp 5220-5223).
/// `duplicate_count` receives --repetitions minus one when it applies.
StepResult apply_transforms(const CliOptions& o, Slic3r::Model& model,
                            const Slic3r::DynamicPrintConfig* process_config, int plate_to_slice,
                            int plate_count, int& duplicate_count);

/// Step 5: --ensure-on-bed.
void ensure_on_bed_if_asked(const CliOptions& o, Slic3r::Model& model);

/// True when the command line asks for an action that works on the loaded
/// model and settings rather than slicing (--info, --export-settings,
/// --export-stl, --export-stls).
bool has_model_actions(const CliOptions& o);

/// True when the command line asks for no slice at all: model actions only.
bool model_actions_only(const CliOptions& o);

/// Step 6 before the plate loop: --export-settings, --info, --export-stl and
/// --export-stls, in command-line order.
StepResult run_model_actions(const CliOptions& o, Slic3r::Model& model, const Slic3r::DynamicPrintConfig& config);

/// The per-plate limits and switches the actions set for the plate loop.
struct PlateLoopSwitches {
    int  max_triangle_count = 0;   // --mtcpp
    int  max_slicing_time_s = 0;   // --mstpp
    bool no_check           = false;
    bool allow_mix_temp     = false;
    bool normative_check    = true;
    bool min_save           = false;
};
PlateLoopSwitches plate_loop_switches(const CliOptions& o);

/// --normative-check (on unless --normative-check 0): the checks the official
/// CLI makes on a 3MF's settings (BambuStudio.cpp 1948-1965; OrcaSlicer.cpp
/// 1609-1619). Empty when the settings pass; else the code and the sentence.
StepResult normative_check(const CliOptions& o, const Slic3r::DynamicPrintConfig& file_config);

/// --skip-objects: marks the listed objects (by their 3MF identify_id) not
/// printable (BambuStudio.cpp 6541-6575). Returns how many were skipped,
/// or -1 when every object on the plate was skipped (CLI_NO_SUITABLE_OBJECTS_AFTER_SKIP).
int apply_skip_objects(const CliOptions& o, Slic3r::Model& model, std::vector<int>& skipped_ids);

/// --mtcpp: the triangle count of the plate's printable instances inside
/// the bed is over the limit; each instance counts its object's parts
/// (BambuStudio.cpp 6541-6597; OrcaSlicer.cpp 5657-5720). Run after the
/// bed check has set each instance's print_volume_state.
StepResult check_triangle_limit(const CliOptions& o, const Slic3r::Model& model, int plate_id);

/// --load-slicedata and --export-slicedata: the plate's folder under the
/// given directory ("<dir>/<plate>", BambuStudio.cpp 7077, 7248; OrcaSlicer.cpp
/// 6068, 6186), or empty when the flag is not given.
std::string slicedata_dir(const CliOptions& o, const char* key, int plate);

/// The combinations the official command line refuses with CLI_INVALID_PARAMS:
/// --load-slicedata with --repetitions (BambuStudio.cpp 6354-6359; OrcaSlicer.cpp
/// 5487-5492) and, in OrcaSlicer only, --load-slicedata with --export-slicedata
/// (OrcaSlicer.cpp 5481-5486, 5555-5560; commented out in BambuStudio.cpp 6348-6353).
StepResult check_slicedata_flags(const CliOptions& o, int duplicate_count);

/// --metadata-name/--metadata-value and --makerlab-name/--makerlab-version on
/// the model, for the exported 3MF (BambuStudio.cpp 8137-8152).
void apply_model_metadata(const CliOptions& o, Slic3r::Model& model);

} // namespace slicer_cli
