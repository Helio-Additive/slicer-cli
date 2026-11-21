#pragma once

#include <chrono>
#include <optional>
#include <string>

struct CliOptions;
namespace Slic3r { struct Calib_Params; class DynamicPrintConfig; }

namespace slicer_cli {

enum class ActionPhase;
struct CliRunContext;
struct LoadedProject;
struct RunSettings;
struct RunPlatePlan;
struct RunPipeProgress;
struct RunActionOutputs;
struct PlateOutcome;

struct PreparedPlate {
    CliRunContext& context;
    const CliOptions& o;
    LoadedProject& loaded_project;
    RunSettings& run_settings;
    RunPlatePlan& plate_plan;
    RunPipeProgress& pipe_progress;
    RunActionOutputs& action_outputs;
    Slic3r::DynamicPrintConfig& config;
    const std::optional<Slic3r::DynamicPrintConfig>& project_settings;
    PlateOutcome& outcome;

    bool prepare_calibration_geometry(Slic3r::Calib_Params& calib_params);
    bool run_model_action_step(ActionPhase phase);
    bool outputs_ok(bool after_slice_actions_only = false);
    int print_plate(Slic3r::Calib_Params& calib_params, int plate_id, const std::string& output_file,
                    std::chrono::steady_clock::time_point plate_started, bool calib_enabled);
};

void pipe_files_loaded(RunPipeProgress& progress);
void pipe_prepare_slicing(RunPipeProgress& progress);

} // namespace slicer_cli
