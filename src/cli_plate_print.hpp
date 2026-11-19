#pragma once

#include <chrono>
#include <optional>
#include <string>

struct CliOptions;
namespace Slic3r { struct Calib_Params; class DynamicPrintConfig; }

namespace slicer_cli {

enum class ActionPhase;
struct CliRunContext;
struct RunState;
struct PlateOutcome;

struct PreparedPlate {
    CliRunContext& context;
    const CliOptions& o;
    RunState& rs;
    Slic3r::DynamicPrintConfig& config;
    const std::optional<Slic3r::DynamicPrintConfig>& project_settings;
    PlateOutcome& outcome;

    bool run_model_action_step(ActionPhase phase);
    bool outputs_ok(bool after_slice_actions_only = false);
    int print_plate(Slic3r::Calib_Params& calib_params, int plate_id, const std::string& output_file,
                    std::chrono::steady_clock::time_point plate_started, bool calib_enabled);
};

void pipe_files_loaded(RunState& rs);
void pipe_prepare_slicing(RunState& rs);

} // namespace slicer_cli
