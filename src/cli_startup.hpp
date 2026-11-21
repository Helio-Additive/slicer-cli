#pragma once

#include <optional>
#include <string>

struct CliOptions;
namespace Slic3r { struct Calib_Params; }

namespace slicer_cli {
struct CliRunContext;
struct ModeArgs;
struct CalibOptions;

std::optional<int> run_timing_or_initialize_logging(int argc, char** argv);

std::optional<int> parse_startup_options(CliRunContext& context, int argc, char** argv, CliOptions& o,
        ModeArgs& mode_args);

void configure_command_logging(CliOptions& o);
void read_arrange_switches(CliRunContext& context, const CliOptions& o);

std::optional<int> check_calibration_options(CliRunContext& context, const CliOptions& o, const CalibOptions& calib_opts,
        Slic3r::Calib_Params& calib_params, bool& calib_self_geometry);

std::optional<int> run_information_commands(CliRunContext& context, const CliOptions& o, const ModeArgs& mode_args);

std::optional<int> check_assemble_input(CliRunContext& context, CliOptions& o);

std::optional<int> prepare_input_presets(CliRunContext& context, CliOptions& o);

std::optional<int> run_layout_commands(CliRunContext& context, char** argv, const CliOptions& o,
        const ModeArgs& mode_args);

std::optional<int> check_slice_input(CliRunContext& context, char** argv, const CliOptions& o,
        bool calib_self_geometry);

} // namespace slicer_cli
