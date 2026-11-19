#pragma once

#include "libslic3r/PrintConfig.hpp"
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

struct CliOptions;

namespace slicer_cli {

struct CliRunContext;
struct LoadedProject;
struct RunSettings;
struct RunPlatePlan;
struct PlateWarnings;
struct PlateOutcome;

struct PlateLoadInputs {
    bool geometry_input;
    bool geometry_only_3mf;
    bool trailing_port;
    bool assemble_input;
    bool calib_self_geometry;
};

struct PlateLoadSettings {
    Slic3r::DynamicPrintConfig config;
    // Values this engine has no word for (unknown enum values), reported
    // together with the out-of-range values at the value check below.
    std::vector<std::string> unknown_values;
    nlohmann::json unknown_items = nlohmann::json::array();
};

int load_slice_inputs(CliRunContext& context, const CliOptions& o, LoadedProject& loaded_project, RunSettings& run_settings,
                      RunPlatePlan& plate_plan, PlateWarnings& plate_warnings, int plate_id,
                      const PlateLoadInputs& inputs, PlateLoadSettings& settings, PlateOutcome& outcome);

} // namespace slicer_cli
