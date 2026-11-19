#pragma once

#include <string>
#include <nlohmann/json.hpp>

struct CliOptions;
namespace Slic3r { struct Calib_Params; }

namespace slicer_cli {

struct CliRunContext;
struct RunState;
struct PlateOutcome;
void announce_noted_warning(const nlohmann::json& announcement);
int slice_one_plate(CliRunContext& context, const CliOptions& o, Slic3r::Calib_Params& calib_params, RunState& rs,
                           int plate_id, const std::string& output_file, PlateOutcome& outcome);

} // namespace slicer_cli
