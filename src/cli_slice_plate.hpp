#pragma once

#include <string>

struct CliOptions;
namespace Slic3r { struct Calib_Params; }

namespace slicer_cli {

struct CliRunContext;
struct RunState;
struct PlateOutcome;
int slice_one_plate(CliRunContext& context, const CliOptions& o, Slic3r::Calib_Params& calib_params, RunState& rs,
                           int plate_id, const std::string& output_file, PlateOutcome& outcome);

} // namespace slicer_cli
