#pragma once


struct CliOptions;
namespace Slic3r { struct Calib_Params; }

namespace slicer_cli {

struct CliRunContext;
int run_cli_slice(CliRunContext& context, const CliOptions& o, Slic3r::Calib_Params& calib_params);

} // namespace slicer_cli
