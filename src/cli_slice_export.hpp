#pragma once

#include <boost/filesystem/path.hpp>
#include <set>
#include <string>
#include <vector>

struct CliOptions;
namespace Slic3r { class DynamicPrintConfig; class Model; }

namespace slicer_cli {

struct CliRunContext;
struct RunState;
struct PlateOutcome;
/// One file a run writes, as check_run_outputs compares it (see there).
struct RunOutput {
    bool        folder = false;   // a folder the run writes into (--export-slicedata)
    bool        stl    = false;   // an object STL file of --export-stl / --export-stls
    std::string flag;             // "--export-3mf NAME"; empty for the run's own files
    boost::filesystem::path path;
    std::string what;             // "the run's own result.json in --outputdir"
    std::string is_what;          // what follows "is" in the same-name sentence
};

int export_sliced_3mf(CliRunContext& context, const CliOptions& o, const boost::filesystem::path& outdir, RunState& rs,
                             std::vector<PlateOutcome>& outcomes, const Slic3r::DynamicPrintConfig& project,
                             const std::set<std::string>* given_settings, std::string& error);
int check_run_outputs(CliRunContext& context, const CliOptions& o, const std::vector<int>& plates, const Slic3r::Model* model,
                             std::string* detail, bool after_slice_actions_only = false,
                             const std::vector<RunOutput>* reserved = nullptr,
                             std::vector<RunOutput>* reserve = nullptr);

} // namespace slicer_cli
