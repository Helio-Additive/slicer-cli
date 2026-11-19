#pragma once

#include <boost/filesystem/path.hpp>
#include <set>
#include <string>
#include <vector>

struct CliOptions;
namespace Slic3r { class DynamicPrintConfig; class Model; }

namespace slicer_cli {

struct CliRunContext;
struct LoadedProject;
struct RunPlatePlan;
struct SettingsMerge;
struct PlateOutcome;
struct RunOutput;

int export_sliced_3mf(CliRunContext& context, const CliOptions& o, const boost::filesystem::path& outdir,
                             LoadedProject& loaded_project, RunPlatePlan& plate_plan, const SettingsMerge& settings_merge,
                             std::vector<PlateOutcome>& outcomes, const Slic3r::DynamicPrintConfig& project,
                             const std::set<std::string>* given_settings, std::string& error);
int check_run_outputs(CliRunContext& context, const CliOptions& o, const std::vector<int>& plates, const Slic3r::Model* model,
                             std::string* detail, bool after_slice_actions_only = false,
                             const std::vector<RunOutput>* reserved = nullptr,
                             std::vector<RunOutput>* reserve = nullptr);

} // namespace slicer_cli
