#pragma once

namespace Slic3r { class DynamicPrintConfig; }
struct CliOptions;

namespace slicer_cli {

struct CliRunContext;
struct RunPlatePlan;
struct PlateOutcome;
struct LoadedProject;
struct RunSettings;

int check_downward_printers(CliRunContext& context, const CliOptions& o, RunPlatePlan& plate_plan,
            PlateOutcome& outcome, bool trailing_port, bool assemble_input, int plate_id,
            Slic3r::DynamicPrintConfig& config, LoadedProject& loaded_project, RunSettings& run_settings);

} // namespace slicer_cli
