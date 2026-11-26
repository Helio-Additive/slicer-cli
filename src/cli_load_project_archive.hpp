#pragma once

struct CliOptions;

namespace slicer_cli {

struct CliRunContext;
struct LoadedProject;
struct RunPlatePlan;
struct PlateOutcome;
struct PlateLoadSettings;
struct PlateLoadInputs;

int load_project_archive(CliRunContext& context, const CliOptions& o, LoadedProject& loaded_project,
            RunPlatePlan& plate_plan, PlateOutcome& outcome, int plate_id, PlateLoadSettings& settings,
            const PlateLoadInputs& inputs);

} // namespace slicer_cli
