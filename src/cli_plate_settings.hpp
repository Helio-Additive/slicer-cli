#pragma once

#include <optional>
namespace Slic3r { class DynamicPrintConfig; class Model; }
struct CliOptions;

namespace slicer_cli {

struct LoadedProject;
struct RunSettings;
struct PlatePreparation;
struct PlateOutcome;

int prepare_plate_configuration(const CliOptions& o, LoadedProject& loaded_project, RunSettings& run_settings,
                                       const PlatePreparation& preparation, PlateOutcome& outcome, bool trailing_port,
                                       int plate_id, Slic3r::DynamicPrintConfig& config,
                                       std::optional<Slic3r::DynamicPrintConfig>& project_settings);

#ifdef ENGINE_BAMBU
bool manual_map_extruders_fit(const Slic3r::DynamicPrintConfig& config, const Slic3r::Model& model, int plate_id,
                                     PlateOutcome& outcome);
#endif

} // namespace slicer_cli
