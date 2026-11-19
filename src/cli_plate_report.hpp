#pragma once

#include <string>
#include "libslic3r/Point.hpp"
namespace Slic3r { class Print; class Model; class DynamicPrintConfig; struct GCodeProcessorResult; }

namespace slicer_cli {

struct CliRunContext;
struct PlateOutcome;
void record_plate_statistics(const Slic3r::Print& print, const Slic3r::Model& model,
                                    const Slic3r::DynamicPrintConfig& config, const Slic3r::DynamicPrintConfig& project,
                                    const Slic3r::GCodeProcessorResult& result, PlateOutcome& outcome);
void record_plate_for_export(CliRunContext& context, const Slic3r::Print& print, const Slic3r::Model& model,
                                    const Slic3r::DynamicPrintConfig& config,
                                    Slic3r::GCodeProcessorResult& result, const std::string& gcode_file,
                                    PlateOutcome& outcome, const Slic3r::Vec2d& plate_origin);
bool post_slice_checks(const Slic3r::Print& print, const Slic3r::Model& model,
                              const Slic3r::GCodeProcessorResult& result, PlateOutcome& outcome);

} // namespace slicer_cli
