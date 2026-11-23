#pragma once

#include <boost/log/trivial.hpp>
#include "libslic3r/PrintBase.hpp"
#include "cli_events.hpp"

namespace Slic3r { struct GCodeProcessorResult; }

namespace slicer_cli {

const char* slicing_notification_tag(int t);
const char* warning_level_tag(Slic3r::PrintStateBase::WarningLevel l);
/// The replaced values of this run in the shape OrcaSlicer from 2.5 lists them
/// in result.json (class config_value_substituted); filled on that build only.
std::vector<nlohmann::json>& recorded_config_substitutions();
void emit_config_substitutions(const Slic3r::ConfigSubstitutionContext& ctx,
                               const std::string& source);
void emit_gcode_result_diagnostics(const Slic3r::GCodeProcessorResult& r);
void emit_status_warning(const Slic3r::PrintBase::SlicingStatus& s);
void emit_validation_event(const Slic3r::StringObjectException& v);

} // namespace slicer_cli
