#pragma once

#include <boost/log/trivial.hpp>
#include "libslic3r/PrintBase.hpp"
#include "cli_events.hpp"

namespace Slic3r { struct GCodeProcessorResult; }

namespace slicer_cli {

const char* slicing_notification_tag(int t);
const char* warning_level_tag(Slic3r::PrintStateBase::WarningLevel l);
void emit_config_substitutions(const Slic3r::ConfigSubstitutionContext& ctx,
                               const std::string& source);
void emit_gcode_result_diagnostics(const Slic3r::GCodeProcessorResult& r);
void emit_status_warning(const Slic3r::PrintBase::SlicingStatus& s);
void emit_validation_event(const Slic3r::StringObjectException& v);

} // namespace slicer_cli
