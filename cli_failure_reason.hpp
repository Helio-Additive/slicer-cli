// cli_failure_reason.hpp — result.json "error_reason": why a plate failed.
//
// error_string stays the official sentence for the code (record_exit_reson,
// BambuStudio.cpp 483-513 at 02.08.02.61). The official command lines print
// an engine exception's text to stderr only (BambuStudio.cpp 7412-7417,
// OrcaSlicer.cpp 6286-6290 at 2.4.2), and what the load noted (a model that
// looks saved in meters or inches, a settings file that could not be read)
// goes out only as an event. A caller that reads
// result.json gets these sentences in error_reason.
#pragma once

#include <string>

namespace slicer_cli {

/// Adds `sentence` after the reasons already in `reasons`.
void add_failure_reason(std::string& reasons, const std::string& sentence);

/// Puts an engine exception's text first: it is what failed, and the load's
/// notes after it say why.
void add_engine_reason(std::string& reasons, const char* what);

} // namespace slicer_cli
