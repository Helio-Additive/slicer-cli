// cli_flag_support.hpp — which official flags this binary has, and why not
// when it does not. One table, read by the parser (refusals) and --help.
#pragma once

#include <string>
#include <vector>

#include "libslic3r/PrintConfig.hpp"

namespace slicer_cli {

/// Empty when the official flag `key` works in this binary; otherwise the
/// refusal sentence, which names the flag and says why.
std::string official_flag_refusal(const std::string& key);

/// Which slicer-cli binaries take the official flag `key`: "both binaries"
/// or "slicer_cli only (Bambu Studio engine)".
std::string works_in(const std::string& key);

/// For a name that is not a flag of this binary: a sentence when it is an
/// official flag of the other engine's CLI, or a flag the official CLIs have
/// commented out. Empty when slicer-cli knows nothing about it.
std::string foreign_flag_refusal(const std::string& cli_name);

/// The official flags only the other engine's CLI has (CLI names, dashed).
std::vector<std::string> other_engine_only_flags();

/// Empty when every print setting given on the command line can be applied
/// like the official m_extra_config; else the refusal sentence.
std::string extra_config_refusal(const Slic3r::DynamicPrintConfig& extra);

} // namespace slicer_cli
