#pragma once

#include "libslic3r/PrintConfig.hpp"

namespace slicer_cli {

int normalize_legacy_gcode_tokens(Slic3r::DynamicPrintConfig& config, bool report_event = true);
#ifdef ENGINE_SNAPMAKER
std::string bambu_only_gcode_placeholder(const Slic3r::DynamicPrintConfig& config,
                                                std::string&                       key_out);
#endif

} // namespace slicer_cli
