#pragma once

#include <map>
#include <string>
#include "libslic3r/PrintConfig.hpp"

namespace slicer_cli {

void apply_command_line_overrides(Slic3r::DynamicPrintConfig& config,
                                         const std::map<std::string, std::string>& overrides,
                                         bool report_rejections);

} // namespace slicer_cli
