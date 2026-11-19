#pragma once

#include <map>
#include <string>

namespace Slic3r { class DynamicPrintConfig; }

namespace slicer_cli {

void apply_command_line_overrides(Slic3r::DynamicPrintConfig& config,
                                         const std::map<std::string, std::string>& overrides,
                                         bool report_rejections);

} // namespace slicer_cli
