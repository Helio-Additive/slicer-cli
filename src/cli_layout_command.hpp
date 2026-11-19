#pragma once

#include <string>

namespace slicer_cli {

int run_layout_plan_command(const std::string& input_file);
int run_layout_command(const std::string& layout_json_file, const char* argv0);

} // namespace slicer_cli
