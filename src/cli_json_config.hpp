#pragma once

#include "libslic3r/PrintConfig.hpp"

namespace slicer_cli {

bool json_file_has_key(const std::string& filepath, const char* key);
bool load_json_config(const std::string& filepath, Slic3r::DynamicPrintConfig& config,
                      bool verbose = false, const std::string& diagnostic_source = {},
                      bool* supplied_nozzle_map = nullptr);
#ifdef ENGINE_BAMBU
bool bbs_3mf_config_contains_nozzle_map(const std::string& filepath,
                                      Slic3r::ConfigOptionInts& accepted_map);
#endif

} // namespace slicer_cli
