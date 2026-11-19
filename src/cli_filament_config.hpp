#pragma once

#include "libslic3r/PrintConfig.hpp"

namespace slicer_cli {

size_t project_filament_count(Slic3r::DynamicPrintConfig& config);
#ifdef ENGINE_BAMBU
bool apply_explicit_nozzle_mapping(
    Slic3r::DynamicPrintConfig& config,
    bool explicit_config_supplied_nozzle_map,
    bool verbose);
void set_default_config(Slic3r::DynamicPrintConfig& config);
void ensure_vector_config_sizes(Slic3r::DynamicPrintConfig& config);
bool is_filament_setting_key(const std::string& key);
size_t align_per_filament_maps(Slic3r::DynamicPrintConfig& config, size_t filament_count);
bool align_per_filament_config_vectors(Slic3r::DynamicPrintConfig& config,
                                       size_t filament_count, bool verbose);
#endif
size_t normalize_filament_config_vectors(Slic3r::DynamicPrintConfig& config);

} // namespace slicer_cli
