#pragma once

#include "cli_preset_staging.hpp"
#include <nlohmann/json.hpp>

namespace slicer_cli {

#ifdef ENGINE_BAMBU
struct PercentRewrite {
    bool        changed = false;
    std::string refusal;            // non-empty: the sentence the slice stops with
    boost::filesystem::path temp_dir;
    std::string temp_path;          // the converted copy, same file name
    nlohmann::json converted = nlohmann::json::array();
    double      nozzle_mm = 0.;
    ~PercentRewrite() { remove_owned_temp_dir(temp_dir); }
};

void rewrite_percent_line_widths(const std::string& input, PercentRewrite& out);
#endif

} // namespace slicer_cli
