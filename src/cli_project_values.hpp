#pragma once

#include "cli_project_settings.hpp"
#include <functional>
#include <nlohmann/json.hpp>
namespace Slic3r { class DynamicPrintConfig; class Model; }

namespace slicer_cli {

struct CliRunContext;
#ifdef ENGINE_ORCA
/// What the BambuStudio values one scope states ask this engine to do: the keys
/// to write (this engine's key and value), the values worth a warning, and the
/// refusal when a value cannot be carried at all. Decides only; the caller
/// writes, names the scope in its events, and refuses.
struct BambuValueAction {
    std::string key;
    std::string value;
    std::string from;
};

std::vector<std::pair<std::string, bool>> filament_keys_split_reads_empty(
    const Slic3r::DynamicPrintConfig& config, const nlohmann::json& file_settings);
bool bambu_value_actions(CliRunContext& context, const RawValues& raw, const std::string& support_type, bool maker_set_raft,
                                std::vector<BambuValueAction>& conversions,
                                std::vector<BambuValueAction>& warnings, std::string& refusal);
void apply_bambu_actions(const std::vector<BambuValueAction>& conversions,
                                const std::vector<BambuValueAction>& warnings,
                                const std::function<void(const char*, const std::string&)>& write,
                                const std::function<void(const char*)>&                      drop_substitution,
                                const std::string& scope, const std::string& scope_name);

#endif

std::string json_scalar_text(const nlohmann::json& settings, const char* key);

} // namespace slicer_cli
