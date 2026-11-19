#include "cli_project_values.hpp"

#include "cli_run_context.hpp"
#include "libslic3r/Preset.hpp"
#include <boost/algorithm/string.hpp>
#include <iostream>
#include "cli_events.hpp"

namespace slicer_cli {

using json = nlohmann::json;

#ifdef ENGINE_ORCA
/// The filament settings PresetBundle::load_config_file_config cannot split:
/// with more than one filament (counted from filament_colour, as it counts
/// them) it copies entry i of every vector filament option into filament i's
/// preset with set_at, which throws on a vector with no entries
/// (PresetBundle.cpp 4456-4479, Config.hpp 439-440 at 31f6803). A BambuStudio
/// project reaches that with filament_notes, a single string there (coString)
/// and a per-filament list here (coStrings): the file's "" deserializes to no
/// entries. Each key is paired with whether the file states it as a single
/// value.
std::vector<std::pair<std::string, bool>> filament_keys_split_reads_empty(
    const Slic3r::DynamicPrintConfig& config, const json& file_settings) {
    std::vector<std::pair<std::string, bool>> keys;
    const auto* colour = config.option<Slic3r::ConfigOptionStrings>("filament_colour");
    if (colour == nullptr || colour->values.size() <= 1)
        return keys;
    for (const std::string& key : Slic3r::Preset::filament_options()) {
        if (key == "compatible_printers" || key == "compatible_prints")
            continue;
        const Slic3r::ConfigOption* opt = config.option(key);
        if (opt == nullptr || opt->is_scalar() || static_cast<const Slic3r::ConfigOptionVectorBase*>(opt)->size() != 0)
            continue;
        keys.emplace_back(key, file_settings.contains(key) && file_settings[key].is_string());
    }
    return keys;
}
#endif

#ifdef ENGINE_ORCA



bool bambu_value_actions(CliRunContext& context, const RawValues& raw, const std::string& support_type, bool maker_set_raft,
                                std::vector<BambuValueAction>& conversions,
                                std::vector<BambuValueAction>& warnings, std::string& refusal) {
    const auto raw_value = [&raw](const char* key) {
        const auto it = raw.find(key);
        return it == raw.end() ? std::string() : it->second;
    };
    const bool tree = boost::starts_with(support_type, "tree");
    // tree_support_wall_count: -1 is BambuStudio's automatic (BambuStudio
    // PrintConfig.cpp 5642-5650 at 5873b5f) and 0 is this engine's (OrcaSlicer
    // PrintConfig.cpp 6389-6396, TreeSupport.cpp 1629), which upstream converts
    // in 434ff3011f -- a commit the pin (31f6803 = upstream 42cce53) does not
    // have. BambuStudio's 0 means infill-only walls, which this engine cannot
    // express: the value is kept and warned about where tree support is in play.
    const std::string wall_count = raw_value("tree_support_wall_count");
    if (wall_count == "-1")
        conversions.push_back({"tree_support_wall_count", "0", "-1"});
#if defined(ENGINE_SNAPMAKER) || defined(ENGINE_ORCA)
    // BambuStudio's 0 means infill-only walls. Snapmaker's 0 is its own automatic
    // (PrintConfig tooltip, 0-2 at bac6fd5) and an OrcaSlicer project's 0 is its
    // own value too, so the warning is about a file made in BambuStudio only.
    else if (wall_count == "0" && tree && context.g_file_made_by_bambustudio)
#else
    else if (wall_count == "0" && tree)
#endif
        warnings.push_back({"tree_support_wall_count", "0", "0"});
    // raft_first_layer_expansion: BambuStudio's -1 is 2 mm of automatic
    // expansion under normal support (BambuStudio Support/SupportCommon.cpp
    // 392-394 and 431-437 at 5873b5f), which is this engine's own 2.0, and a
    // per-branch moment brim under tree support, which it has no value for.
    if (raw_value("raft_first_layer_expansion") == "-1") {
#ifdef ENGINE_SNAPMAKER
        // Snapmaker's engine has -1 = auto natively, and -1 is its own default
        // (references/SnapmakerOrca/src/libslic3r/PrintConfig.cpp:4589-4593 at
        // bac6fd5), passed through raw by TreeSupport.cpp:1617-1623 and
        // SupportCommon.cpp:403. So -1 is neither converted nor refused.
        (void) maker_set_raft;
        (void) tree;
    } else if (false) {
#endif
        if (tree) {
            if (maker_set_raft) {
                refusal = "The settings ask for raft_first_layer_expansion -1, BambuStudio's automatic "
                          "first-layer expansion, with tree support; this engine has no automatic value for tree "
                          "support. Set it to the expansion you want in mm (0 disables it), or use normal "
                          "support, whose automatic expansion is 2 mm.";
                return false;
            }
        } else {
            conversions.push_back({"raft_first_layer_expansion", "2.0", "-1"});
        }
    }
    // top_one_wall_type "not apply": off in BambuStudio (BambuStudio
    // PrintConfig.cpp 256-260 and 1375-1382 at 5873b5f). This engine's own
    // legacy rule turns only_one_wall_top on for every value but "none"
    // (OrcaSlicer PrintConfig.cpp 8011-8013; main 9315-9317 has the same rule).
    if (raw_value("top_one_wall_type") == "not apply")
        conversions.push_back({"only_one_wall_top", "0", "not apply"});
    // ensure_vertical_shell_thickness words (BambuStudio PrintConfig.cpp
    // 282-286): "enabled"/"disabled" are "ensure_all"/"none" here (OrcaSlicer
    // PrintConfig.cpp 369-372; its rule maps only the older "1"/"0",
    // 7978-7984). "partial" (BambuStudio PrintObject.cpp 1944, 1975) has no
    // value here at all.
    const std::string shell = raw_value("ensure_vertical_shell_thickness");
    if (shell == "enabled")
        conversions.push_back({"ensure_vertical_shell_thickness", "ensure_all", "enabled"});
    else if (shell == "disabled")
        conversions.push_back({"ensure_vertical_shell_thickness", "none", "disabled"});
    else if (shell == "partial") {
        refusal = "The settings ask for ensure_vertical_shell_thickness \"partial\", BambuStudio's "
                  "skip-one-shell-pass mode, which this engine has no value for; pick the shell coverage you "
                  "want, e.g. ensure_all for BambuStudio's \"enabled\".";
        return false;
    }
    return true;
}

/// The event tag and message for one action, so every scope reports the same
/// conversion the same way.
static std::string bambu_action_tag(const std::string& key) {
    if (key == "tree_support_wall_count") return "TreeSupportWallCountAutoConverted";
    if (key == "raft_first_layer_expansion") return "RaftAutoExpansionConverted";
    if (key == "only_one_wall_top") return "TopOneWallTypeNotApplyConverted";
    if (key == "ensure_vertical_shell_thickness") return "VerticalShellThicknessWordConverted";
    return "BambuValueConverted";
}

static std::string bambu_action_message(const BambuValueAction& action) {
    if (action.key == "tree_support_wall_count")
        return "tree_support_wall_count -1 is BambuStudio's automatic wall count and 0 this engine's, which "
               "upstream converts in 434ff3011f; converted here until this engine's pin has that commit";
    if (action.key == "raft_first_layer_expansion")
        return "raft_first_layer_expansion -1 is BambuStudio's automatic expansion, 2 mm under normal support, "
               "which is this engine's own 2.0";
    if (action.key == "only_one_wall_top")
        return "top_one_wall_type is BambuStudio's \"not apply\", which means the feature is off; the engine's "
               "own rule turns only_one_wall_top on for every value but \"none\"";
    if (action.key == "ensure_vertical_shell_thickness")
        return "ensure_vertical_shell_thickness \"" + action.from + "\" is BambuStudio's word for this engine's \"" +
               action.value + "\"";
    return "converted " + action.key + " from \"" + action.from + "\" to \"" + action.value + "\"";
}

#endif
/// A value's text as the file states it: a string as it is, a number without
/// json's own formatting, a boolean as "1"/"0". Empty when the key is not there.
std::string json_scalar_text(const json& settings, const char* key) {
    const auto it = settings.find(key);
    if (it == settings.end())
        return {};
    if (it->is_string())
        return it->get<std::string>();
    if (it->is_number_integer())
        return std::to_string(it->get<long long>());
    if (it->is_number_float()) {
        const double number = it->get<double>();
        if (number == double((long long)number))
            return std::to_string((long long)number);
        return it->dump();
    }
    if (it->is_boolean())
        return it->get<bool>() ? "1" : "0";
    return it->dump();
}
#ifdef ENGINE_ORCA

/// Writes one scope's conversions and reports them, then the values worth a
/// warning. `write` sets a key in that scope's own config; `drop_substitution`
/// removes the engine's record of not knowing a key this pass has just written
/// correctly; `scope`/`scope_name` name the object or part the settings belong
/// to, and are left empty for the project itself.
void apply_bambu_actions(const std::vector<BambuValueAction>& conversions,
                                const std::vector<BambuValueAction>& warnings,
                                const std::function<void(const char*, const std::string&)>& write,
                                const std::function<void(const char*)>&                      drop_substitution,
                                const std::string& scope, const std::string& scope_name) {
    for (const BambuValueAction& action : conversions) {
        write(action.key.c_str(), action.value);
        drop_substitution(action.key.c_str());
        json event = {{"event","config_normalized"},
                      {"tag", bambu_action_tag(action.key)},
                      {"opt_key", action.key},
                      {"from", action.from},
                      {"to", action.value},
                      {"message", bambu_action_message(action)}};
        if (!scope_name.empty())
            event[scope] = scope_name;
        emit_event(event);
    }
    for (const BambuValueAction& action : warnings) {
        json event = {{"event","warning"},
                      {"tag","TreeSupportWallCountZeroIsAuto"},
                      {"opt_key", action.key},
                      {"value", 0},
                      {"message","The settings set tree_support_wall_count 0, BambuStudio's infill-only support "
                                 "walls; this engine reads 0 as automatic, so the supports may come out sturdier. "
                                 "Set it to 1 or 2 to choose the wall count."}};
        if (!scope_name.empty())
            event[scope] = scope_name;
        emit_event(event);
    }
}

#endif

} // namespace slicer_cli
