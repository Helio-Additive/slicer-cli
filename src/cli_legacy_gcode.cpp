#include "cli_legacy_gcode.hpp"

#include "cli_events.hpp"

namespace slicer_cli {

using json = nlohmann::json;

// ─────────────────────────────────────────────────────────────────────────────
// Driver-side normalization for the unbound placeholder `initial_no_support_filament_id`.
//
// OrcaSlicer does not bind `initial_no_support_filament_id` in the
// PlaceholderParser: it binds `initial_no_support_extruder` /
// `initial_no_support_hotend`, and the first is the SAME int as BambuStudio's
// `initial_no_support_filament_id` (BambuStudio binds it in GCode.cpp 2657 at
// 5873b5f, from the same `initial_non_support_extruder_id`, GCode.cpp
// 2458-2460). Because the `_extruder` and `_filament` names are aliases of the
// same filament index, the BambuStudio token is semantically identical to
// OrcaSlicer's `initial_no_support_extruder`.
//
// The token is not produced by any stock profile in this repo or upstream master
// start-gcode; it only arrives via a hand-edited / third-party custom gcode embedded
// in a 3MF. When present it makes the PlaceholderParser throw at export and aborts the
// slice. The correct fix is therefore a driver-side alias at config load — NOT an
// engine edit (engine submodules are off-limits).
//
// Whole-token rewrite across every coString custom-gcode key. The character on either
// side of a match must be a non-identifier char, so a hypothetical
// `initial_no_support_filament_idx` and any identifier that merely embeds the token are
// never touched. (The separately-bound `initial_filament_id` is a different, shorter
// string and is never searched for, so it is inherently safe.)
// Single-string (coString) custom-gcode keys.
// Every coString custom-gcode key either engine runs through
// placeholder_parser_process (PrintConfig.cpp `add("*_gcode", coString)`,
// minus `export_gcode` which is the output-path flag, not a template). This
// binary builds BOTH engines (ENGINE_BAMBU / ENGINE_ORCA), so the list is the
// union; keys absent from the active engine's schema are null-guarded no-ops
// (the `file_*` / `*_extrusion_role_*` keys are Orca-only).
static const std::vector<std::string> kGcodeStringKeys = {
    "machine_start_gcode", "machine_end_gcode",
    "before_layer_change_gcode", "layer_change_gcode",
    "change_filament_gcode", "time_lapse_gcode",
    "machine_pause_gcode", "printing_by_object_gcode",
    "template_custom_gcode", "wrapping_detection_gcode",
    // Orca-only:
    "file_start_gcode", "change_extrusion_role_gcode",
    "process_change_extrusion_role_gcode",
};
// Per-filament (coStrings) custom-gcode keys — these ALSO run through the
// PlaceholderParser (filament_start/end emission), so the unbound token can
// abort from them too. (`filament_change_extrusion_role_gcode` is Orca-only.)
static const std::vector<std::string> kGcodeStringsKeys = {
    "filament_start_gcode", "filament_end_gcode",
    "filament_change_extrusion_role_gcode",
};

int normalize_legacy_gcode_tokens(Slic3r::DynamicPrintConfig& config, bool report_event) {
    static const std::string kLegacyToken = "initial_no_support_filament_id";
    static const std::string kBoundToken  = "initial_no_support_extruder";
    const size_t tlen = kLegacyToken.size();
    auto is_ident = [](char c) {
        return (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
               (c >= '0' && c <= '9') || c == '_';
    };
    // Whole-token alias within one string; bumps `rewrites` per replacement.
    auto rewrite_one = [&](const std::string& v, int& rewrites) -> std::string {
        if (v.find(kLegacyToken) == std::string::npos) return v;
        std::string out;
        out.reserve(v.size());
        size_t pos = 0;
        while (pos < v.size()) {
            const size_t found = v.find(kLegacyToken, pos);
            if (found == std::string::npos) { out.append(v, pos, std::string::npos); break; }
            const bool left_ok  = (found == 0)        || !is_ident(v[found - 1]);
            const size_t after  = found + tlen;
            const bool right_ok = (after >= v.size()) || !is_ident(v[after]);
            if (left_ok && right_ok) {
                out.append(v, pos, found - pos);
                out.append(kBoundToken);
                pos = after;
                ++rewrites;
            } else {
                // Substring match inside a longer identifier — copy one char past the
                // match start and keep scanning so overlapping matches still resolve.
                out.append(v, pos, found - pos + 1);
                pos = found + 1;
            }
        }
        return out;
    };
    int total_rewrites = 0;
    std::vector<std::string> rewritten_keys;
    for (const auto& key : kGcodeStringKeys) {
        if (!config.has(key)) continue;
        int key_rewrites = 0;
        const std::string out = rewrite_one(config.opt_string(key), key_rewrites);
        if (key_rewrites > 0) {
            config.set_key_value(key, new Slic3r::ConfigOptionString(out));
            total_rewrites += key_rewrites;
            rewritten_keys.push_back(key);
        }
    }
    for (const auto& key : kGcodeStringsKeys) {
        auto* opt = config.option<Slic3r::ConfigOptionStrings>(key);
        if (!opt) continue;
        int key_rewrites = 0;
        std::vector<std::string> values = opt->values;
        for (auto& s : values) s = rewrite_one(s, key_rewrites);
        if (key_rewrites > 0) {
            config.set_key_value(key, new Slic3r::ConfigOptionStrings(values));
            total_rewrites += key_rewrites;
            rewritten_keys.push_back(key);
        }
    }
    if (total_rewrites > 0 && report_event) {
        json e;
        e["event"]   = "config_normalized";
        e["tag"]     = "LegacyGcodeTokenAliased";
        e["message"] = "Aliased unbound placeholder '" + kLegacyToken + "' -> '" +
                       kBoundToken + "' in " + std::to_string(rewritten_keys.size()) +
                       " custom-gcode key(s); " + std::to_string(total_rewrites) +
                       " occurrence(s) rewritten";
        e["from"]    = kLegacyToken;
        e["to"]      = kBoundToken;
        e["keys"]    = rewritten_keys;
        e["count"]   = total_rewrites;
        emit_event(e);
    }
    return total_rewrites;
}

#ifdef ENGINE_ORCA
/// The Bambu Studio placeholder this engine has no value for, empty when there
/// is none: either of the two names goes unbound in OrcaSlicer -- its
/// PlaceholderParser sets initial_extruder and reads nozzle_diameter from the
/// config, and neither binds BambuStudio's nozzle_diameter_at_nozzle_id or
/// initial_nozzle_id (BambuStudio binds both in GCode.cpp 2656-2660 at
/// 5873b5f). Upstream main binds them with its multi-nozzle engine (GCode.cpp
/// 3334-3335 on main), too large to backport. `key_out` names the custom-gcode
/// key the placeholder was found in.
std::string bambu_only_gcode_placeholder(const Slic3r::DynamicPrintConfig& config,
                                                std::string&                       key_out) {
    static const std::vector<std::string> kPlaceholders = {"nozzle_diameter_at_nozzle_id",
                                                           "initial_nozzle_id"};
    auto scan = [&](const std::string& key, const std::string& text) {
        for (const std::string& token : kPlaceholders) {
            if (text.find(token) == std::string::npos)
                continue;
            key_out = key;
            return token;
        }
        return std::string();
    };
    for (const std::string& key : kGcodeStringKeys) {
        const Slic3r::ConfigOption* opt = config.option(key);
        if (opt == nullptr)
            continue;
        // The serialized form carries every value of a per-filament key too.
        if (const std::string found = scan(key, opt->serialize()); !found.empty())
            return found;
    }
    for (const std::string& key : kGcodeStringsKeys) {
        const Slic3r::ConfigOption* opt = config.option(key);
        if (opt == nullptr)
            continue;
        if (const std::string found = scan(key, opt->serialize()); !found.empty())
            return found;
    }
    return {};
}
#endif

} // namespace slicer_cli
