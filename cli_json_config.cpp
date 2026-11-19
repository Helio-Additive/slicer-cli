#include "cli_json_config.hpp"

#include <boost/algorithm/string.hpp>
#include <boost/nowide/fstream.hpp>
#include "libslic3r/miniz_extension.hpp"
#include "cli_engine_events.hpp"
#include "cli_load_settings.hpp"
#include "cli_snapmaker.hpp"

namespace slicer_cli {

using json = nlohmann::json;

#ifdef ENGINE_BAMBU
// ConfigOptionInts does not check stream failures and can throw after appending
// part of a vector. Check extraction first, then deserialize into a scratch
// option so an invalid overlay cannot replace an earlier accepted map.
static bool deserialize_usable_nozzle_map(const std::string& text, Slic3r::ConfigOptionInts& out) {
    if (text.empty()) return false;
    std::istringstream stream(text);
    std::string token;
    while (std::getline(stream, token, ',')) {
        std::istringstream item(token);
        int nozzle;
        if (!(item >> nozzle) || nozzle < 0) return false;
        item >> std::ws;
        if (!item.eof()) return false;
    }
    Slic3r::ConfigOptionInts candidate;
    try {
        if (!candidate.deserialize(text) || candidate.values.empty()) return false;
    } catch (...) {
        return false;
    }
    out.values = std::move(candidate.values);
    return true;
}

// Match ConfigBase::load_from_json's string/array conversion for coInts:
// homogeneous arrays, string leaves, commas within arrays, '#' between groups.
static bool nozzle_map_json_text(const json& value, std::string& text) {
    if (value.is_string()) {
        text += value.get<std::string>();
        return true;
    }
    if (!value.is_array()) return false;
    std::string type;
    bool first = true;
    for (const auto& element : value) {
        if (first) type = element.type_name();
        else if (type != element.type_name()) return false;
        if (!first) text += element.is_array() ? '#' : ',';
        first = false;
        if (!nozzle_map_json_text(element, text)) return false;
    }
    return true;
}
#endif

// True when the JSON settings file at `filepath` (may be empty) names `key`.
bool json_file_has_key(const std::string& filepath, const char* key) {
    if (filepath.empty())
        return false;
    boost::nowide::ifstream f(filepath);
    if (!f.is_open())
        return false;
    try {
        const json j = json::parse(f);
        return j.is_object() && j.contains(key);
    } catch (...) {
        return false;
    }
}

// Load JSON config and record only accepted, usable nozzle-map overlays.
bool load_json_config(const std::string& filepath, Slic3r::DynamicPrintConfig& config,
                      bool verbose, const std::string& diagnostic_source,
                      bool* supplied_nozzle_map) {
    if (verbose) {
        std::cout << "Loading config: " << filepath << "\n";
    }

    // The path is UTF-8 (boost::nowide::args on Windows).
    boost::nowide::ifstream f(filepath);
    if (!f.is_open()) {
        std::cerr << "Error: Cannot open config file: " << filepath << "\n";
        return false;
    }

    try {
        json j = json::parse(f);

        // Create substitution context for config deserialization
        Slic3r::ConfigSubstitutionContext substitution_context(Slic3r::ForwardCompatibilitySubstitutionRule::Enable);

        // The file-level conversions the engines run for a settings file
        // (ConfigBase::load_from_json, Config.cpp 880-1096 at 31f6803): the
        // special cases at 931-948 (support_type hybrid(auto) -> support_style
        // tree_hybrid, wall_infill_order -> is_infill_first) and
        // handle_legacy_composite at 1094 (thumbnails,
        // wiping_volumes_use_custom_matrix). The per-key loop below runs
        // handle_legacy only (set_deserialize -> Config.cpp 579), so a
        // --machine/--process/--filament file skipped them. Loaded into a
        // scratch and applied, so the loop's own handling (the Bambu build's
        // nozzle map, the substitution reporting) still runs over it.
        {
            Slic3r::DynamicPrintConfig            loaded;
            std::map<std::string, std::string>    key_values;
            std::string                           reason;
            Slic3r::ConfigSubstitutionContext     file_substitutions(
                Slic3r::ForwardCompatibilitySubstitutionRule::EnableSilent);
            try {
                loaded.load_from_json(filepath, file_substitutions, /*load_inherits_in_config=*/false,
                                      key_values, reason);
                if (reason.empty() && !loaded.empty())
                    config.apply(loaded, /*ignore_nonexistent=*/true);
            } catch (const std::exception& e) {
                if (verbose)
                    std::cerr << "Warning: config file conversions failed: " << e.what() << "\n";
            }
        }

        // Iterate through all key-value pairs
        for (auto& [key, value] : j.items()) {
            // Skip profile metadata, including identity/version fields handled
            // separately by both engines' ConfigBase::load_from_json.
            // printer_model is a setting there (Config.cpp 918-960 at
            // 5873b5f: only the version, name, url, type, setting_id,
            // filament_id, from, description, instantiation, inherits and
            // includes are kept aside), and the Bambu printer features are
            // decided from it (BambuStudio.cpp 7055-7070), so a --machine
            // file's printer_model is loaded like any other setting.
            if (key == "type" || key == "name" || key == "inherits" ||
                key == "from" || key == "setting_id" || key == "instantiation" ||
                key == "description" || key == "compatible_printers" ||
                key == "compatible_prints" || key == "include" ||
                key == "upward_compatible_machine" ||
                key == "printer_variant" || key == "default_filament_profile" ||
                key == "default_print_profile" || key == "filament_id" ||
                key == "version" || key == "url" || key == "is_custom_defined") {
                continue;
            }

            try {
#ifdef ENGINE_BAMBU
                if (supplied_nozzle_map && key == "filament_nozzle_map" &&
                    !value.is_string() && !value.is_number() && !value.is_array())
                    continue;
                if (supplied_nozzle_map && key == "filament_nozzle_map" && value.is_array()) {
                    // The generic loader drops unsupported elements. Do not turn
                    // a malformed nozzle-map array into an apparently valid map.
                    bool valid_types = !value.empty();
                    for (const auto& element : value)
                        valid_types = valid_types && (element.is_string() || element.is_number());
                    if (!valid_types) continue;
                }
#endif
                // Use set_deserialize to respect existing config types
                // This handles type conversion properly
                std::string value_str;

                if (value.is_array()) {
                    // An array becomes the option's serialized form, as the
                    // engines' ConfigBase::load_from_json writes it
                    // (parse_str_arr, Config.cpp 863-900 and 1022-1040 at
                    // 5873b5f): a string list ';'-separated with each entry
                    // quoted and escaped (escape_strings_cstyle), a list of
                    // point groups '#'-separated, every other list
                    // ','-separated. Joined with ',' a string list became one
                    // string: a printer's seven extruder variants read as one,
                    // so the second extruder found no variant of its own
                    // (H2D nozzle_volume 130,130 for 130,145).
                    std::vector<std::string> parts;
                    for (auto& v : value) {
                        if (v.is_string()) {
                            parts.push_back(v.get<std::string>());
                        } else if (v.is_number()) {
                            parts.push_back(std::to_string(v.get<double>()));
                        }
                    }
                    const Slic3r::ConfigOptionDef* def = Slic3r::print_config_def.get(key);
                    if (def != nullptr && def->type == Slic3r::coStrings) {
                        value_str = Slic3r::escape_strings_cstyle(parts);
                    } else {
#ifdef ENGINE_SNAPMAKER
                        // Snapmaker Orca has no ConfigOptionPointsGroups, so
                        // no option takes the '#' separator (layout_plan.cpp).
                        const char* sep = ",";
#else
                        const char* sep = def != nullptr && def->type == Slic3r::coPointsGroups ? "#" : ",";
#endif
                        value_str = "";
                        for (size_t i = 0; i < parts.size(); i++) {
                            if (i > 0) value_str += sep;
                            value_str += parts[i];
                        }
                    }
                } else if (value.is_string()) {
                    value_str = value.get<std::string>();
                } else if (value.is_number_float()) {
                    value_str = std::to_string(value.get<double>());
                } else if (value.is_number_integer()) {
                    value_str = std::to_string(value.get<int>());
                } else if (value.is_boolean()) {
                    value_str = value.get<bool>() ? "1" : "0";
                }

                if (!value_str.empty() && value_str != "nil") {
#ifdef ENGINE_BAMBU
                    if (supplied_nozzle_map && key == "filament_nozzle_map") {
                        // The generic numeric conversion uses double formatting
                        // (1 becomes "1.000000"). Preserve integer map tokens
                        // without permitting fractional or malformed values.
                        auto map_token = [](const json& element) -> std::string {
                            if (element.is_string()) return element.get<std::string>();
                            if (element.is_number_float()) {
                                const double number = element.get<double>();
                                if (number >= 0 && number <= std::numeric_limits<int>::max() &&
                                    number == static_cast<int>(number))
                                    return std::to_string(static_cast<int>(number));
                            }
                            return element.dump();
                        };
                        if (value.is_array()) {
                            value_str.clear();
                            for (size_t i = 0; i < value.size(); ++i) {
                                if (i != 0) value_str += ',';
                                value_str += map_token(value[i]);
                            }
                        } else if (value.is_number()) {
                            value_str = map_token(value);
                        }
                        Slic3r::ConfigOptionInts accepted;
                        if (!deserialize_usable_nozzle_map(value_str, accepted)) continue;
                        Slic3r::DynamicPrintConfig scratch;
                        scratch.set_deserialize(key, value_str, substitution_context);
                        const auto* parsed = scratch.option<Slic3r::ConfigOptionInts>(key, false);
                        if (!parsed || parsed->values.empty()) continue;
                        config.set_key_value(key, parsed->clone());
                        *supplied_nozzle_map = true;
                        continue;
                    }
#endif
                    // set_deserialize respects the existing option type
                    config.set_deserialize(key, value_str, substitution_context);
                }
            } catch (const std::exception& e) {
                if (!diagnostic_source.empty()) {
                    emit_event({{"event", "config_value_rejected"},
                            {"tag", "ProfileConfigValueRejected"},
                            {"source", diagnostic_source},
                            {"opt_key", key},
                            {"message", "Failed to deserialize setting '" + key +
                                        "'; its value may have changed: " + e.what()}});
                }
                if (verbose) {
                    std::cerr << "Warning: Failed to set config key '" << key << "': " << e.what() << "\n";
                }
            }
        }

        // Keep non-fatal forward-compatibility substitutions and unknown keys
        // visible before success when the caller opts into slicing events.
        // Legacy --layout callers leave diagnostic_source empty.
#ifdef ENGINE_SNAPMAKER
        slicer_cli::convert_flow_type_spelling(config, substitution_context,
                                               diagnostic_source.empty() ? filepath : diagnostic_source);
#endif
        if (!diagnostic_source.empty()) {
            emit_config_substitutions(substitution_context, diagnostic_source);
        }

        if (verbose) {
            std::cout << "  Loaded successfully\n";
        }
        return true;

    } catch (const std::exception& e) {
        std::cerr << "Error parsing JSON config: " << e.what() << "\n";
        return false;
    }
}

#ifdef ENGINE_BAMBU
// load_bbs_3mf applies Metadata/project_settings.config directly to config.
// Inspect the same input layer before defaults or vector normalization obscure
// whether the file explicitly supplied a nozzle map.
bool bbs_3mf_config_contains_nozzle_map(const std::string& filepath,
                                      Slic3r::ConfigOptionInts& accepted_map) {
    mz_zip_archive zip;
    mz_zip_zero_struct(&zip);
    if (!Slic3r::open_zip_reader(&zip, filepath))
        return false;

    bool contains_nozzle_map = false;
    const mz_uint count = mz_zip_reader_get_num_files(&zip);
    for (mz_uint file_idx = 0; file_idx < count; ++file_idx) {
        mz_zip_archive_file_stat stat;
        if (mz_zip_reader_file_stat(&zip, file_idx, &stat)) {
            std::string name(stat.m_filename);
            std::replace(name.begin(), name.end(), '\\', '/');
            if (!boost::algorithm::iequals(name, "Metadata/project_settings.config")) continue;
            constexpr mz_uint64 max_project_settings_size = 16ULL * 1024 * 1024;
            if (stat.m_uncomp_size > max_project_settings_size) {
                mz_zip_reader_end(&zip);
                throw std::runtime_error("3MF project settings exceed the 16 MiB safety limit");
            }
            std::string content(stat.m_uncomp_size, '\0');
            if (mz_zip_reader_extract_to_mem(&zip, file_idx, content.data(), content.size(), 0)) {
                try {
                    const auto settings = json::parse(content);
                    if (settings.is_object() && settings.contains("filament_nozzle_map")) {
                        std::string text;
                        if (nozzle_map_json_text(settings["filament_nozzle_map"], text) &&
                            deserialize_usable_nozzle_map(text, accepted_map))
                            contains_nozzle_map = true;
                    }
                } catch (...) {
                    // load_bbs_3mf remains the authoritative parser and reports malformed input.
                }
            }
        }
    }
    mz_zip_reader_end(&zip);
    return contains_nozzle_map;
}
#endif

} // namespace slicer_cli
