// cli_snapmaker.cpp — the Snapmaker Orca build's desktop rules (cli_snapmaker.hpp).
// Every rule cites the Snapmaker Orca source it ports, at bac6fd5.
#include "cli_snapmaker.hpp"

#ifdef ENGINE_SNAPMAKER

#include <algorithm>
#include <map>
#include <set>
#include <sstream>

#include <boost/nowide/fstream.hpp>
#include <nlohmann/json.hpp>

#include "libslic3r/Utils.hpp"

#include "cli_events.hpp"

namespace slicer_cli {


/// The filament slots this plate uses, 0-based, as the desktop's
/// collect_used_filament_slots_on_plate lists them
/// (references/SnapmakerOrca/src/slic3r/GUI/Plater.cpp:505-570 at bac6fd5):
/// the plate's own config, each on-plate object's and volume's configs and the
/// volumes' filament ids, the plate's wipe-tower/support filaments, and — when
/// an object uses the default extruder — the process's wall and infill
/// filaments and that default extruder.
std::set<int> desktop_used_filament_slots(const Slic3r::Model& model, const Slic3r::DynamicPrintConfig& config,
                                          int num_filaments) {
    using namespace Slic3r;
    std::set<int> slots;
    if (num_filaments <= 0)
        return slots;
    // Plater.cpp:540-557 at bac6fd5: wipe-tower and support filaments always
    // apply; wall, sparse-infill and solid-infill only when an object on the
    // plate uses the default extruder (and `extruder` itself only then, 559-563).
    auto from_config = [&](const DynamicPrintConfig& c, bool default_extruder) {
        for (const char* key : {"wipe_tower_filament", "support_filament", "support_interface_filament"}) {
            if (const auto* opt = c.option<ConfigOptionInt>(key); opt && opt->value >= 1 && opt->value <= num_filaments)
                slots.insert(opt->value - 1);
        }
        if (!default_extruder)
            return;
        for (const char* key : {"wall_filament", "sparse_infill_filament", "solid_infill_filament"}) {
            if (const auto* opt = c.option<ConfigOptionInt>(key); opt && opt->value >= 1 && opt->value <= num_filaments)
                slots.insert(opt->value - 1);
        }
        if (const auto* opt = c.option<ConfigOptionInt>("extruder"); opt && opt->value >= 1 && opt->value <= num_filaments)
            slots.insert(opt->value - 1);
    };
    auto from_model_config = [&](const ModelConfigObject& c) {
        if (c.has("extruder")) {
            const int id = c.extruder();
            if (id >= 1 && id <= num_filaments) slots.insert(id - 1);
        }
        for (const char* key : {"support_filament", "support_interface_filament", "wall_filament",
                                "sparse_infill_filament", "solid_infill_filament", "wipe_tower_filament"}) {
            if (c.has(key)) {
                const int value = c.opt_int(key);
                if (value >= 1 && value <= num_filaments) slots.insert(value - 1);
            }
        }
    };
    // The driver has no Plater working config separate from the run's own:
    // `config` is the merged settings this plate slices with, so the plate's
    // and the process's keys are read from it.
    bool uses_default_extruder = false;
    for (const ModelObject* object : model.objects)
        if (!object->config.has("extruder") || object->config.extruder() == 0)
            uses_default_extruder = true;
    from_config(config, uses_default_extruder);
    for (const ModelObject* object : model.objects) {
        from_model_config(object->config);
        for (const ModelVolume* volume : object->volumes) {
            from_model_config(volume->config);
            for (int extruder_id : volume->get_extruders())
                if (extruder_id >= 1 && extruder_id <= num_filaments)
                    slots.insert(extruder_id - 1);
        }
    }
    if (uses_default_extruder)
        if (const auto* opt = config.option<ConfigOptionInt>("extruder"); opt && opt->value >= 1 && opt->value <= num_filaments)
            slots.insert(opt->value - 1);
    return slots;
}

/// The engine's own mixed-filament construction, exactly as the slice does it
/// (references/SnapmakerOrca/src/libslic3r/PrintApply.cpp:1439-1449 at bac6fd5):
/// the colours come from filament_colour, padded to the nozzle count with
/// "#26A69A"; the manager is cleared, auto-generated from those colours, given
/// the project's mixed_filament_definitions, and finally given the gradient
/// settings. Skipping the auto_generate leaves it with no rows, which is why
/// load_custom_entries alone found nothing.
Slic3r::MixedFilamentManager mixed_manager_from_config(const Slic3r::DynamicPrintConfig& config, size_t& physical,
                                                       const Slic3r::MixedFilamentManager* before) {
    using namespace Slic3r;
    std::vector<std::string> colours;
    if (const auto* colour = config.option<ConfigOptionStrings>("filament_colour"))
        colours = colour->values;
    // Print::apply counts the physical filaments as filament_diameter.size()
    // (references/SnapmakerOrca/src/libslic3r/PrintApply.cpp:1357) and pads the
    // colours to that count (1442), so the mixed rows are numbered against it.
    size_t extruders = 0;
    if (const auto* diameters = dynamic_cast<const ConfigOptionVectorBase*>(config.option("filament_diameter")))
        extruders = diameters->size();
    physical = std::max(extruders, size_t(1));
    colours.resize(physical, "#26A69A");
    std::string definitions;
    if (const auto* defs = config.option<ConfigOptionString>("mixed_filament_definitions"))
        definitions = defs->value;
    // Each of these is read by its own option type: the fork declares some as
    // Int and some as Bool, and an opt_* cast on the wrong one is null.
    auto read_int = [&](const char* key, int fallback) {
        if (const auto* opt = config.option<ConfigOptionInt>(key)) return opt->value;
        if (const auto* opt = config.option<ConfigOptionBools>(key); opt != nullptr && !opt->values.empty())
            return int(opt->values.front());
        return fallback;
    };
    auto read_float = [&](const char* key, float fallback) {
        if (const auto* opt = config.option<ConfigOptionFloat>(key)) return float(opt->value);
        return fallback;
    };
    auto read_bool = [&](const char* key, bool fallback) {
        if (const auto* opt = config.option<ConfigOptionBool>(key)) return bool(opt->value);
        if (const auto* opt = config.option<ConfigOptionInt>(key)) return opt->value != 0;
        return fallback;
    };
    int gradient_mode = read_int("mixed_filament_gradient_mode", 0);
    float lower = read_float("mixed_filament_height_lower_bound", 0.04f);
    float upper = read_float("mixed_filament_height_upper_bound", 0.16f);
    bool advanced = read_bool("mixed_filament_advanced_dithering", false);
    gradient_mode = std::clamp(gradient_mode, 0, 1);
    lower = std::max(0.01f, lower);
    upper = std::max(lower, upper);
    if (before != nullptr) {
        // A filament count change rebuilds the bundle's own manager, which
        // keeps each surviving auto pair's stable_id, so the remap can follow
        // it (PresetBundle.cpp:3896-3911 at bac6fd5; auto_generate keeps them,
        // MixedFilament.cpp:1714-1778).
        MixedFilamentManager mgr = *before;
        mgr.auto_generate(colours);
        mgr.clear_custom_entries();
        mgr.load_custom_entries(definitions, colours);
        mgr.apply_gradient_settings(gradient_mode, lower, upper, advanced);
        return mgr;
    }
    MixedFilamentManager mgr;
    mgr.clear_custom_entries();
    mgr.auto_generate(colours);
    mgr.load_custom_entries(definitions, colours);
    mgr.apply_gradient_settings(gradient_mode, lower, upper, advanced);
    return mgr;
}

/// The desktop's paint state map for a filament-id remap (Plater.cpp:22233-22250
/// at bac6fd5): every id maps to itself, then each remapped id to its target, or
/// to NONE when it was removed or lies past the physical + mixed total.
Slic3r::EnforcerBlockerStateMap paint_state_map(const std::vector<unsigned int>& id_remap, size_t total) {
    Slic3r::EnforcerBlockerStateMap state_map;
    for (size_t i = 0; i < state_map.size(); ++i)
        state_map[i] = Slic3r::EnforcerBlockerType(i);
    for (size_t i = 1; i < state_map.size(); ++i) {
        const unsigned int mapped = i < id_remap.size() ? id_remap[i] : 0;
        state_map[i] = (mapped == 0 || mapped >= state_map.size() || mapped > total)
                           ? Slic3r::EnforcerBlockerType::NONE
                           : Slic3r::EnforcerBlockerType(mapped);
    }
    return state_map;
}

/// The filament categories the desktop's mixed-filament rule knows, in its enum
/// order (references/SnapmakerOrca/src/slic3r/GUI/MixedColorMatchHelpers.cpp:1031-1040
/// at bac6fd5).
enum class MixCategory : size_t { PLA, PETG, TPU, PET, ABS, ASA, PC, PA, SUPPORT, UNKNOWN };
const char* const k_mix_category_names[] = {"PLA", "PETG", "TPU", "PET", "ABS", "ASA", "PC", "PA", "SUPPORT"};

MixCategory mix_category_from_name(const std::string& name) {
    for (size_t i = 0; i < 9; ++i)
        if (name == k_mix_category_names[i])
            return static_cast<MixCategory>(i);
    return MixCategory::UNKNOWN;
}

/// The compatibility table the desktop loads from the bundled
/// filament_compatibility.json (MixedColorMatchHelpers.cpp:1056-1124): each
/// category is compatible with itself and with the partners the file lists.
const std::vector<std::vector<bool>>& mix_compatibility_table() {
    static const std::vector<std::vector<bool>> table = [] {
        std::vector<std::vector<bool>> t(10, std::vector<bool>(10, false));
        for (size_t i = 0; i < 9; ++i) t[i][i] = true;
        const std::string path = Slic3r::resources_dir() + "/profiles/Snapmaker/filament/filament_compatibility.json";
        try {
            boost::nowide::ifstream ifs(path);
            if (ifs.is_open()) {
                nlohmann::json j;
                ifs >> j;
                for (auto& [a_name, partners] : j["compatibility"].items()) {
                    const MixCategory a = mix_category_from_name(a_name);
                    if (a == MixCategory::UNKNOWN || !partners.is_array()) continue;
                    for (auto& b_val : partners) {
                        const MixCategory b = mix_category_from_name(b_val.get<std::string>());
                        if (b == MixCategory::UNKNOWN) continue;
                        t[(size_t) a][(size_t) b] = true;
                        t[(size_t) b][(size_t) a] = true;
                    }
                }
            }
        } catch (...) {
            // An unreadable table leaves self-compatibility only, as the
            // desktop's own default does.
        }
        return t;
    }();
    return table;
}

/// is_filament_compatible(ids) (MixedColorMatchHelpers.cpp:1214-1260 at
/// bac6fd5): an unknown category is incompatible, one fibre or one shared
/// category is fine, and otherwise the table decides every pair.
bool mixed_filament_ids_compatible(const std::vector<unsigned int>& ids_0based,
                                   const std::vector<std::string>& filament_types) {
    if (ids_0based.size() <= 1) return true;
    std::vector<MixCategory> resolved;
    for (unsigned int id : ids_0based) {
        if (id >= filament_types.size()) return false;
        const MixCategory category = mix_category_from_name(filament_types[id]);
        if (category == MixCategory::UNKNOWN) return false;
        resolved.push_back(category);
    }
    if (resolved.size() <= 1) return true;
    bool all_same = true;
    for (size_t i = 1; i < resolved.size(); ++i)
        if (resolved[i] != resolved[0]) { all_same = false; break; }
    if (all_same) return true;
    const auto& table = mix_compatibility_table();
    for (size_t i = 0; i < resolved.size(); ++i)
        for (size_t j = i + 1; j < resolved.size(); ++j)
            if (!table[(size_t) resolved[i]][(size_t) resolved[j]]) return false;
    return true;
}

/// is_filament_compatible(MixedFilament) (MixedColorMatchHelpers.cpp:1258-1318):
/// the physical components a mixed filament blends, from its components, its
/// gradient ids, or its cycle/pattern string.
bool mixed_filament_compatible(const Slic3r::MixedFilament& mf, size_t num_physical,
                               const std::vector<std::string>& filament_types) {
    using namespace Slic3r;
    std::vector<unsigned int> ids_0based;
    if (!mf.manual_pattern.empty()) {
        const std::string norm = MixedFilamentManager::normalize_manual_pattern(mf.manual_pattern);
        if (norm.empty()) return false;   // a malformed pattern cannot be checked
        for (const std::string& group : MixedFilamentManager::split_pattern_groups(norm))
            for (const std::string& token : MixedFilamentManager::split_pattern_group_to_tokens(group, num_physical)) {
                const unsigned int id = MixedFilamentManager::physical_filament_from_token(token, mf, num_physical);
                if (id >= 1 && id <= num_physical) ids_0based.push_back(id - 1);
            }
    } else {
        if (mf.component_a >= 1 && mf.component_a <= num_physical) ids_0based.push_back(mf.component_a - 1);
        if (mf.component_b >= 1 && mf.component_b <= num_physical) ids_0based.push_back(mf.component_b - 1);
        if (!mf.gradient_component_ids.empty())
            for (unsigned int id : MixedFilamentManager::decode_gradient_component_ids(mf.gradient_component_ids))
                if (id >= 1 && id <= num_physical) ids_0based.push_back(id - 1);
    }
    if (ids_0based.empty()) return false;
    return mixed_filament_ids_compatible(ids_0based, filament_types);
}

/// "[1] Name", the desktop's filament_display_label (Plater.cpp:280-300 at
/// bac6fd5), from the run's filament preset names.
std::string filament_slot_label(const std::vector<std::string>& names, int slot_1based) {
    const int index = slot_1based - 1;
    const std::string name = index >= 0 && index < (int)names.size() ? names[index] : std::string("unknown");
    return "[" + std::to_string(slot_1based) + "] " + name;
}

std::string filament_slot_list(const std::vector<std::string>& names, const std::vector<int>& slots_1based) {
    std::string out;
    for (size_t i = 0; i < slots_1based.size(); ++i) {
        if (i != 0) out += ", ";
        out += filament_slot_label(names, slots_1based[i]);
    }
    return out;
}

/// The desktop's pre-slice blockers, in its own order
/// (references/SnapmakerOrca/src/slic3r/GUI/Plater.cpp:21485-21530 at bac6fd5:
/// the temperature mix and the Cool Steel Plate first, then the flow ratio).
/// Returns the refusal code, or 0 to slice. `warning` carries a non-blocking
/// message the desktop shows as a serious warning.
int desktop_pre_slice_blockers(const Slic3r::Model& model, const Slic3r::DynamicPrintConfig& config, int plate_id,
                               bool allow_mix_temp, const std::vector<std::string>& filament_names,
                               int& code, std::string& message, std::string& warning) {
    using namespace Slic3r;
    code = 0;
    const auto* filament_types = config.option<ConfigOptionStrings>("filament_type");
    const int num_filaments = filament_types ? (int)filament_types->values.size() : 0;
    if (num_filaments <= 0)
        return 0;
    // The desktop reads each slot's values from that slot's filament preset; a
    // loaded project's preset holds the file's own values, so they are read
    // here from the settings this plate slices with, at index = slot.
    // A plate may use mixed (virtual) filaments only: the checks on the physical
    // slots below find nothing then, and the mixed-filament check, which
    // collects its own ids from the volumes, still runs.
    const std::set<int> used = desktop_used_filament_slots(model, config, num_filaments);

    // check_filament_temp_mixing (Plater.cpp:22343-22492): one used filament
    // flagged high temperature and another not is a mix the desktop blocks.
    if (!allow_mix_temp) {
        const auto* high_flags = config.option<ConfigOptionBools>("filament_is_high_temperature");
        std::vector<int> high, low;
        for (int slot : used) {
            const bool is_high = high_flags != nullptr && slot < (int)high_flags->values.size() && high_flags->values[slot];
            (is_high ? high : low).push_back(slot + 1);
        }
        if (!high.empty() && !low.empty()) {
            code = CLI_FILAMENTS_DIFFERENT_TEMP;
            message = "Plate " + std::to_string(plate_id) +
                      ": detected both high and low temperature materials (filament_is_high_temperature)."
                      " Mixed printing may result in extruder clogging, nozzle damage, or layer adhesion issues."
                      " High temperature: " + filament_slot_list(filament_names, high) +
                      ". Low temperature: " + filament_slot_list(filament_names, low) +
                      ". Pass --allow-mix-temp to slice anyway.";
            return code;
        }
    }

    // get_cold_plate_compat_state (Plater.cpp:22820-22900): only under the Cool
    // Steel Plate, which reuses the Supertack bed type.
    // curr_bed_type is a project key; a run whose settings come from presets
    // or settings files may not hold it, and opt_enum does not check.
    const BedType bed_type = config.has("curr_bed_type") ? config.opt_enum<BedType>("curr_bed_type") : btCount;
    if (bed_type == btSuperTack) {
        std::vector<int> unsupported, tpu;
        const auto* supertack_temps = config.option<ConfigOptionInts>("supertack_plate_temp");
        const auto* supertack_firsts = config.option<ConfigOptionInts>("supertack_plate_temp_initial_layer");
        for (int slot : used) {
            const std::string type = filament_types != nullptr && slot < (int)filament_types->values.size()
                                         ? filament_types->values[slot] : std::string();
            if (type == "TPU")
                tpu.push_back(slot + 1);
            const int temp = supertack_temps != nullptr && slot < (int)supertack_temps->values.size()
                                 ? supertack_temps->values[slot] : 0;
            const int temp_first = supertack_firsts != nullptr && slot < (int)supertack_firsts->values.size()
                                       ? supertack_firsts->values[slot] : 0;
            if (temp_first <= 0 || temp <= 0)
                unsupported.push_back(slot + 1);
        }
        if (!unsupported.empty()) {
            code = CLI_FILAMENT_NOT_MATCH_BED_TYPE;
            std::string list;
            for (size_t i = 0; i < unsupported.size(); ++i) {
                if (i != 0) list += ", ";
                const int slot = unsupported[i] - 1;
                const int temp = supertack_temps != nullptr && slot < (int)supertack_temps->values.size()
                                     ? supertack_temps->values[slot] : 0;
                const int temp_first = supertack_firsts != nullptr && slot < (int)supertack_firsts->values.size()
                                           ? supertack_firsts->values[slot] : 0;
                list += filament_slot_label(filament_names, unsupported[i]) + " (supertack_plate_temp = " +
                        std::to_string(temp) + ", supertack_plate_temp_initial_layer = " + std::to_string(temp_first) + ")";
            }
            message = "Plate " + std::to_string(plate_id) + ": the Cool Steel Plate is not recommended for " + list +
                      ". Give the filament settings with --filament <file> and set supertack_plate_temp and"
                      " supertack_plate_temp_initial_layer above 0 there, or choose a filament preset that has them"
                      " with --filament-preset <name>.";
            return code;
        }
        if (!tpu.empty())
            warning = "The Cool Steel Plate is not recommended for " + filament_slot_list(filament_names, tpu) +
                      ". It may be hard to remove. Use a textured PEI plate or heat the bed.";
    }

    // has_incompatible_mixed_filament_in_use (Plater.cpp:13466-13517 at
    // bac6fd5): a plate that uses an enabled mixed (virtual) filament whose
    // component categories the compatibility table marks incompatible cannot be
    // sliced.
    {
        size_t physical_filaments = 0;
        Slic3r::MixedFilamentManager mixed = mixed_manager_from_config(config, physical_filaments);
        int enabled_rows = 0;
        for (const Slic3r::MixedFilament& row : mixed.mixed_filaments())
            if (row.enabled && !row.deleted) ++enabled_rows;
        if (enabled_rows > 0)
            emit({{"event","config_normalized"}, {"tag","MixedFilamentsRead"},
                        {"plate_id", plate_id}, {"physical", int(physical_filaments)},
                        {"enabled_rows", enabled_rows},
                        {"total_filaments", int(mixed.total_filaments(physical_filaments))},
                        {"message","The project states " + std::to_string(enabled_rows) + " enabled mixed filament(s): " +
                                   std::to_string(mixed.total_filaments(physical_filaments)) +
                                   " filaments in all, physical and mixed"}});
        // The mixed manager numbers its rows after max(filament_colour, nozzle
        // count) (mixed_manager_from_config, as Print::apply does), not after the
        // filament_type list, so the virtual ids are counted against that.
        const int physical_count = std::max<int>(1, (int) physical_filaments);
        std::set<unsigned int> used_virtual;
        for (const ModelObject* object : model.objects)
            for (const ModelVolume* volume : object->volumes)
                for (int id : volume->get_extruders())
                    if (id > physical_count) used_virtual.insert((unsigned int) id);
        for (unsigned int id : used_virtual) {
            const Slic3r::MixedFilament* mf = mixed.mixed_filament_from_id(id, (size_t) physical_count);
            if (mf == nullptr || !mf->enabled || mf->deleted) continue;
            if (mixed_filament_compatible(*mf, (size_t) physical_count, filament_types->values)) continue;
            auto describe = [&](unsigned int component) {
                const int index = (int) component - 1;
                const std::string type = index >= 0 && index < (int) filament_types->values.size()
                                             ? filament_types->values[index] : std::string("unknown");
                return filament_slot_label(filament_names, (int) component) + " (" + type + ")";
            };
            code = CLI_VALIDATE_ERROR;
            message = "Plate " + std::to_string(plate_id) + ": filament slot [" + std::to_string(id) +
                      "] is a mixed filament of " + describe(mf->component_a) + " and " + describe(mf->component_b) +
                      ", and the filament compatibility table marks those materials incompatible."
                      " mixed_filament_definitions is a project setting, not a filament one, so correct it on the"
                      " command line with --mixed-filament-definitions \"<rows>\" (each row lists the two component"
                      " filaments it mixes), or in the process settings given with --load-settings.";
            return code;
        }
    }

    // check_flow_ratio_zero (Plater.cpp:22669-22770): a used filament whose
    // flow ratio is not above zero extrudes nothing.
    // filament_flow_ratio is a flow-variant option: one group per filament,
    // sized by that filament's filament_flow_step_size (PresetBundle.cpp:
    // 2673-2710 at bac6fd5). The desktop reads each used filament's own first
    // value (Plater.cpp:22764), which is its group's start, found as
    // get_config_idx does (PrintConfig.cpp:7571-7583).
    const auto* flow_ratios = config.option<ConfigOptionFloats>("filament_flow_ratio");
    const auto* step_sizes  = config.option<ConfigOptionInts>("filament_flow_step_size");
    const auto group_start = [step_sizes](int slot) {
        if (step_sizes == nullptr || step_sizes->values.empty())
            return (size_t) slot;
        size_t start = 0;
        for (int i = 0; i < slot; ++i)
            start += i < (int) step_sizes->values.size() ? (size_t) std::max(1, step_sizes->values[i]) : 1;
        return start;
    };
    std::vector<int> offenders;
    for (int slot : used) {
        // Only a value that is there and not above zero blocks: the desktop
        // reads each preset's own value and has nothing to block on otherwise.
        const size_t idx = group_start(slot);
        if (flow_ratios != nullptr && idx < flow_ratios->values.size() && flow_ratios->values[idx] <= 0.0)
            offenders.push_back(slot + 1);
    }
    if (!offenders.empty()) {
        code = CLI_VALIDATE_ERROR;
        message = "Plate " + std::to_string(plate_id) +
                  ": flow ratio is 0%, resulting in zero extrusion and no valid toolpath. Filament(s): ";
        for (size_t i = 0; i < offenders.size(); ++i) {
            if (i != 0) message += "; ";
            const int slot = offenders[i] - 1;
            const size_t idx = group_start(slot);
            const double ratio = flow_ratios != nullptr && idx < flow_ratios->values.size()
                                     ? flow_ratios->values[idx] : 0.;
            char ratio_text[32];
            std::snprintf(ratio_text, sizeof ratio_text, "%g", ratio);
            message += filament_slot_label(filament_names, offenders[i]) + " (filament_flow_ratio = " +
                       ratio_text + ")";
        }
        message += ". Give the filament settings with --filament <file> and set filament_flow_ratio above 0 there,"
                   " or choose a filament preset that has it with --filament-preset <name>.";
        return code;
    }
    return 0;
}

} // namespace slicer_cli

#endif // ENGINE_SNAPMAKER
