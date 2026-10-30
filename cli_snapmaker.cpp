// cli_snapmaker.cpp — the Snapmaker Orca build's desktop rules (cli_snapmaker.hpp).
// Every rule cites the Snapmaker Orca source it ports, at bac6fd5.
#include "cli_snapmaker.hpp"

#ifdef ENGINE_SNAPMAKER

#include <algorithm>
#include <cmath>
#include <iostream>
#include <map>
#include <set>
#include <sstream>

#include <boost/algorithm/string/predicate.hpp>
#include <boost/filesystem.hpp>
#include <boost/nowide/fstream.hpp>
#include <nlohmann/json.hpp>

#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/FilamentHotBedNozzleRules.hpp"
#include "libslic3r/Preset.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PresetBundle.hpp"
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
/// Why the table could not be read, empty when it was: kept for the refusal
/// sentence, since the load's warning event is emitted once and a multi-plate
/// run's check pass drops the events it held when it passes.
std::string& mix_compatibility_table_problem() {
    static std::string problem;
    return problem;
}

const std::vector<std::vector<bool>>& mix_compatibility_table() {
    static const std::vector<std::vector<bool>> table = [] {
        std::vector<std::vector<bool>> t(10, std::vector<bool>(10, false));
        for (size_t i = 0; i < 9; ++i) t[i][i] = true;
        const std::string path = Slic3r::resources_dir() + "/profiles/Snapmaker/filament/filament_compatibility.json";
        // The desktop logs each way the load fails and keeps what it has
        // (MixedColorMatchHelpers.cpp:1081-1121): here that is a warning event,
        // so a refusal that cites the table can be told from a table never read.
        std::string problem;
        try {
            boost::nowide::ifstream ifs(path);
            if (!ifs.is_open()) {
                problem = "cannot open " + path;
            } else {
                nlohmann::json j;
                ifs >> j;
                if (!j.contains("compatibility"))
                    problem = "no 'compatibility' key in " + path;
                else
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
        } catch (const std::exception& e) {
            problem = "cannot read " + path + ": " + e.what();
        }
        mix_compatibility_table_problem() = problem;
        if (!problem.empty())
            emit({{"event","warning"}, {"tag","FilamentCompatibilityTableNotLoaded"}, {"path", path},
                  {"message","The filament compatibility table was not loaded (" + problem + "); each filament type"
                             " is taken as compatible with itself only, as the desktop does"}});
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
            if (const std::string& problem = mix_compatibility_table_problem(); !problem.empty())
                message += " The filament compatibility table was not loaded (" + problem +
                           "), so only a mix of one material counts as compatible.";
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

/// A switch to a printer that is not a Snapmaker U1 trims the filament list to
/// that printer's extruder count and remaps the paint (Tab.cpp:2247-2270 and
/// Plater.cpp:22281-22290 at bac6fd5). `paint_remapped` is the run's: the model
/// is shared by every plate pass and is remapped once.
void trim_filaments_for_switch(Slic3r::DynamicPrintConfig& config, Slic3r::Model& model, const std::string& printer,
                               int plate_id, bool& paint_remapped) {
    using namespace Slic3r;
    using json = nlohmann::json;
    size_t switch_extruders = 0;
    if (const auto* nozzles = dynamic_cast<const ConfigOptionVectorBase*>(config.option("nozzle_diameter")))
        switch_extruders = nozzles->size();
    const auto* colours = config.option<ConfigOptionStrings>("filament_colour");
    // Tab.cpp:2246 at bac6fd5: only a printer that is not
    // single_extruder_multi_material has its filament count trimmed.
    const bool single_extruder_multi_material =
        config.has("single_extruder_multi_material") && config.opt_bool("single_extruder_multi_material");
    if (!single_extruder_multi_material && switch_extruders > 0 && colours != nullptr &&
        colours->values.size() > switch_extruders) {
        // A flow-variant option (and filament_flow_support) keeps one
        // group of values per filament, each group as long as that
        // filament's filament_flow_step_size (PresetBundle.cpp:2673-2710
        // at bac6fd5), so it is cut after the kept filaments' groups:
        // a plain resize would hand filament 2 filament 1's variants.
        size_t variant_keep = switch_extruders;
        if (const auto* step_sizes = config.option<ConfigOptionInts>("filament_flow_step_size");
            step_sizes != nullptr && !step_sizes->values.empty()) {
            variant_keep = 0;
            for (size_t i = 0; i < switch_extruders; ++i)
                variant_keep += i < step_sizes->values.size() ? (size_t) std::max(1, step_sizes->values[i]) : 1;
        }
        const size_t colours_before = colours->values.size();
        // The mixed filaments before the trim, for the paint remap below.
        size_t physical_before = 0;
        const MixedFilamentManager mixed_before = mixed_manager_from_config(config, physical_before);
        // Finding 9: the presets the switch drops, read before the
        // trim below cuts filament_settings_id.
        json dropped_filaments = json::array();
        if (const auto* names = config.option<ConfigOptionStrings>("filament_settings_id"))
            for (size_t i = switch_extruders; i < names->values.size(); ++i)
                dropped_filaments.push_back(names->values[i]);
        for (const std::string& key : Preset::filament_options()) {
            auto* vec = dynamic_cast<ConfigOptionVectorBase*>(config.option(key, false));
            if (vec == nullptr) continue;
            const bool variant = is_filament_flow_variant_option(key) || key == "filament_flow_support";
            const size_t want = variant ? variant_keep : switch_extruders;
            if (vec->size() > want) vec->resize(want);
        }
        // The group sizes themselves are one per filament and are no
        // filament preset option, so the loop above skips them.
        if (auto* step_sizes = config.option<ConfigOptionInts>("filament_flow_step_size");
            step_sizes != nullptr && step_sizes->values.size() > switch_extruders)
            step_sizes->values.resize(switch_extruders);
        // The other per-filament project lists set_num_filaments and the
        // filament roster keep at one entry per filament
        // (PresetBundle.cpp:2179-2201 at bac6fd5), so they report only the
        // filaments the slice keeps.
        for (const char* key : {"filament_settings_id", "filament_ids", "filament_multi_colors",
                                "filament_colour_mode"})
            if (auto* vec = dynamic_cast<ConfigOptionVectorBase*>(config.option(key, false));
                vec != nullptr && vec->size() > switch_extruders)
                vec->resize(switch_extruders);
        config.option<ConfigOptionStrings>("filament_colour", true)->values.resize(switch_extruders);
        // The paint follows the desktop's switch: set_num_filaments
        // rebuilds the mixed filaments for the new count and builds the
        // old->new filament-id remap (PresetBundle.cpp:3905-3926,
        // build_filament_id_remap 3969-4100 at bac6fd5): a kept physical
        // id keeps its number, a dropped one becomes NONE, and a mixed id
        // follows its row to its new virtual number. The Plater applies
        // it with remap_extruder_ids over the physical + mixed total
        // (Plater.cpp:22225-22283). The engine's own remap is used, on a
        // bundle that only carries the new mixed filaments.
        size_t physical_after = 0;
        const MixedFilamentManager mixed_after = mixed_manager_from_config(config, physical_after, &mixed_before);
        const size_t paint_total = mixed_after.total_filaments(physical_after);
        if (!paint_remapped) {
            std::vector<unsigned int> id_remap;
            {
                PresetBundle remap_bundle;
                remap_bundle.mixed_filaments = mixed_after;
                remap_bundle.update_mixed_filament_id_remap(mixed_before.mixed_filaments(), physical_before,
                                                            physical_after);
                id_remap = remap_bundle.consume_last_filament_id_remap();
            }
            const EnforcerBlockerStateMap state_map = paint_state_map(id_remap, paint_total);
            for (ModelObject* object : model.objects)
                for (ModelVolume* volume : object->volumes)
                    if (volume != nullptr)
                        volume->remap_extruder_ids(paint_total, state_map);
            paint_remapped = true;
        }
        // Finding 7: with fewer filaments, an index that was in range
        // at load can be out of range now; the same reset runs again.
        json shrunk = json::object();
        json dropped = json::array();
        for (size_t oi = 0; oi < model.objects.size(); ++oi) {
            ModelObject* object = model.objects[oi];
            if (object == nullptr) continue;
            auto recheck = [&](ModelConfigObject& cfg, const std::string& where, bool is_object) {
                if (is_object && (!cfg.has("extruder") || (size_t) cfg.extruder() > paint_total)) {
                    const int was = cfg.has("extruder") ? cfg.extruder() : -1;
                    cfg.set_key_value("extruder", new ConfigOptionInt(0));
                    shrunk[where] = {{"from", was}, {"to", 0}};
                }
                for (const char* key : {"wall_filament", "sparse_infill_filament", "solid_infill_filament",
                                        "support_filament", "support_interface_filament"})
                    if (cfg.has(key) && (size_t) cfg.opt_int(key) > paint_total) {
                        dropped.push_back(where + " " + key);
                        cfg.erase(key);
                    }
            };
            recheck(object->config, "object " + std::to_string(oi + 1), true);
            if (object->volumes.size() > 1)
                for (size_t vi = 0; vi < object->volumes.size(); ++vi)
                    if (object->volumes[vi] != nullptr)
                        recheck(object->volumes[vi]->config,
                                "object " + std::to_string(oi + 1) + " volume " + std::to_string(vi + 1), false);
        }
        if (!shrunk.empty() || !dropped.empty())
            emit({{"event","config_normalized"}, {"tag","FilamentIndexOutOfRangeReset"},
                        {"reset_extruder", shrunk}, {"erased_keys", dropped},
                        {"filament_range", int(paint_total)},
                        {"message","After the switch trimmed the filaments, indices past the new "
                                   + std::to_string(paint_total) + "-filament range were reset"}});
        // Finding 9: report what the slice actually uses, and that the
        // switch dropped the rest, so a caller can see it.
        emit({{"event","config_normalized"}, {"tag","FilamentsTrimmedForSwitch"},
                    {"plate_id", plate_id}, {"printer", printer},
                    {"filaments", int(switch_extruders)},
                    {"filaments_before", int(colours_before)},
                    {"dropped_filaments", dropped_filaments},
                    {"message","The switch to '" + printer + "' trims the project's filaments from " +
                               std::to_string(colours_before) + " to " + std::to_string(switch_extruders) +
                               ", as the desktop does"}});
    }
}

/// G: sync_filament_volume_types_for_slice
/// (references/SnapmakerOrca/src/slic3r/GUI/FlowTypeHelper.cpp:187-208 at
/// bac6fd5), called before every slice. The nozzles' flow types come from
/// nozzle_volume_type and the printer's printer_flow_support; when they all
/// agree every filament follows that type, and when they mix the desktop asks
/// the user — headless the file's values are kept and a warning says so.
void sync_filament_volume_types(Slic3r::DynamicPrintConfig& config, int plate_id) {
    using namespace Slic3r;
    const auto* nozzle_types = config.option<ConfigOptionEnumsGeneric>("nozzle_volume_type");
    std::vector<int> types;
    if (nozzle_types != nullptr)
        types = nozzle_types->values;
    if (const auto* nozzles = dynamic_cast<const ConfigOptionVectorBase*>(config.option("nozzle_diameter")))
        types.resize(std::max<size_t>(types.size(), nozzles->size()), 0);
    // The engine's flow-support values are "standard" and "high_flow"
    // (references/SnapmakerOrca/src/libslic3r/PrintConfig.hpp:368-369 at
    // bac6fd5), and its nozzle_volume_types() masks an unsupported
    // high-flow nozzle back to standard before anything counts the
    // distinct types (FlowTypeHelper.cpp:161-176, called from 187-208).
    bool supports_high_flow = false;
    if (const auto* support = config.option<ConfigOptionStrings>("printer_flow_support"))
        supports_high_flow = std::find(support->values.begin(), support->values.end(), "high_flow") !=
                             support->values.end();
    if (!supports_high_flow)
        for (int& type : types) type = 0;
    std::vector<int> distinct = types;
    std::sort(distinct.begin(), distinct.end());
    distinct.erase(std::unique(distinct.begin(), distinct.end()), distinct.end());
    if (distinct.size() >= 2) {
        emit({{"event","warning"}, {"tag","MixedNozzleFlowTypes"},
                    {"plate_id", plate_id},
                    {"message","The nozzles mix standard and high-flow types. The desktop asks which flow"
                               " type the filaments should use; this run keeps the file's filament_volume_type"
                               " values."}});
    } else {
        // 1 (nvtBigTraffic in nozzle_volume_type) is the high-flow type.
        const int wanted = (!types.empty() && types.front() == 1 && supports_high_flow) ? 1 : 0;
        auto* flow_opt = config.option<ConfigOptionEnumsGeneric>("filament_volume_type", true);
        const bool uniform = !flow_opt->values.empty() &&
                             std::all_of(flow_opt->values.begin(), flow_opt->values.end(),
                                         [wanted](int v) { return v == wanted; });
        if (!uniform) {
            size_t filament_count = 1;
            if (const auto* ftypes = config.option<ConfigOptionStrings>("filament_type"))
                filament_count = std::max<size_t>(ftypes->values.size(), 1);
            flow_opt->values.assign(filament_count, wanted);
        }
    }
}

/// H: the U1's bed types (references/SnapmakerOrca/src/slic3r/GUI/Plater.cpp:3615,
/// 3790-3802 at bac6fd5): without support_multi_bed_types a Snapmaker U1 takes
/// Textured PEI, Smooth PEI or Graphic Effect only, and any other plate becomes
/// Textured PEI.
void force_u1_bed_type(Slic3r::DynamicPrintConfig& config, int plate_id) {
    using namespace Slic3r;
    const std::string printer_model = config.has("printer_model") ? config.opt_string("printer_model") : std::string();
    const bool is_u1 = boost::icontains(printer_model, "Snapmaker") && boost::icontains(printer_model, "U1");
    const bool multi_bed = config.has("support_multi_bed_types") && config.opt_bool("support_multi_bed_types");
    if (is_u1 && !multi_bed && config.has("curr_bed_type")) {
        const BedType current = config.opt_enum<BedType>("curr_bed_type");
        if (current != btPTE && current != btPEI && current != btGESP) {
            const auto* bed_def = print_config_def.get("curr_bed_type");
            const std::string before = bed_def != nullptr && (size_t) current < bed_def->enum_labels.size()
                                           ? bed_def->enum_labels[(size_t) current] : std::string("(unknown)");
            config.option<ConfigOptionEnum<BedType>>("curr_bed_type", true)->value = btPTE;
            emit({{"event","warning"}, {"tag","BedTypeForcedForU1"},
                        {"plate_id", plate_id}, {"was", before},
                        {"message","Snapmaker U1 without support_multi_bed_types: plate '" + before +
                                   "' is not one of its plate types, so this run uses Textured PEI."}});
        }
    }
}

/// Item 3b: on a switch to a printer that is not a Snapmaker U1 the desktop
/// trims the filament count to the printer's extruder count, while a U1 keeps
/// the count it had (references/SnapmakerOrca/src/slic3r/GUI/Tab.cpp:2247-2270
/// at bac6fd5; set_num_filaments is its own call, Tab.cpp:2270). Tab.cpp:2246
/// trims only for a printer that is not single_extruder_multi_material
/// (fdm_klipper / fdm_toolchanger printers, the U1 among them, set it).
bool trim_named_filaments_for_switch(Slic3r::PresetBundle& bundle) {
    const Slic3r::DynamicPrintConfig& printer = bundle.printers.get_edited_preset().config;
    const auto* nozzles = printer.option<Slic3r::ConfigOptionFloats>("nozzle_diameter");
    const size_t extruders = nozzles != nullptr ? nozzles->values.size() : 0;
    const bool single_extruder_multi_material =
        printer.has("single_extruder_multi_material") && printer.opt_bool("single_extruder_multi_material");
    if (single_extruder_multi_material || extruders == 0 || extruders == bundle.filament_presets.size())
        return false;
    bundle.set_num_filaments((unsigned int) extruders, std::vector<std::string>());
    return true;
}

void apply_named_preset_rules(Slic3r::DynamicPrintConfig& out, const Slic3r::PresetBundle& bundle) {
    // Item 1: check_filament_max_volumetric_speed (references/SnapmakerOrca/src/slic3r/GUI/ConfigManipulation.cpp:126-163
    // at bac6fd5, run for the filament preset from Tab.cpp:4648): pad the list
    // to the flow-variant count with its own first value and lift anything
    // under 0.5 to 0.5. The desktop asks with a dialog; here it is a warning.
    // The desktop runs it when a filament preset is edited or selected, not on
    // the filaments a 3MF carries — those load into project_config and are
    // never run through ConfigManipulation — so this runs on the named and STL
    // paths, which is where presets by name become the run's filaments.
    // The desktop pads one preset's list; in the merged config full_fff_config
    // has already filled each filament's flow-variant group from that
    // filament's own values (PresetBundle.cpp:2696-2710), so only the 0.5 floor
    // is left, and the desktop says so only when a value was raised.
    if (auto* speeds = out.option<Slic3r::ConfigOptionFloats>("filament_max_volumetric_speed");
        speeds != nullptr && !speeds->values.empty()) {
        bool lifted = false;
        for (double& value : speeds->values)
            if (value < 0.5) { value = 0.5; lifted = true; }
        if (lifted)
            emit({{"event","warning"}, {"tag","MaxVolumetricSpeedCorrected"},
                        {"message","Too small max volumetric speed. Reset to 0.5."}});
    }
    // Item 2: the desktop copies the mixed and dithering keys from the process
    // preset into the project config before PresetBundle applies that config
    // last (references/SnapmakerOrca/src/slic3r/GUI/Tab.cpp:2295-2316,
    // GUI_App.cpp:6798-6826, libslic3r/PresetBundle.cpp:2525 at bac6fd5), so the
    // process wins over a project's own values. Here `out` is that config.
    {
        static const Slic3r::t_config_option_keys mixed_keys = {
            "mixed_filament_gradient_mode", "mixed_filament_height_lower_bound",
            "mixed_filament_height_upper_bound", "mixed_filament_advanced_dithering",
            "mixed_filament_component_bias_enabled", "mixed_filament_surface_indentation",
            "mixed_filament_region_collapse", "mixed_color_layer_height_a", "mixed_color_layer_height_b",
            "dithering_z_step_size", "dithering_local_z_mode", "dithering_local_z_whole_objects",
            "dithering_local_z_infill", "dithering_local_z_direct_multicolor",
            "dithering_step_painted_zones_only", "mixed_filament_pointillism_pixel_size",
            "mixed_filament_pointillism_line_gap", "mixed_filament_definitions"};
        out.apply_only(bundle.prints.get_edited_preset().config, mixed_keys, true);
    }
}

void post_apply_filament_notes(const Slic3r::Print& print, const Slic3r::DynamicPrintConfig& config, int plate_id) {
    // notify_filament_compatibility_after_apply
    // (references/SnapmakerOrca/src/slic3r/GUI/Plater.cpp:13388-13450 at
    // bac6fd5): the engine's own hot-bed/nozzle rules, evaluated after
    // apply. Each note is a warning and the slice goes on.
    {
        Slic3r::NozzleFilamentRuleMismatch nozzle_mismatch;
        bool graphic_plate = false, pei_not_pla = false, pei_tpu = false;
        print.filament_rule_mismatch_flags(nozzle_mismatch, graphic_plate, pei_not_pla, pei_tpu);
        if (pei_tpu && pei_not_pla) pei_not_pla = false;   // as the desktop orders them
        auto note = [&](const char* tag, const std::string& text) {
            emit({{"event","warning"}, {"tag",tag}, {"plate_id",plate_id}, {"message",text}});
            std::cout << "Warning: " << text << "\n";
        };
        if (graphic_plate)
            note("GraphicEffectPlateAdhesion",
                 "Note: Low adhesion to the graphic effect plate may cause failure."
                 " Use a different filament instead.");
        if (pei_not_pla)
            note("SmoothPeiAdhesion",
                 "Note: Filament may not adhere well to the smooth PEI plate on the first layer."
                 " Apply glue before printing.");
        if (nozzle_mismatch.has_mismatch)
            note("NozzleFilamentNotRecommended",
                 "Note: Using a " + nozzle_mismatch.nozzle_diameter_mm + " mm " +
                     nozzle_mismatch.nozzle_type_key + " nozzle for " +
                     (nozzle_mismatch.filament_preset_name.empty() ? std::string("(unknown)")
                                                                   : nozzle_mismatch.filament_preset_name) +
                     " is not recommended.");
        if (pei_tpu)
            note("SmoothPeiTpuAdhesion",
                 "Note: Filament may stick too strongly to the smooth PEI plate."
                 " Apply glue to protect the plate and ease part removal.");
    }
    // J: PLA and PETG on one plate
    // (references/SnapmakerOrca/src/slic3r/GUI/GLCanvas3D.cpp:11061-11080 at
    // bac6fd5) — a warning only; the desktop does not block on it.
    if (const auto* plate_types = config.option<Slic3r::ConfigOptionStrings>("filament_type")) {
        bool has_pla = false, has_petg = false;
        for (const std::string& type : plate_types->values) {
            if (type == "PLA") has_pla = true;
            if (type == "PETG") has_petg = true;
        }
        if (has_pla && has_petg)
            emit({{"event","warning"}, {"tag","PlaPetgMix"}, {"plate_id", plate_id},
                        {"message","This plate uses both PLA and PETG filaments."
                                   " PLA and PETG adhere poorly to each other; check the print."}});
    }
}

void fix_out_of_range_filaments(Slic3r::Model& model, const Slic3r::DynamicPrintConfig& config, size_t physical_count,
                                const std::vector<unsigned int>& id_remap, size_t id_total, bool count_changed) {
    using json = nlohmann::json;
    const size_t physical = std::max<size_t>(1, physical_count);
    size_t total = physical;
    {
        size_t manager_physical = 0;
        const Slic3r::MixedFilamentManager mixed = mixed_manager_from_config(config, manager_physical);
        total = mixed.total_filaments(std::max<size_t>(manager_physical, 1));
    }
    const size_t range = std::max<size_t>(total, 1);
    json reset = json::object();
    json erased = json::array();
    auto fix = [&](Slic3r::ModelConfigObject& cfg, const std::string& where, bool is_object) {
        if (is_object && (!cfg.has("extruder") || (size_t) cfg.extruder() > range)) {
            const int was = cfg.has("extruder") ? cfg.extruder() : -1;
            cfg.set_key_value("extruder", new Slic3r::ConfigOptionInt(0));
            reset[where] = {{"from", was}, {"to", 0}};
        }
        for (const char* key : {"wall_filament", "sparse_infill_filament", "solid_infill_filament",
                                "support_filament", "support_interface_filament"}) {
            if (cfg.has(key) && (size_t) cfg.opt_int(key) > range) {
                erased.push_back(where + " " + key);
                cfg.erase(key);
            }
        }
    };
    // The switch's filament-id remap, applied as the desktop does
    // (Plater.cpp:22281-22290): an id past the physical + mixed total
    // becomes NONE, otherwise the remap's target.
    const auto apply_remap = [&](Slic3r::ModelVolume& volume) {
        const size_t total = id_total > 0 ? id_total : range;
        if (!id_remap.empty()) {
            volume.remap_extruder_ids(total, paint_state_map(id_remap, total));
        } else {
            volume.update_extruder_count(total);
        }
    };
    for (size_t oi = 0; oi < model.objects.size(); ++oi) {
        Slic3r::ModelObject* object = model.objects[oi];
        if (object == nullptr) continue;
        if (!id_remap.empty() || count_changed)
            for (Slic3r::ModelVolume* volume : object->volumes)
                if (volume != nullptr)
                    apply_remap(*volume);
        fix(object->config, "object " + std::to_string(oi + 1), true);
        if (object->volumes.size() > 1)
            for (size_t vi = 0; vi < object->volumes.size(); ++vi)
                if (object->volumes[vi] != nullptr)
                    fix(object->volumes[vi]->config, "object " + std::to_string(oi + 1) + " volume " +
                                                         std::to_string(vi + 1), false);
    }
    if (!reset.empty() || !erased.empty())
        emit({{"event","config_normalized"},
                    {"tag","FilamentIndexOutOfRangeReset"},
                    {"reset_extruder", reset},
                    {"erased_keys", erased},
                    {"filament_range", int(range)},
                    {"message","The project states filament indices outside this printer's " +
                               std::to_string(range) + " filament(s) (physical and mixed): an object's "
                               "extruder is reset to 0 (Default), and too-large feature-filament "
                               "keys are dropped, as the desktop does on load"}});
}

Slic3r::Vec2d snapmaker_wipe_tower_footprint(const Slic3r::DynamicPrintConfig& config, double w, double wipe_volume,
                                            int plate_extruder_size, double layer_height, double rib_width) {
    using namespace Slic3r;
    // The width starts at the wipe volume, kept when no branch below sets it
    // (PartPlate.cpp:2184 at bac6fd5).
    Vec2d footprint(wipe_volume, 0.);
    const FullPrintConfig& print_defaults = FullPrintConfig::defaults();
    // Snapmaker rewrote this sizing, so this is its arithmetic line for line
    // (references/SnapmakerOrca/src/slic3r/GUI/PartPlate.cpp:2177-2248 at
    // bac6fd5): the spacing from wipe_tower_extra_spacing, the change volume
    // from the fresh plate Print's filament_multitool_ramming_volume (max, 0.5
    // when unset), volume = prime_volume*n + change*(n-1), no minimum depth by
    // height, and the rib depth without the 1/sqrt(2) factor.
    auto timelapse_type_opt = config.option<ConfigOptionEnum<TimelapseType>>("timelapse_type");
    const bool need_wipe_tower = timelapse_type_opt && timelapse_type_opt->value == TimelapseType::tlSmooth;
    double extra_spacing = 1.0;
    if (const ConfigOption* spacing_opt = config.option("wipe_tower_extra_spacing"))
        extra_spacing = spacing_opt->getFloat() / 100.;
    const auto* use_rib_wall_opt = config.option<ConfigOptionEnum<WipeTowerWallType>>("wipe_tower_wall_type");
    bool use_rib_wall = use_rib_wall_opt ? use_rib_wall_opt->value == WipeTowerWallType::wtwRib : false;
    const double extra_rib_length = print_defaults.wipe_tower_extra_rib_length.value;
    double filament_change_volume = 0.5;
    {
        const std::vector<double>& change_volumes = print_defaults.filament_multitool_ramming_volume.values;
        if (!change_volumes.empty())
            filament_change_volume = *std::max_element(change_volumes.begin(), change_volumes.end());
    }
    const double volume = wipe_volume * plate_extruder_size + filament_change_volume * (plate_extruder_size - 1);
    double depth;
    if (use_rib_wall) {
        depth = std::sqrt(volume / layer_height * extra_spacing);
        if (need_wipe_tower || plate_extruder_size > 1) {
            rib_width = std::min(rib_width, depth / 2);
            depth = rib_width + std::max(depth + extra_rib_length, depth);
            footprint.x() = footprint.y() = depth;
        }
    } else {
        depth = volume / (layer_height * wipe_volume) * extra_spacing;
        footprint.x() = w;
        footprint.y() = depth;
    }
    return footprint;
}

Slic3r::Vec2d snapmaker_tower_default_corner(const Slic3r::DynamicPrintConfig& config) {
    using namespace Slic3r;
    // Snapmaker's own defaults differ: x = WIPE_TOWER_DEFAULT_X_POS (13, the
    // I3 printers 0) and y at the middle of the plate, not the top corner
    // (references/SnapmakerOrca/src/Snapmaker_Orca.cpp:3948-3962, 4206-4213,
    // 4296-4303, and PartPlate.cpp:4094 for the desktop's setter, at bac6fd5;
    // the constant is PartPlate.cpp:70). The plate depth there is the bed's.
    double plate_depth = 0.;
    if (const auto* area = config.option<ConfigOptionPoints>("printable_area"); area && !area->values.empty()) {
        BoundingBoxf bed;
        for (const Vec2d& pt : area->values)
            bed.merge(pt);
        plate_depth = bed.size().y();
    }
    Vec2d corner(13., plate_depth * 0.5);
    if (const auto* structure = config.option<ConfigOptionEnum<PrinterStructure>>("printer_structure");
        structure && structure->value == PrinterStructure::psI3)
        corner.x() = 0.;   // I3_WIPE_TOWER_DEFAULT_X_POS
    return corner;
}

bool snapmaker_tower_default_applies(const Slic3r::DynamicPrintConfig& config, bool model_file, bool tower_given,
                                     bool arrange_given) {
    // A position from settings files, the assemble list or an arrange is the
    // run's own. No Snapmaker system preset states wipe_tower_x/y, so a value
    // other than the engine's own default (PrintConfig.cpp:6196, 6204 at
    // bac6fd5) was set by one of those.
    const auto at_engine_default = [&config](const char* key) {
        const auto* values = config.option<Slic3r::ConfigOptionFloats>(key);
        const auto* def = dynamic_cast<const Slic3r::ConfigOptionFloats*>(
            Slic3r::print_config_def.get(key)->default_value.get());
        if (values == nullptr || def == nullptr || def->values.empty())
            return true;
        for (double v : values->values)
            if (v != def->values.front())
                return false;
        return true;
    };
    return model_file && !tower_given && !arrange_given && at_engine_default("wipe_tower_x") &&
           at_engine_default("wipe_tower_y");
}

void snapmaker_object_feature_filaments(const Slic3r::ModelObject& object, int glb_wall_extr, int glb_sparse_infill_extr,
                                        int glb_solid_infill_extr, std::vector<int>& plate_extruders) {
    using namespace Slic3r;
    // The fork's own rule (PartPlate.cpp:2075-2081 at bac6fd5): a wall or
    // sparse-infill filament other than 1 is used, and solid infill falls
    // back to the global when the object states none.
    int obj_wall_extr = glb_wall_extr;
    if (const ConfigOption* opt = object.config.option("wall_filament"))
        obj_wall_extr = opt->getInt();
    if (obj_wall_extr != 1)
        plate_extruders.push_back(obj_wall_extr);

    // An object's sparse-infill filament of 0 or less inherits the global
    // one (resolve_sparse_infill_filament, PartPlate.cpp:83-87 at bac6fd5).
    int obj_sparse_extr = glb_sparse_infill_extr;
    if (const ConfigOption* opt = object.config.option("sparse_infill_filament"); opt != nullptr && opt->getInt() > 0)
        obj_sparse_extr = opt->getInt();
    if (obj_sparse_extr != 1)
        plate_extruders.push_back(obj_sparse_extr);

    int obj_solid_extr = 1;
    if (const ConfigOption* opt = object.config.option("solid_infill_filament"))
        obj_solid_extr = opt->getInt();
    if (obj_solid_extr != 1)
        plate_extruders.push_back(obj_solid_extr);
    else if (glb_solid_infill_extr != 1)
        plate_extruders.push_back(glb_solid_infill_extr);
}

bool snapmaker_engine_made(const std::map<std::string, std::string>& meta) {
    const auto app = meta.find("Application");
    return meta.count("OrcaSlicer") != 0 ||
           (app != meta.end() && (boost::starts_with(app->second, "Snapmaker_Orca-") ||
                                  boost::starts_with(app->second, "OrcaSlicer-")));
}

bool snapmaker_tower_settings_present(const Slic3r::DynamicPrintConfig& config) {
    using namespace Slic3r;
    const auto* wipe_x = dynamic_cast<const ConfigOptionFloats*>(config.option("wipe_tower_x"));
    const auto* wipe_y = dynamic_cast<const ConfigOptionFloats*>(config.option("wipe_tower_y"));
    if (wipe_x == nullptr || wipe_y == nullptr || wipe_x->size() == 0 || wipe_y->size() == 0)
        return false;
    return config.option("prime_tower_width") != nullptr && config.option("prime_tower_brim_width") != nullptr &&
           config.option("prime_volume") != nullptr &&
           config.option("prime_tower_brim_chamfer_max_width") != nullptr &&
           config.option("prime_tower_brim_chamfer") != nullptr;
}

boost::filesystem::path snapmaker_profiles_dir(const boost::filesystem::path& exe_dir) {
    for (const auto& p : std::vector<boost::filesystem::path>{
        // A package ships the tree once, under this engine's own resources root,
        // because the engine reads its data files beside it at
        // resources_dir()/profiles/Snapmaker/filament/... (FilamentHotBedNozzleRules.cpp:32
        // at bac6fd5). Linux keeps the binaries in bin/, macOS and Windows at the
        // package root.
        exe_dir / ".." / "resources" / "snapmaker" / "profiles",
        exe_dir / "resources" / "snapmaker" / "profiles",
        exe_dir / ".." / "references" / "SnapmakerOrca" / "resources" / "profiles",
        exe_dir / ".." / ".." / "references" / "SnapmakerOrca" / "resources" / "profiles",
    }) {
        if (boost::filesystem::exists(p) && boost::filesystem::is_directory(p))
            return boost::filesystem::canonical(p);
    }
    return boost::filesystem::path();
}

boost::filesystem::path snapmaker_resources_root(const boost::filesystem::path& exe_dir) {
    // Snapmaker's own resources root (its flush data and info/*.json are its
    // own bytes): references/SnapmakerOrca/resources in a checkout,
    // resources/snapmaker in a package.
    for (const auto& p : std::vector<boost::filesystem::path>{
        exe_dir / ".." / "references" / "SnapmakerOrca" / "resources",
        exe_dir / ".." / ".." / "references" / "SnapmakerOrca" / "resources",
        exe_dir / ".." / "resources" / "snapmaker",
        exe_dir / "resources" / "snapmaker",
    }) {
        if (boost::filesystem::exists(p / "info") && boost::filesystem::is_directory(p / "info"))
            return boost::filesystem::canonical(p);
    }
    return boost::filesystem::path();
}

void snapmaker_keep_data_dir(const std::string& fallback) {
    namespace fs = boost::filesystem;
    fs::path data_dir(Slic3r::data_dir());
    if (data_dir.empty() || !fs::exists(data_dir)) {
        boost::system::error_code ec;
        fs::create_directories(fallback, ec);
        if (!fallback.empty())
            Slic3r::set_data_dir(fallback);
    }
    if (!Slic3r::data_dir().empty()) {
        boost::system::error_code ec;
        fs::create_directories(fs::path(Slic3r::data_dir()) / "SVG", ec);
    }
}

} // namespace slicer_cli

#endif // ENGINE_SNAPMAKER
