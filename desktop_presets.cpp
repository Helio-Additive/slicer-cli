// desktop_presets.cpp — see desktop_presets.hpp.
#include "desktop_presets.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <string>
#include <vector>

#include <boost/algorithm/string/trim.hpp>
#include <boost/filesystem.hpp>
#include <boost/nowide/fstream.hpp>
#include <nlohmann/json.hpp>

#include "libslic3r/Config.hpp"
#include "libslic3r/Preset.hpp"

namespace slicer_cli {
namespace {

/// The printer model of the selected printer, or of its parent:
/// Plater::get_curr_printer_model (BambuStudio Plater.cpp 7731-7747 at
/// 5873b5f; OrcaSlicer Plater.cpp 5514-5531 at 31f6803).
const Slic3r::VendorProfile::PrinterModel* curr_printer_model(Slic3r::PresetBundle& bundle) {
    const Slic3r::Preset& curr = bundle.printers.get_selected_preset();
    const Slic3r::VendorProfile::PrinterModel* pm = Slic3r::PresetUtils::system_printer_model(curr);
    if (!pm) {
        if (const Slic3r::Preset* parent = bundle.printers.get_selected_preset_parent())
            pm = Slic3r::PresetUtils::system_printer_model(*parent);
    }
    return pm;
}

/// The plate types the desktop's plate list offers for this printer: every
/// curr_bed_type label, less the model's not_support_bed_types, in enum
/// order (Sidebar::reset_bed_type_combox_choices, BambuStudio Plater.cpp
/// 3717-3753; OrcaSlicer Plater.cpp 2808-2850).
std::vector<Slic3r::BedType> offered_bed_types(const Slic3r::VendorProfile::PrinterModel* pm) {
    std::vector<Slic3r::BedType> out;
    const Slic3r::ConfigOptionDef* def = Slic3r::print_config_def.get("curr_bed_type");
    if (def == nullptr)
        return out;
    int index = 0;
    for (const std::string& label : def->enum_labels) {
        ++index;
        if (pm && def->enum_keys_map &&
            std::find(pm->not_support_bed_types.begin(), pm->not_support_bed_types.end(), label) !=
                pm->not_support_bed_types.end())
            continue;
        out.push_back(Slic3r::BedType(index));
    }
    return out;
}

bool offered(const std::vector<Slic3r::BedType>& list, Slic3r::BedType type) {
    return std::find(list.begin(), list.end(), type) != list.end();
}

/// A system preset's settings over its whole "inherits" chain: the parent's
/// flattened config, then the file's own keys on top
/// (load_vendor_configs_from_json: `config = *default_config;
/// config.apply(config_src)`, OrcaSlicer PresetBundle.cpp 4932-4936 at
/// 31f6803). `meta` takes the file's own key-values (name, type, from,
/// instantiation, inherits): load_from_json keeps those out of the config, and
/// only its substitution-context form leaves "inherits" to the caller
/// (ConfigBase::load_from_json, Config.cpp 811-822, as parse_subfile calls it).
bool load_flat_preset(const boost::filesystem::path& folder, const std::string& name,
                      Slic3r::DynamicPrintConfig& out, std::map<std::string, std::string>& meta, int depth = 0) {
    if (depth > 16 || name.empty())
        return false;
    const boost::filesystem::path file = folder / (name + ".json");
    if (!boost::filesystem::exists(file))
        return false;
    Slic3r::DynamicPrintConfig own;
    std::map<std::string, std::string> key_values;
    std::string                        reason;
    try {
        Slic3r::ConfigSubstitutionContext substitutions(Slic3r::ForwardCompatibilitySubstitutionRule::EnableSilent);
        own.load_from_json(file.string(), substitutions, /*load_inherits_in_config=*/false, key_values, reason);
    } catch (...) {
        return false;
    }
    if (!reason.empty())
        return false;
    const auto inherits = key_values.find("inherits");
    if (inherits != key_values.end() && !inherits->second.empty()) {
        Slic3r::DynamicPrintConfig parent;
        std::map<std::string, std::string> parent_meta;
        if (!load_flat_preset(folder, inherits->second, parent, parent_meta, depth + 1))
            return false;
        out = std::move(parent);
        // The file's own keys win, the way the vendor loader merges each preset
        // over its parent (`config = *default_config; config.apply(config_src)`,
        // PresetBundle.cpp 4932-4936). DynamicConfig::operator+= cannot be used:
        // it assigns through the ConfigOption base, which copies nothing.
        out.apply(own, /*ignore_nonexistent=*/true);
    } else {
        out = std::move(own);
    }
    meta = std::move(key_values);
    return true;
}

/// The vendor's system process presets in the desktop's own order: the list in
/// the vendor profile beside the tree, each entry's sub_path under the vendor
/// folder (load_vendor_configs_from_json, OrcaSlicer PresetBundle.cpp
/// 4606-4632 and 4805-4812 at 31f6803). The collection holds them sorted by
/// name -- load_vendor_configs_from_json calls sort_presets (Preset.hpp
/// 827-832) over Preset::operator< (Preset.hpp 348) -- so the order here is by
/// preset name, not by the vendor list, and a pick that ties takes the first
/// name. A tree without that list falls back to the folder's own files.
std::vector<boost::filesystem::path>
vendor_process_files(const boost::filesystem::path& profiles_dir, const std::string& vendor) {
    namespace fs = boost::filesystem;
    const fs::path vendor_dir = profiles_dir / vendor;
    std::vector<fs::path> out;
    try {
        nlohmann::json j;
        boost::nowide::ifstream ifs((profiles_dir / (vendor + ".json")).string());
        ifs >> j;
        for (const auto& entry : j.at("process_list"))
            out.push_back(vendor_dir / entry.at("sub_path").get<std::string>());
    } catch (...) {
        out.clear();
    }
    if (out.empty()) {
        try {
            for (fs::recursive_directory_iterator it(vendor_dir / "process"), end; it != end; ++it)
                if (it->path().extension() == ".json")
                    out.push_back(it->path());
        } catch (...) {
        }
    }
    std::sort(out.begin(), out.end(), [](const fs::path& a, const fs::path& b) {
        return a.stem().string() < b.stem().string();
    });
    return out;
}

/// The system process preset `name` over its whole "inherits" chain, from
/// whichever vendor tree under `profiles_dir` lists it: that vendor's
/// process_list names the file's sub_path under the vendor folder, the same
/// list vendor_process_files() reads. False when no tree lists the name (a user
/// or external preset), or when its file cannot be read.
bool find_system_process(const std::string& profiles_dir, const std::string& name,
                         Slic3r::DynamicPrintConfig& out, std::map<std::string, std::string>& meta) {
    namespace fs = boost::filesystem;
    if (name.empty())
        return false;
    try {
        for (fs::directory_iterator it{fs::path(profiles_dir)}, end; it != end; ++it) {
            const fs::path vendor_json = it->path();
            if (vendor_json.extension() != ".json")
                continue;
            nlohmann::json j;
            boost::nowide::ifstream ifs(vendor_json.string());
            ifs >> j;
            if (!j.contains("process_list") || !j["process_list"].is_array())
                continue;
            for (const auto& entry : j["process_list"]) {
                if (!entry.is_object() || entry.value("name", std::string()) != name)
                    continue;
                const fs::path file = fs::path(profiles_dir) / vendor_json.stem().string() /
                                      entry.value("sub_path", std::string());
                return load_flat_preset(file.parent_path(), file.stem().string(), out, meta);
            }
        }
    } catch (...) {
    }
    return false;
}

std::vector<std::string> compatible_printers_of(const Slic3r::DynamicPrintConfig& config) {
    if (const auto* list = dynamic_cast<const Slic3r::ConfigOptionStrings*>(config.option("compatible_printers")))
        return list->values;
    return {};
}

/// The logical name ("alias") of a system preset: the name up to the first "@",
/// right-trimmed, else the whole name (load_vendor_configs_from_json,
/// OrcaSlicer PresetBundle.cpp 5013-5027 at 31f6803, which uses a stated
/// "alias" first -- no preset of the BBL tree states one, so deriving it from
/// the name is exact for this tree).
std::string preset_alias_of(const std::string& name) {
    const size_t at = name.find('@');
    if (at == std::string::npos)
        return name;
    std::string alias = name.substr(0, at);
    boost::trim_right(alias);
    return alias;
}

} // namespace

std::string bed_type_name(Slic3r::BedType type) {
    const Slic3r::ConfigOptionDef* def = Slic3r::print_config_def.get("curr_bed_type");
    const int i = int(type) - 1;
    if (def && i >= 0 && i < int(def->enum_values.size()))
        return def->enum_values[size_t(i)];
    return "Default Plate";
}

DesktopBedType desktop_bed_type(Slic3r::PresetBundle& bundle) {
    DesktopBedType out;
    const Slic3r::VendorProfile::PrinterModel* pm = curr_printer_model(bundle);
    const std::vector<Slic3r::BedType> list = offered_bed_types(pm);
    // The plate list's first entry, which is the selection after the list is
    // rebuilt when no rule below selects another.
    const Slic3r::BedType first = list.empty() ? Slic3r::btPC : list.front();
#ifdef ENGINE_ORCA
    // OrcaSlicer, a printer picked with no remembered plate for it
    // (Sidebar::update_all_preset_comboboxes, Plater.cpp 2527-2563 at
    // 31f6803, both sides of its `is_bbl_vendor || support_multi_bed_types`
    // test: with the list rebuilt or without it, the plate comes from
    // get_default_bed_type). Preset::get_default_bed_type (Preset.cpp 931-958
    // at 31f6803) — the printer's default_bed_type when it reads as a number
    // greater than 0 (atoi; a stated NAME, such as the Snapmaker U1's
    // "Textured PEI Plate", reads as 0, is logged as invalid and gives btPEI),
    // else by printer model id (BL-P001/BL-P002/C13 Cool Plate, C11 and every
    // other id btPEI) — then set_bed_type_accord_combox (2798-2806), which
    // takes the list's first entry when the printer does not offer that plate.
    // So the U1 slices on btPEI, the High Temp Plate, whatever plate the
    // desktop's own printer files name.
    Slic3r::Preset& printer = bundle.printers.get_edited_preset();
    const Slic3r::BedType wanted = printer.get_default_bed_type(&bundle);
    if (offered(list, wanted)) {
        out.type = wanted;
        out.why  = "the printer's default plate";
    } else {
        out.type = first;
        out.why  = "the printer does not offer its default plate (" + bed_type_name(wanted) +
                   "); the first plate it offers";
    }
#else
    // BambuStudio, a printer picked on a fresh install
    // (Sidebar::update_all_preset_comboboxes, Plater.cpp 3340-3414 at
    // 5873b5f). A Bambu Lab printer: use_default_bed_type (3761-3773) selects
    // the model's default_bed_type through set_bed_type (3648-3655) and
    // set_bed_type_accord_combox (3703-3715) when the plate list offers it.
    // Any other printer: the list's entry at btPEI - 1 (3412).
    Slic3r::Preset& printer = bundle.printers.get_edited_preset();
    if (printer.is_bbl_vendor_preset(&bundle)) {
        out.type = first;
        out.why  = "the first plate the printer offers (it names no default plate)";
        if (pm && !pm->default_bed_type.empty()) {
            const Slic3r::ConfigOptionDef* def = Slic3r::print_config_def.get("curr_bed_type");
            for (size_t i = 0; def && i < def->enum_values.size(); ++i) {
                if (def->enum_values[i] != pm->default_bed_type)
                    continue;
                const Slic3r::BedType wanted = Slic3r::BedType(i + 1);
                if (offered(list, wanted)) {
                    out.type = wanted;
                    out.why  = "the printer model's default plate";
                } else {
                    out.why = "the printer does not offer its default plate (" + pm->default_bed_type +
                              "); the first plate it offers";
                }
                break;
            }
        }
    } else {
        const size_t index = size_t(Slic3r::btPEI) - 1;
        out.type = index < list.size() ? list[index] : first;
        out.why  = "the plate the desktop app selects for a printer from another maker";
    }
#endif
    return out;
}

std::string apply_printer_pick(Slic3r::PresetBundle& bundle) {
#ifdef ENGINE_ORCA
    (void)bundle;
    return {};
#else
    // The desktop resets the extruder nozzle statistics when the printer
    // model changes (Plater.cpp 8756, Tab.cpp 5640 at 5873b5f):
    // ExtruderNozzleStat::on_printer_model_change (PresetBundle.cpp 301-317),
    // which full_config() writes as extruder_nozzle_stats (PresetBundle.cpp
    // 3526). The official CLI does the same for a file without the key
    // (BambuStudio.cpp 4144-4157, on_printer_model_change_cli).
    const Slic3r::Preset& printer = bundle.printers.get_selected_preset();
    const auto* max_count = printer.config.option<Slic3r::ConfigOptionIntsNullable>("extruder_max_nozzle_count");
    const auto* volume    = bundle.project_config.option<Slic3r::ConfigOptionEnumsGeneric>("nozzle_volume_type");
    if (max_count == nullptr || max_count->values.empty() || volume == nullptr)
        return {};
    bundle.extruder_nozzle_stat.on_printer_model_change(&bundle);
    return "extruder nozzle statistics set from the printer model";
#endif
}

std::string desktop_printer_vendor(const std::string& profiles_dir, const std::string& printer_preset_name) {
    namespace fs = boost::filesystem;
    if (printer_preset_name.empty())
        return {};
    // PresetBundle::find_preset_vendor(TYPE_PRINTER) (OrcaSlicer
    // PresetBundle.cpp 244-318 at 31f6803): every <vendor>.json in the
    // profiles tree is opened, its machine_list searched for the preset's
    // name, and the file's own name (the vendor) returned. The desktop tags
    // each loaded preset with its vendor bundle the same way
    // (load_vendor_configs_from_json, PresetBundle.cpp 4992), and a printer
    // model resolves to its vendor by that list too (PresetBundle.cpp
    // 617-622), so the printer's own vendor is what its process list is read
    // from.
    try {
        for (fs::directory_iterator it{fs::path(profiles_dir)}, end; it != end; ++it) {
            const fs::path file = it->path();
            if (file.extension() != ".json")
                continue;
            nlohmann::json j;
            boost::nowide::ifstream ifs(file.string());
            ifs >> j;
            if (!j.contains("machine_list") || !j["machine_list"].is_array())
                continue;
            for (const auto& entry : j["machine_list"])
                if (entry.is_object() && entry.contains("name") && entry["name"].is_string() &&
                    entry["name"].get<std::string>() == printer_preset_name)
                    return file.stem().string();
        }
    } catch (...) {
    }
    return {};
}

DesktopProcessSwitch desktop_printer_switch_process(const std::string& profiles_dir, const std::string& vendor,
                                                    const std::string& printer_preset_name,
                                                    const std::string& declared_default,
                                                    const Slic3r::DynamicPrintConfig& current_process,
                                                    const std::vector<std::string>& current_compatible_printers,
                                                    const std::string& current_preset_name) {
    namespace fs = boost::filesystem;
    DesktopProcessSwitch out;
    // PresetBundle::update_compatible(select_other_print_if_incompatible)
    // (OrcaSlicer PresetBundle.cpp 5295-5330 at 31f6803) is called
    // SelectCompatibleType::Always when the printer was picked on another page
    // (Tab::select_preset, Tab.cpp 6130-6140), so the current process is
    // dropped when the new printer is not among the printers it says it suits
    // and kept otherwise. The list it checks is the loaded print preset's
    // "compatible_printers", which a project 3MF carries as
    // print_compatible_printers and load_config_file_config puts back
    // (PresetBundle.cpp 4084 and 4129 erase it, 4390-4393 restore it);
    // Preset::is_compatible_with_printer (Preset.cpp 803-813) takes no list as
    // "suits every printer", so a process with no list at all keeps the
    // project's.
    //
    // A Bambu Studio project states no such list -- BambuStudio writes no
    // print_compatible_printers -- and the desktop does not read the missing
    // list as "suits every printer": loading the project selects the system
    // preset the file names, its print_settings_id (else its inherits), as an
    // external preset over that parent (load_external_preset, Preset.cpp
    // 2446-2500), and the edited preset keeps the parent's compatible_printers
    // -- the X1C family -- so a printer outside them re-selects a process.
    // Only a process with neither a list nor a system parent keeps the
    // project's settings.
    std::vector<std::string> suits = current_compatible_printers;
    if (suits.empty()) {
        Slic3r::DynamicPrintConfig            parent;
        std::map<std::string, std::string>    parent_meta;
        if (find_system_process(profiles_dir, current_preset_name, parent, parent_meta))
            suits = compatible_printers_of(parent);
    }
    if (printer_preset_name.empty() || suits.empty() ||
        std::find(suits.begin(), suits.end(), printer_preset_name) != suits.end())
        return out;   // kept: the desktop reselects nothing

    // first_compatible_idx over the compatible, visible presets, scored by
    // PreferedPrintProfileMatch(current preset, the printer's
    // default_print_profile) (Preset.hpp 686-709, PresetBundle.cpp 5215-5247
    // at 31f6803). A preset whose alias is the current preset's alias wins
    // outright (PreferedProfileMatch returns INT_MAX for it, PresetBundle.cpp
    // 5196-5207: "Matching an alias, always take this preset with priority",
    // and first_compatible_idx stops there). The alias of a system preset is
    // the name up to the "@", right-trimmed, unless its file states one
    // (PresetBundle.cpp 5013-5027); the project's current preset is the system
    // preset it was loaded over, so it keeps that alias. Otherwise the score is
    // the printer's default name, +1 for a listed preset and +1 for a visible
    // one, x10 when the layer height is the current process's; ties keep the
    // earlier preset in collection order. A template (instantiation "false") is
    // no preset at all, only a parent (load_vendor_configs_from_json,
    // PresetBundle.cpp 4893-4898).
    const std::string current_alias = preset_alias_of(current_preset_name);
    const auto* current_layer_height_option =
        dynamic_cast<const Slic3r::ConfigOptionFloat*>(current_process.option("layer_height"));
    const double current_layer_height = current_layer_height_option ? current_layer_height_option->value : 0.;
    int          best_quality         = -1;
    for (const fs::path& file : vendor_process_files(profiles_dir, vendor)) {
        Slic3r::DynamicPrintConfig      config;
        std::map<std::string, std::string> meta;
        if (!load_flat_preset(file.parent_path(), file.stem().string(), config, meta))
            continue;
        const std::string name = meta["name"];
        if (name.empty() || meta["type"] != "process" || meta["from"] != "system" || meta["instantiation"] == "false")
            continue;
        const std::vector<std::string> compatible = compatible_printers_of(config);
        if (std::find(compatible.begin(), compatible.end(), printer_preset_name) == compatible.end())
            continue;
        if (!current_alias.empty() && preset_alias_of(name) == current_alias) {
            out.name   = name;
            out.config = std::move(config);
            break;   // INT_MAX: no better match exists
        }
        int quality = (name == declared_default ? 1 : 0) + 2;   // listed and visible
        const auto* candidate_layer_height =
            dynamic_cast<const Slic3r::ConfigOptionFloat*>(config.option("layer_height"));
        if (current_layer_height > 0. && candidate_layer_height != nullptr &&
            std::abs(candidate_layer_height->value - current_layer_height) < 0.0005)
            quality *= 10;
        if (quality > best_quality) {
            best_quality = quality;
            out.name     = name;
            out.config   = std::move(config);
        }
    }
    out.replaced = !out.name.empty();
    // With no compatible system preset the desktop falls back to its built-in
    // "- default -" preset (first_compatible_idx returns index 0, Preset.hpp
    // 703-708); this command line keeps the project's own settings instead,
    // which is not that preset.
    return out;
}

} // namespace slicer_cli
