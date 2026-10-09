// desktop_presets.cpp — see desktop_presets.hpp.
#include "desktop_presets.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <set>
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

/// The file names an "include" key holds: one name, or a JSON array of them
/// (BambuStudio reads the raw key-value and takes the string between quotes or
/// the JSON array out of it, PresetBundle.cpp 4839-4861 at 5873b5f).
std::vector<std::string> included_names(const std::map<std::string, std::string>& key_values) {
    std::vector<std::string> out;
    const auto it = key_values.find("include");
    if (it == key_values.end() || it->second.size() < 2)
        return out;
    const std::string& text = it->second;
    if (text.front() == '"' && text.back() == '"') {
        out.push_back(text.substr(1, text.size() - 2));
        return out;
    }
    if (text.front() == '[' && text.back() == ']') {
        try {
            for (const auto& item : nlohmann::json::parse(text))
                if (item.is_string())
                    out.push_back(item.get<std::string>());
        } catch (...) {
        }
    }
    return out;
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
    Slic3r::DynamicPrintConfig base;
    const auto inherits = key_values.find("inherits");
    if (inherits != key_values.end() && !inherits->second.empty()) {
        Slic3r::DynamicPrintConfig parent;
        std::map<std::string, std::string> parent_meta;
        if (!load_flat_preset(folder, inherits->second, parent, parent_meta, depth + 1))
            return false;
        base = std::move(parent);
    }
    // "include": further files the preset names, each contributing the keys it
    // states at a value of its own, applied after the parent chain and before
    // the preset's own keys (BambuStudio PresetBundle.cpp 4845-4881:
    // `config.apply_only(include_config, include_config.diff(*include_default_config))`
    // per included file, then `config.apply(config_src)` at 4888). Only the
    // BBL tree uses it — 29 machine and 1393 filament presets, no process
    // preset, and no OrcaSlicer preset (its vendor loader reads no such key).
    // Every included file is a leaf: none of the 1482 names in that tree
    // states "inherits" or "include" of its own, so it is taken as its own
    // file, the way the loader stores a gcode template (4866-4870).
    for (const std::string& included_name : included_names(key_values)) {
        Slic3r::DynamicPrintConfig included;
        std::map<std::string, std::string> included_meta;
        if (!load_flat_preset(folder, included_name, included, included_meta, depth + 1))
            continue;   // 4883: an include that cannot be found is reported and the rest carry on
        const Slic3r::DynamicPrintConfig defaults = Slic3r::DynamicPrintConfig::full_print_config();
        base.apply_only(included, included.diff(defaults), /*ignore_nonexistent=*/true);
    }
    // The file's own keys win, the way the vendor loader merges each preset
    // over its parent (`config = *default_config; config.apply(config_src)`,
    // PresetBundle.cpp 4932-4936). DynamicConfig::operator+= cannot be used:
    // it assigns through the ConfigOption base, which copies nothing.
    base.apply(own, /*ignore_nonexistent=*/true);
    out = std::move(base);
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

/// The file a vendor bundle's `<type>_list` names for `name`, else the folder of
/// that kind beside it. Empty when the bundle holds neither.
boost::filesystem::path preset_file_in_vendor(const std::string& profiles_dir, const std::string& vendor,
                                              const char* list_key, const std::string& type,
                                              const std::string& name) {
    namespace fs = boost::filesystem;
    if (name.empty() || vendor.empty())
        return {};
    const fs::path vendor_dir = fs::path(profiles_dir) / vendor;
    try {
        nlohmann::json j;
        boost::nowide::ifstream ifs((fs::path(profiles_dir) / (vendor + ".json")).string());
        ifs >> j;
        if (j.contains(list_key) && j[list_key].is_array())
            for (const auto& entry : j[list_key]) {
                if (!entry.is_object() || entry.value("name", std::string()) != name)
                    continue;
                const fs::path file = vendor_dir / entry.value("sub_path", std::string());
                if (fs::exists(file))
                    return file;
                break;
            }
    } catch (...) {
    }
    const fs::path plain = vendor_dir / type / (name + ".json");
    return fs::exists(plain) ? plain : fs::path();
}

/// One vendor bundle's preset `name` of `type`, over its whole "inherits" chain,
/// every hop resolved inside that same bundle: the entry's sub_path under the
/// vendor folder names each file, the list the desktop reads
/// (load_vendor_configs_from_json, OrcaSlicer PresetBundle.cpp 4805-4812 at
/// 31f6803), and a name the bundle's list does not hold is looked for in the
/// folder of that kind beside it. The chain stays in the bundle because a
/// preset inherits within it: thirty bundles list an `fdm_filament_pet` of
/// their own, and only the file's own bundle has the one it means. (Walking it
/// folder by folder instead loses the vendors that keep parents in a subfolder:
/// OrcaFilamentLibrary's `filament/base/`, which every Generic filament
/// inherits from.)
bool load_flat_preset_in_vendor(const std::string& profiles_dir, const std::string& vendor, const char* list_key,
                                const std::string& type, const std::string& name, Slic3r::DynamicPrintConfig& out,
                                std::map<std::string, std::string>& meta, int depth = 0) {
    namespace fs = boost::filesystem;
    if (depth > 16 || name.empty())
        return false;
    const fs::path file = preset_file_in_vendor(profiles_dir, vendor, list_key, type, name);
    if (file.empty())
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
    Slic3r::DynamicPrintConfig base;
    const auto inherits = key_values.find("inherits");
    if (inherits != key_values.end() && !inherits->second.empty() &&
        !load_flat_preset_in_vendor(profiles_dir, vendor, list_key, type, inherits->second, base, meta, depth + 1))
        return false;   // a parent the bundle does not hold: the chain is not this bundle's
    // "include": further files beside this one, each contributing the keys it
    // states at a value of its own (BambuStudio PresetBundle.cpp 4845-4881).
    for (const std::string& included_name : included_names(key_values)) {
        Slic3r::DynamicPrintConfig included;
        std::map<std::string, std::string> included_meta;
        if (!load_flat_preset(file.parent_path(), included_name, included, included_meta, depth + 1))
            continue;
        const Slic3r::DynamicPrintConfig defaults = Slic3r::DynamicPrintConfig::full_print_config();
        base.apply_only(included, included.diff(defaults), /*ignore_nonexistent=*/true);
    }
    base.apply(own, /*ignore_nonexistent=*/true);
    out  = std::move(base);
    meta = std::move(key_values);
    return true;
}

/// The system preset `name` of one vendor bundle.
bool find_preset_in_vendor(const boost::filesystem::path& profiles_dir, const std::string& vendor,
                           const char* list_key, const std::string& name, Slic3r::DynamicPrintConfig& out,
                           std::map<std::string, std::string>& meta) {
    const std::string key = list_key;
    const std::string type = key.substr(0, key.find('_'));
    return load_flat_preset_in_vendor(profiles_dir.string(), vendor, list_key, type, name, out, meta);
}

/// The bundled system preset of `type` ("machine", "process", "filament") named
/// `name`. `vendor` is the bundle to read it from when the caller knows it - the
/// leaf file's own vendor, which is where a preset inherits from: five dozen
/// vendors ship a file named fdm_machine_common, each its own (OrcaSlicer's
/// Anker, Anycubic, BBL, Creality ... bundle lists), so a name alone is not
/// enough to know which one a parent means. With no vendor named the bundles
/// are read in name order, the order the desktop loads them, where the first
/// bundle that lists the name owns it (load_vendor_configs_from_json refuses a
/// second copy: "has already been loaded from another Config Bundle",
/// OrcaSlicer PresetBundle.cpp 4855 at 31f6803).
bool find_system_preset(const std::string& profiles_dir, const char* list_key, const std::string& name,
                        Slic3r::DynamicPrintConfig& out, std::map<std::string, std::string>& meta,
                        const std::string& vendor = {}) {
    namespace fs = boost::filesystem;
    if (name.empty())
        return false;
    if (!vendor.empty() && find_preset_in_vendor(profiles_dir, vendor, list_key, name, out, meta))
        return true;
    std::vector<fs::path> vendor_jsons;
    try {
        for (fs::directory_iterator it{fs::path(profiles_dir)}, end; it != end; ++it)
            if (it->path().extension() == ".json")
                vendor_jsons.push_back(it->path());
    } catch (...) {
        return false;
    }
    std::sort(vendor_jsons.begin(), vendor_jsons.end(),
              [](const fs::path& a, const fs::path& b) { return a.filename().string() < b.filename().string(); });
    for (const fs::path& vendor_json : vendor_jsons) {
        // The whole chain comes from the bundle that lists the name, not just
        // its first hop.
        std::map<std::string, std::string> candidate_meta;
        Slic3r::DynamicPrintConfig candidate;
        const std::string key = list_key;
        if (load_flat_preset_in_vendor(profiles_dir, vendor_json.stem().string(), list_key,
                                       key.substr(0, key.find('_')), name, candidate, candidate_meta)) {
            out  = std::move(candidate);
            meta = std::move(candidate_meta);
            return true;
        }
    }
    return false;
}

/// The system process preset `name`, the list vendor_process_files() reads.
bool find_system_process(const std::string& profiles_dir, const std::string& name,
                         Slic3r::DynamicPrintConfig& out, std::map<std::string, std::string>& meta) {
    return find_system_preset(profiles_dir, "process_list", name, out, meta);
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

bool load_bundled_system_preset(const std::string& profiles_dir, const std::string& type, const std::string& name,
                                Slic3r::DynamicPrintConfig& out, const std::string& vendor) {
    // The list the desktop reads this type's presets from
    // (load_vendor_configs_from_json: machine_list, process_list, filament_list;
    // OrcaSlicer PresetBundle.cpp 4805-4812 at 31f6803).
    const char* list_key = type == "machine"  ? "machine_list"
                           : type == "process" ? "process_list"
                           : type == "filament" ? "filament_list"
                                                : nullptr;
    if (list_key == nullptr)
        return false;
    std::map<std::string, std::string> meta;
    return find_system_preset(profiles_dir, list_key, name, out, meta, vendor);
}

/// The bundle a settings file belongs to, when it sits in one:
/// `<profiles_dir>/<Vendor>/<type>/<file>.json` gives `<Vendor>`, the folder a
/// preset's parents come from. Empty for a file of its own (a user preset, an
/// export), whose parents are searched in every bundle.
std::string vendor_of_settings_file(const std::string& profiles_dir, const std::string& type,
                                    const std::string& file) {
    namespace fs = boost::filesystem;
    try {
        const fs::path path = fs::path(file);
        const std::string kind = path.parent_path().filename().string();
        if (kind != type)
            return {};
        const std::string vendor = path.parent_path().parent_path().filename().string();
        if (vendor.empty() || !fs::exists(fs::path(profiles_dir) / vendor))
            return {};
        return vendor;
    } catch (...) {
        return {};
    }
}

InheritsResolution apply_inherited_settings(const std::string& profiles_dir, const std::string& type,
                                           const std::string& from, Slic3r::DynamicPrintConfig& config,
                                           std::string& parent, const std::string& file) {
    parent.clear();
    const auto* inherits = config.option<Slic3r::ConfigOptionString>("inherits");
    if (inherits == nullptr || inherits->value.empty())
        return InheritsResolution::None;
    parent = inherits->value;
    Slic3r::DynamicPrintConfig full;
    // The parents come from the file's own bundle when it sits in one.
    if (!load_bundled_system_preset(profiles_dir, type, parent, full, vendor_of_settings_file(profiles_dir, type, file)))
        return InheritsResolution::NotBundled;
    // The parent chain, then the file's own keys:
    //   * a vendor bundle's file states only its differences from its parents
    //     and the desktop loads it over them (`config = *default_config;
    //     config.apply(config_src)`, OrcaSlicer PresetBundle.cpp 4894,
    //     BambuStudio PresetBundle.cpp 5087);
    //   * a preset a person saved holds the keys it changed and names the
    //     system preset it was saved over, which is the base the desktop
    //     shows its differences against and slices with
    //     (PresetCollection::load_external_preset, OrcaSlicer Preset.cpp
    //     2445-2456: every key the file does not list in
    //     different_settings_to_system takes the parent's value - the same
    //     result, since the file's own keys are applied last either way).
    // `from` is therefore only reported, never a reason to skip the merge.
    (void)from;
    full.apply(config, /*ignore_nonexistent=*/true);
    config = std::move(full);
    // "inherits" and "different_settings_to_system" stay: they are the file's
    // own record of what it was saved over, and the merge reads both (the
    // parent's name for the exported project's inherits_group, the key list
    // for different_settings_to_system).
    return InheritsResolution::Resolved;
}

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
    // from. The bundles are read in name order, as the desktop loads them (see
    // find_system_preset): a name two bundles both list belongs to the first.
    std::vector<fs::path> vendor_jsons;
    try {
        for (fs::directory_iterator it{fs::path(profiles_dir)}, end; it != end; ++it)
            if (it->path().extension() == ".json")
                vendor_jsons.push_back(it->path());
    } catch (...) {
        return {};
    }
    std::sort(vendor_jsons.begin(), vendor_jsons.end(),
              [](const fs::path& a, const fs::path& b) { return a.filename().string() < b.filename().string(); });
    for (const fs::path& file : vendor_jsons) {
        try {
            nlohmann::json j;
            boost::nowide::ifstream ifs(file.string());
            ifs >> j;
            if (!j.contains("machine_list") || !j["machine_list"].is_array())
                continue;
            for (const auto& entry : j["machine_list"])
                if (entry.is_object() && entry.contains("name") && entry["name"].is_string() &&
                    entry["name"].get<std::string>() == printer_preset_name)
                    return file.stem().string();
        } catch (...) {
            continue;
        }
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
