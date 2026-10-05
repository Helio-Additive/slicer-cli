// desktop_presets.cpp — see desktop_presets.hpp.
#include "desktop_presets.hpp"

#include <algorithm>
#include <vector>

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
    // (Sidebar::update_all_preset_comboboxes, Plater.cpp 2527-2556 at
    // 31f6803): Preset::get_default_bed_type (Preset.cpp 931-958) — the
    // printer's default_bed_type read as a number, else by printer model id
    // (BL-P001/BL-P002/C13 Cool Plate, otherwise High Temp Plate) — then
    // set_bed_type_accord_combox (2798-2806), which takes the list's first
    // entry when the printer does not offer that plate.
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

} // namespace slicer_cli
