// desktop_presets.hpp — what the desktop app sets when a printer is picked.
//
// The named-preset path (--printer-preset ...) builds its settings the way
// the desktop app does on a fresh install. Most of that is PresetBundle code
// in libslic3r; the parts below live in each app's GUI code, so they are
// ported here, one function per desktop rule, with the upstream lines cited.
#pragma once

#include <string>

#include "libslic3r/PresetBundle.hpp"
#include "libslic3r/PrintConfig.hpp"

namespace slicer_cli {

/// The plate type the desktop app selects for the printer selected in
/// `bundle`, and in words why.
struct DesktopBedType {
    Slic3r::BedType type = Slic3r::btPC;
    std::string     why;
};
DesktopBedType desktop_bed_type(Slic3r::PresetBundle& bundle);

/// The official name of a plate type ("Textured PEI Plate").
std::string bed_type_name(Slic3r::BedType type);

/// What the desktop app does to the project settings when the printer is
/// picked, on top of selecting presets: on the BambuStudio build, the
/// extruder nozzle statistics (on_printer_model_change). Returns a sentence
/// for the event log, empty when nothing changed.
std::string apply_printer_pick(Slic3r::PresetBundle& bundle);

/// The process preset the desktop app selects when the printer is switched to
/// `printer_preset_name`: the current process is kept while that printer is
/// still among its compatible printers, else the best compatible print preset
/// is taken from the engine's shipped ones, the printer's
/// `default_print_profile` first (PresetBundle::update_compatible and
/// PresetCollection::first_compatible_idx with PreferedPrintProfileMatch,
/// OrcaSlicer PresetBundle.cpp 5295-5330, Preset.hpp 686-709 at 31f6803).
struct DesktopProcessSwitch {
    bool        replaced = false;   ///< the desktop drops the project's process
    std::string name;               ///< the system preset it takes instead
    Slic3r::DynamicPrintConfig config;   ///< that preset, resolved over "inherits"
};

/// `profiles_dir` is the tree holding the vendor folders and `<vendor>.json`;
/// `current_process` the settings the project has now.
DesktopProcessSwitch desktop_printer_switch_process(const std::string& profiles_dir,
                                                    const std::string& vendor,
                                                    const std::string& printer_preset_name,
                                                    const std::string& declared_default,
                                                    const Slic3r::DynamicPrintConfig& current_process);

} // namespace slicer_cli
