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

} // namespace slicer_cli
