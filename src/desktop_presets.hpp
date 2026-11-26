// desktop_presets.hpp — what the desktop app sets when a printer is picked.
//
// The named-preset path (--printer-preset ...) builds its settings the way
// the desktop app does on a fresh install. Most of that is PresetBundle code
// in libslic3r; the parts below live in each app's GUI code, so they are
// ported here, one function per desktop rule, with the upstream lines cited.
#pragma once

#include <set>
#include <string>
#include <vector>

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

/// What applying a settings file's "inherits" chain did.
enum class InheritsResolution {
    None,        ///< the file states no parent, or (a saved preset) names no differences from it
    Resolved,    ///< the parent's values are now in `config`
    NotBundled,  ///< the file names a parent this engine's profiles tree does not hold
};

/// Reads a settings file the way the desktop reads a preset: the file's own
/// values, and the bundled system preset it names in "inherits". `from` is the
/// file's own "from" ("system" for a vendor bundle's file, else a saved one);
/// `type` is "machine", "process" or "filament"; `parent` reports the name the
/// file inherits; `file` is the file's path, whose own vendor bundle holds the
/// parents (a preset inherits within its bundle). See the definition for the
/// upstream lines of each rule.
InheritsResolution apply_inherited_settings(const std::string& profiles_dir, const std::string& type,
                                           const std::string& from, Slic3r::DynamicPrintConfig& config,
                                           std::string& parent, const std::string& file);

/// The full settings of a bundled system preset of `type` ("machine",
/// "process", "filament") named `name`, out of this engine's own profiles
/// tree: its parents' values with the preset's own keys over them, exactly
/// what the desktop holds after loading the vendor bundles
/// (load_vendor_configs_from_json: `config = *default_config;
/// config.apply(config_src)` — OrcaSlicer PresetBundle.cpp 4894, BambuStudio
/// PresetBundle.cpp 5087 at the pins; the list a type is read from is
/// machine_list / process_list / filament_list, 4805-4812 at 31f6803). False
/// when no vendor list under `profiles_dir` names the preset. `vendor` names
/// the bundle to read it from; empty searches every bundle.
bool load_bundled_system_preset(const std::string& profiles_dir, const std::string& type, const std::string& name,
                                Slic3r::DynamicPrintConfig& out, const std::string& vendor = {});

/// The vendor bundle that holds `printer_preset_name`: the `<vendor>.json` in
/// `profiles_dir` whose machine_list names it (PresetBundle::find_preset_vendor,
/// OrcaSlicer PresetBundle.cpp 244-318 at 31f6803). Empty when no bundle lists
/// it. `profiles_dir` is the tree desktop_printer_switch_process reads.
std::string desktop_printer_vendor(const std::string& profiles_dir, const std::string& printer_preset_name);

/// The process preset the desktop app selects when the printer is switched to
/// `printer_preset_name`: the current process is kept while that printer is
/// still among the printers it says it suits, else the compatible print preset
/// with the current preset's alias, or the printer's `default_print_profile`,
/// is taken from the engine's shipped ones (PresetBundle::update_compatible and
/// PresetCollection::first_compatible_idx with PreferedPrintProfileMatch,
/// OrcaSlicer PresetBundle.cpp 8826-8861, Preset.hpp 834-857 at f92a7c0).
struct DesktopProcessSwitch {
    bool        replaced = false;   ///< the desktop drops the project's process
    std::string name;               ///< the system preset it takes instead
    Slic3r::DynamicPrintConfig config;   ///< that preset, resolved over "inherits"
};

/// `profiles_dir` is the tree holding the vendor folders and `<vendor>.json`;
/// `current_process` the project's settings (for their layer height);
/// `current_compatible_printers` the printers it says it suits (a project 3MF
/// carries that list as print_compatible_printers); `current_preset_name` the
/// system preset the project's process was loaded over, whose alias the
/// selection prefers.
DesktopProcessSwitch desktop_printer_switch_process(const std::string& profiles_dir,
                                                    const std::string& vendor,
                                                    const std::string& printer_preset_name,
                                                    const std::string& declared_default,
                                                    const Slic3r::DynamicPrintConfig& current_process,
                                                    const std::vector<std::string>& current_compatible_printers,
                                                    const std::string& current_preset_name);

#ifdef ENGINE_ORCA
/// A preset named by a name its system preset has since been renamed from is
/// that preset, as the desktop resolves it (PresetBundle::update_system_maps,
/// PresetCollection::get_preset_name_renamed). Rewrites `name` to the current
/// name and returns the old one, or empty when `name` is current or unknown;
/// `announce` emits the PresetRenamed warning event.
std::string rename_to_current(Slic3r::PresetCollection& collection, std::string& name, const char* kind,
                              bool announce = true);

/// rename_to_current for a run's named printer, process and filaments.
void apply_preset_renames(std::string& printer, std::string& process, std::vector<std::string>& filaments,
                          Slic3r::PresetBundle& bundle);
#endif

#ifdef ENGINE_SNAPMAKER
/// A process name with the "mm" the tree's presets spell after the layer
/// height: "0.20 Standard @Snapmaker U1 (0.4 nozzle)" →
/// "0.20mm Standard @Snapmaker U1 (0.4 nozzle)". Unchanged when the name does
/// not start with a number.
std::string declared_with_mm_unit(const std::string& declared);

/// The printer's stated default process, read the way the Snapmaker tree's
/// presets spell it, when it is a listed, compatible preset; empty otherwise.
std::string snapmaker_stated_default_process(const Slic3r::PresetCollection& prints, const std::string& declared,
                                             bool (*is_listed_preset)(const Slic3r::Preset&));

/// The printer's default material: the first listed, compatible preset of the
/// stated default filament's vendor with its filament_type; empty when there
/// is none.
std::string snapmaker_default_material(const Slic3r::PresetCollection& filaments, const std::vector<std::string>& declared,
                                       bool (*is_listed_preset)(const Slic3r::Preset&));
#endif

#ifdef ENGINE_SNAPMAKER
/// A preset this tree ships, by its name or a former one: the fork's
/// load_external_preset matches a name against the renamed_from map
/// (Preset.cpp 2114-2120 at bac6fd5), though its find_preset does not
/// (Preset.cpp 2776-2783).
bool preset_shipped_or_renamed(const Slic3r::PresetCollection& collection, const std::string& name);
#endif

#ifdef ENGINE_BAMBU
/// Opening a Bambu Studio project, the two things the desktop keeps from the
/// file before it loads the settings over this engine's presets: the presets
/// saved inside the project go into `bundle` (Plater.cpp 11953-11958 at
/// v02.09.01.70), so a process or filament the maker saved under a name of
/// their own is found and loads over the system preset it inherits; and the
/// file's own nozzle rack is returned (Plater.cpp 11979-11983), for
/// NozzleInventory::open_project after the load (12160). The bundle gets
/// copies of `presets`; the caller keeps its own.
Slic3r::DynamicPrintConfig load_project_embedded_presets(Slic3r::PresetBundle& bundle,
                                                         const std::vector<Slic3r::Preset*>& presets,
                                                         const Slic3r::DynamicPrintConfig& file_config);
#endif


/// The settings an exported project lists as changed
/// (different_settings_to_system), as the desktop computes them when it saves a
/// project: each preset's values against the system preset it stands on
/// (PresetBundle::full_fff_config -> dirty_options_without_option_list;
/// BambuStudio PresetBundle.cpp 4810-4817, 4857-4863, 5007-5013 at
/// v02.09.01.70; OrcaSlicer PresetBundle.cpp 5585-5593 at f92a7c0). Every app
/// keeps only the listed settings when it opens the project, and so does this
/// command line; the official command lines list only what a settings file
/// stated (BambuStudio.cpp 3594-3599), so a value a flag gave was lost when
/// the export was opened again. `bundle` holds the system presets. Only the
/// list is written into `project`: its values stay the ones that were sliced,
/// and the entries the list already holds are kept. `given` narrows the new
/// entries to the settings the command line itself gave, for a run whose other
/// values are the presets': this command line fills some preset values in its
/// own way (an empty per-extruder list becomes one value per extruder), and
/// those are not the user's changes. Null lists every setting that differs.
/// False, with the reason, when the list could not be computed.
bool record_changed_settings(Slic3r::PresetBundle& bundle, const std::string& path,
                             Slic3r::DynamicPrintConfig& project, const std::set<std::string>* given,
                             std::string& error);

} // namespace slicer_cli
