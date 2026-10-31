// cli_snapmaker.hpp — the Snapmaker Orca build's desktop rules: the
// mixed-filament manager as the slice builds it, the paint remap's state map,
// and the checks the desktop runs before a plate slices.
#pragma once

#ifdef ENGINE_SNAPMAKER

#include <map>
#include <string>
#include <vector>

#include <boost/filesystem/path.hpp>

#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/MixedFilament.hpp"
#include "libslic3r/PresetBundle.hpp"
#include "libslic3r/PrintConfig.hpp"

namespace slicer_cli {

/// The engine's own mixed-filament construction, as the slice does it; with
/// `before`, as a filament count change rebuilds the manager the run had.
/// `physical` gets the physical filament count it numbered against.
Slic3r::MixedFilamentManager mixed_manager_from_config(const Slic3r::DynamicPrintConfig& config, size_t& physical,
                                                       const Slic3r::MixedFilamentManager* before = nullptr);

/// The desktop's paint state map for a filament-id remap.
Slic3r::EnforcerBlockerStateMap paint_state_map(const std::vector<unsigned int>& id_remap, size_t total);

/// The desktop warns when a project's schema is newer than this build, and
/// still loads it: a NewerProjectSchema warning event, never a refusal.
void warn_newer_project_schema(const Slic3r::DynamicPrintConfig& config);

/// The filament total a paint remap maps into: the physical filaments plus
/// the enabled mixed ones, counted as Print::apply counts them.
size_t snapmaker_filament_id_total(const Slic3r::DynamicPrintConfig& config);

/// desktop_pre_slice_blockers for one plate, with the filament names from
/// the settings: returns non-zero with (code, message) for a refusal, and
/// reports a ColdPlateNotRecommended warning itself.
int snapmaker_pre_slice_check(const Slic3r::Model& model, const Slic3r::DynamicPrintConfig& config, int plate_id,
                              bool allow_mix_temp, int& code, std::string& message);

/// The desktop's pre-slice checks for one plate: a refusal (code, message) or
/// a warning. Returns the code, 0 when the plate may slice.
int desktop_pre_slice_blockers(const Slic3r::Model& model, const Slic3r::DynamicPrintConfig& config, int plate_id,
                               bool allow_mix_temp, const std::vector<std::string>& filament_names,
                               int& code, std::string& message, std::string& warning);

/// A --printer-preset switch to a printer that is not a Snapmaker U1, the
/// switch trim_filaments_for_switch follows (Tab.cpp:2247-2270 at bac6fd5).
inline bool switch_trims_filaments(bool preset_switch, const std::string& printer) {
    return preset_switch && !printer.empty() && printer.find("Snapmaker U1") == std::string::npos;
}

/// The same trim on the project's own settings, which --export-3mf and
/// --export-settings write: the filament list the slice used, without the
/// paint remap and events trim_filaments_for_switch does once per plate.
/// Returns whether it trimmed.
bool trim_project_roster_for_switch(Slic3r::DynamicPrintConfig& config);

/// A switch to a printer that is not a Snapmaker U1: trims the plate's filament
/// settings to the printer's extruder count and, once per run, remaps the
/// model's paint, as the desktop does.
void trim_filaments_for_switch(Slic3r::DynamicPrintConfig& config, Slic3r::Model& model, const std::string& printer,
                               int plate_id, bool& paint_remapped);

/// The desktop's flow-type sync before a slice (filament_volume_type).
void sync_filament_volume_types(Slic3r::DynamicPrintConfig& config, int plate_id);

/// The Snapmaker U1's plate types: an unsupported plate becomes Textured PEI.
void force_u1_bed_type(Slic3r::DynamicPrintConfig& config, int plate_id);

/// The named-preset path's switch to a printer that is not a U1: trims the
/// bundle's filament count to the printer's extruder count, as the desktop
/// does. True when the count changed.
bool trim_named_filaments_for_switch(Slic3r::PresetBundle& bundle);

/// The desktop's rules for presets picked by name: the 0.5 floor on
/// filament_max_volumetric_speed (a warning when raised) and the process
/// preset's mixed and dithering keys over the project's.
void apply_named_preset_rules(Slic3r::DynamicPrintConfig& out, const Slic3r::PresetBundle& bundle);

/// The desktop's notes after apply (the engine's hot-bed and nozzle rules,
/// notify_filament_compatibility_after_apply) and its PLA/PETG warning, as
/// warning events; the slice goes on.
void post_apply_filament_notes(const Slic3r::Print& print, const Slic3r::DynamicPrintConfig& config, int plate_id);

/// Item 4 (and 3f): the desktop's rule for filament indices that are out of
/// range on load (GUI_ObjectList.cpp:750-793 at bac6fd5): an object's missing
/// or too-large extruder becomes 0 and its too-large feature-filament keys are
/// erased (a volume's only in an object of several volumes), against the
/// physical + mixed count. A named-preset switch's paint remap (`id_remap`,
/// over `id_total`) is applied first when the switch changed the count.
void fix_out_of_range_filaments(Slic3r::Model& model, const Slic3r::DynamicPrintConfig& config, size_t physical_count,
                                const std::vector<unsigned int>& id_remap, size_t id_total, bool count_changed);

/// The tower's width and depth by Snapmaker's own arithmetic, line for line
/// (references/SnapmakerOrca/src/slic3r/GUI/PartPlate.cpp:2177-2248 at bac6fd5).
/// `rib_width` is the config's wipe_tower_rib_width.
Slic3r::Vec2d snapmaker_wipe_tower_footprint(const Slic3r::DynamicPrintConfig& config, double w, double wipe_volume,
                                            int plate_extruder_size, double layer_height, double rib_width);

/// Snapmaker's default prime-tower corner: x = WIPE_TOWER_DEFAULT_X_POS (13,
/// the I3 printers 0) and y at the middle of the bed, not the top corner
/// (references/SnapmakerOrca/src/Snapmaker_Orca.cpp:3948-3962, 4206-4213,
/// 4296-4303, and PartPlate.cpp:4094 for the desktop's setter, at bac6fd5;
/// the constant is PartPlate.cpp:70).
Slic3r::Vec2d snapmaker_tower_default_corner(const Slic3r::DynamicPrintConfig& config);

/// Item 6: whether a run gets the desktop's default tower corner for its
/// plate (PartPlate.cpp:4094 and :70; the CLI's own at Snapmaker_Orca.cpp:
/// 3948-3962 at bac6fd5). Only a run from a model file (a 3MF states its own
/// position), and only when nothing else placed the tower: --wipe-tower-x/y,
/// --arrange, settings files, the assemble list or an arrange.
bool snapmaker_tower_default_applies(const Slic3r::DynamicPrintConfig& config, bool model_file, bool tower_given,
                                     bool arrange_given);

/// The filaments an object's wall, sparse infill and solid infill add to its
/// plate, by the fork's own rule (PartPlate.cpp:2075-2081 and 83-87 at bac6fd5).
void snapmaker_object_feature_filaments(const Slic3r::ModelObject& object, int glb_wall_extr, int glb_sparse_infill_extr,
                                        int glb_solid_infill_extr, std::vector<int>& plate_extruders);

/// A project made by Snapmaker Orca (Application "Snapmaker_Orca-<version>",
/// Format/bbs_3mf.cpp:6645-6647 at bac6fd5) or OrcaSlicer, read from its model
/// metadata: it already states this engine's values. Snapmaker's loader has
/// no "made by OrcaSlicer" flag (Format/bbs_3mf.hpp).
bool snapmaker_engine_made(const std::map<std::string, std::string>& meta);

/// The tower settings the arrange's tower estimate reads are all there:
/// missing or empty coordinates, or a missing tower scalar, means no tower,
/// as Snapmaker's check_plate_wipe_tower returns with has_wipe_tower false
/// (Snapmaker_Orca.cpp:3186-3220 at bac6fd5).
bool snapmaker_tower_settings_present(const Slic3r::DynamicPrintConfig& config);

/// Snapmaker's own resources root (its flush data and info/*.json are its own
/// bytes), or empty: references/SnapmakerOrca/resources in a checkout,
/// resources/snapmaker in a package.
boost::filesystem::path snapmaker_resources_root(const boost::filesystem::path& exe_dir);

/// The engine writes debug SVGs into data_dir()/"SVG" on some slicing paths
/// (references/SnapmakerOrca/src/libslic3r/utils.cpp:302-320 at bac6fd5, where
/// create_directory throws if the parent is gone). The data directory can be
/// the named-preset staging folder, removed once staging ends: a missing one
/// becomes `fallback`, a folder that lives as long as the slice, and its SVG
/// folder is made.
void snapmaker_keep_data_dir(const std::string& fallback);

} // namespace slicer_cli

#endif // ENGINE_SNAPMAKER

#include <boost/filesystem/path.hpp>

namespace slicer_cli {

/// The Snapmaker Orca profiles tree beside this binary, or empty: a checkout
/// has references/SnapmakerOrca; a package ships it as
/// resources/snapmaker/profiles (Linux: bin/../resources; macOS and Windows:
/// next to the binaries). Every build reads it: --engine-info names the
/// Snapmaker engine from all three binaries.
boost::filesystem::path snapmaker_profiles_dir(const boost::filesystem::path& exe_dir);

} // namespace slicer_cli
