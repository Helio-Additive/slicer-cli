// cli_snapmaker.hpp — the Snapmaker Orca build's desktop rules: the
// mixed-filament manager as the slice builds it, the paint remap's state map,
// and the checks the desktop runs before a plate slices.
#pragma once

#ifdef ENGINE_SNAPMAKER

#include <string>
#include <vector>

#include "libslic3r/Model.hpp"
#include "libslic3r/MixedFilament.hpp"
#include "libslic3r/PrintConfig.hpp"

namespace slicer_cli {

/// The engine's own mixed-filament construction, as the slice does it; with
/// `before`, as a filament count change rebuilds the manager the run had.
/// `physical` gets the physical filament count it numbered against.
Slic3r::MixedFilamentManager mixed_manager_from_config(const Slic3r::DynamicPrintConfig& config, size_t& physical,
                                                       const Slic3r::MixedFilamentManager* before = nullptr);

/// The desktop's paint state map for a filament-id remap.
Slic3r::EnforcerBlockerStateMap paint_state_map(const std::vector<unsigned int>& id_remap, size_t total);

/// The desktop's pre-slice checks for one plate: a refusal (code, message) or
/// a warning. Returns the code, 0 when the plate may slice.
int desktop_pre_slice_blockers(const Slic3r::Model& model, const Slic3r::DynamicPrintConfig& config, int plate_id,
                               bool allow_mix_temp, const std::vector<std::string>& filament_names,
                               int& code, std::string& message, std::string& warning);

/// A switch to a printer that is not a Snapmaker U1: trims the plate's filament
/// settings to the printer's extruder count and, once per run, remaps the
/// model's paint, as the desktop does.
void trim_filaments_for_switch(Slic3r::DynamicPrintConfig& config, Slic3r::Model& model, const std::string& printer,
                               int plate_id, bool& paint_remapped);

/// The desktop's flow-type sync before a slice (filament_volume_type).
void sync_filament_volume_types(Slic3r::DynamicPrintConfig& config, int plate_id);

/// The Snapmaker U1's plate types: an unsupported plate becomes Textured PEI.
void force_u1_bed_type(Slic3r::DynamicPrintConfig& config, int plate_id);

} // namespace slicer_cli

#endif // ENGINE_SNAPMAKER
