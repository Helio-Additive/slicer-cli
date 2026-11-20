#ifndef SLICER_CLI_BED_FIT_HPP
#define SLICER_CLI_BED_FIT_HPP

#include <cstddef>
#include <functional>
#include <optional>
#include <string>

#include "libslic3r/BoundingBox.hpp"

namespace Slic3r {
class BuildVolume;
class Model;
class ModelObject;
class DynamicPrintConfig;
}

namespace slicer_cli {

/// An additional Z rotation in degrees whose footprint bounding box fits the
/// bed's bounding box, or no candidate. The printable-area check stays final.
std::optional<double> fitting_z_rotation(const Slic3r::ModelObject& object, size_t instance_index,
                                         const Slic3r::BoundingBoxf& bed);

/// For an instance whose footprint is wider than the printer's rectangular
/// bed in its current pose, but no taller than the bed: the extra Z turn that
/// fits it at full size, or no candidate (a round or shaped bed, a part that
/// fits as it is, or no turn that fits).
std::optional<double> oversized_fitting_z_rotation(const Slic3r::ModelObject& object, size_t instance_index,
                                                   const Slic3r::DynamicPrintConfig& config);

/// Whether --rotate may be offered alone for this run: the run gave no
/// rotation flag (a second --rotate replaces the first, it does not add to
/// it) and its whole model, read before a plate scope hides other plates'
/// objects, is one object with one part and one copy (--rotate turns every
/// object).
bool rotate_flag_eligible(const Slic3r::Model& whole, bool rotation_given);
/// The inside-bed checks a run makes before slicing, on `model`: every copy
/// inside `build_volume`, and with more than one extruder area, every mapped
/// nozzle reaches its filament's parts (extruder_reach_refusal).
bool passes_bed_checks(Slic3r::Model& model, const Slic3r::DynamicPrintConfig& config,
                       const Slic3r::BuildVolume& build_volume, size_t extruder_area_count);
/// The refusal's sentence for that turn, or nothing when the run's own arrange
/// already tried turning (`rotations_tried`: --arrange 1 --allow-rotations).
/// It names --rotate only when `rotate_flag_ok` (rotate_flag_eligible), the
/// turn commutes with the copy's transform, and `turned_fits` accepts the part
/// turned as --rotate turns it (ModelObject::rotate, cli_run_steps.cpp). It
/// names --arrange 1 --allow-rotations only when `arrange_fits` accepts it.
/// Both callbacks run the checks a replay with that flag runs, on copies: the
/// arrange packs into its own bed (get_shrink_bedpts), which on a
/// two-nozzle printer is the area both nozzles reach, not the printable area
/// the turn was found on. With neither, it says nothing. A plate from
/// --load-assemble-list (`from_list`) names no flag: OrcaSlicer takes no
/// transforms with a list (OrcaSlicer.cpp 1530-1535), and on BambuStudio a
/// list part kept at its list position stays over the edge with either flag,
/// so the part is turned in its source file.
std::string full_size_turn_advice(double degrees, const Slic3r::ModelObject& object, size_t instance_index,
                                  bool rotate_flag_ok, bool rotations_tried, bool from_list,
                                  const std::function<bool(Slic3r::Model&)>& turned_fits,
                                  const std::function<bool()>& arrange_fits);

} // namespace slicer_cli

#endif
