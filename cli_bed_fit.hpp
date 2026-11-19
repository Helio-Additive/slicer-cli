#ifndef SLICER_CLI_BED_FIT_HPP
#define SLICER_CLI_BED_FIT_HPP

#include <cstddef>
#include <optional>
#include <string>

#include "libslic3r/BoundingBox.hpp"

namespace Slic3r {
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

/// The refusal's sentence for that turn. It names --rotate only when that flag
/// alone puts the part on the bed: the model has one object with one part and
/// one copy, the run gave no rotation flag (a second --rotate replaces the
/// first, it does not add to it), the turn commutes with the copy's transform,
/// and the part turned as --rotate turns it (ModelObject::rotate,
/// cli_run_steps.cpp) lies inside the bed at its kept position. Otherwise it
/// names the arrange with rotations.
std::string full_size_turn_advice(double degrees, const Slic3r::ModelObject& object, size_t instance_index,
                                  const Slic3r::DynamicPrintConfig& config, bool rotation_given);

} // namespace slicer_cli

#endif
