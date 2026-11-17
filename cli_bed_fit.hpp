#ifndef SLICER_CLI_BED_FIT_HPP
#define SLICER_CLI_BED_FIT_HPP

#include <cstddef>
#include <optional>

#include "libslic3r/BoundingBox.hpp"

namespace Slic3r {
class ModelObject;
}

namespace slicer_cli {

/// An additional Z rotation in degrees whose footprint bounding box fits the
/// bed's bounding box, or no candidate. The printable-area check stays final.
std::optional<double> fitting_z_rotation(const Slic3r::ModelObject& object, size_t instance_index,
                                         const Slic3r::BoundingBoxf& bed);

} // namespace slicer_cli

#endif
