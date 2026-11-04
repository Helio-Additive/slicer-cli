// cli_layer_ranges.hpp — the layer height of every height range.
//
// The engine reads a layer height out of each range while it builds the layer
// height profile, and it reads it without a check: Slicing.cpp
// layer_height_profile_from_ranges (line 187 in the Bambu Studio 2.8.02.61
// copy, line 249 in the OrcaSlicer 2.4.2 copy) does
// `it_range->second.option("layer_height")->getFloat()`. A range
// that states only settings (a --load-assemble-list "height_ranges" entry, or a
// 3MF's Metadata/layer_config_ranges.xml) has no such key, and the slice
// dereferences a null pointer. The desktop always writes one — the range's
// layer height is what the GUI's height-range editor edits — so the file kinds
// the command line reads are the ones that can leave it out.
//
// A range that states no layer height keeps the object's own height here: its
// own `layer_height` when the object has one, else the project's. That is what
// the range means when only settings are given, and it is the value the run
// would have sliced with had the range not been there.
#pragma once

#include <string>
#include <vector>

namespace Slic3r {
class DynamicPrintConfig;
class Model;
class ModelObject;
} // namespace Slic3r

namespace slicer_cli {

/// One range the run filled, for the event line that reports it.
struct FilledRangeHeight {
    Slic3r::ModelObject* model_object = nullptr;
    std::string object;
    double      min_z        = 0.;
    double      max_z        = 0.;
    double      layer_height = 0.;
};

/// Give every height range of every object a layer height, where the range and
/// the object state none. Returns what it filled, in model order; a second call
/// fills nothing.
std::vector<FilledRangeHeight> fill_range_layer_heights(Slic3r::Model&             model,
                                                        const Slic3r::DynamicPrintConfig& project);

/// Take the filled layer heights out again, so the ranges state what they did
/// before fill_range_layer_heights.
void clear_filled_range_layer_heights(const std::vector<FilledRangeHeight>& filled);

} // namespace slicer_cli
