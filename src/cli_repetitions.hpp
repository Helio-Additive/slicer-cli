// cli_repetitions.hpp — --repetitions N: N copies of the plate, as many as fit.
//
// The official CLI copies every object on the plate (PartPlate::
// duplicate_all_instance) and arranges the plate; when the copies do not all
// fit it searches for the largest count that does, or, with one object, keeps
// the copies that landed (BambuStudio.cpp 5558-6183 at 5873b5f; OrcaSlicer.cpp
// 4817-5440 at 31f6803).
#pragma once

#include <functional>
#include <set>
#include <string>

#include "libslic3r/Model.hpp"
#include "libslic3r/PrintConfig.hpp"

namespace slicer_cli {

/// One arrange of the plate: true when every printable object landed on it;
/// `landed` is how many did.
using ArrangeAttempt = std::function<bool(Slic3r::Model& model, Slic3r::DynamicPrintConfig& config, int& landed)>;

struct RepetitionsResult {
    int  copies_kept = 0;        // the duplicate count that was printed
    bool restored    = false;    // nothing extra fitted: the original plate is printed
};

/// PartPlate::duplicate_all_instance (BambuStudio PartPlate.cpp 2820-2880;
/// OrcaSlicer the same): `dup_count` copies of each printable object, named
/// "<name>_<n>"; skipped objects are set unprintable and not copied; every
/// printable instance takes its own id as its label id.
void duplicate_all_instance(Slic3r::Model& model, unsigned dup_count, const std::set<int>& skip_ids);

/// The official count search around `arrange`. `config`'s wipe_tower_x/y
/// entry for the plate starts at the printer's default corner, as the
/// official does for copies (BambuStudio.cpp 5773-5781).
RepetitionsResult arrange_repetitions(Slic3r::Model& model, Slic3r::DynamicPrintConfig& config, int plate_index,
                                      int duplicate_count, const std::set<int>& skip_ids, const ArrangeAttempt& arrange);

} // namespace slicer_cli
