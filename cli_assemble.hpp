// cli_assemble.hpp — --load-assemble-list: plates built from a JSON list of
// model files, as the official command lines build them.
//
// The list (BambuStudio.hpp 22-79 at 5873b5f; OrcaSlicer.hpp at 31f6803 has
// the same keys without "subtype"):
//   {"plates": [{"plate_name": "...", "need_arrange": true|false,
//                "plate_params": {"key": "value", ...},
//                "objects": [{"path": "part.stl", "count": N,
//                             "subtype": "ModelPart" (BambuStudio only),
//                             "filaments": [...], "assemble_index": [...],
//                             "pos_x": [...], "pos_y": [...], "pos_z": [...],
//                             "print_params": {...},
//                             "height_ranges": [{"min_z", "max_z", "range_params"}]}],
//                "assembled_params": [{"assemble_index": n, "print_params": {...},
//                                      "height_ranges": [...]}]}]}
// Each per-copy list holds `count` values or one value for every copy.
#pragma once

#include <cassert>   // libslic3r/Color.hpp uses assert without including it
#include <map>
#include <string>
#include <vector>

#include "libslic3r/Color.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/PrintConfig.hpp"

#include "cli_run_steps.hpp"

namespace slicer_cli {

struct AssembleHeightRange {
    float min_z = 0.f;
    float max_z = 0.f;
    std::map<std::string, std::string> range_params;
};

struct AssembledParams {
    std::map<std::string, std::string> print_params;
    std::vector<AssembleHeightRange>   height_ranges;
};

struct AssembleObject {
    std::string                        path;
    Slic3r::ModelVolumeType            subtype = Slic3r::ModelVolumeType::MODEL_PART;
    int                                count = 0;
    std::vector<int>                   filaments;
    std::vector<int>                   assemble_index;
    std::vector<float>                 pos_x, pos_y, pos_z;
    std::map<std::string, std::string> print_params;
    std::vector<AssembleHeightRange>   height_ranges;
};

struct AssemblePlate {
    std::string                        plate_name;
    bool                               need_arrange = false;
    int                                filaments_count = 0;   // the filaments the plate's objects use
    std::map<std::string, std::string> plate_params;
    std::vector<AssembleObject>        objects;
    std::vector<Slic3r::ModelObject*>  loaded_obj_list;       // in AssembleList::model
    std::map<int, AssembledParams>     assembled_params;
};

/// The whole list, built once per run: every plate's objects in one model, as
/// the official builds it before its plate loop.
struct AssembleList {
    std::vector<AssemblePlate>              plates;
    Slic3r::Model                           model;
    std::vector<Slic3r::DynamicPrintConfig> plate_configs;   // each plate's plate_params
    std::vector<Slic3r::RGBA>               colours;         // the OBJ colours, as filaments
};

/// load_assemble_plate_list (BambuStudio.cpp 746-905; OrcaSlicer.cpp 609-764):
/// a missing file is CLI_FILE_NOTFOUND; a list the parser or the counts
/// refuse is CLI_CONFIG_FILE_ERROR.
StepResult load_assemble_plate_list(const std::string& file, std::vector<AssemblePlate>& plates);

/// construct_assemble_list (BambuStudio.cpp 907-1345; OrcaSlicer.cpp
/// 765-1160): reads each object's STL or OBJ, sets its filament, settings,
/// height ranges and position, makes its copies, and merges the parts that
/// share an assemble_index into one "assemble_<n>" object.
StepResult construct_assemble_list(std::vector<AssemblePlate>& plates, Slic3r::Model& model,
                                   std::vector<Slic3r::DynamicPrintConfig>& plate_configs,
                                   std::vector<Slic3r::RGBA>& all_colours);

/// Both, then model.add_default_instances() (BambuStudio.cpp 2165-2188;
/// OrcaSlicer.cpp 1812-1834): an exception while building is
/// CLI_DATA_FILE_ERROR.
StepResult load_assemble_list(const std::string& file, AssembleList& list);

} // namespace slicer_cli
