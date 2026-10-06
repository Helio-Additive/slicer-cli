// cli_model_load.hpp — model files other than a project 3MF.
//
// A project 3MF (one that carries Metadata/project_settings.config) loads in
// main.cpp's 3MF path. Every other model file loads here: STL, OBJ, AMF, a
// 3MF that holds only geometry, STEP, and on the BambuStudio build GLB, glTF
// and FBX; several of them in one call. Placement on the bed is the desktop
// app's (desktop_place_on_bed) unless --arrange 0 or 1 is given: then the
// file's own coordinates are kept (and 1 arranges them).
#pragma once

#include <string>
#include <vector>

#include "libslic3r/Model.hpp"
#include "libslic3r/PrintConfig.hpp"

#include "cli_options.hpp"

namespace slicer_cli {

/// What a .3mf path is, before anything loads it. `Unreadable` is its own
/// answer, not "geometry only": the official CLI checks that every input
/// exists before it loads one (BambuStudio.cpp 1855-1860 at 5873b5f;
/// OrcaSlicer.cpp 1537-1542 at 31f6803) and leaves a file it cannot open to
/// the loader, so a .3mf that cannot be read is neither a project (which
/// carries its own settings) nor geometry only.
enum class ThreeMfKind {
    NotA3mf,       // not a .3mf at all
    Project,       // holds Metadata/project_settings.config (is_bbl_3mf true)
    GeometryOnly,  // holds no project settings, as the official CLI treats it
                   // (is_bbl_3mf false; BambuStudio.cpp 2075-2131 at 5873b5f)
    Unreadable,    // missing, or not a readable zip archive
};

/// Classify `path`. Only a .3mf is opened; every other path is NotA3mf.
ThreeMfKind classify_3mf(const std::string& path);

/// A model file kind slicer-cli can load on this engine.
bool is_loadable_model_file(const std::string& path);

/// The model extensions this engine loads, for messages ("stl, obj, ...").
std::string loadable_model_extensions();

struct ModelLoadResult {
    int         code = 0;        // 0, or the official CLI_* code
    std::string message;         // why, when code != 0
    std::vector<std::string> notes;  // what changed on load, one sentence each
};

/// Loads `files` (each a geometry file) into `model` the official CLI's way
/// (BambuStudio.cpp 1855-2163; OrcaSlicer.cpp 1537-1810): read_from_file
/// one file after the other, objects without instances as the desktop
/// loads them (the caller places them or adds default instances), with
/// --clone-objects and --load-filament-ids applied per file and each object
/// put on the bed. `first_index` is the input position of files[0], for
/// those two per-file lists. Emits the load events.
ModelLoadResult load_geometry_files(const CliOptions& o, const std::vector<std::string>& files,
                                    size_t first_index, Slic3r::Model& model);

/// The desktop app's placement of loaded geometry when nothing arranges it
/// (Plater::priv::load_files and load_model_objects, BambuStudio Plater.cpp
/// 9151-9166 and 9530-9660 at 5873b5f; OrcaSlicer Plater.cpp 6656-6839 and
/// 7020-7140 at 31f6803): each object centred on its own origin and dropped
/// onto the bed, the first at the bed's centre, every next one at the
/// nearest free 10 mm cell (GLCanvas3D::get_nearest_empty_cell).
void desktop_place_on_bed(Slic3r::Model& model, const Slic3r::DynamicPrintConfig& config,
                          std::vector<std::string>* notes = nullptr);

/// The desktop's placement of a 3MF that holds only geometry: its objects
/// keep their layout, moved as one around the bed centre
/// (center_instances_around_point, BambuStudio Plater.cpp 9183-9185).
void desktop_center_geometry_3mf(Slic3r::Model& model, const Slic3r::DynamicPrintConfig& config);

} // namespace slicer_cli
