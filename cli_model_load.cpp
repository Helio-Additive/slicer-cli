// cli_model_load.cpp — see cli_model_load.hpp.
#include "cli_model_load.hpp"

#include <algorithm>
#include <cmath>

#include <boost/algorithm/string/predicate.hpp>
#include <boost/filesystem.hpp>

#include "libslic3r/BuildVolume.hpp"
#include "libslic3r/Format/bbs_3mf.hpp"
#include "libslic3r/Preset.hpp"
#include "libslic3r/Geometry.hpp"
#include "libslic3r/Utils.hpp"
#include "libslic3r/miniz_extension.hpp"

#include "cli_events.hpp"

namespace slicer_cli {
namespace {

// MAX_CLONEABLE_SIZE (BambuStudio.cpp 105 at 5873b5f; OrcaSlicer.cpp 109 at 31f6803).
constexpr int kMaxCloneableSize = 512;

struct Kind {
    const char* ext;
    bool        orca;
    bool        bambu;
};

// What each engine's loaders read. Model::read_from_file (BambuStudio
// Model.cpp 292-440: stl, obj, glb, gltf, fbx, amf, 3mf; OrcaSlicer
// Model.cpp 242-400: stl, obj, svg, drc, amf, 3mf) and Model::read_from_step
// for STEP, which both desktop apps call for .step/.stp (BambuStudio
// Plater.cpp 8865-8871, OrcaSlicer Plater.cpp 6537-6544). OrcaSlicer's .drc
// (Draco) reader is not part of this build (CMakeLists.txt leaves out
// Format/DRC.cpp), so .drc is not offered.
const std::vector<Kind>& kinds() {
    static const std::vector<Kind> k = {
        {".stl", true, true},  {".obj", true, true},   {".amf", true, true},  {".3mf", true, true},
        {".step", true, true}, {".stp", true, true},
        {".glb", false, true}, {".gltf", false, true}, {".fbx", false, true},
        {".svg", true, false},
    };
    return k;
}

bool is_step(const std::string& path) {
    return boost::algorithm::iends_with(path, ".step") || boost::algorithm::iends_with(path, ".stp");
}

Slic3r::BoundingBoxf bed_box(const Slic3r::DynamicPrintConfig& config) {
    Slic3r::BoundingBoxf box;
    if (const auto* area = config.option<Slic3r::ConfigOptionPoints>("printable_area"))
        for (const Slic3r::Vec2d& p : area->values)
            box.merge(p);
    if (!box.defined)
        box = Slic3r::BoundingBoxf(Slic3r::Vec2d(0, 0), Slic3r::Vec2d(200, 200));
    return box;
}

std::string file_name(const std::string& path) {
    return boost::filesystem::path(path).filename().string();
}

} // namespace

ThreeMfKind classify_3mf(const std::string& path) {
    if (!boost::algorithm::iends_with(path, ".3mf"))
        return ThreeMfKind::NotA3mf;
    mz_zip_archive zip;
    mz_zip_zero_struct(&zip);
    if (!Slic3r::open_zip_reader(&zip, path))
        return ThreeMfKind::Unreadable;   // missing, or not a readable archive
    const int idx = mz_zip_reader_locate_file(&zip, "Metadata/project_settings.config", nullptr, 0);
    Slic3r::close_zip_reader(&zip);
    return idx < 0 ? ThreeMfKind::GeometryOnly : ThreeMfKind::Project;
}

bool is_loadable_model_file(const std::string& path) {
    for (const Kind& k : kinds()) {
#ifdef ENGINE_ORCA
        if (!k.orca) continue;
#else
        if (!k.bambu) continue;
#endif
        if (boost::algorithm::iends_with(path, k.ext))
            return true;
    }
    return false;
}

std::string loadable_model_extensions() {
    std::string out;
    for (const Kind& k : kinds()) {
#ifdef ENGINE_ORCA
        if (!k.orca) continue;
#else
        if (!k.bambu) continue;
#endif
        out += (out.empty() ? "" : ", ") + std::string(k.ext + 1);
    }
    return out;
}

ModelLoadResult load_geometry_files(const CliOptions& o, const std::vector<std::string>& files,
                                    size_t first_index, Slic3r::Model& model) {
    ModelLoadResult r;
    const auto& loaded_filament_ids = o.cli.option<Slic3r::ConfigOptionInts>("load_filament_ids")->values;
    const auto& clone_objects       = o.cli.option<Slic3r::ConfigOptionInts>("clone_objects")->values;
    // The filaments this run has: the files --load-filaments brings, and the
    // presets --filament-preset names (the named-preset path selects one
    // filament per preset, resolve_named_presets -> set_num_filaments). The
    // official CLI counts m_load_filaments alone (BambuStudio.cpp 2078-2130 at
    // 5873b5f; OrcaSlicer.cpp 1725-1777 at 31f6803): it has no preset path, and
    // on this command line the presets are filaments 1..N just as the files are.
    const size_t load_filaments     = o.cli.option<Slic3r::ConfigOptionStrings>("load_filaments")->values.size() +
                                      o.filament_presets.size();

    for (size_t n = 0; n < files.size(); ++n) {
        const std::string& file = files[n];
        const size_t input_index = first_index + n;
        if (!boost::filesystem::exists(file)) {
            r.code    = CLI_FILE_NOTFOUND;
            r.message = "No such file: " + file;
            return r;
        }
        if (!is_loadable_model_file(file)) {
            r.code    = CLI_DATA_FILE_ERROR;
            r.message = file_name(file) + " is not a model file this engine reads (it reads " +
                        loadable_model_extensions() + ")";
            return r;
        }
        Slic3r::Model loaded;
        try {
            // LoadModel only, as the desktop loads geometry (Plater.cpp
            // 19302-19304): objects come without instances, and either the
            // desktop placement or the official arrange gives them one.
            const auto strategy = Slic3r::LoadStrategy::LoadModel;
            if (is_step(file)) {
                // The desktop's STEP import with its default mesh settings:
                // linear deflection 0.003, angle 0.5, compounds kept whole
                // (AppConfig.cpp 483-490 at 5873b5f, 553-558 at 31f6803).
                loaded = Slic3r::Model::read_from_step(
                    file, strategy, nullptr, nullptr,
                    [](Slic3r::Step&, double& linear, double& angle, bool& split) -> int {
                        linear = 0.003;
                        angle  = 0.5;
                        split  = false;
                        return 1;
                    },
                    0.003, 0.5, false);
            } else {
                Slic3r::DynamicPrintConfig file_config;
                Slic3r::ConfigSubstitutionContext subst(Slic3r::ForwardCompatibilitySubstitutionRule::Enable);
                Slic3r::PlateDataPtrs plates;
                std::vector<Slic3r::Preset*> presets;
                bool is_bbl_3mf = false;
                loaded = Slic3r::Model::read_from_file(file, &file_config, &subst, strategy, &plates, &presets,
                                                       &is_bbl_3mf);
                Slic3r::release_PlateData_list(plates);
                for (Slic3r::Preset* p : presets)
                    delete p;
                // A project 3MF goes first: one later in the list is refused
                // (CLI_FILELIST_INVALID_ORDER, BambuStudio.cpp 1890-1897;
                // OrcaSlicer.cpp 1572-1579), whatever came before it.
                if (is_bbl_3mf && input_index > 0) {
                    r.code    = CLI_FILELIST_INVALID_ORDER;
                    r.message = file_name(file) + " is a project 3MF; give it as the first file.";
                    return r;
                }
            }
        } catch (const std::exception& e) {
            r.code    = CLI_DATA_FILE_ERROR;
            r.message = file_name(file) + ": " + e.what();
            return r;
        }

        // The desktop removes objects whose volume is zero, with a notice
        // (BambuStudio Plater.cpp 8991, OrcaSlicer Plater.cpp 6662).
        if (const int removed = loaded.removed_objects_with_zero_volume(); removed > 0) {
            const std::string msg = std::to_string(removed) + " object(s) with zero volume removed from " + file_name(file);
            emit({{"event", "model_loaded"}, {"tag", "ZeroVolumeObjectsRemoved"}, {"path", file},
                  {"removed", removed}, {"message", msg}});
            r.notes.push_back(msg);
        }
        if (loaded.objects.empty()) {
            // The official CLI skips an empty file ("file is empty",
            // BambuStudio.cpp 2159-2162) and goes on with the others.
            emit({{"event", "model_loaded"}, {"tag", "EmptyModelFileSkipped"}, {"path", file},
                  {"message", "Skipped " + file_name(file) + ": it holds no object"}});
            continue;
        }
        // The desktop asks whether a very small model is in meters or inches
        // (BambuStudio Plater.cpp 9001-9020, OrcaSlicer Plater.cpp 6665-6684).
        // A command line cannot ask: it says so, and --convert-unit converts.
        if (!o.given_flag("convert_unit")) {
            const char* unit = loaded.looks_like_saved_in_meters() ? "meters"
                             : loaded.looks_like_imperial_units()  ? "inches" : nullptr;
            if (unit) {
                const std::string msg = "Warning: the object from " + file_name(file) +
                                        " is very small and may be in " + unit +
                                        "; add --convert-unit to scale it to millimeters";
                emit({{"event", "model_warning"}, {"tag", "ModelUnitsLookWrong"}, {"path", file},
                      {"looks_like", unit}, {"message", msg}});
                r.notes.push_back(msg);
            }
        }

        // --load-filament-ids and --clone-objects, per input file
        // (BambuStudio.cpp 2078-2130; OrcaSlicer.cpp 1725-1777).
        int object_extruder_id = 0, clone_count = 1;
        if (loaded_filament_ids.size() > input_index && loaded_filament_ids[input_index] > 0) {
            if (size_t(loaded_filament_ids[input_index]) > load_filaments) {
                r.code    = CLI_INVALID_PARAMS;
                r.message = "--load-filament-ids: filament " + std::to_string(loaded_filament_ids[input_index]) +
                            " for file " + std::to_string(input_index + 1) + " is past the " +
                            std::to_string(load_filaments) + " filament(s) loaded; give --load-filaments with at least " +
                            std::to_string(loaded_filament_ids[input_index]) + " file(s)";
                return r;
            }
            object_extruder_id = loaded_filament_ids[input_index];
        }
        if (clone_objects.size() > input_index && clone_objects[input_index] > 0) {
            if (clone_objects[input_index] > kMaxCloneableSize) {
                r.code    = CLI_INVALID_PARAMS;
                r.message = "--clone-objects: " + std::to_string(clone_objects[input_index]) + " copies for file " +
                            std::to_string(input_index + 1) + " is more than " + std::to_string(kMaxCloneableSize) +
                            "; give " + std::to_string(kMaxCloneableSize) + " copies or fewer";
                return r;
            }
            clone_count = clone_objects[input_index];
        }
        if (clone_count > 1) {
            const size_t object_count = loaded.objects.size();
            for (size_t i = 0; i < object_count; ++i) {
                Slic3r::ModelObject* object = loaded.objects[i];
                for (int c = 1; c < clone_count; ++c) {
                    Slic3r::ModelObject* copy = loaded.add_object(*object);
                    copy->name = object->name + "_" + std::to_string(c + 1);
                }
                object->name = object->name + "_" + std::to_string(1);
            }
        }
        for (Slic3r::ModelObject* object : loaded.objects) {
            if (object_extruder_id != 0)
                object->config.set_key_value("extruder", new Slic3r::ConfigOptionInt(object_extruder_id));
            object->ensure_on_bed();
        }
        for (Slic3r::ModelObject* object : loaded.objects) {
            Slic3r::ModelObject* added = model.add_object(*object);
            added->input_file = file;
        }
        emit({{"event", "model_loaded"}, {"tag", "ModelFileLoaded"}, {"path", file},
              {"objects", loaded.objects.size()},
              {"message", "Loaded " + std::to_string(loaded.objects.size()) + " object(s) from " + file_name(file)}});
    }
    if (model.objects.empty()) {
        r.code    = CLI_NO_SUITABLE_OBJECTS;
        r.message = "The model files hold no object to print; give a file that holds printable objects";
    }
    return r;
}

void desktop_place_on_bed(Slic3r::Model& model, const Slic3r::DynamicPrintConfig& config,
                          std::vector<std::string>* notes) {
    const Slic3r::BoundingBoxf bed = bed_box(config);
    const Slic3r::Vec2d centre = bed.center();
    const Slic3r::Vec3d bed_size = Slic3r::to_3d(bed.size(), 1.0) - 2.0 * Slic3r::Vec3d::Ones();
    std::vector<Slic3r::Polygon> placed;
    for (Slic3r::ModelObject* object : model.objects) {
        // load_files: geometry other than 3MF and AMF is centred on its
        // origin (modifiers left out), and every object is set on the bed
        // (Plater.cpp 9151-9166).
        const bool project_kind = boost::algorithm::iends_with(object->input_file, ".3mf") ||
                                  boost::algorithm::iends_with(object->input_file, ".amf");
        if (!project_kind)
            object->center_around_origin(false);
        object->ensure_on_bed();
        const bool had_instances = !object->instances.empty();
        Slic3r::ModelInstance* new_instance = nullptr;
        if (!had_instances) {
            // load_model_objects: centred again, one instance (9550-9560).
            object->center_around_origin();
            new_instance = object->add_instance();
        }
        // An object far larger than the bed: the desktop scales it down with
        // no choice above 10000 times the bed and asks above 10 times
        // (9566-9590). The command line takes the first and reports the second.
        for (size_t i = 0; i < object->instances.size(); ++i) {
            const Slic3r::Vec3d size  = object->instance_bounding_box(i).size();
            const Slic3r::Vec3d ratio = size.cwiseQuotient(bed_size);
            const double max_ratio = std::max(ratio(0), ratio(1));
            if (max_ratio > 10000) {
                object->scale_mesh_after_creation(float(1. / max_ratio));
                object->origin_translation = Slic3r::Vec3d::Zero();
                object->center_around_origin();
                const std::string msg = "'" + object->name + "' was " + std::to_string(int(max_ratio)) +
                                        " times the bed and was scaled down to fit";
                emit({{"event", "model_loaded"}, {"tag", "ObjectScaledDownToBed"}, {"object", object->name},
                      {"message", msg}});
                if (notes) notes->push_back(msg);
                break;
            } else if (max_ratio > 10) {
                const std::string msg = "Warning: '" + object->name + "' is " + std::to_string(int(max_ratio)) +
                                        " times larger than the bed; --scale scales it";
                emit({{"event", "model_warning"}, {"tag", "ObjectLargerThanBed"}, {"object", object->name},
                      {"message", msg}});
                if (notes) notes->push_back(msg);
            }
        }
        object->ensure_on_bed();
        if (new_instance == nullptr)
            continue;   // instances the file placed stay where they are
        // Auto-placement on load (9600-9625): the bed centre on an empty
        // plate, else the nearest 10 mm cell outside every object already
        // there (GLCanvas3D::get_empty_cells / get_nearest_empty_cell,
        // GLCanvas3D.cpp 6738-6800, step {10, 10} by GLCanvas3D.hpp 1218).
        Slic3r::Vec2d at = centre;
        if (!placed.empty()) {
            const double step = 10.0;
            const double min_x = centre.x() - step * int((centre.x() - bed.min.x()) / step);
            const double min_y = centre.y() - step * int((centre.y() - bed.min.y()) / step);
            std::vector<Slic3r::Vec2d> cells;
            for (double x = min_x; x < bed.max.x() - step / 2; x += step)
                for (double y = min_y; y < bed.max.y() - step / 2; y += step) {
                    const Slic3r::Point p(scale_(x), scale_(y));
                    bool inside = false;
                    for (const Slic3r::Polygon& hull : placed)
                        if (hull.contains(p)) { inside = true; break; }
                    if (!inside)
                        cells.emplace_back(x, y);
                }
            std::stable_sort(cells.begin(), cells.end(), [&](const Slic3r::Vec2d& a, const Slic3r::Vec2d& b) {
                return (a - centre).norm() < (b - centre).norm();
            });
            if (!cells.empty())
                at = cells.front();
            else {
                const double offset = 0.05 * std::max(bed.size().x(), bed.size().y());
                at = centre + Slic3r::Vec2d(offset, offset);
            }
        }
        new_instance->set_offset(Slic3r::Vec3d(at.x(), at.y(), new_instance->get_offset().z()));
        object->invalidate_bounding_box();
        for (const Slic3r::ModelInstance* inst : object->instances)
            placed.push_back(object->convex_hull_2d(inst->get_matrix()));
    }
}

void desktop_center_geometry_3mf(Slic3r::Model& model, const Slic3r::DynamicPrintConfig& config) {
    for (Slic3r::ModelObject* object : model.objects)
        object->ensure_on_bed();
    model.center_instances_around_point(bed_box(config).center());
}

} // namespace slicer_cli
