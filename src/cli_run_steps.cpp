// cli_run_steps.cpp — see cli_run_steps.hpp.
#include "cli_run_steps.hpp"

#include <algorithm>
#include <map>
#include <set>

#include <boost/filesystem.hpp>
#include <boost/nowide/cstdio.hpp>
#include <cstdio>

#include "libslic3r/Format/STL.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/Geometry.hpp"
#include "libslic3r/Orient.hpp"
#if defined(ENGINE_ORCA) && !defined(ENGINE_SNAPMAKER)
#include "libslic3r/LayOnFace.hpp"
#include "libslic3r/MeshRepair.hpp"
#include <functional>
#endif
#include "libslic3r/Utils.hpp"
#include "libslic3r_version.h"

#include "cli_events.hpp"

namespace slicer_cli {
namespace {

#ifdef ENGINE_ORCA
const char* engine_version() { return SoftFever_VERSION; }
#else
const char* engine_version() { return SLIC3R_VERSION; }
#endif

/// CLI::output_filepath for one object (BambuStudio.cpp 8531-8577 at
/// 5873b5f; the same function in OrcaSlicer.cpp at 31f6803):
/// "<dir>/obj_<n>_<name>.stl", where <dir> is the --export-stls folder, or
/// "<outputdir>/stl", or "stl" (object_stl_file). The folder is created if
/// missing.
std::string object_stl_path(const CliOptions& o, const Slic3r::ModelObject& object, unsigned index,
                            const std::string& path_dir) {
    const std::string path = object_stl_file(o, object, index, path_dir);
    // The official creates the last folder only (create_directory), and
    // throws when its parent is missing; the whole path is made here.
    const std::string subdir = !path_dir.empty() ? path_dir : o.outputdir.empty() ? std::string("stl")
                                                                                  : o.outputdir + "/stl";
    boost::system::error_code ec;
    if (!boost::filesystem::exists(subdir, ec))
        boost::filesystem::create_directories(subdir, ec);
    return path;
}

/// CLI::export_models(IO::STL, dir) (BambuStudio.cpp 8427-8465): one STL per
/// object, numbered from 1.
bool export_stls(const CliOptions& o, Slic3r::Model& model, const std::string& dir, std::string& failed_path) {
    unsigned index = 1;
    for (Slic3r::ModelObject* object : model.objects) {
        const std::string path = object_stl_path(o, *object, index++, dir);
        // store_stl reports success even when the file cannot be opened: it
        // drops its writer's false ("FIXME returning false even if write
        // failed", Format/STL.cpp store_stl; TriangleMesh.cpp
        // its_write_stl_binary returns false on a failed fopen "wb"). The
        // official CLI exits 0 then. Here the write is checked: a target that
        // exists must open for writing (the same fopen, without emptying it)
        // or it would keep its old geometry, and the new file must hold the
        // whole mesh (below). Either failure is the official's export failure
        // (CLI_EXPORT_STL_ERROR, BambuStudio.cpp 6391-6392, 6399-6400;
        // OrcaSlicer.cpp 5524-5525, 5532-5533).
        boost::system::error_code ec;
        if (boost::filesystem::exists(path, ec)) {
            FILE* probe = boost::nowide::fopen(path.c_str(), "ab");
            if (probe == nullptr) {
                failed_path = path;
                return false;
            }
            std::fclose(probe);
        }
        // The STL goes to a file next to the target first, and replaces the
        // target only when it holds the whole mesh: a binary STL is 84 bytes
        // of header and count plus 50 bytes per facet (its_write_stl_binary,
        // TriangleMesh.cpp), and the writer checks none of its writes, so a
        // full disk leaves a short or empty file.
        Slic3r::TriangleMesh mesh = object->mesh();
        const uintmax_t expected = 84u + 50u * uintmax_t(mesh.its.indices.size());
        // The staging file is this run's own: a name with a random part,
        // created exclusively ("x": fails if the name exists), so it never
        // takes or removes a file someone else keeps beside the target.
        std::string part;
        const boost::filesystem::path folder = boost::filesystem::path(path).parent_path();
        for (int attempt = 0; attempt < 16 && part.empty(); ++attempt) {
            const std::string name = boost::filesystem::path(path).filename().string() + "." +
                                     boost::filesystem::unique_path("%%%%%%%%%%%%").string() + ".writing";
            const std::string candidate = (folder / name).string();
            if (FILE* created = boost::nowide::fopen(candidate.c_str(), "wbx")) {
                std::fclose(created);
                part = candidate;
            }
        }
        if (part.empty()) {
            failed_path = path;
            return false;
        }
        bool written = Slic3r::store_stl(part.c_str(), &mesh, true);
        if (written) {
            const uintmax_t size = boost::filesystem::file_size(part, ec);
            written = !ec && size == expected;
        }
        if (written) {
            boost::filesystem::rename(part, path, ec);
            written = !ec;
        }
        if (!written) {
            boost::system::error_code ignored;
            boost::filesystem::remove(part, ignored);
            failed_path = path;
            return false;
        }
        emit({{"event", "export"}, {"tag", "StlExported"}, {"path", path},
              {"message", "Exported " + object->name + " as " + path}});
    }
    return true;
}

} // namespace

StepResult apply_transforms(const CliOptions& o, Slic3r::Model& model,
                            const Slic3r::DynamicPrintConfig* process_config, int plate_to_slice,
                            int plate_count, int& duplicate_count) {
    StepResult r;
    const Slic3r::DynamicPrintAndCLIConfig& cfg = o.cli;
    std::map<size_t, bool> orients_requirement;
    for (const std::string& opt_key : o.transforms) {
        if (opt_key == "assemble") {
            // BambuStudio.cpp 4931-4952; OrcaSlicer.cpp 4192-4213.
            if (!cfg.option<Slic3r::ConfigOptionInts>("clone_objects")->values.empty()) {
                r.code    = CLI_INVALID_PARAMS;
                r.message = "--assemble and --clone-objects cannot be used together; give one of them, not both";
                return r;
            }
#if defined(ENGINE_ORCA) && !defined(ENGINE_SNAPMAKER)
            // OrcaSlicer.cpp 4938-4942 at f92a7c0.
            if (model.objects.empty()) {
                r.code    = CLI_INVALID_PARAMS;
                r.message = "--assemble needs at least one input model.";
                return r;
            }
#endif
            Slic3r::Model m;
            Slic3r::ModelObject* new_object = m.add_object();
            new_object->name = "Assembly";
            for (Slic3r::ModelObject* obj : model.objects)
                for (Slic3r::ModelVolume* volume : obj->volumes) {
                    Slic3r::ModelVolume* new_volume = new_object->add_volume(*volume);
                    new_volume->config.set_key_value("extruder", new Slic3r::ConfigOptionInt(obj->config.extruder()));
                }
            // The official adds one instance here and arranges later; a
            // model with nothing placed yet gets its place from the placement step.
            const bool placed = std::all_of(model.objects.begin(), model.objects.end(),
                                            [](const Slic3r::ModelObject* obj) { return !obj->instances.empty(); });
            if (placed)
                new_object->add_instance();
            model.clear_objects();
            model.add_object(*new_object);
        } else if (opt_key == "repetitions") {
            // BambuStudio.cpp 4953-4975; OrcaSlicer.cpp 4214-4236.
            const int repetitions_count = cfg.opt_int("repetitions");
            if (repetitions_count > 1) {
                if (plate_to_slice == 0) {
                    r.code    = CLI_INVALID_PARAMS;
                    r.message = "--repetitions needs one plate: use it with --slice N, not --slice 0";
                    return r;
                }
                if (plate_to_slice > plate_count) {
                    r.code    = CLI_INVALID_PARAMS;
                    r.message = "--slice " + std::to_string(plate_to_slice) + " but the file has " +
                                std::to_string(plate_count) + " plate(s)";
                    return r;
                }
                duplicate_count = repetitions_count - 1;
            }
        } else if (opt_key == "convert_unit") {
            // BambuStudio.cpp 4976-4987; OrcaSlicer.cpp 4237-4248.
            if (model.looks_like_saved_in_meters()) {
                model.convert_from_meters(true);
                emit({{"event", "model_loaded"}, {"tag", "ConvertedFromMeters"},
                      {"message", "--convert-unit: the model looked like meters and was scaled to millimeters"}});
            } else if (model.looks_like_imperial_units()) {
                model.convert_from_imperial_units(true);
                emit({{"event", "model_loaded"}, {"tag", "ConvertedFromInches"},
                      {"message", "--convert-unit: the model looked like inches and was scaled to millimeters"}});
            }
        } else if (opt_key == "orient") {
            // 0 = never, 1 = every object, other = as before
            // (BambuStudio.cpp 4988-5018; OrcaSlicer.cpp 4249-4279).
            const int orient_option = cfg.opt_int("orient");
            if (orient_option == 0 || orient_option == 1)
                for (Slic3r::ModelObject* obj : model.objects)
                    orients_requirement.emplace(obj->id().id, orient_option == 1);
        } else if (opt_key == "arrange") {
            // Read where the arrange runs (cli_slice_plate.cpp, CliOptions::arrange).
        } else if (opt_key == "ensure_on_bed") {
            // The value is used after the arrange (ensure_on_bed_if_asked).
        } else if (opt_key == "rotate" || opt_key == "rotate_x" || opt_key == "rotate_y") {
            // BambuStudio.cpp 5084-5098; OrcaSlicer.cpp 4345-4359.
            const Slic3r::Axis axis = opt_key == "rotate" ? Slic3r::Z : (opt_key == "rotate_x" ? Slic3r::X : Slic3r::Y);
            for (Slic3r::ModelObject* obj : model.objects)
                obj->rotate(Slic3r::Geometry::deg2rad(cfg.opt_float(opt_key)), axis);
        } else if (opt_key == "scale") {
            // BambuStudio.cpp 5099-5109; OrcaSlicer.cpp 4360-4370.
            const float ratio = float(cfg.opt_float(opt_key));
            if (ratio <= 0.f) {
                r.code    = CLI_INVALID_PARAMS;
                r.message = "--scale must be more than 0";
                return r;
            }
            for (Slic3r::ModelObject* obj : model.objects)
                obj->scale(ratio);
#if defined(ENGINE_ORCA) && !defined(ENGINE_SNAPMAKER)
        } else if (opt_key == "repair") {
            // OrcaSlicer from 2.5 (OrcaSlicer.cpp 5267-5280 at f92a7c0): every
            // volume's mesh is repaired as the app's Fix Model does; a mesh
            // that cannot be made solid is left as it is.
            if (cfg.opt_bool(opt_key))
                for (Slic3r::ModelObject* obj : model.objects) {
                    for (Slic3r::ModelVolume* volume : obj->volumes) {
                        Slic3r::MeshRepairStats stats;
                        std::string error;
                        if (!volume->repair(true, &stats, &error))
                            emit({{"event", "config_normalized"}, {"tag", "MeshRepairFailed"}, {"object", obj->name},
                                  {"volume", volume->name},
                                  {"message", "--repair: '" + obj->name + "' / '" + volume->name +
                                                  "' could not be repaired and is left as it is: " + error}});
                        else if (stats.modified)
                            emit({{"event", "model_loaded"}, {"tag", "MeshRepaired"}, {"object", obj->name},
                                  {"volume", volume->name},
                                  {"welded_vertices", stats.welded_vertices}, {"removed_triangles", stats.removed_triangles},
                                  {"reversed_shells", stats.reversed_shells}, {"filled_holes", stats.filled_holes},
                                  {"removed_shells", stats.removed_shells}, {"self_intersections", stats.self_intersections},
                                  {"message", "--repair: '" + obj->name + "' / '" + volume->name + "' was repaired"}});
                    }
                    obj->invalidate_bounding_box();
                }
        } else if (opt_key == "ground_largest_face" || opt_key == "ground_face_normal" || opt_key == "ground_face_point") {
            // OrcaSlicer from 2.5 (OrcaSlicer.cpp 5105-5160 at f92a7c0): each
            // instance is laid on one of its lay-on-face planes, the faces the
            // desktop's "Lay on Face" tool offers. The planes follow the part
            // transformations as they are now, so rotations given before this
            // flag count. A direction or a point is in object coordinates, so
            // it names the same face for every instance of an object.
            const std::string flag = "--" + std::string(opt_key == "ground_largest_face" ? "ground-largest-face"
                                            : opt_key == "ground_face_normal" ? "ground-face-normal" : "ground-face-point");
            std::function<int(const std::vector<Slic3r::LayOnFacePlane>&, const Slic3r::Transform3d&)> pick;
            if (opt_key == "ground_largest_face") {
                if (cfg.opt_bool(opt_key))
                    pick = [](const std::vector<Slic3r::LayOnFacePlane>& planes, const Slic3r::Transform3d&) {
                        return Slic3r::find_largest_plane(planes);
                    };
            } else {
                const std::string& value = cfg.opt_string(opt_key);
                Slic3r::Vec3d v;
                int consumed = 0;
                if (sscanf(value.c_str(), "%lf,%lf,%lf%n", &v.x(), &v.y(), &v.z(), &consumed) != 3 ||
                    consumed != int(value.size()) || !v.allFinite() ||
                    (opt_key == "ground_face_normal" && v.norm() < EPSILON)) {
                    r.code    = CLI_INVALID_PARAMS;
                    r.message = flag + " expects three comma-separated numbers, got \"" + value + "\"";
                    return r;
                }
                if (opt_key == "ground_face_normal")
                    pick = [v](const std::vector<Slic3r::LayOnFacePlane>& planes, const Slic3r::Transform3d&) {
                        return Slic3r::find_plane_by_normal(planes, v);
                    };
                else
                    pick = [v](const std::vector<Slic3r::LayOnFacePlane>& planes, const Slic3r::Transform3d& inst_matrix) {
                        return Slic3r::find_plane_at_point(planes, inst_matrix, v, 0.01);
                    };
            }
            if (pick) {
                size_t laid = 0, missed = 0;
                // The official loads model files with their default instance and
                // turns that. Here a model file gets its instance from the desktop
                // placement after the transforms (see the orient step below), so an
                // object without one is turned itself, by the same rotation an
                // identity instance would get; the placement then puts it on the bed.
                for (Slic3r::ModelObject* obj : model.objects) {
                    const bool no_instance = obj->instances.empty();
                    for (size_t i = 0; i < (no_instance ? size_t(1) : obj->instances.size()); ++i) {
                        const Slic3r::Transform3d inst_matrix =
                            no_instance ? Slic3r::Transform3d::Identity() : obj->instances[i]->get_matrix_no_offset();
                        const std::vector<Slic3r::LayOnFacePlane> planes = Slic3r::lay_on_face_planes(*obj, inst_matrix);
                        if (planes.empty()) {
                            // A small or smooth part (a sphere) has no face to rest on; the tool offers none either.
                            emit({{"event", "config_normalized"}, {"tag", "NoFaceToLayOn"}, {"object", obj->name},
                                  {"message", flag + ": '" + obj->name + "' has no face large enough to lay on; it is left as it is"}});
                            continue;
                        }
                        const int idx = pick(planes, inst_matrix);
                        if (idx < 0) {
                            // Only a point can miss: with several objects it usually belongs to one of them.
                            emit({{"event", "config_normalized"}, {"tag", "NoFaceAtPoint"}, {"object", obj->name},
                                  {"message", flag + ": no face of '" + obj->name + "' contains the point; it is left as it is"}});
                            ++missed;
                            continue;
                        }
                        if (no_instance) {
                            const Eigen::AngleAxisd turn(Slic3r::lay_on_face_rotation(planes[idx].normal, inst_matrix).linear());
                            obj->rotate(turn.angle(), turn.axis());
                        } else {
                            Slic3r::lay_on_face(*obj, i, planes[idx].normal);
                        }
                        emit({{"event", "model_loaded"}, {"tag", "ObjectLaidOnFace"}, {"object", obj->name},
                              {"instance", int(i)}, {"face_area_mm2", planes[idx].area},
                              {"message", flag + ": '" + obj->name + "' is laid on a face of " +
                                              std::to_string(int(planes[idx].area + 0.5f)) + " mm2"}});
                        ++laid;
                    }
                }
                if (laid == 0 && missed > 0) {
                    r.code    = CLI_INVALID_PARAMS;
                    r.message = flag + ": no face of any object contains the point";
                    return r;
                }
            }
#endif
        } else {
            // BambuStudio.cpp 5203-5207.
            r.code    = CLI_UNSUPPORTED_OPERATION;
            r.message = "--" + opt_key + " is not implemented; no flag of this binary does this, drop it";
            return r;
        }
    }
    // Orient the objects that asked for it (BambuStudio.cpp 5213-5234;
    // OrcaSlicer.cpp 4474-4495), each with the loaded process settings.
    for (Slic3r::ModelObject* obj : model.objects) {
        const auto it = orients_requirement.find(obj->id().id);
        if (it == orients_requirement.end() || !it->second)
            continue;
#ifndef ENGINE_ORCA
        if (process_config && !process_config->empty())
            obj->config.assign_config(*process_config);
#else
        (void)process_config;
#endif
        // The official loads model files with their default instance
        // (read_from_file(..., AddDefaultInstances), BambuStudio.cpp 1880-1889;
        // OrcaSlicer.cpp 1562-1571), so the object has one when it is
        // oriented: orient() measures ModelObject::mesh(), the sum of the
        // object's instances (Model.cpp 1611-1621), which is empty without
        // one and crashes in AutoOrienter::get_features
        // (z_projected.minCoeff() of an empty matrix, Orient.cpp 388). Here
        // model files get their instance from the desktop placement after
        // the transforms (desktop_place_on_bed), so the orient gets the same
        // identity instance for its duration only.
        const bool temporary_instance = obj->instances.empty();
        if (temporary_instance)
            obj->add_instance();
        Slic3r::orientation::orient(obj);
        if (temporary_instance)
            obj->clear_instances();
        emit({{"event", "model_loaded"}, {"tag", "ObjectOriented"}, {"object", obj->name},
              {"message", "--orient: '" + obj->name + "' was turned to its best printing orientation"}});
    }
    return r;
}

void ensure_on_bed_if_asked(const CliOptions& o, Slic3r::Model& model) {
    if (o.given_flag("ensure_on_bed") && o.cli.opt_bool("ensure_on_bed"))
        for (Slic3r::ModelObject* obj : model.objects)
            obj->ensure_on_bed();
}

namespace {

/// Whether the action at `index` of o.actions belongs to `phase`: before or
/// after --slice in command-line order (no --slice: all before it).
bool in_phase(const CliOptions& o, size_t index, ActionPhase phase) {
    if (phase == ActionPhase::All)
        return true;
    const auto slice = std::find(o.actions.begin(), o.actions.end(), std::string("slice"));
    const size_t slice_index = slice == o.actions.end() ? o.actions.size() : size_t(slice - o.actions.begin());
    return phase == ActionPhase::BeforeSlice ? index < slice_index : index > slice_index;
}

bool is_model_action(const std::string& a) {
    return a == "info" || a == "export_settings" || a == "export_stl" || a == "export_stls";
}

bool is_model_wide_action(const std::string& a) {
    return a == "info" || a == "export_stl" || a == "export_stls";
}

} // namespace

bool has_model_actions(const CliOptions& o) {
    return has_model_actions(o, ActionPhase::All);
}

bool has_model_actions(const CliOptions& o, ActionPhase phase) {
    for (size_t i = 0; i < o.actions.size(); ++i)
        if (is_model_action(o.actions[i]) && in_phase(o, i, phase))
            return true;
    return false;
}

bool has_model_wide_actions(const CliOptions& o, ActionPhase phase) {
    for (size_t i = 0; i < o.actions.size(); ++i)
        if (is_model_wide_action(o.actions[i]) && in_phase(o, i, phase))
            return true;
    return false;
}

bool model_actions_only(const CliOptions& o) {
    return has_model_actions(o) && !o.given_flag("slice");
}

StepResult run_model_actions(const CliOptions& o, Slic3r::Model& plate_model, const Slic3r::DynamicPrintConfig& config,
                             Slic3r::Model* whole, ActionPhase phase) {
    StepResult r;
    Slic3r::Model& model = whole ? *whole : plate_model;
    for (size_t index = 0; index < o.actions.size(); ++index) {
        const std::string& opt_key = o.actions[index];
        if (!in_phase(o, index, phase))
            continue;
        if (opt_key == "export_settings") {
            // BambuStudio.cpp 6366-6370; OrcaSlicer.cpp 5499-5503.
            const std::string file = o.cli.opt_string("export_settings");
#if defined(ENGINE_ORCA) && !defined(ENGINE_SNAPMAKER)
            // OrcaSlicer from 2.5 writes the settings to stdout for "-"
            // (OrcaSlicer.cpp 1485 and 6346 at f92a7c0). This binary prints
            // its events and progress on stdout, so it asks for a file.
            if (file == "-") {
                r.code    = CLI_INVALID_PARAMS;
                r.message = "--export-settings - writes the settings to stdout in OrcaSlicer's own command line. This "
                            "binary prints its events on stdout: give a file name, and read the file.";
                return r;
            }
#endif
            config.save_to_json(file, "project_settings", "project", engine_version());
            emit({{"event", "export"}, {"tag", "SettingsExported"}, {"path", file},
                  {"message", "Exported the settings as " + file}});
        } else if (opt_key == "info") {
            // BambuStudio.cpp 6371-6376; OrcaSlicer.cpp 5504-5509.
            model.add_default_instances();
            model.print_info();
        } else if (opt_key == "export_stl" || opt_key == "export_stls") {
            // BambuStudio.cpp 6387-6401; OrcaSlicer.cpp 5520-5534.
            model.add_default_instances();
            std::string failed;
            const std::string dir = opt_key == "export_stls" ? o.cli.opt_string("export_stls") : std::string();
            if (!export_stls(o, model, dir, failed)) {
                r.code    = CLI_EXPORT_STL_ERROR;
                r.message = "Writing " + failed + " failed";
                return r;
            }
        }
    }
    return r;
}

PlateLoopSwitches plate_loop_switches(const CliOptions& o) {
    PlateLoopSwitches s;
    const auto& cfg = o.cli;
    if (o.given_flag("mtcpp"))  s.max_triangle_count = cfg.opt_int("mtcpp");
    if (o.given_flag("mstpp"))  s.max_slicing_time_s = cfg.opt_int("mstpp");
    if (o.given_flag("no_check")) s.no_check = cfg.opt_bool("no_check");
    if (o.given_flag("strict")) s.strict = cfg.opt_bool("strict");
    if (o.given_flag("allow_mix_temp")) s.allow_mix_temp = cfg.opt_bool("allow_mix_temp");
#ifdef ENGINE_SNAPMAKER
    // This engine has no allow_mix_temp setting, so the switch is our own flag.
    if (o.allow_mix_temp) s.allow_mix_temp = true;
#endif
    if (o.given_flag("normative_check")) s.normative_check = cfg.opt_bool("normative_check");
    if (o.given_flag("min_save")) s.min_save = cfg.opt_bool("min_save");
    return s;
}

StepResult normative_check(const CliOptions& o, const Slic3r::DynamicPrintConfig& file_config) {
    StepResult r;
    if (!plate_loop_switches(o).normative_check)
        return r;
    // Any post_process entry (values.size() > 0, BambuStudio.cpp 1951-1956;
    // OrcaSlicer.cpp 1832-1839).
    if (const auto* scripts = file_config.option<Slic3r::ConfigOptionStrings>("post_process");
        scripts && !scripts->values.empty()) {
        r.code    = CLI_POSTPROCESS_NOT_SUPPORTED;
        r.message = "The file names post-processing scripts, which the command line does not run; "
                    "clear post_process in the file, or slice it in the desktop app";
        return r;
    }
    // A mixed filament is sliced: the official check for it went with Bambu
    // Studio 2.9.1, whose normative check is the scripts alone
    // (BambuStudio.cpp 2130-2140 at v02.09.01.70).
    return r;
}

int apply_skip_objects(const CliOptions& o, Slic3r::Model& model, std::vector<int>& skipped_ids) {
    const auto& skip = o.cli.option<Slic3r::ConfigOptionInts>("skip_objects")->values;
    if (skip.empty())
        return 0;
    const std::set<int> wanted(skip.begin(), skip.end());
    int skipped = 0, printable = 0;
    for (Slic3r::ModelObject* obj : model.objects)
        for (Slic3r::ModelInstance* inst : obj->instances) {
            inst->use_loaded_id_for_label = true;
            if (wanted.count(inst->loaded_id)) {
                inst->printable = false;
                ++skipped;
                skipped_ids.push_back(inst->loaded_id);
            } else if (inst->printable) {
                ++printable;
            }
        }
    return printable == 0 ? -1 : skipped;
}

StepResult check_triangle_limit(const CliOptions& o, const Slic3r::Model& model, int plate_id) {
    StepResult r;
    const int limit = plate_loop_switches(o).max_triangle_count;
    if (limit == 0)
        return r;
    // Every instance inside the bed counts its parts' triangles, one on the
    // --skip-objects list none (BambuStudio.cpp 6541-6597; OrcaSlicer.cpp
    // 5657-5720). An instance the user made unprintable still counts.
    const std::vector<int>& skip = o.cli.option<Slic3r::ConfigOptionInts>("skip_objects")->values;
    const std::set<int> skipped(skip.begin(), skip.end());
    long long count = 0;
    for (const Slic3r::ModelObject* obj : model.objects) {
        for (const Slic3r::ModelInstance* inst : obj->instances) {
            if (skipped.count(inst->loaded_id) || inst->print_volume_state != Slic3r::ModelInstancePVS_Inside)
                continue;
            for (const Slic3r::ModelVolume* vol : obj->volumes) {
                if (!vol->is_model_part())
                    continue;
                count += (long long)vol->mesh().facets_count();
                if (count > limit) {
                    r.code    = CLI_TRIANGLE_COUNT_EXCEEDS_LIMIT;
                    r.message = "Plate " + std::to_string(plate_id) + " has " + std::to_string(count) +
                                " triangles, more than the --mtcpp limit of " + std::to_string(limit) +
                                "; raise --mtcpp above " + std::to_string(count) + ", or split the plate";
                    return r;
                }
            }
        }
    }
    return r;
}

std::string slicedata_dir(const CliOptions& o, const char* key, int plate) {
    if (!o.given_flag(key))
        return {};
    // load_slicedata is declared a list of strings with a single-string
    // default (PrintConfig.cpp 9508-9512 at 5873b5f; 10364-10368 at 31f6803),
    // so the stored value is a string; a list is read by its first entry.
    std::string dir;
    const Slic3r::ConfigOption* opt = o.cli.option(key);
    if (const auto* list = dynamic_cast<const Slic3r::ConfigOptionStrings*>(opt))
        dir = list->values.empty() ? std::string() : list->values.front();
    else if (const auto* one = dynamic_cast<const Slic3r::ConfigOptionString*>(opt))
        dir = one->value;
    return dir.empty() ? std::string() : dir + "/" + std::to_string(plate);
}

StepResult check_slicedata_flags(const CliOptions& o, int duplicate_count) {
    StepResult r;
    if (!o.given_flag("load_slicedata"))
        return r;
#ifdef ENGINE_ORCA
    if (o.given_flag("export_slicedata")) {
        r.code    = CLI_INVALID_PARAMS;
        r.message = "--load-slicedata and --export-slicedata cannot be used together; "
                    "give one of them: load a saved slicing, or write one";
        return r;
    }
#endif
    if (duplicate_count > 0) {
        r.code    = CLI_INVALID_PARAMS;
        r.message = "--load-slicedata cannot be used with --repetitions; "
                    "drop --repetitions, or slice without --load-slicedata";
    }
    return r;
}

void apply_model_metadata(const CliOptions& o, Slic3r::Model& model) {
    const auto& cfg = o.cli;
    const std::string mk_name = cfg.opt_string("makerlab_name");
    if (o.given_flag("makerlab_name") && !mk_name.empty()) {
        model.mk_name    = mk_name;
        model.mk_version = cfg.opt_string("makerlab_version");
    }
    const auto& names  = cfg.option<Slic3r::ConfigOptionStrings>("metadata_name")->values;
    const auto& values = cfg.option<Slic3r::ConfigOptionStrings>("metadata_value")->values;
    if (!names.empty()) {
        model.md_name  = names;
        model.md_value = values;
    }
}

std::string object_stl_file(const CliOptions& o, const Slic3r::ModelObject& object, unsigned index,
                            const std::string& path_dir) {
    const std::string ext = ".stl";
    std::string subdir = "stl";
    std::string file_name = object.name.empty() ? object.input_file : object.name;
    file_name = "obj_" + std::to_string(index) + "_" + file_name;
    const size_t pos = file_name.rfind(ext);
    const size_t ext_pos = file_name.size() >= ext.size() ? file_name.size() - ext.size() : std::string::npos;
    if (pos == std::string::npos || pos != ext_pos)
        file_name += ext;
    if (path_dir.empty()) {
        if (!o.outputdir.empty())
            subdir = o.outputdir + "/" + subdir;
    } else {
        subdir = path_dir;
    }
    return subdir + "/" + file_name;
}

} // namespace slicer_cli
