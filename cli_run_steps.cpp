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
/// "<outputdir>/stl", or "stl". The folder is created if missing.
std::string object_stl_path(const CliOptions& o, const Slic3r::ModelObject& object, unsigned index,
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
    // The official creates the last folder only (create_directory), and
    // throws when its parent is missing; the whole path is made here.
    boost::system::error_code ec;
    if (!boost::filesystem::exists(subdir, ec))
        boost::filesystem::create_directories(subdir, ec);
    return subdir + "/" + file_name;
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
        const std::string part = path + ".writing";
        boost::filesystem::remove(part, ec);
        ec.clear();
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
            // Read where the arrange runs (main.cpp, CliOptions::arrange).
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
    if (o.given_flag("allow_mix_temp")) s.allow_mix_temp = cfg.opt_bool("allow_mix_temp");
    if (o.given_flag("normative_check")) s.normative_check = cfg.opt_bool("normative_check");
    if (o.given_flag("min_save")) s.min_save = cfg.opt_bool("min_save");
    return s;
}

StepResult normative_check(const CliOptions& o, const Slic3r::DynamicPrintConfig& file_config) {
    StepResult r;
    if (!plate_loop_switches(o).normative_check)
        return r;
    // Any post_process entry (values.size() > 0, BambuStudio.cpp 1951-1956;
    // OrcaSlicer.cpp 1610-1617).
    if (const auto* scripts = file_config.option<Slic3r::ConfigOptionStrings>("post_process");
        scripts && !scripts->values.empty()) {
        r.code    = CLI_POSTPROCESS_NOT_SUPPORTED;
        r.message = "The file names post-processing scripts, which the command line does not run; "
                    "clear post_process in the file, or slice it in the desktop app";
        return r;
    }
#ifndef ENGINE_ORCA
    // BambuStudio.cpp 1959-1964 (the OrcaSlicer CLI has no such check).
    if (const auto* mixed = file_config.option<Slic3r::ConfigOptionBools>("filament_is_mixed")) {
        if (std::any_of(mixed->values.begin(), mixed->values.end(), [](unsigned char v) { return v != 0; })) {
            r.code    = CLI_3MF_FEATURE_NOT_SUPPORTED;
            r.message = "The file uses a mixed filament, which the command line does not slice; "
                        "give the project one filament per extruder, or slice it in the desktop app";
            return r;
        }
    }
#endif
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

} // namespace slicer_cli
