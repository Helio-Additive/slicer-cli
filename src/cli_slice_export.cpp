#include "cli_slice_export.hpp"

#include "cli_run_context.hpp"
#include "cli_slice_state.hpp"
#include "cli_result_json.hpp"
#include "cli_options.hpp"
#include "cli_assemble.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/GCode.hpp"
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/algorithm/string.hpp>
#include <iostream>
#include "cli_events.hpp"
#include "cli_legacy_gcode.hpp"
#include "cli_model_load.hpp"
#include "cli_named_presets.hpp"
#include "cli_preset_staging.hpp"
#include "cli_project_archive.hpp"
#include "cli_run_cleanup.hpp"
#include "cli_run_steps.hpp"
#include "desktop_presets.hpp"

namespace slicer_cli {

/// --export-3mf: the sliced project, written with store_bbs_3mf as the
/// official CLI writes it: export_project(&m_models[0], plate_data_list,
/// project_presets, ..., &m_print_config, minimum_save, plate_to_slice - 1)
/// (BambuStudio.cpp 8156-8157, CLI::export_project 8468-8505 at 5873b5f:
/// Silence | WithGcode | SplitModel | UseLoadedId | ShareMesh; OrcaSlicer.cpp
/// 6985). The model is the run's one model as it was sliced: the transforms,
/// the arrange, the move to a new bed, --repetitions' copies and
/// --skip-objects' unprintable instances are in it. With --slice N it holds
/// plate N's objects only, as the official loads only that plate
/// (BambuStudio.cpp 1889). The plates are the file's (or the run's), each
/// listing its instances, and each sliced plate gets its G-code and slice
/// facts. No thumbnails are rendered (the CLI has no OpenGL), so a project
/// keeps the plate pictures it came with.
int export_sliced_3mf(CliRunContext& context, const CliOptions& o, const boost::filesystem::path& outdir, RunState& rs,
                             std::vector<PlateOutcome>& outcomes, const Slic3r::DynamicPrintConfig& project,
                             const std::set<std::string>* given_settings, std::string& error) {
    using namespace Slic3r;
    const std::string path = (outdir / o.export_3mf).string();
    Model& model = rs.project.model;
    // store_bbs_3mf stages the project's files in the model's backup folder
    // (_add_project_config_file_to_archive writes <backup>/_temp_1.config,
    // bbs_3mf.cpp). The run's folder (once slicer_cli_backup, shared by every
    // slicer_cli run on the host, and removed by a run that ends: ~Model,
    // Model.cpp remove_backup) was not the export's, so a run exporting
    // beside another lost its files. The export stages in a folder of its
    // own, as ba0dcfb's export model did. "detach" first: set_backup_path
    // removes the folder it replaces (Model::set_backup_path), which is the
    // load's own (run_backup_path).
    model.set_backup_path("detach");
    const std::string export_backup =
        context.run_staging.add((boost::filesystem::temp_directory_path() /
                         boost::filesystem::unique_path("slicer_cli_export-%%%%%%%%")).string());
    model.set_backup_path(export_backup);   // removed with the run's other staging folders
    // Detached again when this function ends, so this model's destruction does
    // not queue a removal of a folder the run removes itself
    // (detach_run_backup_path).
    const RunBackupDetach detach_export_model{model};
    PlateDataPtrs plates;
    struct Release {
        PlateDataPtrs& plates;
        ~Release() { release_PlateData_list(plates); }
    } release{plates};
    if (outcomes.empty() || !outcomes.front().plate_data) {
        error = "Nothing was sliced to export.";
        return CLI_EXPORT_3MF_ERROR;
    }
    const bool geometry_input = !input_is_3mf(o.input_file) ||
        slicer_cli::classify_3mf(o.input_file) == slicer_cli::ThreeMfKind::GeometryOnly;

    // The plates (PartPlateList::store_to_3mf_structure, BambuStudio.cpp
    // 7483-7484): the file's, with any the arrange added, or one plate for
    // model files and a 3MF without plates.
    const int count = std::max<int>({1, (int)rs.layout.plates.size(), (int)rs.project.plate_data.size()});
    for (int k = 0; k < count; ++k) {
        PlateData* pd = k < (int)rs.project.plate_data.size() && rs.project.plate_data[k] != nullptr ? new PlateData(*rs.project.plate_data[k])
                                                                                    : new PlateData();
        pd->plate_index = k;
        pd->objects_and_instances.clear();
        plates.push_back(pd);
    }
    // Each plate's instances (PartPlate::obj_to_instance_set, which the
    // writer reads as objects_and_instances, bbs_3mf.cpp 8208, 8328-8350).
    for (size_t oi = 0; oi < model.objects.size(); ++oi)
        for (size_t ii = 0; ii < model.objects[oi]->instances.size(); ++ii) {
            const ModelInstance* inst = model.objects[oi]->instances[ii];
            for (int k = 0; k < count; ++k)
                if (rs.layout.plate(k + 1).is_member(inst) && (k < (int)rs.layout.plates.size() || count == 1)) {
                    plates[k]->objects_and_instances.emplace_back(int(oi), int(ii));
                    break;
                }
        }
    // Each sliced plate's slice result (BambuStudio.cpp 7483-7532).
    for (const PlateOutcome& out : outcomes) {
        if (!out.plate_data) continue;
        const int idx = geometry_input && !context.g_assemble ? 0 : out.plate_id - 1;
        if (idx < 0 || idx >= count) continue;
        PlateData* pd = plates[idx];
        const PlateData& sliced = *out.plate_data;
        pd->gcode_file               = sliced.gcode_file;
        pd->is_sliced_valid          = true;
        pd->gcode_prediction         = sliced.gcode_prediction;
        pd->gcode_weight             = sliced.gcode_weight;
        pd->toolpath_outside         = sliced.toolpath_outside;
        pd->timelapse_warning_code   = sliced.timelapse_warning_code;
        pd->is_label_object_enabled  = sliced.is_label_object_enabled;
#ifndef ENGINE_SNAPMAKER
        // Not in Snapmaker's PlateData at bac6fd5 (see record_plate_for_export).
        pd->limit_filament_maps      = sliced.limit_filament_maps;
        pd->layer_filaments          = sliced.layer_filaments;
        pd->filament_change_sequence = sliced.filament_change_sequence;
        pd->nozzle_change_sequence   = sliced.nozzle_change_sequence;
        pd->optimal_assignment       = sliced.optimal_assignment;
        pd->filament_maps            = sliced.filament_maps;
#endif
        pd->is_support_used          = sliced.is_support_used;
        pd->slice_filaments_info     = sliced.slice_filaments_info;
        pd->skipped_objects          = sliced.skipped_objects;
        pd->printer_model_id         = sliced.printer_model_id;
        pd->nozzle_diameters         = sliced.nozzle_diameters;
#ifndef ENGINE_SNAPMAKER
        // The routing the slice derived from a supplied nozzle map, or the
        // command line's --filament-map, on this plate only: store_bbs_3mf writes a plate's filament_map_mode and
        // filament_maps from its config (bbs_3mf.cpp 8260-8279 at 5873b5f;
        // PartPlate::set_filament_maps keeps the map there, PartPlate.cpp
        // 3860-3863), and a plate mode other than Default wins over the
        // project's (PartPlate.cpp 278-289). The objects keep their own
        // filaments, so an object with copies on other plates stays as it is.
        if (!out.derived_filament_map.empty())
            pd->config.option<ConfigOptionInts>("filament_map", true)->values = out.derived_filament_map;
        if (!out.derived_filament_map_mode.empty()) {
            ConfigSubstitutionContext mode_subst(ForwardCompatibilitySubstitutionRule::Enable);
            pd->config.set_deserialize("filament_map_mode", out.derived_filament_map_mode, mode_subst);
        }
        for (const auto& [key, values] : out.flag_plate_maps)
            pd->config.option<ConfigOptionInts>(key, true)->values = values;
#endif
    }
    // printer_model_id per plate after a printer change (BambuStudio.cpp 7524-7525).
    if (!rs.settings.settings_merge.printer_model_id.empty())
        for (PlateData* pd : plates)
            pd->printer_model_id = rs.settings.settings_merge.printer_model_id;
    if (context.g_assemble)
        for (int k = 0; k < count && k < (int)context.g_assemble->plates.size(); ++k)
            plates[k]->plate_name = context.g_assemble->plates[k].plate_name;

    // The presets the project carries, and the "(auto)" process a printer
    // change makes (BambuStudio.cpp 3094).
    std::vector<Preset*> project_presets;
    for (Preset& p : rs.project.project_presets_kept)
        project_presets.push_back(&p);
    std::unique_ptr<Preset> auto_process;
    if (rs.settings.settings_merge.new_preset) {
        auto_process = std::make_unique<Preset>(*rs.settings.settings_merge.new_preset);
        project_presets.push_back(auto_process.get());
    }

    // The project's settings are the run's, m_print_config (the official
    // export_project(..., &m_print_config, ...), BambuStudio.cpp 8156-8157;
    // OrcaSlicer.cpp 6985), the ones --export-settings writes and
    // result.json reads, with the towers the arrange placed. A plate's own
    // settings stay the plate's (its PlateData config). The G-code header
    // reads the plate's copy, which also holds this command line's own
    // per-plate adjustments (the prime tower turned off when every filament
    // is the same, the per-filament lists filled to the filament count on the
    // Bambu build): those keys can differ from the project's, as the official
    // app never writes them.
    DynamicPrintConfig config = project;
    // The slice ran its custom G-code with the legacy placeholder aliased
    // (normalize_legacy_gcode_tokens, before print.apply); the project
    // states the same templates. The slice already reported the alias.
    if (!geometry_input && !context.g_assemble && o.normalize_legacy_gcode)
        normalize_legacy_gcode_tokens(config, /*report_event=*/false);
    // The settings this run changed, listed as the desktop lists them when it
    // saves (desktop_presets.hpp): a project is opened over the presets and
    // keeps only what is listed. A project whose presets the package does not
    // ship is opened with its saved values, so its list stays as it is.
    {
        std::string unavailable;
        try {
            const boost::filesystem::path staging = make_owned_temp_dir("slicer_cli_project-");
            struct StagingCleanup {
                boost::filesystem::path dir;
                ~StagingCleanup() { remove_owned_temp_dir(dir); }
            } staging_cleanup{staging};
            PresetBundle bundle;
            if (load_system_presets(o.argv0, bundle, staging, unavailable)) {
#ifdef ENGINE_BAMBU
                slicer_cli::load_project_embedded_presets(bundle, project_presets, config);
#endif
                if (project_presets_shipped(bundle, config_id_text(config, "printer_settings_id"),
                                            config_id_text(config, "print_settings_id")))
                    slicer_cli::record_changed_settings(bundle, path, config, given_settings, unavailable);
            }
        } catch (const std::exception& e) {
            unavailable = e.what();
        }
        if (!unavailable.empty())
            emit_event({{"event","export_note"}, {"tag","ChangedSettingsNotListed"}, {"path", path},
                        {"reason", unavailable},
                        {"message","The project does not list the settings this run changed: " + unavailable}});
    }

    // Each sliced plate's Metadata/plate_N.json; an empty box for the others
    // (BambuStudio.cpp 7976-7988).
    std::vector<std::unique_ptr<PlateBBoxData>> bbox_storage;
    std::vector<PlateBBoxData*> plate_bboxes;
    for (size_t i = 0; i < plates.size(); ++i) {
        PlateBBoxData* found = nullptr;
        for (const PlateOutcome& out : outcomes)
            if (out.plate_bbox && (geometry_input && !context.g_assemble ? i == 0 : out.plate_id - 1 == int(i)))
                found = out.plate_bbox.get();
        bbox_storage.push_back(std::make_unique<PlateBBoxData>(found ? *found : PlateBBoxData()));
        plate_bboxes.push_back(bbox_storage.back().get());
    }
    // --makerlab-*, --metadata-* (BambuStudio.cpp 8137-8154).
    slicer_cli::apply_model_metadata(o, model);

    StoreParams store;
    store.path = path.c_str();
    store.model = &model;
    store.plate_data_list = plates;
    store.project_presets = project_presets;
    store.config = &config;
    store.id_bboxes = plate_bboxes;
    store.strategy = SaveStrategy::Silence | SaveStrategy::WithGcode | SaveStrategy::SplitModel |
                     SaveStrategy::UseLoadedId | SaveStrategy::ShareMesh;
    // --min-save: the project without its model parts (BambuStudio.cpp 8493-8494).
    if (o.given_flag("min_save") && o.cli.opt_bool("min_save"))
        store.strategy = store.strategy | SaveStrategy::SkipModel;
    store.export_plate_idx = o.slice_plate - 1;
    if (!store_bbs_3mf(store)) {
        error = "Writing " + path + " failed.";
        return CLI_EXPORT_3MF_ERROR;
    }
    emit_event({{"event","exported_3mf"}, {"tag","SlicedProjectWritten"}, {"path", path},
                {"message","Wrote the sliced project " + path}});
    // The official CLI renders plate pictures with its GUI's OpenGL code;
    // slicer-cli makes none, so the project has no plate pictures. Said
    // once, here, rather than left for a reader to find missing.
    emit_event({{"event","export_note"}, {"tag","NoPlatePictures"}, {"path", path},
                {"message","The project has no plate pictures: slicer-cli does not render them"}});
    return 0;
}

/// Every file this run writes, checked before the run writes any of them,
/// whatever flags are given: no two of them may be one file (the one written
/// last would replace the other, and the run would report success with a
/// requested output lost), and none may be a file the run reads.
///
/// The outputs: result.json and each sliced plate's plate_<n>.gcode in
/// --outputdir (--slice), the G-code of -o (the single-plate call),
/// --export-3mf (joined onto --outputdir, as the official joins it:
/// export_3mf_file = outfile_dir + "/" + export_3mf_file, BambuStudio.cpp
/// 7508-7510 at 5873b5f; OrcaSlicer.cpp 6302-6304 at 31f6803), the
/// --export-settings file, the object STL files of --export-stl /
/// --export-stls (each object's own name, object_stl_file, read from the
/// loaded `model`) and the plate folders under --export-slicedata.
///
/// It runs twice in a --slice run: once the model is loaded and before the
/// first action writes a file (every output but the plates', whose plan may
/// not be final yet; `plates` holds the ones already known), and again once
/// the plate plan is final, before the first plate is sliced, with every
/// plate. A run without --slice checks once, before its actions and its -o
/// G-code.
///
/// Two outputs are compared by name, the link chain followed and without
/// case on every system (they may not exist yet; a name that differs from
/// another only by letter case is refused too, as an odd combination), and
/// two existing names for one file by fs::equivalent. An output overwrites an
/// input only when it is that file, and the inputs exist, so that check asks
/// the file system (fs::equivalent: links, hard links and the volume's own
/// case rule); an output that does not exist overwrites nothing.
///
/// Returns 0, or CLI_INVALID_PARAMS once the refusal is reported (-2:
/// ExportOverwritesInput for an input, ExportNameTaken for two outputs). With
/// `detail` the sentence is handed back for the caller's result; without it,
/// result.json is written here.
///
/// `reserved` holds outputs already written by actions given before --slice
/// (the check the calibration pattern runs after it, once the model is the
/// pattern): their names are taken, so every output still to come is called
/// out against them, and they are not checked or run again. `reserve` hands
/// the outputs of those earlier actions back for that second check.
int check_run_outputs(CliRunContext& context, const CliOptions& o, const std::vector<int>& plates, const Slic3r::Model* model,
                             std::string* detail, bool after_slice_actions_only,
                             const std::vector<RunOutput>* reserved,
                             std::vector<RunOutput>* reserve) {
    namespace fs = boost::filesystem;
    const fs::path outdir = o.outputdir.empty() ? fs::path(".") : fs::path(o.outputdir);
    const auto key = [](const fs::path& p) {
        // A link whose target does not exist yet (result.json before the run
        // writes it) is not resolved by weakly_canonical: follow the chain by
        // hand first.
        fs::path q = fs::absolute(p);
        boost::system::error_code ec;
        for (int hops = 0; hops < 40 && fs::is_symlink(q, ec) && !ec; ++hops) {
            const fs::path link = fs::read_symlink(q, ec);
            if (ec) break;
            q = link.is_absolute() ? link : q.parent_path() / link;
        }
        ec.clear();
        const fs::path resolved = fs::weakly_canonical(q, ec);
        return (ec ? q : resolved).lexically_normal().generic_string();
    };
    const auto lower = [](const std::string& s) { return boost::algorithm::to_lower_copy(s); };

    // ── What the run writes ─────────────────────────────────────────────
    std::vector<RunOutput> outputs;
    const auto add = [&](bool folder, bool stl, const std::string& flag, const fs::path& path,
                         const std::string& what, const std::string& is_what, bool before_slice = false) {
        RunOutput out;
        out.folder  = folder;
        out.stl     = stl;
        out.flag    = flag;
        out.path    = path;
        out.what    = what;
        out.is_what = is_what;
        outputs.push_back(std::move(out));
        // An action given before --slice writes its files before the slice:
        // the caller keeps them as reserved for the check the calibration
        // pattern runs after it (see the second check in slice_one_plate).
        if (before_slice && reserve != nullptr)
            reserve->push_back(outputs.back());
    };
    // The outputs a flag names come first: a clash is told from that flag.
    if (!o.export_3mf.empty() && o.slice_mode)
        add(false, false, "--export-3mf " + o.export_3mf, outdir / o.export_3mf,
            "the project --export-3mf " + o.export_3mf + " writes", "the project --export-3mf " + o.export_3mf + " writes");
    // With after_slice_actions_only, the actions given before --slice have
    // run already (their files are written): only those after it are still
    // to write.
    const size_t slice_index = size_t(std::find(o.actions.begin(), o.actions.end(), std::string("slice")) -
                                      o.actions.begin());
    for (size_t action_index = 0; action_index < o.actions.size(); ++action_index) {
        const std::string& action = o.actions[action_index];
        if (after_slice_actions_only && !(slice_index < o.actions.size() && action_index > slice_index))
            continue;
        const bool before_slice = slice_index < o.actions.size() && action_index < slice_index;
        if (action == "export_settings") {
            const std::string file = o.cli.opt_string("export_settings");
            if (!file.empty())
                add(false, false, "--export-settings " + file, fs::path(file),
                    "the file --export-settings " + file + " writes", "the file --export-settings " + file + " writes",
                    before_slice);
        } else if ((action == "export_stl" || action == "export_stls") && model != nullptr) {
            const std::string dir  = action == "export_stls" ? o.cli.opt_string("export_stls") : std::string();
            const std::string flag = action == "export_stls" ? "--export-stls " + dir : std::string("--export-stl");
            unsigned index = 1;
            for (const Slic3r::ModelObject* object : model->objects) {
                const std::string file = slicer_cli::object_stl_file(o, *object, index++, dir);
                const std::string what = "the object STL file " + file + " " + flag + " writes";
                add(false, true, flag, fs::path(file), what, what, before_slice);
            }
        }
    }
    // The outputs of the actions given before --slice were written before the
    // check that follows the calibration pattern: they keep the names they
    // took, in front of the ones still to come (the sentence names the flag
    // that wrote first). They were checked before they ran, and running them
    // again is not this check's to do.
    if (reserved != nullptr && !reserved->empty())
        outputs.insert(outputs.begin(), reserved->begin(), reserved->end());
    if (o.slice_mode)
        for (int p : plates) {
            const std::string folder = slicer_cli::slicedata_dir(o, "export_slicedata", p);
            if (!folder.empty())
                add(true, false, "--export-slicedata " + folder, fs::path(folder),
                    "the folder --export-slicedata writes (" + folder + ")",
                    "inside the folder --export-slicedata writes (" + folder + ")");
        }
    if (o.slice_mode) {
        add(false, false, "", outdir / "result.json", "the run's own result.json in --outputdir",
            "the run's own result.json in --outputdir");
        for (int p : plates) {
            const std::string name = "plate_" + std::to_string(p) + ".gcode";
            add(false, false, "", outdir / name, "the run's own " + name + " in --outputdir",
                "the run's own " + name + " in --outputdir");
        }
    } else if (!slicer_cli::model_actions_only(o) && !o.output_file.empty()) {
        add(false, false, "-o " + o.output_file, fs::path(o.output_file), "the G-code -o " + o.output_file + " writes",
            "the G-code -o " + o.output_file + " writes");
    }

    // ── What the run reads ──────────────────────────────────────────────
    // With the STL and OBJ files the assemble list names, as its loader opens
    // them (the path as written, construct_assemble_list): they were read into
    // the model, and an output over one would replace the user's part.
    std::vector<std::string> inputs = named_input_files(context, o);
    // The files the loaders read beside those models (an OBJ's mtllib file,
    // its textures, a glTF's buffers), as each loader resolves them: the
    // user's files too, named only inside the model.
    {
        std::vector<std::string> side;
        for (const std::string& file : o.input_files)
            for (std::string& s : slicer_cli::loader_side_files(file, true))
                side.push_back(std::move(s));
        if (context.g_assemble)
            for (const std::string& file : context.g_assemble->sources)
                for (std::string& s : slicer_cli::loader_side_files(file, false))
                    side.push_back(std::move(s));
        inputs.insert(inputs.end(), side.begin(), side.end());
    }

    // ── The checks ──────────────────────────────────────────────────────
    const auto refuse = [&](const char* tag, const std::string& sentence) {
        if (detail != nullptr)
            *detail = sentence;
        else if (o.slice_mode)
            write_result_json(context, outdir.string(), CLI_INVALID_PARAMS, o.slice_plate,
                              cli_error_sentence(CLI_INVALID_PARAMS) + " " + sentence, {}, 0, 0);
        std::cerr << "Error: " << sentence << "\n";
        emit_event({{"event","input_error"}, {"tag", tag}, {"message", sentence}});
        return CLI_INVALID_PARAMS;
    };
    // 0: apart; 1: one name, or two names of one file; 2: names that differ
    // only by letter case.
    const auto same = [&](const fs::path& a, const fs::path& b) {
        boost::system::error_code ec;
        if (fs::equivalent(a, b, ec) && !ec)
            return 1;
        const std::string ka = key(a), kb = key(b);
        if (ka == kb)
            return 1;
        return lower(ka) == lower(kb) ? 2 : 0;
    };
    const auto inside = [&](const fs::path& a, const fs::path& folder) {
        const std::string ka = key(a), kf = key(folder) + "/";
        if (boost::algorithm::starts_with(ka, kf))
            return 1;
        return boost::algorithm::starts_with(lower(ka), lower(kf)) ? 2 : 0;
    };
    // How two outputs meet (0, 1 or 2 as above).
    const auto meet = [&](const RunOutput& a, const RunOutput& b) -> int {
        if (a.folder && b.folder)
            return 0;   // --export-slicedata's own plate folders, one per plate
        if (a.folder || b.folder) {
            const RunOutput& folder = a.folder ? a : b;
            const RunOutput& other  = a.folder ? b : a;
            if (const int c = same(other.path, folder.path))
                return c;
            return inside(other.path, folder.path);
        }
        return same(a.path, b.path);
    };
    for (const RunOutput& out : outputs)
        for (const std::string& input : inputs) {
            boost::system::error_code ec;
            const fs::path in(input);
            bool hit = false;
            if (!out.folder)
                hit = fs::exists(out.path, ec) && fs::equivalent(out.path, in, ec) && !ec;
            else   // the cache files of a plate folder: any input in it
                hit = fs::exists(out.path, ec) && fs::exists(in, ec) && inside(in, out.path) != 0;
            if (!hit)
                continue;
            if (out.flag.empty())   // result.json or a plate's G-code
                return refuse("ExportOverwritesInput", "--slice would overwrite the input file '" + input + "' with " +
                                                           out.what + "; choose another --outputdir.");
            const std::string who = out.flag.rfind("--export-3mf ", 0) == 0 ? std::string("--export-3mf") : out.flag;
            return refuse("ExportOverwritesInput",
                          who + " would overwrite the input file '" + input + "'; give it another name.");
        }
    // --export-slicedata into the folder --load-slicedata reads, or one inside
    // the other: the export would write over the cache the run loads from.
    if (o.slice_mode)
        for (int p : plates) {
            const std::string out_dir = slicer_cli::slicedata_dir(o, "export_slicedata", p);
            const std::string in_dir  = slicer_cli::slicedata_dir(o, "load_slicedata", p);
            if (out_dir.empty() || in_dir.empty())
                continue;
            if (same(fs::path(out_dir), fs::path(in_dir)) || inside(fs::path(out_dir), fs::path(in_dir)) ||
                inside(fs::path(in_dir), fs::path(out_dir)))
                return refuse("ExportOverwritesInput", "--export-slicedata " + out_dir +
                                                           " would overwrite the folder --load-slicedata " + in_dir +
                                                           " reads; give it another folder.");
        }
    for (size_t i = 0; i < outputs.size(); ++i)
        for (size_t j = i + 1; j < outputs.size(); ++j) {
            const RunOutput& a = outputs[i];
            const RunOutput& b = outputs[j];
            if (a.flag.empty())
                continue;   // two of the run's own files never clash (result.json, plate_<n>.gcode)
            if (a.flag == b.flag)
                continue;   // one flag's own files: one object STL each, one folder per plate
            const int c = meet(a, b);
            if (c == 0)
                continue;
            if (a.stl && b.stl)
                // --export-stl and --export-stls into one folder write the same
                // obj_<n>_<name>.stl files, each action in its turn (before or
                // after --slice, so from a model the arrange may have moved):
                // the later one replaces the earlier.
                return refuse("ExportNameTaken", a.flag + " and " + b.flag + " write the same object STL file " +
                                                     b.path.generic_string() +
                                                     (c == 1 ? "" : " (the names differ only by letter case)") +
                                                     "; give --export-stls another folder.");
            if (a.folder)   // a folder that would hold one of the run's own files
                return refuse("ExportNameTaken", a.flag + (c == 1 ? " holds " : " differs only by letter case from ") +
                                                     b.what + "; choose another folder.");
            return refuse("ExportNameTaken", c == 1 ? a.flag + " is " + b.is_what + "; choose another name."
                                                    : a.flag + " differs only by letter case from " + b.what +
                                                          "; choose another name.");
        }
    return 0;
}

} // namespace slicer_cli
