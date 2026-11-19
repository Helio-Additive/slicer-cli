#include "cli_result_json.hpp"

#include "cli_run_context.hpp"
#include "cli_load_settings.hpp"
#include "cli_assemble.hpp"
#include "cli_command_line.hpp"
#include "cli_model_load.hpp"
#include "cli_events.hpp"
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/algorithm/string.hpp>
#include <iostream>
#include <set>

namespace slicer_cli {

using json = nlohmann::json;

/// The --load-assemble-list file, or empty (an empty value is no list, as
/// the official reads it: BambuStudio.cpp 1795-1799).
std::string assemble_list_file(const CliOptions& o) {
    return o.given_flag("load_assemble_list") ? o.cli.opt_string("load_assemble_list") : std::string();
}

// ── Official CLI result codes and sentences ─────────────────────────────────
// The codes are libslic3r's own (Utils.hpp:21-75 in both engines); OrcaSlicer's
// Utils.hpp stops at -102, so the three later Bambu codes are defined here with
// the Bambu values. The sentences are the official CLIs' cli_errors tables
// (BambuStudio.cpp 117-170 at 5873b5f, OrcaSlicer.cpp 111-159 at 31f6803):
// each build reports the sentences of the app it is built from.
std::string cli_error_sentence(int code) {
#ifdef ENGINE_SNAPMAKER
#define SLICER_APP "Snapmaker Orca"
#elif defined(ENGINE_ORCA)
#define SLICER_APP "Orca Slicer"
#else
#define SLICER_APP "Bambu Studio"
#endif
    static const std::map<int, std::string> sentences = {
        {CLI_SUCCESS, "Success."},
        {CLI_ENVIRONMENT_ERROR, "Failed setting up server environment."},
        {CLI_INVALID_PARAMS, "Invalid parameters to the slicer."},
        {CLI_FILE_NOTFOUND, "The input files to the slicer are not found."},
        {CLI_CONFIG_FILE_ERROR, "The input preset file is invalid and can not be parsed."},
        {CLI_DATA_FILE_ERROR, "The input model file to the slicer can not be parsed."},
        {CLI_UNSUPPORTED_OPERATION, "Unsupported CLI instruction."},
        {CLI_EXPORT_3MF_ERROR, "Failed exporting 3mf files."},
        {CLI_OUT_OF_MEMORY, "Out of memory during slicing. Please upload a model with lower geometry resolution and try again."},
        {CLI_3MF_NOT_SUPPORT_MACHINE_CHANGE, "The selected printer is not supported."},
        {CLI_3MF_NEW_MACHINE_NOT_SUPPORTED, "The selected printer is not compatible with the 3mf."},
        {CLI_PROCESS_NOT_COMPATIBLE, "The selected printer is not compatible with the process preset in the 3mf."},
        {CLI_INVALID_VALUES_IN_3MF, "Invalid parameter value(s) included in the 3mf file."},
        {CLI_OBJECT_ARRANGE_FAILED, "An error occurred when auto-arranging object(s)."},
        {CLI_FILE_VERSION_NOT_SUPPORTED, "Unsupported 3MF version. Please make sure the 3MF file was created with the official version of Bambu Studio, not a beta version."},
        {CLI_NO_SUITABLE_OBJECTS, "One of the plate is empty or has no object fully inside it. Please check that the 3mf contains no empty plate in " SLICER_APP " before uploading."},
        {CLI_VALIDATE_ERROR, "There are some incorrect slicing parameters in the 3mf. Please verify the slicing of all plates in " SLICER_APP " before uploading."},
        {CLI_OBJECTS_PARTLY_INSIDE, "Some objects are located over the boundary of the heated bed."},
        {CLI_FILAMENT_NOT_MATCH_BED_TYPE, "Filaments are not compatible with the plate type. Please verify the slicing of all plates in " SLICER_APP " before uploading."},
        {CLI_FILAMENTS_DIFFERENT_TEMP, "The temperature difference of the filaments used is too large. Please verify the slicing of all plates in " SLICER_APP " before uploading."},
        {CLI_OBJECT_COLLISION_IN_SEQ_PRINT, "Object conflicts were detected when using print-by-object mode. Please verify the slicing of all plates in " SLICER_APP " before uploading."},
        {CLI_OBJECT_COLLISION_IN_LAYER_PRINT, "Object conflicts were detected. Please verify the slicing of all plates in " SLICER_APP " before uploading."},
        {CLI_SLICING_ERROR, "Failed slicing the model. Please verify the slicing of all plates on " SLICER_APP " before uploading."},
        {CLI_GCODE_PATH_CONFLICTS, " G-code conflicts detected after slicing. Please make sure the 3mf file can be successfully sliced in the latest " SLICER_APP ". If the file slices normally in " SLICER_APP ", try moving the wipe tower further from other models, as we use more conservative parameters for it during upload."},
#ifndef ENGINE_SNAPMAKER
        // Not in the Snapmaker fork's CLIResultCode enum (PrintConfig.hpp at
        // bac6fd5): its CLI has no post-slice unprintable-area code.
        {CLI_GCODE_PATH_IN_UNPRINTABLE_AREA, "Found G-code in unprintable area of multi-extruder printers after slicing. Please make sure the 3mf file can be successfully sliced in the latest " SLICER_APP "."},
#endif
        {CLI_FILAMENT_UNPRINTABLE_ON_FIRST_LAYER, "Found some filament unprintable at first layer on current Plate. Please make sure the 3mf file can be successfully sliced with the same Plate type in the latest " SLICER_APP "."},
        {CLI_GCODE_PATH_OUTSIDE, "Found G-code outside of the printable area. The issue may be caused by support, wipe tower, brim, or skirt. If the file slices normally in " SLICER_APP ", try moving the wipe tower further inside the build plate, as we use more conservative parameters for it during upload."},
        {CLI_GCODE_IN_WRAPPING_DETECT_AREA, "Found G-code in the wrapping detect area. Please make sure the 3mf file can be successfully sliced in the latest " SLICER_APP "."},
        // The codes the official flags can end with (BambuStudio.cpp 115-160;
        // OrcaSlicer.cpp 113-156), in the official words.
        {CLI_FILELIST_INVALID_ORDER, "File list order to the slicer is invalid. Please make sure the 3mf in the first place."},
        {CLI_INVALID_PRINTER_TECH, "Unsupported printer technology (not FDM)."},
        {CLI_COPY_OBJECTS_ERROR, "Failed copying objects."},
        {CLI_SCALE_TO_FIT_ERROR, "Failed scaling an object to fit the plate."},
        {CLI_EXPORT_STL_ERROR, "Failed exporting STL files."},
        {CLI_EXPORT_OBJ_ERROR, "Failed exporting OBJ files."},
        {CLI_POSTPROCESS_NOT_SUPPORTED, "post_process is not supported under CLI."},
        {CLI_PRINTABLE_SIZE_REDUCED, "The selected printer's bed size is smaller than the bed size used in the print profile."},
        {CLI_OBJECT_ORIENT_FAILED, "An error occurred when auto-orienting object(s)."},
#ifdef ENGINE_ORCA
        {CLI_MODIFIED_PARAMS_TO_PRINTER, "Found modified parameter in printer preset in the 3mf file, which should not be changed."},
#else
        {CLI_MODIFIED_PARAMS_TO_PRINTER, "You cannot change the Printable Area, Printable Height, and Exclude Area in Printer Settings."},
        {CLI_3MF_FEATURE_NOT_SUPPORTED, "Unsupported features were found in this 3MF file. These features are still in an experimental stage. Please wait until MakerWorld supports them before uploading."},
#endif
        {CLI_EXPORT_CACHE_DIRECTORY_CREATE_FAILED, "Failed creating directory when exporting cache data."},
        {CLI_EXPORT_CACHE_WRITE_FAILED, "Failed exporting cache data."},
        {CLI_IMPORT_CACHE_NOT_FOUND, "Cache data not found."},
        {CLI_IMPORT_CACHE_DATA_CAN_NOT_USE, "Cache data can not be parsed."},
        {CLI_IMPORT_CACHE_LOAD_FAILED, "Failed importing cache data."},
        {CLI_SLICING_TIME_EXCEEDS_LIMIT, "Slicing time of a certain plate exceeds the limit. Please simplify the model or use a larger slicing layer height."},
        {CLI_TRIANGLE_COUNT_EXCEEDS_LIMIT, "Triangle count of single plate exceeds the limit. Please simplify the model and try to upload again."},
        {CLI_NO_SUITABLE_OBJECTS_AFTER_SKIP, "No printable objects to slice after skipping."},
        {CLI_SPIRAL_MODE_INVALID_PARAMS, "Some slicing parameters cannot work with Spiral Vase mode. Please solve the issue in " SLICER_APP " before uploading."},
#ifndef ENGINE_SNAPMAKER
        // Not in the Snapmaker fork's CLIResultCode enum (PrintConfig.hpp at
        // bac6fd5): its CLI has neither code.
        {CLI_FILAMENT_CAN_NOT_MAP, "Some filaments cannot be mapped to correct extruders for multi-extruder Printer."},
        {CLI_ONLY_ONE_TPU_SUPPORTED, "Not support printing 2 or more TPU filaments."},
#ifdef ENGINE_BAMBU
        {CLI_FILAMENTS_NOT_SUPPORTED_BY_EXTRUDER, "Some filaments cannot be printed on the extruder mapped to."},
        // Bambu Studio 2.9.1 (Utils.hpp 77 at v02.09.01.70): mixed nozzle diameters.
        {CLI_MULTI_NOZZLE_DIAMETER_INVALID, "Multi-nozzle diameter mode is enabled, but one or more plates use only a single nozzle diameter. Please turn off multi-diameter mode, or use more than one nozzle diameter on every plate, in Bambu Studio before uploading."},
#endif
#endif
    };
#undef SLICER_APP
    const auto it = sentences.find(code);
    return it == sentences.end() ? std::string("Failed slicing the model.") : it->second;
}

/// Records a failure on the plate outcome: the official sentence for `code`,
/// then the specific sentence (which object, which setting) when there is one.
void set_outcome_failure(PlateOutcome& outcome, int code, const std::string& detail) {
    outcome.cli_code = code;
    outcome.error_string = cli_error_sentence(code);
    if (!detail.empty())
        outcome.error_string += " " + detail;
    if (!outcome.refusal_note.empty())
        outcome.error_string += " " + outcome.refusal_note;
}

/// The parts of the assemble list in hand — the built one, or the one a refusal
/// left half-built (they name the run's files just the same) — or nullptr.
static const std::vector<std::string>* assemble_sources(CliRunContext& context) {
    if (context.g_assemble)
        return &context.g_assemble->sources;
    if (!context.g_assemble_pending_sources.empty())
        return &context.g_assemble_pending_sources;
    return nullptr;
}

/// result.json in the official shape (record_exit_reson, BambuStudio.cpp
/// 476-593 at 5873b5f). OrcaSlicer's own record (OrcaSlicer.cpp 416-460) is a
/// subset of these keys, so both builds write this one. Keys beyond the
/// official ones: "engine", per-plate "gcode_file" and "warnings" (every
/// slicing warning, where "warning_message" keeps only the last, as the
/// official one does).
/// The files the command line names for the run to read: the inputs, the
/// settings files and every file of a list option, as the loaders read them
/// (merge_loaded_settings, load_downward_printers).
std::vector<std::string> named_input_files(CliRunContext& context, const CliOptions& o) {
    std::vector<std::string> inputs = o.input_files;
    if (!o.machine_config.empty())  inputs.push_back(o.machine_config);
    if (!o.filament_config.empty()) inputs.push_back(o.filament_config);
    if (!o.process_config.empty())  inputs.push_back(o.process_config);
    if (!o.bundle_config.empty())   inputs.push_back(o.bundle_config);
    for (const slicer_cli::FileOption& option : slicer_cli::file_options()) {
        if (!o.given_flag(option.key))
            continue;
        if (option.vector) {
            if (const auto* files = o.cli.option<Slic3r::ConfigOptionStrings>(option.key))
                for (const std::string& file : files->values)
                    if (!file.empty()) inputs.push_back(file);
        } else if (const std::string file = o.cli.opt_string(option.key); !file.empty()) {
            inputs.push_back(file);
        }
    }
    if (const std::vector<std::string>* sources = assemble_sources(context))
        for (const std::string& file : *sources)
            inputs.push_back(file);
    return inputs;
}

/// The files the loaders read beside the models `paths` name (an OBJ's
/// mtllib file and its textures, a glTF's buffers): the user's files too,
/// named only inside the model, and the run reads them — check_run_outputs
/// protects them the same way.
static std::vector<std::string> model_side_files(const std::vector<std::string>& paths) {
    std::vector<std::string> out;
    for (const std::string& path : paths)
        for (const std::string& side : slicer_cli::loader_side_files(path, true))
            out.push_back(side);
    return out;
}


bool write_result_json(CliRunContext& context, const std::string& outputdir, int code, int plate_id,
                              const std::string& error_string,
                              const std::vector<PlateOutcome>& outcomes,
                              long long prepare_time_ms, long long export_time_ms) {
    // result.json never replaces a file the run reads (an input given as
    // <outputdir>/result.json): the run is refused for that clash, and the
    // user's file stays as it was. Only where a file is there at all: the
    // loader-side scan below opens the models, and nothing of the run's is at
    // risk when the name is free.
    {
        const boost::filesystem::path target = boost::filesystem::path(outputdir) / "result.json";
        boost::system::error_code present;
        if (boost::filesystem::exists(target, present)) {
            const auto keep = [&](const std::string& named, const std::string& why) {
                boost::system::error_code ec;
                if (!(boost::filesystem::exists(target, ec) && boost::filesystem::equivalent(target, named, ec) && !ec))
                    return false;
                const std::string shown = target.string();
                std::cerr << "Error: " << shown << why << "; result.json is not written over it\n";
                emit_event({{"event","output_error"}, {"tag","ResultIsInput"}, {"path", shown},
                            {"message", shown + why + "; result.json is not written over it"}});
                return true;
            };
            if (context.g_result_json_options != nullptr) {
                std::vector<std::string> inputs = named_input_files(context, *context.g_result_json_options);
                for (const std::string& side : model_side_files(context.g_result_json_options->input_files))
                    inputs.push_back(side);
                // The assemble list's models and their side files, as
                // check_run_outputs reads them (named_input_files has the
                // models; their side files are added here). The list being
                // built counts too: its parts are read before it is the run's.
                if (const std::vector<std::string>* sources = assemble_sources(context))
                    for (const std::string& file : *sources)
                        for (std::string& side : slicer_cli::loader_side_files(file, false))
                            inputs.push_back(std::move(side));
                if (context.g_result_json_mode != nullptr) {
                    // --layout's JSON and --engine-info's file: read by the
                    // run, and named by a mode flag rather than an option.
                    if (!context.g_result_json_mode->layout_json_file.empty())
                        inputs.push_back(context.g_result_json_mode->layout_json_file);
                    if (!context.g_result_json_mode->engine_info_file.empty())
                        inputs.push_back(context.g_result_json_mode->engine_info_file);
                }
                for (const std::string& input : inputs)
                    if (keep(input, " is the input file '" + input + "'"))
                        return false;
            }
            // A command line refused before it was all read: what it names
            // after the refusal is not in the options, so its own words are
            // read for the files the run would open (a model, a flag that
            // reads a file, and the files those models' loaders read beside
            // them).
            if (!context.g_command_line_parsed) {
                const slicer_cli::CommandLineFiles files = slicer_cli::command_line_files(context.g_command_line_words);
                std::vector<std::string> named = files.named;
                named.insert(named.end(), files.models.begin(), files.models.end());
                for (const std::string& side : model_side_files(files.models))
                    named.push_back(side);
                for (const std::string& name : named)
                    if (keep(name, " is named on the command line ('" + name + "')"))
                        return false;
            }
        }
    }
    json j;
    j["plate_index"]  = plate_id;
    j["return_code"]  = code;
    j["error_string"] = error_string;
    // The failed plate's reasons, beyond the official sentence
    // (cli_failure_reason.hpp); the failed plate is the last one run.
    if (code != 0 && !outcomes.empty() && outcomes.back().cli_code != 0 && !outcomes.back().error_reason.empty())
        j["error_reason"] = outcomes.back().error_reason;
    j["prepare_time"] = prepare_time_ms;
    j["export_time"]  = export_time_ms;
#ifdef ENGINE_SNAPMAKER
    j["engine"] = "snapmaker";
#elif defined(ENGINE_ORCA)
    j["engine"] = "orcaslicer";
#else
    j["engine"] = "bambustudio";
#endif
    const PlateOutcome* summary = outcomes.empty() ? nullptr : &outcomes.front();
    j["layer_height"]          = summary ? summary->layer_height : 0.f;
    j["wall_loops"]            = summary ? summary->wall_loops : 0;
    j["sparse_infill_density"] = summary ? summary->sparse_infill_density : 0.f;
    // Keep these optional: only catalog-validated named selections are
    // reported, never mutable project or settings-file id fields.
    if (summary && !summary->printer_preset.empty())
        j["printer_preset"] = summary->printer_preset;
    if (summary && !summary->process_preset.empty())
        j["process_preset"] = summary->process_preset;
    if (summary && std::any_of(summary->filament_presets.begin(), summary->filament_presets.end(),
                               [](const std::string& name) { return !name.empty(); }))
        j["filament_presets"] = summary->filament_presets;
    // Settings the file states that this engine has no definition for (the
    // engine ignores them; the GUI lists them as incompatible settings).
    std::set<std::string> unknown;
    for (const PlateOutcome& p : outcomes)
        unknown.insert(p.unknown_settings.begin(), p.unknown_settings.end());
    j["unknown_settings"] = std::vector<std::string>(unknown.begin(), unknown.end());
    // Settings the project file states that the presets replaced: the file
    // does not list them as changed by its maker, and the desktop app drops
    // such a value the same way. A caller that edited the file reads here
    // what was not kept.
    std::set<std::string> replaced;
    for (const PlateOutcome& p : outcomes)
        replaced.insert(p.settings_replaced_by_presets.begin(), p.settings_replaced_by_presets.end());
    j["settings_replaced_by_presets"] = std::vector<std::string>(replaced.begin(), replaced.end());
    // --downward-check and the settings merge: the official keys, written
    // only when they have entries (BambuStudio.cpp 488-493; OrcaSlicer.cpp
    // 429-430). A printer is compatible when every checked plate fits it.
    {
        std::vector<std::string> checked;
        std::set<std::string> failed;
        bool any_checked = false, sequence_plates = false, multi_nozzle_diameters = false;
        for (const PlateOutcome& p : outcomes) {
            if (!p.downward_checked) continue;
            if (!any_checked) checked = p.downward_printers;
            any_checked = true;
            failed.insert(p.downward_failed.begin(), p.downward_failed.end());
            sequence_plates |= p.sequence_plate;
            multi_nozzle_diameters |= p.multi_nozzle_diameters;
        }
        std::vector<std::string> downward;
        for (const std::string& name : checked)
            if (!failed.count(name)) downward.push_back(name);
        if (!downward.empty())
            j["downward_compatible_machine"] = downward;
#ifndef ENGINE_ORCA
        std::vector<std::string> upward;
        for (const PlateOutcome& p : outcomes)
            if (p.settings_merge) { upward = p.settings_merge->upward_machines; break; }
        // BambuStudio.cpp 4894-4901.
        upward.erase(std::remove_if(upward.begin(), upward.end(),
                                    [&](const std::string& name) { return failed.count(name) > 0; }),
                     upward.end());
        if (!upward.empty())
            j["upward_compatible_machine"] = upward;
        // BambuStudio.cpp 5389-5393 at v02.09.01.70.
        std::vector<std::string> taint;
        if (any_checked && sequence_plates)
            taint.push_back("PrintSequenceByObject");
        if (any_checked && multi_nozzle_diameters)
            taint.push_back("MultiNozzleDiameters");
        if (!taint.empty())
            j["upward_compatibility_taint"] = taint;
#endif
    }
    j["sliced_plates"] = json::array();
    for (const PlateOutcome& p : outcomes) {
        if (!p.exported) continue;
        json plate;
        plate["id"]                     = p.plate_id;
        plate["sliced_time"]            = p.sliced_time_ms;
        plate["sliced_time_with_cache"] = 0;
        plate["triangle_count"]         = p.triangle_count;
        plate["warning_message"]        = p.warning_message;
        plate["total_predication"]      = p.total_predication;
        plate["main_predication"]       = p.main_predication;
        plate["filament_change_times"]  = p.filament_change_times;
        if (!p.feature_type_times.empty()) {
            json times = json::object();
            for (const auto& [name, t] : p.feature_type_times) times[name] = t;
            plate["feature_type_times"] = times;
        }
        if (!p.objects.empty())   plate["objects"]   = p.objects;
        if (!p.filaments.empty()) plate["filaments"] = p.filaments;
        if (!p.filament_maps.empty())       plate["filament_maps"] = p.filament_maps;
        if (!p.filament_nozzle_map.empty()) plate["filament_nozzle_map"] = p.filament_nozzle_map;
        if (!p.filament_map_mode.empty())   plate["filament_map_mode"] = p.filament_map_mode;
        plate["warnings"]   = p.warnings;
        plate["gcode_file"] = p.gcode_path;
        plate["unknown_settings"] = p.unknown_settings;
        plate["settings_replaced_by_presets"] = p.settings_replaced_by_presets;
        j["sliced_plates"].push_back(plate);
    }
    const boost::filesystem::path path = boost::filesystem::path(outputdir) / "result.json";
    boost::filesystem::ofstream out(path, std::ios::out | std::ios::trunc);
    if (out.is_open())
        out << j.dump(4, ' ', false, json::error_handler_t::replace) << std::endl;
    if (out.is_open() && out.good())
        return true;
    // The official writer swallows this (c.open ... catch (...) {} in record_exit_reson,
    // BambuStudio.cpp 594-599). A run whose result document is missing is
    // not a finished run here: say so, and let the caller fail it.
    const std::string shown = path.string();
    std::cerr << "Error: cannot write " << shown << "\n";
    emit_event({{"event","output_error"}, {"tag","ResultNotWritten"}, {"path", shown},
                {"message","Could not write " + shown}});
    return false;
}

/// One progress event: {"event":"progress","percent":N,...}. The overall
/// percent follows the official CLI's formula (cli_callback_mgr_t::update,
/// BambuStudio.cpp 375-385): 3 for preparing, then 90 points shared evenly
/// by the plates of this run. The official CLI writes progress only into a
/// named pipe and only on Linux (#if __linux__, BambuStudio.cpp 243-430);
/// this is the same event stream as the warnings, on every OS.
/// A finished last plate is 93, not 100: the official run reports "Exporting
/// 3mf" at 97 (BambuStudio.cpp 8131) and 100 only with "All done, Success"
/// after everything is written (8237), see emit_run_progress.
static void emit_progress_event(const PlateOutcome& outcome, int total, int plate_percent,
                                const std::string& message) {
    const int count = std::max(1, outcome.plate_count);
    const int index = std::max(1, outcome.plate_index);
    emit_event({{"event", "progress"},
                {"tag", "SliceProgress"},
                {"percent", std::min(100, std::max(0, total))},
                {"plate_id", outcome.plate_id},
                {"plate_index", index},
                {"plate_count", count},
                {"plate_percent", std::min(100, std::max(0, plate_percent))},
                {"message", message}});
}

void emit_progress(const PlateOutcome& outcome, int plate_percent, const std::string& message) {
    const int count = std::max(1, outcome.plate_count);
    const int index = std::max(1, outcome.plate_index);
    const int total = 3 + int(((index - 1) * 90.0) / count + (plate_percent * 0.9) / count);
    emit_progress_event(outcome, total, plate_percent, message);
}

/// A run-level step after the plates: the official CLI's fixed percents
/// (97 "Exporting 3mf", BambuStudio.cpp 8131; 100 "All done, Success", 8237).
void emit_run_progress(const PlateOutcome& last, int total, const std::string& message) {
    emit_progress_event(last, total, 100, message);
}

/// The noted-value warning (InvalidValuesNoted) as the run announces it: the
/// event and its line on stderr. --slice 0's check pass holds them until every
/// plate has passed (RunState::noted_held), so the warning's "the run slices
/// with them" is said only of a run that slices.
void announce_noted_warning(const json& announcement) {
    emit_event(announcement);
    std::cerr << "Warning: " << announcement.value("message", std::string()) << "\n";
}

/// The settings check's warnings (InvalidValuesNoted) of the plate this
/// result is for, once: any the outcome already holds (a carry from another
/// plate's call) are replaced.
void take_noted_warnings(PlateOutcome& outcome, const std::map<int, json>& noted, int plate_id) {
    json& warnings = outcome.warnings;
    warnings.erase(std::remove_if(warnings.begin(), warnings.end(),
                                  [](const json& w) { return w.value("tag", "") == "InvalidValuesNoted"; }),
                   warnings.end());
    const int plate = outcome.plate_id > 0 ? outcome.plate_id : (plate_id > 0 ? plate_id : 1);
    if (const auto it = noted.find(plate); it != noted.end())
        for (const json& w : it->second)
            warnings.push_back(w);
}

} // namespace slicer_cli
