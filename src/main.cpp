// libslic3r_standalone - CLI tool for slicing with libslic3r
// Supports loading BambuStudio JSON config files (machine, filament, process)

#include <iostream>
#include <limits>
#include <stdexcept>
#include <fstream>
#include <string>
#include <memory>
#include <map>
#include <vector>
#include <optional>
#include <iterator>
#include <set>
#include <cstdint>
#include <cstdlib>
#include <algorithm>
#include <chrono>
#include <cctype>
#include <cstring>
#include <ctime>
#include <iomanip>
#include <sstream>
#ifdef _WIN32
#include <windows.h>
#include <io.h>
#include <fcntl.h>
#else
#include <cerrno>
#include <unistd.h>
#include <fcntl.h>
#endif
#include <functional>

// Core libslic3r headers
// Bambu 2.9.1 Log.hpp occupies boost/log/trivial.hpp's include guard, so boost's
// severity types must be declared before any engine header (as the engine's own Log.cpp does).
#include <boost/log/trivial.hpp>
#include "libslic3r/libslic3r.h"
#include "libslic3r/Utils.hpp"   // get_process_name/get_current_pid: the engine's own folder-ownership test
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/Thread.hpp"
#include "libslic3r/GCode.hpp"
#include "libslic3r/GCode/GCodeProcessor.hpp"
#include "libslic3r/Format/STL.hpp"
#include "libslic3r/Format/bbs_3mf.hpp"
#include "libslic3r/Preset.hpp"
#include "libslic3r/PresetBundle.hpp"
#include "libslic3r/AppConfig.hpp"
#include "libslic3r/miniz_extension.hpp"
#include "libslic3r/ModelArrange.hpp"
#include "libslic3r/Arrange.hpp"
#include "libslic3r/GCode/WipeTower.hpp"
#ifdef ENGINE_BAMBU
#include "libslic3r/FilamentMixer.hpp"
#endif
#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/BuildVolume.hpp"
#include "libslic3r/Semver.hpp"
#include "libslic3r_version.h"
#include "cli_snapmaker.hpp"
#include "cli_gcode_timing.hpp"

// For JSON parsing (using libslic3r's built-in nlohmann/json)
#include <nlohmann/json.hpp>

#include <boost/algorithm/string.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/filesystem.hpp>
#include <boost/nowide/args.hpp>
#include <boost/nowide/convert.hpp>
#include <boost/nowide/fstream.hpp>
// boost::nowide::nowide_filesystem: <boost/nowide/filesystem.hpp> since Boost
// 1.73 (OrcaSlicer.cpp 44); the older bundled Nowide calls the header
// integration/filesystem.hpp (BambuStudio.cpp 43).
#if __has_include(<boost/nowide/filesystem.hpp>)
#include <boost/nowide/filesystem.hpp>
#else
#include <boost/nowide/integration/filesystem.hpp>
#endif

#include <csignal>
#include "diagnostic_output.hpp"

#include "calib_args.hpp"
#include "layout_plan.hpp"
#include "cli_options.hpp"
#include "cli_command_line.hpp"
#include "cli_flag_support.hpp"
#include "desktop_presets.hpp"
#include "cli_model_load.hpp"
#include "cli_failure_reason.hpp"
#include "cli_run_steps.hpp"
#include "cli_simplify.hpp"
#include "cli_list_flags.hpp"
#include "cli_engine_info.hpp"
#include "cli_load_settings.hpp"
#include "cli_repetitions.hpp"
#include "cli_assemble.hpp"
#include "cli_layer_ranges.hpp"
#include "cli_nozzle_diameter.hpp"
#include "cli_filament_map.hpp"
#include "cli_nozzle_options.hpp"
#include "cli_project_load.hpp"
#include "cli_pipe.hpp"
#include "cli_slice_plate.hpp"
#include "cli_slice_export.hpp"
#include "cli_slice_state.hpp"
#include "cli_file_info.hpp"
#include "cli_plate_plan.hpp"
#include "cli_project_values.hpp"
#include "cli_named_presets.hpp"
#include "cli_plate_arrange.hpp"
#include "cli_plate_report.hpp"
#include "cli_run_cleanup.hpp"
#include "cli_result_json.hpp"
#include "cli_run_context.hpp"
#include "cli_project_settings.hpp"
#include "cli_percent_widths.hpp"
#include "cli_preset_staging.hpp"
#include "cli_arrange_geometry.hpp"
#include "cli_setting_overrides.hpp"
#include "cli_list_settings.hpp"
#include "cli_printer_catalog.hpp"
#include "cli_project_archive.hpp"
#include "cli_filament_config.hpp"
#include "cli_json_config.hpp"
#include "cli_legacy_gcode.hpp"
#include "cli_engine_events.hpp"

#ifdef __APPLE__
#include <mach-o/dyld.h>   // _NSGetExecutablePath
#include <climits>         // PATH_MAX
#endif
using slicer_cli::CliRunContext;
using slicer_cli::PlateOutcome;
using slicer_cli::set_outcome_failure;
using slicer_cli::assemble_list_file;
using slicer_cli::cli_error_sentence;
using slicer_cli::named_input_files;
using slicer_cli::write_result_json;
using slicer_cli::emit_progress;
using slicer_cli::emit_run_progress;
using slicer_cli::RunBackupDetach;
using slicer_cli::sweep_stale_staging_folders;
using slicer_cli::run_backup_path;
using slicer_cli::set_run_backup_path;
using slicer_cli::create_archive_dirs;
using slicer_cli::record_plate_statistics;
using slicer_cli::record_plate_for_export;
using slicer_cli::post_slice_checks;
using slicer_cli::mm_text;
using slicer_cli::check_objects_inside_bed;
using slicer_cli::downward_plate_size;
using slicer_cli::move_to_new_bed;
using slicer_cli::assemble_tower_inside_plate;
using slicer_cli::tower_reach_box;
using slicer_cli::arrange_on_bed;
using slicer_cli::tower_default_corner;
using slicer_cli::assemble_tower_default_corner;
using slicer_cli::TowerWipeParams;
using slicer_cli::tower_wipe_params;
using slicer_cli::estimate_tower_polygon_global;
using slicer_cli::arrange_assemble_plate;
using slicer_cli::this_engine_profiles_dir;
using slicer_cli::apply_file_inherits;
using slicer_cli::config_id_text;
#ifdef ENGINE_ORCA
using slicer_cli::config_value_text;
#endif
using slicer_cli::project_presets_shipped;
using slicer_cli::load_system_presets;
using slicer_cli::stage_named_presets_for_project;
using slicer_cli::desktop_plate_type_for_printer;
using slicer_cli::resolve_named_presets;
using slicer_cli::run_list_presets;
#ifdef ENGINE_ORCA
using slicer_cli::BambuValueAction;
#endif
#ifdef ENGINE_ORCA
using slicer_cli::filament_keys_split_reads_empty;
#endif
#ifdef ENGINE_ORCA
using slicer_cli::bambu_value_actions;
#endif
using slicer_cli::json_scalar_text;
#ifdef ENGINE_ORCA
using slicer_cli::apply_bambu_actions;
#endif
using slicer_cli::ProjectPlan;
using slicer_cli::plate_grid_origin;
using slicer_cli::plate_scene_box;
using slicer_cli::trailing_models_port;
using slicer_cli::plan_project_plates;
using slicer_cli::emit_project_plan_event;
using slicer_cli::run_info;
using slicer_cli::RunOutput;
using slicer_cli::export_sliced_3mf;
using slicer_cli::check_run_outputs;
using slicer_cli::RunState;
using slicer_cli::RunPlate;
using slicer_cli::announce_noted_warning;
using slicer_cli::slice_one_plate;
using json = nlohmann::json;
using slicer_cli::project_settings_json;
#ifdef ENGINE_ORCA
using slicer_cli::RawValues;
using slicer_cli::FileObjectSettings;
using slicer_cli::project_object_settings;
using slicer_cli::maker_changed_key;
using slicer_cli::file_states_number;
using slicer_cli::file_states_text;
#endif

#ifdef ENGINE_BAMBU
using slicer_cli::PercentRewrite;
using slicer_cli::rewrite_percent_line_widths;
#endif

using slicer_cli::staging_owner_marker;
using slicer_cli::write_owner_marker;
using slicer_cli::remove_staging_folder;
using slicer_cli::make_owned_temp_dir;
using slicer_cli::remove_owned_temp_dir;
using slicer_cli::stage_file_copy;
using slicer_cli::stage_vendor_dir;
using slicer_cli::configure_engine_resources;

using slicer_cli::arrange_opt;
using slicer_cli::arrange_exclude_boxes;
using slicer_cli::arrange_add_exclude_areas;
using slicer_cli::arrange_plate_extruders;
using slicer_cli::arrange_estimate_wipe_tower_size;

using slicer_cli::apply_command_line_overrides;

using slicer_cli::run_list_settings;

using slicer_cli::EngineFit;
using slicer_cli::ShippedPrinterPreset;
using slicer_cli::engine_executable_dir;
using slicer_cli::engine_profiles_dir;
using slicer_cli::orca_profiles_dir;
using slicer_cli::preset_printer_model;
using slicer_cli::engine_fit_for;
using slicer_cli::engine_mismatch_sentence;
using slicer_cli::shipped_printer_presets;
using slicer_cli::shipped_printer_preset_of_variant;
using slicer_cli::preset_name_list;

using slicer_cli::FileVersionVerdict;
using slicer_cli::input_is_stl;
using slicer_cli::input_is_3mf;
using slicer_cli::read_zip_member;
using slicer_cli::zip_member_readable;
using slicer_cli::count_3mf_plates;
using slicer_cli::model_metadata;
using slicer_cli::engine_file_version;
using slicer_cli::engine_version_text;
using slicer_cli::file_version_verdict;

using slicer_cli::project_filament_count;
#ifdef ENGINE_BAMBU
using slicer_cli::apply_explicit_nozzle_mapping;
using slicer_cli::set_default_config;
using slicer_cli::ensure_vector_config_sizes;
using slicer_cli::is_filament_setting_key;
using slicer_cli::align_per_filament_maps;
using slicer_cli::align_per_filament_config_vectors;
#endif
using slicer_cli::normalize_filament_config_vectors;

using slicer_cli::json_file_has_key;
using slicer_cli::load_json_config;
#ifdef ENGINE_BAMBU
using slicer_cli::bbs_3mf_config_contains_nozzle_map;
#endif

using slicer_cli::normalize_legacy_gcode_tokens;
#ifdef ENGINE_ORCA
using slicer_cli::bambu_only_gcode_placeholder;
#endif

using slicer_cli::slicing_notification_tag;
using slicer_cli::warning_level_tag;
using slicer_cli::emit_event;
using slicer_cli::emit_config_substitutions;
using slicer_cli::emit_gcode_result_diagnostics;
using slicer_cli::install_engine_log_bridge;
using slicer_cli::emit_status_warning;
using slicer_cli::emit_validation_event;

// Read all of an fd into a string, polling the cancellation flag between reads
// so SIGINT/Ctrl+C during a slow stream (stdin, FIFO, or pipe) is honoured
// with a bounded exit. Returns false if cancelled.
// 0 = ok, 1 = cancelled, 2 = hard read error (not a cancel)
static int read_all_cancellable(int fd, std::string& out) {
    char buf[4096];
    while (true) {
        if (layout_plan::is_cancelled()) return 1;
#ifdef _WIN32
        // Windows: a redirected pipe can block in _read even after the CRT
        // handler runs. PeekNamedPipe before each read; when the pipe is empty,
        // wait a bounded interval and re-check the flag.
        HANDLE h = reinterpret_cast<HANDLE>(_get_osfhandle(fd));
        if (h != INVALID_HANDLE_VALUE && GetFileType(h) == FILE_TYPE_PIPE) {
            DWORD avail = 0;
            if (PeekNamedPipe(h, nullptr, 0, nullptr, &avail, nullptr) && avail == 0) {
                Sleep(50);
                continue;
            }
        }
        int n = ::_read(fd, buf, static_cast<unsigned>(sizeof buf));
#else
        ssize_t n = ::read(fd, buf, sizeof buf);
        if (n < 0 && errno == EINTR) continue;  // sigaction has no SA_RESTART
#endif
        if (n < 0) return 2;       // hard error (not a cancel)
        if (n == 0) return 0;      // EOF, ok
        out.append(buf, static_cast<size_t>(n));
    }
}

void print_usage(const char* prog_name) {
    slicer_cli::print_help(std::cout, prog_name);
}

// Parse a numeric CLI argument, failing fast with usage instead of letting
// std::stod/std::stoi throw an uncaught exception (which would SIGABRT). Used by
// the --calib-* flags so a typo like `--calib-start abc` exits 1 cleanly.
// ── Per-plate outcome ───────────────────────────────────────────────────
// The command line itself is CliOptions (cli_options.hpp), filled by
// cli_command_line.cpp.














#ifdef ENGINE_ORCA
#endif















/// --slice: every plate (0) or plate N of the input, one G-code each as
/// <outputdir>/plate_N.gcode, through the same single-plate path the default
/// call runs. The official loop stops at the first plate that fails
/// (flush_and_exit inside the plate loop, BambuStudio.cpp 6533-7240), and so
/// does this one. The exit status is the official CLI_* code of that failure.
static int run_slice_mode(CliRunContext& context, const CliOptions& o, Slic3r::Calib_Params& calib_params) {
    namespace fs = boost::filesystem;
    const fs::path outdir = o.outputdir.empty() ? fs::path(".") : fs::path(o.outputdir);
    std::vector<PlateOutcome> outcomes;
    int code = 0;
    std::string error_string;

    try {
        fs::create_directories(outdir);
    } catch (const std::exception& e) {
        std::cerr << "Error: cannot create --outputdir " << outdir.string() << ": " << e.what() << "\n";
        return CLI_ENVIRONMENT_ERROR;
    }

    // --load-assemble-list: the list and every part it names, built before
    // the first plate (BambuStudio.cpp 2165-2188; OrcaSlicer.cpp 1812-1834).
    // A refusal names no plate (record_exit_reson(..., 0, ...)).
    const std::string assemble_file = assemble_list_file(o);
    if (!assemble_file.empty()) {
        auto list = std::make_unique<slicer_cli::AssembleList>();
        // The parts are read while the list is built: a refusal here names the
        // ones its loaders read (and the files they read beside them) as the
        // run's own inputs too, so the result.json it writes — here and from
        // this function's caller — lands on none of them. They stay in hand
        // until the list is the run's (g_assemble).
        const slicer_cli::StepResult built = slicer_cli::load_assemble_list(assemble_file, *list);
        context.g_assemble_pending_sources = list->sources;
        if (built.code != 0) {
            write_result_json(context, outdir.string(), built.code, 0, cli_error_sentence(built.code) + " " + built.message, {}, 0, 0);
            std::cerr << "Error: " << built.message << "\n";
            emit_event({{"event","input_error"}, {"tag","AssembleListRefused"}, {"path", assemble_file},
                        {"message", built.message}});
            return built.code;
        }
        context.g_assemble = std::move(list);
        context.g_assemble_pending_sources.clear();
    } else
    // A missing input is the official "not found", not an unparseable model:
    // the official CLI checks every input before loading it
    // (BambuStudio.cpp 1854-1859 at 5873b5f).
    {
        boost::system::error_code exists_ec;
        const bool input_exists = fs::exists(fs::path(o.input_file), exists_ec);
        if (exists_ec && exists_ec != boost::system::errc::no_such_file_or_directory) {
            // The path could not be checked (permissions, I/O): say so, not
            // "not found".
            const std::string detail = "Cannot check the input " + o.input_file + ": " + exists_ec.message();
            write_result_json(context, outdir.string(), CLI_ENVIRONMENT_ERROR, o.slice_plate,
                              cli_error_sentence(CLI_ENVIRONMENT_ERROR) + " " + detail, {}, 0, 0);
            std::cerr << "Error: " << detail << "\n";
            emit_event({{"event","input_error"}, {"tag","InputUnreadable"}, {"path", o.input_file},
                        {"message", detail}});
            return CLI_ENVIRONMENT_ERROR;
        }
        if (!input_exists) {
            write_result_json(context, outdir.string(), CLI_FILE_NOTFOUND, o.slice_plate,
                              cli_error_sentence(CLI_FILE_NOTFOUND), {}, 0, 0);
            std::cerr << "No such file: " << o.input_file << "\n";
            emit_event({{"event","input_error"}, {"tag","InputNotFound"}, {"path", o.input_file},
                        {"message","No such file: " + o.input_file}});
            return CLI_FILE_NOTFOUND;
        }
    }

    // A project with plate metadata slices plate by plate; anything else (an
    // STL, a 3MF without plates) is one plate holding every object — the
    // official CLI resets plate_to_slice to 0 for a non-Bambu 3MF
    // (BambuStudio.cpp 2192-2196).
    int plate_count = 1;
    bool per_plate_load = false;
    if (context.g_assemble) {
        plate_count = int(context.g_assemble->plates.size());
        per_plate_load = true;
    } else if (slicer_cli::classify_3mf(o.input_file) == slicer_cli::ThreeMfKind::Project) {
        // A geometry-only 3MF (no project settings) loads whole as model
        // geometry, whatever plate tags its model_settings.config keeps.
        const int declared = count_3mf_plates(o.input_file);
        if (declared > 0) {
            plate_count = declared;
            per_plate_load = true;
        }
    }
    // The run's one model and settings (RunState): a prepare call loads every
    // input once and runs the steps the official CLI runs before its plate
    // loop (BambuStudio.cpp 1855-6336; OrcaSlicer.cpp 1537-5470), before any
    // plate is checked or sliced. A project 3MF with model files after it
    // prepares first, as its plan (ProjectPlan) decides the plates; every
    // other run prepares once the plates are known (below).
    RunState rs;
    rs.file_plate_count = plate_count;
    int plan_code = 0;
    std::string plan_error;
    const bool trailing = per_plate_load && !context.g_assemble && trailing_models_port(o);
    std::optional<PlateOutcome> prepare_failure;
    const auto prepare_run = [&](int prepare_plate) {
        PlateOutcome prepare_outcome;
        prepare_outcome.prepare_pass = true;
        prepare_outcome.plate_id = prepare_plate;
        const int rc = slice_one_plate(context, o, calib_params, rs, prepare_plate, std::string(), prepare_outcome);
        if (rc == 0 && prepare_outcome.cli_code == 0 && rs.prepared)
            return;
        if (prepare_outcome.cli_code == 0)
            set_outcome_failure(prepare_outcome, CLI_SLICING_ERROR);
        if (prepare_outcome.error_string.empty())
            prepare_outcome.error_string = cli_error_sentence(prepare_outcome.cli_code);
        plan_code = prepare_outcome.cli_code;
        plan_error = prepare_outcome.error_string;
        prepare_failure = prepare_outcome;
    };
    if (trailing) {
        // --slice 0 may check several plates once the plan is known, so the
        // project's noted-value warning found here is held like the check
        // pass's own (released below when the run checks only one plate).
        rs.hold_noted = o.slice_plate == 0;
        prepare_run(0);
        if (plan_code == 0)
            plate_count = rs.plan.plate_count;
    }

    if (plan_code == 0 && o.slice_plate > plate_count) {
        write_result_json(context, outdir.string(), CLI_INVALID_PARAMS, o.slice_plate,
                          cli_error_sentence(CLI_INVALID_PARAMS) + " --slice " + std::to_string(o.slice_plate) +
                          " but the file has only " + std::to_string(plate_count) + " plate(s).",
                          {}, 0, 0);
        std::cerr << "Error: --slice " << o.slice_plate << " but the file has only "
                  << plate_count << " plate(s)\n";
        emit_event({{"event","input_error"},
                    {"tag","PlateOutOfRange"},
                    {"requested_plate", o.slice_plate},
                    {"plate_count", plate_count},
                    {"message","--slice " + std::to_string(o.slice_plate) +
                               " but the file has only " + std::to_string(plate_count) + " plate(s)"}});
        return CLI_INVALID_PARAMS;
    }

    const auto run_started = std::chrono::steady_clock::now();
    if (o.progress) {
        PlateOutcome preparing;
        emit_progress(preparing, 0, "Prepare slicing");
    }

    std::vector<int> plates;
    if (o.slice_plate == 0)
        for (int p = 1; p <= plate_count; ++p) plates.push_back(p);
    else
        plates.push_back(o.slice_plate);

    // A run that checks every plate before slicing any (--slice 0 on several
    // plates) holds each plate's noted-value warning until the whole check
    // pass has passed: the warning says the run slices with the file's values,
    // which a later plate's refusal makes untrue. The prepare call below
    // checks the first plate, so the hold starts before it.
    const bool hold_noted = o.slice_plate == 0 && plates.size() > 1;
    if (rs.hold_noted && !hold_noted) {
        for (const json& announcement : rs.noted_held)
            announce_noted_warning(announcement);
        rs.noted_held.clear();
    }
    rs.hold_noted = hold_noted;

    // --slice 0 on several plates checks every plate before it slices any
    // (pre_check, BambuStudio.cpp 6441, 7027-7028, 7428-7429; OrcaSlicer.cpp
    // 5565, 6034-6035, 6222-6223): each plate runs up to its slice, and the
    // first refusal ends the run with no G-code written. Every event of the
    // pass is held (diagnostics::held_events). A plate that passes is run
    // again by the slice pass below, so its events are dropped. When a later
    // plate is refused they stay dropped: no plate was sliced, and the
    // official's check pass reports only the refused plate (record_exit_reson
    // with its index, BambuStudio.cpp 6533, 6579; OrcaSlicer.cpp 5650). The
    // refused plate's events are sent with its result.
    // Every other run prepares here, for its one plate or for every plate
    // (--slice 0 on several plates).
    if (!trailing && plan_code == 0) {
        prepare_run(plates.size() == 1 && per_plate_load ? plates.front() : 0);
        // --slice 0 --arrange 1 on several plates: the plates the arrange left.
        if (plan_code == 0 && rs.plan.done) {
            plates.clear();
            for (int p = 1; p <= rs.plan.plate_count; ++p) plates.push_back(p);
        }
    }
    // A prepare call that failed ends the run here: with plate 0 for a
    // project with model files after it (its plan), else with the plate it
    // was preparing (the first plate's check, for --slice 0).
    if (plan_code != 0) {
        code = plan_code;
        error_string = plan_error;
        if (!trailing && prepare_failure) {
            prepare_failure->plate_id = plates.size() == 1 ? plates.front() : 1;
            prepare_failure->plate_index = 1;
            prepare_failure->plate_count = int(plates.size());
            // The refused plate starts without a G-code, as in the slice pass.
            prepare_failure->gcode_path =
                (outdir / ("plate_" + std::to_string(prepare_failure->plate_id) + ".gcode")).string();
            boost::system::error_code ignored;
            fs::remove(prepare_failure->gcode_path, ignored);
            outcomes.push_back(*prepare_failure);
        }
    }
    // Every file the run writes (result.json, the plates' G-code, the
    // --export-3mf project, the --export-settings file, the object STL files,
    // the --export-slicedata folders), checked against each other and against
    // the files the run reads (check_run_outputs). The check runs here, once
    // the plate plan is final: the global arrange (--slice 0 --arrange 1) can
    // add plates the pre-arrange count does not know (BambuStudio.cpp
    // 5627-5722; OrcaSlicer.cpp 4887-4983), and a plate_<n>.gcode it then
    // writes would be overwritten. `plates` is the set this run slices.
    if (const int refused = check_run_outputs(context, o, plates, &rs.model, nullptr))
        return refused;

    const bool pre_check = code == 0 && o.slice_plate == 0 && plates.size() > 1;
    for (size_t i = 0; pre_check && i < plates.size(); ++i) {
        PlateOutcome outcome;
        outcome.plate_id    = plates[i];
        outcome.plate_index = int(i) + 1;
        outcome.plate_count = int(plates.size());
        outcome.gcode_path  = (outdir / ("plate_" + std::to_string(plates[i]) + ".gcode")).string();
        outcome.pre_check   = true;
        std::vector<std::string> held;
        slicer_cli::diagnostics::held_events = &held;
        int rc = 0;
        try {
            rc = slice_one_plate(context, o, calib_params, rs, per_plate_load ? plates[i] : 0, outcome.gcode_path, outcome);
        } catch (...) {
            slicer_cli::diagnostics::held_events = nullptr;
            throw;
        }
        slicer_cli::diagnostics::held_events = nullptr;
        if (rc == 0 && outcome.cli_code == 0)
            continue;
        for (const std::string& line : held)
            slicer_cli::diagnostics::write_event_now(line);
        if (outcome.cli_code == 0)
            set_outcome_failure(outcome, CLI_SLICING_ERROR);
        if (outcome.error_string.empty())
            set_outcome_failure(outcome, outcome.cli_code);
        // The refused plate starts without a G-code, as in the slice pass.
        boost::system::error_code ignored;
        fs::remove(outcome.gcode_path, ignored);
        outcomes.push_back(outcome);
        code = outcome.cli_code;
        error_string = outcome.error_string;
        break;
    }
    if (rs.hold_noted) {
        rs.hold_noted = false;
        // Every plate passed: its own warnings are announced now, in plate
        // order.
        if (code == 0)
            for (const json& announcement : rs.noted_held)
                announce_noted_warning(announcement);
        rs.noted_held.clear();
    }

    for (size_t i = 0; code == 0 && i < plates.size(); ++i) {
        PlateOutcome outcome;
        outcome.plate_id    = plates[i];
        outcome.plate_index = int(i) + 1;
        outcome.plate_count = int(plates.size());
        outcome.gcode_path  = (outdir / ("plate_" + std::to_string(plates[i]) + ".gcode")).string();
        // The plate about to be sliced starts without a G-code in --outputdir,
        // so a failure before export_gcode cannot leave an earlier run's
        // plate_N.gcode looking current. Only this plate: a run that stops
        // here leaves the plates it never reached as they were. The official
        // CLI only overwrites (export to <outputdir>/plate_N.gcode,
        // BambuStudio.cpp 7170-7186; flush_and_exit removes backups only,
        // 458-473); result.json stays the verdict, and a G-code written by
        // this run is kept.
        {
            boost::system::error_code ignored;
            fs::remove(outcome.gcode_path, ignored);
        }
        const int rc = slice_one_plate(context, o, calib_params, rs, per_plate_load ? plates[i] : 0,
                                       outcome.gcode_path, outcome);
        // A failure that set no official code (an engine exception in the
        // process or export step) is the official CLI's slicing failure with
        // its sentence (record_exit_reson(..., CLI_SLICING_ERROR, index+1,
        // cli_errors[CLI_SLICING_ERROR], ...), BambuStudio.cpp 7139, 7423);
        // the exception text is in the slicing_error event.
        if (rc != 0 && outcome.cli_code == 0)
            set_outcome_failure(outcome, CLI_SLICING_ERROR);
        if (outcome.cli_code != 0 && outcome.error_string.empty())
            set_outcome_failure(outcome, outcome.cli_code);
        outcomes.push_back(outcome);
        if (outcome.cli_code != 0) {
            code = outcome.cli_code;
            error_string = outcome.error_string;
            break;
        }
    }
    // --pipe: past the plates, the run's own steps (BambuStudio.cpp 7433-7438).
    if (code == 0 && slicer_cli::pipe_started())
        slicer_cli::pipe_set_plate_info(0, int(plates.size()));
    long long export_ms = 0;
    if (code == 0 && !o.export_3mf.empty()) {
        const auto export_started = std::chrono::steady_clock::now();
        if (o.progress) {
            PlateOutcome last = outcomes.empty() ? PlateOutcome() : outcomes.back();
            emit_run_progress(last, 97, "Exporting 3mf");
        }
        slicer_cli::pipe_update(97, "Exporting 3mf");   // BambuStudio.cpp 8128-8133
        std::string export_error;
        try {
            Slic3r::DynamicPrintConfig project = rs.base;
            apply_command_line_overrides(project, o.overrides, /*report_rejections=*/false);
            // The settings the command line gave: the official setting flags
            // and what this command line's own short flags changed. They are
            // the run's only changes when every other value is a named
            // preset's or a project's loaded over its presets; a settings
            // file or a project sliced as saved brings values of its own, and
            // then every setting that differs from the presets is listed.
            std::set<std::string> given_settings;
            for (const std::string& key : rs.extra.keys())
                given_settings.insert(key);
            for (const std::string& key : rs.base.diff(project))
                given_settings.insert(key);
            const bool settings_files = !o.machine_config.empty() || !o.filament_config.empty() ||
                                        !o.process_config.empty() || !o.bundle_config.empty() ||
                                        o.given_flag("load_settings") || o.given_flag("load_filaments");
            const bool presets_only = !settings_files &&
                                      (rs.project_loaded_like_desktop || !input_is_3mf(o.input_file) ||
                                       slicer_cli::classify_3mf(o.input_file) == slicer_cli::ThreeMfKind::GeometryOnly);
#ifdef ENGINE_SNAPMAKER
            // The switch trimmed each plate's filament list and remapped the
            // model's paint to it, so the project carries that list too:
            // otherwise a reopened project reads the paint against the
            // untrimmed numbering.
            if (slicer_cli::switch_trims_filaments(o.preset_switch, o.printer_preset))
                slicer_cli::trim_project_roster_for_switch(project);
#endif
            code = export_sliced_3mf(context, o, outdir, rs, outcomes, project, presets_only ? &given_settings : nullptr,
                                     export_error);
        } catch (const std::exception& e) {
            code = CLI_EXPORT_3MF_ERROR;
            export_error = e.what();
        }
        if (code != 0)
            error_string = cli_error_sentence(code) + (export_error.empty() ? "" : " " + export_error);
        export_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - export_started).count();
    }
    const long long total_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - run_started).count();
    long long sliced_ms = 0;
    for (const PlateOutcome& p : outcomes) sliced_ms += p.sliced_time_ms;
    if (code == 0) error_string = cli_error_sentence(CLI_SUCCESS);
    const int reported_plate = (code != 0 && !outcomes.empty())
                                   ? (outcomes.back().run_step_failed ? 0 : outcomes.back().plate_id)
                                   : o.slice_plate;
    const bool result_written = write_result_json(context, outdir.string(), code, reported_plate, error_string, outcomes,
                                                  std::max(0LL, total_ms - sliced_ms - export_ms), export_ms);
    if (code != 0)
        std::cerr << "Error: " << error_string << "\n";
    // No result document is a failed run, with the code this mode already
    // uses when --outputdir cannot be made (CLI_ENVIRONMENT_ERROR above).
    if (code == 0 && !result_written)
        code = CLI_ENVIRONMENT_ERROR;
    // 100 only once everything is written ("All done, Success",
    // BambuStudio.cpp 8237).
    if (code == 0 && o.progress) {
        PlateOutcome last = outcomes.empty() ? PlateOutcome() : outcomes.back();
        emit_run_progress(last, 100, "All done, Success");
    }
    // --pipe: the last step, then the pipe closes (BambuStudio.cpp 8234-8241;
    // a failed run closes it without one, flush_and_exit 457-460).
    if (code == 0)
        slicer_cli::pipe_update(100, "All done, Success");
    slicer_cli::pipe_stop();
    return code;
}

static int run_cli_slice(CliRunContext& context, const CliOptions& o, Slic3r::Calib_Params& calib_params) {
#ifdef ENGINE_BAMBU
    // --allow-mix-temp has no engine hook on Bambu 2.9.1 (no plate-level
    // temperature-mix check to lift); a caller that passes it is told once,
    // on the --slice path and on the single-file call alike.
    if (o.given_flag("allow_mix_temp"))
        emit_event({{"event","warning"},
                    {"tag","AllowMixTempIgnored"},
                    {"message", "--allow-mix-temp has no effect: Bambu Studio 2.9.1 has no plate-level "
                                "temperature-mix check to lift"}});
#endif
    if (o.slice_mode)
        return run_slice_mode(context, o, calib_params);
    PlateOutcome outcome;
    outcome.plate_id = o.plate_id;
    RunState rs;
    const int rc = slice_one_plate(context, o, calib_params, rs, o.plate_id, o.output_file, outcome);
    // --progress on the single-file call ends like --slice: 100 once the
    // G-code is written ("All done, Success", BambuStudio.cpp 8237).
    if (rc == 0 && o.progress)
        emit_run_progress(outcome, 100, "All done, Success");
    if (rc == 0 && slicer_cli::pipe_started()) {
        slicer_cli::pipe_set_plate_info(0, 1);
        slicer_cli::pipe_update(100, "All done, Success");
    }
    slicer_cli::pipe_stop();
    return rc;
}

int main(int argc, char** argv) {
    static CliRunContext context;
    // Name the main thread and save its id first, as the official command
    // lines start (BambuStudio.cpp:1490-1493 at 926a719, OrcaSlicer.cpp
    // 1189-1192 at 8500fcd, Snapmaker_Orca.cpp:1045-1048 at bac6fd5). On
    // Windows the first name sets up the thread-naming API with no lock
    // (Thread.cpp WindowsGetSetThreadNameAPIInitialize: "name the main thread
    // before spawning worker threads"), so it must happen before the worker
    // threads name themselves.
#ifdef ENGINE_SNAPMAKER
    Slic3r::set_current_thread_name("Snapmaker_Orca_main");
#elif defined(ENGINE_ORCA)
    Slic3r::set_current_thread_name("orcaslicer_main");
#else
    Slic3r::set_current_thread_name("bambustu_main");
#endif
    Slic3r::save_main_thread_id();
    // UTF-8 everywhere, as the official command lines start: on Windows the
    // arguments come from the wide command line as UTF-8 (BambuStudio.cpp
    // 8636-8660, the wide entry of BambuStudio_app_msvc.cpp; the same in
    // OrcaSlicer), and boost::filesystem paths take UTF-8 strings
    // (nowide_filesystem, BambuStudio.cpp 1510-1523; OrcaSlicer.cpp 1233-1246).
    // Elsewhere both are no-ops.
    boost::nowide::args utf8_args(argc, argv);
    // Timing needs the same UTF-8 paths and the engine's resources root as the
    // other modes (an Orca-derived import reads info/nozzle_info.json through
    // Print::get_hrc_by_nozzle_type, Print.cpp 3084 at the Orca pin), but
    // returns before logging or any slicing path.  Quiet: its stdout and
    // stderr are the one JSON document and nothing.
    if (slicer_cli::gcode_timing_requested(argc, argv)) {
        try {
            // UTF-8 paths first: resource discovery converts the executable's
            // path with root.string(), which on Windows needs this locale for
            // a non-ASCII install directory (BambuStudio.cpp 1510-1523).
            boost::nowide::nowide_filesystem();
            configure_engine_resources(argv[0], /*quiet=*/true);
        } catch (...) {
            // Timing owns stdout: either step may throw before the command's
            // own boundary, so keep its one-document contract.
            return slicer_cli::write_gcode_timing_unavailable();
        }
        return slicer_cli::run_gcode_timing_command(argc, argv);
    }
    try {
        boost::nowide::nowide_filesystem();
    } catch (const std::runtime_error& ex) {
        std::cerr << "boost::nowide::nowide_filesystem failed; slicer-cli stops here.\n" << ex.what() << "\n";
        return CLI_ENVIRONMENT_ERROR;
    }
    // Initialize libslic3r
    Slic3r::set_logging_level(3); // Info level
    boost::log::core::get()->set_logging_enabled(true);

    // Parse arguments: the official flags through this engine's own CLI
    // definitions, slicer-cli's own flags beside them (cli_command_line.cpp).
    CliOptions o;
    slicer_cli::ModeArgs mode_args;
    const std::string& input_file = o.input_file;
    int& plate_id = o.plate_id;  // 0 = all plates (default); >0 = slice only that plate
    const std::string& layout_json_file = mode_args.layout_json_file;
    const bool& layout_plan_mode = mode_args.layout_plan_mode;
    slicer_cli::CalibOptions& calib_opts = mode_args.calib;

    for (int i = 1; i < argc; ++i)
        context.g_command_line_words.emplace_back(argv[i]);

    // subcommand: slicer_cli layout capabilities --json
    if (argc >= 3 && std::string(argv[1]) == "layout" && std::string(argv[2]) == "capabilities") {
        if (argc != 4 || std::string(argv[3]) != "--json") {
            std::cerr << "Usage: " << argv[0] << " layout capabilities --json\n";
            // With --slice among the other words the refusal still leaves
            // result.json, as every refused --slice command line does (the
            // parse refusal below): the parser's own first pass finds --slice
            // and --outputdir in the words after the subcommand.
            // The subcommand's own --json is not a flag of the slice.
            std::vector<char*> rest{argv[0]};
            for (int i = 3; i < argc; ++i)
                if (!(i == 3 && std::string(argv[i]) == "--json"))
                    rest.push_back(argv[i]);
            CliOptions probe;
            slicer_cli::ModeArgs probe_mode;
            slicer_cli::ParseRefusal probe_refusal;
            (void) slicer_cli::parse_command_line((int)rest.size(), rest.data(), probe, probe_mode, probe_refusal);
            if (probe.slice_mode || probe.slice_given) {
                const std::string dir = probe.outputdir.empty() ? "." : probe.outputdir;
                boost::system::error_code mk;
                boost::filesystem::create_directories(dir, mk);
                context.g_result_json_options = &probe;
                context.g_result_json_mode = &probe_mode;
                write_result_json(context, dir, CLI_INVALID_PARAMS, probe.slice_mode ? std::max(0, probe.slice_plate) : 0,
                                  cli_error_sentence(CLI_INVALID_PARAMS) +
                                      " layout capabilities takes --json and nothing else; run the slice without it.",
                                  {}, 0, 0);
                return CLI_INVALID_PARAMS;
            }
            return 1;
        }
        layout_plan::install_cancellation_handler();  // ignore SIGPIPE so write failures surface as errors
        boost::log::core::get()->set_logging_enabled(false);
        return layout_plan::run_capabilities();
    }

    // A run that is killed cannot remove its staging folders, and each holds
    // up to the input's embedded G-code (the loader extracts
    // Metadata/plate_1.gcode into it: 175 MB on the OrcaSlicer build with the
    // corpus 44e597511c7d5abd.3mf). Bound a slice with `timeout` — SIGTERM —
    // or kill it, and on a tmpfs /tmp that is RAM left behind. The folders of
    // runs that are gone go now, before this run makes its own. No SIGTERM or
    // SIGHUP handler is installed to remove this run's: the run holds ~30
    // threads, and fork() in a handler (what an `rm -rf` child needs) runs
    // glibc's atfork handlers and takes the malloc arena locks, so a signal
    // that lands while any thread holds one blocks the handler on a futex for
    // ever — a plugin that ends a run with SIGTERM would then wait for ever.
    // The default action ends the run with 143/129, the rest of the corpus and
    // leakcheck's SIGTERM row leave the folder to the sweep above, and the
    // ownership marker's PID check reclaims it, exactly as after SIGKILL.
    sweep_stale_staging_folders();

    // Both engines read slice-time resources (info/, flush/, filament_mixing/)
    // relative to this root, which depends on no argument: resolve it once here
    // so the ENGINE_ORCA build gets it too, and so STL and calibration slices
    // stop falling back to the hardcoded tables.
    {
        // --layout-plan owns stderr as JSON documents, so the resources line
        // is left out there. Whether the command line asks for it is the
        // parser's own answer -- every spelling it takes (-layout-plan,
        // --layout-plan, an inline =value) -- from a dry parse; the parse the
        // run uses follows, and reports any refusal.
        CliOptions               probe;
        slicer_cli::ModeArgs     probe_mode;
        slicer_cli::ParseRefusal probe_refusal;
        (void) slicer_cli::parse_command_line(argc, argv, probe, probe_mode, probe_refusal);
        configure_engine_resources(argv[0], probe_mode.layout_plan_mode);
    }

    {
        slicer_cli::ParseRefusal refusal;
        context.g_result_json_options = &o;
        context.g_result_json_mode = &mode_args;
        if (!slicer_cli::parse_command_line(argc, argv, o, mode_args, refusal)) {
            std::cerr << "Error: " << refusal.message << "\n";
            if (refusal.with_usage) {
                std::cerr << "\n";
                print_usage(argv[0]);
            }
            // --slice always leaves result.json, also when its own value is
            // the one refused (--slice foo, --slice=-1): the official exits
            // with CLI_INVALID_PARAMS for a bad command line ("setup params
            // error", BambuStudio.cpp 1553-1558; OrcaSlicer.cpp 1249-1252),
            // and records an early exit's reason (BambuStudio.cpp 1697).
            if (o.slice_mode || o.slice_given) {
                const std::string dir = o.outputdir.empty() ? "." : o.outputdir;
                boost::system::error_code mk;
                boost::filesystem::create_directories(dir, mk);
                write_result_json(context, dir, refusal.code, o.slice_mode ? std::max(0, o.slice_plate) : 0,
                                  cli_error_sentence(refusal.code) + " " + refusal.message, {}, 0, 0);
                return refusal.code;
            }
            return 1;
        }
        context.g_command_line_parsed = true;
    }
    // --help: the flags. With --slice the run goes on: the official prints its
    // help as one action of its loop and runs the others, --slice among them
    // (BambuStudio.cpp 6337-6338; OrcaSlicer.cpp 5471-5472).
    if (mode_args.help) {
        print_usage(argv[0]);
        if (!o.slice_mode)
            return 0;
    }
    // A refusal before anything loads: under --slice it leaves result.json
    // in --outputdir too, as every --slice run does (the official records the
    // reason of an early exit the same way, record_exit_reson,
    // BambuStudio.cpp 1697; OrcaSlicer.cpp 1387). Without --slice: exit 1.
    auto refuse_run = [&](int code, const std::string& sentence) -> int {
        std::cerr << "Error: " << sentence << "\n";
        if (!o.slice_mode)
            return 1;
        const std::string dir = o.outputdir.empty() ? "." : o.outputdir;
        boost::system::error_code mk;
        boost::filesystem::create_directories(dir, mk);
        write_result_json(context, dir, code, o.slice_plate, cli_error_sentence(code) + " " + sentence, {}, 0, 0);
        return code;
    };
    if (o.verbose)
        Slic3r::set_logging_level(5);
    // --debug N: the engine's log level (BambuStudio.cpp 1661-1667; OrcaSlicer.cpp 1352-1359).
    if (o.given_flag("debug"))
        Slic3r::set_logging_level(o.cli.opt_int("debug"));
    // The arrange switches (read in arrange_on_bed).
    {
        auto flag = [&](const char* key, bool fallback) { return o.given_flag(key) ? o.cli.opt_bool(key) : fallback; };
        context.g_arrange_switches.enable_timelapse            = flag("enable_timelapse", false);
        context.g_arrange_switches.allow_rotations             = flag("allow_rotations", false);
        context.g_arrange_switches.allow_multicolor_oneplate   = flag("allow_multicolor_oneplate", true);
        context.g_arrange_switches.avoid_extrusion_cali_region = flag("avoid_extrusion_cali_region", false);
#ifdef ENGINE_BAMBU
        context.g_skip_useless_pick = flag("skip_useless_pick", false);
#endif
    }

    // Calibration: resolve the mode and params up front so a bad --calib-*
    // value fails fast with usage, before any model/config work.
    Slic3r::Calib_Params calib_params;
    try {
        calib_params = slicer_cli::build_calib_params(calib_opts);
    } catch (const std::exception& e) {
        return refuse_run(CLI_INVALID_PARAMS, e.what());
    }
    const bool calib_self_geometry = slicer_cli::calib_mode_generates_geometry(calib_params.mode);

#ifdef ENGINE_ORCA
    // pressure_advance_pattern's geometry generator is ported only for the Bambu
    // engine (Orca's CalibPressureAdvancePattern API differs); reject it cleanly
    // here so the Orca binary fails fast instead of throwing from apply_pa_pattern.
    if (calib_params.mode == Slic3r::CalibMode::Calib_PA_Pattern)
        return refuse_run(CLI_INVALID_PARAMS, "pressure_advance_pattern is not yet supported on the OrcaSlicer "
                                              "engine; use a tower or pressure_advance_line calib mode instead.");
#endif

    // --engine-info and --list-presets (slicer-cli's own, with no official
    // counterpart): one JSON document on stdout, nothing sliced. With --slice
    // they are refused, so a --slice run never ends without slicing or
    // result.json.
    if (o.slice_mode && !mode_args.engine_info_file.empty())
        return refuse_run(CLI_INVALID_PARAMS, "--engine-info describes a file and slices nothing; give it without --slice.");
    if (o.slice_mode && mode_args.list_presets)
        return refuse_run(CLI_INVALID_PARAMS, "--list-presets lists this engine's presets and slices nothing; give it "
                                              "without --slice.");
    if (o.slice_mode && mode_args.list_settings)
        return refuse_run(CLI_INVALID_PARAMS, "--list-settings lists this engine's settings and slices nothing; give "
                                              "it without --slice.");
    if (o.slice_mode && mode_args.list_flags)
        return refuse_run(CLI_INVALID_PARAMS, "--list-flags lists this program's flags and slices nothing; give it "
                                              "without --slice.");
    if (!mode_args.engine_info_file.empty()) {
        boost::log::core::get()->set_logging_enabled(false);
        return run_info(o.argv0, mode_args.engine_info_file, o.printer_preset);
    }

    if (mode_args.list_presets) {
        boost::log::core::get()->set_logging_enabled(false);
        return run_list_presets(o, mode_args.list_printer);
    }
    if (mode_args.list_settings) {
        boost::log::core::get()->set_logging_enabled(false);
        return run_list_settings();
    }
    if (mode_args.list_flags) {
        boost::log::core::get()->set_logging_enabled(false);
#ifdef ENGINE_SNAPMAKER
        const char* engine = "snapmaker";
#elif defined(ENGINE_ORCA)
        const char* engine = "orcaslicer";
#else
        const char* engine = "bambustudio";
#endif
        return slicer_cli::run_list_flags(engine, engine_version_text());
    }
    // --load-assemble-list builds the plates itself: no model files with it
    // (BambuStudio.cpp 1848-1853), and on OrcaSlicer no transforms either
    // (OrcaSlicer.cpp 1530-1535). It is the official plate loop's input, so
    // it runs with --slice; and as for any input that is not a Bambu-made
    // 3MF, every plate is sliced (plate_to_slice reset to 0,
    // BambuStudio.cpp 2192-2196; OrcaSlicer.cpp 1839-1843).
    if (!assemble_list_file(o).empty()) {
#ifdef ENGINE_ORCA
        if (!o.input_files.empty() || !o.transforms.empty())
            return refuse_run(CLI_INVALID_PARAMS, "--load-assemble-list builds the plates from its list; "
                                                  "give no model files and no transforms with it.");
#else
        if (!o.input_files.empty())
            return refuse_run(CLI_INVALID_PARAMS, "--load-assemble-list builds the plates from its list; "
                                                  "give no model files with it.");
#endif
        if (!o.slice_mode)
            return refuse_run(CLI_INVALID_PARAMS, "--load-assemble-list needs --slice: it slices the list's plates "
                                                  "into --outputdir.");
        if (o.slice_plate > 0) {
            emit_event({{"event","config_normalized"}, {"tag","AssembleListSlicesEveryPlate"},
                        {"requested_plate", o.slice_plate},
                        {"message","--load-assemble-list slices every plate of the list, as the official command "
                                   "line does for any input that is not a Bambu-made 3MF; --slice " +
                                   std::to_string(o.slice_plate) + " is taken as --slice 0"}});
            o.slice_plate = 0;
        }
    }
    // A project 3MF with model files after it (ProjectPlan): is_bbl_3mf ends
    // false, so the official slices every plate (plate_to_slice reset to 0,
    // BambuStudio.cpp 2192-2196; OrcaSlicer.cpp 1839-1843), after arranging
    // everything across the plates. --repetitions is then refused by its own
    // step, as for any --slice 0 (BambuStudio.cpp 4960-4964; OrcaSlicer.cpp
    // 4221-4225).
    if (assemble_list_file(o).empty() && trailing_models_port(o)) {
        if (o.slice_mode && o.slice_plate > 0) {
            emit_event({{"event","config_normalized"}, {"tag","ProjectWithModelsSlicesEveryPlate"},
                        {"requested_plate", o.slice_plate},
                        {"message","Model files after a project 3MF: every plate is sliced, as the official command "
                                   "line does once a file that is not a Bambu-made 3MF is loaded; --slice " +
                                   std::to_string(o.slice_plate) + " is taken as --slice 0"}});
            o.slice_plate = 0;
        }
    }
    // What the input is, in one answer: only a .3mf that OPENS and carries
    // Metadata/project_settings.config is a project. A .3mf that cannot be
    // read (missing, or not an archive) is its own state, never a project:
    // upstream checks every input's existence before it loads one, and a
    // failed load is the loader's to report (BambuStudio.cpp 1855-1860 at
    // 5873b5f; OrcaSlicer.cpp 1537-1542 at 31f6803). Calling it a project
    // here would refuse a named preset as CLI_INVALID_PARAMS ("a 3MF carries
    // its own settings") before the run could report CLI_FILE_NOTFOUND or
    // CLI_DATA_FILE_ERROR, as the same call does without a preset.
    const slicer_cli::ThreeMfKind input_3mf = slicer_cli::classify_3mf(input_file);
    const bool project_3mf_input = input_3mf == slicer_cli::ThreeMfKind::Project;
    // A 3MF with geometry only names no printer: the desktop asks for one,
    // the command line needs it named to slice. A run of model actions only
    // (--info, --export-stl, --export-stls, --export-settings without
    // --slice) needs none, as for an STL: the official runs those actions on
    // the loaded model (BambuStudio.cpp 6366-6401; OrcaSlicer.cpp 5499-5534).
    if (input_3mf == slicer_cli::ThreeMfKind::GeometryOnly && !o.uses_presets() &&
        !slicer_cli::model_actions_only(o) &&
        o.machine_config.empty() && o.bundle_config.empty() &&
        o.cli.option<Slic3r::ConfigOptionStrings>("load_settings")->values.empty())
        return refuse_run(CLI_INVALID_PARAMS,
            input_file + " holds geometry only, with no printer or print settings. Name the printer with "
            "--printer-preset (and optionally --process-preset and --filament-preset), or give settings "
            "files with --load-settings.");
    if (o.uses_presets()) {
        int code = 0;
        std::string error;
        bool resolved = false;
        if (project_3mf_input) {
            // --printer-preset on a project 3MF: the desktop app's printer
            // switch (stage_named_presets_for_project). The presets named are
            // written out in full for the merge --load-settings already runs.
            const std::string dir = context.run_staging.add((boost::filesystem::temp_directory_path() /
                                                     boost::filesystem::unique_path("slicer_cli_preset-%%%%%%%%")).string());
            resolved = stage_named_presets_for_project(o, dir, code, error);
        } else {
            context.g_preset_config = std::make_unique<Slic3r::DynamicPrintConfig>();
            resolved = resolve_named_presets(context, o, *context.g_preset_config, code, error);
        }
        if (!resolved) {
            emit_event({{"event","preset_error"}, {"tag","NamedPresetRefused"}, {"message", error}});
            std::cerr << "Error: " << error << "\n";
            if (o.slice_mode) {
                boost::system::error_code mk;
                boost::filesystem::create_directories(o.outputdir.empty() ? "." : o.outputdir, mk);
                write_result_json(context, o.outputdir.empty() ? "." : o.outputdir, code, o.slice_plate,
                                  cli_error_sentence(code) + " " + error, {}, 0, 0);
                return code;
            }
            return 1;
        }
    }
    if (!o.export_3mf.empty() && !o.slice_mode) {
        std::cerr << "Error: --export-3mf needs --slice\n";
        return 1;
    }
    if (const std::string why = slicer_cli::simplify_flags_refusal(o); !why.empty())
        return refuse_run(CLI_INVALID_PARAMS, why);
    if (o.arrange_forced() && !o.slice_mode) {
        std::cerr << "Error: --arrange needs --slice (the official CLI arranges inside its plate loop)\n";
        return 1;
    }
    if (o.slice_mode)
        o.progress = true;   // --slice reports progress like the official --pipe
    if (o.slice_mode && plate_id > 0)
        return refuse_run(CLI_INVALID_PARAMS, "--plate and --slice are mutually exclusive; use --slice N for plate N");

    // Detect conflicting layout flags. Under --slice this refusal must still
    // leave result.json like every other early refusal (refuse_run); without
    // --slice it prints the same sentence and exits 1, unchanged. The owner
    // rule's "what to do" part goes on the result.json path alone: the
    // sentence the no---slice call prints is the one that was always there.
    if (layout_plan_mode && !layout_json_file.empty())
        return refuse_run(CLI_INVALID_PARAMS, o.slice_mode
                              ? "--layout-plan and --layout are mutually exclusive; give one of them"
                              : "--layout-plan and --layout are mutually exclusive");

    // Both layout modes arrange and return, before any slice: like
    // --engine-info and --list-presets they write no result.json, so with
    // --slice they are refused too, and a --slice run never ends without
    // slicing or result.json. Without --slice neither line below is reached.
    if (o.slice_mode && layout_plan_mode)
        return refuse_run(CLI_INVALID_PARAMS, "--layout-plan arranges the input's objects and slices nothing; give "
                                              "it without --slice.");
    if (o.slice_mode && !layout_json_file.empty())
        return refuse_run(CLI_INVALID_PARAMS, "--layout arranges the model files and slices nothing; give it "
                                              "without --slice.");

    // --layout-plan: versioned headless arrange contract
    if (layout_plan_mode) {
        layout_plan::install_cancellation_handler();  // before input read: honor SIGINT during parse
        json raw;
        std::string input_data;
        int fd = -1;
        if (!input_file.empty()) {
            // opening a FIFO blocks until a writer connects; loop on EINTR and
            // treat it as a cancellation check point
            bool cancelled = false;
            for (;;) {
                if (layout_plan::is_cancelled()) { cancelled = true; break; }
#ifdef _WIN32
                // The path is UTF-8 (boost::nowide::args): open it by its wide form.
                fd = ::_wopen(boost::nowide::widen(input_file).c_str(), _O_RDONLY | _O_BINARY);  // binary: no Ctrl+Z EOF, no newline translation
#else
                fd = ::open(input_file.c_str(), O_RDONLY);
                if (fd < 0 && errno == EINTR) {
                    if (layout_plan::is_cancelled()) { cancelled = true; break; }
                    continue;
                }
#endif
                break;
            }
            if (cancelled) {
                std::cerr << json{{"schemaVersion",1},{"error",{{"code","CANCELLED"},{"message","cancelled during input open"}}}}.dump() << std::endl;
                return 5;
            }
            if (fd < 0) {
                std::cerr << json{{"schemaVersion",1},{"error",{{"code","INVALID_INPUT"},{"message","cannot open --input file"}}}}.dump() << std::endl;
                return 3;
            }
        } else {
            fd = 0;  // stdin
        }
        // route --input through the same cancellable fd read loop as stdin so
        // FIFOs/slow streams observe SIGINT/Ctrl+C with a bounded exit
        int rrc = read_all_cancellable(fd, input_data);
        if (fd > 0) {
#ifdef _WIN32
            ::_close(fd);
#else
            ::close(fd);
#endif
        }
        if (rrc == 1) {
            std::cerr << json{{"schemaVersion",1},{"error",{{"code","CANCELLED"},{"message","cancelled during input read"}}}}.dump() << std::endl;
            return 5;
        }
        if (rrc == 2) {
            std::cerr << json{{"schemaVersion",1},{"error",{{"code","INVALID_INPUT"},{"message","failed to read input stream"}}}}.dump() << std::endl;
            return 3;
        }
        try { raw = json::parse(input_data); } catch (const std::exception& e) {
            if (layout_plan::is_cancelled()) {  // SIGINT during the parse → cancel, not parse-error
                std::cerr << json{{"schemaVersion",1},{"error",{{"code","CANCELLED"},{"message","cancelled during input read"}}}}.dump() << std::endl;
                return 5;
            }
            std::cerr << json{{"schemaVersion",1},{"error",{{"code","INVALID_INPUT"},{"message",std::string("JSON parse error: ")+e.what()}}}}.dump() << std::endl;
            return 3;
        }
        if (layout_plan::is_cancelled()) {  // SIGINT during a large parse → CANCELLED, no continued work
            std::cerr << json{{"schemaVersion",1},{"error",{{"code","CANCELLED"},{"message","cancelled during input read"}}}}.dump() << std::endl;
            return 5;
        }
        layout_plan::LayoutProblemV1 problem;
        layout_plan::LayoutErrorV1   parse_err;
        if (!layout_plan::parse_input(raw, problem, parse_err)) {
            json err_json = {
                {"schemaVersion", parse_err.SCHEMA_VERSION},
                {"error", {
                    {"code",    parse_err.error.code},
                    {"message", parse_err.error.message}
                }}
            };
            if (!parse_err.error.object_ids.empty())
                err_json["error"]["object_ids"] = parse_err.error.object_ids;
            std::cerr << err_json.dump() << std::endl;
            return parse_err.error.code == "CANCELLED" ? 5 : 3;
        }
        boost::log::core::get()->set_logging_enabled(false);
        return layout_plan::run_layout_plan(problem);
    }

    // --layout: headless arrange from the older JSON form
    if (!layout_json_file.empty()) {
        boost::nowide::ifstream lf(layout_json_file);
        if (!lf.is_open()) { std::cerr << "Cannot open layout JSON: " << layout_json_file << "\n"; return 1; }
        json lj;
        try { lj = json::parse(lf); } catch (const std::exception& e) {
            std::cerr << "Failed to parse layout JSON: " << e.what() << "\n"; return 1;
        }
        std::string profiles_dir = lj.value("profilesDir", "");
        while (!profiles_dir.empty() && profiles_dir.back() == '/') profiles_dir.pop_back();

        Slic3r::DynamicPrintConfig cfg;
        if (lj.contains("profiles")) {
            if (profiles_dir.empty()) {
                std::cerr << "profilesDir is required when profiles is specified\n";
                return 1;
            }
            for (auto& [kind, path] : lj["profiles"].items()) {
                if (!load_json_config(profiles_dir + "/" + path.get<std::string>(), cfg)) {
                    std::cerr << "Failed to load profile: " << path.get<std::string>() << "\n";
                    return 1;
                }
                // The profile keeps its own "inherits" chain: the arrange reads
                // the printer's clearances, which a shipped machine preset
                // leaves to its parents.
                apply_file_inherits(argv[0] ? argv[0] : "slicer_cli", profiles_dir + "/" + path.get<std::string>(),
                                    kind.c_str(), cfg, false);
            }
        }
        using namespace Slic3r;
        using namespace Slic3r::arrangement;

        ArrangeParams params;
#ifdef ENGINE_ORCA
        params.clearance_radius = cfg.has("extruder_clearance_max_radius") ? cfg.opt_float("extruder_clearance_max_radius") : 1.0f;
        if (params.clearance_radius < 1.0f) params.clearance_radius = lj.value("clearanceRadiusMm", 68.0f);
#else
        params.cleareance_radius = cfg.has("extruder_clearance_max_radius") ? cfg.opt_float("extruder_clearance_max_radius") : 1.0f;
        if (params.cleareance_radius < 1.0f) params.cleareance_radius = lj.value("clearanceRadiusMm", 68.0f);
#endif

        // Load all STLs into a single Model (one ModelObject per STL)
        Model model;
        if (!lj.contains("objects") || !lj["objects"].is_array()) {
            std::cerr << "objects must be a JSON array\n";
            return 1;
        }
        for (auto& obj : lj["objects"]) {
            if (!obj.is_object()) {
                std::cerr << "each object entry must be a JSON object\n";
                return 1;
            }
            std::string stl_path = obj.value("stl", "");
            if (stl_path.empty()) continue;
            try {
                Model m = Model::read_from_file(stl_path);
                for (ModelObject* mo : m.objects) {
                    ModelObject* new_obj = model.add_object(*mo);
                    if (new_obj->instances.empty())
                        new_obj->add_instance();
                }
            } catch (const std::exception& e) {
                std::cerr << "Failed to load " << stl_path << ": " << e.what() << "\n";
                return 1;
            }
        }
        params.min_obj_distance = scaled<coord_t>(lj.value("spacingMm", 10.0));
        params.allow_rotations = lj.value("allowRotations", true);
        params.do_final_align = lj.value("doFinalAlign", true);

        // Use get_arrange_polys -> arrange pipeline (same as GUI ArrangeJob)
        ModelInstancePtrs instances;
        auto input = get_arrange_polys(model, instances);
        // get_arrange_polygon leaves bed_idx at its UNARRANGED (-1) default,
        // which the nester skips as BIN_ID_UNFIT. Seed bed 0 as the desktop
        // does (ModelArrange.cpp:98, `ap.bed_idx = 0;` in get_arrange_poly;
        // same in Orca v2.4.2 src/libslic3r/ModelArrange.cpp:25).
        for (auto& ap : input) ap.bed_idx = 0;

#ifdef ENGINE_ORCA
        update_arrange_params(params, &cfg, input);
        update_selected_items_inflation(input, &cfg, params);
        Points bed_pts = get_shrink_bedpts(&cfg, params);
#else
        update_arrange_params(params, cfg, input);
        update_selected_items_inflation(input, cfg, params);
        Points bed_pts = get_shrink_bedpts(cfg, params);
#endif
        arrangement::arrange(input, {}, bed_pts, params);

        // Apply results back to model instances
        apply_arrange_polys(input, instances, [](ArrangePolygon&) {});

        json out;
        out["engine"] =
        #ifdef ENGINE_SNAPMAKER
            "snapmaker";
        #elif defined(ENGINE_ORCA)
            "orca";
        #else
            "bambu";
        #endif
        out["placements"] = json::array();
        for (auto& ap : input) {
            json p;
            p["name"] = ap.name;
            p["bed_idx"] = ap.bed_idx;
            p["x_mm"] = unscaled<double>(ap.translation.x());
            p["y_mm"] = unscaled<double>(ap.translation.y());
            p["rotation_deg"] = ap.rotation * 180.0 / M_PI;
            BoundingBox bb = ap.transformed_poly().contour.bounding_box();
            p["cx_mm"] = unscaled<double>(bb.min.x() + bb.max.x()) / 2.0;
            p["cy_mm"] = unscaled<double>(bb.min.y() + bb.max.y()) / 2.0;
            out["placements"].push_back(p);
        }
        std::cout << out.dump() << std::endl;
        return 0;

    }
    // ── Structured-diagnostics install point ───────────────────────────────
    // Install only on the slicing path. Capabilities and --layout-plan require
    // one JSON document. Legacy --layout can include engine text before its
    // result; keep its existing output free of structured diagnostic events.
    install_engine_log_bridge();

    if (input_file.empty() && !calib_self_geometry && assemble_list_file(o).empty()) {
        if (o.slice_mode)
            return refuse_run(CLI_INVALID_PARAMS, "No input file specified");
        std::cerr << "Error: No input file specified\n\n";
        print_usage(argv[0]);
        return 1;
    }
    if (calib_self_geometry) {
        // pressure_advance_pattern DISCARDS the loaded model and generates its own
        // geometry, but it still needs a fully-resolved printer/filament/process
        // config AND the 3MF's per-plate custom-gcode scaffolding. Require a real
        // .3mf --input: a profile bundle alone is insufficient (its files load
        // later with only a warning on failure, and no plate metadata is set up,
        // so the pattern would silently slice the default/wrong printer). An STL
        // supplies only geometry, which the pattern throws away.
        //
        // The loader's own test (input_is_3mf: the final extension), so a
        // path like `part.3mf.stl` — which loads as STL — is NOT a 3MF here.
        if (!input_is_3mf(input_file)) {
            const std::string why = "pressure_advance_pattern requires a .3mf --input for its "
                                    "printer/filament config (it discards the model geometry but reads the "
                                    "embedded config + plate setup; a profile bundle or STL is not "
                                    "sufficient)";
            if (o.slice_mode)
                return refuse_run(CLI_INVALID_PARAMS, why);
            std::cerr << "Error: " << why << "\n\n";
            print_usage(argv[0]);
            return 1;
        }
    }

    std::cout << "libslic3r_standalone - Standalone slicing tool\n";
    std::cout << "Based on BambuStudio libslic3r\n\n";

    // --pipe NAME (Linux; BambuStudio.cpp 1758-1769, OrcaSlicer.cpp
    // 1441-1451): the progress pipe, from the start of the run.
    if (o.given_flag("pipe") && !o.cli.opt_string("pipe").empty() && slicer_cli::pipe_supported()) {
        slicer_cli::pipe_start(o.cli.opt_string("pipe"));
        slicer_cli::pipe_update(1, "Start to load files");
    }

    return run_cli_slice(context, o, calib_params);
}
