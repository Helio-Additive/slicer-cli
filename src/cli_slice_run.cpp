#include "cli_slice_run.hpp"
#include "diagnostic_output.hpp"

#include "cli_run_context.hpp"
#include "cli_slice_state.hpp"
#include "cli_options.hpp"
#include "cli_assemble.hpp"
#include <boost/filesystem.hpp>
#include <iostream>
#include <chrono>
#include "cli_engine_events.hpp"
#include "cli_model_load.hpp"
#include "cli_pipe.hpp"
#include "cli_plate_plan.hpp"
#include "cli_project_archive.hpp"
#include "cli_result_json.hpp"
#include "cli_run_steps.hpp"
#include "cli_setting_overrides.hpp"
#include "cli_slice_export.hpp"
#include "cli_slice_plate.hpp"
#include "cli_snapmaker.hpp"

namespace slicer_cli {

using json = nlohmann::json;

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

int run_cli_slice(CliRunContext& context, const CliOptions& o, Slic3r::Calib_Params& calib_params) {
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

} // namespace slicer_cli
