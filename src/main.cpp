// libslic3r_standalone - CLI tool for slicing with libslic3r
// Supports loading BambuStudio JSON config files (machine, filament, process)

#include "cli_startup.hpp"
#include "cli_slice_run.hpp"
#include "cli_assemble.hpp"
#include "cli_command_line.hpp"
#include "cli_run_context.hpp"
#include "cli_events.hpp"
#include "cli_pipe.hpp"

#include "libslic3r/Thread.hpp"
#include <boost/nowide/args.hpp>
#include <iostream>

using slicer_cli::CliRunContext;
using slicer_cli::install_engine_log_bridge;
using slicer_cli::run_cli_slice;



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
    if (auto result = slicer_cli::run_timing_or_initialize_logging(argc, argv))
        return *result;

    // Parse arguments: the official flags through this engine's own CLI
    // definitions, slicer-cli's own flags beside them (cli_command_line.cpp).
    CliOptions o;
    slicer_cli::ModeArgs mode_args;
    if (auto result = slicer_cli::parse_startup_options(context, argc, argv, o, mode_args))
        return *result;

    slicer_cli::configure_command_logging(context, o);

    // Calibration: resolve the mode and params up front so a bad --calib-*
    // value fails fast with usage, before any model/config work.
    Slic3r::Calib_Params calib_params;
    bool calib_self_geometry;
    if (auto result = slicer_cli::check_calibration_options(context, o, mode_args.calib, calib_params, calib_self_geometry))
        return *result;

    if (auto result = slicer_cli::run_information_commands(context, o, mode_args))
        return *result;
    if (auto result = slicer_cli::check_assemble_input(context, o))
        return *result;
    if (auto result = slicer_cli::prepare_input_presets(context, o))
        return *result;
    if (auto result = slicer_cli::run_layout_commands(context, argv, o, mode_args))
        return *result;
    // ── Structured-diagnostics install point ───────────────────────────────
    // Install only on the slicing path. Capabilities and --layout-plan require
    // one JSON document. Legacy --layout can include engine text before its
    // result; keep its existing output free of structured diagnostic events.
    install_engine_log_bridge();

    if (auto result = slicer_cli::check_slice_input(context, argv, o, calib_self_geometry))
        return *result;

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
