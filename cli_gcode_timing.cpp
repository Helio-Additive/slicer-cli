// cli_gcode_timing.cpp -- native import timing, never slicing or rewriting.
#include "cli_gcode_timing.hpp"
#include "cli_command_line.hpp"

#include <cmath>
#include <cstdio>
#include <exception>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <boost/filesystem.hpp>
#include <boost/nowide/filesystem.hpp>
#include <boost/nowide/fstream.hpp>
#include <nlohmann/json.hpp>

#include "libslic3r/GCode/GCodeProcessor.hpp"

#if defined(_WIN32)
#include <fcntl.h>
#include <io.h>
#else
#include <fcntl.h>
#include <unistd.h>
#endif

namespace slicer_cli {
namespace {

using json = nlohmann::json;

// Importing a file can emit desktop diagnostics through stdout/stderr.  This
// mode is a machine contract, so keep the engine's diagnostics private and
// restore the streams before writing its one response document.
class OutputSilencer {
  public:
    OutputSilencer() {
        flush_streams();
#if defined(_WIN32)
        m_stdout = duplicate_saved_fd(_fileno(stdout));
        m_stderr = duplicate_saved_fd(_fileno(stderr));
        m_null = _open("NUL", _O_WRONLY);
#else
        m_stdout = duplicate_saved_fd(STDOUT_FILENO);
        m_stderr = duplicate_saved_fd(STDERR_FILENO);
        m_null = open("/dev/null", O_WRONLY);
#endif
        if (m_stdout < 0 || m_stderr < 0 || m_null < 0) {
            close_saved();
            return;
        }
#if defined(_WIN32)
        if (_dup2(m_null, _fileno(stdout)) < 0) {
#else
        if (dup2(m_null, STDOUT_FILENO) < 0) {
#endif
            close_saved();
            return;
        }
        m_stdout_redirected = true;
#if defined(_WIN32)
        if (_dup2(m_null, _fileno(stderr)) < 0) {
#else
        if (dup2(m_null, STDERR_FILENO) < 0) {
#endif
            restore();
            return;
        }
        m_stderr_redirected = true;
        m_active = true;
    }

    ~OutputSilencer() { restore(); }

    bool active() const { return m_active; }

    void restore() {
        if (m_stdout_redirected || m_stderr_redirected)
            flush_streams();
#if defined(_WIN32)
        if (m_stdout_redirected)
            _dup2(m_stdout, _fileno(stdout));
        if (m_stderr_redirected)
            _dup2(m_stderr, _fileno(stderr));
#else
        if (m_stdout_redirected)
            dup2(m_stdout, STDOUT_FILENO);
        if (m_stderr_redirected)
            dup2(m_stderr, STDERR_FILENO);
#endif
        m_stdout_redirected = false;
        m_stderr_redirected = false;
        m_active = false;
        close_saved();
    }

  private:
    // Saved streams must never occupy stdout/stderr: a closed descriptor can
    // otherwise make dup(stdout) return 2, which the stderr redirect then
    // overwrites before restore writes the JSON response.
    static int duplicate_saved_fd(int fd) {
#if defined(_WIN32)
        // _dup() may successively fill every closed standard descriptor. Keep
        // each low duplicate live until a later duplicate is above them: with
        // both stdin and stderr closed, closing fd 0 before the next _dup()
        // would simply allocate fd 0 again and let the later stderr redirect
        // overwrite the saved stdout at fd 2.
        std::vector<int> low_duplicates;
        for (;;) {
            const int saved = _dup(fd);
            if (saved < 0) {
                for (const int low : low_duplicates)
                    _close(low);
                return -1;
            }
            if (saved > 2) {
                for (const int low : low_duplicates)
                    _close(low);
                return saved;
            }
            low_duplicates.push_back(saved);
        }
#else
        return fcntl(fd, F_DUPFD, STDERR_FILENO + 1);
#endif
    }

    static void flush_streams() {
        std::cout.flush();
        std::cerr.flush();
        std::fflush(stdout);
        std::fflush(stderr);
    }

    void close_saved() {
#if defined(_WIN32)
        if (m_stdout >= 0)
            _close(m_stdout);
        if (m_stderr >= 0)
            _close(m_stderr);
        if (m_null >= 0)
            _close(m_null);
#else
        if (m_stdout >= 0)
            close(m_stdout);
        if (m_stderr >= 0)
            close(m_stderr);
        if (m_null >= 0)
            close(m_null);
#endif
        m_stdout = -1;
        m_stderr = -1;
        m_null = -1;
    }

    int m_stdout = -1;
    int m_stderr = -1;
    int m_null = -1;
    bool m_active = false;
    bool m_stdout_redirected = false;
    bool m_stderr_redirected = false;
};

// The program names result.json and --engine-info already use (main.cpp
// 3350-3354 and 6358-6362), so a caller correlates the documents without a
// mapping table.
const char *engine_name() {
#ifdef ENGINE_SNAPMAKER
    return "snapmaker";
#elif defined(ENGINE_ORCA)
    return "orcaslicer";
#else
    return "bambustudio";
#endif
}

const char *engine_revision() {
#ifdef ENGINE_COMMIT
    return ENGINE_COMMIT;
#else
    return "unknown";
#endif
}

void write_error(const char *code) {
    std::cout << json{{"schema_version", 1},
                      {"engine", engine_name()},
                      {"engine_revision", engine_revision()},
                      {"error_code", code}}
                     .dump()
              << '\n';
}

void write_success(double original_seconds, double optimized_seconds) {
    std::cout << json{{"schema_version", 1},
                      {"engine", engine_name()},
                      {"engine_revision", engine_revision()},
                      {"original_seconds", original_seconds},
                      {"optimized_seconds", optimized_seconds}}
                     .dump()
              << '\n';
}

// The native Bambu importer can dereference a null file handle on an
// unreadable path, and a FIFO would block the import.  Only a non-empty
// regular file reaches either importer.
bool readable_input(const char *path) {
    boost::system::error_code error;
    if (!boost::filesystem::is_regular_file(path, error) || error)
        return false;
    boost::nowide::ifstream input(path, std::ios::binary);
    return input && input.peek() != std::char_traits<char>::eof();
}

double normal_seconds(const Slic3r::GCodeProcessor &processor) {
    const auto &mode =
        processor.get_result()
            .print_statistics.modes[static_cast<std::size_t>(Slic3r::PrintEstimatedStatistics::ETimeMode::Normal)];
    if (!std::isfinite(mode.time) || mode.time <= 0.0)
        throw std::runtime_error("no normal timing result");
    return mode.time;
}

#ifndef ENGINE_ORCA
using NozzleGroup = std::shared_ptr<Slic3r::MultiNozzleUtils::NozzleGroupResultBase>;

// One import the way the Bambu desktop does it.  A plain import is Plater.cpp
// 25408-25413 at the pin: a fresh processor, the import-only filament maps,
// process_file.  The optimized file of a pair follows HelioDragon.cpp
// 3061-3067 (load_helio_file_to_viwer): the processor is seeded with the
// plate's nozzle grouping when the plate has one and imports unseeded
// otherwise.  GCodeProcessor.cpp 2734-2736 clears that grouping for a G-code
// from an unknown producer, so a null seed is an expected case.
double time_file(const char *path, const NozzleGroup &seed, NozzleGroup *grouping_out) {
    Slic3r::GCodeProcessor processor;
    if (seed)
        processor.initialize_from_context(seed);
    else
        processor.init_filament_maps_when_import_only_gcode();
    processor.process_file(path);
    if (grouping_out)
        *grouping_out = processor.get_result().nozzle_group_result;
    return normal_seconds(processor);
}
#else
// One import the way the Orca-derived desktops do it.  OrcaSlicer Plater.cpp
// 13662-13667 at the Orca pin: a fresh processor, init_filament_maps_and_
// nozzle_type_when_import_only_gcode, s_IsBBLPrinter from the selected
// preset's vendor, process_file.  Snapmaker_Orca Plater.cpp 19111-19115 at
// its pin is the same without the initializer, which that pin lacks.  Both
// desktops take the vendor flag from the selected printer preset; this command
// takes it from --printer-vendor-bbl and never infers it.  process_file may
// refine the flag from an embedded printer_model (Orca GCodeProcessor.cpp
// 2632), so it is set again before each file.
double time_file(const char *path, bool printer_vendor_bbl) {
    Slic3r::GCodeProcessor processor;
#ifndef ENGINE_SNAPMAKER
    processor.init_filament_maps_and_nozzle_type_when_import_only_gcode();
#endif
    Slic3r::GCodeProcessor::s_IsBBLPrinter = printer_vendor_bbl;
    processor.process_file(path);
    return normal_seconds(processor);
}
#endif

} // namespace

int write_gcode_timing_unavailable() {
    write_error("UNAVAILABLE");
    return 1;
}

int run_gcode_timing_command(int argc, char **argv) {
    // Timing deliberately has no slicing/config/output/probe switches.  Its
    // two files are imported only, and accepting any other argument would make
    // the selected native importer context ambiguous.
    if (std::string(argv[1]) != "--gcode-timing") {
        write_error("CONFLICTING_ARGUMENTS");
        return 64;
    }

#ifdef ENGINE_ORCA
    if (argc == 4) {
        write_error("PRINTER_CONTEXT_REQUIRED");
        return 64;
    }
    if (argc != 6 || std::string(argv[4]) != "--printer-vendor-bbl" ||
        (std::string(argv[5]) != "true" && std::string(argv[5]) != "false")) {
        write_error("CONFLICTING_ARGUMENTS");
        return 64;
    }
    const bool printer_vendor_bbl = std::string(argv[5]) == "true";
#else
    if (argc != 4) {
        write_error("CONFLICTING_ARGUMENTS");
        return 64;
    }
#endif

    // A failing nowide_filesystem() is the caller's environment, as main()
    // treats it (CLI_ENVIRONMENT_ERROR); a bad file is the caller's input.
    // Each gets its own code so an agent knows what to fix.
    try {
        boost::nowide::nowide_filesystem();
    } catch (const std::exception &) {
        write_error("UNAVAILABLE");
        return 1;
    }
    if (!readable_input(argv[2]) || !readable_input(argv[3])) {
        write_error("INPUT_UNREADABLE");
        return 1;
    }

    OutputSilencer silence;
    if (!silence.active()) {
        silence.restore();
        write_error("UNAVAILABLE");
        return 1;
    }

    try {
        double original_seconds = 0.0;
        double optimized_seconds = 0.0;
#ifndef ENGINE_ORCA
        NozzleGroup grouping;
        original_seconds = time_file(argv[2], nullptr, &grouping);
        optimized_seconds = time_file(argv[3], grouping, nullptr);
#else
        original_seconds = time_file(argv[2], printer_vendor_bbl);
        optimized_seconds = time_file(argv[3], printer_vendor_bbl);
#endif
        silence.restore();
        write_success(original_seconds, optimized_seconds);
        return 0;
    } catch (...) {
        // The importer's diagnostics stayed private; the code is the contract.
        silence.restore();
        write_error("TIMING_UNAVAILABLE");
        return 1;
    }
}

} // namespace slicer_cli
