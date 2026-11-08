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

const char *engine_name() {
#ifdef ENGINE_SNAPMAKER
    return "SNAPMAKER";
#elif defined(ENGINE_ORCA) && !defined(ENGINE_SNAPMAKER)
    return "ORCA";
#else
    return "BAMBU";
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

} // namespace

bool run_gcode_timing_command(int argc, char **argv, int &exit_code) {
    if (!gcode_timing_requested(argc, argv))
        return false;

    // Timing deliberately has no slicing/config/output/probe switches.  Its
    // two files are imported only, and accepting any other argument would make
    // the selected native importer context ambiguous.
    if (std::string(argv[1]) != "--gcode-timing") {
        write_error("CONFLICTING_ARGUMENTS");
        exit_code = 64;
        return true;
    }

#ifdef ENGINE_ORCA
    // OrcaSlicer Plater.cpp:13663 and Snapmaker_Orca Plater.cpp:19114 take
    // this from the selected desktop preset. Both require that context when
    // the file has no embedded printer model; never default it to BBL.
    if (argc == 4) {
        write_error("PRINTER_CONTEXT_REQUIRED");
        exit_code = 64;
        return true;
    }
    if (argc != 6 || std::string(argv[4]) != "--printer-vendor-bbl" ||
        (std::string(argv[5]) != "true" && std::string(argv[5]) != "false")) {
        write_error("CONFLICTING_ARGUMENTS");
        exit_code = 64;
        return true;
    }
    const bool printer_vendor_bbl = std::string(argv[5]) == "true";
#else
    if (argc != 4) {
        write_error("CONFLICTING_ARGUMENTS");
        exit_code = 64;
        return true;
    }
#endif

    OutputSilencer silence;
    if (!silence.active()) {
        silence.restore();
        write_error("UNAVAILABLE");
        exit_code = 1;
        return true;
    }

    try {
        boost::nowide::nowide_filesystem();
        // The native Bambu importer can dereference a null file handle on an
        // unreadable path. Reject missing, empty, and non-regular inputs before
        // either native import; this mode must never block reading a FIFO.
        if (!readable_input(argv[2]) || !readable_input(argv[3]))
            throw std::runtime_error("unreadable input");
        double original_seconds = 0.0;
        double optimized_seconds = 0.0;
#ifndef ENGINE_ORCA
        // BambuStudio GCode.cpp: the desktop carries this result from an
        // original import into its optimized import.  The Bambu pin exposes
        // that handoff; neither Orca-derived pin does.
        std::shared_ptr<Slic3r::MultiNozzleUtils::NozzleGroupResultBase> nozzle_group_result;
        {
            Slic3r::GCodeProcessor original;
            original.init_filament_maps_when_import_only_gcode();
            original.process_file(argv[2]);
            original_seconds = normal_seconds(original);
            nozzle_group_result = original.get_result().nozzle_group_result;
        }
        {
            Slic3r::GCodeProcessor optimized;
            optimized.initialize_from_context(nozzle_group_result);
            optimized.process_file(argv[3]);
            optimized_seconds = normal_seconds(optimized);
        }
#elif defined(ENGINE_ORCA) && !defined(ENGINE_SNAPMAKER)
        // OrcaSlicer Plater.cpp:13662-13667 imports a G-code with a fresh
        // processor and this initializer.  s_IsBBLPrinter is selected there
        // from the GUI's already-selected preset; process_file also reads an
        // embedded printer_model where the G-code supplies one. This command
        // adds no inferred printer mapping.
        {
            Slic3r::GCodeProcessor original;
            original.init_filament_maps_and_nozzle_type_when_import_only_gcode();
            Slic3r::GCodeProcessor::s_IsBBLPrinter = printer_vendor_bbl;
            original.process_file(argv[2]);
            original_seconds = normal_seconds(original);
        }
        {
            Slic3r::GCodeProcessor optimized;
            optimized.init_filament_maps_and_nozzle_type_when_import_only_gcode();
            Slic3r::GCodeProcessor::s_IsBBLPrinter = printer_vendor_bbl;
            optimized.process_file(argv[3]);
            optimized_seconds = normal_seconds(optimized);
        }
#else
        // Snapmaker_Orca Plater.cpp:19111-19115 imports with a fresh processor
        // and process_file alone; this pin has no Orca initializer. Its desktop
        // preset context is required above, never inferred from either path.
        {
            Slic3r::GCodeProcessor original;
            Slic3r::GCodeProcessor::s_IsBBLPrinter = printer_vendor_bbl;
            original.process_file(argv[2]);
            original_seconds = normal_seconds(original);
        }
        {
            Slic3r::GCodeProcessor optimized;
            Slic3r::GCodeProcessor::s_IsBBLPrinter = printer_vendor_bbl;
            optimized.process_file(argv[3]);
            optimized_seconds = normal_seconds(optimized);
        }
#endif
        silence.restore();
        write_success(original_seconds, optimized_seconds);
        exit_code = 0;
    } catch (const std::exception &) {
        silence.restore();
        write_error("TIMING_UNAVAILABLE");
        exit_code = 1;
    } catch (...) {
        silence.restore();
        write_error("TIMING_UNAVAILABLE");
        exit_code = 1;
    }
    return true;
}

} // namespace slicer_cli
