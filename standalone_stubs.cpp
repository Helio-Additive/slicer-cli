// standalone_stubs.cpp
// Stub implementations for GUI-dependent functions used by libslic3r
// These are no-op or minimal implementations for standalone CLI builds

#include <string>
#include <functional>
#ifdef ENGINE_BAMBU
// LogSink is a BambuStudio-only facility (encrypted remote logging). OrcaSlicer
// has no LogSink.hpp / LogSink.cpp, and nothing in its libslic3r references it.
#include "libslic3r/LogSink.hpp"
#endif

namespace Slic3r {

// Stub for macOS ModelIO temp file creation
// In GUI this uses macOS ModelIO framework to convert USD/USDZ to STL
std::string make_temp_stl_with_modelio(const std::string& path) {
    // Return empty string - file format not supported in standalone
    return "";
}

// Stub for temp file deletion
void delete_temp_file(const std::string& temp_file) {
    // In standalone build, we don't create temp files from ModelIO
    // so nothing to delete
}

// Stub for macOS Boost logging support check
bool is_macos_support_boost_add_file_log() {
    // Always return false in standalone - we don't need encrypted remote logging
    return false;
}


#ifdef ENGINE_ORCA
// Format/DRC.cpp (Google-Draco .drc reader) is excluded from the Orca build to
// avoid the external draco dependency, but Model.cpp's read_from_file references
// load_drc(). The .3mf→gcode path never loads a .drc, so stub both overloads.
// Pointer parameters only need forward declarations (we're already in namespace Slic3r).
class TriangleMesh;
class Model;
bool load_drc(const char* /*path*/, TriangleMesh* /*meshptr*/) { return false; }
bool load_drc(const char* /*path*/, Model* /*model*/, const char* /*object_name*/) { return false; }
#endif // ENGINE_ORCA

#ifdef ENGINE_BAMBU
// LogSinkBackend implementations - we excluded LogSink.cpp but utils.cpp still references it
// (BambuStudio only — OrcaSlicer has no LogSink).
LogSinkBackend::LogSinkBackend(const std::string& base_path, const LogEncOptions& options)
    : boost::log::sinks::text_file_backend(), m_log_enc_options(options) {
    // Minimal initialization - no encryption in standalone
}

bool LogSinkBackend::update_enc_option(const std::string& base_path, const LogEncOptions& enc_options) {
    m_log_enc_options = enc_options;
    return true;
}

void LogSinkBackend::consume(const boost::log::record_view& rec, const std::string& formatted_message) {
    // In standalone, just write to the base text_file_backend without encryption
    boost::log::sinks::text_file_backend::consume(rec, formatted_message);
}
#endif // ENGINE_BAMBU

} // namespace Slic3r