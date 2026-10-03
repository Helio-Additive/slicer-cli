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
#include <cstdint>
#include <cstdlib>
#include <algorithm>
#include <chrono>
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

// Core libslic3r headers
#include "libslic3r/libslic3r.h"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintConfig.hpp"
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
#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/BuildVolume.hpp"
#include "libslic3r/Semver.hpp"
#include "libslic3r_version.h"

// For JSON parsing (using libslic3r's built-in nlohmann/json)
#include <nlohmann/json.hpp>

#include <boost/filesystem/fstream.hpp>
#include <boost/filesystem.hpp>

// Boost.Log bridge: libslic3r raises most of its diagnostics through
// BOOST_LOG_TRIVIAL and never through any callback. `libslic3r_core` already
// links Boost::log and Boost::log_setup unconditionally (CMakeLists.txt — the
// same list for ENGINE=bambu and ENGINE=orca), so attaching one more sink adds
// no new dependency on either engine.
#include <csignal>
#include "diagnostic_output.hpp"
#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/support/date_time.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/utility/setup/console.hpp>
#include <boost/log/sinks/sync_frontend.hpp>
#include <boost/log/sinks/basic_sink_backend.hpp>
#include <boost/log/attributes/value_extraction.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include "calib_args.hpp"
#include "layout_plan.hpp"

#ifdef __APPLE__
#include <mach-o/dyld.h>   // _NSGetExecutablePath
#include <climits>         // PATH_MAX
#endif
using json = nlohmann::json;

// ─────────────────────────────────────────────────────────────────────────────
// Structured warning event emission (purely additive — does NOT alter slicing
// or G-code output).
//
// The slicer engine surfaces non-fatal warnings through `set_status_callback`
// during `print.process()` (cantilever / sharp tail / BBL collisions / etc.)
// and validation findings via `Print::validate()` returning a non-empty
// `StringObjectException`. Both vanish today: progress messages go to
// stdout as plain text, validation errors go to stderr and terminate.
//
// We add one JSON object per line on stdout, prefixed with the magic sentinel
// `[[SLICER_EVENT]]` so the TS demuxer in `ui/src/api/slicer.ts` can pick
// them out without parsing every line. The CLI's regular text output, exit
// codes, and G-code emission are untouched — this is read-only telemetry on
// top of what the engine already does.
//
// `tag` is the canonical enum-name string the agent keys off for remediation
// lookup; `message` is the localized human text from BBS.
// ─────────────────────────────────────────────────────────────────────────────

namespace {

constexpr const char* SLICER_EVENT_PREFIX = "[[SLICER_EVENT]] ";

const char* slicing_notification_tag(int t) {
    using NT = Slic3r::PrintStateBase::SlicingNotificationType;
    switch (static_cast<NT>(t)) {
        case NT::SlicingDefaultNotification:    return "SlicingDefaultNotification";
        case NT::SlicingReplaceInitEmptyLayers: return "SlicingReplaceInitEmptyLayers";
        case NT::SlicingNeedSupportOn:          return "SlicingNeedSupportOn";
        case NT::SlicingEmptyGcodeLayers:       return "SlicingEmptyGcodeLayers";
        case NT::SlicingGcodeOverlap:           return "SlicingGcodeOverlap";
    }
    return "SlicingUnknown";
}

const char* warning_level_tag(Slic3r::PrintStateBase::WarningLevel l) {
    return l == Slic3r::PrintStateBase::WarningLevel::CRITICAL ? "critical" : "non_critical";
}

const char* string_exception_tag(Slic3r::StringExceptionType t) {
    switch (t) {
        case Slic3r::STRING_EXCEPT_NOT_DEFINED:                   return "STRING_EXCEPT_NOT_DEFINED";
        case Slic3r::STRING_EXCEPT_FILAMENT_NOT_MATCH_BED_TYPE:   return "STRING_EXCEPT_FILAMENT_NOT_MATCH_BED_TYPE";
        case Slic3r::STRING_EXCEPT_FILAMENTS_DIFFERENT_TEMP:      return "STRING_EXCEPT_FILAMENTS_DIFFERENT_TEMP";
        case Slic3r::STRING_EXCEPT_OBJECT_COLLISION_IN_SEQ_PRINT: return "STRING_EXCEPT_OBJECT_COLLISION_IN_SEQ_PRINT";
        case Slic3r::STRING_EXCEPT_OBJECT_COLLISION_IN_LAYER_PRINT: return "STRING_EXCEPT_OBJECT_COLLISION_IN_LAYER_PRINT";
        case Slic3r::STRING_EXCEPT_LAYER_HEIGHT_EXCEEDS_LIMIT:    return "STRING_EXCEPT_LAYER_HEIGHT_EXCEEDS_LIMIT";
        case Slic3r::STRING_EXCEPT_COUNT:                         return "STRING_EXCEPT_COUNT";
    }
    return "STRING_EXCEPT_UNKNOWN";
}

void emit_event(const json& payload) {
    // One JSON object per line so a streaming line-reader in TS can split
    // events without buffering. Flush so the host sees events as the slice
    // progresses (warnings can fire mid-pipeline).
    //
    // dump() with the default error handler throws type_error.316 on any byte
    // sequence that is not valid UTF-8, and event payloads now carry engine
    // strings the driver never chose: file paths, object names, filament names,
    // raw log lines. Replacing invalid bytes keeps a diagnostic from turning a
    // slice that used to succeed into a failure; the event stream reports, it
    // never decides. `ensure_ascii=false` preserves today's byte output for the
    // four pre-existing event kinds.
    std::string line;
    try {
        line = payload.dump(-1, ' ', false, json::error_handler_t::replace);
        slicer_cli::diagnostics::write_event(line);
    } catch (...) {
        return;  // a diagnostic must never be the reason a slice fails
    }
}

// ── Channel 1: config deserialization findings ──────────────────────────────
// `ConfigSubstitutionContext` (Config.hpp:255) accumulates BOTH the forward-
// compatibility substitutions the engine silently performed AND the keys it did
// not recognize at all. The GUI shows these ("some settings were incompatible
// and have been substituted"); the CLI constructs the context, hands it to
// load_bbs_3mf / load_from_json, and then never reads it.
/// Emits structured events for substituted and unrecognized loaded-config values.
void emit_config_substitutions(const Slic3r::ConfigSubstitutionContext& ctx,
                               const std::string& source) {
    for (const auto& sub : ctx.substitutions) {
        json e;
        e["event"]     = "config_substituted";
        e["tag"]       = "ForwardCompatibilitySubstitution";
        e["source"]    = source;
        e["opt_key"]   = sub.opt_def ? sub.opt_def->opt_key : std::string{};
        e["old_value"] = sub.old_value;
        e["new_value"] = sub.new_value ? sub.new_value->serialize() : std::string{};
        e["message"]   = "Setting '" + (sub.opt_def ? sub.opt_def->opt_key : std::string("<unknown>")) +
                         "' had an unusable value '" + sub.old_value + "'; the engine substituted a default";
        emit_event(e);
    }
    if (!ctx.unrecogized_keys.empty()) {  // sic — upstream spelling
        json e;
        e["event"]   = "config_unknown_keys";
        e["tag"]     = "UnrecognizedConfigKeys";
        e["source"]  = source;
        e["keys"]    = ctx.unrecogized_keys;
        e["count"]   = ctx.unrecogized_keys.size();
        e["message"] = "The engine did not recognize " +
                       std::to_string(ctx.unrecogized_keys.size()) +
                       " configuration key(s) from " + source + "; they were ignored";
        emit_event(e);
    }
}

// ── Channel 2: G-code processor findings ───────────────────────────────────
// `GCodeProcessorResult` (GCodeProcessor.hpp:165) is populated by export_gcode
// and carries the entire set of post-slice checks the GUI surfaces: the slicing
// warning list, toolpath conflict detection, the multi-extruder printable
// area/height check bitfield, unprintable-filament findings, the
// toolpath-outside-bed flag and the timelapse warning code. main() passes
// `&gcode_result` purely because GCode.cpp dereferences it, then discards it.
/// Maps engine slice-warning levels to stable event severity tags.
const char* slice_warning_level_tag(int level) {
    switch (level) {
        case 0:  return "tip";
        case 1:  return "warning";
        case 2:  return "error";
        default: return "unknown";
    }
}

/// Emits structured flags and details from an engine G-code check result.
void emit_gcode_check_result(const Slic3r::GCodeProcessorResult& r) {
    const int code = r.gcode_check_result.error_code;
    if (code == 0) return;
    // Bit meanings are documented at GCodeProcessor.hpp:141-143.
    static const std::pair<int, const char*> kBits[] = {
        {1 << 0,  "multi_extruder_printable_area"},
        {1 << 1,  "multi_extruder_printable_height"},
        {1 << 2,  "plate_printable_area"},
        {1 << 3,  "plate_printable_height"},
        {1 << 4,  "wrapping_detection_area"},
        {1 << 10, "filament_map"},
        {1 << 11, "printing_mass_over_limit"},
    };
    json flags = json::array();
    for (const auto& [bit, name] : kBits)
        if (code & bit) flags.push_back(name);
    json e;
    e["event"]      = "gcode_check";
    e["tag"]        = "GCodeCheckResult";
    e["error_code"] = code;
    e["flags"]      = flags;
    e["message"]    = "Post-slice G-code check reported error_code " + std::to_string(code);
    json areas = json::object();
    for (const auto& [extruder, entries] : r.gcode_check_result.print_area_error_infos) {
        json list = json::array();
        for (const auto& [filament_id, object_label_id] : entries)
            list.push_back(json{{"filament_id", filament_id}, {"object_label_id", object_label_id}});
        areas[std::to_string(extruder)] = list;
    }
    if (!areas.empty()) e["print_area_errors"] = areas;
    json heights = json::object();
    for (const auto& [extruder, entries] : r.gcode_check_result.print_height_error_infos) {
        json list = json::array();
        for (const auto& [filament_id, object_label_id] : entries)
            list.push_back(json{{"filament_id", filament_id}, {"object_label_id", object_label_id}});
        heights[std::to_string(extruder)] = list;
    }
    if (!heights.empty()) e["print_height_errors"] = heights;
    emit_event(e);
}

/// Emits diagnostics retained in a completed G-code processing result.
void emit_gcode_result_diagnostics(const Slic3r::GCodeProcessorResult& r) {
    for (const auto& w : r.warnings) {
        json e;
        e["event"]      = "slice_warning";
        e["tag"]        = w.msg;          // the enum-name string the GUI keys off
        e["level"]      = slice_warning_level_tag(w.level);
        e["error_code"] = w.error_code;   // the code BambuStudio shows the user
        e["message"]    = w.msg;
        if (!w.params.empty()) e["params"] = w.params;
        emit_event(e);
    }
    if (r.conflict_result.has_value()) {
        const auto& c = *r.conflict_result;
        json e;
        e["event"]    = "toolpath_conflict";
        e["tag"]      = "ToolpathConflict";
        e["object_a"] = c._objName1;
        e["object_b"] = c._objName2;
        e["height"]   = c._height;
        e["layer"]    = c.layer;
        // A null _obj1 means the conflicting party is the prime/wipe tower.
        e["involves_wipe_tower"] = (c._obj1 == nullptr || c._obj2 == nullptr);
        e["message"]  = "Toolpaths of '" + c._objName1 + "' and '" + c._objName2 +
                        "' conflict at height " + std::to_string(c._height);
        emit_event(e);
    }
    emit_gcode_check_result(r);
    // `filament_printable_reuslt` (sic — upstream spelling) carries a
    // has_value() helper, but it is `const` on BambuStudio and NOT const on
    // OrcaSlicer, so calling it through this const& breaks the orca build.
    // Both engines define it as exactly this emptiness test.
    if (!r.filament_printable_reuslt.conflict_filament.empty()) {
        json e;
        e["event"]             = "filament_unprintable";
        e["tag"]               = "FilamentPrintableResult";
        e["conflict_filament"] = r.filament_printable_reuslt.conflict_filament;
        e["plate_name"]        = r.filament_printable_reuslt.plate_name;
        e["message"]           = "One or more filaments cannot be printed on their assigned extruder";
        emit_event(e);
    }
    if (r.toolpath_outside) {
        emit_event({{"event","toolpath_outside_bed"},
                    {"tag","ToolpathOutside"},
                    {"message","Some toolpaths fall outside the printable area"}});
    }
    if (r.timelapse_warning_code != 0) {
        emit_event({{"event","timelapse_warning"},
                    {"tag","TimelapseWarning"},
                    {"code", r.timelapse_warning_code},
                    {"message","Timelapse configuration produced a warning"}});
    }
    if (!r.limit_filament_maps.empty()) {
        emit_event({{"event","filament_map_limited"},
                    {"tag","LimitFilamentMaps"},
                    {"limit_filament_maps", r.limit_filament_maps},
                    {"message","The engine constrained the filament-to-extruder map"}});
    }
}

// ── Channel 3: the Boost.Log bridge ────────────────────────────────────────
// Most engine diagnostics have NO callback and NO return value: they are
// BOOST_LOG_TRIVIAL(error/warning) lines that land on stdout as free text,
// interleaved with the CLI's own banner output. Attach a second sink that
// mirrors warning-and-above records into the structured stream. The existing
// console output is left exactly as it is: humans keep the text log, agents get
// JSON. libslic3r's set_logging_level() installs a CORE filter
// (utils.cpp:113-121), so this sink can never see records the core dropped; the
// sink's own filter pins the event stream at warning+ regardless of -v.
/// Maps Boost.Log severities to stable structured-event tags.
const char* boost_severity_tag(boost::log::trivial::severity_level level) {
    switch (level) {
        case boost::log::trivial::trace:   return "trace";
        case boost::log::trivial::debug:   return "debug";
        case boost::log::trivial::info:    return "info";
        case boost::log::trivial::warning: return "warning";
        case boost::log::trivial::error:   return "error";
        case boost::log::trivial::fatal:   return "fatal";
    }
    return "unknown";
}

class EngineLogEventBackend final
    : public boost::log::sinks::basic_sink_backend<
          boost::log::sinks::combine_requirements<
              boost::log::sinks::synchronized_feeding>::type> {
public:
    /// Converts an accepted engine log record into a best-effort structured event.
    void consume(const boost::log::record_view& rec) {
        // An exception thrown here would propagate out of the BOOST_LOG_TRIVIAL
        // statement inside arbitrary libslic3r code, i.e. this bridge could
        // abort a slice that previously succeeded. Swallow everything: the
        // event stream informs, it never changes the outcome.
        try {
            auto severity = rec[boost::log::trivial::severity];
            auto message  = rec[boost::log::expressions::smessage];
            json e;
            e["event"]    = "engine_log";
            e["tag"]      = "EngineLogRecord";
            e["severity"] = severity ? boost_severity_tag(severity.get()) : "unknown";
            e["message"]  = message ? message.get() : std::string{};
            emit_event(e);
        } catch (...) {
        }
    }
};

/// Installs console and warning-or-higher structured-event Boost.Log sinks.
void install_engine_log_bridge() {
    // Register a real console sink before adding the event sink. Boost.Log's
    // fallback console is disabled as soon as any explicit sink is present.
    // Preserve human-readable records at every level allowed by the core.
    namespace logging = boost::log;
    namespace expr = boost::log::expressions;
    logging::add_common_attributes();
    logging::add_console_log(std::cout,
        logging::keywords::auto_flush = true,
        logging::keywords::format = (
            expr::stream
            << "[" << expr::format_date_time<boost::posix_time::ptime>(
                "TimeStamp", "%Y-%m-%d %H:%M:%S.%f")
            << "] [" << expr::attr<logging::attributes::current_thread_id::value_type>("ThreadID")
            << "] [" << logging::trivial::severity << "] " << expr::smessage));
    using Backend = EngineLogEventBackend;
    using Sink    = boost::log::sinks::synchronous_sink<Backend>;
    auto sink = boost::make_shared<Sink>(boost::make_shared<Backend>());
    // Only warning and above become events; the core filter set by
    // set_logging_level() still governs what reaches any sink at all.
    sink->set_filter(boost::log::trivial::severity >= boost::log::trivial::warning);
    boost::log::core::get()->add_sink(sink);
}

void emit_status_warning(const Slic3r::PrintBase::SlicingStatus& s) {
    using FB = Slic3r::PrintBase::SlicingStatus::FlagBits;
    const bool is_warning =
        (s.flags & (FB::UPDATE_PRINT_STEP_WARNINGS | FB::UPDATE_PRINT_OBJECT_STEP_WARNINGS)) != 0;
    if (!is_warning) return;
    json e;
    e["event"]    = "warning";
    e["tag"]      = slicing_notification_tag(static_cast<int>(s.message_type));
    e["level"]    = warning_level_tag(s.warning_level);
    e["message"]  = s.text;
    e["step"]     = s.warning_step;
    e["scope"]    = (s.flags & FB::UPDATE_PRINT_OBJECT_STEP_WARNINGS) ? "object" : "print";
    emit_event(e);
}

void emit_validation_event(const Slic3r::StringObjectException& v) {
    json e;
    e["event"]    = v.is_warning ? "validation_warning" : "validation_error";
    e["tag"]      = string_exception_tag(v.type);
    e["message"]  = v.string;
    if (!v.opt_key.empty()) e["opt_key"] = v.opt_key;
    if (!v.params.empty())  e["params"]  = v.params;
#ifdef ENGINE_BAMBU
    // `hypetext` (sic) is a BambuStudio-only field on StringObjectException.
    if (!v.hypetext.empty()) e["hypertext"] = v.hypetext;
#endif
    emit_event(e);
}

// ─────────────────────────────────────────────────────────────────────────────
// Driver-side normalization for the unbound placeholder `initial_no_support_filament_id`
// (cli #4 / desktop tracker slicer #152).
//
// Neither BambuStudio nor OrcaSlicer bind `initial_no_support_filament_id` in the
// PlaceholderParser. Both bind only `initial_no_support_tool` /
// `initial_no_support_extruder` / `initial_no_support_hotend`, and the first two are
// the SAME int `initial_non_support_extruder_id` (GCode.cpp:2458-2460). Because
// `_tools` and `_filaments` are aliases of the same filament index, the legacy token
// is semantically identical to `initial_no_support_extruder`.
//
// The token is not produced by any stock profile in this repo or upstream master
// start-gcode; it only arrives via a hand-edited / third-party custom gcode embedded
// in a 3MF. When present it makes the PlaceholderParser throw at export and aborts the
// slice. The correct fix is therefore a driver-side alias at config load — NOT an
// engine edit (engine submodules are off-limits).
//
// Whole-token rewrite across every coString custom-gcode key. The character on either
// side of a match must be a non-identifier char, so a hypothetical
// `initial_no_support_filament_idx` and any identifier that merely embeds the token are
// never touched. (The separately-bound `initial_filament_id` is a different, shorter
// string and is never searched for, so it is inherently safe.)
int normalize_legacy_gcode_tokens(Slic3r::DynamicPrintConfig& config, bool report_event = true) {
    static const std::string kLegacyToken = "initial_no_support_filament_id";
    static const std::string kBoundToken  = "initial_no_support_extruder";
    // Single-string (coString) custom-gcode keys.
    // Every coString custom-gcode key either engine runs through
    // placeholder_parser_process (PrintConfig.cpp `add("*_gcode", coString)`,
    // minus `export_gcode` which is the output-path flag, not a template). This
    // binary builds BOTH engines (ENGINE_BAMBU / ENGINE_ORCA), so the list is the
    // union; keys absent from the active engine's schema are null-guarded no-ops
    // (the `file_*` / `*_extrusion_role_*` keys are Orca-only).
    static const std::vector<std::string> kGcodeStringKeys = {
        "machine_start_gcode", "machine_end_gcode",
        "before_layer_change_gcode", "layer_change_gcode",
        "change_filament_gcode", "time_lapse_gcode",
        "machine_pause_gcode", "printing_by_object_gcode",
        "template_custom_gcode", "wrapping_detection_gcode",
        // Orca-only:
        "file_start_gcode", "change_extrusion_role_gcode",
        "process_change_extrusion_role_gcode",
    };
    // Per-filament (coStrings) custom-gcode keys — these ALSO run through the
    // PlaceholderParser (filament_start/end emission), so the unbound token can
    // abort from them too. (`filament_change_extrusion_role_gcode` is Orca-only.)
    static const std::vector<std::string> kGcodeStringsKeys = {
        "filament_start_gcode", "filament_end_gcode",
        "filament_change_extrusion_role_gcode",
    };
    const size_t tlen = kLegacyToken.size();
    auto is_ident = [](char c) {
        return (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
               (c >= '0' && c <= '9') || c == '_';
    };
    // Whole-token alias within one string; bumps `rewrites` per replacement.
    auto rewrite_one = [&](const std::string& v, int& rewrites) -> std::string {
        if (v.find(kLegacyToken) == std::string::npos) return v;
        std::string out;
        out.reserve(v.size());
        size_t pos = 0;
        while (pos < v.size()) {
            const size_t found = v.find(kLegacyToken, pos);
            if (found == std::string::npos) { out.append(v, pos, std::string::npos); break; }
            const bool left_ok  = (found == 0)        || !is_ident(v[found - 1]);
            const size_t after  = found + tlen;
            const bool right_ok = (after >= v.size()) || !is_ident(v[after]);
            if (left_ok && right_ok) {
                out.append(v, pos, found - pos);
                out.append(kBoundToken);
                pos = after;
                ++rewrites;
            } else {
                // Substring match inside a longer identifier — copy one char past the
                // match start and keep scanning so overlapping matches still resolve.
                out.append(v, pos, found - pos + 1);
                pos = found + 1;
            }
        }
        return out;
    };
    int total_rewrites = 0;
    std::vector<std::string> rewritten_keys;
    for (const auto& key : kGcodeStringKeys) {
        if (!config.has(key)) continue;
        int key_rewrites = 0;
        const std::string out = rewrite_one(config.opt_string(key), key_rewrites);
        if (key_rewrites > 0) {
            config.set_key_value(key, new Slic3r::ConfigOptionString(out));
            total_rewrites += key_rewrites;
            rewritten_keys.push_back(key);
        }
    }
    for (const auto& key : kGcodeStringsKeys) {
        auto* opt = config.option<Slic3r::ConfigOptionStrings>(key);
        if (!opt) continue;
        int key_rewrites = 0;
        std::vector<std::string> values = opt->values;
        for (auto& s : values) s = rewrite_one(s, key_rewrites);
        if (key_rewrites > 0) {
            config.set_key_value(key, new Slic3r::ConfigOptionStrings(values));
            total_rewrites += key_rewrites;
            rewritten_keys.push_back(key);
        }
    }
    if (total_rewrites > 0 && report_event) {
        json e;
        e["event"]   = "config_normalized";
        e["tag"]     = "LegacyGcodeTokenAliased";
        e["message"] = "Aliased unbound placeholder '" + kLegacyToken + "' -> '" +
                       kBoundToken + "' in " + std::to_string(rewritten_keys.size()) +
                       " custom-gcode key(s); " + std::to_string(total_rewrites) +
                       " occurrence(s) rewritten";
        e["from"]    = kLegacyToken;
        e["to"]      = kBoundToken;
        e["keys"]    = rewritten_keys;
        e["count"]   = total_rewrites;
        emit_event(e);
    }
    return total_rewrites;
}

}  // namespace

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
    std::cout << "Usage: " << prog_name << " [options] <input.stl|input.3mf>\n"
              << "\n=== Configuration Options ===\n"
              << "  --config <file>        Load all settings from BambuStudio config bundle (JSON)\n"
              << "  --machine <file>       Load machine/printer config (JSON)\n"
              << "  --filament <file>      Load filament config (JSON)\n"
              << "  --process <file>       Load process/print config (JSON)\n"
              << "\n=== Quick Settings (override config files) ===\n"
              << "  --layer-height <mm>    Layer height (e.g., 0.2)\n"
              << "  --infill <percent>     Infill density 0-100 (e.g., 20)\n"
              << "  --perimeters <n>       Number of perimeters/walls (e.g., 3)\n"
              << "  --nozzle <mm>          Nozzle diameter (e.g., 0.4)\n"
              << "  --temp <C>             Nozzle temperature (e.g., 210)\n"
              << "  --bed-temp <C>         Bed temperature (e.g., 60)\n"
              << "\n=== Output Options ===\n"
              << "  -o, --output <file>    Output G-code file (default: output.gcode)\n"
              << "  --plate <N>            Slice only plate N from a multi-plate 3MF (1-based)\n"
              << "  --slice <N>            Official plate loop: 0 = every plate, N = plate N;\n"
              << "                         writes <outputdir>/plate_N.gcode per plate\n"
              << "  --outputdir <dir>      Output folder for --slice (default: current folder);\n"
              << "                         --slice writes result.json there\n"
              << "  --info <file>          Print what the file is (printer, plates, maker app)\n"
              << "                         and which engine binary fits it, as JSON\n"
              << "  --printer-preset <name>  Printer system preset by name (STL input); the\n"
              << "  --process-preset <name>  desktop app's settings, every parent applied.\n"
              << "  --filament-preset <name> Repeat for more filaments. Omitted process/filament:\n"
              << "                         the printer's defaults.\n"
              << "  --list-presets [--printer <name>]  This engine's system presets as JSON\n"
              << "  --export-3mf <name>    With --slice: write the sliced project (G-code and\n"
              << "                         settings per plate) as <outputdir>/<name>\n"
              << "  --arrange <0|1>        1 = place the objects on the bed before slicing (with --slice)\n"
              << "  --allow-newer-file     Slice a 3MF saved by a newer app version than this engine\n"
              << "  --allow-substitution   With --slice: slice a 3MF whose settings hold values this\n"
              << "                         engine does not know, with the engine's substitutes\n"
              << "                         (--slice refuses them; the default call substitutes)\n"
              << "  --progress             Progress events ({\"event\":\"progress\",...}); on with --slice\n"
              << "  --no-normalize-legacy-gcode  Do NOT alias unbound legacy placeholder\n"
              << "                         tokens (e.g. initial_no_support_filament_id) in\n"
              << "                         custom G-code. Default: normalization is on.\n"
              << "\n=== Layout & Arrange (issue #7) ===\n"
              << "  --layout <file>        Headless arrange spike: legacy JSON with profiles\n"
              << "                         and object paths; emits JSON placements.\n"
              << "  --layout-plan          Run headless arrange with versioned JSON contract.\n"
              << "                         See layout_plan.hpp for schema details.\n"
              << "  --input <file>         Input file for layout modes / stdin default.\n"
              << "\n=== Calibration (slicer-cli #5) ===\n"
              << "  --calib-mode <mode>    Emit a calibration test. One of:\n"
              << "                           temp_tower, retraction_tower,\n"
              << "                           pressure_advance_line, pressure_advance_pattern,\n"
              << "                           pressure_advance_tower\n"
              << "  --calib-start <n>      Sweep start value (mode-specific units)\n"
              << "  --calib-end <n>        Sweep end value\n"
              << "  --calib-step <n>       Sweep step (> 0)\n"
              << "  --calib-extruder-id <n>  Logical extruder to calibrate (default 0)\n"
              << "  --calib-no-numbers     Skip numeric labels (pressure_advance_line only;\n"
              << "                         the pattern always labels its rows)\n"
              << "                         Tower/line modes need an --input model; the\n"
              << "                         pattern mode synthesizes its own handle cube.\n"
              << "  -v, --verbose          Verbose output\n"
              << "  -h, --help             Show this help message\n"
              << "\n=== Examples ===\n"
              << "Using BambuStudio profiles:\n"
              << "  " << prog_name << " model.stl \\\n"
              << "    --machine profiles/BBL/machine/\"Bambu Lab X1 0.4 nozzle.json\" \\\n"
              << "    --filament profiles/BBL/filament/\"Bambu PLA Basic @BBL X1C.json\" \\\n"
              << "    --process profiles/BBL/process/\"0.20mm Standard @BBL X1C.json\" \\\n"
              << "    -o output.gcode\n"
              << "\nLayout plan (JSON on stdin):\n"
              << "  cat problem.json | " << prog_name << " --layout-plan\n"
              << "  " << prog_name << " --layout-plan --input problem.json\n"
              << "\nQuick slicing with defaults:\n"
              << "  " << prog_name << " model.stl --layer-height 0.2 --infill 20 -o output.gcode\n"
              << "\nNote: Config files are located in BambuStudio's resources/profiles/ directory\n";
}

#ifdef ENGINE_BAMBU
// ConfigOptionInts does not check stream failures and can throw after appending
// part of a vector. Check extraction first, then deserialize into a scratch
// option so an invalid overlay cannot replace an earlier accepted map.
bool deserialize_usable_nozzle_map(const std::string& text, Slic3r::ConfigOptionInts& out) {
    if (text.empty()) return false;
    std::istringstream stream(text);
    std::string token;
    while (std::getline(stream, token, ',')) {
        std::istringstream item(token);
        int nozzle;
        if (!(item >> nozzle) || nozzle < 0) return false;
        item >> std::ws;
        if (!item.eof()) return false;
    }
    Slic3r::ConfigOptionInts candidate;
    try {
        if (!candidate.deserialize(text) || candidate.values.empty()) return false;
    } catch (...) {
        return false;
    }
    out.values = std::move(candidate.values);
    return true;
}

// Match ConfigBase::load_from_json's string/array conversion for coInts:
// homogeneous arrays, string leaves, commas within arrays, '#' between groups.
bool nozzle_map_json_text(const json& value, std::string& text) {
    if (value.is_string()) {
        text += value.get<std::string>();
        return true;
    }
    if (!value.is_array()) return false;
    std::string type;
    bool first = true;
    for (const auto& element : value) {
        if (first) type = element.type_name();
        else if (type != element.type_name()) return false;
        if (!first) text += element.is_array() ? '#' : ',';
        first = false;
        if (!nozzle_map_json_text(element, text)) return false;
    }
    return true;
}
#endif

// Load JSON config and record only accepted, usable nozzle-map overlays.
bool load_json_config(const std::string& filepath, Slic3r::DynamicPrintConfig& config,
                      bool verbose = false, const std::string& diagnostic_source = {},
                      bool* supplied_nozzle_map = nullptr) {
    if (verbose) {
        std::cout << "Loading config: " << filepath << "\n";
    }

    std::ifstream f(filepath);
    if (!f.is_open()) {
        std::cerr << "Error: Cannot open config file: " << filepath << "\n";
        return false;
    }

    try {
        json j = json::parse(f);

        // Create substitution context for config deserialization
        Slic3r::ConfigSubstitutionContext substitution_context(Slic3r::ForwardCompatibilitySubstitutionRule::Enable);

        // Iterate through all key-value pairs
        for (auto& [key, value] : j.items()) {
            // Skip profile metadata, including identity/version fields handled
            // separately by both engines' ConfigBase::load_from_json.
            if (key == "type" || key == "name" || key == "inherits" ||
                key == "from" || key == "setting_id" || key == "instantiation" ||
                key == "description" || key == "compatible_printers" ||
                key == "compatible_prints" || key == "include" ||
                key == "upward_compatible_machine" || key == "printer_model" ||
                key == "printer_variant" || key == "default_filament_profile" ||
                key == "default_print_profile" || key == "filament_id" ||
                key == "version" || key == "url" || key == "is_custom_defined") {
                continue;
            }

            try {
#ifdef ENGINE_BAMBU
                if (supplied_nozzle_map && key == "filament_nozzle_map" &&
                    !value.is_string() && !value.is_number() && !value.is_array())
                    continue;
                if (supplied_nozzle_map && key == "filament_nozzle_map" && value.is_array()) {
                    // The generic loader drops unsupported elements. Do not turn
                    // a malformed nozzle-map array into an apparently valid map.
                    bool valid_types = !value.empty();
                    for (const auto& element : value)
                        valid_types = valid_types && (element.is_string() || element.is_number());
                    if (!valid_types) continue;
                }
#endif
                // Use set_deserialize to respect existing config types
                // This handles type conversion properly
                std::string value_str;

                if (value.is_array()) {
                    // Convert array to comma-separated string
                    // All ConfigOption::deserialize() methods split on ','
                    std::vector<std::string> parts;
                    for (auto& v : value) {
                        if (v.is_string()) {
                            parts.push_back(v.get<std::string>());
                        } else if (v.is_number()) {
                            parts.push_back(std::to_string(v.get<double>()));
                        }
                    }
                    value_str = "";
                    for (size_t i = 0; i < parts.size(); i++) {
                        if (i > 0) value_str += ",";
                        value_str += parts[i];
                    }
                } else if (value.is_string()) {
                    value_str = value.get<std::string>();
                } else if (value.is_number_float()) {
                    value_str = std::to_string(value.get<double>());
                } else if (value.is_number_integer()) {
                    value_str = std::to_string(value.get<int>());
                } else if (value.is_boolean()) {
                    value_str = value.get<bool>() ? "1" : "0";
                }

                if (!value_str.empty() && value_str != "nil") {
#ifdef ENGINE_BAMBU
                    if (supplied_nozzle_map && key == "filament_nozzle_map") {
                        // The generic numeric conversion uses double formatting
                        // (1 becomes "1.000000"). Preserve integer map tokens
                        // without permitting fractional or malformed values.
                        auto map_token = [](const json& element) -> std::string {
                            if (element.is_string()) return element.get<std::string>();
                            if (element.is_number_float()) {
                                const double number = element.get<double>();
                                if (number >= 0 && number <= std::numeric_limits<int>::max() &&
                                    number == static_cast<int>(number))
                                    return std::to_string(static_cast<int>(number));
                            }
                            return element.dump();
                        };
                        if (value.is_array()) {
                            value_str.clear();
                            for (size_t i = 0; i < value.size(); ++i) {
                                if (i != 0) value_str += ',';
                                value_str += map_token(value[i]);
                            }
                        } else if (value.is_number()) {
                            value_str = map_token(value);
                        }
                        Slic3r::ConfigOptionInts accepted;
                        if (!deserialize_usable_nozzle_map(value_str, accepted)) continue;
                        Slic3r::DynamicPrintConfig scratch;
                        scratch.set_deserialize(key, value_str, substitution_context);
                        const auto* parsed = scratch.option<Slic3r::ConfigOptionInts>(key, false);
                        if (!parsed || parsed->values.empty()) continue;
                        config.set_key_value(key, parsed->clone());
                        *supplied_nozzle_map = true;
                        continue;
                    }
#endif
                    // set_deserialize respects the existing option type
                    config.set_deserialize(key, value_str, substitution_context);
                }
            } catch (const std::exception& e) {
                if (!diagnostic_source.empty()) {
                    emit_event({{"event", "config_value_rejected"},
                            {"tag", "ProfileConfigValueRejected"},
                            {"source", diagnostic_source},
                            {"opt_key", key},
                            {"message", "Failed to deserialize setting '" + key +
                                        "'; its value may have changed: " + e.what()}});
                }
                if (verbose) {
                    std::cerr << "Warning: Failed to set config key '" << key << "': " << e.what() << "\n";
                }
            }
        }

        // Keep non-fatal forward-compatibility substitutions and unknown keys
        // visible before success when the caller opts into slicing events.
        // Legacy --layout callers leave diagnostic_source empty.
        if (!diagnostic_source.empty()) {
            emit_config_substitutions(substitution_context, diagnostic_source);
        }

        if (verbose) {
            std::cout << "  Loaded successfully\n";
        }
        return true;

    } catch (const std::exception& e) {
        std::cerr << "Error parsing JSON config: " << e.what() << "\n";
        return false;
    }
}

#ifdef ENGINE_BAMBU
// load_bbs_3mf applies Metadata/project_settings.config directly to config.
// Inspect the same input layer before defaults or vector normalization obscure
// whether the file explicitly supplied a nozzle map.
bool bbs_3mf_config_contains_nozzle_map(const std::string& filepath,
                                      Slic3r::ConfigOptionInts& accepted_map) {
    mz_zip_archive zip;
    mz_zip_zero_struct(&zip);
    if (!Slic3r::open_zip_reader(&zip, filepath))
        return false;

    bool contains_nozzle_map = false;
    const mz_uint count = mz_zip_reader_get_num_files(&zip);
    for (mz_uint file_idx = 0; file_idx < count; ++file_idx) {
        mz_zip_archive_file_stat stat;
        if (mz_zip_reader_file_stat(&zip, file_idx, &stat)) {
            std::string name(stat.m_filename);
            std::replace(name.begin(), name.end(), '\\', '/');
            if (!boost::algorithm::iequals(name, "Metadata/project_settings.config")) continue;
            constexpr mz_uint64 max_project_settings_size = 16ULL * 1024 * 1024;
            if (stat.m_uncomp_size > max_project_settings_size) {
                mz_zip_reader_end(&zip);
                throw std::runtime_error("3MF project settings exceed the 16 MiB safety limit");
            }
            std::string content(stat.m_uncomp_size, '\0');
            if (mz_zip_reader_extract_to_mem(&zip, file_idx, content.data(), content.size(), 0)) {
                try {
                    const auto settings = json::parse(content);
                    if (settings.is_object() && settings.contains("filament_nozzle_map")) {
                        std::string text;
                        if (nozzle_map_json_text(settings["filament_nozzle_map"], text) &&
                            deserialize_usable_nozzle_map(text, accepted_map))
                            contains_nozzle_map = true;
                    }
                } catch (...) {
                    // load_bbs_3mf remains the authoritative parser and reports malformed input.
                }
            }
        }
    }
    mz_zip_reader_end(&zip);
    return contains_nozzle_map;
}
#endif

// When a 3MF already carries explicit per-filament physical nozzle assignments,
// recover the corresponding logical extruder map before slicing.  This avoids
// the auto-grouping path re-solving an already-constrained dual-nozzle setup
// into a different logical order than BambuStudio desktop.
// Returns true if it actually derived and applied a cross-nozzle filament_map.
#ifdef ENGINE_BAMBU
// ── BBS-only config-normalization helpers ───────────────────────────────────
// These four helpers (apply_explicit_nozzle_mapping, reassign_objects_to_master_
// nozzle, set_default_config, ensure_vector_config_sizes) exist solely to coax
// the Bambu engine into accepting a non-Bambu printer config. Their bodies use
// Bambu-only config keys/enums (e.g. fmmNozzleManual, filament_extruder_variant),
// so they are compiled only for ENGINE_BAMBU and called only from the gated
// front-end blocks in main(). OrcaSlicer needs none of them.
// `explicit_config_supplied_nozzle_map` says whether the fully-overlaid config
// carries its own `filament_nozzle_map`, rather than the driver's seed. Bambu's own
// headless CLI never derives a filament map from a default: under an automatic
// mode it takes the mode from the plate/global config (BambuStudio.cpp:6662-6665)
// and lets ToolOrdering group the filaments (ToolOrdering.cpp:1897-1914); the
// engine reads filament_nozzle_map only under Nozzle Manual
// (ToolOrdering.cpp:1885-1889), a mode the CLI refuses on one-nozzle-per-
// extruder machines such as the X2D/H2D (BambuStudio.cpp:6841-6846); and the
// CLI-argument nozzle map is consumed only on multi-nozzle machines in a manual
// mode (BambuStudio.cpp:6789, 6836-6838).  Bambu Studio exports always carry
// the key (PresetBundle.cpp:61, written as zeros at 2090-2091), so real Bambu
// files keep their previous behaviour here.  Before this guard, the seed set_default_config()
// writes ({1}) was padded by the extruder-count normalisation to [1,0] and read
// back here as if it were the file's: a fresh STL (or any input with <=2 slots
// and no nozzle map) on a two-head machine was then forced to "Nozzle Manual"
// with filament_map [1,2], and reassign_objects_to_master_nozzle() moved every
// object to physical nozzle 0 — on the X2D the Bowden head — instead of letting
// the engine's automatic grouping (ToolOrdering.cpp:1910-1914) pick the head.
/// Derives and applies a manual nozzle map only when input provenance is explicit.
bool apply_explicit_nozzle_mapping(
    Slic3r::DynamicPrintConfig& config,
    bool explicit_config_supplied_nozzle_map,
    bool verbose)
{
    // No explicit config nozzle map: there is nothing to apply. Leave the
    // engine in the mode the input asked for (Auto For Flush by default).
    if (!explicit_config_supplied_nozzle_map)
        return false;

    // If the plate-level filament_maps were already applied (mode set to "Nozzle Manual"
    // before calling this function), skip re-derivation — the mapping is already correct.
    {
        auto* mode_opt = config.option<Slic3r::ConfigOptionEnum<Slic3r::FilamentMapMode>>("filament_map_mode", false);
        if (mode_opt && mode_opt->value == Slic3r::FilamentMapMode::fmmNozzleManual)
            return false;
    }

    auto* filament_map = config.option<Slic3r::ConfigOptionInts>("filament_map", false);
    auto* filament_nozzle_map = config.option<Slic3r::ConfigOptionInts>("filament_nozzle_map", false);
    auto* physical_extruder_map = config.option<Slic3r::ConfigOptionInts>("physical_extruder_map", false);
    auto* nozzle_diameter = config.option<Slic3r::ConfigOptionFloats>("nozzle_diameter", false);
    if (!filament_map || !filament_nozzle_map || !physical_extruder_map || !nozzle_diameter)
        return false;

    const size_t filament_count = filament_map->values.size();
    const size_t extruder_count = nozzle_diameter->values.size();
    if (filament_count < 2 || extruder_count < 2)
        return false;
    if (filament_nozzle_map->values.size() < filament_count || physical_extruder_map->values.size() < extruder_count)
        return false;

    std::map<int, int> physical_to_logical;
    for (size_t logical_idx = 0; logical_idx < extruder_count; ++logical_idx) {
        // filament_map is 1-based.  Map physical nozzle -> 1-based logical extruder
        // index.  physical_extruder_map[logical_idx] gives the physical nozzle for
        // logical extruder logical_idx; we invert that to get physical -> logical.
        physical_to_logical[physical_extruder_map->values[logical_idx]] =
            static_cast<int>(logical_idx) + 1;
    }
    if (physical_to_logical.size() < extruder_count)
        return false;

    std::vector<int> derived_map = filament_map->values;
    for (size_t filament_idx = 0; filament_idx < filament_count; ++filament_idx) {
        auto it = physical_to_logical.find(filament_nozzle_map->values[filament_idx]);
        if (it == physical_to_logical.end())
            return false;
        derived_map[filament_idx] = it->second;
    }

    const bool uses_multiple_logical_extruders =
        std::adjacent_find(derived_map.begin(), derived_map.end(), std::not_equal_to<int>()) != derived_map.end();
    if (!uses_multiple_logical_extruders)
        return false;

    filament_map->values = derived_map;

    auto* filament_map_2 = config.option<Slic3r::ConfigOptionInts>("filament_map_2", true);
    filament_map_2->values.resize(derived_map.size());
    for (size_t i = 0; i < derived_map.size(); ++i)
        filament_map_2->values[i] = derived_map[i] - 1;

    // Force Nozzle Manual mode when filament_nozzle_map gives a cross-extruder
    // assignment. In this case the explicit config map is the authoritative
    // source for which filament goes on which physical nozzle. Without real AMS
    // data the fmmAutoForFlush algorithm always assigns every filament to the
    // master extruder, overriding the correct split.  Switching to Nozzle Manual
    // preserves the derived_map computed above.
    {
        Slic3r::ConfigSubstitutionContext substitution_context(Slic3r::ForwardCompatibilitySubstitutionRule::Enable);
        config.set_deserialize("filament_map_mode", "Nozzle Manual", substitution_context);
    }
    if (verbose) {
        std::cout << "Nozzle-map derivation: filament_map=[";
        for (size_t i = 0; i < derived_map.size(); ++i)
            std::cout << (i == 0 ? "" : ",") << derived_map[i];
        std::cout << "] mode=Nozzle Manual\n";
    }
    return true;
}

// When apply_explicit_nozzle_mapping derived a cross-nozzle filament_map from
// "Auto For Flush" mode, BambuStudio desktop reassigns objects to the master
// (right) nozzle during Print::process().  Replicate this by changing each
// object's "extruder" config to the filament slot on the master physical nozzle.
//
// This only runs when apply_explicit_nozzle_mapping returned true, meaning the
// plate had filament_maps="1 1" (auto) but filament_nozzle_map showed a cross-
// nozzle split.  Explicit plate maps (e.g. "2 1") skip this path entirely.
/// The 1-based filament slot on the master physical nozzle, or -1.
int master_nozzle_filament_slot(const Slic3r::DynamicPrintConfig& config)
{
    const auto* filament_map = config.option<Slic3r::ConfigOptionInts>("filament_map");
    const auto* physical_extruder_map = config.option<Slic3r::ConfigOptionInts>("physical_extruder_map");
    if (!filament_map || !physical_extruder_map)
        return -1;

    const size_t extruder_count = physical_extruder_map->values.size();
    if (extruder_count < 2)
        return -1;

    // Find master logical extruder: the one whose physical_extruder_map value is 0
    // (physical nozzle 0 = right/master on H2D).
    int master_logical_idx = -1;
    for (size_t i = 0; i < extruder_count; ++i) {
        if (physical_extruder_map->values[i] == 0) {
            master_logical_idx = static_cast<int>(i);
            break;
        }
    }
    if (master_logical_idx < 0)
        return -1;

    // Find the filament slot (1-based) that maps to the master logical extruder.
    // filament_map[i] is the 1-based logical extruder for filament i.
    int master_extruder_1based = master_logical_idx + 1;
    int master_filament_slot = -1;  // 1-based filament slot
    for (size_t i = 0; i < filament_map->values.size(); ++i) {
        if (filament_map->values[i] == master_extruder_1based) {
            master_filament_slot = static_cast<int>(i) + 1;
            break;
        }
    }
    return master_filament_slot;
}

/// Every given object (and its volume-level overrides) onto `master_filament_slot`.
void assign_objects_to_filament_slot(const std::vector<Slic3r::ModelObject*>& objects, int master_filament_slot)
{
    for (auto* obj : objects) {
        int cur = obj->config.extruder();
        if (cur != master_filament_slot) {
            obj->config.set_key_value("extruder", new Slic3r::ConfigOptionInt(master_filament_slot));
        }
        // Also update any volume-level extruder overrides
        for (auto* vol : obj->volumes) {
            const Slic3r::ConfigOption* vopt = vol->config.option("extruder");
            if (vopt && vopt->getInt() != 0 && vopt->getInt() != master_filament_slot) {
                vol->config.set_key_value("extruder", new Slic3r::ConfigOptionInt(master_filament_slot));
            }
        }
    }
}

void reassign_objects_to_master_nozzle(Slic3r::Model& model, const Slic3r::DynamicPrintConfig& config)
{
    const int master_filament_slot = master_nozzle_filament_slot(config);
    if (master_filament_slot < 0)
        return;
    // Reassign each object's extruder to the master nozzle's filament slot.
    assign_objects_to_filament_slot(model.objects, master_filament_slot);
}

// Initialize configuration with BambuStudio defaults
void set_default_config(Slic3r::DynamicPrintConfig& config) {
    /// Initialize config following BambuStudio's PresetBundle::full_fff_config pattern
    /// PresetBundle.cpp:2859-3150
    /// C++: out.apply(FullPrintConfig::defaults());

    // Start with full defaults - this ensures all keys exist with correct types
    config.apply(Slic3r::FullPrintConfig::defaults(), true);

    // Set preset IDs (mimics BambuStudio's preset tracking)
    /// PresetBundle.cpp:3125-3127
    /// C++: out.option<ConfigOptionString>("print_settings_id", true)->value = this->prints.get_selected_preset_name();
    config.set_key_value("print_settings_id", new Slic3r::ConfigOptionString(""));
    config.set_key_value("filament_settings_id", new Slic3r::ConfigOptionStrings({""}));
    config.set_key_value("printer_settings_id", new Slic3r::ConfigOptionString(""));

    // Initialize filament_map and filament_volume_map for single filament
    /// PresetBundle.cpp:2871-2875
    /// C++: std::vector<int> filament_maps = out.option<ConfigOptionInts>("filament_map")->values;
    size_t num_filaments = 1;
    std::vector<int> filament_maps(num_filaments, 1);
    std::vector<int> filament_volume_maps(num_filaments, 0); // nvtStandard = 0

    config.option<Slic3r::ConfigOptionInts>("filament_map", true)->values = filament_maps;
    config.option<Slic3r::ConfigOptionInts>("filament_volume_map", true)->values = filament_volume_maps;

    // Initialize filament_self_index (critical for multi-material code paths)
    /// PresetBundle.cpp:2964-2967
    /// C++: std::vector<int>& filament_self_indice = out.option<ConfigOptionInts>("filament_self_index", true)->values;
    /// C++: int index_size = out.option<ConfigOptionStrings>("filament_extruder_variant")->size();
    /// C++: filament_self_indice.resize(index_size, 1);
    auto filament_variant = config.option<Slic3r::ConfigOptionStrings>("filament_extruder_variant", true);
    if (filament_variant) {
        int index_size = filament_variant->values.size();
        if (index_size == 0) {
            index_size = 1;
            // CRITICAL: Must use valid variant string matching get_extruder_variant_string output
            // get_index_for_extruder compares against get_extruder_variant_string result
            // Format is: "<ExtruderType> <NozzleVolumeType>"
            // Default: etDirectDrive (0) + nvtStandard (0) = "Direct Drive Standard"
            /// PrintConfig.cpp:499-501
            /// C++: variant_string = s_keys_names_ExtruderType[extruder_type];
            /// C++: variant_string+= " ";
            /// C++: variant_string+= s_keys_names_NozzleVolumeType[nozzle_volume_type];
            /// PrintConfig.cpp:7255-7269
            /// C++: std::string extruder_variant = get_extruder_variant_string(extruder_type, nozzle_volume_type);
            /// C++: for (int index = 0; index < v_size; index++) { if (extruder_variant == variant) { ... } }
            filament_variant->values.resize(1, "Direct Drive Standard");
        }
        config.option<Slic3r::ConfigOptionInts>("filament_self_index", true)->values.resize(index_size, 1);
    }

    // Ensure support_filament and support_interface_filament are within bounds
    /// PresetBundle.cpp:3117-3122
    /// C++: auto *opt = dynamic_cast<ConfigOptionInt*>(out.option(key, false));
    /// C++: opt->value = boost::algorithm::clamp<int>(opt->value, 0, int(num_filaments));
    auto support_fil = config.option<Slic3r::ConfigOptionInt>("support_filament");
    if (support_fil) {
        support_fil->value = std::max(0, std::min(support_fil->value, (int)num_filaments));
    }
    auto support_iface = config.option<Slic3r::ConfigOptionInt>("support_interface_filament");
    if (support_iface) {
        support_iface->value = std::max(0, std::min(support_iface->value, (int)num_filaments));
    }

    // BYPASS STRATEGY: Pre-initialize filament maps to match what Print::process() will compute
    // This prevents the "filament maps changed" condition at Print.cpp:2670 from triggering
    // the problematic update_values_to_printer_extruders_for_multiple_filaments call
    /// Print.cpp:2670
    /// C++: if ((m_config.filament_map.values != f_maps) || (m_config.filament_volume_map.values != f_volume_maps) || ...)
    /// By setting these correctly up front, the condition will be FALSE and skip the hang

    // For single extruder setup:
    // - filament_map: which extruder each filament uses (1-indexed)
    // - filament_volume_map: nozzle volume type for each filament (0 = nvtStandard)
    // - filament_nozzle_map: nozzle mapping (typically matches filament_map)

    // Ensure filament_map is exactly what the analysis expects
    auto filament_map_opt = config.option<Slic3r::ConfigOptionInts>("filament_map", true);
    if (filament_map_opt->values.empty() || filament_map_opt->values[0] != 1) {
        filament_map_opt->values = {1};  // Single filament uses extruder 1
    }

    // Ensure filament_volume_map matches extruder nozzle types
    auto filament_volume_map_opt = config.option<Slic3r::ConfigOptionInts>("filament_volume_map", true);
    if (filament_volume_map_opt->values.empty() || filament_volume_map_opt->values[0] != 0) {
        filament_volume_map_opt->values = {0};  // nvtStandard
    }

    // Ensure filament_nozzle_map is set (may prevent other code paths)
    auto filament_nozzle_map_opt = config.option<Slic3r::ConfigOptionInts>("filament_nozzle_map", true);
    if (filament_nozzle_map_opt->values.empty()) {
        filament_nozzle_map_opt->values = {1};
    }

    // Ensure filament_map_2 is initialized (used in Print.cpp:2707)
    // IMPORTANT: filament_map_2 is a 0-based extruder *index*, unlike filament_map
    // which is a 1-based extruder *number*.
    // print.process() computes it via:
    //   get_index_for_extruder(filament_map[i], "print_extruder_id", ...)
    // For a single extruder: filament_map[0]=1, print_extruder_id={1} → index 0.
    // If we leave this at {1} the bypass condition
    //   m_config.filament_map_2 != f_maps_2   ({1} != {0})
    // fires even though all other maps match, triggering
    // update_values_to_printer_extruders_for_multiple_filaments which then
    // crashes with extruder_nozzle_volume_count=0 / "different nozzle volume processing".
    auto filament_map_2_opt = config.option<Slic3r::ConfigOptionInts>("filament_map_2", true);
    if (filament_map_2_opt->values.empty()) {
        filament_map_2_opt->values = {0};  // 0-based: extruder 1 is at index 0
    }

    // CRITICAL: Initialize print_extruder_variant and print_extruder_id
    // These are needed by get_index_for_extruder when processing filament_map_2
    /// Print.cpp:2707
    /// C++: m_config.filament_map_2.values[index] = m_ori_full_print_config.get_index_for_extruder(f_maps[index], "print_extruder_id", ...)
    /// PrintConfig.cpp:7248
    /// C++: const ConfigOptionInts* id_opt = id_name.empty()?nullptr: dynamic_cast<const ConfigOptionInts*>(this->option(id_name));

    // print_extruder_variant: matches filament_extruder_variant format
    auto print_extruder_variant_opt = config.option<Slic3r::ConfigOptionStrings>("print_extruder_variant", true);
    if (!print_extruder_variant_opt || print_extruder_variant_opt->values.empty()) {
        config.set_key_value("print_extruder_variant", new Slic3r::ConfigOptionStrings({"Direct Drive Standard"}));
    }

    // print_extruder_id: IDs for matching (1 for single extruder)
    auto print_extruder_id_opt = config.option<Slic3r::ConfigOptionInts>("print_extruder_id", true);
    if (!print_extruder_id_opt || print_extruder_id_opt->values.empty()) {
        config.set_key_value("print_extruder_id", new Slic3r::ConfigOptionInts({1}));
    }

    // CRITICAL: Initialize ALL keys from filament_options_with_variant to prevent crashes
    // update_values_to_printer_extruders_for_multiple_filaments loops through these keys
    // and calls opt->get_at(variant_index) - if the option doesn't exist or is empty, it crashes/hangs
    /// PrintConfig.cpp:8155-8200
    /// C++: for (auto& key: key_set) { opt->get_at(variant_index[f_index]); }
    // These are the keys from the global filament_options_with_variant set

    auto init_filament_opt_float = [&config](const std::string& key, double default_val) {
        auto opt = config.option<Slic3r::ConfigOptionFloats>(key, true);
        if (!opt || opt->values.empty()) {
            config.set_key_value(key, new Slic3r::ConfigOptionFloats({default_val}));
        }
    };

    auto init_filament_opt_int = [&config](const std::string& key, int default_val) {
        auto opt = config.option<Slic3r::ConfigOptionInts>(key, true);
        if (!opt || opt->values.empty()) {
            config.set_key_value(key, new Slic3r::ConfigOptionInts({default_val}));
        }
    };

    auto init_filament_opt_bool = [&config](const std::string& key, bool default_val) {
        // First try nullable (most filament bool options are nullable)
        auto opt_n = config.option<Slic3r::ConfigOptionBoolsNullable>(key, false);
        if (opt_n) {
            if (opt_n->values.empty()) {
                opt_n->values.push_back((unsigned char)default_val);
            }
            return;
        }
        // Fallback to non-nullable
        auto opt = config.option<Slic3r::ConfigOptionBools>(key, false);
        if (opt) {
            if (opt->values.empty()) {
                opt->values.push_back((unsigned char)default_val);
            }
            return;
        }
        // Option doesn't exist yet — check if definition says nullable
        const auto* def = Slic3r::print_config_def.get(key);
        if (def && def->nullable) {
            config.set_key_value(key, new Slic3r::ConfigOptionBoolsNullable({(unsigned char)default_val}));
        } else {
            config.set_key_value(key, new Slic3r::ConfigOptionBools({default_val}));
        }
    };

    auto init_filament_opt_percent = [&config](const std::string& key, double default_val) {
        auto opt = config.option<Slic3r::ConfigOptionPercents>(key, true);
        if (!opt || opt->values.empty()) {
            config.set_key_value(key, new Slic3r::ConfigOptionPercents({default_val}));
        }
    };

    // Initialize all filament_options_with_variant keys with safe defaults
    // Use correct types: coFloats, coInts, coBools, coPercents
    init_filament_opt_float("filament_flow_ratio", 1.0);  // coFloats, ratio (1.0 = 100%)
    init_filament_opt_float("filament_max_volumetric_speed", 0.0);
    init_filament_opt_float("filament_ramming_volumetric_speed", 0.0);
    init_filament_opt_int("filament_pre_cooling_temperature", 0);
    init_filament_opt_float("filament_ramming_travel_time", 0.0);
    init_filament_opt_float("filament_ramming_volumetric_speed_nc", 0.0);
    init_filament_opt_int("filament_pre_cooling_temperature_nc", 0);
    init_filament_opt_float("filament_ramming_travel_time_nc", 0.0);
    // filament_extruder_variant already initialized above
    init_filament_opt_float("filament_retraction_length", 0.8);
    init_filament_opt_float("filament_retract_length_nc", 0.0);
    init_filament_opt_float("filament_z_hop", 0.0);
    // filament_z_hop_types is enum - skip for now
    init_filament_opt_float("filament_retract_restart_extra", 0.0);
    init_filament_opt_float("filament_retraction_speed", 20.0);
    init_filament_opt_float("filament_deretraction_speed", 20.0);
    init_filament_opt_float("filament_retraction_minimum_travel", 0.0);
    init_filament_opt_bool("filament_retract_when_changing_layer", false);
    init_filament_opt_bool("filament_wipe", false);
    init_filament_opt_float("filament_wipe_distance", 0.0);
    init_filament_opt_percent("filament_retract_before_wipe", 0.0);
    init_filament_opt_bool("filament_long_retractions_when_cut", false);
    init_filament_opt_float("filament_retraction_distances_when_cut", 0.0);
    init_filament_opt_bool("long_retractions_when_ec", false);
    init_filament_opt_float("retraction_distances_when_ec", 0.0);
    // nozzle_temperature and nozzle_temperature_initial_layer already initialized
    init_filament_opt_float("filament_flush_volumetric_speed", 0.0);

    // CRITICAL: Initialize nullable filament/overhang speed options
    // These are ConfigOptionBoolsNullable/FloatsNullable in PrintConfig and will crash
    // with SIGSEGV in PerimeterGenerator::is_enable_overhang_speed if empty
    // PerimeterGenerator.cpp:273-276
    // C++: bool use_filament_overhang_speed = perimeter_generator.print_config->override_process_overhang_speed.get_at(filament_idx);
    init_filament_opt_bool("override_process_overhang_speed", false);
    init_filament_opt_bool("filament_enable_overhang_speed", false);
    init_filament_opt_bool("filament_adaptive_volumetric_speed", false);
    init_filament_opt_float("filament_bridge_speed", 0.0);
    init_filament_opt_float("filament_overhang_1_4_speed", 0.0);
    init_filament_opt_float("filament_overhang_2_4_speed", 0.0);
    init_filament_opt_float("filament_overhang_3_4_speed", 0.0);
    init_filament_opt_float("filament_overhang_4_4_speed", 0.0);
    init_filament_opt_float("filament_overhang_totally_speed", 0.0);

    // Set printer technology
    /// PresetBundle.cpp:3149
    /// C++: out.option<ConfigOptionEnumGeneric>("printer_technology", true)->value = ptFFF;
    config.option<Slic3r::ConfigOptionEnumGeneric>("printer_technology", true)->value = Slic3r::ptFFF;

    // Set minimal G-code templates to prevent export crashes
    /// These are required for GCode::_do_export to work properly
    /// If empty, placeholder parser may fail
    if (!config.has("machine_start_gcode") || config.opt_string("machine_start_gcode").empty()) {
        config.set_key_value("machine_start_gcode", new Slic3r::ConfigOptionString(
            "; Minimal start G-code\n"
            "G28 ; home all axes\n"
            "G1 Z5 F5000 ; lift nozzle\n"
        ));
    }

    if (!config.has("machine_end_gcode") || config.opt_string("machine_end_gcode").empty()) {
        config.set_key_value("machine_end_gcode", new Slic3r::ConfigOptionString(
            "; Minimal end G-code\n"
            "G1 E-1 F300 ; retract\n"
            "G28 X0 Y0 ; home X Y\n"
            "M84 ; disable motors\n"
        ));
    }

    // Ensure temperature settings exist (prevent null pointer access)
    auto ensure_temp_opt = [&config](const std::string& key, int default_val) {
        if (!config.has(key)) {
            config.set_key_value(key, new Slic3r::ConfigOptionInts({default_val}));
        }
    };

    ensure_temp_opt("nozzle_temperature", 200);
    ensure_temp_opt("nozzle_temperature_initial_layer", 200);
    ensure_temp_opt("bed_temperature", 60);
    ensure_temp_opt("bed_temperature_initial_layer", 60);
}

// Ensure critical vector options have minimum sizes
// Call this AFTER loading JSON configs to fix any missing/empty vectors
void ensure_vector_config_sizes(Slic3r::DynamicPrintConfig& config) {
    // Helper: Ensure vector config options exist with at least min_size elements
    // This prevents null pointer crashes in multi-extruder code
    auto ensure_vector_option = [&config](const std::string& key, size_t min_size, const std::string& default_val = "") {
        auto* opt = config.option(key, true);  // Create if doesn't exist
        if (!opt) return;

        if (auto vec_opt = dynamic_cast<Slic3r::ConfigOptionInts*>(opt)) {
            while (vec_opt->values.size() < min_size) {
                vec_opt->values.push_back(vec_opt->values.empty() ? 1 : vec_opt->values[0]);
            }
        } else if (auto vec_opt = dynamic_cast<Slic3r::ConfigOptionFloats*>(opt)) {
            while (vec_opt->values.size() < min_size) {
                vec_opt->values.push_back(vec_opt->values.empty() ? 0.0 : vec_opt->values[0]);
            }
        } else if (auto vec_opt = dynamic_cast<Slic3r::ConfigOptionStrings*>(opt)) {
            while (vec_opt->values.size() < min_size) {
                vec_opt->values.push_back(default_val.empty() ? "" : default_val);
            }
        } else if (auto vec_opt = dynamic_cast<Slic3r::ConfigOptionEnumsGeneric*>(opt)) {
            while (vec_opt->values.size() < min_size) {
                vec_opt->values.push_back(vec_opt->values.empty() ? 0 : vec_opt->values[0]);
            }
        } else if (auto vec_opt = dynamic_cast<Slic3r::ConfigOptionBoolsNullable*>(opt)) {
            // Must check nullable BEFORE non-nullable (nullable doesn't inherit from non-nullable)
            while (vec_opt->values.size() < min_size) {
                vec_opt->values.push_back(vec_opt->values.empty() ? (unsigned char)0 : vec_opt->values[0]);
            }
        } else if (auto vec_opt = dynamic_cast<Slic3r::ConfigOptionBools*>(opt)) {
            while (vec_opt->values.size() < min_size) {
                vec_opt->values.push_back(vec_opt->values.empty() ? (unsigned char)0 : vec_opt->values[0]);
            }
        } else if (auto vec_opt = dynamic_cast<Slic3r::ConfigOptionPercentsNullable*>(opt)) {
            while (vec_opt->values.size() < min_size) {
                vec_opt->values.push_back(vec_opt->values.empty() ? 0.0 : vec_opt->values[0]);
            }
        } else if (auto vec_opt = dynamic_cast<Slic3r::ConfigOptionPercents*>(opt)) {
            while (vec_opt->values.size() < min_size) {
                vec_opt->values.push_back(vec_opt->values.empty() ? 0.0 : vec_opt->values[0]);
            }
        } else if (auto vec_opt = dynamic_cast<Slic3r::ConfigOptionFloatsNullable*>(opt)) {
            while (vec_opt->values.size() < min_size) {
                vec_opt->values.push_back(vec_opt->values.empty() ? 0.0 : vec_opt->values[0]);
            }
        } else if (auto vec_opt = dynamic_cast<Slic3r::ConfigOptionIntsNullable*>(opt)) {
            while (vec_opt->values.size() < min_size) {
                vec_opt->values.push_back(vec_opt->values.empty() ? 0 : vec_opt->values[0]);
            }
        }
    };

    // Ensure all critical vector options have at least 1 element
    // This prevents crashes in multi-extruder code when accessing .get_at(0)
    ensure_vector_option("nozzle_diameter", 1);
    ensure_vector_option("filament_diameter", 1);
    ensure_vector_option("filament_type", 1, "PLA");
    ensure_vector_option("filament_colour", 1, "#FFFFFF");

    // CRITICAL: extruder_type and nozzle_volume_type must be explicitly initialized
    // update_values_to_printer_extruders_for_multiple_filaments crashes if these don't exist or are empty
    /// PrintConfig.cpp:8114-8115, 8129
    /// C++: auto opt_extruder_type = dynamic_cast<const ConfigOptionEnumsGeneric*>(printer_config.option("extruder_type"));
    /// C++: ExtruderType extruder_type = (ExtruderType)(opt_extruder_type->get_at(filament_maps[f_index] - 1));
    // Explicitly create with default values: etDirectDrive (0) and nvtStandard (0)
    if (!config.has("extruder_type") || config.option<Slic3r::ConfigOptionEnumsGeneric>("extruder_type")->values.empty()) {
        config.set_key_value("extruder_type", new Slic3r::ConfigOptionEnumsGeneric({0})); // etDirectDrive
    }
    if (!config.has("nozzle_volume_type") || config.option<Slic3r::ConfigOptionEnumsGeneric>("nozzle_volume_type")->values.empty()) {
        config.set_key_value("nozzle_volume_type", new Slic3r::ConfigOptionEnumsGeneric({0})); // nvtStandard
    }
    // Size per-extruder arrays to match the actual extruder count.
    // ToolOrdering::get_recommended_filament_maps() iterates 0..extruder_nums and
    // accesses nozzle_volume_type.values[idx] and extruder_max_nozzle_count.values[idx]
    // directly (not via get_at()).  On multi-nozzle printers (e.g. H2D with 2 nozzles)
    // these vectors default to size 1, so idx=1 is an OOB read.  On Linux the adjacent
    // heap memory contains allocator metadata; interpreted as a nozzle_count loop bound
    // in build_nozzle_list() this creates thousands of bogus NozzleInfo entries, corrupts
    // the heap, and ultimately crashes as a bad memcpy inside std::string::_M_assign.
    // macOS happens to read zeros there (plausible values) and never trips.
    {
        size_t n_extruders = 1;
        if (auto* nd = config.option<Slic3r::ConfigOptionFloats>("nozzle_diameter"))
            n_extruders = std::max(n_extruders, nd->values.size());
        ensure_vector_option("extruder_type",             n_extruders);
        ensure_vector_option("nozzle_volume_type",        n_extruders);
        ensure_vector_option("extruder_max_nozzle_count", n_extruders);
    }

    // These are already set above in bypass strategy, but ensure they're at least size 1
    ensure_vector_option("filament_map", 1);
    ensure_vector_option("filament_volume_map", 1);
    ensure_vector_option("filament_nozzle_map", 1);
    // filament_map_2 is a 0-based index — pad with 0, not 1.
    {
        auto* opt = config.option<Slic3r::ConfigOptionInts>("filament_map_2", true);
        if (opt && opt->values.empty()) opt->values.push_back(0);
    }
    ensure_vector_option("print_extruder_variant", 1, "Direct Drive Standard");
    ensure_vector_option("print_extruder_id", 1);

    // Required by calc_estimated_filament_print_time()
    // Fallback must be a valid variant string, not a nozzle-diameter string.
    // "Direct Drive Standard" matches get_extruder_variant_string(etDirectDrive, nvtStandard).
    ensure_vector_option("filament_extruder_variant", 1, "Direct Drive Standard");
    ensure_vector_option("filament_self_index", 1);
    ensure_vector_option("filament_max_volumetric_speed", 1);
    ensure_vector_option("filament_flow_ratio", 1);
    ensure_vector_option("printer_extruder_variant", 1, "0.4");

    // Additional printer options that may be accessed
    ensure_vector_option("retract_length", 1);
    ensure_vector_option("retract_lift", 1);
    ensure_vector_option("temperature", 1);
    ensure_vector_option("nozzle_temperature", 1);

    // CRITICAL: Ensure all nullable bool/float filament overhang options have at least 1 element
    // Without these, PerimeterGenerator::is_enable_overhang_speed crashes (SIGSEGV)
    // when calling get_at() on an empty vector
    ensure_vector_option("override_process_overhang_speed", 1);
    ensure_vector_option("filament_enable_overhang_speed", 1);
    ensure_vector_option("filament_adaptive_volumetric_speed", 1);
    ensure_vector_option("filament_bridge_speed", 1);
    ensure_vector_option("filament_overhang_1_4_speed", 1);
    ensure_vector_option("filament_overhang_2_4_speed", 1);
    ensure_vector_option("filament_overhang_3_4_speed", 1);
    ensure_vector_option("filament_overhang_4_4_speed", 1);
    ensure_vector_option("filament_overhang_totally_speed", 1);

    // Other nullable vector options that may be accessed during slicing
    ensure_vector_option("enable_overhang_speed", 1);
    ensure_vector_option("enable_height_slowdown", 1);
    ensure_vector_option("long_retractions_when_cut", 1);
    ensure_vector_option("long_retractions_when_ec", 1);
    ensure_vector_option("retract_before_wipe", 1);
    ensure_vector_option("retraction_length", 1);
    ensure_vector_option("retraction_speed", 1);
    ensure_vector_option("deretraction_speed", 1);
    ensure_vector_option("z_hop", 1);
    ensure_vector_option("travel_speed", 1);
    ensure_vector_option("travel_speed_z", 1);
    ensure_vector_option("outer_wall_speed", 1);
    ensure_vector_option("inner_wall_speed", 1);
    ensure_vector_option("sparse_infill_speed", 1);
    ensure_vector_option("internal_solid_infill_speed", 1);
    ensure_vector_option("top_surface_speed", 1);
    ensure_vector_option("gap_infill_speed", 1);
    ensure_vector_option("support_speed", 1);
    ensure_vector_option("support_interface_speed", 1);
    ensure_vector_option("bridge_speed", 1);
    ensure_vector_option("overhang_totally_speed", 1);
    ensure_vector_option("overhang_1_4_speed", 1);
    ensure_vector_option("overhang_2_4_speed", 1);
    ensure_vector_option("overhang_3_4_speed", 1);
    ensure_vector_option("overhang_4_4_speed", 1);
    ensure_vector_option("filament_flow_ratio", 1);
    ensure_vector_option("filament_max_volumetric_speed", 1);
    ensure_vector_option("filament_ramming_volumetric_speed", 1);
    ensure_vector_option("filament_flush_volumetric_speed", 1);

    // filament_printable is a per-filament bitmask: bit N = filament can be
    // printed on nozzle N (0-based).  The config definition default is 3 (bits
    // 0+1), but FullPrintConfig::defaults() may not propagate that into a
    // DynamicPrintConfig vector option, leaving it empty.  NORM_VEC then pads
    // with 0 ("not printable on any nozzle"), which causes:
    //   "Grouping error: filament1 can not be placed in the right nozzle"
    // For a standalone build we have no nozzle-compatibility data, so set all
    // bits to allow any nozzle assignment.
    {
        auto* fp = config.option<Slic3r::ConfigOptionInts>("filament_printable", true);
        if (!fp || fp->values.empty()) {
            config.set_key_value("filament_printable",
                new Slic3r::ConfigOptionInts({std::numeric_limits<int>::max()}));
        } else {
            for (auto& v : fp->values)
                v = std::numeric_limits<int>::max();
        }
    }
}

// ── Per-filament array alignment (post-load / pre-apply) ─────────────────────
//
// A project 3MF names the loaded filament roster in its
// Metadata/project_settings.config — one entry per spool in filament_colour,
// filament_settings_id, filament_ids and filament_type — while the per-filament
// arrays themselves can still hold the single-slot value this driver seeds.
// BambuStudio's GUI reshapes that flat configuration through
// PresetBundle::full_fff_config(), which fans each filament preset out into the
// assembled vectors (PresetBundle.cpp:161-245); the CLI has no equivalent step
// when profile resolution falls back to the flat 3MF config.
//
// Print::apply() only re-derives those arrays when
// (extruder_count > 1) || support_different_extruders() (PrintApply.cpp:1327-1345,
// via update_values_to_printer_extruders_for_multiple_filaments at
// PrintConfig.cpp:8664).  On a one-nozzle / one-variant machine neither holds,
// so filament_bridge_speed keeps a single element while filament indices run
// 0..filament_count-1.  Consumers that index it with operator[] then read past
// the end of the heap allocation (PrintObject.cpp:366 —
// `print_config.filament_bridge_speed.values[filament_id]`): harmless-looking on
// Linux/macOS, an immediate access violation on Windows with PageHeap enabled.
//
// Every array is only ever EXTENDED, by duplicating its front element — exactly
// the value ConfigOptionVectorBase::get_at() yields out of range
// (Config.hpp:681 clamps to front(); ConfigOptionVector<T>::resize() duplicates
// it, Config.hpp:719).  A config reshaped this way therefore behaves identically
// for every get_at() consumer and for the engine's own variant fan-out; only raw
// `values[i]` indexing changes, from undefined to the value get_at() would have
// returned.  Nothing is truncated, nothing is emptied, and no synthetic zero is
// substituted for a value the project did not have: a populated array only ever
// propagates a value it already carries.

// Filament roster the loaded project declares, 0 when unknown.  filament_colour
// is the authoritative roster (the driver seeds it with one entry, so a longer
// one can only come from the loaded project); the remaining per-filament
// identity vectors are taken as a floor too, because Print::apply() derives its
// own extruder count from filament_diameter (PrintApply.cpp:1504) and every
// per-region filament index it admits must be inside the arrays extended here.
size_t project_filament_count(Slic3r::DynamicPrintConfig& config) {
    size_t count = 0;
    for (const char* key : {"filament_colour", "filament_settings_id", "filament_ids",
                            "filament_type", "filament_diameter"}) {
        // The vector base covers both the plain and the nullable vector spelling
        // of these keys (the two are unrelated types in this engine).
        if (const auto* vec = dynamic_cast<const Slic3r::ConfigOptionVectorBase*>(
                config.option(key, false)))
            count = std::max(count, vec->size());
    }
    return count;
}

// True for the config keys whose arrays carry one entry per FILAMENT rather than
// one per extruder, nozzle or variant slot.  One predicate gates both passes that
// touch these arrays — the extruder-count padding in the front end and the roster
// alignment here — so the two can never disagree about which keys are
// per-filament, the way two hand-maintained key lists drifted apart before.
//
// Per-filament arrays are spelled filament_* (PrintConfig.cpp option table).  Their
// printer-side twins — nozzle_diameter, retraction_length, z_hop, extruder_type,
// nozzle_volume_type, … — carry no prefix and live in a different key set
// (printer_extruder_options / printer_options_with_variant_*, PrintConfig.cpp:7290-7345),
// so this filter can never reach a per-nozzle or per-extruder array and cannot
// flatten a multi-nozzle configuration.
//
// Excluded spellings, each for its own reason:
//   filament_extruder_variant — the variant table; its length is the number
//       of (extruder × nozzle volume) slots that get_parameter_size() sizes
//       printer arrays by (PrintConfig.cpp:7592), so it must not be fanned
//       out to the filament count;
//   filament_self_index — the 1..N owner of the slots in that same table;
//       get_config_index_base() asserts that the two lengths match
//       (PrintConfig.cpp:571) and PrintObject.cpp:342-345 walks one while
//       indexing the other, so it is only extended to a table that already
//       covers the roster (below), never beyond it;
//   filament_map / filament_map_2 — per filament, but 1-based / 0-based
//       indices rather than plain values (aligned by align_per_filament_maps).
bool is_per_filament_config_key(const std::string& key) {
    static const char* const kSpecialFilamentKeys[] = {
        "filament_extruder_variant", "filament_self_index",
        "filament_map", "filament_map_2",
    };
    // Per-filament arrays that live outside the filament_ namespace
    // (filament_options_with_variant, PrintConfig.cpp:7241).
    static const char* const kUnprefixedFilamentKeys[] = {
        "nozzle_temperature", "nozzle_temperature_initial_layer",
        "volumetric_speed_coefficients", "slow_down_min_speed",
        "override_process_overhang_speed",
        "long_retractions_when_ec", "retraction_distances_when_ec",
    };

    if (key.rfind("filament_", 0) == 0) {
        for (const char* special : kSpecialFilamentKeys)
            if (key == special)
                return false;
        return true;
    }
    for (const char* extra : kUnprefixedFilamentKeys)
        if (key == extra)
            return true;
    return false;
}

// Cardinality of the three per-filament index maps — filament_map, filament_map_2
// and filament_nozzle_map — fixed against the roster the project declares, never
// against the machine's head count.  They share the filament axis, so padding any
// of them to the nozzle count invents entries for filaments no identity key
// declares, and the engine then believes in them: it reads filament_map's LENGTH
// as the filament count and fans every per-filament array out to it
// (update_values_to_printer_extruders_for_multiple_filaments, PrintConfig.cpp:8134
// — `size_t filament_count = filament_maps.size()`).  A map longer than the
// roster therefore manufactures the phantom filament downstream, which is why the
// alignment only ever extends toward filament_count and never toward nozzle_count.
//
// Called twice: once before the nozzle-map derivation — which reads filament_map's
// length as the filament count and would otherwise refuse to derive a split the
// project declares, because the driver had seeded the map with a single entry —
// and once in the final alignment, because the plate overlay and the command line
// can rewrite the maps after that.
size_t align_per_filament_maps(Slic3r::DynamicPrintConfig& config, size_t filament_count) {
    if (filament_count <= 1)
        return 0;   // one slot: the flat config already is the whole map

    size_t extended = 0;

    // filament_map maps each filament to its 1-based extruder.  An entry below 1
    // is not a valid extruder and would be read as (size_t)-1 downstream, so
    // repair it onto the first extruder before replicating — the extension must
    // not spread an invalid 0.  Valid entries (including a plate map that points
    // at a different nozzle) are preserved exactly.
    auto* filament_map = config.option<Slic3r::ConfigOptionInts>("filament_map", false);
    if (filament_map && !filament_map->values.empty()) {
        for (int& value : filament_map->values)
            value = std::max(1, value);
        if (filament_map->values.size() < filament_count) {
            filament_map->values.resize(filament_count, filament_map->values.front());
            ++extended;
        }
    }

    // filament_map_2 is the 0-based index mirror of filament_map (see the plate
    // overlay in the driver and PrintApply.cpp:2707).  Existing entries are kept;
    // the entries added here mirror the map that was just aligned.
    auto* filament_map_2 = config.option<Slic3r::ConfigOptionInts>("filament_map_2", false);
    if (filament_map_2 && filament_map && filament_map_2->values.size() < filament_count) {
        const size_t known = filament_map_2->values.size();
        filament_map_2->values.resize(filament_count, 0);
        for (size_t i = known; i < filament_count; ++i)
            filament_map_2->values[i] = std::max(0, filament_map->values[i] - 1);
        ++extended;
    }

    // filament_nozzle_map is the per-filament physical nozzle — the same filament
    // axis as filament_map, and equally exempt from the extruder-count padding.
    // ToolOrdering's Nozzle Manual grouping indexes it by every USED filament
    // (MultiNozzleUtils.cpp:240, req_extruder / input_nozzle_idx), so a map
    // shorter than the roster is an out-of-range read.  Grown with the front
    // value, the fallback get_at() and the engine's own short-map resize use
    // (GCodeProcessor.cpp:1952).
    auto* filament_nozzle_map = config.option<Slic3r::ConfigOptionInts>("filament_nozzle_map", false);
    if (filament_nozzle_map && !filament_nozzle_map->values.empty() &&
        filament_nozzle_map->values.size() < filament_count) {
        filament_nozzle_map->values.resize(filament_count, filament_nozzle_map->values.front());
        ++extended;
    }

    return extended;
}

// filament_count is the roster the caller read out of project_filament_count()
// before anything padded vector options; it is passed in rather than recomputed
// here so values appended by that padding can never be counted as filaments the
// project did not declare.
bool align_per_filament_config_vectors(Slic3r::DynamicPrintConfig& config,
                                       size_t filament_count, bool verbose) {
    if (filament_count <= 1)
        return false;   // one slot: the flat config already is the whole roster

    size_t extended = 0;
    for (const std::string& key : config.keys()) {
        if (!is_per_filament_config_key(key))
            continue;
        auto* vec = dynamic_cast<Slic3r::ConfigOptionVectorBase*>(config.option(key, false));
        if (!vec || vec->size() == 0 || vec->size() >= filament_count)
            continue;
        vec->resize(filament_count);   // duplicates the front value, like get_at()
        ++extended;
    }

    // The per-filament index maps are aligned by the same helper the front end
    // calls before the nozzle-map derivation, so one implementation decides their
    // cardinality in both places.  filament_map has just reached filament_count
    // through the loop above, which leaves the repair of an entry below 1 here;
    // filament_map_2 and filament_nozzle_map are still handled in full.
    extended += align_per_filament_maps(config, filament_count);

    // filament_self_index: only extend to a variant table that already covers the
    // roster.  The slot count can be smaller than the filament count (one slot
    // per extruder × nozzle volume), and growing it past the table would make
    // PrintObject.cpp:342-345 index the shorter variant vector out of range.
    // A one-slot-per-filament table has owners 1..N; extra variant slots have
    // no derivable owner, so preserve the existing get_at() front fallback.
    auto* variant_list = config.option<Slic3r::ConfigOptionStrings>("filament_extruder_variant", false);
    auto* self_index = config.option<Slic3r::ConfigOptionInts>("filament_self_index", false);
    if (variant_list && self_index) {
        const size_t slots = variant_list->values.size();
        if (slots >= filament_count && self_index->values.size() < slots) {
            const size_t known = self_index->values.size();
            self_index->values.resize(slots, 1);
            for (size_t i = known; i < slots; ++i)
                self_index->values[i] = slots == filament_count
                    ? int(i) + 1 : self_index->values.front();
            ++extended;
        }
    }

    // Flush arrays: two purge values per filament, and an N*N*nozzles matrix the
    // wipe-tower generator reads the filament count back out of.  Mirrors
    // PresetBundle::update_multi_material_filament_presets()
    // (PresetBundle.cpp:5542-5585): extend the volume vector to 2*N, and reshape
    // the matrix only when its size does not already match — the stock 4x4 matrix
    // and eight-value vector of a four-filament project are left untouched.
    auto* flush_vector = config.option<Slic3r::ConfigOptionFloats>("flush_volumes_vector", false);
    auto* flush_matrix = config.option<Slic3r::ConfigOptionFloats>("flush_volumes_matrix", false);
    if (flush_vector && !flush_vector->values.empty() &&
        flush_vector->values.size() < 2 * filament_count) {
        // The matrix pairing below reads the vector positionally — even entries
        // from the outgoing filament, odd from the incoming one — so an
        // odd-length vector has to have its half pair completed before whole
        // pairs are appended.  Appending pairs to an odd total overshoots 2*N by
        // one and leaves a stray value the wipe-tower generator never paired
        // with a filament.
        const double unload = flush_vector->values[0];
        const double load   = flush_vector->values.size() > 1 ? flush_vector->values[1] : 140.;
        if (flush_vector->values.size() % 2 != 0)
            flush_vector->values.push_back(load);
        while (flush_vector->values.size() + 2 <= 2 * filament_count) {
            flush_vector->values.push_back(unload);
            flush_vector->values.push_back(load);
        }
        ++extended;
    }
    if (flush_matrix && flush_vector && flush_vector->values.size() >= 2 * filament_count &&
        flush_matrix->values.size() > 0) {
        // Per-nozzle count.  nozzle_diameter is declared coFloats
        // (PrintConfig.cpp:4245) and is stored as ConfigOptionFloats, not the
        // nullable spelling — casting to the nullable sibling yields nullptr and
        // silently collapses a multi-nozzle machine to one nozzle.
        size_t nozzle_count = 1;
        if (const auto* nd = config.option<Slic3r::ConfigOptionFloats>("nozzle_diameter", false))
            nozzle_count = std::max<size_t>(1, nd->values.size());
        const size_t matrix_target = filament_count * filament_count * nozzle_count;

        if (flush_matrix->values.size() < matrix_target) {
            // Shape of the matrix already saved in the project, so the pairs it
            // does describe can be carried over unchanged.  The matrix is one
            // N*N block per nozzle, block after block (get_flush_volumes_matrix,
            // PrintConfig.hpp:1945; GCode.cpp:4752,8090 read it that way), so its
            // LENGTH is what fixes N, and the nozzle count is the machine's own
            // head count — the dimension this reshape is building toward.
            // flush_multiplier is deliberately NOT read here: its length is a
            // per-head hint that a saved project does not keep in step with the
            // matrix (a two-entry multiplier can sit beside a one-nozzle 2x2
            // matrix), and decomposing the matrix by it would read those four
            // saved values as two 1x1 blocks and drop every one of them.
            size_t old_filament_count = 0;
            size_t old_nozzle_count   = 1;
            {
                const size_t old_size = flush_matrix->values.size();

                auto exact_square_root = [](size_t value, size_t& root) {
                    root = 1;
                    while (root * root < value)
                        ++root;
                    return root * root == value;
                };

                size_t square_root   = 1;
                const bool whole_is_square = exact_square_root(old_size, square_root);
                size_t per_head_root = 1;
                const bool per_head_blocks =
                    nozzle_count > 1 && old_size % nozzle_count == 0 &&
                    exact_square_root(old_size / nozzle_count, per_head_root) && per_head_root > 1;

                // Diagonal zero pattern as shape evidence.  The GUI always writes
                // a filament's self-purge as 0, so whichever reading is right has
                // a zero at every diagonal cell it claims.  Bounded to indices the
                // saved LENGTH actually has, so no reading can step past the
                // vector whatever shape it is probed with.
                auto diagonals_all_zero = [&](size_t side, size_t blocks) {
                    for (size_t block = 0; block < blocks; ++block)
                        for (size_t d = 0; d < side; ++d) {
                            const size_t idx = block * side * side + d * side + d;
                            if (idx >= old_size || flush_matrix->values[idx] != 0.)
                                return false;
                        }
                    return true;
                };

                if (whole_is_square && square_root > 1 && square_root >= filament_count) {
                    // One saved square block that covers the active roster — the
                    // reading that keeps every pair the roster can name, and the
                    // one the GUI writes.  Four heads with sixteen saved values and
                    // a roster of three fit a single 4x4 table (pairs for filaments
                    // 0..2 preserved) as easily as four 2x2 head blocks (which keep
                    // only the first two filaments), so the diagonal zeros decide:
                    // four flattened 2x2 blocks put cross-purges at indices 5 and
                    // 10, failing the whole-square diagonal while every per-head
                    // diagonal is zero; a single saved 4x4 passes the whole-square
                    // test and fails the per-head one, whose diagonals land on its
                    // cross-purges (3, 4, 7…).  Per-head takes precedence only on
                    // that one-sided evidence; if both shapes satisfy the test
                    // there is no definitive encoding, so the conservative
                    // whole-square choice stands.  A matrix whose size already
                    // matches the target is never reshaped at all, which is what
                    // tells four right-sized 2x2 head blocks apart.
                    const bool per_head_shape =
                        per_head_blocks && diagonals_all_zero(per_head_root, nozzle_count);
                    if (per_head_shape && !diagonals_all_zero(square_root, 1)) {
                        old_filament_count = per_head_root;
                        old_nozzle_count   = nozzle_count;
                    } else {
                        old_filament_count = square_root;
                    }
                } else if (per_head_blocks) {
                    // Whole N*N blocks per head: exactly the engine's layout, and
                    // the only reading left when the saved square cannot cover the
                    // roster (a block smaller than the roster keeps the pairs it
                    // has per head instead of discarding the head axis).
                    old_filament_count = per_head_root;
                    old_nozzle_count   = nozzle_count;
                } else if (whole_is_square) {
                    // A single-head N*N matrix, whatever else the file claims.
                    // Four values on a four-head machine are one saved 2x2 block
                    // holding all four purge pairs, not four 1x1 blocks; that
                    // reading would keep only the diagonal — the self-pair the GUI
                    // always writes as 0 — and drop every saved cross-filament
                    // value.
                    old_filament_count = square_root;
                } else {
                    // Largest whole block that still divides the length.
                    size_t candidate = 1;
                    while ((candidate + 1) * (candidate + 1) <= old_size)
                        ++candidate;
                    for (; candidate > 1; --candidate)
                        if (old_size % (candidate * candidate) == 0)
                            break;
                    old_filament_count = candidate;
                    old_nozzle_count   = old_size / (candidate * candidate);
                }
            }

            const size_t old_sub = old_filament_count * old_filament_count;
            const size_t new_sub = filament_count * filament_count;
            std::vector<double> rebuilt(matrix_target, 0.);
            for (size_t i = 0; i < filament_count; ++i) {
                for (size_t j = 0; j < filament_count; ++j) {
                    for (size_t nozzle_id = 0; nozzle_id < nozzle_count; ++nozzle_id) {
                        double value;
                        if (nozzle_id < old_nozzle_count &&
                            i < old_filament_count && j < old_filament_count) {
                            value = flush_matrix->values[(i * old_filament_count + j) + old_sub * nozzle_id];
                        } else if (i == j) {
                            value = 0.;   // a filament never purges into itself
                        } else {
                            // Same pairing the GUI uses for a new filament slot:
                            // the unloading volume of i plus the loading volume of j.
                            value = flush_vector->values[2 * i] + flush_vector->values[2 * j + 1];
                        }
                        rebuilt[i * filament_count + j + new_sub * nozzle_id] = value;
                    }
                }
            }
            flush_matrix->values = std::move(rebuilt);
            ++extended;
        }
    }

    if (verbose && extended)
        std::cout << "Filament alignment: " << extended
                  << " array(s) extended to filament_count=" << filament_count << "\n";
    return extended != 0;
}
#endif // ENGINE_BAMBU — BBS-only config-normalization helpers

// Parse a numeric CLI argument, failing fast with usage instead of letting
// std::stod/std::stoi throw an uncaught exception (which would SIGABRT). Used by
// the --calib-* flags so a typo like `--calib-start abc` exits 1 cleanly.
static double parse_cli_double(const char* flag, const char* val, const char* prog) {
    try {
        size_t pos = 0;
        double d = std::stod(val, &pos);
        if (pos != std::string(val).size()) throw std::invalid_argument("trailing");
        return d;
    } catch (const std::exception&) {
        std::cerr << "Error: " << flag << " expects a number, got '" << val << "'\n\n";
        print_usage(prog);
        std::exit(1);
    }
}
static int parse_cli_int(const char* flag, const char* val, const char* prog) {
    try {
        size_t pos = 0;
        int i = std::stoi(val, &pos);
        if (pos != std::string(val).size()) throw std::invalid_argument("trailing");
        return i;
    } catch (const std::exception&) {
        std::cerr << "Error: " << flag << " expects an integer, got '" << val << "'\n\n";
        print_usage(prog);
        std::exit(1);
    }
}

/// The input's kind by its final extension, any case, as both official
/// CLIs and their loaders tell files apart (boost::algorithm::iends_with:
/// BambuStudio.cpp 1873 and Model.cpp 326 at 5873b5f; OrcaSlicer.cpp 1555
/// and Model.cpp 278 at 31f6803). A folder named "x.stl" or a file named
/// "a.3mf.bak" no longer counts.
static bool input_is_stl(const std::string& path) {
    return boost::algorithm::iends_with(path, ".stl");
}

static bool input_is_3mf(const std::string& path) {
    return boost::algorithm::iends_with(path, ".3mf");
}

/// Reads one member of a zip archive (case-insensitive name, either slash).
/// False when the archive or the member cannot be read.
static bool read_zip_member(const std::string& archive, const std::string& member,
                            std::string& content, size_t max_size = 64ULL * 1024 * 1024) {
    mz_zip_archive zip;
    mz_zip_zero_struct(&zip);
    if (!Slic3r::open_zip_reader(&zip, archive))
        return false;
    bool found = false;
    const mz_uint count = mz_zip_reader_get_num_files(&zip);
    for (mz_uint file_idx = 0; file_idx < count && !found; ++file_idx) {
        mz_zip_archive_file_stat stat;
        if (!mz_zip_reader_file_stat(&zip, file_idx, &stat)) continue;
        std::string name(stat.m_filename);
        std::replace(name.begin(), name.end(), '\\', '/');
        if (!boost::algorithm::iequals(name, member)) continue;
        if (stat.m_uncomp_size > max_size) break;
        content.assign(stat.m_uncomp_size, '\0');
        found = mz_zip_reader_extract_to_mem(&zip, file_idx, content.data(), content.size(), 0);
    }
    Slic3r::close_zip_reader(&zip);
    return found;
}

/// Number of plates a Bambu/Orca project declares (one <plate> element each in
/// Metadata/model_settings.config, which is what load_bbs_3mf builds its
/// PlateData list from); 0 when the file declares none.
static int count_3mf_plates(const std::string& path) {
    std::string settings;
    if (!read_zip_member(path, "Metadata/model_settings.config", settings))
        return 0;
    int plates = 0;
    for (size_t pos = settings.find("<plate>"); pos != std::string::npos;
         pos = settings.find("<plate>", pos + 7))
        ++plates;
    return plates;
}

/// Text of every <metadata name="KEY">VALUE</metadata> in a 3MF model part.
static std::map<std::string, std::string> model_metadata(const std::string& model_xml) {
    std::map<std::string, std::string> out;
    size_t pos = 0;
    while ((pos = model_xml.find("<metadata", pos)) != std::string::npos) {
        const size_t tag_end = model_xml.find('>', pos);
        if (tag_end == std::string::npos) break;
        const std::string tag = model_xml.substr(pos, tag_end - pos);
        pos = tag_end + 1;
        if (!tag.empty() && tag.back() == '/') continue;
        const size_t n = tag.find("name=\"");
        if (n == std::string::npos) continue;
        const size_t n_end = tag.find('"', n + 6);
        if (n_end == std::string::npos) continue;
        const size_t close = model_xml.find("</metadata>", pos);
        if (close == std::string::npos) break;
        out[tag.substr(n + 6, n_end - n - 6)] = model_xml.substr(pos, close - pos);
        pos = close;
    }
    return out;
}

/// The file version this engine's 3MF loader reads: OrcaSlicer takes the
/// OrcaSlicer tag when present, else the Application version after
/// "BambuStudio-" or "OrcaSlicer-" (bbs_3mf.cpp 1422-1430, 3945-3961 at
/// 31f6803); BambuStudio reads only "BambuStudio-" (bbs_3mf.cpp 1481 at
/// 5873b5f).
static boost::optional<Slic3r::Semver> engine_file_version(const std::map<std::string, std::string>& meta) {
    const auto app = meta.find("Application");
#ifdef ENGINE_ORCA
    if (const auto orca = meta.find("OrcaSlicer"); orca != meta.end())
        if (auto v = Slic3r::Semver::parse(orca->second)) return v;
    if (app != meta.end()) {
        if (boost::starts_with(app->second, "BambuStudio-")) return Slic3r::Semver::parse(app->second.substr(12));
        if (boost::starts_with(app->second, "OrcaSlicer-"))  return Slic3r::Semver::parse(app->second.substr(11));
    }
#else
    if (app != meta.end() && boost::starts_with(app->second, "BambuStudio-"))
        return Slic3r::Semver::parse(app->second.substr(12));
#endif
    return boost::none;
}

static std::string engine_version_text() {
#ifdef ENGINE_ORCA
    return SoftFever_VERSION;
#else
    return SLIC3R_VERSION;
#endif
}

/// The official version gate (BambuStudio.cpp 1906-1911 at 5873b5f,
/// OrcaSlicer.cpp 1588-1592 at 31f6803): a file whose major.minor is newer
/// than the engine's.
static bool file_newer_than_engine(const Slic3r::Semver& file_version) {
    const auto engine = Slic3r::Semver::parse(engine_version_text());
    if (!engine) return false;
    return engine->maj() < file_version.maj() ||
           (engine->maj() == file_version.maj() && engine->min() < file_version.min());
}

// ── Engine resource roots ────────────────────────────────────────────────
// Both engines read folders that sit beside the profiles tree at slice time:
// Bambu Print.cpp:2687/2722/2755 (info/*.json), FlushVolPredictor.cpp:413 and
// :320 (flush/), ColorDecomposeRecipe.cpp:103 (filament_mixing/); Orca
// Print.cpp:2883/2916 (info/*.json). resources_dir() is an empty global in both
// engines until set_resources_dir() is called, so without the call below both
// binaries open "info/…" relative to the working directory and silently fall
// back to their hardcoded filament/nozzle tables.

/// Directory holding the running executable.
static boost::filesystem::path engine_executable_dir(const char* argv0) {
    boost::filesystem::path exe_dir;
#if defined(_WIN32)
    {
        // The image path, wherever the process was launched from (a PATH lookup
        // by basename leaves argv[0] without a directory).
        wchar_t pathbuf[MAX_PATH + 1];
        DWORD length = GetModuleFileNameW(nullptr, pathbuf, MAX_PATH + 1);
        if (length > 0 && length <= MAX_PATH) {
            try { exe_dir = boost::filesystem::canonical(boost::filesystem::path(pathbuf)).parent_path(); } catch (...) {}
        }
    }
#elif defined(__APPLE__)
    {
        char pathbuf[PATH_MAX];
        uint32_t size = sizeof(pathbuf);
        if (_NSGetExecutablePath(pathbuf, &size) == 0) {
            try { exe_dir = boost::filesystem::canonical(pathbuf).parent_path(); } catch (...) {}
        }
    }
#else
    try { exe_dir = boost::filesystem::canonical("/proc/self/exe").parent_path(); } catch (...) {}
#endif
    // Final fallback: derive from argv[0]
    if (exe_dir.empty()) {
        try { exe_dir = boost::filesystem::canonical(argv0).parent_path(); } catch (...) {}
    }
    return exe_dir;
}

/// The profiles tree the resource folders sit beside, or empty when no known
/// layout is present. Ordered most-specific first: a checkout's references tree
/// (from build/ or build/<config>/),
/// the package layouts — Linux puts the binaries in bin/ with resources/ as its
/// sibling, macOS and Windows put them at the package root next to resources/.
static boost::filesystem::path engine_profiles_dir(const boost::filesystem::path& exe_dir) {
    for (const auto& p : std::vector<boost::filesystem::path>{
        // checkout: build/slicer_cli (single-config) and build/Release/slicer_cli
        // (multi-config generators), both under the repo root
        exe_dir / ".." / "references" / "BambuStudio" / "resources" / "profiles",
        exe_dir / ".." / ".." / "references" / "BambuStudio" / "resources" / "profiles",
        exe_dir / ".." / "resources" / "profiles",
        exe_dir / "resources" / "profiles",
        boost::filesystem::path("/home/user/slicer/references/BambuStudio/resources/profiles"),
    }) {
        if (boost::filesystem::exists(p) && boost::filesystem::is_directory(p))
            return boost::filesystem::canonical(p);
    }
    return boost::filesystem::path();
}

/// The OrcaSlicer profiles tree beside this binary, or empty: the package
/// ships it as resources/profiles-orca (Linux: bin/../resources; macOS and
/// Windows: next to the binaries), a checkout has references/OrcaSlicer.
static boost::filesystem::path orca_profiles_dir(const boost::filesystem::path& exe_dir) {
    for (const auto& p : std::vector<boost::filesystem::path>{
        exe_dir / ".." / "references" / "OrcaSlicer" / "resources" / "profiles",
        exe_dir / ".." / ".." / "references" / "OrcaSlicer" / "resources" / "profiles",
        exe_dir / ".." / "resources" / "profiles-orca",
        exe_dir / "resources" / "profiles-orca",
    }) {
        if (boost::filesystem::exists(p) && boost::filesystem::is_directory(p))
            return boost::filesystem::canonical(p);
    }
    return boost::filesystem::path();
}

/// The printer models a profiles tree offers: every vendor file's
/// machine_model_list (the list PresetBundle::load_vendor_configs_from_json
/// reads its printer models from, PresetBundle.cpp in both engines).
static std::set<std::string> catalog_printer_models(const boost::filesystem::path& profiles_dir) {
    std::set<std::string> models;
    if (profiles_dir.empty()) return models;
    try {
        for (auto& entry : boost::filesystem::directory_iterator(profiles_dir)) {
            if (entry.path().extension() != ".json") continue;
            std::ifstream f(entry.path().string());
            if (!f.is_open()) continue;
            try {
                const json vendor = json::parse(f);
                if (!vendor.is_object() || !vendor.contains("machine_model_list")) continue;
                for (const auto& m : vendor["machine_model_list"])
                    if (m.is_object() && m.contains("name") && m["name"].is_string())
                        models.insert(m["name"].get<std::string>());
            } catch (...) {
            }
        }
    } catch (...) {
    }
    return models;
}

/// Which engine binary of this package has a printer model. "this" is the
/// build running; "other" its sibling in the same package.
struct EngineFit {
    std::string this_app, this_binary, other_app, other_binary;
    bool this_catalog_found  = false;
    bool other_catalog_found = false;
    bool this_has  = false;
    bool other_has = false;
};

static EngineFit engine_fit_for(const std::string& argv0, const std::string& printer_model) {
    const boost::filesystem::path exe_dir = engine_executable_dir(argv0.c_str());
#ifdef ENGINE_ORCA
    const boost::filesystem::path this_dir  = orca_profiles_dir(exe_dir);
    const boost::filesystem::path other_dir = engine_profiles_dir(exe_dir);
    EngineFit fit{"OrcaSlicer", "slicer_cli-orcaslicer", "BambuStudio", "slicer_cli"};
#else
    const boost::filesystem::path this_dir  = engine_profiles_dir(exe_dir);
    const boost::filesystem::path other_dir = orca_profiles_dir(exe_dir);
    EngineFit fit{"BambuStudio", "slicer_cli", "OrcaSlicer", "slicer_cli-orcaslicer"};
#endif
    const std::set<std::string> this_models  = catalog_printer_models(this_dir);
    const std::set<std::string> other_models = catalog_printer_models(other_dir);
    fit.this_catalog_found  = !this_models.empty();
    fit.other_catalog_found = !other_models.empty();
    fit.this_has  = this_models.count(printer_model) > 0;
    fit.other_has = other_models.count(printer_model) > 0;
    return fit;
}

/// The sentence for a printer this engine does not have, naming the binary
/// that has it (owner ruling 10-03: a file for one engine sent to the other
/// is refused, never substituted, and the refusal names the engine that fits).
static std::string engine_mismatch_sentence(const EngineFit& fit, const std::string& printer_model) {
    std::string s = fit.this_app + " has no " + printer_model + " printer; ";
    if (fit.other_has)
        s += "use " + fit.other_binary + " (" + fit.other_app + ").";
    else
        s += "neither engine in this package has it.";
    return s;
}

#ifdef ENGINE_BAMBU
// ── Percent line widths on the Bambu build (cross-engine, same meaning) ──────
// OrcaSlicer declares the ten line-width options coFloatOrPercent with
// ratio_over = "nozzle_diameter" (Orca PrintConfig.cpp at 31f6803) and writes
// them as "100%"; BambuStudio declares the same keys plain coFloat, and its
// ConfigOptionFloat::deserialize reads "100%" as 100 mm, which the engine then
// rejects as "Too large line width" or prints with. The meaning is the same,
// only the unit differs, so the Bambu build converts: the percentage of the
// file's one nozzle diameter, in mm. This is the product's own rule
// (helio-project threemf/canonical/source.rs: percent_line_width_is_engine_foreign,
// normalize_loaded_percent_line_widths, bound_nozzle_diameter_mm) ported:
// a percentage that is not one number, or a project with more than one nozzle
// diameter, is refused naming the setting, never guessed.

static bool percent_line_width_is_engine_foreign(const std::string& key) {
    static const std::string suffix = "_line_width";
    const bool line_width = key == "line_width" ||
        (key.size() > suffix.size() && key.compare(key.size() - suffix.size(), suffix.size(), suffix) == 0);
    if (!line_width) return false;
    const Slic3r::ConfigOptionDef* def = Slic3r::print_config_def.get(key);
    return def != nullptr && def->type == Slic3r::coFloat;
}

enum class PercentShape { Plain, Percent, Malformed };

static PercentShape percent_text_shape(const std::string& text, double& ratio) {
    std::string t = boost::algorithm::trim_copy(text);
    if (t.empty() || t.back() != '%') return PercentShape::Plain;
    t.pop_back();
    boost::algorithm::trim(t);
    try {
        size_t used = 0;
        const double v = std::stod(t, &used);
        if (used != t.size() || !std::isfinite(v)) return PercentShape::Malformed;
        ratio = v;
        return PercentShape::Percent;
    } catch (...) {
        return PercentShape::Malformed;
    }
}

/// A JSON line-width value: a string, or an array of strings that must agree.
static PercentShape percent_json_shape(const json& value, double& ratio) {
    std::vector<json> entries;
    if (value.is_array()) entries.assign(value.begin(), value.end());
    else entries.push_back(value);
    bool any_percent = false, any_plain = false;
    boost::optional<double> agreed;
    for (const json& e : entries) {
        double r = 0.;
        const PercentShape shape = e.is_string() ? percent_text_shape(e.get<std::string>(), r) : PercentShape::Plain;
        if (shape == PercentShape::Malformed) return PercentShape::Malformed;
        if (shape == PercentShape::Plain) { any_plain = true; continue; }
        any_percent = true;
        if (agreed && *agreed != r) return PercentShape::Malformed;
        agreed = r;
    }
    if (!any_percent) return PercentShape::Plain;
    if (any_plain) return PercentShape::Malformed;
    ratio = *agreed;
    return PercentShape::Percent;
}

/// The one nozzle diameter a project states (blank entries skipped), or none
/// when it states none or two different ones (bound_nozzle_diameter_mm).
static boost::optional<double> bound_nozzle_diameter_mm(const json& settings) {
    if (!settings.contains("nozzle_diameter")) return boost::none;
    const json& stated = settings["nozzle_diameter"];
    std::vector<json> entries;
    if (stated.is_array()) entries.assign(stated.begin(), stated.end());
    else entries.push_back(stated);
    boost::optional<double> diameter;
    for (const json& e : entries) {
        double v = 0.;
        if (e.is_null()) continue;
        if (e.is_number()) v = e.get<double>();
        else if (e.is_string()) {
            const std::string t = boost::algorithm::trim_copy(e.get<std::string>());
            if (t.empty()) continue;
            try { size_t used = 0; v = std::stod(t, &used); if (used != t.size()) return boost::none; }
            catch (...) { return boost::none; }
        } else return boost::none;
        if (!std::isfinite(v) || v <= 0.) return boost::none;
        if (diameter && *diameter != v) return boost::none;
        diameter = v;
    }
    return diameter;
}

static std::string width_text(double width) {
    std::ostringstream s;
    s.precision(10);
    s << width;
    return s.str();
}

struct PercentRewrite {
    bool        changed = false;
    std::string refusal;            // non-empty: the sentence the slice stops with
    boost::filesystem::path temp_dir;
    std::string temp_path;          // the converted copy, same file name
    json        converted = json::array();
    double      nozzle_mm = 0.;
    ~PercentRewrite() {
        if (!temp_dir.empty()) {
            boost::system::error_code ignored;
            boost::filesystem::remove_all(temp_dir, ignored);
        }
    }
};

/// One `<metadata key="K" value="V"/>` per line-width key in
/// model_settings.config (per-object and per-part overrides): the value
/// attribute is rewritten, every other byte is kept.
static bool rewrite_metadata_percents(std::string& xml, double nozzle_mm, json& converted,
                                      std::string& refusal, bool apply) {
    bool found = false;
    size_t pos = 0;
    while ((pos = xml.find("<metadata", pos)) != std::string::npos) {
        const size_t end = xml.find('>', pos);
        if (end == std::string::npos) break;
        const size_t k = xml.find("key=\"", pos);
        if (k == std::string::npos || k > end) { pos = end; continue; }
        const size_t k_end = xml.find('"', k + 5);
        const std::string key = xml.substr(k + 5, k_end - k - 5);
        const size_t v = xml.find("value=\"", pos);
        if (v == std::string::npos || v > end || !percent_line_width_is_engine_foreign(key)) { pos = end; continue; }
        const size_t v_start = v + 7;
        const size_t v_end = xml.find('"', v_start);
        if (v_end == std::string::npos || v_end > end) { pos = end; continue; }
        const std::string value = xml.substr(v_start, v_end - v_start);
        double ratio = 0.;
        const PercentShape shape = percent_text_shape(value, ratio);
        if (shape == PercentShape::Plain) { pos = end; continue; }
        if (shape == PercentShape::Malformed) {
            refusal = "Setting '" + key + "' is '" + value + "', which is not one line width.";
            return found;
        }
        found = true;
        if (apply) {
            const double width = ratio / 100. * nozzle_mm;
            if (!std::isfinite(width) || width <= 0.) {
                refusal = "Setting '" + key + "' is '" + value + "', which is not a printable line width.";
                return found;
            }
            const std::string written = width_text(width);
            converted.push_back(json{{"key", key}, {"scope", "object"}, {"from", value}, {"to_mm", width}});
            xml.replace(v_start, v_end - v_start, written);
            pos = v_start + written.size();
        } else {
            pos = end;
        }
    }
    return found;
}

/// Converts the percent line widths of a 3MF into mm in a temporary copy
/// (same file name, so [input_filename_base] in custom G-code is unchanged).
/// A file without percentages is not copied at all.
static void rewrite_percent_line_widths(const std::string& input, PercentRewrite& out) {
    std::string settings_text, model_settings;
    if (!read_zip_member(input, "Metadata/project_settings.config", settings_text))
        return;
    json settings;
    try { settings = json::parse(settings_text); } catch (...) { return; }
    if (!settings.is_object()) return;
    const bool has_model_settings = read_zip_member(input, "Metadata/model_settings.config", model_settings);

    std::vector<std::pair<std::string, double>> globals;
    for (auto& [key, value] : settings.items()) {
        if (!percent_line_width_is_engine_foreign(key)) continue;
        double ratio = 0.;
        const PercentShape shape = percent_json_shape(value, ratio);
        if (shape == PercentShape::Malformed) {
            out.refusal = "Setting '" + key + "' is " + value.dump() + ", which is not one line width.";
            return;
        }
        if (shape == PercentShape::Percent) globals.emplace_back(key, ratio);
    }
    json scratch = json::array();
    bool object_percents = false;
    if (has_model_settings) {
        object_percents = rewrite_metadata_percents(model_settings, 0., scratch, out.refusal, false);
        if (!out.refusal.empty()) return;
    }
    if (globals.empty() && !object_percents) return;

    const boost::optional<double> nozzle = bound_nozzle_diameter_mm(settings);
    const std::string first_key = globals.empty() ? std::string("a per-object line width") : "'" + globals.front().first + "'";
    if (!nozzle) {
        out.refusal = "Setting " + first_key + " is a percentage of the nozzle, and this project states " +
            (settings.contains("nozzle_diameter") ? "more than one nozzle diameter (" + settings["nozzle_diameter"].dump() + ")"
                                                 : std::string("no nozzle diameter")) +
            "; a percentage cannot be turned into one width.";
        return;
    }
    out.nozzle_mm = *nozzle;
    for (const auto& [key, ratio] : globals) {
        const double width = ratio / 100. * *nozzle;
        if (!std::isfinite(width) || width <= 0.) {
            out.refusal = "Setting '" + key + "' is " + settings[key].dump() + ", which is not a printable line width.";
            return;
        }
        out.converted.push_back(json{{"key", key}, {"scope", "project"}, {"from", settings[key]}, {"to_mm", width}});
        settings[key] = width_text(width);
    }
    if (has_model_settings) {
        rewrite_metadata_percents(model_settings, *nozzle, out.converted, out.refusal, true);
        if (!out.refusal.empty()) return;
    }

    // Write the copy: every member as stored, the two settings members rewritten.
    out.temp_dir = boost::filesystem::temp_directory_path() /
                   boost::filesystem::unique_path("slicer_cli_percent-%%%%%%%%");
    boost::filesystem::create_directories(out.temp_dir);
    out.temp_path = (out.temp_dir / boost::filesystem::path(input).filename()).string();
    const std::string new_settings = settings.dump(4);
    mz_zip_archive reader, writer;
    mz_zip_zero_struct(&reader);
    mz_zip_zero_struct(&writer);
    if (!Slic3r::open_zip_reader(&reader, input))
        throw std::runtime_error("cannot reopen " + input);
    if (!Slic3r::open_zip_writer(&writer, out.temp_path)) {
        Slic3r::close_zip_reader(&reader);
        throw std::runtime_error("cannot write " + out.temp_path);
    }
    bool ok = true;
    const mz_uint count = mz_zip_reader_get_num_files(&reader);
    for (mz_uint i = 0; i < count && ok; ++i) {
        mz_zip_archive_file_stat stat;
        if (!mz_zip_reader_file_stat(&reader, i, &stat)) { ok = false; break; }
        std::string name(stat.m_filename);
        std::string norm = name;
        std::replace(norm.begin(), norm.end(), '\\', '/');
        if (boost::algorithm::iequals(norm, "Metadata/project_settings.config"))
            ok = mz_zip_writer_add_mem(&writer, name.c_str(), new_settings.data(), new_settings.size(), MZ_DEFAULT_COMPRESSION);
        else if (has_model_settings && boost::algorithm::iequals(norm, "Metadata/model_settings.config"))
            ok = mz_zip_writer_add_mem(&writer, name.c_str(), model_settings.data(), model_settings.size(), MZ_DEFAULT_COMPRESSION);
        else
            ok = mz_zip_writer_add_from_zip_reader(&writer, &reader, i);
    }
    ok = ok && mz_zip_writer_finalize_archive(&writer);
    Slic3r::close_zip_writer(&writer);
    Slic3r::close_zip_reader(&reader);
    if (!ok)
        throw std::runtime_error("cannot write the converted copy " + out.temp_path);
    out.changed = true;
}
#endif // ENGINE_BAMBU

// Preset staging, both engines: the Bambu 3MF preset rebuild and the
// preset-by-name path (--printer-preset ...) stage a profiles tree the same way.
/// Copy one preset file into the staging tree.
///
/// boost::filesystem::copy_file delegates to the copy_file_range syscall on
/// Linux (selected once from uname in Boost's static init). That syscall
/// reports EXDEV — "Invalid cross-device link" — when source and destination
/// are on different filesystems and the destination has no native copy path:
/// the packaged vendor JSONs on the root filesystem (or a user profiles dir on
/// btrfs) against a TMPDIR on tmpfs (/dev/shm, a tmpfs /tmp) is exactly that
/// case. A stream copy moves the bytes through user space and works there.
///
/// A destination that even the stream copy cannot write is a real staging
/// failure: it throws, and the caller stops the slice instead of continuing on
/// the flat 3MF config.
static void stage_file_copy(const boost::filesystem::path& src,
                            const boost::filesystem::path& dst) {
    boost::system::error_code copy_ec;
    boost::filesystem::copy_file(src, dst, copy_ec);
    if (!copy_ec)
        return;

    const std::string why = copy_ec.message();
    // Boost's path-taking streams open the native path: the wide path on
    // Windows/MSVC, where path::string() would narrow it to the ANSI code page
    // and lose characters outside it. BambuStudio opens a file it copies by
    // size the same way (src/libslic3r/Format/bbs_3mf.cpp:6668 @5873b5f;
    // OrcaSlicer bbs_3mf.cpp:6385 @31f6803).
    boost::filesystem::ifstream in(src, std::ios::binary);
    boost::filesystem::ofstream out(dst, std::ios::binary | std::ios::trunc);
    if (!in.is_open())
        throw std::runtime_error("cannot read " + src.string() + " (copy_file: " + why + ")");
    if (!out.is_open())
        throw std::runtime_error("cannot write " + dst.string() + " (copy_file: " + why + ")");
    // `out << in.rdbuf()` reads through the stream buffer and never sets
    // `in`'s state, and a filebuf need not tell a read error apart from EOF,
    // so a source read that fails part-way could pass as a short, "complete"
    // copy. Copy in a read loop and count the bytes, then compare the count
    // against the source size — the same size check BambuStudio's check_copy()
    // makes on a copied file (src/libslic3r/utils.cpp:977-1006 @5873b5f).
    std::vector<char> buf(64 * 1024);
    std::uintmax_t copied = 0;
    for (;;) {
        in.read(buf.data(), static_cast<std::streamsize>(buf.size()));
        const std::streamsize got = in.gcount();
        if (got > 0) {
            out.write(buf.data(), got);
            copied += static_cast<std::uintmax_t>(got);
        }
        if (!in || !out)
            break;
    }
    out.close();
    if (out.fail())
        throw std::runtime_error("write failed for " + dst.string() + " (copy_file: " + why + ")");
    if (in.bad())
        throw std::runtime_error("read failed for " + src.string() + " (copy_file: " + why + ")");
    boost::system::error_code size_ec;
    const std::uintmax_t src_size = boost::filesystem::file_size(src, size_ec);
    if (size_ec)
        throw std::runtime_error("cannot size " + src.string() + ": " + size_ec.message()
                                 + " (copy_file: " + why + ")");
    if (copied != src_size)
        throw std::runtime_error("read failed for " + src.string() + ": copied "
                                 + std::to_string(copied) + " of " + std::to_string(src_size)
                                 + " bytes (copy_file: " + why + ")");
}

/// Stage one vendor directory under data_dir/system for the PresetBundle
/// rebuild: a directory symlink where the platform grants one, otherwise a
/// copy. Windows refuses CreateSymbolicLink without the symlink privilege or
/// Developer Mode, and boost then throws; a staging failure must not send the
/// slice to the flat 3MF config.
static void stage_vendor_dir(const boost::filesystem::path& src,
                             const boost::filesystem::path& dst) {
    const boost::filesystem::path real_src = boost::filesystem::canonical(src);
    try {
        boost::filesystem::create_directory_symlink(real_src, dst);
        return;
    } catch (const boost::filesystem::filesystem_error&) {
        boost::filesystem::remove(dst);
    }
    boost::filesystem::create_directories(dst);
    for (boost::filesystem::recursive_directory_iterator it(real_src), end; it != end; ++it) {
        const boost::filesystem::path target = dst / boost::filesystem::relative(it->path(), real_src);
        if (boost::filesystem::is_directory(it->path()))
            boost::filesystem::create_directories(target);
        else
            stage_file_copy(it->path(), target);
    }
}
// (end of preset staging helpers)

/// The resources root this engine's libslic3r reads at slice time, or empty
/// when no known layout is present. The two engines' resource files differ
/// (OrcaSlicer's flush/flush_data_standard.txt and info/*.json are not
/// BambuStudio's bytes), so each engine has its own root:
///   Bambu: the parent of the Bambu profiles tree (resources/ in the package,
///          references/BambuStudio/resources in a checkout);
///   Orca:  resources/orca in the package, references/OrcaSlicer/resources in
///          a checkout (build/ or build/<config>/).
static boost::filesystem::path engine_resources_root(const boost::filesystem::path& exe_dir) {
#ifdef ENGINE_ORCA
    for (const auto& p : std::vector<boost::filesystem::path>{
        exe_dir / ".." / "references" / "OrcaSlicer" / "resources",
        exe_dir / ".." / ".." / "references" / "OrcaSlicer" / "resources",
        exe_dir / ".." / "resources" / "orca",
        exe_dir / "resources" / "orca",
    }) {
        if (boost::filesystem::exists(p / "info") && boost::filesystem::is_directory(p / "info"))
            return boost::filesystem::canonical(p);
    }
    return boost::filesystem::path();
#else
    const boost::filesystem::path profiles_dir = engine_profiles_dir(exe_dir);
    if (profiles_dir.empty())
        return boost::filesystem::path();
    return boost::filesystem::canonical(profiles_dir / "..");
#endif
}

/// Point libslic3r at this engine's resources root. The line printed here is
/// the only runtime evidence of which root was chosen; the package tests grep
/// it. It goes to stderr: this runs before the CLI options are read, and the
/// layout-plan subcommand's stdout must stay exactly one JSON document
/// (tests/test_diagnostic_events.sh layout-plan-stdout-stays-one-json-document).
///
/// `quiet` drops the line: `--layout-plan` owns both streams as JSON documents
/// (stdout for the plan, stderr for LayoutErrorV1 — layout_plan.hpp), so that
/// mode configures the root silently.
static void configure_engine_resources(const char* argv0, bool quiet) {
    const boost::filesystem::path root = engine_resources_root(engine_executable_dir(argv0));
    if (root.empty()) {
        if (!quiet)
            std::cerr << "  Engine resources: <not found>\n";
        return;
    }
    Slic3r::set_resources_dir(root.string());
    if (!quiet)
        std::cerr << "  Engine resources: " << root.string() << "\n";
}

// ── Command-line options and per-plate outcome ──────────────────────────
// The default call (file, --plate, -o, --machine/--process/--filament,
// --layout-plan) is what the product drives; every flag added after it only
// changes behaviour when it is passed.
struct CliOptions {
    std::string argv0;
    std::string input_file;
    std::string output_file = "output.gcode";
    std::string machine_config;
    std::string filament_config;
    std::string process_config;
    std::string bundle_config;
    bool verbose = false;
    int  plate_id = 0;  // 0 = all plates (default); >0 = slice only that plate
    bool normalize_legacy_gcode = true;
    std::map<std::string, std::string> overrides;

    // --slice / --outputdir: the official CLI's plate loop and output folder.
    bool        slice_mode  = false;
    int         slice_plate = 0;   // 0 = every plate
    std::string outputdir;
    // Progress events: on with --slice, or asked for with --progress.
    bool        progress    = false;
    // Official overrides of the refusals below (BambuStudio.cpp: allow_newer_file).
    bool        allow_newer_file   = false;
    bool        allow_substitution = false;
    // --arrange 1: place the objects on the bed before slicing (official name).
    bool        arrange = false;
    // Presets by name, resolved like the desktop app (every parent applied).
    std::string printer_preset;
    std::string process_preset;
    std::vector<std::string> filament_presets;
    // --export-3mf NAME: the sliced project, written into --outputdir.
    std::string export_3mf;
    bool uses_presets() const {
        return !printer_preset.empty() || !process_preset.empty() || !filament_presets.empty();
    }
};

// The settings the named presets resolve to, computed once per run (loading a
// profiles tree takes seconds) and applied to every plate.
static std::unique_ptr<Slic3r::DynamicPrintConfig> g_preset_config;

/// What one plate's slice produced, in the shape the official CLI records per
/// plate (sliced_plate_info_t, BambuStudio.cpp 191-219 at 5873b5f).
struct PlateOutcome {
    int         plate_id   = 0;
    int         plate_index = 1;   // 1-based position in this run, for progress
    int         plate_count = 1;   // plates this run slices, for progress
    int         cli_code   = 0;    // CLI_SUCCESS, or the official CLI_* code of the failure
    std::string error_string;      // the official sentence for cli_code, plus specifics
    std::string gcode_path;
    bool        exported   = false;

    // sliced_plate_info_t fields (BambuStudio.cpp 191-219)
    long long   sliced_time_ms = 0;
    std::string warning_message;
    float       total_predication = 0.f;
    float       main_predication  = 0.f;
    int         filament_change_times = 0;
    size_t      triangle_count = 0;
    std::vector<std::pair<std::string, float>> feature_type_times;
    json        objects   = json::array();
    json        filaments = json::array();
    json        warnings  = json::array();   // every slicing warning, not only the last
    std::vector<std::string> unknown_settings;  // keys in the file this engine has no definition for
    // sliced_info_t print summary (BambuStudio.cpp 221-231)
    float       layer_height = 0.f;
    int         wall_loops   = 0;
    float       sparse_infill_density = 0.f;

    // --export-3mf: what the sliced project needs from this plate.
    std::shared_ptr<Slic3r::PlateData>          plate_data;     // slice result, as the GUI stores it
    std::shared_ptr<Slic3r::Model>              export_model;   // STL input: the placed model
    std::shared_ptr<Slic3r::DynamicPrintConfig> export_config;  // STL input: the settings it sliced with
    std::map<size_t, Slic3r::Vec3d>             moved;          // 3MF + --arrange: loaded_id -> offset change
    // Bambu build: routing the slice derived from a supplied filament_nozzle_map
    // (apply_explicit_nozzle_mapping); empty when nothing was derived.
    std::vector<int>                            derived_filament_map;
    int                                         master_filament_slot = -1;  // objects reassigned onto it, or -1
};



// ── Official CLI result codes and sentences ─────────────────────────────────
// The codes are libslic3r's own (Utils.hpp:21-75 in both engines); OrcaSlicer's
// Utils.hpp stops at -102, so the three later Bambu codes are defined here with
// the Bambu values. The sentences are the official CLIs' cli_errors tables
// (BambuStudio.cpp 117-170 at 5873b5f, OrcaSlicer.cpp 111-159 at 31f6803):
// each build reports the sentences of the app it is built from.
#ifndef CLI_FILAMENT_UNPRINTABLE_ON_FIRST_LAYER
#define CLI_FILAMENT_UNPRINTABLE_ON_FIRST_LAYER -103
#endif
#ifndef CLI_GCODE_PATH_OUTSIDE
#define CLI_GCODE_PATH_OUTSIDE             -104
#endif
#ifndef CLI_GCODE_IN_WRAPPING_DETECT_AREA
#define CLI_GCODE_IN_WRAPPING_DETECT_AREA  -105
#endif

static std::string cli_error_sentence(int code) {
#ifdef ENGINE_ORCA
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
        {CLI_GCODE_PATH_IN_UNPRINTABLE_AREA, "Found G-code in unprintable area of multi-extruder printers after slicing. Please make sure the 3mf file can be successfully sliced in the latest " SLICER_APP "."},
        {CLI_FILAMENT_UNPRINTABLE_ON_FIRST_LAYER, "Found some filament unprintable at first layer on current Plate. Please make sure the 3mf file can be successfully sliced with the same Plate type in the latest " SLICER_APP "."},
        {CLI_GCODE_PATH_OUTSIDE, "Found G-code outside of the printable area. The issue may be caused by support, wipe tower, brim, or skirt. If the file slices normally in " SLICER_APP ", try moving the wipe tower further inside the build plate, as we use more conservative parameters for it during upload."},
        {CLI_GCODE_IN_WRAPPING_DETECT_AREA, "Found G-code in the wrapping detect area. Please make sure the 3mf file can be successfully sliced in the latest " SLICER_APP "."},
    };
#undef SLICER_APP
    const auto it = sentences.find(code);
    return it == sentences.end() ? std::string("Failed slicing the model.") : it->second;
}

/// Records a failure on the plate outcome: the official sentence for `code`,
/// then the specific sentence (which object, which setting) when there is one.
static void set_outcome_failure(PlateOutcome& outcome, int code, const std::string& detail = {}) {
    outcome.cli_code = code;
    outcome.error_string = cli_error_sentence(code);
    if (!detail.empty())
        outcome.error_string += " " + detail;
}

/// Formats a length in mm the way a person reads it: no trailing zeros.
static std::string mm_text(double v) {
    std::ostringstream s;
    s.setf(std::ios::fixed);
    s.precision(1);
    s << v;
    std::string t = s.str();
    if (t.size() > 2 && t.compare(t.size() - 2, 2, ".0") == 0) t.resize(t.size() - 2);
    return t;
}

/// result.json in the official shape (record_exit_reson, BambuStudio.cpp
/// 476-593 at 5873b5f). OrcaSlicer's own record (OrcaSlicer.cpp 416-460) is a
/// subset of these keys, so both builds write this one. Keys beyond the
/// official ones: "engine", per-plate "gcode_file" and "warnings" (every
/// slicing warning, where "warning_message" keeps only the last, as the
/// official one does).
static bool write_result_json(const std::string& outputdir, int code, int plate_id,
                              const std::string& error_string,
                              const std::vector<PlateOutcome>& outcomes,
                              long long prepare_time_ms, long long export_time_ms) {
    json j;
    j["plate_index"]  = plate_id;
    j["return_code"]  = code;
    j["error_string"] = error_string;
    j["prepare_time"] = prepare_time_ms;
    j["export_time"]  = export_time_ms;
#ifdef ENGINE_ORCA
    j["engine"] = "orcaslicer";
#else
    j["engine"] = "bambustudio";
#endif
    const PlateOutcome* summary = outcomes.empty() ? nullptr : &outcomes.front();
    j["layer_height"]          = summary ? summary->layer_height : 0.f;
    j["wall_loops"]            = summary ? summary->wall_loops : 0;
    j["sparse_infill_density"] = summary ? summary->sparse_infill_density : 0.f;
    // Settings the file states that this engine has no definition for (the
    // engine ignores them; the GUI lists them as incompatible settings).
    std::set<std::string> unknown;
    for (const PlateOutcome& p : outcomes)
        unknown.insert(p.unknown_settings.begin(), p.unknown_settings.end());
    j["unknown_settings"] = std::vector<std::string>(unknown.begin(), unknown.end());
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
        plate["warnings"]   = p.warnings;
        plate["gcode_file"] = p.gcode_path;
        plate["unknown_settings"] = p.unknown_settings;
        j["sliced_plates"].push_back(plate);
    }
    const std::string path = (boost::filesystem::path(outputdir) / "result.json").string();
    std::ofstream out(path, std::ios::out | std::ios::trunc);
    if (out.is_open())
        out << j.dump(4, ' ', false, json::error_handler_t::replace) << std::endl;
    if (out.is_open() && out.good())
        return true;
    // The official writer swallows this (c.open ... catch (...) {} in record_exit_reson,
    // BambuStudio.cpp 594-599). A run whose result document is missing is
    // not a finished run here: say so, and let the caller fail it.
    std::cerr << "Error: cannot write " << path << "\n";
    emit_event({{"event","output_error"}, {"tag","ResultNotWritten"}, {"path", path},
                {"message","Could not write " + path}});
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

static void emit_progress(const PlateOutcome& outcome, int plate_percent, const std::string& message) {
    const int count = std::max(1, outcome.plate_count);
    const int index = std::max(1, outcome.plate_index);
    const int total = 3 + int(((index - 1) * 90.0) / count + (plate_percent * 0.9) / count);
    emit_progress_event(outcome, total, plate_percent, message);
}

/// A run-level step after the plates: the official CLI's fixed percents
/// (97 "Exporting 3mf", BambuStudio.cpp 8131; 100 "All done, Success", 8237).
static void emit_run_progress(const PlateOutcome& last, int total, const std::string& message) {
    emit_progress_event(last, total, 100, message);
}


/// The bed as a person reads it: "180 x 180 x 180 mm".
static std::string bed_size_text(const Slic3r::DynamicPrintConfig& config) {
    const auto* area = config.option<Slic3r::ConfigOptionPoints>("printable_area");
    if (!area || area->values.empty()) return "unknown";
    Slic3r::BoundingBoxf bed;
    for (const Slic3r::Vec2d& p : area->values) bed.merge(p);
    std::string text = mm_text(bed.size().x()) + " x " + mm_text(bed.size().y());
    if (config.has("printable_height"))
        text += " x " + mm_text(config.opt_float("printable_height"));
    return text + " mm";
}

static Slic3r::BoundingBoxf3 object_world_bbox(const Slic3r::ModelObject* object) {
    Slic3r::BoundingBoxf3 box;
    for (size_t i = 0; i < object->instances.size(); ++i)
        box.merge(object->instance_bounding_box(i));
    return box;
}

static std::string object_size_text(const Slic3r::BoundingBoxf3& box) {
    const Slic3r::Vec3d size = box.size();
    return mm_text(size.x()) + " x " + mm_text(size.y()) + " x " + mm_text(size.z()) + " mm";
}

/// The official per-plate gate before apply (BambuStudio.cpp 6527-6567 at
/// 5873b5f; OrcaSlicer.cpp 5645-5697 at 31f6803): Model::update_print_volume_state
/// against the printer's bed, then refuse an object partly over the edge
/// (CLI_OBJECTS_PARTLY_INSIDE) and a plate with nothing fully inside
/// (CLI_NO_SUITABLE_OBJECTS). The sentence names the object and its size
/// against the bed, which the official result leaves out.
static bool check_objects_inside_bed(Slic3r::Model& model, const Slic3r::DynamicPrintConfig& config,
                                     PlateOutcome& outcome) {
    const auto* area = config.option<Slic3r::ConfigOptionPoints>("printable_area");
    if (!area || area->values.size() < 3)
        return true;   // no bed to check against
    double height = config.has("printable_height") ? config.opt_float("printable_height") : 0.;
    if (height <= 0.) height = 1e6;
    // Per-extruder areas only when the printer states them completely; a
    // padded or partial table would shrink the bed the engine itself uses.
    std::vector<std::vector<Slic3r::Vec2d>> extruder_areas;
    std::vector<double> extruder_heights;
    const auto* groups  = config.option<Slic3r::ConfigOptionPointsGroups>("extruder_printable_area");
    const auto* heights = config.option<Slic3r::ConfigOptionFloatsNullable>("extruder_printable_height");
    if (groups && heights && groups->values.size() > 1 && groups->values.size() == heights->values.size()) {
        bool complete = true;
        for (size_t i = 0; i < groups->values.size(); ++i)
            complete = complete && groups->values[i].size() >= 3 && heights->values[i] > 0.;
        if (complete) {
            extruder_areas = groups->values;
            extruder_heights.assign(heights->values.begin(), heights->values.end());
        }
    }
    Slic3r::BuildVolume build_volume(area->values, height, extruder_areas, extruder_heights);
    const unsigned int inside = model.update_print_volume_state(build_volume);

    for (const Slic3r::ModelObject* object : model.objects) {
        for (size_t i = 0; i < object->instances.size(); ++i) {
            if (object->instances[i]->print_volume_state != Slic3r::ModelInstancePVS_Partly_Outside)
                continue;
            const std::string detail = "Object '" + object->name + "' (" +
                object_size_text(object->instance_bounding_box(i)) +
                ") crosses the edge of the " + bed_size_text(config) + " bed.";
            set_outcome_failure(outcome, CLI_OBJECTS_PARTLY_INSIDE, detail);
            emit_event({{"event","plate_error"}, {"tag","ObjectPartlyOutsideBed"},
                        {"object", object->name}, {"message", detail}});
            return false;
        }
    }
    if (inside == 0) {
        std::string detail;
        for (const Slic3r::ModelObject* object : model.objects) {
            const Slic3r::BoundingBoxf3 box = object_world_bbox(object);
            detail += (detail.empty() ? "" : " ") + std::string("Object '") + object->name + "' is " +
                      object_size_text(box) + ";";
        }
        detail += " the bed is " + bed_size_text(config) + ".";
        set_outcome_failure(outcome, CLI_NO_SUITABLE_OBJECTS, detail);
        emit_event({{"event","plate_error"}, {"tag","NoObjectInsideBed"}, {"message", detail}});
        return false;
    }
    return true;
}


/// --arrange 1: place every object of the plate on the printer's bed, with the
/// arrange kernel `--layout-plan` already uses (get_arrange_polys ->
/// update_arrange_params -> update_selected_items_inflation ->
/// get_shrink_bedpts -> arrangement::arrange, the GUI ArrangeJob pipeline,
/// layout_plan.cpp 624-744), then sit each object on the bed. An object
/// larger than the bed is refused first, with the official -50 sentence plus
/// its size against the bed; objects that do not fit together are refused
/// with the official arrange sentence (-21, BambuStudio.cpp 5938-5944).
/// Rotations stay off, as in Bambu Studio's Arrange by default.
static bool arrange_on_bed(Slic3r::Model& model, const Slic3r::DynamicPrintConfig& config,
                           PlateOutcome& outcome) {
    using namespace Slic3r;
    using namespace Slic3r::arrangement;
    const auto* area = config.option<ConfigOptionPoints>("printable_area");
    if (!area || area->values.size() < 3) {
        set_outcome_failure(outcome, CLI_OBJECT_ARRANGE_FAILED, "The printer states no bed shape (printable_area).");
        return false;
    }
    BoundingBoxf bed;
    for (const Vec2d& pt : area->values) bed.merge(pt);
    const double bed_height = config.has("printable_height") ? config.opt_float("printable_height") : 0.;
    // Each instance on its own: the arrange kernel places every instance as
    // its own item (get_arrange_polys, one ArrangePolygon per instance), so
    // copies that start far apart are not one object wider than the bed.
    for (ModelObject* object : model.objects) {
        object->ensure_on_bed();
        for (size_t i = 0; i < object->instances.size(); ++i) {
            const BoundingBoxf3 box = object->instance_bounding_box(i);
            const Vec3d size = box.size();
            const bool too_wide = size.x() > bed.size().x() + EPSILON || size.y() > bed.size().y() + EPSILON;
            const bool too_tall = bed_height > 0. && size.z() > bed_height + EPSILON;
            if (too_wide || too_tall) {
                const std::string detail = "Object '" + object->name + "' is " + object_size_text(box) +
                                           "; the bed is " + bed_size_text(config) + ".";
                set_outcome_failure(outcome, CLI_NO_SUITABLE_OBJECTS, detail);
                emit_event({{"event","plate_error"}, {"tag","ObjectLargerThanBed"},
                            {"object", object->name}, {"message", detail}});
                return false;
            }
        }
    }

    ArrangeParams params;
    double clearance = 1.0;
    if (config.has("extruder_clearance_max_radius")) {
        const double v = config.opt_float("extruder_clearance_max_radius");
        if (v > 0) clearance = v;
    }
#ifdef ENGINE_ORCA
    params.clearance_radius = clearance;
#else
    params.cleareance_radius = clearance;
#endif
    params.progressind      = [](unsigned, std::string) {};
    params.min_obj_distance = scaled<coord_t>(10.0);
    params.allow_rotations  = false;
    params.do_final_align   = true;

    ModelInstancePtrs instances;
    ArrangePolygons input = get_arrange_polys(model, instances);
#ifdef ENGINE_ORCA
    update_arrange_params(params, &config, input);
    update_selected_items_inflation(input, &config, params);
    Points bed_pts = get_shrink_bedpts(&config, params);
#else
    update_arrange_params(params, config, input);
    update_selected_items_inflation(input, config, params);
    Points bed_pts = get_shrink_bedpts(config, params);
#endif
    arrangement::arrange(input, {}, bed_pts, params);

    std::string off_bed;
    for (const ArrangePolygon& ap : input)
        if (ap.bed_idx != 0)
            off_bed += (off_bed.empty() ? "'" : ", '") + ap.name + "'";
    if (!off_bed.empty()) {
        const std::string detail = "These objects do not fit on the " + bed_size_text(config) +
                                   " bed together: " + off_bed + ".";
        set_outcome_failure(outcome, CLI_OBJECT_ARRANGE_FAILED, detail);
        emit_event({{"event","plate_error"}, {"tag","ArrangeFailed"}, {"message", detail}});
        return false;
    }
    apply_arrange_polys(input, instances, [](ArrangePolygon&) {});
    for (ModelObject* object : model.objects)
        object->ensure_on_bed();
    json placed = json::array();
    for (const ModelObject* object : model.objects) {
        const BoundingBoxf3 box = object_world_bbox(object);
        placed.push_back(json{{"object", object->name},
                              {"center_x_mm", box.center().x()}, {"center_y_mm", box.center().y()}});
    }
    emit_event({{"event","arranged"}, {"tag","ObjectsArranged"}, {"objects", placed},
                {"message","Placed " + std::to_string(model.objects.size()) + " object(s) on the " +
                           bed_size_text(config) + " bed"}});
    return true;
}

/// Per-plate figures the official CLI records after export (BambuStudio.cpp
/// 7262-7356 at 5873b5f): print time, filament per slot in grams, and each
/// object with its bounding box. Orca's statistics carry no per-role times
/// and name the change count total_filament_changes.
static void record_plate_statistics(const Slic3r::Print& print, const Slic3r::Model& model,
                                    const Slic3r::DynamicPrintConfig& config,
                                    const Slic3r::GCodeProcessorResult& result, PlateOutcome& outcome) {
    using Slic3r::PrintEstimatedStatistics;
    const PrintEstimatedStatistics& stat = result.print_statistics;
    const PrintEstimatedStatistics::Mode& mode =
        stat.modes[static_cast<size_t>(PrintEstimatedStatistics::ETimeMode::Normal)];
    outcome.total_predication = mode.time;
    outcome.main_predication  = mode.time - mode.prepare_time;
#ifdef ENGINE_BAMBU
    outcome.filament_change_times = int(stat.total_flush_filament_changes);
    for (const auto& [role, t] : mode.roles_times) {
        if (role == Slic3r::ExtrusionRole::erWipeTower || role == Slic3r::ExtrusionRole::erFlush)
            outcome.main_predication -= t;
        if (t > 0.f)
            outcome.feature_type_times.emplace_back(Slic3r::ExtrusionEntity::role_to_string(role), t);
    }
    for (const auto& [move, t] : mode.moves_times)
        if (move == Slic3r::EMoveType::Travel) {
            if (t > 0.f) outcome.feature_type_times.emplace_back("Travel", t);
            break;
        }
#else
    outcome.filament_change_times = int(stat.total_filament_changes);
#endif
    const auto* filament_ids = dynamic_cast<const Slic3r::ConfigOptionStrings*>(config.option("filament_ids"));
    for (const auto& [extruder, volume] : stat.total_volumes_per_extruder) {
        const double density = extruder < result.filament_densities.size() ? result.filament_densities[extruder] : 1.;
        double main_volume = 0.;
        if (auto it = stat.model_volumes_per_extruder.find(extruder); it != stat.model_volumes_per_extruder.end())
            main_volume += it->second;
        if (auto it = stat.support_volumes_per_extruder.find(extruder); it != stat.support_volumes_per_extruder.end())
            main_volume += it->second;
        outcome.filaments.push_back(json{
            {"id", extruder + 1},
            {"filament_id", filament_ids && extruder < filament_ids->values.size() ? filament_ids->values[extruder] : std::string("unknown")},
            {"total_used_g", 0.001 * volume * density},
            {"main_used_g",  0.001 * main_volume * density}});
    }
    for (const Slic3r::ModelObject* object : model.objects) {
        bool printed = false;
        for (const Slic3r::ModelInstance* inst : object->instances)
            printed = printed || inst->print_volume_state != Slic3r::ModelInstancePVS_Fully_Outside;
        if (!printed) continue;
        const Slic3r::BoundingBoxf3 box = object_world_bbox(object);
        const size_t triangles = object->facets_count();
        outcome.triangle_count += triangles;
        outcome.objects.push_back(json{
            {"id", object->id().id},
            {"name", object->name},
            {"triangle_count", triangles},
            {"bbox", {{"x", box.min.x()}, {"y", box.min.y()}, {"z", box.min.z()},
                      {"width", box.size().x()}, {"depth", box.size().y()}, {"height", box.size().z()}}}});
    }
    if (config.has("layer_height"))          outcome.layer_height = float(config.opt_float("layer_height"));
    if (config.has("wall_loops"))            outcome.wall_loops   = config.opt_int("wall_loops");
    if (const auto* d = config.option<Slic3r::ConfigOptionPercent>("sparse_infill_density"))
        outcome.sparse_infill_density = float(d->value);
    (void)print;
}


/// The slice result of one plate in the shape the GUI stores it into a
/// project (PartPlateList::store_to_3mf_structure, slic3r/GUI/PartPlate.cpp
/// at 5873b5f): G-code file, predicted time, weight, filaments used per slot
/// (PlateData::parse_filament_info), and the processor's per-plate facts.
static void record_plate_for_export(const Slic3r::Print& print, const Slic3r::Model& model,
                                    const Slic3r::DynamicPrintConfig& config,
                                    Slic3r::GCodeProcessorResult& result, const std::string& gcode_file,
                                    bool stl_input, PlateOutcome& outcome) {
    auto pd = std::make_shared<Slic3r::PlateData>();
    pd->plate_index      = std::max(0, outcome.plate_id - 1);
    pd->gcode_file       = gcode_file;
    pd->is_sliced_valid  = true;
    pd->gcode_prediction = std::to_string(int(result.print_statistics
        .modes[static_cast<size_t>(Slic3r::PrintEstimatedStatistics::ETimeMode::Normal)].time));
    pd->toolpath_outside         = result.toolpath_outside;
    pd->timelapse_warning_code   = result.timelapse_warning_code;
    pd->is_label_object_enabled  = result.label_object_enabled;
    pd->limit_filament_maps      = result.limit_filament_maps;
    pd->layer_filaments          = result.layer_filaments;
    pd->filament_change_sequence = result.filament_change_sequence;
    pd->nozzle_change_sequence   = result.nozzle_change_sequence;
    pd->optimal_assignment       = result.optimal_assignment;
    pd->filament_maps            = print.get_filament_maps();
    const double weight = print.print_statistics().total_weight;
    if (weight != 0.) {
        char text[32];
        std::snprintf(text, sizeof text, "%.2f", weight);
        pd->gcode_weight = text;
    }
    pd->is_support_used = print.is_support_used();
    pd->parse_filament_info(&result);
    outcome.plate_data = pd;
    if (stl_input) {
        outcome.export_model  = std::make_shared<Slic3r::Model>(model);
        outcome.export_config = std::make_shared<Slic3r::DynamicPrintConfig>(config);
    }
}

/// The objects a G-code check names: each instance's label id is the one the
/// G-code carries (ModelInstance::get_labeled_id, Model.hpp in both engines);
/// -1 is what the processor records for moves outside any object.
static std::string label_names(const Slic3r::Model& model, const std::set<int>& labels) {
    std::string out;
    for (int label : labels) {
        std::string name;
        if (label < 0) {
            name = "the prime tower, skirt or brim";
        } else {
            for (const Slic3r::ModelObject* object : model.objects)
                for (const Slic3r::ModelInstance* inst : object->instances)
                    if (int(inst->get_labeled_id()) == label) name = "'" + object->name + "'";
            if (name.empty()) name = "object " + std::to_string(label);
        }
        if (out.find(name) != std::string::npos) continue;
        out += (out.empty() ? "" : ", ") + name;
    }
    return out;
}

/// The official checks after export (BambuStudio.cpp 7024-7047, 7187-7243,
/// 7297-7326 at 5873b5f): toolpath conflicts, the critical slicing warnings
/// the official CLI refuses, the G-code check bits and unprintable filaments.
/// Each refusal names what it found. Orca's CLI maps every G-code check bit to
/// CLI_GCODE_PATH_IN_UNPRINTABLE_AREA (OrcaSlicer.cpp 6145-6155); both builds
/// use the finer Bambu mapping here.
static bool post_slice_checks(const Slic3r::Print& print, const Slic3r::Model& model,
                              const Slic3r::GCodeProcessorResult& result, PlateOutcome& outcome) {
    (void)print;
    if (result.conflict_result.has_value()) {
        const auto& c = *result.conflict_result;
        set_outcome_failure(outcome, CLI_GCODE_PATH_CONFLICTS,
            "Toolpaths of '" + c._objName1 + "' and '" + c._objName2 + "' meet at height " +
            mm_text(c._height) + " mm.");
        return false;
    }
    for (const json& w : outcome.warnings) {
        const std::string tag = w.value("tag", "");
        if (w.value("level", "") == "critical" &&
            (tag == "SlicingEmptyGcodeLayers" || tag == "SlicingGcodeOverlap")) {
            set_outcome_failure(outcome, CLI_SLICING_ERROR, w.value("message", ""));
            return false;
        }
    }
    const int code = result.gcode_check_result.error_code;
    if (code != 0 && code != (1 << 11)) {
        std::set<int> labels;
        for (const auto& [extruder, entries] : result.gcode_check_result.print_area_error_infos)
            for (const auto& [filament, label] : entries) labels.insert(label);
        for (const auto& [extruder, entries] : result.gcode_check_result.print_height_error_infos)
            for (const auto& [filament, label] : entries) labels.insert(label);
        std::string where = label_names(model, labels);
        if (code & 0b1100) {
            std::string detail = where.empty()
                ? "The objects themselves are inside the bed, so the cause is the prime tower, a skirt, a brim or support."
                : "It comes from " + where + ".";
            set_outcome_failure(outcome, CLI_GCODE_PATH_OUTSIDE, detail);
        } else if (code & 0b10000) {
            set_outcome_failure(outcome, CLI_GCODE_IN_WRAPPING_DETECT_AREA,
                                where.empty() ? std::string() : "It comes from " + where + ".");
        } else {
            set_outcome_failure(outcome, CLI_GCODE_PATH_IN_UNPRINTABLE_AREA,
                                where.empty() ? std::string() : "It comes from " + where + ".");
        }
        return false;
    }
    if (!result.filament_printable_reuslt.conflict_filament.empty()) {
        std::string slots;
        for (int f : result.filament_printable_reuslt.conflict_filament)
            slots += (slots.empty() ? "" : ", ") + std::to_string(f + 1);
        set_outcome_failure(outcome, CLI_FILAMENT_UNPRINTABLE_ON_FIRST_LAYER,
                            "Filament slot(s) " + slots + " cannot print on the " +
                            result.filament_printable_reuslt.plate_name + " plate.");
        return false;
    }
    return true;
}


// ── Presets by name (desktop-app inheritance) ────────────────────────────────
// The desktop app loads every vendor's system presets with
// PresetBundle::load_system_presets_from_json (BambuStudio PresetBundle.cpp
// 1594 at 5873b5f; OrcaSlicer PresetBundle.cpp 2163 at 31f6803), which
// flattens each preset over its "inherits" chain while it loads
// (load_vendor_configs_from_json), and builds the print's settings with
// PresetBundle::full_config(): defaults, then process, printer and filament
// presets. Neither official CLI does this for --load-settings files: they
// read the one file given and stop at its parent. This path is the desktop's.

/// This engine's profiles tree: Bambu's for the Bambu build, Orca's for Orca.
static boost::filesystem::path this_engine_profiles_dir(const std::string& argv0) {
    const boost::filesystem::path exe_dir = engine_executable_dir(argv0.c_str());
#ifdef ENGINE_ORCA
    return orca_profiles_dir(exe_dir);
#else
    return engine_profiles_dir(exe_dir);
#endif
}

/// Loads this engine's system presets the way the desktop app does: the
/// profiles tree staged as <data_dir>/system, then
/// load_system_presets_from_json. `staging` is removed by the caller's guard.
static bool load_system_presets(const std::string& argv0, Slic3r::PresetBundle& bundle,
                                const boost::filesystem::path& staging, std::string& error,
                                bool document_mode = false) {
    const boost::filesystem::path profiles_dir = this_engine_profiles_dir(argv0);
    if (profiles_dir.empty()) {
        error = "No profiles tree was found beside this binary.";
        return false;
    }
    const boost::filesystem::path sysdir = staging / "system";
    boost::filesystem::create_directories(sysdir);
    for (auto& entry : boost::filesystem::directory_iterator(profiles_dir)) {
        const auto dst = sysdir / entry.path().filename();
        if (boost::filesystem::is_directory(entry.path()))
            stage_vendor_dir(entry.path(), dst);
        else if (entry.path().extension() == ".json" && entry.path().stem() != "blacklist")
            stage_file_copy(entry.path(), dst);
    }
    Slic3r::set_data_dir(staging.string());
    // The desktop app's own entry point (PresetBundle::load_presets, Bambu
    // PresetBundle.cpp 551 / Orca 513): system presets through
    // load_system_presets_from_json, then the (empty) user folder, then the
    // compatibility pass. A fresh AppConfig is what a first launch has.
    Slic3r::AppConfig app_config;
#ifdef ENGINE_ORCA
    bundle.load_presets(app_config, Slic3r::ForwardCompatibilitySubstitutionRule::EnableSilent);
#else
    auto [substitutions, errors] =
        bundle.load_presets(app_config, Slic3r::ForwardCompatibilitySubstitutionRule::EnableSilent);
    (void)substitutions;
    if (!errors.empty()) {
        // --list-presets owns stdout as one JSON document: warnings go to stderr there.
        if (document_mode)
            std::cerr << "Warning: " << errors << "\n";
        else
            emit_event({{"event","preset_warning"}, {"tag","SystemPresetLoadErrors"}, {"message", errors}});
    }
#endif
    return true;
}

/// A vendor preset a person can pick. Templates (instantiation "false") never
/// become presets at all (load_vendor_configs_from_json keeps them only as
/// parents), and the built-in "Default" presets are not system presets.
static bool is_listed_preset(const Slic3r::Preset& preset) {
    return preset.is_system && !preset.is_default;
}

/// Names of presets similar `wanted` (same words), for a refusal that points on.
static std::string near_preset_names(const Slic3r::PresetCollection& collection, const std::string& wanted) {
    std::vector<std::string> words;
    boost::algorithm::split(words, wanted, boost::is_any_of(" @"), boost::token_compress_on);
    std::vector<std::pair<int, std::string>> scored;
    for (const Slic3r::Preset& preset : collection) {
        if (!is_listed_preset(preset)) continue;
        int score = 0;
        for (const std::string& w : words)
            if (!w.empty() && boost::algorithm::icontains(preset.name, w)) ++score;
        if (score > 0) scored.emplace_back(-score, preset.name);
    }
    std::sort(scored.begin(), scored.end());
    std::string out;
    for (size_t i = 0; i < scored.size() && i < 5; ++i)
        out += (i ? "; " : "") + scored[i].second;
    return out;
}

/// Selects the named presets (printer first, then its compatible process and
/// filaments; an omitted process or filament takes the printer's own
/// default_print_profile / default_filament_profile, as the desktop app does
/// when a printer is picked) and returns full_config(). Refuses an unknown
/// name with close matches, and a process or filament the printer cannot use
/// (official CLI_PROCESS_NOT_COMPATIBLE, BambuStudio.cpp at 5873b5f).
/// A preset of this name exists and suits the selected printer.
static bool preset_usable(Slic3r::PresetCollection& collection, const std::string& name) {
    const Slic3r::Preset* p = name.empty() ? nullptr : collection.find_preset(name, false);
    return p && p->name == name && p->is_compatible;
}

/// The compatible preset whose name shares the most words with the
/// default the printer states ("0.20mm Standard @Snapmaker" -> a
/// compatible "0.20mm Standard ..."; "Snapmaker PLA" -> a PLA), so the
/// fallback keeps the default's layer height and material.
static std::string closest_compatible_preset(Slic3r::PresetCollection& collection, const std::string& like) {
    // A printer that states no default gets no guess: the caller asks for
    // --process-preset / --filament-preset by name.
    if (like.empty())
        return {};
    std::vector<std::string> words;
    boost::algorithm::split(words, like, boost::is_any_of(" @()"), boost::token_compress_on);
    std::string best;
    int best_score = -1;
    for (const Slic3r::Preset& p : collection) {
        if (!is_listed_preset(p) || !p.is_compatible) continue;
        int score = 0;
        for (const std::string& w : words)
            if (!w.empty() && boost::algorithm::icontains(p.name, w)) ++score;
        if (score > best_score) { best_score = score; best = p.name; }
    }
    return best;
}

static bool resolve_named_presets(const CliOptions& o, Slic3r::DynamicPrintConfig& out,
                                  int& code, std::string& error) {
    namespace fs = boost::filesystem;
    // The temp folder lookup throws on a bad TMPDIR/TMP; it sits inside the
    // try so that is a refusal with a sentence, never an uncaught exception.
    struct Cleanup { fs::path dir; ~Cleanup() { if (dir.empty()) return; boost::system::error_code e; fs::remove_all(dir, e); } } cleanup;
    Slic3r::PresetBundle bundle;
    try {
        const fs::path staging = fs::temp_directory_path() / fs::unique_path("slicer_cli_named-%%%%%%%%");
        cleanup.dir = staging;
        if (!load_system_presets(o.argv0, bundle, staging, error)) { code = CLI_ENVIRONMENT_ERROR; return false; }
    } catch (const std::exception& e) {
        error = std::string("Loading the system presets failed: ") + e.what();
        code = CLI_ENVIRONMENT_ERROR;
        return false;
    }
#ifdef ENGINE_ORCA
    const std::string app = "OrcaSlicer";
#else
    const std::string app = "BambuStudio";
#endif
    auto find = [&](Slic3r::PresetCollection& collection, const std::string& name, const char* kind) -> bool {
        Slic3r::Preset* preset = collection.find_preset(name, false);
        if (preset && preset->name == name) {
            // A fresh AppConfig has no printer models installed, so system
            // presets load invisible, and select_preset_by_name skips an
            // invisible preset (Preset.cpp 3206 at 5873b5f / 3493 at
            // 31f6803). Naming a preset is installing it, as the desktop
            // app's setup wizard does.
            preset->is_visible = true;
            return true;
        }
        const std::string similar = near_preset_names(collection, name);
        error = app + " has no " + kind + " preset named '" + name + "'." +
                (similar.empty() ? std::string() : " Close names: " + similar + ".") +
                " See --list-presets.";
        code = CLI_CONFIG_FILE_ERROR;
        return false;
    };

    if (o.printer_preset.empty()) {
        error = "--process-preset and --filament-preset need --printer-preset.";
        code = CLI_INVALID_PARAMS;
        return false;
    }
    if (!find(bundle.printers, o.printer_preset, "printer")) return false;
    bundle.printers.select_preset_by_name(o.printer_preset, true);
    if (bundle.printers.get_edited_preset().name != o.printer_preset) {
        error = "Printer preset '" + o.printer_preset + "' could not be selected.";
        code = CLI_CONFIG_FILE_ERROR;
        return false;
    }
    // Compatibility flags against this printer, keeping the selections.
    bundle.update_compatible(Slic3r::PresetSelectCompatibleType::Never);
    const Slic3r::DynamicPrintConfig& printer = bundle.printers.get_edited_preset().config;

    // An omitted process or filament takes the printer's default; when the
    // default is not a compatible preset of this engine's set (the Orca pin's
    // Snapmaker U1 names "0.20mm Standard @Snapmaker", which it does not
    // ship), the first compatible system preset, as the desktop's
    // update_compatible(PresetSelectCompatibleType::Always) replaces one.
    std::vector<std::string> substituted_defaults;
    std::string process = o.process_preset;
    if (process.empty() && printer.has("default_print_profile"))
        process = printer.opt_string("default_print_profile");
    if (o.process_preset.empty() && !preset_usable(bundle.prints, process)) {
        const std::string fallback = closest_compatible_preset(bundle.prints, process);
        if (!fallback.empty()) {
            substituted_defaults.push_back("process '" + fallback + "' (the printer's default '" + process + "' is not available)");
            process = fallback;
        }
    }
    if (process.empty()) {
        error = "Printer preset '" + o.printer_preset + "' names no default process; give --process-preset.";
        code = CLI_INVALID_PARAMS;
        return false;
    }
    if (!find(bundle.prints, process, "process")) return false;
    if (!bundle.prints.find_preset(process, false)->is_compatible) {
        error = "Process preset '" + process + "' is not for printer '" + o.printer_preset +
                "'. See --list-presets --printer \"" + o.printer_preset + "\".";
        code = CLI_PROCESS_NOT_COMPATIBLE;
        return false;
    }
    bundle.prints.select_preset_by_name(process, true);
    if (bundle.prints.get_edited_preset().name != process) {
        error = "Process preset '" + process + "' could not be selected.";
        code = CLI_CONFIG_FILE_ERROR;
        return false;
    }

    std::vector<std::string> filaments = o.filament_presets;
    if (filaments.empty()) {
        std::string wanted;
        if (const auto* d = printer.option<Slic3r::ConfigOptionStrings>("default_filament_profile"))
            if (!d->values.empty()) wanted = d->values.front();
        if (!preset_usable(bundle.filaments, wanted)) {
            const std::string fallback = closest_compatible_preset(bundle.filaments, wanted);
            if (!fallback.empty()) {
                substituted_defaults.push_back("filament '" + fallback + "' (the printer's default '" + wanted + "' is not available)");
                wanted = fallback;
            }
        }
        if (!wanted.empty()) filaments.push_back(wanted);
    }
    if (filaments.empty()) {
        error = "Printer preset '" + o.printer_preset + "' names no default filament; give --filament-preset.";
        code = CLI_INVALID_PARAMS;
        return false;
    }
    for (const std::string& f : filaments) {
        if (!find(bundle.filaments, f, "filament")) return false;
        if (!bundle.filaments.find_preset(f, false)->is_compatible) {
            error = "Filament preset '" + f + "' is not for printer '" + o.printer_preset +
                    "'. See --list-presets --printer \"" + o.printer_preset + "\".";
            code = CLI_PROCESS_NOT_COMPATIBLE;
            return false;
        }
    }
    bundle.filaments.select_preset_by_name(filaments.front(), true);
    if (bundle.filaments.get_edited_preset().name != filaments.front()) {
        error = "Filament preset '" + filaments.front() + "' could not be selected.";
        code = CLI_CONFIG_FILE_ERROR;
        return false;
    }
    bundle.filament_presets = filaments;
    // The desktop app runs this whenever the filament list changes: it sizes
    // the project's flush_volumes_matrix to filaments x filaments per nozzle
    // and flush_multiplier to the nozzle count (PresetBundle.cpp at both
    // pins). Without it a two-nozzle printer (X2D, H2D) reads past the
    // matrix in ToolOrdering::reorder_extruders_for_minimum_flush_volume.
    bundle.update_multi_material_filament_presets();
    filaments = bundle.filament_presets;

    out = bundle.full_config();
    // full_config() writes each filament's own colour; a slot without one
    // keeps the engine default.
    emit_event({{"event","presets_resolved"},
                {"tag","NamedPresetsResolved"},
                {"printer", o.printer_preset},
                {"process", process},
                {"filaments", filaments},
                {"defaults_replaced", substituted_defaults},
                {"message","Settings built from " + app + " system presets with every parent applied" +
                           (substituted_defaults.empty() ? std::string() : "; used " + boost::algorithm::join(substituted_defaults, ", "))}});
    return true;
}

/// --list-presets [--printer NAME]: the system presets of this engine, as one
/// JSON document. With a printer: the processes and filaments it can use,
/// and its defaults.
static int run_list_presets(const CliOptions& o, const std::string& printer_name) {
    namespace fs = boost::filesystem;
    struct Cleanup { fs::path dir; ~Cleanup() { if (dir.empty()) return; boost::system::error_code e; fs::remove_all(dir, e); } } cleanup;
    Slic3r::PresetBundle bundle;
    std::string error;
    json out;
#ifdef ENGINE_ORCA
    out["engine"] = "orcaslicer";
#else
    out["engine"] = "bambustudio";
#endif
    try {
        // Inside the try: the temp folder lookup throws on a bad TMPDIR/TMP.
        const fs::path staging = fs::temp_directory_path() / fs::unique_path("slicer_cli_list-%%%%%%%%");
        cleanup.dir = staging;
        if (!load_system_presets(o.argv0, bundle, staging, error, /*document_mode=*/true)) {
            out["error"] = error;
            std::cout << out.dump(2) << std::endl;
            return CLI_ENVIRONMENT_ERROR;
        }
    } catch (const std::exception& e) {
        out["error"] = std::string("Loading the system presets failed: ") + e.what();
        std::cout << out.dump(2) << std::endl;
        return CLI_ENVIRONMENT_ERROR;
    }
    if (printer_name.empty()) {
        json printers = json::array();
        for (const Slic3r::Preset& p : bundle.printers)
            if (is_listed_preset(p))
                printers.push_back(json{{"name", p.name},
                                        {"printer_model", p.config.has("printer_model") ? p.config.opt_string("printer_model") : std::string()}});
        out["printers"] = printers;
    } else {
        const Slic3r::Preset* printer = bundle.printers.find_preset(printer_name, false);
        if (!printer || printer->name != printer_name) {
            out["error"] = "No printer preset named '" + printer_name + "'.";
            const std::string similar = near_preset_names(bundle.printers, printer_name);
            if (!similar.empty()) out["close_names"] = similar;
            std::cout << out.dump(2) << std::endl;
            return CLI_CONFIG_FILE_ERROR;
        }
        const_cast<Slic3r::Preset*>(printer)->is_visible = true;   // see resolve_named_presets
        bundle.printers.select_preset_by_name(printer_name, true);
        bundle.update_compatible(Slic3r::PresetSelectCompatibleType::Never);
        const Slic3r::DynamicPrintConfig& cfg = bundle.printers.get_edited_preset().config;
        out["printer"] = printer_name;
        // The defaults a slice with only --printer-preset takes: the printer's
        // stated default, or the compatible fallback resolve_named_presets
        // picks when this engine does not ship it, so a listed default is
        // always one --process-preset/--filament-preset accepts.
        json defaults_replaced = json::array();
        auto effective = [&](Slic3r::PresetCollection& collection, const std::string& declared,
                             const char* kind) -> std::string {
            if (declared.empty() || preset_usable(collection, declared)) return declared;
            const std::string fallback = closest_compatible_preset(collection, declared);
            if (fallback.empty()) return declared;
            defaults_replaced.push_back(std::string(kind) + " '" + fallback + "' (the printer's default '" +
                                        declared + "' is not available)");
            return fallback;
        };
        out["default_process"] = effective(bundle.prints,
            cfg.has("default_print_profile") ? cfg.opt_string("default_print_profile") : std::string(), "process");
        json default_filaments = json::array();
        if (const auto* d = cfg.option<Slic3r::ConfigOptionStrings>("default_filament_profile"))
            for (const auto& v : d->values) default_filaments.push_back(effective(bundle.filaments, v, "filament"));
        out["default_filaments"] = default_filaments;
        out["defaults_replaced"] = defaults_replaced;
        json processes = json::array(), filaments = json::array();
        for (const Slic3r::Preset& p : bundle.prints)
            if (is_listed_preset(p) && p.is_compatible) processes.push_back(p.name);
        for (const Slic3r::Preset& p : bundle.filaments)
            if (is_listed_preset(p) && p.is_compatible) filaments.push_back(p.name);
        out["processes"] = processes;
        out["filaments"] = filaments;
    }
    std::cout << out.dump(2, ' ', false, json::error_handler_t::replace) << std::endl;
    return 0;
}

/// The scene origin of plate `index` (0-based) in a project of `plate_count`
/// plates, as the desktop's PartPlateList lays them out (see the call site).
static Slic3r::Vec2d plate_grid_origin(const Slic3r::DynamicPrintConfig& file_config,
                                       const Slic3r::Semver& file_version, int index, int plate_count)
{
    if (index <= 0 || plate_count <= 1)
        return Slic3r::Vec2d::Zero();
    const auto* area = file_config.option<Slic3r::ConfigOptionPoints>("printable_area");
    if (!area || area->values.size() < 4)
        return Slic3r::Vec2d::Zero();
    double width = (int)(area->values[2].x() - area->values[0].x());
    double depth = (int)(area->values[2].y() - area->values[0].y());
    if (file_version.maj() + file_version.min() + file_version.patch() > 0 &&
        file_version < Slic3r::Semver(1, 5, 9)) {
        // + bed3d_ax3s_default_tip_radius, through reset_size(int, int, ...).
        width = (int)(width + 2.5f * 0.5f);
        depth = (int)(depth + 2.5f * 0.5f);
    }
    // compute_colum_count (PartPlate.hpp:38).
    const float value = std::sqrt((float)plate_count);
    const float round_value = std::round(value);
    const int cols = value > round_value ? (int)round_value + 1 : (int)round_value;
    const int row = index / cols, col = index % cols;
    const double gap = 1. / 5.;   // LOGICAL_PART_PLATE_GAP
    return Slic3r::Vec2d(col * width * (1. + gap), -row * depth * (1. + gap));
}

/// The command-line setting overrides (--layer-height, --fill-density, ...)
/// over `config`. The slice reports a rejected value; the --export-3mf
/// rebuild of the same settings stays quiet (the slice already said it).
static void apply_command_line_overrides(Slic3r::DynamicPrintConfig& config,
                                         const std::map<std::string, std::string>& overrides,
                                         bool report_rejections) {
    for (const auto& [key, value] : overrides) {
        try {
            if (key == "layer_height" || key == "nozzle_diameter") {
                config.set_key_value(key, new Slic3r::ConfigOptionFloat(std::stof(value)));
            } else if (key == "fill_density") {
                config.set_key_value(key, new Slic3r::ConfigOptionPercent(std::stoi(value)));
                config.set_key_value("sparse_infill_density", new Slic3r::ConfigOptionPercent(std::stoi(value)));
            } else if (key == "perimeters") {
                config.set_key_value(key, new Slic3r::ConfigOptionInt(std::stoi(value)));
            } else if (key == "nozzle_temperature" || key == "bed_temperature") {
                config.set_key_value(key, new Slic3r::ConfigOptionInts({std::stoi(value)}));
            }
        } catch (const std::exception& e) {
            if (!report_rejections) continue;
            emit_event({{"event","override_rejected"},
                        {"tag","InvalidOverrideValue"},
                        {"opt_key", key},
                        {"value", value},
                        {"message","Command-line override for '" + key + "' was rejected and had no effect"}});
            std::cerr << "Warning: Invalid value for " << key << ": " << value << "\n";
        }
    }
}

/// The selected plate's own settings, as the official CLI lays them over the
/// project's per plate (see the call site): the plate's PlateData::config
/// without the filament-map keys (handled by the plate filament-map blocks
/// of each build: mode always, map in a manual mode) and without keys this
/// engine does not define. Empty
/// without a named plate (plate_id 0 is a whole-file load, not one plate).
static Slic3r::DynamicPrintConfig plate_own_settings(const Slic3r::PlateDataPtrs& plate_data, int plate_id) {
    Slic3r::DynamicPrintConfig plate_config;
    if (plate_id <= 0 || (int)plate_data.size() < plate_id || plate_data[plate_id - 1] == nullptr)
        return plate_config;
    plate_config = plate_data[plate_id - 1]->config;
    for (const char* key : {"filament_map_mode", "filament_map", "filament_volume_map"})
        plate_config.erase(key);
    for (const std::string& key : plate_config.keys())
        if (Slic3r::print_config_def.get(key) == nullptr)
            plate_config.erase(key);
    return plate_config;
}

/// One slice of one plate: load the input, resolve its settings, slice and
/// export G-code to `output_file`. This is the whole single-plate path the
/// default call (`file [--plate N] -o out.gcode`) has always run; `--slice`
/// runs it once per plate (the official CLI's plate loop, BambuStudio.cpp
/// 6437-6505 at 5873b5f). The return value is the process exit code of the
/// default call; `outcome` carries what `--slice` mode reports on top.
static int slice_one_plate(const CliOptions& o, Slic3r::Calib_Params& calib_params,
                           int plate_id, const std::string& output_file, PlateOutcome& outcome) {
    const std::string& input_file      = o.input_file;
    const std::string& machine_config  = o.machine_config;
    const std::string& filament_config = o.filament_config;
    const std::string& process_config  = o.process_config;
    const std::string& bundle_config   = o.bundle_config;
    const bool verbose                 = o.verbose;
    const bool normalize_legacy_gcode  = o.normalize_legacy_gcode;
    const auto& overrides              = o.overrides;
    const bool calib_enabled       = calib_params.mode != Slic3r::CalibMode::Calib_None;
    const bool calib_self_geometry = slicer_cli::calib_mode_generates_geometry(calib_params.mode);
    const auto plate_started = std::chrono::steady_clock::now();
    try {
        // Create configuration BEFORE model loading so load_bbs_3mf can populate it
        // from the embedded Metadata/project_settings.config JSON.
        std::cout << "\nConfiguring print settings...\n";
        Slic3r::DynamicPrintConfig config;

#ifdef ENGINE_BAMBU
        // Start with BambuStudio's full defaults (ensures all keys exist with correct
        // types) PLUS the BBS-specific extruder-variant normalization that coaxes the
        // Bambu engine into accepting a non-Bambu (e.g. Snapmaker U1) printer config.
        set_default_config(config);
#else
        // OrcaSlicer handles non-Bambu printers (U1/Prusa/Voron/…) natively, so none of
        // the BBS variant-array normalization is needed.  Just seed every key with the
        // engine's defaults; load_bbs_3mf (next) overlays the 3MF's project_settings.config.
        config.apply(Slic3r::FullPrintConfig::defaults(), true);
#endif

        // Load model
        std::cout << "Loading model: " << input_file << "\n";
        Slic3r::Model model;
        // Pre-set backup_path to a writable temp dir so the backup manager
        // never touches the read-only /bamboo_model network path.
        model.set_backup_path(boost::filesystem::temp_directory_path().string() + "/slicer_cli_backup");
        bool is_bbl_3mf = false;
        // Values this engine has no word for (unknown enum values), reported
        // together with the out-of-range values at the value check below.
        std::vector<std::string> unknown_values;
        json unknown_items = json::array();
        Slic3r::PlateDataPtrs plate_data;  // hoisted so it is accessible after the 3mf block
        // load_bbs_3mf allocates the plates; free them when this plate's slice
        // ends, so --slice 0 does not keep every plate's load alive
        // (the export keeps its own copy: record_plate_for_export).
        struct PlateDataRelease {
            Slic3r::PlateDataPtrs& plates;
            ~PlateDataRelease() { Slic3r::release_PlateData_list(plates); }
        } plate_data_release{plate_data};
#ifdef ENGINE_BAMBU
        bool explicit_config_supplied_nozzle_map = false;
#endif

        if (input_is_stl(input_file)) {
            bool result = Slic3r::load_stl(input_file.c_str(), &model);
            if (!result) {
                std::cerr << "Failed to load STL file\n";
                set_outcome_failure(outcome, CLI_DATA_FILE_ERROR);
                return 1;
            }
        } else if (input_is_3mf(input_file)) {
#ifdef ENGINE_BAMBU
            Slic3r::ConfigOptionInts accepted_3mf_nozzle_map(
                config.option<Slic3r::ConfigOptionInts>("filament_nozzle_map", true)->values);
            // The file's own filament_nozzle_map is what an earlier slice grouped
            // (written back from Print::get_filament_nozzle_maps, BambuStudio.cpp
            // 7107-7108), not a placement anyone asked for: the official CLI
            // reads a nozzle map only from its command line, and only in a
            // manual mode (BambuStudio.cpp 6789, 6836-6838). So it is kept as
            // the file's value but never makes the mapping explicit; only a
            // --config/--machine/--process/--filament overlay can.
            bbs_3mf_config_contains_nozzle_map(input_file, accepted_3mf_nozzle_map);
#endif
            // The path the engine's 3MF loader reads: the input itself, or on
            // the Bambu build a converted copy when the file states line widths
            // as percentages (see rewrite_percent_line_widths).
            std::string load_path = input_file;
#ifdef ENGINE_BAMBU
            PercentRewrite percent_rewrite;
            rewrite_percent_line_widths(input_file, percent_rewrite);
            if (!percent_rewrite.refusal.empty()) {
                emit_event({{"event","config_refused"},
                            {"tag","PercentLineWidthUnusable"},
                            {"message", percent_rewrite.refusal}});
                std::cerr << "Error: " << percent_rewrite.refusal << "\n";
                set_outcome_failure(outcome, CLI_INVALID_VALUES_IN_3MF, percent_rewrite.refusal);
                return 1;
            }
            if (percent_rewrite.changed) {
                load_path = percent_rewrite.temp_path;
                emit_event({{"event","config_normalized"},
                            {"tag","PercentLineWidthConverted"},
                            {"nozzle_diameter", percent_rewrite.nozzle_mm},
                            {"converted", percent_rewrite.converted},
                            {"message","Converted " + std::to_string(percent_rewrite.converted.size()) +
                                       " line width(s) stated as a percentage of the " +
                                       mm_text(percent_rewrite.nozzle_mm) + " mm nozzle into mm"}});
            }
#endif
            Slic3r::ConfigSubstitutionContext config_subst(Slic3r::ForwardCompatibilitySubstitutionRule::Enable);
            std::vector<Slic3r::Preset*> presets;
            struct PresetsRelease {
                std::vector<Slic3r::Preset*>& presets;
                ~PresetsRelease() { for (Slic3r::Preset* p : presets) delete p; presets.clear(); }
            } presets_release{presets};
            Slic3r::Semver file_version;

            // Pass &config so load_bbs_3mf extracts project_settings.config from the
            // 3MF archive and applies it via config.load_from_json().  Previously this
            // was nullptr which caused load_bbs_3mf to return false immediately
            // (bbs_3mf.cpp:8966 checks config == nullptr).
            //
            // LoadStrategy must include LoadModel (parse geometry) and LoadConfig
            // (parse Metadata/project_settings.config).  AddDefaultInstances ensures
            // each object gets at least one instance.
            auto strategy = Slic3r::LoadStrategy::LoadModel
                          | Slic3r::LoadStrategy::LoadConfig
                          | Slic3r::LoadStrategy::AddDefaultInstances;
#ifdef ENGINE_ORCA
            // OrcaSlicer's load_bbs_3mf inserts a bool* is_orca_3mf between is_bbl_3mf
            // and file_version (and drops the two trailing Bambu-only params).
            bool is_orca_3mf = false;
            bool result = Slic3r::load_bbs_3mf(
                load_path.c_str(),
                &config,
                &config_subst,
                &model,
                &plate_data,
                &presets,
                &is_bbl_3mf,
                &is_orca_3mf,
                &file_version,
                nullptr,   // proFn (progress callback)
                strategy,
                nullptr,   // BBLProject
                plate_id   // 0 = all plates, >0 = specific plate
            );
#else
            bool result = Slic3r::load_bbs_3mf(
                load_path.c_str(),
                &config,
                &config_subst,
                &model,
                &plate_data,
                &presets,
                &is_bbl_3mf,
                &file_version,
                nullptr,   // proFn (progress callback)
                strategy,
                nullptr,   // BBLProject
                plate_id   // 0 = all plates, >0 = specific plate
            );
#endif
            // Surface what the engine silently changed or ignored while
            // deserializing Metadata/project_settings.config. This is the
            // channel the GUI renders as "incompatible settings were
            // substituted", and it is also where an unknown key (a setting the
            // host wrote that this engine has no definition for) is recorded.
            emit_config_substitutions(config_subst, "3mf:project_settings.config");
            outcome.unknown_settings = config_subst.unrecogized_keys;   // sic — upstream spelling

            if (!result) {
                emit_event({{"event","load_error"},
                            {"tag","ThreeMfLoadFailed"},
                            {"path", input_file},
                            {"message","Failed to load 3MF file"}});
                std::cerr << "Failed to load 3MF file\n";
                set_outcome_failure(outcome, CLI_DATA_FILE_ERROR);
                return 1;
            }
            // Engine fit: a project for a printer this engine does not have is
            // refused before anything is sliced. Without the printer's presets
            // the engine slices the flat file against its own defaults (a
            // 200 x 200 bed for a U1 project on the Bambu build) and reports
            // success. Keyed on printer_model, not on the app that made the
            // file: OrcaSlicer ships Bambu Lab printers too, so a Bambu
            // printer's project is valid on both builds.
            // printer_model as the file states it: a loader that does not take
            // the file for its own app's project (the Bambu loader with an
            // "OrcaSlicer-" Application) leaves the setting at the default.
            std::string file_printer_model;
            {
                std::string text;
                if (read_zip_member(input_file, "Metadata/project_settings.config", text)) {
                    try {
                        const json settings = json::parse(text);
                        if (settings.is_object() && settings.contains("printer_model") &&
                            settings["printer_model"].is_string())
                            file_printer_model = settings["printer_model"].get<std::string>();
                    } catch (...) {
                    }
                }
                if (file_printer_model.empty() && config.has("printer_model"))
                    file_printer_model = config.opt_string("printer_model");
            }
            {
                const std::string& printer_model = file_printer_model;
                if (!printer_model.empty()) {
                    const EngineFit fit = engine_fit_for(o.argv0, printer_model);
                    if (fit.this_catalog_found && !fit.this_has) {
                        const std::string sentence = engine_mismatch_sentence(fit, printer_model);
                        json fits = json::array();
                        if (fit.other_has) fits.push_back(fit.other_binary);
                        emit_event({{"event","engine_mismatch"},
                                    {"tag","PrinterModelNotInEngine"},
                                    {"printer_model", printer_model},
                                    {"engine", fit.this_binary},
                                    {"fits", fits},
                                    {"message", sentence}});
                        std::cerr << "Error: " << sentence << "\n";
                        set_outcome_failure(outcome, CLI_3MF_NOT_SUPPORT_MACHINE_CHANGE, sentence);
                        return 1;
                    }
                }
            }
            // The official version gate (BambuStudio.cpp 1906-1911 at 5873b5f,
            // OrcaSlicer.cpp 1588-1592 at 31f6803): a file saved by a newer
            // major.minor than this engine is refused unless --allow-newer-file.
            // The sentence adds what the file says about its maker: a Bambu
            // Studio 02.07 project on the Orca 2.4 build is newer only in the
            // other app's numbering, and the binary to use is the maker's.
            if (!o.allow_newer_file && file_version.maj() + file_version.min() > 0 &&
                file_newer_than_engine(file_version)) {
                std::string model_xml;
                std::map<std::string, std::string> meta;
                if (read_zip_member(input_file, "3D/3dmodel.model", model_xml))
                    meta = model_metadata(model_xml);
                const auto app = meta.find("Application");
                std::string detail = "The file is version " + file_version.to_string() +
                                     (app != meta.end() ? " (" + app->second + ")" : std::string()) +
                                     "; this engine is " + engine_version_text() + ".";
#ifdef ENGINE_ORCA
                if (meta.count("OrcaSlicer") == 0 && app != meta.end() &&
                    boost::starts_with(app->second, "BambuStudio-"))
                    detail += " It was made by Bambu Studio: use slicer_cli (BambuStudio).";
#endif
                emit_event({{"event","config_refused"},
                            {"tag","FileVersionNewerThanEngine"},
                            {"file_version", file_version.to_string()},
                            {"engine_version", engine_version_text()},
                            {"message", cli_error_sentence(CLI_FILE_VERSION_NOT_SUPPORTED) + " " + detail}});
                std::cerr << "Error: " << cli_error_sentence(CLI_FILE_VERSION_NOT_SUPPORTED) << " " << detail << "\n";
                set_outcome_failure(outcome, CLI_FILE_VERSION_NOT_SUPPORTED, detail);
                return 1;
            }
            // A setting whose value this engine has no meaning for is a
            // different print from the one the file states, so it is refused,
            // naming the setting, the value and the values this engine has
            // (owner ruling 10-03: meaning differs -> refuse). Only enum
            // substitutions: those are the ones where the engine swapped in
            // its own default for a word it does not know. --allow-substitution
            // keeps the old behaviour (slice with the engine's substitute).
            // Step 0 (2026-10-03 corpus): the product's own Mode A retarget of
            // an OrcaSlicer-made project to a Bambu printer still carries
            // Orca words (ensure_all, rectilinear) that the default call has
            // always substituted. The product's call must not change, so the
            // refusal is part of the --slice mode only.
            if (o.slice_mode && !o.allow_substitution) {
                std::vector<std::string>& refused = unknown_values;
                json& items = unknown_items;
                for (const auto& sub : config_subst.substitutions) {
                    if (!sub.opt_def) continue;
                    const auto type = sub.opt_def->type;
                    if (type != Slic3r::coEnum && type != Slic3r::coEnums) continue;
                    std::string allowed;
                    if (sub.opt_def->enum_keys_map)
                        for (const auto& [name, value] : *sub.opt_def->enum_keys_map)
                            allowed += (allowed.empty() ? "" : ", ") + name;
                    refused.push_back("'" + sub.opt_def->opt_key + "' is '" + sub.old_value +
                                      "', which this engine does not have (its values: " + allowed + ")");
                    items.push_back(json{{"opt_key", sub.opt_def->opt_key}, {"value", sub.old_value},
                                         {"allowed", allowed}});
                }
            }
#ifdef ENGINE_BAMBU
            // The importer may partially mutate a vector before rejecting it.
            // Retain the last usable input map, or the original driver default
            // when no usable map was supplied. This does not establish provenance.
            config.set_key_value("filament_nozzle_map", accepted_3mf_nozzle_map.clone());
#endif
            // Validate --plate against actual plate count
            if (plate_id > 0 && (int)plate_data.size() < plate_id) {
                emit_event({{"event","input_error"},
                            {"tag","PlateOutOfRange"},
                            {"requested_plate", plate_id},
                            {"plate_count", (int)plate_data.size()},
                            {"message","--plate " + std::to_string(plate_id) +
                                       " but the 3MF has only " +
                                       std::to_string(plate_data.size()) + " plate(s)"}});
                std::cerr << "Error: --plate " << plate_id
                          << " but 3MF only has " << plate_data.size() << " plate(s)\n";
                set_outcome_failure(outcome, CLI_INVALID_PARAMS);
                return 1;
            }

            // ── Multi-plate coordinate translation ───────────────────────
            // A Bambu/Orca 3MF stores every object at its scene position: plate
            // i (0-based) sits at the grid origin the desktop's PartPlateList
            // gives it, and the CLI slices that plate with its print origin set
            // there.  Port of that rule (pins 5873b5f / 31f6803):
            //   - plate size = (int)(printable_area[2] - printable_area[0]) from
            //     the FILE's config (BambuStudio.cpp 2054-2056), plus
            //     bed3d_ax3s_default_tip_radius (2.5 * 0.5 mm) for files older
            //     than 1.5.9 (BambuStudio.cpp 1911-1914, 4316-4318; OrcaSlicer.cpp
            //     3727-3729);
            //   - cols = compute_colum_count(plate count) (PartPlate.hpp:38);
            //   - origin(i) = (col * w * 1.2, -row * d * 1.2) with
            //     LOGICAL_PART_PLATE_GAP = 1/5 (PartPlate.cpp:55, 4343-4355, 5240-5254).
            // We keep the print origin at 0 and move the objects by -origin, which
            // is the same placement.  The objects stay where the maker put them on
            // their plate: no snap to the bed corner, no plate_N.json bbox guess
            // (both moved parts, and on beds with an exclusion area the corner
            // snap pushed them into it).
            if (plate_id > 0 && is_bbl_3mf && !model.objects.empty()) {
                bool is_seq_print_plate = false;
                {
                    mz_zip_archive zip;
                    mz_zip_zero_struct(&zip);
                    if (Slic3r::open_zip_reader(&zip, input_file)) {
                        std::string plate_json_path = "Metadata/plate_" + std::to_string(plate_id) + ".json";
                        int file_idx = mz_zip_reader_locate_file(&zip, plate_json_path.c_str(), nullptr, 0);
                        if (file_idx >= 0) {
                            mz_zip_archive_file_stat stat;
                            if (mz_zip_reader_file_stat(&zip, file_idx, &stat)) {
                                std::string content(stat.m_uncomp_size, '\0');
                                mz_zip_reader_extract_to_mem(&zip, file_idx, content.data(), content.size(), 0);
                                try {
                                    auto plate_json = json::parse(content);
                                    if (plate_json.contains("is_seq_print"))
                                        is_seq_print_plate = plate_json["is_seq_print"].get<bool>();
                                } catch (...) {}
                            }
                        }
                        mz_zip_reader_end(&zip);
                    }
                }

                const Slic3r::Vec2d origin = plate_grid_origin(config, file_version, plate_id - 1, (int)plate_data.size());
                if (verbose)
                    std::cout << "Plate " << plate_id << " of " << plate_data.size()
                              << ": grid origin (" << origin.x() << ", " << origin.y() << ")\n";
                if (origin.x() != 0.0 || origin.y() != 0.0) {
                    for (auto* obj : model.objects)
                        for (auto* inst : obj->instances) {
                            Slic3r::Vec3d off = inst->get_offset();
                            inst->set_offset(Slic3r::Vec3d(off.x() - origin.x(), off.y() - origin.y(), off.z()));
                        }
                }

                // Apply sequential print flag from plate metadata
                if (is_seq_print_plate) {
                    Slic3r::ConfigSubstitutionContext seq_subst(
                        Slic3r::ForwardCompatibilitySubstitutionRule::Enable);
                    config.set_deserialize("print_sequence", "by object", seq_subst);
                    if (verbose)
                        std::cout << "Plate " << plate_id << " uses sequential (by-object) printing\n";
                }
            }

#ifdef ENGINE_BAMBU
            // ── Rebuild config via PresetBundle ──────────────────────────
            // The 3MF's project_settings.config is a merged flat file with
            // multi-element arrays from the printer profile's variant support.
            // BambuStudio desktop builds config from individual presets via
            // PresetBundle::full_config(), producing correctly-sized vectors.
            // Replicate that here: load system presets → select by name →
            // full_config() → overlay any 3MF-only keys.
            //
            // If this fails (profiles not found, names don't match, etc.)
            // we silently fall back to the flat 3MF config from load_bbs_3mf.
            {
                // Same executable dir and profiles tree that
                // configure_engine_resources() resolved at startup, including
                // the package-root layout. exe_dir is kept for the
                // preset_resolution_failed diagnostic below.
                const boost::filesystem::path exe_dir = engine_executable_dir(o.argv0.c_str());
                const boost::filesystem::path profiles_dir = engine_profiles_dir(exe_dir);

                bool preset_loaded = false;
                if (!profiles_dir.empty()) {
                    try {
                        // Prepare data_dir/system/ with symlinks to vendor dirs.
                        // Always start fresh to avoid stale cached copies of
                        // vendor JSON files that can cause extruder variant
                        // lookup failures (see update_values_to_printer_extruders).
                        // One staging directory per process: two packaged slices
                        // running at once must not delete each other's vendor
                        // files mid-load (a shared fixed path with remove_all did).
                        //
                        // The cleanup guard lives outside the fatal try below so
                        // the staged tree survives the PresetBundle load; it gets
                        // its path before create_directories so a half-created
                        // staging dir is still removed.
                        boost::filesystem::path tmpdir;
                        boost::filesystem::path sysdir;
                        struct StagingCleanup {
                            boost::filesystem::path dir;
                            ~StagingCleanup() {
                                if (dir.empty())
                                    return;
                                boost::system::error_code ignored;
                                boost::filesystem::remove_all(dir, ignored);
                            }
                        } staging_cleanup;
                        // A staging step that fails — creating the staging dir
                        // or a copy that even the stream-copy fallback cannot
                        // complete — is fatal: the same event + exit-code shape
                        // as the other preset failures below, except the slice
                        // stops here instead of running on the flat 3MF config
                        // with no presets at all.
                        try {
                            tmpdir = boost::filesystem::temp_directory_path()
                                / boost::filesystem::unique_path("slicer_cli_presets-%%%%%%%%");
                            sysdir = tmpdir / "system";
                            staging_cleanup.dir = tmpdir;
                            boost::filesystem::create_directories(sysdir);
                            for (auto& entry : boost::filesystem::directory_iterator(profiles_dir)) {
                                auto dst = sysdir / entry.path().filename();
                                if (boost::filesystem::is_directory(entry.path()))
                                    stage_vendor_dir(entry.path(), dst);
                                else if (entry.path().extension() == ".json")
                                    stage_file_copy(entry.path(), dst);
                            }
                        } catch (const std::exception& e) {
                            emit_event({{"event","preset_error"},
                                        {"tag","PresetStagingFailed"},
                                        {"exe_dir", exe_dir.empty() ? std::string{} : exe_dir.string()},
                                        {"profiles_dir", profiles_dir.string()},
                                        {"message", std::string("Preset staging failed: ") + e.what()}});
                            std::cerr << "  Preset staging failed: " << e.what() << "\n";
                            set_outcome_failure(outcome, CLI_ENVIRONMENT_ERROR, e.what());
                            return 1;
                        }
                        // resources_dir() already points at profiles_dir/.. — set
                        // once for both engines by configure_engine_resources().
                        Slic3r::set_data_dir(tmpdir.string());

                        Slic3r::PresetBundle preset_bundle;

                        // Load ALL vendors (BBL, Creality, Qidi, …)
                        // First vendor uses LoadSystem (resets bundle),
                        // subsequent vendors just load without reset.
                        bool first_vendor = true;
                        for (auto& dir_entry : boost::filesystem::directory_iterator(sysdir)) {
                            if (dir_entry.path().extension() != ".json")
                                continue;
                            std::string vname = dir_entry.path().stem().string();
                            if (vname == "blacklist") continue;
                            try {
                                if (first_vendor) {
                                    preset_bundle.load_vendor_configs_from_json(
                                        sysdir.string(), vname,
                                        Slic3r::PresetBundle::LoadSystem,
                                        Slic3r::ForwardCompatibilitySubstitutionRule::EnableSilent);
                                    first_vendor = false;
                                } else {
                                    // load_vendor_configs_from_json without LoadSystem
                                    // won't reset — it appends to existing presets
                                    preset_bundle.load_vendor_configs_from_json(
                                        sysdir.string(), vname,
                                        Slic3r::PresetBundle::LoadConfigBundleAttributes(),
                                        Slic3r::ForwardCompatibilitySubstitutionRule::EnableSilent);
                                }
                            } catch (...) {
                                // Skip vendors that fail to load
                            }
                        }

                        // Read preset names from the 3MF config
                        std::string printer_name = config.opt_string("printer_settings_id");
                        std::string print_name   = config.opt_string("print_settings_id");
                        std::string filament_name;
                        if (auto* fsi = config.option<Slic3r::ConfigOptionStrings>("filament_settings_id"))
                            if (!fsi->values.empty())
                                filament_name = fsi->values[0];

                        std::cout << "  Preset lookup: printer='" << printer_name
                                  << "' print='" << print_name
                                  << "' filament='" << filament_name << "'\n";

                        if (!printer_name.empty() && !print_name.empty() && !filament_name.empty()) {
                            bool ok_printer  = preset_bundle.printers.select_preset_by_name(printer_name, true);
                            bool ok_print    = preset_bundle.prints.select_preset_by_name(print_name, true);
                            bool ok_filament = preset_bundle.filaments.select_preset_by_name(filament_name, true);

                            std::cout << "  Preset match: printer=" << ok_printer
                                      << " (resolved='" << preset_bundle.printers.get_edited_preset().name << "')"
                                      << " print=" << ok_print
                                      << " (resolved='" << preset_bundle.prints.get_edited_preset().name << "')"
                                      << " filament=" << ok_filament
                                      << " (resolved='" << preset_bundle.filaments.get_edited_preset().name << "')\n";

                            // Only use PresetBundle config if ALL three presets were found
                            // (not just the default fallbacks)
                            if (ok_printer && ok_print && ok_filament
                                && preset_bundle.printers.get_edited_preset().name == printer_name
                                && preset_bundle.prints.get_edited_preset().name == print_name
                                && preset_bundle.filaments.get_edited_preset().name == filament_name)
                            {
                                // Same flow as BambuStudio desktop:
                                // full_config() builds defaults → printer → process → filament
                                // then apply project_settings.config on top.
                                Slic3r::DynamicPrintConfig base_config = preset_bundle.full_config();
                                base_config.apply(config, /*ignore_nonexistent=*/true);
                                config = std::move(base_config);
                                preset_loaded = true;

                                if (verbose)
                                    std::cout << "  Presets: " << printer_name
                                              << " / " << print_name
                                              << " / " << filament_name << "\n";
                            }
                        }
                    } catch (const std::exception& e) {
                        emit_event({{"event","preset_error"},
                                    {"tag","PresetBundleException"},
                                    {"message", std::string("PresetBundle exception: ") + e.what()}});
                        std::cerr << "  PresetBundle exception: " << e.what() << "\n";
                    }
                }
                if (!preset_loaded) {
                    // The slice is about to run on the flat, merged 3MF config
                    // instead of a preset-resolved one. The desktop app can
                    // never be in this state, so nothing downstream expects it;
                    // it must not stay a plain stdout line.
                    emit_event({{"event","preset_resolution_failed"},
                                {"tag","FlatThreeMfConfigFallback"},
                                {"exe_dir", exe_dir.empty() ? std::string{} : exe_dir.string()},
                                {"profiles_dir", profiles_dir.empty() ? std::string{} : profiles_dir.string()},
                                {"message","Presets were not resolved; slicing from the flat 3MF config"}});
                    std::cout << "  WARNING: Using flat 3MF config (presets not resolved)\n";
                    if (!exe_dir.empty())
                        std::cout << "  exe_dir: " << exe_dir << "\n";
                    else
                        std::cout << "  exe_dir: <empty — could not determine executable path>\n";
                    if (!profiles_dir.empty())
                        std::cout << "  profiles_dir: " << profiles_dir << "\n";
                    else
                        std::cout << "  profiles_dir: <not found>\n";
                }
            }
#endif // ENGINE_BAMBU — PresetBundle preset-resolution + staging symlinks.
       // OrcaSlicer slices directly from the flat 3MF project_settings.config.
        } else if (calib_self_geometry && input_file.empty()) {
            std::cout << "No --input: pressure_advance_pattern will synthesize a handle cube.\n";
        } else {
            std::cerr << "Unsupported file format. Use .stl or .3mf\n";
            set_outcome_failure(outcome, CLI_DATA_FILE_ERROR);
            return 1;
        }

        // For pressure_advance_pattern the model is generated later by
        // apply_pa_pattern (before print.apply), so an empty model here is fine.
        if (model.objects.empty() && !calib_self_geometry) {
            std::cerr << "No objects found in model\n";
            set_outcome_failure(outcome, CLI_NO_SUITABLE_OBJECTS);
            return 1;
        }

        // Ensure all objects have at least one instance
        // Also set use_loaded_id_for_label so that the identify_id from
        // model_settings.config is used for OBJECT_ID labels in G-code
        // (matches BambuStudio desktop behavior at BambuStudio.cpp:6196).
        for (auto* obj : model.objects) {
            if (obj->instances.empty()) {
                obj->add_instance();
            }
            for (auto* inst : obj->instances) {
                inst->use_loaded_id_for_label = true;
            }
        }

        std::cout << "Model loaded successfully:\n";
        for (const auto* obj : model.objects) {
            std::cout << "  - " << obj->name << " (" << obj->volumes.size()
                      << " volumes, " << obj->instances.size() << " instances)\n";
        }

        // Mesh auto-repair census. The GUI puts a warning icon beside every
        // object whose mesh admesh had to repair, with a "(Repair)" hyperlink
        // (GUI_ObjectList.cpp:516/521). The CLI repairs silently, so an agent
        // cannot tell a clean input from one that was rewritten under it.
        for (auto* obj : model.objects) {
            const int repaired = obj->get_repaired_errors_count();
            if (repaired <= 0) continue;
            emit_event({{"event","mesh_repaired"},
                        {"tag","RepairedMeshErrors"},
                        {"object", obj->name},
                        {"repaired_error_count", repaired},
                        {"message","Mesh errors were auto-repaired on load for '" + obj->name + "'"}});
        }

        // Load config files in order (later ones can override earlier ones)
        // A profile that fails to load leaves the slice running on whatever was
        // already in `config` — silently, with only a stderr line. The agent
        // that chose that profile must be told it did not take effect.
        auto load_profile = [&](const std::string& path, const char* kind) {
            if (path.empty()) return;
            if (load_json_config(path, config, verbose,
                                 std::string("profile:") + kind + ":" + path,
#ifdef ENGINE_BAMBU
                                 &explicit_config_supplied_nozzle_map
#else
                                 nullptr
#endif
                                 )) return;
            emit_event({{"event","config_load_failed"},
                        {"tag","ProfileLoadFailed"},
                        {"kind", kind},
                        {"path", path},
                        {"message", std::string("Failed to load ") + kind +
                                    " config; slicing continues with the previously resolved settings"}});
            std::cerr << "Warning: Failed to load " << kind << " config\n";
        };
        // Presets by name: the desktop app's full_config() for the named
        // printer, process and filaments, over the engine defaults.
        if (g_preset_config)
            config.apply(*g_preset_config);
        load_profile(bundle_config,   "bundle");
        load_profile(machine_config,  "machine");
        load_profile(process_config,  "process");
        load_profile(filament_config, "filament");

        // The official value check (OrcaSlicer.cpp 3574-3581 at 31f6803,
        // BambuStudio.cpp 4134-4141 at 5873b5f): DynamicPrintConfig::validate(true)
        // over the settings as the file and profiles state them, before this
        // driver pads any vector. Out-of-range values are refused, each named
        // with the engine's own sentence ("tree_support_wall_count: -1 not in
        // range [0,2]").
        // The official check runs after the command line is applied
        // (m_print_config.apply(m_extra_config) at BambuStudio.cpp 4091,
        // OrcaSlicer.cpp 3542), so a command-line override is range-checked
        // too. The overrides themselves still land where they always have
        // (further down); here they go onto a copy for the check only, and
        // without overrides the check reads `config` as before.
        if (!calib_self_geometry) {
            // The plate's own settings are checked the same way, in the order
            // they are applied below (plate, then command line). The official
            // CLI checks m_print_config (BambuStudio.cpp 4134) before it lays
            // the plate over it (6903), so it never range-checks a plate
            // value; checking the settings a plate is sliced with is stricter
            // and changes nothing for a plate that states none.
            const Slic3r::DynamicPrintConfig plate_settings = plate_own_settings(plate_data, plate_id);
            std::unique_ptr<Slic3r::DynamicPrintConfig> with_overrides;
            if (!overrides.empty() || !plate_settings.empty()) {
                with_overrides = std::make_unique<Slic3r::DynamicPrintConfig>(config);
                with_overrides->apply(plate_settings, true);
                apply_command_line_overrides(*with_overrides, overrides, /*report_rejections=*/false);
            }
            const std::map<std::string, std::string> validity =
                (with_overrides ? *with_overrides : config).validate(true);
            if (!validity.empty() || !unknown_values.empty()) {
                // One refusal naming every value at once: unknown enum
                // values first, then the engine's range findings.
#ifdef ENGINE_ORCA
                std::string sentence = "OrcaSlicer cannot print this file as it is: ";
#else
                std::string sentence = "BambuStudio cannot print this file as it is: ";
#endif
                bool first = true;
                for (const std::string& u : unknown_values) {
                    sentence += (first ? "" : "; ") + u;
                    first = false;
                }
                json items = json::object();
                for (const auto& [key, why] : validity) {
                    sentence += (first ? "" : "; ") + key + ": " + why;
                    first = false;
                    items[key] = why;
                }
                sentence += ".";
                emit_event({{"event","config_refused"},
                            {"tag", unknown_values.empty() ? "InvalidValues" : "InvalidOrUnknownValues"},
                            {"settings", items},
                            {"unknown_values", unknown_items},
                            {"message", sentence}});
                std::cerr << "Param values in 3mf/config error: " << sentence << "\n";
                set_outcome_failure(outcome, CLI_INVALID_VALUES_IN_3MF, sentence);
                return 1;
            }
        }

#ifdef ENGINE_BAMBU
        // Filament roster the project declares, captured before the BBS
        // normalization below reshapes any vector option.  It is taken from the
        // five identity keys alone, and it is the only count that stays the
        // project's: the plate overlay and the nozzle-map derivation further down
        // can rewrite filament_map, and the final alignment grows the per-filament
        // arrays to this count, so asking one of those arrays for its own length
        // later answers a different question.  The identity keys are themselves
        // never padded to the machine's nozzle count
        // (is_per_filament_config_key), so this is also the count the engine
        // derives once the config is applied.
        const size_t project_filaments = project_filament_count(config);
#endif

#ifdef ENGINE_BAMBU
        // Provenance is accumulated while each input/profile layer is parsed,
        // before normalization pads vector options. A supplied [1] is explicit
        // even when it equals the driver's seed value.
        if (verbose)
            std::cout << "Nozzle-map provenance: explicit config map="
                      << (explicit_config_supplied_nozzle_map ? "yes" : "no") << "\n";

        // ── BBS toolchanger / per-extruder normalizations ───────────────────
        // Everything from here to the matching #endif exists to make the Bambu
        // engine accept a non-Bambu printer config (vector-array padding/collapse,
        // explicit nozzle-map derivation, master-nozzle reassignment, prime-tower
        // disable heuristics).  OrcaSlicer handles all of this natively, so the
        // Orca driver skips the whole region and slices from the flat 3MF config.
        //
        // Pad empty vector config options and preserve 3MF multi-element values.
        //
        // The 3MF project_settings.config contains per-extruder arrays from the
        // printer profile (e.g. machine_max_acceleration_e = ["5000","5000"]).
        // BambuStudio desktop keeps these intact through Print::apply() which
        // processes them via update_values_to_printer_extruders.  The CLI must
        // NOT truncate them — the pipeline handles multi-element arrays via
        // get_at() which clamps to the last element for single-filament prints.
        //
        // Pad empty vectors to extruder_count (from nozzle_diameter size).
        {
            // Determine actual extruder count from nozzle_diameter vector.
            size_t extruder_count = 1;
            if (auto* nd = config.option<Slic3r::ConfigOptionFloats>("nozzle_diameter", false)) {
                if (!nd->values.empty())
                    extruder_count = nd->values.size();
            }

            // Keys that must NOT be touched (polygon/group semantics)
            static const std::unordered_set<std::string> skip_pad = {
                "printable_area",           // bed shape polygon (4+ points)
                "bed_exclude_area",         // exclusion zones
                "thumbnails",
                "extruder_printable_area",  // per-extruder bed polygons

                // The two per-filament INDEX maps.  is_per_filament_config_key()
                // returns false for them — they are the "special" spellings that
                // get their own alignment, not plain value arrays — so the
                // predicate below does not reach them on its own.  They are per
                // filament all the same and must not be padded to the nozzle
                // count: padding is exactly how a one-filament project on a
                // two-nozzle machine grows filament_map to [1, 0], whose LENGTH
                // the engine then reads as two filaments and fans every
                // per-filament array out to (PrintConfig.cpp:8134).
                // align_per_filament_maps() extends and repairs them against the
                // roster instead.  (filament_nozzle_map, the third index map, is a
                // plain per-filament key, so the predicate already covers it.)
                "filament_map",
                "filament_map_2",
            };

            // Per-filament arrays are excluded from this padding altogether: the
            // plain ones by the same predicate the final alignment uses, the two
            // index maps by skip_pad above.  Their LENGTH is the
            // engine's filament count, not the head count: Print::apply() reads its
            // num_extruders from filament_diameter (PrintApply.cpp:1438),
            // ToolOrdering sizes its filament loops and the per-nozzle flush
            // geometry from filament_colour / filament_diameter (ToolOrdering.cpp:509,
            // 2503; prepare_flush_matrices:1421), Print::has_wipe_tower()
            // (Print.cpp:3113) tests filament_diameter.size() > 1 directly, and the
            // multi-filament fan-out reads filament_map's length
            // (PrintConfig.cpp:8134).  Padding a one-filament project's
            // per-filament arrays up to a two-nozzle machine's nozzle count
            // invents a filament slot the project never declared — with a zero
            // diameter, no colour and no type — and everything sized from that
            // count downstream (wipe tower, per-nozzle flush matrix, G-code
            // metadata) is then sized for it.  Left at their declared length, the
            // engine sees the roster the project actually loaded, and the final
            // central alignment extends them to exactly that count.
            //
            // A zero spliced in here would also be sticky: the alignment only ever
            // duplicates the FRONT value, so a padded [22, 0] becomes [22, 0, 22]
            // for a three-filament roster instead of [22, 22, 22].
            //
            // The variant-slot arrays (filament_extruder_variant,
            // filament_self_index) are not per-filament and keep the extruder-count
            // padding they have always had — they belong to the printer-side slot
            // table (PrintConfig.cpp:7592).
            //
            // Only pad empty vectors to extruder_count; do NOT truncate.
            // Variant-expanded arrays (retraction_length, machine_max_*, speed, etc.)
            // are collapsed by PrintApply.cpp's update_values_to_printer_extruders()
            // when (extruder_count > 1) || different_extruder.
            // support_different_extruders() returns true when extruder_variant_list
            // has multiple comma-separated variants (e.g. "Direct Drive Standard,
            // Direct Drive High Flow"), which is the normal case for BBL printers.
            // Truncating here would bypass that collapsing and produce wrong sizes.
            for (const auto& key : config.keys()) {
                if (skip_pad.count(key))
                    continue;

                // The predicate shared with the final alignment catches the plain
                // per-filament arrays, so this pass can never pad an array the
                // alignment then treats as a roster, or the reverse.  The index
                // maps the predicate excludes are in skip_pad above.
                if (is_per_filament_config_key(key))
                    continue;

                auto* opt = config.option(key, false);
                if (!opt) continue;

#define NORM_VEC(Type, zero_val) \
                if (auto v = dynamic_cast<Slic3r::Type*>(opt)) { \
                    while (v->values.size() < extruder_count) v->values.push_back(zero_val); \
                } else
                NORM_VEC(ConfigOptionBoolsNullable,          (unsigned char)0)
                NORM_VEC(ConfigOptionBools,                  (unsigned char)0)
                NORM_VEC(ConfigOptionIntsNullable,           0)
                NORM_VEC(ConfigOptionInts,                   0)
                NORM_VEC(ConfigOptionFloatsNullable,         0.0)
                NORM_VEC(ConfigOptionFloats,                 0.0)
                NORM_VEC(ConfigOptionPercentsNullable,       0.0)
                NORM_VEC(ConfigOptionPercents,               0.0)
                NORM_VEC(ConfigOptionFloatsOrPercentsNullable, (Slic3r::FloatOrPercent{0.0, false}))
                NORM_VEC(ConfigOptionFloatsOrPercents,       (Slic3r::FloatOrPercent{0.0, false}))
                NORM_VEC(ConfigOptionStrings,                std::string{})
                NORM_VEC(ConfigOptionEnumsGenericNullable,   0)
                NORM_VEC(ConfigOptionEnumsGeneric,           0)
                { /* ConfigOptionPoints, ConfigOptionPointsGroups, etc. — leave alone */ }
#undef NORM_VEC
            }

            // extruder_variant_list contains comma-separated variant names per slot
            // (e.g. "Direct Drive Standard,Direct Drive High Flow").
            // support_different_extruders() splits these and detects multiple variants,
            // triggering update_values_to_printer_extruders() in PrintApply.cpp.
            // Do NOT truncate this list — it must remain as-is for correct collapsing.

            // Clamp master_extruder_id to extruder_count (1-indexed).
            // ToolOrdering::get_recommended_filament_maps() uses master_extruder_id-1
            // as an index into nozzle_list, which has extruder_count entries.
            if (auto* mid = config.option<Slic3r::ConfigOptionInt>("master_extruder_id", false)) {
                if (mid->value < 1) mid->value = 1;
                if (mid->value > (int)extruder_count) mid->value = (int)extruder_count;
            }
        }

        // Ensure vector options after loading JSON configs and forcing single-extruder
        // JSON configs may have left some vectors empty or missing
        ensure_vector_config_sizes(config);

        // If the fully-overlaid config carries an explicit per-filament nozzle assignment,
        // apply it directly to config's filament_map.  This reproduces the
        // BambuStudio desktop behaviour where the user has already constrained the
        // mapping in the GUI ("Nozzle Manual" mode) and saved it in the plate metadata.
        //
        // Explicit is detected when EITHER:
        //   (a) plate filament_maps have multiple distinct values (e.g. [2,1]), OR
        //   (b) the project config says "Nozzle Manual" (handles uniform maps like [2,2])
        //
        // When the maps are uniform AND mode is "Auto For Flush" (e.g. [1,1]),
        // we fall through to apply_explicit_nozzle_mapping() which re-derives the map
        // from filament_nozzle_map using the physical_extruder_map inverse.
        int plate_data_idx = (plate_id > 0 && (int)plate_data.size() >= plate_id) ? plate_id - 1 : 0;
        bool explicit_plate_mapping_applied = false;
        if (!plate_data.empty() && plate_data[plate_data_idx] != nullptr) {
            const auto& pm = plate_data[plate_data_idx]->filament_maps;
            // The plate's own mode, else the project's
            // (PartPlate::get_real_filament_map_mode, PartPlate.cpp 278-289;
            // BambuStudio.cpp 6661-6665), applied over the print config as the
            // official CLI applies the plate config (BambuStudio.cpp 6905-6907).
            // The stored plate map is used only in a manual mode: under an
            // automatic mode the engine groups the filaments itself
            // (ToolOrdering.cpp 1897-1914), whatever map an earlier slice saved.
            Slic3r::FilamentMapMode real_mode = Slic3r::FilamentMapMode::fmmAutoForFlush;
            if (auto* mode_opt = config.option<Slic3r::ConfigOptionEnum<Slic3r::FilamentMapMode>>("filament_map_mode", false))
                real_mode = mode_opt->value;
            if (auto* plate_mode = plate_data[plate_data_idx]->config.option<Slic3r::ConfigOptionEnum<Slic3r::FilamentMapMode>>("filament_map_mode");
                plate_mode && plate_mode->value != Slic3r::FilamentMapMode::fmmDefault) {
                real_mode = plate_mode->value;
                config.option<Slic3r::ConfigOptionEnum<Slic3r::FilamentMapMode>>("filament_map_mode", true)->value = real_mode;
            }
            const bool is_manual_mode = real_mode == Slic3r::FilamentMapMode::fmmManual ||
                                        real_mode == Slic3r::FilamentMapMode::fmmNozzleManual;
            // Any saved map in a manual mode, one slot included: the official
            // CLI slices a manual plate with get_real_filament_maps(), the
            // plate's own map whenever it has one (BambuStudio.cpp 6751;
            // PartPlate.cpp 266-276), whatever its length.
            bool has_explicit = !pm.empty() && is_manual_mode;
            if (has_explicit) {
                auto* fm = config.option<Slic3r::ConfigOptionInts>("filament_map", true);
                fm->values = pm;  // already 1-based per PlateData docs
                // The plate's nozzle-volume map with it, as the official
                // plate config carries both (bbs_3mf.cpp 4612-4624).
                if (const auto* pvm = plate_data[plate_data_idx]->config.option<Slic3r::ConfigOptionInts>("filament_volume_map");
                    pvm && pvm->values.size() == pm.size())
                    config.option<Slic3r::ConfigOptionInts>("filament_volume_map", true)->values = pvm->values;

                // Also sync filament_map_2 (0-based mirror used by some code paths)
                auto* fm2 = config.option<Slic3r::ConfigOptionInts>("filament_map_2", true);
                fm2->values.resize(pm.size());
                for (size_t i = 0; i < pm.size(); ++i)
                    fm2->values[i] = pm[i] - 1;

                // `Nozzle Manual` drives the multi-nozzle grouping branch.  H2D is
                // instead a multi-extruder machine (one nozzle per extruder), where
                // `Manual` is the branch that preserves this already-resolved plate
                // map through ToolOrdering.  Derive the mode from the configured
                // topology rather than a printer identity.
                bool has_multiple_nozzles_per_extruder = false;
                if (auto* counts = config.option<Slic3r::ConfigOptionInts>("extruder_max_nozzle_count", false)) {
                    has_multiple_nozzles_per_extruder = std::any_of(
                        counts->values.begin(), counts->values.end(),
                        [](int count) { return count > 1; });
                }
                Slic3r::ConfigSubstitutionContext subst(Slic3r::ForwardCompatibilitySubstitutionRule::Enable);
                config.set_deserialize(
                    "filament_map_mode",
                    has_multiple_nozzles_per_extruder ? "Nozzle Manual" : "Manual",
                    subst);
                explicit_plate_mapping_applied = true;
            }
        }

        // Fix the cardinality of the per-filament maps BEFORE the derivation reads
        // them.  apply_explicit_nozzle_mapping() takes filament_map's own length as
        // the filament count, so a map still holding the driver's single seed entry
        // — a project whose filament_map is a placeholder while its
        // filament_nozzle_map carries the real cross-nozzle split — would look like
        // a one-filament project and skip a routing the file declares.  This is the
        // same helper the final alignment calls, which is what keeps the maps
        // cardinality-correct after the plate overlay above and the command-line
        // overrides below have had their chance to rewrite them.
        align_per_filament_maps(config, project_filaments);

        // The selected plate already expresses physical routing when it carries
        // a diverse map. Do not re-derive that routing from the general
        // filament_nozzle_map fallback: the latter's object reassignment is
        // exclusively for an otherwise-auto mapping.
        bool nozzle_mapping_derived =
            explicit_plate_mapping_applied ? false : apply_explicit_nozzle_mapping(
                config, explicit_config_supplied_nozzle_map, verbose);

        // When apply_explicit_nozzle_mapping derived a cross-nozzle split from
        // "Auto For Flush" mode (filament_maps="1 1"), reassign all objects to
        // the master (right) nozzle to match BambuStudio desktop behavior.
        if (nozzle_mapping_derived) {
            reassign_objects_to_master_nozzle(model, config);
            // --export-3mf reloads the file, which knows nothing of this
            // routing: keep it so the project states what the slice used.
            if (o.slice_mode && !o.export_3mf.empty()) {
                outcome.derived_filament_map = config.option<Slic3r::ConfigOptionInts>("filament_map", true)->values;
                outcome.master_filament_slot = master_nozzle_filament_slot(config);
            }
            if (verbose) {
                std::cout << "Nozzle-map reassignment: object_extruders=[";
                for (size_t i = 0; i < model.objects.size(); ++i) {
                    const auto* extruder = dynamic_cast<const Slic3r::ConfigOptionInt*>(
                        model.objects[i]->config.option("extruder"));
                    std::cout << (i == 0 ? "" : ",")
                              << (extruder ? std::to_string(extruder->value) : "missing");
                }
                std::cout << "]\n";
            }
        } else if (verbose) {
            const auto* mode = config.option<Slic3r::ConfigOptionEnum<Slic3r::FilamentMapMode>>(
                "filament_map_mode", false);
            const std::string mode_name = mode ? mode->serialize() : "<unset>";
            std::cout << "Nozzle-map derivation: skipped mode=" << mode_name << "\n";
        }

        // Disable prime tower when there is no actual multi-material printing.
        //
        // BambuStudio (BambuStudio.cpp:3885-3913) disables prime tower when
        // all loaded filament presets are the same AND all filament colors are
        // the same.  The GUI also disables it when all filament slots map to the
        // same physical extruder (filament_map all equal) and all colors match.
        //
        // A prime tower with no tool changes (ToolOrdering::has_wipe_tower()==false)
        // causes Print::has_wipe_tower()==true to take a different code path in
        // psWipeTower: it builds m_wipe_tower_data.tool_ordering but leaves
        // m_tool_ordering empty.  psSkirtBrim then reads the empty m_tool_ordering,
        // producing initial_extruder_id==-1 and wrong path ordering.  Disabling
        // enable_prime_tower forces the correct code path.
        {
            auto* ept = config.option<Slic3r::ConfigOptionBool>("enable_prime_tower", false);
            if (ept && ept->value) {
                bool disable = false;

                // Single filament slot: trivially no multi-material.  This asks
                // the roster the project declared (filament_colour /
                // settings_id / ids / type / diameter), in the snapshot taken
                // from project_filament_count() before any normalization ran —
                // not one array's own current length, which the alignment below
                // has already grown to that same count.  A project that declares
                // no identity key at all still reads as zero here, not as one.
                if (project_filaments <= 1)
                    disable = true;

                // Multiple slots: disable when all filament colors are the same
                // AND all filament_map values point to the same extruder.
                // This covers: same-preset AMS prints (tests 20-23, 18-19),
                // same-extruder different-preset prints (tests 24-25), etc.
                // Multi-color AMS prints (test 10) have different colors → kept enabled.
                if (!disable) {
                    auto* fc = config.option<Slic3r::ConfigOptionStrings>("filament_colour", false);
                    auto* fm = config.option<Slic3r::ConfigOptionInts>("filament_map", false);
                    bool all_same_color = fc && !fc->values.empty() &&
                        std::all_of(fc->values.begin(), fc->values.end(),
                                    [&](const std::string& c){ return c == fc->values[0]; });
                    bool all_same_extruder = fm && !fm->values.empty() &&
                        std::all_of(fm->values.begin(), fm->values.end(),
                                    [&](int v){ return v == fm->values[0]; });
                    if (all_same_color && all_same_extruder)
                        disable = true;
                }

                if (disable)
                    ept->value = false;
            }
        }
#endif // ENGINE_BAMBU — BBS toolchanger / per-extruder normalizations

#ifdef ENGINE_ORCA
        // Toolchanger filament-map (U1/Prusa-XL class): the flat 3MF stores
        // filament_map_mode="Auto For Flush" with a placeholder filament_map. With
        // is_BBL_printer() initialized deterministically (see below, next to print
        // construction) the engine's native Auto-For-Flush resolution computes the
        // per-filament→tool map from the per-volume extruders we already load, matching
        // golden B — so no driver-side filament_map injection is needed here. Model
        // per-extruder static tables (setExtruderParams/setPrintSpeedTable) are still set
        // after apply(), exactly as Orca's own headless CLI does.
#endif // ENGINE_ORCA

        // The selected plate's own settings over the project's, then the
        // command line over both: the official CLI slices each plate with
        // new_print_config = m_print_config; apply(*part_plate->config());
        // apply(m_extra_config) (BambuStudio.cpp 6902-6904 at 5873b5f;
        // OrcaSlicer.cpp 5905-5907 at 31f6803). A plate states bed type,
        // print sequence, first/other layer order, spiral mode and the
        // filament-switcher flags (bbs_3mf.cpp 4563-4728 / Orca 4419-4563).
        // The filament-map keys are not taken here: on the Bambu build the
        // plate block above applies the plate's mode and, in a manual mode,
        // its maps (an automatic mode regroups whatever map was saved,
        // ToolOrdering.cpp 1897-1914); on the Orca build the block just
        // below does the same. Only with a plate named (--plate N or
        // --slice): a whole-file load is not one plate.
        // The plate wins over the --config/--machine/--process/--filament
        // files: they are the official --load_settings/--load_filaments,
        // which merge into m_print_config (update_full_config, BambuStudio.cpp
        // 3246, 3384; OrcaSlicer.cpp 2781, 2925) before the plate is laid over
        // it. The command-line overrides (m_extra_config) come last.
#ifdef ENGINE_ORCA
        // The plate's filament-map keys on the Orca build. The official Orca
        // CLI lays the whole plate config over the project's
        // (OrcaSlicer.cpp 5905-5907), mode and map included. Here: the
        // plate's mode when it states one (PartPlate::get_real_filament_map_mode,
        // fmmDefault defers to the project), and in the manual mode the
        // plate's own map (get_real_filament_maps, the map whenever the plate
        // has one) and nozzle-volume map. An automatic mode keeps the
        // engine's native Auto-For-Flush resolution with the project's map,
        // as before (the U1 toolchanger case).
        if (plate_id > 0 && (int)plate_data.size() >= plate_id && plate_data[plate_id - 1] != nullptr) {
            const Slic3r::PlateData& plate = *plate_data[plate_id - 1];
            Slic3r::FilamentMapMode real_mode = Slic3r::FilamentMapMode::fmmAutoForFlush;
            if (auto* mode_opt = config.option<Slic3r::ConfigOptionEnum<Slic3r::FilamentMapMode>>("filament_map_mode", false))
                real_mode = mode_opt->value;
            if (auto* plate_mode = plate.config.option<Slic3r::ConfigOptionEnum<Slic3r::FilamentMapMode>>("filament_map_mode");
                plate_mode && plate_mode->value != Slic3r::FilamentMapMode::fmmDefault) {
                real_mode = plate_mode->value;
                config.option<Slic3r::ConfigOptionEnum<Slic3r::FilamentMapMode>>("filament_map_mode", true)->value = real_mode;
            }
            if (real_mode == Slic3r::FilamentMapMode::fmmManual && !plate.filament_maps.empty()) {
                config.option<Slic3r::ConfigOptionInts>("filament_map", true)->values = plate.filament_maps;
                if (const auto* pvm = plate.config.option<Slic3r::ConfigOptionInts>("filament_volume_map");
                    pvm && pvm->values.size() == plate.filament_maps.size())
                    config.option<Slic3r::ConfigOptionInts>("filament_volume_map", true)->values = pvm->values;
                emit_event({{"event","config_normalized"}, {"tag","PlateFilamentMapApplied"},
                            {"plate_id", plate_id}, {"filament_map", plate.filament_maps},
                            {"message","Plate " + std::to_string(plate_id) +
                                       " is in Manual mode: its own filament map applies"}});
            }
        }
#endif
        {
            const Slic3r::DynamicPrintConfig plate_config = plate_own_settings(plate_data, plate_id);
            if (!plate_config.empty()) {
                config.apply(plate_config, true);
                emit_event({{"event","config_normalized"}, {"tag","PlateSettingsApplied"},
                            {"plate_id", plate_id}, {"keys", plate_config.keys()},
                            {"message","Applied plate " + std::to_string(plate_id) + "'s own settings: " +
                                       boost::algorithm::join(plate_config.keys(), ", ")}});
            }
        }

        // Apply command-line overrides
        apply_command_line_overrides(config, overrides, /*report_rejections=*/true);

        // Display active settings
        std::cout << "\nActive print settings:\n";
        if (config.has("layer_height")) {
            std::cout << "  Layer height: " << config.opt_float("layer_height") << "mm\n";
        }
        if (config.has("perimeters")) {
            std::cout << "  Perimeters: " << config.opt_int("perimeters") << "\n";
        }
        if (config.has("sparse_infill_density")) {
            auto percent_opt = config.option<Slic3r::ConfigOptionPercent>("sparse_infill_density");
            if (percent_opt) {
                std::cout << "  Infill: " << percent_opt->value << "%\n";
            }
        }
        if (config.has("nozzle_diameter")) {
            auto nozzles = config.option<Slic3r::ConfigOptionFloats>("nozzle_diameter");
            if (nozzles && !nozzles->values.empty()) {
                std::cout << "  Nozzle: " << nozzles->values[0] << "mm\n";
            }
        }

        // --slice: the official per-plate gate before apply (BambuStudio.cpp
        // 6527-6567 at 5873b5f; OrcaSlicer.cpp 5645-5697 at 31f6803). An
        // object partly over the bed edge is refused, and a plate with no
        // object fully inside is refused; objects wholly outside are left
        // out of the print, as the official CLI's apply() leaves them out.
        if (o.slice_mode && !calib_self_geometry) {
            std::map<size_t, Slic3r::Vec3d> before;
            for (const Slic3r::ModelObject* object : model.objects)
                for (const Slic3r::ModelInstance* inst : object->instances)
                    before[inst->loaded_id] = inst->get_offset();
            if (o.arrange && !arrange_on_bed(model, config, outcome)) {
                std::cerr << "Error: " << outcome.error_string << "\n";
                return 1;
            }
            if (o.arrange && !o.export_3mf.empty())
                for (const Slic3r::ModelObject* object : model.objects)
                    for (const Slic3r::ModelInstance* inst : object->instances)
                        if (inst->loaded_id != 0 && before.count(inst->loaded_id))
                            outcome.moved[inst->loaded_id] = inst->get_offset() - before[inst->loaded_id];
            if (!check_objects_inside_bed(model, config, outcome)) {
                std::cerr << "Error: " << outcome.error_string << "\n";
                return 1;
            }
        }

        // Initialize print
        std::cout << "\nInitializing print...\n";
        Slic3r::Print print;

        // Enable BBL printer features (M981 spaghetti detector, M1003 powerlost
        // recovery, etc.) when the 3MF was generated by BambuStudio.
        // Matches BackgroundSlicingProcess.cpp:199.
#ifdef ENGINE_BAMBU
        // set_BBL_Printer is a BambuStudio-only Print method (enables M981/M1003
        // BBL printer features).  OrcaSlicer's Print has no such method.
        if (is_bbl_3mf)
            print.set_BBL_Printer(true);
        // Named presets: the official CLI decides from printer_model
        // (BambuStudio.cpp 7184-7198 at 5873b5f), as the desktop app does.
        else if (g_preset_config && config.opt_string("printer_model", true).rfind("Bambu Lab", 0) == 0)
            print.set_BBL_Printer(true);
#endif
#ifdef ENGINE_ORCA
        // Orca's Print::m_isBBLPrinter (Print.hpp:1143) has NO default initializer and
        // no set_BBL_Printer() method. Left uninitialized it is read with an
        // indeterminate value (UB) across the slice/export phases — driving both the
        // wipe_tower_type() Type1/Type2 choice (Print.hpp:1072), the GCode dialect
        // (GCode.cpp:2046), and the ;Z_HEIGHT vs ;Z: comment (GCode.cpp:4486). The
        // inconsistent reads make process()'s wipe-tower tool ordering disagree with the
        // sequence read at export → "append_tcr ... toolchange it didn't expect".
        // Mirror OrcaSlicer's own headless CLI (OrcaSlicer.cpp:5972-5986): the flag is
        // true iff the printer_model preset begins with "Bambu Lab" — false for the
        // Snapmaker U1, which selects Type2 + the Orca/Prusa-style gcode dialect (golden B).
        {
            const std::string pm = config.opt_string("printer_model", true);
            print.is_BBL_printer() = (pm.rfind("Bambu Lab", 0) == 0);
        }
#endif

        /// Set plate origin to (0,0,0) for standalone mode
        /// Print.hpp:986
        /// C++: void set_plate_origin(Vec3d origin) { m_origin = origin; }
        print.set_plate_origin(Slic3r::Vec3d(0.0, 0.0, 0.0));
        // The plate's own per-plate values (wipe_tower_x/y are one entry per
        // plate, read with get_at(m_plate_index): Print.cpp 2213, GCode.cpp 5006)
        // need the plate's index, as the desktop sets it (PartPlate.cpp:2299).
        // Left at 0, plate N's prime tower stood where plate 1's does.
        if (plate_id > 0)
            print.set_plate_index(plate_id - 1);

        // cli #4: alias the unbound `initial_no_support_filament_id` placeholder to the
        // engine-bound `initial_no_support_extruder` across custom-gcode keys, BEFORE
        // print.apply snapshots the config. Without this the PlaceholderParser throws at
        // export when a 3MF carries the legacy token in its custom gcode. Always on;
        // suppressed with --no-normalize-legacy-gcode.
        if (normalize_legacy_gcode) {
            normalize_legacy_gcode_tokens(config);
        }

        // cli #5: whether the active printer speaks the Bambu G-code dialect.
        // Mirrors the is_BBL_printer() derivation used below for the engine.
        const bool calib_is_bbl_machine =
            config.opt_string("printer_model", true).rfind("Bambu Lab", 0) == 0;

        // cli #5: pressure_advance_pattern generates its own geometry + per-layer
        // custom G-code. This MUST run before print.apply (apply snapshots the
        // model + config and reads plates_custom_gcodes). `calib_params` is a
        // main-scope local, so the reference held by model.calib_pa_pattern
        // stays valid through the whole slice.
        if (calib_params.mode == Slic3r::CalibMode::Calib_PA_Pattern) {
            std::cout << "Generating pressure-advance pattern geometry...\n";
            slicer_cli::apply_pa_pattern(calib_params, config, model, calib_is_bbl_machine);
        }

#ifdef ENGINE_BAMBU
        // Align every per-filament array with the filament roster the project
        // loaded (see align_per_filament_config_vectors above), using the count
        // captured before the normalization pass padded vector options.  This is
        // the one and only alignment, and it is the last config write before
        // print.apply() snapshots the config, so every layer that can reshape a
        // per-filament array is already in place: the plate map overlay and the
        // nozzle-map derivation, the legacy-gcode token aliasing, the calibration
        // pattern, and the command-line overrides — an override such as --temp
        // rewrites nozzle_temperature down to a single value, so aligning any
        // earlier would leave that array short again.
        align_per_filament_config_vectors(config, project_filaments, verbose);

        // Automatic filament grouping on a machine with several extruders
        // groups against the AMS slots it is told about. With none, every
        // filament lands on the master extruder. The official CLI gives each
        // extruder one estimated 4-slot AMS ("1#0|4#1") holding the project's
        // filament colours and types in turn (BambuStudio.cpp 6911-6950 at
        // 5873b5f; estimate_mode is off on this path). OrcaSlicer.cpp
        // 5914-5951 has the same block; the Orca build is left as it is here,
        // since its toolchanger grouping (U1) is matched without it.
        {
            const auto* nozzles = config.option<Slic3r::ConfigOptionFloats>("nozzle_diameter");
            const int extruder_count = nozzles ? int(nozzles->values.size()) : 1;
            const auto* map_mode = config.option<Slic3r::ConfigOptionEnum<Slic3r::FilamentMapMode>>("filament_map_mode");
            if (extruder_count > 1 && (!map_mode || Slic3r::is_auto_filament_map_mode(map_mode->value))) {
                const auto* filament_colour = config.option<Slic3r::ConfigOptionStrings>("filament_colour");
                const auto* filament_type = config.option<Slic3r::ConfigOptionStrings>("filament_type");
                std::vector<std::string> colors = filament_colour ? filament_colour->vserialize() : std::vector<std::string>{};
                if (colors.empty()) colors.push_back("#FFFFFFFF");
                std::vector<std::string> types = filament_type ? filament_type->vserialize() : std::vector<std::string>{"PLA"};
                if (types.empty()) types.push_back("PLA");
                std::vector<std::string> extruder_ams_count(extruder_count, "1#0|4#1");
                std::vector<std::vector<Slic3r::DynamicPrintConfig>> extruder_filament_info(extruder_count);
                int color_count = 0;
                for (int e = 0; e < extruder_count; ++e)
                    for (int slot = 0; slot < 4; ++slot, ++color_count) {
                        Slic3r::DynamicPrintConfig slot_config;
                        slot_config.set_key_value("filament_colour", new Slic3r::ConfigOptionStrings({colors[color_count % colors.size()]}));
                        slot_config.set_key_value("filament_type", new Slic3r::ConfigOptionStrings({types[color_count % types.size()]}));
                        slot_config.set_key_value("filament_is_support", new Slic3r::ConfigOptionBools({false}));
                        slot_config.set_key_value("tray_name", new Slic3r::ConfigOptionStrings({"A1"}));
                        extruder_filament_info[e].push_back(std::move(slot_config));
                    }
                config.option<Slic3r::ConfigOptionStrings>("extruder_ams_count", true)->values = extruder_ams_count;
                print.set_extruder_filament_info(extruder_filament_info);
            }
        }
#endif // ENGINE_BAMBU

        try {
            std::cout << "Applying configuration...\n";
            print.apply(model, config);

            // cli #5: install calibration params after apply (which resets print
            // state) and before validate/process so the engine's per-layer calib
            // emission (GCode.cpp) and PA-line/pattern paths see the mode.
            if (calib_enabled) {
                print.set_calib_params(calib_params);
                std::cout << "Calibration mode: "
                          << slicer_cli::calib_mode_name(calib_params.mode)
                          << " [start=" << calib_params.start
                          << " end=" << calib_params.end
                          << " step=" << calib_params.step << "]\n";
            }

            // Install the structured-warning emitter BEFORE validate() / process()
            // so the TS host sees pre-slice diagnostics in the same JSON-line
            // stream as slicing-time warnings. Pure stdout side-effect — does
            // not change print state, exit codes, or G-code output.
            if (o.progress) {
                print.set_status_callback([&outcome](const Slic3r::PrintBase::SlicingStatus& status) {
                    emit_status_warning(status);
                    using FB = Slic3r::PrintBase::SlicingStatus::FlagBits;
                    const bool is_warning =
                        (status.flags & (FB::UPDATE_PRINT_STEP_WARNINGS | FB::UPDATE_PRINT_OBJECT_STEP_WARNINGS)) != 0;
                    if (status.warning_step != -1 || is_warning) {
                        // The official CLI keeps every warning (g_slicing_warnings,
                        // cli_status_callback) and reports the last one per plate.
                        if (!status.text.empty() &&
                            status.message_type != Slic3r::PrintStateBase::SlicingDefaultNotification) {
                            outcome.warning_message = status.text;
                            outcome.warnings.push_back(json{{"message", status.text},
                                                            {"level", warning_level_tag(status.warning_level)},
                                                            {"tag", slicing_notification_tag(static_cast<int>(status.message_type))}});
                        }
                        return;
                    }
                    if (status.percent >= 0)
                        emit_progress(outcome, status.percent, status.text);
                });
            } else {
                print.set_status_callback(emit_status_warning);
            }

#ifdef ENGINE_ORCA
            // Orca's headless CLI (OrcaSlicer.cpp ~6065) populates these static Model maps
            // between apply() and process(); our driver must too, or slicing uses empty
            // extruder/speed tables (affects brim, speed and — per the wipe-tower export
            // assertion — tool-ordering construction).
            {
                int filament_count = 1;
                if (auto* fc = config.option<Slic3r::ConfigOptionStrings>("filament_colour", false))
                    filament_count = std::max<int>(1, (int)fc->values.size());
                Slic3r::Model::setExtruderParams(config, filament_count);
                Slic3r::Model::setPrintSpeedTable(config, print.config());
                std::cout << "  [orca] setExtruderParams/setPrintSpeedTable (filaments="
                          << filament_count << ")\n";
            }
#endif

            std::cout << "Validating...\n";
            // Option C — pass a warning out-pointer so BBS routes is_warning-
            // flagged exceptions there instead of into the return value
            // (mirrors `Plater.cpp:9888`'s call signature). Hard validation
            // errors still come back as the return value and remain fatal.
            // Soft warnings get emitted as JSON events and slicing proceeds —
            // matching what BBS GUI does in this case.
            Slic3r::StringObjectException validation_warning;
            // The GUI calls validate() with all three out-params
            // (Plater.cpp:10835) and paints the returned hulls on the plate;
            // a human sees the collision as geometry, with no text anywhere.
            // Passing them here is the only way an agent can learn WHERE a
            // sequential-print clearance violation is, rather than just that
            // one happened.
            Slic3r::Polygons collision_polygons;
            std::vector<std::pair<Slic3r::Polygon, float>> height_polygons;
            Slic3r::StringObjectException validation_result =
                print.validate(&validation_warning, &collision_polygons, &height_polygons);
            if (!collision_polygons.empty() || !height_polygons.empty()) {
                json e;
                e["event"] = "clearance_violation";
                e["tag"]   = "SequentialPrintClearance";
                e["collision_hull_count"] = collision_polygons.size();
                e["height_hull_count"]    = height_polygons.size();
                json hulls = json::array();
                for (const auto& poly : collision_polygons) {
                    json points = json::array();
                    for (const auto& p : poly.points)
                        points.push_back(json{{"x_mm", Slic3r::unscaled<double>(p.x())},
                                              {"y_mm", Slic3r::unscaled<double>(p.y())}});
                    hulls.push_back(points);
                }
                if (!hulls.empty()) e["collision_hulls"] = hulls;
                json heights = json::array();
                for (const auto& [poly, height] : height_polygons) {
                    json points = json::array();
                    for (const auto& p : poly.points)
                        points.push_back(json{{"x_mm", Slic3r::unscaled<double>(p.x())},
                                              {"y_mm", Slic3r::unscaled<double>(p.y())}});
                    heights.push_back(json{{"height_mm", height},
                                           {"point_count", poly.points.size()},
                                           {"points", points}});
                }
                if (!heights.empty()) e["height_hulls"] = heights;
                e["message"] = "Object clearance hulls overlap; the GUI would paint these regions red on the plate";
                emit_event(e);
            }
            if (!validation_warning.string.empty()) {
                emit_validation_event(validation_warning);
                std::cout << "Validation warning: " << validation_warning.string << "\n";
            }
            if (!validation_result.string.empty()) {
                emit_validation_event(validation_result);
                std::cerr << "Validation error: " << validation_result.string << "\n";
                std::cerr << "\nTip: You may need to specify proper config files:\n";
                std::cerr << "  --machine, --filament, and --process options\n";
                // The official CLI's mapping of validate() errors to result
                // codes (BambuStudio.cpp 7094-7111 at 5873b5f).
                int validate_error = CLI_VALIDATE_ERROR;
                switch (validation_result.type) {
                    case Slic3r::STRING_EXCEPT_FILAMENT_NOT_MATCH_BED_TYPE:      validate_error = CLI_FILAMENT_NOT_MATCH_BED_TYPE; break;
                    case Slic3r::STRING_EXCEPT_FILAMENTS_DIFFERENT_TEMP:         validate_error = CLI_FILAMENTS_DIFFERENT_TEMP; break;
                    case Slic3r::STRING_EXCEPT_OBJECT_COLLISION_IN_SEQ_PRINT:    validate_error = CLI_OBJECT_COLLISION_IN_SEQ_PRINT; break;
                    case Slic3r::STRING_EXCEPT_OBJECT_COLLISION_IN_LAYER_PRINT:  validate_error = CLI_OBJECT_COLLISION_IN_LAYER_PRINT; break;
                    default: break;
                }
                set_outcome_failure(outcome, validate_error, validation_result.string);
                return 1;
            }

            std::cout << "Slicing...\n";
            print.process();

            std::cout << "\n✓ Slicing complete!\n";

            // --slice: toolpath conflicts stop the plate before any G-code
            // is written, as the official CLI checks them right after
            // process() and exits before export_gcode (BambuStudio.cpp
            // 7111-7116 at 5873b5f). Print::process() sets the conflict
            // result in both engines (Print.cpp 2396 / 2553).
            if (o.slice_mode) {
                const std::string conflict = print.get_conflict_string();
                if (!conflict.empty()) {
                    set_outcome_failure(outcome, CLI_GCODE_PATH_CONFLICTS, conflict + ".");
                    emit_event({{"event","plate_error"}, {"tag","GcodePathConflicts"}, {"message", conflict}});
                    return 1;
                }
            }

            // Attempt G-code export
            std::cout << "\nExporting G-code to: " << output_file << "\n";

            try {
                /// Export G-code with a result object (must not be nullptr — GCode.cpp:1738 dereferences it)
                /// Print.hpp:848
                /// C++: std::string export_gcode(const std::string &path, GCodeProcessorResult* result, ThumbnailsGeneratorCallback thumbnail_cb);
                Slic3r::GCodeProcessorResult gcode_result;
                print.export_gcode(output_file, &gcode_result, nullptr);

                // The result object is fully populated by the export. Drain
                // every check the GUI would show a human before reporting
                // success — a run can exit 0 and still carry warnings,
                // toolpath conflicts or printable-area failures.
                emit_gcode_result_diagnostics(gcode_result);
                if (o.slice_mode) {
                    record_plate_statistics(print, model, config, gcode_result, outcome);
                    outcome.sliced_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now() - plate_started).count();
                    if (!post_slice_checks(print, model, gcode_result, outcome))
                        return 1;
                    outcome.exported = true;
                    if (!o.export_3mf.empty())
                        record_plate_for_export(print, model, config, gcode_result, output_file,
                                                input_is_stl(input_file), outcome);
                    if (o.progress)
                        emit_progress(outcome, 100, "Slicing finished");
                }

                std::cout << "✓ G-code export complete!\n";
                std::cout << "\nOutput file: " << output_file << "\n";

            } catch (const Slic3r::RuntimeError& e) {
                emit_event({{"event","slicing_error"},{"phase","export_gcode"},{"kind","RuntimeError"},{"message",std::string(e.what())}});
                std::cerr << "\n❌ G-code export failed (RuntimeError): " << e.what() << "\n";
                std::cerr << "\nSlicing succeeded, but export needs additional configuration.\n";
                return 1;
            } catch (const std::length_error& e) {
                emit_event({{"event","slicing_error"},{"phase","export_gcode"},{"kind","length_error"},{"message",std::string(e.what())}});
                std::cerr << "\n❌ G-code export failed (length_error): " << e.what() << "\n";
                return 1;
            } catch (const std::exception& e) {
                emit_event({{"event","slicing_error"},{"phase","export_gcode"},{"kind",typeid(e).name()},{"message",std::string(e.what())}});
                std::cerr << "\n❌ G-code export failed (" << typeid(e).name() << "): " << e.what() << "\n";
                std::cerr << "\nSlicing succeeded, but export needs additional configuration.\n";
                return 1;
            } catch (...) {
                emit_event({{"event","slicing_error"},{"phase","export_gcode"},{"kind","unknown"},{"message","unknown exception"}});
                std::cerr << "\n❌ G-code export failed (unknown exception)\n";
                return 1;
            }

        } catch (const Slic3r::RuntimeError& e) {
            emit_event({{"event","slicing_error"},{"phase","process"},{"kind","RuntimeError"},{"message",std::string(e.what())}});
            std::cerr << "Slicing RuntimeError: " << e.what() << "\n";
            std::cerr << "\nThis may indicate missing or incomplete configuration.\n";
            std::cerr << "Try using BambuStudio config files with --machine, --filament, --process\n";
            return 1;
        } catch (const std::length_error& e) {
            emit_event({{"event","slicing_error"},{"phase","process"},{"kind","length_error"},{"message",std::string(e.what())}});
            std::cerr << "Slicing length_error: " << e.what() << "\n";
            return 1;
        } catch (const std::exception& e) {
            emit_event({{"event","slicing_error"},{"phase","process"},{"kind",typeid(e).name()},{"message",std::string(e.what())}});
            std::cerr << "Slicing error (" << typeid(e).name() << "): " << e.what() << "\n";
            std::cerr << "\nThis may indicate missing or incomplete configuration.\n";
            std::cerr << "Try using BambuStudio config files with --machine, --filament, --process\n";
            return 1;
        } catch (...) {
            emit_event({{"event","slicing_error"},{"phase","process"},{"kind","unknown"},{"message","unknown exception"}});
            std::cerr << "Slicing error: unknown exception\n";
            return 1;
        }

        return 0;

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
}

/// --info FILE: what the file is and which binary of this package slices it,
/// as one JSON document on stdout. The plugin calls this first to pick the
/// binary. The app that made the file is reported, and is the tie-break when
/// both engines have the printer; it never refuses anything by itself.
static int run_info(const std::string& argv0, const std::string& path) {
    json out;
    out["file"] = path;
    if (!boost::filesystem::exists(path)) {
        out["error"] = cli_error_sentence(CLI_FILE_NOTFOUND);
        std::cout << out.dump(2, ' ', false, json::error_handler_t::replace) << std::endl;
        return CLI_FILE_NOTFOUND;
    }
    const bool is_3mf = input_is_3mf(path);
    out["kind"] = is_3mf ? "3mf" : (input_is_stl(path) ? "stl" : "unknown");

    std::string printer_model;
    std::string maker_app;
    if (is_3mf) {
        std::string settings_text, model_xml;
        json settings = json::object();
        if (read_zip_member(path, "Metadata/project_settings.config", settings_text)) {
            try { settings = json::parse(settings_text); } catch (...) {}
        }
        auto text = [&](const char* key) -> json {
            return settings.contains(key) ? settings[key] : json(nullptr);
        };
        if (settings.contains("printer_model") && settings["printer_model"].is_string())
            printer_model = settings["printer_model"].get<std::string>();
        out["printer_model"]   = printer_model.empty() ? json(nullptr) : json(printer_model);
        out["printer_preset"]  = text("printer_settings_id");
        out["process_preset"]  = text("print_settings_id");
        out["filament_presets"] = text("filament_settings_id");
        out["nozzle_diameter"] = text("nozzle_diameter");
        out["plates"] = std::max(1, count_3mf_plates(path));

        std::map<std::string, std::string> meta;
        if (read_zip_member(path, "3D/3dmodel.model", model_xml))
            meta = model_metadata(model_xml);
        json maker = json::object();
        const auto app = meta.find("Application");
        const auto orca = meta.find("OrcaSlicer");
        maker["application"] = app != meta.end() ? json(app->second) : json(nullptr);
        maker["orcaslicer"]  = orca != meta.end() ? json(orca->second) : json(nullptr);
        // OrcaSlicer writes "BambuStudio-<its Bambu base>" as Application and
        // its own version in the OrcaSlicer tag (bbs_3mf.cpp 6837 at 31f6803).
        if (orca != meta.end() || (app != meta.end() && boost::starts_with(app->second, "OrcaSlicer-")))
            maker_app = "OrcaSlicer";
        else if (app != meta.end() && boost::starts_with(app->second, "BambuStudio-"))
            maker_app = "BambuStudio";
        else if (app != meta.end())
            maker_app = app->second.substr(0, app->second.find('-'));
        maker["app"] = maker_app.empty() ? json(nullptr) : json(maker_app);
        out["maker"] = maker;

        json this_engine = {{"version", engine_version_text()}};
        if (const auto v = engine_file_version(meta)) {
            this_engine["file_version"] = v->to_string();
            this_engine["file_newer_than_engine"] = file_newer_than_engine(*v);
        }
        out["this_engine_reads"] = this_engine;
    } else {
        out["plates"] = 1;
    }

    EngineFit fit = engine_fit_for(argv0, printer_model);
    json engines = json::array();
    json fits = json::array();
    auto add = [&](const std::string& app, const std::string& binary, bool found, bool has) {
        const bool fits_printer = printer_model.empty() ? found : has;
        engines.push_back(json{{"app", app}, {"binary", binary}, {"catalog_found", found},
                               {"has_printer", printer_model.empty() ? json(nullptr) : json(has)}});
        if (fits_printer) fits.push_back(binary);
    };
    add(fit.this_app, fit.this_binary, fit.this_catalog_found, fit.this_has);
    add(fit.other_app, fit.other_binary, fit.other_catalog_found, fit.other_has);
    out["engines"] = engines;
    out["fits"] = fits;
    json recommended = nullptr;
    for (const auto& e : engines)
        if (std::find(fits.begin(), fits.end(), e["binary"]) != fits.end() && e["app"] == maker_app)
            recommended = e["binary"];
    if (recommended.is_null() && !fits.empty() && !printer_model.empty())
        recommended = fits.front();
    out["recommended"] = recommended;
    std::cout << out.dump(2, ' ', false, json::error_handler_t::replace) << std::endl;
    return 0;
}


/// --export-3mf: the sliced project, written with store_bbs_3mf as the
/// official CLI writes it (CLI::export_project, BambuStudio.cpp 8470-8505 at
/// 5873b5f: Silence | WithGcode | SplitModel | UseLoadedId | ShareMesh, one
/// plate or all). A 3MF input is reloaded whole (every plate, its own
/// positions and settings) and each sliced plate gets its G-code and slice
/// facts; an STL input is the one placed plate with the settings it sliced
/// with. No thumbnails are rendered (the CLI has no OpenGL), so a project
/// keeps the plate pictures it came with.
static int export_sliced_3mf(const CliOptions& o, const boost::filesystem::path& outdir,
                             std::vector<PlateOutcome>& outcomes, std::string& error) {
    using namespace Slic3r;
    const std::string path = (outdir / o.export_3mf).string();
    Model model;
    model.set_backup_path((boost::filesystem::temp_directory_path() /
                           boost::filesystem::unique_path("slicer_cli_export-%%%%%%%%")).string());
    DynamicPrintConfig config;
    PlateDataPtrs plates;
    std::vector<Preset*> project_presets;
    struct Release {
        PlateDataPtrs& plates; std::vector<Preset*>& presets;
        ~Release() { release_PlateData_list(plates); for (Preset* p : presets) delete p; presets.clear(); }
    } release{plates, project_presets};

    if (input_is_stl(o.input_file)) {
        if (outcomes.empty() || !outcomes.front().export_model || !outcomes.front().plate_data) {
            error = "Nothing was sliced to export.";
            return CLI_EXPORT_3MF_ERROR;
        }
        model  = *outcomes.front().export_model;
        config = *outcomes.front().export_config;
        auto* pd = new PlateData(*outcomes.front().plate_data);
        pd->plate_index = 0;
        pd->objects_and_instances.clear();
        for (size_t oi = 0; oi < model.objects.size(); ++oi)
            for (size_t ii = 0; ii < model.objects[oi]->instances.size(); ++ii)
                pd->objects_and_instances.emplace_back(int(oi), int(ii));
        plates.push_back(pd);
    } else {
        config.apply(FullPrintConfig::defaults(), true);
        ConfigSubstitutionContext subst(ForwardCompatibilitySubstitutionRule::Enable);
        bool is_bbl_3mf = false;
        Semver file_version;
        const auto strategy = LoadStrategy::LoadModel | LoadStrategy::LoadConfig |
                              LoadStrategy::AddDefaultInstances | LoadStrategy::LoadAuxiliary;
        // The project carries the settings its G-code was sliced with, as the
        // official export stores m_print_config: the file's settings with the
        // command line applied (BambuStudio.cpp 4091, 8156-8157), never a
        // plate's own overlay (applied to a copy, 6902-6904). So the reload
        // reads what the slice read: on the Bambu build the copy with
        // percentage line widths in mm (project and object scope), then the
        // same --config/--machine/--process/--filament files in the same
        // order, then the command-line overrides.
        std::string load_path = o.input_file;
#ifdef ENGINE_BAMBU
        PercentRewrite percent_rewrite;
        rewrite_percent_line_widths(o.input_file, percent_rewrite);
        if (!percent_rewrite.refusal.empty()) {
            error = percent_rewrite.refusal;
            return CLI_EXPORT_3MF_ERROR;
        }
        if (percent_rewrite.changed)
            load_path = percent_rewrite.temp_path;
#endif
#ifdef ENGINE_ORCA
        bool is_orca_3mf = false;
        const bool loaded = load_bbs_3mf(load_path.c_str(), &config, &subst, &model, &plates, &project_presets,
                                         &is_bbl_3mf, &is_orca_3mf, &file_version, nullptr, strategy, nullptr, 0);
#else
        const bool loaded = load_bbs_3mf(load_path.c_str(), &config, &subst, &model, &plates, &project_presets,
                                         &is_bbl_3mf, &file_version, nullptr, strategy, nullptr, 0);
#endif
        if (!loaded) {
            error = "The input could not be reloaded for export.";
            return CLI_EXPORT_3MF_ERROR;
        }
        // The slice already reported a profile that did not load; the same
        // file fails the same way here and leaves the settings as they were.
        for (const std::string* profile : {&o.bundle_config, &o.machine_config, &o.process_config, &o.filament_config}) {
            if (profile->empty()) continue;
#ifdef ENGINE_BAMBU
            bool supplied_nozzle_map = false;   // same parse as the slice's load_profile
            load_json_config(*profile, config, false, std::string(), &supplied_nozzle_map);
#else
            load_json_config(*profile, config, false, std::string(), nullptr);
#endif
        }
        apply_command_line_overrides(config, o.overrides, /*report_rejections=*/false);
        // A 3MF without plate metadata sliced as one plate holding every
        // object: the official CLI's PartPlateList always has that plate and
        // exports it (partplate_list.store_to_3mf_structure, BambuStudio.cpp
        // 7483-7484, then export_project 8156), so the project gets plate 1
        // here too, as the STL branch above builds it.
        if (plates.empty() && !outcomes.empty() && outcomes.front().plate_id == 1 && outcomes.front().plate_data) {
            auto* pd = new PlateData(*outcomes.front().plate_data);
            pd->plate_index = 0;
            pd->objects_and_instances.clear();
            for (size_t oi = 0; oi < model.objects.size(); ++oi)
                for (size_t ii = 0; ii < model.objects[oi]->instances.size(); ++ii)
                    pd->objects_and_instances.emplace_back(int(oi), int(ii));
            plates.push_back(pd);
        }
        // The slice ran its custom G-code with the legacy placeholder aliased
        // (normalize_legacy_gcode_tokens, before print.apply); the project
        // states the same templates. The slice already reported the alias.
        if (o.normalize_legacy_gcode)
            normalize_legacy_gcode_tokens(config, /*report_event=*/false);
        for (const PlateOutcome& out : outcomes) {
            if (!out.plate_data) continue;
            const int idx = out.plate_id - 1;
            if (idx < 0 || idx >= int(plates.size())) continue;
            PlateData* pd = plates[idx];
            const PlateData& sliced = *out.plate_data;
            pd->gcode_file               = sliced.gcode_file;
            pd->is_sliced_valid          = true;
            pd->gcode_prediction         = sliced.gcode_prediction;
            pd->gcode_weight             = sliced.gcode_weight;
            pd->toolpath_outside         = sliced.toolpath_outside;
            pd->timelapse_warning_code   = sliced.timelapse_warning_code;
            pd->is_label_object_enabled  = sliced.is_label_object_enabled;
            pd->limit_filament_maps      = sliced.limit_filament_maps;
            pd->layer_filaments          = sliced.layer_filaments;
            pd->filament_change_sequence = sliced.filament_change_sequence;
            pd->nozzle_change_sequence   = sliced.nozzle_change_sequence;
            pd->optimal_assignment       = sliced.optimal_assignment;
            pd->filament_maps            = sliced.filament_maps;
            pd->is_support_used          = sliced.is_support_used;
            pd->slice_filaments_info     = sliced.slice_filaments_info;
            // --arrange moved this plate's objects: carry the same moves
            // into the project, matched by the id the file gave each instance.
            for (const auto& [obj_idx, inst_idx] : pd->objects_and_instances) {
                if (obj_idx < 0 || obj_idx >= int(model.objects.size())) continue;
                ModelObject* object = model.objects[obj_idx];
                if (inst_idx < 0 || inst_idx >= int(object->instances.size())) continue;
                ModelInstance* inst = object->instances[inst_idx];
                const auto moved = out.moved.find(inst->loaded_id);
                if (moved != out.moved.end())
                    inst->set_offset(inst->get_offset() + moved->second);
            }
#ifdef ENGINE_BAMBU
            // The routing the slice derived from a supplied nozzle map: the
            // map and Nozzle Manual in the project settings and on the plate
            // (store_bbs_3mf writes a plate's filament_map_mode and
            // filament_maps from its config, bbs_3mf.cpp 8260-8279 at 5873b5f;
            // PartPlate::set_filament_maps keeps the map there, PartPlate.cpp
            // 3860-3863), and this plate's objects on the master nozzle's slot
            // as reassign_objects_to_master_nozzle put them.
            if (!out.derived_filament_map.empty()) {
                std::vector<int> map0(out.derived_filament_map.size());
                for (size_t i = 0; i < map0.size(); ++i) map0[i] = out.derived_filament_map[i] - 1;
                ConfigSubstitutionContext mode_subst(ForwardCompatibilitySubstitutionRule::Enable);
                for (DynamicPrintConfig* target : {&config, static_cast<DynamicPrintConfig*>(&pd->config)}) {
                    target->option<ConfigOptionInts>("filament_map", true)->values = out.derived_filament_map;
                    target->set_deserialize("filament_map_mode", "Nozzle Manual", mode_subst);
                }
                config.option<ConfigOptionInts>("filament_map_2", true)->values = map0;
                if (out.master_filament_slot > 0) {
                    std::vector<ModelObject*> plate_objects;
                    for (const auto& [obj_idx, inst_idx] : pd->objects_and_instances)
                        if (obj_idx >= 0 && obj_idx < int(model.objects.size()) &&
                            std::find(plate_objects.begin(), plate_objects.end(), model.objects[obj_idx]) == plate_objects.end())
                            plate_objects.push_back(model.objects[obj_idx]);
                    assign_objects_to_filament_slot(plate_objects, out.master_filament_slot);
                }
            }
#endif
        }
    }

    StoreParams store;
    store.path = path.c_str();
    store.model = &model;
    store.plate_data_list = plates;
    store.project_presets = project_presets;
    store.config = &config;
    store.strategy = SaveStrategy::Silence | SaveStrategy::WithGcode | SaveStrategy::SplitModel |
                     SaveStrategy::UseLoadedId | SaveStrategy::ShareMesh;
    store.export_plate_idx = o.slice_plate - 1;
    if (!store_bbs_3mf(store)) {
        error = "Writing " + path + " failed.";
        return CLI_EXPORT_3MF_ERROR;
    }
    emit_event({{"event","exported_3mf"}, {"tag","SlicedProjectWritten"}, {"path", path},
                {"message","Wrote the sliced project " + path}});
    return 0;
}

/// --slice: every plate (0) or plate N of the input, one G-code each as
/// <outputdir>/plate_N.gcode, through the same single-plate path the default
/// call runs. The official loop stops at the first plate that fails
/// (flush_and_exit inside the plate loop, BambuStudio.cpp 6533-7240), and so
/// does this one. The exit status is the official CLI_* code of that failure.
static int run_slice_mode(const CliOptions& o, Slic3r::Calib_Params& calib_params) {
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
            write_result_json(outdir.string(), CLI_ENVIRONMENT_ERROR, o.slice_plate,
                              cli_error_sentence(CLI_ENVIRONMENT_ERROR) + " " + detail, {}, 0, 0);
            std::cerr << "Error: " << detail << "\n";
            emit_event({{"event","input_error"}, {"tag","InputUnreadable"}, {"path", o.input_file},
                        {"message", detail}});
            return CLI_ENVIRONMENT_ERROR;
        }
        if (!input_exists) {
            write_result_json(outdir.string(), CLI_FILE_NOTFOUND, o.slice_plate,
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
    if (input_is_3mf(o.input_file)) {
        const int declared = count_3mf_plates(o.input_file);
        if (declared > 0) {
            plate_count = declared;
            per_plate_load = true;
        }
    }
    if (o.slice_plate > plate_count) {
        write_result_json(outdir.string(), CLI_INVALID_PARAMS, o.slice_plate,
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

    // --export-3mf NAME is written into --outputdir next to result.json and
    // plate_N.gcode; a NAME that is one of them would overwrite it (or be
    // overwritten). Compared without case: Windows and macOS file systems
    // treat Result.json and result.json as one file. Links are followed
    // (the NAME's own link chain, then weakly_canonical for every existing
    // part),
    // and two existing names for one file (a hard link) are caught by
    // fs::equivalent, so a NAME that is another name for one of them is
    // refused too.
    if (!o.export_3mf.empty()) {
        const auto key = [](const fs::path& p) {
            // A link whose target does not exist yet (result.json before
            // the run writes it) is not resolved by weakly_canonical: follow
            // the chain by hand first.
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
        const fs::path target_path = outdir / o.export_3mf;
        const std::string target = key(target_path);
        std::vector<std::string> taken = {"result.json"};
        for (int p = (o.slice_plate == 0 ? 1 : o.slice_plate);
             p <= (o.slice_plate == 0 ? plate_count : o.slice_plate); ++p)
            taken.push_back("plate_" + std::to_string(p) + ".gcode");
        for (const std::string& name : taken) {
            boost::system::error_code ec;
            const bool same_file = fs::equivalent(target_path, outdir / name, ec) && !ec;
            if (!same_file && !boost::algorithm::iequals(target, key(outdir / name))) continue;
            const std::string detail = "--export-3mf " + o.export_3mf + " is the run's own " + name +
                                       " in --outputdir; choose another name.";
            write_result_json(outdir.string(), CLI_INVALID_PARAMS, o.slice_plate,
                              cli_error_sentence(CLI_INVALID_PARAMS) + " " + detail, {}, 0, 0);
            std::cerr << "Error: " << detail << "\n";
            emit_event({{"event","input_error"}, {"tag","ExportNameTaken"}, {"message", detail}});
            return CLI_INVALID_PARAMS;
        }
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

    for (size_t i = 0; i < plates.size(); ++i) {
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
        const int rc = slice_one_plate(o, calib_params, per_plate_load ? plates[i] : 0,
                                       outcome.gcode_path, outcome);
        // A failure that set no official code (an engine exception in the
        // process or export step) is the official CLI's slicing failure with
        // its sentence (record_exit_reson(..., CLI_SLICING_ERROR, index+1,
        // cli_errors[CLI_SLICING_ERROR], ...), BambuStudio.cpp 7139, 7423);
        // the exception text is in the slicing_error event.
        if (rc != 0 && outcome.cli_code == 0)
            set_outcome_failure(outcome, CLI_SLICING_ERROR);
        if (outcome.cli_code != 0 && outcome.error_string.empty())
            outcome.error_string = cli_error_sentence(outcome.cli_code);
        outcomes.push_back(outcome);
        if (outcome.cli_code != 0) {
            code = outcome.cli_code;
            error_string = outcome.error_string;
            break;
        }
    }
    long long export_ms = 0;
    if (code == 0 && !o.export_3mf.empty()) {
        const auto export_started = std::chrono::steady_clock::now();
        if (o.progress) {
            PlateOutcome last = outcomes.empty() ? PlateOutcome() : outcomes.back();
            emit_run_progress(last, 97, "Exporting 3mf");
        }
        std::string export_error;
        try {
            code = export_sliced_3mf(o, outdir, outcomes, export_error);
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
    const int reported_plate = (code != 0 && !outcomes.empty()) ? outcomes.back().plate_id : o.slice_plate;
    const bool result_written = write_result_json(outdir.string(), code, reported_plate, error_string, outcomes,
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
    return code;
}

static int run_cli_slice(const CliOptions& o, Slic3r::Calib_Params& calib_params) {
    if (o.slice_mode)
        return run_slice_mode(o, calib_params);
    PlateOutcome outcome;
    outcome.plate_id = o.plate_id;
    const int rc = slice_one_plate(o, calib_params, o.plate_id, o.output_file, outcome);
    // --progress on the single-file call ends like --slice: 100 once the
    // G-code is written ("All done, Success", BambuStudio.cpp 8237).
    if (rc == 0 && o.progress)
        emit_run_progress(outcome, 100, "All done, Success");
    return rc;
}

int main(int argc, char** argv) {
    // Initialize libslic3r
    Slic3r::set_logging_level(3); // Info level
    boost::log::core::get()->set_logging_enabled(true);

    // Parse arguments
    CliOptions o;
    o.argv0 = argv[0];
    std::string& input_file = o.input_file;
    std::string& output_file = o.output_file;
    std::string& machine_config = o.machine_config;
    std::string& filament_config = o.filament_config;
    std::string& process_config = o.process_config;
    std::string& bundle_config = o.bundle_config;
    bool& verbose = o.verbose;
    int& plate_id = o.plate_id;  // 0 = all plates (default); >0 = slice only that plate
    bool& normalize_legacy_gcode = o.normalize_legacy_gcode;
    std::string layout_json_file;
    std::string info_file;
    bool        list_presets = false;
    std::string list_printer;
    bool        layout_plan_mode = false;
    slicer_cli::CalibOptions calib_opts;
    // Override settings

    std::map<std::string, std::string>& overrides = o.overrides;


    // subcommand: slicer_cli layout capabilities --json
    if (argc >= 3 && std::string(argv[1]) == "layout" && std::string(argv[2]) == "capabilities") {
        if (argc != 4 || std::string(argv[3]) != "--json") {
            std::cerr << "Usage: " << argv[0] << " layout capabilities --json\n";
            return 1;
        }
        layout_plan::install_cancellation_handler();  // ignore SIGPIPE so write failures surface as errors
        boost::log::core::get()->set_logging_enabled(false);
        return layout_plan::run_capabilities();
    }

    // Both engines read slice-time resources (info/, flush/, filament_mixing/)
    // relative to this root, which depends on no argument: resolve it once here
    // so the ENGINE_ORCA build gets it too, and so STL and calibration slices
    // stop falling back to the hardcoded tables.
    {
        bool layout_plan_json = false;
        for (int i = 1; i < argc; ++i)
            if (std::string(argv[i]) == "--layout-plan")
                layout_plan_json = true;
        configure_engine_resources(argv[0], layout_plan_json);
    }

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];

        if (arg == "-h" || arg == "--help") {
            print_usage(argv[0]);
            return 0;
        } else if (arg == "-v" || arg == "--verbose") {
            verbose = true;
            Slic3r::set_logging_level(5);
        } else if ((arg == "-o" || arg == "--output") && i + 1 < argc) {
            output_file = argv[++i];
        } else if (arg == "--machine" && i + 1 < argc) {
            machine_config = argv[++i];
        } else if (arg == "--filament" && i + 1 < argc) {
            filament_config = argv[++i];
        } else if (arg == "--process" && i + 1 < argc) {
            process_config = argv[++i];
        } else if (arg == "--config" && i + 1 < argc) {
            bundle_config = argv[++i];
        } else if (arg == "--layer-height" && i + 1 < argc) {
            overrides["layer_height"] = argv[++i];
        } else if (arg == "--infill" && i + 1 < argc) {
            overrides["fill_density"] = argv[++i];
        } else if (arg == "--perimeters" && i + 1 < argc) {
            overrides["perimeters"] = argv[++i];
        } else if (arg == "--nozzle" && i + 1 < argc) {
            overrides["nozzle_diameter"] = argv[++i];
        } else if (arg == "--temp" && i + 1 < argc) {
            overrides["nozzle_temperature"] = argv[++i];
        } else if (arg == "--bed-temp" && i + 1 < argc) {
            overrides["bed_temperature"] = argv[++i];
        } else if (arg == "--plate" && i + 1 < argc) {
            plate_id = std::stoi(argv[++i]);
        } else if (arg == "--input" && i + 1 < argc) {
            input_file = argv[++i];
        } else if (arg == "--no-normalize-legacy-gcode") {
            normalize_legacy_gcode = false;
        } else if (arg == "--calib-mode" && i + 1 < argc) {
            calib_opts.mode = argv[++i];
        } else if (arg == "--calib-start" && i + 1 < argc) {
            calib_opts.start = parse_cli_double("--calib-start", argv[++i], argv[0]); calib_opts.has_start = true;
        } else if (arg == "--calib-end" && i + 1 < argc) {
            calib_opts.end = parse_cli_double("--calib-end", argv[++i], argv[0]); calib_opts.has_end = true;
        } else if (arg == "--calib-step" && i + 1 < argc) {
            calib_opts.step = parse_cli_double("--calib-step", argv[++i], argv[0]); calib_opts.has_step = true;
        } else if (arg == "--calib-extruder-id" && i + 1 < argc) {
            calib_opts.extruder_id = parse_cli_int("--calib-extruder-id", argv[++i], argv[0]);
        } else if (arg == "--calib-no-numbers") {
            calib_opts.print_numbers = false;
        } else if (arg == "--layout" && i + 1 < argc) {
            layout_json_file = argv[++i];
        } else if (arg == "--layout-plan") {
            layout_plan_mode = true;
        } else if (arg == "--slice" && i + 1 < argc) {
            // Official meaning (BambuStudio.cpp 6438 at 5873b5f): 0 = every
            // plate, N = plate N; each written as plate_N.gcode in --outputdir.
            o.slice_mode  = true;
            o.slice_plate = parse_cli_int("--slice", argv[++i], argv[0]);
            if (o.slice_plate < 0) {
                std::cerr << "Error: --slice expects 0 (every plate) or a plate number\n\n";
                print_usage(argv[0]);
                return 1;
            }
        } else if (arg == "--outputdir" && i + 1 < argc) {
            o.outputdir = argv[++i];
        } else if (arg == "--progress") {
            o.progress = true;
        } else if (arg == "--printer-preset" && i + 1 < argc) {
            o.printer_preset = argv[++i];
        } else if (arg == "--process-preset" && i + 1 < argc) {
            o.process_preset = argv[++i];
        } else if (arg == "--filament-preset" && i + 1 < argc) {
            o.filament_presets.push_back(argv[++i]);   // repeat for more filaments
        } else if (arg == "--list-presets") {
            list_presets = true;
        } else if (arg == "--printer" && i + 1 < argc) {
            list_printer = argv[++i];
        } else if (arg == "--export-3mf" && i + 1 < argc) {
            o.export_3mf = argv[++i];
        } else if (arg == "--arrange" && i + 1 < argc) {
            o.arrange = parse_cli_int("--arrange", argv[++i], argv[0]) != 0;
        } else if (arg == "--allow-newer-file") {
            o.allow_newer_file = true;
        } else if (arg == "--allow-substitution") {
            o.allow_substitution = true;
        } else if (arg == "--info" && i + 1 < argc) {
            info_file = argv[++i];
        } else if (arg[0] != '-') {
            input_file = arg;
        } else {
            std::cerr << "Unknown option: " << arg << "\n";
            print_usage(argv[0]);
            return 1;
        }
    }

    // cli #5: resolve the calibration mode/params up front so a bad --calib-*
    // value fails fast with usage, before any model/config work.
    Slic3r::Calib_Params calib_params;
    try {
        calib_params = slicer_cli::build_calib_params(calib_opts);
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n\n";
        print_usage(argv[0]);
        return 1;
    }
    const bool calib_self_geometry = slicer_cli::calib_mode_generates_geometry(calib_params.mode);

#ifdef ENGINE_ORCA
    // pressure_advance_pattern's geometry generator is ported only for the Bambu
    // engine (Orca's CalibPressureAdvancePattern API differs); reject it cleanly
    // here so the Orca binary fails fast instead of throwing from apply_pa_pattern.
    if (calib_params.mode == Slic3r::CalibMode::Calib_PA_Pattern) {
        std::cerr << "Error: pressure_advance_pattern is not yet supported on the OrcaSlicer "
                     "engine; use a tower or pressure_advance_line calib mode instead.\n";
        return 1;
    }
#endif

    // --info: one JSON document on stdout, nothing sliced.
    if (!info_file.empty()) {
        boost::log::core::get()->set_logging_enabled(false);
        return run_info(o.argv0, info_file);
    }

    if (list_presets) {
        boost::log::core::get()->set_logging_enabled(false);
        return run_list_presets(o, list_printer);
    }
    if (o.uses_presets()) {
        if (!input_is_stl(input_file)) {
            std::cerr << "Error: --printer-preset/--process-preset/--filament-preset apply to an STL; "
                         "a 3MF carries its own settings\n";
            return 1;
        }
        g_preset_config = std::make_unique<Slic3r::DynamicPrintConfig>();
        int code = 0;
        std::string error;
        if (!resolve_named_presets(o, *g_preset_config, code, error)) {
            emit_event({{"event","preset_error"}, {"tag","NamedPresetRefused"}, {"message", error}});
            std::cerr << "Error: " << error << "\n";
            if (o.slice_mode) {
                boost::system::error_code mk;
                boost::filesystem::create_directories(o.outputdir.empty() ? "." : o.outputdir, mk);
                write_result_json(o.outputdir.empty() ? "." : o.outputdir, code, o.slice_plate,
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
    if (o.arrange && !o.slice_mode) {
        std::cerr << "Error: --arrange needs --slice (the official CLI arranges inside its plate loop)\n";
        return 1;
    }
    if (o.slice_mode)
        o.progress = true;   // --slice reports progress like the official --pipe
    if (o.slice_mode && plate_id > 0) {
        std::cerr << "Error: --plate and --slice are mutually exclusive; use --slice N for plate N\n";
        return 1;
    }

    // Detect conflicting layout flags
    if (layout_plan_mode && !layout_json_file.empty()) {
        std::cerr << "Error: --layout-plan and --layout are mutually exclusive\n";
        return 1;
    }

    // --layout-plan: versioned headless arrange contract (issue #7)
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
                fd = ::_open(input_file.c_str(), _O_RDONLY | _O_BINARY);  // binary: no Ctrl+Z EOF, no newline translation
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

    // --layout: headless arrange spike (issue #7 milestone 1)
    if (!layout_json_file.empty()) {
        std::ifstream lf(layout_json_file);
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
            for (auto& [_, path] : lj["profiles"].items()) {
                if (!load_json_config(profiles_dir + "/" + path.get<std::string>(), cfg)) {
                    std::cerr << "Failed to load profile: " << path.get<std::string>() << "\n";
                    return 1;
                }
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
        #ifdef ENGINE_ORCA
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

    if (input_file.empty() && !calib_self_geometry) {
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
            std::cerr << "Error: pressure_advance_pattern requires a .3mf --input for its "
                         "printer/filament config (it discards the model geometry but reads the "
                         "embedded config + plate setup; a profile bundle or STL is not "
                         "sufficient)\n\n";
            print_usage(argv[0]);
            return 1;
        }
    }

    std::cout << "libslic3r_standalone - Standalone slicing tool\n";
    std::cout << "Based on BambuStudio libslic3r\n\n";

    return run_cli_slice(o, calib_params);
}
