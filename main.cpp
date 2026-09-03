// libslic3r_standalone - CLI tool for slicing with libslic3r
// Supports loading BambuStudio JSON config files (machine, filament, process)

#include <iostream>
#include <fstream>
#include <string>
#include <memory>
#include <map>
#include <vector>
#include <cstdlib>
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
#include "libslic3r/miniz_extension.hpp"
#include "libslic3r/ModelArrange.hpp"
#include "libslic3r/Arrange.hpp"
#include "libslic3r/BoundingBox.hpp"

// For JSON parsing (using libslic3r's built-in nlohmann/json)
#include <nlohmann/json.hpp>

#include <boost/filesystem.hpp>

// Boost.Log bridge: libslic3r raises most of its diagnostics through
// BOOST_LOG_TRIVIAL and never through any callback. `libslic3r_core` already
// links Boost::log and Boost::log_setup unconditionally (CMakeLists.txt — the
// same list for ENGINE=bambu and ENGINE=orca), so attaching one more sink adds
// no new dependency on either engine.
#include <mutex>
#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>
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

// Serializes event writes. `set_status_callback` already fires from the TBB
// worker threads inside process(), and the Boost.Log sink added below can fire
// from any thread that logs; without this lock two events can interleave inside
// one line and corrupt both JSON objects. (Pre-existing latent defect: the
// status-callback path was already multi-threaded.)
std::mutex g_event_mutex;

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
    } catch (...) {
        return;  // a diagnostic must never be the reason a slice fails
    }
    std::lock_guard<std::mutex> lock(g_event_mutex);
    std::cout << SLICER_EVENT_PREFIX << line << '\n';
    std::cout.flush();
}

// ── Channel 1: config deserialization findings ──────────────────────────────
// `ConfigSubstitutionContext` (Config.hpp:255) accumulates BOTH the forward-
// compatibility substitutions the engine silently performed AND the keys it did
// not recognize at all. The GUI shows these ("some settings were incompatible
// and have been substituted"); the CLI constructs the context, hands it to
// load_bbs_3mf / load_from_json, and then never reads it.
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
const char* slice_warning_level_tag(int level) {
    switch (level) {
        case 0:  return "tip";
        case 1:  return "warning";
        case 2:  return "error";
        default: return "unknown";
    }
}

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
    if (r.filament_printable_reuslt.has_value()) {  // sic — upstream spelling
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

void install_engine_log_bridge() {
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
int normalize_legacy_gcode_tokens(Slic3r::DynamicPrintConfig& config) {
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
    if (total_rewrites > 0) {
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

// Load JSON config file and apply to DynamicPrintConfig
bool load_json_config(const std::string& filepath, Slic3r::DynamicPrintConfig& config, bool verbose = false) {
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
            // Skip metadata fields
            if (key == "type" || key == "name" || key == "inherits" ||
                key == "from" || key == "setting_id" || key == "instantiation" ||
                key == "description" || key == "compatible_printers" ||
                key == "compatible_prints" || key == "include" ||
                key == "upward_compatible_machine" || key == "printer_model" ||
                key == "printer_variant" || key == "default_filament_profile" ||
                key == "default_print_profile") {
                continue;
            }

            try {
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
                    // set_deserialize respects the existing option type
                    config.set_deserialize(key, value_str, substitution_context);
                }
            } catch (const std::exception& e) {
                if (verbose) {
                    std::cerr << "Warning: Failed to set config key '" << key << "': " << e.what() << "\n";
                }
            }
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
bool apply_explicit_nozzle_mapping(Slic3r::DynamicPrintConfig& config)
{
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
    // assignment.  In this case the input 3MF's nozzle map is the authoritative
    // source for which filament goes on which physical nozzle.  Without real AMS
    // data the fmmAutoForFlush algorithm always assigns every filament to the
    // master extruder, overriding the correct split.  Switching to Nozzle Manual
    // preserves the derived_map computed above.
    {
        Slic3r::ConfigSubstitutionContext substitution_context(Slic3r::ForwardCompatibilitySubstitutionRule::Enable);
        config.set_deserialize("filament_map_mode", "Nozzle Manual", substitution_context);
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
void reassign_objects_to_master_nozzle(Slic3r::Model& model, const Slic3r::DynamicPrintConfig& config)
{
    const auto* filament_map = config.option<Slic3r::ConfigOptionInts>("filament_map");
    const auto* physical_extruder_map = config.option<Slic3r::ConfigOptionInts>("physical_extruder_map");
    if (!filament_map || !physical_extruder_map)
        return;

    const size_t extruder_count = physical_extruder_map->values.size();
    if (extruder_count < 2)
        return;

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
        return;

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
    if (master_filament_slot < 0)
        return;

    // Reassign each object's extruder to the master nozzle's filament slot.
    for (auto* obj : model.objects) {
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

int main(int argc, char** argv) {
    // Initialize libslic3r
    Slic3r::set_logging_level(3); // Info level

    // Parse arguments
    std::string input_file;
    std::string output_file = "output.gcode";
    std::string machine_config;
    std::string filament_config;
    std::string process_config;
    std::string bundle_config;
    bool verbose = false;
    int plate_id = 0;  // 0 = all plates (default); >0 = slice only that plate
    bool normalize_legacy_gcode = true;
    std::string layout_json_file;
    bool        layout_plan_mode = false;
    slicer_cli::CalibOptions calib_opts;
    // Override settings

    std::map<std::string, std::string> overrides;


    // subcommand: slicer_cli layout capabilities --json
    if (argc >= 3 && std::string(argv[1]) == "layout" && std::string(argv[2]) == "capabilities") {
        if (argc != 4 || std::string(argv[3]) != "--json") {
            std::cerr << "Usage: " << argv[0] << " layout capabilities --json\n";
            return 1;
        }
        layout_plan::install_cancellation_handler();  // ignore SIGPIPE so write failures surface as errors
        return layout_plan::run_capabilities();
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
    const bool calib_enabled       = calib_params.mode != Slic3r::CalibMode::Calib_None;
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
    // Deliberately placed AFTER every layout early-return above. `layout
    // capabilities`, `--layout-plan` and `--layout` all treat stdout as a
    // strict single-JSON-document channel (see the `out.dump()` at the end of
    // the --layout branch), and ohmyhelio parses it as one — arrangement code
    // can log, and a single [[SLICER_EVENT]] line prepended to that document
    // would break the parse. The slice path below is the only path whose stdout
    // is already a mixed text stream, so it is the only one that gains events.
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
        // Mirror the loader's dispatch (it checks .stl FIRST), so a path like
        // `part.3mf.stl` — which loads as STL — is NOT mistaken for a 3MF here.
        const bool input_is_stl = input_file.find(".stl") != std::string::npos ||
                                  input_file.find(".STL") != std::string::npos;
        const bool input_is_3mf = !input_is_stl &&
                                  (input_file.find(".3mf") != std::string::npos ||
                                   input_file.find(".3MF") != std::string::npos);
        if (!input_is_3mf) {
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
        Slic3r::PlateDataPtrs plate_data;  // hoisted so it is accessible after the 3mf block

        if (input_file.find(".stl") != std::string::npos ||
            input_file.find(".STL") != std::string::npos) {
            bool result = Slic3r::load_stl(input_file.c_str(), &model);
            if (!result) {
                std::cerr << "Failed to load STL file\n";
                return 1;
            }
        } else if (input_file.find(".3mf") != std::string::npos ||
                   input_file.find(".3MF") != std::string::npos) {
            Slic3r::ConfigSubstitutionContext config_subst(Slic3r::ForwardCompatibilitySubstitutionRule::Enable);
            std::vector<Slic3r::Preset*> presets;
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
                input_file.c_str(),
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
                input_file.c_str(),
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

            if (!result) {
                emit_event({{"event","load_error"},
                            {"tag","ThreeMfLoadFailed"},
                            {"path", input_file},
                            {"message","Failed to load 3MF file"}});
                std::cerr << "Failed to load 3MF file\n";
                return 1;
            }

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
                return 1;
            }

            // ── Multi-plate coordinate translation ───────────────────────
            // Multi-plate 3MFs store objects at global positions (plate N
            // offset by N * plate_stride from the global origin).  When slicing
            // a specific plate, translate objects back to plate-local coords
            // so they land within the printer's build volume.
            // Strategy:
            //   1. If plate_N.json exists in the 3MF, use its bbox_all to get
            //      the expected plate-local min coords and apply that offset.
            //   2. If no JSON exists for this plate, translate so the objects'
            //      bounding box starts at (0, 0) — i.e. snap to bed origin.
            if (plate_id > 0 && is_bbl_3mf && !model.objects.empty()) {
                // Compute actual model bounding box (with instance transforms)
                Slic3r::BoundingBoxf3 actual_bbox;
                for (auto* obj : model.objects) {
                    for (size_t i = 0; i < obj->instances.size(); i++) {
                        actual_bbox.merge(obj->instance_bounding_box(i));
                    }
                }

                double expected_min_x = 0.0;
                double expected_min_y = 0.0;
                bool is_seq_print_plate = false;

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
                                if (plate_json.contains("bbox_all") && plate_json["bbox_all"].size() >= 4) {
                                    expected_min_x = plate_json["bbox_all"][0].get<double>();
                                    expected_min_y = plate_json["bbox_all"][1].get<double>();
                                }
                                if (plate_json.contains("is_seq_print"))
                                    is_seq_print_plate = plate_json["is_seq_print"].get<bool>();
                            } catch (...) {}
                        }
                    }
                    mz_zip_reader_end(&zip);
                }

                double offset_x = expected_min_x - actual_bbox.min.x();
                double offset_y = expected_min_y - actual_bbox.min.y();
                if (std::abs(offset_x) > 1.0 || std::abs(offset_y) > 1.0) {
                    if (verbose)
                        std::cout << "Plate " << plate_id << " coord translation: ("
                                  << offset_x << ", " << offset_y << ")\n";
                    for (auto* obj : model.objects) {
                        for (auto* inst : obj->instances) {
                            Slic3r::Vec3d off = inst->get_offset();
                            inst->set_offset(Slic3r::Vec3d(
                                off.x() + offset_x,
                                off.y() + offset_y,
                                off.z()));
                        }
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
                // Find resources/profiles/ directory
                boost::filesystem::path exe_dir;
#ifdef __APPLE__
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
                    try { exe_dir = boost::filesystem::canonical(argv[0]).parent_path(); } catch (...) {}
                }
                boost::filesystem::path profiles_dir;
                for (const auto& p : std::vector<boost::filesystem::path>{
                    exe_dir / ".." / ".." / "references" / "BambuStudio" / "resources" / "profiles",
                    exe_dir / ".." / "resources" / "profiles",
                    boost::filesystem::path("/home/user/slicer/references/BambuStudio/resources/profiles"),
                }) {
                    if (boost::filesystem::exists(p) && boost::filesystem::is_directory(p)) {
                        profiles_dir = boost::filesystem::canonical(p);
                        break;
                    }
                }

                bool preset_loaded = false;
                if (!profiles_dir.empty()) {
                    try {
                        // Prepare data_dir/system/ with symlinks to vendor dirs.
                        // Always start fresh to avoid stale cached copies of
                        // vendor JSON files that can cause extruder variant
                        // lookup failures (see update_values_to_printer_extruders).
                        auto tmpdir = boost::filesystem::temp_directory_path() / "slicer_cli_presets";
                        auto sysdir = tmpdir / "system";
                        boost::filesystem::remove_all(tmpdir);
                        boost::filesystem::create_directories(sysdir);
                        for (auto& entry : boost::filesystem::directory_iterator(profiles_dir)) {
                            auto dst = sysdir / entry.path().filename();
                            if (boost::filesystem::is_directory(entry.path()))
                                boost::filesystem::create_directory_symlink(
                                    boost::filesystem::canonical(entry.path()), dst);
                            else if (entry.path().extension() == ".json")
                                boost::filesystem::copy_file(entry.path(), dst);
                        }
                        Slic3r::set_data_dir(tmpdir.string());
                        Slic3r::set_resources_dir(
                            (profiles_dir / "..").string());

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
            return 1;
        }

        // For pressure_advance_pattern the model is generated later by
        // apply_pa_pattern (before print.apply), so an empty model here is fine.
        if (model.objects.empty() && !calib_self_geometry) {
            std::cerr << "No objects found in model\n";
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
            if (load_json_config(path, config, verbose)) return;
            emit_event({{"event","config_load_failed"},
                        {"tag","ProfileLoadFailed"},
                        {"kind", kind},
                        {"path", path},
                        {"message", std::string("Failed to load ") + kind +
                                    " config; slicing continues with the previously resolved settings"}});
            std::cerr << "Warning: Failed to load " << kind << " config\n";
        };
        load_profile(bundle_config,   "bundle");
        load_profile(machine_config,  "machine");
        load_profile(process_config,  "process");
        load_profile(filament_config, "filament");

#ifdef ENGINE_BAMBU
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
            };

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

        // If the 3MF plate carries an explicit per-filament nozzle assignment,
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
            bool has_diverse_values = pm.size() >= 2 &&
                std::adjacent_find(pm.begin(), pm.end(), std::not_equal_to<int>()) != pm.end();
            bool is_manual_mode = false;
            {
                auto* mode_opt = config.option<Slic3r::ConfigOptionEnum<Slic3r::FilamentMapMode>>("filament_map_mode", false);
                if (mode_opt && mode_opt->value == Slic3r::FilamentMapMode::fmmNozzleManual)
                    is_manual_mode = true;
            }
            bool has_explicit = pm.size() >= 2 && (has_diverse_values || is_manual_mode);
            if (has_explicit) {
                auto* fm = config.option<Slic3r::ConfigOptionInts>("filament_map", true);
                fm->values = pm;  // already 1-based per PlateData docs

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

        // The selected plate already expresses physical routing when it carries
        // a diverse map. Do not re-derive that routing from the general
        // filament_nozzle_map fallback: the latter's object reassignment is
        // exclusively for an otherwise-auto mapping.
        bool nozzle_mapping_derived =
            explicit_plate_mapping_applied ? false : apply_explicit_nozzle_mapping(config);

        // When apply_explicit_nozzle_mapping derived a cross-nozzle split from
        // "Auto For Flush" mode (filament_maps="1 1"), reassign all objects to
        // the master (right) nozzle to match BambuStudio desktop behavior.
        if (nozzle_mapping_derived)
            reassign_objects_to_master_nozzle(model, config);

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

                // Single filament slot: trivially no multi-material
                auto* fd = config.option<Slic3r::ConfigOptionFloats>("filament_diameter", false);
                if (fd && fd->values.size() <= 1)
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

        // Apply command-line overrides
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
                emit_event({{"event","override_rejected"},
                            {"tag","InvalidOverrideValue"},
                            {"opt_key", key},
                            {"value", value},
                            {"message","Command-line override for '" + key + "' was rejected and had no effect"}});
                std::cerr << "Warning: Invalid value for " << key << ": " << value << "\n";
            }
        }

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
            print.set_status_callback(emit_status_warning);

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
                for (const auto& [poly, height] : height_polygons)
                    heights.push_back(json{{"height_mm", height},
                                           {"point_count", poly.points.size()}});
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
                return 1;
            }

            std::cout << "Slicing...\n";
            print.process();

            std::cout << "\n✓ Slicing complete!\n";

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
