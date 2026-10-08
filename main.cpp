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
#include <set>
#include <cstdint>
#include <cstdlib>
#include <algorithm>
#include <chrono>
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
#include "libslic3r/GCode/WipeTower.hpp"
#ifdef ENGINE_BAMBU
#include "libslic3r/FilamentMixer.hpp"
#endif
#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/BuildVolume.hpp"
#include "libslic3r/Semver.hpp"
#include "libslic3r_version.h"

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
#include "cli_options.hpp"
#include "cli_command_line.hpp"
#include "desktop_presets.hpp"
#include "cli_model_load.hpp"
#include "cli_run_steps.hpp"
#include "cli_load_settings.hpp"
#include "cli_repetitions.hpp"
#include "cli_assemble.hpp"
#include "cli_pipe.hpp"

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
        slicer_cli::diagnostics::write_event(line);   // held during --slice 0's check pass
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
// Driver-side normalization for the unbound placeholder `initial_no_support_filament_id`.
//
// OrcaSlicer does not bind `initial_no_support_filament_id` in the
// PlaceholderParser: it binds `initial_no_support_extruder` /
// `initial_no_support_hotend`, and the first is the SAME int as BambuStudio's
// `initial_no_support_filament_id` (BambuStudio binds it in GCode.cpp 2657 at
// 5873b5f, from the same `initial_non_support_extruder_id`, GCode.cpp
// 2458-2460). Because the `_extruder` and `_filament` names are aliases of the
// same filament index, the BambuStudio token is semantically identical to
// OrcaSlicer's `initial_no_support_extruder`.
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

int normalize_legacy_gcode_tokens(Slic3r::DynamicPrintConfig& config, bool report_event = true) {
    static const std::string kLegacyToken = "initial_no_support_filament_id";
    static const std::string kBoundToken  = "initial_no_support_extruder";
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
    slicer_cli::print_help(std::cout, prog_name);
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

// True when the JSON settings file at `filepath` (may be empty) names `key`.
static bool json_file_has_key(const std::string& filepath, const char* key) {
    if (filepath.empty())
        return false;
    boost::nowide::ifstream f(filepath);
    if (!f.is_open())
        return false;
    try {
        const json j = json::parse(f);
        return j.is_object() && j.contains(key);
    } catch (...) {
        return false;
    }
}

// Load JSON config and record only accepted, usable nozzle-map overlays.
bool load_json_config(const std::string& filepath, Slic3r::DynamicPrintConfig& config,
                      bool verbose = false, const std::string& diagnostic_source = {},
                      bool* supplied_nozzle_map = nullptr) {
    if (verbose) {
        std::cout << "Loading config: " << filepath << "\n";
    }

    // The path is UTF-8 (boost::nowide::args on Windows).
    boost::nowide::ifstream f(filepath);
    if (!f.is_open()) {
        std::cerr << "Error: Cannot open config file: " << filepath << "\n";
        return false;
    }

    try {
        json j = json::parse(f);

        // Create substitution context for config deserialization
        Slic3r::ConfigSubstitutionContext substitution_context(Slic3r::ForwardCompatibilitySubstitutionRule::Enable);

        // The file-level conversions the engines run for a settings file
        // (ConfigBase::load_from_json, Config.cpp 880-1096 at 31f6803): the
        // special cases at 931-948 (support_type hybrid(auto) -> support_style
        // tree_hybrid, wall_infill_order -> is_infill_first) and
        // handle_legacy_composite at 1094 (thumbnails,
        // wiping_volumes_use_custom_matrix). The per-key loop below runs
        // handle_legacy only (set_deserialize -> Config.cpp 579), so a
        // --machine/--process/--filament file skipped them. Loaded into a
        // scratch and applied, so the loop's own handling (the Bambu build's
        // nozzle map, the substitution reporting) still runs over it.
        {
            Slic3r::DynamicPrintConfig            loaded;
            std::map<std::string, std::string>    key_values;
            std::string                           reason;
            Slic3r::ConfigSubstitutionContext     file_substitutions(
                Slic3r::ForwardCompatibilitySubstitutionRule::EnableSilent);
            try {
                loaded.load_from_json(filepath, file_substitutions, /*load_inherits_in_config=*/false,
                                      key_values, reason);
                if (reason.empty() && !loaded.empty())
                    config.apply(loaded, /*ignore_nonexistent=*/true);
            } catch (const std::exception& e) {
                if (verbose)
                    std::cerr << "Warning: config file conversions failed: " << e.what() << "\n";
            }
        }

        // Iterate through all key-value pairs
        for (auto& [key, value] : j.items()) {
            // Skip profile metadata, including identity/version fields handled
            // separately by both engines' ConfigBase::load_from_json.
            // printer_model is a setting there (Config.cpp 918-960 at
            // 5873b5f: only the version, name, url, type, setting_id,
            // filament_id, from, description, instantiation, inherits and
            // includes are kept aside), and the Bambu printer features are
            // decided from it (BambuStudio.cpp 7055-7070), so a --machine
            // file's printer_model is loaded like any other setting.
            if (key == "type" || key == "name" || key == "inherits" ||
                key == "from" || key == "setting_id" || key == "instantiation" ||
                key == "description" || key == "compatible_printers" ||
                key == "compatible_prints" || key == "include" ||
                key == "upward_compatible_machine" ||
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
                    // An array becomes the option's serialized form, as the
                    // engines' ConfigBase::load_from_json writes it
                    // (parse_str_arr, Config.cpp 863-900 and 1022-1040 at
                    // 5873b5f): a string list ';'-separated with each entry
                    // quoted and escaped (escape_strings_cstyle), a list of
                    // point groups '#'-separated, every other list
                    // ','-separated. Joined with ',' a string list became one
                    // string: a printer's seven extruder variants read as one,
                    // so the second extruder found no variant of its own
                    // (H2D nozzle_volume 130,130 for 130,145).
                    std::vector<std::string> parts;
                    for (auto& v : value) {
                        if (v.is_string()) {
                            parts.push_back(v.get<std::string>());
                        } else if (v.is_number()) {
                            parts.push_back(std::to_string(v.get<double>()));
                        }
                    }
                    const Slic3r::ConfigOptionDef* def = Slic3r::print_config_def.get(key);
                    if (def != nullptr && def->type == Slic3r::coStrings) {
                        value_str = Slic3r::escape_strings_cstyle(parts);
                    } else {
                        const char* sep = def != nullptr && def->type == Slic3r::coPointsGroups ? "#" : ",";
                        value_str = "";
                        for (size_t i = 0; i < parts.size(); i++) {
                            if (i > 0) value_str += sep;
                            value_str += parts[i];
                        }
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
// Filament roster the loaded project declares, 0 when unknown.  filament_colour
// is the authoritative roster (the driver seeds it with one entry, so a longer
// one can only come from the loaded project); the remaining per-filament
// identity vectors are taken as a floor too, because Print::apply() derives its
// own extruder count from filament_diameter (PrintApply.cpp:1504) and every
// per-region filament index it admits must be inside the arrays extended here.
// It sizes the per-filament alignment below on the Bambu build, and the 3MF
// filament-index reset (PresetBundle.cpp 4090-4105, N =
// filament_presets.size()) on the Orca build, so it is not gated.
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

#ifdef ENGINE_BAMBU
// ── BBS-only config-normalization helpers ───────────────────────────────────
// These helpers (apply_explicit_nozzle_mapping, set_default_config,
// ensure_vector_config_sizes) exist solely to coax
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
// with filament_map [1,2], and every object was then moved to physical nozzle 0
// — on the X2D the Bowden head — instead of letting the engine's automatic
// grouping (ToolOrdering.cpp:1910-1914) pick the head.
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

    // A manual mode keeps the derived map: without real AMS data the
    // automatic grouping would put every filament on the master extruder.
    // Which manual mode follows the machine, as the official CLI decides: a
    // machine with several nozzles per extruder takes Nozzle Manual, and one
    // nozzle per extruder (H2D, X2D) takes Manual, since the official CLI
    // refuses Nozzle Manual there (support_multi_nozzle, BambuStudio.cpp
    // 3417 and 6841-6846 at 5873b5f).
    bool has_multiple_nozzles_per_extruder = false;
    if (auto* counts = config.option<Slic3r::ConfigOptionInts>("extruder_max_nozzle_count", false))
        has_multiple_nozzles_per_extruder = std::any_of(counts->values.begin(), counts->values.end(),
                                                        [](int count) { return count > 1; });
    const char* manual_mode = has_multiple_nozzles_per_extruder ? "Nozzle Manual" : "Manual";
    {
        Slic3r::ConfigSubstitutionContext substitution_context(Slic3r::ForwardCompatibilitySubstitutionRule::Enable);
        config.set_deserialize("filament_map_mode", manual_mode, substitution_context);
    }
    if (verbose) {
        std::cout << "Nozzle-map derivation: filament_map=[";
        for (size_t i = 0; i < derived_map.size(); ++i)
            std::cout << (i == 0 ? "" : ",") << derived_map[i];
        std::cout << "] mode=" << manual_mode << "\n";
    }
    return true;
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

    // filament_printable is a per-filament bitmask: bit N = the filament can be
    // printed on nozzle N (0-based). The engine reads it when it groups the
    // filaments onto the nozzles (check_filament_printable_after_group,
    // ToolOrdering.cpp 87-99 at 5873b5f) and when it lists the nozzles a
    // filament cannot use (Print::get_physical_unprintable_filaments,
    // Print.cpp 3042-3057). The values are the filament presets' own (a
    // filament option, Preset.cpp 1088; written per filament by
    // PresetBundle::full_config) or the project's, and they are kept as they
    // are, as the desktop app keeps them. Only a missing or empty option takes
    // the definition default {3}, both nozzles (PrintConfig.cpp 2691-2695);
    // the final per-filament alignment then sizes it to the roster
    // (align_per_filament_config_vectors, front value duplicated as get_at()
    // reads it).
    //
    // Every entry used to be overwritten with INT_MAX here, which is the nil
    // marker of an int list (ConfigOptionIntsTempl::nil_value, Config.hpp
    // 1051), so store_bbs_3mf refused the project ("Serializing NaN",
    // Config.hpp 1109-1113) and every slice ignored the presets' nozzle limits. Its reason, "Grouping error:
    // filament1 can not be placed in the right nozzle", came from the
    // extruder-count padding appending a 0 to this array; that pass now skips
    // every per-filament key (is_per_filament_config_key), so no 0 is added.
    {
        auto* fp = config.option<Slic3r::ConfigOptionInts>("filament_printable", false);
        if (fp == nullptr || fp->values.empty())
            config.set_key_value("filament_printable",
                Slic3r::print_config_def.get("filament_printable")->default_value->clone());
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
//
// Outside the filament_ namespace: the engine's own filament_options_with_variant
// (PrintConfig.cpp 7241-7290 at 5873b5f, 8187 at 31f6803), the filament
// settings the engine fans out per filament and variant (nozzle_temperature,
// long_retractions_when_ec, ...).
bool is_per_filament_config_key(const std::string& key) {
    static const char* const kSpecialFilamentKeys[] = {
        "filament_extruder_variant", "filament_self_index",
        "filament_map", "filament_map_2",
    };

    if (key.rfind("filament_", 0) == 0) {
        for (const char* special : kSpecialFilamentKeys)
            if (key == special)
                return false;
        return true;
    }
    return Slic3r::filament_options_with_variant.count(key) > 0;
}

// True for every setting a filament preset carries: the keys above plus the
// rest of the engine's own list of filament settings, Preset::filament_options()
// (Preset.cpp 1087-1140 at 5873b5f, 1266 at 31f6803): bed temperatures, fan
// speeds, cooling, hole and counter compensation, ... The desktop writes them
// one per filament (PresetBundle::full_fff_config), and Preset::normalize sizes
// them to the filament count (Preset.cpp 457-466) less the two it skips there,
// compatible_prints and compatible_printers (lists of names). None of them has
// one value per extruder, so the extruder-count padding leaves them alone; a
// hand-written list missed most of them, and a one-filament H2D project got
// hot_plate_temp = 60,0 and fan_max_speed = 100,0. The roster alignment keeps
// to is_per_filament_config_key: the desktop's own projects carry these the
// way the official CLI slices them, and it does not extend them either.
bool is_filament_setting_key(const std::string& key) {
    if (is_per_filament_config_key(key))
        return true;
    static const std::unordered_set<std::string> engine_filament_keys = [] {
        std::unordered_set<std::string> keys(Slic3r::Preset::filament_options().begin(),
                                             Slic3r::Preset::filament_options().end());
        keys.erase("compatible_prints");
        keys.erase("compatible_printers");
        for (const char* special : {"filament_extruder_variant", "filament_self_index", "filament_map", "filament_map_2"})
            keys.erase(special);
        return keys;
    }();
    return engine_filament_keys.count(key) > 0;
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

/// The archive opens as a ZIP and `member` decompresses with a matching CRC.
/// The bytes stream to a discarding sink, so any size is checked without
/// holding it in memory.
static bool zip_member_readable(const std::string& archive, const std::string& member) {
    mz_zip_archive zip;
    mz_zip_zero_struct(&zip);
    if (!Slic3r::open_zip_reader(&zip, archive))
        return false;
    bool readable = false;
    const mz_uint count = mz_zip_reader_get_num_files(&zip);
    for (mz_uint file_idx = 0; file_idx < count; ++file_idx) {
        mz_zip_archive_file_stat stat;
        if (!mz_zip_reader_file_stat(&zip, file_idx, &stat)) continue;
        std::string name(stat.m_filename);
        std::replace(name.begin(), name.end(), '\\', '/');
        if (!boost::algorithm::iequals(name, member)) continue;
        auto discard = [](void*, mz_uint64, const void*, size_t n) -> size_t { return n; };
        readable = mz_zip_reader_extract_to_callback(&zip, file_idx, discard, nullptr, 0);
        break;
    }
    Slic3r::close_zip_reader(&zip);
    return readable;
}

/// Number of plates a Bambu/Orca project declares (one <plate> element each in
/// Metadata/model_settings.config, which is what load_bbs_3mf builds its
/// PlateData list from); 0 when the file declares none.
static int count_3mf_plates(const std::string& path) {
    std::string settings;
    if (!read_zip_member(path, "Metadata/model_settings.config", settings))
        return 0;
    // Every <plate> start tag, by XML name rules: the name ends at
    // whitespace, '>' or '/', so `<plate >` and a tag with attributes count
    // and `<plates>` does not; comments are skipped.
    int plates = 0;
    for (size_t pos = 0; (pos = settings.find('<', pos)) != std::string::npos; ++pos) {
        if (settings.compare(pos, 4, "<!--") == 0) {
            const size_t end = settings.find("-->", pos + 4);
            if (end == std::string::npos) break;
            pos = end + 2;
            continue;
        }
        if (settings.compare(pos, 6, "<plate") != 0 || pos + 6 >= settings.size()) continue;
        const char next = settings[pos + 6];
        if (next == '>' || next == '/' || next == ' ' || next == '\t' || next == '\r' || next == '\n')
            ++plates;
    }
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
/// than the version it is compared against.
static bool file_newer_than_version(const Slic3r::Semver& file_version, const std::string& version) {
    const auto engine = Slic3r::Semver::parse(version);
    if (!engine) return false;
    return engine->maj() < file_version.maj() ||
           (engine->maj() == file_version.maj() && engine->min() < file_version.min());
}

static bool file_newer_than_engine(const Slic3r::Semver& file_version) {
    return file_newer_than_version(file_version, engine_version_text());
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
            // The native path: on Windows a narrowed path loses characters
            // outside the ANSI code page (as stage_file_copy's streams).
            boost::filesystem::ifstream f(entry.path());
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

/// The printer model of a printer system preset, read from the profiles tree:
/// <vendor>/machine/<name>.json, following "inherits" to the file that states
/// printer_model. Empty when the tree has no such preset.
static std::string preset_printer_model(const boost::filesystem::path& profiles_dir, const std::string& name) {
    if (profiles_dir.empty() || name.empty()) return {};
    try {
        for (auto& vendor : boost::filesystem::directory_iterator(profiles_dir)) {
            if (!boost::filesystem::is_directory(vendor.path())) continue;
            std::string current = name;
            for (int depth = 0; depth < 16 && !current.empty(); ++depth) {
                const boost::filesystem::path file = vendor.path() / "machine" / (current + ".json");
                if (!boost::filesystem::exists(file)) break;
                boost::filesystem::ifstream f(file);
                json preset;
                try { preset = json::parse(f); } catch (...) { break; }
                if (preset.contains("printer_model") && preset["printer_model"].is_string() &&
                    !preset["printer_model"].get<std::string>().empty())
                    return preset["printer_model"].get<std::string>();
                current = preset.contains("inherits") && preset["inherits"].is_string()
                              ? preset["inherits"].get<std::string>() : std::string();
            }
        }
    } catch (...) {
    }
    return {};
}

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
/// that has it: a file for one engine sent to the other is refused, never
/// substituted, and the refusal names the engine that fits.
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
// file's one nozzle diameter, in mm. A percentage that is not one number,
// or a project with more than one nozzle diameter, is refused naming the
// setting, never guessed.

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
    // Only a failed data copy falls back. copy_file creates the destination
    // with O_EXCL first and leaves it behind when copy_file_range then fails
    // (Boost.Filesystem 1.84 src/operations.cpp 2847-2934): that file is ours
    // to fill. Any other error, an existing destination (EEXIST) above all,
    // keeps copy_file's refusal to overwrite.
    namespace errc = boost::system::errc;
    const bool data_copy_failed = copy_ec == errc::cross_device_link || copy_ec == errc::not_supported ||
                                  copy_ec == errc::operation_not_supported || copy_ec == errc::invalid_argument ||
                                  copy_ec == errc::function_not_supported;
    if (!data_copy_failed)
        throw std::runtime_error("cannot copy " + src.string() + " to " + dst.string() + ": " + why);
    // The file copy_file created, never a link put in its place.
    boost::system::error_code status_ec;
    if (boost::filesystem::symlink_status(dst, status_ec).type() != boost::filesystem::regular_file)
        throw std::runtime_error("cannot write " + dst.string() + " (copy_file: " + why + ")");
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

// ── Per-plate outcome ───────────────────────────────────────────────────
// The command line itself is CliOptions (cli_options.hpp), filled by
// cli_command_line.cpp.

// The settings the named presets resolve to, computed once per run (loading a
// profiles tree takes seconds) and applied to every plate.
static std::unique_ptr<Slic3r::DynamicPrintConfig> g_preset_config;

// --load-assemble-list: the list's plates and objects, built once per run
// before the plate loop (as BambuStudio.cpp 2165-2188 builds them before
// its own); each plate's slice copies its objects out.
static std::unique_ptr<slicer_cli::AssembleList> g_assemble;

/// The --load-assemble-list file, or empty (an empty value is no list, as
/// the official reads it: BambuStudio.cpp 1795-1799).
static std::string assemble_list_file(const CliOptions& o) {
    return o.given_flag("load_assemble_list") ? o.cli.opt_string("load_assemble_list") : std::string();
}

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
    // Bambu build: routing the slice derived from a supplied filament_nozzle_map
    // (apply_explicit_nozzle_mapping); empty when nothing was derived.
    std::vector<int>                            derived_filament_map;
    std::string                                 derived_filament_map_mode;  // "Manual" or "Nozzle Manual"
    // The run steps (cli_run_steps.cpp): --repetitions copies, the objects
    // --skip-objects left out (plate_N.json skipped_objects), --no-check, and
    // a run of model actions only (no G-code).
    int                                         duplicate_count = 0;
    // --downward-check: the printers checked and those this plate does not
    // fit, and whether it prints by object (BambuStudio.cpp 4785-4914).
    bool                                        downward_checked = false;
    std::vector<std::string>                    downward_printers;
    std::vector<std::string>                    downward_failed;
    bool                                        sequence_plate = false;
    std::vector<int>                            skipped_objects;
    bool                                        no_check = false;
    bool                                        actions_only = false;
    bool                                        run_step_failed = false;  // a model action failed: reported as plate 0
    // --slice 0 on several plates: the check pass, which stops before the
    // slice (run_slice_mode).
    bool                                        pre_check = false;
    // --slice: the pass that loads the run's one model and settings and runs
    // the steps the official CLI runs once before its plate loop (the
    // transforms, the arrange, the actions before --slice; RunState), before
    // any plate is checked or sliced.
    bool                                        prepare_pass = false;
    // --load-settings and friends: what the merge decided, for the export.
    std::shared_ptr<slicer_cli::SettingsMerge>  settings_merge;
    // --export-3mf: the plate's Metadata/plate_N.json (first-layer boxes).
    std::shared_ptr<Slic3r::PlateBBoxData>      plate_bbox;
};

/// --skip-useless-pick (BambuStudio only), read where the export fills the plate.
static bool g_skip_useless_pick = false;

/// The printer model id the desktop app stores per plate for presets picked
/// by name (Preset::get_printer_type, Plater.cpp 22261 at 5873b5f).
static std::string g_preset_printer_model_id;



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
        {CLI_FILAMENT_CAN_NOT_MAP, "Some filaments cannot be mapped to correct extruders for multi-extruder Printer."},
        {CLI_ONLY_ONE_TPU_SUPPORTED, "Not support printing 2 or more TPU filaments."},
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
    // --downward-check and the settings merge: the official keys, written
    // only when they have entries (BambuStudio.cpp 488-493; OrcaSlicer.cpp
    // 429-430). A printer is compatible when every checked plate fits it.
    {
        std::vector<std::string> checked;
        std::set<std::string> failed;
        bool any_checked = false, sequence_plates = false;
        for (const PlateOutcome& p : outcomes) {
            if (!p.downward_checked) continue;
            if (!any_checked) checked = p.downward_printers;
            any_checked = true;
            failed.insert(p.downward_failed.begin(), p.downward_failed.end());
            sequence_plates |= p.sequence_plate;
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
        // BambuStudio.cpp 4913-4914.
        if (any_checked && sequence_plates)
            j["upward_compatibility_taint"] = std::vector<std::string>{"PrintSequenceByObject"};
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
        plate["warnings"]   = p.warnings;
        plate["gcode_file"] = p.gcode_path;
        plate["unknown_settings"] = p.unknown_settings;
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


// ── --arrange 1: the official per-plate arrange ─────────────────────────
// Port of the official CLI's "arrange this plate only" branch (BambuStudio.cpp
// 5730-5952 at 5873b5f; OrcaSlicer.cpp 4990-5210 at 31f6803). The official
// keeps its plates in slic3r/GUI/PartPlate.cpp, which slicer-cli does not
// link; the few plate steps it needs are ported here, each with its source.
//
// Coordinates: the caller has already moved this plate's objects by minus the
// plate's grid origin (plate_grid_origin, the multi-plate translation in
// slice_one_plate), and the print origin stays at 0. Objects, the bed
// (printable_area), the exclusion boxes (bed_exclude_area) and the prime
// tower (wipe_tower_x/y, per plate) are therefore all in the same plate-local
// bed frame; the official reads the boxes from plate 0, whose origin is
// (0, 0), and the tower from the plate's own wipe_tower_x/y entry, both in
// that frame. Nothing is translated here.

/// A print setting from `config`, or the engine's default when the config
/// does not hold it (FullPrintConfig::defaults(), the values a fresh Print
/// starts with).
static const Slic3r::ConfigOption* arrange_opt(const Slic3r::DynamicPrintConfig& config, const std::string& key) {
    if (const Slic3r::ConfigOption* opt = config.option(key))
        return opt;
    return Slic3r::FullPrintConfig::defaults().option(key);
}

/// The printer's bed exclusion boxes, as PartPlate::calc_bounding_boxes makes
/// them from bed_exclude_area: every 4 points are one box (Bambu
/// slic3r/GUI/PartPlate.cpp 375-392; Orca PartPlate.cpp 418-435). Plate 0's
/// position is (0, 0) (PartPlate::set_shape adds it, Bambu PartPlate.cpp
/// 3185-3189), so the boxes are plate-local bed coordinates.
static std::vector<Slic3r::BoundingBoxf> arrange_exclude_boxes(const Slic3r::DynamicPrintConfig& config) {
    std::vector<Slic3r::BoundingBoxf> boxes;
    const auto* area = config.option<Slic3r::ConfigOptionPoints>("bed_exclude_area");
    if (area == nullptr)
        return boxes;
    Slic3r::BoundingBoxf box;
    for (size_t index = 0; index < area->values.size(); ++index) {
        if (index % 4 == 0)
            box = Slic3r::BoundingBoxf();
        box.merge(area->values[index]);
        if (index % 4 == 3)
            boxes.push_back(box);
    }
    return boxes;
}

/// PartPlateList::preprocess_exclude_areas (Bambu PartPlate.cpp 5906-5972;
/// Orca PartPlate.cpp 5363-5429, the same code): the wrapping-detection area
/// when enable_wrapping_detection is on (BambuStudio.cpp 4175-4176;
/// OrcaSlicer.cpp 3588), then one fixed virtual item per exclusion box. The
/// official adds a copy for every bed up to the sliced plate (num_plates,
/// default 16); a one-plate arrange places items on bed 0 only, so it asks
/// for bed 0's copy alone, and the project-wide arrange for the default 16.
static void arrange_add_exclude_areas(Slic3r::arrangement::ArrangePolygons& out,
                                      const Slic3r::DynamicPrintConfig& config, double inflation,
                                      int num_beds = 1) {
    using namespace Slic3r;
    const auto* wrapping = config.option<ConfigOptionBool>("enable_wrapping_detection");
    if (wrapping != nullptr && wrapping->value) {
        const auto* area = config.option<ConfigOptionPoints>("wrapping_exclude_area");
        if (area != nullptr && !area->values.empty()) {
            Slic3r::Polygon ap{};
            for (const Vec2d& p : area->values)
                ap.append({scale_(p(0)), scale_(p(1))});
            for (int j = 0; j < num_beds; ++j) {
                arrangement::ArrangePolygon ret;
                ret.poly.contour   = ap;
                ret.translation    = Vec2crd(0, 0);
                ret.rotation       = 0.0f;
                ret.is_virt_object = true;
                ret.bed_idx        = j;
                ret.height         = 1;
                ret.name           = "WrappingRegion";
                ret.inflation      = inflation;
                out.emplace_back(std::move(ret));
            }
        }
    }
    const std::vector<BoundingBoxf> boxes = arrange_exclude_boxes(config);
    for (size_t index = 0; index < boxes.size(); ++index) {
        const BoundingBoxf& box = boxes[index];
        Slic3r::Polygon ap({
            {scaled(box.min.x()), scaled(box.min.y())},
            {scaled(box.max.x()), scaled(box.min.y())},
            {scaled(box.max.x()), scaled(box.max.y())},
            {scaled(box.min.x()), scaled(box.max.y())}
        });
        for (int j = 0; j < num_beds; ++j) {
            arrangement::ArrangePolygon ret;
            ret.poly.contour   = ap;
            ret.translation    = Vec2crd(0, 0);
            ret.rotation       = 0.0f;
            ret.is_virt_object = true;
            ret.bed_idx        = j;
            ret.height         = 1;
            ret.name           = "ExcludedRegion" + std::to_string(index);
            ret.inflation      = inflation;
            out.emplace_back(std::move(ret));
        }
    }
}

/// The filaments this plate's objects use, as PartPlate::get_extruders_under_cli
/// (true, config) lists them (Bambu PartPlate.cpp 1275-1428; Orca PartPlate.cpp
/// 1625-1739): each printable instance's volumes, its layer ranges, its support
/// filaments when it has support, the plate's tool-change G-codes, sorted and
/// unique. The model holds only this plate's objects (load_bbs_3mf with the
/// plate id), so every instance is the plate's. Bambu's per-feature block
/// (separate_filaments_for_features) is not ported: no such setting is defined
/// in src/libslic3r at 5873b5f, so the official's block never runs.
static std::vector<int> arrange_plate_extruders(const Slic3r::Model& model,
                                                const Slic3r::DynamicPrintConfig& full_config, int plate_index) {
    using namespace Slic3r;
    std::vector<int> plate_extruders;
    const int glb_support_intf_extr = arrange_opt(full_config, "support_interface_filament")->getInt();
    const int glb_support_extr      = arrange_opt(full_config, "support_filament")->getInt();
#ifdef ENGINE_ORCA
    const int glb_wall_extr          = arrange_opt(full_config, "wall_filament")->getInt();
    const int glb_sparse_infill_extr = arrange_opt(full_config, "sparse_infill_filament")->getInt();
    const int glb_solid_infill_extr  = arrange_opt(full_config, "solid_infill_filament")->getInt();
#endif
    bool glb_support = arrange_opt(full_config, "enable_support")->getBool();
    glb_support |= arrange_opt(full_config, "raft_layers")->getInt() > 0;

    for (const ModelObject* object : model.objects) {
        for (const ModelInstance* instance : object->instances) {
            if (!instance->printable)
                continue;
            for (const ModelVolume* mv : object->volumes) {
                const std::vector<int> volume_extruders = mv->get_extruders();
                plate_extruders.insert(plate_extruders.end(), volume_extruders.begin(), volume_extruders.end());
            }
            // layer range
            for (const auto& layer_range : object->layer_config_ranges) {
                if (layer_range.second.has("extruder")) {
                    if (auto id = layer_range.second.option("extruder")->getInt(); id > 0)
                        plate_extruders.push_back(id);
                }
            }
            bool obj_support = false;
            const ConfigOption* obj_support_opt = object->config.option("enable_support");
            const ConfigOption* obj_raft_opt    = object->config.option("raft_layers");
            if (obj_support_opt != nullptr || obj_raft_opt != nullptr) {
                if (obj_support_opt != nullptr)
                    obj_support = obj_support_opt->getBool();
                if (obj_raft_opt != nullptr)
                    obj_support |= obj_raft_opt->getInt() > 0;
            } else
                obj_support = glb_support;
            if (!obj_support)
                continue;

            int obj_support_intf_extr = 0;
            if (const ConfigOption* opt = object->config.option("support_interface_filament"))
                obj_support_intf_extr = opt->getInt();
            if (obj_support_intf_extr != 0)
                plate_extruders.push_back(obj_support_intf_extr);
            else if (glb_support_intf_extr != 0)
                plate_extruders.push_back(glb_support_intf_extr);

            int obj_support_extr = 0;
            if (const ConfigOption* opt = object->config.option("support_filament"))
                obj_support_extr = opt->getInt();
            if (obj_support_extr != 0)
                plate_extruders.push_back(obj_support_extr);
            else if (glb_support_extr != 0)
                plate_extruders.push_back(glb_support_extr);
#ifdef ENGINE_ORCA
            // Orca also lists the wall / infill filaments (PartPlate.cpp 1703-1728).
            int obj_wall_extr = 1;
            if (const ConfigOption* opt = object->config.option("wall_filament"))
                obj_wall_extr = opt->getInt();
            if (obj_wall_extr != 1)
                plate_extruders.push_back(obj_wall_extr);
            else if (glb_wall_extr != 1)
                plate_extruders.push_back(glb_wall_extr);

            int obj_sparse_infill_extr = 1;
            if (const ConfigOption* opt = object->config.option("sparse_infill_filament"))
                obj_sparse_infill_extr = opt->getInt();
            if (obj_sparse_infill_extr != 1)
                plate_extruders.push_back(obj_sparse_infill_extr);
            else if (glb_sparse_infill_extr != 1)
                plate_extruders.push_back(glb_sparse_infill_extr);

            int obj_solid_infill_extr = 1;
            if (const ConfigOption* opt = object->config.option("solid_infill_filament"))
                obj_solid_infill_extr = opt->getInt();
            if (obj_solid_infill_extr != 1)
                plate_extruders.push_back(obj_solid_infill_extr);
            else if (glb_solid_infill_extr != 1)
                plate_extruders.push_back(glb_solid_infill_extr);
#endif
        }
    }

    // conside_custom_gcode: the plate's tool changes.
    if (const auto* color_option = dynamic_cast<const ConfigOptionStrings*>(full_config.option("filament_colour"))) {
        const int nums_extruders = (int)color_option->values.size();
        auto it = model.plates_custom_gcodes.find(plate_index);
        if (it != model.plates_custom_gcodes.end()) {
            for (const auto& item : it->second.gcodes)
                if (item.type == CustomGCode::Type::ToolChange && item.extruder <= nums_extruders)
                    plate_extruders.push_back(item.extruder);
        }
    }

    std::sort(plate_extruders.begin(), plate_extruders.end());
    auto it_end = std::unique(plate_extruders.begin(), plate_extruders.end());
    plate_extruders.resize(std::distance(plate_extruders.begin(), it_end));

#ifdef ENGINE_BAMBU
    // Mixed filament slots count as their physical parts (PartPlate.cpp 1414-1426).
    if (auto* is_mixed_opt = full_config.option<ConfigOptionBools>("filament_is_mixed")) {
        if (auto* comp_strs_opt = full_config.option<ConfigOptionStrings>("filament_mixed_components")) {
            if (has_any_mixed_filament(is_mixed_opt->values)) {
                std::vector<unsigned int> ext_0based;
                for (int e : plate_extruders)
                    if (e >= 1) ext_0based.push_back((unsigned int)(e - 1));
                auto expanded = expand_mixed_filaments(ext_0based, is_mixed_opt->values, comp_strs_opt->values);
                plate_extruders.clear();
                for (unsigned int e : expanded)
                    plate_extruders.push_back((int)(e + 1));
            }
        }
    }
#endif
    return plate_extruders;
}

/// PartPlate::contain_instance_totally(obj, 0) before the arrange: the
/// instance is on the plate and PartPlate::check_outside finds it inside the
/// plate box and clear of every exclusion box (Bambu PartPlate.cpp 2597-2648;
/// Orca PartPlate.cpp 2529+). The official tests the instance's convex hull
/// against each exclusion box; this uses the hull's bounding box.
static bool arrange_instance_totally_inside(const Slic3r::ModelObject* object, const Slic3r::BoundingBoxf3& plate_box_in,
                                            const std::vector<Slic3r::BoundingBoxf>& exclude_boxes) {
    using namespace Slic3r;
    if (object->instances.empty())
        return false;
    const BoundingBoxf3 instance_box = object->instance_convex_hull_bounding_box(size_t(0));
    BoundingBoxf3 plate_box = plate_box_in;
    if (instance_box.max.z() > plate_box.min.z())
        plate_box.min.z() += instance_box.min.z(); // not considering outsize if sinking
    if (!plate_box.contains(instance_box))
        return false;
    for (const BoundingBoxf& box : exclude_boxes)
        if (box.min.x() < instance_box.max.x() && instance_box.min.x() < box.max.x() &&
            box.min.y() < instance_box.max.y() && instance_box.min.y() < box.max.y())
            return false;
    return true;
}

/// PartPlate::estimate_wipe_tower_size (Bambu PartPlate.cpp 2159-2233; Orca
/// PartPlate.cpp 2099-2165). The two engines differ in the tallest-object
/// measure (Bambu: every instance's top; Orca: the object's exact height) and
/// in the rib-wall settings (Bambu prime_tower_rib_wall / prime_tower_rib_width
/// / prime_tower_extra_rib_length; Orca wipe_tower_wall_type == Rib /
/// wipe_tower_rib_width / wipe_tower_extra_rib_length). The official reads the
/// filament-change length, filament diameter and extra rib length from the
/// plate's Print, which at arrange time is a fresh `new Print()` (PartPlateList
/// init, Bambu PartPlate.cpp 4024) holding the engine defaults; so do we.
static Slic3r::Vec3d arrange_estimate_wipe_tower_size(const Slic3r::Model& model, const Slic3r::DynamicPrintConfig& config,
                                                      const double w, const double wipe_volume, int extruder_count,
                                                      int plate_extruder_size, bool enable_wrapping_detection,
                                                      const Slic3r::BoundingBoxf3& plate_box,
                                                      const std::vector<Slic3r::BoundingBoxf>& exclude_boxes,
                                                      const Slic3r::Model* global_model = nullptr) {
    using namespace Slic3r;
    Vec3d wipe_tower_size;
    double layer_height = 0.08f; // hard code layer height
    double max_height = 0.f;
    wipe_tower_size.setZero();

    if (const ConfigOption* layer_height_opt = config.option("layer_height"))
        layer_height = layer_height_opt->getFloat();

    // The official counts the plate's filaments again for an empty count
    // (get_extruders(true)); the caller's count already is that list.
    if (plate_extruder_size == 0)
        return wipe_tower_size;

    // use_global_objects (the assemble-list arrange): every object of the
    // run, on any plate, without the inside check.
    for (const ModelObject* mo : (global_model ? global_model->objects : model.objects)) {
        if (!global_model && !arrange_instance_totally_inside(mo, plate_box, exclude_boxes))
            continue;
#ifdef ENGINE_ORCA
        BoundingBoxf3 bbox = mo->bounding_box_exact();
        max_height = std::max(bbox.size().z(), max_height);
#else
        for (size_t i = 0; i < mo->instances.size(); ++i) {
            BoundingBoxf3 bbox = mo->instance_bounding_box(i);
            max_height         = std::max(bbox.max.z(), max_height);
        }
#endif
    }
    wipe_tower_size(2) = max_height;
    auto timelapse_type  = config.option<ConfigOptionEnum<TimelapseType>>("timelapse_type");
    bool need_wipe_tower = (timelapse_type ? (timelapse_type->value == TimelapseType::tlSmooth) : false) | enable_wrapping_detection;
    double extra_spacing = arrange_opt(config, "prime_tower_infill_gap")->getFloat() / 100.;
    const FullPrintConfig& print_defaults = FullPrintConfig::defaults();
#ifdef ENGINE_ORCA
    const auto* use_rib_wall_opt = config.option<ConfigOptionEnum<WipeTowerWallType>>("wipe_tower_wall_type");
    bool use_rib_wall = use_rib_wall_opt ? use_rib_wall_opt->value == WipeTowerWallType::wtwRib : false;
    double rib_width = arrange_opt(config, "wipe_tower_rib_width")->getFloat();
    const double extra_rib_length = print_defaults.wipe_tower_extra_rib_length.value;
#else
    const ConfigOptionBool* use_rib_wall_opt = config.option<ConfigOptionBool>("prime_tower_rib_wall");
    bool use_rib_wall = use_rib_wall_opt ? use_rib_wall_opt->value : true;
    double rib_width = arrange_opt(config, "prime_tower_rib_width")->getFloat();
    const double extra_rib_length = print_defaults.prime_tower_extra_rib_length.value;
#endif
    double depth;
    double filament_change_volume = 0.;
    {
        const std::vector<double>& filament_change_lengths = print_defaults.filament_change_length.values;
        double length = filament_change_lengths.empty() ? 0 : *std::max_element(filament_change_lengths.begin(), filament_change_lengths.end());
        double diameter = 1.75;
        const std::vector<double>& diameters = print_defaults.filament_diameter.values;
        diameter = diameters.empty() ? diameter : *std::max_element(diameters.begin(), diameters.end());
        filament_change_volume = length * PI * diameter * diameter / 4.;
    }
    double volume = wipe_volume * (extruder_count == 2 ? plate_extruder_size : (plate_extruder_size - 1));
    if (extruder_count == 2) volume += filament_change_volume * (int) (plate_extruder_size / 2);
    if (use_rib_wall) {
        depth = std::sqrt(volume / layer_height * extra_spacing);
        if (need_wipe_tower || plate_extruder_size > 1) {
            float min_wipe_tower_depth = WipeTower::get_limit_depth_by_height(max_height);
            double volume_depth        = depth;
            depth = std::max((double) min_wipe_tower_depth, depth);
            rib_width = std::min(rib_width, depth / 2);
            depth = rib_width / std::sqrt(2) + std::max(depth + extra_rib_length, volume_depth);
            wipe_tower_size(0) = wipe_tower_size(1) = depth;
        }
    } else {
        depth = volume / (layer_height * w) * extra_spacing;
        if (need_wipe_tower || depth > EPSILON) {
            float min_wipe_tower_depth = WipeTower::get_limit_depth_by_height(max_height);
            depth = std::max((double) min_wipe_tower_depth, depth);
        }
        wipe_tower_size(0) = w;
        wipe_tower_size(1) = depth;
    }
    return wipe_tower_size;
}

/// The official arrange switches given on the command line (cli_options.hpp).
struct ArrangeSwitches {
    bool enable_timelapse            = false;   // --enable-timelapse
    bool allow_rotations             = false;   // --allow-rotations (see below)
    bool allow_multicolor_oneplate   = true;    // --allow-multicolor-oneplate
    bool avoid_extrusion_cali_region = false;   // --avoid-extrusion-cali-region
};
static ArrangeSwitches g_arrange_switches;

/// --downward-check's result: the official checks every plate once per run
/// (BambuStudio.cpp 4645-4658, 4779-4790), so a --slice 0 run checks on its
/// first pass and every later pass reports the same.
struct DownwardRun {
    bool done = false;
    std::vector<std::string> printers, failed;
    bool sequence_plate = false;
};
static DownwardRun g_downward_run;

/// The box of the plate's objects with its prime tower, as the lambda
/// check_plate_wipe_tower measures it (BambuStudio.cpp 4418-4495; OrcaSlicer.cpp
/// 3819-3884), for --downward-check and the move to a new bed. The model holds
/// only this plate's objects, at the plate's own origin.
static Slic3r::BoundingBoxf3 downward_plate_bbox(const Slic3r::Model& model, const Slic3r::DynamicPrintConfig& config,
                                                 int plate_index, bool is_sequence, bool* has_wipe_tower = nullptr) {
    using namespace Slic3r;
    if (has_wipe_tower)
        *has_wipe_tower = false;
    BoundingBoxf3 obj_bbox;
    for (const ModelObject* object : model.objects)
        for (size_t i = 0; i < object->instances.size(); ++i)
            obj_bbox.merge(object->instance_bounding_box(i));
    if (!config.has("wipe_tower_x"))
        return obj_bbox;
    const auto* enable_tower = config.option<ConfigOptionBool>("enable_prime_tower");
    if (!enable_tower || !enable_tower->value)
        return obj_bbox;
    int valid_count = 0;
    for (const ModelObject* object : model.objects)
        for (const ModelInstance* inst : object->instances)
            valid_count += inst->printable ? 1 : 0;
    if (valid_count <= 0 || (is_sequence && valid_count > 1))
        return obj_bbox;
    const auto* timelapse_type_opt = config.option("timelapse_type");
    const bool is_smooth_timelapse = g_arrange_switches.enable_timelapse && timelapse_type_opt &&
                                     timelapse_type_opt->getInt() == TimelapseType::tlSmooth;
    const unsigned int filaments_cnt = (unsigned int)arrange_plate_extruders(model, config, plate_index).size();
    if (filaments_cnt <= 1 && !is_smooth_timelapse)
        return obj_bbox;

    const float wipe_x = (float)dynamic_cast<const ConfigOptionFloats*>(config.option("wipe_tower_x"))->get_at(plate_index);
    const float wipe_y = (float)dynamic_cast<const ConfigOptionFloats*>(arrange_opt(config, "wipe_tower_y"))->get_at(plate_index);
    const double width = arrange_opt(config, "prime_tower_width")->getFloat();
    float brim_width = (float)arrange_opt(config, "prime_tower_brim_width")->getFloat();
    if (brim_width < 0)
        brim_width = WipeTower::get_auto_brim_by_height((float)obj_bbox.max.z());
#ifdef ENGINE_ORCA
    const double wipe_volume = arrange_opt(config, "prime_volume")->getFloat();
#else
    const std::vector<double> volumes = dynamic_cast<const ConfigOptionFloats*>(arrange_opt(config, "filament_prime_volume"))->values;
    const double wipe_volume = volumes.empty() ? 0. : *std::max_element(volumes.begin(), volumes.end());
#endif
    const auto* wrapping_detection = config.option<ConfigOptionBool>("enable_wrapping_detection");
    const bool enable_wrapping = wrapping_detection != nullptr && wrapping_detection->value;
    int extruder_count = 1;
    if (const auto* nozzles = dynamic_cast<const ConfigOptionVectorBase*>(config.option("nozzle_diameter")))
        extruder_count = std::max(1, (int)nozzles->size());
    BoundingBoxf bed;
    if (const auto* area = config.option<ConfigOptionPoints>("printable_area"))
        for (const Vec2d& pt : area->values) bed.merge(pt);
    const double bed_height = config.has("printable_height") ? config.opt_float("printable_height") : 0.;
    const BoundingBoxf3 plate_box(Vec3d(bed.min.x(), bed.min.y(), 0.), Vec3d(bed.max.x(), bed.max.y(), bed_height));
    const Vec3d tower = arrange_estimate_wipe_tower_size(model, config, width, wipe_volume, extruder_count, (int)filaments_cnt,
                                                         enable_wrapping, plate_box, arrange_exclude_boxes(config));
#ifdef ENGINE_ORCA
    // OrcaSlicer.cpp 3876-3879: the brim on the sides only.
    obj_bbox.merge(Vec3d(wipe_x - brim_width, wipe_y, 0.f));
    obj_bbox.merge(Vec3d(wipe_x + tower(0) + brim_width, wipe_y + tower(1), 0.f));
#else
    obj_bbox.merge(Vec3d(wipe_x - brim_width, wipe_y - brim_width, 0.f));
    obj_bbox.merge(Vec3d(wipe_x + tower(0) + brim_width, wipe_y + tower(1) + brim_width, 0.f));
#endif
    if (has_wipe_tower)
        *has_wipe_tower = true;
    return obj_bbox;
}

static Slic3r::Vec3d downward_plate_size(const Slic3r::Model& model, const Slic3r::DynamicPrintConfig& config,
                                         int plate_index, bool is_sequence) {
    return downward_plate_bbox(model, config, plate_index, is_sequence).size();
}

/// shrink_to_new_bed alone (BambuStudio.cpp 4280-4310; OrcaSlicer.cpp
/// 3693-3715): 0 the same bed (or no bed known), 1 a larger one, 2 a smaller
/// one or another exclusion area.
static int new_bed_shrink(const slicer_cli::ProjectFacts& facts, const Slic3r::DynamicPrintConfig& config) {
    using namespace Slic3r;
    if (!facts.is_bbl_3mf)
        return 0;
    const auto* area = config.option<ConfigOptionPoints>("printable_area");
    if (!area || area->values.size() < 4)
        return 0;
    const int current_width  = (int)(area->values[2].x() - area->values[0].x());
    const int current_depth  = (int)(area->values[2].y() - area->values[0].y());
    const int current_height = config.has("printable_height") ? (int)config.opt_float("printable_height") : 0;
    const int old_width  = facts.old_printable_width > 0 ? facts.old_printable_width : current_width;
    const int old_depth  = facts.old_printable_depth > 0 ? facts.old_printable_depth : current_depth;
    const int old_height = facts.old_printable_height > 0 ? facts.old_printable_height : current_height;
    std::vector<Vec2d> current_exclude_area;
    if (const auto* exclude = config.option<ConfigOptionPoints>("bed_exclude_area"))
        current_exclude_area = exclude->values;
    int shrink_to_new_bed = 0;
    if (old_width > 0 && old_depth > 0 && old_height > 0) {
        if (old_width > current_width || old_depth > current_depth || old_height > current_height)
            shrink_to_new_bed = 2;
        else if (old_width < current_width || old_depth < current_depth)
            shrink_to_new_bed = 1;
        else if (!current_exclude_area.empty() && current_exclude_area != facts.old_exclude_area)
            shrink_to_new_bed = 2;
    }
    return shrink_to_new_bed;
}

/// A project moved onto a printer with another bed: translate_models
/// (BambuStudio.cpp 4497-4644, called at 4659; OrcaSlicer.cpp 3885-3980), with
/// shrink_to_new_bed from the file's bed against this printer's
/// (BambuStudio.cpp 4280-4310; OrcaSlicer.cpp 3693-3715). 1: the new bed is
/// larger, so the plate keeps its place around the bed centre; 2: it is smaller
/// (or only its exclusion area differs), so the objects and tower are centred
/// on the bed, or on the shared area of a two-nozzle printer. BambuStudio moves
/// only on a printer change through --load-settings, and on such a change with
/// the same bed it centres a plate whose objects and tower fit the shared area
/// there; OrcaSlicer moves whenever the beds differ. The model holds this
/// plate's objects at the plate's own origin, so each move is the official one
/// with both origins taken out, and the tower moves with them. Returns
/// shrink_to_new_bed; `moved` is the offset applied.
static int move_to_new_bed(const slicer_cli::ProjectFacts& facts, bool machine_switch, Slic3r::Model& model,
                           Slic3r::DynamicPrintConfig& config, int plate_index, bool is_sequence, Slic3r::Vec3d& moved) {
    using namespace Slic3r;
    moved = Vec3d::Zero();
    if (!facts.is_bbl_3mf)
        return 0;
    const auto* area = config.option<ConfigOptionPoints>("printable_area");
    if (!area || area->values.size() < 4)
        return 0;
    const int current_width  = (int)(area->values[2].x() - area->values[0].x());
    const int current_depth  = (int)(area->values[2].y() - area->values[0].y());
    const int old_width  = facts.old_printable_width > 0 ? facts.old_printable_width : current_width;
    const int old_depth  = facts.old_printable_depth > 0 ? facts.old_printable_depth : current_depth;
    std::vector<Vec2d> current_exclude_area;
    if (const auto* exclude = config.option<ConfigOptionPoints>("bed_exclude_area"))
        current_exclude_area = exclude->values;
    const int shrink_to_new_bed = new_bed_shrink(facts, config);
#ifdef ENGINE_ORCA
    (void)machine_switch;
    if (shrink_to_new_bed == 0)
        return 0;
#else
    if (!machine_switch)
        return shrink_to_new_bed;
#endif
    // The area every nozzle reaches (BambuStudio.cpp 4250-4277; OrcaSlicer.cpp
    // 3662-3689): an integer box, as the official's.
    int shared_center_x = 0, shared_center_y = 0, shared_width = 0, shared_depth = 0;
    {
        const auto* areas   = config.option<ConfigOptionPointsGroups>("extruder_printable_area");
        const auto* heights = config.option<ConfigOptionFloatsNullable>("extruder_printable_height");
        if (areas && heights && !areas->values.empty() && areas->values.size() == heights->values.size()) {
            BoundingBox current_bbox({0, 0}, {current_width, current_depth});
            for (const auto& shape : areas->values) {
                BoundingBox temp_bbox;
                for (const Vec2d& pt : shape)
                    temp_bbox.merge({pt.x(), pt.y()});
                if (current_bbox.min.x() < temp_bbox.min.x()) current_bbox.min.x() = temp_bbox.min.x();
                if (current_bbox.min.y() < temp_bbox.min.y()) current_bbox.min.y() = temp_bbox.min.y();
                if (current_bbox.max.x() > temp_bbox.max.x()) current_bbox.max.x() = temp_bbox.max.x();
                if (current_bbox.max.y() > temp_bbox.max.y()) current_bbox.max.y() = temp_bbox.max.y();
            }
            shared_width    = (int)current_bbox.size().x();
            shared_depth    = (int)current_bbox.size().y();
            shared_center_x = (int)current_bbox.center().x();
            shared_center_y = (int)current_bbox.center().y();
        }
    }
    double exclude_width = 0., exclude_depth = 0.;
    if (current_exclude_area.size() >= 4) {
        exclude_width = current_exclude_area[2].x() - current_exclude_area[0].x();
        exclude_depth = current_exclude_area[2].y() - current_exclude_area[0].y();
    }
    bool has_wipe_tower = false;
    const BoundingBoxf3 bbox = downward_plate_bbox(model, config, plate_index, is_sequence, &has_wipe_tower);
    Vec3d offset = Vec3d::Zero();
    if (shrink_to_new_bed == 1) {
        offset = Vec3d(double(current_width) / 2 - double(old_width) / 2, double(current_depth) / 2 - double(old_depth) / 2, 0.);
    } else if (shrink_to_new_bed == 2) {
        Vec3d new_center_offset((double(current_width) + exclude_width) / 2, (double(current_depth) + exclude_depth) / 2, 0.);
        const Vec3d size = bbox.size();
#ifndef ENGINE_ORCA
        constexpr int DOWNWARD_CHECK_MARGIN = 10;   // BambuStudio.hpp 20
#endif
        if (shared_center_x != 0)
            new_center_offset(0) = (double)shared_center_x;
        else if (exclude_width > 0) {
            if (size.x() > (current_width - exclude_width))
                new_center_offset(0) = double(current_width) / 2;
#ifndef ENGINE_ORCA
            else if (size.x() <= (current_width - exclude_width - DOWNWARD_CHECK_MARGIN))
                new_center_offset(0) = new_center_offset(0) - DOWNWARD_CHECK_MARGIN / 2;
#endif
        }
        if (shared_center_y != 0)
            new_center_offset(1) = (double)shared_center_y;
        else if (exclude_depth > 0) {
            if (size.y() > (current_depth - exclude_depth))
                new_center_offset(1) = double(current_depth) / 2;
#ifndef ENGINE_ORCA
            else if (size.y() <= (current_depth - exclude_depth - DOWNWARD_CHECK_MARGIN))
                new_center_offset(1) = new_center_offset(1) - DOWNWARD_CHECK_MARGIN / 2;
#endif
        }
        offset = new_center_offset - bbox.center();
    }
#ifndef ENGINE_ORCA
    else if (shared_center_x != 0 && shared_width > 0 && shared_depth > 0 && has_wipe_tower) {
        // The same bed: a plate whose objects and tower fit the shared area
        // is centred on it (BambuStudio.cpp 4584-4620).
        const Vec3d bbox_size = bbox.size();
        if (bbox_size.x() <= shared_width && bbox_size.y() <= shared_depth)
            offset = Vec3d(shared_center_x, shared_center_y, 0.) - bbox.center();
    }
#endif
    offset(2) = 0.;
    if (offset.x() == 0. && offset.y() == 0.)
        return shrink_to_new_bed;
    for (ModelObject* object : model.objects)
        for (ModelInstance* inst : object->instances)
            inst->set_offset(inst->get_offset() + offset);
    for (const char* key : {"wipe_tower_x", "wipe_tower_y"}) {
        auto* opt = config.option<ConfigOptionFloats>(key);
        if (!opt || opt->values.empty())
            continue;
        // A short list grows with copies of its first value, as the
        // official set_at does (Config.hpp 437) and as get_at reads it.
        if ((int)opt->values.size() <= plate_index)
            opt->values.resize(plate_index + 1, opt->values.front());
        opt->values[plate_index] += key[11] == 'x' ? offset.x() : offset.y();
    }
    moved = offset;
    return shrink_to_new_bed;
}

/// --arrange 1: place this plate's objects on the bed the way the official CLI
/// arranges one plate (`--arrange 1` with a plate to slice; BambuStudio.cpp
/// 5730-5952 at 5873b5f, OrcaSlicer.cpp 4990-5210 at 31f6803):
///   - each printable instance is an item from get_instance_arrange_poly
///     (libslic3r/ModelArrange.cpp 119, both engines), which carries its
///     height, temperatures, name and a brim width from its support settings;
///     unprintable instances are arranged on their own;
///   - the prime tower, when the plate needs one, is a fixed item where the
///     plate's wipe_tower_x/y put it, clamped inside the bed, and the clamped
///     position is written back to the plate's wipe_tower_x/y entry;
///   - the bed exclusion boxes (and the wrapping-detection area when it is on)
///     are fixed items and excluded regions;
///   - the arrange settings are the official ones (min_obj_distance 0 so the
///     items' own inflation applies, print-by-object, I3 alignment, the
///     printer's clearances), then update_arrange_params,
///     update_selected_items_inflation, update_unselected_items_inflation,
///     (Orca) update_selected_items_axis_align and get_shrink_bedpts.
/// An object larger than the bed is refused first, with the official -50
/// sentence plus its size against the bed; objects that do not all land on
/// the bed are refused with the official arrange sentence (-21,
/// BambuStudio.cpp 5938-5944; OrcaSlicer.cpp 5198-5204).
/// `landed_out`, for --repetitions: how many printable items landed on the
/// plate; the arrange is applied even when some did not, and those are
/// left off the plate (not printed), as the official copies loop does.
static bool arrange_on_bed(Slic3r::Model& model, Slic3r::DynamicPrintConfig& config,
                           int plate_index, size_t sliced_filament_count, PlateOutcome& outcome,
                           int* landed_out = nullptr) {
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
    // Each instance on its own: the arrange places every instance as its own
    // item, so copies that start far apart are not one object wider than the bed.
    // A part counts as larger than the bed only when it fits in neither
    // orientation. Whether one that fits only turned 90 degrees is turned is
    // the arrange's call, and the engines differ: Bambu's arrange offers a
    // too-big item its minimum-area rotation even with rotations off
    // (Arrange.cpp 937-941 at 5873b5f), Orca's only with rotations on
    // (Arrange.cpp 995-1004 at 31f6803), and both turn a long part onto Y on
    // I3 printers. When the arrange leaves it off the bed, -21 below says why.
    for (ModelObject* object : model.objects) {
        object->ensure_on_bed();
        for (size_t i = 0; i < object->instances.size(); ++i) {
            const BoundingBoxf3 box = object->instance_bounding_box(i);
            const Vec3d size = box.size();
            const bool fits_as_is = size.x() <= bed.size().x() + EPSILON && size.y() <= bed.size().y() + EPSILON;
            const bool fits_turned = size.y() <= bed.size().x() + EPSILON && size.x() <= bed.size().y() + EPSILON;
            const bool too_wide = !fits_as_is && !fits_turned;
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

    // A fresh set of arrange settings (BambuStudio.cpp 5579; OrcaSlicer.cpp
    // 4839), with the progress callback quiet: its default prints to stdout.
    ArrangeParams params;
    params.progressind = [](unsigned, std::string) {};
    // Print by object, from the plate's own sequence or else the project's
    // (get_print_sequence, BambuStudio.cpp 4403-4416, called at 5624;
    // OrcaSlicer.cpp 4883). The caller has already laid the plate's own
    // print_sequence (plate_N.json is_seq_print, the plate's settings) over
    // the config.
    if (const auto* seq = config.option<ConfigOptionEnum<PrintSequence>>("print_sequence"))
        params.is_seq_print = seq->value == PrintSequence::ByObject;

    // Step 1: the items (BambuStudio.cpp 5734-5767; OrcaSlicer.cpp 4994-5027).
    ArrangePolygons selected, unselected, unprintable;
    ModelInstancePtrs selected_instances, unprintable_instances;
    for (ModelObject* mo : model.objects) {
        for (ModelInstance* minst : mo->instances) {
            ArrangePolygon ap = get_instance_arrange_poly(minst, config);
            ArrangePolygons& cont = minst->printable ? selected : unprintable;
            ModelInstancePtrs& owners = minst->printable ? selected_instances : unprintable_instances;
            ap.itemid = cont.size();
            cont.emplace_back(std::move(ap));
            owners.emplace_back(minst);
        }
    }

    const BoundingBoxf3 plate_box(Vec3d(bed.min.x(), bed.min.y(), 0.), Vec3d(bed.max.x(), bed.max.y(), bed_height));
    const std::vector<BoundingBoxf> exclude_boxes = arrange_exclude_boxes(config);
    const auto* wrapping_opt = config.option<ConfigOptionBool>("enable_wrapping_detection");
    const bool enable_wrapping_detect = wrapping_opt != nullptr && wrapping_opt->value;
    const auto* wrapping_area = config.option<ConfigOptionPoints>("wrapping_exclude_area");
    const bool wrapping_area_empty = wrapping_area == nullptr || wrapping_area->values.empty();
    // Smooth timelapse: --enable-timelapse with a smooth timelapse_type
    // (BambuStudio.cpp 4183-4187; OrcaSlicer.cpp 3596-3599).
    const auto* timelapse_type_opt = config.option("timelapse_type");
    const bool is_smooth_timelapse = g_arrange_switches.enable_timelapse && timelapse_type_opt &&
                                     timelapse_type_opt->getInt() == TimelapseType::tlSmooth;

    // The prime tower as a fixed item (BambuStudio.cpp 5770-5878; OrcaSlicer.cpp
    // 5030-5134). No duplicate copies in slicer-cli, so the tower starts where
    // the plate's wipe_tower_x/y put it ("keep the original").
    if (config.has("wipe_tower_x") && (is_smooth_timelapse || !params.is_seq_print || (selected.size() <= 1))) {
        float x = dynamic_cast<const ConfigOptionFloats*>(config.option("wipe_tower_x"))->get_at(plate_index);
        float y = dynamic_cast<const ConfigOptionFloats*>(arrange_opt(config, "wipe_tower_y"))->get_at(plate_index);
        float w = arrange_opt(config, "prime_tower_width")->getFloat();
        float a = arrange_opt(config, "wipe_tower_rotation_angle")->getFloat();
#ifdef ENGINE_ORCA
        float v = arrange_opt(config, "prime_volume")->getFloat();
#else
        std::vector<double> volumes = dynamic_cast<const ConfigOptionFloats*>(arrange_opt(config, "filament_prime_volume"))->values;
        if (config.option<ConfigOptionEnum<PrimeVolumeMode>>("prime_volume_mode", true)->value == pvmSaving) {
            for (auto& val : volumes)
                val = 15.f;
        }
        const double v = volumes.empty() ? 0. : *std::max_element(volumes.begin(), volumes.end());
#endif
        // The plate's filament count from its last slice (slice_info), else the
        // filaments its objects use (PartPlate::get_extruders_under_cli).
        unsigned int filaments_cnt = (unsigned int)sliced_filament_count;
        if (filaments_cnt == 0)
            filaments_cnt = (unsigned int)arrange_plate_extruders(model, config, plate_index).size();

        if ((filaments_cnt <= 1) && !is_smooth_timelapse && (!enable_wrapping_detect || wrapping_area_empty)) {
            // Not a multi-colour plate: no tower to keep clear of.
        } else {
            // The printer's extruder count: one per nozzle_diameter entry
            // (DynamicPrintConfig::support_different_extruders, Bambu
            // PrintConfig.cpp 7776-7783; OrcaSlicer.cpp 2874-2875).
            int extruder_count = 1;
            if (const auto* nozzles = dynamic_cast<const ConfigOptionVectorBase*>(config.option("nozzle_diameter")))
                extruder_count = std::max(1, (int)nozzles->size());
            Vec3d wipe_tower_size = arrange_estimate_wipe_tower_size(model, config, w, v, extruder_count, (int)filaments_cnt,
                                                                     enable_wrapping_detect, plate_box, exclude_boxes);
            // PartPlateList's plate size: the bed's whole-mm width and depth
            // (BambuStudio.cpp 2054-2056, 4242-4247; reset_size 4313-4320).
            const int plate_width = area->values.size() >= 4 ? (int)(area->values[2].x() - area->values[0].x()) : (int)bed.size().x();
            const int plate_depth = area->values.size() >= 4 ? (int)(area->values[2].y() - area->values[0].y()) : (int)bed.size().y();
            float depth = wipe_tower_size(1);
            float margin = 15.f, wp_brim_width = 0.f;
            if (const ConfigOption* wipe_tower_brim_width_opt = config.option("prime_tower_brim_width")) {
                wp_brim_width = wipe_tower_brim_width_opt->getFloat();
                if (wp_brim_width < 0) wp_brim_width = WipeTower::get_auto_brim_by_height((float) wipe_tower_size.z());
            }
            w = wipe_tower_size(0);
            if ((y + depth + margin + wp_brim_width) > (float)plate_depth)
                y = (float)plate_depth - depth - margin - wp_brim_width;
            if ((x + w + margin + wp_brim_width) > (float)plate_width)
                x = (float)plate_width - w - margin - wp_brim_width;
            if (x < margin)
                x = margin;
            if (y < margin)
                y = margin;
            // The clamped position is the plate's tower position from here on:
            // print.set_plate_index makes the slice read this entry.
            ConfigOptionFloat wt_x_opt(x);
            ConfigOptionFloat wt_y_opt(y);
            config.option<ConfigOptionFloats>("wipe_tower_x", true)->set_at(&wt_x_opt, plate_index, 0);
            config.option<ConfigOptionFloats>("wipe_tower_y", true)->set_at(&wt_y_opt, plate_index, 0);

            ArrangePolygon wipe_tower_ap;
            Slic3r::Polygon ap({
                {scaled(x - wp_brim_width), scaled(y - wp_brim_width)},
                {scaled(x + w + wp_brim_width), scaled(y - wp_brim_width)},
                {scaled(x + w + wp_brim_width), scaled(y + depth + wp_brim_width)},
                {scaled(x - wp_brim_width), scaled(y + depth + wp_brim_width)}
            });
            wipe_tower_ap.bed_idx = 0;
            wipe_tower_ap.setter = NULL; // do not move wipe tower
            wipe_tower_ap.poly.contour = std::move(ap);
            wipe_tower_ap.translation  = {scaled(0.f), scaled(0.f)};
            wipe_tower_ap.rotation     = a;
            wipe_tower_ap.name = "WipeTower";
            wipe_tower_ap.is_virt_object = true;
            wipe_tower_ap.is_wipe_tower = true;
            ++wipe_tower_ap.priority;
            unselected.emplace_back(std::move(wipe_tower_ap));
        }
    }

    // The exclusion areas as fixed items (BambuStudio.cpp 5882; OrcaSlicer.cpp 5138).
    arrange_add_exclude_areas(unselected, config, 0.);

    // Step 2: the arrange settings (BambuStudio.cpp 5886-5909; OrcaSlicer.cpp
    // 5142-5166), with the official CLI's option values: allow_multicolor_oneplate
    // true and avoid_extrusion_cali_region false (their defaults, PrintConfig.cpp
    // 9896-9909 at 5873b5f, 10743-10756 at 31f6803); the clearances and the
    // printable height from the printer (BambuStudio.cpp 4226-4230;
    // OrcaSlicer.cpp 3638-3642).
    //
    // Rotations stay off. This is a deliberate choice: it follows the desktop
    // app's Arrange (GLCanvas3D.hpp ArrangeSettings enable_rotation = false,
    // line 564 at 5873b5f, 493 at 31f6803), not the official CLI, whose
    // allow_rotations option defaults to true (PrintConfig.cpp 9901-9904 at
    // 5873b5f, 10748-10751 at 31f6803). On an I3 printer align_to_y_axis
    // still turns a long part to the Y axis, as the official does (Bambu
    // Arrange.cpp 946-959, long side > 1.1 x short side; Orca
    // update_selected_items_axis_align, Arrange.cpp 155-254).
    // --allow-rotations, --allow-multicolor-oneplate and
    // --avoid-extrusion-cali-region set them as the official CLI does
    // (BambuStudio.cpp 5886-5888; OrcaSlicer.cpp 5143-5145); rotations stay
    // off unless --allow-rotations is given.
    params.allow_rotations                     = g_arrange_switches.allow_rotations;
    params.allow_multi_materials_on_same_plate = g_arrange_switches.allow_multicolor_oneplate;
    params.avoid_extrusion_cali_region         = g_arrange_switches.avoid_extrusion_cali_region;
    params.clearance_height_to_rod             = arrange_opt(config, "extruder_clearance_height_to_rod")->getFloat();
    params.clearance_height_to_lid             = arrange_opt(config, "extruder_clearance_height_to_lid")->getFloat();
#ifdef ENGINE_ORCA
    params.clearance_radius                    = arrange_opt(config, "extruder_clearance_radius")->getFloat();
#else
    params.cleareance_radius                   = arrange_opt(config, "extruder_clearance_max_radius")->getFloat();
#endif
    params.printable_height                    = arrange_opt(config, "printable_height")->getFloat();
    params.min_obj_distance = 0;
    if (params.is_seq_print) {
        // BED_SHRINK_SEQ_PRINT: 0 on Bambu, 5 on Orca (each engine's Arrange.hpp).
        params.bed_shrink_x = BED_SHRINK_SEQ_PRINT;
        params.bed_shrink_y = BED_SHRINK_SEQ_PRINT;
    }
    if (auto printer_structure_opt = config.option<ConfigOptionEnum<PrinterStructure>>("printer_structure"))
        params.align_to_y_axis = (printer_structure_opt->value == PrinterStructure::psI3);

#ifdef ENGINE_ORCA
    update_arrange_params(params, &config, selected);
    update_selected_items_inflation(selected, &config, params);
    update_unselected_items_inflation(unselected, &config, params);
    update_selected_items_axis_align(selected, &config, params);
    Points beds = get_shrink_bedpts(&config, params);
#else
    update_arrange_params(params, config, selected);
    update_selected_items_inflation(selected, config, params);
    update_unselected_items_inflation(unselected, config, params);
    Points beds = get_shrink_bedpts(config, params);
#endif
    arrange_add_exclude_areas(params.excluded_regions, config, scale_(1));

    // Step 3 (BambuStudio.cpp 5925-5926; OrcaSlicer.cpp 5185-5186).
    arrangement::arrange(selected, unselected, beds, params);
    arrangement::arrange(unprintable, {}, beds, params);

    // Every selected item must land on this plate (BambuStudio.cpp 5936-5945;
    // OrcaSlicer.cpp 5196-5205).
    std::string off_bed, only_turned;
    if (landed_out != nullptr) {
        int landed = 0;
        for (const ArrangePolygon& ap : selected)
            landed += ap.bed_idx == 0 ? 1 : 0;
        *landed_out = landed;
        apply_arrange_polys(selected, selected_instances, [](ArrangePolygon&) {});
        apply_arrange_polys(unprintable, unprintable_instances, [](ArrangePolygon&) {});
        for (ModelObject* object : model.objects)
            object->ensure_on_bed();
        return landed == int(selected.size());
    }
    for (size_t k = 0; k < selected.size(); ++k) {
        if (selected[k].bed_idx == 0)
            continue;
        off_bed += (off_bed.empty() ? "'" : ", '") + selected[k].name + "'";
        // A part that fits the bed only turned 90 degrees, which this
        // engine's arrange did not do.
        const ModelInstance* inst = selected_instances[k];
        const ModelObject* object = inst->get_object();
        const auto it = std::find(object->instances.begin(), object->instances.end(), inst);
        const Vec3d size = object->instance_bounding_box(size_t(it - object->instances.begin())).size();
        if (!(size.x() <= bed.size().x() + EPSILON && size.y() <= bed.size().y() + EPSILON))
            only_turned += (only_turned.empty() ? "'" : ", '") + object->name + "'";
    }
    if (!off_bed.empty()) {
        std::string detail = "These objects do not fit on the " + bed_size_text(config) +
                             " bed together: " + off_bed + ".";
        if (!only_turned.empty())
            detail += " " + only_turned + " fit" + (only_turned.find(',') == std::string::npos ? "s" : "") +
                      " the bed only turned 90 degrees, which this arrange does not do; turn it in the file.";
        set_outcome_failure(outcome, CLI_OBJECT_ARRANGE_FAILED, detail);
        emit_event({{"event","plate_error"}, {"tag","ArrangeFailed"}, {"message", detail}});
        return false;
    }
    apply_arrange_polys(selected, selected_instances, [](ArrangePolygon&) {});
    // Unprintable instances: the official moves them on to a virtual bed past
    // the last plate (BambuStudio.cpp 5989-5997); a one-plate slice has no such
    // bed, so they keep their own arrange result on this bed. They are not printed.
    apply_arrange_polys(unprintable, unprintable_instances, [](ArrangePolygon&) {});
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

/// The prime tower's starting corner on an assemble-list plate: the
/// desktop's default (top left; the I3 printers' own), kept a margin from the
/// edges (BambuStudio.cpp 5382-5395, 5522-5535; OrcaSlicer.cpp 4635-4649,
/// 4776-4789; the defaults are PartPlate.cpp 68-74 / 66-70 in each engine).
/// The Orca arrange keeps it 1 mm plus prime_tower_width from the edges, as
/// its block reads it (OrcaSlicer.cpp 4636-4637); the Orca fixed plate and
/// both Bambu blocks use WIPE_TOWER_MARGIN (15 mm on Bambu, 1 mm on Orca).
static void assemble_tower_default_corner(Slic3r::DynamicPrintConfig& config, int plate_index, float margin) {
    using namespace Slic3r;
    float x = 165.f, y = 250.f;   // WIPE_TOWER_DEFAULT_X_POS / _Y_POS
    if (const auto* structure = config.option<ConfigOptionEnum<PrinterStructure>>("printer_structure");
        structure && structure->value == PrinterStructure::psI3) {
        x = 0.f;     // I3_WIPE_TOWER_DEFAULT_X_POS
        y = 250.f;   // I3_WIPE_TOWER_DEFAULT_Y_POS
    }
    if (x < margin) x = margin;
    if (y < margin) y = margin;
    ConfigOptionFloat wt_x_opt(x), wt_y_opt(y);
    config.option<ConfigOptionFloats>("wipe_tower_x", true)->set_at(&wt_x_opt, plate_index, 0);
    config.option<ConfigOptionFloats>("wipe_tower_y", true)->set_at(&wt_y_opt, plate_index, 0);
}

/// PartPlate::estimate_wipe_tower_polygon with use_global_objects (Bambu
/// PartPlate.cpp 2236-2281; Orca 2174-2220): plate_index's tower position
/// clamped inside a plate of plate_width x plate_depth (the plate list's
/// size), sized for `filaments_count` filaments and the tallest object of
/// `global_model`. Returns the fixed tower item (bed 0) and the clamped
/// position in `pos`; the caller writes it back, as the official callers do.
static Slic3r::arrangement::ArrangePolygon estimate_tower_polygon_global(const Slic3r::Model& global_model,
                                                                        const Slic3r::DynamicPrintConfig& config,
                                                                        int plate_index, int filaments_count,
                                                                        int plate_width, int plate_depth,
                                                                        Slic3r::Vec2d& pos) {
    using namespace Slic3r;
    using namespace Slic3r::arrangement;
    float x = dynamic_cast<const ConfigOptionFloats*>(config.option("wipe_tower_x"))->get_at(plate_index);
    float y = dynamic_cast<const ConfigOptionFloats*>(config.option("wipe_tower_y"))->get_at(plate_index);
    const float w = arrange_opt(config, "prime_tower_width")->getFloat();
#ifdef ENGINE_ORCA
    const double v = arrange_opt(config, "prime_volume")->getFloat();
    const float margin = WIPE_TOWER_MARGIN + arrange_opt(config, "prime_tower_brim_width")->getFloat();
#else
    std::vector<double> volumes = dynamic_cast<const ConfigOptionFloats*>(arrange_opt(config, "filament_prime_volume"))->values;
    if (const auto* pvm = config.option<ConfigOptionEnum<PrimeVolumeMode>>("prime_volume_mode"); pvm && pvm->value == pvmSaving)
        for (auto& val : volumes)
            val = 15.f;
    const double v = volumes.empty() ? 0. : *std::max_element(volumes.begin(), volumes.end());
    const float margin = WIPE_TOWER_MARGIN;
#endif
    const auto* wrapping_opt = config.option<ConfigOptionBool>("enable_wrapping_detection");
    const bool enable_wrapping_detect = wrapping_opt != nullptr && wrapping_opt->value;
    int extruder_count = 1;
    if (const auto* nozzles = dynamic_cast<const ConfigOptionVectorBase*>(config.option("nozzle_diameter")))
        extruder_count = std::max(1, (int)nozzles->size());
    // use_global_objects: every object, so the plate box and the exclusion
    // boxes are not read.
    const Vec3d wt_size = arrange_estimate_wipe_tower_size(global_model, config, w, v, extruder_count, filaments_count,
                                                           enable_wrapping_detect, BoundingBoxf3(), {}, &global_model);
    const float depth = wt_size(1);
    float wp_brim_width = 0.f;
    if (const ConfigOption* brim_opt = config.option("prime_tower_brim_width")) {
        wp_brim_width = brim_opt->getFloat();
        if (wp_brim_width < 0) wp_brim_width = WipeTower::get_auto_brim_by_height((float) wt_size.z());
    }
    // std::clamp(x, margin, plate - w - margin - brim), spelled out.
    const auto clamp = [](float value, float lo, float hi) { return value < lo ? lo : (hi < value ? hi : value); };
    x = clamp(x, margin, (float)plate_width - w - margin - wp_brim_width);
    y = clamp(y, margin, (float)plate_depth - depth - margin - wp_brim_width);
    pos = Vec2d(x, y);

    ArrangePolygon wipe_tower_ap;
    Slic3r::Polygon ap({
        {scaled(x - wp_brim_width), scaled(y - wp_brim_width)},
        {scaled(x + w + wp_brim_width), scaled(y - wp_brim_width)},
        {scaled(x + w + wp_brim_width), scaled(y + depth + wp_brim_width)},
        {scaled(x - wp_brim_width), scaled(y + depth + wp_brim_width)}
    });
    wipe_tower_ap.bed_idx = plate_index;
    wipe_tower_ap.setter = NULL; // do not move wipe tower
    wipe_tower_ap.poly.contour = std::move(ap);
    wipe_tower_ap.translation = {scaled(0.f), scaled(0.f)};
    wipe_tower_ap.name = "WipeTower";
    wipe_tower_ap.is_virt_object = true;
    wipe_tower_ap.is_wipe_tower = true;
    return wipe_tower_ap;
}

/// One --load-assemble-list plate with need_arrange (BambuStudio.cpp
/// 5350-5502; OrcaSlicer.cpp 4606-4760). Unlike the general arrange, the
/// tower starts at the default corner and is sized for the filaments the
/// plate's objects use, measured over every object of the list
/// (estimate_wipe_tower_polygon with use_global_objects, Bambu PartPlate.cpp
/// 2236-2281, Orca 2174-2220), and the objects are only moved, never dropped
/// to the bed. An object the arrange leaves off the plate is
/// CLI_OBJECT_ARRANGE_FAILED.
static bool arrange_assemble_plate(Slic3r::Model& model, Slic3r::DynamicPrintConfig& config, int plate_index,
                                   int filaments_count, const Slic3r::Model& global_model, PlateOutcome& outcome) {
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

    ArrangeParams params;
    params.progressind = [](unsigned, std::string) {};
    if (const auto* seq = config.option<ConfigOptionEnum<PrintSequence>>("print_sequence"))
        params.is_seq_print = seq->value == PrintSequence::ByObject;

    // Step 1: every instance of the plate's objects.
    ArrangePolygons selected, unselected;
    ModelInstancePtrs selected_instances;
    for (ModelObject* mo : model.objects)
        for (ModelInstance* minst : mo->instances) {
            ArrangePolygon ap = get_instance_arrange_poly(minst, config);
            ap.itemid = selected.size();
            selected.emplace_back(std::move(ap));
            selected_instances.emplace_back(minst);
        }

    const auto* wrapping_opt = config.option<ConfigOptionBool>("enable_wrapping_detection");
    const bool enable_wrapping_detect = wrapping_opt != nullptr && wrapping_opt->value;
    const auto* wrapping_area = config.option<ConfigOptionPoints>("wrapping_exclude_area");
    const bool wrapping_area_empty = wrapping_area == nullptr || wrapping_area->values.empty();
    if ((!params.is_seq_print && filaments_count > 1) || (enable_wrapping_detect && !wrapping_area_empty)) {
#ifdef ENGINE_ORCA
        const float corner_margin = WIPE_TOWER_MARGIN + arrange_opt(config, "prime_tower_width")->getFloat();
#else
        const float corner_margin = WIPE_TOWER_MARGIN;
#endif
        assemble_tower_default_corner(config, plate_index, corner_margin);
        const int plate_width = area->values.size() >= 4 ? (int)(area->values[2].x() - area->values[0].x()) : (int)bed.size().x();
        const int plate_depth = area->values.size() >= 4 ? (int)(area->values[2].y() - area->values[0].y()) : (int)bed.size().y();
        Vec2d pos;
        ArrangePolygon wipe_tower_ap = estimate_tower_polygon_global(global_model, config, plate_index, filaments_count,
                                                                     plate_width, plate_depth, pos);
        ConfigOptionFloat wt_x_opt(pos.x()), wt_y_opt(pos.y());
        config.option<ConfigOptionFloats>("wipe_tower_x", true)->set_at(&wt_x_opt, plate_index, 0);
        config.option<ConfigOptionFloats>("wipe_tower_y", true)->set_at(&wt_y_opt, plate_index, 0);
        wipe_tower_ap.bed_idx = 0;
        unselected.emplace_back(std::move(wipe_tower_ap));
    }
    arrange_add_exclude_areas(unselected, config, 0.);

    // Step 2: the same settings as the general arrange (see arrange_on_bed).
    params.allow_rotations                     = g_arrange_switches.allow_rotations;
    params.allow_multi_materials_on_same_plate = g_arrange_switches.allow_multicolor_oneplate;
    params.avoid_extrusion_cali_region         = g_arrange_switches.avoid_extrusion_cali_region;
    params.clearance_height_to_rod             = arrange_opt(config, "extruder_clearance_height_to_rod")->getFloat();
    params.clearance_height_to_lid             = arrange_opt(config, "extruder_clearance_height_to_lid")->getFloat();
#ifdef ENGINE_ORCA
    params.clearance_radius                    = arrange_opt(config, "extruder_clearance_radius")->getFloat();
#else
    params.cleareance_radius                   = arrange_opt(config, "extruder_clearance_max_radius")->getFloat();
#endif
    params.printable_height                    = arrange_opt(config, "printable_height")->getFloat();
    params.min_obj_distance = 0;
    if (params.is_seq_print) {
        params.bed_shrink_x = BED_SHRINK_SEQ_PRINT;
        params.bed_shrink_y = BED_SHRINK_SEQ_PRINT;
    }
    if (auto printer_structure_opt = config.option<ConfigOptionEnum<PrinterStructure>>("printer_structure"))
        params.align_to_y_axis = (printer_structure_opt->value == PrinterStructure::psI3);
#ifdef ENGINE_ORCA
    update_arrange_params(params, &config, selected);
    update_selected_items_inflation(selected, &config, params);
    update_unselected_items_inflation(unselected, &config, params);
    update_selected_items_axis_align(selected, &config, params);
    Points beds = get_shrink_bedpts(&config, params);
#else
    update_arrange_params(params, config, selected);
    update_selected_items_inflation(selected, config, params);
    update_unselected_items_inflation(unselected, config, params);
    Points beds = get_shrink_bedpts(config, params);
#endif
    arrange_add_exclude_areas(params.excluded_regions, config, scale_(1));

    // Step 3 and the landing check (BambuStudio.cpp 5476-5499; OrcaSlicer.cpp 4733-4756).
    arrangement::arrange(selected, unselected, beds, params);
    std::string off_bed;
    for (const ArrangePolygon& ap : selected)
        if (ap.bed_idx != 0)
            off_bed += (off_bed.empty() ? "'" : ", '") + ap.name + "'";
    if (!off_bed.empty()) {
        const std::string detail = "These objects of the assemble list's plate " + std::to_string(plate_index + 1) +
                                   " do not fit on the " + bed_size_text(config) + " bed together: " + off_bed + ".";
        set_outcome_failure(outcome, CLI_OBJECT_ARRANGE_FAILED, detail);
        emit_event({{"event","plate_error"}, {"tag","ArrangeFailed"}, {"message", detail}});
        return false;
    }
    apply_arrange_polys(selected, selected_instances, [](ArrangePolygon&) {});
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
                                    const Slic3r::DynamicPrintConfig& config, const Slic3r::DynamicPrintConfig& project,
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
    // The run's summary reads the project's settings, m_print_config, not
    // the plate's copy (BambuStudio.cpp 6905-6910; OrcaSlicer.cpp 5908-5913).
    if (project.has("layer_height"))          outcome.layer_height = float(project.opt_float("layer_height"));
    if (project.has("wall_loops"))            outcome.wall_loops   = project.opt_int("wall_loops");
    if (const auto* d = project.option<Slic3r::ConfigOptionPercent>("sparse_infill_density"))
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
                                    PlateOutcome& outcome, const Slic3r::Vec2d& plate_origin) {
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
    // What the official export adds per plate (BambuStudio.cpp 7514-7532;
    // OrcaSlicer.cpp 6323-6343): the skipped objects, the printer model id,
    // the nozzle diameters, and each used filament's type, colour and id.
    pd->skipped_objects.assign(outcome.skipped_objects.begin(), outcome.skipped_objects.end());
    if (outcome.settings_merge && !outcome.settings_merge->printer_model_id.empty())
        pd->printer_model_id = outcome.settings_merge->printer_model_id;
    else if (!g_preset_printer_model_id.empty())
        pd->printer_model_id = g_preset_printer_model_id;
    if (const auto* nozzles = config.option("nozzle_diameter"))
        pd->nozzle_diameters = nozzles->serialize();
    {
        Slic3r::DynamicPrintConfig types = config;
        const auto* colours = config.option<Slic3r::ConfigOptionStrings>("filament_colour");
        const auto* ids     = config.option<Slic3r::ConfigOptionStrings>("filament_ids");
        for (auto& info : pd->slice_filaments_info) {
            std::string display_type;
            info.type        = types.get_filament_type(display_type, info.id);
            info.color       = colours ? colours->get_at(info.id) : "#FFFFFF";
            info.filament_id = ids ? ids->get_at(info.id) : "";
        }
    }
#ifdef ENGINE_BAMBU
    // --skip-useless-pick: a plate of one object needs no object labels
    // (BambuStudio.cpp 7486-7498).
    if (g_skip_useless_pick) {
        int printable = 0;
        for (const Slic3r::ModelObject* object : model.objects)
            for (const Slic3r::ModelInstance* inst : object->instances)
                printable += inst->printable && inst->print_volume_state == Slic3r::ModelInstancePVS_Inside ? 1 : 0;
        if (printable == 1)
            pd->is_label_object_enabled = false;
    }
#endif
    outcome.plate_data = pd;
    // The plate's first-layer boxes, Metadata/plate_N.json (BambuStudio.cpp
    // 8021-8125; OrcaSlicer.cpp 6833-6935), on the plate: the objects' boxes
    // come without the plate offset (PrintObject::get_first_layer_bbox,
    // shift_without_plate_offset), and the tower's corners take the plate's
    // origin off (BambuStudio.cpp 8107-8108; OrcaSlicer.cpp 6938-6939).
    {
        auto bbox = std::make_shared<Slic3r::PlateBBoxData>();
        const auto* seq = config.option<Slic3r::ConfigOptionEnum<Slic3r::PrintSequence>>("print_sequence");
        bbox->is_seq_print = seq && seq->value == Slic3r::PrintSequence::ByObject;
        if (const auto* bed = config.option<Slic3r::ConfigOptionEnum<Slic3r::BedType>>("curr_bed_type"))
            bbox->bed_type = Slic3r::bed_type_to_gcode_string(bed->value);
        bbox->first_extruder = print.get_tool_ordering().first_extruder();
#ifdef ENGINE_BAMBU
        if (const auto* nozzles = dynamic_cast<const Slic3r::ConfigOptionFloatsNullable*>(config.option("nozzle_diameter")))
#else
        if (const auto* nozzles = dynamic_cast<const Slic3r::ConfigOptionFloats*>(config.option("nozzle_diameter")))
#endif
            bbox->nozzle_diameter = float(nozzles->get_at(bbox->first_extruder));
        Slic3r::BoundingBoxf bbox_all;
        for (Slic3r::PrintObject* obj : const_cast<Slic3r::Print&>(print).objects()) {
            Slic3r::BBoxData data;
            const auto bb = Slic3r::unscaled(obj->get_first_layer_bbox(data.area, data.layer_height, data.name));
            bbox_all.merge(bb);
            data.area *= float(SCALING_FACTOR * SCALING_FACTOR);
            data.id   = int(obj->id().id);
            data.bbox = {bb.min.x(), bb.min.y(), bb.max.x(), bb.max.y()};
            bbox->bbox_objs.emplace_back(std::move(data));
        }
        if (print.has_wipe_tower()) {
            const auto corners = print.first_layer_wipe_tower_corners();
            if (!corners.empty()) {
                auto bb = Slic3r::unscaled(Slic3r::BoundingBox(corners[0], corners[2]));
                bb.min -= plate_origin;
                bb.max -= plate_origin;
                bbox_all.merge(bb);
                Slic3r::BBoxData data;
                data.name = "wipe_tower";
                data.id   = std::max(0, outcome.plate_id - 1) + 1000;
                data.bbox = {bb.min.x(), bb.min.y(), bb.max.x(), bb.max.y()};
                bbox->bbox_objs.emplace_back(std::move(data));
            }
        }
        bbox->bbox_all = {bbox_all.min.x(), bbox_all.min.y(), bbox_all.max.x(), bbox_all.max.y()};
        for (const auto& info : pd->slice_filaments_info) {
            bbox->filament_ids.push_back(info.id);
            bbox->filament_colors.push_back(info.color);
        }
        outcome.plate_bbox = bbox;
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
    // With --no-check the official CLI does not stop on these warnings
    // (BambuStudio.cpp 7132-7142; OrcaSlicer.cpp 6113-6123).
    for (const json& w : outcome.no_check ? json::array() : outcome.warnings) {
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

#ifdef ENGINE_ORCA
/// A string setting of `config`, empty when the key is not there. The project's
/// preset ids are read this way: a 3MF that states none is not a project this
/// command line rebases.
static std::string config_id_text(const Slic3r::DynamicPrintConfig& config, const char* key) {
    const auto* opt = config.option<Slic3r::ConfigOptionString>(key);
    return opt != nullptr ? opt->value : std::string();
}

/// Any setting's value as the engine writes it (`serialize`), empty when the key
/// is not there: a coEnum such as support_type has no ConfigOptionString behind
/// it, so the text a decision reads is the serialized one. Takes the config
/// base, so an object's own settings (ModelConfigObject) read the same way as a
/// project's.
static std::string config_value_text(const Slic3r::ConfigBase& config, const char* key) {
    const Slic3r::ConfigOption* opt = config.option(key);
    return opt != nullptr ? opt->serialize() : std::string();
}

/// True when this engine ships the system presets the project names. The
/// desktop loads the project over the preset it names and only warns when the
/// preset is missing (Preset.cpp 2446-2457 load_external_preset runs with
/// found = false); this command line keeps the flat file config for the whole
/// project instead, and says so in an event.
static bool project_presets_shipped(Slic3r::PresetBundle& bundle, const Slic3r::DynamicPrintConfig& config) {
    const std::string printer = config_id_text(config, "printer_settings_id");
    const std::string process = config_id_text(config, "print_settings_id");
    return !printer.empty() && !process.empty() &&
           bundle.printers.find_preset(printer, false) != nullptr &&
           bundle.prints.find_preset(process, false) != nullptr;
}
#endif

#ifdef ENGINE_ORCA
/// The Bambu Studio placeholder this engine has no value for, empty when there
/// is none: either of the two names goes unbound in OrcaSlicer -- its
/// PlaceholderParser sets initial_extruder and reads nozzle_diameter from the
/// config, and neither binds BambuStudio's nozzle_diameter_at_nozzle_id or
/// initial_nozzle_id (BambuStudio binds both in GCode.cpp 2656-2660 at
/// 5873b5f). Upstream main binds them with its multi-nozzle engine (GCode.cpp
/// 3334-3335 on main), too large to backport. `key_out` names the custom-gcode
/// key the placeholder was found in.
static std::string bambu_only_gcode_placeholder(const Slic3r::DynamicPrintConfig& config,
                                                std::string&                       key_out) {
    static const std::vector<std::string> kPlaceholders = {"nozzle_diameter_at_nozzle_id",
                                                           "initial_nozzle_id"};
    auto scan = [&](const std::string& key, const std::string& text) {
        for (const std::string& token : kPlaceholders) {
            if (text.find(token) == std::string::npos)
                continue;
            key_out = key;
            return token;
        }
        return std::string();
    };
    for (const std::string& key : kGcodeStringKeys) {
        const Slic3r::ConfigOption* opt = config.option(key);
        if (opt == nullptr)
            continue;
        // The serialized form carries every value of a per-filament key too.
        if (const std::string found = scan(key, opt->serialize()); !found.empty())
            return found;
    }
    for (const std::string& key : kGcodeStringsKeys) {
        const Slic3r::ConfigOption* opt = config.option(key);
        if (opt == nullptr)
            continue;
        if (const std::string found = scan(key, opt->serialize()); !found.empty())
            return found;
    }
    return {};
}

/// True when tree supports are in play for this run: the project says so, or an
/// object overrides the project's support type with a tree one (a plate's own
/// settings can carry it too, and are applied later). G4's warning is about a
/// value whose meaning only matters then.
static bool tree_support_enabled(const Slic3r::DynamicPrintConfig& config, const Slic3r::Model& model) {
    if (boost::starts_with(config_value_text(config, "support_type"), "tree"))
        return true;
    for (const Slic3r::ModelObject* object : model.objects) {
        if (object == nullptr)
            continue;
        // An object carries a ModelConfigObject, not a DynamicPrintConfig: its
        // own ConfigBase accessor answers the same question.
        const Slic3r::ConfigOption* opt = object->config.option("support_type");
        if (opt != nullptr && boost::starts_with(opt->serialize(), "tree"))
            return true;
    }
    return false;
}

/// Metadata/project_settings.config of a project 3MF, parsed. A null json when
/// the file has none or it cannot be read.
static json project_settings_json(const std::string& path) {
    std::string text;
    if (!read_zip_member(path, "Metadata/project_settings.config", text))
        return json();
    try {
        const json settings = json::parse(text);
        return settings.is_object() ? settings : json();
    } catch (...) {
        return json();
    }
}

/// True when the maker listed `key` among the keys it changed. The desktop (and
/// this command line) keep those keys' file values and take the rest from the
/// system preset (Preset.cpp 2446-2457), so this is the test for "the file
/// states this value on purpose".
static bool maker_changed_key(const Slic3r::DynamicPrintConfig& file_config, const char* key) {
    const auto* diff = file_config.option<Slic3r::ConfigOptionStrings>("different_settings_to_system");
    if (diff == nullptr || diff->values.empty())
        return false;
    std::vector<std::string> keys;
    Slic3r::unescape_strings_cstyle(diff->values.front(), keys);
    return std::find(keys.begin(), keys.end(), key) != keys.end();
}

/// True when the file states `value` for `key` (as a string or a number).
static bool file_states_number(const json& settings, const char* key, double value) {
    const auto it = settings.find(key);
    if (it == settings.end())
        return false;
    if (it->is_string())
        return it->get<std::string>() == std::to_string(int(value));
    if (it->is_number())
        return it->get<double>() == value;
    return false;
}
#endif

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
static bool preset_usable(const Slic3r::PresetCollection& collection, const std::string& name) {
    const Slic3r::Preset* p = name.empty() ? nullptr : collection.find_preset(name, false);
    return p && p->name == name && p->is_compatible;
}

/// The first compatible, listed preset with the highest match quality, the
/// desktop's PresetCollection::first_compatible_idx (Preset.hpp 630 at
/// 5873b5f, 686-709 at 31f6803): ties keep the earlier preset in collection
/// order. The desktop counts only visible presets; here a listed preset is
/// visible, as naming the printer installs its presets, the way the setup
/// wizard does.
template<class Quality>
static std::string first_compatible_name(const Slic3r::PresetCollection& collection, Quality quality) {
    std::string best;
    int best_quality = -1;
    for (const Slic3r::Preset& p : collection) {
        if (!is_listed_preset(p) || !p.is_compatible) continue;
        const int q = quality(p);
        if (q > best_quality) { best_quality = q; best = p.name; }
    }
    return best;
}

/// The process the desktop selects for a printer picked with no process
/// selected yet: PresetBundle::update_compatible re-selects through
/// first_compatible_idx with PreferedPrintProfileMatch(nullptr,
/// default_print_profile), which scores the printer's default name 3 and
/// any other visible preset 2 (PresetBundle.cpp 5617-5640 at 5873b5f,
/// 5215-5240 at 31f6803). A default this engine does not ship (the Orca
/// pin's Snapmaker U1 0.4 names "0.20mm Standard @Snapmaker") therefore
/// gives the first compatible process.
///
/// With no compatible process at all, first_compatible_idx returns index 0,
/// the built-in default process (Preset.hpp 686-709 at 31f6803: "No
/// compatible preset found, return the default preset"; the same at
/// 5873b5f), and the desktop slices with that.
static std::string desktop_default_process(const Slic3r::PresetCollection& prints, const std::string& declared) {
    const std::string picked =
        first_compatible_name(prints, [&](const Slic3r::Preset& p) { return (p.name == declared ? 1 : 0) + 2; });
    return picked.empty() ? prints.default_preset().name : picked;
}

/// The filament the desktop selects for a printer picked on a fresh install
/// (PresetBundle::load_selections, PresetBundle.cpp 2740-2753 and 2850-2857
/// at 31f6803): the printer's first default filament when it is a visible,
/// compatible preset, else the placeholder, which the last step replaces with
/// filaments.first_compatible() in every slot. With no installed-filaments
/// section every system filament is visible (Preset::set_visible_from_appconfig,
/// Preset.cpp 830-843), so the pick is the first compatible filament in
/// collection order: on the Snapmaker U1 "Generic ABS @System", as the
/// OrcaSlicer 2.4.2 app selects. (update_compatible's PLA preference sets
/// only the editor's preset, which this step overrides.)
static std::string desktop_default_filament(const Slic3r::PresetCollection& filaments,
                                            const std::vector<std::string>& declared) {
    if (!declared.empty() && preset_usable(filaments, declared.front()))
        return declared.front();
    return first_compatible_name(filaments, [](const Slic3r::Preset&) { return 0; });
}

/// Why the desktop's pick is not the printer's first stated default, or
/// empty when it is.
static std::string default_replaced_note(const char* kind, const std::string& picked, const std::string& declared) {
    if (picked.empty() || picked == declared) return {};
    std::string why;
    if (declared.empty())
        why = "the printer names no default";
    else
        why = "the printer's default '" + declared + "' is not available";
    return std::string(kind) + " '" + picked + "' (" + why + ")";
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
        // Only the system presets --list-presets offers: never a built-in
        // "- default -" preset, which is no printer's, process's or filament's.
        // The built-in default process is taken only as the desktop's own
        // fallback (desktop_default_process), never by name.
        const bool desktop_fallback = preset && preset->is_default && kind == std::string("process") &&
                                      o.process_preset.empty();
        if (preset && preset->name == name && (is_listed_preset(*preset) || desktop_fallback)) {
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

    // An omitted process or filament takes what the desktop selects when
    // the printer is picked: its default when this engine ships it, else
    // the desktop's own fallback (desktop_default_process/_filament).
    std::vector<std::string> substituted_defaults;
    std::string process = o.process_preset;
    if (process.empty()) {
        const std::string declared = printer.has("default_print_profile") ? printer.opt_string("default_print_profile") : std::string();
        process = desktop_default_process(bundle.prints, declared);
        const std::string note = default_replaced_note("process", process, declared);
        if (!note.empty()) substituted_defaults.push_back(note);
    }
    if (process.empty()) {
        error = "Printer preset '" + o.printer_preset + "' has no compatible process; give --process-preset.";
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
        std::vector<std::string> declared;
        if (const auto* d = printer.option<Slic3r::ConfigOptionStrings>("default_filament_profile"))
            declared = d->values;
        const std::string wanted = desktop_default_filament(bundle.filaments, declared);
        const std::string note = default_replaced_note("filament", wanted,
                                                       declared.empty() ? std::string() : declared.front());
        if (!note.empty()) substituted_defaults.push_back(note);
        if (!wanted.empty()) filaments.push_back(wanted);
    }
    if (filaments.empty()) {
        error = "Printer preset '" + o.printer_preset + "' has no compatible filament; give --filament-preset.";
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
    // The desktop adds each filament slot with PresetBundle::set_num_filaments
    // (Sidebar::add_custom_filament: BambuStudio Plater.cpp 4198, PresetBundle.cpp
    // 2343-2393; OrcaSlicer Plater.cpp 3243, PresetBundle.cpp 2993-3027),
    // which sizes the project's per-filament lists (filament_colour,
    // filament_multi_colour, filament_colour_type, filament_map, and on
    // BambuStudio the nozzle and volume maps) to the slot count, each new slot
    // taking the last slot's value. No colour is picked here, as no one picks
    // one on the command line.
    bundle.set_num_filaments(unsigned(filaments.size()), std::string());
    bundle.filament_presets = filaments;
    // The desktop app runs this whenever the filament list changes: it sizes
    // the project's flush_volumes_matrix to filaments x filaments per nozzle
    // and flush_multiplier to the nozzle count (PresetBundle.cpp at both
    // pins). Without it a two-nozzle printer (X2D, H2D) reads past the
    // matrix in ToolOrdering::reorder_extruders_for_minimum_flush_volume.
    bundle.update_multi_material_filament_presets();
#ifdef ENGINE_ORCA
    // OrcaSlicer's update_multi_material_filament_presets gives a printer with
    // more extruders than filaments one filament per extruder (PresetBundle.cpp
    // 5120-5131), single_extruder_multi_material or not, and leaves the colour
    // lists as they were. The desktop never keeps them apart: it sizes
    // filament_colour, filament_multi_colour and filament_colour_type to the
    // filament count when it loads the printer (PresetBundle::load_selections
    // 2780-2795, update_selections 2636-2651, new slots "#26A69A" and type
    // "1"), and the official CLI refuses a project where they differ (-5,
    // "filament_is_support's count ... not equal to filament_colour's size").
    // A slice with them apart reads past the shorter lists. BambuStudio's
    // update_multi_material_filament_presets adds no filament (PresetBundle.cpp
    // 5527-5538), so its lists already match.
    // set_num_filaments sizes the lists but gives the new slots no colour of
    // their own here (the filament count has already grown, so its
    // new-colour branch is skipped, and the lists fill from the first slot);
    // the new slots then take load_selections' "#26A69A" and type "1".
    if (const auto* colours = bundle.project_config.option<Slic3r::ConfigOptionStrings>("filament_colour");
        colours && colours->values.size() != bundle.filament_presets.size()) {
        const size_t old_count = colours->values.size();
        const size_t count = bundle.filament_presets.size();
        bundle.set_num_filaments(unsigned(count), std::string());
        auto& colour      = bundle.project_config.option<Slic3r::ConfigOptionStrings>("filament_colour")->values;
        auto& multi       = bundle.project_config.option<Slic3r::ConfigOptionStrings>("filament_multi_colour")->values;
        auto& colour_type = bundle.project_config.option<Slic3r::ConfigOptionStrings>("filament_colour_type")->values;
        for (size_t i = old_count; i < count; ++i) {
            if (i < colour.size()) colour[i] = "#26A69A";
            if (i < multi.size()) multi[i] = "#26A69A";
            if (i < colour_type.size()) colour_type[i] = "1";
        }
    }
#endif
    filaments = bundle.filament_presets;

    // What the desktop app sets in the project when the printer is picked:
    // the plate type (desktop_bed_type) and, on the BambuStudio build, the
    // nozzle statistics. Both are project settings full_config() applies
    // (construct_full_config: BambuStudio PresetBundle.cpp 97, OrcaSlicer 82).
    const slicer_cli::DesktopBedType bed = slicer_cli::desktop_bed_type(bundle);
    bundle.project_config.set_key_value("curr_bed_type", new Slic3r::ConfigOptionEnum<Slic3r::BedType>(bed.type));
    g_preset_printer_model_id = bundle.printers.get_edited_preset().get_printer_type(&bundle);
    slicer_cli::apply_printer_pick(bundle);
#ifdef ENGINE_ORCA
    // The settings the desktop slices with: full_config(false), every
    // filament's own values for each extruder variant with
    // filament_self_index (OrcaSlicer Plater.cpp 7959-7963), which the Print
    // maps onto the extruders. full_config(true) maps them already and
    // writes no filament_self_index (PresetBundle.cpp 4005-4060 at 31f6803),
    // so on a printer with several extruders the Print mapped every
    // filament to filament 1's values (H2D, 4 filaments: nozzle_temperature
    // 220,220,220,220 for 220,220,245,270). The official CLI's own merge of
    // --load-filaments files writes some of these lists differently (one
    // value, or the first filament's value followed by 0s); named presets
    // follow the desktop's full_fff_config, one value per filament from
    // each filament's own preset.
    out = bundle.full_config(false);
#else
    // BambuStudio's full_config(true) keeps each filament's own values and
    // its filament_self_index (PresetBundle.cpp 3400-3460 at 5873b5f), and
    // the Print maps them as for full_config(false); false would also write
    // the AMS drying lists per variant, unlike a desktop project.
    out = bundle.full_config();
#endif
    // full_config() writes each filament's own colour; a slot without one
    // keeps the engine default.
    emit_event({{"event","presets_resolved"},
                {"tag","NamedPresetsResolved"},
                {"printer", o.printer_preset},
                {"process", process},
                {"filaments", filaments},
                {"curr_bed_type", slicer_cli::bed_type_name(bed.type)},
                {"curr_bed_type_reason", bed.why},
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
        if (!printer || printer->name != printer_name || !is_listed_preset(*printer)) {
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
        // The process and filament a slice with only --printer-preset takes:
        // the desktop's pick (desktop_default_process/_filament), so a listed
        // default is always one --process-preset/--filament-preset accepts.
        json defaults_replaced = json::array();
        const std::string declared_process = cfg.has("default_print_profile") ? cfg.opt_string("default_print_profile") : std::string();
        const std::string process = desktop_default_process(bundle.prints, declared_process);
        out["default_process"] = process;
        if (const std::string note = default_replaced_note("process", process, declared_process); !note.empty())
            defaults_replaced.push_back(note);
        std::vector<std::string> declared_filaments;
        if (const auto* d = cfg.option<Slic3r::ConfigOptionStrings>("default_filament_profile"))
            declared_filaments = d->values;
        const std::string filament = desktop_default_filament(bundle.filaments, declared_filaments);
        json default_filaments = json::array();
        if (!filament.empty()) default_filaments.push_back(filament);
        out["default_filaments"] = default_filaments;
        if (const std::string note = default_replaced_note("filament", filament,
                declared_filaments.empty() ? std::string() : declared_filaments.front()); !note.empty())
            defaults_replaced.push_back(note);
        out["defaults_replaced"] = defaults_replaced;
        json processes = json::array(), filaments = json::array();
        for (const Slic3r::Preset& p : bundle.prints)
            if (is_listed_preset(p) && p.is_compatible) processes.push_back(p.name);
        for (const Slic3r::Preset& p : bundle.filaments)
            if (is_listed_preset(p) && p.is_compatible) filaments.push_back(p.name);
        out["processes"] = processes;
        out["filaments"] = filaments;
        // The plate type a slice with this printer takes (desktop_bed_type).
        const slicer_cli::DesktopBedType bed = slicer_cli::desktop_bed_type(bundle);
        out["default_bed_type"] = slicer_cli::bed_type_name(bed.type);
    }
    std::cout << out.dump(2, ' ', false, json::error_handler_t::replace) << std::endl;
    return 0;
}

/// A plate's logical width and depth, as the desktop's PartPlateList sizes
/// it from the printable area: whole millimetres, plus the axes tip for files
/// older than 1.5.9 (through reset_size(int, int, ...)). Zero without an area.
static Slic3r::Vec2d plate_logical_size(const Slic3r::DynamicPrintConfig& file_config, const Slic3r::Semver& file_version)
{
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
    return Slic3r::Vec2d(width, depth);
}

/// The scene origin of plate `index` (0-based) in a project of `plate_count`
/// plates, as the desktop's PartPlateList lays them out (see the call site).
static Slic3r::Vec2d plate_grid_origin(const Slic3r::DynamicPrintConfig& file_config,
                                       const Slic3r::Semver& file_version, int index, int plate_count)
{
    if (index <= 0 || plate_count <= 1)
        return Slic3r::Vec2d::Zero();
    const Slic3r::Vec2d size = plate_logical_size(file_config, file_version);
    if (size.x() <= 0. || size.y() <= 0.)
        return Slic3r::Vec2d::Zero();
    // compute_colum_count (PartPlate.hpp:38).
    const float value = std::sqrt((float)plate_count);
    const float round_value = std::round(value);
    const int cols = value > round_value ? (int)round_value + 1 : (int)round_value;
    const int row = index / cols, col = index % cols;
    const double gap = 1. / 5.;   // LOGICAL_PART_PLATE_GAP
    return Slic3r::Vec2d(col * size.x() * (1. + gap), -row * size.y() * (1. + gap));
}

/// Plate `index`'s box in the scene, PartPlate::get_build_volume (PartPlate.cpp
/// 3263-3285 at 5873b5f): its origin plus the first printable_area point, its
/// logical size and the printable height, widened by SceneEpsilon.
static Slic3r::BoundingBoxf3 plate_scene_box(const Slic3r::DynamicPrintConfig& file_config,
                                             const Slic3r::Semver& file_version, int index, int plate_count)
{
    const Slic3r::Vec2d origin = plate_grid_origin(file_config, file_version, index, plate_count);
    const Slic3r::Vec2d size = plate_logical_size(file_config, file_version);
    const auto* area = file_config.option<Slic3r::ConfigOptionPoints>("printable_area");
    const Slic3r::Vec2d first = area && !area->values.empty() ? area->values.front() : Slic3r::Vec2d::Zero();
    const double height = file_config.has("printable_height") ? file_config.opt_float("printable_height") : 0.;
    const double eps = Slic3r::BuildVolume::SceneEpsilon;
    return Slic3r::BoundingBoxf3(
        Slic3r::Vec3d(origin.x() + first.x() - eps, origin.y() + first.y() - eps, -eps),
        Slic3r::Vec3d(origin.x() + first.x() + size.x() + eps, origin.y() + first.y() + size.y() + eps, height + eps));
}

/// A project 3MF followed by model files (`project.3mf part.stl`): what the
/// official CLI does with them, for both engines.
///  - Each file loads into its own model, and is_bbl_3mf is reset for every
///    file, so it ends false (BambuStudio.cpp 1871; OrcaSlicer.cpp 1553):
///    --slice N becomes --slice 0 (2192-2196; 1839-1843), and none of the
///    Bambu-project steps after the load run (the move to a new bed, the
///    clearance re-arrange, the project's settings rules).
///  - A model file sets need_arrange (BambuStudio.cpp 2077; OrcaSlicer.cpp
///    1724), which only --arrange 0 turns off (5066-5081; 4327-4342).
///  - The models merge into one (BambuStudio.cpp 3947-3963; OrcaSlicer.cpp
///    3441-3457) and, with need_arrange, every object of every plate is
///    arranged across the plates (the global arrange, BambuStudio.cpp
///    5627-5722, 5886-5926, 5946-6006; OrcaSlicer.cpp 4887-4983, 5142-5186,
///    5189-5262), which adds plates when they are full and drops empty ones
///    (PartPlateList::rebuild_plates_after_arrangement).
/// The plan is computed once on the run's one model (RunState), before the
/// plates are sliced, and each plate's slice takes its instances from it.
struct ProjectPlan {
    bool done = false;
    int file_plate_count = 0;
    int plate_count = 0;
    // Per plate: its instances in the run's model (every object stays at its
    // place in the scene, in the order the arrange left them), and its grid
    // origin.
    std::vector<std::vector<Slic3r::ModelInstance*>> members;
    std::vector<Slic3r::Vec2d> origins;
    // wipe_tower_x / wipe_tower_y as the arrange left them (empty when the
    // arrange did not set them).
    std::vector<double> tower_x, tower_y;
    // Instances on no plate (left off every plate, or not printable).
    int off_plates = 0;
    bool arranged = false;
};

/// A project 3MF with model files after it, under --slice or for model
/// actions only: the run the official CLI makes of them (see ProjectPlan).
/// The product's default call (`file --plate N -o`) keeps loading the model
/// files into the plate it slices.
static bool trailing_models_port(const CliOptions& o) {
    return o.input_files.size() > 1 && slicer_cli::classify_3mf(o.input_file) == slicer_cli::ThreeMfKind::Project &&
           (o.slice_mode || slicer_cli::model_actions_only(o));
}

/// PartPlateList's plate grid for the plan: plate `index` of a list with
/// `cols` columns sits at (col * width * 1.2, -row * depth * 1.2)
/// (compute_origin, Bambu PartPlate.cpp 4306-4341; plate_stride_x/y with
/// LOGICAL_PART_PLATE_GAP 1/5). Unlike plate_grid_origin it also places a bed
/// past the last plate, as postprocess_arrange_polygon does.
static Slic3r::Vec2d plan_plate_origin(const Slic3r::Vec2d& plate_size, int index, int cols) {
    const double gap = 1. / 5.;
    const int row = index / cols, col = index % cols;
    return Slic3r::Vec2d(col * plate_size.x() * (1. + gap), -row * plate_size.y() * (1. + gap));
}

/// compute_colum_count (PartPlate.hpp:38).
static int plan_plate_cols(int count) {
    const float value = std::sqrt((float)std::max(1, count));
    const float round_value = std::round(value);
    return value > round_value ? (int)round_value + 1 : (int)round_value;
}

/// The plan of a project 3MF with model files after it (see ProjectPlan).
/// `model` holds every plate's objects at their scene places followed by the
/// model files' objects, and the arrange moves them in place; `config` is the
/// run's settings at the arrange (the setting flags applied), whose
/// wipe_tower_x / wipe_tower_y the arrange updates; `file_config` and
/// `file_version` are the project's own, for the plate list's size (the old
/// printer's, as the plate list keeps it without the Bambu-project resize,
/// BambuStudio.cpp 4313-4320; OrcaSlicer.cpp 3727-3729).
static bool plan_project_plates(Slic3r::Model& model, Slic3r::DynamicPrintConfig& config,
                                const Slic3r::DynamicPrintConfig& file_config, const Slic3r::Semver& file_version,
                                const Slic3r::PlateDataPtrs& plate_data, int filament_count, bool need_arrange,
                                ProjectPlan& plan, PlateOutcome& outcome) {
    using namespace Slic3r;
    using namespace Slic3r::arrangement;
    const int file_plates = std::max(1, (int)plate_data.size());
    const Vec2d plate_size = plate_logical_size(file_config, file_version);
    const auto* area = config.option<ConfigOptionPoints>("printable_area");
    if (!area || area->values.size() < 3 || plate_size.x() <= 0. || plate_size.y() <= 0.) {
        set_outcome_failure(outcome, CLI_OBJECT_ARRANGE_FAILED, "The printer states no bed shape (printable_area).");
        return false;
    }
    const Vec2d first_point = area->values.front();
    const double plate_height = file_config.has("printable_height") ? file_config.opt_float("printable_height") : 0.;
    // PartPlate::get_build_volume (Bambu PartPlate.cpp 3263-3285): the
    // plate's origin plus the bed's first point, the plate list's size.
    const auto plate_box = [&](int index, int count) {
        const Vec2d origin = plan_plate_origin(plate_size, index, plan_plate_cols(count));
        const double eps = BuildVolume::SceneEpsilon;
        return BoundingBoxf3(Vec3d(origin.x() + first_point.x() - eps, origin.y() + first_point.y() - eps, -eps),
                             Vec3d(origin.x() + first_point.x() + plate_size.x() + eps,
                                   origin.y() + first_point.y() + plate_size.y() + eps, plate_height + eps));
    };

    // The plate each instance is on, as the file states it: the plate's
    // identify ids, else the first plate its box meets (load_plate_objects).
    std::map<std::pair<int, int>, int> on_plate;
    for (size_t k = 0; k < plate_data.size(); ++k) {
        if (plate_data[k] == nullptr) continue;
        std::set<int> identify_ids;
        for (const auto& entry : plate_data[k]->obj_inst_map)
            if (entry.second.second > 0) identify_ids.insert(entry.second.second);
        for (size_t oi = 0; oi < model.objects.size(); ++oi)
            for (size_t ii = 0; ii < model.objects[oi]->instances.size(); ++ii) {
                const int id = model.objects[oi]->instances[ii]->loaded_id;
                if (id > 0 && identify_ids.count(id))
                    on_plate.emplace(std::make_pair(int(oi), int(ii)), int(k));
            }
    }
    for (size_t oi = 0; oi < model.objects.size(); ++oi)
        for (size_t ii = 0; ii < model.objects[oi]->instances.size(); ++ii) {
            if (on_plate.count({int(oi), int(ii)})) continue;
            const BoundingBoxf3 box = model.objects[oi]->instance_convex_hull_bounding_box(ii);
            for (int k = 0; k < file_plates; ++k)
                if (plate_box(k, file_plates).intersects(box)) {
                    on_plate.emplace(std::make_pair(int(oi), int(ii)), k);
                    break;
                }
        }
    std::vector<bool> locked(file_plates, false);
    for (int k = 0; k < (int)plate_data.size(); ++k)
        locked[k] = plate_data[k] != nullptr && plate_data[k]->locked;

    int plate_count = file_plates;
    if (need_arrange) {
        // Step 1, the global arrange (BambuStudio.cpp 5627-5722; OrcaSlicer.cpp
        // 4887-4983): every instance, those of a locked plate kept where they
        // are (PartPlateList::preprocess_arrange_polygon, Bambu PartPlate.cpp
        // 5831-5871).
        ArrangePolygons selected, unselected, unprintable, locked_aps;
        const int file_cols = plan_plate_cols(file_plates);
        for (size_t oi = 0; oi < model.objects.size(); ++oi) {
            ModelObject* mo = model.objects[oi];
            for (size_t ii = 0; ii < mo->instances.size(); ++ii) {
                ModelInstance* minst = mo->instances[ii];
                ArrangePolygon ap = get_instance_arrange_poly(minst, config);
                const auto it = on_plate.find({int(oi), int(ii)});
                if (it != on_plate.end() && locked[it->second]) {
                    const int k = it->second;
                    ap.bed_idx = k;
                    ap.row = k / file_cols;
                    ap.col = k % file_cols;
                    const Vec2d origin = plan_plate_origin(plate_size, k, file_cols);
                    ap.translation(X) -= scaled<double>(origin.x());
                    ap.translation(Y) -= scaled<double>(origin.y());
                    ap.itemid = locked_aps.size();
                    locked_aps.emplace_back(ap);
                } else {
                    // The official numbers the unprintable items in the
                    // selected list too.
                    ap.itemid = selected.size();
                    if (minst->printable)
                        selected.emplace_back(ap);
                    else
                        unprintable.emplace_back(ap);
                }
            }
        }
        ArrangeParams params;
        params.progressind = [](unsigned, std::string) {};
        if (const auto* seq = config.option<ConfigOptionEnum<PrintSequence>>("print_sequence"))
            params.is_seq_print = seq->value == PrintSequence::ByObject;
        // The exclusion areas on 16 beds (preprocess_exclude_areas' default).
        arrange_add_exclude_areas(unselected, config, 0., 16);
        // The tower on every bed (BambuStudio.cpp 5668-5722; OrcaSlicer.cpp
        // 4927-4981) when filaments are in use: the default corner on each file
        // plate, clamped by estimate_wipe_tower_polygon with every object
        // counted, and the last file plate's tower on the beds after it. The
        // used filaments: on Bambu the project's and --load-filament-ids'
        // (BambuStudio.cpp 2041-2046, 2083-2088); on Orca only
        // --load-filament-ids' (OrcaSlicer.cpp 1734).
        if (filament_count > 0) {
            float x = 165.f, y = 250.f;   // WIPE_TOWER_DEFAULT_X_POS / _Y_POS
            if (const auto* structure = config.option<ConfigOptionEnum<PrinterStructure>>("printer_structure");
                structure && structure->value == PrinterStructure::psI3) {
                x = 0.f;     // I3_WIPE_TOWER_DEFAULT_X_POS
                y = 250.f;   // I3_WIPE_TOWER_DEFAULT_Y_POS
            }
#ifdef ENGINE_ORCA
            const float tower_margin = WIPE_TOWER_MARGIN + arrange_opt(config, "prime_tower_width")->getFloat();
#else
            const float tower_margin = WIPE_TOWER_MARGIN;
#endif
            if (x < tower_margin) x = tower_margin;
            if (y < tower_margin) y = tower_margin;
            ConfigOptionFloat wt_x_opt(x), wt_y_opt(y);
            auto* wipe_x_option = config.option<ConfigOptionFloats>("wipe_tower_x", true);
            auto* wipe_y_option = config.option<ConfigOptionFloats>("wipe_tower_y", true);
            for (int bedid = 0; bedid < 36; bedid++) {   // MAX_PLATE_COUNT
                const int plate_index_valid = std::min(bedid, file_plates - 1);
                if (bedid < file_plates) {
                    wipe_x_option->set_at(&wt_x_opt, plate_index_valid, 0);
                    wipe_y_option->set_at(&wt_y_opt, plate_index_valid, 0);
                }
                Vec2d pos;
                ArrangePolygon wipe_tower_ap = estimate_tower_polygon_global(
                    model, config, plate_index_valid, filament_count, (int)plate_size.x(), (int)plate_size.y(), pos);
                if (bedid < file_plates) {
                    wt_x_opt.value = pos.x();
                    wt_y_opt.value = pos.y();
                    wipe_x_option->set_at(&wt_x_opt, plate_index_valid, 0);
                    wipe_y_option->set_at(&wt_y_opt, plate_index_valid, 0);
                }
                wipe_tower_ap.bed_idx = bedid;
                unselected.emplace_back(std::move(wipe_tower_ap));
            }
        }

        // Step 2 (BambuStudio.cpp 5886-5915; OrcaSlicer.cpp 5142-5172): the
        // same settings as the one-plate arrange (arrange_on_bed).
        params.allow_rotations                     = g_arrange_switches.allow_rotations;
        params.allow_multi_materials_on_same_plate = g_arrange_switches.allow_multicolor_oneplate;
        params.avoid_extrusion_cali_region         = g_arrange_switches.avoid_extrusion_cali_region;
        params.clearance_height_to_rod             = arrange_opt(config, "extruder_clearance_height_to_rod")->getFloat();
        params.clearance_height_to_lid             = arrange_opt(config, "extruder_clearance_height_to_lid")->getFloat();
#ifdef ENGINE_ORCA
        params.clearance_radius                    = arrange_opt(config, "extruder_clearance_radius")->getFloat();
#else
        params.cleareance_radius                   = arrange_opt(config, "extruder_clearance_max_radius")->getFloat();
#endif
        params.printable_height                    = arrange_opt(config, "printable_height")->getFloat();
        params.min_obj_distance = 0;
        if (params.is_seq_print) {
            params.bed_shrink_x = BED_SHRINK_SEQ_PRINT;
            params.bed_shrink_y = BED_SHRINK_SEQ_PRINT;
        }
        if (auto printer_structure_opt = config.option<ConfigOptionEnum<PrinterStructure>>("printer_structure"))
            params.align_to_y_axis = (printer_structure_opt->value == PrinterStructure::psI3);
#ifdef ENGINE_ORCA
        update_arrange_params(params, &config, selected);
        update_selected_items_inflation(selected, &config, params);
        update_unselected_items_inflation(unselected, &config, params);
        update_selected_items_axis_align(selected, &config, params);
        Points beds = get_shrink_bedpts(&config, params);
#else
        update_arrange_params(params, config, selected);
        update_selected_items_inflation(selected, config, params);
        update_unselected_items_inflation(unselected, config, params);
        Points beds = get_shrink_bedpts(config, params);
#endif
        arrange_add_exclude_areas(params.excluded_regions, config, scale_(1));

        // Step 3.
        arrangement::arrange(selected, unselected, beds, params);
        arrangement::arrange(unprintable, {}, beds, params);

        // Step 4 (BambuStudio.cpp 5946-6006; OrcaSlicer.cpp 5189-5262): each
        // selected item's bed past the locked plates, adding plates while the
        // bed is past the last one (postprocess_bed_index_for_selected and
        // create_plate, at most MAX_PLATE_COUNT, Bambu PartPlate.cpp 6009-6051,
        // 4834-4842).
        const auto create_plate = [&]() -> int {
            if (plate_count >= 36)
                return -1;
            locked.push_back(false);
            return plate_count++;
        };
        int bed_idx_max = 0;
        for (ArrangePolygon& ap : selected) {
            if (ap.bed_idx != -1) {
                bool placed = false;
                for (int i = 0; i < plate_count; ++i) {
                    if (locked[i]) {
                        ap.bed_idx += 1;
                    } else if (ap.bed_idx <= i) {
                        placed = true;
                        break;
                    }
                }
                if (!placed) {
                    int plate_index = create_plate();
                    while (plate_index != -1) {
                        if (ap.bed_idx <= plate_index)
                            break;
                        plate_index = create_plate();
                    }
                }
            }
            bed_idx_max = std::max(ap.bed_idx, bed_idx_max);
        }
        // postprocess_arrange_polygon (Bambu PartPlate.cpp 6107-6129): onto
        // its plate in the grid of the plates there are now; an item the
        // arrange could not place goes past the last plate.
        const int cols = plan_plate_cols(plate_count);
        const auto to_grid = [&](ArrangePolygon& ap) {
            const Vec2d origin = plan_plate_origin(plate_size, ap.bed_idx, cols);
            ap.translation(X) += scaled<double>(origin.x());
            ap.translation(Y) += scaled<double>(origin.y());
        };
        for (ArrangePolygon& ap : locked_aps) {
            bed_idx_max = std::max(ap.bed_idx, bed_idx_max);
            to_grid(ap);
            ap.apply();
        }
        for (ArrangePolygon& ap : selected) {
            if (ap.bed_idx == -1) {
                ap.bed_idx = plate_count;
                const BoundingBox apbox = get_extents(ap.transformed_poly());
                const auto apbox_size = apbox.size();
                ap.translation(X) = 0.5 * apbox_size[0];
                ap.translation(Y) = scaled<double>(static_cast<double>((int)plate_size.y())) - 0.5 * apbox_size[1];
            }
            to_grid(ap);
            ap.apply();
        }
        for (ArrangePolygon& ap : unprintable) {
            ap.bed_idx = bed_idx_max + 1;
            to_grid(ap);
            ap.apply();
        }
        // rebuild_plates_after_arrangement (Bambu PartPlate.cpp 6803-6860;
        // Orca 6005-6037): the objects in arrange order (stable here, where the
        // official's std::sort leaves equal orders in any order).
        std::stable_sort(model.objects.begin(), model.objects.end(), [](const ModelObject* a, const ModelObject* b) {
            const int ao = a->instances.empty() ? 0 : a->instances[0]->arrange_order;
            const int bo = b->instances.empty() ? 0 : b->instances[0]->arrange_order;
            return ao < bo;
        });
        plan.arranged = true;
    }

    // reload_all_objects (Bambu PartPlate.cpp 5668-5710): each instance on
    // the first plate its box meets.
    std::vector<std::vector<std::pair<int, int>>> members(plate_count);
    int off_plates = 0;
    for (size_t oi = 0; oi < model.objects.size(); ++oi)
        for (size_t ii = 0; ii < model.objects[oi]->instances.size(); ++ii) {
            const BoundingBoxf3 box = model.objects[oi]->instance_convex_hull_bounding_box(ii);
            int k = 0;
            for (; k < plate_count; ++k)
                if (plate_box(k, plate_count).intersects(box)) {
                    members[k].emplace_back(int(oi), int(ii));
                    break;
                }
            if (k == plate_count)
                ++off_plates;
        }
    // Each instance's place on its plate, which a plate keeps when the plates
    // before it go (delete_plate moves a plate with its objects).
    std::vector<std::vector<Vec3d>> on_plate_offset(plate_count);
    for (int k = 0; k < plate_count; ++k) {
        const Vec2d origin = plan_plate_origin(plate_size, k, plan_plate_cols(plate_count));
        for (const auto& [oi, ii] : members[k])
            on_plate_offset[k].push_back(model.objects[oi]->instances[ii]->get_offset() - Vec3d(origin.x(), origin.y(), 0.));
    }
    if (need_arrange) {
        // Recycle the empty plates (Bambu PartPlate.cpp 6837-6858: every empty
        // plate after the first; Orca 6018-6037: from the last plate back to the
        // first that holds a printable object, past locked ones).
        const auto printable_on = [&](int k) {
            for (const auto& [oi, ii] : members[k])
                if (model.objects[oi]->instances[ii]->printable)
                    return true;
            return false;
        };
        for (int i = plate_count - 1; i > 0; --i) {
            if (members[i].empty() || !printable_on(i)) {
                off_plates += (int)members[i].size();
                members.erase(members.begin() + i);
                on_plate_offset.erase(on_plate_offset.begin() + i);
                locked.erase(locked.begin() + i);
            } else if (locked[i]) {
                continue;
            } else {
#ifdef ENGINE_ORCA
                break;
#else
                continue;
#endif
            }
        }
        plate_count = (int)members.size();
    }
    plan.origins.clear();
    plan.members.assign(plate_count, {});
    const int final_cols = plan_plate_cols(plate_count);
    for (int k = 0; k < plate_count; ++k) {
        const Vec2d origin = plan_plate_origin(plate_size, k, final_cols);
        plan.origins.push_back(origin);
        for (size_t m = 0; m < members[k].size(); ++m) {
            const auto& [oi, ii] = members[k][m];
            ModelInstance* inst = model.objects[oi]->instances[ii];
            inst->set_offset(on_plate_offset[k][m] + Vec3d(origin.x(), origin.y(), 0.));
            plan.members[k].push_back(inst);
        }
    }
    plan.file_plate_count = file_plates;
    plan.plate_count = plate_count;
    plan.off_plates = off_plates;
    if (plan.arranged && filament_count > 0) {
        if (const auto* wx = config.option<ConfigOptionFloats>("wipe_tower_x")) plan.tower_x = wx->values;
        if (const auto* wy = config.option<ConfigOptionFloats>("wipe_tower_y")) plan.tower_y = wy->values;
    }
    plan.done = true;
    return true;
}

/// The event for a project 3MF's plan: the plates the run slices and the
/// objects on each.
static void emit_project_plan_event(const ProjectPlan& plan, bool model_files = true) {
    json plates = json::array();
    for (int k = 0; k < plan.plate_count; ++k) {
        json objects = json::array();
        for (const Slic3r::ModelInstance* inst : plan.members[k])
            objects.push_back(inst->get_object()->name);
        plates.push_back(json{{"plate_id", k + 1}, {"objects", objects}});
    }
    std::string message = plan.arranged && !model_files
        ? "The project's plates were arranged together over " + std::to_string(plan.plate_count) +
              " plate(s), as the official command line arranges them for --slice 0 --arrange 1"
        : plan.arranged
        ? "The project and the model files after it were arranged together over " + std::to_string(plan.plate_count) +
              " plate(s), as the official command line arranges them"
        : "The model files after the project keep their places (--arrange 0) over " +
              std::to_string(plan.plate_count) + " plate(s)";
    if (plan.off_plates > 0)
        message += "; " + std::to_string(plan.off_plates) + " object(s) are on no plate and are not printed";
    emit_event({{"event","arranged"}, {"tag", plan.arranged ? "ProjectArrangedAcrossPlates" : "ProjectPlatesByPosition"},
                {"plate_count", plan.plate_count}, {"file_plate_count", plan.file_plate_count},
                {"off_plates", plan.off_plates}, {"plates", plates}, {"message", message}});
}

/// The command-line setting overrides (--layer-height, --fill-density, ...)
/// over `config`. The slice reports a rejected value; the --export-3mf
/// rebuild of the same settings stays quiet (the slice already said it).
static void apply_command_line_overrides(Slic3r::DynamicPrintConfig& config,
                                         const std::map<std::string, std::string>& overrides,
                                         bool report_rejections) {
    for (const auto& [key, value] : overrides) {
        try {
            if (key == "layer_height") {
                config.set_key_value(key, new Slic3r::ConfigOptionFloat(std::stof(value)));
            } else if (key == "nozzle_diameter") {
                // nozzle_diameter is a list (one value per extruder); a single
                // Float there fails the type check of every later read
                // (Config.hpp 403). The printer's extruder count is that
                // list's size (support_different_extruders), so the value
                // goes into every entry and the list keeps its size: a
                // two-nozzle printer stays two nozzles. A value that is
                // already a list is taken as given.
                (void)std::stof(value);   // a bad number is refused as before
                size_t extruders = 1;
                if (const auto* nozzles = dynamic_cast<const Slic3r::ConfigOptionVectorBase*>(config.option(key)))
                    extruders = std::max<size_t>(1, nozzles->size());
                std::string values = value;
                for (size_t i = 1; i < extruders && value.find(',') == std::string::npos; ++i)
                    values += "," + value;
                config.set_deserialize_strict(key, values);
            } else if (key == "fill_density") {
                config.set_key_value(key, new Slic3r::ConfigOptionPercent(std::stoi(value)));
                config.set_key_value("sparse_infill_density", new Slic3r::ConfigOptionPercent(std::stoi(value)));
            } else if (key == "perimeters") {
                // Neither engine has a "perimeters" setting; the wall count is
                // wall_loops (PrintConfig.cpp 4383 at 5873b5f, 4918 at 31f6803).
                config.set_key_value("wall_loops", new Slic3r::ConfigOptionInt(std::stoi(value)));
            } else if (key == "nozzle_temperature" || key == "bed_temperature") {
                const int v = std::stoi(value);
                // A setting the engine defines keeps its own type (BambuStudio's
                // nozzle_temperature is nullable, and Model::setExtruderParams
                // reads it as such); a key it does not define is kept as before.
                if (Slic3r::print_config_def.get(key) != nullptr)
                    config.set_deserialize_strict(key, std::to_string(v));
                else
                    config.set_key_value(key, new Slic3r::ConfigOptionInts({v}));
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

/// The staging folders this run made: the 3MF load's backup folder and the
/// --export-3mf one. Both are handed to the engine, which asks the backup
/// manager to remove the folder a model ends with (Model.cpp ~Model ->
/// remove_backup), but that removal runs as a UI task the command line never
/// drains (_BBS_Backup_Manager::process_ui_task, bbs_3mf.cpp 8714-8786 at
/// 31f6803), so the run removes them itself as the process ends. The list is
/// a member, not a function-local static: this object's destructor must
/// outlive it. A crash or a kill leaves the folders, as it leaves any temp
/// file.
struct RunStagingFolders {
    std::vector<std::string> paths;

    /// Registers `path`.
    std::string add(std::string path) {
        paths.push_back(path);
        return path;
    }

    ~RunStagingFolders() {
        for (const std::string& path : paths) {
            boost::system::error_code ec;
            boost::filesystem::remove_all(path, ec);
        }
    }
};
static RunStagingFolders run_staging;

/// The folder this run's 3MF loads stage their temporary files in. The
/// engine's loader extracts Metadata/project_settings.config to
/// <backup>/_temp_3.config and the embedded presets to <backup>/_temp_2.config
/// and then parses them back (bbs_3mf.cpp
/// _extract_project_config_from_archive 2729 and
/// _extract_project_embedded_presets_from_archive 2758 at BambuStudio
/// 5873b5f; the same functions at 2636 and 2665 in OrcaSlicer 31f6803). One
/// folder for the whole run: the shared $TMPDIR/slicer_cli_backup name let
/// parallel runs read each other's half-written file ("load_from_json: parse
/// /tmp/slicer_cli_backup/_temp_3.config got a parse_error"), and a folder per
/// load would litter the temp dir. "detach" first, so set_backup_path does not
/// remove the folder it replaces (Model::set_backup_path, Model.cpp 1165-1179
/// / 1028-1042) — as f747602 does for the --export-3mf staging folder.
static const std::string& run_backup_path() {
    static const std::string path =
        run_staging.add((boost::filesystem::temp_directory_path() /
                         boost::filesystem::unique_path("slicer_cli_load-%%%%%%%%")).string());
    return path;
}

/// Gives `model` this run's own backup folder (run_backup_path).
static void set_run_backup_path(Slic3r::Model& model) {
    model.set_backup_path("detach");
    model.set_backup_path(run_backup_path());
}

/// One plate's objects, loaded alone from the project 3MF, for
/// --downward-check: the official sizes every plate of the project
/// (check_plate_wipe_tower per plate, BambuStudio.cpp 4645-4658; OrcaSlicer.cpp
/// 3986-3996), whichever plate is sliced. Plate 0 is the whole project;
/// `plate_members`, when given, receives each plate's (object, instance)
/// pairs in the loaded model (PlateData::objects_and_instances), and
/// `file_config` the project's settings as the file holds them.
static bool load_plate_objects(const std::string& input_file, int plate_id, Slic3r::Model& model,
                               std::vector<std::vector<std::pair<int, int>>>* plate_members = nullptr,
                               Slic3r::DynamicPrintConfig* file_config = nullptr) {
    using namespace Slic3r;
    DynamicPrintConfig scratch;
    scratch.apply(FullPrintConfig::defaults(), true);
    ConfigSubstitutionContext subst(ForwardCompatibilitySubstitutionRule::Enable);
    PlateDataPtrs plates;
    std::vector<Preset*> presets;
    bool is_bbl = false;
    Semver version;
    std::string load_path = input_file;
#ifdef ENGINE_BAMBU
    PercentRewrite percent_rewrite;
    rewrite_percent_line_widths(input_file, percent_rewrite);
    if (!percent_rewrite.refusal.empty())
        return false;
    if (percent_rewrite.changed)
        load_path = percent_rewrite.temp_path;
#endif
    const auto strategy = LoadStrategy::LoadModel | LoadStrategy::LoadConfig | LoadStrategy::AddDefaultInstances;
    // A writable backup folder of this run's own, as the slice's own load
    // sets (never the loader's default /bamboo_model, and never one shared
    // with another run: run_backup_path).
    set_run_backup_path(model);
    bool loaded = false;
    try {
#ifdef ENGINE_ORCA
        bool is_orca = false;
        loaded = load_bbs_3mf(load_path.c_str(), &scratch, &subst, &model, &plates, &presets, &is_bbl, &is_orca,
                              &version, nullptr, strategy, nullptr, plate_id);
#else
        loaded = load_bbs_3mf(load_path.c_str(), &scratch, &subst, &model, &plates, &presets, &is_bbl,
                              &version, nullptr, strategy, nullptr, plate_id);
#endif
    } catch (const std::exception&) {
        loaded = false;
    }
    if (plate_members) {
        // The loader keeps a plate's instances only as obj_inst_map and
        // stamps each loaded instance with its identify_id (bbs_3mf.cpp
        // 2411-2445 at 5873b5f); an instance tied to no plate goes to the
        // first plate its box meets (PartPlateList::reload_all_objects ->
        // PartPlate::intersect_instance, PartPlate.cpp 5668-5710, 2651-2677),
        // as export_sliced_3mf finds them.
        plate_members->assign(plates.size(), {});
        std::set<std::pair<int, int>> placed;
        for (size_t k = 0; loaded && k < plates.size(); ++k) {
            if (plates[k] == nullptr) continue;
            std::set<int> identify_ids;
            for (const auto& entry : plates[k]->obj_inst_map)
                if (entry.second.second > 0) identify_ids.insert(entry.second.second);
            for (size_t oi = 0; oi < model.objects.size(); ++oi)
                for (size_t ii = 0; ii < model.objects[oi]->instances.size(); ++ii) {
                    const int id = model.objects[oi]->instances[ii]->loaded_id;
                    if (id > 0 && identify_ids.count(id) && placed.insert({int(oi), int(ii)}).second)
                        (*plate_members)[k].emplace_back(int(oi), int(ii));
                }
        }
        for (size_t oi = 0; loaded && oi < model.objects.size(); ++oi)
            for (size_t ii = 0; ii < model.objects[oi]->instances.size(); ++ii) {
                if (placed.count({int(oi), int(ii)})) continue;
                const BoundingBoxf3 box = model.objects[oi]->instance_convex_hull_bounding_box(ii);
                for (size_t k = 0; k < plates.size(); ++k)
                    if (plate_scene_box(scratch, version, int(k), int(plates.size())).intersects(box)) {
                        (*plate_members)[k].emplace_back(int(oi), int(ii));
                        break;
                    }
            }
    }
    if (file_config)
        *file_config = scratch;
    release_PlateData_list(plates);
    for (Preset* p : presets)
        delete p;
    return loaded;
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

/// One plate of the run: its grid origin in the scene and its instances in
/// the run's model (PartPlate's origin and obj_to_instance_set). `members`
/// empty with `all` set: every instance of the model is on this plate (a
/// load of one plate, or one plate of model files).
struct RunPlate {
    Slic3r::Vec2d origin = Slic3r::Vec2d::Zero();
    bool all = true;
    std::vector<Slic3r::ModelInstance*> members;
    bool is_member(const Slic3r::ModelInstance* inst) const {
        return all || std::find(members.begin(), members.end(), inst) != members.end();
    }
};

/// The run's one model and one settings base, as the official CLI keeps them:
/// every input is loaded once into one model (BambuStudio.cpp 1855-2163,
/// 3947-3963; OrcaSlicer.cpp 1537-1810, 3441-3457), every object at its
/// place in the scene, and m_print_config is built once from the file, the
/// profiles, the settings merge and the flags (BambuStudio.cpp 4091;
/// OrcaSlicer.cpp 3542). Each plate is then sliced from that model with the
/// plate's origin on its Print (PartPlate::get_print, set_plate_origin;
/// BambuStudio.cpp 6506; OrcaSlicer.cpp 5623) and a copy of the settings with
/// the plate's own laid on top (BambuStudio.cpp 6902-6904; OrcaSlicer.cpp
/// 5905-5907).
struct RunState {
    bool loaded = false;      // the inputs are in `model` and `base`
    bool prepared = false;    // the steps before the plate loop ran
    int prepared_plate = 0;   // the plate the run-level steps ran for (0: every plate)
    Slic3r::Model model;
    // m_print_config: the project's (or the profiles') settings with the
    // settings merge and the flags applied, before any plate's own.
    Slic3r::DynamicPrintConfig base;
    // The settings the steps before the plate loop read and changed (the
    // arrange's tower): `prepared_plate`'s own, or the project's for every plate.
    Slic3r::DynamicPrintConfig run_config;
    Slic3r::PlateDataPtrs plate_data;
    ~RunState() { Slic3r::release_PlateData_list(plate_data); }
    std::vector<RunPlate> plates;   // index = plate id - 1
    ProjectPlan plan;
    // What the load and the run-level steps record for every plate's result.
    PlateOutcome carry;
    // Load-time facts every later step reads.
    bool is_bbl_3mf = false;
    Slic3r::Semver file_version;
    std::vector<Slic3r::Vec2d> plate_origins;   // the file's grid, for --downward-check
    std::optional<Slic3r::DynamicPrintConfig> plan_file_config;
    Slic3r::Semver plan_file_version;
    int plan_file_filaments = 0;
    slicer_cli::ProjectFacts project_facts;
    std::vector<Slic3r::Preset> project_presets_kept;
    Slic3r::DynamicPrintConfig extra;
    bool explicit_config_supplied_nozzle_map = false;
    slicer_cli::SettingsMerge settings_merge;
    std::string merged_print_sequence;
    int shrink_to_new_bed = 0;
    size_t project_filaments = 0;
    // --pipe: steps 2 and 3 go out once per run, before any plate is
    // checked (BambuStudio.cpp 4923-4926 before the transforms, 6455-6458 as
    // the slice action starts, after pre_check is set at 6441; OrcaSlicer.cpp
    // 4185 and 5580, after 5565).
    bool pipe_files_loaded_sent = false;
    bool pipe_prepare_sent = false;
    // The model actions run once per run (BambuStudio.cpp 6336; OrcaSlicer.cpp 5470).
    bool actions_done = false;
    // Plate `plate_id` of the run; a run of one plate (model files, a 3MF
    // without plates) answers with that plate whatever the id.
    RunPlate& plate(int plate_id) {
        if (plates.empty())
            plates.assign(1, RunPlate());
        if (plates.size() == 1 || plate_id < 1 || plate_id > (int)plates.size())
            return plates.front();
        return plates[plate_id - 1];
    }
};

/// Events held while it lives (diagnostics::held_events): dropped when the
/// step they come from passes (drop()), sent when it stops early.
class HeldEventsScope {
public:
    explicit HeldEventsScope(bool active) : m_active(active) {
        if (!m_active) return;
        m_prev = slicer_cli::diagnostics::held_events;
        slicer_cli::diagnostics::held_events = &m_held;
    }
    void drop() { m_held.clear(); }
    ~HeldEventsScope() {
        if (!m_active) return;
        slicer_cli::diagnostics::held_events = m_prev;
        for (const std::string& line : m_held)
            slicer_cli::diagnostics::write_event(line);
    }
    HeldEventsScope(const HeldEventsScope&) = delete;
    HeldEventsScope& operator=(const HeldEventsScope&) = delete;
private:
    bool m_active = false;
    std::vector<std::string>* m_prev = nullptr;
    std::vector<std::string> m_held;
};

static void pipe_files_loaded(RunState& rs) {
    if (!slicer_cli::pipe_started() || rs.pipe_files_loaded_sent)
        return;
    rs.pipe_files_loaded_sent = true;
    slicer_cli::pipe_update(2, "Loading files finished");
}
static void pipe_prepare_slicing(RunState& rs) {
    pipe_files_loaded(rs);
    if (!slicer_cli::pipe_started() || rs.pipe_prepare_sent)
        return;
    rs.pipe_prepare_sent = true;
    slicer_cli::pipe_update(3, "Prepare slicing");
}

/// One plate of the run's model in its own frame, for the steps the official
/// CLI runs on one plate in the plate's coordinates (the move to a new bed,
/// BambuStudio.cpp 4487-4533; the one-plate arrange, 5730-6176, with
/// PartPlateList::preprocess_arrange_polygon taking the plate origin off and
/// postprocess_arrange_polygon putting it back, PartPlate.cpp 5831-5871,
/// 6107-6129; the bed check against the plate's shape, 6527): while the
/// scope lives the model holds only this plate's instances, each moved by
/// minus the plate's origin. On exit every instance goes back to the scene
/// with the move the step made (an instance the step did not move keeps its
/// exact offset), the other instances come back, and instances or objects
/// the step added (--repetitions) are kept, on the plate. Instances are known
/// by their ObjectID, which a copy of the model keeps (Model::assign_copy), so
/// a step that puts back a saved copy of the model (the repetitions loop)
/// finds its instances again. Only a scope that holds the whole model may let
/// its step replace the model's objects.
class PlateScope {
public:
    PlateScope(Slic3r::Model& model, const RunPlate& plate) : m_model(model), m_origin(plate.origin) {
        m_whole = plate.all;
        if (!m_whole) {
            m_whole = true;
            for (const Slic3r::ModelObject* object : model.objects)
                for (const Slic3r::ModelInstance* inst : object->instances)
                    m_whole = m_whole && plate.is_member(inst);
        }
        if (!m_whole) {
            m_all_objects = model.objects;
            std::vector<Slic3r::ModelObject*> kept;
            for (Slic3r::ModelObject* object : model.objects) {
                std::vector<Slic3r::ModelInstance*> mine;
                for (Slic3r::ModelInstance* inst : object->instances)
                    if (plate.is_member(inst))
                        mine.push_back(inst);
                if (mine.empty())
                    continue;
                m_instances[object] = object->instances;
                object->instances = mine;
                kept.push_back(object);
            }
            model.objects = kept;
        }
        for (Slic3r::ModelObject* object : model.objects)
            for (Slic3r::ModelInstance* inst : object->instances) {
                const Slic3r::Vec3d scene = inst->get_offset();
                const Slic3r::Vec3d local = scene - Slic3r::Vec3d(m_origin.x(), m_origin.y(), 0.);
                m_scene[inst->id().id] = scene;
                m_local[inst->id().id] = local;
                m_members.insert(inst);
                if (m_origin.x() != 0. || m_origin.y() != 0.)
                    inst->set_offset(local);
            }
    }
    ~PlateScope() {
        const Slic3r::Vec3d origin(m_origin.x(), m_origin.y(), 0.);
        for (Slic3r::ModelObject* object : m_model.objects) {
            for (Slic3r::ModelInstance* inst : object->instances) {
                const auto it = m_scene.find(inst->id().id);
                if (it == m_scene.end())
                    inst->set_offset(inst->get_offset() + origin);   // added in the scope
                else if (inst->get_offset() != m_local[it->first])
                    inst->set_offset(it->second + (inst->get_offset() - m_local[it->first]));
                else
                    inst->set_offset(it->second);
            }
            object->invalidate_bounding_box();
        }
        if (m_whole)
            return;
        // The other plates' instances back in their places, the plate's own
        // (and any the step added) where they were.
        for (auto& [object, all] : m_instances) {
            std::vector<Slic3r::ModelInstance*> now = object->instances;
            std::vector<Slic3r::ModelInstance*> restored;
            for (Slic3r::ModelInstance* inst : all)
                if (!m_members.count(inst) || std::find(now.begin(), now.end(), inst) != now.end())
                    restored.push_back(inst);
            for (Slic3r::ModelInstance* inst : now)
                if (std::find(all.begin(), all.end(), inst) == all.end())
                    restored.push_back(inst);
            object->instances = restored;
        }
        std::vector<Slic3r::ModelObject*> now = m_model.objects;
        std::vector<Slic3r::ModelObject*> restored = m_all_objects;
        for (Slic3r::ModelObject* object : now)
            if (std::find(m_all_objects.begin(), m_all_objects.end(), object) == m_all_objects.end())
                restored.push_back(object);
        m_model.objects = restored;
    }
    PlateScope(const PlateScope&) = delete;
    PlateScope& operator=(const PlateScope&) = delete;

private:
    Slic3r::Model& m_model;
    Slic3r::Vec2d m_origin;
    bool m_whole = true;
    std::vector<Slic3r::ModelObject*> m_all_objects;
    std::map<Slic3r::ModelObject*, std::vector<Slic3r::ModelInstance*>> m_instances;
    std::set<const Slic3r::ModelInstance*> m_members;
    std::map<size_t, Slic3r::Vec3d> m_scene, m_local;
};

/// --skip-objects on the placed objects (BambuStudio.cpp 6541-6581,
/// CLI_NO_SUITABLE_OBJECTS_AFTER_SKIP when nothing is left; OrcaSlicer.cpp
/// 5645-5720): inside the slice action, after the model actions. Its caller
/// runs --ensure-on-bed first, before the actions (BambuStudio.cpp 6188-6194
/// then the action loop at 6336; OrcaSlicer.cpp 5443-5455 then 5470).
/// --mtcpp follows the bed check (plate_triangle_limit).
static bool plate_object_steps(const CliOptions& o, Slic3r::Model& model, int plate, PlateOutcome& outcome) {
    std::vector<int> skipped;
    if (slicer_cli::apply_skip_objects(o, model, skipped) < 0) {
        const std::string why = "Every object on plate " + std::to_string(plate) + " is in --skip-objects.";
        std::cerr << "Error: " << why << "\n";
        set_outcome_failure(outcome, CLI_NO_SUITABLE_OBJECTS_AFTER_SKIP, why);
        return false;
    }
    outcome.skipped_objects = skipped;
    return true;
}

/// --mtcpp, once the bed check has marked each instance inside or not
/// (BambuStudio.cpp 6541-6597; OrcaSlicer.cpp 5657-5720).
static bool plate_triangle_limit(const CliOptions& o, const Slic3r::Model& model, int plate, PlateOutcome& outcome) {
    const slicer_cli::StepResult tri = slicer_cli::check_triangle_limit(o, model, plate);
    if (tri.code != 0) {
        std::cerr << "Error: " << tri.message << "\n";
        set_outcome_failure(outcome, tri.code, tri.message);
        return false;
    }
    return true;
}

/// One slice of one plate from the run's one model and settings (RunState):
/// the first call of a run loads the inputs and builds the settings base
/// (`rs.loaded`) and runs the steps the official CLI runs once before its
/// plate loop (`rs.prepared`: the transforms, the arrange, the actions before
/// --slice); every call then lays the plate's own settings over a copy of the
/// base and slices the plate with its origin, as the official plate loop does
/// (BambuStudio.cpp 6437-7260; OrcaSlicer.cpp 5561-6200). The default call
/// (`file [--plate N] -o out.gcode`) is one call; `--slice` makes a prepare
/// call (outcome.prepare_pass, which stops before the plate checks) and then
/// one call per plate. The return value is the process exit code of the
/// default call; `outcome` carries what `--slice` mode reports on top.
static int slice_one_plate(const CliOptions& o, Slic3r::Calib_Params& calib_params, RunState& rs,
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
    // The first call of the run loads and prepares; the others slice only.
    const bool first_call = !rs.prepared;
    if (!first_call) {
        // What the load and the run-level steps recorded, for this plate's result.
        PlateOutcome carried = rs.carry;
        carried.plate_id    = outcome.plate_id;
        carried.plate_index = outcome.plate_index;
        carried.plate_count = outcome.plate_count;
        carried.gcode_path  = outcome.gcode_path;
        carried.pre_check   = outcome.pre_check;
        if (plate_id != rs.prepared_plate) {
            // Another plate than the one the run-level steps ran for: its own
            // move to the new bed and none of that plate's arrange results.
            carried.derived_filament_map.clear();
            carried.derived_filament_map_mode.clear();
        }
        outcome = carried;
    }
    try {
        // Create configuration BEFORE model loading so load_bbs_3mf can populate it
        // from the embedded Metadata/project_settings.config JSON.
        Slic3r::DynamicPrintConfig config;
        Slic3r::Model& model = rs.model;
        bool& is_bbl_3mf = rs.is_bbl_3mf;
        // Values this engine has no word for (unknown enum values), reported
        // together with the out-of-range values at the value check below.
        std::vector<std::string> unknown_values;
        json unknown_items = json::array();
        Slic3r::PlateDataPtrs& plate_data = rs.plate_data;
        // Each plate's grid origin in the project (plate_grid_origin), for
        // the plates --downward-check loads alone; empty when not a project.
        std::vector<Slic3r::Vec2d>& plate_origins = rs.plate_origins;
        // A project 3MF with model files after it (ProjectPlan): the file's own
        // settings and version, for the plate list's size, and its filament
        // count, the used filaments of the official's tower step.
        std::optional<Slic3r::DynamicPrintConfig>& plan_file_config = rs.plan_file_config;
        Slic3r::Semver& plan_file_version = rs.plan_file_version;
        int& plan_file_filaments = rs.plan_file_filaments;
        // What the official CLI reads from a Bambu-made 3MF, and its embedded
        // presets, for --load-settings and friends (cli_load_settings.cpp).
        slicer_cli::ProjectFacts& project_facts = rs.project_facts;
        std::vector<Slic3r::Preset>& project_presets_kept = rs.project_presets_kept;
        // The settings given as flags; the settings merge takes filament_colour
        // out of them once it has used it, as the official does.
        Slic3r::DynamicPrintConfig& extra = rs.extra;
#ifdef ENGINE_BAMBU
        bool& explicit_config_supplied_nozzle_map = rs.explicit_config_supplied_nozzle_map;
#endif

        // Geometry files (every model file this engine reads that is not a
        // project 3MF) load the official CLI's way, placed further down once
        // the bed is known. Any other file keeps the unsupported-format
        // refusal below.
        const bool geometry_input = !input_file.empty() && slicer_cli::is_loadable_model_file(input_file) &&
            (!input_is_3mf(input_file) ||
             slicer_cli::classify_3mf(input_file) == slicer_cli::ThreeMfKind::GeometryOnly);
        const bool geometry_only_3mf = geometry_input && input_is_3mf(input_file);
        // A project 3MF with model files after it, run as the official CLI
        // runs it (ProjectPlan): the run loads every plate (plate_id 0) and
        // the model files into the one model and arranges them; each plate's
        // slice then takes its instances from the plan.
        const bool trailing_port = !geometry_input && g_assemble == nullptr && trailing_models_port(o);
        // The plate filter of the one load (Model::read_from_file(...,
        // plate_to_slice), BambuStudio.cpp 1889; OrcaSlicer.cpp 1571): the
        // plate the run slices, or 0 for every plate.
        const int load_plate_id = plate_id;
        // --load-assemble-list: the plates' objects from the list built
        // before the plate loop, and every plate's own settings (plate_params)
        // as its PlateData config, where the official keeps them
        // (BambuStudio.cpp 1066-1078; OrcaSlicer.cpp 877-889).
        const bool assemble_input = g_assemble != nullptr;
        // The settings base the plate's own are laid over (below), and the
        // run-level part of the settings and of the model: once per run.
        std::optional<Slic3r::DynamicPrintConfig> project_settings;
        int& shrink_to_new_bed = rs.shrink_to_new_bed;
        slicer_cli::SettingsMerge& settings_merge = rs.settings_merge;
        std::string& merged_print_sequence = rs.merged_print_sequence;
#ifdef ENGINE_BAMBU
        size_t& project_filaments = rs.project_filaments;
#endif
        if (!rs.loaded) {
        std::cout << "\nConfiguring print settings...\n";
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
        extra = o.extra_config;

        // Load model
        std::cout << "Loading model: " << input_file << "\n";
        // Pre-set backup_path to a writable temp dir of this run's own so the
        // backup manager never touches the read-only /bamboo_model network
        // path, and parallel runs never share the loader's _temp_N.config
        // files (run_backup_path).
        set_run_backup_path(model);
        if (assemble_input) {
            // The list's plates the run slices: plate N, or every plate. Each
            // plate's objects go to its place in the plate grid once the bed
            // is known (below).
            if (plate_id > (int)g_assemble->plates.size() || plate_id < 0) {
                set_outcome_failure(outcome, CLI_INVALID_PARAMS, "The assemble list has no plate " + std::to_string(plate_id) + ".");
                return 1;
            }
            rs.plates.assign(g_assemble->plates.size(), RunPlate());
            for (size_t k = 0; k < g_assemble->plates.size(); ++k) {
                rs.plates[k].all = false;
                if (plate_id > 0 && int(k) != plate_id - 1)
                    continue;
                const size_t before = model.objects.size();
                for (const Slic3r::ModelObject* object : g_assemble->plates[k].loaded_obj_list)
                    model.add_object(*object);
                for (size_t oi = before; oi < model.objects.size(); ++oi)
                    for (Slic3r::ModelInstance* inst : model.objects[oi]->instances)
                        rs.plates[k].members.push_back(inst);
                emit_event({{"event","model_loaded"}, {"tag","AssemblePlateBuilt"}, {"plate_id", int(k) + 1},
                            {"objects", model.objects.size() - before},
                            {"message","Plate " + std::to_string(k + 1) + " of the assemble list holds " +
                                       std::to_string(model.objects.size() - before) + " object(s)"}});
            }
            for (size_t k = 0; k < g_assemble->plates.size(); ++k) {
                auto* pd = new Slic3r::PlateData();
                pd->plate_index = int(k);
                pd->plate_name  = g_assemble->plates[k].plate_name;
                pd->config      = g_assemble->plate_configs[k];
                plate_data.push_back(pd);
            }
        } else if (geometry_input) {
            const slicer_cli::ModelLoadResult loaded = slicer_cli::load_geometry_files(o, o.input_files, 0, model);
            if (loaded.code != 0 && o.input_files.size() == 1 && input_is_stl(input_file) &&
                (loaded.code == CLI_DATA_FILE_ERROR || loaded.code == CLI_FILE_NOTFOUND)) {
                // One STL that cannot be read: the refusal slicer-cli has
                // always given for it.
                std::cerr << "Failed to load STL file\n";
                set_outcome_failure(outcome, CLI_DATA_FILE_ERROR);
                return 1;
            }
            if (loaded.code != 0) {
                emit_event({{"event","load_error"}, {"tag","ModelLoadFailed"}, {"path", input_file},
                            {"message", loaded.message}});
                std::cerr << "Error: " << loaded.message << "\n";
                set_outcome_failure(outcome, loaded.code, loaded.message);
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
                load_plate_id   // 0 = all plates, >0 = specific plate
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
                load_plate_id   // 0 = all plates, >0 = specific plate
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
            // --normative-check, on by default in the official CLI for a
            // Bambu-made 3MF (BambuStudio.cpp 1948-1965; OrcaSlicer.cpp
            // 1609-1619). Part of --slice here, or asked for by name, so the
            // default call slices the same files it always has.
            if (is_bbl_3mf && (o.slice_mode || o.given_flag("normative_check"))) {
                const slicer_cli::StepResult nc = slicer_cli::normative_check(o, config);
                if (nc.code != 0) {
                    emit_event({{"event","config_refused"}, {"tag","NormativeCheckFailed"}, {"message", nc.message}});
                    std::cerr << "Error: " << nc.message << "\n";
                    set_outcome_failure(outcome, nc.code, nc.message);
                    return 1;
                }
            }
            // The settings exactly as the file states them, and whether they
            // were kept: the rebase below (Orca build) replaces `config` with
            // the preset-rebased settings, and the steps that describe the FILE
            // read these instead (read_project_facts).
            Slic3r::DynamicPrintConfig project_file_config;
            bool                       project_file_config_kept = false;
#ifdef ENGINE_ORCA
            // ── A Bambu Studio project is rebased onto its system presets ────
            // The desktop does not slice a project's settings as the file
            // states them. It loads this engine's system presets
            // (PresetBundle::load_presets, Plater.cpp 6040-6160 at 31f6803),
            // hands the file's settings to PresetBundle::load_config_model
            // (PresetBundle.hpp 388 -> PresetBundle.cpp 4239-4567), which loads
            // the project's printer, process and filament presets over the ones
            // this engine ships and, for every key the file does not list in
            // different_settings_to_system, takes the system preset's value
            // (Preset.cpp 2446-2457 load_external_preset ->
            // update_non_diff_values_to_base_config), and slices
            // PresetBundle::full_config(false) (Plater.cpp 7959-7963). Every
            // Bambu-only encoding of a key the maker did not touch -- the
            // 0-based filament indices, tree_support_wall_count -1,
            // raft_first_layer_expansion -1, ensure_vertical_shell_thickness
            // "enabled" -- therefore never reaches the engine. Upstream main's
            // CLI rebases the same way for its --uptodate path (OrcaSlicer
            // 94266c28, OrcaSlicer.cpp 3075-3108 and 2697-2726 on main).
            // wipe_tower_x/y are the plate's own positions and are put back
            // after the load (Plater.cpp 6380-6389).
            // Only a project Bambu Studio made is rebased; an OrcaSlicer
            // project already states this engine's own values, and every file
            // this product writes must keep slicing as it did. This scope test
            // is the one place the scope is decided.
            const bool bambu_made_project = is_bbl_3mf && !is_orca_3mf;
            if (bambu_made_project) {
                // The settings as the file states them, for read_project_facts
                // below: the facts describe the file (its printer, bed, plate
                // membership and the maker's own changed-key list), not the
                // preset-rebased settings this run slices with.
                project_file_config      = config;
                project_file_config_kept = true;
                Slic3r::DynamicPrintConfig rebased;
                std::string                unavailable;
                bool                       rebase = true;
                try {
                    // The profiles tree staged as <data_dir>/system, the same
                    // staging the named-preset path uses (load_system_presets):
                    // the bundle reads system presets from there and writes
                    // nothing back (load_external_preset saves no file).
                    const boost::filesystem::path staging = boost::filesystem::temp_directory_path() /
                                                            boost::filesystem::unique_path("slicer_cli_project-%%%%%%%%");
                    struct StagingCleanup {
                        boost::filesystem::path dir;
                        ~StagingCleanup() {
                            if (dir.empty())
                                return;
                            boost::system::error_code ignored;
                            boost::filesystem::remove_all(dir, ignored);
                        }
                    } staging_cleanup;
                    staging_cleanup.dir = staging;
                    Slic3r::PresetBundle bundle;
                    if (!load_system_presets(o.argv0, bundle, staging, unavailable))
                        rebase = false;
                    else if (!project_presets_shipped(bundle, config))
                        unavailable = "this engine ships no '" +
                                      config_id_text(config, "printer_settings_id") + "' / '" +
                                      config_id_text(config, "print_settings_id") + "' system preset";
                    else {
                        // The tower's plate positions, as the desktop keeps
                        // them across the load (Plater.cpp 6380-6389).
                        Slic3r::ConfigOptionFloats file_wipe_tower_x, file_wipe_tower_y;
                        bool                       has_wipe_tower_x = false, has_wipe_tower_y = false;
                        if (const auto* opt = config.option<Slic3r::ConfigOptionFloats>("wipe_tower_x")) {
                            file_wipe_tower_x = *opt;
                            has_wipe_tower_x  = true;
                        }
                        if (const auto* opt = config.option<Slic3r::ConfigOptionFloats>("wipe_tower_y")) {
                            file_wipe_tower_y = *opt;
                            has_wipe_tower_y  = true;
                        }
                        bundle.load_config_model(input_file, std::move(config), file_version);
                        rebased = bundle.full_config(false);
                        if (has_wipe_tower_x)
                            rebased.set_key_value("wipe_tower_x", new Slic3r::ConfigOptionFloats(file_wipe_tower_x));
                        if (has_wipe_tower_y)
                            rebased.set_key_value("wipe_tower_y", new Slic3r::ConfigOptionFloats(file_wipe_tower_y));
                    }
                } catch (const std::exception& e) {
                    unavailable = e.what();
                    rebase      = false;
                }
                if (rebase) {
                    config = std::move(rebased);
                    emit_event({{"event","config_normalized"},
                                {"tag","ProjectRebasedOnSystemPresets"},
                                {"printer_settings_id", config_id_text(config, "printer_settings_id")},
                                {"print_settings_id", config_id_text(config, "print_settings_id")},
                                {"message","The project was loaded over this engine's system presets, as the desktop loads it: "
                                           "every key the maker did not change takes the preset's value"}});
                } else {
                    // The engine ships no preset to load the project over, so
                    // the file's own settings are sliced, as before.
                    project_file_config.clear();
                    project_file_config_kept = false;
                    emit_event({{"event","config_normalized"},
                                {"tag","ProjectPresetRebaseSkipped"},
                                {"reason", unavailable},
                                {"message","The project keeps the values the file states: " + unavailable}});
                }
            }
            // ── A value whose BambuStudio meaning this engine cannot carry ───
            // The desktop's own load leaves this one alone (it warns and then
            // slices the -1), so the refusal says what to do instead. It is a
            // value the file states on purpose: for an unmodified key the
            // rebase above takes the system preset's value and there is
            // nothing to decide.
            if (bambu_made_project) {
                const json file_settings = project_settings_json(input_file);
                // raft_first_layer_expansion -1 is BambuStudio's automatic
                // expansion: 2 mm under normal support (BambuStudio
                // Support/SupportCommon.cpp 392-394 and 431-437 at 5873b5f),
                // which handle_legacy converts to this engine's 2.0, and a
                // per-branch moment brim under tree support, which this engine
                // has no value for at all (its own tree support shrinks the
                // raft area instead, Support/TreeSupport.cpp 1394).
                const std::string support_type = config_value_text(config, "support_type");
                if (boost::starts_with(support_type, "tree") &&
                    maker_changed_key(project_file_config, "raft_first_layer_expansion") &&
                    file_states_number(file_settings, "raft_first_layer_expansion", -1)) {
                    const std::string sentence =
                        "The project asks for raft_first_layer_expansion -1, BambuStudio's automatic "
                        "first-layer expansion, with tree support; this engine has no automatic value "
                        "for tree support. Set it to the expansion you want in mm (0 disables it), or "
                        "use normal support, whose automatic expansion is 2 mm.";
                    emit_event({{"event","config_refused"},
                                {"tag","RaftAutoExpansionUnsupportedForTreeSupport"},
                                {"opt_key", "raft_first_layer_expansion"},
                                {"value", -1},
                                {"support_type", support_type},
                                {"message", sentence}});
                    std::cerr << "Error: " << sentence << "\n";
                    set_outcome_failure(outcome, CLI_INVALID_VALUES_IN_3MF, sentence);
                    return 1;
                }
                // G4. BambuStudio's 0 means the support walls are infill-only
                // (BambuStudio Support/TreeSupport.cpp 1853-1855 at 5873b5f);
                // this engine reads 0 as automatic -- one wall where the area
                // is thin, extra walls where they are needed (OrcaSlicer
                // Support/TreeSupport.cpp 1623-1630) -- and cannot express
                // "always infill-only". Upstream keeps the value (434ff301 maps
                // only -1), and so does this run: the supports may come out
                // sturdier than the file asked for, which is said once, when
                // tree supports are in play.
                if (file_states_number(file_settings, "tree_support_wall_count", 0) &&
                    tree_support_enabled(config, model)) {
                    emit_event({{"event","warning"},
                                {"tag","TreeSupportWallCountZeroIsAuto"},
                                {"opt_key", "tree_support_wall_count"},
                                {"value", 0},
                                {"message","The project sets tree_support_wall_count 0, BambuStudio's "
                                           "infill-only support walls; this engine reads 0 as automatic, so the "
                                           "supports may come out sturdier. Set it to 1 or 2 to choose the wall "
                                           "count."}});
                }
            }
            // A Bambu Studio project states its filament indices from 0
            // (BambuStudio PrintConfig.cpp 4359-4361 at 5873b5f) while this
            // engine's ranges start at 1 (wall_filament PrintConfig.cpp
            // 4887-4894 at 31f6803), so every one of those projects would be
            // refused by the value check further down. The desktop reads the
            // same file: it only warns about the values (Plater.cpp 6259-6272,
            // "Invalid values found in the 3MF") and slices from
            // PresetBundle::full_fff_config, which walks the same three 1-based
            // keys back to 1 when they fall outside [1, N] and clamps
            // support_filament / support_interface_filament /
            // wipe_tower_filament to [0, N] (PresetBundle.cpp 4090-4105, with
            // N = filament_presets.size(), 3866). Ported here, on the settings
            // the run is sliced with and before that check: a key the file
            // states in its own app's numbering is not a value this engine
            // cannot read. Any other out-of-range value is left as it is and
            // still refuses, exactly as the desktop leaves it (it does not
            // clamp them either, e.g. a BambuStudio -1, its "auto").
            {
                const size_t filament_count = std::max<size_t>(1, project_filament_count(config));
                json reset = json::object();
                json clamped = json::object();
                for (const char* key : {"wall_filament", "sparse_infill_filament", "solid_infill_filament"}) {
                    auto* opt = dynamic_cast<Slic3r::ConfigOptionInt*>(config.option(key, false));
                    if (opt == nullptr || (opt->value >= 1 && opt->value <= int(filament_count)))
                        continue;
                    reset[key] = {{"from", opt->value}, {"to", 1}};
                    opt->value = 1;
                }
                for (const char* key : {"support_filament", "support_interface_filament", "wipe_tower_filament"}) {
                    auto* opt = dynamic_cast<Slic3r::ConfigOptionInt*>(config.option(key, false));
                    if (opt == nullptr)
                        continue;
                    const int value = std::min(std::max(opt->value, 0), int(filament_count));
                    if (value == opt->value)
                        continue;
                    clamped[key] = {{"from", opt->value}, {"to", value}};
                    opt->value = value;
                }
                if (!reset.empty() || !clamped.empty()) {
                    std::string names;
                    for (const auto& [key, change] : reset.items())
                        names += (names.empty() ? "" : ", ") + key + " " + change["from"].dump() + " -> " +
                                 change["to"].dump();
                    for (const auto& [key, change] : clamped.items())
                        names += (names.empty() ? "" : ", ") + key + " " + change["from"].dump() + " -> " +
                                 change["to"].dump();
                    emit_event({{"event","config_normalized"},
                                {"tag","FilamentIndexOutOfRangeReset"},
                                {"reset", reset},
                                {"clamped", clamped},
                                {"filament_count", int(filament_count)},
                                {"message","The 3mf states filament index value(s) outside this engine's range for its " +
                                           std::to_string(filament_count) + " filament(s): " + names +
                                           " (its indices start at 1; the desktop resets the same keys on load)"}});
                }
            }
#endif
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
            // On the Orca build the maker decides which number the file's
            // version is in (G2, see below): a Bambu Studio project is only
            // warned about, an OrcaSlicer project is refused.
            // The refusal sentence adds what the file says about its maker: a
            // Bambu Studio 02.07 project on the Orca 2.4 build is newer only in
            // the other app's numbering, and the binary to use is the maker's.
            if (!o.allow_newer_file && file_version.maj() + file_version.min() > 0) {
                std::string model_xml;
                std::map<std::string, std::string> meta;
                if (read_zip_member(input_file, "3D/3dmodel.model", model_xml))
                    meta = model_metadata(model_xml);
                const auto app = meta.find("Application");
                bool refuse = file_newer_than_engine(file_version);
#ifdef ENGINE_ORCA
                // G2. Which number a file's version is in depends on who wrote
                // it, and the desktop reads it that way (Plater.cpp 6094-6157
                // at 31f6803): a Bambu Studio project is compared with
                // SLIC3R_VERSION, the Bambu base this Orca release is built on
                // (version.inc, 02.06.00.51), and only warned about -- a
                // BambuStudio 02.07 project is not newer than OrcaSlicer 2.4,
                // it is newer than the base both apps share. An OrcaSlicer
                // project is compared with this engine's own version, and a
                // newer one is still refused, as the official CLI refuses it.
                const bool orca_made = meta.count("OrcaSlicer") != 0 ||
                                       (app != meta.end() && boost::starts_with(app->second, "OrcaSlicer-"));
                if (!orca_made && file_newer_than_version(file_version, SLIC3R_VERSION)) {
                    emit_event({{"event","warning"},
                                {"tag","FileNewerThanEngineBase"},
                                {"file_version", file_version.to_string()},
                                {"application", app != meta.end() ? json(app->second) : json(nullptr)},
                                {"base_version", SLIC3R_VERSION},
                                {"unknown_keys", outcome.unknown_settings},
                                {"message","The file is version " + file_version.to_string() +
                                           (app != meta.end() ? " (" + app->second + ")" : std::string()) +
                                           ", newer than the " + SLIC3R_VERSION +
                                           " Bambu base this engine is built on. It is sliced as it is; the "
                                           "setting(s) this engine has no definition for are ignored"}});
                    refuse = false;
                }
#endif
                if (refuse) {
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
            }
            // A setting whose value this engine has no meaning for is a
            // different print from the one the file states, so it is refused,
            // naming the setting, the value and the values this engine has.
            // Only enum
            // substitutions: those are the ones where the engine swapped in
            // its own default for a word it does not know. --allow-substitution
            // keeps the old behaviour (slice with the engine's substitute).
            // A project moved from OrcaSlicer to a Bambu printer can still
            // carry Orca words (ensure_all, rectilinear) that the default call
            // has always substituted; the default call keeps doing so, and the
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
                    // BambuStudio's "partial" skips one shell pass (BambuStudio
                    // PrintObject.cpp 1944, 1975 at 5873b5f) and this engine has
                    // no shell pass to skip, so the value is refused with the
                    // choice to make rather than the engine's generic sentence.
                    // "enabled"/"disabled" are the same meaning in another
                    // encoding and are converted by handle_legacy
                    // (libslic3r/orcaslicer/libslic3r/PrintConfig.cpp).
                    const bool partial_shell =
                        sub.opt_def->opt_key == "ensure_vertical_shell_thickness" && sub.old_value == "partial";
                    refused.push_back(partial_shell
                        ? "'ensure_vertical_shell_thickness' is 'partial', BambuStudio's skip-one-shell-pass mode, "
                          "which this engine has no value for (its values: " + allowed +
                          "); pick the shell coverage you want, e.g. ensure_all for BambuStudio's 'enabled'"
                        : "'" + sub.opt_def->opt_key + "' is '" + sub.old_value +
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
            if (is_bbl_3mf)
                // The file's own settings when the rebase kept them apart: the
                // facts describe the file (its printer, bed, plate membership
                // and the maker's changed-key list), and the printer change
                // reads them to find the system preset the file's process
                // inherits from. full_fff_config erases
                // different_settings_to_system and writes a new inherits_group,
                // so the rebased settings would answer a different question.
                slicer_cli::read_project_facts(project_file_config_kept ? project_file_config : config,
                                               project_facts);
            // is_bbl_3mf ends false once a model file follows the project
            // (BambuStudio.cpp 1871; OrcaSlicer.cpp 1553): the settings steps
            // for a Bambu-made project do not run.
            if (trailing_port)
                project_facts.is_bbl_3mf = false;
            rs.file_version = file_version;
            for (const Slic3r::Preset* p : presets)
                if (p) project_presets_kept.push_back(*p);
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
            // The objects stay at their scene places in the one model, and
            // each plate is sliced in its own frame (the plate's instances
            // moved by minus its origin, PlateScope; see the Print's origin at
            // print.apply). The objects stay where the maker put them on their
            // plate: no snap to the bed
            // corner, no plate_N.json bbox guess (both moved parts, and on beds
            // with an exclusion area the corner snap pushed them into it).
            if (is_bbl_3mf)
                for (int index = 0; index < (int)plate_data.size(); ++index)
                    plate_origins.push_back(plate_grid_origin(config, file_version, index, (int)plate_data.size()));
            // The plates of the run: the one plate loaded (every instance of
            // the model is on it), or with every plate loaded each plate's
            // instances as the file states them (PlateData::obj_inst_map's
            // identify ids, else the first plate an instance's box meets:
            // PartPlateList::reload_all_objects -> PartPlate::intersect_instance,
            // PartPlate.cpp 5668-5710, 2651-2677).
            if (is_bbl_3mf && plate_id > 0) {
                rs.plates.assign(plate_data.size(), RunPlate());
                for (int index = 0; index < (int)plate_data.size(); ++index) {
                    rs.plates[index].origin = plate_origins[index];
                    rs.plates[index].all = index == plate_id - 1;
                }
            } else if (is_bbl_3mf && plate_data.size() > 1 && !trailing_port) {
                rs.plates.assign(plate_data.size(), RunPlate());
                std::set<const Slic3r::ModelInstance*> placed;
                for (int index = 0; index < (int)plate_data.size(); ++index) {
                    rs.plates[index].origin = plate_origins[index];
                    rs.plates[index].all = false;
                    if (plate_data[index] == nullptr) continue;
                    std::set<int> identify_ids;
                    for (const auto& entry : plate_data[index]->obj_inst_map)
                        if (entry.second.second > 0) identify_ids.insert(entry.second.second);
                    for (Slic3r::ModelObject* object : model.objects)
                        for (Slic3r::ModelInstance* inst : object->instances)
                            if (inst->loaded_id > 0 && identify_ids.count(int(inst->loaded_id)) && placed.insert(inst).second)
                                rs.plates[index].members.push_back(inst);
                }
                for (Slic3r::ModelObject* object : model.objects)
                    for (size_t ii = 0; ii < object->instances.size(); ++ii) {
                        if (placed.count(object->instances[ii])) continue;
                        const Slic3r::BoundingBoxf3 box = object->instance_convex_hull_bounding_box(ii);
                        for (int index = 0; index < (int)plate_data.size(); ++index)
                            if (plate_scene_box(config, file_version, index, (int)plate_data.size()).intersects(box)) {
                                rs.plates[index].members.push_back(object->instances[ii]);
                                break;
                            }
                    }
            }
            if (trailing_port || plate_id == 0) {
                plan_file_config = config;
                plan_file_version = file_version;
                if (const auto* ids = config.option<Slic3r::ConfigOptionStrings>("filament_settings_id"))
                    plan_file_filaments = (int)ids->values.size();
            }
            // A plate prints by object when its own print_sequence says so,
            // else when the project's does (get_print_sequence,
            // BambuStudio.cpp 4403-4416; the plate's own settings go over the
            // project's per plate, 6902-6904). plate_N.json's is_seq_print is
            // what the desktop's last slice found, an output, not a setting.
            if (plate_id > 0 && is_bbl_3mf && !model.objects.empty()) {
                const Slic3r::Vec2d origin = plate_grid_origin(config, file_version, plate_id - 1, (int)plate_data.size());
                if (verbose)
                    std::cout << "Plate " << plate_id << " of " << plate_data.size()
                              << ": grid origin (" << origin.x() << ", " << origin.y() << ")\n";
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
            // The kinds this engine's loaders read (cli_model_load.cpp).
            std::cerr << "Unsupported file format. This engine loads: "
                      << slicer_cli::loadable_model_extensions() << "\n";
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

        // Model files after a project 3MF: loaded the same way as geometry
        // (the official CLI reads them after the first file, BambuStudio.cpp
        // 1855-2163) and placed further down with the other new objects.
        if (!geometry_input && o.input_files.size() > 1) {
            const std::vector<std::string> rest(o.input_files.begin() + 1, o.input_files.end());
            const slicer_cli::ModelLoadResult loaded = slicer_cli::load_geometry_files(o, rest, 1, model);
            if (loaded.code != 0) {
                emit_event({{"event","load_error"}, {"tag","ModelLoadFailed"}, {"path", rest.front()},
                            {"message", loaded.message}});
                std::cerr << "Error: " << loaded.message << "\n";
                set_outcome_failure(outcome, loaded.code, loaded.message);
                return 1;
            }
        }
        // A run of one plate (model files, a 3MF sliced as one plate): every
        // instance is on it, at the origin.
        if (rs.plates.empty())
            rs.plates.assign(1, RunPlate());
        const bool has_unplaced_objects = std::any_of(model.objects.begin(), model.objects.end(),
            [](const Slic3r::ModelObject* obj) { return obj->instances.empty(); });

        // Ensure all objects have at least one instance
        // Also set use_loaded_id_for_label so that the identify_id from
        // model_settings.config is used for OBJECT_ID labels in G-code
        // (matches BambuStudio desktop behavior at BambuStudio.cpp:6196).
        // Geometry still to be placed gets its instance from the placement.
        for (auto* obj : model.objects) {
            if (obj->instances.empty() && !has_unplaced_objects) {
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
        // --load-settings, --load-filaments, --uptodate*: the official
        // settings merge (cli_load_settings.cpp), on the settings loaded so far.
        // (rs.settings_merge; rs.merged_print_sequence: the project's
        // print_sequence after the merge, the official m_print_config's.)
#ifdef ENGINE_BAMBU
        // An assemble list's OBJ colours become filaments in the merge, which
        // needs the filaments loaded (BambuStudio.cpp 2574-2580).
        if (assemble_input && !g_assemble->colours.empty()) {
            settings_merge.input_obj_colours.assign(g_assemble->colours.begin(), g_assemble->colours.end());
            if (!slicer_cli::wants_settings_merge(o)) {
                const std::string why = "The assemble list's OBJ files carry colours, which become filaments: "
                                        "give the filaments with --load-filaments.";
                std::cerr << "Error: " << why << "\n";
                set_outcome_failure(outcome, CLI_INVALID_PARAMS, why);
                return 1;
            }
        }
#endif
        if (slicer_cli::wants_settings_merge(o)) {
            // Whether the settings loaded so far carry filament_colour (the
            // official m_print_config holds no engine defaults yet; see
            // SettingsMerge::project_has_filament_colour): a project 3MF's
            // own settings, the named presets' full config, or a profile file
            // that names it.
            bool has_colours = (!input_file.empty() && input_is_3mf(input_file) && !geometry_input && !assemble_input) ||
                               g_preset_config != nullptr;
            for (const std::string* path : {&bundle_config, &machine_config, &process_config, &filament_config})
                has_colours = has_colours || json_file_has_key(*path, "filament_colour");
            settings_merge.project_has_filament_colour = has_colours;
            std::vector<Slic3r::Preset*> preset_ptrs;
            for (Slic3r::Preset& p : project_presets_kept)
                preset_ptrs.push_back(&p);
            // The official merges into m_print_config, which holds the
            // project's print_sequence, not a plate's own (get_print_sequence
            // reads the plate's first, BambuStudio.cpp 4403-4416).
            const slicer_cli::StepResult merged =
                slicer_cli::merge_loaded_settings(o, project_facts, this_engine_profiles_dir(o.argv0).string(), config, extra,
                                                  preset_ptrs, settings_merge);
            if (const Slic3r::ConfigOption* seq = config.option("print_sequence"))
                merged_print_sequence = seq->serialize();
            if (merged.code != 0) {
                emit_event({{"event","config_refused"}, {"tag","SettingsFilesRefused"}, {"message", merged.message}});
                std::cerr << "Error: " << merged.message << "\n";
                set_outcome_failure(outcome, merged.code, merged.message);
                return 1;
            }
            slicer_cli::update_object_configs_after_switch(project_facts, settings_merge, config, model);
            outcome.settings_merge = std::make_shared<slicer_cli::SettingsMerge>(settings_merge);
            if (slicer_cli::disable_tower_after_mapping(o, settings_merge, config, extra))
                emit_event({{"event","config_normalized"}, {"tag","PrimeTowerOffOneFilament"},
                            {"message","Every filament slot holds the same filament: the prime tower is off"}});
        }
        // --load-custom-gcodes, --skip-modified-gcodes (cli_load_settings.cpp).
        {
            const slicer_cli::StepResult g = slicer_cli::apply_custom_gcodes(o, o.slice_mode ? o.slice_plate : plate_id, model);
            if (g.code != 0) {
                std::cerr << "Error: " << g.message << "\n";
                set_outcome_failure(outcome, g.code, g.message);
                return 1;
            }
        }
        // Every setting given as a flag, over the loaded settings: the
        // official m_print_config.apply(m_extra_config, true)
        // (BambuStudio.cpp 4091 at 5873b5f; OrcaSlicer.cpp 3542 at 31f6803).
        if (!extra.empty())
            config.apply(extra, true);

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
            // Every plate the run slices: plate N, or each plate for --slice 0.
            std::vector<int> checked_plates;
            if (plate_id > 0 || plate_data.size() <= 1)
                checked_plates.push_back(plate_id);
            else
                for (int k = 1; k <= (int)plate_data.size(); ++k)
                    checked_plates.push_back(k);
            for (const int checked : checked_plates) {
            const Slic3r::DynamicPrintConfig plate_settings = plate_own_settings(plate_data, checked);
            std::unique_ptr<Slic3r::DynamicPrintConfig> with_overrides;
            if (!overrides.empty() || !plate_settings.empty()) {
                with_overrides = std::make_unique<Slic3r::DynamicPrintConfig>(config);
                with_overrides->apply(plate_settings, true);
                with_overrides->apply(extra, true);
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
        }

        // A project on a printer with another bed: each plate's objects and
        // tower move onto it (move_to_new_bed; translate_models runs on every
        // plate, BambuStudio.cpp 4497-4659; OrcaSlicer.cpp 3885-3980), after
        // the settings given as flags apply (BambuStudio.cpp 4091; OrcaSlicer.cpp
        // 3542), so a flag's tower position or prime tower moves too. Each
        // plate moves in its own frame (PlateScope).
        if (project_facts.is_bbl_3mf && !assemble_input && !model.objects.empty()) {
            const int first = plate_id > 0 ? plate_id : 1;
            const int last  = plate_id > 0 ? plate_id : (int)rs.plates.size();
            for (int moved_plate = first; moved_plate <= last; ++moved_plate) {
                const int index = moved_plate - 1;
                RunPlate& run_plate = rs.plate(moved_plate);
                // The plate's own print sequence, else the project's
                // (get_print_sequence, BambuStudio.cpp 4403-4416).
                bool is_sequence = false;
                if (const auto* seq = config.option<Slic3r::ConfigOptionEnum<Slic3r::PrintSequence>>("print_sequence"))
                    is_sequence = seq->value == Slic3r::PrintSequence::ByObject;
                if (index < (int)plate_data.size() && plate_data[index] != nullptr)
                    if (const auto* own = plate_data[index]->config.option<Slic3r::ConfigOptionEnum<Slic3r::PrintSequence>>("print_sequence");
                        own && own->value != Slic3r::PrintSequence::ByDefault)
                        is_sequence = own->value == Slic3r::PrintSequence::ByObject;
                Slic3r::Vec3d moved;
                {
                    PlateScope scope(model, run_plate);
                    shrink_to_new_bed = move_to_new_bed(project_facts, settings_merge.machine_switch, model, config,
                                                        index, is_sequence, moved);
                }
                if (moved.x() != 0. || moved.y() != 0.) {
                    std::ostringstream text;
                    text << std::fixed << std::setprecision(2) << "The plate's objects and prime tower moved by (" << moved.x()
                         << ", " << moved.y() << ") mm onto this printer's bed";
                    emit_event({{"event","arranged"}, {"tag","MovedToNewBed"}, {"shrink_to_new_bed", shrink_to_new_bed},
                                {"message", text.str()}});
                    // The moved tower is the project's (the official moves it in
                    // m_print_config, which it exports, BambuStudio.cpp 4659,
                    // 8156-8157); the flags laid over the plate below
                    // (6902-6904) reach only the slice.
                }
            }
            // A plate moved onto the new printer also moves to its place in
            // the grid of the new bed size (compute_origin_using_new_size,
            // BambuStudio.cpp 4534; OrcaSlicer.cpp 3919; then reset_size):
            // translate_models moves every plate when the printer changes
            // (BambuStudio.cpp 4516) or, on Orca, when the bed changes
            // (OrcaSlicer.cpp 3902). The plate keeps the same place on itself.
#ifdef ENGINE_ORCA
            const bool translated = shrink_to_new_bed > 0;
#else
            const bool translated = settings_merge.machine_switch;
#endif
            const auto* new_area = config.option<Slic3r::ConfigOptionPoints>("printable_area");
            if (translated && new_area && new_area->values.size() >= 4 && !plate_data.empty())
                for (int index = 0; index < (int)rs.plates.size() && index < (int)plate_data.size(); ++index) {
                    RunPlate& run_plate = rs.plates[index];
                    const Slic3r::Vec2d new_origin =
                        plate_grid_origin(config, Slic3r::Semver(), index, (int)plate_data.size());
                    const Slic3r::Vec3d delta(new_origin.x() - run_plate.origin.x(), new_origin.y() - run_plate.origin.y(), 0.);
                    if (delta == Slic3r::Vec3d::Zero())
                        continue;
                    for (Slic3r::ModelObject* object : model.objects)
                        for (Slic3r::ModelInstance* inst : object->instances)
                            if (run_plate.is_member(inst))
                                inst->set_offset(inst->get_offset() + delta);
                    run_plate.origin = new_origin;
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
        project_filaments = project_filament_count(config);

        // extruder_nozzle_stats: how many nozzles of each flow type each
        // extruder holds; automatic grouping reads it (ToolOrdering.cpp 1321,
        // 1889). The desktop writes it into every project (full_config,
        // PresetBundle.cpp 3526; reset on a printer change, Plater.cpp
        // ~1307-1358 and ExtruderNozzleStat::on_printer_model_change), and a
        // named preset carries it (desktop_presets.cpp). Input without it
        // (settings files, an STL with --machine/--process/--filament) gets
        // the official CLI's: the printer's extruder_max_nozzle_count nozzles
        // of the nozzle_volume_type flow per extruder
        // (on_printer_model_change_cli, BambuStudio.cpp 4141-4155), then each
        // extruder switched to the plate's flow type and written to
        // m_print_config on a printer with several extruders or nozzles
        // (BambuStudio.cpp 6668-6692; new_nozzle_volume_type from 3428-3440).
        // Unlike the official CLI, a value the input carries is never
        // recomputed (the official also does it on a printer change): the
        // product writes the value into every 3MF and it is kept as it is.
        {
            const auto* stats = config.option<Slic3r::ConfigOptionStrings>("extruder_nozzle_stats");
            const bool carried = stats != nullptr &&
                std::any_of(stats->values.begin(), stats->values.end(), [](const std::string& s) { return !s.empty(); });
            const auto* max_opt = config.option<Slic3r::ConfigOptionIntsNullable>("extruder_max_nozzle_count");
            const auto* nozzles = config.option<Slic3r::ConfigOptionFloats>("nozzle_diameter");
            if (!carried && max_opt != nullptr && !max_opt->values.empty() && nozzles != nullptr) {
                const std::vector<int> max_nozzle_count = max_opt->values;
                const size_t slot_count = max_nozzle_count.size();
                const int new_extruder_count = int(nozzles->values.size());
                const auto* opt_nvt = dynamic_cast<const Slic3r::ConfigOptionEnumsGeneric*>(config.option("nozzle_volume_type"));
                // BambuStudio.cpp 4144-4155.
                std::vector<int> curr_volume_map_value(slot_count, static_cast<int>(Slic3r::NozzleVolumeType::nvtStandard));
                if (opt_nvt && !opt_nvt->values.empty())
                    for (size_t idx = 0; idx < slot_count; ++idx)
                        curr_volume_map_value[idx] = idx < opt_nvt->values.size() ? opt_nvt->values[idx] : opt_nvt->values.back();
                Slic3r::ExtruderNozzleStat nozzle_stats_obj;
                nozzle_stats_obj.on_printer_model_change_cli(curr_volume_map_value, max_nozzle_count);
                // BambuStudio.cpp 3428-3440: the flow type per extruder.
                std::vector<Slic3r::NozzleVolumeType> new_nozzle_volume_type;
                if (opt_nvt && opt_nvt->values.size() >= static_cast<size_t>(new_extruder_count)) {
                    for (int i = 0; i < new_extruder_count; i++)
                        new_nozzle_volume_type.push_back(Slic3r::NozzleVolumeType(opt_nvt->values[i]));
                } else {
                    if (!rs.settings_merge.machine_switch)
                        for (int v : project_facts.current_nozzle_volume_type)
                            new_nozzle_volume_type.push_back(Slic3r::NozzleVolumeType(v));
                    new_nozzle_volume_type.resize(new_extruder_count, Slic3r::NozzleVolumeType::nvtStandard);
                }
                // BambuStudio.cpp 3412-3413 and 6668-6692.
                const bool support_multi_nozzle = std::any_of(max_nozzle_count.begin(), max_nozzle_count.end(),
                                                              [](int v) { return v > 1; });
                if (new_extruder_count > 1 || support_multi_nozzle) {
                    for (size_t eid = 0; eid < new_nozzle_volume_type.size(); ++eid)
                        nozzle_stats_obj.on_volume_type_switch(int(eid), new_nozzle_volume_type[eid]);
                    config.option<Slic3r::ConfigOptionStrings>("extruder_nozzle_stats", true)->values =
                        Slic3r::save_extruder_nozzle_stats_to_string(nozzle_stats_obj.get_raw_stat());
                }
            }
        }
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

                // A list of script paths, not a value per extruder. Padding it
                // names an empty script, and a project exported with that entry
                // is refused by the official command line's normative check
                // (BambuStudio.cpp 1948-1956).
                "post_process",

                // Not one value per extruder either; the official CLI leaves
                // each as the settings give it, and a padded 0 is read:
                //  - wipe_tower_x / wipe_tower_y: one entry per plate, read
                //    with get_at(plate index) (Print.cpp 998, 2213, 2662),
                //    which falls back to the first entry (Config.hpp 681-685). A 0 put plate
                //    2's tower at (0, 0).
                //  - flush_multiplier / flush_multiplier_fast: one per nozzle;
                //    when the official sizes them it fills 1.0 / 1.2
                //    (BambuStudio.cpp 3864-3868; OrcaSlicer.cpp 3360-3361),
                //    and get_at() gives a short list's first value. A 0 is no
                //    flush on the second nozzle.
                //  - first_layer_print_sequence / other_layers_print_sequence:
                //    filament orders; more than one entry is read as a custom
                //    first-layer order (ToolOrdering.cpp 467-468, 597-610).
                "wipe_tower_x",
                "wipe_tower_y",
                "flush_multiplier",
                "flush_multiplier_fast",
                "first_layer_print_sequence",
                "other_layers_print_sequence",
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

                // Every filament setting (is_filament_setting_key, which holds
                // all the keys the final alignment treats as a roster) is left
                // alone, so this pass can never pad an array the alignment then
                // treats as a roster.  The index maps the predicate excludes are
                // in skip_pad above.
                if (is_filament_setting_key(key))
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
#endif // ENGINE_BAMBU — the run's settings base

        // The run's settings base, m_print_config (see RunState): everything
        // above runs once per run.
        rs.base = config;
        rs.loaded = true;
        } // !rs.loaded

        // This plate's settings: a copy of the base with the plate's own and
        // the flags laid on top (BambuStudio.cpp 6902-6904; OrcaSlicer.cpp
        // 5905-5907). The plate the run-level steps ran for keeps the settings
        // those steps read and changed (the arrange's tower).
        const bool fresh_plate_config = !(rs.prepared && plate_id == rs.prepared_plate);
        config = fresh_plate_config ? rs.base : rs.run_config;
        // --export-settings writes the project's settings, not this plate's:
        // the official action saves m_print_config before the plate loop
        // applies *part_plate->config() (BambuStudio.cpp 6366-6370 and
        // 6902-6904; OrcaSlicer.cpp 5499-5503 and 5905-5907).
        if (slicer_cli::has_model_actions(o))
            project_settings = rs.base;
        if (fresh_plate_config) {
        // The prepare call of a run of several plates lays no plate's own
        // settings: its settings serve the run-level steps only, and each
        // plate reports its own when it is sliced.
        HeldEventsScope run_config_events(outcome.prepare_pass && plate_id == 0 &&
                                          (trailing_port || plate_data.size() > 1));
#ifdef ENGINE_BAMBU
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
        // filament_nozzle_map fallback, which is for an otherwise-auto mapping.
        bool nozzle_mapping_derived =
            explicit_plate_mapping_applied ? false : apply_explicit_nozzle_mapping(
                config, explicit_config_supplied_nozzle_map, verbose);

        // The derived routing is the plate's (filament_map + its mode); every
        // object keeps its own filament, as in Bambu Studio and OrcaSlicer,
        // which never move an object to another filament for routing.
        if (nozzle_mapping_derived) {
            // --export-3mf reloads the file, which knows nothing of this
            // routing: keep it so the plate states what the slice used.
            if (o.slice_mode && !o.export_3mf.empty()) {
                outcome.derived_filament_map = config.option<Slic3r::ConfigOptionInts>("filament_map", true)->values;
                outcome.derived_filament_map_mode = config.opt_serialize("filament_map_mode");
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

        // Toolchanger filament-map (U1/Prusa-XL class) on the Orca build: the
        // flat 3MF stores filament_map_mode="Auto For Flush" with a placeholder
        // filament_map. With is_BBL_printer() initialized deterministically (see
        // below, next to print construction) the engine's native Auto-For-Flush
        // resolution computes the per-filament→tool map from the per-volume
        // extruders we already load, matching golden B — so no driver-side
        // filament_map injection is needed here. Model per-extruder static
        // tables (setExtruderParams/setPrintSpeedTable) are still set after
        // apply(), exactly as Orca's own headless CLI does.

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

        // The settings given as flags over the plate's own (the official
        // apply(m_extra_config) after apply(*part_plate->config()),
        // BambuStudio.cpp 6902-6904, OrcaSlicer.cpp 5905-5907).
        if (!extra.empty())
            config.apply(extra, true);
        // Apply command-line overrides
        apply_command_line_overrides(config, overrides, /*report_rejections=*/true);
#ifdef ENGINE_ORCA
        // A Bambu Studio project's own start G-code can carry the maker's
        // placeholders, and this engine binds no value for two of them: its
        // PlaceholderParser sets initial_extruder and reads nozzle_diameter
        // from the config, while BambuStudio also sets
        // nozzle_diameter_at_nozzle_id and initial_nozzle_id (BambuStudio
        // GCode.cpp 2656-2660 at 5873b5f; upstream main binds them with its
        // multi-nozzle engine, GCode.cpp 3334-3335 on main). Unbound, the
        // parser throws at export, after the whole run has sliced
        // ("Not a variable name"): refused here, before slicing, naming the
        // placeholder and the value to use instead. The settings are the ones
        // the run slices with, overrides and all.
        {
            std::string       placeholder_key;
            const std::string placeholder = bambu_only_gcode_placeholder(config, placeholder_key);
            if (!placeholder.empty()) {
                const std::string instead = placeholder == "initial_nozzle_id"
                                                ? "{initial_extruder}"
                                                : "{nozzle_diameter[initial_extruder]}";
                const std::string sentence =
                    "The project's " + placeholder_key + " uses {" + placeholder +
                    "}, which this engine has no value for; use " + instead + " instead (OrcaSlicer binds "
                    "the initial extruder as {initial_extruder} and the nozzle diameters as {nozzle_diameter}).";
                emit_event({{"event","config_refused"},
                            {"tag","BambuOnlyGcodePlaceholder"},
                            {"opt_key", placeholder_key},
                            {"placeholder", placeholder},
                            {"replace_with", instead},
                            {"message", sentence}});
                std::cerr << "Error: " << sentence << "\n";
                set_outcome_failure(outcome, CLI_INVALID_VALUES_IN_3MF, sentence);
                return 1;
            }
        }
#endif

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
        run_config_events.drop();
        } // fresh_plate_config
        // A plate sliced after the run-level steps ran for every plate: the
        // towers those steps placed (the official's m_print_config after its
        // arrange, BambuStudio.cpp 5693-5700, 5856; OrcaSlicer.cpp 4954-4961).
        if (fresh_plate_config && rs.prepared)
            for (const char* key : {"wipe_tower_x", "wipe_tower_y"})
                if (const Slic3r::ConfigOption* opt = rs.run_config.option(key))
                    config.set_key_value(key, opt->clone());

        // A plate's print sequence: its own, or else the project's
        // (get_print_sequence, BambuStudio.cpp 4403-4416), for the plates
        // other than this one (--downward-check, and the whole project of
        // the model actions for --slice 0).
        std::optional<bool> project_sequence_read;
        const auto project_sequence = [&]() {
            if (project_sequence_read)
                return *project_sequence_read;
            std::string project_seq;
            if (const Slic3r::ConfigOption* opt = extra.option("print_sequence"))
                project_seq = opt->serialize();
            else if (!merged_print_sequence.empty())
                // --load-settings and friends: the merged settings
                // (m_print_config, BambuStudio.cpp 4785).
                project_seq = merged_print_sequence;
            else if (assemble_input) {
                if (const auto* seq = config.option<Slic3r::ConfigOptionEnum<Slic3r::PrintSequence>>("print_sequence"))
                    project_seq = seq->serialize();
            } else {
                std::string text;
                if (read_zip_member(input_file, "Metadata/project_settings.config", text)) {
                    try {
                        const json settings = json::parse(text);
                        if (settings.contains("print_sequence") && settings["print_sequence"].is_string())
                            project_seq = settings["print_sequence"].get<std::string>();
                    } catch (...) {
                    }
                }
            }
            project_sequence_read = project_seq == "by object";
            return *project_sequence_read;
        };
        const auto plate_sequence = [&](int index) {
            if (index < (int)plate_data.size() && plate_data[index] != nullptr)
                if (const auto* seq = plate_data[index]->config.option<Slic3r::ConfigOptionEnum<Slic3r::PrintSequence>>("print_sequence");
                    seq && seq->value != Slic3r::PrintSequence::ByDefault)
                    return seq->value == Slic3r::PrintSequence::ByObject;
            return project_sequence();
        };

        // --downward-check, before the transforms as the official runs it
        // (BambuStudio.cpp 4669-4914; OrcaSlicer.cpp 4006-4175). The
        // official checks the plates of a project (plate_obj_size_infos
        // exists only then, BambuStudio.cpp 4646-4658), every one of them.
        if (o.given_flag("downward_check") && o.cli.opt_bool("downward_check") && g_downward_run.done) {
            // Every plate was checked by the run's first call.
            outcome.downward_checked = true;
            outcome.downward_printers = g_downward_run.printers;
            outcome.downward_failed = g_downward_run.failed;
            outcome.sequence_plate = g_downward_run.sequence_plate;
        } else if (o.given_flag("downward_check") && o.cli.opt_bool("downward_check")) {
            if (plate_data.empty()) {
                const std::string why = "--downward-check checks the plates of a 3MF project; this input has none";
                std::cerr << "Error: " << why << "\n";
                set_outcome_failure(outcome, CLI_INVALID_PARAMS, why);
                return 1;
            }
            std::vector<slicer_cli::DownwardPrinter> printers;
            const slicer_cli::StepResult loaded = slicer_cli::load_downward_printers(o, project_facts, printers);
            if (loaded.code != 0) {
                std::cerr << "Error: " << loaded.message << "\n";
                set_outcome_failure(outcome, loaded.code, loaded.message);
                return 1;
            }
            // Every plate of the project, whichever is sliced (BambuStudio.cpp
            // 4779-4790; OrcaSlicer.cpp 4104-4117): each plate's objects, and
            // its own print sequence or else the project's (get_print_sequence,
            // BambuStudio.cpp 4403-4416). A plate in the run's model is read
            // from it, in its own frame (PlateScope); a plate the run did not
            // load (--plate N / --slice N load one plate) from the project.
            struct DownwardPlate {
                int index = 0;
                std::unique_ptr<Slic3r::Model> loaded;
                const Slic3r::Model* model = nullptr;
                bool is_sequence = false;
            };
            std::vector<DownwardPlate> downward_plates;
            const int project_plates = (int)plate_data.size();
            for (int index = 0; index < project_plates; ++index) {
                DownwardPlate plate;
                plate.index = index;
                const bool in_model = index < (int)rs.plates.size() &&
                                      (rs.plates[index].all || !rs.plates[index].members.empty());
                if (project_plates == 1 || index == plate_id - 1 || in_model) {
                    {
                        PlateScope scope(model, rs.plate(index + 1));
                        plate.loaded = std::make_unique<Slic3r::Model>(model);
                    }
                    plate.model = plate.loaded.get();
                    if (project_plates == 1 || index == plate_id - 1) {
                        bool is_sequence = false;
                        if (const auto* seq = config.option<Slic3r::ConfigOptionEnum<Slic3r::PrintSequence>>("print_sequence"))
                            is_sequence = seq->value == Slic3r::PrintSequence::ByObject;
                        plate.is_sequence = is_sequence;
                    } else {
                        plate.is_sequence = plate_sequence(index);
                    }
                } else {
                    plate.loaded = std::make_unique<Slic3r::Model>();
                    if (assemble_input) {
                        for (const Slic3r::ModelObject* object : g_assemble->plates[index].loaded_obj_list)
                            plate.loaded->add_object(*object);
                    } else if (!load_plate_objects(input_file, index + 1, *plate.loaded)) {
                        const std::string why = "--downward-check could not read plate " + std::to_string(index + 1) +
                                                " of " + input_file + ".";
                        std::cerr << "Error: " << why << "\n";
                        set_outcome_failure(outcome, CLI_DATA_FILE_ERROR, why);
                        return 1;
                    } else if (verbose) {
                        std::cout << "--downward-check: read plate " << index + 1 << " from the project\n";
                    }
                    if (!assemble_input && index < (int)plate_origins.size()) {
                        // The plate at its own origin, as the sliced plate is:
                        // the official adds the plate's origin to its tower
                        // position instead (plate->get_origin(), BambuStudio.cpp
                        // 4486-4489; OrcaSlicer.cpp 3875-3878).
                        const Slic3r::Vec2d origin = plate_origins[index];
                        for (Slic3r::ModelObject* object : plate.loaded->objects)
                            for (Slic3r::ModelInstance* inst : object->instances) {
                                const Slic3r::Vec3d off = inst->get_offset();
                                inst->set_offset(Slic3r::Vec3d(off.x() - origin.x(), off.y() - origin.y(), off.z()));
                            }
                    }
                    plate.model = plate.loaded.get();
                    plate.is_sequence = plate_sequence(index);
                }
                downward_plates.push_back(std::move(plate));
            }
            // A Bambu-made 3MF with support on anywhere (BambuStudio.cpp 2197-2210).
            bool has_support = false;
            if (is_bbl_3mf && !trailing_port) {
                const auto* support = config.option<Slic3r::ConfigOptionBool>("enable_support");
                has_support = support && support->value;
                for (const DownwardPlate& plate : downward_plates)
                    for (const Slic3r::ModelObject* object : plate.model->objects) {
                        const auto* obj_support = object->config.get().option<Slic3r::ConfigOptionBool>("enable_support");
                        has_support |= obj_support && obj_support->value;
                    }
            }
            outcome.downward_checked = true;
            for (const slicer_cli::DownwardPrinter& p : printers)
                outcome.downward_printers.push_back(p.name);
            for (const DownwardPlate& plate : downward_plates) {
                outcome.sequence_plate |= plate.is_sequence;
                if (printers.empty())
                    continue;
                const std::vector<std::string> failed = slicer_cli::downward_failures(
                    printers, downward_plate_size(*plate.model, config, plate.index, plate.is_sequence),
                    plate.is_sequence, config, project_facts, has_support);
                for (const std::string& name : failed)
                    if (std::find(outcome.downward_failed.begin(), outcome.downward_failed.end(), name) ==
                        outcome.downward_failed.end())
                        outcome.downward_failed.push_back(name);
            }
            g_downward_run.done = true;
            g_downward_run.printers = outcome.downward_printers;
            g_downward_run.failed = outcome.downward_failed;
            g_downward_run.sequence_plate = outcome.sequence_plate;
        }

        // --pipe: the files are loaded (BambuStudio.cpp 4923-4926;
        // OrcaSlicer.cpp 4185), before the transforms.
        pipe_files_loaded(rs);

        // The transforms on the command line, in its order, then orient
        // (cli_run_steps.cpp). Before any placement, as the official CLI
        // runs them before its arrange.
        // A plate of the plan has its objects transformed already (the plan
        // pass ran the transforms on every object, as the official does).
        // The plates of an assemble list at their places in the plate grid of
        // this printer's bed (PartPlateList::compute_origin, PartPlate.cpp
        // 4306-4341), now that the bed is known.
        if (first_call && assemble_input) {
            const int count = (int)g_assemble->plates.size();
            for (int k = 0; k < count && k < (int)rs.plates.size(); ++k) {
                const Slic3r::Vec2d origin = plate_grid_origin(config, Slic3r::Semver(), k, count);
                rs.plates[k].origin = origin;
                for (Slic3r::ModelInstance* inst : rs.plates[k].members)
                    inst->set_offset(inst->get_offset() + Slic3r::Vec3d(origin.x(), origin.y(), 0.));
            }
        }
        if (first_call && !o.transforms.empty()) {
            int duplicate_count = 0;
            const int plate_count = std::max<int>(1, (int)plate_data.size());
            const slicer_cli::StepResult t = slicer_cli::apply_transforms(
                o, model, &settings_merge.load_process_config, o.slice_mode ? o.slice_plate : plate_id, plate_count,
                duplicate_count);
            if (t.code != 0) {
                std::cerr << "Error: " << t.message << "\n";
                set_outcome_failure(outcome, t.code, t.message);
                return 1;
            }
            outcome.duplicate_count = duplicate_count;
        }
        {
            const slicer_cli::StepResult sd = slicer_cli::check_slicedata_flags(o, outcome.duplicate_count);
            if (sd.code != 0) {
                std::cerr << "Error: " << sd.message << "\n";
                set_outcome_failure(outcome, sd.code, sd.message);
                return 1;
            }
        }
        const bool needs_placement = first_call && (geometry_input ||
            std::any_of(model.objects.begin(), model.objects.end(),
                        [](const Slic3r::ModelObject* obj) { return obj->instances.empty(); }));

        // Geometry placement, with the bed now known. --arrange 1 or 0: the
        // official CLI's default instances (read_from_file with
        // AddDefaultInstances, BambuStudio.cpp 1884), then --arrange 1
        // arranges below. Otherwise the desktop's placement on load
        // (cli_model_load.cpp). A project 3MF's own objects keep their places.
        if (first_call && geometry_only_3mf && o.arrange_auto())
            slicer_cli::desktop_center_geometry_3mf(model, config);
        if (needs_placement) {
            if (!o.arrange_auto())
                model.add_default_instances();
            else
                slicer_cli::desktop_place_on_bed(model, config);
            for (auto* obj : model.objects)
                for (auto* inst : obj->instances)
                    inst->use_loaded_id_for_label = true;
        }

        // Actions that work on the loaded model and settings (--info,
        // --export-settings, --export-stl, --export-stls). The official runs
        // every action in one loop in command-line order, the first that fails
        // stopping the rest (BambuStudio.cpp 6336, 6366-6401; OrcaSlicer.cpp
        // 5470, 5499-5534), --slice among them (6437; 5561) with each plate's
        // bed check and slice inside it. So with --slice the actions given
        // before it run before the first plate's bed check, and those given
        // after it once the last plate has sliced; without --slice they are the
        // whole run. A failure is recorded with plate 0, as the official records
        // these (record_exit_reson(..., CLI_EXPORT_STL_ERROR, 0, ...),
        // BambuStudio.cpp 6391, 6399).
        // A project 3MF with model files after it: the plan (ProjectPlan) once
        // the transforms ran, before any action, as the official arranges
        // before its action loop; the actions then see every object at its
        // arranged place.
        // --slice 0 --arrange 1 on a project of several plates: the official
        // arranges every plate's objects together across the plates (the
        // global arrange, BambuStudio.cpp 5628-5728, 5886-6006; OrcaSlicer.cpp
        // 4887-4990, 5142-5262: duplicate_count 0 and plate_to_slice 0), adding
        // or dropping plates, as for model files after the project.
        // A run of model actions only is a run without --slice, whose
        // plate_to_slice stays 0 (BambuStudio.cpp 1677; OrcaSlicer.cpp 1368):
        // the official arranges every plate for it too.
        const bool global_arrange = (o.slice_mode ? o.slice_plate == 0 : slicer_cli::model_actions_only(o)) &&
                                    o.arrange_forced() && is_bbl_3mf &&
                                    !assemble_input && !trailing_port && plate_data.size() > 1;
        if (first_call && (trailing_port || global_arrange) && !calib_self_geometry) {
            std::set<int> used_filaments;
#ifndef ENGINE_ORCA
            for (int f = 1; f <= plan_file_filaments; ++f)
                used_filaments.insert(f);
#endif
            const auto& filament_ids = o.cli.option<Slic3r::ConfigOptionInts>("load_filament_ids")->values;
            for (size_t index = 1; index < o.input_files.size() && index < filament_ids.size(); ++index)
                if (filament_ids[index] > 0)
                    used_filaments.insert(filament_ids[index]);
            ProjectPlan& plan = rs.plan;
            const Slic3r::DynamicPrintConfig& file_config = plan_file_config ? *plan_file_config : config;
            if (!plan_project_plates(model, config, file_config, plan_file_version, plate_data, (int)used_filaments.size(),
                                     global_arrange || o.arrange != 0, plan, outcome)) {
                std::cerr << "Error: " << outcome.error_string << "\n";
                return 1;
            }
            // The plates as the plan left them; a plate the arrange added is a
            // fresh plate with no settings of its own (PartPlateList::create_plate).
            rs.plates.assign(plan.plate_count, RunPlate());
            for (int k = 0; k < plan.plate_count; ++k) {
                rs.plates[k].origin  = plan.origins[k];
                rs.plates[k].all     = false;
                rs.plates[k].members = plan.members[k];
            }
            while ((int)plate_data.size() < plan.plate_count) {
                auto* pd = new Slic3r::PlateData();
                pd->plate_index = int(plate_data.size());
                plate_data.push_back(pd);
            }
            emit_project_plan_event(plan, trailing_port);
        }

        // The actions read the run's one model, every object at its place in
        // the scene (the official's m_models, BambuStudio.cpp 6371-6401;
        // OrcaSlicer.cpp 5504-5534), and --export-settings the project's
        // settings (m_print_config, BambuStudio.cpp 6366-6370; OrcaSlicer.cpp
        // 5499-5503).
        const auto run_model_action_step = [&](slicer_cli::ActionPhase phase) -> bool {
            slicer_cli::StepResult a;
            // --export-settings writes the project's settings, not this
            // plate's (see project_settings above). The command line is
            // already in them; the setting flags go on as on `config`.
            std::optional<Slic3r::DynamicPrintConfig> settings_to_export = project_settings;
            if (settings_to_export)
                apply_command_line_overrides(*settings_to_export, overrides, /*report_rejections=*/false);
            {
                // The actions run once per run, also from the check pass:
                // their events are not held.
                std::vector<std::string>* const held = slicer_cli::diagnostics::held_events;
                slicer_cli::diagnostics::held_events = nullptr;
                a = slicer_cli::run_model_actions(o, model, settings_to_export ? *settings_to_export : config, nullptr,
                                                  phase);
                slicer_cli::diagnostics::held_events = held;
            }
            if (a.code != 0) {
                std::cerr << "Error: " << a.message << "\n";
                set_outcome_failure(outcome, a.code, a.message);
                outcome.run_step_failed = true;
                return false;
            }
            return true;
        };

        // --slice: the official per-plate gate before apply (BambuStudio.cpp
        // 6527-6567 at 5873b5f; OrcaSlicer.cpp 5645-5697 at 31f6803). An
        // object partly over the bed edge is refused, and a plate with no
        // object fully inside is refused; objects wholly outside are left
        // out of the print, as the official CLI's apply() leaves them out.
        // --slice, once per run before the plate loop: the arrange, the
        // --ensure-on-bed lift and the actions given before --slice
        // (BambuStudio.cpp 5237-6194, 6336; OrcaSlicer.cpp 4498-5455, 5470),
        // for the plate the run names (in its own frame, PlateScope) or for
        // every plate. A run of model actions only takes the same steps: the
        // official arranges before its action loop with or without --slice
        // (need_arrange for model files, BambuStudio.cpp 2077, 5558, then
        // 6336; OrcaSlicer.cpp 1724, 4817, then 5470).
        const bool actions_only = slicer_cli::model_actions_only(o);
        if ((o.slice_mode || actions_only) && !calib_self_geometry && first_call) {
            // The plate's towers before the arrange: only the entries the
            // arrange changes go into the project below. The rest are the
            // flags laid over this plate (BambuStudio.cpp 6902-6904), which
            // reach only the slice.
            std::map<std::string, std::vector<double>> tower_before;
            for (const char* key : {"wipe_tower_x", "wipe_tower_y"})
                if (const auto* opt = config.option<Slic3r::ConfigOptionFloats>(key))
                    tower_before[key] = opt->values;
            // The plate's filament count from its last slice (slice_info), as
            // the official reads plate_data_src[plate]->slice_filaments_info
            // (BambuStudio.cpp 5794; OrcaSlicer.cpp 5050); 0 for an STL.
            size_t sliced_filament_count = 0;
            if (plate_id > 0 && (int)plate_data.size() >= plate_id && plate_data[plate_id - 1] != nullptr)
                sliced_filament_count = plate_data[plate_id - 1]->slice_filaments_info.size();
            const int arrange_plate_index = plate_id > 0 ? plate_id - 1 : 0;
            // Several objects from model files and --arrange automatic (not 0
            // or 1, or not given): the official
            // CLI arranges model files (need_arrange is set for them,
            // BambuStudio.cpp 2077 / OrcaSlicer.cpp 1724, and --arrange only
            // overrides it, 5066-5081 / 4327-4342), where the desktop's
            // placement on load could stack them. One object keeps the
            // desktop's placement (the bed centre either way).
            int model_file_instances = 0;
            if (geometry_input)
                for (const Slic3r::ModelObject* object : model.objects)
                    for (const Slic3r::ModelInstance* inst : object->instances)
                        model_file_instances += inst->printable ? 1 : 0;
            const bool auto_arrange_model_files = o.arrange_auto() && model_file_instances > 1;
            // A printer change that changes the extruder clearances re-arranges
            // a plate printed by object (BambuStudio.cpp 5275-5288: only on a
            // change of printer, the distance to the rod counted; OrcaSlicer.cpp
            // 4538-4549: any change of the clearances), unless the bed changed
            // (shrink_to_new_bed) or the plate is arranged anyway.
            bool clearance_arrange = false;
            {
                const auto* seq = config.option<Slic3r::ConfigOptionEnum<Slic3r::PrintSequence>>("print_sequence");
                const bool is_seq_print = seq && seq->value == Slic3r::PrintSequence::ByObject;
                const bool need_arrange = o.arrange_forced() || auto_arrange_model_files || outcome.duplicate_count > 0 ||
                                          assemble_input;
                auto opt_or_zero = [&](const char* key) {
                    return config.option<Slic3r::ConfigOptionFloat>(key) ? float(config.opt_float(key)) : 0.f;
                };
                const float height_to_rod = opt_or_zero("extruder_clearance_height_to_rod");
                const float height_to_lid = opt_or_zero("extruder_clearance_height_to_lid");
#ifdef ENGINE_ORCA
                const float clearance_radius = opt_or_zero("extruder_clearance_radius");
                const bool printer_changed = true;
                const bool rod_distance_changed = false;
#else
                const float clearance_radius = opt_or_zero("extruder_clearance_max_radius");
                const bool printer_changed = !settings_merge.new_printer_system_name.empty() &&
                                             settings_merge.new_printer_system_name != project_facts.current_printer_system_name;
                const float distance_to_rod = opt_or_zero("extruder_clearance_dist_to_rod");
                const bool rod_distance_changed = project_facts.old_distance_to_rod != 0.f &&
                                                  project_facts.old_distance_to_rod != distance_to_rod;
#endif
                const bool clearances_changed =
                    (project_facts.old_height_to_rod != 0.f && project_facts.old_height_to_rod != height_to_rod) ||
                    (project_facts.old_height_to_lid != 0.f && project_facts.old_height_to_lid != height_to_lid) ||
                    (project_facts.old_max_radius != 0.f && project_facts.old_max_radius != clearance_radius) ||
                    rod_distance_changed;
                // plate_to_slice > 0: one plate named, never --slice 0; 0
                // without --slice (BambuStudio.cpp 1677).
                const int plate_to_slice = o.slice_mode ? o.slice_plate : 0;
                clearance_arrange = !need_arrange && is_bbl_3mf && !trailing_port && shrink_to_new_bed == 0 && plate_to_slice > 0 &&
                                    printer_changed && clearances_changed && is_seq_print;
                if (clearance_arrange)
                    emit_event({{"event","arranged"}, {"tag","ClearanceArrange"},
                                {"message","The printer's extruder clearances differ from the project's: the by-object plate is arranged again"}});
            }
            // --repetitions: a spiral vase is copied only when printing by
            // object (BambuStudio.cpp 5259-5269; OrcaSlicer.cpp 4519-4529).
            if (!assemble_input && outcome.duplicate_count > 0) {
                const auto* spiral = config.option<Slic3r::ConfigOptionBool>("spiral_mode");
                const auto* seq    = config.option<Slic3r::ConfigOptionEnum<Slic3r::PrintSequence>>("print_sequence");
                if (spiral && spiral->value && !(seq && seq->value == Slic3r::PrintSequence::ByObject)) {
                    emit_event({{"event","arranged"}, {"tag","RepetitionsSkippedSpiral"},
                                {"message","--repetitions: a spiral vase is copied only when printing by object; the plate is printed once"}});
                    outcome.duplicate_count = 0;
                }
            }
            // --repetitions arranges the copies and sets the tower in `config`
            // (arrange_repetitions); the project keeps that tower.
            if (assemble_input) {
                // An assemble list's plates take the list's own arrange in
                // place of --arrange and --repetitions (BambuStudio.cpp
                // 5326-5557, before the general arrange's else-branch;
                // OrcaSlicer.cpp 4582-4816), every plate of the run in its
                // own frame.
                for (int arranged_plate = plate_id > 0 ? plate_id : 1;
                     arranged_plate <= (plate_id > 0 ? plate_id : (int)g_assemble->plates.size()); ++arranged_plate) {
                const int arrange_plate_index = arranged_plate - 1;
                const slicer_cli::AssemblePlate& plate = g_assemble->plates[arrange_plate_index];
                // A plate of --slice 0 reads its own settings (plate_params)
                // over the run's, as its own slice does.
                Slic3r::DynamicPrintConfig plate_settings_copy;
                if (plate_id == 0) {
                    plate_settings_copy = config;
                    plate_settings_copy.apply(plate_own_settings(plate_data, arranged_plate), true);
                    if (!extra.empty())
                        plate_settings_copy.apply(extra, true);
                    apply_command_line_overrides(plate_settings_copy, overrides, /*report_rejections=*/false);
                }
                Slic3r::DynamicPrintConfig& config_k = plate_id == 0 ? plate_settings_copy : config;
                PlateScope scope(model, rs.plate(arranged_plate));
                if (plate.need_arrange) {
                    if (!arrange_assemble_plate(model, config_k, arrange_plate_index, plate.filaments_count,
                                                g_assemble->model, outcome)) {
                        std::cerr << "Error: " << outcome.error_string << "\n";
                        return 1;
                    }
                } else {
                    // The objects keep the list's positions; a plate of several
                    // filaments gets the tower's default corner unless a
                    // position was given (has_wipe_tower_position, BambuStudio.cpp
                    // 4098: wipe_tower_x/y are in no preset, so only a settings
                    // file or a flag states them for a run without a 3MF).
                    const auto* seq = config_k.option<Slic3r::ConfigOptionEnum<Slic3r::PrintSequence>>("print_sequence");
                    const bool is_seq_print = seq && seq->value == Slic3r::PrintSequence::ByObject;
                    const auto states_tower = [](const Slic3r::DynamicPrintConfig& c) {
                        return c.has("wipe_tower_x") && c.has("wipe_tower_y");
                    };
                    const bool has_wipe_tower_position =
                        states_tower(settings_merge.load_process_config) || states_tower(o.extra_config);
                    if (!is_seq_print && plate.filaments_count > 1 && !has_wipe_tower_position)
                        assemble_tower_default_corner(config_k, arrange_plate_index, WIPE_TOWER_MARGIN);
                }
                // The plate's tower entry goes into the run's settings.
                if (plate_id == 0)
                    for (const char* key : {"wipe_tower_x", "wipe_tower_y"})
                        if (const auto* placed = config_k.option<Slic3r::ConfigOptionFloats>(key);
                            placed && (int)placed->values.size() > arrange_plate_index) {
                            Slic3r::ConfigOptionFloat entry(placed->get_at(arrange_plate_index));
                            config.option<Slic3r::ConfigOptionFloats>(key, true)->set_at(&entry, arrange_plate_index, 0);
                        }
                }
            } else if (outcome.duplicate_count > 0) {
                PlateScope scope(model, rs.plate(plate_id));
                std::set<int> skip_ids;
                for (int id : o.cli.option<Slic3r::ConfigOptionInts>("skip_objects")->values)
                    skip_ids.insert(id);
                model.add_default_instances();
                const slicer_cli::RepetitionsResult reps = slicer_cli::arrange_repetitions(
                    model, config, arrange_plate_index, outcome.duplicate_count, skip_ids,
                    [&](Slic3r::Model& m, Slic3r::DynamicPrintConfig& c, int& landed) {
                        PlateOutcome scratch = outcome;
                        return arrange_on_bed(m, c, arrange_plate_index, sliced_filament_count, scratch, &landed);
                    });
                // Copies the loop left off the plate stand wholly outside it
                // and are left out by the inside check below.
                outcome.duplicate_count = reps.copies_kept;
            } else if ((o.arrange_forced() || auto_arrange_model_files || clearance_arrange) && !rs.plan.done) {
                PlateScope scope(model, rs.plate(plate_id));
                if (!arrange_on_bed(model, config, arrange_plate_index, sliced_filament_count, outcome)) {
                    std::cerr << "Error: " << outcome.error_string << "\n";
                    return 1;
                }
            }
            // --ensure-on-bed before the actions given before --slice, as the
            // official lifts the objects before its action loop
            // (BambuStudio.cpp 6188-6194 then 6336; OrcaSlicer.cpp 5443-5455
            // then 5470); --skip-objects stays in the slice step after them.
            slicer_cli::ensure_on_bed_if_asked(o, model);
            // The towers the arrange placed are the project's from here on:
            // the official arrange writes them into m_print_config
            // (BambuStudio.cpp 5693-5700, 5856, 6075; OrcaSlicer.cpp
            // 4954-4961, 5112, 5335), which --export-settings, result.json and
            // --export-3mf then read (6366-6370, 6905-6910, 8156-8157).
            // Only the entries the arrange set: a tower given as a flag stays
            // the plate's, and the project keeps its own (moved) value.
            for (const char* key : {"wipe_tower_x", "wipe_tower_y"}) {
                const auto* placed = config.option<Slic3r::ConfigOptionFloats>(key);
                if (placed == nullptr)
                    continue;
                const std::vector<double>& before = tower_before[key];
                auto* project_tower = rs.base.option<Slic3r::ConfigOptionFloats>(key);
                if (project_tower == nullptr || project_tower->values.empty()) {
                    rs.base.set_key_value(key, placed->clone());
                    continue;
                }
                for (size_t k = 0; k < placed->values.size(); ++k) {
                    if (k < before.size() && before[k] == placed->values[k])
                        continue;
                    Slic3r::ConfigOptionFloat entry(placed->values[k]);
                    project_tower->set_at(&entry, k, 0);
                }
            }
            if (slicer_cli::has_model_actions(o))
                project_settings = rs.base;
            if (slicer_cli::has_model_actions(o) && !rs.actions_done) {
                rs.actions_done = true;
                if (!run_model_action_step(o.slice_mode ? slicer_cli::ActionPhase::BeforeSlice
                                                        : slicer_cli::ActionPhase::All))
                    return 1;
            }
            // Model actions only: the actions were the whole run.
            if (actions_only) {
                outcome.actions_only = true;
                return 0;
            }
            // --pipe: the slice action starts, before the first plate's
            // checks (BambuStudio.cpp 6455-6458; OrcaSlicer.cpp 5580).
            pipe_prepare_slicing(rs);
        }
        // The run-level steps are done: the prepare call stops here, before
        // any plate is checked.
        if (first_call && o.slice_mode) {
            rs.prepared = true;
            rs.prepared_plate = plate_id;
            rs.run_config = config;
            rs.carry = outcome;
            if (outcome.prepare_pass)
                return 0;
        }

        // --slice: the official per-plate gate before apply (BambuStudio.cpp
        // 6527-6567 at 5873b5f; OrcaSlicer.cpp 5645-5697 at 31f6803). An
        // object partly over the bed edge is refused, and a plate with no
        // object fully inside is refused; objects wholly outside are left
        // out of the print, as the official CLI's apply() leaves them out.
        if (o.slice_mode && !calib_self_geometry) {
            RunPlate& run_plate = rs.plate(plate_id);
            pipe_prepare_slicing(rs);
            // A plate with none of the run's instances: the refusal a load of
            // that plate alone gave (above, "No objects found in model").
            if (!run_plate.all && run_plate.members.empty()) {
                std::cerr << "No objects found in model\n";
                set_outcome_failure(outcome, CLI_NO_SUITABLE_OBJECTS);
                return 1;
            }
            const int step_plate = plate_id > 0 ? plate_id : std::max(1, o.slice_plate);
            {
                // The plate's instances in the plate's own frame: the bed check
                // reads the plate's shape (PartPlate::get_shape, the bed at the
                // plate's origin, BambuStudio.cpp 6527).
                PlateScope scope(model, run_plate);
                if (!plate_object_steps(o, model, step_plate, outcome))
                    return 1;
                if (!check_objects_inside_bed(model, config, outcome)) {
                    std::cerr << "Error: " << outcome.error_string << "\n";
                    return 1;
                }
                if (!plate_triangle_limit(o, model, step_plate, outcome))
                    return 1;
            }
            // The other plates' instances are outside this plate, so this
            // plate's Print leaves them out (ModelInstance::is_printable,
            // Model.hpp 1466 / 1344; PrintApply.cpp 145-155).
            for (Slic3r::ModelObject* object : model.objects)
                for (Slic3r::ModelInstance* inst : object->instances)
                    if (!run_plate.is_member(inst))
                        inst->print_volume_state = Slic3r::ModelInstancePVS_Fully_Outside;
        } else if (!calib_self_geometry) {
            if (first_call)
                slicer_cli::ensure_on_bed_if_asked(o, model);
            // --skip-objects and --mtcpp belong to the slice action
            // (BambuStudio.cpp 6541-6597; OrcaSlicer.cpp 5645-5720): a run of
            // model actions only never slices, so it skips nothing.
            if (!slicer_cli::model_actions_only(o) &&
                (!plate_object_steps(o, model, std::max(1, plate_id), outcome) ||
                 !plate_triangle_limit(o, model, std::max(1, plate_id), outcome)))
                return 1;
        }

        // Without --slice, or for the calibration prints that make their own
        // model, the actions run here; with --slice those before it ran before
        // the bed check above.
        if (slicer_cli::has_model_actions(o) && !rs.actions_done) {
            rs.actions_done = true;
            if (!run_model_action_step(o.slice_mode ? slicer_cli::ActionPhase::BeforeSlice : slicer_cli::ActionPhase::All))
                return 1;
            if (slicer_cli::model_actions_only(o)) {
                outcome.actions_only = true;
                return 0;
            }
        }

        // Initialize print
        std::cout << "\nInitializing print...\n";
        Slic3r::Print print;

        // The plate's own per-layer custom G-code (pauses, colour changes,
        // custom lines): the loader keys them by plate (plate_id - 1,
        // bbs_3mf.cpp 3446/3474), --load-custom-gcodes by the plate sliced,
        // and the Print reads the entry of the model's current plate
        // (Print.cpp 517-518; PrintApply.cpp 1558-1569 copies the index), so
        // the official CLI sets it per plate before slicing it
        // (BambuStudio.cpp 6493; OrcaSlicer.cpp 5617). Left at 0, every plate
        // took plate 1's. The pressure-advance pattern writes its entry at
        // the same index (calib.cpp 655).
        model.curr_plate_index = std::max(0, plate_id - 1);

        // Enable BBL printer features (M981 spaghetti detector, M1003 powerlost
        // recovery, etc.) for a printer of the Bambu Lab vendor.
#ifdef ENGINE_BAMBU
        // set_BBL_Printer is a BambuStudio-only Print method (enables M981/M1003
        // BBL printer features).  OrcaSlicer's Print has no such method.
        // The desktop decides from the printer preset's vendor
        // (BackgroundSlicingProcess.cpp:205, Preset::is_bbl_vendor_preset), and
        // the official CLI from the plate config's printer_model, else the
        // loaded or the project's printer name (BambuStudio.cpp 7055-7070 at
        // 5873b5f): the same rule for a 3MF, a named preset or settings files.
        {
            const std::string printer_model_string = config.opt_string("printer_model", true);
            bool is_bbl_vendor_preset = false;
            if (!printer_model_string.empty())
                is_bbl_vendor_preset = printer_model_string.compare(0, 9, "Bambu Lab") == 0;
            else if (!rs.settings_merge.new_printer_name.empty())
                is_bbl_vendor_preset = rs.settings_merge.new_printer_name.compare(0, 9, "Bambu Lab") == 0;
            else if (!rs.project_facts.current_printer_system_name.empty())
                is_bbl_vendor_preset = rs.project_facts.current_printer_system_name.compare(0, 9, "Bambu Lab") == 0;
            print.set_BBL_Printer(is_bbl_vendor_preset);
        }
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

        // The plate's Print in the plate's own frame. The official CLI slices
        // each plate of the scene model with a Print whose origin is the
        // plate's (PartPlate::get_print and the plate's BuildVolume,
        // BambuStudio.cpp 6506-6529; OrcaSlicer.cpp 5623-5646; the origin from
        // set_plate_origin, PartPlate.cpp 2376/2517 at 5873b5f, 2311/2433 at
        // 31f6803). Here the Print's model is the plate's instances moved by
        // minus the plate's origin (PlateScope at print.apply below) and the
        // Print's origin stays 0: the same placement, and the G-code this
        // command line has always written, which the product relies on. The
        // two differ in bytes on a plate past the first: the engine fits arcs
        // on the Print's own coordinates (arc centres round differently) and
        // starts the first layer's moves from (0,0) of that frame (the first
        // moves and the object order change). Measured on the product corpus
        // for PR #35 with the scene-frame build kept as pkg-origin: object
        // order changed on 11 of 50 plates past the first, moves on 16 beyond
        // the engine's run-to-run noise; extents and settings unchanged.
        const Slic3r::Vec2d plate_origin = Slic3r::Vec2d::Zero();   // the Print's origin
        print.set_plate_origin(Slic3r::Vec3d(0.0, 0.0, 0.0));
        // The plate's own per-plate values (wipe_tower_x/y are one entry per
        // plate, read with get_at(m_plate_index): Print.cpp 2213, GCode.cpp 5006)
        // need the plate's index, as the desktop sets it (PartPlate.cpp:2299).
        // Left at 0, plate N's prime tower stood where plate 1's does.
        if (plate_id > 0)
            print.set_plate_index(plate_id - 1);

        // Alias the unbound `initial_no_support_filament_id` placeholder to the
        // engine-bound `initial_no_support_extruder` across custom-gcode keys, BEFORE
        // print.apply snapshots the config. Without this the PlaceholderParser throws at
        // export when a 3MF carries the legacy token in its custom gcode. Always on;
        // suppressed with --no-normalize-legacy-gcode.
        if (normalize_legacy_gcode) {
            normalize_legacy_gcode_tokens(config);
        }

        // Calibration: whether the active printer speaks the Bambu G-code dialect.
        // Mirrors the is_BBL_printer() derivation used below for the engine.
        const bool calib_is_bbl_machine =
            config.opt_string("printer_model", true).rfind("Bambu Lab", 0) == 0;

        // Calibration: pressure_advance_pattern generates its own geometry + per-layer
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
        // 5873b5f; estimate_mode is off on this path). The Orca build runs
        // OrcaSlicer.cpp's own version of the block below.
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
#ifdef ENGINE_ORCA
        // The same step as the official OrcaSlicer CLI runs it before
        // automatic grouping (OrcaSlicer.cpp 5914-5951 at 31f6803): on a
        // printer with more than one extruder and a filament map mode before
        // Manual (AutoForFlush when the setting is missing), each extruder
        // gets one estimated 4-slot AMS ("1#0|4#1") whose slots take the
        // project's filament types in turn. Unlike BambuStudio, Orca leaves
        // every slot white (its colour copy is commented out) and names no
        // tray. The types are the project's (m_print_config's; a plate's own
        // settings do not change them).
        {
            const auto* nozzles = config.option<Slic3r::ConfigOptionFloats>("nozzle_diameter");
            const int new_extruder_count = nozzles ? int(nozzles->values.size()) : 1;
            if (new_extruder_count > 1) {
                Slic3r::FilamentMapMode map_mode = Slic3r::fmmAutoForFlush;
                if (const auto* mode = config.option<Slic3r::ConfigOptionEnum<Slic3r::FilamentMapMode>>("filament_map_mode"))
                    map_mode = mode->value;
                if (map_mode < Slic3r::fmmManual) {
                    std::vector<std::string> extruder_ams_count(new_extruder_count, "");
                    std::vector<std::vector<Slic3r::DynamicPrintConfig>> extruder_filament_info(new_extruder_count);
                    int color_count = 0;
                    const auto* filament_type = dynamic_cast<const Slic3r::ConfigOptionStrings*>(config.option("filament_type"));
                    std::vector<std::string> types = filament_type ? filament_type->vserialize() : std::vector<std::string>{"PLA"};
                    for (int e_index = 0; e_index < new_extruder_count; e_index++) {
                        extruder_ams_count[e_index] = "1#0|4#1";
                        for (int color_index = 0; color_index < 4; color_index++) {
                            Slic3r::DynamicPrintConfig temp_config;
                            std::vector<std::string> temp_colors(1, "#FFFFFFFF");
                            std::vector<std::string> temp_types(1, "PLA");
                            if (filament_type && !types.empty())
                                temp_types[0] = types[color_count % types.size()];
                            temp_config.option<Slic3r::ConfigOptionStrings>("filament_colour", true)->values = temp_colors;
                            temp_config.option<Slic3r::ConfigOptionStrings>("filament_type", true)->values = temp_types;
                            temp_config.option<Slic3r::ConfigOptionBools>("filament_is_support", true)->values = {0};
                            extruder_filament_info[e_index].push_back(std::move(temp_config));
                            color_count++;
                        }
                    }
                    config.option<Slic3r::ConfigOptionStrings>("extruder_ams_count", true)->values = extruder_ams_count;
                    print.set_extruder_filament_info(extruder_filament_info);
                }
            }
        }
#endif // ENGINE_ORCA

        try {
            std::cout << "Applying configuration...\n";
            {
                // The plate's instances only, on the plate (see the Print's
                // origin above). Print::apply copies the model it is given
                // (PrintApply.cpp 1553 / 1298), so the run's model stays in
                // the scene.
                PlateScope scope(model, rs.plate(plate_id));
                print.apply(model, config);
            }
#ifdef ENGINE_ORCA
            // OrcaSlicer's WipeTowerData::height is read on a path that never
            // writes it, and a value out of Clipper's range then reaches the
            // skirt. Nothing in the engine at 31f6803 initialises it: the struct
            // declares `float height;` with no initialiser (Print.hpp:763) and
            // its constructor, `WipeTowerData(ToolOrdering &) : tool_ordering(...)
            // { clear(); }` (Print.hpp:784), calls a clear() that resets depth,
            // brim_width and rib_offset but not height (Print.hpp:767-778); the
            // only writer, Print::_make_wipe_tower, sets it in its WipeTower2
            // (Type-2) branch alone (Print.cpp:3449), and a Bambu Lab printer
            // always takes the Type-1 branch (wipe_tower_type() is
            // is_BBL_printer() ? Type1 : m_config.wipe_tower_type.value,
            // Print.hpp:1072; is_wipe_tower_type2 at Print.cpp:3196, the
            // `if (!is_wipe_tower_type2)` branch at 3242, which sets depth,
            // brim_width, bbx and rib_offset at 3338-3342 and nothing else). No
            // other writer exists: OrcaSlicer.cpp never touches it, and the
            // desktop only reads it (GLCanvas3D.cpp:2874, 2894).
            // 0 is the value upstream itself gives this field: OrcaSlicer main's
            // WipeTowerData::clear() sets `height = 0.f;` beside the other
            // floats, which is exactly what our pin's clear() is missing. Set
            // here because the engine submodule cannot be patched.
            // Print::first_layer_wipe_tower_corners() reads it unconditionally
            // (Print.cpp:2838) as the height of the stabilization cone — a
            // WipeTower2-only feature, whose base radius is
            // tan(deg2rad(wipe_tower_cone_angle/2)) * height
            // (WipeTower2.cpp:2161-2175) — and adds the ring of points at that
            // radius to the skirt's hull (Print.cpp:2840-2844, appended at
            // 2647), so an indeterminate height puts a point past Clipper's
            // range and Print::_make_skirt's offset() throws
            // clipperException "Coordinate outside allowed range"
            // (Print.cpp:2695, ClipperBase::AddPathInternal). The Type-1 tower
            // (GCode/WipeTower.cpp) has no cone at all, so 0 is also the radius
            // its path means: the ring collapses onto the tower's centre and the
            // tower's own bbox, already in the hull, is unchanged. Measured on
            // the packaged macOS Orca engine of run 37538132874: the E4 project's
            // plate 2 either slices or dies with that exception run to run, and
            // setting wipe_tower_cone_angle to 0 makes every failing input slice.
            if (print.has_wipe_tower() && print.wipe_tower_type() == Slic3r::WipeTowerType::Type1)
                const_cast<Slic3r::WipeTowerData&>(print.wipe_tower_data()).height = 0.f;
#endif

            // Calibration: install the calibration params after apply (which resets print
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
            // --pipe: the steps before the first plate (BambuStudio.cpp
            // 1765-1767, 4923-4926, 6455-6458: 1 at the start, 2 once the
            // files are loaded, 3 before the plates), then this plate
            // (7034-7046: its index and the run's plate count, 4 to begin),
            // and every status the slice reports (cli_status_callback).
            const bool piped = slicer_cli::pipe_started();
            // The check pass sends no plate step: the official reports a plate
            // only once it slices it (BambuStudio.cpp 7034-7046, after the
            // check pass's continue at 7027).
            if (piped && !outcome.pre_check) {
                pipe_prepare_slicing(rs);   // once per run: here only for a run that skipped the plate checks
                slicer_cli::pipe_set_plate_info(std::max(1, outcome.plate_id), std::max(1, outcome.plate_count));
                slicer_cli::pipe_update(4, "Slicing begins");
            }
            if (o.progress || piped) {
                print.set_status_callback([&outcome, &o, piped](const Slic3r::PrintBase::SlicingStatus& status) {
                    if (piped)
                        slicer_cli::pipe_update(status.percent, status.text, status.warning_step);
                    emit_status_warning(status);
                    if (!o.progress)
                        return;
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

            // The per-filament and speed tables the brim width reads
            // (Brim.cpp, through Model::extruderParamsMap and
            // printSpeedMap): both official CLIs fill them between apply()
            // and process() (BambuStudio.cpp 7074-7075 at 5873b5f;
            // OrcaSlicer.cpp 6065-6066 at 31f6803). Left empty, the brim
            // takes no account of the filament (TPU narrower, PETG/PCTG
            // wider) or of the print speed.
            {
                int filament_count = 1;
                if (auto* fc = config.option<Slic3r::ConfigOptionStrings>("filament_colour", false))
                    filament_count = std::max<int>(1, (int)fc->values.size());
                Slic3r::Model::setExtruderParams(config, filament_count);
                Slic3r::Model::setPrintSpeedTable(config, print.config());
#ifdef ENGINE_ORCA
                std::cout << "  [orca] setExtruderParams/setPrintSpeedTable (filaments="
                          << filament_count << ")\n";
#endif
            }
            // --no-check and --allow-mix-temp (BambuStudio.cpp 6979-6982;
            // OrcaSlicer.cpp 5968-5971); only when given, so the default
            // stays the engine's own.
            const slicer_cli::PlateLoopSwitches switches = slicer_cli::plate_loop_switches(o);
            outcome.no_check = switches.no_check;
            if (o.given_flag("no_check"))
                print.set_no_check_flag(switches.no_check);
            if (o.given_flag("allow_mix_temp"))
                print.set_check_multi_filaments_compatibility(!switches.allow_mix_temp);

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
                        points.push_back(json{{"x_mm", Slic3r::unscaled<double>(p.x()) - plate_origin.x()},
                                              {"y_mm", Slic3r::unscaled<double>(p.y()) - plate_origin.y()}});
                    hulls.push_back(points);
                }
                if (!hulls.empty()) e["collision_hulls"] = hulls;
                json heights = json::array();
                for (const auto& [poly, height] : height_polygons) {
                    json points = json::array();
                    for (const auto& p : poly.points)
                        points.push_back(json{{"x_mm", Slic3r::unscaled<double>(p.x()) - plate_origin.x()},
                                              {"y_mm", Slic3r::unscaled<double>(p.y()) - plate_origin.y()}});
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
            // --no-check lets a layer height over the printer's limit pass
            // as a warning (BambuStudio.cpp 6985-6987; OrcaSlicer.cpp 5992-5994).
            if (!validation_result.string.empty() && switches.no_check &&
                validation_result.type == Slic3r::STRING_EXCEPT_LAYER_HEIGHT_EXCEEDS_LIMIT) {
                emit_validation_event(validation_result);
                std::cout << "Validation warning: " << validation_result.string << "\n";
                validation_result.string.clear();
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
            // --slice 0 on several plates: the check pass ends here for this
            // plate, once its bed check, apply and validate passed
            // (BambuStudio.cpp 7027-7028; OrcaSlicer.cpp 6034-6035).
            if (outcome.pre_check)
                return 0;

            // --load-slicedata: the plate's saved slicing, else a normal
            // slice (BambuStudio.cpp 7076-7099; OrcaSlicer.cpp 6067-6090).
            const int cache_plate = plate_id > 0 ? plate_id : 1;
            const std::string load_dir = slicer_cli::slicedata_dir(o, "load_slicedata", cache_plate);
            std::cout << "Slicing...\n";
            if (!load_dir.empty() && print.load_cached_data(load_dir) == 0) {
                emit_event({{"event","model_loaded"}, {"tag","SliceDataLoaded"}, {"path", load_dir},
                            {"message","Loaded the saved slicing of plate " + std::to_string(cache_plate)}});
                slicer_cli::pipe_update(69, "Cache data loaded");   // BambuStudio.cpp 7087-7090
                print.process(nullptr, true);
            } else {
                if (!load_dir.empty())
                    emit_event({{"event","model_loaded"}, {"tag","SliceDataNotLoaded"}, {"path", load_dir},
                                {"message","No usable saved slicing for plate " + std::to_string(cache_plate) + "; slicing it normally"}});
                print.process();
            }

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
                // GCode::do_export makes the file's folder when it is missing and
                // throws on a bare file name, whose folder is "" (GCode.cpp
                // 1793-1797 at 5873b5f): the current folder is named for it.
                const std::string export_path = boost::filesystem::path(output_file).has_parent_path()
                                                    ? output_file : (boost::filesystem::path(".") / output_file).string();
                print.export_gcode(export_path, &gcode_result, nullptr);

                // The result object is fully populated by the export. Drain
                // every check the GUI would show a human before reporting
                // success — a run can exit 0 and still carry warnings,
                // toolpath conflicts or printable-area failures.
                emit_gcode_result_diagnostics(gcode_result);
                if (o.slice_mode) {
                    record_plate_statistics(print, model, config, rs.base, gcode_result, outcome);
                    outcome.sliced_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now() - plate_started).count();
                    // --mstpp: a plate that took longer ends the run, listed with
                    // its note (BambuStudio.cpp 7364-7373, a limit in seconds;
                    // OrcaSlicer.cpp 6202-6211 compares the milliseconds with the
                    // value as given).
                    if (switches.max_slicing_time_s != 0) {
#ifdef ENGINE_ORCA
                        const long long limit = switches.max_slicing_time_s;
#else
                        const long long limit = (long long)switches.max_slicing_time_s * 1000;
#endif
                        if (outcome.sliced_time_ms > limit) {
                            outcome.warning_message = "plate " + std::to_string(cache_plate) + "'s slice time " +
                                                      std::to_string(outcome.sliced_time_ms) + " exceeds the limit " +
                                                      std::to_string(limit) + ", return error.";
                            outcome.exported = true;
                            set_outcome_failure(outcome, CLI_SLICING_TIME_EXCEEDS_LIMIT);
                            return 1;
                        }
                    }
                    if (!post_slice_checks(print, model, gcode_result, outcome))
                        return 1;
                    outcome.exported = true;
                    if (!o.export_3mf.empty())
                        record_plate_for_export(print, model, config, gcode_result, output_file, outcome,
                                                plate_origin);
                    if (o.progress)
                        emit_progress(outcome, 100, "Slicing finished");
                    slicer_cli::pipe_update(100, "Slicing finished");   // BambuStudio.cpp 7242-7245
                }
                {
                    // --export-slicedata (BambuStudio.cpp 7247-7262; OrcaSlicer.cpp 6183-6199).
                    if (const std::string export_dir = slicer_cli::slicedata_dir(o, "export_slicedata", cache_plate);
                        !export_dir.empty()) {
                        const bool with_space = Slic3r::get_logging_level() >= 4;
                        // Print::export_cached_data makes the plate's folder
                        // only, so the folder given on the command line is made
                        // first (the official fails with -53 when it is missing).
                        {
                            boost::system::error_code mk;
                            boost::filesystem::create_directories(boost::filesystem::path(export_dir).parent_path(), mk);
                        }
#ifdef ENGINE_ORCA
                        const int ret = print.export_cached_data(export_dir, with_space);
#else
                        int obj_cached_cnt = 0;
                        const int ret = print.export_cached_data(export_dir, obj_cached_cnt, with_space);
#endif
                        if (ret != 0) {
                            boost::system::error_code rm;
                            boost::filesystem::remove_all(export_dir, rm);
                            set_outcome_failure(outcome, ret, "Could not write the slicing data to " + export_dir + ".");
                            return 1;
                        }
                        emit_event({{"event","export"}, {"tag","SliceDataExported"}, {"path", export_dir},
                                    {"message","Wrote the slicing data of plate " + std::to_string(cache_plate) + " to " + export_dir}});
                    }
                }

                std::cout << "✓ G-code export complete!\n";
                std::cout << "\nOutput file: " << output_file << "\n";

                // The actions given after --slice, once the last plate has
                // sliced (see run_model_action_step).
                if (o.slice_mode && outcome.plate_index == outcome.plate_count &&
                    slicer_cli::has_model_actions(o, slicer_cli::ActionPhase::AfterSlice) &&
                    !run_model_action_step(slicer_cli::ActionPhase::AfterSlice))
                    return 1;

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
static int run_info(const std::string& argv0, const std::string& path, const std::string& printer_preset) {
    json out;
    out["file"] = path;
    // The non-throwing check: a path that cannot be queried (permissions, a
    // failing mount) is still one JSON document, never an uncaught exception.
    boost::system::error_code exists_ec;
    const bool exists = boost::filesystem::exists(path, exists_ec);
    if (exists_ec && exists_ec != boost::system::errc::no_such_file_or_directory) {
        out["error"] = cli_error_sentence(CLI_ENVIRONMENT_ERROR) + " Cannot check " + path + ": " + exists_ec.message();
        std::cout << out.dump(2, ' ', false, json::error_handler_t::replace) << std::endl;
        return CLI_ENVIRONMENT_ERROR;
    }
    if (!exists) {
        out["error"] = cli_error_sentence(CLI_FILE_NOTFOUND);
        std::cout << out.dump(2, ' ', false, json::error_handler_t::replace) << std::endl;
        return CLI_FILE_NOTFOUND;
    }
    const bool is_3mf = input_is_3mf(path);
    // The model file kinds this engine reads (cli_model_load.cpp), by extension.
    std::string kind = "unknown";
    if (slicer_cli::is_loadable_model_file(path)) {
        kind = boost::filesystem::path(path).extension().string().substr(1);
        boost::algorithm::to_lower(kind);
    }
    out["kind"] = kind;

    std::string printer_model;
    std::string maker_app;
    // A 3MF the slicer cannot open is not inspected: the archive must open and
    // its model part (3D/3dmodel.model, the part load_bbs_3mf reads the
    // objects from) must decompress intact, as the slice path would refuse it (-2).
    // An STL is loaded with the slice path's own loader (Slic3r::load_stl), so
    // a folder named .stl or an empty or malformed file is refused here as the
    // slice would refuse it (-2).
    if (input_is_stl(path)) {
        Slic3r::Model stl_model;
        bool loaded = false;
        try {
            loaded = Slic3r::load_stl(path.c_str(), &stl_model) && !stl_model.objects.empty();
        } catch (...) {
            loaded = false;
        }
        if (!loaded) {
            out["kind"] = "stl";
            out["error"] = cli_error_sentence(CLI_DATA_FILE_ERROR) + " " + path + " is not a readable STL file.";
            std::cout << out.dump(2, ' ', false, json::error_handler_t::replace) << std::endl;
            return CLI_DATA_FILE_ERROR;
        }
    }
    // Any other model file: read with the slice path's loader.
    if (!is_3mf && !input_is_stl(path) && kind != "unknown") {
        bool loaded = false;
        try {
            Slic3r::Model m = boost::algorithm::iends_with(path, ".step") || boost::algorithm::iends_with(path, ".stp")
                ? Slic3r::Model::read_from_step(path, Slic3r::LoadStrategy::LoadModel, nullptr, nullptr,
                                                [](Slic3r::Step&, double& l, double& a, bool& s) -> int { l = 0.003; a = 0.5; s = false; return 1; },
                                                0.003, 0.5, false)
                : Slic3r::Model::read_from_file(path, nullptr, nullptr, Slic3r::LoadStrategy::LoadModel);
            loaded = !m.objects.empty();
        } catch (...) {
            loaded = false;
        }
        if (!loaded) {
            out["error"] = cli_error_sentence(CLI_DATA_FILE_ERROR) + " " + path + " is not a readable " + kind + " file.";
            std::cout << out.dump(2, ' ', false, json::error_handler_t::replace) << std::endl;
            return CLI_DATA_FILE_ERROR;
        }
    }
    if (is_3mf && !zip_member_readable(path, "3D/3dmodel.model")) {
        out["error"] = cli_error_sentence(CLI_DATA_FILE_ERROR) + " " + path +
                       " is not a readable 3MF archive with a 3D/3dmodel.model part.";
        std::cout << out.dump(2, ' ', false, json::error_handler_t::replace) << std::endl;
        return CLI_DATA_FILE_ERROR;
    }
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
        // A geometry-only 3MF is one plate, as the slice takes it.
        out["plates"] = slicer_cli::classify_3mf(path) == slicer_cli::ThreeMfKind::GeometryOnly
                            ? 1 : std::max(1, count_3mf_plates(path));

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
    // A model file with --printer-preset: the printer is the preset's model,
    // from either engine's profiles tree.
    if (printer_model.empty() && !printer_preset.empty()) {
        const boost::filesystem::path exe_dir = engine_executable_dir(argv0.c_str());
        printer_model = preset_printer_model(this_engine_profiles_dir(argv0), printer_preset);
        if (printer_model.empty())
#ifdef ENGINE_ORCA
            printer_model = preset_printer_model(engine_profiles_dir(exe_dir), printer_preset);
#else
            printer_model = preset_printer_model(orca_profiles_dir(exe_dir), printer_preset);
#endif
        out["printer_preset"] = printer_preset;
        out["printer_model"]  = printer_model.empty() ? json(nullptr) : json(printer_model);
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
static int export_sliced_3mf(const CliOptions& o, const boost::filesystem::path& outdir, RunState& rs,
                             std::vector<PlateOutcome>& outcomes, const Slic3r::DynamicPrintConfig& project,
                             std::string& error) {
    using namespace Slic3r;
    const std::string path = (outdir / o.export_3mf).string();
    Model& model = rs.model;
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
        run_staging.add((boost::filesystem::temp_directory_path() /
                         boost::filesystem::unique_path("slicer_cli_export-%%%%%%%%")).string());
    model.set_backup_path(export_backup);   // removed with the run's other staging folders
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
    const int count = std::max<int>({1, (int)rs.plates.size(), (int)rs.plate_data.size()});
    for (int k = 0; k < count; ++k) {
        PlateData* pd = k < (int)rs.plate_data.size() && rs.plate_data[k] != nullptr ? new PlateData(*rs.plate_data[k])
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
                if (rs.plate(k + 1).is_member(inst) && (k < (int)rs.plates.size() || count == 1)) {
                    plates[k]->objects_and_instances.emplace_back(int(oi), int(ii));
                    break;
                }
        }
    // Each sliced plate's slice result (BambuStudio.cpp 7483-7532).
    for (const PlateOutcome& out : outcomes) {
        if (!out.plate_data) continue;
        const int idx = geometry_input && !g_assemble ? 0 : out.plate_id - 1;
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
        pd->limit_filament_maps      = sliced.limit_filament_maps;
        pd->layer_filaments          = sliced.layer_filaments;
        pd->filament_change_sequence = sliced.filament_change_sequence;
        pd->nozzle_change_sequence   = sliced.nozzle_change_sequence;
        pd->optimal_assignment       = sliced.optimal_assignment;
        pd->filament_maps            = sliced.filament_maps;
        pd->is_support_used          = sliced.is_support_used;
        pd->slice_filaments_info     = sliced.slice_filaments_info;
        pd->skipped_objects          = sliced.skipped_objects;
        pd->printer_model_id         = sliced.printer_model_id;
        pd->nozzle_diameters         = sliced.nozzle_diameters;
#ifdef ENGINE_BAMBU
        // The routing the slice derived from a supplied nozzle map, on this
        // plate only: store_bbs_3mf writes a plate's filament_map_mode and
        // filament_maps from its config (bbs_3mf.cpp 8260-8279 at 5873b5f;
        // PartPlate::set_filament_maps keeps the map there, PartPlate.cpp
        // 3860-3863), and a plate mode other than Default wins over the
        // project's (PartPlate.cpp 278-289). The objects keep their own
        // filaments, so an object with copies on other plates stays as it is.
        if (!out.derived_filament_map.empty()) {
            ConfigSubstitutionContext mode_subst(ForwardCompatibilitySubstitutionRule::Enable);
            pd->config.option<ConfigOptionInts>("filament_map", true)->values = out.derived_filament_map;
            pd->config.set_deserialize("filament_map_mode", out.derived_filament_map_mode, mode_subst);
        }
#endif
    }
    // printer_model_id per plate after a printer change (BambuStudio.cpp 7524-7525).
    if (!rs.settings_merge.printer_model_id.empty())
        for (PlateData* pd : plates)
            pd->printer_model_id = rs.settings_merge.printer_model_id;
    if (g_assemble)
        for (int k = 0; k < count && k < (int)g_assemble->plates.size(); ++k)
            plates[k]->plate_name = g_assemble->plates[k].plate_name;

    // The presets the project carries, and the "(auto)" process a printer
    // change makes (BambuStudio.cpp 3094).
    std::vector<Preset*> project_presets;
    for (Preset& p : rs.project_presets_kept)
        project_presets.push_back(&p);
    std::unique_ptr<Preset> auto_process;
    if (rs.settings_merge.new_preset) {
        auto_process = std::make_unique<Preset>(*rs.settings_merge.new_preset);
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
    if (!geometry_input && !g_assemble && o.normalize_legacy_gcode)
        normalize_legacy_gcode_tokens(config, /*report_event=*/false);

    // Each sliced plate's Metadata/plate_N.json; an empty box for the others
    // (BambuStudio.cpp 7976-7988).
    std::vector<std::unique_ptr<PlateBBoxData>> bbox_storage;
    std::vector<PlateBBoxData*> plate_bboxes;
    for (size_t i = 0; i < plates.size(); ++i) {
        PlateBBoxData* found = nullptr;
        for (const PlateOutcome& out : outcomes)
            if (out.plate_bbox && (geometry_input && !g_assemble ? i == 0 : out.plate_id - 1 == int(i)))
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

    // --load-assemble-list: the list and every part it names, built before
    // the first plate (BambuStudio.cpp 2165-2188; OrcaSlicer.cpp 1812-1834).
    // A refusal names no plate (record_exit_reson(..., 0, ...)).
    const std::string assemble_file = assemble_list_file(o);
    if (!assemble_file.empty()) {
        auto list = std::make_unique<slicer_cli::AssembleList>();
        const slicer_cli::StepResult built = slicer_cli::load_assemble_list(assemble_file, *list);
        if (built.code != 0) {
            write_result_json(outdir.string(), built.code, 0, cli_error_sentence(built.code) + " " + built.message, {}, 0, 0);
            std::cerr << "Error: " << built.message << "\n";
            emit_event({{"event","input_error"}, {"tag","AssembleListRefused"}, {"path", assemble_file},
                        {"message", built.message}});
            return built.code;
        }
        g_assemble = std::move(list);
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
    if (g_assemble) {
        plate_count = int(g_assemble->plates.size());
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
    int plan_code = 0;
    std::string plan_error;
    const bool trailing = per_plate_load && !g_assemble && trailing_models_port(o);
    std::optional<PlateOutcome> prepare_failure;
    const auto prepare_run = [&](int prepare_plate) {
        PlateOutcome prepare_outcome;
        prepare_outcome.prepare_pass = true;
        prepare_outcome.plate_id = prepare_plate;
        const int rc = slice_one_plate(o, calib_params, rs, prepare_plate, std::string(), prepare_outcome);
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
        prepare_run(0);
        if (plan_code == 0)
            plate_count = rs.plan.plate_count;
    }

    if (plan_code == 0 && o.slice_plate > plate_count) {
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
    // --export-3mf NAME is written into --outputdir next to result.json and
    // plate_N.gcode; a NAME that is one of them would overwrite it (or be
    // overwritten). Compared without case: Windows and macOS file systems
    // treat Result.json and result.json as one file. Links are followed
    // (the NAME's own link chain, then weakly_canonical for every existing
    // part),
    // and two existing names for one file (a hard link) are caught by
    // fs::equivalent, so a NAME that is another name for one of them is
    // refused too.
    // The check runs here, once the plate plan is final: the global arrange
    // (--slice 0 --arrange 1) can add plates the pre-arrange count does not
    // know (BambuStudio.cpp 5627-5722; OrcaSlicer.cpp 4887-4983), and a
    // plate_<n>.gcode it then writes would be overwritten by the export.
    // `plates` is the set this run slices.
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
        for (int p : plates)
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
            rc = slice_one_plate(o, calib_params, rs, per_plate_load ? plates[i] : 0, outcome.gcode_path, outcome);
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
            outcome.error_string = cli_error_sentence(outcome.cli_code);
        // The refused plate starts without a G-code, as in the slice pass.
        boost::system::error_code ignored;
        fs::remove(outcome.gcode_path, ignored);
        outcomes.push_back(outcome);
        code = outcome.cli_code;
        error_string = outcome.error_string;
        break;
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
        const int rc = slice_one_plate(o, calib_params, rs, per_plate_load ? plates[i] : 0,
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
            code = export_sliced_3mf(o, outdir, rs, outcomes, project, export_error);
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
    // --pipe: the last step, then the pipe closes (BambuStudio.cpp 8234-8241;
    // a failed run closes it without one, flush_and_exit 457-460).
    if (code == 0)
        slicer_cli::pipe_update(100, "All done, Success");
    slicer_cli::pipe_stop();
    return code;
}

static int run_cli_slice(const CliOptions& o, Slic3r::Calib_Params& calib_params) {
    if (o.slice_mode)
        return run_slice_mode(o, calib_params);
    PlateOutcome outcome;
    outcome.plate_id = o.plate_id;
    RunState rs;
    const int rc = slice_one_plate(o, calib_params, rs, o.plate_id, o.output_file, outcome);
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
    // UTF-8 everywhere, as the official command lines start: on Windows the
    // arguments come from the wide command line as UTF-8 (BambuStudio.cpp
    // 8636-8660, the wide entry of BambuStudio_app_msvc.cpp; the same in
    // OrcaSlicer), and boost::filesystem paths take UTF-8 strings
    // (nowide_filesystem, BambuStudio.cpp 1510-1523; OrcaSlicer.cpp 1233-1246).
    // Elsewhere both are no-ops.
    boost::nowide::args utf8_args(argc, argv);
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

    {
        slicer_cli::ParseRefusal refusal;
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
                write_result_json(dir, refusal.code, o.slice_mode ? std::max(0, o.slice_plate) : 0,
                                  cli_error_sentence(refusal.code) + " " + refusal.message, {}, 0, 0);
                return refusal.code;
            }
            return 1;
        }
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
        write_result_json(dir, code, o.slice_plate, cli_error_sentence(code) + " " + sentence, {}, 0, 0);
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
        g_arrange_switches.enable_timelapse            = flag("enable_timelapse", false);
        g_arrange_switches.allow_rotations             = flag("allow_rotations", false);
        g_arrange_switches.allow_multicolor_oneplate   = flag("allow_multicolor_oneplate", true);
        g_arrange_switches.avoid_extrusion_cali_region = flag("avoid_extrusion_cali_region", false);
#ifdef ENGINE_BAMBU
        g_skip_useless_pick = flag("skip_useless_pick", false);
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
    if (!mode_args.engine_info_file.empty()) {
        boost::log::core::get()->set_logging_enabled(false);
        return run_info(o.argv0, mode_args.engine_info_file, o.printer_preset);
    }

    if (mode_args.list_presets) {
        boost::log::core::get()->set_logging_enabled(false);
        return run_list_presets(o, mode_args.list_printer);
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
        if (project_3mf_input)
            return refuse_run(CLI_INVALID_PARAMS, "--printer-preset/--process-preset/--filament-preset apply to "
                                                  "an STL; a 3MF carries its own settings");
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
    // --slice it prints the same sentence and exits 1, unchanged.
    if (layout_plan_mode && !layout_json_file.empty())
        return refuse_run(CLI_INVALID_PARAMS, "--layout-plan and --layout are mutually exclusive");

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

    return run_cli_slice(o, calib_params);
}
