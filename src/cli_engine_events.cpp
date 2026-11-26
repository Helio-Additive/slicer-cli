#include "cli_engine_events.hpp"

#include "libslic3r/GCode/GCodeProcessor.hpp"
#include "diagnostic_output.hpp"
#include <iostream>
#include <vector>
// Boost.Log bridge: libslic3r raises most of its diagnostics through
// BOOST_LOG_TRIVIAL and never through any callback. `libslic3r_core` already
// links Boost::log and Boost::log_setup unconditionally (CMakeLists.txt — the
// same list for ENGINE=bambu and ENGINE=orca), so attaching one more sink adds
// no new dependency on either engine.
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

namespace slicer_cli {

using json = nlohmann::json;

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

static const char* string_exception_tag(Slic3r::StringExceptionType t) {
    switch (t) {
        case Slic3r::STRING_EXCEPT_NOT_DEFINED:                   return "STRING_EXCEPT_NOT_DEFINED";
        case Slic3r::STRING_EXCEPT_FILAMENT_NOT_MATCH_BED_TYPE:   return "STRING_EXCEPT_FILAMENT_NOT_MATCH_BED_TYPE";
#ifndef ENGINE_SNAPMAKER
        // Snapmaker Orca names this one STRING_EXCEPT_FILAMENTS_MIXING_TEMP
        // instead (references/SnapmakerOrca/src/libslic3r/PrintBase.hpp:1-12 at
        // bac6fd5), tagged below.
        case Slic3r::STRING_EXCEPT_FILAMENTS_DIFFERENT_TEMP:      return "STRING_EXCEPT_FILAMENTS_DIFFERENT_TEMP";
#else
        case Slic3r::STRING_EXCEPT_FILAMENTS_MIXING_TEMP:         return "STRING_EXCEPT_FILAMENTS_DIFFERENT_TEMP";
#endif
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
std::vector<json>& recorded_config_substitutions() {
    static std::vector<json> recorded;
    return recorded;
}

void emit_config_substitutions(const Slic3r::ConfigSubstitutionContext& ctx,
                               const std::string& source) {
    for (const auto& sub : ctx.substitutions) {
#if defined(ENGINE_ORCA) && !defined(ENGINE_SNAPMAKER)
        // OrcaSlicer from 2.5 says each replaced value on stderr and lists it
        // in result.json with a stable class (cli_report_config_substitutions,
        // OrcaSlicer.cpp 534-546 at f92a7c0).
        {
            const std::string key = sub.opt_def ? sub.opt_def->opt_key : std::string{};
            const std::string new_value = sub.new_value ? sub.new_value->serialize() : std::string{};
            std::cerr << "warning: " << source << ": value \"" << sub.old_value << "\" for " << key
                      << " is not valid, using \"" << new_value << "\" instead\n";
            recorded_config_substitutions().push_back(json{{"class", "config_value_substituted"}, {"file", source},
                                                           {"key", key}, {"old_value", sub.old_value},
                                                           {"new_value", new_value}});
        }
#endif
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
static const char* slice_warning_level_tag(int level) {
    switch (level) {
        case 0:  return "tip";
        case 1:  return "warning";
        case 2:  return "error";
        default: return "unknown";
    }
}

/// Emits structured flags and details from an engine G-code check result.
#ifndef ENGINE_SNAPMAKER
// GCodeProcessorResult::gcode_check_result is OrcaSlicer 2.4 only; the Snapmaker
// fork's GCodeProcessorResult carries no post-slice check result
// (references/SnapmakerOrca/src/libslic3r/GCode/GCodeProcessor.hpp at bac6fd5),
// so the engine reports no check outcome and no such event is emitted.
static void emit_gcode_check_result(const Slic3r::GCodeProcessorResult& r) {
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
#endif // ENGINE_SNAPMAKER

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
#ifndef ENGINE_SNAPMAKER
    emit_gcode_check_result(r);
#endif
#ifndef ENGINE_SNAPMAKER
    // `filament_printable_reuslt` (sic — upstream spelling) carries a
    // has_value() helper, but it is `const` on BambuStudio and NOT const on
    // OrcaSlicer, so calling it through this const& breaks the orca build.
    // Both engines define it as exactly this emptiness test. It is OrcaSlicer
    // 2.4 only: the Snapmaker fork's GCodeProcessorResult has no such field
    // (references/SnapmakerOrca/src/libslic3r/GCode/GCodeProcessor.hpp at
    // bac6fd5), so this engine reports no unprintable-filament result.
    if (!r.filament_printable_reuslt.conflict_filament.empty()) {
        json e;
        e["event"]             = "filament_unprintable";
        e["tag"]               = "FilamentPrintableResult";
        e["conflict_filament"] = r.filament_printable_reuslt.conflict_filament;
        e["plate_name"]        = r.filament_printable_reuslt.plate_name;
        e["message"]           = "One or more filaments cannot be printed on their assigned extruder";
        emit_event(e);
    }
#endif
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
#ifndef ENGINE_SNAPMAKER
    // limit_filament_maps is OrcaSlicer 2.4 only (absent from the Snapmaker
    // fork's GCodeProcessorResult at bac6fd5), so this engine never reports it.
    if (!r.limit_filament_maps.empty()) {
        emit_event({{"event","filament_map_limited"},
                    {"tag","LimitFilamentMaps"},
                    {"limit_filament_maps", r.limit_filament_maps},
                    {"message","The engine constrained the filament-to-extruder map"}});
    }
#endif
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
static const char* boost_severity_tag(boost::log::trivial::severity_level level) {
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
#ifdef ENGINE_SNAPMAKER
    // StringObjectException::is_warning is OrcaSlicer 2.4 only; the Snapmaker
    // fork's struct has no such field
    // (references/SnapmakerOrca/src/libslic3r/PrintBase.hpp:14-21 at bac6fd5),
    // so every exception reaching here is an error.
    e["event"]    = "validation_error";
#else
    e["event"]    = v.is_warning ? "validation_warning" : "validation_error";
#endif
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

} // namespace slicer_cli
