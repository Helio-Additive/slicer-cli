#pragma once

#include <cstdio>
#include <string>
#include <vector>

namespace slicer_cli { namespace diagnostics {

// Keep the default synchronized C/C++ standard streams. One fwrite holds the
// stdout FILE lock also used by printf and synchronized std::cout (including
// the engine console sink). An event-only mutex cannot protect those writers.
// The leading newline separates an event even from an unfinished text line.
/// Writes one flushed, newline-delimited SLICER_EVENT record to stdout now.
inline void write_event_now(const std::string& payload) {
    const std::string record = "\n[[SLICER_EVENT]] " + payload + '\n';
    std::fwrite(record.data(), 1, record.size(), stdout);
    std::fflush(stdout);
}

// While set, every event (emit_event in main.cpp and slicer_cli::emit both
// end in write_event) is kept here instead of written: --slice 0's check pass
// over several plates (run_slice_mode), whose plates the slice pass runs
// again. One run, one thread.
inline std::vector<std::string>* held_events = nullptr;

/// Writes one SLICER_EVENT record, or holds it while held_events is set.
inline void write_event(const std::string& payload) {
    if (held_events) {
        held_events->push_back(payload);
        return;
    }
    write_event_now(payload);
}

// Fatal signals retain the platform's native disposition. Do not write crash
// records here: a closed or full stdout pipe can mask or delay termination.

}} // namespace slicer_cli::diagnostics
