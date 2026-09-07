#pragma once

#include <cstdio>
#include <string>

namespace slicer_cli { namespace diagnostics {

// Keep the default synchronized C/C++ standard streams. One fwrite holds the
// stdout FILE lock also used by printf and synchronized std::cout (including
// the engine console sink). An event-only mutex cannot protect those writers.
// The leading newline separates an event even from an unfinished text line.
inline void write_event(const std::string& payload) {
    const std::string record = "\n[[SLICER_EVENT]] " + payload + '\n';
    std::fwrite(record.data(), 1, record.size(), stdout);
    std::fflush(stdout);
}

// Fatal signals retain the platform's native disposition. Do not write crash
// records here: a closed or full stdout pipe can mask or delay termination.

}} // namespace slicer_cli::diagnostics
