// cli_events.hpp — the [[SLICER_EVENT]] stream, for front-end modules.
//
// Same record as cli_engine_events.cpp's emit_event: one JSON object per line on stdout,
// invalid UTF-8 replaced, and never the reason a run fails.
#pragma once

#include <string>

#include <nlohmann/json.hpp>

#include "diagnostic_output.hpp"

namespace slicer_cli {

/// Writes one structured event; a diagnostic never throws.
inline void emit(const nlohmann::json& payload) {
    try {
        diagnostics::write_event(payload.dump(-1, ' ', false, nlohmann::json::error_handler_t::replace));
    } catch (...) {
    }
}

} // namespace slicer_cli
