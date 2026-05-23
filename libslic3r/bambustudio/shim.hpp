#pragma once
// Thin C++ shim over libslic3r. Single entrypoint, JSON-string in,
// callbacks out via the cxx-bridged Rust EventSink.
//
// All libslic3r includes (Model.hpp, Print.hpp, Preset.hpp, …) stay on the
// C++ side of this header — Rust never sees them. That keeps the FFI
// surface tiny and immune to libslic3r template churn.

#include "rust/cxx.h"

struct EventSink;  // opaque Rust type, defined by cxx-bridge codegen

int32_t slicer_run(::rust::Str job_json, EventSink& sink);

/// Returns a JSON array of preset names for `kind` ("machine"|"filament"|"process").
::rust::String slicer_list_presets(::rust::Str kind);

/// Returns the JSON object for the named preset, or "null" if not found.
::rust::String slicer_get_preset(::rust::Str kind, ::rust::Str name);
