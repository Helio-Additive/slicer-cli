// cli_simplify.hpp — the desktop Simplify tool, headless.
//
// --simplify LEVEL, --simplify-ratio PCT and --simplify-triangles N reduce the
// triangle count of the model parts the way the desktop's Simplify dialog does
// (GLGizmoSimplify.cpp at 56a57d0 / 8500fcd / bac6fd5): the engine's quadric
// edge collapse, bounded by a maximal error (the dialog's "Detail level") or by
// a wanted triangle count (its "Decimate ratio"). The step runs after the
// official transforms and before the objects are placed, so every later check,
// export and slice sees the reduced meshes.
#pragma once

#include <set>
#include <string>

#include "cli_run_steps.hpp"

namespace Slic3r { class Model; }
struct CliOptions;

namespace slicer_cli {

/// The sentence that refuses --simplify-keep-paint in a build whose engine
/// cannot carry paint onto a changed mesh (the Snapmaker build), or "" where
/// the flag works. --list-flags reports it as the flag's refusal.
std::string simplify_keep_paint_refusal();

/// The sentence that refuses the simplify flags as given (two modes at once, a
/// lever without a mode, a lever this build cannot honour), or "" when they agree.
std::string simplify_flags_refusal(const CliOptions& o);

/// Reduces the selected model parts as the flags ask. One `mesh_simplified`
/// event per part. Returns the official code and sentence on a refusal.
/// `known_object_ids` is the set --simplify-objects is checked against: the
/// project's ids on every plate (cli_engine_info.hpp), so an id on another
/// plate than the one sliced is left alone, not refused.
StepResult simplify_meshes(const CliOptions& o, Slic3r::Model& model, const std::set<int>& known_object_ids);

/// The desktop's suggestion (GLGizmoSimplify.cpp 97-101 at 56a57d0): one
/// `warning` / `LargeMesh` event per single-part object of a million triangles
/// or more, as the model stands after the simplify step.
void warn_large_meshes(const Slic3r::Model& model);

} // namespace slicer_cli
