// cli_simplify.cpp — the desktop Simplify tool, headless (see cli_simplify.hpp).
#include "cli_simplify.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "libslic3r/Model.hpp"
#include "libslic3r/QuadricEdgeCollapse.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/Utils.hpp"

#include "cli_events.hpp"
#include "cli_options.hpp"

namespace slicer_cli {

namespace {

// The dialog's five detail levels and the maximal quadric error each one
// allows (GLGizmoSimplify.cpp 301-307 and 317-321 at 56a57d0; OrcaSlicer
// 284-288 at 8500fcd; the Snapmaker app is Orca's). "medium" is the dialog's
// default (GLGizmoSimplify.hpp 68).
struct DetailLevel {
    const char* name;
    float       max_error;
};

const DetailLevel kDetailLevels[] = {
    {"extra-high", 1e-3f},
    {"high",       1e-2f},
    {"medium",     0.1f},
    {"low",        0.5f},
    {"extra-low",  1.f},
};

const DetailLevel* find_level(const std::string& name) {
    for (const DetailLevel& level : kDetailLevels)
        if (name == level.name)
            return &level;
    return nullptr;
}

// Decimate ratio to wanted count, the dialog's rule
// (GLGizmoSimplify::Configuration::fix_count_by_ratio, 859-867 at 56a57d0).
// The parser keeps the ratio inside (0, 100), so the two end cases there
// (0 = keep all, 100 = keep none) never reach this.
uint32_t wanted_count_by_ratio(size_t triangle_count, double decimate_ratio) {
    return static_cast<uint32_t>(std::round(triangle_count * (100. - decimate_ratio) / 100.));
}

// What the desktop asks about before it keeps paint across the new mesh
// (GLGizmoSimplify.cpp 64-67 at 56a57d0). Spelled on the layers themselves:
// the Orca engines have no is_fuzzy_skin_facets_painted (Model.hpp 1012-1014
// at 8500fcd), only the layer.
bool has_any_painting(const Slic3r::ModelVolume& mv) {
    return !mv.mmu_segmentation_facets.empty() || !mv.supported_facets.empty() || !mv.seam_facets.empty() ||
           !mv.fuzzy_skin_facets.empty();
}

std::string join_ints(const std::set<int>& values) {
    std::ostringstream out;
    for (int v : values)
        out << (out.tellp() > 0 ? ", " : "") << v;
    return out.str();
}

int modes_given(const CliOptions::Simplify& s) {
    return int(!s.level.empty()) + int(s.ratio >= 0.) + int(s.triangles >= 0);
}

} // namespace

std::string simplify_flags_refusal(const CliOptions& o) {
    const CliOptions::Simplify& s = o.simplify;
    if (modes_given(s) > 1)
        return "--simplify, --simplify-ratio and --simplify-triangles are mutually exclusive; give one of them";
    if (modes_given(s) == 0) {
        if (s.objects_given)
            return "--simplify-objects needs --simplify, --simplify-ratio or --simplify-triangles";
        if (s.keep_paint)
            return "--simplify-keep-paint needs --simplify, --simplify-ratio or --simplify-triangles";
        return "";
    }
#ifdef ENGINE_SNAPMAKER
    // Snapmaker Orca's engine has neither Orca's save_painting/restore_painting
    // (Model.hpp 884-887 at 8500fcd) nor Bambu's reproject_paint_keep (Model.hpp
    // 1057 at 56a57d0); its Simplify clears the paint (GLGizmoSimplify.cpp 538
    // at bac6fd5, clear_before_change_mesh).
    if (s.keep_paint)
        return "--simplify-keep-paint is not available in slicer_cli-snapmaker: its engine has no way to carry "
               "paint onto a changed mesh. Drop the flag to simplify without the paint, or keep it with "
               "slicer_cli-orcaslicer (OrcaSlicer) for a file an Orca printer can take, or slicer_cli "
               "(BambuStudio) for a Bambu Studio project";
#endif
    return "";
}

StepResult simplify_meshes(const CliOptions& o, Slic3r::Model& model) {
    StepResult r;
    const CliOptions::Simplify& s = o.simplify;
    if (modes_given(s) == 0)
        return r;

    // --simplify-objects: the 3MF's object ids, the ids --skip-objects takes
    // (ModelInstance::loaded_id). An id no object has is a mistake, not a no-op.
    const std::set<int> wanted(s.objects.begin(), s.objects.end());
    if (!wanted.empty()) {
        std::set<int> present, missing;
        for (const Slic3r::ModelObject* obj : model.objects)
            for (const Slic3r::ModelInstance* inst : obj->instances)
                present.insert(inst->loaded_id);
        for (int id : wanted)
            if (!present.count(id))
                missing.insert(id);
        if (!missing.empty()) {
            r.code    = CLI_INVALID_PARAMS;
            r.message = "--simplify-objects: no object has id " + join_ints(missing) + "; the file's object ids are " +
                        join_ints(present);
            return r;
        }
    }

    const DetailLevel* level = s.level.empty() ? nullptr : find_level(s.level);
    const char* mode = level ? "level" : (s.ratio >= 0. ? "ratio" : "triangles");

    for (Slic3r::ModelObject* obj : model.objects) {
        if (!wanted.empty()) {
            const bool selected = std::any_of(obj->instances.begin(), obj->instances.end(),
                [&](const Slic3r::ModelInstance* inst) { return wanted.count(inst->loaded_id) != 0; });
            if (!selected)
                continue;
        }
        bool changed = false;
        for (Slic3r::ModelVolume* vol : obj->volumes) {
            // The dialog works on a model part; modifiers, supports and
            // negative volumes are not offered (on_is_activable, 709-713).
            if (!vol->is_model_part())
                continue;
            const size_t before = vol->mesh().its.indices.size();
            // The dialog's worker (GLGizmoSimplify.cpp 562-565 at 56a57d0): the
            // count mode sets only the wanted count, the error mode only the
            // maximal error; the other bound stays open.
            uint32_t count     = 0;
            float    max_error = std::numeric_limits<float>::max();
            nlohmann::json event = {{"event", "mesh_simplified"}, {"object", obj->name}, {"volume", vol->name},
                                    {"mode", mode}, {"triangles_before", before}};
            if (level) {
                max_error = level->max_error;
                event["level"] = level->name;
            } else if (s.ratio >= 0.) {
                count = wanted_count_by_ratio(before, s.ratio);
                event["ratio"] = s.ratio;
                event["target_triangles"] = count;
            } else {
                count = static_cast<uint32_t>(std::min<long long>(s.triangles, std::numeric_limits<uint32_t>::max()));
                event["target_triangles"] = count;
            }
            if (!level && count >= before) {
                // The collapse returns at once (QuadricEdgeCollapse.cpp 173).
                event["tag"] = "SimplifySkipped";
                event["triangles_after"] = before;
                event["message"] = "'" + vol->name + "' has " + std::to_string(before) +
                                   " triangles, not more than the " + std::to_string(count) + " asked for; left as it is";
                emit(event);
                continue;
            }

            indexed_triangle_set its = vol->mesh().its;
            Slic3r::its_quadric_edge_collapse(its, count, &max_error);
            const size_t after = its.indices.size();
            if (after == before) {
                // Nothing collapsed within the error bound: the part stays as
                // it is, paint and all (the dialog would swap in an identical
                // mesh; headless, an unchanged part is left untouched).
                event["tag"] = "SimplifySkipped";
                event["triangles_after"] = before;
                event["message"] = "'" + vol->name + "' keeps its " + std::to_string(before) +
                                   " triangles: nothing to collapse at this detail level; left as it is";
                emit(event);
                continue;
            }

            // Apply as the dialog does (GLGizmoSimplify.cpp 609-700 at 56a57d0;
            // OrcaSlicer 538-556 at 8500fcd).
            const bool keep = s.keep_paint && has_any_painting(*vol);
            if (keep) {
#if defined(ENGINE_SNAPMAKER)
                // Refused by simplify_flags_refusal before any mesh is read.
                (void)vol;
                r.code    = CLI_UNSUPPORTED_OPERATION;
                r.message = "--simplify-keep-paint is not available in slicer_cli-snapmaker";
                return r;
#elif defined(ENGINE_ORCA)
                // Orca's Simplify with keep_painting on: save, swap, restore
                // (GLGizmoSimplify.cpp 538-551 at 8500fcd).
                const auto saved = vol->save_painting();
                vol->set_mesh(std::move(its));
                vol->restore_painting(saved);
#else
                // Bambu's Simplify with "preserve painting" on: re-project the
                // paint onto the new mesh, then commit both (GLGizmoSimplify.cpp
                // 609-680 at 56a57d0).
                Slic3r::TriangleMesh new_mesh(std::move(its));
                Slic3r::PaintKeepPrepared prepared;
                if (!vol->reproject_paint_keep(new_mesh, prepared)) {
                    r.code    = CLI_UNSUPPORTED_OPERATION;
                    r.message = "--simplify-keep-paint: the paint on '" + vol->name +
                                "' could not be carried onto the simplified mesh; drop the flag to simplify without it";
                    return r;
                }
                prepared.mesh = std::move(new_mesh);
                vol->commit_mesh_keep_paint(std::move(prepared));
#endif
            } else {
                // Paint made for the old triangles means nothing on the new ones
                // (GLGizmoSimplify.cpp 688-692 at 56a57d0; Plater.cpp 17145-17150
                // at 8500fcd).
                vol->supported_facets.reset();
                vol->seam_facets.reset();
                vol->mmu_segmentation_facets.reset();
                vol->fuzzy_skin_facets.reset();
                vol->set_mesh(std::move(its));
            }
            vol->calculate_convex_hull();
            vol->invalidate_convex_hull_2d();
            vol->set_new_unique_id();
            changed = true;

            event["tag"] = "MeshSimplified";
            event["triangles_after"] = after;
            event["paint_kept"] = keep;
            event["message"] = "Simplified '" + vol->name + "' from " + std::to_string(before) + " to " +
                               std::to_string(after) + " triangles";
            emit(event);
        }
        if (changed) {
            obj->invalidate_bounding_box();
            obj->ensure_on_bed();
        }
    }
    return r;
}

void warn_large_meshes(const Slic3r::Model& model) {
    // needs_simplify_suggestion (GLGizmoSimplify.cpp 97-101 at 56a57d0): a
    // single-part object of a million triangles or more.
    const size_t triangles_to_suggest_simplify = 1000000;
    for (const Slic3r::ModelObject* obj : model.objects) {
        if (obj->volumes.size() != 1)
            continue;
        const size_t triangles = obj->volumes.front()->mesh().its.indices.size();
        if (triangles < triangles_to_suggest_simplify)
            continue;
        emit({{"event", "warning"}, {"tag", "LargeMesh"}, {"object", obj->name}, {"triangle_count", triangles},
              {"message", "Processing model '" + obj->name + "' with more than 1M triangles could be slow. It is "
                          "highly recommended to simplify the model: pass --simplify medium, or --simplify-ratio or "
                          "--simplify-triangles for a chosen size"}});
    }
}

} // namespace slicer_cli
