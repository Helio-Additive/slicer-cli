// Implementation of the C ABI shim. See clipper_z_shim.h.
//
// IMPORTANT include ordering: clipper_z.hpp must be included BEFORE clipper.hpp
// (it #errors otherwise). clipper_z.hpp #defines CLIPPERLIB_USE_XYZ, includes
// clipper.hpp into namespace ClipperLib_Z, then #undefs clipper_hpp so a second
// include of clipper.hpp below pulls in the non-XYZ namespace ClipperLib.
#include "clipper_z.hpp"   // -> namespace ClipperLib_Z (XYZ / 3D IntPoint)
#include "clipper.hpp"     // -> namespace ClipperLib   (2D IntPoint)

#include "clipper_z_shim.h"

#include <cmath>
#include <cstdlib>
#include <limits>
#include <vector>

// The vendored clipper is wrapped in the `ClipperZSys` outer namespace
// (CLIPPERLIB_NAMESPACE_PREFIX, set in build.rs) so its int32 ClipperLib symbols
// do not collide at link time with geo-clipper's int64 `clipper-sys` ClipperLib
// (an ODR violation that segfaulted the bridges wave_seeds path). The rest of
// this shim keeps using the unprefixed `ClipperLib` / `ClipperLib_Z` spellings
// via these aliases.
#ifdef CLIPPERLIB_NAMESPACE_PREFIX
namespace ClipperLib = CLIPPERLIB_NAMESPACE_PREFIX::ClipperLib;
namespace ClipperLib_Z = CLIPPERLIB_NAMESPACE_PREFIX::ClipperLib_Z;
#endif

// ---------------------------------------------------------------------------
// M1 smoke functions
// ---------------------------------------------------------------------------

extern "C" const char *cz_version(void) {
    return CLIPPER_VERSION;
}

extern "C" int32_t cz_union_point_count(const int32_t *xy, int32_t n) {
    using namespace ClipperLib;
    Path subject;
    subject.reserve(n);
    for (int32_t i = 0; i < n; ++i)
        subject.emplace_back(xy[2 * i], xy[2 * i + 1]);

    Clipper clipper;
    clipper.AddPath(subject, ptSubject, true);
    Paths solution;
    clipper.Execute(ctUnion, solution, pftNonZero, pftNonZero);

    int32_t count = 0;
    for (const Path &p : solution)
        count += static_cast<int32_t>(p.size());
    return count;
}

// ---------------------------------------------------------------------------
// M2: clip_extrusion — faithful replica of OverhangDetector.cpp:18-108.
//
// The ZFillFunction lambda is copied verbatim from BambuStudio
// (OverhangDetector.cpp:21-55), with the libslic3r-specific post-pass for
// zero-Z output vertices reimplemented locally (we have no Slic3r::Point /
// projection_onto here, so we inline an equivalent projection in int math).
// ---------------------------------------------------------------------------

namespace {

using ClipperLib_Z::IntPoint;
using ClipperLib_Z::Path;
using ClipperLib_Z::Paths;

// coord_t in BambuStudio is int32_t when CLIPPERLIB_INT32 is set (it is here),
// matching ClipperLib_Z::cInt. The lambda casts widths to coord_t, so keep int32.
typedef int32_t coord_t;

} // namespace

extern "C" CzZPaths cz_clip_extrusion(const int32_t *subject_xyz, int32_t subject_n,
                                      const int32_t *clip_xyz, const int32_t *clip_lens,
                                      int32_t clip_num, int32_t clip_type) {
    // Build the open subject ZPath.
    Path subject;
    subject.reserve(subject_n);
    for (int32_t i = 0; i < subject_n; ++i)
        subject.emplace_back(subject_xyz[3 * i], subject_xyz[3 * i + 1], subject_xyz[3 * i + 2]);

    // Build the closed clip ZPaths.
    Paths clip;
    clip.reserve(clip_num);
    {
        const int32_t *cursor = clip_xyz;
        for (int32_t c = 0; c < clip_num; ++c) {
            int32_t len = clip_lens[c];
            Path path;
            path.reserve(len);
            for (int32_t i = 0; i < len; ++i)
                path.emplace_back(cursor[3 * i], cursor[3 * i + 1], cursor[3 * i + 2]);
            cursor += 3 * len;
            clip.emplace_back(std::move(path));
        }
    }

    ClipperLib_Z::Clipper clipper;
    // ---- ZFillFunction lambda — verbatim from OverhangDetector.cpp:21-55 ----
    clipper.ZFillFunction([](const IntPoint &e1bot, const IntPoint &e1top, const IntPoint &e2bot,
                             const IntPoint &e2top, IntPoint &pt) {
        // Both ends of each edge belong to the same source: subject or clip.
        // (asserts from the original are dropped in release; behavior identical.)

        // Start & end points of the clipped polyline (extrusion path).
        IntPoint start = e1bot;
        IntPoint end = e1top;
        if (start.z() <= 0 && end.z() <= 0) {
            start = e2bot;
            end = e2top;
        }

        if (start.z() <= 0 && end.z() <= 0) {
            // Self intersection on the source contour.
            pt.z() = 0;
        } else {
            // Interpolate extrusion line width.
            double length_sqr = (end - start).cast<double>().squaredNorm();
            double dist_sqr = (pt - start).cast<double>().squaredNorm();
            double t = std::sqrt(dist_sqr / length_sqr);
            pt.z() = start.z() + coord_t((end.z() - start.z()) * t);
        }
    });

    clipper.AddPath(subject, ClipperLib_Z::ptSubject, false);
    clipper.AddPaths(clip, ClipperLib_Z::ptClip, true);

    ClipperLib_Z::PolyTree clipped_polytree;
    Paths clipped_paths;
    clipper.Execute(static_cast<ClipperLib_Z::ClipType>(clip_type), clipped_polytree,
                    ClipperLib_Z::pftNonZero, ClipperLib_Z::pftNonZero);
    ClipperLib_Z::PolyTreeToPaths(clipped_polytree, clipped_paths);

    // Clipped path could contain vertices from the clip with Z == 0; assign a
    // value from the subject by projecting onto the nearest subject segment.
    // (OverhangDetector.cpp:68-98, reimplemented with local int/double math.)
    for (Path &path : clipped_paths) {
        for (IntPoint &c_pt : path) {
            if (c_pt.z() != 0)
                continue;
            if (subject.size() <= 2)
                continue;

            const double px = (double) c_pt.x();
            const double py = (double) c_pt.y();
            double dist_sqr_min = std::numeric_limits<double>::max();
            size_t it_min = 0;
            double proj_min_x = 0.0, proj_min_y = 0.0;

            double prev_x = (double) subject.front().x();
            double prev_y = (double) subject.front().y();
            for (size_t i = 1; i < subject.size(); ++i) {
                double cx = (double) subject[i].x();
                double cy = (double) subject[i].y();
                // Projection of pt onto segment (prev -> curr) as an infinite line
                // (matches Slic3r::Point::projection_onto(Line) which clamps to the
                // infinite line, not the segment).
                double dx = cx - prev_x;
                double dy = cy - prev_y;
                double seg_len_sqr = dx * dx + dy * dy;
                double proj_x, proj_y;
                if (seg_len_sqr <= 0.0) {
                    proj_x = prev_x;
                    proj_y = prev_y;
                } else {
                    double tparam = ((px - prev_x) * dx + (py - prev_y) * dy) / seg_len_sqr;
                    proj_x = prev_x + tparam * dx;
                    proj_y = prev_y + tparam * dy;
                }
                double ddx = proj_x - px;
                double ddy = proj_y - py;
                double dist_sqr = ddx * ddx + ddy * ddy;
                if (dist_sqr < dist_sqr_min) {
                    dist_sqr_min = dist_sqr;
                    proj_min_x = proj_x;
                    proj_min_y = proj_y;
                    it_min = i - 1;
                }
                prev_x = cx;
                prev_y = cy;
            }

            const double pa_x = (double) subject[it_min].x();
            const double pa_y = (double) subject[it_min].y();
            const double pb_x = (double) subject[it_min + 1].x();
            const double pb_y = (double) subject[it_min + 1].y();
            const double line_len = std::sqrt((pb_x - pa_x) * (pb_x - pa_x) + (pb_y - pa_y) * (pb_y - pa_y));
            const double dist = std::sqrt((proj_min_x - pa_x) * (proj_min_x - pa_x) +
                                          (proj_min_y - pa_y) * (proj_min_y - pa_y));
            double za = (double) subject[it_min].z();
            double zb = (double) subject[it_min + 1].z();
            if (line_len > 0.0)
                c_pt.z() = (coord_t)(za + (dist / line_len) * (zb - za));
            else
                c_pt.z() = (coord_t) za;
        }
    }

    // Marshal into the flat output struct.
    CzZPaths out;
    out.num_paths = (int32_t) clipped_paths.size();
    int32_t total = 0;
    for (const Path &p : clipped_paths)
        total += (int32_t) p.size();
    out.total_points = total;

    if (out.num_paths > 0) {
        out.path_lens = (int32_t *) std::malloc(sizeof(int32_t) * out.num_paths);
        for (int32_t i = 0; i < out.num_paths; ++i)
            out.path_lens[i] = (int32_t) clipped_paths[i].size();
    } else {
        out.path_lens = nullptr;
    }

    if (total > 0) {
        out.coords = (int32_t *) std::malloc(sizeof(int32_t) * 3 * total);
        int32_t k = 0;
        for (const Path &p : clipped_paths) {
            for (const IntPoint &ip : p) {
                out.coords[3 * k + 0] = (int32_t) ip.x();
                out.coords[3 * k + 1] = (int32_t) ip.y();
                out.coords[3 * k + 2] = (int32_t) ip.z();
                ++k;
            }
        }
    } else {
        out.coords = nullptr;
    }

    return out;
}

// ---------------------------------------------------------------------------
// cz_offset_expolygon — faithful replica of ClipperUtils.cpp
// `offset_expolygon_inner` (ClipperUtils.cpp:437-506), the vertex-exact
// per-ExPolygon offset used for the perimeter inner-wall density fix.
// ---------------------------------------------------------------------------

namespace {

// Marshal a vector of ClipperLib (non-Z) Paths into a freshly-malloc'd CzZPaths
// (z always 0). Shared by cz_offset_expolygon.
CzZPaths marshal_paths(const ClipperLib::Paths &paths) {
    CzZPaths out;
    out.num_paths = (int32_t) paths.size();
    int32_t total = 0;
    for (const ClipperLib::Path &p : paths)
        total += (int32_t) p.size();
    out.total_points = total;

    if (out.num_paths > 0) {
        out.path_lens = (int32_t *) std::malloc(sizeof(int32_t) * out.num_paths);
        for (int32_t i = 0; i < out.num_paths; ++i)
            out.path_lens[i] = (int32_t) paths[i].size();
    } else {
        out.path_lens = nullptr;
    }

    if (total > 0) {
        out.coords = (int32_t *) std::malloc(sizeof(int32_t) * 3 * total);
        int32_t k = 0;
        for (const ClipperLib::Path &p : paths)
            for (const ClipperLib::IntPoint &ip : p) {
                out.coords[3 * k + 0] = (int32_t) ip.x();
                out.coords[3 * k + 1] = (int32_t) ip.y();
                out.coords[3 * k + 2] = 0;
                ++k;
            }
    } else {
        out.coords = nullptr;
    }
    return out;
}

} // namespace

extern "C" CzZPaths cz_offset_expolygon(const int32_t *contour_xy, int32_t contour_n,
                                        const int32_t *holes_xy, const int32_t *hole_lens,
                                        int32_t hole_num, double delta, int32_t join_type,
                                        double miter_limit) {
    ClipperLib::JoinType jt = ClipperLib::jtMiter;
    if (join_type == 1) jt = ClipperLib::jtRound;
    else if (join_type == 2) jt = ClipperLib::jtSquare;

    // ClipperOffsetShortestEdgeFactor = 0.005 (ClipperUtils.cpp).
    const double shortest_edge = std::fabs(delta * 0.005);

    // 1) Offset the outer contour (offset_expolygon_inner step 1).
    ClipperLib::Path contour_path;
    contour_path.reserve(contour_n);
    for (int32_t i = 0; i < contour_n; ++i)
        contour_path.emplace_back(contour_xy[2 * i], contour_xy[2 * i + 1]);

    ClipperLib::Paths contours;
    {
        ClipperLib::ClipperOffset co;
        if (jt == ClipperLib::jtRound) co.ArcTolerance = miter_limit;
        else                            co.MiterLimit = miter_limit;
        co.ShortestEdgeLength = shortest_edge;
        co.AddPath(contour_path, jt, ClipperLib::etClosedPolygon);
        co.Execute(contours, delta);
    }
    if (contours.empty())
        return marshal_paths(ClipperLib::Paths{});

    if (hole_num <= 0) {
        // No holes: done.
        return marshal_paths(contours);
    }

    // 2) Offset the holes one by one (signum reversed: Execute on -delta).
    ClipperLib::Paths holes;
    {
        const int32_t *cursor = holes_xy;
        for (int32_t h = 0; h < hole_num; ++h) {
            int32_t len = hole_lens[h];
            ClipperLib::Path hole_path;
            hole_path.reserve(len);
            for (int32_t i = 0; i < len; ++i)
                hole_path.emplace_back(cursor[2 * i], cursor[2 * i + 1]);
            cursor += 2 * len;

            ClipperLib::ClipperOffset co;
            if (jt == ClipperLib::jtRound) co.ArcTolerance = miter_limit;
            else                            co.MiterLimit = miter_limit;
            co.ShortestEdgeLength = shortest_edge;
            co.AddPath(hole_path, jt, ClipperLib::etClosedPolygon);
            ClipperLib::Paths out2;
            co.Execute(out2, -delta);
            for (ClipperLib::Path &p : out2)
                holes.push_back(std::move(p));
        }
    }

    // 3) Combine contour + holes (offset_expolygon_inner step 3).
    ClipperLib::Paths result;
    if (holes.empty()) {
        result = std::move(contours);
    } else if (delta < 0) {
        // Negative offset: subtract offsetted holes from offsetted contours.
        ClipperLib::Clipper clipper;
        clipper.AddPaths(contours, ClipperLib::ptSubject, true);
        clipper.AddPaths(holes, ClipperLib::ptClip, true);
        ClipperLib::Paths diff;
        clipper.Execute(ClipperLib::ctDifference, diff, ClipperLib::pftNonZero,
                        ClipperLib::pftNonZero);
        result = std::move(diff); // may be empty -> caller treats as collapsed
    } else {
        // Positive offset: append reversed holes.
        result.reserve(contours.size() + holes.size());
        for (ClipperLib::Path &p : contours)
            result.push_back(std::move(p));
        for (ClipperLib::Path &p : holes) {
            std::reverse(p.begin(), p.end());
            result.push_back(std::move(p));
        }
    }

    return marshal_paths(result);
}

// ---------------------------------------------------------------------------
// cz_difference_closed — faithful replica of ClipperUtils.cpp
// `clipper_do<ClipperLib::Paths>(ctDifference, subject, clip, pftNonZero)`
// (ClipperUtils.cpp:309-322), the closed-path boolean difference underpinning
// `diff` / `diff_ex` (ApplySafetyOffset::No path). See the header for the
// faithfulness argument (the Rust caller re-unions the output to a PolyTree,
// matching clipper_do_polytree -> PolyTreeToExPolygons).
// ---------------------------------------------------------------------------

namespace {

// Read `num` closed paths from a flat (x,y) int32 buffer + per-path lengths into
// a vector of ClipperLib (non-Z) Paths, in their natural orientation (matching
// ClipperUtils::ExPolygonsProvider: contour + holes emitted verbatim).
ClipperLib::Paths read_closed_paths(const int32_t *xy, const int32_t *lens, int32_t num) {
    ClipperLib::Paths paths;
    if (num <= 0 || xy == nullptr || lens == nullptr)
        return paths;
    paths.reserve(num);
    const int32_t *cursor = xy;
    for (int32_t p = 0; p < num; ++p) {
        int32_t len = lens[p];
        ClipperLib::Path path;
        path.reserve(len);
        for (int32_t i = 0; i < len; ++i)
            path.emplace_back(cursor[2 * i], cursor[2 * i + 1]);
        cursor += 2 * len;
        paths.emplace_back(std::move(path));
    }
    return paths;
}

} // namespace

// ---------------------------------------------------------------------------
// cz_propagate_wave — faithful replica of RegionExpansion.cpp
// propagate_wave_from_boundary (+ wavefront_initial/step/clip, :391-462): the
// ClipperLib (non-Z) wavefront propagation for a single (boundary, src) seed
// group. The boundary bbox trim (clip_clipper_polygons_with_subject_bbox) native
// applies is a result-neutral speed optimization — the wavefront is bounded by
// max_inflation inside the inflated seed bbox, so `wf ∩ (boundary ∩ bbox)` ≡
// `wf ∩ boundary` — and is omitted; the full boundary is passed as the clip.
// ---------------------------------------------------------------------------
namespace {

// RegionExpansion.cpp:391 wavefront_initial — inflate each open/closed seed
// polyline by `offset` (jtRound; etOpenRound for open, etClosedLine for closed).
ClipperLib::Paths cz_wavefront_initial(ClipperLib::ClipperOffset &co,
                                       const ClipperLib::Paths &polylines, double offset) {
    ClipperLib::Paths out, out_this;
    out.reserve(polylines.size());
    for (const ClipperLib::Path &path : polylines) {
        co.Clear();
        co.AddPath(path, ClipperLib::jtRound,
                   path.front() == path.back() ? ClipperLib::etClosedLine : ClipperLib::etOpenRound);
        co.Execute(out_this, offset);
        out.insert(out.end(), std::make_move_iterator(out_this.begin()),
                   std::make_move_iterator(out_this.end()));
    }
    return out;
}

// RegionExpansion.cpp:409 wavefront_step — inflate each closed polygon by
// `offset` (jtRound/etClosedPolygon); Execute reorients to CCW so the offset
// sign is reversed for CW input and the result reversed back.
ClipperLib::Paths cz_wavefront_step(ClipperLib::ClipperOffset &co,
                                    const ClipperLib::Paths &polygons, double offset) {
    ClipperLib::Paths out, out_this;
    out.reserve(polygons.size());
    for (const ClipperLib::Path &polygon : polygons) {
        co.Clear();
        co.AddPath(polygon, ClipperLib::jtRound, ClipperLib::etClosedPolygon);
        bool ccw = ClipperLib::Orientation(polygon);
        co.Execute(out_this, ccw ? offset : -offset);
        if (!ccw)
            for (ClipperLib::Path &p : out_this) std::reverse(p.begin(), p.end());
        out.insert(out.end(), std::make_move_iterator(out_this.begin()),
                   std::make_move_iterator(out_this.end()));
    }
    return out;
}

// RegionExpansion.cpp:432 wavefront_clip — intersect the wavefront with the
// boundary (ctIntersection, pftPositive/pftPositive).
ClipperLib::Paths cz_wavefront_clip(const ClipperLib::Paths &wavefront,
                                    const ClipperLib::Paths &clipping) {
    ClipperLib::Clipper clipper;
    clipper.AddPaths(wavefront, ClipperLib::ptSubject, true);
    clipper.AddPaths(clipping, ClipperLib::ptClip, true);
    ClipperLib::Paths out;
    clipper.Execute(ClipperLib::ctIntersection, out, ClipperLib::pftPositive, ClipperLib::pftPositive);
    return out;
}

} // namespace

// seed_xy/seed_lens/seed_num: the open seed polylines for ONE (boundary,src)
// group. bnd_xy/bnd_lens/bnd_num: the boundary ExPolygon as closed paths
// (contour CCW + holes CW; pftPositive yields the interior). Returns the
// expanded closed polygons (z=0). Free via cz_free_zpaths.
extern "C" CzZPaths cz_propagate_wave(
    const int32_t *seed_xy, const int32_t *seed_lens, int32_t seed_num,
    const int32_t *bnd_xy, const int32_t *bnd_lens, int32_t bnd_num,
    double initial_step, double other_step, int32_t num_other_steps,
    double arc_tolerance, double shortest_edge_length) {
    ClipperLib::Paths seed = read_closed_paths(seed_xy, seed_lens, seed_num);
    ClipperLib::Paths clipping = read_closed_paths(bnd_xy, bnd_lens, bnd_num);
    if (seed.empty())
        return marshal_paths(ClipperLib::Paths{});

    ClipperLib::ClipperOffset co;
    co.ArcTolerance = arc_tolerance;
    co.ShortestEdgeLength = shortest_edge_length;

    ClipperLib::Paths polygons =
        cz_wavefront_clip(cz_wavefront_initial(co, seed, initial_step), clipping);
    for (int32_t i = 0; i < num_other_steps; ++i)
        polygons = cz_wavefront_clip(cz_wavefront_step(co, polygons, other_step), clipping);

    return marshal_paths(polygons);
}

// Faithful `Slic3r::offset(const Polyline&, delta)` (ClipperUtils.cpp:418-419):
// raw_offset_polyline (per-path ClipperOffset, default jtSquare/etOpenButt,
// MiterLimit=DefaultLineMiterLimit, ShortestEdgeLength=|delta|*0.005) followed by
// clipper_union<Paths> (NonZero). `xy`/`n` = one OPEN polyline (flat i32 pairs);
// delta in scaled units. Returns the covered polygons (z=0). Free via cz_free_zpaths.
extern "C" CzZPaths cz_offset_polyline(const int32_t *xy, int32_t n, double delta,
                                       double miter_limit) {
    ClipperLib::Path path;
    path.reserve(n);
    for (int32_t i = 0; i < n; ++i)
        path.emplace_back(xy[2 * i], xy[2 * i + 1]);
    if (path.size() < 2)
        return marshal_paths(ClipperLib::Paths{});

    ClipperLib::ClipperOffset co;
    co.MiterLimit = miter_limit;
    co.ShortestEdgeLength = std::abs(delta * 0.005);
    co.AddPath(path, ClipperLib::jtSquare, ClipperLib::etOpenButt);
    ClipperLib::Paths raw;
    co.Execute(raw, delta);
    if (raw.empty())
        return marshal_paths(ClipperLib::Paths{});

    // clipper_union<Paths>(NonZero) — ClipperUtils.cpp offset(Polyline) tail.
    ClipperLib::Clipper c;
    c.AddPaths(raw, ClipperLib::ptSubject, true);
    ClipperLib::Paths out;
    c.Execute(ClipperLib::ctUnion, out, ClipperLib::pftNonZero, ClipperLib::pftNonZero);
    return marshal_paths(out);
}

extern "C" CzZPaths cz_difference_closed(const int32_t *subject_xy, const int32_t *subject_lens,
                                         int32_t subject_num, const int32_t *clip_xy,
                                         const int32_t *clip_lens, int32_t clip_num) {
    ClipperLib::Paths subject = read_closed_paths(subject_xy, subject_lens, subject_num);
    ClipperLib::Paths clip = read_closed_paths(clip_xy, clip_lens, clip_num);

    // clipper_do<ClipperLib::Paths>(ctDifference, subject, clip, pftNonZero)
    // (ClipperUtils.cpp:309-322). Both subject and clip paths are closed (true).
    ClipperLib::Clipper clipper;
    clipper.AddPaths(subject, ClipperLib::ptSubject, true);
    clipper.AddPaths(clip, ClipperLib::ptClip, true);
    ClipperLib::Paths solution;
    clipper.Execute(ClipperLib::ctDifference, solution, ClipperLib::pftNonZero,
                    ClipperLib::pftNonZero);

    return marshal_paths(solution);
}

extern "C" CzZPaths cz_intersection_closed(const int32_t *subject_xy, const int32_t *subject_lens,
                                           int32_t subject_num, const int32_t *clip_xy,
                                           const int32_t *clip_lens, int32_t clip_num) {
    ClipperLib::Paths subject = read_closed_paths(subject_xy, subject_lens, subject_num);
    ClipperLib::Paths clip = read_closed_paths(clip_xy, clip_lens, clip_num);

    // clipper_do<ClipperLib::Paths>(ctIntersection, subject, clip, pftNonZero)
    // (ClipperUtils.cpp:802 intersection_ex, ApplySafetyOffset::No). Both closed.
    ClipperLib::Clipper clipper;
    clipper.AddPaths(subject, ClipperLib::ptSubject, true);
    clipper.AddPaths(clip, ClipperLib::ptClip, true);
    ClipperLib::Paths solution;
    clipper.Execute(ClipperLib::ctIntersection, solution, ClipperLib::pftNonZero,
                    ClipperLib::pftNonZero);

    return marshal_paths(solution);
}

// ---------------------------------------------------------------------------
// cz_difference_closed_safety — faithful replica of
// `clipper_do<Paths>(ctDifference, subject, safety_offset(clip), pftNonZero)`
// (ClipperUtils.cpp:334), i.e. `diff_ex(subject, clip, ApplySafetyOffset::Yes)`.
// safety_offset(clip) = raw_offset(clip, ClipperSafetyOffset=10, jtMiter,
// DefaultMiterLimit=3) applied path-by-path with orientation-aware signum
// (ClipperUtils.cpp:273-307). The Rust caller re-unions the raw output to a
// PolyTree (matching diff_ex's PolyTreeToExPolygons), exactly like
// cz_difference_closed.
// ---------------------------------------------------------------------------

namespace {

// raw_offset(paths, ClipperSafetyOffset=10, jtMiter, 3.0) — ClipperUtils.cpp:273-300.
ClipperLib::Paths cz_safety_offset(const ClipperLib::Paths &paths) {
    const float offset = 10.0f;      // ClipperSafetyOffset (ClipperUtils.hpp:29)
    const double miter_limit = 3.0;  // DefaultMiterLimit
    ClipperLib::ClipperOffset co;
    co.MiterLimit = miter_limit;
    co.ShortestEdgeLength = std::abs(offset * 0.005); // ClipperOffsetShortestEdgeFactor
    ClipperLib::Paths out;
    out.reserve(paths.size());
    ClipperLib::Paths out_this;
    for (const ClipperLib::Path &path : paths) {
        co.Clear();
        // Execute reorients so the outer contour is CCW; signum reversed for CW input.
        co.AddPath(path, ClipperLib::jtMiter, ClipperLib::etClosedPolygon);
        bool ccw = ClipperLib::Orientation(path);
        co.Execute(out_this, ccw ? offset : -offset);
        if (!ccw)
            for (ClipperLib::Path &p : out_this)
                std::reverse(p.begin(), p.end());
        for (ClipperLib::Path &p : out_this)
            out.push_back(std::move(p));
        out_this.clear();
    }
    return out;
}

} // namespace

extern "C" CzZPaths cz_difference_closed_safety(const int32_t *subject_xy, const int32_t *subject_lens,
                                                int32_t subject_num, const int32_t *clip_xy,
                                                const int32_t *clip_lens, int32_t clip_num) {
    ClipperLib::Paths subject = read_closed_paths(subject_xy, subject_lens, subject_num);
    ClipperLib::Paths clip = read_closed_paths(clip_xy, clip_lens, clip_num);
    ClipperLib::Paths clip_safe = cz_safety_offset(clip);

    ClipperLib::Clipper clipper;
    clipper.AddPaths(subject, ClipperLib::ptSubject, true);
    clipper.AddPaths(clip_safe, ClipperLib::ptClip, true);
    ClipperLib::Paths solution;
    clipper.Execute(ClipperLib::ctDifference, solution, ClipperLib::pftNonZero,
                    ClipperLib::pftNonZero);

    return marshal_paths(solution);
}

// cz_intersection_closed_safety — `clipper_do<Paths>(ctIntersection, subject,
// safety_offset(clip), pftNonZero)` (ClipperUtils.cpp:334 with Yes), i.e.
// `intersection(subject, clip, ApplySafetyOffset::Yes)`.
extern "C" CzZPaths cz_intersection_closed_safety(const int32_t *subject_xy, const int32_t *subject_lens,
                                                  int32_t subject_num, const int32_t *clip_xy,
                                                  const int32_t *clip_lens, int32_t clip_num) {
    ClipperLib::Paths subject = read_closed_paths(subject_xy, subject_lens, subject_num);
    ClipperLib::Paths clip = read_closed_paths(clip_xy, clip_lens, clip_num);
    ClipperLib::Paths clip_safe = cz_safety_offset(clip);

    ClipperLib::Clipper clipper;
    clipper.AddPaths(subject, ClipperLib::ptSubject, true);
    clipper.AddPaths(clip_safe, ClipperLib::ptClip, true);
    ClipperLib::Paths solution;
    clipper.Execute(ClipperLib::ctIntersection, solution, ClipperLib::pftNonZero,
                    ClipperLib::pftNonZero);

    return marshal_paths(solution);
}

// ---------------------------------------------------------------------------
// cz_union_ex — faithful replica of ClipperUtils.cpp `union_ex(const Polygons&,
// PolyFillType)` (ClipperUtils.cpp:813-814) = PolyTreeToExPolygons(
// clipper_do_polytree(ctUnion, ..., fill_type)). The slice-stage F1 union behind
// make_expolygons (TriangleMeshSlicer.cpp:1819-1823). Output is grouped into
// ExPolygons via the EXACT PolyTreeToExPolygons recursion; each output path's z
// encodes contour(0)/hole(1).
// ---------------------------------------------------------------------------

namespace {

// Faithful replica of ClipperUtils.cpp PolyTreeToExPolygons recursion
// (ClipperUtils.cpp:178-189). Appends, for each contour, the contour path (z=0)
// immediately followed by its hole paths (z=1); contours nested inside holes are
// appended AFTER (recursively), matching the C++ traversal order exactly.
void polytree_to_grouped(ClipperLib::PolyNode &polynode, ClipperLib::Paths &out_paths,
                         std::vector<int32_t> &out_is_hole) {
    // contour
    {
        ClipperLib::Path contour = polynode.Contour;
        out_paths.push_back(std::move(contour));
        out_is_hole.push_back(0);
    }
    // Emit ALL of this contour's holes FIRST, then recurse the nested outer
    // contours. The caller's decode attaches each hole to the most-recent contour
    // ExPolygon; interleaving a nested contour between sibling holes (the previous
    // behavior) mis-attached the holes that followed it to the nested contour
    // instead of this one (rust 7+1 vs C++ 8+0 at L0). Emitting every hole before
    // any grandchild contour makes the attach-to-last decode match C++
    // PolyTreeToExPolygonsRecursive's direct holes[i] assignment (ClipperUtils.cpp:178-189).
    for (int i = 0; i < polynode.ChildCount(); ++i) {
        ClipperLib::Path hole = polynode.Childs[i]->Contour;
        out_paths.push_back(std::move(hole));
        out_is_hole.push_back(1);
    }
    for (int i = 0; i < polynode.ChildCount(); ++i)
        for (int j = 0; j < polynode.Childs[i]->ChildCount(); ++j)
            polytree_to_grouped(*polynode.Childs[i]->Childs[j], out_paths, out_is_hole);
}

// Marshal grouped (path, is_hole) into CzZPaths, encoding is_hole in each
// point's z (contour z=0, hole z=1).
CzZPaths marshal_grouped(const ClipperLib::Paths &paths, const std::vector<int32_t> &is_hole) {
    CzZPaths out;
    out.num_paths = (int32_t) paths.size();
    int32_t total = 0;
    for (const ClipperLib::Path &p : paths)
        total += (int32_t) p.size();
    out.total_points = total;

    if (out.num_paths > 0) {
        out.path_lens = (int32_t *) std::malloc(sizeof(int32_t) * out.num_paths);
        for (int32_t i = 0; i < out.num_paths; ++i)
            out.path_lens[i] = (int32_t) paths[i].size();
    } else {
        out.path_lens = nullptr;
    }

    if (total > 0) {
        out.coords = (int32_t *) std::malloc(sizeof(int32_t) * 3 * total);
        int32_t k = 0;
        for (size_t pi = 0; pi < paths.size(); ++pi) {
            int32_t z = is_hole[pi];
            for (const ClipperLib::IntPoint &ip : paths[pi]) {
                out.coords[3 * k + 0] = (int32_t) ip.x();
                out.coords[3 * k + 1] = (int32_t) ip.y();
                out.coords[3 * k + 2] = z;
                ++k;
            }
        }
    } else {
        out.coords = nullptr;
    }
    return out;
}

} // namespace

extern "C" CzZPaths cz_simplify_polygons(const int32_t *xy, const int32_t *lens, int32_t num,
                                         int32_t fill_type) {
    // Faithful ClipperLib::SimplifyPolygons (clipper.hpp:559-566): a ctUnion with
    // StrictlySimple(true) — used by ClipperUtils simplify_polygons (the
    // ExPolygon::simplify_p post-DP step, ClipperUtils.cpp:1026-1040). Returns flat
    // Paths (z=0); the caller re-unions into ExPolygons. KEY: StrictlySimple(true)
    // differs from cz_union_ex's default-false union → different vertex retention.
    ClipperLib::Paths subject = read_closed_paths(xy, lens, num);
    ClipperLib::PolyFillType pft = ClipperLib::pftNonZero;
    switch (fill_type) {
        case 0: pft = ClipperLib::pftEvenOdd; break;
        case 1: pft = ClipperLib::pftNonZero; break;
        case 2: pft = ClipperLib::pftPositive; break;
        case 3: pft = ClipperLib::pftNegative; break;
        default: pft = ClipperLib::pftNonZero; break;
    }
    ClipperLib::Clipper c;
    c.StrictlySimple(true);
    c.AddPaths(subject, ClipperLib::ptSubject, true);
    ClipperLib::Paths out;
    c.Execute(ClipperLib::ctUnion, out, pft, pft);
    return marshal_paths(out);
}

extern "C" CzZPaths cz_union_ex(const int32_t *xy, const int32_t *lens, int32_t num,
                                int32_t fill_type) {
    ClipperLib::Paths subject = read_closed_paths(xy, lens, num);

    ClipperLib::PolyFillType pft = ClipperLib::pftNonZero;
    switch (fill_type) {
        case 0: pft = ClipperLib::pftEvenOdd; break;
        case 1: pft = ClipperLib::pftNonZero; break;
        case 2: pft = ClipperLib::pftPositive; break;
        case 3: pft = ClipperLib::pftNegative; break;
        default: pft = ClipperLib::pftNonZero; break;
    }

    // clipper_do_polytree(ctUnion, subject, Empty, fill_type) — ClipperUtils.cpp:641-654.
    // C++ does this in TWO PASSES (a single union-into-PolyTree is "very expensive
    // with overlapping edges" + gives a DIFFERENT result): (1) clipper_do<Paths>
    // (ctUnion → flat Paths), (2) clipper_union<PolyTree> (a 2nd union of that
    // output to build the PolyTree ordering). A single-pass union-into-PolyTree
    // dropped/added vertices (R88: npts 4457 vs C++ 4505). Replicate both passes.
    ClipperLib::Paths pass1;
    {
        ClipperLib::Clipper c1;
        c1.AddPaths(subject, ClipperLib::ptSubject, true);
        // empty clip (EmptyPathsProvider) — no ptClip paths added.
        c1.Execute(ClipperLib::ctUnion, pass1, pft, pft);
    }
    if (pass1.empty())
        return marshal_grouped(ClipperLib::Paths{}, std::vector<int32_t>{});

    ClipperLib::PolyTree polytree;
    {
        ClipperLib::Clipper c2;
        c2.AddPaths(pass1, ClipperLib::ptSubject, true);
        c2.Execute(ClipperLib::ctUnion, polytree, pft, pft);
    }

    // PolyTreeToExPolygons grouping (ClipperUtils.cpp:203-210).
    ClipperLib::Paths out_paths;
    std::vector<int32_t> out_is_hole;
    for (int i = 0; i < polytree.ChildCount(); ++i)
        polytree_to_grouped(*polytree.Childs[i], out_paths, out_is_hole);

    return marshal_grouped(out_paths, out_is_hole);
}

// ---------------------------------------------------------------------------
// cz_offset2_ex — faithful replica of ClipperUtils.cpp `offset2_ex(ExPolygons,
// delta1, delta2)` (ClipperUtils.cpp:581) =
//   PolyTreeToExPolygons(offset_paths<PolyTree>(expolygons_offset(expolys, delta1), delta2))
// The post-union morphological close in make_expolygons (TriangleMeshSlicer.cpp:1820,
// called with delta1=+scale(closing_radius), delta2=-scale(closing_radius)). Input is
// the cz_union_ex output layout: flat (x,y) i32 pairs, per-path point counts `lens`,
// per-path `is_hole` (0=contour starts a new ExPolygon, 1=hole attaches to current),
// `num` paths. deltas are SCALED (1e5). join_type 0=miter/1=round/2=square,
// miter_limit=3.0 (DefaultMiterLimit). Output uses the same grouped z-encoding.
// Free via cz_free_zpaths.
// Faithful `union_safety_offset_ex(Polygons)` (ClipperUtils.cpp): the SUBJECT is
// safety-offset (raw +10u, no union) before the two-pass NonZero union.
extern "C" CzZPaths cz_union_ex_safety(const int32_t *xy, const int32_t *lens, int32_t num) {
    ClipperLib::Paths subject = cz_safety_offset(read_closed_paths(xy, lens, num));
    if (subject.empty())
        return marshal_grouped(ClipperLib::Paths{}, std::vector<int32_t>{});
    ClipperLib::Paths pass1;
    {
        ClipperLib::Clipper c1;
        c1.AddPaths(subject, ClipperLib::ptSubject, true);
        c1.Execute(ClipperLib::ctUnion, pass1, ClipperLib::pftNonZero, ClipperLib::pftNonZero);
    }
    if (pass1.empty())
        return marshal_grouped(ClipperLib::Paths{}, std::vector<int32_t>{});
    ClipperLib::PolyTree polytree;
    {
        ClipperLib::Clipper c2;
        c2.AddPaths(pass1, ClipperLib::ptSubject, true);
        c2.Execute(ClipperLib::ctUnion, polytree, ClipperLib::pftNonZero, ClipperLib::pftNonZero);
    }
    // PolyTreeToExPolygons grouping (same walk as cz_union_ex).
    ClipperLib::Paths out_paths;
    std::vector<int32_t> out_is_hole;
    for (int i = 0; i < polytree.ChildCount(); ++i)
        polytree_to_grouped(*polytree.Childs[i], out_paths, out_is_hole);
    return marshal_grouped(out_paths, out_is_hole);
}

extern "C" CzZPaths cz_offset2_ex(const int32_t *xy, const int32_t *lens, const int32_t *is_hole,
                                  int32_t num, double delta1, double delta2,
                                  int32_t join_type, double miter_limit) {
    // Fully qualify to the 2D (non-Z) ClipperLib; `Path`/`Paths` are otherwise
    // ambiguous with ClipperLib_Z at this file scope.
    typedef ClipperLib::Path CzPath;
    typedef ClipperLib::Paths CzPaths;
    ClipperLib::JoinType jt = ClipperLib::jtMiter;
    if (join_type == 1) jt = ClipperLib::jtRound;
    else if (join_type == 2) jt = ClipperLib::jtSquare;
    const double sef = 0.005; // ClipperUtils ClipperOffsetShortestEdgeFactor

    // Reconstruct ExPolygons (contour + holes) from grouped input.
    struct ExP { CzPath contour; CzPaths holes; };
    std::vector<ExP> expolys;
    {
        const int32_t *cur = xy;
        for (int32_t p = 0; p < num; ++p) {
            int32_t len = lens[p];
            CzPath path;
            path.reserve(len);
            for (int32_t i = 0; i < len; ++i)
                path.emplace_back(cur[2 * i], cur[2 * i + 1]);
            cur += 2 * len;
            if (is_hole[p] == 0)
                expolys.push_back(ExP{ std::move(path), {} });
            else if (! expolys.empty())
                expolys.back().holes.push_back(std::move(path));
        }
    }

    // offset_expolygon_inner (ClipperUtils.cpp:437-506): append offset paths to `out`.
    auto offset_expoly_inner = [&](const ExP &e, double delta, CzPaths &out) -> int {
        CzPaths contours;
        {
            ClipperLib::ClipperOffset co;
            if (jt == ClipperLib::jtRound) co.ArcTolerance = miter_limit; else co.MiterLimit = miter_limit;
            co.ShortestEdgeLength = std::fabs(delta * sef);
            co.AddPath(e.contour, jt, ClipperLib::etClosedPolygon);
            co.Execute(contours, delta);
        }
        if (contours.empty())
            return 0;
        if (e.holes.empty()) {
            for (CzPath &c : contours) out.push_back(std::move(c));
            return 1;
        }
        CzPaths holes;
        for (const CzPath &h : e.holes) {
            ClipperLib::ClipperOffset co;
            if (jt == ClipperLib::jtRound) co.ArcTolerance = miter_limit; else co.MiterLimit = miter_limit;
            co.ShortestEdgeLength = std::fabs(delta * sef);
            co.AddPath(h, jt, ClipperLib::etClosedPolygon);
            CzPaths o2;
            co.Execute(o2, -delta); // holes: signum reversed
            for (CzPath &p : o2) holes.push_back(std::move(p));
        }
        if (holes.empty()) {
            for (CzPath &c : contours) out.push_back(std::move(c));
        } else if (delta < 0) {
            ClipperLib::Clipper c;
            c.AddPaths(contours, ClipperLib::ptSubject, true);
            c.AddPaths(holes, ClipperLib::ptClip, true);
            CzPaths diff;
            c.Execute(ClipperLib::ctDifference, diff, ClipperLib::pftNonZero, ClipperLib::pftNonZero);
            if (diff.empty()) return 0;
            for (CzPath &p : diff) out.push_back(std::move(p));
        } else {
            for (CzPath &c : contours) out.push_back(std::move(c));
            for (CzPath &h : holes) { std::reverse(h.begin(), h.end()); out.push_back(std::move(h)); }
        }
        return 1;
    };

    // Step A: expolygons_offset(expolys, delta1) — offset each expoly; unite if >1 && delta1>0.
    CzPaths stepA;
    size_t collected = 0;
    for (const ExP &e : expolys)
        collected += offset_expoly_inner(e, delta1, stepA);
    if (collected > 1 && delta1 > 0) {
        ClipperLib::Clipper c;
        c.AddPaths(stepA, ClipperLib::ptSubject, true);
        CzPaths u;
        c.Execute(ClipperLib::ctUnion, u, ClipperLib::pftNonZero, ClipperLib::pftNonZero);
        stepA = std::move(u);
    }

    // Step B: offset_paths<PolyTree>(stepA, delta2) (ClipperUtils.cpp:399-408).
    // raw_offset(stepA, delta2) is PER-PATH, orientation-aware (ClipperUtils.cpp:272-300),
    // then native DISPATCHES ON THE SIGN of delta2:
    //   delta2 > 0 → expand_paths = clipper_union<PolyTree>(raw), pftNonZero, NO frame
    //                (ClipperUtils.cpp:366-372).
    //   delta2 < 0 → shrink_paths = bounding-frame union with pftNegative + ReverseSolution
    //                + RemoveOutermostPolygon (ClipperUtils.cpp:381-397).
    // R103b: the prior implementation hardcoded the shrink_paths branch — correct only for
    // the slice-closing use (delta1=+r, delta2=-r). offset2_ex callers with delta2 > 0 (a
    // SHRINK-then-GROW, e.g. the perimeter inner offset) must use expand_paths. Branch on
    // the sign to match native. (Byte-identical on Benchy — both reconstructions agree on
    // its non-self-intersecting offset paths — but faithful for the general delta2>0 case.)
    ClipperLib::PolyTree polytree;
    if (! stepA.empty()) {
        CzPaths raw;
        raw.reserve(stepA.size());
        {
            ClipperLib::ClipperOffset co;
            if (jt == ClipperLib::jtRound) co.ArcTolerance = miter_limit; else co.MiterLimit = miter_limit;
            co.ShortestEdgeLength = std::fabs(delta2 * sef);
            for (const CzPath &path : stepA) {
                co.Clear();
                co.AddPath(path, jt, ClipperLib::etClosedPolygon);
                bool ccw = ClipperLib::Orientation(path);
                CzPaths out_this;
                co.Execute(out_this, ccw ? delta2 : -delta2);
                if (! ccw)
                    for (CzPath &p : out_this) std::reverse(p.begin(), p.end());
                for (CzPath &p : out_this) raw.push_back(std::move(p));
            }
        }
        if (! raw.empty()) {
            if (delta2 > 0) {
                // expand_paths: plain union of the raw offset paths into a PolyTree.
                ClipperLib::Clipper clipper;
                clipper.AddPaths(raw, ClipperLib::ptSubject, true);
                clipper.Execute(ClipperLib::ctUnion, polytree, ClipperLib::pftNonZero, ClipperLib::pftNonZero);
            } else {
                // shrink_paths: bounding-frame union with the negative fill rule.
                ClipperLib::Clipper clipper;
                clipper.AddPaths(raw, ClipperLib::ptSubject, true);
                ClipperLib::IntRect r = clipper.GetBounds();
                CzPath frame;
                frame.emplace_back(r.left - 10, r.bottom + 10);
                frame.emplace_back(r.right + 10, r.bottom + 10);
                frame.emplace_back(r.right + 10, r.top - 10);
                frame.emplace_back(r.left - 10, r.top - 10);
                clipper.AddPath(frame, ClipperLib::ptSubject, true);
                clipper.ReverseSolution(true);
                clipper.Execute(ClipperLib::ctUnion, polytree, ClipperLib::pftNegative, ClipperLib::pftNegative);
                polytree.RemoveOutermostPolygon();
            }
        }
    }

    ClipperLib::Paths o2_paths;
    std::vector<int32_t> o2_is_hole;
    for (int i = 0; i < polytree.ChildCount(); ++i)
        polytree_to_grouped(*polytree.Childs[i], o2_paths, o2_is_hole);
    return marshal_grouped(o2_paths, o2_is_hole);
}

// ---------------------------------------------------------------------------
// cz_detect_floating — faithful replica of FillFloatingConcentric.cpp
// `detect_floating_line` (the Z-clipper half, FillFloatingConcentric.cpp:431-475).
// ---------------------------------------------------------------------------

namespace {

// Marshal a vector of ClipperLib_Z Paths (open, with z) into a freshly-malloc'd
// CzZPaths preserving z. Mirrors the inline marshalling in cz_clip_extrusion.
CzZPaths marshal_zpaths(const Paths &paths) {
    CzZPaths out;
    out.num_paths = (int32_t) paths.size();
    int32_t total = 0;
    for (const Path &p : paths)
        total += (int32_t) p.size();
    out.total_points = total;

    if (out.num_paths > 0) {
        out.path_lens = (int32_t *) std::malloc(sizeof(int32_t) * out.num_paths);
        for (int32_t i = 0; i < out.num_paths; ++i)
            out.path_lens[i] = (int32_t) paths[i].size();
    } else {
        out.path_lens = nullptr;
    }

    if (total > 0) {
        out.coords = (int32_t *) std::malloc(sizeof(int32_t) * 3 * total);
        int32_t k = 0;
        for (const Path &p : paths)
            for (const IntPoint &ip : p) {
                out.coords[3 * k + 0] = (int32_t) ip.x();
                out.coords[3 * k + 1] = (int32_t) ip.y();
                out.coords[3 * k + 2] = (int32_t) ip.z();
                ++k;
            }
    } else {
        out.coords = nullptr;
    }
    return out;
}

} // namespace

extern "C" CzZPaths cz_detect_floating(const int32_t *subject_xyz, int32_t subject_n,
                                       const int32_t *clip_xyz, const int32_t *clip_lens,
                                       int32_t clip_num, int32_t subject_idx_range,
                                       int32_t *out_num_diff_paths) {
    // FillFloatingConcentric.cpp:412-415 — build the OPEN subject ZPath (one path).
    Paths subject_paths;
    {
        Path s;
        s.reserve(subject_n);
        for (int32_t i = 0; i < subject_n; ++i)
            s.emplace_back(subject_xyz[3 * i], subject_xyz[3 * i + 1], subject_xyz[3 * i + 2]);
        subject_paths.emplace_back(std::move(s));
    }

    // FillFloatingConcentric.cpp:418-426 — build the CLOSED clip ZPaths.
    Paths clip_paths;
    clip_paths.reserve(clip_num);
    {
        const int32_t *cursor = clip_xyz;
        for (int32_t c = 0; c < clip_num; ++c) {
            int32_t len = clip_lens[c];
            Path path;
            path.reserve(len);
            for (int32_t i = 0; i < len; ++i)
                path.emplace_back(cursor[3 * i], cursor[3 * i + 1], cursor[3 * i + 2]);
            cursor += 3 * len;
            clip_paths.emplace_back(std::move(path));
        }
    }

    // FillFloatingConcentric.cpp:407-411 — the hash function (verbatim).
    auto hash_function = [](const int a1, const int b1, const int a2, const int b2) -> int32_t {
        int32_t hash_val = 1000 * (a1 * 13 + b1) + (a2 * 17 + b2) + 1;
        hash_val &= 0x7fffffff;
        return hash_val;
    };

    // FillFloatingConcentric.cpp:431-456 — the ZFillFunction (verbatim semantics;
    // the C++ BOOST_LOG_TRIVIAL(error) diagnostics are dropped — they do not affect
    // the output `d.z()`).
    ClipperLib_Z::ZFillCallback z_filler = [hash_function, subject_idx_range](
            const IntPoint &e1_a, const IntPoint &e1_b, const IntPoint &e2_a,
            const IntPoint &e2_b, IntPoint &d) {
        // FillFloatingConcentric.cpp:440-445 — both edges from the subject:
        // the intersect is generated by two lines in subject -> keep subject z.
        if (e1_a.z() == e1_b.z() && e1_b.z() == e2_a.z() && e2_a.z() == e2_b.z()) {
            d.z() = e1_a.z();
            return;
        }
        // FillFloatingConcentric.cpp:447 — subject x clip intersection -> negative hash.
        d.z() = -hash_function(e1_a.z(), e1_b.z(), e2_a.z(), e2_b.z());
    };

    // FillFloatingConcentric.cpp:457-464 — ctIntersection pass.
    Paths intersect_out;
    {
        ClipperLib_Z::Clipper c;
        ClipperLib_Z::PolyTree polytree;
        c.ZFillFunction(z_filler);
        c.AddPaths(subject_paths, ClipperLib_Z::ptSubject, false);
        c.AddPaths(clip_paths, ClipperLib_Z::ptClip, true);
        c.Execute(ClipperLib_Z::ctIntersection, polytree, ClipperLib_Z::pftNonZero);
        ClipperLib_Z::PolyTreeToPaths(polytree, intersect_out);
    }

    // FillFloatingConcentric.cpp:467-474 — ctDifference pass.
    Paths diff_out;
    {
        ClipperLib_Z::Clipper c;
        ClipperLib_Z::PolyTree polytree;
        c.ZFillFunction(z_filler);
        c.AddPaths(subject_paths, ClipperLib_Z::ptSubject, false);
        c.AddPaths(clip_paths, ClipperLib_Z::ptClip, true);
        c.Execute(ClipperLib_Z::ctDifference, polytree, ClipperLib_Z::pftNonZero);
        ClipperLib_Z::PolyTreeToPaths(polytree, diff_out);
    }

    // FillFloatingConcentric.cpp:477-481 — to_merge = diff_out ++ intersect_out;
    // floating_flags true for the intersect tail. Concatenate diff-first so the
    // Rust caller can split at out_num_diff_paths.
    if (out_num_diff_paths)
        *out_num_diff_paths = (int32_t) diff_out.size();
    Paths to_merge = std::move(diff_out);
    to_merge.insert(to_merge.end(), intersect_out.begin(), intersect_out.end());

    return marshal_zpaths(to_merge);
}

extern "C" void cz_free_zpaths(CzZPaths paths) {
    std::free(paths.coords);
    std::free(paths.path_lens);
}

// ---------------------------------------------------------------------------
// R195: cz_union_flat — faithful ClipperUtils.cpp clipper_union<Paths>
// (ClipperUtils.cpp:339-350): ONE Clipper Execute(ctUnion, Paths, pftNonZero).
// Used by expolygons_offset (positive delta, >1 expolygons collected) — the
// lower_polygons_series grow. NOT the two-pass union_ex; NOT StrictlySimple.
// ---------------------------------------------------------------------------
extern "C" CzZPaths cz_union_flat(const int32_t *xy, const int32_t *lens, int32_t num) {
    ClipperLib::Paths subject = read_closed_paths(xy, lens, num);
    ClipperLib::Clipper clipper;
    clipper.AddPaths(subject, ClipperLib::ptSubject, true);
    ClipperLib::Paths retval;
    clipper.Execute(ClipperLib::ctUnion, retval, ClipperLib::pftNonZero, ClipperLib::pftNonZero);
    return marshal_paths(retval);
}
