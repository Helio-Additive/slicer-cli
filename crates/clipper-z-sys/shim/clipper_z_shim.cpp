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
#include <algorithm>
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

// R792 — faithful `_clipper_pl_open(ctIntersection, subject, clip)`
// (ClipperUtils.cpp:841-849, the intersection_pl family): OPEN subject
// polylines against closed clip polygons, Execute into a PolyTree
// (pftNonZero/pftNonZero), then the PolyTreeToPolylines preorder flatten
// (ClipperUtils.cpp:213-229 — parent contour before children).
extern "C" CzZPaths cz_intersection_pl(const int32_t *subject_xy, const int32_t *subject_lens,
                                       int32_t subject_num, const int32_t *clip_xy,
                                       const int32_t *clip_lens, int32_t clip_num) {
    ClipperLib::Paths subject = read_closed_paths(subject_xy, subject_lens, subject_num);
    ClipperLib::Paths clip = read_closed_paths(clip_xy, clip_lens, clip_num);

    ClipperLib::Clipper clipper;
    clipper.AddPaths(subject, ClipperLib::ptSubject, false);
    clipper.AddPaths(clip, ClipperLib::ptClip, true);
    ClipperLib::PolyTree tree;
    clipper.Execute(ClipperLib::ctIntersection, tree, ClipperLib::pftNonZero,
                    ClipperLib::pftNonZero);

    ClipperLib::Paths out;
    struct Rec {
        static void add(ClipperLib::PolyNode &node, ClipperLib::Paths &o) {
            if (!node.Contour.empty()) o.push_back(node.Contour);
            for (ClipperLib::PolyNode *child : node.Childs) add(*child, o);
        }
    };
    Rec::add(tree, out);
    return marshal_paths(out);
}

// cz_diff_pl — `_clipper_pl_open(ctDifference, subject, clip)` (ClipperUtils.cpp
// 841-849, the diff_pl family, ClipperUtils.cpp:909-920): the same call as
// cz_intersection_pl with the other clip type. The parts of the OPEN subject
// polylines outside the closed clip polygons, in PolyTree preorder.
extern "C" CzZPaths cz_diff_pl(const int32_t *subject_xy, const int32_t *subject_lens,
                               int32_t subject_num, const int32_t *clip_xy,
                               const int32_t *clip_lens, int32_t clip_num) {
    ClipperLib::Paths subject = read_closed_paths(subject_xy, subject_lens, subject_num);
    ClipperLib::Paths clip = read_closed_paths(clip_xy, clip_lens, clip_num);

    ClipperLib::Clipper clipper;
    clipper.AddPaths(subject, ClipperLib::ptSubject, false);
    clipper.AddPaths(clip, ClipperLib::ptClip, true);
    ClipperLib::PolyTree tree;
    clipper.Execute(ClipperLib::ctDifference, tree, ClipperLib::pftNonZero,
                    ClipperLib::pftNonZero);

    ClipperLib::Paths out;
    struct Rec {
        static void add(ClipperLib::PolyNode &node, ClipperLib::Paths &o) {
            if (!node.Contour.empty()) o.push_back(node.Contour);
            for (ClipperLib::PolyNode *child : node.Childs) add(*child, o);
        }
    };
    Rec::add(tree, out);
    return marshal_paths(out);
}

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

// ---------------------------------------------------------------------------
// cz_variable_offset_inner_ex — VERBATIM adaptation of ClipperUtils.cpp:
// mittered_offset_path_scaled (:1120-1258), fix_after_inner/outer_offset
// (:1077-1118) and variable_offset_inner_ex (:1390-1440). Per-vertex mitered
// offset in doubles with the native rounding (+0.5-with-sign trunc), Clipper1
// NEGATIVE-fill cleanup with the enclosing-rect trick, ctDifference PolyTree
// assembly. deltas are SCALED floats, one per input vertex per ring.
// ---------------------------------------------------------------------------
namespace vo {

static constexpr const double kShortestEdgeFactor = 0.005; // ClipperOffsetShortestEdgeFactor

static inline double vo_cross2(double ax, double ay, double bx, double by) { return ax * by - ay * bx; }

static ClipperLib::Path mittered_offset_path_scaled(const std::vector<std::pair<double,double>> &contour,
                                                    const float *deltas, size_t ndeltas, double miter_limit)
{
    ClipperLib::Path out;
    if (ndeltas > 2) {
        out.reserve(contour.size() * 2);
        miter_limit = (miter_limit > 2.) ? 2. / (miter_limit * miter_limit) : 0.5;
        auto add_offset_point = [&out](double x, double y) {
            x += 0.5 - (x < 0);
            y += 0.5 - (y < 0);
            out.emplace_back(ClipperLib::cInt(x), ClipperLib::cInt(y));
        };
        double lmin = double(*std::max_element(deltas, deltas + ndeltas)) * kShortestEdgeFactor;
        double l2min = lmin * lmin;
        const double sin_min_parallel = 1.;

        double ptx = contour.front().first, pty = contour.front().second;
        size_t iprev = contour.size() - 1;
        double ppx = 0, ppy = 0;
        for (; iprev > 0; --iprev) {
            ppx = contour[iprev].first; ppy = contour[iprev].second;
            double dx = ppx - ptx, dy = ppy - pty;
            if (dx * dx + dy * dy > l2min)
                break;
        }
        if (iprev != 0) {
            size_t ilast = iprev;
            // perp(pt - ptprev).normalized()
            double vx = ptx - ppx, vy = pty - ppy;
            double npx = vy, npy = -vx;
            { double n = std::sqrt(npx * npx + npy * npy); npx /= n; npy /= n; }
            for (size_t i = 0; ;) {
                size_t j = i + 1;
                double nxx = 0, nxy = 0; // ptnext
                for (; j <= ilast; ++j) {
                    nxx = contour[j].first; nxy = contour[j].second;
                    double dx = nxx - ptx, dy = nxy - pty;
                    if (dx * dx + dy * dy > l2min)
                        break;
                }
                if (j > ilast) {
                    i = ilast;
                    nxx = contour.front().first; nxy = contour.front().second;
                }
                double wx = nxx - ptx, wy = nxy - pty;
                double nnx = wy, nny = -wx;
                { double n = std::sqrt(nnx * nnx + nny * nny); nnx /= n; nny /= n; }

                double delta = deltas[i];
                double sin_a = vo_cross2(npx, npy, nnx, nny);
                if (sin_a < -1.) sin_a = -1.; else if (sin_a > 1.) sin_a = 1.;
                double convex = sin_a * delta;
                if (convex <= -sin_min_parallel) {
                    add_offset_point(ptx + npx * delta, pty + npy * delta);
                    add_offset_point(ptx, pty);
                    add_offset_point(ptx + nnx * delta, pty + nny * delta);
                } else {
                    double dot = npx * nnx + npy * nny;
                    if (convex < sin_min_parallel && dot > 0.) {
                        if (dot > 0.)
                            add_offset_point(ptx + npx * delta, pty + npy * delta);
                        else
                            add_offset_point(ptx, pty);
                    } else {
                        double r = 1. + dot;
                        if (r >= miter_limit) {
                            add_offset_point(ptx + (npx + nnx) * (delta / r), pty + (npy + nny) * (delta / r));
                        } else {
                            double dx = std::tan(std::atan2(sin_a, dot) / 4.);
                            // perp(nprev) = (npy, -npx); perp(nnext) = (nny, -nnx)
                            add_offset_point(ptx + (npx - npy * dx) * delta, pty + (npy + npx * dx) * delta);
                            add_offset_point(ptx + (nnx + nny * dx) * delta, pty + (nny - nnx * dx) * delta);
                        }
                    }
                }
                if (i == ilast)
                    break;
                ppx = ptx; ppy = pty;
                npx = nnx; npy = nny;
                ptx = nxx; pty = nxy;
                i = j;
            }
        }
    }
    return out;
}

static ClipperLib::Paths fix_after_outer_offset(const ClipperLib::Path &input,
                                                ClipperLib::PolyFillType filltype, bool reverse_result)
{
    ClipperLib::Paths solution;
    if (!input.empty()) {
        ClipperLib::Clipper clipper;
        clipper.AddPath(input, ClipperLib::ptSubject, true);
        clipper.ReverseSolution(reverse_result);
        clipper.Execute(ClipperLib::ctUnion, solution, filltype, filltype);
    }
    return solution;
}

static ClipperLib::Paths fix_after_inner_offset(const ClipperLib::Path &input,
                                                ClipperLib::PolyFillType filltype, bool reverse_result)
{
    ClipperLib::Paths solution;
    if (!input.empty()) {
        ClipperLib::Clipper clipper;
        clipper.AddPath(input, ClipperLib::ptSubject, true);
        ClipperLib::IntRect r = clipper.GetBounds();
        r.left -= 10; r.top -= 10; r.right += 10; r.bottom += 10;
        if (filltype == ClipperLib::pftPositive)
            clipper.AddPath({ ClipperLib::IntPoint(r.left, r.bottom), ClipperLib::IntPoint(r.left, r.top),
                              ClipperLib::IntPoint(r.right, r.top), ClipperLib::IntPoint(r.right, r.bottom) },
                            ClipperLib::ptSubject, true);
        else
            clipper.AddPath({ ClipperLib::IntPoint(r.left, r.bottom), ClipperLib::IntPoint(r.right, r.bottom),
                              ClipperLib::IntPoint(r.right, r.top), ClipperLib::IntPoint(r.left, r.top) },
                            ClipperLib::ptSubject, true);
        clipper.ReverseSolution(reverse_result);
        clipper.Execute(ClipperLib::ctUnion, solution, filltype, filltype);
        if (!solution.empty())
            solution.erase(solution.begin());
    }
    return solution;
}

} // namespace vo

// ---------------------------------------------------------------------------
// cz_smooth_compensation_banded — VERBATIM adaptation of
// ElephantFootCompensation.cpp:465-532 (+ Utils.hpp prev/next_idx_modulo,
// libslic3r.h lerp). Compiled with the SAME clang as native, so its FMA
// contraction (sqrt/norm/lerp/laplacian) is bit-identical by construction —
// which pure-Rust arithmetic cannot reproduce (R323). Modifies `comp` in place.
// `xy` = n scaled i32 (x,y) pairs (float-cast internally like native Vec2f).
// ---------------------------------------------------------------------------
namespace efc {

static inline int prev_idx_modulo(int idx, int count) { if (idx == 0) idx = count; return --idx; }
static inline int next_idx_modulo(int idx, int count) { if (++idx == count) idx = 0; return idx; }
static inline float efc_lerp(float a, float b, float t) { return (1.f - t) * a + t * b; }

} // namespace efc

extern "C" void cz_smooth_compensation_banded(const int32_t *xy, int32_t n,
                                              float *comp, float band,
                                              float strength, int32_t num_iterations) {
    // contour points as float (Vec2f), matching native contour[i].cast<float>().
    std::vector<float> cx(n), cy(n);
    for (int32_t i = 0; i < n; ++i) { cx[i] = float(xy[2 * i]); cy[i] = float(xy[2 * i + 1]); }

    std::vector<float> compensation(comp, comp + n);
    std::vector<float> out(compensation);
    float dist_min2 = band * band;
    static constexpr bool use_min = false;

    for (int32_t iter = 0; iter < num_iterations; ++iter) {
        for (int i = 0; i < n; ++i) {
            const float pthis_x = cx[i], pthis_y = cy[i];

            // --- previous direction ---
            int   j     = efc::prev_idx_modulo(i, n);
            float pprev_x = cx[j], pprev_y = cy[j];
            float prev  = compensation[j];
            float dx = pthis_x - pprev_x, dy = pthis_y - pprev_y;
            float l2 = dx * dx + dy * dy;
            if (l2 < dist_min2) {
                float l = std::sqrt(l2);
                int jprev = j; j = efc::prev_idx_modulo(j, n);
                while (j != i) {
                    const float pp_x = cx[j], pp_y = cy[j];
                    float ex = pp_x - pprev_x, ey = pp_y - pprev_y;
                    const float lthis = std::sqrt(ex * ex + ey * ey);
                    const float lnext = l + lthis;
                    if (lnext > band) {
                        float t = (band - l) / lthis;
                        prev = use_min ? std::min(prev, efc::efc_lerp(compensation[jprev], compensation[j], t))
                                       : efc::efc_lerp(compensation[jprev], compensation[j], t);
                        break;
                    }
                    prev  = use_min ? std::min(prev, compensation[j]) : compensation[j];
                    pprev_x = pp_x; pprev_y = pp_y;
                    l = lnext;
                    jprev = j; j = efc::prev_idx_modulo(j, n);
                }
            }

            // --- next direction ---
            j = efc::next_idx_modulo(i, n);
            pprev_x = cx[j]; pprev_y = cy[j];
            float next = compensation[j];
            dx = pprev_x - pthis_x; dy = pprev_y - pthis_y;
            l2 = dx * dx + dy * dy;
            if (l2 < dist_min2) {
                float l = std::sqrt(l2);
                int jprev = j; j = efc::next_idx_modulo(j, n);
                while (j != i) {
                    const float pp_x = cx[j], pp_y = cy[j];
                    float ex = pp_x - pprev_x, ey = pp_y - pprev_y;
                    const float lthis = std::sqrt(ex * ex + ey * ey);
                    const float lnext = l + lthis;
                    if (lnext > band) {
                        float t = (band - l) / lthis;
                        next = use_min ? std::min(next, efc::efc_lerp(compensation[jprev], compensation[j], t))
                                       : efc::efc_lerp(compensation[jprev], compensation[j], t);
                        break;
                    }
                    next  = use_min ? std::min(next, compensation[j]) : compensation[j];
                    pprev_x = pp_x; pprev_y = pp_y;
                    l = lnext;
                    jprev = j; j = efc::next_idx_modulo(j, n);
                }
            }

            float laplacian = compensation[i] * (1.f - strength) + 0.5f * strength * (prev + next);
            out[i] = std::max(laplacian, compensation[i]);
        }
        out.swap(compensation);
    }
    for (int32_t i = 0; i < n; ++i) comp[i] = compensation[i];
}

// Input: one ExPolygon (contour + holes, i32 xy pairs, ring lens) + per-ring
// per-vertex SCALED f32 deltas (flat, ring-major, same counts as ring points)
// + miter_limit. Output: grouped ExPolygons (cz_union_ex layout).
extern "C" CzZPaths cz_variable_offset_inner_ex(const int32_t *xy, const int32_t *lens, int32_t num,
                                                const float *deltas, double miter_limit) {
    // Unflatten rings.
    std::vector<std::vector<std::pair<double,double>>> rings;
    rings.reserve(num);
    size_t off = 0;
    for (int32_t p = 0; p < num; ++p) {
        std::vector<std::pair<double,double>> ring;
        ring.reserve(lens[p]);
        for (int32_t i = 0; i < lens[p]; ++i, ++off)
            ring.emplace_back(double(xy[2 * off]), double(xy[2 * off + 1]));
        rings.emplace_back(std::move(ring));
    }
    const float *d = deltas;
    // 1) contour
    ClipperLib::Paths contours = vo::fix_after_inner_offset(
        vo::mittered_offset_path_scaled(rings[0], d, rings[0].size(), miter_limit),
        ClipperLib::pftNegative, true);
    d += rings[0].size();
    // 2) holes
    ClipperLib::Paths holes;
    for (int32_t h = 1; h < num; ++h) {
        ClipperLib::Paths fixed = vo::fix_after_outer_offset(
            vo::mittered_offset_path_scaled(rings[h], d, rings[h].size(), miter_limit),
            ClipperLib::pftNegative, false);
        holes.insert(holes.end(), fixed.begin(), fixed.end());
        d += rings[h].size();
    }
    // 3) subtract, PolyTree assembly (variable_offset_inner_ex tail).
    ClipperLib::Paths out_paths;
    std::vector<int32_t> out_is_hole;
    if (holes.empty()) {
        for (ClipperLib::Path &path : contours) { out_paths.emplace_back(std::move(path)); out_is_hole.push_back(0); }
    } else {
        ClipperLib::Clipper clipper;
        clipper.AddPaths(contours, ClipperLib::ptSubject, true);
        clipper.AddPaths(holes, ClipperLib::ptClip, true);
        ClipperLib::PolyTree polytree;
        clipper.Execute(ClipperLib::ctDifference, polytree, ClipperLib::pftNonZero, ClipperLib::pftNonZero);
        for (int i = 0; i < polytree.ChildCount(); ++i)
            polytree_to_grouped(*polytree.Childs[i], out_paths, out_is_hole);
    }
    return marshal_grouped(out_paths, out_is_hole);
}

// cz_offset_ex_paths — faithful `offset_ex(const Polygons&, float delta)`
// (ClipperUtils.cpp:415): PolyTreeToExPolygons(offset_paths<PolyTree>(paths,
// delta)). raw_offset runs ONE ClipperOffset per path with the orientation-
// aware signum (ClipperUtils.cpp:273-300); delta > 0 reconstructs via
// clipper_union<PolyTree>(pftNonZero), delta < 0 via the shrink trick — a
// bounding rectangle + ReverseSolution + ctUnion(pftNegative) +
// RemoveOutermostPolygon (ClipperUtils.cpp:381-396). Output uses the
// cz_union_ex grouped layout. `delta` is SCALED and already f32-rounded by the
// caller (the C++ parameter is float).
extern "C" CzZPaths cz_offset_ex_paths(const int32_t *xy, const int32_t *lens, int32_t num,
                                       double delta, int32_t join_type, double miter_limit) {
    ClipperLib::Paths paths = read_closed_paths(xy, lens, num);
    ClipperLib::JoinType jt = ClipperLib::jtMiter;
    if (join_type == 1) jt = ClipperLib::jtRound;
    else if (join_type == 2) jt = ClipperLib::jtSquare;

    // raw_offset (ClipperUtils.cpp:273): per-path ClipperOffset, orientation signum.
    ClipperLib::Paths raw;
    raw.reserve(paths.size());
    {
        ClipperLib::ClipperOffset co;
        ClipperLib::Paths out_this;
        if (jt == ClipperLib::jtRound)
            co.ArcTolerance = miter_limit;
        else
            co.MiterLimit = miter_limit;
        co.ShortestEdgeLength = std::fabs(delta * 0.005); // ClipperOffsetShortestEdgeFactor
        for (const ClipperLib::Path &path : paths) {
            co.Clear();
            co.AddPath(path, jt, ClipperLib::etClosedPolygon);
            const bool ccw = ClipperLib::Orientation(path);
            co.Execute(out_this, ccw ? delta : -delta);
            if (!ccw)
                for (ClipperLib::Path &p : out_this)
                    std::reverse(p.begin(), p.end());
            for (ClipperLib::Path &p : out_this)
                raw.emplace_back(std::move(p));
            out_this.clear();
        }
    }

    ClipperLib::PolyTree polytree;
    if (delta > 0) {
        // expand_paths: clipper_union<PolyTree>(raw, pftNonZero).
        if (!raw.empty()) {
            ClipperLib::Clipper c;
            c.AddPaths(raw, ClipperLib::ptSubject, true);
            c.Execute(ClipperLib::ctUnion, polytree, ClipperLib::pftNonZero, ClipperLib::pftNonZero);
        }
    } else {
        // shrink_paths (ClipperUtils.cpp:381-396).
        if (!raw.empty()) {
            ClipperLib::Clipper clipper;
            clipper.AddPaths(raw, ClipperLib::ptSubject, true);
            ClipperLib::IntRect r = clipper.GetBounds();
            clipper.AddPath({{r.left - 10, r.bottom + 10},
                             {r.right + 10, r.bottom + 10},
                             {r.right + 10, r.top - 10},
                             {r.left - 10, r.top - 10}},
                            ClipperLib::ptSubject, true);
            clipper.ReverseSolution(true);
            clipper.Execute(ClipperLib::ctUnion, polytree, ClipperLib::pftNegative, ClipperLib::pftNegative);
            polytree.RemoveOutermostPolygon();
        }
    }

    ClipperLib::Paths out_paths;
    std::vector<int32_t> out_is_hole;
    for (int i = 0; i < polytree.ChildCount(); ++i)
        polytree_to_grouped(*polytree.Childs[i], out_paths, out_is_hole);
    return marshal_grouped(out_paths, out_is_hole);
}

// cz_offset_paths — faithful `offset(const Polygons&, float delta)`
// (ClipperUtils.cpp:413): to_polygons(offset_paths<ClipperLib::Paths>(paths,
// delta)). Same raw_offset as cz_offset_ex_paths; delta > 0 reconstructs via
// clipper_union<Paths>(pftNonZero), delta <= 0 via shrink_paths<Paths> (bounding
// rectangle + ReverseSolution + ctUnion(pftNegative) + erase(begin)). The
// result is the flat Paths in Clipper's own BuildResult order — the ring START
// vertex is part of the contract (R803: WallToolPaths::generate simplifies the
// prepared outline from index 0, so a rotated ring simplifies differently).
extern "C" CzZPaths cz_offset_paths(const int32_t *xy, const int32_t *lens, int32_t num,
                                    double delta, int32_t join_type, double miter_limit) {
    ClipperLib::Paths paths = read_closed_paths(xy, lens, num);
    ClipperLib::JoinType jt = ClipperLib::jtMiter;
    if (join_type == 1) jt = ClipperLib::jtRound;
    else if (join_type == 2) jt = ClipperLib::jtSquare;

    ClipperLib::Paths raw;
    raw.reserve(paths.size());
    {
        ClipperLib::ClipperOffset co;
        ClipperLib::Paths out_this;
        if (jt == ClipperLib::jtRound)
            co.ArcTolerance = miter_limit;
        else
            co.MiterLimit = miter_limit;
        co.ShortestEdgeLength = std::fabs(delta * 0.005); // ClipperOffsetShortestEdgeFactor
        for (const ClipperLib::Path &path : paths) {
            co.Clear();
            co.AddPath(path, jt, ClipperLib::etClosedPolygon);
            const bool ccw = ClipperLib::Orientation(path);
            co.Execute(out_this, ccw ? delta : -delta);
            if (!ccw)
                for (ClipperLib::Path &p : out_this)
                    std::reverse(p.begin(), p.end());
            for (ClipperLib::Path &p : out_this)
                raw.emplace_back(std::move(p));
            out_this.clear();
        }
    }

    ClipperLib::Paths out;
    if (delta > 0) {
        if (!raw.empty()) {
            ClipperLib::Clipper c;
            c.AddPaths(raw, ClipperLib::ptSubject, true);
            c.Execute(ClipperLib::ctUnion, out, ClipperLib::pftNonZero, ClipperLib::pftNonZero);
        }
    } else {
        if (!raw.empty()) {
            ClipperLib::Clipper clipper;
            clipper.AddPaths(raw, ClipperLib::ptSubject, true);
            ClipperLib::IntRect r = clipper.GetBounds();
            clipper.AddPath({{r.left - 10, r.bottom + 10},
                             {r.right + 10, r.bottom + 10},
                             {r.right + 10, r.top - 10},
                             {r.left - 10, r.top - 10}},
                            ClipperLib::ptSubject, true);
            clipper.ReverseSolution(true);
            clipper.Execute(ClipperLib::ctUnion, out, ClipperLib::pftNegative, ClipperLib::pftNegative);
            if (!out.empty())
                out.erase(out.begin());
        }
    }
    return marshal_paths(out);
}

// cz_union_polytree — `PolyTreeToExPolygons(clipper_union<ClipperLib::PolyTree>(
// paths, pftNonZero))` (ClipperUtils.cpp:339-350, 203-210): ONE union into a
// PolyTree. This is the second half of clipper_do_polytree (ClipperUtils.cpp
// 641-654), i.e. what diff_ex / intersection_ex run over the raw output of their
// boolean operation, and the whole reconstruction of offset_ex(ExPolygons)
// (expolygons_offset_pt, ClipperUtils.cpp:552-557). cz_union_ex is the two-pass
// union_ex and is one union too many for those. Grouped output like cz_union_ex.
extern "C" CzZPaths cz_union_polytree(const int32_t *xy, const int32_t *lens, int32_t num) {
    ClipperLib::Paths subject = read_closed_paths(xy, lens, num);
    ClipperLib::PolyTree polytree;
    {
        ClipperLib::Clipper c;
        c.AddPaths(subject, ClipperLib::ptSubject, true);
        c.Execute(ClipperLib::ctUnion, polytree, ClipperLib::pftNonZero, ClipperLib::pftNonZero);
    }
    ClipperLib::Paths out_paths;
    std::vector<int32_t> out_is_hole;
    for (int i = 0; i < polytree.ChildCount(); ++i)
        polytree_to_grouped(*polytree.Childs[i], out_paths, out_is_hole);
    return marshal_grouped(out_paths, out_is_hole);
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

// ---------------------------------------------------------------------------
// R885 — reference exports for the Rust port of clipper.cpp (crates/libslic3r-rs/
// src/clipper.rs). Not called by the engine: tests/clipper_native_parity.rs
// compares the port with the library through them.
// ---------------------------------------------------------------------------
namespace {
ClipperLib::Path cz_ref_path(const int32_t *xy, int32_t n)
{
    ClipperLib::Path p;
    p.reserve(n);
    for (int32_t i = 0; i < n; ++i)
        p.emplace_back(xy[2 * i], xy[2 * i + 1]);
    return p;
}
} // namespace

extern "C" double cz_ref_area(const int32_t *xy, int32_t n)
{
    return ClipperLib::Area(cz_ref_path(xy, n));
}

extern "C" int32_t cz_ref_point_in_polygon(int32_t px, int32_t py, const int32_t *xy, int32_t n)
{
    return ClipperLib::PointInPolygon(ClipperLib::IntPoint(px, py), cz_ref_path(xy, n));
}

// Writes at most n points to out_xy and returns the number written.
extern "C" int32_t cz_ref_clean_polygon(const int32_t *xy, int32_t n, double distance, int32_t *out_xy)
{
    ClipperLib::Path out;
    ClipperLib::CleanPolygon(cz_ref_path(xy, n), out, distance);
    for (size_t i = 0; i < out.size(); ++i) {
        out_xy[2 * i]     = out[i].x();
        out_xy[2 * i + 1] = out[i].y();
    }
    return int32_t(out.size());
}

// ---------------------------------------------------------------------------
// Reference export for the Rust port of the Clipper class (clipper.cpp
// 1048-3313, 3762-3826). Not called by the engine: tests/
// clipper_execute_native_parity.rs compares Clipper::execute of the port with it.
// Every subject goes in with AddPath(ptSubject, closed), every clip with
// AddPath(ptClip, closed); the solution comes back in the order, and each path
// with the start vertex, that Execute produced. clip_type and the fill types are
// the values of ClipperLib::ClipType and ClipperLib::PolyFillType. *succeeded
// (when not null) receives what Execute returned.
// ---------------------------------------------------------------------------
extern "C" CzZPaths cz_ref_execute(const int32_t *subj_xy, const int32_t *subj_lens, int32_t subj_num,
                                   const int32_t *clip_xy, const int32_t *clip_lens, int32_t clip_num,
                                   int32_t clip_type, int32_t subj_fill, int32_t clip_fill,
                                   int32_t strictly_simple, int32_t preserve_collinear,
                                   int32_t reverse_solution, int32_t *succeeded)
{
    ClipperLib::Paths subject = read_closed_paths(subj_xy, subj_lens, subj_num);
    ClipperLib::Paths clip    = read_closed_paths(clip_xy, clip_lens, clip_num);
    ClipperLib::Clipper c;
    c.StrictlySimple(strictly_simple != 0);
    c.PreserveCollinear(preserve_collinear != 0);
    c.ReverseSolution(reverse_solution != 0);
    for (const ClipperLib::Path &p : subject)
        c.AddPath(p, ClipperLib::ptSubject, true);
    for (const ClipperLib::Path &p : clip)
        c.AddPath(p, ClipperLib::ptClip, true);
    ClipperLib::Paths solution;
    bool ok = c.Execute(static_cast<ClipperLib::ClipType>(clip_type), solution,
                        static_cast<ClipperLib::PolyFillType>(subj_fill),
                        static_cast<ClipperLib::PolyFillType>(clip_fill));
    if (succeeded != nullptr)
        *succeeded = ok ? 1 : 0;
    return marshal_paths(solution);
}

// ---------------------------------------------------------------------------
// Reference exports for the Rust port of PolyNode / PolyTree, the PolyTree
// Execute of Clipper and ClipperOffset (clipper.cpp 122-171, 1098-1110,
// 2791-2843, 3315-3751, 3842-3850, 4093-4132). Not called by the engine:
// tests/clipper_polytree_native_parity.rs and tests/clipper_offset_native_parity.rs
// compare the port with them.
//
// The encoding of a PolyTree: the contours of its nodes as a CzZPaths (z always
// 0) in pre-order (a node, then its children in the order of Childs, each with
// its own descendants; the tree itself is not listed), every contour with the
// points and the start vertex the library produced. For node i of that listing
// node_info[4*i .. 4*i+4] receives: the index in the listing of its Parent (-1
// when the Parent is the tree itself), IsHole(), IsOpen(), ChildCount(). At
// most node_info_cap nodes are written; *num_nodes receives the number of nodes
// listed, *total what PolyTree::Total() returns.
// ---------------------------------------------------------------------------
namespace {
void cz_ref_list_node(const ClipperLib::PolyNode &node, int32_t parent, ClipperLib::Paths &contours,
                      std::vector<int32_t> &info)
{
    for (int i = 0; i < node.ChildCount(); ++i) {
        const ClipperLib::PolyNode &child = *node.Childs[i];
        int32_t me = int32_t(contours.size());
        contours.push_back(child.Contour);
        info.push_back(parent);
        info.push_back(child.IsHole() ? 1 : 0);
        info.push_back(child.IsOpen() ? 1 : 0);
        info.push_back(child.ChildCount());
        cz_ref_list_node(child, me, contours, info);
    }
}

CzZPaths cz_ref_marshal_polytree(const ClipperLib::PolyTree &tree, int32_t *node_info, int32_t node_info_cap,
                                 int32_t *num_nodes, int32_t *total)
{
    ClipperLib::Paths contours;
    std::vector<int32_t> info;
    cz_ref_list_node(tree, -1, contours, info);
    if (node_info != nullptr)
        for (size_t i = 0; i < info.size() && i < size_t(node_info_cap) * 4; ++i)
            node_info[i] = info[i];
    if (num_nodes != nullptr)
        *num_nodes = int32_t(contours.size());
    if (total != nullptr)
        *total = contours.empty() && tree.ChildCount() == 0 ? 0 : tree.Total();
    return marshal_paths(contours);
}
} // namespace

// ClipperLib::Clipper::Execute(clip_type, PolyTree&, subj_fill, clip_fill).
// Subject i is closed when subj_closed[i] != 0 and open otherwise; the clips are
// closed. With add_paths == 0 every path goes in through AddPath, subjects first,
// in the given order; otherwise through three AddPaths calls: the closed
// subjects, the open subjects, the clips. output selects what comes back:
//   0: the tree, in the encoding above;
//   1: PolyTreeToPaths(tree);  2: ClosedPathsFromPolyTree(tree);
//   3: OpenPathsFromPolyTree(tree)   (node_info is not written for 1-3).
// *succeeded receives what Execute returned. Free via cz_free_zpaths.
extern "C" CzZPaths cz_ref_execute_polytree(const int32_t *subj_xy, const int32_t *subj_lens,
                                            const int32_t *subj_closed, int32_t subj_num,
                                            const int32_t *clip_xy, const int32_t *clip_lens, int32_t clip_num,
                                            int32_t clip_type, int32_t subj_fill, int32_t clip_fill,
                                            int32_t strictly_simple, int32_t preserve_collinear,
                                            int32_t reverse_solution, int32_t add_paths, int32_t output,
                                            int32_t *node_info, int32_t node_info_cap, int32_t *num_nodes,
                                            int32_t *total, int32_t *succeeded)
{
    ClipperLib::Paths subject = read_closed_paths(subj_xy, subj_lens, subj_num);
    ClipperLib::Paths clip    = read_closed_paths(clip_xy, clip_lens, clip_num);
    ClipperLib::Clipper c;
    c.StrictlySimple(strictly_simple != 0);
    c.PreserveCollinear(preserve_collinear != 0);
    c.ReverseSolution(reverse_solution != 0);
    if (add_paths == 0) {
        for (size_t i = 0; i < subject.size(); ++i)
            c.AddPath(subject[i], ClipperLib::ptSubject, subj_closed[i] != 0);
        for (const ClipperLib::Path &p : clip)
            c.AddPath(p, ClipperLib::ptClip, true);
    } else {
        ClipperLib::Paths closed, open;
        for (size_t i = 0; i < subject.size(); ++i)
            (subj_closed[i] != 0 ? closed : open).push_back(subject[i]);
        c.AddPaths(closed, ClipperLib::ptSubject, true);
        c.AddPaths(open, ClipperLib::ptSubject, false);
        c.AddPaths(clip, ClipperLib::ptClip, true);
    }
    ClipperLib::PolyTree tree;
    bool ok = c.Execute(static_cast<ClipperLib::ClipType>(clip_type), tree,
                        static_cast<ClipperLib::PolyFillType>(subj_fill),
                        static_cast<ClipperLib::PolyFillType>(clip_fill));
    if (succeeded != nullptr)
        *succeeded = ok ? 1 : 0;
    if (output == 0)
        return cz_ref_marshal_polytree(tree, node_info, node_info_cap, num_nodes, total);
    ClipperLib::Paths paths;
    if (output == 1)
        ClipperLib::PolyTreeToPaths(tree, paths);
    else if (output == 2)
        ClipperLib::ClosedPathsFromPolyTree(tree, paths);
    else
        ClipperLib::OpenPathsFromPolyTree(tree, paths);
    return marshal_paths(paths);
}

// ClipperLib::ClipperOffset(miter_limit, arc_tolerance, shortest_edge_length):
// path i goes in through AddPath(path, join_types[i], end_types[i]) (the values
// of ClipperLib::JoinType and ClipperLib::EndType), then Execute(solution,
// delta). With use_polytree == 0 the solution is a Paths and comes back as
// produced; otherwise it is a PolyTree and comes back in the encoding above
// (node_info, num_nodes, total as in cz_ref_execute_polytree). Free via
// cz_free_zpaths.
extern "C" CzZPaths cz_ref_offset(const int32_t *xy, const int32_t *lens, const int32_t *join_types,
                                  const int32_t *end_types, int32_t num, double miter_limit,
                                  double arc_tolerance, double shortest_edge_length, double delta,
                                  int32_t use_polytree, int32_t *node_info, int32_t node_info_cap,
                                  int32_t *num_nodes, int32_t *total)
{
    ClipperLib::Paths paths = read_closed_paths(xy, lens, num);
    ClipperLib::ClipperOffset co(miter_limit, arc_tolerance, shortest_edge_length);
    for (size_t i = 0; i < paths.size(); ++i)
        co.AddPath(paths[i], static_cast<ClipperLib::JoinType>(join_types[i]),
                   static_cast<ClipperLib::EndType>(end_types[i]));
    if (use_polytree == 0) {
        ClipperLib::Paths solution;
        co.Execute(solution, delta);
        return marshal_paths(solution);
    }
    ClipperLib::PolyTree tree;
    co.Execute(tree, delta);
    return cz_ref_marshal_polytree(tree, node_info, node_info_cap, num_nodes, total);
}

// ClipperLib::SimplifyPolygon(path, fill_type) when which == 0 (the first path
// only), ClipperLib::SimplifyPolygons(paths, fill_type) otherwise. Free via
// cz_free_zpaths.
extern "C" CzZPaths cz_ref_simplify_polygons(const int32_t *xy, const int32_t *lens, int32_t num,
                                             int32_t fill_type, int32_t which)
{
    ClipperLib::Paths paths = read_closed_paths(xy, lens, num);
    ClipperLib::PolyFillType ft = static_cast<ClipperLib::PolyFillType>(fill_type);
    if (which == 0)
        return marshal_paths(ClipperLib::SimplifyPolygon(paths.empty() ? ClipperLib::Path() : paths.front(), ft));
    return marshal_paths(ClipperLib::SimplifyPolygons(paths, ft));
}

// ---------------------------------------------------------------------------
// Reference exports for the Rust port of the CLIPPERLIB_USE_XYZ build of
// clipper.cpp, namespace ClipperLib_Z (crates/libslic3r-rs/src/clipper_lib_z.rs).
// Not called by the engine: tests/clipper_z_native_parity.rs compares the port
// with the library through them.
//
// The library of this build does not survive every input. Two ways it dies are
// known: Clipper::ProcessHorizontal (clipper.cpp:2227-2246) dereferences an
// iterator it never set when a run of horizontals turns round, and an edge of no
// length (one place, two Z) is such a turn; and GetBottomPt (clipper.cpp:544-579)
// never leaves `while (dups->Pt != pp->Pt)` once pp is a point that shares its
// place, but not its Z, with the point the outer loop waits for. So the exports
// below run the library under cz_ref_guard: a SIGSEGV or SIGBUS of the calling
// thread, or one second without a result, ends the call with a jump out of the
// library (whatever it had allocated stays allocated), and the export reports it in
// *died instead of returning a result. One guarded call runs at a time.
// ---------------------------------------------------------------------------
#include <csetjmp>
#include <csignal>
#include <mutex>
#include <pthread.h>
#include <sys/time.h>

namespace {
std::mutex            cz_ref_guard_mutex;
sigjmp_buf            cz_ref_guard_jmp;
volatile sig_atomic_t cz_ref_guard_on = 0;
pthread_t             cz_ref_guard_thread;

void cz_ref_guard_signal(int sig)
{
    if (cz_ref_guard_on && !pthread_equal(pthread_self(), cz_ref_guard_thread)) {
        if (sig == SIGALRM) {
            // The timer's signal goes to any thread: pass it on to the guarded one.
            pthread_kill(cz_ref_guard_thread, SIGALRM);
            return;
        }
    } else if (cz_ref_guard_on) {
        siglongjmp(cz_ref_guard_jmp, sig);
    }
    // Not a signal of the guarded call: let it take its default course.
    signal(sig, SIG_DFL);
}

// Runs f. Returns 0 when it returned, the number of the signal that ended it otherwise.
template <typename F> int cz_ref_guard(F &&f)
{
    std::lock_guard<std::mutex> lock(cz_ref_guard_mutex);
    struct sigaction action, old_alrm, old_segv, old_bus;
    action.sa_handler = cz_ref_guard_signal;
    sigemptyset(&action.sa_mask);
    action.sa_flags = 0;
    sigaction(SIGALRM, &action, &old_alrm);
    sigaction(SIGSEGV, &action, &old_segv);
    sigaction(SIGBUS, &action, &old_bus);
    cz_ref_guard_thread = pthread_self();
    struct itimerval one_second = { { 0, 0 }, { 1, 0 } }, off = { { 0, 0 }, { 0, 0 } };
    int died = sigsetjmp(cz_ref_guard_jmp, 1);
    if (died == 0) {
        cz_ref_guard_on = 1;
        setitimer(ITIMER_REAL, &one_second, nullptr);
        f();
    }
    cz_ref_guard_on = 0;
    setitimer(ITIMER_REAL, &off, nullptr);
    sigaction(SIGALRM, &old_alrm, nullptr);
    sigaction(SIGSEGV, &old_segv, nullptr);
    sigaction(SIGBUS, &old_bus, nullptr);
    return died;
}

ClipperLib_Z::Paths cz_ref_read_zpaths(const int32_t *xyz, const int32_t *lens, int32_t num)
{
    ClipperLib_Z::Paths paths;
    if (num <= 0 || xyz == nullptr || lens == nullptr)
        return paths;
    paths.reserve(num);
    const int32_t *cursor = xyz;
    for (int32_t p = 0; p < num; ++p) {
        int32_t len = lens[p];
        ClipperLib_Z::Path path;
        path.reserve(len);
        for (int32_t i = 0; i < len; ++i)
            path.emplace_back(cursor[3 * i], cursor[3 * i + 1], cursor[3 * i + 2]);
        cursor += 3 * len;
        paths.emplace_back(std::move(path));
    }
    return paths;
}

void cz_ref_list_znode(const ClipperLib_Z::PolyNode &node, int32_t parent, ClipperLib_Z::Paths &contours,
                       std::vector<int32_t> &info)
{
    for (int i = 0; i < node.ChildCount(); ++i) {
        const ClipperLib_Z::PolyNode &child = *node.Childs[i];
        int32_t me = int32_t(contours.size());
        contours.push_back(child.Contour);
        info.push_back(parent);
        info.push_back(child.IsHole() ? 1 : 0);
        info.push_back(child.IsOpen() ? 1 : 0);
        info.push_back(child.ChildCount());
        cz_ref_list_znode(child, me, contours, info);
    }
}

CzZPaths cz_ref_marshal_zpolytree(const ClipperLib_Z::PolyTree &tree, int32_t *node_info, int32_t node_info_cap,
                                  int32_t *num_nodes, int32_t *total)
{
    ClipperLib_Z::Paths contours;
    std::vector<int32_t> info;
    cz_ref_list_znode(tree, -1, contours, info);
    if (node_info != nullptr)
        for (size_t i = 0; i < info.size() && i < size_t(node_info_cap) * 4; ++i)
            node_info[i] = info[i];
    if (num_nodes != nullptr)
        *num_nodes = int32_t(contours.size());
    if (total != nullptr)
        *total = contours.empty() && tree.ChildCount() == 0 ? 0 : tree.Total();
    return marshal_zpaths(contours);
}

// The Z-fill callback of the reference. One round per value v, in this order:
//   h = (h ^ uint32(v)) * 0x01000193, starting from h = 0x9E3779B9, over
//   e1bot.x, e1bot.y, e1bot.z, e1top.x, e1top.y, e1top.z,
//   e2bot.x, e2bot.y, e2bot.z, e2top.x, e2top.y, e2top.z, pt.x, pt.y;
// then h ^= h >> 15, and pt.z = 0 when h % 5 == 0, int32(h) otherwise. Every
// argument and its position counts, and a result of 0 leaves the point to be
// filled again by the next intersection at it.
void cz_ref_z_fill(const ClipperLib_Z::IntPoint &e1bot, const ClipperLib_Z::IntPoint &e1top,
                   const ClipperLib_Z::IntPoint &e2bot, const ClipperLib_Z::IntPoint &e2top,
                   ClipperLib_Z::IntPoint &pt)
{
    uint32_t h = 0x9E3779B9u;
    auto mix = [&h](int32_t v) { h = (h ^ uint32_t(v)) * 0x01000193u; };
    for (const ClipperLib_Z::IntPoint *p : { &e1bot, &e1top, &e2bot, &e2top }) {
        mix(p->x());
        mix(p->y());
        mix(p->z());
    }
    mix(pt.x());
    mix(pt.y());
    h ^= h >> 15;
    pt.z() = (h % 5u == 0u) ? 0 : int32_t(h);
}
} // namespace

// ClipperLib_Z::Clipper. The paths are flat (x, y, z) triples. Subject i is
// closed when subj_closed[i] != 0, open otherwise; clips are closed. add_paths
// == 0: every path through AddPath, subjects first; otherwise three AddPaths
// calls (closed subjects, open subjects, clips). z_fill: 0 = no ZFillFunction
// (an intersection gets Z = 0), 1 = cz_ref_z_fill above. output:
//   0: Execute into a PolyTree, returned in the encoding of
//      cz_ref_execute_polytree, every point with its Z;
//   1: the same Execute, then PolyTreeToPaths;
//   2: Execute into a Paths (only without open subjects: Execute throws on them).
// *succeeded receives what Execute returned. *died receives 0, or the signal that
// ended the library (see cz_ref_guard above; nothing is returned then). Free via
// cz_free_zpaths.
extern "C" CzZPaths cz_ref_execute_z(const int32_t *subj_xyz, const int32_t *subj_lens,
                                     const int32_t *subj_closed, int32_t subj_num,
                                     const int32_t *clip_xyz, const int32_t *clip_lens, int32_t clip_num,
                                     int32_t clip_type, int32_t subj_fill, int32_t clip_fill,
                                     int32_t strictly_simple, int32_t preserve_collinear,
                                     int32_t reverse_solution, int32_t add_paths, int32_t z_fill,
                                     int32_t output, int32_t *node_info, int32_t node_info_cap,
                                     int32_t *num_nodes, int32_t *total, int32_t *succeeded,
                                     int32_t *died)
{
    CzZPaths result = { nullptr, nullptr, 0, 0 };
    *died = cz_ref_guard([&] {
        result = [&]() -> CzZPaths {
    ClipperLib_Z::Paths subject = cz_ref_read_zpaths(subj_xyz, subj_lens, subj_num);
    ClipperLib_Z::Paths clip    = cz_ref_read_zpaths(clip_xyz, clip_lens, clip_num);
    ClipperLib_Z::Clipper c;
    c.StrictlySimple(strictly_simple != 0);
    c.PreserveCollinear(preserve_collinear != 0);
    c.ReverseSolution(reverse_solution != 0);
    if (z_fill != 0)
        c.ZFillFunction(cz_ref_z_fill);
    if (add_paths == 0) {
        for (size_t i = 0; i < subject.size(); ++i)
            c.AddPath(subject[i], ClipperLib_Z::ptSubject, subj_closed[i] != 0);
        for (const ClipperLib_Z::Path &p : clip)
            c.AddPath(p, ClipperLib_Z::ptClip, true);
    } else {
        ClipperLib_Z::Paths closed, open;
        for (size_t i = 0; i < subject.size(); ++i)
            (subj_closed[i] != 0 ? closed : open).push_back(subject[i]);
        c.AddPaths(closed, ClipperLib_Z::ptSubject, true);
        c.AddPaths(open, ClipperLib_Z::ptSubject, false);
        c.AddPaths(clip, ClipperLib_Z::ptClip, true);
    }
    ClipperLib_Z::ClipType     ct = static_cast<ClipperLib_Z::ClipType>(clip_type);
    ClipperLib_Z::PolyFillType sf = static_cast<ClipperLib_Z::PolyFillType>(subj_fill);
    ClipperLib_Z::PolyFillType cf = static_cast<ClipperLib_Z::PolyFillType>(clip_fill);
    if (output == 2) {
        ClipperLib_Z::Paths solution;
        bool ok = c.Execute(ct, solution, sf, cf);
        if (succeeded != nullptr)
            *succeeded = ok ? 1 : 0;
        return marshal_zpaths(solution);
    }
    ClipperLib_Z::PolyTree tree;
    bool ok = c.Execute(ct, tree, sf, cf);
    if (succeeded != nullptr)
        *succeeded = ok ? 1 : 0;
    if (output == 0)
        return cz_ref_marshal_zpolytree(tree, node_info, node_info_cap, num_nodes, total);
    ClipperLib_Z::Paths paths;
    ClipperLib_Z::PolyTreeToPaths(tree, paths);
    return marshal_zpaths(paths);
        }();
    });
    return result;
}

// ClipperLib_Z::ClipperOffset(miter_limit, arc_tolerance, shortest_edge_length),
// then Clear() (see below); the rest as cz_ref_offset, with (x, y, z) triples in
// and out. Free via cz_free_zpaths.
extern "C" CzZPaths cz_ref_offset_z(const int32_t *xyz, const int32_t *lens, const int32_t *join_types,
                                    const int32_t *end_types, int32_t num, double miter_limit,
                                    double arc_tolerance, double shortest_edge_length, double delta,
                                    int32_t use_polytree, int32_t *node_info, int32_t node_info_cap,
                                    int32_t *num_nodes, int32_t *total)
{
    ClipperLib_Z::Paths paths = cz_ref_read_zpaths(xyz, lens, num);
    ClipperLib_Z::ClipperOffset co(miter_limit, arc_tolerance, shortest_edge_length);
    // The constructor (clipper.hpp:509-510) initializes m_lowest with `m_lowest(-1, 0)`.
    // For the vector of three of this build Eigen takes the two integers as a number of
    // rows and columns, not as coefficients, and leaves m_lowest as it was: not
    // initialized (a build with assertions stops there). Clear() sets m_lowest.x() to
    // -1, which is all AddPath and FixOrientations read before they assign it.
    co.Clear();
    for (size_t i = 0; i < paths.size(); ++i)
        co.AddPath(paths[i], static_cast<ClipperLib_Z::JoinType>(join_types[i]),
                   static_cast<ClipperLib_Z::EndType>(end_types[i]));
    if (use_polytree == 0) {
        ClipperLib_Z::Paths solution;
        co.Execute(solution, delta);
        return marshal_zpaths(solution);
    }
    ClipperLib_Z::PolyTree tree;
    co.Execute(tree, delta);
    return cz_ref_marshal_zpolytree(tree, node_info, node_info_cap, num_nodes, total);
}

// cz_clip_extrusion and cz_detect_floating under cz_ref_guard, for
// tests/clipper_z_engine_native_parity.rs: the same arguments and results, and
// *died as in cz_ref_execute_z.
extern "C" CzZPaths cz_ref_clip_extrusion_guarded(const int32_t *subject_xyz, int32_t subject_n,
                                                  const int32_t *clip_xyz, const int32_t *clip_lens,
                                                  int32_t clip_num, int32_t clip_type, int32_t *died)
{
    CzZPaths result = { nullptr, nullptr, 0, 0 };
    *died = cz_ref_guard([&] {
        result = cz_clip_extrusion(subject_xyz, subject_n, clip_xyz, clip_lens, clip_num, clip_type);
    });
    return result;
}

extern "C" CzZPaths cz_ref_detect_floating_guarded(const int32_t *subject_xyz, int32_t subject_n,
                                                   const int32_t *clip_xyz, const int32_t *clip_lens,
                                                   int32_t clip_num, int32_t subject_idx_range,
                                                   int32_t *out_num_diff_paths, int32_t *died)
{
    CzZPaths result = { nullptr, nullptr, 0, 0 };
    *died = cz_ref_guard([&] {
        result = cz_detect_floating(subject_xyz, subject_n, clip_xyz, clip_lens, clip_num, subject_idx_range,
                                    out_num_diff_paths);
    });
    return result;
}

// ===========================================================================
// R910 — ClipperUtils.cpp at the pin, for the tests of the GEO_CLIPPER_CLIB arms
// of clipper_utils.rs; called by tests only.
//
// The functions of namespace cu_pin are the text of
// references/BambuStudio/src/libslic3r/ClipperUtils.cpp (the lines are cited at
// each), compiled on the ClipperLib of this shim. What differs from the pin is
// only what it takes from the rest of libslic3r: Polygon, Polyline and ExPolygon
// are reduced to their points (a ClipperLib::Path, which is what Points is in the
// pin's build), the path providers (ClipperUtils.hpp) to the ClipperLib::Paths
// they yield, and `append` and `to_polygons` are copied from libslic3r.h:139-150
// and Polygon.hpp:276-283.
// ===========================================================================

#include <cassert>
#include <iterator>
#include <utility>

namespace cu_pin {

// ClipperUtils.hpp:29-44
static constexpr const float                ClipperSafetyOffset             = 10.f;
static constexpr const ClipperLib::JoinType DefaultJoinType                 = ClipperLib::jtMiter;
static constexpr const double               DefaultMiterLimit               = 3.;
static constexpr const double               ClipperOffsetShortestEdgeFactor = 0.005;
using ClipperLib::jtRound;

// ClipperUtils.hpp:46-49
enum class ApplySafetyOffset {
    No,
    Yes
};

struct Polygon   { ClipperLib::Path points; };
using  Polygons  = std::vector<Polygon>;
struct Polyline  { ClipperLib::Path points; };
struct ExPolygon { Polygon contour; Polygons holes; };
using  ExPolygons = std::vector<ExPolygon>;

// libslic3r.h:139-150
template <typename T>
inline void append(std::vector<T>& dest, std::vector<T>&& src)
{
    if (dest.empty())
        dest = std::move(src);
    else {
        dest.reserve(dest.size() + src.size());
        std::move(std::begin(src), std::end(src), std::back_inserter(dest));
    }
    src.clear();
    src.shrink_to_fit();
}

// Polygon.hpp:276-283
inline Polygons to_polygons(ClipperLib::Paths &&paths)
{
    Polygons out;
    out.reserve(paths.size());
    for (ClipperLib::Path &path : paths)
        out.push_back(Polygon{ std::move(path) });
    return out;
}

// ClipperUtils::PolygonsProvider, SinglePathProvider, ExPolygonsProvider: the paths
// they iterate over, in their order (a contour, then its holes).
inline ClipperLib::Paths PolygonsProvider(const Polygons &polygons)
{
    ClipperLib::Paths out;
    for (const Polygon &p : polygons)
        out.push_back(p.points);
    return out;
}
inline ClipperLib::Paths SinglePathProvider(const ClipperLib::Path &points) { return ClipperLib::Paths{ points }; }
inline ClipperLib::Paths ExPolygonsProvider(const ExPolygons &expolygons)
{
    ClipperLib::Paths out;
    for (const ExPolygon &expoly : expolygons) {
        out.push_back(expoly.contour.points);
        for (const Polygon &hole : expoly.holes)
            out.push_back(hole.points);
    }
    return out;
}
inline ClipperLib::Paths EmptyPathsProvider() { return ClipperLib::Paths(); }

// ClipperUtils.cpp:175-210
static ExPolygons PolyTreeToExPolygons(ClipperLib::PolyTree &&polytree)
{
    struct Inner {
        static void PolyTreeToExPolygonsRecursive(ClipperLib::PolyNode &&polynode, ExPolygons *expolygons)
        {
            size_t cnt = expolygons->size();
            expolygons->resize(cnt + 1);
            (*expolygons)[cnt].contour.points = std::move(polynode.Contour);
            (*expolygons)[cnt].holes.resize(polynode.ChildCount());
            for (int i = 0; i < polynode.ChildCount(); ++ i) {
                (*expolygons)[cnt].holes[i].points = std::move(polynode.Childs[i]->Contour);
                // Add outer polygons contained by (nested within) holes.
                for (int j = 0; j < polynode.Childs[i]->ChildCount(); ++ j)
                    PolyTreeToExPolygonsRecursive(std::move(*polynode.Childs[i]->Childs[j]), expolygons);
            }
        }

        static size_t PolyTreeCountExPolygons(const ClipperLib::PolyNode &polynode)
        {
            size_t cnt = 1;
            for (int i = 0; i < polynode.ChildCount(); ++ i) {
                for (int j = 0; j < polynode.Childs[i]->ChildCount(); ++ j)
                cnt += PolyTreeCountExPolygons(*polynode.Childs[i]->Childs[j]);
            }
            return cnt;
        }
    };

    ExPolygons retval;
    size_t cnt = 0;
    for (int i = 0; i < polytree.ChildCount(); ++ i)
        cnt += Inner::PolyTreeCountExPolygons(*polytree.Childs[i]);
    retval.reserve(cnt);
    for (int i = 0; i < polytree.ChildCount(); ++ i)
        Inner::PolyTreeToExPolygonsRecursive(std::move(*polytree.Childs[i]), &retval);
    return retval;
}

// ClipperUtils.cpp:270-300
// Offset CCW contours outside, CW contours (holes) inside.
// Don't calculate union of the output paths.
static ClipperLib::Paths raw_offset(const ClipperLib::Paths &paths, float offset, ClipperLib::JoinType joinType, double miterLimit, ClipperLib::EndType endType = ClipperLib::etClosedPolygon)
{
    ClipperLib::ClipperOffset co;
    ClipperLib::Paths out;
    out.reserve(paths.size());
    ClipperLib::Paths out_this;
    if (joinType == jtRound)
        co.ArcTolerance = miterLimit;
    else
        co.MiterLimit = miterLimit;
    co.ShortestEdgeLength = std::abs(offset * ClipperOffsetShortestEdgeFactor);
    for (const ClipperLib::Path &path : paths) {
        co.Clear();
        // Execute reorients the contours so that the outer most contour has a positive area. Thus the output
        // contours will be CCW oriented even though the input paths are CW oriented.
        // Offset is applied after contour reorientation, thus the signum of the offset value is reversed.
        co.AddPath(path, joinType, endType);
        bool ccw = endType == ClipperLib::etClosedPolygon ? ClipperLib::Orientation(path) : true;
        co.Execute(out_this, ccw ? offset : - offset);
        if (! ccw) {
            // Reverse the resulting contours.
            for (ClipperLib::Path &path : out_this)
                std::reverse(path.begin(), path.end());
        }
        append(out, std::move(out_this));
    }
    return out;
}

// ClipperUtils.cpp:302-307
// Offset outside by 10um, one by one.
static ClipperLib::Paths safety_offset(const ClipperLib::Paths &paths)
{
    return raw_offset(paths, ClipperSafetyOffset, DefaultJoinType, DefaultMiterLimit);
}

// ClipperUtils.cpp:309-322
template<class TResult>
TResult clipper_do(
    const ClipperLib::ClipType     clipType,
    const ClipperLib::Paths &      subject,
    const ClipperLib::Paths &      clip,
    const ClipperLib::PolyFillType fillType)
{
    ClipperLib::Clipper clipper;
    clipper.AddPaths(subject, ClipperLib::ptSubject, true);
    clipper.AddPaths(clip,    ClipperLib::ptClip,    true);
    TResult retval;
    clipper.Execute(clipType, retval, fillType, fillType);
    return retval;
}

// ClipperUtils.cpp:339-350
template<class TResult>
TResult clipper_union(
    const ClipperLib::Paths &      subject,
    // fillType pftNonZero and pftPositive "should" produce the same result for "normalized with implicit union" set of polygons
    const ClipperLib::PolyFillType fillType = ClipperLib::pftNonZero)
{
    ClipperLib::Clipper clipper;
    clipper.AddPaths(subject, ClipperLib::ptSubject, true);
    TResult retval;
    clipper.Execute(ClipperLib::ctUnion, retval, fillType, fillType);
    return retval;
}

// ClipperUtils.cpp:352-357
// Perform union of input polygons using the positive rule, convert to ExPolygons.
//FIXME is there any benefit of not doing the boolean / using pftEvenOdd?
// ClipperUtils.hpp:337: do_union = false.
static ExPolygons ClipperPaths_to_Slic3rExPolygons(const ClipperLib::Paths &input, bool do_union = false)
{
    return PolyTreeToExPolygons(clipper_union<ClipperLib::PolyTree>(input, do_union ? ClipperLib::pftNonZero : ClipperLib::pftEvenOdd));
}

// ClipperUtils.cpp:359-364
static ClipperLib::Paths raw_offset_polyline(const ClipperLib::Paths &paths, float offset, ClipperLib::JoinType joinType, double miterLimit, ClipperLib::EndType end_type = ClipperLib::etOpenButt)
{
    assert(offset > 0);
    return raw_offset(paths, offset, joinType, miterLimit, end_type);
}

// ClipperUtils.cpp:366-372
template<class TResult>
static TResult expand_paths(const ClipperLib::Paths &paths, float offset, ClipperLib::JoinType joinType, double miterLimit)
{
    // BBS
    //assert(offset > 0);
    return clipper_union<TResult>(raw_offset(paths, offset, joinType, miterLimit));
}

// ClipperUtils.cpp:374-379
// used by shrink_paths()
inline void remove_outermost_polygon(ClipperLib::Paths &solution)
    { if (! solution.empty()) solution.erase(solution.begin()); }
inline void remove_outermost_polygon(ClipperLib::PolyTree &solution)
    { solution.RemoveOutermostPolygon(); }

// ClipperUtils.cpp:381-397
template<class TResult>
static TResult shrink_paths(const ClipperLib::Paths &paths, float offset, ClipperLib::JoinType joinType, double miterLimit)
{
    // BBS
    //assert(offset > 0);
    TResult out;
    if (auto raw = raw_offset(paths, - offset, joinType, miterLimit); ! raw.empty()) {
        ClipperLib::Clipper clipper;
        clipper.AddPaths(raw, ClipperLib::ptSubject, true);
        ClipperLib::IntRect r = clipper.GetBounds();
        clipper.AddPath({ { r.left - 10, r.bottom + 10 }, { r.right + 10, r.bottom + 10 }, { r.right + 10, r.top - 10 }, { r.left - 10, r.top - 10 } }, ClipperLib::ptSubject, true);
        clipper.ReverseSolution(true);
        clipper.Execute(ClipperLib::ctUnion, out, ClipperLib::pftNegative, ClipperLib::pftNegative);
        remove_outermost_polygon(out);
    }
    return out;
}

// ClipperUtils.cpp:399-408
template<class TResult>
static TResult offset_paths(const ClipperLib::Paths &paths, float offset, ClipperLib::JoinType joinType, double miterLimit)
{
    // BBS
    //assert(offset != 0);

    return offset > 0 ?
        expand_paths<TResult>(paths,   offset, joinType, miterLimit) :
        shrink_paths<TResult>(paths, - offset, joinType, miterLimit);
}

// ClipperUtils.cpp:415-416
ExPolygons offset_ex(const Polygons &polygons, const float delta, ClipperLib::JoinType joinType, double miterLimit)
    { return PolyTreeToExPolygons(offset_paths<ClipperLib::PolyTree>(PolygonsProvider(polygons), delta, joinType, miterLimit)); }

// ClipperUtils.cpp:418-419
Polygons offset(const Polyline &polyline, const float delta, ClipperLib::JoinType joinType, double miterLimit, ClipperLib::EndType end_type)
    { assert(delta > 0); return to_polygons(clipper_union<ClipperLib::Paths>(raw_offset_polyline(SinglePathProvider(polyline.points), delta, joinType, miterLimit, end_type))); }

// ClipperUtils.cpp:436-506
    // returns number of expolygons collected (0 or 1).
static int offset_expolygon_inner(const ExPolygon &expoly, const float delta, ClipperLib::JoinType joinType, double miterLimit, ClipperLib::Paths &out)
{
    // 1) Offset the outer contour.
    ClipperLib::Paths contours;
    {
        ClipperLib::ClipperOffset co;
        if (joinType == jtRound)
            co.ArcTolerance = miterLimit;
        else
            co.MiterLimit = miterLimit;
        co.ShortestEdgeLength = double(std::abs(delta * ClipperOffsetShortestEdgeFactor));
        co.AddPath(expoly.contour.points, joinType, ClipperLib::etClosedPolygon);
        co.Execute(contours, delta);
    }
    if (contours.empty())
        // No need to try to offset the holes.
        return 0;

    if (expoly.holes.empty()) {
        // No need to subtract holes from the offsetted expolygon, we are done.
        append(out, std::move(contours));
    } else {
        // 2) Offset the holes one by one, collect the offsetted holes.
        ClipperLib::Paths holes;
        {
            for (const Polygon &hole : expoly.holes) {
                ClipperLib::ClipperOffset co;
                if (joinType == jtRound)
                    co.ArcTolerance = miterLimit;
                else
                    co.MiterLimit = miterLimit;
                co.ShortestEdgeLength = double(std::abs(delta * ClipperOffsetShortestEdgeFactor));
                co.AddPath(hole.points, joinType, ClipperLib::etClosedPolygon);
                ClipperLib::Paths out2;
                // Execute reorients the contours so that the outer most contour has a positive area. Thus the output
                // contours will be CCW oriented even though the input paths are CW oriented.
                // Offset is applied after contour reorientation, thus the signum of the offset value is reversed.
                co.Execute(out2, - delta);
                append(holes, std::move(out2));
            }
        }

        // 3) Subtract holes from the contours.
        if (holes.empty()) {
            // No hole remaining after an offset. Just copy the outer contour.
            append(out, std::move(contours));
        } else if (delta < 0) {
            // Negative offset. There is a chance, that the offsetted hole intersects the outer contour.
            // Subtract the offsetted holes from the offsetted contours.
            if (auto output = clipper_do<ClipperLib::Paths>(ClipperLib::ctDifference, contours, holes, ClipperLib::pftNonZero); ! output.empty()) {
                append(out, std::move(output));
            } else {
                // The offsetted holes have eaten up the offsetted outer contour.
                return 0;
            }
        } else {
            // Positive offset. As long as the Clipper offset does what one expects it to do, the offsetted hole will have a smaller
            // area than the original hole or even disappear, therefore there will be no new intersections.
            // Just collect the reversed holes.
            out.reserve(contours.size() + holes.size());
            append(out, std::move(contours));
            // Reverse the holes in place.
            for (size_t i = 0; i < holes.size(); ++ i)
                std::reverse(holes[i].begin(), holes[i].end());
            append(out, std::move(holes));
        }
    }

    return 1;
}

// ClipperUtils.cpp:513-518
ClipperLib::Paths expolygon_offset(const ExPolygon &expolygon, const float delta, ClipperLib::JoinType joinType, double miterLimit)
{
    ClipperLib::Paths out;
    offset_expolygon_inner(expolygon, delta, joinType, miterLimit, out);
    return out;
}

// ClipperUtils.cpp:520-535
// This is a safe variant of the polygons offset, tailored for multiple ExPolygons.
// It is required, that the input expolygons do not overlap and that the holes of each ExPolygon don't intersect with their respective outer contours.
// Each ExPolygon is offsetted separately. For outer offset, the the offsetted ExPolygons shall be united outside of this function.
static std::pair<ClipperLib::Paths, size_t> expolygons_offset_raw(const ExPolygons &expolygons, const float delta, ClipperLib::JoinType joinType, double miterLimit)
{
    // Offsetted ExPolygons before they are united.
    ClipperLib::Paths output;
    output.reserve(expolygons.size());
    // How many non-empty offsetted expolygons were actually collected into output?
    // If only one, then there is no need to do a final union.
    size_t expolygons_collected = 0;
    for (const auto &expoly : expolygons)
        expolygons_collected += offset_expolygon_inner(expoly, delta, joinType, miterLimit, output);
    return std::make_pair(std::move(output), expolygons_collected);
}

// ClipperUtils.cpp:550-557
// See comment on expolygons_offset_raw. In addition, the polygons are always united to conver to polytree.
static ClipperLib::PolyTree expolygons_offset_pt(const ExPolygons &expolygons, const float delta, ClipperLib::JoinType joinType, double miterLimit)
{
    auto [output, expolygons_collected] = expolygons_offset_raw(expolygons, delta, joinType, miterLimit);
    // Unite the offsetted expolygons for both the
    return clipper_union<ClipperLib::PolyTree>(output);
}

// ClipperUtils.cpp:567-569
ExPolygons offset_ex(const ExPolygon &expolygon, const float delta, ClipperLib::JoinType joinType, double miterLimit)
    //FIXME one may spare one Clipper Union call.
    { return ClipperPaths_to_Slic3rExPolygons(expolygon_offset(expolygon, delta, joinType, miterLimit)); }
// ClipperUtils.cpp:570-571
ExPolygons offset_ex(const ExPolygons &expolygons, const float delta, ClipperLib::JoinType joinType, double miterLimit)
    { return PolyTreeToExPolygons(expolygons_offset_pt(expolygons, delta, joinType, miterLimit)); }

// ClipperUtils.cpp:641-654
inline ClipperLib::PolyTree clipper_do_polytree(
    const ClipperLib::ClipType       clipType,
    const ClipperLib::Paths &        subject,
    const ClipperLib::Paths &        clip,
    const ClipperLib::PolyFillType   fillType)
{
    // Perform the operation with the output to input_subject.
    // This pass does not generate a PolyTree, which is a very expensive operation with the current Clipper library
    // if there are overapping edges.
    if (auto output = clipper_do<ClipperLib::Paths>(clipType, subject, clip, fillType); ! output.empty())
        // Perform an additional Union operation to generate the PolyTree ordering.
        return clipper_union<ClipperLib::PolyTree>(output, fillType);
    return ClipperLib::PolyTree();
}
// ClipperUtils.cpp:655-667
inline ClipperLib::PolyTree clipper_do_polytree(
    const ClipperLib::ClipType       clipType,
    const ClipperLib::Paths &        subject,
    const ClipperLib::Paths &        clip,
    const ClipperLib::PolyFillType   fillType,
    const ApplySafetyOffset          do_safety_offset)
{
    assert(do_safety_offset == ApplySafetyOffset::No || clipType != ClipperLib::ctUnion);
    return do_safety_offset == ApplySafetyOffset::Yes ?
        clipper_do_polytree(clipType, subject, safety_offset(clip), fillType) :
        clipper_do_polytree(clipType, subject, clip, fillType);
}

// ClipperUtils.cpp:737-739
static ExPolygons _clipper_ex(ClipperLib::ClipType clipType, const ClipperLib::Paths &subject, const ClipperLib::Paths &clip, ApplySafetyOffset do_safety_offset, ClipperLib::PolyFillType fill_type = ClipperLib::pftNonZero)
    { return PolyTreeToExPolygons(clipper_do_polytree(clipType, subject, clip, fill_type, do_safety_offset)); }

// ClipperUtils.cpp:755-756
ExPolygons diff_ex(const ExPolygons &subject, const ExPolygons &clip, ApplySafetyOffset do_safety_offset = ApplySafetyOffset::No)
    { return _clipper_ex(ClipperLib::ctDifference, ExPolygonsProvider(subject), ExPolygonsProvider(clip), do_safety_offset); }
// ClipperUtils.cpp:802-803
ExPolygons intersection_ex(const ExPolygons &subject, const ExPolygons &clip, ApplySafetyOffset do_safety_offset = ApplySafetyOffset::No)
    { return _clipper_ex(ClipperLib::ctIntersection, ExPolygonsProvider(subject), ExPolygonsProvider(clip), do_safety_offset); }
// ClipperUtils.cpp:815-816
ExPolygons union_ex(const ExPolygons &subject)
    { return PolyTreeToExPolygons(clipper_do_polytree(ClipperLib::ctUnion, ExPolygonsProvider(subject), EmptyPathsProvider(), ClipperLib::pftNonZero)); }
// ClipperUtils.cpp:824-831
// BBS
ExPolygons union_ex(const ExPolygons& poly1, const ExPolygons& poly2, bool safety_offset_ = false)
    {
    ExPolygons expolys = poly1;
    for (const ExPolygon& expoly : poly2)
        expolys.push_back(expoly);
    return union_ex(expolys);
    }
// ClipperUtils.cpp:836-838
ExPolygons xor_ex(const ExPolygons &subject, const ExPolygons &clip, ApplySafetyOffset do_safety_offset = ApplySafetyOffset::No) {
    return _clipper_ex(ClipperLib::ctXor, ExPolygonsProvider(subject), ExPolygonsProvider(clip), do_safety_offset);
}

// The expolygons of the flat layout of cz_offset2_ex: a path with is_hole == 0
// starts an expolygon, the others are holes of the expolygon before them.
ExPolygons read_expolygons(const int32_t *xy, const int32_t *lens, const int32_t *is_hole, int32_t num)
{
    ExPolygons out;
    ClipperLib::Paths paths = read_closed_paths(xy, lens, num);
    for (int32_t i = 0; i < (int32_t) paths.size(); ++ i) {
        if (is_hole[i] == 0)
            out.push_back(ExPolygon{ Polygon{ std::move(paths[i]) }, Polygons() });
        else if (! out.empty())
            out.back().holes.push_back(Polygon{ std::move(paths[i]) });
    }
    return out;
}

// Expolygons in the grouped layout of cz_union_ex (z = 0: a contour, z = 1: a hole).
CzZPaths marshal_expolygons(const ExPolygons &expolygons)
{
    ClipperLib::Paths paths;
    std::vector<int32_t> is_hole;
    for (const ExPolygon &expoly : expolygons) {
        paths.push_back(expoly.contour.points);
        is_hole.push_back(0);
        for (const Polygon &hole : expoly.holes) {
            paths.push_back(hole.points);
            is_hole.push_back(1);
        }
    }
    return marshal_grouped(paths, is_hole);
}

ClipperLib::JoinType join_type_of(int32_t join_type)
{
    return join_type == 1 ? ClipperLib::jtRound : join_type == 2 ? ClipperLib::jtSquare : ClipperLib::jtMiter;
}

} // namespace cu_pin

extern "C" CzZPaths cz_ref_cu_offset_ex_polygons(const int32_t *xy, const int32_t *lens, int32_t num,
                                                 double delta, int32_t join_type, double miter_limit)
{
    cu_pin::Polygons polygons = cu_pin::to_polygons(read_closed_paths(xy, lens, num));
    return cu_pin::marshal_expolygons(cu_pin::offset_ex(polygons, delta, cu_pin::join_type_of(join_type), miter_limit));
}

extern "C" CzZPaths cz_ref_cu_offset_ex_expolygons(const int32_t *xy, const int32_t *lens, const int32_t *is_hole,
                                                   int32_t num, double delta, int32_t join_type,
                                                   double miter_limit, int32_t which)
{
    cu_pin::ExPolygons expolygons = cu_pin::read_expolygons(xy, lens, is_hole, num);
    if (which == 0) {
        if (expolygons.empty())
            return cu_pin::marshal_expolygons(cu_pin::ExPolygons());
        return cu_pin::marshal_expolygons(cu_pin::offset_ex(expolygons.front(), delta, cu_pin::join_type_of(join_type), miter_limit));
    }
    return cu_pin::marshal_expolygons(cu_pin::offset_ex(expolygons, delta, cu_pin::join_type_of(join_type), miter_limit));
}

extern "C" CzZPaths cz_ref_cu_offset_polyline(const int32_t *xy, int32_t n, double delta, int32_t join_type,
                                              double miter_limit, int32_t end_type)
{
    cu_pin::Polyline polyline;
    for (int32_t i = 0; i < n; ++ i)
        polyline.points.emplace_back(xy[2 * i], xy[2 * i + 1]);
    cu_pin::Polygons out = cu_pin::offset(polyline, delta, cu_pin::join_type_of(join_type), miter_limit,
                                          static_cast<ClipperLib::EndType>(end_type));
    ClipperLib::Paths paths;
    for (cu_pin::Polygon &p : out)
        paths.push_back(std::move(p.points));
    return marshal_paths(paths);
}

extern "C" CzZPaths cz_ref_cu_boolean_ex(const int32_t *subj_xy, const int32_t *subj_lens, const int32_t *subj_is_hole,
                                         int32_t subj_num, const int32_t *clip_xy, const int32_t *clip_lens,
                                         const int32_t *clip_is_hole, int32_t clip_num, int32_t op)
{
    cu_pin::ExPolygons subject = cu_pin::read_expolygons(subj_xy, subj_lens, subj_is_hole, subj_num);
    cu_pin::ExPolygons clip    = cu_pin::read_expolygons(clip_xy, clip_lens, clip_is_hole, clip_num);
    switch (op) {
    case 0:  return cu_pin::marshal_expolygons(cu_pin::intersection_ex(subject, clip));
    case 1:  return cu_pin::marshal_expolygons(cu_pin::union_ex(subject, clip));
    case 2:  return cu_pin::marshal_expolygons(cu_pin::diff_ex(subject, clip));
    default: return cu_pin::marshal_expolygons(cu_pin::xor_ex(subject, clip));
    }
}
