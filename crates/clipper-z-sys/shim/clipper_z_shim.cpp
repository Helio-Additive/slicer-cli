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

extern "C" void cz_free_zpaths(CzZPaths paths) {
    std::free(paths.coords);
    std::free(paths.path_lens);
}
