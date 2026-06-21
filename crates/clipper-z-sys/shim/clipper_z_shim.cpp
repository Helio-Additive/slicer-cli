// Implementation of the C ABI shim. See clipper_z_shim.h.
//
// IMPORTANT include ordering: clipper_z.hpp must be included BEFORE clipper.hpp
// (it #errors otherwise). clipper_z.hpp #defines CLIPPERLIB_USE_XYZ, includes
// clipper.hpp into namespace ClipperLib_Z, then #undefs clipper_hpp so a second
// include of clipper.hpp below pulls in the non-XYZ namespace ClipperLib.
#include "clipper_z.hpp"   // -> namespace ClipperLib_Z (XYZ / 3D IntPoint)
#include "clipper.hpp"     // -> namespace ClipperLib   (2D IntPoint)

#include "clipper_z_shim.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <utility>
#include <vector>

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

extern "C" void cz_free_zpaths(CzZPaths paths) {
    std::free(paths.coords);
    std::free(paths.path_lens);
}

// ---------------------------------------------------------------------------
// M1 (bridges / wave_seeds): Z-preserving OPEN-PATH offset.
// RegionExpansion.cpp:83-106 expolygons_to_zpaths_expanded_opened +
// ClipperZUtils::to_zpaths<true> (Open: repeat first point at end, tag Z=base_idx).
// ---------------------------------------------------------------------------
extern "C" CzZPaths cz_offset_open(const int32_t *contour_xy, const int32_t *contour_lens,
                                   const int32_t *contour_per_ex, int32_t num_ex,
                                   double expansion, double shortest_edge_length,
                                   int32_t base_idx_start, int32_t *base_idx_out) {
    using ClipperLib_Z::Path;
    using ClipperLib_Z::Paths;

    Paths out;

    // ClipperOffset here is the NON-Z ClipperLib (offsetting is a 2D operation);
    // we tag the Z ourselves afterwards (to_zpaths<true>), matching the C++.
    ClipperLib::ClipperOffset offsetter;
    offsetter.ShortestEdgeLength = shortest_edge_length;

    int32_t base_idx = base_idx_start;
    const int32_t *xy_cursor = contour_xy;   // walks the flat (x,y) pairs
    int32_t contour_global = 0;              // index into contour_lens

    for (int32_t ex = 0; ex < num_ex; ++ex) {
        int32_t ncontours = contour_per_ex[ex];
        for (int32_t ic = 0; ic < ncontours; ++ic) {
            int32_t len = contour_lens[contour_global];
            // Build the input contour (2D path for the non-Z offsetter).
            ClipperLib::Path in;
            in.reserve(len);
            for (int32_t i = 0; i < len; ++i)
                in.emplace_back(xy_cursor[2 * i], xy_cursor[2 * i + 1]);
            xy_cursor += 2 * len;
            ++contour_global;

            offsetter.Clear();
            offsetter.AddPath(in, ClipperLib::jtSquare, ClipperLib::etClosedPolygon);
            ClipperLib::Paths expansion_cache;
            // contour 0 (outer) => +expansion; holes => -expansion (RegionExpansion.cpp:100).
            offsetter.Execute(expansion_cache, ic == 0 ? expansion : -expansion);

            // to_zpaths<true>: open each offset polygon (repeat first pt) + tag Z=base_idx.
            for (const ClipperLib::Path &p : expansion_cache) {
                if (p.empty())
                    continue;
                Path zp;
                zp.reserve(p.size() + 1);
                for (const ClipperLib::IntPoint &ip : p)
                    zp.emplace_back((int32_t) ip.x(), (int32_t) ip.y(), base_idx);
                zp.emplace_back(zp.front()); // Open: duplicate first point at end
                out.emplace_back(std::move(zp));
            }
        }
        ++base_idx;
    }

    if (base_idx_out)
        *base_idx_out = base_idx;

    // Marshal into the flat CzZPaths struct.
    CzZPaths res;
    res.num_paths = (int32_t) out.size();
    int32_t total = 0;
    for (const Path &p : out)
        total += (int32_t) p.size();
    res.total_points = total;
    res.path_lens = res.num_paths > 0
        ? (int32_t *) std::malloc(sizeof(int32_t) * res.num_paths)
        : nullptr;
    for (int32_t i = 0; i < res.num_paths; ++i)
        res.path_lens[i] = (int32_t) out[i].size();
    res.coords = total > 0 ? (int32_t *) std::malloc(sizeof(int32_t) * 3 * total) : nullptr;
    {
        int32_t k = 0;
        for (const Path &p : out)
            for (const IntPoint &ip : p) {
                res.coords[3 * k + 0] = (int32_t) ip.x();
                res.coords[3 * k + 1] = (int32_t) ip.y();
                res.coords[3 * k + 2] = (int32_t) ip.z();
                ++k;
            }
    }
    return res;
}

// ---------------------------------------------------------------------------
// M1 (bridges / wave_seeds): provenance Z-clip core.
// RegionExpansion.cpp:302-327 (ClipperLib_Z engine equivalent) +
// ClipperZUtils::ClipperZIntersectionVisitor (ClipperZUtils.hpp:125-160).
// ---------------------------------------------------------------------------
extern "C" CzWaveSeeds cz_wave_seeds_clip(const int32_t *subj_xyz, const int32_t *subj_lens,
                                          int32_t subj_num, const int32_t *clip_xyz,
                                          const int32_t *clip_lens, int32_t clip_num) {
    auto read_zpaths = [](const int32_t *xyz, const int32_t *lens, int32_t num) {
        Paths paths;
        paths.reserve(num);
        const int32_t *cur = xyz;
        for (int32_t c = 0; c < num; ++c) {
            int32_t len = lens[c];
            Path p;
            p.reserve(len);
            for (int32_t i = 0; i < len; ++i)
                p.emplace_back(cur[3 * i], cur[3 * i + 1], cur[3 * i + 2]);
            cur += 3 * len;
            paths.emplace_back(std::move(p));
        }
        return paths;
    };

    Paths subj = read_zpaths(subj_xyz, subj_lens, subj_num);
    Paths clip = read_zpaths(clip_xyz, clip_lens, clip_num);

    // Intersections table built by the ZFillFunction (ClipperZUtils.hpp:125-160).
    std::vector<std::pair<int32_t, int32_t>> intersections;

    ClipperLib_Z::Clipper zclipper;
    zclipper.ZFillFunction([&intersections](const IntPoint &e1bot, const IntPoint &e1top,
                                            const IntPoint &e2bot, const IntPoint &e2top,
                                            IntPoint &pt) {
        // ClipperZIntersectionVisitor::operator() — collect the distinct source Z
        // values; on a 2-distinct-source intersection record a -1-based negative
        // index into the intersections table.
        int32_t srcs[4] = {(int32_t) e1bot.z(), (int32_t) e1top.z(), (int32_t) e2bot.z(),
                           (int32_t) e2top.z()};
        std::sort(srcs, srcs + 4);
        int32_t *end = std::unique(srcs, srcs + 4);
        if (srcs + 1 == end) {
            // Self intersection on a source contour: just copy the Z value.
            pt.z() = srcs[0];
        } else {
            // 2 (or more — take the first two) distinct sources => record intersection.
            intersections.emplace_back(srcs[0], srcs[1]);
            pt.z() = -(int32_t) intersections.size();
        }
    });

    // Boundary as CLOSED clip, offset-opened src as OPEN subject (RegionExpansion.cpp:307/313).
    zclipper.AddPaths(clip, ClipperLib_Z::ptClip, true);
    zclipper.AddPaths(subj, ClipperLib_Z::ptSubject, false);

    ClipperLib_Z::PolyTree polytree;
    zclipper.Execute(ClipperLib_Z::ctIntersection, polytree, ClipperLib_Z::pftNonZero,
                     ClipperLib_Z::pftNonZero);

    // ClipperLib (clipper1) returns open/closed via the PolyTree helpers; the C++
    // wave_seeds (Clipper2) splits them as closed_segs + open_segs and concatenates
    // closed-then-open (RegionExpansion.cpp:324-325). Mirror that ordering here.
    Paths closed_segs, open_segs;
    ClipperLib_Z::ClosedPathsFromPolyTree(polytree, closed_segs);
    ClipperLib_Z::OpenPathsFromPolyTree(polytree, open_segs);

    Paths segments;
    segments.reserve(closed_segs.size() + open_segs.size());
    for (Path &p : closed_segs)
        segments.emplace_back(std::move(p));
    for (Path &p : open_segs)
        segments.emplace_back(std::move(p));

    CzWaveSeeds res;
    res.num_paths = (int32_t) segments.size();
    res.num_closed = (int32_t) closed_segs.size();
    int32_t total = 0;
    for (const Path &p : segments)
        total += (int32_t) p.size();
    res.total_points = total;
    res.path_lens = res.num_paths > 0
        ? (int32_t *) std::malloc(sizeof(int32_t) * res.num_paths)
        : nullptr;
    for (int32_t i = 0; i < res.num_paths; ++i)
        res.path_lens[i] = (int32_t) segments[i].size();
    res.coords = total > 0 ? (int32_t *) std::malloc(sizeof(int32_t) * 3 * total) : nullptr;
    {
        int32_t k = 0;
        for (const Path &p : segments)
            for (const IntPoint &ip : p) {
                res.coords[3 * k + 0] = (int32_t) ip.x();
                res.coords[3 * k + 1] = (int32_t) ip.y();
                res.coords[3 * k + 2] = (int32_t) ip.z();
                ++k;
            }
    }
    res.num_intersections = (int32_t) intersections.size();
    res.intersections = res.num_intersections > 0
        ? (int32_t *) std::malloc(sizeof(int32_t) * 2 * res.num_intersections)
        : nullptr;
    for (int32_t i = 0; i < res.num_intersections; ++i) {
        res.intersections[2 * i + 0] = intersections[i].first;
        res.intersections[2 * i + 1] = intersections[i].second;
    }
    return res;
}

extern "C" void cz_free_wave_seeds(CzWaveSeeds seeds) {
    std::free(seeds.coords);
    std::free(seeds.path_lens);
    std::free(seeds.intersections);
}
