// Implementation of the Clipper2-Z C-ABI shim. See clipper2_z_shim.h.
//
// The vendored Clipper2 headers/sources are namespace-renamed Clipper2Lib ->
// Clipper2ZSys (build.rs) for ODR isolation from clipper2c-sys's non-Z Clipper2.
#include "clipper2/clipper.h"      // -> namespace Clipper2ZSys (USINGZ)

#include "clipper2_z_shim.h"

#include <algorithm>
#include <array>
#include <cstdlib>
#include <vector>

using namespace Clipper2ZSys;

namespace {

// Marshal Clipper2ZSys::Paths64 (with z) into a freshly-malloc'd Cz2ZPaths.
Cz2ZPaths marshal_zpaths(const Paths64 &paths) {
    Cz2ZPaths out;
    out.num_paths = (int32_t) paths.size();
    int32_t total = 0;
    for (const Path64 &p : paths)
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
        out.coords = (int64_t *) std::malloc(sizeof(int64_t) * 3 * total);
        int32_t k = 0;
        for (const Path64 &p : paths)
            for (const Point64 &ip : p) {
                out.coords[3 * k + 0] = (int64_t) ip.x;
                out.coords[3 * k + 1] = (int64_t) ip.y;
                out.coords[3 * k + 2] = (int64_t) ip.z;
                ++k;
            }
    } else {
        out.coords = nullptr;
    }
    return out;
}

// Read flat (x,y,z) triples + per-path lens into Clipper2ZSys::Paths64.
Paths64 read_zpaths(const int64_t *xyz, const int32_t *lens, int32_t num) {
    Paths64 out;
    out.reserve(num);
    const int64_t *cursor = xyz;
    for (int32_t c = 0; c < num; ++c) {
        int32_t len = lens[c];
        Path64 path;
        path.reserve(len);
        for (int32_t i = 0; i < len; ++i)
            path.emplace_back(cursor[3 * i], cursor[3 * i + 1], cursor[3 * i + 2]);
        cursor += 3 * len;
        out.emplace_back(std::move(path));
    }
    return out;
}

} // namespace

extern "C" const char *cz2_version(void) {
    return CLIPPER2_VERSION;
}

extern "C" Cz2ZPaths cz2_offset_z(const int64_t *contour_xyz, int32_t n, double delta) {
    // expolygons_to_zpaths64_expanded_opened (RegionExpansion.cpp:108-136):
    // one ClipperOffset per contour; AddPath(JoinType::Square, EndType::Polygon);
    // Execute(offset_distance, expansion_cache). The input vertices already carry
    // the source z (base_idx); Clipper2's USINGZ offset preserves it.
    ClipperOffset offsetter;
    Path64 path;
    path.reserve(n);
    for (int32_t i = 0; i < n; ++i)
        path.emplace_back(contour_xyz[3 * i], contour_xyz[3 * i + 1], contour_xyz[3 * i + 2]);
    offsetter.AddPath(path, JoinType::Square, EndType::Polygon);

    Paths64 result;
    offsetter.Execute(delta, result);
    return marshal_zpaths(result);
}

extern "C" Cz2WaveClip cz2_intersect_open_z(const int64_t *src_xyz, const int32_t *src_lens,
                                            int32_t src_num, const int64_t *clip_xyz,
                                            const int32_t *clip_lens, int32_t clip_num) {
    Paths64 src = read_zpaths(src_xyz, src_lens, src_num);   // open subject
    Paths64 clip = read_zpaths(clip_xyz, clip_lens, clip_num); // closed clip

    // The Intersections table the Z-callback records (Clipper2ZIntersectionVisitor,
    // Clipper2ZUtils.hpp). Each entry is the (sorted, unique) pair of the two source
    // z-values that produced an intersection point; the point's z becomes
    // -(table_size) so callers can index it as `-z - 1`.
    std::vector<std::pair<int64_t, int64_t>> intersections;

    Clipper64 clipper;
    // SetZCallback — verbatim Clipper2ZIntersectionVisitor::operator() logic.
    clipper.SetZCallback([&intersections](const Point64 &e1bot, const Point64 &e1top,
                                          const Point64 &e2bot, const Point64 &e2top,
                                          Point64 &pt) {
        std::array<int64_t, 4> srcs{e1bot.z, e1top.z, e2bot.z, e2top.z};
        std::sort(srcs.begin(), srcs.end());
        auto it = std::unique(srcs.begin(), srcs.end());
        int new_size = (int) std::distance(srcs.begin(), it);
        if (new_size == 1) {
            pt.z = srcs[0];
        } else if (new_size == 2) {
            intersections.emplace_back(srcs[0], srcs[1]);
            pt.z = -int64_t(intersections.size());
        }
    });

    clipper.AddClip(clip);
    clipper.AddOpenSubject(src);

    Paths64 closed_segs, open_segs;
    clipper.Execute(ClipType::Intersection, FillRule::NonZero, closed_segs, open_segs);

    // segments = closed_segs ++ open_segs (the order wave_seeds consumes).
    Paths64 segments;
    segments.reserve(closed_segs.size() + open_segs.size());
    for (auto &p : closed_segs) segments.emplace_back(std::move(p));
    for (auto &p : open_segs) segments.emplace_back(std::move(p));

    Cz2WaveClip out;
    out.segs = marshal_zpaths(segments);
    out.num_closed = (int32_t) closed_segs.size();
    out.num_is = (int32_t) intersections.size();
    if (out.num_is > 0) {
        out.is_a = (int64_t *) std::malloc(sizeof(int64_t) * out.num_is);
        out.is_b = (int64_t *) std::malloc(sizeof(int64_t) * out.num_is);
        for (int32_t i = 0; i < out.num_is; ++i) {
            out.is_a[i] = intersections[i].first;
            out.is_b[i] = intersections[i].second;
        }
    } else {
        out.is_a = nullptr;
        out.is_b = nullptr;
    }
    return out;
}

// ---------------------------------------------------------------------------
// R196: cz2_pl_open — faithful Clipper2Utils.cpp _clipper2_pl_open
// (Clipper2Utils.cpp:119-136) on the NATIVE Clipper2 1.5.2 (this crate's
// vendored copy tracks the reference tree; clipper2c-sys ships 1.5.4 whose
// open-path clipping differs). clip_type: 0 = Intersection, 1 = Difference.
// Output = closed solution paths then open solution paths, verbatim order.
// z values are ignored by this entry (callers pass z=0).
// ---------------------------------------------------------------------------
extern "C" Cz2ZPaths cz2_pl_open(int32_t clip_type,
                                 const int64_t *src_xyz, const int32_t *src_lens, int32_t src_num,
                                 const int64_t *clip_xyz, const int32_t *clip_lens, int32_t clip_num) {
    Paths64 src = read_zpaths(src_xyz, src_lens, src_num);    // open subject
    Paths64 clip = read_zpaths(clip_xyz, clip_lens, clip_num); // closed clip

    Clipper64 clipper;
    clipper.AddOpenSubject(src);
    clipper.AddClip(clip);

    Paths64 solution, solution_open;
    clipper.Execute(clip_type == 0 ? ClipType::Intersection : ClipType::Difference,
                    FillRule::NonZero, solution, solution_open);

    Paths64 out;
    out.reserve(solution.size() + solution_open.size());
    for (auto &pth : solution) out.emplace_back(std::move(pth));
    for (auto &pth : solution_open) out.emplace_back(std::move(pth));
    return marshal_zpaths(out);
}

extern "C" void cz2_free_zpaths(Cz2ZPaths paths) {
    std::free(paths.coords);
    std::free(paths.path_lens);
}

extern "C" void cz2_free_wave_clip(Cz2WaveClip wc) {
    std::free(wc.segs.coords);
    std::free(wc.segs.path_lens);
    std::free(wc.is_a);
    std::free(wc.is_b);
}
