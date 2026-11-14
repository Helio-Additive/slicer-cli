// Test-only reference exports: `Clipper64::Execute` and helpers of clipper.core.h of the
// Clipper2 the engine pins (`references/BambuStudio/src/clipper2/Clipper2Lib`, see build.rs)
// for the `clipper2_engine*_native_parity` tests of `crates/libslic3r-rs`. Nothing in the
// engine calls these.
//
// This file is compiled twice (build.rs), each time with the pin's clipper.engine.cpp:
//   - with USINGZ, the library's `namespace Clipper2Lib_Z` renamed to `Clipper2RefZ`: the
//     exports are `cz2_ref_z_*`;
//   - without USINGZ, the library's `namespace Clipper2Lib` renamed to `Clipper2RefXY`: the
//     exports are `cz2_ref_xy_*`.
// `CZ2_REF_NAMESPACE` is the namespace of the build.
#include "clipper2/clipper.engine.h"

#include "clipper2_ref_shim.h"

#include <algorithm>
#include <array>
#include <cstdlib>
#include <utility>
#include <vector>

using namespace CZ2_REF_NAMESPACE;

#ifdef USINGZ
#define CZ2_REF_NAME(name) cz2_ref_z_##name
#else
#define CZ2_REF_NAME(name) cz2_ref_xy_##name
#endif

namespace {

Paths64 read_paths(const int64_t *xyz, const int32_t *lens, int32_t num) {
    Paths64 out;
    out.reserve(num);
    const int64_t *cursor = xyz;
    for (int32_t c = 0; c < num; ++c) {
        int32_t len = lens[c];
        Path64 path;
        path.reserve(len);
        for (int32_t i = 0; i < len; ++i) {
#ifdef USINGZ
            path.emplace_back(cursor[3 * i], cursor[3 * i + 1], cursor[3 * i + 2]);
#else
            path.emplace_back(cursor[3 * i], cursor[3 * i + 1]);
#endif
        }
        cursor += 3 * len;
        out.emplace_back(std::move(path));
    }
    return out;
}

struct Flat {
    std::vector<int64_t> coords;
    std::vector<int32_t> lens;
    void add(const Path64 &p) {
        lens.push_back((int32_t) p.size());
        for (const Point64 &q : p) {
            coords.push_back(q.x);
            coords.push_back(q.y);
#ifdef USINGZ
            coords.push_back(q.z);
#else
            coords.push_back(0);
#endif
        }
    }
};

void walk(const PolyPath64 &node, int32_t parent, Flat &flat, std::vector<int32_t> &parents,
          std::vector<int32_t> &holes) {
    for (size_t i = 0; i < node.Count(); ++i) {
        const PolyPath64 *child = node.Child(i);
        int32_t me = (int32_t) parents.size();
        parents.push_back(parent);
        holes.push_back(child->IsHole() ? 1 : 0);
        flat.add(child->Polygon());
        walk(*child, me, flat, parents, holes);
    }
}

template <typename T> T *copy_out(const std::vector<T> &v) {
    if (v.empty()) return nullptr;
    T *out = (T *) std::malloc(sizeof(T) * v.size());
    std::copy(v.begin(), v.end(), out);
    return out;
}

inline void mix(uint64_t &h, int64_t v) {
    h = (h ^ (uint64_t) v) * 0x100000001b3ULL;
}

} // namespace

extern "C" Cz2RefResult CZ2_REF_NAME(execute)(const int64_t *subj_xyz, const int32_t *subj_lens, int32_t subj_num,
                                              const int64_t *open_xyz, const int32_t *open_lens, int32_t open_num,
                                              const int64_t *clip_xyz, const int32_t *clip_lens, int32_t clip_num,
                                              int32_t clip_type, int32_t fill_rule, int32_t preserve_collinear,
                                              int32_t reverse_solution, int32_t use_tree, int32_t add_order,
                                              int32_t z_callback, int64_t default_z) {
    Paths64 subj = read_paths(subj_xyz, subj_lens, subj_num);
    Paths64 open = read_paths(open_xyz, open_lens, open_num);
    Paths64 clip = read_paths(clip_xyz, clip_lens, clip_num);

    Clipper64 clipper;
    clipper.PreserveCollinear(preserve_collinear != 0);
    clipper.ReverseSolution(reverse_solution != 0);

    uint64_t cb_calls = 0;
    uint64_t cb_hash = 0xcbf29ce484222325ULL;
#ifdef USINGZ
    clipper.DefaultZ = default_z;
    std::vector<std::pair<int64_t, int64_t>> intersections;
    if (z_callback == 1) {
        // Clipper2ZIntersectionVisitor::operator() (Clipper2ZUtils.hpp), as in
        // cz2_intersect_open_z of clipper2_z_shim.cpp, without the assert.
        clipper.SetZCallback([&](const Point64 &e1bot, const Point64 &e1top, const Point64 &e2bot,
                                 const Point64 &e2top, Point64 &pt) {
            ++cb_calls;
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
    } else if (z_callback == 2) {
        // Every argument, the z the clipper gave the point before the call, and the number
        // of the call go into the z that comes out.
        clipper.SetZCallback([&](const Point64 &e1bot, const Point64 &e1top, const Point64 &e2bot,
                                 const Point64 &e2top, Point64 &pt) {
            ++cb_calls;
            uint64_t h = 0xcbf29ce484222325ULL;
            mix(h, (int64_t) cb_calls);
            for (const Point64 *p : {&e1bot, &e1top, &e2bot, &e2top, (const Point64 *) &pt}) {
                mix(h, p->x);
                mix(h, p->y);
                mix(h, p->z);
            }
            pt.z = (int64_t) (h >> 1);
            mix(cb_hash, pt.z);
        });
    }
#else
    (void) z_callback;
    (void) default_z;
#endif

    // add_order bit 0: the order the paths go in; bit 1: one `Execute` (Xor, EvenOdd) whose
    // result is dropped runs before the one asked for, on the same clipper; bit 2: the paths
    // go in one by one, each in its own AddSubject / AddOpenSubject / AddClip.
    auto add = [&](const Paths64 &paths, int kind) {
        auto one = [&](const Paths64 &ps) {
            if (kind == 0) clipper.AddSubject(ps);
            else if (kind == 1) clipper.AddOpenSubject(ps);
            else clipper.AddClip(ps);
        };
        if (add_order & 4) {
            for (const Path64 &p : paths) one(Paths64(1, p));
        } else {
            one(paths);
        }
    };
    if ((add_order & 1) == 0) {
        add(subj, 0);
        add(open, 1);
        add(clip, 2);
    } else {
        add(clip, 2);
        add(open, 1);
        add(subj, 0);
    }
    if (add_order & 2) {
        Paths64 dropped_closed, dropped_open;
        clipper.Execute(ClipType::Xor, FillRule::EvenOdd, dropped_closed, dropped_open);
    }

    Flat flat;
    std::vector<int32_t> parents, holes;
    Cz2RefResult out;
    Paths64 open_paths;
    if (use_tree) {
        PolyTree64 tree;
        out.succeeded = clipper.Execute((ClipType) clip_type, (FillRule) fill_rule, tree, open_paths) ? 1 : 0;
        walk(tree, -1, flat, parents, holes);
        out.num_closed = (int32_t) parents.size();
    } else {
        Paths64 closed;
        out.succeeded = clipper.Execute((ClipType) clip_type, (FillRule) fill_rule, closed, open_paths) ? 1 : 0;
        for (const Path64 &p : closed) flat.add(p);
        out.num_closed = (int32_t) closed.size();
    }
    for (const Path64 &p : open_paths) flat.add(p);
    out.num_open = (int32_t) open_paths.size();

#ifdef USINGZ
    for (const auto &is : intersections) {
        mix(cb_hash, is.first);
        mix(cb_hash, is.second);
    }
#endif

    out.total_points = (int32_t) (flat.coords.size() / 3);
    out.coords = copy_out(flat.coords);
    out.path_lens = copy_out(flat.lens);
    out.tree_parent = copy_out(parents);
    out.tree_is_hole = copy_out(holes);
    out.cb_calls = cb_calls;
    out.cb_hash = cb_hash;
    return out;
}

extern "C" void CZ2_REF_NAME(free)(Cz2RefResult r) {
    std::free(r.coords);
    std::free(r.path_lens);
    std::free(r.tree_parent);
    std::free(r.tree_is_hole);
}

// The helpers of clipper.core.h the tests compare one by one.
extern "C" int32_t CZ2_REF_NAME(point_in_polygon)(int64_t x, int64_t y, const int64_t *xyz, int32_t n) {
    Path64 path;
    for (int32_t i = 0; i < n; ++i) path.emplace_back(xyz[3 * i], xyz[3 * i + 1]);
    return (int32_t) PointInPolygon(Point64(x, y), path);
}

extern "C" double CZ2_REF_NAME(area)(const int64_t *xyz, int32_t n) {
    Path64 path;
    for (int32_t i = 0; i < n; ++i) path.emplace_back(xyz[3 * i], xyz[3 * i + 1]);
    return Area(path);
}

// p: 4 points a, b, c, d of (x, y, z) each; z is read in the build with USINGZ only.
// out_i[0..3]: ip of GetSegmentIntersectPt(a, b, c, d, ip) with ip = (0, 0, -7) before the call
//              (x, y, z; z = 0 in the build without Z);
// out_i[3..6]: GetClosestPointOnSegment(a, b, c) (x, y, z);
// out_i[6..9]: MidPoint(a, b) (x, y, z);
// out_b: GetSegmentIntersectPt's result, SegmentsIntersect(a, b, c, d, false) and (…, true),
//        IsCollinear(a, b, c), ProductsAreEqual(a.x, a.y, b.x, b.y), a == b;
// out_d: CrossProduct(a, b, c), DotProduct(a, b, c), PerpendicDistFromLineSqrd(a, b, c),
//        DistanceSqr(a, b).
extern "C" void CZ2_REF_NAME(core)(const int64_t *p, int64_t *out_i, double *out_d, int32_t *out_b) {
#ifdef USINGZ
    Point64 a(p[0], p[1], p[2]), b(p[3], p[4], p[5]), c(p[6], p[7], p[8]), d(p[9], p[10], p[11]);
    Point64 ip(0, 0, -7);
#define CZ2_Z(pt) (pt).z
#else
    Point64 a(p[0], p[1]), b(p[3], p[4]), c(p[6], p[7]), d(p[9], p[10]);
    Point64 ip(0, 0);
#define CZ2_Z(pt) 0
#endif
    out_b[0] = GetSegmentIntersectPt(a, b, c, d, ip) ? 1 : 0;
    out_i[0] = ip.x;
    out_i[1] = ip.y;
    out_i[2] = CZ2_Z(ip);
    Point64 cp = GetClosestPointOnSegment(a, b, c);
    out_i[3] = cp.x;
    out_i[4] = cp.y;
    out_i[5] = CZ2_Z(cp);
    Point64 mp = MidPoint(a, b);
    out_i[6] = mp.x;
    out_i[7] = mp.y;
    out_i[8] = CZ2_Z(mp);
    out_b[1] = SegmentsIntersect(a, b, c, d, false) ? 1 : 0;
    out_b[2] = SegmentsIntersect(a, b, c, d, true) ? 1 : 0;
    out_b[3] = IsCollinear(a, b, c) ? 1 : 0;
    out_b[4] = ProductsAreEqual(a.x, a.y, b.x, b.y) ? 1 : 0;
    out_b[5] = (a == b) ? 1 : 0;
    out_d[0] = CrossProduct(a, b, c);
    out_d[1] = DotProduct(a, b, c);
    out_d[2] = PerpendicDistFromLineSqrd(a, b, c);
    out_d[3] = DistanceSqr(a, b);
#undef CZ2_Z
}

// 1 where the reference is built from the engine's pin, 0 where from the vendored copy.
extern "C" int32_t CZ2_REF_NAME(from_pin)(void) {
    return CZ2_REF_FROM_PIN;
}

// CLIPPER2_VERSION of the sources the reference is built from.
extern "C" const char *CZ2_REF_NAME(version)(void) {
    return CLIPPER2_VERSION;
}
