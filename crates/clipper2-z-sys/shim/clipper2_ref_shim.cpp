// Test-only reference exports: `Clipper64::Execute`, `ClipperOffset::Execute` and helpers
// of clipper.core.h and clipper.h of the Clipper2 the engine pins
// (`references/BambuStudio/src/clipper2/Clipper2Lib`, see build.rs) for the
// `clipper2_engine*_native_parity` and `clipper2_offset*_native_parity` tests of
// `crates/libslic3r-rs`. Nothing in the engine calls these.
//
// This file is compiled twice (build.rs), each time with the pin's clipper.engine.cpp and
// clipper.offset.cpp:
//   - with USINGZ, the library's `namespace Clipper2Lib_Z` renamed to `Clipper2RefZ`: the
//     exports are `cz2_ref_z_*`;
//   - without USINGZ, the library's `namespace Clipper2Lib` renamed to `Clipper2RefXY`: the
//     exports are `cz2_ref_xy_*`.
// `CZ2_REF_NAMESPACE` is the namespace of the build.
#include "clipper2/clipper.engine.h"
#include "clipper2/clipper.h"

#include "clipper2_ref_shim.h"

#include <algorithm>
#include <array>
#include <cmath>
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

// ClipperOffset.
//
// `xyz` / `lens`: the paths; `group_sizes[g]` of them, in order, make group g, which goes in
// with JoinType `group_join[g]` and EndType `group_end[g]`.
// mode bit 0: a group of one path goes in through AddPath (otherwise AddPaths);
//      bit 1: one Execute(delta2) to paths, dropped, runs first on the same offsetter (what
//             it leaves behind in the members is there for the run asked for);
//      bit 2: InflatePaths(paths of group 0, delta, join, end, miter_limit, arc_tolerance)
//             of clipper.h in the place of the offsetter (paths only);
//      bit 3: after the run asked for: Clear(), the solution goes in again as one group
//             with the join and end type of group 0, and Execute(delta2) the same way.
// delta_cb: 0 none (Execute(delta, ...)); otherwise SetDeltaCallback of one of the
//      functions below, each made of `delta` (bit 4 of it: through
//      Execute(DeltaCallback64, paths) in the place of SetDeltaCallback + Execute(delta)).
// z_callback (USINGZ): 0 none; 2 one that hashes all it is given, as in `execute`.
// The result is as of `execute`; `succeeded` is 1 - ErrorCode().
extern "C" Cz2RefResult CZ2_REF_NAME(offset)(const int64_t *xyz, const int32_t *lens, const int32_t *group_sizes,
                                             const int32_t *group_join, const int32_t *group_end, int32_t num_groups,
                                             double delta, double delta2, double miter_limit, double arc_tolerance,
                                             int32_t preserve_collinear, int32_t reverse_solution, int32_t use_tree,
                                             int32_t mode, int32_t delta_cb, int32_t z_callback) {
    int32_t total = 0;
    for (int32_t g = 0; g < num_groups; ++g) total += group_sizes[g];
    Paths64 all = read_paths(xyz, lens, total);

    uint64_t cb_calls = 0;
    uint64_t cb_hash = 0xcbf29ce484222325ULL;
    Flat flat;
    std::vector<int32_t> parents, holes;
    Cz2RefResult out;
    out.num_open = 0;
    out.succeeded = 1;

    if (mode & 4) {
        Paths64 first(all.begin(), all.begin() + (num_groups > 0 ? group_sizes[0] : 0));
        Paths64 sol = InflatePaths(first, delta, (JoinType) group_join[0], (EndType) group_end[0], miter_limit, arc_tolerance);
        for (const Path64 &p : sol) flat.add(p);
        out.num_closed = (int32_t) sol.size();
    } else {
        ClipperOffset co(miter_limit, arc_tolerance, preserve_collinear != 0, reverse_solution != 0);
#ifdef USINGZ
        if (z_callback == 2) {
            co.SetZCallback([&](const Point64 &e1bot, const Point64 &e1top, const Point64 &e2bot,
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
#endif
        size_t at = 0;
        for (int32_t g = 0; g < num_groups; ++g) {
            Paths64 group(all.begin() + at, all.begin() + at + group_sizes[g]);
            at += group_sizes[g];
            if ((mode & 1) && group.size() == 1) co.AddPath(group[0], (JoinType) group_join[g], (EndType) group_end[g]);
            else co.AddPaths(group, (JoinType) group_join[g], (EndType) group_end[g]);
        }
        if (mode & 2) {
            Paths64 dropped;
            co.Execute(delta2, dropped);
        }

        // The delta callbacks. Every product is in a statement of its own, so that none is
        // fused with a sum.
        DeltaCallback64 cb = nullptr;
        switch (delta_cb & 15) {
        case 1: // by the two indices
            cb = [delta](const Path64 &, const PathD &, size_t curr, size_t prev) {
                double f = 0.5 + 0.25 * (double) ((curr + 2 * prev) % 5);
                return delta * f;
            };
            break;
        case 2: // by the normal of the vertex
            cb = [delta](const Path64 &, const PathD &norms, size_t curr, size_t) {
                double t = 0.25 * norms[curr].x;
                double f = 1.0 + t;
                return delta * f;
            };
            break;
        case 3: // nothing at every third vertex
            cb = [delta](const Path64 &, const PathD &, size_t curr, size_t) { return (curr % 3 == 0) ? 0.0 : delta; };
            break;
        case 4: // by the place of the vertex and the size of the path
            cb = [delta](const Path64 &path, const PathD &norms, size_t curr, size_t prev) {
                int64_t v = (path[curr].x ^ path[prev].y) & 7;
                double f = 0.75 + 0.125 * (double) v;
                double g = (norms.size() == path.size()) ? 1.0 : 2.0;
                double d = delta * f;
                return d * g;
            };
            break;
        default: break;
        }

        auto run = [&](double d) {
            flat = Flat();
            parents.clear();
            holes.clear();
            if (use_tree) {
                PolyTree64 tree;
                if (cb) co.SetDeltaCallback(cb);
                co.Execute(d, tree);
                walk(tree, -1, flat, parents, holes);
                out.num_closed = (int32_t) parents.size();
                return PolyTreeToPaths64(tree);
            }
            Paths64 sol;
            if (cb && (delta_cb & 16)) co.Execute(cb, sol);
            else {
                if (cb) co.SetDeltaCallback(cb);
                co.Execute(d, sol);
            }
            for (const Path64 &p : sol) flat.add(p);
            out.num_closed = (int32_t) sol.size();
            return sol;
        };
        Paths64 sol = run(delta);
        if (mode & 8) {
            co.Clear();
            co.AddPaths(sol, (JoinType) group_join[0], (EndType) group_end[0]);
            run(delta2);
        }
        out.succeeded = 1 - co.ErrorCode();
    }

    out.total_points = (int32_t) (flat.coords.size() / 3);
    out.coords = copy_out(flat.coords);
    out.path_lens = copy_out(flat.lens);
    out.tree_parent = copy_out(parents);
    out.tree_is_hole = copy_out(holes);
    out.cb_calls = cb_calls;
    out.cb_hash = cb_hash;
    return out;
}

// One helper of clipper.h on one path of n points. The result is one closed path, or none.
// op 0: SimplifyPath(path, a, flag != 0)            (flag: isClosedPath)
//    1: RamerDouglasPeucker(path, a)
//    2: TrimCollinear(path, flag != 0)              (flag: is_open_path)
//    3: Ellipse(path[0], a, b, (size_t) flag)
//    4: TranslatePath(path, flag, -flag)
//    5: nothing comes back but out_d: Length(path, false), Length(path, true),
//       Distance(path[0], path[1]) and, for a path of 3 points or more,
//       NearCollinear(path[0], path[1], path[2], a) as 0 or 1.
extern "C" Cz2RefResult CZ2_REF_NAME(path_op)(int32_t op, const int64_t *xyz, int32_t n, double a, double b,
                                              int64_t flag, double *out_d) {
    int32_t len = n;
    Paths64 in = read_paths(xyz, &len, 1);
    const Path64 &path = in[0];
    Flat flat;
    Cz2RefResult out;
    out.num_closed = 1;
    out.num_open = 0;
    out.succeeded = 1;
    out.cb_calls = 0;
    out.cb_hash = 0;
    switch (op) {
    case 0: flat.add(SimplifyPath(path, a, flag != 0)); break;
    case 1: flat.add(RamerDouglasPeucker(path, a)); break;
    case 2: flat.add(TrimCollinear(path, flag != 0)); break;
    case 3: flat.add(Ellipse(path[0], a, b, (size_t) flag)); break;
    case 4: flat.add(TranslatePath(path, flag, -flag)); break;
    default:
        out.num_closed = 0;
        out_d[0] = Length(path, false);
        out_d[1] = Length(path, true);
        out_d[2] = n >= 2 ? Distance(path[0], path[1]) : 0.0;
        out_d[3] = n >= 3 ? (NearCollinear(path[0], path[1], path[2], a) ? 1.0 : 0.0) : 0.0;
        break;
    }
    out.total_points = (int32_t) (flat.coords.size() / 3);
    out.coords = copy_out(flat.coords);
    out.path_lens = copy_out(flat.lens);
    out.tree_parent = nullptr;
    out.tree_is_hole = nullptr;
    return out;
}

// BooleanOp and its wrappers of clipper.h.
// op 0: BooleanOp(clip_type, fill_rule, subjects, clips) to paths;
//    1: BooleanOp(clip_type, fill_rule, subjects, clips, tree): the tree, as of `execute`;
//    2: the same, and PolyTreeToPaths64(tree) comes back as paths; `succeeded` is
//       CheckPolytreeFullyContainsChildren(tree);
//    3: Union(subjects, fill_rule);
//    4: Intersect / Union / Difference / Xor(subjects, clips, fill_rule) by clip_type 1..4.
extern "C" Cz2RefResult CZ2_REF_NAME(boolean_op)(int32_t op, int32_t clip_type, int32_t fill_rule,
                                                 const int64_t *subj_xyz, const int32_t *subj_lens, int32_t subj_num,
                                                 const int64_t *clip_xyz, const int32_t *clip_lens, int32_t clip_num) {
    Paths64 subj = read_paths(subj_xyz, subj_lens, subj_num);
    Paths64 clip = read_paths(clip_xyz, clip_lens, clip_num);
    Flat flat;
    std::vector<int32_t> parents, holes;
    Cz2RefResult out;
    out.num_open = 0;
    out.succeeded = 1;
    out.cb_calls = 0;
    out.cb_hash = 0;
    Paths64 sol;
    if (op == 1 || op == 2) {
        PolyTree64 tree;
        BooleanOp((ClipType) clip_type, (FillRule) fill_rule, subj, clip, tree);
        if (op == 1) {
            walk(tree, -1, flat, parents, holes);
            out.num_closed = (int32_t) parents.size();
        } else {
            sol = PolyTreeToPaths64(tree);
            out.succeeded = CheckPolytreeFullyContainsChildren(tree) ? 1 : 0;
        }
    } else if (op == 0) {
        sol = BooleanOp((ClipType) clip_type, (FillRule) fill_rule, subj, clip);
    } else if (op == 3) {
        sol = Union(subj, (FillRule) fill_rule);
    } else {
        switch (clip_type) {
        case 1: sol = Intersect(subj, clip, (FillRule) fill_rule); break;
        case 2: sol = Union(subj, clip, (FillRule) fill_rule); break;
        case 3: sol = Difference(subj, clip, (FillRule) fill_rule); break;
        default: sol = Xor(subj, clip, (FillRule) fill_rule); break;
        }
    }
    if (op != 1) {
        for (const Path64 &p : sol) flat.add(p);
        out.num_closed = (int32_t) sol.size();
    }
    out.total_points = (int32_t) (flat.coords.size() / 3);
    out.coords = copy_out(flat.coords);
    out.path_lens = copy_out(flat.lens);
    out.tree_parent = copy_out(parents);
    out.tree_is_hole = copy_out(holes);
    return out;
}

// Clipper2Utils.cpp:82-88 (libslic3r), word for word.
static void SimplifyPolyTree(const PolyPath64 &polytree, double epsilon, PolyPath64 &result) {
    for (const auto &child : polytree) {
        PolyPath64 *newchild = result.AddChild(SimplifyPath(child->Polygon(), epsilon));
        SimplifyPolyTree(*child, epsilon, *newchild);
    }
}

// What the functions of libslic3r's Clipper2Utils.cpp call, on the pin, for
// `clipper2_utils_native_parity` of `crates/libslic3r-rs`:
// op 0: union_ex_2 (Clipper2Utils.cpp:143-156 and 158-172): the tree of
//       Clipper64::Execute(Union, NonZero, tree) of the paths as subjects;
//    1: offset_ex_2 (:174-184): the tree of ClipperOffset::Execute(delta1, tree) of the
//       paths added with JoinType::Round, EndType::Polygon;
//    2: offset2_ex_2 (:186-208): the same, SimplifyPolyTree(tree, SCALED_EPSILON = 10, tree2),
//       Clear(), AddPaths(PolyTreeToPaths64(tree2), Round, Polygon), Execute(delta2, tree);
//    3, 4: _clipper2_pl_open (:119-136) with Intersection, Difference: the paths as open
//       subjects, the second set as clips; the closed paths, then the open ones.
extern "C" Cz2RefResult CZ2_REF_NAME(utils)(int32_t op, const int64_t *xyz, const int32_t *lens, int32_t num,
                                            const int64_t *clip_xyz, const int32_t *clip_lens, int32_t clip_num,
                                            double delta1, double delta2) {
    Paths64 subject = read_paths(xyz, lens, num);
    Paths64 clip = read_paths(clip_xyz, clip_lens, clip_num);
    Flat flat;
    std::vector<int32_t> parents, holes;
    Cz2RefResult out;
    out.num_open = 0;
    out.num_closed = 0;
    out.succeeded = 1;
    out.cb_calls = 0;
    out.cb_hash = 0;
    if (op == 0) {
        Clipper64 c;
        c.AddSubject(subject);
        ClipType ct = ClipType::Union;
        FillRule fr = FillRule::NonZero;
        PolyTree64 solution;
        c.Execute(ct, fr, solution);
        walk(solution, -1, flat, parents, holes);
        out.num_closed = (int32_t) parents.size();
    } else if (op == 1) {
        ClipperOffset offsetter;
        offsetter.AddPaths(subject, JoinType::Round, EndType::Polygon);
        PolyPath64 polytree;
        offsetter.Execute(delta1, polytree);
        walk(polytree, -1, flat, parents, holes);
        out.num_closed = (int32_t) parents.size();
    } else if (op == 2) {
        // 1st offset
        ClipperOffset offsetter;
        offsetter.AddPaths(subject, JoinType::Round, EndType::Polygon);
        PolyPath64 polytree;
        offsetter.Execute(delta1, polytree);

        // simplify the result
        PolyPath64 polytree2;
        SimplifyPolyTree(polytree, 10.0, polytree2);

        // 2nd offset
        offsetter.Clear();
        offsetter.AddPaths(PolyTreeToPaths64(polytree2), JoinType::Round, EndType::Polygon);
        polytree.Clear();
        offsetter.Execute(delta2, polytree);
        walk(polytree, -1, flat, parents, holes);
        out.num_closed = (int32_t) parents.size();
    } else {
        Clipper64 c;
        c.AddOpenSubject(subject);
        c.AddClip(clip);
        ClipType ct = op == 3 ? ClipType::Intersection : ClipType::Difference;
        FillRule fr = FillRule::NonZero;
        Paths64 solution, solution_open;
        c.Execute(ct, fr, solution, solution_open);
        for (const Path64 &p : solution) flat.add(p);
        for (const Path64 &p : solution_open) flat.add(p);
        out.num_closed = (int32_t) solution.size();
        out.num_open = (int32_t) solution_open.size();
    }
    out.total_points = (int32_t) (flat.coords.size() / 3);
    out.coords = copy_out(flat.coords);
    out.path_lens = copy_out(flat.lens);
    out.tree_parent = copy_out(parents);
    out.tree_is_hole = copy_out(holes);
    return out;
}

// std::sin(x) and std::cos(x) of one argument, as ClipperOffset::DoGroupOffset and Ellipse
// take them (out[0], out[1]), and each from a call the optimizer cannot pair with the other
// (out[2], out[3]).
__attribute__((noinline)) static double cz2_ref_sin_alone(volatile double x) { return std::sin(x); }
__attribute__((noinline)) static double cz2_ref_cos_alone(volatile double x) { return std::cos(x); }
extern "C" void CZ2_REF_NAME(sin_cos)(double x, double *out) {
    out[0] = std::sin(x);
    out[1] = std::cos(x);
    out[2] = cz2_ref_sin_alone(x);
    out[3] = cz2_ref_cos_alone(x);
}

// 1 where the reference is built from the engine's pin, 0 where from the vendored copy.
extern "C" int32_t CZ2_REF_NAME(from_pin)(void) {
    return CZ2_REF_FROM_PIN;
}

// CLIPPER2_VERSION of the sources the reference is built from.
extern "C" const char *CZ2_REF_NAME(version)(void) {
    return CLIPPER2_VERSION;
}
