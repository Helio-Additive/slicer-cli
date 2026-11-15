// Test-only reference exports, second part: what `clipper2_ref_shim.cpp` does not reach of
// the Clipper2 the engine pins (`references/BambuStudio/src/clipper2/Clipper2Lib`, see
// build.rs): clipper.rectclip.cpp, `ClipperD` and `PolyPathD`, `ReuseableDataContainer64`,
// the `PathsD` interface of clipper.h, clipper.minkowski.h and the templates of
// clipper.core.h on doubles, for the `clipper2_rectclip*_native_parity` and
// `clipper2_d*_native_parity` tests of `crates/libslic3r-rs`. Nothing in the engine calls
// these.
//
// Compiled twice, as clipper2_ref_shim.cpp is: `cz2_ref_z_ext` (USINGZ) and `cz2_ref_xy_ext`.
//
// One entry point. A call names an operation and gives two streams, of int64 and of double;
// the answer is two such streams. `tests/clipper2_ext_parity/mod.rs` writes the same
// streams from the Rust port, and the test compares them value for value (the doubles bit
// for bit). The encodings:
//   point64   x y z                      (ints; z = 0 in the build without Z)
//   path64    n, n points
//   paths64   n, n paths
//   pointD    x y (doubles) z (int)
//   pathD     n (int), n points
//   pathsD    n (int), n paths
//   rect64    left top right bottom (ints); rectD the same in doubles
// An exception of the library is the two ints -777 and its number (see `code_of`).
#include "clipper2/clipper.h"

#include "clipper2_ref_shim.h"

#include <cstdlib>
#include <cstring>
#include <sstream>
#include <string>
#include <vector>

using namespace CZ2_REF_NAMESPACE;

#ifdef USINGZ
#define CZ2_REF_NAME(name) cz2_ref_z_##name
#else
#define CZ2_REF_NAME(name) cz2_ref_xy_##name
#endif

// clipper.rectclip.cpp:77 and :430 have external linkage and no declaration in a header.
namespace CZ2_REF_NAMESPACE {
bool GetSegmentIntersection(const Point64 &p1, const Point64 &p2, const Point64 &p3, const Point64 &p4, Point64 &ip);
bool StartLocsAreClockwise(const std::vector<Location> &startlocs);
} // namespace CZ2_REF_NAMESPACE

namespace {

Point64 pt64(int64_t x, int64_t y, int64_t z) {
#ifdef USINGZ
    return Point64(x, y, z);
#else
    (void) z;
    return Point64(x, y);
#endif
}

PointD ptd(double x, double y, int64_t z) {
#ifdef USINGZ
    return PointD(x, y, z);
#else
    (void) z;
    return PointD(x, y);
#endif
}

template <typename P> int64_t zof(const P &p) {
#ifdef USINGZ
    return p.z;
#else
    (void) p;
    return 0;
#endif
}

struct In {
    const int64_t *ints;
    const double *dbls;
    int64_t i() { return *ints++; }
    double d() { return *dbls++; }
    Point64 point64() {
        int64_t x = i(), y = i(), z = i();
        return pt64(x, y, z);
    }
    PointD pointD() {
        double x = d(), y = d();
        int64_t z = i();
        return ptd(x, y, z);
    }
    Path64 path64() {
        int64_t n = i();
        Path64 p;
        p.reserve((size_t) n);
        for (int64_t k = 0; k < n; ++k) p.push_back(point64());
        return p;
    }
    Paths64 paths64() {
        int64_t n = i();
        Paths64 pp;
        for (int64_t k = 0; k < n; ++k) pp.push_back(path64());
        return pp;
    }
    PathD pathD() {
        int64_t n = i();
        PathD p;
        p.reserve((size_t) n);
        for (int64_t k = 0; k < n; ++k) p.push_back(pointD());
        return p;
    }
    PathsD pathsD() {
        int64_t n = i();
        PathsD pp;
        for (int64_t k = 0; k < n; ++k) pp.push_back(pathD());
        return pp;
    }
    Rect64 rect64() {
        int64_t l = i(), t = i(), r = i(), b = i();
        return Rect64(l, t, r, b);
    }
    RectD rectD() {
        double l = d(), t = d(), r = d(), b = d();
        return RectD(l, t, r, b);
    }
};

struct Out {
    std::vector<int64_t> ints;
    std::vector<double> dbls;
    void i(int64_t v) { ints.push_back(v); }
    void d(double v) { dbls.push_back(v); }
    void point64(const Point64 &p) {
        i(p.x);
        i(p.y);
        i(zof(p));
    }
    void pointD(const PointD &p) {
        d(p.x);
        d(p.y);
        i(zof(p));
    }
    void path64(const Path64 &p) {
        i((int64_t) p.size());
        for (const Point64 &q : p) point64(q);
    }
    void paths64(const Paths64 &pp) {
        i((int64_t) pp.size());
        for (const Path64 &p : pp) path64(p);
    }
    void pathD(const PathD &p) {
        i((int64_t) p.size());
        for (const PointD &q : p) pointD(q);
    }
    void pathsD(const PathsD &pp) {
        i((int64_t) pp.size());
        for (const PathD &p : pp) pathD(p);
    }
    void rect64(const Rect64 &r) {
        i(r.left);
        i(r.top);
        i(r.right);
        i(r.bottom);
    }
    void rectD(const RectD &r) {
        d(r.left);
        d(r.top);
        d(r.right);
        d(r.bottom);
    }
    void str(const std::string &s) {
        i((int64_t) s.size());
        for (unsigned char c : s) i(c);
    }
    void tree64(const PolyPath64 &node) {
        i((int64_t) node.Count());
        for (const auto &child : node) {
            i(child->IsHole() ? 1 : 0);
            i(child->Level());
            path64(child->Polygon());
            tree64(*child);
        }
    }
    void treeD(const PolyPathD &node) {
        i((int64_t) node.Count());
        for (const auto &child : node) {
            i(child->IsHole() ? 1 : 0);
            i(child->Level());
            d(child->Scale());
            pathD(child->Polygon());
            treeD(*child);
        }
    }
};

// The number of an exception: the error code of clipper.core.h:52-56 its text belongs to.
int64_t code_of(const std::exception &e) {
    const char *w = e.what();
    if (!std::strcmp(w, "Precision exceeds the permitted range")) return precision_error_i;
    if (!std::strcmp(w, "Invalid scale (either 0 or too large)")) return scale_error_i;
    if (!std::strcmp(w, "There must be 2 values for each coordinate")) return non_pair_error_i;
    if (!std::strcmp(w, "There is an undefined error in Clipper2")) return undefined_error_i;
    if (!std::strcmp(w, "Values exceed permitted range")) return range_error_i;
    return -1;
}

inline void mix(uint64_t &h, int64_t v) {
    h = (h ^ (uint64_t) v) * 0x100000001b3ULL;
}

inline int64_t bits(double v) {
    int64_t b;
    std::memcpy(&b, &v, sizeof b);
    return b;
}

void run(int32_t op, In &in, Out &out) {
    switch (op) {
    // ---- clipper.rectclip.cpp and its wrappers of clipper.h ----
    case 1: { // RectClip(rect, paths)
        Rect64 rect = in.rect64();
        Paths64 paths = in.paths64();
        out.paths64(RectClip(rect, paths));
        break;
    }
    case 2: { // RectClip(rect, path)
        Rect64 rect = in.rect64();
        Path64 path = in.path64();
        out.paths64(RectClip(rect, path));
        break;
    }
    case 3: { // RectClipLines(rect, lines)
        Rect64 rect = in.rect64();
        Paths64 paths = in.paths64();
        out.paths64(RectClipLines(rect, paths));
        break;
    }
    case 4: { // RectClipLines(rect, line)
        Rect64 rect = in.rect64();
        Path64 path = in.path64();
        out.paths64(RectClipLines(rect, path));
        break;
    }
    case 5: { // one RectClip64, Execute twice
        Rect64 rect = in.rect64();
        Paths64 a = in.paths64();
        Paths64 b = in.paths64();
        RectClip64 rc(rect);
        out.paths64(rc.Execute(a));
        out.paths64(rc.Execute(b));
        break;
    }
    case 6: { // one RectClipLines64, Execute twice
        Rect64 rect = in.rect64();
        Paths64 a = in.paths64();
        Paths64 b = in.paths64();
        RectClipLines64 rc(rect);
        out.paths64(rc.Execute(a));
        out.paths64(rc.Execute(b));
        break;
    }
    case 7:   // RectClip(RectD, PathsD or PathD, precision)
    case 8: { // RectClipLines(RectD, PathsD or PathD, precision)
        int precision = (int) in.i();
        int64_t single = in.i();
        RectD rect = in.rectD();
        PathsD paths = in.pathsD();
        if (single) {
            PathD first = paths.empty() ? PathD() : paths[0];
            out.pathsD(op == 7 ? RectClip(rect, first, precision) : RectClipLines(rect, first, precision));
        } else
            out.pathsD(op == 7 ? RectClip(rect, paths, precision) : RectClipLines(rect, paths, precision));
        break;
    }
    case 9: { // GetSegmentIntersection
        Point64 p1 = in.point64(), p2 = in.point64(), p3 = in.point64(), p4 = in.point64();
        Point64 ip = in.point64();
        bool res = GetSegmentIntersection(p1, p2, p3, p4, ip);
        out.i(res ? 1 : 0);
        out.point64(ip);
        break;
    }
    case 10: { // StartLocsAreClockwise
        int64_t n = in.i();
        std::vector<Location> locs;
        for (int64_t k = 0; k < n; ++k) locs.push_back((Location) in.i());
        out.i(StartLocsAreClockwise(locs) ? 1 : 0);
        break;
    }

    // ---- ClipperD ----
    case 20: {
        int precision = (int) in.i();
        int64_t clip_type = in.i(), fill_rule = in.i();
        int64_t preserve_collinear = in.i(), reverse_solution = in.i();
        int64_t mode = in.i(), zcb = in.i(), default_z = in.i();
        int64_t execs = in.i(), clip_type2 = in.i(), fill_rule2 = in.i();
        PathsD subj = in.pathsD(), open = in.pathsD(), clip = in.pathsD();
        uint64_t cb_calls = 0, cb_hash = 0xcbf29ce484222325ULL;
        ClipperD clipper(precision);
        clipper.PreserveCollinear(preserve_collinear != 0);
        clipper.ReverseSolution(reverse_solution != 0);
        out.i(clipper.PreserveCollinear() ? 1 : 0);
        out.i(clipper.ReverseSolution() ? 1 : 0);
#ifdef USINGZ
        clipper.DefaultZ = default_z;
        if (zcb == 1) {
            clipper.SetZCallback([&](const PointD &e1bot, const PointD &e1top, const PointD &e2bot,
                                     const PointD &e2top, PointD &pt) {
                ++cb_calls;
                uint64_t h = 0xcbf29ce484222325ULL;
                mix(h, (int64_t) cb_calls);
                for (const PointD &p : {e1bot, e1top, e2bot, e2top, pt}) {
                    mix(h, bits(p.x));
                    mix(h, bits(p.y));
                    mix(h, p.z);
                }
                pt.z = (int64_t) (h >> 1);
                mix(cb_hash, pt.z);
            });
        }
#else
        (void) zcb;
        (void) default_z;
#endif
        for (int kind = 0; kind < 3; ++kind) {
            try {
                if (kind == 0) clipper.AddSubject(subj);
                else if (kind == 1) clipper.AddOpenSubject(open);
                else clipper.AddClip(clip);
            } catch (const Clipper2Exception &e) {
                out.i(-778);
                out.i(code_of(e));
            }
            out.i(clipper.ErrorCode());
        }
        for (int64_t e = 0; e < execs; ++e) {
            ClipType ct = (ClipType) (e == 0 ? clip_type : clip_type2);
            FillRule fr = (FillRule) (e == 0 ? fill_rule : fill_rule2);
            if (mode < 2) {
                PathsD closed, opened;
                bool ok = mode == 0 ? clipper.Execute(ct, fr, closed) : clipper.Execute(ct, fr, closed, opened);
                out.i(ok ? 1 : 0);
                out.pathsD(closed);
                out.pathsD(opened);
            } else {
                PolyTreeD tree;
                PathsD opened;
                bool ok = mode == 2 ? clipper.Execute(ct, fr, tree) : clipper.Execute(ct, fr, tree, opened);
                out.i(ok ? 1 : 0);
                out.treeD(tree);
                out.pathsD(opened);
                out.pathsD(PolyTreeToPathsD(tree));
                out.d(tree.Area());
                out.d(tree.Scale());
                std::ostringstream os;
                os << tree;
                out.str(os.str());
            }
            out.i(clipper.ErrorCode());
        }
        out.i((int64_t) cb_calls);
        out.i((int64_t) cb_hash);
        break;
    }

    // ---- the PathsD interface of clipper.h ----
    case 21: {
        int64_t which = in.i(), clip_type = in.i(), fill_rule = in.i();
        int precision = (int) in.i();
        PathsD subj = in.pathsD(), clip = in.pathsD();
        FillRule fr = (FillRule) fill_rule;
        switch (which) {
        case 0: out.pathsD(BooleanOp((ClipType) clip_type, fr, subj, clip, precision)); break;
        case 1: {
            PolyTreeD tree;
            tree.AddChild(PathD{ptd(1, 2, 3)}); // BooleanOp clears the tree first
            BooleanOp((ClipType) clip_type, fr, subj, clip, tree, precision);
            out.treeD(tree);
            break;
        }
        case 2: out.pathsD(Intersect(subj, clip, fr, precision)); break;
        case 3: out.pathsD(Union(subj, clip, fr, precision)); break;
        case 4: out.pathsD(Union(subj, fr, precision)); break;
        case 5: out.pathsD(Difference(subj, clip, fr, precision)); break;
        default: out.pathsD(Xor(subj, clip, fr, precision)); break;
        }
        break;
    }
    case 22: { // InflatePaths(PathsD)
        int64_t jt = in.i(), et = in.i();
        int precision = (int) in.i();
        double delta = in.d(), miter_limit = in.d(), arc_tolerance = in.d();
        PathsD paths = in.pathsD();
        out.pathsD(InflatePaths(paths, delta, (JoinType) jt, (EndType) et, miter_limit, precision, arc_tolerance));
        break;
    }
    case 23: { // helpers on PathD
        int64_t which = in.i(), flag = in.i();
        int precision = (int) in.i();
        double a = in.d(), b = in.d(), c = in.d(), d = in.d();
        PathsD paths = in.pathsD();
        PathD first = paths.empty() ? PathD() : paths[0];
        switch (which) {
        case 0: out.pathD(TranslatePath(first, a, b)); break;
        case 1: out.pathsD(TranslatePaths(paths, a, b)); break;
        case 2: out.pathD(TrimCollinear(first, precision, flag != 0)); break;
        case 3: out.d(Length<double>(first, flag != 0)); break;
        case 4: out.pathD(SimplifyPath<double>(first, a, flag != 0)); break;
        case 5: out.pathsD(SimplifyPaths<double>(paths, a, flag != 0)); break;
        case 6: out.pathD(RamerDouglasPeucker<double>(first, a)); break;
        case 7: out.pathsD(RamerDouglasPeucker<double>(paths, a)); break;
        case 8: out.pathD(Ellipse<double>(ptd(c, d, 0), a, b, (size_t) flag)); break;
        case 9: out.pathD(Ellipse<double>(RectD(a, b, c, d), (size_t) flag)); break;
        case 10:
            out.d(Area<double>(first));
            out.d(Area<double>(paths));
            out.i(IsPositive<double>(first) ? 1 : 0);
            break;
        case 11: out.i((int64_t) PointInPolygon<double>(ptd(a, b, 0), first)); break;
        case 12:
            out.rectD(GetBounds(first));
            out.rectD(GetBounds(paths));
            out.rect64(GetBounds<int64_t, double>(first));
            out.rect64(GetBounds<int64_t, double>(paths));
            out.rectD(GetBounds<double, double>(paths));
            break;
        case 13: {
            out.pathD(StripNearEqual<double>(first, a, flag != 0));
            out.pathsD(StripNearEqual<double>(paths, a, flag != 0));
            PathD p = first;
            StripDuplicates<double>(p, flag != 0);
            out.pathD(p);
            PathsD pp = paths;
            StripDuplicates<double>(pp, flag != 0);
            out.pathsD(pp);
            break;
        }
        default: { // 14: the point functions, on the first four points of the first path
            if (first.size() < 4) break;
            const PointD &p0 = first[0], &p1 = first[1], &p2 = first[2], &p3 = first[3];
            out.i(NearCollinear(p0, p1, p2, a) ? 1 : 0);
            out.d(Distance(p0, p1));
            out.d(DistanceSqr(p0, p1));
            out.d(CrossProduct(p0, p1, p2));
            out.d(DotProduct(p0, p1, p2));
            out.d(CrossProduct(p0, p1));
            out.d(DotProduct(p0, p1));
            out.d(PerpendicDistFromLineSqrd(p0, p1, p2));
            out.pointD(MidPoint(p0, p1));
            out.i(NearEqual(p0, p1, a) ? 1 : 0);
            out.pointD(GetClosestPointOnSegment(p0, p1, p2));
            PointD ip = p3;
            out.i(GetSegmentIntersectPt(p0, p1, p2, p3, ip) ? 1 : 0);
            out.pointD(ip);
            out.pointD(TranslatePoint(p0, a, b));
            out.pointD(ReflectPoint(p0, p1));
            out.pointD(p0 * a);
            out.pointD(-p0);
            out.pointD(p0 + p1);
            out.pointD(p0 - p1);
            out.point64(Point64(p0));
            out.i(p0 == p1 ? 1 : 0);
#ifdef USINGZ
            out.point64(Point64(p0, 77));
#else
            out.point64(Point64(p0));
#endif
            PointD q = p0;
            q.Negate();
            out.pointD(q);
            break;
        }
        }
        break;
    }
    case 24: { // helpers on Path64 that clipper2_ref_shim.cpp does not reach
        int64_t which = in.i(), flag = in.i();
        double a = in.d(), b = in.d();
        Paths64 paths = in.paths64();
        Path64 first = paths.empty() ? Path64() : paths[0];
        switch (which) {
        case 0: {
            if (first.size() < 4) break;
            const Point64 &p0 = first[0], &p1 = first[1], &p2 = first[2], &p3 = first[3];
            out.i(NearEqual(p0, p1, a) ? 1 : 0);
            out.point64(TranslatePoint(p0, a, b));
            out.point64(ReflectPoint(p0, p1));
            out.d(CrossProduct(p0, p1));
            out.d(DotProduct(p0, p1));
            out.point64(p0 * a);
            out.pointD(PointD(p0));
#ifdef USINGZ
            out.pointD(PointD(p0, 77));
#else
            out.pointD(PointD(p0));
#endif
            Rect64 r(p0.x, p0.y, p2.x, p2.y);
            out.path64(Ellipse<int64_t>(r, (size_t) flag));
            (void) p3;
            break;
        }
        case 1:
            out.path64(StripNearEqual<int64_t>(first, a, flag != 0));
            out.paths64(StripNearEqual<int64_t>(paths, a, flag != 0));
            break;
        default:
            out.rectD(GetBounds<double, int64_t>(first));
            out.rectD(GetBounds<double, int64_t>(paths));
            break;
        }
        break;
    }
    case 25: { // ScalePath and ScalePaths
        int64_t variant = in.i(), uniform = in.i(), whole = in.i();
        double sx = in.d(), sy = in.d();
        int ec = 0;
        try {
            if (variant == 0 || variant == 3) {
                PathsD paths = in.pathsD();
                PathD first = paths.empty() ? PathD() : paths[0];
                if (variant == 0) {
                    if (whole) out.paths64(uniform ? ScalePaths<int64_t, double>(paths, sx, ec) : ScalePaths<int64_t, double>(paths, sx, sy, ec));
                    else out.path64(uniform ? ScalePath<int64_t, double>(first, sx, ec) : ScalePath<int64_t, double>(first, sx, sy, ec));
                } else {
                    if (whole) out.pathsD(uniform ? ScalePaths<double, double>(paths, sx, ec) : ScalePaths<double, double>(paths, sx, sy, ec));
                    else out.pathD(uniform ? ScalePath<double, double>(first, sx, ec) : ScalePath<double, double>(first, sx, sy, ec));
                }
            } else {
                Paths64 paths = in.paths64();
                Path64 first = paths.empty() ? Path64() : paths[0];
                if (variant == 1) {
                    if (whole) out.pathsD(uniform ? ScalePaths<double, int64_t>(paths, sx, ec) : ScalePaths<double, int64_t>(paths, sx, sy, ec));
                    else out.pathD(uniform ? ScalePath<double, int64_t>(first, sx, ec) : ScalePath<double, int64_t>(first, sx, sy, ec));
                } else {
                    if (whole) out.paths64(uniform ? ScalePaths<int64_t, int64_t>(paths, sx, ec) : ScalePaths<int64_t, int64_t>(paths, sx, sy, ec));
                    else out.path64(uniform ? ScalePath<int64_t, int64_t>(first, sx, ec) : ScalePath<int64_t, int64_t>(first, sx, sy, ec));
                }
            }
        } catch (const Clipper2Exception &e) {
            out.i(-778);
            out.i(code_of(e));
        }
        out.i(ec);
        break;
    }
    case 26: { // TransformPath and TransformPaths
        int64_t variant = in.i();
        if (variant == 0) {
            PathsD paths = in.pathsD();
            out.path64(TransformPath<int64_t, double>(paths.empty() ? PathD() : paths[0]));
            out.paths64(TransformPaths<int64_t, double>(paths));
        } else {
            Paths64 paths = in.paths64();
            out.pathD(TransformPath<double, int64_t>(paths.empty() ? Path64() : paths[0]));
            out.pathsD(TransformPaths<double, int64_t>(paths));
        }
        break;
    }
    case 27: { // CheckPrecisionRange
        int precision = (int) in.i();
        int ec = (int) in.i();
        try {
            CheckPrecisionRange(precision, ec);
        } catch (const Clipper2Exception &e) {
            out.i(-778);
            out.i(code_of(e));
        }
        out.i(precision);
        out.i(ec);
        int p2 = (int) in.i();
        try {
            CheckPrecisionRange(p2);
        } catch (const Clipper2Exception &e) {
            out.i(-778);
            out.i(code_of(e));
        }
        out.i(p2);
        break;
    }
    case 28: { // Rect<double>, Rect<int64_t>::Scale, ScaleRect
        RectD r = in.rectD(), r2 = in.rectD();
        double a = in.d(), px = in.d(), py = in.d();
        Rect64 q = in.rect64();
        out.i(r.IsValid() ? 1 : 0);
        out.d(r.Width());
        out.d(r.Height());
        out.pointD(r.MidPoint());
        out.pathD(r.AsPath());
        out.i(r.Contains(ptd(px, py, 0)) ? 1 : 0);
        out.i(r.Contains(r2) ? 1 : 0);
        out.i(r.IsEmpty() ? 1 : 0);
        out.i(r.Intersects(r2) ? 1 : 0);
        out.i(r == r2 ? 1 : 0);
        out.rectD(r + r2);
        RectD s = r;
        s += r2;
        out.rectD(s);
        s = r;
        s.Scale(a);
        out.rectD(s);
        s = r;
        s.Width(a);
        s.Height(px);
        out.rectD(s);
        out.rectD(RectD(true));
        out.rectD(RectD(false));
        out.rectD(RectD::InvalidRect());
        out.i(RectD::InvalidRect().IsValid() ? 1 : 0);
        out.rectD(InvalidRectD);
        out.pointD(InvalidPointD);
        out.rect64(ScaleRect<int64_t, double>(r, a));
        out.rectD(ScaleRect<double, int64_t>(q, a));
        out.rect64(ScaleRect<int64_t, int64_t>(q, a));
        out.rectD(ScaleRect<double, double>(r, a));
        Rect64 qs = q;
        qs.Scale(a);
        out.rect64(qs);
        break;
    }

    // ---- clipper.minkowski.h ----
    case 30: {
        int64_t is_sum = in.i(), is_closed = in.i();
        Path64 pattern = in.path64(), path = in.path64();
        out.paths64(is_sum ? MinkowskiSum(pattern, path, is_closed != 0) : MinkowskiDiff(pattern, path, is_closed != 0));
        break;
    }
    case 31: {
        int64_t is_sum = in.i(), is_closed = in.i();
        int decimals = (int) in.i();
        PathD pattern = in.pathD(), path = in.pathD();
        out.pathsD(is_sum ? MinkowskiSum(pattern, path, is_closed != 0, decimals) : MinkowskiDiff(pattern, path, is_closed != 0, decimals));
        break;
    }

    // ---- MakePath ----
    case 32: {
        int64_t n = in.i();
        std::vector<int64_t> list;
        std::vector<double> listd;
        for (int64_t k = 0; k < n; ++k) list.push_back(in.i());
        for (int64_t k = 0; k < n; ++k) listd.push_back(in.d());
        try {
            out.path64(MakePath(list));
        } catch (const Clipper2Exception &e) {
            out.i(-778);
            out.i(code_of(e));
        }
        try {
            out.pathD(MakePathD(listd));
        } catch (const Clipper2Exception &e) {
            out.i(-778);
            out.i(code_of(e));
        }
        if (n >= 6) {
            int64_t arr[6];
            double arrd[6];
            for (int k = 0; k < 6; ++k) {
                arr[k] = list[k];
                arrd[k] = listd[k];
            }
            out.path64(MakePath(arr));
            out.pathD(MakePathD(arrd));
#ifdef USINGZ
            out.path64(MakePathZ(arr));
            out.pathD(MakePathZD(arrd));
#else
            // The functions exist under USINGZ only: what they are without the z.
            out.path64(Path64{Point64(arr[0], arr[1]), Point64(arr[3], arr[4])});
            out.pathD(PathD{PointD(arrd[0], arrd[1]), PointD(arrd[3], arrd[4])});
#endif
        }
        break;
    }

    // ---- ReuseableDataContainer64 ----
    case 33: {
        int64_t clip_type = in.i(), fill_rule = in.i(), clip_type2 = in.i(), fill_rule2 = in.i();
        int64_t use_tree = in.i(), mode = in.i();
        int64_t preserve_collinear = in.i(), reverse_solution = in.i();
        Paths64 subj = in.paths64(), open = in.paths64(), clip = in.paths64(), extra = in.paths64();
        auto exec = [&](Clipper64 &c, int64_t ct, int64_t fr) {
            if (use_tree) {
                PolyTree64 tree;
                Paths64 opened;
                out.i(c.Execute((ClipType) ct, (FillRule) fr, tree, opened) ? 1 : 0);
                out.tree64(tree);
                out.paths64(opened);
            } else {
                Paths64 closed, opened;
                out.i(c.Execute((ClipType) ct, (FillRule) fr, closed, opened) ? 1 : 0);
                out.paths64(closed);
                out.paths64(opened);
            }
        };
        // A container is given to a clipper once: its minima a second time are edges twice
        // on the same vertices, which the sweep does not come back from.
        ReuseableDataContainer64 rd, rd2;
        rd.AddPaths(subj, PathType::Subject, false);
        rd.AddPaths(open, PathType::Subject, true);
        if (mode & 1) rd2.AddPaths(clip, PathType::Clip, false);
        else rd.AddPaths(clip, PathType::Clip, false);
        Clipper64 c1;
        c1.PreserveCollinear(preserve_collinear != 0);
        c1.ReverseSolution(reverse_solution != 0);
        if (mode & 2) c1.AddSubject(extra);
        c1.AddReuseableData(rd);
        if (mode & 1) c1.AddReuseableData(rd2);
        if (mode & 4) c1.AddClip(extra);
        // the container grows after the first clipper took it
        if (mode & 8) rd.AddPaths(extra, PathType::Clip, false);
        exec(c1, clip_type, fill_rule);
        Clipper64 c2;
        c2.AddReuseableData(rd);
        if (mode & 1) c2.AddReuseableData(rd2);
        exec(c2, clip_type2, fill_rule2);
        if (mode & 16) {
            c1.Clear();
            c1.AddReuseableData(rd);
            exec(c1, clip_type2, fill_rule);
        }
        if (mode & 32) {
            // the container emptied and filled again, for a third clipper
            rd.Clear();
            rd.AddPaths(clip, PathType::Subject, false);
            rd.AddPaths(subj, PathType::Clip, false);
            Clipper64 c3;
            c3.AddReuseableData(rd);
            exec(c3, clip_type, fill_rule2);
        }
        break;
    }

    // ---- PolyPathD by hand ----
    case 34: {
        double a = in.d(), b = in.d();
        Paths64 p64 = in.paths64();
        PathsD pd = in.pathsD();
        PolyTreeD root;
        try {
            root.SetScale(a);
            PolyPathD *c1 = root.AddChild(p64[0]);
            PolyPathD *c2 = c1->AddChild(pd[0]);
            PolyPathD *c3 = c2->AddChild(p64[1]);
            root.AddChild(pd[1]);
            c3->SetScale(b);
            c3->AddChild(p64[0]);
            out.i(root[0]->Count());
            out.i(root.Child(1)->Count());
        } catch (const Clipper2Exception &e) {
            out.i(-778);
            out.i(code_of(e));
        }
        out.treeD(root);
        out.d(root.Area());
        out.d(PolyPathD::Clipper2LibArea<double>(pd[0]));
        out.d(PolyPath64::Clipper2LibArea<int64_t>(p64[0]));
        out.pathsD(PolyTreeToPathsD(root));
        std::ostringstream os;
        os << root;
        out.str(os.str());
        root.Clear();
        out.i(root.Count());
        break;
    }
    case 35: { // operator<<(PolyTree64)
        int64_t clip_type = in.i(), fill_rule = in.i();
        Paths64 subj = in.paths64(), clip = in.paths64();
        PolyTree64 tree;
        BooleanOp((ClipType) clip_type, (FillRule) fill_rule, subj, clip, tree);
        std::ostringstream os;
        os << tree;
        out.str(os.str());
        out.d(tree.Area());
        break;
    }
    default: out.i(-1); break;
    }
}

template <typename T> T *copy_out(const std::vector<T> &v) {
    if (v.empty()) return nullptr;
    T *out = (T *) std::malloc(sizeof(T) * v.size());
    std::copy(v.begin(), v.end(), out);
    return out;
}

} // namespace

extern "C" Cz2ExtResult CZ2_REF_NAME(ext)(int32_t op, const int64_t *ints, const double *dbls) {
    In in{ints, dbls};
    Out out;
    try {
        run(op, in, out);
    } catch (const Clipper2Exception &e) {
        out.ints.clear();
        out.dbls.clear();
        out.i(-777);
        out.i(code_of(e));
    }
    Cz2ExtResult r;
    r.ints = copy_out(out.ints);
    r.dbls = copy_out(out.dbls);
    r.n_ints = (int64_t) out.ints.size();
    r.n_dbls = (int64_t) out.dbls.size();
    return r;
}

extern "C" void CZ2_REF_NAME(ext_free)(Cz2ExtResult r) {
    std::free(r.ints);
    std::free(r.dbls);
}
