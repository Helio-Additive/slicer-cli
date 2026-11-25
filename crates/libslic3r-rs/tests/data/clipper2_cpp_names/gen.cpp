// Native vectors for tests/clipper2_cpp_names_native_parity.rs: the pin's Clipper2 (src/clipper2,
// clipper.h and clipper.core.h, as libClipper2.a and the headers build them) under the C++ names,
// on deterministic random paths (1,000 calls per Path64 group, 200 per double group). clipper.core.h: GetBounds (Path64, Paths64, Path64 -> RectD),
// StripNearEqual, StripDuplicates (path and paths), CrossProduct and DotProduct (three points, two
// vectors), Area (path, paths), GetSegmentIntersectPt, Rect64 Width / Height / Contains (point,
// rect). clipper.h: BooleanOp, Intersect, Union (two overloads), Difference, Xor, InflatePaths,
// TranslatePath(s), RectClip and RectClipLines (paths and path), TrimCollinear, Ellipse,
// RamerDouglasPeucker (path, paths), SimplifyPath, Distance, Length, NearCollinear.
//
// Build: target/parity-tools/build.sh <this file> <output binary>; run: <binary> > vectors.txt
// Format: one call per line: a tag, the arguments, `->`, the result. Integers in decimal, a double
// as its 16 hex digits. A path is its point count then x y pairs; paths are their count then paths.
#include <cinttypes>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include "clipper2/clipper.h"

using namespace Clipper2Lib;

static uint64_t g_state = 0x2545F4914F6CDD1DULL;
static uint64_t xs() {
    g_state ^= g_state << 13;
    g_state ^= g_state >> 7;
    g_state ^= g_state << 17;
    return g_state;
}
static int64_t between(int64_t lo, int64_t hi) { return lo + int64_t(xs() % uint64_t(hi - lo + 1)); }
static double unit() { return double(xs() >> 11) / 9007199254740992.0; }

static void pd(double v) { uint64_t u; std::memcpy(&u, &v, 8); std::printf(" %016" PRIx64, u); }
static void pi(int64_t v) { std::printf(" %" PRId64, v); }
static void ppath(const Path64 &p) { pi(int64_t(p.size())); for (const Point64 &q : p) { pi(q.x); pi(q.y); } }
static void ppaths(const Paths64 &ps) { pi(int64_t(ps.size())); for (const Path64 &p : ps) ppath(p); }
static void ppathd(const PathD &p) { pi(int64_t(p.size())); for (const PointD &q : p) { pd(q.x); pd(q.y); } }
static void prect(const Rect64 &r) { pi(r.left); pi(r.top); pi(r.right); pi(r.bottom); }
static void prectd(const RectD &r) { pd(r.left); pd(r.top); pd(r.right); pd(r.bottom); }
static void arrow() { std::printf(" ->"); }
static void nl() { std::printf("\n"); }

static Path64 rnd_path(int nmin, int nmax, int64_t m) {
    int n = int(between(nmin, nmax));
    Path64 p;
    if (xs() % 2) {
        // a star around a centre: mostly simple polygons
        int64_t cx = between(-m, m), cy = between(-m, m);
        double r0 = double(m) * (0.2 + unit());
        for (int k = 0; k < n; ++k) {
            double t = 6.283185307179586 * k / n, r = r0 * (0.3 + 0.7 * unit());
            p.emplace_back(cx + int64_t(r * std::cos(t)), cy + int64_t(r * std::sin(t)));
        }
    } else {
        for (int k = 0; k < n; ++k) p.emplace_back(between(-m, m), between(-m, m));
    }
    // repeated and collinear points now and then
    if (n > 2 && xs() % 4 == 0) p.insert(p.begin() + 1, p[1]);
    if (n > 2 && xs() % 4 == 0) p.insert(p.begin() + 2, Point64((p[1].x + p[2].x) / 2, (p[1].y + p[2].y) / 2));
    return p;
}

static Paths64 rnd_paths(int64_t m) {
    Paths64 ps;
    int n = int(between(1, 3));
    for (int k = 0; k < n; ++k) ps.push_back(rnd_path(3, 10, m));
    return ps;
}

static Rect64 rnd_rect(int64_t m) {
    int64_t l = between(-m, m), t = between(-m, m);
    return Rect64(l, t, l + between(0, m), t + between(0, m));
}

int main() {
    const int N = 1000, ND = 200;
    for (int i = 0; i < N; ++i) {
        Path64 p = rnd_path(1, 12, 1000);
        Paths64 ps = rnd_paths(1000);
        std::printf("bounds"); ppath(p); ppaths(ps); arrow(); prect(GetBounds(p)); prect(GetBounds(ps)); prectd(GetBounds<double, int64_t>(p)); nl();
        double md = double(between(0, 2000));
        bool closed = xs() % 2;
        std::printf("strip_near_equal"); ppath(p); pd(md); pi(closed); arrow(); ppath(StripNearEqual(p, md, closed)); ppaths(StripNearEqual(ps, md, closed)); nl();
        Path64 q = p; Paths64 qs = ps;
        StripDuplicates(q, closed); StripDuplicates(qs, closed);
        std::printf("strip_duplicates"); ppath(p); pi(closed); arrow(); ppath(q); ppaths(qs); nl();
        std::printf("area"); ppath(p); arrow(); pd(Area(p)); pd(Area(ps)); nl();
        std::printf("length"); ppath(p); pi(closed); arrow(); pd(Length(p, closed)); nl();
    }
    for (int i = 0; i < N; ++i) {
        Point64 a(between(-100000, 100000), between(-100000, 100000)), b(between(-100000, 100000), between(-100000, 100000));
        Point64 c(between(-100000, 100000), between(-100000, 100000)), d(between(-100000, 100000), between(-100000, 100000));
        if (xs() % 5 == 0) d = Point64(a.x + (b.x - a.x) * 2, a.y + (b.y - a.y) * 2); // collinear
        PointD va(double(a.x) * 0.37, double(a.y) * 1.13), vb(double(b.x) * 0.71, double(b.y) * 0.29);
        Point64 ip(0, 0);
        bool hit = GetSegmentIntersectPt(a, b, c, d, ip);
        double ang = unit() * 0.01;
        std::printf("points"); pi(a.x); pi(a.y); pi(b.x); pi(b.y); pi(c.x); pi(c.y); pi(d.x); pi(d.y); pd(va.x); pd(va.y); pd(vb.x); pd(vb.y); pd(ang); arrow();
        pd(CrossProduct(a, b, c)); pd(CrossProduct(va, vb)); pd(DotProduct(a, b, c)); pd(DotProduct(va, vb));
        pi(hit); pi(ip.x); pi(ip.y); pd(Distance(a, b)); pi(NearCollinear(a, b, c, ang)); nl();
        Rect64 r = rnd_rect(100000), r2 = rnd_rect(100000);
        std::printf("rect"); prect(r); prect(r2); pi(c.x); pi(c.y); arrow(); pi(r.Width()); pi(r.Height()); pi(r.Contains(c)); pi(r.Contains(r2)); nl();
    }
    for (int i = 0; i < N; ++i) {
        Paths64 s = rnd_paths(1000), c = rnd_paths(1000);
        FillRule fr = FillRule(between(0, 3));
        ClipType ct = ClipType(between(1, 4));
        std::printf("boolean"); pi(int64_t(ct)); pi(int64_t(fr)); ppaths(s); ppaths(c); arrow();
        ppaths(BooleanOp(ct, fr, s, c)); ppaths(Intersect(s, c, fr)); ppaths(Union(s, c, fr)); ppaths(Union(s, fr)); ppaths(Difference(s, c, fr)); ppaths(Xor(s, c, fr)); nl();
        double delta = double(between(-200, 200)) + unit();
        JoinType jt = JoinType(between(0, 3));
        EndType et = EndType(between(0, 4));
        double ml = 1.0 + 3.0 * unit(), at = xs() % 2 ? 0.0 : 0.25 + 5.0 * unit();
        std::printf("inflate"); ppaths(s); pd(delta); pi(int64_t(jt)); pi(int64_t(et)); pd(ml); pd(at); arrow(); ppaths(InflatePaths(s, delta, jt, et, ml, at)); nl();
        int64_t dx = between(-5000, 5000), dy = between(-5000, 5000);
        std::printf("translate"); ppaths(s); pi(dx); pi(dy); arrow(); ppath(TranslatePath(s[0], dx, dy)); ppaths(TranslatePaths(s, dx, dy)); nl();
        Rect64 r = rnd_rect(1000);
        std::printf("rectclip"); prect(r); ppaths(s); arrow(); ppaths(RectClip(r, s)); ppaths(RectClip(r, s[0])); ppaths(RectClipLines(r, s)); ppaths(RectClipLines(r, s[0])); nl();
        bool open = xs() % 2;
        double eps = 1.0 + 100.0 * unit();
        std::printf("simplify"); ppaths(s); pi(open); pd(eps); arrow(); ppath(TrimCollinear(s[0], open)); ppath(RamerDouglasPeucker(s[0], eps)); ppaths(RamerDouglasPeucker(s, eps)); ppath(SimplifyPath(s[0], eps, !open)); nl();
        Point64 ctr(between(-1000, 1000), between(-1000, 1000));
        double rx = double(between(-5, 500)) + unit(), ry = xs() % 3 == 0 ? 0.0 : double(between(1, 500)) + unit();
        size_t steps = xs() % 2 ? 0 : size_t(between(3, 40));
        std::printf("ellipse"); pi(ctr.x); pi(ctr.y); pd(rx); pd(ry); pi(int64_t(steps)); arrow(); ppath(Ellipse(ctr, rx, ry, steps)); nl();
    }
    // The double overloads (PathD / PathsD / RectD / PointD) and MakePath / MakePathD / Ellipse(rect).
    for (int i = 0; i < ND; ++i) {
        Paths64 s64 = rnd_paths(1000), c64 = rnd_paths(1000);
        PathsD s, c;
        for (const Path64 &p : s64) { PathD q; for (const Point64 &pt : p) q.emplace_back(double(pt.x) * 0.013 + unit() * 1e-3, double(pt.y) * 0.013); s.push_back(q); }
        for (const Path64 &p : c64) { PathD q; for (const Point64 &pt : p) q.emplace_back(double(pt.x) * 0.013, double(pt.y) * 0.013 - unit() * 1e-3); c.push_back(q); }
        FillRule fr = FillRule(between(0, 3));
        ClipType ct = ClipType(between(1, 4));
        int prec = int(between(0, 4));
        std::printf("boolean_d"); pi(int64_t(ct)); pi(int64_t(fr)); pi(prec); pi(int64_t(s.size())); for (auto &p : s) ppathd(p); pi(int64_t(c.size())); for (auto &p : c) ppathd(p); arrow();
        auto ppsd = [](const PathsD &ps) { pi(int64_t(ps.size())); for (auto &p : ps) ppathd(p); };
        ppsd(BooleanOp(ct, fr, s, c, prec)); ppsd(Intersect(s, c, fr, prec)); ppsd(Union(s, c, fr, prec)); ppsd(Union(s, fr, prec)); ppsd(Difference(s, c, fr, prec)); ppsd(Xor(s, c, fr, prec)); nl();
        double delta = double(between(-3, 3)) + unit();
        JoinType jt = JoinType(between(0, 3));
        EndType et = EndType(between(0, 4));
        double ml = 1.0 + 3.0 * unit(), at = xs() % 2 ? 0.0 : 0.01 + 0.1 * unit();
        std::printf("inflate_d"); ppsd(s); pd(delta); pi(int64_t(jt)); pi(int64_t(et)); pd(ml); pi(prec); pd(at); arrow(); ppsd(InflatePaths(s, delta, jt, et, ml, prec, at)); nl();
        double dx = unit() * 10 - 5, dy = unit() * 10 - 5;
        std::printf("translate_d"); ppsd(s); pd(dx); pd(dy); arrow(); ppathd(TranslatePath(s[0], dx, dy)); ppsd(TranslatePaths(s, dx, dy)); nl();
        RectD r(unit() * 20 - 10, unit() * 20 - 10, 0, 0);
        r.right = r.left + unit() * 15; r.bottom = r.top + unit() * 15;
        std::printf("rectclip_d"); prectd(r); pi(prec); ppsd(s); arrow(); ppsd(RectClip(r, s, prec)); ppsd(RectClip(r, s[0], prec)); ppsd(RectClipLines(r, s, prec)); ppsd(RectClipLines(r, s[0], prec)); nl();
        bool open = xs() % 2;
        double eps = 0.01 + unit();
        std::printf("simplify_d"); ppsd(s); pi(prec); pi(open); pd(eps); arrow(); ppathd(TrimCollinear(s[0], prec, open)); ppathd(RamerDouglasPeucker(s[0], eps)); ppsd(RamerDouglasPeucker(s, eps)); ppathd(SimplifyPath(s[0], eps, !open)); pd(Length(s[0], !open)); nl();
        PointD ctr(unit() * 20 - 10, unit() * 20 - 10);
        double rx = unit() * 10 - 1, ry = xs() % 3 == 0 ? 0.0 : unit() * 10;
        size_t steps = xs() % 2 ? 0 : size_t(between(3, 40));
        Rect64 er = rnd_rect(1000);
        std::printf("ellipse_d"); pd(ctr.x); pd(ctr.y); pd(rx); pd(ry); pi(int64_t(steps)); prect(er); arrow(); ppathd(Ellipse(ctr, rx, ry, steps)); ppath(Ellipse(er, steps)); nl();
        std::vector<int64_t> li; std::vector<double> ld;
        int nn = int(between(0, 8)) * 2;
        for (int k = 0; k < nn; ++k) { li.push_back(between(-1000, 1000)); ld.push_back(unit() * 100 - 50); }
        std::printf("make_path"); pi(int64_t(li.size())); for (int64_t v : li) pi(v); pi(int64_t(ld.size())); for (double v : ld) pd(v); arrow(); ppath(MakePath(li)); ppathd(MakePathD(ld)); nl();
        // clipper.core.h with T = double
        PathD p = s[0];
        double md = unit() * 0.5;
        bool closed = xs() % 2;
        PathD q = p; PathsD qs = s;
        StripDuplicates(q, closed); StripDuplicates(qs, closed);
        std::printf("core_d"); ppsd(s); pd(md); pi(closed); arrow(); prectd(GetBounds(p)); prectd(GetBounds(s)); ppathd(StripNearEqual(p, md, closed)); ppsd(StripNearEqual(s, md, closed)); ppathd(q); ppsd(qs); pd(Area(p)); pd(Area(s)); nl();
        PointD a = p[0], b = p[p.size() > 1 ? 1 : 0], cc(unit() * 20 - 10, unit() * 20 - 10), d(unit() * 20 - 10, unit() * 20 - 10);
        PointD ip(0, 0);
        bool hit = GetSegmentIntersectPt(a, b, cc, d, ip);
        Point64 v1(between(-100000, 100000), between(-100000, 100000)), v2(between(-100000, 100000), between(-100000, 100000));
        std::printf("points_d"); pd(a.x); pd(a.y); pd(b.x); pd(b.y); pd(cc.x); pd(cc.y); pd(d.x); pd(d.y); pi(v1.x); pi(v1.y); pi(v2.x); pi(v2.y); arrow();
        pd(CrossProduct(a, b, cc)); pd(DotProduct(a, b, cc)); pd(CrossProduct(v1, v2)); pd(DotProduct(v1, v2)); pi(hit); pd(ip.x); pd(ip.y); pd(Distance(a, b)); nl();
        std::printf("rect_d"); prectd(r); pd(cc.x); pd(cc.y); arrow(); pd(r.Width()); pd(r.Height()); pi(r.Contains(cc)); nl();
    }
    return 0;
}
