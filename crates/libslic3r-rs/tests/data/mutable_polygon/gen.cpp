// Generator of the vectors of tests/mutable_polygon_native_parity.rs: libslic3r's MutablePolygon.cpp and
// MutablePolygon.hpp at the pin (references/BambuStudio/src/libslic3r, the override layer has no copy).
//
// Build, with REPO = this repository, BUILD = a built tree of the Bambu Studio engine
// (docs/building.md, -DENGINE=bambu) and DATA = this directory: compile with the compile line the
// build uses for a libslic3r source (its defines and include directories, the override layer
// $REPO/libslic3r/bambustudio before the pin, -std=c++17 -O3 -DNDEBUG and the forced include of
// cgal_54_compat.hpp), then link with the build's libslic3r_core and the libraries it links:
//
//   c++ <the build's defines and include directories> -std=c++17 -O3 -DNDEBUG \
//       -include $REPO/libslic3r/bambustudio/cgal_54_compat.hpp -c $DATA/gen.cpp -o gen.o
//   c++ gen.o $BUILD/libslic3r_core.a <the libraries libslic3r_core links> -o gen
//   ./gen $DATA/vectors.txt     (then gzip -9n vectors.txt)
//
// MutablePolygon.cpp is compiled into this file (clip_narrow_corner is static), with the private members
// of MutablePolygon made public so that its storage can be written out. Made on macOS arm64 with Apple
// clang and libc++; the build is a release build (asserts compiled out), as the engine's.
//
// Every line is `<what and inputs> -> <results>`; tokens are separated by one space, integers decimal, a
// double its 16 hexadecimal digits. A polygon is its number of points and the points. A *state* of a
// MutablePolygon is
//   S <m_size> <m_head> <m_head_free> <m_data.size()> { <x> <y> <prev> <next> } E <empty()> <valid()>
//   B <cbegin> <cend> <begin> <end> <begin const> <end const> P <polygon()>
// with iterators as their index. The lines:
//   const -> scaled<double>(0.01), SCALED_EPSILON, sqr(SCALED_EPSILON)
//   ops <case> <operation> <arguments> -> <results> <state>
//       the container: constructors, assign (both), polygon (both), clear, insert and remove through the
//       polygon and through an iterator, the iterators (++, --, prev, next, *, ->, size, valid; const and
//       not), operator== / !=, and a `range` (process_next, advance_front, retract_back, remove_front,
//       remove_back); an iterator argument <k> is begin() advanced k times.
//   rd0 <polygon> -> <state>                               remove_duplicates(MutablePolygon&)
//   rd1 <eps> <polygon> -> <state>                         remove_duplicates(MutablePolygon&, double)
//   rd2 <scaled_eps> <max_angle> <polygon> -> <state>      remove_duplicates(MutablePolygon&, coord_t, double)
//   rd3 <scaled_eps> <max_angle> <expolygons> -> <expolygons>   remove_duplicates(ExPolygons, coord_t, double)
//   cnc <clip> <k> <polygon> -> <returned> <it0> <it2> <range begin> <range end> <state>
//       clip_narrow_corner at the corner k of the polygon, called as smooth_outward calls it: the range
//       advanced past k, the corner removed.
//   so <clip> <polygon> -> <state>                         smooth_outward(MutablePolygon&, coord_t)
//   sop <clip> <polygon> -> <polygon>                      smooth_outward(Polygon, coord_t)
//   sops <clip> <polygons> -> <polygons>                   smooth_outward(Polygons, coord_t)
//   soex <clip> <expolygons> -> <expolygons>               smooth_outward(ExPolygons, coord_t)
// Polygons are a count and the polygons; expolygons a count and, each, the number of holes, the contour
// and the holes.

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "libslic3r/libslic3r.h"
#include "libslic3r/Point.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/Line.hpp"

#define private public
#include "libslic3r/MutablePolygon.cpp"
#undef private

using namespace Slic3r;

static FILE       *g_out = nullptr;
static std::string g_line;

static void put(const char *fmt, ...)
{
    char    buf[512];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    g_line += buf;
}

static void endl_()
{
    fputs(g_line.c_str(), g_out);
    fputc('\n', g_out);
    g_line.clear();
}

static void put_double(double v)
{
    uint64_t bits;
    memcpy(&bits, &v, sizeof(bits));
    put(" %016llx", (unsigned long long) bits);
}

static void put_poly(const Polygon &p)
{
    put(" %zu", p.points.size());
    for (const Point &pt : p.points)
        put(" %d %d", int(pt.x()), int(pt.y()));
}

static void put_polys(const Polygons &ps)
{
    put(" %zu", ps.size());
    for (const Polygon &p : ps)
        put_poly(p);
}

static void put_expolys(const ExPolygons &es)
{
    put(" %zu", es.size());
    for (const ExPolygon &e : es) {
        put(" %zu", e.holes.size());
        put_poly(e.contour);
        for (const Polygon &h : e.holes)
            put_poly(h);
    }
}

static void put_state(MutablePolygon &mp)
{
    put(" S %d %d %d %zu", int(mp.m_size), int(mp.m_head), int(mp.m_head_free), mp.m_data.size());
    for (const auto &lp : mp.m_data)
        put(" %d %d %d %d", int(lp.point.x()), int(lp.point.y()), int(lp.prev), int(lp.next));
    put(" E %d %d", int(mp.empty()), int(mp.valid()));
    const MutablePolygon &cm = mp;
    put(" B %d %d %d %d %d %d", int(mp.cbegin().m_idx), int(mp.cend().m_idx), int(mp.begin().m_idx), int(mp.end().m_idx),
        int(cm.begin().m_idx), int(cm.end().m_idx));
    put(" P");
    put_poly(mp.polygon());
}

struct Lcg {
    uint64_t s;
    uint32_t next() {
        s = s * 6364136223846793005ULL + 1442695040888963407ULL;
        return uint32_t(s >> 33);
    }
    int    range(int lo, int hi) { return lo + int(next() % uint32_t(hi - lo + 1)); }
    double unit() { return next() / 2147483648.0; }
};

// The points of the loop, from begin().
static Polygon ring_of(MutablePolygon &mp)
{
    Polygon out;
    if (! mp.empty()) {
        auto it = mp.begin();
        for (size_t i = 0; i < mp.size(); ++ i, ++ it)
            out.points.emplace_back(*it);
    }
    return out;
}

// --------------------------------------------------------------------------------------------------
// Polygons
// --------------------------------------------------------------------------------------------------

// Steps of about the tolerance of smooth_outward's remove_duplicates (0.01 mm = 1000 scaled).
static const int g_steps[][2] = {
    { 1000, 0 }, { 0, 1000 }, { 600, 800 }, { -800, 600 }, { 999, 0 }, { 0, -999 }, { 999, 30 }, { 707, 707 }, { 708, 707 },
    { -1000, 0 }, { 1, 0 }, { 0, 0 }, { 500, -500 }, { 1001, 0 }, { -999, -44 }, { 999, 45 }, { 0, 0 }, { 30, -999 },
};

// Around a centre, the radius drawn anew for every point.
static Polygon make_star(Lcg &r, int n, double radius, double jitter)
{
    Polygon p;
    int     cx = r.range(-5000000, 5000000);
    int     cy = r.range(-5000000, 5000000);
    for (int i = 0; i < n; ++ i) {
        double a   = 2. * PI * (double(i) + jitter * (r.unit() - 0.5)) / double(n);
        double rad = radius * (0.25 + 0.75 * r.unit());
        p.points.emplace_back(coord_t(double(cx) + rad * cos(a)), coord_t(double(cy) + rad * sin(a)));
    }
    return p;
}

// A closed random walk.
static Polygon make_walk(Lcg &r, int n, int step)
{
    Polygon p;
    Point   c(r.range(-5000000, 5000000), r.range(-5000000, 5000000));
    for (int i = 0; i < n; ++ i) {
        c += Point(r.range(-step, step), r.range(-step, step));
        p.points.emplace_back(c);
    }
    return p;
}

// A rectangle with notches cut into its upper side: cracks of a width about the clipping distance, with walls
// that narrow and widen on the way down.
static Polygon make_notched(Lcg &r, int clip)
{
    Polygon p;
    int     ox      = r.range(-3000000, 3000000);
    int     oy      = r.range(-3000000, 3000000);
    int     notches = r.range(1, 3);
    int     width   = clip * r.range(8, 20) * notches;
    int     height  = clip * r.range(6, 20);
    p.points.emplace_back(ox, oy);
    p.points.emplace_back(ox + width, oy);
    p.points.emplace_back(ox + width, oy + height);
    // Along the upper side from the right to the left.
    for (int k = notches; k -- > 0;) {
        int xc     = ox + (width / notches) * k + width / (2 * notches) + r.range(-clip, clip);
        int levels = r.range(1, 5);
        std::vector<int> depth, left, right;
        int d = 0;
        for (int i = 0; i <= levels; ++ i) {
            int w = int(double(clip) * (0.02 + 1.6 * r.unit() * r.unit()));
            int s = r.range(-clip / 3, clip / 3);
            depth.emplace_back(d);
            left.emplace_back(xc + s - w / 2);
            right.emplace_back(xc + s + w / 2 + 1);
            d += int(double(clip) * (0.1 + 2.5 * r.unit()));
        }
        for (int i = 0; i <= levels; ++ i)
            p.points.emplace_back(right[i], oy + height - depth[i]);
        if (r.range(0, 2) == 0)
            // A pointed bottom.
            p.points.emplace_back(xc + r.range(-clip / 4, clip / 4), oy + height - d);
        for (int i = levels; i >= 0; -- i)
            p.points.emplace_back(left[i], oy + height - depth[i]);
    }
    p.points.emplace_back(ox, oy + height);
    return p;
}

// Points next to existing ones, at the steps of g_steps.
static void add_near_points(Lcg &r, Polygon &p, int count)
{
    for (int i = 0; i < count && ! p.points.empty(); ++ i) {
        size_t      at   = size_t(r.range(0, int(p.points.size()) - 1));
        const int  *step = g_steps[r.range(0, int(sizeof(g_steps) / sizeof(g_steps[0])) - 1)];
        Point       pt   = p.points[at] + Point(step[0], step[1]);
        p.points.insert(p.points.begin() + at + 1, pt);
    }
}

static const int g_clips[] = { 40000, 10000, 42000, 100000, 3000, 45000 };

// One polygon of the mix, for the clipping distance `clip`.
static Polygon make_polygon(Lcg &r, int clip)
{
    Polygon p;
    switch (r.range(0, 9)) {
    case 0: case 1: {
        static const double scales[] = { 0.5, 1., 2., 4., 10., 50. };
        p = make_star(r, r.range(3, 40), double(clip) * scales[r.range(0, 5)], r.range(0, 1) ? 0.9 : 0.);
        break;
    }
    case 2: case 3: {
        static const double steps[] = { 0.3, 0.8, 1.5, 4. };
        p = make_walk(r, r.range(3, 30), std::max(2, int(double(clip) * steps[r.range(0, 3)])));
        break;
    }
    case 4: case 5: case 6: case 7:
        p = make_notched(r, clip);
        break;
    case 8:
        // Triangles and quadrangles of about the clipping distance.
        p = make_star(r, r.range(3, 4), double(clip) * (0.2 + 3. * r.unit()), 0.9);
        break;
    default:
        // A star with a notched rectangle's points appended: self-intersecting, long and short edges mixed.
        p = make_star(r, r.range(5, 12), double(clip) * 6., 0.5);
        {
            Polygon q = make_walk(r, r.range(2, 6), clip / 2);
            Point   shift = p.points.back() - q.points.front();
            for (const Point &pt : q.points)
                p.points.emplace_back(pt + shift);
        }
        break;
    }
    if (r.range(0, 2) == 0)
        add_near_points(r, p, r.range(1, 4));
    if (r.range(0, 3) == 0)
        std::reverse(p.points.begin(), p.points.end());
    return p;
}

static ExPolygons make_expolygons(Lcg &r, int clip)
{
    ExPolygons out;
    int        n = r.range(0, 4);
    for (int i = 0; i < n; ++ i) {
        ExPolygon e;
        e.contour = make_polygon(r, clip);
        if (r.range(0, 5) == 0)
            e.contour.points.resize(r.range(0, 2));
        int holes = r.range(0, 3);
        for (int h = 0; h < holes; ++ h) {
            Polygon hole = make_polygon(r, clip);
            std::reverse(hole.points.begin(), hole.points.end());
            if (r.range(0, 4) == 0)
                hole.points.resize(r.range(0, 2));
            e.holes.emplace_back(std::move(hole));
        }
        out.emplace_back(std::move(e));
    }
    return out;
}

// --------------------------------------------------------------------------------------------------
// The container
// --------------------------------------------------------------------------------------------------

static Polygon small_polygon(Lcg &r)
{
    Polygon p;
    int     n = r.range(0, 9);
    for (int i = 0; i < n; ++ i)
        p.points.emplace_back(r.range(-1000000, 1000000), r.range(-1000000, 1000000));
    return p;
}

static void run_ops(Lcg &r)
{
    for (int c = 0; c < 160; ++ c) {
        MutablePolygon mp;
        put("ops %d new ->", c);
        put_state(mp);
        endl_();
        int nops = r.range(8, 40);
        for (int o = 0; o < nops; ++ o) {
            int what = r.range(0, 13);
            if (mp.empty() && what <= 5)
                // These need a point.
                what = 6 + what % 2;
            switch (what) {
            case 0: case 1: {
                int   k = r.range(0, 2 * int(mp.size()));
                Point pt(r.range(-1000000, 1000000), r.range(-1000000, 1000000));
                auto  it = mp.begin();
                for (int i = 0; i < k; ++ i)
                    ++ it;
                auto  res = what == 0 ? it.insert(pt) : mp.insert(it, pt);
                put("ops %d %s %d %d %d -> %d", c, what == 0 ? "insert_it" : "insert_mp", k, int(pt.x()), int(pt.y()), int(res.m_idx));
                break;
            }
            case 2: {
                int  k  = r.range(0, 2 * int(mp.size()));
                auto it = mp.begin();
                for (int i = 0; i < k; ++ i)
                    it ++;
                auto &res = it.remove();
                put("ops %d remove_it %d -> %d %d %d", c, k, int(it.m_idx), int(it.valid()), int(res.m_idx));
                break;
            }
            case 3: {
                int  k  = r.range(0, 2 * int(mp.size()));
                auto it = mp.begin();
                for (int i = 0; i < k; ++ i)
                    ++ it;
                auto res = mp.remove(it);
                put("ops %d remove_mp %d -> %d %d", c, k, int(res.m_idx), int(res.valid()));
                break;
            }
            case 4: {
                int  k  = r.range(0, 2 * int(mp.size()));
                int  j  = r.range(0, 2 * int(mp.size()));
                put("ops %d walk %d %d ->", c, k, j);
                auto it = mp.begin();
                for (int i = 0; i < k; ++ i)
                    if (i & 1) ++ it; else it ++;
                put(" %d %d %d %d %d %d %d", int(it.m_idx), int(it.prev().m_idx), int(it.next().m_idx), int((*it).x()), int(it->y()),
                    int(it.size()), int(it.valid()));
                for (int i = 0; i < j; ++ i)
                    if (i & 1) -- it; else it --;
                put(" %d %d %d", int(it.m_idx), int(it == mp.begin()), int(it != mp.end()));
                const MutablePolygon &cm = mp;
                auto cit = cm.begin();
                for (int i = 0; i < k; ++ i)
                    if (i & 1) ++ cit; else cit ++;
                put(" %d %d %d %d %d %d %d", int(cit.m_idx), int(cit.prev().m_idx), int(cit.next().m_idx), int((*cit).x()), int(cit->y()),
                    int(cit.size()), int(cit.valid()));
                for (int i = 0; i < j; ++ i)
                    if (i & 1) -- cit; else cit --;
                put(" %d %d %d", int(cit.m_idx), int(cit == cm.begin()), int(cit != cm.end()));
                break;
            }
            case 5: {
                MutablePolygon::range rg(mp);
                put("ops %d range_new -> %d %d %d", c, int(rg.begin().m_idx), int(rg.end().m_idx), int(rg.empty()));
                put_state(mp);
                endl_();
                int steps = r.range(1, 2 * int(mp.size()) + 2);
                for (int s = 0; s < steps && ! mp.empty(); ++ s) {
                    int sub = r.range(0, 4);
                    if (rg.empty())
                        sub = 3 + sub % 2;
                    if (sub == 0) {
                        auto res = rg.process_next();
                        put("ops %d range_process_next -> %d", c, int(res.m_idx));
                    } else if (sub == 1) {
                        rg.advance_front();
                        put("ops %d range_advance_front ->", c);
                    } else if (sub == 2) {
                        rg.retract_back();
                        put("ops %d range_retract_back ->", c);
                    } else {
                        int  k  = r.range(0, int(mp.size()) - 1);
                        auto it = mp.begin();
                        for (int i = 0; i < k; ++ i)
                            ++ it;
                        auto res = sub == 3 ? rg.remove_front(it) : rg.remove_back(it);
                        put("ops %d %s %d -> %d", c, sub == 3 ? "range_remove_front" : "range_remove_back", k, int(res.m_idx));
                    }
                    put(" %d %d %d", int(rg.begin().m_idx), int(rg.end().m_idx), int(rg.empty()));
                    put_state(mp);
                    endl_();
                }
                put("ops %d range_done ->", c);
                break;
            }
            case 6: case 7: {
                Polygon p       = small_polygon(r);
                int     reserve = r.range(0, 30);
                put("ops %d %s %d", c, what == 6 ? "assign_poly" : "assign_range", reserve);
                put_poly(p);
                put(" ->");
                if (what == 6)
                    mp.assign(p, size_t(reserve));
                else
                    mp.assign(p.points.begin(), p.points.end(), size_t(reserve));
                break;
            }
            case 8:
                mp.clear();
                put("ops %d clear ->", c);
                break;
            case 9: {
                Polygon out;
                out.points = { Point(1, 2), Point(3, 4) };
                mp.polygon(out);
                put("ops %d polygon_out ->", c);
                put_poly(out);
                break;
            }
            case 10: {
                Polygon p   = ring_of(mp);
                int     how = r.range(0, 4);
                if (how == 1 && ! p.points.empty())
                    std::rotate(p.points.begin(), p.points.begin() + 1, p.points.end());
                else if (how == 2 && ! p.points.empty())
                    p.points[size_t(r.range(0, int(p.points.size()) - 1))].x() += 1;
                else if (how == 3)
                    p = small_polygon(r);
                else if (how == 4 && ! p.points.empty())
                    p.points.pop_back();
                MutablePolygon other(p, size_t(r.range(0, 5)));
                put("ops %d eq", c);
                put_poly(p);
                put(" -> %d %d %d %d", int(mp == other), int(mp != other), int(other == mp), int(other != mp));
                break;
            }
            case 11: {
                Polygon        p       = small_polygon(r);
                int            reserve = r.range(0, 30);
                MutablePolygon m2(p, size_t(reserve));
                mp = m2;
                put("ops %d ctor_poly %d", c, reserve);
                put_poly(p);
                put(" ->");
                break;
            }
            case 12: {
                Point          a(r.range(-9, 9), r.range(-9, 9)), b(r.range(-9, 9), r.range(-9, 9)), d(r.range(-9, 9), r.range(-9, 9));
                int            reserve = r.range(0, 9);
                MutablePolygon m2({ a, b, d }, size_t(reserve));
                mp = m2;
                put("ops %d ctor_list %d 3 %d %d %d %d %d %d ->", c, reserve, int(a.x()), int(a.y()), int(b.x()), int(b.y()), int(d.x()), int(d.y()));
                break;
            }
            default: {
                Polygon        p       = small_polygon(r);
                int            reserve = r.range(0, 30);
                MutablePolygon m2(p.points.begin(), p.points.end(), size_t(reserve));
                mp = m2;
                put("ops %d ctor_range %d", c, reserve);
                put_poly(p);
                put(" ->");
                break;
            }
            }
            put_state(mp);
            endl_();
        }
    }
}

// --------------------------------------------------------------------------------------------------
// remove_duplicates
// --------------------------------------------------------------------------------------------------

static void run_remove_duplicates(Lcg &r)
{
    // remove_duplicates(MutablePolygon&)
    for (int c = 0; c < 150; ++ c) {
        Polygon p = c % 3 == 0 ? small_polygon(r) : make_polygon(r, g_clips[c % 6]);
        // Exact duplicates, runs of them and at the seam.
        int dups = r.range(0, 6);
        for (int i = 0; i < dups && ! p.points.empty(); ++ i) {
            size_t at = size_t(r.range(0, int(p.points.size()) - 1));
            p.points.insert(p.points.begin() + at, p.points[at]);
        }
        if (r.range(0, 3) == 0 && ! p.points.empty())
            p.points.emplace_back(p.points.front());
        if (c % 17 == 0 && ! p.points.empty())
            p.points.assign(size_t(r.range(1, 5)), p.points.front());
        MutablePolygon mp(p, p.size() * 2);
        remove_duplicates(mp);
        put("rd0");
        put_poly(p);
        put(" ->");
        put_state(mp);
        endl_();
    }
    // remove_duplicates(MutablePolygon&, double)
    const double epss[] = { scaled<double>(0.01), 0.5, 1000., 1500.25, scaled<double>(0.005), 999., 1e-9, 30000. };
    for (int c = 0; c < 400; ++ c) {
        Polygon p = c % 5 == 0 ? small_polygon(r) : make_polygon(r, g_clips[c % 6]);
        add_near_points(r, p, r.range(0, 8));
        if (c % 23 == 0 && ! p.points.empty()) {
            // All within the tolerance of each other.
            Point at = p.points.front();
            p.points.clear();
            int n = r.range(1, 6);
            for (int i = 0; i < n; ++ i)
                p.points.emplace_back(at + Point(r.range(-200, 200), r.range(-200, 200)));
        }
        double         eps = epss[c % 2 == 0 ? 0 : r.range(0, 7)];
        MutablePolygon mp(p, p.size() * 2);
        remove_duplicates(mp, eps);
        put("rd1");
        put_double(eps);
        put_poly(p);
        put(" ->");
        put_state(mp);
        endl_();
    }
    // remove_duplicates(MutablePolygon&, coord_t, double)
    const double angles[] = { PI / 6., PI / 4., 0.1, PI / 3., 1.4, 0. };
    const int    lens[]   = { 1000, 5000, 20000, 60000, 300000 };
    for (int c = 0; c < 400; ++ c) {
        Polygon p = c % 7 == 0 ? small_polygon(r) : make_polygon(r, g_clips[c % 6]);
        add_near_points(r, p, r.range(0, 6));
        int            scaled_eps = lens[r.range(0, 4)];
        double         max_angle  = angles[r.range(0, 5)];
        MutablePolygon mp(p, p.size() * 2);
        remove_duplicates(mp, coord_t(scaled_eps), max_angle);
        put("rd2 %d", scaled_eps);
        put_double(max_angle);
        put_poly(p);
        put(" ->");
        put_state(mp);
        endl_();
    }
    // remove_duplicates(ExPolygons, coord_t, double)
    for (int c = 0; c < 120; ++ c) {
        ExPolygons es         = make_expolygons(r, g_clips[c % 6]);
        int        scaled_eps = lens[r.range(0, 4)];
        double     max_angle  = angles[r.range(0, 5)];
        put("rd3 %d", scaled_eps);
        put_double(max_angle);
        put_expolys(es);
        put(" ->");
        put_expolys(remove_duplicates(es, coord_t(scaled_eps), max_angle));
        endl_();
    }
}

// --------------------------------------------------------------------------------------------------
// clip_narrow_corner, smooth_outward
// --------------------------------------------------------------------------------------------------

// clip_narrow_corner at the corners of `p` where smooth_outward (MutablePolygon.cpp:268-292) would call it on
// the untouched polygon; at most `limit` of them.
static int run_clip_narrow_corner(const Polygon &p, int clip, int limit)
{
    int       done = 0;
    const int n    = int(p.points.size());
    if (n < 3)
        return 0;
    const int64_t clip2 = sqr<int64_t>(clip);
    for (int k = 0; k < n && done < limit; ++ k) {
        const Point   p0 = p.points[size_t((k + n - 1) % n)];
        const Point   p1 = p.points[size_t(k)];
        const Point   p2 = p.points[size_t((k + 1) % n)];
        const Vec2i64 v1 = (p0 - p1).cast<int64_t>();
        const Vec2i64 v2 = (p2 - p1).cast<int64_t>();
        if (cross2(v1, v2) <= 0)
            continue;
        int64_t dot  = v1.dot(v2);
        auto    l2v1 = double(v1.squaredNorm());
        auto    l2v2 = double(v2.squaredNorm());
        if (! (dot > 0 || Slic3r::sqr(double(dot)) * 2. < l2v1 * l2v2))
            continue;
        int64_t l2v02 = (p2 - p0).cast<int64_t>().squaredNorm();
        if (l2v02 >= clip2)
            continue;
        MutablePolygon        mp(p, p.size() * 2);
        MutablePolygon::range rg(mp);
        MutablePolygon::iterator it1 = rg.process_next();
        for (int i = 0; i < k; ++ i)
            it1 = rg.process_next();
        auto it0 = it1.prev();
        auto it2 = it1.next();
        it1.remove();
        bool res = clip_narrow_corner(p1.cast<int64_t>(), it0, it2, rg, l2v02, clip);
        put("cnc %d %d", clip, k);
        put_poly(p);
        put(" -> %d %d %d %d %d", int(res), int(it0.m_idx), int(it2.m_idx), int(rg.begin().m_idx), int(rg.end().m_idx));
        put_state(mp);
        endl_();
        ++ done;
    }
    return done;
}

static void run_smooth_outward(Lcg &r)
{
    int corners = 0;
    for (int c = 0; c < 1700; ++ c) {
        int     clip = g_clips[c % 6];
        Polygon p    = make_polygon(r, clip);
        if (corners < 1200)
            corners += run_clip_narrow_corner(p, clip, 2);
        if (c % 2 == 0) {
            MutablePolygon mp(p, p.size() * 2);
            smooth_outward(mp, coord_t(clip));
            put("so %d", clip);
            put_poly(p);
            put(" ->");
            put_state(mp);
        } else {
            put("sop %d", clip);
            put_poly(p);
            put(" ->");
            put_poly(smooth_outward(p, coord_t(clip)));
        }
        endl_();
    }
    for (int c = 0; c < 150; ++ c) {
        int      clip = g_clips[c % 6];
        Polygons ps;
        int      n = r.range(0, 5);
        for (int i = 0; i < n; ++ i) {
            ps.emplace_back(make_polygon(r, clip));
            if (r.range(0, 5) == 0)
                ps.back().points.resize(r.range(0, 2));
        }
        put("sops %d", clip);
        put_polys(ps);
        put(" ->");
        put_polys(smooth_outward(ps, coord_t(clip)));
        endl_();
    }
    for (int c = 0; c < 150; ++ c) {
        int        clip = g_clips[c % 6];
        ExPolygons es   = make_expolygons(r, clip);
        put("soex %d", clip);
        put_expolys(es);
        put(" ->");
        put_expolys(smooth_outward(es, coord_t(clip)));
        endl_();
    }
}

// What the mix reaches rarely or not at all: small clockwise polygons (the triangle left at the end of
// smooth_outward, MutablePolygon.cpp:357-382), a pointed notch whose width is within SCALED_EPSILON of the
// clipping distance (cpp:288, :297, :346) and a needle, a corner whose arms are nearly parallel (cpp:313).
static void run_smooth_outward_special(Lcg &r)
{
    for (int c = 0; c < 540; ++ c) {
        int     clip = g_clips[c % 6];
        int     ox   = r.range(-3000000, 3000000);
        int     oy   = r.range(-3000000, 3000000);
        Polygon p;
        switch (c % 3) {
        case 0: {
            // Clockwise, three to five points on an ellipse of about the clipping distance.
            int    n = r.range(3, 5);
            double a = double(clip) * (0.3 + 8. * r.unit() * r.unit());
            double b = double(clip) * (0.05 + 2.5 * r.unit() * r.unit());
            if (r.range(0, 1))
                std::swap(a, b);
            for (int i = 0; i < n; ++ i) {
                double angle = - 2. * PI * (double(i) + 0.8 * (r.unit() - 0.5)) / double(n);
                double rad   = 0.4 + 0.6 * r.unit();
                p.points.emplace_back(coord_t(double(ox) + a * rad * cos(angle)), coord_t(double(oy) + b * rad * sin(angle)));
            }
            break;
        }
        case 1: {
            // A rectangle with one pointed notch of the width w.
            int w  = clip - 2 + (c / 3) % 15;
            int h  = int(double(clip) * (0.3 + 2.7 * r.unit()));
            int xl = ox + 6 * clip - w / 2;
            p.points.emplace_back(ox, oy);
            p.points.emplace_back(ox + 12 * clip, oy);
            p.points.emplace_back(ox + 12 * clip, oy + 8 * clip);
            p.points.emplace_back(xl + w, oy + 8 * clip);
            p.points.emplace_back(xl + w / 2 + r.range(- w / 3, w / 3), oy + 8 * clip - h);
            p.points.emplace_back(xl, oy + 8 * clip);
            p.points.emplace_back(ox, oy + 8 * clip);
            break;
        }
        default: {
            // A needle: the corner at (ox, oy), its arms along +x, e apart at their ends.
            int l1 = int(double(clip) * (0.5 + 2.5 * r.unit()));
            int l2 = l1 + int(double(clip) * (1.2 + 2.8 * r.unit()));
            int e  = r.range(1, 150);
            if (r.range(0, 1))
                std::swap(l1, l2);
            p.points.emplace_back(ox + l1, oy);
            p.points.emplace_back(ox, oy);
            p.points.emplace_back(ox + l2, oy + e);
            p.points.emplace_back(ox + std::max(l1, l2), oy + 6 * clip);
            p.points.emplace_back(ox - clip, oy + 6 * clip);
            break;
        }
        }
        if (c % 2 == 0) {
            MutablePolygon mp(p, p.size() * 2);
            smooth_outward(mp, coord_t(clip));
            put("so %d", clip);
            put_poly(p);
            put(" ->");
            put_state(mp);
        } else {
            put("sop %d", clip);
            put_poly(p);
            put(" ->");
            put_poly(smooth_outward(p, coord_t(clip)));
        }
        endl_();
    }
}

// Polygons that are left as a clockwise triangle of a low height at the end of smooth_outward, where the hole is
// closed (MutablePolygon.cpp:365-381); a random search found them, the mixes above do not get there. Each row:
// the clipping distance, the number of points, the points.
static const int g_low_cw_triangles[][12] = {
    { 40000, 5, 22336, -4399, 23887, -33572, -26226, -19188, -12669, 34159, 2110, 40797 },
    { 3000, 4, 3928, 2500, -20, -2302, -3001, 1448, 1077, 2231, 0, 0 },
    { 10000, 5, 31379, -1266, 23726, -5782, -21768, -2018, -11459, 3499, -523, 4922 },
    { 10000, 5, 9264, 5653, 2969, -11020, -4758, -6323, -4932, 15400, 3270, 5830 },
    { 3000, 3, 6763, 1110, 1151, -2374, -3316, 1336, 0, 0, 0, 0 },
    { 40000, 5, 72362, 3388, 80190, -17776, -107871, -2739, -48615, 14976, -13421, 25477 },
    { 1200, 5, 2047, -263, 1049, -586, -1339, -522, -1316, 560, -433, 815 },
    { 1200, 3, 1058, 18, 765, -3464, -845, 2443, 0, 0, 0, 0 },
    { 1200, 3, 827, -488, -620, -3066, -528, 1648, 0, 0, 0, 0 },
    { 3000, 3, 6314, -2202, -144, -3108, -7226, 2625, 0, 0, 0, 0 },
    { 3000, 3, 7941, 394, 2740, -2556, -5733, 744, 0, 0, 0, 0 },
    { 1200, 5, 1705, -1243, 1257, -1740, -927, -622, -505, 1363, 133, 1765 },
    { 3000, 3, 4918, -815, 1074, -3284, -2521, 3337, 0, 0, 0, 0 },
    { 3000, 3, 2137, -852, -804, -8121, -825, 8397, 0, 0, 0, 0 },
    { 1200, 3, 1344, -1323, -1034, -1467, -696, 704, 0, 0, 0, 0 },
    { 3000, 5, 3141, -158, 2206, -1507, -3038, -632, -3525, 1152, -852, 2132 },
    { 40000, 5, 27419, 19081, 16064, -26853, -19465, -25810, -14680, 25618, 6681, 32188 },
    { 1200, 4, 1315, 210, 144, -699, -2496, 404, 259, 551, 0, 0 },
    { 1200, 3, 959, -923, -712, -2216, -488, 1327, 0, 0, 0, 0 },
    { 1200, 3, 2549, 82, -119, -633, -1102, 1255, 0, 0, 0, 0 },
    { 10000, 3, 7067, 8935, -4307, -25675, -1768, 35186, 0, 0, 0, 0 },
    { 1200, 3, 1618, 278, -2, -1295, -856, 978, 0, 0, 0, 0 },
    { 1200, 3, 1213, -53, -859, -900, -754, 1564, 0, 0, 0, 0 },
    { 1200, 3, 803, 2222, -883, -1313, -505, 3634, 0, 0, 0, 0 },
};

static void run_smooth_outward_low_triangles()
{
    for (const auto &row : g_low_cw_triangles) {
        Polygon p;
        for (int i = 0; i < row[1]; ++ i)
            p.points.emplace_back(row[2 + 2 * i], row[3 + 2 * i]);
        MutablePolygon mp(p, p.size() * 2);
        smooth_outward(mp, coord_t(row[0]));
        put("so %d", row[0]);
        put_poly(p);
        put(" ->");
        put_state(mp);
        endl_();
    }
}

int main(int argc, char **argv)
{
    if (argc < 2) {
        fprintf(stderr, "usage: gen <vectors.txt>\n");
        return 2;
    }
    g_out = fopen(argv[1], "w");
    if (g_out == nullptr) {
        fprintf(stderr, "cannot write %s\n", argv[1]);
        return 1;
    }
    put("const ->");
    put_double(scaled<double>(0.01));
    put_double(SCALED_EPSILON);
    put_double(sqr(SCALED_EPSILON));
    endl_();
    {
        Lcg r { 0x9E3779B97F4A7C15ULL };
        run_ops(r);
    }
    {
        Lcg r { 0x0123456789ABCDEFULL };
        run_remove_duplicates(r);
    }
    {
        Lcg r { 0xD1B54A32D192ED03ULL };
        run_smooth_outward(r);
    }
    {
        Lcg r { 0xA24BAED4963EE407ULL };
        run_smooth_outward_special(r);
    }
    run_smooth_outward_low_triangles();
    fclose(g_out);
    return 0;
}
