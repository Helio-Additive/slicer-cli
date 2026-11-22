// Generator of the vectors of tests/bridge_detector_native_parity.rs: libslic3r's BridgeDetector.cpp and
// BridgeDetector.hpp at the pin (references/BambuStudio/src/libslic3r; the override layer has no copy of
// them).
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
//   ./gen vectors.txt
//
// BridgeDetector.cpp is compiled into this file (get_trapezoids2 and get_trapezoids3_half are static), with
// the private members of the class made public.
//
// Groups: "len" (Line::length, Line::direction), "bd" (a BridgeDetector over bridges and lower slices made
// from a fixed linear congruential generator: the constructor, bridge_direction_candidates, detect_angle
// with and without an override, coverage plain and precise, unsupported_edges, and the trapezoid functions
// on the first bridge), "dbd" (detect_bridging_direction of floating edges) and "dbda"
// (detect_bridging_direction of an area to cover and its anchors).
//
// Lines that start with "in" are the inputs the Rust test reads back; the others are results it has to
// reproduce. Tokens are separated by one space; a double is its 16 hexadecimal digits, an integer decimal.
// The vectors were made on macOS arm64 with Apple clang and Eigen 5.0.1.

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <map>
#include <string>
#include <vector>

// Before the access specifiers are redefined: tbb declares nested classes ahead of their definition.
#include <tbb/blocked_range.h>
#include <tbb/global_control.h>
#include <tbb/parallel_for.h>

#define private public
#define protected public
#include "libslic3r/BridgeDetector.cpp"
#undef protected
#undef private

using namespace Slic3r;

static FILE *g_out = nullptr;

static std::string hD(double v)
{
    uint64_t u;
    std::memcpy(&u, &v, 8);
    char b[32];
    std::snprintf(b, sizeof(b), "%016llx", (unsigned long long) u);
    return b;
}
static std::string hI(long long v) { return std::to_string(v); }
static std::string hB(bool v) { return v ? "1" : "0"; }
static void out(const std::string &s) { std::fprintf(g_out, "%s\n", s.c_str()); }
static std::string PT(const Point &p) { return hI(p.x()) + " " + hI(p.y()); }
static std::string PTS(const Points &pts)
{
    std::string o = hI((long long) pts.size());
    for (const Point &pt : pts) o += " " + PT(pt);
    return o;
}
static std::string POLY(const Polygon &p) { return PTS(p.points); }
static std::string POLYS(const Polygons &ps)
{
    std::string o = hI((long long) ps.size());
    for (const Polygon &p : ps) o += " " + POLY(p);
    return o;
}
static std::string EXPOLY(const ExPolygon &e)
{
    std::string o = POLY(e.contour) + " " + hI((long long) e.holes.size());
    for (const Polygon &h : e.holes) o += " " + POLY(h);
    return o;
}
static std::string EXPOLYS(const ExPolygons &es)
{
    std::string o = hI((long long) es.size());
    for (const ExPolygon &e : es) o += " " + EXPOLY(e);
    return o;
}
static std::string PLS(const Polylines &ps)
{
    std::string o = hI((long long) ps.size());
    for (const Polyline &p : ps) o += " " + PTS(p.points);
    return o;
}

// The generator of the inputs.
static uint64_t g_seed = 0x2545F4914F6CDD1DULL;
static uint32_t lcg()
{
    g_seed = g_seed * 6364136223846793005ULL + 1442695040888963407ULL;
    return uint32_t(g_seed >> 33);
}
static int irand(int lo, int hi) { return lo + int(lcg() % uint32_t(hi - lo + 1)); }

static Polygon rect(coord_t x0, coord_t y0, coord_t x1, coord_t y1)
{
    Polygon p;
    p.points = { Point(x0, y0), Point(x1, y0), Point(x1, y1), Point(x0, y1) };
    return p;
}
static Polygon ccw(Polygon p)
{
    if (! p.is_counter_clockwise()) p.reverse();
    return p;
}
// An irregular polygon around a centre, by rational directions (no trigonometry).
static Polygon blob(coord_t cx, coord_t cy, coord_t rx, coord_t ry)
{
    static const int dirs[8][2] = { { 4, 0 }, { 3, 3 }, { 0, 4 }, { -3, 3 }, { -4, 0 }, { -3, -3 }, { 0, -4 }, { 3, -3 } };
    Polygon p;
    for (int i = 0; i < 8; ++ i) {
        const int r = irand(55, 100);
        p.points.emplace_back(Point(cx + coord_t(int64_t(rx) * dirs[i][0] * r / 400), cy + coord_t(int64_t(ry) * dirs[i][1] * r / 400)));
    }
    return ccw(p);
}

static const coord_t MM = 100000;

struct Scene
{
    ExPolygons bridge;
    ExPolygons lower;
};

static Scene make_scene(int type)
{
    Scene         s;
    const coord_t ox = coord_t(irand(-20, 20)) * MM + irand(-5000, 5000);
    const coord_t oy = coord_t(irand(-20, 20)) * MM + irand(-5000, 5000);
    const coord_t w  = coord_t(irand(50, 250)) * (MM / 10);
    const coord_t h  = coord_t(irand(20, 120)) * (MM / 10);
    switch (type) {
    case 0: {
        // A rectangle between two anchors, left and right.
        s.bridge.emplace_back(rect(ox, oy, ox + w, oy + h));
        const coord_t in = irand(0, 20) * (MM / 10), outw = irand(10, 40) * (MM / 10), m = irand(0, 20) * (MM / 10);
        s.lower.emplace_back(rect(ox - outw, oy - m, ox + in, oy + h + m));
        s.lower.emplace_back(rect(ox + w - in, oy - m, ox + w + outw, oy + h + m));
        break;
    }
    case 1: {
        // A rectangle in the mouth of a U: anchored at three sides by one polygon.
        s.bridge.emplace_back(rect(ox, oy, ox + w, oy + h));
        const coord_t t = irand(10, 30) * (MM / 10), in = irand(0, 10) * (MM / 10);
        Polygon       u;
        u.points = { Point(ox - t, oy - t), Point(ox + w + t, oy - t), Point(ox + w + t, oy + h + t), Point(ox + w - in, oy + h + t), Point(ox + w - in, oy + in),
                     Point(ox + in, oy + in),  Point(ox + in, oy + h + t), Point(ox - t, oy + h + t) };
        s.lower.emplace_back(u);
        break;
    }
    case 2: {
        // The shoulder of a counterbore: a ring over a wider hole of the layer below.
        ExPolygon ring(rect(ox, oy, ox + w, oy + h));
        Polygon   hole = rect(ox + w / 3, oy + h / 3, ox + 2 * w / 3, oy + 2 * h / 3);
        hole.reverse();
        ring.holes.emplace_back(hole);
        s.bridge.emplace_back(ring);
        const coord_t t = irand(10, 40) * (MM / 10), in = irand(1, 12) * (MM / 10);
        ExPolygon     low(rect(ox - t, oy - t, ox + w + t, oy + h + t));
        Polygon       lh = rect(ox + in, oy + in, ox + w - in, oy + h - in);
        lh.reverse();
        low.holes.emplace_back(lh);
        s.lower.emplace_back(low);
        break;
    }
    case 3: {
        // An irregular bridge over one to three rectangles.
        s.bridge.emplace_back(blob(ox + w / 2, oy + h / 2, w, h));
        const int n = irand(1, 3);
        for (int i = 0; i < n; ++ i) {
            const coord_t ax = ox + irand(-30, 220) * (MM / 10), ay = oy + irand(-30, 100) * (MM / 10);
            s.lower.emplace_back(rect(ax, ay, ax + irand(10, 80) * (MM / 10), ay + irand(10, 80) * (MM / 10)));
        }
        break;
    }
    case 4: {
        // Two bridges, three lower slices.
        s.bridge.emplace_back(rect(ox, oy, ox + w, oy + h));
        s.bridge.emplace_back(rect(ox + w + 20 * (MM / 10), oy + irand(-20, 20) * (MM / 10), ox + 2 * w + 20 * (MM / 10), oy + h));
        s.lower.emplace_back(rect(ox - 20 * (MM / 10), oy - 10 * (MM / 10), ox + irand(5, 30) * (MM / 10), oy + h + 10 * (MM / 10)));
        s.lower.emplace_back(rect(ox + w - 10 * (MM / 10), oy - 30 * (MM / 10), ox + w + 30 * (MM / 10), oy + h + 30 * (MM / 10)));
        s.lower.emplace_back(rect(ox + 2 * w, oy + h / 2, ox + 2 * w + 50 * (MM / 10), oy + h + 20 * (MM / 10)));
        break;
    }
    case 5: {
        // Nothing below, or nothing near.
        s.bridge.emplace_back(rect(ox, oy, ox + w, oy + h));
        if (irand(0, 1) == 0) s.lower.emplace_back(rect(ox + w + 50 * MM, oy, ox + w + 60 * MM, oy + 10 * MM));
        break;
    }
    default: {
        // A slanted strip between two anchors at its ends.
        const coord_t dx = coord_t(irand(-80, 80)) * (MM / 10);
        Polygon       p;
        p.points = { Point(ox, oy), Point(ox + w, oy + dx), Point(ox + w, oy + dx + h), Point(ox, oy + h) };
        s.bridge.emplace_back(ccw(p));
        const coord_t in = irand(2, 20) * (MM / 10), outw = irand(10, 40) * (MM / 10);
        s.lower.emplace_back(rect(ox - outw, oy - 20 * (MM / 10), ox + in, oy + h + 20 * (MM / 10)));
        s.lower.emplace_back(rect(ox + w - in, oy + dx - 20 * (MM / 10), ox + w + outw, oy + dx + h + 20 * (MM / 10)));
        break;
    }
    }
    return s;
}

// The scene turned by a rational rotation (c, s) / d, the coordinates truncated: no trigonometry.
static void turn(Polygon &p, int c, int s, int d)
{
    for (Point &q : p.points) {
        const coord_t x = (q.x() * c - q.y() * s) / d, y = (q.x() * s + q.y() * c) / d;
        q = Point(x, y);
    }
}
static void turn(ExPolygons &es, int c, int s, int d)
{
    for (ExPolygon &e : es) {
        turn(e.contour, c, s, d);
        for (Polygon &h : e.holes) turn(h, c, s, d);
    }
}

static void run_len()
{
    for (int i = 0; i < 48; ++ i) {
        Point a(irand(-3000000, 3000000), irand(-3000000, 3000000));
        Point b(irand(-3000000, 3000000), irand(-3000000, 3000000));
        if (i % 8 == 1) b = Point(a.x() - irand(1, 500000), a.y());       // pointing to -x: the direction is 0
        if (i % 8 == 2) b = Point(a.x() + irand(1, 500000), a.y());
        if (i % 8 == 3) b = Point(a.x(), a.y() - irand(1, 500000));
        if (i % 8 == 4) b = Point(a.x() - irand(100000, 500000), a.y() - irand(0, 3)); // just below -x
        if (i % 8 == 5) b = Point(a.x() - irand(100000, 500000), a.y() + irand(0, 3)); // just above -x
        const Line l(a, b);
        out("in len " + PT(a) + " " + PT(b));
        out("len " + hD(l.length()) + " " + hD(l.direction()));
    }
}

static void run_bd()
{
    static const double angles[] = { 0., PI / 2., 0.7, 2.5, PI / 4., 1.9, 3. * PI / 4., 0.05 };
    static const double overrides[] = { 0.3, PI + 0.25, 1.9, PI / 2., PI, 2.9 };
    static const int turns[][3] = { { 1, 0, 1 }, { 4, 3, 5 }, { 3, 4, 5 }, { 12, 5, 13 }, { 5, -12, 13 }, { -8, 15, 17 }, { 7, 7, 10 }, { 20, 21, 29 } };
    for (int k = 0; k < 126; ++ k) {
        Scene         s       = make_scene(k % 7);
        // From the 57th scenario on the scenes are turned.
        if (k >= 56) {
            const int *t = turns[1 + (k / 7) % 7];
            turn(s.bridge, t[0], t[1], t[2]);
            turn(s.lower, t[0], t[1], t[2]);
        }
        const coord_t spacing = irand(30000, 90000);
        const double  a       = angles[k % 8];
        const double  ov      = overrides[k % 6];
        const bool    single  = s.bridge.size() == 1 && k % 2 == 0;
        const std::string S   = "bd " + hI(k);
        out("in " + S + " " + hI(spacing) + " " + hB(single) + " " + hD(a) + " " + hD(ov) + " " + EXPOLYS(s.bridge) + " " + EXPOLYS(s.lower));

        // The detector keeps references to the expolygons and the lower slices.
        std::unique_ptr<BridgeDetector> bdp(single ? new BridgeDetector(s.bridge.front(), s.lower, spacing) : new BridgeDetector(s.bridge, s.lower, spacing));
        BridgeDetector                 &bd = *bdp;
        out(S + " init " + hD(bd.resolution) + " " + hD(bd.angle) + " " + PLS(bd._edges) + " " + EXPOLYS(bd._anchor_regions));
        {
            const std::vector<double> cand = bd.bridge_direction_candidates();
            std::string               o    = S + " cand " + hI((long long) cand.size());
            for (double c : cand) o += " " + hD(c);
            out(o);
        }
        // Before an angle is known: nothing covered, no edges.
        out(S + " none " + POLYS(bd.coverage()) + " " + PLS(bd.unsupported_edges()));
        const bool found = bd.detect_angle();
        out(S + " detect " + hB(found) + " " + hD(bd.angle));
        out(S + " cov0 " + POLYS(bd.coverage(-1, false)));
        out(S + " cov1 " + POLYS(bd.coverage(-1, true)));
        out(S + " covA " + POLYS(bd.coverage(a, false)));
        out(S + " covB " + POLYS(bd.coverage(a, true)));
        out(S + " un " + PLS(bd.unsupported_edges()));
        {
            Polylines un;
            un.emplace_back(Polyline(Point(1, 2), Point(3, 4)));
            bd.unsupported_edges(a, &un);
            out(S + " unA " + PLS(un));
        }
        {
            BridgeDetector bd2(s.bridge, s.lower, spacing);
            const bool     found2 = bd2.detect_angle(ov);
            out(S + " ov " + hB(found2) + " " + hD(bd2.angle) + " " + POLYS(bd2.coverage()));
        }
        // The trapezoids of the first bridge.
        {
            Polygons t2, t2a, t3;
            get_trapezoids2(s.bridge.front(), &t2);
            t2a.emplace_back(rect(0, 0, 100000, 50000)); // what is in the output already is rotated back as well
            get_trapezoids2(s.bridge.front(), &t2a, a);
            get_trapezoids3_half(s.bridge.front(), &t3, float(spacing));
            out(S + " tz2 " + POLYS(t2));
            out(S + " tz2a " + POLYS(t2a));
            out(S + " tz3 " + POLYS(t3));
        }
    }
}

static void run_dbd()
{
    for (int k = 0; k < 64; ++ k) {
        Lines     edges;
        const int n = k % 8 == 0 ? 0 : irand(1, 14);
        for (int i = 0; i < n; ++ i) {
            Point a(irand(-3000000, 3000000), irand(-3000000, 3000000));
            Point b(irand(-3000000, 3000000), irand(-3000000, 3000000));
            switch (irand(0, 9)) {
            case 0: b = Point(a.x() + irand(1, 900000), a.y()); break;
            case 1: b = Point(a.x(), a.y() + irand(1, 900000)); break;
            case 2: b = Point(a.x() - irand(1, 900000), a.y()); break;
            case 3:
                // the reverse of an earlier edge: the same cost, the order of the container decides
                if (! edges.empty()) {
                    const Line &e = edges[size_t(irand(0, int(edges.size()) - 1))];
                    a = e.b;
                    b = e.a;
                }
                break;
            case 4:
                // parallel to an earlier edge
                if (! edges.empty()) {
                    const Line &e = edges[size_t(irand(0, int(edges.size()) - 1))];
                    b = a + (e.b - e.a);
                }
                break;
            case 5:
                if (k % 16 == 5) b = a; // an edge without length
                break;
            default: break;
            }
            edges.emplace_back(a, b);
        }
        Polygons  area;
        const int np = k % 16 == 8 ? 0 : irand(1, 3);
        for (int i = 0; i < np; ++ i) {
            const coord_t cx = irand(-2000000, 2000000), cy = irand(-2000000, 2000000);
            if (k % 16 == 0 && i == 0)
                area.emplace_back(rect(cx, cy, cx + irand(1, 3), cy + irand(1, 3))); // smaller than the resolution
            else if (irand(0, 2) == 0)
                area.emplace_back(rect(cx, cy, cx + irand(100000, 3000000), cy + irand(100000, 3000000)));
            else
                area.emplace_back(blob(cx, cy, irand(200000, 3000000), irand(200000, 3000000)));
        }
        std::string in = "in dbd " + hI((long long) edges.size());
        for (const Line &l : edges) in += " " + PT(l.a) + " " + PT(l.b);
        out(in + " " + POLYS(area));
        const auto [dir, cost] = detect_bridging_direction(edges, area);
        out("dbd " + hD(dir.x()) + " " + hD(dir.y()) + " " + hD(cost));
    }
}

static void run_dbda()
{
    for (int k = 0; k < 48; ++ k) {
        const coord_t ox = irand(-2000000, 2000000), oy = irand(-2000000, 2000000);
        const coord_t w = irand(300000, 3000000), h = irand(200000, 2000000);
        Polygons      to_cover, anchors;
        if (k % 6 == 5)
            to_cover.emplace_back(blob(ox + w / 2, oy + h / 2, w, h));
        else
            to_cover.emplace_back(rect(ox, oy, ox + w, oy + h));
        const coord_t t = irand(50000, 400000), in = irand(1000, 150000);
        switch (k % 6) {
        case 0: // anchored left and right
            anchors.emplace_back(rect(ox - t, oy - t, ox + in, oy + h + t));
            anchors.emplace_back(rect(ox + w - in, oy - t, ox + w + t, oy + h + t));
            break;
        case 1: // anchored at one side
            anchors.emplace_back(rect(ox - t, oy - t, ox + w + t, oy + in));
            break;
        case 2: { // anchored all around: a frame
            anchors.emplace_back(rect(ox - t, oy - t, ox + w + t, oy + h + t));
            Polygon hole = rect(ox + in, oy + in, ox + w - in, oy + h - in);
            hole.reverse();
            anchors.emplace_back(hole);
            break;
        }
        case 3: // not anchored
            if (irand(0, 1) == 0) anchors.emplace_back(rect(ox + w + 5 * t, oy, ox + w + 6 * t, oy + h));
            break;
        case 4: // anchored at a corner
            anchors.emplace_back(rect(ox - t, oy - t, ox + w / 2, oy + h / 2));
            break;
        default: // an irregular area over two anchors
            anchors.emplace_back(rect(ox - t, oy - t, ox + w / 4, oy + h + t));
            anchors.emplace_back(rect(ox + 3 * w / 4, oy - t, ox + w + t, oy + h + t));
            break;
        }
        out("in dbda " + POLYS(to_cover) + " " + POLYS(anchors));
        const auto [dir, cost] = detect_bridging_direction(to_cover, anchors);
        out("dbda " + hD(dir.x()) + " " + hD(dir.y()) + " " + hD(cost));
    }
}

int main(int argc, char **argv)
{
    g_out = std::fopen(argc > 1 ? argv[1] : "vectors.txt", "w");
    tbb::global_control gc(tbb::global_control::max_allowed_parallelism, 1);
    run_len();
    run_bd();
    run_dbd();
    run_dbda();
    std::fclose(g_out);
    return 0;
}
