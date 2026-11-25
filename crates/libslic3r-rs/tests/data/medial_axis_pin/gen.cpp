// Native vectors for tests/medial_axis_pin_native_parity.rs: the pin's own
// Geometry::MedialAxis (Geometry/MedialAxis.cpp:445-688, both `build` overloads) and the
// three `ExPolygon::medial_axis` overloads (ExPolygon.cpp:263-380, ExPolygon.hpp:73-76), as
// libslic3r_core.a compiles them (clang, arm64, -O3, Eigen), on deterministic ExPolygons:
// rotated strips, rings, narrow wedges, offset random walks and the gap areas of random stars
// cut as PerimeterGenerator.cpp:1346-1358 cuts them.
//
// Build: target/parity-tools/build.sh <this file> <output binary>; run: <binary> > vectors.txt
//
// Output, one record per case (coordinates are integers, widths hex floats):
//   CASE <i> <kind> <min %a> <max %a>
//   POLY <n> x y ...            the contour, then one HOLE line per hole
//   HOLE <n> x y ...
//   BUILD <count>               MedialAxis::build(ThickPolylines*), then per polyline
//     P <n> x y ...  W <m> <%a> ...  E <first> <second>
//   BUILDP SAME|<count>         MedialAxis::build(Polylines*): SAME when equal to BUILD's points
//   MA <count>                  ExPolygon::medial_axis(min, max, ThickPolylines*), P/W/E as above
//   MAP SAME|<count>            ExPolygon::medial_axis(min, max, Polylines*)
//   MAR SAME|<count>            Polylines ExPolygon::medial_axis(min, max)
//   END
#include "libslic3r/libslic3r.h"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Geometry/MedialAxis.hpp"
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <vector>

using namespace Slic3r;

static uint64_t g_state = 0x853c49e6748fea9bULL;
static uint32_t next_u32() {
    g_state = g_state * 6364136223846793005ULL + 1442695040888963407ULL;
    return uint32_t(g_state >> 32);
}
static double unit() { return double(next_u32()) / 4294967296.0; }
static double rnd(double lo, double hi) { return lo + (hi - lo) * unit(); }
static coord_t sc(double mm) { return coord_t(std::lrint(mm / SCALING_FACTOR)); }

static void print_points(const char *tag, const Points &pts) {
    printf("%s %zu", tag, pts.size());
    for (const Point &p : pts) printf(" %lld %lld", (long long) p.x(), (long long) p.y());
    printf("\n");
}

static void print_thick(const char *tag, const ThickPolylines &tp) {
    printf("%s %zu\n", tag, tp.size());
    for (const ThickPolyline &pl : tp) {
        print_points("P", pl.points);
        printf("W %zu", pl.width.size());
        for (double w : pl.width) printf(" %a", w);
        printf("\nE %d %d\n", int(pl.endpoints.first), int(pl.endpoints.second));
    }
}

static void print_polylines(const char *tag, const Polylines &pls, const ThickPolylines &ref) {
    bool same = pls.size() == ref.size();
    for (size_t i = 0; same && i < pls.size(); ++i) same = pls[i].points == ref[i].points;
    if (same) {
        printf("%s SAME\n", tag);
        return;
    }
    printf("%s %zu\n", tag, pls.size());
    for (const Polyline &pl : pls) print_points("Q", pl.points);
}

static int g_case = 0;

static void run_case(const ExPolygon &ex, int kind, double min_w, double max_w) {
    if (ex.contour.points.size() < 3) return;
    printf("CASE %d %d %a %a\n", g_case++, kind, min_w, max_w);
    print_points("POLY", ex.contour.points);
    for (const Polygon &h : ex.holes) print_points("HOLE", h.points);

    ThickPolylines build_tp;
    {
        Geometry::MedialAxis ma(min_w, max_w, ex);
        ma.build(&build_tp);
    }
    print_thick("BUILD", build_tp);
    Polylines build_pl;
    {
        Geometry::MedialAxis ma(min_w, max_w, ex);
        ma.build(&build_pl);
    }
    print_polylines("BUILDP", build_pl, build_tp);

    ThickPolylines ma_tp;
    ex.medial_axis(min_w, max_w, &ma_tp);
    print_thick("MA", ma_tp);
    Polylines ma_pl;
    ex.medial_axis(min_w, max_w, &ma_pl);
    print_polylines("MAP", ma_pl, ma_tp);
    Polylines ma_ret = ex.medial_axis(min_w, max_w);
    print_polylines("MAR", ma_ret, ma_tp);
    printf("END\n");
}

static Point rot(double x, double y, double a, double cx, double cy) {
    double c = std::cos(a), s = std::sin(a);
    return Point(sc(cx + c * x - s * y), sc(cy + s * x + c * y));
}

int main() {
    // kind 0: rotated strips (rectangles) of width w.
    for (int i = 0; i < 300; ++i) {
        double w = rnd(0.03, 1.2), len = rnd(0.3, 15.), a = rnd(0., 2. * M_PI);
        double cx = rnd(-100., 100.), cy = rnd(-100., 100.);
        ExPolygon ex;
        ex.contour.points = { rot(-len / 2, -w / 2, a, cx, cy), rot(len / 2, -w / 2, a, cx, cy),
                              rot(len / 2, w / 2, a, cx, cy), rot(-len / 2, w / 2, a, cx, cy) };
        if (! ex.contour.is_counter_clockwise()) ex.contour.reverse();
        double ws = w / SCALING_FACTOR;
        run_case(ex, 0, rnd(0., 0.6) * ws, rnd(0.9, 3.) * ws);
    }
    // kind 1: rings (annuli) with jittered vertices.
    for (int i = 0; i < 200; ++i) {
        int n = 8 + int(next_u32() % 57);
        double r = rnd(1., 20.), w = rnd(0.05, 1.0), cx = rnd(-100., 100.), cy = rnd(-100., 100.);
        double phase = rnd(0., 2. * M_PI);
        ExPolygon ex;
        for (int k = 0; k < n; ++k) {
            double t = phase + 2. * M_PI * k / n, ro = r + w + rnd(-0.1, 0.1) * w;
            ex.contour.points.push_back(Point(sc(cx + ro * std::cos(t)), sc(cy + ro * std::sin(t))));
        }
        Polygon hole;
        for (int k = 0; k < n; ++k) {
            double t = phase + 2. * M_PI * k / n, ri = r + rnd(-0.1, 0.1) * w;
            hole.points.push_back(Point(sc(cx + ri * std::cos(t)), sc(cy + ri * std::sin(t))));
        }
        hole.reverse();
        ex.holes.push_back(hole);
        double ws = w / SCALING_FACTOR;
        run_case(ex, 1, rnd(0., 0.6) * ws, rnd(0.9, 3.) * ws);
    }
    // kind 2: narrow wedges (triangles and quads with a short side).
    for (int i = 0; i < 250; ++i) {
        double base = rnd(0.1, 1.5), len = rnd(1., 10.), a = rnd(0., 2. * M_PI);
        double cx = rnd(-100., 100.), cy = rnd(-100., 100.), tip = rnd(0., 0.3) * base;
        ExPolygon ex;
        if (next_u32() % 2)
            ex.contour.points = { rot(0, -base / 2, a, cx, cy), rot(len, 0, a, cx, cy), rot(0, base / 2, a, cx, cy) };
        else
            ex.contour.points = { rot(0, -base / 2, a, cx, cy), rot(len, -tip / 2, a, cx, cy),
                                  rot(len, tip / 2, a, cx, cy), rot(0, base / 2, a, cx, cy) };
        if (! ex.contour.is_counter_clockwise()) ex.contour.reverse();
        double ws = base / SCALING_FACTOR;
        run_case(ex, 2, rnd(0., 0.5) * ws, rnd(0.8, 2.5) * ws);
    }
    // kind 3: random walks offset by half a width (the union of the offset).
    for (int i = 0; i < 250; ++i) {
        int n = 3 + int(next_u32() % 6);
        double w = rnd(0.05, 1.0), x = rnd(-100., 100.), y = rnd(-100., 100.), a = rnd(0., 2. * M_PI);
        Polyline pl;
        pl.points.push_back(Point(sc(x), sc(y)));
        for (int k = 1; k < n; ++k) {
            double step = rnd(1., 5.);
            a += rnd(-1.2, 1.2);
            x += step * std::cos(a);
            y += step * std::sin(a);
            pl.points.push_back(Point(sc(x), sc(y)));
        }
        double ws = w / SCALING_FACTOR;
        ExPolygons exs = union_ex(offset(pl, float(ws / 2.)));
        double min_w = rnd(0., 0.6) * ws, max_w = rnd(0.9, 3.) * ws;
        for (const ExPolygon &ex : exs) run_case(ex, 3, min_w, max_w);
    }
    // kind 4: the gaps of random stars, as PerimeterGenerator.cpp:1346-1358 cuts them (with and
    // without the douglas_peucker of :1356).
    for (int i = 0; i < 150; ++i) {
        int n = 5 + int(next_u32() % 36);
        double r0 = rnd(2., 15.), cx = rnd(-100., 100.), cy = rnd(-100., 100.);
        Polygon star;
        for (int k = 0; k < n; ++k) {
            double t = 2. * M_PI * k / n, r = r0 * rnd(0.5, 1.0);
            star.points.push_back(Point(sc(cx + r * std::cos(t)), sc(cy + r * std::sin(t))));
        }
        double perimeter_width = rnd(0.3, 0.6) / SCALING_FACTOR, perimeter_spacing = perimeter_width * rnd(0.85, 1.0);
        double min = 0.2 * perimeter_width * (1 - INSET_OVERLAP_TOLERANCE);
        double max = 2. * perimeter_spacing;
        // The ring left between two perimeters: a gap band of random width.
        double inset = rnd(0.5, 3.) * perimeter_width, band = rnd(0.2, 2.2) * perimeter_spacing;
        ExPolygons gaps = diff_ex(offset(star, -float(inset)), offset(star, -float(inset + band)));
        ExPolygons gaps_ex = diff_ex(opening_ex(gaps, float(min / 2.)),
                                     offset2_ex(gaps, -float(max / 2.), float(max / 2. + ClipperSafetyOffset)));
        bool dp = (i % 2) == 0;
        for (ExPolygon &ex : gaps_ex) {
            if (dp) ex.douglas_peucker(scaled<double>(0.0125));
            run_case(ex, dp ? 5 : 4, min, max);
        }
    }
    return 0;
}
