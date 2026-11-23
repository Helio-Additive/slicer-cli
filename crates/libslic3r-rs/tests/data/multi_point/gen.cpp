// Generator of tests/data/multi_point/vectors.txt: the pin's MultiPoint (MultiPoint.hpp / MultiPoint.cpp) on
// deterministic inputs (an LCG, no rand()), through Polyline, the MultiPoint whose lines() are the open chain.
//
// Build with the engine's own compile and link line, run without arguments, output on stdout.
//
// One line per case, tokens separated by one space. Integers are decimal, doubles the 16 hex digits of their
// bits. A line is
//   m <pts: n points> <v> <cos> <sin> <angle> <center> <q> <eps> <tol> <x_axis>
//     -> <translate(v): n points> <rotate(cos, sin): n points> <rotate(angle, center): n points>
//        <length()> <length(Points)> <find_point(q)> <find_point(q, eps)> <has_boundary_point(q)>
//        <bounding_box(): box> <has_duplicate_points()> <remove_duplicate_points(): removed n points>
//        <remove_colinear_points(): changed n points> <concave_hull_2d(pts, tol): computed [n points]>
//        <symmetric_y(x_axis): n points> <area(Points)> <get_extents(mp): box>
// where a point is `x y` and a box `defined min max`. concave_hull_2d is computed only for the cases whose
// coordinates keep Eigen's integer squaredNorm inside int32 (range <= 15000); MultiPoint::visivalingam is not
// called: it writes `areas[i]` into a vector that was only reserved and reads it back through a copy of size 0
// (MultiPoint.cpp:287-347), undefined behaviour at the pin.
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cmath>
#include "libslic3r/libslic3r.h"
#include "libslic3r/Point.hpp"
#include "libslic3r/Line.hpp"
#include "libslic3r/MultiPoint.hpp"
#include "libslic3r/Polyline.hpp"
#include "libslic3r/BoundingBox.hpp"

using namespace Slic3r;

static uint64_t g_state = 0x9E3779B97F4A7C15ull;
static uint32_t lcg()
{
    g_state = g_state * 6364136223846793005ull + 1442695040888963407ull;
    return (uint32_t)(g_state >> 33);
}
static int32_t rnd(int32_t range) { return (int32_t)(lcg() % (uint32_t)(2 * range + 1)) - range; }
static double  unit() { return (double)lcg() / 2147483648.0; }
static void pd(double v)
{
    uint64_t b;
    std::memcpy(&b, &v, 8);
    std::printf(" %016llx", (unsigned long long)b);
}
static void pp(const Point &p) { std::printf(" %d %d", (int)p.x(), (int)p.y()); }
static void pps(const Points &pts)
{
    std::printf(" %d", (int)pts.size());
    for (const Point &p : pts) pp(p);
}
static void pbox(const BoundingBox &b) { std::printf(" %d", b.defined ? 1 : 0); pp(b.min); pp(b.max); }

// `gen area` writes tests/data/multi_point/area.txt instead: area(const Points&) (MultiPoint.hpp:144-149) on
// n = 3..99 points with coordinates up to 1e8, where products pass 2^53. clang vectorizes the loop as an ordered
// reduction (the products in vector registers, the sums in order), so the fused multiply-add of the scalar loop
// applies only to the iterations it leaves to the scalar remainder. A line is `a <pts: n points> -> <area>`.
static int area_cases()
{
    for (int n = 3; n < 100; ++n)
        for (int r = 0; r < 8; ++r) {
            Points pts;
            for (int i = 0; i < n; ++i) pts.emplace_back(rnd(100000000), rnd(100000000));
            std::printf("a");
            pps(pts);
            std::printf(" ->");
            pd(area(pts));
            std::printf("\n");
        }
    return 0;
}

// `gen mp3` writes tests/data/multi_point/mp3.txt instead: MultiPoint3 (MultiPoint.hpp:111-125,
// MultiPoint.cpp:388-433) through Polyline3. A line is
//   t <pts: n points x y z> <x> <y> <v> -> <translate(x, y): n points> <translate(v): n points> <length()>
//      <is_valid()> <bounding_box(): defined min max> <remove_duplicate_points(): removed n points>
static void pp3(const Vec3crd &p) { std::printf(" %d %d %d", (int)p.x(), (int)p.y(), (int)p.z()); }
static void pps3(const Points3 &pts)
{
    std::printf(" %d", (int)pts.size());
    for (const Vec3crd &p : pts) pp3(p);
}
static int mp3_cases()
{
    const int32_t ranges[4] = { 20, 3000, 400000, 100000000 };
    for (int icase = 0; icase < 2000; ++icase) {
        const int32_t range = ranges[icase % 4];
        const int n = 1 + int(lcg() % 10);
        Polyline3 pl;
        for (int i = 0; i < n; ++i) pl.append(Vec3crd(rnd(range), rnd(range), (icase % 5 == 0) ? 7 : rnd(range)));
        if (lcg() % 3 == 0) { const size_t k = lcg() % pl.points.size(); pl.points.insert(pl.points.begin() + k, pl.points[k]); }
        const double x = (unit() - 0.5) * double(range), y = (unit() - 0.5) * double(range);
        const Point  v(rnd(range / 2), rnd(range / 2));
        std::printf("t");
        pps3(pl.points); pd(x); pd(y); pp(v);
        std::printf(" ->");
        { Polyline3 t(pl); t.translate(x, y); pps3(t.points); }
        { Polyline3 t(pl); t.translate(v); pps3(t.points); }
        pd(pl.length());
        std::printf(" %d", pl.is_valid() ? 1 : 0);
        { BoundingBox3 b = pl.bounding_box(); std::printf(" %d", b.defined ? 1 : 0); pp3(b.min); pp3(b.max); }
        { Polyline3 t(pl); const bool r = t.remove_duplicate_points(); std::printf(" %d", r ? 1 : 0); pps3(t.points); }
        std::printf("\n");
    }
    return 0;
}

int main(int argc, char **argv)
{
    if (argc > 1 && std::strcmp(argv[1], "mp3") == 0)
        return mp3_cases();
    if (argc > 1 && std::strcmp(argv[1], "area") == 0)
        return area_cases();
    // The last range makes products beyond 2^53, where a fused multiply-add and two roundings differ.
    const int32_t ranges[5] = { 20, 3000, 15000, 400000, 100000000 };
    for (int icase = 0; icase < 5000; ++icase) {
        const int32_t range = ranges[icase % 5];
        const int n = 1 + int(lcg() % 14);
        Points pts;
        for (int i = 0; i < n; ++i) pts.emplace_back(rnd(range), rnd(range));
        // Repeated neighbours, collinear (and nearly collinear) inner points.
        if (lcg() % 3 == 0) { const size_t k = lcg() % pts.size(); pts.insert(pts.begin() + k, pts[k]); }
        if (pts.size() >= 2 && lcg() % 3 == 0) {
            const size_t k = lcg() % (pts.size() - 1);
            const Point  a = pts[k], b = pts[k + 1];
            Point m((a.x() + b.x()) / 2 + rnd(1) * int32_t(lcg() % 3), (a.y() + b.y()) / 2);
            pts.insert(pts.begin() + k + 1, m);
        }
        Polyline mp(pts);
        const Point  v(rnd(range / 2), rnd(range / 2));
        const double a      = (unit() - 0.5) * 4. * M_PI;
        const double c      = std::cos(a);
        const double s      = std::sin(a + 0.); // inputs only: printed, read back by the test
        const double angle  = (unit() - 0.5) * 4. * M_PI;
        const Point  center(rnd(range / 2), rnd(range / 2));
        Point q(rnd(range), rnd(range));
        const int kind = icase % 5;
        if (kind == 1) q = pts[lcg() % pts.size()];
        if (kind == 2 && pts.size() >= 2) { const size_t k = lcg() % (pts.size() - 1); q = Point((pts[k].x() + pts[k + 1].x()) / 2, (pts[k].y() + pts[k + 1].y()) / 2); }
        if (kind == 3) q = pts[lcg() % pts.size()] + Point(rnd(3), rnd(3));
        const double eps    = (icase % 7 == 0) ? 0. : unit() * 0.2 * double(range);
        const double tol    = unit() * 0.1 * double(range);
        const int32_t x_axis = rnd(range / 2);

        std::printf("m");
        pps(pts); pp(v); pd(c); pd(s); pd(angle); pp(center); pp(q); pd(eps); pd(tol);
        std::printf(" %d ->", (int)x_axis);
        { Polyline t(mp); t.translate(v); pps(t.points); }
        { Polyline t(mp); t.rotate(c, s); pps(t.points); }
        { Polyline t(mp); t.rotate(angle, center); pps(t.points); }
        pd(mp.length());
        pd(length(mp.points));
        std::printf(" %d %d %d", mp.find_point(q), mp.find_point(q, eps), mp.has_boundary_point(q) ? 1 : 0);
        pbox(mp.bounding_box());
        std::printf(" %d", mp.has_duplicate_points() ? 1 : 0);
        { Polyline t(mp); const bool r = t.remove_duplicate_points(); std::printf(" %d", r ? 1 : 0); pps(t.points); }
        { Polyline t(mp); const bool r = t.remove_colinear_points(); std::printf(" %d", r ? 1 : 0); pps(t.points); }
        if (range <= 15000) { std::printf(" 1"); pps(MultiPoint::concave_hull_2d(mp.points, tol)); } else std::printf(" 0");
        { Polyline t(mp); t.symmetric_y(x_axis); pps(t.points); }
        pd(area(mp.points));
        pbox(get_extents(mp));
        std::printf("\n");
    }
    return 0;
}
