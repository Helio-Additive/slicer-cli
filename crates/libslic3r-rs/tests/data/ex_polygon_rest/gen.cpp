// Vectors for tests/ex_polygon_rest_native_parity.rs: the inline free functions of ExPolygon.hpp:93-449 at the pin
// (count_points, number_polygons, to_lines, to_points, translate(ExPolygons), polygons_append, expolygons_append,
// expolygons_rotate, expolygons_contain) on LCG ExPolygons.
//
// Usage: gen <cases>   (writes to stdout)
//
// One case is a block of lines:
//   C <case> <count> then per expolygon: <nholes> <ring> <ring>*nholes     (<ring> is <n> <x y>*n)
//   N <count_points(expolys)> <count_points(expoly)>* <number_polygons(expolys)>
//   L <count> <ax ay bx by>*            to_lines(expolys)
//   L1 <count> <ax ay bx by>*           to_lines(expolys[0])
//   P <points>                          to_points(expolys) ; P1 <points> to_points(expolys[0])
//   T <dx> <dy> <expolys>               translate(expolys, d)
//   R <angle double> <expolys>          expolygons_rotate(expolys, angle)
//   S <tolerance double> <expolys>     expolygons_simplify(expolys, tolerance)
//   Q <x> <y> <border> <ret>            expolygons_contain (6 times)
//   A <polygons>                        polygons_append(dst = {expolys[0].contour}, expolys) then (dst, expolys[0])
//   PL <count> <points>*  to_polylines(expolys), (expolys[0]), moved expolys, moved expolys.back()
//   PG <count> <points>*  to_polygons, the same four ; TE <expolys> to_expolygons(to_polygons(expolys)) by reference then moved
//   E <expolys>                         expolygons_append(dst = {expolys.back()}, expolys)
// Integers are decimal, doubles the 16 hex digits of their bits.
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <cmath>
#include <algorithm>
#include <vector>
#include "libslic3r/libslic3r.h"
#include "libslic3r/Point.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/ExPolygon.hpp"

using namespace Slic3r;

static uint64_t g_state = 0xD1B54A32D192ED03ull;
static uint32_t lcg()
{
    g_state = g_state * 6364136223846793005ull + 1442695040888963407ull;
    return (uint32_t)(g_state >> 33);
}
static int32_t  rnd(int32_t range) { return (int32_t)(lcg() % (uint32_t)(2 * range + 1)) - range; }
static uint32_t below(uint32_t n) { return lcg() % n; }
static void     pd(double v)
{
    uint64_t b;
    std::memcpy(&b, &v, 8);
    std::printf(" %016llx", (unsigned long long)b);
}
static void ppts(const Points &pts)
{
    std::printf(" %d", (int)pts.size());
    for (const Point &p : pts) std::printf(" %lld %lld", (long long)p.x(), (long long)p.y());
}
static void pexs(const ExPolygons &ex)
{
    std::printf(" %d", (int)ex.size());
    for (const ExPolygon &e : ex) {
        std::printf(" %d", (int)e.holes.size());
        ppts(e.contour.points);
        for (const Polygon &h : e.holes) ppts(h.points);
    }
}
static void plines(const Lines &lines)
{
    std::printf(" %d", (int)lines.size());
    for (const Line &l : lines) std::printf(" %lld %lld %lld %lld", (long long)l.a.x(), (long long)l.a.y(), (long long)l.b.x(), (long long)l.b.y());
}
static Polygon ring(int cx, int cy, int R, int n, bool ccw)
{
    Polygon p;
    for (int i = 0; i < n; ++i) {
        double a = 2. * M_PI * (double)i / (double)n;
        double r = (double)R * (0.7 + 0.3 * (double)below(1000) / 1000.);
        p.points.emplace_back(cx + (coord_t)std::lround(r * std::cos(a)), cy + (coord_t)std::lround(r * std::sin(a)));
    }
    if (!ccw) std::reverse(p.points.begin(), p.points.end());
    return p;
}

int main(int argc, char **argv)
{
    int cases = argc > 1 ? std::atoi(argv[1]) : 1000;
    for (int c = 0; c < cases; ++c) {
        ExPolygons ex;
        int        cnt = 1 + (int)below(4);
        for (int i = 0; i < cnt; ++i) {
            int       cx = rnd(5000000), cy = rnd(5000000), R = 200000 + (int)below(2000000);
            ExPolygon e;
            e.contour = ring(cx, cy, R, 3 + (int)below(14), true);
            int nh = (int)below(3);
            for (int h = 0; h < nh; ++h) e.holes.push_back(ring(cx + rnd(R / 4), cy + rnd(R / 4), R / 5, 3 + (int)below(8), false));
            ex.push_back(e);
        }
        std::printf("C %d", c); pexs(ex); std::printf("\n");
        std::printf("N %d", (int)count_points(ex));
        for (const ExPolygon &e : ex) std::printf(" %d", (int)count_points(e));
        std::printf(" %d\n", (int)number_polygons(ex));
        std::printf("L"); plines(to_lines(ex)); std::printf("\n");
        std::printf("L1"); plines(to_lines(ex.front())); std::printf("\n");
        std::printf("P"); ppts(to_points(ex)); std::printf("\n");
        std::printf("P1"); ppts(to_points(ex.front())); std::printf("\n");
        { ExPolygons t = ex; Point d(rnd(1000000), rnd(1000000)); translate(t, d); std::printf("T %lld %lld", (long long)d.x(), (long long)d.y()); pexs(t); std::printf("\n"); }
        { ExPolygons t = ex; double a = (double)rnd(1000000) * (M_PI / 1000000.); expolygons_rotate(t, a); std::printf("R"); pd(a); pexs(t); std::printf("\n"); }
        { double tol = (double)below(60000); std::printf("S"); pd(tol); pexs(expolygons_simplify(ex, tol)); std::printf("\n"); }
        for (int q = 0; q < 6; ++q) {
            const ExPolygon &e = ex[below(ex.size())];
            Point pt = q < 2 ? e.contour.points[below(e.contour.points.size())] : (q < 4 && !e.holes.empty() ? e.holes[0].points[0] + Point(rnd(50000), rnd(50000)) : e.contour.points[0] + Point(rnd(2000000), rnd(2000000)));
            bool border = below(2) != 0;
            std::printf("Q %lld %lld %d %d\n", (long long)pt.x(), (long long)pt.y(), border ? 1 : 0, expolygons_contain(ex, pt, border) ? 1 : 0);
        }
        {
            Polygons dst{ex.front().contour};
            polygons_append(dst, ex);
            polygons_append(dst, ex.front());
            ExPolygons mv = ex;
            polygons_append(dst, std::move(mv));
            ExPolygon mv1 = ex.back();
            polygons_append(dst, std::move(mv1));
            std::printf("A %d", (int)dst.size());
            for (const Polygon &p : dst) ppts(p.points);
            std::printf("\n");
        }
        {
            // to_polylines / to_polygons (by reference and moved), to_expolygons
            std::printf("PL"); { Polylines a = to_polylines(ex); Polylines b = to_polylines(ex.front()); ExPolygons m = ex; Polylines c = to_polylines(std::move(m)); ExPolygon m1 = ex.back(); Polylines d = to_polylines(std::move(m1));
                std::printf(" %d", (int)(a.size() + b.size() + c.size() + d.size())); for (auto *v : {&a, &b, &c, &d}) for (const Polyline &pl : *v) ppts(pl.points); } std::printf("\n");
            std::printf("PG"); { Polygons a = to_polygons(ex); Polygons b = to_polygons(ex.front()); ExPolygons m = ex; Polygons c = to_polygons(std::move(m)); ExPolygon m1 = ex.back(); Polygons d = to_polygons(std::move(m1));
                std::printf(" %d", (int)(a.size() + b.size() + c.size() + d.size())); for (auto *v : {&a, &b, &c, &d}) for (const Polygon &pg : *v) ppts(pg.points); } std::printf("\n");
            std::printf("TE"); { Polygons pp = to_polygons(ex); ExPolygons a = to_expolygons(pp); ExPolygons b = to_expolygons(std::move(pp)); a.insert(a.end(), b.begin(), b.end()); pexs(a); } std::printf("\n");
        }
        {
            ExPolygons dst{ex.back()};
            expolygons_append(dst, ex);
            ExPolygons mv = ex;
            expolygons_append(dst, std::move(mv));
            ExPolygons empty;
            ExPolygons mv2 = ex;
            expolygons_append(empty, std::move(mv2));
            expolygons_append(dst, empty);
            std::printf("E"); pexs(dst); std::printf("\n");
        }
    }
    return 0;
}
