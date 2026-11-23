// Generator of tests/data/line_rest/vectors.txt: the pin's `namespace line_alg` of Line.hpp (23-135) on
// a 2D integer Line, the functions tests/data/line/gen.cpp reaches only through the Line members.
// Deterministic inputs (an LCG, no rand()). Build with the engine's own compile and link line, run
// without arguments, output on stdout.
//
// One line per case: c <a x y> <b x y> <p x y> -> <get_a x y> <get_b x y>
//   <distance_to_squared(line, p, &nearest)> <nearest x y> <distance_to(line, p)>
//   <distance_to_infinite_squared(line, p, &closest)> <closest x y> <distance_to_infinite(line, p)>
// followed by
//   | <Line3 a x y z> <b x y z> <Linef3 a x y z> <b x y z> <z> <factor> <pt x y z> <Line3float a x y z> <b x y z>
//   -> <Line3::length> <Line3::vector x y z> <Linef3::intersect_plane(z) x y z> <scale(factor): a, b>
//      <vector> <unit_vector> <length> <reverse: a, b> <get_point_projection_to_line(pt, a, vector()): pt, proj_length>
//      <get_distance_of_point_to_line(pt)> <Line3float::vector> <unit_vector> <length>
//   | <16 doubles of a Transform3d T, column-major> -> <transform(Linef3, T): a, b>
// Integers are decimal, doubles the 16 hex digits of their bits, floats the 8 hex digits of theirs.
#include <cstdio>
#include <cstdint>
#include <cstring>
#include "libslic3r/libslic3r.h"
#include "libslic3r/Point.hpp"
#include "libslic3r/Line.hpp"

using namespace Slic3r;

static uint64_t g_state = 0x3C6EF372FE94F82Bull;
static uint32_t lcg()
{
    g_state = g_state * 6364136223846793005ull + 1442695040888963407ull;
    return (uint32_t)(g_state >> 33);
}
static int32_t rnd(int32_t range) { return (int32_t)(lcg() % (uint32_t)(2 * range + 1)) - range; }
static void pd(double v)
{
    uint64_t b;
    std::memcpy(&b, &v, 8);
    std::printf(" %016llx", (unsigned long long)b);
}
static void pf(float v)
{
    uint32_t b;
    std::memcpy(&b, &v, 4);
    std::printf(" %08x", (unsigned)b);
}
static double unit() { return (double)lcg() / 2147483648.0; }
static double rd(double range) { return (unit() * 2. - 1.) * range; }
static void p3(const Vec3d &v) { pd(v.x()); pd(v.y()); pd(v.z()); }
static void p3f(const Vec3f &v) { pf(v.x()); pf(v.y()); pf(v.z()); }
static void p3c(const Vec3crd &v) { std::printf(" %d %d %d", (int)v.x(), (int)v.y(), (int)v.z()); }
static void pp(const Vec2crd &p) { std::printf(" %d %d", (int)p.x(), (int)p.y()); }

int main()
{
    const int32_t ranges[4] = { 20, 3000, 400000, 20000000 };
    for (int icase = 0; icase < 3000; ++icase) {
        const int32_t range = ranges[icase % 4];
        const Point a(rnd(range), rnd(range));
        Point b(rnd(range), rnd(range));
        if (icase % 23 == 4) b = a;
        Point p(rnd(range), rnd(range));
        if (icase % 13 == 6) p = a;
        const Line line(a, b);
        std::printf("c");
        pp(a); pp(b); pp(p);
        std::printf(" ->");
        pp(line_alg::get_a(line));
        pp(line_alg::get_b(line));
        {
            Vec2crd nearest(0, 0);
            pd(line_alg::distance_to_squared(line, Vec2crd(p), &nearest));
            pp(nearest);
            pd(line_alg::distance_to(line, Vec2crd(p)));
        }
        {
            Vec2crd closest(0, 0);
            pd(line_alg::distance_to_infinite_squared(line, Vec2crd(p), &closest));
            pp(closest);
            pd(line_alg::distance_to_infinite(line, Vec2crd(p)));
        }
        {
            const double drange = (icase % 3 == 0) ? 1. : (icase % 3 == 1) ? 1e3 : 1e7;
            const Line3 l3(Vec3crd(rnd(range), rnd(range), rnd(range)), Vec3crd(rnd(range), rnd(range), rnd(range)));
            Linef3 lf(Vec3d(rd(drange), rd(drange), rd(drange)), Vec3d(rd(drange), rd(drange), rd(drange)));
            if (icase % 41 == 9) lf.b = lf.a;
            const double z = rd(drange), factor = rd(3.);
            const Vec3d  pt(rd(drange), rd(drange), rd(drange));
            Line3float l3f(Vec3f(float(rd(drange)), float(rd(drange)), float(rd(drange))), Vec3f(float(rd(drange)), float(rd(drange)), float(rd(drange))));
            if (icase % 43 == 11) l3f.b = l3f.a;
            std::printf(" |");
            p3c(l3.a); p3c(l3.b); p3(lf.a); p3(lf.b); pd(z); pd(factor); p3(pt); p3f(l3f.a); p3f(l3f.b);
            std::printf(" ->");
            pd(l3.length());
            p3c(l3.vector());
            p3(lf.intersect_plane(z));
            { Linef3 c = lf; c.scale(factor); p3(c.a); p3(c.b); }
            p3(lf.vector());
            p3(lf.unit_vector());
            pd(lf.length());
            { Linef3 c = lf; c.reverse(); p3(c.a); p3(c.b); }
            { Vec3d ip; float pl; Linef3::get_point_projection_to_line(pt, lf.a, lf.vector(), ip, pl); p3(ip); pf(pl); }
            pf(lf.get_distance_of_point_to_line(pt));
            p3f(l3f.vector());
            p3f(l3f.unit_vector());
            pd(l3f.length());
            Transform3d T = Transform3d::Identity();
            for (int r = 0; r < 3; ++r)
                for (int c = 0; c < 4; ++c)
                    T.matrix()(r, c) = rd(c == 3 ? drange : 2.);
            std::printf(" |");
            for (int c = 0; c < 4; ++c)
                for (int r = 0; r < 4; ++r)
                    pd(T.matrix()(r, c));
            std::printf(" ->");
            { const Linef3 tl = transform(lf, T); p3(tl.a); p3(tl.b); }
        }
        std::printf("\n");
    }
    return 0;
}
