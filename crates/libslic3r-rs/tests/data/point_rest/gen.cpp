// Generator of tests/data/point_rest/vectors.txt: the pin's free functions of Point.hpp that
// tests/data/point/gen.cpp does not reach, on deterministic inputs (an LCG, no rand()).
//
// Build with the engine's own compile and link line, run without arguments, output on stdout.
//
// One line per case, tokens separated by one space. Integers are decimal, doubles are the 16 hex digits
// of their bits, floats the 8 hex digits of theirs. A line is
//   c <v1 x y> <v2 x y> <z> <a x y> <b x y> <ia x y> <ib x y> <p3 x y z> <w3 x y z> <eps> <f2a x y> <f2b x y>
//     <f3a x y z> <f3b x y z> <feps> <t> <len> <16 doubles of a Transform3d, column-major>
//     -> <cross2(v1, v2)> <cross2(Vec2i64 ia, ib)> <perp(v1) x y> <perp(a) x y> <angle(v1, v2)>
//        <angle(a, b)> <to_3d(v1, z) x y z> <unscale(a.x, a.y) x y> <unscale(a) x y> <unscale(v1) x y>
//        <unscale(p3.x, p3.y, p3.z) x y z> <unscale(p3) x y z> <unscale(w3) x y z>
//        <is_approx(Vec2d v1, v1 + (w3.x, w3.y) * eps)> <is_approx(Vec3d)> <is_approx(Vec2f)> <is_approx(Vec3f)>
//        <has_reflection(T)> <get_base(0..2, T): 9 doubles> <get_x/y/z_base(T): 9 doubles>
//        <get_base(0..2, T.linear()): 9 doubles> <get_x/y/z_base(T.linear()): 9 doubles>
//        <lerp(a, b, t) x y> <shorter_then(a - b, len)>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cmath>
#include "libslic3r/libslic3r.h"
#include "libslic3r/Point.hpp"

using namespace Slic3r;

static uint64_t g_state = 0x2545F4914F6CDD1Dull;
static uint32_t lcg()
{
    g_state = g_state * 6364136223846793005ull + 1442695040888963407ull;
    return (uint32_t)(g_state >> 33);
}
static int32_t rnd(int32_t range) { return (int32_t)(lcg() % (uint32_t)(2 * range + 1)) - range; }
static double  unit() { return (double)lcg() / 2147483648.0; }
static double  rd(double range) { return (unit() * 2. - 1.) * range; }
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
static void p3(const Vec3d &v) { pd(v.x()); pd(v.y()); pd(v.z()); }

int main()
{
    const int32_t ranges[4] = { 20, 3000, 400000, 20000000 };
    const double  dranges[4] = { 1., 300., 1e5, 1e8 };
    for (int icase = 0; icase < 4000; ++icase) {
        const int32_t range  = ranges[icase % 4];
        const double  drange = dranges[(icase / 4) % 4];
        const Vec2d   v1(rd(drange), rd(drange));
        Vec2d         v2(rd(drange), rd(drange));
        if (icase % 29 == 3) v2 = v1 * 3.;
        const double  z = rd(drange);
        const Point   a(rnd(range), rnd(range));
        Point         b(rnd(range), rnd(range));
        if (icase % 31 == 7) b = a;
        const Vec2i64 ia(int64_t(rnd(range)) * int64_t(rnd(range)), int64_t(rnd(range)) * int64_t(rnd(range)));
        const Vec2i64 ib(int64_t(rnd(range)) * int64_t(rnd(range)), int64_t(rnd(range)) * int64_t(rnd(range)));
        const Vec3crd p3c(rnd(range), rnd(range), rnd(range));
        const Vec3d   w3(rd(2.), rd(2.), rd(2.));
        const double  eps = (icase % 3 == 0) ? EPSILON : unit() * 2e-4;
        const Vec2f   f2a(float(rd(drange)), float(rd(drange)));
        const Vec2f   f2b = f2a + Vec2f(float(rd(2e-4)), float(rd(2e-4)));
        const Vec3f   f3a(float(rd(drange)), float(rd(drange)), float(rd(drange)));
        const Vec3f   f3b = f3a + Vec3f(float(rd(2e-4)), float(rd(2e-4)), float(rd(2e-4)));
        const float   feps = (icase % 3 == 1) ? float(EPSILON) : float(unit() * 2e-4);
        const double  t = unit();
        const coord_t len = coord_t(unit() * 1.5 * range) + 1;
        Transform3d T = Transform3d::Identity();
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 4; ++c)
                T.matrix()(r, c) = rd(c == 3 ? drange : 2.);
        if (icase % 17 == 0) T.matrix()(3, 3) = rd(2.); // a non-affine last row reaches the 4x4 determinant
        if (icase % 19 == 0) T.matrix()(3, 0) = rd(2.);

        std::printf("c");
        pd(v1.x()); pd(v1.y()); pd(v2.x()); pd(v2.y()); pd(z);
        std::printf(" %d %d %d %d", (int)a.x(), (int)a.y(), (int)b.x(), (int)b.y());
        std::printf(" %lld %lld %lld %lld", (long long)ia.x(), (long long)ia.y(), (long long)ib.x(), (long long)ib.y());
        std::printf(" %d %d %d", (int)p3c.x(), (int)p3c.y(), (int)p3c.z());
        p3(w3);
        pd(eps);
        pf(f2a.x()); pf(f2a.y()); pf(f2b.x()); pf(f2b.y());
        pf(f3a.x()); pf(f3a.y()); pf(f3a.z()); pf(f3b.x()); pf(f3b.y()); pf(f3b.z());
        pf(feps);
        pd(t);
        std::printf(" %d", (int)len);
        for (int c = 0; c < 4; ++c)
            for (int r = 0; r < 4; ++r)
                pd(T.matrix()(r, c));
        std::printf(" ->");

        pd(cross2(v1, v2));
        std::printf(" %lld", (long long)cross2(ia, ib));
        { const Vec2d q = perp(v1); pd(q.x()); pd(q.y()); }
        { const Vec2crd q = perp(static_cast<const Vec2crd &>(a)); std::printf(" %d %d", (int)q.x(), (int)q.y()); }
        pd(angle(v1, v2));
        pd(angle(a, b));
        p3(to_3d(v1, z));
        { const Vec2d q = unscale(a.x(), a.y()); pd(q.x()); pd(q.y()); }
        { const Vec2d q = unscale(static_cast<const Vec2crd &>(a)); pd(q.x()); pd(q.y()); }
        { const Vec2d q = unscale(v1); pd(q.x()); pd(q.y()); }
        p3(unscale(p3c.x(), p3c.y(), p3c.z()));
        p3(unscale(p3c));
        p3(unscale(w3));
        std::printf(" %d", is_approx(v1, Vec2d(v1 + Vec2d(w3.x(), w3.y()) * eps), eps) ? 1 : 0);
        std::printf(" %d", is_approx(Vec3d(w3 * drange), Vec3d(w3 * drange + Vec3d(v1.x(), v1.y(), z) * (2. * eps / drange)), eps) ? 1 : 0);
        std::printf(" %d", is_approx(f2a, f2b, feps) ? 1 : 0);
        std::printf(" %d", is_approx(f3a, f3b, feps) ? 1 : 0);
        std::printf(" %d", has_reflection(T) ? 1 : 0);
        for (unsigned i = 0; i < 3; ++i) p3(get_base(i, T));
        p3(get_x_base(T)); p3(get_y_base(T)); p3(get_z_base(T));
        {
            const Transform3d::LinearPart L = T.linear();
            for (unsigned i = 0; i < 3; ++i) p3(get_base(i, L));
            p3(get_x_base(L)); p3(get_y_base(L)); p3(get_z_base(L));
        }
        { const Point q = lerp(a, b, t); std::printf(" %d %d", (int)q.x(), (int)q.y()); }
        std::printf(" %d", shorter_then(Point(a - b), len) ? 1 : 0);
        std::printf("\n");
    }
    return 0;
}
