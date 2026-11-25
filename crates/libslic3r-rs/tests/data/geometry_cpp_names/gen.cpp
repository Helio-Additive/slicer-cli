// Native vectors for tests/geometry_cpp_names_native_parity.rs: the pin's Geometry.hpp / Geometry.cpp free
// functions directions_parallel, directions_perpendicular (Geometry.cpp:28-40), rad2deg_dir (52-57), linint
// (72-75), segments_intersect (Geometry.hpp:117-167), foot_pt x2 (169-180), ray_point_distance_squared x2,
// ray_point_distance x2 (182-200), liang_barsky_line_clipping_interval (202-252), liang_barsky_line_clipping
// x2 (254-288), is_rotation_ninety_degrees x2 (512-524). Deterministic LCG inputs; doubles as %a.
#include "libslic3r/Geometry.hpp"
#include "libslic3r/Line.hpp"
#include <cstdio>
#include <cstdint>

using namespace Slic3r;
using namespace Slic3r::Geometry;

static uint64_t s = 952;
static uint32_t rnd() { s = s * 6364136223846793005ULL + 1442695040888963407ULL; return uint32_t(s >> 33); }
static double rd(double lo, double hi) { return lo + (hi - lo) * (rnd() / 2147483648.0); }
static int ri(int lo, int hi) { return lo + int(rnd() % uint32_t(hi - lo + 1)); }

int main()
{
    const int N = 3000;
    for (int i = 0; i < N; ++i) {
        // angles: a quarter of the cases snapped near multiples of 90 degrees
        double a1 = rd(-7, 7), a2 = rd(-7, 7), md = (i % 3 == 0) ? 0. : rd(0, 0.2);
        if (i % 4 == 0) a2 = a1 + (ri(0, 3)) * 0.5 * PI + rd(-1e-3, 1e-3);
        double ra = (i % 10 == 0) ? PI : rd(-1, 2 * PI + 1);
        printf("ANG %a %a %a %d %d %a %a\n", a1, a2, md, directions_parallel(a1, a2, md) ? 1 : 0,
               directions_perpendicular(a1, a2, md) ? 1 : 0, ra, rad2deg_dir(ra));
        double v = rd(-100, 100), o0 = rd(-50, 50), o1 = o0 + rd(-30, 30), n0 = rd(-50, 50), n1 = rd(-50, 50);
        if (i % 50 == 0) o1 = o0;
        printf("LIN %a %a %a %a %a %a\n", v, o0, o1, n0, n1, linint(v, o0, o1, n0, n1));
        // integer segments, a share collinear
        int R = (i % 2) ? 1000 : 30000000;
        Point ip1(ri(-R, R), ri(-R, R)), ip2(ri(-R, R), ri(-R, R)), jp1(ri(-R, R), ri(-R, R)), jp2(ri(-R, R), ri(-R, R));
        if (i % 5 == 0) { jp1 = ip1 + (ip2 - ip1) * ri(-2, 2); jp2 = ip1 + (ip2 - ip1) * ri(-2, 3); }
        if (ip1 == ip2) ip2.x() += 1;
        if (jp1 == jp2) jp2.y() += 1;
        printf("SEG %d %d %d %d %d %d %d %d %d\n", ip1.x(), ip1.y(), ip2.x(), ip2.y(), jp1.x(), jp1.y(), jp2.x(), jp2.y(),
               segments_intersect(ip1, ip2, jp1, jp2) ? 1 : 0);
        // foot point / ray distance, double and Line forms
        Vec2d lp(rd(-1e3, 1e3), rd(-1e3, 1e3)), ld(rd(-10, 10), rd(-10, 10)), pt(rd(-1e3, 1e3), rd(-1e3, 1e3));
        if (i % 40 == 0) ld = Vec2d::Zero();
        Vec2d f = foot_pt(lp, ld, pt);
        printf("FPD %a %a %a %a %a %a %a %a %a %a\n", lp.x(), lp.y(), ld.x(), ld.y(), pt.x(), pt.y(), f.x(), f.y(),
               ray_point_distance_squared(lp, ld, pt), ray_point_distance(lp, ld, pt));
        Line ln(Point(ri(-R, R), ri(-R, R)), Point(ri(-R, R), ri(-R, R)));
        if (i % 40 == 1) ln.b = ln.a;
        Point ipt(ri(-R, R), ri(-R, R));
        Vec2d fl = foot_pt(ln, ipt);
        printf("FPL %d %d %d %d %d %d %a %a %a %a\n", ln.a.x(), ln.a.y(), ln.b.x(), ln.b.y(), ipt.x(), ipt.y(), fl.x(), fl.y(),
               ray_point_distance_squared(ln, ipt), ray_point_distance(ln, ipt));
        // Liang-Barsky on Vec2d
        BoundingBoxf bb(Vec2d(rd(-50, 0), rd(-50, 0)), Vec2d(rd(0, 50), rd(0, 50)));
        Vec2d x0(rd(-100, 100), rd(-100, 100)), x1(rd(-100, 100), rd(-100, 100));
        if (i % 7 == 0) x1.x() = x0.x();
        std::pair<double, double> iv(-1., -1.);
        bool okI = liang_barsky_line_clipping_interval(x0, Vec2d(x1 - x0), bb, iv);
        Vec2d c0, c1;
        bool okC = liang_barsky_line_clipping(x0, x1, bb, c0, c1);
        Vec2d m0 = x0, m1 = x1;
        bool okM = liang_barsky_line_clipping(m0, m1, bb);
        printf("LB %a %a %a %a %a %a %a %a %d %a %a %d %a %a %a %a %d %a %a %a %a\n", bb.min.x(), bb.min.y(), bb.max.x(),
               bb.max.y(), x0.x(), x0.y(), x1.x(), x1.y(), okI ? 1 : 0, iv.first, iv.second, okC ? 1 : 0, c0.x(), c0.y(),
               c1.x(), c1.y(), okM ? 1 : 0, m0.x(), m0.y(), m1.x(), m1.y());
        // rotations near multiples of 90 degrees
        Vec3d rot(rd(-7, 7), rd(-7, 7), rd(-7, 7));
        if (i % 2 == 0) for (int k = 0; k < 3; ++k) rot(k) = ri(-8, 8) * 0.5 * PI + rd(-2e-3, 2e-3);
        printf("ROT %a %a %a %d %d\n", rot.x(), rot.y(), rot.z(), is_rotation_ninety_degrees(rot.x()) ? 1 : 0,
               is_rotation_ninety_degrees(rot) ? 1 : 0);
    }
    return 0;
}
