// Reference vectors for ray_circle_intersections_r2_lv2_c and ray_circle_intersections in
// crates/libslic3r-rs/src/geometry/circle.rs, taken from Geometry/Circle.hpp at the pin.
//
// The functions are header templates; the program instantiates them for double as slicer_cli's
// libslic3r does (clang, -O3, -ffp-contract=on, the default on arm64 macOS: `r2 * lv2 - c * c`
// and `x0 + b * d` are fused). ray_circle_intersections itself calls an undefined
// `ray_circle_intersections_r2_lv2_c2` and cannot be instantiated at the pin; its cases here run
// its body (Circle.hpp:176-182) with the defined function in that place. Build with the include
// paths and flags of a libslic3r source file of a configured Bambu Studio engine build tree, then:
//   ./circle_gen | gzip -9n > crates/libslic3r-rs/tests/data/geometry_circle/vectors.txt.gz
//
// Doubles are written as the hex of their bits. One case per line:
//   R2 r2 a b lv2 c -> n x1 y1 x2 y2          ray_circle_intersections_r2_lv2_c (out as left when n == 0)
//   RC r a b c -> n x1 y1 x2 y2               ray_circle_intersections (body of Circle.hpp:174-183)

#include "libslic3r/Geometry/Circle.hpp"

#include <cmath>
#include <cstdio>
#include <cstring>

using namespace Slic3r;

static uint64_t s = 0x9E3779B97F4A7C15ull;
static uint64_t rnd() { s ^= s << 13; s ^= s >> 7; s ^= s << 17; return s; }
static double   unit() { return double(rnd() >> 11) / double(1ull << 53); }
static double   uni(double a, double b) { return a + (b - a) * unit(); }
static unsigned long long db(double d) { unsigned long long u; memcpy(&u, &d, 8); return u; }

using RVec = Eigen::Matrix<double, 2, 1, Eigen::DontAlign>;

static void out(int n, const std::pair<RVec, RVec> &p)
{
    printf(" %d %016llx %016llx %016llx %016llx\n", n, db(p.first.x()), db(p.first.y()), db(p.second.x()), db(p.second.y()));
}

// Circle.hpp:174-183 with the defined function in place of the undefined one.
static int ray_circle_intersections_body(double r, double a, double b, double c, std::pair<RVec, RVec> &out)
{
    double lv2 = a * a + b * b;
    if (lv2 < double(SCALED_EPSILON * SCALED_EPSILON))
        return false;
    return Geometry::ray_circle_intersections_r2_lv2_c(r * r, a, b, a * a + b * b, c, out);
}

int main()
{
    for (int i = 0; i < 1500; ++ i) {
        double r2, a, b, lv2, c;
        if (i % 2 == 0) {
            // As FillBase.cpp:708 calls it: a segment end point, a line point and vector, scaled.
            Vec2d segment_pt(uni(-2e7, 2e7), uni(-2e7, 2e7));
            Vec2d line_pt = segment_pt + Vec2d(uni(-1e6, 1e6), uni(-1e6, 1e6));
            Vec2d line_vec(uni(-2e6, 2e6), uni(-2e6, 2e6));
            double offset = uni(1e3, 1e6);
            r2 = offset * offset;
            Vec2d p0 = line_pt - segment_pt;
            lv2 = line_vec.squaredNorm();
            a = line_vec.y();
            b = - line_vec.x();
            c = - line_vec.y() * p0.x() + line_vec.x() * p0.y();
        } else {
            a = uni(-1e4, 1e4); b = uni(-1e4, 1e4); c = uni(-1e7, 1e7);
            lv2 = a * a + b * b; r2 = uni(0, 2e6) * uni(0, 2e6);
            if (i % 10 == 1) c = std::sqrt(r2 * lv2) * (rnd() & 1 ? 1. : -1.);
        }
        std::pair<RVec, RVec> p{RVec(0, 0), RVec(0, 0)};
        int n = Geometry::ray_circle_intersections_r2_lv2_c(r2, a, b, lv2, c, p);
        printf("R2 %016llx %016llx %016llx %016llx %016llx ->", db(r2), db(a), db(b), db(lv2), db(c));
        out(n, p);
    }
    for (int i = 0; i < 700; ++ i) {
        double r = uni(0, 2e6), a = uni(-2e4, 2e4), b = uni(-2e4, 2e4), c = uni(-2e9, 2e9);
        if (i % 7 == 0) { a = uni(-12, 12); b = uni(-12, 12); }
        std::pair<RVec, RVec> p{RVec(0, 0), RVec(0, 0)};
        int n = ray_circle_intersections_body(r, a, b, c, p);
        printf("RC %016llx %016llx %016llx %016llx ->", db(r), db(a), db(b), db(c));
        out(n, p);
    }
    return 0;
}
