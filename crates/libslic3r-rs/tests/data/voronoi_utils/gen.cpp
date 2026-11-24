// Native vectors for Geometry::VoronoiUtils::discretize_parabola (VoronoiUtils.cpp:103-197), the
// instantiation Arachne calls (over Arachne::PolygonsSegmentIndex), on the pin's own function.
// Usage: gen <cases>   (vectors go to stdout)
// Line format:
//   dp <p.x> <p.y> <a.x> <a.y> <b.x> <b.y> <start.x> <start.y> <end.x> <end.y> <step> <angle %a> => <n> <x,y x n>
// The point site p, the segment site a-b, start and end on the parabola between them (rounded), the
// step size and the float transitioning angle. The angles are deg2rad<float>(10) (the transition
// angle WallToolPaths passes, Geometry.hpp:295 over a float) and random floats.
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>

#include "libslic3r/Geometry.hpp"
#include "libslic3r/Geometry/VoronoiUtils.hpp"
#include "libslic3r/Arachne/utils/PolygonsSegmentIndex.hpp"

using namespace Slic3r;

static uint64_t g_state = 0x9E3779B97F4A7C15ULL;
static uint64_t next_u64()
{
    g_state = g_state * 6364136223846793005ULL + 1442695040888963407ULL;
    return g_state >> 11;
}
static int64_t range(int64_t lo, int64_t hi) { return lo + int64_t(next_u64() % uint64_t(hi - lo + 1)); }

static void one(const Point &p, const Point &a, const Point &b, const Point &start, const Point &end, coord_t step, float angle)
{
    Polygons polys{Polygon{a, b}};
    Arachne::PolygonsSegmentIndex seg(&polys, 0, 0);
    Points r = Geometry::VoronoiUtils::discretize_parabola(p, seg, start, end, step, angle);
    printf("dp %lld %lld %lld %lld %lld %lld %lld %lld %lld %lld %lld %a =>", (long long) p.x(), (long long) p.y(), (long long) a.x(),
           (long long) a.y(), (long long) b.x(), (long long) b.y(), (long long) start.x(), (long long) start.y(), (long long) end.x(),
           (long long) end.y(), (long long) step, double(angle));
    printf(" %zu", r.size());
    for (const Point &q : r) printf(" %lld,%lld", (long long) q.x(), (long long) q.y());
    printf("\n");
}

int main(int argc, char **argv)
{
    const int   cases     = argc > 1 ? atoi(argv[1]) : 3000;
    const float angle_10  = Geometry::deg2rad(10.f);
    // The parabolic edge of Majora's layer 110 whose marking moved with the float angle (R945).
    one(Point(-6253169, 1101556), Point(-5849462, 1331046), Point(-5884030, 1389689), Point(-6068251, 1243340), Point(-6080350, 1260441),
        scaled<coord_t>(0.8), angle_10);
    const coord_t steps[] = {2000, 5000, 20000, scaled<coord_t>(0.8)};
    for (int i = 0; i < cases; ++i) {
        const Point  a(range(-7000000, 7000000), range(-7000000, 7000000));
        const double len = double(range(10000, 1000000));
        const double phi = double(range(0, 1000000)) / 1000000. * 2. * M_PI;
        const Point  b(a.x() + coord_t(std::lround(len * std::cos(phi))), a.y() + coord_t(std::lround(len * std::sin(phi))));
        // Frame: u along the segment, n towards the point site.
        const double ux = std::cos(phi), uy = std::sin(phi);
        const double side = range(0, 1) ? 1. : -1.;
        const double nx = -uy * side, ny = ux * side;
        const double d    = double(range(500, 500000));
        const double px   = double(range(-int64_t(len), 2 * int64_t(len)));
        const double fx   = double(a.x()) + ux * px, fy = double(a.y()) + uy * px;
        const Point  p(coord_t(std::lround(fx + nx * d)), coord_t(std::lround(fy + ny * d)));
        auto on_parabola = [&](double x) {
            const double y = x * x / (2. * d) + d / 2.;
            return Point(coord_t(std::lround(fx + ux * x + nx * y)), coord_t(std::lround(fy + uy * x + ny * y)));
        };
        const double span = double(range(1000, 400000));
        const double xs   = double(range(-int64_t(span), int64_t(span)));
        const double xe   = xs + (range(0, 1) ? span : -span);
        const coord_t step  = steps[range(0, 3)];
        const float   angle = range(0, 2) == 0 ? float(double(range(50000, 1500000)) / 1000000.) : angle_10;
        one(p, a, b, on_parabola(xs), on_parabola(xe), step, angle);
    }
    return 0;
}
