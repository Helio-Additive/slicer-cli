// Vectors for tests/auto_circle_compensation_native_parity.rs: the pin's
// LayerRegion::auto_circle_compensation (LayerRegion.cpp:17-18, 69-125) on deterministic surfaces
// (approximate circles with and without holes, polygons that are not circles, too few points) with
// random AutoContourHolesCompensationParams (Print.hpp:369-405) for two filaments.
//
// Usage: gen <cases>  (prints the vectors on stdout)
//
// Format, one record per line, tokens separated by one space; a double is its 16 hexadecimal digits,
// a float its 8, an integer decimal; a polygon is "<n> x0 y0 x1 y1 ...":
//   case <id> <wall_filament> <manual_offset float>
//   params <12 x 2 doubles: counter_speed_coef counter_diameter_coef counter_compensate_coef hole_speed_coef
//          hole_diameter_coef hole_compensate_coef counter_limit_min_value counter_limit_max_value
//          hole_limit_min_value hole_limit_max_value circle_compensation_speed diameter_limit, 2 values each>
//   in <surface count>
//   s <contour> <hole count> <holes...>                      (one line per input surface)
//   out
//   s <contour> <hole count> <holes...> <counter_circle_compensation 0/1> <k> <hole indices...>
// The vectors were made on macOS arm64 with Apple clang.

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <vector>

#define private public
#define protected public
#include "libslic3r/Print.hpp"
#include "libslic3r/Layer.hpp"
#undef private
#undef protected
#include "libslic3r/Surface.hpp"

using namespace Slic3r;

static uint64_t g_seed = 0x9E3779B97F4A7C15ull;
static uint32_t lcg()
{
    g_seed = g_seed * 6364136223846793005ull + 1442695040888963407ull;
    return uint32_t(g_seed >> 33);
}
static double uni(double a, double b) { return a + (b - a) * (double(lcg()) / 2147483648.0); }
static int    irange(int a, int b) { return a + int(lcg() % uint32_t(b - a + 1)); }

static void put_double(double v)
{
    uint64_t u;
    std::memcpy(&u, &v, 8);
    std::printf(" %016llx", (unsigned long long) u);
}
static void put_float(float v)
{
    uint32_t u;
    std::memcpy(&u, &v, 4);
    std::printf(" %08x", u);
}
static void put_polygon(const Polygon &p)
{
    std::printf(" %d", int(p.points.size()));
    for (const Point &pt : p.points) std::printf(" %d %d", int(pt.x()), int(pt.y()));
}

// A ring of n points around (cx, cy) with radius r (mm) and a radial noise of +-noise (mm).
static Polygon ring(double cx, double cy, double r, int n, double noise, bool ccw)
{
    Polygon p;
    for (int i = 0; i < n; ++i) {
        double a  = 2. * M_PI * double(i) / double(n);
        double rr = r + uni(-noise, noise);
        p.points.emplace_back(coord_t(std::lround(scale_(cx + rr * std::cos(a)))), coord_t(std::lround(scale_(cy + rr * std::sin(a)))));
    }
    if (! ccw) p.reverse();
    return p;
}
static Polygon rect(double cx, double cy, double w, double h, bool ccw)
{
    Polygon p;
    p.points = { Point(scale_(cx - w / 2), scale_(cy - h / 2)), Point(scale_(cx + w / 2), scale_(cy - h / 2)),
                 Point(scale_(cx + w / 2), scale_(cy + h / 2)), Point(scale_(cx - w / 2), scale_(cy + h / 2)) };
    if (! ccw) p.reverse();
    return p;
}

static void put_surface(const Surface &s, bool with_flags)
{
    std::printf("s");
    put_polygon(s.expolygon.contour);
    std::printf(" %d", int(s.expolygon.holes.size()));
    for (const Polygon &h : s.expolygon.holes) put_polygon(h);
    if (with_flags) {
        std::printf(" %d %d", s.counter_circle_compensation ? 1 : 0, int(s.holes_circle_compensation.size()));
        for (int i : s.holes_circle_compensation) std::printf(" %d", i);
    }
    std::printf("\n");
}

int main(int argc, char **argv)
{
    int cases = argc > 1 ? std::atoi(argv[1]) : 300;
    for (int c = 0; c < cases; ++c) {
        int   wall_filament = irange(1, 2);
        // Layer.cpp:42 passes scale_(circle_compensation_manual_offset) as the float manual_offset.
        float manual_offset = float(scale_(uni(-0.1, 0.1)));
        if (c % 7 == 0) manual_offset = 0.f;

        AutoContourHolesCompensationParams p;
        for (int f = 0; f < 2; ++f) {
            p.counter_speed_coef.push_back(uni(-0.002, 0.002));
            p.counter_diameter_coef.push_back(uni(-0.02, 0.02));
            p.counter_compensate_coef.push_back(uni(-0.1, 0.1));
            p.hole_speed_coef.push_back(uni(-0.002, 0.002));
            p.hole_diameter_coef.push_back(uni(-0.02, 0.02));
            p.hole_compensate_coef.push_back(uni(-0.1, 0.1));
            p.counter_limit_min_value.push_back(uni(-0.3, 0.));
            p.counter_limit_max_value.push_back(uni(0., 0.3));
            p.hole_limit_min_value.push_back(uni(-0.3, 0.));
            p.hole_limit_max_value.push_back(uni(0., 0.3));
            p.circle_compensation_speed.push_back(uni(20., 300.));
            p.diameter_limit.push_back(uni(2., 30.));
        }

        SurfaceCollection slices;
        int nsurf = irange(1, 4);
        for (int k = 0; k < nsurf; ++k) {
            double cx = uni(-50., 50.), cy = uni(-50., 50.);
            int    kind = irange(0, 5);
            ExPolygon ex;
            double r = uni(0.5, 15.);
            if (kind <= 2) {
                // an approximate circle (kind 2: noisy, so some fail the deviation / variance tests)
                int n = irange(5, 96);
                ex.contour = ring(cx, cy, r, n, kind == 2 ? uni(0., 0.6) : uni(0., 0.01), true);
            } else if (kind == 3) {
                ex.contour = rect(cx, cy, uni(1., 30.), uni(1., 30.), true);
            } else {
                // a star: not a circle
                int n = irange(8, 40);
                Polygon st;
                for (int i = 0; i < n; ++i) {
                    double a  = 2. * M_PI * double(i) / double(n);
                    double rr = (i % 2) ? r : r * 0.5;
                    st.points.emplace_back(coord_t(std::lround(scale_(cx + rr * std::cos(a)))), coord_t(std::lround(scale_(cy + rr * std::sin(a)))));
                }
                ex.contour = st;
                r *= 0.5;
            }
            int nholes = (kind == 4 || r < 2.) ? 0 : irange(0, 3);
            for (int h = 0; h < nholes; ++h) {
                double hr = r * uni(0.05, 0.18);
                double a  = uni(0., 2. * M_PI);
                double d  = (h == 0 && lcg() % 2) ? 0. : r * 0.55;
                int    n  = irange(6, 64);
                if (lcg() % 4 == 0)
                    ex.holes.push_back(rect(cx + d * std::cos(a), cy + d * std::sin(a), hr, hr * 1.3, false));
                else
                    ex.holes.push_back(ring(cx + d * std::cos(a), cy + d * std::sin(a), hr, n, uni(0., 0.02), false));
            }
            slices.surfaces.emplace_back(stInternal, ex);
        }

        std::printf("case %d %d", c, wall_filament);
        put_float(manual_offset);
        std::printf("\nparams");
        for (const std::vector<double> *v : { &p.counter_speed_coef, &p.counter_diameter_coef, &p.counter_compensate_coef,
                                              &p.hole_speed_coef, &p.hole_diameter_coef, &p.hole_compensate_coef,
                                              &p.counter_limit_min_value, &p.counter_limit_max_value, &p.hole_limit_min_value,
                                              &p.hole_limit_max_value, &p.circle_compensation_speed, &p.diameter_limit })
            for (double d : *v) put_double(d);
        std::printf("\nin %d\n", int(slices.surfaces.size()));
        for (const Surface &s : slices.surfaces) put_surface(s, false);

        PrintRegionConfig cfg;
        cfg.wall_filament.value = wall_filament;
        PrintRegion region(cfg);
        Layer       layer(0, nullptr, 0.2, 0.2, 0.1);
        LayerRegion layerm(&layer, &region);
        layerm.auto_circle_compensation(slices, p, manual_offset);

        std::printf("out\n");
        for (const Surface &s : slices.surfaces) put_surface(s, true);
    }
    return 0;
}
