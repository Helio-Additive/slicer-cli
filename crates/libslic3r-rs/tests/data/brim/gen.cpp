// Generator of the vectors of tests/brim_native_parity.rs: the three overloads of compSecondMoment
// of libslic3r's Brim.cpp at the pin (references/BambuStudio/src/libslic3r, the override layer has
// no copy of it), the second moment of area the automatic brim width is computed from.
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
//   ./gen > $DATA/vectors.txt
//
// Made on macOS arm64 with Apple clang and libc++. Lines (doubles as 16 hexadecimal digits, a
// polygon as its number of points and the points, scaled):
//   poly <polygon> -> <ok> <sm.x> <sm.y>
//   expoly <number of holes> <contour> <holes> -> <ok> <aera> <centroid> <secondMomentOfAreaRespectToCentroid>
//   expolys <n> { <number of holes> <contour> <holes> } -> <ok> <smExpolysX> <smExpolysY>

#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/Polygon.hpp"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>

namespace Slic3r {
// Brim.cpp:642, 661-667, 670, 696: defined there, declared by no header.
bool compSecondMoment(Polygon poly, Vec2d &sm);
struct ExPolyProp
{
    double aera = 0;
    Vec2d  centroid;
    Vec2d  secondMomentOfAreaRespectToCentroid;
};
bool compSecondMoment(const ExPolygon &expoly, ExPolyProp &expolyProp);
bool compSecondMoment(const ExPolygons &expolys, double &smExpolysX, double &smExpolysY);
} // namespace Slic3r

using namespace Slic3r;

static std::string hD(double v)
{
    uint64_t u;
    std::memcpy(&u, &v, 8);
    char b[32];
    std::snprintf(b, sizeof(b), "%016llx", (unsigned long long) u);
    return b;
}
static std::string N(long long v) { return std::to_string(v); }
static std::string POLY(const Polygon &p)
{
    std::string s = N(p.points.size());
    for (const Point &q : p.points) s += " " + N(q.x()) + " " + N(q.y());
    return s;
}
static std::string EX(const ExPolygon &e)
{
    std::string s = N(e.holes.size()) + " " + POLY(e.contour);
    for (const Polygon &h : e.holes) s += " " + POLY(h);
    return s;
}

struct Rng
{
    uint64_t s;
    double   next()
    {
        s = s * 6364136223846793005ull + 1442695040888963407ull;
        return double((s >> 33) & 0xFFFFFF) / double(0x1000000);
    }
    double range(double a, double b) { return a + (b - a) * next(); }
};

// A star-shaped polygon around (cx, cy), counter-clockwise.
static Polygon star(Rng &r, double cx, double cy, double rmin, double rmax, int n)
{
    Polygon p;
    for (int i = 0; i < n; ++i) {
        const double a = 2. * M_PI * (i + r.range(-0.3, 0.3)) / n;
        const double d = r.range(rmin, rmax);
        p.points.emplace_back(coord_t(std::lround(cx + d * std::cos(a))), coord_t(std::lround(cy + d * std::sin(a))));
    }
    return p;
}

static ExPolygon make_ex(Rng &r, double cx, double cy, double size, int holes)
{
    ExPolygon e;
    e.contour = star(r, cx, cy, 0.6 * size, size, 5 + int(r.range(0, 12)));
    for (int h = 0; h < holes; ++h) {
        const double a = 2. * M_PI * h / holes;
        Polygon      hole = star(r, cx + 0.3 * size * std::cos(a), cy + 0.3 * size * std::sin(a), 0.03 * size, 0.08 * size, 4 + int(r.range(0, 5)));
        hole.reverse();
        e.holes.emplace_back(hole);
    }
    return e;
}

int main()
{
    Rng r{777ull};
    // Polygons: counter-clockwise, clockwise, degenerate.
    for (int i = 0; i < 40; ++i) {
        Polygon p = star(r, r.range(-2e7, 2e7), r.range(-2e7, 2e7), 1e5 * (1 + i), 3e5 * (1 + i), 3 + i % 17);
        if (i % 3 == 1) p.reverse();
        Vec2d sm(1., 2.);
        bool  ok = compSecondMoment(p, sm);
        std::printf("poly %s -> %d %s %s\n", POLY(p).c_str(), ok ? 1 : 0, hD(sm.x()).c_str(), hD(sm.y()).c_str());
    }
    {
        Polygon two;
        two.points = {Point(0, 0), Point(1000000, 500000)};
        Vec2d sm(1., 2.);
        bool  ok = compSecondMoment(two, sm);
        std::printf("poly %s -> %d %s %s\n", POLY(two).c_str(), ok ? 1 : 0, hD(sm.x()).c_str(), hD(sm.y()).c_str());
        Polygon rect;
        rect.points = {Point(0, 0), Point(20000000, 0), Point(20000000, 1000000), Point(0, 1000000)};
        ok = compSecondMoment(rect, sm);
        std::printf("poly %s -> %d %s %s\n", POLY(rect).c_str(), ok ? 1 : 0, hD(sm.x()).c_str(), hD(sm.y()).c_str());
    }
    // ExPolygons.
    ExPolygons all;
    for (int i = 0; i < 30; ++i) {
        ExPolygon   e = make_ex(r, r.range(-1e7, 1e7), r.range(-1e7, 1e7), 5e5 * (1 + i), i % 4);
        ExPolyProp  prop;
        prop.centroid = Vec2d(0., 0.);
        prop.secondMomentOfAreaRespectToCentroid = Vec2d(0., 0.);
        bool ok = compSecondMoment(e, prop);
        std::printf("expoly %s -> %d %s %s %s %s %s\n", EX(e).c_str(), ok ? 1 : 0, hD(prop.aera).c_str(), hD(prop.centroid.x()).c_str(), hD(prop.centroid.y()).c_str(),
                    hD(prop.secondMomentOfAreaRespectToCentroid.x()).c_str(), hD(prop.secondMomentOfAreaRespectToCentroid.y()).c_str());
        all.emplace_back(e);
    }
    // Groups of ExPolygons.
    for (int i = 0; i < 20; ++i) {
        ExPolygons  group;
        std::string s;
        const int   n = i % 6;
        for (int k = 0; k < n; ++k) {
            ExPolygon e = all[(i * 7 + k * 3) % all.size()];
            e.translate(Point(coord_t(k * 30000000), coord_t(i * 1000000)));
            s += " " + EX(e);
            group.emplace_back(e);
        }
        if (i == 13) {
            ExPolygon e;
            e.contour.points = {Point(0, 0), Point(5, 5)};
            s += " " + EX(e);
            group.emplace_back(e);
        }
        double ix = -1., iy = -2.;
        bool   ok = compSecondMoment(group, ix, iy);
        std::printf("expolys %d%s -> %d %s %s\n", int(group.size()), s.c_str(), ok ? 1 : 0, hD(ix).c_str(), hD(iy).c_str());
    }
    return 0;
}
