// Native vectors for SeamPlacer's PerimeterDistancer (SeamPlacer.cpp:886-921).
// The class lives in SeamPlacer.cpp's SeamPlacerImpl namespace and is built from a Layer;
// this generator carries the class body verbatim, with the outline handed in where the pin
// reads `layer->lslices` (SeamPlacer.cpp:894), and calls the pin's AABBTreeLines.
// Output: "E <expolygon index> <h> <n> x y ..." (h = -1 for the contour, else the hole index),
// then "Q <x bits> <y bits> <distance bits>" per query (raw float bits, %08x).
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>
#include <cmath>
#include "libslic3r/AABBTreeLines.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/Line.hpp"

using namespace Slic3r;

class PerimeterDistancer
{
    std::vector<Linef>                lines;
    AABBTreeIndirect::Tree<2, double> tree;

public:
    PerimeterDistancer(const ExPolygons &layer_lslices)
    {
        ExPolygons layer_outline = layer_lslices;
        for (const ExPolygon &island : layer_outline) {
            assert(island.contour.is_counter_clockwise());
            for (const auto &line : island.contour.lines()) { lines.emplace_back(unscale(line.a), unscale(line.b)); }
            for (const Polygon &hole : island.holes) {
                assert(hole.is_clockwise());
                for (const auto &line : hole.lines()) { lines.emplace_back(unscale(line.a), unscale(line.b)); }
            }
        }
        tree = AABBTreeLines::build_aabb_tree_over_indexed_lines(lines);
    }

    float distance_from_perimeter(const Vec2f &point) const
    {
        Vec2d  p = point.cast<double>();
        size_t hit_idx_out{};
        Vec2d  hit_point_out = Vec2d::Zero();
        auto   distance      = AABBTreeLines::squared_distance_to_indexed_lines(lines, tree, p, hit_idx_out, hit_point_out);
        if (distance < 0) { return std::numeric_limits<float>::max(); }

        distance          = sqrt(distance);
        const Linef &line = lines[hit_idx_out];
        Vec2d        v1   = line.b - line.a;
        Vec2d        v2   = p - line.a;
        if ((v1.x() * v2.y()) - (v1.y() * v2.x()) > 0.0) { distance *= -1; }
        return distance;
    }
};

static uint64_t g_seed = 0x9E3779B97F4A7C15ull;
static uint32_t lcg() { g_seed = g_seed * 6364136223846793005ull + 1442695040888963407ull; return uint32_t(g_seed >> 33); }
static double   unit() { return double(lcg()) / 2147483648.0; }
static unsigned fb(float f) { unsigned u; std::memcpy(&u, &f, 4); return u; }

// A star-shaped polygon around (cx, cy), n vertices, radii in [r0, r1] (mm).
static Polygon star(double cx, double cy, int n, double r0, double r1)
{
    Polygon p;
    for (int i = 0; i < n; ++i) {
        double a = 2. * M_PI * (double(i) + 0.3 * unit()) / double(n);
        double r = r0 + (r1 - r0) * unit();
        p.points.emplace_back(coord_t(std::llround((cx + r * std::cos(a)) / SCALING_FACTOR)), coord_t(std::llround((cy + r * std::sin(a)) / SCALING_FACTOR)));
    }
    return p;
}

int main()
{
    for (int scenario = 0; scenario < 24; ++scenario) {
        ExPolygons outline;
        int islands = 1 + int(lcg() % 3);
        for (int k = 0; k < islands; ++k) {
            double    cx = -60. + 120. * unit(), cy = -60. + 120. * unit();
            ExPolygon ex;
            ex.contour = star(cx, cy, 5 + int(lcg() % 60), 8. + 4. * unit(), 14. + 10. * unit());
            ex.contour.make_counter_clockwise();
            if (lcg() % 2) {
                Polygon h = star(cx, cy, 3 + int(lcg() % 12), 1. + unit(), 3. + 2. * unit());
                h.make_clockwise();
                ex.holes.push_back(h);
            }
            outline.push_back(ex);
        }
        std::printf("S %d %zu\n", scenario, outline.size());
        for (size_t e = 0; e < outline.size(); ++e) {
            for (long h = -1; h < long(outline[e].holes.size()); ++h) {
                const Polygon &pg = h < 0 ? outline[e].contour : outline[e].holes[h];
                std::printf("E %zu %ld %zu", e, h, pg.points.size());
                for (const Point &q : pg.points) std::printf(" %lld %lld", (long long) q.x(), (long long) q.y());
                std::printf("\n");
            }
        }
        PerimeterDistancer d(outline);
        for (int q = 0; q < 200; ++q) {
            // Near a random vertex of a random island, as seam candidates are, plus far points.
            const ExPolygon &ex = outline[lcg() % outline.size()];
            const Point     &v  = ex.contour.points[lcg() % ex.contour.points.size()];
            float            x  = float(unscale<double>(v.x()) + (q % 5 == 0 ? 30. : 1.5) * (2. * unit() - 1.));
            float            y  = float(unscale<double>(v.y()) + (q % 5 == 0 ? 30. : 1.5) * (2. * unit() - 1.));
            float            r  = d.distance_from_perimeter(Vec2f(x, y));
            std::printf("Q %08x %08x %08x\n", fb(x), fb(y), fb(r));
        }
    }
    return 0;
}
