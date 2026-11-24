// Vectors for tests/skeletal_trapezoidation_construct_native_parity.rs: the pin's
// SkeletalTrapezoidation constructor (constructFromPolygons, SkeletalTrapezoidation.cpp:374-516:
// the Voronoi diagram, computePointCellRange / compute_segment_cell_range, transferEdge,
// makeNode, makeRib, separatePointyQuadEndNodes, collapseSmallEdges and the incident edges)
// over random polygon sets with holes, and SkeletalTrapezoidation::discretize
// (SkeletalTrapezoidation.cpp:218-328) on every finite Voronoi edge of the same diagram.
//
// Output (stdout), per case:
//   c <case> <polys> <hole compensation 0/1> <hole index count> <hole indices...>
//   p <points> x y x y ...                          one line per polygon
//   g <nodes> <edges>
//   n x y distance_to_boundary bead_count incident  graph.nodes in list order
//   e from to twin prev next type hole_flag central graph.edges in list order (-1 = null)
//   d <edge index> <points> x y ...                 discretize(vd.edges()[i], segments) for the finite
//                                                   edges with both ends in the polygons' bounding box
// Node and edge references are indices in list order.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <list>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#define protected public
#define private public
#include "libslic3r/Arachne/SkeletalTrapezoidation.hpp"
#include "libslic3r/Geometry/VoronoiUtils.hpp"
#undef protected
#undef private
#include "libslic3r/Arachne/BeadingStrategy/BeadingStrategyFactory.hpp"

using namespace Slic3r;
using namespace Slic3r::Arachne;

static uint64_t g_state = 0x9E3779B97F4A7C15ULL;
static uint64_t next_u64()
{
    g_state = g_state * 6364136223846793005ULL + 1442695040888963407ULL;
    return g_state >> 11;
}
static int64_t range(int64_t lo, int64_t hi) { return lo + int64_t(next_u64() % uint64_t(hi - lo + 1)); }
static double unit() { return double(next_u64() >> 1) / double(1ULL << 52); }

// A simple polygon around (cx, cy): n angles in order with jitter below half the spacing,
// radii in [rmin, rmax]. Counter-clockwise; `hole` reverses it.
static Polygon star(int64_t cx, int64_t cy, int n, double rmin, double rmax, bool hole)
{
    Polygon p;
    const double step = 2. * M_PI / n;
    for (int i = 0; i < n; ++i) {
        double a = step * i + (unit() - 0.5) * 0.8 * step;
        double r = rmin + unit() * (rmax - rmin);
        p.points.emplace_back(cx + int64_t(std::lrint(r * std::cos(a))), cy + int64_t(std::lrint(r * std::sin(a))));
    }
    if (hole)
        std::reverse(p.points.begin(), p.points.end());
    return p;
}

static Polygon rect(int64_t x0, int64_t y0, int64_t x1, int64_t y1, bool hole)
{
    Polygon p{Point(x0, y0), Point(x1, y0), Point(x1, y1), Point(x0, y1)};
    if (hole)
        std::reverse(p.points.begin(), p.points.end());
    return p;
}

// A long thin strip with a jittered long side: parabolic and point-point edges along it.
static Polygon strip(int64_t len, int64_t w, int n)
{
    Polygon p;
    for (int i = 0; i <= n; ++i)
        p.points.emplace_back(len * i / n, range(-w / 8, w / 8));
    for (int i = n; i >= 0; --i)
        p.points.emplace_back(len * i / n, w + range(-w / 8, w / 8));
    return p;
}

static Polygons generate(int c, std::vector<int> &holes)
{
    Polygons polys;
    holes.clear();
    const double R = double(range(60000, 600000));
    switch (c % 5) {
    case 0: polys.push_back(star(0, 0, int(range(5, 40)), 0.6 * R, R, false)); break;
    case 1:
    case 4: {
        polys.push_back(star(range(-100000, 100000), range(-100000, 100000), int(range(8, 48)), 0.6 * R, R, false));
        const Point c0 = polys.front().centroid();
        const int nh = int(range(1, 3));
        for (int k = 0; k < nh; ++k) {
            const double a = 2. * M_PI * k / 3. + 0.3;
            holes.push_back(int(polys.size()));
            polys.push_back(star(c0.x() + int64_t(std::lrint(0.25 * R * std::cos(a))), c0.y() + int64_t(std::lrint(0.25 * R * std::sin(a))),
                                 int(range(3, 16)), 0.05 * R, 0.1 * R, true));
        }
        break;
    }
    case 2: {
        // Axis-aligned, on a coarse grid: co-circular points, secondary edges, equal distances.
        const int64_t g = range(1, 5) * 10000;
        const int64_t w = range(4, 30) * g, h = range(4, 30) * g;
        polys.push_back(rect(0, 0, w, h, false));
        if (range(0, 1)) {
            holes.push_back(1);
            const int64_t hx = range(1, w / g - 3) * g, hy = range(1, h / g - 3) * g;
            polys.push_back(rect(hx, hy, std::min(w - g, hx + range(1, 4) * g), std::min(h - g, hy + range(1, 4) * g), true));
        }
        break;
    }
    default: polys.push_back(strip(int64_t(range(300000, 3000000)), int64_t(range(20000, 120000)), int(range(4, 30)))); break;
    }
    return polys;
}

static void run_case(const BeadingStrategy &strategy, int c, const Polygons &polys, const std::vector<int> &holes, bool hc, coord_t step)
{
    SkeletalTrapezoidation st(polys, strategy, strategy.getTransitioningAngle(), step, 10000000, 2500, 40000, hc, holes);

    printf("c %d %zu %d %zu", c, polys.size(), int(hc), holes.size());
    for (int h : holes)
        printf(" %d", h);
    printf(" %lld\n", (long long) step);
    for (const Polygon &p : polys) {
        printf("p %zu", p.size());
        for (const Point &q : p.points)
            printf(" %lld %lld", (long long) q.x(), (long long) q.y());
        printf("\n");
    }
    std::unordered_map<const void *, int> ni, ei;
    int                                   k = 0;
    for (auto &n : st.graph.nodes)
        ni[&n] = k++;
    k = 0;
    for (auto &e : st.graph.edges)
        ei[&e] = k++;
    auto nidx = [&](const void *p) { return p ? ni.at(p) : -1; };
    auto eidx = [&](const void *p) { return p ? ei.at(p) : -1; };
    printf("g %zu %zu\n", st.graph.nodes.size(), st.graph.edges.size());
    for (auto &n : st.graph.nodes)
        printf("n %lld %lld %lld %lld %d\n", (long long) n.p.x(), (long long) n.p.y(), (long long) n.data.distance_to_boundary,
               (long long) n.data.bead_count, eidx(n.incident_edge));
    for (auto &e : st.graph.edges)
        printf("e %d %d %d %d %d %d %d %d\n", nidx(e.from), nidx(e.to), eidx(e.twin), eidx(e.prev), eidx(e.next), int(e.data.type),
               int(e.data.getHoleCompensationFlag()), int(e.data.is_central));

    // discretize on the diagram constructFromPolygons builds (SkeletalTrapezoidation.cpp:419-424).
    std::vector<PolygonsSegmentIndex> segments;
    for (size_t poly_idx = 0; poly_idx < polys.size(); poly_idx++)
        for (size_t point_idx = 0; point_idx < polys[poly_idx].size(); point_idx++)
            segments.emplace_back(&polys, poly_idx, point_idx);
    Geometry::VoronoiDiagram vd;
    vd.construct_voronoi(segments.cbegin(), segments.cend());
    const auto &edges = vd.edges();
    const BoundingBox bbox = get_extents(polys);
    for (size_t i = 0; i < edges.size(); ++i) {
        const VD::edge_type &edge = edges[i];
        if (!edge.is_finite() || !Geometry::VoronoiUtils::is_in_range<coord_t>(edge))
            continue;
        // The edges inside the polygons' bounding box (the ones outside are far away and long).
        if (!bbox.contains(Geometry::VoronoiUtils::to_point(edge.vertex0()).cast<coord_t>()) ||
            !bbox.contains(Geometry::VoronoiUtils::to_point(edge.vertex1()).cast<coord_t>()))
            continue;
        Points d = st.discretize(edge, segments);
        printf("d %zu %zu", i, d.size());
        for (const Point &q : d)
            printf(" %lld %lld", (long long) q.x(), (long long) q.y());
        printf("\n");
    }
}

int main(int argc, char **argv)
{
    const int cases = argc > 1 ? atoi(argv[1]) : 150;
    // The strategy WallToolPaths makes for a 0.42 mm line (wall_transition_angle 10 degrees as a float,
    // WallToolPaths.hpp:29); the construction reads only its transitioning angle.
    BeadingStrategyPtr strategy = BeadingStrategyFactory::makeStrategy(42000, 42000, 40000, float(M_PI / 18.0), true, 35700, 10500,
                                                                       0.7, 0.85, 4, 0, 1, 0.5);
    for (int c = 0; c < cases; ++c) {
        std::vector<int> holes;
        Polygons         polys = generate(c, holes);
        const bool       hc    = !holes.empty() && range(0, 1) == 1;
        const coord_t    step  = coord_t(range(2, 16)) * 10000;
        run_case(*strategy, c, polys, holes, hc, step);
    }
    // Real outlines (argv[2], lines "o <name> <polygon> x y"): the Voronoi input of WallToolPaths calls on
    // Majora (layer 110 call 1, layer 1 call 1, as the pin prepares them), with the live
    // discretization step scaled(0.8) (WallToolPaths.cpp:456) and no hole compensation.
    if (argc > 2) {
        FILE *f = std::fopen(argv[2], "r");
        if (!f) { std::fprintf(stderr, "cannot open %s\n", argv[2]); return 1; }
        std::vector<std::pair<std::string, Polygons>> sets;
        char name[64]; int pi; long long x, y;
        while (std::fscanf(f, " o %63s %d %lld %lld", name, &pi, &x, &y) == 4) {
            if (sets.empty() || sets.back().first != name) sets.emplace_back(name, Polygons());
            Polygons &ps = sets.back().second;
            while (int(ps.size()) <= pi) ps.emplace_back();
            ps[pi].points.emplace_back(coord_t(x), coord_t(y));
        }
        std::fclose(f);
        int c = cases;
        for (const auto &s : sets)
            run_case(*strategy, c++, s.second, {}, false, 80000);
    }
    return 0;
}
