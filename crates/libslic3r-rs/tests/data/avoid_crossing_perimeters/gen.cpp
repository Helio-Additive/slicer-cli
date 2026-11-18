// Generator of the vectors of tests/avoid_crossing_perimeters_native_parity.rs: the functions of
// GCode/AvoidCrossingPerimeters.cpp at the pin (references/BambuStudio/src/libslic3r) that need
// neither a Layer nor a GCode, on generated polygons.
//
// Build as tests/data/config/gen.cpp says (the compile line of a libslic3r source of a built tree
// of the Bambu Studio engine, linked with its libslic3r_core and the libraries that links), then
//   ./gen > vectors.txt && gzip -9n vectors.txt
//
// The functions are `static` in the C++, so this file includes AvoidCrossingPerimeters.cpp itself;
// the linker then takes every symbol of that unit from here and leaves the library's copy alone.
// The vectors were made on macOS arm64 with Apple clang (-ffp-contract=on) and libc++ (std::sort).
//
// One line per case, tokens separated by one space; a float is its 8 hexadecimal digits, a double
// its 16. A ring is its point count and the points; an expolygon list is its count, then per
// expolygon the contour, the hole count and the holes; a polygon list is its count and the rings.
//   INNER <offset_dis> <expolygons> -> <expolygons>
//   PARAMS <polygons> -> <count> then per polygon <count> <floats>
//   CLOSE <cx> <cy> <radius> <polygons> -> <count> (<border> <line> <x> <y>)*
//   EXTEND <sx> <sy> <ex> <ey> <radius> <polygons> -> <count> (<border> <line> <x> <y> <distance> <do_not_remove>)*
//   SIMPL <polygons> <count> (<x> <y> <border> <do_not_remove>)* -> <count> (<x> <y> <border> <do_not_remove>)*
//   ANY <expolygons> <count> (<x> <y>)* -> <0|1>      (two points: the Line overload)

#include "libslic3r/GCode/AvoidCrossingPerimeters.cpp"

#include <cinttypes>
#include <cstdint>
#include <cstdio>
#include <cstring>

using namespace Slic3r;

static uint64_t g_state = 0x243f6a8885a308d3ull;
static uint64_t rnd()
{
    g_state ^= g_state << 13;
    g_state ^= g_state >> 7;
    g_state ^= g_state << 17;
    return g_state;
}
static int64_t rnd_range(int64_t lo, int64_t hi) { return lo + int64_t(rnd() % uint64_t(hi - lo + 1)); }

static uint32_t bits(float v)
{
    uint32_t u;
    std::memcpy(&u, &v, 4);
    return u;
}
static uint64_t bits(double v)
{
    uint64_t u;
    std::memcpy(&u, &v, 8);
    return u;
}

// A rectangle (counter-clockwise) inside [x0, x0 + span] x [y0, y0 + span], on a grid of `grid` units.
static Polygon rnd_rect(coord_t ox, coord_t oy, coord_t span, coord_t grid)
{
    coord_t x0 = ox + rnd_range(0, span / grid) * grid, x1 = ox + rnd_range(0, span / grid) * grid;
    coord_t y0 = oy + rnd_range(0, span / grid) * grid, y1 = oy + rnd_range(0, span / grid) * grid;
    if (x0 > x1) std::swap(x0, x1);
    if (y0 > y1) std::swap(y0, y1);
    if (x1 - x0 < 4 * grid) x1 = x0 + 4 * grid;
    if (y1 - y0 < 4 * grid) y1 = y0 + 4 * grid;
    return Polygon{Point(x0, y0), Point(x1, y0), Point(x1, y1), Point(x0, y1)};
}

// A star-shaped polygon (counter-clockwise) around (cx, cy).
static Polygon rnd_blob(coord_t cx, coord_t cy, double rmin, double rmax)
{
    const int n = int(rnd_range(5, 28));
    Polygon   p;
    for (int i = 0; i < n; ++i) {
        const double a = 2. * M_PI * (double(i) + double(rnd_range(0, 700)) / 1000.) / double(n);
        const double r = rmin + (rmax - rmin) * double(rnd_range(0, 1000)) / 1000.;
        p.points.emplace_back(coord_t(cx + coord_t(r * std::cos(a))), coord_t(cy + coord_t(r * std::sin(a))));
    }
    return p;
}

// Disjoint expolygons, one per cell of a 3 x 3 board of 60 mm cells.
static ExPolygons rnd_expolygons(int max_count)
{
    ExPolygons out;
    const int  n = int(rnd_range(1, max_count));
    for (int i = 0; i < n; ++i) {
        const coord_t ox = coord_t(i % 3) * 6000000, oy = coord_t(i / 3) * 6000000;
        ExPolygon     e;
        if (rnd() % 2 == 0) {
            e.contour = rnd_blob(ox + 3000000, oy + 3000000, 400000., 2700000.);
            if (rnd() % 3 == 0) {
                Polygon h = rnd_blob(ox + 3000000, oy + 3000000, 50000., 300000.);
                h.reverse();
                e.holes.push_back(h);
            }
        } else {
            e.contour = rnd_rect(ox + 200000, oy + 200000, 5400000, rnd() % 2 ? 100000 : 12345);
            const BoundingBox bb = e.contour.bounding_box();
            const coord_t     w = bb.max.x() - bb.min.x(), h = bb.max.y() - bb.min.y();
            const int         holes = int(rnd_range(0, 2));
            for (int k = 0; k < holes; ++k)
                if (w > 1200000 && h > 600000) {
                    // side by side, clockwise; some are small enough for the 1 mm offset to close them
                    const coord_t hw = (k == 0) ? w / 6 : coord_t(rnd_range(20000, 260000));
                    const coord_t x0 = bb.min.x() + w / 8 + coord_t(k) * (w / 2), y0 = bb.min.y() + h / 3;
                    const coord_t x1 = x0 + hw, y1 = y0 + std::min<coord_t>(hw, h / 4);
                    e.holes.push_back(Polygon{Point(x0, y0), Point(x0, y1), Point(x1, y1), Point(x1, y0)});
                }
        }
        out.push_back(std::move(e));
    }
    return out;
}

static void print_ring(const Polygon &p)
{
    std::printf(" %zu", p.points.size());
    for (const Point &q : p.points)
        std::printf(" %lld %lld", (long long) q.x(), (long long) q.y());
}
static void print_expolygons(const ExPolygons &areas)
{
    std::printf(" %zu", areas.size());
    for (const ExPolygon &e : areas) {
        print_ring(e.contour);
        std::printf(" %zu", e.holes.size());
        for (const Polygon &h : e.holes)
            print_ring(h);
    }
}
static void print_polygons(const Polygons &polys)
{
    std::printf(" %zu", polys.size());
    for (const Polygon &p : polys)
        print_ring(p);
}

static Point rnd_point() { return Point(coord_t(rnd_range(-500000, 18500000)), coord_t(rnd_range(-500000, 18500000))); }
// A point inside `bb`: EdgeGrid::Grid::visit_cells_intersecting_line takes end points inside the grid's
// bounding box only (EdgeGrid.hpp:294-301), which AvoidCrossingPerimeters::travel_to checks before it
// calls avoid_perimeters (AvoidCrossingPerimeters.cpp:1388, 1403).
static Point rnd_point_in(const BoundingBox &bb)
{
    return Point(coord_t(rnd_range(bb.min.x(), bb.max.x())), coord_t(rnd_range(bb.min.y(), bb.max.y())));
}

int main()
{
    for (int i = 0; i < 260; ++i) {
        const ExPolygons in         = rnd_expolygons(i % 4 == 0 ? 1 : 4);
        const double     offset_dis = double(rnd_range(20000, 160000)) + (i % 3 == 0 ? 0.5 : 0.);
        std::printf("INNER %016" PRIx64, bits(offset_dis));
        print_expolygons(in);
        const ExPolygons out = inner_offset(in, offset_dis);
        std::printf(" ->");
        print_expolygons(out);
        std::printf("\n");
    }

    for (int i = 0; i < 500; ++i) {
        const Polygons polys = to_polygons(rnd_expolygons(6));
        const BoundingBox extents = get_extents(polys);
        AvoidCrossingPerimeters::Boundary boundary;
        {
            Polygons copy = polys;
            init_boundary(&boundary, std::move(copy));
        }
        if (i % 5 == 0) {
            std::printf("PARAMS");
            print_polygons(polys);
            std::printf(" -> %zu", boundary.boundaries_params.size());
            for (const std::vector<float> &params : boundary.boundaries_params) {
                std::printf(" %zu", params.size());
                for (float v : params)
                    std::printf(" %08x", bits(v));
            }
            std::printf("\n");
        }
        {
            // near a vertex or anywhere
            Point center = rnd_point();
            if (rnd() % 2 == 0) {
                const Polygon &p = polys[rnd() % polys.size()];
                center = p.points[rnd() % p.points.size()] + Point(coord_t(rnd_range(-40000, 40000)), coord_t(rnd_range(-40000, 40000)));
            }
            const float radius = float(rnd_range(20000, 400000)) + (i % 2 ? 0.5f : 0.f);
            std::printf("CLOSE %lld %lld %08x", (long long) center.x(), (long long) center.y(), bits(radius));
            print_polygons(polys);
            const std::vector<ClosestLine> lines = get_closest_lines_in_radius(boundary.grid, center, radius);
            std::printf(" -> %zu", lines.size());
            for (const ClosestLine &l : lines)
                std::printf(" %zu %zu %lld %lld", l.border_idx, l.line_idx, (long long) l.point.x(), (long long) l.point.y());
            std::printf("\n");
        }
        {
            const Point start = rnd_point_in(extents), end = rnd_point_in(extents);
            const float radius = float(rnd_range(20000, 300000));
            std::vector<Intersection> intersections;
            {
                AllIntersectionsVisitor visitor(boundary.grid, intersections, Line(start, end));
                boundary.grid.visit_cells_intersecting_line(start, end, visitor);
                Vec2d dir = (end - start).cast<double>();
                for (Intersection &intersection : intersections) {
                    float dist_from_line_begin = (intersection.point - boundary.boundaries[intersection.border_idx][intersection.line_idx]).cast<float>().norm();
                    intersection.distance      = boundary.boundaries_params[intersection.border_idx][intersection.line_idx] + dist_from_line_begin;
                }
                std::sort(intersections.begin(), intersections.end(),
                          [dir](const auto &l, const auto &r) { return (r.point - l.point).template cast<double>().dot(dir) > 0.; });
            }
            const std::vector<Intersection> out = extend_for_closest_lines(intersections, boundary, start, end, radius);
            std::printf("EXTEND %lld %lld %lld %lld %08x", (long long) start.x(), (long long) start.y(), (long long) end.x(), (long long) end.y(), bits(radius));
            print_polygons(polys);
            std::printf(" -> %zu", out.size());
            for (const Intersection &x : out)
                std::printf(" %zu %zu %lld %lld %08x %d", x.border_idx, x.line_idx, (long long) x.point.x(), (long long) x.point.y(), bits(x.distance),
                            x.do_not_remove ? 1 : 0);
            std::printf("\n");
        }
        {
            // start, a run of vertices of one or two boundaries pushed inwards by a little, end
            std::vector<TravelPoint> travel;
            travel.push_back({rnd_point_in(extents), -1});
            const int runs = int(rnd_range(1, 2));
            for (int r = 0; r < runs; ++r) {
                const size_t   b = rnd() % polys.size();
                const Polygon &p = polys[b];
                size_t         k = rnd() % p.points.size();
                const int      n = int(rnd_range(1, int(p.points.size())));
                for (int j = 0; j < n; ++j, k = (k + 1) % p.points.size())
                    travel.push_back({get_polygon_vertex_offset(p, k, coord_t(SCALED_EPSILON)), int(b), rnd() % 9 == 0});
            }
            travel.push_back({rnd_point_in(extents), -1});
            const std::vector<TravelPoint> out = simplify_travel(boundary, travel);
            std::printf("SIMPL");
            print_polygons(polys);
            std::printf(" %zu", travel.size());
            for (const TravelPoint &t : travel)
                std::printf(" %lld %lld %d %d", (long long) t.point.x(), (long long) t.point.y(), t.border_idx, t.do_not_remove ? 1 : 0);
            std::printf(" -> %zu", out.size());
            for (const TravelPoint &t : out)
                std::printf(" %lld %lld %d %d", (long long) t.point.x(), (long long) t.point.y(), t.border_idx, t.do_not_remove ? 1 : 0);
            std::printf("\n");
        }
    }

    for (int i = 0; i < 1500; ++i) {
        const ExPolygons         ex_polygons = rnd_expolygons(5);
        std::vector<BoundingBox> bboxes;
        for (const ExPolygon &e : ex_polygons)
            bboxes.emplace_back(get_extents(e));
        BoundingBox bbox_slice(get_extents(ex_polygons));
        bbox_slice.offset(SCALED_EPSILON);
        EdgeGrid::Grid grid;
        grid.set_bbox(bbox_slice);
        grid.create(ex_polygons, coord_t(scale_(1.)));
        // points inside one expolygon's bounding box most of the time
        const BoundingBox &bb = bboxes[rnd() % bboxes.size()];
        Points             pts;
        const int          n = (i % 2 == 0) ? 2 : int(rnd_range(3, 6));
        for (int k = 0; k < n; ++k)
            pts.emplace_back(rnd() % 7 == 0 ? rnd_point() :
                                              Point(coord_t(rnd_range(bb.min.x(), bb.max.x())), coord_t(rnd_range(bb.min.y(), bb.max.y()))));
        bool res;
        if (n == 2)
            res = any_expolygon_contains(ex_polygons, bboxes, grid, Line(pts[0], pts[1]));
        else
            res = any_expolygon_contains(ex_polygons, bboxes, grid, Polyline(pts));
        std::printf("ANY");
        print_expolygons(ex_polygons);
        std::printf(" %zu", pts.size());
        for (const Point &p : pts)
            std::printf(" %lld %lld", (long long) p.x(), (long long) p.y());
        std::printf(" -> %d\n", res ? 1 : 0);
    }
    return 0;
}
