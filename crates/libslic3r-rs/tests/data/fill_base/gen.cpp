// Generator of the vectors of tests/fill_base_native_parity.rs: libslic3r's Fill/FillBase.cpp at the pin
// (references/BambuStudio/src/libslic3r; the override layer has no copy of it).
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
//   ./gen vectors.txt
//
// FillBase.cpp is compiled into this file (most of its functions are static), with the protected members
// of the classes made public.
//
// Groups: "asp" (Fill::_adjust_solid_spacing), "idir" (Fill::_infill_direction of a rectilinear filler),
// "ml" (multiline_fill), "ci" (Fill::connect_infill over an expolygon), "cip" (Fill::connect_infill over
// polygons and a bounding box) and "cbs" (Fill::connect_base_support), on boundaries and line sets made from
// a fixed linear congruential generator: the lines are parallel lines clipped by the boundary, as the
// fillers hand them over.
//
// Lines that start with "in" are the inputs the Rust test reads back; the others are results it has to
// reproduce. Tokens are separated by one space; a double is its 16 hexadecimal digits, a float its 8, an
// integer decimal. The vectors were made on macOS arm64 with Apple clang and Eigen 5.0.1.

#include <algorithm>
#include <cfloat>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <map>
#include <memory>
#include <string>
#include <vector>

// Before the access specifiers are redefined: tbb declares nested classes ahead of their definition.
#include <tbb/blocked_range.h>
#include <tbb/concurrent_unordered_map.h>
#include <tbb/concurrent_unordered_set.h>
#include <tbb/concurrent_vector.h>
#include <tbb/global_control.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_for_each.h>
#include <tbb/scalable_allocator.h>
#include <tbb/spin_mutex.h>
#include <tbb/task_group.h>

#include <boost/container/static_vector.hpp>

#define private public
#define protected public
#include "libslic3r/Fill/FillBase.cpp"
#undef protected
#undef private

using namespace Slic3r;

static FILE *g_out = nullptr;

static std::string hD(double v)
{
    uint64_t u;
    std::memcpy(&u, &v, 8);
    char b[32];
    std::snprintf(b, sizeof(b), "%016llx", (unsigned long long) u);
    return b;
}
static std::string hF(float v)
{
    uint32_t u;
    std::memcpy(&u, &v, 4);
    char b[16];
    std::snprintf(b, sizeof(b), "%08x", (unsigned) u);
    return b;
}
static std::string hI(long long v) { return std::to_string(v); }
static std::string hB(bool v) { return v ? "1" : "0"; }
static void out(const std::string &s) { std::fprintf(g_out, "%s\n", s.c_str()); }
static std::string PT(const Point &p) { return hI(p.x()) + " " + hI(p.y()); }
static std::string PTS(const Points &pts)
{
    std::string o = hI((long long) pts.size());
    for (const Point &pt : pts) o += " " + PT(pt);
    return o;
}
static std::string POLY(const Polygon &p) { return PTS(p.points); }
static std::string POLYS(const Polygons &ps)
{
    std::string o = hI((long long) ps.size());
    for (const Polygon &p : ps) o += " " + POLY(p);
    return o;
}
static std::string EXPOLY(const ExPolygon &e)
{
    std::string o = POLY(e.contour) + " " + hI((long long) e.holes.size());
    for (const Polygon &h : e.holes) o += " " + POLY(h);
    return o;
}
static std::string PLS(const Polylines &ps)
{
    std::string o = hI((long long) ps.size());
    for (const Polyline &p : ps) o += " " + PTS(p.points);
    return o;
}

// The generator of the inputs.
static uint64_t g_seed = 0x9E3779B97F4A7C15ULL;
static uint32_t lcg()
{
    g_seed = g_seed * 6364136223846793005ULL + 1442695040888963407ULL;
    return uint32_t(g_seed >> 33);
}
static int irand(int lo, int hi) { return lo + int(lcg() % uint32_t(hi - lo + 1)); }

static Polygon rect(coord_t x0, coord_t y0, coord_t x1, coord_t y1)
{
    Polygon p;
    p.points = { Point(x0, y0), Point(x1, y0), Point(x1, y1), Point(x0, y1) };
    return p;
}
static Polygon ccw(Polygon p)
{
    if (! p.is_counter_clockwise()) p.reverse();
    return p;
}
// An irregular polygon around a centre, by rational directions (no trigonometry).
static Polygon blob(coord_t cx, coord_t cy, coord_t rx, coord_t ry)
{
    static const int dirs[12][2] = { { 4, 0 }, { 4, 2 }, { 3, 3 }, { 2, 4 }, { 0, 4 }, { -3, 3 }, { -4, 1 }, { -4, 0 }, { -3, -3 }, { 0, -4 }, { 2, -4 }, { 3, -3 } };
    Polygon p;
    for (int i = 0; i < 12; ++ i) {
        const int r = irand(70, 100);
        p.points.emplace_back(Point(cx + coord_t(int64_t(rx) * dirs[i][0] * r / 800), cy + coord_t(int64_t(ry) * dirs[i][1] * r / 800)));
    }
    return ccw(p);
}

static const coord_t MM = 100000;

// A boundary: a contour of 4 to 30 mm with up to two holes inside.
static ExPolygon rnd_boundary()
{
    const coord_t ox = coord_t(irand(-30, 30)) * MM + irand(-5000, 5000), oy = coord_t(irand(-30, 30)) * MM + irand(-5000, 5000);
    const coord_t w = coord_t(irand(40, 300)) * (MM / 10), h = coord_t(irand(40, 300)) * (MM / 10);
    ExPolygon     e;
    switch (irand(0, 3)) {
    case 0: e.contour = rect(ox, oy, ox + w, oy + h); break;
    case 1: e.contour = blob(ox + w / 2, oy + h / 2, w, h); break;
    case 2:
        // an L
        e.contour.points = { Point(ox, oy), Point(ox + w, oy), Point(ox + w, oy + h / 3), Point(ox + w / 3, oy + h / 3), Point(ox + w / 3, oy + h), Point(ox, oy + h) };
        break;
    default:
        // a slanted quadrangle
        e.contour.points = { Point(ox, oy), Point(ox + w, oy + h / 5), Point(ox + w - w / 6, oy + h), Point(ox + w / 8, oy + h - h / 7) };
        break;
    }
    e.contour = ccw(e.contour);
    const int n_holes = irand(0, 2);
    for (int i = 0; i < n_holes; ++ i) {
        // In the lower left third of the contour, one beside the other: inside every shape above.
        const coord_t hw = w / 12, hh = h / 12;
        const coord_t hx = ox + w / 10 + coord_t(i) * (hw + w / 20), hy = oy + h / 10;
        Polygon       hole = irand(0, 1) == 0 ? rect(hx, hy, hx + hw, hy + hh) : blob(hx + hw / 2, hy + hh / 2, hw, hh);
        hole = ccw(hole);
        hole.reverse();
        // Only when it lies inside the contour.
        bool inside = true;
        for (const Point &q : hole.points) inside = inside && e.contour.contains(q);
        if (inside) e.holes.emplace_back(hole);
    }
    return e;
}

// Parallel lines `pitch` apart, in the direction (dx, dy), clipped by the boundary.
static Polylines rnd_lines(const ExPolygon &boundary, coord_t pitch, int dir)
{
    const BoundingBox bb = get_extents(boundary.contour);
    Polylines         lines;
    const coord_t     ext = (bb.max.x() - bb.min.x()) + (bb.max.y() - bb.min.y());
    if (dir == 0) {
        // vertical
        for (coord_t x = bb.min.x() + pitch / 2; x < bb.max.x(); x += pitch) lines.emplace_back(Polyline(Point(x, bb.min.y() - MM), Point(x, bb.max.y() + MM)));
    } else if (dir == 1) {
        // horizontal
        for (coord_t y = bb.min.y() + pitch / 2; y < bb.max.y(); y += pitch) lines.emplace_back(Polyline(Point(bb.min.x() - MM, y), Point(bb.max.x() + MM, y)));
    } else {
        // diagonal, rising or falling
        const coord_t sgn = dir == 2 ? 1 : -1;
        for (coord_t c = -ext; c < ext; c += pitch + pitch / 2) {
            const Point a(bb.min.x() - MM, (sgn > 0 ? bb.min.y() : bb.max.y()) + sgn * (c - MM));
            const Point b(a.x() + 2 * ext, a.y() + sgn * 2 * ext);
            lines.emplace_back(Polyline(a, b));
        }
    }
    return intersection_pl(lines, to_polygons(boundary));
}

static std::string PARAMS(const FillParams &p) { return hF(p.density) + " " + hF(p.anchor_length) + " " + hF(p.anchor_length_max) + " " + hI(p.multiline); }

static void run_asp()
{
    for (int i = 0; i < 240; ++ i) {
        coord_t distance = irand(20000, 120000);
        coord_t width    = i % 6 == 0 ? irand(0, 200000) : irand(0, 30000000);
        if (i % 12 == 1) width = distance * irand(1, 40);         // a whole number of intervals
        if (i % 12 == 2) width = distance * irand(1, 40) + irand(-2, 2);
        if (i % 12 == 3) width = distance + distance / irand(3, 9); // the 20 % limit
        out("in asp " + hI(width) + " " + hI(distance));
        out("asp " + hI(Fill::_adjust_solid_spacing(width, distance)));
    }
}

static void run_idir()
{
    static const float  angles[]  = { FLT_MAX, 0.f, 0.7853982f, 1.5707964f, 2.3f, -0.4f };
    static const double bridges[] = { -1., 0., 0.3, 1.5707963267948966, 2.9, -0.5 };
    for (int i = 0; i < 96; ++ i) {
        std::unique_ptr<Fill> f(Fill::new_from_type(ipRectilinear));
        f->angle    = angles[i % 6];
        f->layer_id = i % 5 == 0 ? size_t(-1) : size_t(irand(0, 40));
        const bool has_bbox = i % 4 != 0;
        if (has_bbox) {
            const coord_t x = irand(-3000000, 3000000), y = irand(-3000000, 3000000);
            f->set_bounding_box(BoundingBox(Point(x, y), Point(x + irand(1, 5000001), y + irand(1, 5000001))));
        }
        Surface surface(stInternal, ExPolygon(rect(irand(-900000, 0), irand(-900000, 0), irand(1, 900001), irand(1, 900001))));
        surface.bridge_angle     = bridges[(i / 6) % 6];
        surface.thickness_layers = (unsigned short) irand(1, 3);
        out("in idir " + hF(f->angle) + " " + hI(f->layer_id == size_t(-1) ? -1 : (long long) f->layer_id) + " " + hB(has_bbox) + " " + PT(f->bounding_box.min) + " " + PT(f->bounding_box.max) + " " +
            hD(surface.bridge_angle) + " " + hI(surface.thickness_layers) + " " + EXPOLY(surface.expolygon));
        const std::pair<float, Point> d = f->_infill_direction(&surface);
        const BoundingBox             e = f->extended_object_bounding_box();
        out("idir " + hF(d.first) + " " + PT(d.second) + " " + hF(f->_layer_angle(f->layer_id == size_t(-1) ? 0 : f->layer_id)) + " " + hB(f->use_bridge_flow()) + " " + hB(f->no_sort()) + " " +
            PT(e.min) + " " + PT(e.max));
    }
}

static void run_ml()
{
    for (int i = 0; i < 48; ++ i) {
        Polylines pls;
        const int n = irand(0, 5);
        for (int k = 0; k < n; ++ k) {
            Polyline  pl;
            const int np = k == 0 && i % 6 == 0 ? 1 : irand(2, 9);
            Point     p(irand(-3000000, 3000000), irand(-3000000, 3000000));
            for (int j = 0; j < np; ++ j) {
                pl.points.emplace_back(p);
                // now and then the same point again: a tangent without length
                if (irand(0, 9) != 0) p += Point(irand(-400000, 400000), irand(-400000, 400000));
            }
            pls.emplace_back(pl);
        }
        FillParams params;
        params.multiline    = irand(1, 5);
        const float spacing = float(irand(30, 90)) / 100.f;
        out("in ml " + hI(params.multiline) + " " + hF(spacing) + " " + PLS(pls));
        multiline_fill(pls, params, spacing);
        out("ml " + PLS(pls));
    }
}

static FillParams rnd_params(int i)
{
    static const float anchors[][2] = { { 1000.f, 1000.f }, { 2.5f, 20.f }, { 1.f, 5.f }, { 0.3f, 0.6f }, { 0.f, 20.f }, { 5.f, 5.f }, { 0.f, 0.05f }, { 2.f, 50.f } };
    FillParams params;
    params.density           = float(irand(8, 60)) / 100.f;
    params.anchor_length     = anchors[i % 8][0];
    params.anchor_length_max = anchors[i % 8][1];
    return params;
}

static void run_ci()
{
    for (int i = 0; i < 160; ++ i) {
        const ExPolygon  boundary = rnd_boundary();
        const FillParams params   = rnd_params(i);
        const double     spacing  = double(irand(30, 80)) / 100.;
        const coord_t    pitch    = coord_t(scale_(spacing / params.density));
        Polylines        lines    = rnd_lines(boundary, pitch, i % 4);
        if (lines.empty()) lines.emplace_back(Polyline(boundary.contour.points[0], boundary.contour.points[1]));
        // Now and then the lines in reverse order. (An empty polyline among the lines is not a valid input: the
        // pin reads its end points.)
        if (i % 16 == 9) std::reverse(lines.begin(), lines.end());
        out("in ci " + hD(spacing) + " " + PARAMS(params) + " " + EXPOLY(boundary) + " " + PLS(lines));
        Polylines result;
        result.emplace_back(Polyline(Point(7, 8), Point(9, 10))); // the output is appended to
        Fill::connect_infill(std::move(lines), boundary, result, spacing, params);
        out("ci " + PLS(result));
    }
}

static void run_cip()
{
    for (int i = 0; i < 48; ++ i) {
        const ExPolygon  boundary = rnd_boundary();
        const FillParams params   = rnd_params(i + 3);
        const double     spacing  = double(irand(30, 80)) / 100.;
        const coord_t    pitch    = coord_t(scale_(spacing / params.density));
        Polylines        lines    = rnd_lines(boundary, pitch, (i + 1) % 4);
        if (lines.empty()) lines.emplace_back(Polyline(boundary.contour.points[0], boundary.contour.points[1]));
        const Polygons polygons = to_polygons(boundary);
        // The bounding box the caller hands in: the one of the contour, or a wider one.
        BoundingBox bbox = get_extents(boundary.contour);
        if (i % 3 == 1) bbox.offset(irand(1, 30) * (MM / 10));
        out("in cip " + hD(spacing) + " " + PARAMS(params) + " " + PT(bbox.min) + " " + PT(bbox.max) + " " + POLYS(polygons) + " " + PLS(lines));
        Polylines result;
        Fill::connect_infill(std::move(lines), polygons, bbox, result, spacing, params);
        out("cip " + PLS(result));
    }
}

static void run_cbs()
{
    for (int i = 0; i < 96; ++ i) {
        const ExPolygon boundary = rnd_boundary();
        FillParams      params   = rnd_params(i);
        if (i % 5 == 0) params.density = float(irand(60, 100)) / 100.f;
        const double  spacing = double(irand(30, 80)) / 100.;
        const coord_t pitch   = coord_t(scale_(spacing / params.density));
        // The lines of the base of the supports are vertical.
        Polylines lines = rnd_lines(boundary, pitch, 0);
        if (lines.empty()) lines.emplace_back(Polyline(boundary.contour.points[0], boundary.contour.points[1]));
        const Polygons polygons = to_polygons(boundary);
        BoundingBox    bbox     = get_extents(boundary.contour);
        if (i % 3 == 1) bbox.offset(irand(1, 30) * (MM / 10));
        out("in cbs " + hD(spacing) + " " + PARAMS(params) + " " + PT(bbox.min) + " " + PT(bbox.max) + " " + POLYS(polygons) + " " + PLS(lines));
        Polylines result;
        Fill::connect_base_support(std::move(lines), polygons, bbox, result, spacing, params);
        out("cbs " + PLS(result));
    }
}

int main(int argc, char **argv)
{
    g_out = std::fopen(argc > 1 ? argv[1] : "vectors.txt", "w");
    // The groups to write: all of them, or those named.
    auto want = [argc, argv](const char *g) {
        if (argc <= 2) return true;
        for (int i = 2; i < argc; ++ i)
            if (std::strcmp(argv[i], g) == 0) return true;
        return false;
    };
    tbb::global_control gc(tbb::global_control::max_allowed_parallelism, 1);
    if (want("asp")) run_asp();
    if (want("idir")) run_idir();
    if (want("ml")) run_ml();
    if (want("ci")) run_ci();
    if (want("cip")) run_cip();
    if (want("cbs")) run_cbs();
    std::fclose(g_out);
    return 0;
}
