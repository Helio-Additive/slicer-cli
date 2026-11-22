// Generator of the vectors of tests/fill_rectilinear_native_parity.rs: libslic3r's Fill/FillRectilinear.cpp
// at the pin (references/BambuStudio/src/libslic3r; the override layer has no copy of it).
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
// FillRectilinear.cpp is compiled into this file (most of its functions are static), with the protected
// members of the classes made public.
//
// Groups:
//   "fs"   Fill::fill_surface of FillRectilinear, FillMonotonic, FillMonotonicLine, FillAlignedRectilinear,
//          FillGrid, FillTriangles, FillStars, FillCubic, FillSupportBase, FillZigZag and FillCrossZag on
//          surfaces made from a fixed linear congruential generator (rectangles, L shapes, slanted
//          quadrangles, blobs, a shape with a thin neck, each with up to two holes and turned by a rational
//          rotation), densities 0.1 to 1, angles, dont_adjust, anchors, link_max_length, multiline: the
//          polylines in order and the spacing the filler is left with.
//   "sl"   slice_region_by_vertical_lines + connect_segment_intersections_by_contours of the same kind of
//          input: every intersection with its links.
//   "mfl"  make_fill_lines.
//   "sgp"  sample_grid_pattern.
//   "wg"   FillMonotonicLineWGapFill::fill_surface_by_lines and the same of FillGlobalMonotonicLine.
//   "sia"  the link accessors of SegmentIntersection over every combination of intersection type, link
//          types and link qualities; "dos" distance_of_segmens.
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

#include <boost/container/small_vector.hpp>
#include <boost/container/static_vector.hpp>
#include <boost/log/trivial.hpp>
#include <random>
#include <sstream>

#define private public
#define protected public
#include "libslic3r/Fill/FillRectilinear.cpp"
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

// Turns a point by a rational rotation (integer arithmetic only): k = 0 none, else a Pythagorean triple.
static Point turn(const Point &p, int k)
{
    static const int64_t t[5][3] = { { 1, 0, 1 }, { 4, 3, 5 }, { 12, 5, 13 }, { 15, 8, 17 }, { 7, 24, 25 } };
    const int64_t c = t[k][0], s = t[k][1], d = t[k][2];
    return Point((c * p.x() - s * p.y()) / d, (s * p.x() + c * p.y()) / d);
}

// A surface: a contour of 3 to 40 mm with up to two holes inside, near the origin.
static ExPolygon rnd_surface()
{
    const coord_t w = coord_t(irand(30, 400)) * (MM / 10) + irand(0, 999), h = coord_t(irand(30, 400)) * (MM / 10) + irand(0, 999);
    const coord_t ox = -w / 2 + coord_t(irand(-80, 80)) * (MM / 10) + irand(-5000, 5000), oy = -h / 2 + coord_t(irand(-80, 80)) * (MM / 10) + irand(-5000, 5000);
    ExPolygon     e;
    const int     shape = irand(0, 4);
    switch (shape) {
    case 0: e.contour = rect(ox, oy, ox + w, oy + h); break;
    case 1: e.contour = blob(ox + w / 2, oy + h / 2, w, h); break;
    case 2:
        // an L
        e.contour.points = { Point(ox, oy), Point(ox + w, oy), Point(ox + w, oy + h / 3), Point(ox + w / 3, oy + h / 3), Point(ox + w / 3, oy + h), Point(ox, oy + h) };
        break;
    case 3:
        // a slanted quadrangle
        e.contour.points = { Point(ox, oy), Point(ox + w, oy + h / 5), Point(ox + w - w / 6, oy + h), Point(ox + w / 8, oy + h - h / 7) };
        break;
    default: {
        // two blocks joined by a thin neck (0.2 to 1.5 mm wide)
        const coord_t n = coord_t(irand(2, 15)) * (MM / 10);
        const coord_t yc = oy + h / 2;
        e.contour.points = { Point(ox, oy), Point(ox + w / 3, oy), Point(ox + w / 3, yc - n / 2), Point(ox + 2 * w / 3, yc - n / 2), Point(ox + 2 * w / 3, oy), Point(ox + w, oy),
                             Point(ox + w, oy + h), Point(ox + 2 * w / 3, oy + h), Point(ox + 2 * w / 3, yc + n / 2), Point(ox + w / 3, yc + n / 2), Point(ox + w / 3, oy + h), Point(ox, oy + h) };
        break;
    }
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
    // Turned, now and then.
    const int k = irand(0, 7);
    if (k >= 1 && k <= 4) {
        for (Point &p : e.contour.points) p = turn(p, k);
        for (Polygon &hole : e.holes)
            for (Point &p : hole.points) p = turn(p, k);
    }
    return e;
}

enum { T_RECT = 0, T_MONO, T_MONOLINE, T_ALIGNED, T_GRID, T_TRIANGLES, T_STARS, T_CUBIC, T_SUPPORT_BASE, T_ZIGZAG, T_CROSSZAG, T_COUNT };

static Fill *make_filler(int type)
{
    switch (type) {
    case T_RECT: return new FillRectilinear();
    case T_MONO: return new FillMonotonic();
    case T_MONOLINE: return new FillMonotonicLine();
    case T_ALIGNED: return new FillAlignedRectilinear();
    case T_GRID: return new FillGrid();
    case T_TRIANGLES: return new FillTriangles();
    case T_STARS: return new FillStars();
    case T_CUBIC: return new FillCubic();
    case T_SUPPORT_BASE: return new FillSupportBase();
    case T_ZIGZAG: return new FillZigZag();
    default: return new FillCrossZag();
    }
}
static InfillPattern pattern_of(int type)
{
    switch (type) {
    case T_RECT: return ipRectilinear;
    case T_MONO: return ipMonotonic;
    case T_MONOLINE: return ipMonotonicLine;
    case T_ALIGNED: return ipAlignedRectilinear;
    case T_GRID: return ipGrid;
    case T_TRIANGLES: return ipTriangles;
    case T_STARS: return ipStars;
    case T_CUBIC: return ipCubic;
    case T_SUPPORT_BASE: return ipSupportBase;
    case T_ZIGZAG: return ipZigZag;
    default: return ipCrossZag;
    }
}

struct Case
{
    int        type;
    size_t     layer_id;
    double     z, spacing, overlap, angle_deg;
    coord_t    link_max_length;
    BoundingBox bbox;
    double     bridge_angle;
    int        thickness_layers;
    FillParams params;
    ExPolygon  expolygon;
};

static Case rnd_case(int i, int type)
{
    static const double angles[]     = { 45., 0., 90., 135., 30., -15., 60., 22.5 };
    static const float  anchors[][2] = { { 1000.f, 1000.f }, { 2.5f, 20.f }, { 1.f, 5.f }, { 0.3f, 0.6f }, { 0.f, 20.f }, { 5.f, 5.f }, { 0.f, 0.03f }, { 2.f, 50.f } };
    Case c;
    c.type      = type;
    c.layer_id  = size_t(irand(0, 40));
    c.z         = 0.2 * double(c.layer_id + 1);
    c.spacing   = double(irand(380, 620)) / 1000.;
    c.overlap   = i % 5 == 0 ? 0. : double(irand(0, 150)) / 1000.;
    c.angle_deg = i % 9 == 8 ? double(irand(-1800, 1800)) / 10. : angles[i % 8];
    const bool multi = type == T_GRID || type == T_TRIANGLES || type == T_STARS || type == T_CUBIC || type == T_SUPPORT_BASE;
    // Dense in half of the cases of the fillers that take it.
    const bool full = ! multi && i % 2 == 0;
    c.params.density           = full ? 1.f : float(irand(10, 95)) / 100.f;
    c.params.anchor_length     = anchors[(i / 2) % 8][0];
    c.params.anchor_length_max = anchors[(i / 2) % 8][1];
    c.params.dont_adjust       = i % 4 >= 2;
    c.params.multiline         = (! full && i % 7 == 3) ? irand(2, 3) : 1;
    c.params.pattern           = pattern_of(type);
    // Fill.cpp:691-700: 3 spacings above 80 % density, else no limit.
    c.link_max_length = c.params.density > 0.8f ? coord_t(scale_(3. * c.spacing)) : 0;
    if (i % 11 == 10) c.link_max_length = coord_t(irand(0, 500000));
    // The bounding box of the object: centred at the origin (PrintObject::bounding_box), now and then
    // elsewhere or undefined.
    const coord_t sx = coord_t(irand(100, 600)) * MM + irand(0, 9), sy = coord_t(irand(100, 600)) * MM + irand(0, 9);
    c.bbox = BoundingBox(Point(-sx / 2, -sy / 2), Point(sx / 2, sy / 2));
    if (i % 13 == 12) c.bbox.translate(coord_t(irand(-20, 20)) * MM + irand(0, 999), coord_t(irand(-20, 20)) * MM + irand(0, 999));
    if (i % 29 == 28) c.bbox = BoundingBox(Point(0, 0), Point(-1, -1));
    c.bridge_angle     = i % 17 == 16 ? double(irand(0, 3000)) / 1000. : -1.;
    c.thickness_layers = i % 19 == 18 ? 2 : 1;
    c.expolygon        = rnd_surface();
    return c;
}

static std::string CASE(const Case &c)
{
    return hI(c.type) + " " + hI((long long) c.layer_id) + " " + hD(c.z) + " " + hD(c.spacing) + " " + hD(c.overlap) + " " + hD(c.angle_deg) + " " + hI(c.link_max_length) + " " + PT(c.bbox.min) + " " +
           PT(c.bbox.max) + " " + hD(c.bridge_angle) + " " + hI(c.thickness_layers) + " " + hF(c.params.density) + " " + hF(c.params.anchor_length) + " " + hF(c.params.anchor_length_max) + " " +
           hI(c.params.multiline) + " " + hB(c.params.dont_adjust) + " " + EXPOLY(c.expolygon);
}

static void run_fs(int n_per_type)
{
    for (int type = 0; type < T_COUNT; ++ type)
        for (int i = 0; i < n_per_type; ++ i) {
            const Case            c = rnd_case(i, type);
            std::unique_ptr<Fill> f(make_filler(type));
            f->layer_id        = c.layer_id;
            f->z               = c.z;
            f->spacing         = c.spacing;
            f->overlap         = c.overlap;
            f->angle           = float(Geometry::deg2rad(c.angle_deg));
            f->link_max_length = c.link_max_length;
            f->set_bounding_box(c.bbox);
            Surface surface(stInternal, c.expolygon);
            surface.bridge_angle     = c.bridge_angle;
            surface.thickness_layers = (unsigned short) c.thickness_layers;
            out("in fs " + CASE(c));
            const Polylines result = f->fill_surface(&surface, c.params);
            out("fs " + hD(f->spacing) + " " + PLS(result));
        }
}

static std::string SEGS(const std::vector<SegmentedIntersectionLine> &segs)
{
    std::string o = hI((long long) segs.size());
    for (const SegmentedIntersectionLine &il : segs) {
        o += " " + hI((long long) il.idx) + " " + hI(il.pos) + " " + hI((long long) il.intersections.size());
        for (const SegmentIntersection &it : il.intersections)
            o += " " + hI((long long) it.iContour) + " " + hI((long long) it.iSegment) + " " + hI(it.pos_p) + " " + hI(it.pos_q) + " " + hI(int(it.type)) + " " + hI(it.prev_on_contour) + " " +
                 hI(int(it.prev_on_contour_type)) + " " + hI(int(it.prev_on_contour_quality)) + " " + hI(it.next_on_contour) + " " + hI(int(it.next_on_contour_type)) + " " +
                 hI(int(it.next_on_contour_quality)) + " " + hI((long long) it.prev_idx);
    }
    return o;
}

// The stages of fill_surface_by_lines, one at a time, on a polygon turned by `angle` and offset twice.
static void run_sl(int n)
{
    static const float angles[] = { 0.f, 0.7853982f, 1.5707964f, 2.3561945f, 0.5f, -0.3f };
    for (int i = 0; i < n; ++ i) {
        const ExPolygon expolygon    = rnd_surface();
        const float     angle        = angles[i % 6];
        const coord_t   aoffset1     = coord_t(irand(-30000, 5000));
        const coord_t   aoffset2     = i % 5 == 0 ? 0 : aoffset1 - coord_t(irand(1000, 30000));
        const coord_t   line_spacing = coord_t(irand(30000, 400000));
        const bool      full         = i % 3 == 0;
        FillParams      params;
        params.density           = full ? 1.f : 0.3f;
        params.anchor_length     = 1000.f;
        params.anchor_length_max = i % 7 == 6 ? 0.f : 1000.f;
        const coord_t link_max_length = i % 4 == 0 ? 0 : coord_t(irand(50000, 600000));
        out("in sl " + hF(angle) + " " + hI(aoffset1) + " " + hI(aoffset2) + " " + hI(line_spacing) + " " + hB(full) + " " + hF(params.anchor_length_max) + " " + hI(link_max_length) + " " +
            EXPOLY(expolygon));
        ExPolygonWithOffset poly_with_offset(expolygon, -angle, aoffset1, aoffset2);
        std::string         o = "pwo " + hI((long long) poly_with_offset.n_contours_outer) + " " + hI((long long) poly_with_offset.n_contours_inner) + " " + hI((long long) poly_with_offset.n_contours);
        for (const Polygon &p : poly_with_offset.polygons_outer) o += " " + POLY(p);
        for (const Polygon &p : poly_with_offset.polygons_inner) o += " " + POLY(p);
        for (size_t k = 0; k < poly_with_offset.polygons_ccw.size(); ++ k) o += " " + hB(poly_with_offset.polygons_ccw[k]);
        out(o);
        if (poly_with_offset.n_contours == 0) continue;
        const BoundingBox bounding_box = poly_with_offset.bounding_box_src();
        out("bbs " + PT(bounding_box.min) + " " + PT(bounding_box.max));
        const size_t n_vlines = (bounding_box.max(0) - bounding_box.min(0) + line_spacing - 1) / line_spacing;
        coord_t      x0       = bounding_box.min(0);
        if (full) x0 += (line_spacing + coord_t(SCALED_EPSILON)) / 2;
        std::vector<SegmentedIntersectionLine> segs = slice_region_by_vertical_lines(poly_with_offset, n_vlines, x0, line_spacing);
        out("sl " + SEGS(segs));
        if (aoffset2 == 0) continue; // no inner contour: only the outer intersections
        connect_segment_intersections_by_contours(poly_with_offset, segs, params, link_max_length);
        out("csi " + SEGS(segs));
        {
            std::vector<SegmentedIntersectionLine> segs2 = segs;
            Polylines                              pls;
            traverse_graph_generate_polylines(poly_with_offset, params, segs2, i % 2 == 1, pls);
            out("tg " + hB(i % 2 == 1) + " " + PLS(pls));
        }
        {
            std::vector<SegmentedIntersectionLine> segs2 = segs;
            pinch_contours_insert_phony_outer_intersections(segs2);
            out("pinch " + SEGS(segs2));
            std::vector<MonotonicRegion> regions = generate_montonous_regions(segs2);
            connect_monotonic_regions(regions, poly_with_offset, segs2);
            std::string r = "mr " + hI((long long) regions.size());
            for (const MonotonicRegion &reg : regions) {
                r += " " + hI(reg.left.vline) + " " + hI(reg.left.low) + " " + hI(reg.left.high) + " " + hI(reg.right.vline) + " " + hI(reg.right.low) + " " + hI(reg.right.high) + " " +
                     hF(reg.len1) + " " + hF(reg.len2) + " " + hB(reg.flips) + " " + hI((long long) reg.left_neighbors.size());
                for (const MonotonicRegion *nb : reg.left_neighbors) r += " " + hI((long long) (nb - regions.data()));
                r += " " + hI((long long) reg.right_neighbors.size());
                for (const MonotonicRegion *nb : reg.right_neighbors) r += " " + hI((long long) (nb - regions.data()));
            }
            out(r);
            if (! regions.empty()) {
                std::mt19937_64                  rng;
                std::vector<MonotonicRegionLink> path = chain_monotonic_regions(regions, poly_with_offset, segs2, rng);
                std::string                      p    = "chain " + hI((long long) path.size());
                for (const MonotonicRegionLink &l : path) p += " " + hI((long long) (l.region - regions.data())) + " " + hB(l.flipped);
                out(p);
                Polylines pls;
                polylines_from_paths(path, poly_with_offset, segs2, pls);
                out("pfp " + PLS(pls));
            }
        }
    }
}

static void run_mfl(int n)
{
    static const double angles[] = { 0., 0.7853981633974483, 1.5707963267948966, 1.0471975511965976, 2.0943951023931953, -0.4 };
    for (int i = 0; i < n; ++ i) {
        const ExPolygon expolygon     = rnd_surface();
        const double    angle         = angles[i % 6];
        const coord_t   aoffset1      = coord_t(irand(-30000, 5000));
        const coord_t   line_spacing  = coord_t(irand(30000, 400000));
        const coord_t   x_margin      = i % 3 == 0 ? 0 : coord_t(irand(0, 60000));
        const coord_t   pattern_shift = i % 4 == 0 ? 0 : coord_t(irand(-900000, 900000));
        const Point     refpt(irand(-500000, 500000), irand(-500000, 500000));
        out("in mfl " + hD(angle) + " " + hI(aoffset1) + " " + hI(line_spacing) + " " + hI(x_margin) + " " + hI(pattern_shift) + " " + PT(refpt) + " " + EXPOLY(expolygon));
        ExPolygonWithOffset base(expolygon, 0, aoffset1);
        Polylines           fill_lines;
        if (base.n_contours > 0) make_fill_lines(ExPolygonWithOffset(base, float(-angle)), refpt, angle, x_margin, line_spacing, pattern_shift, fill_lines);
        out("mfl " + PLS(fill_lines));
    }
}

// FillMonotonicLineWGapFill::fill_surface_by_lines (the lines of a top surface) and the same of
// FillGlobalMonotonicLine, which sweeps the columns from left to right.
static void run_wg(int n)
{
    static const double angles[] = { 45., 0., 90., 135., 30., -15., 60., 22.5 };
    for (int i = 0; i < n; ++ i) {
        const bool                                 global = i % 3 == 2;
        std::unique_ptr<FillMonotonicLineWGapFill> f(global ? new FillGlobalMonotonicLine() : new FillMonotonicLineWGapFill());
        f->layer_id        = size_t(irand(0, 40));
        f->spacing         = double(irand(380, 620)) / 1000.;
        f->overlap         = 0.;
        const double angle_deg = i % 9 == 8 ? double(irand(-1800, 1800)) / 10. : angles[i % 8];
        f->angle           = float(Geometry::deg2rad(angle_deg));
        f->link_max_length = 0;
        const coord_t sx = coord_t(irand(100, 600)) * MM + irand(0, 9), sy = coord_t(irand(100, 600)) * MM + irand(0, 9);
        BoundingBox   bbox(Point(-sx / 2, -sy / 2), Point(sx / 2, sy / 2));
        if (i % 13 == 12) bbox.translate(coord_t(irand(-20, 20)) * MM + irand(0, 999), coord_t(irand(-20, 20)) * MM + irand(0, 999));
        f->set_bounding_box(bbox);
        FillParams params;
        params.density           = 1.f;
        params.monotonic         = true;
        params.anchor_length_max = 0.f;
        // The caller always sets it (FillRectilinear.cpp:3652); the other branch now and then.
        params.dont_adjust = i % 5 != 4;
        params.flow        = Flow(float(irand(380, 620)) / 1000.f, 0.2f, 0.4f);
        Surface surface(stTop, rnd_surface());
        surface.bridge_angle = i % 17 == 16 ? double(irand(0, 3000)) / 1000. : -1.;
        out("in wg " + hB(global) + " " + hI((long long) f->layer_id) + " " + hD(f->spacing) + " " + hD(angle_deg) + " " + PT(bbox.min) + " " + PT(bbox.max) + " " + hD(surface.bridge_angle) + " " +
            hB(params.dont_adjust) + " " + hF(params.flow.spacing()) + " " + EXPOLY(surface.expolygon));
        Polylines result;
        result.emplace_back(Polyline(Point(7, 8), Point(9, 10))); // the output is appended to
        f->fill_surface_by_lines(&surface, params, result);
        out("wg " + hD(f->spacing) + " " + PLS(result));
    }
}

static void run_sgp(int n)
{
    for (int i = 0; i < n; ++ i) {
        const ExPolygon   expolygon = rnd_surface();
        const coord_t     spacing   = coord_t(irand(50000, 600000));
        BoundingBox       bbox      = get_extents(expolygon.contour);
        bbox.offset(coord_t(irand(0, 30)) * (MM / 10));
        out("in sgp " + hI(spacing) + " " + PT(bbox.min) + " " + PT(bbox.max) + " " + EXPOLY(expolygon));
        out("sgp " + PTS(sample_grid_pattern(expolygon, spacing, bbox)));
    }
}

// The link accessors of SegmentIntersection (FillRectilinear.cpp:211-271) over every combination of the
// intersection type, the two link types and the two link qualities, and distance_of_segmens.
static void run_sia()
{
    typedef SegmentIntersection SI;
    int n = 0;
    for (int ty = 0; ty < 5; ++ ty)
        for (int pt = 0; pt < 4; ++ pt)
            for (int nt = 0; nt < 4; ++ nt)
                for (int pq = 0; pq < 3; ++ pq)
                    for (int nq = 0; nq < 3; ++ nq, ++ n) {
                        SI s;
                        s.type                    = SI::SegmentIntersectionType(ty);
                        s.prev_on_contour_type    = SI::LinkType(pt);
                        s.next_on_contour_type    = SI::LinkType(nt);
                        s.prev_on_contour_quality = SI::LinkQuality(pq);
                        s.next_on_contour_quality = SI::LinkQuality(nq);
                        s.prev_on_contour         = 3 + n % 7;
                        s.next_on_contour         = 11 + n % 5;
                        out("in sia " + hI(ty) + " " + hI(pt) + " " + hI(nt) + " " + hI(pq) + " " + hI(nq) + " " + hI(s.prev_on_contour) + " " +
                            hI(s.next_on_contour));
                        std::string o = "sia";
                        auto        b = [&o](bool v) { o += v ? " 1" : " 0"; };
                        auto        i = [&o](int v) { o += " " + std::to_string(v); };
                        b(s.has_left_vertical(SI::Direction::Up));
                        b(s.has_left_vertical(SI::Direction::Down));
                        b(s.has_left_vertical());
                        b(s.has_left_vertical_outside());
                        b(s.has_right_vertical(SI::Direction::Up));
                        b(s.has_right_vertical(SI::Direction::Down));
                        b(s.has_right_vertical());
                        b(s.has_right_vertical_outside());
                        b(s.has_vertical());
                        b(s.has_vertical(SI::Side::Left));
                        b(s.has_vertical(SI::Side::Right));
                        b(s.has_vertical_up());
                        b(s.has_vertical_down());
                        b(s.has_vertical(SI::Direction::Up));
                        b(s.has_vertical(SI::Direction::Down));
                        // horizontal_quality asserts has_horizontal(side).
                        i(s.has_horizontal(SI::Side::Left) ? int(s.horizontal_quality(SI::Side::Left)) : -1);
                        i(s.has_horizontal(SI::Side::Right) ? int(s.horizontal_quality(SI::Side::Right)) : -1);
                        i(s.left_vertical(SI::Direction::Up));
                        i(s.left_vertical(SI::Direction::Down));
                        i(s.left_vertical());
                        i(s.left_vertical_outside());
                        i(s.right_vertical(SI::Direction::Up));
                        i(s.right_vertical(SI::Direction::Down));
                        i(s.right_vertical());
                        i(s.right_vertical_outside());
                        i(s.vertical_up(SI::Side::Left));
                        i(s.vertical_up(SI::Side::Right));
                        i(s.vertical_down(SI::Side::Left));
                        i(s.vertical_down(SI::Side::Right));
                        i(s.vertical_outside(SI::Side::Left));
                        i(s.vertical_outside(SI::Side::Right));
                        i(s.vertical_up());
                        i(s.vertical_down());
                        i(s.vertical_outside());
                        out(o);
                    }
    for (int k = 0; k < 64; ++ k) {
        const int np   = 3 + k % 9;
        const int seg1 = (k * 5) % np;
        const int seg2 = (k * 7 + 2) % np;
        const bool forward = (k / 3) % 2 == 0;
        Polygon poly;
        for (int j = 0; j < np; ++ j)
            poly.points.emplace_back(Point(coord_t(j) * 1000, coord_t(j % 3) * 1000));
        out("in dos " + hI(np) + " " + hI(seg1) + " " + hI(seg2) + " " + hB(forward));
        out("dos " + hI(distance_of_segmens(poly, size_t(seg1), size_t(seg2), forward)));
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
    if (want("fs")) run_fs(40);
    if (want("sl")) run_sl(96);
    if (want("mfl")) run_mfl(48);
    if (want("sgp")) run_sgp(24);
    if (want("wg")) run_wg(96);
    if (want("sia")) run_sia();
    std::fclose(g_out);
    return 0;
}
