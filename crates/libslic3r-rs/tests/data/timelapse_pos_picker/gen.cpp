// Generator of the vectors of tests/timelapse_pos_picker_native_parity.rs: `pick_pos_internal` of
// GCode/TimelapsePosPicker.cpp at the pin (references/BambuStudio/src/libslic3r), on generated
// safe areas, path collision areas, positions and farthest points.
//
// Build as tests/data/config/gen.cpp says (the compile line of a libslic3r source of a built tree
// of the Bambu Studio engine, linked with its libslic3r_core and the libraries that links), then
//   ./gen > vectors.txt && gzip -9 -n vectors.txt
//
// `pick_pos_internal` is a function of namespace Slic3r without a declaration in the header; it is
// declared here. The vectors were made on macOS arm64 with Apple clang and libc++: which of two
// candidates of equal penalty leaves the `std::priority_queue` first is that library's heap.
//
// One line per case, integers separated by one space:
//   CASE <curr x> <curr y> <detect> <has farthest> <fx> <fy> <safe areas> <collision areas> -> <x> <y>
// An area list is its count, then per expolygon the contour and the hole count and holes; a ring is
// its point count and the points.

#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/libslic3r.h"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <optional>
#include <vector>

namespace Slic3r {
Point pick_pos_internal(const Point &curr_pos, const ExPolygons &safe_areas, const ExPolygons &path_collision_area, bool detect_path_collision,
                        const std::optional<Point> &farthest_point);
}

using namespace Slic3r;

static uint64_t g_state = 0x9e3779b97f4a7c15ull;
static uint64_t rnd()
{
    g_state ^= g_state << 13;
    g_state ^= g_state >> 7;
    g_state ^= g_state << 17;
    return g_state;
}
static int64_t rnd_range(int64_t lo, int64_t hi) { return lo + int64_t(rnd() % uint64_t(hi - lo + 1)); }

// A coordinate in [0, 256] mm, on a grid of `grid` mm so that equal penalties happen.
static coord_t rnd_coord(int grid)
{
    if (grid == 0)
        return coord_t(rnd_range(0, 25600000));
    return coord_t(rnd_range(0, 256 / grid)) * grid * 100000;
}

static Polygon rnd_rect(int grid)
{
    coord_t x0 = rnd_coord(grid), x1 = rnd_coord(grid), y0 = rnd_coord(grid), y1 = rnd_coord(grid);
    if (x0 > x1) std::swap(x0, x1);
    if (y0 > y1) std::swap(y0, y1);
    if (x0 == x1) x1 += 100000 * (grid ? grid : 1);
    if (y0 == y1) y1 += 100000 * (grid ? grid : 1);
    return Polygon{Point(x0, y0), Point(x1, y0), Point(x1, y1), Point(x0, y1)};
}

static Polygon rnd_blob()
{
    const coord_t cx = rnd_coord(0), cy = rnd_coord(0);
    const double  rx = double(rnd_range(200000, 6000000)), ry = double(rnd_range(200000, 6000000));
    const int     n  = int(rnd_range(3, 24));
    Polygon       p;
    for (int i = 0; i < n; ++i) {
        const double a = 2. * M_PI * (double(i) + double(rnd_range(0, 800)) / 1000.) / double(n);
        p.points.emplace_back(coord_t(cx + coord_t(rx * std::cos(a))), coord_t(cy + coord_t(ry * std::sin(a))));
    }
    return p;
}

static ExPolygons rnd_areas(int max_count, int grid)
{
    ExPolygons out;
    const int  n = int(rnd_range(0, max_count));
    for (int i = 0; i < n; ++i) {
        ExPolygon e;
        if (rnd() % 3 == 0) {
            e.contour = rnd_blob();
        } else {
            e.contour = rnd_rect(grid);
            if (rnd() % 4 == 0) {
                // a rectangular hole inside, clockwise
                const BoundingBox bb = e.contour.bounding_box();
                const coord_t     w = bb.max.x() - bb.min.x(), h = bb.max.y() - bb.min.y();
                if (w > 400000 && h > 400000) {
                    const coord_t x0 = bb.min.x() + w / 4, x1 = bb.max.x() - w / 4, y0 = bb.min.y() + h / 4, y1 = bb.max.y() - h / 4;
                    e.holes.push_back(Polygon{Point(x0, y0), Point(x0, y1), Point(x1, y1), Point(x1, y0)});
                }
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

static void print_areas(const ExPolygons &areas)
{
    std::printf(" %zu", areas.size());
    for (const ExPolygon &e : areas) {
        print_ring(e.contour);
        std::printf(" %zu", e.holes.size());
        for (const Polygon &h : e.holes)
            print_ring(h);
    }
}

int main()
{
    for (int i = 0; i < 2000; ++i) {
        const int        grid   = (i % 3 == 0) ? 0 : ((i % 3 == 1) ? 1 : 8);
        const ExPolygons safe   = rnd_areas(i % 50 == 0 ? 0 : 6, grid);
        const ExPolygons coll   = rnd_areas(4, grid);
        const Point      curr(rnd_coord(grid), rnd_coord(grid));
        const bool       detect = rnd() % 2 == 0;
        std::optional<Point> farthest;
        if (rnd() % 3 == 0)
            farthest = Point(rnd_coord(grid), rnd_coord(grid));
        const Point res = pick_pos_internal(curr, safe, coll, detect, farthest);
        std::printf("CASE %lld %lld %d %d %lld %lld", (long long) curr.x(), (long long) curr.y(), detect ? 1 : 0, farthest ? 1 : 0,
                    (long long) (farthest ? farthest->x() : 0), (long long) (farthest ? farthest->y() : 0));
        print_areas(safe);
        print_areas(coll);
        std::printf(" -> %lld %lld\n", (long long) res.x(), (long long) res.y());
    }
    return 0;
}
