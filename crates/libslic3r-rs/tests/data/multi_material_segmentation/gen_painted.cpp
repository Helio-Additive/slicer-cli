// Generator of the painted-line vectors of tests/multi_material_segmentation_native_parity.rs
// (test `painted_lines_native_parity`): post_process_painted_lines (with filter_painted_lines) and
// colorize_contours (with colorize_contour, colorize_line and filter_colorized_polygon) of
// libslic3r's MultiMaterialSegmentation.cpp at the pin, on synthetic contours.
//
// Build as gen.cpp of this directory (the engine's compile and link line; MultiMaterialSegmentation.cpp
// is compiled into this file because the functions are static), then `./gen_painted painted_vectors.txt`.
//
// Inputs are deterministic (a 64-bit LCG): a few closed contours (EdgeGrid::Contour over a vector of
// points), then painted lines on their segments. The projected lines start and end at parameters
// taken mostly from a coarse grid {0, 1/4, 1/2, 3/4, 1} of the segment, so several painted lines of
// one segment share their start point and their length: the comparator of post_process_painted_lines
// ties on them and the order std::sort (libc++ introsort, not stable) leaves them in decides which
// colour wins (MultiMaterialSegmentation.cpp:689-719).
//
// A line is "pp <contours> ; <painted lines> => <post-processed> | <colorized> # <segments>", integers only:
// contours: N, then per contour n x0 y0 ... ; painted lines: M, then per line ci li ax ay bx by color;
// post-processed: per contour k, then k times ci li ax ay bx by color; colorized: per contour k, then
// k times ax ay bx by color poly_idx local_line_idx; segments: "one" when has_layer_only_one_color, else
// the number of colour slots, then per slot the number of polygons and per polygon its number of holes
// (always 0), its number of points and the points (build_graph .. extract_colored_segments).
// The vectors were made on macOS arm64 with Apple clang, Eigen 5.0.1 and Boost 1.90.

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <map>
#include <string>
#include <thread>
#include <vector>

#include <tbb/blocked_range.h>
#include <tbb/concurrent_unordered_map.h>
#include <tbb/concurrent_unordered_set.h>
#include <tbb/concurrent_vector.h>
#include <tbb/global_control.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_for_each.h>
#include <tbb/spin_mutex.h>

#include "libslic3r/MultiMaterialSegmentation.cpp"

using namespace Slic3r;

static uint64_t g_seed = 0x9e3779b97f4a7c15ULL;
static uint32_t lcg()
{
    g_seed = g_seed * 6364136223846793005ULL + 1442695040888963407ULL;
    return uint32_t(g_seed >> 33);
}
static int rnd(int n) { return int(lcg() % uint32_t(n)); }

int main(int argc, char **argv)
{
    if (argc < 2) {
        std::fprintf(stderr, "usage: gen_painted <out.txt>\n");
        return 2;
    }
    tbb::global_control one_thread(tbb::global_control::max_allowed_parallelism, 1);
    FILE *out = std::fopen(argv[1], "w");
    const int num_cases = 2000;
    for (int c = 0; c < num_cases; ++c) {
        // Contours: regular polygons with jittered radii, CCW, of 3..10 points, apart from each other (a layer's
        // contours do not cross; crossing input makes the Voronoi builder fall back to a rotated input).
        const int                         nc = 1 + rnd(3);
        std::vector<std::vector<Point>>   pts(nc);
        for (int ci = 0; ci < nc; ++ci) {
            const int    n  = 3 + rnd(8);
            const double cx = double(ci) * 1.2e7 + double(rnd(2000000)) - 1000000., cy = double(rnd(2000000)) - 1000000.;
            const double r0 = 200000. + double(rnd(4000000));
            for (int i = 0; i < n; ++i) {
                double a = 2. * M_PI * double(i) / double(n);
                double r = r0 * (0.7 + 0.3 * double(rnd(1000)) / 1000.);
                pts[ci].emplace_back(coord_t(std::lrint(cx + r * std::cos(a))), coord_t(std::lrint(cy + r * std::sin(a))));
            }
        }
        std::vector<EdgeGrid::Contour> contours;
        for (int ci = 0; ci < nc; ++ci) contours.emplace_back(pts[ci], false);

        // Painted lines.
        static const double grid[5] = {0., 0.25, 0.5, 0.75, 1.};
        std::vector<PaintedLine> painted;
        // Some cases with more than 30 lines, where libc++ std::sort partitions instead of sorting by insertion.
        const int                kind = rnd(8);
        const int                m    = kind == 0 ? 0 : kind == 1 ? 31 + rnd(60) : 1 + rnd(24);
        for (int k = 0; k < m; ++k) {
            const size_t ci = size_t(rnd(nc));
            const size_t li = size_t(rnd(int(contours[ci].num_segments())));
            // Concentrate on few segments to make ties.
            const size_t li2  = rnd(2) == 0 ? li % 2 : li;
            const Line   seg  = contours[ci].get_segment(li2);
            double       t0, t1;
            if (rnd(4) != 0) {
                int i0 = rnd(4), i1 = i0 + 1 + rnd(4 - i0);
                t0 = grid[i0]; t1 = grid[i1];
            } else {
                t0 = double(rnd(900)) / 1000.;
                t1 = t0 + double(1 + rnd(int(1000 - t0 * 1000.))) / 1000.;
                if (t1 > 1.) t1 = 1.;
            }
            const Vec2d a = seg.a.cast<double>(), b = seg.b.cast<double>();
            const Vec2d pa = a + (b - a) * t0, pb = a + (b - a) * t1;
            Line proj(Point(coord_t(std::lrint(pa.x())), coord_t(std::lrint(pa.y()))), Point(coord_t(std::lrint(pb.x())), coord_t(std::lrint(pb.y()))));
            if (proj.a == proj.b) continue;
            painted.push_back({ci, li2, proj, rnd(4)});
        }

        std::fprintf(out, "pp %d", nc);
        for (int ci = 0; ci < nc; ++ci) {
            std::fprintf(out, " %zu", pts[ci].size());
            for (const Point &p : pts[ci]) std::fprintf(out, " %lld %lld", (long long) p.x(), (long long) p.y());
        }
        std::fprintf(out, " ; %zu", painted.size());
        for (const PaintedLine &pl : painted)
            std::fprintf(out, " %zu %zu %lld %lld %lld %lld %d", pl.contour_idx, pl.line_idx, (long long) pl.projected_line.a.x(), (long long) pl.projected_line.a.y(),
                         (long long) pl.projected_line.b.x(), (long long) pl.projected_line.b.y(), pl.color);

        std::vector<std::vector<PaintedLine>> post = post_process_painted_lines(contours, std::vector<PaintedLine>(painted));
        std::fprintf(out, " =>");
        if (post.empty()) post.assign(contours.size(), {}); // as the caller: no painted line, no colorization input
        for (const std::vector<PaintedLine> &v : post) {
            std::fprintf(out, " %zu", v.size());
            for (const PaintedLine &pl : v)
                std::fprintf(out, " %zu %zu %lld %lld %lld %lld %d", pl.contour_idx, pl.line_idx, (long long) pl.projected_line.a.x(), (long long) pl.projected_line.a.y(),
                             (long long) pl.projected_line.b.x(), (long long) pl.projected_line.b.y(), pl.color);
        }
        std::vector<ColoredLines> col = colorize_contours(contours, post);
        std::fprintf(out, " |");
        for (const ColoredLines &v : col) {
            std::fprintf(out, " %zu", v.size());
            for (const ColoredLine &cl : v)
                std::fprintf(out, " %lld %lld %lld %lld %d %d %d", (long long) cl.line.a.x(), (long long) cl.line.a.y(), (long long) cl.line.b.x(), (long long) cl.line.b.y(),
                             cl.color, cl.poly_idx, cl.local_line_idx);
        }
        // The layer's graph stage as MultiMaterialSegmentation.cpp:2420-2428 runs it: one colour only
        // gives "one"; else build_graph, remove_multiple_edges_in_vertices, remove_nodes_with_one_arc and
        // extract_colored_segments (num_extruders 3: colours 0..3), per colour the polygons with their points.
        std::fprintf(out, " #");
        if (has_layer_only_one_color(col)) {
            std::fprintf(out, " one");
        } else {
            MMU_Graph graph = build_graph(0, col, []() {});
            remove_multiple_edges_in_vertices(graph, col);
            graph.remove_nodes_with_one_arc();
            std::vector<ExPolygons> segs = extract_colored_segments(graph, 3);
            std::fprintf(out, " %zu", segs.size());
            for (const ExPolygons &v : segs) {
                std::fprintf(out, " %zu", v.size());
                for (const ExPolygon &ex : v) {
                    std::fprintf(out, " %zu %zu", ex.holes.size(), ex.contour.points.size());
                    for (const Point &p : ex.contour.points) std::fprintf(out, " %lld %lld", (long long) p.x(), (long long) p.y());
                }
            }
        }
        std::fprintf(out, "\n");
    }
    std::fclose(out);
    return 0;
}
