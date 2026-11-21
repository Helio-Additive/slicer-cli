// Generator of the vectors of tests/multi_material_segmentation_native_parity.rs: the functions of
// libslic3r's MultiMaterialSegmentation.cpp at the pin (references/BambuStudio/src/libslic3r; the
// override layer has no copy of it) that work on an annotated Voronoi diagram.
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
// MultiMaterialSegmentation.cpp is compiled into this file: the functions are static. Their only
// caller at the pin, extract_colored_segments over the Voronoi diagram, is compiled out (#if (0),
// MultiMaterialSegmentation.cpp:1147-1283), so this generator annotates a diagram itself: the diagram
// of the lines of a few polygons, built by boost::polygon::construct_voronoi, with the infinite edges
// deleted (as the third step of that caller does; the functions read both ends of an edge that is not
// deleted), some vertices marked as on the contour and some edges deleted with their twins.
//
// A line is "vd <inputs> => <A> | <B> | <C>". The inputs: the lines (ax ay bx by each), the vertices
// marked VERTEX_ON_CONTOUR, the edges deleted (their twins are deleted with them), the vertices
// delete_vertex_deep is tried on. A: per vertex non_deleted_edge_count and can_vertex_be_deleted, then
// per edge "-" when it is deleted, else edge_length and calc_total_edge_length (a double is its 16
// hexadecimal digits). B: the colours of the vertices and of the edges after delete_vertex_deep on
// those of the listed vertices that can be deleted. C: the colours after remove_multiple_edges_in_vertex
// on every vertex that is on the contour, in the order of the diagram.
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

static FILE *g_out = nullptr;

static std::string hD(double v)
{
    uint64_t u;
    std::memcpy(&u, &v, 8);
    char b[32];
    std::snprintf(b, sizeof(b), "%016llx", (unsigned long long) u);
    return b;
}
static std::string hI(long long v) { return std::to_string(v); }
static void out(const std::string &s) { std::fprintf(g_out, "%s\n", s.c_str()); }

struct Lcg
{
    uint64_t s;
    explicit Lcg(uint64_t seed) : s(seed) {}
    uint32_t next()
    {
        s = s * 6364136223846793005ULL + 1442695040888963407ULL;
        return uint32_t(s >> 33);
    }
    // An integer in [lo, hi].
    long long range(long long lo, long long hi) { return lo + (long long) (next() % uint32_t(hi - lo + 1)); }
};

using BVD = boost::polygon::voronoi_diagram<double>;

static std::string COLORS(const BVD &vd)
{
    std::string o;
    for (const BVD::vertex_type &v : vd.vertices()) o += (o.empty() ? "" : " ") + hI((long long) v.color());
    for (const BVD::edge_type &e : vd.edges()) o += " " + hI((long long) e.color());
    return o;
}

// The lines of one to three polygons that do not touch: rectangles with cut corners, a notch in one
// side now and then (a concave corner gives a vertex several edges).
static ColoredLines random_lines(Lcg &g)
{
    ColoredLines lines;
    const int    n_polys = (int) g.range(1, 3);
    for (int poly = 0; poly < n_polys; ++poly) {
        // The polygons sit in cells of a row, so that they stay apart.
        const coord_t cx = poly * 3000000 + g.range(-200000, 200000), cy = g.range(-300000, 300000);
        const coord_t rx = g.range(300000, 1000000), ry = g.range(300000, 1000000);
        const coord_t cut_x = g.range(rx / 8, rx / 3), cut_y = g.range(ry / 8, ry / 3);
        Points pts = {Point(cx - rx + cut_x, cy - ry), Point(cx + rx - cut_x, cy - ry), Point(cx + rx, cy - ry + cut_y), Point(cx + rx, cy + ry - cut_y),
                      Point(cx + rx - cut_x, cy + ry), Point(cx - rx + cut_x, cy + ry), Point(cx - rx, cy + ry - cut_y), Point(cx - rx, cy - ry + cut_y)};
        if (g.range(0, 1) == 0) {
            // A notch in the bottom side.
            const coord_t half = (rx - cut_x) / 3, depth = g.range(ry / 4, ry);
            const coord_t at   = g.range(-half, half);
            pts.insert(pts.begin() + 1, {Point(cx + at - half / 2, cy - ry), Point(cx + at, cy - ry + depth), Point(cx + at + half / 2, cy - ry)});
        }
        const int color = (int) g.range(0, 3);
        for (size_t i = 0; i < pts.size(); ++i) {
            ColoredLine line{Line(pts[i], pts[(i + 1) % pts.size()]), (i % 3 == 0) ? (int) g.range(0, 3) : color, poly, (int) i};
            lines.emplace_back(line);
        }
    }
    return lines;
}

static void run_voronoi()
{
    Lcg g(0x9e3779b97f4a7c15ULL);
    for (int c = 0; c < 200; ++c) {
        const ColoredLines lines = random_lines(g);
        BVD                vd;
        boost::polygon::construct_voronoi(lines.begin(), lines.end(), &vd);

        std::string in = hI((long long) lines.size());
        for (const ColoredLine &l : lines) in += " " + hI(l.line.a.x()) + " " + hI(l.line.a.y()) + " " + hI(l.line.b.x()) + " " + hI(l.line.b.y());

        // The infinite edges are deleted.
        for (const BVD::edge_type &edge : vd.edges())
            if (edge.is_infinite()) {
                edge.color(VD_ANNOTATION::DELETED);
                edge.twin()->color(VD_ANNOTATION::DELETED);
            }
        // Vertices on the contour.
        std::vector<size_t> on_contour;
        for (size_t i = 0; i < vd.vertices().size(); ++i)
            if (g.range(0, 2) == 0) {
                vd.vertices()[i].color(VD_ANNOTATION::VERTEX_ON_CONTOUR);
                on_contour.push_back(i);
            }
        // Edges deleted with their twins.
        std::vector<size_t> deleted;
        for (size_t i = 0; i < vd.edges().size(); ++i)
            if (vd.edges()[i].color() != VD_ANNOTATION::DELETED && g.range(0, 6) == 0) {
                vd.edges()[i].color(VD_ANNOTATION::DELETED);
                vd.edges()[i].twin()->color(VD_ANNOTATION::DELETED);
                deleted.push_back(i);
            }
        // The vertices delete_vertex_deep is tried on.
        std::vector<size_t> tried;
        for (size_t i = 0; i < vd.vertices().size(); ++i)
            if (g.range(0, 3) == 0) tried.push_back(i);

        in += " " + hI((long long) on_contour.size());
        for (size_t i : on_contour) in += " " + hI((long long) i);
        in += " " + hI((long long) deleted.size());
        for (size_t i : deleted) in += " " + hI((long long) i);
        in += " " + hI((long long) tried.size());
        for (size_t i : tried) in += " " + hI((long long) i);

        // A
        std::string a;
        for (const BVD::vertex_type &v : vd.vertices()) a += (a.empty() ? "" : " ") + hI((long long) non_deleted_edge_count(v)) + " " + hI(can_vertex_be_deleted(v) ? 1 : 0);
        for (const BVD::edge_type &e : vd.edges())
            a += e.color() == VD_ANNOTATION::DELETED ? std::string(" -") : " " + hD(edge_length(e)) + " " + hD(calc_total_edge_length(e));
        // B
        for (size_t i : tried)
            if (can_vertex_be_deleted(vd.vertices()[i])) delete_vertex_deep(vd.vertices()[i]);
        const std::string b = COLORS(vd);
        // C
        for (const BVD::vertex_type &v : vd.vertices())
            if (v.color() == VD_ANNOTATION::VERTEX_ON_CONTOUR) remove_multiple_edges_in_vertex(v);
        out("vd " + in + " => " + a + " | " + b + " | " + COLORS(vd));
    }
}

int main(int argc, char **argv)
{
    if (argc < 2) {
        std::fprintf(stderr, "usage: gen <vectors.txt>\n");
        return 1;
    }
    g_out = std::fopen(argv[1], "w");
    if (g_out == nullptr) return 1;
    run_voronoi();
    std::fclose(g_out);
    return 0;
}
