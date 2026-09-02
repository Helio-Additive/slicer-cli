// C ABI shim around the REAL boost::polygon Voronoi builder.
//
// Mirrors libslic3r's construction exactly (Geometry/Voronoi.hpp):
//   using voronoi_diagram_type = boost::polygon::voronoi_diagram<double>;
//   construct_voronoi(segments.begin(), segments.end(), &vd);
// with the default voronoi builder (int32 input coordinates — every scaled
// libslic3r coordinate fed to Voronoi fits i32, the codebase asserts this at
// its clipper boundaries too). Compiled -O3 -DNDEBUG against the same nix
// boost headers as the native engine, so every f64 vertex coordinate rounds
// identically to the C++ slicer.
//
// The diagram is marshalled into flat index-based arrays: pointers in the
// boost structures become indices into the corresponding container (edges are
// stored in boost's edge vector order, which is stable and matches what the
// native consumers iterate).

#include <cstdint>
#include <cstdlib>
#include <vector>

#include "boost/polygon/point_data.hpp"
#include "boost/polygon/segment_data.hpp"
#include "boost/polygon/voronoi.hpp"

using boost::polygon::voronoi_diagram;

extern "C" {

typedef struct {
    // Vertices: num_vertices * 2 doubles (x, y), in vd.vertices() order.
    double *vert_xy;
    int32_t num_vertices;
    // Edges: num_edges * 6 int32 in vd.edges() order:
    //   [vertex0, vertex1, twin, next, prev, cell]  (-1 = null pointer)
    int32_t *edges;
    // Per-edge flags: bit0 = is_primary, bit1 = is_curved, bit2 = is_finite.
    uint8_t *edge_flags;
    int32_t num_edges;
    // Cells: num_cells * 3 int32 in vd.cells() order:
    //   [source_index, incident_edge (-1 = null), source_category]
    // source_category: raw boost::polygon::SourceCategory value.
    int32_t *cells;
    // Per-cell flags: bit0 = contains_point, bit1 = contains_segment,
    // bit2 = is_degenerate.
    uint8_t *cell_flags;
    int32_t num_cells;
} BvDiagram;

// Segments: num_segs * 4 int32 (x1, y1, x2, y2).
BvDiagram bv_construct_segments(const int32_t *seg_xy, int32_t num_segs) {
    using segment_type = boost::polygon::segment_data<int32_t>;
    std::vector<segment_type> segments;
    segments.reserve(size_t(num_segs));
    for (int32_t i = 0; i < num_segs; ++i) {
        const int32_t *s = seg_xy + 4 * i;
        segments.emplace_back(boost::polygon::point_data<int32_t>(s[0], s[1]),
                              boost::polygon::point_data<int32_t>(s[2], s[3]));
    }

    voronoi_diagram<double> vd;
    boost::polygon::construct_voronoi(segments.begin(), segments.end(), &vd);

    BvDiagram out{};
    out.num_vertices = int32_t(vd.num_vertices());
    out.num_edges = int32_t(vd.num_edges());
    out.num_cells = int32_t(vd.num_cells());

    out.vert_xy = (double *) malloc(sizeof(double) * 2 * size_t(out.num_vertices));
    out.edges = (int32_t *) malloc(sizeof(int32_t) * 6 * size_t(out.num_edges));
    out.edge_flags = (uint8_t *) malloc(size_t(out.num_edges));
    out.cells = (int32_t *) malloc(sizeof(int32_t) * 3 * size_t(out.num_cells));
    out.cell_flags = (uint8_t *) malloc(size_t(out.num_cells));

    const auto *vert0 = vd.vertices().empty() ? nullptr : &vd.vertices()[0];
    const auto *edge0 = vd.edges().empty() ? nullptr : &vd.edges()[0];
    const auto *cell0 = vd.cells().empty() ? nullptr : &vd.cells()[0];

    for (int32_t i = 0; i < out.num_vertices; ++i) {
        out.vert_xy[2 * i] = vd.vertices()[size_t(i)].x();
        out.vert_xy[2 * i + 1] = vd.vertices()[size_t(i)].y();
    }
    for (int32_t i = 0; i < out.num_edges; ++i) {
        const auto &e = vd.edges()[size_t(i)];
        int32_t *rec = out.edges + 6 * i;
        rec[0] = e.vertex0() ? int32_t(e.vertex0() - vert0) : -1;
        rec[1] = e.vertex1() ? int32_t(e.vertex1() - vert0) : -1;
        rec[2] = e.twin() ? int32_t(e.twin() - edge0) : -1;
        rec[3] = e.next() ? int32_t(e.next() - edge0) : -1;
        rec[4] = e.prev() ? int32_t(e.prev() - edge0) : -1;
        rec[5] = e.cell() ? int32_t(e.cell() - cell0) : -1;
        out.edge_flags[i] = uint8_t((e.is_primary() ? 1 : 0) | (e.is_curved() ? 2 : 0) |
                                    (e.is_finite() ? 4 : 0));
    }
    for (int32_t i = 0; i < out.num_cells; ++i) {
        const auto &c = vd.cells()[size_t(i)];
        int32_t *rec = out.cells + 3 * i;
        rec[0] = int32_t(c.source_index());
        rec[1] = c.incident_edge() ? int32_t(c.incident_edge() - edge0) : -1;
        rec[2] = int32_t(c.source_category());
        out.cell_flags[i] = uint8_t((c.contains_point() ? 1 : 0) |
                                    (c.contains_segment() ? 2 : 0) |
                                    (c.is_degenerate() ? 4 : 0));
    }
    return out;
}

void bv_free(BvDiagram d) {
    free(d.vert_xy);
    free(d.edges);
    free(d.edge_flags);
    free(d.cells);
    free(d.cell_flags);
}

} // extern "C"
