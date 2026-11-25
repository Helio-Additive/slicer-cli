// Native vectors for tests/stl_repair_native_parity.rs: admesh's whole import repair as the pin runs it
// (TriangleMesh.cpp:102-197 trianglemesh_repair_on_import, step by step with the admesh functions it
// calls, then TriangleMesh.cpp:204-235 from_stl: stl_generate_shared_vertices and the its_volume flip),
// on the binary STL files given in argv (written by mk_inputs.py). Also checks that
// TriangleMesh::ReadSTLFile(path, true) gives the same indexed set.
// Per file: "FILE <name>", "S0 ..." the stats after stl_open, "S1 ..." after the repair, the repaired
// facets "R <v0 v1 v2 normal as float bits>" and neighbors "N n0 n1 n2 w0 w1 w2", then the mesh
// "V" / "F" lines and "M <same as ReadSTLFile 0/1>".
#include "libslic3r/TriangleMesh.hpp"
#include "admesh/stl.h"
#include <cstdio>
#include <cstring>
#include <string>

using namespace Slic3r;

static unsigned bits(float f) { unsigned u; std::memcpy(&u, &f, 4); return u; }

static void print_stats(const char *tag, const stl_stats &s)
{
    printf("%s %u %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n", tag,
           s.number_of_facets, bits(s.min(0)), bits(s.min(1)), bits(s.min(2)), bits(s.max(0)), bits(s.max(1)), bits(s.max(2)),
           bits(s.size(0)), bits(s.size(1)), bits(s.size(2)), bits(s.bounding_diameter), bits(s.shortest_edge), bits(s.volume),
           s.connected_edges, s.connected_facets_1_edge, s.connected_facets_2_edge, s.connected_facets_3_edge,
           s.facets_w_1_bad_edge, s.facets_w_2_bad_edge, s.facets_w_3_bad_edge, s.edges_fixed, s.degenerate_facets,
           s.facets_removed, s.facets_added, s.facets_reversed, s.backwards_edges, s.normals_fixed, s.number_of_parts);
}

int main(int argc, char **argv)
{
    for (int a = 1; a < argc; ++a) {
        std::string path = argv[a];
        std::string name = path.substr(path.find_last_of('/') + 1);
        printf("FILE %s\n", name.c_str());
        stl_file stl;
        if (! stl_open(&stl, path.c_str())) {
            printf("FAIL\n");
            continue;
        }
        print_stats("S0", stl.stats);
        // TriangleMesh.cpp:102-197, in order.
        if (stl.stats.number_of_facets > 0) {
            stl_check_facets_exact(&stl);
            stl.stats.facets_w_1_bad_edge = (stl.stats.connected_facets_2_edge - stl.stats.connected_facets_3_edge);
            stl.stats.facets_w_2_bad_edge = (stl.stats.connected_facets_1_edge - stl.stats.connected_facets_2_edge);
            stl.stats.facets_w_3_bad_edge = (stl.stats.number_of_facets - stl.stats.connected_facets_1_edge);
            float tolerance = (float)stl.stats.shortest_edge;
            float increment = (float)stl.stats.bounding_diameter / 10000.0f;
            int iterations = 2;
            if (stl.stats.connected_facets_3_edge < int(stl.stats.number_of_facets)) {
                for (int i = 0; i < iterations; ++ i) {
                    if (stl.stats.connected_facets_3_edge < int(stl.stats.number_of_facets)) {
                        stl_check_facets_nearby(&stl, tolerance);
                        tolerance += increment;
                    } else
                        break;
                }
            }
            if (stl.stats.connected_facets_3_edge < (int)stl.stats.number_of_facets)
                stl_remove_unconnected_facets(&stl);
            stl_fix_normal_directions(&stl);
            stl_fix_normal_values(&stl);
            stl_calculate_volume(&stl);
            stl_verify_neighbors(&stl);
            if (auto nr_degenerated = stl.stats.degenerate_facets; stl.stats.number_of_facets > 0 && nr_degenerated > 0)
                stl_check_facets_exact(&stl);
        }
        print_stats("S1", stl.stats);
        for (uint32_t i = 0; i < stl.stats.number_of_facets; ++ i) {
            const stl_facet &f = stl.facet_start[i];
            printf("R");
            for (int k = 0; k < 3; ++ k)
                printf(" %08x %08x %08x", bits(f.vertex[k](0)), bits(f.vertex[k](1)), bits(f.vertex[k](2)));
            printf(" %08x %08x %08x\n", bits(f.normal(0)), bits(f.normal(1)), bits(f.normal(2)));
            const stl_neighbors &n = stl.neighbors_start[i];
            printf("N %d %d %d %d %d %d\n", n.neighbor[0], n.neighbor[1], n.neighbor[2], (int)n.which_vertex_not[0],
                   (int)n.which_vertex_not[1], (int)n.which_vertex_not[2]);
        }
        indexed_triangle_set its;
        stl_generate_shared_vertices(&stl, its);
        float vol = its_volume(its);
        if (vol < 0.)
            its_flip_triangles(its);
        printf("VOL %08x\n", bits(vol));
        for (const stl_vertex &v : its.vertices)
            printf("V %08x %08x %08x\n", bits(v.x()), bits(v.y()), bits(v.z()));
        for (const stl_triangle_vertex_indices &t : its.indices)
            printf("F %d %d %d\n", t(0), t(1), t(2));
        TriangleMesh mesh;
        bool ok = mesh.ReadSTLFile(path.c_str(), true);
        bool same = ok && mesh.its.vertices == its.vertices && mesh.its.indices == its.indices;
        printf("M %d\n", same ? 1 : 0);
    }
    return 0;
}
