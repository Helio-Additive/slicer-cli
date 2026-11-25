// Native vectors for tests/stl_ascii_native_parity.rs: the pin's TriangleMesh::ReadSTLFile
// (TriangleMesh.cpp:237-243 -> admesh stl_open, stlinit.cpp:126-148, 157-330 -> from_stl(repair=true),
// TriangleMesh.cpp:204-235) on ASCII STL files given in argv. Per file: "FILE <name> OK <nv> <nf>"
// then the vertices as float bit patterns and the triangles, or "FILE <name> FAIL".
#include "libslic3r/TriangleMesh.hpp"
#include <cstdio>
#include <cstring>
#include <string>

using namespace Slic3r;

static unsigned bits(float f) { unsigned u; std::memcpy(&u, &f, 4); return u; }

int main(int argc, char **argv)
{
    for (int a = 1; a < argc; ++a) {
        std::string path = argv[a];
        std::string name = path.substr(path.find_last_of('/') + 1);
        TriangleMesh mesh;
        if (! mesh.ReadSTLFile(path.c_str(), true)) {
            printf("FILE %s FAIL\n", name.c_str());
            continue;
        }
        const indexed_triangle_set &its = mesh.its;
        printf("FILE %s OK %zu %zu\n", name.c_str(), its.vertices.size(), its.indices.size());
        for (const stl_vertex &v : its.vertices)
            printf("V %08x %08x %08x\n", bits(v.x()), bits(v.y()), bits(v.z()));
        for (const stl_triangle_vertex_indices &t : its.indices)
            printf("F %d %d %d\n", t(0), t(1), t(2));
    }
    return 0;
}
