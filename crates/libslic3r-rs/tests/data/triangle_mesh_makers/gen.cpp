// Generator of the vectors of tests/triangle_mesh_makers_native_parity.rs: the mesh makers of
// libslic3r's TriangleMesh.cpp at the pin (its_make_xoy_center_rect, its_make_cube,
// its_make_prism, its_make_cylinder, its_make_thread, its_make_cone, its_make_frustum_dowel,
// its_make_pyramid, its_make_sphere, its_make_snap; references/BambuStudio/src/libslic3r, the
// override layer has no copy of TriangleMesh.cpp).
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
//   ./gen | gzip -9 -n > $DATA/vectors.txt.gz
//
// Made on macOS arm64 with Apple clang and libc++. One line per mesh: the maker's name, its
// arguments (doubles as 16 hexadecimal digits, floats as 8, integers decimal), `->`, the numbers
// of vertices and faces, every vertex (three floats as 8 hexadecimal digits) and every face (three
// indices, decimal).

#include "libslic3r/TriangleMesh.hpp"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>

using namespace Slic3r;

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
    std::snprintf(b, sizeof(b), "%08x", u);
    return b;
}

static void out(const std::string &head, const indexed_triangle_set &its)
{
    std::string s = head + " -> " + std::to_string(its.vertices.size()) + " " + std::to_string(its.indices.size());
    for (const Vec3f &v : its.vertices) s += " " + hF(v.x()) + " " + hF(v.y()) + " " + hF(v.z());
    for (const Vec3i &f : its.indices) s += " " + std::to_string(f.x()) + " " + std::to_string(f.y()) + " " + std::to_string(f.z());
    std::printf("%s\n", s.c_str());
}

int main()
{
    const float rect[][3] = {{2.f, 3.f, 0.5f}, {1.5f, 0.25f, 4.f}};
    for (auto &a : rect) out("rect " + hF(a[0]) + " " + hF(a[1]) + " " + hF(a[2]), its_make_xoy_center_rect(a[0], a[1], a[2]));
    const double cube[][3] = {{1, 1, 1}, {2.5, 3.25, 0.7}, {10, 20, 30}};
    for (auto &a : cube) out("cube " + hD(a[0]) + " " + hD(a[1]) + " " + hD(a[2]), its_make_cube(a[0], a[1], a[2]));
    const float prism[][3] = {{1.f, 2.f, 3.f}, {0.3f, 0.7f, 1.1f}};
    for (auto &a : prism) out("prism " + hF(a[0]) + " " + hF(a[1]) + " " + hF(a[2]), its_make_prism(a[0], a[1], a[2]));
    const double cyl[][3] = {{1, 1, 2 * PI / 8}, {5, 10, 2 * PI / 36}, {0.5, 2, PI / 18}, {2.5, 0.3, 0.1}};
    for (auto &a : cyl) out("cylinder " + hD(a[0]) + " " + hD(a[1]) + " " + hD(a[2]), its_make_cylinder(a[0], a[1], a[2]));
    const double thr[][4] = {{3, 3, 1.5, 2 * PI / 16}, {2, 2, 0.7, 2 * PI / 12}};
    for (auto &a : thr)
        out("thread " + hD(a[0]) + " " + hD(a[1]) + " " + hD(a[2]) + " " + hD(a[3]), its_make_thread(a[0], a[1], a[2], a[3]));
    const double cone[][3] = {{1, 1, 2 * PI / 8}, {5, 10, 2 * PI / 36}, {2, 1, 0.1}};
    for (auto &a : cone) out("cone " + hD(a[0]) + " " + hD(a[1]) + " " + hD(a[2]), its_make_cone(a[0], a[1], a[2]));
    const double dowel[][2] = {{1, 1}, {1, 1}, {1, 1}, {1, 1}, {2.5, 4}, {1, 1}, {0.75, 1.5}};
    const int dowel_n[] = {3, 4, 6, 8, 5, 36, 7};
    for (int i = 0; i < 7; ++i)
        out("dowel " + hD(dowel[i][0]) + " " + hD(dowel[i][1]) + " " + std::to_string(dowel_n[i]),
            its_make_frustum_dowel(dowel[i][0], dowel[i][1], dowel_n[i]));
    const float pyr[][2] = {{2.f, 3.f}, {0.7f, 1.3f}};
    for (auto &a : pyr) out("pyramid " + hF(a[0]) + " " + hF(a[1]), its_make_pyramid(a[0], a[1]));
    const double sph[][2] = {{1, PI / 4}, {12.8, PI / 12}, {3, 0.3}};
    for (auto &a : sph) out("sphere " + hD(a[0]) + " " + hD(a[1]), its_make_sphere(a[0], a[1]));
    const double snap_rh[][2] = {{1, 1}, {2.5, 4}, {1, 2}, {0.5, 0.75}};
    const float snap_sb[][2] = {{0.3f, 0.15f}, {0.25f, 0.1f}, {0.4f, 0.2f}, {0.1f, 0.05f}};
    for (int i = 0; i < 4; ++i)
        out("snap " + hD(snap_rh[i][0]) + " " + hD(snap_rh[i][1]) + " " + hF(snap_sb[i][0]) + " " + hF(snap_sb[i][1]),
            its_make_snap(snap_rh[i][0], snap_rh[i][1], snap_sb[i][0], snap_sb[i][1]));
    return 0;
}
