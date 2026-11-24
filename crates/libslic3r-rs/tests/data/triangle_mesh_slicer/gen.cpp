// Native parity vectors for slice_mesh_slabs (TriangleMeshSlicer.cpp:2177-2280) and the loop
// chaining it runs through make_slab_loops (1589-1710) -> chain_lines_by_triangle_connectivity
// (1063-1161). R941. Usage: gen <out.txt>
// Per case: "case k nv nt nz", "v %a %a %a" (mesh vertices, float), "t a b c", "z %a" (float),
// then for every layer "top i npoly" / "bot i npoly" and per polygon "p n x y x y ...".
// Meshes from the pin's makers (sphere, cylinder, cone, pyramid, cube) at deterministic offsets;
// slicing planes are a regular grid mixed with the z of mesh vertices, so that triangles and
// horizontal faces lie on the planes and loops chain through shared vertices.
#include <cstdio>
#include <cstdint>
#include <vector>
#include <algorithm>
#include <tbb/global_control.h>
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/TriangleMeshSlicer.hpp"

using namespace Slic3r;

static uint64_t g_seed = 0x2545F4914F6CDD1DULL;
static double rnd() { g_seed = g_seed * 6364136223846793005ULL + 1442695040888963407ULL; return double(g_seed >> 11) / double(1ULL << 53); }

static void dump(FILE *f, const char *tag, const std::vector<Polygons> &layers)
{
    for (size_t i = 0; i < layers.size(); ++i) {
        std::fprintf(f, "%s %zu %zu\n", tag, i, layers[i].size());
        for (const Polygon &p : layers[i]) {
            std::fprintf(f, "p %zu", p.points.size());
            for (const Point &q : p.points) std::fprintf(f, " %lld %lld", (long long) q.x(), (long long) q.y());
            std::fprintf(f, "\n");
        }
    }
}

int main(int argc, char **argv)
{
    if (argc < 2) { std::fprintf(stderr, "usage: gen <out.txt>\n"); return 2; }
    // One thread: slice_facet_with_slabs emits lines from a tbb::parallel_for (1:1 order on one thread).
    tbb::global_control one_thread(tbb::global_control::max_allowed_parallelism, 1);
    FILE *f = std::fopen(argv[1], "w");
    int k = 0;
    for (int rep = 0; rep < 6; ++rep)
        for (int kind = 0; kind < 8; ++kind) {
            indexed_triangle_set its;
            switch (kind) {
            case 0: its = its_make_sphere(5. + 3. * rnd(), PI / (6. + 10. * rnd())); break;
            case 1: its = its_make_cylinder(4. + 2. * rnd(), 6. + 4. * rnd(), PI / (5. + 10. * rnd())); break;
            case 2: its = its_make_cone(4. + 2. * rnd(), 6. + 4. * rnd(), PI / (5. + 10. * rnd())); break;
            case 3: its = its_make_pyramid(6. + 4. * rnd(), 5. + 5. * rnd()); break;
            case 4: its = its_make_cube(3. + 5. * rnd(), 3. + 5. * rnd(), 2. + 4. * rnd()); break;
            case 6: case 7: {
                // Cubes touching at corners / edges (a checkerboard, kind 7 also stacked), vertices
                // merged: the projections pinch at shared vertices and the chaining meets ties.
                its.clear();
                int n = 2 + (rep % 2);
                for (int i = 0; i < n; ++i)
                    for (int j = 0; j < n; ++j)
                        for (int l = 0; l < (kind == 7 ? 2 : 1); ++l)
                            if ((i + j + l) % 2 == 0) {
                                indexed_triangle_set c = its_make_cube(2., 2., 1.);
                                for (Vec3f &v : c.vertices) v += Vec3f(2.f * i, 2.f * j, 1.f * l);
                                its_merge(its, c);
                            }
                its_merge_vertices(its);
            } break;
            default: {
                // Two cubes stacked with a shared face plane (a step): horizontal faces on a plane.
                indexed_triangle_set a = its_make_cube(6., 6., 2.);
                indexed_triangle_set b = its_make_cube(3., 3., 2.);
                for (Vec3f &v : b.vertices) v += Vec3f(1.5f, 1.5f, 2.f);
                its = a;
                its_merge(its, b);
            } break;
            }
            Vec3f off(float(-3. + 6. * rnd()), float(-3. + 6. * rnd()), float(rnd() * 0.5));
            for (Vec3f &v : its.vertices) v += off;
            float zmin = 1e10f, zmax = -1e10f;
            for (const Vec3f &v : its.vertices) { zmin = std::min(zmin, v.z()); zmax = std::max(zmax, v.z()); }
            std::vector<float> zs;
            double lh = 0.2 + 0.2 * rnd();
            for (double z = zmin + 0.5 * lh; z < zmax; z += lh) zs.push_back(float(z));
            // Planes through mesh vertices (every vertex z, or a sample of them).
            for (size_t i = 0; i < its.vertices.size(); i += (rep % 2 ? 1 : 3)) zs.push_back(its.vertices[i].z());
            std::sort(zs.begin(), zs.end());
            zs.erase(std::unique(zs.begin(), zs.end()), zs.end());
            std::vector<Polygons> top, bot;
            slice_mesh_slabs(its, zs, Transform3d::Identity(), &top, &bot, nullptr, []() {});
            std::fprintf(f, "case %d %zu %zu %zu\n", k, its.vertices.size(), its.indices.size(), zs.size());
            for (const Vec3f &v : its.vertices) std::fprintf(f, "v %a %a %a\n", double(v.x()), double(v.y()), double(v.z()));
            for (const auto &t : its.indices) std::fprintf(f, "t %d %d %d\n", t(0), t(1), t(2));
            for (float z : zs) std::fprintf(f, "z %a\n", double(z));
            dump(f, "top", top);
            dump(f, "bot", bot);
            ++k;
        }
    std::fclose(f);
    std::fprintf(stderr, "cases %d\n", k);
    return 0;
}
