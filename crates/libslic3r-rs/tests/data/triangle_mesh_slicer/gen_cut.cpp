// Native parity vectors for cut_mesh (TriangleMeshSlicer.cpp:2429-, z overload), which runs
// slice_facet_for_cut_mesh, triangulate_slice and make_expolygons_simple (1716-1788) for the caps.
// R945. Usage: gen_cut <out.txt>
// Per case: "case k nv nt caps", "z %a", "v %a %a %a" (mesh vertices, float), "t a b c", then for
// "upper" and "lower": "<tag> nv nt", "v ..." / "t ..." of the cut mesh and "src n f f ..." (source faces).
#include <cstdio>
#include <cstdint>
#include <vector>
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/TriangleMeshSlicer.hpp"

using namespace Slic3r;

static uint64_t g_seed = 0xD1B54A32D192ED03ULL;
static double rnd() { g_seed = g_seed * 6364136223846793005ULL + 1442695040888963407ULL; return double(g_seed >> 11) / double(1ULL << 53); }

static void dump(FILE *f, const char *tag, const indexed_triangle_set &its, const std::vector<int> &src)
{
    std::fprintf(f, "%s %zu %zu\n", tag, its.vertices.size(), its.indices.size());
    for (const Vec3f &v : its.vertices) std::fprintf(f, "v %a %a %a\n", double(v.x()), double(v.y()), double(v.z()));
    for (const auto &t : its.indices) std::fprintf(f, "t %d %d %d\n", t(0), t(1), t(2));
    std::fprintf(f, "src %zu", src.size());
    for (int s : src) std::fprintf(f, " %d", s);
    std::fprintf(f, "\n");
}

int main(int argc, char **argv)
{
    if (argc < 2) { std::fprintf(stderr, "usage: gen_cut <out.txt>\n"); return 2; }
    FILE *f = std::fopen(argv[1], "w");
    int k = 0;
    for (int rep = 0; rep < 6; ++rep)
        for (int kind = 0; kind < 6; ++kind) {
            indexed_triangle_set its;
            switch (kind) {
            case 0: its = its_make_sphere(5. + 3. * rnd(), PI / (6. + 10. * rnd())); break;
            case 1: its = its_make_cylinder(4. + 2. * rnd(), 6. + 4. * rnd(), PI / (5. + 10. * rnd())); break;
            case 2: its = its_make_cone(4. + 2. * rnd(), 6. + 4. * rnd(), PI / (5. + 10. * rnd())); break;
            case 3: its = its_make_pyramid(6. + 4. * rnd(), 5. + 5. * rnd()); break;
            case 4: its = its_make_cube(3. + 5. * rnd(), 3. + 5. * rnd(), 2. + 4. * rnd()); break;
            default: {
                // A cube with a cube-shaped hole through it in z (a hole in the cap).
                its = its_make_cube(8., 8., 4.);
                indexed_triangle_set h = its_make_cube(3., 3., 4.);
                for (Vec3f &v : h.vertices) v += Vec3f(2.5f, 2.5f, 0.f);
                its_flip_triangles(h);
                its_merge(its, h);
            } break;
            }
            Vec3f off(float(-3. + 6. * rnd()), float(-3. + 6. * rnd()), float(rnd() * 0.5));
            for (Vec3f &v : its.vertices) v += off;
            float zmin = 1e10f, zmax = -1e10f;
            for (const Vec3f &v : its.vertices) { zmin = std::min(zmin, v.z()); zmax = std::max(zmax, v.z()); }
            // A plane through the middle, or through a vertex z (rep odd).
            float z = (rep % 2) ? its.vertices[its.vertices.size() / 2].z() : float(zmin + (0.2 + 0.6 * rnd()) * (zmax - zmin));
            bool caps = rep % 3 != 2;
            indexed_triangle_set upper, lower;
            std::vector<int> upper_src, lower_src;
            cut_mesh(its, z, &upper, &lower, caps, &upper_src, &lower_src);
            std::fprintf(f, "case %d %zu %zu %d\n", k, its.vertices.size(), its.indices.size(), int(caps));
            std::fprintf(f, "z %a\n", double(z));
            for (const Vec3f &v : its.vertices) std::fprintf(f, "v %a %a %a\n", double(v.x()), double(v.y()), double(v.z()));
            for (const auto &t : its.indices) std::fprintf(f, "t %d %d %d\n", t(0), t(1), t(2));
            dump(f, "upper", upper, upper_src);
            dump(f, "lower", lower, lower_src);
            ++k;
        }
    std::fclose(f);
    std::fprintf(stderr, "cases %d\n", k);
    return 0;
}
