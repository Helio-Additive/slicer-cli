// Native parity vectors for transform_mesh_vertices_for_slicing (TriangleMeshSlicer.cpp:1958-1980, with
// is_identity 1953-1956) and the two project_mesh overloads (2403-2427), which run slice_mesh_slabs
// (2177-2280) with a non-identity trafo. R945. Usage: gen_project <out.txt>
// Per case: "case k nv nt", "m %a x16" (the Transform3d, row-major), "v %a %a %a" (mesh vertices,
// float), "t a b c", then "tv %a %a %a" per transformed vertex, "top n" / "bot n" / "uni n" (the
// polygons of project_mesh(mesh, trafo, &top, &bottom) and of project_mesh(mesh, trafo)), each
// polygon "p n x y x y ...".
// Meshes from the pin's makers at deterministic offsets; trafos: identity, translation, rotation
// about z, rotation about a tilted axis with a non-uniform scale, a mirror (negative determinant).
#include <cstdio>
#include <cstdint>
#include <vector>
#include <tbb/global_control.h>
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/TriangleMeshSlicer.cpp"

using namespace Slic3r;

static uint64_t g_seed = 0x9E3779B97F4A7C15ULL;
static double rnd() { g_seed = g_seed * 6364136223846793005ULL + 1442695040888963407ULL; return double(g_seed >> 11) / double(1ULL << 53); }

static void dump(FILE *f, const char *tag, const Polygons &ps)
{
    std::fprintf(f, "%s %zu\n", tag, ps.size());
    for (const Polygon &p : ps) {
        std::fprintf(f, "p %zu", p.points.size());
        for (const Point &q : p.points) std::fprintf(f, " %lld %lld", (long long) q.x(), (long long) q.y());
        std::fprintf(f, "\n");
    }
}

int main(int argc, char **argv)
{
    if (argc < 2) { std::fprintf(stderr, "usage: gen_project <out.txt>\n"); return 2; }
    // One thread: slice_slabs_make_lines appends lines from a tbb::parallel_for.
    tbb::global_control one_thread(tbb::global_control::max_allowed_parallelism, 1);
    FILE *f = std::fopen(argv[1], "w");
    int k = 0;
    for (int rep = 0; rep < 4; ++rep)
        for (int kind = 0; kind < 5; ++kind)
            for (int tk = 0; tk < 5; ++tk) {
                indexed_triangle_set its;
                switch (kind) {
                case 0: its = its_make_sphere(5. + 3. * rnd(), PI / (6. + 10. * rnd())); break;
                case 1: its = its_make_cylinder(4. + 2. * rnd(), 6. + 4. * rnd(), PI / (5. + 10. * rnd())); break;
                case 2: its = its_make_cone(4. + 2. * rnd(), 6. + 4. * rnd(), PI / (5. + 10. * rnd())); break;
                case 3: its = its_make_pyramid(6. + 4. * rnd(), 5. + 5. * rnd()); break;
                default: its = its_make_cube(3. + 5. * rnd(), 3. + 5. * rnd(), 2. + 4. * rnd()); break;
                }
                Vec3f off(float(-3. + 6. * rnd()), float(-3. + 6. * rnd()), float(rnd() * 0.5));
                for (Vec3f &v : its.vertices) v += off;
                Transform3d trafo = Transform3d::Identity();
                switch (tk) {
                case 0: break;
                case 1: trafo.translate(Vec3d(-20. + 40. * rnd(), -20. + 40. * rnd(), -2. + 4. * rnd())); break;
                case 2: trafo.translate(Vec3d(-5. + 10. * rnd(), -5. + 10. * rnd(), 0.)); trafo.rotate(Eigen::AngleAxisd(2. * PI * rnd(), Vec3d::UnitZ())); break;
                case 3: {
                    Vec3d axis(rnd() - 0.5, rnd() - 0.5, 1.);
                    trafo.translate(Vec3d(-5. + 10. * rnd(), -5. + 10. * rnd(), 1.));
                    trafo.rotate(Eigen::AngleAxisd(0.6 * rnd(), axis.normalized()));
                    trafo.scale(Vec3d(0.5 + rnd(), 0.5 + rnd(), 0.5 + rnd()));
                } break;
                default:
                    trafo.rotate(Eigen::AngleAxisd(2. * PI * rnd(), Vec3d::UnitZ()));
                    trafo.scale(Vec3d(-1., 1., 1.));
                    break;
                }
                std::vector<stl_vertex> tv = transform_mesh_vertices_for_slicing(its, trafo);
                Polygons top, bot;
                project_mesh(its, trafo, &top, &bot, []() {});
                Polygons uni = project_mesh(its, trafo, []() {});
                std::fprintf(f, "case %d %zu %zu\n", k, its.vertices.size(), its.indices.size());
                std::fprintf(f, "m");
                for (int r = 0; r < 4; ++r)
                    for (int c = 0; c < 4; ++c) std::fprintf(f, " %a", trafo.matrix()(r, c));
                std::fprintf(f, "\n");
                for (const Vec3f &v : its.vertices) std::fprintf(f, "v %a %a %a\n", double(v.x()), double(v.y()), double(v.z()));
                for (const auto &t : its.indices) std::fprintf(f, "t %d %d %d\n", t(0), t(1), t(2));
                for (const Vec3f &v : tv) std::fprintf(f, "tv %a %a %a\n", double(v.x()), double(v.y()), double(v.z()));
                dump(f, "top", top);
                dump(f, "bot", bot);
                dump(f, "uni", uni);
                ++k;
            }
    std::fclose(f);
    std::fprintf(stderr, "cases %d\n", k);
    return 0;
}
