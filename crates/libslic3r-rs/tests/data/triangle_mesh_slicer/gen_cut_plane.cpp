// R946: the pin's cut_mesh by the plane of four points (TriangleMeshSlicer.cpp:2727-2788: calc_plane_normal,
// rotation_from_two_vectors, extract_euler_angles, its_transform there and back around cut_mesh at z = 0) on meshes
// of the pin's makers, with tilted planes (normal up and down, so the points are reversed), vertical planes and
// horizontal planes (the cut at the height of the first point).
// Usage: gen_cut_plane <out.txt>; gzip -9n the output into cut_plane_vectors.txt.gz.
#include <cstdio>
#include <cstdint>
#include <vector>
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/TriangleMeshSlicer.hpp"

using namespace Slic3r;

static uint64_t g_seed = 0xD1B54A32D192ED03ULL;
static double rnd() { g_seed = g_seed * 6364136223846793005ULL + 1442695040888963407ULL; return double(g_seed >> 11) / double(1ULL << 53); }

static void dump(FILE *f, const char *tag, const indexed_triangle_set &its)
{
    std::fprintf(f, "%s %zu %zu\n", tag, its.vertices.size(), its.indices.size());
    for (const Vec3f &v : its.vertices) std::fprintf(f, "v %a %a %a\n", double(v.x()), double(v.y()), double(v.z()));
    for (const auto &t : its.indices) std::fprintf(f, "t %d %d %d\n", t(0), t(1), t(2));
}

int main(int argc, char **argv)
{
    if (argc < 2) { std::fprintf(stderr, "usage: gen_cut_plane <out.txt>\n"); return 2; }
    FILE *f = std::fopen(argv[1], "w");
    int k = 0;
    for (int rep = 0; rep < 8; ++rep)
        for (int kind = 0; kind < 5; ++kind) {
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
            BoundingBoxf3 bb;
            for (const Vec3f &v : its.vertices) bb.merge(v.cast<double>());
            Vec3d c = bb.center() + Vec3d((rnd() - 0.5) * 0.3 * bb.size().x(), (rnd() - 0.5) * 0.3 * bb.size().y(), (rnd() - 0.5) * 0.3 * bb.size().z());
            Vec3d n;
            switch (rep % 4) {
            case 0: n = Vec3d(rnd() - 0.5, rnd() - 0.5, 0.5 + rnd()); break;      // tilted, normal up
            case 1: n = Vec3d(rnd() - 0.5, rnd() - 0.5, -0.5 - rnd()); break;     // tilted, normal down (reversed)
            case 2: n = Vec3d(rnd() - 0.5, rnd() - 0.5, 0.); break;               // vertical
            default: n = Vec3d(0., 0., rnd() < 0.5 ? 1. : -1.); break;            // horizontal
            }
            n.normalize();
            Vec3d u = (std::abs(n.z()) < 0.9 ? Vec3d::UnitZ() : Vec3d::UnitX()).cross(n).normalized();
            Vec3d w = n.cross(u);
            const double s = 20.;
            std::array<Vec3d, 4> pp = { c - s * u - s * w, c + s * u - s * w, c + s * u + s * w, c - s * u + s * w };
            indexed_triangle_set upper, lower;
            cut_mesh(its, pp, &upper, &lower, true);
            std::fprintf(f, "case %d %zu %zu\n", k, its.vertices.size(), its.indices.size());
            std::fprintf(f, "pp");
            for (const Vec3d &p : pp) std::fprintf(f, " %a %a %a", p.x(), p.y(), p.z());
            std::fprintf(f, "\n");
            for (const Vec3f &v : its.vertices) std::fprintf(f, "v %a %a %a\n", double(v.x()), double(v.y()), double(v.z()));
            for (const auto &t : its.indices) std::fprintf(f, "t %d %d %d\n", t(0), t(1), t(2));
            dump(f, "upper", upper);
            dump(f, "lower", lower);
            ++k;
        }
    std::fclose(f);
    std::fprintf(stderr, "cases %d\n", k);
    return 0;
}
