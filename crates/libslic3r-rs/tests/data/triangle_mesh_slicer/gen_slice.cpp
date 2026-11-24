// R946: the pin's slice_mesh (both overloads, TriangleMeshSlicer.cpp:1982-2056 and 2059-2119), slice_mesh_ex
// (the main one 2121-2170 and the two inline ones of TriangleMeshSlicer.hpp:78-95), the vector-of-zs
// slice_make_lines (511-535) and make_expolygons_simple (1716-1750) on meshes of the pin's makers, with
// identity and non-identity transforms, slicing planes in the middle and through vertices, all slicing modes.
// One thread (the per-layer line order of slice_make_lines depends on the thread schedule otherwise).
// Usage: gen_slice <out.txt>; gzip -9n the output into slice_vectors.txt.gz.
#include <cstdio>
#include <cstdint>
#include <vector>
#include <tbb/global_control.h>
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/TriangleMeshSlicer.cpp"

using namespace Slic3r;

static uint64_t g_seed = 0x2545F4914F6CDD1DULL;
static double rnd() { g_seed = g_seed * 6364136223846793005ULL + 1442695040888963407ULL; return double(g_seed >> 11) / double(1ULL << 53); }

static void dump_polys(FILE *f, const char *tag, const Polygons &ps)
{
    std::fprintf(f, "%s %zu\n", tag, ps.size());
    for (const Polygon &p : ps) {
        std::fprintf(f, "p %zu", p.points.size());
        for (const Point &q : p.points) std::fprintf(f, " %lld %lld", (long long) q.x(), (long long) q.y());
        std::fprintf(f, "\n");
    }
}

static void dump_ex(FILE *f, const char *tag, const ExPolygons &exs)
{
    std::fprintf(f, "%s %zu\n", tag, exs.size());
    for (const ExPolygon &ex : exs) {
        std::fprintf(f, "e %zu\n", ex.holes.size());
        dump_polys(f, "c", Polygons{ ex.contour });
        dump_polys(f, "h", ex.holes);
    }
}

int main(int argc, char **argv)
{
    if (argc < 2) { std::fprintf(stderr, "usage: gen_slice <out.txt>\n"); return 2; }
    tbb::global_control one_thread(tbb::global_control::max_allowed_parallelism, 1);
    FILE *f = std::fopen(argv[1], "w");
    int k = 0;
    for (int rep = 0; rep < 3; ++rep)
        for (int kind = 0; kind < 6; ++kind)
            for (int tk = 0; tk < 4; ++tk) {
                indexed_triangle_set its;
                switch (kind) {
                case 0: its = its_make_sphere(5. + 3. * rnd(), PI / (6. + 10. * rnd())); break;
                case 1: its = its_make_cylinder(4. + 2. * rnd(), 6. + 4. * rnd(), PI / (5. + 10. * rnd())); break;
                case 2: its = its_make_cone(4. + 2. * rnd(), 6. + 4. * rnd(), PI / (5. + 10. * rnd())); break;
                case 3: its = its_make_pyramid(6. + 4. * rnd(), 5. + 5. * rnd()); break;
                case 4: {
                    // A cube with a cube-shaped hole through it (holes in the slices), as two merged cubes minus nothing:
                    // an outer cube and an inner cube with flipped faces.
                    its = its_make_cube(8. + 4. * rnd(), 8. + 4. * rnd(), 4. + 4. * rnd());
                    indexed_triangle_set inner = its_make_cube(2. + 2. * rnd(), 2. + 2. * rnd(), 2. + 2. * rnd());
                    for (Vec3f &v : inner.vertices) v += Vec3f(3.f, 3.f, 1.f);
                    for (auto &t : inner.indices) std::swap(t(1), t(2));
                    its_merge(its, inner);
                } break;
                default: its = its_make_cube(3. + 5. * rnd(), 3. + 5. * rnd(), 2. + 4. * rnd()); break;
                }
                Vec3f off(float(-3. + 6. * rnd()), float(-3. + 6. * rnd()), float(rnd() * 0.5));
                for (Vec3f &v : its.vertices) v += off;
                Transform3d trafo = Transform3d::Identity();
                switch (tk) {
                case 0: break;
                case 1: trafo.translate(Vec3d(-20. + 40. * rnd(), -20. + 40. * rnd(), 0.)); trafo.rotate(Eigen::AngleAxisd(2. * PI * rnd(), Vec3d::UnitZ())); break;
                case 2: {
                    Vec3d axis(rnd() - 0.5, rnd() - 0.5, 1.);
                    trafo.translate(Vec3d(-5. + 10. * rnd(), -5. + 10. * rnd(), 0.));
                    trafo.rotate(Eigen::AngleAxisd(0.4 * rnd(), axis.normalized()));
                    trafo.scale(Vec3d(0.5 + rnd(), 0.5 + rnd(), 1.));
                } break;
                default:
                    trafo.rotate(Eigen::AngleAxisd(2. * PI * rnd(), Vec3d::UnitZ()));
                    trafo.scale(Vec3d(-1., 1., 1.));
                    break;
                }
                // Z extent after the transform (the transforms above keep z within [zmin, zmax] up to the tilt).
                float zmin = 1e30f, zmax = -1e30f;
                for (const Vec3f &v : its.vertices) { float z = float((trafo * v.cast<double>()).z()); zmin = std::min(zmin, z); zmax = std::max(zmax, z); }
                std::vector<float> zs;
                int nz = 2 + int(rnd() * 6.);
                for (int i = 0; i < nz; ++i) zs.push_back(zmin + float((i + 0.5) / nz) * (zmax - zmin));
                if (tk <= 1) {
                    // Planes through vertices (on-plane edges and horizontal faces): the identity and the rotation
                    // about Z keep z exactly. A vertex in the middle of the list, and for the cubes the top of the
                    // inner cube (kind 4: horizontal faces inside the object) or of the cube.
                    zs.push_back(its.vertices[its.vertices.size() / 2].z());
                    if (kind >= 4) {
                        float zt = -1e30f;
                        for (size_t i = (kind == 4 ? 8 : 0); i < its.vertices.size(); ++i) zt = std::max(zt, its.vertices[i].z());
                        zs.push_back(zt);
                    }
                    std::sort(zs.begin(), zs.end());
                    zs.erase(std::unique(zs.begin(), zs.end()), zs.end());
                }
                MeshSlicingParams params;
                params.trafo = trafo;
                const int mode = k % 4, mode_below = (k / 4) % 4;
                params.mode = MeshSlicingParams::SlicingMode(mode);
                params.mode_below = MeshSlicingParams::SlicingMode(mode_below);
                params.slicing_mode_normal_below_layer = size_t((k / 2) % 3);
                MeshSlicingParamsEx pex;
                (MeshSlicingParams&) pex = params;
                pex.closing_radius = (k % 2) ? float(0.05 + 0.1 * rnd()) : 0.f;
                pex.extra_offset = (k % 5 == 1) ? float(0.1 * rnd()) : 0.f;
                pex.resolution = (k % 3 != 2) ? 0.0025 + 0.02 * rnd() : 0.;
                const float closing_radius = float(0.2 * rnd());
                const float plane_z = zs[size_t(rnd() * zs.size())];

                std::fprintf(f, "case %d %zu %zu %zu\n", k, its.vertices.size(), its.indices.size(), zs.size());
                std::fprintf(f, "m");
                for (int r = 0; r < 4; ++r)
                    for (int c = 0; c < 4; ++c) std::fprintf(f, " %a", trafo.matrix()(r, c));
                std::fprintf(f, "\n");
                for (const Vec3f &v : its.vertices) std::fprintf(f, "v %a %a %a\n", double(v.x()), double(v.y()), double(v.z()));
                for (const auto &t : its.indices) std::fprintf(f, "t %d %d %d\n", t(0), t(1), t(2));
                std::fprintf(f, "z");
                for (float z : zs) std::fprintf(f, " %a", double(z));
                std::fprintf(f, "\n");
                std::fprintf(f, "par %d %d %zu %a %a %a %a %a\n", mode, mode_below, params.slicing_mode_normal_below_layer,
                    double(pex.closing_radius), double(pex.extra_offset), pex.resolution, double(closing_radius), double(plane_z));

                // slice_make_lines (vector of zs), as slice_mesh calls it for more than one z: the vertices through
                // transform_mesh_vertices_for_slicing, an identity transform_vertex_fn.
                std::vector<Vec3i> face_edge_ids = its_face_edge_ids(its);
                std::vector<IntersectionLines> lines = slice_make_lines(transform_mesh_vertices_for_slicing(its, trafo),
                    [](const Vec3f &p) { return p; }, its.indices, face_edge_ids, zs, []() {});
                for (size_t i = 0; i < lines.size(); ++i) {
                    std::fprintf(f, "lines %zu %zu\n", i, lines[i].size());
                    for (const IntersectionLine &l : lines[i])
                        std::fprintf(f, "l %lld %lld %lld %lld %d %d %d %d %d %u\n", (long long) l.a.x(), (long long) l.a.y(),
                            (long long) l.b.x(), (long long) l.b.y(), l.a_id, l.b_id, l.edge_a_id, l.edge_b_id, int(l.edge_type), l.flags);
                }
                // slice_mesh, vector of zs.
                std::vector<Polygons> sm = slice_mesh(its, zs, params, []() {});
                for (size_t i = 0; i < sm.size(); ++i) {
                    char tag[32]; std::snprintf(tag, sizeof(tag), "sm%zu", i);
                    dump_polys(f, tag, sm[i]);
                }
                // slice_mesh, single plane.
                dump_polys(f, "sp", slice_mesh(its, plane_z, params));
                // slice_mesh_ex, three overloads.
                std::vector<ExPolygons> ex1 = slice_mesh_ex(its, zs, pex, []() {});
                std::vector<ExPolygons> ex2 = slice_mesh_ex(its, zs, []() {});
                std::vector<ExPolygons> ex3 = slice_mesh_ex(its, zs, closing_radius, []() {});
                for (size_t i = 0; i < zs.size(); ++i) {
                    char tag[32];
                    std::snprintf(tag, sizeof(tag), "ex1_%zu", i); dump_ex(f, tag, ex1[i]);
                    std::snprintf(tag, sizeof(tag), "ex2_%zu", i); dump_ex(f, tag, ex2[i]);
                    std::snprintf(tag, sizeof(tag), "ex3_%zu", i); dump_ex(f, tag, ex3[i]);
                }
                // make_expolygons_simple over the single-plane lines of every face (as cut_mesh feeds it,
                // TriangleMeshSlicer.cpp:2442-2460 with the identity: scaled<float> XY).
                {
                    IntersectionLines pl = slice_make_lines(its.vertices,
                        [](const Vec3f &p) { return Vec3f(scaled<float>(p.x()), scaled<float>(p.y()), p.z()); },
                        its.indices, face_edge_ids, plane_z, [&its, plane_z](int face_idx) {
                            // The faces that reach the plane (slice_make_lines asserts it).
                            const Vec3i &t = its.indices[face_idx];
                            float a = its.vertices[t(0)].z(), b = its.vertices[t(1)].z(), c = its.vertices[t(2)].z();
                            return std::min(a, std::min(b, c)) <= plane_z && std::max(a, std::max(b, c)) >= plane_z;
                        });
                    dump_ex(f, "es", make_expolygons_simple(pl));
                }
                ++k;
            }
    std::fclose(f);
    std::fprintf(stderr, "cases %d\n", k);
    return 0;
}
