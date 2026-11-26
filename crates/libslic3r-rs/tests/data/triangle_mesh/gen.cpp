// Generator of the vectors of tests/triangle_mesh_native_parity.rs: libslic3r's TriangleMesh.cpp
// at the pin (references/BambuStudio/src/libslic3r, the override layer has no copy of it) over
// synthetic meshes: the statistics of the TriangleMesh constructor, the its_* helpers, the
// transformations and queries of the TriangleMesh class, the STL writers, its_make_groove_plane.
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
//   ./gen <a scratch directory> | gzip -9 -n > $DATA/vectors.txt.gz
//
// Made on macOS arm64 with Apple clang and libc++. Lines:
//   M <name> <its>                     a mesh the following lines refer to by name
//   <op> <name> [arguments] -> <result>
// <its> is the numbers of vertices and faces, every vertex (three floats as 8 hexadecimal digits)
// and every face (three indices, decimal). Floats are 8 hexadecimal digits, doubles 16, integers
// decimal. <stats> is number_of_facets, max, min, size, volume, number_of_parts, open_edges,
// non_manifold_edges, non_manifold_vertices, has_reversed_faces and the five repaired_errors.
// A file is printed as its length and the FNV-1a 64 hash of its bytes.

#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/CutUtils.hpp"
#include "libslic3r/Geometry.hpp"
#include "libslic3r/BoundingBox.hpp"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>

// TriangleMesh.cpp:1821 defines it; no header of the pin declares it.
namespace Slic3r { void its_reverse_all_facets(indexed_triangle_set &its); }

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
static std::string N(long long v) { return std::to_string(v); }
static std::string V3f(const Vec3f &v) { return hF(v.x()) + " " + hF(v.y()) + " " + hF(v.z()); }
static std::string V3d(const Vec3d &v) { return hD(v.x()) + " " + hD(v.y()) + " " + hD(v.z()); }
static std::string V3i(const Vec3i &v) { return N(v.x()) + " " + N(v.y()) + " " + N(v.z()); }

static std::string ITS(const indexed_triangle_set &its)
{
    std::string s = N(its.vertices.size()) + " " + N(its.indices.size());
    for (const Vec3f &v : its.vertices) s += " " + V3f(v);
    for (const Vec3i &f : its.indices) s += " " + V3i(f);
    return s;
}
static std::string ST(const TriangleMeshStats &s)
{
    return N(s.number_of_facets) + " " + V3f(s.max) + " " + V3f(s.min) + " " + V3f(s.size) + " " + hF(s.volume) + " " + N(s.number_of_parts) + " " +
           N(s.open_edges) + " " + N(s.non_manifold_edges) + " " + N(s.non_manifold_vertices) + " " + N(s.has_reversed_faces ? 1 : 0) + " " +
           N(s.repaired_errors.edges_fixed) + " " + N(s.repaired_errors.degenerate_facets) + " " + N(s.repaired_errors.facets_removed) + " " +
           N(s.repaired_errors.facets_reversed) + " " + N(s.repaired_errors.backwards_edges);
}
static std::string MESH(const TriangleMesh &m) { return ST(m.stats()) + " | " + ITS(m.its); }
static std::string V3IS(const std::vector<Vec3i> &v)
{
    std::string s = N(v.size());
    for (const Vec3i &f : v) s += " " + V3i(f);
    return s;
}
static std::string PTS(const Points &pts)
{
    std::string s = N(pts.size());
    for (const Point &p : pts) s += " " + N(p.x()) + " " + N(p.y());
    return s;
}
static std::string BB(const BoundingBoxf3 &b) { return N(b.defined ? 1 : 0) + " " + V3d(b.min) + " " + V3d(b.max); }
static std::string FILEH(const std::string &path)
{
    std::ifstream f(path, std::ios::binary);
    std::stringstream ss;
    ss << f.rdbuf();
    const std::string d = ss.str();
    uint64_t h = 1469598103934665603ull;
    for (unsigned char c : d) {
        h ^= c;
        h *= 1099511628211ull;
    }
    char b[32];
    std::snprintf(b, sizeof(b), "%016llx", (unsigned long long) h);
    return N(d.size()) + " " + b;
}

static void out(const std::string &lhs, const std::string &rhs) { std::printf("%s -> %s\n", lhs.c_str(), rhs.c_str()); }

// A deterministic generator of small numbers (no library generator: the sequence is part of the vectors).
struct Rng
{
    uint64_t s;
    double   next()
    {
        s = s * 6364136223846793005ull + 1442695040888963407ull;
        return double((s >> 33) & 0xFFFFFF) / double(0x1000000);
    }
    double range(double a, double b) { return a + (b - a) * next(); }
};

static std::string tmpdir;

static void run_mesh(const std::string &name, const indexed_triangle_set &src, Rng &r)
{
    std::printf("M %s %s\n", name.c_str(), ITS(src).c_str());
    const size_t nf = src.indices.size();

    // The constructor's statistics (fill_initial_stats, update_bounding_box).
    {
        TriangleMesh m(src);
        out("stats " + name, ST(m.stats()));
        out("stats_flags " + name, N(m.stats().manifold() ? 1 : 0) + " " + N(m.stats().has_open_edges() ? 1 : 0) + " " +
                                       N(m.stats().has_any_issue() ? 1 : 0) + " " + N(m.stats().repaired() ? 1 : 0));
    }
    out("volume " + name, hF(its_volume(src)));
    out("avg_edge " + name, hF(its_average_edge_length(src)));
    std::vector<Vec3i> nb = its_face_neighbors(src);
    out("neighbors " + name, V3IS(nb));
    out("neighbors_par " + name, V3IS(its_face_neighbors_par(src)));
    out("edge_ids " + name, V3IS(its_face_edge_ids(src)));
    {
        int calls = 0;
        std::vector<Vec3i> ids = its_face_edge_ids(src, [&calls]() { ++calls; });
        out("edge_ids_cb " + name, V3IS(ids));
    }
    {
        std::vector<bool> mask(nf);
        for (size_t i = 0; i < nf; ++i) mask[i] = (i % 3) != 0;
        out("edge_ids_mask " + name, V3IS(its_face_edge_ids(src, mask)));
    }
    for (int assign = 0; assign < 2; ++assign) {
        std::vector<Vec3i> nb2 = nb;
        int                num = -7;
        std::vector<Vec3i> ids = its_face_edge_ids(src, nb2, assign != 0, &num);
        out("edge_ids_nb " + name + " " + N(assign), N(num) + " " + V3IS(ids) + " " + V3IS(nb2));
    }
    {
        indexed_triangle_set its = src;
        int                  n   = its_merge_vertices(its);
        out("merge_vertices " + name, N(n) + " " + ITS(its));
        int d = its_remove_degenerate_faces(its);
        out("merge_remove_degenerate " + name, N(d) + " " + ITS(its));
        int c = its_compactify_vertices(its);
        out("merge_remove_compactify " + name, N(c) + " " + ITS(its));
    }
    {
        indexed_triangle_set its = src;
        int                  d   = its_remove_degenerate_faces(its);
        out("remove_degenerate " + name, N(d) + " " + ITS(its));
    }
    {
        indexed_triangle_set its = src;
        int                  c   = its_compactify_vertices(its);
        out("compactify " + name, N(c) + " " + ITS(its));
    }
    {
        indexed_triangle_set its = src;
        its_flip_triangles(its);
        out("flip " + name, ITS(its));
    }
    {
        indexed_triangle_set its = src;
        its_reverse_all_facets(its);
        out("reverse " + name, ITS(its));
    }
    {
        std::vector<indexed_triangle_set> parts = its_split(src);
        std::string                       s     = N(parts.size());
        for (const indexed_triangle_set &p : parts) s += " | " + ITS(p);
        out("split " + name, s);
    }
    out("patches " + name, N(its_number_of_patches(src)) + " " + N(its_number_of_patches(src, nb)));
    out("splittable " + name, N(its_is_splittable(src) ? 1 : 0) + " " + N(its_is_splittable(src, nb) ? 1 : 0));
    out("open_edges " + name, N(its_num_open_edges(src)));
    {
        std::vector<Vec3f> n = its_face_normals(src);
        std::string        s = N(n.size());
        for (const Vec3f &v : n) s += " " + V3f(v);
        out("normals " + name, s);
    }
    {
        std::string s;
        for (size_t i = 0; i < nf; i += 3) {
            s += V3f(its_face_normal(src, int(i))) + " " + V3f(its_face_normal(src, src.indices[i])) + " " + V3f(its_unnormalized_normal(src, i)) + " ";
            const its_triangle t = its_triangle_vertices(src, i);
            const stl_vertex   tv[3] = {t[0], t[1], t[2]};
            s += V3f(face_normal(tv)) + " " + V3f(face_normal_normalized(tv)) + " ";
        }
        out("face_normal " + name, s + "end");
    }
    out("bbox " + name, BB(bounding_box(src)));
    {
        VertexFaceIndex vfi(src);
        std::string     s;
        for (size_t v = 0; v < src.vertices.size(); ++v) {
            s += N(vfi.count(v));
            for (auto it = vfi.begin(v); it != vfi.end(v); ++it) s += " " + N(*it);
            s += " ; ";
        }
        out("vfi " + name, s + "end");
    }
    // The triangle helpers of TriangleMesh.hpp.
    {
        std::string s;
        for (size_t i = 0; i + 1 < nf; i += 5) {
            const stl_triangle_vertex_indices &a = src.indices[i];
            const stl_triangle_vertex_indices &b = src.indices[i + 1];
            for (int k = 0; k < 3; ++k) {
                const Vec2i e = its_triangle_edge(a, k);
                s += N(its_triangle_vertex_index(a, b[k])) + " " + N(e.x()) + " " + N(e.y()) + " " + N(its_triangle_edge_index(a, e)) + " " +
                     N(its_triangle_edge_index(b, e)) + " " + N(its_triangle_edge_index(a, Vec2i(e.y(), e.x()))) + " ";
            }
            const stl_triangle_vertex_indices perms[6] = {Vec3i(a[0], a[1], a[2]), Vec3i(a[0], a[2], a[1]), Vec3i(a[1], a[0], a[2]),
                                                          Vec3i(a[1], a[2], a[0]), Vec3i(a[2], a[0], a[1]), Vec3i(a[2], a[1], a[0])};
            for (const auto &p : perms) s += N(its_triangle_vertex_the_same(a, p) ? 1 : 0) + " ";
            s += N(its_triangle_vertex_the_same(a, b) ? 1 : 0) + " ";
        }
        out("triangle_helpers " + name, s + "end");
    }
    // its_merge, its three overloads.
    {
        indexed_triangle_set a = src;
        indexed_triangle_set b = its_make_cube(1., 2., 3.);
        its_merge(a, b);
        out("its_merge " + name, ITS(a));
        std::vector<Vec3f> tris;
        Pointf3s           trisd;
        for (int i = 0; i < 6; ++i) {
            Vec3f v(float(r.range(-5, 5)), float(r.range(-5, 5)), float(r.range(-5, 5)));
            tris.emplace_back(v);
            trisd.emplace_back(v.cast<double>() * 1.1);
        }
        std::string args;
        for (const Vec3f &v : tris) args += " " + V3f(v);
        indexed_triangle_set c = src;
        its_merge(c, tris);
        out("its_merge_tris " + name + args, ITS(c));
        std::string argsd;
        for (const Vec3d &v : trisd) argsd += " " + V3d(v);
        indexed_triangle_set d = src;
        its_merge(d, trisd);
        out("its_merge_pointf3s " + name + argsd, ITS(d));
    }
    // Projections above a plane.
    {
        const BoundingBoxf3 bb = bounding_box(src);
        Transform3d         td = Transform3d::Identity();
        td.rotate(Eigen::AngleAxisd(r.range(-3, 3), Vec3d(r.range(-1, 1), r.range(-1, 1), r.range(0.1, 1)).normalized()));
        td.scale(Vec3d(r.range(0.5, 2), r.range(0.5, 2), r.range(0.5, 2)));
        td.pretranslate(Vec3d(r.range(-10, 10), r.range(-10, 10), r.range(-1, 1)));
        const Transform3f t = td.cast<float>();
        const Matrix3f    m = t.matrix().block<3, 3>(0, 0);
        std::string       margs, targs;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) margs += " " + hF(m(i, j));
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 4; ++j) targs += " " + hF(t.matrix()(i, j));
        const float zs[3] = {float(bb.min.z() - 100.), float(r.range(-1, 1)), float(r.range(-1, 1) + 0.5 * (bb.min.z() + bb.max.z()))};
        for (float z : zs) {
            Points p1, p2;
            its_collect_mesh_projection_points_above(src, m, z, p1);
            its_collect_mesh_projection_points_above(src, t, z, p2);
            out("pts_m3 " + name + " " + hF(z) + margs, PTS(p1));
            out("pts_t3 " + name + " " + hF(z) + targs, PTS(p2));
            out("hull_m3 " + name + " " + hF(z) + margs, PTS(its_convex_hull_2d_above(src, m, z).points));
            out("hull_t3 " + name + " " + hF(z) + targs, PTS(its_convex_hull_2d_above(src, t, z).points));
        }
    }
    // The STL and OBJ writers.
    {
        const std::string fa = tmpdir + "/a.stl", fb = tmpdir + "/b.stl", fo = tmpdir + "/o.obj";
        bool              ok = its_write_stl_ascii(fa.c_str(), "label", src);
        out("stl_ascii " + name, N(ok ? 1 : 0) + " " + FILEH(fa));
        ok = its_write_stl_binary(fb.c_str(), "label", src);
        out("stl_binary " + name, N(ok ? 1 : 0) + " " + FILEH(fb));
        TriangleMesh m(src);
        ok = m.write_ascii(fa.c_str());
        out("c_write_ascii " + name, N(ok ? 1 : 0) + " " + FILEH(fa));
        ok = m.write_binary(fb.c_str());
        out("c_write_binary " + name, N(ok ? 1 : 0) + " " + FILEH(fb));
        m.WriteOBJFile(fo.c_str());
        out("c_write_obj " + name, FILEH(fo));
        if (nf > 0) {
            ok = its_store_triangle(src, fo.c_str(), nf / 2);
            out("store_triangle " + name + " " + N(nf / 2), N(ok ? 1 : 0) + " " + FILEH(fo));
            ok = its_store_triangles(src, fo.c_str(), {0, nf - 1, nf / 3});
            out("store_triangles " + name + " 0 " + N(nf - 1) + " " + N(nf / 3), N(ok ? 1 : 0) + " " + FILEH(fo));
        }
    }

    // The TriangleMesh class.
    {
        const float f = float(r.range(0.3, 3));
        TriangleMesh m(src);
        m.scale(f);
        out("c_scale " + name + " " + hF(f), MESH(m));
        const Vec3f v(float(r.range(0.3, 3)), float(r.range(0.3, 3)), float(r.range(0.3, 3)));
        m.scale(v);
        out("c_scale3 " + name + " " + V3f(v), MESH(m));
        const Vec3f d(float(r.range(-9, 9)), float(r.range(-9, 9)), float(r.range(-9, 9)));
        m.translate(d);
        out("c_translate3 " + name + " " + V3f(d), MESH(m));
        m.translate(d.y(), d.z(), d.x());
        out("c_translate " + name + " " + V3f(d), MESH(m));
        m.translate(0.f, 0.f, 0.f);
        out("c_translate0 " + name, MESH(m));
        m.align_to_origin();
        out("c_align " + name, MESH(m));
        out("c_volume " + name, hF(m.volume()));
    }
    for (int ax = 0; ax < 3; ++ax) {
        const float  a = float(r.range(-3.2, 3.2));
        TriangleMesh m(src);
        m.rotate(a, Axis(ax));
        out("c_rotate_axis " + name + " " + hF(a) + " " + N(ax), MESH(m));
        if (ax == 0) m.rotate_x(a * 0.5f);
        if (ax == 1) m.rotate_y(a * 0.5f);
        if (ax == 2) m.rotate_z(a * 0.5f);
        out("c_rotate_xyz " + name + " " + hF(a * 0.5f) + " " + N(ax), MESH(m));
        m.rotate(0.f, Axis(ax));
        out("c_rotate_axis0 " + name + " " + N(ax), MESH(m));
        m.mirror(Axis(ax));
        out("c_mirror " + name + " " + N(ax), MESH(m));
        if (ax == 0) m.mirror_y();
        if (ax == 1) m.mirror_z();
        if (ax == 2) m.mirror_x();
        out("c_mirror_xyz " + name + " " + N(ax), MESH(m));
    }
    {
        const float  a = float(r.range(-3.2, 3.2));
        const Vec3d  axis(r.range(-2, 2), r.range(-2, 2), r.range(0.2, 2));
        TriangleMesh m(src);
        m.rotate(a, axis);
        out("c_rotate_vec " + name + " " + hF(a) + " " + V3d(axis), MESH(m));
        const double ad = r.range(-170, 170);
        Point        c(coord_t(r.range(-20, 20)), coord_t(r.range(-20, 20)));
        m.rotate(ad, &c);
        out("c_rotate_center " + name + " " + hD(ad) + " " + N(c.x()) + " " + N(c.y()), MESH(m));
        m.flip_triangles();
        out("c_flip " + name, MESH(m));
    }
    for (int k = 0; k < 2; ++k) {
        Transform3d t = Transform3d::Identity();
        t.rotate(Eigen::AngleAxisd(r.range(-3, 3), Vec3d(r.range(-1, 1), r.range(-1, 1), r.range(0.1, 1)).normalized()));
        t.scale(Vec3d(r.range(0.5, 2) * (k == 1 ? -1. : 1.), r.range(0.5, 2), r.range(0.5, 2)));
        t.pretranslate(Vec3d(r.range(-10, 10), r.range(-10, 10), r.range(-10, 10)));
        std::string targs, margs;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 4; ++j) targs += " " + hD(t.matrix()(i, j));
        const Matrix3d m3 = t.matrix().block<3, 3>(0, 0);
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) margs += " " + hD(m3(i, j));
        for (int fix = 0; fix < 2; ++fix) {
            TriangleMesh m(src);
            m.transform(t, fix != 0);
            out("c_transform " + name + " " + N(fix) + targs, MESH(m));
            TriangleMesh m2(src);
            m2.transform(m3, fix != 0);
            out("c_transform3 " + name + " " + N(fix) + margs, MESH(m2));
        }
        TriangleMesh m(src);
        out("c_tbbox " + name + targs, BB(m.transformed_bounding_box(t)));
        const BoundingBoxf3 tb = m.transformed_bounding_box(t);
        const double        zs[3] = {tb.min.z() - 1., 0.5 * (tb.min.z() + tb.max.z()), tb.max.z() + 1.};
        for (double z : zs) out("c_tbbox_z " + name + " " + hD(z) + targs, BB(m.transformed_bounding_box(t, z)));
    }
    {
        TriangleMesh m(src);
        out("c_bbox " + name, BB(m.bounding_box()) + " " + BB(bounding_box(m)) + " " + V3d(m.size()) + " " + V3d(m.center()));
        out("c_misc " + name, N(m.facets_count()) + " " + N(m.empty() ? 1 : 0) + " " + N(m.is_splittable() ? 1 : 0) + " " + N(m.memsize()) + " " +
                                  N(m.release_optional()));
        out("c_hull " + name, PTS(m.convex_hull().points));
        for (float det : {1.f, 0.001f}) {
            std::vector<TriangleMesh> parts = m.split(det);
            std::string               s     = N(parts.size());
            for (const TriangleMesh &p : parts) s += " | " + MESH(p);
            out("c_split " + name + " " + hF(det), s);
            std::vector<std::unordered_map<int, int>> rel;
            std::vector<TriangleMesh>                 parts2 = m.split_and_save_relationship(rel, det);
            s = N(parts2.size());
            for (const TriangleMesh &p : parts2) s += " | " + MESH(p);
            s += " # " + N(rel.size());
            for (const auto &mp : rel) {
                std::vector<std::pair<int, int>> kv(mp.begin(), mp.end());
                std::sort(kv.begin(), kv.end());
                s += " | " + N(kv.size());
                for (const auto &p : kv) s += " " + N(p.first) + " " + N(p.second);
            }
            out("c_split_rel " + name + " " + hF(det), s);
        }
        TriangleMesh other(its_make_cube(1.5, 0.5, 2.5));
        other.translate(Vec3f(3.f, -2.f, 0.25f));
        TriangleMesh merged(src);
        merged.merge(other);
        out("c_merge " + name, MESH(merged));
        if (nf >= 4) {
            TriangleMesh son;
            m.extract_son_mesh(son, 1, int(nf) - 2);
            out("c_son " + name + " 1 " + N(int(nf) - 2), ITS(son.its));
            TriangleMesh son2;
            m.extract_son_mesh(son2, 2, int(nf));
            out("c_son " + name + " 2 " + N(int(nf)), ITS(son2.its));
        }
        m.clear();
        out("c_clear " + name, MESH(m));
    }
}

static indexed_triangle_set transformed(indexed_triangle_set its, Rng &r)
{
    Transform3d t = Transform3d::Identity();
    t.rotate(Eigen::AngleAxisd(r.range(-3, 3), Vec3d(r.range(-1, 1), r.range(-1, 1), r.range(0.1, 1)).normalized()));
    t.scale(Vec3d(r.range(0.5, 2), r.range(0.5, 2), r.range(0.5, 2)));
    t.pretranslate(Vec3d(r.range(-10, 10), r.range(-10, 10), r.range(-10, 10)));
    its_transform(its, t);
    return its;
}

int main(int argc, char **argv)
{
    // TriangleMesh.cpp 2028-2059. Check the C-string label, the little-endian
    // face count and the complete file on both sides of the one-byte count.
    if (argc == 3 && std::string(argv[1]) == "--binary-headers") {
        const std::string file = std::string(argv[2]) + "/headers.stl";
        const std::vector<std::string> labels = {
            "", "cube", std::string("A\0B", 3), std::string(80, 'X'),
            std::string(79, 'A') + "\xc3\xa9", std::string(81, 'Y')
        };
        auto mesh = its_make_cube(1., 2., 3.);
        const auto face = mesh.indices.front();
        for (size_t i = 0; i < labels.size(); ++i) {
            for (size_t count : {0, 1, 255, 256}) {
                mesh.indices.assign(count, face);
                if (!its_write_stl_binary(file.c_str(), labels[i].c_str(), mesh)) return 1;
                std::printf("%zu %zu %s\n", i, count, FILEH(file).c_str());
            }
        }
        return 0;
    }
    tmpdir = argc > 1 ? argv[1] : ".";
    Rng r{20240917ull};

    run_mesh("cube", its_make_cube(2.5, 3.25, 0.7), r);
    run_mesh("cylinder", transformed(its_make_cylinder(5, 10, 2 * PI / 12), r), r);
    run_mesh("sphere", transformed(its_make_sphere(3, 2 * PI / 10), r), r);
    {
        // Two parts.
        indexed_triangle_set a = transformed(its_make_cube(2, 3, 4), r);
        indexed_triangle_set b = its_make_cone(1.5, 2.5, 2 * PI / 9);
        for (Vec3f &v : b.vertices) v += Vec3f(20.f, 1.f, -3.f);
        its_merge(a, b);
        run_mesh("two_parts", a, r);
    }
    {
        // Open edges: a cube without its last two faces.
        indexed_triangle_set a = its_make_cube(4, 5, 6);
        a.indices.pop_back();
        a.indices.pop_back();
        run_mesh("open", a, r);
    }
    {
        // A triangle soup with duplicated vertices, two degenerate faces and an unused vertex.
        const indexed_triangle_set c = transformed(its_make_cylinder(3, 4, 2 * PI / 7), r);
        indexed_triangle_set       a;
        for (const Vec3i &f : c.indices) {
            const int base = int(a.vertices.size());
            for (int k = 0; k < 3; ++k) a.vertices.emplace_back(c.vertices[f[k]]);
            a.indices.emplace_back(base, base + 1, base + 2);
        }
        a.vertices.emplace_back(100.f, 100.f, 100.f);
        a.indices.emplace_back(0, 0, 5);
        a.indices.emplace_back(4, 7, 7);
        run_mesh("soup", a, r);
    }
    {
        // One face reversed.
        indexed_triangle_set a = transformed(its_make_cube(3, 3, 3), r);
        std::swap(a.indices[5][1], a.indices[5][2]);
        run_mesh("one_reversed", a, r);
    }
    {
        // Two cubes that share an edge once their vertices are merged: a non-manifold edge.
        indexed_triangle_set a = its_make_cube(1, 1, 1);
        indexed_triangle_set b = its_make_cube(1, 1, 1);
        for (Vec3f &v : b.vertices) v += Vec3f(1.f, 1.f, 0.f);
        its_merge(a, b);
        its_merge_vertices(a);
        run_mesh("shared_edge", a, r);
    }
    {
        // All faces reversed: a negative volume.
        indexed_triangle_set a = transformed(its_make_prism(2.f, 3.f, 4.f), r);
        its_flip_triangles(a);
        run_mesh("inside_out", a, r);
    }

    // its_make_groove_plane.
    {
        const float g[][5] = {{5.f, 4.f, 1.0471976f, 0.f, 20.f}, {3.f, 6.f, 0.5f, 0.3f, 12.f}, {2.5f, 2.f, 0.f, 0.f, 8.f}, {4.f, 3.f, 0.9f, 0.6f, 30.f}};
        for (auto &a : g) {
            Groove groove;
            groove.depth       = a[0];
            groove.width       = a[1];
            groove.flaps_angle = a[2];
            groove.angle       = a[3];
            std::vector<Vec3d>   verts;
            indexed_triangle_set its = its_make_groove_plane(groove, a[4], verts);
            std::string          s   = ITS(its) + " | " + N(verts.size());
            for (const Vec3d &v : verts) s += " " + V3d(v);
            out("groove " + hF(a[0]) + " " + hF(a[1]) + " " + hF(a[2]) + " " + hF(a[3]) + " " + hF(a[4]), s);
        }
    }
    return 0;
}
