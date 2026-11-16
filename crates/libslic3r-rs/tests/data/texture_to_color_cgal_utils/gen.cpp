// Reference vectors of tests/texture_to_color_cgal_utils_native_parity.rs and
// tests/texture_to_color_repair_native_parity.rs: Bambu Studio's TextureToColor/CgalUtils.hpp and
// Repair.hpp at the pin over CGAL, and the parts of CGAL that `cgal_utils.rs` writes out
// (Surface_mesh, the kernel's constructions, the closest point of an AABB tree), called with
// deterministic random inputs.
//
// Build, with REPO = this repository, PIN = $REPO/references/BambuStudio, BUILD = a configured
// build tree of the Bambu Studio engine (for the generated libslic3r headers), EIGEN = the Eigen
// include directory (`pkg-config --cflags eigen3`), PREFIX = where Boost, CGAL, TBB, GMP and MPFR
// are installed (`brew --prefix` on macOS) and DATA = this directory:
//
//   clang++ -std=c++17 -O2 -ffp-contract=on -DNDEBUG -w \
//     -include $REPO/libslic3r/bambustudio/cgal_54_compat.hpp \
//     -I$PIN/src -I$PIN/src/libslic3r -I$REPO/libslic3r/bambustudio/libigl -I$PIN/src/libigl \
//     -I$PIN/src/admesh -I$BUILD -I$BUILD/src/libslic3r -I$EIGEN -I$PREFIX/include \
//     $DATA/gen.cpp -o gen -L$PREFIX/lib -ltbb -lgmp -lmpfr
//   for part in mesh kernel tree; do
//     ./gen $part > $DATA/vectors_$part.txt
//   done
//
// The vectors in the repository were made with CGAL 6.1.1.
//
// Format: one call per line: a tag, the arguments, `->`, the result. A double is its 16 hex
// digits, a float its 8 hex digits, everything else decimal. A mesh is `nv` then 3*nv floats, `nf`
// then 3*nf indices. A line `mesh` gives the mesh the lines after it are called with.

#include <fstream>
#include <map>
#include <cstdint>
#include <cstring>
#include <CGAL/AABB_face_graph_triangle_primitive.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <CGAL/centroid.h>
#include "libslic3r/TextureToColor/CgalUtils.hpp"
#include "libslic3r/TextureToColor/Repair.hpp"
#include "libslic3r/Log.hpp"

namespace Slic3r { namespace log_detail { bool log_enabled(LogLevel) { return false; } } }

using namespace Slic3r;
using namespace Slic3r::tex2color;
using cgalutils::CGALMesh;
typedef cgalutils::Kernel K;

static uint64_t g_state = 0x9E3779B97F4A7C15ull;
static uint64_t xs() {
    g_state ^= g_state << 13;
    g_state ^= g_state >> 7;
    g_state ^= g_state << 17;
    return g_state;
}
static double unit() { return (double) (xs() >> 11) / 9007199254740992.0; }
static double range(double lo, double hi) { return lo + (hi - lo) * unit(); }
static std::size_t below(std::size_t n) { return (std::size_t) (xs() % n); }

static void pd(double v) { uint64_t u; std::memcpy(&u, &v, 8); std::printf(" %016llx", (unsigned long long) u); }
static void pf(float v) { uint32_t u; std::memcpy(&u, &v, 4); std::printf(" %08x", u); }
static void pu(std::size_t v) { std::printf(" %zu", v); }
static void p3d(const std::array<double, 3>& a) { pd(a[0]); pd(a[1]); pd(a[2]); }
static void p3u(const std::array<std::size_t, 3>& a) { pu(a[0]); pu(a[1]); pu(a[2]); }
static void tag(const char* t) { std::printf("%s", t); }
static void arrow() { std::printf(" ->"); }
static void nl() { std::printf("\n"); }

static void pmesh(const TriMesh& m) {
    pu(m.vertices.size());
    for (auto& v : m.vertices) { pf(v.x()); pf(v.y()); pf(v.z()); }
    pu(m.indices.size());
    for (auto& f : m.indices) std::printf(" %d %d %d", f[0], f[1], f[2]);
}

// ---------------------------------------------------------------------------------------------
// meshes

// An open height field of nx by ny cells, each cell split along one of its diagonals.
static TriMesh grid_mesh(int nx, int ny, double noise, Vec3f offset = Vec3f(0, 0, 0)) {
    TriMesh m;
    for (int y = 0; y <= ny; ++y)
        for (int x = 0; x <= nx; ++x)
            m.vertices.push_back(Vec3f((float) (x + range(-0.2, 0.2)), (float) (y + range(-0.2, 0.2)), (float) range(-noise, noise)) + offset);
    auto id = [&](int x, int y) { return y * (nx + 1) + x; };
    for (int y = 0; y < ny; ++y)
        for (int x = 0; x < nx; ++x) {
            int a = id(x, y), b = id(x + 1, y), c = id(x + 1, y + 1), d = id(x, y + 1);
            if (below(2)) {
                m.indices.push_back(Vec3i(a, b, c));
                m.indices.push_back(Vec3i(a, c, d));
            } else {
                m.indices.push_back(Vec3i(a, b, d));
                m.indices.push_back(Vec3i(b, c, d));
            }
        }
    return m;
}

// A closed torus of nu by nv cells.
static TriMesh torus_mesh(int nu, int nv, double noise, Vec3f offset = Vec3f(0, 0, 0)) {
    TriMesh m;
    for (int u = 0; u < nu; ++u)
        for (int v = 0; v < nv; ++v) {
            double a = 2.0 * 3.14159265358979323846 * u / nu, b = 2.0 * 3.14159265358979323846 * v / nv;
            double r = 2.0 + (1.0 + range(-noise, noise)) * std::cos(b);
            m.vertices.push_back(Vec3f((float) (r * std::cos(a)), (float) (r * std::sin(a)), (float) ((1.0 + range(-noise, noise)) * std::sin(b))) + offset);
        }
    auto id = [&](int u, int v) { return ((u % nu) * nv) + (v % nv); };
    for (int u = 0; u < nu; ++u)
        for (int v = 0; v < nv; ++v) {
            int a = id(u, v), b = id(u + 1, v), c = id(u + 1, v + 1), d = id(u, v + 1);
            if (below(2)) {
                m.indices.push_back(Vec3i(a, b, c));
                m.indices.push_back(Vec3i(a, c, d));
            } else {
                m.indices.push_back(Vec3i(a, b, d));
                m.indices.push_back(Vec3i(b, c, d));
            }
        }
    return m;
}

static void append(TriMesh& dst, const TriMesh& src) {
    int base = (int) dst.vertices.size();
    for (auto& v : src.vertices) dst.vertices.push_back(v);
    for (auto& f : src.indices) dst.indices.push_back(Vec3i(f[0] + base, f[1] + base, f[2] + base));
}

// The faces in a random order (the indices CGAL gives the halfedges depend on it).
static void shuffle_faces(TriMesh& m) {
    for (std::size_t i = m.indices.size(); i > 1; --i) std::swap(m.indices[i - 1], m.indices[below(i)]);
}

// Random faces over a few vertices: edges with three faces, faces the wrong way round, a vertex
// twice in a face. CGAL refuses some of them.
static TriMesh soup_mesh(int nv, int nf) {
    TriMesh m;
    for (int i = 0; i < nv; ++i) m.vertices.push_back(Vec3f((float) range(-3, 3), (float) range(-3, 3), (float) range(-3, 3)));
    for (int i = 0; i < nf; ++i) {
        int a = (int) below(nv), b = (int) below(nv), c = (int) below(nv);
        m.indices.push_back(Vec3i(a, b, c));
    }
    return m;
}

static TriMesh pick_mesh(int kind) {
    switch (kind % 8) {
    case 0: return grid_mesh(3 + (int) below(6), 3 + (int) below(6), 0.5);
    case 1: return torus_mesh(4 + (int) below(5), 3 + (int) below(5), 0.1);
    case 2: {
        TriMesh m = grid_mesh(2 + (int) below(4), 2 + (int) below(4), 0.3);
        append(m, torus_mesh(3 + (int) below(4), 3 + (int) below(3), 0.05, Vec3f(10, 0, 0)));
        append(m, grid_mesh(1 + (int) below(3), 1 + (int) below(3), 1.0, Vec3f(0, 20, 0)));
        shuffle_faces(m);
        return m;
    }
    case 3: { TriMesh m = grid_mesh(4 + (int) below(5), 4 + (int) below(5), 2.5); shuffle_faces(m); return m; }
    case 4: { TriMesh m = torus_mesh(5 + (int) below(6), 4 + (int) below(4), 0.3); shuffle_faces(m); return m; }
    case 5: return soup_mesh(6 + (int) below(10), 4 + (int) below(30));
    case 6: {
        // a grid with holes: some faces dropped
        TriMesh m = grid_mesh(4 + (int) below(5), 4 + (int) below(5), 0.4);
        TriFaces kept;
        for (auto& f : m.indices) if (below(5) != 0) kept.push_back(f);
        m.indices = kept;
        return m;
    }
    default: {
        // two fans that touch in one vertex, and an isolated vertex
        TriMesh m = grid_mesh(2, 2, 0.2);
        TriMesh n = grid_mesh(2, 2, 0.2, Vec3f(2, 2, 0));
        int base = (int) m.vertices.size();
        for (auto& v : n.vertices) m.vertices.push_back(v);
        for (auto& f : n.indices) {
            Vec3i g(f[0] + base, f[1] + base, f[2] + base);
            for (int i = 0; i < 3; ++i) if (g[i] == base) g[i] = 8;  // the corner of the first grid
            m.indices.push_back(g);
        }
        return m;
    }
    }
}

// ---------------------------------------------------------------------------------------------

static void pp(const K::Point_3& p) { pd(p.x()); pd(p.y()); pd(p.z()); }
static void pv(const K::Vector_3& p) { pd(p.x()); pd(p.y()); pd(p.z()); }
static void pi(long v) { std::printf(" %ld", v); }

// The whole connectivity: per vertex its halfedge and its point, per halfedge target, face, next and prev, per
// face its halfedge; -1 is the null descriptor.
static void pconn(const CGALMesh& cm) {
    auto idx = [](std::size_t i) { return i == (std::size_t) (std::numeric_limits<std::uint32_t>::max)() ? -1l : (long) i; };
    pu(cm.num_vertices());
    for (auto v : cm.vertices()) { pi(idx((std::size_t) cm.halfedge(v))); pp(cm.point(v)); }
    pu(cm.num_halfedges());
    for (auto h : cm.halfedges()) { pi(idx((std::size_t) cm.target(h))); pi(idx((std::size_t) cm.face(h))); pi(idx((std::size_t) cm.next(h))); pi(idx((std::size_t) cm.prev(h))); }
    pu(cm.num_faces());
    for (auto f : cm.faces()) pi(idx((std::size_t) cm.halfedge(f)));
}

// Whether the two meshes have the same connectivity and points.
static bool same(const CGALMesh& a, const CGALMesh& b) {
    if (a.num_vertices() != b.num_vertices() || a.num_halfedges() != b.num_halfedges() || a.num_faces() != b.num_faces()) return false;
    for (auto v : a.vertices()) if (a.halfedge(v) != b.halfedge(v) || a.point(v) != b.point(v)) return false;
    for (auto h : a.halfedges()) if (a.target(h) != b.target(h) || a.face(h) != b.face(h) || a.next(h) != b.next(h) || a.prev(h) != b.prev(h)) return false;
    for (auto f : a.faces()) if (a.halfedge(f) != b.halfedge(f)) return false;
    return true;
}

static void gen_mesh() {
    for (int i = 0; i < 32; ++i) {
        TriMesh m = pick_mesh(i);
        if (i == 30) m = TriMesh();
        if (i == 31) m.indices.clear();
        tag("mesh"); pmesh(m); nl();
        CGALMesh cm = cgalutils::trimesh_to_cgal(m);
        tag("trimesh_to_cgal"); arrow(); pconn(cm); nl();
        tag("counts"); arrow(); pu(cm.number_of_vertices()); pu(cm.number_of_halfedges()); pu(cm.number_of_edges()); pu(cm.number_of_faces()); pu(CGAL::is_closed(cm)); nl();
        tag("cgal_to_trimesh"); arrow(); pmesh(cgalutils::cgal_to_trimesh(cm)); nl();
        CGALMesh cm2;
        cm2.add_vertex(K::Point_3(1, 2, 3));  // replaced by the call
        bool ok = cgalutils::convert_trimesh_to_cgal(m, cm2);
        tag("convert_trimesh_to_cgal"); arrow(); pu(ok); pu(same(cm, cm2)); nl();
        std::vector<Vec2f> uvs, cuvs;
        std::size_t nuv = i % 5 == 4 ? m.vertices.size() / 2 : (i % 5 == 3 ? m.vertices.size() + 2 : m.vertices.size());
        for (std::size_t k = 0; k < nuv; ++k) uvs.push_back(Vec2f((float) unit(), (float) unit()));
        cuvs.push_back(Vec2f(9, 9));
        ok = cgalutils::convert_trimesh_to_cgal(m, uvs, cm2, cuvs);
        tag("convert_trimesh_to_cgal_with_uvs"); pu(uvs.size()); for (auto& uv : uvs) { pf(uv.x()); pf(uv.y()); }
        arrow(); pu(ok); pu(same(cm, cm2)); pu(cuvs.size()); for (auto& uv : cuvs) { pf(uv.x()); pf(uv.y()); } nl();
        tag("is_mesh_halfedge_compatible"); arrow(); pu(cgalutils::is_mesh_halfedge_compatible(m)); nl();
        BoundaryEdgeStats st = ComputeBoundaryEdgeStats(cm);
        tag("ComputeBoundaryEdgeStats"); arrow(); pu(st.total_boundary_edges); pu(st.max_cycle_edges); pu(st.cycle_count); nl();
        std::vector<std::size_t> around;
        for (auto v : cm.vertices()) {
            if (cm.is_isolated(v)) { around.push_back(0); continue; }
            std::size_t n = 0;
            std::vector<std::size_t> hs;
            for (auto h : cm.halfedges_around_target(cm.halfedge(v))) hs.push_back((std::size_t) h);
            around.push_back(hs.size());
            for (auto h : hs) around.push_back(h);
        }
        tag("halfedges_around_target"); arrow(); pu(around.size()); for (auto x : around) pu(x); nl();
    }
    // faces that cannot be faces of a halfedge mesh
    for (int i = 0; i < 80; ++i) {
        TriMesh m = i % 2 == 0 ? soup_mesh(4 + (int) below(8), 1 + (int) below(14)) : grid_mesh(2 + (int) below(3), 2 + (int) below(3), 0.1);
        if (i % 2 == 1 && !m.indices.empty()) {
            std::size_t f = below(m.indices.size());
            switch ((i / 2) % 8) {
            case 0: m.indices[f][1] = m.indices[f][0]; break;                        // a vertex twice
            case 1: m.indices[f][2] = (int) m.vertices.size(); break;                // out of range
            case 2: m.indices[f][0] = -1; break;                                     // negative
            case 3: m.indices.push_back(m.indices[f]); break;                        // a face twice
            case 4: std::swap(m.indices[f][0], m.indices[f][1]); break;              // the wrong way round
            case 5: m.indices.push_back(Vec3i(m.indices[f][0], m.indices[f][1], (int) below(m.vertices.size()))); break;
            case 6: shuffle_faces(m); break;
            default: break;
            }
        }
        tag("mesh"); pmesh(m); nl();
        tag("is_mesh_halfedge_compatible"); arrow(); pu(cgalutils::is_mesh_halfedge_compatible(m)); nl();
    }
}

static K::Point_3 rp(double s) { return K::Point_3(range(-s, s), range(-s, s), range(-s, s)); }

static void gen_kernel() {
    for (int i = 0; i < 600; ++i) {
        K::Point_3 a = rp(5), b = rp(5), c = rp(5), q = rp(8);
        switch (i % 12) {
        case 3: c = a + (b - a) * range(-1, 2); break;                      // nearly on one line
        case 4: c = a + (b - a) * 2.0; break;
        case 5: c = b; break;                                               // two vertices the same
        case 6: b = a; c = a; break;                                        // one point
        case 7: a = K::Point_3(0, 0, 0); b = K::Point_3(1, 0, 0); c = K::Point_3(0, 1, 0); q = K::Point_3(range(-1, 2), range(-1, 2), range(-1, 1)); break;
        case 8: q = a + (b - a) * unit(); break;                            // on an edge
        case 9: q = a; break;                                               // a vertex
        case 10: q = CGAL::ORIGIN + ((a - CGAL::ORIGIN) + (b - CGAL::ORIGIN) + (c - CGAL::ORIGIN)) / 3.0; break;  // in the triangle
        case 11: a = K::Point_3((double) below(3), (double) below(3), 0); b = K::Point_3((double) below(3), (double) below(3), 0);
                 c = K::Point_3((double) below(3), (double) below(3), 0); q = K::Point_3((double) below(4), (double) below(4), (double) below(2)); break;
        default: break;
        }
        K::Point_3 r = K().construct_projected_point_3_object()(K::Triangle_3(a, b, c), q);
        tag("construct_projected_point_3"); pp(a); pp(b); pp(c); pp(q); arrow(); pp(r); nl();
    }
    for (int i = 0; i < 400; ++i) {
        K::Point_3 p = rp(5), q = rp(5), r = rp(5);
        if (i % 4 == 1) r = q;
        if (i % 4 == 2) r = p + (p - q);                                    // the mirror image: equal but for rounding
        if (i % 4 == 3) r = K::Point_3(q.x() + range(-1e-15, 1e-15), q.y(), q.z());
        if (i % 16 == 14) { p = K::Point_3(0, 0, 0); q = K::Point_3(3, 4, 0); r = K::Point_3(0, 5, 0); }
        if (i % 16 == 6) { p = K::Point_3(0.1, 0.2, 0.3); q = K::Point_3(0.1 + 3, 0.2 + 4, 0.3); r = K::Point_3(0.1, 0.2 + 5, 0.3); }
        tag("compare_distance_3"); pp(p); pp(q); pp(r); arrow(); pi((long) K().compare_distance_3_object()(p, q, r)); nl();
        tag("squared_distance"); pp(p); pp(q); arrow(); pd(CGAL::squared_distance(p, q)); nl();
    }
    for (int i = 0; i < 100; ++i) {
        std::vector<K::Point_3> pts;
        std::size_t n = 1 + below(5);
        for (std::size_t k = 0; k < n; ++k) pts.push_back(rp(100));
        tag("centroid"); pu(n); for (auto& p : pts) pp(p); arrow(); pp(CGAL::centroid(pts.begin(), pts.end())); nl();
    }
}

static void gen_tree() {
    typedef CGAL::AABB_face_graph_triangle_primitive<CGALMesh> Primitive;
    typedef CGAL::AABB_traits<K, Primitive> Traits;
    typedef CGAL::AABB_tree<Traits> Tree;
    for (int i = 0; i < 28; ++i) {
        static const int kinds[] = {0, 1, 2, 3, 4, 6, 7};
        TriMesh m = pick_mesh(kinds[i % 7]);
        CGALMesh cm = cgalutils::trimesh_to_cgal(m);
        tag("mesh"); pmesh(m); nl();
        tag("compute_face_normal"); arrow(); pu(cm.number_of_faces());
        for (auto f : cm.faces()) pv(CGAL::Polygon_mesh_processing::compute_face_normal(f, cm));
        nl();
        Tree tree(cm.faces().begin(), cm.faces().end(), cm);
        tree.accelerate_distance_queries();
        for (int k = 0; k < 24; ++k) {
            K::Point_3 q;
            if (k % 4 == 0) q = rp(12);
            else {
                // near the surface: a point of a face, moved a little
                auto f = CGALMesh::Face_index((std::uint32_t) below(cm.number_of_faces()));
                auto h = cm.halfedge(f);
                K::Point_3 a = cm.point(cm.target(h)), b = cm.point(cm.target(cm.next(h))), c = cm.point(cm.source(h));
                double u = unit(), v = unit();
                if (u + v > 1) { u = 1 - u; v = 1 - v; }
                if (k % 4 == 2) v = 0;         // on an edge
                if (k % 8 == 3) { u = 0; v = 0; }  // a vertex
                q = a + (b - a) * u + (c - a) * v;
                if (k % 4 == 1) q = q + K::Vector_3(range(-0.3, 0.3), range(-0.3, 0.3), range(-0.3, 0.3));
            }
            tag("closest_point"); pp(q); arrow(); pp(tree.closest_point(q)); nl();
        }
    }
}

int main(int argc, char** argv) {
    if (argc != 2) return 1;
    std::string what = argv[1];
    if (what == "mesh") gen_mesh();
    else if (what == "kernel") { g_state ^= 0x1111; gen_kernel(); }
    else if (what == "tree") { g_state ^= 0x2222; gen_tree(); }
    else return 1;
    return 0;
}
