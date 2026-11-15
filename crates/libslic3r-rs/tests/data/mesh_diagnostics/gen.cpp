// Reference vectors for crates/libslic3r-rs/src/mesh_diagnostics.rs and mesh_raycast.rs, taken from
// the C++ at the pin.
//
// The program compiles references/BambuStudio/src/libslic3r/MeshDiagnostics.cpp and MeshRaycast.cpp
// as they are, with the pin's headers (Point.hpp, AABBTreeIndirect.hpp, admesh/stl.h, libigl's
// Hit.h), Boost.Log and TBB. Eigen is the one `pkg-config eigen3` names, as for
// crates/eigen-transform-sys: the rounding of Vec3d::dot depends on the Eigen version
// (docs/rust-engine/README.md).
//
// Build and run, from the repository root (macOS arm64 clang is the reference build; PREFIX is where
// Boost and TBB are installed; STUB is a directory holding an empty libslic3r_version.h, the header
// CMake generates):
//   clang++ -w -std=c++17 -O2 -ffp-contract=on -DBOOST_LOG_DYN_LINK -I $STUB \
//       -I references/BambuStudio/src/libslic3r -I references/BambuStudio/src \
//       -I references/BambuStudio/src/libigl $(pkg-config --cflags eigen3) -I $PREFIX/include \
//       crates/libslic3r-rs/tests/data/mesh_diagnostics/gen.cpp \
//       -L $PREFIX/lib -lboost_log -lboost_thread -ltbb -o /tmp/md_gen
//   /tmp/md_gen > crates/libslic3r-rs/tests/data/mesh_diagnostics/vectors.txt
//
// Output: a stream of whitespace separated tokens (floats and doubles as the hex of their bits).
//   N count
// per mesh:
//   M nv nf  3*nv floats  3*nf indices
//   E open_edges non_manifold_edges same_direction_edges  3*nf neighbors     its_edge_diagnostics
//   Q open non_manifold same_direction has_reversed_faces                    its_quick_diagnostics
//   D open non_manifold_edges non_manifold_vertices same_direction has_reversed_faces
//                                                                            its_mesh_diagnostics
//   S has_reversed_faces      its_detect_reversed_faces on stats that are all zero (the ray test)
//   R n { origin(3) dir(3) -> hits { face t u v } }    TriangleRaycaster::all_hits; n is 0 for a
//                                                      mesh with an index out of range

#include "MeshDiagnostics.cpp"
#include "MeshRaycast.cpp"

#include <boost/log/core.hpp>
#include <cstdio>
#include <cstring>

// Log.hpp asks this before it logs; it is defined in Log.cpp. Nothing is logged here: the log would
// go to the same stream as the vectors.
namespace Slic3r { namespace log_detail { bool log_enabled(LogLevel) { return false; } } }

using namespace Slic3r;

static uint64_t s = 0x9E3779B97F4A7C15ull;
static uint64_t rnd() { s ^= s << 13; s ^= s >> 7; s ^= s << 17; return s; }
static uint64_t rnd(uint64_t n) { return rnd() % n; }
static float    rf(float lo, float hi) { return lo + (hi - lo) * float(rnd(1000001)) / 1000000.f; }
static void pd(double v) { uint64_t b; memcpy(&b, &v, 8); printf(" %llx", (unsigned long long) b); }
static void pf(float v) { uint32_t b; memcpy(&b, &v, 4); printf(" %x", b); }

struct Part { std::vector<Vec3f> v; std::vector<Vec3i> f; };

static Part box()
{
    Part p;
    for (int i = 0; i < 8; ++ i) p.v.emplace_back(float(i & 1), float((i >> 1) & 1), float((i >> 2) & 1));
    static const int q[6][4] = { {0,2,3,1}, {4,5,7,6}, {0,1,5,4}, {2,6,7,3}, {0,4,6,2}, {1,3,7,5} };
    for (const auto &quad : q) { p.f.emplace_back(quad[0], quad[1], quad[2]); p.f.emplace_back(quad[0], quad[2], quad[3]); }
    return p;
}
static Part tetrahedron()
{
    Part p;
    p.v = { Vec3f(0, 0, 0), Vec3f(1, 0, 0), Vec3f(0, 1, 0), Vec3f(0, 0, 1) };
    p.f = { Vec3i(0, 2, 1), Vec3i(0, 1, 3), Vec3i(1, 2, 3), Vec3i(0, 3, 2) };
    return p;
}
static Part sphere(int lat, int lon)
{
    Part p;
    p.v.emplace_back(0.f, 0.f, 1.f);
    for (int i = 1; i < lat; ++ i)
        for (int j = 0; j < lon; ++ j) {
            const double th = M_PI * i / lat, ph = 2. * M_PI * j / lon;
            p.v.emplace_back(float(std::sin(th) * std::cos(ph)), float(std::sin(th) * std::sin(ph)), float(std::cos(th)));
        }
    p.v.emplace_back(0.f, 0.f, -1.f);
    const int last = int(p.v.size()) - 1;
    auto at = [lon](int i, int j) { return 1 + (i - 1) * lon + (j % lon); };
    for (int j = 0; j < lon; ++ j) p.f.emplace_back(0, at(1, j), at(1, j + 1));
    for (int i = 1; i + 1 < lat; ++ i)
        for (int j = 0; j < lon; ++ j) {
            p.f.emplace_back(at(i, j), at(i + 1, j), at(i + 1, j + 1));
            p.f.emplace_back(at(i, j), at(i + 1, j + 1), at(i, j + 1));
        }
    for (int j = 0; j < lon; ++ j) p.f.emplace_back(last, at(lat - 1, j + 1), at(lat - 1, j));
    return p;
}
static Part torus(int n, int m)
{
    Part p;
    for (int i = 0; i < n; ++ i)
        for (int j = 0; j < m; ++ j) {
            const double a = 2. * M_PI * i / n, b = 2. * M_PI * j / m;
            p.v.emplace_back(float((1. + 0.35 * std::cos(b)) * std::cos(a)), float((1. + 0.35 * std::cos(b)) * std::sin(a)), float(0.35 * std::sin(b)));
        }
    auto at = [n, m](int i, int j) { return (i % n) * m + (j % m); };
    for (int i = 0; i < n; ++ i)
        for (int j = 0; j < m; ++ j) {
            p.f.emplace_back(at(i, j), at(i + 1, j), at(i + 1, j + 1));
            p.f.emplace_back(at(i, j), at(i + 1, j + 1), at(i, j + 1));
        }
    return p;
}

// A closed part, scaled, sheared a little and moved; flipped inside out when asked.
static void add(indexed_triangle_set &its, Part p, float scale, const Vec3f &offset, bool flip, bool exact)
{
    const Vec3f k(rf(0.5f, 1.5f), rf(0.5f, 1.5f), rf(0.5f, 1.5f));
    const float shear = exact ? 0.f : rf(-0.3f, 0.3f);
    const int   base  = int(its.vertices.size());
    for (const Vec3f &v : p.v) {
        Vec3f w = exact ? Vec3f(v * scale) : Vec3f(v.x() * k.x() + shear * v.y(), v.y() * k.y(), v.z() * k.z() + shear * v.x()) * scale;
        its.vertices.emplace_back(w + offset);
    }
    for (const Vec3i &f : p.f)
        its.indices.emplace_back(flip ? Vec3i(base + f[0], base + f[2], base + f[1]) : Vec3i(base + f[0], base + f[1], base + f[2]));
}

static Part part(int kind)
{
    switch (kind % 5) {
    case 0: return box();
    case 1: return tetrahedron();
    case 2: return sphere(3 + int(rnd(3)), 4 + int(rnd(4)));
    case 3: return torus(5 + int(rnd(3)), 4 + int(rnd(2)));
    default: return sphere(2, 3 + int(rnd(3)));
    }
}

static void emit(const indexed_triangle_set &its, bool rays)
{
    printf("M %zu %zu", its.vertices.size(), its.indices.size());
    for (const Vec3f &v : its.vertices) { pf(v.x()); pf(v.y()); pf(v.z()); }
    for (const Vec3i &f : its.indices) printf(" %d %d %d", f[0], f[1], f[2]);
    std::vector<Vec3i> neighbors(3, Vec3i(7, 7, 7));
    MeshDiagnosticStats e = its_edge_diagnostics(its, &neighbors);
    printf("\nE %zu %zu %zu", e.open_edges, e.non_manifold_edges, e.same_direction_edges);
    // An empty mesh leaves the vector as it was.
    if (! its.indices.empty())
        for (const Vec3i &n : neighbors) printf(" %d %d %d", n[0], n[1], n[2]);
    MeshDiagnosticStats q = its_quick_diagnostics(its);
    printf("\nQ %zu %zu %zu %d", q.open_edges, q.non_manifold_edges, q.same_direction_edges, int(q.has_reversed_faces));
    MeshDiagnosticStats d = its_mesh_diagnostics(its);
    printf("\nD %zu %zu %zu %zu %d", d.open_edges, d.non_manifold_edges, d.non_manifold_vertices, d.same_direction_edges, int(d.has_reversed_faces));
    MeshDiagnosticStats z;
    its_detect_reversed_faces(its, z);
    printf("\nS %d", int(z.has_reversed_faces));

    bool valid = ! its.indices.empty() && ! its.vertices.empty();
    for (const Vec3i &f : its.indices)
        for (int k = 0; k < 3; ++ k) valid = valid && f[k] >= 0 && size_t(f[k]) < its.vertices.size();
    const int num_rays = rays && valid ? 8 : 0;
    printf("\nR %d\n", num_rays);
    if (num_rays == 0) return;
    const TriangleRaycaster caster(its);
    Vec3f lo = its.vertices.front(), hi = lo;
    for (const Vec3f &v : its.vertices) { lo = lo.cwiseMin(v); hi = hi.cwiseMax(v); }
    const Vec3d center = 0.5 * (lo + hi).cast<double>();
    const double size  = double((hi - lo).norm()) + 1.;
    for (int r = 0; r < num_rays; ++ r) {
        Vec3d origin, dir;
        if (r == 0) {
            // Along an axis through the centre.
            dir    = Vec3d(0., 0., -1.);
            origin = center - dir * size;
        } else if (r == 1) {
            // Through a vertex.
            origin = center + Vec3d(size, 0.3 * size, -0.2 * size);
            dir    = its.vertices[rnd(its.vertices.size())].cast<double>() - origin;
        } else if (r == 2) {
            dir    = Vec3d(0., 0., 0.);
            origin = center;
        } else {
            origin = center + Vec3d(double(rf(-1.f, 1.f)), double(rf(-1.f, 1.f)), double(rf(-1.f, 1.f))) * size;
            const Vec3d target = center + Vec3d(double(rf(-1.f, 1.f)), double(rf(-1.f, 1.f)), double(rf(-1.f, 1.f))) * (0.3 * size);
            dir = target - origin;
            if (r == 3) dir.normalize();
        }
        int coplanar = -1;
        const std::vector<RayMeshHit> hits = caster.all_hits(origin, dir, &coplanar);
        pd(origin.x()); pd(origin.y()); pd(origin.z()); pd(dir.x()); pd(dir.y()); pd(dir.z());
        printf(" -> %zu", hits.size());
        for (const RayMeshHit &h : hits) { printf(" %d", h.face); pd(h.t); pd(h.u); pd(h.v); }
        printf("\n");
    }
}

int main()
{
    // The log would go to the same stream as the vectors.
    boost::log::core::get()->set_logging_enabled(false);

    const int num_soups = 450, num_closed = 54, num_special = 6;
    printf("N %d\n", num_soups + num_closed + num_special);

    // Triangle soups over a few vertices: open, non-manifold and same-direction edges, bowtie
    // vertices, degenerate faces, indices out of range.
    for (int i = 0; i < num_soups; ++ i) {
        indexed_triangle_set its;
        const size_t nv = 3 + rnd(8);
        for (size_t k = 0; k < nv; ++ k) its.vertices.emplace_back(rf(-10.f, 10.f), rf(-10.f, 10.f), rf(-10.f, 10.f));
        const size_t nf = rnd(16);
        const bool   bad = rnd(8) == 0;
        for (size_t k = 0; k < nf; ++ k) {
            Vec3i f(int(rnd(nv)), int(rnd(nv)), int(rnd(nv)));
            if (rnd(4) != 0)
                while (f[1] == f[0]) f[1] = int(rnd(nv));
            if (rnd(4) != 0)
                while (f[2] == f[0] || f[2] == f[1]) f[2] = int(rnd(nv));
            if (bad && rnd(4) == 0) f[rnd(3)] = rnd(2) ? -1 - int(rnd(3)) : int(nv + rnd(3));
            its.indices.emplace_back(f);
            // The same face again, or its mirror image.
            if (rnd(8) == 0) its.indices.emplace_back(rnd(2) ? f : Vec3i(f[0], f[2], f[1]));
        }
        emit(its, false);
    }

    // Closed meshes, one or two parts, each either way round; some with a damaged face.
    for (int i = 0; i < num_closed; ++ i) {
        indexed_triangle_set its;
        const bool  exact = rnd(4) == 0;
        const float scale = rnd(3) == 0 ? rf(0.5f, 2.f) : rnd(2) ? rf(20.f, 150.f) : 1.f;
        add(its, part(i), scale, exact ? Vec3f(0.f, 0.f, 0.f) : Vec3f(rf(-50.f, 50.f), rf(-50.f, 50.f), rf(0.f, 50.f)), rnd(3) == 0, exact);
        if (rnd(3) == 0) {
            // A second part: beside the first, or inside it.
            const bool  inside = rnd(2);
            const Vec3f c      = its.vertices.front();
            add(its, part(int(rnd(5))), inside ? 0.05f * scale : scale, inside ? Vec3f(c * 0.999f) : Vec3f(c + Vec3f(4.f * scale, 0.f, 0.f)), rnd(2), exact);
        }
        switch (rnd(10)) {
        case 0: { Vec3i &f = its.indices[rnd(its.indices.size())]; std::swap(f[1], f[2]); break; }
        case 1: its.indices.erase(its.indices.begin() + rnd(its.indices.size())); break;
        default: break;
        }
        emit(its, true);
    }

    // The corners of the ray test.
    {
        indexed_triangle_set its;                      // no faces
        its.vertices.emplace_back(1.f, 2.f, 3.f);
        emit(its, false);
        its.vertices.clear();                          // faces, no vertices
        its.indices.emplace_back(0, 1, 2);
        emit(its, false);
        its.vertices.assign(3, Vec3f(1.f, 2.f, 3.f));  // zero radius
        emit(its, true);
        its.indices.assign(1, Vec3i(0, 1, 5));         // index out of range
        emit(its, true);
        indexed_triangle_set flat;                     // two coincident sheets: closed, no volume
        flat.vertices = { Vec3f(0, 0, 0), Vec3f(10, 0, 0), Vec3f(0, 10, 0) };
        flat.indices  = { Vec3i(0, 1, 2), Vec3i(0, 2, 1) };
        emit(flat, true);
        indexed_triangle_set thin;                     // a sliver of a box
        add(thin, box(), 1.f, Vec3f(0.f, 0.f, 0.f), true, true);
        for (Vec3f &v : thin.vertices) v.z() *= 1e-7f;
        emit(thin, true);
    }
    return 0;
}
