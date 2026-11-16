// Reference vectors of tests/triangle_selector_native_parity.rs: Bambu Studio's TriangleSelector.cpp
// at the pin, driven with deterministic random inputs.
//
// TriangleSelector.cpp is included unchanged, with its class members made public for the generator
// (`#define private public` around the include, after every header it pulls in has been read), so
// the private recursion helpers and the anonymous-namespace stream guards are called directly. The
// rest of the engine comes from the engine's own static libraries, linked as the `slicer_cli`
// program links them. Build, with REPO = this repository, BUILD = a configured and built tree of
// the Bambu Studio engine (`cmake -DENGINE=bambu`, docs/building.md) and DATA = this directory:
//
//   compile gen.cpp with the compile flags, include directories and definitions of a libslic3r
//   source of $BUILD (its `compile_commands.json`, e.g. the entry of TriangleSelector.cpp: -O3
//   -DNDEBUG -std=c++17 and `-include $REPO/libslic3r/bambustudio/cgal_54_compat.hpp`), then
//   link it with the link line of `slicer_cli` ($BUILD/CMakeFiles/slicer_cli.dir/link.txt) with
//   main.cpp's objects replaced by gen.o.
//   for part in subdivide region stream cursor; do
//     ./gen $part > $DATA/vectors_$part.txt && gzip -9n -f $DATA/vectors_$part.txt
//   done
//
// Format: one call per line: a tag, the arguments, `->`, the result. A double is its 16 hex digits,
// a float its 8 hex digits, everything else decimal. A mesh is `nv` then 3*nv floats, `nf` then
// 3*nf indices; serialized paint is the count of its entries, the (root, first bit) pairs, the count
// of its bits and the bits (`-` for none). A transform is its 4x4 matrix, row by row.
//
// The callbacks handed to set_facets_with_subdivision and paint_region compute in double one
// operation per statement, so the build's floating-point contraction cannot fuse them and the test
// can repeat them operation by operation.

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <functional>
#include <memory>
#include <mutex>
#include <queue>
#include <set>
#include <string>
#include <vector>
#include <boost/container/small_vector.hpp>

#include "libslic3r/libslic3r.h"
#include "libslic3r/Point.hpp"
#include "libslic3r/Geometry.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Utils.hpp"
#include "libslic3r/Log.hpp"

#define private public
#define protected public
#include "libslic3r/TriangleSelector.cpp"
#undef private
#undef protected

using namespace Slic3r;

using Data = std::pair<std::vector<std::pair<int, int>>, std::vector<bool>>;

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
static void pi(long v) { std::printf(" %ld", v); }
static void tag(const char* t) { std::printf("%s", t); }
static void arrow() { std::printf(" ->"); }
static void nl() { std::printf("\n"); }
static void p3f(const Vec3f& v) { pf(v.x()); pf(v.y()); pf(v.z()); }
static void pmesh(const indexed_triangle_set& m) {
    pu(m.vertices.size()); for (auto& v : m.vertices) p3f(v);
    pu(m.indices.size()); for (auto& f : m.indices) std::printf(" %d %d %d", f[0], f[1], f[2]);
}
static void pdata(const Data& d) {
    pu(d.first.size()); for (auto& p : d.first) { pi(p.first); pi(p.second); }
    pu(d.second.size()); std::printf(" "); if (d.second.empty()) std::printf("-"); for (bool b : d.second) std::printf("%d", (int) b);
}
static void ptrafo(const Transform3d& t) { for (int r = 0; r < 4; ++r) for (int c = 0; c < 4; ++c) pd(t.matrix()(r, c)); }
static void ptrafof(const Transform3f& t) { for (int r = 0; r < 4; ++r) for (int c = 0; c < 4; ++c) pf(t.matrix()(r, c)); }

// The state of the selector a test can read back: the tree, the node counts and the leaves.
static void pselector(TriangleSelector& sel) {
    pdata(sel.serialize());
    std::size_t valid = 0;
    for (auto& t : sel.m_triangles) valid += t.valid();
    pu(sel.m_triangles.size()); pu(valid); pu(sel.m_vertices.size());
}

// A grid of nx by ny cells, z ~ noise.
static indexed_triangle_set grid(int nx, int ny, double noise) {
    indexed_triangle_set m;
    for (int y = 0; y <= ny; ++y)
        for (int x = 0; x <= nx; ++x) m.vertices.push_back(Vec3f((float) (x + range(-0.2, 0.2)), (float) (y + range(-0.2, 0.2)), (float) range(-noise, noise)));
    auto id = [&](int x, int y) { return y * (nx + 1) + x; };
    for (int y = 0; y < ny; ++y)
        for (int x = 0; x < nx; ++x) {
            int a = id(x, y), b = id(x + 1, y), c = id(x + 1, y + 1), d = id(x, y + 1);
            if (below(2)) { m.indices.push_back(Vec3i(a, b, c)); m.indices.push_back(Vec3i(a, c, d)); }
            else { m.indices.push_back(Vec3i(a, b, d)); m.indices.push_back(Vec3i(b, c, d)); }
        }
    return m;
}

// A closed latitude-longitude sphere, outward oriented.
static indexed_triangle_set sphere(int rings, int segs, double radius) {
    indexed_triangle_set m;
    m.vertices.push_back(Vec3f(0.f, 0.f, (float) radius));
    for (int r = 1; r < rings; ++r) {
        double th = M_PI * double(r) / double(rings);
        for (int s = 0; s < segs; ++s) {
            double ph = 2. * M_PI * double(s) / double(segs) + range(-0.05, 0.05);
            double rr = radius * (1. + range(-0.05, 0.05));
            m.vertices.push_back(Vec3f((float) (rr * std::sin(th) * std::cos(ph)), (float) (rr * std::sin(th) * std::sin(ph)), (float) (rr * std::cos(th))));
        }
    }
    m.vertices.push_back(Vec3f(0.f, 0.f, (float) -radius));
    const int south = (int) m.vertices.size() - 1;
    auto id = [&](int r, int s) { return 1 + (r - 1) * segs + (s % segs); };
    for (int s = 0; s < segs; ++s) m.indices.push_back(Vec3i(0, id(1, s), id(1, s + 1)));
    for (int r = 1; r + 1 < rings; ++r)
        for (int s = 0; s < segs; ++s) {
            int a = id(r, s), b = id(r + 1, s), c = id(r + 1, s + 1), d = id(r, s + 1);
            m.indices.push_back(Vec3i(a, b, c));
            m.indices.push_back(Vec3i(a, c, d));
        }
    for (int s = 0; s < segs; ++s) m.indices.push_back(Vec3i(id(rings - 1, s), south, id(rings - 1, s + 1)));
    return m;
}

static indexed_triangle_set rnd_mesh() {
    if (below(3) == 0) return sphere(3 + (int) below(4), 4 + (int) below(5), range(1.5, 4.));
    return grid(2 + (int) below(5), 2 + (int) below(4), range(0., 0.4));
}

static Transform3d rnd_trafo(int kind) {
    if (kind == 0) return Transform3d::Identity();
    Vec3d rot(range(-3, 3), range(-3, 3), range(-3, 3));
    double s = range(0.5, 2);
    Vec3d sc(s, s, s);
    if (kind == 2) sc = Vec3d(range(0.5, 2), range(0.5, 2), range(0.5, 2));
    return Geometry::assemble_transform(Vec3d(range(-20, 20), range(-20, 20), range(-20, 20)), rot, sc, Vec3d(kind == 3 ? -1 : 1, 1, 1));
}

// ---------------------------------------------------------------------------------------------
// Callbacks, one floating-point operation per statement.
// ---------------------------------------------------------------------------------------------

static double tri_area(const std::array<Vec3f, 3>& t) {
    double ax = double(t[1].x()); ax = ax - double(t[0].x());
    double ay = double(t[1].y()); ay = ay - double(t[0].y());
    double az = double(t[1].z()); az = az - double(t[0].z());
    double bx = double(t[2].x()); bx = bx - double(t[0].x());
    double by = double(t[2].y()); by = by - double(t[0].y());
    double bz = double(t[2].z()); bz = bz - double(t[0].z());
    double p, q;
    p = ay * bz; q = az * by; double cx = p - q;
    p = az * bx; q = ax * bz; double cy = p - q;
    p = ax * by; q = ay * bx; double cz = p - q;
    double s = cx * cx;
    p = cy * cy; s = s + p;
    p = cz * cz; s = s + p;
    double r = std::sqrt(s);
    return 0.5 * r;
}

struct Bands {
    double band, shift;
    int    nstates, state_offset, quirk;
    int state_at(double x) const {
        double q = x + shift;
        q = q / band;
        int s = (int) std::floor(q) % nstates;
        return s < 0 ? 0 : s + (s == 0 ? 0 : state_offset);
    }
    TriangleSelector::FacetSubdivisionMeasurement operator()(int root, const std::array<Vec3f, 3>& t) const {
        TriangleSelector::FacetSubdivisionMeasurement m;
        const double area = tri_area(t);
        int s0 = state_at(t[0].x()), s1 = state_at(t[1].x()), s2 = state_at(t[2].x());
        double cx = double(t[0].x()); cx = cx + double(t[1].x()); cx = cx + double(t[2].x()); cx = cx / 3.0;
        int sc = state_at(cx);
        m.dominant_state = EnforcerBlockerType(sc);
        m.surface_area = area;
        m.error_area = (s0 == s1 && s1 == s2 && s2 == sc) ? 0.0 : area;
        if (root % 7 == 3) m.error_area = m.error_area * 0.5;
        // Out-of-range answers, which normalize_measurement() clamps.
        if (quirk == 1 && root % 5 == 1) m.error_area = m.error_area * 2.0 + 0.25;
        if (quirk == 2 && root % 5 == 2) m.surface_area = -area;
        if (quirk == 3 && root % 5 == 3) m.error_area = -1.0;
        return m;
    }
};

static Bands rnd_bands() {
    Bands b;
    b.band = range(0.3, 1.3);
    b.shift = range(0, 1);
    b.nstates = 2 + (int) below(5);
    b.state_offset = below(3) == 0 ? (int) below(30) : 0;
    b.quirk = (int) below(4);
    return b;
}

// A region: a disk of the mesh-local xy plane, or the half space x < k.
struct Region {
    int    kind;
    double cx, cy, r2, k;
    bool inside(const Vec3f& v) const {
        if (kind == 1) return double(v.x()) < k;
        double dx = double(v.x()); dx = dx - cx;
        double dy = double(v.y()); dy = dy - cy;
        double d = dx * dx;
        double e = dy * dy;
        d = d + e;
        return d <= r2;
    }
    TriangleSelector::RegionCoverage operator()(const std::array<Vec3f, 3>& t) const {
        int n = int(inside(t[0])) + int(inside(t[1])) + int(inside(t[2]));
        if (n == 3) return TriangleSelector::RegionCoverage::Full;
        if (n > 0) return TriangleSelector::RegionCoverage::Partial;
        double lo_x = std::min(std::min(double(t[0].x()), double(t[1].x())), double(t[2].x()));
        double hi_x = std::max(std::max(double(t[0].x()), double(t[1].x())), double(t[2].x()));
        double lo_y = std::min(std::min(double(t[0].y()), double(t[1].y())), double(t[2].y()));
        double hi_y = std::max(std::max(double(t[0].y()), double(t[1].y())), double(t[2].y()));
        if (kind == 1) return lo_x < k ? TriangleSelector::RegionCoverage::Partial : TriangleSelector::RegionCoverage::None;
        double r = std::sqrt(r2);
        double a = cx - r, b = cx + r, c = cy - r, d = cy + r;
        if (hi_x < a || lo_x > b || hi_y < c || lo_y > d) return TriangleSelector::RegionCoverage::None;
        return TriangleSelector::RegionCoverage::Partial;
    }
};

static Region rnd_region() {
    Region g;
    g.kind = below(4) == 0 ? 1 : 0;
    g.cx = range(-1, 5);
    g.cy = range(-1, 4);
    double r = range(0.2, 2.5);
    g.r2 = r * r;
    g.k = range(-1, 5);
    return g;
}

static void pregion(const Region& g) { pi(g.kind); pd(g.cx); pd(g.cy); pd(g.r2); pd(g.k); }
static void pbands(const Bands& b) { pd(b.band); pd(b.shift); pi(b.nstates); pi(b.state_offset); pi(b.quirk); }

static std::vector<int> rnd_facets(int n, int keep_of_3) {
    std::vector<int> out;
    for (int f = 0; f < n; ++f) if ((int) below(3) < keep_of_3) out.push_back(f);
    // shuffle
    for (std::size_t i = out.size(); i > 1; --i) std::swap(out[i - 1], out[below(i)]);
    return out;
}

static void pints(const std::vector<int>& v) { pu(v.size()); for (int x : v) pi(x); }

// Everything the selector answers about its current tree.
static void queries(TriangleSelector& sel, const indexed_triangle_set& its, int max_state) {
    tag("state"); arrow(); pselector(sel); nl();
    // the facet sets of every state by their sizes, those of one state in full
    for (int s = 0; s <= max_state; ++s) {
        indexed_triangle_set a = sel.get_facets(EnforcerBlockerType(s)), b = sel.get_facets_strict(EnforcerBlockerType(s));
        tag("facets"); pi(s); arrow(); pi(sel.num_facets(EnforcerBlockerType(s))); pi(sel.has_facets(EnforcerBlockerType(s)));
        pi(TriangleSelector::has_facets(sel.serialize(), EnforcerBlockerType(s)));
        pu(a.vertices.size()); pu(a.indices.size()); pu(b.vertices.size()); pu(b.indices.size()); nl();
    }
    {
        const int s = max_state > 0 ? 1 + (int) below(max_state) : 0;
        tag("facets_full"); pi(s); arrow(); pmesh(sel.get_facets_strict(EnforcerBlockerType(s))); nl();
    }
    {
        std::vector<indexed_triangle_set> per_type;
        sel.get_facets(per_type);
        tag("facets_per_type"); arrow(); pu(per_type.size());
        std::size_t first = 0;
        for (std::size_t s = 0; s < per_type.size(); ++s)
            if (!per_type[s].indices.empty()) {
                pu(s); pu(per_type[s].vertices.size()); pu(per_type[s].indices.size());
                if (first == 0 && s > 0) first = s;
            }
        pu(first); if (first > 0) pmesh(per_type[first]);
        nl();
    }
    // neighbor_child / triangle_midpoint / triangle_subtriangles over the triangles' own edges
    // and over the edges the original faces share with their neighbours.
    std::vector<int> valid;
    for (int t = 0; t < (int) sel.m_triangles.size(); ++t) if (sel.m_triangles[t].valid()) valid.push_back(t);
    for (int k = 0; k < 25 && !valid.empty(); ++k) {
        int t = valid[below(valid.size())];
        const auto& tr = sel.m_triangles[t];
        tag("edges"); pi(t); arrow();
        for (const Vec3f& v : sel.facet_vertices(t)) p3f(v);
        for (int e = 0; e < 3; ++e) {
            int vi = tr.verts_idxs[e], vj = tr.verts_idxs[(e + 1) % 3];
            auto st = sel.triangle_subtriangles(t, vi, vj);
            pi(vi); pi(vj);
            pi(sel.neighbor_child(t, vi, vj, TriangleSelector::Partition::First));
            pi(sel.neighbor_child(t, vi, vj, TriangleSelector::Partition::Second));
            pi(sel.triangle_midpoint(t, vi, vj)); pi(st.first); pi(st.second);
        }
        nl();
    }
    for (int f = 0; f < sel.m_orig_size_indices; ++f) {
        if (below(3) != 0) continue;
        const auto& tr = sel.m_triangles[f];
        tag("neighbor_edges"); pi(f); arrow();
        for (int e = 0; e < 3; ++e) {
            int n = sel.m_neighbors[f](e);
            int vi = tr.verts_idxs[(e + 1) % 3], vj = tr.verts_idxs[e];
            auto st = sel.triangle_subtriangles(n, vi, vj);
            pi(n); pi(vi); pi(vj);
            pi(sel.neighbor_child(n, vi, vj, TriangleSelector::Partition::First));
            pi(sel.neighbor_child(n, vi, vj, TriangleSelector::Partition::Second));
            pi(sel.triangle_midpoint(n, vi, vj)); pi(st.first); pi(st.second);
        }
        nl();
    }
    // select_unsplit_triangle at points inside original faces
    for (int k = 0; k < 10; ++k) {
        int f = (int) below(its.indices.size());
        double u = unit(), v = unit();
        if (u + v > 1.) { u = 1. - u; v = 1. - v; }
        const Vec3f &a = its.vertices[its.indices[f][0]], &b = its.vertices[its.indices[f][1]], &c = its.vertices[its.indices[f][2]];
        Vec3f hit = (a.cast<double>() + u * (b - a).cast<double>() + v * (c - a).cast<double>()).cast<float>();
        tag("select_unsplit_triangle"); pi(f); p3f(hit); arrow();
        pi(sel.select_unsplit_triangle(hit, f)); pi(sel.select_unsplit_triangle(hit, f, sel.m_neighbors[f])); nl();
    }
}

// ---------------------------------------------------------------------------------------------

static void gen_subdivide() {
    for (int i = 0; i < 40; ++i) {
        indexed_triangle_set its = rnd_mesh();
        TriangleMesh mesh(its);
        TriangleSelector sel(mesh);
        tag("mesh"); pmesh(its); nl();
        const int rounds = 1 + (int) below(2);
        int max_state = 0;
        for (int round = 0; round < rounds; ++round) {
            Bands b = rnd_bands();
            max_state = std::max(max_state, b.nstates - 1 + b.state_offset);
            std::vector<int> faces = rnd_facets((int) its.indices.size(), 2);
            if (below(8) == 0) faces.push_back((int) its.indices.size() + 3); // out of range: skipped
            if (below(8) == 0) faces.push_back(-1);
            std::size_t budget = below(3) == 0 ? 8 + below(200) : (below(4) == 0 ? 40000 : 4000);
            double rel = below(2) ? 1e-3 : 0.05;
            int depth = (int) below(budget >= 40000 ? 9 : 6);
            int cancel_at = below(3) == 0 ? 1 + (int) below(3) : 0;
            if (i % 8 == 7 && round == 0) {
                // long enough for the cancel poll (every 1024 iterations) to be reached
                budget = 40000; rel = 1e-12; depth = 9; cancel_at = 1 + (int) below(2);
            }
            int calls = 0, cancels = 0;
            auto eval = [&](int root, const std::array<Vec3f, 3>& t) { ++calls; return b(root, t); };
            TriangleSelector::FacetSubdivisionCancelCallback cancel = nullptr;
            if (cancel_at > 0) cancel = [&]() { return ++cancels >= cancel_at; };
            tag("set_facets_with_subdivision"); pbands(b); pints(faces); pu(budget); pd(rel); pd(1e-8); pi(depth); pi(cancel_at); arrow();
            auto r = sel.set_facets_with_subdivision(faces, eval, budget, rel, 1e-8, depth, cancel);
            pd(r.surface_area); pd(r.error_area); pu(r.nodes_created); pi(r.node_budget_exhausted); pi(r.depth_limit_reached); pi(r.canceled);
            pi(calls); pi(cancels); pselector(sel); nl();
        }
        queries(sel, its, max_state);
        // fill_unpainted / replace_state / shift_states_above, each on a copy of the tree
        const Data data = sel.serialize();
        for (int k = 0; k < 3; ++k) {
            TriangleSelector s2(mesh);
            s2.deserialize(data, true);
            int a = (int) below(max_state + 2), c = (int) below(40);
            if (k == 0) { s2.fill_unpainted(EnforcerBlockerType(a)); tag("fill_unpainted"); }
            if (k == 1) { s2.replace_state(EnforcerBlockerType(a), EnforcerBlockerType(c)); tag("replace_state"); }
            if (k == 2) { c = (int) below(5) - 2; s2.shift_states_above(EnforcerBlockerType(a), c); tag("shift_states_above"); }
            pi(a); pi(c); arrow(); pselector(s2); nl();
        }
    }
}

static void gen_region() {
    for (int i = 0; i < 50; ++i) {
        indexed_triangle_set its = rnd_mesh();
        TriangleMesh mesh(its);
        TriangleSelector sel(mesh);
        tag("mesh"); pmesh(its); nl();
        int max_state = 0;
        if (below(2)) {
            Bands b = rnd_bands();
            max_state = b.nstates - 1 + b.state_offset;
            std::vector<int> faces = rnd_facets((int) its.indices.size(), 2);
            int calls = 0;
            auto eval = [&](int root, const std::array<Vec3f, 3>& t) { ++calls; return b(root, t); };
            tag("set_facets_with_subdivision"); pbands(b); pints(faces); pu(4000); pd(1e-3); pd(1e-8); pi(3); pi(0); arrow();
            auto r = sel.set_facets_with_subdivision(faces, eval, 4000, 1e-3, 1e-8, 3);
            pd(r.surface_area); pd(r.error_area); pu(r.nodes_created); pi(r.node_budget_exhausted); pi(r.depth_limit_reached); pi(r.canceled);
            pi(calls); pi(0); pselector(sel); nl();
        }
        const int strokes = 1 + (int) below(3);
        for (int k = 0; k < strokes; ++k) {
            Region g = rnd_region();
            std::vector<int> start = rnd_facets((int) its.indices.size(), below(3) == 0 ? 3 : 1);
            if (below(6) == 0) start.push_back(-2);
            const int state = (int) below(3) == 0 ? 1 + (int) below(40) : 1 + (int) below(4);
            max_state = std::max(max_state, state);
            float edge = (float) range(0.05, 0.6);
            std::size_t budget = below(3) == 0 ? below(120) : 200000;
            int depth = below(4) == 0 ? (int) below(4) : 20;
            int cancel_at = below(4) == 0 ? 1 + (int) below(4) : 0;
            if (i % 6 == 5 && k == 0) {
                // long enough for the cancel poll (every 256 nodes) to be reached
                edge = 0.03f; budget = 200000; depth = 20; cancel_at = 1 + (int) below(3);
            }
            std::vector<char> mask;
            const bool use_mask = below(3) == 0;
            if (use_mask) { mask.resize(its.indices.size()); for (auto& m : mask) m = below(3) != 0; }
            const int trafo_kind = below(2) ? 0 : 1 + (int) below(3);
            Transform3f trafo = rnd_trafo(trafo_kind).cast<float>();
            const float before = (float) range(0.1, 1.0);
            sel.set_edge_limit(before);
            int calls = 0, cancels = 0;
            auto cov = [&](const std::array<Vec3f, 3>& t) { ++calls; return g(t); };
            TriangleSelector::FacetSubdivisionCancelCallback cancel = nullptr;
            if (cancel_at > 0) cancel = [&]() { return ++cancels >= cancel_at; };
            tag("paint_region"); pregion(g); pints(start); pi(state); pf(edge); pu(budget); pi(depth); pi(cancel_at);
            pi(use_mask); if (use_mask) { pu(mask.size()); for (char m : mask) pi(m); }
            pi(trafo_kind != 0); if (trafo_kind != 0) ptrafof(trafo);
            pf(before); arrow();
            auto r = sel.paint_region(start, cov, EnforcerBlockerType(state), edge, budget, depth, cancel, use_mask ? &mask : nullptr,
                                      trafo_kind != 0 ? &trafo : nullptr);
            pu(r.nodes_created); pu(r.facets_touched); pi(r.node_budget_exhausted); pi(r.canceled); pi(calls); pi(cancels);
            indexed_triangle_set strict = sel.get_facets_strict(EnforcerBlockerType(state));
            pf(sel.m_edge_limit_sqr); pselector(sel); pu(strict.vertices.size()); pu(strict.indices.size()); nl();
        }
        {
            // an empty coverage callback, and a selector of an empty mesh, paint nothing
            TriangleSelector::RegionCoverageEvaluator none = nullptr;
            auto r = sel.paint_region({0}, none, EnforcerBlockerType(1), 0.2f, 100, 5);
            tag("paint_region_empty"); arrow(); pu(r.nodes_created); pu(r.facets_touched); pi(r.node_budget_exhausted); pi(r.canceled); pselector(sel); nl();
        }
        queries(sel, its, max_state);
    }
}

// A random serialized tree on a root, by the grammar serialize() writes.
static void rnd_tree(std::vector<bool>& bits, int depth, int max_state_bits) {
    const bool split = depth < 3 && below(3) == 0;
    if (split) {
        int sides = 1 + (int) below(3);
        int special = sides == 3 ? 0 : (int) below(3);
        bits.push_back(sides & 1); bits.push_back(sides & 2);
        bits.push_back(special & 1); bits.push_back(special & 2);
        for (int c = 0; c <= sides; ++c) rnd_tree(bits, depth + 1, max_state_bits);
    } else {
        int n = below(3) == 0 ? (int) below(3) : (int) below(max_state_bits);
        bits.push_back(false); bits.push_back(false);
        if (n >= 3) {
            bits.push_back(true); bits.push_back(true);
            n -= 3;
            while (n >= 15) { for (int b = 0; b < 4; ++b) bits.push_back(true); n -= 15; }
            for (int b = 0; b < 4; ++b) bits.push_back((n >> b) & 1);
        } else {
            bits.push_back(n & 1); bits.push_back(n & 2);
        }
    }
}

static void gen_stream() {
    // read_paint_nibble and clamp_paint_state on their own
    for (int i = 0; i < 200; ++i) {
        std::vector<bool> bits(below(12));
        for (std::size_t k = 0; k < bits.size(); ++k) bits[k] = below(2);
        int ibit = (int) below(16) - 3, nibble = -7;
        tag("read_paint_nibble"); pu(bits.size()); std::printf(" "); if (bits.empty()) std::printf("-"); for (bool b : bits) std::printf("%d", (int) b);
        pi(ibit); arrow();
        bool ok = read_paint_nibble(bits, ibit, nibble);
        pi(ok); pi(ibit); pi(nibble); nl();
    }
    for (int i = 0; i < 100; ++i) {
        int s = (int) below(300) - 100, m = (int) below(127);
        tag("clamp_paint_state"); pi(s); pi(m); arrow(); pi((int) clamp_paint_state(s, EnforcerBlockerType(m))); nl();
    }
    // deserialize / has_facets over random and spoilt streams
    for (int i = 0; i < 150; ++i) {
        indexed_triangle_set its = rnd_mesh();
        TriangleMesh mesh(its);
        tag("mesh"); pmesh(its); nl();
        Data d;
        const int max_state_bits = below(3) == 0 ? 300 : (below(2) ? 120 : 40);
        for (int f = 0; f < (int) its.indices.size(); ++f) {
            if (below(2)) continue;
            d.first.emplace_back(f, (int) d.second.size());
            rnd_tree(d.second, 0, max_state_bits);
        }
        const int spoil = (int) below(6);
        if (spoil == 1 && !d.second.empty()) d.second.resize(below(d.second.size()));        // truncated
        if (spoil == 2 && !d.first.empty()) d.first[below(d.first.size())].second = (int) d.second.size() + (int) below(9) - 2; // past the end
        if (spoil == 3 && !d.first.empty()) d.first[below(d.first.size())].second = -1 - (int) below(3);
        if (spoil == 4 && !d.first.empty()) d.first.back().first = (int) its.indices.size() + (int) below(3); // no such root
        // 32 is left out: the Rust port reads EnforcerBlockerType::EXTRUDER_MAX (32) as the default ExtruderMax
        int max_ebt = below(3) == 0 ? 5 + (int) below(60) : (int) EnforcerBlockerType::ExtruderMax;
        if (max_ebt == 32) max_ebt = 31;
        const int to_delete = below(3) == 0 ? (int) below(8) : 0;
        const int replace = below(2) ? (int) below(5) : 0;
        TriangleSelector sel(mesh);
        tag("deserialize"); pdata(d); pi(max_ebt); pi(to_delete); pi(replace); arrow();
        sel.deserialize(d, true, EnforcerBlockerType(max_ebt), EnforcerBlockerType(to_delete), EnforcerBlockerType(replace));
        pselector(sel);
        for (int s : {0, 1, 2, 3, 17, 18, 33, 96, 100, 120}) pi(TriangleSelector::has_facets(d, EnforcerBlockerType(s)));
        nl();
        if (spoil == 0) {
            // a second stream onto the same selector without a reset
            // (onto roots the first stream left unsplit: a split root would be split again, which
            // breaks Triangle::set_division()'s and set_state()'s asserts)
            Data d2;
            for (int f = 0; f < (int) its.indices.size(); ++f) {
                if (below(4) != 0 || sel.m_triangles[f].is_split()) continue;
                d2.first.emplace_back(f, (int) d2.second.size());
                rnd_tree(d2.second, 0, 40);
            }
            tag("deserialize_more"); pdata(d2); arrow();
            sel.deserialize(d2, false);
            pselector(sel); nl();
        }
    }
}

// ---------------------------------------------------------------------------------------------
// Brush strokes, seed fill and bucket fill.
// ---------------------------------------------------------------------------------------------

static Vec3f point_on(const indexed_triangle_set& its, int f) {
    double u = range(0.05, 0.9), v = range(0.05, 0.9);
    if (u + v > 0.95) { u = 0.95 - u * 0.5; v = 0.95 - u - 0.01; if (v < 0.01) v = 0.01; }
    const Vec3f &a = its.vertices[its.indices[f][0]], &b = its.vertices[its.indices[f][1]], &c = its.vertices[its.indices[f][2]];
    return (a.cast<double>() + u * (b - a).cast<double>() + v * (c - a).cast<double>()).cast<float>();
}

static Vec3f rnd_dir() {
    Vec3d d(range(-1, 1), range(-1, 1), range(-1, 1));
    if (d.norm() < 0.1) d = Vec3d(0, 0, 1);
    return d.normalized().cast<float>();
}

static void pclp(const TriangleSelector::ClippingPlane& c) { p3f(c.normal); pf(c.offset); }

static TriangleSelector::ClippingPlane rnd_clp() {
    if (below(3) != 0) return TriangleSelector::ClippingPlane();
    Vec3f n = rnd_dir();
    return TriangleSelector::ClippingPlane({n.x(), n.y(), n.z(), (float) range(-1, 1)});
}

static void gen_cursor() {
    for (int i = 0; i < 40; ++i) {
        indexed_triangle_set its = rnd_mesh();
        TriangleMesh mesh(its);
        TriangleSelector sel(mesh);
        tag("mesh"); pmesh(its); nl();
        // The identity or a non-uniform scale: the cursor's uniform-scaling test reads the scaling
        // factor off Eigen's JacobiSVD (Transformation::get_scaling_factor()), whose singular
        // vectors of a uniform scale are arbitrary, so a rotated uniform scale is left out.
        const int trafo_kind = below(2) ? 0 : 2;
        Transform3d trafo = rnd_trafo(trafo_kind);
        Transform3d trafo_no_translate = trafo;
        trafo_no_translate.translation() = Vec3d::Zero();
        int max_state = 3;
        const int strokes = 1 + (int) below(4);
        for (int k = 0; k < strokes; ++k) {
            const int kind = (int) below(5); // sphere, circle, capsule3d, capsule2d, height range
            const int f = (int) below(its.indices.size());
            const int f2 = (int) below(its.indices.size());
            Vec3f c1 = point_on(its, f), c2 = point_on(its, f2);
            if ((c2 - c1).norm() < 1e-3f) c2 = c1 + Vec3f(0.3f, 0.1f, 0.f);
            Vec3f camera = c1 + rnd_dir() * (float) range(3, 20);
            const float radius = (float) range(0.2, 2.0);
            TriangleSelector::ClippingPlane clp = rnd_clp();
            const int state = 1 + (int) below(6);
            max_state = std::max(max_state, state);
            const bool splitting = below(4) != 0;
            const float highlight = below(4) == 0 ? (float) range(20, 80) : 0.f;
            const float z_world = (float) range(-3, 3), height = (float) range(0.2, 1.5);
            std::unique_ptr<TriangleSelector::Cursor> cursor;
            if (kind <= 1)
                cursor = TriangleSelector::SinglePointCursor::cursor_factory(c1, camera, radius, kind == 0 ? TriangleSelector::SPHERE : TriangleSelector::CIRCLE, trafo, clp);
            else if (kind <= 3)
                cursor = TriangleSelector::DoublePointCursor::cursor_factory(c1, c2, camera, radius, kind == 2 ? TriangleSelector::SPHERE : TriangleSelector::CIRCLE, trafo, clp);
            else
                cursor = TriangleSelector::SinglePointCursor::cursor_factory(z_world, camera, height, trafo, clp);
            tag("select_patch"); pi(kind); pi(f); p3f(c1); p3f(c2); p3f(camera); pf(radius); pf(z_world); pf(height); pclp(clp); pi(state); pi(splitting); pf(highlight);
            ptrafo(trafo); arrow();
            sel.select_patch(f, std::move(cursor), EnforcerBlockerType(state), trafo_no_translate, splitting, highlight);
            pf(sel.m_edge_limit_sqr); pf(sel.m_old_cursor_radius_sqr); pselector(sel); nl();
        }
        // seed fill and bucket fill from a point of a random face
        for (int k = 0; k < 2; ++k) {
            const int f = (int) below(its.indices.size());
            Vec3f hit = point_on(its, f);
            TriangleSelector::ClippingPlane clp = rnd_clp();
            const float angle = below(4) == 0 ? -1.f : (float) range(5, 90);
            const float highlight = below(4) == 0 ? (float) range(20, 80) : 0.f;
            const bool bucket = below(2);
            const bool propagate = below(4) != 0;
            const bool force = below(3) == 0;
            const int state = 1 + (int) below(6);
            if (bucket && sel.select_unsplit_triangle(hit, f) == -1) {
                // bucket_fill_select_triangles() asserts that the hit is found; leave such a hit out
                tag("bucket_fill_skipped"); arrow(); nl();
                continue;
            }
            if (bucket) {
                tag("bucket_fill"); pi(f); p3f(hit); pclp(clp); pf(angle); pi(propagate); pi(force); pi(state); arrow();
                sel.bucket_fill_select_triangles(hit, f, clp, angle, propagate, force);
            } else {
                tag("seed_fill"); pi(f); p3f(hit); pclp(clp); pf(angle < 0.f ? 30.f : angle); pf(highlight); pi(force); pi(state); ptrafo(trafo_no_translate); arrow();
                sel.seed_fill_select_triangles(hit, f, trafo_no_translate, clp, angle < 0.f ? 30.f : angle, highlight, force);
            }
            auto contour = sel.get_seed_fill_contour();
            pu(contour.size()); for (auto& e : contour) { pi(e.x()); pi(e.y()); }
            int fill_state = -1;
            // the mesh's vertices are all of m_vertices: its indices and the vertex count stand for it
            indexed_triangle_set fill = sel.get_seed_fill_mesh(fill_state);
            pu(fill.vertices.size()); pu(fill.indices.size()); for (auto& t : fill.indices) std::printf(" %d %d %d", t[0], t[1], t[2]);
            pi(fill_state);
            sel.seed_fill_apply_on_triangles(EnforcerBlockerType(state));
            pselector(sel); nl();
            max_state = std::max(max_state, state);
        }
        queries(sel, its, max_state);
    }
}

int main(int argc, char** argv) {
    if (argc != 2) return 1;
    std::string what = argv[1];
    if (what == "subdivide") gen_subdivide();
    else if (what == "region") { g_state ^= 0x2222; gen_region(); }
    else if (what == "stream") { g_state ^= 0x3333; gen_stream(); }
    else if (what == "cursor") { g_state ^= 0x4444; gen_cursor(); }
    else return 1;
    return 0;
}
