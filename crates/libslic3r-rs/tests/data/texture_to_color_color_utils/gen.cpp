// Reference vectors of tests/texture_to_color_color_utils_native_parity.rs: Bambu Studio's
// TextureToColor/ColorUtils.cpp at the pin, called with deterministic random inputs.
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
//   for part in color difference cluster mesh mesh_cluster smooth; do
//     ./gen $part > $DATA/vectors_$part.txt
//   done
//
// ColorUtils.cpp is included unchanged, so its static functions are called directly. The meshes go
// through CGAL::Surface_mesh as in the application.
//
// Format: one call per line: a tag, the arguments, `->`, the result. A double is its 16 hex
// digits, a float its 8 hex digits, everything else decimal. A mesh is `nv` then 3*nv floats, `nf`
// then 3*nf indices. A line `mesh` gives the mesh the lines after it are called with.

#include <fstream>
#include <map>
#include <cstdint>
#include <cstring>
#include "libslic3r/TextureToColor/ColorUtils.cpp"

namespace Slic3r { namespace log_detail { bool log_enabled(LogLevel) { return false; } } }

using namespace Slic3r;
using namespace Slic3r::tex2color;
using namespace Slic3r::tex2color::color_utils;

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

static RGB rnd_rgb() { return RGB{below(256), below(256), below(256)}; }

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

static void gen_color() {
    std::vector<RGB> rgbs = {{0, 0, 0}, {255, 255, 255}, {255, 0, 0}, {0, 255, 0}, {0, 0, 255}, {255, 255, 0}, {0, 255, 255}, {255, 0, 255},
                             {128, 128, 128}, {1, 1, 1}, {10, 10, 11}, {255, 254, 255}, {11, 10, 10}, {10, 11, 10}, {200, 100, 50}, {50, 100, 200}};
    for (int i = 0; i < 134; ++i) rgbs.push_back(rnd_rgb());
    for (auto& c : rgbs) { tag("rgb2hsv"); p3u(c); arrow(); p3d(convert_rgb_to_hsv(c)); nl(); }
    for (int i = 0; i < 200; ++i) {
        HSV h{range(0, 360), unit(), unit()};
        if (i < 14) h[0] = 30.0 * i;
        if (i == 14) h = {0, 0, 0};
        if (i == 15) h = {359.999, 1, 1};
        tag("hsv2rgb"); p3d(h); arrow(); p3u(convert_hsv_to_rgb(h)); nl();
    }
    for (auto& c : rgbs) { tag("rgb2xyz"); p3u(c); arrow(); p3d(convert_rgb_to_xyz(c)); nl(); }
    for (int i = 0; i < 200; ++i) {
        double r = range(-0.2, 1.2), g = range(-0.2, 1.2), b = range(-0.2, 1.2);
        if (i < 20) { r = range(0, 0.06); g = range(0, 0.06); b = 0.04045; }
        tag("srgb2xyz"); pd(r); pd(g); pd(b); arrow(); p3d(convert_srgb01_to_xyz(r, g, b)); nl();
        ColorDouble c{r, g, b};
        tag("srgb2lab"); p3d(c); arrow(); p3d(convert_srgb01_to_lab(c)); nl();
    }
    for (int i = 0; i < 200; ++i) {
        XYZ x{range(0, 1.1), range(0, 1.1), range(0, 1.1)};
        if (i < 30) x = {range(0, 0.02), range(0, 0.02), range(0, 0.02)};
        tag("xyz2lab"); p3d(x); arrow(); p3d(convert_xyz_to_lab(x)); nl();
    }
    for (auto& c : rgbs) { tag("rgb2lab"); p3u(c); arrow(); p3d(convert_rgb_to_lab(c)); nl(); }
    for (int i = 0; i < 300; ++i) {
        LAB l{range(0, 100), range(-128, 128), range(-128, 128)};
        if (i < 40) l = {range(0, 12), range(-10, 10), range(-10, 10)};
        if (i >= 40 && i < 80) l = {range(-20, 140), range(-200, 200), range(-200, 200)};
        tag("lab2rgb"); p3d(l); arrow(); p3u(convert_lab_to_rgb(l)); nl();
    }
    for (auto& c : rgbs) { LAB l = convert_rgb_to_lab(c); tag("lab2rgb"); p3d(l); arrow(); p3u(convert_lab_to_rgb(l)); nl(); }
}

static void gen_difference() {
    std::vector<RGB> rgbs = {{0, 0, 0}, {255, 255, 255}, {255, 0, 0}, {0, 255, 0}, {0, 0, 255}, {128, 128, 128}, {10, 10, 11}, {200, 100, 50}};
    for (int i = 0; i < 60; ++i) rgbs.push_back(rnd_rgb());
    for (int i = 0; i < 600; ++i) {
        LAB a{range(0, 100), range(-128, 128), range(-128, 128)};
        LAB b{range(0, 100), range(-128, 128), range(-128, 128)};
        if (i < 50) b = a;
        else if (i < 100) { a[1] = 0; a[2] = 0; }
        else if (i < 150) { a[1] = 0; a[2] = 0; b[1] = 0; b[2] = 0; }
        else if (i < 250) { b = {a[0] + range(-1, 1), a[1] + range(-1, 1), a[2] + range(-1, 1)}; }
        else if (i < 300) { b = {a[0], -a[1], -a[2]}; }
        else if (i < 350) { a[1] = range(-2, 2); a[2] = range(-2, 2); b[1] = range(-2, 2); b[2] = range(-2, 2); }
        tag("ciede2000"); p3d(a); p3d(b); arrow(); pd(ciede2000(a, b)); nl();
    }
    for (int i = 0; i < 300; ++i) {
        RGB a = rgbs[below(rgbs.size())], b = (i % 3 == 0) ? rgbs[below(rgbs.size())] : rnd_rgb();
        tag("de_rgb"); p3u(a); p3u(b); arrow(); pd(calc_rgb_color_difference_by_ciede2000(a, b)); nl();
        tag("sq_rgb"); p3u(a); p3u(b); arrow(); pd(calc_rgb_color_difference_by_squared_rgb(a, b)); nl();
    }
    for (int i = 0; i < 200; ++i) {
        ColorDouble a{range(-0.1, 1.1), range(-0.1, 1.1), range(-0.1, 1.1)}, b{range(-0.1, 1.1), range(-0.1, 1.1), range(-0.1, 1.1)};
        tag("de_srgb01"); p3d(a); p3d(b); arrow(); pd(calc_rgb_color_difference_by_ciede2000_srgb01(a, b)); nl();
        ColorDouble c{range(0, 255), range(0, 255), range(0, 255)}, d{range(0, 255), range(0, 255), range(0, 255)};
        tag("sq_double"); p3d(c); p3d(d); arrow(); pd(calc_rgb_color_difference_by_squared_rgb_double(c, d)); nl();
    }
    for (int i = 0; i < 200; ++i) {
        double s = (i < 30) ? 1e-3 : (i < 60 ? 1e-4 : 5.0);
        CGALKernel::Vector_3 a(range(-s, s), range(-s, s), range(-s, s)), b(range(-s, s), range(-s, s), range(-s, s));
        if (i >= 60 && i < 90) b = a * range(-3, 3);
        if (i >= 90 && i < 120) b = CGALKernel::Vector_3(a.x() + range(-1e-3, 1e-3), a.y() + range(-1e-3, 1e-3), a.z() + range(-1e-3, 1e-3));
        if (i >= 120 && i < 150) b = CGALKernel::Vector_3(-a.x() + range(-1e-3, 1e-3), -a.y() + range(-1e-3, 1e-3), -a.z() + range(-1e-3, 1e-3));
        CGALKernel::Vector_3 n = a;
        normalize(n);
        tag("normalize"); pd(a.x()); pd(a.y()); pd(a.z()); arrow(); pd(n.x()); pd(n.y()); pd(n.z()); nl();
        tag("angle"); pd(a.x()); pd(a.y()); pd(a.z()); pd(b.x()); pd(b.y()); pd(b.z()); arrow(); pd(get_angle_between_vectors(a, b)); nl();
    }
}

// A list of colors: `spread` of them random, the rest near one of those.
static std::vector<Color> rnd_colors(std::size_t n, std::size_t spread, std::size_t jitter) {
    std::vector<Color> seeds;
    for (std::size_t i = 0; i < spread; ++i) seeds.push_back(rnd_rgb());
    std::vector<Color> out;
    for (std::size_t i = 0; i < n; ++i) {
        Color c = seeds[below(seeds.size())];
        if (jitter > 0)
            for (int k = 0; k < 3; ++k) {
                long v = (long) c[k] + (long) below(2 * jitter + 1) - (long) jitter;
                c[k] = (std::size_t) std::min(255l, std::max(0l, v));
            }
        out.push_back(c);
    }
    return out;
}

static void pcolors(const std::vector<Color>& v) { pu(v.size()); for (auto& c : v) p3u(c); }
static void pdcolors(const std::vector<ColorDouble>& v) { pu(v.size()); for (auto& c : v) p3d(c); }

static void gen_cluster() {
    for (int i = 0; i < 60; ++i) {
        const bool lab = i % 2 == 0;
        std::size_t n = 2 + below(40);
        std::vector<Color> colors = rnd_colors(n, 1 + below(6), below(3) == 0 ? 0 : below(40));
        std::sort(colors.begin(), colors.end());
        colors.erase(std::unique(colors.begin(), colors.end()), colors.end());
        std::vector<ColorDouble> working;
        for (auto& c : colors) working.push_back(lab ? convert_rgb_to_lab(c) : ColorDouble{(double) c[0], (double) c[1], (double) c[2]});
        std::size_t k = 1 + below(working.size());
        WorkingDistFunc f = lab ? ciede2000 : calc_rgb_color_difference_by_squared_rgb_double;
        tag("fps"); pu(lab); pu(k); pdcolors(working); arrow(); pdcolors(farthest_point_sampling_init(working, k, f)); nl();
        std::size_t max_iter = below(4) == 0 ? below(3) : 50;
        tag("kmeans"); pu(lab); pu(k); pu(max_iter); pdcolors(working); arrow(); pdcolors(kmeans_core(working, k, max_iter, f)); nl();
    }
    for (int i = 0; i < 80; ++i) {
        ClusterParameters p;
        p.color_difference_method = i % 2 == 0 ? ColorDifferenceMethod::Lab : ColorDifferenceMethod::RGB;
        p.cluster_k = below(12);
        p.max_iter = below(5) == 0 ? below(4) : 50;
        std::size_t n = i == 0 ? 0 : 1 + below(120);
        std::vector<Color> colors = rnd_colors(n, 1 + below(8), below(4) == 0 ? 0 : below(50));
        tag("cluster_k_means"); pu((std::size_t) p.color_difference_method); pu(p.cluster_k); pu(p.max_iter); pcolors(colors); arrow();
        pcolors(cluster_k_means(colors, p)); nl();
    }
    for (int i = 0; i < 40; ++i) {
        std::vector<Color> colors = rnd_colors(below(60), 1 + below(8), below(50));
        std::vector<Color> specified = rnd_colors(1 + below(8), 1 + below(8), 0);
        if (i % 5 == 0 && specified.size() > 1) specified[1] = specified[0];
        tag("cluster_to_specified_colors"); pcolors(colors); pcolors(specified); arrow(); pcolors(cluster_to_specified_colors(colors, specified)); nl();
    }
    for (int i = 0; i < 36; ++i) {
        ClusterParameters p;
        p.color_difference_method = i % 3 == 2 ? ColorDifferenceMethod::RGB : ColorDifferenceMethod::Lab;
        p.max_color_distance = p.color_difference_method == ColorDifferenceMethod::RGB ? range(50, 6000) : range(1, 40);
        p.max_cluster_k = i % 7 == 6 ? 1 + below(6) : (i % 7 == 5 ? 64 : 32);
        p.max_iter = below(6) == 0 ? below(4) : 50;
        std::size_t n = i < 3 ? below(40) : 40 + below(160);
        std::vector<Color> colors = rnd_colors(n, 2 + below(12), 5 + below(60));
        // the cancel callback: true from its `cancel_after`-th call on; 0 is no callback
        std::size_t cancel_after = i % 6 == 5 ? 1 + below(12) : 0;
        std::size_t calls = 0;
        if (cancel_after > 0) p.cancel_callback = [&calls, cancel_after]() { return ++calls >= cancel_after; };
        tag("cluster_adaptive"); pu((std::size_t) p.color_difference_method); pd(p.max_color_distance); pu(p.max_cluster_k); pu(p.max_iter); pu(cancel_after);
        pcolors(colors); arrow();
        pcolors(cluster_adaptive(colors, p)); pu(calls); nl();
    }
    for (int i = 0; i < 60; ++i) {
        std::vector<RGB> colors = rnd_colors(i == 0 ? 0 : 1 + below(20), 1 + below(8), below(30));
        RGB c = rnd_rgb();
        std::size_t id = 12345;
        bool ok = calc_nearest_color_id(colors, c, id);
        tag("calc_nearest_color_id"); pcolors(colors); p3u(c); arrow(); pu(ok); pu(id); nl();
    }
}

static void plabels(const std::vector<std::size_t>& v) { pu(v.size()); for (auto x : v) pu(x); }

// Labels in patches: a few seed faces, every face takes the label of the seed nearest to it, and
// some faces a random one.
static std::vector<std::size_t> rnd_labels(const TriMesh& m, std::size_t nlabels, std::size_t noise_one_in) {
    std::vector<std::size_t> out(m.indices.size(), 0);
    if (m.indices.empty()) return out;
    std::vector<Vec3f> seeds;
    for (std::size_t i = 0; i < nlabels; ++i) seeds.push_back(m.vertices[m.indices[below(m.indices.size())][0]]);
    for (std::size_t f = 0; f < m.indices.size(); ++f) {
        Vec3f c = m.vertices[m.indices[f][0]];
        float best = 1e30f;
        for (std::size_t i = 0; i < nlabels; ++i) {
            float d = (seeds[i] - c).squaredNorm();
            if (d < best) { best = d; out[f] = i; }
        }
        if (noise_one_in > 0 && below(noise_one_in) == 0) out[f] = below(nlabels);
    }
    return out;
}

static void gen_mesh() {
    for (int i = 0; i < 30; ++i) {
        TriMesh m = pick_mesh(i);
        cgalutils::CGALMesh cm;
        cgalutils::convert_trimesh_to_cgal(m, cm);
        tag("mesh"); pmesh(m); nl();
        tag("average_edge_length_impl"); arrow(); pd(detail::average_edge_length_impl(cm)); nl();
        std::vector<Eigen::Vector3d> normals;
        calc_face_normals(cm, normals);
        tag("calc_face_normals"); arrow(); pu(normals.size());
        for (auto& n : normals) { pd(n.x()); pd(n.y()); pd(n.z()); }
        nl();
        tag("is_closed"); arrow(); pu(is_closed(m)); nl();
        auto groups = get_connected_face_groups(cm);
        tag("get_connected_face_groups"); arrow(); pu(groups.size());
        for (auto& g : groups) { pu(g.size()); for (auto f : g) pu((std::size_t) f); }
        nl();
        std::vector<Vec2f> uvs;
        std::size_t nuv = (i % 9 == 8) ? m.vertices.size() + 1 : m.vertices.size();
        for (std::size_t k = 0; k < nuv; ++k) uvs.push_back(Vec2f((float) unit(), (float) unit()));
        std::vector<TriMesh> cms;
        std::vector<std::vector<Vec2f>> cuvs;
        cms.push_back(m);  // cleared by the call
        bool ok = get_components(m, uvs, cms, cuvs);
        tag("get_components"); pu(uvs.size()); for (auto& uv : uvs) { pf(uv.x()); pf(uv.y()); }
        arrow(); pu(ok); pu(cms.size());
        for (std::size_t c = 0; c < cms.size(); ++c) { pmesh(cms[c]); pu(cuvs[c].size()); for (auto& uv : cuvs[c]) { pf(uv.x()); pf(uv.y()); } }
        nl();
    }
    for (int i = 0; i < 50; ++i) {
        TriMesh m = pick_mesh(i);
        std::vector<std::size_t> labels = rnd_labels(m, 2 + below(4), 1 + below(6));
        if (i % 13 == 12) labels.push_back(0);
        std::size_t max_iters = i % 4 == 3 ? below(3) : 20;
        cgalutils::CGALMesh cm;
        cgalutils::convert_trimesh_to_cgal(m, cm);
        if (cm.number_of_faces() != labels.size()) continue;
        tag("mesh"); pmesh(m); nl();
        tag("smooth_region_topo_boundary"); plabels(labels); pu(max_iters); arrow();
        bool ok = smooth_region_topo_boundary(cm, labels, max_iters);
        pu(ok); plabels(labels); nl();
    }
    for (int i = 0; i < 6; ++i) {
        std::vector<std::vector<CGALKernel::Point_3>> polylines;
        std::size_t n = below(5);
        for (std::size_t k = 0; k < n; ++k) {
            std::vector<CGALKernel::Point_3> pl;
            std::size_t np = below(6);
            for (std::size_t q = 0; q < np; ++q) {
                double s = below(4) == 0 ? 1e-7 : (below(4) == 0 ? 1e7 : 10.0);
                pl.emplace_back(below(5) == 0 ? (double) below(2000000) : range(-s, s), below(7) == 0 ? 0.0 : range(-s, s), range(-s, s) * 1e-3);
            }
            polylines.push_back(pl);
        }
        save_polylines("gen_polylines.obj", polylines);
        std::ifstream in("gen_polylines.obj", std::ios::binary);
        std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
        tag("save_polylines"); pu(polylines.size());
        for (auto& pl : polylines) { pu(pl.size()); for (auto& p : pl) { pd(p.x()); pd(p.y()); pd(p.z()); } }
        arrow(); pu(text.size());
        for (unsigned char ch : text) std::printf(" %u", (unsigned) ch);
        nl();
        std::remove("gen_polylines.obj");
    }
}

static void gen_mesh_cluster() {
    for (int i = 0; i < 90; ++i) {
        TriMesh m = pick_mesh(i % 2 == 0 ? i / 2 : (int) below(5));
        std::size_t nc = i == 0 ? 0 : (i == 1 ? 1 : 2 + below(5));
        std::vector<RGB> centers = rnd_colors(nc, nc, 0);
        if (i % 6 == 5 && nc > 2) centers[2] = centers[0];
        // face colors: mixtures of two centers, so that many faces are near no center
        std::vector<RGB> rgbs;
        std::size_t nf = m.indices.size() + (i % 17 == 16 ? 1 : 0);
        std::vector<std::size_t> patch = rnd_labels(m, std::max<std::size_t>(nc, 1), 3);
        patch.resize(nf, 0);
        for (std::size_t f = 0; f < nf; ++f) {
            if (nc == 0) { rgbs.push_back(rnd_rgb()); continue; }
            RGB a = centers[patch[f] % nc], b = centers[below(nc)];
            double t = below(3) == 0 ? 0.0 : range(0.0, 0.7);
            RGB c;
            for (int k = 0; k < 3; ++k) {
                long v = (long) std::lround((1 - t) * (double) a[k] + t * (double) b[k]) + (long) below(31) - 15;
                c[k] = (std::size_t) std::min(255l, std::max(0l, v));
            }
            if (below(8) == 0) c = rnd_rgb();
            rgbs.push_back(c);
        }
        std::vector<std::size_t> ids;
        if (i % 10 == 9) ids = {0, 1};  // resize() keeps what is there
        tag("mesh"); pmesh(m); nl();
        tag("mesh_cluster"); pcolors(centers); pcolors(rgbs); plabels(ids); arrow();
        bool ok = mesh_cluster(m, centers, rgbs, ids);
        pu(ok); pcolors(rgbs); plabels(ids); nl();
    }
}

static void gen_smooth() {
    for (int i = 0; i < 48; ++i) {
        static const int kinds[] = {0, 1, 2, 3, 4, 6, 7, 5};
        TriMesh m = pick_mesh(kinds[i % 8]);
        SmoothParameters sp;
        static const double weights[] = {0.5, 0.0, 0.2, 0.3, 1.0, 0.75, 1e-7, 1.5};
        sp.smooth_weight = weights[(i / 8) % 8];
        if (i >= 40) sp.smooth_weight = range(0.0, 1.0);
        if (i % 2 == 0) {
            std::vector<std::size_t> labels = rnd_labels(m, 2 + below(4), i % 3 == 0 ? 0 : 2 + below(8));
            if (i % 22 == 20) labels.push_back(1);
            tag("mesh"); pmesh(m); nl();
        tag("smooth_region_labels"); plabels(labels); pd(sp.smooth_weight); arrow();
            bool ok = smooth_region(m, labels, sp);
            pu(ok); pmesh(m); plabels(labels); nl();
        } else {
            std::vector<std::size_t> labels = rnd_labels(m, 2 + below(4), i % 3 == 0 ? 0 : 2 + below(8));
            std::vector<RGB> palette = rnd_colors(6, 6, 0);
            std::vector<std::array<std::size_t, 3>> colors;
            for (auto l : labels) colors.push_back(palette[l]);
            tag("mesh"); pmesh(m); nl();
        tag("smooth_region_colors"); pcolors(colors); pd(sp.smooth_weight); arrow();
            bool ok = smooth_region(m, colors, sp);
            pu(ok); pmesh(m); pcolors(colors); nl();
        }
    }
}

int main(int argc, char** argv) {
    if (argc != 2) return 1;
    std::string what = argv[1];
    if (what == "color") gen_color();
    else if (what == "difference") { g_state ^= 0x4444; gen_difference(); }
    else if (what == "cluster") { g_state ^= 0x1111; gen_cluster(); }
    else if (what == "mesh_cluster") { g_state ^= 0x5555; gen_mesh_cluster(); }
    else if (what == "mesh") { g_state ^= 0x2222; gen_mesh(); }
    else if (what == "smooth") { g_state ^= 0x3333; gen_smooth(); }
    else return 1;
    return 0;
}
