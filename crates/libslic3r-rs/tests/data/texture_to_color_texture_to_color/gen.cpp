// Reference vectors of tests/texture_to_color_texture_to_color_native_parity.rs: Bambu Studio's
// TextureToColor/TextureToColor.cpp at the pin, called with deterministic random inputs.
//
// Build, with REPO = this repository, PIN = $REPO/references/BambuStudio, BUILD = a built tree of
// the Bambu Studio engine (its generated headers and its static libraries), EIGEN = the Eigen
// include directory (`pkg-config --cflags eigen3`), PREFIX = where Boost, CGAL, TBB, GMP, MPFR and
// OpenCV are installed (`brew --prefix` on macOS) and DATA = this directory:
//
//   clang++ -std=c++17 -O2 -ffp-contract=on -DNDEBUG -w \
//     -include $REPO/libslic3r/bambustudio/cgal_54_compat.hpp \
//     -I$PIN/src -I$PIN/src/libslic3r -I$REPO/libslic3r/bambustudio/libigl -I$PIN/src/libigl \
//     -I$PIN/src/admesh -I$BUILD -I$BUILD/src/libslic3r -I$EIGEN -I$PREFIX/include \
//     -I$PREFIX/include/opencv4 \
//     $DATA/gen.cpp $PIN/src/libslic3r/TextureToColor/ColorUtils.cpp \
//     $PIN/src/libslic3r/MeshDiagnostics.cpp $PIN/src/libslic3r/MeshRaycast.cpp -o gen \
//     -L$PREFIX/lib -lopencv_core -ltbb -lgmp -lmpfr
//   for part in sampling split pipeline repair; do
//     ./gen $part > $DATA/vectors_$part.txt
//   done
//
// TextureToColor.cpp is included unchanged, so its static functions are called directly;
// ColorUtils.cpp, MeshDiagnostics.cpp and MeshRaycast.cpp are the pin's; the logging is off (Log.cpp,
// Boost.Log, is not linked; see the definitions below). Of TriangleMesh.cpp the unit needs one
// constructor and, of the statistics it fills, the open and the non-manifold edges and vertices:
// that constructor is defined below with those lines of `fill_initial_stats`
// (TriangleMesh.cpp:46-59), so that the whole engine does not have to be linked. TBB runs on one
// thread, which makes the order of the progress reports the order of the faces. The meshes are
// halfedge-compatible, so `RepairMesh` (CGAL) is not reached.
//
// Format: one call per line: a tag, the arguments, `->`, the result. A double is its 16 hex
// digits, a float its 8 hex digits, everything else decimal. A mesh is `nv` then 3*nv floats, `nf`
// then 3*nf indices. A texture is rows, columns, channels, the bytes of a row and its bytes in
// hex. Progress reports are a count and then `percent~message` each, blanks as `_`.

#include <fstream>
#include <map>
#include <cstdint>
#include <cstring>
#include <tbb/global_control.h>
#include "libslic3r/TextureToColor/TextureToColor.cpp"
#include "libslic3r/MeshDiagnostics.hpp"

// The logging is off: Log.cpp (Boost.Log) is not linked, and ColorUtils.cpp, a separate
// translation unit, still refers to the record type, which is never made.
namespace Slic3r { namespace log_detail {
bool log_enabled(LogLevel) { return false; }
void write_failed(std::ostream&) noexcept {}
LogStream::LogStream(LogLevel level, const char*, int, const char*) : m_level(level) {}
LogStream::~LogStream() {}
LogStream& LogStream::operator<<(std::ostream& (*manip)(std::ostream&)) { manip(m_buf); return *this; }
} }

using namespace Slic3r;
using namespace Slic3r::tex2color;
using namespace Slic3r::tex2color::color_utils;
using cgalutils::CGALMesh;

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

namespace Slic3r {
// TriangleMesh.cpp:89-92 with the lines of fill_initial_stats (TriangleMesh.cpp:46-59) that fill
// what TextureToColor.cpp reads.
TriangleMesh::TriangleMesh(const indexed_triangle_set& its) : its(its) {
    std::vector<Vec3i> face_neighbors;
    const auto nm_stats = its_quick_diagnostics(this->its, &face_neighbors);
    m_stats.open_edges = static_cast<int>(nm_stats.open_edges);
    m_stats.non_manifold_edges = static_cast<int>(nm_stats.non_manifold_edges);
    m_stats.non_manifold_vertices = static_cast<int>(nm_stats.non_manifold_vertices);
    m_stats.has_reversed_faces = nm_stats.has_reversed_faces;
}
}  // namespace Slic3r

static void pcolors(const std::vector<RGB>& v) { pu(v.size()); for (auto& c : v) p3u(c); }
static void plabels(const std::vector<std::size_t>& v) { pu(v.size()); for (auto x : v) pu(x); }
static void puv(const Vec2f& uv) { pf(uv.x()); pf(uv.y()); }

static cv::Mat rnd_texture(int rows, int cols, int ch, int smooth) {
    cv::Mat t(rows, cols, CV_8UC(ch));
    for (int r = 0; r < rows; ++r)
        for (int c = 0; c < cols; ++c)
            for (int k = 0; k < ch; ++k) {
                int v;
                if (smooth == 0) v = (int) below(256);
                else if (smooth == 1) v = ((r * 4 / rows + c * 4 / cols + k) % 4) * 80 + (int) below(16);  // patches
                else v = (r * 255 / std::max(rows - 1, 1) + c * 37 * (k + 1)) % 256;
                t.data[r * t.step[0] + c * ch + k] = (uchar) v;
            }
    return t;
}

static void ptex(const cv::Mat& t) {
    std::printf(" %d %d %d %zu ", t.rows, t.cols, t.channels(), (std::size_t) t.step[0]);
    std::size_t n = (std::size_t) t.rows * t.step[0];
    if (n == 0) std::printf("-");
    for (std::size_t i = 0; i < n; ++i) std::printf("%02x", t.data[i]);
}

struct Events {
    std::vector<std::pair<int, std::string>> seen;
    AlgoProgressCallback callback() { return [this](AlgoProgress p) { seen.emplace_back(p.percent, p.message ? p.message : "(null)"); }; }
    void print() const {
        pu(seen.size());
        for (auto& e : seen) {
            std::string m = e.second;
            for (auto& ch : m) if (ch == ' ') ch = '_';
            std::printf(" %d~%s", e.first, m.c_str());
        }
    }
};

// The cancel callback: true from its `after`-th call on; 0 is no callback.
struct Cancel {
    std::size_t after = 0, calls = 0;
    AlgoCancelCallback callback() { if (after == 0) return nullptr; return [this]() { return ++calls >= after; }; }
};

static std::vector<RGB> rnd_colors(std::size_t n, std::size_t spread, std::size_t jitter) {
    std::vector<RGB> seeds;
    for (std::size_t i = 0; i < spread; ++i) seeds.push_back(RGB{below(256), below(256), below(256)});
    std::vector<RGB> out;
    for (std::size_t i = 0; i < n; ++i) {
        RGB c = seeds[below(seeds.size())];
        if (jitter > 0)
            for (int k = 0; k < 3; ++k) {
                long v = (long) c[k] + (long) below(2 * jitter + 1) - (long) jitter;
                c[k] = (std::size_t) std::min(255l, std::max(0l, v));
            }
        out.push_back(c);
    }
    return out;
}

// Face colors in patches over the mesh.
static std::vector<RGB> patch_colors(const TriMesh& m, std::size_t npatch, std::size_t jitter) {
    std::vector<RGB> palette = rnd_colors(npatch, npatch, 0);
    std::vector<Vec3f> seeds;
    for (std::size_t i = 0; i < npatch; ++i) seeds.push_back(m.vertices[m.indices[below(m.indices.size())][0]]);
    std::vector<RGB> out;
    for (auto& f : m.indices) {
        Vec3f c = m.vertices[f[0]];
        float best = 1e30f;
        std::size_t id = 0;
        for (std::size_t i = 0; i < npatch; ++i) {
            float d = (seeds[i] - c).squaredNorm();
            if (d < best) { best = d; id = i; }
        }
        RGB col = palette[id];
        for (int k = 0; k < 3; ++k) {
            long v = (long) col[k] + (long) below(2 * jitter + 1) - (long) jitter;
            col[k] = (std::size_t) std::min(255l, std::max(0l, v));
        }
        out.push_back(col);
    }
    return out;
}

static TriMesh good_mesh(int kind) {
    for (;;) {
        static const int kinds[] = {0, 1, 2, 3, 4, 6};
        TriMesh m = pick_mesh(kinds[kind % 6]);
        if (!m.indices.empty() && cgalutils::is_mesh_halfedge_compatible(m)) return m;
    }
}

static void gen_sampling() {
    for (int i = 0; i < 24; ++i) {
        int ch = i % 3 == 0 ? 4 : 3;
        cv::Mat t = rnd_texture(1 + (int) below(9), 1 + (int) below(9), ch, i % 3);
        tag("texture"); ptex(t); nl();
        for (int k = 0; k < 40; ++k) {
            float u = (float) range(-1.5, 2.5), v = (float) range(-1.5, 2.5);
            if (k % 8 == 0) { u = (float) below(3) - 1.0f; v = (float) below(3) - 1.0f; }
            if (k % 8 == 1) { u = 0.99999994f; v = 0.5f; }
            RGB c = get_pixel_color(u, v, t);
            tag("get_pixel_color"); pf(u); pf(v); arrow(); p3u(c); nl();
        }
        for (int k = 0; k < 20; ++k) {
            double s = k % 2 == 0 ? 1.0 : 0.1;
            Vec2f o((float) unit(), (float) unit());
            FaceUVArray uvs = {o + Vec2f((float) range(-s, s), (float) range(-s, s)), o + Vec2f((float) range(-s, s), (float) range(-s, s)),
                               o + Vec2f((float) range(-s, s), (float) range(-s, s))};
            RGB c = sample_face_color(uvs, t);
            tag("sample_face_color"); puv(uvs[0]); puv(uvs[1]); puv(uvs[2]); arrow(); p3u(c); nl();
        }
    }
    for (int i = 0; i < 60; ++i) {
        std::size_t k = 1 + below(7), n = below(40);
        std::vector<std::size_t> labels;
        for (std::size_t f = 0; f < n; ++f) labels.push_back(below(k + (i % 4 == 0 ? 2 : 0)) / (below(3) == 0 ? 2 : 1));
        auto usage = count_cluster_label_usage(labels, k);
        tag("count_cluster_label_usage"); plabels(labels); pu(k); arrow(); plabels(usage); nl();
        std::vector<RGB> centers = rnd_colors(i % 9 == 8 ? 0 : k, std::max<std::size_t>(k, 1), 0);
        tag("discard_unused_cluster_centers"); pcolors(centers); plabels(labels); arrow();
        std::vector<std::size_t> l2 = labels;
        bool ok = discard_unused_cluster_centers(centers, l2, "test");
        pu(ok); pcolors(centers); plabels(l2); nl();
        std::vector<RGB> centers2 = rnd_colors(i % 9 == 7 ? 0 : k, std::max<std::size_t>(k, 1), 0);
        std::vector<RGB> source = rnd_colors(i % 11 == 10 ? n + 1 : n, 4, 30);
        std::vector<std::size_t> l3 = labels;
        tag("ensure_all_cluster_centers_used"); pcolors(source); pcolors(centers2); plabels(l3); arrow();
        ok = ensure_all_cluster_centers_used(source, centers2, l3, "test");
        pu(ok); plabels(l3); nl();
    }
    for (int i = 0; i < 8; ++i) {
        MeshRepairCache c;
        c.has_prepared_mesh = i % 4 != 3;
        c.key_input_face_count = 10; c.key_oversampling_iters = 2; c.key_oversampling_min_face_count = 100; c.key_oversampling_max_face_count = 1000;
        std::size_t a = i == 4 ? 11 : 10, b = i == 5 ? 3 : 2, d = i == 6 ? 101 : 100, e = i == 7 ? 1001 : 1000;
        tag("matches_input"); pu(c.has_prepared_mesh); pu(a); pu(b); pu(d); pu(e); arrow(); pu(c.matches_input(a, b, d, e)); nl();
    }
    for (int i = 0; i < 6; ++i) {
        TriMesh m = good_mesh(i);
        std::vector<RGB> colors = rnd_colors(i % 2 == 0 ? m.indices.size() : m.indices.size() / 2, 5, 40);
        for (auto& v : m.vertices) if (below(6) == 0) v = Vec3f((float) below(5), 1e-7f * (float) below(9), 123456.789f * (float) below(3));
        SaveToOFF("gen_off_dir/sub/gen.off", m, colors);
        std::ifstream in("gen_off_dir/sub/gen.off", std::ios::binary);
        std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
        in.close();
        tag("mesh"); pmesh(m); nl();
        tag("SaveToOFF"); pcolors(colors); arrow(); pu(text.size());
        for (unsigned char ch : text) std::printf(" %u", (unsigned) ch);
        nl();
        std::filesystem::remove_all("gen_off_dir");
    }
}

static void puvs(const std::vector<FaceUVArray>& uvs) { pu(uvs.size()); for (auto& f : uvs) { puv(f[0]); puv(f[1]); puv(f[2]); } }

static void gen_split() {
    for (int i = 0; i < 14; ++i) {
        TriMesh m = i % 7 == 6 ? soup_mesh(5 + (int) below(5), 3 + (int) below(8)) : good_mesh(i);
        std::vector<FaceUVArray> uvs;
        for (std::size_t f = 0; f < m.indices.size(); ++f)
            uvs.push_back({Vec2f((float) unit(), (float) unit()), Vec2f((float) unit(), (float) unit()), Vec2f((float) unit(), (float) unit())});
        tag("mesh"); pmesh(m); nl();
        std::vector<int> seen;
        std::function<void(int)> cb = [&](int p) { seen.push_back(p); };
        tag("linear_subdivision"); puvs(uvs); pu(i % 3 != 0); arrow();
        bool ok = linear_subdivision(m, uvs, i % 3 != 0 ? cb : std::function<void(int)>());
        pu(ok); pmesh(m); puvs(uvs); pu(seen.size()); for (int p : seen) pu((std::size_t) p); nl();
    }
    for (int i = 0; i < 40; ++i) {
        TextureToColorSettings s;
        s.target_colors_num = i % 3 == 0 ? 0 : 1 + below(6);
        s.max_color_distance = range(2, 30);
        s.max_cluster_k = i % 6 == 3 ? 3 : 32;
        std::vector<RGB> colors = rnd_colors(i == 0 ? 0 : 1 + below(i % 3 == 0 ? 150 : 40), 2 + below(8), below(40));
        Cancel cancel;
        cancel.after = i % 8 == 7 ? 1 + below(4) : 0;
        std::vector<RGB> centers = {{1, 2, 3}};
        std::vector<std::size_t> ids = {9};
        tag("quantize_colors"); pu(s.target_colors_num); pd(s.max_color_distance); pu(s.max_cluster_k); pu(cancel.after); pcolors(colors); arrow();
        bool ok = quantize_colors(colors, s, cancel.callback(), centers, ids);
        pu(ok); pcolors(centers); plabels(ids); pu(cancel.calls); nl();
    }
    for (int i = 0; i < 20; ++i) {
        TextureToColorSettings s;
        s.target_colors_num = i % 3 == 0 ? 0 : 1 + below(5);
        std::vector<VertexColor> vc;
        std::size_t n = i == 0 ? 0 : 1 + below(60);
        std::vector<RGB> seeds = rnd_colors(4, 4, 0);
        for (std::size_t k = 0; k < n; ++k) {
            RGB c = seeds[below(4)];
            vc.push_back({(float) (c[0] / 255.0 + range(-0.05, 0.05)), (float) (c[1] / 255.0 + range(-0.05, 0.05)), (float) (c[2] / 255.0 + range(-0.05, 0.05)),
                          (float) unit()});
        }
        if (n > 2) { vc[0] = {-0.5f, 1.5f, 1.0f, 1.0f}; vc[1] = {0.999999f, 0.0f, 0.00392157f, 0.0f}; }
        std::vector<RGB> centers;
        std::vector<std::size_t> ids;
        tag("quantize_vertex_colors"); pu(s.target_colors_num); pu(vc.size()); for (auto& c : vc) { pf(c[0]); pf(c[1]); pf(c[2]); pf(c[3]); } arrow();
        bool ok = quantize_vertex_colors(vc, s, nullptr, centers, ids);
        pu(ok); pcolors(centers); plabels(ids); nl();
    }
    for (int i = 0; i < 40; ++i) {
        TriMesh m = i % 8 == 7 ? soup_mesh(5 + (int) below(5), 3 + (int) below(8)) : good_mesh(i);
        if (i == 38) m = TriMesh();
        std::size_t k = i == 37 ? 0 : 1 + below(5);
        std::vector<RGB> centers = rnd_colors(k, std::max<std::size_t>(k, 1), 0);
        std::vector<std::size_t> ids;
        std::size_t n = i == 36 ? m.vertices.size() + 1 : m.vertices.size();
        // clusters in bands along x, so that neighbours mostly agree; some at random
        for (std::size_t v = 0; v < n; ++v) {
            std::size_t id = k == 0 ? 0 : (v < m.vertices.size() ? (std::size_t) std::fabs(m.vertices[v].x() * 0.7f) % k : 0);
            if (k > 0 && below(5) == 0) id = below(k);
            ids.push_back(id);
        }
        // some equilateral faces, where the longest edge is not one alone
        if (i % 5 == 4 && m.vertices.size() >= 3) { m.vertices[0] = Vec3f(0, 0, 0); m.vertices[1] = Vec3f(1, 0, 0); m.vertices[2] = Vec3f(0, 1, 0); }
        std::vector<RGB> out = {{4, 5, 6}};
        tag("mesh"); pmesh(m); nl();
        tag("adaptive_split_by_vertex_clusters"); plabels(ids); pcolors(centers); arrow();
        bool ok = adaptive_split_by_vertex_clusters(m, ids, centers, out);
        pu(ok); pmesh(m); pcolors(out); nl();
    }
}

static void psettings(const TextureToColorSettings& s) {
    pu(s.target_colors_num); pd(s.smooth_weight); pu(s.oversampling_iters); pu(s.oversampling_min_face_count); pu(s.oversampling_max_face_count);
    pd(s.max_color_distance); pu(s.max_cluster_k); pu(s.mesh_repair_decision == MeshRepairDecision::RepairAndImport);
}

static void pcache(const MeshRepairCache& c) {
    pu((std::size_t) c.win10_attempt); pu(c.has_prepared_mesh); pmesh(c.mesh); pcolors(c.face_colors);
    pu(c.key_input_face_count); pu(c.key_oversampling_iters); pu(c.key_oversampling_min_face_count); pu(c.key_oversampling_max_face_count);
}

static TextureToColorSettings rnd_settings(int i) {
    TextureToColorSettings s;
    s.target_colors_num = i % 3 == 0 ? 0 : 1 + below(5);
    static const double weights[] = {0.5, 0.0, 0.3, 1.0, 0.2, 0.8};
    s.smooth_weight = weights[i % 6];
    s.max_color_distance = i % 2 == 0 ? 25.0 : range(3, 30);
    s.max_cluster_k = i % 7 == 6 ? 4 : 32;
    return s;
}

static void gen_pipeline() {
    // repair_cluster_smooth, with the session cache every third time: the second call takes the prepared mesh
    for (int i = 0; i < 30; ++i) {
        TriMesh m = good_mesh(i);
        std::vector<RGB> colors = patch_colors(m, 2 + below(5), below(40));
        TextureToColorSettings s = rnd_settings(i);
        if (i % 5 == 4) {
            // a repair callback that fails, or is not asked: the mesh is taken as it is
            s.mesh_repair_decision = MeshRepairDecision::RepairAndImport;
        }
        MeshRepairCache cache;
        const bool with_cache = i % 3 == 2;
        if (with_cache) s.mesh_repair_cache = &cache;
        for (int round = 0; round < (with_cache ? 2 : 1); ++round) {
            Events ev;
            Cancel cancel;
            cancel.after = (i % 6 == 5 && round == 0) ? 1 + below(14) : 0;
            TriMesh mm = m;
            std::vector<RGB> cc = colors, out = {{7, 7, 7}};
            if (round == 1) { mm = good_mesh(i + 1); cc = patch_colors(mm, 2, 0); }  // replaced by what the cache holds
            std::size_t key = m.indices.size();
            tag("mesh"); pmesh(mm); nl();
            tag("repair_cluster_smooth"); pcolors(cc); psettings(s); pu(with_cache); pu(round); pu(cancel.after); pu(key); arrow();
            bool ok = repair_cluster_smooth(mm, cc, out, s, ev.callback(), cancel.callback(), "test", key);
            pu(ok); pmesh(mm); pcolors(cc); pcolors(out); ev.print(); pu(cancel.calls);
            if (with_cache) pcache(cache);
            nl();
        }
    }
    // TextureToColor
    for (int i = 0; i < 26; ++i) {
        TriMesh m = good_mesh(i);
        int ch = i % 4 == 0 ? 4 : 3;
        cv::Mat t = rnd_texture(4 + (int) below(13), 4 + (int) below(13), ch, 1 + i % 2);
        std::vector<std::vector<Vec2f>> uvs;
        for (auto& f : m.indices) {
            std::vector<Vec2f> fu;
            for (int k = 0; k < 3; ++k) {
                const Vec3f& p = m.vertices[f[k]];
                fu.push_back(Vec2f(p.x() * 0.11f + 0.05f * (float) unit(), p.y() * 0.13f + 0.05f * (float) unit()));
            }
            uvs.push_back(fu);
        }
        TextureToColorSettings s = rnd_settings(i);
        s.oversampling_iters = i % 3 == 1 ? 1 + below(2) : 0;
        s.oversampling_min_face_count = i % 3 == 2 ? 10 : 150 + below(200);
        s.oversampling_max_face_count = i % 9 == 4 ? m.indices.size() * 5 : 1000000;
        // inputs that are refused
        if (i == 20) m.indices.clear();
        if (i == 21) t = cv::Mat();
        if (i == 22) t = rnd_texture(4, 4, 1, 0);
        if (i == 23) uvs.pop_back();
        if (i == 24) uvs[0].pop_back();
        MeshRepairCache cache;
        const bool with_cache = i % 4 == 3;
        if (with_cache) s.mesh_repair_cache = &cache;
        tag("texture"); ptex(t); nl();
        for (int round = 0; round < (with_cache ? 2 : 1); ++round) {
            Events ev;
            Cancel cancel;
            cancel.after = (i % 5 == 4 && round == 0) ? 1 + below(20) : 0;
            TriMesh out_mesh = m;
            std::vector<RGB> out = {{7, 7, 7}};
            tag("mesh"); pmesh(m); nl();
            tag("TextureToColor"); pu(uvs.size()); for (auto& fu : uvs) { pu(fu.size()); for (auto& uv : fu) puv(uv); }
            psettings(s); pu(with_cache); pu(round); pu(cancel.after); arrow();
            bool ok = TextureToColor(m, uvs, t, out_mesh, out, s, ev.callback(), cancel.callback());
            pu(ok); pmesh(out_mesh); pcolors(out); ev.print(); pu(cancel.calls);
            if (with_cache) pcache(cache);
            nl();
        }
    }
    // ClusterAndSmooth
    for (int i = 0; i < 24; ++i) {
        TriMesh m = good_mesh(i);
        std::vector<RGB> colors = patch_colors(m, 2 + below(5), below(40));
        if (i % 6 == 5) colors.resize(colors.size() / 2);
        if (i == 22) colors.clear();
        TextureToColorSettings s = rnd_settings(i);
        // the vertex-color route for a mesh with fewer faces than oversampling_min_face_count
        s.oversampling_min_face_count = i % 4 == 3 ? 1 : 10000;
        std::vector<std::array<float, 4>> vc;
        if (i % 2 == 1) {
            std::size_t n = i % 8 == 5 ? m.vertices.size() - 1 : m.vertices.size();
            std::vector<RGB> seeds = rnd_colors(4, 4, 0);
            for (std::size_t k = 0; k < n; ++k) {
                RGB c = seeds[(std::size_t) std::fabs(m.vertices[k].x()) % 4];
                vc.push_back({(float) (c[0] / 255.0), (float) (c[1] / 255.0), (float) (c[2] / 255.0), 1.0f});
            }
        }
        Events ev;
        Cancel cancel;
        cancel.after = i % 7 == 6 ? 1 + below(6) : 0;
        TriMesh out_mesh;
        std::vector<RGB> out = {{7, 7, 7}};
        tag("mesh"); pmesh(m); nl();
        tag("ClusterAndSmooth"); pcolors(colors); psettings(s); pu(cancel.after); pu(vc.size()); for (auto& c : vc) { pf(c[0]); pf(c[1]); pf(c[2]); pf(c[3]); } arrow();
        bool ok = ClusterAndSmooth(m, colors, out_mesh, out, s, ev.callback(), cancel.callback(), vc);
        pu(ok); pmesh(out_mesh); pcolors(out); ev.print(); pu(cancel.calls); nl();
    }
}

// repair_cluster_smooth with a mesh repair callback (the Windows 3D repair of the application): the
// callback reports two messages, asks the cancel callback `cancel_queries` times and then fails
// (mode 2) or gives the repaired mesh printed on the line, which is the mesh with its vertices
// moved (mode 0) or the mesh subdivided once with its vertices moved (mode 1). The face colors are
// then looked up on the old faces by the centroids of the new ones.
static void gen_repair() {
    for (int i = 0; i < 36; ++i) {
        TriMesh m = good_mesh(i);
        std::vector<RGB> colors = patch_colors(m, 2 + below(5), below(40));
        TextureToColorSettings s = rnd_settings(i);
        s.mesh_repair_decision = i % 9 == 8 ? MeshRepairDecision::ImportWithoutRepair : MeshRepairDecision::RepairAndImport;
        const int mode = i % 3 == 2 && i % 4 == 0 ? 2 : (int) (i % 2);
        const int cancel_queries = (int) below(3);
        TriMesh repaired = m;
        if (mode == 1) {
            std::vector<FaceUVArray> uvs(repaired.indices.size(), FaceUVArray{Vec2f(0, 0), Vec2f(0, 0), Vec2f(0, 0)});
            linear_subdivision(repaired, uvs);
        }
        const double jitter = i % 5 == 0 ? 0.0 : 0.05;
        for (auto& v : repaired.vertices) v += Vec3f((float) range(-jitter, jitter), (float) range(-jitter, jitter), (float) range(-jitter, jitter));
        s.mesh_repair_callback = [&](const indexed_triangle_set& in, indexed_triangle_set& out, std::function<void(const char*, unsigned)> progress,
                                     std::function<bool()> cancel, std::string* error) -> bool {
            progress("Repairing on Windows", 10);
            progress(nullptr, 50);
            for (int q = 0; q < cancel_queries; ++q)
                if (cancel()) { *error = "cancelled"; return false; }
            if (mode == 2) { *error = "no repair"; return false; }
            out = repaired;
            return true;
        };
        MeshRepairCache cache;
        const bool with_cache = i % 3 != 1;
        if (with_cache) s.mesh_repair_cache = &cache;
        for (int round = 0; round < (with_cache ? 2 : 1); ++round) {
            Events ev;
            Cancel cancel;
            if (round == 0) cancel.after = i % 6 == 5 ? 4 + cancel_queries : (i % 7 == 3 ? 1 + below(6) : 0);
            TriMesh mm = m;
            std::vector<RGB> cc = colors, out = {{7, 7, 7}};
            std::size_t key = m.indices.size();
            tag("mesh"); pmesh(mm); nl();
            tag("repair_cluster_smooth_with_callback"); pcolors(cc); psettings(s); pu(with_cache); pu(round); pu(cancel.after); pu(key);
            pu((std::size_t) mode); pu((std::size_t) cancel_queries); pmesh(repaired); arrow();
            bool ok = repair_cluster_smooth(mm, cc, out, s, ev.callback(), cancel.callback(), "test", key);
            pu(ok); pmesh(mm); pcolors(cc); pcolors(out); ev.print(); pu(cancel.calls);
            if (with_cache) pcache(cache);
            nl();
        }
    }
}

int main(int argc, char** argv) {
    if (argc != 2) return 1;
    tbb::global_control one_thread(tbb::global_control::max_allowed_parallelism, 1);
    std::string what = argv[1];
    if (what == "sampling") gen_sampling();
    else if (what == "split") { g_state ^= 0x1111; gen_split(); }
    else if (what == "pipeline") { g_state ^= 0x2222; gen_pipeline(); }
    else if (what == "repair") { g_state ^= 0x3333; gen_repair(); }
    else return 1;
    return 0;
}
