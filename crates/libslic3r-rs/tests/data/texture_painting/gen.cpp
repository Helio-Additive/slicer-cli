// Reference vectors of tests/texture_painting_native_parity.rs: Bambu Studio's TexturePainting.cpp
// at the pin, called with deterministic random inputs.
//
// TexturePainting.cpp is included unchanged, so its static functions are called directly; the rest
// of the engine (TextureToColor, Model, TriangleSelector, the logging) comes from the engine's own
// static libraries, linked as the `slicer_cli` program links them. Build, with REPO = this
// repository, BUILD = a configured and built tree of the Bambu Studio engine (`cmake
// -DENGINE=bambu`, docs/building.md) and DATA = this directory:
//
//   compile gen.cpp with the compile flags, include directories and definitions of a libslic3r
//   source of $BUILD (its `compile_commands.json`, e.g. the entry of TexturePainting.cpp: -O3
//   -DNDEBUG -std=c++17 and `-include $REPO/libslic3r/bambustudio/cgal_54_compat.hpp`), then
//   link it with the link line of `slicer_cli` ($BUILD/CMakeFiles/slicer_cli.dir/link.txt) with
//   main.cpp's objects replaced by gen.o: $BUILD/libslic3r_core.a, $BUILD/liblibslic3r_cgal.a,
//   OpenCV (core, imgproc, imgcodecs), TBB, Boost, GMP, MPFR and the rest of that line.
//   for part in basic pipeline volume; do
//     ./gen $part > $DATA/vectors_$part.txt
//   done
//
// TBB runs on one thread, so the progress reports and the questions to the cancel callback come in
// the order of the faces. The meshes of the pipeline are halfedge-compatible, so `RepairMesh` (CGAL)
// is not reached. Encoded images (PNG / JPEG) are not among the inputs (the port has no decoder);
// bytes that do not decode are.
//
// Format: one call per line: a tag, the arguments, `->`, the result. A double is its 16 hex
// digits, a float its 8 hex digits, everything else decimal (an int that can be negative is
// printed as a decimal int). A textured mesh, a painted mesh and a texture are printed by
// `ptm`, `ppainted` and `ptex` below.

#include <fstream>
#include <map>
#include <cstdint>
#include <cstring>
#include <tbb/global_control.h>
#include "libslic3r/TexturePainting.cpp"

using namespace Slic3r;
using Rgb = std::array<std::size_t, 3>;

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
static void p3u(const Rgb& a) { pu(a[0]); pu(a[1]); pu(a[2]); }
static void tag(const char* t) { std::printf("%s", t); }
static void arrow() { std::printf(" ->"); }
static void nl() { std::printf("\n"); }
static void pcolors(const std::vector<Rgb>& v) { pu(v.size()); for (auto& c : v) p3u(c); }
static void pbytes(const unsigned char* p, std::size_t n) {
    std::printf(" ");
    if (n == 0) std::printf("-");
    for (std::size_t i = 0; i < n; ++i) std::printf("%02x", p[i]);
}

static void ptex(const cv::Mat& t) {
    std::printf(" %d %d %d %zu", t.rows, t.cols, t.channels(), (std::size_t) t.step[0]);
    pbytes(t.data, (std::size_t) t.rows * t.step[0]);
}

static void ptm(const TexturedMesh& m) {
    pu(m.vertices.size()); for (auto& v : m.vertices) { pf(v[0]); pf(v[1]); pf(v[2]); }
    pu(m.indices.size()); for (auto& f : m.indices) { pi(f[0]); pi(f[1]); pi(f[2]); }
    pu(m.uvs.size()); for (auto& u : m.uvs) { pf(u[0]); pf(u[1]); }
    pu(m.textures.size());
    for (auto& t : m.textures) { pi(t.width); pi(t.height); pi(t.channels); pu(t.data.size()); pbytes(t.data.data(), t.data.size()); }
    pu(m.material_ids.size()); for (int x : m.material_ids) pi(x);
    pu(m.material_texture_map.size()); for (int x : m.material_texture_map) pi(x);
    pu(m.material_colors.size()); for (auto& c : m.material_colors) { pf(c[0]); pf(c[1]); pf(c[2]); pf(c[3]); }
    pu(m.uv_coords.size()); for (auto& u : m.uv_coords) { pf(u[0]); pf(u[1]); }
    pu(m.uv_indices.size()); for (auto& f : m.uv_indices) { pi(f[0]); pi(f[1]); pi(f[2]); }
    pcolors(m.precomputed_face_colors);
    pu(m.precomputed_vertex_colors.size()); for (auto& c : m.precomputed_vertex_colors) { pf(c[0]); pf(c[1]); pf(c[2]); pf(c[3]); }
    pu(m.is_standard_3mf);
}

static void ppainted(const PaintedMesh& p) {
    pu(p.vertices.size()); for (auto& v : p.vertices) { pf(v[0]); pf(v[1]); pf(v[2]); }
    pu(p.indices.size()); for (auto& f : p.indices) { pi(f[0]); pi(f[1]); pi(f[2]); }
    pcolors(p.face_colors);
    pcolors(p.cluster_colors);
    pu(p.original_color_to_cluster.size());
    for (auto& kv : p.original_color_to_cluster) { p3u(kv.first); pi(kv.second); }
}

static void pmesh(const tex2color::TriMesh& m) {
    pu(m.vertices.size());
    for (auto& v : m.vertices) { pf(v.x()); pf(v.y()); pf(v.z()); }
    pu(m.indices.size());
    for (auto& f : m.indices) std::printf(" %d %d %d", f[0], f[1], f[2]);
}

static std::vector<Rgb> rnd_colors(std::size_t n, std::size_t spread, std::size_t jitter) {
    std::vector<Rgb> seeds;
    for (std::size_t i = 0; i < spread; ++i) seeds.push_back(Rgb{below(256), below(256), below(256)});
    std::vector<Rgb> out;
    for (std::size_t i = 0; i < n; ++i) {
        Rgb c = seeds[below(seeds.size())];
        if (jitter > 0)
            for (int k = 0; k < 3; ++k) {
                long v = (long) c[k] + (long) below(2 * jitter + 1) - (long) jitter;
                c[k] = (std::size_t) std::min(255l, std::max(0l, v));
            }
        out.push_back(c);
    }
    return out;
}

// An image of `w` x `h` pixels of `ch` channels, in patches when `smooth`.
static TextureImage rnd_image(int w, int h, int ch, bool smooth) {
    TextureImage img;
    img.width = w; img.height = h; img.channels = ch;
    for (int r = 0; r < h; ++r)
        for (int c = 0; c < w; ++c)
            for (int k = 0; k < ch; ++k)
                img.data.push_back((unsigned char) (smooth ? ((r * 3 / std::max(h, 1) + c * 3 / std::max(w, 1) + k) % 4) * 70 + below(12) : below(256)));
    return img;
}

// A grid of nx by ny cells in the plane z ~ 0 (halfedge-compatible), its vertices and faces, UVs
// from the position.
static void grid(TexturedMesh& m, int nx, int ny, double noise, float x0 = 0.f) {
    const int base = (int) m.vertices.size();
    for (int y = 0; y <= ny; ++y)
        for (int x = 0; x <= nx; ++x) {
            m.vertices.push_back({(float) (x0 + x + range(-0.2, 0.2)), (float) (y + range(-0.2, 0.2)), (float) range(-noise, noise)});
            m.uvs.push_back({(float) (x / (double) nx + range(-0.02, 0.02)), (float) (y / (double) ny + range(-0.02, 0.02))});
        }
    auto id = [&](int x, int y) { return base + y * (nx + 1) + x; };
    for (int y = 0; y < ny; ++y)
        for (int x = 0; x < nx; ++x) {
            int a = id(x, y), b = id(x + 1, y), c = id(x + 1, y + 1), d = id(x, y + 1);
            if (below(2)) { m.indices.push_back({a, b, c}); m.indices.push_back({a, c, d}); }
            else { m.indices.push_back({a, b, d}); m.indices.push_back({b, c, d}); }
        }
}

static TexturedMesh rnd_textured(int i, bool for_pipeline) {
    TexturedMesh m;
    grid(m, 2 + (int) below(for_pipeline ? 3 : 5), 2 + (int) below(for_pipeline ? 3 : 5), 0.4);
    if (i % 3 == 1) grid(m, 1 + (int) below(3), 1 + (int) below(3), 0.4, 20.f);
    const std::size_t nf = m.indices.size(), nv = m.vertices.size();
    if (!for_pipeline && i % 5 == 4) { m.indices[0][1] = -1; m.indices[nf - 1][2] = (int) nv + 3; }
    if (i % 4 == 1 || i % 4 == 2) {
        // per-face UVs from a pool, a few indices out of range
        for (std::size_t k = 0; k < nv + 4; ++k) m.uv_coords.push_back({(float) range(-1.5, 2.5), (float) range(-1.5, 2.5)});
        for (std::size_t f = 0; f < nf; ++f) {
            std::array<int, 3> u;
            for (int k = 0; k < 3; ++k) u[k] = below(15) == 0 ? (below(2) ? -1 : (int) m.uv_coords.size() + 2) : (int) below(m.uv_coords.size());
            m.uv_indices.push_back(u);
        }
    }
    if (i % 6 == 5) m.uvs.resize(nv / 2);
    const int ntex = i % 4 == 3 ? 0 : (i % 3 == 0 ? 1 : 2 + (int) below(2));
    for (int t = 0; t < ntex; ++t) {
        int ch = below(4) == 0 ? 4 : 3;
        m.textures.push_back(rnd_image(2 + (int) below(9), 2 + (int) below(9), ch, below(3) != 0));
    }
    if (ntex > 0 && i % 7 == 6) { m.textures.back().channels = 2; }
    if (ntex > 1 && i % 8 == 5) { m.textures[0].data.clear(); }
    if (ntex > 0 && i % 9 == 7) { m.textures.back().width = -1; }  // bytes that do not decode
    const int nmat = 1 + (int) below(4);
    if (i % 2 == 0 || ntex > 1) {
        for (int k = 0; k < nmat; ++k) m.material_texture_map.push_back(below(5) == 0 ? -1 : (int) below((std::size_t) std::max(ntex, 1) + 1) - (below(6) == 0 ? 1 : 0));
        for (int k = 0; k < nmat; ++k) if (below(4) != 0) m.material_colors.push_back({(float) unit(), (float) range(-0.2, 1.2), (float) unit(), 1.f});
        for (std::size_t f = 0; f < nf; ++f) if (below(9) != 0) m.material_ids.push_back((int) below((std::size_t) nmat + 1) - 1 + (below(10) == 0 ? nmat + 2 : 0));
    }
    return m;
}

struct Events {
    std::vector<std::pair<int, std::string>> seen;
    PaintProgressCallback callback() { return [this](int p, const char* m) { seen.emplace_back(p, m ? m : "(null)"); }; }
    void print() const {
        pu(seen.size());
        for (auto& e : seen) {
            std::string m = e.second;
            for (auto& ch : m) if (ch == ' ') ch = '_';
            std::printf(" %d~%s", e.first, m.c_str());
        }
    }
};

struct Cancel {
    std::size_t after = 0, calls = 0;
    PaintCancelCallback callback() { if (after == 0) return nullptr; return [this]() { return ++calls >= after; }; }
};

static void psettings(const TexturePaintingSettings& s) {
    pu(s.target_colors_num); pd(s.smooth_weight); pu(s.oversampling_iters);
    pu(s.mesh_repair_decision == TexturePaintingSettings::MeshRepairDecision::RepairAndImport);
}

static void pcache(const tex2color::MeshRepairCache& c) {
    pu((std::size_t) c.win10_attempt); pu(c.has_prepared_mesh); pmesh(c.mesh); pcolors(c.face_colors);
    pu(c.key_input_face_count); pu(c.key_oversampling_iters); pu(c.key_oversampling_min_face_count); pu(c.key_oversampling_max_face_count);
}

static void gen_basic() {
    // decode_texture_image / decode_texture_to_pixels
    for (int i = 0; i < 30; ++i) {
        TextureImage img = rnd_image(1 + (int) below(6), 1 + (int) below(6), 1 + (int) below(5), i % 2 == 0);
        if (i % 10 == 3) img.data.clear();
        if (i % 10 == 5) img.width = -1;
        if (i % 10 == 7) img.height = 0;
        tag("decode_texture_image"); pi(img.width); pi(img.height); pi(img.channels); pu(img.data.size()); pbytes(img.data.data(), img.data.size()); arrow();
        cv::Mat m = decode_texture_image(img);
        ptex(m);
        std::vector<unsigned char> px = {1, 2};
        int w = -5, h = -6;
        bool ok = decode_texture_to_pixels(img, px, w, h);
        pu(ok); pi(w); pi(h); pu(px.size()); pbytes(px.data(), px.size()); nl();
    }
    // build_tex2color_mesh, build_multi_texture_atlas, sample_original_face_colors
    for (int i = 0; i < 40; ++i) {
        TexturedMesh tm = rnd_textured(i, false);
        tag("textured"); ptm(tm); nl();
        {
            tex2color::TriMesh mesh;
            std::vector<std::vector<Vec2f>> uvs;
            if (i % 5 == 0) { mesh.vertices.push_back(Vec3f(9, 9, 9)); uvs.push_back({Vec2f(1, 1)}); }
            build_tex2color_mesh(tm, mesh, uvs);
            tag("build_tex2color_mesh"); arrow(); pmesh(mesh); pu(uvs.size());
            for (auto& f : uvs) { pu(f.size()); for (auto& uv : f) { pf(uv.x()); pf(uv.y()); } }
            nl();
        }
        {
            cv::Mat atlas;
            std::vector<std::vector<Vec2f>> uvs;
            bool ok = build_multi_texture_atlas(tm, atlas, uvs);
            tag("build_multi_texture_atlas"); arrow(); pu(ok); ptex(atlas); pu(uvs.size());
            for (auto& f : uvs) { pu(f.size()); for (auto& uv : f) { pf(uv.x()); pf(uv.y()); } }
            nl();
        }
        {
            std::vector<Rgb> out = {{1, 2, 3}};
            bool ok = sample_original_face_colors(tm, out);
            tag("sample_original_face_colors"); arrow(); pu(ok); pcolors(out); nl();
        }
    }
    // sample_face_from_uvs / sample_face_from_texture
    for (int i = 0; i < 12; ++i) {
        TextureImage img = rnd_image(1 + (int) below(8), 1 + (int) below(8), i % 3 == 0 ? 4 : 3, i % 2 == 0);
        cv::Mat tex = decode_texture_image(img);
        tag("texture"); ptex(tex); nl();
        for (int k = 0; k < 20; ++k) {
            std::array<float, 2> a = {(float) range(-2, 3), (float) range(-2, 3)}, b = {(float) range(-2, 3), (float) range(-2, 3)},
                                 c = {(float) range(-2, 3), (float) range(-2, 3)};
            if (k % 7 == 0) { a = {0.f, 0.f}; b = {1.f, 1.f}; c = {1.f, 0.f}; }
            tag("sample_face_from_uvs"); pf(a[0]); pf(a[1]); pf(b[0]); pf(b[1]); pf(c[0]); pf(c[1]); arrow(); p3u(sample_face_from_uvs(tex, a, b, c)); nl();
        }
        std::vector<std::array<float, 2>> uvs;
        for (int k = 0; k < 6; ++k) uvs.push_back({(float) range(-1, 2), (float) range(-1, 2)});
        for (int k = 0; k < 10; ++k) {
            std::array<int, 3> f = {(int) below(8) - 1, (int) below(8) - 1, (int) below(8) - 1};
            tag("sample_face_from_texture"); pu(uvs.size()); for (auto& u : uvs) { pf(u[0]); pf(u[1]); } pi(f[0]); pi(f[1]); pi(f[2]); arrow();
            p3u(sample_face_from_texture(tex, uvs, f)); nl();
        }
    }
    // extract_painted_mesh
    for (int i = 0; i < 10; ++i) {
        TexturedMesh tm = rnd_textured(i * 3, true);
        tex2color::TriMesh mesh;
        std::vector<std::vector<Vec2f>> uvs;
        build_tex2color_mesh(tm, mesh, uvs);
        std::vector<Rgb> colors = rnd_colors(i == 9 ? 0 : mesh.indices.size(), 1 + below(6), below(3));
        PaintedMesh painted;
        painted.original_color_to_cluster[{1, 2, 3}] = 4;
        painted.vertices.push_back({7, 7, 7});
        tag("extract_painted_mesh"); pmesh(mesh); pcolors(colors); arrow();
        extract_painted_mesh(mesh, colors, painted);
        ppainted(painted); nl();
    }
    // compute_delta_e, match_clusters_to_filaments
    for (int i = 0; i < 40; ++i) {
        Rgb a = {below(256), below(256), below(256)}, b = {below(256), below(256), below(256)};
        std::array<float, 4> f = {(float) unit(), (float) unit(), (float) unit(), 1.f};
        if (i % 10 == 9) f = {1.f, 0.99999994f, 0.5f, 0.f};
        tag("compute_delta_e"); p3u(a); p3u(b); pf(f[0]); pf(f[1]); pf(f[2]); pf(f[3]); arrow(); pd(compute_delta_e(a, f)); pd(compute_delta_e(a, b)); nl();
    }
    for (int i = 0; i < 20; ++i) {
        std::vector<Rgb> clusters = rnd_colors(below(7), 6, 0);
        std::vector<std::array<float, 4>> fils;
        std::size_t nfil = i % 7 == 3 ? 0 : 1 + below(6);
        for (std::size_t k = 0; k < nfil; ++k) fils.push_back({(float) unit(), (float) unit(), (float) unit(), (float) unit()});
        if (nfil > 2 && i % 3 == 0) fils[2] = fils[0];
        std::vector<std::string> names(nfil, "x");
        auto matches = match_clusters_to_filaments(clusters, fils, names);
        tag("match_clusters_to_filaments"); pcolors(clusters); pu(fils.size()); for (auto& c : fils) { pf(c[0]); pf(c[1]); pf(c[2]); pf(c[3]); } arrow();
        pu(matches.size());
        for (auto& m : matches) { pi(m.cluster_index); pi(m.filament_index); pd(m.delta_e); p3u(m.cluster_color); pf(m.filament_color[0]); pf(m.filament_color[1]); pf(m.filament_color[2]); pf(m.filament_color[3]); }
        nl();
    }
    // the cache helpers
    for (int i = 0; i < 24; ++i) {
        MeshRepairCachePtr cache = i % 8 == 7 ? MeshRepairCachePtr() : make_mesh_repair_cache();
        TexturePaintingSettings s;
        s.oversampling_iters = below(3);
        std::size_t faces = 10 + below(3);
        if (cache) {
            cache->has_prepared_mesh = i % 4 != 0;
            cache->win10_attempt = (tex2color::MeshRepairAttempt) below(3);
            cache->key_input_face_count = 10 + below(3);
            cache->key_oversampling_iters = below(3);
            cache->key_oversampling_min_face_count = i % 6 == 5 ? 5 : 10000;
            cache->key_oversampling_max_face_count = 1000000;
            if (i % 3 != 2) {
                cache->mesh.vertices = {Vec3f(0, 0, 0), Vec3f(1, 0, 0), Vec3f(0, 1, 0), Vec3f(1, 1, (float) unit())};
                cache->mesh.indices = {Vec3i(0, 1, 2), Vec3i(1, 3, 2)};
                cache->face_colors = rnd_colors(i % 5 == 4 ? 1 : 2, 2, 0);
            }
        }
        tag("cache"); pu((bool) cache); if (cache) pcache(*cache); pu(s.oversampling_iters); pu(faces); arrow();
        pu(mesh_repair_cache_win10_failed(cache));
        PaintedMesh out;
        out.vertices.push_back({5, 5, 5});
        bool copied = copy_prepared_mesh_from_cache(cache, out);
        pu(copied); ppainted(out);
        pu(mesh_repair_cache_can_skip_input(cache, faces, s));
        pu((bool) cache); if (cache) pcache(*cache);
        nl();
    }
}

static TexturePaintingSettings rnd_settings(int i) {
    TexturePaintingSettings s;
    s.target_colors_num = i % 3 == 0 ? 0 : 1 + below(5);
    static const double weights[] = {0.5, 0.0, 0.3, 1.0, 0.2, 0.8};
    s.smooth_weight = weights[i % 6];
    // A fixed number of subdivisions: the adaptive one (iterations 0) subdivides these small meshes
    // to 10000 faces, which TextureToColor's own vectors cover with smaller counts.
    s.oversampling_iters = i % 5 == 0 ? 2 : 1;
    s.mesh_repair_decision = i % 5 == 4 ? TexturePaintingSettings::MeshRepairDecision::RepairAndImport
                                        : TexturePaintingSettings::MeshRepairDecision::ImportWithoutRepair;
    return s;
}

static void gen_pipeline() {
    // texture_to_painting, with a cache over two rounds for some
    for (int i = 0; i < 30; ++i) {
        TexturedMesh tm = rnd_textured(i, true);
        TexturePaintingSettings s = rnd_settings(i);
        MeshRepairCachePtr cache = make_mesh_repair_cache();
        const bool with_cache = i % 4 == 2;
        if (with_cache) s.mesh_repair_cache = cache.get();
        tag("textured"); ptm(tm); nl();
        for (int round = 0; round < (with_cache ? 2 : 1); ++round) {
            Events ev;
            Cancel cancel;
            cancel.after = (i % 6 == 5 && round == 0) ? 1 + below(20) : 0;
            PaintedMesh painted;
            painted.original_color_to_cluster[{9, 9, 9}] = 1;
            tag("texture_to_painting"); psettings(s); pu(with_cache); pu(round); pu(cancel.after); pu(i % 7 == 3); arrow();
            bool ok = texture_to_painting(tm, painted, s, i % 7 == 3 ? nullptr : ev.callback(), cancel.callback());
            pu(ok); ppainted(painted); ev.print(); pu(cancel.calls);
            if (with_cache) pcache(*cache);
            nl();
            if (with_cache && round == 0) {
                // cluster_from_repair_cache with other settings
                TexturePaintingSettings s2 = rnd_settings(i + 1);
                s2.mesh_repair_cache = cache.get();
                Events ev2;
                PaintedMesh p2;
                tag("cluster_from_repair_cache"); psettings(s2); arrow();
                bool ok2 = cluster_from_repair_cache(p2, s2, ev2.callback(), nullptr);
                pu(ok2); ppainted(p2); ev2.print(); pcache(*cache); nl();
            }
        }
    }
    // face_colors_to_painting
    for (int i = 0; i < 24; ++i) {
        TexturedMesh tm = rnd_textured(i * 2, true);
        tm.precomputed_face_colors = rnd_colors(i % 6 == 5 ? tm.indices.size() / 2 : tm.indices.size(), 2 + below(4), below(30));
        if (i == 22) tm.precomputed_face_colors.clear();
        if (i % 2 == 1) {
            std::size_t n = i % 8 == 5 ? tm.vertices.size() - 1 : tm.vertices.size();
            std::vector<Rgb> seeds = rnd_colors(4, 4, 0);
            for (std::size_t k = 0; k < n; ++k) {
                Rgb c = seeds[(std::size_t) std::fabs(tm.vertices[k][0]) % 4];
                tm.precomputed_vertex_colors.push_back({(float) (c[0] / 255.0), (float) (c[1] / 255.0), (float) (c[2] / 255.0), 1.0f});
            }
        }
        TexturePaintingSettings s = rnd_settings(i);
        Events ev;
        Cancel cancel;
        cancel.after = i % 7 == 6 ? 1 + below(6) : 0;
        PaintedMesh painted;
        tag("textured"); ptm(tm); nl();
        tag("face_colors_to_painting"); psettings(s); pu(cancel.after); arrow();
        bool ok = face_colors_to_painting(tm, painted, s, ev.callback(), cancel.callback());
        pu(ok); ppainted(painted); ev.print(); pu(cancel.calls); nl();
    }
    // standard_3mf_colors_to_painting
    for (int i = 0; i < 30; ++i) {
        TexturedMesh tm = rnd_textured(i * 2, true);
        std::size_t npal = 1 + below(i % 4 == 0 ? 40 : 8);
        std::vector<std::array<float, 4>> pal;
        for (std::size_t k = 0; k < npal; ++k) pal.push_back({(float) range(-0.1, 1.1), (float) unit(), (float) (below(256) / 255.0), 1.f});
        if (npal > 1) pal[1] = {0.5f / 255.f + 0.1f, 1.5f / 255.f, 2.5f / 255.f, 1.f};
        for (std::size_t k = 0; k < tm.vertices.size(); ++k) tm.precomputed_vertex_colors.push_back(pal[((std::size_t) std::fabs(tm.vertices[k][0]) + below(2)) % npal]);
        if (i == 29) tm.precomputed_vertex_colors.pop_back();
        tm.is_standard_3mf = true;
        TexturePaintingSettings s;
        s.target_colors_num = i % 3 == 0 ? 0 : 1 + below(i % 5 == 1 ? 40 : 6);
        Events ev;
        Cancel cancel;
        cancel.after = i % 7 == 6 ? 1 + below(4) : 0;
        PaintedMesh painted;
        painted.original_color_to_cluster[{9, 9, 9}] = 1;
        tag("textured"); ptm(tm); nl();
        tag("standard_3mf_colors_to_painting"); pu(s.target_colors_num); pu(cancel.after); pu(i % 9 == 4); arrow();
        bool ok = standard_3mf_colors_to_painting(tm, painted, s, i % 9 == 4 ? nullptr : ev.callback(), cancel.callback());
        pu(ok); ppainted(painted); ev.print(); pu(cancel.calls); nl();
    }
}

static void pfacets(const FacetsAnnotation& f) {
    const auto& d = f.get_data();
    pu(d.first.size()); for (auto& p : d.first) { pi(p.first); pi(p.second); }
    pu(d.second.size()); std::printf(" "); if (d.second.empty()) std::printf("-"); for (bool b : d.second) std::printf("%d", (int) b);
}

static void pvolume(const ModelVolume& v) {
    const TriangleMesh& m = v.mesh();
    pu(m.its.vertices.size()); for (auto& p : m.its.vertices) { pf(p.x()); pf(p.y()); pf(p.z()); }
    pu(m.its.indices.size()); for (auto& f : m.its.indices) std::printf(" %d %d %d", f[0], f[1], f[2]);
    Vec3d sh = m.get_init_shift(); pd(sh.x()); pd(sh.y()); pd(sh.z());
    const Vec3d& off = v.get_offset(); pd(off.x()); pd(off.y()); pd(off.z());
    pd(v.source.mesh_offset.x()); pd(v.source.mesh_offset.y()); pd(v.source.mesh_offset.z());
    pu(v.source.is_converted_from_inches); pu(v.source.is_converted_from_meters);
    pfacets(v.supported_facets); pfacets(v.seam_facets); pfacets(v.mmu_segmentation_facets); pfacets(v.fuzzy_skin_facets);
}

static void gen_volume() {
    for (int i = 0; i < 24; ++i) {
        TexturedMesh tm = rnd_textured(i, true);
        indexed_triangle_set its;
        for (auto& v : tm.vertices) its.vertices.push_back(Vec3f(v[0] + 30.f, v[1] - 7.f, v[2] + 2.5f));
        for (auto& f : tm.indices) its.indices.push_back(Vec3i(f[0], f[1], f[2]));
        Model model;
        ModelObject* obj = model.add_object();
        ModelVolume* vol = obj->add_volume(TriangleMesh(its));
        vol->source.is_converted_from_meters = i % 5 == 3;
        vol->source.is_converted_from_inches = i % 4 == 1;
        if (i % 6 == 4) vol->source.mesh_offset = Vec3d::Zero();
        if (i % 7 == 5) vol->source.mesh_offset = Vec3d(1e-170, 0, 0);
        if (i % 3 == 0) {
            // paint something that is then reset
            TriangleSelector sel(vol->mesh());
            sel.set_facet(0, EnforcerBlockerType::ENFORCER);
            vol->supported_facets.set(sel);
            vol->seam_facets.set(sel);
        }

        // The painted mesh: the original mesh (in its un-centered frame) subdivided or not, its
        // faces colored from a palette.
        PaintedMesh painted;
        for (auto& v : its.vertices) painted.vertices.push_back({v.x() * (i % 5 == 3 ? 1000.f : 1.f), v.y(), v.z()});
        for (auto& f : its.indices) painted.indices.push_back({f[0], f[1], f[2]});
        if (i % 8 == 6) painted.indices.pop_back();
        std::vector<Rgb> pal = rnd_colors(1 + below(5), 5, 0);
        for (std::size_t f = 0; f < painted.indices.size() + (i % 9 == 2 ? 3 : 0); ++f) painted.face_colors.push_back(pal[below(pal.size())]);
        if (i == 23) painted.face_colors.clear();
        std::set<Rgb> u(painted.face_colors.begin(), painted.face_colors.end());
        painted.cluster_colors.assign(u.begin(), u.end());
        std::vector<FilamentMatch> matches;
        for (std::size_t c = 0; c < painted.cluster_colors.size() + 1; ++c) {
            FilamentMatch m;
            m.cluster_index = c == painted.cluster_colors.size() ? (int) c + 2 : (int) c;
            m.filament_index = below(6) == 0 ? -1 : (below(5) == 0 ? 31 + (int) below(3) : (int) below(4));
            if (i % 10 == 7) m.filament_index = 120 + (int) below(20);
            matches.push_back(m);
        }
        if (i == 22) matches.clear();
        tag("volume"); pvolume(*vol); nl();
        tag("apply_painted_mesh_to_volume"); ppainted(painted); pu(matches.size()); for (auto& m : matches) { pi(m.cluster_index); pi(m.filament_index); } arrow();
        bool ok = apply_painted_mesh_to_volume(painted, matches, *vol);
        pu(ok); pvolume(*vol); nl();
    }
}

int main(int argc, char** argv) {
    if (argc != 2) return 1;
    tbb::global_control one_thread(tbb::global_control::max_allowed_parallelism, 1);
    std::string what = argv[1];
    if (what == "basic") gen_basic();
    else if (what == "pipeline") { g_state ^= 0x1111; gen_pipeline(); }
    else if (what == "volume") { g_state ^= 0x2222; gen_volume(); }
    else return 1;
    return 0;
}
