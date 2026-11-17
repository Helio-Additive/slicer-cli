// Generator of the vectors of tests/obj_color_utils_native_parity.rs: libslic3r's ObjColorUtils.hpp /
// ObjColorUtils.cpp at the pin (references/BambuStudio/src/libslic3r), on the OpenCV slicer_cli links
// (4.13.0 for the vectors in this directory).
//
// Build like tests/data/model/gen.cpp (the build's compile line for a libslic3r source, linked with
// libslic3r_core and the libraries it links, OpenCV core and imgproc among them), then
//
//   ./gen > obj_color_utils.txt
//
// Line format: tokens separated by one space; a float is its 8 hexadecimal digits, a double its 16,
// an integer decimal. The first token names the line. The script:
// - cvt_*: cv::cvtColor of every 8-bit input (RGB -> Lab, Lab -> RGB, RGB -> HSV, HSV -> RGB) laid
//   out as rows of 4096 pixels (the SIMD loops), of 1 pixel (the scalar loops) and of 37 pixels (both),
//   as an FNV-1a digest of the output, and 2048 sampled outputs;
// - nl2: cv::hal::normL2Sqr_ of random triples;
// - km_*: cv::kmeans (k-means++, 3 attempts, EPS+COUNT 300 / 0.5) of random samples with a seeded
//   cv::theRNG(): labels, centers, compactness, the generator state after;
// - qa_* / deal_*: QuantKMeans::apply of color lists and obj_color_deal_algo;
// - qi_* / qal_*: QuantKMeans::apply of an image (color spaces 0, 1, 2) and apply_aplha;
// - s3_*: standard_3mf_corner_color and standard_3mf_to_textured_meshes of models built in code.
//
// The test data of this file comes from its own generator (splitmix64), so the Rust test makes the
// same inputs. The arithmetic of this file is compiled with `#pragma clang fp contract(off)`.

#include "libslic3r/ObjColorUtils.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/TexturePainting.hpp"
#include "libslic3r/Format/OBJ.hpp"
#include "libslic3r/TriangleMesh.hpp"

#include <opencv2/core/hal/hal.hpp>

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <map>
#include <string>
#include <unordered_map>
#include <vector>

using namespace Slic3r;

#pragma clang fp contract(off)

static std::string hF(float v)
{
    uint32_t u;
    std::memcpy(&u, &v, 4);
    char b[16];
    std::snprintf(b, sizeof(b), "%08x", u);
    return b;
}
static std::string hD(double v)
{
    uint64_t u;
    std::memcpy(&u, &v, 8);
    char b[32];
    std::snprintf(b, sizeof(b), "%016llx", (unsigned long long) u);
    return b;
}
static std::string hI(long long v) { return std::to_string(v); }
static void out(const std::string &s) { std::printf("%s\n", s.c_str()); }

struct Rng
{
    uint64_t s;
    uint64_t next()
    {
        s += 0x9E3779B97F4A7C15ull;
        uint64_t z = s;
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
        return z ^ (z >> 31);
    }
    // [0, 1)
    double uni() { return double(next() >> 11) * (1.0 / 9007199254740992.0); }
    int below(int n) { return int(next() % uint64_t(n)); }
};

static uint64_t fnv(const uchar *p, size_t n)
{
    uint64_t h = 0xcbf29ce484222325ull;
    for (size_t i = 0; i < n; ++i) {
        h ^= p[i];
        h *= 0x100000001b3ull;
    }
    return h;
}

// Every 24-bit input i = (c0 << 16) | (c1 << 8) | c2, laid out in rows of `width` pixels (the last
// pixels that do not fill a row are left out).
static void scenario_cvt()
{
    const int total = 1 << 24;
    struct Code { const char *name; int code; };
    const Code codes[] = { {"rgb2lab", cv::COLOR_RGB2Lab}, {"lab2rgb", cv::COLOR_Lab2RGB}, {"rgb2hsv", cv::COLOR_RGB2HSV}, {"hsv2rgb", cv::COLOR_HSV2RGB} };
    const int widths[] = { 4096, 1, 37 };
    for (const Code &c : codes) {
        for (int width : widths) {
            const int rows = total / width;
            cv::Mat src(rows, width, CV_8UC3);
            for (int i = 0; i < rows * width; ++i) {
                uchar *p = src.ptr<uchar>(0) + size_t(i) * 3;
                p[0] = uchar(i >> 16);
                p[1] = uchar(i >> 8);
                p[2] = uchar(i);
            }
            cv::Mat dst;
            cv::cvtColor(src, dst, c.code);
            const uchar *d = dst.ptr<uchar>(0);
            std::string line = std::string("cvt_") + c.name + " " + hI(width) + " " + hI((long long) fnv(d, size_t(rows) * width * 3));
            out(line);
            std::string s = std::string("cvts_") + c.name + " " + hI(width);
            for (int k = 0; k < 2048; ++k) {
                const int i = int((long long) k * 8191 % (rows * width));
                s += " " + hI(d[size_t(i) * 3]) + " " + hI(d[size_t(i) * 3 + 1]) + " " + hI(d[size_t(i) * 3 + 2]);
            }
            out(s);
        }
    }
}

static void scenario_norm()
{
    Rng r{11};
    for (int k = 0; k < 2000; ++k) {
        float a[3], b[3];
        const double scale = k < 1000 ? 255.0 : 3.0;
        for (int j = 0; j < 3; ++j) { a[j] = float(r.uni() * scale); b[j] = float(r.uni() * scale); }
        if (k % 7 == 0) { a[0] = float(r.below(256)); b[0] = float(r.below(256)); }
        out("nl2 " + hF(a[0]) + " " + hF(a[1]) + " " + hF(a[2]) + " " + hF(b[0]) + " " + hF(b[1]) + " " + hF(b[2]) + " " +
            hF(cv::hal::normL2Sqr_(a, b, 3)));
    }
}

// Samples: integers in [0, 256) (like Lab bytes) or floats, some duplicated.
static cv::Mat km_data(Rng &r, int n, bool ints)
{
    cv::Mat data(n, 1, CV_32FC3);
    for (int i = 0; i < n; ++i) {
        cv::Vec3f &p = data.at<cv::Vec3f>(i, 0);
        if (i > 0 && r.below(5) == 0) {
            p = data.at<cv::Vec3f>(r.below(i), 0);
            continue;
        }
        for (int j = 0; j < 3; ++j)
            p[j] = ints ? float(r.below(256)) : float(r.uni() * 200.0 - 50.0);
    }
    return data;
}

static void scenario_kmeans()
{
    Rng r{23};
    const int sizes[][2] = { {1, 1}, {5, 1}, {5, 5}, {17, 3}, {40, 7}, {100, 15}, {333, 15}, {1000, 8}, {64, 2}, {250, 12} };
    int case_id = 0;
    for (const auto &sz : sizes) {
        for (int ints = 1; ints >= 0; --ints) {
            const int n = sz[0], K = sz[1];
            cv::Mat data = km_data(r, n, ints != 0);
            const uint64 seed = 1 + r.below(100000);
            cv::theRNG().state = seed;
            cv::Mat labels, centers;
            const double comp = cv::kmeans(data, K, labels, cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 300, 0.5), 3, cv::KMEANS_PP_CENTERS, centers);
            std::string s = "km_labels " + hI(case_id);
            for (int i = 0; i < n; ++i) s += " " + hI(labels.at<int>(i, 0));
            out(s);
            s = "km_centers " + hI(case_id) + " " + hI(centers.rows) + " " + hI(centers.cols * centers.channels());
            for (int k = 0; k < centers.rows; ++k)
                for (int j = 0; j < centers.cols * centers.channels(); ++j) s += " " + hF(centers.ptr<float>(k)[j]);
            out(s);
            out("km_comp " + hI(case_id) + " " + hD(comp) + " " + hI((long long) cv::theRNG().state));
            ++case_id;
        }
    }
}

// Colors in [0, 1]: from a small palette (exact bytes), or anything.
static std::vector<std::array<float, 4>> color_list(Rng &r, int n, int palette)
{
    std::vector<std::array<float, 4>> pal;
    for (int k = 0; k < palette; ++k)
        pal.push_back({float(r.below(256)) / 255.f, float(r.below(256)) / 255.f, float(r.below(256)) / 255.f, 1.f});
    std::vector<std::array<float, 4>> colors;
    for (int i = 0; i < n; ++i) {
        if (palette > 0)
            colors.push_back(pal[r.below(palette)]);
        else
            colors.push_back({float(r.uni()), float(r.uni()), float(r.uni()), 1.f});
    }
    return colors;
}

static void scenario_quant()
{
    Rng r{37};
    const int cases[][4] = {
        // n, palette (0: random), num_cluster, max_cluster
        {10, 3, -1, 15}, {50, 6, -1, 15}, {50, 20, -1, 15}, {200, 0, -1, 15}, {200, 0, 4, 15}, {200, 0, 20, 15},
        {120, 15, -1, 15}, {120, 16, -1, 15}, {300, 40, 8, 32}, {500, 0, -1, 32}, {80, 0, 1, 15}, {1, 1, -1, 15},
    };
    int case_id = 0;
    for (const auto &c : cases) {
        std::vector<std::array<float, 4>> colors = color_list(r, c[0], c[1]);
        QuantKMeans quant(10);
        std::vector<std::array<float, 4>> results;
        std::vector<int> labels;
        quant.apply(colors, results, labels, c[2], c[3]);
        std::string s = "qa_clusters " + hI(case_id) + " " + hI((long long) results.size());
        for (auto &v : results) s += " " + hF(v[0]) + " " + hF(v[1]) + " " + hF(v[2]) + " " + hF(v[3]);
        out(s);
        s = "qa_labels " + hI(case_id);
        for (int l : labels) s += " " + hI(l);
        out(s);

        std::vector<RGBA> in(colors.begin(), colors.end()), cl;
        std::vector<int> lb;
        char number = char(c[2]);
        const bool ok = obj_color_deal_algo(in, cl, lb, number, c[3]);
        s = "deal " + hI(case_id) + " " + hI(ok) + " " + hI((long long) cl.size()) + " " + hI((long long) lb.size());
        for (auto &v : cl) s += " " + hF(v[0]) + " " + hF(v[1]) + " " + hF(v[2]);
        out(s);
        ++case_id;
    }
}

static void scenario_image()
{
    Rng r{41};
    const int rows = 7, cols = 19;
    for (int cs = 0; cs <= 2; ++cs) {
        for (int pal = 0; pal <= 1; ++pal) {
            cv::Mat img(rows, cols, CV_8UC3);
            std::vector<cv::Vec3b> palette;
            for (int k = 0; k < 9; ++k) palette.push_back(cv::Vec3b(uchar(r.below(256)), uchar(r.below(256)), uchar(r.below(256))));
            for (int i = 0; i < rows; ++i)
                for (int j = 0; j < cols; ++j)
                    img.at<cv::Vec3b>(i, j) = pal ? palette[r.below(9)] : cv::Vec3b(uchar(r.below(256)), uchar(r.below(256)), uchar(r.below(256)));
            const int k = 4 + r.below(4);
            cv::theRNG().state = 99 + cs * 2 + pal;
            QuantKMeans quant(10);
            cv::Mat result = img.clone();
            quant.apply(img, result, k, cs);
            std::string s = "qi " + hI(cs) + " " + hI(pal) + " " + hI(k) + " " + hI((long long) cv::theRNG().state);
            for (int i = 0; i < rows; ++i)
                for (int j = 0; j < cols; ++j) {
                    const cv::Vec3b p = result.at<cv::Vec3b>(i, j);
                    s += " " + hI(p[0]) + " " + hI(p[1]) + " " + hI(p[2]);
                }
            out(s);

            cv::Mat img4(rows, cols, CV_8UC4);
            for (int i = 0; i < rows; ++i)
                for (int j = 0; j < cols; ++j) {
                    const cv::Vec3b p = img.at<cv::Vec3b>(i, j);
                    img4.at<cv::Vec4b>(i, j) = cv::Vec4b(p[0], p[1], p[2], uchar(r.below(4) == 0 ? r.below(10) : 10 + r.below(246)));
                }
            cv::theRNG().state = 199 + cs * 2 + pal;
            QuantKMeans quant4(10);
            cv::Mat result4 = img4.clone();
            quant4.apply_aplha(img4, result4, k, cs);
            s = "qal " + hI(cs) + " " + hI(pal) + " " + hI(k) + " " + hI((long long) cv::theRNG().state);
            for (int i = 0; i < rows; ++i)
                for (int j = 0; j < cols; ++j) {
                    const cv::Vec4b p = result4.at<cv::Vec4b>(i, j);
                    s += " " + hI(p[0]) + " " + hI(p[1]) + " " + hI(p[2]) + " " + hI(p[3]);
                }
            out(s);
        }
    }
}

static indexed_triangle_set box_its(float sx, float sy, float sz, float ox, float oy, float oz)
{
    indexed_triangle_set its = its_make_cube(sx, sy, sz);
    for (auto &v : its.vertices) v += Vec3f(ox, oy, oz);
    return its;
}

static const char *COLORS[] = {"#FF0000", "#00FF00FF", "#0000FF", "", "#123456", "#ABCDEF80", "bad", "#7F7F7F"};

static void dump_tex(const std::string &tag, const TexturedMesh &t, bool ret, bool has_uncolored)
{
    std::string s = tag + " " + hI(ret) + " " + hI(has_uncolored) + " " + hI(t.is_standard_3mf) + " " + hI((long long) t.vertices.size()) + " " +
                    hI((long long) t.indices.size());
    for (auto &v : t.vertices) s += " " + hF(v[0]) + " " + hF(v[1]) + " " + hF(v[2]);
    out(s);
    s = tag + "_f";
    for (auto &f : t.indices) s += " " + hI(f[0]) + " " + hI(f[1]) + " " + hI(f[2]);
    for (auto &c : t.precomputed_face_colors) s += " " + hI((long long) c[0]) + " " + hI((long long) c[1]) + " " + hI((long long) c[2]);
    for (auto &c : t.precomputed_vertex_colors) s += " " + hF(c[0]) + " " + hF(c[1]) + " " + hF(c[2]) + " " + hF(c[3]);
    out(s);
}

static void scenario_standard_3mf()
{
    Rng r{53};
    std::map<int, std::vector<std::string>> groups;
    groups[1] = {COLORS[0], COLORS[1], COLORS[2]};
    groups[2] = {COLORS[3], COLORS[4], COLORS[5], COLORS[6]};
    groups[3] = {};
    groups[5] = {COLORS[7]};

    // corner colors of random bindings
    for (int k = 0; k < 200; ++k) {
        VolumeColorInfo info;
        info.pid = r.below(7) - 1;
        info.pindex = r.below(6) - 1;
        TriangleColor binding;
        binding.pid = r.below(7) - 1;
        for (int c = 0; c < 3; ++c) binding.indices[c] = r.below(7) - 2;
        std::string s = "s3_corner " + hI(info.pid) + " " + hI(info.pindex) + " " + hI(binding.pid) + " " + hI(binding.indices[0]) + " " +
                        hI(binding.indices[1]) + " " + hI(binding.indices[2]);
        for (int c = 0; c < 3; ++c) {
            const std::string *col = standard_3mf_corner_color(groups, info, binding, c);
            s += " " + (col ? (col->empty() ? std::string("E") : *col) : std::string("-"));
        }
        out(s);
    }

    for (int scene = 0; scene < 4; ++scene) {
        Model model;
        std::unordered_map<int, VolumeColorInfo> data;
        const int n_objects = scene == 0 ? 1 : 2;
        for (int o = 0; o < n_objects; ++o) {
            ModelObject *obj = model.add_object();
            const int n_vol = scene == 2 ? 2 : 1 + (o == 1 ? 1 : 0);
            for (int v = 0; v < n_vol; ++v) {
                ModelVolume *vol = obj->add_volume(TriangleMesh(box_its(1.f + v, 2.f, 1.5f, float(o), 0.5f, 0.25f * v)));
                vol->set_offset(Vec3d(0.5 * v, -0.25 * o, 0.125));
                vol->set_rotation(Vec3d(0.1 * scene, 0.2 * v, 0.3 * o));
                vol->set_scaling_factor(Vec3d(1.0, 1.0 + 0.5 * v, 1.0));
                if (scene == 3 && v == 1) vol->set_type(ModelVolumeType::PARAMETER_MODIFIER);
                VolumeColorInfo info;
                info.pid = (v + o) % 2 == 0 ? 1 : -1;
                info.pindex = 1;
                const size_t nf = vol->mesh().its.indices.size();
                const size_t used = (scene == 1 && o == 1) ? nf - 1 : nf;
                for (size_t f = 0; f < used; ++f) {
                    TriangleColor tc;
                    const int pick = r.below(5);
                    tc.pid = pick == 0 ? -1 : (pick == 1 ? 2 : (pick == 2 ? 5 : 1));
                    for (int c = 0; c < 3; ++c) tc.indices[c] = r.below(5) - 1;
                    info.triangle_colors.push_back(tc);
                }
                if (!(scene == 3 && o == 1))
                    data[int(vol->id().id)] = info;
            }
            ModelInstance *inst = obj->add_instance();
            inst->set_offset(Vec3d(10.0 * o, 5.0, 0.0));
            inst->set_rotation(Vec3d(0.0, 0.0, 0.5 * o + 0.1 * scene));
        }
        TexturedMesh tex;
        bool has_uncolored = true;
        const bool ret = standard_3mf_to_textured_meshes(model, groups, data, tex, has_uncolored);
        dump_tex("s3_tex" + hI(scene), tex, ret, has_uncolored);
    }
}

int main()
{
    scenario_cvt();
    scenario_norm();
    scenario_kmeans();
    scenario_quant();
    scenario_image();
    scenario_standard_3mf();
    return 0;
}
