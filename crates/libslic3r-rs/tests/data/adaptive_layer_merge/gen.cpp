// Reference vectors for crates/libslic3r-rs/src/adaptive_layer_merge.rs, taken from the C++ at the pin.
//
// The program compiles references/BambuStudio/src/libslic3r/AdaptiveLayerMerge.cpp as it is. The
// headers that file includes besides its own (ExPolygon.hpp, ClipperUtils.hpp, Layer.hpp,
// PrintConfig.hpp, libslic3r.h) are replaced by the declarations below, by defining their include
// guards first: the functions under test (the band planner and the nozzle helpers) use none of
// the geometry, and the geometry functions of the unit are compiled against the stand-ins but
// never called.
//
// Build and run, from the repository root (macOS arm64 clang is the reference build):
//   clang++ -std=c++17 -O2 -ffp-contract=on -I references/BambuStudio/src/libslic3r \
//       crates/libslic3r-rs/tests/data/adaptive_layer_merge/gen.cpp -o /tmp/alm_gen
//   /tmp/alm_gen > crates/libslic3r-rs/tests/data/adaptive_layer_merge/vectors.txt
//
// Output, one case per line (doubles and floats as the hex of their bits):
//   T <32 heights>                               the table of layer heights the cases index
//   RANGE nd -> min max
//   CONTAINS min max h -> 0|1
//   TOPZ nd -> float
//   BSIZE nd min_nd base -> size
//   MULTI n ids.. m diameters(float).. -> 0|1
//   BANDS band_size min max n idx.. m tops.. -> k (bottom top merged)..
//   PULL bottom top merged min max n idx.. mat.. -> ret bottom top merged
//   SPLIT bottom top preferred min max n idx.. -> ret k (bottom top)..

#define slic3r_ExPolygon_hpp_
#define slic3r_ClipperUtils_hpp_
#define slic3r_Layer_hpp_
#define slic3r_PrintConfig_hpp_
#define _libslic3r_h_

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <utility>
#include <vector>

namespace Slic3r {
static constexpr double SCALING_FACTOR = 0.00001;
#define scale_(val) ((val) / SCALING_FACTOR)
constexpr double EPSILON = 1e-4;
struct Polygon {};
using Polygons = std::vector<Polygon>;
struct ExPolygon { double area() const { return 0.; } };
using ExPolygons = std::vector<ExPolygon>;
struct Surface {};
using Surfaces = std::vector<Surface>;
inline ExPolygons shrink_ex(const ExPolygons &, float) { return {}; }
inline ExPolygons opening_ex(const ExPolygons &, float) { return {}; }
inline ExPolygons intersection_ex(const ExPolygons &, const ExPolygons &) { return {}; }
inline ExPolygons union_ex(const Polygons &) { return {}; }
inline ExPolygons diff_ex(const ExPolygons &, const ExPolygons &) { return {}; }
inline Polygons to_polygons(const ExPolygons &) { return {}; }
inline Polygons to_polygons(const Surfaces &) { return {}; }
inline void polygons_append(Polygons &, Polygons &&) {}
template<class T> void append(std::vector<T> &dst, std::vector<T> &&src) { dst.insert(dst.end(), src.begin(), src.end()); }
class PrintConfig { public: std::vector<float> diameters; };
// Stand-in for PrintConfig.cpp:454-469: the diameter of the 1 based filament id.
inline float nozzle_diameter_for_filament(const PrintConfig &config, unsigned int filament_id)
{
    return config.diameters[(filament_id == 0 ? 0 : filament_id - 1) % config.diameters.size()];
}
} // namespace Slic3r

#include "AdaptiveLayerMerge.hpp"

namespace Slic3r {
struct LayerRegion
{
    std::vector<AdaptiveMergePatch> adaptive_merge_patches;
    struct { Surfaces surfaces; } slices;
};
class Layer
{
public:
    std::vector<LayerRegion *> m_regions;
    const std::vector<LayerRegion *> &regions() const { return m_regions; }
};
} // namespace Slic3r

#include "AdaptiveLayerMerge.cpp"

using namespace Slic3r;

static uint64_t s = 0x9E3779B97F4A7C15ull;
static uint64_t rnd() { s ^= s << 13; s ^= s >> 7; s ^= s << 17; return s; }
static uint64_t rnd(uint64_t n) { return rnd() % n; }
static void pd(double v) { uint64_t b; memcpy(&b, &v, 8); printf(" %016llx", (unsigned long long) b); }
static void pf(float v) { uint32_t b; memcpy(&b, &v, 4); printf(" %08x", b); }

static const double table[32] = {
    0.04, 0.05, 0.06, 0.07, 0.08, 0.1, 0.12, 0.14, 0.16, 0.2, 0.24, 0.28, 0.3, 0.32, 0.36, 0.4,
    0.42, 0.56, 0.0399, 0.1401, 0.28 / 3., 0.42 / 4., 0.56 / 5., 0.07 + 1e-4, 0.14 + 1e-4, 0.14 - 1e-4,
    0.08 - 1e-4, 0.2 + 2e-4, 0.01, 0.6, 0.15, 0.09 };
static const double nozzles[8] = { 0.2, 0.4, 0.6, 0.8, 0.25, 0.5, 1.0, 0.3 };

// A layer stack: mostly one height, sometimes a different first layer, sometimes mixed.
static std::vector<int> stack(size_t max_n)
{
    size_t n = rnd(max_n + 1);
    std::vector<int> idx(n);
    int    base = int(rnd(4) == 0 ? rnd(18) : rnd(9));
    int    mode = int(rnd(8));
    for (size_t i = 0; i < n; ++ i)
        idx[i] = mode == 0 ? int(rnd(32)) : (mode <= 2 && rnd(6) == 0) ? int(rnd(32)) : base;
    if (n > 0 && mode >= 3 && rnd(3) == 0)
        idx[0] = int(rnd(18));
    return idx;
}
static std::vector<double> heights(const std::vector<int> &idx)
{
    std::vector<double> h;
    for (int i : idx) h.push_back(table[i]);
    return h;
}
static AdaptiveLayerHeightRange range()
{
    if (rnd(8) == 0)
        return { table[rnd(32)], table[rnd(32)] + table[rnd(32)] };
    return adaptive_layer_height_range(nozzles[rnd(8)]);
}
static void pidx(const std::vector<int> &idx)
{
    printf(" %zu", idx.size());
    for (int i : idx) printf(" %d", i);
}

int main()
{
    printf("T");
    for (double v : table) pd(v);
    printf("\n");

    for (int i = 0; i < 40; ++ i) {
        double nd = i < 8 ? nozzles[i] : double(rnd(2000)) / 1000.;
        AdaptiveLayerHeightRange r = adaptive_layer_height_range(nd);
        printf("RANGE"); pd(nd); printf(" ->"); pd(r.min_height); pd(r.max_height); printf("\n");
        printf("TOPZ"); pd(nd); printf(" ->"); pf(support_top_z_for_nozzle(nd)); printf("\n");
    }
    for (int i = 0; i < 9; ++ i) {
        double nd = 0.1 * (i + 1);
        printf("TOPZ"); pd(nd); printf(" ->"); pf(support_top_z_for_nozzle(nd)); printf("\n");
    }
    for (int i = 0; i < 200; ++ i) {
        AdaptiveLayerHeightRange r = range();
        double h;
        switch (rnd(6)) {
        case 0: h = r.min_height - EPSILON; break;
        case 1: h = r.max_height + EPSILON; break;
        case 2: h = r.min_height - 1.0001 * EPSILON; break;
        case 3: h = r.max_height + 1.0001 * EPSILON; break;
        default: h = table[rnd(32)] * double(1 + rnd(4)); break;
        }
        printf("CONTAINS"); pd(r.min_height); pd(r.max_height); pd(h); printf(" -> %d\n", int(r.contains(h)));
    }
    for (int i = 0; i < 800; ++ i) {
        double nd   = rnd(8) == 0 ? double(rnd(1500)) / 1000. : nozzles[rnd(8)];
        double mind = rnd(8) == 0 ? double(rnd(1500)) / 1000. : nozzles[rnd(8)];
        double base = rnd(10) == 0 ? 0. : rnd(4) == 0 ? double(rnd(600)) / 1000. : table[rnd(32)];
        printf("BSIZE"); pd(nd); pd(mind); pd(base);
        printf(" -> %zu\n", adaptive_layer_band_size(nd, mind, base));
    }
    for (int i = 0; i < 300; ++ i) {
        size_t n = rnd(6);
        std::vector<unsigned int> ids(n);
        for (unsigned int &id : ids) id = (unsigned int) rnd(6);
        PrintConfig config;
        config.diameters.resize(1 + rnd(6));
        int mode = int(rnd(3));
        for (float &d : config.diameters)
            d = mode == 0 ? 0.4f : rnd(6) == 0 ? 0.f : rnd(6) == 0 ? float(0.4 + 1e-4 * double(rnd(4)) - 2e-4) : float(nozzles[rnd(4)]);
        printf("MULTI %zu", n);
        for (unsigned int id : ids) printf(" %u", id);
        printf(" %zu", config.diameters.size());
        for (float d : config.diameters) pf(d);
        printf(" -> %d\n", int(has_multiple_used_nozzle_diameters(config, ids)));
    }
    for (int i = 0; i < 600; ++ i) {
        std::vector<int>         idx = stack(26);
        std::vector<double>      h   = heights(idx);
        size_t                   band_size = rnd(8) == 0 ? rnd(2) : 2 + rnd(5);
        AdaptiveLayerHeightRange r   = range();
        std::vector<uint8_t>     tops;
        if (rnd(3) == 0) {
            tops.resize(rnd(idx.size() + 3));
            for (uint8_t &t : tops) t = rnd(5) == 0 ? uint8_t(1 + rnd(2)) : 0;
        }
        std::vector<AdaptiveLayerBand> bands = build_adaptive_layer_bands(h, band_size, r, tops);
        printf("BANDS %zu", band_size); pd(r.min_height); pd(r.max_height); pidx(idx);
        printf(" %zu", tops.size());
        for (uint8_t t : tops) printf(" %d", int(t));
        printf(" -> %zu", bands.size());
        for (const AdaptiveLayerBand &b : bands) { printf(" %zu %zu", b.bottom_idx, b.top_idx); pd(b.merged_height); }
        printf("\n");
    }
    for (int i = 0; i < 500; ++ i) {
        std::vector<int>         idx = stack(14);
        std::vector<double>      h   = heights(idx);
        AdaptiveLayerHeightRange r   = range();
        std::vector<unsigned char> mat(rnd(4) == 0 ? rnd(idx.size() + 2) : idx.size());
        int density = int(1 + rnd(4));
        for (unsigned char &m : mat) m = rnd(density) == 0 ? 0 : 1;
        AdaptiveLayerBand band;
        band.bottom_idx    = rnd(idx.size() / 2 + 2);
        band.top_idx       = rnd(8) == 0 ? rnd(idx.size() + 2) : band.bottom_idx + rnd(6);
        band.merged_height = table[rnd(32)];
        printf("PULL %zu %zu", band.bottom_idx, band.top_idx); pd(band.merged_height); pd(r.min_height); pd(r.max_height); pidx(idx);
        // has_material has its own length
        printf(" %zu", mat.size());
        for (unsigned char m : mat) printf(" %d", int(m));
        bool ret = pull_adaptive_band_top_to_material(band, h, r, mat);
        printf(" -> %d %zu %zu", int(ret), band.bottom_idx, band.top_idx); pd(band.merged_height); printf("\n");
    }
    for (int i = 0; i < 600; ++ i) {
        std::vector<int>         idx = stack(22);
        std::vector<double>      h   = heights(idx);
        AdaptiveLayerHeightRange r   = range();
        size_t bottom = rnd(6) == 0 ? rnd(idx.size() + 2) : rnd(idx.size() / 2 + 1);
        size_t top    = rnd(6) == 0 ? rnd(idx.size() + 2) : bottom + rnd(idx.size() + 1 > bottom ? idx.size() + 1 - bottom : 1);
        size_t pref   = rnd(8) == 0 ? 0 : 1 + rnd(6);
        std::vector<std::pair<size_t, size_t>> groups;
        groups.emplace_back(7, 7);
        bool ret = split_adaptive_layer_span(h, bottom, top, pref, r, groups);
        printf("SPLIT %zu %zu %zu", bottom, top, pref); pd(r.min_height); pd(r.max_height); pidx(idx);
        printf(" -> %d %zu", int(ret), groups.size());
        for (const auto &g : groups) printf(" %zu %zu", g.first, g.second);
        printf("\n");
    }
    return 0;
}
