// Reference vectors for crates/libslic3r-rs/src/support_merge_plan.rs, taken from the C++ at the pin.
//
// The program compiles references/BambuStudio/src/libslic3r/SupportMergePlan.cpp and
// AdaptiveLayerMerge.cpp as they are. The libslic3r headers they include besides their own are
// replaced by the stand-ins below (their include guards are defined first): a PrintObject, Print,
// Layer, LayerRegion and the config structs with just the members make_support_merge_plan() reads,
// each returning what the generator put there.
//
// Build and run, from the repository root (macOS arm64 clang is the reference build):
//   clang++ -std=c++17 -O2 -ffp-contract=on -I references/BambuStudio/src/libslic3r \
//       crates/libslic3r-rs/tests/data/support_merge_plan/gen.cpp -o /tmp/smp_gen
//   /tmp/smp_gen > crates/libslic3r-rs/tests/data/support_merge_plan/vectors.txt
//
// Output: a stream of whitespace separated tokens, read back in the order written (doubles and
// floats as the hex of their bits). Per object:
//   OBJ uses_multiple_nozzle_diameters grid_nozzle base_height support_filament interface_filament
//   ND n diameters(float)..            nozzle diameter of filament 1..n
//   SOL n flags..                      filament_soluble
//   VOL k { n filament ids.. }         get_extruders() of every model part volume
//   LAYERS n { print_z height empty regions { slices_empty perimeters_empty wall sparse solid
//              n slots.. } }           slots: filament_slot_by_role of the non-empty fill collections
//   PLAN <plan>                        what make_support_merge_plan() returned
//   MUT <plan>                         the same plan after the generator changed some of it; the
//                                      queries below run on this one
//   Q n { top_z height filament_id band_size layer_idx -> results }
// <plan>: enabled interface_fold_enabled, PZ n.., LH n.., CT n { filament_id nozzle_diameter min max
//   band_size } (the distinct candidates), CPL layers { n indices into CT.. }, ICPL the same,
//   FB k { filament_id n { bottom top merged } }, SL n { bottom top top_z height filament_id
//   nozzle_diameter bottom_anchor }, SI n indices..
// results: candidates_in_span(top_z - height, top_z) as n filament ids, candidates_for_slab as n
//   filament ids, coarsest_for_slab (filament_id nozzle_diameter), finest_interface_for_slab (the
//   same), max_interface_height, max_height_for_slab, base_height_at, fixed_band_height_at,
//   coarsest_band_containing (filament_id nozzle_diameter band_height), slab_for_layer (index or -1)

#define slic3r_ExPolygon_hpp_
#define slic3r_ClipperUtils_hpp_
#define slic3r_Layer_hpp_
#define slic3r_PrintConfig_hpp_
#define slic3r_ToolOrdering_hpp_
#define slic3r_Model_hpp_
#define slic3r_Print_hpp_
#define _libslic3r_h_

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
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

template<class T> struct Opt { T value {}; };
struct OptBools
{
    std::vector<unsigned char> values;
    // Config.hpp:681-685: out of range falls back to the first value.
    bool get_at(size_t i) const { return i < values.size() ? values[i] != 0 : values.front() != 0; }
};
struct OptFloats { std::vector<double> values; size_t size() const { return values.size(); } };
class PrintConfig
{
public:
    std::vector<float> diameters;
    OptBools           filament_soluble;
    OptFloats          filament_diameter;
};
// Stand-in for PrintConfig.cpp:454-469: the diameter of the 1 based filament id.
inline float nozzle_diameter_for_filament(const PrintConfig &config, unsigned int filament_id)
{
    return config.diameters[(filament_id == 0 ? 0 : filament_id - 1) % config.diameters.size()];
}
class DynamicPrintConfig { public: double grid_nozzle { 0. }; };
inline double layer_grid_nozzle_diameter(const DynamicPrintConfig &config) { return config.grid_nozzle; }
struct PrintRegionConfig { Opt<int> wall_filament, sparse_infill_filament, solid_infill_filament; };
struct PrintObjectConfig { Opt<int> support_filament, support_interface_filament; };
struct GlobalFeatureFilaments
{
    static GlobalFeatureFilaments from_config(const PrintRegionConfig &, size_t) { return {}; }
};
struct ModelVolume
{
    bool             model_part { true };
    std::vector<int> extruders;
    bool             is_model_part() const { return model_part; }
    std::vector<int> get_extruders(const GlobalFeatureFilaments &) const { return extruders; }
};
struct ModelObject { std::vector<ModelVolume *> volumes; };
struct ExtrusionEntity { virtual ~ExtrusionEntity() = default; };
struct ExtrusionPath : ExtrusionEntity {};
struct ExtrusionEntityCollection : ExtrusionEntity
{
    int  slot { 0 };
    bool is_empty { false };
    bool empty() const { return is_empty; }
};
struct LayerTools
{
    static unsigned int filament_slot_by_role(const ExtrusionEntityCollection &fill, const PrintRegionConfig &)
    {
        return (unsigned int) fill.slot;
    }
};
} // namespace Slic3r

#include "AdaptiveLayerMerge.hpp"

namespace Slic3r {
struct PrintRegion
{
    PrintRegionConfig        cfg;
    const PrintRegionConfig &config() const { return cfg; }
};
struct LayerRegion
{
    std::vector<AdaptiveMergePatch> adaptive_merge_patches;
    struct { Surfaces surfaces; bool is_empty { false }; bool empty() const { return is_empty; } } slices;
    struct { bool is_empty { false }; bool empty() const { return is_empty; } } perimeters;
    struct { std::vector<ExtrusionEntity *> entities; } fills;
    PrintRegion        reg;
    const PrintRegion &region() const { return reg; }
};
class Layer
{
public:
    double                     print_z { 0. };
    double                     height { 0. };
    bool                       is_empty { false };
    std::vector<LayerRegion *> m_regions;
    const std::vector<LayerRegion *> &regions() const { return m_regions; }
    bool                       empty() const { return is_empty; }
};
using ConstLayerPtrsAdaptor = std::vector<const Layer *>;
struct SlicingParameters { double layer_height { 0. }; };
class Print
{
public:
    PrintConfig        m_config;
    DynamicPrintConfig m_full;
    PrintRegionConfig  m_default_region;
    bool               multi { false };
    const PrintConfig        &config() const { return m_config; }
    const DynamicPrintConfig &full_print_config() const { return m_full; }
    const PrintRegionConfig  &default_region_config() const { return m_default_region; }
    bool                      uses_multiple_nozzle_diameters() const { return multi; }
};
class PrintObject
{
public:
    PrintObjectConfig     m_config;
    Print                 m_print;
    ConstLayerPtrsAdaptor m_layers;
    SlicingParameters     m_slicing;
    ModelObject          *m_model_object { nullptr };
    const PrintObjectConfig     &config() const { return m_config; }
    const Print                 *print() const { return &m_print; }
    const ConstLayerPtrsAdaptor &layers() const { return m_layers; }
    const SlicingParameters     &slicing_parameters() const { return m_slicing; }
    const ModelObject           *model_object() const { return m_model_object; }
};
} // namespace Slic3r

#include "AdaptiveLayerMerge.cpp"
#include "SupportMergePlan.cpp"

using namespace Slic3r;

static uint64_t s = 0xD1B54A32D192ED03ull;
static uint64_t rnd() { s ^= s << 13; s ^= s >> 7; s ^= s << 17; return s; }
static uint64_t rnd(uint64_t n) { return rnd() % n; }
static void pd(double v) { uint64_t b; memcpy(&b, &v, 8); printf(" %llx", (unsigned long long) b); }
static void pf(float v) { uint32_t b; memcpy(&b, &v, 4); printf(" %x", b); }

static bool same(const SupportMergePlan::Candidate &a, const SupportMergePlan::Candidate &b)
{
    return a.filament_id == b.filament_id && memcmp(&a.nozzle_diameter, &b.nozzle_diameter, 8) == 0 &&
           memcmp(&a.window.min_height, &b.window.min_height, 8) == 0 &&
           memcmp(&a.window.max_height, &b.window.max_height, 8) == 0 && a.band_size == b.band_size;
}

static void dump(const char *tag, const SupportMergePlan &plan)
{
    printf("%s %d %d\n", tag, int(plan.enabled), int(plan.interface_fold_enabled));
    printf("PZ %zu", plan.layer_print_z.size());
    for (double v : plan.layer_print_z) pd(v);
    printf("\nLH %zu", plan.layer_heights.size());
    for (double v : plan.layer_heights) pd(v);
    std::vector<SupportMergePlan::Candidate> table;
    auto index = [&table](const SupportMergePlan::Candidate &c) {
        for (size_t i = 0; i < table.size(); ++ i)
            if (same(table[i], c)) return i;
        table.push_back(c);
        return table.size() - 1;
    };
    for (const auto *list : { &plan.candidates_per_layer, &plan.interface_candidates_per_layer })
        for (const auto &layer : *list)
            for (const auto &c : layer) index(c);
    printf("\nCT %zu", table.size());
    for (const auto &c : table) {
        printf(" %d", c.filament_id); pd(c.nozzle_diameter); pd(c.window.min_height); pd(c.window.max_height);
        printf(" %zu", c.band_size);
    }
    for (const auto *list : { &plan.candidates_per_layer, &plan.interface_candidates_per_layer }) {
        printf("\n%s %zu", list == &plan.candidates_per_layer ? "CPL" : "ICPL", list->size());
        for (const auto &layer : *list) {
            printf(" %zu", layer.size());
            for (const auto &c : layer) printf(" %zu", index(c));
        }
    }
    printf("\nFB %zu", plan.fixed_bands_per_filament.size());
    for (const auto &entry : plan.fixed_bands_per_filament) {
        printf(" %d %zu", entry.first, entry.second.size());
        for (const AdaptiveLayerBand &b : entry.second) { printf(" %zu %zu", b.bottom_idx, b.top_idx); pd(b.merged_height); }
    }
    printf("\nSL %zu", plan.slabs.size());
    for (const SupportMergePlan::Slab &b : plan.slabs) {
        printf(" %zu %zu", b.bottom_idx, b.top_idx); pd(b.top_z); pd(b.height);
        printf(" %d", b.filament_id); pd(b.nozzle_diameter); printf(" %d", int(b.bottom_anchor));
    }
    printf("\nSI %zu", plan.slab_index_per_layer.size());
    for (int i : plan.slab_index_per_layer) printf(" %d", i);
    printf("\n");
}

int main()
{
    static const double nozzles[4]  = { 0.2, 0.4, 0.6, 0.8 };
    static const double bases[8]    = { 0.04, 0.08, 0.1, 0.12, 0.16, 0.2, 0.06, 0.14 };
    const int num_objects = 52;
    printf("N %d\n", num_objects);
    for (int obj = 0; obj < num_objects; ++ obj) {
        PrintObject &object = *new PrintObject();
        Print       &print  = object.m_print;
        const size_t num_filaments = 2 + rnd(4);
        print.multi = rnd(10) != 0;
        int diameter_mode = int(rnd(4));
        double min_nozzle = 10.;
        for (size_t i = 0; i < num_filaments; ++ i) {
            double d = diameter_mode == 0 ? nozzles[rnd(2)] : nozzles[rnd(4)];
            if (rnd(12) == 0) d = 0.;
            print.m_config.diameters.push_back(float(d));
            if (d > 0. && d < min_nozzle) min_nozzle = d;
            print.m_config.filament_soluble.values.push_back(rnd(7) == 0);
            print.m_config.filament_diameter.values.push_back(1.75);
        }
        if (min_nozzle > 5.) min_nozzle = 0.4;
        print.m_full.grid_nozzle = rnd(8) == 0 ? nozzles[rnd(4)] : double(float(min_nozzle));
        const double base = rnd(3) == 0 ? bases[rnd(8)] : bases[rnd(3)] * (min_nozzle / 0.2 > 2. ? 2. : 1.);
        object.m_slicing.layer_height = base;
        object.m_config.support_filament.value           = rnd(4) == 0 ? int(1 + rnd(num_filaments)) : 0;
        object.m_config.support_interface_filament.value = rnd(4) == 0 ? int(1 + rnd(num_filaments)) : 0;

        printf("OBJ %d", int(print.multi)); pd(print.m_full.grid_nozzle); pd(base);
        printf(" %d %d\n", object.m_config.support_filament.value, object.m_config.support_interface_filament.value);
        printf("ND %zu", num_filaments);
        for (float d : print.m_config.diameters) pf(d);
        printf("\nSOL %zu", num_filaments);
        for (unsigned char b : print.m_config.filament_soluble.values) printf(" %d", int(b));

        // The filaments this object uses: a subset, so that layers repeat the same few.
        std::vector<int> used;
        for (size_t i = 0; i < 1 + rnd(3); ++ i) used.push_back(int(1 + rnd(num_filaments)));
        auto pick = [&used, num_filaments]() {
            return rnd(12) == 0 ? 0 : rnd(10) == 0 ? int(1 + rnd(num_filaments)) : used[rnd(used.size())];
        };

        std::vector<std::vector<int>> model_parts;
        if (rnd(8) != 0) {
            object.m_model_object = new ModelObject();
            for (size_t i = 0; i < 1 + rnd(3); ++ i) {
                ModelVolume *volume = new ModelVolume();
                volume->model_part = rnd(4) != 0;
                for (size_t j = 0; j < 1 + rnd(3); ++ j) volume->extruders.push_back(pick());
                object.m_model_object->volumes.push_back(volume);
                if (volume->model_part) model_parts.push_back(volume->extruders);
            }
        }
        printf("\nVOL %zu", model_parts.size());
        for (const auto &e : model_parts) { printf(" %zu", e.size()); for (int id : e) printf(" %d", id); }

        const size_t num_layers = rnd(12) == 0 ? rnd(3) : 4 + rnd(21);
        printf("\nLAYERS %zu\n", num_layers);
        double z = 0.;
        const size_t region_change = rnd(num_layers + 1);
        for (size_t l = 0; l < num_layers; ++ l) {
            Layer *layer  = new Layer();
            layer->height = (l == 0 && rnd(3) == 0) ? bases[rnd(8)] : rnd(15) == 0 ? bases[rnd(8)] : base;
            z += layer->height;
            layer->print_z = z;
            const size_t num_regions = rnd(10) == 0 ? 0 : 1 + rnd(2);
            bool all_empty = true;
            pd(layer->print_z); pd(layer->height);
            std::vector<LayerRegion *> regions;
            for (size_t r = 0; r < num_regions; ++ r) {
                LayerRegion *region = new LayerRegion();
                region->slices.is_empty     = rnd(8) == 0;
                region->perimeters.is_empty = rnd(5) == 0;
                // Mostly one filament per region, changing once along the height.
                int main_filament = used[(r + (l >= region_change ? 1 : 0)) % used.size()];
                region->reg.cfg.wall_filament.value          = rnd(6) == 0 ? pick() : main_filament;
                region->reg.cfg.sparse_infill_filament.value = rnd(6) == 0 ? pick() : main_filament;
                region->reg.cfg.solid_infill_filament.value  = rnd(6) == 0 ? pick() : main_filament;
                for (size_t f = 0; f < rnd(3); ++ f) {
                    if (rnd(6) == 0) { region->fills.entities.push_back(new ExtrusionPath()); continue; }
                    ExtrusionEntityCollection *fill = new ExtrusionEntityCollection();
                    fill->is_empty = rnd(6) == 0;
                    fill->slot     = rnd(2) ? region->reg.cfg.sparse_infill_filament.value : region->reg.cfg.solid_infill_filament.value;
                    region->fills.entities.push_back(fill);
                }
                all_empty = all_empty && region->slices.is_empty;
                regions.push_back(region);
            }
            layer->m_regions = regions;
            layer->is_empty  = rnd(10) == 0 ? ! all_empty : all_empty;
            printf(" %d %zu", int(layer->is_empty), num_regions);
            for (const LayerRegion *region : regions) {
                printf(" %d %d %d %d %d", int(region->slices.is_empty), int(region->perimeters.is_empty),
                       region->reg.cfg.wall_filament.value, region->reg.cfg.sparse_infill_filament.value,
                       region->reg.cfg.solid_infill_filament.value);
                std::vector<int> slots;
                for (const ExtrusionEntity *ee : region->fills.entities)
                    if (const auto *fill = dynamic_cast<const ExtrusionEntityCollection *>(ee); fill != nullptr && ! fill->empty())
                        slots.push_back(fill->slot);
                printf(" %zu", slots.size());
                for (int slot : slots) printf(" %d", slot);
            }
            printf("\n");
            object.m_layers.push_back(layer);
        }

        SupportMergePlan plan = make_support_merge_plan(object);
        dump("PLAN", plan);

        // Change some of the plan, so that the queries also meet states make_support_merge_plan() does
        // not produce: a candidate without its ladder, a layer without candidates, another band size.
        if (rnd(4) == 0 && ! plan.fixed_bands_per_filament.empty()) {
            auto it = plan.fixed_bands_per_filament.begin();
            std::advance(it, rnd(plan.fixed_bands_per_filament.size()));
            plan.fixed_bands_per_filament.erase(it);
        }
        if (rnd(6) == 0 && ! plan.candidates_per_layer.empty())
            plan.candidates_per_layer[rnd(plan.candidates_per_layer.size())].clear();
        if (rnd(6) == 0 && ! plan.interface_candidates_per_layer.empty())
            plan.interface_candidates_per_layer[rnd(plan.interface_candidates_per_layer.size())].clear();
        if (rnd(6) == 0 && ! plan.candidates_per_layer.empty()) {
            auto &layer = plan.candidates_per_layer[rnd(plan.candidates_per_layer.size())];
            if (! layer.empty()) layer[rnd(layer.size())].band_size = 1 + rnd(3);
        }
        dump("MUT", plan);

        const size_t n = plan.layer_print_z.size();
        const int    num_queries = 20;
        printf("Q %d\n", num_queries);
        for (int q = 0; q < num_queries; ++ q) {
            const size_t layer_idx = rnd(n + 3);
            double top_z = n == 0 ? bases[rnd(8)] * double(rnd(5)) : plan.layer_print_z[rnd(n)];
            switch (rnd(8)) {
            case 0: top_z += 0.5 * EPSILON; break;
            case 1: top_z -= 0.5 * EPSILON; break;
            case 2: top_z += 2. * EPSILON; break;
            case 3: top_z -= 2. * EPSILON; break;
            case 4: top_z += 0.5 * base; break;
            case 5: if (rnd(3) == 0) top_z = double(rnd(6000)) / 1000. - 0.5; break;
            default: break;
            }
            double height = base * double(1 + rnd(4));
            if (n > 0 && rnd(3) == 0) height = plan.layer_heights[rnd(n)];
            if (! plan.slabs.empty() && rnd(2)) {
                const SupportMergePlan::Slab &slab = plan.slabs[rnd(plan.slabs.size())];
                height = slab.height;
                if (rnd(3) != 0) top_z = slab.top_z;
            } else if (! plan.fixed_bands_per_filament.empty() && rnd(2)) {
                auto it = plan.fixed_bands_per_filament.begin();
                std::advance(it, rnd(plan.fixed_bands_per_filament.size()));
                if (! it->second.empty()) {
                    const AdaptiveLayerBand &band = it->second[rnd(it->second.size())];
                    height = band.merged_height;
                    if (rnd(3) != 0 && band.top_idx < n) top_z = plan.layer_print_z[band.top_idx];
                }
            }
            SupportMergePlan::Candidate candidate;
            candidate.filament_id = int(rnd(num_filaments + 2));
            candidate.band_size   = 1 + rnd(3);
            if (n > 0 && rnd(4) != 0) {
                const auto &layer = plan.candidates_per_layer[rnd(n) % plan.candidates_per_layer.size()];
                if (! layer.empty()) candidate = layer[rnd(layer.size())];
            }
            pd(top_z); pd(height); printf(" %d %zu %zu ->", candidate.filament_id, candidate.band_size, layer_idx);
            for (const auto &list : { plan.candidates_in_span(top_z - height, top_z), plan.candidates_for_slab(top_z, height) }) {
                printf(" %zu", list.size());
                for (const auto &c : list) printf(" %d", c.filament_id);
            }
            for (const auto &c : { plan.coarsest_for_slab(top_z, height), plan.finest_interface_for_slab(top_z, height) }) {
                printf(" %d", c.filament_id); pd(c.nozzle_diameter);
            }
            pd(plan.max_interface_height(top_z, height));
            pd(plan.max_height_for_slab(top_z, height));
            pd(plan.base_height_at(top_z));
            pd(plan.fixed_band_height_at(candidate, top_z));
            double band_height = -1.;
            const SupportMergePlan::Candidate c = plan.coarsest_band_containing(top_z, band_height);
            printf(" %d", c.filament_id); pd(c.nozzle_diameter); pd(band_height);
            const SupportMergePlan::Slab *slab = plan.slab_for_layer(layer_idx);
            printf(" %d\n", slab == nullptr ? -1 : int(slab - plan.slabs.data()));
        }
    }
    return 0;
}
