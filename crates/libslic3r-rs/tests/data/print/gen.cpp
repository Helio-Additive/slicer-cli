// Generator of the vectors of tests/print_native_parity.rs: libslic3r's Print.cpp and PrintApply.cpp
// at the pin (references/BambuStudio/src/libslic3r; the override layer has no copy of them).
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
//   ./gen > vectors.txt
//
// PrintApply.cpp is compiled into this file (its helpers are static), with the private members
// of Print, PrintObject and PrintRegion made public to read the state `Print::apply` builds.
//
// It builds small Models in code (cubes, objects with several instances and rotations, modifier
// volumes, part and object filaments, layer ranges) with a full print config, runs `Print::apply`
// and prints one line per result: the PrintObjects with their instances, configs and regions, the
// region lists of their layer ranges, the query functions of Print.cpp, and the static helpers of
// PrintApply.cpp. The Rust test builds the same models, assembles the same Print with the ported
// functions and has to print the same lines. The vectors were made on macOS arm64 with Apple
// clang (its default -ffp-contract=on) and Eigen 5.0.1.
//
// Line format: tokens separated by one space; a double is its 16 hexadecimal digits, a float its
// 8, an integer decimal. An object ID is written relative to the ID of the scenario's model.

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <functional>
#include <map>
#include <memory>
#include <numeric>
#include <set>
#include <string>
#include <vector>

#define private public
#define protected public
#include "libslic3r/PrintApply.cpp"
#undef protected
#undef private
#include "libslic3r/TriangleMesh.hpp"

using namespace Slic3r;

#pragma clang fp contract(off)

static size_t g_base = 0;

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
static std::string hI(long long v) { return std::to_string(v); }
static std::string hB(bool v) { return v ? "1" : "0"; }
static std::string ID(const ObjectID &id) { return id.id == 0 ? "-1" : hI((long long) id.id - (long long) g_base); }
static std::string hM(const Transform3d &t)
{
    std::string o;
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 4; ++c) o += (o.empty() ? "" : " ") + hD(t.matrix()(r, c));
    return o;
}
template<typename T> static std::string VI(const std::vector<T> &v)
{
    std::string o = hI((long long) v.size());
    for (auto x : v) o += " " + hI((long long) x);
    return o;
}
static std::string BOX(const PrintObjectRegions::BoundingBox &b)
{
    return hF(b.min().x()) + " " + hF(b.min().y()) + " " + hF(b.min().z()) + " " + hF(b.max().x()) + " " + hF(b.max().y()) + " " +
           hF(b.max().z());
}
static void out(const std::string &s) { std::printf("%s\n", s.c_str()); }

static indexed_triangle_set box_its(double sx, double sy, double sz, float ox, float oy, float oz)
{
    indexed_triangle_set its = its_make_cube(sx, sy, sz);
    for (Vec3f &v : its.vertices) v += Vec3f(ox, oy, oz);
    return its;
}

// The print config of a scenario: the defaults of every option, three filaments, then the keys.
static DynamicPrintConfig base_config(int scenario)
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_num_filaments(3);
    config.option<ConfigOptionStrings>("filament_colour", true)->values = {"#FF0000", "#00FF00", "#0000FF"};
    config.option<ConfigOptionInts>("filament_map", true)->values = {1, 1, 1};
    if (scenario == 1) {
        config.set_key_value("skirt_loops", new ConfigOptionInt(2));
        config.set_key_value("skirt_height", new ConfigOptionInt(1));
        config.set_key_value("brim_type", new ConfigOptionEnum<BrimType>(btOuterOnly));
        config.set_key_value("brim_width", new ConfigOptionFloat(5.));
    }
    if (scenario == 2) {
        config.set_key_value("draft_shield", new ConfigOptionEnum<DraftShield>(dsEnabled));
        config.set_key_value("skirt_loops", new ConfigOptionInt(1));
        config.set_key_value("ooze_prevention", new ConfigOptionBool(true));
        config.set_key_value("enable_prime_tower", new ConfigOptionBool(true));
        config.set_key_value("wall_filament", new ConfigOptionInt(2));
        config.set_key_value("sparse_infill_density", new ConfigOptionPercent(15.));
    }
    if (scenario == 3) {
        config.set_key_value("enable_support", new ConfigOptionBool(true));
        config.set_key_value("support_filament", new ConfigOptionInt(3));
        config.set_key_value("support_interface_filament", new ConfigOptionInt(0));
        config.set_key_value("brim_type", new ConfigOptionEnum<BrimType>(btNoBrim));
        config.set_key_value("timelapse_type", new ConfigOptionEnum<TimelapseType>(tlSmooth));
        config.set_key_value("enable_prime_tower", new ConfigOptionBool(true));
        config.set_key_value("print_sequence", new ConfigOptionEnum<PrintSequence>(PrintSequence::ByObject));
    }
    return config;
}

static Model build_model(int scenario)
{
    Model model;
    g_base = model.id().id;
    if (scenario == 0 || scenario == 1) {
        ModelObject *o = model.add_object();
        o->name = "cube";
        o->add_volume(TriangleMesh(box_its(20, 20, 20, -10.f, -10.f, 0.f)));
        o->add_instance()->set_offset(Vec3d(100., 100., 0.));
        o->add_instance()->set_offset(Vec3d(140.5, 90.25, 0.));
    }
    if (scenario == 1 || scenario == 2) {
        ModelObject *o = model.add_object();
        o->name = "two";
        o->config.set("extruder", 2);
        ModelVolume *v0 = o->add_volume(TriangleMesh(box_its(10, 12, 8, -5.f, -6.f, 0.f)));
        v0->config.set("wall_filament", 3);
        ModelVolume *v1 = o->add_volume(TriangleMesh(box_its(4, 4, 4, 0.f, 0.f, 2.f)), ModelVolumeType::PARAMETER_MODIFIER);
        v1->config.set("sparse_infill_filament", 1);
        v1->set_offset(Vec3d(1., -1., 0.5));
        ModelVolume *v2 = o->add_volume(TriangleMesh(box_its(3, 3, 3, 20.f, 20.f, 0.f)), ModelVolumeType::PARAMETER_MODIFIER);
        v2->config.set("wall_loops", 5);
        o->add_volume(TriangleMesh(box_its(2, 2, 10, -1.f, -1.f, 0.f)), ModelVolumeType::NEGATIVE_VOLUME);
        o->add_instance()->set_offset(Vec3d(60., 60., 0.));
        ModelInstance *i1 = o->add_instance();
        i1->set_offset(Vec3d(80., 40., 0.));
        i1->set_rotation(Vec3d(0., 0., 0.5));
        ModelInstance *i2 = o->add_instance();
        i2->set_offset(Vec3d(30., 120., 0.));
    }
    if (scenario == 2 || scenario == 3) {
        ModelObject *o = model.add_object();
        o->name = "ranges";
        o->add_volume(TriangleMesh(box_its(15, 15, 30, -7.5f, -7.5f, 0.f)));
        o->layer_config_ranges[{2.0, 6.0}].set("extruder", 3);
        o->layer_config_ranges[{2.0, 6.0}].set("layer_height", 0.2);
        o->layer_config_ranges[{10.0, 14.0}].set("wall_filament", 2);
        o->layer_config_ranges[{10.0, 14.0}].set("layer_height", 0.2);
        o->add_instance()->set_offset(Vec3d(150., 50., 0.));
    }
    if (scenario == 3) {
        ModelObject *o = model.add_object();
        o->name = "support";
        o->config.set("support_interface_filament", 2);
        o->add_volume(TriangleMesh(box_its(8, 8, 8, -4.f, -4.f, 0.f)));
        o->add_instance()->set_offset(Vec3d(50., 150., 0.));
    }
    return model;
}

static void dump_print(int scenario, const Print &print, const Model &model)
{
    std::string sc = "s" + std::to_string(scenario);
    out(sc + " objects " + hI(print.objects().size()) + " regions " + hI(print.num_print_regions()));
    for (size_t io = 0; io < print.objects().size(); ++io) {
        const PrintObject *po = print.objects()[io];
        const ModelObject *mo = po->model_object();
        size_t mo_idx = std::find(print.model().objects.begin(), print.model().objects.end(), mo) - print.model().objects.begin();
        std::string p = sc + " o" + hI(io);
        out(p + " model " + hI(mo_idx) + " trafo " + hM(po->trafo()));
        std::string inst;
        for (const PrintInstance &pi : po->instances()) {
            size_t mi = std::find(mo->instances.begin(), mo->instances.end(), pi.model_instance) - mo->instances.begin();
            inst += " " + hI(mi) + " " + hI(pi.shift.x()) + " " + hI(pi.shift.y());
        }
        out(p + " instances " + hI(po->instances().size()) + inst);
        const PrintObjectConfig &oc = po->config();
        out(p + " config " + hI(oc.support_filament.value) + " " + hI(oc.support_interface_filament.value) + " " + hB(oc.enable_support.value) +
            " " + hI(oc.raft_layers.value) + " " + hI(oc.brim_type.value) + " " + hD(oc.brim_width.value) + " " + hB(po->has_brim()) + " " +
            hB(po->has_support_material()));
        const PrintObjectRegions *r = po->shared_regions();
        out(p + " all_regions " + hI(r->all_regions.size()) + " layer_ranges " + hI(r->layer_ranges.size()) + " cached " +
            hI(r->cached_volume_ids.size()));
        for (size_t ir = 0; ir < r->all_regions.size(); ++ir) {
            const PrintRegion &reg = *r->all_regions[ir];
            const PrintRegionConfig &rc = reg.config();
            out(p + " region " + hI(ir) + " " + hI(rc.wall_filament.value) + " " + hI(rc.sparse_infill_filament.value) + " " +
                hI(rc.solid_infill_filament.value) + " " + hI(rc.wall_loops.value) + " " + hD(rc.sparse_infill_density.value) + " " +
                hI(reg.print_object_region_id()) + " " + hI(reg.print_region_id()) + " " + hB(reg.gradient_volume_id().valid()));
            out(p + " region_hash " + hI(ir) + " " + hI((long long) reg.config_hash()));
        }
        for (size_t il = 0; il < r->layer_ranges.size(); ++il) {
            const auto &lr = r->layer_ranges[il];
            std::string l = p + " lr" + hI(il);
            out(l + " range " + hD(lr.layer_height_range.first) + " " + hD(lr.layer_height_range.second) + " " + hB(lr.config != nullptr));
            for (const auto &ve : lr.volumes) out(l + " extents " + ID(ve.volume_id) + " " + BOX(ve.bbox));
            for (const auto &vr : lr.volume_regions) {
                size_t vi = std::find(mo->volumes.begin(), mo->volumes.end(), vr.model_volume) - mo->volumes.begin();
                out(l + " vregion " + hI(vi) + " " + hI(vr.parent) + " " + hI(vr.region ? vr.region->print_object_region_id() : -1));
            }
            for (const auto &pr : lr.painted_regions)
                out(l + " pregion " + hI(pr.extruder_id) + " " + hI(pr.parent) + " " + hI(pr.region->print_object_region_id()));
        }
        out(p + " object_extruders " + VI(po->object_extruders()));
    }
    out(sc + " extruders " + VI(print.extruders()) + " custom " + VI(print.extruders(true)));
    out(sc + " object_extruders " + VI(print.object_extruders()));
    out(sc + " support_material_extruders " + VI(print.support_material_extruders()));
    out(sc + " flags " + hB(print.has_skirt()) + " " + hB(print.has_infinite_skirt()) + " " + hB(print.has_brim()) + " " +
        hB(print.has_auto_brim()) + " " + hB(print.has_support_material()) + " " + hB(print.is_sequential_print()) + " " +
        hB(print.has_wipe_tower()) + " " + hB(print.enable_timelapse_print()) + " " + hB(print.uses_multiple_nozzle_diameters()) + " " +
        hB(print.support_material_on_wipe_tower()) + " " + hB(print.is_dynamic_group_reorder()) + " " + hB(print.empty()));
    out(sc + " num_object_instances " + hI(print.num_object_instances()) + " max_allowed_layer_height " + hD(print.max_allowed_layer_height()) +
        " skirt_first_layer_height " + hD(print.skirt_first_layer_height()));
    std::vector<int> usage;
    for (auto t : print.get_filament_usage_type()) usage.push_back(int(t));
    out(sc + " usage " + VI(usage));
    out(sc + " filament_maps " + VI(print.get_filament_maps()) + " volume_maps " + VI(print.get_filament_volume_maps()) + " mode " +
        hI(int(print.get_filament_map_mode())));
    std::vector<int> modes;
    for (auto m : print.get_available_filament_map_modes()) modes.push_back(int(m));
    out(sc + " map_modes " + VI(modes));
    out(sc + " extruder_id " + hI(print.get_extruder_id(0)) + " " + hI(print.get_extruder_id(2)) + " " + hI(print.get_extruder_id(7)));
    std::vector<unsigned int> used = {0, 1, 2};
    auto pu = print.get_physical_unprintable_filaments(used);
    out(sc + " physical_unprintable " + hI(pu.size()));
    std::vector<size_t> rids;
    for (size_t i = 0; i < print.num_print_regions(); ++i) rids.push_back(print.get_print_region(i).print_region_id());
    out(sc + " print_region_ids " + VI(rids));
}

static void dump_helpers(int scenario, const Model &model, const DynamicPrintConfig &config)
{
    std::string sc = "s" + std::to_string(scenario);
    for (size_t im = 0; im < model.objects.size(); ++im) {
        const ModelObject &mo = *model.objects[im];
        std::string p = sc + " m" + hI(im);
        auto trafos = print_objects_from_model_object(mo);
        out(p + " trafos " + hI(trafos.size()));
        for (const auto &t : trafos) {
            std::string inst;
            for (const PrintInstance &pi : t.instances) {
                size_t mi = std::find(mo.instances.begin(), mo.instances.end(), pi.model_instance) - mo.instances.begin();
                inst += " " + hI(mi) + " " + hI(pi.shift.x()) + " " + hI(pi.shift.y());
            }
            out(p + " trafo " + hM(t.trafo) + " " + hI(t.instances.size()) + inst);
        }
        LayerRanges lr(mo.layer_config_ranges);
        std::string rs;
        for (const auto &r : lr) rs += " " + hD(r.layer_height_range.first) + " " + hD(r.layer_height_range.second) + " " + hB(r.config != nullptr);
        out(p + " layer_ranges " + hI(lr.size()) + rs);
        out(p + " lr_config " + hB(lr.config({2.0, 6.0}) != nullptr) + " " + hB(lr.config({0.0, 2.0}) != nullptr) + " " +
            hB(lr.config({6.0, 10.0}) != nullptr));
        for (size_t iv = 0; iv < mo.volumes.size(); ++iv) {
            const ModelVolume &v = *mo.volumes[iv];
            out(p + " v" + hI(iv) + " solid_or_modifier " + hB(model_volume_solid_or_modifier(v)));
        }
        out(p + " lh_equal " + hB(layer_height_ranges_equal(mo.layer_config_ranges, mo.layer_config_ranges, true)) + " " +
            hB(layer_height_ranges_equal(mo.layer_config_ranges, t_layer_config_ranges{}, false)));
    }
    // print_config_diffs / full_print_config_diffs between the config and a changed copy
    DynamicPrintConfig changed = config;
    changed.set_key_value("skirt_loops", new ConfigOptionInt(7));
    changed.set_key_value("nozzle_temperature", new ConfigOptionInts({222, 223, 224}));
    changed.set_key_value("wipe_tower_x", new ConfigOptionFloats({10., 20.}));
    PrintConfig pc;
    pc.apply(config, true);
    DynamicPrintConfig overrides;
    std::vector<int> fmaps = {1, 1, 1};
    t_config_option_keys d1 = print_config_diffs(pc, changed, overrides, 0, fmaps);
    std::string s1;
    for (auto &k : d1) s1 += " " + k;
    out(sc + " print_config_diffs " + hI(d1.size()) + s1);
    t_config_option_keys d2 = full_print_config_diffs(config, changed, 0);
    std::string s2;
    for (auto &k : d2) s2 += " " + k;
    out(sc + " full_print_config_diffs " + hI(d2.size()) + s2);
    out(sc + " same_ams " + hB(same_ams_grouping({1, 2, 3}, {1, 4, 3}, 2)) + " " + hB(same_ams_grouping({1, 2, 3}, {2, 2, 3}, 2)) + " " +
        hB(same_ams_grouping({1, 2}, {1, 2, 3}, 2)));
}


// verify_update_print_object_regions over the regions Print::apply made, then after a modifier's
// wall_loops changed.
static void dump_verify(int scenario, Print &print)
{
    std::string sc = "s" + std::to_string(scenario);
    size_t num_extruders = print.config().filament_diameter.size();
    for (size_t io = 0; io < print.m_objects.size(); ++io) {
        PrintObject *po = print.m_objects[io];
        if (io > 0 && print.m_objects[io - 1]->m_shared_regions == po->m_shared_regions)
            continue;
        ModelObject *mo = po->m_model_object;
        int calls = 0;
        std::vector<int> vidx = {0};
        auto cb = [&calls](const PrintRegionConfig &, const PrintRegionConfig &, const t_config_option_keys &k) { calls += int(k.size()); };
        bool ok = verify_update_print_object_regions(mo->volumes, print.m_default_region_config, num_extruders, {}, *po->m_shared_regions, cb, vidx);
        std::string refs;
        for (auto &r : po->m_shared_regions->all_regions) refs += " " + hI(print_region_ref_cnt(*r));
        out(sc + " verify o" + hI(io) + " " + hB(ok) + " " + hI(calls) + refs);
        for (ModelVolume *v : mo->volumes)
            if (v->is_modifier()) {
                v->config.set("wall_loops", 6);
                break;
            }
        calls = 0;
        ok = verify_update_print_object_regions(mo->volumes, print.m_default_region_config, num_extruders, {}, *po->m_shared_regions, cb, vidx);
        std::string wl;
        for (auto &r : po->m_shared_regions->all_regions) wl += " " + hI(r->config().wall_loops.value) + " " + hI(print_region_ref_cnt(*r));
        out(sc + " verify2 o" + hI(io) + " " + hB(ok) + " " + hI(calls) + wl);
    }
}

int main()
{
    for (int scenario = 0; scenario < 4; ++scenario) {
        Model model = build_model(scenario);
        DynamicPrintConfig config = base_config(scenario);
        dump_helpers(scenario, model, config);
        Print print;
        print.apply(model, config);
        dump_print(scenario, print, model);
        dump_verify(scenario, print);
    }
    return 0;
}
