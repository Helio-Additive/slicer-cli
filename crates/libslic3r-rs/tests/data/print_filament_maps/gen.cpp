// Generator of the vectors of tests/print_filament_maps_native_parity.rs: Print::update_filament_maps_to_config
// (Print.cpp:3241-3320) of libslic3r at the pin (references/BambuStudio/src/libslic3r), at Print level.
//
// Build with the compile and link line the Bambu Studio engine build uses for a libslic3r source (its
// defines and include directories, the override layer $REPO/libslic3r/bambustudio before the pin,
// -std=c++17 -O3 -DNDEBUG, the forced include of cgal_54_compat.hpp), linked with the build's
// libslic3r_core and the libraries it links. Run:
//
//   ./gen <project_settings.config of a two-nozzle job> <output> && gzip -9n <output>
//
// Three configurations: 0 the job's project settings (ConfigBase::load_from_json, EnableSilent); 1 the
// same with two variants per filament ("Direct Drive Standard" / "Direct Drive High Flow": every
// option of filament_options_with_variant the settings hold doubled, a few made distinct, filament
// retraction overrides); 2 the same as 0 under fmmNozzleManual with two nozzles on the first extruder
// and a filament_nozzle_map of group keys.
// Each is applied to a Print with an empty Model (Print::apply, PrintApply.cpp:1295-). The state
// Print::apply leaves is written once per configuration ("opt"); then for every scenario a fresh Print
// gets the same apply and the call, and the options that changed are written ("post"). The functions:
// update_filament_maps_to_config (Print.cpp:3241-3320), update_to_config_by_nozzle_group_result
// (:3347-3400, with a NozzleGroupResultBase whose get_nozzles_for_filament returns a fixed table) and
// resolve_manual_nozzle_map (:3322-3345). Private members are reached by including the headers with
// `private` and `protected` defined as `public`.
//
// Lines (tokens separated by one space; a text is the hexadecimal of its bytes, "-" when empty):
//   opt <cfg> <which> <key> <value> which: config (m_config, the static PrintConfig), ori
//                                   (m_ori_full_print_config), full (m_full_print_config), pp (the
//                                   placeholder parser's own config); value: "s" and the serialization,
//                                   or "i" <n> <ints> for an enum vector without keys map (the static
//                                   class's members: PrintConfig.hpp:880-897), "!" and what() if it throws
//   scenario <id> <cfg> maps <n> <f_maps>*n <m> <f_volume_maps>*m <k> <f_nozzle_maps>*k
//   scenario <id> <cfg> group <n> (<m> (<extruder_id> <volume_type>)*m)*n   the nozzles of filament 0..n-1
//   scenario <id> <cfg> manual <n> <used_filaments>*n
//   result <id> <ok | !what>        what the call threw
//   pre_self_index <cfg> <n> <ints> m_filament_self_index as Print::apply leaves it
//   post <id> <which> <key> <value> an option whose value differs from the "pre" state after the call
//   gone <id> <which> <key>         an option the call removed
//   self_index <id> <n> <ints>      m_filament_self_index after the call
//   auto <id> <0|1>                 m_has_auto_filament_map_result after the call

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#define private public
#define protected public
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/MultiNozzleUtils.hpp"
#undef private
#undef protected

using namespace Slic3r;

static FILE *g_out = nullptr;

static std::string hex(const std::string &s)
{
    if (s.empty()) return "-";
    static const char *d = "0123456789abcdef";
    std::string        o;
    for (unsigned char c : s) {
        o += d[c >> 4];
        o += d[c & 15];
    }
    return o;
}

static std::string value_of(const ConfigOption *o)
{
    const t_config_enum_values *km      = reinterpret_cast<const t_config_enum_values *>(1);
    const std::vector<int>     *vals    = nullptr;
    if (auto *a = dynamic_cast<const ConfigOptionEnumsGeneric *>(o)) { km = a->keys_map; vals = &a->values; }
    else if (auto *b = dynamic_cast<const ConfigOptionEnumsGenericNullable *>(o)) { km = b->keys_map; vals = &b->values; }
    if (vals && km == nullptr) {
        std::string s = "i " + std::to_string(vals->size());
        for (int v : *vals) s += " " + std::to_string(v);
        return s;
    }
    try {
        return "s " + hex(o->serialize());
    } catch (const std::exception &e) {
        return "! " + hex(e.what());
    }
}

typedef std::map<std::string, std::string> Values;

static Values values_of(const ConfigBase &cfg)
{
    Values v;
    for (const std::string &k : cfg.keys()) v[k] = value_of(cfg.option(k));
    return v;
}

static std::map<std::string, Values> state_of(Print &print)
{
    std::map<std::string, Values> s;
    s["config"] = values_of(print.m_config);
    s["ori"]    = values_of(print.m_ori_full_print_config);
    s["full"]   = values_of(print.m_full_print_config);
    s["pp"]     = values_of(print.m_placeholder_parser.config());
    return s;
}

static std::string ints(const std::vector<int> &v)
{
    std::string s = std::to_string(v.size());
    for (int x : v) s += " " + std::to_string(x);
    return s;
}

struct FixedNozzles : public MultiNozzleUtils::NozzleGroupResultBase
{
    std::vector<std::vector<MultiNozzleUtils::NozzleInfo>> table;
    std::optional<MultiNozzleUtils::NozzleInfo> get_nozzle_from_id(int) const override { return std::nullopt; }
    std::optional<MultiNozzleUtils::NozzleInfo> get_first_nozzle_for_filament(int) const override { return std::nullopt; }
    std::vector<MultiNozzleUtils::NozzleInfo>   get_nozzles_for_filament(int filament_id) const override
    {
        return filament_id < int(table.size()) ? table[filament_id] : std::vector<MultiNozzleUtils::NozzleInfo>();
    }
    int                                       get_extruder_count() const override { return 2; }
    std::vector<MultiNozzleUtils::NozzleInfo> get_used_nozzles_in_extruder(int) const override { return {}; }
    std::vector<int>                          get_used_extruders() const override { return {}; }
    std::vector<unsigned int>                 get_used_filaments() const override { return {}; }
};

int main(int argc, char **argv)
{
    if (argc < 3) {
        fprintf(stderr, "usage: gen <project_settings.config> <output>\n");
        return 1;
    }
    DynamicPrintConfig config;
    {
        ConfigSubstitutionContext          ctxt(ForwardCompatibilitySubstitutionRule::EnableSilent);
        std::map<std::string, std::string> key_values;
        std::string                        reason;
        if (config.load_from_json(argv[1], ctxt, true, key_values, reason) != 0) {
            fprintf(stderr, "load_from_json failed: %s\n", reason.c_str());
            return 1;
        }
    }
    std::vector<DynamicPrintConfig> configs(3, config);
    {
        // 1: two variants per filament
        DynamicPrintConfig &c = configs[1];
        for (const std::string &key : filament_options_with_variant) {
            ConfigOption *o = c.option(key);
            if (o == nullptr) continue;
            auto *vb = dynamic_cast<ConfigOptionVectorBase *>(o);
            if (vb == nullptr || vb->size() == 0) continue;
            std::unique_ptr<ConfigOption> orig(o->clone());
            size_t n = vb->size();
            vb->resize(2 * n, orig.get());
            for (size_t i = 0; i < 2 * n; ++i) vb->set_at(orig.get(), i, i / 2);
        }
        std::string variants, self_index, mvs, flow;
        for (int i = 0; i < 8; ++i) {
            variants += std::string(i ? ";" : "") + "\"Direct Drive Standard\";\"Direct Drive High Flow\"";
            self_index += std::string(i ? "," : "") + std::to_string(i + 1) + "," + std::to_string(i + 1);
            mvs += std::string(i ? "," : "") + std::to_string(10 + 2 * i) + "," + std::to_string(11 + 2 * i);
            flow += std::string(i ? "," : "") + "0.9" + std::to_string(i) + ",0.8" + std::to_string(i);
        }
        c.set_deserialize_strict("filament_extruder_variant", variants);
        c.set_deserialize_strict("filament_self_index", self_index);
        c.set_deserialize_strict("filament_max_volumetric_speed", mvs);
        c.set_deserialize_strict("filament_flow_ratio", flow);
        c.set_deserialize_strict("filament_retraction_length", "nil,nil,nil,nil,1.1,1.6,nil,nil,nil,nil,0.5,0.7,nil,nil,nil,nil");
        c.set_deserialize_strict("filament_z_hop", "nil,nil,0.3,0.5,nil,nil,nil,nil,nil,nil,nil,nil,nil,nil,0.2,0.25");
    }
    {
        // 2: fmmNozzleManual, two nozzles on the first extruder
        DynamicPrintConfig &c = configs[2];
        c.set_deserialize_strict("filament_map_mode", "Nozzle Manual");
        c.set_deserialize_strict("extruder_max_nozzle_count", "2,1");
        c.set_deserialize_strict("extruder_nozzle_count", "2,1");
        c.set_deserialize_strict("extruder_nozzle_volume_type", "Standard,Standard,Standard");
        c.set_deserialize_strict("filament_map", "1,1,2,1,1,2,2,1");
        c.set_deserialize_strict("filament_volume_map", "0,0,0,0,0,0,0,0");
        // group keys (5, 6, 7), resolved to nozzle indices by resolve_manual_nozzle_map_to_index
        c.set_deserialize_strict("filament_nozzle_map", "5,6,7,5,6,7,7,5");
    }
    g_out = fopen(argv[2], "wb");
    if (!g_out) {
        perror(argv[2]);
        return 1;
    }
    Model model;
    std::vector<std::map<std::string, Values>> pre(configs.size());
    for (size_t ci = 0; ci < configs.size(); ++ci) {
        Print print;
        print.apply(model, configs[ci]);
        pre[ci] = state_of(print);
        for (const auto &w : pre[ci])
            for (const auto &kv : w.second)
                fprintf(g_out, "opt %zu %s %s %s\n", ci, w.first.c_str(), hex(kv.first).c_str(), kv.second.c_str());
        fprintf(g_out, "pre_self_index %zu %s\n", ci, ints(print.m_filament_self_index).c_str());
    }
    using MultiNozzleUtils::NozzleInfo;
    auto nz = [](int extruder_id, NozzleVolumeType vt) {
        NozzleInfo n;
        n.extruder_id = extruder_id;
        n.volume_type = vt;
        return n;
    };
    struct Scenario { int cfg; std::string fn; std::vector<std::vector<int>> maps; std::vector<std::vector<NozzleInfo>> table; std::vector<unsigned int> used; };
    // maps: (f_maps, f_volume_maps, f_nozzle_maps): the H2D job's call (ToolOrdering.cpp:3609-3614, the
    // header of slicer_cli's G-code), the volume and nozzle maps left empty, the maps unchanged, a
    // volume map with High Flow.
    const std::vector<Scenario> scenarios = {
        {0, "maps", {{1, 2, 2, 1, 2, 2, 2, 1}, {0, 0, 0, 0, 0, 0, 0, 0}, {0, 1, 1, 0, 1, 1, 1, 0}}, {}, {}},
        {0, "maps", {{2, 1, 1, 2, 1, 1, 1, 2}, {}, {}}, {}, {}},
        {0, "maps", {{1, 1, 1, 1, 1, 1, 1, 1}, {}, {}}, {}, {}},
        {0, "maps", {{2, 2, 1, 1, 2, 2, 1, 1}, {1, 0, 1, 0, 0, 1, 0, 1}, {1, 1, 0, 0, 1, 1, 0, 0}}, {}, {}},
        {1, "maps", {{1, 2, 2, 1, 2, 2, 2, 1}, {0, 0, 0, 0, 0, 0, 0, 0}, {0, 1, 1, 0, 1, 1, 1, 0}}, {}, {}},
        {1, "maps", {{2, 2, 1, 1, 2, 2, 1, 1}, {1, 0, 1, 0, 0, 1, 0, 1}, {1, 1, 0, 0, 1, 1, 0, 0}}, {}, {}},
        {1, "maps", {{2, 1, 1, 2, 1, 1, 1, 2}, {}, {}}, {}, {}},
        {0, "group", {}, {{nz(0, nvtStandard)}, {nz(1, nvtStandard)}, {nz(1, nvtStandard), nz(0, nvtStandard)}, {}, {nz(1, nvtHighFlow), nz(1, nvtStandard)}, {nz(0, nvtStandard)}, {nz(1, nvtStandard)}, {nz(0, nvtStandard)}}, {}},
        {1, "group", {}, {{nz(0, nvtStandard)}, {nz(1, nvtStandard)}, {nz(1, nvtStandard), nz(0, nvtStandard)}, {}, {nz(1, nvtHighFlow), nz(1, nvtStandard)}, {nz(0, nvtHighFlow)}, {nz(1, nvtStandard)}, {nz(0, nvtHighFlow), nz(0, nvtStandard)}}, {}},
        {0, "manual", {}, {}, {0, 1, 2, 3, 4, 5, 6, 7}},
        {2, "manual", {}, {}, {0, 1, 2, 3, 4, 5, 6, 7}},
        {2, "manual", {}, {}, {1, 3}},
        {2, "manual", {}, {}, {0, 2, 6}},
        {2, "manual", {}, {}, {9}},
    };
    for (size_t id = 0; id < scenarios.size(); ++id) {
        const Scenario &sc = scenarios[id];
        std::string     args;
        if (sc.fn == "maps")
            args = ints(sc.maps[0]) + " " + ints(sc.maps[1]) + " " + ints(sc.maps[2]);
        else if (sc.fn == "group") {
            args = std::to_string(sc.table.size());
            for (const auto &row : sc.table) {
                args += " " + std::to_string(row.size());
                for (const NozzleInfo &n : row) args += " " + std::to_string(n.extruder_id) + " " + std::to_string(int(n.volume_type));
            }
        } else {
            args = std::to_string(sc.used.size());
            for (unsigned int u : sc.used) args += " " + std::to_string(u);
        }
        fprintf(g_out, "scenario %zu %d %s %s\n", id, sc.cfg, sc.fn.c_str(), args.c_str());
        Print print;
        print.apply(model, configs[sc.cfg]);
        std::string result = "ok";
        try {
            if (sc.fn == "maps")
                print.update_filament_maps_to_config(sc.maps[0], sc.maps[1], sc.maps[2]);
            else if (sc.fn == "group") {
                FixedNozzles g;
                g.table = sc.table;
                print.update_to_config_by_nozzle_group_result(g);
            } else
                print.resolve_manual_nozzle_map(sc.used);
        } catch (const std::exception &e) {
            result = "!" + hex(e.what());
        }
        fprintf(g_out, "result %zu %s\n", id, result.c_str());
        std::map<std::string, Values> post = state_of(print);
        for (const auto &w : post) {
            const Values &before = pre[sc.cfg][w.first];
            for (const auto &kv : w.second) {
                auto it = before.find(kv.first);
                if (it == before.end() || it->second != kv.second)
                    fprintf(g_out, "post %zu %s %s %s\n", id, w.first.c_str(), hex(kv.first).c_str(), kv.second.c_str());
            }
            for (const auto &kv : before)
                if (w.second.find(kv.first) == w.second.end()) fprintf(g_out, "gone %zu %s %s\n", id, w.first.c_str(), hex(kv.first).c_str());
        }
        fprintf(g_out, "self_index %zu %s\n", id, ints(print.m_filament_self_index).c_str());
        fprintf(g_out, "auto %zu %d\n", id, int(print.m_has_auto_filament_map_result));
    }
    fclose(g_out);
    return 0;
}
