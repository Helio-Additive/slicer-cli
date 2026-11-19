// Generator of the vectors of tests/tool_ordering_native_parity.rs: libslic3r's GCode/ToolOrdering.cpp
// at the pin (references/BambuStudio/src/libslic3r), its free functions and the member functions of
// ToolOrdering, LayerTools and GroupReorder driven on synthetic layers, configurations and nozzle
// group results (no sliced Print).
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
//   ./gen 2>/dev/null | grep -v '^\[' > vectors.txt && gzip -9 -n -c vectors.txt > $DATA/vectors.txt.gz
// (the grouping writes log lines, which start with '[', to the standard output)
//
// ToolOrdering.cpp is included here with private and protected members made public, so that its
// file-static functions and the private members of ToolOrdering are reachable; its definitions
// replace those of the library's copy at link time. The run is deterministic (a fixed LCG).
//
// Line format: tokens separated by one space. A text token is the hexadecimal of its bytes ("-"
// for the empty text), a double its 16 hexadecimal digits, a float its 8, an integer is decimal.
// A list is its length then its items. Every line is
//   <op> <inputs...> => <outputs...>
// A configuration is "C" <n> then n pairs <key> <value>: the options set with
// set_deserialize_strict() on top of the defaults. Layers are "L" <n> then per layer
// <print_z> <has_object> <has_support> <extruders list>.

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <queue>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#define private public
#define protected public
#include "libslic3r/GCode/ToolOrdering.cpp"
#undef protected
#undef private

using namespace Slic3r;

// ---------------------------------------------------------------------------------------------
// Output
// ---------------------------------------------------------------------------------------------

static std::string out;

static void tok(const std::string &s)
{
    if (!out.empty() && out.back() != '\n')
        out += ' ';
    out += s;
}
static void ti(long long v) { tok(std::to_string(v)); }
static void td(double v)
{
    uint64_t b;
    std::memcpy(&b, &v, 8);
    char buf[32];
    snprintf(buf, sizeof(buf), "%016llx", (unsigned long long) b);
    tok(buf);
}
static void tf(float v)
{
    uint32_t b;
    std::memcpy(&b, &v, 4);
    char buf[16];
    snprintf(buf, sizeof(buf), "%08x", b);
    tok(buf);
}
static std::string hex(const std::string &s)
{
    if (s.empty())
        return "-";
    static const char *d = "0123456789abcdef";
    std::string r;
    for (unsigned char c : s) {
        r += d[c >> 4];
        r += d[c & 15];
    }
    return r;
}
static void ts(const std::string &s) { tok(hex(s)); }
template<class T> static void tlist_u(const std::vector<T> &v)
{
    ti((long long) v.size());
    for (auto x : v)
        ti((long long) x);
}
static void arrow() { tok("=>"); }
static void endl_() { out += '\n'; fputs(out.c_str(), stdout); fflush(stdout); out.clear(); }

// ---------------------------------------------------------------------------------------------
// Deterministic random numbers
// ---------------------------------------------------------------------------------------------

static uint64_t g_seed = 0x9e3779b97f4a7c15ull;
static uint32_t rnd()
{
    g_seed = g_seed * 6364136223846793005ull + 1442695040888963407ull;
    return uint32_t(g_seed >> 33);
}
static int    ri(int lo, int hi) { return lo + int(rnd() % uint32_t(hi - lo + 1)); }
static double rd(double lo, double hi) { return lo + (hi - lo) * (double(rnd()) / 2147483648.0); }
static bool   rb(int pct = 50) { return ri(0, 99) < pct; }

// ---------------------------------------------------------------------------------------------
// Configurations
// ---------------------------------------------------------------------------------------------

typedef std::vector<std::pair<std::string, std::string>> Cfg;

static void write_cfg(const Cfg &c)
{
    tok("C");
    ti((long long) c.size());
    for (auto &kv : c) {
        ts(kv.first);
        ts(kv.second);
    }
}

// The options are set on a full DynamicPrintConfig (the enum vectors of a static configuration have no
// key map to deserialize with) and applied to a PrintConfig.
static DynamicPrintConfig make_dyn(const Cfg &c);

static PrintConfig make_cfg(const Cfg &c)
{
    DynamicPrintConfig dc = DynamicPrintConfig::full_print_config();
    for (auto &kv : c) {
        // The default value of an enum vector is built with the initializer_list constructor, which leaves
        // its key map uninitialized (Config.hpp:2135); give it the definition's before deserializing names.
        const ConfigOptionDef *def = print_config_def.get(kv.first);
        if (def != nullptr && def->type == coEnums)
            if (auto *o = dynamic_cast<ConfigOptionEnumsGeneric *>(dc.option(kv.first, true)))
                o->keys_map = def->enum_keys_map;
        dc.set_deserialize_strict(kv.first, kv.second);
    }
    PrintConfig pc;
    pc.apply(dc, true);
    return pc;
}

template<class T> static std::string join(const std::vector<T> &v)
{
    std::ostringstream ss;
    for (size_t i = 0; i < v.size(); ++i) {
        if (i)
            ss << ',';
        ss << v[i];
    }
    return ss.str();
}

static std::string join_d(const std::vector<double> &v)
{
    std::string s;
    char buf[64];
    for (size_t i = 0; i < v.size(); ++i) {
        if (i)
            s += ',';
        snprintf(buf, sizeof(buf), "%.6g", v[i]);
        s += buf;
    }
    return s;
}

// A string vector option in the form set_deserialize() reads (unescape_strings_cstyle): every item quoted,
// separated by ';'.
static std::string join_s(const std::vector<std::string> &v)
{
    std::string s;
    for (size_t i = 0; i < v.size(); ++i) {
        if (i)
            s += ';';
        s += "\"" + v[i] + "\"";
    }
    return s;
}

// ---------------------------------------------------------------------------------------------
// Layers
// ---------------------------------------------------------------------------------------------

static void write_layers(const std::vector<LayerTools> &lts)
{
    tok("L");
    ti((long long) lts.size());
    for (const LayerTools &lt : lts) {
        td(lt.print_z);
        ti(lt.has_object);
        ti(lt.has_support);
        tlist_u(lt.extruders);
    }
}

static void write_layers_full(const std::vector<LayerTools> &lts)
{
    ti((long long) lts.size());
    for (const LayerTools &lt : lts) {
        td(lt.print_z);
        tlist_u(lt.extruders);
        ti(lt.has_wipe_tower);
        ti((long long) lt.wipe_tower_partitions);
        td(lt.wipe_tower_layer_height);
        ti(lt.has_skirt);
    }
}

static std::vector<LayerTools> rand_layers(int n, int nfil, bool one_based, int empty_pct, int zero_pct)
{
    std::vector<LayerTools> lts;
    double z = rd(0.1, 0.3);
    for (int i = 0; i < n; ++i) {
        LayerTools lt(z);
        z += rb(10) ? rd(0.3, 0.9) : rd(0.08, 0.3);
        lt.has_object  = rb(80);
        lt.has_support = rb(20);
        if (!rb(empty_pct)) {
            int k = ri(1, std::min(nfil, 4));
            std::set<unsigned int> s;
            for (int j = 0; j < k; ++j)
                s.insert(one_based ? (rb(zero_pct) ? 0u : unsigned(ri(1, nfil))) : unsigned(ri(0, nfil - 1)));
            std::vector<unsigned int> v(s.begin(), s.end());
            if (!one_based && rb(50))
                std::reverse(v.begin(), v.end());
            lt.extruders = v;
        }
        lts.push_back(lt);
    }
    return lts;
}

// ---------------------------------------------------------------------------------------------
// The cases
// ---------------------------------------------------------------------------------------------

static void case_calc_max_layer_height()
{
    for (int it = 0; it < 300; ++it) {
        int n = ri(1, 3);
        std::vector<double> nd, mlh;
        for (int i = 0; i < n; ++i) {
            nd.push_back(std::vector<double>{0.2, 0.4, 0.6, 0.8}[ri(0, 3)] + (rb(10) ? rd(-0.05, 0.05) : 0.));
            mlh.push_back(rb(30) ? 0. : rd(0.1, 0.6));
        }
        Cfg c{{"nozzle_diameter", join_d(nd)}, {"max_layer_height", join_d(mlh)}};
        double moh = rb(30) ? 0. : rd(0.05, 0.8);
        PrintConfig pc = make_cfg(c);
        tok("calc_max_layer_height");
        write_cfg(c);
        td(moh);
        arrow();
        td(calc_max_layer_height(pc, moh));
        endl_();
    }
}

static void case_snap_to_simple_fraction()
{
    for (int it = 0; it < 500; ++it) {
        double r = rb(20) ? double(ri(0, 20)) / 20. : rd(-0.2, 1.2);
        int md = rb(80) ? 10 : ri(1, 16);
        tok("snap_to_simple_fraction");
        td(r);
        ti(md);
        arrow();
        td(snap_to_simple_fraction(r, md));
        endl_();
    }
}

static void case_get_filament_by_type()
{
    const std::vector<std::string> types{"PLA", "PETG", "ABS", "TPU", "PLA-CF"};
    for (int it = 0; it < 200; ++it) {
        int nf = ri(1, 6);
        std::vector<std::string> ft;
        for (int i = 0; i < nf; ++i)
            ft.push_back(types[ri(0, 4)]);
        Cfg c{{"filament_type", join_s(ft)}};
        std::vector<unsigned int> used;
        for (int i = 0; i < ri(0, 6); ++i)
            used.push_back(unsigned(ri(0, nf + 1)));
        std::string type = types[ri(0, 4)];
        PrintConfig pc = make_cfg(c);
        tok("get_filament_by_type");
        write_cfg(c);
        tlist_u(used);
        ts(type);
        arrow();
        std::set<int> r = get_filament_by_type(used, &pc, type);
        tlist_u(std::vector<int>(r.begin(), r.end()));
        endl_();
    }
}

static void case_check_filament_printable_after_group()
{
    const std::vector<std::string> variants{"Direct Drive Standard", "Direct Drive High Flow", "Bowden Standard"};
    for (int it = 0; it < 300; ++it) {
        int nf = ri(1, 5);
        int ne = ri(1, 2);
        std::vector<int> printable;
        for (int i = 0; i < nf; ++i)
            printable.push_back(rb(80) ? 3 : ri(0, 3));
        std::vector<double> nd(ne, 0.4);
        std::vector<std::string> pev;
        for (int i = 0; i < ne; ++i)
            pev.push_back(variants[ri(0, 2)]);
        Cfg c{{"filament_printable", join(printable)}, {"nozzle_diameter", join_d(nd)},
              {"printer_extruder_variant", join_s(pev)}};
        std::vector<int> maps;
        for (int i = 0; i < nf; ++i)
            maps.push_back(ri(0, ne - 1));
        std::vector<unsigned int> used;
        for (int i = 0; i < nf; ++i)
            if (rb(60))
                used.push_back(unsigned(i));
        std::unordered_map<int, std::vector<std::string>> fv;
        std::vector<std::pair<int, std::vector<std::string>>> fv_list;
        for (int i = 0; i < nf; ++i) {
            if (rb(20))
                continue;
            std::vector<std::string> v;
            for (auto &s : variants)
                if (rb(70))
                    v.push_back(s);
            fv[i] = v;
            fv_list.emplace_back(i, v);
        }
        PrintConfig pc = make_cfg(c);
        std::vector<FilamentUsageType> ut;
        tok("check_filament_printable_after_group");
        write_cfg(c);
        tlist_u(used);
        tlist_u(maps);
        ti((long long) fv_list.size());
        for (auto &kv : fv_list) {
            ti(kv.first);
            ti((long long) kv.second.size());
            for (auto &s : kv.second)
                ts(s);
        }
        arrow();
        try {
            bool r = check_filament_printable_after_group(used, maps, fv, ut, &pc);
            ti(r);
        } catch (const Slic3r::RuntimeError &e) {
            tok("!R");
            ts(e.what());
        }
        endl_();
    }
}

static std::vector<MultiNozzleUtils::NozzleInfo> rand_nozzles(int n_ext, int n_noz)
{
    std::vector<MultiNozzleUtils::NozzleInfo> list;
    for (int i = 0; i < n_noz; ++i) {
        MultiNozzleUtils::NozzleInfo ni;
        ni.diameter    = "0.4";
        ni.volume_type = NozzleVolumeType(ri(0, 1));
        ni.extruder_id = i < n_ext ? i : ri(0, n_ext - 1);
        ni.group_id    = i;
        list.push_back(ni);
    }
    return list;
}

static void write_nozzles(const std::vector<MultiNozzleUtils::NozzleInfo> &list)
{
    ti((long long) list.size());
    for (auto &n : list) {
        ts(n.diameter);
        ti((int) n.volume_type);
        ti(n.extruder_id);
        ti(n.group_id);
    }
}

static void case_calc_filament_change_info_by_toolorder()
{
    for (int it = 0; it < 300; ++it) {
        int nf = ri(1, 6), ne = ri(1, 2), nn = ne + (rb(30) ? ri(0, 2) : 0);
        auto nozzles = rand_nozzles(ne, nn);
        std::vector<int> map;
        for (int f = 0; f < nf; ++f)
            map.push_back(ri(0, nn - 1));
        std::vector<unsigned int> used;
        for (int f = 0; f < nf; ++f)
            used.push_back(unsigned(f));
        std::vector<GroupReorder::FlushMatrix> fm(ne);
        for (int e = 0; e < ne; ++e) {
            fm[e].assign(nf, std::vector<float>(nf, 0.f));
            for (int a = 0; a < nf; ++a)
                for (int b = 0; b < nf; ++b)
                    fm[e][a][b] = a == b ? 0.f : float(rd(0, 900));
        }
        std::vector<std::vector<unsigned int>> seqs;
        for (int l = 0; l < ri(0, 12); ++l) {
            std::vector<unsigned int> s;
            for (int k = 0; k < ri(0, 4); ++k)
                s.push_back(unsigned(ri(0, nf - 1)));
            seqs.push_back(s);
        }
        std::vector<double> dens;
        for (int f = 0; f < nf; ++f)
            dens.push_back(rd(0.9, 1.6));
        Cfg c{{"filament_density", join_d(dens)}};
        PrintConfig pc = make_cfg(c);
        auto gr = MultiNozzleUtils::LayeredNozzleGroupResult::create(map, nozzles, used, false);
        tok("calc_filament_change_info_by_toolorder");
        write_cfg(c);
        tlist_u(map);
        write_nozzles(nozzles);
        tlist_u(used);
        ti(ne);
        ti(nf);
        for (int e = 0; e < ne; ++e)
            for (int a = 0; a < nf; ++a)
                for (int b = 0; b < nf; ++b)
                    tf(fm[e][a][b]);
        ti((long long) seqs.size());
        for (auto &s : seqs)
            tlist_u(s);
        arrow();
        FilamentChangeStats st = calc_filament_change_info_by_toolorder(pc, *gr, fm, seqs);
        ti(st.filament_flush_weight);
        ti(st.flush_filament_change_count);
        ti(st.filament_change_count);
        endl_();
    }
}

static void case_handle_dontcare()
{
    for (int it = 0; it < 400; ++it) {
        int  nf  = ri(1, 5);
        auto lts = rand_layers(ri(0, 10), nf, true, 25, 25);
        // sort_remove_duplicates() puts a "don't care" first, as collect_extruders() leaves it.
        for (auto &lt : lts)
            std::sort(lt.extruders.begin(), lt.extruders.end());
        bool vec_overload = rb(50);
        if (vec_overload) {
            std::vector<unsigned int> order;
            for (int k = 0; k < ri(0, nf); ++k)
                order.push_back(unsigned(ri(1, nf)));
            ToolOrdering to{};
            to.m_layer_tools = lts;
            tok("handle_dontcare_extruder_vec");
            write_layers(lts);
            tlist_u(order);
            arrow();
            to.handle_dontcare_extruder(order);
            write_layers(to.m_layer_tools);
        } else {
            unsigned int last = rb(40) ? (unsigned int) -1 : unsigned(ri(0, nf - 1));
            bool         has_cfg = rb(60);
            std::vector<int> sol;
            for (int f = 0; f < nf; ++f)
                sol.push_back(rb(30));
            Cfg c{{"enable_prime_tower", rb(60) ? "1" : "0"}, {"filament_soluble", join(sol)}};
            PrintConfig pc = make_cfg(c);
            ToolOrdering to{};
            to.m_layer_tools = lts;
            if (has_cfg)
                to.m_print_config_ptr = &pc;
            tok("handle_dontcare_extruder_u32");
            write_layers(lts);
            ti(has_cfg);
            write_cfg(c);
            ti((long long) last);
            arrow();
            to.handle_dontcare_extruder(last);
            write_layers(to.m_layer_tools);
        }
        endl_();
    }
}

static void case_initialize_layers()
{
    for (int it = 0; it < 200; ++it) {
        std::vector<double> zs;
        for (int i = 0; i < ri(0, 20); ++i) {
            double z = rd(0.1, 5.0);
            zs.push_back(z);
            if (rb(30))
                zs.push_back(z + rd(-0.0002, 0.0002));
            if (rb(10))
                zs.push_back(z);
        }
        ToolOrdering to{};
        tok("initialize_layers");
        ti((long long) zs.size());
        for (double z : zs)
            td(z);
        arrow();
        std::vector<double> zz = zs;
        to.initialize_layers(zz);
        ti((long long) to.m_layer_tools.size());
        for (auto &lt : to.m_layer_tools)
            td(lt.print_z);
        endl_();
    }
}

static void case_fill_wipe_tower_partitions()
{
    for (int it = 0; it < 500; ++it) {
        int  nf  = ri(1, 4);
        auto lts = rand_layers(ri(1, 14), nf, false, 20, 0);
        Cfg  c{{"wrapping_detection_layers", std::to_string(rb(70) ? 0 : ri(1, 4))},
              {"enable_wrapping_detection", rb(50) ? "1" : "0"},
              {"timelapse_type", rb(80) ? "0" : "1"}};
        double bottom = rb(50) ? 0. : lts[std::min<size_t>(lts.size() - 1, size_t(ri(0, 3)))].print_z - rd(0., 0.1);
        double mlh    = rd(0.15, 0.6);
        // A raft gap is only planned below a layer with a wipe tower and filaments: keep the layers that
        // the insertion reads non-empty, as collect_extruders() and handle_dontcare_extruder() leave them.
        for (auto &lt : lts)
            if (lt.extruders.empty() && (lt.has_object || lt.print_z >= bottom))
                lt.extruders.push_back(0);
        PrintConfig  pc = make_cfg(c);
        ToolOrdering to{};
        to.m_layer_tools = lts;
        tok("fill_wipe_tower_partitions");
        write_cfg(c);
        write_layers(lts);
        td(bottom);
        td(mlh);
        arrow();
        to.fill_wipe_tower_partitions(pc, bottom, mlh);
        write_layers_full(to.m_layer_tools);
        endl_();
    }
}

static void case_mark_skirt_layers()
{
    for (int it = 0; it < 300; ++it) {
        int  nf  = ri(1, 4);
        auto lts = rand_layers(ri(1, 14), nf, false, 20, 0);
        if (!lts.empty() && rb(85) && lts.front().extruders.empty())
            lts.front().extruders.push_back(0);
        // every layer below an object layer that the skirt walks down to has filaments
        for (auto &lt : lts)
            if (lt.extruders.empty() && rb(50))
                lt.extruders.push_back(unsigned(ri(0, nf - 1)));
        double       mlh = rd(0.15, 0.6);
        ToolOrdering to{};
        to.m_layer_tools = lts;
        tok("mark_skirt_layers");
        write_layers(lts);
        td(mlh);
        arrow();
        // a skirt walking down onto an empty layer reads past the first one in the C++ (size_t wrap)
        bool walks_off = false;
        {
            size_t i = 0;
            if (!lts.front().extruders.empty()) {
                for (;;) {
                    size_t j = i + 1;
                    for (; j < lts.size() && !lts[j].has_object; ++j);
                    if (j == lts.size())
                        break;
                    double last_z = lts[i].print_z;
                    for (size_t k = i + 1; k < j; ++k)
                        if (lts[k + 1].print_z - last_z > mlh + EPSILON) {
                            size_t kk = k;
                            while (lts[kk].extruders.empty()) {
                                if (kk == 0) { walks_off = true; break; }
                                --kk;
                            }
                            break;
                        }
                    if (walks_off)
                        break;
                    i = j;
                }
            }
        }
        if (walks_off) {
            tok("skip");
        } else {
            to.mark_skirt_layers(PrintConfig(), mlh);
            write_layers_full(to.m_layer_tools);
        }
        endl_();
    }
}

static void case_collect_extruder_statistics()
{
    for (int it = 0; it < 200; ++it) {
        auto lts = rand_layers(ri(0, 10), 5, false, 30, 0);
        bool prime = rb(50);
        ToolOrdering to{};
        to.m_layer_tools = lts;
        tok("collect_extruder_statistics");
        write_layers(lts);
        ti(prime);
        arrow();
        to.collect_extruder_statistics(prime);
        ti((long long) to.m_first_printing_extruder);
        ti((long long) to.m_last_printing_extruder);
        tlist_u(to.m_all_printing_extruders);
        endl_();
    }
}

static void case_seed_first_tower_layer()
{
    for (int it = 0; it < 100; ++it) {
        auto lts = rand_layers(ri(0, 6), 4, false, 40, 0);
        for (auto &lt : lts)
            lt.has_wipe_tower = rb(40);
        unsigned int f = unsigned(ri(0, 3));
        ToolOrdering to{};
        to.m_layer_tools = lts;
        tok("seed_first_tower_layer");
        write_layers(lts);
        ti((long long) lts.size());
        for (auto &lt : lts)
            ti(lt.has_wipe_tower);
        ti(f);
        arrow();
        to.seed_first_tower_layer(f);
        write_layers(to.m_layer_tools);
        endl_();
    }
}

static void case_cal_max_additional_fan()
{
    for (int it = 0; it < 100; ++it) {
        int nf = ri(1, 5);
        std::vector<int> a, o;
        for (int f = 0; f < nf; ++f) {
            a.push_back(ri(0, 100));
            o.push_back(ri(0, 100));
        }
        Cfg c{{"additional_cooling_fan_speed", join(a)}, {"optional_cooling_fan_speed", join(o)}};
        auto lts = rand_layers(ri(0, 8), nf + 1, false, 20, 0);
        PrintConfig pc = make_cfg(c);
        ToolOrdering to{};
        to.m_layer_tools = lts;
        tok("cal_max_additional_fan");
        write_cfg(c);
        write_layers(lts);
        arrow();
        tf(to.cal_max_additional_fan(pc));
        endl_();
    }
}

static void case_cal_non_support_filaments()
{
    for (int it = 0; it < 300; ++it) {
        int nf = ri(1, 5), ne = ri(1, 2);
        std::vector<int> sup, fmap;
        for (int f = 0; f < nf; ++f) {
            sup.push_back(rb(30));
            fmap.push_back(ri(1, ne));
        }
        Cfg c{{"filament_is_support", join(sup)}, {"filament_map", rb(10) ? std::string() : join(fmap)}};
        auto lts = rand_layers(ri(0, 8), nf, false, 20, 0);
        PrintConfig pc = make_cfg(c);
        ToolOrdering to{};
        to.m_layer_tools = lts;
        to.collect_extruder_statistics(false);
        unsigned int first = (unsigned int) -1;
        std::vector<int> init_ns(ne, -1), init(ne, -1);
        tok("cal_non_support_filaments");
        write_cfg(c);
        write_layers(lts);
        ti(ne);
        arrow();
        bool hns = to.has_non_support_filament(pc);
        bool r   = to.cal_non_support_filaments(pc, first, init_ns, init);
        ti(hns);
        ti(r);
        ti((long long) first);
        tlist_u(init_ns);
        tlist_u(init);
        endl_();
    }
}

static void case_calc_most_used_extruder()
{
    for (int it = 0; it < 150; ++it) {
        int nf = ri(1, 5), ne = ri(1, 3);
        auto nozzles = rand_nozzles(ne, ne);
        std::vector<int> map;
        std::vector<unsigned int> used;
        for (int f = 0; f < nf; ++f) {
            map.push_back(ri(0, ne - 1));
            used.push_back(unsigned(f));
        }
        std::vector<double> nd(ne, 0.4);
        Cfg c{{"nozzle_diameter", join_d(nd)}};
        auto lts = rand_layers(ri(0, 10), nf, false, 20, 0);
        PrintConfig pc = make_cfg(c);
        Print       print;
        print.set_nozzle_group_result(std::make_shared<MultiNozzleUtils::LayeredNozzleGroupResult>(
            *MultiNozzleUtils::LayeredNozzleGroupResult::create(map, nozzles, used, false)));
        ToolOrdering to{};
        to.m_layer_tools = lts;
        to.m_print       = &print;
        to.m_most_used_extruder = -7;
        tok("calc_most_used_extruder");
        write_cfg(c);
        tlist_u(map);
        write_nozzles(nozzles);
        tlist_u(used);
        write_layers(lts);
        arrow();
        to.calc_most_used_extruder(pc);
        ti(to.m_most_used_extruder);
        endl_();
    }
}

static void case_prepare_flush_matrices()
{
    for (int it = 0; it < 150; ++it) {
        int ne = ri(1, 2), nf = ri(1, 5);
        std::vector<double> fvm;
        for (int e = 0; e < ne; ++e)
            for (int k = 0; k < nf * nf; ++k)
                fvm.push_back(double(ri(0, 900)) + (rb(30) ? 0.5 : 0.));
        std::vector<double> mult, multf;
        for (int e = 0; e < ri(0, ne); ++e)
            mult.push_back(rd(0.3, 2.0));
        for (int e = 0; e < ri(1, ne); ++e)
            multf.push_back(rd(0.3, 2.0));
        if (mult.empty())
            mult.push_back(1.0);
        std::vector<std::string> col(nf, "#FF0000");
        std::vector<double> nd(ne, 0.4);
        Cfg c{{"nozzle_diameter", join_d(nd)}, {"filament_colour", join_s(col)}, {"flush_volumes_matrix", join_d(fvm)},
              {"flush_multiplier", join_d(mult)}, {"flush_multiplier_fast", join_d(multf)},
              {"prime_volume_mode", std::vector<std::string>{"Default", "Saving", "Fast"}[ri(0, 2)]}};
        PrintConfig pc = make_cfg(c);
        tok("prepare_flush_matrices");
        write_cfg(c);
        arrow();
        auto m = GroupReorder::prepare_flush_matrices(pc);
        ti((long long) m.size());
        for (auto &mm : m) {
            ti((long long) mm.size());
            for (auto &row : mm) {
                ti((long long) row.size());
                for (float v : row)
                    tf(v);
            }
        }
        endl_();
    }
}

static void case_nozzle_lists()
{
    const std::vector<std::string> vt{"Standard", "High Flow"};
    for (int it = 0; it < 200; ++it) {
        int ne = ri(1, 2);
        std::vector<double> nd;
        std::vector<int> nvt, maxc;
        std::vector<std::string> stats;
        for (int e = 0; e < ne; ++e) {
            nd.push_back(std::vector<double>{0.2, 0.4, 0.6, 0.8}[ri(0, 3)]);
            nvt.push_back(rb(20) ? 2 : ri(0, 1));
            maxc.push_back(ri(1, 6));
            std::string s;
            if (rb(70)) {
                int k = ri(1, 2);
                for (int j = 0; j < k; ++j) {
                    if (j)
                        s += '|';
                    s += vt[j % 2] + "#" + std::to_string(ri(0, 4));
                }
            }
            stats.push_back(s);
        }
        std::vector<std::string> nvt_names;
        for (int v : nvt)
            nvt_names.push_back(std::vector<std::string>{"Standard", "High Flow", "Hybrid"}[v]);
        Cfg c{{"nozzle_diameter", join_d(nd)}, {"nozzle_volume_type", join(nvt_names)},
              {"extruder_max_nozzle_count", join(maxc)}};
        c.emplace_back("extruder_nozzle_stats", join_s(stats));
        PrintConfig pc = make_cfg(c);
        tok("nozzle_lists");
        write_cfg(c);
        ti(ne);
        arrow();
        auto groups = GroupReorder::build_nozzle_groups(pc, ne);
        ti((long long) groups.size());
        for (auto &g : groups) {
            ts(g.diameter);
            ti((int) g.volume_type);
            ti(g.extruder_id);
            ti(g.nozzle_count);
        }
        auto list = GroupReorder::build_default_nozzle_list(pc, ne);
        write_nozzles(list);
        endl_();
    }
}

static void case_create_custom_seq_function()
{
    for (int it = 0; it < 200; ++it) {
        int nseq = ri(0, 3);
        std::vector<int> flat;
        int item = ri(3, 5);
        for (int s = 0; s < nseq; ++s) {
            int a = ri(1, 8), b = a + ri(0, 6);
            flat.push_back(a);
            flat.push_back(b);
            for (int k = 2; k < item; ++k)
                flat.push_back(ri(1, 4));
        }
        Cfg c{{"other_layers_print_sequence_nums", std::to_string(nseq)},
              {"other_layers_print_sequence", flat.empty() ? std::string() : join(flat)}};
        bool include_first = rb(50);
        std::vector<unsigned int> first;
        for (int k = 0; k < ri(0, 3); ++k)
            first.push_back(unsigned(ri(0, 3)));
        PrintConfig pc = make_cfg(c);
        auto fn = GroupReorder::create_custom_seq_function(pc, include_first, first);
        tok("create_custom_seq_function");
        write_cfg(c);
        ti(include_first);
        tlist_u(first);
        arrow();
        for (int l = 0; l < 16; ++l) {
            std::vector<int> o{-5};
            bool r = fn(l, o);
            ti(r);
            tlist_u(o);
        }
        endl_();
    }
}

static void case_validate_manual_diameter()
{
    for (int it = 0; it < 200; ++it) {
        int ne = ri(1, 3), nf = ri(1, 5);
        std::vector<double> nd;
        for (int e = 0; e < ne; ++e)
            nd.push_back(std::vector<double>{0.2, 0.4, 0.6, 0.8}[ri(0, 3)]);
        std::vector<std::string> sel;
        for (int f = 0; f < nf; ++f)
            sel.push_back(rb(40) ? std::string() : std::vector<std::string>{"0.2", "0.4", "0.6", "0.8"}[ri(0, 3)]);
        Cfg c{{"nozzle_diameter", join_d(nd)}, {"filament_diameter_map", join_s(sel)},
              {"handle_hotend_as_extruder", rb(10) ? "1" : "0"}};
        std::vector<unsigned int> used;
        for (int f = 0; f < nf; ++f)
            if (rb(70))
                used.push_back(unsigned(f));
        std::vector<int> map;
        for (int f = 0; f < nf; ++f)
            map.push_back(ri(-1, ne));
        bool mode = rb(80);
        PrintConfig pc = make_cfg(c);
        tok("validate_manual_diameter_or_throw");
        write_cfg(c);
        tlist_u(used);
        tlist_u(map);
        ti(mode);
        arrow();
        try {
            validate_manual_diameter_or_throw(pc, used, map, mode);
            tok("ok");
        } catch (const Slic3r::RuntimeError &e) {
            tok("!R");
            ts(e.what());
        }
        endl_();
    }
}

static void write_coll(const ExtrusionEntityCollection &c, int depth);

static void write_entity(const ExtrusionEntity *e, int depth)
{
    if (auto *p = dynamic_cast<const ExtrusionPath *>(e)) {
        tok("P");
        ti((int) p->role());
        tf(p->height);
    } else if (auto *mp = dynamic_cast<const ExtrusionMultiPath *>(e)) {
        tok("M");
        ti((long long) mp->paths.size());
        for (auto &p : mp->paths) {
            ti((int) p.role());
            tf(p.height);
        }
    } else if (auto *lp = dynamic_cast<const ExtrusionLoop *>(e)) {
        tok("O");
        ti((long long) lp->paths.size());
        for (auto &p : lp->paths) {
            ti((int) p.role());
            tf(p.height);
        }
    } else if (auto *c = dynamic_cast<const ExtrusionEntityCollection *>(e)) {
        tok("C");
        write_coll(*c, depth + 1);
    }
}

static void write_coll(const ExtrusionEntityCollection &c, int depth)
{
    ti((long long) c.entities.size());
    for (auto *e : c.entities)
        write_entity(e, depth);
}

static ExtrusionRole rand_role()
{
    static const ExtrusionRole roles[] = {erPerimeter, erExternalPerimeter, erOverhangPerimeter, erInternalInfill,
                                          erSolidInfill, erTopSolidInfill, erBottomSurface, erBridgeInfill, erGapFill,
                                          erIroning, erSupportMaterial, erSkirt};
    return roles[ri(0, 11)];
}

static ExtrusionPath rand_path(bool single_role, ExtrusionRole r0)
{
    ExtrusionPath p(single_role ? r0 : rand_role(), 0.1, 0.42f, float(rd(0.04, 0.6)));
    return p;
}

static ExtrusionEntityCollection rand_coll(int depth)
{
    ExtrusionEntityCollection c;
    bool          single = rb(40);
    ExtrusionRole r0     = rand_role();
    int           n      = ri(0, 4);
    for (int i = 0; i < n; ++i) {
        int kind = ri(0, depth < 2 ? 3 : 2);
        if (kind == 0)
            c.append(rand_path(single, r0));
        else if (kind == 1) {
            ExtrusionMultiPath mp;
            for (int k = 0; k < ri(0, 3); ++k)
                mp.paths.push_back(rand_path(single, r0));
            c.append(mp);
        } else if (kind == 2) {
            ExtrusionPaths ps;
            for (int k = 0; k < ri(0, 3); ++k)
                ps.push_back(rand_path(single, r0));
            c.append(ExtrusionLoop(std::move(ps)));
        } else
            c.append(rand_coll(depth + 1));
    }
    return c;
}

static void case_slots_and_bed()
{
    for (int it = 0; it < 400; ++it) {
        ExtrusionEntityCollection c = rand_coll(0);
        PrintRegionConfig         rc;
        rc.wall_filament.value          = ri(1, 4);
        rc.sparse_infill_filament.value = ri(1, 4);
        rc.solid_infill_filament.value  = ri(1, 4);
        PrintRegion  region(rc);
        LayerTools   lt(1.0);
        lt.extruder_override = rb(70) ? 0 : unsigned(ri(1, 4));
        int nres = ri(0, 2);
        for (int k = 0; k < nres; ++k)
            lt.mixed_filament_resolution[unsigned(ri(0, 4))] = unsigned(ri(0, 4));
        double print_z = rd(0.05, 0.7);
        tok("slots_and_bed");
        write_coll(c, 0);
        ti(rc.wall_filament.value);
        ti(rc.sparse_infill_filament.value);
        ti(rc.solid_infill_filament.value);
        ti(lt.extruder_override);
        ti((long long) lt.mixed_filament_resolution.size());
        for (auto &kv : lt.mixed_filament_resolution) {
            ti(kv.first);
            ti(kv.second);
        }
        td(print_z);
        arrow();
        ti((int) c.role());
        ti(LayerTools::filament_slot_by_role(c, rc));
        ti(lt.filament_slot(c, region));
        ti(lt.extruder(c, region));
        ti(lt.wall_filament(region));
        ti(lt.sparse_infill_filament(region));
        ti(lt.solid_infill_filament(region));
        ti(extrusion_touches_bed(c, print_z));
        endl_();
    }
}

static void case_tools_for_layer()
{
    for (int it = 0; it < 100; ++it) {
        auto lts = rand_layers(ri(1, 10), 3, false, 30, 0);
        ToolOrdering to{};
        to.m_layer_tools = lts;
        tok("tools_for_layer");
        write_layers(lts);
        arrow();
        for (size_t i = 0; i < lts.size(); ++i) {
            double z = lts[i].print_z + rd(-0.00009, 0.00009);
            td(z);
            ti((long long) (&to.tools_for_layer(z) - to.m_layer_tools.data()));
        }
        endl_();
    }
}

static void write_mixed(const ToolOrdering &to)
{
    ti((long long) to.m_layer_tools.size());
    for (const LayerTools &lt : to.m_layer_tools) {
        tlist_u(lt.extruders);
        ti((long long) lt.mixed_filament_resolution.size());
        for (auto &kv : lt.mixed_filament_resolution) {
            ti(kv.first);
            ti(kv.second);
        }
        ti((long long) lt.mixed_sub_layer_groups.size());
        for (auto &g : lt.mixed_sub_layer_groups) {
            ti(g.mixed_slot_0based);
            tlist_u(g.components_0based);
            ti((long long) g.sub_heights.size());
            for (double h : g.sub_heights)
                td(h);
            td(g.layer_height);
            ti(g.is_gradient);
            ti(g.gradient_first_sorted_idx);
            ti((long long) g.per_object_gradient.size());
            for (auto &kv : g.per_object_gradient) {
                ti((long long) (uintptr_t) kv.first);
                ti((long long) kv.second.total_layers);
                ti((long long) kv.second.current_idx);
                td(kv.second.gradient_start);
                td(kv.second.gradient_end);
                ti((long long) kv.second.curve.points.size());
            }
            ti((long long) g.per_volume_gradient.size());
            for (auto &kv : g.per_volume_gradient) {
                ti((long long) (uintptr_t) kv.first.obj);
                ti((long long) kv.first.volume_id.id);
                ti((long long) kv.second.total_layers);
                ti((long long) kv.second.current_idx);
            }
        }
    }
    tlist_u(to.m_used_mixed_filaments);
    tlist_u(to.m_directly_used_filaments);
}

static void case_resolve_mixed()
{
    const std::vector<std::string> curves{"", "0,0.2|1,0.8", "0,0.1|0.5,0.9|1,0.3", "0.2,0.5,0.1,|0.8,0.6,,0.2|0.5,0.4",
                                          "0,0.5", "x,1|0,0.4|1,0.6"};
    for (int it = 0; it < 500; ++it) {
        int nphys = ri(2, 4);
        int nmix  = ri(1, 2);
        int nf    = nphys + nmix;
        std::vector<int>         is_mixed(nf, 0), grad(nf, 0);
        std::vector<std::string> comps(nf), ratios(nf), ranges(nf), gcurves(nf);
        for (int m = nphys; m < nf; ++m) {
            is_mixed[m] = 1;
            int nc      = rb(80) ? 2 : ri(1, 3);
            std::vector<int> cs;
            for (int k = 0; k < nc; ++k)
                cs.push_back(rb(5) ? nf : ri(1, nphys));
            comps[m] = join(cs);
            if (rb(70)) {
                std::vector<double> rs;
                for (int k = 0; k < nc; ++k)
                    rs.push_back(rd(0.05, 1.0));
                ratios[m] = join_d(rs);
            }
            grad[m] = rb(50);
            if (rb(60))
                ranges[m] = std::to_string(rd(0.0, 1.0)).substr(0, 5) + "," + std::to_string(rd(0.0, 1.0)).substr(0, 5);
            gcurves[m] = curves[ri(0, (int) curves.size() - 1)];
        }
        bool sublayer = rb(60);
        Cfg  c{{"filament_is_mixed", join(is_mixed)},
              {"filament_mixed_components", join_s(comps)},
              {"enable_mixed_color_sublayer", sublayer ? "1" : "0"},
              {"filament_mixed_gradient", join(grad)}};
        c.emplace_back("filament_mixed_sublayer_ratios", join_s(ratios));
        c.emplace_back("filament_mixed_gradient_range", join_s(ranges));
        c.emplace_back("filament_mixed_gradient_curve", join_s(gcurves));
        PrintConfig pc = make_cfg(c);

        auto lts = rand_layers(ri(1, 12), nf, false, 15, 0);
        for (auto &lt : lts)
            std::sort(lt.extruders.begin(), lt.extruders.end());
        ToolOrdering to{};
        to.m_layer_tools = lts;
        // per-object tracking as collect_extruders() would leave it: objects 1 and 2
        std::vector<std::tuple<int, int, std::vector<size_t>>> mol;   // slot, obj, layers
        std::vector<std::pair<int, std::vector<size_t>>> oal;          // obj, layers
        std::vector<std::tuple<int, int, int, std::vector<size_t>>> gvl; // slot, obj, vol, layers
        std::vector<std::tuple<int, size_t, double>> mlh;              // slot, layer, height
        for (int obj = 1; obj <= 2; ++obj) {
            if (rb(30))
                continue;
            std::vector<size_t> all;
            for (size_t li = 0; li < lts.size(); ++li)
                if (rb(80))
                    all.push_back(li);
            oal.emplace_back(obj, all);
            for (int m = nphys; m < nf; ++m) {
                std::vector<size_t> ls;
                for (size_t li : all)
                    if (std::find(lts[li].extruders.begin(), lts[li].extruders.end(), unsigned(m)) != lts[li].extruders.end() && rb(85))
                        ls.push_back(li);
                if (!ls.empty())
                    mol.emplace_back(m, obj, ls);
                if (!ls.empty() && rb(30))
                    gvl.emplace_back(m, obj, ri(1, 3), ls);
            }
        }
        for (int m = nphys; m < nf; ++m)
            for (size_t li = 0; li < lts.size(); ++li)
                if (rb(15))
                    mlh.emplace_back(m, li, rd(0.05, 0.5));
        auto P = [](int obj) { return reinterpret_cast<const PrintObject *>(uintptr_t(obj)); };
        for (auto &[m, obj, ls] : mol)
            to.m_mixed_object_layers[unsigned(m)][P(obj)] = ls;
        for (auto &[obj, ls] : oal)
            to.m_object_all_layer_indices[P(obj)] = ls;
        for (auto &[m, obj, vol, ls] : gvl) {
            ObjectID oid{size_t(vol)};
            LayerTools::MixedSubLayerGroup::VolumeKey vk{P(obj), oid};
            to.m_gradient_volume_layers[unsigned(m)][vk] = ls;
        }
        for (auto &[m, li, h] : mlh)
            to.m_mixed_layer_heights[unsigned(m)][li] = h;

        tok("resolve_mixed_filaments");
        write_cfg(c);
        write_layers(lts);
        ti((long long) mol.size());
        for (auto &[m, obj, ls] : mol) {
            ti(m);
            ti(obj);
            tlist_u(ls);
        }
        ti((long long) oal.size());
        for (auto &[obj, ls] : oal) {
            ti(obj);
            tlist_u(ls);
        }
        ti((long long) gvl.size());
        for (auto &[m, obj, vol, ls] : gvl) {
            ti(m);
            ti(obj);
            ti(vol);
            tlist_u(ls);
        }
        ti((long long) mlh.size());
        for (auto &[m, li, h] : mlh) {
            ti(m);
            ti((long long) li);
            td(h);
        }
        arrow();
        to.resolve_mixed_filaments(pc);
        write_mixed(to);
        // shuffle the order the flush optimizer would have produced (written), then enforce the component order
        for (auto &lt : to.m_layer_tools)
            if (lt.extruders.size() > 1 && rb(50))
                std::reverse(lt.extruders.begin(), lt.extruders.end());
        ti((long long) to.m_layer_tools.size());
        for (auto &lt : to.m_layer_tools)
            tlist_u(lt.extruders);
        to.enforce_mixed_component_order();
        ti((long long) to.m_layer_tools.size());
        for (auto &lt : to.m_layer_tools)
            tlist_u(lt.extruders);
        // first_layer_extruders over random bed contacts
        for (int k = 0; k < ri(0, 3); ++k)
            to.m_bed_contact_filaments.emplace_back(to.m_layer_tools[size_t(ri(0, (int) to.m_layer_tools.size() - 1))].print_z,
                                                    unsigned(ri(1, nf)));
        ti((long long) to.m_bed_contact_filaments.size());
        for (auto &bc : to.m_bed_contact_filaments) {
            td(bc.first);
            ti(bc.second);
        }
        tlist_u(to.first_layer_extruders());
        endl_();
    }
}

static void case_assign_custom_gcodes()
{
    for (int it = 0; it < 300; ++it) {
        int  nf  = ri(1, 4);
        auto lts = rand_layers(ri(1, 10), nf, false, 20, 0);
        std::vector<double> fd(nf, 1.75);
        Cfg c{{"filament_diameter", join_d(fd)}};
        Print print;
        print.m_config.apply(make_cfg(c), true);
        CustomGCode::Info info;
        info.mode = std::vector<CustomGCode::Mode>{CustomGCode::SingleExtruder, CustomGCode::MultiAsSingle,
                                                   CustomGCode::MultiExtruder}[ri(0, 2)];
        for (int k = 0; k < ri(0, 5); ++k) {
            CustomGCode::Item item;
            item.print_z  = rb(20) ? lts[size_t(ri(0, (int) lts.size() - 1))].print_z : rd(0.0, lts.back().print_z + 0.5);
            item.type     = std::vector<CustomGCode::Type>{CustomGCode::ColorChange, CustomGCode::PausePrint,
                                                       CustomGCode::ToolChange, CustomGCode::Template, CustomGCode::Custom}[ri(0, 4)];
            item.extruder = ri(1, nf);
            item.color    = "#00FF00";
            item.extra    = "x" + std::to_string(k);
            info.gcodes.push_back(item);
        }
        print.m_model.plates_custom_gcodes[print.m_model.curr_plate_index] = info;
        ToolOrdering to{};
        to.m_layer_tools = lts;
        tok("assign_custom_gcodes");
        write_cfg(c);
        write_layers(lts);
        ti((int) info.mode);
        ti((long long) info.gcodes.size());
        for (auto &g : info.gcodes) {
            td(g.print_z);
            ti((int) g.type);
            ti(g.extruder);
            ts(g.color);
            ts(g.extra);
        }
        arrow();
        to.assign_custom_gcodes(print);
        ti((long long) to.m_layer_tools.size());
        for (auto &lt : to.m_layer_tools)
            ts(lt.custom_gcode ? lt.custom_gcode->extra : std::string("none"));
        endl_();
    }
}

// The two constructors over a Print built by hand: objects with layers, regions, perimeter and fill
// collections, raw slices and support layers (no meshes are sliced).
static DynamicPrintConfig make_dyn(const Cfg &c)
{
    DynamicPrintConfig dc = DynamicPrintConfig::full_print_config();
    for (auto &kv : c) {
        const ConfigOptionDef *def = print_config_def.get(kv.first);
        if (def != nullptr && def->type == coEnums)
            if (auto *o = dynamic_cast<ConfigOptionEnumsGeneric *>(dc.option(kv.first, true)))
                o->keys_map = def->enum_keys_map;
        dc.set_deserialize_strict(kv.first, kv.second);
    }
    return dc;
}

static ExtrusionEntityCollection rand_island(const std::vector<ExtrusionRole> &roles, double layer_z)
{
    ExtrusionEntityCollection c;
    int n = ri(1, 3);
    for (int i = 0; i < n; ++i) {
        float h = rb(50) ? float(layer_z) : float(rd(0.05, 0.3));
        ExtrusionPath p(roles[size_t(ri(0, (int) roles.size() - 1))], 0.1, 0.42f, h);
        if (rb(70))
            c.append(p);
        else {
            ExtrusionPaths ps{p};
            c.append(ExtrusionLoop(std::move(ps)));
        }
    }
    return c;
}

static void write_expolys(const ExPolygons &es)
{
    ti((long long) es.size());
    for (auto &e : es) {
        ti((long long) e.contour.points.size());
        for (auto &p : e.contour.points) {
            ti(p.x());
            ti(p.y());
        }
    }
}

static void write_layer_tools_build(const ToolOrdering &to)
{
    ti((long long) to.m_layer_tools.size());
    for (const LayerTools &lt : to.m_layer_tools) {
        td(lt.print_z);
        tlist_u(lt.extruders);
        ti(lt.has_object);
        ti(lt.has_support);
        ti(lt.has_skirt);
        ti(lt.extruder_override);
        ti(lt.m_wiping_extrusions.something_overridable);
    }
    ti((long long) to.m_first_printing_extruder);
    ti((long long) to.m_last_printing_extruder);
    tlist_u(to.m_all_printing_extruders);
    ti((long long) to.m_bed_contact_filaments.size());
    for (auto &bc : to.m_bed_contact_filaments) {
        td(bc.first);
        ti(bc.second);
    }
}

static void case_build()
{
    const std::vector<ExtrusionRole> peri{erPerimeter, erExternalPerimeter, erOverhangPerimeter};
    const std::vector<ExtrusionRole> fill{erInternalInfill, erSolidInfill, erTopSolidInfill, erBridgeInfill, erGapFill, erBottomSurface};
    const std::vector<ExtrusionRole> supp{erSupportMaterial, erSupportMaterialInterface, erSupportTransition};
    for (int it = 0; it < 300; ++it) {
        int nf = ri(1, 4);
        std::vector<int> sol;
        for (int f = 0; f < nf; ++f)
            sol.push_back(rb(15));
        Cfg c{{"filament_diameter", join_d(std::vector<double>(nf, 1.75))},
              {"filament_soluble", join(sol)},
              {"filament_colour", join_s(std::vector<std::string>(nf, "#FF0000"))},
              {"enable_prime_tower", rb(50) ? "1" : "0"},
              {"skirt_loops", rb(50) ? "1" : "0"},
              {"skirt_height", std::to_string(ri(0, 3))},
              {"initial_layer_line_width", rb(50) ? "0.42" : "0.5"},
              {"max_layer_height", rb(50) ? "0" : "0.28"}};
        if (rb(30)) {
            std::vector<int> seq;
            for (int f = 1; f <= nf; ++f)
                seq.push_back(f);
            for (int k = 0; k < nf; ++k)
                std::swap(seq[size_t(ri(0, nf - 1))], seq[size_t(ri(0, nf - 1))]);
            if (rb(30))
                seq.pop_back();
            c.emplace_back("first_layer_print_sequence", join(seq));
        }
        Print *print = new Print();
        print->m_config.apply(make_cfg(c), true);
        print->m_full_print_config = make_dyn(c);
        Model *model = new Model();
        int nobj = ri(1, 2);
        std::vector<std::vector<int>> region_filaments; // per object, per region: wall sparse solid
        for (int o = 0; o < nobj; ++o) {
            ModelObject   *mo = model->add_object();
            ModelInstance *mi = mo->add_instance();
            PrintInstances insts;
            insts.push_back(PrintInstance{nullptr, mi, Point(0, 0)});
            PrintObject *obj = new PrintObject(print, mo, Transform3d::Identity(), std::move(insts));
            obj->m_config.support_filament.value           = rb(50) ? 0 : ri(1, nf);
            obj->m_config.support_interface_filament.value = obj->m_config.support_filament.value > 0 && rb(50) ? ri(1, nf) : 0;
            obj->m_config.support_interface_not_for_body.value = rb(30);
            obj->m_config.flush_into_objects.value = rb(30);
            obj->m_config.flush_into_infill.value  = rb(40);
            obj->m_config.flush_into_support.value = rb(50);
            obj->m_config.line_width.value         = rb(50) ? 0.42 : 0.45;
            obj->m_config.layer_height.value       = 0.2;
            obj->m_config.raft_layers.value        = 0;
            obj->m_config.brim_type.value          = rb(50) ? btNoBrim : btOuterOnly;
            obj->m_config.brim_width.value         = rb(80) ? 5. : 0.;
            PrintObjectRegions *regs = new PrintObjectRegions();
            int nreg = ri(1, 3);
            for (int r = 0; r < nreg; ++r) {
                PrintRegionConfig rc;
                rc.wall_filament.value          = ri(1, nf);
                rc.sparse_infill_filament.value = rb(10) ? 0 : ri(1, nf);
                rc.solid_infill_filament.value  = rb(10) ? 0 : ri(1, nf);
                regs->all_regions.push_back(std::make_unique<PrintRegion>(rc));
            }
            obj->m_shared_regions = regs;
            double z = o == 1 && rb(50) ? 0.25 : 0.2;
            int nl = ri(1, 8);
            for (int l = 0; l < nl; ++l) {
                double h = l == 0 ? z : (rb(80) ? 0.2 : rd(0.08, 0.3));
                double pz = l == 0 ? z : obj->m_layers.back()->print_z + h;
                Layer *layer = new Layer(size_t(l), obj, h, pz, pz - h / 2);
                obj->m_layers.push_back(layer);
                for (int r = 0; r < nreg; ++r) {
                    if (rb(20))
                        continue;
                    LayerRegion *lr = layer->add_region(&obj->printing_region(size_t(r)));
                    if (rb(70))
                        for (int k = 0; k < ri(1, 2); ++k)
                            lr->perimeters.append(rand_island(peri, pz));
                    if (rb(70))
                        for (int k = 0; k < ri(1, 2); ++k)
                            lr->fills.append(rand_island(rb(20) ? std::vector<ExtrusionRole>{erTopSolidInfill, erPerimeter, erInternalInfill} : fill, pz));
                    for (int k = 0; k < ri(0, 2); ++k) {
                        double w = rb(30) ? rd(0.02, 0.2) : rd(1.0, 20.0);
                        double x0 = rd(-50, 50), y0 = rd(-50, 50);
                        Polygon poly{Point::new_scale(x0, y0), Point::new_scale(x0 + w, y0), Point::new_scale(x0 + w, y0 + w * rd(0.5, 2.0)),
                                     Point::new_scale(x0, y0 + w)};
                        lr->raw_slices.emplace_back(poly);
                    }
                }
            }
            if (rb(40)) {
                int ns = ri(1, 4);
                for (int k = 0; k < ns; ++k) {
                    double pz = rb(60) && k < (int) obj->m_layers.size() ? obj->m_layers[size_t(k)]->print_z : 0.2 * (k + 1) + 0.1;
                    SupportLayer *sl = obj->add_support_layer(k, k, 0.2, pz);
                    for (int j = 0; j < ri(1, 2); ++j)
                        sl->support_fills.append(rand_island(supp, pz));
                    sl->planned_filament_id           = rb(80) ? 0 : ri(1, nf);
                    sl->planned_interface_filament_id = rb(80) ? 0 : ri(1, nf);
                }
            }
            print->m_objects.push_back(obj);
        }
        unsigned int first = rb(50) ? (unsigned int) -1 : unsigned(ri(1, nf));
        bool         prime = rb(30);

        tok("build");
        write_cfg(c);
        ti(print->has_skirt());
        ti((long long) print->m_objects.size());
        for (PrintObject *obj : print->m_objects) {
            ti(obj->config().support_filament.value);
            ti(obj->config().support_interface_filament.value);
            ti(obj->config().support_interface_not_for_body.value);
            ti(obj->config().flush_into_objects.value);
            ti(obj->config().flush_into_infill.value);
            ti(obj->config().flush_into_support.value);
            td(obj->config().line_width.value);
            td(obj->config().layer_height.value);
            ti(obj->has_brim());
            ti((long long) obj->num_printing_regions());
            ti((long long) obj->layers().size());
            for (const Layer *layer : obj->layers()) {
                ti((long long) layer->id());
                td(layer->print_z);
                td(layer->height);
                ti(layer->has_extrusions());
                ti((long long) layer->regions().size());
                for (const LayerRegion *lr : layer->regions()) {
                    ti(lr->region().config().wall_filament.value);
                    ti(lr->region().config().sparse_infill_filament.value);
                    ti(lr->region().config().solid_infill_filament.value);
                    write_coll(lr->perimeters, 0);
                    write_coll(lr->fills, 0);
                    write_expolys(lr->raw_slices);
                    ti(lr->slices.empty());
                }
            }
            ti((long long) obj->support_layers().size());
            for (const SupportLayer *sl : obj->support_layers()) {
                td(sl->print_z);
                write_coll(sl->support_fills, 0);
                ti(sl->planned_filament_id);
                ti(sl->planned_interface_filament_id);
            }
        }
        ti((long long) first);
        ti(prime);
        arrow();
        {
            ToolOrdering to(*print, first, prime);
            write_layer_tools_build(to);
            for (PrintObject *obj : print->m_objects) {
                tlist_u(obj->object_first_layer_wall_extruders);
                ti(obj->object_brim_filament);
                td(obj->object_brim_print_z);
            }
        }
        for (PrintObject *obj : print->m_objects) {
            ToolOrdering to(*obj, first, prime);
            write_layer_tools_build(to);
            tlist_u(obj->object_first_layer_wall_extruders);
            ti(obj->object_brim_filament);
            td(obj->object_brim_print_z);
        }
        endl_();
    }
}

// ---------------------------------------------------------------------------------------------
// The filament grouping of ToolOrdering.cpp: GroupReorder::build_manual_nozzle_map and
// build_filament_group_context, refine_groups_by_Nozzle_State, plan_filament_mapping_and_order_by_combo_ranges,
// ToolOrdering::get_recommended_filament_maps, and sort_and_build_data (reorder_extruders_for_minimum_flush_volume,
// calculate_and_store_statistics) over hand-built prints. Inputs stay small so that FilamentGroup always takes its
// enumeration path (no time limit is reached).
//
// A grouping configuration is "C" (above) then the Print state "P": calib extruder_id has_bowden mode,
// the extruder filament info (n groups, per group n items: colour type tray is_support filament_id texts),
// the filament usage types list, the filament print time as insertion steps (n, then filament extruder
// time), the nozzle loaded filaments (n, then nozzle_id filament_id valid colour type is_support usage
// diameter), is_dynamic_group_reorder, is_sequential_print.
// ---------------------------------------------------------------------------------------------

static std::string rand_colour()
{
    static const char *d = "0123456789ABCDEF";
    std::string s = "#";
    for (int i = 0; i < 6; ++i)
        s += d[ri(0, 15)];
    return s;
}

static const std::vector<std::string> k_ftypes{"PLA", "PETG", "ABS", "PLA-S", "Sup.PLA", "TPU"};

struct GroupPrintState
{
    int                                            calib_extruder = 0;
    bool                                           calib_bowden   = false;
    int                                            calib_mode     = 0;
    std::vector<std::vector<std::vector<std::string>>> ext_items; // per extruder, per item: colour type tray support id
    std::vector<std::tuple<int, int, double>>      print_time;
    std::vector<FilamentGroupUtils::NozzleLoadedFilamentInfo> loaded;
};

// A random grouping configuration: kind 0 = one nozzle per extruder (1 or 2 extruders), 1 = two
// extruders, the second with several nozzles, 2 = one extruder driving several nozzles (hotend as
// extruder, per-slot nozzle parameters).
static Cfg rand_group_cfg(int kind, int nf, int &ne_out, int &nozzle_cnt_out, const std::string &mode_name)
{
    const std::vector<std::string> vtn{"Standard", "High Flow", "Hybrid", "TPU High Flow"};
    int ne = kind == 0 ? ri(1, 2) : (kind == 1 ? 2 : 1);
    ne_out = ne;
    std::vector<double> nd;
    std::vector<int>    maxc;
    std::vector<std::string> nvt, etype, stats, amsc;
    for (int e = 0; e < ne; ++e) {
        nd.push_back(rb(80) ? 0.4 : 0.2);
        maxc.push_back(kind == 0 ? 1 : (kind == 1 ? (e == 0 ? 1 : ri(2, 4)) : ri(2, 4)));
        nvt.push_back(kind == 1 && e == 1 && rb(40) ? "Hybrid" : vtn[size_t(ri(0, 1))]);
        etype.push_back(rb(25) ? "Bowden" : "Direct Drive");
        std::string st;
        if (kind == 1 && e == 1) {
            if (nvt.back() == "Hybrid")
                st = "Standard#" + std::to_string(ri(1, 2)) + "|High Flow#" + std::to_string(ri(1, 2));
            else
                st = nvt.back() + "#" + std::to_string(maxc.back());
        }
        stats.push_back(st);
        int r = ri(0, 3);
        amsc.push_back(r == 0 ? std::string() : (r == 1 ? "1#0" : (r == 2 ? "4#1" : "1#1|4#1")));
    }
    int nozzle_cnt = 0;
    for (int m : maxc)
        nozzle_cnt += m;
    nozzle_cnt_out = nozzle_cnt;
    std::vector<std::string> col, ft, fid, sel;
    std::vector<int> sup, fmap, fvol, fnoz;
    std::vector<double> fdia;
    for (int f = 0; f < nf; ++f) {
        col.push_back(rand_colour());
        ft.push_back(k_ftypes[size_t(ri(0, (int) k_ftypes.size() - 1))]);
        fid.push_back(std::vector<std::string>{"GFA00", "GFL99", "GFB00", ""}[ri(0, 3)]);
        sel.push_back(rb(80) ? std::string() : (rb(70) ? "0.4" : "0.2"));
        sup.push_back(rb(20));
        fmap.push_back(ri(1, ne));
        fvol.push_back(ri(0, 2));
        fnoz.push_back(ri(0, nozzle_cnt - 1));
        fdia.push_back(rb(90) ? 1.75 : 2.85);
    }
    std::vector<double> fvm;
    for (int e = 0; e < ne; ++e)
        for (int i = 0; i < nf; ++i)
            for (int j = 0; j < nf; ++j)
                fvm.push_back(i == j ? 0. : double(ri(20, 900)));
    Cfg c{{"nozzle_diameter", join_d(nd)},
          {"extruder_max_nozzle_count", join(maxc)},
          {"nozzle_volume_type", join(nvt)},
          {"extruder_type", join(etype)},
          {"extruder_nozzle_stats", join_s(stats)},
          {"extruder_ams_count", join_s(amsc)},
          {"filament_colour", join_s(col)},
          {"filament_type", join_s(ft)},
          {"filament_ids", join_s(fid)},
          {"filament_is_support", join(sup)},
          {"filament_diameter", join_d(fdia)},
          {"filament_map", join(fmap)},
          {"filament_volume_map", join(fvol)},
          {"filament_nozzle_map", join(fnoz)},
          {"filament_diameter_map", join_s(sel)},
          {"filament_map_mode", mode_name},
          {"master_extruder_id", std::to_string(ri(1, ne))},
          {"flush_volumes_matrix", join_d(fvm)},
          {"flush_multiplier", join_d(std::vector<double>(ne, rb(70) ? 1.0 : 0.8))},
          {"group_algo_with_time", rb(30) ? "1" : "0"},
          {"machine_load_filament_time", std::to_string(ri(10, 30))},
          {"machine_unload_filament_time", std::to_string(ri(10, 30))},
          {"machine_switch_extruder_time", std::to_string(ri(5, 20))},
          {"has_filament_switcher", rb(10) ? "1" : "0"},
          {"enable_filament_dynamic_map", rb(25) ? "1" : "0"},
          {"multi_nozzle_diameter_mode", rb(15) ? "1" : "0"},
          {"printer_variant", "0.4"},
          {"handle_hotend_as_extruder", kind == 2 ? "1" : (rb(20) ? "1" : "0")}};
    if (kind == 2 || (kind == 1 && rb(50))) {
        // per-slot nozzle parameters
        std::vector<int> cnt, slots;
        std::vector<std::string> svt;
        std::vector<double> sdia;
        int total = 0;
        for (int e = 0; e < ne; ++e) {
            int k = rb(80) ? maxc[size_t(e)] : std::max(1, maxc[size_t(e)] - 1);
            cnt.push_back(k);
            total += k;
        }
        std::vector<int> perm((size_t) nozzle_cnt);
        for (int i = 0; i < nozzle_cnt; ++i)
            perm[size_t(i)] = i;
        for (int i = nozzle_cnt - 1; i > 0; --i)
            std::swap(perm[size_t(i)], perm[size_t(ri(0, i))]);
        for (int i = 0; i < total; ++i) {
            svt.push_back(vtn[size_t(ri(0, 1))]);
            sdia.push_back(rb(80) ? 0.4 : 0.2);
            slots.push_back(perm[size_t(i)]);
        }
        c.emplace_back("extruder_nozzle_count", join(cnt));
        c.emplace_back("extruder_nozzle_volume_type", join(svt));
        if (rb(70))
            c.emplace_back("extruder_nozzle_diameter", join_d(sdia));
        if (rb(70))
            c.emplace_back("extruder_nozzle_slot", join(slots));
        if (kind == 2)
            c.emplace_back("ams_nozzle_idx", std::to_string(rb(25) ? -1 : ri(0, nozzle_cnt - 1)));
    }
    return c;
}

static GroupPrintState rand_print_state(int ne, int nf, int nozzle_cnt)
{
    GroupPrintState st;
    st.calib_extruder = ri(-1, 1);
    st.calib_bowden   = rb(15);
    st.calib_mode     = rb(80) ? 0 : std::vector<int>{1, 4}[ri(0, 1)];
    int ng = rb(70) ? 2 : ne;
    for (int g = 0; g < ng; ++g) {
        std::vector<std::vector<std::string>> items;
        for (int k = 0; k < ri(0, 4); ++k)
            items.push_back({rb(10) ? std::string() : rand_colour(), k_ftypes[size_t(ri(0, (int) k_ftypes.size() - 1))],
                             std::vector<std::string>{"A1", "B2", "Ext", ""}[ri(0, 3)], rb(20) ? "1" : "0",
                             std::vector<std::string>{"GFA00", "GFL99", ""}[ri(0, 2)]});
        st.ext_items.push_back(items);
    }
    std::vector<int> fl((size_t) nf);
    for (int f = 0; f < nf; ++f)
        fl[size_t(f)] = f;
    for (int i = nf - 1; i > 0; --i)
        std::swap(fl[size_t(i)], fl[size_t(ri(0, i))]);
    if (rb(70))
        for (int f : fl) {
            if (rb(20))
                continue;
            for (int e = 0; e < std::max(ne, nozzle_cnt); ++e)
                if (rb(80))
                    st.print_time.emplace_back(f, e, rd(10.0, 500.0));
        }
    for (int k = 0; k < ri(0, 3); ++k) {
        FilamentGroupUtils::NozzleLoadedFilamentInfo l;
        l.nozzle_id   = ri(0, nozzle_cnt);
        l.filament_id = std::vector<std::string>{"GFA00", "GFL99", ""}[ri(0, 2)];
        l.valid       = rb(80);
        l.color       = FilamentGroupUtils::Color(rand_colour());
        l.type        = k_ftypes[size_t(ri(0, (int) k_ftypes.size() - 1))];
        l.is_support  = rb(20);
        l.usage_type  = FilamentUsageType(ri(0, 2));
        l.diameter    = rb(90) ? 1.75f : 2.85f;
        st.loaded.push_back(l);
    }
    return st;
}

static void apply_print_state(Print &print, const Cfg &c, const GroupPrintState &st)
{
    print.m_config.apply(make_cfg(c), true);
    print.m_full_print_config     = make_dyn(c);
    print.m_ori_full_print_config = make_dyn(c);
    print.m_calib_params.extruder_id         = st.calib_extruder;
    print.m_calib_params.has_bowden_extruder = st.calib_bowden;
    print.m_calib_params.mode                = CalibMode(st.calib_mode);
    print.m_extruder_filament_info.clear();
    for (auto &g : st.ext_items) {
        std::vector<DynamicPrintConfig> items;
        for (auto &it : g) {
            DynamicPrintConfig d;
            d.set_key_value("filament_colour", new ConfigOptionStrings({it[0]}));
            d.set_key_value("filament_type", new ConfigOptionStrings({it[1]}));
            d.set_key_value("tray_name", new ConfigOptionStrings({it[2]}));
            d.set_key_value("filament_is_support", new ConfigOptionBools({it[3] == "1"}));
            d.set_key_value("filament_id", new ConfigOptionStrings({it[4]}));
            items.push_back(d);
        }
        print.m_extruder_filament_info.push_back(items);
    }
    print.m_filament_print_time.clear();
    for (auto &t : st.print_time)
        print.m_filament_print_time[std::get<0>(t)][std::get<1>(t)] = std::get<2>(t);
    print.m_nozzle_loaded_filament_info = st.loaded;
}

static void write_print_state(const Print &print, const GroupPrintState &st)
{
    tok("P");
    ti(st.calib_extruder);
    ti(st.calib_bowden);
    ti(st.calib_mode);
    ti((long long) st.ext_items.size());
    for (auto &g : st.ext_items) {
        ti((long long) g.size());
        for (auto &it : g)
            for (auto &s : it)
                ts(s);
    }
    std::vector<int> usage;
    for (auto u : print.get_filament_usage_type())
        usage.push_back((int) u);
    tlist_u(usage);
    ti((long long) st.print_time.size());
    for (auto &t : st.print_time) {
        ti(std::get<0>(t));
        ti(std::get<1>(t));
        td(std::get<2>(t));
    }
    ti((long long) st.loaded.size());
    for (auto &l : st.loaded) {
        ti(l.nozzle_id);
        ts(l.filament_id);
        ti(l.valid);
        ts(l.color.to_hex_str(true));
        ts(l.type);
        ti(l.is_support);
        ti((int) l.usage_type);
        tf(l.diameter);
    }
    ti(print.is_dynamic_group_reorder());
    ti(print.is_sequential_print());
}

static void write_nozzles_installed(const std::vector<MultiNozzleUtils::NozzleInfo> &list)
{
    ti((long long) list.size());
    for (auto &n : list) {
        ts(n.diameter);
        ti((int) n.volume_type);
        ti(n.extruder_id);
        ti(n.group_id);
        ti(n.installed);
    }
}

static void write_sets(const std::vector<std::set<int>> &v)
{
    ti((long long) v.size());
    for (auto &s : v)
        tlist_u(std::vector<int>(s.begin(), s.end()));
}

static void write_ctx(const FilamentGroupContext &ctx)
{
    const auto &m = ctx.model_info;
    ti((long long) m.flush_matrix.size());
    for (auto &fm : m.flush_matrix) {
        ti((long long) fm.size());
        for (auto &row : fm) {
            ti((long long) row.size());
            for (float v : row)
                tf(v);
        }
    }
    ti((long long) m.layer_filaments.size());
    for (auto &l : m.layer_filaments)
        tlist_u(l);
    ti((long long) m.filament_info.size());
    for (auto &f : m.filament_info) {
        ts(f.color.to_hex_str(true));
        ts(f.type);
        ti(f.is_support);
        ti((int) f.usage_type);
        tf(f.diameter);
    }
    ti((long long) m.filament_ids.size());
    for (auto &s : m.filament_ids)
        ts(s);
    ti((long long) m.selected_nozzle_diameters.size());
    for (auto &s : m.selected_nozzle_diameters)
        ts(s);
    write_sets(m.unprintable_filaments);
    ti((long long) m.unprintable_volumes.size());
    for (auto &kv : m.unprintable_volumes) {
        ti(kv.first);
        std::vector<int> v;
        for (auto t : kv.second)
            v.push_back((int) t);
        tlist_u(v);
    }
    const auto &g = ctx.group_info;
    ti(g.total_filament_num);
    td(g.max_gap_threshold);
    ti((int) g.mode);
    ti((int) g.strategy);
    ti(g.ignore_ext_filament);
    ti(g.has_filament_switcher);
    tlist_u(g.filament_volume_map);
    const auto &mi = ctx.machine_info;
    tlist_u(mi.max_group_size);
    ti((long long) mi.machine_filament_info.size());
    for (auto &e : mi.machine_filament_info) {
        ti((long long) e.size());
        for (auto &f : e) {
            ts(f.color.to_hex_str(true));
            ts(f.type);
            ti(f.is_support);
            ti(f.extruder_id);
            ti(f.is_extended);
            ts(f.filament_id);
        }
    }
    ti((long long) mi.nozzle_loaded_filaments.size());
    std::vector<int> pref;
    for (bool b : mi.prefer_non_model_filament)
        pref.push_back(b);
    tlist_u(pref);
    ti(mi.master_extruder_id);
    const auto &sp = ctx.speed_info;
    ti((long long) sp.filament_print_time.size());
    for (auto &kv : sp.filament_print_time) {
        ti(kv.first);
        ti((long long) kv.second.size());
        for (auto &kv2 : kv.second) {
            ti(kv2.first);
            td(kv2.second);
        }
    }
    td(sp.extruder_change_time);
    td(sp.filament_change_time);
    ti(sp.group_with_time);
    tf(sp.change_time_params.selector_load_time);
    tf(sp.change_time_params.selector_unload_time);
    tf(sp.change_time_params.standard_load_time);
    tf(sp.change_time_params.standard_unload_time);
    ti((long long) sp.ams_preload_enabled.size());
    const auto &ni = ctx.nozzle_info;
    ti((long long) ni.extruder_nozzle_list.size());
    for (auto &kv : ni.extruder_nozzle_list) {
        ti(kv.first);
        tlist_u(kv.second);
    }
    write_nozzles_installed(ni.nozzle_list);
    ti((long long) ni.nozzle_status.size());
    for (auto &kv : ni.nozzle_status) {
        ti(kv.first);
        ti(kv.second);
    }
    tlist_u(ni.per_nozzle_capacity);
}

static void write_group(const MultiNozzleUtils::LayeredNozzleGroupResult &r)
{
    ti(r.is_support_dynamic_nozzle_map());
    ti(r.is_handle_nozzle_as_extruder());
    tlist_u(r.get_used_filaments());
    tlist_u(r.get_extruder_map(true, -1));
    tlist_u(r.get_nozzle_map(-1));
    tlist_u(r.get_volume_map(-1));
    ti((long long) r.get_layer_count());
    for (auto &m : r.get_layer_filament_nozzle_maps())
        tlist_u(m);
    ti((long long) r.get_layer_filament_sequences().size());
    for (auto &s : r.get_layer_filament_sequences())
        tlist_u(s);
    for (auto f : r.get_used_filaments()) {
        auto n = r.get_nozzle_for_filament(int(f));
        if (n) {
            tok("N");
            ts(n->diameter);
            ti((int) n->volume_type);
            ti(n->extruder_id);
            ti(n->group_id);
            ti(n->installed);
        } else
            tok("-");
    }
}

static std::vector<std::vector<unsigned int>> rand_layer_filaments(int nl, int nf)
{
    std::vector<std::vector<unsigned int>> lf;
    for (int l = 0; l < nl; ++l) {
        std::vector<unsigned int> v;
        for (int f = 0; f < nf; ++f)
            if (rb(55))
                v.push_back(unsigned(f));
        if (v.empty() && rb(80))
            v.push_back(unsigned(ri(0, nf - 1)));
        for (size_t i = v.size(); i > 1; --i)
            std::swap(v[i - 1], v[size_t(ri(0, int(i) - 1))]);
        lf.push_back(v);
    }
    return lf;
}

static void write_layer_filaments(const std::vector<std::vector<unsigned int>> &lf)
{
    ti((long long) lf.size());
    for (auto &l : lf)
        tlist_u(l);
}

struct GroupInputs
{
    std::vector<std::set<int>>                phys, geom;
    std::map<int, std::set<NozzleVolumeType>> vols;
    std::vector<std::pair<int, int>>          status; // nozzle -> filament, insertion order
};

static GroupInputs rand_group_inputs(int ne, int nf, int nozzle_cnt)
{
    GroupInputs in;
    auto rset = [&](int pct) {
        std::set<int> s;
        for (int f = 0; f < nf; ++f)
            if (rb(pct))
                s.insert(f);
        return s;
    };
    int np = ri(0, ne), ng = ri(0, ne);
    for (int e = 0; e < np; ++e)
        in.phys.push_back(rset(12));
    for (int e = 0; e < ng; ++e)
        in.geom.push_back(rset(10));
    for (int f = 0; f < nf; ++f)
        if (rb(10)) {
            std::set<NozzleVolumeType> s;
            s.insert(NozzleVolumeType(ri(0, 1)));
            in.vols[f] = s;
        }
    if (rb(50))
        for (int k = 0; k < ri(1, 3); ++k)
            in.status.emplace_back(ri(0, nozzle_cnt - 1), ri(-1, nf - 1));
    return in;
}

static void write_group_inputs(const GroupInputs &in)
{
    write_sets(in.phys);
    write_sets(in.geom);
    ti((long long) in.vols.size());
    for (auto &kv : in.vols) {
        ti(kv.first);
        std::vector<int> v;
        for (auto t : kv.second)
            v.push_back((int) t);
        tlist_u(v);
    }
    ti((long long) in.status.size());
    for (auto &p : in.status) {
        ti(p.first);
        ti(p.second);
    }
}

static std::unordered_map<int, int> status_map(const std::vector<std::pair<int, int>> &s)
{
    std::unordered_map<int, int> m;
    for (auto &p : s)
        m[p.first] = p.second;
    return m;
}

static const std::vector<std::string> k_modes{"Auto For Flush", "Auto For Match", "Manual", "Nozzle Manual", "Auto For Quality"};

static void case_build_manual_nozzle_map()
{
    for (int it = 0; it < 200; ++it) {
        int nf = ri(1, 4), ne = 0, nz = 0;
        Cfg c = rand_group_cfg(2, nf, ne, nz, rb(50) ? "Manual" : "Nozzle Manual");
        PrintConfig pc = make_cfg(c);
        auto list = MultiNozzleUtils::build_nozzle_list_from_extruder_nozzles(pc);
        if (list.empty())
            list = rand_nozzles(1, ri(1, 4));
        for (auto &n : list)
            if (rb(15))
                n.installed = false;
        std::vector<unsigned int> used;
        for (int f = 0; f < nf; ++f)
            if (rb(75))
                used.push_back(unsigned(f));
        FilamentMapMode mode = rb(50) ? fmmManual : fmmNozzleManual;
        int ams_idx = ri(-1, (int) list.size());
        int ams_cap = ri(0, 3);
        tok("build_manual_nozzle_map");
        write_cfg(c);
        write_nozzles_installed(list);
        tlist_u(used);
        ti((int) mode);
        ti(ams_idx);
        ti(ams_cap);
        ti(nf);
        arrow();
        auto map = GroupReorder::build_manual_nozzle_map(pc, list, used, mode, ams_idx, ams_cap, size_t(nf));
        tlist_u(map);
        write_nozzles_installed(list);
        endl_();
    }
}

static void case_build_filament_group_context()
{
    for (int it = 0; it < 300; ++it) {
        int kind = ri(0, 2), nf = kind == 0 ? ri(1, 6) : ri(1, 4), ne = 0, nz = 0;
        std::string mode_name = k_modes[size_t(ri(0, 4))];
        Cfg c = rand_group_cfg(kind, nf, ne, nz, mode_name);
        GroupPrintState st = rand_print_state(ne, nf, nz);
        Print print;
        apply_print_state(print, c, st);
        auto lf = rand_layer_filaments(ri(1, 5), nf);
        GroupInputs in = rand_group_inputs(ne, nf, nz);
        FilamentMapMode mode = FilamentMapMode(ri(0, 4));
        tok("build_filament_group_context");
        write_cfg(c);
        write_print_state(print, st);
        write_layer_filaments(lf);
        write_group_inputs(in);
        ti((int) mode);
        arrow();
        try {
            auto ctx = GroupReorder::build_filament_group_context(&print, lf, in.phys, in.geom, in.vols, mode, status_map(in.status));
            tok("ok");
            write_ctx(ctx);
        } catch (const std::exception &e) {
            tok("ERR");
            ts(e.what());
        }
        endl_();
    }
}

static void case_get_recommended_filament_maps()
{
    for (int it = 0; it < 400; ++it) {
        int kind = ri(0, 2), nf = kind == 0 ? ri(1, 6) : ri(1, 4), ne = 0, nz = 0;
        std::string mode_name = k_modes[size_t(ri(0, 4))];
        Cfg c = rand_group_cfg(kind, nf, ne, nz, mode_name);
        GroupPrintState st = rand_print_state(ne, nf, nz);
        Print print;
        apply_print_state(print, c, st);
        auto lf = rand_layer_filaments(ri(1, 5), nf);
        GroupInputs in = rand_group_inputs(ne, nf, nz);
        FilamentMapMode mode = rb(70) ? print.m_config.filament_map_mode.value : FilamentMapMode(ri(0, 4));
        tok("get_recommended_filament_maps");
        write_cfg(c);
        write_print_state(print, st);
        write_layer_filaments(lf);
        write_group_inputs(in);
        ti((int) mode);
        arrow();
        try {
            auto r = ToolOrdering::get_recommended_filament_maps(&print, lf, mode, in.phys, in.geom, in.vols, status_map(in.status));
            tok("ok");
            write_group(r);
        } catch (const std::exception &e) {
            tok("ERR");
            ts(e.what());
        }
        endl_();
    }
}

static void case_refine_groups()
{
    for (int it = 0; it < 200; ++it) {
        int kind = ri(1, 2), nf = ri(1, 4), ne = 0, nz = 0;
        Cfg c = rand_group_cfg(kind, nf, ne, nz, "Auto For Flush");
        GroupPrintState st = rand_print_state(ne, nf, nz);
        Print print;
        apply_print_state(print, c, st);
        auto lf = rand_layer_filaments(ri(1, 5), nf);
        GroupInputs in = rand_group_inputs(ne, nf, nz);
        FilamentGroupContext ctx;
        try {
            ctx = GroupReorder::build_filament_group_context(&print, lf, in.phys, in.geom, in.vols, fmmAutoForFlush, status_map(in.status));
        } catch (const std::exception &) {
            continue;
        }
        if (ctx.nozzle_info.nozzle_list.empty())
            continue;
        std::vector<int> gmap;
        for (int f = 0; f < nf; ++f)
            gmap.push_back(ri(0, (int) ctx.nozzle_info.nozzle_list.size() - 1));
        std::vector<std::pair<int, int>> ns;
        for (int k = 0; k < ri(0, 4); ++k)
            ns.emplace_back(ri(0, (int) ctx.nozzle_info.nozzle_list.size() - 1), ri(-1, nf));
        auto used = collect_sorted_used_filaments(lf);
        auto group = MultiNozzleUtils::LayeredNozzleGroupResult::create(gmap, ctx.nozzle_info.nozzle_list, used, rb(30));
        if (!group)
            continue;
        tok("refine_groups_by_Nozzle_State");
        write_cfg(c);
        write_print_state(print, st);
        write_layer_filaments(lf);
        write_group_inputs(in);
        tlist_u(gmap);
        ti(group->is_handle_nozzle_as_extruder());
        ti((long long) ns.size());
        for (auto &p : ns) {
            ti(p.first);
            ti(p.second);
        }
        arrow();
        auto r = refine_groups_by_Nozzle_State(ctx, *group, status_map(ns));
        write_group(r);
        endl_();
    }
}

static void write_recorder(const MultiNozzleUtils::NozzleStatusRecorder &r)
{
    ti(r.get_current_extruder_id());
    ti((long long) r.get_nozzle_filament_map().size());
    for (auto &kv : r.get_nozzle_filament_map()) {
        ti(kv.first);
        ti(kv.second);
    }
    ti((long long) r.get_extruder_nozzle_map().size());
    for (auto &kv : r.get_extruder_nozzle_map()) {
        ti(kv.first);
        ti(kv.second);
    }
}

static void case_plan_combo_ranges()
{
    for (int it = 0; it < 150; ++it) {
        int kind = ri(0, 1), nf = ri(1, 4), ne = 0, nz = 0;
        Cfg c = rand_group_cfg(kind, nf, ne, nz, "Auto For Flush");
        c.emplace_back("other_layers_print_sequence_nums", "0");
        GroupPrintState st = rand_print_state(ne, nf, nz);
        Print print;
        apply_print_state(print, c, st);
        auto lf = rand_layer_filaments(ri(1, 6), nf);
        if (rb(50) && lf.size() > 2)
            lf[2] = lf[1];
        GroupInputs in = rand_group_inputs(ne, nf, nz);
        FilamentGroupContext ctx;
        try {
            ctx = GroupReorder::build_filament_group_context(&print, lf, in.phys, in.geom, in.vols, fmmAutoForFlush, status_map(in.status));
        } catch (const std::exception &) {
            continue;
        }
        ToolOrdering to{};
        for (auto &l : lf) {
            LayerTools lt(0.2 * double(to.m_layer_tools.size() + 1));
            lt.extruders = l;
            to.m_layer_tools.push_back(lt);
        }
        bool reorder_first = rb(50);
        auto octx = to.prepare_ordering_context(print.config(), reorder_first);
        MultiNozzleUtils::NozzleStatusRecorder rec;
        std::vector<std::tuple<int, int, int>> calls;
        for (auto &p : in.status)
            if (p.second >= 0)
                calls.emplace_back(p.first, p.second, ri(-1, ne - 1));
        int cur_ext = ri(-1, ne - 1);
        for (auto &cl : calls)
            rec.set_nozzle_status(std::get<0>(cl), std::get<1>(cl), std::get<2>(cl));
        rec.set_current_extruder_id(cur_ext);
        tok("plan_filament_mapping_and_order_by_combo_ranges");
        write_cfg(c);
        write_print_state(print, st);
        write_layer_filaments(lf);
        write_group_inputs(in);
        ti(reorder_first);
        ti((long long) calls.size());
        for (auto &cl : calls) {
            ti(std::get<0>(cl));
            ti(std::get<1>(cl));
            ti(std::get<2>(cl));
        }
        ti(cur_ext);
        arrow();
        try {
            auto res = plan_filament_mapping_and_order_by_combo_ranges(&print, ctx, octx, fmmAutoForFlush, in.phys, in.geom, in.vols, &rec);
            tok("ok");
            ti((long long) res.size());
            for (auto &r : res) {
                tlist_u(r.fil_order);
                tlist_u(r.fil_nozzle_match);
            }
            write_recorder(rec);
        } catch (const std::exception &e) {
            tok("ERR");
            ts(e.what());
        }
        endl_();
    }
}

// sort_and_build_data over hand-built prints (the objects of case_build) with a grouping configuration.
static Print *rand_build_print(const Cfg &c, const GroupPrintState &st, int nf)
{
    const std::vector<ExtrusionRole> peri{erPerimeter, erExternalPerimeter, erOverhangPerimeter};
    const std::vector<ExtrusionRole> fill{erInternalInfill, erSolidInfill, erTopSolidInfill, erBridgeInfill, erGapFill, erBottomSurface};
    const std::vector<ExtrusionRole> supp{erSupportMaterial, erSupportMaterialInterface, erSupportTransition};
    Print *print = new Print();
    apply_print_state(*print, c, st);
    Model *model = new Model();
    int nobj = ri(1, 2);
    for (int o = 0; o < nobj; ++o) {
        ModelObject   *mo = model->add_object();
        ModelInstance *mi = mo->add_instance();
        PrintInstances insts;
        insts.push_back(PrintInstance{nullptr, mi, Point(0, 0)});
        PrintObject *obj = new PrintObject(print, mo, Transform3d::Identity(), std::move(insts));
        obj->m_config.support_filament.value           = rb(60) ? 0 : ri(1, nf);
        obj->m_config.support_interface_filament.value = obj->m_config.support_filament.value > 0 && rb(50) ? ri(1, nf) : 0;
        obj->m_config.support_interface_not_for_body.value = rb(30);
        obj->m_config.flush_into_objects.value = rb(30);
        obj->m_config.flush_into_infill.value  = rb(40);
        obj->m_config.flush_into_support.value = rb(50);
        obj->m_config.line_width.value         = 0.42;
        obj->m_config.layer_height.value       = 0.2;
        obj->m_config.raft_layers.value        = 0;
        obj->m_config.brim_type.value          = rb(50) ? btNoBrim : btOuterOnly;
        obj->m_config.brim_width.value         = rb(80) ? 5. : 0.;
        PrintObjectRegions *regs = new PrintObjectRegions();
        int nreg = ri(1, 3);
        for (int r = 0; r < nreg; ++r) {
            PrintRegionConfig rc;
            rc.wall_filament.value          = ri(1, nf);
            rc.sparse_infill_filament.value = rb(10) ? 0 : ri(1, nf);
            rc.solid_infill_filament.value  = rb(10) ? 0 : ri(1, nf);
            regs->all_regions.push_back(std::make_unique<PrintRegion>(rc));
        }
        obj->m_shared_regions = regs;
        int nl = ri(1, 6);
        for (int l = 0; l < nl; ++l) {
            double h = 0.2;
            double pz = l == 0 ? 0.2 : obj->m_layers.back()->print_z + h;
            Layer *layer = new Layer(size_t(l), obj, h, pz, pz - h / 2);
            obj->m_layers.push_back(layer);
            for (int r = 0; r < nreg; ++r) {
                if (rb(20))
                    continue;
                LayerRegion *lr = layer->add_region(&obj->printing_region(size_t(r)));
                if (rb(80))
                    lr->perimeters.append(rand_island(peri, pz));
                if (rb(70))
                    lr->fills.append(rand_island(fill, pz));
                double x0 = rd(-50, 50), y0 = rd(-50, 50), w = rd(1.0, 20.0);
                Polygon poly{Point::new_scale(x0, y0), Point::new_scale(x0 + w, y0), Point::new_scale(x0 + w, y0 + w), Point::new_scale(x0, y0 + w)};
                lr->raw_slices.emplace_back(poly);
            }
        }
        if (rb(30)) {
            int ns = ri(1, 3);
            for (int k = 0; k < ns; ++k) {
                double pz = k < (int) obj->m_layers.size() ? obj->m_layers[size_t(k)]->print_z : 0.2 * (k + 1);
                SupportLayer *sl = obj->add_support_layer(k, k, 0.2, pz);
                sl->support_fills.append(rand_island(supp, pz));
                sl->planned_filament_id           = 0;
                sl->planned_interface_filament_id = 0;
            }
        }
        print->m_objects.push_back(obj);
    }
    return print;
}

static void write_build_print(const Print *print)
{
    ti(print->has_skirt());
    ti((long long) print->m_objects.size());
    for (PrintObject *obj : print->m_objects) {
        ti(obj->config().support_filament.value);
        ti(obj->config().support_interface_filament.value);
        ti(obj->config().support_interface_not_for_body.value);
        ti(obj->config().flush_into_objects.value);
        ti(obj->config().flush_into_infill.value);
        ti(obj->config().flush_into_support.value);
        td(obj->config().line_width.value);
        td(obj->config().layer_height.value);
        ti(obj->has_brim());
        ti((long long) obj->num_printing_regions());
        ti((long long) obj->layers().size());
        for (const Layer *layer : obj->layers()) {
            ti((long long) layer->id());
            td(layer->print_z);
            td(layer->height);
            ti(layer->has_extrusions());
            ti((long long) layer->regions().size());
            for (const LayerRegion *lr : layer->regions()) {
                ti(lr->region().config().wall_filament.value);
                ti(lr->region().config().sparse_infill_filament.value);
                ti(lr->region().config().solid_infill_filament.value);
                write_coll(lr->perimeters, 0);
                write_coll(lr->fills, 0);
                write_expolys(lr->raw_slices);
                ti(lr->slices.empty());
            }
        }
        ti((long long) obj->support_layers().size());
        for (const SupportLayer *sl : obj->support_layers()) {
            td(sl->print_z);
            write_coll(sl->support_fills, 0);
            ti(sl->planned_filament_id);
            ti(sl->planned_interface_filament_id);
        }
    }
}

static void write_sorted(Print *print, ToolOrdering &to, const Cfg &c)
{
    write_layer_tools_build(to);
    write_group(to.m_nozzle_group_result);
    for (auto *s : {&to.m_stats_by_single_extruder, &to.m_stats_by_multi_extruder_curr, &to.m_stats_by_multi_extruder_best}) {
        ti(s->filament_flush_weight);
        ti(s->flush_filament_change_count);
        ti(s->filament_change_count);
    }
    ti(to.m_most_used_extruder);
    auto g = print->get_layered_nozzle_group_result();
    ti(g != nullptr);
    if (g)
        write_group(*g);
    // The arguments of the write-back (Print::update_filament_maps_to_config or
    // update_to_config_by_nozzle_group_result), from the configuration before the call.
    if (!print->is_sequential_print()) {
        PrintConfig orig = make_cfg(c);
        std::vector<std::vector<unsigned int>> lf;
        for (auto &lt : to.m_layer_tools)
            lf.push_back(lt.extruders);
        auto used = collect_sorted_used_filaments(lf);
        const auto &gr = to.m_nozzle_group_result;
        if (!gr.is_support_dynamic_nozzle_map()) {
            tok("M");
            tlist_u(FilamentGroupUtils::update_used_filament_values(orig.filament_map.values, gr.get_extruder_map(false), used));
            tlist_u(FilamentGroupUtils::update_used_filament_values(orig.filament_volume_map.values, gr.get_volume_map(), used));
            bool keep = orig.handle_hotend_as_extruder && orig.filament_map_mode.value == fmmNozzleManual;
            tlist_u(keep ? orig.filament_nozzle_map.values : gr.get_nozzle_map());
        } else
            tok("G");
    }
}

static void case_sorted_build()
{
    for (int it = 0; it < 200; ++it) {
        int kind = ri(0, 1), nf = kind == 0 ? ri(1, 5) : ri(1, 4), ne = 0, nz = 0;
        std::string mode_name = std::vector<std::string>{"Auto For Flush", "Auto For Match", "Manual", "Auto For Quality"}[ri(0, 3)];
        Cfg c = rand_group_cfg(kind, nf, ne, nz, mode_name);
        std::vector<int> sol;
        for (int f = 0; f < nf; ++f)
            sol.push_back(rb(10));
        c.emplace_back("filament_soluble", join(sol));
        // Print::get_physical_unprintable_filaments (Print.cpp:3512) indexes filament_printable by the
        // filament without a bound check: one entry per filament, as the presets give it.
        std::vector<int> printable;
        for (int f = 0; f < nf; ++f)
            printable.push_back(rb(85) ? 3 : ri(1, 2));
        c.emplace_back("filament_printable", join(printable));
        // calc_max_layer_height (ToolOrdering.cpp:233) indexes max_layer_height by the extruder.
        std::vector<double> mlh;
        for (int e = 0; e < ne; ++e)
            mlh.push_back(rb(50) ? 0. : std::vector<double>{0.12, 0.2, 0.28, 0.32}[ri(0, 3)]);
        c.emplace_back("max_layer_height", join_d(mlh));
        c.emplace_back("enable_prime_tower", rb(50) ? "1" : "0");
        c.emplace_back("skirt_loops", rb(50) ? "1" : "0");
        c.emplace_back("initial_layer_line_width", "0.42");
        GroupPrintState st = rand_print_state(ne, nf, nz);
        // first_extruder is a 0-based filament (GCode.cpp passes initial_extruder_id), or -1 for none
        unsigned int first = rb(50) ? (unsigned int) -1 : unsigned(ri(0, nf - 1));
        bool         prime = rb(30);
        uint64_t seed = g_seed;
        Print *print = rand_build_print(c, st, nf);
        g_seed = seed;
        Print *print2 = rand_build_print(c, st, nf);

        tok("sorted_build");
        write_cfg(c);
        write_print_state(*print, st);
        write_build_print(print);
        ti((long long) first);
        ti(prime);
        arrow();
        try {
            ToolOrdering to(*print, first, prime);
            to.sort_and_build_data(*print, first, prime);
            tok("ok");
            write_sorted(print, to, c);
        } catch (const std::exception &e) {
            tok("ERR");
            ts(e.what());
        }
        try {
            PrintObject *obj = print2->m_objects.front();
            ToolOrdering to(*obj, first, prime);
            to.sort_and_build_data(*obj, first, prime);
            tok("ok");
            write_sorted(print2, to, c);
        } catch (const std::exception &e) {
            tok("ERR");
            ts(e.what());
        }
        endl_();
    }
}

int main()
{
    case_calc_max_layer_height();
    case_snap_to_simple_fraction();
    case_get_filament_by_type();
    case_check_filament_printable_after_group();
    case_calc_filament_change_info_by_toolorder();
    case_handle_dontcare();
    case_initialize_layers();
    case_fill_wipe_tower_partitions();
    case_mark_skirt_layers();
    case_collect_extruder_statistics();
    case_seed_first_tower_layer();
    case_cal_max_additional_fan();
    case_cal_non_support_filaments();
    case_calc_most_used_extruder();
    case_prepare_flush_matrices();
    case_nozzle_lists();
    case_create_custom_seq_function();
    case_validate_manual_diameter();
    case_slots_and_bed();
    case_tools_for_layer();
    case_resolve_mixed();
    case_assign_custom_gcodes();
    case_build();
    case_build_manual_nozzle_map();
    case_build_filament_group_context();
    case_get_recommended_filament_maps();
    case_refine_groups();
    case_plan_combo_ranges();
    case_sorted_build();
    return 0;
}
