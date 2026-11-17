// Generator of the vectors of tests/multi_nozzle_utils_native_parity.rs: libslic3r's MultiNozzleUtils.cpp at
// the pin (references/BambuStudio/src/libslic3r), its free functions and the member functions of
// LayeredNozzleGroupResult, StaticNozzleGroupResult, NozzleStatusRecorder, NozzleInfo and NozzleGroupInfo
// driven on random nozzle lists, filament change sequences, configurations and slice results.
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
//   ./gen > vectors.txt && gzip -9 -n -c vectors.txt > $DATA/vectors.txt.gz
//
// MultiNozzleUtils.cpp is included here so that the definitions compared are those of the pin's source
// (they replace the library's copy at link time). The run is deterministic (a fixed LCG).
//
// Line format: tokens separated by one space. A text token is the hexadecimal of its bytes ("-" for
// the empty text), a double its 16 hexadecimal digits, a float its 8, an integer is decimal. A list is
// its length then its items. Every line is
//   <op> <inputs...> => <outputs...>
// A nozzle is <diameter text> <volume_type> <extruder_id> <group_id> <installed>; a nozzle list is its
// length then its nozzles; an optional nozzle is "-" or "+" and the nozzle. A configuration is "C" <n>
// then n pairs <key> <value>: the options set with set_deserialize_strict() on top of the defaults of a
// full configuration ("D" instead of "C": on top of an empty DynamicPrintConfig).
// Volume types are kept within 0..3 (the four values the Rust enum has).

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <map>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include "libslic3r/MultiNozzleUtils.cpp"

using namespace Slic3r;
using namespace Slic3r::MultiNozzleUtils;

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
template<class T> static void tlist(const std::vector<T> &v)
{
    ti((long long) v.size());
    for (auto x : v)
        ti((long long) x);
}
static void tlist_f(const std::vector<float> &v)
{
    ti((long long) v.size());
    for (float x : v)
        tf(x);
}
static void tlist_d(const std::vector<double> &v)
{
    ti((long long) v.size());
    for (double x : v)
        td(x);
}
static void arrow() { tok("=>"); }
static void endl_() { out += '\n'; fputs(out.c_str(), stdout); fflush(stdout); out.clear(); }

static void tnozzle(const NozzleInfo &n)
{
    ts(n.diameter);
    ti((int) n.volume_type);
    ti(n.extruder_id);
    ti(n.group_id);
    ti(n.installed ? 1 : 0);
}
static void tnozzles(const std::vector<NozzleInfo> &v)
{
    ti((long long) v.size());
    for (auto &n : v)
        tnozzle(n);
}
static void topt(const std::optional<NozzleInfo> &n)
{
    if (!n) {
        tok("-");
        return;
    }
    tok("+");
    tnozzle(*n);
}

// ---------------------------------------------------------------------------------------------
// Deterministic random numbers
// ---------------------------------------------------------------------------------------------

static uint64_t g_seed = 0x2545f4914f6cdd1dull;
static uint32_t rnd()
{
    g_seed = g_seed * 6364136223846793005ull + 1442695040888963407ull;
    return uint32_t(g_seed >> 33);
}
static int    ri(int lo, int hi) { return lo + int(rnd() % uint32_t(hi - lo + 1)); }
static double rd(double lo, double hi) { return lo + (hi - lo) * (double(rnd()) / 2147483648.0); }
static bool   rb(int pct = 50) { return ri(0, 99) < pct; }

static const char *DIAMS[]     = {"0.2", "0.4", "0.6", "0.8"};
static const double DIAMS_D[]  = {0.2, 0.4, 0.6, 0.8};
static const char *VT_NAMES[]  = {"Standard", "High Flow", "Hybrid", "TPU High Flow"};

static NozzleInfo rnozzle(int idx)
{
    NozzleInfo n;
    n.diameter    = DIAMS[ri(0, 3)];
    n.volume_type = NozzleVolumeType(ri(0, 3));
    n.extruder_id = ri(0, 2);
    n.group_id    = rb(85) ? idx : ri(-1, 7);
    n.installed   = rb(85);
    return n;
}
static std::vector<NozzleInfo> rnozzles(int lo, int hi)
{
    std::vector<NozzleInfo> v;
    int n = ri(lo, hi);
    for (int i = 0; i < n; ++i)
        v.push_back(rnozzle(i));
    return v;
}

static void tparams(const FilamentChangeTimeParams &p)
{
    tf(p.selector_load_time);
    tf(p.selector_unload_time);
    tf(p.standard_load_time);
    tf(p.standard_unload_time);
}
static float rtime()
{
    switch (ri(0, 3)) {
    case 0: return float(ri(0, 60));
    case 1: return float(rd(0., 60.));
    case 2: return float(ri(0, 120)) * 0.5f;
    default: return float(rd(0., 30.)) * 1.1f;
    }
}
static FilamentChangeTimeParams rparams()
{
    FilamentChangeTimeParams p;
    p.selector_load_time   = rtime();
    p.selector_unload_time = rtime();
    p.standard_load_time   = rtime();
    p.standard_unload_time = rtime();
    return p;
}

// ---------------------------------------------------------------------------------------------
// Configurations
// ---------------------------------------------------------------------------------------------

typedef std::vector<std::pair<std::string, std::string>> Cfg;

static void write_cfg(const char *tag, const Cfg &c)
{
    tok(tag);
    ti((long long) c.size());
    for (auto &kv : c) {
        ts(kv.first);
        ts(kv.second);
    }
}

static void set_opts(DynamicPrintConfig &dc, const Cfg &c)
{
    for (auto &kv : c) {
        // The default value of an enum vector is built with the initializer_list constructor, which leaves
        // its key map uninitialized (Config.hpp:2135); give it the definition's before deserializing names.
        const ConfigOptionDef *def = print_config_def.get(kv.first);
        if (def != nullptr && def->type == coEnums)
            if (auto *o = dynamic_cast<ConfigOptionEnumsGeneric *>(dc.option(kv.first, true)))
                o->keys_map = def->enum_keys_map;
        dc.set_deserialize_strict(kv.first, kv.second);
    }
}

static PrintConfig make_cfg(const Cfg &c)
{
    DynamicPrintConfig dc = DynamicPrintConfig::full_print_config();
    set_opts(dc, c);
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

// ---------------------------------------------------------------------------------------------
// simulate_filament_change_time, calc_filament_change_gap_for_assignment, find_optimal_physical_assignment
// ---------------------------------------------------------------------------------------------

struct SimInput
{
    std::vector<int>        logical;
    std::vector<NozzleInfo> nozzles;
    std::vector<int>        fseq, nseq, groups;
    FilamentChangeTimeParams params;
    std::vector<bool>       preload;
};

static SimInput rsim(int max_fil, int max_seq)
{
    SimInput in;
    if (!rb(4)) {
        std::vector<int> pool;
        for (int i = 0; i < 10; ++i)
            pool.push_back(i);
        int n = ri(1, max_fil);
        for (int i = 0; i < n; ++i) {
            int j = ri(i, 9);
            std::swap(pool[i], pool[j]);
            in.logical.push_back(pool[i]);
        }
    }
    if (!rb(4))
        in.nozzles = rnozzles(1, 6);
    int groups = ri(1, 4);
    for (size_t i = 0; i < in.logical.size(); ++i)
        in.groups.push_back(rb(8) ? -1 : ri(0, groups - 1));
    int seq = rb(4) ? 0 : ri(1, max_seq);
    for (int i = 0; i < seq; ++i) {
        int f = (in.logical.empty() || rb(6)) ? ri(0, 11) : in.logical[ri(0, (int) in.logical.size() - 1)];
        int n = (in.nozzles.empty() || rb(6)) ? ri(-1, 7) : in.nozzles[ri(0, (int) in.nozzles.size() - 1)].group_id;
        in.fseq.push_back(f);
        in.nseq.push_back(n);
    }
    if (rb(10) && !in.nseq.empty())
        in.nseq.pop_back();
    in.params = rparams();
    int np = ri(0, 4);
    for (int i = 0; i < np; ++i)
        in.preload.push_back(rb(60));
    return in;
}

static void write_sim(const SimInput &in)
{
    tlist(in.logical);
    tnozzles(in.nozzles);
    tlist(in.fseq);
    tlist(in.nseq);
    tlist(in.groups);
    tparams(in.params);
    ti((long long) in.preload.size());
    for (bool b : in.preload)
        ti(b ? 1 : 0);
}

static void case_simulate()
{
    for (int c = 0; c < 3000; ++c) {
        SimInput in = rsim(6, 24);
        bool     sliced = rb(60);
        tok("sim");
        write_sim(in);
        ti(sliced ? 1 : 0);
        arrow();
        FilamentChangeSimResult r = simulate_filament_change_time(in.logical, in.nozzles, in.fseq, in.nseq, in.groups, in.params, in.preload, sliced);
        td(r.actual_time);
        td(r.sliced_time);
        endl_();
    }
    for (int c = 0; c < 1500; ++c) {
        SimInput in = rsim(6, 24);
        tok("gap");
        write_sim(in);
        arrow();
        tf(calc_filament_change_gap_for_assignment(in.logical, in.nozzles, in.fseq, in.nseq, in.groups, in.params, in.preload));
        endl_();
    }
    for (int c = 0; c < 400; ++c) {
        SimInput in = rsim(5, 16);
        int      group_count = ri(0, 3);
        tok("opt");
        tlist(in.logical);
        tnozzles(in.nozzles);
        tlist(in.fseq);
        tlist(in.nseq);
        ti(group_count);
        tparams(in.params);
        arrow();
        tlist(find_optimal_physical_assignment(in.logical, in.nozzles, in.fseq, in.nseq, group_count, in.params, 100000));
        endl_();
    }
}

// ---------------------------------------------------------------------------------------------
// build_nozzle_list (both overloads), build_nozzle_list_from_extruder_nozzles, resolve_manual_nozzle_map_to_index
// ---------------------------------------------------------------------------------------------

static void case_build_nozzle_list()
{
    for (int c = 0; c < 500; ++c) {
        std::vector<NozzleGroupInfo> groups;
        int n = ri(0, 6);
        for (int i = 0; i < n; ++i)
            groups.emplace_back(DIAMS[ri(0, 3)], NozzleVolumeType(ri(0, 3)), ri(0, 2), ri(0, 3));
        tok("bnl");
        ti(n);
        for (auto &g : groups) {
            ts(g.diameter);
            ti((int) g.volume_type);
            ti(g.extruder_id);
            ti(g.nozzle_count);
        }
        arrow();
        tnozzles(build_nozzle_list(groups));
        endl_();
    }
    for (int c = 0; c < 500; ++c) {
        double d = rb(70) ? DIAMS_D[ri(0, 3)] : rd(0.1, 1.2);
        int    n = ri(0, 8);
        std::vector<int> nozzle_map, volume_map, filament_map;
        for (int i = 0; i < n; ++i) {
            nozzle_map.push_back(ri(0, 4));
            volume_map.push_back(ri(0, 3));
            filament_map.push_back(ri(0, 2));
        }
        tok("bnl2");
        td(d);
        tlist(nozzle_map);
        tlist(volume_map);
        tlist(filament_map);
        arrow();
        tnozzles(build_nozzle_list(d, nozzle_map, volume_map, filament_map));
        endl_();
    }
}

static Cfg rnozzle_cfg()
{
    Cfg              c;
    int              extruders = ri(1, 2);
    std::vector<int> counts;
    int              total = 0;
    for (int e = 0; e < extruders; ++e) {
        counts.push_back(rb(10) ? 0 : ri(1, 4));
        total += counts.back();
    }
    std::vector<std::string> vts;
    int                      nvt = rb(85) ? total : ri(0, total + 1);
    for (int i = 0; i < nvt; ++i)
        vts.push_back(VT_NAMES[ri(0, 3)]);
    std::vector<double> diams;
    int                 nd = rb(60) ? total : ri(0, total + 1);
    for (int i = 0; i < nd; ++i)
        diams.push_back(DIAMS_D[ri(0, 3)]);
    std::vector<int> slots;
    if (rb(60)) {
        std::vector<int> pool;
        for (int i = 0; i < total + 3; ++i)
            pool.push_back(i);
        for (int i = 0; i < total; ++i) {
            int j = ri(i, (int) pool.size() - 1);
            std::swap(pool[i], pool[j]);
            slots.push_back(pool[i]);
        }
        if (rb(10) && !slots.empty())
            slots[ri(0, (int) slots.size() - 1)] = ri(-2, total + 4);
    } else if (rb(20)) {
        int ns = ri(0, total + 1);
        for (int i = 0; i < ns; ++i)
            slots.push_back(ri(0, total + 2));
    }
    std::vector<double> nozzle_diameter;
    for (int e = 0; e < extruders; ++e)
        nozzle_diameter.push_back(DIAMS_D[ri(0, 3)]);
    std::vector<int> maxc;
    for (int e = 0; e < extruders; ++e)
        maxc.push_back(ri(1, 6));
    if (!counts.empty() || rb(80))
        c.push_back({"extruder_nozzle_count", join(counts)});
    c.push_back({"extruder_nozzle_volume_type", join(vts)});
    c.push_back({"extruder_nozzle_diameter", join(diams)});
    c.push_back({"extruder_nozzle_slot", join(slots)});
    c.push_back({"nozzle_diameter", join(nozzle_diameter)});
    c.push_back({"extruder_max_nozzle_count", join(maxc)});
    c.push_back({"handle_hotend_as_extruder", rb(50) ? "1" : "0"});
    // The vector options with an empty value: an empty list.
    Cfg r;
    for (auto &kv : c)
        if (!kv.second.empty())
            r.push_back(kv);
        else if (kv.first != "extruder_nozzle_count" || rb(50))
            r.push_back(kv);
    return r;
}

static void case_extruder_nozzles()
{
    for (int c = 0; c < 1500; ++c) {
        Cfg         cfg = rnozzle_cfg();
        PrintConfig pc  = make_cfg(cfg);
        tok("bxn");
        write_cfg("C", cfg);
        arrow();
        auto list = build_nozzle_list_from_extruder_nozzles(pc);
        tnozzles(list);
        endl_();

        int                       filaments = ri(1, 7);
        std::vector<unsigned int> used;
        for (int f = 0; f < filaments; ++f)
            if (rb(70))
                used.push_back(f);
        if (rb(5))
            used.push_back(filaments + ri(0, 2));
        std::vector<int> fmap, vmap, kmap;
        int nf = rb(90) ? filaments : ri(0, filaments + 1);
        for (int f = 0; f < nf; ++f)
            fmap.push_back(ri(1, 2));
        int nv = rb(90) ? filaments : ri(0, filaments + 1);
        for (int f = 0; f < nv; ++f)
            vmap.push_back(ri(0, 3));
        int nk = rb(90) ? filaments : ri(0, filaments + 1);
        bool as_index = rb(40) && !list.empty();
        for (int f = 0; f < nk; ++f)
            kmap.push_back(as_index ? ri(0, (int) list.size() - 1) : ri(-1, 6));
        tok("rmn");
        write_cfg("C", cfg);
        tlist(used);
        tlist(fmap);
        tlist(vmap);
        tlist(kmap);
        arrow();
        tlist(resolve_manual_nozzle_map_to_index(pc, used, fmap, vmap, kmap));
        endl_();
    }
}

// ---------------------------------------------------------------------------------------------
// load_nozzle_infos_with_compatibility
// ---------------------------------------------------------------------------------------------

static const char *VT_STRS[] = {"Standard", "High Flow", "Hybrid", "TPU High Flow", "Unknown", ""};

static FilamentInfo rfilament_info(int id)
{
    FilamentInfo f;
    f.id = id;
    int ng = rb(70) ? 1 : ri(0, 3);
    for (int i = 0; i < ng; ++i)
        f.group_id.push_back(ri(-1, 4));
    f.nozzle_diameter    = rb(80) ? DIAMS_D[ri(0, 3)] : rd(0.1, 1.0);
    f.nozzle_volume_type = VT_STRS[ri(0, 5)];
    return f;
}

static void write_filament_infos(const std::vector<FilamentInfo> &v)
{
    ti((long long) v.size());
    for (auto &f : v) {
        ti(f.id);
        tlist(f.group_id);
        td(f.nozzle_diameter);
        ts(f.nozzle_volume_type);
    }
}

static void case_load_nozzle_infos()
{
    for (int c = 0; c < 800; ++c) {
        std::vector<NozzleInfo> nozzles;
        if (rb(40))
            nozzles = rnozzles(1, 5);
        int                       nf = ri(0, 5);
        std::vector<FilamentInfo> infos;
        for (int i = 0; i < nf; ++i)
            infos.push_back(rfilament_info(i));
        std::vector<int> fmap;
        for (int i = 0; i < nf; ++i)
            fmap.push_back(ri(1, 2));
        int                           ne = ri(1, 3);
        std::vector<NozzleVolumeType> vts;
        int                           nvt = ri(0, ne + 1);
        for (int i = 0; i < nvt; ++i)
            vts.push_back(NozzleVolumeType(ri(0, 3)));
        std::vector<double> nd;
        for (int i = 0; i < ne; ++i)
            nd.push_back(rb(80) ? DIAMS_D[ri(0, 3)] : rd(0.1, 1.0));
        tok("lnc");
        tnozzles(nozzles);
        write_filament_infos(infos);
        tlist(fmap);
        ti((long long) vts.size());
        for (auto v : vts)
            ti((int) v);
        tlist_d(nd);
        arrow();
        tnozzles(load_nozzle_infos_with_compatibility(nozzles, infos, fmap, vts, nd));
        endl_();
    }
}

// ---------------------------------------------------------------------------------------------
// LayeredNozzleGroupResult
// ---------------------------------------------------------------------------------------------

static void dump_layered(const std::optional<LayeredNozzleGroupResult> &ro, int filaments, int layers, int nozzles)
{
    if (!ro) {
        ti(0);
        return;
    }
    ti(1);
    const LayeredNozzleGroupResult &r = *ro;
    ti(r.is_support_dynamic_nozzle_map() ? 1 : 0);
    ti(r.is_handle_nozzle_as_extruder() ? 1 : 0);
    ti(r.get_extruder_count());
    ti((long long) r.get_layer_count());
    tlist(r.get_used_filaments());
    tlist(r.get_used_extruders());
    for (int e = -1; e <= 2; ++e)
        tnozzles(r.get_used_nozzles_in_extruder(e));
    for (int l = -1; l <= layers; ++l) {
        tlist(r.get_layer_filament_nozzle_map(l));
        tlist(r.get_used_filaments(l));
        tlist(r.get_used_extruders(l));
        for (int e = -1; e <= 2; ++e)
            tnozzles(r.get_used_nozzles_in_extruder(e, l));
        tlist(r.get_extruder_map(true, l));
        tlist(r.get_extruder_map(false, l));
        tlist(r.get_nozzle_map(l));
        tlist(r.get_volume_map(l));
        for (int f = -1; f <= filaments; ++f) {
            topt(r.get_nozzle_for_filament(f, l));
            ti(r.get_extruder_id(f, l));
            ti(r.get_nozzle_id(f, l));
            for (int g = 0; g < filaments; ++g) {
                ti(r.are_filaments_same_extruder(f, g, l) ? 1 : 0);
                ti(r.are_filaments_same_nozzle(f, g, l) ? 1 : 0);
            }
        }
    }
    for (int f = -1; f <= filaments; ++f) {
        tnozzles(r.get_nozzles_for_filament(f));
        topt(r.get_first_nozzle_for_filament(f));
    }
    for (int n = -1; n <= nozzles; ++n)
        topt(r.get_nozzle_from_id(n));
    tlist(r.get_layer_filament_sequences().size() > 0 ? r.get_layer_filament_sequences().front() : std::vector<unsigned int>());
}

static std::vector<unsigned int> rused(int filaments)
{
    std::vector<unsigned int> used;
    for (int f = 0; f < filaments; ++f)
        if (rb(75))
            used.push_back(f);
    if (rb(5))
        used.push_back(filaments + ri(0, 1));
    return used;
}

static void case_layered()
{
    // create(filament_nozzle_map, nozzle_list, used_filaments, handle_nozzle_as_extruder)
    for (int c = 0; c < 400; ++c) {
        int                     filaments = ri(0, 5);
        auto                    nozzles   = rb(5) ? std::vector<NozzleInfo>() : rnozzles(1, 5);
        std::vector<int>        map;
        int                     nm = rb(90) ? filaments : ri(0, filaments + 1);
        for (int f = 0; f < nm; ++f)
            map.push_back(ri(-1, (int) nozzles.size()));
        auto used   = rused(filaments);
        bool handle = rb(30);
        tok("lc1");
        tlist(map);
        tnozzles(nozzles);
        tlist(used);
        ti(handle ? 1 : 0);
        ti(filaments);
        arrow();
        dump_layered(LayeredNozzleGroupResult::create(map, nozzles, used, handle), filaments, 0, (int) nozzles.size());
        endl_();
    }
    // create(layer_filament_nozzle_maps, nozzle_list, used_filaments, layer_filament_sequences, handle)
    for (int c = 0; c < 400; ++c) {
        int  filaments = ri(0, 5);
        auto nozzles   = rb(5) ? std::vector<NozzleInfo>() : rnozzles(1, 5);
        int  layers    = rb(5) ? 0 : ri(1, 4);
        std::vector<std::vector<int>>          maps;
        std::vector<std::vector<unsigned int>> seqs;
        for (int l = 0; l < layers; ++l) {
            std::vector<int> m;
            int              nm = rb(90) ? filaments : ri(0, filaments + 1);
            for (int f = 0; f < nm; ++f)
                m.push_back(rb(50) && l > 0 && f < (int) maps[l - 1].size() ? maps[l - 1][f] : ri(-1, (int) nozzles.size()));
            maps.push_back(m);
        }
        int nseq = rb(80) ? layers : ri(0, layers + 1);
        for (int l = 0; l < nseq; ++l) {
            std::vector<unsigned int> s;
            for (int f = 0; f < filaments; ++f)
                if (rb(60))
                    s.push_back(f);
            seqs.push_back(s);
        }
        auto used   = rused(filaments);
        bool handle = rb(30);
        tok("lc2");
        ti((long long) maps.size());
        for (auto &m : maps)
            tlist(m);
        tnozzles(nozzles);
        tlist(used);
        ti((long long) seqs.size());
        for (auto &s : seqs)
            tlist(s);
        ti(handle ? 1 : 0);
        ti(filaments);
        arrow();
        dump_layered(LayeredNozzleGroupResult::create(maps, nozzles, used, seqs, handle), filaments, layers, (int) nozzles.size());
        endl_();
    }
    // create(used_filaments, filament_map, filament_volume_map, filament_nozzle_map, nozzle_count, diameter, handle)
    for (int c = 0; c < 400; ++c) {
        int filaments = ri(1, 5);
        int extruders = ri(1, 2);
        std::vector<std::map<NozzleVolumeType, int>> counts(extruders);
        for (int e = 0; e < extruders; ++e) {
            int nt = ri(0, 2);
            for (int t = 0; t < nt; ++t)
                counts[e][NozzleVolumeType(ri(0, 3))] = ri(0, 3);
        }
        std::vector<int> fmap, vmap, nmap;
        for (int f = 0; f < filaments; ++f) {
            fmap.push_back(ri(0, extruders - 1));
            vmap.push_back(ri(0, 3));
            nmap.push_back(ri(0, 4));
            // Mostly enough nozzles of the requested kind, so that the assignment can succeed.
            if (rb(80))
                counts[fmap.back()][NozzleVolumeType(vmap.back())] += ri(0, 2);
        }
        std::vector<unsigned int> used;
        for (int f = 0; f < filaments; ++f)
            if (rb(75))
                used.push_back(f);
        float diameter = float(DIAMS_D[ri(0, 3)]);
        bool  handle   = rb(30);
        tok("lc3");
        tlist(used);
        tlist(fmap);
        tlist(vmap);
        tlist(nmap);
        ti(extruders);
        for (auto &m : counts) {
            ti((long long) m.size());
            for (auto &kv : m) {
                ti((int) kv.first);
                ti(kv.second);
            }
        }
        tf(diameter);
        ti(handle ? 1 : 0);
        ti(filaments);
        arrow();
        auto r = LayeredNozzleGroupResult::create(used, fmap, vmap, nmap, counts, diameter, handle);
        int  total = 0;
        for (auto &m : counts)
            for (auto &kv : m)
                total += kv.second;
        dump_layered(r, filaments, 0, total);
        endl_();
    }
    // estimate_seq_flush_weight
    for (int c = 0; c < 400; ++c) {
        int  filaments = ri(1, 5);
        auto nozzles   = rnozzles(1, 4);
        std::vector<int> map;
        for (int f = 0; f < filaments; ++f)
            map.push_back(ri(-1, (int) nozzles.size()));
        std::vector<unsigned int> used;
        for (int f = 0; f < filaments; ++f)
            used.push_back(f);
        int extruders = ri(0, 3);
        std::vector<std::vector<std::vector<float>>> fm(extruders);
        for (auto &m : fm) {
            int rows = rb(85) ? filaments : ri(0, filaments + 1);
            m.resize(rows);
            for (auto &row : m) {
                int cols = rb(85) ? filaments : ri(0, filaments + 1);
                for (int k = 0; k < cols; ++k)
                    row.push_back(rb(70) ? float(ri(0, 900)) : float(rd(0., 800.)));
            }
        }
        std::vector<int> seq;
        int              ns = ri(0, 20);
        for (int i = 0; i < ns; ++i)
            seq.push_back(ri(-1, filaments));
        tok("esw");
        tlist(map);
        tnozzles(nozzles);
        tlist(used);
        ti((long long) fm.size());
        for (auto &m : fm) {
            ti((long long) m.size());
            for (auto &row : m)
                tlist_f(row);
        }
        tlist(seq);
        arrow();
        auto r = LayeredNozzleGroupResult::create(map, nozzles, used);
        ti(r ? r->estimate_seq_flush_weight(fm, seq) : -999999);
        endl_();
    }
}

// ---------------------------------------------------------------------------------------------
// StaticNozzleGroupResult
// ---------------------------------------------------------------------------------------------

static void case_static()
{
    for (int c = 0; c < 600; ++c) {
        int                       nf = ri(0, 5);
        std::vector<FilamentInfo> infos;
        for (int i = 0; i < nf; ++i)
            infos.push_back(rfilament_info(rb(85) ? i : ri(-1, 6)));
        auto             nozzles = rb(5) ? std::vector<NozzleInfo>() : rnozzles(1, 5);
        std::vector<int> fseq, nseq;
        int              ns = ri(0, 8);
        for (int i = 0; i < ns; ++i) {
            fseq.push_back(ri(-1, 6));
            nseq.push_back(ri(-1, 6));
        }
        if (rb(15) && !nseq.empty())
            nseq.pop_back();
        bool support = rb(50);
        tok("sc");
        write_filament_infos(infos);
        tnozzles(nozzles);
        tlist(fseq);
        tlist(nseq);
        ti(support ? 1 : 0);
        arrow();
        auto r = StaticNozzleGroupResult::create(infos, nozzles, fseq, nseq, support);
        if (!r) {
            ti(0);
        } else {
            ti(1);
            ti(r->is_support_dynamic_nozzle_map() ? 1 : 0);
            ti(r->get_extruder_count());
            for (int e = -1; e <= 2; ++e)
                tnozzles(r->get_used_nozzles_in_extruder(e));
            tlist(r->get_used_extruders());
            tlist(r->get_used_filaments());
            for (int n = -1; n <= 7; ++n)
                topt(r->get_nozzle_from_id(n));
            for (int f = -1; f <= 6; ++f) {
                tnozzles(r->get_nozzles_for_filament(f));
                topt(r->get_first_nozzle_for_filament(f));
            }
        }
        endl_();
    }
}

// ---------------------------------------------------------------------------------------------
// NozzleStatusRecorder
// ---------------------------------------------------------------------------------------------

static void dump_recorder(const NozzleStatusRecorder &r)
{
    for (int n = -1; n <= 12; ++n) {
        ti(r.is_nozzle_empty(n) ? 1 : 0);
        ti(r.get_filament_in_nozzle(n));
    }
    for (int e = -1; e <= 4; ++e)
        ti(r.get_nozzle_in_extruder(e));
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

static void case_recorder()
{
    // Ops: "s" <which> <nozzle> <filament> <extruder>, "c" <which> <nozzle>, "x" <which> <extruder>,
    // "A" (r0 = r1, copy assignment), "K" (r1 = NozzleStatusRecorder(r0), copy construction then move).
    for (int c = 0; c < 800; ++c) {
        NozzleStatusRecorder r[2];
        int                  nops = ri(0, 60);
        int                  range = rb(50) ? 12 : 40;
        tok("rec");
        ti(nops);
        for (int i = 0; i < nops; ++i) {
            int k = ri(0, 99);
            int w = ri(0, 1);
            if (k < 55) {
                int n = ri(-1, range), f = ri(-1, 9), e = rb(30) ? -1 : ri(0, 3);
                tok("s");
                ti(w);
                ti(n);
                ti(f);
                ti(e);
                r[w].set_nozzle_status(n, f, e);
            } else if (k < 80) {
                int n = ri(-1, range);
                tok("c");
                ti(w);
                ti(n);
                r[w].clear_nozzle_status(n);
            } else if (k < 88) {
                int e = ri(-1, 3);
                tok("x");
                ti(w);
                ti(e);
                r[w].set_current_extruder_id(e);
            } else if (k < 94) {
                tok("A");
                r[0] = r[1];
            } else {
                tok("K");
                r[1] = NozzleStatusRecorder(r[0]);
            }
        }
        arrow();
        dump_recorder(r[0]);
        dump_recorder(r[1]);
        endl_();
    }
}

// ---------------------------------------------------------------------------------------------
// std::unordered_map<int, int> (the order of iteration the port reproduces in UnorderedMapInt)
// ---------------------------------------------------------------------------------------------

static void dump_umap(const std::unordered_map<int, int> &m)
{
    ti((long long) m.size());
    ti((long long) m.bucket_count());
    for (auto &kv : m) {
        ti(kv.first);
        ti(kv.second);
    }
}

static void case_unordered_map()
{
    // Ops: "o" <which> <k> <v> (m[k] = v), "i" <which> <k> <v> (insert / emplace), "e" <which> <k> (erase),
    // "r" <which> <n> (reserve), "c" <which> (clear), "A" (m0 = m1), "K" (m1 = unordered_map(m0)),
    // "B" <n> (m1 = unordered_map(n)).
    for (int c = 0; c < 1200; ++c) {
        std::unordered_map<int, int> m[2];
        int                          nops  = ri(0, 80);
        int                          range = rb(40) ? 16 : (rb(50) ? 100 : 100000);
        tok("umap");
        ti(nops);
        for (int i = 0; i < nops; ++i) {
            int k = ri(0, 99);
            int w = ri(0, 1);
            int key = rb(5) ? -ri(1, 5) : ri(0, range);
            if (k < 40) {
                int v = ri(-5, 50);
                tok("o");
                ti(w);
                ti(key);
                ti(v);
                m[w][key] = v;
            } else if (k < 55) {
                int v = ri(-5, 50);
                tok("i");
                ti(w);
                ti(key);
                ti(v);
                m[w].insert({key, v});
            } else if (k < 75) {
                tok("e");
                ti(w);
                ti(key);
                m[w].erase(key);
            } else if (k < 83) {
                int n = ri(0, 40);
                tok("r");
                ti(w);
                ti(n);
                m[w].reserve(n);
            } else if (k < 87) {
                tok("c");
                ti(w);
                m[w].clear();
            } else if (k < 92) {
                tok("A");
                m[0] = m[1];
            } else if (k < 97) {
                tok("K");
                m[1] = std::unordered_map<int, int>(m[0]);
            } else {
                int n = ri(0, 30);
                tok("B");
                ti(n);
                m[1] = std::unordered_map<int, int>(n);
            }
        }
        arrow();
        dump_umap(m[0]);
        dump_umap(m[1]);
        endl_();
    }
}

// ---------------------------------------------------------------------------------------------
// NozzleInfo / NozzleGroupInfo (de)serialization, resolve_ams_load_unload
// ---------------------------------------------------------------------------------------------

static void case_serialize()
{
    for (int c = 0; c < 400; ++c) {
        NozzleInfo n = rnozzle(ri(0, 9));
        tok("nis");
        tnozzle(n);
        arrow();
        ts(n.serialize());
        endl_();

        NozzleGroupInfo g(rb(80) ? DIAMS[ri(0, 3)] : "", NozzleVolumeType(ri(0, 3)), ri(-1, 3), ri(-2, 9));
        tok("ngs");
        ts(g.diameter);
        ti((int) g.volume_type);
        ti(g.extruder_id);
        ti(g.nozzle_count);
        arrow();
        ts(g.serialize());
        endl_();
    }
    static const char *PIECES[] = {"0", "1", "-1", "12", "x", " 3", "+4", "7a", "", "0.4", "0.2", "Standard", "High Flow",
                                   "Hybrid", "TPU High Flow", "TPU", "HighFlow", "standard", "99999999999", "2"};
    // (No "TPU285" / "E3D High Flow": the Rust NozzleVolumeType has the four values of an earlier pin.)
    for (int c = 0; c < 1200; ++c) {
        std::string s;
        if (rb(50)) {
            // A serialized group with some damage.
            NozzleGroupInfo g(DIAMS[ri(0, 3)], NozzleVolumeType(ri(0, 3)), ri(0, 3), ri(0, 9));
            s = g.serialize();
            int k = ri(0, 4);
            if (k == 1)
                s += "-";
            else if (k == 2 && !s.empty())
                s.erase(ri(0, (int) s.size() - 1), 1);
            else if (k == 3)
                s = "-" + s;
        } else {
            int np = ri(0, 6);
            for (int i = 0; i < np; ++i) {
                if (i)
                    s += "-";
                s += PIECES[ri(0, 19)];
            }
        }
        tok("ngd");
        ts(s);
        arrow();
        auto r = NozzleGroupInfo::deserialize(s);
        if (!r)
            ti(0);
        else {
            ti(1);
            ts(r->diameter);
            ti((int) r->volume_type);
            ti(r->extruder_id);
            ti(r->nozzle_count);
        }
        endl_();
    }
    for (int c = 0; c < 400; ++c) {
        std::vector<double> loads, unloads;
        int                 nl = ri(0, 4), nu = ri(0, 4);
        for (int i = 0; i < nl; ++i)
            loads.push_back(rb(50) ? double(ri(0, 60)) : rd(0., 60.));
        for (int i = 0; i < nu; ++i)
            unloads.push_back(rb(50) ? double(ri(0, 60)) : rd(0., 60.));
        int   type = ri(-2, 4);
        float ll = rtime(), lu = rtime();
        tok("ams");
        tlist_d(loads);
        tlist_d(unloads);
        ti(type);
        tf(ll);
        tf(lu);
        arrow();
        float ol = -1.f, ou = -1.f;
        bool  ok = resolve_ams_load_unload(loads, unloads, type, ll, lu, ol, ou);
        ti(ok ? 1 : 0);
        tf(ol);
        tf(ou);
        endl_();
    }
}

// ---------------------------------------------------------------------------------------------
// estimate_mapped_print_time
// ---------------------------------------------------------------------------------------------

static std::string fmt_double(double v)
{
    char buf[64];
    snprintf(buf, sizeof(buf), "%.17g", v);
    return buf;
}

static void case_estimate()
{
    static const char *KEYS[] = {"ams_filament_load_time_ams", "ams_filament_unload_time_ams", "ams_filament_load_time_ams_lite",
                                 "ams_filament_unload_time_ams_lite", "ams_filament_load_time_n3f_s", "ams_filament_unload_time_n3f_s"};
    for (int c = 0; c < 800; ++c) {
        bool full = rb(40);
        Cfg  cfg;
        if (!rb(15))
            cfg.push_back({"default_ams_type", std::to_string(ri(-1, 4))});
        for (const char *k : KEYS)
            if (full || rb(70))
                cfg.push_back({k, rb(50) ? std::to_string(ri(0, 60)) : fmt_double(rd(0., 60.))});
        if (full || rb(70))
            cfg.push_back({"machine_load_filament_time", rb(50) ? std::to_string(ri(0, 60)) : fmt_double(rd(0., 60.))});
        if (full || rb(70))
            cfg.push_back({"machine_unload_filament_time", rb(50) ? std::to_string(ri(0, 60)) : fmt_double(rd(0., 60.))});
        DynamicPrintConfig dc = full ? DynamicPrintConfig::full_print_config() : DynamicPrintConfig();
        set_opts(dc, cfg);

        GCodeProcessorResult res;
        float                t = rb(10) ? -float(ri(1, 100)) : (rb(50) ? float(ri(0, 20000)) : float(rd(0., 20000.)));
        res.print_statistics.modes[size_t(PrintEstimatedStatistics::ETimeMode::Normal)].time = t;
        int nseq = ri(0, 12);
        for (int i = 0; i < nseq; ++i)
            res.filament_change_sequence.push_back(ri(0, 6));
        std::vector<int> types;
        int              nt = rb(70) ? 7 : ri(0, 7);
        for (int i = 0; i < nt; ++i)
            types.push_back(rb(5) ? -1 : ri(0, 4));
        int null_flags = rb(10) ? ri(1, 3) : 0;

        tok("emp");
        write_cfg(full ? "C" : "D", cfg);
        tf(t);
        tlist(res.filament_change_sequence);
        tlist(types);
        ti(null_flags);
        arrow();
        PrintTimeRecalcResult r = estimate_mapped_print_time((null_flags & 1) ? nullptr : &res, (null_flags & 2) ? nullptr : &dc, types);
        tf(r.total_seconds);
        tf(r.gap_seconds);
        ti((int) r.reason);
        endl_();
    }
}

int main()
{
    case_simulate();
    case_build_nozzle_list();
    case_extruder_nozzles();
    case_load_nozzle_infos();
    case_layered();
    case_static();
    case_recorder();
    case_unordered_map();
    case_serialize();
    case_estimate();
    return 0;
}
