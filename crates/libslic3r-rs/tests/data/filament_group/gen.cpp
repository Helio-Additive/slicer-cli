// Generator of the vectors of tests/filament_group_native_parity.rs: libslic3r's FilamentGroup.cpp at the
// pin (references/BambuStudio/src/libslic3r), its file-static functions, the members of FilamentGroup,
// KMediods, FlushDistanceEvaluator, TimeEvaluator and FilamentGroupContext, and the free grouping
// functions, driven on random grouping contexts (filaments, nozzles, flush matrices, layers, machine
// filaments, loaded filaments, print times, nozzle states).
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
// FilamentGroup.cpp is included here with private and protected members made public, so that its
// file-static functions and the private members are reachable; its definitions replace those of the
// library's copy at link time. The run is deterministic (a fixed LCG); the time limits of KMediods are
// given values no case reaches.
//
// Line format: tokens separated by one space. A text token is the hexadecimal of its bytes ("-" for the
// empty text), a double its 16 hexadecimal digits, a float its 8, an integer is decimal. A list is its
// length then its items. Every line is
//   <op> <inputs...> => <outputs...>
// except "ctx <context>", which sets the grouping context the following lines use (the writer
// write_ctx below gives its layout; the two unordered maps are written in their insertion order).
// A C++ exception is the output "EXC <kind> <what()>" (kind 0 FilamentGroupException, 1
// RuntimeError, 2 out_of_range).

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <queue>
#include <random>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <deque>

#define private public
#define protected public
#include "libslic3r/FilamentGroup.cpp"
#undef protected
#undef private

using namespace Slic3r;
using namespace Slic3r::FilamentGroupUtils;

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
static void tu(unsigned long long v) { tok(std::to_string(v)); }
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
static void tset(const std::set<int> &v)
{
    ti((long long) v.size());
    for (auto x : v)
        ti(x);
}
static void arrow() { tok("=>"); }
static void endl_() { out += '\n'; fputs(out.c_str(), stdout); fflush(stdout); out.clear(); }

static void exc(int kind, const char *what)
{
    tok("EXC");
    ti(kind);
    ts(what);
}

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
// Contexts
// ---------------------------------------------------------------------------------------------

struct Gen
{
    FilamentGroupContext ctx;
    // insertion orders of the unordered maps
    std::vector<std::pair<int, std::vector<std::pair<int, double>>>> print_time_order;
    std::vector<std::pair<int, int>> status_order;
};

static const char *TYPES[]    = {"PLA", "PETG", "TPU", "ABS"};
static const char *FIL_IDS[]  = {"GFA00", "GFL99", "", "GFA17"};
static const char *DIAMS[]    = {"0.4", "0.2", "0.6"};

static FilamentGroupUtils::FilamentInfo rand_filament_info()
{
    FilamentGroupUtils::FilamentInfo f;
    f.color      = Color((unsigned char) ri(0, 255), (unsigned char) ri(0, 255), (unsigned char) ri(0, 255), (unsigned) (rb(80) ? 255 : ri(0, 255)));
    if (rb(30)) // a few equal colours
        f.color = Color(rb(50) ? 0 : 255, rb(50) ? 0 : 255, 0, 255);
    f.type       = TYPES[ri(0, 3)];
    f.is_support = rb(20);
    f.usage_type = FilamentUsageType(ri(0, 2));
    f.diameter   = rb(70) ? 1.75f : (rb(50) ? 2.85f : 0.f);
    return f;
}

static Gen rand_ctx(int max_fil, int max_noz, int max_layers, bool single)
{
    Gen g;
    FilamentGroupContext &c = g.ctx;
    const int F = ri(1, max_fil);
    const int E = ri(1, 2);
    const int N = ri(1, max_noz);

    // nozzles
    for (int i = 0; i < N; ++i) {
        MultiNozzleUtils::NozzleInfo n;
        n.diameter    = rb(80) ? "0.4" : DIAMS[ri(0, 2)];
        n.volume_type = NozzleVolumeType(rb(60) ? 0 : ri(0, 3));
        n.extruder_id = (i < E) ? i : ri(0, E - 1);
        n.group_id    = i;
        n.installed   = rb(90);
        c.nozzle_info.nozzle_list.push_back(n);
    }
    c.nozzle_info.extruder_nozzle_list = build_extruder_nozzle_list(c.nozzle_info.nozzle_list);
    if (rb(70))
        for (int i = 0; i < N; ++i)
            c.nozzle_info.per_nozzle_capacity.push_back(c.nozzle_info.nozzle_list[i].installed ? ri(1, 4) : 0);
    if (rb(50)) {
        int k = ri(1, N);
        for (int j = 0; j < k; ++j) {
            int nid = ri(0, N - 1), fid = rb(15) ? -1 : ri(0, F - 1);
            g.status_order.emplace_back(nid, fid);
            c.nozzle_info.nozzle_status[nid] = fid;
        }
    }

    // model
    for (int e = 0; e < E; ++e) {
        FlushMatrix fm(F, std::vector<float>(F, 0.f));
        for (int a = 0; a < F; ++a)
            for (int b = 0; b < F; ++b)
                fm[a][b] = a == b ? 0.f : (rb(50) ? float(ri(0, 800)) : float(rd(0, 800)));
        c.model_info.flush_matrix.push_back(fm);
    }
    int L = ri(1, max_layers);
    for (int l = 0; l < L; ++l) {
        std::vector<unsigned int> lf;
        // at least one filament is used: with none, ToolOrderUtils.cpp:1653 dereferences the end of an
        // empty vector (undefined behaviour)
        if (l == 0 || !rb(10)) {
            int k = single ? 1 : ri(1, std::min(F, 4));
            std::set<unsigned int> s;
            for (int j = 0; j < k; ++j)
                s.insert(unsigned(ri(0, F - 1)));
            lf.assign(s.begin(), s.end());
            if (rb(50))
                std::reverse(lf.begin(), lf.end());
        }
        c.model_info.layer_filaments.push_back(lf);
    }
    for (int f = 0; f < F; ++f) {
        c.model_info.filament_info.push_back(rand_filament_info());
        c.model_info.filament_ids.push_back(FIL_IDS[ri(0, 3)]);
        c.model_info.selected_nozzle_diameters.push_back(rb(80) ? "" : DIAMS[ri(0, 1)]);
    }
    c.model_info.unprintable_filaments.resize(2);
    for (int e = 0; e < 2; ++e)
        if (rb(30))
            c.model_info.unprintable_filaments[e].insert(ri(0, F - 1));
    if (rb(30)) {
        int fid = ri(0, F - 1);
        c.model_info.unprintable_volumes[fid].insert(NozzleVolumeType(ri(0, 3)));
    }

    // group
    c.group_info.total_filament_num   = F;
    c.group_info.max_gap_threshold    = rb(70) ? 0.01 : rd(0, 0.2);
    c.group_info.mode                 = rb(70) ? FGMode::FlushMode : FGMode::MatchMode;
    c.group_info.strategy             = rb(70) ? FGStrategy::BestCost : FGStrategy::BestFit;
    c.group_info.ignore_ext_filament  = rb(20);
    c.group_info.has_filament_switcher = rb(10);
    for (int f = 0; f < F; ++f)
        c.group_info.filament_volume_map.push_back(rb(80) ? 2 : ri(0, 3));

    // machine
    for (int e = 0; e < E; ++e)
        c.machine_info.max_group_size.push_back(ri(1, 8));
    c.machine_info.machine_filament_info.resize(2);
    for (int e = 0; e < 2; ++e) {
        int k = rb(25) ? 0 : ri(1, 4);
        for (int j = 0; j < k; ++j) {
            MachineFilamentInfo m;
            static_cast<FilamentGroupUtils::FilamentInfo &>(m) = rand_filament_info();
            m.extruder_id = e;
            m.is_extended = rb(20);
            m.filament_id = FIL_IDS[ri(0, 3)];
            c.machine_info.machine_filament_info[e].push_back(m);
        }
    }
    {
        int k = rb(30) ? 0 : ri(1, N + 1);
        for (int j = 0; j < k; ++j) {
            NozzleLoadedFilamentInfo m;
            static_cast<FilamentGroupUtils::FilamentInfo &>(m) = rand_filament_info();
            m.nozzle_id   = ri(0, N - 1);
            m.filament_id = FIL_IDS[ri(0, 3)];
            m.valid       = rb(85);
            c.machine_info.nozzle_loaded_filaments.push_back(m);
        }
    }
    for (int i = 0; i < std::max(N, 2); ++i)
        c.machine_info.prefer_non_model_filament.push_back(rb(25));
    c.machine_info.master_extruder_id = ri(0, E - 1);

    // speed
    if (rb(60)) {
        int k = ri(1, F);
        for (int j = 0; j < k; ++j) {
            int fid = ri(0, F - 1);
            std::vector<std::pair<int, double>> inner;
            int m = ri(1, std::max(N, E));
            for (int q = 0; q < m; ++q) {
                int key = ri(0, std::max(N, E) - 1);
                double t = rd(0, 500);
                inner.emplace_back(key, t);
                c.speed_info.filament_print_time[fid][key] = t;
            }
            g.print_time_order.emplace_back(fid, inner);
        }
    }
    c.speed_info.extruder_change_time = rd(0, 30);
    c.speed_info.filament_change_time = rd(0, 60);
    c.speed_info.group_with_time      = rb(40);
    c.speed_info.change_time_params.selector_load_time   = float(rd(0, 20));
    c.speed_info.change_time_params.selector_unload_time = float(rd(0, 20));
    c.speed_info.change_time_params.standard_load_time   = float(rd(10, 40));
    c.speed_info.change_time_params.standard_unload_time = float(rd(10, 40));
    if (rb(50))
        for (int e = 0; e < E; ++e)
            c.speed_info.ams_preload_enabled.push_back(rb(50));
    return g;
}

static void write_finfo(const FilamentGroupUtils::FilamentInfo &f)
{
    ti(f.color.r);
    ti(f.color.g);
    ti(f.color.b);
    ti(f.color.a);
    ts(f.type);
    ti(f.is_support);
    ti((int) f.usage_type);
    tf(f.diameter);
}

static void write_nozzle(const MultiNozzleUtils::NozzleInfo &n)
{
    ts(n.diameter);
    ti((int) n.volume_type);
    ti(n.extruder_id);
    ti(n.group_id);
    ti(n.installed);
}

static void write_ctx(const Gen &g)
{
    const FilamentGroupContext &c = g.ctx;
    tok("ctx");
    // model
    ti((long long) c.model_info.flush_matrix.size());
    for (auto &fm : c.model_info.flush_matrix) {
        ti((long long) fm.size());
        for (auto &row : fm) {
            ti((long long) row.size());
            for (float v : row)
                tf(v);
        }
    }
    ti((long long) c.model_info.layer_filaments.size());
    for (auto &lf : c.model_info.layer_filaments)
        tlist(lf);
    ti((long long) c.model_info.filament_info.size());
    for (auto &f : c.model_info.filament_info)
        write_finfo(f);
    ti((long long) c.model_info.filament_ids.size());
    for (auto &s : c.model_info.filament_ids)
        ts(s);
    ti((long long) c.model_info.selected_nozzle_diameters.size());
    for (auto &s : c.model_info.selected_nozzle_diameters)
        ts(s);
    ti((long long) c.model_info.unprintable_filaments.size());
    for (auto &s : c.model_info.unprintable_filaments)
        tset(s);
    ti((long long) c.model_info.unprintable_volumes.size());
    for (auto &kv : c.model_info.unprintable_volumes) {
        ti(kv.first);
        ti((long long) kv.second.size());
        for (auto v : kv.second)
            ti((int) v);
    }
    // group
    ti(c.group_info.total_filament_num);
    td(c.group_info.max_gap_threshold);
    ti((int) c.group_info.mode);
    ti((int) c.group_info.strategy);
    ti(c.group_info.ignore_ext_filament);
    ti(c.group_info.has_filament_switcher);
    tlist(c.group_info.filament_volume_map);
    // machine
    tlist(c.machine_info.max_group_size);
    ti((long long) c.machine_info.machine_filament_info.size());
    for (auto &v : c.machine_info.machine_filament_info) {
        ti((long long) v.size());
        for (auto &m : v) {
            write_finfo(m);
            ti(m.extruder_id);
            ti(m.is_extended);
            ts(m.filament_id);
        }
    }
    ti((long long) c.machine_info.nozzle_loaded_filaments.size());
    for (auto &m : c.machine_info.nozzle_loaded_filaments) {
        write_finfo(m);
        ti(m.nozzle_id);
        ts(m.filament_id);
        ti(m.valid);
    }
    ti((long long) c.machine_info.prefer_non_model_filament.size());
    for (bool b : c.machine_info.prefer_non_model_filament)
        ti(b);
    ti(c.machine_info.master_extruder_id);
    // speed (insertion order)
    ti((long long) g.print_time_order.size());
    for (auto &kv : g.print_time_order) {
        ti(kv.first);
        ti((long long) kv.second.size());
        for (auto &e : kv.second) {
            ti(e.first);
            td(e.second);
        }
    }
    td(c.speed_info.extruder_change_time);
    td(c.speed_info.filament_change_time);
    ti(c.speed_info.group_with_time);
    tf(c.speed_info.change_time_params.selector_load_time);
    tf(c.speed_info.change_time_params.selector_unload_time);
    tf(c.speed_info.change_time_params.standard_load_time);
    tf(c.speed_info.change_time_params.standard_unload_time);
    ti((long long) c.speed_info.ams_preload_enabled.size());
    for (bool b : c.speed_info.ams_preload_enabled)
        ti(b);
    // nozzle
    ti((long long) c.nozzle_info.nozzle_list.size());
    for (auto &n : c.nozzle_info.nozzle_list)
        write_nozzle(n);
    ti((long long) c.nozzle_info.extruder_nozzle_list.size());
    for (auto &kv : c.nozzle_info.extruder_nozzle_list) {
        ti(kv.first);
        tlist(kv.second);
    }
    ti((long long) g.status_order.size());
    for (auto &kv : g.status_order) {
        ti(kv.first);
        ti(kv.second);
    }
    tlist(c.nozzle_info.per_nozzle_capacity);
    endl_();
}

// The parts of a context a function may change, in iteration order.
static void write_ctx_state(const FilamentGroupContext &c)
{
    ti((long long) c.nozzle_info.nozzle_list.size());
    for (auto &n : c.nozzle_info.nozzle_list)
        write_nozzle(n);
    ti((long long) c.nozzle_info.nozzle_status.size());
    for (auto &kv : c.nozzle_info.nozzle_status) {
        ti(kv.first);
        ti(kv.second);
    }
    tlist(c.nozzle_info.per_nozzle_capacity);
    ti((long long) c.nozzle_info.extruder_nozzle_list.size());
    for (auto &kv : c.nozzle_info.extruder_nozzle_list) {
        ti(kv.first);
        tlist(kv.second);
    }
    ti(c.machine_info.master_extruder_id);
    ti((long long) c.machine_info.prefer_non_model_filament.size());
    for (bool b : c.machine_info.prefer_non_model_filament)
        ti(b);
    // print time iteration order
    ti((long long) c.speed_info.filament_print_time.size());
    for (auto &kv : c.speed_info.filament_print_time) {
        ti(kv.first);
        ti((long long) kv.second.size());
        for (auto &e : kv.second) {
            ti(e.first);
            td(e.second);
        }
    }
    ti((long long) c.model_info.unprintable_volumes.size());
    for (auto &kv : c.model_info.unprintable_volumes) {
        ti(kv.first);
        ti((long long) kv.second.size());
    }
}

// The protected container `c` of std::priority_queue, reached through a pointer to member.
struct HeapAccess : MemoryedGroupHeap
{
    static std::vector<MemoryedGroup> MemoryedGroupHeap::*ptr() { return &HeapAccess::c; }
};

static void write_heap(const MemoryedGroupHeap &h)
{
    // the container in its order
    const std::vector<MemoryedGroup> &c = h.*HeapAccess::ptr();
    ti((long long) c.size());
    for (auto &m : c) {
        td(m.cost);
        ti(m.prefer_level);
        tlist(m.group);
    }
}

static std::vector<int> rand_map(int F, int N)
{
    std::vector<int> m(F);
    for (int f = 0; f < F; ++f)
        m[f] = ri(0, N - 1);
    return m;
}

#define TRY_BEGIN try {
#define TRY_END                                                                                          \
    }                                                                                                    \
    catch (const FilamentGroupException &e) { exc(0, e.what()); }                                        \
    catch (const Slic3r::RuntimeError &e) { exc(1, e.what()); }                                          \
    catch (const std::out_of_range &e) { exc(2, e.what()); }

// ---------------------------------------------------------------------------------------------
// The cases
// ---------------------------------------------------------------------------------------------

static void case_free()
{
    for (int it = 0; it < 400; ++it) {
        float cd = rb(30) ? float(ri(0, 60)) + 0.005f * ri(0, 2) : float(rd(0, 80));
        std::string a = FIL_IDS[ri(0, 3)], b = FIL_IDS[ri(0, 3)];
        int n = ri(-1, 9);
        tok("cmc"); tf(cd); ts(a); ts(b); ti(n); arrow(); ti(calc_match_mode_cost(cd, a, b, n)); endl_();
    }
    for (int it = 0; it < 200; ++it) {
        std::string d = DIAMS[ri(0, 2)];
        int vt = ri(0, 5), r = ri(0, 1), lf = ri(-2, 8), cap = ri(-2, 5);
        tok("fnv"); ts(d); ti(vt); ti(r); ti(lf); ti(cap); arrow(); tu(fnv_hash_nozzle(d, vt, r, lf, cap)); endl_();
    }
    for (int it = 0; it < 200; ++it) {
        double f = rb(30) ? double(ri(0, 5000)) : rd(0, 9000), t = rd(0, 900);
        bool w = rb(70);
        tok("evs"); td(f); td(t); ti(w); arrow(); td(evaluate_score(f, t, w)); endl_();
    }
    for (int it = 0; it < 300; ++it) {
        FilamentGroupUtils::FilamentInfo f = rand_filament_info();
        int vt = ri(0, 3);
        tok("fdm"); tf(f.diameter); ti(vt); arrow(); ti(filament_diameter_matches_nozzle_volume(f, NozzleVolumeType(vt))); endl_();
    }
    // update_memoryed_groups sequences and the heap
    for (int it = 0; it < 300; ++it) {
        MemoryedGroupHeap heap;
        double budget = rb(30) ? 0. : rd(0, 40), thr = rb(50) ? 0.01 : rd(0, 0.3);
        int n = ri(1, 20);
        tok("umg"); td(budget); td(thr); ti(n);
        for (int i = 0; i < n; ++i) {
            double cost = rb(40) ? double(ri(0, 20) * 5) : rd(0, 200);
            int lvl = ri(0, 2);
            std::vector<int> grp{ri(0, 3), ri(0, 3), i};
            td(cost); ti(lvl); tlist(grp);
            update_memoryed_groups(MemoryedGroup(grp, cost, lvl), budget, thr, heap);
        }
        arrow();
        write_heap(heap);
        std::vector<std::vector<int>> arrs;
        std::vector<unsigned int> used{0, 2, 3};
        change_memoryed_heaps_to_arrays(heap, 5, used, arrs);
        ti((long long) arrs.size());
        for (auto &a : arrs)
            tlist(a);
        endl_();
    }
    for (int it = 0; it < 200; ++it) {
        std::vector<std::vector<unsigned int>> lf;
        int L = ri(0, 5);
        tok("csu"); ti(L);
        for (int l = 0; l < L; ++l) {
            std::vector<unsigned int> v;
            int k = ri(0, 4);
            for (int j = 0; j < k; ++j)
                v.push_back(unsigned(ri(0, 9)));
            tlist(v);
            lf.push_back(v);
        }
        arrow(); tlist(collect_sorted_used_filaments(lf)); endl_();
    }
}

static void case_ctx(int iters, int max_fil, int max_noz, int max_layers, bool heavy, bool single)
{
    for (int it = 0; it < iters; ++it) {
        Gen g = rand_ctx(max_fil, max_noz, max_layers, single);
        write_ctx(g);
        const FilamentGroupContext &ctx = g.ctx;
        const int F = ctx.group_info.total_filament_num;
        const int N = (int) ctx.nozzle_info.nozzle_list.size();
        auto used = collect_sorted_used_filaments(ctx.model_info.layer_filaments);
        // The operations whose result goes through reorder_filaments_for_minimum_flush_volume_base run on
        // layers of one filament only. FIDELITY-NOTE: the Rust port of ToolOrderUtils.cpp still forecasts
        // the next layer (`use_forcast` of an earlier source); the pin sets it false (ToolOrderUtils.cpp:1100).
        // With one filament per layer get_extruders_order returns before reading it.
        const bool reorder_ok = single;

        // estimate_dedup_enum_count
        {
            int k = ri(0, 5), n = ri(0, 8);
            tok("ded"); ti(k); ti(n); arrow(); ti(estimate_dedup_enum_count(k, n, ctx)); endl_();
        }
        // calc_one_change_budget
        {
            FilamentGroup fg(ctx);
            tok("ocb"); arrow(); td(fg.calc_one_change_budget(used)); endl_();
        }
        // full_evaluate / TimeEvaluator
        for (int r = 0; r < 2; ++r) {
            auto m = rand_map(F, N);
            int flush = -7;
            double s = full_evaluate(used, m, ctx, std::nullopt, &flush);
            if (reorder_ok) { tok("fe"); tlist(m); arrow(); td(s); ti(flush); endl_(); }
            TimeEvaluator te(ctx.speed_info);
            auto m2 = rand_map(F, std::max(N, 2));
            tok("te"); tlist(m2); arrow(); td(te.get_estimated_time(m2)); endl_();
        }
        // FlushDistanceEvaluator
        {
            double p = rb(70) ? 0.65 : rd(0, 1);
            FlushDistanceEvaluator fde(ctx.model_info.flush_matrix, used, ctx.model_info.layer_filaments, p);
            tok("fde"); td(p); arrow();
            for (size_t e = 0; e < ctx.model_info.flush_matrix.size(); ++e)
                for (size_t a = 0; a < used.size(); ++a)
                    for (size_t b = 0; b < used.size(); ++b)
                        td(fde.get_distance(a, b, e));
            endl_();
        }
        // available_only
        std::vector<int> full_ids;
        FilamentGroupContext actx = ctx.available_only(full_ids);
        tok("ao"); arrow(); tlist(full_ids); write_ctx_state(actx); endl_();

        // rebuild_unprintables / rebuild_nozzle_unprintables
        {
            FilamentGroup fg(ctx);
            std::map<int, int> lim;
            extract_unprintable_limit_indices(ctx.model_info.unprintable_filaments, used, lim);
            tok("eul"); arrow(); ti((long long) lim.size());
            for (auto &kv : lim) { ti(kv.first); ti(kv.second); }
            auto r = fg.rebuild_unprintables(used, lim);
            ti((long long) r.size());
            for (auto &kv : r) { ti(kv.first); ti(kv.second); }
            write_ctx_state(fg.ctx);
            endl_();
        }
        {
            FilamentGroup fg(actx);
            std::unordered_map<int, std::vector<int>> lim;
            extract_unprintable_limit_indices(actx.model_info.unprintable_filaments, used, lim);
            auto r = fg.rebuild_nozzle_unprintables(used, lim, actx.group_info.filament_volume_map);
            tok("rnu"); arrow(); ti((long long) r.size());
            for (auto &kv : r) { ti(kv.first); tlist(kv.second); }
            endl_();
        }
        // try_merge_filaments / seperate_merged_filaments / rebuild_context
        {
            FilamentGroup fg(ctx);
            auto merged = fg.try_merge_filaments();
            std::map<int, std::vector<int>> sorted(merged.begin(), merged.end());
            auto m = rand_map(F, N);
            auto sep = fg.seperate_merged_filaments(m, merged);
            fg.rebuild_context(merged);
            tok("tmf"); tlist(m); arrow(); ti((long long) sorted.size());
            for (auto &kv : sorted) { ti(kv.first); tlist(kv.second); }
            tlist(sep);
            ti((long long) fg.ctx.model_info.layer_filaments.size());
            for (auto &lf : fg.ctx.model_info.layer_filaments) tlist(lf);
            endl_();
        }
        // The enumeration of calc_min_flush_group, not its KMediods (whose 3 s limit is not reached the
        // same way by both builds): the grouping functions below run only when it enumerates.
        const bool enum_ok = reorder_ok && (full_ids.empty() ||
            estimate_dedup_enum_count((int) actx.nozzle_info.nozzle_list.size(), (int) used.size(), actx) < ENUM_THRESHOLD);
        // calc_filament_group (the enum path) and calc_min_flush_group
        if (enum_ok) {
            FilamentGroup fg(ctx);
            int cost = -7;
            tok("cfg"); arrow();
            TRY_BEGIN
                auto r = fg.calc_filament_group(&cost);
                tlist(r); ti(cost);
                ti((long long) fg.m_memoryed_groups.size());
                for (auto &a : fg.m_memoryed_groups) tlist(a);
            TRY_END
            endl_();
        }
        // KMediods through calc_group_by_kmedoids, on the available context
        if (heavy && reorder_ok && !full_ids.empty()) {
            FilamentGroup fg(actx);
            int k = (int) actx.nozzle_info.nozzle_list.size();
            std::unordered_map<int, std::vector<int>> lim;
            extract_unprintable_limit_indices(actx.model_info.unprintable_filaments, used, lim);
            auto ul = fg.rebuild_nozzle_unprintables(used, lim, actx.group_info.filament_volume_map);
            int cost = -7;
            tok("kmd"); arrow();
            TRY_BEGIN
                auto r = fg.calc_group_by_kmedoids(k, used, ul, &cost, 1000000000);
                tlist(r); ti(cost); write_heap(fg.m_memoryed_heap);
            TRY_END
            endl_();
        }
        // KMediods members
        if (!full_ids.empty() && !used.empty()) {
            int k = (int) actx.nozzle_info.nozzle_list.size();
            auto ev = std::make_shared<FlushDistanceEvaluator>(actx.model_info.flush_matrix, used, actx.model_info.layer_filaments);
            KMediods km(k, (int) used.size(), ev, actx.machine_info.master_extruder_id);
            std::unordered_map<int, std::vector<int>> lim;
            extract_unprintable_limit_indices(actx.model_info.unprintable_filaments, used, lim);
            FilamentGroup fg(actx);
            auto ul = fg.rebuild_nozzle_unprintables(used, lim, actx.group_info.filament_volume_map);
            km.set_unplacable_limits(ul);
            std::vector<std::pair<std::set<int>, int>> csl;
            for (auto &kv : actx.nozzle_info.extruder_nozzle_list)
                csl.emplace_back(std::set<int>(kv.second.begin(), kv.second.end()), kv.first < (int) actx.machine_info.max_group_size.size() ? actx.machine_info.max_group_size[kv.first] : 1);
            km.set_cluster_group_size(csl);
            std::vector<int> cs(k);
            for (int i = 0; i < k; ++i) cs[i] = ri(0, 3);
            int seed = ri(0, 40);
            auto center = km.init_cluster_center(km.m_placeable_limits, km.m_unplaceable_limits, cs, csl, seed);
            auto labels = km.assign_cluster_label(center, km.m_placeable_limits, km.m_unplaceable_limits, cs, csl);
            tok("kmm"); tlist(cs); ti(seed); arrow();
            tlist(center); tlist(labels);
            ti(km.have_enough_size(cs, csl, (int) used.size()));
            ti(km.calc_cost(labels, center, -1));
            ti(km.calc_cost(labels, center, labels.empty() ? 0 : labels[0]));
            endl_();
            if (heavy && reorder_ok) {
                km.set_memory_threshold(0.01);
                km.set_one_change_budget(fg.calc_one_change_budget(used));
                int retry = ri(1, 4);
                km.do_clustering(actx, 1000000000, retry);
                tok("kdc"); ti(retry); arrow(); tlist(km.get_cluster_labels()); write_heap(km.get_memoryed_groups());
                // m_unplaceable_limits after do_clustering (iteration order)
                ti((long long) km.m_unplaceable_limits.size());
                for (auto &kv : km.m_unplaceable_limits) { ti(kv.first); tlist(kv.second); }
                endl_();
            }
        }
        // calc_filament_group_for_match
        {
            FilamentGroup fg(ctx);
            tok("cfm"); arrow();
            TRY_BEGIN
                tlist(fg.calc_filament_group_for_match());
            TRY_END
            endl_();
        }
        // select_best_group_for_ams
        {
            std::vector<std::vector<int>> maps;
            int m = ri(1, 4);
            for (int i = 0; i < m; ++i) maps.push_back(rand_map(F, N));
            std::vector<FilamentGroupUtils::FilamentInfo> info;
            for (auto f : used) info.push_back(ctx.model_info.filament_info[f]);
            bool sw = rb(10);
            double thr = rb(70) ? 20. : rd(0, 60);
            tok("sbg"); ti((long long) maps.size()); for (auto &a : maps) tlist(a); ti(sw); td(thr); arrow();
            tlist(select_best_group_for_ams(maps, ctx.nozzle_info.nozzle_list, used, info, ctx.machine_info.machine_filament_info, sw, thr));
            endl_();
        }
        // the multi-nozzle grouping functions
        if (enum_ok) {
            tok("mmn"); arrow();
            TRY_BEGIN tlist(calc_filament_group_for_match_multi_nozzle(ctx)); TRY_END
            endl_();
        }
        if (enum_ok) {
            std::vector<int> man(F);
            for (int f = 0; f < F; ++f) man[f] = ri(0, 1);
            tok("man"); tlist(man); arrow();
            TRY_BEGIN tlist(calc_filament_group_for_manual_multi_nozzle(man, ctx)); TRY_END
            endl_();
        }
        if (enum_ok) {
            FilamentGroupContext c2 = ctx;
            tok("cnm"); arrow();
            TRY_BEGIN tlist(calc_filament_group_for_nozzle_match(c2)); write_ctx_state(c2); TRY_END
            endl_();
        }
        {
            tok("pfn"); arrow();
            TRY_BEGIN
                auto res = plan_filament_nozzle_mapping_and_order(ctx);
                ti((long long) res.size());
                for (auto &r : res) { tlist(r.fil_order); tlist(r.fil_nozzle_match); }
            TRY_END
            endl_();
        }
        if (ctx.nozzle_info.extruder_nozzle_list.size() == 2 && ctx.model_info.flush_matrix.size() == 2) {
            FilamentGroup fg(ctx);
            tok("tpu"); arrow();
            auto r = fg.calc_filament_group_for_tpu();
            tlist(r); write_ctx_state(fg.ctx);
            endl_();
        }
    }
}

int main()
{
    case_free();
    g_seed = 0x0123456789abcdefull;
    case_ctx(300, 6, 4, 6, false, false);
    g_seed = 0x5555aaaa5555aaaaull;
    case_ctx(30, 7, 5, 4, true, false);
    g_seed = 0x3141592653589793ull;
    case_ctx(400, 6, 4, 8, false, true);
    g_seed = 0x2718281828459045ull;
    case_ctx(40, 7, 5, 6, true, true);
    return 0;
}
