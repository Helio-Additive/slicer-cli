// Generator of the vectors of tests/tool_order_utils_native_parity.rs: libslic3r's
// GCode/ToolOrderUtils.cpp at the pin (references/BambuStudio/src/libslic3r, the override layer has
// no copy of it): the flow solvers (MaxFlowSolver, GeneralMinCostSolver,
// GeneralMinCostLowerBoundsSolver, GroupMinCostFlowSolver, MinFlushFlowSolver,
// MatchModeGroupSolver), the three single-nozzle order solvers, get_extruders_order,
// reorder_filaments_for_minimum_flush_volume(_base), reorder_filaments_for_multi_nozzle_extruder
// and the functions of the `#if DEBUG_MULTI_NOZZLE_MCMF` block.
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
// ToolOrderUtils.cpp is included here, so that its file-static functions are reachable; its
// definitions replace those of the library's copy at link time. The header is included first and
// DEBUG_MULTI_NOZZLE_MCMF (0 in the header) is then set to 1, so that the block the engine leaves
// out is compiled too: the port carries it in the module `debug_multi_nozzle_mcmf`. The inputs stay
// within what the functions assert (the oracle is built with NDEBUG, the port checks the same
// conditions in debug builds). The run is deterministic (a fixed LCG). Made on macOS arm64 with
// Apple clang and libc++.
//
// Line format: tokens separated by one space; a float is the 8 hexadecimal digits of its bits, an
// integer is decimal, a list is its length then its items, a matrix is its number of rows then
// every row as a list of floats, a limits map is its number of keys then per key (ascending) the
// key and its list. An optional filament is -1 when absent. A custom sequence function is "Q" <n>
// then n pairs <layer> <list>: the function returns true with that list for those layers and false
// for the others; n = -1 is "no function". Every line is
//   <op> <inputs...> => <outputs...>

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <functional>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#define private public
#include "libslic3r/GCode/ToolOrderUtils.hpp"
#undef private
#undef DEBUG_MULTI_NOZZLE_MCMF
#define DEBUG_MULTI_NOZZLE_MCMF 1
#include "libslic3r/GCode/ToolOrderUtils.cpp"

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
static void tf(float v)
{
    uint32_t b;
    std::memcpy(&b, &v, 4);
    char buf[16];
    snprintf(buf, sizeof(buf), "%08x", b);
    tok(buf);
}
template<class T> static void tlist(const std::vector<T> &v)
{
    ti((long long) v.size());
    for (auto x : v)
        ti((long long) x);
}
static void tmat(const FlushMatrix &m)
{
    ti((long long) m.size());
    for (auto &row : m) {
        ti((long long) row.size());
        for (float v : row)
            tf(v);
    }
}
static void tmats(const std::vector<FlushMatrix> &ms)
{
    ti((long long) ms.size());
    for (auto &m : ms)
        tmat(m);
}
using Limits = std::unordered_map<int, std::vector<int>>;
static void tlim(const Limits &l)
{
    std::map<int, std::vector<int>> sorted(l.begin(), l.end());
    ti((long long) sorted.size());
    for (auto &[k, v] : sorted) {
        ti(k);
        tlist(v);
    }
}
static void tlayers(const std::vector<std::vector<unsigned int>> &ls)
{
    ti((long long) ls.size());
    for (auto &l : ls)
        tlist(l);
}
static void tmatch(const std::vector<std::vector<int>> &m)
{
    ti((long long) m.size());
    for (auto &l : m)
        tlist(l);
}
static void arrow() { tok("=>"); }
static void endl_()
{
    out += '\n';
    fputs(out.c_str(), stdout);
    out.clear();
}

// ---------------------------------------------------------------------------------------------
// Deterministic random numbers
// ---------------------------------------------------------------------------------------------

static uint64_t g_seed = 0x243f6a8885a308d3ull;
static uint32_t rnd()
{
    g_seed = g_seed * 6364136223846793005ull + 1442695040888963407ull;
    return uint32_t(g_seed >> 33);
}
static int    ri(int lo, int hi) { return lo + int(rnd() % uint32_t(hi - lo + 1)); }
static double rd(double lo, double hi) { return lo + (hi - lo) * (double(rnd()) / 2147483648.0); }
static bool   rb(int pct = 50) { return ri(0, 99) < pct; }

// A flush matrix. kind 0: whole numbers; 1: fractions; 2: a handful of values, so that ties decide;
// 3: as 1 with values above MCMF_MAX_EDGE_COST here and there.
static FlushMatrix rand_matrix(int rows, int cols, int kind)
{
    FlushMatrix m(rows, std::vector<float>(cols, 0.f));
    for (int a = 0; a < rows; ++a)
        for (int b = 0; b < cols; ++b) {
            float v;
            switch (kind) {
            case 0: v = float(ri(0, 900)); break;
            case 1: v = float(rd(0., 900.)); break;
            case 2: v = float(100 * ri(0, 3)); break;
            default: v = rb(8) ? float(rd(1.1e7, 3e8)) : float(rd(0., 900.)); break;
            }
            if (a == b && rb(90))
                v = 0.f;
            m[a][b] = v;
        }
    return m;
}
static FlushMatrix rand_square(int n, bool allow_huge = false)
{
    int kind = ri(0, 9);
    kind     = kind < 4 ? 0 : kind < 7 ? 1 : kind < 9 ? 2 : (allow_huge ? 3 : 1);
    return rand_matrix(n, n, kind);
}

// k distinct values of 0..n-1 in a random order.
static std::vector<unsigned int> rand_subset(int n, int k)
{
    std::vector<unsigned int> all;
    for (int i = 0; i < n; ++i)
        all.push_back(unsigned(i));
    for (int i = n - 1; i > 0; --i)
        std::swap(all[i], all[ri(0, i)]);
    all.resize(k);
    return all;
}

static Limits rand_limits(int L, int R, int pct, bool allow_empty)
{
    Limits l;
    for (int i = 0; i < L; ++i)
        if (rb(pct)) {
            std::vector<int> v;
            int              n = ri(allow_empty ? 0 : 1, std::min(R, 3));
            for (int k = 0; k < n; ++k)
                v.push_back(ri(0, R - 1));
            l[i] = v;
        }
    return l;
}

static std::vector<std::pair<std::set<int>, int>> rand_group_capacity(int R)
{
    std::vector<std::pair<std::set<int>, int>> g;
    int                                        n = rb(40) ? ri(1, 2) : 0;
    for (int i = 0; i < n; ++i) {
        std::set<int> s;
        for (int k = 0; k < ri(1, R); ++k)
            s.insert(ri(0, R - 1));
        g.push_back({s, ri(0, 3)});
    }
    return g;
}
static void tgroup_capacity(const std::vector<std::pair<std::set<int>, int>> &g)
{
    ti((long long) g.size());
    for (auto &[s, c] : g) {
        tlist(std::vector<int>(s.begin(), s.end()));
        ti(c);
    }
}

static std::optional<unsigned int> rand_start(int nf, const std::vector<unsigned int> &curr)
{
    int k = ri(0, 2);
    if (k == 0)
        return std::nullopt;
    if (k == 1 && !curr.empty())
        return curr[ri(0, int(curr.size()) - 1)];
    return unsigned(ri(0, nf - 1));
}
static void tstart(const std::optional<unsigned int> &s) { ti(s ? (long long) *s : -1); }

// ---------------------------------------------------------------------------------------------
// Custom sequences
// ---------------------------------------------------------------------------------------------

using CustomSeq = std::map<int, std::vector<int>>;

// For some layers that print something, the layer's filaments (1-based) in another order, now and
// then with filaments the layer does not print in between (a custom sequence names every filament
// of the print). What is kept of a sequence is as long as the layer's list, which the C++ asserts
// (ToolOrderUtils.cpp:1061, 1156, 1435, 1577), and its front() is read (:1264).
static CustomSeq rand_custom(const std::vector<std::vector<unsigned int>> &layers, int nf, int pct)
{
    CustomSeq c;
    for (size_t l = 0; l < layers.size(); ++l) {
        if (layers[l].empty() || !rb(pct))
            continue;
        std::vector<int> seq;
        for (auto f : layers[l])
            seq.push_back(int(f) + 1);
        for (int i = int(seq.size()) - 1; i > 0; --i)
            std::swap(seq[i], seq[ri(0, i)]);
        if (rb(40))
            for (int f = 0; f < nf; ++f)
                if (std::find(layers[l].begin(), layers[l].end(), unsigned(f)) == layers[l].end() && rb(60))
                    seq.insert(seq.begin() + ri(0, int(seq.size())), f + 1);
        c[int(l)] = seq;
    }
    return c;
}
static void tcustom(bool has, const CustomSeq &c)
{
    tok("Q");
    if (!has) {
        ti(-1);
        return;
    }
    ti((long long) c.size());
    for (auto &[l, s] : c) {
        ti(l);
        tlist(s);
    }
}
static std::function<bool(int, std::vector<int> &)> custom_fn(const CustomSeq &c)
{
    return [c](int layer, std::vector<int> &seq) {
        auto it = c.find(layer);
        if (it == c.end())
            return false;
        seq = it->second;
        return true;
    };
}

// Layers over the filaments 0..nf-1: per layer at most max_k of them, in a random order.
static std::vector<std::vector<unsigned int>> rand_layers(int nl, int nf, int max_k, int empty_pct)
{
    std::vector<std::vector<unsigned int>> ls;
    for (int l = 0; l < nl; ++l) {
        if (rb(empty_pct)) {
            ls.push_back({});
            continue;
        }
        ls.push_back(rand_subset(nf, ri(1, std::min(nf, max_k))));
    }
    return ls;
}

// ---------------------------------------------------------------------------------------------
// The solvers
// ---------------------------------------------------------------------------------------------

static std::vector<int> rand_nodes(int n, int lo, int hi)
{
    std::vector<int> v;
    for (int i = 0; i < n; ++i)
        v.push_back(ri(lo, hi));
    return v;
}

static void case_max_flow_solver()
{
    for (int it = 0; it < 800; ++it) {
        int  L = ri(1, 6), R = ri(1, 6);
        auto u = rand_nodes(L, 0, 9), v = rand_nodes(R, 0, 9);
        auto link = rand_limits(L, R, 25, true), unlink = rand_limits(L, R, 30, true);
        auto ucap = rb(30) ? rand_nodes(L, 0, 3) : std::vector<int>();
        auto vcap = rb(30) ? rand_nodes(R, 0, 3) : std::vector<int>();
        auto gcap = rand_group_capacity(R);
        tok("max_flow_solver");
        tlist(u);
        tlist(v);
        tlim(link);
        tlim(unlink);
        tlist(ucap);
        tlist(vcap);
        tgroup_capacity(gcap);
        arrow();
        MaxFlowSolver s(u, v, link, unlink, ucap, vcap, gcap);
        tlist(s.solve());
        endl_();
    }
}

static void case_general_min_cost_solver()
{
    for (int it = 0; it < 600; ++it) {
        int  nf = ri(2, 8), L = ri(1, 5), R = ri(1, 6);
        auto m = rand_square(nf, true);
        auto u = rand_nodes(L, -1, nf - 1), v = rand_nodes(R, 0, nf - 1);
        tok("general_min_cost_solver");
        tmat(m);
        tlist(u);
        tlist(v);
        arrow();
        GeneralMinCostSolver s(m, u, v);
        tlist(s.solve());
        endl_();
    }
}

// The inputs the two solvers over groups of right nodes share.
struct GroupIn
{
    std::vector<FlushMatrix> ms;
    std::vector<int>         u, v, vg;
    Limits                   link, unlink;
};
static GroupIn rand_group_in(bool limits)
{
    GroupIn g;
    int     nf = ri(2, 8), G = ri(1, 3), L = ri(1, std::min(nf, 6)), R = ri(1, 7);
    for (int i = 0; i < G; ++i)
        g.ms.push_back(rand_square(nf));
    for (auto f : rand_subset(nf, L))
        g.u.push_back(int(f));
    for (int j = 0; j < R; ++j) {
        g.v.push_back(rb(35) ? -1 : ri(0, nf - 1));
        // Every group has a node, most of the time.
        g.vg.push_back(j < G && rb(90) ? j : ri(0, G - 1));
    }
    if (limits) {
        g.link   = rand_limits(L, R, 20, false);
        g.unlink = rand_limits(L, R, 25, true);
    }
    return g;
}
static void tgroup_in(const GroupIn &g)
{
    tmats(g.ms);
    tlist(g.u);
    tlist(g.v);
    tlist(g.vg);
    tlim(g.link);
    tlim(g.unlink);
}

// Whether the lower bounds can be met: the first two steps of solve() on a solver of its own, up to
// the `assert(need == pushed_flow)` of ToolOrderUtils.cpp:408.
static bool lower_bounds_feasible(const GroupIn &g)
{
    GeneralMinCostLowerBoundsSolver s(g.ms, g.u, g.v, g.vg, g.link, g.unlink);
    std::unordered_set<int>         no_lower_group;
    for (size_t i = 0; i < g.v.size(); i++)
        if (g.v[i] >= 0)
            no_lower_group.insert(g.vg[i]);
    s.build_feasible_graph(no_lower_group);
    int need = 0;
    for (int d : s.demand)
        if (d > 0)
            need += d;
    std::vector<int> matching;
    return need == s.m_solver_lower_bounds->solve(matching);
}

static void case_general_min_cost_lower_bounds_solver()
{
    for (int it = 0; it < 600;) {
        GroupIn g = rand_group_in(rb(40));
        if (!lower_bounds_feasible(g))
            continue;
        ++it;
        tok("general_min_cost_lower_bounds_solver");
        tgroup_in(g);
        arrow();
        GeneralMinCostLowerBoundsSolver s(g.ms, g.u, g.v, g.vg, g.link, g.unlink);
        tlist(s.solve());
        endl_();
    }
}

static void case_group_min_cost_flow_solver()
{
    for (int it = 0; it < 600; ++it) {
        GroupIn g = rand_group_in(rb(40));
        tok("group_min_cost_flow_solver");
        tgroup_in(g);
        arrow();
        GroupMinCostFlowSolver s(g.ms, g.u, g.v, g.vg, g.link, g.unlink);
        tlist(s.solve());
        endl_();
    }
}

static void case_min_flush_flow_solver()
{
    for (int it = 0; it < 800; ++it) {
        int  nf = ri(2, 8), L = ri(1, 6), R = ri(1, 6);
        auto m = rand_square(nf, true);
        auto u = rand_nodes(L, -1, nf - 1), v = rand_nodes(R, 0, nf - 1);
        auto link = rand_limits(L, R, 20, true), unlink = rand_limits(L, R, 30, true);
        auto ucap = rb(30) ? rand_nodes(L, 0, 3) : std::vector<int>();
        auto vcap = rb(30) ? rand_nodes(R, 0, 3) : std::vector<int>();
        auto gcap = rand_group_capacity(R);
        tok("min_flush_flow_solver");
        tmat(m);
        tlist(u);
        tlist(v);
        tlim(link);
        tlim(unlink);
        tlist(ucap);
        tlist(vcap);
        tgroup_capacity(gcap);
        arrow();
        MinFlushFlowSolver s(m, u, v, link, unlink, ucap, vcap, gcap);
        tlist(s.solve());
        endl_();
    }
}

static void case_match_mode_group_solver()
{
    for (int it = 0; it < 600; ++it) {
        int  nu = ri(1, 7), nv = ri(1, 7), L = ri(1, 6), R = ri(1, 6);
        int  kind = ri(0, 3);
        auto m = rand_matrix(nu, nv, kind);
        auto u = rand_nodes(L, -1, nu - 1), v = rand_nodes(R, 0, nv - 1);
        auto vcap = rand_nodes(R, 0, L);
        auto unlink = rand_limits(L, R, 30, true);
        tok("match_mode_group_solver");
        tmat(m);
        tlist(u);
        tlist(v);
        tlist(vcap);
        tlim(unlink);
        arrow();
        MatchModeGroupSolver s(m, u, v, vcap, unlink);
        tlist(s.solve());
        endl_();
    }
}

// ---------------------------------------------------------------------------------------------
// The orders of one nozzle
// ---------------------------------------------------------------------------------------------

// The cost a solver leaves alone is the value it had: -1.
static const float COST_UNSET = -1.f;

static void case_solve_extruder_order_with_greedy()
{
    for (int it = 0; it < 600; ++it) {
        int  nf = rb(12) ? ri(21, 26) : ri(1, 10);
        auto m = rand_square(nf);
        auto curr = rand_subset(nf, ri(1, nf));
        auto start = rand_start(nf, curr);
        tok("solve_extruder_order_with_greedy");
        tmat(m);
        tlist(curr);
        tstart(start);
        arrow();
        float cost = COST_UNSET;
        tlist(solve_extruder_order_with_greedy(m, curr, start, &cost));
        tf(cost);
        endl_();
    }
}

static void case_solve_extruder_order_with_forcast()
{
    for (int it = 0; it < 500; ++it) {
        int  nf = ri(1, 8);
        auto m = rand_square(nf);
        auto curr = rand_subset(nf, ri(rb(5) ? 0 : 1, std::min(nf, 5)));
        auto next = rand_subset(nf, ri(0, std::min(nf, 5)));
        auto start = rand_start(nf, curr);
        tok("solve_extruder_order_with_forcast");
        tmat(m);
        tlist(curr);
        tlist(next);
        tstart(start);
        arrow();
        float cost = COST_UNSET;
        tlist(solve_extruder_order_with_forcast(m, curr, next, start, &cost));
        tf(cost);
        endl_();
    }
}

static void case_solve_extruder_order()
{
    for (int it = 0; it < 500; ++it) {
        int  nf = ri(1, 12);
        auto m = rand_square(nf);
        auto curr = rand_subset(nf, ri(1, std::min(nf, 10)));
        auto start = rand_start(nf, curr);
        // Nothing to print and a filament loaded: the path is empty.
        if (start && rb(4))
            curr.clear();
        tok("solve_extruder_order");
        tmat(m);
        tlist(curr);
        tstart(start);
        arrow();
        float cost = COST_UNSET;
        tlist(solve_extruder_order(m, curr, start, &cost));
        tf(cost);
        endl_();
    }
}

static void case_get_extruders_order()
{
    for (int it = 0; it < 700; ++it) {
        int  nf = rb(8) ? ri(21, 26) : ri(1, 11);
        auto m = rand_square(nf);
        bool forcast = nf <= 11 && rb(45);
        int  k;
        if (nf > 20)
            k = rb(70) ? ri(21, nf) : ri(0, 2);
        else
            k = ri(rb(10) ? 0 : 1, std::min(nf, forcast ? 5 : 11));
        auto curr = rand_subset(nf, k);
        auto next = rand_subset(nf, ri(0, std::min(nf, 5)));
        auto start = rand_start(nf, curr);
        bool with_cost = rb(85);
        tok("get_extruders_order");
        tmat(m);
        tlist(curr);
        tlist(next);
        tstart(start);
        ti(forcast);
        ti(with_cost);
        arrow();
        float cost = COST_UNSET;
        tlist(get_extruders_order(m, curr, next, start, forcast, with_cost ? &cost : nullptr));
        tf(cost);
        endl_();
    }
}

// ---------------------------------------------------------------------------------------------
// The orders of a print
// ---------------------------------------------------------------------------------------------

static void case_reorder_filaments_for_minimum_flush_volume_base()
{
    for (int it = 0; it < 500; ++it) {
        bool big = rb(6);
        int  nf = big ? ri(22, 25) : ri(1, 10);
        auto m = rand_square(nf);
        auto list = rand_subset(nf, big ? nf : ri(rb(5) ? 0 : 1, nf));
        std::vector<std::vector<unsigned int>> layers;
        if (big) {
            for (int l = 0; l < ri(1, 4); ++l)
                layers.push_back(rand_subset(nf, rb(60) ? ri(21, nf) : ri(0, 6)));
        } else
            layers = rand_layers(ri(0, 10), nf, 10, 15);
        bool      has_fn = rb(45);
        CustomSeq custom = has_fn ? rand_custom(layers, nf, 35) : CustomSeq();
        std::optional<unsigned int> initial;
        if (rb(50))
            initial = unsigned(rb(85) ? ri(0, nf - 1) : nf + ri(0, 3));
        bool with_seq = rb(85);
        tok("reorder_filaments_for_minimum_flush_volume_base");
        tlist(list);
        tlayers(layers);
        tmat(m);
        tcustom(has_fn, custom);
        tstart(initial);
        ti(with_seq);
        arrow();
        std::vector<std::vector<unsigned int>> seqs;
        std::function<bool(int, std::vector<int> &)> fn;
        if (has_fn)
            fn = custom_fn(custom);
        int cost = reorder_filaments_for_minimum_flush_volume_base(list, layers, m, fn, with_seq ? &seqs : nullptr, initial);
        ti(cost);
        tlayers(seqs);
        endl_();
    }
}

static void case_reorder_filaments_for_minimum_flush_volume()
{
    for (int it = 0; it < 700; ++it) {
        bool big = rb(4);
        int  nf = big ? ri(22, 25) : ri(1, 10);
        std::vector<FlushMatrix> ms{rand_square(nf), rand_square(nf)};
        auto list = rand_subset(nf, big ? nf : ri(rb(5) ? 0 : 1, nf));
        std::vector<int> maps;
        int              one_group = ri(0, 9); // 0: all on the first, 1: all on the second
        for (size_t i = 0; i < list.size(); ++i)
            maps.push_back(big ? 0 : one_group < 2 ? one_group : rb(6) ? 2 : ri(0, 1));
        std::vector<std::vector<unsigned int>> layers;
        if (big) {
            for (int l = 0; l < ri(1, 3); ++l)
                layers.push_back(rand_subset(nf, rb(60) ? ri(21, nf) : ri(0, 5)));
        } else
            layers = rand_layers(ri(0, 12), nf, 10, 15);
        bool      has_fn = rb(45);
        CustomSeq custom = has_fn ? rand_custom(layers, nf, 35) : CustomSeq();
        std::unordered_map<int, int> status;
        std::vector<std::pair<int, int>> status_list;
        if (rb(45))
            for (int g = 0; g < 2; ++g)
                if (rb(70)) {
                    int f = rb(80) ? ri(0, nf - 1) : rb(50) ? -1 : nf + ri(0, 2);
                    status[g] = f;
                    status_list.push_back({g, f});
                }
        bool with_seq = rb(85);
        tok("reorder_filaments_for_minimum_flush_volume");
        tlist(list);
        tlist(maps);
        tlayers(layers);
        tmats(ms);
        tcustom(has_fn, custom);
        ti((long long) status_list.size());
        for (auto &[g, f] : status_list) {
            ti(g);
            ti(f);
        }
        ti(with_seq);
        arrow();
        std::vector<std::vector<unsigned int>> seqs;
        std::optional<std::function<bool(int, std::vector<int> &)>> fn;
        if (has_fn)
            fn = custom_fn(custom);
        int cost = reorder_filaments_for_minimum_flush_volume(list, maps, layers, ms, fn, with_seq ? &seqs : nullptr, status);
        ti(cost);
        tlayers(seqs);
        endl_();
    }
}

static void case_reorder_filaments_for_multi_nozzle_extruder()
{
    for (int it = 0; it < 700; ++it) {
        int n_ext = ri(1, 3), n_noz = n_ext + ri(0, 3), nf = ri(1, 9);
        std::vector<MultiNozzleUtils::NozzleInfo> nozzles;
        for (int i = 0; i < n_noz; ++i) {
            MultiNozzleUtils::NozzleInfo ni;
            ni.diameter    = "0.4";
            ni.volume_type = NozzleVolumeType(ri(0, 1));
            // Extruder ids need not be contiguous.
            ni.extruder_id = i < n_ext ? i : ri(0, n_ext - 1);
            if (rb(6))
                ni.extruder_id += ri(1, 2);
            ni.group_id = rb(92) ? i : ri(0, n_noz + 1);
            nozzles.push_back(ni);
        }
        // Some filaments have no nozzle: the map is short, or names a nozzle that does not exist.
        std::vector<int> map;
        int              map_len = rb(88) ? nf : ri(1, nf);
        for (int f = 0; f < map_len; ++f)
            map.push_back(rb(94) ? ri(0, n_noz - 1) : rb(50) ? -1 : n_noz + ri(0, 1));
        std::vector<unsigned int> used;
        for (int f = 0; f < nf; ++f)
            used.push_back(unsigned(f));
        auto list = rb(80) ? used : rand_subset(nf, ri(0, nf));
        // One matrix per extruder most of the time; fewer leaves the last extruders without one.
        int n_mat = rb(85) ? n_ext + 2 : ri(1, n_ext);
        std::vector<FlushMatrix> ms;
        for (int e = 0; e < n_mat; ++e)
            ms.push_back(rand_square(nf));
        auto      layers = rand_layers(ri(0, 12), nf, 9, 15);
        bool      has_fn = rb(45);
        CustomSeq custom = has_fn ? rand_custom(layers, nf, 35) : CustomSeq();
        // The recorder: calls of set_nozzle_status, then the current extruder (-2: not set).
        MultiNozzleUtils::NozzleStatusRecorder recorder;
        std::vector<std::array<int, 3>>         rec_calls;
        int                                     rec_ext = -2;
        if (rb(55)) {
            for (int k = 0; k < ri(0, 4); ++k) {
                std::array<int, 3> c{ri(0, n_noz), rb(85) ? ri(0, nf - 1) : rb(50) ? -1 : nf + ri(0, 2), ri(-1, n_ext + 1)};
                rec_calls.push_back(c);
                recorder.set_nozzle_status(c[0], c[1], c[2]);
            }
            if (rb(70)) {
                rec_ext = ri(-1, n_ext + 2);
                recorder.set_current_extruder_id(rec_ext);
            }
        }
        bool with_seq = rb(88);
        bool as_extruder = rb(20);
        tok("reorder_filaments_for_multi_nozzle_extruder");
        tlist(list);
        tlist(map);
        ti((long long) nozzles.size());
        for (auto &n : nozzles) {
            ti((int) n.volume_type);
            ti(n.extruder_id);
            ti(n.group_id);
        }
        tlist(used);
        ti(as_extruder);
        tlayers(layers);
        tmats(ms);
        tcustom(has_fn, custom);
        ti((long long) rec_calls.size());
        for (auto &c : rec_calls) {
            ti(c[0]);
            ti(c[1]);
            ti(c[2]);
        }
        ti(rec_ext);
        ti(with_seq);
        arrow();
        auto group = MultiNozzleUtils::LayeredNozzleGroupResult::create(map, nozzles, used, as_extruder);
        std::vector<std::vector<unsigned int>> seqs;
        std::function<bool(int, std::vector<int> &)> fn;
        if (has_fn)
            fn = custom_fn(custom);
        int cost = reorder_filaments_for_multi_nozzle_extruder(list, *group, layers, ms, fn, with_seq ? &seqs : nullptr, recorder);
        ti(cost);
        tlayers(seqs);
        endl_();
    }
}

// ---------------------------------------------------------------------------------------------
// The DEBUG_MULTI_NOZZLE_MCMF block
// ---------------------------------------------------------------------------------------------

static void case_solve_extruder_order_with_mcmf()
{
    for (int it = 0; it < 500; ++it) {
        int  nf = ri(1, 9);
        auto m = rand_square(nf, true);
        std::vector<int> curr;
        for (auto f : rand_subset(nf, ri(0, nf)))
            curr.push_back(int(f));
        auto state = rand_nodes(ri(1, 4), -1, nf - 1);
        bool by_name = rb(30);
        tok("solve_extruder_order_with_mcmf");
        tmat(m);
        tlist(curr);
        tlist(state);
        ti(by_name);
        arrow();
        float cost = COST_UNSET;
        // The four-argument get_extruders_order is the same function under its public name.
        auto ret = by_name ? get_extruders_order(m, curr, state, &cost) : solve_extruder_order_with_MCMF(m, curr, state, &cost);
        tmatch(ret);
        tf(cost);
        endl_();
    }
}

static void case_get_nozzle_match_in_given_order()
{
    for (int it = 0; it < 500; ++it) {
        int  nf = ri(1, 9);
        auto m = rand_square(nf);
        std::vector<unsigned int> seq;
        if (rb(80))
            seq = rand_subset(nf, ri(0, nf));
        else
            for (int k = 0; k < ri(0, 8); ++k)
                seq.push_back(unsigned(ri(0, nf - 1)));
        auto state = rand_nodes(ri(1, 4), -1, nf - 1);
        tok("get_nozzle_match_in_given_order");
        tmat(m);
        tlist(seq);
        tlist(state);
        arrow();
        float cost = COST_UNSET;
        tmatch(get_nozzle_match_in_given_order(m, seq, state, &cost));
        tf(cost);
        endl_();
    }
}

static void case_debug_reorder_filaments_for_multi_nozzle_extruder()
{
    for (int it = 0; it < 500; ++it) {
        int nf = ri(2, 9);
        std::vector<FlushMatrix> ms{rand_square(nf), rand_square(nf)};
        bool with_seq = rb(80), with_match = rb(70);
        auto list = rand_subset(nf, ri(2, nf));
        std::vector<int> maps;
        for (size_t i = 0; i < list.size(); ++i)
            maps.push_back(ri(0, 1));
        // The sequences of both extruders are read for every layer without a custom sequence: with
        // sequences asked for, both extruders print something.
        if (with_seq) {
            maps[0] = 0;
            maps[1] = 1;
        }
        auto      layers = rand_layers(ri(0, 10), nf, 9, 15);
        bool      has_fn = rb(40);
        CustomSeq custom = has_fn ? rand_custom(layers, nf, 35) : CustomSeq();
        int       multi_id = ri(0, 1), multi_num = ri(1, 4);
        tok("debug_reorder_filaments_for_multi_nozzle_extruder");
        tlist(list);
        tlist(maps);
        tlayers(layers);
        tmats(ms);
        tcustom(has_fn, custom);
        ti(multi_id);
        ti(multi_num);
        ti(with_seq);
        ti(with_match);
        arrow();
        std::vector<std::vector<unsigned int>>     seqs;
        std::vector<std::vector<std::vector<int>>> match;
        std::optional<std::function<bool(int, std::vector<int> &)>> fn;
        if (has_fn)
            fn = custom_fn(custom);
        int cost = reorder_filaments_for_multi_nozzle_extruder(list, maps, layers, ms, fn, multi_id, multi_num, with_seq ? &seqs : nullptr,
                                                               with_match ? &match : nullptr);
        ti(cost);
        tlayers(seqs);
        ti((long long) match.size());
        for (auto &mm : match)
            tmatch(mm);
        endl_();
    }
}

int main()
{
    case_max_flow_solver();
    case_general_min_cost_solver();
    case_general_min_cost_lower_bounds_solver();
    case_group_min_cost_flow_solver();
    case_min_flush_flow_solver();
    case_match_mode_group_solver();
    case_solve_extruder_order_with_greedy();
    case_solve_extruder_order_with_forcast();
    case_solve_extruder_order();
    case_get_extruders_order();
    case_reorder_filaments_for_minimum_flush_volume_base();
    case_reorder_filaments_for_minimum_flush_volume();
    case_reorder_filaments_for_multi_nozzle_extruder();
    case_solve_extruder_order_with_mcmf();
    case_get_nozzle_match_in_given_order();
    case_debug_reorder_filaments_for_multi_nozzle_extruder();
    return 0;
}
