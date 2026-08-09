#!/usr/bin/env python3
"""Inject the Arachne parity probes into the BambuStudio C++ submodule.

The submodule is a pristine checkout of the reference engine, so these counters
cannot live there permanently. This script re-applies them on demand; revert with
    cd libslic3r/bambustudio/references/BambuStudio && git checkout -- src/libslic3r/Arachne
and rebuild.

Each probe mirrors a Rust probe of the same name in
crates/libslic3r-rs/src/arachne/, so the two engines report comparable numbers.
All are env-gated and off by default:

    BEADPROBE     BeadingStrategy::compute call count + width/thickness spread   (R543/R547)
    PROPPROBE     propagateBeadingsDownward copy-vs-interpolate split            (R546/R547)
    STAGEPROBE    per-ExtrusionLine flat% through WallToolPaths post-processing  (R544/R547)
    GRAPHPROBE    skeleton size + share of nodes carrying a bead count           (R547)
    CENTRALPROBE  central-edge and bead_count census after each marking stage    (R548)
    ISCPROBE      which branch of updateIsCentral decides each edge, + constants  (R548)
    WTPPARAMS     the six resolved WallToolPathsParams, deduped                   (R550)
    TRANSPROBE    transition mids/ends surviving each stage of the ribs pipeline   (R551)
    POLYPROBE     polygon/point counts through the prepared_outline chain          (R552)
    LASTPROBE     contour/hole counts of `last` / `last_p` in PerimeterGenerator     (R553)
    MPPROBE       surfaces/holes/points entering LayerRegion::make_perimeters        (R555)

Written because `git diff > file` in this environment does not produce an
applicable patch (R548) — string injection is verifiable and survives the tool.

Usage:  python3 scripts/inject-arachne-probes.py [--check]
"""
import sys
import os

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ARACHNE = os.path.join(
    ROOT, "libslic3r/bambustudio/references/BambuStudio/src/libslic3r/Arachne"
)
LIBSLIC3R = os.path.join(
    ROOT, "libslic3r/bambustudio/references/BambuStudio/src/libslic3r"
)
LIBSLIC3R_FILES = ("PerimeterGenerator.cpp", "LayerRegion.cpp", "VariableWidth.cpp",
                   "BridgeDetector.cpp",  # R595: bridge angle candidates
                   "Fill/FillBase.cpp")   # R597: the fill-angle CONSUMER

ST_INCLUDES_OLD = """#include <stack>
#include <functional>
#include <sstream>
#include <queue>
#include <functional>
#include <boost/log/trivial.hpp>"""

ST_INCLUDES_NEW = """#include <stack>
#include <functional>
#include <sstream>
#include <queue>
#include <functional>
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <mutex>
#include <vector>
#include <atomic>
#include <boost/log/trivial.hpp>

// R581: C++ speculatively builds a one-wall WallToolPaths on ~16,503 surfaces and
// keeps 4 (R580, TOWPROBE seperate_POST=4). That call runs the WHOLE Arachne
// pipeline, so every internal probe counted work that never reaches the G-code.
// This flag lets the probes exclude it. Thread-local: the speculative call and
// the pipeline it drives run on the same thread.
bool& probe_speculative() { static thread_local bool v = false; return v; }

// R584: every outer-wall width change is the SAME bead index resolving to a
// different beading on the next edge (LINEPROBE2 MECH: ch_idx=0 on both engines).
// So changes/junction ~ P(adjacent beadings differ) / (junctions per edge).
// This counts the denominator; the numerator is the idx-0 width spread below.
std::atomic<size_t> g_ep_edges{0};

// PROPCLASS (R586): how a node's beading comes to exist. R585 proved a node's
// beading is NOT a pure function of its thickness -- it is propagated -- and that
// C++ injects ~2x the width variation per unit of thickness variation. There are
// exactly four sites:
//   0 fresh      getOrCreateBeading -> beading_strategy.compute()
//   1 copy_new   propagateBeadingsDownward, `from` had no beading: straight copy
//   2 copy_ratio propagateBeadingsDownward, ratio_of_top >= 1.0: straight copy
//   3 interp     propagateBeadingsDownward, else: interpolate()
// A COPY is bit-identical to its source and cannot produce a width change between
// neighbours; only fresh and interp can. Counting the mix is the whole question.
std::atomic<size_t> g_pc[4] = {};
std::atomic<size_t> g_pc_total{0};

// UPPROBE (R587): why does C++ reach the has-beading branch 2.25x more often
// (R586: 14.25% vs 6.33%)? Two candidates, both counted here as per-item rates.
//   upward: propagateBeadingsUpward SEEDS nodes; three guards can skip a seed.
//   downward: the dispatcher skips central edges and has an equidistant `twin`
//             branch, so the set of edges actually walked can differ.
// Classify by FIRST failing guard, in source order.
std::atomic<size_t> g_up_total{0}, g_up_skip_bc{0}, g_up_skip_nofrom{0},
                    g_up_skip_tohas{0}, g_up_seed{0};
std::atomic<size_t> g_dn_total{0}, g_dn_central{0}, g_dn_twin{0}, g_dn_normal{0};

// CENSUS (R588): R587 showed the upward pass seeds half as many nodes, the whole
// deficit sitting in the `to->bead_count >= 0` guard, and that we mark more edges
// central (60.10% vs 56.67%, measured per dispatcher iteration). This measures both
// quantities DIRECTLY on the graph, as per-node and per-edge rates over the whole
// graph, once per generate() call -- order-independent and on the population the
// guard actually tests. Summed across calls.
std::atomic<size_t> g_cs_calls{0}, g_cs_nodes{0}, g_cs_bc{0}, g_cs_hasb{0},
                    g_cs_edges{0}, g_cs_central{0};

// GBUILD (R589): R588 found C++'s skeletal graph is ~25% denser (1.254x nodes,
// 1.256x edges per generate() call) while every per-item rate inside it matches.
// This brackets WHERE the density appears: the Voronoi INPUT (one segment per
// polygon point), the raw Voronoi OUTPUT before any filtering, and the number of
// points `discretize` emits per Voronoi edge. If the input is already 1.25x the
// cause is upstream outline discretisation; if input matches and Voronoi output
// matches, it is the discretise/filter chain.
std::atomic<size_t> g_gb_calls{0}, g_gb_polys{0}, g_gb_segs{0},
                    g_gb_vd_verts{0}, g_gb_vd_edges{0}, g_gb_vd_cells{0};
std::atomic<size_t> g_gb_disc_calls{0}, g_gb_disc_pts{0};

// CONV (R590): R589 localised the 1.25x graph density to the Voronoi->half-edge
// CONVERSION (graph edges per Voronoi edge: rust 0.9333 vs cpp 1.0878). Two ways
// that can happen: we CREATE fewer half-edges, or we REMOVE more afterwards. The
// creation path is transferEdge (from discretized points) plus makeRib (2 EXTRA_VD
// edges per call); the removal path is collapseSmallEdges. Three edge counts --
// after the cell loop, after separatePointyQuadEndNodes, after collapseSmallEdges --
// decide it. Also counts cells seen vs skipped (`!cell.incident_edge()`).
std::atomic<size_t> g_cv_calls{0}, g_cv_cells{0}, g_cv_cells_skipped{0};
std::atomic<size_t> g_cv_e0{0}, g_cv_e1{0}, g_cv_e2{0};
std::atomic<size_t> g_cv_n0{0}, g_cv_n1{0}, g_cv_n2{0};
void conv_cell(bool skipped)
{
    g_cv_cells.fetch_add(1);
    if (skipped) g_cv_cells_skipped.fetch_add(1);
}
void conv_stage(int stage, size_t edges, size_t nodes)
{
    if (stage == 0) { g_cv_e0.fetch_add(edges); g_cv_n0.fetch_add(nodes); }
    else if (stage == 1) { g_cv_e1.fetch_add(edges); g_cv_n1.fetch_add(nodes); }
    else {
        g_cv_e2.fetch_add(edges); g_cv_n2.fetch_add(nodes);
        const size_t n = g_cv_calls.fetch_add(1) + 1;
        if (n == 1 || n % 2000 == 0) {
            const double d = double(n);
            fprintf(stderr,
                "[CONV] calls=%zu cells/call=%.3f skipped/call=%.3f | e_after_cells/call=%.3f "
                "e_after_separate/call=%.3f e_after_collapse/call=%.3f | n_after_cells/call=%.3f "
                "n_after_collapse/call=%.3f | collapse_keep=%.4f\\n",
                n, double(g_cv_cells.load()) / d, double(g_cv_cells_skipped.load()) / d,
                double(g_cv_e0.load()) / d, double(g_cv_e1.load()) / d, double(g_cv_e2.load()) / d,
                double(g_cv_n0.load()) / d, double(g_cv_n2.load()) / d,
                double(g_cv_e2.load()) / double(std::max<size_t>(g_cv_e1.load(), 1)));
        }
    }
}
void gbuild_disc(size_t n) { g_gb_disc_calls.fetch_add(1); g_gb_disc_pts.fetch_add(n); }
void gbuild_tick(size_t polys, size_t segs, size_t vv, size_t ve, size_t vc)
{
    g_gb_polys.fetch_add(polys); g_gb_segs.fetch_add(segs);
    g_gb_vd_verts.fetch_add(vv); g_gb_vd_edges.fetch_add(ve); g_gb_vd_cells.fetch_add(vc);
    const size_t n = g_gb_calls.fetch_add(1) + 1;
    if (n == 1 || n % 2000 == 0) {
        const double d = double(n);
        fprintf(stderr,
            "[GBUILD] calls=%zu polys/call=%.3f segs/call=%.3f | vd_verts/call=%.3f "
            "vd_edges/call=%.3f vd_cells/call=%.3f | disc_calls=%zu disc_pts/call=%.3f "
            "pts_per_disc=%.4f\\n",
            n, double(g_gb_polys.load()) / d, double(g_gb_segs.load()) / d,
            double(g_gb_vd_verts.load()) / d, double(g_gb_vd_edges.load()) / d,
            double(g_gb_vd_cells.load()) / d, g_gb_disc_calls.load(),
            double(g_gb_disc_pts.load()) / d,
            double(g_gb_disc_pts.load()) / double(std::max<size_t>(g_gb_disc_calls.load(), 1)));
    }
}
void census_tick(size_t nodes, size_t bc, size_t hasb, size_t edges, size_t central)
{
    g_cs_nodes.fetch_add(nodes); g_cs_bc.fetch_add(bc); g_cs_hasb.fetch_add(hasb);
    g_cs_edges.fetch_add(edges); g_cs_central.fetch_add(central);
    const size_t n = g_cs_calls.fetch_add(1) + 1;
    if (n == 1 || n % 2000 == 0) {
        const double nn = double(std::max<size_t>(g_cs_nodes.load(), 1));
        const double ne = double(std::max<size_t>(g_cs_edges.load(), 1));
        fprintf(stderr,
            "[CENSUS] calls=%zu nodes=%zu bead_count>=0=%zu (%.4f) hasBeading=%zu (%.4f) "
            "| edges=%zu central=%zu (%.4f)\\n",
            n, g_cs_nodes.load(), g_cs_bc.load(), double(g_cs_bc.load()) / nn,
            g_cs_hasb.load(), double(g_cs_hasb.load()) / nn,
            g_cs_edges.load(), g_cs_central.load(), double(g_cs_central.load()) / ne);
    }
}
void upprobe_tick(bool s1, bool s2, bool s3)
{
    const size_t n = g_up_total.fetch_add(1) + 1;
    if      (s1) g_up_skip_bc.fetch_add(1);
    else if (s2) g_up_skip_nofrom.fetch_add(1);
    else if (s3) g_up_skip_tohas.fetch_add(1);
    else         g_up_seed.fetch_add(1);
    if (n == 1 || n % 100000 == 0)
        fprintf(stderr,
            "[UPPROBE] up_total=%zu skip_beadcount=%zu (%.4f) skip_no_from=%zu (%.4f) "
            "skip_to_has=%zu (%.4f) SEEDED=%zu (%.4f)\\n",
            n, g_up_skip_bc.load(), double(g_up_skip_bc.load()) / double(n),
            g_up_skip_nofrom.load(), double(g_up_skip_nofrom.load()) / double(n),
            g_up_skip_tohas.load(), double(g_up_skip_tohas.load()) / double(n),
            g_up_seed.load(), double(g_up_seed.load()) / double(n));
}
void dnprobe_tick(bool central, bool equi)
{
    const size_t n = g_dn_total.fetch_add(1) + 1;
    if      (central) g_dn_central.fetch_add(1);
    else if (equi)    g_dn_twin.fetch_add(1);
    else              g_dn_normal.fetch_add(1);
    if (n == 1 || n % 100000 == 0)
        fprintf(stderr,
            "[DNPROBE] dn_total=%zu central_skip=%zu (%.4f) twin=%zu (%.4f) normal=%zu (%.4f)\\n",
            n, g_dn_central.load(), double(g_dn_central.load()) / double(n),
            g_dn_twin.load(), double(g_dn_twin.load()) / double(n),
            g_dn_normal.load(), double(g_dn_normal.load()) / double(n));
}
std::atomic<size_t> g_pc_interp_zero{0};   // interpolate() that changed nothing
std::atomic<size_t> g_pc_interp_small{0};  // |delta w0| < 1um
std::atomic<size_t> g_pc_interp_big{0};    // |delta w0| >= 1um
void propclass_tick(int cls)
{
    if (::getenv("PROPCLASS") == nullptr || probe_speculative())
        return;
    // Increment the class counter, then take the checkpoint off a SINGLE atomic.
    // Summing four separate loads is racy under TBB and skips the exact multiple,
    // which is why the first attempt printed nothing at all.
    g_pc[cls].fetch_add(1);
    const size_t n = g_pc_total.fetch_add(1) + 1;
    if (n == 1 || n % 100000 == 0) {
        const size_t f = g_pc[0].load(), c1 = g_pc[1].load(),
                     c2 = g_pc[2].load(), ip = g_pc[3].load();
        const double tot = double(f + c1 + c2 + ip);
        fprintf(stderr,
            "[PROPCLASS] total=%.0f fresh=%zu (%.4f) copy_new=%zu (%.4f) copy_ratio=%zu (%.4f) "
            "interp=%zu (%.4f) | copies=%.4f | interp_zero=%zu interp_small=%zu interp_big=%zu\\n",
            tot, f, f / tot, c1, c1 / tot, c2, c2 / tot, ip, ip / tot,
            double(c1 + c2) / tot,
            g_pc_interp_zero.load(), g_pc_interp_small.load(), g_pc_interp_big.load());
    }
}"""

ST_PROBES_OLD = """namespace Slic3r::Arachne
{

#ifdef ARACHNE_DEBUG"""

ST_PROBES_NEW = r"""namespace Slic3r::Arachne
{

// ---------------------------------------------------------------------------
// Parity instrumentation (temporary; env-gated, off by default). Mirrors the
// Rust probes of the same name in
// crates/libslic3r-rs/src/arachne/skeletal_trapezoidation.rs so the two engines
// report comparable numbers. Coordinates are printed in mm using
// SCALING_FACTOR = 1e-5 (libslic3r.h:58), the same scale the Rust crate uses.
// ---------------------------------------------------------------------------
static bool probe_enabled(const char *name)
{
    return ::getenv(name) != nullptr;
}

static void beadprobe(coord_t thickness, coord_t bead_count, const std::vector<coord_t> &widths)
{
    static std::mutex        mtx;
    static std::vector<coord_t> thicknesses;
    static std::vector<coord_t> w0s;
    static size_t            calls = 0;
    static size_t            flat_w = 0;

    std::lock_guard<std::mutex> lock(mtx);
    const size_t n = ++calls;
    if (widths.size() > 1) {
        const auto mn = *std::min_element(widths.begin(), widths.end());
        const auto mx = *std::max_element(widths.begin(), widths.end());
        if (mn == mx)
            ++flat_w;
    }
    thicknesses.emplace_back(thickness);
    if (!widths.empty())
        w0s.emplace_back(widths.front());

    if (n == 1 || n == 2000 || n % 20000 == 0) {
        std::vector<coord_t> td = thicknesses;
        std::sort(td.begin(), td.end());
        td.erase(std::unique(td.begin(), td.end()), td.end());
        std::vector<coord_t> wd = w0s;
        std::sort(wd.begin(), wd.end());
        wd.erase(std::unique(wd.begin(), wd.end()), wd.end());
        const double tmin = thicknesses.empty() ? 0. : double(*std::min_element(thicknesses.begin(), thicknesses.end())) / 1e5;
        const double tmax = thicknesses.empty() ? 0. : double(*std::max_element(thicknesses.begin(), thicknesses.end())) / 1e5;
        const double wmin = w0s.empty() ? 0. : double(*std::min_element(w0s.begin(), w0s.end())) / 1e5;
        const double wmax = w0s.empty() ? 0. : double(*std::max_element(w0s.begin(), w0s.end())) / 1e5;
        fprintf(stderr,
                "[CPP-BEADPROBE] compute calls=%zu | thickness distinct=%zu range=%.3f..%.3fmm | "
                "bead_widths[0] distinct=%zu range=%.3f..%.3fmm | multi-bead beadings with all-equal widths=%zu\n",
                n, td.size(), tmin, tmax, wd.size(), wmin, wmax, flat_w);
    }
    (void) bead_count;
}

static void propprobe(double ratio_of_top, coord_t transition_dist, coord_t total_dist)
{
    static std::mutex mtx;
    static size_t     calls = 0, copies = 0, clamped = 0, td_lt = 0;

    std::lock_guard<std::mutex> lock(mtx);
    const size_t n = ++calls;
    if (ratio_of_top >= 1.0)
        ++copies;
    if (total_dist < transition_dist)
        ++td_lt;
    if (ratio_of_top == 0.0)
        ++clamped;

    if (n == 1 || n % 5000 == 0)
        fprintf(stderr,
                "[CPP-PROPPROBE] calls=%zu | ratio>=1.0 (pure COPY)=%zu (%.1f%%) | ratio==0=%zu | "
                "total_dist<transition_dist=%zu | transition_dist=%.3fmm\n",
                n, copies, 100. * double(copies) / double(n), clamped, td_lt, double(transition_dist) / 1e5);
}

#ifdef ARACHNE_DEBUG"""

ST_COMPUTE1_OLD = """                node_beadings.emplace_back(new BeadingPropagation(beading_strategy.compute(node.data.distance_to_boundary * 2, node.data.bead_count)));
                node.data.setBeading(node_beadings.back());
                assert(node_beadings.back()->beading.total_thickness == node.data.distance_to_boundary * 2);"""

ST_COMPUTE1_NEW = """                node_beadings.emplace_back(new BeadingPropagation(beading_strategy.compute(node.data.distance_to_boundary * 2, node.data.bead_count)));
                if (probe_enabled("BEADPROBE"))
                    beadprobe(node.data.distance_to_boundary * 2, node.data.bead_count, node_beadings.back()->beading.bead_widths);
                node.data.setBeading(node_beadings.back());
                assert(node_beadings.back()->beading.total_thickness == node.data.distance_to_boundary * 2);"""

ST_COMPUTE2_OLD = """        node_beadings.emplace_back(new BeadingPropagation(beading_strategy.compute(node->data.distance_to_boundary * 2, node->data.bead_count)));
        node->data.setBeading(node_beadings.back());"""

ST_COMPUTE2_NEW = """        node_beadings.emplace_back(new BeadingPropagation(beading_strategy.compute(node->data.distance_to_boundary * 2, node->data.bead_count)));
        if (probe_enabled("BEADPROBE"))
            beadprobe(node->data.distance_to_boundary * 2, node->data.bead_count, node_beadings.back()->beading.bead_widths);
        node->data.setBeading(node_beadings.back());"""

ST_PROP_OLD = """        ratio_of_top = std::max(0.0, ratio_of_top);
        if (ratio_of_top >= 1.0)"""

ST_PROP_NEW = """        ratio_of_top = std::max(0.0, ratio_of_top);
        if (probe_enabled("PROPPROBE"))
            propprobe(ratio_of_top, beading_propagation_transition_dist, total_dist);
        if (ratio_of_top >= 1.0)"""

ST_GRAPH_OLD = """void SkeletalTrapezoidation::generateSegments()
{
    std::vector<edge_t*> upward_quad_mids;"""

ST_GRAPH_NEW = r"""void SkeletalTrapezoidation::generateSegments()
{
    // R692: exclude the R581 speculative one-wall pass, exactly as UPPROBE /
    // PROPCLASS / GBUILD do. GRAPHPROBE predates that flag (R547 vs R581), so it
    // was the last probe still counting work that never reaches the G-code —
    // which made its per-call node/edge counts incomparable to the Rust side.
    if (probe_enabled("GRAPHPROBE") && !probe_speculative()) {
        static std::mutex mtx;
        static size_t calls = 0, nodes = 0, edges = 0, upward = 0, beaded = 0;
        size_t n_up = 0, n_bead = 0;
        for (const edge_t &e : graph.edges)
            if (e.prev && e.next && const_cast<edge_t &>(e).isUpward())
                ++n_up;
        for (const node_t &nd : graph.nodes)
            if (nd.data.bead_count > 0)
                ++n_bead;
        std::lock_guard<std::mutex> lock(mtx);
        const size_t c = ++calls;
        nodes += graph.nodes.size();
        edges += graph.edges.size();
        upward += n_up;
        beaded += n_bead;
        if (c == 1 || c % 200 == 0)
            fprintf(stderr,
                    "[CPP-GRAPHPROBE] generateSegments calls=%zu | nodes=%zu | edges=%zu | "
                    "upward_quad_mids=%zu | nodes with bead_count>0=%zu\n",
                    c, nodes, edges, upward, beaded);
    }

    std::vector<edge_t*> upward_quad_mids;"""

# R548: census after each marking stage. `bead_count` is assigned to `edge.to` of
# every central edge (updateBeadCount), so this separates "we mark fewer edges
# central" from "we mark the same edges and assign bead_count <= 0 more often".
ST_CENTRAL_FN = r"""
static void centralprobe(const char *stage, size_t edges, size_t central_set, size_t central,
                         size_t nodes, const size_t *bc_hist)
{
    struct Acc { size_t edges, central_set, central, nodes, bc[6]; };
    static std::mutex                 mtx;
    static std::map<std::string, Acc> acc;
    static size_t                     rounds = 0;

    std::lock_guard<std::mutex> lock(mtx);
    Acc &e = acc[stage];
    e.edges += edges;
    e.central_set += central_set;
    e.central += central;
    e.nodes += nodes;
    for (int i = 0; i < 6; ++i)
        e.bc[i] += bc_hist[i];

    // One table per N completed sequences, so every stage shares a population.
    if (stage[0] == '4' && ++rounds % 4000 == 0) {
        fprintf(stderr, "[CPP-CENTRALPROBE] ---- cumulative ----\n");
        for (const auto &kv : acc) {
            const Acc &a = kv.second;
            fprintf(stderr,
                    "  %-28s edges=%9zu central=%9zu (%5.1f%%) set=%9zu | nodes=%9zu "
                    "bc[-1]=%8zu bc[0]=%8zu bc[1]=%8zu bc[2]=%8zu bc[3]=%8zu bc[4+]=%8zu\n",
                    kv.first.c_str(), a.edges, a.central,
                    100. * double(a.central) / double(a.edges ? a.edges : 1), a.central_set,
                    a.nodes, a.bc[0], a.bc[1], a.bc[2], a.bc[3], a.bc[4], a.bc[5]);
        }
    }
}

void SkeletalTrapezoidation::centralCensus(const char *stage)
{
    size_t central_set = 0, central = 0;
    for (const edge_t &e : graph.edges) {
        if (e.data.centralIsSet()) {
            ++central_set;
            if (const_cast<edge_t &>(e).data.isCentral())
                ++central;
        }
    }
    size_t bc[6] = {0, 0, 0, 0, 0, 0};
    for (const node_t &nd : graph.nodes) {
        const coord_t c = nd.data.bead_count;
        if (c < 0)          ++bc[0];
        else if (c == 0)    ++bc[1];
        else if (c == 1)    ++bc[2];
        else if (c == 2)    ++bc[3];
        else if (c == 3)    ++bc[4];
        else                ++bc[5];
    }
    centralprobe(stage, graph.edges.size(), central_set, central, graph.nodes.size(), bc);
}
"""

ST_CENTRAL_CALLS_OLD = """    updateIsCentral();
"""
ST_CENTRAL_CALLS_NEW = """    updateIsCentral();
    if (probe_enabled("CENTRALPROBE")) centralCensus("0 after updateIsCentral");
"""

ST_CENTRAL_CALLS2_OLD = """    filterCentral(central_filter_dist);
"""
ST_CENTRAL_CALLS2_NEW = """    filterCentral(central_filter_dist);
    if (probe_enabled("CENTRALPROBE")) centralCensus("1 after filterCentral");
"""

ST_CENTRAL_CALLS3_OLD = """    updateBeadCount();
"""
ST_CENTRAL_CALLS3_NEW = """    updateBeadCount();
    if (probe_enabled("CENTRALPROBE")) centralCensus("2 after updateBeadCount");
"""

ST_CENTRAL_CALLS4_OLD = """    filterNoncentralRegions();
"""
ST_CENTRAL_CALLS4_NEW = """    filterNoncentralRegions();
    if (probe_enabled("CENTRALPROBE")) centralCensus("3 after filterNoncentralRegions");
"""

ST_CENTRAL_CALLS5_OLD = """    generateTransitioningRibs();
"""
ST_CENTRAL_CALLS5_NEW = """    generateTransitioningRibs();
    if (probe_enabled("CENTRALPROBE")) centralCensus("4 after generateTransitioningRibs");
"""

ST_HPP_OLD = """    void updateIsCentral();"""
ST_HPP_NEW = """    void updateIsCentral();

    // Parity instrumentation (env-gated, CENTRALPROBE); see scripts/inject-arachne-probes.py
    void centralCensus(const char *stage);"""

WTP_INCLUDES_OLD = """#include <algorithm> //For std::partition_copy and std::min_element.
#include <unordered_set>"""

WTP_INCLUDES_NEW = """#include <algorithm> //For std::partition_copy and std::min_element.
#include <unordered_set>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <mutex>
#include <set>
#include <string>"""

WTP_PROBE_OLD = """namespace Slic3r::Arachne
{"""

WTP_PROBE_NEW = r"""bool &probe_speculative(); // R581, defined in SkeletalTrapezoidation.cpp (global scope)

namespace Slic3r::Arachne
{

// ---------------------------------------------------------------------------
// Parity instrumentation (env-gated via STAGEPROBE, off by default). Mirrors
// `stageprobe` in crates/libslic3r-rs/src/arachne/wall_tool_paths.rs:
// per-ExtrusionLine flat% and distinct widths per line, per stage.
// R583: also count width CHANGES. The five stages were cleared on junction and
// line counts (R544/R547/R558) but never on change density, which is what the
// 2.62x tags-per-line term is made of.
// R583: speculation-gated (R581) -- this probe predates probe_speculative() and
// was still counting the discarded one-wall pass.
// ---------------------------------------------------------------------------
static void stageprobe(const char *stage, const std::vector<VariableWidthLines> &toolpaths)
{
    if (::getenv("STAGEPROBE") == nullptr || probe_speculative())
        return;

    // R583b: inset 0 broken out -- the all-inset density narrows through these
    // stages while the outer wall reaching the ZPath is 2.10x adrift.
    struct Acc { size_t lines = 0, juncs = 0, flat = 0, distinct_total = 0, changes = 0,
                        l0 = 0, j0 = 0, c0 = 0; };
    static std::mutex                 mtx;
    static std::map<std::string, Acc> acc;

    size_t lines = 0, juncs = 0, flat = 0, distinct_total = 0, changes = 0;
    size_t l0 = 0, j0 = 0, c0 = 0;
    for (const VariableWidthLines &vwl : toolpaths)
        for (const ExtrusionLine &line : vwl) {
            if (line.junctions.empty())
                continue;
            ++lines;
            juncs += line.junctions.size();
            size_t ch = 0;
            for (size_t k = 1; k < line.junctions.size(); ++k)
                if (line.junctions[k].w != line.junctions[k - 1].w)
                    ++ch;
            changes += ch;
            if (line.inset_idx == 0) {
                ++l0;
                j0 += line.junctions.size();
                c0 += ch;
            }
            std::vector<coord_t> ws;
            ws.reserve(line.junctions.size());
            for (const ExtrusionJunction &j : line.junctions)
                ws.emplace_back(j.w);
            std::sort(ws.begin(), ws.end());
            ws.erase(std::unique(ws.begin(), ws.end()), ws.end());
            distinct_total += ws.size();
            if (ws.size() <= 1)
                ++flat;
        }

    std::lock_guard<std::mutex> lock(mtx);
    Acc &e = acc[stage];
    e.lines += lines;
    e.juncs += juncs;
    e.flat += flat;
    e.distinct_total += distinct_total;
    e.changes += changes;
    e.l0 += l0;
    e.j0 += j0;
    e.c0 += c0;

    if (stage[0] == '5' && e.lines > 0 && e.lines % 20000 < std::max<size_t>(lines, 1)) {
        fprintf(stderr, "[CPP-STAGEPROBE] ---- cumulative ----\n");
        for (const auto &kv : acc)
            fprintf(stderr, "  %-38s lines=%8zu juncs=%9zu flat=%5.1f%% distinct_w/line=%.2f "
                            "ch=%8zu ch/line=%.4f ch/junc=%.5f | "
                            "i0_lines=%7zu i0_juncs=%9zu i0_ch=%8zu i0_ch/junc=%.5f\n",
                    kv.first.c_str(), kv.second.lines, kv.second.juncs,
                    100. * double(kv.second.flat) / double(std::max<size_t>(kv.second.lines, 1)),
                    double(kv.second.distinct_total) / double(std::max<size_t>(kv.second.lines, 1)),
                    kv.second.changes,
                    double(kv.second.changes) / double(std::max<size_t>(kv.second.lines, 1)),
                    double(kv.second.changes) / double(std::max<size_t>(kv.second.juncs, 1)),
                    kv.second.l0, kv.second.j0, kv.second.c0,
                    double(kv.second.c0) / double(std::max<size_t>(kv.second.j0, 1)));
    }
}"""

WTP_CHAIN_OLD = """    wall_maker.generateToolpaths(toolpaths);

    stitchToolPaths(toolpaths, this->bead_width_x);

    removeSmallLines(toolpaths);

    separateOutInnerContour();

    simplifyToolPaths(toolpaths);

    removeEmptyToolPaths(toolpaths);"""

WTP_CHAIN_NEW = """    wall_maker.generateToolpaths(toolpaths);
    stageprobe("0 after generate_toolpaths", toolpaths);

    stitchToolPaths(toolpaths, this->bead_width_x);
    stageprobe("1 after stitch_tool_paths", toolpaths);

    removeSmallLines(toolpaths);
    stageprobe("2 after remove_small_lines", toolpaths);

    separateOutInnerContour();
    stageprobe("3 after separate_out_inner_contour", toolpaths);

    simplifyToolPaths(toolpaths);
    stageprobe("4 after simplify_tool_paths", toolpaths);

    removeEmptyToolPaths(toolpaths);
    stageprobe("5 after remove_empty_tool_paths", toolpaths);"""

ST_ISC_FN = r"""
// R548: which branch of updateIsCentral decides each edge, and the two constants
// the last two branches turn on.
static void iscprobe(int branch, bool central, coord_t oefl, float cap)
{
    static std::mutex mtx;
    static size_t n = 0, taken[4] = {0,0,0,0}, yes[4] = {0,0,0,0};
    std::lock_guard<std::mutex> lock(mtx);
    ++n; ++taken[branch]; if (central) ++yes[branch];
    if (n == 1 || n % 2000000 == 0)
        fprintf(stderr,
            "[CPP-ISCPROBE] edges=%zu | twin-copy=%zu(central %zu) extra_vd=%zu short=%zu "
            "geom=%zu(central %zu) | outer_edge_filter_length=%.4fmm cap=%.6f\n",
            n, taken[0], yes[0], taken[1], taken[2], taken[3], yes[3],
            double(oefl) / 1e5, double(cap));
}

void SkeletalTrapezoidation::updateIsCentral()"""

ST_ISC_ANCHOR = "\nvoid SkeletalTrapezoidation::updateIsCentral()"

ST_ISC_BRANCHES_OLD = """        if(edge.twin->data.centralIsSet())
        {
            edge.data.setIsCentral(edge.twin->data.isCentral());
        }
        else if(edge.data.type == SkeletalTrapezoidationEdge::EdgeType::EXTRA_VD)
        {
            edge.data.setIsCentral(false);
        }
        else if(std::max(edge.from->data.distance_to_boundary, edge.to->data.distance_to_boundary) < outer_edge_filter_length)
        {
            edge.data.setIsCentral(false);
        }
        else
        {"""

ST_ISC_BRANCHES_NEW = """        if(edge.twin->data.centralIsSet())
        {
            edge.data.setIsCentral(edge.twin->data.isCentral());
            if (probe_enabled("ISCPROBE")) iscprobe(0, edge.data.isCentral(), outer_edge_filter_length, cap);
        }
        else if(edge.data.type == SkeletalTrapezoidationEdge::EdgeType::EXTRA_VD)
        {
            edge.data.setIsCentral(false);
            if (probe_enabled("ISCPROBE")) iscprobe(1, false, outer_edge_filter_length, cap);
        }
        else if(std::max(edge.from->data.distance_to_boundary, edge.to->data.distance_to_boundary) < outer_edge_filter_length)
        {
            edge.data.setIsCentral(false);
            if (probe_enabled("ISCPROBE")) iscprobe(2, false, outer_edge_filter_length, cap);
        }
        else
        {"""

ST_ISC_GEOM_OLD = """            edge.data.setIsCentral(dR < dD * cap);
        }"""

ST_ISC_GEOM_NEW = """            edge.data.setIsCentral(dR < dD * cap);
            if (probe_enabled("ISCPROBE")) iscprobe(3, dR < dD * cap, outer_edge_filter_length, cap);
        }"""


WTP_PARAMS_OLD = """    const double  transitioning_angle = Geometry::deg2rad(m_params.wall_transition_angle);"""

WTP_PARAMS_NEW = r"""    const double  transitioning_angle = Geometry::deg2rad(m_params.wall_transition_angle);
    if (::getenv("WTPPARAMS") != nullptr) { // R550
        static std::mutex pmtx;
        static std::set<std::string> seen;
        char buf[512];
        snprintf(buf, sizeof(buf),
                 "min_bead_width=%.6f min_feature_size=%.6f wall_transition_length=%.6f "
                 "wall_transition_angle=%.6f(deg) -> %.9f(rad) wall_transition_filter_deviation=%.6f "
                 "wall_distribution_count=%d",
                 double(m_params.min_bead_width), double(m_params.min_feature_size),
                 double(m_params.wall_transition_length), double(m_params.wall_transition_angle),
                 transitioning_angle, double(m_params.wall_transition_filter_deviation),
                 int(m_params.wall_distribution_count));
        std::lock_guard<std::mutex> lock(pmtx);
        if (seen.insert(std::string(buf)).second)
            fprintf(stderr, "[CPP-WTPPARAMS] %s\n", buf);
    }"""


ST_TRANS_FN = r"""
// R551: transition census after each stage of generateTransitioningRibs. The
// direct analogue of the failing G-code metric (how often width changes along a
// wall). Mirrors `transition_census` in the Rust crate.
static void transprobe(const char *stage, size_t edges_with, size_t total_items)
{
    struct Acc { size_t calls, edges_with, items; };
    static std::mutex                 mtx;
    static std::map<std::string, Acc> acc;
    static size_t                     rounds = 0;

    std::lock_guard<std::mutex> lock(mtx);
    Acc &a = acc[stage];
    ++a.calls;
    a.edges_with += edges_with;
    a.items += total_items;

    if (stage[0] == '3' && ++rounds % 4000 == 0) {
        fprintf(stderr, "[CPP-TRANSPROBE] ---- cumulative ----\n");
        for (const auto &kv : acc)
            fprintf(stderr, "  %-34s calls=%7zu edges_with_transitions=%9zu items=%9zu\n",
                    kv.first.c_str(), kv.second.calls, kv.second.edges_with, kv.second.items);
    }
}

void SkeletalTrapezoidation::transitionCensus(const char *stage)
{
    size_t edges_with = 0, items = 0;
    for (edge_t &e : graph.edges) {
        if (e.data.hasTransitions(true)) {
            ++edges_with;
            if (auto t = e.data.getTransitions())
                items += t->size();
        }
    }
    transprobe(stage, edges_with, items);
}

void SkeletalTrapezoidation::generateTransitioningRibs()"""

ST_TRANS_ANCHOR = "\nvoid SkeletalTrapezoidation::generateTransitioningRibs()"

ST_TRANS_CALLS_OLD = """    generateTransitionMids(edge_transitions);
"""
ST_TRANS_CALLS_NEW = """    generateTransitionMids(edge_transitions);
    if (probe_enabled("TRANSPROBE")) transitionCensus("0 after generateTransitionMids");
"""

ST_TRANS_CALLS2_OLD = """    filterTransitionMids();
"""
ST_TRANS_CALLS2_NEW = """    filterTransitionMids();
    if (probe_enabled("TRANSPROBE")) transitionCensus("1 after filterTransitionMids");
"""

ST_TRANS_CALLS3_OLD = """    generateAllTransitionEnds(edge_transition_ends);
"""
ST_TRANS_CALLS3_NEW = """    generateAllTransitionEnds(edge_transition_ends);
    if (probe_enabled("TRANSPROBE")) transitionCensus("2 after generateAllTransitionEnds");
"""

ST_TRANS_CALLS4_OLD = """    applyTransitions(edge_transition_ends);
"""
ST_TRANS_CALLS4_NEW = """    applyTransitions(edge_transition_ends);
    if (probe_enabled("TRANSPROBE")) transitionCensus("3 after applyTransitions");
"""

ST_TRANS_HPP_OLD = """    void updateIsCentral();"""
ST_TRANS_HPP_NEW = """    void updateIsCentral();

    // Parity instrumentation (env-gated, TRANSPROBE); see scripts/inject-arachne-probes.py
    void transitionCensus(const char *stage);"""


WTP_POLY_FN = r"""
// R552: polygon and point counts through the prepared_outline preparation chain.
// Brackets the 2.33x graph-edge gap at its source.
static void polyprobe(const char *stage, const Polygons &polys)
{
    // R694: exclude the R581 speculative one-wall pass, as UPPROBE / PROPCLASS /
    // GBUILD / GRAPHPROBE (R692) do. POLYPROBE is an R552 probe and predates the
    // flag, so it was counting outline preparation that never reaches the
    // G-code — 50,200 calls against the Rust side's 28,000.
    if (::getenv("POLYPROBE") == nullptr || probe_speculative())
        return;
    struct Acc { size_t calls, polys, points; };
    static std::mutex                 mtx;
    static std::map<std::string, Acc> acc;
    static size_t                     rounds = 0;
    size_t pts = 0;
    for (const Polygon &p : polys)
        pts += p.points.size();
    std::lock_guard<std::mutex> lock(mtx);
    Acc &a = acc[stage];
    ++a.calls;
    a.polys += polys.size();
    a.points += pts;
    // R561: was `% 4000`, which capped the last print at 48,000 calls on BOTH
    // engines and so never showed a TOTAL. R560 therefore compared them at a
    // matched call INDEX, which is not a matched surface set, because C++ calls
    // generate() ~twice per surface. 200 puts the last print within 200 calls of
    // the true total, making totals/calls a sound per-surface mean.
    if (stage[0] == '3' && ++rounds % 200 == 0) {
        fprintf(stderr, "[CPP-POLYPROBE] ---- cumulative ----\n");
        for (const auto &kv : acc)
            fprintf(stderr, "  %-30s calls=%7zu polys=%9zu points=%10zu\n",
                    kv.first.c_str(), kv.second.calls, kv.second.polys, kv.second.points);
    }
}
"""

WTP_AREA_FN = r"""
// R562: mirrors `areaprobe` in crates/libslic3r-rs/src/arachne/wall_tool_paths.rs.
// R561 found 49.6% of Rust's generate() calls end at the `area <= 0` return
// against C++'s 18.3%. This attributes each failure to the step that zeroes the
// area, separating "the surface arrived empty" from "a step killed it" (R511),
// and splits an empty input from a non-empty input with signed area <= 0.
static void areaprobe(const std::vector<double> &areas, size_t n_input_polys)
{
    if (::getenv("AREAPROBE") == nullptr)
        return;
    static const char *STEPS[10] = {
        "0 input outline", "1 triple offset", "2 simplify", "3 fixSelfIntersections",
        "4 removeDegenerateVerts", "5 removeColinearEdges", "6 fixSelfIntersections",
        "7 removeDegenerateVerts", "8 removeSmallAreas", "9 union_"};
    static std::mutex mtx;
    static size_t calls = 0, survived = 0, first_zero[10] = {0}, empty_in = 0, neg_area_in = 0;
    std::lock_guard<std::mutex> lock(mtx);
    ++calls;
    const double final_area = areas.empty() ? 0. : areas.back();
    if (final_area > 0.) {
        ++survived;
    } else {
        for (size_t k = 0; k < areas.size(); ++k)
            if (areas[k] <= 0.) {
                ++first_zero[k < 10 ? k : 9];
                if (k == 0) { if (n_input_polys == 0) ++empty_in; else ++neg_area_in; }
                break;
            }
    }
    if (calls == 1 || calls % 5000 == 0) {
        const size_t failed = calls - survived;
        fprintf(stderr,
                "[CPP-AREAPROBE] calls=%zu survived=%zu failed=%zu (%.1f%%) | "
                "input_empty=%zu input_nonempty_but_area<=0=%zu\n",
                calls, survived, failed, 100. * double(failed) / double(calls),
                empty_in, neg_area_in);
        for (size_t k = 0; k < 10; ++k)
            if (first_zero[k] > 0)
                fprintf(stderr, "    first_zero_at %-26s %8zu  (%.1f%% of failures)\n",
                        STEPS[k], first_zero[k],
                        100. * double(first_zero[k]) / double(failed ? failed : 1));
    }
}
"""

WTP_AREA_CALLS_OLD = """    Polygons prepared_outline = offset(offset(offset(outline, -epsilon_offset), epsilon_offset * 2), -epsilon_offset);
    polyprobe("1 after triple offset", prepared_outline);"""

WTP_AREA_CALLS_NEW = """    std::vector<double> ap_areas; // R562
    const bool ap = ::getenv("AREAPROBE") != nullptr;
    if (ap) ap_areas.emplace_back(area(outline));
    Polygons prepared_outline = offset(offset(offset(outline, -epsilon_offset), epsilon_offset * 2), -epsilon_offset);
    polyprobe("1 after triple offset", prepared_outline);
    if (ap) ap_areas.emplace_back(area(prepared_outline));"""

WTP_AREA_CALLS2_OLD = """    process_with_size_check([&] { simplify(prepared_outline, smallest_segment, allowed_distance);});
    polyprobe("2 after simplify", prepared_outline);"""

WTP_AREA_CALLS2_NEW = """    process_with_size_check([&] { simplify(prepared_outline, smallest_segment, allowed_distance);});
    polyprobe("2 after simplify", prepared_outline);
    if (ap) ap_areas.emplace_back(area(prepared_outline));"""

WTP_AREA_CALLS3_OLD = """    process_with_size_check([&] { fixSelfIntersections(epsilon_offset, prepared_outline); });
    process_with_size_check([&] { removeDegenerateVerts(prepared_outline); });
    process_with_size_check([&] { removeColinearEdges(prepared_outline, 0.005); });
    // Removing collinear edges may introduce self intersections, so we need to fix them again
    process_with_size_check([&] { fixSelfIntersections(epsilon_offset, prepared_outline); });
    process_with_size_check([&] { removeDegenerateVerts(prepared_outline); });
    process_with_size_check([&] { removeSmallAreas(prepared_outline, small_area_length * small_area_length, false); });"""

WTP_AREA_CALLS3_NEW = """    process_with_size_check([&] { fixSelfIntersections(epsilon_offset, prepared_outline); });
    if (ap) ap_areas.emplace_back(area(prepared_outline));
    process_with_size_check([&] { removeDegenerateVerts(prepared_outline); });
    if (ap) ap_areas.emplace_back(area(prepared_outline));
    process_with_size_check([&] { removeColinearEdges(prepared_outline, 0.005); });
    if (ap) ap_areas.emplace_back(area(prepared_outline));
    // Removing collinear edges may introduce self intersections, so we need to fix them again
    process_with_size_check([&] { fixSelfIntersections(epsilon_offset, prepared_outline); });
    if (ap) ap_areas.emplace_back(area(prepared_outline));
    process_with_size_check([&] { removeDegenerateVerts(prepared_outline); });
    if (ap) ap_areas.emplace_back(area(prepared_outline));
    process_with_size_check([&] { removeSmallAreas(prepared_outline, small_area_length * small_area_length, false); });
    if (ap) ap_areas.emplace_back(area(prepared_outline));"""

WTP_AREA_CALLS4_OLD = """    polyprobe("3 final prepared_outline", prepared_outline);
    update_outline_size_change(prepared_outline);"""

WTP_AREA_CALLS4_NEW = """    polyprobe("3 final prepared_outline", prepared_outline);
    if (ap) { ap_areas.emplace_back(area(prepared_outline)); areaprobe(ap_areas, outline.size()); }
    update_outline_size_change(prepared_outline);"""

WTP_POLY_ANCHOR = """static void stageprobe(const char *stage, const std::vector<VariableWidthLines> &toolpaths)"""

WTP_POLY_CALLS_OLD = """    Polygons prepared_outline = offset(offset(offset(outline, -epsilon_offset), epsilon_offset * 2), -epsilon_offset);
    update_outline_size_change(prepared_outline);"""

WTP_POLY_CALLS_NEW = """    polyprobe("0 outline", outline);
    Polygons prepared_outline = offset(offset(offset(outline, -epsilon_offset), epsilon_offset * 2), -epsilon_offset);
    polyprobe("1 after triple offset", prepared_outline);
    update_outline_size_change(prepared_outline);"""

WTP_POLY_CALLS2_OLD = """    process_with_size_check([&] { simplify(prepared_outline, smallest_segment, allowed_distance);});"""

WTP_POLY_CALLS2_NEW = """    process_with_size_check([&] { simplify(prepared_outline, smallest_segment, allowed_distance);});
    polyprobe("2 after simplify", prepared_outline);"""

WTP_POLY_CALLS3_OLD = """    prepared_outline = union_(prepared_outline);
    update_outline_size_change(prepared_outline);"""

WTP_POLY_CALLS3_NEW = """    prepared_outline = union_(prepared_outline);
    polyprobe("3 final prepared_outline", prepared_outline);
    update_outline_size_change(prepared_outline);"""


PG_LAST_FN = r"""
// R553: contour/hole census of `last` and `last_p` where PerimeterGenerator hands
// the region to Arachne. Splits contours from holes because a union that merges
// contours preserves area while halving the count -- the observed signature.
static void lastprobe(const char *stage, size_t contours, size_t holes, size_t points)
{
    if (::getenv("LASTPROBE") == nullptr)
        return;
    struct Acc { size_t calls, contours, holes, points; };
    static std::mutex                 mtx;
    static std::map<std::string, Acc> acc;
    static size_t                     rounds = 0;
    std::lock_guard<std::mutex> lock(mtx);
    Acc &a = acc[stage];
    ++a.calls;
    a.contours += contours;
    a.holes += holes;
    a.points += points;
    if (stage[0] == 'D' && ++rounds % 4000 == 0) {
        fprintf(stderr, "[CPP-LASTPROBE] ---- cumulative ----\n");
        for (const auto &kv : acc)
            fprintf(stderr, "  %-26s calls=%7zu contours=%9zu holes=%9zu points=%10zu\n",
                    kv.first.c_str(), kv.second.calls, kv.second.contours,
                    kv.second.holes, kv.second.points);
    }
}
"""

PG_LAST_ANCHOR = """void PerimeterGenerator::process_arachne()"""

PG_LAST_CALLS_OLD = """        std::vector<int> circle_poly_indices;
        Polygons   last_p;"""

PG_LAST_CALLS_NEW = """        if (::getenv("LASTPROBE") != nullptr) { // R553
            size_t sp = surface.expolygon.contour.points.size();
            for (const Polygon &h : surface.expolygon.holes) sp += h.points.size();
            lastprobe("A surface.expolygon", 1, surface.expolygon.holes.size(), sp);
            size_t lc = 0, lh = 0, lp = 0;
            for (const ExPolygon &e : last) {
                ++lc; lh += e.holes.size(); lp += e.contour.points.size();
                for (const Polygon &h : e.holes) lp += h.points.size();
            }
            lastprobe("C last (ExPolygons)", lc, lh, lp);
        }
        std::vector<int> circle_poly_indices;
        Polygons   last_p;"""

PG_LAST_CALLS2_OLD = """        std::vector<Arachne::VariableWidthLines> total_perimeters;
        ExPolygons infill_contour;"""

PG_LAST_CALLS2_NEW = """        if (::getenv("LASTPROBE") != nullptr) { // R553
            size_t pp = 0;
            for (const Polygon &p : last_p) pp += p.points.size();
            lastprobe("D last_p (Polygons)", last_p.size(), 0, pp);
        }
        std::vector<Arachne::VariableWidthLines> total_perimeters;
        ExPolygons infill_contour;"""

PG_JW_FN = r"""
// R558: per-ExtrusionLine junction-WIDTH census, taken where PerimeterGenerator
// builds the Z subject path from `extrusion->junctions` -- the exact point the
// Rust LINEPROBE/ARACHWIDTH probes measure. The Rust side reports 84.4% of loops
// with min==max (43 junctions, 1.45 distinct widths); this is the reference to
// compare that against. If C++ is far less flat, the width gap is in the BEADING,
// upstream of every path-construction and emission mechanism already eliminated.
// R567 adds the ORDERING-sensitive quantity: how often CONSECUTIVE junctions
// differ. R566 established the output gap is a RATE (C++ changes width 2.73x more
// often per extruded move). Distinct-value counts and spread are both blind to
// ordering and so cannot speak to a rate; this is the direct internal analogue.
// Mirrors the Rust ARACHWIDTH extension exactly.
static void jwprobe(size_t njunc, size_t distinct, coord_t spread, bool flat,
                    size_t transitions, size_t changes, bool outer)
{
    if (::getenv("JWPROBE") == nullptr)
        return;
    static std::mutex mtx;
    static size_t loops = 0, tot_j = 0, tot_d = 0, tot_flat = 0;
    static size_t tot_tr = 0, tot_ch = 0, v_loops = 0, v_tr = 0, v_ch = 0;
    static size_t o_loops = 0, o_tr = 0, o_ch = 0; // R567: outer wall only
    static double tot_spread_um = 0;
    std::lock_guard<std::mutex> lock(mtx);
    ++loops; tot_j += njunc; tot_d += distinct; tot_spread_um += double(spread) / 100.0;
    tot_tr += transitions; tot_ch += changes;
    if (flat) ++tot_flat;
    else { ++v_loops; v_tr += transitions; v_ch += changes; }
    if (outer) { ++o_loops; o_tr += transitions; o_ch += changes; }
    if (loops == 1 || loops % 5000 == 0)
        fprintf(stderr,
                "[CPP-JWPROBE] loops=%zu flat(min==max)=%zu (%.1f%%) mean_spread=%.1fum "
                "juncs/loop=%.2f distinct_w/loop=%.3f | CHANGE_RATE all=%.4f (%zu/%zu) "
                "varying=%.4f (%zu/%zu) v_loops=%zu OUTER=%.4f (%zu/%zu) o_loops=%zu\n",
                loops, tot_flat, 100.0 * double(tot_flat) / double(loops),
                tot_spread_um / double(loops),
                double(tot_j) / double(loops), double(tot_d) / double(loops),
                double(tot_ch) / double(tot_tr ? tot_tr : 1), tot_ch, tot_tr,
                double(v_ch) / double(v_tr ? v_tr : 1), v_ch, v_tr, v_loops,
                double(o_ch) / double(o_tr ? o_tr : 1), o_ch, o_tr, o_loops);
}
"""

# The call site lives in `traverse_extrusions`, which is defined BEFORE
# `process_arachne`, so the definition must be anchored here -- anchoring it on
# process_arachne compiles to "use of undeclared identifier 'jwprobe'".
PG_JW_ANCHOR = """static ExtrusionEntityCollection traverse_extrusions(const PerimeterGenerator& perimeter_generator, std::vector<PerimeterGeneratorArachneExtrusion>& pg_extrusions)"""

PG_JW_CALLS_OLD = """            ZPath subject_path;
            for (auto& ej : extrusion->junctions)
                subject_path.emplace_back(ej.p.x(), ej.p.y(), ej.w);"""

PG_JW_CALLS_NEW = """            ZPath subject_path;
            for (auto& ej : extrusion->junctions)
                subject_path.emplace_back(ej.p.x(), ej.p.y(), ej.w);
            if (::getenv("JWPROBE") != nullptr) { // R558
                std::vector<coord_t> ws;
                ws.reserve(extrusion->junctions.size());
                for (const Arachne::ExtrusionJunction &ej : extrusion->junctions)
                    ws.emplace_back(ej.w);
                if (!ws.empty()) {
                    coord_t mn = *std::min_element(ws.begin(), ws.end());
                    coord_t mx = *std::max_element(ws.begin(), ws.end());
                    std::vector<coord_t> u = ws;
                    std::sort(u.begin(), u.end());
                    u.erase(std::unique(u.begin(), u.end()), u.end());
                    size_t ch = 0; // R567: consecutive junctions that differ
                    for (size_t k = 1; k < ws.size(); ++k)
                        if (ws[k] != ws[k - 1]) ++ch;
                    jwprobe(ws.size(), u.size(), mx - mn, mn == mx, ws.size() - 1, ch,
                            extrusion->inset_idx == 0);
                }
            }"""

PG_TOW_FN = r"""
// R560: does the top-one-wall path actually fire? `seperate_wall_generation`
// (PerimeterGenerator.cpp:1534) is the reason C++ builds a SECOND WallToolPaths
// per surface -- :1560 runs a full one-wall generate PURELY to decide it, which
// is what inflated R558's cumulative stage-0 line count. Every sub-condition is
// counted separately (R536) so a false result can be attributed, and the
// post-detection value is counted too: `should_enable_top_one_wall` can flip it
// back to false AFTER the speculative generate has already run.
static void towprobe(int stage, bool by_first_layer, bool by_top_most, bool by_top,
                     bool is_one_wall, bool seperate, int loop_number)
{
    if (::getenv("TOWPROBE") == nullptr)
        return;
    static std::mutex mtx;
    static size_t surfaces = 0, n_first = 0, n_topmost = 0, n_bytop = 0,
                  n_onewall = 0, n_sep_pre = 0, n_sep_post = 0, n_detect = 0,
                  n_loop0 = 0;
    std::lock_guard<std::mutex> lock(mtx);
    if (stage == 0) {
        ++surfaces;
        if (by_first_layer) ++n_first;
        if (by_top_most)    ++n_topmost;
        if (by_top)         ++n_bytop;
        if (is_one_wall)    ++n_onewall;
        if (loop_number == 0) ++n_loop0;
        if (seperate)       ++n_sep_pre;
    } else {
        ++n_detect;                       // speculative generate actually ran
        if (seperate)   ++n_sep_post;     // survived should_enable_top_one_wall
        if (n_detect % 500 == 0 || n_detect == 1)
            fprintf(stderr,
                    "[CPP-TOWPROBE] surfaces=%zu | loop_number==0=%zu by_first_layer=%zu "
                    "by_top_most=%zu | is_one_wall=%zu | by_top=%zu | seperate_PRE=%zu "
                    "detect_runs=%zu seperate_POST=%zu (%.1f%% of detects survive)\n",
                    surfaces, n_loop0, n_first, n_topmost, n_onewall, n_bytop,
                    n_sep_pre, n_detect, n_sep_post,
                    100. * double(n_sep_post) / double(n_detect ? n_detect : 1));
    }
}
"""

PG_TOW_ANCHOR = """void PerimeterGenerator::process_arachne()"""

PG_TOW_CALLS_OLD = """            bool seperate_wall_generation = !is_one_wall && generate_one_wall_by_top;
"""

PG_TOW_CALLS_NEW = """            bool seperate_wall_generation = !is_one_wall && generate_one_wall_by_top;
            towprobe(0, generate_one_wall_by_first_layer, generate_one_wall_by_top_most, // R560
                     generate_one_wall_by_top, is_one_wall, seperate_wall_generation, int(loop_number));
"""

PG_TOW_CALLS2_OLD = """                seperate_wall_generation = should_enable_top_one_wall(last, top_expolys_by_one_wall);"""

PG_TOW_CALLS2_NEW = """                seperate_wall_generation = should_enable_top_one_wall(last, top_expolys_by_one_wall);
                towprobe(1, false, false, false, false, seperate_wall_generation, 0); // R560"""

PG_INCLUDES_OLD = """#include "PerimeterGenerator.hpp\""""
PG_INCLUDES_NEW = """#include "PerimeterGenerator.hpp"
bool& probe_speculative();   // R581, defined in SkeletalTrapezoidation.cpp
#include <cstdio>
#include <cstdlib>
#include <map>
#include <mutex>
#include <string>"""


LR_MP_FN = r"""
// R555: what LayerRegion::make_perimeters actually receives. Counts surfaces,
// holes and points per (layer, region) so the 1.5x surface gap can be localised
// to a layer band or a region rather than only seen in aggregate.
static void mpprobe(int layer_id, int region_id, size_t surfaces, size_t holes, size_t points)
{
    if (::getenv("MPPROBE") == nullptr)
        return;
    static std::mutex mtx;
    static size_t calls = 0, tot_s = 0, tot_h = 0, tot_p = 0;
    std::lock_guard<std::mutex> lock(mtx);
    ++calls; tot_s += surfaces; tot_h += holes; tot_p += points;
    if (calls == 1 || calls % 200 == 0)
        fprintf(stderr,
                "[CPP-MPPROBE] calls=%zu | surfaces=%zu holes=%zu points=%zu | "
                "last(layer=%d region=%d s=%zu h=%zu)\n",
                calls, tot_s, tot_h, tot_p, layer_id, region_id, surfaces, holes);
}
"""

LR_MP_ANCHOR = """void LayerRegion::make_perimeters(const SurfaceCollection &slices, const PerimeterRegions &perimeter_regions, SurfaceCollection *fill_surfaces, ExPolygons *fill_no_overlap, std::vector<LoopNode> &loop_nodes)
{"""

LR_MP_CALL_OLD = """    this->perimeters.clear();
    this->thin_fills.clear();"""

LR_MP_CALL_NEW = """    if (::getenv("MPPROBE") != nullptr) { // R555
        size_t h = 0, p = 0;
        for (const Surface &s : slices.surfaces) {
            h += s.expolygon.holes.size();
            p += s.expolygon.contour.points.size();
            for (const Polygon &hp : s.expolygon.holes) p += hp.points.size();
        }
        mpprobe(int(this->layer()->id()), -1, slices.surfaces.size(), h, p);
    }
    this->perimeters.clear();
    this->thin_fills.clear();"""

LR_INCLUDES_OLD = """#include "RegionExpansion.hpp\""""
LR_INCLUDES_NEW = """#include "RegionExpansion.hpp"
#include <cstdio>
#include <cstdlib>
#include <mutex>"""


# ---------------------------------------------------------------------------
# TPMPPROBE (R569) - width variation ENTERING thick_polyline_to_multi_path vs
# the number of ExtrusionPaths LEAVING it. Each extra path is one intra-loop
# `; LINE_WIDTH:` tag, so this brackets the exact stage R568 localised.
# Mirrors the Rust probe of the same name in crates/libslic3r-rs/src/variable_width.rs.
# ---------------------------------------------------------------------------
VW_INCLUDES_OLD = '''#include "VariableWidth.hpp"
'''
VW_INCLUDES_NEW = '''#include "VariableWidth.hpp"
#include <atomic>
#include <algorithm>
#include <cstdio>
#include <cstdlib>
bool& probe_speculative();   // R582, defined in SkeletalTrapezoidation.cpp

// TPMPPROBE (R569)
static bool tpmp_on() { static bool v = getenv("TPMPPROBE") != nullptr; return v; }
static std::atomic<uint64_t> tpmp_calls{0}, tpmp_pts{0}, tpmp_changes{0},
                             tpmp_distinct{0}, tpmp_spread{0}, tpmp_flat{0}, tpmp_paths{0};
'''

VW_HEAD_OLD = '''    ExtrusionMultiPath multi_path;
    ExtrusionPath      path(role);
    ThickLines         lines = thick_polyline.thicklines();
'''
VW_HEAD_NEW = '''    ExtrusionMultiPath multi_path;
    ExtrusionPath      path(role);
    ThickLines         lines = thick_polyline.thicklines();

    // TPMPPROBE (R569) - scoped to the outer wall to match the G-code classification.
    const bool tpmp = tpmp_on() && role == erExternalPerimeter && !probe_speculative();
    size_t tpmp_chg = 0, tpmp_dis = 0;
    uint64_t tpmp_spr = 0;
    if (tpmp) {
        const std::vector<coordf_t> &ws = thick_polyline.width;
        for (size_t k = 1; k < ws.size(); ++k)
            if (ws[k] != ws[k - 1]) ++tpmp_chg;
        std::vector<coordf_t> d(ws.begin(), ws.end());
        std::sort(d.begin(), d.end());
        d.erase(std::unique(d.begin(), d.end()), d.end());
        tpmp_dis = d.size();
        if (!ws.empty())
            tpmp_spr = (uint64_t)(*std::max_element(ws.begin(), ws.end()) -
                                  *std::min_element(ws.begin(), ws.end()));
    }
'''

VW_TAIL_OLD = '''    if( path.polyline.is_valid() ) {
        path.overhang_degree = overhang;
        multi_path.paths.emplace_back(std::move(path));
    }
    return multi_path;
'''
VW_TAIL_NEW = '''    if( path.polyline.is_valid() ) {
        path.overhang_degree = overhang;
        multi_path.paths.emplace_back(std::move(path));
    }
    // TPMPPROBE (R569) - cumulative totals; take the LAST printed line.
    if (tpmp) {
        tpmp_pts += thick_polyline.width.size();
        tpmp_changes += tpmp_chg;
        tpmp_distinct += tpmp_dis;
        tpmp_spread += tpmp_spr;
        if (tpmp_chg == 0) ++tpmp_flat;
        tpmp_paths += multi_path.paths.size();
        uint64_t n = ++tpmp_calls;
        if (n % 1000 == 0)
            printf("TPMPPROBE calls=%llu widthpts=%llu in_changes=%llu in_distinct=%llu "
                   "in_spread=%llu flat_calls=%llu out_paths=%llu\\n",
                   (unsigned long long)n, (unsigned long long)tpmp_pts.load(),
                   (unsigned long long)tpmp_changes.load(),
                   (unsigned long long)tpmp_distinct.load(),
                   (unsigned long long)tpmp_spread.load(),
                   (unsigned long long)tpmp_flat.load(),
                   (unsigned long long)tpmp_paths.load());
    }
    return multi_path;
'''

# ---------------------------------------------------------------------------
# JUNCPROBE (R571) - the junction site is the FORK. bead_widths[idx] comes from a
# Beading whose input is total_thickness. Counting DISTINCT values of each is
# order-independent, so it is safe under parallelism (R559). Mirrors the Rust
# probe of the same name in arachne/skeletal_trapezoidation.rs.
# ---------------------------------------------------------------------------
ST_JUNC_OLD = '''            ret.emplace_back(ExtrusionJunction(junction, beading->bead_widths[junction_idx], junction_idx, apply_hole_compensation));
'''
ST_JUNC_NEW = '''            if (getenv("BEADPROBE") && !probe_speculative()) {
                static std::mutex jp_mtx;
                static std::vector<coord_t> jp_w, jp_t, jp_w0;
                static std::vector<std::pair<coord_t, coord_t>> jp_pairs;
                static size_t jp_n = 0, jp_idx0 = 0;
                std::lock_guard<std::mutex> jp_lock(jp_mtx);
                ++jp_n;
                // R584: restrict the width spread to bead index 0 -- the outer wall
                // draws every junction from bead_widths[0], so the global distinct
                // count (mixed across all indices) does not speak to it.
                if (junction_idx == 0) { ++jp_idx0; jp_w0.push_back(beading->bead_widths[0]); }
                jp_w.push_back(beading->bead_widths[junction_idx]);
                jp_t.push_back(beading->total_thickness);
                jp_pairs.emplace_back(beading->total_thickness, beading->bead_widths[junction_idx]);
                if (jp_n % 500000 == 0) {  // R584: was 20000; the dedup sorts grow with the vector
                    std::vector<coord_t> d(jp_w), dt(jp_t);
                    std::sort(d.begin(), d.end());
                    d.erase(std::unique(d.begin(), d.end()), d.end());
                    std::sort(dt.begin(), dt.end());
                    dt.erase(std::unique(dt.begin(), dt.end()), dt.end());
                    std::vector<std::pair<coord_t, coord_t>> dp(jp_pairs);
                    std::sort(dp.begin(), dp.end());
                    dp.erase(std::unique(dp.begin(), dp.end()), dp.end());
                    coord_t mn = *std::min_element(jp_w.begin(), jp_w.end());
                    coord_t mx = *std::max_element(jp_w.begin(), jp_w.end());
                    coord_t tmn = *std::min_element(jp_t.begin(), jp_t.end());
                    coord_t tmx = *std::max_element(jp_t.begin(), jp_t.end());
                    std::vector<coord_t> d0(jp_w0);
                    std::sort(d0.begin(), d0.end());
                    d0.erase(std::unique(d0.begin(), d0.end()), d0.end());
                    const size_t ep_edges = g_ep_edges.load();
                    fprintf(stderr,
                        "[JUNCPROBE] junctions=%zu idx0=%zu | distinct_width=%zu distinct_thick=%zu distinct_pairs=%zu | w_range=%.3f..%.3fmm t_range=%.3f..%.3fmm | edges=%zu juncs/edge=%.4f distinct_w0=%zu w0_per_idx0=%.6f\\n",
                        jp_n, jp_idx0, d.size(), dt.size(), dp.size(),
                        mn / 1e5, mx / 1e5, tmn / 1e5, tmx / 1e5,
                        ep_edges, double(jp_n) / double(std::max<size_t>(ep_edges, 1)),
                        d0.size(), double(d0.size()) / double(std::max<size_t>(jp_idx0, 1)));
                }
            }
            ret.emplace_back(ExtrusionJunction(junction, beading->bead_widths[junction_idx], junction_idx, apply_hole_compensation));
'''

# ---------------------------------------------------------------------------
# LINEPROBE2 (R573) - per-ASSEMBLED-LINE width variety, measured at the end of
# generateToolpaths. The beading is out of scope at every assembly point
# (addToolpathSegment receives only junctions), so this is the earliest per-loop
# measurement possible without tagging ExtrusionJunction. Mirrors the Rust probe
# of the same name in arachne/skeletal_trapezoidation.rs.
# ---------------------------------------------------------------------------
ST_LP2_OLD = '''    generateSegments();
'''
ST_LP2_NEW = '''    generateSegments();

    if (getenv("LINEPROBE2") && !probe_speculative()) {
        static std::mutex lp2_mtx;
        static size_t lp2_lines = 0, lp2_juncs = 0, lp2_distinct = 0, lp2_flat = 0, lp2_changes = 0;
        static size_t lp2_ol = 0, lp2_oj = 0, lp2_od = 0, lp2_of = 0, lp2_oc = 0;
        // R584: decompose the outer-wall width changes by MECHANISM. Within one
        // graph edge every junction draws from a single beading (edge->to's) with
        // junction_idx descending, so a change between consecutive junctions is
        // either an index step or the SAME index resolving to a different beading
        // on the next edge. R583 put the 1.378x change-density gap at birth here;
        // this says which of the two mechanisms supplies it.
        static size_t lp2_ch_idx = 0, lp2_ch_bead = 0, lp2_idx_same_w = 0, lp2_pairs = 0;
        std::lock_guard<std::mutex> lp2_lock(lp2_mtx);
        for (size_t inset = 0; inset < p_generated_toolpaths->size(); ++inset) {
            for (const ExtrusionLine &line : (*p_generated_toolpaths)[inset]) {
                const size_t n = line.junctions.size();
                if (n == 0) continue;
                std::vector<coord_t> w;
                w.reserve(n);
                for (const ExtrusionJunction &j : line.junctions) w.push_back(j.w);
                size_t changes = 0;
                for (size_t k = 1; k < w.size(); ++k) if (w[k] != w[k - 1]) ++changes;
                std::sort(w.begin(), w.end());
                w.erase(std::unique(w.begin(), w.end()), w.end());
                const size_t d = w.size();
                ++lp2_lines; lp2_juncs += n; lp2_distinct += d; lp2_changes += changes;
                if (d == 1) ++lp2_flat;
                if (inset == 0) {
                    ++lp2_ol; lp2_oj += n; lp2_od += d; lp2_oc += changes;
                    if (d == 1) ++lp2_of;
                    for (size_t k = 1; k < n; ++k) {
                        const ExtrusionJunction &a = line.junctions[k - 1];
                        const ExtrusionJunction &b = line.junctions[k];
                        ++lp2_pairs;
                        if (b.w != a.w) {
                            if (b.perimeter_index != a.perimeter_index) ++lp2_ch_idx;
                            else                                        ++lp2_ch_bead;
                        } else if (b.perimeter_index != a.perimeter_index) {
                            ++lp2_idx_same_w;
                        }
                    }
                }
            }
        }
        if (lp2_lines > 0)
            fprintf(stderr,
                "[LINEPROBE2] lines=%zu juncs=%zu distinct=%zu flat=%zu changes=%zu | OUTER lines=%zu juncs=%zu distinct=%zu flat=%zu changes=%zu"
                " | MECH pairs=%zu ch_idx=%zu ch_bead=%zu idx_same_w=%zu\\n",
                lp2_lines, lp2_juncs, lp2_distinct, lp2_flat, lp2_changes,
                lp2_ol, lp2_oj, lp2_od, lp2_of, lp2_oc,
                lp2_pairs, lp2_ch_idx, lp2_ch_bead, lp2_idx_same_w);
    }
'''

# ---------------------------------------------------------------------------
# SPLITPROBE (R575) - how many pieces does ONE assembled ExtrusionLine become at
# the overhang/ZPath split? R574 measured 6.37 builder calls per assembled outer
# line vs C++'s 3.39 (1.88x) aggregated over every stage; this attributes the
# pieces to the two branches at this site. Mirrors the Rust probe of the same
# name in perimeter_generator.rs. Outer wall only (inset_idx == 0).
# ---------------------------------------------------------------------------
PG_SPLIT_OLD = '''            ZPaths path_overhang = clip_extrusion(subject_path, clip_paths, ClipperLib_Z::ctDifference);
'''
PG_SPLIT_NEW = '''            const size_t sp_supported = paths.size();
            ZPaths path_overhang = clip_extrusion(subject_path, clip_paths, ClipperLib_Z::ctDifference);
'''

PG_SPLIT2_OLD = '''            // Reapply the nearest point search for starting point.
            // We allow polyline reversal because Clipper may have randomly reversed polylines during clipping.
            // Arachne sometimes creates extrusion with zero-length (just two same endpoints);
            if (!paths.empty()) {
'''
PG_SPLIT2_NEW = '''            if (getenv("SPLITPROBE") && extrusion->inset_idx == 0) {
                static std::mutex sp_mtx;
                static size_t sp_lines = 0, sp_pieces = 0, sp_supp = 0, sp_over = 0, sp_juncs = 0;
                std::lock_guard<std::mutex> sp_lock(sp_mtx);
                ++sp_lines;
                sp_pieces += paths.size();
                sp_supp += sp_supported;
                sp_over += (paths.size() >= sp_supported ? paths.size() - sp_supported : 0);
                sp_juncs += extrusion->junctions.size();
                if (sp_lines % 2000 == 0)
                    fprintf(stderr,
                        "[SPLITPROBE] lines=%zu juncs=%zu pieces=%zu supported=%zu overhang=%zu\\n",
                        sp_lines, sp_juncs, sp_pieces, sp_supp, sp_over);
            }
            // Reapply the nearest point search for starting point.
            // We allow polyline reversal because Clipper may have randomly reversed polylines during clipping.
            // Arachne sometimes creates extrusion with zero-length (just two same endpoints);
            if (!paths.empty()) {
'''

# ---------------------------------------------------------------------------
# NEWLINEPROBE (R576) - which condition starts each new ExtrusionLine? C++
# assembles 1.96x more outer-wall lines than Rust (R574); this attributes every
# new-line decision to its cause. Outer wall only (inset_idx == 0). Mirrors the
# Rust probe of the same name in arachne/skeletal_trapezoidation.rs.
# ---------------------------------------------------------------------------
ST_NL_OLD = '''    if (generated_toolpaths[inset_idx].empty()
        || generated_toolpaths[inset_idx].back().is_odd != is_odd
        || generated_toolpaths[inset_idx].back().junctions.back().perimeter_index != inset_idx // inset_idx should always be consistent
    )
'''
ST_NL_NEW = '''    const bool nlp = getenv("NEWLINEPROBE") && inset_idx == 0 && !probe_speculative();
    const bool nlp_caller = force_new_path;
    const bool nlp_empty = generated_toolpaths[inset_idx].empty();
    const bool nlp_odd = !nlp_empty && generated_toolpaths[inset_idx].back().is_odd != is_odd;
    const bool nlp_perim = !nlp_empty && !nlp_odd &&
        generated_toolpaths[inset_idx].back().junctions.back().perimeter_index != inset_idx;

    if (generated_toolpaths[inset_idx].empty()
        || generated_toolpaths[inset_idx].back().is_odd != is_odd
        || generated_toolpaths[inset_idx].back().junctions.back().perimeter_index != inset_idx // inset_idx should always be consistent
    )
'''

ST_NL2_OLD = '''    else
    {
        generated_toolpaths[inset_idx].emplace_back(inset_idx, is_odd);
        generated_toolpaths[inset_idx].back().junctions.push_back(from);
        generated_toolpaths[inset_idx].back().junctions.push_back(to);
    }
'''
ST_NL2_NEW = '''    else
    {
        if (nlp) {
            static std::mutex nl_mtx;
            static size_t nl_n = 0, nl_empty = 0, nl_odd = 0, nl_perim = 0,
                          nl_caller = 0, nl_gap = 0, nl_width = 0, nl_3way = 0;
            std::lock_guard<std::mutex> nl_lock(nl_mtx);
            if (nlp_empty) ++nl_empty;
            else if (nlp_odd) ++nl_odd;
            else if (nlp_perim) ++nl_perim;
            else if (nlp_caller) ++nl_caller;
            else {
                const ExtrusionJunction &last = generated_toolpaths[inset_idx].back().junctions.back();
                const bool gap_ok = shorter_then(last.p - from.p, scaled<coord_t>(0.010));
                const bool w_ok = std::abs(last.w - from.w) < scaled<coord_t>(0.010);
                if (!gap_ok) ++nl_gap;
                else if (!w_ok) ++nl_width;
                else ++nl_3way;
            }
            ++nl_n;
            if (nl_n % 2000 == 0)
                fprintf(stderr,
                    "[NEWLINEPROBE] newlines=%zu empty=%zu odd=%zu perim=%zu caller=%zu gap=%zu width=%zu threeway=%zu\\n",
                    nl_n, nl_empty, nl_odd, nl_perim, nl_caller, nl_gap, nl_width, nl_3way);
        }
        generated_toolpaths[inset_idx].emplace_back(inset_idx, is_odd);
        generated_toolpaths[inset_idx].back().junctions.push_back(from);
        generated_toolpaths[inset_idx].back().junctions.push_back(to);
    }
'''

# ---------------------------------------------------------------------------
# ODDPROBE (R577) - the `odd` new-line cause is 3.21x (R576, 41.5% of factor 1).
# Counts EVERY segment reaching addToolpathSegment at inset 0, split by is_odd,
# plus alternations in the call sequence. Distinguishes "C++ generates more odd
# walls" (share differs) from "C++ interleaves them differently" (share matches,
# alternations differ). Mirrors the Rust probe in skeletal_trapezoidation.rs.
# ---------------------------------------------------------------------------
ST_ODD_OLD = '''    const bool nlp = getenv("NEWLINEPROBE") && inset_idx == 0 && !probe_speculative();
'''
ST_ODD_NEW = '''    if (getenv("ODDPROBE") && inset_idx == 0 && !probe_speculative()) {
        static std::mutex od_mtx;
        static size_t od_calls = 0, od_odd = 0, od_alt = 0;
        static int od_prev = 2;
        std::lock_guard<std::mutex> od_lock(od_mtx);
        const int od_cur = is_odd ? 1 : 0;
        if (od_prev != od_cur) ++od_alt;
        od_prev = od_cur;
        if (is_odd) ++od_odd;
        ++od_calls;
        if (od_calls % 5000 == 0)
            fprintf(stderr, "[ODDPROBE] segments=%zu odd=%zu even=%zu alternations=%zu\\n",
                    od_calls, od_odd, od_calls - od_odd, od_alt);
    }

    const bool nlp = getenv("NEWLINEPROBE") && inset_idx == 0 && !probe_speculative();
'''

# ---------------------------------------------------------------------------
# WTPCALL (R580) - every WallToolPaths construction is one generateToolpaths
# invocation. C++ makes 1.592x more of them (41,188 vs 25,876, R577), the larger
# of the two upstream terms feeding the 3.23x tag chain. Buckets by layer to see
# whether the gap is uniform or concentrated in a band. Mirrors the Rust probe of
# the same name in perimeter_generator.rs.
# ---------------------------------------------------------------------------
PG_SPEC_OLD = '''            if (seperate_wall_generation) {
                Arachne::WallToolPaths one_wall_paths(last_p, ext_perimeter_spacing, perimeter_spacing, 1, wall_0_inset, layer_height, input_params);
'''
PG_SPEC_NEW = '''            if (seperate_wall_generation) {
                probe_speculative() = true;   // R581
                Arachne::WallToolPaths one_wall_paths(last_p, ext_perimeter_spacing, perimeter_spacing, 1, wall_0_inset, layer_height, input_params);
'''

PG_SPEC2_OLD = '''                infill_contour_by_one_wall = union_ex(one_wall_paths.getInnerContour());
'''
PG_SPEC2_NEW = '''                infill_contour_by_one_wall = union_ex(one_wall_paths.getInnerContour());
                probe_speculative() = false;  // R581
'''

PG_WTP_OLD = '''            coord_t wall_0_inset = 0;
            if (apply_precise_outer_wall)
                wall_0_inset = -coord_t(ext_perimeter_width / 2 - ext_perimeter_spacing / 2);
'''
PG_WTP_NEW = '''            coord_t wall_0_inset = 0;
            if (apply_precise_outer_wall)
                wall_0_inset = -coord_t(ext_perimeter_width / 2 - ext_perimeter_spacing / 2);

            if (getenv("WTPCALL")) {
                static std::mutex wtp_mtx;
                static size_t wtp_calls = 0, wtp_onewall = 0, wtp_polys = 0;
                static std::vector<unsigned> wtp_per_layer;
                std::lock_guard<std::mutex> wtp_lock(wtp_mtx);
                ++wtp_calls;
                if (seperate_wall_generation) ++wtp_onewall;
                wtp_polys += last_p.size();
                const int li = this->layer_id < 0 ? 0 : this->layer_id;
                if ((int)wtp_per_layer.size() <= li) wtp_per_layer.resize(li + 1, 0);
                ++wtp_per_layer[li];
                if (wtp_calls % 2000 == 0) {
                    std::vector<unsigned> nz;
                    for (unsigned v : wtp_per_layer) if (v > 0) nz.push_back(v);
                    std::sort(nz.begin(), nz.end());
                    const unsigned med = nz.empty() ? 0 : nz[nz.size() / 2];
                    const unsigned mx = nz.empty() ? 0 : nz.back();
                    fprintf(stderr,
                        "[WTPCALL] calls=%zu onewall=%zu polys=%zu layers_touched=%zu per_layer_median=%u max=%u\\n",
                        wtp_calls, wtp_onewall, wtp_polys, nz.size(), med, mx);
                }
            }
'''

# ---------------------------------------------------------------------------
# REDPROBE (R583) - width changes ENTERING the ZPath stage. R583 (Rust) found the
# clip does NOT destroy changes (ch_out/ch_in = 1.11), but the subject arriving at
# the clip carries only 0.82 changes/line against 4.72 at assembly -- so the loss
# is UPSTREAM of the ZPath, in WallToolPaths post-processing. This counts the same
# input quantity on the C++ side. Outer wall only (inset_idx == 0).
# ---------------------------------------------------------------------------
PG_RED_OLD = '''            ZPaths clip_paths;
'''
PG_RED_NEW = '''            if (getenv("REDPROBE") && extrusion->inset_idx == 0) {
                static std::mutex rd_mtx;
                static size_t rd_lines = 0, rd_pts = 0, rd_ch = 0;
                std::lock_guard<std::mutex> rd_lock(rd_mtx);
                ++rd_lines;
                rd_pts += subject_path.size();
                for (size_t k = 1; k < subject_path.size(); ++k)
                    if (subject_path[k].z() != subject_path[k - 1].z()) ++rd_ch;
                if (rd_lines % 2000 == 0)
                    fprintf(stderr, "[REDPROBE] lines=%zu pts_in=%zu ch_in=%zu (%.4f/line)\\n",
                            rd_lines, rd_pts, rd_ch, (double)rd_ch / (double)rd_lines);
            }

            ZPaths clip_paths;
'''

# ---------------------------------------------------------------------------
# R584: count the graph edges that actually emit junctions, so JUNCPROBE can
# report junctions-per-edge. Every outer-wall width change happens at an edge
# boundary (LINEPROBE2 MECH: ch_idx=0), so this is the denominator of the
# change-density gap.
# ---------------------------------------------------------------------------
# NOTE: the bare emplace_back line occurs 3x in this file -- anchor on the
# getOrCreateBeading line above it, which is unique to generateJunctions.
ST_EDGE_OLD = '''        Beading* beading = &getOrCreateBeading(edge->to, node_beadings)->beading;
        edge_junctions.emplace_back(std::make_shared<LineJunctions>());
'''
ST_EDGE_NEW = '''        Beading* beading = &getOrCreateBeading(edge->to, node_beadings)->beading;
        if (getenv("BEADPROBE") && !probe_speculative()) g_ep_edges.fetch_add(1);

        // BEADPAIR (R585): P(adjacent beadings differ in bead_widths[0]).
        // R584 showed junctions-per-edge is at parity (1.022x) while the outer-wall
        // change density is 1.378x, and that every change is one bead index
        // resolving to a different beading at an edge boundary -- so the whole gap
        // must live in this probability. A per-edge Bernoulli rate is ORDER-
        // INDEPENDENT, unlike the prefix distinct-count R584 had to retract.
        // Read-only: hasBeading()/getBeading() never create, so this cannot perturb
        // the run (calling getOrCreateBeading on `from` would).
        if (getenv("BEADPAIR") && !probe_speculative()) {
            static std::mutex bp_mtx;
            // bp_same: the two endpoints resolve to the SAME BeadingPropagation
            // object, so their widths are identical by construction rather than by
            // computation. If we share beadings across neighbouring nodes more than
            // C++ does, that alone depresses P(differ).
            static size_t bp_n = 0, bp_both = 0, bp_diff = 0, bp_tdiff = 0, bp_same = 0;
            static size_t bp_d1 = 0, bp_d10 = 0, bp_d100 = 0, bp_dbig = 0;
            static size_t bp_mod[100] = {0};
            std::lock_guard<std::mutex> bp_lock(bp_mtx);
            ++bp_n;
            const coord_t w_to = beading->bead_widths.empty() ? -1 : beading->bead_widths[0];
            if (w_to >= 0) ++bp_mod[w_to % 100];
            if (w_to >= 0 && edge->from->data.hasBeading()) {
                auto fb = edge->from->data.getBeading();
                if (fb && !fb->beading.bead_widths.empty()) {
                    ++bp_both;
                    if (&fb->beading == beading) ++bp_same;
                    const coord_t w_from = fb->beading.bead_widths[0];
                    const coord_t d = std::abs(w_to - w_from);
                    if (d != 0) ++bp_diff;
                    if (fb->beading.total_thickness != beading->total_thickness) ++bp_tdiff;
                    // coord_t is 1e-5 mm, so 100 units == 1 micron.
                    if      (d == 0)     {}
                    else if (d < 100)    ++bp_d1;
                    else if (d < 1000)   ++bp_d10;
                    else if (d < 10000)  ++bp_d100;
                    else                 ++bp_dbig;
                }
            }
            if (bp_n % 500000 == 0) {
                size_t nz = 0;
                for (int k = 0; k < 100; ++k) if (bp_mod[k]) ++nz;
                fprintf(stderr,
                    "[BEADPAIR] edges=%zu both=%zu differ=%zu (%.4f) tdiff=%zu (%.4f) same_obj=%zu (%.4f) | "
                    "d<1um=%zu 1-10um=%zu 10-100um=%zu >100um=%zu | w0_mod100_nonzero=%zu/100\\n",
                    bp_n, bp_both, bp_diff, double(bp_diff) / double(std::max<size_t>(bp_both, 1)),
                    bp_tdiff, double(bp_tdiff) / double(std::max<size_t>(bp_both, 1)),
                    bp_same, double(bp_same) / double(std::max<size_t>(bp_both, 1)),
                    bp_d1, bp_d10, bp_d100, bp_dbig, nz);
            }
        }

        edge_junctions.emplace_back(std::make_shared<LineJunctions>());
'''

# ---------------------------------------------------------------------------
# PROPCLASS (R586) - classify every beading creation into fresh / copy / interp.
# A copy is bit-identical to its source and cannot produce a width change between
# neighbours; only fresh and interp can. R585 left the propagation chain as the
# only unexamined link behind the 1.85-2.41x P(adjacent beadings differ) gap.
# ---------------------------------------------------------------------------
ST_PC_FRESH_OLD = """        node_beadings.emplace_back(new BeadingPropagation(beading_strategy.compute(node->data.distance_to_boundary * 2, node->data.bead_count)));
"""
ST_PC_FRESH_NEW = """        propclass_tick(0);
        node_beadings.emplace_back(new BeadingPropagation(beading_strategy.compute(node->data.distance_to_boundary * 2, node->data.bead_count)));
"""

ST_PC_COPYNEW_OLD = """        BeadingPropagation propagated_beading = top_beading;
"""
ST_PC_COPYNEW_NEW = """        propclass_tick(1);
        BeadingPropagation propagated_beading = top_beading;
"""

ST_PC_COPYRATIO_OLD = """            bottom_beading = top_beading;
            bottom_beading.dist_from_top_source += length;
"""
ST_PC_COPYRATIO_NEW = """            propclass_tick(2);
            bottom_beading = top_beading;
            bottom_beading.dist_from_top_source += length;
"""

ST_PC_INTERP_OLD = """            Beading merged_beading = interpolate(top_beading.beading, ratio_of_top, bottom_beading.beading, edge_to_peak->from->data.distance_to_boundary);
"""
ST_PC_INTERP_NEW = """            const coord_t pc_w_before = bottom_beading.beading.bead_widths.empty() ? -1 : bottom_beading.beading.bead_widths[0];
            Beading merged_beading = interpolate(top_beading.beading, ratio_of_top, bottom_beading.beading, edge_to_peak->from->data.distance_to_boundary);
            if (::getenv("PROPCLASS") && !probe_speculative() && pc_w_before >= 0 && !merged_beading.bead_widths.empty()) {
                const coord_t pc_d = std::abs(merged_beading.bead_widths[0] - pc_w_before);
                if (pc_d == 0)        g_pc_interp_zero.fetch_add(1);
                else if (pc_d < 100)  g_pc_interp_small.fetch_add(1);
                else                  g_pc_interp_big.fetch_add(1);
            }
            propclass_tick(3);
"""

# ---------------------------------------------------------------------------
# UPPROBE / DNPROBE (R587) - the two candidates for R586's 2.25x gap in how often
# propagateBeadingsDownward finds `from` already seeded: upward SEED coverage, and
# the set of edges the downward dispatcher actually walks.
# ---------------------------------------------------------------------------
ST_UP_OLD = """        edge_t* upward_edge = *upward_quad_mids_it;
"""
ST_UP_NEW = """        edge_t* upward_edge = *upward_quad_mids_it;
        if (getenv("UPPROBE") && !probe_speculative())
            upprobe_tick(upward_edge->to->data.bead_count >= 0,
                         !upward_edge->from->data.hasBeading(),
                         upward_edge->to->data.hasBeading());
"""

ST_DN_OLD = """    for (edge_t* upward_quad_mid : upward_quad_mids)
    {
        // Transfer beading information to lower nodes
"""
ST_DN_NEW = """    for (edge_t* upward_quad_mid : upward_quad_mids)
    {
        if (getenv("UPPROBE") && !probe_speculative()) {
            const bool dn_central = upward_quad_mid->data.isCentral();
            const bool dn_equi = !dn_central
                && upward_quad_mid->from->data.distance_to_boundary == upward_quad_mid->to->data.distance_to_boundary
                && upward_quad_mid->from->data.hasBeading()
                && !upward_quad_mid->to->data.hasBeading();
            dnprobe_tick(dn_central, dn_equi);
        }
        // Transfer beading information to lower nodes
"""

# ---------------------------------------------------------------------------
# CENSUS (R588) - per-NODE `bead_count >= 0` share (the quantity the upward guard
# actually tests) and per-EDGE central share, measured directly on the graph at the
# moment propagateBeadingsUpward runs. R587 left both as the open question.
# ---------------------------------------------------------------------------
ST_CENSUS_OLD = """    for (auto upward_quad_mids_it = upward_quad_mids.rbegin(); upward_quad_mids_it != upward_quad_mids.rend(); ++upward_quad_mids_it)
"""
ST_CENSUS_NEW = """    if (getenv("CENSUS") && !probe_speculative()) {
        size_t cs_nodes = 0, cs_bc = 0, cs_hasb = 0, cs_edges = 0, cs_central = 0;
        for (node_t &nd : graph.nodes) {
            ++cs_nodes;
            if (nd.data.bead_count >= 0) ++cs_bc;
            if (nd.data.hasBeading()) ++cs_hasb;
        }
        for (edge_t &eg : graph.edges) {
            ++cs_edges;
            if (eg.data.isCentral()) ++cs_central;
        }
        census_tick(cs_nodes, cs_bc, cs_hasb, cs_edges, cs_central);
    }

    for (auto upward_quad_mids_it = upward_quad_mids.rbegin(); upward_quad_mids_it != upward_quad_mids.rend(); ++upward_quad_mids_it)
"""

# ---------------------------------------------------------------------------
# GBUILD (R589) - bracket WHERE the 1.25x graph density appears: Voronoi input,
# raw Voronoi output, and discretize() output per Voronoi edge.
# ---------------------------------------------------------------------------
ST_GB_OLD = """    voronoi_diagram.construct_voronoi(segments.cbegin(), segments.cend());
"""
ST_GB_NEW = """    voronoi_diagram.construct_voronoi(segments.cbegin(), segments.cend());
    if (getenv("GBUILD") && !probe_speculative())
        gbuild_tick(polys.size(), segments.size(), voronoi_diagram.num_vertices(),
                    voronoi_diagram.num_edges(), voronoi_diagram.num_cells());
"""

ST_GBD_OLD = """        Points discretized = discretize(vd_edge, segments);
"""
ST_GBD_NEW = """        Points discretized = discretize(vd_edge, segments);
        if (getenv("GBUILD") && !probe_speculative()) gbuild_disc(discretized.size());
"""

# ---------------------------------------------------------------------------
# CONV (R590) - does the 1.25x density come from CREATING fewer half-edges or
# REMOVING more? Edge counts after the cell loop, after separatePointyQuadEndNodes,
# and after collapseSmallEdges, plus cells seen vs skipped.
# ---------------------------------------------------------------------------
ST_CV_CELL_OLD = """    for (const VD::cell_type &cell : voronoi_diagram.cells()) {
        if (!cell.incident_edge())
            continue; // There is no spoon
"""
ST_CV_CELL_NEW = """    for (const VD::cell_type &cell : voronoi_diagram.cells()) {
        if (getenv("CONV") && !probe_speculative()) conv_cell(!cell.incident_edge());
        if (!cell.incident_edge())
            continue; // There is no spoon
"""

ST_CV_STAGE_OLD = """    separatePointyQuadEndNodes();

    graph.collapseSmallEdges();
"""
ST_CV_STAGE_NEW = """    if (getenv("CONV") && !probe_speculative()) conv_stage(0, graph.edges.size(), graph.nodes.size());

    separatePointyQuadEndNodes();
    if (getenv("CONV") && !probe_speculative()) conv_stage(1, graph.edges.size(), graph.nodes.size());

    graph.collapseSmallEdges();
    if (getenv("CONV") && !probe_speculative()) conv_stage(2, graph.edges.size(), graph.nodes.size());
"""

# ---------------------------------------------------------------------------
# BRIDGEPROBE (R595) - R594 found the bridge FILL DIRECTION differs per layer
# (L47 rust 45 vs cpp 135, 90 apart). Two candidate causes: the candidate SET /
# coverages differ (upstream: _anchor_regions / expolygons), or they agree and only
# the tie-break differs (C++ std::sort is NOT stable; the port's sort_by IS).
# Dump every candidate so the divergence point is visible rather than argued.
# ---------------------------------------------------------------------------
BD_OLD = """    this->angle = candidates[i_best].angle;
"""
BD_NEW = """    if (getenv("BRIDGEPROBE")) {
        static std::mutex bd_mtx;
        static size_t bd_call = 0;
        std::lock_guard<std::mutex> bd_lock(bd_mtx);
        ++bd_call;
        if (bd_call <= 40) {
            fprintf(stderr, "[BRIDGEPROBE] call=%zu ncand=%zu spacing=%d i_best=%zu\\n",
                    bd_call, candidates.size(), (int)this->spacing, i_best);
            for (size_t bi = 0; bi < candidates.size() && bi < 12; ++bi)
                fprintf(stderr,
                    "[BRIDGECAND] call=%zu i=%zu angle=%.6f coverage=%.3f max_length=%.3f anchored=%.6f\\n",
                    bd_call, bi, candidates[bi].angle, candidates[bi].coverage,
                    candidates[bi].max_length, candidates[bi].archored_percent);
        }
    }
    this->angle = candidates[i_best].angle;
"""

BD_INC_OLD = """#include "BridgeDetector.hpp"
"""
BD_INC_NEW = """#include "BridgeDetector.hpp"
#include <mutex>
#include <cstdio>
#include <cstdlib>
"""

# ---------------------------------------------------------------------------
# BRIDGEIN (R596) - R595 verified the ARITHMETIC of detect_bridging_direction and
# eliminated BridgeDetector::detect_angle (0 calls on both engines). What remains
# is its INPUT. Dump, at the call site: the floating-edge set (count + total
# length), the bridge expolygon size, and the resulting direction/angle. If the
# inputs already differ, the cause is upstream in the anchors/expansion, not here.
# ---------------------------------------------------------------------------
LR_BIN_OLD = """        auto [bridging_dir, unsupported_dist] = detect_bridging_direction(lines, to_polygons(bridge.expolygon));
        bridge.angle = M_PI + std::atan2(bridging_dir.y(), bridging_dir.x());
"""
LR_BIN_NEW = """        auto [bridging_dir, unsupported_dist] = detect_bridging_direction(lines, to_polygons(bridge.expolygon));
        bridge.angle = M_PI + std::atan2(bridging_dir.y(), bridging_dir.x());
        if (getenv("BRIDGEIN")) {
            static std::mutex bi_mtx;
            static size_t bi_n = 0;
            std::lock_guard<std::mutex> bi_lock(bi_mtx);
            ++bi_n;
            if (bi_n <= 30) {
                double bi_len = 0.0;
                for (const Line &bl : lines) bi_len += bl.length();
                size_t bi_pts = 0;
                for (const Polygon &bp : to_polygons(bridge.expolygon)) bi_pts += bp.points.size();
                fprintf(stderr,
                    "[BRIDGEIN] n=%zu edges=%zu edge_len=%.3f anchors=%zu poly_pts=%zu "
                    "area=%.3f dir=%.6f,%.6f angle=%.6f unsup=%.3f\\n",
                    bi_n, lines.size(), bi_len, anchor_areas.size(), bi_pts,
                    std::abs(bridge.expolygon.area()), bridging_dir.x(), bridging_dir.y(),
                    *bridge.angle, unsupported_dist);
            }
        }
"""

# ---------------------------------------------------------------------------
# FILLANG (R597) - the CONSUMER of the bridge angle. R595/R596 both fixed producer
# code that turned out not to reach Benchy's output. This probes where the angle is
# actually READ: Fill::_infill_direction, FillBase.cpp:224. The branch hinges on
# `surface->bridge_angle >= 0`; if Benchy's bridge surfaces arrive with it unset,
# the direction comes from the alternating layer angle instead and the producer is
# irrelevant for that fixture.
# ---------------------------------------------------------------------------
FB_INC_OLD = """#include "FillBase.hpp"
"""
FB_INC_NEW = """#include "FillBase.hpp"
#include <mutex>
#include <cstdio>
#include <cstdlib>
"""

FB_OLD = """    out_angle += float(M_PI/2.);
    return std::pair<float, Point>(out_angle, out_shift);
"""
FB_NEW = """    out_angle += float(M_PI/2.);
    if (getenv("FILLANG")) {
        static std::mutex fa_mtx;
        static size_t fa_n = 0, fa_bridge = 0, fa_layer = 0, fa_thick = 0;
        std::lock_guard<std::mutex> fa_lock(fa_mtx);
        ++fa_n;
        if (surface->bridge_angle >= 0) ++fa_bridge; else ++fa_layer;
        if (surface->thickness_layers > 1) ++fa_thick;
        if (fa_n <= 5 || fa_n % 500 == 0)
            fprintf(stderr,
                "[FILLANG] n=%zu used_bridge=%zu used_layer=%zu thick_gt1=%zu | "
                "surf_type=%d thickness_layers=%d bridge_angle=%.6f out_angle=%.6f\\n",
                fa_n, fa_bridge, fa_layer, fa_thick, (int)surface->surface_type,
                (int)surface->thickness_layers, surface->bridge_angle, out_angle);
    }
    return std::pair<float, Point>(out_angle, out_shift);
"""

EDITS = [
    ("Fill/FillBase.cpp", FB_INC_OLD, FB_INC_NEW),
    ("Fill/FillBase.cpp", FB_OLD, FB_NEW),
    ("LayerRegion.cpp", LR_BIN_OLD, LR_BIN_NEW),
    ("BridgeDetector.cpp", BD_INC_OLD, BD_INC_NEW),
    ("BridgeDetector.cpp", BD_OLD, BD_NEW),
    ("SkeletalTrapezoidation.cpp", ST_INCLUDES_OLD, ST_INCLUDES_NEW),
    ("SkeletalTrapezoidation.cpp", ST_EDGE_OLD, ST_EDGE_NEW),
    ("SkeletalTrapezoidation.cpp", ST_CENSUS_OLD, ST_CENSUS_NEW),
    ("SkeletalTrapezoidation.cpp", ST_GB_OLD, ST_GB_NEW),
    ("SkeletalTrapezoidation.cpp", ST_CV_CELL_OLD, ST_CV_CELL_NEW),
    ("SkeletalTrapezoidation.cpp", ST_CV_STAGE_OLD, ST_CV_STAGE_NEW),
    ("SkeletalTrapezoidation.cpp", ST_GBD_OLD, ST_GBD_NEW),
    ("SkeletalTrapezoidation.cpp", ST_UP_OLD, ST_UP_NEW),
    ("SkeletalTrapezoidation.cpp", ST_DN_OLD, ST_DN_NEW),
    ("SkeletalTrapezoidation.cpp", ST_PC_FRESH_OLD, ST_PC_FRESH_NEW),
    ("SkeletalTrapezoidation.cpp", ST_PC_COPYNEW_OLD, ST_PC_COPYNEW_NEW),
    ("SkeletalTrapezoidation.cpp", ST_PC_COPYRATIO_OLD, ST_PC_COPYRATIO_NEW),
    ("SkeletalTrapezoidation.cpp", ST_PC_INTERP_OLD, ST_PC_INTERP_NEW),
    ("SkeletalTrapezoidation.cpp", ST_PROBES_OLD, ST_PROBES_NEW),
    ("SkeletalTrapezoidation.cpp", ST_COMPUTE1_OLD, ST_COMPUTE1_NEW),
    ("SkeletalTrapezoidation.cpp", ST_COMPUTE2_OLD, ST_COMPUTE2_NEW),
    ("SkeletalTrapezoidation.cpp", ST_PROP_OLD, ST_PROP_NEW),
    ("SkeletalTrapezoidation.cpp", ST_GRAPH_OLD, ST_GRAPH_NEW),
    ("SkeletalTrapezoidation.cpp", ST_TRANS_CALLS_OLD, ST_TRANS_CALLS_NEW),
    ("SkeletalTrapezoidation.cpp", ST_TRANS_CALLS2_OLD, ST_TRANS_CALLS2_NEW),
    ("SkeletalTrapezoidation.cpp", ST_TRANS_CALLS3_OLD, ST_TRANS_CALLS3_NEW),
    ("SkeletalTrapezoidation.cpp", ST_TRANS_CALLS4_OLD, ST_TRANS_CALLS4_NEW),
    ("SkeletalTrapezoidation.hpp", ST_TRANS_HPP_OLD, ST_TRANS_HPP_NEW),
    ("SkeletalTrapezoidation.cpp", ST_ISC_BRANCHES_OLD, ST_ISC_BRANCHES_NEW),
    ("SkeletalTrapezoidation.cpp", ST_ISC_GEOM_OLD, ST_ISC_GEOM_NEW),
    ("SkeletalTrapezoidation.cpp", ST_CENTRAL_CALLS_OLD, ST_CENTRAL_CALLS_NEW),
    ("SkeletalTrapezoidation.cpp", ST_CENTRAL_CALLS2_OLD, ST_CENTRAL_CALLS2_NEW),
    ("SkeletalTrapezoidation.cpp", ST_CENTRAL_CALLS3_OLD, ST_CENTRAL_CALLS3_NEW),
    ("SkeletalTrapezoidation.cpp", ST_CENTRAL_CALLS4_OLD, ST_CENTRAL_CALLS4_NEW),
    ("SkeletalTrapezoidation.cpp", ST_CENTRAL_CALLS5_OLD, ST_CENTRAL_CALLS5_NEW),
    ("SkeletalTrapezoidation.hpp", ST_HPP_OLD, ST_HPP_NEW),
    ("LayerRegion.cpp", LR_INCLUDES_OLD, LR_INCLUDES_NEW),
    ("LayerRegion.cpp", LR_MP_CALL_OLD, LR_MP_CALL_NEW),
    ("PerimeterGenerator.cpp", PG_INCLUDES_OLD, PG_INCLUDES_NEW),
    ("PerimeterGenerator.cpp", PG_LAST_CALLS_OLD, PG_LAST_CALLS_NEW),
    ("PerimeterGenerator.cpp", PG_LAST_CALLS2_OLD, PG_LAST_CALLS2_NEW),
    ("PerimeterGenerator.cpp", PG_JW_CALLS_OLD, PG_JW_CALLS_NEW),
    ("PerimeterGenerator.cpp", PG_TOW_CALLS_OLD, PG_TOW_CALLS_NEW),
    ("PerimeterGenerator.cpp", PG_TOW_CALLS2_OLD, PG_TOW_CALLS2_NEW),
    ("WallToolPaths.cpp", WTP_INCLUDES_OLD, WTP_INCLUDES_NEW),
    ("WallToolPaths.cpp", WTP_PROBE_OLD, WTP_PROBE_NEW),
    ("WallToolPaths.cpp", WTP_CHAIN_OLD, WTP_CHAIN_NEW),
    ("WallToolPaths.cpp", WTP_POLY_CALLS_OLD, WTP_POLY_CALLS_NEW),
    ("WallToolPaths.cpp", WTP_POLY_CALLS2_OLD, WTP_POLY_CALLS2_NEW),
    ("WallToolPaths.cpp", WTP_POLY_CALLS3_OLD, WTP_POLY_CALLS3_NEW),
    # R562 AREAPROBE — MUST come after the POLY edits above: these anchors match
    # the text as it exists once the polyprobe calls have been inserted.
    ("WallToolPaths.cpp", WTP_AREA_CALLS_OLD, WTP_AREA_CALLS_NEW),
    ("WallToolPaths.cpp", WTP_AREA_CALLS2_OLD, WTP_AREA_CALLS2_NEW),
    ("WallToolPaths.cpp", WTP_AREA_CALLS3_OLD, WTP_AREA_CALLS3_NEW),
    ("WallToolPaths.cpp", WTP_AREA_CALLS4_OLD, WTP_AREA_CALLS4_NEW),
    ("WallToolPaths.cpp", WTP_PARAMS_OLD, WTP_PARAMS_NEW),
    ("VariableWidth.cpp", VW_INCLUDES_OLD, VW_INCLUDES_NEW),
    ("VariableWidth.cpp", VW_HEAD_OLD, VW_HEAD_NEW),
    ("VariableWidth.cpp", VW_TAIL_OLD, VW_TAIL_NEW),
    ("SkeletalTrapezoidation.cpp", ST_JUNC_OLD, ST_JUNC_NEW),
    ("SkeletalTrapezoidation.cpp", ST_LP2_OLD, ST_LP2_NEW),
    ("PerimeterGenerator.cpp", PG_SPLIT_OLD, PG_SPLIT_NEW),
    ("PerimeterGenerator.cpp", PG_SPLIT2_OLD, PG_SPLIT2_NEW),
    ("SkeletalTrapezoidation.cpp", ST_NL_OLD, ST_NL_NEW),
    ("SkeletalTrapezoidation.cpp", ST_NL2_OLD, ST_NL2_NEW),
    ("SkeletalTrapezoidation.cpp", ST_ODD_OLD, ST_ODD_NEW),
    ("PerimeterGenerator.cpp", PG_WTP_OLD, PG_WTP_NEW),
    ("PerimeterGenerator.cpp", PG_SPEC_OLD, PG_SPEC_NEW),
    ("PerimeterGenerator.cpp", PG_SPEC2_OLD, PG_SPEC2_NEW),
    ("PerimeterGenerator.cpp", PG_RED_OLD, PG_RED_NEW),
]


def main():
    check_only = "--check" in sys.argv
    # centralCensus's body is appended to the .cpp separately (it needs the class
    # to be fully declared, so it goes after the include block rewrite).
    texts = {}
    for fname in ("SkeletalTrapezoidation.cpp", "SkeletalTrapezoidation.hpp", "WallToolPaths.cpp"):
        texts[fname] = open(os.path.join(ARACHNE, fname)).read()
    for fname in LIBSLIC3R_FILES:
        texts[fname] = open(os.path.join(LIBSLIC3R, fname)).read()

    failures = []
    for fname, old, new in EDITS:
        s = texts[fname]
        if new in s:
            continue  # already applied
        n = s.count(old)
        if n != 1:
            failures.append(f"{fname}: anchor matched {n} times (need exactly 1): {old.splitlines()[0][:70]!r}")
            continue
        texts[fname] = s.replace(old, new)

    # centralCensus definition, appended just before generateSegments.
    st = texts["SkeletalTrapezoidation.cpp"]
    if "SkeletalTrapezoidation::centralCensus" not in st:
        anchor = "void SkeletalTrapezoidation::generateSegments()"
        if st.count(anchor) != 1:
            failures.append("SkeletalTrapezoidation.cpp: generateSegments anchor not unique")
        else:
            texts["SkeletalTrapezoidation.cpp"] = st.replace(anchor, ST_CENTRAL_FN + "\n" + anchor)

    lr = texts["LayerRegion.cpp"]
    if "static void mpprobe" not in lr:
        if lr.count(LR_MP_ANCHOR) != 1:
            failures.append("LayerRegion.cpp: make_perimeters anchor not unique")
        else:
            texts["LayerRegion.cpp"] = lr.replace(LR_MP_ANCHOR, LR_MP_FN + "\n" + LR_MP_ANCHOR, 1)

    pg = texts["PerimeterGenerator.cpp"]
    if "static void lastprobe" not in pg:
        if pg.count(PG_LAST_ANCHOR) != 1:
            failures.append("PerimeterGenerator.cpp: process_arachne anchor not unique")
        else:
            texts["PerimeterGenerator.cpp"] = pg.replace(PG_LAST_ANCHOR, PG_LAST_FN + "\n" + PG_LAST_ANCHOR, 1)

    pg2 = texts["PerimeterGenerator.cpp"]
    if "static void jwprobe" not in pg2:
        if pg2.count(PG_JW_ANCHOR) != 1:
            failures.append("PerimeterGenerator.cpp: jwprobe process_arachne anchor not unique")
        else:
            texts["PerimeterGenerator.cpp"] = pg2.replace(PG_JW_ANCHOR, PG_JW_FN + "\n" + PG_JW_ANCHOR, 1)

    pg3 = texts["PerimeterGenerator.cpp"]
    if "static void towprobe" not in pg3:
        if pg3.count(PG_TOW_ANCHOR) != 1:
            failures.append("PerimeterGenerator.cpp: towprobe process_arachne anchor not unique")
        else:
            texts["PerimeterGenerator.cpp"] = pg3.replace(PG_TOW_ANCHOR, PG_TOW_FN + "\n" + PG_TOW_ANCHOR, 1)

    wt = texts["WallToolPaths.cpp"]
    if "static void polyprobe" not in wt:
        if wt.count(WTP_POLY_ANCHOR) != 1:
            failures.append("WallToolPaths.cpp: stageprobe anchor not unique")
        else:
            texts["WallToolPaths.cpp"] = wt.replace(WTP_POLY_ANCHOR, WTP_POLY_FN + "\n" + WTP_POLY_ANCHOR, 1)

    wt2 = texts["WallToolPaths.cpp"]
    if "static void areaprobe" not in wt2:
        if wt2.count(WTP_POLY_ANCHOR) != 1:
            failures.append("WallToolPaths.cpp: areaprobe stageprobe anchor not unique")
        else:
            texts["WallToolPaths.cpp"] = wt2.replace(WTP_POLY_ANCHOR, WTP_AREA_FN + "\n" + WTP_POLY_ANCHOR, 1)

    st3 = texts["SkeletalTrapezoidation.cpp"]
    if "SkeletalTrapezoidation::transitionCensus" not in st3:
        if st3.count(ST_TRANS_ANCHOR) != 1:
            failures.append("SkeletalTrapezoidation.cpp: generateTransitioningRibs anchor not unique")
        else:
            texts["SkeletalTrapezoidation.cpp"] = st3.replace(ST_TRANS_ANCHOR, ST_TRANS_FN, 1)

    st2 = texts["SkeletalTrapezoidation.cpp"]
    if "iscprobe(int branch" not in st2:
        if st2.count(ST_ISC_ANCHOR) != 1:
            failures.append("SkeletalTrapezoidation.cpp: updateIsCentral anchor not unique")
        else:
            texts["SkeletalTrapezoidation.cpp"] = st2.replace(ST_ISC_ANCHOR, ST_ISC_FN, 1)

    if failures:
        print("FAILED — no files written:")
        for f in failures:
            print("  " + f)
        return 1

    if check_only:
        print("OK — all anchors resolve; nothing written (--check)")
        return 0

    for fname, text in texts.items():
        base = LIBSLIC3R if fname in LIBSLIC3R_FILES else ARACHNE
        open(os.path.join(base, fname), "w").write(text)
    print("Injected probes into: " + ", ".join(sorted(texts)))
    print("Revert with: cd libslic3r/bambustudio/references/BambuStudio && "
          "git checkout -- src/libslic3r/Arachne src/libslic3r/PerimeterGenerator.cpp "
          "src/libslic3r/VariableWidth.cpp "
          "src/libslic3r/LayerRegion.cpp")
    return 0


if __name__ == "__main__":
    sys.exit(main())
