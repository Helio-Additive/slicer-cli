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
LIBSLIC3R_FILES = ("PerimeterGenerator.cpp", "LayerRegion.cpp")

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
#include <boost/log/trivial.hpp>"""

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
    if (probe_enabled("GRAPHPROBE")) {
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

WTP_PROBE_NEW = r"""namespace Slic3r::Arachne
{

// ---------------------------------------------------------------------------
// Parity instrumentation (env-gated via STAGEPROBE, off by default). Mirrors
// `stageprobe` in crates/libslic3r-rs/src/arachne/wall_tool_paths.rs:
// per-ExtrusionLine flat% and distinct widths per line, per stage.
// ---------------------------------------------------------------------------
static void stageprobe(const char *stage, const std::vector<VariableWidthLines> &toolpaths)
{
    if (::getenv("STAGEPROBE") == nullptr)
        return;

    struct Acc { size_t lines = 0, juncs = 0, flat = 0, distinct_total = 0; };
    static std::mutex                 mtx;
    static std::map<std::string, Acc> acc;

    size_t lines = 0, juncs = 0, flat = 0, distinct_total = 0;
    for (const VariableWidthLines &vwl : toolpaths)
        for (const ExtrusionLine &line : vwl) {
            if (line.junctions.empty())
                continue;
            ++lines;
            juncs += line.junctions.size();
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

    if (stage[0] == '5' && e.lines > 0 && e.lines % 20000 < std::max<size_t>(lines, 1)) {
        fprintf(stderr, "[CPP-STAGEPROBE] ---- cumulative ----\n");
        for (const auto &kv : acc)
            fprintf(stderr, "  %-38s lines=%8zu juncs=%9zu flat=%5.1f%% distinct_w/line=%.2f\n",
                    kv.first.c_str(), kv.second.lines, kv.second.juncs,
                    100. * double(kv.second.flat) / double(std::max<size_t>(kv.second.lines, 1)),
                    double(kv.second.distinct_total) / double(std::max<size_t>(kv.second.lines, 1)));
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
    if (::getenv("POLYPROBE") == nullptr)
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
    if (stage[0] == '3' && ++rounds % 4000 == 0) {
        fprintf(stderr, "[CPP-POLYPROBE] ---- cumulative ----\n");
        for (const auto &kv : acc)
            fprintf(stderr, "  %-30s calls=%7zu polys=%9zu points=%10zu\n",
                    kv.first.c_str(), kv.second.calls, kv.second.polys, kv.second.points);
    }
}
"""

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
static void jwprobe(size_t njunc, size_t distinct, coord_t spread, bool flat)
{
    if (::getenv("JWPROBE") == nullptr)
        return;
    static std::mutex mtx;
    static size_t loops = 0, tot_j = 0, tot_d = 0, tot_flat = 0;
    static double tot_spread_um = 0;
    std::lock_guard<std::mutex> lock(mtx);
    ++loops; tot_j += njunc; tot_d += distinct; tot_spread_um += double(spread) / 100.0;
    if (flat) ++tot_flat;
    if (loops == 1 || loops % 5000 == 0)
        fprintf(stderr,
                "[CPP-JWPROBE] loops=%zu flat(min==max)=%zu (%.1f%%) mean_spread=%.1fum "
                "juncs/loop=%.2f distinct_w/loop=%.3f\n",
                loops, tot_flat, 100.0 * double(tot_flat) / double(loops),
                tot_spread_um / double(loops),
                double(tot_j) / double(loops), double(tot_d) / double(loops));
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
                    jwprobe(ws.size(), u.size(), mx - mn, mn == mx);
                }
            }"""

PG_INCLUDES_OLD = """#include "PerimeterGenerator.hpp\""""
PG_INCLUDES_NEW = """#include "PerimeterGenerator.hpp"
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


EDITS = [
    ("SkeletalTrapezoidation.cpp", ST_INCLUDES_OLD, ST_INCLUDES_NEW),
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
    ("WallToolPaths.cpp", WTP_INCLUDES_OLD, WTP_INCLUDES_NEW),
    ("WallToolPaths.cpp", WTP_PROBE_OLD, WTP_PROBE_NEW),
    ("WallToolPaths.cpp", WTP_CHAIN_OLD, WTP_CHAIN_NEW),
    ("WallToolPaths.cpp", WTP_POLY_CALLS_OLD, WTP_POLY_CALLS_NEW),
    ("WallToolPaths.cpp", WTP_POLY_CALLS2_OLD, WTP_POLY_CALLS2_NEW),
    ("WallToolPaths.cpp", WTP_POLY_CALLS3_OLD, WTP_POLY_CALLS3_NEW),
    ("WallToolPaths.cpp", WTP_PARAMS_OLD, WTP_PARAMS_NEW),
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

    wt = texts["WallToolPaths.cpp"]
    if "static void polyprobe" not in wt:
        if wt.count(WTP_POLY_ANCHOR) != 1:
            failures.append("WallToolPaths.cpp: stageprobe anchor not unique")
        else:
            texts["WallToolPaths.cpp"] = wt.replace(WTP_POLY_ANCHOR, WTP_POLY_FN + "\n" + WTP_POLY_ANCHOR, 1)

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
          "src/libslic3r/LayerRegion.cpp")
    return 0


if __name__ == "__main__":
    sys.exit(main())
