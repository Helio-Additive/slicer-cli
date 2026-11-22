// Generator of the vectors of tests/shortest_path_native_parity.rs: ShortestPath.cpp of libslic3r on
// deterministic inputs, with the results the Rust port has to reproduce bit for bit.
//
// Build with the compile line the build uses for a libslic3r source (its defines and include directories,
// -std=c++17 -O3 -DNDEBUG), link with the engine's libraries and run `./gen vectors.txt`.
// ShortestPath.cpp is compiled into this file, so its templates and static functions are reachable.
//
// One case per line, tokens separated by blanks; `P` is `x y`, a polyline is `m P*m`:
//   cp  n hs sx sy P*n                      | k idx*k               chain_points
//   g1  rcf n hs sx sy (P P cr)*n           | chain                 chain_segments_greedy (rcf 0) /
//                                                                   chain_segments_greedy_constrained_reversals (rcf 1)
//   g2  rcf n hs sx sy (P P cr)*n           | chain                 chain_segments_greedy2 /
//                                                                   chain_segments_greedy_constrained_reversals2
//   pl  n hs sx sy polyline*n               | k polyline*k          chain_polylines
//   rst n polyline*n                        | polyline*n            reorder_by_shortest_traverse
//   two n (P P)*n                           | (src P P)*n           reorder_by_two_exchanges_with_segment_flipping
//   imp n polyline*n                        | polyline*n            improve_ordering_by_two_exchanges_with_segment_flipping
//   mcc n (P P)*n a b c1 c1f c2 c2f c3 c3f cur | cost flip          minimum_crossover_cost (doubles as %a)
//   dc  n (P P)*n a b i                     | (src P P)*n           do_crossover
//   ex  n (polyline h polyline*h)*n         | idx*n                 chain_expolygons
//   cl  eps n (P P)*n                       | k polyline*k          chain_lines
//   ep  n hs sx sy polyline*n               | chain k polyline*k    chain_extrusion_paths, then
//                                                                   chain_and_reorder_extrusion_paths
//   ee  n hs sx sy entity*n                 | f chain r [chain entity*n] k entity*k
//                                             chain_extrusion_entities (f: no entity is empty, else the chain
//                                             is empty), reorder_extrusion_entities with a chain of the
//                                             generator's (r: present), chain_and_reorder_extrusion_entities
//   pn  n P*n                               | k idx*k               chain_clipper_polynodes (chain_path_items);
//                                                                   a node is named by its place in the input
//   inst no (k P*k)*no hs sx sy             | m (obj inst)*m m (obj inst)*m
//                                             chain_print_object_instances of the objects of a Print (the shift
//                                             of every instance of every object is the input) with the start
//                                             point, then the overload that takes the Print
// A chain is `k (idx rev)*k`: the second variant with a start point can return fewer segments than it was
// given (its loop ends after 16 rounds per segment, ShortestPath.cpp:750), and the release build returns that.
// An entity is `0 cr polyline` (path), `1 np polyline*np` (loop), `2 cr np polyline*np` (multi path) or
// `3 no_sort is_reverse k entity*k` (collection).
//
// The vectors were made on macOS arm64 with Apple clang.

#include "libslic3r/ShortestPath.cpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/TriangleMesh.hpp"

#include <cinttypes>
#include <cstdio>
#include <cstdlib>
#include <vector>

using namespace Slic3r;

struct Lcg {
    unsigned long long s;
    explicit Lcg(unsigned long long seed) : s(seed) {}
    unsigned next()
    {
        s = s * 6364136223846793005ULL + 1442695040888963407ULL;
        return unsigned(s >> 33);
    }
    // lo <= result < hi
    int range(int lo, int hi) { return lo + int(next() % unsigned(hi - lo)); }
};

struct Seg {
    Point a, b;
};

static const int NUM_MODES = 8;

// End points of n segments. The modes make what the chaining has to break ties on: shared end points,
// duplicates, equal distances, closed segments.
static std::vector<Seg> gen_segs(Lcg &r, int mode, int n)
{
    std::vector<Seg> out;
    out.reserve(n);
    switch (mode) {
    case 0: // anywhere on a bed
        for (int i = 0; i < n; ++i)
            out.push_back({Point(r.range(-20000000, 20000000), r.range(-20000000, 20000000)),
                           Point(r.range(-20000000, 20000000), r.range(-20000000, 20000000))});
        break;
    case 1: // a coarse grid: many equal distances and equal points
        for (int i = 0; i < n; ++i)
            out.push_back({Point(r.range(0, 7) * 100000, r.range(0, 7) * 100000), Point(r.range(0, 7) * 100000, r.range(0, 7) * 100000)});
        break;
    case 2: { // rasters of an infill: parallel lines at one spacing, the direction of each one drawn
        int spacing = r.range(1, 5) * 45000;
        int ylo     = r.range(-50, 50) * 1000;
        int yhi     = ylo + r.range(1, 80) * 10000;
        for (int i = 0; i < n; ++i) {
            int  x  = i * spacing;
            int  dy = (r.next() % 4 == 0) ? r.range(-3, 4) * 5000 : 0;
            bool up = r.next() % 2 == 0;
            Point lo(x, ylo + dy), hi(x, yhi + dy);
            out.push_back(up ? Seg{lo, hi} : Seg{hi, lo});
        }
        break;
    }
    case 3: // a tree: each segment starts where an earlier one starts or ends
        for (int i = 0; i < n; ++i) {
            Point a;
            if (i == 0)
                a = Point(r.range(0, 40) * 25000, r.range(0, 40) * 25000);
            else {
                const Seg &s = out[size_t(r.range(0, i))];
                a            = r.next() % 2 == 0 ? s.a : s.b;
            }
            out.push_back({a, Point(r.range(0, 40) * 25000, r.range(0, 40) * 25000)});
        }
        break;
    case 4: // closed segments (loops) among open ones
        for (int i = 0; i < n; ++i) {
            Point a(r.range(-3000000, 3000000), r.range(-3000000, 3000000));
            Point b = r.next() % 3 != 0 ? a : Point(r.range(-3000000, 3000000), r.range(-3000000, 3000000));
            out.push_back({a, b});
        }
        break;
    case 5: // duplicates and reversed duplicates
        for (int i = 0; i < n; ++i) {
            if (i > 0 && r.next() % 3 == 0) {
                Seg s = out[size_t(r.range(0, i))];
                if (r.next() % 2 == 0)
                    std::swap(s.a, s.b);
                out.push_back(s);
            } else
                out.push_back({Point(r.range(-500000, 500000), r.range(-500000, 500000)), Point(r.range(-500000, 500000), r.range(-500000, 500000))});
        }
        break;
    case 6: // a few units apart
        for (int i = 0; i < n; ++i)
            out.push_back({Point(r.range(-6, 7), r.range(-6, 7)), Point(r.range(-6, 7), r.range(-6, 7))});
        break;
    default: // closed loops only, on a grid (perimeters of islands in a row)
        for (int i = 0; i < n; ++i) {
            Point a(r.range(0, 12) * 200000, r.range(0, 3) * 200000);
            out.push_back({a, a});
        }
        break;
    }
    return out;
}

// start_near of a case: none, anywhere, or exactly on an end point.
static bool gen_start(Lcg &r, int hs, const std::vector<Seg> &segs, Point &start)
{
    start = Point(0, 0);
    if (hs == 0)
        return false;
    if (hs == 2 && !segs.empty()) {
        const Seg &s = segs[size_t(r.range(0, int(segs.size())))];
        start        = r.next() % 2 == 0 ? s.a : s.b;
    } else
        start = Point(r.range(-1000000, 1000000), r.range(-1000000, 1000000));
    return true;
}

static Polyline gen_polyline(Lcg &r, const Seg &s)
{
    Polyline pl;
    pl.points.push_back(s.a);
    int mid = r.range(0, 3);
    for (int i = 0; i < mid; ++i)
        pl.points.push_back(Point(s.a.x() + r.range(-40000, 40000), s.a.y() + r.range(-40000, 40000)));
    pl.points.push_back(s.b);
    return pl;
}

static void put(FILE *f, const Point &p) { std::fprintf(f, " %lld %lld", (long long) p.x(), (long long) p.y()); }
static void put(FILE *f, const Vec2d &p) { std::fprintf(f, " %a %a", p.x(), p.y()); }
static void put(FILE *f, const Polyline &pl)
{
    std::fprintf(f, " %zu", pl.points.size());
    for (const Point &p : pl.points)
        put(f, p);
}
static void put(FILE *f, const std::vector<std::pair<size_t, bool>> &chain)
{
    std::fprintf(f, " %zu", chain.size());
    for (const auto &c : chain)
        std::fprintf(f, " %zu %d", c.first, int(c.second));
}

static void put_entity(FILE *f, const ExtrusionEntity *ee)
{
    if (const ExtrusionEntityCollection *c = dynamic_cast<const ExtrusionEntityCollection *>(ee)) {
        std::fprintf(f, " 3 %d %d %zu", int(c->no_sort), int(c->no_sort ? 0 : c->can_reverse()), c->entities.size());
        for (const ExtrusionEntity *e : c->entities)
            put_entity(f, e);
    } else if (const ExtrusionLoop *l = dynamic_cast<const ExtrusionLoop *>(ee)) {
        std::fprintf(f, " 1 %zu", l->paths.size());
        for (const ExtrusionPath &p : l->paths)
            put(f, p.polyline);
    } else if (const ExtrusionMultiPath *m = dynamic_cast<const ExtrusionMultiPath *>(ee)) {
        std::fprintf(f, " 2 %d %zu", int(m->can_reverse()), m->paths.size());
        for (const ExtrusionPath &p : m->paths)
            put(f, p.polyline);
    } else {
        const ExtrusionPath *p = dynamic_cast<const ExtrusionPath *>(ee);
        std::fprintf(f, " 0 %d", int(p->can_reverse()));
        put(f, p->polyline);
    }
}

static ExtrusionPath gen_path(Lcg &r, const Seg &s)
{
    ExtrusionPath p(erInternalInfill, 1., 0.4f, 0.2f);
    p.polyline = gen_polyline(r, s);
    return p;
}

// An entity that runs from s.a to s.b (a loop starts and ends at s.a).
static ExtrusionEntity *gen_entity(Lcg &r, const Seg &s, int depth)
{
    int kind = r.range(0, depth > 0 ? 3 : 4);
    if (kind == 1) {
        // A loop of one or two paths that comes back to s.a.
        ExtrusionPaths paths;
        Point          mid(s.a.x() + r.range(-40000, 40000), s.a.y() + r.range(-40000, 40000));
        if (r.next() % 2 == 0) {
            paths.push_back(gen_path(r, Seg{s.a, mid}));
            paths.push_back(gen_path(r, Seg{mid, s.a}));
        } else
            paths.push_back(gen_path(r, Seg{s.a, s.a}));
        return new ExtrusionLoop(paths, elrDefault);
    }
    if (kind == 2) {
        ExtrusionMultiPath *m = new ExtrusionMultiPath();
        Point               mid(s.a.x() + r.range(-40000, 40000), s.a.y() + r.range(-40000, 40000));
        m->paths.push_back(gen_path(r, Seg{s.a, mid}));
        m->paths.push_back(gen_path(r, Seg{mid, s.b}));
        if (r.next() % 3 == 0)
            m->set_reverse();
        return m;
    }
    if (kind == 3) {
        ExtrusionEntityCollection *c = new ExtrusionEntityCollection();
        Point                      mid(s.a.x() + r.range(-40000, 40000), s.a.y() + r.range(-40000, 40000));
        c->entities.push_back(gen_entity(r, Seg{s.a, mid}, depth + 1));
        c->entities.push_back(gen_entity(r, Seg{mid, s.b}, depth + 1));
        unsigned flags = r.next() % 4;
        if (flags == 1)
            c->no_sort = true;
        if (flags == 2)
            c->set_reverse();
        return c;
    }
    ExtrusionPath *p = new ExtrusionPath(gen_path(r, s));
    if (r.next() % 3 == 0)
        p->set_reverse();
    return p;
}

static void free_entities(std::vector<ExtrusionEntity *> &ents)
{
    for (ExtrusionEntity *e : ents)
        delete e;
    ents.clear();
}

int main(int argc, char **argv)
{
    FILE *out = argc > 1 ? std::fopen(argv[1], "w") : stdout;
    if (out == nullptr)
        return 1;
    std::fprintf(out, "traits %d\n", int(sizeof(coord_t)));

    const int sizes[]       = {0, 1, 2, 3, 4, 5, 6, 7, 8, 10, 13, 17, 25, 40, 64};
    const int sizes_small[] = {0, 1, 2, 3, 4, 5, 7, 10, 16, 30};
    unsigned long long seed = 20240922ULL;

    // chain_points
    for (int mode = 0; mode < NUM_MODES; ++mode)
        for (int n : sizes)
            for (int hs = 0; hs < 3; ++hs) {
                Lcg              r(++seed);
                std::vector<Seg> segs = gen_segs(r, mode, n);
                Points           pts;
                for (const Seg &s : segs)
                    pts.push_back(s.a);
                Point start;
                bool  has = gen_start(r, hs, segs, start);
                std::fprintf(out, "cp %d %d", n, int(has));
                put(out, start);
                for (const Point &p : pts)
                    put(out, p);
                std::vector<size_t> order = chain_points(pts, has ? &start : nullptr);
                std::fprintf(out, " %zu", order.size());
                for (size_t idx : order)
                    std::fprintf(out, " %zu", idx);
                std::fprintf(out, "\n");
            }

    // chain_segments_greedy / chain_segments_greedy_constrained_reversals and the second variant of both
    for (int variant = 1; variant <= 2; ++variant)
        for (int rcf = 0; rcf < 2; ++rcf)
            for (int mode = 0; mode < NUM_MODES; ++mode)
                for (int n : sizes)
                    for (int hs = 0; hs < 3; ++hs) {
                        Lcg               r(++seed);
                        std::vector<Seg>  segs = gen_segs(r, mode, n);
                        std::vector<char> cr(segs.size(), 1);
                        if (rcf) {
                            unsigned kind = r.next() % 3;
                            for (char &c : cr)
                                c = kind == 0 ? 0 : kind == 1 ? char(r.next() % 2) : char(r.next() % 4 != 0);
                            // What chain_extrusion_entities passes: a closed segment is one that may be reversed.
                            if (r.next() % 2 == 0)
                                for (size_t i = 0; i < segs.size(); ++i)
                                    if (segs[i].a == segs[i].b)
                                        cr[i] = 1;
                        }
                        Point start;
                        bool  has               = gen_start(r, hs, segs, start);
                        auto  segment_end_point = [&segs](size_t idx, bool first_point) -> const Point & { return first_point ? segs[idx].a : segs[idx].b; };
                        auto  could_reverse     = [&cr](size_t idx) -> bool { return cr[idx] != 0; };
                        std::vector<std::pair<size_t, bool>> chain;
                        const Point *sn = has ? &start : nullptr;
                        if (variant == 1)
                            chain = rcf ? chain_segments_greedy_constrained_reversals<Point, decltype(segment_end_point), decltype(could_reverse)>(segment_end_point, could_reverse, segs.size(), sn) :
                                          chain_segments_greedy<Point, decltype(segment_end_point)>(segment_end_point, segs.size(), sn);
                        else
                            chain = rcf ? chain_segments_greedy_constrained_reversals2<Point, decltype(segment_end_point), decltype(could_reverse)>(segment_end_point, could_reverse, segs.size(), sn) :
                                          chain_segments_greedy2<Point, decltype(segment_end_point)>(segment_end_point, segs.size(), sn);
                        std::fprintf(out, "g%d %d %d %d", variant, rcf, n, int(has));
                        put(out, start);
                        for (size_t i = 0; i < segs.size(); ++i) {
                            put(out, segs[i].a);
                            put(out, segs[i].b);
                            std::fprintf(out, " %d", int(cr[i]));
                        }
                        put(out, chain);
                        std::fprintf(out, "\n");
                    }

    // chain_polylines, reorder_by_shortest_traverse, improve_ordering_by_two_exchanges_with_segment_flipping
    for (int mode = 0; mode < NUM_MODES; ++mode)
        for (int n : sizes)
            for (int hs = 0; hs < 3; ++hs) {
                Lcg              r(++seed);
                std::vector<Seg> segs = gen_segs(r, mode, n);
                Polylines        pls;
                for (const Seg &s : segs)
                    pls.push_back(gen_polyline(r, s));
                Point start;
                bool  has = gen_start(r, hs, segs, start);
                std::fprintf(out, "pl %d %d", n, int(has));
                put(out, start);
                for (const Polyline &pl : pls)
                    put(out, pl);
                Polylines chained = chain_polylines(pls, has ? &start : nullptr);
                std::fprintf(out, " %zu", chained.size());
                for (const Polyline &pl : chained)
                    put(out, pl);
                std::fprintf(out, "\n");
                if (hs == 0 && n > 0) {
                    Polylines rst = pls;
                    reorder_by_shortest_traverse(rst);
                    std::fprintf(out, "rst %d", n);
                    for (const Polyline &pl : pls)
                        put(out, pl);
                    for (const Polyline &pl : rst)
                        put(out, pl);
                    std::fprintf(out, "\n");
                    Polylines imp = pls;
                    improve_ordering_by_two_exchanges_with_segment_flipping(imp, false);
                    std::fprintf(out, "imp %d", n);
                    for (const Polyline &pl : pls)
                        put(out, pl);
                    for (const Polyline &pl : imp)
                        put(out, pl);
                    std::fprintf(out, "\n");
                }
            }

    // reorder_by_two_exchanges_with_segment_flipping, minimum_crossover_cost, do_crossover
    for (int mode = 0; mode < NUM_MODES; ++mode)
        for (int n : sizes)
            for (int rep = 0; rep < 2; ++rep) {
                Lcg                   r(++seed);
                std::vector<Seg>      segs = gen_segs(r, mode, n);
                std::vector<FlipEdge> edges;
                for (size_t i = 0; i < segs.size(); ++i)
                    edges.emplace_back(segs[i].a.cast<double>(), segs[i].b.cast<double>(), i);
                {
                    std::vector<FlipEdge> e = edges;
                    reorder_by_two_exchanges_with_segment_flipping(e);
                    std::fprintf(out, "two %d", n);
                    for (const FlipEdge &x : edges) {
                        put(out, x.p1);
                        put(out, x.p2);
                    }
                    for (const FlipEdge &x : e) {
                        std::fprintf(out, " %zu", x.source_index);
                        put(out, x.p1);
                        put(out, x.p2);
                    }
                    std::fprintf(out, "\n");
                }
                if (n >= 3) {
                    // The arguments reorder_by_two_exchanges_with_segment_flipping passes (ShortestPath.cpp:1565-1595).
                    std::vector<ConnectionCost> connections(edges.size());
                    for (size_t i = 1; i < edges.size(); ++i) {
                        ConnectionCost &c = connections[i];
                        c                 = connections[i - 1];
                        c.cost += (edges[i].p1 - edges[i - 1].p2).norm();
                        c.cost_flipped += (edges[i].p2 - edges[i - 1].p1).norm();
                    }
                    for (int k = 0; k < 3; ++k) {
                        size_t a = size_t(r.range(1, n - 1));
                        size_t b = size_t(r.range(int(a) + 1, n));
                        ConnectionCost c1 = connections[a - 1], c2 = connections[b - 1] - connections[a], c3 = connections.back() - connections[b];
                        std::pair<double, size_t> res = minimum_crossover_cost(edges, std::make_pair(size_t(0), a), c1, std::make_pair(a, b), c2,
                                                                               std::make_pair(b, edges.size()), c3, connections.back().cost);
                        std::fprintf(out, "mcc %d", n);
                        for (const FlipEdge &x : edges) {
                            put(out, x.p1);
                            put(out, x.p2);
                        }
                        std::fprintf(out, " %zu %zu %a %a %a %a %a %a %a %a %zu\n", a, b, c1.cost, c1.cost_flipped, c2.cost, c2.cost_flipped, c3.cost, c3.cost_flipped,
                                     connections.back().cost, res.first, res.second);
                        size_t                i = size_t(r.range(1, 3 << 6));
                        std::vector<FlipEdge> edges_out(edges);
                        do_crossover(edges, edges_out, std::make_pair(size_t(0), a), std::make_pair(a, b), std::make_pair(b, edges.size()), i);
                        std::fprintf(out, "dc %d", n);
                        for (const FlipEdge &x : edges) {
                            put(out, x.p1);
                            put(out, x.p2);
                        }
                        std::fprintf(out, " %zu %zu %zu", a, b, i);
                        for (const FlipEdge &x : edges_out) {
                            std::fprintf(out, " %zu", x.source_index);
                            put(out, x.p1);
                            put(out, x.p2);
                        }
                        std::fprintf(out, "\n");
                    }
                }
            }

    // chain_expolygons
    for (int mode = 0; mode < NUM_MODES; ++mode)
        for (int n : sizes_small) {
            Lcg              r(++seed);
            std::vector<Seg> segs = gen_segs(r, mode, n);
            ExPolygons       expolys;
            for (const Seg &s : segs) {
                ExPolygon ex;
                int       w = r.range(1, 60) * 1000, h = r.range(1, 60) * 1000;
                ex.contour.points = {s.a, Point(s.a.x() + w, s.a.y()), Point(s.a.x() + w, s.a.y() + h), Point(s.a.x() + r.range(0, w), s.a.y() + h + r.range(0, 5000))};
                if (r.next() % 3 == 0) {
                    Polygon hole;
                    hole.points = {Point(s.a.x() + w / 4, s.a.y() + h / 4), Point(s.a.x() + w / 4, s.a.y() + h / 2), Point(s.a.x() + w / 2, s.a.y() + h / 2)};
                    ex.holes.push_back(hole);
                }
                expolys.push_back(ex);
            }
            std::fprintf(out, "ex %d", n);
            for (const ExPolygon &ex : expolys) {
                std::fprintf(out, " %zu", ex.contour.points.size());
                for (const Point &p : ex.contour.points)
                    put(out, p);
                std::fprintf(out, " %zu", ex.holes.size());
                for (const Polygon &hole : ex.holes) {
                    std::fprintf(out, " %zu", hole.points.size());
                    for (const Point &p : hole.points)
                        put(out, p);
                }
            }
            for (size_t idx : chain_expolygons(expolys))
                std::fprintf(out, " %zu", idx);
            std::fprintf(out, "\n");
        }

    // chain_lines. `point_distance_epsilon2` is a function static (ShortestPath.cpp:2035): the square of the
    // epsilon of the FIRST call of the process is what every later call compares with, so one epsilon here.
    // Only inputs where no two lines join (mode 0): once a line is appended the pin erases the map node
    // `line_end` points into and reads it twice more (ShortestPath.cpp:2053-2055); this generator crashed
    // there on the first joined pair, so what the pin returns for joined lines cannot be recorded.
    const double chain_lines_epsilon = 1500.;
    for (int mode = 0; mode < 1; ++mode)
        for (int n : sizes)
            for (int rep = 0; rep < 2; ++rep) {
                Lcg              r(++seed);
                std::vector<Seg> segs = gen_segs(r, mode, n);
                Lines            lines;
                for (const Seg &s : segs) {
                    // End points that almost meet: within the epsilon, on its border and beyond it.
                    Point b = s.b;
                    if (r.next() % 2 == 0)
                        b = Point(b.x() + r.range(-1600, 1601), b.y() + r.range(-1600, 1601));
                    lines.emplace_back(s.a, b);
                }
                std::fprintf(out, "cl %a %d", chain_lines_epsilon, n);
                for (const Line &l : lines) {
                    put(out, l.a);
                    put(out, l.b);
                }
                Polylines res = chain_lines(lines, chain_lines_epsilon);
                std::fprintf(out, " %zu", res.size());
                for (const Polyline &pl : res)
                    put(out, pl);
                std::fprintf(out, "\n");
            }

    // chain_extrusion_paths, chain_and_reorder_extrusion_paths
    for (int mode = 0; mode < NUM_MODES; ++mode)
        for (int n : sizes_small)
            for (int hs = 0; hs < 3; ++hs) {
                Lcg              r(++seed);
                std::vector<Seg> segs = gen_segs(r, mode, n);
                ExtrusionPaths   paths;
                for (const Seg &s : segs)
                    paths.push_back(gen_path(r, s));
                Point start;
                bool  has = gen_start(r, hs, segs, start);
                std::fprintf(out, "ep %d %d", n, int(has));
                put(out, start);
                for (const ExtrusionPath &p : paths)
                    put(out, p.polyline);
                put(out, chain_extrusion_paths(paths, has ? &start : nullptr));
                chain_and_reorder_extrusion_paths(paths, has ? &start : nullptr);
                std::fprintf(out, " %zu", paths.size());
                for (const ExtrusionPath &p : paths)
                    put(out, p.polyline);
                std::fprintf(out, "\n");
            }

    // chain_extrusion_entities, chain_and_reorder_extrusion_entities, reorder_extrusion_entities
    for (int mode = 0; mode < NUM_MODES; ++mode)
        for (int n : sizes_small)
            for (int hs = 0; hs < 3; ++hs) {
                Lcg                            r(++seed);
                std::vector<Seg>               segs = gen_segs(r, mode, n);
                std::vector<ExtrusionEntity *> ents;
                // chain_and_reorder_extrusion_entities reads every entity as a collection (ShortestPath.cpp:1034):
                // every third case is made of collections only, some of them empty, the other cases hold no
                // empty entity.
                bool collections = r.next() % 3 == 0;
                for (const Seg &s : segs) {
                    if (collections) {
                        ExtrusionEntityCollection *c = new ExtrusionEntityCollection();
                        if (r.next() % 4 != 0) {
                            Point mid(s.a.x() + r.range(-40000, 40000), s.a.y() + r.range(-40000, 40000));
                            c->entities.push_back(gen_entity(r, Seg{s.a, mid}, 1));
                            if (r.next() % 2 == 0)
                                c->entities.push_back(gen_entity(r, Seg{mid, s.b}, 1));
                        }
                        unsigned flags = r.next() % 4;
                        if (flags == 1)
                            c->no_sort = true;
                        if (flags == 2)
                            c->set_reverse();
                        ents.push_back(c);
                    } else
                        ents.push_back(gen_entity(r, s, 0));
                }
                Point start;
                bool  has = gen_start(r, hs, segs, start);
                std::fprintf(out, "ee %d %d", n, int(has));
                put(out, start);
                for (const ExtrusionEntity *e : ents)
                    put_entity(out, e);
                // chain_extrusion_entities asks every entity for its end points: only on a set without empty ones.
                std::vector<ExtrusionEntity *> full;
                for (ExtrusionEntity *e : ents) {
                    const ExtrusionEntityCollection *c = dynamic_cast<const ExtrusionEntityCollection *>(e);
                    if (c == nullptr || !c->empty())
                        full.push_back(e);
                }
                bool                                 all_full = full.size() == ents.size();
                std::vector<std::pair<size_t, bool>> chain;
                if (all_full)
                    chain = chain_extrusion_entities(ents, has ? &start : nullptr);
                std::fprintf(out, " %d", int(all_full));
                put(out, chain);
                if (all_full && !chain.empty()) {
                    // reorder_extrusion_entities with a chain of its own: the reversed order, flags drawn.
                    std::vector<ExtrusionEntity *> copy;
                    for (const ExtrusionEntity *e : ents)
                        copy.push_back(e->clone());
                    std::vector<std::pair<size_t, bool>> chain2;
                    for (size_t i = copy.size(); i-- > 0;)
                        chain2.emplace_back(i, r.next() % 2 == 0);
                    std::fprintf(out, " 1");
                    put(out, chain2);
                    reorder_extrusion_entities(copy, chain2);
                    for (const ExtrusionEntity *e : copy)
                        put_entity(out, e);
                    free_entities(copy);
                } else
                    std::fprintf(out, " 0");
                std::vector<ExtrusionEntity *> removed;
                for (ExtrusionEntity *e : ents) {
                    const ExtrusionEntityCollection *c = dynamic_cast<const ExtrusionEntityCollection *>(e);
                    if (c != nullptr && c->empty())
                        removed.push_back(e);
                }
                chain_and_reorder_extrusion_entities(ents, has ? &start : nullptr);
                std::fprintf(out, " %zu", ents.size());
                for (const ExtrusionEntity *e : ents)
                    put_entity(out, e);
                std::fprintf(out, "\n");
                free_entities(ents);
                free_entities(removed);
            }

    // chain_clipper_polynodes (chain_path_items)
    for (int mode = 0; mode < NUM_MODES; ++mode)
        for (int n : sizes_small) {
            Lcg              r(++seed);
            std::vector<Seg> segs = gen_segs(r, mode, n);
            Points           pts;
            for (const Seg &s : segs)
                pts.push_back(s.a);
            std::vector<Slic3r::ClipperLib::PolyNode> store(segs.size());
            Slic3r::ClipperLib::PolyNodes             nodes;
            for (Slic3r::ClipperLib::PolyNode &node : store)
                nodes.push_back(&node);
            std::fprintf(out, "pn %d", n);
            for (const Point &p : pts)
                put(out, p);
            Slic3r::ClipperLib::PolyNodes res = chain_clipper_polynodes(pts, nodes);
            std::fprintf(out, " %zu", res.size());
            for (const Slic3r::ClipperLib::PolyNode *node : res)
                std::fprintf(out, " %zu", size_t(node - store.data()));
            std::fprintf(out, "\n");
        }

    // chain_print_object_instances: a Print of boxes, a few instances of each.
    for (int mode = 0; mode < NUM_MODES; ++mode)
        for (int n_objects : {1, 2, 3, 5})
            for (int hs = 0; hs < 3; ++hs) {
                Lcg   r(++seed);
                Model model;
                for (int i = 0; i < n_objects; ++i) {
                    ModelObject *o = model.add_object();
                    o->name        = "box";
                    o->add_volume(TriangleMesh(its_make_cube(10, 10, 5)));
                    for (const Seg &s : gen_segs(r, mode, r.range(1, 5)))
                        o->add_instance()->set_offset(Vec3d(unscale<double>(s.a.x()), unscale<double>(s.a.y()), 0.));
                }
                DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
                Print              print;
                print.apply(model, config);
                std::vector<Seg> shifts;
                std::fprintf(out, "inst %zu", print.objects().size());
                for (const PrintObject *po : print.objects()) {
                    std::fprintf(out, " %zu", po->instances().size());
                    for (const PrintInstance &pi : po->instances()) {
                        put(out, pi.shift);
                        shifts.push_back({pi.shift, pi.shift});
                    }
                }
                Point start;
                bool  has = gen_start(r, hs, shifts, start);
                std::fprintf(out, " %d", int(has));
                put(out, start);
                auto put_instances = [&print, out](const std::vector<const PrintInstance *> &v) {
                    std::fprintf(out, " %zu", v.size());
                    for (const PrintInstance *pi : v) {
                        size_t oi = 0;
                        while (oi < print.objects().size() && print.objects()[oi] != pi->print_object)
                            ++oi;
                        std::fprintf(out, " %zu %zu", oi, size_t(pi - print.objects()[oi]->instances().data()));
                    }
                };
                put_instances(chain_print_object_instances(print.objects().vector(), has ? &start : nullptr));
                put_instances(chain_print_object_instances(print));
                std::fprintf(out, "\n");
            }

    if (out != stdout)
        std::fclose(out);
    return 0;
}
