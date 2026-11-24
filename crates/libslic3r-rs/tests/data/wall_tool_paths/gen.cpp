// Vectors for tests/wall_tool_paths_native_parity.rs: the outline preparation of
// WallToolPaths::generate (WallToolPaths.cpp:458-480) step by step, with the pin's own free functions
// simplify, fixSelfIntersections, removeDegenerateVerts, removeColinearEdges and removeSmallAreas
// (WallToolPaths.cpp:50-439), on
//   - "chain" cases: the triple offset of a generated outline, then the seven steps in the order of
//     generate(), each printed after it runs (the test feeds every step the pin's previous output);
//   - "real" case: the prepared outline (after the triple offset) of a Majora layer-1 call whose
//     fixSelfIntersections output differed in R945, read from argv[1] (majora_l1_outline.txt);
//   - direct cases of each function on raw generated polygons with near-touching parts, spikes,
//     back-and-forth vertices and colinear runs, so every branch is reached.
// Usage: gen <majora_l1_outline.txt> > vectors.txt
// Output: "case <kind> <id>" then per step "step <name> <npolys>" followed by one "poly <n> x,y ..." line
// per polygon.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/Arachne/WallToolPaths.hpp"

using namespace Slic3r;

namespace Slic3r::Arachne {
void simplify(Polygons &thiss, const int64_t smallest_line_segment, const int64_t allowed_error_distance);
void fixSelfIntersections(const coord_t epsilon, Polygons &thiss);
void removeDegenerateVerts(Polygons &thiss);
void removeSmallAreas(Polygons &thiss, const double min_area_size, const bool remove_holes);
void removeColinearEdges(Polygons &thiss, const double max_deviation_angle);
} // namespace Slic3r::Arachne

static uint64_t g_state = 0x2545F4914F6CDD1DULL;
static uint64_t next_u64()
{
    g_state = g_state * 6364136223846793005ULL + 1442695040888963407ULL;
    return g_state >> 11;
}
static int64_t range(int64_t lo, int64_t hi) { return lo + int64_t(next_u64() % uint64_t(hi - lo + 1)); }

static void print_step(const char *name, const Polygons &pp)
{
    printf("step %s %zu\n", name, pp.size());
    for (const Polygon &p : pp) {
        printf("poly %zu", p.points.size());
        for (const Point &q : p.points) printf(" %lld,%lld", (long long) q.x(), (long long) q.y());
        printf("\n");
    }
}

// A closed outline around (cx, cy): n vertices at increasing angles, radius r with relative noise,
// occasional spikes, repeated points, colinear mid-points and 1-unit jitter. `hole` reverses it.
static Polygon noisy_outline(int64_t cx, int64_t cy, int64_t r, int n, int noise_permille, bool hole)
{
    Polygon p;
    std::vector<double> angles;
    for (int i = 0; i < n; ++i) angles.push_back(double(range(0, 1000000)) / 1000000. * 2. * M_PI);
    std::sort(angles.begin(), angles.end());
    for (int i = 0; i < n; ++i) {
        double rr = double(r) * (1. + double(range(-noise_permille, noise_permille)) / 1000.);
        if (range(0, 40) == 0) rr *= 1.0 + double(range(5, 40)) / 100.; // spike
        Point q(cx + int64_t(rr * cos(angles[i])) + range(-1, 1), cy + int64_t(rr * sin(angles[i])) + range(-1, 1));
        if (!p.points.empty() && range(0, 30) == 0) {
            // colinear mid-point before q
            const Point &a = p.points.back();
            p.points.emplace_back((a.x() + q.x()) / 2, (a.y() + q.y()) / 2);
        }
        p.points.push_back(q);
        if (range(0, 50) == 0) p.points.push_back(q); // repeated point
        if (range(0, 60) == 0 && p.points.size() > 2) {
            // back-and-forth vertex: go out and come back along the same line
            const Point &a = p.points[p.points.size() - 2];
            p.points.emplace_back(q.x() + (q.x() - a.x()) / 3, q.y() + (q.y() - a.y()) / 3);
            p.points.push_back(q);
        }
    }
    if (hole) std::reverse(p.points.begin(), p.points.end());
    return p;
}

// A thin comb: a rectangle with teeth whose gaps are a few microns, so offsets close or keep them
// and fixSelfIntersections finds vertices near other segments.
static Polygon comb(int64_t x0, int64_t y0, int teeth, int64_t tooth_w, int64_t gap, int64_t len)
{
    Polygon p;
    p.points.emplace_back(x0, y0);
    int64_t x = x0;
    for (int i = 0; i < teeth; ++i) {
        p.points.emplace_back(x, y0 + len + range(-3, 3));
        p.points.emplace_back(x + tooth_w, y0 + len + range(-3, 3));
        x += tooth_w;
        if (i + 1 < teeth) {
            p.points.emplace_back(x, y0 + range(200, 900));
            x += gap + range(-2, 2);
            p.points.emplace_back(x, y0 + range(200, 900));
        }
    }
    p.points.emplace_back(x, y0 - 3000);
    p.points.emplace_back(x0, y0 - 3000);
    return p;
}

static Polygons random_outline_set(int kind)
{
    Polygons out;
    int64_t cx = range(-8000000, 8000000), cy = range(-8000000, 8000000);
    switch (kind % 5) {
    case 0: { // one noisy island with a hole
        int64_t r = range(200000, 3000000);
        out.push_back(noisy_outline(cx, cy, r, int(range(8, 250)), int(range(0, 60)), false));
        if (range(0, 1)) out.push_back(noisy_outline(cx, cy, r / 2, int(range(8, 120)), int(range(0, 60)), true));
        break;
    }
    case 1: { // two islands almost touching
        int64_t r = range(100000, 1500000);
        int64_t gap = range(-3000, 6000);
        out.push_back(noisy_outline(cx, cy, r, int(range(8, 150)), int(range(0, 20)), false));
        out.push_back(noisy_outline(cx + 2 * r + gap, cy, r, int(range(8, 150)), int(range(0, 20)), false));
        break;
    }
    case 2: // a comb with micron gaps
        out.push_back(comb(cx, cy, int(range(2, 12)), range(1000, 60000), range(500, 6000), range(20000, 400000)));
        break;
    case 3: { // a ring with a thin wall
        int64_t r = range(300000, 2000000);
        int64_t w = range(1500, 40000);
        out.push_back(noisy_outline(cx, cy, r, int(range(30, 200)), 2, false));
        out.push_back(noisy_outline(cx, cy, r - w, int(range(30, 200)), 2, true));
        break;
    }
    default: { // many small islands, some below the small-area limit
        int k = int(range(2, 12));
        for (int i = 0; i < k; ++i)
            out.push_back(noisy_outline(cx + range(-2000000, 2000000), cy + range(-2000000, 2000000), range(3000, 300000), int(range(3, 60)),
                                        int(range(0, 100)), range(0, 4) == 0));
        break;
    }
    }
    return out;
}

// The chain of WallToolPaths::generate (WallToolPaths.cpp:458-480), from a prepared outline.
static void run_chain(Polygons prepared_outline, double small_area_length)
{
    const coord_t smallest_segment = Slic3r::Arachne::meshfix_maximum_resolution;
    const coord_t allowed_distance = Slic3r::Arachne::meshfix_maximum_deviation;
    const coord_t epsilon_offset   = (allowed_distance / 2) - 1;
    printf("param %lld %lld %lld %a\n", (long long) smallest_segment, (long long) allowed_distance, (long long) epsilon_offset, small_area_length);
    print_step("in", prepared_outline);
    Arachne::simplify(prepared_outline, smallest_segment, allowed_distance);
    print_step("simplify", prepared_outline);
    Arachne::fixSelfIntersections(epsilon_offset, prepared_outline);
    print_step("fixself", prepared_outline);
    Arachne::removeDegenerateVerts(prepared_outline);
    print_step("degenerate", prepared_outline);
    Arachne::removeColinearEdges(prepared_outline, 0.005);
    print_step("colinear", prepared_outline);
    Arachne::fixSelfIntersections(epsilon_offset, prepared_outline);
    print_step("fixself", prepared_outline);
    Arachne::removeDegenerateVerts(prepared_outline);
    print_step("degenerate", prepared_outline);
    Arachne::removeSmallAreas(prepared_outline, small_area_length * small_area_length, false);
    print_step("smallareas", prepared_outline);
}

int main(int argc, char **argv)
{
    const coord_t allowed_distance = Slic3r::Arachne::meshfix_maximum_deviation;
    const coord_t epsilon_offset   = (allowed_distance / 2) - 1;

    // The Majora layer-1 call (bead_width_0 = 0.42 mm scaled: small_area_length = 21000).
    if (argc > 1) {
        FILE *f = fopen(argv[1], "r");
        if (f == nullptr) {
            fprintf(stderr, "cannot open %s\n", argv[1]);
            return 1;
        }
        Polygons pp;
        char tag[8];
        long long pi, x, y;
        while (fscanf(f, "%7s %lld %lld %lld", tag, &pi, &x, &y) == 4) {
            while (int64_t(pp.size()) <= pi) pp.emplace_back();
            pp[size_t(pi)].points.emplace_back(coord_t(x), coord_t(y));
        }
        fclose(f);
        printf("case real 0\n");
        run_chain(pp, 42000. / 2.);
    }

    // Chain cases: the triple offset (WallToolPaths.cpp:461) of a generated outline, then the steps.
    for (int i = 0; i < 250; ++i) {
        Polygons outline = random_outline_set(i);
        Polygons prepared = offset(offset(offset(outline, -epsilon_offset), epsilon_offset * 2), -epsilon_offset);
        printf("case chain %d\n", i);
        run_chain(prepared, double(range(10000, 30000)) / 2.);
    }

    // Direct cases on the raw outlines (no offset), each function on its own input.
    for (int i = 0; i < 250; ++i) {
        Polygons outline = random_outline_set(i + 7);
        printf("case direct %d\n", i);
        print_step("in", outline);
        {
            Polygons pp = outline;
            int64_t seg = range(1000, 80000), dev = range(100, 5000);
            printf("arg simplify %lld %lld\n", (long long) seg, (long long) dev);
            Arachne::simplify(pp, seg, dev);
            print_step("simplify", pp);
        }
        {
            Polygons pp = outline;
            coord_t eps = coord_t(range(0, 4)) == 0 ? 0 : coord_t(range(1, 6000));
            printf("arg fixself %lld\n", (long long) eps);
            Arachne::fixSelfIntersections(eps, pp);
            print_step("fixself", pp);
        }
        {
            Polygons pp = outline;
            Arachne::removeDegenerateVerts(pp);
            print_step("degenerate", pp);
        }
        {
            Polygons pp = outline;
            double ang = (range(0, 1) ? 0.005 : 0.0005) * double(range(1, 40));
            printf("arg colinear %a\n", ang);
            Arachne::removeColinearEdges(pp, ang);
            print_step("colinear", pp);
        }
        {
            Polygons pp = outline;
            double a = double(range(1000, 400000));
            bool holes = range(0, 1) != 0;
            printf("arg smallareas %a %d\n", a * a, int(holes));
            Arachne::removeSmallAreas(pp, a * a, holes);
            print_step("smallareas", pp);
        }
    }
    return 0;
}
