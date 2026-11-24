// Native vectors for SkeletalTrapezoidation::interpolate (SkeletalTrapezoidation.cpp:1709-1771), both
// overloads, called on the pin's own class. Usage: gen <cases>   (vectors go to stdout)
// Line format:
//   i3 <ratio %a> <fma_sensitive 0|1> L <beading> R <beading> => <beading>
//   i4 <ratio %a> <switching_radius> L <beading> R <beading> => <beading>
// with <beading> = <total_thickness> <left_over> <n> <bead_widths x n> <toolpath_locations x n>.
// fma_sensitive is 1 when a fused and an unfused evaluation of the two sums of the 3-argument overload
// truncate to different integers for this case (the cases that tell clang's contraction apart).
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

#define protected public
#define private public
#include "libslic3r/Arachne/SkeletalTrapezoidation.hpp"
#include "libslic3r/Arachne/BeadingStrategy/BeadingStrategyFactory.hpp"

using namespace Slic3r;
using namespace Slic3r::Arachne;

static uint64_t g_state = 0x2545F4914F6CDD1DULL;
static uint64_t next_u64()
{
    g_state = g_state * 6364136223846793005ULL + 1442695040888963407ULL;
    return g_state >> 11;
}
static int64_t range(int64_t lo, int64_t hi) { return lo + int64_t(next_u64() % uint64_t(hi - lo + 1)); }

using Beading = BeadingStrategy::Beading;

static Beading random_beading(size_t n)
{
    Beading b;
    b.total_thickness = range(10000, 900000);
    b.left_over       = range(0, 20000);
    coord_t loc       = range(0, 30000);
    for (size_t i = 0; i < n; ++i) {
        // 0-width wall markers now and then, as the inner contour lines carry them.
        b.bead_widths.push_back(range(0, 9) == 0 ? 0 : range(20000, 90000));
        b.toolpath_locations.push_back(loc);
        loc += range(10000, 60000);
    }
    return b;
}

// A second beading that shares most bead widths and toolpath locations with `left` (as the beadings of
// neighbouring nodes do: equal outer wall widths and locations), so that rl * v + rr * v sits next to an
// integer and the contraction decides the truncation.
static Beading related_beading(const Beading &left)
{
    Beading b = left;
    b.total_thickness = range(10000, 900000);
    size_t n = size_t(range(1, 6));
    b.bead_widths.resize(n, 0);
    b.toolpath_locations.resize(n, 0);
    for (size_t i = 0; i < n; ++i) {
        if (i >= left.bead_widths.size() || range(0, 3) == 0) {
            b.bead_widths[i]        = range(0, 9) == 0 ? 0 : range(20000, 90000);
            b.toolpath_locations[i] = (i == 0 ? 0 : b.toolpath_locations[i - 1]) + range(10000, 60000);
        }
    }
    return b;
}

static void print_beading(const Beading &b)
{
    printf(" %lld %lld %zu", (long long) b.total_thickness, (long long) b.left_over, b.bead_widths.size());
    for (coord_t w : b.bead_widths) printf(" %lld", (long long) w);
    for (coord_t t : b.toolpath_locations) printf(" %lld", (long long) t);
}

// One sum of interpolate() evaluated with and without the fused multiply-add.
static bool sum_sensitive(double rl, float rr, coord_t l, coord_t r)
{
    volatile double prod = rl * double(l);
    double          rhs  = double(rr * float(r));
    coord_t         unfused = coord_t(prod + rhs);
    coord_t         fused   = coord_t(std::fma(rl, double(l), rhs));
    return unfused != fused;
}

static bool fma_sensitive(const Beading &left, double rl, const Beading &right)
{
    float rr = 1.0 - rl;
    for (size_t i = 0; i < std::min(left.bead_widths.size(), right.bead_widths.size()); ++i) {
        if (left.bead_widths[i] != 0 && right.bead_widths[i] != 0 && sum_sensitive(rl, rr, left.bead_widths[i], right.bead_widths[i]))
            return true;
        if (sum_sensitive(rl, rr, left.toolpath_locations[i], right.toolpath_locations[i]))
            return true;
    }
    return false;
}

static double random_ratio()
{
    switch (range(0, 4)) {
    case 3: {
        // As generateSegments computes it: 1.0 - transition_ratio, a float (SkeletalTrapezoidation.cpp:1538).
        float tr = float(double(range(1, 1000000)) / 1000000.0) * float(std::ldexp(1.0, -int(range(0, 20))));
        return 1.0 - tr;
    }
    case 0: return double(next_u64() >> 1) / double(1ULL << 52); // any double in [0, 1)
    case 1: {
        // As propagateBeadingsDownward computes it: float(dist) / min(total, transition) (SkeletalTrapezoidation.cpp:1686).
        coord_t den = range(1000, 400000);
        coord_t num = range(0, den);
        return double(static_cast<float>(num) / den);
    }
    default: return double(range(0, 20)) / 20.0;
    }
}

int main(int argc, char **argv)
{
    int cases = argc > 1 ? atoi(argv[1]) : 3000;
    BeadingStrategyPtr strategy = BeadingStrategyFactory::makeStrategy(42000, 42000, 40000, float(M_PI / 18.0), true, 35700, 10500,
                                                                       0.7, 0.85, 4, 0, 1, 0.5);
    Polygons square{Polygon{Point(0, 0), Point(1000000, 0), Point(1000000, 1000000), Point(0, 1000000)}};
    SkeletalTrapezoidation st(square, *strategy, strategy->getTransitioningAngle(), 80000, 10000000, 2500, 40000, false, {});

    // 3-argument overload: random cases, then fma-sensitive ones searched for (the pin decides the result).
    int sensitive = 0, printed = 0;
    for (int attempt = 0; printed < cases && attempt < cases * 400; ++attempt) {
        Beading left  = random_beading(size_t(range(1, 6)));
        Beading right = range(0, 1) == 0 ? random_beading(size_t(range(1, 6))) : related_beading(left);
        double  rl    = random_ratio();
        bool    sens  = fma_sensitive(left, rl, right);
        if (!sens && printed >= cases / 2 && sensitive < cases / 2)
            continue; // the second half prefers the sensitive cases while they keep coming
        Beading ret = st.interpolate(left, rl, right);
        printf("i3 %a %d L", rl, int(sens));
        print_beading(left);
        printf(" R");
        print_beading(right);
        printf(" =>");
        print_beading(ret);
        printf("\n");
        sensitive += sens;
        ++printed;
    }
    // 4-argument overload (the switching radius branch re-interpolates with a float ratio + 0.1).
    for (int c = 0; c < cases; ++c) {
        Beading left  = random_beading(size_t(range(1, 6)));
        Beading right = range(0, 1) == 0 ? random_beading(left.bead_widths.size()) : related_beading(left);
        // :1741 reads right.toolpath_locations[next_inset_idx] with an index of left: same sizes.
        while (right.bead_widths.size() < left.bead_widths.size()) {
            right.bead_widths.push_back(range(20000, 90000));
            right.toolpath_locations.push_back(right.toolpath_locations.empty() ? range(0, 30000) : right.toolpath_locations.back() + range(10000, 60000));
        }
        right.bead_widths.resize(left.bead_widths.size());
        right.toolpath_locations.resize(left.bead_widths.size());
        double  rl    = random_ratio();
        coord_t sr    = range(0, left.toolpath_locations.back() + 30000);
        // Keep the cases whose re-interpolation ratio stays in [0, 1] (the pin asserts it, :1754; the port's
        // debug build checks it): the same branch as :1716-1744.
        {
            coord_t next;
            for (next = coord_t(left.toolpath_locations.size()) - 1; next >= 0; --next)
                if (sr > left.toolpath_locations[next]) break;
            if (next >= 0 && next + 1 != coord_t(left.toolpath_locations.size())) {
                Beading r3 = st.interpolate(left, rl, right);
                if (r3.toolpath_locations[next] > sr) {
                    coord_t den = left.toolpath_locations[next] - right.toolpath_locations[next];
                    if (den == 0) { --c; continue; }
                    float nr = static_cast<float>(sr - right.toolpath_locations[next]) / static_cast<float>(den);
                    if (!(nr + 0.1 >= 0.0)) { --c; continue; }
                }
            }
        }
        Beading ret   = st.interpolate(left, rl, right, sr);
        printf("i4 %a %lld L", rl, (long long) sr);
        print_beading(left);
        printf(" R");
        print_beading(right);
        printf(" =>");
        print_beading(ret);
        printf("\n");
    }
    fprintf(stderr, "i3 %d cases (%d fma-sensitive), i4 %d cases\n", printed, sensitive, cases);
    return 0;
}
