// Reference vectors for crates/libslic3r-rs/src/arachne/utils/linear_alg2d.rs, taken from
// Arachne/utils/linearAlg2D.hpp at the pin.
//
// The three functions are inline in the header; the program includes it and calls them on
// generated points, compiled as slicer_cli's libslic3r is (clang, -O3, -ffp-contract=on, the
// include paths and flags of a libslic3r source file of a configured Bambu Studio engine build
// tree). Then:
//   ./linear_alg2d_gen | gzip -9n > crates/libslic3r-rs/tests/data/linear_alg2d/vectors.txt.gz
//
// One case per line, points as "x y" (coord_t):
//   IC a b c q -> 0|1             isInsideCorner(a, b, c, q)
//   PL p a b -> n                 pointIsLeftOfLine(p, a, b)
//   AL a b c -> bits              getAngleLeft(a, b, c), the float's bits in hex
// The last cases put the points on the corner cases: collinear, coincident, axis-aligned,
// the query on a leg.

#include "libslic3r/Arachne/utils/linearAlg2D.hpp"

#include <cstdio>
#include <cstring>

using namespace Slic3r;
using namespace Slic3r::Arachne;

static uint64_t s = 0xD1B54A32D192ED03ull;
static uint64_t rnd() { s ^= s << 13; s ^= s >> 7; s ^= s << 17; return s; }
static coord_t  coord(int64_t range) { return coord_t(int64_t(rnd() % uint64_t(2 * range + 1)) - range); }
static Point    pt(int64_t range) { return Point(coord(range), coord(range)); }
static unsigned fb(float f) { unsigned u; memcpy(&u, &f, 4); return u; }

int main()
{
    const int64_t ranges[] = { 3, 50, 1000, 100000, 10000000 };
    for (int i = 0; i < 3000; ++ i) {
        int64_t r = ranges[i % 5];
        Point a = pt(r), b = pt(r), c = pt(r), q = pt(r);
        switch (i % 9) {
        case 0: c = b + (b - a); break;                        // straight
        case 1: c = a; break;                                  // pointy
        case 2: q = b + (a - b) * 2; break;                    // query on the leg ba
        case 3: q = b; break;                                  // query at the corner
        case 4: a = b; break;                                  // degenerate leg
        case 5: a = Point(b.x() + coord(r), b.y()); c = Point(b.x(), b.y() + coord(r)); break; // axis-aligned
        default: break;
        }
        printf("IC %d %d %d %d %d %d %d %d -> %d\n", a.x(), a.y(), b.x(), b.y(), c.x(), c.y(), q.x(), q.y(),
               int(LinearAlg2D::isInsideCorner(a, b, c, q.cast<int64_t>())));
        printf("PL %d %d %d %d %d %d -> %lld\n", q.x(), q.y(), a.x(), a.y(), b.x(), b.y(),
               (long long)LinearAlg2D::pointIsLeftOfLine(q, a, b));
        printf("AL %d %d %d %d %d %d -> %08x\n", a.x(), a.y(), b.x(), b.y(), c.x(), c.y(), fb(LinearAlg2D::getAngleLeft(a, b, c)));
    }
    return 0;
}
