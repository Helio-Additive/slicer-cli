// Native vectors for tests/bounding_box_cpp_names_native_parity.rs: BoundingBox.hpp:248-263, empty() of the
// four box types, scaled(BoundingBoxf / BoundingBoxf3), unscaled(BoundingBox / BoundingBox3).
#include "libslic3r/BoundingBox.hpp"
#include <cstdio>
#include <cstring>
#include <cstdint>

using namespace Slic3r;

static uint64_t g_s = 0xA0761D6478BD642FULL;
static uint32_t rnd() { g_s = g_s * 6364136223846793005ULL + 1442695040888963407ULL; return uint32_t(g_s >> 33); }
static int32_t ri(int32_t lo, int32_t hi) { return lo + int32_t(rnd() % uint32_t(hi - lo + 1)); }
static double rd(double lo, double hi) { return lo + (hi - lo) * (double(rnd()) / 2147483648.0); }
static unsigned long long db(double d) { unsigned long long u; std::memcpy(&u, &d, 8); return u; }

int main()
{
    for (int i = 0; i < 4000; ++i) {
        // a degenerate axis in some cases
        double a[6]; int32_t c[6];
        for (int k = 0; k < 6; ++k) { a[k] = (i % 7 == 0) ? double(ri(-200, 200)) / 8. : rd(-900., 900.); c[k] = ri(-90000000, 90000000); }
        if (i % 5 == 0) { a[3] = a[0]; c[3] = c[0]; }
        if (i % 11 == 0) { a[5] = a[2]; c[5] = c[2]; }
        BoundingBoxf bf(Vec2d(a[0], a[1]), Vec2d(a[3], a[4]));
        BoundingBoxf3 bf3(Vec3d(a[0], a[1], a[2]), Vec3d(a[3], a[4], a[5]));
        BoundingBox bi(Point(c[0], c[1]), Point(c[3], c[4]));
        BoundingBox3 bi3(Vec3crd(c[0], c[1], c[2]), Vec3crd(c[3], c[4], c[5]));
        if (i % 13 == 0) { bf.defined = true; bf3.defined = true; bi.defined = true; bi3.defined = true; }
        BoundingBox s2 = scaled(bf);
        BoundingBox3 s3 = scaled(bf3);
        BoundingBoxf u2 = unscaled(bi);
        BoundingBoxf3 u3 = unscaled(bi3);
        printf("BB");
        for (int k = 0; k < 6; ++k) printf(" %016llx", db(a[k]));
        for (int k = 0; k < 6; ++k) printf(" %d", c[k]);
        printf(" %d %d %d %d", int(bf.defined), int(bf3.defined), int(bi.defined), int(bi3.defined));
        printf(" | %d %d %d %d", int(empty(bf)), int(empty(bf3)), int(empty(bi)), int(empty(bi3)));
        printf(" %d %d %d %d %d", s2.min.x(), s2.min.y(), s2.max.x(), s2.max.y(), int(s2.defined));
        printf(" %d %d %d %d %d %d %d", s3.min.x(), s3.min.y(), s3.min.z(), s3.max.x(), s3.max.y(), s3.max.z(), int(s3.defined));
        printf(" %016llx %016llx %016llx %016llx %d", db(u2.min.x()), db(u2.min.y()), db(u2.max.x()), db(u2.max.y()), int(u2.defined));
        printf(" %016llx %016llx %016llx %016llx %016llx %016llx %d\n", db(u3.min.x()), db(u3.min.y()), db(u3.min.z()),
               db(u3.max.x()), db(u3.max.y()), db(u3.max.z()), int(u3.defined));
    }
    return 0;
}
