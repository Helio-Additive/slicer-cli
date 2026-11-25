// R951: BoundingBox.hpp:265-277 `cast<Tout>(BoundingBoxBase<Tin>)` / `cast<Tout>(BoundingBox3Base<Tin>)` on
// random corners (some with min >= max on an axis). Usage: gen_cast <out.txt>.
#include <cstdio>
#include <cstdint>
#include <cmath>
#include "libslic3r/BoundingBox.hpp"

using namespace Slic3r;

static uint64_t g_seed = 0xD1B54A32D192ED03ULL;
static double rnd() { g_seed = g_seed * 6364136223846793005ULL + 1442695040888963407ULL; return double(g_seed >> 11) / double(1ULL << 53); }

int main(int argc, char **argv)
{
    if (argc < 2) { std::fprintf(stderr, "usage: gen_cast <out.txt>\n"); return 2; }
    FILE *f = std::fopen(argv[1], "w");
    for (int k = 0; k < 2000; ++k) {
        double s = (k % 3 == 0) ? 1e3 : 2e8;
        Vec3crd a(coord_t((rnd() - 0.5) * s), coord_t((rnd() - 0.5) * s), coord_t((rnd() - 0.5) * s));
        Vec3crd b(coord_t((rnd() - 0.5) * s), coord_t((rnd() - 0.5) * s), coord_t((rnd() - 0.5) * s));
        if (k % 5 == 0) b.z() = a.z();
        Vec3d ad((rnd() - 0.5) * s, (rnd() - 0.5) * s, (rnd() - 0.5) * s);
        Vec3d bd((rnd() - 0.5) * s, (rnd() - 0.5) * s, (rnd() - 0.5) * s);
        if (k % 7 == 0) bd.y() = ad.y();
        BoundingBox3 bi(a, b);
        auto ci = cast<double>(bi);
        BoundingBoxf3 bf(ad, bd);
        auto cf = cast<coord_t>(bf);
        BoundingBoxBase<Vec3crd> bb(a, b);
        auto cb = cast<double>(bb);
        std::fprintf(f, "i %d %d %d %d %d %d %d\n", int(a.x()), int(a.y()), int(a.z()), int(b.x()), int(b.y()), int(b.z()), int(bi.defined));
        std::fprintf(f, "ci %a %a %a %a %a %a %d\n", ci.min.x(), ci.min.y(), ci.min.z(), ci.max.x(), ci.max.y(), ci.max.z(), int(ci.defined));
        std::fprintf(f, "d %a %a %a %a %a %a %d\n", ad.x(), ad.y(), ad.z(), bd.x(), bd.y(), bd.z(), int(bf.defined));
        std::fprintf(f, "cf %d %d %d %d %d %d %d\n", int(cf.min.x()), int(cf.min.y()), int(cf.min.z()), int(cf.max.x()), int(cf.max.y()), int(cf.max.z()), int(cf.defined));
        std::fprintf(f, "cb %a %a %a %a %a %a %d\n", cb.min.x(), cb.min.y(), cb.min.z(), cb.max.x(), cb.max.y(), cb.max.z(), int(cb.defined));
    }
    std::fclose(f);
    return 0;
}
