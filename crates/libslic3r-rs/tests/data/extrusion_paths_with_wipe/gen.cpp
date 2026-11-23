// Native vectors for extrusion_entities_append_paths_with_wipe (ExtrusionEntity.hpp:671-724).
// Each case: width, gap compensation ratio, the input polylines; then the entities the pin
// appends (each an ExtrusionMultiPath: per path the no_extrusion flag and the points).
#include <cstdint>
#include <cstdio>
#include "libslic3r/ExtrusionEntity.hpp"
using namespace Slic3r;
static uint64_t s = 0x2545F4914F6CDD1Dull;
static uint32_t lcg() { s = s * 6364136223846793005ull + 1442695040888963407ull; return uint32_t(s >> 33); }
int main() {
    const float widths[] = {0.42f, 0.45f, 0.4f, 0.62f, 0.38f};
    const float ratios[] = {0.f, 0.45f, 0.3f, 0.75f};
    for (int c = 0; c < 400; ++c) {
        float w = widths[lcg() % 5];
        float r = ratios[lcg() % 4];
        int n = 1 + lcg() % 8;
        Polylines pls;
        coord_t x = coord_t(lcg() % 1000000), y = coord_t(lcg() % 1000000);
        for (int i = 0; i < n; ++i) {
            Polyline pl;
            int np = 2 + lcg() % 2;
            // a hop from the last end: mostly near 3 * width
            coord_t hx = coord_t(lcg() % 200000) - 50000, hy = coord_t(lcg() % 160000) - 20000;
            x += hx; y += hy;
            for (int k = 0; k < np; ++k) {
                pl.points.emplace_back(x, y);
                x += coord_t(lcg() % 2000000) - 1000000; y += coord_t(lcg() % 400000) - 200000;
            }
            if (lcg() % 13 == 0) pl.points.resize(1); // invalid polyline
            pls.push_back(pl);
        }
        printf("C %a %a %d", w, r, (int)pls.size());
        for (auto &pl : pls) { printf(" %d", (int)pl.points.size()); for (auto &p : pl.points) printf(" %lld %lld", (long long)p.x(), (long long)p.y()); }
        printf("\n");
        ExtrusionEntitiesPtr dst;
        extrusion_entities_append_paths_with_wipe(dst, std::move(pls), erTopSolidInfill, 0.02, w, 0.2f, 0.4f, r);
        printf("R %d", (int)dst.size());
        for (auto *e : dst) {
            auto *mp = dynamic_cast<ExtrusionMultiPath*>(e);
            printf(" M %d", (int)mp->paths.size());
            for (auto &p : mp->paths) { printf(" %d %d", p.is_force_no_extrusion() ? 1 : 0, (int)p.polyline.points.size()); for (auto &q : p.polyline.points) printf(" %lld %lld", (long long)q.x(), (long long)q.y()); }
        }
        printf("\n");
        for (auto *e : dst) delete e;
    }
    return 0;
}
