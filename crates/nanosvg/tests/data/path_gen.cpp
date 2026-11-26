// BambuStudio 56a57d0 src/nanosvg/nanosvg.h 230-232,540-616,721-733,1029-1078.
// clang++ -std=c++17 -O3 -DNDEBUG -Ireferences/BambuStudio/src/nanosvg path_gen.cpp -o gen
// Capture ./gen on arm64 (paths-fused.txt) and baseline x86_64 (paths-unfused.txt).
#define NANOSVG_IMPLEMENTATION
#include "nanosvg.h"
#include <array>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>
static void bits(float v) {
    uint32_t word;
    std::memcpy(&word, &v, sizeof word);
    std::printf(" %08x", word);
}
int main() {
    using Point = std::array<float, 2>;
    std::vector<std::vector<Point>> cases = {
        {},
        {{1, 2}},
        {{0, 0}, {1, 1}, {2, 2}},
        {{0, 0}, {1, 1}, {2, 2}, {3, 3}, {4, 4}},
        {{-1, 2}, {0, 3}, {2, 4}, {3, 5}},
        {{0, 0}, {2, 12}, {-3, -8}, {4, 1}},
        {{1, 0}, {2, 7}, {3, 7}, {4, 0}},
        {{4, 1}, {3, 6}, {-5, 4}, {2, 2}, {2, 2}, {2, 2}, {2, 2}}};
    cases.push_back({{-0.f, 1.f}, {0.f, -0.f}, {-0.f, 0.f}, {0.f, -1.f}});
    cases.push_back({{0.f, 0.f}, {-0.f, -0.f}, {0.f, -0.f}, {-0.f, 0.f}});
    cases.push_back({{0.f, 1.f}, {0.f, -1.e-25f}, {0.f, 1.e-25f}, {0.f, -1.f}});
    uint32_t state = 56;
    auto next = [&] {
        state = state * 1664525u + 1013904223u;
        return (float(int(state >> 8) - 0x800000)) / 509003.0f;
    };
    for (int i = 0; i < 32; ++i) {
        std::vector<Point> points(1 + 3 * (1 + i % 4));
        for (auto& pt : points)
            pt = {next(), next()};
        cases.push_back(points);
    }
    const std::array<std::array<float, 6>, 4> transforms = {
        {{{1, 0, 0, 1, 0, 0}},
         {{-0.f, -0.f, -0.f, -0.f, -0.f, -0.f}},
         {{0, -1, 2, 0, 3, -7}},
         {{0.738241f, -0.549827f, 0.28391f, 1.01983f, -5.24929f, 9.3831f}}}};
    for (const auto& points : cases) {
        for (auto transform : transforms)
            for (bool closed : {false, true}) {
                auto* p = nsvg__createParser();
                for (auto pt : points)
                    nsvg__addPoint(p, pt[0], pt[1]);
                std::memcpy(nsvg__getAttr(p)->xform, transform.data(), 6 * sizeof(float));
                std::printf("input %d %zu", closed, points.size());
                for (float v : transform)
                    bits(v);
                for (auto pt : points)
                    for (float v : pt)
                        bits(v);
                std::puts("");
                nsvg__addPath(p, closed);
                if (p->plist) {
                    auto* r = p->plist;
                    std::printf("path %d %d", r->closed, r->npts);
                    for (float v : r->bounds)
                        bits(v);
                    for (int i = 0; i < r->npts * 2; ++i)
                        bits(r->pts[i]);
                    std::puts("");
                } else
                    std::puts("none");
                nsvg__deleteParser(p);
            }
    }
}
