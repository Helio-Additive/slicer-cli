// BambuStudio 56a57d0 src/nanosvg/nanosvg.h 1029-1078,1990-2410 and
// libslic3r/NSVGUtils.cpp 325-374. Compile with the Bambu target's include
// paths and libraries and redirect stdout to linear.txt.
#define NANOSVG_IMPLEMENTATION
#include "libslic3r/NSVGUtils.cpp"
#include "nanosvg.h"
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
static void bits(float x) {
    uint32_t u;
    std::memcpy(&u, &x, sizeof u);
    std::printf(" %08x", u);
}
int main() {
    const char* cases[] = {"",
                           "M 5 6",
                           "L 5 6",
                           "M0 0 5 0 5 3z",
                           "m1 2 3 4 5 -2 h4 v3 H-3 V0",
                           "M-2 1 C0 5 4 -3 6 1 s8 2 10 1",
                           "M-2 1 Q0 5 4 -3 t8 2 T20 -3",
                           "M0 0 A7 2 25 0 0 10 5",
                           "M0 0 A7 2 25 0 1 10 5",
                           "M0 0 A7 2 25 1 0 10 5",
                           "M0 0 a7 2 25 1 1 10 5",
                           "M0 0 A5 5 0 0110 0",
                           "M1 1 a0 1 0 0 1 2 3",
                           "M2 1 A7 8 90 1 0 2 1",
                           "M1 1 C2 3 4 5 6 7 Z l3 0",
                           "M0 0L1 1 M4 6h2z M12 14v5",
                           "M.5-.25L1e-1,2E+1",
                           "M1 2 C3 4 L4 5",
                           "M2 3 Z Z",
                           "M1 1S3 4 5 6T7 8",
                           "M-2 -3q1 5 7 9s2 3 6 7"};
    for (const char* value : cases) {
        NSVGparser* p = nsvg__createParser();
        if (!p)
            return 1;
        const char* attrs[] = {"d", value, nullptr};
        nsvg__parsePath(p, attrs);
        std::printf("case%s%s\n", *value ? " " : "", value);
        for (NSVGshape* shape = p->image->shapes; shape; shape = shape->next) {
            for (auto* path = shape->paths; path; path = path->next) {
                std::printf("path %d %d", path->closed, path->npts);
                for (int i = 0; i < path->npts * 2; ++i)
                    bits(path->pts[i]);
                std::puts("");
            }
            for (int test = 0; test < 3; ++test) {
                Slic3r::NSVGLineParams params(test == 0 ? 0.15 : 2.0);
                params.max_level = test == 2 ? 2 : 9;
                params.scale = test == 0 ? 1.0 : 128;
                params.is_y_negative = test == 1;
                const auto lines = linearize_path(shape->paths, params);
                std::printf("lines %d %zu %zu\n", test, lines.polygons.size(),
                            lines.polylines.size());
                for (const auto& polygon : lines.polygons) {
                    std::printf("polygon");
                    for (const auto& pt : polygon.points)
                        std::printf(" %d,%d", int(pt.x()), int(pt.y()));
                    std::puts("");
                }
                for (const auto& polyline : lines.polylines) {
                    std::printf("polyline");
                    for (const auto& pt : polyline.points)
                        std::printf(" %d,%d", int(pt.x()), int(pt.y()));
                    std::puts("");
                }
            }
        }
        std::puts("end");
        nsvg__deleteParser(p);
    }
}
