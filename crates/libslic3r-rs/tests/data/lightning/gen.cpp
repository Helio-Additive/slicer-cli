// Generator of the vectors of tests/lightning_native_parity.rs: what Fill/Lightning of libslic3r takes from the
// C++ library and the Rust port has to reproduce.
//
// Build with the compile line the build uses for a libslic3r source (its defines and include directories,
// -std=c++17 -O3 -DNDEBUG) and run `./gen vectors.txt`.
//
//  - `rand()` after `srand(1)` (TreeNode.cpp:365 `rand() % m_children.size()`): the stream of the C library.
//  - `std::stable_sort` with a comparator that is not a strict weak ordering (DistanceField.cpp:74-79 sorts the
//    unsupported points with one): the order the algorithm of the C++ library leaves. The elements are Points,
//    which are not trivially copy assignable, as the cells of the distance field are not.
//
// The vectors were made on macOS arm64 with Apple clang (libc++, Libc).

#include "libslic3r/Point.hpp"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <type_traits>
#include <vector>

using namespace Slic3r;

int main(int argc, char **argv)
{
    FILE *out = argc > 1 ? std::fopen(argv[1], "w") : stdout;
    if (out == nullptr)
        return 1;
    std::fprintf(out, "traits %d %d\n", int(sizeof(coord_t)), int(std::is_trivially_copy_assignable<Point>::value));
    srand(1);
    std::fprintf(out, "rand 1");
    for (int i = 0; i < 40; ++i) std::fprintf(out, " %d", rand());
    std::fprintf(out, "\n");
    srand(0);
    std::fprintf(out, "rand 0");
    for (int i = 0; i < 8; ++i) std::fprintf(out, " %d", rand());
    std::fprintf(out, "\n");
    srand(20240229);
    std::fprintf(out, "rand 20240229");
    for (int i = 0; i < 8; ++i) std::fprintf(out, " %d", rand());
    std::fprintf(out, "\n");
    for (int n : {0, 1, 2, 3, 4, 5, 7, 8, 9, 15, 16, 17, 18, 31, 33, 64, 100, 129, 257, 300, 1000}) {
        for (int kind = 0; kind < 2; ++kind) {
            std::vector<Point> v;
            unsigned long long s = 12345ULL + (unsigned long long) n * 2 + kind;
            for (int i = 0; i < n; ++i) {
                s     = s * 6364136223846793005ULL + 1442695040888963407ULL;
                int a = int((s >> 33) % 50);
                s     = s * 6364136223846793005ULL + 1442695040888963407ULL;
                int b = int((s >> 33) % 50);
                v.emplace_back(a, b);
            }
            if (kind == 0)
                // the shape of DistanceField.cpp:74-79: two rules that disagree
                std::stable_sort(v.begin(), v.end(), [](const Point &a, const Point &b) {
                    return std::abs(b.x() - a.x()) > 10 ? a.x() < b.x() : (a.y() % 7) < (b.y() % 7);
                });
            else
                // a strict weak ordering with many equal keys: the stability
                std::stable_sort(v.begin(), v.end(), [](const Point &a, const Point &b) { return a.x() / 5 < b.x() / 5; });
            std::fprintf(out, "ss %d %d", n, kind);
            for (const Point &p : v) std::fprintf(out, " %d %d", int(p.x()), int(p.y()));
            std::fprintf(out, "\n");
        }
    }
    if (out != stdout)
        std::fclose(out);
    return 0;
}
