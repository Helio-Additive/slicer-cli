// Reproduce orders.txt with clang++ -std=c++17 -stdlib=libc++ gen.cpp -o gen.
// Clipper's value/comparator combination selects libc++'s non-bitset introsort.
#include <algorithm>
#include <cstdint>
#include <iostream>
#include <vector>
#ifndef _LIBCPP_VERSION
#error This generator compares against libc++.
#endif
struct Item { uint32_t key; size_t index; };
int main()
{
    auto less = [](const Item& a, const Item& b) { return a.key < b.key; };
    for (size_t n : {0, 1, 2, 5, 22, 23, 24, 25, 127, 128, 129, 257}) {
        for (uint32_t distinct : {1, 3, 17}) {
            std::vector<Item> items;
            uint32_t state = 17;
            for (size_t i = 0; i < n; ++i) {
                state = state * 1664525u + 1013904223u;
                items.push_back({(state >> 16) % distinct, i});
            }
            std::sort(items.begin(), items.end(), less);
            std::cout << n << ' ' << distinct;
            for (const auto& item : items) std::cout << ' ' << item.index;
            std::cout << '\n';
        }
    }
}
