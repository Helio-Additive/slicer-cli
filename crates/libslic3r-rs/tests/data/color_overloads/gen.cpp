// Native vectors for Color.cpp's ColorRGB / ColorRGBA overloads (Color.cpp:306-454): complementary, saturate,
// decode_color, decode_colors, encode_color. Deterministic inputs from an LCG; floats printed with %a.
//
//   gen > vectors.txt
//
// Build with the engine's compile and link line (target/parity-tools/build.sh in a checkout).
#include <cstdio>
#include <cstdint>
#include <string>
#include <vector>
#include "libslic3r/Color.hpp"

using namespace Slic3r;

static uint64_t g_state = 0x2545F4914F6CDD1DULL;
static uint32_t lcg() { g_state = g_state * 6364136223846793005ULL + 1442695040888963407ULL; return uint32_t(g_state >> 33); }
static float unit() { return float(lcg() % 100001) / 100000.0f; }

int main() {
    for (int k = 0; k < 300; ++k) {
        float r = unit(), g = unit(), b = unit(), a = unit();
        if (k % 17 == 0) { g = r; b = r; }           // grey
        if (k % 23 == 0) { r = 1.0f; g = 0.0f; }      // hue edges
        float factor = float(lcg() % 3001) / 1000.0f; // 0 .. 3
        ColorRGB c3(r, g, b);
        ColorRGBA c4(r, g, b, a);
        ColorRGB cc3 = complementary(c3);
        ColorRGBA cc4 = complementary(c4);
        ColorRGB s3 = saturate(c3, factor);
        ColorRGBA s4 = saturate(c4, factor);
        printf("c %a %a %a %a %a | %a %a %a | %a %a %a %a | %a %a %a | %a %a %a %a | %s %s\n", r, g, b, a, factor,
               cc3.r(), cc3.g(), cc3.b(), cc4.r(), cc4.g(), cc4.b(), cc4.a(), s3.r(), s3.g(), s3.b(), s4.r(), s4.g(), s4.b(), s4.a(),
               encode_color(c3).c_str(), encode_color(c4).c_str());
    }
    std::vector<std::string> strs = {"#FF8000", "#ff8000", "#00000000", "#12345678", "#ABCDEF", "#abcdeF80", "FF8000", "#FF800",
                                     "#FF80000", "#GG8000", "#FF80ZZ", "#1G2H3I4J", "", "#", "#FFFFFFFF", "#000000"};
    const char *hex = "0123456789abcdefABCDEFxg";
    for (int k = 0; k < 120; ++k) {
        int len = (k % 3 == 0) ? 7 : (k % 3 == 1) ? 9 : int(lcg() % 11);
        std::string s = "#";
        for (int i = 1; i < len; ++i) s.push_back(hex[lcg() % 24]);
        if (k % 29 == 0 && !s.empty()) s[0] = '0';
        strs.push_back(s);
    }
    for (const std::string &s : strs) {
        ColorRGB o3(0.25f, 0.5f, 0.75f);
        ColorRGBA o4(0.25f, 0.5f, 0.75f, 0.125f);
        bool b3 = decode_color(s, o3);
        bool b4 = decode_color(s, o4);
        printf("d [%s] %d %a %a %a %d %a %a %a %a\n", s.c_str(), (int) b3, o3.r(), o3.g(), o3.b(), (int) b4, o4.r(), o4.g(), o4.b(), o4.a());
    }
    // decode_colors over groups of four consecutive strings (a failure returns early with the rest BLACK)
    for (size_t k = 0; k + 4 <= strs.size(); k += 3) {
        std::vector<std::string> in(strs.begin() + k, strs.begin() + k + 4);
        std::vector<ColorRGB> o3(2, ColorRGB(0.5f, 0.5f, 0.5f));
        std::vector<ColorRGBA> o4(5, ColorRGBA(0.5f, 0.5f, 0.5f, 0.5f));
        bool b3 = decode_colors(in, o3);
        bool b4 = decode_colors(in, o4);
        printf("ds %zu %d %zu", k, (int) b3, o3.size());
        for (const auto &c : o3) printf(" %a %a %a", c.r(), c.g(), c.b());
        printf(" | %d %zu", (int) b4, o4.size());
        for (const auto &c : o4) printf(" %a %a %a %a", c.r(), c.g(), c.b(), c.a());
        printf("\n");
    }
    return 0;
}
