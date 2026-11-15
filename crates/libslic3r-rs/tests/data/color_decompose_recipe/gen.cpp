// Reference vectors of tests/color_decompose_recipe_native_parity.rs: Bambu Studio's
// ColorDecomposeRecipe.cpp at the pin, called with deterministic random inputs, over the table of
// standard recipes the pin ships (resources/filament_mixing/standard_color_recipes.json).
//
// Build, from an empty directory, with PIN = references/BambuStudio of this repository and DATA =
// this directory:
//
//   cp $PIN/src/libslic3r/ColorDecomposeRecipe.cpp $PIN/src/libslic3r/ColorDecomposeRecipe.hpp .
//   printf '#include <string>\nnamespace Slic3r { const std::string& resources_dir(); }\n' > Utils.hpp
//   clang++ -std=c++17 -O2 -ffp-contract=on -I. -I$PIN/src -I$PIN/src/libslic3r \
//     $DATA/gen.cpp ColorDecomposeRecipe.cpp -o gen
//   ./gen $PIN/resources > $DATA/vectors.txt
//
// ColorDecomposeRecipe.cpp is compiled unchanged. Utils.hpp stands in for the header that declares
// resources_dir(); FilamentMixer.hpp and FilamentMixerModel.hpp are the pin's. Of FilamentMixer.cpp
// only blend_color_multi and what it calls are needed; they are copied below, verbatim.
//
// Format of vectors.txt: one call per line, a tag, the arguments, `->` and the result. A string is
// `~` followed by the text, with '%' and every byte outside the printable ASCII characters
// percent-encoded.

#include "ColorDecomposeRecipe.hpp"
#include "FilamentMixer.hpp"
#include "FilamentMixerModel.hpp"

#include <cassert>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

namespace Slic3r {

static std::string g_resources_dir;
const std::string& resources_dir() { return g_resources_dir; }

// FilamentMixer.cpp:47-53, verbatim
void filament_mixer_lerp(unsigned char r1, unsigned char g1, unsigned char b1,
                         unsigned char r2, unsigned char g2, unsigned char b2,
                         float t,
                         unsigned char* out_r, unsigned char* out_g, unsigned char* out_b)
{
    ::filament_mixer::lerp(r1, g1, b1, r2, g2, b2, t, out_r, out_g, out_b);
}

// FilamentMixer.cpp:91-98, verbatim
static bool parse_hex(const std::string &hex, unsigned char &r, unsigned char &g, unsigned char &b)
{
    if (hex.size() < 7 || hex[0] != '#') return false;
    unsigned rv = 0, gv = 0, bv = 0;
    if (std::sscanf(hex.c_str(), "#%02x%02x%02x", &rv, &gv, &bv) != 3) return false;
    r = (unsigned char)rv; g = (unsigned char)gv; b = (unsigned char)bv;
    return true;
}

// FilamentMixer.cpp:116-162, verbatim
std::string blend_color_multi(const std::vector<std::string> &hex_colors,
                              const std::vector<int> &weights)
{
    if (hex_colors.size() >= 2 && hex_colors.size() == weights.size()) {
        std::string measured = lookup_measured_blend_color(hex_colors, weights);
        if (!measured.empty())
            return measured;
    }

    if (hex_colors.empty())
        return "#000000";
    if (hex_colors.size() == 1) {
        unsigned char cr = 128, cg = 128, cb = 128;
        parse_hex(hex_colors.front(), cr, cg, cb);
        char buf[8];
        std::snprintf(buf, sizeof(buf), "#%02X%02X%02X", cr, cg, cb);
        return std::string(buf);
    }

    assert(hex_colors.size() == weights.size());

    unsigned char r = 128, g = 128, b = 128;
    int accumulated = 0;

    for (size_t i = 0; i < hex_colors.size() && i < weights.size(); ++i) {
        if (weights[i] <= 0)
            continue;
        unsigned char cr = 128, cg = 128, cb = 128;
        parse_hex(hex_colors[i], cr, cg, cb);
        if (accumulated == 0) {
            r = cr; g = cg; b = cb;
            accumulated = weights[i];
        } else {
            const int new_total = accumulated + weights[i];
            const float t = static_cast<float>(weights[i]) / static_cast<float>(new_total);
            filament_mixer_lerp(r, g, b, cr, cg, cb, t, &r, &g, &b);
            accumulated = new_total;
        }
    }

    if (accumulated == 0)
        return "#000000";

    char buf[8];
    std::snprintf(buf, sizeof(buf), "#%02X%02X%02X", r, g, b);
    return std::string(buf);
}

} // namespace Slic3r

using namespace Slic3r;

static uint64_t g_seed = 0x2545F4914F6CDD1Dull;
static uint64_t rnd()
{
    g_seed ^= g_seed << 13;
    g_seed ^= g_seed >> 7;
    g_seed ^= g_seed << 17;
    return g_seed;
}
static int  rint_(int n) { return int(rnd() % uint64_t(n)); } // 0 .. n-1
static bool chance(int percent) { return rint_(100) < percent; }
template<class T, size_t N> static T pick(const T (&items)[N]) { return items[rint_(int(N))]; }

static std::string tok(const std::string &s)
{
    std::string out = "~";
    for (unsigned char c : s) {
        if (c <= ' ' || c == '%' || c >= 0x7f) {
            char buf[4];
            std::snprintf(buf, sizeof(buf), "%%%02X", c);
            out += buf;
        } else
            out += char(c);
    }
    return out;
}

// The component colors of the table, by its three sets.
static const char *const k_cmyw[]  = {"#0086D6", "#EC008C", "#F4EE2A", "#FFFFFF"};
static const char *const k_rybw[]  = {"#C12E1F", "#F4EE2A", "#0A2989", "#FFFFFF"};
static const char *const k_cmykw[] = {"#0090DA", "#E24585", "#FCE300", "#000000", "#FFFFFF"};
static const char *const k_materials[] = {"", "PLA", "PLA Basic", "PLA Palette", "PETG", "PETG Basic", "ABS", "PLA Matte", "PLA  Basic", " "};

static std::string rnd_hex()
{
    char buf[8];
    std::snprintf(buf, sizeof(buf), chance(80) ? "#%02X%02X%02X" : "#%02x%02x%02x", rint_(256), rint_(256), rint_(256));
    return buf;
}

static std::string lower(std::string s)
{
    for (char &c : s)
        if (c >= 'A' && c <= 'F') c = char(c - 'A' + 'a');
    return s;
}

// A color of the table (sometimes in lower case or with an alpha suffix), or any color.
static std::string table_hex(int set)
{
    std::string hex = set == 0 ? pick(k_cmyw) : set == 1 ? pick(k_rybw) : pick(k_cmykw);
    if (chance(15)) hex = lower(hex);
    if (chance(8)) hex += "FF";
    return hex;
}

static void print_result(const ColorDecomposeRecipeResult &r)
{
    std::printf("%d %d %s %zu", r.valid ? 1 : 0, int(r.mode), tok(r.matched_color_hex).c_str(), r.components.size());
    for (const auto &c : r.components)
        std::printf(" %s %s %d %u", tok(c.color_hex).c_str(), tok(c.base_color).c_str(), c.ratio, c.filament_index);
    std::printf("\n");
}

int main(int argc, char **argv)
{
    if (argc < 2) {
        std::fprintf(stderr, "usage: gen <resources dir of the pin>\n");
        return 1;
    }
    g_resources_dir = argv[1];

    // color_decompose_hex_to_rgb: what sscanf takes for a color, on a value that shows a result left alone
    static const char *const k_odd[] = {
        "", "#", "#12345", "#123456", "#abcdef", "#ABCDEF", "#AbCdEf", "#1234567", "#12345678", "123456#", "0123456", "#GG0000", "#00GG00",
        "#0000GG", "#0000G0", "#00000G", "# 1 2 3 4", "#  12  34  56", "#1 2 3   ", "#1 22 33", "#\t1\n2\r3  ", "#-1-2-3 ", "#+1+2+3 ", "#-1+2-3x",
        "#0x0x0x0", "#0x12345", "#000x000", "#0X1 2 3", "#-0x1234", "#x123456", "#1x23456", "#12x3456", "#1234x56", "#--12345", "#+-12345", "#- 12345",
        "#-a-B-c ", "#-ff-00-1", "#0 0 0  ", "#000000", "#ffffff", "#FFFFFF", "#00 0000", "#0000 00", "#000 000", "#0 00000", "#       ", "#      1",
        "#     12", "#    123", "#1      ", "#12     ", "#123    ", "#1234   ", "#12345  ", "#123456 ", " #123456", "#12345g", "#1234g6", "#1g3456",
        "#g23456", "#12 3 45", "#+0+0+0 ", "#-0-0-0 ", "#0-1-2-3", "#1-2-3-4", "#+a+b+c+", "#0xa0xb0", "#00x1234", "#-x12345", "#1 0x2 3", "#12 0x 3",
        "#12 0x3 4", "#12 34 0x", "#12 34 0", "#12 34 -", "#12 34 +5", "#12 34 +", "#1.2.3.4", "#1,2,3,4", "#12:34:56", "#ÿÿÿÿ", "#12345ÿ"};
    for (const char *s : k_odd) {
        ColorDecomposeRgb rgb{1, 2, 3};
        const bool ok = color_decompose_hex_to_rgb(s, rgb);
        std::printf("HEX %s -> %d %d %d %d\n", tok(s).c_str(), ok ? 1 : 0, rgb.r, rgb.g, rgb.b);
    }
    for (int i = 0; i < 60; ++i) {
        const std::string s = chance(50) ? rnd_hex() : table_hex(rint_(3));
        ColorDecomposeRgb rgb{1, 2, 3};
        const bool ok = color_decompose_hex_to_rgb(s, rgb);
        std::printf("HEX %s -> %d %d %d %d\n", tok(s).c_str(), ok ? 1 : 0, rgb.r, rgb.g, rgb.b);
        std::printf("RGB %d %d %d -> %s\n", rgb.r, rgb.g, rgb.b, tok(color_decompose_rgb_to_hex(rgb)).c_str());
    }

    // color_decompose_material_family_matches
    for (const char *a : k_materials)
        for (const char *b : k_materials)
            std::printf("FAM %s %s -> %d\n", tok(a).c_str(), tok(b).c_str(), color_decompose_material_family_matches(a, b) ? 1 : 0);

    // lookup_standard_recipe
    for (int i = 0; i < 420; ++i) {
        const ColorDecomposeRgb target{(unsigned char) rint_(256), (unsigned char) rint_(256), (unsigned char) rint_(256)};
        const int               mode     = rint_(4);
        const std::string       material = pick(k_materials);
        std::printf("STD %d %d %d %d %s -> ", target.r, target.g, target.b, mode, tok(material).c_str());
        print_result(lookup_standard_recipe(target, ColorDecomposeRecipeMode(mode), material));
    }

    // lookup_measured_blend_color and blend_color_multi: components of one set of the table, or any colors
    for (int i = 0; i < 1500; ++i) {
        const int                set = rint_(3);
        const int                n   = chance(45) ? 2 : chance(80) ? 3 : 1 + rint_(5);
        std::vector<std::string> hexes;
        for (int k = 0; k < n; ++k) {
            // distinct colors mostly: a set of the table has no color twice
            std::string hex;
            for (int attempt = 0; attempt < 8; ++attempt) {
                hex = chance(88) ? table_hex(set) : chance(70) ? rnd_hex() : std::string(pick(k_odd));
                bool repeated = false;
                for (const std::string &other : hexes) repeated = repeated || other.substr(0, 7) == hex.substr(0, 7);
                if (!repeated || chance(5))
                    break;
            }
            hexes.push_back(hex);
        }
        std::vector<int> ratios;
        const int        kind = rint_(100);
        int              left = 100;
        for (int k = 0; k < n; ++k) {
            int r;
            if (kind < 30) { // on the grid of the table
                r = k + 1 == n ? left : 5 * (1 + rint_(std::max(1, (left - 5 * (n - k - 1)) / 5)));
                left -= r;
            } else if (kind < 60) { // any split of 100
                r = k + 1 == n ? left : rint_(left + 1);
                left -= r;
            } else if (kind < 80) // any weights
                r = rint_(1000);
            else if (kind < 90) // the dialog's weights: ratio * 10000
                r = rint_(101) * 10000;
            else
                r = rint_(60) - 10;
            ratios.push_back(r);
        }
        std::string args = std::to_string(n);
        for (int k = 0; k < n; ++k) args += " " + tok(hexes[size_t(k)]) + " " + std::to_string(ratios[size_t(k)]);
        std::printf("MEAS %s -> %s\n", args.c_str(), tok(lookup_measured_blend_color(hexes, ratios)).c_str());
        std::printf("BLEND %s -> %s\n", args.c_str(), tok(blend_color_multi(hexes, ratios)).c_str());
    }

    // recommend_from_physical_filaments
    static const char *const k_types[] = {"PLA", "PLA", "PLA Basic", "PLA Palette", "PETG", "ABS", "", "PLA Matte"};
    for (int i = 0; i < 260; ++i) {
        ColorDecomposeRgb target{(unsigned char) rint_(256), (unsigned char) rint_(256), (unsigned char) rint_(256)};
        const int         set = rint_(3);
        const int         n   = chance(25) ? 9 + rint_(6) : rint_(9);
        std::vector<ColorDecomposePhysicalFilament> filaments;
        for (int k = 0; k < n; ++k) {
            ColorDecomposePhysicalFilament f;
            const int r = rint_(100);
            if (r < 45)
                f.color_hex = table_hex(set);
            else if (r < 85)
                f.color_hex = rnd_hex();
            else if (r < 90)
                f.color_hex = pick(k_odd);
            else if (r < 95 && !filaments.empty())
                f.color_hex = filaments[size_t(rint_(int(filaments.size())))].color_hex;
            else {
                // a few colors at the same distance from the target: the same color with another last digit
                f.color_hex = rnd_hex();
                f.color_hex[6] = "0123456789ABCDEF"[rint_(16)];
            }
            f.name           = "F" + std::to_string(k);
            f.type           = pick(k_types);
            f.is_mixed       = chance(10);
            f.filament_index = chance(90) ? unsigned(k + 1) : unsigned(rint_(20));
            filaments.push_back(f);
        }
        if (chance(8) && !filaments.empty()) {
            // the target is one of the filaments
            ColorDecomposeRgb rgb;
            if (color_decompose_hex_to_rgb(filaments[size_t(rint_(int(filaments.size())))].color_hex, rgb))
                target = rgb;
        }
        const std::string preferred = chance(40) ? std::string() : std::string(pick(k_types));
        std::printf("REC %d %d %d %s %zu", target.r, target.g, target.b, tok(preferred).c_str(), filaments.size());
        for (const auto &f : filaments)
            std::printf(" %s %s %s %d %u", tok(f.color_hex).c_str(), tok(f.name).c_str(), tok(f.type).c_str(), f.is_mixed ? 1 : 0, f.filament_index);
        std::printf(" -> ");
        print_result(recommend_from_physical_filaments(target, filaments, preferred));
    }
    return 0;
}
