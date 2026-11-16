// Reference vectors for crates/libslic3r-rs/src/utils.rs, taken from utils.cpp / Utils.hpp at the pin.
//
// The program links the engine's libslic3r (utils.cpp as slicer_cli compiles it, with
// BBL_RELEASE_TO_PUBLIC=1) and calls PathSanitizer::sanitize, split_string and bbl_calc_md5 on
// generated inputs. PathSanitizer reads the user profile from HOME once (the macOS branch of
// utils.cpp:952-958, the reference build), so the program sets HOME from its first argument and is
// run once per profile. Build it as a libslic3r test program of a configured Bambu Studio engine
// build tree, then, with a scratch directory for the md5 files:
//   for h in /Users/Tester /home/ab "/Users/T\xc3\xabster"; do ./utils_gen "$h" "$scratch"; done \
//     | gzip -9n > crates/libslic3r-rs/tests/data/utils/vectors.txt.gz
//
// One case per line, strings as the hex of their bytes ("-" when empty):
//   SAN home raw -> out                 PathSanitizer::sanitize(raw) with HOME = home
//   SPLIT delim str -> n s1 s2 ...      split_string(str, delim)
//   MD5 i n -> digest                   bbl_calc_md5 of a file of n bytes, byte k being
//                                       uint32_t((k * 2654435761 + i * 97) >> 13) & 0xff

#include "libslic3r/Utils.hpp"

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>
#include <vector>

using namespace Slic3r;

static uint64_t s = 0xBF58476D1CE4E5B9ull;
static uint64_t rnd() { s ^= s << 13; s ^= s >> 7; s ^= s << 17; return s; }
static uint64_t rnd(uint64_t n) { return rnd() % n; }
static std::string hex(const std::string &v)
{
    if (v.empty()) return "-";
    std::string out;
    char b[3];
    for (unsigned char c : v) { snprintf(b, 3, "%02x", c); out += b; }
    return out;
}

static const char *pieces[] = { "a", "B", "7", "0", ".", "_", "-", " ", "\xc3\xa9", "\xe4\xb8\xad", "\xf0\x9f\x98\x80",
                                "/", "\\", "/", "user", "Documents", "model.3mf", "12345", "x" };
static std::string text(size_t max_len)
{
    std::string out;
    const size_t n = rnd(max_len + 1);
    for (size_t i = 0; i < n; ++ i) out += pieces[rnd(sizeof(pieces) / sizeof(pieces[0]))];
    return out;
}

int main(int argc, char **argv)
{
    if (argc < 3) return 1;
    const std::string home = argv[1];
    const std::string scratch = argv[2];
    setenv("HOME", home.c_str(), 1);
    s ^= std::hash<std::string>()(home);

    const std::string suffix = "/Library/Application Support/BambuStudio/user";
    const size_t      sep    = home.find_last_of("\\/");
    std::string upper_home   = home;
    if (sep != std::string::npos && sep + 1 < upper_home.size() && upper_home[sep + 1] >= 'a' && upper_home[sep + 1] <= 'z')
        upper_home[sep + 1] = char(upper_home[sep + 1] - 'a' + 'A');
    std::string lower_home = home;
    if (sep != std::string::npos && sep + 1 < lower_home.size() && lower_home[sep + 1] >= 'A' && lower_home[sep + 1] <= 'Z')
        lower_home[sep + 1] = char(lower_home[sep + 1] - 'A' + 'a');
    for (int i = 0; i < 500; ++ i) {
        std::string raw;
        switch (rnd(10)) {
        case 0: raw = home + suffix + "/" + std::to_string(rnd(1000000000)) + "/" + text(4); break;
        case 1: raw = home + suffix + "\\" + std::to_string(rnd(100000)) + text(3); break;
        case 2: raw = home + "/" + text(6); break;
        case 3: raw = home; break;
        case 4: raw = home + text(3); break;
        case 5: raw = "C" + upper_home.substr(1) + "/" + text(5); break;   // different first byte, name capitalised
        case 6: raw = lower_home + "/" + text(5); break;
        case 7: raw = text(12); break;
        case 8: raw = upper_home + text(4); break;
        default: raw = home.substr(0, rnd(home.size() + 1)) + text(8); break;
        }
        printf("SAN %s %s -> %s\n", hex(home).c_str(), hex(raw).c_str(), hex(PathSanitizer::sanitize(raw)).c_str());
    }
    for (int i = 0; i < 100; ++ i) {
        const char delims[] = { ',', ';', '/', ' ', 'a' };
        char d = delims[rnd(5)];
        std::string str;
        size_t n = rnd(10);
        for (size_t k = 0; k < n; ++ k) str += rnd(3) ? std::string(1, d) : text(2);
        std::vector<std::string> parts = split_string(str, d);
        printf("SPLIT %02x %s -> %zu", (unsigned char)d, hex(str).c_str(), parts.size());
        for (const std::string &p : parts) printf(" %s", hex(p).c_str());
        printf("\n");
    }
    for (int i = 0; i < 20; ++ i) {
        std::string content;
        size_t n = (i == 0) ? 0 : (i == 1) ? 64 * 1024 : (i == 2) ? 64 * 1024 + 1 : (i < 8) ? rnd(120) : rnd(200000);
        // Bytes from a formula the test repeats.
        for (size_t k = 0; k < n; ++ k) content += char(uint32_t((uint64_t(k) * 2654435761u + uint64_t(i) * 97u) >> 13));
        std::string filename = scratch + "/md5_" + std::to_string(i) + ".bin";
        { std::ofstream f(filename, std::ios::binary); f.write(content.data(), content.size()); }
        std::string md5;
        bbl_calc_md5(filename, md5);
        printf("MD5 %d %zu -> %s\n", i, n, md5.c_str());
    }
    return 0;
}
