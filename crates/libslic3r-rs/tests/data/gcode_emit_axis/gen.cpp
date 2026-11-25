// Native vectors for GCodeFormatter::emit_axis (GCodeWriter.cpp:1097-1137), the formatter of every
// X/Y/Z/E/F/I/J word the pin writes. Prints, per case, the bits of the double (16 hex digits), the
// digits and the text emit_axis appends after the axis letter. Usage: gen <cases> | gzip -9n
#include "libslic3r/GCodeWriter.hpp"
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

using namespace Slic3r;

static uint64_t g_state = 0x9e3779b97f4a7c15ULL;
static uint64_t lcg() { g_state = g_state * 6364136223846793005ULL + 1442695040888963407ULL; return g_state >> 11; }
static double unit() { return double(lcg()) / double(1ULL << 53); }

static void emit(double v, size_t digits)
{
    GCodeFormatter f;
    f.emit_axis('F', v, digits);
    std::string s = f.string();
    // " F<digits>\n" -> the digits only
    std::string body = s.substr(2, s.size() - 3);
    uint64_t bits;
    std::memcpy(&bits, &v, sizeof bits);
    std::printf("%016llx %zu %s\n", (unsigned long long)bits, digits, body.c_str());
}

int main(int argc, char **argv)
{
    int n = argc > 1 ? std::atoi(argv[1]) : 1000;
    static const size_t digit_set[] = { 1, 3, 5 };
    static const double pow10[] = { 1, 10, 100, 1000, 10000, 100000, 1000000 };
    for (int i = 0; i < n; ++i) {
        size_t d = digit_set[lcg() % 3];
        double scale = pow10[lcg() % 6];
        // plain values over several magnitudes, both signs
        double v = (unit() * 2. - 1.) * scale;
        emit(v, d);
        // values on a half of the last digit, and one ulp either side
        double k = std::floor(unit() * 20000. * scale) - 10000. * scale;
        double h = (k + 0.5) / pow10[d];
        emit(h, d);
        emit(std::nextafter(h, HUGE_VAL), d);
        emit(std::nextafter(h, -HUGE_VAL), d);
        // feedrates as the writer formats them (speed * 60)
        double speed = unit() * 600.;
        emit(speed * 60., 3);
    }
    // fixed cases: zero, negative zero, tiny values, the H2D feedrate and R949's example
    const double fixed[] = { 0., -0., 0.0004, -0.0004, 0.0005, -0.0005, 9416.0045, 9416.7995, 60000., 0.4, -0.02, 1e-9, -1e-9 };
    for (double v : fixed)
        for (size_t d : digit_set)
            emit(v, d);
    return 0;
}
