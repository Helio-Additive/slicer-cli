// Reference vectors for crates/libslic3r-rs/src/format/format.rs, taken from the C++ at the pin.
//
// The program includes references/BambuStudio/src/libslic3r/format.hpp as it is and calls
// Slic3r::format() with the Boost the engine is built with (docs/rust-engine/README.md).
//
// Build and run, from the repository root (macOS arm64 clang with libc++ is the reference build:
// what an argument prints as is what its operator<< writes):
//   D=crates/libslic3r-rs/tests/data/format
//   clang++ -std=c++17 -O2 -ffp-contract=on -DNDEBUG -I references/BambuStudio/src/libslic3r \
//       -I "$(brew --prefix)/include" $D/gen.cpp -o /tmp/format_gen
// (NDEBUG as the engine is built: Boost.Format asserts on flag combinations it otherwise accepts.)
//   /tmp/format_gen > $D/vectors.txt
//
// Output: two lines per case.
//   F <format string> <n> <argument>..      the call Slic3r::format(fmt, arguments...)
//   ok <text> | throw <what()>              what it returned or threw
// Texts are escaped: \\ \n \r \s for a space, \xHH for another control byte, \e alone for the
// empty text. An argument is its C++ type and value: h: short, i: int, l: long long,
// H: unsigned short, u: unsigned, U: unsigned long long (decimal); f: float, d: double (the hex of
// the bits); b: bool (0 or 1); c: char (its code); s: std::string (escaped).

#include "format.hpp"

#include <climits>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
#include <string>
#include <variant>
#include <vector>

static std::string esc(const std::string &s)
{
    if (s.empty())
        return "\\e";
    std::string out;
    char        buf[8];
    for (unsigned char c : s) {
        if (c == '\\')
            out += "\\\\";
        else if (c == '\n')
            out += "\\n";
        else if (c == '\r')
            out += "\\r";
        else if (c == ' ')
            out += "\\s";
        else if (c < 0x20 || c == 0x7f) {
            snprintf(buf, sizeof(buf), "\\x%02X", c);
            out += buf;
        } else
            out += (char) c;
    }
    return out;
}

using Arg = std::variant<short, int, long long, unsigned short, unsigned, unsigned long long, float, double, bool, char, std::string>;

static std::string arg_text(const Arg &a)
{
    char buf[64];
    switch (a.index()) {
    case 0: return "h:" + std::to_string(std::get<short>(a));
    case 1: return "i:" + std::to_string(std::get<int>(a));
    case 2: return "l:" + std::to_string(std::get<long long>(a));
    case 3: return "H:" + std::to_string(std::get<unsigned short>(a));
    case 4: return "u:" + std::to_string(std::get<unsigned>(a));
    case 5: return "U:" + std::to_string(std::get<unsigned long long>(a));
    case 6: {
        uint32_t b;
        float    v = std::get<float>(a);
        memcpy(&b, &v, 4);
        snprintf(buf, sizeof(buf), "f:%08x", b);
        return buf;
    }
    case 7: {
        uint64_t b;
        double   v = std::get<double>(a);
        memcpy(&b, &v, 8);
        snprintf(buf, sizeof(buf), "d:%016llx", (unsigned long long) b);
        return buf;
    }
    case 8: return std::string("b:") + (std::get<bool>(a) ? "1" : "0");
    case 9: return "c:" + std::to_string((int) (unsigned char) std::get<char>(a));
    default: return "s:" + esc(std::get<std::string>(a));
    }
}

static void run(const std::string &fmt, const std::vector<Arg> &args)
{
    printf("F %s %zu", esc(fmt).c_str(), args.size());
    for (const Arg &a : args)
        printf(" %s", arg_text(a).c_str());
    printf("\n");
    try {
        std::string out;
        switch (args.size()) {
        case 0: out = Slic3r::format(fmt); break;
        case 1: out = std::visit([&](const auto &a) { return Slic3r::format(fmt, a); }, args[0]); break;
        case 2: out = std::visit([&](const auto &a, const auto &b) { return Slic3r::format(fmt, a, b); }, args[0], args[1]); break;
        default:
            // The overload for `const char*`.
            out = std::visit([&](const auto &a, const auto &b, const auto &c) { return Slic3r::format(fmt.c_str(), a, b, c); }, args[0], args[1], args[2]);
            break;
        }
        printf("ok %s\n", esc(out).c_str());
    } catch (const std::exception &e) {
        printf("throw %s\n", esc(e.what()).c_str());
    }
}

static uint64_t g_seed = 0x13198A2E03707344ull;
static uint32_t rnd()
{
    g_seed = g_seed * 6364136223846793005ull + 1442695040888963407ull;
    return (uint32_t)(g_seed >> 33);
}
static int  rint(int n) { return (int) (rnd() % (uint32_t) n); }
static bool chance(int pct) { return rint(100) < pct; }

static Arg rarg()
{
    static const short              H[]  = {0, 1, -1, 255, -255, 32767, -32768, 12};
    static const int                I[]  = {0, 1, -1, 7, 42, -42, 255, -255, 1000, 65535, 123456789, -123456789, INT_MAX, INT_MIN};
    static const long long          L[]  = {0, -1, 1234567890123ll, -1234567890123ll, LLONG_MAX, LLONG_MIN, 99};
    static const unsigned short     UH[] = {0, 1, 255, 65535};
    static const unsigned           U[]  = {0, 1, 42, 255, 4294967295u, 123456789u};
    static const unsigned long long UL[] = {0, 1, 18446744073709551615ull, 1234567890123ull};
    static const float              F[]  = {0.1f, 1.5f, 3.4e38f, -2.25f, 0.f, 1e-10f};
    static const double             D[]  = {0.0, -0.0, 1.0, -1.0, 0.5, 1.5, -2.5, 3.14159265358979, 100.0, 123456.789, 1e-5, 0.000123456, 1234567.0, 1e21,
                                            1e100, 1.0 / 3, 2.0 / 3, 0.1, 5e-324, 1.7976931348623157e308, 9.995, 0.0005, 999999.5, 0.00001234, -1e-7, 1e6, 1e5,
                                            123456789.0, 2.2250738585072014e-308, std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity(),
                                            std::numeric_limits<double>::quiet_NaN(), 8.0, 255.0, -0.75, 1e15, 1e16, 1e17};
    static const char               C[]  = {'a', 'Z', '0', ' ', '%', '-'};
    static const char              *S[]  = {"", "x", "hello", "h\xC3\xA9llo", "a b", "-5", "+5", "0x1f", "a longer text"};
    switch (rint(16)) {
    case 0: return H[rint(8)];
    case 1:
    case 2:
    case 3: return I[rint(14)];
    case 4: return L[rint(7)];
    case 5: return UH[rint(4)];
    case 6: return U[rint(6)];
    case 7: return UL[rint(4)];
    case 8: return F[rint(6)];
    case 9:
    case 10:
    case 11: return D[rint(38)];
    case 12: return chance(50);
    case 13: return C[rint(6)];
    default: return std::string(S[rint(9)]);
    }
}

static std::string rdirective(bool positional, int n)
{
    static const char *WIDTH[] = {"", "", "1", "5", "8", "12", "*", "20"};
    static const char *PREC[]  = {"", "", "", ".", ".0", ".1", ".3", ".10", ".*", ".17"};
    static const char *MOD[]   = {"", "", "", "", "l", "ll", "h", "hh", "L", "z", "j"};
    static const char  CONV[]  = "ddiuxXoeEfFgGaAsSscCbpdfgs";
    static const char  FLAGS[] = "-+ #0=_'";
    std::string        s       = "%";
    const bool         bracket = chance(12);
    if (bracket)
        s += "|";
    if (positional)
        s += std::to_string(n) + "$";
    for (int k = rint(4); k > 0; --k)
        s += FLAGS[rint(8)];
    s += WIDTH[rint(8)];
    s += PREC[rint(10)];
    s += MOD[rint(11)];
    if (bracket && chance(30)) {
        // %|spec| without a conversion character
        s += "|";
        return s;
    }
    const int r = rint(100);
    if (r < 2)
        s += "q";
    else if (r < 3)
        s += "n";
    else
        s += CONV[rint(26)];
    if (bracket && chance(92))
        s += "|";
    return s;
}

int main()
{
    // The forms libslic3r passes.
    run("Invalid value provided for parameter %1%: %2%", {std::string("layer_height"), std::string("0,2")});
    run("Failed loading configuration file \"%1%\": %2%", {std::string("a.json"), std::string("parse error")});
    run("Config tag \"; CONFIG_BLOCK_START\" not found", {});
    run("Suspiciously low number of configuration values extracted from %1%: %2%", {std::string("x.gcode"), (unsigned long long) 3});
    run("ZFiller: both point in subject : %d, %d, %d", {(long long) 1, (long long) -2, (long long) 300000});
    run("round %1% ant %2% path length %3%", {1, 2, 123.456});
    run("mcut mcDebugOutput message ( %d ): %s ", {(unsigned) 7, std::string("msg")});
    run("top%d", {3});
    run("Failed loading the Config Bundle \"%1%\": %2% at line %3%", {std::string("b.ini"), std::string("bad"), (unsigned long long) 12});
    run("Plate %d: %s does not support filament %s", {2, std::string("Cool Plate"), 3});
    run("Object: %1%", {std::string("cube")});
    run("Generating G-code: layer %1%", {std::string("17")});

    // Marks, positions and errors.
    const char *plain[] = {"", "no directive", "100%%", "%%", "%%%%", "a%%b%%c", "%", "50%", "50% off", "%%%", "%1", "%1$", "%|", "%|1", "%-", "%5", "%.", "%.3",
                           "%l", "%1%%", "%T", "%5T", "% ", "%q", "%I", "%I3", "%I32", "%I64", "%I6", "%3", "%n", "x%ny", "%5.2n%d"};
    for (const char *fmt : plain) {
        run(fmt, {});
        run(fmt, {42});
        run(fmt, {42, std::string("s")});
    }
    const char *positional[] = {"%1%", "%1% %1%", "%2% %1%", "%1% %3%", "%3%", "%1%%2%%3%", "%1$d %2$s", "%2$5d|%1$-8s|", "%|1$+8.3f|", "%|2$|", "%1% %d", "%d %1%",
                                "%d %2$d", "%1$d %d", "%0%", "%1%%", "%|1%|", "%10%", "%1$5%", "%1%%20t|%2%", "%1%%20T.|%2%", "%5t%1%", "%1%%3t%2%", "%1$s%10T*x",
                                "%|1$5|%|2$-5|%|1$=9|", "%1$n%2$d", "%n%d", "%-5n%d|%d", "%2$#x %1$#o", "%1$+d %1$ d %1$05d", "%1$.2s|%1$5.1s|%1$-5.1s|",
                                "%1$c%2$c", "%1$b %2$b", "%1$8b|", "%1$-8b|", "%1$08b|", "%1$I64d", "%1$I32x", "%1$wd", "%1$lld %2$hhu", "%1$.0e %2$#.0e %2$#.0f %1$#g",
                                "%1$'d", "%1$=7d|%2$=8s|", "%1$_8d|%2$_8s|", "%1$ 5d|%2$ 5s|%2$ s|", "%1$ 05d|%1$+05d|%1$-05d|", "%1$0 5d|%1$#010x|%2$010s|",
                                "%1$10.4d|%2$10.4f|%2$.4d|", "%1$*d|%2$.*f|", "%1$5c|%2$-5c|%2$05c|"};
    for (const char *fmt : positional) {
        for (int k = 0; k < 4; ++k) {
            std::vector<Arg> args;
            for (int n = rint(4); n > 0; --n)
                args.push_back(rarg());
            run(fmt, args);
        }
        run(fmt, {-42, 3.14159});
        run(fmt, {std::string("hello"), true});
        run(fmt, {'c', 1e-5, (unsigned long long) 255});
    }

    // One directive, one argument.
    for (int i = 0; i < 3200; ++i) {
        std::string fmt;
        if (chance(30))
            fmt += chance(50) ? "[" : "a %% b ";
        fmt += rdirective(false, 0);
        if (chance(30))
            fmt += chance(50) ? "]" : " z";
        run(fmt, {rarg()});
    }
    // Several directives.
    for (int i = 0; i < 500; ++i) {
        const int        n          = 1 + rint(3);
        const bool       positional = chance(40);
        std::string      fmt;
        std::vector<Arg> args;
        for (int k = 0; k < n; ++k) {
            fmt += rdirective(positional, positional ? 1 + rint(n) : 0);
            fmt += chance(50) ? "|" : ", ";
        }
        for (int k = n + (chance(8) ? rint(3) - 1 : 0); k > 0 && args.size() < 3; --k)
            args.push_back(rarg());
        run(fmt, args);
    }
    return 0;
}
