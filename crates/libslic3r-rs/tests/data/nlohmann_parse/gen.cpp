// Native vectors for `nlohmann::json::parse(std::string)` of the nlohmann 3.10.4 the pin bundles
// (src/nlohmann), the reader of Desensitize.cpp:219 (json_log's nested documents) and of the assembly
// JSON readers (R954). Documents: nlohmann's own dump of doubles drawn from a fixed LCG (17
// significant digits, what nlohmann writes), and hand-made edge cases (BOM, -0, overflow, trailing
// text, deep nesting, big integers).
//
// usage: gen > vectors.txt
// Per case: `DOC <hex of the document bytes>`, then `ERR` where parse throws, else the walk of the
// value: `N <integer>` (number_integer / number_unsigned), `F <16 hex digits of the double's bits>`,
// `S <hex of the bytes>`, `B 0|1`, `Z` (null), `A <n>` / `O <n>` (array / object with n members;
// object keys `K <hex>` in std::map order), then `END`.
#include <nlohmann/json.hpp>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

using json = nlohmann::json;

static uint64_t g_state = 0x9E3779B97F4A7C15ull;
static uint64_t lcg()
{
    g_state = g_state * 6364136223846793005ull + 1442695040888963407ull;
    return g_state;
}

static std::string hex(const std::string &s)
{
    std::string out;
    char buf[3];
    for (unsigned char c : s) {
        std::snprintf(buf, sizeof(buf), "%02x", c);
        out += buf;
    }
    return out;
}

static void walk(const json &j)
{
    switch (j.type()) {
    case json::value_t::null: std::printf("Z\n"); break;
    case json::value_t::boolean: std::printf("B %d\n", j.get<bool>() ? 1 : 0); break;
    case json::value_t::number_integer: std::printf("N %lld\n", (long long) j.get<int64_t>()); break;
    case json::value_t::number_unsigned: std::printf("N %llu\n", (unsigned long long) j.get<uint64_t>()); break;
    case json::value_t::number_float: {
        double d = j.get<double>();
        uint64_t bits;
        std::memcpy(&bits, &d, 8);
        std::printf("F %016llx\n", (unsigned long long) bits);
        break;
    }
    case json::value_t::string: std::printf("S %s\n", hex(j.get<std::string>()).c_str()); break;
    case json::value_t::array:
        std::printf("A %zu\n", j.size());
        for (const auto &v : j) walk(v);
        break;
    case json::value_t::object:
        std::printf("O %zu\n", j.size());
        for (auto it = j.begin(); it != j.end(); ++it) {
            std::printf("K %s\n", hex(it.key()).c_str());
            walk(it.value());
        }
        break;
    default: std::printf("?\n"); break;
    }
}

static double draw_double()
{
    const uint64_t r = lcg();
    switch (r % 6) {
    case 0: { // any finite double
        uint64_t bits = lcg();
        double d;
        std::memcpy(&d, &bits, 8);
        if (!(d == d) || d - d != 0) return 1.0 / 3.0;
        return d;
    }
    case 1: return double(int64_t(lcg() % 2000001) - 1000000) * 0.00001; // unscaled coordinates
    case 2: return double(int64_t(lcg() % 20001) - 10000) / 10.0;
    case 3: return double(lcg() >> 11) * (1.0 / 9007199254740992.0); // [0, 1)
    case 4: return double(int64_t(lcg() % 1000000) + 1) / double(int64_t(lcg() % 997) + 3);
    default: return std::ldexp(double(lcg() >> 11), int(lcg() % 120) - 90);
    }
}

int main()
{
    std::vector<std::string> docs;
    for (int i = 0; i < 300; ++i) {
        json j;
        if (i % 3 == 0) {
            j = json::array();
            for (int k = 0; k < 6; ++k) j.push_back(draw_double());
        } else if (i % 3 == 1) {
            j = json::object();
            j["x"] = draw_double();
            j["y"] = draw_double();
            j["z"] = json::array({draw_double(), int64_t(lcg() % 1000) - 500, "s"});
        } else {
            j = json::object();
            j["matrix"] = json::array();
            for (int k = 0; k < 16; ++k) j["matrix"].push_back(draw_double());
            j["name"] = "part";
        }
        docs.push_back(j.dump());
    }
    const char *edge[] = {
        "-0", "[-0.0]", "[0.0, -0, 0e0, -0e-0]", "1e400", "-1e400", "1e-400", "[4.9e-324, 2.4703282292062328e-324]",
        "[2.2250738585072011e-308, 2.2250738585072014e-308]", "[1.7976931348623157e308, 1.7976931348623159e308]",
        "\xEF\xBB\xBF[1.5]", "[1] x", "  {\"a\": 0.1}  \n", "{\"a\":1}{\"b\":2}", "18446744073709551615",
        "18446744073709551616", "-9223372036854775808", "-9223372036854775809", "123456789012345678901234567890",
        "0.1e1", "1E+2", "1.e2", ".5", "01", "[]", "{}", "nul", "\"\\u00e9\\ud83d\\ude00\"", "\"\\ud83d\"",
        "{\"b\": 1, \"a\": 2, \"b\": 3}", "[0.30000000000000004, 0.1, 0.2]", "[9007199254740993, 9007199254740993.0]",
        "[3.141592653589793238462643383279]", "[1e23, 8.98846567431158e307]", "true", "[false, null]", "",
        "   ", "[1,]", "{\"a\" 1}", "[\"\\x\"]", "\"tab\there\"", "[1.0e+00000000000000000001]",
        "[0.000000000000000000000000000000000000000000000000000000000000001]",
    };
    for (const char *e : edge) docs.emplace_back(e);
    {
        std::string deep(200, '[');
        deep += "0.5";
        deep += std::string(200, ']');
        docs.push_back(deep);
    }
    for (const std::string &doc : docs) {
        std::printf("DOC %s\n", hex(doc).c_str());
        try {
            json j = json::parse(doc);
            walk(j);
        } catch (const std::exception &) {
            std::printf("ERR\n");
        }
        std::printf("END\n");
    }
    return 0;
}
