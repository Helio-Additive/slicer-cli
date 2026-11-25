// Vectors for tests/utils_xml_escape_native_parity.rs: the pin's xml_escape (both is_marked values),
// xml_escape_double_quotes_attribute_value and xml_unescape (utils.cpp:1207-1300) over strings built
// from a fixed-seed LCG out of special characters, entities and UTF-8 text. Each line: the input and the
// four outputs, as hex. Usage: gen > vectors.txt
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>
#include "libslic3r/Utils.hpp"

using namespace Slic3r;

static uint64_t g_state = 0x2545F4914F6CDD1Dull;
static uint32_t lcg()
{
    g_state = g_state * 6364136223846793005ull + 1442695040888963407ull;
    return uint32_t(g_state >> 33);
}

static std::string hex(const std::string &s)
{
    std::string out;
    char buf[3];
    for (unsigned char c : s) { snprintf(buf, sizeof(buf), "%02x", c); out += buf; }
    return out.empty() ? "-" : out;
}

int main()
{
    const std::vector<std::string> tokens = {"\"", "'", "&", "<", ">", "\r", "\n", "\t", "&lt;", "&gt;", "&amp;", "&quot;", "&apos;",
                                             "&#xD;", "&#xA;", "&#x9;", "&lt", "&am", "&#x", "\xc3\xa9", "\xe2\x82\xac", "a", "Z", " ", "1", ";"};
    for (int i = 0; i < 400; ++i) {
        std::string s;
        int n = int(lcg() % 9u);
        for (int k = 0; k < n; ++k) s += tokens[lcg() % tokens.size()];
        printf("%s %s %s %s %s\n", hex(s).c_str(), hex(xml_escape(s, false)).c_str(), hex(xml_escape(s, true)).c_str(),
               hex(xml_escape_double_quotes_attribute_value(s)).c_str(), hex(xml_unescape(s)).c_str());
    }
    return 0;
}
