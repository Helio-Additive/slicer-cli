// Vectors for tests/utils_string_printf_native_parity.rs: the pin's string_printf("%s", s)
// (utils.cpp:1126-1146) for strings of every length from 0 to 260 bytes, ASCII and UTF-8.
// Each line: the input and the output, as hex. Usage: gen > vectors.txt
#include <cstdio>
#include <string>
#include "libslic3r/Utils.hpp"

using namespace Slic3r;

static std::string hex(const std::string &s)
{
    std::string out;
    char buf[3];
    for (unsigned char c : s) { snprintf(buf, sizeof(buf), "%02x", c); out += buf; }
    return out.empty() ? "-" : out;
}

int main()
{
    for (int len = 0; len <= 260; ++len) {
        std::string s;
        for (int i = 0; i < len; ++i) s.push_back(char('a' + i % 26));
        printf("%s %s\n", hex(s).c_str(), hex(string_printf("%s", s.c_str())).c_str());
    }
    // UTF-8: a two-byte character across the 198-byte cut.
    for (int len = 195; len <= 202; ++len) {
        std::string s(size_t(len - 2), 'x');
        s += "\xc3\xa9";
        printf("%s %s\n", hex(s).c_str(), hex(string_printf("%s", s.c_str())).c_str());
    }
    return 0;
}
