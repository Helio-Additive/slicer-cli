// Reference vectors for crates/libslic3r-rs/src/desensitize.rs, taken from the C++ at the pin.
//
// The program compiles references/BambuStudio/src/libslic3r/Desensitize.cpp as it is, with the pin's
// nlohmann/json.hpp, Log.hpp and LogKeys.hpp, and with BBL_RELEASE_TO_PUBLIC=1 as the build of
// slicer_cli sets it (CMakeLists.txt), so the masks are on.
//
// Build and run, from the repository root (macOS arm64 clang is the reference build; PREFIX is where
// Boost is installed; STUB is a directory holding an empty libslic3r_version.h, the header CMake
// generates):
//   clang++ -w -std=c++17 -O2 -ffp-contract=on -DBOOST_LOG_DYN_LINK -DBBL_RELEASE_TO_PUBLIC=1 \
//       -I $STUB -I references/BambuStudio/src/libslic3r -I references/BambuStudio/src \
//       -I $PREFIX/include crates/libslic3r-rs/tests/data/desensitize/gen.cpp \
//       -L $PREFIX/lib -lboost_log -lboost_thread -o /tmp/des_gen
//   /tmp/des_gen > crates/libslic3r-rs/tests/data/desensitize/vectors.txt
//
// Output, one case per line. Strings are the hex of their bytes ("-" when empty).
//   MASK input -> dev_id dev_ip dev_name user_id mask_all
//   KEY key value -> 0 | 1 masked                       mask_by_key
//   JSON indent text -> json_log(parse(text), indent)
//   FLT bits -> json_log({"x": double}, -1)             the float printing of nlohmann's dump
//   INT text -> json_log({"x": integer}, -1)

#include "Desensitize.cpp"

#include <cstdio>
#include <cstring>

// Log.hpp asks this before it logs; it is defined in Log.cpp. Nothing is logged here: the log would
// go to the same stream as the vectors.
namespace Slic3r { namespace log_detail { bool log_enabled(LogLevel) { return false; } } }

using namespace Slic3r;

static uint64_t s = 0x9E3779B97F4A7C15ull;
static uint64_t rnd() { s ^= s << 13; s ^= s >> 7; s ^= s << 17; return s; }
static uint64_t rnd(uint64_t n) { return rnd() % n; }
static void ps(const std::string &v)
{
    printf(" ");
    if (v.empty()) printf("-");
    for (unsigned char c : v) printf("%02x", c);
}

static const char *alphabet[] = { "a", "B", "7", "0", ".", ".", "_", "_", "-", " ", "\xc3\xa9", "\xe4\xb8\xad", "\xf0\x9f\x98\x80", "*", "x" };
static std::string text(size_t max_len)
{
    std::string out;
    const size_t n = rnd(max_len + 1);
    for (size_t i = 0; i < n; ++ i) out += alphabet[rnd(sizeof(alphabet) / sizeof(alphabet[0]))];
    return out;
}

static const char *keys[] = {
    "dev_id", "sn", "dev_ip", "dev_name", "uid", "access_code", "token", "password", "authkey", "ttcode_enc", "url",
    "region", "model_id", "my_dev_id_2", "snapshot", "x_sn", "the_dev_name", "dev_ip_v6", "ip", "ipcam", "description",
    "user_id", "tag_uid", "tray_uuid", "passwd", "old_password", "access_code2", "base_url", "channel_name",
    "my_channel_name", "ttcode_enc_x", "authkey2", "region_code", "access_token", "modelId", "designModelId",
    "the_model_id", "name", "value", "data", "list", "seq", "ret", "k\xc3\xa9y", "", "a\"b\\c\n\t\x01"
};
static const size_t num_keys = sizeof(keys) / sizeof(keys[0]);

static std::string quote(const std::string &v)
{
    std::string out = "\"";
    for (unsigned char c : v) {
        if (c == '"' || c == '\\') { out += '\\'; out += char(c); }
        else if (c < 0x20) { char buf[8]; snprintf(buf, sizeof(buf), "\\u%04x", c); out += buf; }
        else out += char(c);
    }
    return out + "\"";
}

static std::string number()
{
    char buf[64];
    switch (rnd(8)) {
    case 0: snprintf(buf, sizeof(buf), "%lld", (long long) rnd(1000000) - 500000); break;
    case 1: snprintf(buf, sizeof(buf), "%llu", (unsigned long long) rnd()); break;
    case 2: snprintf(buf, sizeof(buf), "-%llu", (unsigned long long) (rnd() >> 1)); break;
    // At most 15 significant digits and a small exponent: read exactly by any parser.
    case 3: snprintf(buf, sizeof(buf), "%.3f", (double(rnd(2000001)) - 1000000.) / 1000.); break;
    case 4: snprintf(buf, sizeof(buf), "%llue%d", (unsigned long long) rnd(1000000), int(rnd(40)) - 20); break;
    case 5: snprintf(buf, sizeof(buf), "%llu.%06lluE+%d", (unsigned long long) rnd(1000), (unsigned long long) rnd(1000000), int(rnd(12))); break;
    case 6: { static const char *special[] = { "0", "-0", "0.0", "-0.0", "1e0", "1E-7", "123456789012345", "0.000123", "1e15", "1e16", "9007199254740993", "18446744073709551615", "-9223372036854775808", "1.5e300", "5e-324" }; return special[rnd(15)]; }
    default: snprintf(buf, sizeof(buf), "%llu", (unsigned long long) rnd(100)); break;
    }
    return buf;
}

static std::string value(int depth);

static std::string object(int depth)
{
    std::string out = "{";
    const size_t n = rnd(depth == 0 ? 7 : 4) + (depth == 0 ? 1 : 0);
    for (size_t i = 0; i < n; ++ i) {
        if (i > 0) out += rnd(2) ? "," : " , ";
        out += quote(keys[rnd(4) == 0 ? rnd(num_keys) : rnd(37)]) + (rnd(2) ? ":" : ": ") + value(depth + 1);
    }
    return out + "}";
}

static std::string string_value()
{
    switch (rnd(8)) {
    case 0: { char buf[32]; snprintf(buf, sizeof(buf), "%d.%d.%d.%d", int(rnd(256)), int(rnd(256)), int(rnd(256)), int(rnd(256))); return quote(buf); }
    case 1: return quote("01S00A123456789");
    case 2: return quote(text(4) + "_" + text(8));
    case 3: return quote("");
    default: return quote(text(14));
    }
}

static std::string array(int depth)
{
    std::string out = "[";
    const size_t n = rnd(5);
    for (size_t i = 0; i < n; ++ i) {
        if (i > 0) out += ",";
        switch (rnd(9)) {
        // A string that is itself a json text: json_log masks it and stores the dump.
        case 0: out += quote(object(depth + 2)); break;
        case 1: out += quote(rnd(2) ? number() : rnd(2) ? "null" : rnd(2) ? "[]" : "\"quoted\""); break;
        case 2: out += quote(array(depth + 2)); break;
        case 3: out += quote("{\"dev_id\": \"0123456789\", \"n\": 1.50, \"l\": [\"{\\\"sn\\\": \\\"ABCDEFGHIJ\\\"}\"]} "); break;
        case 4: out += quote(text(10)); break;
        default: out += value(depth + 1); break;
        }
    }
    return out + "]";
}

static std::string value(int depth)
{
    const uint64_t pick = rnd(depth >= 4 ? 6 : 10);
    switch (pick) {
    case 0: case 1: case 2: return string_value();
    case 3: case 4: return number();
    case 5: return rnd(3) == 0 ? "null" : rnd(2) ? "true" : "false";
    case 6: case 7: return object(depth);
    default: return array(depth);
    }
}

int main()
{
    for (int i = 0; i < 300; ++ i) {
        std::string in = text(i < 60 ? size_t(i / 5) : 16);
        if (rnd(5) == 0) { char buf[32]; snprintf(buf, sizeof(buf), "%d.%d.%d.%d", int(rnd(256)), int(rnd(300)), int(rnd(256)), int(rnd(256))); in = buf; }
        if (rnd(9) == 0) in = "1.2.3.4";
        printf("MASK"); ps(in); printf(" ->");
        ps(Desensitize::dev_id(in)); ps(Desensitize::dev_ip(in)); ps(Desensitize::dev_name(in)); ps(Desensitize::user_id(in)); ps(Desensitize::mask_all(in));
        printf("\n");
    }
    for (size_t k = 0; k < num_keys; ++ k)
        for (int i = 0; i < 3; ++ i) {
            const std::string in = i == 0 ? "192.168.1.20" : text(12);
            const std::optional<std::string> out = Desensitize::mask_by_key(keys[k], in);
            printf("KEY"); ps(keys[k]); ps(in); printf(" -> %d", int(bool(out)));
            if (out) ps(*out);
            printf("\n");
        }
    for (int i = 0; i < 190; ++ i) {
        std::string doc;
        switch (rnd(14)) {
        case 0: doc = array(0); break;
        case 1: doc = rnd(2) ? number() : string_value(); break;
        case 2: doc = rnd(3) == 0 ? "null" : rnd(2) ? "[]" : "{}"; break;
        default: doc = object(0); break;
        }
        const int indent = rnd(3) == 0 ? int(rnd(5)) : -1;
        printf("JSON %d", indent); ps(doc); printf(" ->"); ps(Desensitize::json_log(nlohmann::json::parse(doc), indent)); printf("\n");
    }
    for (int i = 0; i < 1400; ++ i) {
        double v;
        uint64_t bits = rnd();
        switch (rnd(6)) {
        case 0: v = double(rnd(100000)) / double(1 + rnd(1000)); break;
        case 1: v = std::ldexp(double(rnd() >> 11), int(rnd(200)) - 150); break;
        case 2: { static const double special[] = { 0., -0., 1., 0.1, 1e15, 1e16, 123456789012345678., 1e-4, 1e-5, 9.999e-5, 5e-324, 1.7976931348623157e308, 2.2250738585072014e-308, 0.3, 1e21, 1e22, 1e23, 999999999999999.9, 0.001, 100. }; v = special[rnd(20)]; if (rnd(4) == 0) v = -v; break; }
        case 3: v = std::pow(10., double(rnd(60)) - 30.) * double(1 + rnd(9999)); break;
        default: memcpy(&v, &bits, 8); break;
        }
        if (! std::isfinite(v)) continue;
        memcpy(&bits, &v, 8);
        nlohmann::json doc;
        doc["x"] = v;
        printf("FLT %llx ->", (unsigned long long) bits); ps(Desensitize::json_log(doc, -1)); printf("\n");
    }
    for (const char *t : { "0", "-1", "18446744073709551615", "-9223372036854775808", "9223372036854775807", "9223372036854775808", "42" }) {
        nlohmann::json doc;
        doc["x"] = nlohmann::json::parse(t);
        printf("INT"); ps(t); printf(" ->"); ps(Desensitize::json_log(doc, -1)); printf("\n");
    }
    return 0;
}
