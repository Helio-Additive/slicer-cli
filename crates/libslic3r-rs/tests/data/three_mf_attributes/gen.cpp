// Vectors for tests/three_mf_attributes_native_parity.rs: the pin's XML attribute helpers of 3mf.cpp
// (get_attribute_value_charptr / _string / _float / _int / _bool, 3mf.cpp:165-204, global functions of
// the engine library) and their bbs_3mf.cpp twins (bbs_get_attribute_value_*, 490-536) over attribute values.
// Each line: the value (hex), then found, string (hex), float bits, int, bool for each of the two. Usage: gen > vectors.txt
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

const char *get_attribute_value_charptr(const char **attributes, unsigned int attributes_size, const char *attribute_key);
std::string get_attribute_value_string(const char **attributes, unsigned int attributes_size, const char *attribute_key);
float get_attribute_value_float(const char **attributes, unsigned int attributes_size, const char *attribute_key);
int get_attribute_value_int(const char **attributes, unsigned int attributes_size, const char *attribute_key);
bool get_attribute_value_bool(const char **attributes, unsigned int attributes_size, const char *attribute_key);
static std::string hex(const std::string &s);
// bbs_3mf.cpp:490-536, the same helpers of the BBS reader.
const char *bbs_get_attribute_value_charptr(const char **attributes, unsigned int attributes_size, const char *attribute_key);
std::string bbs_get_attribute_value_string(const char **attributes, unsigned int attributes_size, const char *attribute_key);
float bbs_get_attribute_value_float(const char **attributes, unsigned int attributes_size, const char *attribute_key);
int bbs_get_attribute_value_int(const char **attributes, unsigned int attributes_size, const char *attribute_key);
bool bbs_get_attribute_value_bool(const char **attributes, unsigned int attributes_size, const char *attribute_key);

static void bbs_columns(const char **attrs, unsigned int n)
{
    float f = bbs_get_attribute_value_float(attrs, n, "key");
    unsigned int bits;
    memcpy(&bits, &f, 4);
    printf(" %d %s %08x %d %d", bbs_get_attribute_value_charptr(attrs, n, "key") != nullptr, hex(bbs_get_attribute_value_string(attrs, n, "key")).c_str(),
           bits, bbs_get_attribute_value_int(attrs, n, "key"), int(bbs_get_attribute_value_bool(attrs, n, "key")));
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
    const std::vector<std::string> values = {"1", "0", "-1", "+1", " 1", "1.5", "-0.25", ".5", "5.", "1e3", "1e", "1e+", "2E-2x", "0x10",
                                             "inf", "-inf", "Infinity", "infx", "nan", "-nan", "NaN", "abc", "", "-", ".", "3.4028236e38",
                                             "1e50", "1e-50", "1.17549435e-38", "0.1", "123456789", "2147483647", "2147483648",
                                             "-2147483648", "-2147483649", "99999999999", "12abc", "007", "-0", "+-1", "0.30000001192092896",
                                             "16777217", "1.0000001", "   ", "\t2", "1,5"};
    for (const std::string &v : values) {
        const char *attrs[] = {"other", "x", "key", v.c_str()};
        const char *p = get_attribute_value_charptr(attrs, 4, "key");
        float f = get_attribute_value_float(attrs, 4, "key");
        unsigned int bits;
        memcpy(&bits, &f, 4);
        printf("%s %d %s %08x %d %d", hex(v).c_str(), p != nullptr, hex(get_attribute_value_string(attrs, 4, "key")).c_str(), bits,
               get_attribute_value_int(attrs, 4, "key"), int(get_attribute_value_bool(attrs, 4, "key")));
        bbs_columns(attrs, 4);
        printf("\n");
    }
    // A missing key: nullptr, "", 0, 0, true.
    const char *attrs[] = {"other", "x"};
    float f = get_attribute_value_float(attrs, 2, "key");
    unsigned int bits;
    memcpy(&bits, &f, 4);
    printf("missing %d %s %08x %d %d", get_attribute_value_charptr(attrs, 2, "key") != nullptr, hex(get_attribute_value_string(attrs, 2, "key")).c_str(),
           bits, get_attribute_value_int(attrs, 2, "key"), int(get_attribute_value_bool(attrs, 2, "key")));
    bbs_columns(attrs, 2);
    printf("\n");
    return 0;
}
