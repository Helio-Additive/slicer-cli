// Vectors for tests/app_config_keys_native_parity.rs: the pin's AppConfig::get_mouse_device_lock_horizon
// (AppConfig.hpp:257-258, through get_3dmouse_device_numeric_value<bool>) over stored strings, and
// dual_extruder_first_slice_video_app_config_key (AppConfig.hpp:303-315) over printer model names,
// get_language_code and get_hms_host over stored values.
// Usage: gen > vectors.txt
#include <cstdio>
#include <string>
#include <vector>
#include "libslic3r/AppConfig.hpp"

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
    const std::vector<std::string> values = {"1", "0", "0.5", "-0", "-1", "abc", "1e3", "", "2,5", "0.0000001", "nan", "inf",
                                             "0x10", "1abc", "true", "false", "00", "-0.0", "1e-400", "1e400"};
    AppConfig cfg;
    for (const std::string &v : values) {
        cfg.set("mouse_device:dev", "lock_horizon", v);
        bool lock = false;
        bool found = cfg.get_mouse_device_lock_horizon("dev", lock);
        printf("lock %s %d %d\n", hex(v).c_str(), int(found), int(lock));
    }
    bool lock = false;
    printf("lock_missing_device %d\n", int(cfg.get_mouse_device_lock_horizon("other", lock)));
    cfg.set("mouse_device:dev2", "swap_yz", "1");
    printf("lock_missing_key %d\n", int(cfg.get_mouse_device_lock_horizon("dev2", lock)));
    const std::vector<std::string> models = {"Bambu Lab H2D", "Bambu Lab X1 Carbon", "", " ", "  a  b ", "N\xc3\xa9o 2", "A\tB", "H2D Pro"};
    for (const std::string &m : models)
        printf("key %s %s\n", hex(m).c_str(), hex(dual_extruder_first_slice_video_app_config_key(m)).c_str());
    // AppConfig::get_language_code (AppConfig.cpp:48-63) and get_hms_host (AppConfig.cpp:65-82).
    const std::vector<std::string> langs = {"", "zh_CN", "zh_TW", "en", "en_US", "e", "de_DE", "zh_cn", "\xc3\xa9t", "ja_JP.UTF-8"};
    for (const std::string &l : langs) {
        cfg.set("language", l);
        printf("lang %s %s\n", hex(l).c_str(), hex(cfg.get_language_code()).c_str());
    }
    const std::vector<std::string> envs = {"", "dev", "qat", "pre", "product", "other"};
    for (const std::string &e : envs) {
        cfg.set("iot_environment", e);
        printf("hms %s %s\n", hex(e).c_str(), hex(cfg.get_hms_host()).c_str());
    }
    return 0;
}
