// Generator of the vectors of tests/print_config_validate_native_parity.rs: the configuration check of
// PrintConfig.cpp at the pin (references/BambuStudio/src/libslic3r): `DynamicPrintConfig::validate(bool
// under_cli)` (PrintConfig.cpp:8969-9007), which lays the configuration over a FullPrintConfig and calls
// `validate(const FullPrintConfig &cfg, bool under_cli)` (:10641-10916) and, through it,
// `validate_mixed_filament_params` (FilamentMixer.cpp:748-840) with its strict parsers (:597-696), and
// check_mixed_filament_diameter_consistency (:535-560).
//
// Usage: gen <scenarios.txt> > vectors.txt. Build, with REPO = this repository, BUILD = a built tree of the
// Bambu Studio engine (docs/building.md, -DENGINE=bambu) and DATA = this directory: compile with the compile
// line the build uses for a libslic3r source (its defines and include directories, the override layer
// $REPO/libslic3r/bambustudio before the pin, -std=c++17 -O3 -DNDEBUG and the forced include of
// cgal_54_compat.hpp), then link with the build's libslic3r_core and the libraries it links.
//
// `gen --defaults > vectors_defaults.txt`: every option of FullPrintConfig::defaults() as
// `D <key>\t<type()>\t<nullable()>\t<is_nil()>\t<serialize()>` in key order (ConfigOption*::type, ::serialize,
// ::nullable, ::is_nil of every option class the configuration has, Config.hpp); `-` for the serialization of
// an enum vector (coEnums), whose static default has no keys map in the pin.
//
// scenarios.txt: per scenario `CASE <under_cli>`, then `SET <key>\t<value>` lines (set_deserialize on an
// empty DynamicPrintConfig), then `END`. Output per scenario: `CASE <n>`, each finding as `E <key>\t<message>`
// in key order, then `N <count>`.

#include <algorithm>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "libslic3r/PrintConfig.hpp"

using namespace Slic3r;

int main(int argc, char **argv)
{
    if (argc < 2) {
        std::fprintf(stderr, "usage: %s <scenarios.txt> | --defaults\n", argv[0]);
        return 2;
    }
    if (std::string(argv[1]) == "--defaults") {
        const FullPrintConfig &defaults = FullPrintConfig::defaults();
        std::vector<std::string> keys = defaults.keys();
        std::sort(keys.begin(), keys.end());
        for (const std::string &key : keys) {
            const ConfigOption *opt = defaults.option(key);
            // A ConfigOptionEnumsGeneric of the static defaults has a null keys_map (its definition sets it
            // only on options made from the definition) and its serialize() dereferences it: `-` is written.
            std::string value = opt->type() == coEnums ? std::string("-") : opt->serialize();
            for (char &c : value)
                if (c == '\n') c = '\x01';
            std::printf("D %s\t%d\t%d\t%d\t%s\n", key.c_str(), int(opt->type()), int(opt->nullable()), int(opt->is_nil()), value.c_str());
        }
        return 0;
    }
    std::ifstream in(argv[1]);
    std::string line;
    int n = 0;
    bool under_cli = false;
    DynamicPrintConfig config;
    while (std::getline(in, line)) {
        if (line.rfind("CASE ", 0) == 0) {
            config = DynamicPrintConfig();
            under_cli = line.substr(5) == "1";
        } else if (line.rfind("SET ", 0) == 0) {
            const size_t tab = line.find('\t');
            const std::string key = line.substr(4, tab - 4);
            const std::string value = line.substr(tab + 1);
            ConfigSubstitutionContext ctxt{ForwardCompatibilitySubstitutionRule::Disable};
            config.set_deserialize(key, value, ctxt);
        } else if (line == "END") {
            const std::map<std::string, std::string> found = config.validate(under_cli);
            std::printf("CASE %d\n", n);
            for (const auto &[key, why] : found)
                std::printf("E %s\t%s\n", key.c_str(), why.c_str());
            std::printf("N %zu\n", found.size());
            ++n;
        }
    }
    return 0;
}
