// Native vectors for the typed machine variant fields (R954): `printer_extruder_id`
// (PrintConfig.hpp:1536), `printer_extruder_variant` (1538) and `extruder_type` (1519) as the pin's
// ConfigBase::load_from_json reads them from a settings file (Config.cpp:885-1080) over
// FullPrintConfig's defaults, and as GCodeProcessor::apply_config copies them into its result
// (GCodeProcessor.cpp:2038-2040: `.values`, `.values`, `cast<ExtruderType>(.values)`).
//
// usage: gen <scratch dir for the temporary json files> > vectors.txt
// Each case prints `JSON <document>` (one line), then `ID`, `VAR` (escape_strings_cstyle of the
// values), `ET` (the ExtruderType ints) of the loaded config, or `ERR <reason>`.
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/Config.hpp"
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

using namespace Slic3r;

int main(int argc, char **argv)
{
    if (argc < 2) {
        std::fprintf(stderr, "usage: %s <scratch dir>\n", argv[0]);
        return 2;
    }
    const std::string dir = argv[1];
    const std::vector<std::string> docs = {
        R"({"printer_extruder_id": ["1", "2"], "printer_extruder_variant": ["Direct Drive Standard", "Direct Drive High Flow"], "extruder_type": ["Direct Drive", "Direct Drive"]})",
        R"({"printer_extruder_id": ["1", "1", "2", "2"], "printer_extruder_variant": ["Direct Drive Standard", "Direct Drive High Flow", "Direct Drive Standard", "Direct Drive High Flow"], "extruder_type": ["Direct Drive", "Bowden"]})",
        R"({"extruder_type": ["Bowden"]})",
        R"({"extruder_type": ["Direct Drive", "Foo"]})",
        R"({"extruder_type": ["Bowden", "Bowden", "Bowden"]})",
        R"({"printer_extruder_variant": ["a;b", "c\"d", "e\\f", "g,h"]})",
        R"({"printer_extruder_variant": ["Bowden Standard"]})",
        R"({"printer_extruder_variant": []})",
        R"({"printer_extruder_id": []})",
        R"({"printer_extruder_id": ["7"]})",
        R"({"printer_extruder_id": ["3", "-1", "0"]})",
        R"({"printer_extruder_id": "2", "printer_extruder_variant": "Direct Drive High Flow", "extruder_type": "Bowden"})",
        R"({"printer_extruder_id": "1,2,2", "printer_extruder_variant": "\"x y\";\"z\"", "extruder_type": "Bowden,Direct Drive"})",
        R"({"layer_height": "0.2"})",
    };
    int n = 0;
    for (const std::string &doc : docs) {
        const std::string file = dir + "/mvf_case.json";
        {
            std::ofstream out(file, std::ios::binary);
            out << doc;
        }
        DynamicPrintConfig config;
        config.apply(FullPrintConfig::defaults());
        std::map<std::string, std::string> key_values;
        std::string reason;
        std::printf("CASE %d\nJSON %s\n", n++, doc.c_str());
        try {
            config.load_from_json(file, ForwardCompatibilitySubstitutionRule::Enable, key_values, reason);
        } catch (const std::exception &e) {
            std::printf("ERR %s\n", e.what());
            continue;
        }
        const auto *ids = config.option<ConfigOptionInts>("printer_extruder_id");
        const auto *vars = config.option<ConfigOptionStrings>("printer_extruder_variant");
        const auto *types = config.option<ConfigOptionEnumsGeneric>("extruder_type");
        std::string s;
        for (size_t i = 0; i < ids->values.size(); ++i)
            s += (i ? "," : "") + std::to_string(ids->values[i]);
        std::printf("ID %s\n", s.c_str());
        std::printf("VAR %s\n", escape_strings_cstyle(vars->values).c_str());
        // GCodeProcessor.cpp:2040 `cast<ExtruderType>(config.extruder_type.values)`.
        std::vector<ExtruderType> ets = cast<ExtruderType>(types->values);
        s.clear();
        for (size_t i = 0; i < ets.size(); ++i)
            s += (i ? "," : "") + std::to_string(int(ets[i]));
        std::printf("ET %s\n", s.c_str());
    }
    return 0;
}
