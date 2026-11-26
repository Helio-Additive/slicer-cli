// Link against the Bambu CLI objects in place of main.cpp. Pass a temporary
// settings file and the output JSONL path. main.cpp 735–756, 770–850, 883.
#define main slicer_reference_main
#include "main.cpp"
#undef main
#include <cstring>

int main(int argc, char** argv)
{
    if (argc != 3) return 1;
    boost::nowide::ofstream output(argv[2]);
    const std::vector<json> inputs = {
        nullptr, true, false, 1, 0.600000123456789, "", "nil", "0.55", "0.6,0.4",
        json::array(), json::array({"0.400000123456789", "nil"}),
        json::array({0.400000123456789, 0.65}), json::array({true, "0.6", 0.4}),
        json::array({json::array({"0.4", "0.6"})}), json::object()
    };
    const std::vector<std::string> keys = {"nozzle_diameter", "printer_model", "printer_variant", "printer_settings_id"};
    for (bool seeded : {false, true}) {
        for (const auto& key : keys) {
            for (const auto& value : inputs) {
                Slic3r::DynamicPrintConfig config;
                if (seeded) {
                    config.set_deserialize_strict("nozzle_diameter", "0.6,0.4");
                    config.set_deserialize_strict("printer_model", "Bambu Lab X1 Carbon");
                    config.set_deserialize_strict("printer_variant", "0.6");
                    config.set_deserialize_strict("printer_settings_id", "Saved preset");
                }
                json file = {{key, value}};
                { boost::nowide::ofstream f(argv[1]); f << file.dump(); }
                if (!load_json_config(argv[1], config)) return 2;
                json effective = json::object();
                for (const auto& name : keys) {
                    const auto* opt = config.option(name);
                    if (!opt) continue;
                    if (name != "nozzle_diameter") {
                        effective[name] = opt->serialize();
                    } else {
                        json bits = json::array();
                        for (double d : static_cast<const Slic3r::ConfigOptionVector<double>*>(opt)->values) {
                            uint64_t raw;
                            std::memcpy(&raw, &d, sizeof(raw));
                            bits.push_back(raw);
                        }
                        effective[name] = bits;
                    }
                }
                output << json({{"seeded", seeded}, {"file", file}, {"effective", effective}}).dump() << '\n';
            }
        }
    }
    return output ? 0 : 3;
}
