// main.cpp 7244–7257. Run the same override function as the Bambu CLI after
// the separate printer-variant check. The output includes typed row bits.
#define main slicer_reference_main
#include "main.cpp"
#undef main
#include <cstring>

int main()
{
    using namespace Slic3r;
    for (const char *initial : {"absent", "empty", "0.4", "0.4,0.4", "0.4,0.6"}) {
        for (const char *arg : {"0.4", "0.6", "0.4,0.6", ".4000004", "0.3,0.65,0.8", "1e-1",
                                "0.6mm", " 0.2", "0.6 ", "0.4,bad", "bad", "", "0.00000001", "0.4,nil", "1e-39mm",
                                "1e-499mm", "0x1p-149mm", "0x1p-150mm", "1e40mm",
                                "0x1.0000000000001p-149mm",
                                "1.40129846432481707092372958328991613128026194187651577175706828388979108268586060148663818836212158203125e-45mm"}) {
            DynamicPrintConfig cfg;
            if (std::string(initial) == "empty")
                cfg.set_key_value("nozzle_diameter", new ConfigOptionFloats());
            else if (std::string(initial) != "absent")
                cfg.set_deserialize_strict("nozzle_diameter", initial);
            apply_command_line_overrides(cfg, {{"nozzle_diameter", arg}}, false);
            nlohmann::json bits = nlohmann::json::array();
            if (auto *nozzles = cfg.option<ConfigOptionFloats>("nozzle_diameter")) {
                for (double value : nozzles->values) {
                    uint64_t raw;
                    std::memcpy(&raw, &value, sizeof(raw));
                    bits.push_back(raw);
                }
            }
            std::cout << nlohmann::json({{"initial", initial}, {"argument", arg}, {"bits", bits},
                                         {"present", cfg.has("nozzle_diameter")}}).dump() << '\n';
        }
    }
}
