// Run the Bambu root CLI's actual override function on typed configurations.
// Build with the slicer_cli target's flags and the repository root on the include path.
// Link this unit in place of main.cpp.o, discarding unused sections, and capture stdout.
#define main slicer_reference_main
#include "main.cpp"
#undef main

int main() {
    using namespace Slic3r;
    for (const char* plate : {"Default Plate", "Cool Plate", "Engineering Plate", "High Temp Plate", "Textured PEI Plate", "Supertack Plate"}) {
        for (const char* rows : {"", "191", "191,202,213", "empty", "nullable", "plain"}) {
            for (const char* operation : {"bed", "nozzle", "both", "invalid"}) {
                DynamicPrintConfig c;
                c.set_deserialize_strict("curr_bed_type", plate);
                const auto* bed = c.option<ConfigOptionEnum<BedType>>("curr_bed_type");
                const std::string regular = get_bed_temp_key(bed->value);
                const std::string first = get_bed_temp_1st_layer_key(bed->value);
                if (std::string(rows) == "empty") {
                    c.set_key_value("nozzle_temperature", new ConfigOptionIntsNullable());
                    if (!regular.empty()) c.set_key_value(regular, new ConfigOptionInts());
                    if (!first.empty()) c.set_key_value(first, new ConfigOptionInts());
                } else if (std::string(rows) == "nullable" || std::string(rows) == "plain") {
                    if (std::string(rows) == "nullable")
                        c.set_deserialize_strict("nozzle_temperature", "nil,202,nil");
                    else
                        c.set_key_value("nozzle_temperature", new ConfigOptionInts({190, 202, 210}));
                    c.set_deserialize_strict("nozzle_temperature_initial_layer", "221");
                    if (!regular.empty()) c.set_deserialize_strict(regular, "40,41,42");
                    if (!first.empty()) c.set_deserialize_strict(first, "55,59");
                } else if (*rows) {
                    for (const auto& key : std::vector<std::string>{"nozzle_temperature", "nozzle_temperature_initial_layer", regular, first})
                        if (!key.empty()) c.set_deserialize_strict(key, rows);
                }
                std::map<std::string,std::string> overrides;
                const std::string op(operation);
                if (op != "nozzle") overrides["bed_temperature"] = op == "invalid" ? "invalid" : "53 C";
                if (op != "bed") overrides["nozzle_temperature"] = op == "invalid" ? "invalid" : "  +222.9";
                apply_command_line_overrides(c, overrides, false);
                nlohmann::json values = nlohmann::json::object();
                for (const auto& key : c.keys()) values[key] = c.opt_serialize(key);
                std::cout << nlohmann::json({{"plate",plate},{"input",rows},{"operation",operation},{"values",values}}).dump() << '\n';
            }
        }
    }
}
