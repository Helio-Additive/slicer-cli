// Link in place of the Bambu root main.cpp object and run beside its pinned
// resources/profiles tree. Arguments are a scratch directory and an output
// JSONL file. Calls main.cpp 5459–5517 and desktop_presets.cpp 393–424.
#define main slicer_reference_main
#include "main.cpp"
#undef main

int main(int argc, char **argv) {
    if (argc != 3) return 2;
    const auto file = (boost::filesystem::path(argv[1]) / "settings.json").string();
    boost::nowide::ofstream output(argv[2]);
    for (const auto& item : std::vector<std::pair<std::string, std::string>>{
             {"machine", "Bambu Lab X1 Carbon 0.6 nozzle"}, {"process", "0.20mm Standard @BBL X1C"},
             {"filament", "Bambu PLA Basic @BBL X1C"}, {"machine", "unshipped parent"},
             {"unknown", "Bambu Lab X1 Carbon 0.6 nozzle"}, {"machine", ""}}) {
        for (bool local : {false, true}) {
            json doc = {{"type", item.first}, {"inherits", item.second}, {"from", local ? "user" : "system"}};
            if (local) {
                doc["nozzle_diameter"] = {"0.40000123456789", "0.6"};
                doc["wall_loops"] = "7";
                doc["filament_diameter"] = {"1.85"};
            }
            { boost::nowide::ofstream settings(file); settings << doc; }
            Slic3r::DynamicPrintConfig config;
            apply_file_inherits(argv[0], file, "bundle", config, false);
            json selected = json::object();
            for (const char* key : {"inherits", "nozzle_diameter", "printer_model", "printer_variant",
                     "wall_loops", "top_shell_layers", "layer_height", "sparse_infill_density",
                     "filament_type", "filament_diameter", "filament_density", "nozzle_temperature",
                     "printable_area", "use_relative_e_distances", "machine_start_gcode"}) {
                if (!config.has(key)) continue;
                const std::string value = config.opt_serialize(key);
                if (std::string(key) == "machine_start_gcode") {
                    uint64_t hash = UINT64_C(14695981039346656037);
                    for (unsigned char c : value) hash = (hash ^ c) * UINT64_C(1099511628211);
                    selected[key] = {value.size(), hash};
                } else {
                    selected[key] = value;
                }
            }
            output << json({{"file", doc}, {"resolved", config.has("inherits")}, {"selected", selected}}).dump() << '\n';
        }
    }
    return output ? 0 : 1;
}
