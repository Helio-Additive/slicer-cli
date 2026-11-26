// Build in place of main.cpp with the Bambu CLI compile/link flags.
// Run with inputs.jsonl on stdin to reproduce native.txt (main.cpp 540–610).
#define main slicer_cli_product_main
#include "main.cpp"
#undef main
int main()
{
    std::string line;
    while (std::getline(std::cin, line)) {
        const auto sample = nlohmann::json::parse(line);
        Slic3r::DynamicPrintConfig config;
        const auto& in = sample.at("settings");
        for (auto it = in.begin(); it != in.end(); ++it) {
            if (it->is_array())
                config.set_key_value(it.key(), new Slic3r::ConfigOptionStrings(it->get<std::vector<std::string>>()));
            else
                config.set_key_value(it.key(), new Slic3r::ConfigOptionString(it->get<std::string>()));
        }
        int count = sample.at("normalize").get<bool>() ?
            normalize_legacy_gcode_tokens(config, sample.at("report").get<bool>()) : 0;
        auto out = in;
        for (auto it = out.begin(); it != out.end(); ++it) {
            if (it->is_array())
                *it = config.opt<Slic3r::ConfigOptionStrings>(it.key())->values;
            else
                *it = config.opt_string(it.key());
        }
        std::cout << nlohmann::json{{"count",count},{"settings",out}}.dump() << '\n';
    }
}
