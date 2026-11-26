// Compare the root Rust CLI's single-model calls with cli_command_line.cpp 465–571,
// 620, 751–752 and 781–782. Link with the Bambu CLI objects in place of main.cpp.
#include "cli_command_line.hpp"
#include <nlohmann/json.hpp>
#include <iostream>
#include <set>

static void sample(std::vector<std::string> words, bool geometry = false)
{
    using nlohmann::json;
    std::string program = "slicer_cli";
    std::vector<char*> argv{program.data()};
    for (auto& word : words)
        argv.push_back(word.data());
    const int argc = static_cast<int>(argv.size());
    argv.push_back(nullptr);
    CliOptions opts;
    slicer_cli::ModeArgs mode;
    slicer_cli::ParseRefusal refusal;
    const bool ok = slicer_cli::parse_command_line(argc, argv.data(), opts, mode, refusal);
    json row{{"args", words}};
    if (!ok)
        row["error"] = refusal.message;
    else if (mode.help)
        row["help"] = true;
    else {
        if (geometry) {
            row["overrides"] = opts.overrides;
            row["extra_config"] = json::object();
            for (const auto& key : opts.extra_config.keys())
                row["extra_config"][key] = opts.extra_config.opt_serialize(key);
        }
        row["input"] = opts.input_file;
        row["config"] = opts.bundle_config;
        row["output"] = opts.output_file;
        row["verbose"] = opts.verbose;
    }
    std::cout << row.dump() << '\n';
}

int main(int argc, char **argv)
{
    if (argc == 2 && std::string(argv[1]) == "--geometry-options") {
        for (const char* flag : {"--layer-height", "--infill", "--perimeters", "--wall-loops", "--sparse-infill-density", "--nozzle", "--temp", "--bed-temp"}) {
            for (const char* value : {"0.25", "35%", "3", "", "bad", "0.2mm", " -2", "0.55,0.6", "8e-2", "+3.5"}) {
                sample({flag, value, "model.stl"}, true);
                sample({std::string(flag) + "=" + value, "model.stl"}, true);
            }
            sample({"model.stl", flag}, true);
        }
        sample({"--layer-height=0.125", "--layer-height", "0.2", "model.stl"}, true);
        sample({"--infill=15", "--sparse-infill-density=23%", "--infill=27", "model.stl"}, true);
        sample({"--wall-loops=2", "--perimeters=3", "model.stl"}, true);
        sample({"--perimeters=3", "--wall-loops=2", "model.stl"}, true);
        sample({"--layer-height", "0.25", "--infill", "27", "--perimeters", "3", "model.stl"}, true);
        return 0;
    }
    // cli_command_line.cpp 388-422. A timing token consumed as the value
    // of a preceding switch never selects the early timing command.
    if (argc == 2 && std::string(argv[1]) == "--timing-options") {
        std::set<std::string> names;
        Slic3r::DynamicPrintAndCLIConfig config;
        for (const auto &[key, def] : config.def()->options)
            for (const auto &name : def.cli_args(key)) names.insert(name);
        for (const auto *name : {"o", "output", "v", "verbose", "input", "machine", "filament",
                "process", "config", "infill", "perimeters", "nozzle", "temp", "bed-temp", "plate",
                "calib-mode", "calib-start", "calib-end", "calib-step", "calib-extruder-id",
                "calib-no-numbers", "layout", "layout-plan", "progress", "printer-preset",
                "process-preset", "filament-preset", "list-presets", "list-settings", "list-flags",
                "printer", "allow-substitution", "engine-info", "single-instance", "simplify",
                "simplify-ratio", "simplify-triangles", "simplify-objects", "simplify-keep-paint",
                "no-normalize-legacy-gcode", "project-settings-as-saved", "unrecognized-name"})
            names.insert(name);
        for (const auto &name : names) {
            std::string program = "slicer_cli", flag = "--" + name, timing = "--gcode-timing";
            char *words[] = {program.data(), flag.data(), timing.data(), nullptr};
            std::cout << name << ' ' << slicer_cli::gcode_timing_requested(3, words) << '\n';
        }
        return 0;
    }
    sample({"--", "-cube.stl"});
    sample({"--", "--help"});
    sample({"--", "--"});
    sample({"-"});
    sample({"--", "-name=value.3mf"});
    sample({"--single-instance", "m.stl"});
    sample({"-single-instance=yes", "m.stl"});
    sample({"--single-instance", "--single-instance", "m.stl"});
    sample({"-config=first= profile.json", "--o", "out.gcode", "--v", "-input", "m.stl"});
    sample({"-config", "--", "-output=out.gcode", "-verbose", "m.stl"});
    sample({"--config", "--help", "--", "-w.stl"});
    sample({"--input", "--single-instance"});
    sample({"-input=-a.3mf"});
    sample({"-help"});
    sample({"--h"});
    sample({"--h=0"});
    sample({"--h="});
    sample({"--help=bad"});
    sample({"-h=01"});
    sample({"--help", "-config"});
    sample({"--v=false", "m.3mf"});
    sample({"-v", "--", "-"});
    sample({"-config=x", "--config", "y", "m.stl"});
    sample({"--input=", "-o", "out"});
    sample({"--single-instance", "--help"});
    sample({"--config=", "m.stl"});
    sample({"--help", "first", "second"});
    sample({"--output"});
    sample({"--config", "-psn_123", "m.stl"});
}
