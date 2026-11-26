// Compare the root Rust CLI's single-model calls with cli_command_line.cpp 465–571,
// 620, 751–752 and 781–782. Link with the Bambu CLI objects in place of main.cpp.
#include "cli_command_line.hpp"
#include <nlohmann/json.hpp>
#include <iostream>

static void sample(std::vector<std::string> words)
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
        row["input"] = opts.input_file;
        row["config"] = opts.bundle_config;
        row["output"] = opts.output_file;
        row["verbose"] = opts.verbose;
    }
    std::cout << row.dump() << '\n';
}

int main()
{
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
