#include "cli_setting_overrides.hpp"
#include "libslic3r/PrintConfig.hpp"

#include "cli_events.hpp"
#include <iostream>

namespace slicer_cli {

/// The command-line setting overrides (--layer-height, --fill-density, ...)
/// over `config`. The slice reports a rejected value; the --export-3mf
/// rebuild of the same settings stays quiet (the slice already said it).
void apply_command_line_overrides(Slic3r::DynamicPrintConfig& config,
                                         const std::map<std::string, std::string>& overrides,
                                         bool report_rejections) {
    for (const auto& [key, value] : overrides) {
        try {
            if (key == "layer_height") {
                config.set_key_value(key, new Slic3r::ConfigOptionFloat(std::stof(value)));
            } else if (key == "nozzle_diameter") {
                // nozzle_diameter is a list (one value per extruder); a single
                // Float there fails the type check of every later read
                // (Config.hpp 403). The printer's extruder count is that
                // list's size (support_different_extruders), so the value
                // goes into every entry and the list keeps its size: a
                // two-nozzle printer stays two nozzles. A value that is
                // already a list is taken as given.
                (void)std::stof(value);   // a bad number is refused as before
                size_t extruders = 1;
                if (const auto* nozzles = dynamic_cast<const Slic3r::ConfigOptionVectorBase*>(config.option(key)))
                    extruders = std::max<size_t>(1, nozzles->size());
                std::string values = value;
                for (size_t i = 1; i < extruders && value.find(',') == std::string::npos; ++i)
                    values += "," + value;
                config.set_deserialize_strict(key, values);
            } else if (key == "fill_density") {
                config.set_key_value(key, new Slic3r::ConfigOptionPercent(std::stoi(value)));
                config.set_key_value("sparse_infill_density", new Slic3r::ConfigOptionPercent(std::stoi(value)));
            } else if (key == "perimeters") {
                // Neither engine has a "perimeters" setting; the wall count is
                // wall_loops (PrintConfig.cpp 4383 at 5873b5f, 4918 at 31f6803).
                config.set_key_value("wall_loops", new Slic3r::ConfigOptionInt(std::stoi(value)));
            } else if (key == "nozzle_temperature") {
                const int v = std::stoi(value);
                // --temp: one temperature for the whole print, so every
                // filament slot the run uses is set, not just the first. A
                // printer with several extruders (the Snapmaker U1 has four,
                // loaded with four identical slots) otherwise slices with 0
                // on every slot after the first, which prints at no
                // temperature at all (Slic3r::GCode::_do_export writes each
                // extruder's own value, GCode.cpp 1430-1500, 3860-3875).
                // The list's size is the filament count the engine reads it
                // with (is_per_filament_config_key), so it keeps its size.
                if (auto* opt = dynamic_cast<Slic3r::ConfigOptionInts*>(config.option(key, true))) {
                    opt->values.assign(std::max<size_t>(1, opt->values.size()), v);
                } else if (Slic3r::print_config_def.get(key) != nullptr) {
                    // A setting the engine defines keeps its own type
                    // (BambuStudio's nozzle_temperature is nullable, and
                    // Model::setExtruderParams reads it as such).
                    config.set_deserialize_strict(key, std::to_string(v));
                } else {
                    config.set_key_value(key, new Slic3r::ConfigOptionInts({v}));
                }
            } else if (key == "bed_temperature") {
                const int v = std::stoi(value);
                // A setting the engine defines keeps its own type; a key it
                // does not define is kept as before.
                if (Slic3r::print_config_def.get(key) != nullptr)
                    config.set_deserialize_strict(key, std::to_string(v));
                else
                    config.set_key_value(key, new Slic3r::ConfigOptionInts({v}));
                // --bed-temp: the bed heat commands do not read
                // bed_temperature. They read the active plate type's own keys
                // (get_bed_temperature -> get_bed_temp_key, GCode.cpp 3866-3876
                // at 5873b5f / Print.cpp 1245-1260 at 31f6803), and the start
                // G-code's {bed_temperature} and {bed_temperature_initial_layer}
                // placeholders are overwritten with the same two keys before
                // they are expanded (GCode.cpp 2789-2793 at 5873b5f,
                // GCode.cpp 3347-3351 at f92a7c0). So the flag sets the plate
                // type the printer is on, both phases, exactly as the
                // desktop's bed temperature setting for that plate type does.
                const auto* bed = config.option<Slic3r::ConfigOptionEnum<Slic3r::BedType>>("curr_bed_type");
                if (bed != nullptr) {
                    for (const std::string& plate_key : {Slic3r::get_bed_temp_key(bed->value),
                                                         Slic3r::get_bed_temp_1st_layer_key(bed->value)}) {
                        if (plate_key.empty()) continue;
                        if (auto* opt = dynamic_cast<Slic3r::ConfigOptionInts*>(config.option(plate_key, true)))
                            opt->values.assign(std::max<size_t>(1, opt->values.size()), v);
                    }
                }
            }
        } catch (const std::exception& e) {
            if (!report_rejections) continue;
            emit_event({{"event","override_rejected"},
                        {"tag","InvalidOverrideValue"},
                        {"opt_key", key},
                        {"value", value},
                        {"message","Command-line override for '" + key + "' was rejected and had no effect"}});
            std::cerr << "Warning: Invalid value for " << key << ": " << value << "\n";
        }
    }
}

} // namespace slicer_cli
