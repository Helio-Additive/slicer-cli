#pragma once

#include <string>
#include <vector>
#include <boost/filesystem/path.hpp>

namespace slicer_cli {

/// Which engine binary of this package has a printer model. "this" is the
/// build running; "other" its sibling in the same package, the one a
/// refusal names; "third" the package's remaining binary.
struct EngineFit {
    std::string this_app, this_binary, other_app, other_binary;
    bool this_catalog_found  = false;
    bool other_catalog_found = false;
    bool this_has  = false;
    bool other_has = false;
    std::string third_app, third_binary;
    boost::filesystem::path third_dir;
    bool third_catalog_found = false;
    bool third_has = false;
    /// The printer is one of Snapmaker's own (the Snapmaker vendor file of
    /// the Snapmaker Orca tree), whichever build runs.
    bool snapmaker_own_printer = false;
};

/// A printer preset this engine ships: the name a user gives to
/// --printer-preset, and the nozzle variant the preset is built for.
/// The desktop changes a printer's nozzle by choosing this preset, not by
/// editing a diameter (Sidebar::update_all_preset_comboboxes rebuilds the
/// printer list per nozzle variant, BambuStudio Plater.cpp 3340-3414 at
/// 5873b5f; OrcaSlicer Plater.cpp 3880-3909 at f92a7c0).
struct ShippedPrinterPreset {
    std::string variant;   ///< printer_variant: "0.4", or "0.4+0.6" on a mixed U1
    std::string name;      ///< the preset name, e.g. "Bambu Lab X1 Carbon 0.6 nozzle"
};

boost::filesystem::path engine_executable_dir(const char* argv0);
boost::filesystem::path engine_profiles_dir(const boost::filesystem::path& exe_dir);
boost::filesystem::path orca_profiles_dir(const boost::filesystem::path& exe_dir);
std::string preset_printer_model(const boost::filesystem::path& profiles_dir, const std::string& name);
EngineFit engine_fit_for(const std::string& argv0, const std::string& printer_model);
std::string engine_mismatch_sentence(const EngineFit& fit, const std::string& printer_model);
std::vector<ShippedPrinterPreset> shipped_printer_presets(const boost::filesystem::path& profiles_dir,
                                                                 const std::string& printer_model);
std::string shipped_printer_preset_of_variant(const std::vector<ShippedPrinterPreset>& presets,
                                                     const std::string& variant);
std::string preset_name_list(const std::vector<ShippedPrinterPreset>& presets, size_t max_names);

} // namespace slicer_cli
