#include "cli_filament_config.hpp"

#include <iostream>
#include <unordered_set>
#include "libslic3r/Preset.hpp"

namespace slicer_cli {

// When a 3MF already carries explicit per-filament physical nozzle assignments,
// recover the corresponding logical extruder map before slicing.  This avoids
// the auto-grouping path re-solving an already-constrained dual-nozzle setup
// into a different logical order than BambuStudio desktop.
// Returns true if it actually derived and applied a cross-nozzle filament_map.
// Filament roster the loaded project declares, 0 when unknown.  filament_colour
// is the authoritative roster (the driver seeds it with one entry, so a longer
// one can only come from the loaded project); the remaining per-filament
// identity vectors are taken as a floor too, because Print::apply() derives its
// own extruder count from filament_diameter (PrintApply.cpp:1504) and every
// per-region filament index it admits must be inside the arrays extended here.
// It sizes the per-filament alignment below on the Bambu build, the 3MF
// filament-index reset (PresetBundle.cpp 4127-4136, N = filament_presets.size())
// on the Orca build, and the Orca filament-vector normalization
// (normalize_filament_config_vectors, Preset.cpp 444-486), so it is not gated:
// one definition is what keeps those passes agreeing on what the roster is.
size_t project_filament_count(Slic3r::DynamicPrintConfig& config) {
    size_t count = 0;
    for (const char* key : {"filament_colour", "filament_settings_id", "filament_ids",
                            "filament_type", "filament_diameter"}) {
        // The vector base covers both the plain and the nullable vector spelling
        // of these keys (the two are unrelated types in this engine).
        if (const auto* vec = dynamic_cast<const Slic3r::ConfigOptionVectorBase*>(
                config.option(key, false)))
            count = std::max(count, vec->size());
    }
    return count;
}

#ifdef ENGINE_BAMBU
// ── BBS-only config-normalization helpers ───────────────────────────────────
// These helpers (apply_explicit_nozzle_mapping, set_default_config,
// ensure_vector_config_sizes) exist solely to coax
// the Bambu engine into accepting a non-Bambu printer config. Their bodies use
// Bambu-only config keys/enums (e.g. fmmNozzleManual, filament_extruder_variant),
// so they are compiled only for ENGINE_BAMBU and called only from the gated
// front-end blocks in main(). OrcaSlicer needs none of them.
// `explicit_config_supplied_nozzle_map` says whether the fully-overlaid config
// carries its own `filament_nozzle_map`, rather than the driver's seed. Bambu's own
// headless CLI never derives a filament map from a default: under an automatic
// mode it takes the mode from the plate/global config (BambuStudio.cpp:6662-6665)
// and lets ToolOrdering group the filaments (ToolOrdering.cpp:1897-1914); the
// engine reads filament_nozzle_map only under Nozzle Manual
// (ToolOrdering.cpp:1885-1889), a mode the CLI refuses on one-nozzle-per-
// extruder machines such as the X2D/H2D (BambuStudio.cpp:6841-6846); and the
// CLI-argument nozzle map is consumed only on multi-nozzle machines in a manual
// mode (BambuStudio.cpp:6789, 6836-6838).  Bambu Studio exports always carry
// the key (PresetBundle.cpp:61, written as zeros at 2090-2091), so real Bambu
// files keep their previous behaviour here.  Before this guard, the seed set_default_config()
// writes ({1}) was padded by the extruder-count normalisation to [1,0] and read
// back here as if it were the file's: a fresh STL (or any input with <=2 slots
// and no nozzle map) on a two-head machine was then forced to "Nozzle Manual"
// with filament_map [1,2], and every object was then moved to physical nozzle 0
// — on the X2D the Bowden head — instead of letting the engine's automatic
// grouping (ToolOrdering.cpp:1910-1914) pick the head.
/// Derives and applies a manual nozzle map only when input provenance is explicit.
bool apply_explicit_nozzle_mapping(
    Slic3r::DynamicPrintConfig& config,
    bool explicit_config_supplied_nozzle_map,
    bool verbose)
{
    // No explicit config nozzle map: there is nothing to apply. Leave the
    // engine in the mode the input asked for (Auto For Flush by default).
    if (!explicit_config_supplied_nozzle_map)
        return false;

    // If the plate-level filament_maps were already applied (mode set to "Nozzle Manual"
    // before calling this function), skip re-derivation — the mapping is already correct.
    {
        auto* mode_opt = config.option<Slic3r::ConfigOptionEnum<Slic3r::FilamentMapMode>>("filament_map_mode", false);
        if (mode_opt && mode_opt->value == Slic3r::FilamentMapMode::fmmNozzleManual)
            return false;
    }

    auto* filament_map = config.option<Slic3r::ConfigOptionInts>("filament_map", false);
    auto* filament_nozzle_map = config.option<Slic3r::ConfigOptionInts>("filament_nozzle_map", false);
    auto* physical_extruder_map = config.option<Slic3r::ConfigOptionInts>("physical_extruder_map", false);
    auto* nozzle_diameter = config.option<Slic3r::ConfigOptionFloats>("nozzle_diameter", false);
    if (!filament_map || !filament_nozzle_map || !physical_extruder_map || !nozzle_diameter)
        return false;

    const size_t filament_count = filament_map->values.size();
    const size_t extruder_count = nozzle_diameter->values.size();
    if (filament_count < 2 || extruder_count < 2)
        return false;
    if (filament_nozzle_map->values.size() < filament_count || physical_extruder_map->values.size() < extruder_count)
        return false;

    std::map<int, int> physical_to_logical;
    for (size_t logical_idx = 0; logical_idx < extruder_count; ++logical_idx) {
        // filament_map is 1-based.  Map physical nozzle -> 1-based logical extruder
        // index.  physical_extruder_map[logical_idx] gives the physical nozzle for
        // logical extruder logical_idx; we invert that to get physical -> logical.
        physical_to_logical[physical_extruder_map->values[logical_idx]] =
            static_cast<int>(logical_idx) + 1;
    }
    if (physical_to_logical.size() < extruder_count)
        return false;

    std::vector<int> derived_map = filament_map->values;
    for (size_t filament_idx = 0; filament_idx < filament_count; ++filament_idx) {
        auto it = physical_to_logical.find(filament_nozzle_map->values[filament_idx]);
        if (it == physical_to_logical.end())
            return false;
        derived_map[filament_idx] = it->second;
    }

    const bool uses_multiple_logical_extruders =
        std::adjacent_find(derived_map.begin(), derived_map.end(), std::not_equal_to<int>()) != derived_map.end();
    if (!uses_multiple_logical_extruders)
        return false;

    filament_map->values = derived_map;

    auto* filament_map_2 = config.option<Slic3r::ConfigOptionInts>("filament_map_2", true);
    filament_map_2->values.resize(derived_map.size());
    for (size_t i = 0; i < derived_map.size(); ++i)
        filament_map_2->values[i] = derived_map[i] - 1;

    // A manual mode keeps the derived map, which automatic grouping would
    // otherwise replace with its own.
    // Which manual mode follows the machine, as the official CLI decides: a
    // machine with several nozzles per extruder takes Nozzle Manual, and one
    // nozzle per extruder (H2D, X2D) takes Manual, since the official CLI
    // refuses Nozzle Manual there (support_multi_nozzle, BambuStudio.cpp
    // 3417 and 6841-6846 at 5873b5f).
    bool has_multiple_nozzles_per_extruder = false;
    if (auto* counts = config.option<Slic3r::ConfigOptionInts>("extruder_max_nozzle_count", false))
        has_multiple_nozzles_per_extruder = std::any_of(counts->values.begin(), counts->values.end(),
                                                        [](int count) { return count > 1; });
    const char* manual_mode = has_multiple_nozzles_per_extruder ? "Nozzle Manual" : "Manual";
    {
        Slic3r::ConfigSubstitutionContext substitution_context(Slic3r::ForwardCompatibilitySubstitutionRule::Enable);
        config.set_deserialize("filament_map_mode", manual_mode, substitution_context);
    }
    if (verbose) {
        std::cout << "Nozzle-map derivation: filament_map=[";
        for (size_t i = 0; i < derived_map.size(); ++i)
            std::cout << (i == 0 ? "" : ",") << derived_map[i];
        std::cout << "] mode=" << manual_mode << "\n";
    }
    return true;
}

static void initialize_filament_setting_defaults(Slic3r::DynamicPrintConfig& config) {
    // CRITICAL: Initialize ALL keys from filament_options_with_variant to prevent crashes
    // update_values_to_printer_extruders_for_multiple_filaments loops through these keys
    // and calls opt->get_at(variant_index) - if the option doesn't exist or is empty, it crashes/hangs
    /// PrintConfig.cpp:8155-8200
    /// C++: for (auto& key: key_set) { opt->get_at(variant_index[f_index]); }
    // These are the keys from the global filament_options_with_variant set

    auto init_filament_opt_float = [&config](const std::string& key, double default_val) {
        auto opt = config.option<Slic3r::ConfigOptionFloats>(key, true);
        if (!opt || opt->values.empty()) {
            config.set_key_value(key, new Slic3r::ConfigOptionFloats({default_val}));
        }
    };

    auto init_filament_opt_int = [&config](const std::string& key, int default_val) {
        auto opt = config.option<Slic3r::ConfigOptionInts>(key, true);
        if (!opt || opt->values.empty()) {
            config.set_key_value(key, new Slic3r::ConfigOptionInts({default_val}));
        }
    };

    auto init_filament_opt_bool = [&config](const std::string& key, bool default_val) {
        // First try nullable (most filament bool options are nullable)
        auto opt_n = config.option<Slic3r::ConfigOptionBoolsNullable>(key, false);
        if (opt_n) {
            if (opt_n->values.empty()) {
                opt_n->values.push_back((unsigned char)default_val);
            }
            return;
        }
        // Fallback to non-nullable
        auto opt = config.option<Slic3r::ConfigOptionBools>(key, false);
        if (opt) {
            if (opt->values.empty()) {
                opt->values.push_back((unsigned char)default_val);
            }
            return;
        }
        // Option doesn't exist yet — check if definition says nullable
        const auto* def = Slic3r::print_config_def.get(key);
        if (def && def->nullable) {
            config.set_key_value(key, new Slic3r::ConfigOptionBoolsNullable({(unsigned char)default_val}));
        } else {
            config.set_key_value(key, new Slic3r::ConfigOptionBools({default_val}));
        }
    };

    auto init_filament_opt_percent = [&config](const std::string& key, double default_val) {
        auto opt = config.option<Slic3r::ConfigOptionPercents>(key, true);
        if (!opt || opt->values.empty()) {
            config.set_key_value(key, new Slic3r::ConfigOptionPercents({default_val}));
        }
    };

    // Initialize all filament_options_with_variant keys with safe defaults
    // Use correct types: coFloats, coInts, coBools, coPercents
    init_filament_opt_float("filament_flow_ratio", 1.0);  // coFloats, ratio (1.0 = 100%)
    init_filament_opt_float("filament_max_volumetric_speed", 0.0);
    init_filament_opt_float("filament_ramming_volumetric_speed", 0.0);
    init_filament_opt_int("filament_pre_cooling_temperature", 0);
    init_filament_opt_float("filament_ramming_travel_time", 0.0);
    init_filament_opt_float("filament_ramming_volumetric_speed_nc", 0.0);
    init_filament_opt_int("filament_pre_cooling_temperature_nc", 0);
    init_filament_opt_float("filament_ramming_travel_time_nc", 0.0);
    // filament_extruder_variant already initialized above
    init_filament_opt_float("filament_retraction_length", 0.8);
    init_filament_opt_float("filament_retract_length_nc", 0.0);
    init_filament_opt_float("filament_z_hop", 0.0);
    // filament_z_hop_types is enum - skip for now
    init_filament_opt_float("filament_retract_restart_extra", 0.0);
    init_filament_opt_float("filament_retraction_speed", 20.0);
    init_filament_opt_float("filament_deretraction_speed", 20.0);
    init_filament_opt_float("filament_retraction_minimum_travel", 0.0);
    init_filament_opt_bool("filament_retract_when_changing_layer", false);
    init_filament_opt_bool("filament_wipe", false);
    init_filament_opt_float("filament_wipe_distance", 0.0);
    init_filament_opt_percent("filament_retract_before_wipe", 0.0);
    init_filament_opt_bool("filament_long_retractions_when_cut", false);
    init_filament_opt_float("filament_retraction_distances_when_cut", 0.0);
    init_filament_opt_bool("long_retractions_when_ec", false);
    init_filament_opt_float("retraction_distances_when_ec", 0.0);
    // nozzle_temperature and nozzle_temperature_initial_layer already initialized
    init_filament_opt_float("filament_flush_volumetric_speed", 0.0);

    // CRITICAL: Initialize nullable filament/overhang speed options
    // These are ConfigOptionBoolsNullable/FloatsNullable in PrintConfig and will crash
    // with SIGSEGV in PerimeterGenerator::is_enable_overhang_speed if empty
    // PerimeterGenerator.cpp:273-276
    // C++: bool use_filament_overhang_speed = perimeter_generator.print_config->override_process_overhang_speed.get_at(filament_idx);
    init_filament_opt_bool("override_process_overhang_speed", false);
    init_filament_opt_bool("filament_enable_overhang_speed", false);
    init_filament_opt_bool("filament_adaptive_volumetric_speed", false);
    init_filament_opt_float("filament_bridge_speed", 0.0);
    init_filament_opt_float("filament_overhang_1_4_speed", 0.0);
    init_filament_opt_float("filament_overhang_2_4_speed", 0.0);
    init_filament_opt_float("filament_overhang_3_4_speed", 0.0);
    init_filament_opt_float("filament_overhang_4_4_speed", 0.0);
    init_filament_opt_float("filament_overhang_totally_speed", 0.0);
}

static void initialize_printer_templates_and_temperatures(Slic3r::DynamicPrintConfig& config) {
    // Set printer technology
    /// PresetBundle.cpp:3149
    /// C++: out.option<ConfigOptionEnumGeneric>("printer_technology", true)->value = ptFFF;
    config.option<Slic3r::ConfigOptionEnumGeneric>("printer_technology", true)->value = Slic3r::ptFFF;

    // Set minimal G-code templates to prevent export crashes
    /// These are required for GCode::_do_export to work properly
    /// If empty, placeholder parser may fail
    if (!config.has("machine_start_gcode") || config.opt_string("machine_start_gcode").empty()) {
        config.set_key_value("machine_start_gcode", new Slic3r::ConfigOptionString(
            "; Minimal start G-code\n"
            "G28 ; home all axes\n"
            "G1 Z5 F5000 ; lift nozzle\n"
        ));
    }

    if (!config.has("machine_end_gcode") || config.opt_string("machine_end_gcode").empty()) {
        config.set_key_value("machine_end_gcode", new Slic3r::ConfigOptionString(
            "; Minimal end G-code\n"
            "G1 E-1 F300 ; retract\n"
            "G28 X0 Y0 ; home X Y\n"
            "M84 ; disable motors\n"
        ));
    }

    // Ensure temperature settings exist (prevent null pointer access)
    auto ensure_temp_opt = [&config](const std::string& key, int default_val) {
        if (!config.has(key)) {
            config.set_key_value(key, new Slic3r::ConfigOptionInts({default_val}));
        }
    };

    ensure_temp_opt("nozzle_temperature", 200);
    ensure_temp_opt("nozzle_temperature_initial_layer", 200);
    ensure_temp_opt("bed_temperature", 60);
    ensure_temp_opt("bed_temperature_initial_layer", 60);
}

// Initialize configuration with BambuStudio defaults
void set_default_config(Slic3r::DynamicPrintConfig& config) {
    /// Initialize config following BambuStudio's PresetBundle::full_fff_config pattern
    /// PresetBundle.cpp:2859-3150
    /// C++: out.apply(FullPrintConfig::defaults());

    // Start with full defaults - this ensures all keys exist with correct types
    config.apply(Slic3r::FullPrintConfig::defaults(), true);

    // Set preset IDs (mimics BambuStudio's preset tracking)
    /// PresetBundle.cpp:3125-3127
    /// C++: out.option<ConfigOptionString>("print_settings_id", true)->value = this->prints.get_selected_preset_name();
    config.set_key_value("print_settings_id", new Slic3r::ConfigOptionString(""));
    config.set_key_value("filament_settings_id", new Slic3r::ConfigOptionStrings({""}));
    config.set_key_value("printer_settings_id", new Slic3r::ConfigOptionString(""));

    // Initialize filament_map and filament_volume_map for single filament
    /// PresetBundle.cpp:2871-2875
    /// C++: std::vector<int> filament_maps = out.option<ConfigOptionInts>("filament_map")->values;
    size_t num_filaments = 1;
    std::vector<int> filament_maps(num_filaments, 1);
    std::vector<int> filament_volume_maps(num_filaments, 0); // nvtStandard = 0

    config.option<Slic3r::ConfigOptionInts>("filament_map", true)->values = filament_maps;
    config.option<Slic3r::ConfigOptionInts>("filament_volume_map", true)->values = filament_volume_maps;

    // Initialize filament_self_index (critical for multi-material code paths)
    /// PresetBundle.cpp:2964-2967
    /// C++: std::vector<int>& filament_self_indice = out.option<ConfigOptionInts>("filament_self_index", true)->values;
    /// C++: int index_size = out.option<ConfigOptionStrings>("filament_extruder_variant")->size();
    /// C++: filament_self_indice.resize(index_size, 1);
    auto filament_variant = config.option<Slic3r::ConfigOptionStrings>("filament_extruder_variant", true);
    if (filament_variant) {
        int index_size = filament_variant->values.size();
        if (index_size == 0) {
            index_size = 1;
            // CRITICAL: Must use valid variant string matching get_extruder_variant_string output
            // get_index_for_extruder compares against get_extruder_variant_string result
            // Format is: "<ExtruderType> <NozzleVolumeType>"
            // Default: etDirectDrive (0) + nvtStandard (0) = "Direct Drive Standard"
            /// PrintConfig.cpp:499-501
            /// C++: variant_string = s_keys_names_ExtruderType[extruder_type];
            /// C++: variant_string+= " ";
            /// C++: variant_string+= s_keys_names_NozzleVolumeType[nozzle_volume_type];
            /// PrintConfig.cpp:7255-7269
            /// C++: std::string extruder_variant = get_extruder_variant_string(extruder_type, nozzle_volume_type);
            /// C++: for (int index = 0; index < v_size; index++) { if (extruder_variant == variant) { ... } }
            filament_variant->values.resize(1, "Direct Drive Standard");
        }
        config.option<Slic3r::ConfigOptionInts>("filament_self_index", true)->values.resize(index_size, 1);
    }

    // Ensure support_filament and support_interface_filament are within bounds
    /// PresetBundle.cpp:3117-3122
    /// C++: auto *opt = dynamic_cast<ConfigOptionInt*>(out.option(key, false));
    /// C++: opt->value = boost::algorithm::clamp<int>(opt->value, 0, int(num_filaments));
    auto support_fil = config.option<Slic3r::ConfigOptionInt>("support_filament");
    if (support_fil) {
        support_fil->value = std::max(0, std::min(support_fil->value, (int)num_filaments));
    }
    auto support_iface = config.option<Slic3r::ConfigOptionInt>("support_interface_filament");
    if (support_iface) {
        support_iface->value = std::max(0, std::min(support_iface->value, (int)num_filaments));
    }

    // BYPASS STRATEGY: Pre-initialize filament maps to match what Print::process() will compute
    // This prevents the "filament maps changed" condition at Print.cpp:2670 from triggering
    // the problematic update_values_to_printer_extruders_for_multiple_filaments call
    /// Print.cpp:2670
    /// C++: if ((m_config.filament_map.values != f_maps) || (m_config.filament_volume_map.values != f_volume_maps) || ...)
    /// By setting these correctly up front, the condition will be FALSE and skip the hang

    // For single extruder setup:
    // - filament_map: which extruder each filament uses (1-indexed)
    // - filament_volume_map: nozzle volume type for each filament (0 = nvtStandard)
    // - filament_nozzle_map: nozzle mapping (typically matches filament_map)

    // Ensure filament_map is exactly what the analysis expects
    auto filament_map_opt = config.option<Slic3r::ConfigOptionInts>("filament_map", true);
    if (filament_map_opt->values.empty() || filament_map_opt->values[0] != 1) {
        filament_map_opt->values = {1};  // Single filament uses extruder 1
    }

    // Ensure filament_volume_map matches extruder nozzle types
    auto filament_volume_map_opt = config.option<Slic3r::ConfigOptionInts>("filament_volume_map", true);
    if (filament_volume_map_opt->values.empty() || filament_volume_map_opt->values[0] != 0) {
        filament_volume_map_opt->values = {0};  // nvtStandard
    }

    // Ensure filament_nozzle_map is set (may prevent other code paths)
    auto filament_nozzle_map_opt = config.option<Slic3r::ConfigOptionInts>("filament_nozzle_map", true);
    if (filament_nozzle_map_opt->values.empty()) {
        filament_nozzle_map_opt->values = {1};
    }

    // Ensure filament_map_2 is initialized (used in Print.cpp:2707)
    // IMPORTANT: filament_map_2 is a 0-based extruder *index*, unlike filament_map
    // which is a 1-based extruder *number*.
    // print.process() computes it via:
    //   get_index_for_extruder(filament_map[i], "print_extruder_id", ...)
    // For a single extruder: filament_map[0]=1, print_extruder_id={1} → index 0.
    // If we leave this at {1} the bypass condition
    //   m_config.filament_map_2 != f_maps_2   ({1} != {0})
    // fires even though all other maps match, triggering
    // update_values_to_printer_extruders_for_multiple_filaments which then
    // crashes with extruder_nozzle_volume_count=0 / "different nozzle volume processing".
    auto filament_map_2_opt = config.option<Slic3r::ConfigOptionInts>("filament_map_2", true);
    if (filament_map_2_opt->values.empty()) {
        filament_map_2_opt->values = {0};  // 0-based: extruder 1 is at index 0
    }

    // CRITICAL: Initialize print_extruder_variant and print_extruder_id
    // These are needed by get_index_for_extruder when processing filament_map_2
    /// Print.cpp:2707
    /// C++: m_config.filament_map_2.values[index] = m_ori_full_print_config.get_index_for_extruder(f_maps[index], "print_extruder_id", ...)
    /// PrintConfig.cpp:7248
    /// C++: const ConfigOptionInts* id_opt = id_name.empty()?nullptr: dynamic_cast<const ConfigOptionInts*>(this->option(id_name));

    // print_extruder_variant: matches filament_extruder_variant format
    auto print_extruder_variant_opt = config.option<Slic3r::ConfigOptionStrings>("print_extruder_variant", true);
    if (!print_extruder_variant_opt || print_extruder_variant_opt->values.empty()) {
        config.set_key_value("print_extruder_variant", new Slic3r::ConfigOptionStrings({"Direct Drive Standard"}));
    }

    // print_extruder_id: IDs for matching (1 for single extruder)
    auto print_extruder_id_opt = config.option<Slic3r::ConfigOptionInts>("print_extruder_id", true);
    if (!print_extruder_id_opt || print_extruder_id_opt->values.empty()) {
        config.set_key_value("print_extruder_id", new Slic3r::ConfigOptionInts({1}));
    }

    initialize_filament_setting_defaults(config);

    initialize_printer_templates_and_temperatures(config);
}

// Ensure critical vector options have minimum sizes
// Call this AFTER loading JSON configs to fix any missing/empty vectors
void ensure_vector_config_sizes(Slic3r::DynamicPrintConfig& config) {
    // Helper: Ensure vector config options exist with at least min_size elements
    // This prevents null pointer crashes in multi-extruder code
    auto ensure_vector_option = [&config](const std::string& key, size_t min_size, const std::string& default_val = "") {
        auto* opt = config.option(key, true);  // Create if doesn't exist
        if (!opt) return;

        if (auto vec_opt = dynamic_cast<Slic3r::ConfigOptionInts*>(opt)) {
            while (vec_opt->values.size() < min_size) {
                vec_opt->values.push_back(vec_opt->values.empty() ? 1 : vec_opt->values[0]);
            }
        } else if (auto vec_opt = dynamic_cast<Slic3r::ConfigOptionFloats*>(opt)) {
            while (vec_opt->values.size() < min_size) {
                vec_opt->values.push_back(vec_opt->values.empty() ? 0.0 : vec_opt->values[0]);
            }
        } else if (auto vec_opt = dynamic_cast<Slic3r::ConfigOptionStrings*>(opt)) {
            while (vec_opt->values.size() < min_size) {
                vec_opt->values.push_back(default_val.empty() ? "" : default_val);
            }
        } else if (auto vec_opt = dynamic_cast<Slic3r::ConfigOptionEnumsGeneric*>(opt)) {
            while (vec_opt->values.size() < min_size) {
                vec_opt->values.push_back(vec_opt->values.empty() ? 0 : vec_opt->values[0]);
            }
        } else if (auto vec_opt = dynamic_cast<Slic3r::ConfigOptionBoolsNullable*>(opt)) {
            // Must check nullable BEFORE non-nullable (nullable doesn't inherit from non-nullable)
            while (vec_opt->values.size() < min_size) {
                vec_opt->values.push_back(vec_opt->values.empty() ? (unsigned char)0 : vec_opt->values[0]);
            }
        } else if (auto vec_opt = dynamic_cast<Slic3r::ConfigOptionBools*>(opt)) {
            while (vec_opt->values.size() < min_size) {
                vec_opt->values.push_back(vec_opt->values.empty() ? (unsigned char)0 : vec_opt->values[0]);
            }
        } else if (auto vec_opt = dynamic_cast<Slic3r::ConfigOptionPercentsNullable*>(opt)) {
            while (vec_opt->values.size() < min_size) {
                vec_opt->values.push_back(vec_opt->values.empty() ? 0.0 : vec_opt->values[0]);
            }
        } else if (auto vec_opt = dynamic_cast<Slic3r::ConfigOptionPercents*>(opt)) {
            while (vec_opt->values.size() < min_size) {
                vec_opt->values.push_back(vec_opt->values.empty() ? 0.0 : vec_opt->values[0]);
            }
        } else if (auto vec_opt = dynamic_cast<Slic3r::ConfigOptionFloatsNullable*>(opt)) {
            while (vec_opt->values.size() < min_size) {
                vec_opt->values.push_back(vec_opt->values.empty() ? 0.0 : vec_opt->values[0]);
            }
        } else if (auto vec_opt = dynamic_cast<Slic3r::ConfigOptionIntsNullable*>(opt)) {
            while (vec_opt->values.size() < min_size) {
                vec_opt->values.push_back(vec_opt->values.empty() ? 0 : vec_opt->values[0]);
            }
        }
    };

    // Ensure all critical vector options have at least 1 element
    // This prevents crashes in multi-extruder code when accessing .get_at(0)
    ensure_vector_option("nozzle_diameter", 1);
    ensure_vector_option("filament_diameter", 1);
    ensure_vector_option("filament_type", 1, "PLA");
    ensure_vector_option("filament_colour", 1, "#FFFFFF");

    // CRITICAL: extruder_type and nozzle_volume_type must be explicitly initialized
    // update_values_to_printer_extruders_for_multiple_filaments crashes if these don't exist or are empty
    /// PrintConfig.cpp:8114-8115, 8129
    /// C++: auto opt_extruder_type = dynamic_cast<const ConfigOptionEnumsGeneric*>(printer_config.option("extruder_type"));
    /// C++: ExtruderType extruder_type = (ExtruderType)(opt_extruder_type->get_at(filament_maps[f_index] - 1));
    // Explicitly create with default values: etDirectDrive (0) and nvtStandard (0)
    if (!config.has("extruder_type") || config.option<Slic3r::ConfigOptionEnumsGeneric>("extruder_type")->values.empty()) {
        config.set_key_value("extruder_type", new Slic3r::ConfigOptionEnumsGeneric({0})); // etDirectDrive
    }
    if (!config.has("nozzle_volume_type") || config.option<Slic3r::ConfigOptionEnumsGeneric>("nozzle_volume_type")->values.empty()) {
        config.set_key_value("nozzle_volume_type", new Slic3r::ConfigOptionEnumsGeneric({0})); // nvtStandard
    }
    // Size per-extruder arrays to match the actual extruder count.
    // ToolOrdering::get_recommended_filament_maps() iterates 0..extruder_nums and
    // accesses nozzle_volume_type.values[idx] and extruder_max_nozzle_count.values[idx]
    // directly (not via get_at()).  On multi-nozzle printers (e.g. H2D with 2 nozzles)
    // these vectors default to size 1, so idx=1 is an OOB read.  On Linux the adjacent
    // heap memory contains allocator metadata; interpreted as a nozzle_count loop bound
    // in build_nozzle_list() this creates thousands of bogus NozzleInfo entries, corrupts
    // the heap, and ultimately crashes as a bad memcpy inside std::string::_M_assign.
    // macOS happens to read zeros there (plausible values) and never trips.
    {
        size_t n_extruders = 1;
        if (auto* nd = config.option<Slic3r::ConfigOptionFloats>("nozzle_diameter"))
            n_extruders = std::max(n_extruders, nd->values.size());
        ensure_vector_option("extruder_type",             n_extruders);
        ensure_vector_option("nozzle_volume_type",        n_extruders);
        ensure_vector_option("extruder_max_nozzle_count", n_extruders);
    }

    // These are already set above in bypass strategy, but ensure they're at least size 1
    ensure_vector_option("filament_map", 1);
    ensure_vector_option("filament_volume_map", 1);
    ensure_vector_option("filament_nozzle_map", 1);
    // filament_map_2 is a 0-based index — pad with 0, not 1.
    {
        auto* opt = config.option<Slic3r::ConfigOptionInts>("filament_map_2", true);
        if (opt && opt->values.empty()) opt->values.push_back(0);
    }
    ensure_vector_option("print_extruder_variant", 1, "Direct Drive Standard");
    ensure_vector_option("print_extruder_id", 1);

    // Required by calc_estimated_filament_print_time()
    // Fallback must be a valid variant string, not a nozzle-diameter string.
    // "Direct Drive Standard" matches get_extruder_variant_string(etDirectDrive, nvtStandard).
    ensure_vector_option("filament_extruder_variant", 1, "Direct Drive Standard");
    ensure_vector_option("filament_self_index", 1);
    ensure_vector_option("filament_max_volumetric_speed", 1);
    ensure_vector_option("filament_flow_ratio", 1);
    ensure_vector_option("printer_extruder_variant", 1, "0.4");

    // Additional printer options that may be accessed
    ensure_vector_option("retract_length", 1);
    ensure_vector_option("retract_lift", 1);
    ensure_vector_option("temperature", 1);
    ensure_vector_option("nozzle_temperature", 1);

    // CRITICAL: Ensure all nullable bool/float filament overhang options have at least 1 element
    // Without these, PerimeterGenerator::is_enable_overhang_speed crashes (SIGSEGV)
    // when calling get_at() on an empty vector
    ensure_vector_option("override_process_overhang_speed", 1);
    ensure_vector_option("filament_enable_overhang_speed", 1);
    ensure_vector_option("filament_adaptive_volumetric_speed", 1);
    ensure_vector_option("filament_bridge_speed", 1);
    ensure_vector_option("filament_overhang_1_4_speed", 1);
    ensure_vector_option("filament_overhang_2_4_speed", 1);
    ensure_vector_option("filament_overhang_3_4_speed", 1);
    ensure_vector_option("filament_overhang_4_4_speed", 1);
    ensure_vector_option("filament_overhang_totally_speed", 1);

    // Other nullable vector options that may be accessed during slicing
    ensure_vector_option("enable_overhang_speed", 1);
    ensure_vector_option("enable_height_slowdown", 1);
    ensure_vector_option("long_retractions_when_cut", 1);
    ensure_vector_option("long_retractions_when_ec", 1);
    ensure_vector_option("retract_before_wipe", 1);
    ensure_vector_option("retraction_length", 1);
    ensure_vector_option("retraction_speed", 1);
    ensure_vector_option("deretraction_speed", 1);
    ensure_vector_option("z_hop", 1);
    ensure_vector_option("travel_speed", 1);
    ensure_vector_option("travel_speed_z", 1);
    ensure_vector_option("outer_wall_speed", 1);
    ensure_vector_option("inner_wall_speed", 1);
    ensure_vector_option("sparse_infill_speed", 1);
    ensure_vector_option("internal_solid_infill_speed", 1);
    ensure_vector_option("top_surface_speed", 1);
    ensure_vector_option("gap_infill_speed", 1);
    ensure_vector_option("support_speed", 1);
    ensure_vector_option("support_interface_speed", 1);
    ensure_vector_option("bridge_speed", 1);
    ensure_vector_option("overhang_totally_speed", 1);
    ensure_vector_option("overhang_1_4_speed", 1);
    ensure_vector_option("overhang_2_4_speed", 1);
    ensure_vector_option("overhang_3_4_speed", 1);
    ensure_vector_option("overhang_4_4_speed", 1);
    ensure_vector_option("filament_flow_ratio", 1);
    ensure_vector_option("filament_max_volumetric_speed", 1);
    ensure_vector_option("filament_ramming_volumetric_speed", 1);
    ensure_vector_option("filament_flush_volumetric_speed", 1);

    // filament_printable is a per-filament bitmask: bit N = the filament can be
    // printed on nozzle N (0-based). The engine reads it when it groups the
    // filaments onto the nozzles (check_filament_printable_after_group,
    // ToolOrdering.cpp 87-99 at 5873b5f) and when it lists the nozzles a
    // filament cannot use (Print::get_physical_unprintable_filaments,
    // Print.cpp 3042-3057). The values are the filament presets' own (a
    // filament option, Preset.cpp 1088; written per filament by
    // PresetBundle::full_config) or the project's, and they are kept as they
    // are, as the desktop app keeps them. Only a missing or empty option takes
    // the definition default {3}, both nozzles (PrintConfig.cpp 2691-2695);
    // the final per-filament alignment then sizes it to the roster
    // (align_per_filament_config_vectors, front value duplicated as get_at()
    // reads it).
    //
    // Every entry used to be overwritten with INT_MAX here, which is the nil
    // marker of an int list (ConfigOptionIntsTempl::nil_value, Config.hpp
    // 1051), so store_bbs_3mf refused the project ("Serializing NaN",
    // Config.hpp 1109-1113) and every slice ignored the presets' nozzle limits. Its reason, "Grouping error:
    // filament1 can not be placed in the right nozzle", came from the
    // extruder-count padding appending a 0 to this array; that pass now skips
    // every per-filament key (is_per_filament_config_key), so no 0 is added.
    {
        auto* fp = config.option<Slic3r::ConfigOptionInts>("filament_printable", false);
        if (fp == nullptr || fp->values.empty())
            config.set_key_value("filament_printable",
                Slic3r::print_config_def.get("filament_printable")->default_value->clone());
    }
}

// ── Per-filament array alignment (post-load / pre-apply) ─────────────────────
//
// A project 3MF names the loaded filament roster in its
// Metadata/project_settings.config — one entry per spool in filament_colour,
// filament_settings_id, filament_ids and filament_type — while the per-filament
// arrays themselves can still hold the single-slot value this driver seeds.
// BambuStudio's GUI reshapes that flat configuration through
// PresetBundle::full_fff_config(), which fans each filament preset out into the
// assembled vectors (PresetBundle.cpp:161-245); the CLI has no equivalent step
// when profile resolution falls back to the flat 3MF config.
//
// Print::apply() only re-derives those arrays when
// (extruder_count > 1) || support_different_extruders() (PrintApply.cpp:1327-1345,
// via update_values_to_printer_extruders_for_multiple_filaments at
// PrintConfig.cpp:8664).  On a one-nozzle / one-variant machine neither holds,
// so filament_bridge_speed keeps a single element while filament indices run
// 0..filament_count-1.  Consumers that index it with operator[] then read past
// the end of the heap allocation (PrintObject.cpp:366 —
// `print_config.filament_bridge_speed.values[filament_id]`): harmless-looking on
// Linux/macOS, an immediate access violation on Windows with PageHeap enabled.
//
// Every array is only ever EXTENDED, by duplicating its front element — exactly
// the value ConfigOptionVectorBase::get_at() yields out of range
// (Config.hpp:681 clamps to front(); ConfigOptionVector<T>::resize() duplicates
// it, Config.hpp:719).  A config reshaped this way therefore behaves identically
// for every get_at() consumer and for the engine's own variant fan-out; only raw
// `values[i]` indexing changes, from undefined to the value get_at() would have
// returned.  Nothing is truncated, nothing is emptied, and no synthetic zero is
// substituted for a value the project did not have: a populated array only ever
// propagates a value it already carries.

// True for the config keys whose arrays carry one entry per FILAMENT rather than
// one per extruder, nozzle or variant slot.  One predicate gates both passes that
// touch these arrays — the extruder-count padding in the front end and the roster
// alignment here — so the two can never disagree about which keys are
// per-filament, the way two hand-maintained key lists drifted apart before.
//
// Per-filament arrays are spelled filament_* (PrintConfig.cpp option table).  Their
// printer-side twins — nozzle_diameter, retraction_length, z_hop, extruder_type,
// nozzle_volume_type, … — carry no prefix and live in a different key set
// (printer_extruder_options / printer_options_with_variant_*, PrintConfig.cpp:7290-7345),
// so this filter can never reach a per-nozzle or per-extruder array and cannot
// flatten a multi-nozzle configuration.
//
// Excluded spellings, each for its own reason:
//   filament_extruder_variant — the variant table; its length is the number
//       of (extruder × nozzle volume) slots that get_parameter_size() sizes
//       printer arrays by (PrintConfig.cpp:7592), so it must not be fanned
//       out to the filament count;
//   filament_self_index — the 1..N owner of the slots in that same table;
//       get_config_index_base() asserts that the two lengths match
//       (PrintConfig.cpp:571) and PrintObject.cpp:342-345 walks one while
//       indexing the other, so it is only extended to a table that already
//       covers the roster (below), never beyond it;
//   filament_map / filament_map_2 — per filament, but 1-based / 0-based
//       indices rather than plain values (aligned by align_per_filament_maps).
//
// Outside the filament_ namespace: the engine's own filament_options_with_variant
// (PrintConfig.cpp 7241-7290 at 5873b5f, 8187 at 31f6803), the filament
// settings the engine fans out per filament and variant (nozzle_temperature,
// long_retractions_when_ec, ...).
static bool is_per_filament_config_key(const std::string& key) {
    static const char* const kSpecialFilamentKeys[] = {
        "filament_extruder_variant", "filament_self_index",
        "filament_map", "filament_map_2",
    };

    if (key.rfind("filament_", 0) == 0) {
        for (const char* special : kSpecialFilamentKeys)
            if (key == special)
                return false;
        return true;
    }
    // Snapmaker Orca has no filament_options_with_variant set (its OrcaSlicer
    // base predates the per-extruder variant tables), so nothing is excluded
    // there and every remaining filament_ key is per filament.
#ifndef ENGINE_SNAPMAKER
    return Slic3r::filament_options_with_variant.count(key) > 0;
#else
    return false;
#endif
}

// True for every setting a filament preset carries: the keys above plus the
// rest of the engine's own list of filament settings, Preset::filament_options()
// (Preset.cpp 1087-1140 at 5873b5f, 1266 at 31f6803): bed temperatures, fan
// speeds, cooling, hole and counter compensation, ... The desktop writes them
// one per filament (PresetBundle::full_fff_config), and Preset::normalize sizes
// them to the filament count (Preset.cpp 457-466) less the two it skips there,
// compatible_prints and compatible_printers (lists of names). None of them has
// one value per extruder, so the extruder-count padding leaves them alone; a
// hand-written list missed most of them, and a one-filament H2D project got
// hot_plate_temp = 60,0 and fan_max_speed = 100,0. The roster alignment keeps
// to is_per_filament_config_key: the desktop's own projects carry these the
// way the official CLI slices them, and it does not extend them either.
bool is_filament_setting_key(const std::string& key) {
    if (is_per_filament_config_key(key))
        return true;
    static const std::unordered_set<std::string> engine_filament_keys = [] {
        std::unordered_set<std::string> keys(Slic3r::Preset::filament_options().begin(),
                                             Slic3r::Preset::filament_options().end());
        keys.erase("compatible_prints");
        keys.erase("compatible_printers");
        for (const char* special : {"filament_extruder_variant", "filament_self_index", "filament_map", "filament_map_2"})
            keys.erase(special);
        return keys;
    }();
    return engine_filament_keys.count(key) > 0;
}

// Cardinality of the three per-filament index maps — filament_map, filament_map_2
// and filament_nozzle_map — fixed against the roster the project declares, never
// against the machine's head count.  They share the filament axis, so padding any
// of them to the nozzle count invents entries for filaments no identity key
// declares, and the engine then believes in them: it reads filament_map's LENGTH
// as the filament count and fans every per-filament array out to it
// (update_values_to_printer_extruders_for_multiple_filaments, PrintConfig.cpp:8134
// — `size_t filament_count = filament_maps.size()`).  A map longer than the
// roster therefore manufactures the phantom filament downstream, which is why the
// alignment only ever extends toward filament_count and never toward nozzle_count.
//
// Called twice: once before the nozzle-map derivation — which reads filament_map's
// length as the filament count and would otherwise refuse to derive a split the
// project declares, because the driver had seeded the map with a single entry —
// and once in the final alignment, because the plate overlay and the command line
// can rewrite the maps after that.
size_t align_per_filament_maps(Slic3r::DynamicPrintConfig& config, size_t filament_count) {
    if (filament_count <= 1)
        return 0;   // one slot: the flat config already is the whole map

    size_t extended = 0;

    // filament_map maps each filament to its 1-based extruder.  An entry below 1
    // is not a valid extruder and would be read as (size_t)-1 downstream, so
    // repair it onto the first extruder before replicating — the extension must
    // not spread an invalid 0.  Valid entries (including a plate map that points
    // at a different nozzle) are preserved exactly.
    auto* filament_map = config.option<Slic3r::ConfigOptionInts>("filament_map", false);
    if (filament_map && !filament_map->values.empty()) {
        for (int& value : filament_map->values)
            value = std::max(1, value);
        if (filament_map->values.size() < filament_count) {
            filament_map->values.resize(filament_count, filament_map->values.front());
            ++extended;
        }
    }

    // filament_map_2 is the 0-based index mirror of filament_map (see the plate
    // overlay in the driver and PrintApply.cpp:2707).  Existing entries are kept;
    // the entries added here mirror the map that was just aligned.
    auto* filament_map_2 = config.option<Slic3r::ConfigOptionInts>("filament_map_2", false);
    if (filament_map_2 && filament_map && !filament_map->values.empty() &&
        filament_map_2->values.size() < filament_count) {
        const size_t known = filament_map_2->values.size();
        filament_map_2->values.resize(filament_count, 0);
        for (size_t i = known; i < filament_count; ++i)
            filament_map_2->values[i] = std::max(0, filament_map->get_at(i) - 1);
        ++extended;
    }

    // filament_nozzle_map is the per-filament physical nozzle — the same filament
    // axis as filament_map, and equally exempt from the extruder-count padding.
    // ToolOrdering's Nozzle Manual grouping indexes it by every USED filament
    // (MultiNozzleUtils.cpp:240, req_extruder / input_nozzle_idx), so a map
    // shorter than the roster is an out-of-range read.  Grown with the front
    // value, the fallback get_at() and the engine's own short-map resize use
    // (GCodeProcessor.cpp:1952).
    auto* filament_nozzle_map = config.option<Slic3r::ConfigOptionInts>("filament_nozzle_map", false);
    if (filament_nozzle_map && !filament_nozzle_map->values.empty() &&
        filament_nozzle_map->values.size() < filament_count) {
        filament_nozzle_map->values.resize(filament_count, filament_nozzle_map->values.front());
        ++extended;
    }

    return extended;
}

static void align_filament_flush_arrays(Slic3r::DynamicPrintConfig& config, size_t filament_count,
        size_t& extended) {
    // Flush arrays: two purge values per filament, and an N*N*nozzles matrix the
    // wipe-tower generator reads the filament count back out of.  Mirrors
    // PresetBundle::update_multi_material_filament_presets()
    // (PresetBundle.cpp:5542-5585): extend the volume vector to 2*N, and reshape
    // the matrix only when its size does not already match — the stock 4x4 matrix
    // and eight-value vector of a four-filament project are left untouched.
    auto* flush_vector = config.option<Slic3r::ConfigOptionFloats>("flush_volumes_vector", false);
    auto* flush_matrix = config.option<Slic3r::ConfigOptionFloats>("flush_volumes_matrix", false);
    if (flush_vector && !flush_vector->values.empty() &&
        flush_vector->values.size() < 2 * filament_count) {
        // The matrix pairing below reads the vector positionally — even entries
        // from the outgoing filament, odd from the incoming one — so an
        // odd-length vector has to have its half pair completed before whole
        // pairs are appended.  Appending pairs to an odd total overshoots 2*N by
        // one and leaves a stray value the wipe-tower generator never paired
        // with a filament.
        const double unload = flush_vector->values[0];
        const double load   = flush_vector->values.size() > 1 ? flush_vector->values[1] : 140.;
        if (flush_vector->values.size() % 2 != 0)
            flush_vector->values.push_back(load);
        while (flush_vector->values.size() + 2 <= 2 * filament_count) {
            flush_vector->values.push_back(unload);
            flush_vector->values.push_back(load);
        }
        ++extended;
    }
    if (flush_matrix && flush_vector && flush_vector->values.size() >= 2 * filament_count &&
        flush_matrix->values.size() > 0) {
        // Per-nozzle count.  nozzle_diameter is declared coFloats
        // (PrintConfig.cpp:4245) and is stored as ConfigOptionFloats, not the
        // nullable spelling — casting to the nullable sibling yields nullptr and
        // silently collapses a multi-nozzle machine to one nozzle.
        size_t nozzle_count = 1;
        if (const auto* nd = config.option<Slic3r::ConfigOptionFloats>("nozzle_diameter", false))
            nozzle_count = std::max<size_t>(1, nd->values.size());
        const size_t matrix_target = filament_count * filament_count * nozzle_count;

        if (flush_matrix->values.size() < matrix_target) {
            // Shape of the matrix already saved in the project, so the pairs it
            // does describe can be carried over unchanged.  The matrix is one
            // N*N block per nozzle, block after block (get_flush_volumes_matrix,
            // PrintConfig.hpp:1945; GCode.cpp:4752,8090 read it that way), so its
            // LENGTH is what fixes N, and the nozzle count is the machine's own
            // head count — the dimension this reshape is building toward.
            // flush_multiplier is deliberately NOT read here: its length is a
            // per-head hint that a saved project does not keep in step with the
            // matrix (a two-entry multiplier can sit beside a one-nozzle 2x2
            // matrix), and decomposing the matrix by it would read those four
            // saved values as two 1x1 blocks and drop every one of them.
            size_t old_filament_count = 0;
            size_t old_nozzle_count   = 1;
            {
                const size_t old_size = flush_matrix->values.size();

                auto exact_square_root = [](size_t value, size_t& root) {
                    root = 1;
                    while (root * root < value)
                        ++root;
                    return root * root == value;
                };

                size_t square_root   = 1;
                const bool whole_is_square = exact_square_root(old_size, square_root);
                size_t per_head_root = 1;
                const bool per_head_blocks =
                    nozzle_count > 1 && old_size % nozzle_count == 0 &&
                    exact_square_root(old_size / nozzle_count, per_head_root) && per_head_root > 1;

                // Diagonal zero pattern as shape evidence.  The GUI always writes
                // a filament's self-purge as 0, so whichever reading is right has
                // a zero at every diagonal cell it claims.  Bounded to indices the
                // saved LENGTH actually has, so no reading can step past the
                // vector whatever shape it is probed with.
                auto diagonals_all_zero = [&](size_t side, size_t blocks) {
                    for (size_t block = 0; block < blocks; ++block)
                        for (size_t d = 0; d < side; ++d) {
                            const size_t idx = block * side * side + d * side + d;
                            if (idx >= old_size || flush_matrix->values[idx] != 0.)
                                return false;
                        }
                    return true;
                };

                if (whole_is_square && square_root > 1 && square_root >= filament_count) {
                    // One saved square block that covers the active roster — the
                    // reading that keeps every pair the roster can name, and the
                    // one the GUI writes.  Four heads with sixteen saved values and
                    // a roster of three fit a single 4x4 table (pairs for filaments
                    // 0..2 preserved) as easily as four 2x2 head blocks (which keep
                    // only the first two filaments), so the diagonal zeros decide:
                    // four flattened 2x2 blocks put cross-purges at indices 5 and
                    // 10, failing the whole-square diagonal while every per-head
                    // diagonal is zero; a single saved 4x4 passes the whole-square
                    // test and fails the per-head one, whose diagonals land on its
                    // cross-purges (3, 4, 7…).  Per-head takes precedence only on
                    // that one-sided evidence; if both shapes satisfy the test
                    // there is no definitive encoding, so the conservative
                    // whole-square choice stands.  A matrix whose size already
                    // matches the target is never reshaped at all, which is what
                    // tells four right-sized 2x2 head blocks apart.
                    const bool per_head_shape =
                        per_head_blocks && diagonals_all_zero(per_head_root, nozzle_count);
                    if (per_head_shape && !diagonals_all_zero(square_root, 1)) {
                        old_filament_count = per_head_root;
                        old_nozzle_count   = nozzle_count;
                    } else {
                        old_filament_count = square_root;
                    }
                } else if (per_head_blocks) {
                    // Whole N*N blocks per head: exactly the engine's layout, and
                    // the only reading left when the saved square cannot cover the
                    // roster (a block smaller than the roster keeps the pairs it
                    // has per head instead of discarding the head axis).
                    old_filament_count = per_head_root;
                    old_nozzle_count   = nozzle_count;
                } else if (whole_is_square) {
                    // A single-head N*N matrix, whatever else the file claims.
                    // Four values on a four-head machine are one saved 2x2 block
                    // holding all four purge pairs, not four 1x1 blocks; that
                    // reading would keep only the diagonal — the self-pair the GUI
                    // always writes as 0 — and drop every saved cross-filament
                    // value.
                    old_filament_count = square_root;
                } else {
                    // Largest whole block that still divides the length.
                    size_t candidate = 1;
                    while ((candidate + 1) * (candidate + 1) <= old_size)
                        ++candidate;
                    for (; candidate > 1; --candidate)
                        if (old_size % (candidate * candidate) == 0)
                            break;
                    old_filament_count = candidate;
                    old_nozzle_count   = old_size / (candidate * candidate);
                }
            }

            const size_t old_sub = old_filament_count * old_filament_count;
            const size_t new_sub = filament_count * filament_count;
            std::vector<double> rebuilt(matrix_target, 0.);
            for (size_t i = 0; i < filament_count; ++i) {
                for (size_t j = 0; j < filament_count; ++j) {
                    for (size_t nozzle_id = 0; nozzle_id < nozzle_count; ++nozzle_id) {
                        double value;
                        if (nozzle_id < old_nozzle_count &&
                            i < old_filament_count && j < old_filament_count) {
                            value = flush_matrix->values[(i * old_filament_count + j) + old_sub * nozzle_id];
                        } else if (i == j) {
                            value = 0.;   // a filament never purges into itself
                        } else {
                            // Same pairing the GUI uses for a new filament slot:
                            // the unloading volume of i plus the loading volume of j.
                            value = flush_vector->values[2 * i] + flush_vector->values[2 * j + 1];
                        }
                        rebuilt[i * filament_count + j + new_sub * nozzle_id] = value;
                    }
                }
            }
            flush_matrix->values = std::move(rebuilt);
            ++extended;
        }
    }
}

// filament_count is the roster the caller read out of project_filament_count()
// before anything padded vector options; it is passed in rather than recomputed
// here so values appended by that padding can never be counted as filaments the
// project did not declare.
bool align_per_filament_config_vectors(Slic3r::DynamicPrintConfig& config,
                                       size_t filament_count, bool verbose) {
    if (filament_count <= 1)
        return false;   // one slot: the flat config already is the whole roster

    size_t extended = 0;
    for (const std::string& key : config.keys()) {
        if (!is_per_filament_config_key(key))
            continue;
        auto* vec = dynamic_cast<Slic3r::ConfigOptionVectorBase*>(config.option(key, false));
        if (!vec || vec->size() == 0 || vec->size() >= filament_count)
            continue;
        vec->resize(filament_count);   // duplicates the front value, like get_at()
        ++extended;
    }

    // The per-filament index maps are aligned by the same helper the front end
    // calls before the nozzle-map derivation, so one implementation decides their
    // cardinality in both places.  filament_map has just reached filament_count
    // through the loop above, which leaves the repair of an entry below 1 here;
    // filament_map_2 and filament_nozzle_map are still handled in full.
    extended += align_per_filament_maps(config, filament_count);

    // filament_self_index: only extend to a variant table that already covers the
    // roster.  The slot count can be smaller than the filament count (one slot
    // per extruder × nozzle volume), and growing it past the table would make
    // PrintObject.cpp:342-345 index the shorter variant vector out of range.
    // A one-slot-per-filament table has owners 1..N; extra variant slots have
    // no derivable owner, so preserve the existing get_at() front fallback.
    auto* variant_list = config.option<Slic3r::ConfigOptionStrings>("filament_extruder_variant", false);
    auto* self_index = config.option<Slic3r::ConfigOptionInts>("filament_self_index", false);
    if (variant_list && self_index) {
        const size_t slots = variant_list->values.size();
        if (slots >= filament_count && self_index->values.size() < slots) {
            const size_t known = self_index->values.size();
            self_index->values.resize(slots, 1);
            for (size_t i = known; i < slots; ++i)
                self_index->values[i] = slots == filament_count
                    ? int(i) + 1 : self_index->values.front();
            ++extended;
        }
    }

    align_filament_flush_arrays(config, filament_count, extended);

    if (verbose && extended)
        std::cout << "Filament alignment: " << extended
                  << " array(s) extended to filament_count=" << filament_count << "\n";
    return extended != 0;
}
#endif // ENGINE_BAMBU — BBS-only config-normalization helpers

// ── Filament-vector normalization: the desktop's own rule ────────────────────
//
// Preset::normalize() sizes every filament vector to the filament count,
// filling a slot it has to add with that option's definition default and
// skipping only the two name lists and the variant tables (Preset.cpp 444-486
// at OrcaSlicer v2.4.2, 442-487 at BambuStudio v02.08.02.61, citing the same
// keys). The desktop reaches it for every preset it loads
// (PresetBundle::load_config_model -> Preset::normalize), and
// PresetBundle::full_fff_config then assembles the slicing config out of those
// presets, so no filament vector the engine slices with is ever shorter than
// the roster it prints.
//
// This build has no preset bundle on the Orca path: the flat project config
// goes straight to Print::apply(). A project whose filament_printable is empty
// or shorter than its roster therefore reached
// Print::get_physical_unprintable_filaments, which reads
// m_config.filament_printable.values[filament_idx] with no bounds check
// (OrcaSlicer v2.4.2 Print.cpp:3229), and died with SIGSEGV inside
// Print::_make_wipe_tower (Orca build, 10-filament X2D tower project,
// 2026-10-03). The Bambu build already has the equivalent pass for its own
// flat-config path (align_per_filament_config_vectors above).
//
// Only a vector shorter than the roster is touched; one already at the roster
// length keeps every value it holds, so a project's own filament_printable is
// never overwritten.
size_t normalize_filament_config_vectors(Slic3r::DynamicPrintConfig& config) {
    const size_t filament_count = project_filament_count(config);
    if (filament_count == 0)
        return 0;
    const auto& defaults = Slic3r::FullPrintConfig::defaults();
    size_t grown = 0;
    for (const std::string& key : Slic3r::Preset::filament_options()) {
        // The two Preset::normalize skips as well: lists of preset names.
        if (key == "compatible_prints" || key == "compatible_printers")
            continue;
        // The per-(extruder x nozzle volume) variant tables, skipped there too:
        // their length is the slot count, not the filament count. This engine
        // has none (no filament_options_with_variant at bac6fd5).
#ifndef ENGINE_SNAPMAKER
        if (Slic3r::filament_options_with_variant.count(key) > 0)
            continue;
#endif
        const Slic3r::ConfigOption* fill = defaults.option(key);
        if (fill == nullptr)
            continue;
        auto* vec = dynamic_cast<Slic3r::ConfigOptionVectorBase*>(config.option(key, false));
        if (vec == nullptr || vec->size() >= filament_count)
            continue;
        vec->resize(filament_count, fill);
        ++grown;
    }
    return grown;
}

} // namespace slicer_cli
