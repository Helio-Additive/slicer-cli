// Generator of the vectors of tests/print_config_def_native_parity.rs: libslic3r's PrintConfig.hpp /
// PrintConfig.cpp at the pin (references/BambuStudio/src/libslic3r).
//
// Build, with REPO = this repository, BUILD = a built tree of the Bambu Studio engine
// (docs/building.md, -DENGINE=bambu) and DATA = this directory: compile with the compile line the
// build uses for a libslic3r source (its defines and include directories, the override layer
// $REPO/libslic3r/bambustudio before the pin, -std=c++17 -O3 -DNDEBUG and the forced include of
// cgal_54_compat.hpp), then link with the build's libslic3r_core and the libraries it links:
//
//   c++ <the build's defines and include directories> -std=c++17 -O3 -DNDEBUG \
//       -include $REPO/libslic3r/bambustudio/cgal_54_compat.hpp -c $DATA/gen.cpp -o gen.o
//   c++ gen.o $BUILD/libslic3r_core.a <the libraries libslic3r_core links> -o gen
//   ./gen <output directory>
//   cd <output directory> && for f in *.txt; do gzip -9 -n -c $f > $DATA/$f.gz; done
//
// It writes print_config_def.txt (the definitions), enums.txt (the static key maps) and
// functions.txt (handle_legacy, the free functions, DynamicPrintConfig and the defaults of the static
// configuration classes). The vectors in this directory were made on macOS arm64 with Apple clang and
// libc++. The run is deterministic.
//
// Line format: tokens separated by one space. A text token is the hexadecimal of its bytes ("-"
// for the empty text), a double its 16 hexadecimal digits, an integer is decimal, a list of texts
// is its length and the texts. Lines:
//
//   enum <name> <n> (<key> <value>)*n <m> <name>*m
//       ConfigOptionEnum<T>::get_enum_values() and get_enum_names() of a PrintConfig.cpp map.
//   def <key> <type> <nullable> <gui_type> <gui_flags> <label> <full_label> <printer_technology>
//       <category> <tooltip> <sidetext> <cli> <cli_params> <ratio_over> <multiline> <full_width>
//       <is_code> <readonly> <height> <width> <min> <max> <max_literal> <mode> <aliases>
//       <shortcut> <enum_values> <enum_labels> <keys_map> <ordinal rank> <default> <created>
//     one per key of print_config_def.options, in the order of the map. <keys_map> is "-" or the
//     number of pairs and the pairs; <ordinal rank> is the place of serialization_key_ordinal in
//     by_serialization_key_ordinal; <default> is "-" or <class> <type> <nullable> <is_nil> and the
//     serialization (for the class EnumsGeneric, whose keys_map the C++ leaves uninitialized, the
//     number of values and the values instead); <created> is create_default_option()->serialize()
//     ("-" when it is not asked: an enum without a keys map).
//   ordinals <n> <key>*n     by_serialization_key_ordinal in order
//   keys <which> <list>      extruder_option_keys / extruder_retract_keys / filament_option_keys /
//                            filament_retract_keys
//   legacy <key> <value> => <key> <value>     PrintConfigDef::handle_legacy
//   fn <name> <inputs...> => <outputs...>     a free function or a method of DynamicPrintConfig; a
//                            configuration after a call is the number of its options and the key and
//                            serialize() of each ("!" and the text of what() when it throws)
//   scenario <id> <n> (<key> <value>)*n       a configuration the following "fn ... <id>" lines use,
//                            built with set_deserialize_strict() (a key starting with '!' is not
//                            defined and is stored as ConfigOptionStrings)
//   process <id> ... / overrides <id> ...     the same, for the update_values_* and the
//                            align_filament_variant_overrides lines
//   static <class> <options>  T::defaults() of a static configuration class: key, class and value
//                            of each option (an enum vector without keys map: its values)

#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/libslic3r.h"

#include <cstdio>
#include <cstring>
#include <functional>
#include <memory>
#include <map>
#include <set>
#include <string>
#include <vector>

using namespace Slic3r;

typedef std::vector<std::string> Toks;

static FILE *g_out = nullptr;

static std::string hex(const std::string &s)
{
    if (s.empty()) return "-";
    static const char *d = "0123456789abcdef";
    std::string        o;
    for (unsigned char c : s) {
        o += d[c >> 4];
        o += d[c & 15];
    }
    return o;
}
static std::string num(long long v) { return std::to_string(v); }
static std::string dbl(double v)
{
    uint64_t b;
    memcpy(&b, &v, 8);
    char buf[32];
    snprintf(buf, sizeof(buf), "%016llx", (unsigned long long) b);
    return buf;
}
static void emit(const Toks &l)
{
    for (size_t i = 0; i < l.size(); ++i) {
        if (i) fputc(' ', g_out);
        fputs(l[i].c_str(), g_out);
    }
    fputc('\n', g_out);
}
static void push_list(Toks &l, const std::vector<std::string> &v)
{
    l.push_back(num(v.size()));
    for (const std::string &s : v) l.push_back(hex(s));
}
static void open_out(const std::string &dir, const char *name)
{
    if (g_out) fclose(g_out);
    g_out = fopen((dir + "/" + name).c_str(), "wb");
    if (!g_out) {
        perror(name);
        exit(1);
    }
}

static const char *class_name(const ConfigOption *o)
{
    if (dynamic_cast<const ConfigOptionEnumsGeneric *>(o)) return "EnumsGeneric";
    if (dynamic_cast<const ConfigOptionEnumsGenericNullable *>(o)) return "EnumsGenericNullable";
    if (dynamic_cast<const ConfigOptionEnumGeneric *>(o)) return "EnumGeneric";
    if (o->type() == coEnum) return "Enum";
    if (dynamic_cast<const ConfigOptionFloatOrPercent *>(o)) return "FloatOrPercent";
    if (dynamic_cast<const ConfigOptionPercent *>(o)) return "Percent";
    if (dynamic_cast<const ConfigOptionFloat *>(o)) return "Float";
    if (dynamic_cast<const ConfigOptionPercents *>(o)) return "Percents";
    if (dynamic_cast<const ConfigOptionPercentsNullable *>(o)) return "PercentsNullable";
    if (dynamic_cast<const ConfigOptionFloats *>(o)) return "Floats";
    if (dynamic_cast<const ConfigOptionFloatsNullable *>(o)) return "FloatsNullable";
    if (dynamic_cast<const ConfigOptionInt *>(o)) return "Int";
    if (dynamic_cast<const ConfigOptionInts *>(o)) return "Ints";
    if (dynamic_cast<const ConfigOptionIntsNullable *>(o)) return "IntsNullable";
    if (dynamic_cast<const ConfigOptionString *>(o)) return "String";
    if (dynamic_cast<const ConfigOptionStrings *>(o)) return "Strings";
    if (dynamic_cast<const ConfigOptionFloatsOrPercents *>(o)) return "FloatsOrPercents";
    if (dynamic_cast<const ConfigOptionFloatsOrPercentsNullable *>(o)) return "FloatsOrPercentsNullable";
    if (dynamic_cast<const ConfigOptionPoint *>(o)) return "Point";
    if (dynamic_cast<const ConfigOptionPoints *>(o)) return "Points";
    if (dynamic_cast<const ConfigOptionPoint3 *>(o)) return "Point3";
    if (dynamic_cast<const ConfigOptionPointsGroups *>(o)) return "PointsGroups";
    if (dynamic_cast<const ConfigOptionIntsGroups *>(o)) return "IntsGroups";
    if (dynamic_cast<const ConfigOptionBool *>(o)) return "Bool";
    if (dynamic_cast<const ConfigOptionBools *>(o)) return "Bools";
    if (dynamic_cast<const ConfigOptionBoolsNullable *>(o)) return "BoolsNullable";
    return "?";
}

// ---------------------------------------------------------------------------------------------
// enums.txt
// ---------------------------------------------------------------------------------------------

template<typename T> static void gen_enum(const char *name)
{
    Toks l{"enum", name};
    const t_config_enum_values &values = ConfigOptionEnum<T>::get_enum_values();
    l.push_back(num(values.size()));
    for (const auto &kvp : values) {
        l.push_back(hex(kvp.first));
        l.push_back(num(kvp.second));
    }
    push_list(l, ConfigOptionEnum<T>::get_enum_names());
    emit(l);
}

#define GEN_ENUM(T) gen_enum<T>(#T)

static void gen_enums()
{
    GEN_ENUM(PrinterTechnology);
    GEN_ENUM(PrintHostType);
    GEN_ENUM(AuthorizationType);
    GEN_ENUM(GCodeFlavor);
    GEN_ENUM(BedTempFormula);
    GEN_ENUM(FuzzySkinType);
    GEN_ENUM(NoiseType);
    GEN_ENUM(FuzzySkinMode);
    GEN_ENUM(InfillPattern);
    GEN_ENUM(IroningType);
    GEN_ENUM(ConformalStagger);
    GEN_ENUM(ConformalPole);
    GEN_ENUM(TopOneWallType);
    GEN_ENUM(WallInfillOrder);
    GEN_ENUM(WallSequence);
    GEN_ENUM(EnsureVerticalThicknessLevel);
    GEN_ENUM(PrintSequence);
    GEN_ENUM(SlicingMode);
    GEN_ENUM(SupportMaterialPattern);
    GEN_ENUM(SupportMaterialStyle);
    GEN_ENUM(SupportMaterialInterfacePattern);
    GEN_ENUM(SupportType);
    GEN_ENUM(SeamPosition);
    GEN_ENUM(SeamScarfType);
    GEN_ENUM(SLADisplayOrientation);
    GEN_ENUM(SLAPillarConnectionMode);
    GEN_ENUM(SLAMaterialSpeed);
    GEN_ENUM(BrimType);
    GEN_ENUM(TimelapseType);
    GEN_ENUM(SideScrapeFace);
    GEN_ENUM(DraftShield);
    GEN_ENUM(ForwardCompatibilitySubstitutionRule);
    GEN_ENUM(OverhangFanThreshold);
    GEN_ENUM(OverhangThresholdParticipatingCooling);
    GEN_ENUM(CoolingSlowdownLogicType);
    GEN_ENUM(BedType);
    GEN_ENUM(LayerSeq);
    GEN_ENUM(NozzleType);
    GEN_ENUM(FanDirection);
    GEN_ENUM(PrinterStructure);
    GEN_ENUM(PerimeterGeneratorType);
    GEN_ENUM(ZHopType);
    GEN_ENUM(ExtruderType);
    GEN_ENUM(NozzleVolumeType);
    GEN_ENUM(FilamentMapMode);
    GEN_ENUM(PrimeVolumeMode);
    GEN_ENUM(ReduceInfillRetractionMode);
    GEN_ENUM(FilamentMetalStickiness);
    GEN_ENUM(CounterboreHoleBridgingOption);
}

// ---------------------------------------------------------------------------------------------
// print_config_def.txt
// ---------------------------------------------------------------------------------------------

static void gen_defs()
{
    const PrintConfigDef &d = print_config_def;
    std::map<size_t, size_t> rank;
    {
        Toks l{"ordinals", num(d.by_serialization_key_ordinal.size())};
        size_t i = 0;
        for (const auto &kvp : d.by_serialization_key_ordinal) {
            rank[kvp.first] = i++;
            l.push_back(hex(kvp.second->opt_key));
        }
        emit(l);
    }
    for (const auto &kvp : d.options) {
        const ConfigOptionDef &def = kvp.second;
        Toks l{"def", hex(kvp.first), num(def.type), num(def.nullable), num((int) def.gui_type), hex(def.gui_flags),
               hex(def.label), hex(def.full_label), num(def.printer_technology), hex(def.category), hex(def.tooltip),
               hex(def.sidetext), hex(def.cli), hex(def.cli_params), hex(def.ratio_over), num(def.multiline),
               num(def.full_width), num(def.is_code), num(def.readonly), num(def.height), num(def.width), dbl(def.min),
               dbl(def.max), dbl(def.max_literal), num(def.mode)};
        push_list(l, def.aliases);
        push_list(l, def.shortcut);
        push_list(l, def.enum_values);
        push_list(l, def.enum_labels);
        if (def.enum_keys_map == nullptr)
            l.push_back("-");
        else {
            l.push_back(num(def.enum_keys_map->size()));
            for (const auto &kv : *def.enum_keys_map) {
                l.push_back(hex(kv.first));
                l.push_back(num(kv.second));
            }
        }
        l.push_back(num(rank.at(def.serialization_key_ordinal)));
        const ConfigOption *dv = def.default_value.get();
        if (dv == nullptr)
            l.push_back("-");
        else {
            const char *cls = class_name(dv);
            l.push_back(cls);
            l.push_back(num(dv->type()));
            l.push_back(num(dv->nullable()));
            l.push_back(num(dv->is_nil()));
            if (strcmp(cls, "EnumsGeneric") == 0 || strcmp(cls, "EnumsGenericNullable") == 0) {
                const std::vector<int> &v = static_cast<const ConfigOptionInts *>(dv)->values;
                l.push_back(num(v.size()));
                for (int x : v) l.push_back(num(x));
            } else
                l.push_back(hex(dv->serialize()));
        }
        if ((def.type == coEnum || def.type == coEnums) && def.enum_keys_map == nullptr)
            l.push_back("-");
        else {
            std::unique_ptr<ConfigOption> created(def.create_default_option());
            l.push_back(class_name(created.get()));
            l.push_back(hex(created->serialize()));
        }
        emit(l);
    }
    {
        Toks l{"keys", "extruder_option_keys"};
        push_list(l, d.extruder_option_keys());
        emit(l);
    }
    {
        Toks l{"keys", "extruder_retract_keys"};
        push_list(l, d.extruder_retract_keys());
        emit(l);
    }
    {
        Toks l{"keys", "filament_option_keys"};
        push_list(l, d.filament_option_keys());
        emit(l);
    }
    {
        Toks l{"keys", "filament_retract_keys"};
        push_list(l, d.filament_retract_keys());
        emit(l);
    }
}

// ---------------------------------------------------------------------------------------------
// functions.txt
// ---------------------------------------------------------------------------------------------

static void legacy(const std::string &key, const std::string &value)
{
    std::string k = key, v = value;
    PrintConfigDef::handle_legacy(k, v);
    emit({"legacy", hex(key), hex(value), "=>", hex(k), hex(v)});
}

static void gen_legacy()
{
    // Every branch of PrintConfigDef::handle_legacy (PrintConfig.cpp:8306-8494), in its order.
    for (const char *k : {"enable_wipe_tower", "wipe_tower_width", "bottom_solid_infill_flow_ratio", "wiping_volume", "prime_volume",
                          "wipe_tower_brim_width", "tool_change_gcode", "bridge_fan_speed", "infill_extruder", "solid_infill_extruder",
                          "perimeter_extruder", "support_material_extruder", "support_material_interface_extruder",
                          "support_material_angle", "support_material_enforce_layers"})
        legacy(k, "1");
    for (const char *k : {"initial_layer_print_height", "initial_layer_speed", "internal_solid_infill_speed", "top_surface_speed",
                          "support_interface_speed", "outer_wall_speed", "support_object_xy_distance"}) {
        legacy(k, "50%");
        legacy(k, "0.2");
        legacy(k, "%");
    }
    legacy("ams_filament_load_time", "1,2,3");
    legacy("ams_filament_unload_time", "1,2,3");
    for (const char *k : {"inherits_cummulative", "compatible_printers_condition_cummulative",
                          "compatible_prints_condition_cummulative", "cooling", "timelapse_no_toolhead"})
        legacy(k, "x");
    for (const char *v : {"2", "0", "1", "20"}) legacy("timelapse_type", v);
    for (const char *v : {"normal", "tree", "hybrid(auto)", "normal(auto)", "tree(manual)", "Normal"}) legacy("support_type", v);
    for (const char *v : {"none", "rectilinear", "hollow"}) legacy("support_base_pattern", v);
    legacy("infill_anchor", "400%");
    legacy("infill_anchor_max", "20");
    for (const char *v : {"", "\"infill_extruder;wall_loops;bridge_fan_speed\"", "a;b;cooling;tool_change_gcode",
                          "sparse_infill_density;;support_material_angle", "infill_extruder;infill_extruder;\"cooling\"",
                          "wall_loops;layer_height", "\"\"", ";", "acceleration;scale;wall_loops", "not_a_key", "cooling",
                          "infill_anchor;infill_anchor_max", "wall_infill_order;wall_sequence", "enable_wipe_tower;wipe_tower_width",
                          "support_material_extruder;support_material_interface_extruder", "prime_volume;wiping_volume",
                          "\"timelapse_no_toolhead\";\"support_type\""})
        legacy("different_settings_to_system", v);
    for (const char *v : {"5%", "10%", "5"}) legacy("overhang_fan_threshold", v);
    for (const char *v : {"inner wall/outer wall/infill", "infill/inner wall/outer wall", "outer wall/inner wall/infill",
                          "infill/outer wall/inner wall", "inner-outer-inner wall/infill", "inner wall/outer wall", ""})
        legacy("wall_infill_order", v);
    for (const char *k : {"nozzle_volume_type", "default_nozzle_volume_type", "printer_extruder_variant", "print_extruder_variant",
                          "filament_extruder_variant", "extruder_variant_list"})
        for (const char *v : {"Normal", "Big Traffic", "Direct Drive Normal,Bowden Big Traffic", "NormalNormal", "Standard", ""})
            legacy(k, v);
    for (const char *v : {"DirectDrive", "DirectDrive,Bowden", "Direct Drive", "DirectDriveDirectDrive"}) legacy("extruder_type", v);
    for (const char *v : {"1", "0", "enabled", "2"}) legacy("ensure_vertical_shell_thickness", v);
    for (const char *v : {"Auto", "Manual", "auto"}) legacy("filament_map_mode", v);
    for (const char *v : {"PLA", "ASA-Aero", "\"ASA-Aero\";\"PLA\"", "PLA;ASA-Aero;", "\"\"", ";ASA-Aero", "\"ASA-Aero", "\"",
                          "ASA-Aero;;PLA", "\"PLA\";\"ASA-AERO\"", "", ";", "ASA-Aero;", "a\"ASA-Aero\""})
        legacy("filament_type", v);
    // The obsolete keys (PrintConfig.cpp:8458-8483).
    for (const char *k : {"acceleration", "scale", "rotate", "duplicate", "duplicate_grid", "bed_size", "reduce_infill_retraction",
                          "print_center", "g0", "wipe_tower_per_color_wipe", "max_volumetric_extrusion_rate_slope_positive",
                          "max_volumetric_extrusion_rate_slope_negative", "support_sharp_tails", "support_remove_small_overhangs",
                          "support_with_sheath", "tree_support_collision_resolution", "tree_support_with_infill",
                          "tree_support_brim_width", "max_volumetric_speed", "max_print_speed", "support_closing_radius",
                          "remove_freq_sweep", "remove_bed_leveling", "remove_extrusion_calibration", "support_transition_line_width",
                          "support_transition_speed", "bed_temperature", "bed_temperature_initial_layer", "can_switch_nozzle_type",
                          "can_add_auxiliary_fan", "extra_flush_volume", "spaghetti_detector", "adaptive_layer_height",
                          "prime_tower_feedrate_factor_nozzle_0_2", "prime_tower_feedrate_factor_nozzle_0_4",
                          "prime_tower_feedrate_factor_nozzle_0_6", "prime_tower_feedrate_factor_nozzle_0_8",
                          "prime_tower_line_width_nozzle_0_2", "prime_tower_line_width_nozzle_0_4",
                          "prime_tower_line_width_nozzle_0_6", "prime_tower_line_width_nozzle_0_8",
                          "prime_tower_ramming_line_width_nozzle_0_2", "prime_tower_ramming_line_width_nozzle_0_4",
                          "prime_tower_ramming_line_width_nozzle_0_6", "prime_tower_ramming_line_width_nozzle_0_8", "z_hop_type",
                          "nozzle_hrc", "chamber_temperature", "only_one_wall_top", "bed_temperature_difference",
                          "long_retraction_when_cut", "retraction_distance_when_cut", "apply_top_surface_compensation", "nozzle_type"})
        legacy(k, "1");
    // Keys that are defined, and keys that are not.
    for (const char *k : {"layer_height", "wall_loops", "filament_prime_volume", "", "not_a_key", "Layer_height", "layer_height ",
                          "filament_retraction_length", "machine_max_speed_x", "machine_max_force_Y", "display_width"})
        legacy(k, "0.2");
    // Every key of the definition, with a value that touches no rule.
    for (const auto &kvp : print_config_def.options) legacy(kvp.first, "v");
}

// The test configurations: key / value pairs given to set_deserialize_strict(); a key starting with
// '!' is not defined by print_config_def and is stored with set_key_value() as ConfigOptionStrings.
typedef std::vector<std::pair<std::string, std::string>> Scenario;
static const std::vector<Scenario> &scenarios()
{
    static const std::vector<Scenario> s = {
        {},
        {{"printer_technology", "FFF"}, {"nozzle_diameter", "0.4"}, {"extruder_type", "Direct Drive"},
         {"nozzle_volume_type", "Standard"}, {"extruder_variant_list", "\"Direct Drive Standard\""},
         {"printer_extruder_variant", "\"Direct Drive Standard\""}, {"printer_extruder_id", "1"},
         {"print_extruder_variant", "\"Direct Drive Standard\""}, {"print_extruder_id", "1"},
         {"filament_extruder_variant", "\"Direct Drive Standard\""}, {"filament_self_index", "1"},
         {"filament_max_volumetric_speed", "12"}, {"filament_flow_ratio", "0.98"}, {"filament_type", "\"PLA\""},
         {"filament_vendor", "\"Bambu\""}, {"filament_is_support", "0"}, {"extruder_clearance_max_radius", "68"},
         {"print_sequence", "by object"}, {"enable_prime_tower", "1"}, {"independent_support_layer_height", "1"},
         {"timelapse_type", "0"}, {"extruder", "2"}, {"spiral_mode", "1"}, {"resolution", "0.0001"}, {"auxiliary_fan", "1"},
         {"ams_filament_load_time_ams", "20"}, {"ams_filament_unload_time_n3f_s", "30"}, {"extruder_ams_count", "\"1#0|4#1\""},
         {"ams_nozzle_idx", "0"}},
        {{"printer_technology", "SLA"}, {"nozzle_diameter", "0.4,0.4"}, {"extruder_type", "Direct Drive,Direct Drive"},
         {"nozzle_volume_type", "Standard,High Flow"}, {"default_nozzle_volume_type", "High Flow,Standard"},
         {"extruder_max_nozzle_count", "1,1"},
         {"extruder_variant_list", "\"Direct Drive Standard,Direct Drive High Flow\";\"Direct Drive Standard,Direct Drive High Flow,Direct Drive TPU High Flow\""},
         {"printer_extruder_variant", "\"Direct Drive Standard\";\"Direct Drive High Flow\";\"Direct Drive Standard\";\"Direct Drive High Flow\""},
         {"printer_extruder_id", "1,1,2,2"},
         {"print_extruder_variant", "\"Direct Drive Standard\";\"Direct Drive High Flow\";\"Direct Drive Standard\";\"Direct Drive High Flow\""},
         {"print_extruder_id", "1,1,2,2"},
         {"filament_extruder_variant", "\"Direct Drive Standard\";\"Direct Drive High Flow\";\"Bowden High Flow\";\"Direct Drive Standard\";\"Direct Drive High Flow\""},
         {"filament_self_index", "1,1,1,2,2"}, {"filament_max_volumetric_speed", "nil,-1,15,0,20"},
         {"filament_flow_ratio", "0.98,0,0.95,1,1.02"}, {"filament_type", "\"PLA\";\"PA\";\"ABS\""},
         {"filament_is_support", "1,1,1"}, {"!filament_id", "\"GFS00\";\"GFL99\";\"GFS01\""}, {"filament_vendor", "\"\""},
         {"print_sequence", "by layer"}, {"enable_prime_tower", "1"}, {"independent_support_layer_height", "1"},
         {"timelapse_type", "1"}, {"extruder_ams_count", "\"1#0|4#0\";\"1#0\""}, {"ams_nozzle_idx", "1"},
         {"extruder_nozzle_stats", "\"Standard#1|High Flow#1\";\"TPU285#1|E3D High Flow#2|Bogus#3\""}, {"auxiliary_fan", "1"},
         {"aux_fan_independent_control", "1"}, {"extruder_clearance_max_radius", "3"}},
        {{"multi_nozzle_diameter_mode", "1"}, {"handle_hotend_as_extruder", "1"}, {"nozzle_diameter", "0.4"},
         {"extruder_max_nozzle_count", "6"}, {"extruder_nozzle_diameter", "0.2,0.4,0.4,0.6,0.8,0.4"}, {"extruder_nozzle_count", "6"},
         {"printer_variant", "0.4"}, {"printer_nozzle_diameter_mode", "multi"},
         {"default_nozzle_volume_type", "Standard,High Flow,Standard,Standard,TPU High Flow,Standard"},
         {"filament_diameter_map", "\"0.4\";\"0.6\";\"\";\"0.25\";\"1.0\""}, {"filament_is_mixed", "0,0,1,0,0"},
         {"filament_mixed_components", "\"\";\"\";\"1,2\";\"\";\"\""},
         {"filament_colour", "\"#FF0000\";\"#00FF00\";\"#0000FF\";\"#FFFFFF\";\"#000000\";\"#123456\""},
         {"sync_auxiliary_fan_to_second_fan", "1"}, {"auxiliary_fan", "1"}, {"extruder_ams_count", "\"\""}, {"ams_nozzle_idx", "-1"},
         {"extruder_nozzle_stats", "\"Standard#4|High Flow#2\""}, {"nozzle_volume_type", "Standard"}},
        {{"multi_nozzle_diameter_mode", "1"}, {"nozzle_diameter", "0.4,0.6"}, {"extruder_nozzle_diameter", "0.4,0.6"},
         {"extruder_nozzle_count", "1,1"}, {"handle_hotend_as_extruder", "0"},
         {"filament_diameter_map", "\"0.4\";\"0.4\";\"x\";\"0.6\";\"0.6;0.4\""}, {"filament_is_mixed", "0,0,1,1,0,1"},
         {"filament_mixed_components", "\"\";\"\";\"1,2\";\"1,4\";\"\";\"2,5\""},
         {"default_nozzle_volume_type", "Standard,High Flow,TPU High Flow"}, {"extruder_max_nozzle_count", "2,nil"},
         {"extruder_type", "Direct Drive,Bowden"}, {"nozzle_volume_type", "High Flow"}, {"relative_correction", "1,0.99"},
         {"material_correction", "1.01,1.02"}, {"printer_variant", "abc"}},
        {{"printer_technology", "SLA"}, {"relative_correction", "0.9,0.8"}, {"relative_correction_y", "0.5"},
         {"multi_nozzle_diameter_mode", "1"}, {"nozzle_diameter", "0.6,0.6,0.4"}, {"default_nozzle_volume_type", "Hybrid"},
         {"extruder_max_nozzle_count", "nil,3,0"}, {"extruder_ams_count", "\"2#3\""}, {"ams_nozzle_idx", "2"},
         {"extruder_nozzle_stats", "\"\";\"High Flow#0|Standard#2\""}},
    };
    return s;
}

static DynamicPrintConfig make_config(const Scenario &sc)
{
    DynamicPrintConfig cfg;
    for (const auto &kv : sc) {
        if (kv.first[0] == '!') {
            ConfigOptionStrings *o = new ConfigOptionStrings();
            o->deserialize(kv.second);
            cfg.set_key_value(kv.first.substr(1), o);
        } else
            cfg.set_deserialize_strict(kv.first, kv.second);
    }
    return cfg;
}

// The options of a configuration: their number, then key and serialization of each.
static void push_config(Toks &l, const ConfigBase &cfg)
{
    t_config_option_keys keys = cfg.keys();
    l.push_back(num(keys.size()));
    for (const std::string &k : keys) {
        l.push_back(hex(k));
        try {
            l.push_back(hex(cfg.option(k)->serialize()));
        } catch (const std::exception &e) {
            l.push_back("!" + hex(e.what()));
        }
    }
}

static std::string nvts(const std::vector<NozzleVolumeType> &v)
{
    std::string s = num(v.size());
    for (NozzleVolumeType t : v) s += "," + num(t);
    return s;
}

static void gen_functions()
{
    // Pure functions.
    {
        const std::vector<std::vector<int>>    counts = {{}, {1}, {2, 0}, {1, 1}, {-1, 3}, {6}};
        const std::vector<std::vector<double>> diams  = {{}, {0.4}, {0.4, 0.6}, {0.2, 0.4, 0.6, 0.8, 0.4, 0.4}, {0.4, 0.4, 0.4}};
        for (int h = 0; h < 2; ++h)
            for (size_t c = 0; c < counts.size(); ++c)
                for (size_t d = 0; d < diams.size(); ++d)
                    emit({"fn", "has_authoritative_per_slot_nozzle_diameters", num(h), num(c), num(d), "=>",
                          num(has_authoritative_per_slot_nozzle_diameters(h != 0, counts[c], diams[d]))});
    }
    for (double d : {0.2, 0.4, 0.6, 0.8, 0.2005, 0.199, 0.401, 0.3, 1.0, 0.0, 0.4009, 0.39901, 0.8010, -0.2}) {
        emit({"fn", "multi_nozzle_double", dbl(d), "=>", dbl(multi_nozzle_retraction_length(d, 9.5)), dbl(multi_nozzle_ironing_inset(d, 9.5)),
              dbl(multi_nozzle_top_shell_thickness(d, 9.5)), dbl(multi_nozzle_bottom_shell_thickness(d, 9.5)),
              dbl(multi_nozzle_support_z_distance(d, 9.5)), dbl(multi_nozzle_prime_tower_line_width(d, 9.5)),
              dbl(multi_nozzle_prime_tower_ramming_line_width(d, 9.5)), dbl(multi_nozzle_prime_tower_feedrate_factor(d, 9.5))});
        emit({"fn", "multi_nozzle_int", dbl(d), "=>", num(multi_nozzle_wall_loops(d, 99)), num(multi_nozzle_top_shell_layers(d, 99)),
              num(multi_nozzle_bottom_shell_layers(d, 99)), num(multi_nozzle_top_color_penetration_layers(d, 99)),
              num(multi_nozzle_bottom_color_penetration_layers(d, 99))});
    }
    for (const char *k : {"retraction_length", "min_layer_height", "max_layer_height", "ironing_inset", "wall_loops", "top_shell_layers",
                          "bottom_shell_layers", "top_shell_thickness", "bottom_shell_thickness", "top_color_penetration_layers",
                          "bottom_color_penetration_layers", "support_top_z_distance", "support_bottom_z_distance",
                          "prime_tower_feedrate_factor", "prime_tower_line_width", "prime_tower_ramming_line_width", "wall_loops#2",
                          "retraction_length#", "#wall_loops", "layer_height", "", "#", "wall_loops#1#2", "Wall_loops"})
        emit({"fn", "is_multi_nozzle_auto_option", hex(k), "=>", num(is_multi_nozzle_auto_option(k))});
    for (int t = 0; t < 4; ++t) {
        emit({"fn", "is_tree", num(t), "=>", num(is_tree(SupportType(t))), num(is_auto(SupportType(t)))});
        for (int st = 0; st < 7; ++st)
            emit({"fn", "is_tree_slim", num(t), num(st), "=>", num(is_tree_slim(SupportType(t), SupportMaterialStyle(st)))});
    }
    for (int m = 0; m < 6; ++m) emit({"fn", "is_auto_filament_map_mode", num(m), "=>", num(is_auto_filament_map_mode(FilamentMapMode(m)))});
    for (int v = 0; v <= int(nvtCount); ++v) {
        Toks l{"fn", "nozzle_volume_type", num(v), "=>", num(legacy_fallback_nozzle_volume_type(NozzleVolumeType(v))),
               hex(get_nozzle_volume_type_string(NozzleVolumeType(v)))};
        for (int e = 0; e < 2; ++e) l.push_back(hex(get_extruder_variant_string(ExtruderType(e), NozzleVolumeType(v))));
        emit(l);
    }
    {
        Toks l{"fn", "get_valid_nozzle_volume_type", "=>"};
        for (NozzleVolumeType t : get_valid_nozzle_volume_type()) l.push_back(num(t));
        emit(l);
    }
    for (int b = 0; b <= int(btCount); ++b)
        emit({"fn", "bed_type", num(b), "=>", hex(bed_type_to_gcode_string(BedType(b))), hex(get_bed_temp_key(BedType(b))),
              hex(get_bed_temp_1st_layer_key(BedType(b)))});
    {
        const std::vector<std::string> variants = {"Direct Drive Standard", "Direct Drive High Flow", "Direct Drive Standard",
                                                   "Bowden Standard", "Direct Drive High Flow"};
        const std::vector<int>         ids      = {1, 1, 2, 2, 3};
        for (int v = 0; v < int(nvtCount); ++v)
            for (int e = 0; e < 2; ++e)
                for (int id = 0; id <= 4; ++id)
                    emit({"fn", "get_config_index_base", num(v), num(e), num(id), "=>",
                          num(get_config_index_base(NozzleVolumeType(v), ExtruderType(e), id, variants, ids))});
    }
    for (int a = -1; a <= 4; ++a)
        emit({"fn", "ams", num(a), "=>", hex(get_ams_type_name(a)), hex(get_ams_type_display_name(a)), hex(get_ams_load_time_key(a)),
              hex(get_ams_unload_time_key(a))});
    {
        Toks l{"fn", "get_ams_time_types", "=>"};
        for (int t : get_ams_time_types()) l.push_back(num(t));
        emit(l);
    }
    for (const std::vector<std::string> &names : std::vector<std::vector<std::string>>{
             {}, {"AMS"}, {"N3F_S", "AMS", "AMS"}, {"AMS_LITE", "ams", "X", "N3F_S", "AMS_LITE"}, {""}}) {
        Toks l{"fn", "get_supported_ams_time_types"};
        push_list(l, names);
        l.push_back("=>");
        for (int t : get_supported_ams_time_types(names)) l.push_back(num(t));
        emit(l);
    }
    for (const std::vector<std::string> &strs : std::vector<std::vector<std::string>>{
             {}, {""}, {"1#0|4#1"}, {"4#2|1#3", "", "2#5|2#7"}, {" 3#+4", "-1#-2|0#0"}}) {
        Toks l{"fn", "get_extruder_ams_count"};
        push_list(l, strs);
        l.push_back("=>");
        for (const auto &m : get_extruder_ams_count(strs)) {
            std::string e = num(m.size());
            for (const auto &kv : m) e += "," + num(kv.first) + ":" + num(kv.second);
            l.push_back(e);
        }
        push_list(l, save_extruder_ams_count_to_string(get_extruder_ams_count(strs)));
        emit(l);
    }
    for (const std::vector<std::string> &strs : std::vector<std::vector<std::string>>{
             {}, {""}, {"Standard#1|High Flow#2"}, {"TPU285#1|Bogus#2|Standard#3", "E3D High Flow#x"},
             {"Hybrid#1|malformed|#5|Standard#-1", "High Flow# 7|High Flow#2"}, {"Standard"}}) {
        Toks l{"fn", "get_extruder_nozzle_stats"};
        push_list(l, strs);
        l.push_back("=>");
        auto stats = get_extruder_nozzle_stats(strs);
        for (const auto &m : stats) {
            std::string e = num(m.size());
            for (const auto &kv : m) e += "," + num(kv.first) + ":" + num(kv.second);
            l.push_back(e);
        }
        push_list(l, save_extruder_nozzle_stats_to_string(stats));
        emit(l);
    }
    for (const char *n : {"Standard", "High Flow", "TPU High Flow", "TPU285", "Hybrid", "E3D High Flow", "standard", "", "Normal"})
        emit({"fn", "nozzle_volume_type_from_string", hex(n), "=>", num(nozzle_volume_type_from_string(n))});
    const std::vector<std::string> variants = {"Direct Drive Standard", "Bowden High Flow", " Direct Drive TPU High Flow\t",
                                               "Direct DriveTPU285", "Direct Drive Hybrid", "Bowden E3D High Flow", "Standard", "",
                                               "Direct Drive", "Bowden Direct Drive Standard", "Direct Drive Standard Bowden",
                                               "High Flow Direct Drive", "Direct Drive  Standard ", "Bowden\nStandard"};
    for (const std::string &v : variants) emit({"fn", "convert_to_nvt_type", hex(v), "=>", num(convert_to_nvt_type(v))});
    {
        const std::vector<std::vector<std::string>> lists = {{}, {"Direct Drive Standard"}, {"Direct Drive TPU285"},
                                                             {"Bowden High Flow", "Direct Drive TPU285"}, {"Direct Drive E3D High Flow", "x"}};
        for (int v = 0; v < int(nvtCount); ++v)
            for (size_t i = 0; i < lists.size(); ++i)
                for (int c = 0; c < 2; ++c)
                    emit({"fn", "is_nozzle_printable_for_filament", num(v), num(i), num(c), "=>",
                          num(is_nozzle_printable_for_filament(NozzleVolumeType(v), lists[i], c != 0))});
    }

    // Functions of a configuration.
    const std::vector<Scenario> &scs = scenarios();
    for (size_t si = 0; si < scs.size(); ++si) {
        {
            Toks l{"scenario", num(si), num(scs[si].size())};
            for (const auto &kv : scs[si]) {
                l.push_back(hex(kv.first));
                l.push_back(hex(kv.second));
            }
            emit(l);
        }
        const DynamicPrintConfig cfg = make_config(scs[si]);
        const std::string        S   = num(si);
        emit({"fn", "config", S, "=>", num(multi_nozzle_diameter_mode_enabled(cfg)), hex(printer_nozzle_diameter_mode_tag(cfg)),
              num(has_multiple_configured_nozzle_diameters(cfg)), dbl(layer_grid_nozzle_diameter(cfg)), num(layer_grid_subdivision(cfg)),
              dbl(primary_nozzle_diameter(cfg)), num(get_effective_ams_nozzle_idx(cfg)), num(is_single_ext_multi_nozzle(cfg)),
              dbl(min_object_distance(cfg))});
        {
            Toks l{"fn", "ams_times", S, "=>"};
            for (double t : get_ams_load_times(cfg)) l.push_back(dbl(t));
            l.push_back("|");
            for (double t : get_ams_unload_times(cfg)) l.push_back(dbl(t));
            emit(l);
        }
        for (int e = -1; e <= 3; ++e) {
            Toks l{"fn", "default_nozzle_volume", S, num(e), "=>", num(get_extruder_default_nozzle_volume_type(cfg, e))};
            for (int n = -1; n <= 6; ++n) l.push_back(num(get_default_nozzle_volume_type(cfg, e, n)));
            std::string sup;
            for (NozzleVolumeType t : get_extruder_supported_nozzle_volume_types(cfg, e)) sup += num(t) + ",";
            l.push_back(sup.empty() ? "-" : sup);
            emit(l);
        }
        for (const char *v : {"Direct Drive Standard", "Direct Drive High Flow", "", "Bowden Standard"})
            emit({"fn", "find_printer_variant_index", S, hex(v), "=>", num(find_printer_variant_index(cfg, v))});
        {
            Toks l{"fn", "collect_aux_fan_slots", S, "=>"};
            for (const AuxFanSlot &a : collect_aux_fan_slots(cfg)) {
                l.push_back(num(a.optional));
                l.push_back(num(a.gcode_index));
                l.push_back(num(a.independent));
            }
            emit(l);
        }
        DynamicPrintConfig c = cfg;
        for (const char *k : {"retraction_length", "machine_max_speed_x", "filament_flow_ratio", "outer_wall_speed", "nozzle_diameter", "x"})
            emit({"fn", "get_parameter_size", S, hex(k), "=>", num(c.get_parameter_size(k, 3))});
        for (int id = 0; id < 4; ++id) {
            std::string displayed = "?";
            std::string t         = c.get_filament_type(displayed, id);
            emit({"fn", "get_filament_type", S, num(id), "=>", hex(t), hex(displayed)});
        }
        {
            int  ec = -7;
            bool sd = c.support_different_extruders(ec);
            emit({"fn", "extruders", S, "=>", num(c.is_using_different_extruders()), num(sd), num(ec), hex(cfg.get_filament_vendor()),
                  hex(cfg.get_filament_type())});
        }
        for (int ec = 1; ec <= 2; ++ec) {
            std::vector<std::vector<NozzleVolumeType>> types(1, std::vector<NozzleVolumeType>{nvtHybrid});
            int                                        n = cfg.get_extruder_nozzle_volume_count(ec, types);
            Toks                                       l{"fn", "get_extruder_nozzle_volume_count", S, num(ec), "=>", num(n)};
            for (const auto &t : types) l.push_back(nvts(t));
            emit(l);
        }
        for (const char *vn : {"printer_extruder_variant", "print_extruder_variant", "filament_extruder_variant"})
            for (const char *idn : {"", "printer_extruder_id", "filament_self_index"})
                for (int id = 0; id <= 2; ++id)
                    for (int e = 0; e < 2; ++e)
                        for (int v = 0; v < int(nvtCount); ++v)
                            emit({"fn", "get_index_for_extruder", S, hex(vn), hex(idn), num(id), num(e), num(v), "=>",
                                  num(cfg.get_index_for_extruder(id, idn, ExtruderType(e), NozzleVolumeType(v), vn, 2))});

        // Functions that change the configuration: the configuration after each, from a copy.
        auto after = [&](const char *name, const std::function<std::string(DynamicPrintConfig &)> &f) {
            DynamicPrintConfig c = cfg;
            Toks               l{"fn", name, S, "=>"};
            try {
                l.push_back(f(c));
            } catch (const std::exception &e) {
                l.push_back("!" + hex(e.what()));
            }
            push_config(l, c);
            emit(l);
        };
        after("sync_mixed_filament_nozzle_diameters", [](DynamicPrintConfig &c) { sync_mixed_filament_nozzle_diameters(c); return std::string("-"); });
        for (const std::vector<std::string> &sup :
             std::vector<std::vector<std::string>>{{}, {"0.4;0.6", "", "0.8", "0.2;0.4;abc", "0.6", "0.2"}}) {
            after("complete_filament_nozzle_diameter_selections",
                  [&sup](DynamicPrintConfig &c) { return num(complete_filament_nozzle_diameter_selections(c, sup)); });
        }
        after("sync_nozzle_volume_type_to_extruder_count_0", [](DynamicPrintConfig &c) { sync_nozzle_volume_type_to_extruder_count(c, false); return std::string("-"); });
        after("sync_nozzle_volume_type_to_extruder_count_1", [](DynamicPrintConfig &c) { sync_nozzle_volume_type_to_extruder_count(c, true); return std::string("-"); });
        after("split_nozzle_stats_for_export", [](DynamicPrintConfig &c) { split_nozzle_stats_for_export(c); return std::string("-"); });
        after("repair_invalid_filament_extrusion_parameters", [](DynamicPrintConfig &c) { c.repair_invalid_filament_extrusion_parameters(); return std::string("-"); });
        after("normalize_fdm", [](DynamicPrintConfig &c) { c.normalize_fdm(); return std::string("-"); });
        after("normalize_fdm_1", [](DynamicPrintConfig &c) { c.normalize_fdm_1(); return std::string("-"); });
        after("normalize_fdm_2", [](DynamicPrintConfig &c) {
            std::string r;
            for (const auto &k : c.normalize_fdm_2(1, 1, nullptr)) r += k + ";";
            r += "|";
            DynamicConfig ori;
            for (const auto &k : c.normalize_fdm_2(2, 2, &ori)) r += k + ";";
            r += "|";
            for (const auto &k : c.normalize_fdm_2(2, 1, &ori)) r += k + ";";
            r += "|";
            for (const auto &k : c.normalize_fdm_2(1, 3, &ori)) r += k + ";";
            r += "|" + num(ori.size());
            return hex(r);
        });
        after("handle_legacy_sla", [](DynamicPrintConfig &c) { handle_legacy_sla(c); return std::string("-"); });
    }
}

// The options of a configuration with their classes; an enum vector without a keys map (a member of a
// static configuration) is written as its values.
static void push_config_classes(Toks &l, const ConfigBase &cfg)
{
    t_config_option_keys keys = cfg.keys();
    l.push_back(num(keys.size()));
    for (const std::string &k : keys) {
        const ConfigOption *o   = cfg.option(k);
        const char         *cls = class_name(o);
        l.push_back(hex(k));
        l.push_back(cls);
        if ((strcmp(cls, "EnumsGeneric") == 0 && static_cast<const ConfigOptionEnumsGeneric *>(o)->keys_map == nullptr) ||
            (strcmp(cls, "EnumsGenericNullable") == 0 && static_cast<const ConfigOptionEnumsGenericNullable *>(o)->keys_map == nullptr)) {
            const std::vector<int> &v = static_cast<const ConfigOptionInts *>(o)->values;
            std::string             e = "*" + num(v.size());
            for (int x : v) e += "," + num(x);
            l.push_back(e);
        } else {
            try {
                l.push_back(hex(o->serialize()));
            } catch (const std::exception &e) {
                l.push_back("!" + hex(e.what()));
            }
        }
    }
}

template<typename T> static void gen_static(const char *name)
{
    Toks l{"static", name};
    push_config_classes(l, T::defaults());
    emit(l);
}

static void gen_static_configs()
{
    gen_static<PrintObjectConfig>("PrintObjectConfig");
    gen_static<PrintRegionConfig>("PrintRegionConfig");
    gen_static<MachineEnvelopeConfig>("MachineEnvelopeConfig");
    gen_static<GCodeConfig>("GCodeConfig");
    gen_static<PrintConfig>("PrintConfig");
    gen_static<FullPrintConfig>("FullPrintConfig");
    gen_static<SLAPrintConfig>("SLAPrintConfig");
    gen_static<SLAPrintObjectConfig>("SLAPrintObjectConfig");
    gen_static<SLAMaterialConfig>("SLAMaterialConfig");
    gen_static<SLAPrinterConfig>("SLAPrinterConfig");
    gen_static<SLAFullPrintConfig>("SLAFullPrintConfig");
    {
        Toks l{"fn", "full_print_config", "=>"};
        push_config_classes(l, DynamicPrintConfig::full_print_config());
        emit(l);
    }
    std::vector<std::string> keys = print_config_def.extruder_option_keys();
    for (const char *k : {"printer_extruder_variant", "filament_extruder_variant", "print_extruder_variant", "brim_type",
                          "printhost_apikey", "z_hop_types", "nozzle_volume_type", "machine_max_speed_x", "filament_flow_ratio"})
        keys.push_back(k);
    std::unique_ptr<DynamicPrintConfig> base(DynamicPrintConfig::new_from_defaults_keys(keys));
    {
        Toks l{"fn", "new_from_defaults_keys", "=>"};
        push_config_classes(l, *base);
        emit(l);
    }
    for (const char *variants : {"", "\"Direct Drive Standard\";\"Direct Drive High Flow\";\"Bowden Standard\""}) {
        DynamicPrintConfig with = *base;
        if (*variants) {
            with.set_deserialize_strict("printer_extruder_variant", variants);
            with.set_deserialize_strict("filament_extruder_variant", variants);
            with.set_deserialize_strict("retraction_length", "1,2");
            with.set_deserialize_strict("nozzle_diameter", "0.4,0.6,0.8,1.0");
        }
        for (unsigned int n : {0u, 1u, 2u, 3u}) {
            DynamicPrintConfig c = with;
            Toks               l{"fn", "set_num_extruders", hex(variants), num(n), "=>"};
            try {
                c.set_num_extruders(n);
                c.set_num_filaments(n);
                l.push_back("-");
            } catch (const std::exception &e) {
                l.push_back("!" + hex(e.what()));
            }
            push_config_classes(l, c);
            emit(l);
        }
    }
}

// update_values_from_single_to_multi / update_values_from_multi_to_multi over process configurations.
static const std::vector<Scenario> &process_scenarios()
{
    static const std::vector<Scenario> s = {
        {{"print_extruder_variant", "\"Direct Drive Standard\""}, {"print_extruder_id", "1"}, {"outer_wall_speed", "200"},
         {"inner_wall_speed", "100"}, {"small_perimeter_speed", "40%"}, {"sparse_infill_acceleration", "5000"},
         {"enable_overhang_speed", "1"}, {"wall_loops", "3"}},
        {{"print_extruder_variant", "\"Direct Drive Standard\";\"Direct Drive High Flow\""}, {"print_extruder_id", "1,2"},
         {"outer_wall_speed", "150,250"}, {"inner_wall_speed", "300,50"}, {"small_perimeter_speed", "30%,60"},
         {"sparse_infill_acceleration", "100%,4000"}, {"enable_overhang_speed", "0,1"}, {"top_solid_infill_flow_ratio", "1,0.9"}},
        {{"print_extruder_variant", "\"Direct Drive High Flow\";\"Direct Drive Standard\";\"Direct Drive High Flow\";\"Direct Drive Standard\""},
         {"print_extruder_id", "1,1,2,2"}, {"outer_wall_speed", "100,110,120,130"}, {"inner_wall_speed", "1000,1000,1000,10"},
         {"small_perimeter_speed", "10,20%,30,40%"}, {"sparse_infill_acceleration", "1,2,3,4"}, {"enable_overhang_speed", "1,0,0,1"},
         {"top_solid_infill_flow_ratio", "0.5,0.6,0.7,0.8"}},
        {{"print_extruder_variant", "\"Bowden Standard\";\"Bowden Standard\""}, {"print_extruder_id", "2,1"}, {"outer_wall_speed", "80,90"},
         {"inner_wall_speed", "nil,70"}, {"small_perimeter_speed", "5,6"}, {"sparse_infill_acceleration", "50%,7"},
         {"enable_overhang_speed", "nil,0"}},
    };
    return s;
}

static void gen_update_values()
{
    const std::set<std::string> key_set = {"outer_wall_speed", "inner_wall_speed", "small_perimeter_speed", "sparse_infill_acceleration",
                                           "enable_overhang_speed", "print_extruder_id", "print_extruder_variant",
                                           "top_solid_infill_flow_ratio", "wall_loops", "not_a_key", "filament_type"};
    const std::vector<Scenario> &ps = process_scenarios();
    for (size_t a = 0; a < ps.size(); ++a) {
        Toks l{"process", num(a), num(ps[a].size())};
        for (const auto &kv : ps[a]) {
            l.push_back(hex(kv.first));
            l.push_back(hex(kv.second));
        }
        emit(l);
    }
    const std::vector<std::vector<std::string>> new_variants = {
        {"Direct Drive Standard", "Direct Drive High Flow"}, {"Direct Drive High Flow", "Direct Drive High Flow"}, {"Bowden Standard"}, {}};
    for (size_t a = 0; a < ps.size(); ++a)
        for (size_t b = 0; b < ps.size(); ++b) {
            {
                DynamicPrintConfig c     = make_config(ps[a]);
                DynamicPrintConfig multi = make_config(ps[b]);
                std::set<std::string> ks = key_set;
                Toks               l{"fn", "update_values_from_single_to_multi", num(a), num(b), "=>"};
                try {
                    l.push_back(num(c.update_values_from_single_to_multi(multi, ks, "print_extruder_id", "print_extruder_variant")));
                } catch (const std::exception &e) {
                    l.push_back("!" + hex(e.what()));
                }
                push_config(l, c);
                emit(l);
            }
            {
                DynamicPrintConfig       c   = make_config(ps[a]);
                const DynamicPrintConfig dst = make_config(ps[b]);
                const std::set<std::string> ks = {"outer_wall_speed", "inner_wall_speed", "small_perimeter_speed",
                                                  "sparse_infill_acceleration", "enable_overhang_speed", "wall_loops", "print_extruder_id"};
                std::vector<std::string> src_v = c.option<ConfigOptionStrings>("print_extruder_variant")->values;
                std::vector<std::string> dst_v = dst.option<ConfigOptionStrings>("print_extruder_variant")->values;
                Toks l{"fn", "update_values_from_multi_to_multi_2", num(a), num(b), "=>"};
                try {
                    l.push_back(num(c.update_values_from_multi_to_multi_2(src_v, dst_v, dst, ks)));
                } catch (const std::exception &e) {
                    l.push_back("!" + hex(e.what()));
                }
                push_config(l, c);
                emit(l);
            }
            for (size_t v = 0; v < new_variants.size(); ++v) {
                DynamicPrintConfig       c   = make_config(ps[a]);
                DynamicPrintConfig       nc  = make_config(ps[b]);
                std::set<std::string>    ks  = key_set;
                std::vector<std::string> nv  = new_variants[v];
                Toks                     l{"fn", "update_values_from_multi_to_multi", num(a), num(b), num(v), "=>"};
                try {
                    l.push_back(num(c.update_values_from_multi_to_multi(nc, ks, "print_extruder_id", "print_extruder_variant", nv)));
                } catch (const std::exception &e) {
                    l.push_back("!" + hex(e.what()));
                }
                push_config(l, c);
                emit(l);
            }
        }
}

static std::string ints_str(const std::vector<int> &v)
{
    std::string s = num(v.size());
    for (int x : v) s += "," + num(x);
    return s;
}

// update_static_print_config_from_dynamic, align_filament_variant_overrides,
// filament_override_default_indices and compute_filament_override_value.
static void gen_overrides()
{
    const std::vector<Scenario> &ps = process_scenarios();
    const std::set<std::string>  key_set1 = {"outer_wall_speed", "inner_wall_speed", "small_perimeter_speed", "enable_overhang_speed",
                                             "print_extruder_variant"};
    const std::vector<std::vector<int>> vis = {{}, {1, 0}, {0}, {3, -1}};
    for (size_t a = 0; a < ps.size(); ++a)
        for (size_t b = 0; b < ps.size(); ++b)
            for (size_t v = 0; v < vis.size(); ++v) {
                DynamicPrintConfig       c    = make_config(ps[a]);
                const DynamicPrintConfig dest = make_config(ps[b]);
                std::set<std::string>    ks   = key_set1;
                Toks                     l{"fn", "update_static_print_config_from_dynamic", num(a), num(b), num(v), "=>"};
                try {
                    update_static_print_config_from_dynamic(c, dest, vis[v], ks, 1);
                    l.push_back("-");
                } catch (const std::exception &e) {
                    l.push_back("!" + hex(e.what()));
                }
                push_config(l, c);
                emit(l);
            }

    {
        const std::set<std::string> different = {"outer_wall_speed", "small_perimeter_speed", "enable_overhang_speed", "inner_wall_speed"};
        const std::set<std::string> ks1       = {"outer_wall_speed", "small_perimeter_speed", "enable_overhang_speed"};
        const std::vector<std::set<std::string>> ks2s = {{}, {"inner_wall_speed"}};
        for (size_t a = 0; a < ps.size(); ++a)
            for (size_t b = 0; b < ps.size(); ++b)
                for (size_t k2 = 0; k2 < ks2s.size(); ++k2)
                    for (const char *idn : {"print_extruder_id", ""}) {
                        {
                            DynamicPrintConfig c  = make_config(ps[a]);
                            DynamicPrintConfig nc = make_config(ps[b]);
                            std::set<std::string> s1 = ks1, s2 = ks2s[k2];
                            Toks l{"fn", "update_non_diff_values_to_base_config", num(a), num(b), num(k2), hex(idn), "=>"};
                            try {
                                c.update_non_diff_values_to_base_config(nc, nc.keys(), different, idn, "print_extruder_variant", s1, s2);
                                l.push_back("-");
                            } catch (const std::exception &e) {
                                l.push_back("!" + hex(e.what()));
                            }
                            push_config(l, c);
                            emit(l);
                        }
                        {
                            DynamicPrintConfig c  = make_config(ps[a]);
                            DynamicPrintConfig nc = make_config(ps[b]);
                            std::set<std::string> s1 = ks1, s2 = ks2s[k2];
                            Toks l{"fn", "update_diff_values_to_child_config", num(a), num(b), num(k2), hex(idn), "=>"};
                            try {
                                c.update_diff_values_to_child_config(nc, idn, "print_extruder_variant", s1, s2);
                                l.push_back("-");
                            } catch (const std::exception &e) {
                                l.push_back("!" + hex(e.what()));
                            }
                            push_config(l, c);
                            emit(l);
                        }
                    }
    }

    const std::vector<Scenario> align = {
        {{"handle_hotend_as_extruder", "1"},
         {"filament_extruder_variant", "\"Direct Drive Standard\";\"Direct Drive High Flow\";\"Direct Drive Standard\";\"Direct Drive High Flow\""},
         {"filament_self_index", "1,1,2,2"}, {"filament_retract_length_toolchange", "1.5,nil"},
         {"filament_retract_restart_extra_toolchange", "0.5,0.6,0.7,0.8"}},
        {{"handle_hotend_as_extruder", "1"}, {"filament_extruder_variant", "\"a\";\"b\";\"c\";\"d\""}, {"filament_self_index", "1,2,2,3"},
         {"filament_retract_length_toolchange", "1,2,3"}, {"filament_retract_restart_extra_toolchange", "nil,4,5"}},
        {{"handle_hotend_as_extruder", "0"}, {"filament_extruder_variant", "\"a\";\"b\""}, {"filament_self_index", "1,1"},
         {"filament_retract_length_toolchange", "1"}},
        {{"handle_hotend_as_extruder", "1"}, {"filament_extruder_variant", "\"a\";\"b\""}, {"filament_self_index", "0,0"},
         {"filament_retract_length_toolchange", "1"}},
        {{"handle_hotend_as_extruder", "1"}, {"filament_extruder_variant", "\"a\";\"b\";\"c\""}, {"filament_self_index", "1,1,1"},
         {"filament_retract_restart_extra_toolchange", "9"}},
        {{"handle_hotend_as_extruder", "1"}, {"filament_extruder_variant", "\"a\";\"b\";\"c\""}, {"filament_self_index", "1,1"}},
    };
    for (size_t a = 0; a < align.size(); ++a) {
        Toks l0{"overrides", num(a), num(align[a].size())};
        for (const auto &kv : align[a]) {
            l0.push_back(hex(kv.first));
            l0.push_back(hex(kv.second));
        }
        emit(l0);
        DynamicPrintConfig c = make_config(align[a]);
        Toks               l{"fn", "align_filament_variant_overrides", num(a), "=>"};
        try {
            align_filament_variant_overrides(c);
            l.push_back("-");
        } catch (const std::exception &e) {
            l.push_back("!" + hex(e.what()));
        }
        push_config(l, c);
        emit(l);
    }

    const Scenario full = {
        {"filament_self_index", "1,1,2,2,3"},
        {"filament_extruder_variant",
         "\"Direct Drive Standard\";\"Direct Drive High Flow\";\"Direct Drive Standard\";\"Bowden High Flow\";\"Direct Drive High Flow\""},
        {"printer_extruder_variant", "\"Direct Drive Standard\";\"Direct Drive High Flow\";\"Bowden Standard\""}};
    const DynamicPrintConfig fc = make_config(full);
    const std::vector<std::vector<int>> fmaps = {{}, {1, 0, 2}, {2, 2, 2, 2}, {-1, 5}};
    for (const char *k : {"retraction_length", "z_hop", "wall_loops", "retract_restart_extra"})
        for (size_t rows : {size_t(5), size_t(3)})
            for (size_t m = 0; m < fmaps.size(); ++m)
                emit({"fn", "filament_override_default_indices", hex(k), num(rows), num(m), "=>",
                      ints_str(filament_override_default_indices(k, rows, fc, fmaps[m]))});

    struct OverrideCase { const char *key, *cls, *old_machine, *new_machine, *filament; };
    const std::vector<OverrideCase> cases = {
        {"retraction_length", "FloatsNullable", "0.8,0.8", "0.8,1.0", "nil,1.5"},
        {"retraction_length", "FloatsNullable", "0.8,0.8", "0.8,0.8", "nil,nil"},
        {"retraction_length", "FloatsNullable", "0.4,0.8", "0.8,1.0", "0.4,nil"},
        {"z_hop", "FloatsNullable", "0.4,0.4", "0.4,0.4", "0.4,nil"},
        {"z_hop", "FloatsNullable", "0.4,0.4", "0.2,0.6", "nil,0.3"},
        {"long_retractions_when_cut", "BoolsNullable", "1,0", "1,0", "nil,1"},
        {"long_retractions_when_cut", "BoolsNullable", "0,0", "1,0", "0,nil"},
        {"retraction_distances_when_cut", "FloatsNullable", "18,18", "18,18", "10,nil"},
        {"retraction_distances_when_cut", "FloatsNullable", "18,18", "20,18", "nil,nil"},
    };
    for (const char *longret : {"0", "2"})
        for (const char *multi : {"0", "1"}) {
            const Scenario sc = {{"enable_long_retraction_when_cut", longret}, {"multi_nozzle_diameter_mode", multi},
                                 {"filament_self_index", "1,2"}, {"filament_diameter_map", "\"0.2\";\"0.6\""},
                                 {"filament_extruder_variant", "\"Direct Drive Standard\";\"Direct Drive High Flow\""},
                                 {"printer_extruder_variant", "\"Direct Drive Standard\";\"Direct Drive High Flow\""}};
            const DynamicPrintConfig nfc = make_config(sc);
            for (size_t ci = 0; ci < cases.size(); ++ci) {
                const OverrideCase &oc = cases[ci];
                std::unique_ptr<ConfigOption> om, nm, nf;
                if (strcmp(oc.cls, "FloatsNullable") == 0) {
                    om.reset(new ConfigOptionFloatsNullable());
                    nm.reset(new ConfigOptionFloatsNullable());
                    nf.reset(new ConfigOptionFloatsNullable());
                } else {
                    om.reset(new ConfigOptionBoolsNullable());
                    nm.reset(new ConfigOptionBoolsNullable());
                    nf.reset(new ConfigOptionBoolsNullable());
                }
                om->deserialize(oc.old_machine);
                nm->deserialize(oc.new_machine);
                nf->deserialize(oc.filament);
                t_config_option_keys diff_keys;
                DynamicPrintConfig   overrides;
                std::vector<int>     fmap = {0, 1};
                Toks                 l{"fn", "compute_filament_override_value", hex(longret), hex(multi), num(ci), "=>"};
                try {
                    compute_filament_override_value(oc.key, om.get(), nm.get(), nf.get(), nfc, diff_keys, overrides, fmap);
                    l.push_back("-");
                } catch (const std::exception &e) {
                    l.push_back("!" + hex(e.what()));
                }
                push_list(l, diff_keys);
                push_config(l, overrides);
                l.push_back(ints_str(fmap));
                emit(l);
            }
        }
}

// R943: update_values_to_printer_extruders_for_multiple_filaments (PrintConfig.cpp:9996-10191) on
// a few filament tables, the printer config a copy of the config (as Print::apply passes
// m_ori_full_print_config). Written to filaments.txt.
static const std::vector<std::pair<std::pair<int, int>, Scenario>> &filament_scenarios()
{
    static const std::vector<std::pair<std::pair<int, int>, Scenario>> s = {
        {{1, 1}, {{"filament_map", "1,1"}, {"extruder_type", "Direct Drive"}, {"nozzle_volume_type", "Standard"},
                  {"filament_extruder_variant", "\"Direct Drive Standard\";\"Direct Drive High Flow\";\"Direct Drive Standard\";\"Direct Drive High Flow\""},
                  {"filament_self_index", "1,1,2,2"}, {"filament_max_volumetric_speed", "10,20,11,21"}, {"filament_flow_ratio", "0.9,0.8,0.95,0.85"}}},
        {{2, 2}, {{"filament_map", "1,2,2"}, {"extruder_type", "Direct Drive,Bowden"}, {"nozzle_volume_type", "Standard,High Flow"},
                  {"filament_volume_map", "0,1,1"},
                  {"filament_extruder_variant", "\"Direct Drive Standard\";\"Bowden High Flow\";\"Direct Drive Standard\";\"Bowden High Flow\";\"Bowden High Flow\""},
                  {"filament_self_index", "1,1,2,2,3"}, {"filament_max_volumetric_speed", "1,2,3,4,5"}}},
        {{2, 3}, {{"filament_map", "1,2,2"}, {"extruder_type", "Direct Drive,Bowden"}, {"nozzle_volume_type", "Standard,Standard"},
                  {"filament_volume_map", "0,1,0"},
                  {"filament_extruder_variant", "\"Direct Drive Standard\";\"Bowden High Flow\";\"Bowden Standard\";\"Bowden High Flow\";\"Bowden Standard\""},
                  {"filament_self_index", "1,2,2,3,3"}, {"filament_max_volumetric_speed", "1,2,3,4,5"}, {"filament_flow_ratio", "0.5,0.6,0.7,0.8,0.9"}}},
        {{1, 1}, {{"filament_map", "1,1"}, {"extruder_type", "Direct Drive"}, {"nozzle_volume_type", "High Flow"},
                  {"filament_extruder_variant", "\"Direct Drive Standard\";\"Direct Drive Standard\""},
                  {"filament_self_index", "2,1"}, {"filament_max_volumetric_speed", "7,8"}}},
    };
    return s;
}

static void gen_filaments()
{
    const auto &fs = filament_scenarios();
    for (size_t a = 0; a < fs.size(); ++a) {
        DynamicPrintConfig c = make_config(fs[a].second);
        DynamicPrintConfig p = c;
        std::set<std::string> ks = filament_options_with_variant;
        ks.insert("filament_self_index");
        Toks l{"fn", "update_values_to_printer_extruders_for_multiple_filaments", num(a), "=>"};
        c.update_values_to_printer_extruders_for_multiple_filaments(p, fs[a].first.first, fs[a].first.second, ks, "filament_self_index",
                                                                    "filament_extruder_variant");
        push_config(l, c);
        emit(l);
    }
}

// R944: update_filament_config_values_for_multiple_extruders (PrintConfig.cpp:10194-10320) over
// filament_scenarios() with a nozzle-info map per scenario (used filaments with one or two nozzles,
// unused ones through filament_volume_map, and the opt_ids fallback of scenario 3).
static void gen_filament_nozzle_infos()
{
    const auto &fs = filament_scenarios();
    std::vector<std::unordered_map<int, std::vector<ExtruderNozleInfo>>> maps(fs.size());
    maps[0][0] = {{etDirectDrive, nvtStandard}, {etDirectDrive, nvtHighFlow}};
    maps[1][1] = {{etBowden, nvtHighFlow}};
    maps[1][2] = {{etBowden, nvtHighFlow}};
    maps[2][1] = {{etBowden, nvtStandard}, {etBowden, nvtHighFlow}};
    for (size_t a = 0; a < fs.size(); ++a) {
        DynamicPrintConfig c = make_config(fs[a].second);
        DynamicPrintConfig p = c;
        // The C++ dereferences option<T>(key) of every key of the set: only the keys the config holds.
        std::set<std::string> ks;
        for (const std::string &k : filament_options_with_variant)
            if (c.has(k))
                ks.insert(k);
        Toks l{"fn", "update_filament_config_values_for_multiple_extruders", num(a), "=>"};
        c.update_filament_config_values_for_multiple_extruders(p, maps[a], fs[a].first.first, fs[a].first.second, ks, "filament_self_index",
                                                               "filament_extruder_variant");
        push_config(l, c);
        emit(l);
    }
}

// R943: update_values_to_printer_extruders (PrintConfig.cpp:9780-9994) over the printer options
// with variant (stride 1), for the machine (extruder_id 0) and for extruder 1 / 2 with a filament
// volume type. Written to filaments.txt after gen_filaments.
static void gen_printer_extruders()
{
    const std::vector<Scenario> ps = {
        {{"extruder_type", "Direct Drive"}, {"nozzle_volume_type", "Standard"},
         {"printer_extruder_variant", "\"Direct Drive Standard\";\"Direct Drive High Flow\""}, {"printer_extruder_id", "1,1"},
         {"retraction_length", "0.8,1.2"}, {"z_hop", "0.4,0.6"}},
        {{"extruder_type", "Direct Drive,Direct Drive"}, {"nozzle_volume_type", "Standard,High Flow"},
         {"printer_extruder_variant", "\"Direct Drive Standard\";\"Direct Drive High Flow\";\"Direct Drive Standard\";\"Direct Drive High Flow\""},
         {"printer_extruder_id", "1,1,2,2"}, {"retraction_length", "1,2,3,4"}, {"z_hop", "0.1,0.2,0.3,0.4"}},
        {{"extruder_type", "Direct Drive,Bowden"}, {"nozzle_volume_type", "Hybrid,Standard"},
         {"printer_extruder_variant", "\"Direct Drive Standard\";\"Direct Drive High Flow\";\"Bowden Standard\""},
         {"printer_extruder_id", "1,1,2"}, {"retraction_length", "1,2,3"}, {"z_hop", "0.1,0.2,0.3"}},
    };
    for (size_t a = 0; a < ps.size(); ++a)
        for (unsigned int extruder_id = 0; extruder_id < 3; ++extruder_id) {
            DynamicPrintConfig c = make_config(ps[a]);
            int extruder_count = 1;
            c.support_different_extruders(extruder_count);
            std::vector<std::vector<NozzleVolumeType>> nvt;
            int envc = c.get_extruder_nozzle_volume_count(extruder_count, nvt);
            std::set<std::string> ks = printer_options_with_variant_1;
            std::vector<int> vi = c.update_values_to_printer_extruders(c, extruder_count, envc, nvt, ks, "printer_extruder_id", "printer_extruder_variant", 1,
                                                                       extruder_id, nvtHighFlow);
            Toks l{"fn", "update_values_to_printer_extruders", num(a), num(extruder_id), "=>", ints_str(vi)};
            push_config(l, c);
            emit(l);
        }
}

// R945: the enum vectors (ConfigOptionEnumsGenericTempl::deserialize / serialize, Config.hpp:2128-2227)
// of the nine options a two-nozzle job sets by name, through ConfigBase::set_deserialize
// (Config.cpp:636-719) on a fresh DynamicPrintConfig (the option made by
// ConfigOptionDef::create_default_option, Config.cpp:296-325, which sets keys_map), and whether the
// option of DynamicPrintConfig::full_print_config() (a clone of FullPrintConfig::defaults(), whose
// static options get their values by set() only, PrintConfig.hpp:880-897) has a keys map. Written to
// enum_vectors.txt:
//   km <key> <created keys_map == def->enum_keys_map> <full_print_config() keys_map == nullptr>
//   ev <key> <first or -> <its result: - / ok / !what> <value> <append> <rule 0 Disable / 1 EnableSilent> => <ok or !what>
//      <n> <values>*n <serialize or !what> <substitutions>
static const std::vector<std::pair<std::string, std::string>> &enum_vector_keys()
{
    // the values of the H2D job (tests/data/tool_ordering/h2d_majora_order.txt.gz)
    static const std::vector<std::pair<std::string, std::string>> s = {
        {"cooling_slowdown_logic", "uniform_cooling,uniform_cooling,uniform_cooling,uniform_cooling,uniform_cooling,uniform_cooling,uniform_cooling,uniform_cooling"},
        {"extruder_nozzle_volume_type", "Standard,Standard"},
        {"extruder_type", "Direct Drive,Direct Drive"},
        {"filament_metal_stickiness", "None,None,None,None,None,None,None,None"},
        {"filament_scarf_seam_type", "none,none,none,none,none,none,none,none"},
        {"nozzle_volume_type", "Standard,Standard"},
        {"overhang_fan_threshold", "50%,50%,50%,50%,50%,50%,50%,50%"},
        {"overhang_threshold_participating_cooling", "95%,95%,95%,95%,95%,95%,95%,95%"},
        {"z_hop_types", "Auto Lift,Auto Lift"},
    };
    return s;
}

static void gen_enum_vectors()
{
    for (const auto &kv : enum_vector_keys()) {
        const std::string     &key = kv.first;
        const ConfigOptionDef *def = print_config_def.get(key);
        DynamicPrintConfig     fresh;
        const ConfigOption    *created = fresh.option(key, true);
        auto km_of = [](const ConfigOption *o) -> const t_config_enum_values * {
            if (auto *a = dynamic_cast<const ConfigOptionEnumsGeneric *>(o)) return a->keys_map;
            if (auto *b = dynamic_cast<const ConfigOptionEnumsGenericNullable *>(o)) return b->keys_map;
            return reinterpret_cast<const t_config_enum_values *>(1);
        };
        DynamicPrintConfig full = DynamicPrintConfig::full_print_config();
        emit({"km", hex(key), num(km_of(created) == def->enum_keys_map), num(km_of(full.option(key)) == nullptr)});

        const std::vector<std::string> &names = def->enum_values;
        std::string all, spaced;
        for (size_t i = 0; i < names.size(); ++i) {
            all += (i ? "," : "") + names[i];
            spaced += (i ? " , " : " ") + names[i];
        }
        spaced += " ";
        struct Case { std::string first; std::string value; bool append; int rule; };
        std::vector<Case> cases = {
            {"", kv.second, false, 0},
            {"", all, false, 0},
            {"", spaced, false, 0},
            {"", names.front() + ",Bogus", false, 0},
            {"", names.front() + ",Bogus", false, 1},
            {"", "nil," + names.back(), false, 0},
            {"", "", false, 0},
            {names.front(), names.back(), true, 0},
            {names.back(), names.front() + "," + names.back(), false, 1},
        };
        for (const Case &c : cases) {
            DynamicPrintConfig cfg;
            std::string        first_result = "-";
            if (!c.first.empty()) {
                try {
                    cfg.set_deserialize_strict(key, c.first);
                    first_result = "ok";
                } catch (const std::exception &e) {
                    first_result = "!" + hex(e.what());
                }
            }
            ConfigSubstitutionContext ctxt(c.rule == 0 ? ForwardCompatibilitySubstitutionRule::Disable : ForwardCompatibilitySubstitutionRule::EnableSilent);
            Toks l{"ev", hex(key), hex(c.first), first_result, hex(c.value), num(c.append), num(c.rule), "=>"};
            try {
                cfg.set_deserialize(key, c.value, ctxt, c.append);
                l.push_back("ok");
            } catch (const std::exception &e) {
                l.push_back("!" + hex(e.what()));
            }
            const ConfigOption *o = cfg.option(key);
            if (o == nullptr) {
                l.push_back("none");
            } else {
                const std::vector<int> &v = static_cast<const ConfigOptionInts *>(o)->values;
                l.push_back(num(v.size()));
                for (int x : v) l.push_back(num(x));
                try {
                    l.push_back(hex(o->serialize()));
                } catch (const std::exception &e) {
                    l.push_back("!" + hex(e.what()));
                }
            }
            l.push_back(num(ctxt.substitutions.size()));
            emit(l);
        }
    }
}

int main(int argc, char **argv)
{
    if (argc < 2) {
        fprintf(stderr, "usage: gen <output directory>\n");
        return 1;
    }
    std::string dir = argv[1];
    open_out(dir, "enums.txt");
    gen_enums();
    open_out(dir, "print_config_def.txt");
    gen_defs();
    open_out(dir, "functions.txt");
    gen_legacy();
    gen_functions();
    gen_static_configs();
    gen_update_values();
    gen_overrides();
    fclose(g_out);
    open_out(dir, "filaments.txt");
    gen_filaments();
    gen_printer_extruders();
    gen_filament_nozzle_infos();
    fclose(g_out);
    g_out = nullptr;
    open_out(dir, "enum_vectors.txt");
    gen_enum_vectors();
    fclose(g_out);
    g_out = nullptr;
    return 0;
}
