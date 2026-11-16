// Generator of the vectors of tests/placeholder_parser_native_parity.rs: libslic3r's
// PlaceholderParser.hpp / PlaceholderParser.cpp at the pin (references/BambuStudio/src/libslic3r; the
// override layer has no copy of them).
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
//   env -i ./gen $REPO/references/BambuStudio/resources/profiles/BBL > placeholder_parser.txt
//   gzip -9 -n -c placeholder_parser.txt > $DATA/placeholder_parser.txt.gz
//
// Run it with an empty environment (env -i): the parser reads the SLIC3R_* variables of the
// environment. The vectors in this directory were made on macOS arm64 with Apple clang and libc++
// (the distributions behind random() are those of libc++). GEN_TRACE=1 (env -i GEN_TRACE=1 ./gen ...)
// writes each loaded profile file, option and case to stderr before it runs, to find a case that crashes.
// The cases avoid the inputs the C++ dereferences a null pointer on or loops forever with (a percentage
// variable without a definition, a legacy [a[b]] whose vector does not exist, a vector read as a scalar in
// evaluate_boolean_expression, a ratio_over chain over a vector); placeholder_parser.rs documents them.
//
// Line format: tokens separated by one space; a text token is the hexadecimal of its bytes ("-" for
// the empty text). The first token names the line:
//
//   env <name>                         starts an environment; the lines up to "end" describe it
//   ext <key> <text>                   an option of the external configuration, a DynamicPrintConfig:
//                                      set_deserialize_strict(key, text)
//   ppv <key> <type> <text>            an option set into the parser (PlaceholderParser::set)
//   ovr <key> <type> <text>            an option of the override configuration (config_override)
//   end
//   case <env> <mode> <extruder> <override 0|1> <template> <class> <text>
//                                      mode P: process(template, extruder, override, nullptr);
//                                      mode R: the same with a fresh ContextData (default seed);
//                                      mode B: evaluate_boolean_expression(template, external, override).
//                                      class "ok" and the result ("true"/"false" for B), or the class
//                                      of the exception (PPE PlaceholderParserError, FEMV
//                                      FlowErrorMissingVariable, RE RuntimeError, CE ConfigurationError,
//                                      EX another std::exception) and its what()
//   api <name> <tokens...>             the PlaceholderParser methods, see gen_api()
//
// <type> of ppv / ovr: i int, f float, s string, b bool, p percent, fp float-or-percent, pt point,
// is ints, isn ints nullable, fs floats, fsn floats nullable, ss strings, bs bools, ps percents,
// fps floats-or-percents, pts points. The option is made empty and deserialize(text) fills it.

#include "libslic3r/PlaceholderParser.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/Flow.hpp"
#include "libslic3r/Exception.hpp"
#include "libslic3r/libslic3r.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <vector>

using namespace Slic3r;

static std::string hex(const std::string &s)
{
    if (s.empty())
        return "-";
    static const char *d = "0123456789abcdef";
    std::string o;
    for (unsigned char c : s) {
        o += d[c >> 4];
        o += d[c & 15];
    }
    return o;
}

static ConfigOption *make_opt(const std::string &type)
{
    if (type == "i") return new ConfigOptionInt();
    if (type == "f") return new ConfigOptionFloat();
    if (type == "s") return new ConfigOptionString();
    if (type == "b") return new ConfigOptionBool();
    if (type == "p") return new ConfigOptionPercent();
    if (type == "fp") return new ConfigOptionFloatOrPercent();
    if (type == "pt") return new ConfigOptionPoint();
    if (type == "is") return new ConfigOptionInts();
    if (type == "isn") return new ConfigOptionIntsNullable();
    if (type == "fs") return new ConfigOptionFloats();
    if (type == "fsn") return new ConfigOptionFloatsNullable();
    if (type == "ss") return new ConfigOptionStrings();
    if (type == "bs") return new ConfigOptionBools();
    if (type == "ps") return new ConfigOptionPercents();
    if (type == "fps") return new ConfigOptionFloatsOrPercents();
    if (type == "pts") return new ConfigOptionPoints();
    fprintf(stderr, "bad type %s\n", type.c_str());
    exit(1);
}

struct Var { std::string key, type, text; };

struct Env {
    std::string                    name;
    DynamicPrintConfig             ext;
    std::vector<Var>               ppv;
    std::vector<Var>               ovr;
    std::unique_ptr<PlaceholderParser> pp;
    DynamicConfig                  override_cfg;
};

static std::map<std::string, std::unique_ptr<Env>> g_envs;

// Writes the environment and rebuilds its configurations from the written texts, as the Rust test does.
static Env &define_env(const std::string &name, const DynamicPrintConfig &ext_in, const std::vector<Var> &ppv, const std::vector<Var> &ovr)
{
    auto env = std::make_unique<Env>();
    env->name = name;
    printf("env %s\n", name.c_str());
    for (const std::string &key : ext_in.keys()) {
        if (getenv("GEN_TRACE"))
            fprintf(stderr, "ext %s\n", key.c_str());
        std::string text = ext_in.option(key)->serialize();
        printf("ext %s %s\n", key.c_str(), hex(text).c_str());
        env->ext.set_deserialize_strict(key, text);
    }
    env->pp = std::make_unique<PlaceholderParser>(&env->ext);
    for (const Var &v : ppv) {
        ConfigOption *opt = make_opt(v.type);
        opt->deserialize(v.text);
        printf("ppv %s %s %s\n", v.key.c_str(), v.type.c_str(), hex(v.text).c_str());
        env->pp->set(v.key, opt);
    }
    for (const Var &v : ovr) {
        ConfigOption *opt = make_opt(v.type);
        opt->deserialize(v.text);
        printf("ovr %s %s %s\n", v.key.c_str(), v.type.c_str(), hex(v.text).c_str());
        env->override_cfg.set_key_value(v.key, opt);
    }
    printf("end\n");
    Env &ref = *env;
    g_envs[name] = std::move(env);
    return ref;
}

static void run_case(const std::string &env_name, char mode, int extruder, bool use_override, const std::string &templ)
{
    Env &env = *g_envs.at(env_name);
    if (getenv("GEN_TRACE"))
        fprintf(stderr, "case %s %c %d %d %s\n", env_name.c_str(), mode, extruder, use_override ? 1 : 0, templ.c_str());
    std::string cls = "ok", text;
    try {
        if (mode == 'B') {
            bool r = PlaceholderParser::evaluate_boolean_expression(templ, env.ext, use_override ? &env.override_cfg : nullptr);
            text = r ? "true" : "false";
        } else if (mode == 'R') {
            PlaceholderParser::ContextData ctx;
            text = env.pp->process(templ, (unsigned int)extruder, use_override ? &env.override_cfg : nullptr, &ctx);
        } else {
            text = env.pp->process(templ, (unsigned int)extruder, use_override ? &env.override_cfg : nullptr, nullptr);
        }
    } catch (const PlaceholderParserError &e) {
        cls = "PPE"; text = e.what();
    } catch (const FlowErrorMissingVariable &e) {
        cls = "FEMV"; text = e.what();
    } catch (const Slic3r::RuntimeError &e) {
        cls = "RE"; text = e.what();
    } catch (const ConfigurationError &e) {
        cls = "CE"; text = e.what();
    } catch (const std::exception &e) {
        cls = "EX"; text = e.what();
    }
    printf("case %s %c %d %d %s %s %s\n", env_name.c_str(), mode, extruder, use_override ? 1 : 0, hex(templ).c_str(), cls.c_str(), hex(text).c_str());
}

// ---------------------------------------------------------------------------------------------
// The Bambu profiles
// ---------------------------------------------------------------------------------------------

static nlohmann::json read_json(const std::string &path)
{
    std::ifstream ifs(path);
    if (!ifs) {
        fprintf(stderr, "cannot read %s\n", path.c_str());
        exit(1);
    }
    nlohmann::json j;
    ifs >> j;
    return j;
}

// The files of a preset from its root ancestor down to itself, by "inherits", then its "include"s.
static void load_chain(DynamicPrintConfig &cfg, const std::string &dir, const std::string &name)
{
    std::vector<std::string> files;
    std::vector<std::string> includes;
    std::string n = name;
    while (!n.empty()) {
        std::string path = dir + "/" + n + ".json";
        files.push_back(path);
        nlohmann::json j = read_json(path);
        if (j.contains("include")) {
            for (auto &inc : j["include"])
                includes.push_back(dir + "/" + inc.get<std::string>() + ".json");
        }
        n = j.contains("inherits") ? j["inherits"].get<std::string>() : std::string();
    }
    std::reverse(files.begin(), files.end());
    files.insert(files.end(), includes.begin(), includes.end());
    for (const std::string &f : files) {
        if (getenv("GEN_TRACE"))
            fprintf(stderr, "load %s\n", f.c_str());
        std::map<std::string, std::string> key_values;
        std::string                        reason;
        cfg.load_from_json(f, ForwardCompatibilitySubstitutionRule::Enable, key_values, reason);
    }
}

static DynamicPrintConfig profile_config(const std::string &bbl, const std::string &machine, const std::string &process, const std::string &filament)
{
    // The defaults through the definitions (full_print_config() holds enum vectors without their key maps).
    std::unique_ptr<DynamicPrintConfig> defaults(DynamicPrintConfig::new_from_defaults_keys(FullPrintConfig::defaults().keys()));
    DynamicPrintConfig cfg = *defaults;
    load_chain(cfg, bbl + "/machine", machine);
    load_chain(cfg, bbl + "/process", process);
    load_chain(cfg, bbl + "/filament", filament);
    return cfg;
}

// The placeholders GCode.cpp sets before it processes the machine templates (their types follow GCode.cpp).
static std::vector<Var> gcode_placeholders(bool dual)
{
    std::vector<Var> v = {
        {"bed_temperature", "is", dual ? "55,55" : "55"},
        {"bed_temperature_initial_layer", "is", dual ? "55,55" : "55"},
        {"bed_temperature_initial_layer_single", "i", "55"},
        {"overall_chamber_temperature", "i", "0"},
        {"first_layer_temperature", "isn", dual ? "220,220" : "220"},
        {"first_layer_print_min", "fs", "110.5,98.25"},
        {"first_layer_print_size", "fs", "35,42.5"},
        {"first_layer_center_no_wipe_tower", "fs", "128,128"},
        {"first_filaments", "is", dual ? "0,1" : "0"},
        {"first_non_support_filaments", "is", dual ? "0,1" : "0"},
        {"first_non_support_hotend", "is", dual ? "0,1" : "0"},
        {"during_print_exhaust_fan_speed_num", "is", dual ? "0,0" : "0"},
        {"has_tpu_in_first_layer", "b", "0"},
        {"has_wipe_tower", "b", dual ? "1" : "0"},
        {"hold_chamber_temp_for_flat_print", "b", "0"},
        {"in_head_wrap_detect_zone", "b", "0"},
        {"initial_extruder", "i", "0"},
        {"initial_filament_id", "i", "0"},
        {"initial_no_support_extruder", "i", "0"},
        {"initial_no_support_filament_id", "i", "0"},
        {"initial_no_support_hotend", "i", "0"},
        {"initial_no_support_nozzle_id", "i", "0"},
        {"initial_nozzle_id", "i", "0"},
        {"is_all_bbl_filament", "b", "1"},
        {"max_additional_fan", "i", "70"},
        {"max_print_z", "i", "48"},
        {"min_vitrification_temperature", "i", "45"},
        {"total_layer_count", "i", "240"},
        {"current_extruder", "i", "0"},
        {"current_hotend", "i", "0"},
        {"long_retraction_when_cut", "b", "0"},
        {"long_retraction_when_ec", "b", "1"},
        {"retraction_distance_when_cut", "f", "18"},
        {"retraction_distance_when_ec", "f", "10"},
        {"max_layer_z", "f", "48.2"},
        {"SLIC3R_PP_TEST", "s", "shadowed"},
    };
    return v;
}

static std::vector<Var> gcode_override(bool dual)
{
    std::vector<Var> v = {
        {"layer_num", "i", "12"},
        {"filament_ids", "ss", dual ? "GFA00;GFG02" : "GFA00"},
        {"layer_z", "f", "2.6"},
        {"max_layer_z", "f", "48.2"},
        {"previous_extruder", "i", "0"},
        {"next_extruder", "i", "1"},
        {"current_filament_id", "i", "0"},
        {"next_filament_id", "i", "1"},
        {"current_nozzle_id", "i", "0"},
        {"next_nozzle_id", "i", dual ? "1" : "0"},
        {"next_hotend", "i", dual ? "1" : "0"},
        {"curr_physical_extruder_id", "i", "0"},
        {"most_used_physical_extruder_id", "i", "0"},
        {"curr_y_acceleration_limit", "f", "9000"},
        {"clear_to_x0", "b", "0"},
        {"farthest_point_timelapse_enabled", "b", "0"},
        {"flush_length", "f", "152.5"},
        {"flush_length_1", "f", "40"},
        {"flush_length_2", "f", "38.5"},
        {"flush_length_3", "f", "37"},
        {"flush_length_4", "f", "37"},
        {"flush_temperatures", "is", "220,240"},
        {"flush_volumetric_speeds", "fs", "12,18.5"},
        {"has_timelapse_safe_pos", "b", "1"},
        {"is_prime_tower_interface", "b", "0"},
        {"new_extruder_retracted_length", "f", "2"},
        {"new_filament_e_feedrate", "i", "180"},
        {"new_filament_temp", "i", "240"},
        {"new_retract_length_toolchange", "f", "2"},
        {"old_filament_e_feedrate", "i", "120"},
        {"old_filament_temp", "i", "220"},
        {"old_retract_length_toolchange", "f", "2"},
        {"outer_wall_volumetric_speed", "f", "15.25"},
        {"nozzle_diameter_at_nozzle_id", "fs", dual ? "0.4,0.4" : "0.4"},
        {"nozzle_volume_types", "ss", dual ? "Standard;High Flow" : "Standard"},
        {"timelapse_inline_photo", "b", "0"},
        {"timelapse_pos_x", "i", "120"},
        {"timelapse_pos_y", "i", "250"},
        {"toolchange_count", "i", "3"},
        {"travel_point_1_x", "f", "190"},
        {"travel_point_1_y", "f", "250"},
        {"travel_point_2_x", "f", "110"},
        {"travel_point_2_y", "f", "250"},
        {"travel_point_3_x", "f", "70"},
        {"travel_point_3_y", "f", "260"},
        {"wipe_avoid_perimeter", "b", "0"},
        {"wipe_avoid_pos_x", "f", "300"},
        {"wipe_tower_center_pos_valid", "b", "1"},
        {"wipe_tower_center_pos_x", "f", "165"},
        {"wipe_tower_center_pos_y", "f", "220"},
        {"x_after_toolchange", "f", "102.5"},
        {"y_after_toolchange", "f", "88"},
        {"z_after_toolchange", "f", "2.8"},
    };
    return v;
}

static std::vector<std::string> profile_templates(const std::string &bbl)
{
    std::set<std::string> seen;
    std::vector<std::string> out;
    for (const char *sub : {"machine", "filament", "process"}) {
        std::string dir = bbl + "/" + sub;
        std::vector<std::string> files;
        // The file list comes from the BBL.json index order-independently: sort the names.
        FILE *ls = popen(("ls \"" + dir + "\"").c_str(), "r");
        char buf[4096];
        while (fgets(buf, sizeof(buf), ls)) {
            std::string f = buf;
            while (!f.empty() && (f.back() == '\n' || f.back() == '\r'))
                f.pop_back();
            if (f.size() > 5 && f.substr(f.size() - 5) == ".json")
                files.push_back(dir + "/" + f);
        }
        pclose(ls);
        std::sort(files.begin(), files.end());
        for (const std::string &f : files) {
            nlohmann::json j;
            try {
                j = read_json(f);
            } catch (...) {
                continue;
            }
            for (auto it = j.begin(); it != j.end(); ++it) {
                const std::string &k = it.key();
                if (k.size() < 6 || k.substr(k.size() - 6) != "_gcode")
                    continue;
                std::vector<std::string> vals;
                if (it.value().is_string())
                    vals.push_back(it.value().get<std::string>());
                else if (it.value().is_array())
                    for (auto &e : it.value())
                        if (e.is_string())
                            vals.push_back(e.get<std::string>());
                for (const std::string &v : vals)
                    if (seen.insert(v).second)
                        out.push_back(v);
            }
        }
    }
    return out;
}

// ---------------------------------------------------------------------------------------------
// The synthetic environment
// ---------------------------------------------------------------------------------------------

static DynamicPrintConfig synthetic_config()
{
    DynamicPrintConfig c;
    const std::vector<std::pair<std::string, std::string>> kv = {
        {"layer_height", "0.2"},
        {"initial_layer_print_height", "0.25"},
        {"nozzle_diameter", "0.4,0.6"},
        {"filament_map", "1,2,1"},
        {"line_width", "0.42"},
        {"outer_wall_line_width", "0"},
        {"inner_wall_line_width", "0.45"},
        {"sparse_infill_line_width", "0"},
        {"top_surface_line_width", "0"},
        {"support_line_width", "0"},
        {"initial_layer_line_width", "0.5"},
        {"internal_solid_infill_line_width", "0"},
        {"wall_loops", "3"},
        {"sparse_infill_density", "15%"},
        {"sparse_infill_anchor", "400%"},
        {"sparse_infill_anchor_max", "20"},
        {"seam_slope_start_height", "10%"},
        {"seam_slope_gap", "15%"},
        {"spiral_mode_max_xy_smoothing", "1.5"},
        {"filament_scarf_height", "10%,0.3"},
        {"filament_scarf_gap", "0.5"},
        {"vertical_shell_speed", "80%,60"},
        {"filament_type", "PLA;PETG;TPU"},
        {"filament_settings_id", "\"Bambu PLA Basic @BBL X1C\";\"Generic PETG\";\"\""},
        {"nozzle_temperature", "220,240,230"},
        {"nozzle_temperature_initial_layer", "225,245,nil"},
        {"bed_exclude_area", "0x0,18x0,18x28,0x28"},
        {"printable_area", "0x0,256x0,256x256,0x256"},
        {"extruder_offset", "0x2"},
        {"spiral_mode", "1"},
        {"enable_prime_tower", "0"},
        {"printer_notes", "PRINTER_VENDOR_BBL\nPRINTER_MODEL_X1"},
        {"printer_model", "Bambu Lab X1 Carbon"},
        {"curr_bed_type", "Textured PEI Plate"},
        {"wall_generator", "arachne"},
        {"print_sequence", "by layer"},
        {"filament_diameter", "1.75,1.75,1.75"},
        {"retraction_length", "0.8,1.2"},
        {"wipe", "1,0"},
        {"z_hop_types", "Auto Lift,Spiral Lift"},
        {"default_acceleration", "10000,8000"},
        {"outer_wall_speed", "200,180"},
        {"internal_solid_infill_speed", "250,230"},
        {"machine_start_gcode", "G28 ; home\nM104 S[nozzle_temperature_0]"},
        {"timelapse_type", "0"},
        {"filament_flow_ratio", "0.98,1,0.95"},
        {"max_layer_height", "0.28,0.32"},
    };
    for (auto &p : kv)
        c.set_deserialize_strict(p.first, p.second);
    return c;
}

static std::vector<Var> synthetic_ppv()
{
    return {
        {"i0", "i", "0"},
        {"i1", "i", "1"},
        {"i2", "i", "2"},
        {"i7", "i", "7"},
        {"ineg", "i", "-3"},
        {"ibig", "i", "2147483647"},
        {"imin", "i", "-2147483648"},
        {"d0", "f", "0"},
        {"d1", "f", "1.5"},
        {"dpi", "f", "3.14159265358979"},
        {"dneg", "f", "-2.75"},
        {"dsmall", "f", "1e-07"},
        {"dlarge", "f", "123456789"},
        {"dhalf", "f", "2.5"},
        {"s0", "s", ""},
        {"s1", "s", "abc"},
        {"s10", "s", "10"},
        {"snl", "s", "line1\\nline2"},
        {"sutf", "s", "\xc3\xa9t\xc3\xa9"},
        {"btrue", "b", "1"},
        {"bfalse", "b", "0"},
        {"pc", "p", "35"},
        {"fop", "fp", "0.5"},
        {"fopp", "fp", "50%"},
        {"pt", "pt", "12.5x-3"},
        {"vi", "is", "4,5,6"},
        {"vin", "isn", "4,nil,6"},
        {"vi_empty", "is", ""},
        {"vf", "fs", "0.1,0.25,1e+20"},
        {"vf1", "fs", "7.5"},
        {"vfn", "fsn", "1.5,nil"},
        {"vs", "ss", "a;\"b c\";\"\""},
        {"vs_empty", "ss", ""},
        {"vb", "bs", "1,0,1"},
        {"vp", "ps", "10,20"},
        {"vfp", "fps", "1,2%"},
        {"vpts", "pts", "1x2,3.5x-4"},
        {"idx1", "i", "1"},
        {"idx5", "i", "5"},
        {"idxneg", "i", "-1"},
        {"idxs", "s", "1"},
        {"idxf", "f", "1"},
        {"a_", "i", "42"},
        {"vec_", "is", "9,8"},
        {"x_1", "i", "11"},
        {"under_score", "s", "u"},
        {"Upper", "i", "5"},
        {"elsifoo", "i", "3"},
        {"if_x", "i", "4"},
        {"notx", "i", "6"},
        {"orange", "s", "fruit"},
        {"andy", "s", "name"},
        {"truex", "i", "8"},
        {"internal_solid_infill_line_width", "f", "0.47"},
        {"skeleton_infill_line_width", "f", "0"},
        {"odd_line_width", "f", "0"},
        {"current_extruder", "i", "1"},
    };
}

static std::vector<Var> synthetic_ovr()
{
    return {
        {"i1", "i", "100"},
        {"ovonly", "s", "from override"},
        {"layer_height", "f", "0.12"},
        {"line_width", "f", "0"},
    };
}

// ---------------------------------------------------------------------------------------------
// The cases
// ---------------------------------------------------------------------------------------------

static const std::vector<std::string> &operand_set()
{
    static const std::vector<std::string> v = {
        "0", "1", "-2", "7", "2147483647", "0.0", "1.5", "-2.25", "1e-9", "3.0",
        "\"a\"", "\"10\"", "\"\"", "true", "false",
        "i0", "i7", "ineg", "ibig", "imin", "d0", "d1", "dneg", "dsmall", "s0", "s1", "s10", "btrue", "bfalse",
        "pc", "fop", "vi[1]", "vf[2]", "vs[0]", "vb[1]", "vp[0]", "pt", "vpts[1]",
        "filament_type[1]", "wall_loops", "layer_height", "nozzle_diameter[1]", "spiral_mode", "curr_bed_type",
        "filament_change(1)",
    };
    return v;
}

static void gen_operators()
{
    const std::vector<std::string> binops = {"+", "-", "*", "/", "%", "==", "!=", "<>", "<", ">", "<=", ">=",
                                             "and", "or", "&&", "||"};
    const std::vector<std::string> &ops = operand_set();
    for (const std::string &op : binops)
        for (const std::string &a : ops)
            for (const std::string &b : ops)
                run_case("syn", 'P', 0, false, "{(" + a + " " + op + " " + b + ")}");
    // Without the parentheses: the macro takes an additive expression only.
    for (const std::string &op : binops)
        for (const std::string &a : {"1", "d1", "s1", "btrue"})
            for (const std::string &b : {"2", "dneg", "s10", "bfalse"})
                run_case("syn", 'P', 0, false, std::string("{") + a + " " + op + " " + b + "}");
    // Unary operators and functions over every operand.
    for (const std::string &a : ops) {
        for (const std::string &f : {"-%", "+%", "!%", "not %", "not(%)", "int(%)", "round(%)", "floor(%)", "ceil(%)",
                                     "min(%, 3)", "max(%, 3)", "min(2.5, %)", "max(-1, %)", "digits(%, 6)", "zdigits(%, 6)",
                                     "digits(%, 8, 3)", "zdigits(%, 8, 3)", "digits(%, 0, 0)", "digits(7, %)", "zdigits(1.25, 5, %)",
                                     "(% ? 1 : 2)", "(true ? % : 0)", "(false ? 0 : %)", "%[0]", "[%]"}) {
            std::string t = f;
            t.replace(t.find('%'), 1, a);
            run_case("syn", 'P', 0, false, "{" + t + "}");
        }
    }
}

static void gen_handwritten()
{
    const std::vector<std::string> P = {
        // plain text, white space, braces
        "", " ", "   leading spaces", "\n\nleading newlines", "\t tab", "plain text", "text with ] and } inside",
        "trailing   ", "a\nb\r\nc", "{i7}", "{ i7 }", " { i7 } ", "x{i7}y{d1}z", "{s1}{s10}", "{snl}", "{sutf}",
        "\xc3\xa9t\xc3\xa9 {i7} \xe2\x82\xac", "{}", "{ }", "{{i7}}", "{i7", "{i7}}", "}", "{", "[", "]", "[]", "[ ]",
        // legacy expansion
        "[i7]", "[ i7 ]", "[d1]", "[s1]", "[snl]", "[btrue]", "[pc]", "[fop]", "[fopp]", "[pt]", "[curr_bed_type]",
        "[vi]", "[vi_0]", "[vi_1]", "[vi_2]", "[vi_3]", "[vi_99]", "[vi_]", "[vi_x]", "[vi_1x]", "[vi_-1]",
        "[vf]", "[vf_2]", "[vfn_1]", "[vin_1]", "[vs]", "[vs_1]", "[vs_2]", "[vb_1]", "[vp_1]", "[vfp_1]", "[vpts_1]",
        "[vi_empty]", "[vi_empty_0]", "[i7_1]", "[nonexistent]", "[nonexistent_1]", "[no_such_var_2]",
        "[vi[idx1]]", "[vi[idx5]]", "[vi[idxneg]]", "[vi[idxs]]", "[vi[idxf]]", "[vi[nonexistent]]", "[vi [ idx1 ] ]",
        "[vec_[idx1]]", "[i7[idx1]]", "[vi_empty[idx1]]", "[vi[1]]", "[vi idx1]", "[vi[idx1]",
        "[vi[idx1] ]", "[1]", "[if]", "[a_]", "[x_1]", "[Upper]", "[nozzle_temperature_0]", "[nozzle_temperature_2]",
        "[nozzle_temperature]", "[filament_type]", "[filament_type_2]", "[filament_settings_id_1]", "[spiral_mode]",
        "[nozzle_diameter_1]", "[layer_height]", "[ovonly]", "[i1]", "[SLIC3R_PP_TEST]", "[version]",
        // scalar and vector references in expressions
        "{vi[0]}", "{vi[1]}", "{vi[2]}", "{vi[3]}", "{vi[-1]}", "{vi[i2]}", "{vi[i1+1]}", "{vi[ 1 ]}", "{vi[1.5]}",
        "{vi[\"1\"]}", "{vi[btrue]}", "{vi[]}", "{vi[1}", "{vi}", "{vf}", "{vf1}", "{vfn}", "{vfn[1]}", "{vin[1]}",
        "{vs}", "{vs[1]}", "{vs[2]}", "{vb}", "{vb[0]}", "{vp}", "{vp[1]}", "{vfp[1]}", "{vpts[0]}",
        "{vi_empty[0]}", "{vs_empty[0]}", "{i7[0]}", "{s1[0]}", "{pt}", "{pc}", "{fop}", "{curr_bed_type}",
        "{wall_generator}", "{print_sequence}", "{timelapse_type}", "{z_hop_types[1]}", "{z_hop_types}",
        "{nozzle_diameter}", "{nozzle_diameter[0]}", "{nozzle_diameter[1]}", "{nozzle_temperature}",
        "{nozzle_temperature[2]}", "{nozzle_temperature_initial_layer[2]}", "{filament_flow_ratio}",
        "{filament_flow_ratio[2]}", "{retraction_length}", "{max_layer_height}", "{bed_exclude_area}",
        "{bed_exclude_area[2]}", "{printable_area[1]}", "{extruder_offset}", "{extruder_offset[0]}",
        "{filament_map}", "{filament_map[1]}", "{wipe[1]}", "{wipe}", "{spiral_mode}", "{enable_prime_tower}",
        "{printer_notes}", "{printer_model}", "{ovonly}", "{i1}", "{layer_height}", "{nonexistent}", "{Nonexistent + 1}",
        "{SLIC3R_PP_TEST}", "{version}",
        // line widths, percentages and their ratio_over chains
        "{line_width}", "{outer_wall_line_width}", "{inner_wall_line_width}", "{sparse_infill_line_width}",
        "{top_surface_line_width}", "{support_line_width}", "{initial_layer_line_width}",
        "{internal_solid_infill_line_width}", "{skeleton_infill_line_width}", "{odd_line_width}",
        "{sparse_infill_density}", "{sparse_infill_anchor}", "{sparse_infill_anchor_max}", "{seam_slope_start_height}",
        "{seam_slope_gap}", "{spiral_mode_max_xy_smoothing}", "{filament_scarf_height}", "{filament_scarf_height[0]}",
        "{filament_scarf_gap}", "{vertical_shell_speed[1]}",
        // numbers
        "{1}", "{007}", "{0}", "{-0}", "{2147483647}", "{2147483648}", "{-2147483648}", "{-2147483647 - 1}",
        "{99999999999}", "{1.}", "{.5}", "{1.5}", "{1e5}", "{1E5}", "{1e+5}", "{1e-5}", "{1.5e3}", "{1e}", "{1e+}",
        "{5e}", "{1.e2}", "{.5e1}", "{.}", "{1x}", "{0x10}", "{1.2.3}", "{1e309}", "{1e308}", "{1e-320}", "{4.9e-324}",
        "{1e-400}", "{123456789012345678901234567890}", "{123456789012345678901234567890.5}",
        "{0.1234567890123456789012345}", "{00000000000000000001234.5}", "{000000000000000000012345}",
        "{12345678901234567.5}", "{1234567890123456789.25}", "{0.30000000000000004}", "{1e-307}", "{1e-308}",
        "{2.5e-310}", "{18446744073709551615.5}", "{18446744073709551616.5}", "{0.000000000000000000001}",
        "{1e2147483647}", "{1e2147483648}", "{1/3}", "{1.0/3}", "{2/3.0*3}", "{10%3}", "{-10%3}", "{10.5%3}",
        "{-7/2}", "{7/-2}", "{1/0}", "{1%0}", "{1.0/0}", "{1/0.0}", "{1%0.0}", "{imin/-1}", "{imin%-1}",
        "{ibig+1}", "{ibig*2}", "{-imin}", "{0.1+0.2}", "{1e20*1e20}", "{1e300*1e300}", "{-1e300*1e300}",
        "{-(0.0)}", "{-d0}", "{dsmall}", "{dlarge}", "{dlarge*10}", "{1/7.0}", "{100000.0}", "{1000000.0}",
        "{999999.5}", "{0.0001}", "{0.00001}", "{123456.7}",
        // strings
        "{\"\"}", "{\"abc\"}", "{\"a\\\"b\"}", "{\"a\\\\b\"}", "{\"a\\nb\"}", "{\"{inner}\"}", "{\"[inner]\"}",
        "{\"abc}", "{\"abc\\", "{\"ab\" + \"cd\"}", "{\"ab\" + 1}", "{1 + \"ab\"}", "{\"ab\" + 1.5}", "{\"x\" + true}",
        "{\"x\" + vi[0] + vf[2]}", "{\"x\" - 1}", "{\"x\" * 2}", "{\"\xc3\xa9\"}", "{ \"spaced\" }",
        // if / elsif / else
        "{if true}yes{endif}", "{if false}yes{endif}", "{if true}yes{else}no{endif}", "{if false}yes{else}no{endif}",
        "{if i7 == 7}seven{elsif i7 == 8}eight{else}other{endif}", "{if i7 == 8}eight{elsif i7 == 7}seven{endif}",
        "{if i7 > 1}a{elsif i7 > 2}b{else}c{endif}", "{if false}a{elsif false}b{elsif true}c{else}d{endif}",
        "{if false}a{elsif false}b{endif}", "{if true}\n  G28\n{endif}\n", "  {if true} x {endif}  ",
        "{if i7}yes{endif}", "{if s1}yes{endif}", "{if 1}yes{endif}", "{if}yes{endif}", "{if true yes{endif}",
        "{if true}yes", "{if true}yes{", "{if true}yes{else}", "{if true}yes{else}no", "{if true}yes{endif",
        "{if true}yes{end}", "{if true}yes{endif }", "{ if true }yes{ endif }", "{if true}{if false}a{else}b{endif}{endif}",
        "{if true}a{else}b{else}c{endif}", "{if true}a{elsif}b{endif}", "{if true}a{elsif 1}b{endif}",
        "{if true}a{else}{nonexistent}{endif}", "{if false}{nonexistent}{else}b{endif}",
        "{if true}a{elsif nonexistent}b{endif}", "{if true}a{elsif 1/0 == 1}b{endif}", "{else}", "{endif}",
        "{elsif true}", "a{else}b", "{if true}[i7]{else}[nonexistent]{endif}", "{if true}{ }{endif}",
        "{if (i7 == 7) and (d1 > 1)}both{endif}", "{if i7 == 7 and d1 > 1}both{endif}", "{if not bfalse}nb{endif}",
        "{if !bfalse}nb{endif}", "{if filament_type[0] == \"PLA\"}pla{endif}", "{if filament_type[1] != \"PLA\"}np{endif}",
        "{if curr_bed_type == \"Textured PEI Plate\"}pei{endif}", "{if curr_bed_type==\"Cool Plate\"}cool{endif}",
        "{if spiral_mode}vase{endif}", "{if wipe[0]}w{endif}", "{if vi[1] == 5}v{endif}",
        "{if printer_notes =~ /.*X1.*/}x1{endif}", "{if s1 =~ /a.c/}m{endif}", "{if s1 !~ /a.c/}m{endif}",
        "line1\n{if true}line2\n{endif}line3\n{nonexistent}\nline5", "a\n\nb\n{1 +}\nc",
        // ternary
        "{(true ? 1 : 2)}", "{(false ? 1 : 2)}", "{(i7 > 3 ? \"big\" : \"small\")}", "{(true ? 1 : 1/0)}",
        "{(1 ? 1 : 2)}", "{(true ? 1)}", "{(true ? : 2)}", "{(true ? 1 : )}", "{(true ? false ? 1 : 2 : 3)}",
        "{(false ? 1 : true ? 2 : 3)}", "{true ? 1 : 2}",
        // logic
        "{(true and false)}", "{(true or false)}", "{(true && true)}", "{(false || false)}", "{(not true)}",
        "{(!true)}", "{(! true)}", "{(notx)}", "{(orange)}", "{(andy)}", "{(truex)}", "{(elsifoo)}", "{(if_x)}",
        "{(true and)}", "{(true or)}", "{(1 and true)}", "{(true or 1)}", "{(false and (1/0 == 1))}",
        "{(true andfalse)}", "{(true&&false)}", "{(true||false)}", "{(!= 1)}", "{(1 = 1)}", "{(1 == )}",
        "{(1 < )}", "{(1 <> 2)}", "{(1 << 2)}", "{(true == true)}", "{(true != false)}", "{(true < false)}",
        "{(true == 1)}", "{(\"a\" < \"b\")}", "{(\"b\" > \"a\")}", "{(\"10\" < \"9\")}", "{(10 < \"9\")}",
        "{(1.5 == \"1.5\")}", "{(1 == 1.000000001)}", "{(1 == 1.0000001)}", "{(0.1 + 0.2 == 0.3)}",
        // regex
        "{(s1 =~ /abc/)}", "{(s1 =~ /ab/)}", "{(s1 =~ /a.*/)}", "{(s1 =~ /^abc$/)}", "{(s1 =~ /[a-c]+/)}",
        "{(s1 =~ /[^x]{3}/)}", "{(s1 =~ /a(b|x)c/)}", "{(s1 =~ /(a)(b)\\2?c/)}", "{(s1 !~ /abc/)}",
        "{(s1 =~ /ABC/)}", "{(s10 =~ /\\d+/)}", "{(s10 =~ /\\d{2}/)}", "{(s10 =~ /\\d{3,}/)}", "{(s1 =~ /\\w+/)}",
        "{(snl =~ /.*/)}", "{(s0 =~ //)}", "{(s0 =~ /a?/)}", "{(s1 =~ /a\\/b/)}", "{(s1 =~ / abc/)}",
        "{(s1 =~ /(abc/)}", "{(s1 =~ /abc)/)}", "{(s1 =~ /[abc/)}", "{(s1 =~ /*abc/)}", "{(s1 =~ /a**/)}",
        "{(s1 =~ /a{2,1}/)}", "{(s1 =~ /a\\/)}", "{(s1 =~ /abc)}", "{(s1 =~ abc)}", "{(s1 =~ )}", "{(i7 =~ /7/)}",
        "{(d1 =~ /1.5/)}", "{(\"PRINTER_VENDOR_BBL\" =~ /.*BBL.*/)}", "{(s1 =~ /a|/)}", "{(s1 =~ /(?:a)bc/)}",
        "{(s1 =~ /a(?=b)bc/)}", "{(s1 =~ /a(?!x)bc/)}", "{(s1 =~ /a.+?c/)}", "{(s1 =~ /a*+abc/)}",
        "{(s1 =~ /[[:alpha:]]+/)}", "{(s1 =~ /\\Aabc\\z/)}", "{(s1 =~ /\\babc\\b/)}",
        // functions
        "{min(1, 2)}", "{max(1, 2)}", "{min(1.5, 2)}", "{max(-1, -1.5)}", "{min(\"a\", 1)}", "{min(1)}", "{min 1, 2}",
        "{min(1, 2}", "{min(1 2)}", "{min()}", "{max(ibig, imin)}", "{digits(5, 3)}", "{zdigits(5, 3)}",
        "{digits(-5, 4)}", "{zdigits(-5, 4)}", "{digits(3.14159, 7, 2)}", "{zdigits(3.14159, 7, 2)}",
        "{zdigits(-3.14159, 8, 3)}", "{digits(2.5, 3)}", "{digits(3.5, 3)}", "{digits(-2.5, 3)}", "{digits(1, 100)}",
        "{digits(1, -5)}", "{digits(1.5, 4, 100)}", "{digits(1.5, 4, -2)}", "{digits(1, 2.0)}", "{digits(1, 2, 1.0)}",
        "{digits(\"a\", 2)}", "{digits(1, 2, 3, 4)}", "{digits(1, 2 3)}", "{digits(1, \"2\")}", "{digits(1)}",
        "{digits(dpi, 0, 10)}", "{int(3.7)}", "{int(-3.7)}", "{int(3)}", "{int(\"3\")}",
        "{int(1e10)}", "{int(-1e10)}", "{round(2.5)}", "{round(-2.5)}", "{round(2.4999)}", "{round(1e10)}",
        "{floor(-0.5)}", "{ceil(-0.5)}", "{floor(2)}", "{ceil(2.000001)}", "{floor(true)}", "{ceil(\"a\")}",
        "{round(true)}", "{int}", "{int(}", "{int(1}", "{round 1}", "{random(1, 2)}", "{filament_change(2)}",
        "{filament_change(1)}{nozzle_diameter}", "{filament_change(2)}{nozzle_diameter}", "{filament_change(1.7)}[vi]",
        "{filament_change(2)}[nozzle_temperature]", "{filament_change(0)}{line_width}", "{filament_change()}",
        "{filament_change(1, 2)}",
        // parse errors at several lines and columns
        "line one\nline two {1 +} rest\nline three", "{1 + }", "{1 +    }", "{1 + \n}", "{(1}", "{(1 + 2}", "{(}",
        "{1 2}", "{-}", "{+}", "{!}", "{not}", "{*1}", "{1 * }", "{1 / }", "{1 % }", "{1 - }", "{ 1 +* 2}", "{vi[}",
        "{vi[1 2]}", "{i7 i7}", "{\"a\" \"b\"}", "{true false}", "{if 1 == 1}", "x\ny\nz{", "\xc3\xa9\xc3\xa9{1 +}",
        "[\xc3\xa9t\xc3\xa9]", "{\xc3\xa9t\xc3\xa9}", "{ab\xc3\xa9}", "{\xc2\xa0i7}", "{i7\xc2\xa0}",
    };
    for (const std::string &t : P)
        run_case("syn", 'P', 0, false, t);
    // The same with the override configuration and other current extruders.
    for (const std::string &t : P) {
        run_case("syn", 'P', 1, true, t);
        run_case("syn", 'P', 2, false, t);
    }
    // random() with and without a generator.
    for (const std::string &t : {"{random(1, 6)}", "{random(1, 6)} {random(1, 6)} {random(1, 6)}", "{random(0.0, 1.0)}",
                                 "{random(-5, 5.5)}", "{random(3, 3)}", "{random(5, 1)}", "{random(imin, ibig)}",
                                 "{random(0, 1000000)}", "{random(\"a\", 1)}", "{random(1)}", "{random(1.5, 1.5)}",
                                 "{random(1, 6) + random(1, 6) * 10}", "{digits(random(0.0, 100.0), 8, 4)}"}) {
        run_case("syn", 'R', 0, false, t);
        run_case("syn", 'P', 0, false, t);
    }
}

// Characters outside ASCII in the positions where the grammar classifies characters (alpha, alnum and
// the space skipper of iso8859_1).
static std::string utf8(unsigned int cp)
{
    std::string o;
    if (cp < 0x80) {
        o += char(cp);
    } else if (cp < 0x800) {
        o += char(0xc0 | (cp >> 6));
        o += char(0x80 | (cp & 0x3f));
    } else if (cp < 0x10000) {
        o += char(0xe0 | (cp >> 12));
        o += char(0x80 | ((cp >> 6) & 0x3f));
        o += char(0x80 | (cp & 0x3f));
    } else {
        o += char(0xf0 | (cp >> 18));
        o += char(0x80 | ((cp >> 12) & 0x3f));
        o += char(0x80 | ((cp >> 6) & 0x3f));
        o += char(0x80 | (cp & 0x3f));
    }
    return o;
}

static void gen_bytes()
{
    std::vector<unsigned int> cps;
    for (unsigned int cp = 0x80; cp < 0x800; ++cp)
        cps.push_back(cp);
    for (unsigned int cp = 0x800; cp < 0x10000; cp += 0x3f)
        if (cp < 0xd800 || cp > 0xdfff)
            cps.push_back(cp);
    for (unsigned int cp = 0x10000; cp < 0x110000; cp += 0x3fff)
        cps.push_back(cp);
    for (unsigned int cp : cps) {
        std::string c = utf8(cp);
        run_case("syn", 'P', 0, false, "{" + c + "}");
        run_case("syn", 'P', 0, false, "{i7" + c + "}");
        run_case("syn", 'P', 0, false, "[" + c + "]");
    }
}

// Regular expressions: patterns over subjects through `=~` and `!~`.
static void gen_regex()
{
    const std::vector<std::string> subjects = {"", "abc", "aaa", "ABC", "a1b2", "PRINTER_VENDOR_BBL\nPRINTER_MODEL_X1",
                                               "line1\nline2", "a.b", "a/b", "tab\there", "x y", "abcabc", "aab"};
    const std::vector<std::string> patterns = {
        "abc", "a.c", ".*", ".+", "a*", "a+", "a?", "a{2}", "a{2,}", "a{1,2}", "a{ 1 , 2 }", "a{,2}", "a{x}", "a{2",
        "a{}", "[a-c]+", "[^a-c]+", "[abc]{3}", "[-a]+", "[a-]+", "[]a]+", "[^]a]+", "[a\\]]+", "[\\d]+", "[\\w.]+",
        "[[:alpha:]]+", "[[:digit:][:alpha:]]+", "[[:upper:]]+", "[[:space:]]", "(a|b|c)+", "a(bc)?", "(?:ab)+c?",
        "^abc$", "line1$\nline2", "line1\n^line2", ".*\\bline2\\b.*", "\\Aabc\\z", "\\d+", "\\D+", "\\w+",
        "\\W+", "\\s+", "\\S+", "a\\.b", "a\\/b", "\\x41BC", "tab\\there", "(a)\\1+", "(a+)b\\1", "a*?b", "a+?",
        "a*+a", "a*+", "(?=a)abc", "(?!b)abc", "a|", "|a", "()", "(a*)*", "(a|ab)(c|bcd)", ".*PRINTER_VENDOR_BBL.*",
        ".*X1.*", "PRINTER.*", "\\n", ".", "..", "x\\sy", "[a-c]{2}[a-c]", "a{0}abc", "(abc){2}", "(a|aa)+b",
        "[^\\n]*\\n.*", "\\.", "a\\Z", "[.]", "[$]", "a\\$", "\\(", "[(]", "x?y?z?", "", "a{1,1}?", "(?:)",
        // errors
        "(", "(abc", ")", "abc)", "a)b", "[", "[abc", "[^", "[a-", "[z-a]", "*a", "+a", "?a", "a**", "(*a)", "a|*b",
        "^*", "a{2,1}", "a{3,2}b", "\\", "abc\\", "[[:foo:]]", "(?", "\\1", "(a)\\2", "a{1}{2}", "[\\", "\\xZZ",
        "a??b", "a++b", "(a))", "((a)", "[a", "a[b-a]c", "0123456789abcdef(", "0123456789abcdef0123456789(",
    };
    for (const std::string &pat : patterns)
        for (const std::string &subj : subjects)
            run_case("syn", 'P', 0, false, "{(\"" + subj + "\" =~ /" + pat + "/)}");
    for (const std::string &pat : {"abc", "a.c", "(", "[a-c]+", "\\d"})
        for (const std::string &subj : subjects)
            run_case("syn", 'P', 0, false, "{(\"" + subj + "\" !~ /" + pat + "/)}");
}

static void gen_boolean()
{
    const std::vector<std::string> B = {
        "true", "false", "", " ", "1", "i7", "layer_height", "layer_height == 0.2", "layer_height==0.12",
        "nozzle_diameter[0]==0.4", "nozzle_diameter[0] == 0.4 and nozzle_diameter[1] == 0.6",
        "nozzle_diameter[0]==0.6", "printer_notes=~/.*PRINTER_VENDOR_BBL.*/", "printer_notes=~/.*X1.*/",
        "printer_notes!~/.*X1.*/", "printer_notes=~/PRINTER_VENDOR_BBL/", "printer_model==\"Bambu Lab X1 Carbon\"",
        "printer_model == \"Bambu Lab X1 Carbon\" or printer_model == \"Bambu Lab P1S\"",
        "wall_loops > 2 && spiral_mode", "not spiral_mode", "!enable_prime_tower", "(wall_loops >= 3) ? true : false",
        "wall_loops ? true : false", "curr_bed_type==\"Textured PEI Plate\"", "filament_type[0]==\"PLA\"",
        "filament_type[2]==\"TPU\" || filament_type[2]==\"PETG\"", "layer_height", "\"true\"", "true false", "true }",
        "{true}", "[layer_height]", "nonexistent", "nonexistent == 1", "layer_height ==", "(true", "true)",
        "1 + 1 == 2", "1 + 1", "min(layer_height, 0.1) < 0.15", "ovonly == \"from override\"", "i1 == 100", "i1 == 1",
        "layer_height == 0.12", "line_width == 0", "spiral_mode == 1", "  true  ", "\ttrue\n", "\n\nfalse",
        "true\nand false", "true and\n\n  nonexistent", "filament_map[1] == 2", "random(1, 2) == 1",
        "filament_change(1) == \"\"", "digits(1, 2) == \" 1\"", "sparse_infill_density == 15",
        "sparse_infill_anchor > 1",
    };
    for (const std::string &t : B) {
        run_case("syn", 'B', 0, false, t);
        run_case("syn", 'B', 0, true, t);
    }
    const std::vector<std::string> &ops = operand_set();
    for (const std::string &a : ops)
        for (const std::string &op : {"==", "!=", "<", ">=", "and", "or"})
            for (const std::string &b : {"1", "1.5", "\"a\"", "true", "i7", "d1", "s1", "btrue"})
                run_case("syn", 'B', 0, false, a + " " + op + " " + b);
}

static void gen_profiles(const std::string &bbl)
{
    std::vector<std::string> templates = profile_templates(bbl);
    for (const std::string &t : templates) {
        run_case("x1c", 'P', 0, false, t);
        run_case("x1c", 'P', 0, true, t);
        run_case("h2d", 'P', 0, true, t);
        run_case("h2d", 'P', 1, true, t);
    }
}

// ---------------------------------------------------------------------------------------------
// The PlaceholderParser methods
// ---------------------------------------------------------------------------------------------

static void dump_config(const char *name, const DynamicConfig &c)
{
    printf("api %s %zu", name, c.keys().size());
    for (const std::string &k : c.keys()) {
        // The clock: written by its type only.
        if (k == "timestamp" || k == "year" || k == "month" || k == "day" || k == "hour" || k == "minute" || k == "second") {
            printf(" %s %d CLOCK", hex(k).c_str(), int(c.option(k)->type()));
            continue;
        }
        printf(" %s %d %s", hex(k).c_str(), int(c.option(k)->type()), hex(c.option(k)->serialize()).c_str());
    }
    printf("\n");
}

static void dump_keys(const char *name, const std::vector<std::string> &keys)
{
    printf("api %s %zu", name, keys.size());
    for (const std::string &k : keys)
        printf(" %s", hex(k).c_str());
    printf("\n");
}

static void gen_api()
{
    PlaceholderParser pp;
    dump_config("ctor", pp.config());
    printf("api external %d\n", pp.external_config() == nullptr ? 0 : 1);
    pp.clear_config();
    dump_config("clear", pp.config());
    DynamicPrintConfig a;
    a.set_deserialize_strict("layer_height", "0.2");
    a.set_deserialize_strict("wall_loops", "3");
    a.set_deserialize_strict("filament_type", "PLA;PETG");
    a.set_deserialize_strict("nozzle_diameter", "0.4,0.6");
    dump_keys("diff1", pp.config_diff(a));
    printf("api apply1 %d\n", pp.apply_config(a) ? 1 : 0);
    dump_config("after_apply1", pp.config());
    dump_keys("diff2", pp.config_diff(a));
    printf("api apply2 %d\n", pp.apply_config(a) ? 1 : 0);
    DynamicPrintConfig b = a;
    b.set_deserialize_strict("layer_height", "0.3");
    b.set_deserialize_strict("wall_loops", "4");
    b.set_deserialize_strict("top_shell_layers", "5");
    dump_keys("diff3", pp.config_diff(b));
    pp.apply_only(b, {"layer_height", "top_shell_layers"});
    dump_config("after_apply_only", pp.config());
    pp.set("s", std::string("str"));
    pp.set("i", 5);
    pp.set("u", (unsigned int)7);
    pp.set("bo", true);
    pp.set("d", 2.5);
    pp.set("v", std::vector<std::string>{"x", "y z"});
    pp.set("o", new ConfigOptionPercent(12.5));
    dump_config("after_set", pp.config());
    DynamicPrintConfig c;
    c.set_deserialize_strict("z_hop", "0.4,0.6");
    c.set_deserialize_strict("wall_loops", "9");
    pp.apply_config(std::move(c));
    dump_config("after_apply_move", pp.config());
    printf("api option_i %s\n", pp.option("i") ? hex(pp.option("i")->serialize()).c_str() : "null");
    printf("api option_nope %s\n", pp.option("nope") ? hex(pp.option("nope")->serialize()).c_str() : "null");
    pp.update_timestamp();
    dump_config("after_timestamp", pp.config());
    const ConfigOptionString *ts = pp.config().option<ConfigOptionString>("timestamp");
    printf("api timestamp_len %zu\n", ts ? ts->value.size() : 0);
    DynamicConfig tc;
    PlaceholderParser::update_timestamp(tc);
    dump_config("static_timestamp", tc);
    printf("api env %s\n", pp.option("SLIC3R_PP_TEST") ? hex(pp.option("SLIC3R_PP_TEST")->serialize()).c_str() : "null");
    PlaceholderParser pp2;
    printf("api env2 %s\n", pp2.option("SLIC3R_PP_TEST") ? hex(pp2.option("SLIC3R_PP_TEST")->serialize()).c_str() : "null");
    printf("api env_empty %s\n", pp2.option("SLIC3R_PP_EMPTY") ? hex(pp2.option("SLIC3R_PP_EMPTY")->serialize()).c_str() : "null");
}

int main(int argc, char **argv)
{
    if (argc < 2) {
        fprintf(stderr, "usage: gen <references/BambuStudio/resources/profiles/BBL>\n");
        return 1;
    }
    std::string bbl = argv[1];
    // apply_env_variables(): "SLIC3R_" variables, the value up to the first white space.
    setenv("SLIC3R_PP_TEST", "  hello world", 1);
    setenv("SLIC3R_PP_EMPTY", "", 1);

    define_env("syn", synthetic_config(), synthetic_ppv(), synthetic_ovr());
    define_env("x1c", profile_config(bbl, "Bambu Lab X1 Carbon 0.4 nozzle", "0.20mm Standard @BBL X1C", "Bambu PLA Basic @BBL X1C"),
               gcode_placeholders(false), gcode_override(false));
    define_env("h2d", profile_config(bbl, "Bambu Lab H2D 0.4 nozzle", "0.20mm Standard @BBL H2D", "Bambu PLA Basic @BBL H2D"),
               gcode_placeholders(true), gcode_override(true));

    gen_api();
    gen_handwritten();
    gen_boolean();
    gen_bytes();
    gen_regex();
    gen_operators();
    gen_profiles(bbl);
    return 0;
}
