// Generator of the vectors of tests/preset_bundle_native_parity.rs: libslic3r's PresetBundle.hpp /
// PresetBundle.cpp at the pin (references/BambuStudio/src/libslic3r; the override layer has no copy
// of them).
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
//   gzip -9 -n -c <output directory>/preset_bundle.txt > $DATA/preset_bundle.txt.gz
//
// It runs one scenario and writes preset_bundle.txt. The Rust test runs the same scenario on the
// port and has to print the same lines. The scenario works in <output directory>/scratch (an
// absolute path, removed first), which is also the current directory; data_dir() is scratch/data
// and resources_dir() scratch/resources. A small vendor bundle "Acme" (one printer model, a single
// and a two-extruder printer, processes, filaments, all with inherits) and user presets are written
// there, loaded with load_presets() and driven through the public PresetBundle API. The dumps write
// the scratch path as "<D>" and the times read from the clock (an updated_time above 1500000000) as
// "NOW". The vectors in this directory were made on macOS arm64 with Apple clang and libc++.
//
// Line format: tokens separated by one space. A text token is the hexadecimal of its bytes ("-"
// for the empty text), a boolean 0 or 1, an integer decimal, a list its length and the texts. The
// first token names the line:
//
//   cfg <tag> <config>                       a configuration: number of options, key and serialize()
//   p <tag> <summary>                        a preset: see push_preset()
//   coll <tag> ...                           a collection: size, defaults, selection, its presets
//   bundle <tag> ...                         filament_presets, AMS counts, flags of a PresetBundle
//   vendor <tag> ...                         a vendor profile and its printer models
//   app <tag> <section> ...                  a section of the AppConfig
//   file / gone <tag> <path> [<content>]     a file under the scratch directory after a step
//   <other>                                  the result of one call, named after the function

#include "libslic3r/Preset.hpp"
#include "libslic3r/PresetBundle.hpp"
#include "libslic3r/AppConfig.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/Utils.hpp"
#include "libslic3r/libslic3r.h"

#include <boost/filesystem.hpp>
#include <miniz/miniz.h>

#include <cstdio>
#include <cstring>
#include <fstream>
#include <functional>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <vector>

// Defined in PresetBundle.cpp with external linkage, not declared in PresetBundle.hpp.
namespace Slic3r {
bool find_filament_preset_map_name(std::string machine_name, std::string filament_old_name, std::string &filament_new_name);
}

using namespace Slic3r;
namespace bfs = boost::filesystem;

typedef std::vector<std::string> Toks;

static FILE       *g_out = nullptr;
static std::string D; // the scratch directory

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
static std::string b(bool v) { return v ? "1" : "0"; }
static void        emit(const Toks &l)
{
    for (size_t i = 0; i < l.size(); ++i) {
        if (i) fputc(' ', g_out);
        fputs(l[i].c_str(), g_out);
    }
    fputc('\n', g_out);
    fflush(g_out);
}
static void push_list(Toks &l, const std::vector<std::string> &v)
{
    l.push_back(num(v.size()));
    for (const std::string &s : v) l.push_back(hex(s));
}
static std::string rel(std::string s)
{
    for (size_t pos = s.find(D); !D.empty() && pos != std::string::npos; pos = s.find(D, pos)) s.replace(pos, D.size(), "<D>");
    return s;
}
static std::string norm_time(long long t) { return t > 1500000000LL ? std::string("NOW") : std::to_string(t); }
static std::string idx(size_t i) { return i == size_t(-1) ? std::string("-1") : std::to_string(i); }
static std::string dbl(double v)
{
    char buf[64];
    snprintf(buf, sizeof(buf), "%.17g", v);
    return buf;
}

static std::string ser(const ConfigOption *opt)
{
    try {
        return opt->serialize();
    } catch (const std::exception &e) {
        return std::string("!") + e.what();
    }
}
static void push_cfg(Toks &l, const ConfigBase &c)
{
    t_config_option_keys keys = c.keys();
    l.push_back(num(keys.size()));
    for (const std::string &k : keys) {
        l.push_back(hex(k));
        l.push_back(hex(rel(ser(c.option(k)))));
    }
}
// FNV-1a 64 of the keys and serialized values.
static std::string cfg_hash(const ConfigBase &c)
{
    uint64_t h   = 0xcbf29ce484222325ULL;
    auto     add = [&h](const std::string &s) {
        for (unsigned char ch : s) {
            h ^= ch;
            h *= 0x100000001b3ULL;
        }
    };
    for (const std::string &k : c.keys()) {
        add(k);
        add("\x01");
        add(rel(ser(c.option(k))));
        add("\x02");
    }
    char buf[32];
    snprintf(buf, sizeof(buf), "%016llx", (unsigned long long) h);
    return buf;
}
static std::string inherits_of(const Preset &p)
{
    auto *o = dynamic_cast<const ConfigOptionString *>(p.config.option("inherits"));
    return o ? o->value : std::string("<none>");
}
static void push_preset(Toks &l, const Preset &p)
{
    l.push_back(hex(p.name));
    l.push_back(hex(rel(p.file)));
    l.push_back(num((int) p.type));
    l.push_back(b(p.is_default));
    l.push_back(b(p.is_external));
    l.push_back(b(p.is_system));
    l.push_back(b(p.is_visible));
    l.push_back(b(p.is_dirty));
    l.push_back(b(p.is_compatible));
    l.push_back(b(p.is_project_embedded));
    l.push_back(b(p.loaded));
    l.push_back(hex(p.alias));
    l.push_back(hex(inherits_of(p)));
    l.push_back(hex(p.filament_id));
    l.push_back(hex(p.setting_id));
    l.push_back(hex(p.base_id));
    l.push_back(hex(p.user_id));
    l.push_back(hex(p.sync_info));
    l.push_back(hex(p.description));
    l.push_back(hex(p.version.to_string()));
    l.push_back(norm_time(p.updated_time));
    l.push_back(hex(p.vendor ? p.vendor->id : std::string()));
    push_list(l, p.renamed_from);
    l.push_back(num(p.config.keys().size()));
    l.push_back(cfg_hash(p.config));
}
static void dump_preset(const std::string &tag, const Preset &p)
{
    Toks l{"p", hex(tag)};
    push_preset(l, p);
    emit(l);
}
static void dump_cfg(const std::string &tag, const ConfigBase &c)
{
    Toks l{"cfg", hex(tag)};
    push_cfg(l, c);
    emit(l);
}
static void dump_coll(const std::string &tag, const PresetCollection &c)
{
    emit({"coll", hex(tag), num(c.size()), num(const_cast<PresetCollection &>(c).num_default_presets()), idx(c.get_selected_idx()),
          b(c.is_default_suppressed()), hex(rel(c.path_from_name("x", false))), num(c.num_visible()), b(c.has_defaults_only())});
    for (size_t i = 0; i < c.get_presets().size(); ++i) dump_preset(tag + "#" + std::to_string(i), c.get_presets()[i]);
    dump_preset(tag + "#edited", c.get_edited_preset());
}
static void dump_vendors(const std::string &tag, const VendorMap &vendors)
{
    for (const auto &kv : vendors) {
        const VendorProfile &v = kv.second;
        Toks l{"vendor", hex(tag), hex(kv.first), hex(v.name), hex(v.id), hex(v.config_version.to_string()), hex(v.config_update_url), num(v.models.size())};
        for (const auto &m : v.models) {
            l.push_back(hex(m.id));
            l.push_back(hex(m.name));
            l.push_back(hex(m.model_id));
            l.push_back(num((int) m.technology));
            l.push_back(hex(m.family));
            l.push_back(hex(m.bed_model));
            l.push_back(hex(m.bed_texture));
            l.push_back(num(m.variants.size()));
            for (const auto &var : m.variants) l.push_back(hex(var.name));
            push_list(l, m.default_materials);
        }
        emit(l);
    }
}
static void dump_bundle(const std::string &tag, PresetBundle &pb)
{
    Toks l{"bundle", hex(tag)};
    push_list(l, pb.filament_presets);
    push_list(l, save_extruder_ams_count_to_string(pb.extruder_ams_counts));
    l.push_back(num(pb.calibrate_filaments.size()));
    l.push_back(num(pb.incompatible_nozzle_diameter_slots.size()));
    for (size_t s : pb.incompatible_nozzle_diameter_slots) l.push_back(num(s));
    l.push_back(num(pb.ams_multi_color_filment.size()));
    for (const auto &v : pb.ams_multi_color_filment) push_list(l, v);
    l.push_back(num(pb.extruder_ams_selections.size()));
    for (const auto &s : pb.extruder_ams_selections) {
        l.push_back(b(s.initialized));
        l.push_back(hex(s.device_id));
        push_list(l, s.ams4);
        push_list(l, s.ams1);
    }
    push_list(l, pb.obsolete_presets.prints);
    push_list(l, pb.obsolete_presets.filaments);
    push_list(l, pb.obsolete_presets.printers);
    l.push_back(num(pb.ams_sync_substitutions.size()));
    for (const auto &s : pb.ams_sync_substitutions) {
        l.push_back(num(s.slot));
        l.push_back(hex(s.tray));
        l.push_back(hex(s.wanted));
        l.push_back(hex(s.applied));
    }
    l.push_back(b(pb.has_defauls_only()));
    emit(l);
    dump_coll(tag + ":prints", pb.prints);
    dump_coll(tag + ":filaments", pb.filaments);
    dump_coll(tag + ":printers", pb.printers);
    dump_cfg(tag + ":project", pb.project_config);
    dump_vendors(tag, pb.vendors);
}
static void dump_app(const std::string &tag, AppConfig &config)
{
    for (const char *section : {"presets", "filaments", "nozzle_volume_types", "nozzle_slots"}) {
        Toks l{"app", hex(tag), hex(section), b(config.has_section(section))};
        if (config.has_section(section)) {
            const auto &m = config.get_section(section);
            l.push_back(num(m.size()));
            for (const auto &kv : m) {
                l.push_back(hex(kv.first));
                l.push_back(hex(kv.second));
            }
        }
        emit(l);
    }
    emit({"app_keys", hex(tag), hex(config.get("flush_multiplier")), hex(config.get("flush_multiplier_fast")), hex(config.get("preset_folder"))});
}
static std::string norm_content(const std::string &s)
{
    std::string out;
    size_t      pos = 0;
    while (pos < s.size()) {
        size_t      e    = s.find('\n', pos);
        std::string line = s.substr(pos, e == std::string::npos ? std::string::npos : e - pos);
        if (line.rfind("updated_time = ", 0) == 0) {
            long long t = atoll(line.c_str() + 15);
            line        = "updated_time = " + norm_time(t);
        }
        out += line;
        if (e == std::string::npos) break;
        out += '\n';
        pos = e + 1;
    }
    return out;
}
static std::string read_file(const std::string &path)
{
    std::ifstream     f(path, std::ios::binary);
    std::stringstream ss;
    ss << f.rdbuf();
    return ss.str();
}
static void write_file(const std::string &path, const std::string &content)
{
    bfs::create_directories(bfs::path(path).parent_path());
    std::ofstream f(path, std::ios::binary | std::ios::trunc);
    f << content;
}
// The files under the scratch directory that changed since the last call ("file"), and those that
// went away ("gone").
static std::map<std::string, std::string> g_files;
static void dump_files(const std::string &tag)
{
    std::vector<std::string> files;
    if (bfs::exists(D))
        for (bfs::recursive_directory_iterator it(D), end; it != end; ++it)
            if (bfs::is_regular_file(it->path())) files.push_back(it->path().string());
    std::sort(files.begin(), files.end());
    emit({"files", hex(tag), num(files.size())});
    std::map<std::string, std::string> now;
    for (const std::string &f : files) {
        std::string r = rel(f), c = norm_content(read_file(f));
        now[r]        = c;
        auto it       = g_files.find(r);
        if (it == g_files.end() || it->second != c) emit({"file", hex(tag), hex(r), hex(rel(c))});
    }
    for (const auto &e : g_files)
        if (now.find(e.first) == now.end()) emit({"gone", hex(tag), hex(e.first)});
    g_files = now;
}
static void err(const std::string &what, const std::exception &e) { emit({what, hex(rel(std::string("!") + e.what()))}); }

static ConfigOptionStrings *strs_opt(const std::vector<std::string> &v) { return new ConfigOptionStrings(v); }
static DynamicPrintConfig   tray(const std::string &filament_id, const std::string &colour, const std::string &tray_name, const std::string &type,
                                 const std::string &ams_id, const std::string &slot_id, bool changed)
{
    DynamicPrintConfig c;
    c.set_key_value("filament_id", strs_opt({filament_id}));
    c.set_key_value("filament_colour", strs_opt({colour}));
    c.set_key_value("filament_colour_type", strs_opt({"1"}));
    c.set_key_value("tray_name", strs_opt({tray_name}));
    c.set_key_value("filament_type", strs_opt({type}));
    c.set_key_value("ams_id", strs_opt({ams_id}));
    c.set_key_value("slot_id", strs_opt({slot_id}));
    c.set_key_value("filament_multi_colour", strs_opt({colour}));
    c.set_key_value("filament_changed", new ConfigOptionBool(changed));
    return c;
}
static void dump_maps(const std::string &tag, const std::map<int, AMSMapInfo> &maps)
{
    Toks l{"maps", hex(tag), num(maps.size())};
    for (const auto &kv : maps) {
        l.push_back(num(kv.first));
        l.push_back(hex(kv.second.ams_id));
        l.push_back(hex(kv.second.slot_id));
        l.push_back(hex(kv.second.nozzle_diameter));
        l.push_back(kv.second.nozzle_volume_type ? num((int) *kv.second.nozzle_volume_type) : std::string("none"));
    }
    emit(l);
}

// ---------------------------------------------------------------------------------------------
// The vendor bundle and the user presets.

static const char *VERSION = "01.02.03.04";

static void write_vendor(const std::string &sys)
{
    write_file(sys + "/Acme.json", R"({
    "name": "Acme",
    "version": "01.02.03.04",
    "force_update": "0",
    "description": "Acme test vendor",
    "url": "https://example.invalid/acme",
    "machine_model_list": [{"name": "Acme X1", "sub_path": "machine/Acme X1.json"}, {"name": "Missing", "sub_path": "machine/none.json"}],
    "process_list": [{"name": "fdm_process_common", "sub_path": "process/fdm_process_common.json"},
                     {"name": "0.20mm Standard @Acme X1", "sub_path": "process/0.20mm Standard @Acme X1.json"},
                     {"name": "0.12mm Fine @Acme X1", "sub_path": "process/0.12mm Fine @Acme X1.json"}],
    "filament_list": [{"name": "fdm_filament_common", "sub_path": "filament/fdm_filament_common.json"},
                      {"name": "Acme PLA @Acme X1", "sub_path": "filament/Acme PLA @Acme X1.json"},
                      {"name": "Acme PETG @Acme X1", "sub_path": "filament/Acme PETG @Acme X1.json"},
                      {"name": "Acme TPU @Acme X1 0.4", "sub_path": "filament/Acme TPU @Acme X1 0.4.json"},
                      {"name": "Generic PLA @Acme X1", "sub_path": "filament/Generic PLA @Acme X1.json"}],
    "machine_list": [{"name": "fdm_machine_common", "sub_path": "machine/fdm_machine_common.json"},
                     {"name": "Acme X1 0.4 nozzle", "sub_path": "machine/Acme X1 0.4 nozzle.json"},
                     {"name": "Acme X1 0.6 nozzle", "sub_path": "machine/Acme X1 0.6 nozzle.json"},
                     {"name": "Acme X1 0.4 rack", "sub_path": "machine/Acme X1 0.4 rack.json"}]
})");
    const std::string A = sys + "/Acme";
    write_file(A + "/machine/Acme X1.json", R"({"type": "machine_model", "name": "Acme X1", "model_id": "AX1", "nozzle_diameter": "0.4;0.6",
        "machine_tech": "FFF", "family": "Acme", "bed_model": "acme_bed.stl", "bed_texture": "acme_tex.svg",
        "default_materials": "Acme PLA @Acme X1;Acme PETG @Acme X1;"})");
    write_file(A + "/machine/fdm_machine_common.json", R"({"type": "machine", "name": "fdm_machine_common", "from": "system", "instantiation": "false",
        "printer_technology": "FFF", "gcode_flavor": "marlin", "printable_height": "240"})");
    write_file(A + "/machine/Acme X1 0.4 nozzle.json", R"({"type": "machine", "name": "Acme X1 0.4 nozzle", "inherits": "fdm_machine_common",
        "from": "system", "setting_id": "AM001", "instantiation": "true", "printer_model": "Acme X1", "printer_variant": "0.4",
        "nozzle_diameter": ["0.4"], "default_print_profile": "0.20mm Standard @Acme X1", "default_filament_profile": ["Acme PLA @Acme X1"],
        "printable_area": ["0x0", "250x0", "250x250", "0x250"]})");
    write_file(A + "/machine/Acme X1 0.6 nozzle.json", R"({"type": "machine", "name": "Acme X1 0.6 nozzle", "inherits": "fdm_machine_common",
        "from": "system", "setting_id": "AM002", "instantiation": "true", "printer_model": "Acme X1", "printer_variant": "0.6",
        "nozzle_diameter": ["0.6", "0.6"], "extruder_type": ["Direct Drive", "Direct Drive"],
        "extruder_variant_list": ["Direct Drive Standard,Direct Drive High Flow", "Direct Drive Standard,Direct Drive High Flow"],
        "printer_extruder_id": ["1", "1", "2", "2"],
        "printer_extruder_variant": ["Direct Drive Standard", "Direct Drive High Flow", "Direct Drive Standard", "Direct Drive High Flow"],
        "retraction_length": ["0.8", "1", "0.9", "1.1"], "nozzle_volume": ["100", "110", "120", "130"],
        "extruder_printable_area": ["0x0,250x0,250x250,0x250", "10x0,260x0,260x250,10x250"],
        "default_print_profile": "0.20mm Standard @Acme X1", "default_filament_profile": ["Acme PLA @Acme X1", "Acme PETG @Acme X1"]})");
    write_file(A + "/machine/Acme X1 0.4 rack.json", R"({"type": "machine", "name": "Acme X1 0.4 rack", "inherits": "fdm_machine_common",
        "from": "system", "setting_id": "AM003", "instantiation": "true", "printer_model": "Acme X1", "printer_variant": "0.4",
        "nozzle_diameter": ["0.4"], "handle_hotend_as_extruder": "1", "extruder_max_nozzle_count": ["4"], "default_nozzle_volume_type": ["Standard"],
        "default_print_profile": "0.20mm Standard @Acme X1", "default_filament_profile": ["Acme PLA @Acme X1"]})");
    write_file(A + "/process/fdm_process_common.json", R"({"type": "process", "name": "fdm_process_common", "from": "system", "instantiation": "false",
        "layer_height": "0.2", "print_extruder_id": ["1", "1", "2", "2"],
        "print_extruder_variant": ["Direct Drive Standard", "Direct Drive High Flow", "Direct Drive Standard", "Direct Drive High Flow"],
        "outer_wall_speed": ["200", "210", "220", "230"], "description": "common process"})");
    write_file(A + "/process/0.20mm Standard @Acme X1.json", R"({"type": "process", "name": "0.20mm Standard @Acme X1", "inherits": "fdm_process_common",
        "from": "system", "setting_id": "AP001", "instantiation": "true", "wall_loops": "3",
        "compatible_printers": ["Acme X1 0.4 nozzle", "Acme X1 0.6 nozzle", "Acme X1 0.4 rack"]})");
    write_file(A + "/process/0.12mm Fine @Acme X1.json", R"({"type": "process", "name": "0.12mm Fine @Acme X1", "inherits": "fdm_process_common",
        "from": "system", "setting_id": "AP002", "instantiation": "true", "layer_height": "0.12",
        "compatible_printers": ["Acme X1 0.4 nozzle"]})");
    write_file(A + "/filament/fdm_filament_common.json", R"({"type": "filament", "name": "fdm_filament_common", "from": "system", "instantiation": "false",
        "filament_type": ["PLA"], "filament_extruder_variant": ["Direct Drive Standard", "Direct Drive High Flow"],
        "nozzle_temperature": ["210", "215"], "filament_flow_ratio": ["0.98", "0.97"]})");
    write_file(A + "/filament/Acme PLA @Acme X1.json", R"({"type": "filament", "name": "Acme PLA @Acme X1", "inherits": "fdm_filament_common",
        "from": "system", "setting_id": "AF001", "filament_id": "AP1", "instantiation": "true", "filament_vendor": ["Acme"],
        "required_nozzle_HRC": ["3"], "compatible_printers": ["Acme X1 0.4 nozzle", "Acme X1 0.6 nozzle", "Acme X1 0.4 rack"]})");
    write_file(A + "/filament/Acme PETG @Acme X1.json", R"({"type": "filament", "name": "Acme PETG @Acme X1", "inherits": "fdm_filament_common",
        "from": "system", "setting_id": "AF002", "filament_id": "AG1", "instantiation": "true", "filament_type": ["PETG"],
        "nozzle_temperature": ["240", "245"], "compatible_printers": ["Acme X1 0.4 nozzle", "Acme X1 0.6 nozzle", "Acme X1 0.4 rack"]})");
    write_file(A + "/filament/Acme TPU @Acme X1 0.4.json", R"({"type": "filament", "name": "Acme TPU @Acme X1 0.4", "inherits": "fdm_filament_common",
        "from": "system", "setting_id": "AF003", "filament_id": "AT1", "instantiation": "true", "filament_type": ["TPU"],
        "filament_extruder_variant": ["Direct Drive Standard"], "nozzle_temperature": ["220"], "filament_flow_ratio": ["1"],
        "compatible_printers": ["Acme X1 0.4 nozzle"]})");
    write_file(A + "/filament/Generic PLA @Acme X1.json", R"({"type": "filament", "name": "Generic PLA @Acme X1", "inherits": "fdm_filament_common",
        "from": "system", "setting_id": "AF004", "filament_id": "GP1", "instantiation": "true", "filament_vendor": ["Generic"],
        "compatible_printers": ["Acme X1 0.4 nozzle", "Acme X1 0.6 nozzle", "Acme X1 0.4 rack"]})");
}

static void write_user(const std::string &user)
{
    write_file(user + "/filament/My PLA.json", std::string(R"({"version": ")") + VERSION +
               R"(", "name": "My PLA", "from": "User", "inherits": "Acme PLA @Acme X1", "nozzle_temperature": ["225", "235"], "filament_settings_id": ["My PLA"]})");
    write_file(user + "/filament/My PLA.info", "sync_info = \nuser_id = \nsetting_id = US1\nbase_id = AF001\nupdated_time = 1000\n");
    write_file(user + "/process/My Process.json", std::string(R"({"version": ")") + VERSION +
               R"(", "name": "My Process", "from": "User", "inherits": "0.20mm Standard @Acme X1", "layer_height": "0.16", "print_settings_id": "My Process"})");
    write_file(user + "/machine/My Printer.json", std::string(R"({"version": ")") + VERSION +
               R"(", "name": "My Printer", "from": "User", "inherits": "Acme X1 0.4 nozzle", "printer_settings_id": "My Printer", "printable_height": "200"})");
}

// ---------------------------------------------------------------------------------------------

static void gen_free_functions()
{
    for (int t = 0; t <= 6; ++t) emit({"nvt_str", num(t), hex(GetNozzleVolumeTypeStr(NozzleVolumeType(t)))});
    {
        FilamentBaseInfo info;
        info.set_filament_extruder_compatibility(0x3a7);
        Toks l{"fbi", num(info.get_filament_extruder_compatibility())};
        for (int e = -1; e <= 11; ++e) l.push_back(num(info.get_extruder_compatibility(e)));
        emit(l);
    }
    {
        ExtruderNozzleStat s;
        s.set_extruder_nozzle_count(1, nvtHighFlow, 2, false);
        s.set_extruder_nozzle_count(1, nvtStandard, 3, false);
        Toks l{"ens"};
        for (int e = -1; e <= 2; ++e)
            for (int t : {-1, 0, 1, 2, 3}) l.push_back(num(s.get_extruder_nozzle_count(e, t < 0 ? std::nullopt : std::optional<NozzleVolumeType>(NozzleVolumeType(t)))));
        s.on_volume_type_switch(1, nvtHighFlow);
        s.on_volume_type_switch(3, nvtStandard);
        s.on_volume_type_switch(0, nvtHybrid);
        for (int e = 0; e <= 3; ++e) l.push_back(num(s.get_extruder_nozzle_count(e)));
        s.on_printer_model_change_cli({1}, {2, 4});
        for (int e = 0; e <= 2; ++e)
            for (int t : {0, 1}) l.push_back(num(s.get_extruder_nozzle_count(e, NozzleVolumeType(t))));
        emit(l);
        ExtruderFullStat full = build_extruder_full_stat_from_nozzle_stat(s, {3, 1}, "0.4");
        Toks l2{"full_stat", num(full.get_extruder_count())};
        for (int e = 0; e < 3; ++e) {
            l2.push_back(num(full.get_extruder_nozzle_count(e)));
            for (const auto &p : full.get_extruder_stats(e)) {
                l2.push_back(hex(p.first));
                l2.push_back(hex(p.second));
            }
        }
        emit(l2);
        DynamicPrintConfig c;
        emit({"per_slot_consistent", b(per_slot_nozzle_options_consistent(c))});
        sync_per_slot_nozzle_options_from_full_stat(c, full);
        dump_cfg("per_slot", c);
        emit({"per_slot_consistent", b(per_slot_nozzle_options_consistent(c))});
        c.option<ConfigOptionInts>("extruder_nozzle_count")->values = {1, -1};
        emit({"per_slot_consistent", b(per_slot_nozzle_options_consistent(c))});
    }
    {
        DynamicPrintConfig c;
        c.set_deserialize_strict("default_nozzle_volume_type", "Standard,High Flow");
        c.set_deserialize_strict("extruder_max_nozzle_count", "3,1");
        Toks l{"count_default_nvt"};
        for (int e = 0; e < 3; ++e)
            for (int n : {0, 1, 3}) {
                auto m = count_default_nozzle_volume_types(c, e, n);
                l.push_back(num(m.size()));
                for (const auto &kv : m) {
                    l.push_back(num((int) kv.first));
                    l.push_back(num(kv.second));
                }
            }
        emit(l);
    }
    {
        // filament_name_map.json is read once.
        for (auto [m, f] : std::vector<std::pair<std::string, std::string>>{{"Acme X1 0.6 nozzle", "Old PLA"}, {"Acme X1 0.6 nozzle", "Other"}, {"Nope", "Old PLA"}}) {
            std::string out = "unchanged";
            bool        r   = find_filament_preset_map_name(m, f, out);
            std::string f2  = f;
            convert_filament_preset_name(m, f2);
            emit({"filament_map_name", hex(m), hex(f), b(r), hex(out), hex(f2)});
        }
    }
}


static void gen_more(PresetBundle &pb)
{
    const std::string res = D + "/resources";

    // temperatures of an AMS tray against a user root filament
    try {
        write_file(D + "/import2/Imported Root2.json", std::string(R"({"version": ")") + VERSION +
                   R"(", "name": "Imported Root2", "from": "User", "inherits": "", "filament_id": "IR2", "filament_settings_id": ["Imported Root2"], "compatible_printers": ["Acme X1 0.6 nozzle"], "nozzle_temperature_range_low": ["190"], "nozzle_temperature_range_high": ["250"]})");
        std::vector<std::string> files{D + "/import2/Imported Root2.json"};
        auto subs = pb.import_presets(files, [](const std::string &) { return 1; }, ForwardCompatibilitySubstitutionRule::EnableSilent);
        Toks l{"import2", num(subs.size())};
        for (const auto &f : files) l.push_back(hex(rel(f)));
        emit(l);
    } catch (const std::exception &e) {
        err("import2", e);
    }
    for (auto [sid, tag, mn, mx] : std::vector<std::tuple<std::string, std::string, std::string, std::string>>{
             {"IR2", "1234", "190", "250"}, {"IR2", "1234", "180", "250"}, {"IR2", "0", "180", "250"}, {"IR2", "123456789012010A", "180", "260"},
             {"ZZ", "1234", "1", "2"}, {"IR2", "1234", "abc", "250"}}) {
        try {
            std::string nozzle = "0.6", psid;
            bool        r      = pb.check_filament_temp_equation_by_printer_type_and_nozzle_for_mas_tray("Acme X1", nozzle, sid, tag, mn, mx, psid);
            emit({"temp_equation", hex(sid), hex(tag), b(r), hex(mn), hex(mx), hex(psid)});
        } catch (const std::exception &e) {
            err("temp_equation", e);
        }
    }

    // the vendor bundle of the resources
    write_vendor(res + "/profiles");
    try {
        PresetBundle other;
        auto         r = other.load_system_models_from_json(ForwardCompatibilitySubstitutionRule::EnableSystemSilent);
        emit({"system_models", num(r.first.size()), hex(rel(r.second))});
        dump_vendors("system_models", other.vendors);
        dump_coll("system_models:printers", other.printers);
    } catch (const std::exception &e) {
        err("system_models", e);
    }
    try {
        PresetBundle other;
        auto         r = other.load_system_filaments_json(ForwardCompatibilitySubstitutionRule::EnableSilentDisableSystem);
        emit({"system_filaments", num(r.first.size()), hex(rel(r.second))});
        dump_vendors("system_filaments", other.vendors);
        dump_coll("system_filaments:filaments", other.filaments);
        dump_coll("system_filaments:prints", other.prints);
    } catch (const std::exception &e) {
        err("system_filaments", e);
    }

    // model files and vendor errors
    write_file(D + "/models/bad.json", "{ nope");
    write_file(D + "/models/num.json", R"({"name": 5})");
    write_file(D + "/models/good.json", R"({"name": "Good", "model_id": "G1", "nozzle_diameter": "0.4;0.4;0.8", "machine_tech": "SLA", "family": "F",
        "bed_model": "b.stl", "bed_texture": "t.svg", "hotend_model": "h.stl", "default_bed_type": "Cool Plate", "use_rect_grid": "true",
        "default_materials": "B;A;A;", "not_support_bed_type": ";X", "bottom_texture_rect_longer_ignore_list": "Z;Y", "support_side_panel_fan": "false",
        "resource_bind_name": "R", "version": "1", "url": "u"})");
    for (const char *f : {"none.json", "bad.json", "num.json", "good.json"}) {
        try {
            auto m = pb.load_vendor_configs_from_json(D + "/models/" + f);
            Toks l{"model", hex(f), hex(m.id), hex(m.name), hex(m.model_id), num((int) m.technology), hex(m.family), hex(m.bed_model), hex(m.bed_texture),
                   hex(m.hotend_model), hex(m.default_bed_type), hex(m.use_rect_grid), hex(m.support_side_panel_fan), hex(m.resource_bind_name), num(m.variants.size())};
            for (const auto &v : m.variants) l.push_back(hex(v.name));
            push_list(l, m.default_materials);
            push_list(l, m.not_support_bed_types);
            push_list(l, m.bottom_texture_rect_longer_ignore_list);
            emit(l);
        } catch (const std::exception &e) {
            err("model", e);
        }
    }
    write_file(D + "/badv/Bad.json", R"({"name": "Bad", "version": "x"})");
    write_file(D + "/badp/Badp.json", R"({"name": "Badp", "version": "1.0.0.0", "process_list": [{"name": "P", "sub_path": "p.json"}]})");
    write_file(D + "/badp/Badp/p.json", R"({"name": "P", "inherits": "Nope", "instantiation": "true"})");
    for (auto [dir, vendor] : std::vector<std::pair<std::string, std::string>>{{"none_dir", "Acme"}, {"badv", "Bad"}, {"badp", "Badp"}}) {
        try {
            PresetBundle other;
            auto         r = other.load_vendor_configs_from_json(D + "/" + dir, vendor, PresetBundle::LoadSystem, ForwardCompatibilitySubstitutionRule::Disable);
            emit({"vendor_load", hex(vendor), num(r.first.size()), num(r.second)});
        } catch (const std::exception &e) {
            err("vendor_load", e);
        }
    }

    // a preset bundle file
    try {
        const std::string zp = D + "/bundle.bbscfg";
        mz_zip_archive    zip;
        mz_zip_zero_struct(&zip);
        mz_zip_writer_init_file(&zip, zp.c_str(), 0);
        const std::string a = std::string(R"({"version": ")") + VERSION +
                              R"(", "name": "Zip PLA", "from": "User", "inherits": "Acme PETG @Acme X1", "filament_settings_id": ["Zip PLA"], "nozzle_temperature": ["250", "251"]})";
        mz_zip_writer_add_mem(&zip, "bundle/Zip PLA.json", a.data(), a.size(), MZ_DEFAULT_COMPRESSION);
        const std::string st = "{}";
        mz_zip_writer_add_mem(&zip, "bundle_structure.json", st.data(), st.size(), MZ_DEFAULT_COMPRESSION);
        mz_zip_writer_add_mem(&zip, "bundle/", nullptr, 0, MZ_DEFAULT_COMPRESSION);
        mz_zip_writer_finalize_archive(&zip);
        mz_zip_writer_end(&zip);
        std::vector<std::string> files{zp};
        auto subs = pb.import_presets(files, [](const std::string &) { return 1; }, ForwardCompatibilitySubstitutionRule::EnableSilent);
        Toks l{"import_zip", num(subs.size())};
        for (const auto &f : files) l.push_back(hex(rel(f)));
        emit(l);
        bfs::remove(zp);
        dump_files("import_zip");
    } catch (const std::exception &e) {
        err("import_zip", e);
    }

    // a printer with a nozzle rack in multi-diameter mode
    try {
        // The vendor section of the AppConfig does not install this variant: make it selectable.
        pb.printers.find_preset("Acme X1 0.4 rack", false, true)->is_visible = true;
        pb.printers.select_preset_by_name("Acme X1 0.4 rack", true);
        pb.update_compatible(PresetSelectCompatibleType::Always);
        pb.set_num_filaments(2);
        pb.filament_presets = {"Acme PLA @Acme X1", "Acme PETG @Acme X1"};
        pb.project_config.option<ConfigOptionBool>("multi_nozzle_diameter_mode")->value = true;
        DynamicPrintConfig file;
        file.set_deserialize_strict("extruder_nozzle_count", "3");
        file.set_deserialize_strict("extruder_nozzle_volume_type", "Standard,High Flow,Standard");
        file.set_deserialize_strict("extruder_nozzle_diameter", "0.4,0.6,0.6");
        file.set_deserialize_strict("extruder_nozzle_slot", "0,1,2");
        pb.nozzle_inventory.open_project(pb.printers, file);
        dump_cfg("rack:project", pb.project_config);
        Toks l{"rack_helpers"};
        push_list(l, pb.compute_filament_supported_nozzle_diameters(2));
        push_list(l, pb.get_filament_nozzle_diameter_options(0));
        push_list(l, pb.get_filament_nozzle_diameter_options(1));
        for (const auto &v : pb.get_installed_nozzle_variants()) {
            l.push_back(hex(v.first));
            l.push_back(num((int) v.second));
        }
        push_list(l, pb.get_extra_compatible_printers_by_full_stat());
        l.push_back(b(pb.is_filament_nozzle_diameter_valid(1, "0.6")));
        emit(l);
        pb.update_compatible(PresetSelectCompatibleType::Always);
        dump_bundle("rack", pb);
        std::map<int, AMSMapInfo> dm;
        dm[0]                 = AMSMapInfo{"0", "0"};
        dm[0].nozzle_diameter = "0.6";
        dm[1]                 = AMSMapInfo{"0", "3"};
        pb.validate_filament_nozzle_diameter_selections(&dm, true);
        dump_bundle("rack_validated", pb);
        pb.seed_filament_nozzles_for_multi_diameter("0.6");
        dump_cfg("rack_seeded", pb.project_config);
        std::string sel;
        bool        r1 = pb.set_filament_nozzle_diameter(0, "0.4", &sel);
        bool        r2 = pb.set_filament_nozzle_diameter(1, "0.8");
        bool        r3 = pb.set_filament_nozzle_diameter(5, "0.4");
        emit({"rack_set_diameter", b(r1), hex(sel), b(r2), b(r3)});
        pb.incompatible_nozzle_diameter_slots = {1};
        pb.validate_filament_nozzle_diameter_selections();
        dump_bundle("rack_revalidated", pb);
        dump_cfg("rack_full", pb.full_config());
        pb.stamp_printer_nozzle_diameter_mode();
        dump_preset("rack_stamp", pb.printers.get_edited_preset());
        emit({"rack_multi_user", b(pb.is_multi_nozzle_user_printer_preset(pb.printers.get_edited_preset()))});
    } catch (const std::exception &e) {
        err("rack", e);
    }

    // wizard, saving, copies, project embedded presets
    try {
        DynamicPrintConfig cfg = pb.full_config();
        pb.load_config_from_wizard("Wizard Set", cfg, Semver());
        dump_bundle("wizard", pb);
        dump_files("wizard");
        pb.filaments.get_edited_preset().config.option<ConfigOptionStrings>("filament_vendor", true)->values = {"Saved"};
        pb.save_changes_for_preset("Saved PLA", Preset::TYPE_FILAMENT, {"nozzle_temperature"});
        dump_bundle("saved", pb);
        dump_files("saved");
    } catch (const std::exception &e) {
        err("misc", e);
    }
    write_file(D + "/old/process/old.json", "{}");
    write_file(D + "/old/filament/sub/f.json", "{\"a\": 1}");
    for (int pass = 0; pass < 2; ++pass) {
        try {
            pb.copy_files(D + "/old");
            emit({"copy_files", "ok"});
        } catch (const std::exception &e) {
            err("copy_files", e);
        }
        bfs::create_directories(D + "/data/old");
    }
    dump_files("copied");
    try {
        Preset e1(Preset::TYPE_FILAMENT, "Embedded PLA");
        e1.is_project_embedded = true;
        e1.config.set_key_value("inherits", new ConfigOptionString("Acme PLA @Acme X1"));
        e1.config.set_deserialize_strict("nozzle_temperature", "222,224");
        Preset e2(Preset::TYPE_PRINT, "Embedded Process");
        e2.is_project_embedded = true;
        e2.config.set_key_value("inherits", new ConfigOptionString("Nope"));
        std::vector<Preset *> embedded{&e1, &e2};
        auto subs = pb.load_project_embedded_presets(embedded, ForwardCompatibilitySubstitutionRule::EnableSilent);
        emit({"load_embedded", num(subs.size()), num(pb.get_current_project_embedded_presets().size())});
        dump_preset("embedded:e1", e1);
        pb.filament_presets = {"Embedded PLA", "Acme PETG @Acme X1"};
        pb.reset_project_embedded_presets();
        dump_bundle("embedded_reset", pb);
    } catch (const std::exception &e) {
        err("misc", e);
    }
}

static void gen_bundle()
{
    const std::string data = D + "/data";
    const std::string res  = D + "/resources";
    set_data_dir(data);
    set_resources_dir(res);
    write_file(res + "/profiles/BBL/filament/filament_name_map.json", R"({"Acme X1 0.6 nozzle": {"Old PLA": "Acme PLA @Acme X1", "Old PETG": "Acme PETG @Acme X1"}})");

    gen_free_functions();

    PresetBundle *pbp = new PresetBundle();
    PresetBundle &pb  = *pbp;
    dump_bundle("new", pb);
    try {
        pb.setup_directories();
        emit({"setup_directories", "ok"});
    } catch (const std::exception &e) {
        err("setup_directories", e);
    }
    write_vendor(data + "/system");
    write_user(data + "/user/default");
    dump_files("written");

    AppConfig config;
    // AppConfig::set() has a bool overload, which a string literal would pick: pass std::string.
    auto cset = [&config](const std::string &section, const std::string &key, const std::string &value) { config.set(section, key, value); };
    cset("app", "version", "02.09.01.70");
    cset("presets", "machine", "Acme X1 0.6 nozzle");
    cset("presets", "process", "My Process");
    cset("presets", "filament", "My PLA");
    cset("presets", "filament_01", "Acme PETG @Acme X1.ini");
    cset("presets", "filament_colors", "#FF0000,#00FF00");
    cset("presets", "flush_volumes_vector", "140|150|160|170");
    cset("presets", "extruder_ams_count", "4#1|1#0,4#2");
    cset("presets", "filament_is_mixed", "0,0");
    cset("presets", "filament_mixed_gradient_curve", "0,0.2|1,0.8;0,1");
    cset("app", "flush_multiplier", "1|1.5");
    try {
        auto r = pb.load_presets(config, ForwardCompatibilitySubstitutionRule::EnableSilent);
        emit({"load_presets", num(r.first.size()), hex(rel(r.second))});
    } catch (const std::exception &e) {
        err("load_presets", e);
    }
    dump_bundle("loaded", pb);
    dump_app("loaded", config);
    dump_files("loaded");

    // full configs
    try {
        dump_cfg("full", pb.full_config());
        dump_cfg("full_noext", pb.full_config(false));
        DynamicPrintConfig sec = pb.full_config_secure();
        emit({"full_secure", num(sec.keys().size()), cfg_hash(sec)});
        dump_cfg("full_maps", pb.full_config(true, std::vector<int>{2, 1}, std::vector<int>{1, 0}));
    } catch (const std::exception &e) {
        err("full_config", e);
    }
    try {
        Preset printer = pb.printers.get_edited_preset();
        Preset print   = pb.prints.get_edited_preset();
        std::vector<Preset> fils;
        for (const auto &n : pb.filament_presets) fils.push_back(*pb.filaments.find_preset(n, true));
        dump_cfg("construct", PresetBundle::construct_full_config(printer, print, pb.project_config, fils, true, std::nullopt, std::nullopt));
        std::vector<Preset> one{fils.front()};
        dump_cfg("construct1", PresetBundle::construct_full_config(printer, print, pb.project_config, one, false, std::vector<int>{2}, std::nullopt));
        dump_cfg("construct_noext", PresetBundle::construct_full_config(printer, print, pb.project_config, fils, false, std::nullopt, std::nullopt));
        dump_preset("construct:print", print);
    } catch (const std::exception &e) {
        err("construct_full_config", e);
    }
    try {
        for (bool m : {true, false}) {
            auto mat = pb.get_full_flush_matrix(m);
            Toks l{"flush_matrix", b(m), num(mat.size())};
            for (const auto &e : mat) {
                l.push_back(num(e.size()));
                for (const auto &r : e)
                    for (float v : r) l.push_back(dbl(v));
            }
            emit(l);
        }
    } catch (const std::exception &e) {
        err("flush_matrix", e);
    }

    // helpers
    emit({"extruder_count", num(pb.get_printer_extruder_count()), b(pb.support_different_extruders())});
    {
        Toks l{"nozzle_volume_list"};
        for (auto t : pb.get_printer_nozzle_volume_list()) l.push_back(num((int) t));
        emit(l);
        std::vector<int> fm{1, 2, 3, 0};
        Toks l2{"default_nvt_for_filaments"};
        for (int v : pb.get_default_nozzle_volume_types_for_filaments(fm)) l2.push_back(num(v));
        emit(l2);
    }
    for (auto [id, printer, sys] : std::vector<std::tuple<std::string, std::string, bool>>{
             {"AP1", "", false}, {"AP1", "Acme X1 0.6 nozzle", false}, {"AT1", "Acme X1 0.6 nozzle", false}, {"AP1", "", true}, {"ZZ", "", false}, {"", "", false}}) {
        auto r = pb.get_filament_by_filament_id(id, printer, sys);
        if (!r)
            emit({"filament_by_id", hex(id), hex(printer), b(sys), "none"});
        else
            emit({"filament_by_id", hex(id), hex(printer), b(sys), hex(r->filament_name), hex(r->filament_id), hex(r->filament_type), hex(r->vendor),
                  num(r->nozzle_temp_range_low), num(r->nozzle_temp_range_high), num(r->temperature_vitrification), b(r->is_support), b(r->is_system),
                  num(r->filament_printable), hex(r->setting_id), num(r->get_filament_extruder_compatibility())});
    }
    for (auto [t, n] : std::vector<std::pair<std::string, std::string>>{{"Acme X1", "0.6"}, {"Acme X1", ""}, {"Acme X1", "0.0"}, {"", "0.4"}, {"Nope", "0.4"}}) {
        Toks l{"printer_names", hex(t), hex(n)};
        for (const auto &s : pb.get_printer_names_by_printer_type_and_nozzle(t, n)) l.push_back(hex(s));
        emit(l);
    }
    for (auto [m, v] : std::vector<std::pair<std::string, std::string>>{{"", ""}, {"Acme X1", "0.4"}, {"", "0.4"}, {"Nope", ""}}) {
        Preset *p = pb.get_similar_printer_preset(m, v);
        emit({"similar_printer", hex(m), hex(v), hex(p ? p->name : "<null>")});
    }
    emit({"model_files", hex(rel(pb.get_texture_for_printer_model("Acme X1"))), hex(rel(pb.get_stl_model_for_printer_model("Acme X1"))),
          hex(rel(pb.get_hotend_model_for_printer_model("Acme X1"))), hex(rel(pb.get_texture_for_printer_model("Nope")))});
    {
        VendorMap vm;
        vm["Custom"] = pb.get_custom_vendor_models();
        dump_vendors("custom", vm);
    }
    emit({"vendor_version", hex(pb.get_vendor_profile_version("Acme").to_string()), hex(pb.get_vendor_profile_version("Nope").to_string())});
    emit({"hrc", num(pb.get_required_hrc_by_filament_id("AP1")), num(pb.get_required_hrc_by_filament_id("AG1")), num(pb.get_required_hrc_by_filament_id("ZZ"))});
    emit({"by_alias", hex(pb.get_preset_name_by_alias(Preset::TYPE_FILAMENT, "Acme PLA")), hex(pb.get_preset_name_by_alias(Preset::TYPE_PRINTER, "Acme PLA")),
          hex(pb.get_preset_name_by_alias(Preset::TYPE_PRINT, "0.20mm Standard"))});
    emit({"only_edited", b(pb.is_the_only_edited_filament(0)), b(pb.is_the_only_edited_filament(1)), b(pb.is_the_only_edited_filament(5))});
    {
        Toks l{"tpu"};
        for (int v : pb.get_used_tpu_filaments({1, 2})) l.push_back(num(v));
        emit(l);
    }
    {
        Toks l{"mixed", num(pb.physical_filament_count()), num(pb.mixed_filament_count()), b(pb.is_mixed_filament(0))};
        for (size_t i : pb.physical_filament_config_indices()) l.push_back(num(i));
        emit(l);
    }
    {
        Toks l{"supported_diameters"};
        push_list(l, pb.compute_filament_supported_nozzle_diameters(3));
        push_list(l, pb.get_filament_nozzle_diameter_options(0));
        l.push_back(b(pb.is_filament_nozzle_diameter_valid(0, "0.6")));
        l.push_back(b(pb.is_filament_nozzle_diameter_valid(0, "0.4")));
        l.push_back(b(pb.is_filament_nozzle_diameter_valid(0, "")));
        for (const auto &v : pb.get_installed_nozzle_variants()) {
            l.push_back(hex(v.first));
            l.push_back(num((int) v.second));
        }
        l.push_back(hex(pb.tray_diameter_for_filament_slot(0)));
        push_list(l, pb.get_extra_compatible_printers_by_full_stat());
        l.push_back(b(pb.is_multi_nozzle_user_printer_preset(pb.printers.get_edited_preset())));
        emit(l);
    }
    try {
        DynamicPrintConfig full = pb.full_config();
        std::set<std::string> diffs;
        int                   r = pb.validate_presets("proj.3mf", full, diffs);
        Toks                  l{"validate", num(r)};
        for (const auto &s : diffs) l.push_back(hex(s));
        emit(l);
        DynamicPrintConfig full2 = pb.full_config();
        full2.option<ConfigOptionStrings>("inherits_group", true)->values = {"0.20mm Standard @Acme X1", "Acme PLA @Acme X1", "Acme PETG @Acme X1", "Acme X1 0.6 nozzle"};
        full2.option<ConfigOptionStrings>("filament_settings_id", true)->values = {"Acme PLA @Acme X1", "Acme PETG @Acme X1"};
        full2.option<ConfigOptionString>("printer_settings_id", true)->value  = "Acme X1 0.6 nozzle";
        full2.option<ConfigOptionStrings>("different_settings_to_system", true)->values = {"", "machine_start_gcode;layer_height", "", "filament_start_gcode"};
        diffs.clear();
        r = pb.validate_presets("proj2.3mf", full2, diffs);
        Toks l2{"validate", num(r)};
        for (const auto &s : diffs) l2.push_back(hex(s));
        emit(l2);
    } catch (const std::exception &e) {
        err("validate", e);
    }
    {
        Preset *p = pb.get_preset_differed_for_save(pb.filaments.get_edited_preset());
        emit({"differed_for_save", hex(p ? p->name : "<null>"), p ? cfg_hash(p->config) : std::string("-")});
        delete p;
        std::map<std::string, std::string> kv;
        int                                r = pb.get_differed_values_to_update(pb.filaments.get_edited_preset(), kv);
        Toks                               l{"differed_values", num(r), num(kv.size())};
        for (const auto &e : kv) {
            l.push_back(hex(e.first));
            l.push_back(hex(e.second));
        }
        emit(l);
    }

    pb.export_selections(config);
    dump_app("export", config);

    // AMS counts and selections
    pb.set_ams_count(0, 2, 1);
    pb.set_ams_count(1, 1, 3);
    pb.set_ams_count(5, 1, 1);
    {
        auto c0 = pb.get_ams_count(0), c1 = pb.get_ams_count(1), c5 = pb.get_ams_count(5);
        emit({"ams_count", num(c0.first), num(c0.second), num(c1.first), num(c1.second), num(c5.first), num(c5.second)});
    }
    pb.initialize_ams_selection(0, "dev1", {"A", "B", "C"}, {"X"});
    pb.initialize_ams_selection(0, "dev1", {"Q"}, {});
    pb.resize_ams_selection(1, "dev1", {"A", "B", "C"}, {"X", "Y"}, 3, 1);
    pb.resize_ams_selection(0, "dev2", {"C", "A", "D"}, {"X", "Z"}, 4, 2);
    pb.replace_ams_selection(1, "dev3", {"R"}, {"S", "T"});
    emit({"ams_selection", b(pb.get_ams_selection(7) == nullptr)});
    dump_bundle("ams", pb);

    // AMS sync
    pb.filament_ams_list.clear();
    pb.filament_ams_list[0] = tray("AG1", "#00FF00FF", "A1", "PETG", "0", "0", true);
    pb.filament_ams_list[1] = tray("ZZ9", "#0000FFFF", "A2", "PLA", "0", "1", true);
    pb.filament_ams_list[2] = tray("", "#FFFFFFFF", "A3", "", "0", "2", true);
    pb.filament_ams_list[3] = tray("AT1", "#111111FF", "A4", "TPU", "0", "3", true);
    {
        DynamicPrintConfig t = tray("AP1", "#FF0000FF", "B1", "PLA", "1", "0", false);
        t.set_key_value("tray_nozzle_diameter", new ConfigOptionFloats({0.6}));
        t.set_key_value("tray_nozzle_volume_type", new ConfigOptionInts({1}));
        pb.filament_ams_list[0x10000] = t;
    }
    {
        AMSComboInfo info;
        pb.get_ams_cobox_infos(info, false);
        Toks l{"cobox"};
        push_list(l, info.ams_filament_presets);
        push_list(l, info.ams_filament_colors);
        push_list(l, info.ams_names);
        l.push_back(b(info.empty()));
        AMSComboInfo info2;
        pb.get_ams_cobox_infos(info2, true);
        push_list(l, info2.ams_filament_presets);
        emit(l);
        auto fi = pb.get_extruder_filament_info();
        Toks l2{"extruder_filament_info", num(fi.size())};
        for (const auto &v : fi) l2.push_back(num(v.size()));
        emit(l2);
    }
    {
        std::vector<std::pair<DynamicPrintConfig *, std::string>> unknowns;
        std::map<int, AMSMapInfo>                                 maps;
        MergeFilamentInfo                                         merge;
        unsigned int n = pb.sync_ams_list(unknowns, false, maps, false, merge, false);
        Toks         l{"sync_ams", num(n), num(unknowns.size())};
        for (const auto &u : unknowns) {
            int key = -1;
            for (auto &kv : pb.filament_ams_list)
                if (&kv.second == u.first) key = kv.first;
            l.push_back(num(key));
            l.push_back(hex(u.second));
        }
        l.push_back(num(merge.merges.size()));
        emit(l);
        dump_maps("sync1", maps);
        dump_bundle("sync1", pb);
    }
    {
        std::vector<std::pair<DynamicPrintConfig *, std::string>> unknowns;
        std::map<int, AMSMapInfo>                                 maps;
        maps[0] = AMSMapInfo{"0", "1"};
        maps[1] = AMSMapInfo{"1", "0"};
        maps[4] = AMSMapInfo{"1", "0"};
        MergeFilamentInfo merge;
        unsigned int      n = pb.sync_ams_list(unknowns, true, maps, true, merge, true);
        Toks              l{"sync_ams", num(n), num(unknowns.size())};
        for (const auto &u : unknowns) {
            int key = -1;
            for (auto &kv : pb.filament_ams_list)
                if (&kv.second == u.first) key = kv.first;
            l.push_back(num(key));
            l.push_back(hex(u.second));
        }
        l.push_back(num(merge.merges.size()));
        for (const auto &m : merge.merges) {
            l.push_back(num(m.size()));
            for (int i : m) l.push_back(num(i));
        }
        emit(l);
        dump_maps("sync2", maps);
        dump_bundle("sync2", pb);
    }

    // filament count changes
    try {
        pb.set_num_filaments(4, "#123456");
        dump_bundle("num4", pb);
        pb.update_num_filaments(1);
        dump_bundle("del1", pb);
        pb.set_filament_preset(1, "Acme PETG @Acme X1 (modified)");
        pb.set_filament_preset(9, "x");
        pb.set_filament_preset(2, "Nope");
        dump_bundle("set_filament", pb);
    } catch (const std::exception &e) {
        err("num_filaments", e);
    }

    // printer switch
    try {
        pb.printers.select_preset_by_name("Acme X1 0.4 nozzle", true);
        pb.update_compatible(PresetSelectCompatibleType::Always);
        dump_bundle("switch04", pb);
        pb.on_extruders_count_changed(1);
        dump_bundle("extruders1", pb);
        dump_cfg("full04", pb.full_config());
        pb.set_calibrate_printer("Acme X1 0.6 nozzle");
        emit({"calibrate", num(pb.calibrate_filaments.size()), hex(pb.calibrate_printer ? pb.calibrate_printer->name : "<null>")});
        pb.set_calibrate_printer("");
        emit({"calibrate", num(pb.calibrate_filaments.size())});
        pb.stamp_printer_nozzle_diameter_mode();
        dump_preset("stamp", pb.printers.get_edited_preset());
        pb.validate_filament_nozzle_diameter_selections();
        pb.seed_filament_nozzles_for_multi_diameter("0.4");
        std::string sel;
        bool        r1 = pb.set_filament_nozzle_diameter(0, "0.4", &sel);
        bool        r2 = pb.set_filament_nozzle_diameter(0, "0.6", nullptr, true);
        emit({"set_nozzle_diameter", b(r1), hex(sel), b(r2)});
        dump_bundle("nozzle", pb);
    } catch (const std::exception &e) {
        err("switch", e);
    }

    // defaults replaced
    try {
        pb.prints.select_preset(0);
        pb.filament_presets = {"Default Filament", "Acme PETG @Acme X1", "Default Filament"};
        bool r = pb.replace_default_presets();
        emit({"replace_default_presets", b(r)});
        dump_bundle("replaced", pb);
    } catch (const std::exception &e) {
        err("replace_default_presets", e);
    }

    // external config (a project)
    try {
        pb.printers.select_preset_by_name("Acme X1 0.6 nozzle", true);
        pb.update_compatible(PresetSelectCompatibleType::Always);
        pb.set_num_filaments(2);
        pb.filament_presets = {"My PLA", "Acme PETG @Acme X1"};
        DynamicPrintConfig cfg = pb.full_config();
        cfg.option<ConfigOptionFloat>("layer_height")->value = 0.24;
        pb.load_config_model("project.3mf", cfg);
        dump_bundle("model", pb);
        DynamicPrintConfig cfg1 = pb.full_config(false);
        cfg1.option<ConfigOptionStrings>("filament_colour")->values = {"#AABBCC"};
        pb.load_config_model("one.3mf", cfg1);
        dump_bundle("model1", pb);
        try {
            pb.load_config_file(D + "/none.ini", ForwardCompatibilitySubstitutionRule::Disable);
        } catch (const std::exception &e) {
            err("load_config_file", e);
        }
        DynamicPrintConfig empty;
        try {
            pb.load_config_model("empty.3mf", empty);
        } catch (const std::exception &e) {
            err("load_config_model", e);
        }
        auto embedded = pb.get_current_project_embedded_presets();
        emit({"embedded", num(embedded.size())});
        pb.reset_project_embedded_presets();
        dump_bundle("reset_embedded", pb);
    } catch (const std::exception &e) {
        err("model", e);
    }

    // import / export
    try {
        const std::string exp = D + "/export";
        bfs::create_directories(exp);
        int  asked = 0;
        auto r     = pb.export_current_configs(exp, [&asked](const std::string &) { ++asked; return 3; }, true);
        Toks l{"export_current_configs", num(asked)};
        for (const auto &f : r) l.push_back(hex(rel(f)));
        emit(l);
        dump_files("exported");
        write_file(D + "/import/Imported PLA.json", std::string(R"({"version": ")") + VERSION +
                   R"(", "name": "Imported PLA", "from": "User", "inherits": "Acme PLA @Acme X1", "filament_id": "IP1", "filament_settings_id": ["Imported PLA"], "nozzle_temperature": ["230", "232"]})");
        write_file(D + "/import/Imported Root.json", std::string(R"({"version": ")") + VERSION +
                   R"(", "name": "Imported Root", "from": "User", "inherits": "", "filament_settings_id": ["Imported Root"], "filament_type": ["ABS"]})");
        write_file(D + "/import/Bad.json", R"({"version": "x", "name": "Bad", "filament_settings_id": ["Bad"]})");
        write_file(D + "/import/Unknown.json", std::string(R"({"version": ")") + VERSION + R"(", "name": "Unknown", "layer_height": "0.1"})");
        std::vector<std::string> files{D + "/import/Imported PLA.json", D + "/import/Imported Root.json", D + "/import/Bad.json", D + "/import/Unknown.json",
                                       D + "/import/missing.zip"};
        int  confirms = 0;
        auto subs     = pb.import_presets(files, [&confirms](const std::string &) { ++confirms; return 1; }, ForwardCompatibilitySubstitutionRule::EnableSilent);
        Toks l2{"import_presets", num(subs.size()), num(confirms)};
        for (const auto &f : files) l2.push_back(hex(rel(f)));
        emit(l2);
        dump_bundle("imported", pb);
        dump_files("imported");
    } catch (const std::exception &e) {
        err("import_export", e);
    }

    // user presets from the cloud, removal, directories
    try {
        std::map<std::string, std::map<std::string, std::string>> my_presets;
        my_presets["Cloud PLA"] = {{"type", "filament"}, {"name", "Cloud PLA"}, {"inherits", "Acme PLA @Acme X1"}, {"version", VERSION},
                                   {"setting_id", "CS1"}, {"updated_time", "1200"}, {"nozzle_temperature", "218,219"}, {"filament_settings_id", "Cloud PLA"}};
        my_presets["Cloud Process"] = {{"type", "print"}, {"name", "Cloud Process"}, {"inherits", "0.20mm Standard @Acme X1"}, {"version", VERSION},
                                       {"setting_id", "CS2"}, {"updated_time", "1300"}, {"layer_height", "0.28"}, {"print_settings_id", "Cloud Process"}};
        my_presets["No Type"] = {{"name", "No Type"}};
        my_presets["Odd Type"] = {{"type", "odd"}};
        auto subs = pb.load_user_presets(config, my_presets, ForwardCompatibilitySubstitutionRule::EnableSilent);
        emit({"load_user_presets_cloud", num(subs.size())});
        dump_bundle("cloud", pb);
        std::map<std::string, std::map<std::string, std::string>> sys;
        sys["Acme PETG @Acme X1"] = {{"type", "filament"}, {"setting_id", "NEW"}};
        sys["Acme PLA @Acme X1"]  = {{"type", "filament"}};
        sys["Nope"]               = {{"type", "print"}, {"setting_id", "X"}};
        pb.update_system_preset_setting_ids(sys);
        std::vector<std::string> del;
        pb.save_user_presets(config, del);
        pb.update_user_presets_directory("u2");
        pb.remove_user_presets_directory("");
        pb.remove_user_presets_directory("u2");
        std::map<std::string, std::map<std::string, std::string>> keep;
        pb.remove_users_preset(config, &keep);
        dump_bundle("users", pb);
        dump_files("users");
    } catch (const std::exception &e) {
        err("users", e);
    }

    gen_more(pb);

    // copy, reset
    {
        PresetBundle copy(pb);
        dump_bundle("copy", copy);
        pb.set_default_suppressed(false);
        pb.reset(false);
        dump_bundle("reset", pb);
    }
    delete pbp;
}

int main(int argc, char **argv)
{
    if (argc < 2) {
        fprintf(stderr, "usage: gen <output directory>\n");
        return 1;
    }
    std::string out = bfs::absolute(argv[1]).string();
    D               = out + "/scratch";
    bfs::remove_all(D);
    bfs::create_directories(D);
    g_out = fopen((out + "/preset_bundle.txt").c_str(), "wb");
    if (!g_out) {
        perror("preset_bundle.txt");
        return 1;
    }
    const bfs::path cwd = bfs::current_path();
    bfs::current_path(D);
    gen_bundle();
    fclose(g_out);
    bfs::current_path(cwd);
    bfs::remove_all(D);
    return 0;
}
