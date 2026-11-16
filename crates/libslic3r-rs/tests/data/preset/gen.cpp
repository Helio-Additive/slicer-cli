// Generator of the vectors of tests/preset_native_parity.rs: libslic3r's Preset.hpp / Preset.cpp at
// the pin (references/BambuStudio/src/libslic3r; the override layer has no copy of them).
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
//   gzip -9 -n -c <output directory>/preset.txt > $DATA/preset.txt.gz
//
// It runs one scenario and writes preset.txt. The Rust test runs the same scenario on the port and
// has to print the same lines. The scenario works in <output directory>/scratch (an absolute path,
// removed first), which is also the current directory; the dumps write that path as "<D>". Times read from the clock (an updated_time
// above 1500000000) are written as "NOW". The vectors in this directory were made on macOS arm64
// with Apple clang and libc++.
//
// Line format: tokens separated by one space. A text token is the hexadecimal of its bytes ("-"
// for the empty text), a boolean 0 or 1, an integer decimal, a list its length and the texts. The
// first token names the line:
//
//   keys <name> <list>                       the option-key lists of Preset and PhysicalPrinter
//   fll <i> <bits>                           filament_list_less(key i, key j) for every j
//   ini <i> <type|!what> <n> (<key> <data> <m> (<key> <data>)*m)*n
//                                            read_ini of a text and guess_config_file_type
//   ver / vals <file> ...                    get_version_from_json / get_values_from_json
//   ext <i> <config>                         extend_default_config_length
//   cfg <config>                             a configuration: number of options, key and serialize()
//   p <summary>                              a preset: see preset_summary()
//   coll <tag> ...                           a collection: size, defaults, selection, its presets
//   file <path> <content>                    a file under the scratch directory after a step
//   <other>                                  the result of one call, named after the function

#include "libslic3r/Preset.hpp"
#include "libslic3r/PresetBundle.hpp"
#include "libslic3r/AppConfig.hpp"
#include "libslic3r/libslic3r.h"

#include <boost/filesystem.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <cstdio>
#include <cstring>
#include <fstream>
#include <functional>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <vector>

// Defined in Preset.cpp with external linkage, not declared in Preset.hpp.
namespace Slic3r {
bool is_compatible_with_parent_printer(const PresetWithVendorProfile &preset, const PresetWithVendorProfile &active_printer);
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
// The scratch directory written as <D>.
static std::string rel(std::string s)
{
    for (size_t pos = s.find(D); !D.empty() && pos != std::string::npos; pos = s.find(D, pos)) s.replace(pos, D.size(), "<D>");
    return s;
}
static std::string norm_time(long long t) { return t > 1500000000LL ? std::string("NOW") : std::to_string(t); }
static std::string idx(size_t i) { return i == size_t(-1) ? std::string("-1") : std::to_string(i); }

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
        l.push_back(hex(ser(c.option(k))));
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
        add(ser(c.option(k)));
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
    emit({"coll", tag, num(c.size()), num(const_cast<PresetCollection &>(c).num_default_presets()), idx(c.get_selected_idx()),
          b(c.is_default_suppressed()), hex(rel(c.path_from_name("x", false))), num(c.num_visible()), b(c.has_defaults_only())});
    for (size_t i = 0; i < c.get_presets().size(); ++i) dump_preset(tag + "#" + std::to_string(i), c.get_presets()[i]);
    dump_preset(tag + "#edited", c.get_edited_preset());
}
// The content of a file with the clock-made updated_time values written as NOW.
static std::string norm_content(const std::string &s)
{
    std::string        out;
    std::istringstream is(s);
    std::string        line;
    bool               first = true;
    size_t             pos   = 0;
    while (pos < s.size()) {
        size_t e = s.find('\n', pos);
        line     = s.substr(pos, e == std::string::npos ? std::string::npos : e - pos);
        if (line.rfind("updated_time = ", 0) == 0) {
            long long t = atoll(line.c_str() + 15);
            line        = "updated_time = " + norm_time(t);
        }
        out += line;
        if (e == std::string::npos) break;
        out += '\n';
        pos = e + 1;
    }
    (void) first;
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
    emit({"files", tag, num(files.size())});
    std::map<std::string, std::string> now;
    for (const std::string &f : files) {
        std::string r = rel(f), c = norm_content(read_file(f));
        now[r]        = c;
        auto it       = g_files.find(r);
        if (it == g_files.end() || it->second != c) emit({"file", tag, hex(r), hex(c)});
    }
    for (const auto &e : g_files)
        if (now.find(e.first) == now.end()) emit({"gone", tag, hex(e.first)});
    g_files = now;
}
static DynamicPrintConfig make_cfg(const std::vector<std::pair<std::string, std::string>> &kvs)
{
    DynamicPrintConfig c;
    for (const auto &kv : kvs) c.set_deserialize_strict(kv.first, kv.second);
    return c;
}

// A PresetCollection with its protected members made public (PresetBundle is a friend of it).
struct TPC : public PresetCollection
{
    TPC(Preset::Type type, const std::vector<std::string> &keys, const StaticPrintConfig &defaults, const std::string &default_name)
        : PresetCollection(type, keys, defaults, default_name)
    {}
    using PresetCollection::merge_presets;
    using PresetCollection::select_preset_by_name_strict;
    using PresetCollection::set_custom_preset_alias;
    using PresetCollection::update_map_alias_to_profile_name;
    using PresetCollection::update_map_system_profile_renamed;
    using PresetCollection::update_vendor_ptrs_after_copy;
    void assign(const PresetCollection &rhs) { PresetCollection::operator=(rhs); }
};

// ---------------------------------------------------------------------------------------------

static void gen_keys()
{
    auto k = [](const char *name, const std::vector<std::string> &v) {
        Toks l{"keys", name};
        push_list(l, v);
        emit(l);
    };
    k("print", Preset::print_options());
    k("filament", Preset::filament_options());
    k("machine_limits", Preset::machine_limits_options());
    k("nozzle", Preset::nozzle_options());
    k("printer", Preset::printer_options());
    k("sla_print", Preset::sla_print_options());
    k("sla_material", Preset::sla_material_options());
    k("sla_printer", Preset::sla_printer_options());
    k("physical_printer", PhysicalPrinter::printer_options());
}

static void gen_filament_list_less()
{
    std::vector<FilamentListKey> keys = {
        {"Bambu PLA Basic", "Bambu Lab", "PLA"}, {"Bambu PLA Matte", "Bambu", "PLA"}, {"Bambu PETG HF", "Bambu Lab", "PETG"},
        {"Bambu Zeta", "Bambu Lab", "PLA"},      {"Bambu Alpha", "Bambu Lab", "ABS"}, {"Generic PLA", "Generic", "PLA"},
        {"Generic TPU", "Generic", "TPU"},       {"Generic PC", "Generic", "PC"},     {"PolyLite PLA", "Polymaker", "PLA"},
        {"Fiberon PA6-CF", "Polymaker", "PA"},   {"Fiberon PET-CF", "Polymaker", "PET"}, {"PolyMax PLA", "Polymaker", "PLA"},
        {"Overture PLA", "Overture", "PLA"},     {"eSUN PLA+", "eSUN", "PLA"},        {"My Custom", "", "PLA"},
        {"Another", "", "PETG"},                 {"Acme PLA", "Acme", "PLA"},         {"Acme PETG", "Acme", "PETG"},
        {"Acme ASA", "Acme", "ASA"},             {"Zed PLA", "Zed", "PLA"},           {"Acme ABS", "Acme", "ABS"},
        {"Bambu TPU 95A", "Bambu Lab", "TPU"},   {"Bambu Support G", "Bambu Lab", "PA"}, {"Fiberon", "Polymaker", "PA"},
    };
    for (size_t i = 0; i < keys.size(); ++i) {
        std::string bits;
        for (size_t j = 0; j < keys.size(); ++j) bits += filament_list_less(keys[i], keys[j]) ? '1' : '0';
        emit({"fll", num(i), bits});
    }
}

static const std::vector<std::string> &ini_texts()
{
    static const std::vector<std::string> v = {
        "nozzle_diameter = 0.4\nfilament_diameter = 1.75\n",
        "[print:abc]\nlayer_height = 0.2\n[filament:x]\na = 1\n",
        "last_export_path = /tmp\n[presets]\nprint = x\n[recent]\nsettings_folder = a\nlast_opened_folder = b\n",
        "[settings]\nx=1\nnozzle_diameter=0.4\n",
        "; comment\n# c2\n\n  key  =  value with spaces  \n[empty]\n[s]\nk=v\n",
        "[a]\nx=1\n[a]\ny=2\n",
        "k=1\nk=2\n",
        "novalue\n",
        "=x\n",
        "[unterminated\n",
        "a=1\r\nb = 2\r\n\r\n[sec]\r\nc = 3",
        "[vendor]\nname = Acme\nconfig_version = 1.2.3\n[printer_model:MK3]\nname = Original\nvariants = 0.4; 0.6\n",
        "",
    };
    return v;
}

static void gen_ini()
{
    const auto &texts = ini_texts();
    for (size_t i = 0; i < texts.size(); ++i) {
        boost::property_tree::ptree tree;
        Toks                        l{"ini", num(i)};
        try {
            std::istringstream is(texts[i]);
            boost::property_tree::read_ini(is, tree);
            l.push_back(num((int) guess_config_file_type(tree)));
        } catch (const std::exception &e) {
            l.push_back(hex(std::string("!") + e.what()));
            emit(l);
            continue;
        }
        l.push_back(num(tree.size()));
        for (const auto &kv : tree) {
            l.push_back(hex(kv.first));
            l.push_back(hex(kv.second.data()));
            l.push_back(num(kv.second.size()));
            for (const auto &kv2 : kv.second) {
                l.push_back(hex(kv2.first));
                l.push_back(hex(kv2.second.data()));
            }
        }
        emit(l);
    }
}

static void gen_json()
{
    std::string dir = D + "/json";
    write_file(dir + "/v_ok.json", R"({"version": "02.01.00.05", "name": "x", "a": "1", "b": "two"})");
    write_file(dir + "/v_bad.json", R"({"version": "abc", "a": "x"})");
    write_file(dir + "/v_missing.json", R"({"name": "x", "c": "3"})");
    write_file(dir + "/v_num.json", R"({"version": 3, "a": "1"})");
    write_file(dir + "/v_parse.json", R"({ nope)");
    write_file(dir + "/v_array.json", R"([1, 2])");
    write_file(dir + "/v_trailing.json", R"({"version": "1.2.3.4"} trailing)");
    for (const char *f : {"v_ok.json", "v_bad.json", "v_missing.json", "v_num.json", "v_parse.json", "v_array.json", "v_trailing.json", "v_none.json"}) {
        std::string path = dir + "/" + f;
        try {
            emit({"ver", f, hex(get_version_from_json(path).to_string())});
        } catch (const std::exception &e) {
            emit({"ver", f, hex(std::string("!") + e.what())});
        }
        std::vector<std::string>           keys = {"a", "b", "c", "version"};
        std::map<std::string, std::string> kv;
        kv["c"] = "pre";
        try {
            int  n = get_values_from_json(path, keys, kv);
            Toks l{"vals", f, num(n), num(kv.size())};
            for (const auto &p : kv) {
                l.push_back(hex(p.first));
                l.push_back(hex(p.second));
            }
            emit(l);
        } catch (const std::exception &e) {
            emit({"vals", f, hex(std::string("!") + e.what())});
        }
    }
    bfs::remove_all(dir);
}

static void gen_extend()
{
    DynamicPrintConfig inherit = make_cfg({{"filament_extruder_variant", "\"Direct Drive Standard\";\"Direct Drive High Flow\""},
                                           {"print_extruder_variant", "\"Direct Drive Standard\";\"Direct Drive High Flow\""},
                                           {"print_extruder_id", "1,2"},
                                           {"printer_extruder_variant", "\"Direct Drive Standard\";\"Direct Drive High Flow\""},
                                           {"printer_extruder_id", "1,2"}});
    DynamicPrintConfig defaults = make_cfg({{"nozzle_temperature", "200"}, {"filament_retraction_length", "0.8"}, {"filament_flow_ratio", "0.95"}});
    std::vector<std::vector<std::pair<std::string, std::string>>> cases = {
        {{"nozzle_temperature", "220"}, {"filament_flow_ratio", "0.9"}, {"filament_type", "PLA"}},
        {{"print_extruder_variant", "\"Direct Drive Standard\""}, {"wall_loops", "3"}, {"inner_wall_speed", "100"}},
        {{"nozzle_temperature", "nil"}, {"filament_retraction_length", "nil"}, {"filament_flow_ratio", "0.9,0.8,0.7"}},
        {{"printer_extruder_variant", "\"Direct Drive Standard\";\"Direct Drive High Flow\";\"X\""}, {"printer_extruder_id", "1,2,3"},
         {"nozzle_diameter", "0.4"}, {"max_layer_height", "0.28"}},
    };
    for (size_t i = 0; i < cases.size(); ++i) {
        for (int mode = 0; mode < 3; ++mode) {
            DynamicPrintConfig c = make_cfg(cases[i]);
            Toks               l{"ext", num(i), num(mode)};
            try {
                if (mode == 0)
                    extend_default_config_length(c, inherit, false, {});
                else if (mode == 1)
                    extend_default_config_length(c, {}, true, defaults);
                else
                    extend_default_config_length(c, inherit, true, defaults);
                push_cfg(l, c);
            } catch (const std::exception &e) {
                l.push_back(hex(std::string("!") + e.what()));
            }
            emit(l);
        }
    }
}

static void gen_preset_statics()
{
    for (int t = 0; t <= 9; ++t)
        emit({"type", num(t), hex(Preset::get_type_string(Preset::Type(t))), hex(Preset::get_iot_type_string(Preset::Type(t)))});
    for (const char *s : {"process", "print", "filament", "machine", "printer", "bogus", ""})
        emit({"type_from", hex(s), num((int) Preset::get_type_from_string(s))});
    for (const char *s : {" (modified)foo", "foo (modified)", "foo", " (modified)", ""})
        emit({"remove_suffix", hex(s), hex(Preset::remove_suffix_modified(s))});
    Preset::update_suffix_modified("* ");
    emit({"remove_suffix2", hex(Preset::remove_suffix_modified("* bar")), hex(PresetCollection::get_suffix_modified())});
    {
        Preset p(Preset::TYPE_FILAMENT, "Name A", false);
        p.is_dirty = true;
        emit({"label_star", hex(p.label(true))});
    }
    Preset::update_suffix_modified(" (modified)");
    {
        Preset p(Preset::TYPE_PRINT, "MyPreset", false);
        emit({"label", hex(p.label(true)), hex(p.label(false))});
        p.alias = "Alias";
        emit({"label", hex(p.label(true)), hex(p.label(false))});
        p.is_dirty = true;
        emit({"label", hex(p.label(true)), hex(p.label(false))});
    }
    struct DN
    {
        const char *name, *alias, *vendor;
    };
    for (DN d : std::vector<DN>{{"Bambu PLA Basic @BBL X1C", "", "Bambu Lab"},
                                {"  Foo Bar  @x", "", ""},
                                {"Acme ABS @BBL A1", "Acme ABS", "Acme"},
                                {"Acme", "", "Acme"},
                                {"Generic PLA", "Generic PLA", "Generic"},
                                {"NoAt", "", "Bambu"},
                                {"@lead", "", ""}}) {
        Preset p(Preset::TYPE_FILAMENT, d.name, false);
        p.alias = d.alias;
        if (*d.vendor) p.config.set_deserialize_strict("filament_vendor", std::string("\"") + d.vendor + "\"");
        emit({"display_name", hex(d.name), hex(p.display_name())});
    }
    {
        DynamicPrintConfig c = make_cfg({{"filament_diameter", "1.75,2.85,1.75"},
                                         {"nozzle_temperature", "200"},
                                         {"filament_type", "PLA"},
                                         {"filament_settings_id", "\"a\""},
                                         {"compatible_printers", "\"x\""},
                                         {"filament_dev_ams_drying_time", "8"},
                                         {"filament_density", "1.24"},
                                         {"relative_correction", "1,0.5"},
                                         {"filament_cost", "20,30"}});
        Preset::normalize(c);
        dump_cfg("normalize", c);
        DynamicPrintConfig c2 = make_cfg({{"nozzle_temperature", "200"}, {"filament_settings_id", "\"a\""}});
        Preset::normalize(c2);
        dump_cfg("normalize2", c2);
        DynamicPrintConfig def = make_cfg({{"nozzle_temperature", "200"}, {"filament_type", "PLA"}});
        DynamicPrintConfig c3  = make_cfg({{"nozzle_temperature", "210"}, {"layer_height", "0.2"}, {"filament_type", "PETG"}, {"wall_loops", "2"}});
        emit({"remove_invalid_keys", hex(Preset::remove_invalid_keys(c3, def))});
        dump_cfg("remove_invalid_keys", c3);
    }
    for (int t = 0; t <= 9; ++t) {
        std::string            id_name, variant_name;
        std::set<std::string> *s1 = nullptr, *s2 = nullptr;
        Preset::get_extruder_names_and_keysets(Preset::Type(t), id_name, variant_name, &s1, &s2);
        emit({"keysets", num(t), hex(id_name), hex(variant_name), num(s1->size()), num(s2->size())});
    }
    {
        Preset p(Preset::TYPE_PRINTER, "pt", false);
        emit({"printer_technology", num((int) p.printer_technology())});
        p.config.set_deserialize_strict("printer_technology", "SLA");
        emit({"printer_technology", num((int) p.printer_technology()), num((int) Preset::printer_technology(p.config))});
        p.printer_technology_ref() = ptFFF;
        emit({"printer_technology", num((int) p.printer_technology())});
    }
}

static void push_vendor(Toks &l, const VendorProfile &v)
{
    l.push_back(hex(v.name));
    l.push_back(hex(v.id));
    l.push_back(hex(v.config_version.to_string()));
    l.push_back(hex(v.config_update_url));
    l.push_back(hex(v.changelog_url));
    l.push_back(num(v.models.size()));
    for (const VendorProfile::PrinterModel &m : v.models) {
        l.push_back(hex(m.id));
        l.push_back(hex(m.name));
        l.push_back(num((int) m.technology));
        l.push_back(hex(m.family));
        l.push_back(num(m.variants.size()));
        for (const auto &var : m.variants) l.push_back(hex(var.name));
        push_list(l, m.default_materials);
        l.push_back(hex(m.bed_model));
        l.push_back(hex(m.bed_texture));
    }
    push_list(l, std::vector<std::string>(v.default_filaments.begin(), v.default_filaments.end()));
    push_list(l, std::vector<std::string>(v.default_sla_materials.begin(), v.default_sla_materials.end()));
    l.push_back(num(v.num_variants()));
    push_list(l, v.families());
    l.push_back(b(v.valid()));
}

static void gen_vendor()
{
    const std::string dir = D + "/vendor";
    write_file(dir + "/Acme.ini", "[vendor]\nname = Acme\nconfig_version = 1.2.3\nconfig_update_url = https://example.invalid/acme\n"
                                  "changelog_url = https://example.invalid/log\n\n"
                                  "[printer_model:MK3]\nname = Original MK3\nvariants = 0.4; 0.6; 0.4\ntechnology = FFF\n"
                                  "default_materials = \"PLA @Acme\"; \"PETG @Acme\"; \"\"; \"PLA @Acme\"\nbed_model = mk3_bed.stl\nbed_texture = mk3.svg\n\n"
                                  "[printer_model:SL1]\nvariants = default\nfamily = SLA\n\n"
                                  "[printer_model:NOVAR]\nname = No variants\n\n"
                                  "[printer_model:BADTECH]\ntechnology = LASER\nvariants = 0.4\ndefault_filaments = \"X\"\n\n"
                                  "[printer_model:BADVAR]\nvariants = \"unterminated\n\n"
                                  "[default_filaments]\nPLA @Acme = 1\nPETG @Acme = 0\n\n[default_sla_materials]\nResin = 1\n");
    write_file(dir + "/BBL.ini", "[vendor]\nname = BBL\nconfig_version = 2.0.0\n[printer_model:MK3]\nvariants = 0.4\n"
                                 "[printer_model:MK2S]\nvariants=0.4\nfamily = Custom\n[printer_model:X9]\nvariants=0.4\n");
    write_file(dir + "/NoVendor.ini", "[other]\na = 1\n");
    write_file(dir + "/NoName.ini", "[vendor]\nconfig_version = 1.0.0\n");
    write_file(dir + "/BadVersion.ini", "[vendor]\nname = X\nconfig_version = one\n");
    write_file(dir + "/BadIni.ini", "[vendor\n");
    for (auto c : std::vector<std::pair<std::string, bool>>{{"Acme.ini", true}, {"Acme.ini", false}, {"BBL.ini", true}, {"NoVendor.ini", true},
                                                           {"NoName.ini", true}, {"BadVersion.ini", true}, {"BadIni.ini", true}, {"Missing.ini", true}}) {
        Toks l{"vendor", hex(c.first), b(c.second)};
        try {
            push_vendor(l, VendorProfile::from_ini(dir + "/" + c.first, c.second));
        } catch (const std::exception &e) {
            l.push_back(hex(rel(std::string("!") + e.what())));
        }
        emit(l);
    }
    VendorProfile::PrinterModel pm;
    pm.bottom_texture_end_name = "end";
    pm.middle_texture_rect     = "1,2,3,4";
    pm.use_double_extruder_default_texture = "true";
    Toks l{"bed_texture_maps"};
    for (const auto &kv : pm.get_bed_texture_maps()) {
        l.push_back(hex(kv.first));
        l.push_back(hex(kv.second));
    }
    l.push_back(b(pm.variant("x") == nullptr));
    l.push_back(hex(pm.support_side_panel_fan));
    emit(l);
    bfs::remove_all(dir);
}

// The scenario on the collections.
static void gen_collections()
{
    const std::string USER = D + "/user";

    // Vendors.
    VendorMap vendors;
    {
        VendorProfile bbl("BBL");
        bbl.name           = "BBL";
        bbl.config_version = *Semver::parse("02.00.00.01");
        VendorProfile::PrinterModel x1;
        x1.id       = "X1C";
        x1.name     = "Bambu Lab X1 Carbon";
        x1.model_id = "BL-P001";
        x1.variants.emplace_back("0.4");
        x1.variants.emplace_back("0.6");
        x1.bed_model = "x1_bed.stl";
        VendorProfile::PrinterModel a1;
        a1.id       = "A1";
        a1.name     = "Bambu Lab A1";
        a1.model_id = "N2S";
        a1.variants.emplace_back("0.4");
        bbl.models = {x1, a1};
        vendors["BBL"] = bbl;
        VendorProfile custom("Custom");
        custom.name = "Custom";
        VendorProfile::PrinterModel mk;
        mk.id       = "MK1";
        mk.name     = "MyKlipper";
        mk.model_id = "C11";
        custom.models   = {mk};
        vendors["Custom"] = custom;
    }
    const VendorProfile *BBL = &vendors["BBL"];

    // The printers of a PresetBundle.
    PresetBundle bundle;
    bundle.vendors                      = vendors;
    BBL                                 = &bundle.vendors["BBL"];
    PrinterPresetCollection &printers   = bundle.printers;
    auto                     add_printer = [&](const std::string &name, const std::string &model, const std::string &variant,
                               const std::string &nozzle, bool system, const std::string &inherits) -> Preset & {
        DynamicPrintConfig c = make_cfg({{"printer_model", model}, {"printer_variant", variant}, {"nozzle_diameter", nozzle}, {"inherits", inherits}});
        Preset &p = printers.load_preset("<sys>/" + name + ".json", name, c, false);
        p.is_system = system;
        if (system) p.vendor = BBL;
        return p;
    };
    add_printer("Bambu Lab X1 Carbon 0.4 nozzle", "Bambu Lab X1 Carbon", "0.4", "0.4", true, "");
    add_printer("Bambu Lab X1 Carbon 0.6 nozzle", "Bambu Lab X1 Carbon", "0.6", "0.6", true, "");
    add_printer("Bambu Lab A1 0.4 nozzle", "Bambu Lab A1", "0.4", "0.4,0.4", true, "");
    add_printer("My X1C", "Bambu Lab X1 Carbon", "0.4", "0.4", false, "Bambu Lab X1 Carbon 0.4 nozzle");
    add_printer("Klipper Box", "MyKlipper", "0.4", "0.4", false, "");
    emit({"select_printer", b(printers.select_preset_by_name("Bambu Lab X1 Carbon 0.4 nozzle", false))});
    dump_coll("printers", printers);

    // PrinterPresetCollection
    {
        const Preset *p = printers.find_system_preset_by_model_and_variant("Bambu Lab X1 Carbon", "0.6");
        emit({"find_system_preset", hex(p ? p->name : "<null>")});
        p = printers.find_system_preset_by_model_and_variant("Bambu Lab X1 Carbon", "");
        emit({"find_system_preset", hex(p ? p->name : "<null>")});
        p = printers.find_system_preset_by_model_and_variant("MyKlipper", "");
        emit({"find_system_preset", hex(p ? p->name : "<null>")});
        p = printers.find_custom_preset_by_model_and_variant("MyKlipper", "0.4");
        emit({"find_custom_preset", hex(p ? p->name : "<null>")});
        p = printers.find_custom_preset_by_model_and_variant("", "0.4");
        emit({"find_custom_preset", hex(p ? p->name : "<null>")});
        for (bool so : {true, false}) {
            Toks l{"find_all_presets_by_model", b(so)};
            for (const Preset *q : printers.find_all_presets_by_model("Bambu Lab X1 Carbon", so)) l.push_back(hex(q->name));
            emit(l);
        }
        emit({"only_default_printers", b(printers.only_default_printers())});
        emit({"default_preset_for", hex(printers.default_preset_for(make_cfg({{"printer_technology", "SLA"}})).name)});
        DynamicPrintConfig generic;
        generic.set_key_value("printer_technology", new ConfigOptionEnumGeneric(&ConfigOptionEnum<PrinterTechnology>::get_enum_values(), (int) ptSLA));
        emit({"default_preset_for", hex(printers.default_preset_for(generic).name)});
        {
            Toks l{"diameters"};
            push_list(l, printers.diameters_of_selected_printer());
            emit(l);
        }
        Toks l{"presets_with_print_host_information"};
        push_list(l, PhysicalPrinter::presets_with_print_host_information(printers));
        emit(l);
    }

    // Prints.
    TPC prints(Preset::TYPE_PRINT, Preset::print_options(), static_cast<const PrintRegionConfig &>(FullPrintConfig::defaults()), "Default Setting");
    prints.default_preset().config.optptr("print_settings_id", true);
    prints.default_preset().compatible_printers_condition();
    prints.default_preset().inherits();
    prints.select_preset(0);
    for (auto nm : {std::make_pair("0.20mm Standard @BBL X1C", "0.2"), std::make_pair("0.16mm Optimal @BBL X1C", "0.16")}) {
        const DynamicPrintConfig c = make_cfg({{"layer_height", nm.second}, {"compatible_printers", "\"Bambu Lab X1 Carbon 0.4 nozzle\""}});
        Preset &p = prints.load_preset("<sys>/p.json", nm.first, c, false); // the const& overload
        p.is_system = true;
        p.vendor    = BBL;
    }
    prints.update_map_alias_to_profile_name();
    prints.select_preset_by_name("0.16mm Optimal @BBL X1C", false);

    // Filaments, as PresetBundle::PresetBundle() makes them.
    TPC filaments(Preset::TYPE_FILAMENT, Preset::filament_options(), static_cast<const PrintRegionConfig &>(FullPrintConfig::defaults()),
                  "Default Filament");
    filaments.default_preset().config.option<ConfigOptionStrings>("filament_settings_id", true)->values = {""};
    filaments.default_preset().compatible_printers_condition();
    filaments.default_preset().inherits();
    {
        auto &default_config = filaments.default_preset().config;
        for (const std::string &opt_key : default_config.keys()) {
            ConfigOption *opt             = default_config.optptr(opt_key, false);
            bool          is_override_key = is_filament_extruder_override_key(opt_key);
            if (!is_override_key || !opt->nullable()) continue;
            opt->deserialize("nil", ForwardCompatibilitySubstitutionRule::Disable);
        }
    }
    filaments.select_preset(0);
    dump_cfg("default_filament", filaments.default_preset().config);

    struct Sys
    {
        std::string name, alias, filament_id, renamed;
        std::vector<std::pair<std::string, std::string>> kvs;
    };
    const std::string X1C04 = "\"Bambu Lab X1 Carbon 0.4 nozzle\"";
    std::vector<Sys>  sys   = {
        {"Bambu PLA Basic @BBL X1C", "Bambu PLA Basic", "GFA00", "",
         {{"filament_type", "\"PLA\""}, {"filament_vendor", "\"Bambu Lab\""}, {"compatible_printers", X1C04 + ";\"Bambu Lab X1 Carbon 0.6 nozzle\""}, {"nozzle_temperature", "220"}}},
        {"Bambu PLA Basic @BBL A1", "Bambu PLA Basic", "GFA00", "",
         {{"filament_type", "\"PLA\""}, {"filament_vendor", "\"Bambu Lab\""}, {"compatible_printers", "\"Bambu Lab A1 0.4 nozzle\""}}},
        {"Bambu PLA Basic @BBL X1C 0.6", "Bambu PLA Basic", "GFA00", "",
         {{"filament_type", "\"PLA\""}, {"filament_vendor", "\"Bambu Lab\""}, {"compatible_printers", "\"Bambu Lab X1 Carbon 0.6 nozzle\""}}},
        {"Generic PETG @BBL X1C", "Generic PETG", "GFG99", "Generic PETG Old",
         {{"filament_type", "\"PETG\""}, {"filament_vendor", "\"Generic\""}, {"compatible_printers", X1C04}}},
        {"Generic ABS", "Generic ABS", "GFB99", "", {{"filament_type", "\"ABS\""}, {"filament_vendor", "\"Generic\""}}},
        {"Special For Optimal", "Special", "GFS00", "",
         {{"filament_type", "\"PLA\""}, {"compatible_prints", "\"0.16mm Optimal @BBL X1C\""}, {"compatible_printers_condition", "nozzle_diameter[0]==0.4"}}},
        {"Conditional", "Conditional", "GFC00", "", {{"filament_type", "\"PLA\""}, {"compatible_printers_condition", "nozzle_diameter[0]==0.8"}}},
    };
    for (const Sys &s : sys) {
        const DynamicPrintConfig c = make_cfg(s.kvs);
        Preset &p     = filaments.load_preset("<sys>/" + s.name + ".json", s.name, c, false); // the const& overload
        p.is_system   = true;
        p.vendor      = BBL;
        p.alias       = s.alias;
        p.filament_id = s.filament_id;
        if (!s.renamed.empty()) p.renamed_from.push_back(s.renamed);
    }
    {
        // A two-extruder system preset: the variant options extended to two values.
        DynamicPrintConfig c = filaments.default_preset().config;
        c.apply(make_cfg({{"filament_type", "\"PLA\""},
                          {"filament_vendor", "\"Bambu Lab\""},
                          {"filament_extruder_variant", "\"Direct Drive Standard\";\"Direct Drive High Flow\""},
                          {"compatible_printers", "\"Bambu Lab H2D 0.4 nozzle\""},
                          {"nozzle_temperature", "220,230"},
                          {"filament_max_volumetric_speed", "20,22"}}));
        extend_default_config_length(c, {}, false, {});
        Preset &p     = filaments.load_preset("<sys>/h2d.json", "Bambu PLA Basic @BBL H2D", std::move(c), false);
        p.is_system   = true;
        p.vendor      = BBL;
        p.alias       = "Bambu PLA Basic";
        p.filament_id = "GFA00";
    }
    filaments.update_map_alias_to_profile_name();
    filaments.update_map_system_profile_renamed();
    dump_coll("sys", filaments);

    // User presets on disk.
    const std::string FD = USER + "/filament";
    write_file(FD + "/base/Custom Root @Bambu Lab X1 Carbon 0.4 nozzle.json",
               R"({"version": "2.0.0.0", "name": "Custom Root @Bambu Lab X1 Carbon 0.4 nozzle", "from": "User", "inherits": "", "filament_type": ["PETG"], "filament_vendor": ["Acme"], "nozzle_temperature": ["245"]})");
    write_file(FD + "/My PLA.json",
               R"({"version": "2.1.0.0", "inherits": "Bambu PLA Basic @BBL X1C", "nozzle_temperature": ["225"], "filament_settings_id": ["My PLA"]})");
    write_file(FD + "/My PLA.info", "sync_info = update\nuser_id = u1\nsetting_id = S1\nbase_id = B1\nupdated_time = 1000\n");
    write_file(FD + "/My PLA 2.json", R"({"version": "2.1.0.0", "inherits": "My PLA"})");
    write_file(FD + "/Broken.json", R"({ "version": )");
    write_file(FD + "/Broken.info", "updated_time = 5\n");
    write_file(FD + "/Future.json", R"({"version": "3.0.0.0", "inherits": "Generic ABS"})");
    write_file(FD + "/NoVersion.json", R"({"inherits": "Generic ABS"})");
    write_file(FD + "/Invisible.json",
               R"({"version": "2.0.0.0", "inherits": "Generic ABS", "instantiation": "false", "description": "hidden one", "filament_id": "FX1"})");
    write_file(FD + "/SameIds.json", R"({"version": "2.0.0.0", "inherits": "Generic ABS", "filament_cost": ["30"]})");
    write_file(FD + "/SameIds.info", "setting_id = X\nbase_id = X\nuser_id = null\nupdated_time = 2000\n");
    write_file(FD + "/Orphan.json", R"({"version": "2.0.0.0", "inherits": "Nonexistent"})");
    write_file(FD + "/Bambu PLA Basic @BBL X1C.json", R"({"version": "2.0.0.0"})");
    write_file(FD + "/Root2 @Bambu Lab A1 0.4 nozzle.json", R"({"version": "2.0.0.0", "inherits": "", "filament_type": ["PLA"], "filament_id": "R2"})");
    write_file(FD + "/Root2 @Bambu Lab A1 0.4 nozzle.info", "sync_info = \nuser_id = \nsetting_id = null\nbase_id = null\nupdated_time = 0\n");
    write_file(FD + "/Dual.json", R"({"version": "2.0.0.0", "inherits": "Bambu PLA Basic @BBL H2D", "nozzle_temperature": ["225", "235"]})");
    write_file(FD + "/Single.json", R"({"version": "2.0.0.0", "inherits": "Bambu PLA Basic @BBL H2D", "nozzle_temperature": ["228"]})");
    write_file(FD + "/BadInfo.json", R"({"version": "2.0.0.0", "inherits": "Generic ABS"})");
    write_file(FD + "/BadInfo.info", "this is not an ini line\n");
    write_file(FD + "/readme.txt", "ignored");
    dump_files("before_load");

    PresetsConfigSubstitutions subs;
    try {
        filaments.load_presets(USER, "filament", subs, ForwardCompatibilitySubstitutionRule::EnableSilent);
        emit({"load_presets", "ok", num(subs.size())});
    } catch (const std::exception &e) {
        emit({"load_presets", hex(std::string("!") + e.what())});
    }
    dump_coll("loaded", filaments);
    for (const char *n : {"My PLA", "Dual", "Single", "Custom Root @Bambu Lab X1 Carbon 0.4 nozzle", "Root2 @Bambu Lab A1 0.4 nozzle"})
        dump_cfg(std::string("loaded:") + n, filaments.find_preset(n, false)->config);
    dump_files("after_load");

    // Lookups.
    for (const char *n : {"My PLA", "Generic PETG Old", "Nope", "Default Filament", "Invisible", ""}) {
        const Preset *p  = filaments.find_preset(n, false);
        const Preset *p2 = filaments.find_preset(n, true);
        emit({"find_preset", hex(n), hex(p ? p->name : "<null>"), hex(p2 ? p2->name : "<null>")});
        Preset *pr = filaments.find_preset(n, false, true);
        emit({"find_preset_real", hex(n), hex(pr ? pr->name : "<null>")});
    }
    for (const char *n : {"Bambu PLA Basic", "Generic PETG", "Custom Root", "Root2", "Nope", ""})
        emit({"alias", hex(n), hex(filaments.get_preset_name_by_alias(n))});
    for (const char *n : {"Generic PETG Old", "Nope"}) {
        const std::string *s = filaments.get_preset_name_renamed(n);
        emit({"renamed", hex(n), hex(s ? *s : "<null>")});
    }
    for (size_t i = 0; i < filaments.get_presets().size(); ++i) {
        const Preset &p      = filaments.get_presets()[i];
        const Preset *parent = filaments.get_preset_parent(p);
        const Preset *base   = filaments.get_preset_base(p);
        auto          pwv    = filaments.get_preset_with_vendor_profile(p);
        emit({"lineage", num(i), hex(parent ? parent->name : "<null>"), hex(base ? base->name : "<null>"), hex(pwv.vendor ? pwv.vendor->id : "<null>"),
              b(filaments.is_bbl_brand_filament(p)), b(filaments.is_base_preset(p)), hex(rel(filaments.path_for_preset(p)))});
    }
    {
        Toks l{"system_preset_names"};
        push_list(l, filaments.system_preset_names());
        emit(l);
    }
    emit({"names", hex(filaments.name()), hex(filaments.section_name()), hex(prints.name()), hex(printers.name()), hex(printers.section_name())});
    for (auto pn : std::vector<std::pair<std::string, std::string>>{
             {"Generic ABS", ""}, {"Generic PETG Old", ""}, {"My PLA", ""}, {"My PLA", "Generic ABS"}, {"Nope", "Nope2"}, {"Default Filament", ""}}) {
        std::string inh = pn.second;
        emit({"validate_preset", hex(pn.first), hex(pn.second), b(filaments.validate_preset(pn.first, inh))});
    }
    for (const char *a : {"Bambu PLA Basic", "Custom Root", "Nope"}) {
        emit({"is_alias_exist", hex(a), b(filaments.is_alias_exist(a))});
        Preset q(Preset::TYPE_FILAMENT, "Q", false);
        q.config = filaments.default_preset().config;
        q.config.set_deserialize_strict("compatible_printers", X1C04);
        emit({"is_alias_exist_p", hex(a), b(filaments.is_alias_exist(a, &q))});
    }
    emit({"path_from_name", hex(rel(filaments.path_from_name("abc", false))), hex(rel(filaments.path_from_name("abc.JSON", true)))});
    {
        Preset q = *filaments.find_preset("Root2 @Bambu Lab A1 0.4 nozzle", false);
        q.alias.clear();
        q.name = "Another Root @Bambu Lab A1 0.4 nozzle";
        emit({"get_preset_alias", hex(filaments.get_preset_alias(q, false)), hex(q.alias)});
        Preset q2 = q;
        q2.alias.clear();
        q2.name = "Third Root @Bambu Lab X1 Carbon 0.4 nozzle";
        q2.config.set_deserialize_strict("compatible_printers", X1C04);
        q2.config.set_deserialize_strict("inherits", "Generic ABS");
        emit({"get_preset_alias", hex(filaments.get_preset_alias(q2, true)), hex(q2.alias)});
        emit({"get_preset_alias", hex(filaments.get_preset_alias(q2, false)), hex(q2.alias)});
    }

    // Selection, edits and dirty tracking.
    emit({"first_visible_idx", num(filaments.first_visible_idx()), num(filaments.first_compatible_idx())});
    emit({"select_by_name", b(filaments.select_preset_by_name("My PLA", false))});
    emit({"select_by_name", b(filaments.select_preset_by_name("My PLA", false))});
    emit({"select_by_name", b(filaments.select_preset_by_name(" (modified)My PLA", true))});
    emit({"selected", idx(filaments.get_selected_idx()), hex(filaments.get_selected_preset_name()), b(filaments.current_is_dirty()), b(filaments.saved_is_dirty())});
    filaments.get_edited_preset().config.set_deserialize_strict("nozzle_temperature", "231");
    filaments.get_edited_preset().config.set_deserialize_strict("filament_settings_id", "\"Other\"");
    auto list = [](const char *tag, const std::vector<std::string> &v) {
        Toks l{tag};
        push_list(l, v);
        emit(l);
    };
    emit({"dirty", b(filaments.current_is_dirty()), b(filaments.saved_is_dirty())});
    list("current_dirty_options", filaments.current_dirty_options(false));
    list("current_dirty_options_deep", filaments.current_dirty_options(true));
    list("different_from_parent", filaments.current_different_from_parent_options(false));
    list("different_from_parent_deep", filaments.current_different_from_parent_options(true));
    {
        const Preset *parent = filaments.get_selected_preset_parent();
        emit({"selected_parent", hex(parent ? parent->name : "<null>")});
    }
    list("dirty_without", PresetCollection::dirty_options_without_option_list(&filaments.get_edited_preset(), &filaments.get_selected_preset(),
                                                                                  {"nozzle_temperature"}, false));
    list("dirty_without_deep", PresetCollection::dirty_options_without_option_list(&filaments.get_edited_preset(), &filaments.get_selected_preset(),
                                                                                       {"nozzle_temperature#0"}, true));
    emit({"update_dirty", b(filaments.update_dirty()), b(filaments.update_dirty()), b(filaments.get_selected_preset().is_dirty)});
    {
        Preset a = filaments.get_edited_preset();
        Preset r = filaments.get_selected_preset();
        r.config.erase("compatible_prints");
        emit({"is_dirty_optional", b(PresetCollection::is_dirty(&a, &r)), b(PresetCollection::is_dirty(&a, nullptr))});
        list("dirty_optional", PresetCollection::dirty_options(&a, &r, false));
        list("dirty_optional_deep", PresetCollection::dirty_options(&r, &a, true));
    }

    // Save as a new preset, then over it, then detached.
    filaments.save_current_preset("My PLA Saved");
    dump_coll("saved1", filaments);
    filaments.get_edited_preset().config.set_deserialize_strict("nozzle_temperature", "232");
    filaments.save_current_preset("My PLA Saved");
    filaments.select_preset_by_name("Dual", false);
    filaments.get_edited_preset().config.set_deserialize_strict("nozzle_temperature", "225,240");
    filaments.save_current_preset("Dual Saved");
    filaments.select_preset_by_name("Generic ABS", false);
    filaments.get_edited_preset().config.set_deserialize_strict("filament_cost", "40");
    filaments.save_current_preset("ABS Detached", true);
    filaments.select_preset_by_name("Custom Root @Bambu Lab X1 Carbon 0.4 nozzle", false);
    filaments.save_current_preset("Custom Child @Bambu Lab X1 Carbon 0.4 nozzle");
    filaments.select_preset_by_name("Custom Root @Bambu Lab X1 Carbon 0.4 nozzle", false);
    filaments.get_edited_preset().config.set_deserialize_strict("filament_cost", "55");
    filaments.save_current_preset("Custom Root @Bambu Lab X1 Carbon 0.4 nozzle");
    filaments.save_current_preset("Generic ABS");
    dump_coll("saved2", filaments);
    dump_cfg("custom_child_reloaded", filaments.find_preset("Custom Child @Bambu Lab X1 Carbon 0.4 nozzle", false)->config);
    dump_files("after_save");

    // Compatibility.
    {
        auto printer = printers.get_edited_preset_with_vendor_profile();
        auto print   = prints.get_edited_preset_with_vendor_profile();
        filaments.select_preset_by_name("Generic PETG @BBL X1C", false);
        filaments.update_compatible(printer, &print, PresetSelectCompatibleType::OnlyIfWasCompatible);
        Toks l{"compat1", idx(filaments.get_selected_idx())};
        for (const Preset &p : filaments.get_presets()) l.push_back(b(p.is_compatible));
        l.push_back(b(filaments.get_edited_preset().is_compatible));
        emit(l);
        printers.select_preset_by_name("My X1C", false);
        auto printer2 = printers.get_edited_preset_with_vendor_profile();
        filaments.update_compatible(printer2, nullptr, PresetSelectCompatibleType::Always);
        Toks l2{"compat2", idx(filaments.get_selected_idx())};
        for (const Preset &p : filaments.get_presets()) l2.push_back(b(p.is_compatible));
        emit(l2);
        printers.select_preset_by_name("Bambu Lab X1 Carbon 0.4 nozzle", false);
        auto printer3 = printers.get_edited_preset_with_vendor_profile();
        filaments.select_preset_by_name("Bambu PLA Basic @BBL X1C 0.6", false);
        filaments.set_extra_compatible_printers({"Bambu Lab X1 Carbon 0.6 nozzle"});
        filaments.update_compatible(printer3, nullptr, PresetSelectCompatibleType::Always, [](const Preset &p) { return p.is_system ? 1 : 0; });
        filaments.clear_extra_compatible_printers();
        Toks l3{"compat3", idx(filaments.get_selected_idx())};
        for (const Preset &p : filaments.get_presets()) l3.push_back(b(p.is_compatible));
        emit(l3);
        emit({"alias_after_compat", hex(filaments.get_preset_name_by_alias("Bambu PLA Basic")), num(filaments.first_compatible_idx()),
              num(filaments.first_compatible_idx([](const Preset &p) { return p.name.find("Generic") != std::string::npos ? 5 : 0; }))});
        printers.select_preset_by_name("Bambu Lab A1 0.4 nozzle", false);
        auto printer4 = printers.get_edited_preset_with_vendor_profile();
        filaments.update_compatible(printer4, nullptr, PresetSelectCompatibleType::Never);
        Toks l4{"compat4", idx(filaments.get_selected_idx())};
        for (const Preset &p : filaments.get_presets()) l4.push_back(b(p.is_compatible));
        emit(l4);
        emit({"alias_after_compat4", hex(filaments.get_preset_name_by_alias("Bambu PLA Basic"))});
        filaments.update_compatible(printer4, nullptr, PresetSelectCompatibleType::OnlyIfWasCompatible);
        emit({"compat5", idx(filaments.get_selected_idx()), hex(filaments.get_selected_preset_name())});
        // The free functions.
        const Preset &pla = *filaments.find_preset("Bambu PLA Basic @BBL X1C", false);
        auto          pv  = filaments.get_preset_with_vendor_profile(pla);
        printers.select_preset_by_name("My X1C", false);
        auto myx1c = printers.get_edited_preset_with_vendor_profile();
        emit({"free", b(is_compatible_with_printer(pv, myx1c)), b(is_compatible_with_parent_printer(pv, myx1c)),
              b(is_compatible_with_printer_names(pv, {"x", "Bambu Lab X1 Carbon 0.6 nozzle"})), b(is_compatible_with_printer_names(pv, {})),
              b(is_compatible_with_print(pv, print, myx1c))});
        printers.select_preset_by_name("Bambu Lab X1 Carbon 0.4 nozzle", false);
    }
    dump_coll("compat", filaments);

    // External presets.
    {
        std::set<std::string> none, diff = {"nozzle_temperature"};
        auto                  ext        = [&](const char *tag, const std::string &path, const std::string &name, const std::string &original,
                       const DynamicPrintConfig &cfg, const std::set<std::string> &d, PresetCollection::LoadAndSelect sel, const std::string &fid) {
            try {
                auto r = filaments.load_external_preset(path, name, original, cfg, d, sel, *Semver::parse("2.2.0.0"), fid);
                emit({"external", tag, hex(r.first->name), b(r.second), idx(filaments.get_selected_idx()), hex(filaments.get_edited_preset().name)});
            } catch (const std::exception &e) {
                emit({"external", tag, hex(std::string("!") + e.what())});
            }
        };
        DynamicPrintConfig pla = filaments.find_preset("Bambu PLA Basic @BBL X1C", false)->config;
        ext("same", "/x/a.3mf", "a.3mf", "Bambu PLA Basic @BBL X1C", pla, none, PresetCollection::LoadAndSelect::Always, "");
        DynamicPrintConfig pla2 = pla;
        pla2.set_deserialize_strict("nozzle_temperature", "199");
        ext("modified_never", "/x/a.3mf", "a.3mf", "Bambu PLA Basic @BBL X1C", pla2, none, PresetCollection::LoadAndSelect::Never, "");
        ext("modified_again", "/x/a.3mf", "a.3mf", "Bambu PLA Basic @BBL X1C", pla2, none, PresetCollection::LoadAndSelect::Never, "");
        DynamicPrintConfig abs = filaments.find_preset("Generic ABS", false)->config;
        abs.set_deserialize_strict("inherits", "Generic ABS");
        abs.set_deserialize_strict("filament_cost", "77");
        ext("inherits_always", "/x/b.json", "b.json", "", abs, none, PresetCollection::LoadAndSelect::Always, "");
        ext("inherits_again", "/x/b.json", "b.json", "", abs, none, PresetCollection::LoadAndSelect::OnlyIfModified, "FID");
        DynamicPrintConfig abs2 = abs;
        abs2.set_deserialize_strict("filament_cost", "78");
        ext("inherits_new", "/x/b.json", "b.json", "", abs2, none, PresetCollection::LoadAndSelect::OnlyIfModified, "");
        DynamicPrintConfig petg = filaments.find_preset("Generic PETG @BBL X1C", false)->config;
        petg.set_deserialize_strict("nozzle_temperature", "250");
        ext("renamed", "/x/c.3mf", "c.3mf", "Generic PETG Old", petg, diff, PresetCollection::LoadAndSelect::Always, "");
        DynamicPrintConfig petg2 = petg;
        petg2.set_deserialize_strict("inherits", "Generic PETG @BBL X1C");
        petg2.set_deserialize_strict("filament_cost", "11");
        ext("diff_settings", "/x/d.3mf", "d.3mf", "Unknown", petg2, diff, PresetCollection::LoadAndSelect::OnlyIfModified, "");
        DynamicPrintConfig root = filaments.find_preset("Root2 @Bambu Lab A1 0.4 nozzle", false)->config;
        root.set_deserialize_strict("filament_cost", "12");
        ext("noname", "/x/e.json", "e.json", "", root, none, PresetCollection::LoadAndSelect::Always, "");
        ext("noname2", "/x/e.json", "e.json", "", root, none, PresetCollection::LoadAndSelect::Never, "");
    }
    dump_coll("external", filaments);
    dump_files("after_external");

    // Cloud presets.
    {
        auto user = [&](const char *tag, const std::string &name, const std::map<std::string, std::string> &values) {
            try {
                PresetsConfigSubstitutions s;
                bool                       r = filaments.load_user_preset(name, values, s, ForwardCompatibilitySubstitutionRule::EnableSilent);
                emit({"load_user_preset", tag, b(r), num(s.size()), idx(filaments.get_selected_idx())});
            } catch (const std::exception &e) {
                emit({"load_user_preset", tag, hex(std::string("!") + e.what())});
            }
        };
        std::map<std::string, std::string> v = {{"version", "2.0.0.0"},   {"setting_id", "SX"},          {"updated_time", "500"},
                                                {"user_id", "U"},         {"base_id", "B"},              {"inherits", "Generic ABS"},
                                                {"nozzle_temperature", "240"}, {"filament_id", "null"}, {"name", "Cloud ABS"}};
        user("new", "Cloud ABS", v);
        user("same", "Cloud ABS", v);
        auto v2              = v;
        v2["updated_time"]   = "900";
        v2["nozzle_temperature"] = "241";
        user("newer", "Cloud ABS", v2);
        auto v3            = v;
        v3["updated_time"] = "100";
        user("older", "Cloud ABS", v3);
        auto v4 = v;
        v4.erase("version");
        user("no_version", "Cloud X", v4);
        auto v5 = v;
        v5.erase("setting_id");
        user("no_setting", "Cloud X", v5);
        auto v6 = v;
        v6.erase("base_id");
        user("no_base", "Cloud X", v6);
        auto v7        = v;
        v7["inherits"] = "Missing Parent";
        user("missing_parent", "Cloud X", v7);
        auto v8        = v;
        v8["inherits"] = "";
        v8["filament_type"] = "PLA";
        user("root", "Cloud Root @Bambu Lab X1 Carbon 0.4 nozzle", v8);
        auto v9       = v;
        v9["version"] = "9.0.0.0";
        user("future", "Cloud X", v9);
        auto v10              = v;
        v10["filament_type"]  = "\"PLA\";\"PETG\"";
        v10["filament_scarf_seam_type"] = "bogus";
        user("bad_enum", "A Cloud Enum", v10);
        filaments.update_after_user_presets_loaded();
        emit({"after_user", idx(filaments.get_selected_idx()), hex(filaments.get_selected_preset_name())});
    }
    emit({"need_sync", b(filaments.need_sync("Cloud ABS", "SX", 900)), b(filaments.need_sync("Cloud ABS", "SX", 901)),
          b(filaments.need_sync("Cloud ABS", "SY", 1)), b(filaments.need_sync("Nope", "SX", 1))});
    {
        std::vector<Preset> r;
        int                 n = filaments.get_user_presets(nullptr, r);
        Toks                l{"get_user_presets", num(n)};
        for (const Preset &p : r) l.push_back(hex(p.name));
        emit(l);
    }
    for (const char *n : {"My PLA", "Custom Root @Bambu Lab X1 Carbon 0.4 nozzle", "Cloud ABS", "Generic ABS", "Dual Saved"}) {
        std::map<std::string, std::string> kv;
        Preset                            *p = filaments.find_preset(n, false, true);
        int                                r = filaments.get_differed_values_to_update(*p, kv);
        Toks                               l{"differed_values", hex(n), num(r), num(kv.size())};
        for (const auto &e : kv) {
            l.push_back(hex(e.first));
            l.push_back(hex(e.first == "updated_time" ? norm_time(atoll(e.second.c_str())) : e.second));
        }
        emit(l);
        Preset *d = filaments.get_preset_differed_for_save(*p);
        if (d) {
            dump_preset(std::string("differed:") + n, *d);
            dump_cfg(std::string("differed:") + n, d->config);
            delete d;
        } else
            emit({"differed", hex(n), "null"});
    }
    filaments.set_sync_info_and_save("Custom Root @Bambu Lab X1 Carbon 0.4 nozzle", "SR", "update", 3000);
    filaments.set_sync_info_and_save("My PLA", "S2", "", 0);
    filaments.set_sync_info_and_save("Cloud ABS", "S3", "hold", 0);
    {
        std::vector<std::string> del;
        filaments.save_user_presets(USER, "filament", del);
    }
    dump_coll("cloud", filaments);
    dump_files("after_cloud");

    // Project presets.
    {
        std::vector<Preset *> project = filaments.get_project_embedded_presets();
        emit({"project_presets", num(project.size())});
        for (Preset *p : project) dump_preset("project", *p);
        emit({"reset_project", b(filaments.reset_project_embedded_presets()), idx(filaments.get_selected_idx())});
        filaments.select_preset_by_name("Generic ABS", false);
        PresetsConfigSubstitutions s;
        for (Preset *p : project) p->config.set_deserialize_strict("filament_cost", "99");
        Preset *extra = new Preset(Preset::TYPE_FILAMENT, "Orphan Project", false);
        extra->is_project_embedded = true;
        extra->config              = make_cfg({{"inherits", "Nope"}});
        project.push_back(extra);
        Preset *wrong = new Preset(Preset::TYPE_PRINT, "Wrong Type", false);
        wrong->is_project_embedded = true;
        project.push_back(wrong);
        filaments.load_project_embedded_presets(project, "filament", s, ForwardCompatibilitySubstitutionRule::Enable);
        emit({"load_project", num(s.size()), idx(filaments.get_selected_idx())});
        for (Preset *p : project) delete p;
    }
    dump_coll("project", filaments);

    // Clones.
    {
        // The templates are looked up before each call: a clone inserts into the deque, which moves its elements.
        auto templates = [&filaments]() {
            return std::vector<Preset const *>{filaments.find_preset("Generic ABS", false), filaments.find_preset("My PLA", false)};
        };
        std::vector<std::string> failures;
        bool r = filaments.clone_presets_for_printer(templates(), failures, "Klipper Box", [](std::string p) { return "P" + p; });
        Toks l{"clone_for_printer", b(r)};
        push_list(l, failures);
        emit(l);
        failures.clear();
        r = filaments.clone_presets_for_printer(templates(), failures, "Klipper Box", [](std::string p) { return "Q" + p; });
        Toks l2{"clone_for_printer_again", b(r)};
        push_list(l2, failures);
        emit(l2);
        failures.clear();
        r = filaments.clone_presets_for_printer(templates(), failures, "Klipper Box", [](std::string p) { return "R" + p; }, true);
        Toks l3{"clone_for_printer_force", b(r)};
        push_list(l3, failures);
        emit(l3);
        failures.clear();
        DynamicConfig dc;
        dc.set_key_value("filament_vendor", new ConfigOptionStrings({"Acme"}));
        dc.set_key_value("filament_type", new ConfigOptionStrings({"PETG"}));
        dc.set_key_value("compatible_printers", new ConfigOptionStrings());
        r = filaments.clone_presets_for_filament(filaments.find_preset("Generic ABS", false), failures, "Acme Blue", "AC01", dc, "Klipper Box");
        Toks l4{"clone_for_filament", b(r)};
        push_list(l4, failures);
        emit(l4);
        auto fp = filaments.get_filament_presets();
        Toks l5{"filament_presets", num(fp.size())};
        for (const auto &e : fp) {
            l5.push_back(hex(e.first));
            l5.push_back(num(e.second.size()));
            for (const Preset *p : e.second) l5.push_back(hex(p->name));
        }
        emit(l5);
    }
    dump_coll("clones", filaments);

    // Visibility from the application configuration.
    {
        AppConfig app;
        app.set_variant("BBL", "Bambu Lab X1 Carbon", "0.4", true);
        // std::string values: a string literal would pick the bool overload of AppConfig::set.
        app.set(AppConfig::SECTION_FILAMENTS, "Bambu PLA Basic @BBL A1", std::string("1"));
        app.set(AppConfig::SECTION_FILAMENTS, "Generic PETG Old", std::string("1"));
        app.set(AppConfig::SECTION_FILAMENTS, "Generic ABS", std::string(""));
        Toks l{"visible_from_appconfig"};
        for (const char *n : {"Bambu PLA Basic @BBL X1C", "Bambu PLA Basic @BBL A1", "Generic PETG @BBL X1C", "Generic ABS", "My PLA"}) {
            Preset p = *filaments.find_preset(n, false);
            p.set_visible_from_appconfig(app);
            l.push_back(b(p.is_visible));
        }
        for (const char *n : {"Bambu Lab X1 Carbon 0.4 nozzle", "Bambu Lab X1 Carbon 0.6 nozzle", "My X1C"}) {
            Preset p = *printers.find_preset(n, false);
            p.set_visible_from_appconfig(app);
            l.push_back(b(p.is_visible));
        }
        emit(l);
    }

    // Bundle lookups.
    for (const char *pn : {"Bambu Lab X1 Carbon 0.4 nozzle", "Bambu Lab A1 0.4 nozzle", "Klipper Box", "My X1C"}) {
        printers.select_preset_by_name(pn, false);
        Preset      &edited = printers.get_edited_preset();
        Preset       f      = *filaments.find_preset("Generic PETG @BBL X1C", false);
        Preset       g      = *filaments.find_preset("Generic ABS", false);
        std::string  disp;
        std::string  ft     = f.get_filament_type(disp);
        emit({"bundle", hex(pn), hex(edited.get_printer_type(&bundle)), hex(edited.get_printer_name(&bundle)), hex(edited.get_current_printer_type(&bundle)),
              b(edited.has_lidar(&bundle)), b(edited.is_bbl_vendor_preset(&bundle)), b(f.is_bbl_vendor_preset(&bundle)), b(g.is_bbl_vendor_preset(&bundle)),
              num((int) edited.get_default_bed_type(&bundle)), b(edited.has_cali_lines(&bundle)), hex(ft), hex(disp), hex(edited.get_printer_id()),
              hex(PresetUtils::system_printer_model(edited) ? PresetUtils::system_printer_model(edited)->id : "<null>")});
        emit({"bundle_null", hex(edited.get_printer_type(nullptr)), b(edited.has_lidar(nullptr)), b(edited.is_bbl_vendor_preset(nullptr)),
              num((int) edited.get_default_bed_type(nullptr)), b(edited.has_cali_lines(nullptr))});
    }

    // System printer models, extruder counts, the user directory.
    {
        Preset x = *printers.find_preset("Bambu Lab X1 Carbon 0.4 nozzle", false);
        x.config.set_deserialize_strict("printer_model", "X1C");
        const VendorProfile::PrinterModel *pm = PresetUtils::system_printer_model(x);
        emit({"system_printer", hex(pm ? pm->id : "<null>"), hex(PresetUtils::system_printer_bed_model(x)), hex(PresetUtils::system_printer_bed_texture(x)),
              hex(PresetUtils::system_printer_hotend_model(x))});
        x.set_num_extruders(2);
        dump_cfg("set_num_extruders", x.config);
        prints.update_user_presets_directory(USER, "process");
        emit({"user_presets_directory", hex(rel(prints.path_from_name("p", false)))});
    }

    // Deletion and defaults.
    filaments.select_preset_by_name("My PLA Saved", false);
    emit({"delete_current", b(filaments.delete_current_preset()), idx(filaments.get_selected_idx()), hex(filaments.get_selected_preset_name())});
    filaments.select_preset_by_name("Custom Root @Bambu Lab X1 Carbon 0.4 nozzle", false);
    emit({"delete_current_root", b(filaments.delete_current_preset()), idx(filaments.get_selected_idx())});
    filaments.select_preset_by_name("Default Filament", false);
    emit({"delete_current_default", b(filaments.delete_current_preset())});
    emit({"delete_preset", b(filaments.delete_preset("Invisible")), idx(filaments.get_selected_idx())});
    emit({"delete_preset", b(filaments.delete_preset("Default Filament"))});
    emit({"delete_preset", b(filaments.delete_preset("Acme Blue @Klipper Box")), idx(filaments.get_selected_idx())});
    filaments.set_default_suppressed(false);
    emit({"default_suppressed", b(filaments.is_default_suppressed()), num(filaments.first_visible_idx()), num(filaments.num_visible())});
    filaments.set_default_suppressed(true);
    emit({"strict", b(filaments.select_preset_by_name_strict("Generic ABS")), idx(filaments.get_selected_idx())});
    emit({"strict", b(filaments.select_preset_by_name_strict("Nope")), idx(filaments.get_selected_idx())});
    emit({"selected_name_invalid", hex(filaments.get_selected_preset_name())});
    filaments.get_selected_preset();
    emit({"selected_fixed", idx(filaments.get_selected_idx())});
    filaments.select_preset(1000);
    emit({"select_out_of_range", idx(filaments.get_selected_idx())});
    filaments.discard_current_changes();
    dump_coll("deleted", filaments);
    dump_files("after_delete");

    // Copies and merges.
    {
        TPC other(Preset::TYPE_FILAMENT, Preset::filament_options(), static_cast<const PrintRegionConfig &>(FullPrintConfig::defaults()), "Default Filament");
        Preset &a  = other.load_preset("<sys>/m.json", "Merged A", make_cfg({{"filament_type", "\"PVA\""}}), false);
        a.vendor   = BBL;
        a.is_system = true;
        other.load_preset("<sys>/m.json", "Generic ABS", make_cfg({{"filament_type", "\"ABS\""}}), false);
        Preset &e  = other.load_preset("<sys>/m.json", "Ext", make_cfg({{"filament_type", "\"ABS\""}}), false);
        e.is_external = true;
        VendorMap newv = bundle.vendors;
        Toks      l{"merge"};
        push_list(l, filaments.merge_presets(std::move(other), newv));
        emit(l);
        const Preset *m = filaments.find_preset("Merged A", false);
        emit({"merged_vendor", b(m->vendor == &newv["BBL"])});
        filaments.update_vendor_ptrs_after_copy(bundle.vendors);
        emit({"merged_vendor2", b(m->vendor == &bundle.vendors["BBL"])});
        TPC copy(Preset::TYPE_PRINT, Preset::print_options(), static_cast<const PrintRegionConfig &>(FullPrintConfig::defaults()), "Default Setting");
        copy.assign(filaments);
        dump_coll("assigned", copy);
        emit({"reset"});
        copy.reset(false);
        dump_coll("reset", copy);
    }
    dump_files("end_filaments");

    // Physical printers.
    {
        const std::string PD = USER + "/physical";
        write_file(PD + "/Office.json", R"({"version": "2.0.0.0", "preset_names": ["Bambu Lab X1 Carbon 0.4 nozzle", "My X1C"], "print_host": "192.168.0.2", "host_type": "octoprint"})");
        write_file(PD + "/lab.json", R"({"version": "2.0.0.0", "preset_name": "Bambu Lab A1 0.4 nozzle", "preset_names": ["Bambu Lab A1 0.4 nozzle"]})");
        write_file(PD + "/Klipper.json", R"({"version": "2.0.0.0", "preset_names": ["Klipper Box"]})");
        write_file(PD + "/Empty.json", R"({"version": "2.0.0.0"})");
        PhysicalPrinterCollection pp(PhysicalPrinter::printer_options());
        dump_cfg("physical_default", pp.default_config());
        PresetsConfigSubstitutions s;
        try {
            pp.load_printers(USER, "physical", s, ForwardCompatibilitySubstitutionRule::Enable);
            emit({"load_printers", "ok"});
        } catch (const std::exception &e) {
            emit({"load_printers", hex(std::string("!") + e.what())});
        }
        auto dump_pp = [&](const char *tag) {
            emit({"pp", tag, num(pp().size()), idx(pp.get_selected_idx()), hex(pp.get_selected_printer_name()), hex(pp.get_selected_full_printer_name()),
                  hex(pp.get_selected_printer_preset_name()), b(pp.has_selection()), num((int) pp.get_selected_printer_technology()),
                  hex(rel(pp.path_from_name("y"))), b(pp.empty())});
            for (const PhysicalPrinter &p : pp()) {
                Toks l{"ppp", tag, hex(p.name), hex(rel(p.file)), b(p.loaded)};
                push_list(l, std::vector<std::string>(p.preset_names.begin(), p.preset_names.end()));
                l.push_back(cfg_hash(p.config));
                l.push_back(hex(p.get_full_name("Z")));
                emit(l);
            }
        };
        dump_pp("loaded");
        for (const char *n : {"Office", "office", "KLIPPER", "lab", "Nope", "Empty"}) {
            const PhysicalPrinter *p1 = pp.find_printer(n);
            const PhysicalPrinter *p2 = pp.find_printer(n, false);
            emit({"find_printer", hex(n), hex(p1 ? p1->name : "<null>"), hex(p2 ? p2->name : "<null>")});
        }
        pp.select_printer("Office * My X1C (modified)");
        dump_pp("select1");
        pp.select_printer("lab");
        dump_pp("select2");
        pp.select_printer("Office", "Bambu Lab X1 Carbon 0.4 nozzle");
        emit({"is_selected", b(pp.is_selected(pp.begin() + pp.get_selected_idx(), "Bambu Lab X1 Carbon 0.4 nozzle")), b(pp.is_selected(pp.begin(), "x"))});
        pp.select_printer("Zzz");
        dump_pp("select3");
        {
            Toks l{"with_preset"};
            push_list(l, pp.get_printers_with_preset("My X1C"));
            push_list(l, pp.get_printers_with_only_preset("Klipper Box"));
            push_list(l, pp.get_printers_with_only_preset("My X1C"));
            emit(l);
        }
        PhysicalPrinter np("New One", pp.default_config(), *printers.find_preset("Klipper Box", false));
        np.add_preset("My X1C");
        emit({"add_preset", b(np.add_preset("My X1C")), b(np.delete_preset("Nope")), b(np.delete_preset("My X1C"))});
        pp.save_printer(np);
        dump_pp("saved_new");
        PhysicalPrinter renamed = *pp.find_printer("Office");
        renamed.set_name("Office 2");
        pp.save_printer(renamed, "Office");
        dump_pp("renamed");
        emit({"renamed_config", b(renamed.config.keys().empty())});
        DynamicPrintConfig c = pp.default_config();
        c.set_deserialize_strict("preset_names", "\"My X1C\"");
        pp.load_printer(PD + "/Loaded.json", "Loaded", std::move(c), true, true);
        dump_pp("load_printer");
        pp.delete_preset_from_printers("My X1C");
        dump_pp("deleted_preset");
        emit({"delete_printer", b(pp.delete_printer("New One")), b(pp.delete_printer("zzz"))});
        pp.select_printer("Office 2");
        emit({"delete_selected", b(pp.delete_selected_printer()), b(pp.delete_selected_printer())});
        dump_pp("end");
        emit({"names", hex(PhysicalPrinter::get_short_name("A * B")), hex(PhysicalPrinter::get_short_name(" * B")), hex(PhysicalPrinter::get_short_name("AB")),
              hex(PhysicalPrinter::get_preset_name("A * B (modified)")), hex(PhysicalPrinter::get_preset_name("AB")), hex(PhysicalPrinter::get_preset_name("A")),
              hex(PhysicalPrinter::separator())});
        dump_files("end_physical");
    }
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
    g_out = fopen((out + "/preset.txt").c_str(), "wb");
    // A preset without a file writes its .info file to ".info" in the current directory
    // (Preset::save_info), so the scenario runs in the scratch directory.
    const bfs::path cwd = bfs::current_path();
    bfs::current_path(D);
    if (!g_out) {
        perror("preset.txt");
        return 1;
    }
    gen_keys();
    gen_filament_list_less();
    gen_ini();
    gen_json();
    gen_extend();
    gen_preset_statics();
    gen_vendor();
    gen_collections();
    fclose(g_out);
    bfs::current_path(cwd);
    bfs::remove_all(D);
    return 0;
}
