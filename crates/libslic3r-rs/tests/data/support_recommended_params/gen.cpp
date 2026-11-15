// Reference vectors for crates/libslic3r-rs/src/support_recommended_params.rs, taken from the C++
// at the pin.
//
// The program compiles references/BambuStudio/src/libslic3r/SupportRecommendedParams.cpp as it
// is, with the pin's nlohmann/json.hpp and the system's Boost (filesystem, log, nowide, string
// algorithms). Utils.hpp and libslic3r.h are replaced by the two declarations the file needs
// (resources_dir() and EPSILON), by defining their include guards first.
//
// Build and run, from the repository root (macOS arm64 clang is the reference build; BOOST is the
// prefix Boost is installed under):
//   clang++ -std=c++17 -O2 -ffp-contract=on -DBOOST_LOG_DYN_LINK \
//       -I references/BambuStudio/src/libslic3r -I references/BambuStudio/src -I $BOOST/include \
//       crates/libslic3r-rs/tests/data/support_recommended_params/gen.cpp \
//       -L $BOOST/lib -lboost_filesystem -lboost_log -lboost_thread -o /tmp/srp_gen
//   D=crates/libslic3r-rs/tests/data/support_recommended_params
//   /tmp/srp_gen table $D/resources > $D/vectors.txt
//   /tmp/srp_gen dump references/BambuStudio/resources > $D/pin_table.txt
//
// "dump" prints the table loaded from <resources>/profiles/BBL/filament/support_recommended_params.json.
// "table" prints that, then the cases. Strings are the hex of their bytes ("-" when empty),
// doubles the hex of their bits.
//   KEYS n
//   KEY key n                                  n entries follow
//   ENTRY model_material model_material_type priority support_material_type
//         n support_materials.. n printer_models.. n params { key kind value }
//         kind: 0 double, 1 bool, 2 string, 3 vector<double> (n values..)
//   NAME n { preset_name -> support_recommended_material_name }
//   EVAL n { support(name type soluble support) model(name type soluble support) printer_model
//            -> id built_in_rule zero_top zero_bottom }      id: the "id" parameter of the entry
//                                                              found, -1 when there is none

#define slic3r_Utils_hpp_
#define _libslic3r_h_

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>

namespace Slic3r {
constexpr double EPSILON = 1e-4;
static std::string g_resources_dir;
inline const std::string &resources_dir() { return g_resources_dir; }
} // namespace Slic3r

#include "SupportRecommendedParams.cpp"

#include <boost/log/core.hpp>

using namespace Slic3r;

static uint64_t s = 0x2545F4914F6CDD1Dull;
static uint64_t rnd() { s ^= s << 13; s ^= s >> 7; s ^= s << 17; return s; }
static uint64_t rnd(uint64_t n) { return rnd() % n; }
static void pd(double v) { uint64_t b; memcpy(&b, &v, 8); printf(" %llx", (unsigned long long) b); }
static void ps(const std::string &v)
{
    printf(" ");
    if (v.empty()) printf("-");
    for (unsigned char c : v) printf("%02x", c);
}

static void dump()
{
    const auto &table = support_recommended_params();
    printf("KEYS %zu\n", table.size());
    for (const auto &item : table) {
        printf("KEY"); ps(item.first); printf(" %zu\n", item.second.size());
        for (const SupportRecommendedParams &e : item.second) {
            printf("ENTRY"); ps(e.model_material); ps(e.model_material_type); printf(" %d", e.priority); ps(e.support_material_type);
            printf(" %zu", e.support_material_list.size());
            for (const std::string &v : e.support_material_list) ps(v);
            printf(" %zu", e.printer_models.size());
            for (const std::string &v : e.printer_models) ps(v);
            printf(" %zu", e.params.params.size());
            for (const auto &p : e.params.params) {
                ps(p.first);
                printf(" %zu", p.second.index());
                if (const double *v = std::get_if<double>(&p.second)) pd(*v);
                else if (const bool *v = std::get_if<bool>(&p.second)) printf(" %d", int(*v));
                else if (const std::string *v = std::get_if<std::string>(&p.second)) ps(*v);
                else if (const std::vector<double> *v = std::get_if<std::vector<double>>(&p.second)) {
                    printf(" %zu", v->size());
                    for (double d : *v) pd(d);
                }
            }
            printf("\n");
        }
    }
}

int main(int argc, char **argv)
{
    if (argc != 3) return 1;
    // The log would go to the same stream as the vectors.
    boost::log::core::get()->set_logging_enabled(false);
    g_resources_dir = argv[2];
    load_support_recommended_params();
    dump();
    if (std::string(argv[1]) != "table") return 0;

    // support_recommended_material_name()
    static const char *pieces[] = { "Bambu PLA Basic", "@BBL X1C", " ", "\t", "\n", "\v", "\f", "\r", "@", "Generic PETG",
                                    "\xc2\xa0", "\xe4\xb8\xad", " 0.4 nozzle", "", "x" };
    const int num_names = 300;
    printf("NAME %d\n", num_names);
    for (int i = 0; i < num_names; ++ i) {
        std::string name;
        for (size_t k = 0; k < rnd(6); ++ k) name += pieces[rnd(sizeof(pieces) / sizeof(pieces[0]))];
        ps(name); printf(" ->"); ps(support_recommended_material_name(name)); printf("\n");
    }

    static const char *names[]    = { "", "PETG", "PLA", "Support W", "Generic PLA", "PVA", "Mat\xc3\xa9riau|x", "\xe4\xb8\xad\xe6\x96\x87", "ABS", "Other" };
    static const char *types[]    = { "", "PETG", "PLA", "PVA", "ABS", "TPU", "TPU-AMS", "PETG-CF", "Support W", "Generic PLA", "pla" };
    static const char *printers[] = { "", "P1", "P2", "P3", "P4", "P|1", "Imprimante \xc3\xa9", "1" };
    const int num_evals = 3000;
    printf("EVAL %d\n", num_evals);
    for (int i = 0; i < num_evals; ++ i) {
        SupportMaterialProperties support, model;
        // Mostly the materials and printers of the table, so that the lookup finds entries.
        static const char *hot_support_names[] = { "PETG", "Support W", "PVA", "PLA" };
        static const char *hot_support_types[] = { "PETG", "PVA", "PLA" };
        static const char *hot_model_names[]   = { "PLA", "Generic PLA", "ABS" };
        static const char *hot_model_types[]   = { "PLA", "ABS", "TPU", "PETG" };
        support.name    = rnd(4) == 0 ? names[rnd(10)] : hot_support_names[rnd(4)];
        support.type    = rnd(4) == 0 ? types[rnd(11)] : hot_support_types[rnd(3)];
        support.soluble = rnd(5) == 0;
        support.support = rnd(4) == 0;
        model.name      = rnd(4) == 0 ? names[rnd(10)] : hot_model_names[rnd(3)];
        model.type      = rnd(4) == 0 ? types[rnd(11)] : hot_model_types[rnd(4)];
        model.soluble   = rnd(5) == 0;
        model.support   = rnd(5) == 0;
        const std::string printer = printers[rnd(6) == 0 ? rnd(8) : 1 + rnd(3)];
        const SupportMaterialCombination result = evaluate_support_material_combination(support, model, printer);
        double id = -1.;
        if (result.recommended_params != nullptr)
            id = result.recommended_params->params.get<double>("id").value_or(-2.);
        ps(support.name); ps(support.type); printf(" %d %d", int(support.soluble), int(support.support));
        ps(model.name); ps(model.type); printf(" %d %d", int(model.soluble), int(model.support));
        ps(printer);
        printf(" -> %d %d %d %d\n", int(id), int(result.built_in_rule), int(result.zero_top), int(result.zero_bottom));
    }
    return 0;
}
