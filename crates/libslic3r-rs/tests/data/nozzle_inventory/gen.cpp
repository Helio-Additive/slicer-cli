// Reference vectors of tests/nozzle_inventory_native_parity.rs: Bambu Studio's NozzleInventory.cpp
// at the pin, driven through its public interface by deterministic random operations.
//
// Build, from an empty directory, with PIN = references/BambuStudio/src of this repository, DATA =
// this directory and BOOST = a Boost include directory (headers only):
//
//   cp $PIN/libslic3r/NozzleInventory.cpp $PIN/libslic3r/NozzleInventory.hpp .
//   for h in PrintConfig AppConfig LocalesUtils Log Preset PresetBundle Utils; do
//     echo '#include "env.hpp"' > $h.hpp; done
//   clang++ -std=c++17 -O2 -ffp-contract=on -I. -I$DATA -I$PIN -I$BOOST \
//     $DATA/gen.cpp NozzleInventory.cpp -o gen
//   ./gen > $DATA/vectors.txt
//
// NozzleInventory.cpp is compiled unchanged; env.hpp stands in for the headers it includes.
// `./gen full` writes the state text after every operation instead of its hash, to find a difference.
//
// Format of vectors.txt: a case starts with `CASE`; every other line is an operation with its
// arguments as tokens, then `->` and the FNV-1a 64 hash of the state text after it (state_text()).
// A string token is `~` followed by the text with ' ' and '%' percent-encoded.

#include "NozzleInventory.hpp"
#include "env.hpp"

#include <cstdint>
#include <cstdio>
#include <cstring>

using namespace Slic3r;

static uint64_t g_seed = 0x9E3779B97F4A7C15ull;
static uint64_t rnd()
{
    g_seed ^= g_seed << 13;
    g_seed ^= g_seed >> 7;
    g_seed ^= g_seed << 17;
    return g_seed;
}
static int  rint_(int n) { return int(rnd() % uint64_t(n)); } // 0 .. n-1
static bool chance(int percent) { return rint_(100) < percent; }
template<class T, size_t N> static T pick(const T (&items)[N]) { return items[rint_(int(N))]; }

static std::string tok(const std::string &s)
{
    std::string out = "~";
    for (char c : s) {
        if (c == ' ')
            out += "%20";
        else if (c == '%')
            out += "%25";
        else
            out += c;
    }
    return out;
}

static std::string bits(double v)
{
    uint64_t u;
    std::memcpy(&u, &v, sizeof(u));
    char buf[17];
    std::snprintf(buf, sizeof(buf), "%016llx", (unsigned long long) u);
    return buf;
}

// ---- configs ----

static void set_string(DynamicPrintConfig &c, const char *key, const std::string &v) { c.option<ConfigOptionString>(key, true)->value = v; }
static void set_bool(DynamicPrintConfig &c, const char *key, bool v) { c.option<ConfigOptionBool>(key, true)->value = v; }

// The config as tokens: the number of keys, then per key its name, a type letter and the values.
static std::string cfg_tokens(const DynamicPrintConfig &c)
{
    std::ostringstream o;
    o << "C " << c.options.size();
    for (const auto &[key, opt] : c.options) {
        o << ' ' << key;
        if (auto *s = dynamic_cast<const ConfigOptionString *>(opt.get()))
            o << " s " << tok(s->value);
        else if (auto *b = dynamic_cast<const ConfigOptionBool *>(opt.get()))
            o << " b " << (b->value ? 1 : 0);
        else if (auto *ss = dynamic_cast<const ConfigOptionStrings *>(opt.get())) {
            o << " S " << ss->values.size();
            for (const auto &v : ss->values) o << ' ' << tok(v);
        } else if (auto *i = dynamic_cast<const ConfigOptionInts *>(opt.get())) {
            o << " I " << i->values.size();
            for (int v : i->values) o << ' ' << v;
        } else if (auto *e = dynamic_cast<const ConfigOptionEnumsGeneric *>(opt.get())) {
            o << " I " << e->values.size();
            for (int v : e->values) o << ' ' << v;
        } else if (auto *n = dynamic_cast<const ConfigOptionIntsNullable *>(opt.get())) {
            o << " I " << n->values.size();
            for (int v : n->values) o << ' ' << v;
        } else if (auto *f = dynamic_cast<const ConfigOptionFloats *>(opt.get())) {
            o << " F " << f->values.size();
            for (double v : f->values) o << ' ' << bits(v);
        } else if (auto *fn = dynamic_cast<const ConfigOptionFloatsNullable *>(opt.get())) {
            o << " F " << fn->values.size();
            for (double v : fn->values) o << ' ' << bits(v);
        }
    }
    return o.str();
}

// The config as state text: `key=values;` per key in key order.
static std::string cfg_text(const DynamicPrintConfig &c)
{
    std::ostringstream o;
    auto ints = [&o](const std::vector<int> &values) {
        for (size_t i = 0; i < values.size(); ++i) o << (i ? "," : "") << values[i];
    };
    auto floats = [&o](const std::vector<double> &values) {
        for (size_t i = 0; i < values.size(); ++i) o << (i ? "," : "") << bits(values[i]);
    };
    for (const auto &[key, opt] : c.options) {
        o << key << '=';
        if (auto *s = dynamic_cast<const ConfigOptionString *>(opt.get()))
            o << s->value;
        else if (auto *b = dynamic_cast<const ConfigOptionBool *>(opt.get()))
            o << (b->value ? 1 : 0);
        else if (auto *ss = dynamic_cast<const ConfigOptionStrings *>(opt.get())) {
            for (size_t i = 0; i < ss->values.size(); ++i) o << (i ? "," : "") << '<' << ss->values[i] << '>';
        } else if (auto *i = dynamic_cast<const ConfigOptionInts *>(opt.get()))
            ints(i->values);
        else if (auto *e = dynamic_cast<const ConfigOptionEnumsGeneric *>(opt.get()))
            ints(e->values);
        else if (auto *n = dynamic_cast<const ConfigOptionIntsNullable *>(opt.get()))
            ints(n->values);
        else if (auto *f = dynamic_cast<const ConfigOptionFloats *>(opt.get()))
            floats(f->values);
        else if (auto *fn = dynamic_cast<const ConfigOptionFloatsNullable *>(opt.get()))
            floats(fn->values);
        o << ';';
    }
    return o.str();
}

static const char *const k_models[]   = {"N7", "N7", "O1D", "C12", ""};
static const char *const k_variants[] = {"0.4", "0.4", "0.4", "0.6", "0.2", "0.8", "0.40", "junk"};
static const char *const k_diameters[] = {"0.4", "0.4", "0.40", "0.6", "0.2", "0.8", "", "x", "0.61"};

static int rnd_type() { return chance(8) ? 6 + rint_(2) : rint_(6); }

struct Printer
{
    PresetCollection presets;
    std::string      base_name;
    int              base_kind = 0; // 0 null, 1 the selected preset, 2 another preset
    Preset           other_base;
};

static void make_printer(Printer &p)
{
    DynamicPrintConfig c;
    set_string(c, "printer_model", pick(k_models));
    set_string(c, "printer_variant", pick(k_variants));
    int extruders = 1;
    if (chance(92)) {
        extruders = 1 + rint_(3);
        c.option<ConfigOptionFloatsNullable>("nozzle_diameter", true)->values.assign(size_t(extruders), 0.4);
    }
    std::vector<int> slot_counts(size_t(extruders), 1);
    if (chance(70)) {
        static const int k_counts[] = {1, 1, 2, 3, 6, 0, -1, std::numeric_limits<int>::max()};
        std::vector<int> values;
        const int        n = chance(80) ? extruders : rint_(extruders + 2);
        for (int i = 0; i < n; ++i) {
            values.push_back(pick(k_counts));
            if (i < extruders && values.back() > 0 && values.back() != std::numeric_limits<int>::max())
                slot_counts[size_t(i)] = values.back();
        }
        c.option<ConfigOptionIntsNullable>("extruder_max_nozzle_count", true)->values = values;
    }
    if (chance(60))
        set_bool(c, "handle_hotend_as_extruder", chance(70));
    if (chance(70)) {
        int total = 0;
        for (int v : slot_counts) total += v;
        const int        lens[] = {1, extruders, total, total, rint_(9)};
        const int        n      = pick(lens);
        std::vector<int> values;
        for (int i = 0; i < n; ++i) values.push_back(rnd_type());
        c.option<ConfigOptionEnumsGeneric>("default_nozzle_volume_type", true)->values = values;
    }
    p.presets.selected.name   = std::string("P") + std::to_string(rint_(3)) + " " + c.opt_string("printer_variant") + " nozzle";
    p.presets.edited.name     = p.presets.selected.name;
    p.presets.edited.config   = c;
    p.presets.selected.config = c;
    p.base_kind               = chance(10) ? 0 : (chance(65) ? 1 : 2);
    p.other_base.name         = std::string("Base ") + std::to_string(rint_(2));
}

static void fix_base(Printer &p)
{
    p.presets.base = p.base_kind == 0 ? nullptr : (p.base_kind == 1 ? &p.presets.selected : &p.other_base);
}

// The selected preset as tokens: its name, the name of its base (or `-` for none), the edited config.
static std::string printer_tokens(const Printer &p)
{
    return tok(p.presets.selected.name) + " " + (p.presets.base ? tok(p.presets.base->name) : std::string("-")) + " " +
           cfg_tokens(p.presets.edited.config);
}

static std::vector<int> printer_slot_counts(const DynamicPrintConfig &c)
{
    const auto      *d = c.option<ConfigOptionFloatsNullable>("nozzle_diameter");
    const auto      *m = c.option<ConfigOptionIntsNullable>("extruder_max_nozzle_count");
    const size_t     n = d ? d->values.size() : 1;
    std::vector<int> out;
    for (size_t i = 0; i < n; ++i) {
        int v = 1;
        if (m && i < m->values.size() && !m->is_nil(i) && m->values[i] > 0)
            v = m->values[i];
        out.push_back(v);
    }
    return out;
}

static DynamicPrintConfig make_file_keys(const Printer &p)
{
    DynamicPrintConfig     c;
    const std::vector<int> slot_counts = printer_slot_counts(p.presets.edited.config);
    const int              extruders   = int(slot_counts.size());
    const int              r           = rint_(100);
    if (r < 45) {
        // per-slot keys that are consistent with each other
        const int        n = chance(85) ? extruders : 1 + rint_(4);
        std::vector<int> counts, types, ids;
        std::vector<double> diams;
        int              base = 0;
        for (int e = 0; e < n; ++e) {
            const int cap   = e < extruders ? slot_counts[size_t(e)] : 1;
            const int count = rint_(cap + 2);
            counts.push_back(count);
            for (int k = 0; k < count; ++k) {
                types.push_back(rnd_type());
                static const double k_d[] = {0.4, 0.4, 0.6, 0.2, 0.8, 0.0, 0.47};
                diams.push_back(pick(k_d));
                ids.push_back(chance(80) ? base + k : rint_(12) - 2);
            }
            base += std::max(count, cap);
        }
        c.option<ConfigOptionInts>("extruder_nozzle_count", true)->values               = counts;
        c.option<ConfigOptionEnumsGeneric>("extruder_nozzle_volume_type", true)->values = types;
        c.option<ConfigOptionFloats>("extruder_nozzle_diameter", true)->values          = diams;
        c.option<ConfigOptionInts>("extruder_nozzle_slot", true)->values                = ids;
    } else if (r < 55) {
        // per-slot keys that do not fit each other
        c.option<ConfigOptionInts>("extruder_nozzle_count", true)->values = {1, chance(50) ? -1 : 2};
        c.option<ConfigOptionEnumsGeneric>("extruder_nozzle_volume_type", true)->values = {0};
        if (chance(50))
            c.option<ConfigOptionFloats>("extruder_nozzle_diameter", true)->values = {0.4};
    }
    static const char *const k_stats[] = {"Standard#1", "Standard#2|High Flow#1", "High Flow#3", "", "Bogus#2", "Standard", "TPU285#1|E3D High Flow#2",
                                          "Hybrid#1", "Standard#0", "High Flow#x", "Standard#1|Standard#2"};
    for (const char *key : {"extruder_nozzle_stats_new", "extruder_nozzle_stats"}) {
        if (!chance(40))
            continue;
        const int                n = chance(80) ? extruders : rint_(4);
        std::vector<std::string> values;
        for (int e = 0; e < n; ++e) values.push_back(pick(k_stats));
        c.option<ConfigOptionStrings>(key, true)->values = values;
    }
    if (chance(60)) {
        const int        n = chance(85) ? extruders : rint_(4);
        std::vector<int> values;
        for (int e = 0; e < n; ++e) values.push_back(rnd_type());
        c.option<ConfigOptionEnumsGeneric>("nozzle_volume_type", true)->values = values;
    }
    if (chance(20))
        set_bool(c, "multi_nozzle_diameter_mode", chance(50));
    return c;
}

static NozzleSlot rnd_slot()
{
    NozzleSlot slot;
    slot.diameter    = pick(k_diameters);
    slot.volume_type = NozzleVolumeType(rnd_type());
    slot.status      = chance(75) ? NozzleSlotStatus::Normal : NozzleSlotStatus(rint_(4));
    if (chance(10))
        slot = NozzleSlot();
    return slot;
}

static ExtruderSlots rnd_slots(int cap)
{
    ExtruderSlots slots;
    const int     n = chance(70) ? cap : rint_(cap + 3);
    for (int i = 0; i < n; ++i) slots.push_back(rnd_slot());
    return slots;
}

// Slots as tokens: their number, then diameter, volume type and status of each.
static std::string slots_tokens(const ExtruderSlots &slots)
{
    std::ostringstream o;
    o << slots.size();
    for (const NozzleSlot &slot : slots) o << ' ' << tok(slot.diameter) << ' ' << int(slot.volume_type) << ' ' << int(slot.status);
    return o.str();
}

// ---- state text ----

static std::string counts_text(const std::map<NozzleVolumeType, int> &counts)
{
    std::ostringstream o;
    bool               first = true;
    for (const auto &[type, count] : counts) {
        o << (first ? "" : ",") << int(type) << ':' << count;
        first = false;
    }
    return o.str();
}

// Everything observable from outside: the log records since the last call, the mirror, the history
// and every read of the inventory, for the extruders -1 .. extruder_count().
static std::string state_text(const NozzleInventory &inv, const DynamicPrintConfig &mirror, const AppConfig *history)
{
    std::ostringstream o;
    for (const LogRecord &record : g_log_records) o << "LOG " << record.level << ": " << record.message << '\n';
    g_log_records.clear();
    o << "MIRROR " << cfg_text(mirror) << '\n';
    o << "HIST";
    if (!history)
        o << " none";
    else
        for (const auto &[section, data] : history->m_storage)
            for (const auto &[key, value] : data) o << ' ' << section << '/' << key << '=' << value << ';';
    o << '\n';
    const int n = inv.extruder_count();
    o << "EC " << n << '\n';
    for (int eid = -1; eid <= n; ++eid) {
        o << 'E' << eid << " slots=";
        const ExtruderSlots &slots = inv.slots(eid);
        for (size_t i = 0; i < slots.size(); ++i)
            o << (i ? "|" : "") << slots[i].diameter << '/' << int(slots[i].volume_type) << '/' << int(slots[i].status);
        o << " counts=" << counts_text(inv.counts(eid));
        o << " c04=" << counts_text(inv.counts(eid, std::string("0.4")));
        o << " c06=" << counts_text(inv.counts(eid, std::string("0.6")));
        o << " c040=" << counts_text(inv.counts(eid, std::string("0.40")));
        o << " cempty=" << counts_text(inv.counts(eid, std::string()));
        o << " n=" << inv.count(eid);
        o << " nS=" << inv.count(eid, nvtStandard);
        o << " nH=" << inv.count(eid, nvtHybrid);
        o << " nHF04=" << inv.count(eid, nvtHighFlow, std::string("0.4"));
        o << " vt=" << int(inv.volume_type(eid)) << '\n';
    }
    return o.str();
}

static bool g_full = false;

static void emit(const std::string &op, const std::string &state)
{
    if (g_full) {
        std::printf("%s\n%s", op.c_str(), state.c_str());
        return;
    }
    uint64_t h = 0xcbf29ce484222325ull;
    for (unsigned char ch : state) {
        h ^= ch;
        h *= 0x100000001b3ull;
    }
    std::printf("%s -> %016llx\n", op.c_str(), (unsigned long long) h);
}

int main(int argc, char **argv)
{
    g_full = argc > 1 && std::strcmp(argv[1], "full") == 0;
    static const char *const k_history_types[] = {"Standard", "High Flow,Standard", "Standard|High Flow| ,TPU285", "Bogus", "", " Standard ",
                                                  "High Flow|,E3D High Flow|Standard,Standard", "Hybrid", "E3D High Flow", "TPU High Flow|Standard|"};
    static const char *const k_history_slots[] = {"0.4 Standard", "0.4 Standard|0.6 High Flow,0.4 Standard", "0.4 Bogus", "Standard", "0 Standard",
                                                  "0.40 E3D High Flow|,0.2 High Flow", "0.6 Standard|0.6 Standard,|", "", "0.8 TPU285,0.4 High Flow,0.4 Standard",
                                                  " 0.4 High Flow | 0.6 Standard ", "0.4  Standard", "0.2 E3D High Flow"};

    for (int case_id = 0; case_id < 255; ++case_id) {
        std::printf("CASE\n");
        Printer printer;
        make_printer(printer);
        fix_base(printer);

        DynamicPrintConfig mirror;
        if (chance(40))
            set_bool(mirror, "multi_nozzle_diameter_mode", chance(60));
        AppConfig history;
        bool      has_history = chance(75);
        NozzleInventory inv(mirror);
        g_log_records.clear();
        emit("mirror_init " + cfg_tokens(mirror), state_text(inv, mirror, nullptr));

        const int ops = 5 + rint_(6);
        for (int op_id = 0; op_id < ops; ++op_id) {
            std::ostringstream op;
            const int          r = op_id == 0 ? 0 : (op_id == 1 ? 6 : rint_(100));
            const std::vector<int> slot_counts = printer_slot_counts(printer.presets.edited.config);
            const int              extruders   = int(slot_counts.size());
            const int              eid         = chance(90) ? rint_(extruders) : rint_(6) - 2;
            const int              cap         = eid >= 0 && eid < extruders ? slot_counts[size_t(eid)] : 1;
            if (r < 3) {
                // history: on or off, with a few saved values
                has_history = op_id == 0 ? has_history : !has_history;
                op << "set_history " << (has_history ? 1 : 0);
                history.m_storage.clear();
                std::vector<std::pair<std::string, std::string>> types, slots;
                if (chance(70)) {
                    types.emplace_back(printer.presets.base ? printer.presets.base->name : printer.presets.selected.name, pick(k_history_types));
                    if (chance(30))
                        types.emplace_back("Base 0", pick(k_history_types));
                }
                if (chance(60)) {
                    slots.emplace_back(printer.presets.edited.config.opt_string("printer_model"), pick(k_history_slots));
                    if (chance(30))
                        slots.emplace_back("O1D", pick(k_history_slots));
                }
                op << ' ' << types.size();
                for (const auto &[key, value] : types) {
                    op << ' ' << tok(key) << ' ' << tok(value);
                    history.m_storage["nozzle_volume_types"][key] = value;
                }
                op << ' ' << slots.size();
                for (const auto &[key, value] : slots) {
                    op << ' ' << tok(key) << ' ' << tok(value);
                    history.m_storage["nozzle_slots"][key] = value;
                }
                inv.set_history(has_history ? &history : nullptr);
            } else if (r < 18) {
                if (op_id > 1 && chance(60)) {
                    if (chance(50)) {
                        // only the variant changes
                        set_string(printer.presets.edited.config, "printer_variant", pick(k_variants));
                    } else {
                        make_printer(printer);
                        fix_base(printer);
                    }
                }
                op << "select_printer " << printer_tokens(printer);
                inv.select_printer(printer.presets);
            } else if (r < 32) {
                const DynamicPrintConfig file_keys = make_file_keys(printer);
                op << "open_project " << printer_tokens(printer) << ' ' << cfg_tokens(file_keys);
                inv.open_project(printer.presets, file_keys);
            } else if (r < 37) {
                op << "close_project";
                inv.close_project();
            } else if (r < 55) {
                const std::string model = chance(80) ? printer.presets.edited.config.opt_string("printer_model") : std::string(pick(k_models));
                const int         n     = chance(85) ? extruders : rint_(5);
                std::vector<ExtruderSlots> layout;
                for (int e = 0; e < n; ++e) layout.push_back(rnd_slots(e < extruders ? slot_counts[size_t(e)] : 1));
                op << "sync_machine " << tok(model) << ' ' << layout.size();
                for (const ExtruderSlots &slots : layout) op << ' ' << slots_tokens(slots);
                inv.sync_machine(model, layout);
            } else if (r < 67) {
                const ExtruderSlots slots = rnd_slots(cap);
                op << "user_set_slots " << eid << ' ' << slots_tokens(slots);
                inv.user_set_slots(eid, slots);
            } else if (r < 77) {
                std::map<NozzleVolumeType, int> counts;
                const int                       n = rint_(4);
                for (int i = 0; i < n; ++i) counts[NozzleVolumeType(rnd_type())] = rint_(cap + 2) - (chance(10) ? 1 : 0);
                op << "user_set_counts " << eid << ' ' << counts.size();
                for (const auto &[type, count] : counts) op << ' ' << int(type) << ' ' << count;
                inv.user_set_counts(eid, counts);
            } else if (r < 84) {
                const int type = rnd_type();
                op << "user_set_volume_type " << eid << ' ' << type;
                inv.user_set_volume_type(eid, NozzleVolumeType(type));
            } else if (r < 90) {
                const std::string diameter = pick(k_diameters);
                op << "user_unify_diameter " << tok(diameter);
                inv.user_unify_diameter(diameter);
            } else if (r < 95) {
                // a bulk copy into the mirror replaces the nozzle keys, then the keys are written again
                mirror = make_file_keys(printer);
                op << "mirror_set " << cfg_tokens(mirror);
                emit(op.str(), state_text(inv, mirror, has_history ? &history : nullptr));
                inv.refresh_mirror();
                emit("refresh_mirror", state_text(inv, mirror, has_history ? &history : nullptr));
                continue;
            } else {
                // a scratch copy: its own mirror, no history
                DynamicPrintConfig scratch_mirror;
                if (chance(50))
                    set_bool(scratch_mirror, "multi_nozzle_diameter_mode", chance(50));
                op << "assign " << cfg_tokens(scratch_mirror);
                NozzleInventory scratch(scratch_mirror);
                scratch = inv;
                scratch.refresh_mirror();
                emit(op.str(), state_text(scratch, scratch_mirror, nullptr));
                continue;
            }
            emit(op.str(), state_text(inv, mirror, has_history ? &history : nullptr));
        }
    }
    return 0;
}
