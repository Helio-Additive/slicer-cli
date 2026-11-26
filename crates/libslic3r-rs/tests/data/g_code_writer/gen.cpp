// Generator of the vectors of tests/g_code_writer_native_parity.rs: libslic3r's GCodeWriter.hpp /
// GCodeWriter.cpp and Extruder.hpp / Extruder.cpp at the pin (references/BambuStudio/src/libslic3r;
// the override layer has no copy of them).
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
//   ./gen | gzip -9n > $DATA/g_code_writer.txt.gz
//   ./gen --axis-modes > $DATA/axis_modes.jsonl
//   ./gen --shared-axes > $DATA/shared_axes.txt
//
// Made on macOS arm64 with Apple clang (-ffp-contract=on). The private members are reached by
// including the libslic3r headers with `private` and `protected` defined as `public`.
//
// Every case is a configuration (option values set from text over the defaults of every option,
// as the Rust test sets them), applied to a new writer with apply_print_config, the extruders of
// set_extruders, then a program of writer calls drawn from a seeded generator. The lines:
//
//   case <n> (<key> <hex value>)* <n ids> <id>*      the configuration and the extruders
//   cfg <derived state>                              what apply_print_config / set_extruders set
//   <op> <arguments> => <hex output> <state>         one call, what it returned, the state after
//
// Doubles are 16 hexadecimal digits of their bits, floats 8, texts the hexadecimal of their bytes
// ("-" when empty).

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>
#define private public
#define protected public
#include "libslic3r/GCodeWriter.hpp"
#include "libslic3r/Extruder.hpp"
#include "libslic3r/PrintConfig.hpp"
#undef private
#undef protected

using namespace Slic3r;

static std::string hD(double v)
{
    uint64_t u;
    std::memcpy(&u, &v, 8);
    char b[32];
    std::snprintf(b, sizeof(b), "%016llx", (unsigned long long) u);
    return b;
}

static std::string hF(float v)
{
    uint32_t u;
    std::memcpy(&u, &v, 4);
    char b[16];
    std::snprintf(b, sizeof(b), "%08x", u);
    return b;
}

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

struct Rng
{
    uint64_t s;
    uint32_t next()
    {
        s ^= s << 13;
        s ^= s >> 7;
        s ^= s << 17;
        return uint32_t(s >> 16);
    }
    double u01() { return double(next() & 0xFFFFFF) / double(0x1000000); }
    double uni(double a, double b) { return a + (b - a) * u01(); }
    int    pick(int n) { return int(next() % uint32_t(n)); }
    bool   coin() { return (next() & 1) != 0; }
};

using KVs = std::vector<std::pair<std::string, std::string>>;

struct Case
{
    KVs                       kv;
    std::vector<unsigned int> ids;
    uint64_t                  seed;
    int                       ops;
    bool                      bbl;
};

static std::string state(GCodeWriter &w)
{
    std::string s = hD(w.m_pos.x()) + " " + hD(w.m_pos.y()) + " " + hD(w.m_pos.z()) + " " + hD(w.m_lifted) + " " +
                    hD(w.m_to_lift) + " " + std::to_string(int(w.m_to_lift_type)) + " " + std::to_string(w.m_curr_extruder_id) + " " +
                    std::to_string(w.m_current_process_config_idx) + " " + std::to_string(w.m_last_acceleration) + " " + hD(w.m_last_jerk) +
                    " " + std::to_string(int(w.m_is_current_pos_clear)) + " " + hD(w.m_current_speed) + " " +
                    std::to_string(w.m_last_bed_temperature) + " " + std::to_string(int(w.m_last_bed_temperature_reached));
    // m_retracted of a shared extruder is never set (Extruder::reset() clears it only when the
    // extruder is not shared, the constructor leaves it uninitialized): it is not compared.
    for (const Extruder &e : w.m_filament_extruders)
        s += " " + hD(e.E()) + " " + hD(e.used_filament()) + " " + hD(e.extruded_volume()) + " " +
             (e.is_share_extruder() ? std::string("-") : hD(e.get_single_retracted_length())) + " " + hD(e.get_share_retracted_length());
    return s;
}

static std::string cfg_line(GCodeWriter &w)
{
    std::string s = "cfg " + std::to_string(w.m_max_acceleration) + " " + hD(w.m_max_jerk) + " " +
                    std::to_string(int(w.m_single_extruder_multi_material)) + " " + std::to_string(int(w.multiple_extruders)) + " " +
                    std::to_string(int(w.m_bed_bbox_valid)) + " " + hD(w.m_bed_min.x()) + " " + hD(w.m_bed_min.y()) + " " +
                    hD(w.m_bed_max.x()) + " " + hD(w.m_bed_max.y()) + " " + std::to_string(w.m_extruder_bed_valid.size());
    for (size_t i = 0; i < w.m_extruder_bed_valid.size(); ++i)
        s += " " + std::to_string(int(w.m_extruder_bed_valid[i])) + " " + hD(w.m_extruder_bed_min[i].x()) + " " +
             hD(w.m_extruder_bed_min[i].y()) + " " + hD(w.m_extruder_bed_max[i].x()) + " " + hD(w.m_extruder_bed_max[i].y());
    s += " " + std::to_string(int(w.config.gcode_flavor.value)) + " " + std::to_string(w.m_filament_extruders.size());
    for (const Extruder &e : w.m_filament_extruders)
        s += " " + std::to_string(e.id()) + " " + std::to_string(e.extruder_id()) + " " + std::to_string(int(e.m_exclusive_nozzle)) +
             " " + hD(e.e_per_mm3()) + " " + hD(e.retraction_length()) + " " + hD(e.retract_length_toolchange()) + " " +
             hD(e.retract_restart_extra_toolchange()) + " " + std::to_string(e.retract_speed()) + " " +
             std::to_string(e.deretract_speed()) + " " + hD(e.retract_before_wipe()) + " " + hD(e.retract_restart_extra());
    return s;
}

static const char *const comments[] = {"", "travel", "move to first layer point", "wipe and retract"};
static const char *const markers[]  = {"", ";_EXTRUDE_SET_SPEED", ";_WIPE"};

static void emit(const std::string &head, const std::string &out, GCodeWriter &w)
{
    std::printf("%s => %s %s\n", head.c_str(), hex(out).c_str(), state(w).c_str());
}

static double coord(Rng &r)
{
    switch (r.pick(4)) {
    case 0: return r.uni(-5., 260.);
    case 1: return double(r.pick(2600)) * 0.1;           // on a 0.1 grid
    case 2: return double(r.pick(260000)) * 0.001 + 0.0005; // at the rounding edge of 3 digits
    default: return r.uni(100., 160.);
    }
}

static double zval(Rng &r)
{
    switch (r.pick(3)) {
    case 0: return 0.2 + 0.2 * r.pick(40);
    case 1: return r.uni(0., 12.);
    default: return 0.08 * r.pick(100) + 0.04;
    }
}

static void run(const Case &c)
{
    DynamicPrintConfig dc = DynamicPrintConfig::full_print_config();
    std::string head = "case " + std::to_string(c.kv.size());
    for (const auto &p : c.kv) {
        dc.set_deserialize_strict(p.first, p.second);
        head += " " + p.first + " " + hex(p.second);
    }
    head += " " + std::to_string(c.ids.size());
    for (unsigned int id : c.ids)
        head += " " + std::to_string(id);
    head += " " + std::to_string(int(c.bbl));
    std::printf("%s\n", head.c_str());

    PrintConfig pc;
    pc.apply(dc, true);
    GCodeWriter w;
    w.m_last_bed_temperature = 0; // left uninitialized by the constructor
    w.m_last_additional_fan_speed = 0;
    w.apply_print_config(pc);
    w.set_extruders(c.ids);
    w.set_is_bbl_printer(c.bbl);
    std::printf("%s\n", cfg_line(w).c_str());

    Rng r{c.seed};
    // The first calls: the preamble before any filament, then a filament.
    emit("pre", w.preamble(), w);
    {
        unsigned int f = c.ids[r.pick(int(c.ids.size()))];
        unsigned int n = unsigned(r.pick(2));
        if (r.coin()) {
            w.init_extruder(f, n);
            emit("init " + std::to_string(f) + " " + std::to_string(n), "", w);
        } else {
            std::string o = w.toolchange(f, n);
            emit("tc " + std::to_string(f) + " " + std::to_string(n), o, w);
        }
    }
    for (int k = 0; k < c.ops; ++k) {
        int op = r.pick(36);
        switch (op) {
        case 0: {
            unsigned t = unsigned(r.pick(300));
            bool wait = r.coin();
            int tool = r.pick(3) == 0 ? -1 : int(c.ids[r.pick(int(c.ids.size()))]);
            int ct = r.pick(2);
            std::string o = w.set_temperature(t, wait, tool, ct ? TemperatureCommandType::LayerChange : TemperatureCommandType::Default);
            emit("temp " + std::to_string(t) + " " + std::to_string(int(wait)) + " " + std::to_string(tool) + " " + std::to_string(ct), o, w);
            break;
        }
        case 1: {
            int t = r.pick(4) == 0 ? w.m_last_bed_temperature : r.pick(120);
            bool wait = r.coin();
            std::string o = w.set_bed_temperature(t, wait);
            emit("bed " + std::to_string(t) + " " + std::to_string(int(wait)), o, w);
            break;
        }
        case 2: {
            int t = r.pick(80);
            bool wait = r.coin();
            std::string o = w.set_chamber_temperature(t, wait);
            emit("chamber " + std::to_string(t) + " " + std::to_string(int(wait)), o, w);
            break;
        }
        case 3: {
            unsigned a = r.pick(5) == 0 ? 0u : unsigned(500 * r.pick(40) + r.pick(7));
            w.set_acceleration(a);
            emit("acc " + std::to_string(a), "", w);
            break;
        }
        case 4: {
            int which = r.pick(3);
            // set_travel_acceleration(bool) indexes the travel vectors by the extruder unchecked: they
            // are empty or hold both extruders.
            int n = which == 1 ? r.pick(3) : 2 * r.pick(2);
            std::vector<unsigned int> v;
            std::string h = which == 0 ? "tacc" : which == 1 ? "tsacc" : "flacc";
            h += " " + std::to_string(n);
            for (int i = 0; i < n; ++i) {
                v.push_back(r.pick(4) == 0 ? 0u : unsigned(1000 * r.pick(20) + r.pick(3)));
                h += " " + std::to_string(v.back());
            }
            if (which == 0)
                w.set_travel_acceleration(v);
            else if (which == 1)
                w.set_travel_short_acceleration(v);
            else
                w.set_first_layer_travel_acceleration(v);
            emit(h, "", w);
            break;
        }
        case 5: {
            bool b = r.coin();
            w.set_first_layer(b);
            emit("first " + std::to_string(int(b)), "", w);
            break;
        }
        case 6: {
            w.reset_last_acceleration();
            emit("resetacc", "", w);
            break;
        }
        case 7: {
            double pa = r.pick(5) == 0 ? -0.01 : r.uni(0., 0.2);
            bool b = r.coin();
            std::string o = w.set_pressure_advance(pa, b);
            emit("pa " + hD(pa) + " " + std::to_string(int(b)), o, w);
            break;
        }
        case 8: {
            double j = r.pick(6) == 0 ? w.m_last_jerk : r.pick(6) == 0 ? 0.005 : r.uni(1., 25.);
            std::string o = w.set_jerk_xy(j);
            emit("jerk " + hD(j), o, w);
            break;
        }
        case 9: {
            bool f = r.coin();
            std::string o = w.reset_e(f);
            emit("reset_e " + std::to_string(int(f)), o, w);
            break;
        }
        case 10: {
            unsigned tot = unsigned(1 + r.pick(500));
            unsigned num = unsigned(r.pick(int(tot) + 1));
            bool a = r.coin();
            std::string o = w.update_progress(num, tot, a);
            emit("prog " + std::to_string(num) + " " + std::to_string(tot) + " " + std::to_string(int(a)), o, w);
            break;
        }
        case 11: {
            emit("tcp", w.toolchange_prefix(), w);
            break;
        }
        case 12: {
            unsigned int f = c.ids[r.pick(int(c.ids.size()))];
            unsigned int n = unsigned(r.pick(2));
            if (r.coin()) {
                std::string o = w.set_extruder(f, n);
                emit("setext " + std::to_string(f) + " " + std::to_string(n), o, w);
            } else {
                std::string o = w.toolchange(f, n);
                emit("tc " + std::to_string(f) + " " + std::to_string(n), o, w);
            }
            break;
        }
        case 13: {
            double F = r.pick(3) == 0 ? double(60 * r.pick(300) + 60) : r.uni(100., 30000.);
            int ci = r.pick(4), mi = r.pick(3);
            std::string o = w.set_speed(F, comments[ci], markers[mi]);
            emit("speed " + hD(F) + " " + std::to_string(ci) + " " + std::to_string(mi), o, w);
            break;
        }
        case 14:
        case 15: {
            Vec2d p(coord(r), coord(r));
            int ci = r.pick(4);
            bool s = r.coin();
            std::string o = s ? w.travel_to_xy(p, comments[ci], true) : (r.coin() ? w.travel_to_xy(p, comments[ci], false) : w.travel_to_xy(p, comments[ci]));
            emit("txy " + hD(p.x()) + " " + hD(p.y()) + " " + std::to_string(ci) + " " + std::to_string(int(s)), o, w);
            break;
        }
        case 16:
        case 17:
        case 18: {
            Vec3d p(coord(r), coord(r), r.pick(4) == 0 ? w.m_pos.z() : zval(r));
            if (r.pick(8) == 0) {
                p.x() = w.m_pos.x();
                p.y() = w.m_pos.y();
            }
            int ci = r.pick(4);
            bool s = r.coin();
            std::string o = s ? w.travel_to_xyz(p, comments[ci], true) : w.travel_to_xyz(p, comments[ci]);
            emit("txyz " + hD(p.x()) + " " + hD(p.y()) + " " + hD(p.z()) + " " + std::to_string(ci) + " " + std::to_string(int(s)), o, w);
            break;
        }
        case 19: {
            double z = r.pick(3) == 0 ? w.m_pos.z() - w.m_lifted * r.u01() : zval(r);
            int ci = r.pick(4);
            std::string o = w.travel_to_z(z, comments[ci]);
            emit("tz " + hD(z) + " " + std::to_string(ci), o, w);
            break;
        }
        case 20:
        case 21: {
            int t = r.pick(3);
            bool sv = r.pick(4) == 0, tc = r.coin();
            std::string o = w.lazy_lift(LiftType(t), sv, tc);
            emit("lazy " + std::to_string(t) + " " + std::to_string(int(sv)) + " " + std::to_string(int(tc)), o, w);
            break;
        }
        case 22: {
            int t = r.pick(3);
            bool tc = r.coin();
            std::string o = w.eager_lift(LiftType(t), tc);
            emit("eager " + std::to_string(t) + " " + std::to_string(int(tc)), o, w);
            break;
        }
        case 23:
        case 24:
        case 25: {
            Vec2d p(coord(r), coord(r));
            double de = r.pick(6) == 0 ? -r.uni(0., 1.) : r.uni(0., 2.);
            bool force = r.pick(6) == 0;
            int ci = r.pick(4);
            std::string o = w.extrude_to_xy(p, de, comments[ci], force);
            emit("exy " + hD(p.x()) + " " + hD(p.y()) + " " + hD(de) + " " + std::to_string(ci) + " " + std::to_string(int(force)), o, w);
            break;
        }
        case 26: {
            Vec2d p(coord(r), coord(r)), ij(r.uni(-20., 20.), r.uni(-20., 20.));
            double de = r.uni(0., 2.);
            bool ccw = r.coin(), force = r.pick(6) == 0;
            int ci = r.pick(4);
            std::string o = w.extrude_arc_to_xy(p, ij, de, ccw, comments[ci], force);
            emit("earc " + hD(p.x()) + " " + hD(p.y()) + " " + hD(ij.x()) + " " + hD(ij.y()) + " " + hD(de) + " " + std::to_string(int(ccw)) +
                     " " + std::to_string(ci) + " " + std::to_string(int(force)),
                 o, w);
            break;
        }
        case 27: {
            Vec3d p(coord(r), coord(r), zval(r));
            double de = r.uni(0., 2.);
            bool force = r.pick(6) == 0;
            int ci = r.pick(4);
            std::string o = w.extrude_to_xyz(p, de, comments[ci], force);
            emit("exyz " + hD(p.x()) + " " + hD(p.y()) + " " + hD(p.z()) + " " + hD(de) + " " + std::to_string(ci) + " " + std::to_string(int(force)), o, w);
            break;
        }
        case 28:
        case 29: {
            bool bw = r.coin();
            bool tc = r.pick(3) == 0;
            std::string o = tc ? w.retract_for_toolchange(bw) : w.retract(bw);
            emit(std::string(tc ? "retracttc " : "retract ") + std::to_string(int(bw)), o, w);
            break;
        }
        case 30: {
            float extra = r.pick(3) == 0 ? float(r.uni(0., 0.5)) : 0.f;
            std::string o = w.unretract(extra);
            emit("unretract " + hF(extra), o, w);
            break;
        }
        case 31: {
            emit("unlift", w.unlift(), w);
            break;
        }
        case 32: {
            unsigned s = r.pick(4) == 0 ? 0u : unsigned(r.pick(101));
            int which = r.pick(4);
            if (which == 0) {
                emit("fan " + std::to_string(s), w.set_fan(s), w);
            } else if (which == 1) {
                int fl = r.pick(13);
                emit("fanf " + std::to_string(fl) + " " + std::to_string(s), GCodeWriter::set_fan(GCodeFlavor(fl), s), w);
            } else if (which == 2) {
                int idx = r.pick(5);
                emit("afan " + std::to_string(idx) + " " + std::to_string(s), GCodeWriter::set_additional_fan(idx, s), w);
            } else {
                bool eol = r.coin();
                emit("exfan " + std::to_string(int(s)) + " " + std::to_string(int(eol)), GCodeWriter::set_exhaust_fan(int(s), eol), w);
            }
            break;
        }
        case 33: {
            int which = r.pick(5);
            if (which == 0) {
                double x = r.pick(2) == 0 ? 0. : r.uni(-5., 5.), y = r.pick(2) == 0 ? 0. : r.uni(-5., 5.);
                w.set_xy_offset(x, y);
                emit("offset " + hD(x) + " " + hD(y), "", w);
            } else if (which == 1) {
                bool b = r.pick(3) != 0;
                w.set_current_position_clear(b);
                emit("clear " + std::to_string(int(b)), "", w);
            } else if (which == 2) {
                bool b = r.coin();
                w.set_avoid_z_descent_travel(b);
                emit("avoid " + std::to_string(int(b)), "", w);
            } else if (which == 3) {
                bool b = r.coin();
                w.set_is_bbl_printer(b);
                emit("bbl " + std::to_string(int(b)), "", w);
            } else {
                double z = zval(r);
                emit("wmz " + hD(z) + " " + std::to_string(int(w.will_move_z(z))), "", w);
            }
            break;
        }
        case 34: {
            unsigned int f = c.ids[r.pick(int(c.ids.size()))];
            emit("retlen " + std::to_string(f) + " " + hD(w.get_extruder_retracted_length(int(f))), "", w);
            break;
        }
        default: {
            int which = r.pick(3);
            std::string g = "G1 X1 ; ";
            if (which == 0) {
                w.set_object_start_str("M624 AQAAAAAAAAA=\n");
                w.set_object_end_str("M625\n");
                w.add_object_change_labels(g);
            } else if (which == 1) {
                w.set_object_start_str("M624 AgAAAAAAAAA=\n");
                w.add_object_start_labels(g);
            } else {
                w.add_object_end_labels(g);
            }
            emit("labels " + std::to_string(which) + " " + std::to_string(int(w.empty_object_start_str())) + " " +
                     std::to_string(int(w.empty_object_end_str())),
                 g, w);
            break;
        }
        }
    }
    emit("post", w.postamble(), w);
}

static void axis_modes()
{
    const char *flavors[] = {"marlin", "marlin2", "reprap", "reprapfirmware", "repetier", "teacup", "smoothie",
                             "klipper", "makerware", "sailfish", "mach3", "machinekit", "no-extrusion"};
    for (bool initial : {false, true}) {
        for (const char *input : {"0", "1", "true", "false", " 0", "0 ", "", "2", "on", "off"}) {
            for (const char *flavor : flavors) {
                ConfigOptionBool value(initial);
                const bool accepted = value.deserialize(input, false);
                DynamicPrintConfig config;
                config.set_deserialize_strict("gcode_flavor", flavor);
                config.set_key_value("use_relative_e_distances", value.clone());
                PrintConfig print;
                print.apply(config, true);
                GCodeWriter writer;
                writer.apply_print_config(print);
                nlohmann::json row = {{"initial", initial}, {"input", input}, {"flavor", flavor},
                                      {"accepted", accepted}, {"mode", writer.config.use_relative_e_distances.value},
                                      {"preamble", writer.preamble()}};
                std::cout << row.dump() << '\n';
            }
        }
    }
}

// Extruder.cpp 5-6,38-48,60-146 and Extruder.hpp 19-40 at 56a57d0.
// All worker calls are joined before the parent's next read.
static void shared_axes()
{
    for (bool relative : {false, true}) {
        for (bool hotend : {false, true}) {
            GCodeConfig config;
            config.use_relative_e_distances.value = relative;
            config.handle_hotend_as_extruder.value = hotend;
            config.filament_map.values = {1, 1, 2};
            config.filament_nozzle_map.values = {4, 3, 4};
            Extruder a(0, &config, true), b(1, &config, true), c(2, &config, true);
            Extruder separate(0, &config, false);
            auto record = [&](const char *op, double result) {
                std::printf("%d %d %s %s", int(relative), int(hotend), op, hD(result).c_str());
                for (const Extruder *e : {&a, &b, &c, &separate})
                    std::printf(" %s %s %s", hD(e->E()).c_str(), hD(e->get_share_retracted_length()).c_str(), hD(e->used_filament()).c_str());
                std::puts("");
            };
            record("init", 0.);
            record("a-extrude", a.extrude(3.25));
            record("b-negative", b.extrude(-0.5));
            record("c-retract", c.retract(1.25, 0.25));
            record("a-shorter", a.retract(0.125, 0.625));
            record("b-equal", b.retract(0.5, 0.375));
            record("b-repay", b.unretract());
            record("a-longer", a.retract(2., 0.375));
            record("c-repay", c.unretract());
            record("a-repay", a.unretract());
            record("separate", separate.extrude(8.));
            separate.reset_E();
            record("reset-e", 0.);
            std::thread other([&] {
                Extruder peer(1, &config, true);
                peer.extrude(7.);
                peer.retract(1.75, 0.125);
            });
            other.join();
            record("worker", 0.);
            record("b-worker-repay", b.unretract());
            a.reset();
            record("reset", 0.);
        }
    }
}

int main(int argc, char **argv)
{
    if (argc == 2 && std::string(argv[1]) == "--shared-axes") {
        shared_axes();
        return 0;
    }
    if (argc == 2 && std::string(argv[1]) == "--axis-modes") {
        axis_modes();
        return 0;
    }

    // The machines: a flavor each, and the options the writer and the extruders read.
    const char *flavors[] = {"marlin", "klipper", "reprap", "reprapfirmware", "repetier", "teacup", "makerware",
                             "marlin2", "sailfish", "smoothie", "mach3", "machinekit", "no-extrusion"};
    std::vector<Case> cases;
    uint64_t seed = 0x9E3779B97F4A7C15ull;
    for (int fi = 0; fi < 13; ++fi) {
        for (int v = 0; v < 4; ++v) {
            Case c;
            c.kv.push_back({"gcode_flavor", flavors[fi]});
            c.kv.push_back({"use_relative_e_distances", v & 1 ? "1" : "0"});
            c.kv.push_back({"use_firmware_retraction", v == 2 ? "1" : "0"});
            c.kv.push_back({"z_hop", v == 3 ? "0.4,0.6" : "0.4"});
            c.kv.push_back({"retract_lift_above", "0"});
            c.kv.push_back({"retract_lift_below", v == 1 ? "6" : v == 0 ? "0" : "400"});
            c.kv.push_back({"travel_speed", v == 2 ? "250.5" : "300"});
            c.kv.push_back({"travel_speed_z", v == 1 ? "0" : "12.5"});
            c.kv.push_back({"retraction_length", v == 3 ? "0.8,1.2" : "0.8"});
            c.kv.push_back({"retraction_speed", "35.5"});
            c.kv.push_back({"deretraction_speed", v == 0 ? "0" : "27.4"});
            c.kv.push_back({"retract_restart_extra", v == 2 ? "0.1" : "0"});
            c.kv.push_back({"retract_before_wipe", "70%"});
            c.kv.push_back({"accel_to_decel_enable", v & 1 ? "1" : "0"});
            c.kv.push_back({"accel_to_decel_factor", v == 1 ? "33.3%" : "50%"});
            c.kv.push_back({"machine_max_acceleration_extruding", v == 0 ? "5000.5,4000" : "20000,10000"});
            c.kv.push_back({"machine_max_jerk_x", v == 2 ? "9.5" : "20"});
            c.kv.push_back({"machine_max_jerk_y", v == 2 ? "12" : "8.5"});
            c.kv.push_back({"prime_tower_lift_height", v == 3 ? "2.5" : "0"});
            c.kv.push_back({"prime_tower_lift_speed", v == 3 ? "40" : "0"});
            c.kv.push_back({"printable_area", v == 1 ? "0x0,256x0,256x256,0x256" : "0x0,180x0,180x180,0x180"});
            if (v == 2) {
                c.kv.push_back({"filament_map", "1,2,1"});
                c.kv.push_back({"extruder_printable_area", "0x0,200x0,200x256,0x256#25x0,256x0,256x256,25x256"});
                c.ids = {0, 1, 2};
            } else if (v == 3) {
                c.kv.push_back({"single_extruder_multi_material", "1"});
                c.ids = {2, 0, 3, 1};
            } else if (v == 1) {
                c.kv.push_back({"single_extruder_multi_material", "0"});
                c.ids = {0, 1};
            } else {
                c.ids = {0};
            }
            c.seed = seed;
            seed = seed * 6364136223846793005ull + 1442695040888963407ull;
            c.ops  = 160;
            c.bbl  = (fi + v) % 3 == 0;
            cases.push_back(c);
        }
    }
    // Machines with hotends handled as extruders and a PVA filament on a hotend of its own.
    for (int v = 0; v < 4; ++v) {
        Case c;
        c.kv.push_back({"gcode_flavor", v & 1 ? "klipper" : "marlin2"});
        c.kv.push_back({"handle_hotend_as_extruder", v < 3 ? "1" : "0"});
        c.kv.push_back({"single_extruder_multi_material", v == 2 ? "0" : "1"});
        c.kv.push_back({"filament_nozzle_map", v == 1 ? "0,1,1,2" : "0,1,0,2"});
        c.kv.push_back({"filament_type", "PLA,PVA,PETG,PVA"});
        c.kv.push_back({"filament_retract_length_toolchange", v == 0 ? "nil,8.5,nil,9" : "nil,7,3,nil"});
        c.kv.push_back({"filament_retract_restart_extra_toolchange", v == 0 ? "nil,0.25,nil,nil" : "nil,0.5,0.1,0.2"});
        c.kv.push_back({"retract_length_toolchange", "2"});
        c.kv.push_back({"retract_restart_extra_toolchange", "0.05"});
        c.kv.push_back({"z_hop", "0.2,0.4,0.3,0.5"});
        c.kv.push_back({"retract_lift_below", "300"});
        c.kv.push_back({"retraction_length", "0.6,1,1.5,2"});
        c.kv.push_back({"extruder_printable_area", v == 2 ? "0x0,10x0,10x10,0x10" : ""});
        c.ids  = {0, 1, 2, 3};
        c.seed = seed;
        seed   = seed * 6364136223846793005ull + 1442695040888963407ull;
        c.ops  = 240;
        c.bbl  = v != 1;
        cases.push_back(c);
    }
    for (const Case &c : cases)
        run(c);
    return 0;
}
