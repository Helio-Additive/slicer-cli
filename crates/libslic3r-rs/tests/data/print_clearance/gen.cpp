// Generator of the vectors of tests/print_clearance_native_parity.rs: the clearance checks of Print.cpp at
// the pin (references/BambuStudio/src/libslic3r; the override layer has no copy of it):
// Print::sequential_print_clearance_valid (Print.cpp:777-1122) and the static
// layered_print_cleareance_valid (Print.cpp:1123-1241).
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
//   ./gen > vectors.txt
//
// Print.cpp is compiled into this file (layered_print_cleareance_valid is static), with the private
// members of Print and PrintObject made public to read the state `Print::apply` builds.
//
// It builds small Models in code (cubes with several instances, a rotated instance, tall and short
// objects, exclusion and clumping detection areas, a prime tower) with a full print config of three
// filaments as tests/data/print/gen.cpp does, runs `Print::apply`, then both checks, and prints one
// line per result: the message, its object, the collision polygons, the height polygons, the
// warnings and the ModelInstances' arrange_order the checks set. The Rust test builds the same models,
// assembles the Print as tests/print_native_parity.rs does and has to print the same lines. The
// vectors were made on macOS arm64 with Apple clang (its default -ffp-contract=on) and Eigen 5.0.1.
//
// Line format: tokens separated by one space; a float its 8 hexadecimal digits, an integer decimal. An
// object ID is written relative to the ID of the scenario's model. A newline in a message is "\n".

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <map>
#include <string>
#include <vector>

#define private public
#define protected public
#include "libslic3r/Print.cpp"
#undef protected
#undef private
#include "libslic3r/TriangleMesh.hpp"

using namespace Slic3r;

static size_t g_base = 0;

static std::string hF(float v)
{
    uint32_t u;
    std::memcpy(&u, &v, 4);
    char b[16];
    std::snprintf(b, sizeof(b), "%08x", u);
    return b;
}
static std::string hI(long long v) { return std::to_string(v); }
static std::string hB(bool v) { return v ? "1" : "0"; }
static std::string ID(const ObjectID &id) { return id.id == 0 ? "-1" : hI((long long) id.id - (long long) g_base); }
static void out(const std::string &s) { std::printf("%s\n", s.c_str()); }
static std::string POLY(const Polygon &p)
{
    std::string o = hI((long long) p.points.size());
    for (const Point &pt : p.points) o += " " + hI(pt.x()) + " " + hI(pt.y());
    return o;
}
static std::string POLYS(const Polygons &ps)
{
    std::string o = hI((long long) ps.size());
    for (const Polygon &p : ps) o += " " + POLY(p);
    return o;
}
static std::string MSG(const std::string &s)
{
    std::string o;
    for (char c : s) {
        if (c == '\n') o += "\\n";
        else o += c;
    }
    return o;
}

static indexed_triangle_set box_its(double sx, double sy, double sz, float ox, float oy, float oz)
{
    indexed_triangle_set its = its_make_cube(sx, sy, sz);
    for (Vec3f &v : its.vertices) v += Vec3f(ox, oy, oz);
    return its;
}

static void deser(DynamicPrintConfig &config, const char *key, const char *value)
{
    ConfigSubstitutionContext ctxt{ForwardCompatibilitySubstitutionRule::Disable};
    config.set_deserialize(key, value, ctxt);
}

// The print config of a scenario: the defaults of every option, three filaments, then the keys.
static DynamicPrintConfig base_config(int scenario)
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_num_filaments(3);
    config.option<ConfigOptionStrings>("filament_colour", true)->values = {"#FF0000", "#00FF00", "#0000FF"};
    config.option<ConfigOptionInts>("filament_map", true)->values = {1, 1, 1};
    if (scenario == 0) {
        deser(config, "print_sequence", "by object");
        deser(config, "skirt_loops", "2");
        deser(config, "bed_exclude_area", "0x0,40x0,40x40,0x40");
    }
    if (scenario == 1) {
        deser(config, "print_sequence", "by object");
    }
    if (scenario == 2) {
        deser(config, "enable_prime_tower", "1");
        deser(config, "wall_filament", "2");
        deser(config, "wipe_tower_x", "90");
        deser(config, "wipe_tower_y", "90");
        deser(config, "enable_wrapping_detection", "1");
        deser(config, "wrapping_exclude_area", "300x300,310x300,310x310,300x310");
    }
    if (scenario == 3 || scenario == 7) {
        deser(config, "enable_prime_tower", "1");
        deser(config, "wall_filament", "2");
        deser(config, "bed_exclude_area", "0x0,50x0,50x50,0x50");
        deser(config, "wipe_tower_x", "20");
        deser(config, "wipe_tower_y", "20");
        deser(config, "enable_wrapping_detection", "1");
        deser(config, "wrapping_exclude_area", "300x300,310x300,310x310,300x310");
        if (scenario == 3) deser(config, "wipe_tower_rotation_angle", "30");
    }
    if (scenario == 4) {
        deser(config, "enable_wrapping_detection", "1");
        deser(config, "wrapping_exclude_area", "140x140,160x140,160x160,140x160");
    }
    if (scenario == 5) {
        deser(config, "enable_prime_tower", "1");
        deser(config, "wall_filament", "2");
        deser(config, "enable_wrapping_detection", "1");
        deser(config, "wrapping_exclude_area", "190x190,250x190,250x250,190x250");
        deser(config, "wipe_tower_x", "200");
        deser(config, "wipe_tower_y", "200");
    }
    if (scenario == 6) {
        deser(config, "bed_exclude_area", "0x0,40x0,40x40,0x40,200x0,250x0,250x30,200x30");
        deser(config, "print_sequence", "by object");
        deser(config, "skirt_loops", "1");
        deser(config, "skirt_distance", "6");
    }
    return config;
}

// The models of the scenarios.
static Model build_model(int scenario)
{
    Model model;
    g_base = model.id().id;
    if (scenario == 0 || scenario == 6) {
        ModelObject *o = model.add_object();
        o->name = "a";
        o->add_volume(TriangleMesh(box_its(20, 20, 20, -10.f, -10.f, 0.f)));
        o->add_instance()->set_offset(Vec3d(100., 100., 0.));
        o->add_instance()->set_offset(Vec3d(125., 100., 0.));
        ModelInstance *i2 = o->add_instance();
        i2->set_offset(Vec3d(180., 60., 0.));
        i2->set_rotation(Vec3d(0., 0., 0.5));
        ModelObject *b = model.add_object();
        b->name = "b";
        b->add_volume(TriangleMesh(box_its(10, 10, 60, -5.f, -5.f, 0.f)));
        b->add_instance()->set_offset(Vec3d(200., 200., 0.));
        ModelObject *c = model.add_object();
        c->name = "c";
        c->add_volume(TriangleMesh(box_its(8, 8, 5, -4.f, -4.f, 0.f)));
        c->add_instance()->set_offset(Vec3d(30., 30., 0.));
        ModelObject *d = model.add_object();
        d->name = "d";
        d->add_volume(TriangleMesh(box_its(10, 10, 50, -5.f, -5.f, 0.f)));
        d->add_instance()->set_offset(Vec3d(60., 205., 0.));
    }
    if (scenario == 1) {
        ModelObject *o = model.add_object();
        o->name = "a";
        o->add_volume(TriangleMesh(box_its(20, 20, 10, -10.f, -10.f, 0.f)));
        o->add_instance()->set_offset(Vec3d(50., 50., 0.));
        ModelObject *b = model.add_object();
        b->name = "b";
        b->add_volume(TriangleMesh(box_its(20, 20, 10, -10.f, -10.f, 0.f)));
        b->add_instance()->set_offset(Vec3d(200., 200., 0.));
    }
    if ((scenario >= 2 && scenario <= 5) || scenario == 7) {
        ModelObject *o = model.add_object();
        o->name = "a";
        o->add_volume(TriangleMesh(box_its(20, 20, 20, -10.f, -10.f, 0.f)));
        o->add_volume(TriangleMesh(box_its(6, 6, 6, 12.f, -3.f, 0.f)));
        o->add_instance()->set_offset(Vec3d(100., 100., 0.));
        ModelInstance *i1 = o->add_instance();
        i1->set_offset(Vec3d(60., 170., 0.));
        i1->set_rotation(Vec3d(0., 0., 0.3));
        ModelObject *b = model.add_object();
        b->name = "b";
        b->add_volume(TriangleMesh(box_its(10, 12, 8, -5.f, -6.f, 0.f)));
        b->add_instance()->set_offset(Vec3d(150., 150., 0.));
    }
    return model;
}

// The ModelInstances' arrange_order, minus `base` (sort_object_instances_by_model_order sets it to the
// instance's ObjectID, written relative to the ID of the scenario's model).
static std::string orders(const Model &model, long long base = 0)
{
    std::string o;
    for (const ModelObject *mo : model.objects)
        for (const ModelInstance *mi : mo->instances) o += " " + hI(mi->arrange_order - base);
    return o;
}

// R947: Print::validate (Print.cpp:1413-1844) on the eight scenarios and on variants of scenario 1 (by object,
// two objects far apart) that reach its other checks; `gen validate` prints only these lines.
static void validate_line(const std::string &tag, Print &print)
{
    std::vector<StringObjectException> warnings;
    Polygons polys;
    std::vector<std::pair<Polygon, float>> hp;
    StringObjectException e = print.validate(&warnings, &polys, &hp);
    std::string ws = hI((long long) warnings.size());
    for (auto &w : warnings)
        ws += " " + hB(w.is_warning) + " " + hB(w.object != nullptr) + " " + (w.opt_key.empty() ? std::string("-") : w.opt_key) + " " +
              (w.hypetext.empty() ? std::string("-") : MSG(w.hypetext)) + " " + MSG(w.string);
    std::string ps = hI((long long) e.params.size());
    for (auto &p : e.params) ps += " " + p;
    out(tag + " validate " + hI((long long) e.type) + " " + hB(e.object != nullptr) + " " + (e.opt_key.empty() ? std::string("-") : e.opt_key) +
        " " + hB(e.is_warning) + " params " + ps + " polys " + hI((long long) polys.size()) + " hp " + hI((long long) hp.size()));
    out(tag + " validate_msg " + MSG(e.string));
    out(tag + " validate_warnings " + ws);
}

static int validate_main()
{
    for (int scenario = 0; scenario < 8; ++scenario) {
        Model model = build_model(scenario);
        DynamicPrintConfig config = base_config(scenario);
        Print print;
        // Print.hpp:1328 `Vec3d m_origin;` is not initialised (Eigen); the front end always sets it. Without this the
        // clearance checks read stack garbage (R947: an unrelated edit of this file moved every exclusion check).
        print.set_plate_origin(Vec3d::Zero());
        print.apply(model, config);
        validate_line("c" + std::to_string(scenario), print);
    }
    const std::vector<std::vector<std::pair<std::string, std::string>>> variants = {
        {{"timelapse_type", "1"}},
        {{"enable_wrapping_detection", "1"}},
        {{"layer_height", "0.6"}},
        {{"print_sequence", "by layer"}, {"spiral_mode", "1"}},
        {{"curr_bed_type", "Cool Plate"}, {"cool_plate_temp", "0"}},
        {{"precise_outer_wall", "1"}, {"wall_sequence", "outer wall/inner wall"}},
        {{"sparse_infill_line_width", "2"}},
        {{"enable_support", "1"}, {"support_line_width", "0.1"}},
        {{"initial_layer_print_height", "0.5"}},
        {{"print_sequence", "by layer"}, {"enable_prime_tower", "1"}, {"wall_filament", "2"}, {"use_relative_e_distances", "0"}},
        {{"print_sequence", "by layer"}, {"enable_prime_tower", "1"}, {"wall_filament", "2"}, {"ooze_prevention", "1"}},
        {{"print_sequence", "by layer"}, {"enable_prime_tower", "1"}, {"wall_filament", "2"}, {"precise_z_height", "1"}},
    };
    for (size_t v = 0; v < variants.size(); ++v) {
        Model model = build_model(1);
        DynamicPrintConfig config = base_config(1);
        for (auto &kv : variants[v]) deser(config, kv.first.c_str(), kv.second.c_str());
        Print print;
        // Print.hpp:1328 `Vec3d m_origin;` is not initialised (Eigen); the front end always sets it. Without this the
        // clearance checks read stack garbage (R947: an unrelated edit of this file moved every exclusion check).
        print.set_plate_origin(Vec3d::Zero());
        print.apply(model, config);
        validate_line("v" + std::to_string(v), print);
    }
    return 0;
}

int main(int argc, char **argv)
{
    if (argc > 1 && std::string(argv[1]) == "validate") return validate_main();
    for (int scenario = 0; scenario < 8; ++scenario) {
        Model model = build_model(scenario);
        DynamicPrintConfig config = base_config(scenario);
        Print print;
        // Print.hpp:1328 `Vec3d m_origin;` is not initialised (Eigen); the front end always sets it. Without this the
        // clearance checks read stack garbage (R947: an unrelated edit of this file moved every exclusion check).
        print.set_plate_origin(Vec3d::Zero());
        print.apply(model, config);
        std::string sc = "c" + std::to_string(scenario);
        out(sc + " objects " + hI((long long) print.objects().size()) + " sequential " + hB(print.is_sequential_print()) + " short " +
            hB(print.is_all_objects_are_short()) + " wipe_tower " + hB(print.has_wipe_tower()) +
            " extruders " + hI((long long) print.extruders().size()) + " depth " + hF(print.wipe_tower_data(print.extruders().size()).depth));
        Polygons polys;
        std::vector<std::pair<Polygon, float>> hp;
        StringObjectException e = Print::sequential_print_clearance_valid(print, &polys, &hp);
        std::string hps = hI((long long) hp.size());
        for (auto &h : hp) hps += " " + POLY(h.first) + " " + hF(h.second);
        out(sc + " seq " + hB(! e.string.empty()) + " " + ID(e.object ? e.object->id() : ObjectID()) + " polys " + POLYS(polys) + " hp " + hps);
        out(sc + " seq_msg " + MSG(e.string));
        out(sc + " seq_order" + orders(print.model()));
        StringObjectException e2 = Print::sequential_print_clearance_valid(print);
        out(sc + " seq_null " + MSG(e2.string));
        std::vector<StringObjectException> warnings;
        StringObjectException l = layered_print_cleareance_valid(print, &warnings);
        std::string ws = hI((long long) warnings.size());
        for (auto &w : warnings) ws += " " + hB(w.is_warning) + " " + MSG(w.string);
        out(sc + " lay " + hB(! l.string.empty()) + " " + ID(l.object ? l.object->id() : ObjectID()) + " " + hB(l.is_warning) + " warnings " + ws);
        out(sc + " lay_msg " + MSG(l.string));
        out(sc + " lay_order" + orders(print.model(), (long long) g_base));
        out(sc + " lay_null " + MSG(layered_print_cleareance_valid(print, nullptr).string));
    }
    return 0;
}
