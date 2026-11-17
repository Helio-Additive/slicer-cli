// Generator of the vectors of tests/arrange_native_parity.rs: the helpers of libslic3r's
// Arrange.cpp at the pin (references/BambuStudio/src/libslic3r, the override layer has no copy of
// it) around the nester: the scaling helper, ArrangeParams::to_json, update_arrange_params,
// update_selected_items_inflation, update_unselected_items_inflation, get_shrink_bedpts,
// fill_config, fixed_overfit, fixed_overfit_topright_sliding, remove_large_items,
// min_area_boundingbox_rotation, fit_into_box_rotation, to_nestbin, width, height, area,
// poly_area, distance_to, to_circle, process_arrangeable and call_with_bed. Most of them are
// static or templates of Arrange.cpp, so this file includes Arrange.cpp itself.
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
//   ./gen > $DATA/vectors.txt
//
// Made on macOS arm64 with Apple clang and libc++. Lines:
//   P <name> <n> <x y>...               a point set the following lines refer to by name
//   <op> [arguments] -> <result>
// Doubles are 16 hexadecimal digits, floats 8, integers decimal. tests/arrange_native_parity.rs
// builds the same ArrangePolygons, ArrangeParams and configs from the same small integers
// (`make_ap`, `make_params`, `make_config` below).

#include "libslic3r/Arrange.cpp"
#include "libslic3r/PrintConfig.hpp"

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <map>
#include <string>

using namespace Slic3r;
using namespace Slic3r::arrangement;

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
static std::string N(long long v) { return std::to_string(v); }
static std::string PTS(const Points &pts)
{
    std::string s = N(pts.size());
    for (const Point &p : pts) s += " " + N(p.x()) + " " + N(p.y());
    return s;
}
static std::string BOX(const Box &b)
{
    return N(b.minCorner().x()) + " " + N(b.minCorner().y()) + " " + N(b.maxCorner().x()) + " " + N(b.maxCorner().y());
}
static void out(const std::string &lhs, const std::string &rhs) { std::printf("%s -> %s\n", lhs.c_str(), rhs.c_str()); }

static std::map<std::string, Points> g_sets;
static std::vector<std::string>      g_names;
static void def(const std::string &name, const Points &pts)
{
    g_sets[name] = pts;
    g_names.push_back(name);
    std::printf("P %s %s\n", name.c_str(), PTS(pts).c_str());
}

// The ArrangePolygon of a point set and a small integer.
static ArrangePolygon make_ap(const Points &pts, int k)
{
    ArrangePolygon ap;
    ap.poly.contour.points = pts;
    ap.translation         = Vec2crd(coord_t(k * 1234567 % 5000000), coord_t(-k * 7654321 % 3000000));
    ap.rotation            = 0.37 * k;
    ap.inflation           = coord_t(k * 100000 - 150000);
    ap.bed_idx             = k - 1;
    ap.priority            = k % 4;
    ap.itemid              = k + 10;
    ap.is_virt_object           = (k % 5) == 3;
    ap.is_extrusion_cali_object = (k % 7) == 3;
    ap.is_wipe_tower            = (k % 6) == 4;
    ap.has_tree_support         = (k % 4) == 2;
    ap.extrude_id_filament_types[k % 3] = "PLA";
    ap.extrude_id_filament_types[k % 3 + 2] = "PETG";
    ap.filament_temp_type = k % 3;
    ap.bed_temp           = 55 + k;
    ap.print_temp         = 210 + k;
    ap.first_bed_temp     = 60 + k;
    ap.first_print_temp   = 215 + k;
    ap.vitrify_temp       = 50 + 3 * k;
    ap.height             = 10.5 * k;
    ap.brim_width         = 0.75 * (k % 4);
    ap.name               = "obj" + std::to_string(k);
    return ap;
}

static ArrangeParams make_params(int k)
{
    ArrangeParams p;
    if (k == 0) return p;
    p.min_obj_distance    = (k % 2) ? coord_t(k * 250000) : 0;
    p.min_inflation_floor = (k % 3 == 2) ? coord_t(120000) : 0;
    p.accuracy            = 1.f / float(k);
    p.parallel            = (k % 2) == 0;
    p.allow_rotations     = (k % 2) == 1;
    p.do_final_align      = (k % 3) != 0;
    p.allow_multi_materials_on_same_plate = (k % 4) != 1;
    p.avoid_extrusion_cali_region         = (k % 4) != 2;
    p.is_seq_print        = (k % 3) == 1;
    p.save_svg            = (k % 5) == 1;
    p.bed_shrink_x        = 0.4f * float(k);
    p.bed_shrink_y        = 0.3f * float(k % 3);
    p.clearance_height_to_rod = 25.f + k;
    p.clearance_height_to_lid = 90.f + k;
    p.cleareance_radius   = 30.f + 2.5f * float(k);
    p.nozzle_height       = 4.f * float(k);
    p.printable_height    = 250.f + k;
    return p;
}

static DynamicPrintConfig make_config(int k)
{
    DynamicPrintConfig c = DynamicPrintConfig::full_print_config();
    auto set = [&c](const char *key, const char *value) {
        ConfigSubstitutionContext ctxt(ForwardCompatibilitySubstitutionRule::Disable);
        c.set_deserialize(key, value, ctxt);
    };
    set("printable_area", "0x0,256x0,256x256,0x256");
    if (k == 1) {
        set("skirt_loops", "2");
        set("skirt_distance", "3.5");
        set("skirt_height", "1");
    }
    if (k == 2) {
        set("skirt_loops", "3");
        set("skirt_distance", "2");
        set("skirt_height", "2");
        set("printable_area", "10x-5,190x-5,190x215,10x215");
        set("initial_layer_line_width", "0.6");
    }
    if (k == 3) {
        set("draft_shield", "enabled");
        set("skirt_distance", "6");
        set("printable_area", "-90x-90,90x-90,90x90,-90x90");
    }
    return c;
}

static std::string PARAMS(const ArrangeParams &p)
{
    return N(p.min_obj_distance) + " " + N(p.min_inflation_floor) + " " + hF(p.bed_shrink_x) + " " + hF(p.bed_shrink_y) + " " + hF(p.brim_skirt_distance) + " " +
           hD(p.brim_max) + " " + N(p.plate_has_tree_support ? 1 : 0);
}

static std::string ITEM(Item &it)
{
    std::string s = N(it.binId()) + " " + N(it.priority()) + " " + N(it.itemId()) + " " + hD(double(it.rotation())) + " " + N(it.translation().x()) + " " +
                    N(it.translation().y()) + " " + N(it.inflation()) + " " + hD(it.height) + " " + it.name + " " + N(it.is_virt_object ? 1 : 0) + " " +
                    N(it.is_wipe_tower ? 1 : 0) + " " + N(it.is_extrusion_cali_object ? 1 : 0) + " " + hD(it.bed_temp) + " " + hD(it.print_temp) + " " +
                    hD(it.vitrify_temp) + " " + N(it.filament_temp_type) + " " + N(it.isFixed() ? 1 : 0);
    s += " " + N(it.extrude_id_filament_types.size());
    for (const auto &kv : it.extrude_id_filament_types) s += " " + N(kv.first) + " " + kv.second;
    s += " " + N(it.vertexCount()) + " " + hD(it.area());
    s += " | " + PTS(it.transformedShape().contour.points);
    s += " | " + BOX(it.boundingBox());
    return s;
}

int main()
{
    // Point sets, scaled.
    def("rect", {{0, 0}, {2000000, 0}, {2000000, 3000000}, {0, 3000000}});
    def("ell", {{0, 0}, {4000000, 0}, {4000000, 1000000}, {1000000, 1000000}, {1000000, 5000000}, {0, 5000000}});
    def("tri", {{-1500000, -200000}, {2500000, 700000}, {300000, 3300000}});
    def("bed256", {{0, 0}, {25600000, 0}, {25600000, 25600000}, {0, 25600000}});
    def("skew", {{100000, 0}, {5100000, 900000}, {4300000, 2600000}, {-700000, 1700000}});
    def("small", {{0, 0}, {300000, 0}, {300000, 200000}, {0, 200000}});
    def("cw", {{0, 0}, {0, 1800000}, {2700000, 1800000}, {2700000, 0}});
    def("two", {{0, 0}, {1000000, 1000000}});
    {
        Points c, c2, oct;
        for (int i = 0; i < 48; ++i) {
            const double a = 2. * PI * i / 48.;
            c.emplace_back(coord_t(std::lround(9000000. * std::cos(a))) + 500000, coord_t(std::lround(9000000. * std::sin(a))) - 250000);
            c2.emplace_back(coord_t(std::lround(9000000. * std::cos(a))), coord_t(std::lround(8990000. * std::sin(a))));
        }
        for (int i = 0; i < 8; ++i) {
            const double a = 2. * PI * i / 8. + 0.2;
            oct.emplace_back(coord_t(std::lround(2000000. * std::cos(a))), coord_t(std::lround(2000000. * std::sin(a))));
        }
        def("circle", c);
        def("oval", c2);
        def("oct", oct);
    }

    // unscaled(IntPoint)
    for (const Point &p : {Point(0, 0), Point(123456, -98765), Point(2147483647, -2147483647), Point(1, 3)})
        out("unscaled " + N(p.x()) + " " + N(p.y()), hD(unscaled(p).x()) + " " + hD(unscaled(p).y()));

    // ArrangeParams::to_json
    for (int k = 0; k < 6; ++k) out("to_json " + N(k), make_params(k).to_json());

    // The measures of a point set and the bed dispatch.
    for (const std::string &name : g_names) {
        const Points &    pts = g_sets[name];
        const BoundingBox bb(pts);
        out("measures " + name, N(width(bb)) + " " + N(height(bb)) + " " + hD(area(bb)) + " " + hD(poly_area(pts)) + " " + hD(distance_to(pts.front(), pts.back())));
        const CircleBed circ = to_circle(bb.center(), pts);
        out("to_circle " + name, N(circ.center().x()) + " " + N(circ.center().y()) + " " + (std::isnan(circ.radius()) ? std::string("nan") : hD(circ.radius())));
        std::string kind = call_with_bed(pts, [](const auto &bed) -> std::string {
            using T = std::decay_t<decltype(bed)>;
            if constexpr (std::is_same_v<T, InfiniteBed>) {
                const Box b = to_nestbin(bed);
                return "infinite " + N(bed.center.x()) + " " + N(bed.center.y()) + " " + BOX(b);
            } else if constexpr (std::is_same_v<T, BoundingBox>) {
                const Box b = to_nestbin(bed);
                return "box " + BOX(b) + " " + N(b.width()) + " " + N(b.height()) + " " + N(b.area());
            } else if constexpr (std::is_same_v<T, CircleBed>) {
                const arrangement::Circle c = to_nestbin(bed);
                return "circle " + N(c.center().x()) + " " + N(c.center().y()) + " " + hD(c.radius());
            } else {
                const ExPolygon e = to_nestbin(bed);
                return "polygon " + PTS(e.contour.points) + " " + N(e.holes.size());
            }
        });
        out("call_with_bed " + name, kind);
    }
    out("call_with_bed_empty", call_with_bed(Points{}, [](const auto &bed) -> std::string {
            using T = std::decay_t<decltype(bed)>;
            if constexpr (std::is_same_v<T, InfiniteBed>) return "infinite " + N(bed.center.x()) + " " + N(bed.center.y()) + " " + BOX(to_nestbin(bed));
            else return "other";
        }));
    // A center of non-negative coordinates: `lowest() + 2 * getY(center)` of _Box::infinite
    // (geometry_traits.hpp:502-503) overflows the int for a negative one, which is undefined (the
    // port wraps).
    out("call_with_bed_one", call_with_bed(Points{Point(700000, 300000)}, [](const auto &bed) -> std::string {
            using T = std::decay_t<decltype(bed)>;
            if constexpr (std::is_same_v<T, InfiniteBed>) return "infinite " + N(bed.center.x()) + " " + N(bed.center.y()) + " " + BOX(to_nestbin(bed));
            else return "other";
        }));

    // The rotations.
    for (const std::string &name : g_names) {
        const Points &pts = g_sets[name];
        if (pts.size() < 3) continue;
        ExPolygon sh;
        sh.contour.points = pts;
        if (sh.contour.is_counter_clockwise()) sh.contour.reverse();
        out("min_area_rotation " + name, hD(double(min_area_boundingbox_rotation(sh))));
        const int boxes[][2] = {{2500000, 3500000}, {6000000, 2000000}, {1000000, 1000000}, {30000000, 30000000}};
        for (auto &b : boxes) {
            std::string r;
            try {
                r = hD(double(fit_into_box_rotation(sh, Box(b[0], b[1]))));
            } catch (const std::exception &) {
                r = "throws";
            }
            out("fit_into_box " + name + " " + N(b[0]) + " " + N(b[1]), r);
        }
    }

    // process_arrangeable and remove_large_items.
    {
        std::vector<Item> all;
        int               k = 0;
        for (const std::string &name : g_names) {
            ++k;
            ArrangePolygon    ap = make_ap(g_sets[name], k);
            std::vector<Item> items;
            process_arrangeable(ap, items);
            std::string s = N(items.size());
            for (Item &it : items) s += " # " + ITEM(it);
            out("process_arrangeable " + name + " " + N(k), s);
            process_arrangeable(ap, all);
        }
        const int boxes[][4] = {{0, 0, 30000000, 30000000}, {-20000000, -20000000, 20000000, 20000000}, {0, -3000000, 9000000, 6000000}};
        for (auto &b : boxes) {
            std::vector<Item> items = all;
            remove_large_items(items, Box(Point(b[0], b[1]), Point(b[2], b[3])));
            std::string s = N(items.size());
            for (Item &it : items) s += " " + N(it.itemId());
            out("remove_large_items_box " + N(b[0]) + " " + N(b[1]) + " " + N(b[2]) + " " + N(b[3]), s);
        }
        const double circles[][3] = {{0, 0, 30000000.}, {2000000, 1000000, 6500000.5}, {0, 0, 100000.}};
        for (auto &c : circles) {
            std::vector<Item> items = all;
            remove_large_items(items, arrangement::Circle(Point(coord_t(c[0]), coord_t(c[1])), c[2]));
            std::string s = N(items.size());
            for (Item &it : items) s += " " + N(it.itemId());
            out("remove_large_items_circle " + N((long long) c[0]) + " " + N((long long) c[1]) + " " + hD(c[2]), s);
        }
    }

    // fixed_overfit and fixed_overfit_topright_sliding.
    {
        const int piles[][4] = {{0, 0, 5000000, 4000000}, {-3000000, -2000000, 30000000, 1000000}, {20000000, 20000000, 26000000, 27000000}, {1000000, 1000000, 2000000, 2000000}};
        const Box binbb(Point(0, 0), Point(25600000, 25600000));
        const std::vector<Box> excluded = {Box(Point(0, 0), Point(1800000, 2800000)), Box(Point(20000000, 24000000), Point(25600000, 25600000))};
        double score = 0.125;
        for (auto &p : piles) {
            const Box                      pile(Point(p[0], p[1]), Point(p[2], p[3]));
            const std::tuple<double, Box> res{score, pile};
            out("fixed_overfit " + hD(score) + " " + BOX(pile), hD(fixed_overfit(res, binbb)) + " " + hD(fixed_overfit_topright_sliding(res, binbb, excluded)) + " " +
                                                                  hD(fixed_overfit_topright_sliding(res, binbb, {})));
            score = score * 3.5 + 1.;
        }
    }

    // fill_config.
    for (int k = 0; k < 6; ++k) {
        ArrangeParams p = make_params(k);
        int           j = 0;
        for (const std::string &name : g_names) {
            ++j;
            if ((j + k) % 3 == 0) p.excluded_regions.emplace_back(make_ap(g_sets[name], j));
            if ((j + k) % 4 == 1) p.nonprefered_regions.emplace_back(make_ap(g_sets[name], j + 1));
        }
        placers::NfpPConfig<ExPolygon> pcfg;
        fill_config(pcfg, p);
        std::string s = N(int(pcfg.starting_point)) + " " + N(int(pcfg.alignment)) + " " + N(pcfg.rotations.size());
        for (const auto &r : pcfg.rotations) s += " " + hD(double(r));
        s += " " + N(pcfg.bed_shrink.x()) + " " + N(pcfg.bed_shrink.y()) + " " + hF(pcfg.accuracy) + " " + N(pcfg.parallel ? 1 : 0) + " " + N(pcfg.save_svg ? 1 : 0);
        s += " " + N(pcfg.m_excluded_regions.size());
        for (Item &it : pcfg.m_excluded_regions) s += " # " + ITEM(it);
        s += " ## " + N(pcfg.m_nonprefered_regions.size());
        for (Item &it : pcfg.m_nonprefered_regions) s += " # " + ITEM(it);
        out("fill_config " + N(k), s);
    }

    // The functions over a print config.
    for (int c = 0; c < 4; ++c) {
        const DynamicPrintConfig cfg = make_config(c);
        for (int k = 0; k < 6; ++k) {
            for (int variant = 0; variant < 2; ++variant) {
                ArrangeParams   p = make_params(k);
                ArrangePolygons selected, unselected;
                int             j = 0;
                for (const std::string &name : g_names) {
                    ++j;
                    if (g_sets[name].size() < 3) continue;
                    ArrangePolygon ap = make_ap(g_sets[name], j);
                    if (variant == 1) {
                        ap.height           = 0.5 * j;
                        ap.has_tree_support = false;
                    }
                    if (j % 2) selected.emplace_back(ap);
                    else unselected.emplace_back(ap);
                }
                const std::string id = N(c) + " " + N(k) + " " + N(variant);
                out("shrink_bedpts_before " + id, PTS(get_shrink_bedpts(cfg, p)));
                update_arrange_params(p, cfg, selected);
                out("update_arrange_params " + id, PARAMS(p));
                out("shrink_bedpts " + id, PTS(get_shrink_bedpts(cfg, p)));
                update_selected_items_inflation(selected, cfg, p);
                std::string s = PARAMS(p);
                for (const ArrangePolygon &ap : selected) s += " " + N(ap.inflation);
                out("update_selected_items_inflation " + id, s);
                update_unselected_items_inflation(unselected, cfg, p);
                s.clear();
                for (const ArrangePolygon &ap : unselected) s += N(ap.inflation) + " " + N(ap.poly.contour.points.front().x()) + " " + N(ap.poly.contour.points.front().y()) + " ";
                out("update_unselected_items_inflation " + id, s + "end");
            }
        }
    }
    return 0;
}
