// Generator of the vectors of tests/wipe_tower_native_parity.rs: libslic3r's GCode/WipeTower.cpp
// at the pin (references/BambuStudio/src/libslic3r), compiled into this program (it is included
// below, so that the WipeTowerWriter class it defines and the file's static helpers can be driven),
// linked with the rest of libslic3r.
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
// It writes free.txt (the free functions and static members), writer.txt (WipeTowerWriter
// driven operation by operation) and runs.txt (whole WipeTower runs: the constructor, set_extruder,
// plan_toolchange and generate_new over small synthetic plates). The vectors in this directory
// were made on macOS arm64 with Apple clang (its default -ffp-contract=on) and libc++. The runs
// are deterministic.
//
// Line format: tokens separated by one space. A float is the 8 hexadecimal digits of its bits, a
// double its 16, an integer decimal, a text the hexadecimal of its bytes ("-" when empty).
//   free.txt:   <function> <inputs...> => <outputs...>
//   writer.txt: CASE <name> / NEW <writer arguments> / OP <operation> <arguments...> /
//               RES <state of the writer> / END
//   runs.txt:   RUN <name> / CFG <key> <value> ... / CALL <setter> <arguments...> /
//               PLAN <plan_toolchange arguments> / GEN / LAYER <n> / TCR <fields...> / TOWER ... / END
// The Rust test replays every operation on the port and compares what it gets with the RES / TCR
// lines.

#include "libslic3r/libslic3r.h"
#include "libslic3r/Point.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/Polyline.hpp"
#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/MultiNozzleUtils.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/Circle.hpp"
#include "libslic3r/GCode/GCodeProcessor.hpp"

#include <cstdio>
#include <cstdint>
#include <cstring>
#include <functional>
#include <map>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

// The private members of WipeTower (and of WipeTowerWriter) are driven directly.
#define private public
#include "libslic3r/GCode/WipeTower.cpp"
#undef private

using namespace Slic3r;

static FILE *g_out = nullptr;

static std::string hx(const std::string &s)
{
    if (s.empty()) return "-";
    static const char *d = "0123456789abcdef";
    std::string o;
    o.reserve(s.size() * 2);
    for (unsigned char c : s) { o += d[c >> 4]; o += d[c & 15]; }
    return o;
}
static std::string ff(float v)
{
    uint32_t u;
    std::memcpy(&u, &v, 4);
    char b[16];
    snprintf(b, sizeof(b), "%08x", u);
    return b;
}
static std::string dd(double v)
{
    uint64_t u;
    std::memcpy(&u, &v, 8);
    char b[24];
    snprintf(b, sizeof(b), "%016llx", (unsigned long long) u);
    return b;
}
static std::string ii(long long v) { return std::to_string(v); }
static std::string v2(const Vec2f &v) { return ff(v.x()) + " " + ff(v.y()); }
static std::string pt(const Point &p) { return ii(p.x()) + " " + ii(p.y()); }
static std::string pts(const Points &p)
{
    std::string o = ii(p.size());
    for (const Point &q : p) o += " " + pt(q);
    return o;
}
static std::string v2s(const std::vector<Vec2f> &v)
{
    std::string o = ii(v.size());
    for (const Vec2f &q : v) o += " " + v2(q);
    return o;
}
static void line(const std::string &s) { fprintf(g_out, "%s\n", s.c_str()); }

// Deterministic pseudo random numbers.
static uint64_t g_seed = 0x2545F4914F6CDD1DULL;
static uint32_t rnd()
{
    g_seed ^= g_seed << 13;
    g_seed ^= g_seed >> 7;
    g_seed ^= g_seed << 17;
    return uint32_t(g_seed >> 11);
}
static float frand(float lo, float hi) { return lo + (hi - lo) * float(rnd() % 1000001) / 1000000.f; }
static int irand(int lo, int hi) { return lo + int(rnd() % uint32_t(hi - lo + 1)); }

static Polygon make_polygon(const std::vector<std::pair<double, double>> &xy)
{
    Polygon p;
    for (auto &q : xy) p.points.push_back(Point(coord_t(scale_(q.first)), coord_t(scale_(q.second))));
    return p;
}

// ---------------------------------------------------------------------------------------------
// free.txt
// ---------------------------------------------------------------------------------------------

static void gen_free()
{
    // nozzle_change_width_for_diameter
    for (float d : {0.1f, 0.2f, 0.25f, 0.3f, 0.35f, 0.4f, 0.45f, 0.5f, 0.6f, 0.7f, 0.8f, 1.0f, 1.2f, 0.f})
        line("nozzle_change_width_for_diameter " + ff(d) + " => " + ff(nozzle_change_width_for_diameter(d)));

    // parseDoubles
    for (std::string s : {std::string(""), std::string("1"), std::string("0.5,1,2"), std::string(" 0.5, 1.5 ,7"),
                          std::string("0.001,10"), std::string("a,1"), std::string("1,,2"), std::string("1e1,2.5e-3"),
                          std::string("3,"), std::string("\t2.25")}) {
        std::vector<double> r = parseDoubles(s);
        std::string o = "parseDoubles " + hx(s) + " => " + ii(r.size());
        for (double v : r) o += " " + dd(v);
        line(o);
    }

    // align_round / align_ceil / align_floor
    for (int i = 0; i < 40; ++i) {
        float v = frand(-50.f, 50.f), b = frand(0.1f, 3.f);
        line("align " + ff(v) + " " + ff(b) + " => " + ff(align_round(v, b)) + " " + ff(align_ceil(v, b)) + " " + ff(align_floor(v, b)));
    }

    // is_valid_gcode
    for (std::string s : {std::string(""), std::string(";comment\n"), std::string("  ;c\n   \nG1 X1\n"), std::string("G1 X1"),
                          std::string("\n\n"), std::string("   G92 E0\n"), std::string("; a\n;b\n")})
        line("is_valid_gcode " + hx(s) + " => " + ii(is_valid_gcode(s)));

    // get_limit_depth_by_height / get_auto_brim_by_height
    for (float h : {0.f, 1.f, 5.f, 5.5f, 50.f, 99.9f, 100.f, 150.f, 250.f, 300.f, 350.f, 400.f, 196.8f})
        line("depth_brim " + ff(h) + " => " + ff(WipeTower::get_limit_depth_by_height(h)) + " " + ff(WipeTower::get_auto_brim_by_height(h)));

    // encode_block_key / decode_block_category
    for (int c : {-3, 0, 1, 7, 100}) for (float d : {0.2f, 0.4f, 0.6f, 0.8f, 3.5f}) for (int lane : {0, 1, 15, 17}) {
        int k = WipeTower::encode_block_key(c, d, lane);
        line("block_key " + ii(c) + " " + ff(d) + " " + ii(lane) + " => " + ii(k) + " " + ii(WipeTower::decode_block_category(k)));
    }

    // chamfer_polygon / rounding_rectangle / WipeTower::rounding_polygon
    std::vector<Polygon> polys = {
        make_polygon({{0, 0}, {35, 0}, {35, 20}, {0, 20}}),
        make_polygon({{1.5, 2.25}, {30.1, 1.9}, {29.7, 18.4}, {2.2, 19.1}}),
        make_polygon({{0, 0}, {10, 0}, {10, 0.5}, {0, 0.5}}),
        make_polygon({{0, 0}, {20, 0}, {25, 10}, {10, 15}, {-3, 8}}),
        make_polygon({{0, 0}, {5, 0}}),
        make_polygon({{0, 0}, {40, 0}, {40, 40}, {20, 41}, {0, 40}}),
    };
    for (int i = 0; i < 6; ++i) {
        std::vector<std::pair<double, double>> xy;
        int n = irand(3, 9);
        for (int k = 0; k < n; ++k) {
            double a = 2 * PI * k / n + frand(-0.2f, 0.2f);
            double r = frand(3.f, 20.f);
            xy.push_back({r * cos(a), r * sin(a)});
        }
        polys.push_back(make_polygon(xy));
    }
    for (const Polygon &p : polys) {
        for (double dis : {2., 0.7, 5.}) for (double tol : {30. / 180. * PI, 10. / 180. * PI, 80. / 180. * PI}) {
            Polygon q = p;
            line("chamfer_polygon " + pts(p.points) + " " + dd(dis) + " " + dd(tol) + " => " + pts(chamfer_polygon(q, dis, tol).points));
            q = p;
            line("rounding_rectangle " + pts(p.points) + " " + dd(dis) + " " + dd(tol) + " => " + pts(rounding_rectangle(q, dis, tol).points));
            q = p;
            line("rounding_polygon " + pts(p.points) + " " + dd(dis) + " " + dd(tol) + " => " + pts(WipeTower::rounding_polygon(q, dis, tol).points));
        }
    }

    // ray_intersetion_line
    for (int i = 0; i < 60; ++i) {
        Vec2f a(frand(-5, 40), frand(-5, 40)), v1(i % 3 == 0 ? Vec2f(-1, 0) : (i % 3 == 1 ? Vec2f(1, 0) : Vec2f(frand(-1, 1), frand(-1, 1))));
        Vec2f b(frand(-5, 40), frand(-5, 40)), c(i % 4 == 0 ? b + Vec2f(0, frand(1, 20)) : Vec2f(frand(-5, 40), frand(-5, 40)));
        auto [ok, r] = ray_intersetion_line(a, v1, b, c);
        line("ray_intersetion_line " + v2(a) + " " + v2(v1) + " " + v2(b) + " " + v2(c) + " => " + ii(ok) + " " + v2(r));
    }

    // scale_polygon / unscale_polygon
    for (int i = 0; i < 10; ++i) {
        std::vector<Vec2f> v;
        int n = irand(1, 8);
        for (int k = 0; k < n; ++k) v.push_back(Vec2f(frand(-30, 60), frand(-30, 60)));
        if (i == 0) v = {Vec2f(35.f, 0.f), Vec2f(0.1f, 0.2f), Vec2f(-0.35f, 17.5f)};
        Polygon p = scale_polygon(v);
        line("scale_polygon " + v2s(v) + " => " + pts(p.points) + " " + v2s(unscale_polygon(p)));
    }

    // generate_rectange
    for (int i = 0; i < 12; ++i) {
        Line l(Point(irand(-3000000, 3000000), irand(-3000000, 3000000)), Point(irand(-3000000, 3000000), irand(-3000000, 3000000)));
        coord_t off = irand(1, 300000);
        line("generate_rectange " + pt(l.a) + " " + pt(l.b) + " " + ii(off) + " => " + pts(generate_rectange(l, off).points));
    }

    // remove_points_from_segment
    for (int i = 0; i < 12; ++i) {
        Segment seg(Vec2f(frand(0, 5), frand(0, 5)), Vec2f(0, frand(10, 40)));
        seg.end.x() = seg.start.x();
        std::vector<Vec2f> skip;
        int n = irand(0, 4);
        for (int k = 0; k < n; ++k) skip.push_back(Vec2f(frand(0, 5), frand(-2, 45)));
        double range = frand(0.2f, 3.f);
        std::vector<Segment> r = remove_points_from_segment(seg, skip, range);
        std::string o = "remove_points_from_segment " + v2(seg.start) + " " + v2(seg.end) + " " + v2s(skip) + " " + dd(range) + " => " + ii(r.size());
        for (auto &s : r) o += " " + v2(s.start) + " " + v2(s.end);
        line(o);
    }

    // move_point_along_polygon / insert_points
    for (int i = 0; i < 30; ++i) {
        std::vector<Vec2f> poly;
        int n = irand(3, 7);
        for (int k = 0; k < n; ++k) {
            double a = 2 * PI * k / n;
            poly.push_back(Vec2f(float(10 + 8 * cos(a)), float(10 + 8 * sin(a))));
        }
        int idx = irand(0, n - 1);
        float t = frand(0, 1);
        Vec2f start = poly[idx] + t * (poly[(idx + 1) % n] - poly[idx]);
        float off = frand(0.1f, 30.f);
        bool fwd = (i & 1) != 0;
        IntersectionInfo r = move_point_along_polygon(poly, start, idx, off, fwd, i);
        line("move_point_along_polygon " + v2s(poly) + " " + v2(start) + " " + ii(idx) + " " + ff(off) + " " + ii(fwd) + " " + ii(i) + " => " +
             v2(r.pos) + " " + ii(r.idx) + " " + ii(r.pair_idx) + " " + ff(r.dis_from_idx));
        std::vector<PointWithFlag> pl;
        for (auto &p : poly) pl.push_back({p, -1, false});
        Vec2f ins = (i % 5 == 0) ? poly[idx] : r.pos;
        insert_points(pl, idx, ins, 7, fwd);
        std::string o = "insert_points " + v2s(poly) + " " + ii(idx) + " " + v2(ins) + " 7 " + ii(fwd) + " => " + ii(pl.size());
        for (auto &p : pl) o += " " + v2(p.pos) + " " + ii(p.pair_idx) + " " + ii(p.is_forward);
        line(o);
    }

    // add_extra_point / remove_points_from_polygon / contrust_gap_for_skip_points / generate_rectange_polygon
    for (int i = 0; i < 24; ++i) {
        float w = frand(20, 60), d = frand(10, 50);
        Vec2f mn(0, 0), mx(w, d);
        if (i % 3 == 1) mn = Vec2f(frand(-1, 1), frand(-1, 1));
        Polygon rect = generate_rectange_polygon(mn, mx);
        line("generate_rectange_polygon " + v2(mn) + " " + v2(mx) + " => " + pts(rect.points));
        int range_scaled = irand(10000, 400000);
        line("add_extra_point " + pts(rect.points) + " " + ii(range_scaled) + " => " + pts(add_extra_point(rect, range_scaled).points));
        std::vector<Vec2f> skip;
        int n = irand(0, 4);
        for (int k = 0; k < n; ++k) {
            bool left = (rnd() & 1) != 0;
            skip.push_back(Vec2f(left ? mn.x() + frand(0.f, 2.f) : mx.x() - frand(0.f, 2.f), frand(mn.y() + 1, mx.y() - 1)));
        }
        float gap = frand(0.5f, 3.f);
        Polygon ins;
        Polylines r = contrust_gap_for_skip_points(rect, skip, w, gap, ins);
        std::string o = "contrust_gap_for_skip_points " + pts(rect.points) + " " + v2s(skip) + " " + ff(w) + " " + ff(gap) + " => " + ii(r.size());
        for (auto &p : r) o += " " + pts(p.points);
        o += " " + pts(ins.points);
        line(o);
    }

    // polylines_extreme_at
    for (int i = 0; i < 30; ++i) {
        Polylines pls;
        int n = irand(1, 3);
        for (int k = 0; k < n; ++k) {
            Polyline pl;
            int m = irand(2, 6);
            for (int j = 0; j < m; ++j) pl.points.push_back(Point(irand(-500000, 4000000), irand(-500000, 4000000)));
            pls.push_back(pl);
        }
        bool ax = (i & 1) != 0, pos = (i & 2) != 0;
        float along = frand(-5, 40);
        float out = -7.f;
        bool ok = polylines_extreme_at(pls, ax, pos, along, out);
        std::string o = "polylines_extreme_at " + ii(pls.size());
        for (auto &p : pls) o += " " + pts(p.points);
        o += " " + ii(ax) + " " + ii(pos) + " " + ff(along) + " => " + ii(ok) + " " + ff(out);
        line(o);
    }

    // move_box_inside_box
    for (int i = 0; i < 20; ++i) {
        BoundingBox b1(Point(irand(-1000000, 3000000), irand(-1000000, 3000000)), Point(0, 0));
        b1.max = b1.min + Point(irand(100000, 2000000), irand(100000, 2000000));
        BoundingBox b2(Point(irand(-2000000, 0), irand(-2000000, 0)), Point(irand(2000000, 6000000), irand(2000000, 6000000)));
        int off = irand(0, 300000);
        line("move_box_inside_box " + pt(b1.min) + " " + pt(b1.max) + " " + pt(b2.min) + " " + pt(b2.max) + " " + ii(off) + " => " +
             v2(WipeTower::move_box_inside_box(b1, b2, off)));
    }

    // rib_section / its_make_rib_tower / its_make_rib_brim (vertices and the side faces: the caps
    // come from Triangulation, compared by count only)
    for (int i = 0; i < 10; ++i) {
        float w = frand(15, 60), d = frand(10, 60), rl = std::sqrt(w * w + d * d) + frand(0, 20), rw = frand(3, 12);
        bool fillet = (i & 1) != 0;
        if (i == 0) { w = 35; d = 35; rl = 60; rw = 8; }
        Polygon sec = WipeTower::rib_section(w, d, rl, rw, fillet);
        line("rib_section " + ff(w) + " " + ff(d) + " " + ff(rl) + " " + ff(rw) + " " + ii(fillet) + " => " + pts(sec.points));
        float h = frand(5, 200);
        TriangleMesh m = WipeTower::its_make_rib_tower(w, d, h, rl, rw, fillet);
        std::string o = "its_make_rib_tower " + ff(w) + " " + ff(d) + " " + ff(h) + " " + ff(rl) + " " + ff(rw) + " " + ii(fillet) + " => " +
                        ii(m.its.vertices.size());
        for (auto &v : m.its.vertices) o += " " + ff(v.x()) + " " + ff(v.y()) + " " + ff(v.z());
        o += " " + ii(m.its.indices.size());
        size_t side = std::min(m.its.indices.size(), size_t(2 * sec.points.size()));
        o += " " + ii(side);
        for (size_t k = m.its.indices.size() - side; k < m.its.indices.size(); ++k)
            o += " " + ii(m.its.indices[k][0]) + " " + ii(m.its.indices[k][1]) + " " + ii(m.its.indices[k][2]);
        line(o);
        TriangleMesh b = WipeTower::its_make_rib_brim(sec, h / 100.f);
        o = "its_make_rib_brim " + pts(sec.points) + " " + ff(h / 100.f) + " => " + ii(b.its.vertices.size());
        for (auto &v : b.its.vertices) o += " " + ff(v.x()) + " " + ff(v.y()) + " " + ff(v.z());
        o += " " + ii(b.its.indices.size());
        side = std::min(b.its.indices.size(), size_t(2 * sec.points.size()));
        o += " " + ii(side);
        for (size_t k = b.its.indices.size() - side; k < b.its.indices.size(); ++k)
            o += " " + ii(b.its.indices[k][0]) + " " + ii(b.its.indices[k][1]) + " " + ii(b.its.indices[k][2]);
        line(o);
    }

    // wipe_tower_layer_is_sparse / compute_compacted_wipe_tower_z
    for (int i = 0; i < 8; ++i) {
        std::vector<std::vector<WipeTower::ToolChangeResult>> tcs;
        int n = irand(0, 8);
        std::string o = "compute_compacted_wipe_tower_z " + ii(n);
        for (int k = 0; k < n; ++k) {
            std::vector<WipeTower::ToolChangeResult> layer;
            int m = irand(0, 3);
            o += " " + ii(m);
            for (int j = 0; j < m; ++j) {
                WipeTower::ToolChangeResult r;
                r.initial_tool = irand(0, 2);
                r.new_tool     = irand(0, 2);
                r.layer_height = frand(0.08f, 0.3f);
                o += " " + ii(r.initial_tool) + " " + ii(r.new_tool) + " " + ff(r.layer_height);
                layer.push_back(r);
            }
            tcs.push_back(layer);
        }
        std::vector<float> z = compute_compacted_wipe_tower_z(tcs);
        o += " =>";
        for (size_t k = 0; k < z.size(); ++k) o += " " + ff(z[k]) + " " + ii(!tcs[k].empty() && wipe_tower_layer_is_sparse(tcs[k]));
        line(o);
    }

    // wipe_tower_sparse_layers_skipped / prime_tower_side_scrape_margin
    for (int i = 0; i < 16; ++i) {
        PrintConfig cfg;
        std::string nsl = (i & 1) ? "1" : "0", tl = (i & 2) ? "1" : "0", wd = (i & 4) ? "1" : "0", ss = (i & 8) ? "1" : "0";
        cfg.set_deserialize_strict("wipe_tower_no_sparse_layers", nsl);
        cfg.set_deserialize_strict("timelapse_type", tl == "1" ? "1" : "0");
        cfg.set_deserialize_strict("enable_wrapping_detection", wd);
        cfg.set_deserialize_strict("prime_tower_side_scrape", ss);
        std::string gap = std::to_string(frand(-1, 1)), len = std::to_string(frand(0, 4));
        cfg.set_deserialize_strict("prime_tower_side_scrape_wall_gap", gap);
        cfg.set_deserialize_strict("prime_tower_side_scrape_length", len);
        line("config_flags " + nsl + " " + tl + " " + wd + " " + ss + " " + hx(gap) + " " + hx(len) + " => " +
             ii(wipe_tower_sparse_layers_skipped(cfg)) + " " + dd(prime_tower_side_scrape_margin(cfg)));
    }
}

// ---------------------------------------------------------------------------------------------
// writer.txt
// ---------------------------------------------------------------------------------------------

struct WCase
{
    std::vector<WipeTower::FilamentParameters> filpar;
    std::unique_ptr<WipeTowerWriter>           w;
    std::optional<MultiNozzleUtils::LayeredNozzleGroupResult> group;

    WCase(const std::string &name, GCodeFlavor flavor, float lh, float lw, bool arc, int nfil)
    {
        line("CASE " + name);
        std::string o = "NEW " + ii(int(flavor)) + " " + ff(lh) + " " + ff(lw) + " " + ii(arc) + " " + ii(nfil);
        for (int i = 0; i < nfil; ++i) {
            WipeTower::FilamentParameters p;
            p.filament_area = float((M_PI / 4.f) * pow(frand(1.6f, 2.9f), 2));
            p.max_e_speed   = (i % 3 == 2) ? std::numeric_limits<float>::max() : frand(1.f, 15.f);
            p.max_e_ramming_speed = {frand(1.f, 15.f), frand(1.f, 15.f)};
            filpar.push_back(p);
            o += " " + ff(p.filament_area) + " " + ff(p.max_e_speed) + " " + ff(p.max_e_ramming_speed.first) + " " + ff(p.max_e_ramming_speed.second);
        }
        line(o);
        w = std::make_unique<WipeTowerWriter>(lh, lw, flavor, filpar, arc);
    }
    void op(const std::string &s) { line("OP " + s); }
    void res()
    {
        WipeTowerWriter &wr = *w;
        std::string o = "RES " + hx(wr.gcode()) + " " + v2(wr.pos()) + " " + v2(wr.start_pos_rotated()) + " " + v2(wr.pos_rotated()) + " " +
                        ff(wr.elapsed_time()) + " " + ff(wr.m_current_feedrate) + " " + ff(wr.get_and_reset_used_filament_length());
        o += " " + ii(wr.extrusions().size());
        for (auto &e : wr.extrusions()) o += " " + v2(e.pos) + " " + ff(e.width) + " " + ii(e.tool);
        o += " " + v2s(wr.wipe_path());
        line(o);
        line("END");
    }
};

static void gen_writer()
{
    const GCodeFlavor flavors[] = {gcfMarlinLegacy, gcfMarlinFirmware, gcfKlipper, gcfRepRapFirmware, gcfRepetier, gcfRepRapSprinter};
    for (int c = 0; c < 36; ++c) {
        GCodeFlavor flavor = flavors[c % 6];
        float lh = (c % 4 == 0) ? 0.2f : frand(0.08f, 0.4f);
        float lw = (c % 5 == 0) ? 0.5f : frand(0.3f, 1.2f);
        bool arc = (c % 3) != 1;
        int nfil = irand(1, 4);
        WCase k("w" + std::to_string(c), flavor, lh, lw, arc, nfil);
        WipeTowerWriter &w = *k.w;
        float wtw = frand(20, 60), wtd = frand(10, 60);
        float angle = (c % 7 == 0) ? 0.f : ((c % 7 == 1) ? 90.f : frand(-180, 180));
        if (c < 3) angle = 0.f;
        Vec2f p0(frand(0, wtw), frand(0, wtd));
        k.op("init_pos " + v2(p0) + " " + ff(wtw) + " " + ff(wtd) + " " + ff(angle)); w.set_initial_position(p0, wtw, wtd, angle);
        int tool = irand(0, nfil - 1);
        k.op("tool " + ii(tool)); w.set_initial_tool(tool);
        float z = frand(0.2f, 50.f);
        k.op("z " + ff(z)); w.set_z(z);
        float flow = frand(0.01f, 0.08f);
        k.op("flow " + ff(flow)); w.set_extrusion_flow(flow);
        if (c % 4 == 3) { float ys = frand(-5, 5); k.op("yshift " + ff(ys)); w.set_y_shift(ys); }
        if (c % 6 >= 3) {
            // accelerations with a nozzle group result
            std::vector<int> fmap;
            std::vector<MultiNozzleUtils::NozzleInfo> nozzles;
            int next = irand(1, 2);
            for (int e = 0; e < next; ++e) {
                MultiNozzleUtils::NozzleInfo ni;
                ni.diameter = "0.4";
                ni.volume_type = nvtStandard;
                ni.extruder_id = e;
                ni.group_id = e;
                nozzles.push_back(ni);
            }
            std::vector<unsigned int> used;
            std::string g = "group " + ii(nfil);
            for (int f = 0; f < nfil; ++f) { fmap.push_back(f % next); used.push_back(f); g += " " + ii(f % next); }
            g += " " + ii(next);
            k.group = MultiNozzleUtils::LayeredNozzleGroupResult::create(fmap, nozzles, used);
            k.op(g); w.set_multi_nozzle_group_result(&*k.group);
            std::vector<unsigned int> a1, a2, a3, a4;
            std::string s1 = "accels", s2, s3, s4;
            for (int e = 0; e < next; ++e) { a1.push_back(irand(0, 3) * 1000 + 500); a2.push_back(irand(1, 20) * 250); a3.push_back(irand(1, 30) * 500); a4.push_back(irand(1, 10) * 300); }
            unsigned int mx = (c % 2) ? 0 : irand(2, 12) * 1000;
            s1 += " " + ii(next);
            for (int e = 0; e < next; ++e) s1 += " " + ii(a1[e]) + " " + ii(a2[e]) + " " + ii(a3[e]) + " " + ii(a4[e]);
            s1 += " " + ii(mx);
            bool a2d = (c % 4) < 2;
            float a2df = frand(10, 90);
            int layer_id = irand(-1, 5);
            s1 += " " + ii(a2d) + " " + ff(a2df) + " " + ii(layer_id);
            k.op(s1);
            w.set_normal_acceleration(a1); w.set_first_layer_normal_acceleration(a2); w.set_travel_acceleration(a3); w.set_first_layer_travel_acceleration(a4);
            w.set_max_acceleration(mx); w.set_accel_to_decel_enable(a2d); w.set_accel_to_decel_factor(a2df); w.set_layer_id(layer_id);
            std::vector<int> phys;
            std::string sp = "phys " + ii(next);
            for (int e = 0; e < next; ++e) { phys.push_back(next - 1 - e); sp += " " + ii(next - 1 - e); }
            k.op(sp); w.set_physical_extruder_map(phys);
        }
        // a random program
        int nops = irand(15, 45);
        for (int i = 0; i < nops; ++i) {
            int kind = irand(0, 25);
            float fx = frand(-2, wtw + 2), fy = frand(-2, wtd + 2);
            float f = (rnd() % 3 == 0) ? 0.f : float(irand(6, 400)) * 60.f + ((rnd() & 1) ? 0.5f : 0.f);
            switch (kind) {
            case 0: case 1: case 2:
                k.op("travel " + ff(fx) + " " + ff(fy) + " " + ff(f)); w.travel(fx, fy, f); break;
            case 3: case 4: case 5: case 6: {
                int lf = irand(0, 3);
                k.op("extrude " + ff(fx) + " " + ff(fy) + " " + ff(f) + " " + ii(lf)); w.extrude(fx, fy, f, LimitFlow(lf)); break;
            }
            case 7: {
                float e = (rnd() % 4 == 0) ? 0.f : frand(-2, 5);
                bool rec = (rnd() & 1) != 0;
                int lf = irand(0, 3);
                k.op("extrude_explicit " + ff(fx) + " " + ff(fy) + " " + ff(e) + " " + ff(f) + " " + ii(rec) + " " + ii(lf));
                w.extrude_explicit(fx, fy, e, f, rec, LimitFlow(lf));
                break;
            }
            case 8: { k.op("feedrate " + ff(f)); w.feedrate(f); break; }
            case 9: {
                float l = frand(0, wtw / 2), b = frand(0, wtd / 2), bw = frand(0.5f, wtw / 2), bh = frand(0.2f, wtd / 2);
                k.op("rect_box " + ff(l) + " " + ff(b) + " " + ff(bw) + " " + ff(bh) + " " + ff(f));
                w.rectangle(WipeTower::box_coordinates(l, b, bw, bh), f);
                break;
            }
            case 10: {
                Vec2f ld(frand(0, wtw / 2), frand(0, wtd / 2));
                float bw = frand(0.5f, wtw / 2), bh = frand(0.2f, wtd / 2);
                k.op("rect_ld " + v2(ld) + " " + ff(bw) + " " + ff(bh) + " " + ff(f));
                w.rectangle(ld, bw, bh, f);
                break;
            }
            case 11: {
                Polygon poly;
                if (rnd() & 1) {
                    poly = generate_rectange_polygon(Vec2f(frand(0, 3), frand(0, 3)), Vec2f(frand(10, wtw), frand(8, wtd)));
                    if (rnd() & 1) poly = WipeTower::rounding_polygon(poly);
                } else {
                    int n = irand(3, 12);
                    for (int q = 0; q < n; ++q) {
                        double a = 2 * PI * q / n;
                        poly.points.push_back(Point(coord_t(scale_(wtw / 2 + wtw / 3 * cos(a))), coord_t(scale_(wtd / 2 + wtd / 3 * sin(a)))));
                    }
                }
                k.op("polygon " + pts(poly.points) + " " + ff(f));
                w.polygon(poly, f);
                break;
            }
            case 12: { float e = frand(-3, 3); k.op("load " + ff(e) + " " + ff(f)); w.load(e, f); break; }
            case 13: { float e = frand(-3, 3); k.op("retract " + ff(e) + " " + ff(f)); w.retract(e, f); break; }
            case 14: {
                if (rnd() & 1) { float h = frand(0, 2); k.op("z_hop " + ff(h) + " " + ff(f)); w.z_hop(h, f); }
                else { k.op("z_hop_reset " + ff(f)); w.z_hop_reset(f); }
                break;
            }
            case 15: {
                float x1 = frand(0, wtw), x2 = frand(0, wtw), dy = frand(-1, 1), e0 = frand(0, 2), e = frand(0, 2);
                if (rnd() & 1) { k.op("ram " + ff(x1) + " " + ff(x2) + " " + ff(dy) + " " + ff(e0) + " " + ff(e) + " " + ff(f)); w.ram(x1, x2, dy, e0, e, f); }
                else { k.op("cool " + ff(x1) + " " + ff(x2) + " " + ff(e0) + " " + ff(e) + " " + ff(f)); w.cool(x1, x2, e0, e, f); }
                break;
            }
            case 16: {
                float far = frand(0, wtw), dist = frand(0.5f, 20), speed = frand(1, 30), mxs = (rnd() & 1) ? 50.f : frand(5, 80);
                k.op("lmxa " + ff(far) + " " + ff(dist) + " " + ff(speed) + " " + ff(mxs)); w.load_move_x_advanced(far, dist, speed, mxs);
                break;
            }
            case 17: {
                int sub = irand(0, 7);
                if (sub == 0) { int t = irand(150, 280); bool wt = rnd() & 1; k.op("temp " + ii(t) + " " + ii(wt)); w.set_extruder_temp(t, wt); }
                else if (sub == 1) { float t = (rnd() % 3 == 0) ? 0.f : frand(0, 20); k.op("wait " + ff(t)); w.wait(t); }
                else if (sub == 2) { int s = irand(1, 200); k.op("speed_override " + ii(s)); w.speed_override(s); }
                else if (sub == 3) { k.op("speed_backup"); w.speed_override_backup(); }
                else if (sub == 4) { k.op("speed_restore"); w.speed_override_restore(); }
                else if (sub == 5) { unsigned s = irand(0, 2) == 0 ? 0 : irand(1, 100); k.op("fan " + ii(s)); w.set_fan(s); }
                else if (sub == 6) { k.op("flush"); w.flush_planner_queue(); }
                else { k.op("reset_e"); w.reset_extruder(); }
                break;
            }
            case 18: {
                int sub = irand(0, 5);
                if (sub == 0) { int v = irand(-5, 500); k.op("comment_value " + hx(" CP TEST ") + " " + ii(v)); w.comment_with_value(" CP TEST ", v); }
                else if (sub == 1) { std::string s = "; free text " + std::to_string(i) + "\n"; k.op("append " + hx(s)); w.append(s); }
                else if (sub == 2) { k.op("disable_la"); w.disable_linear_advance(); }
                else if (sub == 3) { k.op("suppress"); w.suppress_preview(); }
                else if (sub == 4) { k.op("resume"); w.resume_preview(); }
                else { float v = frand(0.1f, 2); k.op("change_lw " + ff(v)); w.change_analyzer_line_width(v); }
                break;
            }
            case 19: { k.op("wipe_point " + ff(fx) + " " + ff(fy)); w.add_wipe_point(fx, fy); break; }
            case 20: {
                Polygon poly = generate_rectange_polygon(Vec2f(frand(0, 3), frand(0, 3)), Vec2f(frand(10, wtw), frand(8, wtd)));
                if (rnd() & 1) poly = add_extra_point(poly, irand(20000, 200000));
                double dist = frand(0, 40);
                k.op("wipe_path " + pts(poly.points) + " " + dd(dist)); w.add_wipe_path(poly, dist);
                break;
            }
            case 21: {
                Polygon rect = generate_rectange_polygon(Vec2f(0, 0), Vec2f(wtw, wtd));
                if (rnd() & 1) rect = WipeTower::rounding_polygon(rect);
                std::vector<Vec2f> skip;
                int n = irand(0, 3);
                for (int q = 0; q < n; ++q) skip.push_back(Vec2f((rnd() & 1) ? frand(0, 1) : wtw - frand(0, 1), frand(2, wtd - 2)));
                Polygon ins;
                Polylines pls = contrust_gap_for_skip_points(rect, skip, wtw, frand(0.5f, 2.5f), ins);
                float rl = frand(0.2f, 1.5f), rs = float(irand(10, 60)) * 60.f, feed = float(irand(10, 300)) * 60.f;
                bool fillet = rnd() & 1;
                std::string o = "gen_path " + ii(pls.size());
                for (auto &p : pls) o += " " + pts(p.points);
                o += " " + ff(feed) + " " + ff(rl) + " " + ff(rs) + " " + ii(fillet);
                k.op(o);
                w.generate_path(pls, feed, rl, rs, fillet);
                break;
            }
            case 22: {
                Vec2f c(frand(5, wtw - 5), frand(5, wtd - 5));
                float area = frand(0, 30), step = frand(0.2f, 2);
                k.op("spiral " + v2(c) + " " + ff(area) + " " + ff(step) + " " + ff(f)); w.spiral_flat_ironing(c, area, step, f);
                break;
            }
            case 23: {
                std::string zs = (rnd() & 1) ? std::string("[wipe_tower_scrape_z_top]") : std::to_string(z + 1.5f);
                k.op("travel_xyz " + ff(fx) + " " + ff(fy) + " " + hx(zs) + " " + ff(f)); w.travel_xyz(Vec2f(fx, fy), zs, f);
                break;
            }
            case 24: {
                if (k.group) {
                    int t = irand(150, 290), tl = irand(0, nfil - 1);
                    bool heat = rnd() & 1, wfm = rnd() & 1;
                    std::string cm = (rnd() & 1) ? std::string("") : std::string("pre cool");
                    if (rnd() & 1) { k.op("m104 " + ii(t) + " " + ii(tl) + " " + ii(heat) + " " + ii(wfm) + " " + hx(cm)); w.format_line_M104(t, tl, heat, wfm, cm); }
                    else { k.op("m109 " + ii(t) + " " + ii(tl) + " " + hx(cm)); w.format_line_M109(t, tl, cm); }
                } else {
                    float lh2 = frand(0.05f, 0.5f);
                    k.op("change_lh " + ff(lh2)); w.change_analyzer_layer_height(lh2);
                }
                break;
            }
            default: {
                int sub = irand(0, 3);
                if (sub == 0) { bool fl = rnd() & 1; k.op("first_layer " + ii(fl)); w.set_first_layer(fl); }
                else if (sub == 1) { int t = irand(0, nfil - 1); k.op("set_tool " + ii(t)); w.set_tool(t); }
                else if (sub == 2) { float fl2 = frand(0.01f, 0.08f); k.op("flow " + ff(fl2)); w.set_extrusion_flow(fl2); }
                else { float lw2 = frand(0.3f, 1.2f); k.op("lw " + ff(lw2)); w.set_line_width(lw2); }
                break;
            }
            }
        }
        k.res();
    }
}

// ---------------------------------------------------------------------------------------------
// runs.txt
// ---------------------------------------------------------------------------------------------

struct RunSpec
{
    std::string                                       name;
    std::vector<std::pair<std::string, std::string>>  cfg;
    int                                               nfil = 2;
    std::vector<int>                                  filament_nozzle_map;   // per filament
    std::vector<std::string>                          nozzle_diameters;      // per nozzle (= extruder here)
    float                                             height = 10.f;
    int                                               initial_tool = 0;
    bool                                              multi_diameter = false;
    float                                             grid_reference = 0.f;
    float                                             grid_layer_height = 0.f;
    bool                                              per_nozzle = false;
    bool                                              has_tpu = false;
    bool                                              foundation_required = true;
    float                                             first_layer_flow_ratio = 1.f;
    std::vector<int>                                  categories;
    // layers: z, height, list of (new tool, wipe_volume_ec, wipe_volume_nc, prime_volume)
    struct Change { int tool; float ec, nc, prime; };
    struct Layer { float z, h; std::vector<Change> changes; bool no_fill = false; };
    std::vector<Layer>                                layers;
};

static void dump_tcr(const WipeTower::ToolChangeResult &t)
{
    std::string o = "TCR " + ff(t.print_z) + " " + ff(t.layer_height) + " " + hx(t.gcode) + " " + v2(t.start_pos) + " " + v2(t.end_pos) + " " +
                    ff(t.elapsed_time) + " " + ii(t.priming) + " " + ii(t.is_tool_change) + " " + v2(t.tool_change_start_pos) + " " +
                    v2s(t.wipe_path) + " " + ff(t.purge_volume) + " " + ii(t.initial_tool) + " " + ii(t.new_tool) + " " + ii(t.is_finish_first) + " " +
                    ii(t.is_contact) + " " + ii(t.side_scrape) + " " + ff(t.side_scrape_z_low) + " " + ff(t.side_scrape_depth) + " " +
                    ff(t.side_scrape_min_height_above_base);
    const WipeTower::NozzleChangeResult &n = t.nozzle_change_result;
    o += " " + hx(n.gcode) + " " + v2(n.start_pos) + " " + v2(n.end_pos) + " " + v2(n.origin_start_pos) + " " + v2s(n.wipe_path) + " " +
         ii(n.is_extruder_change) + " " + ii(n.zero_flush_idle_cooling) + " " + ii(n.heater_off_only) + " " + ff(n.precool_target_temp) + " " +
         ii(n.precool_filament_id);
    o += " " + ii(t.extrusions.size());
    for (auto &e : t.extrusions) o += " " + v2(e.pos) + " " + ff(e.width) + " " + ii(e.tool);
    line(o);
}

static void gen_run(const RunSpec &r)
{
    line("RUN " + r.name);
    PrintConfig cfg;
    for (auto &kv : r.cfg) {
        cfg.set_deserialize_strict(kv.first, kv.second);
        line("CFG " + hx(kv.first) + " " + hx(kv.second));
    }
    std::vector<MultiNozzleUtils::NozzleInfo> nozzles;
    for (size_t e = 0; e < r.nozzle_diameters.size(); ++e) {
        MultiNozzleUtils::NozzleInfo ni;
        ni.diameter    = r.nozzle_diameters[e];
        ni.volume_type = nvtStandard;
        ni.extruder_id = int(e);
        ni.group_id    = int(e);
        nozzles.push_back(ni);
    }
    std::vector<unsigned int> used;
    for (int f = 0; f < r.nfil; ++f) used.push_back(f);
    auto group = MultiNozzleUtils::LayeredNozzleGroupResult::create(r.filament_nozzle_map, nozzles, used);
    {
        std::string o = "CALL group " + ii(r.nfil);
        for (int m : r.filament_nozzle_map) o += " " + ii(m);
        o += " " + ii(r.nozzle_diameters.size());
        for (auto &d : r.nozzle_diameters) o += " " + hx(d);
        line(o);
    }
    line("CALL tower " + ff(r.height) + " " + ii(r.initial_tool) + " " + ii(r.nfil));
    WipeTower wt(cfg, 0, Vec3d(0, 0, 0), size_t(r.initial_tool), r.height, used);
    line("CALL setup " + ii(r.foundation_required) + " " + ii(r.has_tpu) + " " + ii(r.multi_diameter) + " " + ff(r.grid_reference) + " " +
         ff(r.grid_layer_height) + " " + ii(r.per_nozzle) + " " + ff(r.first_layer_flow_ratio));
    wt.set_foundation_required(r.foundation_required);
    wt.set_first_layer_flow_ratio(r.first_layer_flow_ratio);
    wt.set_has_tpu_filament(r.has_tpu);
    wt.set_multi_diameter_print(r.multi_diameter);
    wt.set_tower_grid_reference_nozzle(r.grid_reference);
    wt.set_tower_grid_layer_height(r.grid_layer_height);
    wt.set_per_nozzle_tower_params(r.per_nozzle);
    wt.set_nozzle_group_result(*group);
    // A 256 x 256 bed.
    Polygons bed{make_polygon({{0, 0}, {256, 0}, {256, 256}, {0, 256}})};
    wt.set_shared_print_bed(bed);
    // config_idx 0: the filament options of these configurations carry one variant row.
    for (int i = 0; i < r.nfil; ++i)
        wt.set_extruder(size_t(i), cfg, 0);
    int old_tool = r.initial_tool;
    for (const auto &l : r.layers) {
        line("PLAN " + ff(l.z) + " " + ff(l.h) + " " + ii(old_tool) + " " + ii(old_tool) + " " + ff(0.f) + " " + ff(0.f) + " " + ff(0.f));
        wt.plan_toolchange(l.z, l.h, old_tool, old_tool);
        for (const auto &c : l.changes) {
            line("PLAN " + ff(l.z) + " " + ff(l.h) + " " + ii(old_tool) + " " + ii(c.tool) + " " + ff(c.ec) + " " + ff(c.nc) + " " + ff(c.prime));
            wt.plan_toolchange(l.z, l.h, old_tool, c.tool, c.ec, c.nc, c.prime);
            old_tool = c.tool;
        }
        if (l.no_fill) {
            line("CALL no_fill");
            wt.set_last_layer_extruder_fill(false);
        }
    }
    {
        std::vector<int> ids;
        for (int f = 0; f < r.nfil; ++f) ids.push_back(f);
        std::string o = "CALL ids " + ii(ids.size());
        for (int v : ids) o += " " + ii(v);
        line(o);
        wt.set_used_filament_ids(ids);
        o = "CALL categories " + ii(r.categories.size());
        for (int v : r.categories) o += " " + ii(v);
        line(o);
        wt.set_filament_categories(r.categories);
    }
    std::vector<std::vector<WipeTower::ToolChangeResult>> result;
    line("GEN");
    wt.generate_new(result);
    for (size_t i = 0; i < result.size(); ++i) {
        line("LAYER " + ii(result[i].size()));
        for (auto &t : result[i]) dump_tcr(t);
    }
    BoundingBoxf bb = wt.get_bbx();
    std::string o = "TOWER " + ff(wt.get_depth()) + " " + ff(wt.get_brim_width()) + " " + ff(wt.get_height()) + " " + ii(wt.get_number_of_toolchanges()) + " " +
                    dd(bb.min.x()) + " " + dd(bb.min.y()) + " " + dd(bb.max.x()) + " " + dd(bb.max.y()) + " " + ff(wt.get_rib_length()) + " " +
                    ff(wt.get_rib_width()) + " " + v2(wt.get_rib_offset());
    std::vector<float> used_len = wt.get_used_filament();
    o += " " + ii(used_len.size());
    for (float v : used_len) o += " " + ff(v);
    line(o);
    line("END");
}

static std::vector<RunSpec> run_specs()
{
    std::vector<RunSpec> specs;
    auto base = [](const std::string &name, int nfil) {
        RunSpec r;
        r.name = name;
        r.nfil = nfil;
        r.filament_nozzle_map.assign(nfil, 0);
        r.nozzle_diameters = {"0.4"};
        std::string types, diam, temps, vols, cats, fcl, fcl_nc;
        for (int i = 0; i < nfil; ++i) {
            types  += (i ? ";" : "") + std::string("PLA");
            diam   += (i ? "," : "") + std::string("1.75");
            temps  += (i ? "," : "") + std::string("220");
            vols   += (i ? "," : "") + std::string("20");
            cats   += (i ? "," : "") + std::string("0");
            fcl    += (i ? "," : "") + std::to_string(10 + 2 * i);
            fcl_nc += (i ? "," : "") + std::to_string(15 + i);
        }
        r.cfg = {{"prime_tower_width", "35"},         {"prime_tower_brim_width", "3"}, {"wipe_tower_x", "165"},
                 {"wipe_tower_y", "200"},             {"filament_type", types},        {"filament_diameter", diam},
                 {"nozzle_temperature", temps},       {"nozzle_temperature_initial_layer", temps},
                 {"filament_max_volumetric_speed", vols}, {"filament_adhesiveness_category", cats},
                 {"enable_arc_fitting", "0"},         {"prime_tower_skip_points", "0"}, {"prime_tower_rib_wall", "0"},
                 {"prime_tower_fillet_wall", "0"},    {"timelapse_type", "0"},          {"wipe_tower_no_sparse_layers", "0"},
                 {"enable_wrapping_detection", "0"},  {"prime_tower_infill_gap", "150"},
                 {"filament_change_length", fcl},     {"filament_change_length_nc", fcl_nc}};
        r.categories.assign(nfil, 0);
        return r;
    };
    auto add_layers = [](RunSpec &r, int n, float h, const std::vector<std::vector<int>> &tools, float vol) {
        float z = 0.f;
        for (int i = 0; i < n; ++i) {
            z += h;
            RunSpec::Layer l;
            l.z = z;
            l.h = h;
            for (int t : tools[i % tools.size()]) l.changes.push_back({t, vol, vol * 1.25f, vol});
            r.layers.push_back(l);
        }
    };

    // Two filaments, a change on every layer.
    {
        RunSpec r = base("two_filaments", 2);
        add_layers(r, 6, 0.2f, {{1}, {0}}, 45.f);
        specs.push_back(r);
    }
    // Three filaments, several changes per layer, some layers without any.
    {
        RunSpec r = base("three_filaments", 3);
        r.first_layer_flow_ratio = 0.95f;
        add_layers(r, 8, 0.2f, {{1, 2}, {}, {0}, {2, 1, 0}}, 30.f);
        specs.push_back(r);
    }
    // Gap wall with arc fitting and the rib wall.
    {
        RunSpec r = base("rib_gap_wall", 2);
        for (auto &kv : r.cfg) {
            if (kv.first == "prime_tower_skip_points") kv.second = "1";
            if (kv.first == "prime_tower_rib_wall") kv.second = "1";
            if (kv.first == "prime_tower_fillet_wall") kv.second = "1";
            if (kv.first == "enable_arc_fitting") kv.second = "1";
        }
        add_layers(r, 5, 0.16f, {{1}, {0}, {}}, 50.f);
        specs.push_back(r);
    }
    // Smooth timelapse.
    {
        RunSpec r = base("timelapse", 2);
        for (auto &kv : r.cfg) if (kv.first == "timelapse_type") kv.second = "1";
        add_layers(r, 5, 0.2f, {{1}, {}, {0}}, 40.f);
        specs.push_back(r);
    }
    // Two adhesiveness categories (two blocks).
    {
        RunSpec r = base("two_categories", 3);
        for (auto &kv : r.cfg) {
            if (kv.first == "filament_adhesiveness_category") kv.second = "0,0,1";
            if (kv.first == "filament_type") kv.second = "PLA;PLA;PETG";
        }
        r.categories = {0, 0, 1};
        add_layers(r, 6, 0.2f, {{2}, {1}, {0, 2}}, 35.f);
        specs.push_back(r);
    }
    // Two extruders (nozzle change: ramming).
    {
        RunSpec r = base("two_extruders", 2);
        r.filament_nozzle_map = {0, 1};
        r.nozzle_diameters = {"0.4", "0.4"};
        r.cfg.push_back({"nozzle_diameter", "0.4,0.4"});
        r.cfg.push_back({"filament_map", "1,2"});
        for (const char *k : {"default_acceleration", "initial_layer_acceleration", "travel_acceleration", "initial_layer_travel_acceleration"})
            r.cfg.push_back({k, "4000,5000"});
        r.cfg.push_back({"extruder_printable_height", "250,240"});
        r.cfg.push_back({"hotend_heating_rate", "2,2.5"});
        r.cfg.push_back({"hotend_cooling_rate", "2,1.5"});
        r.cfg.push_back({"physical_extruder_map", "0,1"});
        add_layers(r, 5, 0.2f, {{1}, {0}}, 45.f);
        specs.push_back(r);
    }
    auto set = [](RunSpec &r, const std::string &k, const std::string &v) {
        for (auto &kv : r.cfg)
            if (kv.first == k) { kv.second = v; return; }
        r.cfg.push_back({k, v});
    };
    // Two nozzle diameters: the Z ladder and the merged slabs.
    {
        RunSpec r = base("merged_slabs", 2);
        r.filament_nozzle_map = {0, 1};
        r.nozzle_diameters = {"0.4", "0.8"};
        set(r, "nozzle_diameter", "0.4,0.8");
        set(r, "filament_map", "1,2");
        for (const char *k : {"default_acceleration", "initial_layer_acceleration", "travel_acceleration", "initial_layer_travel_acceleration"})
            set(r, k, "4000,5000");
        set(r, "extruder_printable_height", "250,240");
        set(r, "hotend_heating_rate", "2,2.5");
        set(r, "hotend_cooling_rate", "2,1.5");
        set(r, "physical_extruder_map", "0,1");
        r.multi_diameter = true;
        r.grid_reference = 0.4f;
        r.grid_layer_height = 0.1f;
        add_layers(r, 10, 0.1f, {{1}, {}, {}, {0}, {}, {1, 0}}, 30.f);
        specs.push_back(r);
    }
    // The diagonal side scrape, on both pairs of walls.
    for (int face = 0; face < 2; ++face) {
        RunSpec r = base(face == 0 ? "side_scrape_lr" : "side_scrape_fb", 2);
        set(r, "prime_tower_side_scrape", "1");
        set(r, "prime_tower_side_scrape_face", face == 0 ? "left_right" : "front_back");
        set(r, "prime_tower_brim_width", face == 0 ? "3" : "0");
        add_layers(r, 22, 0.2f, {{1}, {0}, {}}, 45.f);
        specs.push_back(r);
    }
    // Gap wall with flat ironing.
    {
        RunSpec r = base("flat_ironing", 2);
        set(r, "prime_tower_skip_points", "1");
        set(r, "prime_tower_flat_ironing", "1");
        set(r, "filament_tower_ironing_area", "4,6");
        add_layers(r, 5, 0.2f, {{1}, {0}}, 45.f);
        specs.push_back(r);
    }
    // Tower interface features: contact layers between PLA and PETG.
    {
        RunSpec r = base("interface", 2);
        set(r, "enable_tower_interface_features", "1");
        set(r, "filament_type", "PLA;PETG");
        set(r, "filament_adhesiveness_category", "0,1");
        r.categories = {0, 1};
        add_layers(r, 6, 0.2f, {{1}, {0}, {1}}, 40.f);
        specs.push_back(r);
    }
    // Sparse layers skipped.
    {
        RunSpec r = base("no_sparse_layers", 2);
        set(r, "wipe_tower_no_sparse_layers", "1");
        add_layers(r, 8, 0.2f, {{1}, {}, {}, {0}}, 45.f);
        specs.push_back(r);
    }
    // Tail slowdown and a speed factor curve.
    {
        RunSpec r = base("tail_slowdown", 2);
        set(r, "prime_tower_tail_speed_factor", "50%");
        set(r, "prime_tower_tail_slowdown_volume", "8");
        set(r, "filament_prime_tower_speed_factor_curve", "\"1,0.8,0.5\";\"\"");
        add_layers(r, 5, 0.2f, {{1}, {0}}, 45.f);
        specs.push_back(r);
    }
    // Two extruders with pre-cooling and the ramming travel.
    {
        RunSpec r = base("precool", 2);
        r.filament_nozzle_map = {0, 1};
        r.nozzle_diameters = {"0.4", "0.4"};
        set(r, "nozzle_diameter", "0.4,0.4");
        set(r, "filament_map", "1,2");
        for (const char *k : {"default_acceleration", "initial_layer_acceleration", "travel_acceleration", "initial_layer_travel_acceleration"})
            set(r, k, "4000,5000");
        set(r, "extruder_printable_height", "250,240");
        set(r, "hotend_heating_rate", "2,2.5");
        set(r, "hotend_cooling_rate", "2,1.5");
        set(r, "physical_extruder_map", "0,1");
        set(r, "enable_pre_heating", "1");
        set(r, "filament_pre_cooling_temperature", "180");
        set(r, "filament_pre_cooling_temperature_nc", "190");
        set(r, "filament_ramming_travel_time", "2");
        set(r, "filament_ramming_travel_time_nc", "1.5");
        add_layers(r, 5, 0.2f, {{1}, {0}}, 45.f);
        specs.push_back(r);
    }
    // A TPU filament.
    {
        RunSpec r = base("tpu", 2);
        set(r, "filament_type", "PLA;TPU");
        r.has_tpu = true;
        add_layers(r, 5, 0.2f, {{1}, {0}}, 45.f);
        specs.push_back(r);
    }
    // Internal ribs (framework) and an automatic brim.
    {
        RunSpec r = base("framework", 3);
        set(r, "prime_tower_enable_framework", "1");
        set(r, "prime_tower_brim_width", "-1");
        add_layers(r, 6, 0.2f, {{1}, {2}, {0}}, 25.f);
        specs.push_back(r);
    }
    return specs;
}

static void gen_runs()
{
    for (const RunSpec &r : run_specs())
        gen_run(r);
}

int main(int argc, char **argv)
{
    if (argc < 2) {
        fprintf(stderr, "usage: gen <output directory>\n");
        return 1;
    }
    std::string dir = argv[1];
    g_out = fopen((dir + "/free.txt").c_str(), "w");
    gen_free();
    fclose(g_out);
    g_out = fopen((dir + "/writer.txt").c_str(), "w");
    gen_writer();
    fclose(g_out);
    if (argc < 3 || std::string(argv[2]) != "--no-runs") {
        g_out = fopen((dir + "/runs.txt").c_str(), "w");
        gen_runs();
        fclose(g_out);
    }
    return 0;
}
