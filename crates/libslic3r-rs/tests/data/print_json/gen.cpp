// Vectors for tests/print_json_native_parity.rs: the pin's JSON (de)serializers of the cached slicing
// data (Print.cpp:4500-5314). For each case the generator builds a structure from a fixed-seed LCG,
// writes it with the pin's to_json (`T <kind> <json>`), reads that text back with the pin's from_json
// (or extract_layer / extract_support_layer) and writes the result again (`R <kind> <json>`).
// Usage: gen > vectors.txt
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

using namespace Slic3r;

static uint64_t g_state = 0x9E3779B97F4A7C15ull;
static uint32_t lcg()
{
    g_state = g_state * 6364136223846793005ull + 1442695040888963407ull;
    return uint32_t(g_state >> 33);
}
static int rnd(int n) { return int(lcg() % uint32_t(n)); }
static double rnd_d()
{
    // Doubles with all their bits: a 52-bit mantissa from two draws, a varying scale.
    uint64_t m = (uint64_t(lcg()) << 21) ^ uint64_t(lcg());
    double v = double(m) / double(1ull << 53);
    static const double scales[] = {1., 10., 0.001, 1e6, 1e-7, 123.456, 1e17};
    v *= scales[rnd(7)];
    if (rnd(4) == 0) v = -v;
    if (rnd(10) == 0) v = double(rnd(7)) - 3.;
    return v;
}
static float rnd_f() { return float(rnd_d()); }

static Point rnd_point() { return Point(coord_t(int(lcg() % 4000001u) - 2000000), coord_t(int(lcg() % 4000001u) - 2000000)); }
static Points rnd_points(int lo, int hi)
{
    Points pts;
    int n = lo + rnd(hi - lo + 1);
    for (int i = 0; i < n; ++i) pts.push_back(rnd_point());
    return pts;
}
static ExPolygon rnd_expolygon()
{
    ExPolygon e;
    e.contour.points = rnd_points(3, 6);
    int holes = rnd(3);
    for (int h = 0; h < holes; ++h) {
        Polygon p;
        p.points = rnd_points(3, 5);
        e.holes.push_back(p);
    }
    return e;
}
static ExPolygons rnd_expolygons(int hi)
{
    ExPolygons out;
    int n = rnd(hi + 1);
    for (int i = 0; i < n; ++i) out.push_back(rnd_expolygon());
    return out;
}
static Surface rnd_surface()
{
    Surface s(SurfaceType(rnd(10)));
    s.expolygon = rnd_expolygon();
    s.thickness = rnd_d();
    s.thickness_layers = (unsigned short) rnd(65536);
    s.bridge_angle = rnd(3) == 0 ? -1. : rnd_d();
    s.extra_perimeters = (unsigned short) rnd(6);
    return s;
}
static ArcSegment rnd_arc()
{
    ArcSegment a;
    a.is_arc = rnd(2) == 1;
    a.length = rnd_d();
    a.angle_radians = rnd_d();
    a.polar_start_theta = rnd_d();
    a.polar_end_theta = rnd_d();
    a.start_point = rnd_point();
    a.end_point = rnd_point();
    a.direction = ArcDirection(rnd(3));
    a.radius = rnd_d();
    a.center = rnd_point();
    return a;
}
static Polyline rnd_polyline()
{
    Polyline p;
    p.points = rnd_points(2, 6);
    int nf = rnd(4);
    for (int i = 0; i < nf; ++i) {
        PathFittingData f;
        f.start_point_index = size_t(rnd(10));
        f.end_point_index = size_t(rnd(10));
        f.path_type = EMovePathType(rnd(4));
        f.arc_data = rnd_arc();
        // A fitting without an arc keeps the default ArcSegment: from_json reads none back.
        if (!f.arc_data.is_arc) f.arc_data = ArcSegment();
        p.fitting_result.push_back(f);
    }
    return p;
}
static ExtrusionPath rnd_path()
{
    ExtrusionPath path(ExtrusionRole(rnd(22)), rnd_d(), rnd_f(), rnd_f(), rnd(5) == 0);
    path.polyline = rnd_polyline();
    path.overhang_degree = rnd(3) == 0 ? rnd_d() : double(rnd(60)) / 10.;
    path.curve_degree = rnd(11) - 5;
    return path;
}
static ExtrusionEntity *rnd_entity(int depth)
{
    int kind = rnd(depth > 1 ? 3 : 4);
    if (kind == 0) return new ExtrusionPath(rnd_path());
    if (kind == 1) {
        ExtrusionMultiPath *mp = new ExtrusionMultiPath();
        int n = rnd(4);
        for (int i = 0; i < n; ++i) mp->paths.push_back(rnd_path());
        return mp;
    }
    if (kind == 2) {
        static const ExtrusionLoopRole roles[] = {elrDefault, elrContourInternalPerimeter, elrSkirt, elrPerimeterHole, elrSecondPerimeter,
                                                   ExtrusionLoopRole(elrPerimeterHole | elrSecondPerimeter)};
        ExtrusionLoop *loop = new ExtrusionLoop(roles[rnd(6)]);
        int n = 1 + rnd(3);
        for (int i = 0; i < n; ++i) loop->paths.push_back(rnd_path());
        return loop;
    }
    ExtrusionEntityCollection *c = new ExtrusionEntityCollection();
    c->no_sort = rnd(2) == 1;
    int n = rnd(4);
    for (int i = 0; i < n; ++i) c->entities.push_back(rnd_entity(depth + 1));
    return c;
}
static void rnd_collection(ExtrusionEntityCollection &c, int hi)
{
    c.no_sort = rnd(2) == 1;
    int n = rnd(hi + 1);
    for (int i = 0; i < n; ++i) c.entities.push_back(rnd_entity(0));
}
static AdaptiveMergePatch rnd_patch()
{
    AdaptiveMergePatch p;
    p.bottom_idx = size_t(rnd(100));
    p.top_idx = size_t(rnd(100));
    p.group_top = rnd(2) == 1;
    p.merged_height = rnd_d();
    p.set_area(rnd_expolygons(2));
    if (rnd(2) == 0) p.set_suppressed_area(rnd_expolygons(2));
    return p;
}
static void fill_region(LayerRegion &r)
{
    int n = rnd(3);
    for (int i = 0; i < n; ++i) r.slices.surfaces.push_back(rnd_surface());
    r.raw_slices = rnd_expolygons(2);
    rnd_collection(r.thin_fills, 2);
    r.fill_expolygons = rnd_expolygons(2);
    n = rnd(3);
    for (int i = 0; i < n; ++i) r.fill_surfaces.surfaces.push_back(rnd_surface());
    r.fill_no_overlap_expolygons = rnd_expolygons(2);
    n = rnd(3);
    for (int i = 0; i < n; ++i) r.unsupported_bridge_edges.push_back(rnd_polyline());
    rnd_collection(r.perimeters, 3);
    rnd_collection(r.fills, 3);
    n = rnd(3);
    for (int i = 0; i < n; ++i) r.adaptive_merge_patches.push_back(rnd_patch());
}

static json collection_json(const ExtrusionEntityCollection &c)
{
    json cj, entities = json::array();
    cj[JSON_EXTRUSION_NO_SORT] = c.no_sort;
    cj[JSON_EXTRUSION_ENTITY_TYPE] = JSON_EXTRUSION_TYPE_COLLECTION;
    for (const ExtrusionEntity *e : c.entities) {
        json ej, ep = json::array();
        if (!convert_extrusion_to_json(ej, ep, e)) continue;
        entities.push_back(std::move(ej));
    }
    cj[JSON_EXTRUSION_ENTITIES] = std::move(entities);
    return cj;
}

// The members export_cached_data writes for a layer (Print.cpp:5402-5460), as a test layout.
static json layer_json(const Layer &layer)
{
    json j, polys = json::array(), boxes = json::array(), over = json::array(), regions = json::array();
    for (const ExPolygon &e : layer.lslices) polys.push_back(json(e));
    for (const BoundingBox &b : layer.lslices_bboxes) boxes.push_back(json(b));
    for (const ExPolygon &e : layer.loverhangs) over.push_back(json(e));
    j[JSON_LAYER_SLICED_POLYGONS] = std::move(polys);
    j[JSON_LAYER_SLLICED_BBOXES] = std::move(boxes);
    j[JSON_LAYER_OVERHANG_POLYGONS] = std::move(over);
    j[JSON_LAYER_OVERHANG_BBOX] = layer.loverhangs_bbox;
    for (const LayerRegion *r : layer.regions()) regions.push_back(json(*r));
    j[JSON_LAYER_REGIONS] = std::move(regions);
    return j;
}

static void emit(const char *tag, const char *kind, const json &j) { printf("%s %s %s\n", tag, kind, j.dump().c_str()); }

int main()
{
    // Elements.
    for (int i = 0; i < 60; ++i) {
        Point a = rnd_point(), b = rnd_point();
        BoundingBox bb(Point(std::min(a.x(), b.x()), std::min(a.y(), b.y())), Point(std::max(a.x(), b.x()), std::max(a.y(), b.y())));
        json t = bb; emit("T", "bbox", t);
        BoundingBox back; back = t; emit("R", "bbox", json(back));
    }
    for (int i = 0; i < 100; ++i) {
        json t = rnd_surface(); emit("T", "surface", t);
        Surface back; back = t; emit("R", "surface", json(back));
    }
    for (int i = 0; i < 100; ++i) {
        json t = rnd_polyline(); emit("T", "polyline", t);
        Polyline back; back = t; emit("R", "polyline", json(back));
    }
    for (int i = 0; i < 100; ++i) {
        json t = rnd_path(); emit("T", "path", t);
        ExtrusionPath back; back = t; emit("R", "path", json(back));
    }
    for (int i = 0; i < 150; ++i) {
        std::unique_ptr<ExtrusionEntity> e(rnd_entity(0));
        json t, tp = json::array();
        convert_extrusion_to_json(t, tp, e.get());
        emit("T", "entity", t);
        ExtrusionEntityCollection back;
        bool ok = convert_extrusion_from_json(t, back);
        json r, rp = json::array();
        if (ok && back.entities.size() == 1) convert_extrusion_to_json(r, rp, back.entities.front());
        emit("R", "entity", r);
    }
    for (int i = 0; i < 60; ++i) {
        json t = adaptive_merge_patch_to_json(rnd_patch()); emit("T", "patch", t);
        emit("R", "patch", adaptive_merge_patch_to_json(adaptive_merge_patch_from_json(t)));
    }
    for (int i = 0; i < 40; ++i) {
        groupedVolumeSlices g;
        g.groupId = rnd(20) - 1;
        int n = rnd(4);
        for (int k = 0; k < n; ++k) g.volume_ids.push_back(ObjectID(size_t(lcg())));
        g.slices = rnd_expolygons(3);
        json t = g; emit("T", "group", t);
        groupedVolumeSlices back; back = t; emit("R", "group", json(back));
    }
    // Layers: a Layer with 1-3 regions written as export_cached_data lays it out, extract_layer into a
    // fresh Layer with the same regions, written again.
    PrintRegionConfig rcfg;
    std::vector<std::unique_ptr<PrintRegion>> print_regions;
    for (int k = 0; k < 3; ++k) print_regions.emplace_back(new PrintRegion(rcfg, size_t(0x1234567u + 977u * k)));
    for (int i = 0; i < 30; ++i) {
        int nreg = 1 + rnd(3);
        Layer layer(size_t(i), nullptr, 0.2, 0.2 * (i + 1), 0.2 * i + 0.1);
        layer.lslices = rnd_expolygons(3);
        for (const ExPolygon &e : layer.lslices) layer.lslices_bboxes.push_back(get_extents(e));
        layer.loverhangs = rnd_expolygons(2);
        layer.loverhangs_bbox = get_extents(layer.loverhangs);
        for (int k = 0; k < nreg; ++k) fill_region(*layer.add_region(print_regions[k].get()));
        json t = layer_json(layer);
        printf("T layer %d %s\n", nreg, t.dump().c_str());
        Layer back(size_t(i), nullptr, 0.2, 0.2 * (i + 1), 0.2 * i + 0.1);
        for (int k = 0; k < nreg; ++k) back.add_region(print_regions[k].get());
        extract_layer(t, back);
        emit("R", "layer", layer_json(back));
    }
    for (int i = 0; i < 20; ++i) {
        int nreg = 1 + rnd(2);
        SupportLayer layer(size_t(i), 0, nullptr, 0.2, 0.2 * (i + 1), 0.2 * i + 0.1);
        layer.lslices = rnd_expolygons(2);
        for (const ExPolygon &e : layer.lslices) layer.lslices_bboxes.push_back(get_extents(e));
        layer.loverhangs_bbox = get_extents(layer.loverhangs);
        for (int k = 0; k < nreg; ++k) fill_region(*layer.add_region(print_regions[k].get()));
        layer.support_islands = rnd_expolygons(3);
        rnd_collection(layer.support_fills, 3);
        json t = layer_json(layer);
        t[JSON_SUPPORT_LAYER_ISLANDS] = json::array();
        for (const ExPolygon &e : layer.support_islands) t[JSON_SUPPORT_LAYER_ISLANDS].push_back(json(e));
        t[JSON_SUPPORT_LAYER_FILLS] = collection_json(layer.support_fills);
        printf("T support %d %s\n", nreg, t.dump().c_str());
        SupportLayer back(size_t(i), 0, nullptr, 0.2, 0.2 * (i + 1), 0.2 * i + 0.1);
        for (int k = 0; k < nreg; ++k) back.add_region(print_regions[k].get());
        extract_support_layer(t, back);
        json r = layer_json(back);
        r[JSON_SUPPORT_LAYER_ISLANDS] = json::array();
        for (const ExPolygon &e : back.support_islands) r[JSON_SUPPORT_LAYER_ISLANDS].push_back(json(e));
        r[JSON_SUPPORT_LAYER_FILLS] = collection_json(back.support_fills);
        emit("R", "support", r);
    }
    return 0;
}
