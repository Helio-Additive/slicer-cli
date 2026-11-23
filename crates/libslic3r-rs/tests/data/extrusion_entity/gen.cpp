// Vectors for tests/extrusion_entity_native_parity.rs: ExtrusionPath, ExtrusionLoop and
// ExtrusionMultiPath (ExtrusionEntity.cpp/.hpp at the pin) on LCG loops cut into 1 to 4 paths.
//
// Usage: gen <cases>   (writes to stdout)
//
// First one line per role: R <role> <is_perimeter> <is_infill> <is_top_surface> <is_solid_infill> <is_bridge> <is_support>
// <string_to_role(s)> <s = role_to_string(role), to the end of the line>; then RS <string_to_role(s)> <s> for a few other strings.
// One case is a block of lines:
//   C <case> <npaths>
//   p <role> <mm3 double> <width float> <height float> <overhang double> <n> <x y>*n     (npaths times)
//   then one line per result, first token the tag:
//   LEN <loop.length()> ; PL <path.length()>* ; MIN <loop.min_mm3_per_mm()> ; CCW <0/1> ; POLY <n> <x y>*n
//   MCW <ret> <paths> ; MCCW <ret> <paths> ; HOP <x> <y> <ret> (3 times) ; HOPP <ret>
//   SAV <x> <y> <eps double> <ret> <paths> ; SA <x> <y> <prefer> <eps double> <paths> ; SAD <x> <y> <prefer> <paths>
//   CE <dist double> <paths> ; CW <eps float> <polygons> ; CS <eps float> <polygons>
//   MPL <len> ; MPMIN <min> ; MPPL <n> <x y>*n
//   KIN <entity> ; KIC <items_count> ; KF0 <entity> ; KF1 <entity> ; KMIN <min> ; KTV <total_volume>
//   KAPP <entity> (extrusion_entities_append_loops then _loops_and_paths into a collection)
//   KCH <x> <y> <role> <entity> ; KREV <entity> ; KCW <eps float> <polygons>   (a random nested collection, see pent())
// <paths> is <count> then per path <role> <mm3 double> <n> <x y>*n; <polygons> is <count> then per polygon <n> <x y>*n.
// Integers are decimal, doubles the 16 hex digits of their bits, floats the 8 hex digits of theirs.
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <algorithm>
#include <vector>
#include <string>
#include "libslic3r/libslic3r.h"
#include "libslic3r/Point.hpp"
#include "libslic3r/Polyline.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"

using namespace Slic3r;

static uint64_t g_state = 0x9E3779B97F4A7C15ull;
static uint32_t lcg()
{
    g_state = g_state * 6364136223846793005ull + 1442695040888963407ull;
    return (uint32_t)(g_state >> 33);
}
static int32_t rnd(int32_t range) { return (int32_t)(lcg() % (uint32_t)(2 * range + 1)) - range; }
static uint32_t below(uint32_t n) { return lcg() % n; }
static void pd(double v)
{
    uint64_t b;
    std::memcpy(&b, &v, 8);
    std::printf(" %016llx", (unsigned long long)b);
}
static void pf(float v)
{
    uint32_t b;
    std::memcpy(&b, &v, 4);
    std::printf(" %08x", (unsigned)b);
}
static void ppts(const Points &pts)
{
    std::printf(" %d", (int)pts.size());
    for (const Point &p : pts) std::printf(" %lld %lld", (long long)p.x(), (long long)p.y());
}
static void ppaths(const ExtrusionPaths &paths)
{
    std::printf(" %d", (int)paths.size());
    for (const ExtrusionPath &p : paths) {
        std::printf(" %d", (int)p.role());
        pd(p.mm3_per_mm);
        ppts(p.polyline.points);
    }
}
static void ppolys(const Polygons &polys)
{
    std::printf(" %d", (int)polys.size());
    for (const Polygon &p : polys) ppts(p.points);
}

static const ExtrusionRole kRoles[] = {erPerimeter, erExternalPerimeter, erOverhangPerimeter, erBridgeInfill, erSolidInfill};


// Collections: <entity> is P <path> | M <count> <path>* | L <count> <path>* | K <no_sort> <count> <entity>*
// (a path inside an entity is <role> <mm3 double> <width float> <height float> <points>)
static void ppathw(const ExtrusionPath &p)
{
    std::printf(" %d", (int)p.role()); pd(p.mm3_per_mm); pf(p.width); pf(p.height); ppts(p.polyline.points);
}
static void ppathsw(const ExtrusionPaths &paths)
{
    std::printf(" %d", (int)paths.size());
    for (const ExtrusionPath &p : paths) ppathw(p);
}
static void pent(const ExtrusionEntity *e)
{
    if (auto *p = dynamic_cast<const ExtrusionPath *>(e)) {
        std::printf(" P"); ppathw(*p);
    } else if (auto *m = dynamic_cast<const ExtrusionMultiPath *>(e)) {
        std::printf(" M"); ppathsw(m->paths);
    } else if (auto *l = dynamic_cast<const ExtrusionLoop *>(e)) {
        std::printf(" L"); ppathsw(l->paths);
    } else if (auto *c = dynamic_cast<const ExtrusionEntityCollection *>(e)) {
        std::printf(" K %d %d", c->no_sort ? 1 : 0, (int)c->entities.size());
        for (const ExtrusionEntity *x : c->entities) pent(x);
    } else std::abort();
}
static ExtrusionPath rpath(ExtrusionRole role, const Points &pts)
{
    float  width  = 0.3f + 0.5f * (float)below(1000) / 1000.f;
    float  height = is_bridge(role) ? width : 0.08f + 0.2f * (float)below(1000) / 1000.f;
    double mm3    = 0.01 + (double)below(100000) / 1000000.;
    ExtrusionPath path(0., 0, role, mm3, width, height);
    path.polyline.points = pts;
    return path;
}
static Points rpolyline(int n, int cx, int cy, int R)
{
    Points pts;
    Point  p(cx + rnd(R), cy + rnd(R));
    for (int i = 0; i < n; ++i) { pts.push_back(p); p += Point(rnd(R / 2), rnd(R / 2)); }
    return pts;
}
static void rentities(ExtrusionEntityCollection &out, int depth)
{
    int cnt = 1 + (int)below(depth == 0 ? 10 : 4);
    for (int i = 0; i < cnt; ++i) {
        int kind = (int)below(depth < 2 ? 4 : 3);
        int cx = rnd(3000000), cy = rnd(3000000), R = 20000 + (int)below(800000);
        if (kind == 0) {
            out.append(rpath(kRoles[below(5)], rpolyline(2 + (int)below(6), cx, cy, R)));
        } else if (kind == 1) {
            Points pts = rpolyline(3 + (int)below(8), cx, cy, R);
            ExtrusionPaths paths;
            size_t a = 0;
            while (a + 1 < pts.size()) {
                size_t b = std::min(pts.size() - 1, a + 1 + below(3));
                paths.push_back(rpath(kRoles[below(5)], Points(pts.begin() + a, pts.begin() + b + 1)));
                a = b;
            }
            out.append(ExtrusionMultiPath(paths));
        } else if (kind == 2) {
            // a closed loop: a triangle fan of 3..8 points, cut into 1..3 paths
            int    n = 3 + (int)below(6);
            Points pts;
            for (int j = 0; j < n; ++j) {
                double a = 2. * M_PI * (double)j / (double)n;
                pts.emplace_back(cx + (coord_t)std::lround(R * std::cos(a)), cy + (coord_t)std::lround(R * std::sin(a)));
            }
            if (below(2)) std::reverse(pts.begin(), pts.end());
            ExtrusionPaths paths;
            int a = 0;
            while (a < n) {
                int b = std::min(n, a + 1 + (int)below(n));
                Points pp;
                for (int j = a; j <= b; ++j) pp.push_back(pts[j % n]);
                paths.push_back(rpath(kRoles[below(5)], pp));
                a = b;
            }
            out.append(ExtrusionLoop(paths, elrDefault));
        } else {
            ExtrusionEntityCollection sub;
            sub.no_sort = below(3) == 0;
            rentities(sub, depth + 1);
            out.append(sub);
        }
    }
}

int main(int argc, char **argv)
{
    int cases = argc > 1 ? std::atoi(argv[1]) : 1000;
    // The role helpers (ExtrusionEntity.hpp:92-135) and role_to_string / string_to_role (ExtrusionEntity.cpp:613-689).
    for (int r = 0; r < (int)erCount; ++r) {
        ExtrusionRole role = ExtrusionRole(r);
        std::string   s    = ExtrusionEntity::role_to_string(role);
        std::printf("R %d %d %d %d %d %d %d %d %s\n", r, is_perimeter(role) ? 1 : 0, is_infill(role) ? 1 : 0, is_top_surface(role) ? 1 : 0,
                    is_solid_infill(role) ? 1 : 0, is_bridge(role) ? 1 : 0, is_support(role) ? 1 : 0,
                    (int)ExtrusionEntity::string_to_role(s), s.c_str());
    }
    for (const char *s : {"inner wall", "Wall", "Bridge infill", "Supports", "Prime", "Multiple roles"})
        std::printf("RS %d %s\n", (int)ExtrusionEntity::string_to_role(s), s);
    for (int c = 0; c < cases; ++c) {
        // A star-shaped loop around a center, angles sorted, possibly clockwise.
        int    n  = 4 + (int)below(21);
        int    R  = 100000 + (int)below(4000000);
        int    cx = rnd(5000000), cy = rnd(5000000);
        std::vector<double> ang;
        for (int i = 0; i < n; ++i) ang.push_back((double)below(1000000) * (2. * M_PI / 1000000.));
        std::sort(ang.begin(), ang.end());
        ang.erase(std::unique(ang.begin(), ang.end()), ang.end());
        n = (int)ang.size();
        Points pts;
        for (int i = 0; i < n; ++i) {
            double r = (double)R * (0.5 + 0.5 * (double)below(1000) / 1000.);
            // integer arithmetic only on the output side: the polar inputs go through lround of products
            pts.emplace_back(cx + (coord_t)std::lround(r * std::cos(ang[i])), cy + (coord_t)std::lround(r * std::sin(ang[i])));
        }
        if (below(8) == 0 && n > 4) pts.insert(pts.begin() + 2, pts[2]); // a repeated vertex
        if (below(2)) std::reverse(pts.begin(), pts.end());
        n = (int)pts.size();
        // Cut into k paths: path i runs from cut[i] to cut[i+1] inclusive, the last one closes the loop.
        int k = 1 + (int)below(4);
        std::vector<int> cuts{0};
        for (int i = 1; i < k; ++i) cuts.push_back(1 + (int)below(n - 1));
        std::sort(cuts.begin(), cuts.end());
        cuts.erase(std::unique(cuts.begin(), cuts.end()), cuts.end());
        k = (int)cuts.size();
        ExtrusionPaths paths;
        for (int i = 0; i < k; ++i) {
            ExtrusionRole role   = kRoles[below(5)];
            float         width  = 0.3f + 0.5f * (float)below(1000) / 1000.f;
            float         height = is_bridge(role) ? width : 0.08f + 0.2f * (float)below(1000) / 1000.f;
            double        mm3    = 0.01 + (double)below(100000) / 1000000.;
            double        od     = (double)below(11) * 0.1 + (below(4) == 0 ? 5. : 0.);
            ExtrusionPath path(od, (int)below(3), role, mm3, width, height);
            int a = cuts[i], b = i + 1 < k ? cuts[i + 1] : n;
            for (int j = a; j <= b; ++j) path.polyline.points.push_back(pts[j % n]);
            paths.push_back(path);
        }
        ExtrusionLoop loop(paths, elrDefault);
        std::printf("C %d %d\n", c, (int)paths.size());
        for (const ExtrusionPath &p : paths) {
            std::printf("p %d", (int)p.role());
            pd(p.mm3_per_mm);
            pf(p.width);
            pf(p.height);
            pd(p.overhang_degree);
            ppts(p.polyline.points);
            std::printf("\n");
        }
        std::printf("LEN"); pd(loop.length()); std::printf("\n");
        std::printf("PL"); for (const ExtrusionPath &p : paths) pd(p.length()); std::printf("\n");
        std::printf("MIN"); pd(loop.min_mm3_per_mm()); std::printf("\n");
        std::printf("CCW %d\n", loop.is_counter_clockwise() ? 1 : 0);
        std::printf("POLY"); ppts(loop.polygon().points); std::printf("\n");
        { ExtrusionLoop l = loop; bool r = l.make_clockwise(); std::printf("MCW %d", r ? 1 : 0); ppaths(l.paths); std::printf("\n"); }
        { ExtrusionLoop l = loop; bool r = l.make_counter_clockwise(); std::printf("MCCW %d", r ? 1 : 0); ppaths(l.paths); std::printf("\n"); }
        for (int q = 0; q < 3; ++q) {
            const ExtrusionPath &pp = paths[below(paths.size())];
            Point pt = q == 2 ? Point(cx + rnd(R), cy + rnd(R)) : pp.polyline.points[below(pp.polyline.points.size())];
            std::printf("HOP %lld %lld %d\n", (long long)pt.x(), (long long)pt.y(), loop.has_overhang_point(pt) ? 1 : 0);
        }
        std::printf("HOPP %d\n", loop.has_overhang_paths() ? 1 : 0);
        for (int q = 0; q < 2; ++q) {
            const ExtrusionPath &pp = paths[below(paths.size())];
            Point  pt  = pp.polyline.points[below(pp.polyline.points.size())] + Point(rnd(120), rnd(120));
            double eps = below(3) == 0 ? 0. : scaled<double>(0.001) * (double)(1 + below(2));
            ExtrusionLoop l = loop;
            bool r = l.split_at_vertex(pt, eps);
            std::printf("SAV %lld %lld", (long long)pt.x(), (long long)pt.y()); pd(eps); std::printf(" %d", r ? 1 : 0); ppaths(l.paths); std::printf("\n");
        }
        for (int q = 0; q < 3; ++q) {
            const ExtrusionPath &pp = paths[below(paths.size())];
            Point pt = pp.polyline.points[below(pp.polyline.points.size())] + Point(rnd(R / 3), rnd(R / 3));
            if (q == 2) pt = pp.polyline.points[below(pp.polyline.points.size())] + Point(rnd(150), rnd(150));
            bool   prefer = below(2) != 0;
            double eps    = below(2) ? 0. : (double)below(2000);
            { ExtrusionLoop l = loop; l.split_at(pt, prefer, eps); std::printf("SA %lld %lld %d", (long long)pt.x(), (long long)pt.y(), prefer ? 1 : 0); pd(eps); ppaths(l.paths); std::printf("\n"); }
            { ExtrusionLoop l = loop; l.split_at(pt, prefer); std::printf("SAD %lld %lld %d", (long long)pt.x(), (long long)pt.y(), prefer ? 1 : 0); ppaths(l.paths); std::printf("\n"); }
        }
        {
            double dist = loop.length() * (double)below(1100) / 1000.;
            ExtrusionPaths out;
            loop.clip_end(dist, &out);
            std::printf("CE"); pd(dist); ppaths(out); std::printf("\n");
        }
        {
            float eps = below(2) ? 0.f : (float)scaled<double>(0.0001) * (float)below(10);
            Polygons out;
            loop.polygons_covered_by_width(out, eps);
            std::printf("CW"); pf(eps); ppolys(out); std::printf("\n");
            Polygons out2;
            loop.polygons_covered_by_spacing(out2, eps);
            std::printf("CS"); pf(eps); ppolys(out2); std::printf("\n");
        }
        {
            ExtrusionMultiPath mp(paths);
            std::printf("MPL"); pd(mp.length()); std::printf("\n");
            std::printf("MPMIN"); pd(mp.min_mm3_per_mm()); std::printf("\n");
            std::printf("MPPL"); ppts(mp.as_polyline().points); std::printf("\n");
        }
        {
            // ExtrusionEntityCollection.cpp / .hpp on a random nested collection
            ExtrusionEntityCollection coll;
            coll.no_sort = below(6) == 0;
            rentities(coll, 0);
            std::printf("KIN"); pent(&coll); std::printf("\n");
            {
                // ExtrusionEntity.hpp:732-763 extrusion_entities_append_loops / _loops_and_paths
                ExtrusionEntityCollection app;
                Polygon poly = loop.polygon();
                Polygons loops{poly, Polygon(Points{poly.points[0]}), poly};
                extrusion_entities_append_loops(app.entities, std::move(loops), erPerimeter, 0.0123, 0.45f, 0.2f);
                Polylines pls{loop.paths.front().polyline, Polyline(Points{poly.points[0]}), loop.as_polyline()};
                extrusion_entities_append_loops_and_paths(app.entities, std::move(pls), erSolidInfill, 0.0456, 0.42f, 0.16f);
                std::printf("KAPP"); pent(&app); std::printf("\n");
            }
            std::printf("KIC %d\n", (int)coll.items_count());
            std::printf("KF0"); { ExtrusionEntityCollection f = coll.flatten(false); pent(&f); } std::printf("\n");
            std::printf("KF1"); { ExtrusionEntityCollection f = coll.flatten(true); pent(&f); } std::printf("\n");
            std::printf("KMIN"); pd(coll.min_mm3_per_mm()); std::printf("\n");
            std::printf("KTV"); pd(coll.total_volume()); std::printf("\n");
            {
                Point         start(rnd(4000000), rnd(4000000));
                ExtrusionRole role = below(2) ? erMixed : kRoles[below(5)];
                ExtrusionEntityCollection ch = coll.chained_path_from(start, role);
                std::printf("KCH %lld %lld %d", (long long)start.x(), (long long)start.y(), (int)role); pent(&ch); std::printf("\n");
            }
            { ExtrusionEntityCollection r = coll; r.reverse(); std::printf("KREV"); pent(&r); std::printf("\n"); }
            {
                float eps = below(2) ? 0.f : (float)scaled<double>(0.0001) * (float)below(10);
                Polygons out;
                coll.polygons_covered_by_width(out, eps);
                std::printf("KCW"); pf(eps); ppolys(out); std::printf("\n");
            }
        }
    }
    return 0;
}
