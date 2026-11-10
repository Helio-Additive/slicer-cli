// PP probe, C++ side: checksums per layer and stage (sums that do not depend on the order,
// and an order-sensitive hash `h=`), the twin of crates/libslic3r-rs/src/stage_dump.rs (pp_*).
// `PPL=<layer>` with `PPS=<stage>` prints that layer's fill-surface points as `PPT` lines.
// For local diagnosis builds only: see docs/rust-engine/README.md, "Finding the first stage
// that differs". Never commit a probed engine file.
#pragma once
#include <cstdio>
#include <string>
#include <cstdlib>
#include "Layer.hpp"
#include "ExtrusionEntity.hpp"
#include "ExtrusionEntityCollection.hpp"
namespace Slic3r { namespace pp {
struct Acc { size_t n = 0, np = 0; long long sx = 0, sy = 0; unsigned long long h = 1469598103934665603ULL; };
inline void mix(Acc &a, long long v) { a.h ^= (unsigned long long)v; a.h *= 1099511628211ULL; }
inline bool on() { static int v = -1; if (v < 0) v = ::getenv("PP") ? 1 : 0; return v == 1; }
inline void add(Acc &a, const Points &pts) { a.n++; mix(a, -1); for (const Point &p : pts) { a.np++; a.sx += p.x(); a.sy += p.y(); mix(a, p.x()); mix(a, p.y()); } }
inline void add(Acc &a, const ExPolygon &e) { add(a, e.contour.points); for (const Polygon &h : e.holes) add(a, h.points); }
inline void add(Acc &a, const ExtrusionEntity *e) {
    if (auto *c = dynamic_cast<const ExtrusionEntityCollection*>(e)) { for (const ExtrusionEntity *x : c->entities) add(a, x); }
    else if (auto *l = dynamic_cast<const ExtrusionLoop*>(e)) { for (const ExtrusionPath &p : l->paths) add(a, p.polyline.points); }
    else if (auto *m = dynamic_cast<const ExtrusionMultiPath*>(e)) { for (const ExtrusionPath &p : m->paths) add(a, p.polyline.points); }
    else if (auto *p = dynamic_cast<const ExtrusionPath*>(e)) { add(a, p->polyline.points); }
}
inline void print(const char *stage, size_t layer, const Acc &a) {
    if (a.n == 0) return;
    fprintf(stderr, "PP %s L%zu n=%zu np=%zu sx=%lld sy=%lld h=%llx\n", stage, layer, a.n, a.np, a.sx, a.sy, a.h);
}
inline void slices(const LayerPtrs &layers) {
    if (!on()) return;
    for (size_t i = 0; i < layers.size(); ++i) {
        Acc a; for (const ExPolygon &e : layers[i]->lslices) add(a, e); print("lslices", i, a);
        Acc b; for (const LayerRegion *r : layers[i]->regions()) for (const Surface &s : r->slices.surfaces) add(b, s.expolygon); print("rslices", i, b);
    }
}
inline void detail(const char *tag, const ExtrusionEntity *e, int depth) {
    if (auto *c = dynamic_cast<const ExtrusionEntityCollection*>(e)) { fprintf(stderr, "PPD %s %*scoll n=%zu no_sort=%d\n", tag, depth*2, "", c->entities.size(), int(c->no_sort)); for (const ExtrusionEntity *x : c->entities) detail(tag, x, depth+1); }
    else if (auto *l = dynamic_cast<const ExtrusionLoop*>(e)) { fprintf(stderr, "PPD %s %*sloop paths=%zu role=%d\n", tag, depth*2, "", l->paths.size(), int(l->loop_role())); for (const ExtrusionPath &p : l->paths) detail(tag, &p, depth+1); }
    else if (auto *m = dynamic_cast<const ExtrusionMultiPath*>(e)) { fprintf(stderr, "PPD %s %*smulti paths=%zu\n", tag, depth*2, "", m->paths.size()); for (const ExtrusionPath &p : m->paths) detail(tag, &p, depth+1); }
    else if (auto *p = dynamic_cast<const ExtrusionPath*>(e)) { const Points &q = p->polyline.points; fprintf(stderr, "PPD %s %*spath role=%d np=%zu od=%.3f w=%.5f mm3=%.6f first=(%lld,%lld) last=(%lld,%lld)\n", tag, depth*2, "", int(p->role()), q.size(), p->overhang_degree, p->width, p->mm3_per_mm, q.empty()?0LL:(long long)q.front().x(), q.empty()?0LL:(long long)q.front().y(), q.empty()?0LL:(long long)q.back().x(), q.empty()?0LL:(long long)q.back().y()); }
}
inline void perimeters(const LayerPtrs &layers) {
    if (!on()) return;
    if (const char *ppl = ::getenv("PPL")) { size_t L = (size_t)atoi(ppl); if (L < layers.size()) for (const LayerRegion *r : layers[L]->regions()) detail("perim", &r->perimeters, 0); }
    if (const char *ppl = ::getenv("PPL")) { size_t L = (size_t)atoi(ppl); if (L < layers.size()) for (const LayerRegion *r : layers[L]->regions()) for (const Surface &x : r->fill_surfaces.surfaces) { auto ring=[&](const char*t,const Points&p){ for (const Point &q : p) fprintf(stderr,"PPF %s %lld %lld\n",t,(long long)q.x(),(long long)q.y()); }; ring("c",x.expolygon.contour.points); for (const Polygon &hh : x.expolygon.holes) ring("h",hh.points); } }
    for (size_t i = 0; i < layers.size(); ++i) {
        Acc a, b, c;
        for (const LayerRegion *r : layers[i]->regions()) {
            add(a, &r->perimeters); add(c, &r->thin_fills);
            for (const Surface &s : r->fill_surfaces.surfaces) add(b, s.expolygon);
        }
        print("perim", i, a); print("perim_fillsurf", i, b); print("perim_thin", i, c);
    }
}
inline void typed(const char *stage, const LayerPtrs &layers) {
    if (!on()) return;
    static const char *names[] = {"top","bottom","bottombridge","internal","fvs","internalsolid","subtop","internalbridge","internalvoid","perimeter","o10","o11"};
    char buf[96];
    for (size_t i = 0; i < layers.size(); ++i) {
        Acc s[12], f[12];
        for (const LayerRegion *r : layers[i]->regions()) {
            for (const Surface &x : r->slices.surfaces) { int k = int(x.surface_type); if (k >= 0 && k < 12) add(s[k], x.expolygon); }
            for (const Surface &x : r->fill_surfaces.surfaces) { int k = int(x.surface_type); if (k >= 0 && k < 12) add(f[k], x.expolygon); }
        }
        for (int k = 0; k < 12; ++k) { snprintf(buf, sizeof(buf), "%s_s_%s", stage, names[k]); print(buf, i, s[k]); snprintf(buf, sizeof(buf), "%s_f_%s", stage, names[k]); print(buf, i, f[k]); }
        { const char *ppl = ::getenv("PPL"); const char *pps = ::getenv("PPS");
          if (ppl && pps && (size_t)atoi(ppl) == i && std::string(pps) == stage)
            for (const LayerRegion *r : layers[i]->regions()) for (const Surface &x : r->fill_surfaces.surfaces) { int k = int(x.surface_type); if (k < 0 || k >= 12) continue;
                for (const Point &q : x.expolygon.contour.points) fprintf(stderr, "PPT %s c %lld %lld\n", names[k], (long long)q.x(), (long long)q.y());
                for (const Polygon &hh : x.expolygon.holes) for (const Point &q : hh.points) fprintf(stderr, "PPT %s h %lld %lld\n", names[k], (long long)q.x(), (long long)q.y()); } }
    }
}
inline void infill(const LayerPtrs &layers) {
    if (!on()) return;
    if (const char *ppl = ::getenv("PPL")) { size_t L = (size_t)atoi(ppl); if (L < layers.size()) for (const LayerRegion *r : layers[L]->regions()) detail("fills", &r->fills, 0); }
    static const char *names[] = {"prep_top","prep_bottom","prep_bottombridge","prep_internal","prep_fvs","prep_internalsolid","prep_subtop","prep_internalbridge","prep_internalvoid","prep_perimeter","prep_other10","prep_other11"};
    for (size_t i = 0; i < layers.size(); ++i) {
        Acc t[12]; Acc f;
        for (const LayerRegion *r : layers[i]->regions()) {
            for (const Surface &s : r->fill_surfaces.surfaces) { int k = int(s.surface_type); if (k >= 0 && k < 12) add(t[k], s.expolygon); }
            add(f, &r->fills);
        }
        for (int k = 0; k < 12; ++k) print(names[k], i, t[k]);
        print("fills", i, f);
    }
}
}}
