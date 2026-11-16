// Reference vectors of tests/paint_reproject_native_parity.rs: the functions of Bambu Studio's
// PaintReproject.cpp at the pin that paint_reproject.rs ports, called with deterministic random
// inputs.
//
// PaintReproject.cpp is included unchanged, so its static functions are called directly; the rest
// of the engine (Model, TriangleSelector, the logging) comes from the engine's own static
// libraries, linked as the `slicer_cli` program links them. Build, with REPO = this repository,
// BUILD = a configured and built tree of the Bambu Studio engine (`cmake -DENGINE=bambu`,
// docs/building.md) and DATA = this directory:
//
//   compile gen.cpp with the compile flags, include directories and definitions of a libslic3r
//   source of $BUILD (its `compile_commands.json`, e.g. the entry of PaintReproject.cpp: -O3
//   -DNDEBUG -std=c++17 and `-include $REPO/libslic3r/bambustudio/cgal_54_compat.hpp`), then
//   link it with the link line of `slicer_cli` ($BUILD/CMakeFiles/slicer_cli.dir/link.txt) with
//   main.cpp's objects replaced by gen.o.
//   for part in planar annotation; do
//     ./gen $part > $DATA/vectors_$part.txt
//   done
//
// The painted annotations are made with the pin's TriangleSelector::set_facets_with_subdivision
// (a split tree per painted face) and handed over serialized.
//
// Format: one call per line: a tag, the arguments, `->`, the result. A double is its 16 hex
// digits, a float its 8 hex digits, everything else decimal. A mesh is `nv` then 3*nv floats, `nf`
// then 3*nf indices; an annotation is the count of its entries, the (root, first bit) pairs, the
// count of its bits and the bits (`-` for none). A flag after the arguments of
// extract_source_roots and splice_congruent_paint says that they are given no bit ranges.

#include <cstdint>
#include <cstring>
#include "libslic3r/PaintReproject.cpp"

using namespace Slic3r;

static uint64_t g_state = 0x9E3779B97F4A7C15ull;
static uint64_t xs() {
    g_state ^= g_state << 13;
    g_state ^= g_state >> 7;
    g_state ^= g_state << 17;
    return g_state;
}
static double unit() { return (double) (xs() >> 11) / 9007199254740992.0; }
static double range(double lo, double hi) { return lo + (hi - lo) * unit(); }
static std::size_t below(std::size_t n) { return (std::size_t) (xs() % n); }

static void pd(double v) { uint64_t u; std::memcpy(&u, &v, 8); std::printf(" %016llx", (unsigned long long) u); }
static void pf(float v) { uint32_t u; std::memcpy(&u, &v, 4); std::printf(" %08x", u); }
static void pu(std::size_t v) { std::printf(" %zu", v); }
static void pi(long v) { std::printf(" %ld", v); }
static void tag(const char* t) { std::printf("%s", t); }
static void arrow() { std::printf(" ->"); }
static void nl() { std::printf("\n"); }
static void p2(const Vec2d& v) { pd(v.x()); pd(v.y()); }
static void p3f(const Vec3f& v) { pf(v.x()); pf(v.y()); pf(v.z()); }
static void ppoly(const std::vector<Vec2d>& p) { pu(p.size()); for (auto& v : p) p2(v); }
static void ptri(const std::array<Vec3f, 3>& t) { p3f(t[0]); p3f(t[1]); p3f(t[2]); }
static void pmesh(const indexed_triangle_set& m) {
    pu(m.vertices.size()); for (auto& v : m.vertices) p3f(v);
    pu(m.indices.size()); for (auto& f : m.indices) std::printf(" %d %d %d", f[0], f[1], f[2]);
}
static void pdata(const AnnotationData& d) {
    pu(d.first.size()); for (auto& p : d.first) { pi(p.first); pi(p.second); }
    pu(d.second.size()); std::printf(" "); if (d.second.empty()) std::printf("-"); for (bool b : d.second) std::printf("%d", (int) b);
}
static void ptrafo(const Transform3d& t) { for (int r = 0; r < 4; ++r) for (int c = 0; c < 4; ++c) pd(t.matrix()(r, c)); }

static Vec2d rnd2(double s) { return Vec2d(range(-s, s), range(-s, s)); }
static Vec3f rnd3(double s) { return Vec3f((float) range(-s, s), (float) range(-s, s), (float) range(-s, s)); }

static Transform3d rnd_trafo(int i) {
    if (i % 5 == 0) return Transform3d::Identity();
    Vec3d rot(range(-3, 3), range(-3, 3), range(-3, 3)), sc(range(0.5, 2), range(0.5, 2), range(0.5, 2));
    if (i % 5 == 1) sc = Vec3d(1, 1, 1);
    return Geometry::assemble_transform(Vec3d(range(-50, 50), range(-50, 50), range(-50, 50)), rot, sc, Vec3d(i % 7 == 3 ? -1 : 1, 1, 1));
}

static void gen_planar() {
    for (int i = 0; i < 100; ++i) {
        Vec2d a = rnd2(10), b = rnd2(10);
        if (i % 10 == 0) b = a * 2.0;
        tag("cross_2d"); p2(a); p2(b); arrow(); pd(cross_2d(a, b)); nl();
    }
    for (int i = 0; i < 30; ++i) {
        Vec3f p = rnd3(100);
        tag("project_to_2d"); p3f(p); pi(i % 4); arrow(); p2(project_to_2d(p, i % 4)); nl();
    }
    for (int i = 0; i < 60; ++i) {
        std::vector<Vec2d> poly;
        std::size_t n = below(8);
        for (std::size_t k = 0; k < n; ++k) poly.push_back(rnd2(5));
        tag("polygon_signed_area"); ppoly(poly); arrow(); pd(polygon_signed_area(poly)); nl();
    }
    for (int i = 0; i < 300; ++i) {
        std::vector<Vec2d> poly;
        std::size_t n = 3 + below(4);
        Vec2d o = rnd2(1);
        for (std::size_t k = 0; k < n; ++k) poly.push_back(o + rnd2(2));
        if (i % 9 == 0) poly.clear();
        std::array<Vec2d, 3> clip = {rnd2(2), rnd2(2), rnd2(2)};
        if (i % 7 == 1) clip[2] = clip[0] + (clip[1] - clip[0]) * 0.5;      // collinear
        if (i % 7 == 2) clip[1] = clip[0];                                   // degenerate
        if (i % 7 == 3) { poly = {clip[0], clip[1], clip[2]}; }              // the same triangle
        if (i % 7 == 4) { poly = {clip[0] * 0.5, clip[1] * 0.5, clip[2] * 0.5}; }
        tag("clip_polygon_by_triangle"); ppoly(poly); p2(clip[0]); p2(clip[1]); p2(clip[2]); arrow();
        ppoly(clip_polygon_by_triangle(poly, clip)); nl();
    }
    for (int i = 0; i < 200; ++i) {
        std::array<Vec3f, 3> l = {rnd3(3), rnd3(3), rnd3(3)}, r = {rnd3(3), rnd3(3), rnd3(3)};
        if (i % 5 == 0) r = l;
        if (i % 5 == 1) { r = l; r[2] = (l[0] + l[1]) * 0.5f + Vec3f(0.1f, 0.2f, 0.f); }
        int axis = dominant_projection_axis(l);
        tag("triangle_overlap_area"); ptri(l); ptri(r); arrow(); pi(axis); pd(triangle_overlap_area(l, r, axis)); pd(triangle_overlap_area(l, r, (axis + 1) % 3)); nl();
    }
    for (int i = 0; i < 60; ++i) {
        std::array<Vec3f, 3> t = {rnd3(3), rnd3(3), rnd3(3)};
        if (i % 6 == 0) t = {Vec3f(0, 0, 0), Vec3f(1, 0, 0), Vec3f(0, 1, 0)};
        if (i % 6 == 1) t = {Vec3f(0, 0, 0), Vec3f(1, 1, 0), Vec3f(0, 1, 1)};
        if (i % 6 == 2) t = {Vec3f(1, 1, 1), Vec3f(2, 2, 2), Vec3f(3, 3, 3)};
        tag("dominant_projection_axis"); ptri(t); arrow(); pi(dominant_projection_axis(t)); nl();
    }
    for (int i = 0; i < 60; ++i) {
        Transform3d tr = rnd_trafo(i);
        std::array<Vec3f, 3> t = {rnd3(30), rnd3(30), rnd3(30)};
        tag("transform_triangle"); ptrafo(tr); ptri(t); arrow(); ptri(transform_triangle(t, tr)); nl();
    }
}

// A grid of nx by ny unit cells, z ~ noise.
static indexed_triangle_set grid(int nx, int ny, double noise) {
    indexed_triangle_set m;
    for (int y = 0; y <= ny; ++y)
        for (int x = 0; x <= nx; ++x) m.vertices.push_back(Vec3f((float) (x + range(-0.2, 0.2)), (float) (y + range(-0.2, 0.2)), (float) range(-noise, noise)));
    auto id = [&](int x, int y) { return y * (nx + 1) + x; };
    for (int y = 0; y < ny; ++y)
        for (int x = 0; x < nx; ++x) {
            int a = id(x, y), b = id(x + 1, y), c = id(x + 1, y + 1), d = id(x, y + 1);
            if (below(2)) { m.indices.push_back(Vec3i(a, b, c)); m.indices.push_back(Vec3i(a, c, d)); }
            else { m.indices.push_back(Vec3i(a, b, d)); m.indices.push_back(Vec3i(b, c, d)); }
        }
    return m;
}

// Paint on some faces of the mesh: bands of states along x, the faces across a band border
// subdivided by the pin's TriangleSelector.
static void paint(const indexed_triangle_set& its, FacetsAnnotation& out) {
    TriangleMesh mesh(its);
    TriangleSelector sel(mesh);
    std::vector<int> faces;
    for (int f = 0; f < (int) its.indices.size(); ++f) if (below(3) != 0) faces.push_back(f);
    const double band = range(0.4, 1.3), shift = range(0, 1);
    const int nstates = 2 + (int) below(5);
    auto state_at = [&](float x) { return (int) std::floor((x + shift) / band) % nstates; };
    auto eval = [&](int, const std::array<Vec3f, 3>& t) {
        TriangleSelector::FacetSubdivisionMeasurement m;
        const double area = 0.5 * double((t[1] - t[0]).cross(t[2] - t[0]).norm());
        int s0 = state_at(t[0].x()), s1 = state_at(t[1].x()), s2 = state_at(t[2].x());
        Vec3f c = (t[0] + t[1] + t[2]) / 3.f;
        int sc = state_at(c.x());
        m.dominant_state = EnforcerBlockerType(std::max(0, sc));
        m.surface_area = area;
        m.error_area = (s0 == s1 && s1 == s2) ? 0.0 : area;
        return m;
    };
    sel.set_facets_with_subdivision(faces, eval, 4000, 1e-3, 1e-8, 1 + (int) below(3));
    out.set(sel);
}

static Model g_model;
static ModelVolume* new_volume(const indexed_triangle_set& its) {
    ModelObject* obj = g_model.add_object();
    return obj->add_volume(TriangleMesh(its), ModelVolumeType::MODEL_PART, false);
}

static void gen_annotation() {
    for (int i = 0; i < 30; ++i) {
        indexed_triangle_set its = grid(2 + (int) below(5), 2 + (int) below(4), 0.3);
        ModelVolume* vol = new_volume(its);
        paint(its, vol->mmu_segmentation_facets);
        const FacetsAnnotation& ann = vol->mmu_segmentation_facets;
        const int roots = (int) its.indices.size();
        tag("mesh"); pmesh(its); nl();
        tag("annotation"); pdata(ann.get_data()); nl();

        {
            TriangleSelector sel(vol->mesh());
            sel.deserialize(ann.get_data(), true);
            tag("count_valid_nodes"); arrow(); pu(count_valid_nodes(sel)); nl();
        }

        // annotation_bit_ranges, over the annotation and spoilt copies of it
        for (int k = 0; k < 4; ++k) {
            ModelVolume* v2 = new_volume(its);
            AnnotationData d = ann.get_data();
            int rc = roots;
            if (k == 1 && d.first.size() > 1) std::swap(d.first[0], d.first[1]);
            if (k == 2 && !d.first.empty()) rc = d.first.back().first;
            if (k == 3 && !d.first.empty()) d.first.back().second = (int) d.second.size();
            // FacetsAnnotation takes its data only from a selector; a spoilt copy is written in
            tag("annotation_bit_ranges"); pdata(d); pi(rc); arrow();
            FacetsAnnotation& a2 = v2->supported_facets;
            const_cast<AnnotationData&>(a2.get_data()) = d;
            auto ranges = annotation_bit_ranges(a2, rc);
            pu(ranges.size()); for (auto& r : ranges) { pi(r.first); pi(r.second); }
            nl();
        }

        auto ranges = annotation_bit_ranges(ann, roots);
        {
            std::vector<char> mask(roots);
            for (auto& m : mask) m = below(3) == 0 ? 0 : 1;
            tag("extract_source_roots"); pu(mask.size()); for (char m : mask) pu((std::size_t) m); pu(i % 7 == 6); arrow();
            pdata(extract_source_roots(ann, mask, i % 7 == 6 ? std::vector<std::pair<int, int>>() : ranges)); nl();
        }
        {
            const std::size_t ndst = below(roots * 2);
            std::vector<int> map;
            std::vector<CutFaceOrigin> origin;
            for (std::size_t f = 0; f < ndst; ++f) {
                map.push_back((int) below(roots));
                origin.push_back((CutFaceOrigin) below(3));
            }
            tag("splice_congruent_paint"); pu(ndst); for (std::size_t f = 0; f < ndst; ++f) { pi(map[f]); pu((std::size_t) origin[f]); } pu(i % 7 == 5); arrow();
            pdata(splice_congruent_paint(ann, map, origin, i % 7 == 5 ? std::vector<std::pair<int, int>>() : ranges)); nl();
        }
        {
            // append_shifted_annotation onto the paint of a smaller mesh, offset by its face count
            indexed_triangle_set small = grid(1 + (int) below(2), 1 + (int) below(2), 0.1);
            ModelVolume* v3 = new_volume(small);
            if (i % 3 != 0) paint(small, v3->seam_facets);
            tag("append_shifted_annotation"); pdata(v3->seam_facets.get_data()); pi((long) small.indices.size()); arrow();
            append_shifted_annotation(v3->seam_facets, ann, (int) small.indices.size());
            pdata(v3->seam_facets.get_data()); nl();
        }
        {
            // classify_cut_faces: the destination is the source moved by the inverse of dst_to_src,
            // with some faces kept, some moved off (fragments), some without a source face
            Transform3d dst_to_src = rnd_trafo(i);
            Transform3d src_to_dst = dst_to_src.inverse();
            indexed_triangle_set dst;
            std::vector<int> map;
            for (std::size_t f = 0; f < its.indices.size(); ++f) {
                const int kind = (int) below(5);
                Vec3i face;
                for (int k = 0; k < 3; ++k) {
                    Vec3f v = its.vertices[its.indices[f][k]];
                    if (kind == 1 && k == 1) v += Vec3f(0.3f, 0.1f, 0.f);
                    if (kind == 2 && k == 2) v += Vec3f(1e-7f, 0.f, 0.f);
                    dst.vertices.push_back((src_to_dst * v.cast<double>()).cast<float>());
                    face[k] = (int) dst.vertices.size() - 1;
                }
                if (kind == 3) std::swap(face[0], face[1]);
                dst.indices.push_back(face);
                map.push_back(kind == 4 ? (below(2) ? -1 : roots + 1) : (int) f);
            }
            if (i % 4 == 1) map.resize(map.size() / 2);
            const TriangleMesh src_mesh(its), dst_mesh(dst);
            DestinationInSourceFrame dst_in_source(dst_mesh, dst_to_src);
            const float tol = float(CutCongruenceRelativeTolerance * (i % 6 == 2 ? 0.0 : 10.0));
            tag("mesh"); pmesh(dst); nl();
            tag("classify_cut_faces"); ptrafo(dst_to_src); pu(map.size()); for (int m : map) pi(m); pf(tol); arrow();
            auto c = classify_cut_faces(src_mesh, dst_mesh, map, dst_in_source, tol);
            pu(c.origin.size()); for (auto o : c.origin) pu((std::size_t) o);
            pu(c.fragment_faces.size()); for (char x : c.fragment_faces) pu((std::size_t) x);
            pu(c.fragment_source_roots.size()); for (char x : c.fragment_source_roots) pu((std::size_t) x);
            pu(c.congruent_count); pu(c.fragment_count); pu(c.invented_count);
            pu(dst_in_source.vertices().size()); for (auto& v : dst_in_source.vertices()) p3f(v);
            std::array<Vec3f, 3> t = {rnd3(5), rnd3(5), rnd3(5)};
            ptri(t); ptri(dst_in_source.to_source(t));
            nl();
        }
        for (int k = 0; k < 6; ++k) {
            int f = (int) below(its.indices.size());
            tag("face_samples"); pi(f); arrow();
            for (auto& s : face_samples(its, f)) p3f(s);
            nl();
        }
    }
}

int main(int argc, char** argv) {
    if (argc != 2) return 1;
    std::string what = argv[1];
    if (what == "planar") gen_planar();
    else if (what == "annotation") { g_state ^= 0x1111; gen_annotation(); }
    else return 1;
    return 0;
}
