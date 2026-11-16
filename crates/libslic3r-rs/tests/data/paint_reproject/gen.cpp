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
//   for part in planar annotation aabb reproject volumes; do
//     ./gen $part > $DATA/vectors_$part.txt
//   done
//   gzip -9n -f $DATA/vectors_{aabb,reproject,volumes}.txt   # planar and annotation stay plain
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
static void ptrafof(const Transform3f& t) { for (int r = 0; r < 4; ++r) for (int c = 0; c < 4; ++c) pf(t.matrix()(r, c)); }

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


// ---------------------------------------------------------------------------------------------
// The float AABB tree, the samplers and the reprojection drivers.
// ---------------------------------------------------------------------------------------------

// A closed latitude-longitude sphere, outward oriented.
static indexed_triangle_set sphere(int rings, int segs, double radius) {
    indexed_triangle_set m;
    m.vertices.push_back(Vec3f(0.f, 0.f, (float) radius));
    for (int r = 1; r < rings; ++r) {
        double th = M_PI * double(r) / double(rings);
        for (int s = 0; s < segs; ++s) {
            double ph = 2. * M_PI * double(s) / double(segs) + range(-0.05, 0.05);
            double rr = radius * (1. + range(-0.05, 0.05));
            m.vertices.push_back(Vec3f((float) (rr * std::sin(th) * std::cos(ph)), (float) (rr * std::sin(th) * std::sin(ph)), (float) (rr * std::cos(th))));
        }
    }
    m.vertices.push_back(Vec3f(0.f, 0.f, (float) -radius));
    const int south = (int) m.vertices.size() - 1;
    auto id = [&](int r, int s) { return 1 + (r - 1) * segs + (s % segs); };
    for (int s = 0; s < segs; ++s) m.indices.push_back(Vec3i(0, id(1, s), id(1, s + 1)));
    for (int r = 1; r + 1 < rings; ++r)
        for (int s = 0; s < segs; ++s) {
            int a = id(r, s), b = id(r + 1, s), c = id(r + 1, s + 1), d = id(r, s + 1);
            m.indices.push_back(Vec3i(a, b, c));
            m.indices.push_back(Vec3i(a, c, d));
        }
    for (int s = 0; s < segs; ++s) m.indices.push_back(Vec3i(id(rings - 1, s), south, id(rings - 1, s + 1)));
    return m;
}

static void pbox(const AABBTreeIndirect::Tree3f::BoundingBox& b) { p3f(b.min()); p3f(b.max()); }

static void gen_aabb() {
    for (int i = 0; i < 40; ++i) {
        indexed_triangle_set its = below(3) == 0 ? sphere(3 + (int) below(5), 4 + (int) below(6), range(1, 5)) : grid(1 + (int) below(7), 1 + (int) below(6), range(0, 1));
        if (i % 9 == 4) { its.vertices.resize(3); its.indices.resize(1); its.indices[0] = Vec3i(0, 1, 2); }
        auto tree = AABBTreeIndirect::build_aabb_tree_over_indexed_triangle_set(its.vertices, its.indices);
        tag("mesh"); pmesh(its); nl();
        tag("aabb_build"); arrow(); pu(tree.nodes().size());
        for (auto& n : tree.nodes()) { pi(n.idx == size_t(-1) ? -1 : n.idx == size_t(-2) ? -2 : (long) n.idx); if (n.is_valid()) pbox(n.bbox); }
        nl();
        for (int k = 0; k < 40; ++k) {
            Vec3f p;
            const int kind = (int) below(4);
            if (kind == 0) p = its.vertices[below(its.vertices.size())];                                  // on a vertex
            else if (kind == 1) { const Vec3i& f = its.indices[below(its.indices.size())]; p = (its.vertices[f[0]] + its.vertices[f[1]]) * 0.5f; } // on an edge
            else p = rnd3(6);
            if (kind == 3) p += Vec3f(0.f, 0.f, 0.01f);
            size_t hit = size_t(-1); Vec3f hp(0, 0, 0);
            float d = AABBTreeIndirect::squared_distance_to_indexed_triangle_set(its.vertices, its.indices, tree, p, hit, hp);
            tag("aabb_distance"); p3f(p); arrow(); pf(d); pi((long) hit); p3f(hp); nl();
        }
        for (int k = 0; k < 10; ++k) {
            Vec3f a = rnd3(4), b = a + Vec3f((float) range(0, 3), (float) range(0, 3), (float) range(0, 3));
            AABBTreeIndirect::Tree3f::BoundingBox q(a, b);
            std::vector<size_t> out;
            AABBTreeIndirect::traverse(tree, AABBTreeIndirect::intersecting(q), [&out](const AABBTreeIndirect::Tree3f::Node& n) { out.push_back(n.idx); return true; });
            tag("aabb_traverse"); pbox(q); arrow(); pu(out.size()); for (size_t x : out) pu(x); nl();
        }
    }
}

// The destination of a repair: the source's faces re-triangulated, some split into four, moved by
// the inverse of dst_to_src.
static indexed_triangle_set retriangulate(const indexed_triangle_set& its, const Transform3d& src_to_dst) {
    indexed_triangle_set dst;
    for (auto& v : its.vertices) dst.vertices.push_back((src_to_dst * v.cast<double>()).cast<float>());
    for (auto& f : its.indices) {
        if (below(3) == 0) {
            const int a = f[0], b = f[1], c = f[2];
            auto mid = [&](int x, int y) { dst.vertices.push_back((src_to_dst * ((its.vertices[x] + its.vertices[y]) * 0.5f).cast<double>()).cast<float>()); return (int) dst.vertices.size() - 1; };
            const int ab = mid(a, b), bc = mid(b, c), ca = mid(c, a);
            dst.indices.push_back(Vec3i(a, ab, ca)); dst.indices.push_back(Vec3i(ab, b, bc));
            dst.indices.push_back(Vec3i(ca, bc, c)); dst.indices.push_back(Vec3i(ab, bc, ca));
        } else
            dst.indices.push_back(f);
    }
    return dst;
}

// The destination of a cut: faces kept (congruent), faces split into fragments, faces invented,
// with the destination -> source face map.
static indexed_triangle_set cut(const indexed_triangle_set& its, const Transform3d& src_to_dst, std::vector<int>& map) {
    indexed_triangle_set dst;
    map.clear();
    auto add = [&](const Vec3f& v) { dst.vertices.push_back((src_to_dst * v.cast<double>()).cast<float>()); return (int) dst.vertices.size() - 1; };
    for (int f = 0; f < (int) its.indices.size(); ++f) {
        const Vec3f &a = its.vertices[its.indices[f][0]], &b = its.vertices[its.indices[f][1]], &c = its.vertices[its.indices[f][2]];
        const int kind = (int) below(4);
        if (kind <= 1) { dst.indices.push_back(Vec3i(add(a), add(b), add(c))); map.push_back(f); }
        else if (kind == 2) {
            const float t = (float) range(0.2, 0.8);
            const Vec3f m = a + (b - a) * t;
            dst.indices.push_back(Vec3i(add(a), add(m), add(c))); map.push_back(f);
            dst.indices.push_back(Vec3i(add(m), add(b), add(c))); map.push_back(f);
        } else {
            const Vec3f g = (a + b + c) / 3.f;
            dst.indices.push_back(Vec3i(add(a), add(b), add(g))); map.push_back(f);
            dst.indices.push_back(Vec3i(add(b), add(c), add(g))); map.push_back(f);
            dst.indices.push_back(Vec3i(add(c), add(a), add(g))); map.push_back(f);
        }
        if (below(12) == 0) { dst.indices.push_back(Vec3i(add(a + Vec3f(0, 0, 0.5f)), add(b + Vec3f(0, 0, 0.5f)), add(c + Vec3f(0, 0, 0.5f)))); map.push_back(-1); }
    }
    return dst;
}

static Transform3d rnd_pose(int i) {
    if (i % 3 == 0) return Transform3d::Identity();
    Vec3d rot(range(-3, 3), range(-3, 3), range(-3, 3));
    return Geometry::assemble_transform(Vec3d(range(-20, 20), range(-20, 20), range(-20, 20)), rot, Vec3d(1, 1, 1), Vec3d(1, 1, 1));
}

struct Calls { int progress = 0; long percent_sum = 0; int last = -1; int cancel = 0; int cancel_at = 0; };
static void pcalls(const Calls& c) { pi(c.progress); pi(c.percent_sum); pi(c.last); pi(c.cancel); }

static void gen_reproject() {
    for (int i = 0; i < 24; ++i) {
        indexed_triangle_set its = (i % 4 == 3) ? sphere(4 + (int) below(3), 6 + (int) below(4), range(2, 4)) : grid(2 + (int) below(4), 2 + (int) below(3), i % 2 ? 0.0 : 0.3);
        ModelVolume* vol = new_volume(its);
        paint(its, vol->mmu_segmentation_facets);
        if (below(2)) paint(its, vol->supported_facets);
        if (below(3) == 0) paint(its, vol->seam_facets);
        tag("mesh"); pmesh(its); nl();
        tag("annotation"); pdata(vol->mmu_segmentation_facets.get_data()); nl();
        tag("annotation2"); pdata(vol->supported_facets.get_data()); pdata(vol->seam_facets.get_data()); nl();
        const TriangleMesh src_mesh(its);
        const double maxd = i % 5 == 1 ? -1.0 : src_mesh.bounding_box().size().norm() / 100.0;

        {
            // the samplers on their own
            PaintReprojector pt(src_mesh, vol->mmu_segmentation_facets, true, maxd);
            for (int k = 0; k < 12; ++k) {
                const int f = (int) below(its.indices.size());
                std::array<Vec3f, 3> t = {its.vertices[its.indices[f][0]], its.vertices[its.indices[f][1]], its.vertices[its.indices[f][2]]};
                if (k % 3 == 1) { t[1] = (t[0] + t[1]) * 0.5f; }
                if (k % 3 == 2) { t[0] = (t[0] + t[1] + t[2]) / 3.f; }
                auto m = pt.measure_point_sample(t, t);
                tag("measure_point_sample"); pd(maxd); ptri(t); arrow(); pi((int) m.dominant_state); pd(m.surface_area); pd(m.error_area); pu(pt.fallback_count()); nl();
                auto roots = pt.resolve_source_roots(f, t);
                auto mo = pt.measure_overlap(roots, t, t);
                tag("measure_overlap"); pi(f); ptri(t); arrow(); pu(roots.size()); for (int r : roots) pi(r); pi((int) mo.dominant_state); pd(mo.surface_area); pd(mo.error_area); pu(pt.rebound_count()); pu(pt.fallback_count()); nl();
            }
            const double band = i % 4 == 2 ? -1.0 : maxd;
            PaintRegionSource rs(src_mesh, vol->mmu_segmentation_facets.get_data(), band);
            tag("region_source"); pd(band); arrow(); pi(rs.empty()); pu(rs.leaf_count()); pf(rs.finest_leaf_edge()); pu(rs.states().size());
            for (auto& st : rs.states()) { pi((int) st.state); pd(st.area); pbox(st.bbox); }
            nl();
            for (int k = 0; k < 12; ++k) {
                const int f = (int) below(its.indices.size());
                std::array<Vec3f, 3> t = {its.vertices[its.indices[f][0]], its.vertices[its.indices[f][1]], its.vertices[its.indices[f][2]]};
                if (k % 2) for (auto& v : t) v = (v + (t[0] + t[1] + t[2]) / 3.f) * 0.5f;
                const int state = 1 + (int) below(5);
                tag("coverage"); pd(band); pi(state); ptri(t); arrow(); pi((int) rs.coverage(EnforcerBlockerType(state), t)); nl();
            }
        }
        if (i % 2 == 0) {
            // reproject_one_annotation, point sampling, onto a re-triangulated copy
            const Transform3d dst_to_src = rnd_pose(i);
            const indexed_triangle_set dst = retriangulate(its, dst_to_src.inverse());
            const TriangleMesh dst_mesh(dst);
            PaintKeepPrepared out;
            Calls calls; calls.cancel_at = i % 7 == 6 ? 1 + (int) below(3) : 0;
            PaintReprojectProgressCallback progress = [&calls](int p, const char*) { ++calls.progress; calls.percent_sum += p; calls.last = p; };
            PaintReprojectCancelCallback cancel = nullptr;
            if (calls.cancel_at > 0) cancel = [&calls]() { return ++calls.cancel >= calls.cancel_at; };
            tag("mesh"); pmesh(dst); nl();
            tag("reproject_point"); ptrafo(dst_to_src); pd(maxd); pi(calls.cancel_at); arrow();
            bool ok = reproject_one_annotation(vol->mmu_segmentation_facets, out.mmu, src_mesh, dst_mesh, dst_to_src, nullptr, ReprojectMode::PointNearest, "mmu", progress, cancel, maxd, nullptr, 1e-6);
            pi(ok); pdata(out.mmu.get_data()); pcalls(calls); nl();
        }
        {
            // reproject_one_annotation, coplanar overlap, onto a cut copy
            const Transform3d dst_to_src = rnd_pose(i + 1);
            std::vector<int> map;
            const indexed_triangle_set dst = cut(its, dst_to_src.inverse(), map);
            const TriangleMesh dst_mesh(dst);
            PaintKeepPrepared out;
            tag("mesh"); pmesh(dst); nl();
            tag("reproject_overlap"); ptrafo(dst_to_src); pu(map.size()); for (int m : map) pi(m); arrow();
            bool ok = reproject_one_annotation(vol->mmu_segmentation_facets, out.mmu, src_mesh, dst_mesh, dst_to_src, &map, ReprojectMode::OverlapCoplanar, "mmu");
            pi(ok); pdata(out.mmu.get_data()); nl();

            // reproject_paint (the cut path) onto the same copy
            ModelVolume* dvol = new_volume(dst);
            Calls calls; calls.cancel_at = i % 6 == 5 ? 1 + (int) below(3) : 0;
            PaintReprojectProgressCallback progress = [&calls](int p, const char*) { ++calls.progress; calls.percent_sum += p; calls.last = p; };
            PaintReprojectCancelCallback cancel = nullptr;
            if (calls.cancel_at > 0) cancel = [&calls]() { return ++calls.cancel >= calls.cancel_at; };
            tag("reproject_paint"); pi(calls.cancel_at); arrow();
            ok = reproject_paint(*vol, *dvol, map, dst_to_src, progress, cancel);
            pi(ok); pdata(dvol->supported_facets.get_data()); pdata(dvol->seam_facets.get_data()); pdata(dvol->mmu_segmentation_facets.get_data()); pdata(dvol->fuzzy_skin_facets.get_data()); pcalls(calls); nl();
        }
        {
            // reproject_paint_geometric onto a re-triangulated copy, with and without a world matrix
            const Transform3d dst_to_src = rnd_pose(i + 2);
            const indexed_triangle_set dst = retriangulate(its, dst_to_src.inverse());
            const TriangleMesh dst_mesh(dst);
            const int wkind = (int) below(3);
            Transform3d world = Transform3d::Identity();
            if (wkind == 1) { const double s = range(0.5, 3); world = Geometry::assemble_transform(Vec3d(range(-5, 5), range(-5, 5), 0), Vec3d::Zero(), Vec3d(s, s, s)); }
            if (wkind == 2) world = Geometry::assemble_transform(Vec3d(range(-5, 5), range(-5, 5), 0), Vec3d(range(-3, 3), range(-3, 3), range(-3, 3)), Vec3d(range(0.5, 3), range(0.5, 3), range(0.5, 3)));
            PaintKeepPrepared out;
            Calls calls; calls.cancel_at = i % 5 == 4 ? 1 + (int) below(3) : 0;
            PaintReprojectProgressCallback progress = [&calls](int p, const char*) { ++calls.progress; calls.percent_sum += p; calls.last = p; };
            PaintReprojectCancelCallback cancel = nullptr;
            if (calls.cancel_at > 0) cancel = [&calls]() { return ++calls.cancel >= calls.cancel_at; };
            tag("mesh"); pmesh(dst); nl();
            tag("reproject_paint_geometric"); ptrafo(dst_to_src); pi(wkind); if (wkind) ptrafo(world); pi(calls.cancel_at); arrow();
            bool ok = reproject_paint_geometric(src_mesh, vol->supported_facets, vol->seam_facets, vol->mmu_segmentation_facets, vol->fuzzy_skin_facets,
                                                dst_mesh, dst_to_src, out.supported, out.seam, out.mmu, out.fuzzy, progress, cancel, wkind ? &world : nullptr);
            pi(ok); pdata(out.supported.get_data()); pdata(out.seam.get_data()); pdata(out.mmu.get_data()); pdata(out.fuzzy.get_data()); pcalls(calls); nl();
            RegionEdgeLimit el(dst_mesh, wkind ? &world : nullptr);
            tag("region_edge_limit"); pi(wkind); if (wkind) ptrafo(world); arrow(); pf(el.value()); pi(el.trafo() != nullptr); if (el.trafo()) ptrafof(*el.trafo()); nl();
        }
    }
}


// Boolean / merge: several operands, each with its paint, its filament and its pose, onto a
// destination that is their concatenation (a merge) or a re-triangulation of it (a boolean).
static void gen_volumes() {
    for (int i = 0; i < 30; ++i) {
        const int nops = 1 + (int) below(3);
        std::vector<indexed_triangle_set> meshes(nops);
        std::vector<ModelVolume*> vols(nops);
        std::vector<PaintSourceVolume> sources(nops);
        const bool unpainted = i % 5 == 3; // the filament-only fast path
        indexed_triangle_set combined;
        for (int k = 0; k < nops; ++k) {
            meshes[k] = below(3) == 0 ? sphere(3 + (int) below(3), 5 + (int) below(4), range(1, 3)) : grid(2 + (int) below(3), 2 + (int) below(3), range(0, 0.3));
            vols[k] = new_volume(meshes[k]);
            if (!unpainted) {
                if (below(3) != 0) paint(meshes[k], vols[k]->mmu_segmentation_facets);
                if (below(3) == 0) paint(meshes[k], vols[k]->supported_facets);
                if (below(4) == 0) paint(meshes[k], vols[k]->fuzzy_skin_facets);
            }
            PaintSourceVolume& src = sources[k];
            src.mesh = &vols[k]->mesh();
            src.supported = &vols[k]->supported_facets;
            src.seam = below(4) == 0 ? nullptr : &vols[k]->seam_facets;
            src.mmu = &vols[k]->mmu_segmentation_facets;
            src.fuzzy = &vols[k]->fuzzy_skin_facets;
            src.base_filament = below(3) == 0 ? 0 : 1 + (int) below(4);
            Vec3d off(range(-1.5, 1.5), range(-1.5, 1.5), range(-0.5, 0.5));
            src.source_to_destination = k == 0 && i % 2 ? Transform3d::Identity()
                : Geometry::assemble_transform(off, Vec3d(0, 0, range(-0.5, 0.5)), Vec3d(1, 1, 1), Vec3d(i % 7 == 2 && k == 1 ? -1 : 1, 1, 1));
            TriangleMesh m(meshes[k]);
            m.transform(src.source_to_destination, true);
            its_merge(combined, m.its);
        }
        const int kind = (int) below(3);
        const indexed_triangle_set dst = kind == 0 ? combined : retriangulate(combined, Transform3d::Identity());
        for (int k = 0; k < nops; ++k) {
            tag("mesh"); pmesh(meshes[k]); nl();
            tag("operand"); pdata(vols[k]->supported_facets.get_data()); pdata(vols[k]->seam_facets.get_data()); pdata(vols[k]->mmu_segmentation_facets.get_data()); pdata(vols[k]->fuzzy_skin_facets.get_data());
            pi(sources[k].seam != nullptr); pi(sources[k].base_filament); ptrafo(sources[k].source_to_destination); nl();
        }
        tag("mesh"); pmesh(dst); nl();
        const bool concatenated = kind == 0 && below(4) != 0;
        PaintKeepPrepared out;
        tag("reproject_paint_from_volumes"); pi(concatenated); arrow();
        bool ok = reproject_paint_from_volumes(TriangleMesh(dst), sources, out.supported, out.seam, out.mmu, out.fuzzy, nullptr, nullptr, nullptr, concatenated);
        pi(ok); pdata(out.supported.get_data()); pdata(out.seam.get_data()); pdata(out.mmu.get_data()); pdata(out.fuzzy.get_data()); nl();

        // measure_operand on the first operand, as the ranking builds it
        BooleanOperand op;
        op.mesh = TriangleMesh(meshes[0]);
        op.mesh.transform(sources[0].source_to_destination, true);
        op.tree = AABBTreeIndirect::build_aabb_tree_over_indexed_triangle_set(op.mesh.its.vertices, op.mesh.its.indices);
        op.bbox = op.mesh.bounding_box();
        op.extent = op.bbox.size().norm();
        for (int k = 0; k < 8; ++k) {
            const int f = (int) below(dst.indices.size());
            const std::array<Vec3f, 4> samples = face_samples(dst, f);
            const OperandHit hit = measure_operand(op, 0, samples);
            tag("measure_operand"); pi(f); arrow(); pd(operand_band(op)); pd(operand_reach(op)); pi(hit.op); pf(hit.distance); pi(hit.covers); nl();
        }
    }
}

int main(int argc, char** argv) {
    if (argc != 2) return 1;
    set_logging_level(0); // the reprojection logs its statistics; keep them off the vectors
    std::string what = argv[1];
    if (what == "planar") gen_planar();
    else if (what == "annotation") { g_state ^= 0x1111; gen_annotation(); }
    else if (what == "aabb") { g_state ^= 0x5555; gen_aabb(); }
    else if (what == "reproject") { g_state ^= 0x6666; gen_reproject(); }
    else if (what == "volumes") { g_state ^= 0x7777; gen_volumes(); }
    else return 1;
    return 0;
}
