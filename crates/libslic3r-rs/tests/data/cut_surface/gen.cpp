// Reference vectors for crates/libslic3r-rs/src/cut_surface.rs, taken from CutSurface.cpp at the pin.
//
// The program compiles CutSurface.cpp into itself (so that the functions of its namespace `priv`
// can be called) and links the engine's libslic3r for the rest. Build it as a libslic3r test
// program of a configured Bambu Studio engine build tree, then:
//   ./cut_surface_gen | gzip -9n > crates/libslic3r-rs/tests/data/cut_surface/vectors.txt.gz
//
// For every case it runs `Slic3r::cut_surface` stage by stage (`staged_cut_surface`,
// `staged_cut_from_model`, `staged_diff_models`, `staged_clip_cut` below repeat the bodies of
// `cut_surface`, `priv::cut_from_model`, `priv::diff_models` and `priv::clip_cut` line by line and
// call the pin's functions for everything else), printing what every stage produced, and then as
// the pin's function. The two results have to be equal, or the program stops. Around the two
// algorithms of CGAL the port takes through a trait (`CGAL::Polygon_mesh_processing::corefine`
// and `clip`) it prints what the port needs to replay them: the calls CGAL made to the visitor,
// in order, and the mesh, the constrained edges and the properties as CGAL left them.
//
// Two things in the pin change from run to run:
// - `priv::set_skip_for_out_of_aoi` sets the bits of a `std::vector<bool>` from a
//   `tbb::parallel_for` (CutSurface.cpp:812-818), where threads lose each other's writes. The
//   program runs TBB on one thread, where the loop does what it says.
// - CGAL's corefinement numbers the vertices and the faces it adds, and triangulates the faces it
//   splits, in an order that is not the same for every run of the same input (records POST, ECM
//   and CLIP, and what follows from them). So the file this program writes differs from run to
//   run, and in about every second run the pin's `cut_surface` does not take the way the staged
//   run took for one of the cases: the program then stops with a message and writes no complete
//   file; run it again. The test replays the run it is given; it passes with every complete file
//   this program writes.
//
// After the cases come the functions of the file that `cut_surface` never calls (`D_` records).
//
// One record per line, `TAG values`. Integers are decimal, doubles and floats the hex of their
// bits, a null index is 4294967295. A mesh is `nv ne nf`, per vertex `x y z halfedge removed`, per
// halfedge `face target next prev`, per edge `removed`, per face `halfedge removed`, then the heads
// of the three free lists and the garbage flag: every index, removed ones included.

#include "libslic3r/CutSurface.cpp"

#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <tbb/global_control.h>

using namespace Slic3r;
using priv::CutMesh;
using priv::EI;
using priv::FI;
using priv::HI;
using priv::VI;

static unsigned long long db(double d) { unsigned long long u; memcpy(&u, &d, 8); return u; }
static unsigned fb(float f) { unsigned u; memcpy(&u, &f, 4); return u; }

static void out(double d) { printf(" %016llx", db(d)); }
static void outf(float f) { printf(" %08x", fb(f)); }
static void outu(size_t u) { printf(" %zu", u); }
static void out(const Polygon &p)
{
    printf(" %zu", p.points.size());
    for (const Point &pt : p.points) printf(" %lld %lld", (long long) pt.x(), (long long) pt.y());
}
static void out(const Polygons &ps)
{
    printf(" %zu", ps.size());
    for (const Polygon &p : ps) out(p);
}
static void out(const ExPolygons &es)
{
    printf(" %zu", es.size());
    for (const ExPolygon &e : es) { out(e.contour); out(e.holes); }
}
static void out(const indexed_triangle_set &its)
{
    printf(" %zu", its.vertices.size());
    for (const Vec3f &v : its.vertices) { outf(v.x()); outf(v.y()); outf(v.z()); }
    printf(" %zu", its.indices.size());
    for (const auto &t : its.indices) printf(" %d %d %d", t[0], t[1], t[2]);
}
static void out_bits(const std::vector<bool> &b)
{
    printf(" %zu ", b.size());
    if (b.empty()) printf("-");
    for (bool x : b) putchar(x ? '1' : '0');
}

// The head of a free list of CGAL::Surface_mesh is private; it is the removed element no other
// removed element links to (Surface_mesh.h: remove_vertex, remove_edge, remove_face).
template<class Link, class Removed> static uint32_t freelist_head(uint32_t n, Link link, Removed removed)
{
    std::vector<bool> linked(n, false);
    size_t count = 0;
    for (uint32_t i = 0; i < n; ++i)
        if (removed(i)) {
            ++count;
            uint32_t l = link(i);
            if (l != std::numeric_limits<uint32_t>::max()) {
                if (l >= n || linked[l]) { fprintf(stderr, "free list is not a chain\n"); exit(1); }
                linked[l] = true;
            }
        }
    uint32_t head = std::numeric_limits<uint32_t>::max();
    size_t   heads = 0;
    for (uint32_t i = 0; i < n; ++i)
        if (removed(i) && !linked[i]) { head = i; ++heads; }
    if ((count == 0) != (heads == 0) || heads > 1) { fprintf(stderr, "free list: %zu removed, %zu heads\n", count, heads); exit(1); }
    return head;
}

static void out(const CutMesh &m)
{
    uint32_t nv = m.num_vertices(), ne = m.num_edges(), nf = m.num_faces();
    printf(" %u %u %u", nv, ne, nf);
    for (uint32_t i = 0; i < nv; ++i) {
        VI v(i);
        const priv::P3 &p = m.point(v);
        out(p.x()); out(p.y()); out(p.z());
        printf(" %u %d", (uint32_t) m.halfedge(v), (int) m.is_removed(v));
    }
    for (uint32_t i = 0; i < 2 * ne; ++i) {
        HI h(i);
        printf(" %u %u %u %u", (uint32_t) m.face(h), (uint32_t) m.target(h), (uint32_t) m.next(h), (uint32_t) m.prev(h));
    }
    for (uint32_t i = 0; i < ne; ++i) printf(" %d", (int) m.is_removed(EI(i)));
    for (uint32_t i = 0; i < nf; ++i) printf(" %u %d", (uint32_t) m.halfedge(FI(i)), (int) m.is_removed(FI(i)));
    uint32_t vfl = freelist_head(nv, [&](uint32_t i) { return (uint32_t) m.halfedge(VI(i)); }, [&](uint32_t i) { return m.is_removed(VI(i)); });
    // the edges are linked through the `next` of their first halfedge, by halfedge index
    uint32_t efl = freelist_head(ne, [&](uint32_t i) { uint32_t l = (uint32_t) m.next(HI(2 * i)); return l == std::numeric_limits<uint32_t>::max() ? l : l / 2; },
                                 [&](uint32_t i) { return m.is_removed(EI(i)); });
    if (efl != std::numeric_limits<uint32_t>::max()) efl *= 2;
    uint32_t ffl = freelist_head(nf, [&](uint32_t i) { return (uint32_t) m.halfedge(FI(i)); }, [&](uint32_t i) { return m.is_removed(FI(i)); });
    printf(" %u %u %u %d", vfl, efl, ffl, (int) m.has_garbage());
}

static void out_ie(const priv::IntersectingElement &ie) { printf(" %u %u", ie.shape_point_index, (unsigned) ie.attr); }

static void out_vsm(const CutMesh &m)
{
    const priv::VertexShapeMap &vsm = m.property_map<VI, const priv::IntersectingElement *>(priv::vert_shape_map_name).first;
    printf(" %u", m.num_vertices());
    for (uint32_t i = 0; i < m.num_vertices(); ++i) {
        const priv::IntersectingElement *ie = vsm[VI(i)];
        if (ie == nullptr) printf(" -");
        else printf(" %u:%u", ie->shape_point_index, (unsigned) ie->attr);
    }
}

static void out_face_types(const CutMesh &m, const priv::FaceTypeMap &map)
{
    printf(" %u ", m.num_faces());
    if (m.num_faces() == 0) printf("-");
    for (uint32_t i = 0; i < m.num_faces(); ++i) putchar('0' + (int) map[FI(i)]);
}

static void out_vi_map(const CutMesh &m, const priv::CvtVI2VI &map)
{
    printf(" %u", m.num_vertices());
    for (uint32_t i = 0; i < m.num_vertices(); ++i) printf(" %u", (uint32_t) map[VI(i)]);
}

static void out(const priv::CutAOIs &aois)
{
    printf(" %zu", aois.size());
    for (const priv::CutAOI &aoi : aois) {
        printf(" %zu", aoi.first.size());
        for (FI f : aoi.first) printf(" %u", (uint32_t) f);
        printf(" %zu", aoi.second.size());
        for (HI h : aoi.second) printf(" %u", (uint32_t) h);
    }
}

static void out(const BoundingBoxf3 &bb)
{
    out(bb.min.x()); out(bb.min.y()); out(bb.min.z()); out(bb.max.x()); out(bb.max.y()); out(bb.max.z());
    printf(" %d", (int) bb.defined);
}

static void out(const priv::SurfacePatch &patch)
{
    printf(" %zu %zu %zu %d", patch.model_id, patch.aoi_id, patch.shape_id, (int) patch.is_whole_aoi);
    out(patch.bb);
    printf(" %zu", patch.loops.size());
    for (const priv::Loop &l : patch.loops) {
        printf(" %zu", l.size());
        for (VI v : l) printf(" %u", (uint32_t) v);
    }
    out(patch.mesh);
    out_vi_map(patch.mesh, patch.mesh.property_map<VI, VI>(priv::patch_source_name).first);
}

static void out(const priv::ProjectionDistance &d)
{
    printf(" %u %u %u", d.model_index, d.aoi_index, d.patch_index);
    outf(d.distance);
}

static void out(const SurfaceCut &cut)
{
    out(static_cast<const indexed_triangle_set &>(cut));
    printf(" %zu", cut.contours.size());
    for (const auto &c : cut.contours) {
        printf(" %zu", c.size());
        for (unsigned i : c) printf(" %u", i);
    }
}

static bool same(const SurfaceCut &a, const SurfaceCut &b)
{
    if (a.vertices.size() != b.vertices.size() || a.indices.size() != b.indices.size() || a.contours != b.contours) return false;
    for (size_t i = 0; i < a.vertices.size(); ++i)
        if (memcmp(a.vertices[i].data(), b.vertices[i].data(), 12) != 0) return false;
    for (size_t i = 0; i < a.indices.size(); ++i)
        if (a.indices[i] != b.indices[i]) return false;
    return true;
}

// ---------------------------------------------------------------------------------------------
// The calls of CGAL to the visitor of priv::cut_from_model, recorded
// ---------------------------------------------------------------------------------------------

struct Event
{
    char     kind; // 'I' intersection_point_detected, 'N' new_vertex_added
    size_t   i_id;
    int      sdim;
    uint32_t h_f, h_e;
    int      e_is_shape, is_target_coplanar, is_source_coplanar;
    uint32_t v;
};

struct RecVisitor : public priv::Visitor
{
    std::vector<Event> *log;
    uint32_t            nv, nh, nf; // the object before the corefinement
    RecVisitor(const CutMesh &object, const CutMesh &shape, priv::EdgeShapeMap edge_shape_map, priv::FaceShapeMap face_shape_map,
               priv::VertexShapeMap vert_shape_map, bool *is_valid, std::vector<Event> *log)
        : priv::Visitor(object, shape, edge_shape_map, face_shape_map, vert_shape_map, is_valid), log(log),
          nv(object.num_vertices()), nh(object.num_halfedges()), nf(object.num_faces())
    {}

    void intersection_point_detected(std::size_t i_id, int sdim, HI h_f, HI h_e, const CutMesh &tm_f, const CutMesh &tm_e,
                                     bool is_target_coplanar, bool is_source_coplanar)
    {
        // The port replays these calls on the object as it was before the corefinement.
        if (object.num_vertices() != nv || object.num_halfedges() != nh || object.num_faces() != nf) {
            fprintf(stderr, "intersection_point_detected after the object was modified\n");
            exit(1);
        }
        if (!log->empty() && log->back().kind == 'N') {
            fprintf(stderr, "intersection_point_detected after new_vertex_added\n");
            exit(1);
        }
        log->push_back({'I', i_id, sdim, (uint32_t) h_f, (uint32_t) h_e, (&tm_e == &shape) ? 1 : 0, (int) is_target_coplanar, (int) is_source_coplanar, 0});
        priv::Visitor::intersection_point_detected(i_id, sdim, h_f, h_e, tm_f, tm_e, is_target_coplanar, is_source_coplanar);
    }

    void new_vertex_added(std::size_t i_id, VI v, const CutMesh &tm)
    {
        if (&tm != &object) {
            fprintf(stderr, "new_vertex_added for the shape\n");
            exit(1);
        }
        log->push_back({'N', i_id, 0, 0, 0, 0, 0, 0, (uint32_t) v});
        priv::Visitor::new_vertex_added(i_id, v, tm);
    }
};

// priv::cut_from_model (CutSurface.cpp:1400-1443), stage by stage
static priv::CutAOIs staged_cut_from_model(size_t m, CutMesh &cgal_model, const ExPolygons &shapes, CutMesh &cgal_shape, float projection_ratio,
                                           const ExPolygonsIndices &s2i)
{
    using namespace priv;
    VertexShapeMap vert_shape_map = cgal_model.add_property_map<VI, const IntersectingElement *>(vert_shape_map_name, nullptr).first;

    bool                is_valid       = true;
    const EdgeShapeMap &edge_shape_map = cgal_shape.property_map<EI, IntersectingElement>(edge_shape_map_name).first;
    const FaceShapeMap &face_shape_map = cgal_shape.property_map<FI, IntersectingElement>(face_shape_map_name).first;
    std::vector<Event>  log;
    RecVisitor          visitor{cgal_model, cgal_shape, edge_shape_map, face_shape_map, vert_shape_map, &is_valid, &log};

    EdgeBoolMap ecm = cgal_model.add_property_map<EI, bool>(is_constrained_edge_name, false).first;
    const auto &p   = CGAL::parameters::visitor(visitor).edge_is_constrained_map(ecm).throw_on_self_intersection(false);
    const auto &q   = CGAL::parameters::do_not_modify(true);
    CGAL::Polygon_mesh_processing::corefine(cgal_model, cgal_shape, p, q);

    printf("COREFINE %zu %zu", m, log.size());
    for (const Event &e : log) {
        if (e.kind == 'I') printf(" I %zu %d %u %u %d %d %d", e.i_id, e.sdim, e.h_f, e.h_e, e.e_is_shape, e.is_target_coplanar, e.is_source_coplanar);
        else printf(" N %zu %u", e.i_id, e.v);
    }
    printf("\nPOST %zu", m); out(cgal_model);
    {
        std::vector<bool> b;
        for (uint32_t i = 0; i < cgal_model.num_edges(); ++i) b.push_back(ecm[EI(i)]);
        printf("\nECM %zu", m); out_bits(b);
    }
    printf("\nVSM %zu", m); out_vsm(cgal_model);
    printf("\nVALID %zu %d\n", m, (int) is_valid);

    if (!is_valid) return {};

    FaceTypeMap face_type_map = cgal_model.add_property_map<FI, FaceType>(face_type_map_name, FaceType::not_constrained).first;

    set_face_type(face_type_map, cgal_model, vert_shape_map, ecm, cgal_shape, s2i);
    printf("FT1 %zu", m); out_face_types(cgal_model, face_type_map); printf("\n");

    flood_fill_inner(cgal_model, face_type_map);
    printf("FT2 %zu", m); out_face_types(cgal_model, face_type_map); printf("\n");

    CutAOIs aois = create_cut_area_of_interests(cgal_model, shapes, face_type_map);
    printf("FT3 %zu", m); out_face_types(cgal_model, face_type_map); printf("\n");
    return aois;
}

// priv::clip_cut (CutSurface.cpp:2559-2608), with what CGAL did printed
static bool staged_clip_cut(priv::SurfacePatch &cut, CutMesh clipper)
{
    using namespace priv;
    CutMesh &tm = cut.mesh;
    CutMesh  backup_copy = tm;

    class ExistIntersectionClipVisitor : public CGAL::Polygon_mesh_processing::Corefinement::Default_visitor<CutMesh>
    {
        bool *exist_intersection;

    public:
        ExistIntersectionClipVisitor(bool *exist_intersection) : exist_intersection(exist_intersection) {}
        void intersection_point_detected(std::size_t, int, HI, HI, const CutMesh &, const CutMesh &, bool, bool) { *exist_intersection = true; }
    };
    bool                         exist_intersection = false;
    ExistIntersectionClipVisitor visitor{&exist_intersection};

    const auto &np_tm = CGAL::parameters::visitor(visitor).throw_on_self_intersection(false);
    const auto &np_c  = CGAL::parameters::throw_on_self_intersection(false);

    bool suc = false;
    try {
        suc = CGAL::Polygon_mesh_processing::clip(tm, clipper, np_tm, np_c);
    } catch (const std::exception &e) {
        printf("CLIP 1 0 0\n");
        cut.mesh = backup_copy;
        return false;
    }
    printf("CLIP 0 %d %d", (int) suc, (int) exist_intersection);
    out(tm);
    out_vi_map(tm, tm.property_map<VI, VI>(patch_source_name).first);
    printf("\n");

    if (!exist_intersection || !suc) {
        cut.mesh = backup_copy;
        return false;
    }
    return true;
}

// priv::diff_models (CutSurface.cpp:3040-3135), with staged_clip_cut for priv::clip_cut
static priv::SurfacePatches staged_diff_models(priv::VCutAOIs &cuts, priv::CutMeshes &cut_models, priv::CutMeshes &models, const priv::Project3d &projection)
{
    using namespace priv;
    priv::ModelCut2index m2i(cuts);

    std::vector<BoundingBoxf3> bbs = create_bbs(cuts, cut_models);
    Trees                      trees(models.size());

    SurfacePatches   patches;
    SurfacePatchesEx aoi_patches;

    patches.reserve(m2i.get_count());
    size_t index = 0;
    for (size_t model_index = 0; model_index < models.size(); ++model_index) {
        CutAOIs       &model_cuts = cuts[model_index];
        CutMesh       &cut_model_ = cut_models[model_index];
        const CutMesh &cut_model  = cut_model_;
        ReductionMap   vertex_reduction_map = cut_model_.add_property_map<VI, VI>(vertex_reduction_map_name).first;
        create_reduce_map(vertex_reduction_map, cut_model);
        printf("REDUCE %zu", model_index); out_vi_map(cut_model, vertex_reduction_map); printf("\n");

        for (size_t cut_index = 0; cut_index < model_cuts.size(); ++cut_index, ++index) {
            const CutAOI  &cut = model_cuts[cut_index];
            SurfacePatchEx patch_ex;
            SurfacePatch  &patch = patch_ex.patch;
            patch              = create_surface_patch(cut.first, cut_model_, &vertex_reduction_map);
            patch.bb           = bbs[index];
            patch.aoi_id       = cut_index;
            patch.model_id     = model_index;
            patch.shape_id     = get_shape_point_index(cut, cut_model);
            patch.is_whole_aoi = true;

            aoi_patches.clear();
            aoi_patches.push_back(patch_ex);
            for (size_t model_index2 = 0; model_index2 < models.size(); ++model_index2) {
                if (model_index == model_index2) continue;
                for (SurfacePatchEx &patch_ex : aoi_patches) {
                    SurfacePatch &patch = patch_ex.patch;
                    if (has_bb_intersection(patch.bb, model_index2, bbs, m2i) && staged_clip_cut(patch, models[model_index2])) {
                        patch_ex.just_cliped = true;
                    } else {
                        Tree &tree = trees[model_index2];
                        if (tree.empty()) {
                            const CutMesh &model   = models[model_index2];
                            auto           f_range = faces(model);
                            tree.insert(f_range.first, f_range.second, model);
                            tree.build();
                        }
                        if (is_patch_inside_of_model(patch, tree, projection)) patch_ex.full_inside = true;
                    }
                }
                for (size_t i = aoi_patches.size(); i != 0; --i) {
                    auto it = aoi_patches.begin() + (i - 1);
                    if (it->full_inside) aoi_patches.erase(it);
                }

                if (aoi_patches.empty()) break;

                size_t end = aoi_patches.size();
                for (size_t i = 0; i < end; ++i)
                    if (aoi_patches[i].just_cliped) divide_patch(i, aoi_patches);
            }

            if (!aoi_patches.empty()) {
                patches.reserve(patches.size() + aoi_patches.size());
                for (SurfacePatchEx &patch : aoi_patches) patches.push_back(std::move(patch.patch));
            }
        }
        cut_model_.remove_property_map(vertex_reduction_map);
    }

    collect_open_edges(patches);
    return patches;
}

// Slic3r::cut_surface (CutSurface.cpp:525-617), stage by stage
static SurfaceCut staged_cut_surface(const ExPolygons &shapes, const std::vector<indexed_triangle_set> &models, const Emboss::IProjection &projection,
                                     float projection_ratio)
{
    if (models.empty() || shapes.empty()) return {};

    BoundingBox shapes_bb = get_extents(shapes);
    printf("BB %lld %lld %lld %lld\n", (long long) shapes_bb.min.x(), (long long) shapes_bb.min.y(), (long long) shapes_bb.max.x(), (long long) shapes_bb.max.y());

    const float     max_angle = 89.9f;
    priv::CutMeshes cgal_models;
    priv::CutMeshes cgal_neg_models;
    cgal_models.reserve(models.size());
    for (const indexed_triangle_set &its : models) {
        size_t            m = &its - &models.front();
        std::vector<bool> skip_indicies(its.indices.size(), {false});
        priv::set_skip_for_out_of_aoi(skip_indicies, its, projection, shapes_bb);
        printf("SKIP1 %zu", m); out_bits(skip_indicies); printf("\n");

        bool flip = true;
        cgal_neg_models.push_back(priv::to_cgal(its, skip_indicies, flip));
        printf("NEG %zu", m); out(cgal_neg_models.back()); printf("\n");

        priv::set_skip_by_angle(skip_indicies, its, projection, max_angle);
        printf("SKIP2 %zu", m); out_bits(skip_indicies); printf("\n");
        cgal_models.push_back(priv::to_cgal(its, skip_indicies));
        printf("MODEL %zu", m); out(cgal_models.back()); printf("\n");

        // Slic3r::its_cut_AoI (and its_mask) on the same input
        printf("ITS_CUT_AOI %zu", m); out(its_cut_AoI(its, shapes_bb, projection)); printf("\n");
    }

    priv::CutMesh cgal_shape = priv::to_cgal(shapes, projection);
    printf("SHAPE"); out(cgal_shape); printf("\n");
    {
        const priv::EdgeShapeMap &esm = cgal_shape.property_map<EI, priv::IntersectingElement>(priv::edge_shape_map_name).first;
        const priv::FaceShapeMap &fsm = cgal_shape.property_map<FI, priv::IntersectingElement>(priv::face_shape_map_name).first;
        printf("ESM %u", cgal_shape.num_edges());
        for (uint32_t i = 0; i < cgal_shape.num_edges(); ++i) out_ie(esm[EI(i)]);
        printf("\nFSM %u", cgal_shape.num_faces());
        for (uint32_t i = 0; i < cgal_shape.num_faces(); ++i) out_ie(fsm[FI(i)]);
        printf("\n");
    }

    ExPolygonsIndices s2i(shapes);
    priv::VCutAOIs    model_cuts;
    for (priv::CutMesh &cgal_model : cgal_models) {
        size_t        m       = &cgal_model - &cgal_models.front();
        priv::CutAOIs cutAOIs = staged_cut_from_model(m, cgal_model, shapes, cgal_shape, projection_ratio, s2i);
        printf("AOIS %zu", m); out(cutAOIs); printf("\n");
        model_cuts.push_back(std::move(cutAOIs));
    }

    priv::SurfacePatches patches = staged_diff_models(model_cuts, cgal_models, cgal_neg_models, projection);
    printf("PATCHES %zu\n", patches.size());
    for (const priv::SurfacePatch &patch : patches) {
        printf("PATCH %zu", size_t(&patch - &patches.front())); out(patch); printf("\n");
    }
    if (patches.empty()) return {};

    for (priv::SurfacePatch &patch : patches) patch.shape_id = s2i.cvt(patch.shape_id).expolygons_index;
    printf("SHAPE_IDS %zu", patches.size());
    for (const priv::SurfacePatch &patch : patches) printf(" %zu", patch.shape_id);
    printf("\n");

    uint32_t         shapes_points = s2i.get_count();
    priv::VDistances distances     = priv::calc_distances(patches, cgal_models, cgal_shape, shapes_points, projection_ratio);
    printf("DIST %zu", distances.size());
    for (const priv::ProjectionDistances &ds : distances) {
        printf(" %zu", ds.size());
        for (const priv::ProjectionDistance &d : ds) out(d);
    }
    printf("\n");

    Point start = shapes_bb.center();

    priv::ProjectionDistances best_projection = priv::choose_best_distance(distances, shapes, start, s2i, patches);
    printf("BEST %zu", best_projection.size());
    for (const priv::ProjectionDistance &d : best_projection) out(d);
    printf("\n");

    std::vector<bool> use_patch = priv::select_patches(best_projection, patches, shapes, shapes_bb, s2i, model_cuts, cgal_models, projection);
    printf("USE"); out_bits(use_patch); printf("\n");

    SurfaceCut result = merge_patches(patches, use_patch);
    return result;
}

// ---------------------------------------------------------------------------------------------
// Cases
// ---------------------------------------------------------------------------------------------

static void translate(indexed_triangle_set &its, float x, float y, float z)
{
    for (Vec3f &v : its.vertices) v += Vec3f(x, y, z);
}

// (x, y, z) -> (z, y, -x): a quarter turn about the y axis
static void turn_y(indexed_triangle_set &its)
{
    for (Vec3f &v : its.vertices) v = Vec3f(v.z(), v.y(), -v.x());
}

static Polygon poly(const Points &pts) { return Polygon(pts); }
// counter-clockwise, a contour
static Polygon rect(coord_t x0, coord_t y0, coord_t x1, coord_t y1) { return poly({Point(x0, y0), Point(x1, y0), Point(x1, y1), Point(x0, y1)}); }
// clockwise, a hole
static Polygon rect_hole(coord_t x0, coord_t y0, coord_t x1, coord_t y1) { return poly({Point(x0, y0), Point(x0, y1), Point(x1, y1), Point(x1, y0)}); }

// The shapes are in thousandths of a millimeter; the matrix scales them and puts them at `origin`.
static Transform3d shape_to_world(const Vec3d &origin, const Matrix3d &rot = Matrix3d::Identity())
{
    Transform3d tr = Transform3d::Identity();
    tr.translate(origin);
    tr.rotate(rot);
    tr.scale(0.001);
    return tr;
}

static void run(const char *name, const ExPolygons &shapes, const std::vector<indexed_triangle_set> &models, const Transform3d &tr, const Vec3d &dir, float ratio)
{
    printf("CASE %s\n", name);
    printf("IN_SHAPES"); out(shapes); printf("\n");
    printf("IN_MODELS %zu\n", models.size());
    for (const indexed_triangle_set &its : models) { printf("IN_ITS"); out(its); printf("\n"); }
    printf("IN_PROJ");
    for (int r = 0; r < 4; ++r) for (int c = 0; c < 4; ++c) out(tr.matrix()(r, c));
    out(dir.x()); out(dir.y()); out(dir.z());
    outf(ratio);
    printf("\n");

    Emboss::OrthoProject projection(tr, dir);
    SurfaceCut           staged = staged_cut_surface(shapes, models, projection, ratio);
    printf("CUT"); out(staged); printf("\n");

    // CGAL's corefinement does not give the same mesh for the same input every time (see the head
    // of the file): when the pin's function did not take the way the staged run took, the program
    // stops, and is run again.
    bool equal = false;
    for (int tries = 0; tries < 8 && !equal; ++tries) equal = same(staged, cut_surface(shapes, models, projection, ratio));
    if (!equal) {
        fprintf(stderr, "%s: Slic3r::cut_surface did not take the way of the staged run; run the program again\n", name);
        exit(1);
    }
    printf("REAL_EQUAL 1\n");

    if (!staged.empty()) {
        Emboss::OrthoProject3d back(dir);
        printf("CUT2MODEL"); out(cut2model(staged, back)); printf("\n");
    }
    fflush(stdout);
}

// ---------------------------------------------------------------------------------------------
// Functions of the file that Slic3r::cut_surface never calls, called here: one line per call,
// `TAG inputs -> outputs`
// ---------------------------------------------------------------------------------------------

static uint64_t lcg = 0x2545F4914F6CDD1Dull;
static uint64_t rnd() { lcg = lcg * 6364136223846793005ull + 1442695040888963407ull; return lcg >> 11; }
static double   rndd(double lo, double hi) { return lo + (hi - lo) * double(rnd() % 1000001) / 1000000.; }

// priv::create_trees and the count of the triangles a ray intersects (what
// priv::is_patch_inside_of_model asks the tree): random rays, rays through the vertices, rays
// along the edges, rays that start on a vertex
static void direct_tree(const indexed_triangle_set &its, bool flip, const Vec3d &lo, const Vec3d &hi)
{
    std::vector<bool> skip(its.indices.size(), false);
    priv::CutMeshes   models;
    models.push_back(priv::to_cgal(its, skip, flip));
    priv::Trees          trees = priv::create_trees(models);
    const priv::CutMesh &m     = models.front();

    std::vector<std::array<double, 6>> rays;
    for (int i = 0; i < 300; ++i) {
        std::array<double, 6> r;
        for (int k = 0; k < 6; ++k) r[k] = rndd(lo[k % 3], hi[k % 3]);
        rays.push_back(r);
    }
    Vec3d c = (lo + hi) / 2.;
    Vec3d inside(c.x() + 0.37, c.y() - 0.23, c.z() + 0.11);
    for (uint32_t i = 0; i < m.num_vertices() && i < 60; ++i) {
        const priv::P3 &p = m.point(VI(i));
        rays.push_back({inside.x(), inside.y(), inside.z(), p.x(), p.y(), p.z()});
        rays.push_back({p.x(), p.y(), p.z(), p.x(), p.y(), p.z() + 1.});
        rays.push_back({p.x(), p.y(), p.z(), inside.x(), inside.y(), inside.z()});
    }
    for (uint32_t i = 0; i < m.num_edges() && i < 60; ++i) {
        HI              h = m.halfedge(EI(i));
        const priv::P3 &a = m.point(m.source(h));
        const priv::P3 &b = m.point(m.target(h));
        rays.push_back({a.x(), a.y(), a.z(), b.x(), b.y(), b.z()});
        rays.push_back({inside.x(), inside.y(), inside.z(), (a.x() + b.x()) / 2., (a.y() + b.y()) / 2., (a.z() + b.z()) / 2.});
    }

    printf("D_TREE %d", (int) flip);
    out(its);
    printf(" %zu", rays.size());
    for (const auto &r : rays) for (double d : r) out(d);
    printf(" ->");
    for (const auto &r : rays)
        printf(" %zu", (size_t) trees[0].number_of_intersected_primitives(priv::Ray(priv::P3(r[0], r[1], r[2]), priv::P3(r[3], r[4], r[5]))));
    printf("\n");
}

// priv::ModelCut2index
static void direct_m2i(const std::vector<size_t> &sizes)
{
    priv::VCutAOIs cuts(sizes.size());
    for (size_t i = 0; i < sizes.size(); ++i) cuts[i].resize(sizes[i]);
    priv::ModelCut2index m2i(cuts);
    printf("D_M2I %zu", sizes.size());
    for (size_t n : sizes) printf(" %zu", n);
    printf(" -> %u", m2i.get_count());
    for (uint32_t o : m2i.get_offsets()) printf(" %u", o);
    for (uint32_t m = 0; m < sizes.size(); ++m)
        for (uint32_t c = 0; c < sizes[m]; ++c) printf(" %u", m2i.calc_index({m, c}));
    for (uint32_t i = 0; i < m2i.get_count(); ++i) {
        priv::ModelCutId id = m2i.calc_id(i);
        printf(" %u %u", id.model_index, id.cut_index);
    }
    printf("\n");
}

// priv::calc_size_sq and priv::find_close_point
static void direct_close()
{
    for (int i = 0; i < 200; ++i) {
        Point p(coord_t(rndd(-3e6, 3e6)), coord_t(rndd(-3e6, 3e6)));
        printf("D_SIZE %lld %lld ->", (long long) p.x(), (long long) p.y());
        outf(priv::calc_size_sq(p));
        printf("\n");
    }

    ExPolygon with_hole(rect(20000, 1000, 61000, 43000));
    with_hole.holes.push_back(rect_hole(30000, 11000, 50000, 33000));
    ExPolygons        shapes{ExPolygon(rect(1000, 2000, 11000, 13000)), with_hole, ExPolygon(poly({Point(70000, 5000), Point(93000, 9000), Point(81000, 37000)}))};
    ExPolygonsIndices s2i(shapes);
    for (int k = 0; k < 4; ++k) {
        priv::ProjectionDistances result(s2i.get_count());
        for (uint32_t i = 0; i < result.size(); ++i)
            if ((i + k) % 3 != 0) result[i].aoi_index = i;
        for (int flags = 0; flags < 8; ++flags) {
            std::vector<bool> finished{(flags & 1) != 0, (flags & 2) != 0, (flags & 4) != 0};
            for (int i = 0; i < 6; ++i) {
                Point p(coord_t(rndd(-10000, 100000)), coord_t(rndd(-10000, 50000)));
                printf("D_CLOSE");
                out(shapes);
                printf(" %zu", result.size());
                for (const priv::ProjectionDistance &d : result) printf(" %u", d.aoi_index);
                out_bits(finished);
                printf(" %lld %lld ->", (long long) p.x(), (long long) p.y());
                priv::ClosePoint cp = priv::find_close_point(p, result, finished, s2i, shapes);
                printf(" %u", cp.index);
                outf(cp.dist_sq);
                printf("\n");
            }
        }
    }
}

struct RecSources : public priv::IntersectionSources
{
    std::vector<Event> *log;
    uint32_t            nv, nh, nf; // the patch before the corefinement
    void intersection_point_detected(std::size_t i_id, int sdim, HI h_f, HI h_e, const CutMesh &tm_f, const CutMesh &tm_e, bool is_target_coplanar,
                                     bool is_source_coplanar)
    {
        if (patch->num_vertices() != nv || patch->num_halfedges() != nh || patch->num_faces() != nf || (!log->empty() && log->back().kind == 'N')) {
            fprintf(stderr, "intersection_point_detected after the patch was modified\n");
            exit(1);
        }
        log->push_back({'I', i_id, sdim, (uint32_t) h_f, (uint32_t) h_e, (&tm_e == model) ? 1 : 0, (int) is_target_coplanar, (int) is_source_coplanar, 0});
        priv::IntersectionSources::intersection_point_detected(i_id, sdim, h_f, h_e, tm_f, tm_e, is_target_coplanar, is_source_coplanar);
    }
    void new_vertex_added(std::size_t i_id, VI v, const CutMesh &tm)
    {
        if (&tm != patch) {
            fprintf(stderr, "new_vertex_added for the model\n");
            exit(1);
        }
        log->push_back({'N', i_id, 0, 0, 0, 0, 0, 0, (uint32_t) v});
        priv::IntersectionSources::new_vertex_added(i_id, v, tm);
    }
};

// priv::IntersectionSources as the visitor of a corefinement, priv::create_face_types over what
// it wrote, and priv::bounding_box of a patch
static void direct_face_types(const indexed_triangle_set &its1, const indexed_triangle_set &its2)
{
    using namespace priv;
    CutMesh tm1 = to_cgal(its1, std::vector<bool>(its1.indices.size(), false));
    CutMesh tm2 = to_cgal(its2, std::vector<bool>(its2.indices.size(), false));

    VertexSourceMap    vmap = tm1.add_property_map<VI, Source>(vertex_source_map_name).first;
    Sources            sources;
    std::vector<Event> log;
    RecSources         visitor{{&tm1, &tm2, vmap, &sources}, &log, tm1.num_vertices(), tm1.num_halfedges(), tm1.num_faces()};
    EdgeBoolMap        ecm = tm1.add_property_map<EI, bool>(is_constrained_edge_name, false).first;
    const auto        &p   = CGAL::parameters::visitor(visitor).edge_is_constrained_map(ecm).throw_on_self_intersection(false);
    const auto        &q   = CGAL::parameters::do_not_modify(true);
    CGAL::Polygon_mesh_processing::corefine(tm1, tm2, p, q);

    printf("D_FT"); out(its1); out(its2); printf("\n");
    printf("D_FT_COREFINE %zu", log.size());
    for (const Event &e : log) {
        if (e.kind == 'I') printf(" I %zu %d %u %u %d %d %d", e.i_id, e.sdim, e.h_f, e.h_e, e.e_is_shape, e.is_target_coplanar, e.is_source_coplanar);
        else printf(" N %zu %u", e.i_id, e.v);
    }
    printf("\nD_FT_POST"); out(tm1);
    {
        std::vector<bool> b;
        for (uint32_t i = 0; i < tm1.num_edges(); ++i) b.push_back(ecm[EI(i)]);
        printf("\nD_FT_ECM"); out_bits(b);
    }
    printf("\nD_FT_VMAP %u", tm1.num_vertices());
    for (uint32_t i = 0; i < tm1.num_vertices(); ++i) printf(" %u %d", (uint32_t) vmap[VI(i)].hi, vmap[VI(i)].sdim);
    printf("\nD_FT_SOURCES %zu", sources.size());
    for (const Source &s : sources) printf(" %u %d", (uint32_t) s.hi, s.sdim);

    FaceTypeMap map = tm1.add_property_map<FI, FaceType>(face_type_map_name, FaceType::not_constrained).first;
    create_face_types(map, tm1, tm2, ecm, vmap);
    printf("\nD_FT_TYPES"); out_face_types(tm1, map);

    SurfacePatch patch;
    patch.mesh = tm1;
    printf("\nD_FT_BB"); out(bounding_box(patch));
    printf("\n");
}

int main()
{
    // priv::set_skip_for_out_of_aoi (CutSurface.cpp:812-818) sets the bits of a std::vector<bool>
    // from a tbb::parallel_for: two threads writing bits of the same word lose one of the writes,
    // and the skip flags (and Slic3r::its_cut_AoI, and the meshes built from them) change from run
    // to run. On one thread every write lands, which is what the loop says and what the port does.
    tbb::global_control one_thread(tbb::global_control::max_allowed_parallelism, 1);

    const double fa = 2. * PI / 24.;

    // The projection goes from inside a model outwards: a face is kept when its normal looks
    // along the projection (set_skip_by_angle).
    Transform3d up     = shape_to_world(Vec3d(0., 0., 5.));
    Vec3d       up_dir = Vec3d(0., 0., 10.);

    indexed_triangle_set cube = its_make_cube(10., 10., 10.);

    // a square on the top of a cube
    run("cube_square", {ExPolygon(rect(2100, 3300, 6700, 7900))}, {cube}, up, up_dir, 0.5f);
    // a square with a hole
    {
        ExPolygon e(rect(1300, 1700, 8900, 8100));
        e.holes.push_back(rect_hole(3100, 3700, 6300, 5900));
        run("cube_square_hole", {e}, {cube}, up, up_dir, 0.5f);
    }
    // two squares
    run("cube_two_squares", {ExPolygon(rect(1100, 1300, 4300, 4700)), ExPolygon(rect(5300, 5100, 8700, 8900))}, {cube}, up, up_dir, 0.25f);
    // a letter like polygon and a triangle
    {
        Polygon l = poly({Point(1200, 1100), Point(7300, 1100), Point(7300, 3200), Point(3400, 3200), Point(3400, 8800), Point(1200, 8800)});
        Polygon t = poly({Point(5100, 4300), Point(8900, 5200), Point(6200, 8700)});
        run("cube_letter", {ExPolygon(l), ExPolygon(t)}, {cube}, up, up_dir, 0.75f);
    }
    // a square that sticks out of the top face
    run("cube_square_over_border", {ExPolygon(rect(6100, 2300, 13700, 6900))}, {cube}, up, up_dir, 0.5f);
    // a square beside the cube: nothing is cut
    run("cube_miss", {ExPolygon(rect(12100, 2300, 15700, 6900))}, {cube}, up, up_dir, 0.5f);

    // a sphere: the surface is curved, the quads of the shape leave diagonal vertices to reduce
    {
        indexed_triangle_set sphere = its_make_sphere(6., fa);
        Transform3d          tr     = shape_to_world(Vec3d(-5., -5., 2.));
        Vec3d                dir(0., 0., 8.);
        run("sphere_square", {ExPolygon(rect(2100, 3300, 6700, 7900))}, {sphere}, tr, dir, 0.5f);
        ExPolygon e(rect(1300, 1700, 8900, 8100));
        e.holes.push_back(rect_hole(3100, 3700, 6300, 5900));
        Polygon t = poly({Point(4100, 4300), Point(5900, 4700), Point(4700, 5600)});
        run("sphere_hole_island", {e, ExPolygon(t)}, {sphere}, tr, dir, 0.3f);
    }

    // a cylinder cut from the side: the projection is turned to look along x
    {
        indexed_triangle_set cyl = its_make_cylinder(5., 12., fa);
        Matrix3d             rot;
        rot << 0., 0., 1.,
               1., 0., 0.,
               0., 1., 0.; // shape x -> world y, shape y -> world z, shape z -> world x
        Transform3d tr = shape_to_world(Vec3d(1., -4., 1.5), rot);
        Vec3d       dir(9., 0., 0.);
        Polygon     l = poly({Point(1200, 1100), Point(7300, 1100), Point(7300, 3200), Point(3400, 3200), Point(3400, 8800), Point(1200, 8800)});
        run("cylinder_letter_side", {ExPolygon(l)}, {cyl}, tr, dir, 0.5f);
    }

    // two models: a cube and a sphere that comes out of its top face. The outer contour of the
    // shape goes around both (the models stay whole for the difference), the hole cuts both.
    {
        indexed_triangle_set sphere = its_make_sphere(3., fa);
        translate(sphere, 5.f, 5.f, 9.f);
        Transform3d tr = shape_to_world(Vec3d(0., 0., 7.5));
        Vec3d       dir(0., 0., 7.);
        ExPolygon   e(rect(-2300, -2100, 12700, 12900));
        e.holes.push_back(rect_hole(4300, 3100, 5900, 6300));
        run("cube_sphere_hole", {e}, {cube, sphere}, tr, dir, 0.5f);
        run("sphere_cube_hole", {e}, {sphere, cube}, tr, dir, 0.9f);
    }

    // two models: a cube and a cylinder lying across its top face, which splits the patch of the
    // top face in two
    {
        indexed_triangle_set cyl = its_make_cylinder(2., 13., fa);
        turn_y(cyl);
        translate(cyl, -1.5f, 5.f, 9.5f);
        Transform3d tr = shape_to_world(Vec3d(0., 0., 8.));
        Vec3d       dir(0., 0., 6.);
        ExPolygon   e(rect(-3300, -3100, 13700, 13900));
        e.holes.push_back(rect_hole(4100, 1200, 6300, 8700));
        run("cube_cylinder_split", {e}, {cube, cyl}, tr, dir, 0.5f);
    }

    // two cubes, one inside the other's footprint and higher: the patch of the lower top face is
    // inside the upper cube
    {
        indexed_triangle_set small = its_make_cube(6., 6., 6.);
        translate(small, 2.f, 2.5f, 8.f);
        Transform3d tr = shape_to_world(Vec3d(0., 0., 9.));
        Vec3d       dir(0., 0., 7.);
        run("cube_in_cube", {ExPolygon(rect(3100, 3300, 6700, 6900))}, {cube, small}, tr, dir, 0.5f);
    }

    // the functions cut_surface does not call
    {
        indexed_triangle_set sphere = its_make_sphere(3., fa);
        translate(sphere, 5.f, 5.f, 9.f);
        indexed_triangle_set cyl = its_make_cylinder(2., 13., fa);
        turn_y(cyl);
        translate(cyl, -1.5f, 5.f, 9.5f);
        direct_tree(cube, false, Vec3d(-3., -3., -3.), Vec3d(13., 13., 13.));
        direct_tree(cube, true, Vec3d(-3., -3., -3.), Vec3d(13., 13., 13.));
        direct_tree(sphere, true, Vec3d(1., 1., 5.), Vec3d(9., 9., 13.));
        direct_tree(cyl, false, Vec3d(-3., 2., 6.5), Vec3d(13., 8., 12.5));
        direct_m2i({3, 0, 2, 5});
        direct_m2i({1});
        direct_m2i({0, 4, 4, 0, 1});
        direct_close();
        direct_face_types(cube, sphere);
        direct_face_types(cube, cyl);
    }
    return 0;
}
