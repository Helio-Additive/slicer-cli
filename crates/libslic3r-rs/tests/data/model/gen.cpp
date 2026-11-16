// Generator of the vectors of tests/model_native_parity.rs: libslic3r's Model.hpp / Model.cpp at
// the pin (references/BambuStudio/src/libslic3r; the override layer has no copy of them).
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
//   ./gen > model.txt
//   gzip -9 -n -c model.txt > $DATA/model.txt.gz
//
// It runs a fixed script of Model / ModelObject / ModelVolume / ModelInstance operations on models
// built in code (boxes, multi-volume objects, instances with offsets, rotations, scales and
// mirrors) and prints one line per result. The Rust test runs the same script on the port and has
// to print the same lines. The vectors in this directory were made on macOS arm64 with Apple clang
// (its default -ffp-contract=on) and Eigen 5.0.1.
//
// The arithmetic of this file is compiled with `#pragma clang fp contract(off)` (after the
// includes), so its own expressions are evaluated as the Rust test writes them; the engine code
// it calls is compiled as the engine is.
//
// Line format: tokens separated by one space; a double is its 16 hexadecimal digits, a float its
// 8, an integer decimal, a text the hexadecimal of its bytes ("-" when empty). An object ID is
// written relative to the ID of the first model of the scenario. The first token names the line;
// lines whose name starts with "svd_" are results that depend on Geometry::Transformation's
// decomposition (Eigen's JacobiSVD, computeRotationScaling) or on its rotation getters, lines
// whose name starts with "geo_" results of other units (TriangleMesh.cpp mesh makers, BuildVolume).

#include "libslic3r/Model.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/Geometry.hpp"
#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/Arrange.hpp"
#include "libslic3r/BuildVolume.hpp"

#include <boost/log/core.hpp>

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

using namespace Slic3r;

namespace Slic3r {
// Model.cpp:4481, 4498 (not declared in Model.hpp)
double getadhesionCoeff(const ModelVolumePtrs objectVolumes);
double getTemperatureFromExtruder(const ModelVolumePtrs objectVolumes);
} // namespace Slic3r

#pragma clang fp contract(off)

static size_t g_base = 0;

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
static std::string hI(long long v) { return std::to_string(v); }
static std::string hB(bool v) { return v ? "1" : "0"; }
static std::string hT(const std::string &s)
{
    if (s.empty()) return "-";
    std::string o;
    char b[4];
    for (unsigned char c : s) {
        std::snprintf(b, sizeof(b), "%02x", c);
        o += b;
    }
    return o;
}
static std::string ID(const ObjectID &id) { return hI((long long) id.id - (long long) g_base); }
static std::string V3(const Vec3d &v) { return hD(v.x()) + " " + hD(v.y()) + " " + hD(v.z()); }
static std::string BBX(const BoundingBoxf3 &b) { return hB(b.defined) + " " + V3(b.min) + " " + V3(b.max); }
static std::string hM(const Transform3d &t)
{
    std::string o;
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 4; ++c) o += (o.empty() ? "" : " ") + hD(t.matrix()(r, c));
    return o;
}
static std::string M3(const Matrix3d &m)
{
    std::string o;
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) o += (o.empty() ? "" : " ") + hD(m(r, c));
    return o;
}
static std::string ST(const TriangleMeshStats &s)
{
    return hI(s.number_of_facets) + " " + hF(s.min.x()) + " " + hF(s.min.y()) + " " + hF(s.min.z()) + " " + hF(s.max.x()) + " " +
           hF(s.max.y()) + " " + hF(s.max.z()) + " " + hF(s.size.x()) + " " + hF(s.size.y()) + " " + hF(s.size.z()) + " " + hF(s.volume) +
           " " + hI(s.number_of_parts) + " " + hI(s.open_edges) + " " + hI(s.repaired_errors.degenerate_facets) + " " +
           hI(s.repaired_errors.edges_fixed) + " " + hI(s.repaired_errors.facets_removed) + " " + hI(s.repaired_errors.facets_reversed) +
           " " + hI(s.repaired_errors.backwards_edges);
}
// FNV-1a over the float bits of the vertices and the indices
static std::string ITS(const indexed_triangle_set &its)
{
    uint64_t h = 1469598103934665603ULL;
    auto mix = [&h](uint32_t v) {
        for (int k = 0; k < 4; ++k) {
            h ^= (v >> (8 * k)) & 0xff;
            h *= 1099511628211ULL;
        }
    };
    for (const Vec3f &v : its.vertices)
        for (int k = 0; k < 3; ++k) {
            uint32_t u;
            float f = v(k);
            std::memcpy(&u, &f, 4);
            mix(u);
        }
    for (const Vec3i &f : its.indices)
        for (int k = 0; k < 3; ++k) mix(uint32_t(f(k)));
    char b[32];
    std::snprintf(b, sizeof(b), "%016llx", (unsigned long long) h);
    return hI(its.vertices.size()) + " " + hI(its.indices.size()) + " " + b;
}
static std::string POLY(const Polygon &p)
{
    std::string o = hI(p.points.size());
    for (const Point &pt : p.points) o += " " + hI(pt.x()) + " " + hI(pt.y());
    return o;
}
static void out(const std::string &s) { std::printf("%s\n", s.c_str()); }

// splitmix64
struct Rng
{
    uint64_t s;
    explicit Rng(uint64_t seed) : s(seed) {}
    uint64_t next()
    {
        s += 0x9e3779b97f4a7c15ULL;
        uint64_t z = s;
        z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ULL;
        z = (z ^ (z >> 27)) * 0x94d049bb133111ebULL;
        return z ^ (z >> 31);
    }
    double uni() { return double(next() >> 11) * (1.0 / 9007199254740992.0); }
    double range(double a, double b) { return a + (b - a) * uni(); }
};

// A transformation built without Eigen's rotation code: a rotation of a normalized quaternion,
// scales per axis, an optional mirrored axis, a translation.
static Transform3d rand_affine(Rng &r, bool allow_mirror)
{
    double w = r.range(-1, 1), x = r.range(-1, 1), y = r.range(-1, 1), z = r.range(-1, 1);
    double n = std::sqrt(w * w + x * x + y * y + z * z);
    w /= n; x /= n; y /= n; z /= n;
    double R[3][3] = {{1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)},
                      {2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)},
                      {2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)}};
    double s[3] = {r.range(0.5, 2.0), r.range(0.5, 2.0), r.range(0.5, 2.0)};
    int mi = int(r.next() % 4);
    if (allow_mirror && mi < 3) s[mi] = -s[mi];
    Transform3d t = Transform3d::Identity();
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) t.matrix()(i, j) = R[i][j] * s[j];
    for (int i = 0; i < 3; ++i) t.matrix()(i, 3) = r.range(-50, 50);
    return t;
}

// A rotation about Z by `a` with a uniform scale and a translation (cos / sin of libm).
static Transform3d z_affine(double a, double sc, const Vec3d &tr)
{
    Transform3d t = Transform3d::Identity();
    double c = std::cos(a), s = std::sin(a);
    t.matrix()(0, 0) = c * sc;
    t.matrix()(0, 1) = -s * sc;
    t.matrix()(1, 0) = s * sc;
    t.matrix()(1, 1) = c * sc;
    t.matrix()(2, 2) = sc;
    t.matrix()(0, 3) = tr.x();
    t.matrix()(1, 3) = tr.y();
    t.matrix()(2, 3) = tr.z();
    return t;
}

static indexed_triangle_set box_its(double sx, double sy, double sz, float ox, float oy, float oz)
{
    indexed_triangle_set its = its_make_cube(sx, sy, sz);
    for (Vec3f &v : its.vertices) v += Vec3f(ox, oy, oz);
    return its;
}

static indexed_triangle_set two_boxes_its()
{
    indexed_triangle_set a = box_its(2, 3, 4, 1.f, 0.5f, 0.f);
    indexed_triangle_set b = box_its(1.5, 1, 2, 7.f, -2.f, 0.25f);
    its_merge(a, b);
    return a;
}

static indexed_triangle_set flat_its()
{
    indexed_triangle_set its;
    its.vertices = {Vec3f(0, 0, 0), Vec3f(1, 0, 0), Vec3f(1, 1, 0), Vec3f(0, 1, 0)};
    its.indices  = {Vec3i(0, 1, 2), Vec3i(0, 2, 3)};
    return its;
}

static void dump_volume(const std::string &tag, const ModelVolume &v)
{
    out(tag + " ids " + ID(v.id()) + " " + ID(v.config.id()) + " " + ID(v.supported_facets.id()) + " " + ID(v.seam_facets.id()) + " " +
        ID(v.mmu_segmentation_facets.id()) + " " + ID(v.fuzzy_skin_facets.id()) + " " + ID(v.exterior_facets.id()));
    out(tag + " type " + hI(int(v.type())) + " " + hT(v.name) + " " + hT(v.material_id()));
    out(tag + " m " + hM(v.get_matrix()));
    out(tag + " stats " + ST(v.mesh().stats()));
    out(tag + " its " + ITS(v.mesh().its));
    out(tag + " mbb " + BBX(v.mesh().bounding_box()) + " " + V3(v.source.mesh_offset) + " " + V3(v.mesh().get_init_shift()));
    if (v.get_convex_hull_shared_ptr()) {
        const TriangleMesh &h = v.get_convex_hull();
        out(tag + " hull " + hB(h.empty()) + " " + BBX(h.bounding_box()));
    } else
        out(tag + " hull none");
}

static void dump_object(const std::string &tag, const ModelObject &o)
{
    out(tag + " ids " + ID(o.id()) + " " + ID(o.config.id()) + " " + ID(o.layer_height_profile.id()) + " " + hT(o.name) + " " +
        hT(o.input_file) + " " + hI(o.volumes.size()) + " " + hI(o.instances.size()) + " " + hI(o.config.size()) + " " +
        hI(o.config.has("extruder") ? o.config.extruder() : -1) + " " + V3(o.origin_translation));
    for (size_t i = 0; i < o.volumes.size(); ++i) dump_volume(tag + " v" + hI(i), *o.volumes[i]);
    for (size_t i = 0; i < o.instances.size(); ++i) {
        const ModelInstance &mi = *o.instances[i];
        out(tag + " i" + hI(i) + " " + ID(mi.id()) + " " + hM(mi.get_matrix()) + " " + hB(mi.printable) + " " + hI(int(mi.print_volume_state)) +
            " " + hB(mi.is_printable()) + " " + hI((long long) mi.get_labeled_id() - (long long) g_base));
    }
}

static void dump_object_geometry(const std::string &tag, const ModelObject &o)
{
    out(tag + " bb " + BBX(o.bounding_box()));
    out(tag + " rmb " + BBX(o.raw_mesh_bounding_box()));
    out(tag + " frb " + BBX(o.full_raw_mesh_bounding_box()));
    if (!o.instances.empty()) out(tag + " rbb " + BBX(o.raw_bounding_box()));
    for (size_t i = 0; i < o.instances.size(); ++i) {
        out(tag + " ibb " + hI(i) + " " + BBX(o.instance_bounding_box(i, false)) + " " + BBX(o.instance_bounding_box(i, true)));
        out(tag + " ibb2 " + hI(i) + " " + BBX(o.instance_bounding_box(*o.instances[i], false)));
        out(tag + " ichb " + hI(i) + " " + BBX(o.instance_convex_hull_bounding_box(i, false)) + " " +
            BBX(o.instance_convex_hull_bounding_box(i, true)));
        out(tag + " iz " + hI(i) + " " + hD(o.get_instance_min_z(i)) + " " + hD(o.get_instance_max_z(i)));
    }
    out(tag + " z " + hD(o.get_min_z()) + " " + hD(o.get_max_z()));
    TriangleMeshStats st = o.get_object_stl_stats();
    out(tag + " stl " + ST(st) + " " + hI(o.get_repaired_errors_count()));
    out(tag + " cnt " + hI(o.facets_count()) + " " + hI(o.parts_count()) + " " + hI(o.materials_count()) + " " + hB(o.is_multiparts()) + " " +
        hB(o.has_custom_layering()) + " " + hB(o.is_cut()));
    TriangleMesh mesh = o.mesh();
    out(tag + " mesh " + ST(mesh.stats()) + " " + ITS(mesh.its));
    TriangleMesh raw = o.raw_mesh();
    out(tag + " raw " + ST(raw.stats()) + " " + ITS(raw.its));
    out(tag + " rits " + ITS(o.raw_indexed_triangle_set()));
}

// Model 1: three objects.
static Model build_model_1()
{
    Rng r(12345);
    Model m;
    g_base = m.id().id;
    {
        ModelObject *o = m.add_object();
        o->name = "box";
        o->input_file = "/tmp/in/box.stl";
        ModelVolume *v = o->add_volume(TriangleMesh(box_its(10, 20, 30, 5.f, -3.f, 2.f)));
        v->name = "box_v";
        v->set_transformation(Geometry::Transformation(rand_affine(r, false)));
        ModelInstance *i0 = o->add_instance();
        i0->set_transformation(Geometry::Transformation(rand_affine(r, true)));
        ModelInstance *i1 = o->add_instance();
        i1->set_offset(Vec3d(r.range(-80, 80), r.range(-80, 80), r.range(-5, 5)));
    }
    {
        ModelObject *o = m.add_object();
        o->name = "multi";
        ModelVolume *v0 = o->add_volume(TriangleMesh(box_its(4, 4, 4, 0.f, 0.f, 0.f)));
        v0->set_transformation(Geometry::Transformation(rand_affine(r, false)));
        ModelVolume *v1 = o->add_volume(TriangleMesh(box_its(2, 3, 1, 1.f, 1.f, 1.f)), ModelVolumeType::PARAMETER_MODIFIER);
        v1->set_offset(Vec3d(r.range(-3, 3), r.range(-3, 3), r.range(-3, 3)));
        ModelVolume *v2 = o->add_volume(TriangleMesh(box_its(1, 1, 8, -1.f, -1.f, 0.f)), ModelVolumeType::NEGATIVE_VOLUME);
        v2->set_transformation(Geometry::Transformation(z_affine(0.3, 1.25, Vec3d(0.5, -0.25, 1.0))));
        ModelVolume *v3 = o->add_volume(TriangleMesh(two_boxes_its()), ModelVolumeType::MODEL_PART);
        v3->set_transformation(Geometry::Transformation(rand_affine(r, true)));
        for (int k = 0; k < 3; ++k) {
            ModelInstance *mi = o->add_instance();
            if (k == 2)
                mi->set_offset(Vec3d(10.5, -20.25, 3.0));
            else
                mi->set_transformation(Geometry::Transformation(rand_affine(r, k == 1)));
        }
        o->config.set("extruder", 2);
    }
    {
        TriangleMesh mesh(box_its(3, 2, 1, -10.f, 4.f, 6.f));
        ModelObject *o = m.add_object("named", "/tmp/in/named.obj", std::move(mesh));
        o->add_instance()->set_transformation(Geometry::Transformation(z_affine(1.1, 0.75, Vec3d(30, 40, 0))));
    }
    return m;
}

// Model 2: one object, one volume of two disjoint boxes, two instances.
static Model build_model_2()
{
    Model m;
    g_base = m.id().id;
    ModelObject *o = m.add_object();
    o->name = "two";
    ModelVolume *v = o->add_volume(TriangleMesh(two_boxes_its()));
    v->name = "two_v";
    o->add_instance()->set_transformation(Geometry::Transformation(z_affine(0.5, 1.0, Vec3d(20, 30, 5))));
    o->add_instance()->set_offset(Vec3d(-15, 2, 0));
    return m;
}

// A * B with plain loops (this file's arithmetic, unfused).
static Transform3d mul_plain(const Transform3d &A, const Transform3d &Bm)
{
    Transform3d r = Transform3d::Identity();
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 4; ++j) {
            double acc = 0;
            for (int k = 0; k < 3; ++k) acc += A.matrix()(i, k) * Bm.matrix()(k, j);
            if (j == 3) acc += A.matrix()(i, 3);
            r.matrix()(i, j) = acc;
        }
    return r;
}

// Model 3: one object of two volumes; instance 1 is instance 0 turned about Z (for
// bake_xy_rotation_into_meshes).
static Model build_model_3()
{
    Rng r(999);
    Model m;
    g_base = m.id().id;
    ModelObject *o = m.add_object();
    o->name = "bake";
    o->add_volume(TriangleMesh(box_its(6, 4, 2, 0.f, 0.f, 0.f)))->set_transformation(Geometry::Transformation(rand_affine(r, false)));
    o->add_volume(TriangleMesh(box_its(1, 2, 3, 2.f, 1.f, 0.f)))->set_offset(Vec3d(1.5, -0.5, 0.25));
    Transform3d t0 = rand_affine(r, false);
    o->add_instance()->set_transformation(Geometry::Transformation(t0));
    o->add_instance()->set_transformation(Geometry::Transformation(mul_plain(z_affine(0.7, 1.0, Vec3d(25, -5, 0)), t0)));
    return m;
}

static void scenario_prim()
{
    Rng r(777);
    for (int k = 0; k < 40; ++k) {
        Transform3d A = rand_affine(r, true), Bm = rand_affine(r, true);
        Vec3d v(r.range(-30, 30), r.range(-30, 30), r.range(-30, 30));
        out("prim_mul " + hM(A * Bm));
        out("prim_pt " + V3(A * v));
        out("prim_det " + hD(A.matrix().block<3, 3>(0, 0).determinant()));
        out("prim_inv " + hM(A.inverse()));
        Matrix3d a3 = A.matrix().block<3, 3>(0, 0), b3 = Bm.matrix().block<3, 3>(0, 0);
        out("prim_m3v " + V3(a3 * v));
        out("prim_m3m " + M3(a3 * b3));
        BoundingBoxf3 bb(Vec3d(r.range(-10, 0), r.range(-10, 0), r.range(-10, 0)), Vec3d(r.range(0, 10), r.range(0, 10), r.range(0, 10)));
        out("prim_bbt " + BBX(bb.transformed(A)));
        double ang = r.range(-3, 3);
        double ax = r.range(-1, 1), ay = r.range(-1, 1), az = r.range(-1, 1);
        double an = std::sqrt(ax * ax + ay * ay + az * az);
        Vec3d axis(ax / an, ay / an, az / an);
        out("prim_aa " + M3(Eigen::AngleAxisd(ang, axis).toRotationMatrix()));
        out("prim_qaa " + M3(Eigen::Quaterniond(Eigen::AngleAxisd(ang, axis)).toRotationMatrix()));
        Geometry::Transformation t(A);
        out("prim_mt " + hM(t.get_matrix(true)));
        out("svd_rot " + V3(t.get_rotation()));
        out("svd_scale " + V3(t.get_scaling_factor()));
        out("svd_mirror " + V3(t.get_mirror()));
        out("svd_mtfsm " + hM(t.get_matrix(true, false, true, true)));
        out("svd_euler " + V3(Geometry::extract_euler_angles(Eigen::Quaterniond(Eigen::AngleAxisd(ang, axis)).toRotationMatrix())));
        Vec3d rot(r.range(-3, 3), r.range(-3, 3), r.range(-3, 3));
        out("svd_rott " + hM(Geometry::rotation_transform(rot)));
    }
}

static void scenario_build()
{
    Model m = build_model_1();
    out("model " + ID(m.id()) + " " + ID(m.wipe_tower.id()) + " " + hI(m.objects.size()));
    for (size_t i = 0; i < m.objects.size(); ++i) {
        dump_object("o" + hI(i), *m.objects[i]);
        dump_object_geometry("g" + hI(i), *m.objects[i]);
    }
    out("mbb " + BBX(m.bounding_box()));
    TriangleMesh mm = m.mesh();
    out("mmesh " + ST(mm.stats()) + " " + ITS(mm.its));
    out("mflags " + hB(m.looks_like_multipart_object()) + " " + hB(m.looks_like_imperial_units()) + " " + hB(m.looks_like_saved_in_meters()) +
        " " + hB(model_has_multi_part_objects(m)) + " " + hB(model_has_advanced_features(m)) + " " + hB(m.is_fdm_support_painted()) + " " +
        hB(m.is_seam_painted()) + " " + hB(m.is_mm_painted()));
    out("mname " + hT(m.propose_export_file_name_and_path()) + " " + hT(m.propose_export_file_name_and_path(".3mf")));
    for (size_t i = 0; i < m.objects.size(); ++i) out("oname " + hI(i) + " " + hT(m.objects[i]->get_export_filename()));
    // 2D convex hulls of the instances
    for (size_t i = 0; i < m.objects.size(); ++i)
        for (size_t j = 0; j < m.objects[i]->instances.size(); ++j) {
            ModelInstance *mi = m.objects[i]->instances[j];
            out("svd_ch2d " + hI(i) + " " + hI(j) + " " + POLY(mi->convex_hull_2d()));
        }
    // the volume extruders
    ModelObject *o1 = m.objects[1];
    o1->volumes[0]->config.set("wall_filament", 3);
    o1->volumes[1]->config.set("extruder", 4);
    o1->config.set("sparse_infill_filament", 5);
    for (size_t j = 0; j < o1->volumes.size(); ++j) {
        GlobalFeatureFilaments gf;
        gf.solid_infill_filament = 6;
        std::vector<int> e = o1->volumes[j]->get_extruders(gf);
        std::string s = "ext " + hI(j) + " " + hI(o1->volumes[j]->extruder_id()) + " " + hI(e.size());
        for (int x : e) s += " " + hI(x);
        out(s + " " + hB(o1->volumes[j]->is_splittable()) + " " + hB(o1->volumes[j]->is_the_only_one_part()));
    }
    out("ext2 " + hI(m.objects[0]->volumes[0]->extruder_id()) + " " + hI(m.objects[2]->volumes[0]->extruder_id()) + " " +
        hB(m.objects[0]->volumes[0]->is_the_only_one_part()));
    out("mflags2 " + hB(model_has_advanced_features(m)));
}

static void scenario_ops()
{
    // translate with a cached bounding box: m_bounding_box translated, raw mesh box stale
    {
        Model m = build_model_1();
        ModelObject *o = m.objects[0];
        out("t0 " + BBX(o->bounding_box()) + " " + BBX(o->raw_mesh_bounding_box()));
        o->translate(1.5, -2.25, 0.125);
        out("t1 " + BBX(o->bounding_box()) + " " + BBX(o->raw_mesh_bounding_box()) + " " + hM(o->volumes[0]->get_matrix()));
        o->invalidate_bounding_box();
        out("t2 " + BBX(o->bounding_box()) + " " + BBX(o->raw_mesh_bounding_box()));
        m.translate(3, 4, 5);
        out("t3 " + BBX(m.bounding_box()));
    }
    // center_around_origin, translate_instances, ensure_on_bed, adjust_min_z
    {
        Model m = build_model_1();
        ModelObject *o = m.objects[1];
        o->center_around_origin(true);
        out("c0 " + V3(o->origin_translation) + " " + BBX(o->full_raw_mesh_bounding_box()));
        o->center_around_origin(false);
        out("c1 " + V3(o->origin_translation) + " " + BBX(o->raw_mesh_bounding_box()));
        o->translate_instances(Vec3d(1, 2, -40));
        o->translate_instance(1, Vec3d(-0.5, 0.25, 0.75));
        out("c2 " + BBX(o->bounding_box()) + " " + hD(o->get_min_z()));
        o->ensure_on_bed(false);
        out("c3 " + hD(o->get_min_z()) + " " + hM(o->instances[0]->get_matrix()));
        o->translate_instances(Vec3d(0, 0, -0.0005));
        o->ensure_on_bed(true);
        out("c4 " + hD(o->get_min_z()) + " " + hD(o->get_max_z()));
        m.objects[0]->translate_instances(Vec3d(0, 0, -100));
        m.adjust_min_z();
        out("c5 " + BBX(m.bounding_box()) + " " + BBX(m.objects[0]->bounding_box()));
        bool moved = m.center_instances_around_point(Vec2d(128, 96));
        out("c6 " + hB(moved) + " " + BBX(m.bounding_box()) + " " + hB(m.center_instances_around_point(Vec2d(128, 96))));
    }
    // delete_volume collapses the last volume's transformation into the instances
    {
        Model m = build_model_1();
        ModelObject *o = m.objects[1];
        o->delete_volume(1);
        o->delete_volume(1);
        out("d0 " + hI(o->volumes.size()));
        o->delete_volume(0);
        dump_object("d1", *o);
        dump_object_geometry("d1g", *o);
        o->delete_instance(0);
        o->delete_last_instance();
        out("d2 " + hI(o->instances.size()) + " " + BBX(o->bounding_box()));
        o->clear_instances();
        m.add_default_instances();
        out("d3 " + hI(o->instances.size()) + " " + ID(o->instances[0]->id()) + " " + BBX(o->bounding_box()));
    }
    // sort_volumes
    {
        Model m = build_model_1();
        ModelObject *o = m.objects[1];
        o->volumes[0]->set_type(ModelVolumeType::SUPPORT_ENFORCER);
        o->sort_volumes(false);
        std::string s = "s0";
        for (ModelVolume *v : o->volumes) s += " " + ID(v->id()) + " " + hI(int(v->type()));
        out(s);
        o->sort_volumes(true);
        s = "s1";
        for (ModelVolume *v : o->volumes) s += " " + ID(v->id()) + " " + hI(int(v->type()));
        out(s);
    }
    // units
    {
        Model m = build_model_1();
        Model small;
        g_base = m.id().id;
        ModelObject *so = small.add_object();
        so->add_volume(TriangleMesh(box_its(0.1, 0.1, 0.1, 0.f, 0.f, 0.f)));
        so->add_instance();
        out("u0 " + hB(small.looks_like_imperial_units()) + " " + hB(small.looks_like_saved_in_meters()) + " " + ST(so->get_object_stl_stats()));
        small.convert_from_meters(true);
        dump_object("u1", *so);
        out("u1g " + BBX(so->bounding_box()) + " " + ST(so->get_object_stl_stats()));
        small.convert_from_imperial_units(false);
        dump_object("u2", *so);
        m.objects[0]->scale_mesh_after_creation(2.5f);
        dump_object("u3", *m.objects[0]);
        dump_object_geometry("u3g", *m.objects[0]);
        ModelObject *fo = m.add_object();
        fo->add_volume(TriangleMesh(flat_its()));
        fo->add_instance();
        out("u4 " + ST(fo->get_object_stl_stats()) + " " + hB(fo->volumes[0]->get_convex_hull().empty()) + " " + hI(m.objects.size()));
        int removed = m.removed_objects_with_zero_volume();
        out("u5 " + hI(removed) + " " + hI(m.objects.size()));
    }
    // ModelVolume conversions and mesh transforms
    {
        Model m = build_model_1();
        ModelVolume *v = m.objects[0]->volumes[0];
        v->convert_from_imperial_units();
        dump_volume("vc0", *v);
        ModelVolume *w = m.objects[1]->volumes[3];
        w->convert_from_meters();
        dump_volume("vc1", *w);
    }
    // split and merge
    {
        Model m = build_model_2();
        ModelObjectPtrs new_objects;
        m.objects[0]->split(&new_objects);
        out("sp0 " + hI(new_objects.size()) + " " + hI(m.objects.size()));
        for (size_t i = 0; i < new_objects.size(); ++i) {
            dump_object("sp" + hI(i), *new_objects[i]);
            dump_object_geometry("spg" + hI(i), *new_objects[i]);
        }
    }
    {
        Model m = build_model_1();
        ModelObjectPtrs new_objects;
        m.objects[1]->split(&new_objects);
        out("sq0 " + hI(new_objects.size()));
        for (size_t i = 0; i < new_objects.size(); ++i) dump_object("sq" + hI(i), *new_objects[i]);
        m.objects[1]->merge();
        dump_object("mg", *m.objects[1]);
        dump_object_geometry("mgg", *m.objects[1]);
    }
    {
        Model m = build_model_2();
        ModelObject *o = m.objects[0];
        size_t n = o->volumes[0]->split(1);
        out("vs0 " + hI(n) + " " + hI(o->volumes.size()));
        for (size_t i = 0; i < o->volumes.size(); ++i) dump_volume("vs" + hI(i), *o->volumes[i]);
        dump_object_geometry("vsg", *o);
    }
    // convert_multipart_object
    {
        Model m = build_model_1();
        m.delete_object(size_t(1));
        out("mp0 " + hB(m.looks_like_multipart_object()));
        m.convert_multipart_object(4);
        dump_object("mp1", *m.objects[0]);
        dump_object_geometry("mp1g", *m.objects[0]);
    }
    // duplicate_objects_grid
    {
        Model m = build_model_2();
        m.duplicate_objects_grid(2, 3, 5.0);
        dump_object("dg", *m.objects[0]);
        out("dgb " + BBX(m.bounding_box()));
    }
}

static void scenario_ids()
{
    Model m = build_model_1();
    Model copy(m);
    out("cp0 " + ID(copy.id()) + " " + ID(copy.wipe_tower.id()) + " " + hB(model_object_list_equal(m, copy)) + " " +
        hB(model_object_list_extended(m, copy)));
    dump_object("cp1", *copy.objects[1]);
    ModelObject *cl = copy.add_object(*m.objects[1]);
    out("cp2 " + hB(model_object_list_equal(m, copy)) + " " + hB(model_object_list_extended(m, copy)));
    dump_object("cp3", *cl);
    dump_object_geometry("cp3g", *cl);
    // volume list changes against the copy
    ModelObject &a = *m.objects[1];
    ModelObject &b = *copy.objects[1];
    out("vl0 " + hB(model_volume_list_changed(a, b, ModelVolumeType::MODEL_PART)) + " " +
        hB(model_volume_list_changed(a, b, {ModelVolumeType::PARAMETER_MODIFIER, ModelVolumeType::NEGATIVE_VOLUME})) + " " +
        hB(model_custom_supports_data_changed(a, b)) + " " + hB(model_custom_seam_data_changed(a, b)) + " " +
        hB(model_mmu_segmentation_data_changed(a, b)) + " " + hB(model_custom_fuzzy_skin_data_changed(a, b)) + " " +
        hB(model_brim_points_data_changed(a, b)));
    b.volumes[0]->supported_facets.reset();
    b.volumes[3]->mmu_segmentation_facets.reset();
    out("vl1 " + hB(model_custom_supports_data_changed(a, b)) + " " + hB(model_mmu_segmentation_data_changed(a, b)) + " " +
        hB(model_custom_seam_data_changed(a, b)));
    b.volumes[1]->set_offset(Vec3d(9, 9, 9));
    out("vl2 " + hB(model_volume_list_changed(a, b, ModelVolumeType::MODEL_PART)) + " " +
        hB(model_volume_list_changed(a, b, ModelVolumeType::PARAMETER_MODIFIER)));
    b.volumes[0]->set_new_unique_id();
    out("vl3 " + hB(model_volume_list_changed(a, b, ModelVolumeType::MODEL_PART)) + " " +
        hB(model_volume_list_changed(a, b, ModelVolumeType::NEGATIVE_VOLUME)) + " " + ID(b.volumes[0]->id()) + " " +
        ID(b.volumes[0]->config.id()));
    b.delete_volume(2);
    out("vl4 " + hB(model_volume_list_changed(a, b, ModelVolumeType::NEGATIVE_VOLUME)) + " " +
        hB(model_volume_list_changed(a, b, ModelVolumeType::PARAMETER_MODIFIER)));
    // materials
    m.add_material("pla");
    ModelMaterial *mat = m.add_material("petg");
    out("mat0 " + hI(m.materials.size()) + " " + ID(mat->id()) + " " + ID(mat->config.id()));
    m.objects[0]->volumes[0]->set_material_id("pla");
    m.objects[1]->volumes[0]->set_material_id("abs");
    out("mat1 " + hI(m.materials.size()) + " " + hI(m.objects[1]->materials_count()) + " " + ID(m.materials["abs"]->id()));
    m.delete_material("pla");
    m.clear_materials();
    out("mat2 " + hI(m.materials.size()));
    // delete objects
    ObjectID id1 = m.objects[1]->id();
    out("del0 " + hB(m.delete_object(id1)) + " " + hB(m.delete_object(id1)) + " " + hI(m.objects.size()));
    m.clear_objects();
    out("del1 " + hI(m.objects.size()));
}

static void scenario_instance()
{
    Model m = build_model_1();
    ModelObject *o = m.objects[1];
    ModelInstance *mi = o->instances[0];
    Rng r(4242);
    for (int k = 0; k < 6; ++k) {
        Vec3d v(r.range(-20, 20), r.range(-20, 20), r.range(-20, 20));
        out("tv " + V3(mi->transform_vector(v, false)) + " " + V3(mi->transform_vector(v, true)));
        BoundingBoxf3 bb(Vec3d(r.range(-10, 0), r.range(-10, 0), r.range(-10, 0)), Vec3d(r.range(0, 10), r.range(0, 10), r.range(0, 10)));
        out("tb " + BBX(mi->transform_bounding_box(bb, false)) + " " + BBX(mi->transform_bounding_box(bb, true)));
    }
    out("svd_tmb " + BBX(mi->transform_mesh_bounding_box(o->volumes[0]->mesh(), false)) + " " +
        BBX(mi->transform_mesh_bounding_box(o->volumes[0]->mesh(), true)));
    TriangleMesh tm = o->volumes[3]->mesh();
    mi->transform_mesh(&tm, false);
    out("tm " + ST(tm.stats()) + " " + ITS(tm.its));
    Polygon poly({Point(1000000, 0), Point(0, 2000000), Point(-1500000, -500000)});
    mi->transform_polygon(&poly);
    out("svd_tp " + POLY(poly));
    out("svd_asm " + hM(mi->get_assemble_transformation().get_matrix()) + " " + hB(mi->is_assemble_initialized()));
    mi->set_assemble_offset(Vec3d(1, 2, 3));
    out("svd_asm2 " + hM(mi->get_assemble_transformation().get_matrix()) + " " + hB(mi->is_assemble_initialized()));
    out("svd_bbav " + BBX(o->bounding_box_in_assembly_view()) + " " + BBX(m.bounding_box_in_assembly_view(o)));
    ModelInstance *m2 = o->instances[2];
    m2->apply_arrange_result(Vec2d(12345678, -2345678), 0.75);
    out("svd_arr " + hM(m2->get_matrix()) + " " + BBX(o->bounding_box()));
    m2->rotate(Eigen::AngleAxisd(0.5, Vec3d::UnitZ()).toRotationMatrix());
    out("svd_irot " + hM(m2->get_matrix()));
    m2->printable = false;
    out("prn " + hB(m2->is_printable()) + " " + hB(o->instances[0]->is_printable()) + " " + hT(m.propose_export_file_name_and_path()));
    o->printable = false;
    out("prn2 " + hB(o->instances[0]->is_printable()) + " " + hT(m.propose_export_file_name_and_path()));
    // transformations set through the decomposition
    ModelInstance *m1 = o->instances[1];
    m1->set_rotation(Vec3d(0.1, -0.2, 0.3));
    out("svd_set0 " + hM(m1->get_matrix()));
    m1->set_scaling_factor(Vec3d(1.5, 0.75, 2.0));
    out("svd_set1 " + hM(m1->get_matrix()));
    m1->set_mirror(Vec3d(-1, 1, 1));
    out("svd_set2 " + hM(m1->get_matrix()) + " " + V3(m1->get_rotation()) + " " + V3(m1->get_scaling_factor()) + " " + V3(m1->get_mirror()));
    // object operations through the decomposition
    o->scale(Vec3d(1.25, 0.5, 2.0));
    out("svd_os " + hM(o->volumes[0]->get_matrix()) + " " + BBX(o->bounding_box()));
    o->mirror(Axis::Y);
    out("svd_om " + hM(o->volumes[0]->get_matrix()) + " " + BBX(o->bounding_box()));
    o->rotate(0.4, Axis::X);
    out("svd_or " + hM(o->volumes[0]->get_matrix()) + " " + V3(o->origin_translation));
    o->rotate(-0.7, Vec3d(0, 0.6, 0.8));
    out("svd_or2 " + hM(o->volumes[0]->get_matrix()) + " " + V3(o->origin_translation) + " " + hM(o->instances[0]->get_assemble_transformation().get_matrix()));
    o->scale_to_fit(Vec3d(50, 60, 70));
    out("svd_osf " + hM(o->volumes[0]->get_matrix()) + " " + BBX(o->bounding_box()));
    Model m3 = build_model_3();
    m3.objects[0]->bake_xy_rotation_into_meshes(0);
    dump_object("svd_bake", *m3.objects[0]);
}


static std::string FACETS(const ModelVolume &v, const FacetsAnnotation &f)
{
    std::string o = hI(v.mesh().its.indices.size());
    for (size_t i = 0; i < v.mesh().its.indices.size(); ++i) {
        std::string t = f.get_triangle_as_string(int(i));
        if (!t.empty()) o += " " + hI(i) + ":" + t;
    }
    return o;
}

static void scenario_extra()
{
    // painted volume: extruders, facets per state, clamping
    {
        Model m = build_model_2();
        ModelObject *o = m.objects[0];
        ModelVolume *v = o->volumes[0];
        v->mmu_segmentation_facets.set_triangle_from_string(0, "4");
        v->mmu_segmentation_facets.set_triangle_from_string(3, "8");
        v->mmu_segmentation_facets.set_triangle_from_string(5, "1C");
        v->mmu_segmentation_facets.set_triangle_from_string(14, "8");
        v->supported_facets.set_triangle_from_string(2, "4");
        std::vector<int> e = v->get_extruders(GlobalFeatureFilaments());
        std::string s = "pe0 " + hI(e.size());
        for (int x : e) s += " " + hI(x);
        out(s + " " + hB(v->mmu_segmentation_facets.has_facets(*v, EnforcerBlockerType::Extruder2)) + " " +
            hB(v->mmu_segmentation_facets.has_facets(*v, EnforcerBlockerType(7))));
        for (int st = 0; st < 5; ++st) {
            out("pf " + hI(st) + " " + ITS(v->mmu_segmentation_facets.get_facets(*v, EnforcerBlockerType(st))) + " " +
                ITS(v->mmu_segmentation_facets.get_facets_strict(*v, EnforcerBlockerType(st))));
        }
        std::vector<indexed_triangle_set> per_type;
        v->mmu_segmentation_facets.get_facets(*v, per_type);
        s = "pft " + hI(per_type.size());
        for (const indexed_triangle_set &its : per_type) s += " " + hI(its.indices.size());
        out(s);
        bool changed = v->mmu_segmentation_facets.replace_state(*v, EnforcerBlockerType::Extruder2, EnforcerBlockerType(3));
        out("pr " + hB(changed) + " " + FACETS(*v, v->mmu_segmentation_facets));
        v->mmu_segmentation_facets.shift_states_above(*v, EnforcerBlockerType(2), 1);
        out("ps " + FACETS(*v, v->mmu_segmentation_facets));
        e = v->get_extruders(GlobalFeatureFilaments());
        s = "pe1 " + hI(e.size());
        for (int x : e) s += " " + hI(x);
        out(s);
        v->config.set("extruder", 6);
        v->update_extruder_count(4);
        out("pu " + FACETS(*v, v->mmu_segmentation_facets) + " " + hB(v->config.has("extruder")));
        v->config.set("extruder", 3);
        v->update_extruder_count_when_delete_filament(4, 2, -1, {});
        out("pd " + FACETS(*v, v->mmu_segmentation_facets) + " " + hI(v->extruder_id()) + " " + hB(v->config.has("extruder")));
        v->reset_extra_facets();
        out("px " + hB(v->is_mm_painted()) + " " + hB(v->is_fdm_support_painted()) + " " + hB(o->is_mm_painted()) + " " +
            hB(m.is_mm_painted()));
    }
    // extruder parameters, brim width, arrange polygon
    {
        Model m = build_model_1();
        Model::extruderParamsMap = {{0, {"PLA", 0, 210.}}, {1, {"PLA", 0, 210.}}, {2, {"PETG", 60, 240.}}, {4, {"TPU", 45, 220.}}};
        for (size_t i = 0; i < m.objects.size(); ++i)
            out("ad " + hI(i) + " " + hD(getadhesionCoeff(m.objects[i]->volumes)) + " " + hD(getTemperatureFromExtruder(m.objects[i]->volumes)) +
                " " + hD(Model::getThermalLength(m.objects[i]->volumes)) + " " + hD(Model::findMaxSpeed(m.objects[i])));
        Model::printSpeedMap.maxSpeed = 180;
        Model::printSpeedMap.perimeterSpeed = 150;
        Model::printSpeedMap.externalPerimeterSpeed = 120;
        m.objects[1]->config.set_key_value("inner_wall_speed", new ConfigOptionFloatsNullable({90.}));
        m.objects[1]->config.set_key_value("support_speed", new ConfigOptionFloatsNullable({130.}));
        for (size_t i = 0; i < m.objects.size(); ++i)
            out("br " + hI(i) + " " + hD(Model::findMaxSpeed(m.objects[i])) + " " + hD(m.objects[i]->instances[0]->get_auto_brim_width(10., 1.5)) +
                " " + hD(m.objects[i]->instances[0]->get_auto_brim_width()));
        DynamicPrintConfig cfg;
        cfg.set_key_value("filament_type", new ConfigOptionStrings({"PLA", "PETG", "TPU"}));
        cfg.set_key_value("nozzle_temperature", new ConfigOptionIntsNullable({210, 240, 225}));
        Model::setExtruderParams(cfg, 3);
        std::string s = "xp " + hI(Model::extruderParamsMap.size());
        for (auto &kv : Model::extruderParamsMap)
            s += " " + hI(kv.first) + " " + hT(kv.second.materialName) + " " + hI(kv.second.bedTemp) + " " + hD(kv.second.heatEndTemp);
        out(s);
        cfg.set_key_value("enable_support", new ConfigOptionBool(true));
        cfg.set_key_value("support_filament", new ConfigOptionInt(3));
        cfg.set_key_value("support_interface_filament", new ConfigOptionInt(0));
        cfg.set_key_value("filament_diameter", new ConfigOptionFloats({1.75, 1.75, 1.75}));
        cfg.set_key_value("wall_filament", new ConfigOptionInt(2));
        for (size_t i = 0; i < m.objects.size(); ++i)
            for (size_t j = 0; j < m.objects[i]->instances.size(); ++j) {
                arrangement::ArrangePolygon ap;
                m.objects[i]->instances[j]->get_arrange_polygon(&ap, cfg);
                s = "svd_ap " + hI(i) + " " + hI(j) + " " + POLY(ap.poly.contour) + " " + hI(ap.translation.x()) + " " + hI(ap.translation.y()) +
                    " " + hD(ap.rotation) + " " + hI(ap.extrude_id_filament_types.size());
                for (auto &kv : ap.extrude_id_filament_types) s += " " + hI(kv.first) + " " + hT(kv.second);
                out(s);
            }
        Model::extruderParamsMap = {{0, {"", 0, 0}}};
        Model::printSpeedMap = GlobalSpeedMap{};
    }
    // print volume state
    {
        Model m = build_model_1();
        BuildVolume bv({Vec2d(-60, -60), Vec2d(60, -60), Vec2d(60, 60), Vec2d(-60, 60)}, 80., {}, {});
        unsigned int n = m.update_print_volume_state(bv);
        std::string s = "pv " + hI(n);
        for (ModelObject *o : m.objects)
            for (ModelInstance *mi : o->instances) s += " " + hI(int(mi->print_volume_state)) + " " + hB(mi->is_printable());
        out(s);
        BuildVolume bc({Vec2d(0, -100), Vec2d(70.7, -70.7), Vec2d(100, 0), Vec2d(70.7, 70.7), Vec2d(0, 100), Vec2d(-70.7, 70.7), Vec2d(-100, 0),
                        Vec2d(-70.7, -70.7)},
                       100., {}, {});
        n = m.update_print_volume_state(bc);
        out("geo_bvtype " + hI(int(bc.type())));
        s = "pvc " + hI(n);
        for (ModelObject *o : m.objects)
            for (ModelInstance *mi : o->instances) s += " " + hI(int(mi->print_volume_state));
        out(s);
    }
    // cut bookkeeping and connectors
    {
        Model m = build_model_1();
        ModelObject *o = m.objects[0];
        ModelObjectCutAttributes attrs = ModelObjectCutAttribute::KeepUpper | ModelObjectCutAttribute::KeepLower |
                                         ModelObjectCutAttribute::CreateDowels;
        o->apply_cut_attributes(attrs);
        out("cu0 " + hB(o->is_cut()) + " " + ID(o->cut_id.id()) + " " + hI(o->cut_id.check_sum()) + " " + hI(o->cut_id.connectors_cnt()));
        o->cut_connectors.emplace_back(Vec3d(1, 2, 3), Transform3d::Identity(), 2.5f, 4.f, 0.1f, 0.2f,
                                       CutConnectorAttributes(CutConnectorType::Plug, CutConnectorStyle::Prizm, CutConnectorShape::Square));
        o->cut_connectors.emplace_back(Vec3d(-1, 0, 2), Transform3d(z_affine(0.25, 1.0, Vec3d(0, 0, 0))), 1.5f, 3.f, 0.f, 0.1f,
                                       CutConnectorAttributes(CutConnectorType::Plug, CutConnectorStyle::Frustum, CutConnectorShape::Triangle));
        o->apply_cut_connectors("conn");
        out("cu1 " + hB(o->has_connectors()) + " " + hI(o->cut_id.connectors_cnt()) + " " + hI(o->cut_connectors.size()));
        for (size_t i = 1; i < o->volumes.size(); ++i) {
            dump_volume("cuv" + hI(i), *o->volumes[i]);
            const ModelVolume::CutInfo &ci = o->volumes[i]->cut_info;
            out("cuc " + hB(ci.is_connector) + " " + hB(ci.is_processed) + " " + hI(int(ci.connector_type)) + " " + hF(ci.radius) + " " + hF(ci.height) +
                " " + hB(o->volumes[i]->is_cut_connector()));
        }
        o->volumes[1]->cut_info.set_processed();
        o->delete_connectors();
        out("cu2 " + hI(o->volumes.size()));
        ModelObject *c = nullptr;
        o->clone_for_cut(&c);
        dump_object("cu3", *c);
        // (the clone is not freed: ~ModelObject is private)
        o->invalidate_cut();
        out("cu4 " + hB(o->is_cut()) + " " + hI(o->cut_id.check_sum()));
        std::array<Vec3d, 4> pts = {Vec3d(0, 0, 1), Vec3d(1, 0, 1.5), Vec3d(1, 1, 1.5), Vec3d(0, 1, 1)};
        out("svd_cpm " + hM(o->calculate_cut_plane_inverse_matrix(pts)));
        CutConnectorAttributes kinds[] = {{CutConnectorType::Plug, CutConnectorStyle::Prizm, CutConnectorShape::Square},
                                          {CutConnectorType::Plug, CutConnectorStyle::Frustum, CutConnectorShape::Triangle},
                                          {CutConnectorType::Dowel, CutConnectorStyle::Frustum, CutConnectorShape::Hexagon},
                                          {CutConnectorType::Snap, CutConnectorStyle::Prizm, CutConnectorShape::Circle}};
        for (int k = 0; k < 4; ++k) out("geo_conn " + hI(k) + " " + ITS(ModelObject::get_connector_mesh(kinds[k], CutConnectorParas())));
    }
    // unit conversion of an object into a new one, reusable objects
    {
        Model m = build_model_1();
        ModelObjectPtrs new_objects;
        m.objects[1]->convert_units(new_objects, ConversionType::CONV_FROM_INCH, {0, 3});
        dump_object("cv", *new_objects[0]);
        dump_object_geometry("cvg", *new_objects[0]);
        // (not freed: ~ModelObject is private)
        std::vector<ObjectBase *> reusable;
        m.collect_reusable_objects(reusable);
        std::string s = "re " + hI(reusable.size()) + " " + hI(m.objects.size());
        for (ObjectBase *ob : reusable) s += " " + ID(ob->id());
        out(s);
    }
    // assembly position
    {
        Model m = build_model_1();
        m.set_assembly_pos(m.objects[1]);
        out("svd_apos " + hM(m.objects[1]->instances[0]->get_assemble_transformation().get_matrix()) + " " +
            hM(m.objects[1]->instances[1]->get_assemble_transformation().get_matrix()));
    }
}

int main()
{
    // The engine logs through Boost.Log (qhull's "Unable to create convex hull" of a flat mesh).
    boost::log::core::get()->set_logging_enabled(false);
    scenario_prim();
    scenario_build();
    scenario_ops();
    scenario_ids();
    scenario_instance();
    scenario_extra();
    return 0;
}
