//! `slicer::model` (the port of libslic3r/Model.hpp and Model.cpp) against the C++ at the pin.
//!
//! `tests/data/model/model.txt` is the output of `gen.cpp` beside it, which links the pin's
//! Model.cpp and runs a fixed script of Model / ModelObject / ModelVolume / ModelInstance
//! operations on models built in code; the header of gen.cpp describes the lines. The same script
//! runs here on the port and every line has to be the same: doubles and floats bit for bit, object
//! IDs (relative to the first model of a scenario), mesh digests, polygons.
//!
//! Lines whose name starts with `svd_` depend on `Geometry::Transformation`'s decomposition and
//! rotation getters (Eigen's JacobiSVD / computeRotationScaling in C++, nalgebra's SVD in
//! `geometry/geometry.rs`); lines whose name starts with `geo_` are results of other units
//! (TriangleMesh.cpp mesh makers, BuildVolume's bed type). They are compared and their differences
//! reported, without failing: those ports are not this unit. Every other line has to be equal.
//!
//! The IDs come from the process-wide counter of `object_id.rs`, so this file holds one test.

use nalgebra::Matrix3;
use slicer::bounding_box::BoundingBoxf3;
use slicer::geometry::geometry::{
    extract_euler_angles_from_matrix, rotation_transform, Axis, Transform3d, Transformation, Vec2d, Vec3d,
};
use slicer::model::*;
use slicer::normal_utils::indexed_triangle_set;
use slicer::object_id::ObjectID;
use slicer::triangle_mesh::{its_make_cube, its_merge, TriangleMeshStats, Vec3f, Vec3i};
use slicer::Polygon;

type Matrix3d = Matrix3<f64>;

struct Out {
    lines: Vec<String>,
    base: usize,
}

impl Out {
    fn out(&mut self, s: String) {
        self.lines.push(s);
    }
    fn id(&self, id: ObjectID) -> String {
        (id.id as i64 - self.base as i64).to_string()
    }
}

fn hd(v: f64) -> String {
    format!("{:016x}", v.to_bits())
}
fn hf(v: f32) -> String {
    format!("{:08x}", v.to_bits())
}
fn hb(v: bool) -> String {
    (if v { "1" } else { "0" }).to_string()
}
fn ht(s: &str) -> String {
    if s.is_empty() {
        return "-".to_string();
    }
    s.bytes().map(|c| format!("{:02x}", c)).collect()
}
fn v3(v: &Vec3d) -> String {
    format!("{} {} {}", hd(v.x), hd(v.y), hd(v.z))
}
fn bbx(b: &BoundingBoxf3) -> String {
    format!("{} {} {} {} {} {} {}", hb(b.defined), hd(b.min.x), hd(b.min.y), hd(b.min.z), hd(b.max.x), hd(b.max.y), hd(b.max.z))
}
fn hm(t: &Transform3d) -> String {
    let mut o = Vec::new();
    for r in 0..3 {
        for c in 0..4 {
            o.push(hd(t[(r, c)]));
        }
    }
    o.join(" ")
}
fn m3(m: &Matrix3d) -> String {
    let mut o = Vec::new();
    for r in 0..3 {
        for c in 0..3 {
            o.push(hd(m[(r, c)]));
        }
    }
    o.join(" ")
}
fn st(s: &TriangleMeshStats) -> String {
    format!(
        "{} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {}",
        s.number_of_facets,
        hf(s.min.x),
        hf(s.min.y),
        hf(s.min.z),
        hf(s.max.x),
        hf(s.max.y),
        hf(s.max.z),
        hf(s.size.x),
        hf(s.size.y),
        hf(s.size.z),
        hf(s.volume),
        s.number_of_parts,
        s.open_edges,
        s.repaired_errors.degenerate_facets,
        s.repaired_errors.edges_fixed,
        s.repaired_errors.facets_removed,
        s.repaired_errors.facets_reversed,
        s.repaired_errors.backwards_edges
    )
}
fn its_digest(its: &indexed_triangle_set) -> String {
    let mut h: u64 = 1469598103934665603;
    let mut mix = |v: u32| {
        for k in 0..4 {
            h ^= ((v >> (8 * k)) & 0xff) as u64;
            h = h.wrapping_mul(1099511628211);
        }
    };
    for v in &its.vertices {
        for k in 0..3 {
            mix(v[k].to_bits());
        }
    }
    for f in &its.indices {
        for k in 0..3 {
            mix(f[k] as u32);
        }
    }
    format!("{} {} {:016x}", its.vertices.len(), its.indices.len(), h)
}
fn poly(p: &Polygon) -> String {
    let mut o = p.points.len().to_string();
    for pt in &p.points {
        o += &format!(" {} {}", pt.x, pt.y);
    }
    o
}

/// splitmix64, as gen.cpp
struct Rng {
    s: u64,
}
impl Rng {
    fn new(seed: u64) -> Self {
        Rng { s: seed }
    }
    fn next(&mut self) -> u64 {
        self.s = self.s.wrapping_add(0x9e3779b97f4a7c15);
        let mut z = self.s;
        z = (z ^ (z >> 30)).wrapping_mul(0xbf58476d1ce4e5b9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94d049bb133111eb);
        z ^ (z >> 31)
    }
    fn uni(&mut self) -> f64 {
        (self.next() >> 11) as f64 * (1.0 / 9007199254740992.0)
    }
    fn range(&mut self, a: f64, b: f64) -> f64 {
        a + (b - a) * self.uni()
    }
}

fn rand_affine(r: &mut Rng, allow_mirror: bool) -> Transform3d {
    let (mut w, mut x, mut y, mut z) = (r.range(-1.0, 1.0), r.range(-1.0, 1.0), r.range(-1.0, 1.0), r.range(-1.0, 1.0));
    let n = (w * w + x * x + y * y + z * z).sqrt();
    w /= n;
    x /= n;
    y /= n;
    z /= n;
    let rm = [
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ];
    let mut s = [r.range(0.5, 2.0), r.range(0.5, 2.0), r.range(0.5, 2.0)];
    let mi = (r.next() % 4) as usize;
    if allow_mirror && mi < 3 {
        s[mi] = -s[mi];
    }
    let mut t = Transform3d::identity();
    for i in 0..3 {
        for j in 0..3 {
            t[(i, j)] = rm[i][j] * s[j];
        }
    }
    for i in 0..3 {
        t[(i, 3)] = r.range(-50.0, 50.0);
    }
    t
}

fn z_affine(a: f64, sc: f64, tr: &Vec3d) -> Transform3d {
    let mut t = Transform3d::identity();
    let (c, s) = (a.cos(), a.sin());
    t[(0, 0)] = c * sc;
    t[(0, 1)] = -s * sc;
    t[(1, 0)] = s * sc;
    t[(1, 1)] = c * sc;
    t[(2, 2)] = sc;
    t[(0, 3)] = tr.x;
    t[(1, 3)] = tr.y;
    t[(2, 3)] = tr.z;
    t
}

fn tf(m: Transform3d) -> Transformation {
    Transformation::from_transform(m)
}

fn box_its(sx: f64, sy: f64, sz: f64, ox: f32, oy: f32, oz: f32) -> indexed_triangle_set {
    let mut its = its_make_cube(sx, sy, sz);
    for v in &mut its.vertices {
        *v += Vec3f::new(ox, oy, oz);
    }
    its
}

fn two_boxes_its() -> indexed_triangle_set {
    let mut a = box_its(2.0, 3.0, 4.0, 1.0, 0.5, 0.0);
    let b = box_its(1.5, 1.0, 2.0, 7.0, -2.0, 0.25);
    its_merge(&mut a, &b);
    a
}

fn flat_its() -> indexed_triangle_set {
    let mut its = indexed_triangle_set::default();
    its.vertices = vec![Vec3f::new(0.0, 0.0, 0.0), Vec3f::new(1.0, 0.0, 0.0), Vec3f::new(1.0, 1.0, 0.0), Vec3f::new(0.0, 1.0, 0.0)];
    its.indices = vec![Vec3i::new(0, 1, 2), Vec3i::new(0, 2, 3)];
    its
}

fn mesh(its: indexed_triangle_set) -> ModelMesh {
    ModelMesh::from_its(its)
}

fn dump_volume(o: &mut Out, tag: &str, v: &ModelVolume) {
    let s = format!(
        "{} ids {} {} {} {} {} {} {}",
        tag,
        o.id(v.id()),
        o.id(v.config.id()),
        o.id(v.supported_facets.id()),
        o.id(v.seam_facets.id()),
        o.id(v.mmu_segmentation_facets.id()),
        o.id(v.fuzzy_skin_facets.id()),
        o.id(v.exterior_facets.id())
    );
    o.out(s);
    o.out(format!("{} type {} {} {}", tag, v.type_() as i32, ht(&v.name), ht(&v.material_id())));
    o.out(format!("{} m {}", tag, hm(&v.matrix())));
    o.out(format!("{} stats {}", tag, st(v.mesh().stats())));
    o.out(format!("{} its {}", tag, its_digest(&v.mesh().its)));
    o.out(format!("{} mbb {} {} {}", tag, bbx(&v.mesh().bounding_box()), v3(&v.source.mesh_offset), v3(&v.mesh().get_init_shift())));
    if v.get_convex_hull_shared_ptr().is_some() {
        let h = v.get_convex_hull();
        o.out(format!("{} hull {} {}", tag, hb(h.empty()), bbx(&h.bounding_box())));
    } else {
        o.out(format!("{} hull none", tag));
    }
}

fn dump_object(o: &mut Out, tag: &str, ob: &ModelObject) {
    let s = format!(
        "{} ids {} {} {} {} {} {} {} {} {} {}",
        tag,
        o.id(ob.id()),
        o.id(ob.config.id()),
        o.id(ob.layer_height_profile.id()),
        ht(&ob.name),
        ht(&ob.input_file),
        ob.volumes.len(),
        ob.instances.len(),
        ob.config.size(),
        if ob.config.has("extruder") { ob.config.extruder() } else { -1 },
        v3(&ob.origin_translation)
    );
    o.out(s);
    for (i, v) in ob.volumes.iter().enumerate() {
        dump_volume(o, &format!("{} v{}", tag, i), v);
    }
    for (i, mi) in ob.instances.iter().enumerate() {
        let s = format!(
            "{} i{} {} {} {} {} {} {}",
            tag,
            i,
            o.id(mi.id()),
            hm(&mi.matrix()),
            hb(mi.printable),
            mi.print_volume_state as i32,
            hb(mi.is_printable(ob)),
            mi.get_labeled_id() as i64 - o.base as i64
        );
        o.out(s);
    }
}

fn dump_object_geometry(o: &mut Out, tag: &str, ob: &ModelObject) {
    o.out(format!("{} bb {}", tag, bbx(&ob.bounding_box())));
    o.out(format!("{} rmb {}", tag, bbx(&ob.raw_mesh_bounding_box())));
    o.out(format!("{} frb {}", tag, bbx(&ob.full_raw_mesh_bounding_box())));
    if !ob.instances.is_empty() {
        o.out(format!("{} rbb {}", tag, bbx(&ob.raw_bounding_box().unwrap())));
    }
    for i in 0..ob.instances.len() {
        o.out(format!("{} ibb {} {} {}", tag, i, bbx(&ob.instance_bounding_box(i, false)), bbx(&ob.instance_bounding_box(i, true))));
        o.out(format!("{} ibb2 {} {}", tag, i, bbx(&ob.instance_bounding_box_of(&ob.instances[i], false))));
        o.out(format!(
            "{} ichb {} {} {}",
            tag,
            i,
            bbx(&ob.instance_convex_hull_bounding_box(i, false)),
            bbx(&ob.instance_convex_hull_bounding_box(i, true))
        ));
        o.out(format!("{} iz {} {} {}", tag, i, hd(ob.get_instance_min_z(i)), hd(ob.get_instance_max_z(i))));
    }
    o.out(format!("{} z {} {}", tag, hd(ob.get_min_z()), hd(ob.get_max_z())));
    let s = ob.get_object_stl_stats();
    o.out(format!("{} stl {} {}", tag, st(&s), ob.get_repaired_errors_count(-1)));
    o.out(format!(
        "{} cnt {} {} {} {} {} {}",
        tag,
        ob.facets_count(),
        ob.parts_count(),
        ob.materials_count(),
        hb(ob.is_multiparts()),
        hb(ob.has_custom_layering()),
        hb(ob.is_cut())
    ));
    let me = ob.mesh();
    o.out(format!("{} mesh {} {}", tag, st(me.stats()), its_digest(&me.its)));
    let raw = ob.raw_mesh();
    o.out(format!("{} raw {} {}", tag, st(raw.stats()), its_digest(&raw.its)));
    o.out(format!("{} rits {}", tag, its_digest(&ob.raw_indexed_triangle_set())));
}

fn build_model_1(o: &mut Out) -> Model {
    let mut r = Rng::new(12345);
    let mut m = Model::new();
    o.base = m.id().id;
    {
        let ob = m.add_object_new();
        ob.name = "box".into();
        ob.input_file = "/tmp/in/box.stl".into();
        let v = ob.add_volume_mesh(mesh(box_its(10.0, 20.0, 30.0, 5.0, -3.0, 2.0)), ModelVolumeType::ModelPart, true);
        v.name = "box_v".into();
        v.set_transformation(&tf(rand_affine(&mut r, false)));
        let t = tf(rand_affine(&mut r, true));
        ob.add_instance().set_transformation(&t);
        let off = Vec3d::new(r.range(-80.0, 80.0), r.range(-80.0, 80.0), r.range(-5.0, 5.0));
        ob.add_instance().set_offset(&off);
    }
    {
        let ob = m.add_object_new();
        ob.name = "multi".into();
        let t0 = tf(rand_affine(&mut r, false));
        ob.add_volume_mesh(mesh(box_its(4.0, 4.0, 4.0, 0.0, 0.0, 0.0)), ModelVolumeType::ModelPart, true).set_transformation(&t0);
        let off = Vec3d::new(r.range(-3.0, 3.0), r.range(-3.0, 3.0), r.range(-3.0, 3.0));
        ob.add_volume_mesh(mesh(box_its(2.0, 3.0, 1.0, 1.0, 1.0, 1.0)), ModelVolumeType::ParameterModifier, true).set_offset(&off);
        ob.add_volume_mesh(mesh(box_its(1.0, 1.0, 8.0, -1.0, -1.0, 0.0)), ModelVolumeType::NegativeVolume, true)
            .set_transformation(&tf(z_affine(0.3, 1.25, &Vec3d::new(0.5, -0.25, 1.0))));
        let t3 = tf(rand_affine(&mut r, true));
        ob.add_volume_mesh(mesh(two_boxes_its()), ModelVolumeType::ModelPart, true).set_transformation(&t3);
        for k in 0..3 {
            if k == 2 {
                ob.add_instance().set_offset(&Vec3d::new(10.5, -20.25, 3.0));
            } else {
                let t = tf(rand_affine(&mut r, k == 1));
                ob.add_instance().set_transformation(&t);
            }
        }
        ob.config.set("extruder", 2);
    }
    {
        let me = mesh(box_its(3.0, 2.0, 1.0, -10.0, 4.0, 6.0));
        let ob = m.add_object_with_mesh("named", "/tmp/in/named.obj", me);
        ob.add_instance().set_transformation(&tf(z_affine(1.1, 0.75, &Vec3d::new(30.0, 40.0, 0.0))));
    }
    m
}

fn build_model_2(o: &mut Out) -> Model {
    let mut m = Model::new();
    o.base = m.id().id;
    let ob = m.add_object_new();
    ob.name = "two".into();
    ob.add_volume_mesh(mesh(two_boxes_its()), ModelVolumeType::ModelPart, true).name = "two_v".into();
    ob.add_instance().set_transformation(&tf(z_affine(0.5, 1.0, &Vec3d::new(20.0, 30.0, 5.0))));
    ob.add_instance().set_offset(&Vec3d::new(-15.0, 2.0, 0.0));
    m
}

/// A * B with plain loops, as gen.cpp's `mul_plain`.
fn mul_plain(a: &Transform3d, b: &Transform3d) -> Transform3d {
    let mut r = Transform3d::identity();
    for i in 0..3 {
        for j in 0..4 {
            let mut acc = 0.0;
            for k in 0..3 {
                acc += a[(i, k)] * b[(k, j)];
            }
            if j == 3 {
                acc += a[(i, 3)];
            }
            r[(i, j)] = acc;
        }
    }
    r
}

fn build_model_3(o: &mut Out) -> Model {
    let mut r = Rng::new(999);
    let mut m = Model::new();
    o.base = m.id().id;
    let ob = m.add_object_new();
    ob.name = "bake".into();
    let t = tf(rand_affine(&mut r, false));
    ob.add_volume_mesh(mesh(box_its(6.0, 4.0, 2.0, 0.0, 0.0, 0.0)), ModelVolumeType::ModelPart, true).set_transformation(&t);
    ob.add_volume_mesh(mesh(box_its(1.0, 2.0, 3.0, 2.0, 1.0, 0.0)), ModelVolumeType::ModelPart, true).set_offset(&Vec3d::new(1.5, -0.5, 0.25));
    let t0 = rand_affine(&mut r, false);
    ob.add_instance().set_transformation(&tf(t0));
    ob.add_instance().set_transformation(&tf(mul_plain(&z_affine(0.7, 1.0, &Vec3d::new(25.0, -5.0, 0.0)), &t0)));
    m
}

fn scenario_prim(o: &mut Out) {
    let mut r = Rng::new(777);
    for _ in 0..40 {
        let a = rand_affine(&mut r, true);
        let bm = rand_affine(&mut r, true);
        let v = Vec3d::new(r.range(-30.0, 30.0), r.range(-30.0, 30.0), r.range(-30.0, 30.0));
        o.out(format!("prim_mul {}", hm(&eigen_mul_affine(&a, &bm))));
        o.out(format!("prim_pt {}", v3(&eigen_transform_point(&a, &v))));
        let a3: Matrix3d = a.fixed_view::<3, 3>(0, 0).into_owned();
        let b3: Matrix3d = bm.fixed_view::<3, 3>(0, 0).into_owned();
        o.out(format!("prim_det {}", hd(eigen_determinant3(&a3))));
        o.out(format!("prim_inv {}", hm(&eigen_inverse_affine(&a))));
        o.out(format!("prim_m3v {}", v3(&eigen_matrix3_mul_vec(&a3, &v))));
        o.out(format!("prim_m3m {}", m3(&eigen_matrix3_mul(&a3, &b3))));
        let pmin = slicer::geometry::Vec3d::new(r.range(-10.0, 0.0), r.range(-10.0, 0.0), r.range(-10.0, 0.0));
        let pmax = slicer::geometry::Vec3d::new(r.range(0.0, 10.0), r.range(0.0, 10.0), r.range(0.0, 10.0));
        let bb = BoundingBoxf3::new_from_points(pmin, pmax);
        o.out(format!("prim_bbt {}", bbx(&eigen_bbox_transformed(&bb, &a))));
        let ang = r.range(-3.0, 3.0);
        let (ax, ay, az) = (r.range(-1.0, 1.0), r.range(-1.0, 1.0), r.range(-1.0, 1.0));
        let an = (ax * ax + ay * ay + az * az).sqrt();
        let axis = Vec3d::new(ax / an, ay / an, az / an);
        o.out(format!("prim_aa {}", m3(&eigen_angle_axis_matrix3(ang, &axis))));
        let q = eigen_angle_axis_rotation_matrix(ang, &axis);
        o.out(format!("prim_qaa {}", m3(&q)));
        let t = tf(a);
        o.out(format!("prim_mt {}", hm(&t.get_matrix(true, false, false, false))));
        o.out(format!("svd_rot {}", v3(&t.get_rotation())));
        o.out(format!("svd_scale {}", v3(&t.get_scaling_factor())));
        o.out(format!("svd_mirror {}", v3(&t.get_mirror())));
        o.out(format!("svd_mtfsm {}", hm(&t.get_matrix(true, false, true, true))));
        o.out(format!("svd_euler {}", v3(&extract_euler_angles_from_matrix(&q))));
        let rot = Vec3d::new(r.range(-3.0, 3.0), r.range(-3.0, 3.0), r.range(-3.0, 3.0));
        o.out(format!("svd_rott {}", hm(&rotation_transform(&rot))));
    }
}

fn scenario_build(o: &mut Out) {
    let mut m = build_model_1(o);
    o.out(format!("model {} {} {}", o.id(m.id()), o.id(m.wipe_tower.id()), m.objects.len()));
    for i in 0..m.objects.len() {
        dump_object(o, &format!("o{}", i), &m.objects[i]);
        dump_object_geometry(o, &format!("g{}", i), &m.objects[i]);
    }
    o.out(format!("mbb {}", bbx(&m.bounding_box())));
    let mm = m.mesh();
    o.out(format!("mmesh {} {}", st(mm.stats()), its_digest(&mm.its)));
    o.out(format!(
        "mflags {} {} {} {} {} {} {} {}",
        hb(m.looks_like_multipart_object()),
        hb(m.looks_like_imperial_units()),
        hb(m.looks_like_saved_in_meters()),
        hb(model_has_multi_part_objects(&m)),
        hb(model_has_advanced_features(&m)),
        hb(m.is_fdm_support_painted()),
        hb(m.is_seam_painted()),
        hb(m.is_mm_painted())
    ));
    o.out(format!("mname {} {}", ht(&m.propose_export_file_name_and_path()), ht(&m.propose_export_file_name_and_path_ext(".3mf"))));
    for i in 0..m.objects.len() {
        o.out(format!("oname {} {}", i, ht(&m.objects[i].get_export_filename())));
    }
    for i in 0..m.objects.len() {
        for j in 0..m.objects[i].instances.len() {
            let ob = &m.objects[i];
            o.out(format!("svd_ch2d {} {} {}", i, j, poly(&ob.instances[j].convex_hull_2d(ob))));
        }
    }
    {
        let o1 = &mut m.objects[1];
        o1.volumes[0].config.set("wall_filament", 3);
        o1.volumes[1].config.set("extruder", 4);
        o1.config.set("sparse_infill_filament", 5);
    }
    let o1 = &m.objects[1];
    for j in 0..o1.volumes.len() {
        let mut gf = GlobalFeatureFilaments::default();
        gf.solid_infill_filament = 6;
        let e = o1.volumes[j].get_extruders(o1, &gf);
        let mut s = format!("ext {} {} {}", j, o1.volumes[j].extruder_id(o1), e.len());
        for x in &e {
            s += &format!(" {}", x);
        }
        o.out(format!("{} {} {}", s, hb(o1.volumes[j].is_splittable()), hb(o1.volumes[j].is_the_only_one_part(Some(o1)))));
    }
    o.out(format!(
        "ext2 {} {} {}",
        m.objects[0].volumes[0].extruder_id(&m.objects[0]),
        m.objects[2].volumes[0].extruder_id(&m.objects[2]),
        hb(m.objects[0].volumes[0].is_the_only_one_part(Some(&m.objects[0])))
    ));
    o.out(format!("mflags2 {}", hb(model_has_advanced_features(&m))));
}

fn scenario_ops(o: &mut Out) {
    // translate with a cached bounding box: m_bounding_box translated, raw mesh box stale
    {
        let mut m = build_model_1(o);
        let ob = &mut m.objects[0];
        o.out(format!("t0 {} {}", bbx(&ob.bounding_box()), bbx(&ob.raw_mesh_bounding_box())));
        ob.translate(1.5, -2.25, 0.125);
        o.out(format!("t1 {} {} {}", bbx(&ob.bounding_box()), bbx(&ob.raw_mesh_bounding_box()), hm(&ob.volumes[0].matrix())));
        ob.invalidate_bounding_box();
        o.out(format!("t2 {} {}", bbx(&ob.bounding_box()), bbx(&ob.raw_mesh_bounding_box())));
        m.translate(3.0, 4.0, 5.0);
        o.out(format!("t3 {}", bbx(&m.bounding_box())));
    }
    // center_around_origin, translate_instances, ensure_on_bed, adjust_min_z
    {
        let mut m = build_model_1(o);
        {
            let ob = &mut m.objects[1];
            ob.center_around_origin(true);
            o.out(format!("c0 {} {}", v3(&ob.origin_translation), bbx(&ob.full_raw_mesh_bounding_box())));
            ob.center_around_origin(false);
            o.out(format!("c1 {} {}", v3(&ob.origin_translation), bbx(&ob.raw_mesh_bounding_box())));
            ob.translate_instances(&Vec3d::new(1.0, 2.0, -40.0));
            ob.translate_instance(1, &Vec3d::new(-0.5, 0.25, 0.75));
            o.out(format!("c2 {} {}", bbx(&ob.bounding_box()), hd(ob.get_min_z())));
            ob.ensure_on_bed(false);
            o.out(format!("c3 {} {}", hd(ob.get_min_z()), hm(&ob.instances[0].matrix())));
            ob.translate_instances(&Vec3d::new(0.0, 0.0, -0.0005));
            ob.ensure_on_bed(true);
            o.out(format!("c4 {} {}", hd(ob.get_min_z()), hd(ob.get_max_z())));
        }
        m.objects[0].translate_instances(&Vec3d::new(0.0, 0.0, -100.0));
        m.adjust_min_z();
        o.out(format!("c5 {} {}", bbx(&m.bounding_box()), bbx(&m.objects[0].bounding_box())));
        let moved = m.center_instances_around_point(&Vec2d::new(128.0, 96.0));
        let bb = m.bounding_box();
        let again = m.center_instances_around_point(&Vec2d::new(128.0, 96.0));
        o.out(format!("c6 {} {} {}", hb(moved), bbx(&bb), hb(again)));
    }
    // delete_volume collapses the last volume's transformation into the instances
    {
        let mut m = build_model_1(o);
        {
            let ob = &mut m.objects[1];
            ob.delete_volume(1);
            ob.delete_volume(1);
            o.out(format!("d0 {}", ob.volumes.len()));
            ob.delete_volume(0);
        }
        dump_object(o, "d1", &m.objects[1]);
        dump_object_geometry(o, "d1g", &m.objects[1]);
        {
            let ob = &mut m.objects[1];
            ob.delete_instance(0);
            ob.delete_last_instance();
            o.out(format!("d2 {} {}", ob.instances.len(), bbx(&ob.bounding_box())));
            ob.clear_instances();
        }
        m.add_default_instances();
        let ob = &m.objects[1];
        o.out(format!("d3 {} {} {}", ob.instances.len(), o.id(ob.instances[0].id()), bbx(&ob.bounding_box())));
    }
    // sort_volumes
    {
        let mut m = build_model_1(o);
        let ob = &mut m.objects[1];
        ob.volumes[0].set_type(ModelVolumeType::SupportEnforcer);
        ob.sort_volumes(false);
        let mut s = "s0".to_string();
        for v in &ob.volumes {
            s += &format!(" {} {}", o.id(v.id()), v.type_() as i32);
        }
        o.out(s);
        ob.sort_volumes(true);
        let mut s = "s1".to_string();
        for v in &ob.volumes {
            s += &format!(" {} {}", o.id(v.id()), v.type_() as i32);
        }
        o.out(s);
    }
    // units
    {
        let mut m = build_model_1(o);
        let mut small = Model::new();
        o.base = m.id().id;
        {
            let so = small.add_object_new();
            so.add_volume_mesh(mesh(box_its(0.1, 0.1, 0.1, 0.0, 0.0, 0.0)), ModelVolumeType::ModelPart, true);
            so.add_instance();
        }
        o.out(format!(
            "u0 {} {} {}",
            hb(small.looks_like_imperial_units()),
            hb(small.looks_like_saved_in_meters()),
            st(&small.objects[0].get_object_stl_stats())
        ));
        small.convert_from_meters(true);
        dump_object(o, "u1", &small.objects[0]);
        o.out(format!("u1g {} {}", bbx(&small.objects[0].bounding_box()), st(&small.objects[0].get_object_stl_stats())));
        small.convert_from_imperial_units(false);
        dump_object(o, "u2", &small.objects[0]);
        m.objects[0].scale_mesh_after_creation(2.5);
        dump_object(o, "u3", &m.objects[0]);
        dump_object_geometry(o, "u3g", &m.objects[0]);
        {
            let fo = m.add_object_new();
            fo.add_volume_mesh(mesh(flat_its()), ModelVolumeType::ModelPart, true);
            fo.add_instance();
        }
        let n = m.objects.len();
        let fo = &m.objects[n - 1];
        o.out(format!("u4 {} {} {}", st(&fo.get_object_stl_stats()), hb(fo.volumes[0].get_convex_hull().empty()), n));
        let removed = m.removed_objects_with_zero_volume();
        o.out(format!("u5 {} {}", removed, m.objects.len()));
    }
    // ModelVolume conversions and mesh transforms
    {
        let mut m = build_model_1(o);
        m.objects[0].volumes[0].convert_from_imperial_units();
        dump_volume(o, "vc0", &m.objects[0].volumes[0]);
        m.objects[1].volumes[3].convert_from_meters();
        dump_volume(o, "vc1", &m.objects[1].volumes[3]);
    }
    // split and merge
    {
        let mut m = build_model_2(o);
        let mut new_objects = Vec::new();
        m.objects[0].split(&mut new_objects);
        let n = new_objects.len();
        m.objects.extend(new_objects);
        o.out(format!("sp0 {} {}", n, m.objects.len()));
        for i in 0..n {
            dump_object(o, &format!("sp{}", i), &m.objects[1 + i]);
            dump_object_geometry(o, &format!("spg{}", i), &m.objects[1 + i]);
        }
    }
    {
        let mut m = build_model_1(o);
        let mut new_objects = Vec::new();
        m.objects[1].split(&mut new_objects);
        let n = new_objects.len();
        let first = m.objects.len();
        m.objects.extend(new_objects);
        o.out(format!("sq0 {}", n));
        for i in 0..n {
            dump_object(o, &format!("sq{}", i), &m.objects[first + i]);
        }
        m.objects[1].merge();
        dump_object(o, "mg", &m.objects[1]);
        dump_object_geometry(o, "mgg", &m.objects[1]);
    }
    {
        let mut m = build_model_2(o);
        let n = ModelVolume::split(&mut m.objects[0], 0, 1, 1.0);
        o.out(format!("vs0 {} {}", n, m.objects[0].volumes.len()));
        for i in 0..m.objects[0].volumes.len() {
            dump_volume(o, &format!("vs{}", i), &m.objects[0].volumes[i]);
        }
        dump_object_geometry(o, "vsg", &m.objects[0]);
    }
    // convert_multipart_object
    {
        let mut m = build_model_1(o);
        m.delete_object(1);
        o.out(format!("mp0 {}", hb(m.looks_like_multipart_object())));
        m.convert_multipart_object(4);
        dump_object(o, "mp1", &m.objects[0]);
        dump_object_geometry(o, "mp1g", &m.objects[0]);
    }
    // duplicate_objects_grid
    {
        let mut m = build_model_2(o);
        m.duplicate_objects_grid(2, 3, 5.0).unwrap();
        dump_object(o, "dg", &m.objects[0]);
        o.out(format!("dgb {}", bbx(&m.bounding_box())));
    }
}

fn scenario_ids(o: &mut Out) {
    let mut m = build_model_1(o);
    let mut copy = m.clone();
    o.out(format!(
        "cp0 {} {} {} {}",
        o.id(copy.id()),
        o.id(copy.wipe_tower.id()),
        hb(model_object_list_equal(&m, &copy)),
        hb(model_object_list_extended(&m, &copy))
    ));
    dump_object(o, "cp1", &copy.objects[1]);
    copy.add_object_clone(&m.objects[1]);
    o.out(format!("cp2 {} {}", hb(model_object_list_equal(&m, &copy)), hb(model_object_list_extended(&m, &copy))));
    let last = copy.objects.len() - 1;
    dump_object(o, "cp3", &copy.objects[last]);
    dump_object_geometry(o, "cp3g", &copy.objects[last]);
    // volume list changes against the copy
    {
        let a = &m.objects[1];
        let b = &copy.objects[1];
        o.out(format!(
            "vl0 {} {} {} {} {} {} {}",
            hb(model_volume_list_changed(a, b, ModelVolumeType::ModelPart)),
            hb(model_volume_list_changed_types(a, b, &[ModelVolumeType::ParameterModifier, ModelVolumeType::NegativeVolume])),
            hb(model_custom_supports_data_changed(a, b)),
            hb(model_custom_seam_data_changed(a, b)),
            hb(model_mmu_segmentation_data_changed(a, b)),
            hb(model_custom_fuzzy_skin_data_changed(a, b)),
            hb(model_brim_points_data_changed(a, b))
        ));
    }
    copy.objects[1].volumes[0].supported_facets.reset();
    copy.objects[1].volumes[3].mmu_segmentation_facets.reset();
    {
        let a = &m.objects[1];
        let b = &copy.objects[1];
        o.out(format!(
            "vl1 {} {} {}",
            hb(model_custom_supports_data_changed(a, b)),
            hb(model_mmu_segmentation_data_changed(a, b)),
            hb(model_custom_seam_data_changed(a, b))
        ));
    }
    copy.objects[1].volumes[1].set_offset(&Vec3d::new(9.0, 9.0, 9.0));
    o.out(format!(
        "vl2 {} {}",
        hb(model_volume_list_changed(&m.objects[1], &copy.objects[1], ModelVolumeType::ModelPart)),
        hb(model_volume_list_changed(&m.objects[1], &copy.objects[1], ModelVolumeType::ParameterModifier))
    ));
    copy.objects[1].volumes[0].set_new_unique_id();
    o.out(format!(
        "vl3 {} {} {} {}",
        hb(model_volume_list_changed(&m.objects[1], &copy.objects[1], ModelVolumeType::ModelPart)),
        hb(model_volume_list_changed(&m.objects[1], &copy.objects[1], ModelVolumeType::NegativeVolume)),
        o.id(copy.objects[1].volumes[0].id()),
        o.id(copy.objects[1].volumes[0].config.id())
    ));
    copy.objects[1].delete_volume(2);
    o.out(format!(
        "vl4 {} {}",
        hb(model_volume_list_changed(&m.objects[1], &copy.objects[1], ModelVolumeType::NegativeVolume)),
        hb(model_volume_list_changed(&m.objects[1], &copy.objects[1], ModelVolumeType::ParameterModifier))
    ));
    // materials
    m.add_material("pla");
    let (mid, mcid) = {
        let mat = m.add_material("petg");
        (mat.id(), mat.config.id())
    };
    o.out(format!("mat0 {} {} {}", m.materials.len(), o.id(mid), o.id(mcid)));
    {
        let Model { objects, materials, .. } = &mut m;
        objects[0].volumes[0].set_material_id("pla".into(), materials);
        objects[1].volumes[0].set_material_id("abs".into(), materials);
    }
    o.out(format!("mat1 {} {} {}", m.materials.len(), m.objects[1].materials_count(), o.id(m.materials["abs"].id())));
    m.delete_material("pla");
    m.clear_materials();
    o.out(format!("mat2 {}", m.materials.len()));
    // delete objects
    let id1 = m.objects[1].id();
    let d0 = m.delete_object_by_id(id1);
    let d1 = m.delete_object_by_id(id1);
    o.out(format!("del0 {} {} {}", hb(d0), hb(d1), m.objects.len()));
    m.clear_objects();
    o.out(format!("del1 {}", m.objects.len()));
}

fn scenario_instance(o: &mut Out) {
    let mut m = build_model_1(o);
    let mut r = Rng::new(4242);
    {
        let ob = &m.objects[1];
        let mi = &ob.instances[0];
        for _ in 0..6 {
            let v = Vec3d::new(r.range(-20.0, 20.0), r.range(-20.0, 20.0), r.range(-20.0, 20.0));
            o.out(format!("tv {} {}", v3(&mi.transform_vector(&v, false)), v3(&mi.transform_vector(&v, true))));
            let pmin = slicer::geometry::Vec3d::new(r.range(-10.0, 0.0), r.range(-10.0, 0.0), r.range(-10.0, 0.0));
            let pmax = slicer::geometry::Vec3d::new(r.range(0.0, 10.0), r.range(0.0, 10.0), r.range(0.0, 10.0));
            let bb = BoundingBoxf3::new_from_points(pmin, pmax);
            o.out(format!("tb {} {}", bbx(&mi.transform_bounding_box(&bb, false)), bbx(&mi.transform_bounding_box(&bb, true))));
        }
        o.out(format!(
            "svd_tmb {} {}",
            bbx(&mi.transform_mesh_bounding_box(ob.volumes[0].mesh(), false)),
            bbx(&mi.transform_mesh_bounding_box(ob.volumes[0].mesh(), true))
        ));
        let mut tm = ob.volumes[3].mesh().clone();
        mi.transform_mesh(&mut tm, false);
        o.out(format!("tm {} {}", st(tm.stats()), its_digest(&tm.its)));
        let mut p = Polygon::from_points(vec![
            slicer::Point::new(1000000, 0),
            slicer::Point::new(0, 2000000),
            slicer::Point::new(-1500000, -500000),
        ]);
        mi.transform_polygon(&mut p);
        o.out(format!("svd_tp {}", poly(&p)));
        o.out(format!("svd_asm {} {}", hm(&mi.get_assemble_transformation().matrix()), hb(mi.is_assemble_initialized())));
    }
    m.objects[1].instances[0].set_assemble_offset(&Vec3d::new(1.0, 2.0, 3.0));
    {
        let mi = &m.objects[1].instances[0];
        o.out(format!("svd_asm2 {} {}", hm(&mi.get_assemble_transformation().matrix()), hb(mi.is_assemble_initialized())));
    }
    o.out(format!("svd_bbav {} {}", bbx(&m.objects[1].bounding_box_in_assembly_view()), bbx(&m.bounding_box_in_assembly_view(Some(1)))));
    ModelInstance::apply_arrange_result(&mut m.objects[1], 2, &Vec2d::new(12345678.0, -2345678.0), 0.75);
    o.out(format!("svd_arr {} {}", hm(&m.objects[1].instances[2].matrix()), bbx(&m.objects[1].bounding_box())));
    let rz = eigen_angle_axis_matrix3(0.5, &Vec3d::new(0.0, 0.0, 1.0));
    m.objects[1].instances[2].rotate(&rz);
    o.out(format!("svd_irot {}", hm(&m.objects[1].instances[2].matrix())));
    m.objects[1].instances[2].printable = false;
    {
        let ob = &m.objects[1];
        o.out(format!(
            "prn {} {} {}",
            hb(ob.instances[2].is_printable(ob)),
            hb(ob.instances[0].is_printable(ob)),
            ht(&m.propose_export_file_name_and_path())
        ));
    }
    m.objects[1].printable = false;
    o.out(format!(
        "prn2 {} {}",
        hb(m.objects[1].instances[0].is_printable(&m.objects[1])),
        ht(&m.propose_export_file_name_and_path())
    ));
    {
        let m1 = &mut m.objects[1].instances[1];
        m1.set_rotation(&Vec3d::new(0.1, -0.2, 0.3));
        o.out(format!("svd_set0 {}", hm(&m1.matrix())));
        m1.set_scaling_factor(&Vec3d::new(1.5, 0.75, 2.0));
        o.out(format!("svd_set1 {}", hm(&m1.matrix())));
        m1.set_mirror(&Vec3d::new(-1.0, 1.0, 1.0));
        o.out(format!(
            "svd_set2 {} {} {} {}",
            hm(&m1.matrix()),
            v3(&m1.get_rotation()),
            v3(&m1.get_scaling_factor()),
            v3(&m1.get_mirror())
        ));
    }
    {
        let ob = &mut m.objects[1];
        ob.scale(&Vec3d::new(1.25, 0.5, 2.0));
        o.out(format!("svd_os {} {}", hm(&ob.volumes[0].matrix()), bbx(&ob.bounding_box())));
        ob.mirror(Axis::Y);
        o.out(format!("svd_om {} {}", hm(&ob.volumes[0].matrix()), bbx(&ob.bounding_box())));
        ob.rotate_axis(0.4, Axis::X);
        o.out(format!("svd_or {} {}", hm(&ob.volumes[0].matrix()), v3(&ob.origin_translation)));
        ob.rotate(-0.7, &Vec3d::new(0.0, 0.6, 0.8));
        o.out(format!(
            "svd_or2 {} {} {}",
            hm(&ob.volumes[0].matrix()),
            v3(&ob.origin_translation),
            hm(&ob.instances[0].get_assemble_transformation().matrix())
        ));
        ob.scale_to_fit(&Vec3d::new(50.0, 60.0, 70.0));
        o.out(format!("svd_osf {} {}", hm(&ob.volumes[0].matrix()), bbx(&ob.bounding_box())));
    }
    let mut m3m = build_model_3(o);
    m3m.objects[0].bake_xy_rotation_into_meshes(0);
    dump_object(o, "svd_bake", &m3m.objects[0]);
}


fn facets_dump(v: &ModelVolume, f: &FacetsAnnotation) -> String {
    let n = v.mesh().its.indices.len();
    let mut o = n.to_string();
    for i in 0..n {
        let t = f.get_triangle_as_string(i as i32);
        if !t.is_empty() {
            o += &format!(" {}:{}", i, t);
        }
    }
    o
}

fn scenario_extra(o: &mut Out) {
    use slicer::config::{ConfigOptionBool, ConfigOptionFloats, ConfigOptionFloatsNullable, ConfigOptionInt, ConfigOptionIntsNullable, ConfigOptionStrings};
    use slicer::print_config::config_def::DynamicPrintConfig;
    use slicer::triangle_selector::EnforcerBlockerType;
    use std::collections::BTreeMap;
    // painted volume: extruders, facets per state, clamping
    {
        let mut m = build_model_2(o);
        {
            let v = &mut m.objects[0].volumes[0];
            v.mmu_segmentation_facets.set_triangle_from_string(0, "4");
            v.mmu_segmentation_facets.set_triangle_from_string(3, "8");
            v.mmu_segmentation_facets.set_triangle_from_string(5, "1C");
            v.mmu_segmentation_facets.set_triangle_from_string(14, "8");
            v.supported_facets.set_triangle_from_string(2, "4");
        }
        let mesh = m.objects[0].volumes[0].get_mesh_shared_ptr().unwrap().clone();
        {
            let ob = &m.objects[0];
            let v = &ob.volumes[0];
            let e = v.get_extruders(ob, &GlobalFeatureFilaments::default());
            let mut s = format!("pe0 {}", e.len());
            for x in &e {
                s += &format!(" {}", x);
            }
            o.out(format!(
                "{} {} {}",
                s,
                hb(v.mmu_segmentation_facets.has_facets(EnforcerBlockerType::EXTRUDER2)),
                hb(v.mmu_segmentation_facets.has_facets(EnforcerBlockerType(7)))
            ));
            for st in 0..5 {
                o.out(format!(
                    "pf {} {} {}",
                    st,
                    its_digest(&v.mmu_segmentation_facets.get_facets(&mesh, EnforcerBlockerType(st as i8))),
                    its_digest(&v.mmu_segmentation_facets.get_facets_strict(&mesh, EnforcerBlockerType(st as i8)))
                ));
            }
            let mut per_type = Vec::new();
            v.mmu_segmentation_facets.get_facets_per_type(&mesh, &mut per_type);
            let mut s = format!("pft {}", per_type.len());
            for its in &per_type {
                s += &format!(" {}", its.indices.len());
            }
            o.out(s);
        }
        let changed = m.objects[0].volumes[0].mmu_segmentation_facets.replace_state(&mesh, EnforcerBlockerType::EXTRUDER2, EnforcerBlockerType(3));
        o.out(format!("pr {} {}", hb(changed), facets_dump(&m.objects[0].volumes[0], &m.objects[0].volumes[0].mmu_segmentation_facets)));
        m.objects[0].volumes[0].mmu_segmentation_facets.shift_states_above(&mesh, EnforcerBlockerType(2), 1);
        o.out(format!("ps {}", facets_dump(&m.objects[0].volumes[0], &m.objects[0].volumes[0].mmu_segmentation_facets)));
        {
            let ob = &m.objects[0];
            let e = ob.volumes[0].get_extruders(ob, &GlobalFeatureFilaments::default());
            let mut s = format!("pe1 {}", e.len());
            for x in &e {
                s += &format!(" {}", x);
            }
            o.out(s);
        }
        m.objects[0].volumes[0].config.set("extruder", 6);
        ModelVolume::update_extruder_count(&mut m.objects[0], 0, 4);
        o.out(format!(
            "pu {} {}",
            facets_dump(&m.objects[0].volumes[0], &m.objects[0].volumes[0].mmu_segmentation_facets),
            hb(m.objects[0].volumes[0].config.has("extruder"))
        ));
        m.objects[0].volumes[0].config.set("extruder", 3);
        ModelVolume::update_extruder_count_when_delete_filament(&mut m.objects[0], 0, 4, 2, -1, &[]);
        o.out(format!(
            "pd {} {} {}",
            facets_dump(&m.objects[0].volumes[0], &m.objects[0].volumes[0].mmu_segmentation_facets),
            m.objects[0].volumes[0].extruder_id(&m.objects[0]),
            hb(m.objects[0].volumes[0].config.has("extruder"))
        ));
        m.objects[0].volumes[0].reset_extra_facets();
        o.out(format!(
            "px {} {} {} {}",
            hb(m.objects[0].volumes[0].is_mm_painted()),
            hb(m.objects[0].volumes[0].is_fdm_support_painted()),
            hb(m.objects[0].is_mm_painted()),
            hb(m.is_mm_painted())
        ));
    }
    // extruder parameters, brim width, arrange polygon
    {
        let mut m = build_model_1(o);
        let ep = |name: &str, bed: i32, end: f64| ExtruderParams { material_name: name.to_string(), bed_temp: bed, heat_end_temp: end };
        let mut params: BTreeMap<usize, ExtruderParams> = BTreeMap::new();
        params.insert(0, ep("PLA", 0, 210.0));
        params.insert(1, ep("PLA", 0, 210.0));
        params.insert(2, ep("PETG", 60, 240.0));
        params.insert(4, ep("TPU", 45, 220.0));
        let mut speed = GlobalSpeedMap::default();
        for i in 0..m.objects.len() {
            let ob = &m.objects[i];
            o.out(format!(
                "ad {} {} {} {} {}",
                i,
                hd(getadhesion_coeff(ob, &params)),
                hd(get_temperature_from_extruder(ob, &params)),
                hd(Model::get_thermal_length_volumes(ob, &params)),
                hd(Model::find_max_speed(ob, &speed))
            ));
        }
        speed.max_speed = 180.0;
        speed.perimeter_speed = 150.0;
        speed.external_perimeter_speed = 120.0;
        m.objects[1].config.set_key_value("inner_wall_speed", Box::new(ConfigOptionFloatsNullable::from_values(vec![90.0])));
        m.objects[1].config.set_key_value("support_speed", Box::new(ConfigOptionFloatsNullable::from_values(vec![130.0])));
        for i in 0..m.objects.len() {
            let ob = &m.objects[i];
            o.out(format!(
                "br {} {} {} {}",
                i,
                hd(Model::find_max_speed(ob, &speed)),
                hd(ob.instances[0].get_auto_brim_width_with(ob, 10.0, 1.5, &speed, &params)),
                hd(ob.instances[0].get_auto_brim_width())
            ));
        }
        let mut cfg = DynamicPrintConfig::new();
        cfg.set_key_value("filament_type", Box::new(ConfigOptionStrings::from_values(vec!["PLA".into(), "PETG".into(), "TPU".into()])));
        cfg.set_key_value("nozzle_temperature", Box::new(ConfigOptionIntsNullable::from_values(vec![210, 240, 225])));
        let params = Model::set_extruder_params(&cfg, 3);
        let mut s = format!("xp {}", params.len());
        for (k, v) in &params {
            s += &format!(" {} {} {} {}", k, ht(&v.material_name), v.bed_temp, hd(v.heat_end_temp));
        }
        o.out(s);
        cfg.set_key_value("enable_support", Box::new(ConfigOptionBool::with_value(true)));
        cfg.set_key_value("support_filament", Box::new(ConfigOptionInt::with_value(3)));
        cfg.set_key_value("support_interface_filament", Box::new(ConfigOptionInt::with_value(0)));
        cfg.set_key_value("filament_diameter", Box::new(ConfigOptionFloats::from_values(vec![1.75, 1.75, 1.75])));
        cfg.set_key_value("wall_filament", Box::new(ConfigOptionInt::with_value(2)));
        for i in 0..m.objects.len() {
            for j in 0..m.objects[i].instances.len() {
                let ob = &m.objects[i];
                let mut ap = slicer::model_arrange::ArrangePolygon::default();
                ob.instances[j].get_arrange_polygon(ob, &mut ap, &cfg, &params);
                let mut s = format!(
                    "svd_ap {} {} {} {} {} {} {}",
                    i,
                    j,
                    poly(&ap.poly.contour),
                    ap.translation.x,
                    ap.translation.y,
                    hd(ap.rotation),
                    ap.extrude_id_filament_types.len()
                );
                for (k, v) in &ap.extrude_id_filament_types {
                    s += &format!(" {} {}", k, ht(v));
                }
                o.out(s);
            }
        }
    }
    // print volume state
    {
        use slicer::geometry::Vec2d as BvVec2d;
        let mut m = build_model_1(o);
        let bv = slicer::build_volume::BuildVolume::new_from_config(
            vec![BvVec2d::new(-60.0, -60.0), BvVec2d::new(60.0, -60.0), BvVec2d::new(60.0, 60.0), BvVec2d::new(-60.0, 60.0)],
            80.0,
            vec![],
            vec![],
        );
        let n = m.update_print_volume_state(&bv);
        let mut s = format!("pv {}", n);
        for ob in &m.objects {
            for mi in &ob.instances {
                s += &format!(" {} {}", mi.print_volume_state as i32, hb(mi.is_printable(ob)));
            }
        }
        o.out(s);
        let bc = slicer::build_volume::BuildVolume::new_from_config(
            vec![
                BvVec2d::new(0.0, -100.0),
                BvVec2d::new(70.7, -70.7),
                BvVec2d::new(100.0, 0.0),
                BvVec2d::new(70.7, 70.7),
                BvVec2d::new(0.0, 100.0),
                BvVec2d::new(-70.7, 70.7),
                BvVec2d::new(-100.0, 0.0),
                BvVec2d::new(-70.7, -70.7),
            ],
            100.0,
            vec![],
            vec![],
        );
        let n = m.update_print_volume_state(&bc);
        o.out(format!("geo_bvtype {}", bc.volume_type() as i32));
        let mut s = format!("pvc {}", n);
        for ob in &m.objects {
            for mi in &ob.instances {
                s += &format!(" {}", mi.print_volume_state as i32);
            }
        }
        o.out(s);
    }
    // cut bookkeeping and connectors
    {
        let mut m = build_model_1(o);
        let ob = &mut m.objects[0];
        let attrs = ModelObjectCutAttributes::default()
            .with(ModelObjectCutAttribute::KeepUpper)
            .with(ModelObjectCutAttribute::KeepLower)
            .with(ModelObjectCutAttribute::CreateDowels);
        ob.apply_cut_attributes(attrs);
        o.out(format!("cu0 {} {} {} {}", hb(ob.is_cut()), o.id(ob.cut_id.id()), ob.cut_id.check_sum(), ob.cut_id.connectors_cnt()));
        ob.cut_connectors.push(CutConnector::new(
            Vec3d::new(1.0, 2.0, 3.0),
            Transform3d::identity(),
            2.5,
            4.0,
            0.1,
            0.2,
            CutConnectorAttributes::with(CutConnectorType::Plug, CutConnectorStyle::Prizm, CutConnectorShape::Square),
        ));
        ob.cut_connectors.push(CutConnector::new(
            Vec3d::new(-1.0, 0.0, 2.0),
            z_affine(0.25, 1.0, &Vec3d::zeros()),
            1.5,
            3.0,
            0.0,
            0.1,
            CutConnectorAttributes::with(CutConnectorType::Plug, CutConnectorStyle::Frustum, CutConnectorShape::Triangle),
        ));
        ob.apply_cut_connectors("conn");
        o.out(format!("cu1 {} {} {}", hb(ob.has_connectors()), ob.cut_id.connectors_cnt(), ob.cut_connectors.len()));
        for i in 1..ob.volumes.len() {
            dump_volume(o, &format!("cuv{}", i), &ob.volumes[i]);
            let v = &ob.volumes[i];
            let ci = &v.cut_info;
            o.out(format!(
                "cuc {} {} {} {} {} {}",
                hb(ci.is_connector),
                hb(ci.is_processed),
                ci.connector_type as i32,
                hf(ci.radius),
                hf(ci.height),
                hb(v.is_cut_connector())
            ));
        }
        ob.volumes[1].cut_info.set_processed();
        ob.delete_connectors();
        o.out(format!("cu2 {}", ob.volumes.len()));
        let c = ob.clone_for_cut();
        dump_object(o, "cu3", &c);
        let ob = &mut m.objects[0];
        ob.invalidate_cut();
        o.out(format!("cu4 {} {}", hb(ob.is_cut()), ob.cut_id.check_sum()));
        let pts = [Vec3d::new(0.0, 0.0, 1.0), Vec3d::new(1.0, 0.0, 1.5), Vec3d::new(1.0, 1.0, 1.5), Vec3d::new(0.0, 1.0, 1.0)];
        o.out(format!("svd_cpm {}", hm(&ob.calculate_cut_plane_inverse_matrix(&pts))));
        let kinds = [
            CutConnectorAttributes::with(CutConnectorType::Plug, CutConnectorStyle::Prizm, CutConnectorShape::Square),
            CutConnectorAttributes::with(CutConnectorType::Plug, CutConnectorStyle::Frustum, CutConnectorShape::Triangle),
            CutConnectorAttributes::with(CutConnectorType::Dowel, CutConnectorStyle::Frustum, CutConnectorShape::Hexagon),
            CutConnectorAttributes::with(CutConnectorType::Snap, CutConnectorStyle::Prizm, CutConnectorShape::Circle),
        ];
        for (k, kind) in kinds.iter().enumerate() {
            o.out(format!("geo_conn {} {}", k, its_digest(&ModelObject::get_connector_mesh(*kind, CutConnectorParas::default()))));
        }
    }
    // unit conversion of an object into a new one, reusable objects
    {
        let mut m = build_model_1(o);
        let mut new_objects = Vec::new();
        m.objects[1].convert_units(&mut new_objects, ConversionType::ConvFromInch, &[0, 3]);
        dump_object(o, "cv", &new_objects[0]);
        dump_object_geometry(o, "cvg", &new_objects[0]);
        let mut reusable = Vec::new();
        m.collect_reusable_objects(&mut reusable);
        let mut s = format!("re {} {}", reusable.len(), m.objects.len());
        for id in &reusable {
            s += &format!(" {}", o.id(*id));
        }
        o.out(s);
    }
    // assembly position
    {
        let mut m = build_model_1(o);
        m.set_assembly_pos(1);
        o.out(format!(
            "svd_apos {} {}",
            hm(&m.objects[1].instances[0].get_assemble_transformation().matrix()),
            hm(&m.objects[1].instances[1].get_assemble_transformation().matrix())
        ));
    }
}

fn read_vectors() -> String {
    let path = format!("{}/tests/data/model/model.txt", env!("CARGO_MANIFEST_DIR"));
    std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{path}: {e}"))
}

/// The number of differing tokens of two lines and the largest distance in units of the last
/// place of the 16-digit doubles among them.
fn token_diff(a: &str, b: &str) -> (usize, u64) {
    let (ta, tb): (Vec<&str>, Vec<&str>) = (a.split(' ').collect(), b.split(' ').collect());
    let mut n = 0;
    let mut ulps = 0u64;
    for k in 0..ta.len().max(tb.len()) {
        let (x, y) = (ta.get(k).copied().unwrap_or(""), tb.get(k).copied().unwrap_or(""));
        if x != y {
            n += 1;
            if x.len() == 16 && y.len() == 16 {
                if let (Ok(u), Ok(v)) = (u64::from_str_radix(x, 16), u64::from_str_radix(y, 16)) {
                    ulps = ulps.max((u as i64).wrapping_sub(v as i64).unsigned_abs());
                }
            }
        }
    }
    (n, ulps)
}

#[test]
fn model_matches_the_pin() {
    let mut o = Out { lines: Vec::new(), base: 0 };
    scenario_prim(&mut o);
    scenario_build(&mut o);
    scenario_ops(&mut o);
    scenario_ids(&mut o);
    scenario_instance(&mut o);
    scenario_extra(&mut o);

    let expected = read_vectors();
    let expected: Vec<&str> = expected.lines().collect();
    if let Ok(path) = std::env::var("MODEL_PARITY_DUMP") {
        std::fs::write(path, o.lines.join("\n") + "\n").expect("dump");
    }
    assert_eq!(o.lines.len(), expected.len(), "number of lines");

    let mut failures = Vec::new();
    let mut svd_diffs: std::collections::BTreeMap<String, (usize, usize, u64)> = std::collections::BTreeMap::new();
    let mut compared: std::collections::BTreeMap<String, usize> = std::collections::BTreeMap::new();
    for (k, (got, want)) in o.lines.iter().zip(expected.iter()).enumerate() {
        let name = want.split(' ').next().unwrap_or("").to_string();
        let key = if name.starts_with("svd_") || name.starts_with("geo_") { name.clone() } else { name.trim_end_matches(char::is_numeric).to_string() };
        *compared.entry(key.clone()).or_insert(0) += 1;
        if got == want {
            continue;
        }
        if name.starts_with("svd_") || name.starts_with("geo_") {
            let (n, ulps) = token_diff(got, want);
            let e = svd_diffs.entry(name).or_insert((0, 0, 0));
            e.0 += 1;
            e.1 += n;
            e.2 = e.2.max(ulps);
        } else {
            failures.push(format!("line {}:\n  rust {}\n  c++  {}", k + 1, got, want));
        }
    }
    println!("compared lines by name: {:?}", compared);
    for (name, (lines, tokens, ulps)) in &svd_diffs {
        println!("reported-only differences (decomposition / other units): {name}: {lines} lines, {tokens} tokens, max {ulps} ulps");
    }
    if !failures.is_empty() {
        let shown: Vec<&String> = failures.iter().take(12).collect();
        panic!("{} lines differ from the C++:\n{}", failures.len(), shown.iter().map(|s| s.as_str()).collect::<Vec<_>>().join("\n"));
    }
}
