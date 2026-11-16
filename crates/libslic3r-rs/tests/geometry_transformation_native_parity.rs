//! The transformation functions of `slicer::geometry::geometry` (the port of Geometry.cpp:
//! assemble_transform, extract_euler_angles, rotation_transform, Transformation,
//! TransformationSVD, volume_to_bed_transformation, rotation_from_two_vectors, rotation_xyz_diff,
//! rotation_diff_z, mat_around_a_point_rotate, generate_transform), `BoundingBoxf3::transformed`
//! (BoundingBox.cpp) and the Eigen operations of `geometry::eigen` against the C++ at the pin.
//!
//! `tests/data/geometry_transformation/vectors.txt.gz` is the output of `gen.cpp` beside it, which
//! links the pin's libslic3r and runs these functions on 2350 matrices of nine families (random
//! affines with mirrors, rotations, rotations from Euler angles with multiples of 90 degrees,
//! assembled transforms, scales, near-singular, skewed, signed permutations, tiny rotations), plus
//! pairs of vectors and Eigen primitives; the header of gen.cpp describes the lines. The same
//! inputs are made here with the same generator, and every result has to be the same bit for bit
//! (the first four lines of every tag written out, the others compared through the FNV-1a hash of
//! the text). Run with the Eigen arm of `TRANSFORMATION_EIGEN` (the default).
//!
//! `GEOMETRY_TRANSFORMATION_VECTORS=<file>` reads a plain-text output of the generator instead (run
//! with `FULL=1`, every payload written out) to see the values of a differing hashed line.

use nalgebra::{Matrix3, Matrix4, Vector3};
use slicer::bounding_box::BoundingBoxf3;
use slicer::geometry::geometry::{
    assemble_transform, eigen, extract_euler_angles, extract_euler_angles_from_matrix, generate_transform,
    mat_around_a_point_rotate, rotation_diff_z, rotation_from_two_vectors, rotation_transform, rotation_xyz_diff,
    scale_transform, scale_transform_uniform, translation_transform, Axis, Transformation, TransformationSVD,
};
use std::collections::BTreeMap;
use std::io::Read;

type Matrix3d = Matrix3<f64>;
type Vec3d = Vector3<f64>;
type Transform3d = Matrix4<f64>;

const FULL: usize = 4;
const PI_: f64 = 3.14159265358979323846;

fn read_vectors() -> String {
    if let Ok(path) = std::env::var("GEOMETRY_TRANSFORMATION_VECTORS") {
        return std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
    }
    let path = format!("{}/tests/data/geometry_transformation/vectors.txt.gz", env!("CARGO_MANIFEST_DIR"));
    let file = std::fs::File::open(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
    let mut text = String::new();
    flate2::read::GzDecoder::new(file).read_to_string(&mut text).expect("gzip");
    text
}

fn fnv(s: &str) -> String {
    let mut h: u64 = 1469598103934665603;
    for c in s.bytes() {
        h ^= c as u64;
        h = h.wrapping_mul(1099511628211);
    }
    format!("#{:016x}", h)
}

#[derive(Default)]
struct Out {
    s: String,
}

impl Out {
    fn push(&mut self, t: String) {
        if !self.s.is_empty() {
            self.s.push(' ');
        }
        self.s += &t;
    }
    fn d(&mut self, v: f64) {
        self.push(format!("{:016x}", v.to_bits()));
    }
    fn b(&mut self, v: bool) {
        self.push((if v { "1" } else { "0" }).to_string());
    }
    fn v3(&mut self, v: &Vec3d) {
        self.d(v.x);
        self.d(v.y);
        self.d(v.z);
    }
    fn m3(&mut self, m: &Matrix3d) {
        for r in 0..3 {
            for c in 0..3 {
                self.d(m[(r, c)]);
            }
        }
    }
    fn m4(&mut self, t: &Transform3d) {
        for r in 0..3 {
            for c in 0..4 {
                self.d(t[(r, c)]);
            }
        }
    }
    fn q(&mut self, x: f64, y: f64, z: f64, w: f64) {
        self.d(x);
        self.d(y);
        self.d(z);
        self.d(w);
    }
}

/// The lines of the vectors, compared in order.
struct Checker<'a> {
    lines: std::str::Lines<'a>,
    full_all: bool,
    compared: BTreeMap<String, usize>,
    differing: BTreeMap<String, usize>,
    shown: usize,
}

impl<'a> Checker<'a> {
    fn check(&mut self, tag: &str, k: usize, out: &Out) {
        let line = self.lines.next().unwrap_or_else(|| panic!("vectors end before {tag} {k}"));
        let mut it = line.splitn(3, ' ');
        let (t, kk, want) = (it.next().unwrap(), it.next().unwrap(), it.next().unwrap_or(""));
        assert_eq!((t, kk), (tag, k.to_string().as_str()), "line order");
        *self.compared.entry(tag.to_string()).or_insert(0) += 1;
        let ok = if let Some(h) = want.strip_prefix('#') {
            fnv(&out.s)[1..] == *h
        } else {
            want == out.s
        };
        if !ok {
            *self.differing.entry(tag.to_string()).or_insert(0) += 1;
            if self.shown < 12 {
                self.shown += 1;
                eprintln!("{tag} {k} differs:\n  c++  {want}\n  rust {}", out.s);
            }
        }
        let _ = self.full_all;
    }
}

/// splitmix64, as gen.cpp
struct Rng {
    s: u64,
}
impl Rng {
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
    fn below(&mut self, n: i32) -> i32 {
        (self.next() % n as u64) as i32
    }
}

fn rand_rot(r: &mut Rng) -> Matrix3d {
    let (mut w, mut x, mut y, mut z) = (r.range(-1.0, 1.0), r.range(-1.0, 1.0), r.range(-1.0, 1.0), r.range(-1.0, 1.0));
    let n = (w * w + x * x + y * y + z * z).sqrt();
    w /= n;
    x /= n;
    y /= n;
    z /= n;
    Matrix3d::new(
        1.0 - 2.0 * (y * y + z * z),
        2.0 * (x * y - z * w),
        2.0 * (x * z + y * w),
        2.0 * (x * y + z * w),
        1.0 - 2.0 * (x * x + z * z),
        2.0 * (y * z - x * w),
        2.0 * (x * z - y * w),
        2.0 * (y * z + x * w),
        1.0 - 2.0 * (x * x + y * y),
    )
}

fn euler_rot(a: f64, b: f64, c: f64) -> Matrix3d {
    let (ca, sa, cb, sb, cc, sc) = (a.cos(), a.sin(), b.cos(), b.sin(), c.cos(), c.sin());
    let rx = Matrix3d::new(1.0, 0.0, 0.0, 0.0, ca, -sa, 0.0, sa, ca);
    let ry = Matrix3d::new(cb, 0.0, sb, 0.0, 1.0, 0.0, -sb, 0.0, cb);
    let rz = Matrix3d::new(cc, -sc, 0.0, sc, cc, 0.0, 0.0, 0.0, 1.0);
    let mut t = Matrix3d::zeros();
    let mut m = Matrix3d::zeros();
    for i in 0..3 {
        for j in 0..3 {
            t[(i, j)] = rz[(i, 0)] * ry[(0, j)] + rz[(i, 1)] * ry[(1, j)] + rz[(i, 2)] * ry[(2, j)];
        }
    }
    for i in 0..3 {
        for j in 0..3 {
            m[(i, j)] = t[(i, 0)] * rx[(0, j)] + t[(i, 1)] * rx[(1, j)] + t[(i, 2)] * rx[(2, j)];
        }
    }
    m
}

fn rand_angle(r: &mut Rng) -> f64 {
    let kind = r.below(4);
    if kind == 0 {
        return (r.below(9) - 4) as f64 * (0.5 * PI_);
    }
    if kind == 1 {
        let k = (r.below(9) - 4) as f64;
        let e = r.range(-1e-9, 1e-9);
        return k * (0.5 * PI_) + e;
    }
    r.range(-PI_, PI_)
}

fn affine(lin: &Matrix3d, t: &Vec3d) -> Transform3d {
    let mut m = Transform3d::identity();
    for i in 0..3 {
        for j in 0..3 {
            m[(i, j)] = lin[(i, j)];
        }
        m[(i, 3)] = t[i];
    }
    m
}

fn times_diag(m: &Matrix3d, s: &Vec3d) -> Matrix3d {
    let mut o = Matrix3d::zeros();
    for i in 0..3 {
        for j in 0..3 {
            o[(i, j)] = m[(i, j)] * s[j];
        }
    }
    o
}

fn rand_vec(r: &mut Rng, a: f64, b: f64) -> Vec3d {
    let x = r.range(a, b);
    let y = r.range(a, b);
    let z = r.range(a, b);
    Vec3d::new(x, y, z)
}

const FAM_NAMES: [&str; 9] = ["aff", "rot", "rte", "asm", "sca", "sing", "gen", "int", "tiny"];
const FAM_COUNT: [usize; 9] = [600, 250, 250, 400, 150, 200, 250, 150, 100];

fn make_case(fam: usize, r: &mut Rng) -> Transform3d {
    match fam {
        0 => {
            let rot = rand_rot(r);
            let mut s = rand_vec(r, 0.5, 2.0);
            let mi = r.below(4) as usize;
            if mi < 3 {
                s[mi] = -s[mi];
            }
            let m = times_diag(&rot, &s);
            affine(&m, &rand_vec(r, -50.0, 50.0))
        }
        1 => {
            let m = rand_rot(r);
            affine(&m, &rand_vec(r, -50.0, 50.0))
        }
        2 => {
            let (a, b, c) = (rand_angle(r), rand_angle(r), rand_angle(r));
            let m = euler_rot(a, b, c);
            affine(&m, &rand_vec(r, -50.0, 50.0))
        }
        3 => {
            let (a, b, c) = (rand_angle(r), rand_angle(r), rand_angle(r));
            let mut s = rand_vec(r, 0.1, 10.0);
            for i in 0..3 {
                if r.below(4) == 0 {
                    s[i] = -s[i];
                }
            }
            if r.below(3) == 0 {
                s[2] = s[0];
                s[1] = s[0];
            }
            let m = times_diag(&euler_rot(a, b, c), &s);
            affine(&m, &rand_vec(r, -100.0, 100.0))
        }
        4 => {
            let mut s = rand_vec(r, 0.01, 100.0);
            let kind = r.below(3);
            if kind == 0 {
                s[2] = s[0];
                s[1] = s[0];
            }
            if kind == 1 {
                let i = r.below(3) as usize;
                s[i] *= -1.0;
            }
            let m = times_diag(&Matrix3d::identity(), &s);
            affine(&m, &rand_vec(r, -50.0, 50.0))
        }
        5 => {
            let rot = rand_rot(r);
            let mut s = rand_vec(r, 0.5, 2.0);
            let k = r.below(5);
            let tiny = match k {
                0 => 1e-3,
                1 => 1e-6,
                2 => 1e-9,
                3 => 1e-12,
                _ => 0.0,
            };
            let axis = r.below(3) as usize;
            s[axis] = tiny * s[axis];
            if r.below(4) == 0 {
                s[(axis + 1) % 3] = tiny * 0.5;
            }
            let m = times_diag(&rot, &s);
            affine(&m, &rand_vec(r, -50.0, 50.0))
        }
        6 => {
            let mut m = Matrix3d::zeros();
            for i in 0..3 {
                for j in 0..3 {
                    m[(i, j)] = r.range(-2.0, 2.0);
                }
            }
            affine(&m, &rand_vec(r, -50.0, 50.0))
        }
        7 => {
            const PERMS: [[usize; 3]; 6] = [[0, 1, 2], [0, 2, 1], [1, 0, 2], [1, 2, 0], [2, 0, 1], [2, 1, 0]];
            const SCALES: [f64; 4] = [1.0, 2.0, 0.5, 3.0];
            let p = r.below(6) as usize;
            let mut m = Matrix3d::zeros();
            for i in 0..3 {
                m[(i, PERMS[p][i])] = if r.below(2) != 0 { -1.0 } else { 1.0 };
            }
            let s0 = SCALES[r.below(4) as usize];
            let s1 = SCALES[r.below(4) as usize];
            let s2 = SCALES[r.below(4) as usize];
            let t0 = (r.below(21) - 10) as f64;
            let t1 = (r.below(21) - 10) as f64;
            affine(&times_diag(&m, &Vec3d::new(s0, s1, s2)), &Vec3d::new(t0, t1, 0.0))
        }
        _ => {
            let ax = rand_vec(r, -1.0, 1.0);
            let n = (ax.x * ax.x + ax.y * ax.y + ax.z * ax.z).sqrt();
            let ax = Vec3d::new(ax.x / n, ax.y / n, ax.z / n);
            let ang = 10.0f64.powf(r.range(-12.0, -2.0));
            let (c, s) = (ang.cos(), ang.sin());
            let t = 1.0 - c;
            let m = Matrix3d::new(
                t * ax.x * ax.x + c,
                t * ax.x * ax.y - s * ax.z,
                t * ax.x * ax.z + s * ax.y,
                t * ax.x * ax.y + s * ax.z,
                t * ax.y * ax.y + c,
                t * ax.y * ax.z - s * ax.x,
                t * ax.x * ax.z - s * ax.y,
                t * ax.y * ax.z + s * ax.x,
                t * ax.z * ax.z + c,
            );
            let sc = if r.below(2) != 0 { Vec3d::new(1.5, 1.5, 1.5) } else { Vec3d::new(1.0, 1.0, 1.0) };
            let ms = times_diag(&m, &sc);
            affine(&ms, &rand_vec(r, -50.0, 50.0))
        }
    }
}

fn rand_mirror(r: &mut Rng) -> f64 {
    match r.below(5) {
        0 => -1.0,
        1 => 1.0,
        2 => 0.0,
        3 => -2.5,
        _ => 0.5,
    }
}

fn axis_of(i: i32) -> Axis {
    match i {
        0 => Axis::X,
        1 => Axis::Y,
        _ => Axis::Z,
    }
}

fn linear(t: &Transform3d) -> Matrix3d {
    t.fixed_view::<3, 3>(0, 0).into_owned()
}

fn line_a(t: &Transform3d) -> Out {
    let mut p = Out::default();
    let (u, v, sv) = eigen::jacobi_svd3(&linear(t));
    p.m3(&u);
    p.m3(&v);
    p.v3(&sv);
    let (rr, ss) = eigen::compute_rotation_scaling(&linear(t));
    p.m3(&rr);
    p.m3(&ss);
    let tr = Transformation::from_transform(*t);
    p.v3(&tr.get_rotation());
    p.v3(&tr.get_scaling_factor());
    p.v3(&tr.get_mirror());
    p.v3(&tr.get_rotation_by_quaternion());
    p.b(tr.has_skew());
    p.b(tr.is_left_handed());
    p.b(tr.is_scaling_uniform());
    p.v3(&extract_euler_angles(t));
    p.v3(&extract_euler_angles_from_matrix(&rr));
    let ts = TransformationSVD::from_transform(t);
    p.m3(&ts.u);
    p.m3(&ts.s);
    p.m3(&ts.v);
    p.b(ts.mirror);
    p.b(ts.scale);
    p.b(ts.anisotropic_scale);
    p.b(ts.rotation);
    p.b(ts.rotation_90_degrees);
    p.b(ts.skew);
    p.m4(&tr.get_rotation_matrix());
    p.m4(&tr.get_scaling_factor_matrix());
    p.m4(&tr.get_mirror_matrix());
    p
}

fn line_b(t: &Transform3d, r: &mut Rng) -> Out {
    let mut p = Out::default();
    let tr = Transformation::from_transform(*t);
    let r0 = rand_angle(r);
    let r1 = rand_angle(r);
    let r2 = rand_angle(r);
    let rot = Vec3d::new(r0, r1, r2);
    let sc = rand_vec(r, 0.2, 3.0);
    let axis = r.below(3);
    let sa = r.range(0.2, 3.0);
    let m0 = rand_mirror(r);
    let m1 = rand_mirror(r);
    let m2 = rand_mirror(r);
    let mir = Vec3d::new(m0, m1, m2);
    let ma = rand_mirror(r);
    let tv = rand_vec(r, -100.0, 100.0);
    let su = r.range(0.2, 3.0);
    let mut c = tr;
    c.set_rotation(&rot);
    p.m4(&c.matrix());
    let mut c = tr;
    c.set_scaling_factor(&sc);
    p.m4(&c.matrix());
    let mut c = tr;
    c.set_scaling_factor_axis(axis_of(axis), sa);
    p.m4(&c.matrix());
    let mut c = tr;
    c.set_mirror(&mir);
    p.m4(&c.matrix());
    let mut c = tr;
    c.set_mirror_axis(axis_of(axis), ma);
    p.m4(&c.matrix());
    let mut c = tr;
    c.set_rotation_matrix(&rotation_transform(&rot));
    p.m4(&c.matrix());
    p.m4(&rotation_transform(&rot));
    let m3 = Vec3d::new(
        if mir.x < 0.0 { -1.0 } else { 1.0 },
        if mir.y < 0.0 { -1.0 } else { 1.0 },
        if mir.z < 0.0 { -1.0 } else { 1.0 },
    );
    p.m4(&assemble_transform(&tv, &rot, &sc, &m3));
    p.m4(&translation_transform(&tv));
    p.m4(&scale_transform(&sc));
    p.m4(&scale_transform_uniform(su));
    p
}

fn line_c(t: &Transform3d) -> Out {
    let mut p = Out::default();
    let tr = Transformation::from_transform(*t);
    let mut c = tr;
    c.reset_rotation();
    p.m4(&c.matrix());
    let mut c = tr;
    c.reset_scaling_factor();
    p.m4(&c.matrix());
    let mut c = tr;
    c.reset_skew();
    p.m4(&c.matrix());
    let mut c = tr;
    c.reset_mirror();
    p.m4(&c.matrix());
    for f in 1..16 {
        p.m4(&tr.get_matrix(f & 1 != 0, f & 2 != 0, f & 4 != 0, f & 8 != 0));
    }
    p.m4(&tr.get_matrix_no_offset());
    p.m4(&tr.get_matrix_no_scaling_factor());
    p
}

fn bb_matrix(t: &Transform3d) -> [[f64; 4]; 4] {
    let mut m = [[0.0; 4]; 4];
    for (r, row) in m.iter_mut().enumerate() {
        for (c, x) in row.iter_mut().enumerate() {
            *x = t[(r, c)];
        }
    }
    m
}

fn line_d(t: &Transform3d, r: &mut Rng) -> Out {
    let mut p = Out::default();
    let tr = Transformation::from_transform(*t);
    let t2 = make_case(0, r);
    let bmin = rand_vec(r, -20.0, 0.0);
    let bmax = rand_vec(r, 0.0, 20.0);
    let bb = BoundingBoxf3::new_from_points(
        slicer::geometry::Vec3d::new(bmin.x, bmin.y, bmin.z),
        slicer::geometry::Vec3d::new(bmax.x, bmax.y, bmax.z),
    );
    let v = rand_vec(r, -30.0, 30.0);
    let pt = rand_vec(r, -30.0, 30.0);
    let ax = rand_vec(r, -1.0, 1.0);
    let theta = r.range(-PI_, PI_) as f32;
    p.m4(&(tr * Transformation::from_transform(t2)).matrix());
    let (sf, mf) = (tr.get_scaling_factor(), tr.get_mirror());
    let v2b_ok = (0..3).all(|i| sf[i] > 0.0 && sf[i].is_finite() && mf[i].is_finite());
    let v2b_in = if v2b_ok { tr } else { Transformation::new() };
    p.m4(&Transformation::volume_to_bed_transformation(&v2b_in, &bb).matrix());
    p.m4(&rotation_transform(&tr.get_rotation()));
    let tb = bb.transformed(&bb_matrix(t));
    p.b(tb.defined);
    p.v3(&Vec3d::new(tb.min.x, tb.min.y, tb.min.z));
    p.v3(&Vec3d::new(tb.max.x, tb.max.y, tb.max.z));
    p.v3(&eigen::transform_mul_vec(t, &v));
    p.m4(&eigen::transform_inverse(t));
    p.d(eigen::determinant3(&linear(t)));
    p.m4(&mat_around_a_point_rotate(&tr, &pt, &ax, theta).matrix());
    p
}

fn lines_v(r: &mut Rng, ck: &mut Checker) {
    for k in 0..400 {
        let mut p = Out::default();
        let mut from = rand_vec(r, -2.0, 2.0);
        let to;
        let kind = r.below(6);
        if kind == 0 || kind == 1 {
            let mut f = r.range(0.5, 2.0);
            if kind == 1 {
                f = -f;
            }
            let e = rand_vec(r, -1e-7, 1e-7);
            to = Vec3d::new(from.x * f + e.x, from.y * f + e.y, from.z * f + e.z);
        } else if kind == 2 {
            to = Vec3d::new(0.0, 0.0, 1.0);
        } else if kind == 3 {
            let fx = r.range(-1e-3, 1e-3);
            let fy = r.range(-1e-3, 1e-3);
            let fz = if r.below(2) != 0 { 1.0 } else { -1.0 };
            from = Vec3d::new(fx, fy, fz);
            to = Vec3d::new(0.0, 0.0, 1.0);
        } else {
            to = rand_vec(r, -2.0, 2.0);
        }
        let mut axis = Vec3d::zeros();
        let mut phi = 0.0;
        let mut m = Matrix3d::zeros();
        rotation_from_two_vectors(from, to, &mut axis, &mut phi, Some(&mut m));
        p.v3(&axis);
        p.d(phi);
        p.m3(&m);
        let f0 = rand_angle(r);
        let f1 = rand_angle(r);
        let f2 = rand_angle(r);
        let t0 = rand_angle(r);
        let t1 = rand_angle(r);
        let t2 = rand_angle(r);
        let r_from = Vec3d::new(f0, f1, f2);
        let mut r_to = Vec3d::new(t0, t1, t2);
        if r.below(2) != 0 {
            let z = r.range(-PI_, PI_);
            r_to = Vec3d::new(r_from.x, r_from.y, z);
        }
        let q = rotation_xyz_diff(&r_from, &r_to).into_inner();
        p.q(q.i, q.j, q.k, q.w);
        p.d(rotation_diff_z(&r_from, &r_to));
        let o = rand_vec(r, -50.0, 50.0);
        let b = rand_rot(r);
        let sx = rand_vec(r, 0.5, 2.0);
        let gx = Vec3d::new(b[(0, 0)] * sx.x, b[(1, 0)] * sx.x, b[(2, 0)] * sx.x);
        let gy = Vec3d::new(b[(0, 1)] * sx.y, b[(1, 1)] * sx.y, b[(2, 1)] * sx.y);
        let gz = Vec3d::new(b[(0, 2)] * sx.z, b[(1, 2)] * sx.z, b[(2, 2)] * sx.z);
        p.m4(&generate_transform(&gx, &gy, &gz, &o).matrix());
        ck.check(if kind == 1 { "va" } else { "v" }, k, &p);
    }
    let exact: [[f64; 6]; 5] =
        [[0.0, 0.0, -1.0, 0.0, 0.0, 1.0], [1.0, 0.0, 0.0, -2.0, 0.0, 0.0], [0.6, 0.0, -0.8, -0.6, 0.0, 0.8], [0.0, 1.0, 0.0, 0.0, -1.0, 0.0], [0.0, 0.0, 1.0, 0.0, 0.0, -1.0]];
    for (k, e) in exact.iter().enumerate() {
        let mut p = Out::default();
        let mut axis = Vec3d::zeros();
        let mut phi = 0.0;
        let mut m = Matrix3d::zeros();
        rotation_from_two_vectors(Vec3d::new(e[0], e[1], e[2]), Vec3d::new(e[3], e[4], e[5]), &mut axis, &mut phi, Some(&mut m));
        p.v3(&axis);
        p.d(phi);
        p.m3(&m);
        ck.check("vx", k, &p);
    }
}

fn lines_q(r: &mut Rng, ck: &mut Checker) {
    for k in 0..600 {
        let mut p = Out::default();
        let a1 = rand_angle(r);
        let a2 = r.range(-10.0, 10.0);
        let ax1 = rand_vec(r, -1.0, 1.0);
        let ax2 = rand_vec(r, -1.0, 1.0);
        let n1 = (ax1.x * ax1.x + ax1.y * ax1.y + ax1.z * ax1.z).sqrt();
        let n2 = (ax2.x * ax2.x + ax2.y * ax2.y + ax2.z * ax2.z).sqrt();
        let mut ax1 = Vec3d::new(ax1.x / n1, ax1.y / n1, ax1.z / n1);
        let mut ax2 = Vec3d::new(ax2.x / n2, ax2.y / n2, ax2.z / n2);
        if k % 7 == 0 {
            ax1 = Vec3d::new(0.0, 0.0, 1.0);
        }
        if k % 11 == 0 {
            ax2 = Vec3d::new(1.0, 0.0, 0.0);
        }
        let q1 = eigen::Quaterniond::from_angle_axis(a1, &ax1);
        let q2 = eigen::Quaterniond::from_angle_axis(a2, &ax2);
        p.q(q1.x, q1.y, q1.z, q1.w);
        p.q(q2.x, q2.y, q2.z, q2.w);
        let q12 = q1.mul(&q2);
        p.q(q12.x, q12.y, q12.z, q12.w);
        let m12 = q12.to_rotation_matrix();
        p.m3(&m12);
        let rr = if k % 5 == 0 { times_diag(&rand_rot(r), &Vec3d::new(1.0, 1.0, 1.0)) } else { m12 };
        let qm = eigen::Quaterniond::from_rotation_matrix(&rr);
        p.q(qm.x, qm.y, qm.z, qm.w);
        let mut qn = qm;
        qn.normalize();
        p.q(qn.x, qn.y, qn.z, qn.w);
        let aa = eigen::AngleAxisd::from_rotation_matrix(&rr);
        p.v3(&aa.axis);
        p.d(aa.angle);
        p.m3(&eigen::AngleAxisd::new(a2, ax2).to_rotation_matrix());
        p.v3(&eigen::euler_angles(&rr, 2, 1, 0));
        let v = rand_vec(r, -10.0, 10.0);
        let w = rand_vec(r, -10.0, 10.0);
        p.d(eigen::squared_norm3(&v));
        p.d(eigen::norm3(&v));
        p.v3(&eigen::normalized3(&v));
        p.d(eigen::dot3(&v, &w));
        p.v3(&eigen::cross3(&v, &w));
        let mut a = Matrix3d::zeros();
        let mut b = Matrix3d::zeros();
        for i in 0..3 {
            for j in 0..3 {
                a[(i, j)] = r.range(-3.0, 3.0);
                b[(i, j)] = r.range(-3.0, 3.0);
            }
        }
        p.m3(&eigen::mat3_mul(&a, &b));
        p.v3(&eigen::mat3_mul_vec(&a, &v));
        p.d(eigen::determinant3(&a));
        p.m3(&eigen::mat3_mul(&a, &b.transpose()));
        ck.check("q", k, &p);
    }
}

#[test]
fn transformations_match_the_pin() {
    let text = read_vectors();
    let full_all = !text.lines().any(|l| l.contains(" #"));
    let mut ck = Checker { lines: text.lines(), full_all, compared: BTreeMap::new(), differing: BTreeMap::new(), shown: 0 };
    let mut r = Rng { s: 0x5eed5eed };
    for fam in 0..9 {
        let name = FAM_NAMES[fam];
        for i in 0..FAM_COUNT[fam] {
            let t = make_case(fam, &mut r);
            let mut m = Out::default();
            m.m4(&t);
            ck.check(&format!("m.{name}"), i, &m);
            let a = line_a(&t);
            ck.check(&format!("a.{name}"), i, &a);
            let b = line_b(&t, &mut r);
            ck.check(&format!("b.{name}"), i, &b);
            let c = line_c(&t);
            ck.check(&format!("c.{name}"), i, &c);
            let d = line_d(&t, &mut r);
            ck.check(&format!("d.{name}"), i, &d);
        }
    }
    lines_v(&mut r, &mut ck);
    lines_q(&mut r, &mut ck);
    assert!(ck.lines.next().is_none(), "vectors have more lines");
    let _ = FULL;
    println!("compared lines by tag: {:?}", ck.compared);
    if !ck.differing.is_empty() {
        panic!("lines differ from the C++ by tag: {:?}", ck.differing);
    }
}
