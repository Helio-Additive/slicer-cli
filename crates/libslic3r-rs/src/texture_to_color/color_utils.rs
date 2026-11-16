//! Port of `TextureToColor/ColorUtils.hpp` / `ColorUtils.cpp`: color spaces and the CIEDE2000
//! difference, clustering of a list of colors (K-Means, adaptive, to given colors), and the
//! operations on a mesh whose faces carry a color or a label (connected components, clustering
//! of the face colors, smoothing of the labels across faces).
//!
//! C++ Reference:
//! - src/libslic3r/TextureToColor/ColorUtils.hpp
//! - src/libslic3r/TextureToColor/ColorUtils.cpp
//!
//! Floating point follows the reference build (clang, `-ffp-contract=on`): `a * b + c` inside one
//! source expression is one fused operation, written `mul_add` here; a product that comes out of
//! a called function (`CGAL_NTS square`, a lambda) is not fused with the sum around it.
//!
//! FIDELITY-NOTE: `std::pow(x, 2)` (ColorUtils.cpp:615, 619, 624) is written `x * x`: clang turns
//! that call into the multiplication, and the product is not fused with the sum around it.
//! `std::pow(C_prime_avg, 7)` (ColorUtils.cpp:613) stays a call of `pow`.
//!
//! FIDELITY-NOTE: the mesh functions work on `cgal_utils::CGALMesh`, the crate's stand-in for
//! `CGAL::Surface_mesh`, and on what `cgal_utils.rs` writes out of CGAL: the kernel's
//! constructions, `PMP::compute_face_normal`, `CGAL::centroid` and the AABB tree's closest point.
//! `PMP::face_area` (`squared_area` of a triangle), `Vector_3::squared_length` and the division
//! of a vector are written out here.
//!
//! FIDELITY-NOTE: `remesh_mesh` (ColorUtils.cpp:699-839) and `check_and_repair_self_intersect`
//! (ColorUtils.cpp:126-142) call CGAL algorithms the crate has no port of
//! (`PMP::isotropic_remeshing`, `PMP::does_self_intersect`,
//! `PMP::experimental::remove_self_intersections`). They take them as
//! `cgal_utils::PolygonMeshProcessing`, which nothing in the crate implements, and are ports of
//! the rest of their bodies. `remesh_mesh` has no caller at the pin.
//!
//! FIDELITY-NOTE: not ported: `sort_colors_by_pca` with `struct ColorPCA` (ColorUtils.cpp:1145-1230,
//! `[[maybe_unused]]`, called by nothing): Eigen's dense matrix product and
//! `SelfAdjointEigenSolver`, of which the crate has no port.
//!
//! FIDELITY-NOTE: the two overloads of `smooth_region` (ColorUtils.cpp:927, 954) are
//! `smooth_region` (colors) and `smooth_region_with_labels` (labels).
//!
//! What C++ writes to the log is not written.
//!
//! FIDELITY-NOTE: `tbb::parallel_for` (ColorUtils.cpp:983, 1121, 1282, 1467, 1485) is a plain
//! loop here. Each iteration writes its own element only, so the result is the same.
//!
//! FIDELITY-NOTE: `mesh_cluster` and `smooth_region_geom_boundary` iterate `std::unordered_set` /
//! `std::unordered_map` keyed by an index, and their results depend on the order (a face takes
//! the label of neighbours that were labelled earlier in the same pass; a vertex moved earlier is
//! read by the ones after it). `LibcxxUnorderedMap` reproduces the order of libc++'s
//! hash table (identity hash, the bucket counts and the node list of `__hash_table`), which is
//! what the reference build iterates.
//!
//! FIDELITY-NOTE: `cluster_adaptive` takes the percentile with `std::nth_element`
//! (ColorUtils.cpp:1298); here the distances are sorted and the same element is read. The value
//! at that position is the same for any correct selection as long as no distance is NaN.
//!
//! Nothing in the slicing path calls this unit. Its callers in C++ are TextureToColor.cpp and
//! TexturePainting.cpp (the texture-to-painting import of the desktop app).

use std::collections::{HashMap, HashSet, VecDeque};
use std::sync::Arc;

use crate::aabb_tree_indirect;
use crate::geometry::aabb_tree::Vec3;
use crate::geometry::Point3F;
use crate::texture_to_color::cgal_utils::{
    self, centroid, compute_face_normal, squared_distance, CGALMesh, Point_3, SM_Edge_index, SM_Face_index, SM_Vertex_index, Tree, Vector_3,
};
use crate::texture_to_color::cgal_utils::PolygonMeshProcessing;
use crate::texture_to_color::tri_mesh::{TriFace, TriFaces, TriMesh, TriVertex, TriVertices};
use crate::triangle_mesh::Vec2f;
use crate::triangle_set_sampling::Vec3d;

/// ColorUtils.hpp:11, RGB: [R, G, B] 0~255
pub type Color = [usize; 3];
/// ColorUtils.hpp:12
pub type ColorList = Vec<Color>;
/// ColorUtils.hpp:13
pub type ColorDouble = [f64; 3];
/// ColorUtils.hpp:14
pub type RGB = [usize; 3];

/// ColorUtils.hpp:16
pub type DistanceFunction = fn(&Color, &Color) -> f64;

/// ColorUtils.hpp:18-21
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ColorDifferenceMethod {
    /// Simplest and fastest
    RGB = 0,
    /// Most perceptually accurate
    Lab = 1,
}

/// ColorUtils.hpp:23-35
#[derive(Clone)]
pub struct ClusterParameters {
    /// Method for measuring color difference; Lab is the most accurate
    pub color_difference_method: ColorDifferenceMethod,
    /// Max intra-cluster radius (CIEDE2000 dE) for adaptive clustering; ignored by the fixed-K algorithm
    pub max_color_distance: f64,
    /// Target number of cluster centers; ignored by the adaptive algorithm
    pub cluster_k: usize,
    /// Max cluster count upper bound for adaptive algorithm
    pub max_cluster_k: usize,
    /// Maximum number of iterations
    pub max_iter: usize,
    /// Optional cancellation check; returns true when the caller requests abort
    pub cancel_callback: Option<Arc<dyn Fn() -> bool + Send + Sync>>,
}

impl Default for ClusterParameters {
    /// ColorUtils.hpp:24-34, the member initializers
    fn default() -> Self {
        ClusterParameters {
            color_difference_method: ColorDifferenceMethod::Lab,
            max_color_distance: 25.0,
            cluster_k: 10,
            max_cluster_k: 32,
            max_iter: 50,
            cancel_callback: None,
        }
    }
}

/// ColorUtils.hpp:37-39
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SmoothParameters {
    /// Controls smoothing intensity; larger values produce smoother results. Range: [0.0, 1.0]
    pub smooth_weight: f64,
}

impl Default for SmoothParameters {
    /// ColorUtils.hpp:38
    fn default() -> Self {
        SmoothParameters { smooth_weight: 0.5 }
    }
}

/// ColorUtils.cpp:37-39 does not apply: `<cmath>` defines `M_PI`.
const M_PI: f64 = 3.14159265358979323846264338327950288;

/// ColorUtils.cpp:41-43
const EPSILON: f64 = 1e-6;

/// ColorUtils.cpp:56
const TOPO_SMOOTH_WEIGHT_THRESHOLD: f64 = 0.3;
/// ColorUtils.cpp:57-58: skip adaptive clustering when unique colors are already at or below this count.
const ADAPTIVE_CLUSTER_SKIP_UNIQUE_COUNT: usize = 32;

/// `std::unordered_map<std::size_t, V>` of libc++, as far as the order of iteration goes: the
/// nodes are one singly linked list, a bucket points at the node before its first one, the hash
/// of a `std::size_t` is the value itself (`__hash_table` of libc++). `std::unordered_set` is the
/// map with `V = ()`.
#[derive(Debug, Clone)]
pub struct LibcxxUnorderedMap<V> {
    /// node 0 is the node before the first (`__p1_`); its key is not read
    keys: Vec<usize>,
    values: Vec<Option<V>>,
    next: Vec<usize>,
    /// `__bucket_list_`: for each bucket the node before its first node, or `NIL`
    buckets: Vec<usize>,
    size: usize,
}

const NIL: usize = usize::MAX;

impl<V: Default> Default for LibcxxUnorderedMap<V> {
    fn default() -> Self {
        Self::new()
    }
}

impl<V: Default> LibcxxUnorderedMap<V> {
    pub fn new() -> Self {
        LibcxxUnorderedMap { keys: vec![0], values: vec![None], next: vec![NIL], buckets: Vec::new(), size: 0 }
    }

    /// `__constrain_hash`
    fn constrain_hash(h: usize, bc: usize) -> usize {
        if bc & (bc - 1) == 0 {
            h & (bc - 1)
        } else if h < bc {
            h
        } else {
            h % bc
        }
    }

    /// `__next_prime`: the smallest prime not below `n`
    fn next_prime(n: usize) -> usize {
        let mut p = n.max(2);
        loop {
            let mut is_prime = true;
            let mut d = 2usize;
            while d * d <= p {
                if p % d == 0 {
                    is_prime = false;
                    break;
                }
                d += 1;
            }
            if is_prime {
                return p;
            }
            p += 1;
        }
    }

    /// `__hash_table::__rehash<true>(n)`, the growing branch (the only one an insertion takes)
    fn rehash(&mut self, mut n: usize) {
        if n == 1 {
            n = 2;
        } else if n & (n - 1) != 0 {
            n = Self::next_prime(n);
        }
        let bc = self.buckets.len();
        if n > bc {
            self.do_rehash(n);
        }
    }

    /// `__hash_table::__do_rehash<true>(nbc)`
    fn do_rehash(&mut self, nbc: usize) {
        self.buckets = vec![NIL; nbc];
        let mut pp = 0usize;
        let mut cp = self.next[pp];
        if cp != NIL {
            let mut chash = Self::constrain_hash(self.keys[cp], nbc);
            self.buckets[chash] = pp;
            let mut phash = chash;
            pp = cp;
            cp = self.next[cp];
            while cp != NIL {
                chash = Self::constrain_hash(self.keys[cp], nbc);
                if chash == phash {
                    pp = cp;
                } else if self.buckets[chash] == NIL {
                    self.buckets[chash] = pp;
                    pp = cp;
                    phash = chash;
                } else {
                    let np = cp;
                    self.next[pp] = self.next[np];
                    self.next[np] = self.next[self.buckets[chash]];
                    let b = self.buckets[chash];
                    self.next[b] = cp;
                }
                cp = self.next[pp];
            }
        }
    }

    /// `find(k)`: the node of the key, or `NIL`
    fn find_node(&self, k: usize) -> usize {
        let bc = self.buckets.len();
        if bc != 0 {
            let chash = Self::constrain_hash(k, bc);
            let mut nd = self.buckets[chash];
            if nd != NIL {
                nd = self.next[nd];
                while nd != NIL && (self.keys[nd] == k || Self::constrain_hash(self.keys[nd], bc) == chash) {
                    if self.keys[nd] == k {
                        return nd;
                    }
                    nd = self.next[nd];
                }
            }
        }
        NIL
    }

    /// `count(k)`
    pub fn count(&self, k: usize) -> usize {
        (self.find_node(k) != NIL) as usize
    }

    /// `size()`
    pub fn len(&self) -> usize {
        self.size
    }

    /// `empty()`
    pub fn is_empty(&self) -> bool {
        self.size == 0
    }

    /// `operator[](k)` (`__emplace_unique_key_args`): the value of the key, added with the default
    /// value when it is not there
    pub fn entry(&mut self, k: usize) -> &mut V {
        let found = self.find_node(k);
        if found != NIL {
            return self.values[found].as_mut().expect("a node with a value");
        }
        let mut bc = self.buckets.len();
        // if (size() + 1 > __bc * max_load_factor() || __bc == 0)
        if (self.size + 1) as f32 > bc as f32 * 1.0f32 || bc == 0 {
            let is_hash_power2 = bc > 2 && (bc & (bc - 1)) == 0;
            let a = 2 * bc + (!is_hash_power2) as usize;
            let b = ((self.size + 1) as f32 / 1.0f32).ceil() as usize;
            self.rehash(a.max(b));
            bc = self.buckets.len();
        }
        let chash = Self::constrain_hash(k, bc);
        let nd = self.keys.len();
        self.keys.push(k);
        self.values.push(Some(V::default()));
        self.next.push(NIL);
        let pn = self.buckets[chash];
        if pn == NIL {
            let pn = 0usize;
            self.next[nd] = self.next[pn];
            self.next[pn] = nd;
            // fix up __bucket_list_
            self.buckets[chash] = pn;
            if self.next[nd] != NIL {
                let nh = Self::constrain_hash(self.keys[self.next[nd]], bc);
                self.buckets[nh] = nd;
            }
        } else {
            self.next[nd] = self.next[pn];
            self.next[pn] = nd;
        }
        self.size += 1;
        self.values[nd].as_mut().expect("a node with a value")
    }

    /// `insert(k)` of the set: nothing happens when the key is there
    pub fn insert(&mut self, k: usize) {
        self.entry(k);
    }

    /// `erase(k)` (`__hash_table::remove`)
    pub fn erase(&mut self, k: usize) {
        let cn = self.find_node(k);
        if cn == NIL {
            return;
        }
        let bc = self.buckets.len();
        let chash = Self::constrain_hash(self.keys[cn], bc);
        // find previous node
        let mut pn = self.buckets[chash];
        while self.next[pn] != cn {
            pn = self.next[pn];
        }
        // Fix up __bucket_list_
        if pn == 0 || Self::constrain_hash(self.keys[pn], bc) != chash {
            if self.next[cn] == NIL || Self::constrain_hash(self.keys[self.next[cn]], bc) != chash {
                self.buckets[chash] = NIL;
            }
        }
        if self.next[cn] != NIL {
            let nhash = Self::constrain_hash(self.keys[self.next[cn]], bc);
            if nhash != chash {
                self.buckets[nhash] = pn;
            }
        }
        // remove __cn
        self.next[pn] = self.next[cn];
        self.next[cn] = NIL;
        self.values[cn] = None;
        self.size -= 1;
    }

    /// The keys from `begin()` to `end()`
    pub fn keys(&self) -> Vec<usize> {
        let mut out = Vec::with_capacity(self.size);
        let mut nd = self.next[0];
        while nd != NIL {
            out.push(self.keys[nd]);
            nd = self.next[nd];
        }
        out
    }
}

impl<V: Default + Clone> LibcxxUnorderedMap<V> {
    /// The pairs from `begin()` to `end()`
    pub fn items(&self) -> Vec<(usize, V)> {
        let mut out = Vec::with_capacity(self.size);
        let mut nd = self.next[0];
        while nd != NIL {
            out.push((self.keys[nd], self.values[nd].clone().expect("a node with a value")));
            nd = self.next[nd];
        }
        out
    }
}

/// `detail::average_edge_length_impl` (ColorUtils.cpp:61-73)
pub fn average_edge_length_impl(m: &CGALMesh) -> f64 {
    let mut total = 0.0f64;
    let mut count: usize = 0;
    for e in m.edges() {
        let h = m.halfedge_e(e);
        let p0 = m.point(m.source(h));
        let p1 = m.point(m.target(h));
        total += squared_distance(&p0, &p1).sqrt();
        count += 1;
    }
    if count > 0 {
        total / count as f64
    } else {
        1.0
    }
}

/// `Compute_squared_area_3` of the kernel: `squared_areaC3` of
/// CGAL/constructions/kernel_ftC3.h
fn squared_area(p: &Point_3, q: &Point_3, r: &Point_3) -> f64 {
    // Compute vectors pq and pr, then the cross product,
    // then 1/4 of its squared length.
    let dqx = q[0] - p[0];
    let dqy = q[1] - p[1];
    let dqz = q[2] - p[2];
    let drx = r[0] - p[0];
    let dry = r[1] - p[1];
    let drz = r[2] - p[2];

    let vx = dqy.mul_add(drz, -(dqz * dry));
    let vy = dqz.mul_add(drx, -(dqx * drz));
    let vz = dqx.mul_add(dry, -(dqy * drx));

    let sx = vx * vx;
    let sy = vy * vy;
    let sz = vz * vz;
    (sx + sy + sz) / 4.0
}

/// `PMP::face_area(f, tmesh)` (CGAL/Polygon_mesh_processing/measure.h): the square root of the
/// squared area of `source(hd)`, `target(hd)`, `target(next(hd))`, `hd` the halfedge of the face
fn face_area(f: SM_Face_index, tmesh: &CGALMesh) -> f64 {
    let hd = tmesh.halfedge_f(f);
    let nhd = tmesh.next(hd);
    squared_area(&tmesh.point(tmesh.source(hd)), &tmesh.point(tmesh.target(hd)), &tmesh.point(tmesh.target(nhd))).sqrt()
}

/// ColorUtils.cpp:85-87
#[inline]
fn convert_rgb_uint_to_rgb_double(color: &Color) -> ColorDouble {
    [color[0] as f64, color[1] as f64, color[2] as f64]
}

/// ColorUtils.cpp:89-94. `vec.squared_length()` is `square(x) + square(y) + square(z)`, and
/// `vec /= c` divides each coordinate.
pub fn normalize(vec: &mut Vector_3) {
    let sx = vec[0] * vec[0];
    let sy = vec[1] * vec[1];
    let sz = vec[2] * vec[2];
    let squared_length = sx + sy + sz;
    if squared_length > EPSILON {
        let d = squared_length.sqrt();
        *vec = [vec[0] / d, vec[1] / d, vec[2] / d];
    }
}

/// ColorUtils.cpp:96-107: the angle between the two vectors, in degrees
pub fn get_angle_between_vectors(v1: &Vector_3, v2: &Vector_3) -> f64 {
    let mut dir1 = *v1;
    let mut dir2 = *v2;
    normalize(&mut dir1);
    normalize(&mut dir2);
    let product_dot = dir1[2].mul_add(dir2[2], dir1[0].mul_add(dir2[0], dir1[1] * dir2[1]));
    if product_dot > 1.0 - EPSILON {
        return 0.0;
    } else if product_dot < -1.0 + EPSILON {
        return 180.0;
    }
    product_dot.acos() / M_PI * 180.0
}

/// ColorUtils.cpp:109-124: per face `(pos3 - pos2).cross(pos1 - pos2)`, normalized. Measured
/// against the Eigen of the reference build: each component of the cross product is one fused
/// operation, the squared norm of `normalize()` is `x*x + y*y + z*z` without one, and a vector of
/// squared norm 0 is left as it is.
pub fn calc_face_normals(mesh: &CGALMesh, face_normals: &mut Vec<Vec3d>) {
    let fcnt = mesh.number_of_faces();
    face_normals.resize(fcnt, Vec3d::new(0.0, 0.0, 0.0));
    for face in mesh.faces() {
        let mut points: Vec<Point_3> = Vec::new();
        for vtx in mesh.vertices_around_face(mesh.halfedge_f(face)) {
            points.push(mesh.point(vtx));
        }
        let pos1 = points[0];
        let pos2 = points[1];
        let pos3 = points[2];
        let a = [pos3[0] - pos2[0], pos3[1] - pos2[1], pos3[2] - pos2[2]];
        let b = [pos1[0] - pos2[0], pos1[1] - pos2[1], pos1[2] - pos2[2]];
        let cx = a[1].mul_add(b[2], -(a[2] * b[1]));
        let cy = a[2].mul_add(b[0], -(a[0] * b[2]));
        let cz = a[0].mul_add(b[1], -(a[1] * b[0]));
        // face_normals[face].normalize();
        let (xx, yy, zz) = (cx * cx, cy * cy, cz * cz);
        let z = xx + yy + zz;
        if z > 0.0 {
            let d = z.sqrt();
            face_normals[face.idx()] = Vec3d::new(cx / d, cy / d, cz / d);
        } else {
            face_normals[face.idx()] = Vec3d::new(cx, cy, cz);
        }
    }
}

/// ColorUtils.cpp:126-142: true when the mesh does not intersect itself, or did and was
/// repaired. `is_self_intersect_status`, when given, is told whether it did. The two CGAL
/// algorithms come through `pmp` (see `cgal_utils::PolygonMeshProcessing`).
pub fn check_and_repair_self_intersect(mesh: &mut CGALMesh, is_self_intersect_status: Option<&mut bool>, pmp: &dyn PolygonMeshProcessing) -> bool {
    // true means the mesh is no self intersect now
    // false means the mesh is still self intersect
    let is_self_intersect = pmp.does_self_intersect(mesh);
    if let Some(status) = is_self_intersect_status {
        *status = is_self_intersect;
    }
    if is_self_intersect {
        let repair = pmp.remove_self_intersections(mesh);
        if repair {
            return true;
        } else {
            return false;
        }
    }
    true
}

/// `operator<<(std::ostream&, double)` with the stream's defaults: `%g`, six significant digits
pub(crate) fn ostream_double(v: f64) -> String {
    if v.is_nan() {
        return "nan".to_string();
    }
    if v.is_infinite() {
        return if v < 0.0 { "-inf".to_string() } else { "inf".to_string() };
    }
    if v == 0.0 {
        return if v.is_sign_negative() { "-0".to_string() } else { "0".to_string() };
    }
    let sci = format!("{:.5e}", v);
    let (mantissa, exp) = sci.split_once('e').expect("an exponent");
    let x: i32 = exp.parse().expect("an integer exponent");
    if x < -4 || x >= 6 {
        let mut m = mantissa.to_string();
        if m.contains('.') {
            m = m.trim_end_matches('0').trim_end_matches('.').to_string();
        }
        format!("{}e{}{:02}", m, if x < 0 { '-' } else { '+' }, x.abs())
    } else {
        let s = format!("{:.*}", (5 - x) as usize, v);
        if s.contains('.') {
            s.trim_end_matches('0').trim_end_matches('.').to_string()
        } else {
            s
        }
    }
}

/// ColorUtils.cpp:144-165: the polylines as `v` and `l` lines of an OBJ file. As in C++ the
/// result is true whether or not the file could be written.
pub fn save_polylines(file_name: &str, polylines: &[Vec<Point_3>]) -> bool {
    let mut points: Vec<Point_3> = Vec::new();
    let mut lines: Vec<(usize, usize)> = Vec::new();
    for polyline in polylines.iter() {
        let begin_pt_idx = points.len();
        for pt in polyline.iter() {
            points.push(*pt);
        }
        for i in 1..polyline.len() {
            lines.push((begin_pt_idx + i, begin_pt_idx + i + 1)); // obj is begin at 1
        }
    }
    let mut output = String::new();
    for point in points.iter() {
        output += &format!("v {} {} {}\n", ostream_double(point[0]), ostream_double(point[1]), ostream_double(point[2]));
    }
    for line in lines.iter() {
        output += &format!("l {} {}\n", line.0, line.1);
    }
    let _ = std::fs::write(file_name, output);
    true
}

/// ColorUtils.cpp:167-203: topological smoothing, a face takes the label most of its neighbours
/// have when more of them have it than have the face's own. `max_iters` is 20 by default in C++.
pub fn smooth_region_topo_boundary(mesh: &mut CGALMesh, face_labels: &mut Vec<usize>, max_iters: usize) -> bool {
    // Topological smoothing: reassign face labels
    let mut iter: usize = 0;
    while iter < max_iters {
        iter += 1;
        let mut flip_flag = false;
        for face in mesh.faces() {
            let mut same_label_count: usize = 0;
            let mut map_label_to_cnt: HashMap<usize, usize> = HashMap::new();
            let mut max_adj_cnt: usize = 0;
            let mut max_adj_label: usize = face_labels[face.idx()];
            for adj_face in mesh.faces_around_face(mesh.halfedge_f(face)) {
                if adj_face == CGALMesh::null_face() || !mesh.is_valid_f(adj_face) || mesh.is_removed_f(adj_face) {
                    continue;
                }
                if face_labels[adj_face.idx()] == face_labels[face.idx()] {
                    same_label_count += 1;
                } else {
                    let cnt = map_label_to_cnt.entry(face_labels[adj_face.idx()]).or_insert(0);
                    *cnt += 1;
                    if *cnt > max_adj_cnt {
                        max_adj_cnt = *cnt;
                        max_adj_label = face_labels[adj_face.idx()];
                    }
                }
            }
            if max_adj_cnt > same_label_count {
                face_labels[face.idx()] = max_adj_label;
                flip_flag = true;
            }
        }

        if !flip_flag {
            break;
        }
    }
    true
}

/// ColorUtils.cpp:205-351: geometric smoothing. The edges between faces of different labels are
/// chained into polylines; five times over, every vertex of a polyline that is not on a sharp
/// edge (45 degrees) is moved toward the middle of its two neighbours on the polyline and put
/// back on the surface; at the end every vertex of a polyline is put on the surface once more.
///
/// The polylines are found in the order `std::unordered_map` / `std::unordered_set` iterate
/// their vertices and edges, and a vertex moved earlier is read by the ones after it, so the
/// result depends on that order: `LibcxxUnorderedMap` gives libc++'s (the hash of a CGAL index is
/// the index). `boundary_tree` is `cgal_utils::Tree`, the port of CGAL's AABB tree: its boxes
/// stay those of the mesh before the first vertex is moved, as in CGAL.
pub fn smooth_region_geom_boundary(mesh: &mut CGALMesh, face_labels: &mut Vec<usize>, smooth_parameters: &SmoothParameters) -> bool {
    // 1. extract boundary vertices and make polines
    let mut map_vtx_to_degree: LibcxxUnorderedMap<usize> = LibcxxUnorderedMap::new();
    let mut segment_boundary_edges: LibcxxUnorderedMap<()> = LibcxxUnorderedMap::new();
    let mut feature_edges: HashSet<SM_Edge_index> = HashSet::new();
    let mut feature_vertices: HashSet<SM_Vertex_index> = HashSet::new();
    const FEATURE_ANGLE: f64 = 45.0;
    for edge in mesh.edges() {
        if !mesh.is_valid_e(edge) || mesh.is_border_e(edge) {
            continue;
        }
        let source = mesh.source(mesh.halfedge_e(edge));
        let target = mesh.target(mesh.halfedge_e(edge));
        let face_1 = mesh.face(mesh.halfedge_e(edge));
        let face_2 = mesh.face(mesh.opposite(mesh.halfedge_e(edge)));
        let normal_1 = compute_face_normal(face_1, mesh);
        let normal_2 = compute_face_normal(face_2, mesh);
        let angle = get_angle_between_vectors(&normal_1, &normal_2);
        if angle > FEATURE_ANGLE {
            feature_edges.insert(edge);
            feature_vertices.insert(source);
            feature_vertices.insert(target);
        }

        if face_labels[face_1.idx()] == face_labels[face_2.idx()] {
            continue;
        }

        segment_boundary_edges.insert(edge.idx());
        *map_vtx_to_degree.entry(source.idx()) += 1;
        *map_vtx_to_degree.entry(target.idx()) += 1;
    }

    // 2. smooth each polyline
    let mut polylines: Vec<Vec<SM_Vertex_index>> = Vec::new();
    let mut visited_edges: HashSet<usize> = HashSet::new();

    // Walk degree-2 chains iteratively.
    fn trace_polyline(
        mesh: &CGALMesh,
        map_vtx_to_degree: &mut LibcxxUnorderedMap<usize>,
        segment_boundary_edges: &LibcxxUnorderedMap<()>,
        visited_edges: &mut HashSet<usize>,
        polyline: &mut Vec<SM_Vertex_index>,
    ) {
        while !polyline.is_empty() {
            let curr_vtx = *polyline.last().expect("not empty");
            if *map_vtx_to_degree.entry(curr_vtx.idx()) != 2 {
                return;
            }
            let mut extended = false;
            for halfedge in mesh.halfedges_around_target(mesh.halfedge_v(curr_vtx)) {
                let edge = mesh.edge(halfedge);
                if visited_edges.contains(&edge.idx()) || segment_boundary_edges.count(edge.idx()) == 0 {
                    continue;
                }
                visited_edges.insert(edge.idx());
                let adj_vtx = mesh.source(halfedge);
                if map_vtx_to_degree.count(adj_vtx.idx()) == 0 {
                    continue;
                }
                polyline.push(adj_vtx);
                extended = true;
                break;
            }
            if !extended {
                return;
            }
        }
    }

    // 2.1. open polyline: from T nodes search other 2-degree nodes
    for (src_vtx, degree) in map_vtx_to_degree.items() {
        if degree == 2 {
            continue;
        }
        let src_vtx = SM_Vertex_index(src_vtx as u32);
        for src_halfedge in mesh.halfedges_around_target(mesh.halfedge_v(src_vtx)) {
            let src_edge = mesh.edge(src_halfedge);
            if visited_edges.contains(&src_edge.idx()) || segment_boundary_edges.count(src_edge.idx()) == 0 {
                continue;
            }
            visited_edges.insert(src_edge.idx());
            let adj_vtx = mesh.source(src_halfedge);
            if map_vtx_to_degree.count(adj_vtx.idx()) == 0 {
                continue;
            }
            let mut polyline: Vec<SM_Vertex_index> = vec![src_vtx, adj_vtx];
            trace_polyline(mesh, &mut map_vtx_to_degree, &segment_boundary_edges, &mut visited_edges, &mut polyline);
            polylines.push(polyline);
        }
    }

    // 2.2. closed polylines
    for edge in segment_boundary_edges.keys() {
        if visited_edges.contains(&edge) {
            continue;
        }
        visited_edges.insert(edge);
        let halfedge = mesh.halfedge_e(SM_Edge_index::new(edge as u32));
        let mut polyline: Vec<SM_Vertex_index> = vec![mesh.source(halfedge), mesh.target(halfedge)];
        trace_polyline(mesh, &mut map_vtx_to_degree, &segment_boundary_edges, &mut visited_edges, &mut polyline);
        // C++ logs "loop polyline but not closed" when the two ends differ
        polylines.push(polyline);
    }

    // 3. smooth boundary
    let mut boundary_tree = Tree::new(mesh);
    boundary_tree.accelerate_distance_queries(mesh);

    const MAX_ITERS: usize = 5;
    let smooth_weight = smooth_parameters.smooth_weight; // Controls smoothing intensity; larger values produce smoother results. Range: 0.1~1.0.
    // std::max(1.0 - smooth_weight, 0.0)
    let origin_weight = if 1.0 - smooth_weight < 0.0 { 0.0 } else { 1.0 - smooth_weight };
    // CGAL::ORIGIN + ((CGAL::centroid(pts) - CGAL::ORIGIN) * smooth_weight + (point - CGAL::ORIGIN) * origin_weight):
    // the products are made in `Construct_scaled_vector_3`, the sum in `VectorC3::operator+`
    let weighted = |c: &Point_3, p: &Point_3| -> Point_3 {
        let a = [smooth_weight * c[0], smooth_weight * c[1], smooth_weight * c[2]];
        let b = [origin_weight * p[0], origin_weight * p[1], origin_weight * p[2]];
        [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
    };
    for _iter in 0..MAX_ITERS {
        for polyline in polylines.iter() {
            let pt_cnt = polyline.len();
            let mut points: Vec<Point_3> = vec![[0.0; 3]; pt_cnt];
            let mut pt_idx = 1usize;
            while pt_idx + 1 < pt_cnt {
                let pts = [mesh.point(polyline[pt_idx - 1]), mesh.point(polyline[pt_idx + 1])];
                let smooth_pt = weighted(&centroid(&pts), &mesh.point(polyline[pt_idx]));
                points[pt_idx] = boundary_tree.closest_point(mesh, &smooth_pt);
                pt_idx += 1;
            }

            if polyline[0] == polyline[pt_cnt - 1] {
                if feature_vertices.contains(&polyline[0]) {
                    continue;
                }
                let pts = [mesh.point(polyline[1]), mesh.point(polyline[pt_cnt - 2])];
                let smooth_pt = weighted(&centroid(&pts), &mesh.point(polyline[0]));
                let moved = boundary_tree.closest_point(mesh, &smooth_pt);
                *mesh.point_mut(polyline[0]) = moved;
            }

            let mut pt_idx = 1usize;
            while pt_idx + 1 < pt_cnt {
                if !feature_vertices.contains(&polyline[pt_idx]) {
                    *mesh.point_mut(polyline[pt_idx]) = points[pt_idx];
                }
                pt_idx += 1;
            }
        }
    }

    for polyline in polylines.iter() {
        for &vtx in polyline.iter() {
            let moved = boundary_tree.closest_point(mesh, &mesh.point(vtx));
            *mesh.point_mut(vtx) = moved;
        }
    }

    true
}

// HSV, XYZ, and LAB are only used internally for computing color differences.
/// ColorUtils.cpp:354
pub type HSV = [f64; 3];
/// ColorUtils.cpp:355, intermediate space for converting between LAB and RGB
pub type XYZ = [f64; 3];
/// ColorUtils.cpp:356
pub type LAB = [f64; 3];
/// ColorUtils.cpp:357-358, common white points
pub const D65_WHITE: XYZ = [0.95047, 1.0, 1.08883];

/// ColorUtils.cpp:366-400: RGB 0~255 to HSV, H in 0~360, S and V in 0~1
pub fn convert_rgb_to_hsv(rgb: &RGB) -> HSV {
    // Normalize to [0, 1]
    let r = rgb[0] as f64 / 255.0;
    let g = rgb[1] as f64 / 255.0;
    let b = rgb[2] as f64 / 255.0;

    // std::max({r, g, b}) / std::min({r, g, b})
    let mut max = r;
    if max < g {
        max = g;
    }
    if max < b {
        max = b;
    }
    let mut min = r;
    if g < min {
        min = g;
    }
    if b < min {
        min = b;
    }
    let delta = max - min;

    // Compute hue H
    let mut h = 0.0f64;
    if delta == 0.0 {
        h = 0.0; // Gray; hue is undefined
    } else {
        if max == r {
            h = 60.0 * (((g - b) / delta) % 6.0);
        } else if max == g {
            h = 60.0 * ((b - r) / delta + 2.0);
        } else {
            // max == b
            h = 60.0 * ((r - g) / delta + 4.0);
        }
        if h < 0.0 {
            h += 360.0;
        }
    }

    // Compute saturation S
    let s = if max == 0.0 { 0.0 } else { delta / max };

    // Compute value V
    let v = max;

    [h, s, v]
}

/// ColorUtils.cpp:408-446: HSV to RGB 0~255
pub fn convert_hsv_to_rgb(hsv: &HSV) -> RGB {
    let h = hsv[0];
    let s = hsv[1];
    let v = hsv[2];

    let c = v * s;
    let x = c * (1.0 - (((h / 60.0) % 2.0) - 1.0).abs());
    let m = v - c;

    let (r, g, b);

    if h < 60.0 {
        r = c;
        g = x;
        b = 0.0;
    } else if h < 120.0 {
        r = x;
        g = c;
        b = 0.0;
    } else if h < 180.0 {
        r = 0.0;
        g = c;
        b = x;
    } else if h < 240.0 {
        r = 0.0;
        g = x;
        b = c;
    } else if h < 300.0 {
        r = x;
        g = 0.0;
        b = c;
    } else {
        r = c;
        g = 0.0;
        b = x;
    }

    [(r + m).mul_add(255.0, 0.5) as usize, (g + m).mul_add(255.0, 0.5) as usize, (b + m).mul_add(255.0, 0.5) as usize]
}

/// The rows of the sRGB to XYZ matrix (ColorUtils.cpp:464, 476): each is
/// `r * a + g * b + b * c`, two fused operations.
#[inline]
fn linear_rgb_to_xyz(r: f64, g: f64, b: f64) -> XYZ {
    [
        b.mul_add(0.1804375, r.mul_add(0.4124564, g * 0.3575761)),
        b.mul_add(0.0721750, r.mul_add(0.2126729, g * 0.7151522)),
        b.mul_add(0.9503041, r.mul_add(0.0193339, g * 0.1191920)),
    ]
}

/// ColorUtils.cpp:448-465
pub fn convert_rgb_to_xyz(color_rgb: &RGB) -> XYZ {
    let rgb: ColorDouble = [color_rgb[0] as f64, color_rgb[1] as f64, color_rgb[2] as f64];
    let gamma_correct = |v: f64| -> f64 {
        let v = v / 255.0;
        if v > 0.04045 {
            ((v + 0.055) / 1.055).powf(2.4)
        } else {
            v / 12.92
        }
    };

    let r = gamma_correct(rgb[0]);
    let g = gamma_correct(rgb[1]);
    let b = gamma_correct(rgb[2]);

    // sRGB to XYZ matrix
    linear_rgb_to_xyz(r, g, b)
}

/// ColorUtils.cpp:467-477: sRGB non-linear channel values [0,1] (double) -> linear light -> XYZ;
/// equivalent to convert_rgb_to_xyz when v=n/255
pub fn convert_srgb01_to_xyz(rs: f64, gs: f64, bs: f64) -> XYZ {
    let gamma_correct = |v: f64| -> f64 {
        // std::clamp(v, 0.0, 1.0)
        let v = if v < 0.0 {
            0.0
        } else if 1.0 < v {
            1.0
        } else {
            v
        };
        if v > 0.04045 {
            ((v + 0.055) / 1.055).powf(2.4)
        } else {
            v / 12.92
        }
    };
    let r = gamma_correct(rs);
    let g = gamma_correct(gs);
    let b = gamma_correct(bs);
    linear_rgb_to_xyz(r, g, b)
}

/// ColorUtils.cpp:479-497
pub fn convert_xyz_to_lab(xyz: &XYZ) -> LAB {
    let f = |t: f64| -> f64 {
        let delta: f64 = 6.0 / 29.0;
        if t > delta * delta * delta {
            t.cbrt()
        } else {
            t / (3.0 * delta * delta) + 4.0 / 29.0
        }
    };

    // D65 white point
    let xn = D65_WHITE[0];
    let yn = D65_WHITE[1];
    let zn = D65_WHITE[2];

    let fx = f(xyz[0] / xn);
    let fy = f(xyz[1] / yn);
    let fz = f(xyz[2] / zn);

    [116.0f64.mul_add(fy, -16.0), 500.0 * (fx - fy), 200.0 * (fy - fz)]
}

/// ColorUtils.cpp:499-502. ColorDouble here represents sRGB [0,1].
pub fn convert_srgb01_to_lab(srgb01: &ColorDouble) -> LAB {
    convert_xyz_to_lab(&convert_srgb01_to_xyz(srgb01[0], srgb01[1], srgb01[2]))
}

/// ColorUtils.cpp:504-506
pub fn convert_rgb_to_lab(rgb: &RGB) -> LAB {
    convert_xyz_to_lab(&convert_rgb_to_xyz(rgb))
}

/// ColorUtils.cpp:508-535
pub fn convert_lab_to_rgb(lab: &LAB) -> Color {
    // Lab → XYZ
    let delta: f64 = 6.0 / 29.0;
    let delta2x3: f64 = 3.0 * delta * delta;

    let fy = (lab[0] + 16.0) / 116.0;
    let fx = lab[1] / 500.0 + fy;
    let fz = fy - lab[2] / 200.0;

    let x = D65_WHITE[0] * (if fx > delta { fx * fx * fx } else { delta2x3 * (fx - 4.0 / 29.0) });
    let y = D65_WHITE[1] * (if fy > delta { fy * fy * fy } else { delta2x3 * (fy - 4.0 / 29.0) });
    let z = D65_WHITE[2] * (if fz > delta { fz * fz * fz } else { delta2x3 * (fz - 4.0 / 29.0) });

    // XYZ -> linear RGB (sRGB inverse matrix)
    let r_lin = (-0.4985314f64).mul_add(z, 3.2404542f64.mul_add(x, -(1.5371385 * y)));
    let g_lin = 0.0415560f64.mul_add(z, (-0.9692660f64).mul_add(x, 1.8760108 * y));
    let b_lin = 1.0572252f64.mul_add(z, 0.0556434f64.mul_add(x, -(0.2040259 * y)));

    // linear RGB -> sRGB (inverse gamma correction)
    let inverse_gamma = |v: f64| -> f64 {
        // std::max(v, 0.0)
        let v = if v < 0.0 { 0.0 } else { v };
        if v <= 0.0031308 {
            12.92 * v
        } else {
            1.055f64.mul_add(v.powf(1.0 / 2.4), -0.055)
        }
    };

    let to_uint8 = |v: f64| -> usize {
        // std::clamp(std::round(v * 255.0), 0.0, 255.0)
        let r = (v * 255.0).round();
        let c = if r < 0.0 {
            0.0
        } else if 255.0 < r {
            255.0
        } else {
            r
        };
        c as usize
    };

    [to_uint8(inverse_gamma(r_lin)), to_uint8(inverse_gamma(g_lin)), to_uint8(inverse_gamma(b_lin))]
}

/// ColorUtils.cpp:546-627: the CIEDE2000 color difference of two LAB colors
pub fn ciede2000(lab1: &[f64; 3], lab2: &[f64; 3]) -> f64 {
    // Parameters in the CIE L*C*h* formula
    let (l1, a1, b1) = (lab1[0], lab1[1], lab1[2]);
    let (l2, a2, b2) = (lab2[0], lab2[1], lab2[2]);

    // Compute C1 and C2
    let c1 = a1.mul_add(a1, b1 * b1).sqrt();
    let c2 = a2.mul_add(a2, b2 * b2).sqrt();
    let c_avg = (c1 + c2) / 2.0;

    // G factor (compensates for non-linearity in the mid-low chroma region)
    let c7 = c_avg * c_avg * c_avg * c_avg * c_avg * c_avg * c_avg;
    let g = 0.5 * (1.0 - (c7 / (c7 + 6103515625.0)).sqrt());

    // a1' and a2'
    let a1_prime = (1.0 + g) * a1;
    let a2_prime = (1.0 + g) * a2;

    // C'1 and C'2
    let c1_prime = a1_prime.mul_add(a1_prime, b1 * b1).sqrt();
    let c2_prime = a2_prime.mul_add(a2_prime, b2 * b2).sqrt();
    let c_prime_avg = (c1_prime + c2_prime) / 2.0;

    // h'1 and h'2
    let mut h1_prime = b1.atan2(a1_prime);
    let mut h2_prime = b2.atan2(a2_prime);
    if h1_prime < 0.0 {
        h1_prime += 2.0 * M_PI;
    }
    if h2_prime < 0.0 {
        h2_prime += 2.0 * M_PI;
    }

    // Compute dh'
    let dh_prime;
    if (h1_prime - h2_prime).abs() <= M_PI {
        dh_prime = h2_prime - h1_prime;
    } else if h2_prime <= h1_prime {
        dh_prime = h2_prime - h1_prime + 2.0 * M_PI;
    } else {
        dh_prime = h2_prime - h1_prime - 2.0 * M_PI;
    }

    // Compute dH'
    let d_h_prime = 2.0 * (c1_prime * c2_prime).sqrt() * (dh_prime / 2.0).sin();

    // Compute dL'
    let d_l_prime = l2 - l1;

    // Compute dC'
    let d_c_prime = c2_prime - c1_prime;

    // Compute h_prime_avg
    let h_prime_avg;
    if (h1_prime - h2_prime).abs() > M_PI {
        h_prime_avg = (h1_prime + h2_prime + 2.0 * M_PI) / 2.0;
    } else {
        h_prime_avg = (h1_prime + h2_prime) / 2.0;
    }

    // Compute T
    let cos1 = (h_prime_avg - M_PI / 6.0).cos();
    let cos2 = (2.0 * h_prime_avg).cos();
    let cos3 = 3.0f64.mul_add(h_prime_avg, M_PI / 30.0).cos();
    let cos4 = 4.0f64.mul_add(h_prime_avg, -(3.0 * M_PI / 6.0)).cos();
    let t = (-0.20f64).mul_add(cos4, 0.32f64.mul_add(cos3, 0.24f64.mul_add(cos2, (-0.17f64).mul_add(cos1, 1.0))));

    // Compute rotation term R_T = -R_C * sin(2*delta_theta), where delta_theta = 30 * exp(-((h_bar'-275)/25)^2)
    // h_prime_avg is in radians; convert to degrees for delta_theta; 2*delta_theta = 60 * exp(...), convert back to radians for sin
    let h_prime_avg_deg = h_prime_avg * 180.0 / M_PI;
    let c_prime_avg_7 = c_prime_avg.powf(7.0);
    let u = (h_prime_avg_deg - 275.0) / 25.0;
    let r = -2.0 * (c_prime_avg_7 / (c_prime_avg_7 + 6103515625.0)).sqrt() * ((60.0 * M_PI / 180.0) * (-(u * u)).exp()).sin();

    // Compute SL, SC, SH
    let l_prime_avg = (l1 + l2) / 2.0;
    let dl50 = l_prime_avg - 50.0;
    let p50 = dl50 * dl50;
    let sl = 1.0 + 0.015 * p50 / (20.0 + p50).sqrt();
    let sc = 0.045f64.mul_add(c_prime_avg, 1.0);
    let sh = (0.015 * c_prime_avg).mul_add(t, 1.0);

    // Final color difference
    let ql = d_l_prime / sl;
    let qc = d_c_prime / sc;
    let qh = d_h_prime / sh;
    let pl = ql * ql;
    let pc = qc * qc;
    let ph = qh * qh;
    (r * (d_c_prime / sc)).mul_add(d_h_prime / sh, pl + pc + ph).sqrt()
}

/// ColorUtils.cpp:629-633: the CIEDE2000 difference of two RGB colors, 0~255
pub fn calc_rgb_color_difference_by_ciede2000(rgb1: &RGB, rgb2: &RGB) -> f64 {
    let lab1 = convert_rgb_to_lab(rgb1);
    let lab2 = convert_rgb_to_lab(rgb2);
    ciede2000(&lab1, &lab2)
}

/// ColorUtils.cpp:635-639: the CIEDE2000 difference of two sRGB colors with channels in [0,1]
pub fn calc_rgb_color_difference_by_ciede2000_srgb01(rgb1: &ColorDouble, rgb2: &ColorDouble) -> f64 {
    let lab1 = convert_srgb01_to_lab(rgb1);
    let lab2 = convert_srgb01_to_lab(rgb2);
    ciede2000(&lab1, &lab2)
}

/// ColorUtils.cpp:641-642: working-space distance function type: inputs are two colors in the
/// same space (RGB-double or Lab)
pub type WorkingDistFunc = fn(&ColorDouble, &ColorDouble) -> f64;

/// ColorUtils.cpp:644-697: Farthest Point Sampling (FPS) initialization. The first center is the
/// point nearest to the global centroid; subsequent centers are the points farthest from the
/// existing set.
pub fn farthest_point_sampling_init(working_colors: &[ColorDouble], k: usize, dist_func: WorkingDistFunc) -> Vec<ColorDouble> {
    let mut centers: Vec<ColorDouble> = vec![[0.0; 3]; k];

    // 1. Compute the global centroid and pick the nearest point as the first center
    let mut centroid: ColorDouble = [0.0, 0.0, 0.0];
    for c in working_colors.iter() {
        centroid[0] += c[0];
        centroid[1] += c[1];
        centroid[2] += c[2];
    }
    let n = working_colors.len() as f64;
    centroid[0] /= n;
    centroid[1] /= n;
    centroid[2] /= n;

    let mut best_dist = f64::MAX;
    let mut first_idx: usize = 0;
    for i in 0..working_colors.len() {
        let d = dist_func(&working_colors[i], &centroid);
        if d < best_dist {
            best_dist = d;
            first_idx = i;
        }
    }
    centers[0] = working_colors[first_idx];

    // Minimum distance from each point to the already-selected center set
    let mut min_distances: Vec<f64> = vec![f64::MAX; working_colors.len()];

    // 2. Greedily select the remaining K-1 centers: pick the point with the largest min_distance each time
    for i in 1..k {
        let last_center = centers[i - 1];

        // Update each point's minimum distance with the newly added center
        let mut farthest_dist = -1.0f64;
        let mut farthest_idx: usize = 0;
        for c_idx in 0..working_colors.len() {
            let d = dist_func(&working_colors[c_idx], &last_center);
            if d < min_distances[c_idx] {
                min_distances[c_idx] = d;
            }
            if min_distances[c_idx] > farthest_dist {
                farthest_dist = min_distances[c_idx];
                farthest_idx = c_idx;
            }
        }

        centers[i] = working_colors[farthest_idx];
    }

    centers
}

/// ColorUtils.cpp:699-839: the mesh remeshed to edges of `target_edge_length_ratio` times the
/// average length, the edges between faces of different labels and the border edges kept; every
/// new face takes the label of the old face nearest to its centroid.
///
/// The CGAL algorithms (`PMP::isotropic_remeshing`, `PMP::does_self_intersect`,
/// `PMP::experimental::remove_self_intersections`) come through `pmp` (see
/// `cgal_utils::PolygonMeshProcessing`); the rest is the port.
///
/// FIDELITY-NOTE: the labels are looked up with the crate's `aabb_tree_indirect`, which works in
/// doubles; C++ instantiates `AABBTreeIndirect` with `Vec3f` here. Where two old faces are at
/// distances from a centroid that differ only below float precision, the face found can differ.
pub fn remesh_mesh(bbs_mesh: &mut TriMesh, face_labels: &mut Vec<usize>, target_edge_length_ratio: f64, pmp: &dyn PolygonMeshProcessing) -> bool {
    if face_labels.len() != bbs_mesh.indices.len() {
        return false;
    }

    // Back up face labels for recovery after remeshing.
    let face_labels_of_original_mesh: Vec<usize> = face_labels.clone();

    let mut cgal_mesh = CGALMesh::new();
    cgal_utils::convert_trimesh_to_cgal(bbs_mesh, &mut cgal_mesh);

    // AABBTreeIndirect references vertices/faces externally, so snapshot the
    // pre-remesh geometry by moving it out of bbs_mesh (which is overwritten
    // below with the post-remesh mesh).
    let old_vertices: TriVertices = std::mem::take(&mut bbs_mesh.vertices);
    let old_indices: TriFaces = std::mem::take(&mut bbs_mesh.indices);
    let tree_vertices: Vec<Point3F> = old_vertices.iter().map(|v| Point3F::new(v.x as f64, v.y as f64, v.z as f64)).collect();
    let tree_faces: Vec<[usize; 3]> = old_indices.iter().map(|f| [f[0] as usize, f[1] as usize, f[2] as usize]).collect();
    let original_mesh_tree = aabb_tree_indirect::build_aabb_tree_over_indexed_triangle_set(&tree_vertices, &tree_faces);

    let mut feature_edges: HashSet<SM_Edge_index> = HashSet::new();
    let mut feature_vertices: HashSet<SM_Vertex_index> = HashSet::new();
    // the property maps "constrained_edges" and "constrained_vertices", false by default
    let mut constrained_edges: Vec<bool> = vec![false; cgal_mesh.num_edges()];
    let mut constrained_vertices: Vec<bool> = vec![false; cgal_mesh.num_vertices()];

    // An edge is considered a geometric feature edge if its dihedral angle is less than 135 degrees (loose threshold)
    #[allow(dead_code)]
    const FEATURE_ANGLE: f64 = 135.0;

    let is_feature_edge = |cgal_mesh: &CGALMesh, edge: SM_Edge_index| -> bool {
        if cgal_mesh.is_border_e(edge) {
            return true;
        }
        let halfedge_1 = cgal_mesh.halfedge_e(edge);
        let halfedge_2 = cgal_mesh.opposite(halfedge_1);
        let face_1 = cgal_mesh.face(halfedge_1);
        let face_2 = cgal_mesh.face(halfedge_2);
        if face_labels_of_original_mesh[face_1.idx()] != face_labels_of_original_mesh[face_2.idx()] {
            // Boundary between different color regions; treated as a feature edge
            return true;
        }
        // TODO (C++): CGAL remeshing tends to crash when too many constrained edges are added; needs handling
        false
    };

    for edge in cgal_mesh.edges() {
        if is_feature_edge(&cgal_mesh, edge) {
            feature_edges.insert(edge);
            feature_vertices.insert(cgal_mesh.source(cgal_mesh.halfedge_e(edge)));
            feature_vertices.insert(cgal_mesh.target(cgal_mesh.halfedge_e(edge)));
            constrained_edges[edge.idx()] = true;
            constrained_vertices[cgal_mesh.source(cgal_mesh.halfedge_e(edge)).idx()] = true;
            constrained_vertices[cgal_mesh.target(cgal_mesh.halfedge_e(edge)).idx()] = true;
        }
    }

    let iters: usize = 5;
    // TODO (C++): CGAL remeshing preserves geometric boundaries but does not maintain face labels well; colors need to be recomputed
    let target_edge_length = target_edge_length_ratio * average_edge_length_impl(&cgal_mesh);
    pmp.isotropic_remeshing(&mut cgal_mesh, target_edge_length, iters, true, &constrained_edges, &constrained_vertices, true);
    if pmp.does_self_intersect(&cgal_mesh) {
        pmp.remove_self_intersections(&mut cgal_mesh);
    }

    bbs_mesh.clear();

    let mut map_cgal_vtx_to_bbs_vtx: HashMap<SM_Vertex_index, usize> = HashMap::new();
    let mut bbs_vertices: TriVertices = Vec::new();
    let mut bbs_faces: TriFaces = Vec::new();
    face_labels.clear();
    face_labels.reserve(cgal_mesh.number_of_faces());

    for cgal_vtx in cgal_mesh.vertices() {
        if !cgal_mesh.is_valid_v(cgal_vtx) || cgal_mesh.is_removed_v(cgal_vtx) || cgal_mesh.is_isolated(cgal_vtx) {
            continue;
        }
        if !map_cgal_vtx_to_bbs_vtx.contains_key(&cgal_vtx) {
            map_cgal_vtx_to_bbs_vtx.insert(cgal_vtx, bbs_vertices.len());
            let cgal_point = cgal_mesh.point(cgal_vtx);
            bbs_vertices.push(TriVertex::new(cgal_point[0] as f32, cgal_point[1] as f32, cgal_point[2] as f32));
        }
    }

    for cgal_face in cgal_mesh.faces() {
        if !cgal_mesh.is_valid_f(cgal_face) || cgal_mesh.is_removed_f(cgal_face) {
            continue;
        }
        let mut bbs_face: TriFace = TriFace::new(0, 0, 0);
        let mut face_vid: usize = 0;
        for cgal_vtx in cgal_mesh.vertices_around_face(cgal_mesh.halfedge_f(cgal_face)) {
            if !map_cgal_vtx_to_bbs_vtx.contains_key(&cgal_vtx) {
                // "CGAL mesh contains a face with an invalid vertex"
                return false;
            }
            bbs_face[face_vid] = map_cgal_vtx_to_bbs_vtx[&cgal_vtx] as i32;
            face_vid += 1;
        }
        bbs_faces.push(bbs_face);
    }

    *bbs_mesh = TriMesh::new(bbs_faces, bbs_vertices);

    face_labels.resize(bbs_mesh.indices.len(), 0);
    for fid in 0..bbs_mesh.indices.len() {
        let face = bbs_mesh.indices[fid];
        // Vec3f face_centroid = (v0 + v1 + v2) / 3.0: Eigen makes the divisor a float
        let v0 = bbs_mesh.vertices[face[0] as usize];
        let v1 = bbs_mesh.vertices[face[1] as usize];
        let v2 = bbs_mesh.vertices[face[2] as usize];
        let sum = v0 + v1 + v2;
        let face_centroid = [sum.x / 3.0f32, sum.y / 3.0f32, sum.z / 3.0f32];
        let (_, hit_idx, _) = aabb_tree_indirect::squared_distance_to_indexed_triangle_set(
            &tree_vertices,
            &tree_faces,
            &original_mesh_tree,
            Vec3::new(face_centroid[0] as f64, face_centroid[1] as f64, face_centroid[2] as f64),
        );
        face_labels[fid] = face_labels_of_original_mesh[hit_idx];
    }

    true
}

/// ColorUtils.cpp:841-901: whether the mesh has no border edge
pub fn is_closed(bbs_mesh: &TriMesh) -> bool {
    let mut cgal_mesh = CGALMesh::new();
    cgal_utils::convert_trimesh_to_cgal(bbs_mesh, &mut cgal_mesh);

    cgal_utils::is_closed(&cgal_mesh)
}

/// ColorUtils.cpp:903-925: the labels smoothed across the faces (`smooth_weight` at least 0.3)
/// and the lines between them smoothed on the surface (`smooth_weight` above 1e-6)
pub fn smooth_region_labels(tri_mesh: &mut TriMesh, face_labels: &mut Vec<usize>, smooth_parameters: &SmoothParameters) -> bool {
    let mut mesh = CGALMesh::new();
    cgal_utils::convert_trimesh_to_cgal(tri_mesh, &mut mesh);

    if mesh.number_of_faces() != face_labels.len() {
        return false;
    }

    if smooth_parameters.smooth_weight >= TOPO_SMOOTH_WEIGHT_THRESHOLD {
        // Topological smoothing: reassign face labels.
        smooth_region_topo_boundary(&mut mesh, face_labels, 20);
    }

    if smooth_parameters.smooth_weight > EPSILON {
        // Geometric smoothing: smooth polylines and project back onto the original mesh.
        smooth_region_geom_boundary(&mut mesh, face_labels, smooth_parameters);
    }

    *tri_mesh = cgal_utils::cgal_to_trimesh(&mesh);

    true
}

/// ColorUtils.cpp:927-952, `smooth_region` over RGB colors: the colors numbered in the order
/// they come, the labels smoothed, the colors put back
pub fn smooth_region(tri_mesh: &mut TriMesh, face_colors: &mut Vec<[usize; 3]>, smooth_parameters: &SmoothParameters) -> bool {
    // Convert colors to labels
    let mut label_next: usize = 0;
    let mut face_labels: Vec<usize> = Vec::with_capacity(face_colors.len());
    let mut map_color_to_label: std::collections::BTreeMap<[usize; 3], usize> = std::collections::BTreeMap::new();
    let mut map_label_to_color: HashMap<usize, [usize; 3]> = HashMap::new();
    for color in face_colors.iter() {
        if !map_color_to_label.contains_key(color) {
            map_color_to_label.insert(*color, label_next);
            map_label_to_color.insert(label_next, *color);
            label_next += 1;
        }
        face_labels.push(map_color_to_label[color]);
    }

    if !smooth_region_labels(tri_mesh, &mut face_labels, smooth_parameters) {
        return false;
    }

    // Convert labels back to colors
    for fid in 0..face_labels.len() {
        face_colors[fid] = map_label_to_color[&face_labels[fid]];
    }

    true
}

/// ColorUtils.cpp:954-956, `smooth_region` over integer labels
pub fn smooth_region_with_labels(tri_mesh: &mut TriMesh, face_labels: &mut Vec<usize>, smooth_parameters: &SmoothParameters) -> bool {
    smooth_region_labels(tri_mesh, face_labels, smooth_parameters)
}

/// ColorUtils.cpp:958-964: the squared Euclidean distance of two colors
pub fn calc_rgb_color_difference_by_squared_rgb_double(c1: &ColorDouble, c2: &ColorDouble) -> f64 {
    let dr = c1[0] - c2[0];
    let dg = c1[1] - c2[1];
    let db = c1[2] - c2[2];
    db.mul_add(db, dr.mul_add(dr, dg * dg))
}

/// ColorUtils.cpp:966-971: the squared Euclidean distance of two colors
pub fn calc_rgb_color_difference_by_squared_rgb(c1: &RGB, c2: &RGB) -> f64 {
    let c1d = convert_rgb_uint_to_rgb_double(c1);
    let c2d = convert_rgb_uint_to_rgb_double(c2);
    calc_rgb_color_difference_by_squared_rgb_double(&c1d, &c2d)
}

/// ColorUtils.cpp:973-1048: K-Means core: FPS initialization + assign/update iterations in
/// working space, the cluster centers are returned
pub fn kmeans_core(
    working_colors: &[ColorDouble],
    k: usize,
    max_iter: usize,
    dist_func: WorkingDistFunc,
    cancel_cb: &Option<Arc<dyn Fn() -> bool + Send + Sync>>,
) -> Vec<ColorDouble> {
    let mut centers = farthest_point_sampling_init(working_colors, k, dist_func);
    let mut assignments: Vec<usize> = vec![0; working_colors.len()];

    for _iter in 0..max_iter {
        if let Some(cb) = cancel_cb {
            if cb() {
                return centers;
            }
        }
        let mut changed = false;

        for i in 0..working_colors.len() {
            let mut min_dist = f64::MAX;
            let mut best_cluster: usize = 0;

            for j in 0..k {
                let d = dist_func(&working_colors[i], &centers[j]);
                if d < min_dist {
                    min_dist = d;
                    best_cluster = j;
                }
            }

            if assignments[i] != best_cluster {
                changed = true;
                assignments[i] = best_cluster;
            }
        }

        if !changed {
            break;
        }

        let mut new_centers: Vec<ColorDouble> = vec![[0.0, 0.0, 0.0]; k];
        let mut counts: Vec<usize> = vec![0; k];

        for i in 0..working_colors.len() {
            let cluster_id = assignments[i];
            counts[cluster_id] += 1;
            new_centers[cluster_id][0] += working_colors[i][0];
            new_centers[cluster_id][1] += working_colors[i][1];
            new_centers[cluster_id][2] += working_colors[i][2];
        }

        for i in 0..k {
            if counts[i] == 0 {
                let mut max_min_dist = -1.0f64;
                let mut best_idx: usize = 0;
                for p in 0..working_colors.len() {
                    let mut nearest = f64::MAX;
                    for c in 0..k {
                        if c == i || counts[c] == 0 {
                            continue;
                        }
                        let d = dist_func(&working_colors[p], &centers[c]);
                        if d < nearest {
                            nearest = d;
                        }
                    }
                    if nearest > max_min_dist {
                        max_min_dist = nearest;
                        best_idx = p;
                    }
                }
                centers[i] = working_colors[best_idx];
            } else {
                centers[i][0] = new_centers[i][0] / counts[i] as f64;
                centers[i][1] = new_centers[i][1] / counts[i] as f64;
                centers[i][2] = new_centers[i][2] / counts[i] as f64;
            }
        }
    }

    centers
}

/// ColorUtils.cpp:1050-1111: K-Means clustering of the colors into `cluster_k` centers
pub fn cluster_k_means(colors: &[Color], cluster_parameters: &ClusterParameters) -> Vec<Color> {
    let mut k = cluster_parameters.cluster_k;
    let max_iter = cluster_parameters.max_iter;

    if k == 0 || colors.is_empty() {
        return Vec::new();
    }

    let use_lab = cluster_parameters.color_difference_method != ColorDifferenceMethod::RGB;
    let working_dist_func: WorkingDistFunc = if use_lab { ciede2000 } else { calc_rgb_color_difference_by_squared_rgb_double };

    // 1. Preprocessing: deduplicate + pre-convert to working space
    let mut unique_colors: Vec<Color> = colors.to_vec();
    unique_colors.sort();
    unique_colors.dedup();

    if unique_colors.len() < k {
        k = unique_colors.len();
        if k == 0 {
            return Vec::new();
        }
        return unique_colors;
    }

    let mut working_colors: Vec<ColorDouble> = vec![[0.0; 3]; unique_colors.len()];
    for i in 0..unique_colors.len() {
        if use_lab {
            working_colors[i] = convert_rgb_to_lab(&unique_colors[i]);
        } else {
            working_colors[i] = [unique_colors[i][0] as f64, unique_colors[i][1] as f64, unique_colors[i][2] as f64];
        }
    }

    // 2. K-Means clustering
    let centers = kmeans_core(&working_colors, k, max_iter, working_dist_func, &cluster_parameters.cancel_callback);

    // 3. Output: convert from working space back to RGB
    let mut result: Vec<Color> = vec![[0; 3]; k];
    for i in 0..k {
        if use_lab {
            result[i] = convert_lab_to_rgb(&centers[i]);
        } else {
            result[i] = [centers[i][0].round() as usize, centers[i][1].round() as usize, centers[i][2].round() as usize];
        }
    }

    result
}

/// ColorUtils.cpp:1113-1142: each color replaced by the nearest of the specified colors (squared
/// RGB distance). C++ reads `specified_colors[0]` when the list is empty; this panics there.
pub fn cluster_to_specified_colors(colors: &[Color], specified_colors: &[Color]) -> Vec<Color> {
    let mut cluster_colors: Vec<Color> = colors.to_vec();
    let mut specified_double_colors: Vec<ColorDouble> = Vec::with_capacity(specified_colors.len());
    for color in specified_colors.iter() {
        specified_double_colors.push([color[0] as f64, color[1] as f64, color[2] as f64]);
    }

    for i in 0..cluster_colors.len() {
        let p_color: ColorDouble = [cluster_colors[i][0] as f64, cluster_colors[i][1] as f64, cluster_colors[i][2] as f64];

        let mut min_dist = f64::MAX;
        let mut best_cluster: usize = 0;

        for j in 0..specified_double_colors.len() {
            let d = calc_rgb_color_difference_by_squared_rgb_double(&p_color, &specified_double_colors[j]);
            if d < min_dist {
                min_dist = d;
                best_cluster = j;
            }
        }

        cluster_colors[i] = specified_colors[best_cluster];
    }

    cluster_colors
}

/// ColorUtils.cpp:1232-1345: K-Means with the smallest number of centers (binary search, at most
/// `max_cluster_k`) for which the 99th percentile of the distance of a color to its center is
/// within `max_color_distance`
pub fn cluster_adaptive(colors: &[Color], cluster_parameters: &ClusterParameters) -> Vec<Color> {
    if colors.is_empty() {
        return Vec::new();
    }

    let max_color_distance = cluster_parameters.max_color_distance;
    let max_iter = cluster_parameters.max_iter;
    let use_lab = cluster_parameters.color_difference_method != ColorDifferenceMethod::RGB;
    let working_dist_func: WorkingDistFunc = if use_lab { ciede2000 } else { calc_rgb_color_difference_by_squared_rgb_double };

    // 1. Deduplicate + convert to working space
    let mut unique_colors: Vec<Color> = colors.to_vec();
    unique_colors.sort();
    unique_colors.dedup();

    if unique_colors.len() <= ADAPTIVE_CLUSTER_SKIP_UNIQUE_COUNT {
        return unique_colors;
    }

    let mut working_colors: Vec<ColorDouble> = vec![[0.0; 3]; unique_colors.len()];
    for i in 0..unique_colors.len() {
        if use_lab {
            working_colors[i] = convert_rgb_to_lab(&unique_colors[i]);
        } else {
            working_colors[i] = [unique_colors[i][0] as f64, unique_colors[i][1] as f64, unique_colors[i][2] as f64];
        }
    }

    // 2. Binary search k: find the smallest k where P99 radius <= max_color_distance
    let max_k = cluster_parameters.max_cluster_k;
    let mut lo: usize = 1;
    let mut hi: usize = max_k.min(unique_colors.len());
    let mut best_k: usize = 0;
    let mut best_centers: Vec<ColorDouble> = Vec::new();

    const K_RADIUS_PERCENTILE: f64 = 0.99;

    let calc_max_radius = |centers: &Vec<ColorDouble>| -> f64 {
        let mut distances: Vec<f64> = vec![0.0; working_colors.len()];
        for i in 0..working_colors.len() {
            let mut min_dist = f64::MAX;
            for center in centers.iter() {
                let d = working_dist_func(&working_colors[i], center);
                if d < min_dist {
                    min_dist = d;
                }
            }
            distances[i] = min_dist;
        }
        if distances.is_empty() {
            return 0.0;
        }
        let idx = ((distances.len() as f64 * K_RADIUS_PERCENTILE) as usize).min(distances.len() - 1);
        // std::nth_element(distances.begin(), distances.begin() + idx, distances.end());
        distances.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        distances[idx]
    };

    let cancel_cb = &cluster_parameters.cancel_callback;
    let cancelled = || -> bool {
        match cancel_cb {
            Some(cb) => cb(),
            None => false,
        }
    };

    while lo <= hi {
        if cancelled() {
            return Vec::new();
        }
        let mid = lo + (hi - lo) / 2;
        let centers = kmeans_core(&working_colors, mid, max_iter, working_dist_func, cancel_cb);
        if cancelled() {
            return Vec::new();
        }
        let max_radius = calc_max_radius(&centers);

        if max_radius <= max_color_distance {
            best_k = mid;
            best_centers = centers;
            hi = mid - 1;
        } else {
            lo = mid + 1;
        }
    }

    if best_k == 0 {
        best_k = max_k.min(unique_colors.len());
        best_centers = kmeans_core(&working_colors, best_k, max_iter, working_dist_func, cancel_cb);
    }

    // 3. Convert centers back to RGB
    let mut result: Vec<Color> = vec![[0; 3]; best_k];
    for i in 0..best_k {
        if use_lab {
            result[i] = convert_lab_to_rgb(&best_centers[i]);
        } else {
            result[i] = [best_centers[i][0].round() as usize, best_centers[i][1].round() as usize, best_centers[i][2].round() as usize];
        }
    }

    result
}

/// ColorUtils.cpp:1347-1373: the faces of each connected component, in the order a breadth-first
/// walk from the component's first face meets them
pub fn get_connected_face_groups(mesh: &CGALMesh) -> Vec<Vec<SM_Face_index>> {
    let mut face_groups: Vec<Vec<SM_Face_index>> = Vec::new();
    let mut visited_faces: HashSet<SM_Face_index> = HashSet::new();
    for src_face in mesh.faces() {
        if visited_faces.contains(&src_face) {
            continue;
        }
        let mut face_group: Vec<SM_Face_index> = Vec::new();
        let mut que: VecDeque<SM_Face_index> = VecDeque::new();
        que.push_back(src_face);
        visited_faces.insert(src_face);
        while let Some(curr_face) = que.pop_front() {
            face_group.push(curr_face);
            for adj_face in mesh.faces_around_face(mesh.halfedge_f(curr_face)) {
                if !mesh.is_valid_f(adj_face) || mesh.is_removed_f(adj_face) || visited_faces.contains(&adj_face) {
                    continue;
                }
                que.push_back(adj_face);
                visited_faces.insert(adj_face);
            }
        }
        face_groups.push(face_group);
    }
    face_groups
}

/// ColorUtils.cpp:1375-1421: the mesh split into its connected components, each with the texture
/// coordinates of its vertices
pub fn get_components(
    bbs_mesh: &TriMesh,
    bbs_vertex_uvs: &[Vec2f],
    component_meshes: &mut Vec<TriMesh>,
    component_vertex_uvs: &mut Vec<Vec<Vec2f>>,
) -> bool {
    component_meshes.clear();
    component_vertex_uvs.clear();

    if bbs_mesh.vertices.len() != bbs_vertex_uvs.len() {
        return false;
    }

    let mut cgal_mesh = CGALMesh::new();
    let mut cgal_vertex_uvs: Vec<Vec2f> = Vec::new();
    if !cgal_utils::convert_trimesh_to_cgal_with_uvs(bbs_mesh, bbs_vertex_uvs, &mut cgal_mesh, &mut cgal_vertex_uvs) {
        return false;
    }

    let face_groups = get_connected_face_groups(&cgal_mesh);

    for faces in face_groups.iter() {
        let mut map_cgal_vtx_to_bbs_vtx: HashMap<SM_Vertex_index, usize> = HashMap::new();
        let mut bbs_vertices: TriVertices = Vec::new();
        let mut bbs_faces: TriFaces = Vec::new();
        let mut bbs_vertex_uvs: Vec<Vec2f> = Vec::new();

        for &cgal_face in faces.iter() {
            let mut bbs_face: TriFace = TriFace::new(0, 0, 0);
            let mut face_vid: usize = 0;
            for cgal_vtx in cgal_mesh.vertices_around_face(cgal_mesh.halfedge_f(cgal_face)) {
                if !map_cgal_vtx_to_bbs_vtx.contains_key(&cgal_vtx) {
                    map_cgal_vtx_to_bbs_vtx.insert(cgal_vtx, bbs_vertices.len());
                    let cgal_point = cgal_mesh.point(cgal_vtx);
                    bbs_vertices.push(TriVertex::new(cgal_point[0] as f32, cgal_point[1] as f32, cgal_point[2] as f32));
                    bbs_vertex_uvs.push(cgal_vertex_uvs[cgal_vtx.idx()]);
                }
                bbs_face[face_vid] = map_cgal_vtx_to_bbs_vtx[&cgal_vtx] as i32;
                face_vid += 1;
            }
            bbs_faces.push(bbs_face);
        }

        component_meshes.push(TriMesh::new(bbs_faces, bbs_vertices));
        component_vertex_uvs.push(bbs_vertex_uvs);
    }

    true
}

/// ColorUtils.cpp:1423-1438: the index of the color nearest (CIEDE2000) to `color`
pub fn calc_nearest_color_id(colors: &[RGB], color: &RGB, nearest_color_id: &mut usize) -> bool {
    if colors.is_empty() {
        return false;
    }
    let mut min_dist = f64::MAX;
    *nearest_color_id = 0;
    for i in 0..colors.len() {
        let dist = calc_rgb_color_difference_by_ciede2000(&colors[i], color);
        if dist < min_dist {
            min_dist = dist;
            *nearest_color_id = i;
        }
    }
    true
}

/// ColorUtils.cpp:1440-1648: every face given one of the cluster centers: the faces whose color
/// is clearly nearest to one center first, then their neighbours by flooding, then what is left
/// by the average color of its connected region.
pub fn mesh_cluster(
    bbs_mesh: &TriMesh,
    cluster_centers: &[RGB],
    map_face_to_rgb: &mut Vec<RGB>,
    map_face_to_cluster_id: &mut Vec<usize>,
) -> bool {
    if cluster_centers.is_empty() {
        return false;
    }

    let mut mesh = CGALMesh::new();
    cgal_utils::convert_trimesh_to_cgal(bbs_mesh, &mut mesh);
    if mesh.number_of_faces() != bbs_mesh.indices.len() {
        return false;
    }
    if mesh.number_of_faces() != map_face_to_rgb.len() {
        return false;
    }

    if cluster_centers.len() == 1 {
        for rgb in map_face_to_rgb.iter_mut() {
            *rgb = cluster_centers[0];
        }
        return true;
    }

    let mut map_face_to_area: Vec<f64> = vec![0.0; mesh.number_of_faces()];
    for fid in 0..mesh.number_of_faces() {
        let face = SM_Face_index(fid as u32);
        // std::max(PMP::face_area(face, mesh), EPSILON)
        let area = face_area(face, &mesh);
        map_face_to_area[fid] = if area < EPSILON { EPSILON } else { area };
    }

    // Step 1: Identify faces that definitely belong to a cluster center.
    // A face is definitively assigned when dist1 * absolute_difference_times < dist2 (nearest vs. second-nearest center).
    const ABSOLUTE_DIFFERENCE_TIMES: f64 = 1.5;
    // dE <= 3.0: noticeable by ordinary observers; general quality control
    const DIFFERENCE_EPSILON: f64 = 3.0;
    const INVALID_CLUSTER_ID: usize = usize::MAX;
    map_face_to_cluster_id.resize(mesh.number_of_faces(), INVALID_CLUSTER_ID);
    let mut map_face_to_dists: Vec<Vec<(f64, usize)>> = vec![vec![(0.0, 0); cluster_centers.len()]; mesh.number_of_faces()];
    for fid in 0..mesh.number_of_faces() {
        let dist_and_cid_vec = &mut map_face_to_dists[fid];
        for cluster_id in 0..cluster_centers.len() {
            let dist = calc_rgb_color_difference_by_ciede2000(&map_face_to_rgb[fid], &cluster_centers[cluster_id]);
            dist_and_cid_vec[cluster_id] = (dist, cluster_id);
        }
        // std::sort of pairs: by the distance, then by the id
        dist_and_cid_vec.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal).then(a.1.cmp(&b.1)));
        if dist_and_cid_vec[0].0 < DIFFERENCE_EPSILON || dist_and_cid_vec[0].0 * ABSOLUTE_DIFFERENCE_TIMES < dist_and_cid_vec[1].0 {
            map_face_to_cluster_id[fid] = dist_and_cid_vec[0].1;
        }
    }

    let mut unclusted_fids: LibcxxUnorderedMap<()> = LibcxxUnorderedMap::new();
    for fid in 0..mesh.number_of_faces() {
        if map_face_to_cluster_id[fid] == INVALID_CLUSTER_ID {
            unclusted_fids.insert(fid);
        }
    }

    fn convert_unclustered_to_clustered(unclusted_fids: &mut LibcxxUnorderedMap<()>, iter_clustered_fids: &LibcxxUnorderedMap<()>) -> bool {
        if iter_clustered_fids.is_empty() {
            return false;
        }
        for fid in iter_clustered_fids.keys() {
            unclusted_fids.erase(fid);
        }
        true
    }

    // Step 2: Flood. Use faces computed in the previous step as seeds and propagate outward.
    // C++ clears and reuses one set (`iter_clustered_fids.clear()` keeps its buckets); the order
    // it is iterated in does not reach the result, so a new one is made here.
    while !unclusted_fids.is_empty() {
        let mut changed = false;
        let mut iter_clustered_fids: LibcxxUnorderedMap<()> = LibcxxUnorderedMap::new();
        // If an uncolored face has an adjacent color whose count exceeds the sum of all other colors, assign that color
        for fid in unclusted_fids.keys() {
            let face = SM_Face_index(fid as u32);
            let mut count: usize = 0;
            let mut map_cluster_id_to_count: LibcxxUnorderedMap<usize> = LibcxxUnorderedMap::new();
            for adj_face in mesh.faces_around_face(mesh.halfedge_f(face)) {
                if !mesh.is_valid_f(adj_face) || mesh.is_removed_f(adj_face) {
                    continue;
                }
                count += 1;
                *map_cluster_id_to_count.entry(map_face_to_cluster_id[adj_face.idx()]) += 1;
            }
            for (cluster_id, cnt) in map_cluster_id_to_count.items() {
                if cluster_id != INVALID_CLUSTER_ID && cnt * 2 > count {
                    map_face_to_cluster_id[fid] = cluster_id;
                    iter_clustered_fids.insert(fid);
                    break;
                }
            }
        }
        changed = convert_unclustered_to_clustered(&mut unclusted_fids, &iter_clustered_fids) || changed;
        let mut iter_clustered_fids: LibcxxUnorderedMap<()> = LibcxxUnorderedMap::new();

        // If a face's nearest cluster center (by color distance) happens to have an adjacent face, assign that color too
        for fid in unclusted_fids.keys() {
            let face = SM_Face_index(fid as u32);
            let mut adj_cluster_ids: HashSet<usize> = HashSet::new();
            for adj_face in mesh.faces_around_face(mesh.halfedge_f(face)) {
                if !mesh.is_valid_f(adj_face) || mesh.is_removed_f(adj_face) {
                    continue;
                }
                if map_face_to_cluster_id[adj_face.idx()] == INVALID_CLUSTER_ID {
                    continue;
                }
                adj_cluster_ids.insert(map_face_to_cluster_id[adj_face.idx()]);
            }
            if adj_cluster_ids.contains(&map_face_to_dists[fid][0].1) {
                map_face_to_cluster_id[fid] = map_face_to_dists[fid][0].1;
                iter_clustered_fids.insert(fid);
            }
        }
        changed = convert_unclustered_to_clustered(&mut unclusted_fids, &iter_clustered_fids) || changed;

        // If no face colors were modified in this iteration, stop
        if !changed {
            break;
        }
    }

    // Step 3: Handle remaining unclustered faces (run after the above operations complete)
    const USE_AVERAGE_COLOR: bool = true;
    for src_fid in unclusted_fids.keys() {
        if unclusted_fids.count(src_fid) == 0 {
            continue;
        }
        // Compute connected unclustered faces
        let mut que: VecDeque<usize> = VecDeque::new();
        let mut connected_unclustered_faces: LibcxxUnorderedMap<()> = LibcxxUnorderedMap::new();
        que.push_back(src_fid);
        connected_unclustered_faces.insert(src_fid);
        let (mut sum_r, mut sum_g, mut sum_b) = (0.0f64, 0.0f64, 0.0f64);
        let mut sum_area = 0.0f64;
        let mut map_cluster_id_to_adj_area: LibcxxUnorderedMap<f64> = LibcxxUnorderedMap::new();
        while let Some(curr_fid) = que.pop_front() {
            sum_r = map_face_to_area[curr_fid].mul_add(map_face_to_rgb[curr_fid][0] as f64, sum_r);
            sum_g = map_face_to_area[curr_fid].mul_add(map_face_to_rgb[curr_fid][1] as f64, sum_g);
            sum_b = map_face_to_area[curr_fid].mul_add(map_face_to_rgb[curr_fid][2] as f64, sum_b);
            sum_area += map_face_to_area[curr_fid];
            let curr_face = SM_Face_index(curr_fid as u32);
            for adj_face in mesh.faces_around_face(mesh.halfedge_f(curr_face)) {
                if !mesh.is_valid_f(adj_face) || mesh.is_removed_f(adj_face) {
                    // Invalid face
                    continue;
                }
                if map_face_to_cluster_id[adj_face.idx()] != INVALID_CLUSTER_ID {
                    *map_cluster_id_to_adj_area.entry(map_face_to_cluster_id[adj_face.idx()]) += map_face_to_area[adj_face.idx()];
                } else if connected_unclustered_faces.count(adj_face.idx()) == 0 {
                    // Already clustered or already recorded
                    que.push_back(adj_face.idx());
                    connected_unclustered_faces.insert(adj_face.idx());
                }
            }
        }
        let mut matched_cluster_id = INVALID_CLUSTER_ID;
        if USE_AVERAGE_COLOR {
            // Use average color
            let average_color: RGB = [(sum_r / sum_area) as usize, (sum_g / sum_area) as usize, (sum_b / sum_area) as usize];
            let mut min_dist = f64::MAX;
            for (cluster_id, _area) in map_cluster_id_to_adj_area.items() {
                let dist = calc_rgb_color_difference_by_ciede2000(&average_color, &cluster_centers[cluster_id]);
                if dist < min_dist {
                    min_dist = dist;
                    matched_cluster_id = cluster_id;
                }
            }
        } else {
            // Use adjacent area
            let mut adj_max_area = 0.0f64;
            for (cluster_id, area) in map_cluster_id_to_adj_area.items() {
                if area > adj_max_area {
                    adj_max_area = area;
                    matched_cluster_id = cluster_id;
                }
            }
        }
        for fid in connected_unclustered_faces.keys() {
            map_face_to_cluster_id[fid] = matched_cluster_id;
            unclusted_fids.erase(fid);
        }
    }

    // Convert cluster center IDs to colors
    for fid in 0..mesh.number_of_faces() {
        if map_face_to_cluster_id[fid] == INVALID_CLUSTER_ID {
            map_face_to_rgb[fid] = cluster_centers[0];
            map_face_to_cluster_id[fid] = 0; // Prevent out-of-bounds errors when using cluster_id later
        } else {
            map_face_to_rgb[fid] = cluster_centers[map_face_to_cluster_id[fid]];
        }
    }

    true
}
