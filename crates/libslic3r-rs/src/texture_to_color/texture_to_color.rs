//! Port of `TextureToColor/TextureToColor.hpp` / `TextureToColor.cpp`: a textured mesh, or a mesh
//! with a color per face, turned into a mesh whose faces carry one of a few colors: the faces
//! subdivided, the texture sampled per face, the mesh repaired, the colors clustered and the
//! regions of one color smoothed.
//!
//! C++ Reference:
//! - src/libslic3r/TextureToColor/TextureToColor.hpp
//! - src/libslic3r/TextureToColor/TextureToColor.cpp
//!
//! Floating point follows the reference build (clang, `-ffp-contract=on`): `a * b + c` inside one
//! source expression is one fused operation, written `mul_add` here.
//!
//! FIDELITY-NOTE: `cv::Mat` is `Mat` here, a stand-in with what the unit reads of it: `rows`,
//! `cols`, `channels()`, `step[0]`, `data` and `empty()`. The crate decodes no images: the caller
//! gives the pixels as OpenCV would have decoded them (8 bits per channel, blue first).
//!
//! FIDELITY-NOTE: `RepairMesh` (Repair.hpp) is a chain of CGAL algorithms the crate has no port
//! of, so `repair_cluster_smooth`, `TextureToColor` and `ClusterAndSmooth` take them as
//! `cgal_utils::PolygonMeshProcessing`. They are called only for a mesh that is not
//! halfedge-compatible (TextureToColor.cpp:747-751).
//!
//! FIDELITY-NOTE: `repair_cluster_smooth` reads of `TriangleMesh::stats()` only `manifold()` and
//! `has_open_edges()` (TextureToColor.cpp:689-691). Those come from `its_quick_diagnostics`
//! (`fill_initial_stats`, TriangleMesh.cpp:46-59), which is called here directly; the volume, the
//! bounding box and the number of parts of the stats are not computed.
//!
//! FIDELITY-NOTE: `resample_face_colors` (TextureToColor.cpp:631-657, reached only after a mesh
//! repair) looks the old faces up with the crate's `aabb_tree_indirect`, which works in doubles;
//! C++ instantiates `AABBTreeIndirect` with `Vec3f` there. Where two old faces are at distances
//! from a centroid that differ only below float precision, the face found can differ.
//!
//! FIDELITY-NOTE: `MeshRepairCache* mesh_repair_cache` of the settings, written through a const
//! reference in C++, is an `Arc<Mutex<MeshRepairCache>>`.
//!
//! FIDELITY-NOTE: `tbb::parallel_for` (TextureToColor.cpp:308, 644, 803, 1004) is a plain loop
//! here. Each iteration writes its own element only, so the results are the same; the progress
//! reports and the questions to the cancel callback come in the order of the faces, as they do
//! in C++ on one thread.
//!
//! What C++ writes to the log, and the durations it measures for it, are left out.
//!
//! Nothing in the slicing path calls this unit. Its callers in C++ are TexturePainting.cpp
//! (`TextureToColor`, `ClusterAndSmooth`), ObjColorUtils.cpp (`quantize_colors`,
//! `adaptive_split_by_vertex_clusters`) and the texture import dialog of the desktop app.

#![allow(non_snake_case)]

use std::cell::Cell;
use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use crate::aabb_tree_indirect;
use crate::geometry::aabb_tree::Vec3;
use crate::geometry::Point3F;
use crate::mesh_diagnostics::its_quick_diagnostics;
use crate::texture_to_color::callbacks::{AlgoCancelCallback, AlgoProgress, AlgoProgressCallback};
use crate::texture_to_color::cgal_utils::{self, PolygonMeshProcessing};
use crate::texture_to_color::color_utils::{
    calc_nearest_color_id, calc_rgb_color_difference_by_ciede2000, cluster_adaptive, cluster_k_means, is_closed, ostream_double,
    smooth_region_with_labels, ClusterParameters, SmoothParameters, RGB,
};
use crate::texture_to_color::repair::{RepairMesh, RepairSetting};
use crate::texture_to_color::tri_mesh::{TriFaces, TriMesh, TriVertex, TriVertices};
use crate::triangle_mesh::Vec2f;
use crate::triangle_set_sampling::{indexed_triangle_set, Vec3f, Vec3i};

/// `cv::Mat` as far as this unit reads it. See the FIDELITY-NOTE of the file.
#[derive(Debug, Clone, Default)]
pub struct Mat {
    pub rows: i32,
    pub cols: i32,
    /// `Mat::channels()`
    pub channels: i32,
    /// `Mat::step[0]`: the bytes of one row
    pub step: usize,
    /// `Mat::data`: the rows one after the other, each pixel `channels` bytes, blue first
    pub data: Vec<u8>,
}

impl Mat {
    /// A matrix of `rows` by `cols` pixels of `channels` bytes over `data`, its rows packed
    pub fn new(rows: i32, cols: i32, channels: i32, data: Vec<u8>) -> Self {
        Mat { rows, cols, channels, step: (cols.max(0) as usize) * (channels.max(0) as usize), data }
    }

    /// `Mat::channels()`
    pub fn channels(&self) -> i32 {
        self.channels
    }

    /// `Mat::empty()`: no data or no elements
    pub fn empty(&self) -> bool {
        self.data.is_empty() || self.rows == 0 || self.cols == 0
    }
}

/// TextureToColor.hpp:14-17
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MeshRepairDecision {
    ImportWithoutRepair,
    RepairAndImport,
}

/// TextureToColor.hpp:19-23: `(mesh, repaired_mesh, progress_callback(message, progress),
/// cancel_callback, error_message) -> bool`. The message of the progress callback can be null.
pub type MeshRepairCallback = Option<
    Arc<dyn Fn(&indexed_triangle_set, &mut indexed_triangle_set, &dyn Fn(Option<&str>, u32), &dyn Fn() -> bool, &mut String) -> bool + Send + Sync>,
>;

/// TextureToColor.hpp:25-29
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MeshRepairAttempt {
    /// never tried, or the last attempt was cancelled
    NotAttempted,
    Succeeded,
    /// tried and failed (including timeout); do not retry Win10 repair
    Failed,
}

/// TextureToColor.hpp:31-68: session cache for one mesh. Callers must use a given cache serially
/// and must not share it across unrelated meshes.
#[derive(Debug, Clone)]
pub struct MeshRepairCache {
    pub win10_attempt: MeshRepairAttempt,
    pub has_prepared_mesh: bool,
    /// oversampled + repaired geometry
    pub mesh: TriMesh,
    /// pre-cluster face colors
    pub face_colors: Vec<[usize; 3]>,

    pub key_input_face_count: usize,
    pub key_oversampling_iters: usize,
    pub key_oversampling_min_face_count: usize,
    pub key_oversampling_max_face_count: usize,
}

impl Default for MeshRepairCache {
    /// TextureToColor.hpp:34-42, the member initializers
    fn default() -> Self {
        MeshRepairCache {
            win10_attempt: MeshRepairAttempt::NotAttempted,
            has_prepared_mesh: false,
            mesh: TriMesh::default(),
            face_colors: Vec::new(),
            key_input_face_count: 0,
            key_oversampling_iters: 0,
            key_oversampling_min_face_count: 0,
            key_oversampling_max_face_count: 0,
        }
    }
}

impl MeshRepairCache {
    /// TextureToColor.hpp:44-54
    pub fn matches_input(
        &self,
        face_count: usize,
        oversampling_iters: usize,
        oversampling_min_face_count: usize,
        oversampling_max_face_count: usize,
    ) -> bool {
        self.has_prepared_mesh
            && self.key_input_face_count == face_count
            && self.key_oversampling_iters == oversampling_iters
            && self.key_oversampling_min_face_count == oversampling_min_face_count
            && self.key_oversampling_max_face_count == oversampling_max_face_count
    }

    /// TextureToColor.hpp:56-67: drops prepared geometry. `win10_attempt` is kept so a Failed
    /// Win10 attempt is not retried for the rest of the session.
    pub fn invalidate_prepared_mesh(&mut self) {
        self.has_prepared_mesh = false;
        self.mesh = TriMesh::default();
        self.face_colors.clear();
        self.key_input_face_count = 0;
        self.key_oversampling_iters = 0;
        self.key_oversampling_min_face_count = 0;
        self.key_oversampling_max_face_count = 0;
    }
}

/// TextureToColor.hpp:70-99
#[derive(Clone)]
pub struct TextureToColorSettings {
    /// the number of colors; 0 lets the clustering find it
    pub target_colors_num: usize,

    /// smoothing weight, [0, 1]: 0 is no smoothing, 1 all of it
    pub smooth_weight: f64,

    /// above 0: that many oversampling iterations; otherwise the oversampling adapts
    pub oversampling_iters: usize,
    /// adaptive oversampling: the faces are subdivided while there are fewer than this
    pub oversampling_min_face_count: usize,
    /// whatever else is given, the oversampled mesh has no more faces than this
    pub oversampling_max_face_count: usize,

    /// the largest radius (CIEDE2000) of a cluster of the adaptive clustering
    pub max_color_distance: f64,
    /// the largest number of colors of the adaptive clustering
    pub max_cluster_k: usize,

    pub mesh_repair_decision: MeshRepairDecision,

    pub mesh_repair_callback: MeshRepairCallback,

    /// Max wait for Windows 3D repair. Timed-out repair is cancelled and the input mesh is kept;
    /// CGAL RepairMesh may still run afterwards. Zero disables the timeout.
    pub mesh_repair_timeout: Duration,

    /// Optional session cache. When set, a successful repair is stored so later recomputes can
    /// skip Win10 / CGAL repair. Failed Win10 attempts are also recorded so they are not
    /// retried. User cancel does not count as failure.
    pub mesh_repair_cache: Option<Arc<Mutex<MeshRepairCache>>>,
}

impl Default for TextureToColorSettings {
    /// TextureToColor.hpp:71-98, the member initializers
    fn default() -> Self {
        TextureToColorSettings {
            target_colors_num: 4,
            smooth_weight: 0.5,
            oversampling_iters: 0,
            oversampling_min_face_count: 10000,
            oversampling_max_face_count: 1000000,
            max_color_distance: 25.0,
            max_cluster_k: 32,
            mesh_repair_decision: MeshRepairDecision::ImportWithoutRepair,
            mesh_repair_callback: None,
            mesh_repair_timeout: Duration::from_secs(0),
            mesh_repair_cache: None,
        }
    }
}

/// TextureToColor.cpp:27-57: the mesh as an OFF file with a color per face. As in C++ nothing is
/// said when the file cannot be written.
pub fn SaveToOFF(path: &str, mesh: &TriMesh, face_colors: &[RGB]) {
    if let Some(parent) = std::path::Path::new(path).parent() {
        let _ = std::fs::create_dir_all(parent);
    }

    let vertices = &mesh.vertices;
    let faces = &mesh.indices;

    let mut ofs = String::new();
    ofs += "OFF\n";
    ofs += &format!("{} {} 0\n", vertices.len(), faces.len());

    for v in vertices.iter() {
        ofs += &format!("{} {} {}\n", ostream_double(v.x as f64), ostream_double(v.y as f64), ostream_double(v.z as f64));
    }

    for i in 0..faces.len() {
        let f = &faces[i];
        ofs += &format!("3 {} {} {}", f[0], f[1], f[2]);
        if i < face_colors.len() {
            ofs += &format!(
                " {} {} {} 1.0",
                ostream_double(face_colors[i][0] as f64 / 255.0),
                ostream_double(face_colors[i][1] as f64 / 255.0),
                ostream_double(face_colors[i][2] as f64 / 255.0)
            );
        }
        ofs += "\n";
    }
    let _ = std::fs::write(path, ofs);
}

/// TextureToColor.cpp:59-68: how many faces carry each label below `cluster_count`
pub fn count_cluster_label_usage(face_labels: &[usize], cluster_count: usize) -> Vec<usize> {
    let mut usage: Vec<usize> = vec![0; cluster_count];
    for &label in face_labels.iter() {
        if label < cluster_count {
            usage[label] += 1;
        }
    }
    usage
}

/// TextureToColor.cpp:70-113: the cluster centers no face uses are dropped and the labels
/// renumbered. `stage_name` is for the log only.
pub fn discard_unused_cluster_centers(cluster_centers: &mut Vec<RGB>, face_labels: &mut Vec<usize>, _stage_name: &str) -> bool {
    if cluster_centers.is_empty() {
        return false;
    }

    let usage = count_cluster_label_usage(face_labels, cluster_centers.len());
    let mut label_remap: Vec<usize> = vec![usize::MAX; cluster_centers.len()];
    let mut used_cluster_centers: Vec<RGB> = Vec::with_capacity(cluster_centers.len());

    for cluster_id in 0..cluster_centers.len() {
        if usage[cluster_id] == 0 {
            continue;
        }
        label_remap[cluster_id] = used_cluster_centers.len();
        used_cluster_centers.push(cluster_centers[cluster_id]);
    }

    if used_cluster_centers.len() == cluster_centers.len() {
        return true;
    }
    if used_cluster_centers.is_empty() {
        return false;
    }

    for label in face_labels.iter_mut() {
        if *label >= label_remap.len() || label_remap[*label] == usize::MAX {
            return false;
        }
        *label = label_remap[*label];
    }

    *cluster_centers = used_cluster_centers;
    true
}

/// TextureToColor.cpp:115-184: every cluster center no face uses is given the face that loses
/// least by the change, taken from a cluster with more than one face. True when every center is
/// used in the end. `stage_name` is for the log only.
pub fn ensure_all_cluster_centers_used(source_face_colors: &[RGB], cluster_centers: &[RGB], face_labels: &mut Vec<usize>, _stage_name: &str) -> bool {
    if source_face_colors.len() != face_labels.len() {
        return false;
    }
    if cluster_centers.is_empty() {
        return false;
    }
    if cluster_centers.len() > face_labels.len() {
        return false;
    }

    let mut usage = count_cluster_label_usage(face_labels, cluster_centers.len());
    let mut _missing_count: usize = 0;
    for cluster_id in 0..usage.len() {
        if usage[cluster_id] != 0 {
            continue;
        }
        _missing_count += 1;

        let mut best_cost = f64::MAX;
        let mut best_face_id = usize::MAX;
        let mut best_old_cluster_id = usize::MAX;

        for fid in 0..face_labels.len() {
            let old_cluster_id = face_labels[fid];
            if old_cluster_id >= cluster_centers.len() || usage[old_cluster_id] <= 1 {
                continue;
            }

            let old_dist = calc_rgb_color_difference_by_ciede2000(&source_face_colors[fid], &cluster_centers[old_cluster_id]);
            let new_dist = calc_rgb_color_difference_by_ciede2000(&source_face_colors[fid], &cluster_centers[cluster_id]);
            let cost = new_dist - old_dist;
            if cost < best_cost {
                best_cost = cost;
                best_face_id = fid;
                best_old_cluster_id = old_cluster_id;
            }
        }

        if best_face_id == usize::MAX {
            continue;
        }

        face_labels[best_face_id] = cluster_id;
        usage[best_old_cluster_id] -= 1;
        usage[cluster_id] += 1;
    }

    for &count in usage.iter() {
        if count == 0 {
            return false;
        }
    }
    true
}

/// `std::clamp(v, lo, hi)` of floats
#[inline]
fn clamp_f32(v: f32, lo: f32, hi: f32) -> f32 {
    if v < lo {
        lo
    } else if hi < v {
        hi
    } else {
        v
    }
}

/// `std::clamp(v, lo, hi)` of ints
#[inline]
fn clamp_i32(v: i32, lo: i32, hi: i32) -> i32 {
    if v < lo {
        lo
    } else if hi < v {
        hi
    } else {
        v
    }
}

/// TextureToColor.cpp:186-222: bilinear interpolation texture sampling; sub-pixel precision
/// avoids nearest-neighbor aliasing
pub fn get_pixel_color(u: f32, v: f32, texture: &Mat) -> RGB {
    let u = u - u.floor();
    let v = v - v.floor();

    // glTF UV convention: (0,0) = top-left, v increases downward
    let fx = u * (texture.cols - 1) as f32;
    let fy = v * (texture.rows - 1) as f32;

    let x0 = clamp_i32(fx as i32, 0, texture.cols - 1);
    let y0 = clamp_i32(fy as i32, 0, texture.rows - 1);
    let x1 = (x0 + 1).min(texture.cols - 1);
    let y1 = (y0 + 1).min(texture.rows - 1);

    let wx = fx - x0 as f32;
    let wy = fy - y0 as f32;

    let ch = texture.channels();
    let sample = |row: i32, col: i32| -> [f32; 3] {
        let ptr = row as usize * texture.step + col as usize * ch as usize;
        [texture.data[ptr + 2] as f32, texture.data[ptr + 1] as f32, texture.data[ptr] as f32]
    };

    let c00 = sample(y0, x0);
    let c10 = sample(y0, x1);
    let c01 = sample(y1, x0);
    let c11 = sample(y1, x1);

    // Bilinear blend: lerp(lerp(c00,c10,wx), lerp(c01,c11,wx), wy)
    let mut color: RGB = [0; 3];
    for i in 0..3 {
        let top = c00[i].mul_add(1.0f32 - wx, c10[i] * wx);
        let bot = c01[i].mul_add(1.0f32 - wx, c11[i] * wx);
        color[i] = clamp_f32(top.mul_add(1.0f32 - wy, bot * wy), 0.0, 255.0) as usize;
    }
    color
}

/// TextureToColor.cpp:224-233: 7-point triangular Gaussian quadrature barycentric coordinates
pub const GAUSS_TRI_BARY: [[f32; 3]; 7] = [
    [1.0f32 / 3.0f32, 1.0f32 / 3.0f32, 1.0f32 / 3.0f32],
    [0.059715871, 0.470142064, 0.470142064],
    [0.470142064, 0.059715871, 0.470142064],
    [0.470142064, 0.470142064, 0.059715871],
    [0.797426985, 0.101286507, 0.101286507],
    [0.101286507, 0.797426985, 0.101286507],
    [0.101286507, 0.101286507, 0.797426985],
];
/// TextureToColor.cpp:234: the weights of the quadrature; they sum to 1 (TextureToColor.cpp:235-243)
pub const GAUSS_TRI_WEIGHT: [f32; 7] = [0.225, 0.132394152, 0.132394152, 0.132394152, 0.125939181, 0.125939181, 0.125939181];

/// TextureToColor.cpp:245-260: multi-point Gaussian quadrature sampling on a single face; the
/// weighted average color
pub fn sample_face_color(uvs: &FaceUVArray, texture: &Mat) -> RGB {
    let (mut r, mut g, mut b) = (0.0f32, 0.0f32, 0.0f32);
    for k in 0..7 {
        let u = GAUSS_TRI_BARY[k][2].mul_add(uvs[2].x, GAUSS_TRI_BARY[k][0].mul_add(uvs[0].x, GAUSS_TRI_BARY[k][1] * uvs[1].x));
        let v = GAUSS_TRI_BARY[k][2].mul_add(uvs[2].y, GAUSS_TRI_BARY[k][0].mul_add(uvs[0].y, GAUSS_TRI_BARY[k][1] * uvs[1].y));
        let c = get_pixel_color(u, v, texture);
        let w = GAUSS_TRI_WEIGHT[k];
        r = w.mul_add(c[0] as f32, r);
        g = w.mul_add(c[1] as f32, g);
        b = w.mul_add(c[2] as f32, b);
    }
    [clamp_f32(r, 0.0, 255.0) as usize, clamp_f32(g, 0.0, 255.0) as usize, clamp_f32(b, 0.0, 255.0) as usize]
}

/// TextureToColor.cpp:262-263: the UVs of the three corners of a face
pub type FaceUVArray = [Vec2f; 3];

/// TextureToColor.cpp:279-281, 485-488: the two vertex indices of an edge in one number
#[inline]
fn edge_key(a: usize, b: usize) -> u64 {
    if a < b {
        ((a as u64) << 32) | b as u64
    } else {
        ((b as u64) << 32) | a as u64
    }
}

/// TextureToColor.cpp:265-379: every face split into four at the middles of its edges, the UVs
/// with it. `sub_progress` is told 50 and 100.
pub fn linear_subdivision(mesh: &mut TriMesh, uv_coords: &mut Vec<FaceUVArray>, sub_progress: Option<&dyn Fn(i32)>) -> bool {
    let original_vertices = &mesh.vertices;
    let original_faces = &mesh.indices;
    let mut sub_vertices: TriVertices = mesh.vertices.clone();
    sub_vertices.reserve(original_faces.len() * 3);
    let mut sub_faces: TriFaces;
    let mut sub_uv_coords: Vec<FaceUVArray>;

    // Single-level flat map with edge key encoding
    if original_vertices.len() as u64 >= (1u64 << 32) {
        return false;
    }
    let mut map_edge_to_sub_vtx: HashMap<u64, usize> = HashMap::with_capacity(original_faces.len() * 3 / 2);

    for face in original_faces.iter() {
        for i in 0..3usize {
            let vtx_1 = face[i] as i64 as usize;
            let vtx_2 = face[(i + 1) % 3] as i64 as usize;
            let key = edge_key(vtx_1, vtx_2);
            if map_edge_to_sub_vtx.contains_key(&key) {
                continue;
            }
            let edge_vtx: TriVertex = (original_vertices[vtx_1] + original_vertices[vtx_2]) * 0.5f32;
            map_edge_to_sub_vtx.insert(key, sub_vertices.len());
            sub_vertices.push(edge_vtx);
        }
    }
    if let Some(cb) = sub_progress {
        cb(50);
    }

    // Subdivide faces and their UVs: each original face splits into 4 sub-faces
    let n = original_faces.len();
    sub_faces = vec![Vec3i::new(0, 0, 0); n * 4];
    sub_uv_coords = vec![[Vec2f::new(0.0, 0.0); 3]; n * 4];
    let mut has_missing_edge = false;

    for fid in 0..n {
        let base = fid * 4;
        let face = &original_faces[fid];
        let vtx_0 = face[0] as i64 as usize;
        let vtx_1 = face[1] as i64 as usize;
        let vtx_2 = face[2] as i64 as usize;

        let it01 = map_edge_to_sub_vtx.get(&edge_key(vtx_0, vtx_1));
        let it12 = map_edge_to_sub_vtx.get(&edge_key(vtx_1, vtx_2));
        let it20 = map_edge_to_sub_vtx.get(&edge_key(vtx_2, vtx_0));
        let (e01, e12, e20) = match (it01, it12, it20) {
            (Some(&a), Some(&b), Some(&c)) => (a, b, c),
            _ => {
                has_missing_edge = true;
                let degen = Vec3i::new(vtx_0 as i32, vtx_0 as i32, vtx_0 as i32);
                let degen_uv: FaceUVArray = [uv_coords[fid][0], uv_coords[fid][0], uv_coords[fid][0]];
                for k in 0..4 {
                    sub_faces[base + k] = degen;
                    sub_uv_coords[base + k] = degen_uv;
                }
                continue;
            }
        };

        let uv0 = uv_coords[fid][0];
        let uv1 = uv_coords[fid][1];
        let uv2 = uv_coords[fid][2];
        let uv_e01: Vec2f = (uv0 + uv1) * 0.5f32;
        let uv_e12: Vec2f = (uv1 + uv2) * 0.5f32;
        let uv_e20: Vec2f = (uv2 + uv0) * 0.5f32;

        sub_faces[base] = Vec3i::new(vtx_0 as i32, e01 as i32, e20 as i32);
        sub_uv_coords[base] = [uv0, uv_e01, uv_e20];

        sub_faces[base + 1] = Vec3i::new(e01 as i32, vtx_1 as i32, e12 as i32);
        sub_uv_coords[base + 1] = [uv_e01, uv1, uv_e12];

        sub_faces[base + 2] = Vec3i::new(e01 as i32, e12 as i32, e20 as i32);
        sub_uv_coords[base + 2] = [uv_e01, uv_e12, uv_e20];

        sub_faces[base + 3] = Vec3i::new(e20 as i32, e12 as i32, vtx_2 as i32);
        sub_uv_coords[base + 3] = [uv_e20, uv_e12, uv2];
    }
    // Remove degenerate triangles (three identical vertices) to avoid impacting downstream SDF / Remesh steps
    if has_missing_edge {
        let mut write_idx: usize = 0;
        for i in 0..sub_faces.len() {
            if sub_faces[i][0] == sub_faces[i][1] && sub_faces[i][1] == sub_faces[i][2] {
                continue;
            }
            if write_idx != i {
                sub_faces[write_idx] = sub_faces[i];
                sub_uv_coords[write_idx] = sub_uv_coords[i];
            }
            write_idx += 1;
        }
        sub_faces.truncate(write_idx);
        sub_uv_coords.truncate(write_idx);
    }

    if let Some(cb) = sub_progress {
        cb(100);
    }
    *mesh = TriMesh::new(sub_faces, sub_vertices);
    *uv_coords = sub_uv_coords;
    true
}

/// TextureToColor.cpp:381
pub type VertexColor = [f32; 4];

/// TextureToColor.cpp:383-408: quantize continuous per-vertex colors into a small palette of
/// cluster centers
pub fn quantize_vertex_colors(
    vertex_colors: &[VertexColor],
    settings: &TextureToColorSettings,
    cancel_callback: &AlgoCancelCallback,
    out_centers: &mut Vec<RGB>,
    out_vertex_cluster_ids: &mut Vec<usize>,
) -> bool {
    out_centers.clear();
    out_vertex_cluster_ids.clear();
    if vertex_colors.is_empty() {
        return false;
    }

    let mut vertex_rgb: Vec<RGB> = vec![[0; 3]; vertex_colors.len()];
    for i in 0..vertex_colors.len() {
        for c in 0..3 {
            let v = clamp_f32(vertex_colors[i][c] * 255.0f32, 0.0, 255.0);
            vertex_rgb[i][c] = v as usize;
        }
    }

    quantize_colors(&vertex_rgb, settings, cancel_callback, out_centers, out_vertex_cluster_ids)
}

/// TextureToColor.cpp:410-447: cluster RGB [0..255] colors (adaptive when `target_colors_num` is
/// 0, otherwise k-means); `out_cluster_ids` is each input color's nearest center
pub fn quantize_colors(
    colors: &[[usize; 3]],
    settings: &TextureToColorSettings,
    cancel_callback: &AlgoCancelCallback,
    out_centers: &mut Vec<[usize; 3]>,
    out_cluster_ids: &mut Vec<usize>,
) -> bool {
    out_centers.clear();
    out_cluster_ids.clear();
    if colors.is_empty() {
        return false;
    }

    let mut para = ClusterParameters::default();
    para.cancel_callback = cancel_callback.clone();
    if settings.target_colors_num == 0 {
        para.max_color_distance = settings.max_color_distance;
        para.max_cluster_k = settings.max_cluster_k;
        *out_centers = cluster_adaptive(colors, &para);
    } else {
        para.cluster_k = settings.target_colors_num;
        *out_centers = cluster_k_means(colors, &para);
    }
    if out_centers.is_empty() {
        return false;
    }

    out_cluster_ids.resize(colors.len(), 0);
    for i in 0..colors.len() {
        let mut nearest_id: usize = 0;
        if !calc_nearest_color_id(out_centers, &colors[i], &mut nearest_id) {
            nearest_id = 0;
        }
        out_cluster_ids[i] = nearest_id;
    }
    true
}

/// TextureToColor.cpp:449-586: single-level adaptive subdivision driven by per-vertex cluster
/// ids. An edge is split at its midpoint if and only if its two endpoints belong to different
/// clusters. Each resulting face is given the center color of its cluster.
pub fn adaptive_split_by_vertex_clusters(
    mesh: &mut TriMesh,
    vertex_cluster_ids: &[usize],
    cluster_centers: &[RGB],
    out_face_colors: &mut Vec<RGB>,
) -> bool {
    let original_vertices: TriVertices = mesh.vertices.clone();
    let original_faces: TriFaces = mesh.indices.clone();
    if original_vertices.is_empty() || original_faces.is_empty() || cluster_centers.is_empty() {
        return false;
    }
    if vertex_cluster_ids.len() != original_vertices.len() {
        return false;
    }
    if original_vertices.len() as u64 >= (1u64 << 32) {
        return false;
    }

    let mut out_vertices: TriVertices = original_vertices.clone();
    let mut out_faces: TriFaces = Vec::with_capacity(original_faces.len() * 5);
    out_face_colors.clear();
    out_face_colors.reserve(original_faces.len() * 5);

    let mut edge_to_mid: HashMap<u64, usize> = HashMap::with_capacity(original_faces.len() * 3 / 2);

    // Midpoints on shared edges must be deduplicated so that neighbouring faces
    // reference the same vertex instead of coincident duplicates.
    fn midpoint_of_edge(edge_to_mid: &mut HashMap<u64, usize>, out_vertices: &mut TriVertices, original_vertices: &TriVertices, a: usize, b: usize) -> usize {
        let key = edge_key(a, b);
        if let Some(&idx) = edge_to_mid.get(&key) {
            return idx;
        }
        let idx = out_vertices.len();
        out_vertices.push((original_vertices[a] + original_vertices[b]) * 0.5f32);
        edge_to_mid.insert(key, idx);
        idx
    }
    // Points strictly inside an original face are never shared, so they skip the map.
    fn append_interior_midpoint(out_vertices: &mut TriVertices, a: usize, b: usize) -> usize {
        let mid: TriVertex = (out_vertices[a] + out_vertices[b]) * 0.5f32;
        let idx = out_vertices.len();
        out_vertices.push(mid);
        idx
    }
    let mut emit = |out_faces: &mut TriFaces, a: usize, b: usize, c: usize, cluster_id: usize| {
        out_faces.push(Vec3i::new(a as i32, b as i32, c as i32));
        out_face_colors.push(cluster_centers[cluster_id]);
    };

    for f in original_faces.iter() {
        let v: [usize; 3] = [f[0] as i64 as usize, f[1] as i64 as usize, f[2] as i64 as usize];
        let c: [usize; 3] = [vertex_cluster_ids[v[0]], vertex_cluster_ids[v[1]], vertex_cluster_ids[v[2]]];

        // Case A: uniform cluster, keep the face untouched.
        if c[0] == c[1] && c[1] == c[2] {
            emit(&mut out_faces, v[0], v[1], v[2], c[0]);
            continue;
        }

        // Case B: two vertices share a cluster and the third is isolated. Split the
        // two edges incident to the isolated vertex, which are exactly the
        // cross-cluster ones; the opposite edge stays intact.
        let mut iso: i32 = -1;
        if c[1] == c[2] {
            iso = 0;
        } else if c[2] == c[0] {
            iso = 1;
        } else if c[0] == c[1] {
            iso = 2;
        }
        if iso >= 0 {
            let i = iso as usize;
            let j = (iso as usize + 1) % 3;
            let k = (iso as usize + 2) % 3;
            let m_ij = midpoint_of_edge(&mut edge_to_mid, &mut out_vertices, &original_vertices, v[i], v[j]);
            let m_ki = midpoint_of_edge(&mut edge_to_mid, &mut out_vertices, &original_vertices, v[k], v[i]);
            emit(&mut out_faces, v[i], m_ij, m_ki, c[i]);
            emit(&mut out_faces, m_ij, v[j], m_ki, c[j]);
            emit(&mut out_faces, v[j], v[k], m_ki, c[j]);
            continue;
        }

        // Case C: all three clusters differ. Split every edge, then cut the centre
        // triangle once more. The centre is equidistant from all three clusters, so
        // the legacy heuristic selects the cut by widest interior angle, which is
        // the vertex opposite the longest edge.
        let m01 = midpoint_of_edge(&mut edge_to_mid, &mut out_vertices, &original_vertices, v[0], v[1]);
        let m12 = midpoint_of_edge(&mut edge_to_mid, &mut out_vertices, &original_vertices, v[1], v[2]);
        let m20 = midpoint_of_edge(&mut edge_to_mid, &mut out_vertices, &original_vertices, v[2], v[0]);
        emit(&mut out_faces, v[0], m01, m20, c[0]);
        emit(&mut out_faces, m01, v[1], m12, c[1]);
        emit(&mut out_faces, m12, v[2], m20, c[2]);

        let p0 = original_vertices[v[0]];
        let p1 = original_vertices[v[1]];
        let p2 = original_vertices[v[2]];
        // Eigen's squaredNorm of a float 3-vector: x*x + y*y + z*z, no fused operation
        let squared_norm = |d: Vec3f| -> f32 {
            let (xx, yy, zz) = (d.x * d.x, d.y * d.y, d.z * d.z);
            xx + yy + zz
        };
        let sq_opposite_v0 = squared_norm(p2 - p1);
        let sq_opposite_v1 = squared_norm(p0 - p2);
        let sq_opposite_v2 = squared_norm(p1 - p0);
        let mut widest = 0;
        let mut widest_len = sq_opposite_v0;
        if sq_opposite_v1 > widest_len {
            widest = 1;
            widest_len = sq_opposite_v1;
        }
        if sq_opposite_v2 > widest_len {
            widest = 2;
        }

        if widest == 0 {
            let mc = append_interior_midpoint(&mut out_vertices, m20, m01);
            emit(&mut out_faces, m12, m20, mc, c[1]);
            emit(&mut out_faces, mc, m01, m12, c[2]);
        } else if widest == 1 {
            let mc = append_interior_midpoint(&mut out_vertices, m01, m12);
            emit(&mut out_faces, m20, m01, mc, c[0]);
            emit(&mut out_faces, mc, m12, m20, c[2]);
        } else {
            let mc = append_interior_midpoint(&mut out_vertices, m12, m20);
            emit(&mut out_faces, m01, m12, mc, c[1]);
            emit(&mut out_faces, mc, m20, m01, c[0]);
        }
    }

    *mesh = TriMesh::new(out_faces, out_vertices);
    true
}

/// `resample_face_colors` of `repair_cluster_smooth` (TextureToColor.cpp:630-657): the mesh
/// replaced by the repaired one, every face of it given the color of the old face nearest to its
/// centroid. See the FIDELITY-NOTE of the file.
fn resample_face_colors(mesh: &mut TriMesh, face_colors: &mut Vec<RGB>, repaired_mesh: TriMesh) -> bool {
    let old_vertices: TriVertices = std::mem::take(&mut mesh.vertices);
    let old_indices: TriFaces = std::mem::take(&mut mesh.indices);
    let tree_vertices: Vec<Point3F> = old_vertices.iter().map(|v| Point3F::new(v.x as f64, v.y as f64, v.z as f64)).collect();
    let tree_faces: Vec<[usize; 3]> = old_indices.iter().map(|f| [f[0] as usize, f[1] as usize, f[2] as usize]).collect();
    let aabb_tree = aabb_tree_indirect::build_aabb_tree_over_indexed_triangle_set(&tree_vertices, &tree_faces);
    *mesh = repaired_mesh;

    // C++ logs whether the repaired mesh is closed
    let _ = is_closed(mesh);

    let mut new_face_colors: Vec<RGB> = vec![[0; 3]; mesh.facets_count()];
    for fid in 0..mesh.facets_count() {
        let face = mesh.indices[fid];
        let sum = mesh.vertices[face[0] as usize] + mesh.vertices[face[1] as usize] + mesh.vertices[face[2] as usize];
        let center = [sum.x / 3.0f32, sum.y / 3.0f32, sum.z / 3.0f32];
        let (_, hit_idx, _) = aabb_tree_indirect::squared_distance_to_indexed_triangle_set(
            &tree_vertices,
            &tree_faces,
            &aabb_tree,
            Vec3::new(center[0] as f64, center[1] as f64, center[2] as f64),
        );
        new_face_colors[fid] = face_colors[hit_idx];
    }
    *face_colors = new_face_colors;
    true
}

/// TextureToColor.cpp:588-862: shared pipeline: mesh repair -> color clustering -> label
/// assignment -> smoothing. Called by both `TextureToColor` (after UV sampling) and
/// `ClusterAndSmooth`. `progress_callback` reports 0~100 within this function; the caller maps it
/// to its own range. `log_prefix` is for the log only.
#[allow(clippy::too_many_arguments)]
pub fn repair_cluster_smooth(
    mesh: &mut TriMesh,
    face_colors: &mut Vec<RGB>,
    out_clustered_face_colors: &mut Vec<RGB>,
    settings: &TextureToColorSettings,
    progress_callback: &AlgoProgressCallback,
    cancel_callback: &AlgoCancelCallback,
    _log_prefix: &str,
    cache_input_face_count: usize,
    pmp: &dyn PolygonMeshProcessing,
) -> bool {
    let report = |pct: i32, msg: &str| {
        if let Some(cb) = progress_callback {
            cb(AlgoProgress::new(pct, msg));
        }
    };
    let cancelled = || -> bool {
        match cancel_callback {
            Some(cb) => cb(),
            None => false,
        }
    };

    report(0, "Repairing mesh");
    if cancelled() {
        return false;
    }

    let mut cache_guard = settings.mesh_repair_cache.as_ref().map(|c| c.lock().expect("the mesh repair cache"));
    let mut cache: Option<&mut MeshRepairCache> = cache_guard.as_deref_mut();

    if let Some(cache) = cache.as_deref_mut() {
        if cache.has_prepared_mesh
            && !cache.matches_input(
                cache_input_face_count,
                settings.oversampling_iters,
                settings.oversampling_min_face_count,
                settings.oversampling_max_face_count,
            )
        {
            cache.invalidate_prepared_mesh();
        }
    }

    if cache.as_deref().map_or(false, |c| c.has_prepared_mesh) {
        let cache = cache.as_deref().expect("a cache");
        *mesh = cache.mesh.clone();
        *face_colors = cache.face_colors.clone();
    } else {
        let skip_win10 = cache.as_deref().map_or(false, |c| c.win10_attempt == MeshRepairAttempt::Failed);
        if cache.as_deref().map_or(false, |c| c.win10_attempt == MeshRepairAttempt::Succeeded && !c.mesh.indices.is_empty()) {
            // Win10 succeeded earlier but CGAL repair did not finish; reuse the Win10 result.
            let cache = cache.as_deref().expect("a cache");
            *mesh = cache.mesh.clone();
            *face_colors = cache.face_colors.clone();
        } else {
            // TriangleMesh stats_mesh(mesh); stats.manifold(), stats.has_open_edges()
            let mut face_neighbors: Vec<Vec3i> = Vec::new();
            let nm_stats = its_quick_diagnostics(&mesh.0, Some(&mut face_neighbors));
            let manifold = nm_stats.non_manifold_edges == 0 && nm_stats.non_manifold_vertices == 0;
            let has_open_edges = nm_stats.open_edges > 0;
            if !manifold || has_open_edges {
                if settings.mesh_repair_decision == MeshRepairDecision::RepairAndImport {
                    if skip_win10 {
                        // skipping Windows 3D mesh repair because a previous attempt failed
                    } else {
                        let mut repaired_its = indexed_triangle_set::default();
                        let mut repair_error = String::new();
                        let repair_start = Instant::now();
                        let repair_timed_out = Cell::new(false);
                        let repaired = match &settings.mesh_repair_callback {
                            Some(callback) => callback(
                                &mesh.0,
                                &mut repaired_its,
                                &|message: Option<&str>, _percent: u32| {
                                    report(5, message.unwrap_or("Repairing mesh"));
                                },
                                &|| {
                                    if cancelled() {
                                        return true;
                                    }
                                    if settings.mesh_repair_timeout > Duration::from_secs(0) && repair_start.elapsed() >= settings.mesh_repair_timeout {
                                        repair_timed_out.set(true);
                                        return true;
                                    }
                                    false
                                },
                                &mut repair_error,
                            ),
                            None => false,
                        };
                        if cancelled() {
                            return false;
                        }
                        if repaired {
                            if !resample_face_colors(mesh, face_colors, TriMesh::from(repaired_its)) {
                                return false;
                            }
                            if let Some(cache) = cache.as_deref_mut() {
                                cache.win10_attempt = MeshRepairAttempt::Succeeded;
                                cache.mesh = mesh.clone();
                                cache.face_colors = face_colors.clone();
                            }
                        } else if repair_timed_out.get() {
                            if let Some(cache) = cache.as_deref_mut() {
                                cache.win10_attempt = MeshRepairAttempt::Failed;
                            }
                        } else if let Some(cache) = cache.as_deref_mut() {
                            cache.win10_attempt = MeshRepairAttempt::Failed;
                        }
                    }
                }
            }
        }

        if !cgal_utils::is_mesh_halfedge_compatible(mesh) {
            // repair_and_resample
            let mut repaired_mesh: Option<TriMesh> = None;
            if !RepairMesh(mesh, &mut repaired_mesh, &None, &None, &RepairSetting::default(), pmp) {
                return false;
            }
            if cancelled() {
                return false;
            }
            if !resample_face_colors(mesh, face_colors, repaired_mesh.expect("RepairMesh gives a mesh when it succeeds")) {
                return false;
            }
        }

        if cancelled() {
            return false;
        }
        // store_prepared_mesh
        if let Some(cache) = cache.as_deref_mut() {
            cache.mesh = mesh.clone();
            cache.face_colors = face_colors.clone();
            cache.has_prepared_mesh = true;
            cache.key_input_face_count = cache_input_face_count;
            cache.key_oversampling_iters = settings.oversampling_iters;
            cache.key_oversampling_min_face_count = settings.oversampling_min_face_count;
            cache.key_oversampling_max_face_count = settings.oversampling_max_face_count;
        }
    }
    drop(cache_guard);

    report(20, "Color clustering");
    if cancelled() {
        return false;
    }

    // Clustering
    let mut cluster_centers: Vec<RGB>;
    *out_clustered_face_colors = face_colors.clone();
    let mut clustered_face_labels: Vec<usize> = vec![0; face_colors.len()];
    let adaptive_cluster = settings.target_colors_num == 0;

    if adaptive_cluster {
        let mut para = ClusterParameters::default();
        para.max_color_distance = settings.max_color_distance;
        para.max_cluster_k = settings.max_cluster_k;
        para.cancel_callback = cancel_callback.clone();
        cluster_centers = cluster_adaptive(face_colors, &para);
        if cancelled() {
            return false;
        }
    } else {
        let mut para = ClusterParameters::default();
        para.cluster_k = settings.target_colors_num;
        para.cancel_callback = cancel_callback.clone();
        cluster_centers = cluster_k_means(face_colors, &para);
        if cancelled() {
            return false;
        }
    }

    if cluster_centers.is_empty() {
        return false;
    }

    report(40, "Assigning cluster labels");
    if cancelled() {
        return false;
    }

    // Assign each face to nearest cluster center
    {
        let mut done: usize = 0;
        let mut cancel_requested = false;
        let total = mesh.indices.len();
        let interval = (total / 20).max(1);
        for fid in 0..total {
            if cancel_requested {
                break;
            }
            let mut nearest_id: usize = 0;
            calc_nearest_color_id(&cluster_centers, &face_colors[fid], &mut nearest_id);
            clustered_face_labels[fid] = nearest_id;
            out_clustered_face_colors[fid] = cluster_centers[nearest_id];
            done += 1;
            let cnt = done;
            if cnt % interval == 0 {
                if cancelled() {
                    cancel_requested = true;
                    break;
                }
            }
        }
        if cancel_requested || cancelled() {
            return false;
        }
    }
    if adaptive_cluster {
        if !discard_unused_cluster_centers(&mut cluster_centers, &mut clustered_face_labels, "cluster assignment") {
            return false;
        }
    } else {
        ensure_all_cluster_centers_used(face_colors, &cluster_centers, &mut clustered_face_labels, "cluster assignment");
    }

    report(65, "Smoothing colors");
    if cancelled() {
        return false;
    }

    let mut smooth_parameters = SmoothParameters::default();
    smooth_parameters.smooth_weight = settings.smooth_weight;
    if !smooth_region_with_labels(mesh, &mut clustered_face_labels, &smooth_parameters) {
        return false;
    }
    if adaptive_cluster {
        if !discard_unused_cluster_centers(&mut cluster_centers, &mut clustered_face_labels, "color smoothing") {
            return false;
        }
    } else {
        ensure_all_cluster_centers_used(face_colors, &cluster_centers, &mut clustered_face_labels, "color smoothing");
    }

    report(90, "Updating face colors");
    if cancelled() {
        return false;
    }

    for i in 0..out_clustered_face_colors.len() {
        out_clustered_face_colors[i] = cluster_centers[clustered_face_labels[i]];
    }

    report(100, "Completed");
    true
}

/// TextureToColor.cpp:864-1045: the texture sampled over the faces of `texture_mesh` (subdivided
/// first so that the faces are small enough), the colors clustered and smoothed. `uv_coords` has
/// the three UVs of every face. `color_mesh` and `face_colors` are the result. False when the
/// input is not usable (no faces, no UVs, an empty texture) or the work was cancelled.
#[allow(clippy::too_many_arguments)]
pub fn TextureToColor(
    texture_mesh: &TriMesh,
    texture_mesh_uv_coords: &[Vec<Vec2f>],
    texture: &Mat,
    color_mesh: &mut TriMesh,
    face_colors: &mut Vec<[usize; 3]>,
    settings: &TextureToColorSettings,
    progress_callback: &AlgoProgressCallback,
    cancel_callback: &AlgoCancelCallback,
    pmp: &dyn PolygonMeshProcessing,
) -> bool {
    let report = |pct: i32, msg: &str| {
        if let Some(cb) = progress_callback {
            cb(AlgoProgress::new(pct, msg));
        }
    };
    let sub_report = |sub_pct: i32, range_start: i32, range_end: i32, msg: &str| {
        let pct = range_start + sub_pct * (range_end - range_start) / 100;
        report(pct, msg);
    };
    let cancelled = || -> bool {
        match cancel_callback {
            Some(cb) => cb(),
            None => false,
        }
    };

    color_mesh.clear();
    face_colors.clear();

    report(0, "Initializing");
    if cancelled() {
        return false;
    }

    let mut use_prepared = false;
    if let Some(cache) = &settings.mesh_repair_cache {
        let mut cache = cache.lock().expect("the mesh repair cache");
        use_prepared = cache.matches_input(
            texture_mesh.indices.len(),
            settings.oversampling_iters,
            settings.oversampling_min_face_count,
            settings.oversampling_max_face_count,
        );
        if cache.has_prepared_mesh && !use_prepared {
            cache.invalidate_prepared_mesh();
        }
        if use_prepared {
            *color_mesh = cache.mesh.clone();
            *face_colors = cache.face_colors.clone();
        }
    }
    if !use_prepared {
        if texture_mesh.indices.is_empty() {
            return false;
        }
        if texture.empty() {
            return false;
        }
        if texture.channels() < 3 {
            return false;
        }
        if texture_mesh_uv_coords.len() != texture_mesh.indices.len() {
            return false;
        }
        for fid in 0..texture_mesh.indices.len() {
            if texture_mesh_uv_coords[fid].len() != 3 {
                return false;
            }
        }
        *color_mesh = texture_mesh.clone();

        report(5, "Oversampling");
        if cancelled() {
            return false;
        }

        // Step 1: Oversampling (subdivision while propagating UVs)
        let mut color_mesh_uv_coords: Vec<FaceUVArray> = Vec::with_capacity(texture_mesh_uv_coords.len());
        for i in 0..texture_mesh_uv_coords.len() {
            color_mesh_uv_coords.push([texture_mesh_uv_coords[i][0], texture_mesh_uv_coords[i][1], texture_mesh_uv_coords[i][2]]);
        }
        {
            // Estimate total iterations and map each iteration's sub-progress to the [5, 25] range
            let mut estimated_iters: usize = 0;
            if settings.oversampling_iters > 0 {
                estimated_iters = settings.oversampling_iters;
            } else {
                let mut fc = color_mesh.facets_count();
                while fc < settings.oversampling_min_face_count {
                    fc *= 4;
                    estimated_iters += 1;
                }
                if estimated_iters == 0 {
                    estimated_iters = 1;
                }
            }

            let iter_progress = |iter: usize, pct: i32| {
                let iter_start = (iter * 100 / estimated_iters) as i32;
                let iter_end = ((iter + 1) * 100 / estimated_iters) as i32;
                let sub_pct = iter_start + pct * (iter_end - iter_start) / 100;
                sub_report(sub_pct, 5, 25, "Oversampling");
            };

            if settings.oversampling_iters > 0 {
                let mut i: usize = 0;
                while i < settings.oversampling_iters && (color_mesh.facets_count() as f64) * 4.0 < settings.oversampling_max_face_count as f64 {
                    if cancelled() {
                        return false;
                    }
                    linear_subdivision(color_mesh, &mut color_mesh_uv_coords, Some(&|pct: i32| iter_progress(i, pct)));
                    i += 1;
                }
            } else {
                let mut iter: usize = 0;
                while color_mesh.facets_count() < settings.oversampling_min_face_count {
                    if cancelled() {
                        return false;
                    }
                    let this_iter = iter;
                    iter += 1;
                    linear_subdivision(color_mesh, &mut color_mesh_uv_coords, Some(&|pct: i32| iter_progress(this_iter, pct)));
                }
            }
        }

        face_colors.resize(color_mesh.indices.len(), [0; 3]);

        report(25, "Computing face colors");
        if cancelled() {
            return false;
        }

        // Step 2: Compute each face's color (7-point Gaussian quadrature + bilinear interpolation sampling)
        {
            let mut done_faces: usize = 0;
            let mut cancel_requested = false;
            let total_faces = color_mesh.indices.len();
            let report_interval = (total_faces / 20).max(1);
            for fid in 0..total_faces {
                if cancel_requested {
                    break;
                }
                face_colors[fid] = sample_face_color(&color_mesh_uv_coords[fid], texture);
                done_faces += 1;
                let cnt = done_faces;
                if cnt % report_interval == 0 {
                    if cancelled() {
                        cancel_requested = true;
                        break;
                    }
                    sub_report((cnt * 100 / total_faces) as i32, 25, 40, "Computing face colors");
                }
            }
            if cancel_requested || cancelled() {
                return false;
            }
        }
    }

    // Map progress from repair_cluster_smooth's [0,100] to TextureToColor's [40,100]
    let rcs_progress: AlgoProgressCallback = match progress_callback {
        Some(cb) => {
            let cb = cb.clone();
            Some(Arc::new(move |p: AlgoProgress| {
                let mapped_pct = 40 + p.percent * 60 / 100;
                cb(AlgoProgress { percent: mapped_pct, message: p.message });
            }))
        }
        None => None,
    };

    let mut clustered_face_colors: Vec<RGB> = Vec::new();
    if !repair_cluster_smooth(
        color_mesh,
        face_colors,
        &mut clustered_face_colors,
        settings,
        &rcs_progress,
        cancel_callback,
        "TextureToColor",
        texture_mesh.indices.len(),
        pmp,
    ) {
        return false;
    }

    *face_colors = clustered_face_colors;
    report(100, "Completed");
    true
}

/// TextureToColor.cpp:1047-1131: pre-computed per-face colors turned into a clustered color mesh
/// (no texture/UV). A low-poly mesh with a color per vertex (`vertex_colors`, RGBA in [0, 1], as
/// many as the mesh has vertices) is split along the boundaries of its quantized vertex colors;
/// everything else goes through repair, clustering and smoothing as in `TextureToColor`. In C++
/// `vertex_colors` defaults to none.
#[allow(clippy::too_many_arguments)]
pub fn ClusterAndSmooth(
    mesh: &TriMesh,
    input_face_colors: &[[usize; 3]],
    out_mesh: &mut TriMesh,
    out_face_colors: &mut Vec<[usize; 3]>,
    settings: &TextureToColorSettings,
    progress_callback: &AlgoProgressCallback,
    cancel_callback: &AlgoCancelCallback,
    vertex_colors: &[[f32; 4]],
    pmp: &dyn PolygonMeshProcessing,
) -> bool {
    let report = |pct: i32, msg: &str| {
        if let Some(cb) = progress_callback {
            cb(AlgoProgress::new(pct, msg));
        }
    };
    let cancelled = || -> bool {
        match cancel_callback {
            Some(cb) => cb(),
            None => false,
        }
    };

    *out_mesh = mesh.clone();
    out_face_colors.clear();

    if mesh.indices.is_empty() || input_face_colors.is_empty() {
        return false;
    }
    // a face color count that is not the face count is clamped below (C++ logs a warning)

    report(0, "Initializing");
    if cancelled() {
        return false;
    }

    // Prepare face colors aligned to mesh size
    let mut face_colors: Vec<RGB> = vec![[0; 3]; out_mesh.indices.len()];
    for i in 0..out_mesh.indices.len() {
        if i < input_face_colors.len() {
            face_colors[i] = input_face_colors[i];
        } else {
            face_colors[i] = [128, 128, 128];
        }
    }

    // Low-poly vertex-color meshes take the legacy OBJ import route: quantize the
    // vertex colors, then split only across cluster boundaries. Colors are exact
    // cluster centers afterwards, so repair / re-clustering / smoothing are skipped.
    // A vertex color count that disagrees with the mesh falls through to the generic
    // pipeline below rather than failing the import outright.
    if !vertex_colors.is_empty() && vertex_colors.len() == out_mesh.vertices.len() && out_mesh.facets_count() < settings.oversampling_min_face_count {
        report(10, "Quantizing vertex colors");
        let mut cluster_centers: Vec<RGB> = Vec::new();
        let mut vertex_cluster_ids: Vec<usize> = Vec::new();
        if !quantize_vertex_colors(vertex_colors, settings, cancel_callback, &mut cluster_centers, &mut vertex_cluster_ids) {
            return false;
        }
        if cancelled() {
            return false;
        }

        report(50, "Splitting color boundaries");
        if !adaptive_split_by_vertex_clusters(out_mesh, &vertex_cluster_ids, &cluster_centers, &mut face_colors) {
            return false;
        }
        if cancelled() {
            return false;
        }

        *out_face_colors = face_colors;
        report(100, "Completed");
        return true;
    }

    let mut clustered_face_colors: Vec<RGB> = Vec::new();
    if !repair_cluster_smooth(
        out_mesh,
        &mut face_colors,
        &mut clustered_face_colors,
        settings,
        progress_callback,
        cancel_callback,
        "ClusterAndSmooth",
        mesh.indices.len(),
        pmp,
    ) {
        return false;
    }

    *out_face_colors = clustered_face_colors;
    true
}
