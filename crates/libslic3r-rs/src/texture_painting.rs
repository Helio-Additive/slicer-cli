//! Port of `TexturePainting.hpp` / `TexturePainting.cpp`: a textured mesh, or a mesh with colors per
//! face or per vertex, turned into a painted mesh of a few colors (through `tex2color`), the colors
//! matched to filaments, and the painted mesh applied to a model volume as its multi-material
//! painting.
//!
//! C++ Reference:
//! - src/libslic3r/TexturePainting.hpp
//! - src/libslic3r/TexturePainting.cpp
//!
//! The functions stand in the order of the .cpp; the types of the .hpp come first.
//!
//! Floating point follows the reference build (clang, `-ffp-contract=on`): `a * b + c` inside one
//! source expression is one fused operation, written `mul_add` here.
//!
//! FIDELITY-NOTE: `cv::Mat` is `texture_to_color::Mat` (see that file), with the 8-bit matrices of
//! three or four channels this unit makes. `cv::cvtColor(RGBA2BGR / RGB2BGR)`, `cv::Mat::zeros`,
//! a region's `copyTo` and `at<cv::Vec3b>` are written out over it.
//!
//! FIDELITY-NOTE: `cv::imdecode` (OpenCV imgcodecs: PNG, JPEG and the other formats OpenCV reads)
//! has no counterpart in the crate. A `TextureImage` that holds encoded bytes (`width` or `height`
//! not positive) decodes to an empty matrix here, as `imdecode` gives for bytes it cannot read; a
//! caller that has such an image decodes it and passes the pixels. A `TextureImage` with fewer
//! bytes than `width * height * channels` (C++ reads past the end of the buffer) also decodes to an
//! empty matrix.
//!
//! FIDELITY-NOTE: `RepairMesh` (CGAL), reached from `TextureToColor` and `ClusterAndSmooth` for a
//! mesh that is not halfedge-compatible, is passed in as `cgal_utils::PolygonMeshProcessing`, as
//! `texture_to_color.rs` takes it.
//!
//! FIDELITY-NOTE: `MeshRepairCachePtr` (a `std::unique_ptr` with `MeshRepairCacheDeleter`) and the
//! raw `tex2color::MeshRepairCache*` of the settings are `Option<Arc<Mutex<MeshRepairCache>>>`, as
//! in `texture_to_color.rs`; the deleter is the `Drop` of the `Arc`.
//!
//! FIDELITY-NOTE: `ModelVolume` and `ModelObject` (`apply_painted_mesh_to_volume`). `crate::model`
//! has no volumes, so this file defines stand-ins with exactly the members this unit reads and
//! writes (section "Model stand-ins"), each citing the Model.hpp / Model.cpp / TriangleMesh.cpp
//! lines it follows. `ModelVolume::calculate_convex_hull` needs `TriangleMesh::convex_hull_3d`
//! (qhull), which the crate has no port of: the caller passes it to `apply_painted_mesh_to_volume`.
//!
//! What C++ writes to the log is left out (`apply_painted_mesh_to_volume` measures the drift of the
//! bounding box only to log it).
//!
//! Nothing in the slicing path calls this unit; its callers in C++ are the texture import dialog
//! and the 3MF / OBJ import of the desktop app.

use std::collections::{BTreeMap, BTreeSet};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use crate::model::FacetsAnnotation;
use crate::texture_to_color::callbacks::{AlgoCancelCallback, AlgoProgress, AlgoProgressCallback};
use crate::texture_to_color::cgal_utils::PolygonMeshProcessing;
use crate::texture_to_color::color_utils::calc_rgb_color_difference_by_ciede2000;
use crate::texture_to_color::texture_to_color::{
    self as tex2color, quantize_colors, ClusterAndSmooth, Mat, MeshRepairAttempt, MeshRepairCache, MeshRepairCallback, TextureToColor,
    TextureToColorSettings,
};
use crate::texture_to_color::tri_mesh::TriMesh;
use crate::triangle_mesh::{Triangle, TriangleMesh, Vec2f};
use crate::triangle_selector::{EnforcerBlockerType, TriangleSelector};
use crate::triangle_set_sampling::{indexed_triangle_set, Vec3d, Vec3f, Vec3i};

/// Model.hpp:718 `constexpr size_t MAX_PHYSICAL_FILAMENT_COUNT = 32;`
pub const MAX_PHYSICAL_FILAMENT_COUNT: usize = 32;
/// Model.hpp:719 `constexpr size_t MAX_MIXED_FILAMENT_COUNT = 64;`
pub const MAX_MIXED_FILAMENT_COUNT: usize = 64;
/// Model.hpp:720 `constexpr size_t MAX_FILAMENT_COUNT = MAX_PHYSICAL_FILAMENT_COUNT + MAX_MIXED_FILAMENT_COUNT;`
pub const MAX_FILAMENT_COUNT: usize = MAX_PHYSICAL_FILAMENT_COUNT + MAX_MIXED_FILAMENT_COUNT;
/// Model.hpp:763 `EnforcerBlockerType::ExtruderMax = int8_t(MAX_FILAMENT_COUNT)`. The pin's value;
/// `triangle_selector::EnforcerBlockerType::EXTRUDER_MAX` is 32, the value of an earlier Bambu
/// Studio, and is left as it is there (it is on the slicing path).
pub const EXTRUDER_MAX: EnforcerBlockerType = EnforcerBlockerType(MAX_FILAMENT_COUNT as i8);

// ---------------------------------------------------------------------------------------------
// TexturePainting.hpp
// ---------------------------------------------------------------------------------------------

/// TexturePainting.hpp:28 `using MeshRepairCachePtr = std::unique_ptr<tex2color::MeshRepairCache,
/// MeshRepairCacheDeleter>;` (see the FIDELITY-NOTE of the file). `None` is the null pointer.
pub type MeshRepairCachePtr = Option<Arc<Mutex<MeshRepairCache>>>;

/// TexturePainting.hpp:31-36
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TextureImage {
    pub width: i32,
    pub height: i32,
    pub channels: i32,
    pub data: Vec<u8>,
}

impl Default for TextureImage {
    /// TexturePainting.hpp:32-35, the member initializers
    fn default() -> Self {
        TextureImage { width: 0, height: 0, channels: 4, data: Vec::new() }
    }
}

/// TexturePainting.hpp:38-73
#[derive(Debug, Clone, Default)]
pub struct TexturedMesh {
    pub vertices: Vec<[f32; 3]>,
    pub indices: Vec<[i32; 3]>,
    pub uvs: Vec<[f32; 2]>,
    pub textures: Vec<TextureImage>,
    pub material_ids: Vec<i32>,
    /// material index -> index in textures[] (-1 if no texture, use material_colors)
    pub material_texture_map: Vec<i32>,
    /// per-material baseColorFactor (RGBA 0-1), indexed by material index
    pub material_colors: Vec<[f32; 4]>,

    /// Per-face independent UV support (for OBJ where the same vertex can have different texture
    /// coordinates on different faces): the UV coordinate pool
    pub uv_coords: Vec<[f32; 2]>,
    /// per-face UV indices into uv_coords
    pub uv_indices: Vec<[i32; 3]>,

    /// Pre-computed per-face colors (e.g. from OBJ vertex colors or MTL Kd), {R, G, B} in [0..255].
    /// When non-empty, the pipeline consumes these instead of sampling a texture.
    pub precomputed_face_colors: Vec<[usize; 3]>,

    /// Per-vertex colors from OBJ (RGBA, [0..1]), indexed by vertex index.
    pub precomputed_vertex_colors: Vec<[f32; 4]>,

    /// The source already stores a small palette of exact colors (standard 3MF color groups).
    pub is_standard_3mf: bool,
}

impl TexturedMesh {
    /// TexturePainting.hpp:54
    pub fn has_face_uvs(&self) -> bool {
        !self.uv_indices.is_empty() && !self.uv_coords.is_empty()
    }
}

/// TexturePainting.hpp:75-83
#[derive(Debug, Clone, Default, PartialEq)]
pub struct PaintedMesh {
    pub vertices: Vec<[f32; 3]>,
    pub indices: Vec<[i32; 3]>,
    /// per-face RGB [0..255]
    pub face_colors: Vec<[usize; 3]>,
    pub cluster_colors: Vec<[usize; 3]>,
    /// Only filled by standard_3mf_colors_to_painting(): source RGB -> index into cluster_colors
    /// (-1 if unused).
    pub original_color_to_cluster: BTreeMap<[usize; 3], i32>,
}

/// TexturePainting.hpp:85 `std::function<void(int percent, const char* message)>`. `None` is the
/// empty function.
pub type PaintProgressCallback = Option<Arc<dyn Fn(i32, &str) + Send + Sync>>;
/// TexturePainting.hpp:86 `std::function<bool()>`. `None` is the empty function.
pub type PaintCancelCallback = Option<Arc<dyn Fn() -> bool + Send + Sync>>;
/// TexturePainting.hpp:87-91, the same signature as `tex2color::MeshRepairCallback`
pub type PaintMeshRepairCallback = MeshRepairCallback;

/// TexturePainting.hpp:97-100 `enum class TexturePaintingSettings::MeshRepairDecision`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MeshRepairDecision {
    ImportWithoutRepair,
    RepairAndImport,
}

/// TexturePainting.hpp:93-108
#[derive(Clone)]
pub struct TexturePaintingSettings {
    pub target_colors_num: usize,
    pub smooth_weight: f64,
    pub oversampling_iters: usize,
    pub mesh_repair_decision: MeshRepairDecision,
    pub mesh_repair_callback: PaintMeshRepairCallback,
    /// Max wait for Windows 3D repair. Timed-out repair is cancelled and the original mesh is kept.
    /// Zero disables the timeout.
    pub mesh_repair_timeout: Duration,
    pub mesh_repair_cache: Option<Arc<Mutex<MeshRepairCache>>>,
}

impl Default for TexturePaintingSettings {
    /// TexturePainting.hpp:94-107, the member initializers
    fn default() -> Self {
        TexturePaintingSettings {
            target_colors_num: 4,
            smooth_weight: 0.5,
            oversampling_iters: 0,
            mesh_repair_decision: MeshRepairDecision::ImportWithoutRepair,
            mesh_repair_callback: None,
            mesh_repair_timeout: Duration::from_secs(0),
            mesh_repair_cache: None,
        }
    }
}

/// TexturePainting.hpp:128-134
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FilamentMatch {
    pub cluster_index: i32,
    pub filament_index: i32,
    pub delta_e: f64,
    pub cluster_color: [usize; 3],
    pub filament_color: [f32; 4],
}

impl Default for FilamentMatch {
    /// TexturePainting.hpp:129-133, the member initializers
    fn default() -> Self {
        FilamentMatch { cluster_index: -1, filament_index: -1, delta_e: 0.0, cluster_color: [0, 0, 0], filament_color: [0.0, 0.0, 0.0, 1.0] }
    }
}

// ---------------------------------------------------------------------------------------------
// Model stand-ins (see the FIDELITY-NOTE of the file)
// ---------------------------------------------------------------------------------------------

/// `TriangleMesh` as a `ModelVolume` holds it, the members this unit reads and writes:
/// `its`, `m_stats.min` / `m_stats.max` (TriangleMesh.hpp:50-51) and `m_init_shift`
/// (TriangleMesh.hpp:172).
#[derive(Debug, Clone, Default)]
pub struct VolumeMesh {
    pub its: indexed_triangle_set,
    pub min: Vec3f,
    pub max: Vec3f,
    pub init_shift: Vec3d,
}

impl VolumeMesh {
    /// TriangleMesh.cpp:94-98 `TriangleMesh(indexed_triangle_set &&its)`: of `fill_initial_stats`
    /// (TriangleMesh.cpp:46-59) the bounding box (`update_bounding_box`, TriangleMesh.cpp:38-44,
    /// over `bounding_box(its)`, TriangleMesh.hpp:373-386).
    pub fn new(its: indexed_triangle_set) -> Self {
        let (mut min, mut max) = (Vec3f::zeros(), Vec3f::zeros());
        if let Some(&first) = its.vertices.first() {
            let (mut bmin, mut bmax) = (first, first);
            for p in its.vertices.iter() {
                // Eigen cwiseMin / cwiseMax: `std::min(p, b)` / `std::max(p, b)` per component
                for k in 0..3 {
                    bmin[k] = if bmin[k] < p[k] { bmin[k] } else { p[k] };
                    bmax[k] = if p[k] < bmax[k] { bmax[k] } else { p[k] };
                }
            }
            min = bmin;
            max = bmax;
        }
        VolumeMesh { its, min, max, init_shift: Vec3d::zeros() }
    }

    /// TriangleMesh.hpp `bool empty() const { return its.empty(); }`: no faces
    pub fn empty(&self) -> bool {
        self.its.indices.is_empty()
    }

    /// TriangleMesh.cpp:296-304 `TriangleMesh::translate(const Vec3f &displacement)`
    pub fn translate(&mut self, displacement: Vec3f) {
        if displacement.x != 0.0 || displacement.y != 0.0 || displacement.z != 0.0 {
            for v in self.its.vertices.iter_mut() {
                *v += displacement;
            }
            self.min += displacement;
            self.max += displacement;
        }
    }

    /// TriangleMesh.hpp:165
    pub fn set_init_shift(&mut self, offset: Vec3d) {
        self.init_shift = offset;
    }

    /// TriangleMesh.cpp:492-499 `bounding_box()`: `m_stats.min` and `m_stats.max` as doubles; its
    /// center (`BoundingBoxf3::center()`), `(min + max) * 0.5`
    pub fn bounding_box_center(&self) -> Vec3d {
        let (min, max) = (self.min.cast::<f64>(), self.max.cast::<f64>());
        Vec3d::new((min.x + max.x) * 0.5, (min.y + max.y) * 0.5, (min.z + max.z) * 0.5)
    }
}

/// Model.hpp:929-944 `ModelVolume::Source`, the members this unit reads
#[derive(Debug, Clone, Default, PartialEq)]
pub struct ModelVolumeSource {
    pub mesh_offset: Vec3d,
    pub is_converted_from_inches: bool,
    pub is_converted_from_meters: bool,
}

/// Model.hpp:443 `ModelObject::invalidate_bounding_box()`, the members it clears
#[derive(Debug, Clone, PartialEq)]
pub struct ModelObject {
    pub bounding_box_valid: bool,
    pub raw_bounding_box_valid: bool,
    pub raw_mesh_bounding_box_valid: bool,
}

impl ModelObject {
    /// Model.hpp:443
    pub fn invalidate_bounding_box(&mut self) {
        self.bounding_box_valid = false;
        self.raw_bounding_box_valid = false;
        self.raw_mesh_bounding_box_valid = false;
    }
}

/// Model.hpp:923 `class ModelVolume`, the members this unit uses
#[derive(Debug, Clone)]
pub struct ModelVolume {
    /// Model.hpp:929-944
    pub source: ModelVolumeSource,
    /// `m_mesh` (`std::shared_ptr<const TriangleMesh>`)
    pub mesh: VolumeMesh,
    /// `m_convex_hull`; `None` is the null pointer
    pub convex_hull: Option<VolumeMesh>,
    /// `m_convex_hull_2d` (a `Polygon`), its points
    pub convex_hull_2d: Vec<crate::Point>,
    /// `m_transformation.get_offset()`
    pub offset: Vec3d,
    pub supported_facets: FacetsAnnotation,
    pub seam_facets: FacetsAnnotation,
    pub mmu_segmentation_facets: FacetsAnnotation,
    pub fuzzy_skin_facets: FacetsAnnotation,
    /// `ModelVolume::object` (`get_object()`); `None` is the null pointer
    pub object: Option<ModelObject>,
}

impl ModelVolume {
    /// Model.hpp:988
    pub fn mesh(&self) -> &VolumeMesh {
        &self.mesh
    }

    /// Model.hpp:991 `set_mesh(TriangleMesh &&mesh)`
    pub fn set_mesh(&mut self, mesh: VolumeMesh) {
        self.mesh = mesh;
    }

    /// Model.cpp:3531-3535 `m_convex_hull = make_shared<TriangleMesh>(mesh().convex_hull_3d())`;
    /// `convex_hull_3d` is the caller's (see the FIDELITY-NOTE of the file).
    pub fn calculate_convex_hull(&mut self, convex_hull_3d: &dyn Fn(&indexed_triangle_set) -> indexed_triangle_set) {
        self.convex_hull = Some(VolumeMesh::new(convex_hull_3d(&self.mesh.its)));
    }

    /// Model.cpp:3832-3835 `translate(displacement)`: `set_offset(get_offset() + displacement)`
    pub fn translate(&mut self, displacement: Vec3d) {
        self.offset += displacement;
    }

    /// Model.cpp:3513-3529
    pub fn center_geometry_after_creation(&mut self, update_source_offset: bool) {
        let shift = self.mesh().bounding_box_center();
        if !is_approx_zero(&shift) {
            let d = Vec3f::new(-(shift[0] as f32), -(shift[1] as f32), -(shift[2] as f32));
            self.mesh.translate(d);
            self.mesh.set_init_shift(shift);
            if let Some(hull) = self.convex_hull.as_mut() {
                hull.translate(d);
            }
            self.translate(shift);
        }

        if update_source_offset {
            self.source.mesh_offset = shift;
        }
    }

    /// Model.hpp:1110-1113
    pub fn invalidate_convex_hull_2d(&mut self) {
        self.convex_hull_2d.clear();
    }

    /// Model.hpp:1025
    pub fn get_object(&mut self) -> Option<&mut ModelObject> {
        self.object.as_mut()
    }
}

/// Eigen `v.isApprox(Vec3d::Zero())`: `(v - 0).squaredNorm() <= prec^2 * min(v.squaredNorm(), 0)`,
/// true exactly when the squared norm is 0 (or underflows to it)
fn is_approx_zero(v: &Vec3d) -> bool {
    let (xx, yy, zz) = (v.x * v.x, v.y * v.y, v.z * v.z);
    xx + yy + zz <= 0.0
}

// ---------------------------------------------------------------------------------------------
// TexturePainting.cpp
// ---------------------------------------------------------------------------------------------

/// TexturePainting.cpp:29-32
pub fn make_mesh_repair_cache() -> MeshRepairCachePtr {
    Some(Arc::new(Mutex::new(MeshRepairCache::default())))
}

/// TexturePainting.cpp:34-59: the texture as BGR pixels, 8 bits per channel; an empty matrix
/// when it cannot be decoded. See the FIDELITY-NOTE of the file on encoded images.
pub fn decode_texture_image(img: &TextureImage) -> Mat {
    if img.data.is_empty() {
        return Mat::default();
    }

    // Raw encoded image data (PNG/JPEG) from glTF loader: width == -1
    if img.width <= 0 || img.height <= 0 {
        // cv::imdecode(raw, cv::IMREAD_COLOR), see the FIDELITY-NOTE of the file
        return Mat::default();
    }

    let src_channels: usize = if img.channels == 4 { 4 } else { 3 };
    let (rows, cols) = (img.height as usize, img.width as usize);
    if img.data.len() < rows * cols * src_channels {
        return Mat::default();
    }

    // cv::cvtColor(src, bgr, COLOR_RGBA2BGR / COLOR_RGB2BGR)
    if img.channels == 4 || img.channels == 3 {
        let mut bgr = Vec::with_capacity(rows * cols * 3);
        for p in 0..rows * cols {
            let s = &img.data[p * src_channels..p * src_channels + 3];
            bgr.push(s[2]);
            bgr.push(s[1]);
            bgr.push(s[0]);
        }
        Mat::new(img.height, img.width, 3, bgr)
    } else {
        Mat::default()
    }
}

/// TexturePainting.cpp:61-110: the mesh of `tex2color` and the three UVs of each face, from the
/// per-face UVs when the mesh has them, else from the per-vertex ones; (0, 0) where an index is
/// out of range
pub fn build_tex2color_mesh(textured: &TexturedMesh, mesh: &mut TriMesh, uv_coords: &mut Vec<Vec<Vec2f>>) {
    let nv = textured.vertices.len();
    let nf = textured.indices.len();

    mesh.vertices.resize(nv, Vec3f::zeros());
    for i in 0..nv {
        mesh.vertices[i] = Vec3f::new(textured.vertices[i][0], textured.vertices[i][1], textured.vertices[i][2]);
    }

    mesh.indices.resize(nf, Vec3i::zeros());
    for i in 0..nf {
        mesh.indices[i] = Vec3i::new(textured.indices[i][0], textured.indices[i][1], textured.indices[i][2]);
    }

    uv_coords.resize(nf, Vec::new());
    for fi in 0..nf {
        uv_coords[fi].resize(3, Vec2f::zeros());
        for vi in 0..3 {
            if textured.has_face_uvs() {
                let uv_idx = textured.uv_indices[fi][vi];
                if uv_idx >= 0 && (uv_idx as usize) < textured.uv_coords.len() {
                    uv_coords[fi][vi] = Vec2f::new(textured.uv_coords[uv_idx as usize][0], textured.uv_coords[uv_idx as usize][1]);
                } else {
                    uv_coords[fi][vi] = Vec2f::new(0.0, 0.0);
                }
            } else {
                let vtx_idx = textured.indices[fi][vi];
                if vtx_idx >= 0 && (vtx_idx as usize) < textured.uvs.len() {
                    uv_coords[fi][vi] = Vec2f::new(textured.uvs[vtx_idx as usize][0], textured.uvs[vtx_idx as usize][1]);
                } else {
                    uv_coords[fi][vi] = Vec2f::new(0.0, 0.0);
                }
            }
        }
    }
}

/// TexturePainting.cpp:112-136: the painted mesh from a `tex2color` mesh and its face colors; the
/// cluster colors are the distinct face colors, sorted (`std::set`)
pub fn extract_painted_mesh(color_mesh: &TriMesh, face_colors: &[[usize; 3]], painted: &mut PaintedMesh) {
    let nv = color_mesh.vertices.len();
    let nf = color_mesh.indices.len();

    painted.vertices.resize(nv, [0.0; 3]);
    for i in 0..nv {
        let v = &color_mesh.vertices[i];
        painted.vertices[i] = [v.x, v.y, v.z];
    }

    painted.indices.resize(nf, [0; 3]);
    for i in 0..nf {
        let f = &color_mesh.indices[i];
        painted.indices[i] = [f[0], f[1], f[2]];
    }

    painted.face_colors = face_colors.to_vec();

    let unique_colors: BTreeSet<[usize; 3]> = face_colors.iter().copied().collect();
    painted.cluster_colors = unique_colors.into_iter().collect();
}

/// TexturePainting.cpp:138-157
pub fn make_tex2color_settings(settings: &TexturePaintingSettings) -> TextureToColorSettings {
    let mut algo_settings = TextureToColorSettings::default();
    algo_settings.target_colors_num = settings.target_colors_num;
    algo_settings.smooth_weight = settings.smooth_weight;
    algo_settings.oversampling_iters = settings.oversampling_iters;
    algo_settings.mesh_repair_decision = match settings.mesh_repair_decision {
        MeshRepairDecision::RepairAndImport => tex2color::MeshRepairDecision::RepairAndImport,
        MeshRepairDecision::ImportWithoutRepair => tex2color::MeshRepairDecision::ImportWithoutRepair,
    };
    algo_settings.mesh_repair_callback = settings.mesh_repair_callback.clone();
    algo_settings.mesh_repair_timeout = settings.mesh_repair_timeout;
    algo_settings.mesh_repair_cache = settings.mesh_repair_cache.clone();
    algo_settings
}

/// TexturePainting.cpp:159-174: true when the cache holds a prepared mesh of this input; a cache
/// with a prepared mesh of another input drops it
pub fn take_prepared_cache(cache: Option<&Arc<Mutex<MeshRepairCache>>>, input_face_count: usize, settings: &TexturePaintingSettings) -> bool {
    let Some(cache) = cache else {
        return false;
    };
    let mut cache = cache.lock().expect("the mesh repair cache");
    if !cache.has_prepared_mesh {
        return false;
    }
    let algo = make_tex2color_settings(settings);
    if !cache.matches_input(input_face_count, algo.oversampling_iters, algo.oversampling_min_face_count, algo.oversampling_max_face_count) {
        cache.invalidate_prepared_mesh();
        return false;
    }
    true
}

/// TexturePainting.cpp:176-181: true if the cache holds a prepared mesh matching this input. A
/// mismatched identity clears the prepared geometry but keeps `win10_attempt`.
pub fn mesh_repair_cache_can_skip_input(cache: &MeshRepairCachePtr, input_face_count: usize, settings: &TexturePaintingSettings) -> bool {
    take_prepared_cache(cache.as_ref(), input_face_count, settings)
}

/// TexturePainting.cpp:183-186
pub fn mesh_repair_cache_win10_failed(cache: &MeshRepairCachePtr) -> bool {
    match cache {
        Some(cache) => cache.lock().expect("the mesh repair cache").win10_attempt == MeshRepairAttempt::Failed,
        None => false,
    }
}

/// TexturePainting.cpp:188-198: the oversampled, unclustered prepared mesh for the original-model
/// preview; false when the cache has no prepared geometry
pub fn copy_prepared_mesh_from_cache(cache: &MeshRepairCachePtr, out: &mut PaintedMesh) -> bool {
    let Some(cache) = cache else {
        return false;
    };
    let cache = cache.lock().expect("the mesh repair cache");
    if !cache.has_prepared_mesh {
        return false;
    }
    if cache.mesh.indices.is_empty() || cache.face_colors.is_empty() {
        return false;
    }
    if cache.face_colors.len() != cache.mesh.indices.len() {
        return false;
    }
    extract_painted_mesh(&cache.mesh, &cache.face_colors, out);
    true
}

/// TexturePainting.cpp:200-214: the callbacks of the painting as the callbacks of `tex2color`
pub fn bind_paint_callbacks(
    progress: &PaintProgressCallback,
    cancel: &PaintCancelCallback,
    algo_progress: &mut AlgoProgressCallback,
    algo_cancel: &mut AlgoCancelCallback,
) {
    if let Some(progress) = progress {
        let progress = progress.clone();
        *algo_progress = Some(Arc::new(move |p: AlgoProgress| {
            progress(p.percent, &p.message);
        }));
    }
    if let Some(cancel) = cancel {
        let cancel = cancel.clone();
        *algo_cancel = Some(Arc::new(move || -> bool { cancel() }));
    }
}

/// TexturePainting.cpp:216-245: the prepared mesh of the cache clustered and smoothed
pub fn painting_from_repair_cache(
    settings: &TexturePaintingSettings,
    progress: &PaintProgressCallback,
    cancel: &PaintCancelCallback,
    painted: &mut PaintedMesh,
    pmp: &dyn PolygonMeshProcessing,
) -> bool {
    let Some(cache) = settings.mesh_repair_cache.as_ref() else {
        return false;
    };
    let cache = cache.lock().expect("the mesh repair cache");
    if !cache.has_prepared_mesh {
        return false;
    }

    let mut algo_settings = make_tex2color_settings(settings);
    // Prepared geometry is already stored; do not copy cache->mesh again inside
    // repair_cluster_smooth, and do not retry Win10 repair.
    algo_settings.mesh_repair_cache = None;
    algo_settings.mesh_repair_decision = tex2color::MeshRepairDecision::ImportWithoutRepair;
    let mut algo_progress: AlgoProgressCallback = None;
    let mut algo_cancel: AlgoCancelCallback = None;
    bind_paint_callbacks(progress, cancel, &mut algo_progress, &mut algo_cancel);

    let mut out_mesh = TriMesh::default();
    let mut out_face_colors: Vec<[usize; 3]> = Vec::new();
    let ok = ClusterAndSmooth(
        &cache.mesh,
        &cache.face_colors,
        &mut out_mesh,
        &mut out_face_colors,
        &algo_settings,
        &algo_progress,
        &algo_cancel,
        &[],
        pmp,
    );
    if !ok {
        return false;
    }

    extract_painted_mesh(&out_mesh, &out_face_colors, painted);
    true
}

/// TexturePainting.cpp:247-254: cluster and smooth a previously prepared cache mesh; does not
/// read a `TexturedMesh`
pub fn cluster_from_repair_cache(
    painted: &mut PaintedMesh,
    settings: &TexturePaintingSettings,
    progress: &PaintProgressCallback,
    cancel: &PaintCancelCallback,
    pmp: &dyn PolygonMeshProcessing,
) -> bool {
    painting_from_repair_cache(settings, progress, cancel, painted, pmp)
}

/// `static_cast<uchar>(std::clamp(v, 0.f, 255.f))`
fn clamp_to_uchar(v: f32) -> u8 {
    let c = if v < 0.0 {
        0.0
    } else if 255.0 < v {
        255.0
    } else {
        v
    };
    c as u8
}

/// `static_cast<std::size_t>(std::clamp(v, 0.f, 255.f))`
fn clamp_to_size_t(v: f32) -> usize {
    let c = if v < 0.0 {
        0.0
    } else if 255.0 < v {
        255.0
    } else {
        v
    };
    c as usize
}

/// TexturePainting.cpp:256-399: a vertically stacked atlas of the textures, left-aligned, a 1x1
/// swatch row for every material with a base color but no texture, and the UVs of every face
/// remapped into the region of its texture (or swatch)
pub fn build_multi_texture_atlas(textured: &TexturedMesh, out_atlas: &mut Mat, out_uv_coords: &mut Vec<Vec<Vec2f>>) -> bool {
    let mut decoded: Vec<Mat> = Vec::with_capacity(textured.textures.len());
    for ti in textured.textures.iter() {
        decoded.push(decode_texture_image(ti));
    }

    let has_mapping = !textured.material_texture_map.is_empty();
    let nf = textured.indices.len();

    let resolve_tex_idx = |mat_idx: i32| -> i32 {
        if !has_mapping || mat_idx < 0 || mat_idx as usize >= textured.material_texture_map.len() {
            return -1;
        }
        let ti = textured.material_texture_map[mat_idx as usize];
        if ti < 0 || ti as usize >= decoded.len() || decoded[ti as usize].empty() {
            return -1;
        }
        ti
    };

    // Determine atlas width (max width across all textures) and per-texture row offsets.
    let mut atlas_w: i32 = 0;
    let mut atlas_h: i32 = 0;
    let mut y_offsets: Vec<i32> = vec![0; decoded.len()];
    let mut first_usable_tex: i32 = -1;
    for i in 0..decoded.len() {
        if decoded[i].empty() {
            continue;
        }
        if first_usable_tex < 0 {
            first_usable_tex = i as i32;
        }
        y_offsets[i] = atlas_h;
        atlas_w = atlas_w.max(decoded[i].cols);
        atlas_h += decoded[i].rows;
    }
    if atlas_w == 0 || atlas_h == 0 {
        return false;
    }

    // Collect materials that have a baseColor but no usable texture so we can route their faces
    // to a dedicated 1x1 solid swatch instead of aliasing them onto textures[0].
    let mut mat_solid_y: BTreeMap<i32, i32> = BTreeMap::new();
    let mut mat_solid_color: BTreeMap<i32, [f32; 4]> = BTreeMap::new();
    for fi in 0..nf {
        let mat_idx = if fi < textured.material_ids.len() { textured.material_ids[fi] } else { -1 };
        if mat_idx < 0 {
            continue;
        }
        if resolve_tex_idx(mat_idx) >= 0 {
            continue;
        }
        if mat_idx as usize >= textured.material_colors.len() {
            continue;
        }
        if mat_solid_y.contains_key(&mat_idx) {
            continue;
        }
        mat_solid_y.insert(mat_idx, atlas_h);
        atlas_h += 1;
        mat_solid_color.insert(mat_idx, textured.material_colors[mat_idx as usize]);
    }

    // cv::Mat::zeros(atlas_h, atlas_w, CV_8UC3)
    *out_atlas = Mat::new(atlas_h, atlas_w, 3, vec![0u8; atlas_h as usize * atlas_w as usize * 3]);
    for i in 0..decoded.len() {
        if decoded[i].empty() {
            continue;
        }
        // decoded[i].copyTo(out_atlas(cv::Rect(0, y_offsets[i], cols, rows)))
        let d = &decoded[i];
        let row_bytes = d.cols as usize * 3;
        for r in 0..d.rows as usize {
            let dst = (y_offsets[i] as usize + r) * out_atlas.step;
            let src = r * d.step;
            out_atlas.data[dst..dst + row_bytes].copy_from_slice(&d.data[src..src + row_bytes]);
        }
    }
    for (mat_idx, c) in mat_solid_color.iter() {
        // OpenCV stores BGR; baseColor is RGBA in [0,1].
        let at = mat_solid_y[mat_idx] as usize * out_atlas.step;
        out_atlas.data[at] = clamp_to_uchar(c[2] * 255.0f32);
        out_atlas.data[at + 1] = clamp_to_uchar(c[1] * 255.0f32);
        out_atlas.data[at + 2] = clamp_to_uchar(c[0] * 255.0f32);
    }

    out_uv_coords.resize(nf, Vec::new());
    for fi in 0..nf {
        let mat_idx = if fi < textured.material_ids.len() { textured.material_ids[fi] } else { -1 };
        let tex_idx = resolve_tex_idx(mat_idx);

        // Pick the atlas region this face samples from.
        let (mut y_off, x_off, mut th, mut tw) = (0i32, 0i32, atlas_h, atlas_w);
        let mut use_solid = false;
        if tex_idx >= 0 {
            y_off = y_offsets[tex_idx as usize];
            th = decoded[tex_idx as usize].rows;
            tw = decoded[tex_idx as usize].cols;
        } else if mat_idx >= 0 && mat_solid_y.contains_key(&mat_idx) {
            y_off = mat_solid_y[&mat_idx];
            th = 1;
            tw = 1;
            use_solid = true;
        } else if first_usable_tex >= 0 {
            // Last-resort fallback: the first usable texture.
            y_off = y_offsets[first_usable_tex as usize];
            th = decoded[first_usable_tex as usize].rows;
            tw = decoded[first_usable_tex as usize].cols;
        }

        out_uv_coords[fi].resize(3, Vec2f::zeros());
        for vi in 0..3 {
            let (mut u, mut v) = (0.0f32, 0.0f32);
            if textured.has_face_uvs() {
                let uv_idx = textured.uv_indices[fi][vi];
                if uv_idx >= 0 && (uv_idx as usize) < textured.uv_coords.len() {
                    u = textured.uv_coords[uv_idx as usize][0];
                    v = textured.uv_coords[uv_idx as usize][1];
                }
            } else {
                let vtx_idx = textured.indices[fi][vi];
                if vtx_idx >= 0 && (vtx_idx as usize) < textured.uvs.len() {
                    u = textured.uvs[vtx_idx as usize][0];
                    v = textured.uvs[vtx_idx as usize][1];
                }
            }
            if use_solid {
                // Aim at the centre of the 1x1 swatch so bilinear sampling (in tex2color) cannot
                // drift into neighbouring rows.
                let u_atlas = (x_off as f32 + 0.5f32) / atlas_w as f32;
                let v_atlas = (y_off as f32 + 0.5f32) / atlas_h as f32;
                out_uv_coords[fi][vi] = Vec2f::new(u_atlas, v_atlas);
            } else {
                // Wrap to [0,1) on both axes, then scale by the sub-texture extents so samples land
                // inside its actual region.
                u -= u.floor();
                v -= v.floor();
                let u_atlas = u.mul_add(tw as f32, x_off as f32) / atlas_w as f32;
                let v_atlas = v.mul_add(th as f32, y_off as f32) / atlas_h as f32;
                out_uv_coords[fi][vi] = Vec2f::new(u_atlas, v_atlas);
            }
        }
    }
    true
}

/// TexturePainting.cpp:401-459: the textured mesh sampled, clustered and smoothed (a prepared
/// mesh of the cache is clustered instead when it matches)
pub fn texture_to_painting(
    textured: &TexturedMesh,
    painted: &mut PaintedMesh,
    settings: &TexturePaintingSettings,
    progress: &PaintProgressCallback,
    cancel: &PaintCancelCallback,
    pmp: &dyn PolygonMeshProcessing,
) -> bool {
    if take_prepared_cache(settings.mesh_repair_cache.as_ref(), textured.indices.len(), settings) {
        return painting_from_repair_cache(settings, progress, cancel, painted, pmp);
    }

    if textured.vertices.is_empty() || textured.indices.is_empty() || textured.textures.is_empty() {
        return false;
    }

    let mut texture = Mat::default();
    let mut input_mesh = TriMesh::default();
    let mut uv_coords: Vec<Vec<Vec2f>> = Vec::new();

    let multi_tex = textured.textures.len() > 1 && !textured.material_texture_map.is_empty();

    if multi_tex {
        if !build_multi_texture_atlas(textured, &mut texture, &mut uv_coords) {
            return false;
        }
        // Build mesh geometry (atlas UVs already computed above)
        let nv = textured.vertices.len();
        let nf = textured.indices.len();
        input_mesh.vertices.resize(nv, Vec3f::zeros());
        for i in 0..nv {
            input_mesh.vertices[i] = Vec3f::new(textured.vertices[i][0], textured.vertices[i][1], textured.vertices[i][2]);
        }
        input_mesh.indices.resize(nf, Vec3i::zeros());
        for i in 0..nf {
            input_mesh.indices[i] = Vec3i::new(textured.indices[i][0], textured.indices[i][1], textured.indices[i][2]);
        }
    } else {
        texture = decode_texture_image(&textured.textures[0]);
        if texture.empty() {
            return false;
        }
        build_tex2color_mesh(textured, &mut input_mesh, &mut uv_coords);
    }

    let algo_settings = make_tex2color_settings(settings);

    let mut algo_progress: AlgoProgressCallback = None;
    let mut algo_cancel: AlgoCancelCallback = None;
    bind_paint_callbacks(progress, cancel, &mut algo_progress, &mut algo_cancel);

    let mut color_mesh = TriMesh::default();
    let mut face_colors: Vec<[usize; 3]> = Vec::new();
    let ok = TextureToColor(&input_mesh, &uv_coords, &texture, &mut color_mesh, &mut face_colors, &algo_settings, &algo_progress, &algo_cancel, pmp);

    if !ok {
        return false;
    }

    extract_painted_mesh(&color_mesh, &face_colors, painted);
    true
}

/// TexturePainting.cpp:461-501: pre-computed per-face colors turned into a painted mesh, skipping
/// texture decode and UV sampling. A low-poly mesh that also carries per-vertex colors is split
/// along quantized color boundaries, which replaces its geometry.
pub fn face_colors_to_painting(
    mesh: &TexturedMesh,
    painted: &mut PaintedMesh,
    settings: &TexturePaintingSettings,
    progress: &PaintProgressCallback,
    cancel: &PaintCancelCallback,
    pmp: &dyn PolygonMeshProcessing,
) -> bool {
    if take_prepared_cache(settings.mesh_repair_cache.as_ref(), mesh.indices.len(), settings) {
        return painting_from_repair_cache(settings, progress, cancel, painted, pmp);
    }

    if mesh.vertices.is_empty() || mesh.indices.is_empty() || mesh.precomputed_face_colors.is_empty() {
        return false;
    }

    // Build tex2color::TriMesh from input geometry
    let mut input_mesh = TriMesh::default();
    input_mesh.vertices.resize(mesh.vertices.len(), Vec3f::zeros());
    for i in 0..mesh.vertices.len() {
        input_mesh.vertices[i] = Vec3f::new(mesh.vertices[i][0], mesh.vertices[i][1], mesh.vertices[i][2]);
    }
    input_mesh.indices.resize(mesh.indices.len(), Vec3i::zeros());
    for i in 0..mesh.indices.len() {
        input_mesh.indices[i] = Vec3i::new(mesh.indices[i][0], mesh.indices[i][1], mesh.indices[i][2]);
    }

    let algo_settings = make_tex2color_settings(settings);

    let mut algo_progress: AlgoProgressCallback = None;
    let mut algo_cancel: AlgoCancelCallback = None;
    bind_paint_callbacks(progress, cancel, &mut algo_progress, &mut algo_cancel);

    let mut out_mesh = TriMesh::default();
    let mut out_face_colors: Vec<[usize; 3]> = Vec::new();
    let ok = ClusterAndSmooth(
        &input_mesh,
        &mesh.precomputed_face_colors,
        &mut out_mesh,
        &mut out_face_colors,
        &algo_settings,
        &algo_progress,
        &algo_cancel,
        &mesh.precomputed_vertex_colors,
        pmp,
    );

    if !ok {
        return false;
    }

    extract_painted_mesh(&out_mesh, &out_face_colors, painted);
    true
}

/// TexturePainting.cpp:503-613: an exact source palette (standard 3MF color groups) clustered in
/// color space. Mesh repair, oversampling and smoothing are skipped; the mesh is only split along
/// the color borders of its vertices, for the preview.
pub fn standard_3mf_colors_to_painting(
    mesh: &TexturedMesh,
    painted: &mut PaintedMesh,
    settings: &TexturePaintingSettings,
    progress: &PaintProgressCallback,
    cancel: &PaintCancelCallback,
) -> bool {
    *painted = PaintedMesh::default();
    if mesh.indices.is_empty() || mesh.vertices.is_empty() || mesh.precomputed_vertex_colors.len() != mesh.vertices.len() {
        return false;
    }

    let report = |percent: i32, message: &str| {
        if let Some(progress) = progress {
            progress(percent, message);
        }
    };
    let cancelled = || -> bool {
        match cancel {
            Some(cancel) => cancel(),
            None => false,
        }
    };

    report(0, "Collecting colors");
    if cancelled() {
        return false;
    }

    // Per-face colors are averages of the corner colors, so clustering them would invent colors
    // the file never had. Cluster the source palette instead.
    type RGB = [usize; 3];
    let mut unique_colors: Vec<RGB> = Vec::new();
    let mut unique_index: BTreeMap<RGB, usize> = BTreeMap::new();
    let mut vertex_source_ids: Vec<usize> = vec![0; mesh.vertices.len()];
    for i in 0..mesh.vertices.len() {
        let mut rgb: RGB = [0; 3];
        for c in 0..3 {
            // std::lround(std::clamp(c, 0.f, 1.f) * 255.f): to nearest, halves away from zero
            let v = mesh.precomputed_vertex_colors[i][c];
            let clamped = if v < 0.0f32 {
                0.0f32
            } else if 1.0f32 < v {
                1.0f32
            } else {
                v
            };
            rgb[c] = (clamped * 255.0f32).round() as i64 as usize;
        }
        let next = unique_colors.len();
        let id = *unique_index.entry(rgb).or_insert_with(|| {
            unique_colors.push(rgb);
            next
        });
        vertex_source_ids[i] = id;
    }

    let max_cluster = MAX_PHYSICAL_FILAMENT_COUNT;
    let target = if settings.target_colors_num == 0 {
        unique_colors.len().min(max_cluster)
    } else {
        settings.target_colors_num.min(max_cluster)
    };

    report(20, "Clustering colors");
    if cancelled() {
        return false;
    }

    // The file already fits within the requested count, so keep its exact colors rather than
    // clustering them and drifting off the authored values.
    let mut centers: Vec<RGB> = Vec::new();
    let mut cluster_labels: Vec<usize> = Vec::new();
    if unique_colors.len() <= target {
        centers = unique_colors.clone();
        cluster_labels = (0..unique_colors.len()).collect();
    } else {
        let mut algo_settings = make_tex2color_settings(settings);
        algo_settings.target_colors_num = if settings.target_colors_num == 0 { 0 } else { target };
        algo_settings.max_cluster_k = algo_settings.max_cluster_k.min(max_cluster);
        let mut algo_progress: AlgoProgressCallback = None;
        let mut algo_cancel: AlgoCancelCallback = None;
        bind_paint_callbacks(&None, cancel, &mut algo_progress, &mut algo_cancel);
        if !quantize_colors(&unique_colors, &algo_settings, &algo_cancel, &mut centers, &mut cluster_labels) {
            return false;
        }
    }
    if centers.is_empty() || cluster_labels.len() != unique_colors.len() {
        return false;
    }

    report(60, "Building preview");
    if cancelled() {
        return false;
    }

    let mut preview_mesh = TriMesh::default();
    preview_mesh.vertices.resize(mesh.vertices.len(), Vec3f::zeros());
    for i in 0..mesh.vertices.len() {
        preview_mesh.vertices[i] = Vec3f::new(mesh.vertices[i][0], mesh.vertices[i][1], mesh.vertices[i][2]);
    }
    preview_mesh.indices.resize(mesh.indices.len(), Vec3i::zeros());
    for i in 0..mesh.indices.len() {
        preview_mesh.indices[i] = Vec3i::new(mesh.indices[i][0], mesh.indices[i][1], mesh.indices[i][2]);
    }

    // Split across color borders so the preview shows the same sharp boundaries the painting will
    // produce inside a triangle. This mesh never reaches the model.
    let mut vertex_cluster_ids: Vec<usize> = vec![0; vertex_source_ids.len()];
    for i in 0..vertex_source_ids.len() {
        vertex_cluster_ids[i] = cluster_labels[vertex_source_ids[i]];
    }
    let mut face_colors: Vec<[usize; 3]> = Vec::new();
    if !tex2color::adaptive_split_by_vertex_clusters(&mut preview_mesh, &vertex_cluster_ids, &centers, &mut face_colors) {
        return false;
    }

    if cancelled() {
        return false;
    }

    // Cluster colors are derived from the faces that actually use them, so a cluster that ended
    // up empty never takes a color slot in the dialog. They come out sorted.
    extract_painted_mesh(&preview_mesh, &face_colors, painted);
    for i in 0..unique_colors.len() {
        let center = centers[cluster_labels[i]];
        let it = painted.cluster_colors.partition_point(|c| *c < center);
        let id = if it != painted.cluster_colors.len() && painted.cluster_colors[it] == center { it as i32 } else { -1 };
        painted.original_color_to_cluster.insert(unique_colors[i], id);
    }
    report(100, "Completed");

    true
}

/// TexturePainting.cpp:615-626: CIEDE2000 between an RGB [0..255] color and an RGBA [0..1] one,
/// truncated to [0..255]
pub fn compute_delta_e(rgb1: &[usize; 3], rgba2: &[f32; 4]) -> f64 {
    calc_rgb_color_difference_by_ciede2000(rgb1, &[(rgba2[0] * 255.0f32) as usize, (rgba2[1] * 255.0f32) as usize, (rgba2[2] * 255.0f32) as usize])
}

/// TexturePainting.cpp:628-633: `compute_delta_e(const RGB&, const RGB&)`
pub fn compute_delta_e_rgb(rgb1: &[usize; 3], rgb2: &[usize; 3]) -> f64 {
    calc_rgb_color_difference_by_ciede2000(rgb1, rgb2)
}

/// TexturePainting.cpp:635-657: every cluster color matched to the filament color nearest to it
/// (CIEDE2000), the first of equal ones
pub fn match_clusters_to_filaments(cluster_colors: &[[usize; 3]], filament_colors: &[[f32; 4]], _filament_names: &[String]) -> Vec<FilamentMatch> {
    let mut matches: Vec<FilamentMatch> = vec![FilamentMatch::default(); cluster_colors.len()];

    for ci in 0..cluster_colors.len() {
        matches[ci].cluster_index = ci as i32;
        matches[ci].cluster_color = cluster_colors[ci];
        matches[ci].delta_e = 1e9;

        for fi in 0..filament_colors.len() {
            let de = compute_delta_e(&cluster_colors[ci], &filament_colors[fi]);
            if de < matches[ci].delta_e {
                matches[ci].delta_e = de;
                matches[ci].filament_index = fi as i32;
                matches[ci].filament_color = filament_colors[fi];
            }
        }
    }
    matches
}

/// TexturePainting.cpp:659-787: the painted mesh becomes the mesh of the volume (in the frame of
/// its original source, recentered) and its face colors, through the filament matches, the
/// multi-material painting of the volume. `convex_hull_3d` is `TriangleMesh::convex_hull_3d`
/// (see the FIDELITY-NOTE of the file).
pub fn apply_painted_mesh_to_volume(
    painted: &PaintedMesh,
    matches: &[FilamentMatch],
    volume: &mut ModelVolume,
    convex_hull_3d: &dyn Fn(&indexed_triangle_set) -> indexed_triangle_set,
) -> bool {
    if painted.face_colors.is_empty() || matches.is_empty() {
        return false;
    }

    let cluster_colors = &painted.cluster_colors;
    let mut color_to_filament: BTreeMap<[usize; 3], i32> = BTreeMap::new();
    for m in matches.iter() {
        if m.cluster_index >= 0 && m.cluster_index < cluster_colors.len() as i32 && m.filament_index >= 0 {
            color_to_filament.insert(cluster_colors[m.cluster_index as usize], m.filament_index);
        }
    }

    let mut its = indexed_triangle_set::default();
    its.vertices.resize(painted.vertices.len(), Vec3f::zeros());
    for i in 0..painted.vertices.len() {
        its.vertices[i] = Vec3f::new(painted.vertices[i][0], painted.vertices[i][1], painted.vertices[i][2]);
    }
    its.indices.resize(painted.indices.len(), Vec3i::zeros());
    for i in 0..painted.indices.len() {
        its.indices[i] = Vec3i::new(painted.indices[i][0], painted.indices[i][1], painted.indices[i][2]);
    }

    let mut new_mesh = VolumeMesh::new(its);

    // The painted mesh is expressed in the original un-centered coordinate frame; reuse the
    // recorded shift (brought into millimetres when the model was converted after load) to align
    // it.
    let mut mesh_offset = volume.source.mesh_offset;
    let mut unit_scale = 1.0f64;
    if volume.source.is_converted_from_meters {
        unit_scale = 1000.0;
    } else if volume.source.is_converted_from_inches {
        unit_scale = 25.4;
    }
    if unit_scale != 1.0 {
        mesh_offset *= unit_scale;
    }

    if !is_approx_zero(&mesh_offset) {
        let m = mesh_offset.cast::<f32>();
        new_mesh.translate(-m);
    }
    new_mesh.set_init_shift(mesh_offset);

    // (C++ logs the drift of the bounding box center here, TexturePainting.cpp:727-747.)

    volume.set_mesh(new_mesh);
    volume.calculate_convex_hull(convex_hull_3d);

    // Re-center the replaced mesh so its bbox center sits at the origin; keep source.mesh_offset.
    volume.center_geometry_after_creation(false);
    volume.invalidate_convex_hull_2d();

    // Mesh geometry has been replaced; any per-face annotation indexed against the previous
    // triangle set is now stale.
    volume.supported_facets.reset();
    volume.fuzzy_skin_facets.reset();
    volume.seam_facets.reset();

    if let Some(obj) = volume.get_object() {
        obj.invalidate_bounding_box();
    }

    // TriangleSelector selector(volume.mesh()), with the default edge limit (TriangleSelector.hpp)
    let selector_mesh = TriangleMesh::from_parts(
        volume.mesh().its.vertices.iter().map(|v| crate::geometry::Point3F::new(v.x as f64, v.y as f64, v.z as f64)).collect(),
        volume.mesh().its.indices.iter().map(|f| Triangle::new(f[0] as u32, f[1] as u32, f[2] as u32)).collect(),
    );
    let mut selector = TriangleSelector::new(selector_mesh, 0.6f32);
    let mut fi = 0usize;
    while fi < painted.face_colors.len() && fi < volume.mesh().its.indices.len() {
        if let Some(&extruder_idx) = color_to_filament.get(&painted.face_colors[fi]) {
            // static_cast<EnforcerBlockerType>(int(Extruder1) + extruder_idx): the int8_t enum
            let state = EnforcerBlockerType((EnforcerBlockerType::EXTRUDER1.0 as i32 + extruder_idx) as i8);
            if state <= EXTRUDER_MAX {
                selector.set_facet(fi as i32, state);
            }
        }
        fi += 1;
    }

    volume.mmu_segmentation_facets.set(&selector);
    true
}

/// TexturePainting.cpp:789-811: a `TextureImage` decoded into BGR pixel data, 3 bytes per pixel
pub fn decode_texture_to_pixels(img: &TextureImage, out_pixels: &mut Vec<u8>, out_w: &mut i32, out_h: &mut i32) -> bool {
    let decoded = decode_texture_image(img);
    if decoded.empty() {
        return false;
    }

    // decoded is BGR, CV_8UC3
    *out_w = decoded.cols;
    *out_h = decoded.rows;
    let nbytes = *out_w as usize * *out_h as usize * 3;
    out_pixels.resize(nbytes, 0);

    // decoded.isContinuous(): the matrices made here are
    out_pixels.copy_from_slice(&decoded.data[..nbytes]);
    true
}

/// TexturePainting.cpp:813-855: the color of a face sampled at the centroid of its three UVs,
/// bilinearly
pub fn sample_face_from_uvs(tex: &Mat, uv0: &[f32; 2], uv1: &[f32; 2], uv2: &[f32; 2]) -> [usize; 3] {
    let mut cu = (uv0[0] + uv1[0] + uv2[0]) / 3.0f32;
    let mut cv_val = (uv0[1] + uv1[1] + uv2[1]) / 3.0f32;

    cu -= cu.floor();
    cv_val -= cv_val.floor();

    let fx = cu * (tex.cols - 1) as f32;
    let fy = cv_val * (tex.rows - 1) as f32;

    let clamp_i32 = |v: i32, lo: i32, hi: i32| -> i32 {
        if v < lo {
            lo
        } else if hi < v {
            hi
        } else {
            v
        }
    };
    let x0 = clamp_i32(fx as i32, 0, tex.cols - 1);
    let y0 = clamp_i32(fy as i32, 0, tex.rows - 1);
    let x1 = (x0 + 1).min(tex.cols - 1);
    let y1 = (y0 + 1).min(tex.rows - 1);

    let wx = fx - x0 as f32;
    let wy = fy - y0 as f32;

    let ch = tex.channels();
    let sample = |row: i32, col: i32| -> [f32; 3] {
        let ptr = row as usize * tex.step + col as usize * ch as usize;
        [tex.data[ptr + 2] as f32, tex.data[ptr + 1] as f32, tex.data[ptr] as f32]
    };

    let c00 = sample(y0, x0);
    let c10 = sample(y0, x1);
    let c01 = sample(y1, x0);
    let c11 = sample(y1, x1);

    let mut color: [usize; 3] = [0; 3];
    for i in 0..3 {
        let top = c00[i].mul_add(1.0f32 - wx, c10[i] * wx);
        let bot = c01[i].mul_add(1.0f32 - wx, c11[i] * wx);
        color[i] = clamp_to_size_t(top.mul_add(1.0f32 - wy, bot * wy));
    }
    color
}

/// TexturePainting.cpp:857-868: the legacy overload, the UVs looked up in the per-vertex array by
/// the vertex indices ((0, 0) where out of range)
pub fn sample_face_from_texture(tex: &Mat, uvs: &[[f32; 2]], face: &[i32; 3]) -> [usize; 3] {
    let (mut uv0, mut uv1, mut uv2) = ([0.0f32, 0.0], [0.0f32, 0.0], [0.0f32, 0.0]);
    if face[0] >= 0 && (face[0] as usize) < uvs.len() {
        uv0 = uvs[face[0] as usize];
    }
    if face[1] >= 0 && (face[1] as usize) < uvs.len() {
        uv1 = uvs[face[1] as usize];
    }
    if face[2] >= 0 && (face[2] as usize) < uvs.len() {
        uv2 = uvs[face[2] as usize];
    }
    sample_face_from_uvs(tex, &uv0, &uv1, &uv2)
}

/// TexturePainting.cpp:870-927: per-face colors sampled from the texture of each face's material
/// (`material_texture_map` / `material_colors`, falling back to `textures[0]`), without
/// oversampling; the base color of a material without a texture, gray otherwise
pub fn sample_original_face_colors(textured: &TexturedMesh, out_face_colors: &mut Vec<[usize; 3]>) -> bool {
    if textured.indices.is_empty() {
        return false;
    }

    // Decode all textures up front
    let mut decoded_textures: Vec<Mat> = Vec::with_capacity(textured.textures.len());
    for ti in textured.textures.iter() {
        decoded_textures.push(decode_texture_image(ti));
    }

    let has_mapping = !textured.material_texture_map.is_empty();
    let nf = textured.indices.len();
    out_face_colors.resize(nf, [0; 3]);

    for fi in 0..nf {
        let mat_idx = if fi < textured.material_ids.len() { textured.material_ids[fi] } else { -1 };

        let mut tex_idx: i32 = -1;
        if has_mapping && mat_idx >= 0 && (mat_idx as usize) < textured.material_texture_map.len() {
            tex_idx = textured.material_texture_map[mat_idx as usize];
        } else if !decoded_textures.is_empty() {
            tex_idx = 0; // fallback: single-texture model
        }

        if tex_idx >= 0 && (tex_idx as usize) < decoded_textures.len() && !decoded_textures[tex_idx as usize].empty() {
            if textured.has_face_uvs() {
                let ui = &textured.uv_indices[fi];
                let get_uv = |vi: usize| -> [f32; 2] {
                    let idx = ui[vi];
                    if idx >= 0 && (idx as usize) < textured.uv_coords.len() {
                        return textured.uv_coords[idx as usize];
                    }
                    [0.0, 0.0]
                };
                out_face_colors[fi] = sample_face_from_uvs(&decoded_textures[tex_idx as usize], &get_uv(0), &get_uv(1), &get_uv(2));
            } else {
                out_face_colors[fi] = sample_face_from_texture(&decoded_textures[tex_idx as usize], &textured.uvs, &textured.indices[fi]);
            }
        } else if has_mapping && mat_idx >= 0 && (mat_idx as usize) < textured.material_colors.len() {
            // No texture — use baseColorFactor as solid color
            let c = &textured.material_colors[mat_idx as usize];
            out_face_colors[fi] = [clamp_to_size_t(c[0] * 255.0f32), clamp_to_size_t(c[1] * 255.0f32), clamp_to_size_t(c[2] * 255.0f32)];
        } else {
            out_face_colors[fi] = [192, 192, 192]; // default gray
        }
    }
    true
}
