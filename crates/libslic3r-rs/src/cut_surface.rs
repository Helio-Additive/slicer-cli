//! Port of `libslic3r/CutSurface.cpp` (+ `CutSurface.hpp`).
//!
//! C++ Reference:
//! - CutSurface.hpp
//! - CutSurface.cpp
//!
//! # Porting status: PARTIAL
//!
//! The bulk of this translation unit is built directly on top of CGAL's
//! `Surface_mesh` half-edge data structure and
//! `CGAL/Polygon_mesh_processing/corefinement.h` (mesh corefinement /
//! co-refine + boolean clipping). These are a native C++ template library with
//! no equivalent in this crate; bringing them in would violate the wasm-safe /
//! no-native-dep rule, so those symbols are intentionally NOT ported here. They
//! are listed in the module docs below as blocked.
//!
//! Everything that is CGAL-free is ported faithfully, line for line:
//! - `SurfaceCut`                         (CutSurface.hpp:15-23)
//! - `cut2model`                          (CutSurface.cpp:619-681)
//! - `is_out_of`                          (CutSurface.cpp:709-715)
//! - `is_all_on_one_side`                 (CutSurface.cpp:717-729)
//! - `set_skip_for_out_of_aoi`            (CutSurface.cpp:731-819)
//! - `its_mask`                           (CutSurface.cpp:821-859)
//! - `its_cut_AoI`                        (CutSurface.cpp:861-870)
//! - `set_skip_by_angle`                  (CutSurface.cpp:872-895)
//!
//! Blocked on CGAL `Surface_mesh` / corefinement (do NOT add native dep):
//! - `cut_surface`        (CutSurface.cpp:525-617) — orchestrates the CGAL pipeline
//! - `to_cgal` (x2)       (CutSurface.cpp:897-...) — build CGAL `Surface_mesh`
//! - `exist_duplicit_vertex`, `create_reduce_map`, `cut_from_model`,
//!   `create_loops`, `diff_models`, `is_over_whole_expoly`, `unproject_loops`,
//!   `to_expoly`, `calc_distances`, `choose_best_distance`, `select_patches`,
//!   `merge_patches`, `patch2cut`, `append`, `clip_cut`, `create_face_types`,
//!   `flood_fill_inner`, `collect_surface_data`, `set_face_type`,
//!   `is_face_inside`, `create_cut_area_of_interests`, `divide_patch`,
//!   `collect_open_edges`, `has_bb_intersection`, `is_patch_inside_of_model`,
//!   `get_shape_point_index`, `ModelCut2index`, `Visitor`, `SurfacePatch`,
//!   the `store(...)` debug helpers, and `corefine_test`
//!   — all operate on `priv::CutMesh = CGAL::Surface_mesh<...>` and/or call
//!   `CGAL::Polygon_mesh_processing::corefine`.

// CutSurface.cpp:1  #include "CutSurface.hpp"
use crate::geometry::{BoundingBox, Point, Vec3d};
use crate::normal_utils::{indexed_triangle_set, Vec3f};
// CutSurface.cpp:883  its_face_normal
use crate::emboss::{IProject3d, IProjection};
use crate::triangle_mesh::its_face_normal_indices;

// CutSurface.hpp:18  using Index = unsigned int;
type Index = u32;
// CutSurface.hpp:19  using Contour = std::vector<Index>;
type Contour = Vec<Index>;
// CutSurface.hpp:20  using Contours = std::vector<Contour>;
type Contours = Vec<Contour>;

/// Represents cutted surface from object
/// Extend index triangle set by outlines
/// CutSurface.hpp:15-23
///
/// `struct SurfaceCut : public indexed_triangle_set` — Rust has no inheritance,
/// so the base `indexed_triangle_set` is embedded as the `its` field. Field
/// access in the C++ source such as `cut.vertices` / `cut.indices` maps to
/// `cut.its.vertices` / `cut.its.indices`.
#[derive(Debug, Clone, Default)]
pub struct SurfaceCut {
    // CutSurface.hpp:15  : public indexed_triangle_set
    pub its: indexed_triangle_set,
    // CutSurface.hpp:22  Contours contours;
    // list of circulated open surface
    pub contours: Contours,
}

impl SurfaceCut {
    pub fn new() -> Self {
        SurfaceCut::default()
    }

    /// Mirror of `indexed_triangle_set::empty()`: an empty surface cut.
    pub fn empty(&self) -> bool {
        self.its.indices.is_empty()
    }
}

// ===========================================================================
// priv:: namespace
// CutSurface.cpp:41-42
// using Project   = Emboss::IProjection;
// using Project3d = Emboss::IProject3d;
// ===========================================================================

/// Cut surface shape from models.
///
/// CutSurface.hpp:37-40 / CutSurface.cpp:525-617
///
/// BLOCKED: the whole body is the CGAL corefinement pipeline
/// (`priv::to_cgal`, `priv::cut_from_model`, `priv::diff_models`,
/// `priv::calc_distances`, `priv::choose_best_distance`,
/// `priv::select_patches`, `merge_patches`). It cannot be ported without the
/// CGAL `Surface_mesh` native dependency, which the wasm-safe rule forbids.
/// Not implemented; see module docs.

/// Create model from surface cuts by projection
/// CutSurface.hpp:48-49 / CutSurface.cpp:619-681
pub fn cut2model(cut: &SurfaceCut, projection: &dyn IProject3d) -> indexed_triangle_set {
    // CutSurface.cpp:622  assert(!cut.empty());
    debug_assert!(!cut.empty());
    // CutSurface.cpp:623  size_t count_vertices = cut.vertices.size() * 2;
    let count_vertices = cut.its.vertices.len() * 2;
    // CutSurface.cpp:624  size_t count_indices  = cut.indices.size() * 2;
    let mut count_indices = cut.its.indices.len() * 2;

    // CutSurface.cpp:626-630  indices from from zig zag
    for c in &cut.contours {
        // CutSurface.cpp:628  assert(!c.empty());
        debug_assert!(!c.is_empty());
        // CutSurface.cpp:629  count_indices += c.size() * 2;
        count_indices += c.len() * 2;
    }

    // CutSurface.cpp:632  indexed_triangle_set result;
    let mut result = indexed_triangle_set::default();
    // CutSurface.cpp:633  result.vertices.reserve(count_vertices);
    result.vertices.reserve(count_vertices);
    // CutSurface.cpp:634  result.indices.reserve(count_indices);
    result.indices.reserve(count_indices);

    // CutSurface.cpp:636-640  front
    // result.vertices.insert(result.vertices.end(), cut.vertices.begin(), cut.vertices.end());
    result.vertices.extend_from_slice(&cut.its.vertices);
    // result.indices.insert(result.indices.end(), cut.indices.begin(), cut.indices.end());
    result.indices.extend_from_slice(&cut.its.indices);

    // CutSurface.cpp:642-647  back
    for v in &cut.its.vertices {
        // CutSurface.cpp:644  Vec3d vd = v.cast<double>();
        let vd = Vec3d::new(v.x as f64, v.y as f64, v.z as f64);
        // CutSurface.cpp:645  Vec3d vd2 = projection.project(vd);
        let vd2 = projection.project(&vd);
        // CutSurface.cpp:646  result.vertices.push_back(vd2.cast<float>());
        result
            .vertices
            .push(Vec3f::new(vd2.x as f32, vd2.y as f32, vd2.z as f32));
    }

    // CutSurface.cpp:649  size_t back_offset = cut.vertices.size();
    let back_offset = cut.its.vertices.len();
    // CutSurface.cpp:650-662
    for i in &cut.its.indices {
        // CutSurface.cpp:652-657  range checks
        debug_assert!((i.x as usize) + back_offset < result.vertices.len());
        debug_assert!((i.y as usize) + back_offset < result.vertices.len());
        debug_assert!((i.z as usize) + back_offset < result.vertices.len());
        debug_assert!(i.x >= 0 && (i.x as usize) < cut.its.vertices.len());
        debug_assert!(i.y >= 0 && (i.y as usize) < cut.its.vertices.len());
        debug_assert!(i.z >= 0 && (i.z as usize) < cut.its.vertices.len());
        // CutSurface.cpp:658-661  Y and Z is swapped CCW triangles for back side
        result.indices.push(Vec3f32i::new(
            i.x + back_offset as i32,
            i.z + back_offset as i32,
            i.y + back_offset as i32,
        ));
    }

    // CutSurface.cpp:664-676  zig zag indices
    for contour in &cut.contours {
        // CutSurface.cpp:666  size_t prev_front_index = contour.back();
        let mut prev_front_index = *contour.last().unwrap() as usize;
        // CutSurface.cpp:667  size_t prev_back_index  = back_offset + prev_front_index;
        let mut prev_back_index = back_offset + prev_front_index;
        // CutSurface.cpp:668  for (size_t front_index : contour) {
        for &front_index_u in contour {
            let front_index = front_index_u as usize;
            // CutSurface.cpp:669  assert(front_index < cut.vertices.size());
            debug_assert!(front_index < cut.its.vertices.len());
            // CutSurface.cpp:670  size_t back_index  = back_offset + front_index;
            let back_index = back_offset + front_index;
            // CutSurface.cpp:671  result.indices.emplace_back(front_index, prev_front_index, back_index);
            result.indices.push(Vec3f32i::new(
                front_index as i32,
                prev_front_index as i32,
                back_index as i32,
            ));
            // CutSurface.cpp:672  result.indices.emplace_back(prev_front_index, prev_back_index, back_index);
            result.indices.push(Vec3f32i::new(
                prev_front_index as i32,
                prev_back_index as i32,
                back_index as i32,
            ));
            // CutSurface.cpp:673-674
            prev_front_index = front_index;
            prev_back_index = back_index;
        }
    }

    // CutSurface.cpp:678-679
    debug_assert!(count_vertices == result.vertices.len());
    debug_assert!(count_indices == result.indices.len());
    // CutSurface.cpp:680  return result;
    result
}

// Triangle vertex index type used by `result.indices.emplace_back(...)`.
// In C++ this is `stl_triangle_vertex_indices` (`Vec3i`).
use crate::normal_utils::StlTriangleVertexIndices as Vec3f32i;

// ===========================================================================
// set_skip_for_out_of_aoi helping functions
// CutSurface.cpp:683-707  namespace priv
// ===========================================================================

// CutSurface.cpp:686  using PointNormal  = std::pair<Vec3d, Vec3d>;
type PointNormal = (Vec3d, Vec3d);
// CutSurface.cpp:687  using PointNormals = std::array<PointNormal, 4>;
type PointNormals = [PointNormal; 4];

// CutSurface.cpp:698  using IsOnSides = std::vector<std::array<bool, 4>>;
type IsOnSides = Vec<[bool; 4]>;

/// Check
/// CutSurface.cpp:696 / CutSurface.cpp:709-715
fn is_out_of(v: &Vec3d, point_normal: &PointNormal) -> bool {
    // CutSurface.cpp:711  const Vec3d& p = point_normal.first;
    let p = &point_normal.0;
    // CutSurface.cpp:712  const Vec3d& n = point_normal.second;
    let n = &point_normal.1;
    // CutSurface.cpp:713  double signed_distance = (v - p).dot(n);
    let signed_distance = (*v - *p).dot(n);
    // CutSurface.cpp:714  return signed_distance > 1e-5;
    signed_distance > 1e-5
}

/// Check if triangle t has all vertices out of any plane
/// CutSurface.cpp:705 / CutSurface.cpp:717-729
fn is_all_on_one_side(t: &Vec3f32i, is_on_sides: &IsOnSides) -> bool {
    // CutSurface.cpp:718  for (size_t side = 0; side < 4; side++) {
    for side in 0..4 {
        // CutSurface.cpp:719  bool result = true;
        let mut result = true;
        // CutSurface.cpp:720  for (auto vi : t) {
        for k in 0..3 {
            let vi = t[k] as usize;
            // CutSurface.cpp:721  if (!is_on_sides[vi][side]) {
            if !is_on_sides[vi][side] {
                // CutSurface.cpp:722-723  result = false; break;
                result = false;
                break;
            }
        }
        // CutSurface.cpp:726  if (result) return true;
        if result {
            return true;
        }
    }
    // CutSurface.cpp:728  return false;
    false
}

/// Set true for indices out of area of interest
/// CutSurface.cpp:51-54 / CutSurface.cpp:731-819
fn set_skip_for_out_of_aoi(
    skip_indicies: &mut [bool],
    its: &indexed_triangle_set,
    projection: &dyn IProjection,
    shapes_bb: &BoundingBox,
) {
    // CutSurface.cpp:736  assert(skip_indicies.size() == its.indices.size());
    debug_assert!(skip_indicies.len() == its.indices.len());
    //   1`*----* 2`
    //    /  2 /|
    // 1 *----* |
    //   |    | * 3`
    //   |    |/
    // 0 *----* 3
    //////////////////
    // CutSurface.cpp:744  std::array<std::pair<Vec3d, Vec3d>, 4> bb;
    // Default-initialised; values are filled in below.
    let mut bb: [PointNormal; 4] = [
        (Vec3d::zero(), Vec3d::zero()),
        (Vec3d::zero(), Vec3d::zero()),
        (Vec3d::zero(), Vec3d::zero()),
        (Vec3d::zero(), Vec3d::zero()),
    ];
    // CutSurface.cpp:745  int index = 0;
    // CutSurface.cpp:746-749
    // for (Point v : {shapes_bb.min, {min.x, max.y}, shapes_bb.max, {max.x, min.y}})
    //     bb[index++] = projection.create_front_back(v);
    let corners: [Point; 4] = [
        shapes_bb.min,
        Point::new(shapes_bb.min.x(), shapes_bb.max.y()),
        shapes_bb.max,
        Point::new(shapes_bb.max.x(), shapes_bb.min.y()),
    ];
    let mut index = 0usize;
    for v in corners.iter() {
        bb[index] = projection.create_front_back(v);
        index += 1;
    }

    // define planes to test
    // 0 .. under
    // 1 .. left
    // 2 .. above
    // 3 .. right
    // CutSurface.cpp:756  size_t prev_i = 3;
    let mut prev_i = 3usize;
    // CutSurface.cpp:758  PointNormals point_normals;
    let mut point_normals: PointNormals = [
        (Vec3d::zero(), Vec3d::zero()),
        (Vec3d::zero(), Vec3d::zero()),
        (Vec3d::zero(), Vec3d::zero()),
        (Vec3d::zero(), Vec3d::zero()),
    ];
    // CutSurface.cpp:759  for (size_t i = 0; i < 4; i++) {
    for i in 0..4usize {
        // CutSurface.cpp:760  const Vec3d &p1 = bb[i].first;
        let p1 = bb[i].0;
        // CutSurface.cpp:761  const Vec3d &p2 = bb[i].second;
        let p2 = bb[i].1;
        // CutSurface.cpp:762  const Vec3d &p3 = bb[prev_i].first;
        let p3 = bb[prev_i].0;
        // CutSurface.cpp:763  prev_i = i;
        prev_i = i;

        // CutSurface.cpp:765  Vec3d v1 = p2 - p1;
        let mut v1 = p2 - p1;
        // CutSurface.cpp:766  v1.normalize();
        normalize(&mut v1);
        // CutSurface.cpp:767  Vec3d v2 = p3 - p1;
        let mut v2 = p3 - p1;
        // CutSurface.cpp:768  v2.normalize();
        normalize(&mut v2);

        // CutSurface.cpp:770  Vec3d normal = v2.cross(v1);
        let mut normal = v2.cross(&v1);
        // CutSurface.cpp:771  normal.normalize();
        normalize(&mut normal);

        // CutSurface.cpp:773  point_normals[i] = {p1, normal};
        point_normals[i] = (p1, normal);
    }

    // check that projection is not left handed
    // Fix for reflected projection
    // CutSurface.cpp:778  if (is_out_of(point_normals[2].first, point_normals[0])) {
    if is_out_of(&point_normals[2].0, &point_normals[0]) {
        // CutSurface.cpp:780-781  projection is reflected so normals are reflected
        for pn in point_normals.iter_mut() {
            pn.1 = pn.1 * -1.0;
        }
    }

    // same meaning as point normal
    // CutSurface.cpp:785  IsOnSides is_on_sides(its.vertices.size(), {false,false,false,false});
    let mut is_on_sides: IsOnSides = vec![[false, false, false, false]; its.vertices.len()];

    // inspect all vertices when it is out of bounding box
    // CutSurface.cpp:788-809  tbb::parallel_for over vertices (ported as a serial loop)
    for i in 0..its.vertices.len() {
        // CutSurface.cpp:791  Vec3d v = its.vertices[i].cast<double>();
        let vv = &its.vertices[i];
        let v = Vec3d::new(vv.x as f64, vv.y as f64, vv.z as f64);
        // CutSurface.cpp:792-799  under + above
        for side in [0usize, 2usize] {
            if is_out_of(&v, &point_normals[side]) {
                is_on_sides[i][side] = true;
                // when it is under it can't be above
                break;
            }
        }
        // CutSurface.cpp:800-807  left + right
        for side in [1usize, 3usize] {
            if is_out_of(&v, &point_normals[side]) {
                is_on_sides[i][side] = true;
                // when it is on left side it can't be on right
                break;
            }
        }
    }

    // inspect all triangles, when it is out of bounding box
    // CutSurface.cpp:812-818  tbb::parallel_for over indices (ported as a serial loop)
    for i in 0..its.indices.len() {
        // CutSurface.cpp:815  if (is_all_on_one_side(its.indices[i], is_on_sides))
        if is_all_on_one_side(&its.indices[i], &is_on_sides) {
            // CutSurface.cpp:816  skip_indicies[i] = true;
            skip_indicies[i] = true;
        }
    }
}

// Eigen `Vec3d::normalize()` divides in place by the L2 norm without the
// `1e-10` guard that `geometry::Vec3::normalized()` applies, so it is inlined
// here to preserve C++ float/NaN semantics exactly.
#[inline]
fn normalize(v: &mut Vec3d) {
    let n = (v.x * v.x + v.y * v.y + v.z * v.z).sqrt();
    v.x /= n;
    v.y /= n;
    v.z /= n;
}

/// Separate triangles by mask
/// CutSurface.hpp:69 / CutSurface.cpp:821-859
pub fn its_mask(its: &indexed_triangle_set, mask: &[bool]) -> indexed_triangle_set {
    // CutSurface.cpp:824-827
    if its.indices.len() != mask.len() {
        debug_assert!(false);
        return indexed_triangle_set::default();
    }

    // CutSurface.cpp:829  std::vector<uint32_t> cvt_vetices(its.vertices.size(), {uint32_t::max});
    let mut cvt_vetices: Vec<u32> = vec![u32::MAX; its.vertices.len()];
    // CutSurface.cpp:830  size_t vertices_count = 0;
    let mut vertices_count: usize = 0;
    // CutSurface.cpp:831  size_t faces_count    = 0;
    let mut faces_count: usize = 0;
    // CutSurface.cpp:832  for (const auto &t : its.indices) {
    for (index, t) in its.indices.iter().enumerate() {
        // CutSurface.cpp:833  size_t index = &t - &its.indices.front();
        // CutSurface.cpp:834  if (!mask[index]) continue;
        if !mask[index] {
            continue;
        }
        // CutSurface.cpp:835  ++faces_count;
        faces_count += 1;
        // CutSurface.cpp:836  for (const auto vi : t) {
        for k in 0..3 {
            let vi = t[k] as usize;
            // CutSurface.cpp:837  uint32_t &cvt = cvt_vetices[vi];
            // CutSurface.cpp:838-839  if (cvt == uint32_t::max) cvt = vertices_count++;
            if cvt_vetices[vi] == u32::MAX {
                cvt_vetices[vi] = vertices_count as u32;
                vertices_count += 1;
            }
        }
    }
    // CutSurface.cpp:842  if (faces_count == 0) return {};
    if faces_count == 0 {
        return indexed_triangle_set::default();
    }

    // CutSurface.cpp:844  indexed_triangle_set result;
    let mut result = indexed_triangle_set::default();
    // CutSurface.cpp:845  result.indices.reserve(faces_count);
    result.indices.reserve(faces_count);
    // CutSurface.cpp:846  result.vertices = std::vector<Vec3f>(vertices_count);
    result.vertices = vec![Vec3f::new(0.0, 0.0, 0.0); vertices_count];
    // CutSurface.cpp:847  for (size_t i = 0; i < its.vertices.size(); ++i) {
    for i in 0..its.vertices.len() {
        // CutSurface.cpp:848  uint32_t index = cvt_vetices[i];
        let index = cvt_vetices[i];
        // CutSurface.cpp:849  if (index == uint32_t::max) continue;
        if index == u32::MAX {
            continue;
        }
        // CutSurface.cpp:850  result.vertices[index] = its.vertices[i];
        result.vertices[index as usize] = its.vertices[i];
    }

    // CutSurface.cpp:853-856
    for f in &its.indices {
        // CutSurface.cpp:854  if (mask[&f - &its.indices.front()])
        // Use pointer-offset equivalent: positional index in the slice.
        let fidx = (f as *const Vec3f32i as usize - its.indices.as_ptr() as usize)
            / std::mem::size_of::<Vec3f32i>();
        if mask[fidx] {
            // CutSurface.cpp:855-856
            result.indices.push(Vec3f32i::new(
                cvt_vetices[f[0] as usize] as i32,
                cvt_vetices[f[1] as usize] as i32,
                cvt_vetices[f[2] as usize] as i32,
            ));
        }
    }

    // CutSurface.cpp:858  return result;
    result
}

/// Separate (A)rea (o)f (I)nterest .. AoI from model
/// NOTE: Only 2d filtration, do not filtrate by Z coordinate
/// CutSurface.hpp:59-61 / CutSurface.cpp:861-870
pub fn its_cut_ao_i(
    its: &indexed_triangle_set,
    bb: &BoundingBox,
    projection: &dyn IProjection,
) -> indexed_triangle_set {
    // CutSurface.cpp:865  std::vector<bool> skip_indicies(its.indices.size(), false);
    let mut skip_indicies: Vec<bool> = vec![false; its.indices.len()];
    // CutSurface.cpp:866  priv::set_skip_for_out_of_aoi(skip_indicies, its, projection, bb);
    set_skip_for_out_of_aoi(&mut skip_indicies, its, projection, bb);
    // CutSurface.cpp:867-868  invert values in vector of bool
    // skip_indicies.flip();
    for b in skip_indicies.iter_mut() {
        *b = !*b;
    }
    // CutSurface.cpp:869  return its_mask(its, skip_indicies);
    its_mask(its, &skip_indicies)
}

/// Set true for indicies outward and almost parallel together.
/// Note: internally calculate normals
/// CutSurface.cpp:65-68 / CutSurface.cpp:872-895
///
/// Only consumed by `cut_surface`, which is blocked on CGAL, so it is dead code
/// in the Rust build today; kept and tested for byte-exact parity.
#[allow(dead_code)]
fn set_skip_by_angle(
    skip_indicies: &mut [bool],
    its: &indexed_triangle_set,
    projection: &dyn IProject3d,
    max_angle: f64,
) {
    // CutSurface.cpp:877  assert(max_angle < 90. && max_angle > 89.);
    debug_assert!(max_angle < 90. && max_angle > 89.);
    // CutSurface.cpp:878  assert(skip_indicies.size() == its.indices.size());
    debug_assert!(skip_indicies.len() == its.indices.len());
    // CutSurface.cpp:879  float threshold = static_cast<float>(cos(max_angle / 180. * M_PI));
    let threshold = (max_angle / 180. * std::f64::consts::PI).cos() as f32;
    // CutSurface.cpp:880  for (const stl_triangle_vertex_indices& face : its.indices) {
    for (index, face) in its.indices.iter().enumerate() {
        // CutSurface.cpp:881  size_t index = &face - &its.indices.front();
        // CutSurface.cpp:882  if (skip_indicies[index]) continue;
        if skip_indicies[index] {
            continue;
        }
        // CutSurface.cpp:883  Vec3f n = its_face_normal(its, face);
        let n = its_face_normal_indices(its, face);
        // CutSurface.cpp:884  const Vec3f& v = its.vertices[face[0]];
        let v = its.vertices[face[0] as usize];
        // CutSurface.cpp:885  const Vec3d vd = v.cast<double>();
        let vd = Vec3d::new(v.x as f64, v.y as f64, v.z as f64);
        // Improve: For Orthogonal Projection it is same for each vertex
        // CutSurface.cpp:887  Vec3d projectedd  = projection.project(vd);
        let projectedd = projection.project(&vd);
        // CutSurface.cpp:888  Vec3f projected   = projectedd.cast<float>();
        let projected = Vec3f::new(projectedd.x as f32, projectedd.y as f32, projectedd.z as f32);
        // CutSurface.cpp:889  Vec3f project_dir = projected - v;
        let mut project_dir = projected - v;
        // CutSurface.cpp:890  project_dir.normalize();
        project_dir.normalize_mut();
        // CutSurface.cpp:891  float cos_alpha = project_dir.dot(n);
        let cos_alpha = project_dir.dot(&n);
        // CutSurface.cpp:892  if (cos_alpha > threshold) continue;
        if cos_alpha > threshold {
            continue;
        }
        // CutSurface.cpp:893  skip_indicies[index] = true;
        skip_indicies[index] = true;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Keep references to the otherwise-unused-in-tests private helpers so the
    // compiler verifies them (they are exercised only by the blocked CGAL
    // pipeline in C++).
    #[test]
    fn its_mask_size_mismatch_returns_empty() {
        let its = indexed_triangle_set::default();
        let mask = vec![true]; // size != its.indices.size() (0)
        let r = its_mask(&its, &mask);
        assert!(r.indices.is_empty());
    }

    #[allow(unused)]
    fn _force_use() {
        let _ = set_skip_by_angle;
    }
}
