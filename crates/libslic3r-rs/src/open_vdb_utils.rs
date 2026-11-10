//! Faithful 1:1 port of `OpenVDBUtils.{hpp,cpp}` from BambuStudio libslic3r.
//!
//! C++ Reference:
//! - src/libslic3r/OpenVDBUtils.hpp
//! - src/libslic3r/OpenVDBUtils.cpp
//!
//! These utilities convert between `indexed_triangle_set` meshes and OpenVDB
//! `FloatGrid` level-set grids (mesh -> grid, grid -> mesh, redistance).
//!
//! ## Native-dependency status (PARTIAL port)
//!
//! Every grid-producing/consuming function in this file is a thin wrapper over
//! the **native** OpenVDB C++ library:
//!   - `openvdb::FloatGrid` (the sparse voxel/level-set grid type)
//!   - `openvdb::math::Transform`
//!   - `openvdb::tools::meshToVolume` / `volumeToMesh`
//!   - `openvdb::tools::csgUnion` / `levelSetRebuild`
//!   - `openvdb::initialize`, `openvdb::FloatMetadata`
//!
//! OpenVDB is a heavyweight C++ dependency (pulling in TBB, Boost, Blosc); there
//! is no pure-Rust port, and per the porting rules we must NOT add a native /
//! dylib dependency here (it is not wasm-safe). Therefore the three public grid
//! functions — `mesh_to_grid`, `grid_to_mesh`, `redistance_grid` — are
//! **blocked on the native OpenVDB backend** and are documented (with exact C++
//! line refs) rather than faked.
//!
//! Note: their *non*-OpenVDB dependencies `its_split` / `its_volume` (used at
//! `OpenVDBUtils.cpp:57` and `:60`) ARE now faithfully ported — see
//! `triangle_mesh::its_volume` (TriangleMesh.cpp:1827-1846) and
//! `triangle_mesh::its_split` (TriangleMesh.cpp:1863-1866, dispatching to
//! `mesh_split_impl::its_split_collect`). So the *only* remaining blocker for the
//! three grid functions is the native OpenVDB level-set machinery itself
//! (`meshToVolume` / `volumeToMesh` / `csgUnion` / `levelSetRebuild` /
//! `FloatMetadata`); the surrounding control flow is otherwise portable.
//!
//! What IS faithfully ported here (OpenVDB-algorithm-free, exact logic):
//!   - `TriangleMeshDataAdapter` (the mesh -> index-space-point adapter)
//!   - the inline header helpers `to_vec3f` / `to_vec3d` / `to_vec3i`
//!
//! coord_t -> i64, coordf_t -> f64 per the porting convention (none appear in
//! this file; meshes use `Vec3f` vertices and `Vec3i` indices exactly as C++).

// OpenVDBUtils.cpp:20  namespace Slic3r {

use crate::triangle_set_sampling::{indexed_triangle_set, Vec3d, Vec3f, Vec3i};

// ----------------------------------------------------------------------------
// Inline header helpers (OpenVDBUtils.hpp:18-20)
//
// In C++ these take `openvdb::Vec3s` (a `Vec3<float>`) and `openvdb::Vec3I`
// (a `Vec3<uint32_t>`). We model those native triples as `[f32; 3]` / `[u32; 3]`
// so the conversion logic is byte-exact. (These exist only to convert OpenVDB
// `volumeToMesh` output into Slic3r types; see `grid_to_mesh` below.)
// ----------------------------------------------------------------------------

/// OpenVDBUtils.hpp:18
/// `inline Vec3f to_vec3f(const openvdb::Vec3s &v) { return Vec3f{v.x(), v.y(), v.z()}; }`
#[inline]
pub fn to_vec3f(v: [f32; 3]) -> Vec3f {
    Vec3f::new(v[0], v[1], v[2])
}

/// OpenVDBUtils.hpp:19
/// `inline Vec3d to_vec3d(const openvdb::Vec3s &v) { return to_vec3f(v).cast<double>(); }`
#[inline]
pub fn to_vec3d(v: [f32; 3]) -> Vec3d {
    to_vec3f(v).cast::<f64>()
}

/// OpenVDBUtils.hpp:20
/// `inline Vec3i to_vec3i(const openvdb::Vec3I &v) { return Vec3i{int(v[0]), int(v[1]), int(v[2])}; }`
#[inline]
pub fn to_vec3i(v: [u32; 3]) -> Vec3i {
    Vec3i::new(v[0] as i32, v[1] as i32, v[2] as i32)
}

// ----------------------------------------------------------------------------
// TriangleMeshDataAdapter (OpenVDBUtils.cpp:22-43)
//
// Mesh adapter consumed by `openvdb::tools::meshToVolume`. Fully portable: it
// only reads `indexed_triangle_set` and applies a uniform `voxel_scale`. The
// `openvdb::Vec3d&` out-parameter of `getIndexSpacePoint` is modelled as a
// returned `Vec3d` (a plain 3-double), preserving the exact computation.
// ----------------------------------------------------------------------------

// OpenVDBUtils.cpp:22  class TriangleMeshDataAdapter {
pub struct TriangleMeshDataAdapter<'a> {
    // OpenVDBUtils.cpp:24  const indexed_triangle_set &its;
    pub its: &'a indexed_triangle_set,
    // OpenVDBUtils.cpp:25  float voxel_scale;
    pub voxel_scale: f32,
}

impl<'a> TriangleMeshDataAdapter<'a> {
    // OpenVDBUtils.cpp:27  size_t polygonCount() const { return its.indices.size(); }
    pub fn polygon_count(&self) -> usize {
        self.its.indices.len()
    }

    // OpenVDBUtils.cpp:28  size_t pointCount() const { return its.vertices.size(); }
    pub fn point_count(&self) -> usize {
        self.its.vertices.len()
    }

    // OpenVDBUtils.cpp:29  size_t vertexCount(size_t) const { return 3; }
    pub fn vertex_count(&self, _n: usize) -> usize {
        3
    }

    // OpenVDBUtils.cpp:31-39
    // Return position pos in local grid index space for polygon n and vertex v
    // The actual mesh will appear to openvdb as scaled uniformly by voxel_size
    // And the voxel count per unit volume can be affected this way.
    // void getIndexSpacePoint(size_t n, size_t v, openvdb::Vec3d& pos) const
    pub fn get_index_space_point(&self, n: usize, v: usize) -> Vec3d {
        // OpenVDBUtils.cpp:36  auto vidx = size_t(its.indices[n](Eigen::Index(v)));
        let vidx = self.its.indices[n][v] as usize;
        // OpenVDBUtils.cpp:37  Slic3r::Vec3d p = its.vertices[vidx].cast<double>() * voxel_scale;
        let p: Vec3d = self.its.vertices[vidx].cast::<f64>() * (self.voxel_scale as f64);
        // OpenVDBUtils.cpp:38  pos = {p.x(), p.y(), p.z()};
        Vec3d::new(p.x, p.y, p.z)
    }

    // OpenVDBUtils.cpp:41-42
    // TriangleMeshDataAdapter(const indexed_triangle_set &m, float voxel_sc = 1.f)
    //     : its{m}, voxel_scale{voxel_sc} {};
    pub fn new(m: &'a indexed_triangle_set, voxel_scale: f32) -> Self {
        Self {
            its: m,
            voxel_scale,
        }
    }
}

// ----------------------------------------------------------------------------
// BLOCKED on native OpenVDB backend — NOT ported (no fakes).
//
// The following three functions cannot be faithfully ported without the native
// OpenVDB library, which we must not add (not wasm-safe). They are recorded here
// in full with exact C++ line refs so a future FFI/native-backend porter can
// wire them up. They are intentionally left unimplemented (commented) rather
// than stubbed with fake return values that would silently corrupt G-code
// parity.
//
// // TODO: Do I need to call initialize? Seems to work without it as well but the
// // docs say it should be called ones. It does a mutex lock-unlock sequence all
// // even if was called previously.
// openvdb::FloatGrid::Ptr mesh_to_grid(const indexed_triangle_set &    mesh,
//                                      const openvdb::math::Transform &tr,
//                                      float voxel_scale,
//                                      float exteriorBandWidth,
//                                      float interiorBandWidth,
//                                      int   flags)                         // .cpp:48-87
// {
//     openvdb::initialize();                                                // .cpp:55  (NATIVE OpenVDB)
//     std::vector<indexed_triangle_set> meshparts = its_split(mesh);        // .cpp:57  (PORTED: triangle_mesh::its_split)
//     auto it = std::remove_if(meshparts.begin(), meshparts.end(),
//                              [](auto &m) { return its_volume(m) < EPSILON; }); // .cpp:59-60  (PORTED: triangle_mesh::its_volume; EPSILON = libslic3r::EPSILON = 1e-4)
//     meshparts.erase(it, meshparts.end());                                 // .cpp:62
//     openvdb::FloatGrid::Ptr grid;                                         // .cpp:64
//     for (auto &m : meshparts) {                                           // .cpp:65
//         auto subgrid = openvdb::tools::meshToVolume<openvdb::FloatGrid>(  // .cpp:66  (NATIVE OpenVDB)
//             TriangleMeshDataAdapter{m, voxel_scale}, tr, exteriorBandWidth,
//             interiorBandWidth, flags);                                    // .cpp:67-68
//         if (grid && subgrid) openvdb::tools::csgUnion(*grid, *subgrid);   // .cpp:70  (NATIVE OpenVDB)
//         else if (subgrid) grid = std::move(subgrid);                      // .cpp:71
//     }
//     if (grid) {                                                          // .cpp:74
//         grid = openvdb::tools::levelSetRebuild(*grid, 0., exteriorBandWidth,
//                                                interiorBandWidth);        // .cpp:75-76  (NATIVE OpenVDB)
//     } else if(meshparts.empty()) {                                        // .cpp:77
//         // Splitting failed, fall back to hollow the original mesh
//         grid = openvdb::tools::meshToVolume<openvdb::FloatGrid>(          // .cpp:79  (NATIVE OpenVDB)
//             TriangleMeshDataAdapter{mesh}, tr, exteriorBandWidth,
//             interiorBandWidth, flags);                                    // .cpp:80-81
//     }
//     grid->insertMeta("voxel_scale", openvdb::FloatMetadata(voxel_scale)); // .cpp:84
//     return grid;                                                          // .cpp:86
// }
//
// indexed_triangle_set grid_to_mesh(const openvdb::FloatGrid &grid,
//                           double                    isovalue,
//                           double                    adaptivity,
//                           bool                      relaxDisorientedTriangles) // .cpp:89-120
// {
//     openvdb::initialize();                                                // .cpp:94
//     std::vector<openvdb::Vec3s> points;                                   // .cpp:96
//     std::vector<openvdb::Vec3I> triangles;                                // .cpp:97
//     std::vector<openvdb::Vec4I> quads;                                    // .cpp:98
//     openvdb::tools::volumeToMesh(grid, points, triangles, quads, isovalue,
//                                  adaptivity, relaxDisorientedTriangles);  // .cpp:100-101  (NATIVE OpenVDB)
//     float scale = 1.;                                                     // .cpp:103
//     try { scale = grid.template metaValue<float>("voxel_scale"); }
//     catch (...) { }                                                       // .cpp:104-106
//     indexed_triangle_set ret;                                             // .cpp:108
//     ret.vertices.reserve(points.size());                                  // .cpp:109
//     ret.indices.reserve(triangles.size() + quads.size() * 2);             // .cpp:110
//     for (auto &v : points) ret.vertices.emplace_back(to_vec3f(v) / scale);// .cpp:112
//     for (auto &v : triangles) ret.indices.emplace_back(to_vec3i(v));      // .cpp:113
//     for (auto &quad : quads) {                                            // .cpp:114
//         ret.indices.emplace_back(quad(0), quad(1), quad(2));              // .cpp:115
//         ret.indices.emplace_back(quad(2), quad(3), quad(0));              // .cpp:116
//     }
//     return ret;                                                           // .cpp:119
// }
//
// openvdb::FloatGrid::Ptr redistance_grid(const openvdb::FloatGrid &grid,
//                                         double                    iso,
//                                         double                    er,
//                                         double                    ir)     // .cpp:122-134
// {
//     auto new_grid = openvdb::tools::levelSetRebuild(grid, float(iso),
//                                                     float(er), float(ir)); // .cpp:127-128  (NATIVE OpenVDB)
//     // Copies voxel_scale metadata, if it exists.
//     new_grid->insertMeta(*grid.deepCopyMeta());                           // .cpp:131
//     return new_grid;                                                      // .cpp:133
// }
// ----------------------------------------------------------------------------

// OpenVDBUtils.cpp:136  } // namespace Slic3r

#[cfg(test)]
mod tests {
    use super::*;
    use crate::triangle_set_sampling::indexed_triangle_set;

    #[test]
    fn to_vec3f_passthrough() {
        // OpenVDBUtils.hpp:18 — Vec3f{v.x(), v.y(), v.z()}
        let r = to_vec3f([1.5_f32, -2.0, 3.25]);
        assert_eq!(r, Vec3f::new(1.5, -2.0, 3.25));
    }

    #[test]
    fn to_vec3d_casts_f32_to_f64() {
        // OpenVDBUtils.hpp:19 — to_vec3f(v).cast<double>()
        let r = to_vec3d([0.5_f32, 1.0, 2.0]);
        // Cast goes through f32 first, exactly like C++.
        assert_eq!(r, Vec3d::new(0.5_f32 as f64, 1.0_f32 as f64, 2.0_f32 as f64));
    }

    #[test]
    fn to_vec3i_truncates_u32_to_int() {
        // OpenVDBUtils.hpp:20 — Vec3i{int(v[0]), int(v[1]), int(v[2])}
        let r = to_vec3i([0_u32, 7, 42]);
        assert_eq!(r, Vec3i::new(0, 7, 42));
    }

    #[test]
    fn adapter_counts_and_index_space_point() {
        // A single triangle scaled by voxel_scale = 2.
        let mut its = indexed_triangle_set::default();
        its.vertices.push(Vec3f::new(1.0, 2.0, 3.0));
        its.vertices.push(Vec3f::new(4.0, 5.0, 6.0));
        its.vertices.push(Vec3f::new(7.0, 8.0, 9.0));
        its.indices.push(Vec3i::new(0, 1, 2));

        let adapter = TriangleMeshDataAdapter::new(&its, 2.0);

        // OpenVDBUtils.cpp:27-29
        assert_eq!(adapter.polygon_count(), 1);
        assert_eq!(adapter.point_count(), 3);
        assert_eq!(adapter.vertex_count(0), 3);

        // OpenVDBUtils.cpp:36-38 — vertices[indices[n](v)].cast<double>() * voxel_scale
        let p = adapter.get_index_space_point(0, 1);
        assert_eq!(p, Vec3d::new(8.0, 10.0, 12.0));
    }
}
