//! Mesh boolean operations — port of BambuStudio `src/libslic3r/MeshBoolean.cpp`.
//!
//! IMPORTANT (native backends): the C++ `MeshBoolean.cpp` is built entirely on top
//! of three native, non-wasm-safe C/C++ libraries:
//!
//!   * **libigl** (`igl::copyleft::cgal::mesh_boolean`) — the free-function
//!     `Slic3r::MeshBoolean::minus` / `self_union` overloads (MeshBoolean.cpp:38-105).
//!   * **CGAL** (`Surface_mesh`, `Polygon_mesh_processing::corefine_and_compute_*`,
//!     `does_self_intersect`, `does_bound_a_volume`, `sdf_values`,
//!     `segmentation_from_sdf_values`, hole filling, …) — the entire
//!     `Slic3r::MeshBoolean::cgal` namespace (MeshBoolean.cpp:107-483).
//!   * **mcut** (`mcCreateContext`, `mcDispatch`, `mcGetConnectedComponents`, …) — the
//!     boolean compute kernel `do_boolean_single` inside the
//!     `Slic3r::MeshBoolean::mcut` namespace (MeshBoolean.cpp:623-822).
//!
//! None of these have a Rust equivalent in this crate, and the crate is wasm-safe
//! (no system/dylib deps), so the compute kernels are deliberately NOT bound here.
//! See the BLOCKED list at the bottom of this file.
//!
//! What IS ported faithfully (and needs no native backend) is the `mcut` namespace's
//! pure data-marshalling layer: the `McutMesh` array-of-doubles / array-of-uint32
//! representation and the conversions to and from our `TriangleMesh`. These are exact
//! 1:1 translations and feed the (native) mcut dispatch in the C++ build.
//!
//! `coord_t -> i64`, `coordf_t -> f64`. Vertices are stored single-precision
//! (`Vec3f` / `Point3F`) exactly as in C++ `indexed_triangle_set`.

use crate::geometry::Point3F;
use crate::triangle_mesh::{Triangle, TriangleMesh};

// MeshBoolean.cpp:32   namespace Slic3r {
// MeshBoolean.cpp:33   namespace MeshBoolean {
//
// The free-function igl overloads (eigen_to_triangle_mesh, triangle_mesh_to_eigen,
// minus, self_union — MeshBoolean.cpp:38-105) require libigl + CGAL and are BLOCKED.
// The whole `cgal` namespace (MeshBoolean.cpp:107-483) requires CGAL and is BLOCKED.

// MeshBoolean.cpp:486   namespace mcut {
pub mod mcut {
    use super::*;

    /// MeshBoolean.cpp:487-496
    /// /* BBS: MusangKing
    ///  * mcut mesh array format for Boolean Opts calculation
    ///  */
    /// struct McutMesh
    /// {
    ///     // variables for mesh data in a format suited for mcut
    ///     std::vector<uint32_t> faceSizesArray;
    ///     std::vector<uint32_t> faceIndicesArray;
    ///     std::vector<double>   vertexCoordsArray;
    /// };
    #[derive(Debug, Clone, Default)]
    pub struct McutMesh {
        // variables for mesh data in a format suited for mcut
        /// MeshBoolean.cpp:493
        pub face_sizes_array: Vec<u32>,
        /// MeshBoolean.cpp:494
        pub face_indices_array: Vec<u32>,
        /// MeshBoolean.cpp:495
        pub vertex_coords_array: Vec<f64>,
    }

    // MeshBoolean.cpp:497   void McutMeshDeleter::operator()(McutMesh *ptr) { delete ptr; }
    // (Rust manages the lifetime of `McutMesh` directly; the C++ custom deleter for the
    //  `unique_ptr<McutMesh, McutMeshDeleter>` PIMPL has no Rust analog.)

    /// MeshBoolean.cpp:499
    /// `bool empty(const McutMesh &mesh)`
    pub fn empty(mesh: &McutMesh) -> bool {
        // MeshBoolean.cpp:499
        mesh.vertex_coords_array.is_empty() || mesh.face_indices_array.is_empty()
    }

    /// MeshBoolean.cpp:500-524
    /// `void triangle_mesh_to_mcut(const TriangleMesh &src_mesh, McutMesh &srcMesh,
    ///                             const Transform3d &src_nm = Transform3d::Identity())`
    ///
    /// NOTE on the `src_nm` transform argument (MeshBoolean.cpp:500): the only caller in
    /// this file (`make_boolean`, MeshBoolean.cpp:899-900) relies on the default
    /// `Transform3d::Identity()`, so this port applies the identity directly
    /// (`v = its.vertices[i].cast<double>()`). The transform code path is BLOCKED on the
    /// `Transform3d` matrix-vector product that the C++ default never exercises; it is
    /// noted here for parity completeness.
    pub fn triangle_mesh_to_mcut(src_mesh: &TriangleMesh, src_mesh_out: &mut McutMesh) {
        // MeshBoolean.cpp:503  vertices precision convention and copy
        // srcMesh.vertexCoordsArray.reserve(src_mesh.its.vertices.size() * 3);
        src_mesh_out
            .vertex_coords_array
            .reserve(src_mesh.vertices().len() * 3);
        // MeshBoolean.cpp:504-509
        // for (int i = 0; i < src_mesh.its.vertices.size(); ++i) {
        for i in 0..src_mesh.vertices().len() {
            // const Vec3d v = src_nm * src_mesh.its.vertices[i].cast<double>();
            // src_nm == Transform3d::Identity() at the sole call site.
            let v = src_mesh.vertices()[i];
            // srcMesh.vertexCoordsArray.push_back(v[0]);
            src_mesh_out.vertex_coords_array.push(v.x as f64);
            // srcMesh.vertexCoordsArray.push_back(v[1]);
            src_mesh_out.vertex_coords_array.push(v.y as f64);
            // srcMesh.vertexCoordsArray.push_back(v[2]);
            src_mesh_out.vertex_coords_array.push(v.z as f64);
        }

        // MeshBoolean.cpp:511  faces copy
        // srcMesh.faceIndicesArray.reserve(src_mesh.its.indices.size() * 3);
        src_mesh_out
            .face_indices_array
            .reserve(src_mesh.indices().len() * 3);
        // MeshBoolean.cpp:513
        // srcMesh.faceSizesArray.reserve(src_mesh.its.indices.size());
        src_mesh_out
            .face_sizes_array
            .reserve(src_mesh.indices().len());
        // MeshBoolean.cpp:514-523
        // for (int i = 0; i < src_mesh.its.indices.size(); ++i) {
        for i in 0..src_mesh.indices().len() {
            // const int &f0 = src_mesh.its.indices[i][0];
            let f0 = src_mesh.indices()[i].indices[0];
            // const int &f1 = src_mesh.its.indices[i][1];
            let f1 = src_mesh.indices()[i].indices[1];
            // const int &f2 = src_mesh.its.indices[i][2];
            let f2 = src_mesh.indices()[i].indices[2];
            // srcMesh.faceIndicesArray.push_back(f0);
            src_mesh_out.face_indices_array.push(f0);
            // srcMesh.faceIndicesArray.push_back(f1);
            src_mesh_out.face_indices_array.push(f1);
            // srcMesh.faceIndicesArray.push_back(f2);
            src_mesh_out.face_indices_array.push(f2);

            // srcMesh.faceSizesArray.push_back((uint32_t) 3);
            src_mesh_out.face_sizes_array.push(3u32);
        }
    }

    /// MeshBoolean.cpp:526-532
    /// `McutMeshPtr triangle_mesh_to_mcut(const indexed_triangle_set &M)`
    ///
    /// In C++ this takes an `indexed_triangle_set`, wraps it in a temporary
    /// `TriangleMesh` and forwards to the overload above. The crate's primary mesh type
    /// already is the `TriangleMesh`, so this port takes the `TriangleMesh` directly and
    /// returns the owned `McutMesh` (the C++ `unique_ptr<McutMesh, McutMeshDeleter>` PIMPL
    /// becomes a plain owned value in Rust).
    pub fn triangle_mesh_to_mcut_from_its(m: &TriangleMesh) -> McutMesh {
        // MeshBoolean.cpp:528  std::unique_ptr<McutMesh, McutMeshDeleter> out(new McutMesh{});
        let mut out = McutMesh::default();
        // MeshBoolean.cpp:529  TriangleMesh trimesh(M);
        // MeshBoolean.cpp:530  triangle_mesh_to_mcut(trimesh, *out.get());
        triangle_mesh_to_mcut(m, &mut out);
        // MeshBoolean.cpp:531  return out;
        out
    }

    /// MeshBoolean.cpp:534-564
    /// `TriangleMesh mcut_to_triangle_mesh(const McutMesh &mcutmesh)`
    pub fn mcut_to_triangle_mesh(mcutmesh: &McutMesh) -> TriangleMesh {
        // MeshBoolean.cpp:536  uint32_t ccVertexCount = mcutmesh.vertexCoordsArray.size() / 3;
        let cc_vertex_count: u32 = (mcutmesh.vertex_coords_array.len() / 3) as u32;
        // MeshBoolean.cpp:537  auto &ccVertices = mcutmesh.vertexCoordsArray;
        let cc_vertices = &mcutmesh.vertex_coords_array;
        // MeshBoolean.cpp:538  auto &ccFaceIndices = mcutmesh.faceIndicesArray;
        let cc_face_indices = &mcutmesh.face_indices_array;
        // MeshBoolean.cpp:539  auto &faceSizes = mcutmesh.faceSizesArray;
        let face_sizes = &mcutmesh.face_sizes_array;
        // MeshBoolean.cpp:540  uint32_t ccFaceCount = faceSizes.size();
        let cc_face_count: u32 = face_sizes.len() as u32;
        // MeshBoolean.cpp:541  rearrange vertices/faces and save into result mesh
        // MeshBoolean.cpp:542  std::vector<Vec3f> vertices(ccVertexCount);
        let mut vertices: Vec<Point3F> = vec![Point3F::zero(); cc_vertex_count as usize];
        // MeshBoolean.cpp:543-547
        // for (uint32_t i = 0; i < ccVertexCount; i++) {
        for i in 0..cc_vertex_count {
            // vertices[i][0] = (float) ccVertices[(uint64_t) i * 3 + 0];
            vertices[i as usize].x = cc_vertices[(i as u64 * 3 + 0) as usize] as f32 as f64;
            // vertices[i][1] = (float) ccVertices[(uint64_t) i * 3 + 1];
            vertices[i as usize].y = cc_vertices[(i as u64 * 3 + 1) as usize] as f32 as f64;
            // vertices[i][2] = (float) ccVertices[(uint64_t) i * 3 + 2];
            vertices[i as usize].z = cc_vertices[(i as u64 * 3 + 2) as usize] as f32 as f64;
        }

        // MeshBoolean.cpp:549-550  output faces
        // int faceVertexOffsetBase = 0;
        let mut face_vertex_offset_base: i32 = 0;

        // MeshBoolean.cpp:552-553  for each face in CC
        // std::vector<Vec3i> faces(ccFaceCount);
        let mut faces: Vec<Triangle> = vec![Triangle::new(0, 0, 0); cc_face_count as usize];
        // MeshBoolean.cpp:554-560
        // for (uint32_t f = 0; f < ccFaceCount; ++f) {
        for f in 0..cc_face_count {
            // int faceSize = faceSizes.at(f);
            let face_size: i32 = face_sizes[f as usize] as i32;

            // for each vertex in face
            // for (int v = 0; v < faceSize; v++) { faces[f][v] = ccFaceIndices[(uint64_t) faceVertexOffsetBase + v]; }
            for v in 0..face_size {
                faces[f as usize].indices[v as usize] =
                    cc_face_indices[(face_vertex_offset_base as u64 + v as u64) as usize];
            }
            // faceVertexOffsetBase += faceSize;
            face_vertex_offset_base += face_size;
        }

        // MeshBoolean.cpp:562  TriangleMesh out(vertices, faces);
        // MeshBoolean.cpp:563  return out;
        TriangleMesh::from_parts(vertices, faces)
    }

    /// MeshBoolean.cpp:566-573
    /// `void merge_mcut_meshes(McutMesh& src, const McutMesh& cut)`
    pub fn merge_mcut_meshes(src: &mut McutMesh, cut: &McutMesh) {
        // MeshBoolean.cpp:567  indexed_triangle_set all_its;
        // MeshBoolean.cpp:568  TriangleMesh tri_src = mcut_to_triangle_mesh(src);
        let mut all = TriangleMesh::new();
        let tri_src = mcut_to_triangle_mesh(src);
        // MeshBoolean.cpp:569  TriangleMesh tri_cut = mcut_to_triangle_mesh(cut);
        let tri_cut = mcut_to_triangle_mesh(cut);
        // MeshBoolean.cpp:570  its_merge(all_its, tri_src.its);
        all.merge(tri_src);
        // MeshBoolean.cpp:571  its_merge(all_its, tri_cut.its);
        all.merge(tri_cut);
        // MeshBoolean.cpp:572  src = *triangle_mesh_to_mcut(all_its);
        *src = triangle_mesh_to_mcut_from_its(&all);
    }

    // MeshBoolean.cpp:575-620   mcDebugOutput — debug callback for the mcut C API, BLOCKED.
    // MeshBoolean.cpp:623-822   do_boolean_single — mcCreateContext/mcDispatch/
    //                           mcGetConnectedComponents kernel, BLOCKED (native mcut).
    // MeshBoolean.cpp:824-894   do_boolean — control flow over its_split parts driving
    //                           the (BLOCKED) do_boolean_single kernel, BLOCKED.
    // MeshBoolean.cpp:896-931   make_boolean — wraps do_boolean then split/fix-volume/
    //                           merge post-processing; BLOCKED on do_boolean above.
} // namespace mcut

// MeshBoolean.cpp:936   } // namespace MeshBoolean
// MeshBoolean.cpp:937   } // namespace Slic3r

#[cfg(test)]
mod tests {
    use super::mcut::*;
    use crate::geometry::Point3F;
    use crate::triangle_mesh::{Triangle, TriangleMesh};

    fn unit_tri_mesh() -> TriangleMesh {
        let vertices = vec![
            Point3F::new(0.0, 0.0, 0.0),
            Point3F::new(1.0, 0.0, 0.0),
            Point3F::new(0.0, 1.0, 0.0),
        ];
        let faces = vec![Triangle::new(0, 1, 2)];
        TriangleMesh::from_parts(vertices, faces)
    }

    #[test]
    fn test_empty_mcut_mesh() {
        let m = McutMesh::default();
        assert!(empty(&m));
    }

    #[test]
    fn test_triangle_mesh_to_mcut_roundtrip() {
        let mesh = unit_tri_mesh();
        let mut mc = McutMesh::default();
        triangle_mesh_to_mcut(&mesh, &mut mc);

        // 3 vertices * 3 coords each.
        assert_eq!(mc.vertex_coords_array.len(), 9);
        // 1 face * 3 indices.
        assert_eq!(mc.face_indices_array.len(), 3);
        // 1 face, size 3.
        assert_eq!(mc.face_sizes_array, vec![3u32]);
        assert!(!empty(&mc));

        // First vertex coords (0,0,0), second (1,0,0).
        assert_eq!(mc.vertex_coords_array[0], 0.0);
        assert_eq!(mc.vertex_coords_array[3], 1.0);

        let back = mcut_to_triangle_mesh(&mc);
        assert_eq!(back.vertex_count(), 3);
        assert_eq!(back.triangle_count(), 1);
        assert_eq!(back.indices()[0].indices, [0, 1, 2]);
    }

    #[test]
    fn test_triangle_mesh_to_mcut_from_its() {
        let mesh = unit_tri_mesh();
        let mc = triangle_mesh_to_mcut_from_its(&mesh);
        assert_eq!(mc.vertex_coords_array.len(), 9);
        assert_eq!(mc.face_sizes_array.len(), 1);
    }

    #[test]
    fn test_merge_mcut_meshes() {
        let mesh = unit_tri_mesh();
        let mut src = McutMesh::default();
        triangle_mesh_to_mcut(&mesh, &mut src);
        let mut cut = McutMesh::default();
        triangle_mesh_to_mcut(&mesh, &mut cut);

        merge_mcut_meshes(&mut src, &cut);
        // After merge: 6 vertices, 2 faces.
        assert_eq!(src.vertex_coords_array.len(), 18);
        assert_eq!(src.face_sizes_array.len(), 2);
    }
}
