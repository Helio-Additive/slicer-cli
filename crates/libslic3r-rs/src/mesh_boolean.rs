//! Port of `MeshBoolean.hpp` / `MeshBoolean.cpp` from Bambu Studio's libslic3r (pin 56a57d0).
//!
//! Mesh booleans over three third-party libraries: libigl (`igl::copyleft::cgal::mesh_boolean`,
//! the `EigenMesh` overloads), CGAL (`Surface_mesh`, `Polygon_mesh_processing`, the mesh
//! segmentation; namespace `cgal`) and mcut (namespace `mcut`). slicer_cli builds this file with
//! `SLIC3R_ENABLE_OPENMESHCRAFT_BOOLEAN` off, so the `#else` branches are the ones ported:
//! `openmeshcraft::is_enabled()` is false and the dispatcher `make_boolean` runs mcut.
//!
//! Everything libslic3r does around the library calls is ported in the order of the C++: the
//! conversions to and from the libraries' mesh types, the control flow of the booleans, the
//! splitting of the parts, the fallbacks on failure, the progress and the winding of the result.
//! The library calls themselves are the methods of three traits, as `texture_to_color/cgal_utils.rs`
//! does for the CGAL calls of CgalUtils.cpp:
//! - [`IglMeshBoolean`]: `igl::copyleft::cgal::mesh_boolean`;
//! - [`cgal::CgalMeshBooleanKernel`]: the corefinement booleans, the self intersection and volume
//!   tests, the polygon soup orientation, the SDF segmentation, hole filling and stitching;
//! - [`mcut::McutApi`]: the mcut C API (`mcCreateContext`, `mcDispatch`,
//!   `mcGetConnectedComponents`, `mcGetConnectedComponentData`, the releases).
//!
//! FIDELITY-NOTE: the crate implements these traits nowhere (the libraries are native C/C++,
//! not wasm-safe, and their cores are not ported), so the functions that take one cannot be run by
//! the engine yet. The CGAL mesh is `texture_to_color::cgal_utils::CGALMesh`, the crate's port of
//! `CGAL::Surface_mesh` over doubles, which is the `Exact_predicates_inexact_constructions_kernel`
//! mesh `_EpicMesh` of the C++.
//!
//! A C++ `TriangleMesh` is [`crate::model::ModelMesh`] (its `its` and its stats).

use crate::model::ModelMesh as TriangleMesh;
use crate::normal_utils::indexed_triangle_set;
use crate::triangle_mesh::{Vec3f, Vec3i};

/// The error a function of this file throws: `Slic3r::RuntimeError` (Exception.hpp) with its text.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MeshBooleanError(pub String);

impl std::fmt::Display for MeshBooleanError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}

impl std::error::Error for MeshBooleanError {}

// =============================================================================================
// MeshBoolean.cpp:35-105 — the libigl overloads
// =============================================================================================

/// MeshBoolean.hpp:14 `using EigenMesh = std::pair<Eigen::MatrixXd, Eigen::MatrixXi>`: a vertex
/// matrix (one row of three doubles per vertex) and a face matrix (one row of three indices).
pub type EigenMesh = (nalgebra::DMatrix<f64>, nalgebra::DMatrix<i32>);

/// `igl::MeshBooleanType` (igl/MeshBooleanType.h), the values MeshBoolean.cpp uses.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IglMeshBooleanType {
    /// `igl::MESH_BOOLEAN_TYPE_UNION`
    Union = 0,
    /// `igl::MESH_BOOLEAN_TYPE_INTERSECT`
    Intersect = 1,
    /// `igl::MESH_BOOLEAN_TYPE_MINUS`
    Minus = 2,
}

/// `igl::copyleft::cgal::mesh_boolean(VA, FA, VB, FB, type, VC, FC)`: libigl's exact boolean on
/// CGAL. FIDELITY-NOTE: implemented nowhere in the crate (see the top of the file).
pub trait IglMeshBoolean {
    #[allow(clippy::too_many_arguments)]
    fn mesh_boolean(
        &self,
        va: &nalgebra::DMatrix<f64>,
        fa: &nalgebra::DMatrix<i32>,
        vb: &nalgebra::DMatrix<f64>,
        fb: &nalgebra::DMatrix<i32>,
        boolean_type: IglMeshBooleanType,
        vc: &mut nalgebra::DMatrix<f64>,
        fc: &mut nalgebra::DMatrix<i32>,
    ) -> bool;
}

/// MeshBoolean.cpp:38-53 `TriangleMesh eigen_to_triangle_mesh(const EigenMesh &emesh)`
pub fn eigen_to_triangle_mesh(emesh: &EigenMesh) -> TriangleMesh {
    let (vc, fc) = (&emesh.0, &emesh.1);

    let mut its = indexed_triangle_set::default();
    its.vertices.reserve(vc.nrows());
    its.indices.reserve(fc.nrows());

    for i in 0..vc.nrows() {
        its.vertices.push(Vec3f::new(vc[(i, 0)] as f32, vc[(i, 1)] as f32, vc[(i, 2)] as f32));
    }

    for i in 0..fc.nrows() {
        its.indices.push(Vec3i::new(fc[(i, 0)], fc[(i, 1)], fc[(i, 2)]));
    }

    TriangleMesh::from_its(its)
}

/// MeshBoolean.cpp:55-66 `EigenMesh triangle_mesh_to_eigen(const TriangleMesh &mesh)`: the
/// row-major float vertices mapped and cast to double, the indices mapped.
pub fn triangle_mesh_to_eigen(mesh: &TriangleMesh) -> EigenMesh {
    let vertices = &mesh.its.vertices;
    let indices = &mesh.its.indices;
    let first = nalgebra::DMatrix::<f64>::from_fn(vertices.len(), 3, |r, c| vertices[r][c] as f64);
    let second = nalgebra::DMatrix::<i32>::from_fn(indices.len(), 3, |r, c| indices[r][c]);
    (first, second)
}

/// MeshBoolean.cpp:68-79 `void minus(EigenMesh &A, const EigenMesh &B)`
pub fn minus_eigen(igl: &dyn IglMeshBoolean, a: &mut EigenMesh, b: &EigenMesh) {
    let mut vc = nalgebra::DMatrix::<f64>::zeros(0, 0);
    let mut fc = nalgebra::DMatrix::<i32>::zeros(0, 0);
    let boolean_type = IglMeshBooleanType::Minus;
    igl.mesh_boolean(&a.0, &a.1, &b.0, &b.1, boolean_type, &mut vc, &mut fc);

    a.0 = vc;
    a.1 = fc;
}

/// MeshBoolean.cpp:81-86 `void minus(TriangleMesh& A, const TriangleMesh& B)`
pub fn minus(igl: &dyn IglMeshBoolean, a: &mut TriangleMesh, b: &TriangleMesh) {
    let mut e_a = triangle_mesh_to_eigen(a);
    minus_eigen(igl, &mut e_a, &triangle_mesh_to_eigen(b));
    *a = eigen_to_triangle_mesh(&e_a);
}

/// MeshBoolean.cpp:88-98 `void self_union(EigenMesh &A)`
pub fn self_union_eigen(igl: &dyn IglMeshBoolean, a: &mut EigenMesh) {
    let mut result: EigenMesh = (nalgebra::DMatrix::<f64>::zeros(0, 0), nalgebra::DMatrix::<i32>::zeros(0, 0));

    let boolean_type = IglMeshBooleanType::Union;
    igl.mesh_boolean(
        &a.0,
        &a.1,
        &nalgebra::DMatrix::<f64>::zeros(0, 0),
        &nalgebra::DMatrix::<i32>::zeros(0, 0),
        boolean_type,
        &mut result.0,
        &mut result.1,
    );

    *a = result;
}

/// MeshBoolean.cpp:100-105 `void self_union(TriangleMesh& mesh)`
pub fn self_union(igl: &dyn IglMeshBoolean, mesh: &mut TriangleMesh) {
    let mut e_m = triangle_mesh_to_eigen(mesh);
    self_union_eigen(igl, &mut e_m);
    *mesh = eigen_to_triangle_mesh(&e_m);
}

// =============================================================================================
// MeshBoolean.cpp:107-484 — namespace cgal
// =============================================================================================

pub mod cgal {
    use super::{indexed_triangle_set, MeshBooleanError, TriangleMesh, Vec3f, Vec3i};
    use crate::texture_to_color::cgal_utils::{self, Point_3, SM_Face_index, SM_Halfedge_index, SM_Vertex_index};

    /// MeshBoolean.cpp:114 `using _EpicMesh = CGAL::Surface_mesh<EpicKernel::Point_3>`
    pub type EpicMesh = cgal_utils::CGALMesh;

    /// MeshBoolean.cpp:117-121 `struct CGALMesh { _EpicMesh m; }`
    #[derive(Debug, Clone, Default)]
    pub struct CGALMesh {
        pub m: EpicMesh,
    }

    impl CGALMesh {
        /// MeshBoolean.cpp:120 `CGALMesh(const _EpicMesh& _m) :m(_m) {}`
        pub fn from_epic(m: &EpicMesh) -> Self {
            CGALMesh { m: m.clone() }
        }
    }

    /// MeshBoolean.hpp:39 `using CGALMeshPtr = std::unique_ptr<CGALMesh, CGALMeshDeleter>`
    /// (`CGALMeshDeleter::operator()` (MeshBoolean.cpp:467) is the drop of the box).
    pub type CGALMeshPtr = Box<CGALMesh>;

    /// The CGAL calls of MeshBoolean.cpp's namespace `cgal` (CGAL/Polygon_mesh_processing,
    /// CGAL/mesh_segmentation.h, CGAL/boost/graph). An `Err` is the exception CGAL throws, with its
    /// text. FIDELITY-NOTE: implemented nowhere in the crate (see the top of the file).
    pub trait CgalMeshBooleanKernel {
        /// `CGALProc::orient_polygon_soup(points, indices)`
        fn orient_polygon_soup(&self, points: &mut Vec<Point_3>, polygons: &mut Vec<[usize; 3]>) -> bool;
        /// `CGALProc::polygon_soup_to_polygon_mesh(points, indices, out)`
        fn polygon_soup_to_polygon_mesh(&self, points: &[Point_3], polygons: &[[usize; 3]], out: &mut EpicMesh);
        /// `CGALProc::orient_to_bound_a_volume(out)`
        fn orient_to_bound_a_volume(&self, mesh: &mut EpicMesh);
        /// `CGALProc::corefine_and_compute_difference(A, B, R, p, p)` with
        /// `p = throw_on_self_intersection(true)`
        fn corefine_and_compute_difference(&self, a: &mut EpicMesh, b: &mut EpicMesh, r: &mut EpicMesh) -> Result<bool, String>;
        /// `CGALProc::corefine_and_compute_union(A, B, R, p, p)`
        fn corefine_and_compute_union(&self, a: &mut EpicMesh, b: &mut EpicMesh, r: &mut EpicMesh) -> Result<bool, String>;
        /// `CGALProc::corefine_and_compute_intersection(A, B, R, p, p)`
        fn corefine_and_compute_intersection(&self, a: &mut EpicMesh, b: &mut EpicMesh, r: &mut EpicMesh) -> Result<bool, String>;
        /// `CGALProc::does_self_intersect(mesh)`
        fn does_self_intersect(&self, mesh: &EpicMesh) -> bool;
        /// `CGALProc::does_bound_a_volume(mesh)`
        fn does_bound_a_volume(&self, mesh: &EpicMesh) -> bool;
        /// `CGAL::sdf_values(mesh, sdf_property_map)`: the SDF value of every face, by face index.
        fn sdf_values(&self, mesh: &EpicMesh) -> Vec<f64>;
        /// `CGAL::segmentation_from_sdf_values(mesh, sdf, segment_ids, number_of_clusters,
        /// smoothing_lambda)`: the number of segments and the segment ID of every face.
        fn segmentation_from_sdf_values(
            &self,
            mesh: &EpicMesh,
            sdf: &[f64],
            segment_ids: &mut Vec<usize>,
            number_of_clusters: usize,
            smoothing_lambda: f64,
        ) -> usize;
        /// `CGAL::Face_filtered_graph(mesh).set_selected_faces(id, segment_ids)` then
        /// `CGAL::copy_face_graph(segment_mesh, out)`: the faces of one segment as a mesh.
        fn copy_selected_faces(&self, mesh: &EpicMesh, segment_ids: &[usize], id: usize, out: &mut EpicMesh);
        /// `CGALProc::triangulate_hole(out, h, std::back_inserter(patch_facets))`
        fn triangulate_hole(&self, mesh: &mut EpicMesh, border_halfedge: SM_Halfedge_index, patch_faces: &mut Vec<SM_Face_index>);
        /// `Surface_mesh::join(other)`
        fn join(&self, mesh: &mut EpicMesh, other: &EpicMesh);
        /// `CGALProc::stitch_borders(mesh)`
        fn stitch_borders(&self, mesh: &mut EpicMesh);
    }

    /// MeshBoolean.cpp:123-127 `save_CGALMesh(fname, cgal_mesh)`: `os << cgal_mesh.m`, the OFF
    /// writer of Surface_mesh ("OFF", the counts, the points at the stream's default precision,
    /// the faces). FIDELITY-NOTE: the text follows CGAL's `write_OFF` as documented (no colors,
    /// normals or texture coordinates); a failure to open the file is ignored as in the C++.
    pub fn save_cgal_mesh(fname: &str, cgal_mesh: &CGALMesh) {
        use std::fmt::Write as _;
        let m = &cgal_mesh.m;
        let mut os = String::new();
        let _ = writeln!(os, "OFF");
        let _ = writeln!(os, "{} {} 0", m.number_of_vertices(), m.number_of_faces());
        for v in m.vertices() {
            let p = m.point(v);
            let _ = writeln!(os, "{} {} {}", crate::config::ostream_double(p[0]), crate::config::ostream_double(p[1]), crate::config::ostream_double(p[2]));
        }
        for f in m.faces() {
            let vs = m.vertices_around_face(m.halfedge_f(f));
            let _ = write!(os, "{}", vs.len());
            for v in vs {
                let _ = write!(os, " {}", v.0);
            }
            let _ = writeln!(os);
        }
        let _ = std::fs::write(fname, os);
    }

    // /////////////////////////////////////////////////////////////////////////////
    // Converions from and to CGAL mesh
    // /////////////////////////////////////////////////////////////////////////////

    /// MeshBoolean.cpp:132-159 `template<class _Mesh> void triangle_mesh_to_cgal(const TriangleMesh&
    /// M, _Mesh& out)`: the soup oriented, made a mesh, oriented to bound a volume; an open mesh
    /// throws.
    pub fn triangle_mesh_to_cgal_oriented(
        kernel: &dyn CgalMeshBooleanKernel,
        m: &TriangleMesh,
        out: &mut EpicMesh,
    ) -> Result<(), MeshBooleanError> {
        if m.empty() {
            return Ok(());
        }

        let mut points: Vec<Point_3> = Vec::with_capacity(m.its.vertices.len());
        let mut indices: Vec<[usize; 3]> = Vec::with_capacity(m.its.indices.len());
        for v in m.its.vertices.iter() {
            points.push([v.x as f64, v.y as f64, v.z as f64]);
        }
        for f in m.its.indices.iter() {
            // `_f.cast<size_t>()`
            indices.push([f[0] as usize, f[1] as usize, f[2] as usize]);
        }

        kernel.orient_polygon_soup(&mut points, &mut indices);
        kernel.polygon_soup_to_polygon_mesh(&points, &indices, out);

        // Number the faces because 'orient_to_bound_a_volume' needs a face <--> index map
        // (`for (auto face : out.faces()) face = CGAL::SM_Face_index(index++);` assigns to a copy
        // of each index: the faces of a Surface_mesh are numbered by their index already.)

        if cgal_utils::is_closed(out) {
            kernel.orient_to_bound_a_volume(out);
            Ok(())
        } else {
            Err(MeshBooleanError("Mesh not watertight".to_string()))
        }
    }

    /// MeshBoolean.cpp:161-179 `template<class _Mesh> void triangle_mesh_to_cgal(const
    /// std::vector<stl_vertex>& V, const std::vector<stl_triangle_vertex_indices>& F, _Mesh& out)`
    pub fn triangle_mesh_to_cgal_into(v: &[Vec3f], f: &[Vec3i], out: &mut EpicMesh) {
        if f.is_empty() {
            return;
        }

        // `out.reserve(vertices_count, edges_count, faces_count)`: capacity only.

        for p in v.iter() {
            out.add_vertex([p.x as f64, p.y as f64, p.z as f64]);
        }

        for t in f.iter() {
            out.add_face(SM_Vertex_index(t[0] as u32), SM_Vertex_index(t[1] as u32), SM_Vertex_index(t[2] as u32));
        }
    }

    /// MeshBoolean.cpp:181-184 `inline Vec3f to_vec3f(const _EpicMesh::Point& v)`
    ///
    /// The overload of the exact kernel's point (MeshBoolean.cpp:186-191) converts with
    /// `CGAL::Cartesian_converter` first; the crate has no exact-kernel mesh, and nothing in the
    /// file instantiates the conversions with one.
    pub fn to_vec3f(v: &Point_3) -> Vec3f {
        Vec3f::new(v[0] as f32, v[1] as f32, v[2] as f32)
    }

    /// MeshBoolean.cpp:193-225 `template<class _Mesh> indexed_triangle_set
    /// cgal_to_indexed_triangle_set(const _Mesh &cgalmesh)`
    pub fn cgal_to_indexed_triangle_set_epic(cgalmesh: &EpicMesh) -> indexed_triangle_set {
        let mut its = indexed_triangle_set::default();
        its.vertices.reserve(cgalmesh.num_vertices());
        its.indices.reserve(cgalmesh.num_faces());

        let vsize = cgalmesh.number_of_vertices() as i32;

        for vi in cgalmesh.vertices() {
            let v = cgalmesh.point(vi); // Don't ask...
            its.vertices.push(to_vec3f(&v));
        }

        for face in cgalmesh.faces() {
            let vtc = cgalmesh.vertices_around_face(cgalmesh.halfedge_f(face));

            let mut i = 0usize;
            let mut facet = Vec3i::zeros();
            for v in vtc {
                let iv = v.0 as i32;
                if i > 2 || iv < 0 || iv >= vsize {
                    i = 0;
                    break;
                }
                facet[i] = iv;
                i += 1;
            }

            if i == 3 {
                its.indices.push(facet);
            }
        }

        its
    }

    /// MeshBoolean.cpp:227-231 `template<class _Mesh> TriangleMesh cgal_to_triangle_mesh(const _Mesh &cgalmesh)`
    pub fn cgal_to_triangle_mesh_epic(cgalmesh: &EpicMesh) -> TriangleMesh {
        let its = cgal_to_indexed_triangle_set_epic(cgalmesh);
        TriangleMesh::from_its(its)
    }

    /// MeshBoolean.cpp:233-240 `CGALMeshPtr triangle_mesh_to_cgal(const std::vector<stl_vertex> &V,
    /// const std::vector<stl_triangle_vertex_indices> &F)`
    pub fn triangle_mesh_to_cgal(v: &[Vec3f], f: &[Vec3i]) -> CGALMeshPtr {
        let mut out: CGALMeshPtr = Box::default();
        triangle_mesh_to_cgal_into(v, f, &mut out.m);
        out
    }

    /// MeshBoolean.hpp:49-52 `inline CGALMeshPtr triangle_mesh_to_cgal(const indexed_triangle_set &M)`
    pub fn triangle_mesh_to_cgal_its(m: &indexed_triangle_set) -> CGALMeshPtr {
        triangle_mesh_to_cgal(&m.vertices, &m.indices)
    }

    /// MeshBoolean.hpp:53-56 `inline CGALMeshPtr triangle_mesh_to_cgal(const TriangleMesh &M)`
    pub fn triangle_mesh_to_cgal_mesh(m: &TriangleMesh) -> CGALMeshPtr {
        triangle_mesh_to_cgal_its(&m.its)
    }

    /// MeshBoolean.cpp:242-245 `TriangleMesh cgal_to_triangle_mesh(const CGALMesh &cgalmesh)`
    pub fn cgal_to_triangle_mesh(cgalmesh: &CGALMesh) -> TriangleMesh {
        TriangleMesh::from_its(cgal_to_indexed_triangle_set_epic(&cgalmesh.m))
    }

    /// MeshBoolean.cpp:247-250 `indexed_triangle_set cgal_to_indexed_triangle_set(const CGALMesh &cgalmesh)`
    pub fn cgal_to_indexed_triangle_set(cgalmesh: &CGALMesh) -> indexed_triangle_set {
        cgal_to_indexed_triangle_set_epic(&cgalmesh.m)
    }

    // /////////////////////////////////////////////////////////////////////////////
    // Boolean operations for CGAL meshes
    // /////////////////////////////////////////////////////////////////////////////

    /// The three corefinement booleans, `Op` of `_cgal_do`.
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub enum CgalOp {
        Diff,
        Union,
        Intersection,
    }

    /// MeshBoolean.cpp:256-260 `static bool _cgal_diff(CGALMesh &A, CGALMesh &B, CGALMesh &R)`
    fn cgal_diff(kernel: &dyn CgalMeshBooleanKernel, a: &mut CGALMesh, b: &mut CGALMesh, r: &mut CGALMesh) -> Result<bool, String> {
        kernel.corefine_and_compute_difference(&mut a.m, &mut b.m, &mut r.m)
    }

    /// MeshBoolean.cpp:262-266 `static bool _cgal_union(CGALMesh &A, CGALMesh &B, CGALMesh &R)`
    fn cgal_union(kernel: &dyn CgalMeshBooleanKernel, a: &mut CGALMesh, b: &mut CGALMesh, r: &mut CGALMesh) -> Result<bool, String> {
        kernel.corefine_and_compute_union(&mut a.m, &mut b.m, &mut r.m)
    }

    /// MeshBoolean.cpp:268-272 `static bool _cgal_intersection(CGALMesh &A, CGALMesh &B, CGALMesh &R)`
    fn cgal_intersection(kernel: &dyn CgalMeshBooleanKernel, a: &mut CGALMesh, b: &mut CGALMesh, r: &mut CGALMesh) -> Result<bool, String> {
        kernel.corefine_and_compute_intersection(&mut a.m, &mut b.m, &mut r.m)
    }

    /// MeshBoolean.cpp:274-294 `template<class Op> void _cgal_do(Op &&op, CGALMesh &A, CGALMesh &B)`:
    /// the result replaces `A` unless the operation throws; a failed or throwing operation is
    /// `RuntimeError("CGAL mesh boolean operation failed.")`.
    ///
    /// The C++ also runs the operation under `try_catch_signal({SIGSEGV, SIGFPE})` and throws
    /// `HardCrash` when it crashes; a Rust kernel does not raise signals.
    pub fn cgal_do(kernel: &dyn CgalMeshBooleanKernel, op: CgalOp, a: &mut CGALMesh, b: &mut CGALMesh) -> Result<(), MeshBooleanError> {
        let success;
        {
            let mut result = CGALMesh::default();
            let r = match op {
                CgalOp::Diff => cgal_diff(kernel, a, b, &mut result),
                CgalOp::Union => cgal_union(kernel, a, b, &mut result),
                CgalOp::Intersection => cgal_intersection(kernel, a, b, &mut result),
            };
            match r {
                Ok(s) => {
                    success = s;
                    *a = result; // In-place operation does not work
                }
                Err(e) => {
                    log::warn!("{}", e);
                    success = false;
                }
            }
        }

        if !success {
            return Err(MeshBooleanError("CGAL mesh boolean operation failed.".to_string()));
        }
        Ok(())
    }

    /// MeshBoolean.cpp:296 `void minus(CGALMesh &A, CGALMesh &B)`
    pub fn minus_cgal(kernel: &dyn CgalMeshBooleanKernel, a: &mut CGALMesh, b: &mut CGALMesh) -> Result<(), MeshBooleanError> {
        cgal_do(kernel, CgalOp::Diff, a, b)
    }
    /// MeshBoolean.cpp:297 `void plus(CGALMesh &A, CGALMesh &B)`
    pub fn plus_cgal(kernel: &dyn CgalMeshBooleanKernel, a: &mut CGALMesh, b: &mut CGALMesh) -> Result<(), MeshBooleanError> {
        cgal_do(kernel, CgalOp::Union, a, b)
    }
    /// MeshBoolean.cpp:298 `void intersect(CGALMesh &A, CGALMesh &B)`
    pub fn intersect_cgal(kernel: &dyn CgalMeshBooleanKernel, a: &mut CGALMesh, b: &mut CGALMesh) -> Result<(), MeshBooleanError> {
        cgal_do(kernel, CgalOp::Intersection, a, b)
    }
    /// MeshBoolean.cpp:299 `bool does_self_intersect(const CGALMesh &mesh)`
    pub fn does_self_intersect_cgal(kernel: &dyn CgalMeshBooleanKernel, mesh: &CGALMesh) -> bool {
        kernel.does_self_intersect(&mesh.m)
    }

    /// MeshBoolean.cpp:300-363 `void segment(CGALMesh& src, std::vector<CGALMesh>& dst, double
    /// smoothing_alpha, int segment_number)`: the SDF segmentation of the mesh, each segment copied
    /// out with its holes filled.
    pub fn segment_cgal(kernel: &dyn CgalMeshBooleanKernel, src: &CGALMesh, dst: &mut Vec<CGALMesh>, smoothing_alpha: f64, segment_number: i32) {
        let mesh: EpicMesh = src.m.clone();

        let sdf_property_map = kernel.sdf_values(&mesh);

        // create a property-map for segment-ids
        let mut segment_property_map: Vec<usize> = vec![0; mesh.num_faces()];
        // segment the mesh using default parameters for number of levels, and smoothing lambda
        // Any other scalar values can be used instead of using SDF values computed using the CGAL function
        let number_of_segments =
            kernel.segmentation_from_sdf_values(&mesh, &sdf_property_map, &mut segment_property_map, segment_number as usize, smoothing_alpha);
        //print area of each segment and then put it in a Mesh and print it in an OFF file
        for id in 0..number_of_segments {
            let mut out = EpicMesh::new();
            kernel.copy_selected_faces(&mesh, &segment_property_map, id, &mut out);

            // save_CGALMesh("out.off", out);

            // fill holes
            let border_cycles: Vec<SM_Halfedge_index> = cgal_utils::extract_boundary_cycles(&out);
            for h in border_cycles {
                let mut patch_facets: Vec<SM_Face_index> = Vec::new();
                kernel.triangulate_hole(&mut out, h, &mut patch_facets);
            }

            //if (id > 2) {
            //    mesh_merged.join(out);
            //}
            //else
            {
                dst.push(CGALMesh::from_epic(&out));
            }
        }
    }

    /// MeshBoolean.cpp:365-379 `std::vector<TriangleMesh> segment(const TriangleMesh& src, double
    /// smoothing_alpha, int segment_number)` (defaults 0.5 and 5).
    pub fn segment(
        kernel: &dyn CgalMeshBooleanKernel,
        src: &TriangleMesh,
        smoothing_alpha: f64,
        segment_number: i32,
    ) -> Result<Vec<TriangleMesh>, MeshBooleanError> {
        let mut in_cgal_mesh = CGALMesh::default();
        triangle_mesh_to_cgal_oriented(kernel, src, &mut in_cgal_mesh.m)?;
        let mut out_cgal_meshes: Vec<CGALMesh> = Vec::new();
        segment_cgal(kernel, &in_cgal_mesh, &mut out_cgal_meshes, smoothing_alpha, segment_number);

        let mut out_meshes: Vec<TriangleMesh> = Vec::new();
        for outf_cgal_mesh in out_cgal_meshes.iter() {
            out_meshes.push(cgal_to_triangle_mesh_epic(&outf_cgal_mesh.m));
        }

        Ok(out_meshes)
    }

    /// MeshBoolean.cpp:381-392 `void merge(std::vector<_EpicMesh>& srcs, _EpicMesh& dst)`
    pub fn merge_epic(kernel: &dyn CgalMeshBooleanKernel, srcs: &[EpicMesh], dst: &mut EpicMesh) {
        let mut mesh_merged = EpicMesh::new();
        for src in srcs.iter() {
            kernel.join(&mut mesh_merged, src);
        }
        // `is_empty()`: no vertices
        if mesh_merged.number_of_vertices() != 0 {
            kernel.stitch_borders(&mut mesh_merged);
            *dst = mesh_merged;
        }
    }

    /// MeshBoolean.cpp:394-404 `TriangleMesh merge(std::vector<TriangleMesh> meshes)`
    pub fn merge(kernel: &dyn CgalMeshBooleanKernel, meshes: &[TriangleMesh]) -> Result<TriangleMesh, MeshBooleanError> {
        let mut srcs: Vec<EpicMesh> = vec![EpicMesh::new(); meshes.len()];
        for i in 0..meshes.len() {
            triangle_mesh_to_cgal_oriented(kernel, &meshes[i], &mut srcs[i])?;
        }
        let mut dst = EpicMesh::new();
        merge_epic(kernel, &srcs, &mut dst);
        Ok(cgal_to_triangle_mesh_epic(&dst))
    }

    /// MeshBoolean.cpp:406-416 `template<class Op> void _mesh_boolean_do(Op &&op,
    /// indexed_triangle_set &A, const indexed_triangle_set &B)`
    pub fn mesh_boolean_do_its(
        kernel: &dyn CgalMeshBooleanKernel,
        op: CgalOp,
        a: &mut indexed_triangle_set,
        b: &indexed_triangle_set,
    ) -> Result<(), MeshBooleanError> {
        let mut mesh_a = CGALMesh::default();
        let mut mesh_b = CGALMesh::default();
        triangle_mesh_to_cgal_into(&a.vertices, &a.indices, &mut mesh_a.m);
        triangle_mesh_to_cgal_into(&b.vertices, &b.indices, &mut mesh_b.m);

        cgal_do(kernel, op, &mut mesh_a, &mut mesh_b)?;

        *a = cgal_to_indexed_triangle_set_epic(&mesh_a.m);
        Ok(())
    }

    /// MeshBoolean.cpp:418-428 `template<class Op> void _mesh_boolean_do(Op &&op, TriangleMesh &A,
    /// const TriangleMesh &B)`
    pub fn mesh_boolean_do(kernel: &dyn CgalMeshBooleanKernel, op: CgalOp, a: &mut TriangleMesh, b: &TriangleMesh) -> Result<(), MeshBooleanError> {
        let mut mesh_a = CGALMesh::default();
        let mut mesh_b = CGALMesh::default();
        triangle_mesh_to_cgal_into(&a.its.vertices, &a.its.indices, &mut mesh_a.m);
        triangle_mesh_to_cgal_into(&b.its.vertices, &b.its.indices, &mut mesh_b.m);

        cgal_do(kernel, op, &mut mesh_a, &mut mesh_b)?;

        *a = cgal_to_triangle_mesh(&mesh_a);
        Ok(())
    }

    /// MeshBoolean.cpp:430-433 `void minus(TriangleMesh &A, const TriangleMesh &B)`
    pub fn minus(kernel: &dyn CgalMeshBooleanKernel, a: &mut TriangleMesh, b: &TriangleMesh) -> Result<(), MeshBooleanError> {
        mesh_boolean_do(kernel, CgalOp::Diff, a, b)
    }

    /// MeshBoolean.cpp:435-438 `void plus(TriangleMesh &A, const TriangleMesh &B)`
    pub fn plus(kernel: &dyn CgalMeshBooleanKernel, a: &mut TriangleMesh, b: &TriangleMesh) -> Result<(), MeshBooleanError> {
        mesh_boolean_do(kernel, CgalOp::Union, a, b)
    }

    /// MeshBoolean.cpp:440-443 `void intersect(TriangleMesh &A, const TriangleMesh &B)`
    pub fn intersect(kernel: &dyn CgalMeshBooleanKernel, a: &mut TriangleMesh, b: &TriangleMesh) -> Result<(), MeshBooleanError> {
        mesh_boolean_do(kernel, CgalOp::Intersection, a, b)
    }

    /// MeshBoolean.cpp:445-448 `void minus(indexed_triangle_set &A, const indexed_triangle_set &B)`
    pub fn minus_its(kernel: &dyn CgalMeshBooleanKernel, a: &mut indexed_triangle_set, b: &indexed_triangle_set) -> Result<(), MeshBooleanError> {
        mesh_boolean_do_its(kernel, CgalOp::Diff, a, b)
    }

    /// MeshBoolean.cpp:450-453 `void plus(indexed_triangle_set &A, const indexed_triangle_set &B)`
    pub fn plus_its(kernel: &dyn CgalMeshBooleanKernel, a: &mut indexed_triangle_set, b: &indexed_triangle_set) -> Result<(), MeshBooleanError> {
        mesh_boolean_do_its(kernel, CgalOp::Union, a, b)
    }

    /// MeshBoolean.cpp:455-458 `void intersect(indexed_triangle_set &A, const indexed_triangle_set &B)`
    pub fn intersect_its(kernel: &dyn CgalMeshBooleanKernel, a: &mut indexed_triangle_set, b: &indexed_triangle_set) -> Result<(), MeshBooleanError> {
        mesh_boolean_do_its(kernel, CgalOp::Intersection, a, b)
    }

    /// MeshBoolean.cpp:460-465 `bool does_self_intersect(const TriangleMesh &mesh)`
    pub fn does_self_intersect(kernel: &dyn CgalMeshBooleanKernel, mesh: &TriangleMesh) -> bool {
        let mut cgalm = CGALMesh::default();
        triangle_mesh_to_cgal_into(&mesh.its.vertices, &mesh.its.indices, &mut cgalm.m);
        kernel.does_self_intersect(&cgalm.m)
    }

    /// MeshBoolean.cpp:469-472 `bool does_bound_a_volume(const CGALMesh &mesh)`
    pub fn does_bound_a_volume(kernel: &dyn CgalMeshBooleanKernel, mesh: &CGALMesh) -> bool {
        cgal_utils::is_closed(&mesh.m) && kernel.does_bound_a_volume(&mesh.m)
    }

    /// MeshBoolean.cpp:474-477 `bool empty(const CGALMesh &mesh)`: `Surface_mesh::is_empty()`, no
    /// vertices.
    pub fn empty(mesh: &CGALMesh) -> bool {
        mesh.m.number_of_vertices() == 0
    }

    /// MeshBoolean.cpp:479-482 `CGALMeshPtr clone(const CGALMesh &m)`
    pub fn clone(m: &CGALMesh) -> CGALMeshPtr {
        Box::new(CGALMesh::from_epic(&m.m))
    }
}

// =============================================================================================
// MeshBoolean.cpp:486-512 — namespace openmeshcraft (SLIC3R_ENABLE_OPENMESHCRAFT_BOOLEAN off)
// =============================================================================================

pub mod openmeshcraft {
    use super::mcut::{BooleanCancelCB, BooleanFailedCB, BooleanProgressCB};
    use super::TriangleMesh;

    /// MeshBoolean.cpp:488-495 `bool is_enabled()`: slicer_cli is built without
    /// `SLIC3R_ENABLE_OPENMESHCRAFT_BOOLEAN`.
    pub fn is_enabled() -> bool {
        false
    }

    /// MeshBoolean.cpp:497-510 `make_boolean(...)` of a build without OpenMeshCraft: reports the
    /// failure.
    pub fn make_boolean(
        _src_mesh: &TriangleMesh,
        _cut_mesh: &TriangleMesh,
        _dst_mesh: &mut Vec<TriangleMesh>,
        _boolean_opts: &str,
        _cancel_cb: BooleanCancelCB,
        _progress_cb: BooleanProgressCB,
        failed_cb: BooleanFailedCB,
    ) {
        if let Some(failed_cb) = failed_cb {
            failed_cb();
        }
        log::error!("OpenMeshCraft mesh boolean backend is not enabled in this build.");
    }
}

// =============================================================================================
// MeshBoolean.cpp:515-990 — namespace mcut
// =============================================================================================

pub mod mcut {
    use super::{indexed_triangle_set, TriangleMesh, Vec3f, Vec3i};
    use crate::geometry::geometry::{Transform3d, Vec3d};
    use std::cell::Cell;

    /// MeshBoolean.cpp:516-525
    /// /* BBS: MusangKing
    ///  * mcut mesh array format for Boolean Opts calculation
    ///  */
    #[derive(Debug, Clone, Default, PartialEq)]
    pub struct McutMesh {
        // variables for mesh data in a format suited for mcut
        pub face_sizes_array: Vec<u32>,
        pub face_indices_array: Vec<u32>,
        pub vertex_coords_array: Vec<f64>,
    }

    /// MeshBoolean.hpp:91 `using McutMeshPtr = std::unique_ptr<McutMesh, McutMeshDeleter>`
    /// (`McutMeshDeleter::operator()` (MeshBoolean.cpp:526) is the drop of the box).
    pub type McutMeshPtr = Box<McutMesh>;

    /// MeshBoolean.hpp:97 `using BooleanCancelCB = std::function<bool()>`; `None` is an empty function.
    pub type BooleanCancelCB<'a> = Option<&'a dyn Fn() -> bool>;
    /// MeshBoolean.hpp:98 `using BooleanProgressCB = std::function<void(float)>`
    pub type BooleanProgressCB<'a> = Option<&'a dyn Fn(f32)>;
    /// MeshBoolean.hpp:99 `using BooleanFailedCB = std::function<void()>`
    pub type BooleanFailedCB<'a> = Option<&'a dyn Fn()>;

    // mcut/include/mcut/mcut.h: the values MeshBoolean.cpp uses.
    /// `McResult`
    pub type McResult = i32;
    pub const MC_NO_ERROR: McResult = 0;
    /// `McFlags`
    pub type McFlags = u32;
    pub const MC_DISPATCH_VERTEX_ARRAY_DOUBLE: McFlags = 1 << 1;
    pub const MC_DISPATCH_FILTER_FRAGMENT_LOCATION_ABOVE: McFlags = 1 << 5;
    pub const MC_DISPATCH_FILTER_FRAGMENT_LOCATION_BELOW: McFlags = 1 << 6;
    pub const MC_DISPATCH_FILTER_FRAGMENT_SEALING_INSIDE: McFlags = 1 << 8;
    pub const MC_DISPATCH_FILTER_FRAGMENT_SEALING_OUTSIDE: McFlags = 1 << 9;
    pub const MC_DISPATCH_ENFORCE_GENERAL_POSITION: McFlags = 1 << 15;
    /// `McConnectedComponentType`
    pub const MC_CONNECTED_COMPONENT_TYPE_FRAGMENT: u32 = 1 << 0;
    /// `McFragmentLocation`
    pub type McFragmentLocation = u32;
    pub const MC_FRAGMENT_LOCATION_ABOVE: McFragmentLocation = 1 << 0;
    pub const MC_FRAGMENT_LOCATION_BELOW: McFragmentLocation = 1 << 1;
    pub const MC_FRAGMENT_LOCATION_UNDEFINED: McFragmentLocation = 1 << 2;
    /// `McPatchLocation`
    pub type McPatchLocation = u32;
    pub const MC_PATCH_LOCATION_INSIDE: McPatchLocation = 1 << 0;
    pub const MC_PATCH_LOCATION_OUTSIDE: McPatchLocation = 1 << 1;
    pub const MC_PATCH_LOCATION_UNDEFINED: McPatchLocation = 1 << 2;
    /// `McDebugSource`
    pub type McDebugSource = u32;
    pub const MC_DEBUG_SOURCE_API: McDebugSource = 1 << 0;
    pub const MC_DEBUG_SOURCE_KERNEL: McDebugSource = 1 << 1;
    pub const MC_DEBUG_SOURCE_ALL: McDebugSource = 0xFFFF_FFFF;
    /// `McDebugType`
    pub type McDebugType = u32;
    pub const MC_DEBUG_TYPE_ERROR: McDebugType = 1 << 0;
    pub const MC_DEBUG_TYPE_DEPRECATED_BEHAVIOR: McDebugType = 1 << 1;
    pub const MC_DEBUG_TYPE_OTHER: McDebugType = 1 << 2;
    /// `McDebugSeverity`
    pub type McDebugSeverity = u32;
    pub const MC_DEBUG_SEVERITY_HIGH: McDebugSeverity = 1 << 0;
    pub const MC_DEBUG_SEVERITY_MEDIUM: McDebugSeverity = 1 << 1;
    pub const MC_DEBUG_SEVERITY_LOW: McDebugSeverity = 1 << 2;
    pub const MC_DEBUG_SEVERITY_NOTIFICATION: McDebugSeverity = 1 << 3;

    /// `McContext` / `McConnectedComponent`: the library's handles.
    pub type McContext = usize;
    pub type McConnectedComponent = usize;

    /// The mcut C API calls of `do_boolean_single`, each returning its `McResult` (and its data
    /// where the C++ passes an output buffer). FIDELITY-NOTE: implemented nowhere in the crate (see
    /// the top of the file).
    pub trait McutApi {
        /// `mcCreateContext(&context, 0)`
        fn mc_create_context(&self, context: &mut McContext) -> McResult;
        /// `mcDebugMessageCallback(context, mcDebugOutput, nullptr)` and
        /// `mcDebugMessageControl(context, source, type, severity, enabled)`
        fn mc_debug_message_control(&self, context: McContext, source: McDebugSource, type_: McDebugType, severity: McDebugSeverity, enabled: bool);
        /// `mcDispatch(context, flags, src vertices, faces, face sizes, counts, cut ...)`
        fn mc_dispatch(&self, context: McContext, dispatch_flags: McFlags, src_mesh: &McutMesh, cut_mesh: &McutMesh) -> McResult;
        /// `mcGetConnectedComponents(context, type, ...)`: the connected components of the type.
        fn mc_get_connected_components(&self, context: McContext, type_: u32, connected_components: &mut Vec<McConnectedComponent>) -> McResult;
        /// `mcGetConnectedComponentData(..., MC_CONNECTED_COMPONENT_DATA_VERTEX_DOUBLE, ...)`
        fn mc_get_connected_component_vertices(&self, context: McContext, connected_component: McConnectedComponent, vertices: &mut Vec<f64>) -> McResult;
        /// `mcGetConnectedComponentData(..., MC_CONNECTED_COMPONENT_DATA_FACE_TRIANGULATION, ...)`
        fn mc_get_connected_component_face_triangulation(&self, context: McContext, connected_component: McConnectedComponent, face_indices: &mut Vec<u32>) -> McResult;
        /// `mcGetConnectedComponentData(..., MC_CONNECTED_COMPONENT_DATA_PATCH_LOCATION, ...)`
        fn mc_get_connected_component_patch_location(&self, context: McContext, connected_component: McConnectedComponent, patch_location: &mut McPatchLocation) -> McResult;
        /// `mcGetConnectedComponentData(..., MC_CONNECTED_COMPONENT_DATA_FRAGMENT_LOCATION, ...)`
        fn mc_get_connected_component_fragment_location(&self, context: McContext, connected_component: McConnectedComponent, fragment_location: &mut McFragmentLocation) -> McResult;
        /// `mcReleaseConnectedComponents(context, 0, NULL)`
        fn mc_release_connected_components(&self, context: McContext) -> McResult;
        /// `mcReleaseContext(context)`
        fn mc_release_context(&self, context: McContext) -> McResult;
    }

    /// MeshBoolean.cpp:528 `bool empty(const McutMesh &mesh)`
    pub fn empty(mesh: &McutMesh) -> bool {
        mesh.vertex_coords_array.is_empty() || mesh.face_indices_array.is_empty()
    }

    /// MeshBoolean.cpp:529-553 `void triangle_mesh_to_mcut(const TriangleMesh &src_mesh, McutMesh
    /// &srcMesh, const Transform3d &src_nm = Transform3d::Identity())`
    pub fn triangle_mesh_to_mcut(src_mesh: &TriangleMesh, src_mesh_out: &mut McutMesh, src_nm: &Transform3d) {
        // vertices precision convention and copy
        src_mesh_out.vertex_coords_array.reserve(src_mesh.its.vertices.len() * 3);
        for i in 0..src_mesh.its.vertices.len() {
            let p = &src_mesh.its.vertices[i];
            let v: Vec3d = crate::geometry::geometry::eigen::transform_mul_vec(src_nm, &Vec3d::new(p.x as f64, p.y as f64, p.z as f64));
            src_mesh_out.vertex_coords_array.push(v[0]);
            src_mesh_out.vertex_coords_array.push(v[1]);
            src_mesh_out.vertex_coords_array.push(v[2]);
        }

        // faces copy
        src_mesh_out.face_indices_array.reserve(src_mesh.its.indices.len() * 3);
        src_mesh_out.face_sizes_array.reserve(src_mesh.its.indices.len());
        for i in 0..src_mesh.its.indices.len() {
            let f0 = src_mesh.its.indices[i][0];
            let f1 = src_mesh.its.indices[i][1];
            let f2 = src_mesh.its.indices[i][2];
            src_mesh_out.face_indices_array.push(f0 as u32);
            src_mesh_out.face_indices_array.push(f1 as u32);
            src_mesh_out.face_indices_array.push(f2 as u32);

            src_mesh_out.face_sizes_array.push(3u32);
        }
    }

    /// MeshBoolean.cpp:555-561 `McutMeshPtr triangle_mesh_to_mcut(const indexed_triangle_set &M)`
    pub fn triangle_mesh_to_mcut_its(m: &indexed_triangle_set) -> McutMeshPtr {
        let mut out: McutMeshPtr = Box::default();
        let trimesh = TriangleMesh::from_its(m.clone());
        triangle_mesh_to_mcut(&trimesh, &mut out, &Transform3d::identity());
        out
    }

    /// The crate's f64 `triangle_mesh::TriangleMesh` (csg_mesh's parts) through
    /// [`triangle_mesh_to_mcut_its`], its vertices back to float.
    pub fn triangle_mesh_to_mcut_from_its(m: &crate::triangle_mesh::TriangleMesh) -> McutMesh {
        *triangle_mesh_to_mcut_its(&TriangleMesh::from_triangle_mesh(m).its)
    }

    /// MeshBoolean.cpp:563-593 `TriangleMesh mcut_to_triangle_mesh(const McutMesh &mcutmesh)`
    pub fn mcut_to_triangle_mesh(mcutmesh: &McutMesh) -> TriangleMesh {
        let cc_vertex_count: u32 = (mcutmesh.vertex_coords_array.len() / 3) as u32;
        let cc_vertices = &mcutmesh.vertex_coords_array;
        let cc_face_indices = &mcutmesh.face_indices_array;
        let face_sizes = &mcutmesh.face_sizes_array;
        let cc_face_count: u32 = face_sizes.len() as u32;
        // rearrange vertices/faces and save into result mesh
        let mut vertices: Vec<Vec3f> = vec![Vec3f::zeros(); cc_vertex_count as usize];
        for i in 0..cc_vertex_count {
            vertices[i as usize][0] = cc_vertices[(i as u64 * 3 + 0) as usize] as f32;
            vertices[i as usize][1] = cc_vertices[(i as u64 * 3 + 1) as usize] as f32;
            vertices[i as usize][2] = cc_vertices[(i as u64 * 3 + 2) as usize] as f32;
        }

        // output faces
        let mut face_vertex_offset_base: i32 = 0;

        // for each face in CC
        let mut faces: Vec<Vec3i> = vec![Vec3i::zeros(); cc_face_count as usize];
        for f in 0..cc_face_count {
            let face_size = face_sizes[f as usize] as i32;

            // for each vertex in face
            for v in 0..face_size {
                faces[f as usize][v as usize] = cc_face_indices[(face_vertex_offset_base as u64 + v as u64) as usize] as i32;
            }
            face_vertex_offset_base += face_size;
        }

        // TriangleMesh.cpp:79-82 `TriangleMesh(const std::vector<Vec3f>&, const std::vector<Vec3i>&)`
        let mut its = indexed_triangle_set::default();
        its.vertices = vertices;
        its.indices = faces;
        TriangleMesh::from_its(its)
    }

    /// MeshBoolean.cpp:595-602 `void merge_mcut_meshes(McutMesh& src, const McutMesh& cut)`
    pub fn merge_mcut_meshes(src: &mut McutMesh, cut: &McutMesh) {
        let mut all_its = indexed_triangle_set::default();
        let tri_src = mcut_to_triangle_mesh(src);
        let tri_cut = mcut_to_triangle_mesh(cut);
        crate::triangle_mesh::its_merge(&mut all_its, &tri_src.its);
        crate::triangle_mesh::its_merge(&mut all_its, &tri_cut.its);
        *src = *triangle_mesh_to_mcut_its(&all_its);
    }

    /// MeshBoolean.cpp:604-649 `mcDebugOutput(source, type, id, severity, length, message,
    /// userParam)`: the debug callback, which logs the message and its source, type and severity.
    pub fn mc_debug_output(source: McDebugSource, type_: McDebugType, id: u32, severity: McDebugSeverity, _length: usize, message: &str) {
        log::debug!("mcut mcDebugOutput message ( {} ): {} ", id, message);

        match source {
            MC_DEBUG_SOURCE_API => log::debug!("Source: API"),
            MC_DEBUG_SOURCE_KERNEL => log::debug!("Source: Kernel"),
            _ => {}
        }

        match type_ {
            MC_DEBUG_TYPE_ERROR => log::debug!("Type: Error"),
            MC_DEBUG_TYPE_DEPRECATED_BEHAVIOR => log::debug!("Type: Deprecated Behaviour"),
            MC_DEBUG_TYPE_OTHER => log::debug!("Type: Other"),
            _ => {}
        }

        match severity {
            MC_DEBUG_SEVERITY_HIGH => log::debug!("Severity: high"),
            MC_DEBUG_SEVERITY_MEDIUM => log::debug!("Severity: medium"),
            MC_DEBUG_SEVERITY_LOW => log::debug!("Severity: low"),
            MC_DEBUG_SEVERITY_NOTIFICATION => log::debug!("Severity: notification"),
            _ => {}
        }
    }

    /// MeshBoolean.cpp:670-675 `booleanOpts`: the dispatch filter flags of each operation.
    fn boolean_opts_flags(boolean_opts: &str) -> McFlags {
        match boolean_opts {
            "A_NOT_B" => MC_DISPATCH_FILTER_FRAGMENT_SEALING_INSIDE | MC_DISPATCH_FILTER_FRAGMENT_LOCATION_ABOVE,
            "B_NOT_A" => MC_DISPATCH_FILTER_FRAGMENT_SEALING_OUTSIDE | MC_DISPATCH_FILTER_FRAGMENT_LOCATION_BELOW,
            "UNION" => MC_DISPATCH_FILTER_FRAGMENT_SEALING_OUTSIDE | MC_DISPATCH_FILTER_FRAGMENT_LOCATION_ABOVE,
            "INTERSECTION" => MC_DISPATCH_FILTER_FRAGMENT_SEALING_INSIDE | MC_DISPATCH_FILTER_FRAGMENT_LOCATION_BELOW,
            // `booleanOpts.find(boolean_opts)->second` of an unknown operation dereferences the end
            // iterator in the C++ (undefined behaviour); no filter flags here.
            _ => 0,
        }
    }

    /// MeshBoolean.cpp:652-851 `bool do_boolean_single(McutMesh &srcMesh, const McutMesh &cutMesh,
    /// const std::string &boolean_opts, cancel_cb, progress_cb)`: one boolean of two connected
    /// meshes through mcut; the fragments are concatenated into `src_mesh`, the faces of the
    /// fragments below the cut mesh with outside patches reversed.
    pub fn do_boolean_single(
        api: &dyn McutApi,
        src_mesh: &mut McutMesh,
        cut_mesh: &McutMesh,
        boolean_opts: &str,
        cancel_cb: BooleanCancelCB,
        progress_cb: BooleanProgressCB,
    ) -> bool {
        // create context
        let mut context: McContext = 0;
        let mut err: McResult = api.mc_create_context(&mut context);
        let _ = err;
        // add debug callback according to https://cutdigital.github.io/mcut.site/tutorials/debugging/
        api.mc_debug_message_control(context, MC_DEBUG_SOURCE_ALL, MC_DEBUG_TYPE_ERROR, MC_DEBUG_SEVERITY_MEDIUM, true);
        // We can either let MCUT compute all possible meshes (including patches etc.), or we can
        // constrain the library to compute exactly the boolean op mesh we want. This 'constrained' case
        // is done with the following flags.
        // NOTE#1: you can extend these flags by bitwise ORing with additional flags (see `McDispatchFlags' in mcut.h)
        // NOTE#2: below order of columns MATTERS
        let bool_op_flags: McFlags = boolean_opts_flags(boolean_opts);

        if src_mesh.vertex_coords_array.is_empty() && (boolean_opts == "UNION" || boolean_opts == "B_NOT_A") {
            *src_mesh = cut_mesh.clone();
            api.mc_release_context(context);
            if let Some(progress_cb) = progress_cb {
                progress_cb(100.0);
            }
            return true;
        }

        err = api.mc_dispatch(
            context,
            MC_DISPATCH_VERTEX_ARRAY_DOUBLE |          // vertices are in array of doubles
                MC_DISPATCH_ENFORCE_GENERAL_POSITION | // perturb if necessary
                bool_op_flags,                         // filter flags which specify the type of output we want
            src_mesh,
            cut_mesh,
        );
        if let Some(progress_cb) = progress_cb {
            progress_cb(10.0);
        }
        if err != MC_NO_ERROR {
            log::debug!("MCUT mcDispatch fails! err={}", err);
            api.mc_release_context(context);
            if boolean_opts == "UNION" {
                merge_mcut_meshes(src_mesh, cut_mesh);
                if let Some(progress_cb) = progress_cb {
                    progress_cb(100.0);
                }
                return true;
            }
            return false;
        }

        // query the number of available connected component
        let mut connected_components: Vec<McConnectedComponent> = Vec::new();
        err = api.mc_get_connected_components(context, MC_CONNECTED_COMPONENT_TYPE_FRAGMENT, &mut connected_components);
        let num_conn_comps = connected_components.len() as u32;
        if let Some(progress_cb) = progress_cb {
            progress_cb(20.0);
        }
        if err != MC_NO_ERROR || num_conn_comps == 0 {
            log::debug!("MCUT mcGetConnectedComponents fails! err={}, numConnComps{}", err, num_conn_comps);
            api.mc_release_context(context);
            if num_conn_comps == 0 && boolean_opts == "UNION" {
                merge_mcut_meshes(src_mesh, cut_mesh);
                if let Some(progress_cb) = progress_cb {
                    progress_cb(100.0);
                }
                return true;
            }
            return false;
        }

        if let Some(progress_cb) = progress_cb {
            progress_cb(30.0);
        }
        let mut out_mesh = McutMesh::default();
        let mut n_vertices: i32 = 0;
        // traversal of all connected components
        for n in 0..num_conn_comps as i32 {
            if let Some(cancel_cb) = cancel_cb {
                if cancel_cb() {
                    return false;
                }
            }
            // query the data of each connected component from MCUT
            let conn_comp = connected_components[n as usize];

            // query the vertices
            let mut cc_vertices: Vec<f64> = Vec::new();
            err = api.mc_get_connected_component_vertices(context, conn_comp, &mut cc_vertices);

            // `60.0 * n / numConnComps` in double, stored as float
            let sub_progress: f32 = (60.0 * n as f64 / num_conn_comps as f64) as f32;
            let temp_progress: f32 = 30.0f32 + sub_progress;
            // `temp_progress + 60.0 / numConnComps * k`: the float promoted, the sum in double
            let step = |k: f32| -> f32 { (temp_progress as f64 + 60.0 / num_conn_comps as f64 * k as f64) as f32 };
            if let Some(progress_cb) = progress_cb {
                progress_cb(step(0.1));
            }

            let cc_vertex_count: u32 = (cc_vertices.len() / 3) as u32;

            if let Some(progress_cb) = progress_cb {
                progress_cb(step(0.2));
            }

            // query the faces
            let mut cc_face_indices: Vec<u32> = Vec::new();
            if let Some(progress_cb) = progress_cb {
                progress_cb(step(0.3));
            }
            err = api.mc_get_connected_component_face_triangulation(context, conn_comp, &mut cc_face_indices);

            if let Some(progress_cb) = progress_cb {
                progress_cb(step(0.4));
            }

            let face_sizes: Vec<u32> = vec![3; cc_face_indices.len() / 3];

            let cc_face_count: u32 = face_sizes.len() as u32;

            // Here we show, how to know when connected components, pertain particular boolean operations.
            let mut patch_location: McPatchLocation = 0;
            err = api.mc_get_connected_component_patch_location(context, conn_comp, &mut patch_location);

            if let Some(progress_cb) = progress_cb {
                progress_cb(step(0.5));
            }

            let mut fragment_location: McFragmentLocation = 0;
            err = api.mc_get_connected_component_fragment_location(context, conn_comp, &mut fragment_location);

            if let Some(progress_cb) = progress_cb {
                progress_cb(step(0.6));
            }

            out_mesh.vertex_coords_array.extend_from_slice(&cc_vertices);

            // add offset to face index
            for i in 0..cc_face_indices.len() {
                cc_face_indices[i] = cc_face_indices[i].wrapping_add(n_vertices as u32);
            }

            let mut face_vertex_offset_base: i32 = 0;

            // for each face in CC
            for f in 0..cc_face_count {
                if let Some(cancel_cb) = cancel_cb {
                    if cancel_cb() {
                        return false;
                    }
                }
                let reverse_winding_order = (fragment_location == MC_FRAGMENT_LOCATION_BELOW) && (patch_location == MC_PATCH_LOCATION_OUTSIDE);
                let face_size = face_sizes[f as usize] as i32;
                if reverse_winding_order {
                    let mut face_index: Vec<u32> = vec![0; face_size as usize];
                    // for each vertex in face
                    let mut v = face_size - 1;
                    while v >= 0 {
                        face_index[v as usize] = cc_face_indices[(face_vertex_offset_base as u64 + v as u64) as usize];
                        v -= 1;
                    }
                    cc_face_indices[face_vertex_offset_base as usize..face_vertex_offset_base as usize + face_size as usize]
                        .copy_from_slice(&face_index);
                }
                face_vertex_offset_base += face_size;
            }

            if let Some(progress_cb) = progress_cb {
                progress_cb(step(0.8));
            }

            out_mesh.face_indices_array.extend_from_slice(&cc_face_indices);
            out_mesh.face_sizes_array.extend_from_slice(&face_sizes);

            n_vertices += cc_vertex_count as i32;

            if let Some(progress_cb) = progress_cb {
                progress_cb(step(1.0));
            }
        }
        let _ = err;

        // free connected component data
        api.mc_release_connected_components(context);
        // destroy context
        api.mc_release_context(context);

        *src_mesh = out_mesh;
        if let Some(progress_cb) = progress_cb {
            progress_cb(100.0);
        }
        true
    }

    /// MeshBoolean.cpp:853-947 `bool do_boolean(McutMesh& srcMesh, const McutMesh& cutMesh, const
    /// std::string& boolean_opts, cancel_cb, progress_cb, failed_cb)`, the branch of a build
    /// without OpenMeshCraft: mcut refuses a source of several connected components, so each part
    /// of the source is cut by each part of the cut mesh and the results merged.
    ///
    /// The C++ catches a `std::exception` around it (calls `failed_cb`, returns false); nothing
    /// here throws.
    pub fn do_boolean(
        api: &dyn McutApi,
        src_mesh: &mut McutMesh,
        cut_mesh: &McutMesh,
        boolean_opts: &str,
        cancel_cb: BooleanCancelCB,
        progress_cb: BooleanProgressCB,
        _failed_cb: BooleanFailedCB,
    ) -> bool {
        let tri_src = mcut_to_triangle_mesh(src_mesh);
        let src_parts: Vec<indexed_triangle_set> = crate::triangle_mesh::its_split(&tri_src.its);

        let tri_cut = mcut_to_triangle_mesh(cut_mesh);
        let cut_parts: Vec<indexed_triangle_set> = crate::triangle_mesh::its_split(&tri_cut.its);

        if src_parts.is_empty() && boolean_opts == "UNION" {
            *src_mesh = cut_mesh.clone();
            return true;
        }
        if cut_parts.is_empty() {
            return true;
        }

        // when src mesh has multiple connected components, mcut refuses to work.
        // But we can force it to work by spliting the src mesh into disconnected components,
        // and do booleans seperately, then merge all the results.
        let mut all_its = indexed_triangle_set::default();
        let total_count = src_parts.len() * cut_parts.len();
        let count_index: Cell<i32> = Cell::new(0);
        let temp_progress = |progress: f32| {
            let current_progress = 10.0f32
                + 70.0f32 * (count_index.get() as f32 * 1.0f32 / total_count as f32)
                + 70.0f32 * 1.0f32 / total_count as f32 * progress / 100.0f32;
            if let Some(progress_cb) = progress_cb {
                progress_cb(current_progress);
            }
        };
        let temp_progress_cb: BooleanProgressCB = if progress_cb.is_some() { Some(&temp_progress) } else { None };
        if boolean_opts == "UNION" || boolean_opts == "A_NOT_B" {
            for i in 0..src_parts.len() {
                let mut src_part = triangle_mesh_to_mcut_its(&src_parts[i]);
                for j in 0..cut_parts.len() {
                    if let Some(cancel_cb) = cancel_cb {
                        if cancel_cb() {
                            return false;
                        }
                    }
                    let cut_part = triangle_mesh_to_mcut_its(&cut_parts[j]);
                    do_boolean_single(api, &mut src_part, &cut_part, boolean_opts, cancel_cb, temp_progress_cb);
                    count_index.set(count_index.get() + 1);
                }
                let tri_part = mcut_to_triangle_mesh(&src_part);
                crate::triangle_mesh::its_merge(&mut all_its, &tri_part.its);
            }
        } else if boolean_opts == "INTERSECTION" {
            for i in 0..src_parts.len() {
                for j in 0..cut_parts.len() {
                    if let Some(cancel_cb) = cancel_cb {
                        if cancel_cb() {
                            return false;
                        }
                    }
                    count_index.set(count_index.get() + 1);
                    let mut src_part = triangle_mesh_to_mcut_its(&src_parts[i]);
                    let cut_part = triangle_mesh_to_mcut_its(&cut_parts[j]);
                    let success = do_boolean_single(api, &mut src_part, &cut_part, boolean_opts, cancel_cb, temp_progress_cb);
                    if success {
                        let tri_part = mcut_to_triangle_mesh(&src_part);
                        crate::triangle_mesh::its_merge(&mut all_its, &tri_part.its);
                    }
                }
            }
        }
        *src_mesh = *triangle_mesh_to_mcut_its(&all_its);
        true
    }

    /// MeshBoolean.cpp:949-988 `void make_boolean(const TriangleMesh &src_mesh, const TriangleMesh
    /// &cut_mesh, std::vector<TriangleMesh> &dst_mesh, const std::string &boolean_opts, cancel_cb,
    /// progress_cb, failed_cb)`, the branch of a build without OpenMeshCraft: the boolean through
    /// mcut, the result split into parts, each part of negative volume flipped, merged back.
    #[allow(clippy::too_many_arguments)]
    pub fn make_boolean(
        api: &dyn McutApi,
        src_mesh: &TriangleMesh,
        cut_mesh: &TriangleMesh,
        dst_mesh: &mut Vec<TriangleMesh>,
        boolean_opts: &str,
        cancel_cb: BooleanCancelCB,
        progress_cb: BooleanProgressCB,
        failed_cb: BooleanFailedCB,
    ) {
        let mut src_mcut = McutMesh::default();
        let mut cut_mcut = McutMesh::default();
        triangle_mesh_to_mcut(src_mesh, &mut src_mcut, &Transform3d::identity());
        triangle_mesh_to_mcut(cut_mesh, &mut cut_mcut, &Transform3d::identity());
        //dst_mesh = make_boolean(srcMesh, cutMesh, boolean_opts);
        let rt = do_boolean(api, &mut src_mcut, &cut_mcut, boolean_opts, cancel_cb, progress_cb, failed_cb);
        if !rt {
            return;
        }
        if let Some(cancel_cb) = cancel_cb {
            if cancel_cb() {
                return;
            }
        }
        if let Some(progress_cb) = progress_cb {
            progress_cb(90.0);
        }
        let mut tri_src = mcut_to_triangle_mesh(&src_mcut);
        if !tri_src.empty() {
            // Split into parts, fix negative volume for each part, then merge back
            let parts: Vec<TriangleMesh> = tri_src.split(1.0);
            if parts.len() > 1 {
                let mut fixed_mesh = TriangleMesh::new();
                for mut part in parts {
                    if part.volume() < 0.0 {
                        part.flip_triangles();
                    }
                    fixed_mesh.merge(&part);
                }
                dst_mesh.push(fixed_mesh);
            } else {
                if tri_src.volume() < 0.0 {
                    tri_src.flip_triangles();
                }
                dst_mesh.push(tri_src);
            }
        }
        if let Some(progress_cb) = progress_cb {
            progress_cb(100.0);
        }
    }
}

/// MeshBoolean.cpp:992-1001 `void make_boolean(...)`, the backend-neutral dispatcher: mcut in a
/// build without OpenMeshCraft.
#[allow(clippy::too_many_arguments)]
pub fn make_boolean(
    api: &dyn mcut::McutApi,
    src_mesh: &TriangleMesh,
    cut_mesh: &TriangleMesh,
    dst_mesh: &mut Vec<TriangleMesh>,
    boolean_opts: &str,
    cancel_cb: mcut::BooleanCancelCB,
    progress_cb: mcut::BooleanProgressCB,
    failed_cb: mcut::BooleanFailedCB,
) {
    mcut::make_boolean(api, src_mesh, cut_mesh, dst_mesh, boolean_opts, cancel_cb, progress_cb, failed_cb);
}

#[cfg(test)]
mod tests {
    use super::mcut::*;
    use super::*;

    fn unit_tri_its() -> indexed_triangle_set {
        let mut its = indexed_triangle_set::default();
        its.vertices = vec![Vec3f::new(0.0, 0.0, 0.0), Vec3f::new(1.0, 0.0, 0.0), Vec3f::new(0.0, 1.0, 0.0)];
        its.indices = vec![Vec3i::new(0, 1, 2)];
        its
    }

    #[test]
    fn test_empty_mcut_mesh() {
        let m = McutMesh::default();
        assert!(empty(&m));
    }

    #[test]
    fn test_triangle_mesh_to_mcut_roundtrip() {
        let mc = triangle_mesh_to_mcut_its(&unit_tri_its());
        assert_eq!(mc.vertex_coords_array.len(), 9);
        assert_eq!(mc.face_indices_array, vec![0, 1, 2]);
        assert_eq!(mc.face_sizes_array, vec![3u32]);
        assert!(!empty(&mc));
        let back = mcut_to_triangle_mesh(&mc);
        assert_eq!(back.its.vertices.len(), 3);
        assert_eq!(back.its.indices, vec![Vec3i::new(0, 1, 2)]);
    }

    #[test]
    fn test_merge_mcut_meshes() {
        let mut src = *triangle_mesh_to_mcut_its(&unit_tri_its());
        let cut = *triangle_mesh_to_mcut_its(&unit_tri_its());
        merge_mcut_meshes(&mut src, &cut);
        assert_eq!(src.vertex_coords_array.len(), 18);
        assert_eq!(src.face_sizes_array.len(), 2);
        assert_eq!(&src.face_indices_array[3..], &[3, 4, 5]);
    }

    #[test]
    fn test_eigen_round_trip() {
        let mesh = TriangleMesh::from_its(unit_tri_its());
        let e = triangle_mesh_to_eigen(&mesh);
        assert_eq!((e.0.nrows(), e.0.ncols(), e.1.nrows()), (3, 3, 1));
        let back = eigen_to_triangle_mesh(&e);
        assert_eq!(back.its.vertices, mesh.its.vertices);
        assert_eq!(back.its.indices, mesh.its.indices);
    }

    #[test]
    fn test_openmeshcraft_disabled() {
        assert!(!openmeshcraft::is_enabled());
        let called = std::cell::Cell::new(false);
        let failed = || called.set(true);
        let mut dst = Vec::new();
        let m = TriangleMesh::from_its(unit_tri_its());
        openmeshcraft::make_boolean(&m, &m, &mut dst, "UNION", None, None, Some(&failed));
        assert!(called.get());
        assert!(dst.is_empty());
    }

    #[test]
    fn test_cgal_conversions() {
        let its = unit_tri_its();
        let cm = cgal::triangle_mesh_to_cgal_its(&its);
        assert!(!cgal::empty(&cm));
        let back = cgal::cgal_to_indexed_triangle_set(&cm);
        assert_eq!(back.vertices, its.vertices);
        assert_eq!(back.indices, its.indices);
    }
}
