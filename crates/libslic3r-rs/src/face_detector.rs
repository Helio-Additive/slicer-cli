//! Exterior face detection via axis-aligned ray casting.
//!
//! C++ Reference:
//! - FaceDetector.hpp
//! - FaceDetector.cpp
//!
//! Faithful 1:1 port of `Slic3r::FaceDetector`.

use std::collections::HashSet;

use crate::aabb_mesh::{AABBMesh, IndexedTriangleSet};
use crate::geometry::{Transform3D, Vec3};
use crate::triangle_mesh::{EnumFaceTypes, TriangleMesh};

// FaceDetector.cpp:8
static BBOX_OFFSET: f64 = 2.0;

/// FaceDetector.hpp:10-21
/// C++:
/// ```text
/// class FaceDetector {
/// public:
///     FaceDetector(std::vector<TriangleMesh>& tms, std::vector<Transform3d>& transfos, double sample_interval)
///         : m_meshes(tms), m_transfos(transfos), m_sample_interval(sample_interval) {}
///
///     void detect_exterior_face();
///
/// private:
///     std::vector<TriangleMesh>& m_meshes;
///     std::vector<Transform3d>& m_transfos;
///     double m_sample_interval;
/// };
/// ```
///
/// The C++ class holds references to the caller-owned mesh and transform
/// vectors and mutates the meshes in place. In Rust we mirror this by passing
/// the borrows into [`FaceDetector::detect_exterior_face`] rather than storing
/// references in the struct (which would impose a lifetime on every use).
pub struct FaceDetector {
    /// FaceDetector.hpp:20
    m_sample_interval: f64,
}

impl FaceDetector {
    /// FaceDetector.hpp:12-13
    /// C++:
    /// ```text
    /// FaceDetector(std::vector<TriangleMesh>& tms, std::vector<Transform3d>& transfos, double sample_interval)
    ///     : m_meshes(tms), m_transfos(transfos), m_sample_interval(sample_interval) {}
    /// ```
    ///
    /// `m_meshes`/`m_transfos` are passed to [`detect_exterior_face`] instead of
    /// being stored, so the constructor only retains the sample interval.
    pub fn new(sample_interval: f64) -> Self {
        Self {
            m_sample_interval: sample_interval,
        }
    }

    /// FaceDetector.cpp:9-88
    /// C++: `void FaceDetector::detect_exterior_face()`
    pub fn detect_exterior_face(
        &self,
        m_meshes: &mut [TriangleMesh],
        m_transfos: &[Transform3D],
    ) {
        // FaceDetector.cpp:11-18
        // C++:
        // struct MeshFacetRange {
        //     MeshFacetRange(TriangleMesh* tm, uint32_t facet_begin, uint32_t facet_end) :
        //         tm(tm), facet_begin(facet_begin), facet_end(facet_end) {}
        //     MeshFacetRange() : tm(nullptr), facet_begin(0), facet_end(0) {}
        //     TriangleMesh* tm;
        //     uint32_t facet_begin;
        //     uint32_t facet_end;
        // };
        //
        // In the C++ code `MeshFacetRange::tm` is a raw pointer into `m_meshes`.
        // Rust's borrow checker forbids keeping a pointer while we later index
        // `m_meshes` mutably, so we store the owning mesh index instead and use
        // it to mutate `m_meshes` in the final loop.
        struct MeshFacetRange {
            mesh_idx: usize,
            facet_begin: u32,
            facet_end: u32,
        }

        // FaceDetector.cpp:20
        // C++: TriangleMesh object_mesh;
        let mut object_mesh = TriangleMesh::new();
        // FaceDetector.cpp:21
        // C++: std::vector<MeshFacetRange> volume_facet_ranges;
        let mut volume_facet_ranges: Vec<MeshFacetRange> = Vec::new();
        // FaceDetector.cpp:22
        // C++: for (int i = 0; i < m_meshes.size(); i++) {
        for i in 0..m_meshes.len() {
            // FaceDetector.cpp:23
            // C++: TriangleMesh vol_mesh = m_meshes[i];
            let mut vol_mesh = m_meshes[i].clone();
            // FaceDetector.cpp:24
            // C++: volume_facet_ranges.emplace_back(&m_meshes[i], object_mesh.stats().number_of_facets, object_mesh.stats().number_of_facets + vol_mesh.stats().number_of_facets);
            volume_facet_ranges.push(MeshFacetRange {
                mesh_idx: i,
                facet_begin: object_mesh.stats().number_of_facets,
                facet_end: object_mesh.stats().number_of_facets + vol_mesh.stats().number_of_facets,
            });

            // FaceDetector.cpp:26
            // C++: vol_mesh.transform(m_transfos[i]);
            vol_mesh.transform(&m_transfos[i]);
            // FaceDetector.cpp:27
            // C++: object_mesh.merge(std::move(vol_mesh));
            object_mesh.merge(vol_mesh);
        }

        // FaceDetector.cpp:30
        // C++: sla::IndexedMesh indexed_mesh(object_mesh);
        // IndexedMesh.hpp:52 — `explicit IndexedMesh(const TriangleMesh &mesh, bool calculate_epsilon = false);`
        // The call site uses the default argument, so `calculate_epsilon == false`
        // (triangle_ray_epsilon stays at the constant default rather than being
        // derived from the average edge length).
        let indexed_mesh = AABBMesh::new(triangle_mesh_to_its(&object_mesh), false);
        // FaceDetector.cpp:31
        // C++: BoundingBoxf3 bbox = object_mesh.bounding_box();
        let mut bbox = object_mesh.bounding_box();
        // FaceDetector.cpp:32
        // C++: bbox.offset(BBOX_OFFSET);
        bbox.min.x -= BBOX_OFFSET;
        bbox.min.y -= BBOX_OFFSET;
        bbox.min.z -= BBOX_OFFSET;
        bbox.max.x += BBOX_OFFSET;
        bbox.max.y += BBOX_OFFSET;
        bbox.max.z += BBOX_OFFSET;

        // FaceDetector.cpp:34
        // C++: std::unordered_set<size_t> hit_face_indices;
        let mut hit_face_indices: HashSet<usize> = HashSet::new();

        // FaceDetector.cpp:36
        // x-axis rays
        // FaceDetector.cpp:37
        // C++: for (double y = bbox.min.y(); y < bbox.max.y(); y += m_sample_interval) {
        let mut y = bbox.min.y();
        while y < bbox.max.y() {
            // FaceDetector.cpp:38
            // C++: for (double z = bbox.min.z(); z < bbox.max.z(); z += m_sample_interval) {
            let mut z = bbox.min.z();
            while z < bbox.max.z() {
                // FaceDetector.cpp:39
                // C++: auto hit_result = indexed_mesh.query_ray_hit({ bbox.min.x(), y, z }, { 1.0, 0.0, 0.0 });
                let hit_result = indexed_mesh
                    .query_ray_hit(Vec3::new(bbox.min.x(), y, z), Vec3::new(1.0, 0.0, 0.0));
                // FaceDetector.cpp:40-41
                // C++: if (hit_result.is_hit())
                //          hit_face_indices.insert(hit_result.face());
                if hit_result.is_hit() {
                    hit_face_indices.insert(hit_result.face() as usize);
                }

                // FaceDetector.cpp:43
                // C++: hit_result = indexed_mesh.query_ray_hit({ bbox.max.x(), y, z }, { -1.0, 0.0, 0.0 });
                let hit_result = indexed_mesh
                    .query_ray_hit(Vec3::new(bbox.max.x(), y, z), Vec3::new(-1.0, 0.0, 0.0));
                // FaceDetector.cpp:44-45
                // C++: if (hit_result.is_hit())
                //          hit_face_indices.insert(hit_result.face());
                if hit_result.is_hit() {
                    hit_face_indices.insert(hit_result.face() as usize);
                }

                z += self.m_sample_interval;
            }
            y += self.m_sample_interval;
        }

        // FaceDetector.cpp:49
        // y-axis rays
        // FaceDetector.cpp:50
        // C++: for (double x = bbox.min.x(); x < bbox.max.x(); x += m_sample_interval) {
        let mut x = bbox.min.x();
        while x < bbox.max.x() {
            // FaceDetector.cpp:51
            // C++: for (double z = bbox.min.z(); z < bbox.max.z(); z += m_sample_interval) {
            let mut z = bbox.min.z();
            while z < bbox.max.z() {
                // FaceDetector.cpp:52
                // C++: auto hit_result = indexed_mesh.query_ray_hit({ x, bbox.min.y(), z }, { 0.0, 1.0, 0.0 });
                let hit_result = indexed_mesh
                    .query_ray_hit(Vec3::new(x, bbox.min.y(), z), Vec3::new(0.0, 1.0, 0.0));
                // FaceDetector.cpp:53-54
                // C++: if (hit_result.is_hit())
                //          hit_face_indices.insert(hit_result.face());
                if hit_result.is_hit() {
                    hit_face_indices.insert(hit_result.face() as usize);
                }

                // FaceDetector.cpp:56
                // C++: hit_result = indexed_mesh.query_ray_hit({ x, bbox.max.y(), z }, { 0.0, -1.0, 0.0 });
                let hit_result = indexed_mesh
                    .query_ray_hit(Vec3::new(x, bbox.max.y(), z), Vec3::new(0.0, -1.0, 0.0));
                // FaceDetector.cpp:57-58
                // C++: if (hit_result.is_hit())
                //          hit_face_indices.insert(hit_result.face());
                if hit_result.is_hit() {
                    hit_face_indices.insert(hit_result.face() as usize);
                }

                z += self.m_sample_interval;
            }
            x += self.m_sample_interval;
        }

        // FaceDetector.cpp:62
        // z-axis rays
        // FaceDetector.cpp:63
        // C++: for (double x = bbox.min.x(); x < bbox.max.x(); x += m_sample_interval) {
        let mut x = bbox.min.x();
        while x < bbox.max.x() {
            // FaceDetector.cpp:64
            // C++: for (double y = bbox.min.y(); y < bbox.max.y(); y += m_sample_interval) {
            let mut y = bbox.min.y();
            while y < bbox.max.y() {
                // FaceDetector.cpp:65
                // C++: auto hit_result = indexed_mesh.query_ray_hit({ x, y, bbox.min.z() }, { 0.0, 0.0, 1.0 });
                let hit_result = indexed_mesh
                    .query_ray_hit(Vec3::new(x, y, bbox.min.z()), Vec3::new(0.0, 0.0, 1.0));
                // FaceDetector.cpp:66-67
                // C++: if (hit_result.is_hit())
                //          hit_face_indices.insert(hit_result.face());
                if hit_result.is_hit() {
                    hit_face_indices.insert(hit_result.face() as usize);
                }

                // FaceDetector.cpp:69
                // C++: hit_result = indexed_mesh.query_ray_hit({ x, y, bbox.max.z() }, { 0.0, 0.0, -1.0 });
                let hit_result = indexed_mesh
                    .query_ray_hit(Vec3::new(x, y, bbox.max.z()), Vec3::new(0.0, 0.0, -1.0));
                // FaceDetector.cpp:70-71
                // C++: if (hit_result.is_hit())
                //          hit_face_indices.insert(hit_result.face());
                if hit_result.is_hit() {
                    hit_face_indices.insert(hit_result.face() as usize);
                }

                y += self.m_sample_interval;
            }
            x += self.m_sample_interval;
        }

        // FaceDetector.cpp:75
        // C++: for (size_t facet_idx : hit_face_indices) {
        for facet_idx in hit_face_indices {
            // FaceDetector.cpp:76
            // C++: TriangleMesh* tm = nullptr;
            let mut tm: Option<usize> = None;
            // FaceDetector.cpp:77
            // C++: uint32_t vol_facet_idx = 0;
            let mut vol_facet_idx: u32 = 0;
            // FaceDetector.cpp:78
            // C++: for (auto range : volume_facet_ranges) {
            for range in &volume_facet_ranges {
                // FaceDetector.cpp:79
                // C++: if (facet_idx >= range.facet_begin && facet_idx < range.facet_end) {
                if facet_idx >= range.facet_begin as usize && facet_idx < range.facet_end as usize {
                    // FaceDetector.cpp:80
                    // C++: tm = range.tm;
                    tm = Some(range.mesh_idx);
                    // FaceDetector.cpp:81
                    // C++: vol_facet_idx = facet_idx - range.facet_begin;
                    vol_facet_idx = facet_idx as u32 - range.facet_begin;
                    // FaceDetector.cpp:82
                    // C++: break;
                    break;
                }
            }

            // FaceDetector.cpp:86
            // C++: tm->its.get_property(vol_facet_idx).type = EnumFaceTypes::eExteriorAppearance;
            let tm = tm.expect("hit facet must fall within a volume facet range");
            m_meshes[tm].get_property(vol_facet_idx as usize).type_ =
                EnumFaceTypes::EExteriorAppearance;
        }
    }
}

/// Convert a [`TriangleMesh`] into the [`IndexedTriangleSet`] consumed by
/// [`AABBMesh`] (the crate's equivalent of `sla::IndexedMesh`).
///
/// FaceDetector.cpp:30 — `sla::IndexedMesh indexed_mesh(object_mesh);`
fn triangle_mesh_to_its(mesh: &TriangleMesh) -> IndexedTriangleSet {
    let vertices = mesh.vertices().to_vec();
    let indices = mesh
        .indices()
        .iter()
        .map(|tri| {
            [
                tri.indices[0] as usize,
                tri.indices[1] as usize,
                tri.indices[2] as usize,
            ]
        })
        .collect();
    IndexedTriangleSet::from_parts(vertices, indices)
}
