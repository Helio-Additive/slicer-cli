//! Convert model volumes to CSG mesh parts.
//!
//! C++ Reference:
//! - CSGMesh/ModelToCSGMesh.hpp
//!
//! Provides functions to convert model volumes into a collection of CSG parts,
//! selecting positive parts, negative parts, drill holes, etc.

use super::csg_mesh::{CSGPart, CSGStackOp, CSGType, MeshPtr};
use crate::geometry::{Point3F, Transform3D};
use crate::normal_utils::{indexed_triangle_set, StlTriangleVertexIndices, StlVertex};
use crate::triangle_mesh::{its_is_splittable, its_split, Triangle, TriangleMesh};

/// Flags to select which parts to export from a model into a CSG part collection.
///
/// These flags can be combined with bitwise OR.
///
/// ModelToCSGMesh.hpp:14-19
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ModelParts(pub u32);

impl ModelParts {
    /// Include positive (model) parts
    /// ModelToCSGMesh.hpp:15
    pub const POSITIVE: ModelParts = ModelParts(1);

    /// Include negative (subtracted) parts
    /// ModelToCSGMesh.hpp:16
    pub const NEGATIVE: ModelParts = ModelParts(2);

    /// Include drill holes
    /// ModelToCSGMesh.hpp:17
    pub const DRILL_HOLES: ModelParts = ModelParts(4);

    /// Split each splittable mesh and export as a union of CSG parts
    /// ModelToCSGMesh.hpp:18
    pub const DO_SPLITS: ModelParts = ModelParts(8);

    /// Check if a flag is set.
    pub fn contains(&self, flag: ModelParts) -> bool {
        (self.0 & flag.0) != 0
    }
}

impl std::ops::BitOr for ModelParts {
    type Output = Self;
    fn bitor(self, rhs: Self) -> Self {
        ModelParts(self.0 | rhs.0)
    }
}

impl Default for ModelParts {
    /// Default: include positive parts only
    /// ModelToCSGMesh.hpp:26
    fn default() -> Self {
        ModelParts::POSITIVE
    }
}

/// A model volume for CSG conversion.
///
/// This is a simplified representation of C++'s ModelVolume.
/// In a full implementation, this would reference the actual Model types.
pub struct ModelVolume {
    /// The mesh data
    pub mesh: TriangleMesh,

    /// Whether this is a model (positive) part
    pub is_model_part: bool,

    /// Whether this is a negative volume
    pub is_negative_volume: bool,

    /// The volume's transformation matrix
    pub matrix: Transform3D,

    /// Volume name
    pub name: String,
}

/// Bridge a `TriangleMesh` (the CSG module's mesh representation) into an
/// `indexed_triangle_set` (the `its`-level primitives consumed by `its_split` /
/// `its_is_splittable`).
///
/// In C++ `ModelVolume::mesh().its` already *is* an `indexed_triangle_set`, so no
/// conversion exists there (ModelToCSGMesh.hpp:40,46,64). This crate's CSGMesh
/// module was deliberately built around `TriangleMesh` (see the divergence note in
/// `csg_mesh.rs`), so we materialise the equivalent `its` here. The vertex element
/// type narrows `Point3F` (f64) to `StlVertex` (f32) exactly as the C++ STL/ITS
/// pipeline stores single-precision vertices.
fn triangle_mesh_to_its(mesh: &TriangleMesh) -> indexed_triangle_set {
    let mut its = indexed_triangle_set::default();
    its.vertices.reserve(mesh.vertices().len());
    for v in mesh.vertices() {
        its.vertices
            .push(StlVertex::new(v.x as f32, v.y as f32, v.z as f32));
    }
    its.indices.reserve(mesh.indices().len());
    for tri in mesh.indices() {
        its.indices.push(StlTriangleVertexIndices::new(
            tri.indices[0] as i32,
            tri.indices[1] as i32,
            tri.indices[2] as i32,
        ));
    }
    its
}

/// Inverse of [`triangle_mesh_to_its`]: rebuild a `TriangleMesh` from an
/// `indexed_triangle_set` part produced by `its_split`.
fn its_to_triangle_mesh(its: indexed_triangle_set) -> TriangleMesh {
    let vertices: Vec<Point3F> = its
        .vertices
        .iter()
        .map(|v| Point3F::new(v.x as f64, v.y as f64, v.z as f64))
        .collect();
    let indices: Vec<Triangle> = its
        .indices
        .iter()
        .map(|f| Triangle::new(f[0] as u32, f[1] as u32, f[2] as u32))
        .collect();
    TriangleMesh::from_parts(vertices, indices)
}

/// Convert model volumes to CSG mesh parts.
///
/// ModelToCSGMesh.hpp:22-88
pub fn model_to_csgmesh(
    volumes: &[&ModelVolume],
    trafo: &Transform3D,
    parts_to_include: ModelParts,
) -> (Vec<CSGPart>, bool) {
    // ModelToCSGMesh.hpp:29-32
    let do_positives = parts_to_include.contains(ModelParts::POSITIVE);
    let do_negatives = parts_to_include.contains(ModelParts::NEGATIVE);
    let _do_drillholes = parts_to_include.contains(ModelParts::DRILL_HOLES);
    let do_splits = parts_to_include.contains(ModelParts::DO_SPLITS);
    // ModelToCSGMesh.hpp:33
    let mut has_splitable_volume = false;

    let mut out: Vec<CSGPart> = Vec::new();

    // ModelToCSGMesh.hpp:35
    for vol in volumes {
        // ModelToCSGMesh.hpp:36-38
        // if (vol && vol->mesh_ptr() &&
        //     ((do_positives && vol->is_model_part()) ||
        //      (do_negatives && vol->is_negative_volume()))) {
        //
        // `vol` and the mesh pointer are always present in this crate's `ModelVolume`
        // representation (mesh is an owned `TriangleMesh`), so the `vol && vol->mesh_ptr()`
        // guards reduce to the part-selection test below.
        if (do_positives && vol.is_model_part) || (do_negatives && vol.is_negative_volume) {
            // ModelToCSGMesh.hpp:40
            // if (do_splits && its_is_splittable(vol->mesh().its)) {
            // Materialise the `its` only when `do_splits` is set, mirroring C++'s `&&`
            // short-circuit (the C++ accesses `vol->mesh().its` directly with no copy).
            let its = if do_splits {
                Some(triangle_mesh_to_its(&vol.mesh))
            } else {
                None
            };
            if do_splits && its_is_splittable(its.as_ref().unwrap()) {
                let its = its.unwrap();
                // ModelToCSGMesh.hpp:41-44
                // CSGPart part_begin{{}, vol->is_model_part() ? CSGType::Union : CSGType::Difference};
                // part_begin.stack_operation = CSGStackOp::Push;
                // *out = std::move(part_begin); ++out;
                let mut part_begin = CSGPart::from_parts(
                    MeshPtr::None,
                    if vol.is_model_part {
                        CSGType::Union
                    } else {
                        CSGType::Difference
                    },
                    Transform3D::identity(),
                );
                part_begin.stack_operation = CSGStackOp::Push;
                out.push(part_begin);

                // ModelToCSGMesh.hpp:46-56
                // its_split(vol->mesh().its, SplitOutputFn{[&out, &vol, &trafo](indexed_triangle_set &&its) {
                //     if (its.empty()) return;
                //     CSGPart part{std::make_unique<indexed_triangle_set>(std::move(its)),
                //              CSGType::Union,
                //              (trafo * vol->get_matrix()).cast<float>()};
                //     *out = std::move(part); ++out;
                // }});
                for part_its in its_split(&its) {
                    // ModelToCSGMesh.hpp:47-48  if (its.empty()) return;
                    if part_its.indices.is_empty() {
                        continue;
                    }
                    // ModelToCSGMesh.hpp:50-52
                    // then(self, other) == other * self, so vol.matrix.then(trafo) == trafo * vol.matrix.
                    let part = CSGPart::from_parts(
                        MeshPtr::from_owned(its_to_triangle_mesh(part_its)),
                        CSGType::Union,
                        vol.matrix.then(trafo),
                    );
                    out.push(part);
                }

                // ModelToCSGMesh.hpp:58-61
                // CSGPart part_end{{}};
                // part_end.stack_operation = CSGStackOp::Pop;
                // *out = std::move(part_end); ++out;
                let mut part_end = CSGPart::from_mesh(MeshPtr::None);
                part_end.stack_operation = CSGStackOp::Pop;
                out.push(part_end);

                // ModelToCSGMesh.hpp:62
                has_splitable_volume = true;
            } else {
                // ModelToCSGMesh.hpp:64-69
                // CSGPart part{&(vol->mesh().its),
                //              vol->is_model_part() ? CSGType::Union : CSGType::Difference,
                //              (trafo * vol->get_matrix()).cast<float>()};
                // part.name = vol->name;
                // *out = std::move(part); ++out;
                let mut part = CSGPart::from_parts(
                    MeshPtr::from_owned(vol.mesh.clone()),
                    if vol.is_model_part {
                        CSGType::Union
                    } else {
                        CSGType::Difference
                    },
                    // then(self, other) == other * self, so vol.matrix.then(trafo) == trafo * vol.matrix.
                    vol.matrix.then(trafo),
                );
                part.name = vol.name.clone();
                out.push(part);
            }
        }
    }

    // ModelToCSGMesh.hpp:74-85  (drill holes — disabled in the C++ source too)
    //if (do_drillholes) {
    //    sla::DrainHoles drainholes = sla::transformed_drainhole_points(mo, trafo);
    //
    //    for (const sla::DrainHole &dhole : drainholes) {
    //        CSGPart part{std::make_unique<const indexed_triangle_set>(
    //                         dhole.to_mesh()),
    //                     CSGType::Difference};
    //
    //        *out = std::move(part);
    //        ++out;
    //    }
    //}

    // ModelToCSGMesh.hpp:87
    (out, has_splitable_volume)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_volume(is_model: bool, is_negative: bool, name: &str) -> ModelVolume {
        ModelVolume {
            mesh: TriangleMesh::new(),
            is_model_part: is_model,
            is_negative_volume: is_negative,
            matrix: Transform3D::identity(),
            name: name.to_string(),
        }
    }

    #[test]
    fn test_model_parts_flags() {
        let flags = ModelParts::POSITIVE | ModelParts::NEGATIVE;
        assert!(flags.contains(ModelParts::POSITIVE));
        assert!(flags.contains(ModelParts::NEGATIVE));
        assert!(!flags.contains(ModelParts::DRILL_HOLES));
    }

    #[test]
    fn test_model_to_csgmesh_empty() {
        let volumes: Vec<&ModelVolume> = vec![];
        let (parts, has_split) =
            model_to_csgmesh(&volumes, &Transform3D::identity(), ModelParts::default());
        assert!(parts.is_empty());
        assert!(!has_split);
    }

    #[test]
    fn test_model_to_csgmesh_positive_only() {
        let v1 = make_volume(true, false, "positive");
        let v2 = make_volume(false, true, "negative");
        let volumes: Vec<&ModelVolume> = vec![&v1, &v2];

        let (parts, _) = model_to_csgmesh(&volumes, &Transform3D::identity(), ModelParts::POSITIVE);
        assert_eq!(parts.len(), 1);
        assert_eq!(parts[0].name, "positive");
        assert_eq!(parts[0].operation, CSGType::Union);
    }

    #[test]
    fn test_model_to_csgmesh_both() {
        let v1 = make_volume(true, false, "positive");
        let v2 = make_volume(false, true, "negative");
        let volumes: Vec<&ModelVolume> = vec![&v1, &v2];

        let flags = ModelParts::POSITIVE | ModelParts::NEGATIVE;
        let (parts, _) = model_to_csgmesh(&volumes, &Transform3D::identity(), flags);
        assert_eq!(parts.len(), 2);
        assert_eq!(parts[0].operation, CSGType::Union);
        assert_eq!(parts[1].operation, CSGType::Difference);
    }

    /// A mesh of two disconnected tetrahedra so `its_is_splittable` is true.
    fn two_part_mesh() -> TriangleMesh {
        // Tetra A around origin.
        let mut verts = vec![
            Point3F::new(0.0, 0.0, 0.0),
            Point3F::new(1.0, 0.0, 0.0),
            Point3F::new(0.0, 1.0, 0.0),
            Point3F::new(0.0, 0.0, 1.0),
        ];
        // Tetra B translated far away (disconnected).
        verts.extend_from_slice(&[
            Point3F::new(10.0, 0.0, 0.0),
            Point3F::new(11.0, 0.0, 0.0),
            Point3F::new(10.0, 1.0, 0.0),
            Point3F::new(10.0, 0.0, 1.0),
        ]);
        let indices = vec![
            Triangle::new(0, 1, 2),
            Triangle::new(0, 1, 3),
            Triangle::new(1, 2, 3),
            Triangle::new(0, 2, 3),
            Triangle::new(4, 5, 6),
            Triangle::new(4, 5, 7),
            Triangle::new(5, 6, 7),
            Triangle::new(4, 6, 7),
        ];
        TriangleMesh::from_parts(verts, indices)
    }

    #[test]
    fn test_model_to_csgmesh_do_splits() {
        let vol = ModelVolume {
            mesh: two_part_mesh(),
            is_model_part: true,
            is_negative_volume: false,
            matrix: Transform3D::identity(),
            name: "splitme".to_string(),
        };
        let volumes: Vec<&ModelVolume> = vec![&vol];

        let flags = ModelParts::POSITIVE | ModelParts::DO_SPLITS;
        let (parts, has_split) =
            model_to_csgmesh(&volumes, &Transform3D::identity(), flags);

        // ModelToCSGMesh.hpp:62  has_splitable_volume == true
        assert!(has_split);
        // Push + 2 union parts + Pop.
        assert_eq!(parts.len(), 4);
        assert_eq!(parts[0].stack_operation, CSGStackOp::Push);
        assert_eq!(parts[0].operation, CSGType::Union);
        assert_eq!(parts[1].operation, CSGType::Union);
        assert_eq!(parts[1].stack_operation, CSGStackOp::Continue);
        assert_eq!(parts[2].operation, CSGType::Union);
        assert_eq!(parts[3].stack_operation, CSGStackOp::Pop);
    }

    #[test]
    fn test_model_to_csgmesh_splittable_but_no_split_flag() {
        // Without DO_SPLITS, a splittable mesh is emitted as a single part.
        let vol = ModelVolume {
            mesh: two_part_mesh(),
            is_model_part: true,
            is_negative_volume: false,
            matrix: Transform3D::identity(),
            name: "whole".to_string(),
        };
        let volumes: Vec<&ModelVolume> = vec![&vol];

        let (parts, has_split) =
            model_to_csgmesh(&volumes, &Transform3D::identity(), ModelParts::POSITIVE);
        assert!(!has_split);
        assert_eq!(parts.len(), 1);
        assert_eq!(parts[0].name, "whole");
        assert_eq!(parts[0].stack_operation, CSGStackOp::Continue);
    }
}
