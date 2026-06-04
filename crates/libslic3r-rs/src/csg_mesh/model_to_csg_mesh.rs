//! Convert model volumes to CSG mesh parts.
//!
//! C++ Reference:
//! - CSGMesh/ModelToCSGMesh.hpp
//!
//! Provides functions to convert model volumes into a collection of CSG parts,
//! selecting positive parts, negative parts, drill holes, etc.

use super::csg_mesh::{CSGPart, CSGStackOp, CSGType, MeshPtr};
use crate::geometry::Transform3D;
use crate::triangle_mesh::TriangleMesh;

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

/// Convert model volumes to CSG mesh parts.
///
/// ModelToCSGMesh.hpp:22-88
pub fn model_to_csgmesh(
    volumes: &[&ModelVolume],
    trafo: &Transform3D,
    parts_to_include: ModelParts,
) -> (Vec<CSGPart>, bool) {
    let do_positives = parts_to_include.contains(ModelParts::POSITIVE);
    let do_negatives = parts_to_include.contains(ModelParts::NEGATIVE);
    let _do_drillholes = parts_to_include.contains(ModelParts::DRILL_HOLES);
    let _do_splits = parts_to_include.contains(ModelParts::DO_SPLITS);

    let mut out = Vec::new();
    let has_splitable_volume = false;

    for vol in volumes {
        // Check if this volume should be included
        // ModelToCSGMesh.hpp:37-38
        let should_include =
            (do_positives && vol.is_model_part) || (do_negatives && vol.is_negative_volume);

        if !should_include {
            continue;
        }

        // Create the CSG part
        // ModelToCSGMesh.hpp:63-69
        let operation = if vol.is_model_part {
            CSGType::Union
        } else {
            CSGType::Difference
        };

        // Compute combined transform: trafo * vol.matrix  (ModelToCSGMesh.hpp:66)
        // then(self, other) == other * self, so vol.matrix.then(trafo) == trafo * vol.matrix.
        let combined_transform = vol.matrix.then(trafo);

        let mut part = CSGPart::from_parts(
            MeshPtr::from_owned(vol.mesh.clone()),
            operation,
            combined_transform,
        );
        part.name = vol.name.clone();

        out.push(part);
    }

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
}
