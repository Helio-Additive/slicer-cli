//! Slicing parameters and configuration.
//!
//! This module provides the SlicingParams type containing all configuration
//! needed for the slicing process, mirroring BambuStudio's SlicingParameters.

use crate::CoordF;
use serde::{Deserialize, Serialize};
use std::fmt;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
/// Slicing mode determines how the mesh is interpreted during slicing
/// TriangleMeshSlicer.hpp:25-30
pub enum SlicingMode {
    /// Regular slicing - maintains all contours and their orientation
    /// TriangleMeshSlicer.hpp:26
    #[default]
    Regular,
    /// Even-odd fill rule - for compatibility with certain model types
    /// TriangleMeshSlicer.hpp:27
    EvenOdd,
    /// Positive mode - orients all contours CCW, closes holes
    /// TriangleMeshSlicer.hpp:28
    Positive,
    /// Positive largest contour - keeps only the largest contour
    /// TriangleMeshSlicer.hpp:29
    PositiveLargestContour,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
/// Parameters controlling the slicing process
/// Slicing.hpp:29-105
pub struct SlicingParams {
    /// Regular layer height (mm)
    /// Slicing.hpp:63
    pub layer_height: CoordF,

    /// First layer height (mm)
    /// Slicing.hpp:73
    pub first_layer_height: CoordF,

    /// Minimum layer height for variable layer height (mm)
    /// Slicing.hpp:67
    pub min_layer_height: CoordF,

    /// Maximum layer height for variable layer height (mm)
    /// Slicing.hpp:68
    pub max_layer_height: CoordF,

    /// Slicing mode
    /// TriangleMeshSlicer.hpp:25
    pub mode: SlicingMode,

    /// Morphological closing radius (mm) applied to slice contours
    /// TriangleMeshSlicer.hpp:45
    pub closing_radius: CoordF,

    /// Extra offset applied to slice contours (mm)
    /// TriangleMeshSlicer.hpp:48
    pub extra_offset: CoordF,

    /// Resolution for contour simplification (mm)
    /// TriangleMeshSlicer.hpp:50
    pub resolution: CoordF,

    /// Number of raft base layers
    /// Slicing.hpp:49
    pub base_raft_layers: usize,

    /// Number of raft interface layers
    /// Slicing.hpp:51
    pub interface_raft_layers: usize,

    /// Height of raft base layers (mm)
    /// Slicing.hpp:54
    pub base_raft_layer_height: CoordF,

    /// Height of raft interface layers (mm)
    /// Slicing.hpp:55
    pub interface_raft_layer_height: CoordF,

    /// Height of raft contact layer (mm)
    /// Slicing.hpp:56
    pub contact_raft_layer_height: CoordF,

    /// Whether the first object layer uses bridging flow over non-soluble raft
    /// Slicing.hpp:81
    pub first_object_layer_bridging: bool,

    /// Whether the support interface is soluble
    /// Slicing.hpp:85
    pub soluble_interface: bool,

    /// Gap between raft and object (mm)
    /// Slicing.hpp:87
    pub gap_raft_object: CoordF,

    /// Gap between object and support (mm)
    /// Slicing.hpp:89
    pub gap_object_support: CoordF,

    /// Gap between support and object (mm)
    /// Slicing.hpp:91
    pub gap_support_object: CoordF,
}

/// Implementation of SlicingParams methods
/// Slicing.hpp:29-44
impl SlicingParams {
    // Create new slicing parameters with default values
    // Slicing.hpp:31
    pub fn new() -> Self {
        Self::default()
    }

    /// Create slicing parameters with a specific layer height
    /// Slicing.hpp:31
    pub fn with_layer_height(layer_height: CoordF) -> Self {
        Self {
            layer_height,
            ..Default::default()
        }
    }

    /// Create slicing parameters with specific layer heights
    /// Slicing.hpp:31
    pub fn with_layer_heights(layer_height: CoordF, first_layer_height: CoordF) -> Self {
        Self {
            layer_height,
            first_layer_height,
            ..Default::default()
        }
    }

    /// Check if parameters are valid
    /// Slicing.hpp:47
    pub fn is_valid(&self) -> bool {
        self.layer_height > 0.0
            && self.first_layer_height > 0.0
            && self.min_layer_height > 0.0
            && self.max_layer_height >= self.min_layer_height
    }

    /// Check if raft is enabled
    /// Slicing.hpp:40
    pub fn has_raft(&self) -> bool {
        self.raft_layers() > 0
    }

    /// Get the total number of raft layers
    /// Slicing.hpp:41
    pub fn raft_layers(&self) -> usize {
        self.base_raft_layers + self.interface_raft_layers
    }

    /// Check if the first object layer height is fixed
    /// Slicing.hpp:44
    pub fn first_object_layer_height_fixed(&self) -> bool {
        !self.has_raft() || self.first_object_layer_bridging
    }

    /// Builder method: set layer height
    /// Slicing.hpp:63
    pub fn layer_height(mut self, height: CoordF) -> Self {
        // Slicing.hpp:63
        self.layer_height = height;
        // Slicing.hpp:63
        self
    }

    /// Builder method: set first layer height
    /// Slicing.hpp:73
    pub fn first_layer_height(mut self, height: CoordF) -> Self {
        // Slicing.hpp:73
        self.first_layer_height = height;
        // Slicing.hpp:73
        self
    }

    /// Builder method: set slicing mode
    /// TriangleMeshSlicer.hpp:25
    pub fn mode(mut self, mode: SlicingMode) -> Self {
        // TriangleMeshSlicer.hpp:25
        self.mode = mode;
        // TriangleMeshSlicer.hpp:25
        self
    }

    /// Builder method: set closing radius
    /// TriangleMeshSlicer.hpp:45
    pub fn closing_radius(mut self, radius: CoordF) -> Self {
        // TriangleMeshSlicer.hpp:45
        self.closing_radius = radius;
        // TriangleMeshSlicer.hpp:45
        self
    }

    /// Builder method: set resolution
    /// TriangleMeshSlicer.hpp:50
    pub fn resolution(mut self, resolution: CoordF) -> Self {
        // TriangleMeshSlicer.hpp:50
        self.resolution = resolution;
        // TriangleMeshSlicer.hpp:50
        self
    }
}

/// Default trait implementation for SlicingParams
/// Slicing.hpp:31
impl Default for SlicingParams {
    // Create default SlicingParams with standard values
    // Slicing.hpp:31
    fn default() -> Self {
        Self {
            layer_height: 0.2,
            first_layer_height: 0.2,
            min_layer_height: 0.07,
            max_layer_height: 0.3,
            mode: SlicingMode::Regular,
            closing_radius: 0.0,
            extra_offset: 0.0,
            resolution: 0.0,
            base_raft_layers: 0,
            interface_raft_layers: 0,
            base_raft_layer_height: 0.3,
            interface_raft_layer_height: 0.2,
            contact_raft_layer_height: 0.2,
            first_object_layer_bridging: false,
            soluble_interface: false,
            gap_raft_object: 0.1,
            gap_object_support: 0.2,
            gap_support_object: 0.2,
        }
    }
}

/// Display trait implementation for SlicingParams
/// Slicing.hpp:29
impl fmt::Display for SlicingParams {
    // Format SlicingParams for display
    // Slicing.hpp:29
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "SlicingParams(layer_height={:.3}mm, first_layer={:.3}mm, mode={:?})",
            self.layer_height, self.first_layer_height, self.mode
        )
    }
}

/// Check if two slicing parameter sets produce the same layer heights
/// Slicing.cpp:45-58
pub fn equal_layering(sp1: &SlicingParams, sp2: &SlicingParams) -> bool {
    sp1.base_raft_layers == sp2.base_raft_layers
        && sp1.interface_raft_layers == sp2.interface_raft_layers
        && (sp1.base_raft_layer_height - sp2.base_raft_layer_height).abs() < 1e-6
        && (sp1.interface_raft_layer_height - sp2.interface_raft_layer_height).abs() < 1e-6
        && (sp1.contact_raft_layer_height - sp2.contact_raft_layer_height).abs() < 1e-6
        && (sp1.layer_height - sp2.layer_height).abs() < 1e-6
        && (sp1.min_layer_height - sp2.min_layer_height).abs() < 1e-6
        && (sp1.max_layer_height - sp2.max_layer_height).abs() < 1e-6
        && (sp1.first_layer_height - sp2.first_layer_height).abs() < 1e-6
        && sp1.first_object_layer_bridging == sp2.first_object_layer_bridging
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_slicing_params_default() {
        let params = SlicingParams::default();
        assert!((params.layer_height - 0.2).abs() < 1e-6);
        assert!((params.first_layer_height - 0.2).abs() < 1e-6);
        assert!(params.is_valid());
    }

    #[test]
    fn test_slicing_params_builder() {
        let params = SlicingParams::new()
            .layer_height(0.15)
            .first_layer_height(0.25)
            .mode(SlicingMode::EvenOdd);

        assert!((params.layer_height - 0.15).abs() < 1e-6);
        assert!((params.first_layer_height - 0.25).abs() < 1e-6);
        assert_eq!(params.mode, SlicingMode::EvenOdd);
    }

    #[test]
    fn test_slicing_params_raft() {
        let mut params = SlicingParams::default();
        assert!(!params.has_raft());
        assert_eq!(params.raft_layers(), 0);

        params.base_raft_layers = 2;
        params.interface_raft_layers = 1;
        assert!(params.has_raft());
        assert_eq!(params.raft_layers(), 3);
    }

    #[test]
    fn test_slicing_params_invalid() {
        let mut params = SlicingParams::default();
        assert!(params.is_valid());

        params.layer_height = 0.0;
        assert!(!params.is_valid());

        params.layer_height = 0.2;
        params.min_layer_height = 0.5;
        params.max_layer_height = 0.3;
        assert!(!params.is_valid());
    }

    #[test]
    fn test_equal_layering() {
        let params1 = SlicingParams::default();
        let params2 = SlicingParams::default();
        assert!(equal_layering(&params1, &params2));

        let params3 = SlicingParams::default().layer_height(0.15);
        assert!(!equal_layering(&params1, &params3));
    }

    #[test]
    fn test_slicing_mode() {
        assert_eq!(SlicingMode::default(), SlicingMode::Regular);
    }
}
