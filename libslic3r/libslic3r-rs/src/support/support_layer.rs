//! Support layer types.
//!
//! C++ Reference:
//! - Support/SupportLayer.hpp
//!
//! Defines the support layer type enum and the `SupportGeneratorLayer` struct
//! used internally during support generation. These carry more detailed
//! information than the final PrintObject layers.
//!
//! NOTE: This file is not currently wired into the module tree. The active
//! support implementation lives in `support/mod.rs`.

use crate::geometry::ExPolygons;

/// Support layer type.
///
/// SupportLayer.hpp: enum SupporLayerType
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SupportLayerType {
    Unknown,
    /// Raft base layer, printed with support material.
    RaftBase,
    /// Raft interface layer, printed with support interface material.
    RaftInterface,
    /// Bottom contact layer placed over a top surface of an object.
    BottomContact,
    /// Dense interface layer separated from object by a BottomContact layer.
    BottomInterface,
    /// Sparse base support layer.
    Base,
    /// Dense interface layer separated from object by a TopContact layer.
    TopInterface,
    /// Top contact layer directly supporting an overhang.
    TopContact,
    /// Undecided intermediate type; will become Base, BottomInterface, or TopInterface.
    Intermediate,
}

impl Default for SupportLayerType {
    fn default() -> Self {
        Self::Unknown
    }
}

/// Internal support layer used during generation.
///
/// SupportLayer.hpp: class SupportGeneratorLayer
#[derive(Debug, Clone, Default)]
pub struct SupportGeneratorLayer {
    pub layer_type: SupportLayerType,
    pub print_z: f64,
    pub height: f64,
    pub bottom_z: f64,
    pub bridging: bool,
    pub polygons: ExPolygons,
}

impl SupportGeneratorLayer {
    pub fn new() -> Self {
        Self::default()
    }

    /// Reset to default state.
    /// SupportLayer.hpp: reset()
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    /// Merge another layer's polygons into this one.
    /// SupportLayer.hpp: merge()
    pub fn merge(&mut self, other: SupportGeneratorLayer) {
        // Simplified: just append polygons. Full implementation uses Clipper union.
        self.polygons.extend(other.polygons);
    }
}

impl PartialOrd for SupportGeneratorLayer {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for SupportGeneratorLayer {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        // Order by increasing print_z, then decreasing height
        self.print_z
            .partial_cmp(&other.print_z)
            .unwrap_or(std::cmp::Ordering::Equal)
            .then(
                other
                    .height
                    .partial_cmp(&self.height)
                    .unwrap_or(std::cmp::Ordering::Equal),
            )
            .then(if self.bridging && !other.bridging {
                std::cmp::Ordering::Less
            } else if !self.bridging && other.bridging {
                std::cmp::Ordering::Greater
            } else {
                std::cmp::Ordering::Equal
            })
    }
}

impl Eq for SupportGeneratorLayer {}

/// Storage for support generator layers with arena-like allocation.
///
/// SupportLayer.hpp: class SupportGeneratorLayerStorage
#[derive(Debug, Clone, Default)]
pub struct SupportGeneratorLayerStorage {
    pub layers: Vec<SupportGeneratorLayer>,
}

impl SupportGeneratorLayerStorage {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn allocate(&mut self) -> &mut SupportGeneratorLayer {
        self.layers.push(SupportGeneratorLayer::default());
        self.layers.last_mut().unwrap()
    }
}
