//! # Flow Calculation Module
//!
//! This module calculates extrusion flow parameters, providing the fundamental math
//! that converts desired extrusion dimensions (width, height) into actual material
//! flow rates (mm³/mm of travel).
//!
//! This is a **direct port** of BambuStudio's `libslic3r/Flow.hpp` and `libslic3r/Flow.cpp`.
//! The calculations must match exactly to produce identical slicing results.
//!
//! ## Key Concept: Rounded Rectangle Cross-Section
//!
//! Extruded plastic forms a shape that is approximately a rectangle with semicircular
//! ends (like a stadium/discorectangle). The cross-sectional area is:
//!
//! ```text
//! area = height × (width - height × (1 - π/4))
//!      ≈ height × (width - 0.2146 × height)
//! ```
//!
//! This is NOT simply `width × height` - that would give ~10-15% error.
//!
//! ## Reference
//!
//! - `BambuStudio/src/libslic3r/Flow.hpp`
//! - `BambuStudio/src/libslic3r/Flow.cpp`

use std::f64::consts::PI;
use thiserror::Error;

use crate::{scale, Coord};

/// Extra spacing between bridge threads (mm)
/// Flow.hpp:14
pub const BRIDGE_EXTRA_SPACING: f64 = 0.05;

/// Flow calculation errors
/// Flow.hpp:25-47
#[derive(Debug, Error)]
pub enum FlowError {
    /// Spacing calculation produced a negative value
    /// Flow.cpp:15
    #[error("Flow spacing calculation produced negative spacing. Is extrusion width too small?")]
    NegativeSpacing,

    /// Flow calculation produced a negative value
    /// Flow.cpp:18
    #[error("Flow mm3_per_mm() produced negative flow. Is extrusion width too small?")]
    NegativeFlow,

    /// Invalid argument provided
    /// Flow.hpp:25-30
    #[error("Invalid flow argument: {0}")]
    InvalidArgument(String),

    /// Missing configuration variable
    /// Flow.hpp:43-47
    #[error("Missing flow configuration variable: {0}")]
    MissingVariable(String),
}

/// Result type for flow calculations.
pub type FlowResult<T> = Result<T, FlowError>;

/// Extrusion role - determines default width calculations
/// Flow.hpp:16-24
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum FlowRole {
    /// External (outer) perimeter - visible surface
    /// Flow.hpp:17
    ExternalPerimeter,
    /// Internal perimeters
    /// Flow.hpp:18
    Perimeter,
    /// Sparse infill
    /// Flow.hpp:19
    Infill,
    /// Solid infill (top/bottom surfaces)
    /// Flow.hpp:20
    SolidInfill,
    /// Top solid infill (topmost surface)
    /// Flow.hpp:21
    TopSolidInfill,
    /// Support material
    /// Flow.hpp:22
    SupportMaterial,
    /// Support material interface layer
    /// Flow.hpp:23
    SupportMaterialInterface,
    /// Support transition (BBS tree support)
    /// Flow.hpp:24
    SupportTransition,
}

/// Flow parameters for extrusion (rounded rectangle cross-section model)
/// Flow.hpp:49-117
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Flow {
    /// Extrusion width (mm)
    /// Flow.hpp:107
    width: f64,

    /// Extrusion height (mm)
    /// Flow.hpp:109
    height: f64,

    /// Spacing between extrusion centerlines (mm)
    /// Flow.hpp:111
    spacing: f64,

    /// Nozzle diameter used (mm)
    /// Flow.hpp:112
    nozzle_diameter: f64,

    /// Whether this is a bridging flow
    /// Flow.hpp:113
    bridge: bool,
}

/// Flow implementation
/// Flow.hpp:49-117
impl Flow {
    /// Create a new Flow for non-bridge extrusion
    /// Flow.cpp:196-205
    pub fn new(width: f64, height: f64, nozzle_diameter: f64) -> FlowResult<Self> {
        // Flow.cpp:198
        let spacing = Self::rounded_rectangle_extrusion_spacing(width, height)?;
        Ok(Self {
            width,
            height,
            spacing,
            nozzle_diameter,
            bridge: false,
        })
    }

    /// Create a new Flow with explicit spacing (internal use).
    ///
    /// This is the low-level constructor that matches the private C++ constructor.
    fn new_with_spacing(
        width: f64,
        height: f64,
        spacing: f64,
        nozzle_diameter: f64,
        bridge: bool,
    ) -> Self {
        // Note: C++ has an assertion that width >= height for non-bridge,
        // but comments note that gap fill can violate this, so we don't enforce.
        Self {
            width,
            height,
            spacing,
            nozzle_diameter,
            bridge,
        }
    }

    /// Create a bridging flow.
    ///
    /// Bridge extrusions have circular cross-section because unsupported
    /// filament naturally forms a round thread.
    ///
    /// # Arguments
    ///
    /// * `diameter` - Thread diameter (mm), typically close to nozzle diameter
    /// * `nozzle_diameter` - Nozzle diameter (mm)
    pub fn bridging_flow(diameter: f64, nozzle_diameter: f64) -> Self {
        Self::new_with_spacing(
            diameter,
            diameter,
            Self::bridge_extrusion_spacing(diameter),
            nozzle_diameter,
            true,
        )
    }

    /// Create a Flow from configuration, handling auto-width (0 = auto).
    ///
    /// This mirrors `Flow::new_from_config_width()` in libslic3r.
    ///
    /// # Arguments
    ///
    /// * `role` - Extrusion role (affects auto width calculation)
    /// * `width` - Configured width (0 = auto)
    /// * `nozzle_diameter` - Nozzle diameter (mm)
    /// * `height` - Layer height (mm)
    ///
    /// # Errors
    ///
    /// Returns error if height is invalid or spacing calculation fails.
    pub fn new_from_config_width(
        role: FlowRole,
        width: f64,
        nozzle_diameter: f64,
        height: f64,
    ) -> FlowResult<Self> {
        if height <= 0.0 {
            return Err(FlowError::InvalidArgument(
                "Invalid flow height (must be positive)".to_string(),
            ));
        }

        let w = if width == 0.0 {
            // Auto width based on role
            Self::auto_extrusion_width(role, nozzle_diameter)
        } else {
            width
        };

        Self::new(w, height, nozzle_diameter)
    }

    // === Getters ===

    /// Get the extrusion width (mm).
    #[inline]
    pub fn width(&self) -> f64 {
        self.width
    }

    /// Get the extrusion width as scaled coordinate.
    #[inline]
    pub fn scaled_width(&self) -> Coord {
        scale(self.width)
    }

    /// Get the extrusion height / layer height (mm).
    #[inline]
    pub fn height(&self) -> f64 {
        self.height
    }

    /// Get the spacing between extrusion centerlines (mm).
    #[inline]
    pub fn spacing(&self) -> f64 {
        self.spacing
    }

    /// Get the spacing as scaled coordinate.
    #[inline]
    pub fn scaled_spacing(&self) -> Coord {
        scale(self.spacing)
    }

    /// Get the nozzle diameter (mm).
    #[inline]
    pub fn nozzle_diameter(&self) -> f64 {
        self.nozzle_diameter
    }

    /// Check if this is a bridging flow.
    #[inline]
    pub fn is_bridge(&self) -> bool {
        self.bridge
    }

    // === Core Calculation: Cross-Section Area ===

    /// Calculate the cross-sectional area of the extrusion (mm²).
    ///
    /// This returns mm³ per mm of travel distance, which is equivalent to
    /// the cross-sectional area in mm².
    ///
    /// **This is the most critical function for flow accuracy.**
    ///
    /// # Formula
    ///
    /// For bridges (circular cross-section):
    /// ```text
    /// area = π × (width/2)² = width² × π/4
    /// ```
    ///
    /// For normal extrusions (rounded rectangle):
    /// ```text
    /// area = height × (width - height × (1 - π/4))
    ///      ≈ height × (width - 0.2146 × height)
    /// ```
    ///
    /// # Errors
    ///
    /// Returns `FlowError::NegativeFlow` if the result would be negative.
    pub fn mm3_per_mm(&self) -> FlowResult<f64> {
        let res = if self.bridge {
            // Area of a circle with diameter = width
            (self.width * self.width) * 0.25 * PI
        } else {
            // Rectangle with semicircles at the ends
            // = height × (width - height × (1 - π/4))
            // ≈ height × (width - 0.2146 × height)
            self.height * (self.width - self.height * (1.0 - 0.25 * PI))
        };

        if res <= 0.0 {
            Err(FlowError::NegativeFlow)
        } else {
            Ok(res)
        }
    }

    /// Calculate mm3_per_mm, panicking on error.
    ///
    /// Use this when you're certain the Flow is valid.
    #[inline]
    pub fn mm3_per_mm_unchecked(&self) -> f64 {
        self.mm3_per_mm()
            .expect("Flow::mm3_per_mm() produced negative flow")
    }

    // === Elephant Foot Compensation ===

    /// Get the spacing for elephant foot compensation detection.
    ///
    /// This is used to detect narrow parts where elephant foot compensation
    /// cannot be applied. Only used for external perimeters.
    ///
    /// Allows some perimeter squish (see INSET_OVERLAP_TOLERANCE in libslic3r).
    /// An overlap of 0.2× external perimeter spacing is allowed.
    #[inline]
    pub fn scaled_elephant_foot_spacing(&self) -> Coord {
        // 0.5 × (width + 0.6 × spacing)
        scale(0.5 * (self.width + 0.6 * self.spacing))
    }

    // === Flow Modification Methods ===

    /// Create a new Flow with different width, maintaining other parameters.
    ///
    /// # Panics
    ///
    /// Panics if this is a bridge flow (bridges have fixed width = height).
    pub fn with_width(&self, width: f64) -> FlowResult<Self> {
        assert!(!self.bridge, "Cannot modify width of bridge flow");
        let spacing = Self::rounded_rectangle_extrusion_spacing(width, self.height)?;
        Ok(Self::new_with_spacing(
            width,
            self.height,
            spacing,
            self.nozzle_diameter,
            false,
        ))
    }

    /// Create a new Flow with different height, maintaining other parameters.
    ///
    /// # Panics
    ///
    /// Panics if this is a bridge flow.
    pub fn with_height(&self, height: f64) -> FlowResult<Self> {
        assert!(!self.bridge, "Cannot modify height of bridge flow");
        let spacing = Self::rounded_rectangle_extrusion_spacing(self.width, height)?;
        Ok(Self::new_with_spacing(
            self.width,
            height,
            spacing,
            self.nozzle_diameter,
            false,
        ))
    }

    /// Create a new Flow adjusted for different spacing while maintaining proper extrusion.
    ///
    /// This adjusts width/height to achieve the new spacing while keeping the
    /// gap between extrusions constant.
    pub fn with_spacing(&self, new_spacing: f64) -> FlowResult<Self> {
        if self.bridge {
            // For bridge: adjust diameter, maintaining gap
            let gap = self.spacing - self.width;
            let new_diameter = new_spacing - gap;
            if new_diameter <= 0.0 {
                return Err(FlowError::InvalidArgument(
                    "New spacing too small for bridge flow".to_string(),
                ));
            }
            Ok(Self::new_with_spacing(
                new_diameter,
                new_diameter,
                new_spacing,
                self.nozzle_diameter,
                true,
            ))
        } else {
            // For non-bridge: adjust width to achieve new spacing
            let new_width = self.width + (new_spacing - self.spacing);
            if new_width < self.height {
                return Err(FlowError::InvalidArgument(
                    "New spacing produces width < height".to_string(),
                ));
            }
            Ok(Self::new_with_spacing(
                new_width,
                self.height,
                new_spacing,
                self.nozzle_diameter,
                false,
            ))
        }
    }

    /// Create a new Flow with adjusted width/height to reach a target cross-section area
    /// while maintaining the current spacing.
    ///
    /// This is used for flow ratio adjustments (e.g., bridge_flow_ratio).
    ///
    /// # Arguments
    ///
    /// * `area_new` - Target cross-sectional area (mm²)
    pub fn with_cross_section(&self, area_new: f64) -> FlowResult<Self> {
        assert!(!self.bridge, "Cannot adjust cross section of bridge flow");
        assert!(
            self.width >= self.height,
            "Flow width must be >= height for cross section adjustment"
        );

        let area = self.mm3_per_mm()?;
        const EPSILON: f64 = 1e-9;

        if area_new > area + EPSILON {
            // Increasing flow rate
            let new_full_spacing = area_new / self.height;
            if new_full_spacing > self.spacing {
                // Would create air gap - grow height instead
                let height = area_new / self.spacing;
                let width =
                    Self::rounded_rectangle_extrusion_width_from_spacing(self.spacing, height);
                Ok(Self::new_with_spacing(
                    width,
                    height,
                    self.spacing,
                    self.nozzle_diameter,
                    false,
                ))
            } else {
                // Can fit in current spacing - adjust width
                let width = Self::rounded_rectangle_extrusion_width_from_spacing(
                    area_new / self.height,
                    self.height,
                );
                Ok(Self::new_with_spacing(
                    width,
                    self.height,
                    self.spacing,
                    self.nozzle_diameter,
                    false,
                ))
            }
        } else if area_new < area - EPSILON {
            // Decreasing flow rate
            let width_new = self.width - (area - area_new) / self.height;
            if width_new > self.height {
                // Still a rounded rectangle - shrink width
                Ok(Self::new_with_spacing(
                    width_new,
                    self.height,
                    self.spacing,
                    self.nozzle_diameter,
                    false,
                ))
            } else {
                // Would become taller than wide - create circular extrusion
                let diameter = (area_new / PI).sqrt() * 2.0;
                Ok(Self::new_with_spacing(
                    diameter,
                    diameter,
                    self.spacing,
                    self.nozzle_diameter,
                    false, // Not a bridge, just small
                ))
            }
        } else {
            // No change needed
            Ok(*self)
        }
    }

    /// Create a new Flow with the cross-section area scaled by a ratio.
    ///
    /// This is a convenience wrapper around `with_cross_section()`.
    ///
    /// # Arguments
    ///
    /// * `ratio` - Multiplier for cross-section area (e.g., 1.05 for +5%)
    #[inline]
    pub fn with_flow_ratio(&self, ratio: f64) -> FlowResult<Self> {
        let current_area = self.mm3_per_mm()?;
        self.with_cross_section(current_area * ratio)
    }

    // === Static Helper Functions ===

    /// Calculate spacing between extrusion centerlines for rounded rectangle profile.
    ///
    /// The spacing is less than the width because adjacent extrusions overlap
    /// at their rounded ends.
    ///
    /// # Formula
    ///
    /// ```text
    /// spacing = width - height × (1 - π/4)
    ///         ≈ width - 0.2146 × height
    /// ```
    ///
    /// # Errors
    ///
    /// Returns `FlowError::NegativeSpacing` if the result would be non-positive.
    pub fn rounded_rectangle_extrusion_spacing(width: f64, height: f64) -> FlowResult<f64> {
        let spacing = width - height * (1.0 - 0.25 * PI);
        if spacing <= 0.0 {
            Err(FlowError::NegativeSpacing)
        } else {
            Ok(spacing)
        }
    }

    /// Calculate extrusion width from desired spacing for rounded rectangle profile.
    ///
    /// This is the inverse of `rounded_rectangle_extrusion_spacing()`.
    ///
    /// # Formula
    ///
    /// ```text
    /// width = spacing + height × (1 - π/4)
    /// ```
    #[inline]
    pub fn rounded_rectangle_extrusion_width_from_spacing(spacing: f64, height: f64) -> f64 {
        spacing + height * (1.0 - 0.25 * PI)
    }

    /// Calculate spacing for bridge extrusions.
    ///
    /// Bridge threads are round, so spacing = diameter + small gap.
    #[inline]
    pub fn bridge_extrusion_spacing(diameter: f64) -> f64 {
        diameter + BRIDGE_EXTRA_SPACING
    }

    /// Calculate sensible default extrusion width based on nozzle diameter and role.
    ///
    /// These defaults match the manual Prusa MK3 profiles in BambuStudio.
    pub fn auto_extrusion_width(role: FlowRole, nozzle_diameter: f64) -> f64 {
        match role {
            FlowRole::SupportMaterial
            | FlowRole::SupportMaterialInterface
            | FlowRole::SupportTransition
            | FlowRole::TopSolidInfill => nozzle_diameter,

            FlowRole::ExternalPerimeter
            | FlowRole::Perimeter
            | FlowRole::SolidInfill
            | FlowRole::Infill => 1.125 * nozzle_diameter,
        }
    }

    // === E-Value Calculation Helpers ===

    /// Calculate E-axis distance for a given travel distance.
    ///
    /// This converts the volumetric flow (mm³) to filament length (mm) based on
    /// filament diameter.
    ///
    /// # Arguments
    ///
    /// * `distance` - Travel distance (mm)
    /// * `filament_diameter` - Filament diameter (mm), typically 1.75 or 2.85
    ///
    /// # Formula
    ///
    /// ```text
    /// volume = mm3_per_mm × distance
    /// E = volume / (π × (filament_diameter/2)²)
    /// ```
    pub fn e_per_mm(&self, filament_diameter: f64) -> FlowResult<f64> {
        let mm3_per_mm = self.mm3_per_mm()?;
        let filament_area = PI * (filament_diameter / 2.0).powi(2);
        Ok(mm3_per_mm / filament_area)
    }

    /// Calculate E-axis distance for a path of given length.
    ///
    /// # Arguments
    ///
    /// * `path_length_mm` - Total path length (mm)
    /// * `filament_diameter` - Filament diameter (mm)
    pub fn extrusion_for_length(
        &self,
        path_length_mm: f64,
        filament_diameter: f64,
    ) -> FlowResult<f64> {
        Ok(self.e_per_mm(filament_diameter)? * path_length_mm)
    }

    /// Calculate E-axis distance, applying a flow multiplier.
    ///
    /// # Arguments
    ///
    /// * `path_length_mm` - Total path length (mm)
    /// * `filament_diameter` - Filament diameter (mm)
    /// * `flow_multiplier` - Extrusion multiplier (1.0 = 100%)
    pub fn extrusion_for_length_with_multiplier(
        &self,
        path_length_mm: f64,
        filament_diameter: f64,
        flow_multiplier: f64,
    ) -> FlowResult<f64> {
        Ok(self.e_per_mm(filament_diameter)? * path_length_mm * flow_multiplier)
    }
}

impl PartialOrd for Flow {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        // Compare by cross-section area (mm3_per_mm)
        // Use unchecked since ordering requires valid flows
        let self_area = self.mm3_per_mm().ok()?;
        let other_area = other.mm3_per_mm().ok()?;
        self_area.partial_cmp(&other_area)
    }
}

// === Support Material Flow Helpers ===
// These mirror the free functions in libslic3r/Flow.cpp

/// Create flow for support material.
///
/// # Arguments
///
/// * `support_line_width` - Configured support line width (0 = use default line_width)
/// * `default_line_width` - Default line width from config
/// * `nozzle_diameter` - Nozzle diameter for support extruder
/// * `layer_height` - Layer height (0 = use config layer_height)
/// * `default_layer_height` - Default layer height from config
pub fn support_material_flow(
    support_line_width: f64,
    default_line_width: f64,
    nozzle_diameter: f64,
    layer_height: f64,
    default_layer_height: f64,
) -> FlowResult<Flow> {
    let width = if support_line_width > 0.0 {
        support_line_width
    } else {
        default_line_width
    };
    let height = if layer_height > 0.0 {
        layer_height
    } else {
        default_layer_height
    };

    Flow::new_from_config_width(FlowRole::SupportMaterial, width, nozzle_diameter, height)
}

/// Create flow for support transition (tree support).
///
/// Support transitions use bridge flow (circular cross-section).
pub fn support_transition_flow(nozzle_diameter: f64) -> Flow {
    Flow::bridging_flow(nozzle_diameter, nozzle_diameter)
}

/// Create flow for first layer support material.
pub fn support_material_1st_layer_flow(
    initial_layer_line_width: f64,
    support_line_width: f64,
    default_line_width: f64,
    nozzle_diameter: f64,
    initial_layer_height: f64,
) -> FlowResult<Flow> {
    let width = if initial_layer_line_width > 0.0 {
        initial_layer_line_width
    } else if support_line_width > 0.0 {
        support_line_width
    } else {
        default_line_width
    };

    Flow::new_from_config_width(
        FlowRole::SupportMaterial,
        width,
        nozzle_diameter,
        initial_layer_height,
    )
}

/// Create flow for support material interface.
pub fn support_material_interface_flow(
    support_line_width: f64,
    default_line_width: f64,
    nozzle_diameter: f64,
    layer_height: f64,
    default_layer_height: f64,
) -> FlowResult<Flow> {
    let width = if support_line_width > 0.0 {
        support_line_width
    } else {
        default_line_width
    };
    let height = if layer_height > 0.0 {
        layer_height
    } else {
        default_layer_height
    };

    Flow::new_from_config_width(
        FlowRole::SupportMaterialInterface,
        width,
        nozzle_diameter,
        height,
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    const EPSILON: f64 = 1e-6;

    fn approx_eq(a: f64, b: f64) -> bool {
        (a - b).abs() < EPSILON
    }

    #[test]
    fn test_flow_new() {
        let flow = Flow::new(0.45, 0.2, 0.4).unwrap();
        assert!(approx_eq(flow.width(), 0.45));
        assert!(approx_eq(flow.height(), 0.2));
        assert!(approx_eq(flow.nozzle_diameter(), 0.4));
        assert!(!flow.is_bridge());
    }

    #[test]
    fn test_bridging_flow() {
        let flow = Flow::bridging_flow(0.4, 0.4);
        assert!(approx_eq(flow.width(), 0.4));
        assert!(approx_eq(flow.height(), 0.4));
        assert!(flow.is_bridge());
        assert!(approx_eq(flow.spacing(), 0.4 + BRIDGE_EXTRA_SPACING));
    }

    #[test]
    fn test_mm3_per_mm_non_bridge() {
        // Test the rounded rectangle formula
        let flow = Flow::new(0.45, 0.2, 0.4).unwrap();
        let area = flow.mm3_per_mm().unwrap();

        // Expected: height × (width - height × (1 - π/4))
        // = 0.2 × (0.45 - 0.2 × (1 - π/4))
        // = 0.2 × (0.45 - 0.2 × 0.2146)
        // = 0.2 × (0.45 - 0.0429)
        // = 0.2 × 0.4071
        // ≈ 0.0814
        let expected = 0.2 * (0.45 - 0.2 * (1.0 - 0.25 * PI));
        assert!(
            approx_eq(area, expected),
            "Got {}, expected {}",
            area,
            expected
        );
    }

    #[test]
    fn test_mm3_per_mm_bridge() {
        // Test the circular cross-section formula
        let flow = Flow::bridging_flow(0.4, 0.4);
        let area = flow.mm3_per_mm().unwrap();

        // Expected: π × (diameter/2)² = π × 0.2² = π × 0.04 ≈ 0.1257
        let expected = PI * 0.2 * 0.2;
        assert!(
            approx_eq(area, expected),
            "Got {}, expected {}",
            area,
            expected
        );
    }

    #[test]
    fn test_rounded_rectangle_spacing() {
        // spacing = width - height × (1 - π/4)
        let spacing = Flow::rounded_rectangle_extrusion_spacing(0.45, 0.2).unwrap();
        let expected = 0.45 - 0.2 * (1.0 - 0.25 * PI);
        assert!(
            approx_eq(spacing, expected),
            "Got {}, expected {}",
            spacing,
            expected
        );
    }

    #[test]
    fn test_width_from_spacing_roundtrip() {
        let original_width = 0.45;
        let height = 0.2;

        let spacing = Flow::rounded_rectangle_extrusion_spacing(original_width, height).unwrap();
        let recovered_width = Flow::rounded_rectangle_extrusion_width_from_spacing(spacing, height);

        assert!(
            approx_eq(original_width, recovered_width),
            "Roundtrip failed: {} -> {} -> {}",
            original_width,
            spacing,
            recovered_width
        );
    }

    #[test]
    fn test_auto_extrusion_width() {
        let nozzle = 0.4;

        // Top solid infill uses nozzle diameter
        assert!(approx_eq(
            Flow::auto_extrusion_width(FlowRole::TopSolidInfill, nozzle),
            0.4
        ));

        // Perimeter uses 1.125× nozzle
        assert!(approx_eq(
            Flow::auto_extrusion_width(FlowRole::Perimeter, nozzle),
            0.45
        ));

        // Support uses nozzle diameter
        assert!(approx_eq(
            Flow::auto_extrusion_width(FlowRole::SupportMaterial, nozzle),
            0.4
        ));
    }

    #[test]
    fn test_e_per_mm() {
        let flow = Flow::new(0.45, 0.2, 0.4).unwrap();
        let filament_diameter = 1.75;

        let e_per_mm = flow.e_per_mm(filament_diameter).unwrap();

        // E = mm3_per_mm / filament_area
        let mm3 = flow.mm3_per_mm().unwrap();
        let filament_area = PI * (filament_diameter / 2.0).powi(2);
        let expected = mm3 / filament_area;

        assert!(
            approx_eq(e_per_mm, expected),
            "Got {}, expected {}",
            e_per_mm,
            expected
        );
    }

    #[test]
    fn test_extrusion_for_length() {
        let flow = Flow::new(0.45, 0.2, 0.4).unwrap();
        let path_length = 10.0; // 10mm path
        let filament_diameter = 1.75;

        let e = flow
            .extrusion_for_length(path_length, filament_diameter)
            .unwrap();
        let e_per_mm = flow.e_per_mm(filament_diameter).unwrap();

        assert!(approx_eq(e, e_per_mm * path_length));
    }

    #[test]
    fn test_with_width() {
        let flow = Flow::new(0.45, 0.2, 0.4).unwrap();
        let wider = flow.with_width(0.5).unwrap();

        assert!(approx_eq(wider.width(), 0.5));
        assert!(approx_eq(wider.height(), 0.2)); // Height unchanged

        // Spacing should have been recalculated
        let expected_spacing = Flow::rounded_rectangle_extrusion_spacing(0.5, 0.2).unwrap();
        assert!(approx_eq(wider.spacing(), expected_spacing));
    }

    #[test]
    fn test_with_height() {
        let flow = Flow::new(0.45, 0.2, 0.4).unwrap();
        let taller = flow.with_height(0.3).unwrap();

        assert!(approx_eq(taller.width(), 0.45)); // Width unchanged
        assert!(approx_eq(taller.height(), 0.3));
    }

    #[test]
    fn test_with_flow_ratio() {
        let flow = Flow::new(0.45, 0.2, 0.4).unwrap();
        let original_area = flow.mm3_per_mm().unwrap();

        // Increase flow by 10%
        let boosted = flow.with_flow_ratio(1.1).unwrap();
        let boosted_area = boosted.mm3_per_mm().unwrap();

        // Area should be ~10% larger
        let expected_area = original_area * 1.1;
        assert!(
            (boosted_area - expected_area).abs() < 0.001,
            "Expected area {}, got {}",
            expected_area,
            boosted_area
        );

        // Spacing should be maintained
        assert!(
            approx_eq(boosted.spacing(), flow.spacing()),
            "Spacing should be maintained"
        );
    }

    #[test]
    fn test_negative_spacing_error() {
        // Width too small relative to height should produce negative spacing
        let result = Flow::rounded_rectangle_extrusion_spacing(0.1, 0.5);
        assert!(matches!(result, Err(FlowError::NegativeSpacing)));
    }

    #[test]
    fn test_flow_comparison() {
        let small = Flow::new(0.4, 0.15, 0.4).unwrap();
        let large = Flow::new(0.5, 0.25, 0.4).unwrap();

        assert!(small < large);
        assert!(large > small);
    }

    #[test]
    fn test_scaled_values() {
        let flow = Flow::new(0.45, 0.2, 0.4).unwrap();

        // 0.45mm = 450,000 scaled units (with SCALING_FACTOR = 1_000_000)
        assert_eq!(flow.scaled_width(), 450_000);

        // Spacing should also scale correctly
        let spacing = flow.spacing();
        assert_eq!(flow.scaled_spacing(), scale(spacing));
    }

    // === Parity tests with libslic3r ===
    // These test specific values that should match BambuStudio output

    #[test]
    fn test_parity_typical_perimeter() {
        // Typical external perimeter: 0.4mm nozzle, 0.2mm layer, auto width
        let width = Flow::auto_extrusion_width(FlowRole::ExternalPerimeter, 0.4);
        let flow = Flow::new(width, 0.2, 0.4).unwrap();

        // These values should match libslic3r output
        assert!(approx_eq(width, 0.45), "Auto width should be 0.45mm");

        let area = flow.mm3_per_mm().unwrap();
        // Expected from C++: 0.2 × (0.45 - 0.2 × 0.2146) ≈ 0.0814
        let expected = 0.2 * (0.45 - 0.2 * (1.0 - 0.25 * PI));
        assert!(
            (area - expected).abs() < 1e-9,
            "mm3_per_mm mismatch: got {}, expected {}",
            area,
            expected
        );
    }

    #[test]
    fn test_parity_bridge() {
        // Typical bridge flow
        let flow = Flow::bridging_flow(0.4, 0.4);

        // Bridge area = π × r² = π × 0.2²
        let area = flow.mm3_per_mm().unwrap();
        let expected = PI * 0.04;
        assert!(
            (area - expected).abs() < 1e-9,
            "Bridge mm3_per_mm mismatch: got {}, expected {}",
            area,
            expected
        );

        // Bridge spacing = diameter + BRIDGE_EXTRA_SPACING
        assert!(approx_eq(flow.spacing(), 0.4 + BRIDGE_EXTRA_SPACING));
    }

    #[test]
    fn test_simple_rectangle_vs_rounded() {
        // Demonstrate the difference between simple rectangle and proper formula
        let width = 0.45;
        let height = 0.2;

        // Wrong (simple rectangle): width × height
        let wrong_area = width * height;

        // Correct (rounded rectangle): height × (width - height × (1 - π/4))
        let flow = Flow::new(width, height, 0.4).unwrap();
        let correct_area = flow.mm3_per_mm().unwrap();

        // The simple formula overestimates by about 10-12%
        let error_percent = (wrong_area - correct_area) / correct_area * 100.0;
        assert!(
            error_percent > 10.0 && error_percent < 15.0,
            "Simple rectangle should overestimate by 10-15%, got {}%",
            error_percent
        );
    }
}
