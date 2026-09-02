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

use crate::libslic3r::EPSILON;
use crate::print_object::PrintObject;
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

/// R230: gate for the f32 Flow-chain fidelity port. Native Flow stores
/// float members (Flow.hpp:107-113) — every constructed width/height/spacing
/// is quantized to f32, and mm3_per_mm casts its double arithmetic back to
/// float (Flow.cpp:212-219). The rust f64 chain drifts ~2e-8, which is
/// invisible in E values (5 decimals) but flips LINE_WIDTH %g digits and
/// volumetric-capped F digits. Gated separately (FLOW_F32) so its
/// geometry impact can be measured in isolation.
pub fn flow_f32() -> bool {
    // R760: default ON. The f64 Flow chain made scaled widths off by one unit
    // (q64 0.45 → 45000 vs native float 0.44999999 → 44999), which shifted the
    // gap-collapse `min` by 0.12 units and drifted the gap-fill footprint on
    // all 254 gap layers (the R759 bisect's sole non-byte-exact stage).
    // Was presence-only env (R693 trap: FLOW_F32=0 turned it ON); now
    // faithful_gate semantics — set FLOW_F32=0 for the old f64 arm.
    static G: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    *G.get_or_init(|| crate::faithful_gate("FLOW_F32"))
}

/// Quantize to f32 under the FLOW_F32 gate (native float member store).
fn q32(v: f64) -> f64 {
    if flow_f32() {
        v as f32 as f64
    } else {
        v
    }
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
            width: q32(width),
            height: q32(height),
            spacing: q32(spacing),
            nozzle_diameter: q32(nozzle_diameter),
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
            width: q32(width),
            height: q32(height),
            spacing: q32(spacing),
            nozzle_diameter: q32(nozzle_diameter),
            bridge,
        }
    }

    /// Default-constructed flow with all members zero.
    ///
    /// Mirrors the C++ default constructor `Flow flow;` used e.g. by
    /// `SurfaceFillParams` in Fill.cpp (all fields zero-initialized).
    pub const fn zero() -> Self {
        Self {
            width: 0.0,
            height: 0.0,
            spacing: 0.0,
            nozzle_diameter: 0.0,
            bridge: false,
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
    // Flow.hpp:62  coord_t scaled_width() const { return coord_t(scale_(m_width)); }
    // FIDELITY-NOTE(F2): C++ coord_t is int32 here truncated toward zero from the
    // double `scale_(m_width)`; this crate's shared `scale()` primitive rounds and
    // targets Coord=i64. Reusing the crate primitive (per task) rather than
    // re-routing scaling per-file.
    #[inline]
    pub fn scaled_width(&self) -> Coord {
        // R775 (gate FLOW_SCALED_TRUNC): native `coord_t(scale_(m_width))` is a
        // TRUNCATION of double(f32 width)/1e-5 — e.g. width 0.42: f32 round-trip
        // gives 41999.998 scaled, native truncates to 41999 while the crate's
        // rounding `scale()` returned 42000. The 1-unit width shifted the arachne
        // `last` inset by 0.5 units, which moved EVERY WallToolPaths input point
        // (AWIN median drift 1.00 unit/pt).
        if crate::faithful_gate("FLOW_SCALED_TRUNC") {
            ((self.width as f32) as f64 / crate::libslic3r::SCALING_FACTOR) as Coord
        } else {
            scale(self.width)
        }
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
    // Flow.hpp:68  coord_t scaled_spacing() const { return coord_t(scale_(m_spacing)); }
    // FIDELITY-NOTE(F2): see scaled_width() — crate `scale()` (round, Coord=i64) vs
    // C++ coord_t(scale_(...)) (truncate-toward-zero, int32).
    #[inline]
    pub fn scaled_spacing(&self) -> Coord {
        // R775 — see scaled_width: native truncates double(f32 spacing)/1e-5.
        if crate::faithful_gate("FLOW_SCALED_TRUNC") {
            ((self.spacing as f32) as f64 / crate::libslic3r::SCALING_FACTOR) as Coord
        } else {
            scale(self.spacing)
        }
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
            q32((self.width * self.width) * 0.25 * PI)
        } else {
            // Rectangle with semicircles at the ends
            // = height × (width - height × (1 - π/4))
            // ≈ height × (width - 0.2146 × height)
            // Native computes in double from float members, then casts the
            // result to float (Flow.cpp:214-218 `float res = ...`).
            q32(self.height * (self.width - self.height * (1.0 - 0.25 * PI)))
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
    // Flow.hpp:80  coord_t scaled_elephant_foot_spacing() const
    //   { return coord_t(0.5f * float(this->scaled_width() + 0.6f * this->scaled_spacing())); }
    // FIDELITY-NOTE(F2): C++ does the intermediate arithmetic in `float` (f32) and the
    // final `coord_t(...)` is an int32 truncate-toward-zero. We compute in f64 (crate
    // convention: C++ `float` -> Rust f64) and `as Coord` truncates toward zero into
    // i64; operator order, the 0.5/0.6 constants and the truncation direction match.
    #[inline]
    pub fn scaled_elephant_foot_spacing(&self) -> Coord {
        // coord_t(0.5f * float(this->scaled_width() + 0.6f * this->scaled_spacing()))
        // Computed on the scaled (coord_t) values, matching C++ ordering / rounding.
        (0.5 * (self.scaled_width() as f64 + 0.6 * self.scaled_spacing() as f64)) as Coord
    }

    // === Flow Modification Methods ===

    /// Create a new Flow with different width, maintaining other parameters.
    ///
    /// # Panics
    ///
    /// Panics if this is a bridge flow (bridges have fixed width = height).
    pub fn with_width(&self, width: f64) -> FlowResult<Self> {
        // Flow.hpp:92
        debug_assert!(!self.bridge, "Cannot modify width of bridge flow");
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
        // Flow.hpp:96
        debug_assert!(!self.bridge, "Cannot modify height of bridge flow");
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
        // Flow.cpp:140
        let mut out = *self;
        if self.bridge {
            // Diameter of the rounded extrusion.
            // Flow.cpp:143
            debug_assert!(self.width == self.height);
            // Flow.cpp:144
            let gap = self.spacing - self.width;
            // Flow.cpp:145
            let new_diameter = new_spacing - gap;
            // Flow.cpp:146
            out.width = new_diameter;
            out.height = new_diameter;
        } else {
            // Flow.cpp:148
            debug_assert!(self.width >= self.height);
            // Flow.cpp:149
            out.width += new_spacing - self.spacing;
            // Flow.cpp:150-151
            if out.width < out.height {
                return Err(FlowError::InvalidArgument(
                    "Invalid spacing supplied to Flow::with_spacing()".to_string(),
                ));
            }
        }
        // Flow.cpp:153
        out.spacing = new_spacing;
        out.width = q32(out.width);
        out.height = q32(out.height);
        out.spacing = q32(out.spacing);
        Ok(out)
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
        // Flow.cpp:160
        debug_assert!(!self.bridge, "Cannot adjust cross section of bridge flow");
        // Flow.cpp:161
        debug_assert!(
            self.width >= self.height,
            "Flow width must be >= height for cross section adjustment"
        );

        // Adjust for bridge_flow, maintain the extrusion spacing.
        // Flow.cpp:164
        let area = self.mm3_per_mm()?;
        if area_new > area + EPSILON {
            // Increasing the flow rate.
            // Flow.cpp:166-167
            let new_full_spacing = area_new / self.height;
            if new_full_spacing > self.spacing {
                // Filling up the spacing without an air gap. Grow the extrusion in height.
                // Flow.cpp:169-171
                let height = area_new / self.spacing;
                Ok(Self::new_with_spacing(
                    Self::rounded_rectangle_extrusion_width_from_spacing(self.spacing, height),
                    height,
                    self.spacing,
                    self.nozzle_diameter,
                    false,
                ))
            } else {
                // Flow.cpp:173
                self.with_width(Self::rounded_rectangle_extrusion_width_from_spacing(
                    area / self.height,
                    self.height,
                ))
            }
        } else if area_new < area - EPSILON {
            // Decreasing the flow rate.
            // Flow.cpp:177
            let width_new = self.width - (area - area_new) / self.height;
            // Flow.cpp:178
            debug_assert!(width_new > 0.0);
            if width_new > self.height {
                // Shrink the extrusion width.
                // Flow.cpp:181
                self.with_width(width_new)
            } else {
                // Create a rounded extrusion.
                // Flow.cpp:184-185
                let dmr = (area_new / PI).sqrt();
                Ok(Self::new_with_spacing(
                    dmr,
                    dmr,
                    self.spacing,
                    self.nozzle_diameter,
                    false,
                ))
            }
        } else {
            // Flow.cpp:188
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
        // Native (Flow.cpp:191-199) runs an ALL-f32 chain: the coefficient
        // float(1. - 0.25*PI) is rounded to f32 FIRST, then height*coef and
        // the subtraction round in f32 — one f64 pass + final rounding lands
        // 1 f32-ULP off (measured: sp .377079636 vs native .377079606, R281).
        let spacing = if flow_f32() {
            let coef = (1.0 - 0.25 * PI) as f32;
            ((width as f32) - (height as f32) * coef) as f64
        } else {
            width - height * (1.0 - 0.25 * PI)
        };
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
        // Native (Flow.cpp:201-204) is DIFFERENT from spacing(): the inner
        // arithmetic promotes to DOUBLE (f32 spacing + f32 height * double
        // coef) and float() rounds ONCE at the end.
        if flow_f32() {
            (((spacing as f32) as f64) + ((height as f32) as f64) * (1.0 - 0.25 * PI)) as f32
                as f64
        } else {
            spacing + height * (1.0 - 0.25 * PI)
        }
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

    // === BLOCKED: Flow::extrusion_width (Flow.cpp:67-116) ===========================
    //
    // The two `Flow::extrusion_width(opt_key, ...)` static overloads, together with
    // their file-local helpers `opt_key_to_flow_role` (Flow.cpp:41-59) and
    // `throw_on_missing_variable` (Flow.cpp:61-64), are NOT portable into this crate
    // at present.
    //
    // They are built entirely on the dynamic, string-keyed `ConfigOptionResolver`
    // API:  `config.option<ConfigOptionFloatOrPercent>(opt_key)`,
    //        `config.option(opt_key_layer_height)`,
    //        `config.option<ConfigOptionFloatsNullable>("nozzle_diameter")`.
    // This crate models the config as a flat struct of concrete typed fields
    // (PrintConfig / PrintObjectConfig / PrintRegionConfig); there is no
    // `ConfigOptionResolver` / `DynamicConfig` / runtime `option(opt_key)` lookup
    // (see config.rs — the ConfigBase/DynamicConfig family is documented but not
    // implemented).
    //
    // In C++ these two overloads are consumed ONLY by PlaceholderParser.cpp
    // (PlaceholderParser.cpp:913 and :929) to provide hint values for `{...}`
    // template variables. PlaceholderParser is itself a dynamic-config consumer
    // and is not ported. No ported call site needs `Flow::extrusion_width`.
    //
    // Porting these faithfully would require first standing up the dynamic
    // `ConfigOptionResolver` infrastructure; doing so now would be a stub against
    // the flat-config reality. Left blocked until that infrastructure exists.

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

// === Support Material Flow free functions (Flow.cpp:225-264) ===
// These mirror the free functions in libslic3r/Flow.cpp. They take a
// `&PrintObject` exactly like the C++ `const PrintObject *object`, reaching the
// config through the now-wired view shapes `object.config()` (PrintObjectConfig)
// and `object.print().config()` (PrintConfig).
//
// NOTE on `get_at(support_filament - 1)`: C++ indexes the per-extruder
// `nozzle_diameter` vector by the support filament id. This crate models
// `nozzle_diameter` as a scalar (CoordF), so `get_at(...)` collapses to a
// direct read and the filament index is never consulted. The same scalar
// collapse is documented throughout print_config.rs.
//
// NOTE on config layout: in C++ `support_filament` / `support_interface_filament`
// live on PrintObjectConfig; in this crate they live on PrintConfig. Because of
// the scalar nozzle_diameter collapse above, the index source is irrelevant to
// the resulting Flow, so the layout difference is inert here.

// Flow.cpp:225  Flow support_material_flow(const PrintObject *object, float layer_height)
pub fn support_material_flow(object: &PrintObject, layer_height: f64) -> FlowResult<Flow> {
    let object_config = object.config();
    // Flow.cpp:227-233
    Flow::new_from_config_width(
        // Flow.cpp:228
        FlowRole::SupportMaterial,
        // The width parameter accepted by new_from_config_width is of type
        // ConfigOptionFloatOrPercent, the Flow class takes care of the percent
        // to value substitution.
        // Flow.cpp:230  (object->config().support_line_width.value > 0) ? support_line_width : line_width
        if object_config.support_line_width > 0.0 {
            object_config.support_line_width
        } else {
            object_config.line_width
        },
        // Flow.cpp:232  if object->config().support_filament == 0 (which means to not
        // trigger tool change, but use the current extruder instead), get_at will
        // return the 0th component. Scalar collapse: read nozzle_diameter directly.
        object.print().config().nozzle_diameter,
        // Flow.cpp:233  (layer_height > 0.f) ? layer_height : float(object->config().layer_height.value)
        if layer_height > 0.0 {
            layer_height
        } else {
            object_config.layer_height
        },
    )
}

//BBS
// Flow.cpp:236  Flow support_transition_flow(const PrintObject* object)
pub fn support_transition_flow(object: &PrintObject) -> Flow {
    //BBS: support transition of tree support is bridge flow
    // Flow.cpp:239  float dmr = float(object->print()->config().nozzle_diameter.get_at(object->config().support_filament - 1));
    let dmr = object.print().config().nozzle_diameter;
    // Flow.cpp:240
    Flow::bridging_flow(dmr, dmr)
}

// Flow.cpp:243  Flow support_material_1st_layer_flow(const PrintObject *object, float layer_height)
pub fn support_material_1st_layer_flow(object: &PrintObject, layer_height: f64) -> FlowResult<Flow> {
    // Flow.cpp:245  const PrintConfig &print_config = object->print()->config();
    let print_config = object.print().config();
    let object_config = object.config();
    // Flow.cpp:246  const auto &width = (print_config.initial_layer_line_width.value > 0) ? initial_layer_line_width : object->config().support_line_width;
    let width = if print_config.initial_layer_line_width > 0.0 {
        print_config.initial_layer_line_width
    } else {
        object_config.support_line_width
    };
    // Flow.cpp:247-253
    Flow::new_from_config_width(
        // Flow.cpp:248
        FlowRole::SupportMaterial,
        // The width parameter accepted by new_from_config_width is of type
        // ConfigOptionFloatOrPercent, the Flow class takes care of the percent
        // to value substitution.
        // Flow.cpp:250  (width.value > 0) ? width : object->config().line_width
        if width > 0.0 {
            width
        } else {
            object_config.line_width
        },
        // Flow.cpp:251  float(print_config.nozzle_diameter.get_at(object->config().support_filament-1))
        print_config.nozzle_diameter,
        // Flow.cpp:252  (layer_height > 0.f) ? layer_height : float(print_config.initial_layer_print_height.value)
        // In this crate the first-layer print height lives on PrintObjectConfig
        // as `first_layer_height` (C++ keeps it on PrintConfig as
        // `initial_layer_print_height`); the value is identical.
        if layer_height > 0.0 {
            layer_height
        } else {
            object_config.first_layer_height
        },
    )
}

// Flow.cpp:255  Flow support_material_interface_flow(const PrintObject *object, float layer_height)
pub fn support_material_interface_flow(
    object: &PrintObject,
    layer_height: f64,
) -> FlowResult<Flow> {
    let object_config = object.config();
    // Flow.cpp:257-263
    Flow::new_from_config_width(
        // Flow.cpp:258
        FlowRole::SupportMaterialInterface,
        // The width parameter accepted by new_from_config_width is of type
        // ConfigOptionFloatOrPercent, the Flow class takes care of the percent
        // to value substitution.
        // Flow.cpp:260  (object->config().support_line_width > 0) ? support_line_width : line_width
        if object_config.support_line_width > 0.0 {
            object_config.support_line_width
        } else {
            object_config.line_width
        },
        // Flow.cpp:262  if object->config().support_interface_filament == 0 ..., get_at
        // returns the 0th component. Scalar collapse: read nozzle_diameter directly.
        object.print().config().nozzle_diameter,
        // Flow.cpp:263  (layer_height > 0.f) ? layer_height : float(object->config().layer_height.value)
        if layer_height > 0.0 {
            layer_height
        } else {
            object_config.layer_height
        },
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
