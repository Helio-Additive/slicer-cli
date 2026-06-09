//! Bicubic interpolation kernels and utilities.
//!
//! C++ Reference:
//! - Geometry/Bicubic.hpp
//! - SLA/bicubic.h (legacy duplicate of the same header-only template code)
//!
//! `Geometry/Bicubic.hpp` and the older `SLA/bicubic.h` are header-only and
//! contain the same kernel/interpolation templates (the SLA copy uses the name
//! `CubicKernel` instead of `CubicKernelWrapper`, free `typedef`s instead of
//! `using` aliases, and an unused extra `dx` parameter on the interpolation
//! free functions). The faithful 1:1 port lives in
//! [`crate::geometry::bicubic`]; this module re-exports it so the
//! `Slic3r::*` / `Slic3r::Geometry::*` symbols are reachable under the `sla`
//! path as well, without duplicating logic.
//!
//! NOTE: A previous version of this file implemented a fictional
//! `SLA/bicubic.h` API (`BicubicCoeffs`, `calculate_bicubic_coeffs`,
//! `bicubic_grid_interpolate`) that does not exist in BambuStudio. Those
//! symbols had no C++ counterpart and no callers, and have been removed in
//! favor of the faithful re-export below.

pub use crate::geometry::bicubic::{
    bicubic_interpolate, clamp, cubic_interpolate, CubicBSplineKernel, CubicCatmulRomKernel,
    CubicKernelWrapper, KernelCoefficients, LinearKernel,
};
