//! Bicubic interpolation for SLA rasterization
//!
//! This module implements bicubic interpolation used in SLA support generation
//! and raster image processing.

/// C++ Reference: SLA/bicubic.h
/// Bicubic interpolation implementation
use crate::{Error, Result};

// ---------------------------------------------------------------------------
// Bicubic Interpolation
// ---------------------------------------------------------------------------

/// Bicubic interpolation coefficients
/// SLA/bicubic.h:15-20
/// C++: struct BicubicCoeffs {
/// C++:     double a00, a01, a02, a03;
/// C++:     double a10, a11, a12, a13;
/// C++:     double a20, a21, a22, a23;
/// C++:     double a30, a31, a32, a33;
/// C++: };
#[derive(Debug, Clone, Copy)]
pub struct BicubicCoeffs {
    pub a00: f64,
    pub a01: f64,
    pub a02: f64,
    pub a03: f64,
    pub a10: f64,
    pub a11: f64,
    pub a12: f64,
    pub a13: f64,
    pub a20: f64,
    pub a21: f64,
    pub a22: f64,
    pub a23: f64,
    pub a30: f64,
    pub a31: f64,
    pub a32: f64,
    pub a33: f64,
}

impl Default for BicubicCoeffs {
    /// Initialize with zero coefficients
    /// SLA/bicubic.h:23
    /// C++: BicubicCoeffs() : a00(0), a01(0), ..., a33(0) {}
    fn default() -> Self {
        Self {
            a00: 0.0,
            a01: 0.0,
            a02: 0.0,
            a03: 0.0,
            a10: 0.0,
            a11: 0.0,
            a12: 0.0,
            a13: 0.0,
            a20: 0.0,
            a21: 0.0,
            a22: 0.0,
            a23: 0.0,
            a30: 0.0,
            a31: 0.0,
            a32: 0.0,
            a33: 0.0,
        }
    }
}

/// Calculate bicubic interpolation coefficients from 4x4 grid
/// SLA/bicubic.h:45
/// C++: BicubicCoeffs calculate_bicubic_coeffs(const double grid[4][4]);
pub fn calculate_bicubic_coeffs(grid: &[[f64; 4]; 4]) -> BicubicCoeffs {
    // Compute coefficients using bicubic interpolation matrix
    // SLA/bicubic.h:46-95
    // C++: BicubicCoeffs calculate_bicubic_coeffs(const double grid[4][4]) {
    // C++:     // Apply bicubic interpolation matrix
    // C++:     BicubicCoeffs c;
    // C++:     // Matrix multiplication for coefficient calculation
    // C++:     // ... (detailed matrix math)
    // C++:     return c;
    // C++: }
    // Simplified implementation - full bicubic matrix multiplication needed
    let mut coeffs = BicubicCoeffs::default();

    // Center 4 values form the main coefficients
    coeffs.a00 = grid[1][1];
    coeffs.a01 = 0.5 * (grid[1][2] - grid[1][0]);
    coeffs.a10 = 0.5 * (grid[2][1] - grid[0][1]);
    coeffs.a11 = 0.25 * (grid[2][2] - grid[2][0] - grid[0][2] + grid[0][0]);

    // TODO: Full bicubic coefficient calculation
    // This is a simplified version - the C++ implementation uses
    // a full 16x16 matrix multiplication

    coeffs
}

/// Evaluate bicubic interpolation at position (x, y)
/// SLA/bicubic.h:98
/// C++: double bicubic_interpolate(const BicubicCoeffs& c, double x, double y);
pub fn bicubic_interpolate(coeffs: &BicubicCoeffs, x: f64, y: f64) -> f64 {
    // Evaluate polynomial with precomputed coefficients
    // SLA/bicubic.h:99-115
    // C++: double bicubic_interpolate(const BicubicCoeffs& c, double x, double y) {
    // C++:     double x2 = x * x;
    // C++:     double x3 = x2 * x;
    // C++:     double y2 = y * y;
    // C++:     double y3 = y2 * y;
    // C++:
    // C++:     return c.a00 + c.a01 * y + c.a02 * y2 + c.a03 * y3 +
    // C++:            c.a10 * x + c.a11 * x * y + c.a12 * x * y2 + c.a13 * x * y3 +
    // C++:            c.a20 * x2 + c.a21 * x2 * y + c.a22 * x2 * y2 + c.a23 * x2 * y3 +
    // C++:            c.a30 * x3 + c.a31 * x3 * y + c.a32 * x3 * y2 + c.a33 * x3 * y3;
    // C++: }
    let x2 = x * x;
    let x3 = x2 * x;
    let y2 = y * y;
    let y3 = y2 * y;

    coeffs.a00
        + coeffs.a01 * y
        + coeffs.a02 * y2
        + coeffs.a03 * y3
        + coeffs.a10 * x
        + coeffs.a11 * x * y
        + coeffs.a12 * x * y2
        + coeffs.a13 * x * y3
        + coeffs.a20 * x2
        + coeffs.a21 * x2 * y
        + coeffs.a22 * x2 * y2
        + coeffs.a23 * x2 * y3
        + coeffs.a30 * x3
        + coeffs.a31 * x3 * y
        + coeffs.a32 * x3 * y2
        + coeffs.a33 * x3 * y3
}

/// Bicubic interpolation over a 2D grid
/// SLA/bicubic.h:118
/// C++: double bicubic_grid_interpolate(const double* grid, int width, int height, double x, double y);
pub fn bicubic_grid_interpolate(
    grid: &[f64],
    width: usize,
    height: usize,
    x: f64,
    y: f64,
) -> Result<f64> {
    // Sample 4x4 neighborhood and interpolate
    // SLA/bicubic.h:119-145
    // C++: double bicubic_grid_interpolate(const double* grid, int width, int height, double x, double y) {
    // C++:     // Clamp coordinates to grid bounds
    // C++:     int xi = (int)floor(x);
    // C++:     int yi = (int)floor(y);
    // C++:
    // C++:     // Extract 4x4 neighborhood
    // C++:     double neighborhood[4][4];
    // C++:     for (int dy = -1; dy <= 2; ++dy) {
    // C++:         for (int dx = -1; dx <= 2; ++dx) {
    // C++:             int sx = std::clamp(xi + dx, 0, width - 1);
    // C++:             int sy = std::clamp(yi + dy, 0, height - 1);
    // C++:             neighborhood[dy + 1][dx + 1] = grid[sy * width + sx];
    // C++:         }
    // C++:     }
    // C++:
    // C++:     // Calculate coefficients and interpolate
    /// C++:     BicubicCoeffs c = calculate_bicubic_coeffs(neighborhood);
    /// C++:     double fx = x - xi;
    /// C++:     double fy = y - yi;
    /// C++:     return bicubic_interpolate(c, fx, fy);
    /// C++: }
    if grid.len() != width * height {
        return Err(Error::InvalidInput(
            "Grid size doesn't match width * height".to_string(),
        ));
    }

    let xi = x.floor() as isize;
    let yi = y.floor() as isize;

    // Extract 4x4 neighborhood with clamping
    let mut neighborhood = [[0.0; 4]; 4];
    for dy in -1..=2 {
        for dx in -1..=2 {
            let sx = (xi + dx).clamp(0, width as isize - 1) as usize;
            let sy = (yi + dy).clamp(0, height as isize - 1) as usize;
            neighborhood[(dy + 1) as usize][(dx + 1) as usize] = grid[sy * width + sx];
        }
    }

    let coeffs = calculate_bicubic_coeffs(&neighborhood);
    let fx = x - xi as f64;
    let fy = y - yi as f64;

    Ok(bicubic_interpolate(&coeffs, fx, fy))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_bicubic_coeffs_default() {
        /// Test default coefficient initialization
        let coeffs = BicubicCoeffs::default();
        assert_eq!(coeffs.a00, 0.0);
        assert_eq!(coeffs.a11, 0.0);
        assert_eq!(coeffs.a33, 0.0);
    }

    #[test]
    fn test_bicubic_interpolate_zero() {
        /// Test interpolation with zero coefficients
        let coeffs = BicubicCoeffs::default();
        let result = bicubic_interpolate(&coeffs, 0.5, 0.5);
        assert_eq!(result, 0.0);
    }

    #[test]
    fn test_bicubic_interpolate_constant() {
        /// Test interpolation with constant field
        let mut coeffs = BicubicCoeffs::default();
        coeffs.a00 = 1.0;
        let result = bicubic_interpolate(&coeffs, 0.5, 0.5);
        assert!((result - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_bicubic_grid_interpolate() {
        /// Test grid interpolation
        let grid = vec![
            1.0, 2.0, 3.0, 4.0, 2.0, 3.0, 4.0, 5.0, 3.0, 4.0, 5.0, 6.0, 4.0, 5.0, 6.0, 7.0,
        ];
        let result = bicubic_grid_interpolate(&grid, 4, 4, 1.5, 1.5);
        assert!(result.is_ok());
        assert!(result.unwrap() > 0.0);
    }

    #[test]
    fn test_bicubic_grid_invalid_size() {
        /// Test error handling for invalid grid size
        let grid = vec![1.0, 2.0, 3.0];
        let result = bicubic_grid_interpolate(&grid, 4, 4, 1.0, 1.0);
        assert!(result.is_err());
    }
}
