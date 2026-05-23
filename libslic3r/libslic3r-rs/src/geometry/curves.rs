//! Curve fitting utilities.
//!
//! C++ Reference:
//! - Geometry/Curves.hpp
//!
//! Provides polynomial curve fitting and piecewise curve fitting
//! using least squares approximation.

/// A polynomial curve represented by its coefficients matrix.
///
/// For a curve in `Dimension` dimensions with polynomial order `n`,
/// the coefficients matrix has `Dimension` rows and `n+1` columns.
/// Each column corresponds to a power of the parameter (x^0, x^1, ..., x^n).
///
/// Geometry/Curves.hpp: PolynomialCurve
#[derive(Debug, Clone)]
pub struct PolynomialCurve {
    /// Coefficients matrix: each row is a dimension, each column is a polynomial order.
    /// Stored as row-major: coefficients[dim][order].
    pub coefficients: Vec<Vec<f32>>,
    /// Number of spatial dimensions.
    pub dimension: usize,
}

impl PolynomialCurve {
    /// Create a new empty PolynomialCurve.
    pub fn new(dimension: usize) -> Self {
        Self {
            coefficients: vec![Vec::new(); dimension],
            dimension,
        }
    }

    /// Evaluate the fitted curve at a given parameter value.
    ///
    /// Returns a vector of `dimension` values.
    ///
    /// Geometry/Curves.hpp: PolynomialCurve::get_fitted_value
    pub fn get_fitted_value(&self, value: f32) -> Vec<f32> {
        let mut result = vec![0.0f32; self.dimension];
        if self.coefficients.is_empty() || self.coefficients[0].is_empty() {
            return result;
        }
        let order = self.coefficients[0].len() - 1;
        let mut x = 1.0f32;
        for idx in 0..=order {
            for dim in 0..self.dimension {
                if idx < self.coefficients[dim].len() {
                    result[dim] += x * self.coefficients[dim][idx];
                }
            }
            x *= value;
        }
        result
    }
}

/// A piecewise curve fitted using a kernel function (e.g., B-spline or Catmul-Rom).
///
/// The curve is defined by coefficients at evenly-spaced control points,
/// and evaluation uses the kernel function to blend nearby coefficients.
///
/// Geometry/Curves.hpp: PiecewiseFittedCurve
#[derive(Debug, Clone)]
pub struct PiecewiseFittedCurve {
    /// Coefficients matrix: each row is a dimension, each column is a segment.
    pub coefficients: Vec<Vec<f32>>,
    /// Number of spatial dimensions.
    pub dimension: usize,
    /// Start parameter value.
    pub start: f32,
    /// Size of each segment in parameter space.
    pub segment_size: f32,
    /// Number of extra degrees of freedom at each endpoint.
    pub endpoints_level_of_freedom: usize,
}

impl PiecewiseFittedCurve {
    /// Create a new empty PiecewiseFittedCurve.
    pub fn new(dimension: usize) -> Self {
        Self {
            coefficients: vec![Vec::new(); dimension],
            dimension,
            start: 0.0,
            segment_size: 1.0,
            endpoints_level_of_freedom: 0,
        }
    }
}

/// Get the coefficients from a polynomial curve.
///
/// Returns a reference to the coefficients matrix.
///
/// Geometry/Curves.hpp: coefficients accessor
pub fn coefficients(curve: &PolynomialCurve) -> &Vec<Vec<f32>> {
    &curve.coefficients
}

/// Fit a polynomial curve to the given observations using weighted least squares.
///
/// Arguments:
/// - `observations`: data points to fit (each is a vector of `dimension` values)
/// - `observation_points`: parameter values where observations were made
/// - `weights`: importance weight for each observation
/// - `order`: polynomial order
/// - `dimension`: number of spatial dimensions
///
/// Returns a PolynomialCurve with fitted coefficients.
///
/// Geometry/Curves.hpp: fit_polynomial
pub fn fit_polynomial(
    observations: &[Vec<f32>],
    observation_points: &[f32],
    weights: &[f32],
    order: usize,
    dimension: usize,
) -> PolynomialCurve {
    let n = observation_points.len();
    let cols = order + 1;

    if n < cols || n != weights.len() || n != observations.len() {
        return PolynomialCurve::new(dimension);
    }

    // Build the weighted Vandermonde-like matrix T and weighted data_points
    // Then solve T^T * T * coeffs = T^T * data using normal equations
    // This is a simplified implementation using normal equations instead of QR

    let mut result = PolynomialCurve::new(dimension);

    // Build T matrix (n x cols) with weights applied
    let mut t_matrix = vec![vec![0.0f64; cols]; n];
    let mut weighted_data = vec![vec![0.0f64; n]; dimension];

    for i in 0..n {
        let sw = (weights[i] as f64).sqrt();
        let mut x = sw;
        let c = observation_points[i] as f64;
        for j in 0..cols {
            t_matrix[i][j] = x;
            x *= c;
        }
        for dim in 0..dimension {
            weighted_data[dim][i] = observations[i][dim] as f64 * sw;
        }
    }

    // Compute T^T * T (cols x cols)
    let mut ata = vec![vec![0.0f64; cols]; cols];
    for i in 0..cols {
        for j in 0..cols {
            let mut sum = 0.0;
            for k in 0..n {
                sum += t_matrix[k][i] * t_matrix[k][j];
            }
            ata[i][j] = sum;
        }
    }

    // Solve for each dimension using Gaussian elimination
    for dim in 0..dimension {
        // Compute T^T * b
        let mut atb = vec![0.0f64; cols];
        for i in 0..cols {
            let mut sum = 0.0;
            for k in 0..n {
                sum += t_matrix[k][i] * weighted_data[dim][k];
            }
            atb[i] = sum;
        }

        // Solve ata * x = atb using Gaussian elimination with partial pivoting
        let mut aug = vec![vec![0.0f64; cols + 1]; cols];
        for i in 0..cols {
            for j in 0..cols {
                aug[i][j] = ata[i][j];
            }
            aug[i][cols] = atb[i];
        }

        for i in 0..cols {
            // Find pivot
            let mut max_val = aug[i][i].abs();
            let mut max_row = i;
            for k in (i + 1)..cols {
                if aug[k][i].abs() > max_val {
                    max_val = aug[k][i].abs();
                    max_row = k;
                }
            }
            aug.swap(i, max_row);

            let pivot = aug[i][i];
            if pivot.abs() < 1e-12 {
                continue; // singular
            }

            for k in (i + 1)..cols {
                let factor = aug[k][i] / pivot;
                for j in i..=cols {
                    aug[k][j] -= factor * aug[i][j];
                }
            }
        }

        // Back substitution
        let mut x = vec![0.0f64; cols];
        for i in (0..cols).rev() {
            if aug[i][i].abs() < 1e-12 {
                continue;
            }
            x[i] = aug[i][cols];
            for j in (i + 1)..cols {
                x[i] -= aug[i][j] * x[j];
            }
            x[i] /= aug[i][i];
        }

        result.coefficients[dim] = x.iter().map(|&v| v as f32).collect();
    }

    result
}
