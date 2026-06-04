//! NLopt-style numerical optimizer.
//!
//! C++ Reference:
//! - Optimize/NLoptOptimizer.hpp
//!
//! In C++ this wraps the NLopt C library. Since NLopt is not readily available
//! as a Rust crate, this implementation provides a pure-Rust Nelder-Mead simplex
//! method that covers the most common use case (NLOPT_LN_NELDERMEAD).
//! The interface matches the C++ optimizer patterns.

use super::optimizer::{Bound, Bounds, Input, OptDir, OptResult, Optimizer, StopCriteria};

/// NLopt algorithm identifiers (mirrors a subset of nlopt_algorithm enum).
///
/// NLoptOptimizer.hpp:225-229
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NLoptAlgorithm {
    /// Nelder-Mead simplex (local, derivative-free)
    /// NLoptOptimizer.hpp:227
    NelderMead,

    /// Subplex (local, derivative-free)
    /// NLoptOptimizer.hpp:226
    Subplex,

    /// Evolutionary strategy (global)
    /// NLoptOptimizer.hpp:225
    GeneticESCH,

    /// DIRECT (global, derivative-free)
    /// NLoptOptimizer.hpp:228
    DIRECT,

    /// Multi-Level Single-Linkage (global)
    /// NLoptOptimizer.hpp:229
    MLSL,
}

/// NLopt-based optimizer.
///
/// Provides a pure-Rust implementation of derivative-free optimization
/// using the Nelder-Mead simplex method as the primary algorithm.
///
/// NLoptOptimizer.hpp:196-222
pub struct NLoptOptimizer {
    criteria: StopCriteria,
    direction: OptDir,
    algorithm: NLoptAlgorithm,
    seed_value: Option<u64>,
}

impl NLoptOptimizer {
    /// Create a new NLopt optimizer with the given algorithm.
    ///
    /// NLoptOptimizer.hpp:212
    pub fn new(algorithm: NLoptAlgorithm) -> Self {
        Self {
            criteria: StopCriteria::default(),
            direction: OptDir::Min,
            algorithm,
            seed_value: None,
        }
    }

    /// Create with stop criteria.
    pub fn with_criteria(algorithm: NLoptAlgorithm, criteria: StopCriteria) -> Self {
        Self {
            criteria,
            direction: OptDir::Min,
            algorithm,
            seed_value: None,
        }
    }

    /// Run Nelder-Mead simplex optimization.
    ///
    /// This is a faithful implementation of the Nelder-Mead algorithm which is
    /// the most commonly used NLopt algorithm in the C++ codebase.
    ///
    /// NLoptOptimizer.hpp:150-159
    fn nelder_mead<const N: usize, F>(
        &self,
        func: &F,
        initvals: &Input<N>,
        bounds: &Bounds<N>,
    ) -> OptResult<N>
    where
        F: Fn(&Input<N>) -> f64,
    {
        let sign: f64 = match self.direction {
            OptDir::Min => 1.0,
            OptDir::Max => -1.0,
        };

        // Objective: we always minimize internally
        let objective = |x: &Input<N>| -> f64 { sign * func(x) };

        let max_iter = if self.criteria.get_max_iterations() > 0 {
            self.criteria.get_max_iterations() as usize
        } else {
            1000 * N
        };

        let ftol_abs = self.criteria.get_abs_score_diff();
        let ftol_rel = self.criteria.get_rel_score_diff();
        let stopval = self.criteria.get_stop_score();

        // Clamp initial values to bounds
        let mut x0 = *initvals;
        for d in 0..N {
            x0[d] = x0[d].max(bounds[d].min()).min(bounds[d].max());
        }

        // Initialize simplex: N+1 vertices
        let mut simplex: Vec<([f64; N], f64)> = Vec::with_capacity(N + 1);
        let f0 = objective(&x0);
        simplex.push((x0, f0));

        for d in 0..N {
            let mut vertex = x0;
            let range = bounds[d].max() - bounds[d].min();
            let step = if range.is_finite() && range > 0.0 {
                range * 0.05
            } else {
                (vertex[d].abs() * 0.05).max(0.00025)
            };
            vertex[d] = (vertex[d] + step).min(bounds[d].max());
            let fv = objective(&vertex);
            simplex.push((vertex, fv));
        }

        // Nelder-Mead parameters
        let alpha = 1.0; // reflection
        let gamma = 2.0; // expansion
        let rho = 0.5; // contraction
        let sigma = 0.5; // shrink

        let clamp = |x: &mut [f64; N]| {
            for d in 0..N {
                x[d] = x[d].max(bounds[d].min()).min(bounds[d].max());
            }
        };

        for _iter in 0..max_iter {
            // Check stop condition
            if self.criteria.check_stop_condition() {
                break;
            }

            // Sort simplex by function value
            simplex.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));

            let n = simplex.len();
            let best_score = simplex[0].1;
            let worst_score = simplex[n - 1].1;

            // Check convergence
            let score_diff = (worst_score - best_score).abs();
            if !ftol_abs.is_nan() && score_diff < ftol_abs {
                break;
            }
            if !ftol_rel.is_nan() && score_diff < ftol_rel * best_score.abs() {
                break;
            }
            if !stopval.is_nan() && best_score <= sign * stopval {
                break;
            }

            // Compute centroid of all but worst
            let mut centroid = [0.0f64; N];
            for i in 0..(n - 1) {
                for d in 0..N {
                    centroid[d] += simplex[i].0[d];
                }
            }
            for d in 0..N {
                centroid[d] /= (n - 1) as f64;
            }

            // Reflection
            let mut reflected = [0.0f64; N];
            for d in 0..N {
                reflected[d] = centroid[d] + alpha * (centroid[d] - simplex[n - 1].0[d]);
            }
            clamp(&mut reflected);
            let f_reflected = objective(&reflected);

            if f_reflected < simplex[n - 2].1 && f_reflected >= simplex[0].1 {
                // Accept reflection
                simplex[n - 1] = (reflected, f_reflected);
                continue;
            }

            if f_reflected < simplex[0].1 {
                // Expansion
                let mut expanded = [0.0f64; N];
                for d in 0..N {
                    expanded[d] = centroid[d] + gamma * (reflected[d] - centroid[d]);
                }
                clamp(&mut expanded);
                let f_expanded = objective(&expanded);
                if f_expanded < f_reflected {
                    simplex[n - 1] = (expanded, f_expanded);
                } else {
                    simplex[n - 1] = (reflected, f_reflected);
                }
                continue;
            }

            // Contraction
            let mut contracted = [0.0f64; N];
            for d in 0..N {
                contracted[d] = centroid[d] + rho * (simplex[n - 1].0[d] - centroid[d]);
            }
            clamp(&mut contracted);
            let f_contracted = objective(&contracted);

            if f_contracted < simplex[n - 1].1 {
                simplex[n - 1] = (contracted, f_contracted);
                continue;
            }

            // Shrink
            let best = simplex[0].0;
            for i in 1..n {
                for d in 0..N {
                    simplex[i].0[d] = best[d] + sigma * (simplex[i].0[d] - best[d]);
                }
                clamp(&mut simplex[i].0);
                simplex[i].1 = objective(&simplex[i].0);
            }
        }

        // Return best result
        simplex.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));

        OptResult {
            result_code: 1, // Success
            optimum: simplex[0].0,
            score: sign * simplex[0].1, // Convert back from internal representation
        }
    }
}

impl<const N: usize> Optimizer<N> for NLoptOptimizer {
    /// Switch to minimization.
    /// NLoptOptimizer.hpp:201
    fn to_min(&mut self) -> &mut Self {
        self.direction = OptDir::Min;
        self
    }

    /// Switch to maximization.
    /// NLoptOptimizer.hpp:200
    fn to_max(&mut self) -> &mut Self {
        self.direction = OptDir::Max;
        self
    }

    /// Set optimization criteria.
    /// NLoptOptimizer.hpp:214-216
    fn set_criteria(&mut self, criteria: StopCriteria) -> &mut Self {
        self.criteria = criteria;
        self
    }

    /// Get current criteria.
    /// NLoptOptimizer.hpp:218
    fn get_criteria(&self) -> &StopCriteria {
        &self.criteria
    }

    /// Run the optimization.
    /// NLoptOptimizer.hpp:204-210
    fn optimize<F>(&mut self, func: F, initvals: &Input<N>, bounds: &Bounds<N>) -> OptResult<N>
    where
        F: Fn(&Input<N>) -> f64,
    {
        self.nelder_mead(&func, initvals, bounds)
    }

    /// Set the random seed.
    /// NLoptOptimizer.hpp:220
    fn seed(&mut self, s: u64) {
        self.seed_value = Some(s);
    }
}

/// Type aliases for commonly used NLopt algorithms.
///
/// NLoptOptimizer.hpp:225-229
pub type AlgNLoptSimplex = NLoptOptimizer;
pub type AlgNLoptSubplex = NLoptOptimizer;

/// Helper to create a Nelder-Mead optimizer (most common use case).
pub fn simplex_optimizer(criteria: StopCriteria) -> NLoptOptimizer {
    NLoptOptimizer::with_criteria(NLoptAlgorithm::NelderMead, criteria)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::optimize::optimizer::Bound;

    #[test]
    fn test_nlopt_minimize_1d() {
        let mut stc = StopCriteria::default();
        stc.abs_score_diff(1e-8).max_iterations(500);
        let mut opt = simplex_optimizer(stc);
        opt.to_min();

        let result: OptResult<1> = opt.optimize(
            |x: &[f64; 1]| (x[0] - 0.3) * (x[0] - 0.3),
            &[0.5],
            &[Bound::new(0.0, 1.0)],
        );

        assert!(
            (result.optimum[0] - 0.3).abs() < 0.01,
            "optimum={}",
            result.optimum[0]
        );
        assert!(result.score < 0.001);
    }

    #[test]
    fn test_nlopt_maximize_1d() {
        let mut stc = StopCriteria::default();
        stc.abs_score_diff(1e-8).max_iterations(500);
        let mut opt = simplex_optimizer(stc);
        opt.to_max();

        let result: OptResult<1> = opt.optimize(
            |x: &[f64; 1]| -((x[0] - 0.7) * (x[0] - 0.7)),
            &[0.5],
            &[Bound::new(0.0, 1.0)],
        );

        assert!(
            (result.optimum[0] - 0.7).abs() < 0.01,
            "optimum={}",
            result.optimum[0]
        );
    }

    #[test]
    fn test_nlopt_minimize_2d() {
        let mut stc = StopCriteria::default();
        stc.abs_score_diff(1e-10).max_iterations(2000);
        let mut opt = simplex_optimizer(stc);
        opt.to_min();

        let result: OptResult<2> = opt.optimize(
            |x: &[f64; 2]| (x[0] - 0.3).powi(2) + (x[1] - 0.7).powi(2),
            &[0.5, 0.5],
            &[Bound::new(0.0, 1.0), Bound::new(0.0, 1.0)],
        );

        assert!(
            (result.optimum[0] - 0.3).abs() < 0.05,
            "x={}",
            result.optimum[0]
        );
        assert!(
            (result.optimum[1] - 0.7).abs() < 0.05,
            "y={}",
            result.optimum[1]
        );
    }

    #[test]
    fn test_nlopt_algorithm_enum() {
        let alg = NLoptAlgorithm::NelderMead;
        assert_eq!(alg, NLoptAlgorithm::NelderMead);
    }

    #[test]
    fn test_nlopt_seed() {
        let mut opt = NLoptOptimizer::new(NLoptAlgorithm::NelderMead);
        opt.seed(42);
        assert_eq!(opt.seed_value, Some(42));
    }
}
