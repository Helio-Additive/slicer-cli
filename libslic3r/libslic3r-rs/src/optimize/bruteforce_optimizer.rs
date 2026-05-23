//! Brute-force grid search optimizer.
//!
//! C++ Reference:
//! - Optimize/BruteforceOptimizer.hpp
//!
//! Implements a grid search where the search interval is sampled in equidistant
//! points for each dimension. Grid size determines the number of samples per
//! dimension, so the total number of function calls is `grid_size ^ dimension`.

use super::optimizer::{Bounds, Input, OptResult, Optimizer, StopCriteria};

/// Return the iteration number for a given grid position.
///
/// BruteforceOptimizer.hpp:13-18
fn num_iter<const N: usize>(idx: &[usize; N], grid_size: usize) -> u64 {
    let mut ret: u64 = 0;
    for i in 0..N {
        ret += idx[i] as u64 * (grid_size as u64).pow(i as u32);
    }
    ret
}

/// Internal state for the brute-force algorithm.
///
/// BruteforceOptimizer.hpp:23-103
struct AlgBruteForce {
    to_min: bool,
    stc: StopCriteria,
    grid_size: usize,
}

impl AlgBruteForce {
    /// Create a new brute force algorithm.
    /// BruteforceOptimizer.hpp:28
    fn new(stc: StopCriteria, grid_size: usize) -> Self {
        Self {
            to_min: true,
            stc,
            grid_size,
        }
    }

    /// Recursive grid search implementation.
    ///
    /// For each dimension D, iterates over grid positions.
    /// When D < 0 (all dimensions assigned), evaluates the function.
    ///
    /// BruteforceOptimizer.hpp:36-80
    fn run<const N: usize, F, Cmp>(
        &self,
        dim: isize,
        idx: &mut [usize; N],
        result: &mut OptResult<N>,
        bounds: &Bounds<N>,
        func: &F,
        cmp: &Cmp,
    ) -> bool
    where
        F: Fn(&Input<N>) -> f64,
        Cmp: Fn(f64, f64) -> bool,
    {
        // Check stop condition
        // BruteforceOptimizer.hpp:42
        if self.stc.check_stop_condition() {
            return false;
        }

        if dim < 0 {
            // Evaluate the function
            // BruteforceOptimizer.hpp:44-68

            // Check max iterations
            // BruteforceOptimizer.hpp:47-49
            let max_iter = self.stc.get_max_iterations();
            if max_iter > 0 && num_iter(idx, self.grid_size) >= max_iter as u64 {
                return false;
            }

            // Compute input values from grid indices
            // BruteforceOptimizer.hpp:51-55
            let mut inp = [0.0f64; N];
            for d in 0..N {
                let b = &bounds[d];
                let step = (b.max() - b.min()) / (self.grid_size as f64 - 1.0);
                inp[d] = b.min() + idx[d] as f64 * step;
            }

            // Evaluate function
            // BruteforceOptimizer.hpp:57
            let score = func(&inp);

            // Compare and update best
            // BruteforceOptimizer.hpp:58-67
            if cmp(score, result.score) {
                let abs_diff = (score - result.score).abs();

                result.score = score;
                result.optimum = inp;

                // Check precision criteria
                // BruteforceOptimizer.hpp:65-67
                let abs_thresh = self.stc.get_abs_score_diff();
                let rel_thresh = self.stc.get_rel_score_diff();
                if (!abs_thresh.is_nan() && abs_diff < abs_thresh)
                    || (!rel_thresh.is_nan() && abs_diff < rel_thresh * score.abs())
                {
                    return false;
                }
            }
        } else {
            // Iterate over grid positions for this dimension
            // BruteforceOptimizer.hpp:70-76
            let d = dim as usize;
            for i in 0..self.grid_size {
                idx[d] = i;
                if !self.run(dim - 1, idx, result, bounds, func, cmp) {
                    return false;
                }
            }
        }

        true
    }

    /// Run the brute force optimization.
    ///
    /// BruteforceOptimizer.hpp:82-102
    fn optimize<const N: usize, F>(
        &self,
        func: F,
        _initvals: &Input<N>,
        bounds: &Bounds<N>,
    ) -> OptResult<N>
    where
        F: Fn(&Input<N>) -> f64,
    {
        let mut idx = [0usize; N];
        let mut result = OptResult::<N>::default();

        if self.to_min {
            // BruteforceOptimizer.hpp:90-92
            result.score = f64::MAX;
            self.run(
                N as isize - 1,
                &mut idx,
                &mut result,
                bounds,
                &func,
                &|a: f64, b: f64| a < b,
            );
        } else {
            // BruteforceOptimizer.hpp:94-98
            result.score = f64::MIN;
            self.run(
                N as isize - 1,
                &mut idx,
                &mut result,
                bounds,
                &func,
                &|a: f64, b: f64| a > b,
            );
        }

        result
    }
}

/// Brute-force optimizer that searches a grid over the parameter space.
///
/// BruteforceOptimizer.hpp:110-136
pub struct BruteForceOptimizer {
    alg: AlgBruteForce,
}

impl BruteForceOptimizer {
    /// Create a new brute force optimizer.
    ///
    /// BruteforceOptimizer.hpp:115-116
    pub fn new(criteria: StopCriteria, grid_size: usize) -> Self {
        Self {
            alg: AlgBruteForce::new(criteria, grid_size),
        }
    }

    /// Create with default criteria and grid size of 100.
    pub fn default_grid() -> Self {
        Self::new(StopCriteria::default(), 100)
    }
}

impl<const N: usize> Optimizer<N> for BruteForceOptimizer {
    /// Switch to minimization.
    /// BruteforceOptimizer.hpp:119
    fn to_min(&mut self) -> &mut Self {
        self.alg.to_min = true;
        self
    }

    /// Switch to maximization.
    /// BruteforceOptimizer.hpp:118
    fn to_max(&mut self) -> &mut Self {
        self.alg.to_min = false;
        self
    }

    /// Set criteria.
    /// BruteforceOptimizer.hpp:130-132
    fn set_criteria(&mut self, criteria: StopCriteria) -> &mut Self {
        self.alg.stc = criteria;
        self
    }

    /// Get criteria.
    /// BruteforceOptimizer.hpp:134
    fn get_criteria(&self) -> &StopCriteria {
        &self.alg.stc
    }

    /// Run the optimization.
    /// BruteforceOptimizer.hpp:122-128
    fn optimize<F>(&mut self, func: F, initvals: &Input<N>, bounds: &Bounds<N>) -> OptResult<N>
    where
        F: Fn(&Input<N>) -> f64,
    {
        self.alg.optimize(func, initvals, bounds)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::optimize::optimizer::Bound;

    #[test]
    fn test_num_iter() {
        assert_eq!(num_iter(&[0usize, 0], 10), 0);
        assert_eq!(num_iter(&[1usize, 0], 10), 1);
        assert_eq!(num_iter(&[0usize, 1], 10), 10);
        assert_eq!(num_iter(&[3usize, 2], 10), 23);
    }

    #[test]
    fn test_brute_force_minimize_1d() {
        // Minimize f(x) = (x - 0.3)^2 on [0, 1]
        let mut opt = BruteForceOptimizer::new(StopCriteria::default(), 100);
        opt.to_min();
        let result: OptResult<1> = opt.optimize(
            |x: &[f64; 1]| (x[0] - 0.3) * (x[0] - 0.3),
            &[0.5],
            &[Bound::new(0.0, 1.0)],
        );
        assert!((result.optimum[0] - 0.3).abs() < 0.02);
        assert!(result.score < 0.001);
    }

    #[test]
    fn test_brute_force_maximize_1d() {
        // Maximize f(x) = -(x - 0.7)^2 on [0, 1]
        let mut opt = BruteForceOptimizer::new(StopCriteria::default(), 100);
        opt.to_max();
        let result: OptResult<1> = opt.optimize(
            |x: &[f64; 1]| -((x[0] - 0.7) * (x[0] - 0.7)),
            &[0.5],
            &[Bound::new(0.0, 1.0)],
        );
        assert!((result.optimum[0] - 0.7).abs() < 0.02);
    }

    #[test]
    fn test_brute_force_minimize_2d() {
        // Minimize f(x,y) = (x-0.5)^2 + (y-0.5)^2 on [0,1]x[0,1]
        let mut opt = BruteForceOptimizer::new(StopCriteria::default(), 50);
        opt.to_min();
        let result: OptResult<2> = opt.optimize(
            |x: &[f64; 2]| (x[0] - 0.5).powi(2) + (x[1] - 0.5).powi(2),
            &[0.0, 0.0],
            &[Bound::new(0.0, 1.0), Bound::new(0.0, 1.0)],
        );
        assert!((result.optimum[0] - 0.5).abs() < 0.05);
        assert!((result.optimum[1] - 0.5).abs() < 0.05);
    }

    #[test]
    fn test_brute_force_with_max_iterations() {
        let mut stc = StopCriteria::new();
        stc.max_iterations(10);
        let mut opt = BruteForceOptimizer::new(stc, 100);
        opt.to_min();
        let result: OptResult<1> =
            opt.optimize(|x: &[f64; 1]| x[0] * x[0], &[0.5], &[Bound::new(0.0, 1.0)]);
        // Should still find something reasonable even with limited iterations
        assert!(result.score.is_finite());
    }

    #[test]
    fn test_default_grid() {
        let opt = BruteForceOptimizer::default_grid();
        assert_eq!(opt.alg.grid_size, 100);
    }
}
