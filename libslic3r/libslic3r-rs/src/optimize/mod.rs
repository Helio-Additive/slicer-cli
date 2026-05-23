//! Numerical optimization module.
//!
//! Provides generic optimizer interface and implementations:
//! - Brute force grid search
//! - NLopt-style derivative-free optimization (Nelder-Mead simplex)
//!
//! C++ Reference: Optimize/Optimizer.hpp, BruteforceOptimizer.hpp, NLoptOptimizer.hpp

pub mod bruteforce_optimizer;
pub mod n_lopt_optimizer;
pub mod optimizer;

// Re-export key types
pub use bruteforce_optimizer::BruteForceOptimizer;
pub use n_lopt_optimizer::{NLoptAlgorithm, NLoptOptimizer};
pub use optimizer::{
    Bound, Bounds, Input, OptDir, OptResult, Optimizer, ScoreGradient, StopCriteria,
};
