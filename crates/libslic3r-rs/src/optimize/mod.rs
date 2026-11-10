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
// NLoptOptimizer.hpp: the NLopt-based optimizers and predefined algorithm tags.
// The numerical backend (libnlopt) is a native, non-wasm-safe dependency and is
// intentionally not wired up (see n_lopt_optimizer.rs module docs).
pub use n_lopt_optimizer::{
    alg_nlopt_direct, alg_nlopt_genetic, alg_nlopt_mlsl, alg_nlopt_simplex, alg_nlopt_subplex,
    NLoptAlgCombOptimizer, NLoptAlgOptimizer, NLoptBackendError, ObjectiveValue,
};
pub use optimizer::{
    Bound, Bounds, Input, OptDir, OptResult, Optimizer, ScoreGradient, StopCriteria,
};
