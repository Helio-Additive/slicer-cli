//! Generic optimizer interface.
//!
//! C++ Reference:
//! - Optimize/Optimizer.hpp
//!
//! Provides the core types and traits for numerical optimization:
//! - `OptResult`: Holds the optimization result (optimum point + score)
//! - `Bound`: Defines an interval of valid input values
//! - `StopCriteria`: Configurable stopping conditions
//! - `Optimizer` trait: Generic interface for optimization methods
//! - `ScoreGradient`: Score + optional gradient for gradient-based methods

/// Result of an optimization run.
///
/// Optimizer.hpp:16-20
#[derive(Debug, Clone)]
pub struct OptResult<const N: usize> {
    /// Method-dependent result code
    /// Optimizer.hpp:17
    pub result_code: i32,

    /// The optimum point found
    /// Optimizer.hpp:18
    pub optimum: [f64; N],

    /// The score at the optimum
    /// Optimizer.hpp:19
    pub score: f64,
}

impl<const N: usize> Default for OptResult<N> {
    fn default() -> Self {
        Self {
            result_code: 0,
            optimum: [0.0; N],
            score: 0.0,
        }
    }
}

/// An interval of possible input values for optimization.
///
/// Optimizer.hpp:23-33
#[derive(Debug, Clone, Copy)]
pub struct Bound {
    min: f64,
    max: f64,
}

impl Bound {
    /// Create a new bound with min and max values.
    ///
    /// Optimizer.hpp:27-29
    pub fn new(min: f64, max: f64) -> Self {
        Self { min, max }
    }

    /// Get the minimum value.
    /// Optimizer.hpp:31
    pub fn min(&self) -> f64 {
        self.min
    }

    /// Get the maximum value.
    /// Optimizer.hpp:32
    pub fn max(&self) -> f64 {
        self.max
    }
}

impl Default for Bound {
    /// Default bound: [numeric_limits<double>::min(), numeric_limits<double>::max()].
    /// NOTE: `std::numeric_limits<double>::min()` is the smallest *positive*
    /// normalized value (~2.2e-308), which maps to `f64::MIN_POSITIVE` in Rust,
    /// NOT `f64::MIN` (the most negative value).
    /// Optimizer.hpp:27-28
    fn default() -> Self {
        Self {
            min: f64::MIN_POSITIVE,
            max: f64::MAX,
        }
    }
}

/// Helper type aliases for optimization input and bounds.
///
/// Optimizer.hpp:37-38
pub type Input<const N: usize> = [f64; N];
pub type Bounds<const N: usize> = [Bound; N];

/// Stopping criteria for optimization.
///
/// Setter methods return `&mut Self` for method chaining.
///
/// Optimizer.hpp:41-95
pub struct StopCriteria {
    /// Absolute score difference threshold
    /// Optimizer.hpp:44
    abs_score_diff: f64,

    /// Relative score difference threshold
    /// Optimizer.hpp:47
    rel_score_diff: f64,

    /// Stop if this score or better is found
    /// Optimizer.hpp:50
    stop_score: f64,

    /// Predicate that triggers early termination
    /// Optimizer.hpp:54
    stop_condition: Option<std::sync::Arc<dyn Fn() -> bool + Send + Sync>>,

    /// Maximum number of iterations (0 = unlimited)
    /// Optimizer.hpp:57
    max_iterations: u32,
}

impl Clone for StopCriteria {
    fn clone(&self) -> Self {
        Self {
            abs_score_diff: self.abs_score_diff,
            rel_score_diff: self.rel_score_diff,
            stop_score: self.stop_score,
            stop_condition: self.stop_condition.clone(),
            max_iterations: self.max_iterations,
        }
    }
}

impl StopCriteria {
    /// Create default stop criteria.
    pub fn new() -> Self {
        Self::default()
    }

    /// Set the absolute score difference threshold.
    /// Optimizer.hpp:61-63
    pub fn abs_score_diff(&mut self, val: f64) -> &mut Self {
        self.abs_score_diff = val;
        self
    }

    /// Get the absolute score difference threshold.
    /// Optimizer.hpp:65
    pub fn get_abs_score_diff(&self) -> f64 {
        self.abs_score_diff
    }

    /// Set the relative score difference threshold.
    /// Optimizer.hpp:67-69
    pub fn rel_score_diff(&mut self, val: f64) -> &mut Self {
        self.rel_score_diff = val;
        self
    }

    /// Get the relative score difference threshold.
    /// Optimizer.hpp:71
    pub fn get_rel_score_diff(&self) -> f64 {
        self.rel_score_diff
    }

    /// Set the stop score threshold.
    /// Optimizer.hpp:73-75
    pub fn stop_score(&mut self, val: f64) -> &mut Self {
        self.stop_score = val;
        self
    }

    /// Get the stop score threshold.
    /// Optimizer.hpp:77
    pub fn get_stop_score(&self) -> f64 {
        self.stop_score
    }

    /// Set the maximum number of iterations.
    /// In C++ the parameter is `double val` and it is assigned into the
    /// `unsigned m_max_iterations` member, truncating toward zero.
    /// Optimizer.hpp:82-85
    pub fn max_iterations(&mut self, val: f64) -> &mut Self {
        self.max_iterations = val as u32;
        self
    }

    /// Get the maximum number of iterations.
    /// In C++ this getter returns `double` despite the member being `unsigned`.
    /// Optimizer.hpp:87
    pub fn get_max_iterations(&self) -> f64 {
        self.max_iterations as f64
    }

    /// Set the stop condition predicate.
    /// Optimizer.hpp:85-88
    pub fn set_stop_condition<F: Fn() -> bool + Send + Sync + 'static>(
        &mut self,
        cond: F,
    ) -> &mut Self {
        self.stop_condition = Some(std::sync::Arc::new(cond));
        self
    }

    /// Evaluate the stop condition. Returns true if optimization should stop.
    /// Optimizer.hpp:90
    pub fn check_stop_condition(&self) -> bool {
        match &self.stop_condition {
            Some(cond) => cond(),
            None => false,
        }
    }
}

impl Default for StopCriteria {
    fn default() -> Self {
        Self {
            abs_score_diff: f64::NAN,
            rel_score_diff: f64::NAN,
            stop_score: f64::NAN,
            stop_condition: None,
            max_iterations: 0,
        }
    }
}

impl std::fmt::Debug for StopCriteria {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("StopCriteria")
            .field("abs_score_diff", &self.abs_score_diff)
            .field("rel_score_diff", &self.rel_score_diff)
            .field("stop_score", &self.stop_score)
            .field("max_iterations", &self.max_iterations)
            .field(
                "stop_condition",
                &self.stop_condition.as_ref().map(|_| "..."),
            )
            .finish()
    }
}

/// Score + optional gradient for gradient-based optimization methods.
///
/// Optimizer.hpp:98-105
#[derive(Debug, Clone)]
pub struct ScoreGradient<const N: usize> {
    /// The function score at the evaluation point
    /// Optimizer.hpp:99
    pub score: f64,

    /// Optional gradient vector
    /// Optimizer.hpp:100
    pub gradient: Option<[f64; N]>,
}

impl<const N: usize> ScoreGradient<N> {
    /// Create a ScoreGradient with score and gradient.
    /// Optimizer.hpp:102-104
    pub fn new(score: f64, gradient: [f64; N]) -> Self {
        Self {
            score,
            gradient: Some(gradient),
        }
    }

    /// Create a ScoreGradient with just a score (no gradient).
    pub fn score_only(score: f64) -> Self {
        Self {
            score,
            gradient: None,
        }
    }
}

/// Direction of optimization.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OptDir {
    Min,
    Max,
}

// Optimizer.hpp:107-108
//   template<class T> struct always_false { enum { value = false }; };
// This is a C++ template-metaprogramming helper used solely inside the
// unimplemented primary `Optimizer` template (Optimizer.hpp:114-118) so that
// `static_assert(always_false<Method>::value, ...)` fires only when the base
// template is instantiated for a method that has no specialization. In Rust the
// equivalent guarantee is provided by the trait system itself: a type that does
// not implement the `Optimizer` trait simply fails to compile at the use site,
// so there is no runtime/compile helper to materialize here.

// Optimizer.hpp:110-153 — the unimplemented primary template
//   template<class Method, class Enable = void> class Optimizer { ... };
// Its members (to_min/to_max returning *this, set_criteria, get_criteria,
// optimize returning {}, seed) exist only to produce the static_assert error.
// The faithful Rust representation is the `Optimizer` trait below: concrete
// methods (Bruteforce / NLopt) are the C++ partial specializations.

/// Generic optimizer trait.
///
/// Optimizer.hpp:111-153
pub trait Optimizer<const N: usize> {
    /// Switch optimization towards function minimum.
    /// Optimizer.hpp:121
    fn to_min(&mut self) -> &mut Self;

    /// Switch optimization towards function maximum.
    /// Optimizer.hpp:124
    fn to_max(&mut self) -> &mut Self;

    /// Set criteria for the optimization.
    /// Optimizer.hpp:127
    fn set_criteria(&mut self, criteria: StopCriteria) -> &mut Self;

    /// Get the current criteria.
    /// Optimizer.hpp:130
    fn get_criteria(&self) -> &StopCriteria;

    /// Find function minimum or maximum.
    ///
    /// `func` takes an `Input<N>` and returns a `f64` score.
    /// `initvals` is the starting point.
    /// `bounds` defines the valid range for each dimension.
    ///
    /// Optimizer.hpp:146-149
    fn optimize<F>(&mut self, func: F, initvals: &Input<N>, bounds: &Bounds<N>) -> OptResult<N>
    where
        F: Fn(&Input<N>) -> f64;

    /// Set the random seed (optional for randomized methods).
    /// Optimizer.hpp:152
    fn seed(&mut self, _s: u64) {}
}

// Optimizer.hpp:155-171 — namespace detail
//   template<size_t N, class T> auto to_arr(const T *a) { ... std::copy ... }
//   template<size_t N, class T> auto to_arr(const T (&a) [N]) { ... }
// These convert a C-style array into a std::array (the header notes "The copy
// should be optimized away with modern compilers"). In Rust, `[T; N]` is
// already a value type, so the `bounds`/`initvals`/`score_gradient` helpers
// below accept `[T; N]` directly and no `to_arr` conversion is required.

/// Helper to create bounds from an array.
///
/// Optimizer.hpp:174
pub fn bounds<const N: usize>(b: [Bound; N]) -> Bounds<N> {
    b
}

/// Helper to create initial values from an array.
///
/// Optimizer.hpp:175
pub fn initvals<const N: usize>(a: [f64; N]) -> Input<N> {
    a
}

/// Helper to create a ScoreGradient.
///
/// Optimizer.hpp:176-179
pub fn score_gradient<const N: usize>(score: f64, grad: [f64; N]) -> ScoreGradient<N> {
    ScoreGradient::new(score, grad)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_bound_default() {
        let b = Bound::default();
        // C++ std::numeric_limits<double>::min() == smallest positive normal.
        assert_eq!(b.min(), f64::MIN_POSITIVE);
        assert_eq!(b.max(), f64::MAX);
    }

    #[test]
    fn test_bound_custom() {
        let b = Bound::new(-1.0, 1.0);
        assert_eq!(b.min(), -1.0);
        assert_eq!(b.max(), 1.0);
    }

    #[test]
    fn test_stop_criteria_default() {
        let sc = StopCriteria::default();
        assert!(sc.get_abs_score_diff().is_nan());
        assert!(sc.get_rel_score_diff().is_nan());
        assert!(sc.get_stop_score().is_nan());
        assert_eq!(sc.get_max_iterations(), 0);
        assert!(!sc.check_stop_condition());
    }

    #[test]
    fn test_stop_criteria_chaining() {
        let mut sc = StopCriteria::new();
        sc.abs_score_diff(0.001)
            .rel_score_diff(0.01)
            .stop_score(-100.0)
            .max_iterations(1000);

        assert_eq!(sc.get_abs_score_diff(), 0.001);
        assert_eq!(sc.get_rel_score_diff(), 0.01);
        assert_eq!(sc.get_stop_score(), -100.0);
        assert_eq!(sc.get_max_iterations(), 1000);
    }

    #[test]
    fn test_stop_condition() {
        let mut sc = StopCriteria::new();
        sc.set_stop_condition(|| true);
        assert!(sc.check_stop_condition());
    }

    #[test]
    fn test_opt_result_default() {
        let r: OptResult<3> = OptResult::default();
        assert_eq!(r.result_code, 0);
        assert_eq!(r.optimum, [0.0; 3]);
        assert_eq!(r.score, 0.0);
    }

    #[test]
    fn test_score_gradient() {
        let sg: ScoreGradient<2> = ScoreGradient::new(1.5, [0.1, 0.2]);
        assert_eq!(sg.score, 1.5);
        assert_eq!(sg.gradient.unwrap(), [0.1, 0.2]);
    }

    #[test]
    fn test_helpers() {
        let b = bounds([Bound::new(0.0, 1.0), Bound::new(-1.0, 2.0)]);
        assert_eq!(b[0].min(), 0.0);
        assert_eq!(b[1].max(), 2.0);

        let iv = initvals([0.5, 0.5]);
        assert_eq!(iv, [0.5, 0.5]);
    }
}
