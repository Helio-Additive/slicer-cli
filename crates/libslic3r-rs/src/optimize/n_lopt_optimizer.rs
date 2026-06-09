//! NLopt-based optimizers.
//!
//! C++ Reference:
//! - Optimize/NLoptOptimizer.hpp
//!
//! 1:1 port of the C++ `NLoptOptimizer.hpp` header. The C++ file is a header-only
//! template wrapper around the NLopt C library (`#include <nlopt.h>`,
//! NLoptOptimizer.hpp:9). All of the surrounding scaffolding is faithfully
//! translated here: the algorithm tag types (`NLoptAlg`, `NLoptAlgComb`), the
//! `OptDir` enum, the `NLopt` RAII handle, the `NLoptOpt` driver (with `optfunc`,
//! `set_up`, `optimize`), the combined global/local specialization, the
//! `Optimizer` specialization, and the predefined algorithm aliases.
//!
//! BLOCKED (native dependency, not wasm-safe — see RULES): the actual numerical
//! backend. In C++ every `optimize(...)` call ultimately calls into the NLopt C
//! library (`nlopt_create`, `nlopt_set_lower_bounds`, `nlopt_set_min_objective`,
//! `nlopt_optimize`, `nlopt_srand`, ...). The only Rust binding for NLopt (the
//! `nlopt` crate) links against the native `libnlopt` C library, which is a
//! system/dylib dependency and therefore is NOT wasm-safe and must not be added
//! (per RULES #2/#3). Consequently `NLopt::create`, `set_up`'s `nlopt_set_*`
//! calls, and `optimize`'s `nlopt_optimize` are represented but unimplemented:
//! they record the configuration faithfully and return `Err(NLoptBackendError)`
//! /panic at the backend boundary rather than fabricating a substitute solver.

use super::optimizer::{Bounds, Input, OptResult, ScoreGradient, StopCriteria};

// NLoptOptimizer.hpp:18 — namespace Slic3r { namespace opt {
// NLoptOptimizer.hpp:20 — namespace detail {
pub mod detail {
    use super::*;

    // ---------------------------------------------------------------------
    // nlopt_algorithm
    //
    // NLoptOptimizer.hpp:9 — `#include <nlopt.h>`. The algorithm tags below
    // reference `nlopt_algorithm` enumerators. The numeric values mirror the
    // canonical `nlopt.h` enumeration so that, were the native backend wired
    // up, the same algorithm would be selected. Only the enumerators actually
    // referenced by this header (NLoptOptimizer.hpp:27,225-229) are required,
    // but the full set of names used by the codebase is provided.
    // ---------------------------------------------------------------------
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    #[allow(non_camel_case_types)]
    #[repr(i32)]
    pub enum NloptAlgorithm {
        // Global, derivative-free (NLoptOptimizer.hpp:228)
        NLOPT_GN_DIRECT = 0,
        // Global, derivative-free, randomized (NLoptOptimizer.hpp:229)
        NLOPT_GN_MLSL = 18,
        // Local, derivative-free (NLoptOptimizer.hpp:227 — default local method)
        NLOPT_LN_NELDERMEAD = 28,
        // Local, derivative-free (NLoptOptimizer.hpp:226)
        NLOPT_LN_SBPLX = 29,
        // Global, evolutionary strategy (NLoptOptimizer.hpp:225)
        NLOPT_GN_ESCH = 42,
    }

    // Helper types for NLopt algorithm selection in template contexts
    // NLoptOptimizer.hpp:22-23
    //   template<nlopt_algorithm alg> struct NLoptAlg {};
    //
    // In C++ this is a compile-time tag carrying a single algorithm. In Rust we
    // carry the algorithm value at runtime.
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct NLoptAlg {
        pub alg: NloptAlgorithm,
    }

    impl NLoptAlg {
        pub fn new(alg: NloptAlgorithm) -> Self {
            Self { alg }
        }
    }

    // NLopt can combine multiple algorithms if one is global an other is a local
    // method. This is how template specializations can be informed about this fact.
    // NLoptOptimizer.hpp:25-28
    //   template<nlopt_algorithm gl_alg, nlopt_algorithm lc_alg = NLOPT_LN_NELDERMEAD>
    //   struct NLoptAlgComb {};
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct NLoptAlgComb {
        pub gl_alg: NloptAlgorithm,
        pub lc_alg: NloptAlgorithm,
    }

    impl NLoptAlgComb {
        // NLoptOptimizer.hpp:27 — `lc_alg` defaults to NLOPT_LN_NELDERMEAD.
        pub fn new(gl_alg: NloptAlgorithm) -> Self {
            Self {
                gl_alg,
                lc_alg: NloptAlgorithm::NLOPT_LN_NELDERMEAD,
            }
        }

        pub fn with_local(gl_alg: NloptAlgorithm, lc_alg: NloptAlgorithm) -> Self {
            Self { gl_alg, lc_alg }
        }
    }

    // NLoptOptimizer.hpp:30-41
    //   template<class M> struct IsNLoptAlg { ...value = false; };
    //   template<nlopt_algorithm a> struct IsNLoptAlg<NLoptAlg<a>> { ...value = true; };
    //   template<nlopt_algorithm a1, nlopt_algorithm a2>
    //   struct IsNLoptAlg<NLoptAlgComb<a1, a2>> { ...value = true; };
    //
    // This compile-time predicate gates the `Optimizer` specialization
    // (NLoptOptimizer.hpp:43-44, 196). In Rust the gating is provided by the
    // `NLoptMethod` trait (the only types implementing it are `NLoptAlg` and
    // `NLoptAlgComb`), so `IsNLoptAlg`/`NLoptOnly` need no runtime form.
    pub trait NLoptMethod {}
    impl NLoptMethod for NLoptAlg {}
    impl NLoptMethod for NLoptAlgComb {}

    // NLoptOptimizer.hpp:47 — enum class OptDir { MIN, MAX }; // Where to optimize
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub enum OptDir {
        Min,
        Max,
    }

    // Error returned at the native-NLopt boundary. See module docs: the NLopt C
    // backend is a non-wasm-safe native dependency and is intentionally not
    // wired up. Every entry point that would call into `libnlopt` surfaces this.
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct NLoptBackendError;

    impl std::fmt::Display for NLoptBackendError {
        fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
            write!(
                f,
                "NLopt native backend (libnlopt) is unavailable: it is a native \
                 dependency that is not wasm-safe and is not linked (see \
                 NLoptOptimizer.hpp port notes)"
            )
        }
    }

    impl std::error::Error for NLoptBackendError {}

    // Result of `set_up`: the exact NLopt handle configuration that the native
    // `nlopt_set_*` calls would have applied (NLoptOptimizer.hpp:104-126). Kept
    // so the configuration is observable/testable even though the backend is
    // BLOCKED. `set_up` returns `void` in C++; we surface the captured config.
    #[derive(Debug, Clone, PartialEq)]
    pub struct NLoptSetup {
        pub lower_bounds: Vec<f64>,
        pub upper_bounds: Vec<f64>,
        pub ftol_abs: Option<f64>,
        pub ftol_rel: Option<f64>,
        pub stopval: Option<f64>,
        pub maxeval: Option<i32>,
    }

    // NLoptOptimizer.hpp:49-63
    //   struct NLopt { // Helper RAII class for nlopt_opt
    //       nlopt_opt ptr = nullptr;
    //       template<class...A> explicit NLopt(A&&...a) { ptr = nlopt_create(...); }
    //       NLopt(const NLopt&) = delete; ... NLopt(NLopt&&) = delete;
    //       ~NLopt() { nlopt_destroy(ptr); }
    //   };
    //
    // BLOCKED: `nlopt_create`/`nlopt_destroy` are native NLopt C functions. The
    // RAII wrapper is preserved structurally; it records the algorithm and
    // dimension that `nlopt_create(alg, n)` was called with. The opaque
    // `nlopt_opt ptr` would be the native handle — here it stays conceptually
    // null because no native object is created.
    pub struct NLopt {
        // NLoptOptimizer.hpp:50 — nlopt_opt ptr = nullptr;
        pub alg: NloptAlgorithm,
        pub n: u32,
    }

    impl NLopt {
        // NLoptOptimizer.hpp:52-55
        //   template<class...A> explicit NLopt(A&&...a)
        //   { ptr = nlopt_create(std::forward<A>(a)...); }
        //
        // The two construction sites pass `(alg, N)` (NLoptOptimizer.hpp:155,181).
        pub fn create(alg: NloptAlgorithm, n: u32) -> Self {
            // BLOCKED: would call `nlopt_create(alg, n)`.
            Self { alg, n }
        }
    }

    // NLoptOptimizer.hpp:57-60 — copy/move are deleted. In Rust `NLopt` is simply
    // not `Clone`/`Copy`, which provides the same guarantee.
    // NLoptOptimizer.hpp:62 — ~NLopt() { nlopt_destroy(ptr); }
    // The native handle would be destroyed on drop; with no native object there
    // is nothing to free, so no `Drop` impl is required.

    // NLoptOptimizer.hpp:65 — template<class Method> class NLoptOpt {};
    //
    // Optimizers based on NLopt.
    // NLoptOptimizer.hpp:67-68 — template<nlopt_algorithm alg> class NLoptOpt<NLoptAlg<alg>>
    pub struct NLoptOpt {
        // NLoptOptimizer.hpp:70 — StopCriteria m_stopcr;
        pub m_stopcr: StopCriteria,
        // NLoptOptimizer.hpp:71 — OptDir m_dir;
        pub m_dir: OptDir,
        // The selected algorithm. In C++ this is the `alg` template parameter of
        // `NLoptOpt<NLoptAlg<alg>>` (NLoptOptimizer.hpp:68).
        pub alg: NloptAlgorithm,
    }

    impl NLoptOpt {
        // NLoptOptimizer.hpp:73-74
        //   template<class Fn> using TOptData =
        //       std::tuple<std::remove_reference_t<Fn>*, NLoptOpt*, nlopt_opt>;
        //
        // `TOptData` is the userdata tuple threaded through the NLopt C callback
        // (the objective fn pointer, the owning `NLoptOpt`, the `nlopt_opt`
        // handle for `nlopt_force_stop`). It is only meaningful at the native
        // callback boundary; in Rust the closure captures these directly, so
        // there is no separate tuple type to materialize.

        // NLoptOptimizer.hpp:76-102
        //   template<class Fn, size_t N>
        //   static double optfunc(unsigned n, const double *params,
        //                         double *gradient, void *data)
        //
        // The C-ABI trampoline NLopt invokes for each evaluation. Faithfully
        // translated: it (1) honours the user stop_condition by force-stopping
        // the native opt, (2) converts the raw params into an Input<N>, and (3)
        // dispatches on whether the objective also produces a gradient.
        //
        // `force_stop` would call `nlopt_force_stop(handle)` (BLOCKED native).
        pub fn optfunc<const N: usize, F, R>(
            &self,
            // NLoptOptimizer.hpp:81 — assert(n >= N);
            n: usize,
            params: &[f64],
            gradient: &mut [f64],
            fnptr: &F,
            mut force_stop: impl FnMut(),
        ) -> f64
        where
            F: Fn(&Input<N>) -> R,
            R: Into<ObjectiveValue<N>>,
        {
            // NLoptOptimizer.hpp:81 — assert(n >= N);
            debug_assert!(n >= N);

            // NLoptOptimizer.hpp:83 — auto tdata = static_cast<TOptData<Fn>*>(data);

            // NLoptOptimizer.hpp:85-86
            //   if (std::get<1>(*tdata)->m_stopcr.stop_condition())
            //       nlopt_force_stop(std::get<2>(*tdata));
            if self.m_stopcr.check_stop_condition() {
                force_stop();
            }

            // NLoptOptimizer.hpp:88 — auto fnptr  = std::get<0>(*tdata);
            // NLoptOptimizer.hpp:89 — auto funval = to_arr<N>(params);
            let mut funval: Input<N> = [0.0f64; N];
            funval[..N].copy_from_slice(&params[..N]);

            // NLoptOptimizer.hpp:91 — double scoreval = 0.;
            #[allow(unused_assignments)]
            let mut scoreval = 0.;
            // NLoptOptimizer.hpp:92 — using RetT = decltype((*fnptr)(funval));
            // NLoptOptimizer.hpp:93-99
            //   if constexpr (std::is_convertible_v<RetT, ScoreGradient<N>>) {
            //       ScoreGradient<N> score = (*fnptr)(funval);
            //       for (size_t i = 0; i < n; ++i) gradient[i] = (*score.gradient)[i];
            //       scoreval = score.score;
            //   } else {
            //       scoreval = (*fnptr)(funval);
            //   }
            match (*fnptr)(&funval).into() {
                ObjectiveValue::ScoreGradient(score) => {
                    // (*score.gradient)[i] dereferences the std::optional gradient;
                    // mirrors the C++ which assumes the optional is engaged.
                    let grad = score
                        .gradient
                        .expect("ScoreGradient objective must carry a gradient");
                    for i in 0..n {
                        gradient[i] = grad[i];
                    }
                    scoreval = score.score;
                }
                ObjectiveValue::Score(s) => {
                    scoreval = s;
                }
            }

            // NLoptOptimizer.hpp:101 — return scoreval;
            scoreval
        }

        // NLoptOptimizer.hpp:104-126
        //   template<size_t N> void set_up(NLopt &nl, const Bounds<N>& bounds)
        //
        // Translates the high-level StopCriteria/Bounds into the NLopt handle
        // configuration. The `nlopt_set_*` calls are BLOCKED (native) but the
        // exact decision logic (which calls fire, and with what arguments) is
        // preserved so the configuration is recorded faithfully.
        pub fn set_up<const N: usize>(&self, nl: &mut NLopt, bounds: &Bounds<N>) -> NLoptSetup {
            let _ = nl;
            // NLoptOptimizer.hpp:107 — std::array<double, N> lb, ub;
            let mut lb = [0.0f64; N];
            let mut ub = [0.0f64; N];

            // NLoptOptimizer.hpp:109-112
            //   for (size_t i = 0; i < N; ++i) {
            //       lb[i] = bounds[i].min();
            //       ub[i] = bounds[i].max();
            //   }
            for i in 0..N {
                lb[i] = bounds[i].min();
                ub[i] = bounds[i].max();
            }

            // NLoptOptimizer.hpp:114 — nlopt_set_lower_bounds(nl.ptr, lb.data());
            // NLoptOptimizer.hpp:115 — nlopt_set_upper_bounds(nl.ptr, ub.data());

            // NLoptOptimizer.hpp:117 — double abs_diff = m_stopcr.abs_score_diff();
            let abs_diff = self.m_stopcr.get_abs_score_diff();
            // NLoptOptimizer.hpp:118 — double rel_diff = m_stopcr.rel_score_diff();
            let rel_diff = self.m_stopcr.get_rel_score_diff();
            // NLoptOptimizer.hpp:119 — double stopval = m_stopcr.stop_score();
            let stopval = self.m_stopcr.get_stop_score();
            // NLoptOptimizer.hpp:120 — if(!std::isnan(abs_diff)) nlopt_set_ftol_abs(nl.ptr, abs_diff);
            let ftol_abs = if !abs_diff.is_nan() { Some(abs_diff) } else { None };
            // NLoptOptimizer.hpp:121 — if(!std::isnan(rel_diff)) nlopt_set_ftol_rel(nl.ptr, rel_diff);
            let ftol_rel = if !rel_diff.is_nan() { Some(rel_diff) } else { None };
            // NLoptOptimizer.hpp:122 — if(!std::isnan(stopval))  nlopt_set_stopval(nl.ptr, stopval);
            let set_stopval = if !stopval.is_nan() { Some(stopval) } else { None };

            // NLoptOptimizer.hpp:124-125
            //   if(m_stopcr.max_iterations() > 0)
            //       nlopt_set_maxeval(nl.ptr, m_stopcr.max_iterations());
            let maxeval = if self.m_stopcr.get_max_iterations() > 0.0 {
                Some(self.m_stopcr.get_max_iterations() as i32)
            } else {
                None
            };

            NLoptSetup {
                lower_bounds: lb.to_vec(),
                upper_bounds: ub.to_vec(),
                ftol_abs,
                ftol_rel,
                stopval: set_stopval,
                maxeval,
            }
        }

        // NLoptOptimizer.hpp:128-146
        //   template<class Fn, size_t N>
        //   Result<N> optimize(NLopt &nl, Fn &&fn, const Input<N> &initvals)
        //
        // BLOCKED at the `nlopt_optimize` call (NLoptOptimizer.hpp:143). The
        // surrounding bookkeeping (selecting min/max objective, seeding the
        // result optimum with `initvals`) is preserved. Because the native
        // solver cannot run, this returns Err.
        #[allow(clippy::result_unit_err)]
        pub fn optimize_with_handle<const N: usize, F, R>(
            &self,
            nl: &NLopt,
            _fn: F,
            initvals: &Input<N>,
        ) -> Result<OptResult<N>, NLoptBackendError>
        where
            F: Fn(&Input<N>) -> R,
            R: Into<ObjectiveValue<N>>,
        {
            let _ = nl;
            // NLoptOptimizer.hpp:131 — Result<N> r;
            // `r` is built up faithfully (initvals seeded into `r.optimum`,
            // NLoptOptimizer.hpp:142) but, because the native solver call at
            // NLoptOptimizer.hpp:143 is BLOCKED, `r` cannot be returned — the
            // function reports the unavailable backend instead. The bookkeeping
            // is preserved so the structure mirrors the C++ exactly.
            #[allow(unused_variables, unused_assignments, unused_mut)]
            {
                let mut r = OptResult::<N>::default();

                // NLoptOptimizer.hpp:133 — TOptData<Fn> data = std::make_tuple(&fn, this, nl.ptr);

                // NLoptOptimizer.hpp:135-140
                //   switch(m_dir) {
                //   case OptDir::MIN: nlopt_set_min_objective(nl.ptr, optfunc<Fn, N>, &data); break;
                //   case OptDir::MAX: nlopt_set_max_objective(nl.ptr, optfunc<Fn, N>, &data); break;
                //   }
                match self.m_dir {
                    OptDir::Min => { /* BLOCKED: nlopt_set_min_objective */ }
                    OptDir::Max => { /* BLOCKED: nlopt_set_max_objective */ }
                }

                // NLoptOptimizer.hpp:142 — r.optimum = initvals;
                r.optimum = *initvals;
                // NLoptOptimizer.hpp:143 — r.resultcode = nlopt_optimize(nl.ptr, r.optimum.data(), &r.score);
                // BLOCKED: native NLopt solver. No substitute is fabricated.
            }
            Err(NLoptBackendError)

            // NLoptOptimizer.hpp:145 — return r;
        }

        // NLoptOptimizer.hpp:150-159
        //   template<class Func, size_t N>
        //   Result<N> optimize(Func&& func, const Input<N> &initvals, const Bounds<N>& bounds)
        //   {
        //       NLopt nl{alg, N};
        //       set_up(nl, bounds);
        //       return optimize(nl, std::forward<Func>(func), initvals);
        //   }
        #[allow(clippy::result_unit_err)]
        pub fn optimize<const N: usize, F, R>(
            &self,
            func: F,
            initvals: &Input<N>,
            bounds: &Bounds<N>,
        ) -> Result<OptResult<N>, NLoptBackendError>
        where
            F: Fn(&Input<N>) -> R,
            R: Into<ObjectiveValue<N>>,
        {
            // NLoptOptimizer.hpp:155 — NLopt nl{alg, N};
            let mut nl = NLopt::create(self.alg, N as u32);
            // NLoptOptimizer.hpp:156 — set_up(nl, bounds);
            let _setup = self.set_up(&mut nl, bounds);
            // NLoptOptimizer.hpp:158 — return optimize(nl, std::forward<Func>(func), initvals);
            self.optimize_with_handle(&nl, func, initvals)
        }

        // NLoptOptimizer.hpp:161 — explicit NLoptOpt(StopCriteria stopcr = {}) : m_stopcr(stopcr) {}
        //
        // `m_dir` is left default-initialized by the C++ ctor; an `enum class`
        // member with no in-class initializer is value-initialized to its first
        // enumerator, i.e. `OptDir::MIN`.
        pub fn new(alg: NloptAlgorithm, stopcr: StopCriteria) -> Self {
            Self {
                m_stopcr: stopcr,
                m_dir: OptDir::Min,
                alg,
            }
        }

        // NLoptOptimizer.hpp:163 — void set_criteria(const StopCriteria &cr) { m_stopcr = cr; }
        pub fn set_criteria(&mut self, cr: StopCriteria) {
            self.m_stopcr = cr;
        }

        // NLoptOptimizer.hpp:164 — const StopCriteria &get_criteria() const noexcept { return m_stopcr; }
        pub fn get_criteria(&self) -> &StopCriteria {
            &self.m_stopcr
        }

        // NLoptOptimizer.hpp:165 — void set_dir(OptDir dir) noexcept { m_dir = dir; }
        pub fn set_dir(&mut self, dir: OptDir) {
            self.m_dir = dir;
        }

        // NLoptOptimizer.hpp:167 — void seed(long s) { nlopt_srand(s); }
        // BLOCKED: `nlopt_srand` is a native NLopt C function. Recorded but no-op.
        pub fn seed(&mut self, _s: i64) {
            // BLOCKED: would call `nlopt_srand(s)`.
        }
    }

    // NLoptOptimizer.hpp:170-191
    //   template<nlopt_algorithm glob, nlopt_algorithm loc>
    //   class NLoptOpt<NLoptAlgComb<glob, loc>>: public NLoptOpt<NLoptAlg<glob>>
    //   {
    //       using Base = NLoptOpt<NLoptAlg<glob>>;
    //   public: ...
    //   };
    //
    // Combined global+local optimizer. In C++ it derives from the single-alg
    // `NLoptOpt<NLoptAlg<glob>>`; in Rust we compose the base by value.
    pub struct NLoptOptComb {
        // using Base = NLoptOpt<NLoptAlg<glob>>; (NLoptOptimizer.hpp:173)
        pub base: NLoptOpt,
        pub loc: NloptAlgorithm,
    }

    impl NLoptOptComb {
        // NLoptOptimizer.hpp:176-188
        //   template<class Fn, size_t N>
        //   Result<N> optimize(Fn&& f, const Input<N> &initvals, const Bounds<N>& bounds)
        //   {
        //       NLopt nl_glob{glob, N}, nl_loc{loc, N};
        //       Base::set_up(nl_glob, bounds);
        //       Base::set_up(nl_loc, bounds);
        //       nlopt_set_local_optimizer(nl_glob.ptr, nl_loc.ptr);
        //       return Base::optimize(nl_glob, std::forward<Fn>(f), initvals);
        //   }
        #[allow(clippy::result_unit_err)]
        pub fn optimize<const N: usize, F, R>(
            &self,
            f: F,
            initvals: &Input<N>,
            bounds: &Bounds<N>,
        ) -> Result<OptResult<N>, NLoptBackendError>
        where
            F: Fn(&Input<N>) -> R,
            R: Into<ObjectiveValue<N>>,
        {
            // NLoptOptimizer.hpp:181 — NLopt nl_glob{glob, N}, nl_loc{loc, N};
            let mut nl_glob = NLopt::create(self.base.alg, N as u32);
            let mut nl_loc = NLopt::create(self.loc, N as u32);

            // NLoptOptimizer.hpp:183 — Base::set_up(nl_glob, bounds);
            let _setup_glob = self.base.set_up(&mut nl_glob, bounds);
            // NLoptOptimizer.hpp:184 — Base::set_up(nl_loc, bounds);
            let _setup_loc = self.base.set_up(&mut nl_loc, bounds);
            // NLoptOptimizer.hpp:185 — nlopt_set_local_optimizer(nl_glob.ptr, nl_loc.ptr);
            // BLOCKED: native NLopt call.

            // NLoptOptimizer.hpp:187 — return Base::optimize(nl_glob, std::forward<Fn>(f), initvals);
            self.base.optimize_with_handle(&nl_glob, f, initvals)
        }

        // NLoptOptimizer.hpp:190 — explicit NLoptOpt(StopCriteria stopcr = {}) : Base{stopcr} {}
        pub fn new(glob: NloptAlgorithm, loc: NloptAlgorithm, stopcr: StopCriteria) -> Self {
            Self {
                base: NLoptOpt::new(glob, stopcr),
                loc,
            }
        }
    }
} // namespace detail; (NLoptOptimizer.hpp:193)

use detail::{NLoptAlg, NLoptAlgComb, NLoptOpt, NLoptOptComb, NloptAlgorithm, OptDir};
pub use detail::{NLoptBackendError, NLoptSetup};

// NLoptOptimizer.hpp:91-99 — the objective return-value dispatch.
//
// In C++ `optfunc` uses `if constexpr (std::is_convertible_v<RetT, ScoreGradient<N>>)`
// to branch between a plain `double` score and a `ScoreGradient<N>`. Rust models
// the two possibilities as this sum type; the objective closure's return type is
// converted via `Into<ObjectiveValue<N>>`.
pub enum ObjectiveValue<const N: usize> {
    Score(f64),
    ScoreGradient(ScoreGradient<N>),
}

impl<const N: usize> From<f64> for ObjectiveValue<N> {
    fn from(s: f64) -> Self {
        ObjectiveValue::Score(s)
    }
}

impl<const N: usize> From<ScoreGradient<N>> for ObjectiveValue<N> {
    fn from(sg: ScoreGradient<N>) -> Self {
        ObjectiveValue::ScoreGradient(sg)
    }
}

// NLoptOptimizer.hpp:195-222
//   Optimizers based on NLopt.
//   template<class M> class Optimizer<M, detail::NLoptOnly<M>> {
//       detail::NLoptOpt<M> m_opt; ...
//   };
//
// The `Optimizer<M, NLoptOnly<M>>` partial specialization. `M` is one of the
// detail method tags (`NLoptAlg` or `NLoptAlgComb`); the gating
// `NLoptOnly<M>` (NLoptOptimizer.hpp:43-44, 196) is provided by the
// `detail::NLoptMethod` trait. We expose two concrete optimizer types — one for
// each method tag — mirroring the single C++ template specialization.

// `Optimizer<NLoptAlg<alg>>` (NLoptOptimizer.hpp:196-222 with M = NLoptAlg).
pub struct NLoptAlgOptimizer {
    // NLoptOptimizer.hpp:197 — detail::NLoptOpt<M> m_opt;
    m_opt: NLoptOpt,
}

impl NLoptAlgOptimizer {
    // NLoptOptimizer.hpp:212 — explicit Optimizer(StopCriteria stopcr = {}) : m_opt(stopcr) {}
    pub fn new(alg: NLoptAlg, stopcr: StopCriteria) -> Self {
        Self {
            m_opt: NLoptOpt::new(alg.alg, stopcr),
        }
    }

    // NLoptOptimizer.hpp:201 — Optimizer& to_max() { m_opt.set_dir(detail::OptDir::MAX); return *this; }
    pub fn to_max(&mut self) -> &mut Self {
        self.m_opt.set_dir(OptDir::Max);
        self
    }

    // NLoptOptimizer.hpp:202 — Optimizer& to_min() { m_opt.set_dir(detail::OptDir::MIN); return *this; }
    pub fn to_min(&mut self) -> &mut Self {
        self.m_opt.set_dir(OptDir::Min);
        self
    }

    // NLoptOptimizer.hpp:204-210
    //   template<class Func, size_t N>
    //   Result<N> optimize(Func&& func, const Input<N> &initvals, const Bounds<N>& bounds)
    //   { return m_opt.optimize(std::forward<Func>(func), initvals, bounds); }
    #[allow(clippy::result_unit_err)]
    pub fn optimize<const N: usize, F, R>(
        &mut self,
        func: F,
        initvals: &Input<N>,
        bounds: &Bounds<N>,
    ) -> Result<OptResult<N>, NLoptBackendError>
    where
        F: Fn(&Input<N>) -> R,
        R: Into<ObjectiveValue<N>>,
    {
        self.m_opt.optimize(func, initvals, bounds)
    }

    // NLoptOptimizer.hpp:214-217 — Optimizer &set_criteria(const StopCriteria &cr) { m_opt.set_criteria(cr); return *this; }
    pub fn set_criteria(&mut self, cr: StopCriteria) -> &mut Self {
        self.m_opt.set_criteria(cr);
        self
    }

    // NLoptOptimizer.hpp:219 — const StopCriteria &get_criteria() const { return m_opt.get_criteria(); }
    pub fn get_criteria(&self) -> &StopCriteria {
        self.m_opt.get_criteria()
    }

    // NLoptOptimizer.hpp:221 — void seed(long s) { m_opt.seed(s); }
    pub fn seed(&mut self, s: i64) {
        self.m_opt.seed(s);
    }
}

// `Optimizer<NLoptAlgComb<glob, loc>>` (NLoptOptimizer.hpp:196-222 with
// M = NLoptAlgComb). Same surface; the only difference is the underlying
// `detail::NLoptOpt<M>` is the combined global+local driver.
pub struct NLoptAlgCombOptimizer {
    // NLoptOptimizer.hpp:197 — detail::NLoptOpt<M> m_opt;
    m_opt: NLoptOptComb,
}

impl NLoptAlgCombOptimizer {
    // NLoptOptimizer.hpp:212 — explicit Optimizer(StopCriteria stopcr = {}) : m_opt(stopcr) {}
    pub fn new(alg: NLoptAlgComb, stopcr: StopCriteria) -> Self {
        Self {
            m_opt: NLoptOptComb::new(alg.gl_alg, alg.lc_alg, stopcr),
        }
    }

    // NLoptOptimizer.hpp:201 — Optimizer& to_max()
    pub fn to_max(&mut self) -> &mut Self {
        self.m_opt.base.set_dir(OptDir::Max);
        self
    }

    // NLoptOptimizer.hpp:202 — Optimizer& to_min()
    pub fn to_min(&mut self) -> &mut Self {
        self.m_opt.base.set_dir(OptDir::Min);
        self
    }

    // NLoptOptimizer.hpp:204-210 — Result<N> optimize(...)
    #[allow(clippy::result_unit_err)]
    pub fn optimize<const N: usize, F, R>(
        &mut self,
        func: F,
        initvals: &Input<N>,
        bounds: &Bounds<N>,
    ) -> Result<OptResult<N>, NLoptBackendError>
    where
        F: Fn(&Input<N>) -> R,
        R: Into<ObjectiveValue<N>>,
    {
        self.m_opt.optimize(func, initvals, bounds)
    }

    // NLoptOptimizer.hpp:214-217 — set_criteria
    pub fn set_criteria(&mut self, cr: StopCriteria) -> &mut Self {
        self.m_opt.base.set_criteria(cr);
        self
    }

    // NLoptOptimizer.hpp:219 — get_criteria
    pub fn get_criteria(&self) -> &StopCriteria {
        self.m_opt.base.get_criteria()
    }

    // NLoptOptimizer.hpp:221 — seed
    pub fn seed(&mut self, s: i64) {
        self.m_opt.base.seed(s);
    }
}

// Predefinded NLopt algorithms
// NLoptOptimizer.hpp:224-229
//   using AlgNLoptGenetic = detail::NLoptAlgComb<NLOPT_GN_ESCH>;
//   using AlgNLoptSubplex = detail::NLoptAlg<NLOPT_LN_SBPLX>;
//   using AlgNLoptSimplex = detail::NLoptAlg<NLOPT_LN_NELDERMEAD>;
//   using AlgNLoptDIRECT  = detail::NLoptAlg<NLOPT_GN_DIRECT>;
//   using AlgNLoptMLSL    = detail::NLoptAlg<NLOPT_GN_MLSL>;
//
// In C++ these are type aliases for method tags. In Rust the method tag is a
// runtime value, so each alias is a constructor producing the corresponding tag.

// NLoptOptimizer.hpp:225 — using AlgNLoptGenetic = detail::NLoptAlgComb<NLOPT_GN_ESCH>;
pub fn alg_nlopt_genetic() -> NLoptAlgComb {
    NLoptAlgComb::new(NloptAlgorithm::NLOPT_GN_ESCH)
}

// NLoptOptimizer.hpp:226 — using AlgNLoptSubplex = detail::NLoptAlg<NLOPT_LN_SBPLX>;
pub fn alg_nlopt_subplex() -> NLoptAlg {
    NLoptAlg::new(NloptAlgorithm::NLOPT_LN_SBPLX)
}

// NLoptOptimizer.hpp:227 — using AlgNLoptSimplex = detail::NLoptAlg<NLOPT_LN_NELDERMEAD>;
pub fn alg_nlopt_simplex() -> NLoptAlg {
    NLoptAlg::new(NloptAlgorithm::NLOPT_LN_NELDERMEAD)
}

// NLoptOptimizer.hpp:228 — using AlgNLoptDIRECT = detail::NLoptAlg<NLOPT_GN_DIRECT>;
pub fn alg_nlopt_direct() -> NLoptAlg {
    NLoptAlg::new(NloptAlgorithm::NLOPT_GN_DIRECT)
}

// NLoptOptimizer.hpp:229 — using AlgNLoptMLSL = detail::NLoptAlg<NLOPT_GN_MLSL>;
pub fn alg_nlopt_mlsl() -> NLoptAlg {
    NLoptAlg::new(NloptAlgorithm::NLOPT_GN_MLSL)
}

// NLoptOptimizer.hpp:231 — }} // namespace Slic3r::opt

#[cfg(test)]
mod tests {
    use super::*;
    use crate::optimize::optimizer::Bound;

    // NLoptOptimizer.hpp:225-229 — predefined algorithm tags map to the right
    // nlopt_algorithm enumerators.
    #[test]
    fn test_predefined_algorithms() {
        assert_eq!(alg_nlopt_subplex().alg, NloptAlgorithm::NLOPT_LN_SBPLX);
        assert_eq!(alg_nlopt_simplex().alg, NloptAlgorithm::NLOPT_LN_NELDERMEAD);
        assert_eq!(alg_nlopt_direct().alg, NloptAlgorithm::NLOPT_GN_DIRECT);
        assert_eq!(alg_nlopt_mlsl().alg, NloptAlgorithm::NLOPT_GN_MLSL);

        let g = alg_nlopt_genetic();
        assert_eq!(g.gl_alg, NloptAlgorithm::NLOPT_GN_ESCH);
        // NLoptOptimizer.hpp:27 — default local method.
        assert_eq!(g.lc_alg, NloptAlgorithm::NLOPT_LN_NELDERMEAD);
    }

    // NLoptOptimizer.hpp:104-126 — set_up records the exact nlopt configuration.
    #[test]
    fn test_set_up_records_configuration() {
        let mut stc = StopCriteria::default();
        stc.abs_score_diff(1e-6)
            .rel_score_diff(1e-3)
            .stop_score(5.0)
            .max_iterations(250.0);
        let opt = NLoptOpt::new(NloptAlgorithm::NLOPT_LN_SBPLX, stc);
        let mut nl = detail::NLopt::create(NloptAlgorithm::NLOPT_LN_SBPLX, 2);
        let bounds = [Bound::new(0.0, 1.0), Bound::new(-2.0, 3.0)];
        let setup = opt.set_up(&mut nl, &bounds);

        assert_eq!(setup.lower_bounds, vec![0.0, -2.0]);
        assert_eq!(setup.upper_bounds, vec![1.0, 3.0]);
        assert_eq!(setup.ftol_abs, Some(1e-6));
        assert_eq!(setup.ftol_rel, Some(1e-3));
        assert_eq!(setup.stopval, Some(5.0));
        assert_eq!(setup.maxeval, Some(250));
    }

    // NLoptOptimizer.hpp:117-125 — NaN criteria are skipped; max_iterations==0 is skipped.
    #[test]
    fn test_set_up_default_criteria_skipped() {
        let opt = NLoptOpt::new(NloptAlgorithm::NLOPT_LN_NELDERMEAD, StopCriteria::default());
        let mut nl = detail::NLopt::create(NloptAlgorithm::NLOPT_LN_NELDERMEAD, 1);
        let setup = opt.set_up(&mut nl, &[Bound::new(0.0, 1.0)]);
        assert_eq!(setup.ftol_abs, None);
        assert_eq!(setup.ftol_rel, None);
        assert_eq!(setup.stopval, None);
        assert_eq!(setup.maxeval, None);
    }

    // NLoptOptimizer.hpp:128-159 — the backend is BLOCKED (native, non-wasm-safe).
    // `optimize` faithfully seeds the optimum with initvals but cannot run the
    // native solver, so it returns Err rather than a fabricated result.
    #[test]
    fn test_optimize_backend_blocked() {
        let mut opt = NLoptAlgOptimizer::new(alg_nlopt_simplex(), StopCriteria::default());
        opt.to_min();
        let res: Result<OptResult<1>, NLoptBackendError> = opt.optimize(
            |x: &[f64; 1]| (x[0] - 0.3) * (x[0] - 0.3),
            &[0.5],
            &[Bound::new(0.0, 1.0)],
        );
        assert!(res.is_err());
    }

    // NLoptOptimizer.hpp:170-191 — combined optimizer wires glob+loc; still BLOCKED.
    #[test]
    fn test_comb_optimizer_backend_blocked() {
        let mut opt = NLoptAlgCombOptimizer::new(alg_nlopt_genetic(), StopCriteria::default());
        opt.to_max();
        let res: Result<OptResult<2>, NLoptBackendError> = opt.optimize(
            |x: &[f64; 2]| -(x[0] * x[0] + x[1] * x[1]),
            &[0.5, 0.5],
            &[Bound::new(-1.0, 1.0), Bound::new(-1.0, 1.0)],
        );
        assert!(res.is_err());
    }

    // NLoptOptimizer.hpp:76-102 — optfunc converts params, dispatches on whether
    // the objective produces a plain score or a ScoreGradient, and honours the
    // stop_condition force-stop.
    #[test]
    fn test_optfunc_plain_score() {
        let opt = NLoptOpt::new(NloptAlgorithm::NLOPT_LN_NELDERMEAD, StopCriteria::default());
        let mut grad = [0.0f64; 2];
        let mut forced = false;
        let val = opt.optfunc::<2, _, f64>(
            2,
            &[2.0, 3.0],
            &mut grad,
            &|x: &[f64; 2]| x[0] + x[1],
            || forced = true,
        );
        assert_eq!(val, 5.0);
        assert!(!forced);
    }

    // NLoptOptimizer.hpp:93-96 — ScoreGradient path copies the gradient.
    #[test]
    fn test_optfunc_score_gradient() {
        let opt = NLoptOpt::new(NloptAlgorithm::NLOPT_LN_NELDERMEAD, StopCriteria::default());
        let mut grad = [0.0f64; 2];
        let val = opt.optfunc::<2, _, ScoreGradient<2>>(
            2,
            &[2.0, 3.0],
            &mut grad,
            &|x: &[f64; 2]| ScoreGradient::new(x[0] * x[1], [x[1], x[0]]),
            || {},
        );
        assert_eq!(val, 6.0);
        assert_eq!(grad, [3.0, 2.0]);
    }

    // NLoptOptimizer.hpp:85-86 — stop_condition triggers force_stop.
    #[test]
    fn test_optfunc_force_stop() {
        let mut stc = StopCriteria::default();
        stc.set_stop_condition(|| true);
        let opt = NLoptOpt::new(NloptAlgorithm::NLOPT_LN_NELDERMEAD, stc);
        let mut grad = [0.0f64; 1];
        let mut forced = false;
        let _ = opt.optfunc::<1, _, f64>(1, &[1.0], &mut grad, &|x: &[f64; 1]| x[0], || {
            forced = true
        });
        assert!(forced);
    }
}
