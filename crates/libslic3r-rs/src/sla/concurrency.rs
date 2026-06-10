//! SLA concurrency facade.
//!
//! C++ Reference:
//! - SLA/Concurrency.hpp
//!
//! Faithfulness notes (the C++ is a header-only template facade that cannot be
//! expressed verbatim in Rust):
//! - C++ `template<bool> struct _ccr {}` with explicit specializations
//!   `_ccr<true>` / `_ccr<false>` becomes two Rust modules (`ccr_true`,
//!   `ccr_false`) holding the same static members as free functions / type
//!   aliases. The C++ namespace aliases `ccr`, `ccr_seq`, `ccr_par` become
//!   `pub use` module aliases.
//! - The C++ member templates `for_each` (iterators OR integers) and the
//!   variadic `reduce` forwarders are monomorphized here into the
//!   instantiations actually used by libslic3r: an integer-index form plus a
//!   slice (iterator-pair) form, matching the two `execution::reduce`
//!   overloads in Execution.hpp:54-72 and Execution.hpp:76-91.
//! - Default argument `granularity = 1` (Concurrency.hpp:24,47) cannot be
//!   expressed in Rust; callers pass it explicitly.
//! - The function name `max_concurreny` preserves the typo from the C++
//!   source (Concurrency.hpp:35,58).

// Concurrency.hpp:4: // FIXME: Deprecated

// Concurrency.hpp:6: #include <libslic3r/Execution/ExecutionSeq.hpp>
// Concurrency.hpp:7: #include <libslic3r/Execution/ExecutionTBB.hpp>

// Concurrency.hpp:12-13:
// Set this to true to enable full parallelism in this module.
// Only the well tested parts will be concurrent if this is set to false.
// Concurrency.hpp:14: const constexpr bool USE_FULL_CONCURRENCY = true;
pub const USE_FULL_CONCURRENCY: bool = true;

// Concurrency.hpp:16: template<bool> struct _ccr {};

/// `_ccr<true>` — parallel (TBB-backed, here rayon-backed) concurrency facade.
///
/// Concurrency.hpp:18: template<> struct _ccr<true>
pub mod ccr_true {
    use crate::execution::execution::{self, ExecutionPolicy};
    use crate::execution::execution_tbb::{self, EX_TBB};

    // Concurrency.hpp:20: using SpinningMutex = execution::SpinningMutex<ExecutionTBB>;
    pub type SpinningMutex<T> = execution_tbb::SpinningMutex<T>;
    // Concurrency.hpp:21: using BlockingMutex = execution::BlockingMutex<ExecutionTBB>;
    pub type BlockingMutex<T> = execution_tbb::BlockingMutex<T>;

    /// Concurrency.hpp:23-27:
    /// template<class It, class Fn>
    /// static void for_each(It from, It to, Fn &&fn, size_t granularity = 1)
    ///
    /// Integer-index instantiation (`It` = `size_t`), the dominant call form
    /// in libslic3r (e.g. `ccr::for_each(size_t(0), hits.size(), ...)`).
    pub fn for_each<F>(from: usize, to: usize, fnc: F, granularity: usize)
    where
        F: Fn(usize) + Send + Sync,
    {
        // Concurrency.hpp:26: execution::for_each(ex_tbb, from, to, std::forward<Fn>(fn), granularity);
        EX_TBB.for_each(from, to, fnc, granularity);
    }

    /// Concurrency.hpp:23-27, iterator-pair instantiation over const elements
    /// (e.g. `ccr::for_each(m_iheads_onmodel.begin(), m_iheads_onmodel.end(), ...)`).
    pub fn for_each_ref<T, F>(items: &[T], fnc: F)
    where
        T: Send + Sync,
        F: Fn(&T) + Send + Sync,
    {
        // Concurrency.hpp:26: execution::for_each(ex_tbb, from, to, std::forward<Fn>(fn), granularity);
        EX_TBB.for_each_ref(items, fnc);
    }

    /// Concurrency.hpp:23-27, iterator-pair instantiation over mutable elements.
    pub fn for_each_mut<T, F>(items: &mut [T], fnc: F)
    where
        T: Send + Sync,
        F: Fn(&mut T) + Send + Sync,
    {
        // Concurrency.hpp:26: execution::for_each(ex_tbb, from, to, std::forward<Fn>(fn), granularity);
        EX_TBB.for_each_mut(items, fnc);
    }

    /// Concurrency.hpp:29-33:
    /// template<class...Args>
    /// static auto reduce(Args&&...args)
    /// { return execution::reduce(ex_tbb, std::forward<Args>(args)...); }
    ///
    /// Variadic forwarder instantiated as the access-function overload
    /// (Execution.hpp:54-72).
    pub fn reduce<T, MergeFn, AccessFn>(
        from: usize,
        to: usize,
        init: T,
        mergefn: MergeFn,
        accessfn: AccessFn,
        granularity: usize,
    ) -> T
    where
        T: Clone + Send + Sync,
        MergeFn: Fn(T, T) -> T + Send + Sync,
        AccessFn: Fn(usize) -> T + Send + Sync,
    {
        // Concurrency.hpp:32: return execution::reduce(ex_tbb, std::forward<Args>(args)...);
        EX_TBB.reduce(from, to, init, mergefn, accessfn, granularity)
    }

    /// Concurrency.hpp:29-33, variadic forwarder instantiated as the
    /// iterator overload without an access functor (Execution.hpp:76-91,
    /// which forwards with the identity access function).
    pub fn reduce_slice<T, MergeFn>(items: &[T], init: T, mergefn: MergeFn) -> T
    where
        T: Clone + Send + Sync,
        MergeFn: Fn(T, T) -> T + Send + Sync,
    {
        // Concurrency.hpp:32: return execution::reduce(ex_tbb, std::forward<Args>(args)...);
        EX_TBB.reduce_slice(items, init, mergefn)
    }

    /// Concurrency.hpp:35-38:
    /// static size_t max_concurreny()
    /// { return execution::max_concurrency(ex_tbb); }
    ///
    /// (sic: typo preserved from the C++ source)
    pub fn max_concurreny() -> usize {
        // Concurrency.hpp:37: return execution::max_concurrency(ex_tbb);
        execution::max_concurrency(&EX_TBB)
    }
}

/// `_ccr<false>` — sequential concurrency facade.
///
/// Concurrency.hpp:41: template<> struct _ccr<false>
pub mod ccr_false {
    use crate::execution::execution::{self, ExecutionPolicy};
    use crate::execution::execution_seq::{self, EX_SEQ};

    // Concurrency.hpp:43: using SpinningMutex = execution::SpinningMutex<ExecutionSeq>;
    //
    // Note: the sequential mutexes are the no-op `_Mtx` (ExecutionSeq.hpp:36)
    // and carry no data, hence no `<T>` parameter unlike the parallel aliases.
    pub type SpinningMutex = execution_seq::SpinningMutex;
    // Concurrency.hpp:44: using BlockingMutex = execution::BlockingMutex<ExecutionSeq>;
    pub type BlockingMutex = execution_seq::BlockingMutex;

    /// Concurrency.hpp:46-50:
    /// template<class It, class Fn>
    /// static void for_each(It from, It to, Fn &&fn, size_t granularity = 1)
    ///
    /// Integer-index instantiation (`It` = `size_t`).
    pub fn for_each<F>(from: usize, to: usize, fnc: F, granularity: usize)
    where
        F: Fn(usize) + Send + Sync,
    {
        // Concurrency.hpp:49: execution::for_each(ex_seq, from, to, std::forward<Fn>(fn), granularity);
        EX_SEQ.for_each(from, to, fnc, granularity);
    }

    /// Concurrency.hpp:46-50, iterator-pair instantiation over const elements.
    pub fn for_each_ref<T, F>(items: &[T], fnc: F)
    where
        T: Send + Sync,
        F: Fn(&T) + Send + Sync,
    {
        // Concurrency.hpp:49: execution::for_each(ex_seq, from, to, std::forward<Fn>(fn), granularity);
        EX_SEQ.for_each_ref(items, fnc);
    }

    /// Concurrency.hpp:46-50, iterator-pair instantiation over mutable elements.
    pub fn for_each_mut<T, F>(items: &mut [T], fnc: F)
    where
        T: Send + Sync,
        F: Fn(&mut T) + Send + Sync,
    {
        // Concurrency.hpp:49: execution::for_each(ex_seq, from, to, std::forward<Fn>(fn), granularity);
        EX_SEQ.for_each_mut(items, fnc);
    }

    /// Concurrency.hpp:52-56:
    /// template<class...Args>
    /// static auto reduce(Args&&...args)
    /// { return execution::reduce(ex_seq, std::forward<Args>(args)...); }
    ///
    /// Variadic forwarder instantiated as the access-function overload
    /// (Execution.hpp:54-72).
    pub fn reduce<T, MergeFn, AccessFn>(
        from: usize,
        to: usize,
        init: T,
        mergefn: MergeFn,
        accessfn: AccessFn,
        granularity: usize,
    ) -> T
    where
        T: Clone + Send + Sync,
        MergeFn: Fn(T, T) -> T + Send + Sync,
        AccessFn: Fn(usize) -> T + Send + Sync,
    {
        // Concurrency.hpp:55: return execution::reduce(ex_seq, std::forward<Args>(args)...);
        EX_SEQ.reduce(from, to, init, mergefn, accessfn, granularity)
    }

    /// Concurrency.hpp:52-56, variadic forwarder instantiated as the
    /// iterator overload without an access functor (Execution.hpp:76-91).
    pub fn reduce_slice<T, MergeFn>(items: &[T], init: T, mergefn: MergeFn) -> T
    where
        T: Clone + Send + Sync,
        MergeFn: Fn(T, T) -> T + Send + Sync,
    {
        // Concurrency.hpp:55: return execution::reduce(ex_seq, std::forward<Args>(args)...);
        EX_SEQ.reduce_slice(items, init, mergefn)
    }

    /// Concurrency.hpp:58-61:
    /// static size_t max_concurreny()
    /// { return execution::max_concurrency(ex_seq); }
    ///
    /// (sic: typo preserved from the C++ source)
    pub fn max_concurreny() -> usize {
        // Concurrency.hpp:60: return execution::max_concurrency(ex_seq);
        execution::max_concurrency(&EX_SEQ)
    }
}

// Concurrency.hpp:64: using ccr = _ccr<USE_FULL_CONCURRENCY>;
// (USE_FULL_CONCURRENCY is `true`, so `ccr` aliases the parallel facade.)
pub use self::ccr_true as ccr;
// Concurrency.hpp:65: using ccr_seq = _ccr<false>;
pub use self::ccr_false as ccr_seq;
// Concurrency.hpp:66: using ccr_par = _ccr<true>;
pub use self::ccr_true as ccr_par;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ccr_for_each_indices() {
        use std::sync::atomic::{AtomicUsize, Ordering};
        let counter = AtomicUsize::new(0);
        ccr::for_each(
            0,
            64,
            |i| {
                counter.fetch_add(i, Ordering::Relaxed);
            },
            1,
        );
        assert_eq!(counter.load(Ordering::Relaxed), (0..64).sum::<usize>());
    }

    #[test]
    fn test_ccr_seq_reduce() {
        let sum = ccr_seq::reduce(0usize, 10, 0usize, |a, b| a + b, |i| i, 1);
        assert_eq!(sum, 45);
    }

    #[test]
    fn test_ccr_par_reduce_slice() {
        let items: Vec<usize> = (1..=10).collect();
        let sum = ccr_par::reduce_slice(&items, 0usize, |a, b| a + b);
        assert_eq!(sum, 55);
    }

    #[test]
    fn test_max_concurreny() {
        assert_eq!(ccr_seq::max_concurreny(), 1);
        assert!(ccr_par::max_concurreny() >= 1);
        // ccr aliases ccr_par because USE_FULL_CONCURRENCY = true.
        assert!(USE_FULL_CONCURRENCY);
        assert!(ccr::max_concurreny() >= 1);
    }

    #[test]
    fn test_mutex_aliases() {
        let m: ccr::BlockingMutex<i32> = ccr::BlockingMutex::new(0);
        *m.lock().unwrap() += 1;
        assert_eq!(*m.lock().unwrap(), 1);

        let s: ccr::SpinningMutex<i32> = ccr::SpinningMutex::new(0);
        *s.lock() += 1;
        assert_eq!(*s.lock(), 1);

        let nm: ccr_seq::BlockingMutex = ccr_seq::BlockingMutex::default();
        nm.lock();
        nm.unlock();
    }
}
