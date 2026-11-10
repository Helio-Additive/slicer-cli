//! Parallel execution policy using rayon (Rust equivalent of TBB).
//!
//! C++ Reference:
//! - Execution/ExecutionTBB.hpp
//!
//! Implements the ExecutionPolicy trait using rayon for parallel execution.
//! This is the Rust equivalent of the C++ TBB-based execution policy.
//! Rayon provides work-stealing parallelism similar to TBB.
//!
//! Faithfulness notes (the C++ is a header-only template specialization that
//! cannot be expressed verbatim in Rust):
//! - C++ uses template specialization + SFINAE (`execution::Traits<ExecutionTBB>`,
//!   `IsExecutionPolicy_<ExecutionTBB> : std::true_type`). Rust uses the
//!   `ExecutionPolicy` trait (see execution.rs) implemented by `ParallelPolicy`.
//! - The two private `loop_` helpers (ExecutionTBB.hpp:24-34) dispatch on whether
//!   the range is over iterators or integers; in Rust the integer-range loop is
//!   `from..to` and the slice loop is `par_iter`, folded directly into each method.
//! - TBB native backend (`tbb::*`) is replaced by rayon, which is pure-Rust and
//!   wasm-safe (no system/dylib dependency).

use super::execution::ExecutionPolicy;
use rayon::prelude::*;

/// Parallel execution policy using rayon.
///
/// This is the Rust equivalent of C++'s `ExecutionTBB` which uses Intel TBB.
/// Rayon provides similar work-stealing parallelism.
///
/// ExecutionTBB.hpp:15: `struct ExecutionTBB {};`
/// ExecutionTBB.hpp:16: `template<> struct IsExecutionPolicy_<ExecutionTBB> : public std::true_type {};`
#[derive(Debug, Clone, Copy)]
pub struct ParallelPolicy;

/// Global instance of the parallel execution policy.
///
/// ExecutionTBB.hpp:18-19:
/// `// Execution policy using Intel TBB library under the hood.`
/// `static constexpr ExecutionTBB ex_tbb = {};`
pub static EX_TBB: ParallelPolicy = ParallelPolicy;

/// Spinning mutex type for parallel policy.
///
/// Uses parking_lot::Mutex as a lightweight spinning mutex.
///
/// ExecutionTBB.hpp:37: `using SpinningMutex = tbb::spin_mutex;`
pub type SpinningMutex<T> = parking_lot::Mutex<T>;

/// Blocking mutex type for parallel policy.
///
/// Uses std::sync::Mutex for blocking operations.
///
/// ExecutionTBB.hpp:38: `using BlockingMutex = std::mutex;`
pub type BlockingMutex<T> = std::sync::Mutex<T>;

impl ExecutionPolicy for ParallelPolicy {
    /// Parallel for_each using rayon.
    ///
    /// ExecutionTBB.hpp:40-48:
    /// ```cpp
    /// template<class It, class Fn>
    /// static void for_each(const ExecutionTBB &,
    ///                      It from, It to, Fn &&fn, size_t granularity)
    /// {
    ///     tbb::parallel_for(tbb::blocked_range{from, to, granularity},
    ///                       [&fn](const auto &range) {
    ///         loop_(range, std::forward<Fn>(fn));
    ///     });
    /// }
    /// ```
    /// The integer `loop_` (ExecutionTBB.hpp:30-34) iterates `i` from `range.begin()`
    /// to `range.end()`; rayon partitions `from..to` and applies `fn` to each `i`.
    /// Granularity is ignored: rayon manages its own work-splitting.
    fn for_each<F>(&self, from: usize, to: usize, f: F, _granularity: usize)
    where
        F: Fn(usize) + Send + Sync,
    {
        (from..to).into_par_iter().for_each(|i| f(i));
    }

    /// Parallel for_each over a slice by reference.
    fn for_each_ref<T, F>(&self, items: &[T], f: F)
    where
        T: Send + Sync,
        F: Fn(&T) + Send + Sync,
    {
        items.par_iter().for_each(|item| f(item));
    }

    /// Parallel for_each over a mutable slice.
    fn for_each_mut<T, F>(&self, items: &mut [T], f: F)
    where
        T: Send + Sync,
        F: Fn(&mut T) + Send + Sync,
    {
        items.par_iter_mut().for_each(|item| f(item));
    }

    /// Parallel reduce using rayon.
    ///
    /// ExecutionTBB.hpp:50-68:
    /// ```cpp
    /// return tbb::parallel_reduce(
    ///     tbb::blocked_range{from, to, granularity}, init,
    ///     [&](const auto &range, T subinit) {
    ///         T acc = subinit;
    ///         loop_(range, [&](auto &i) { acc = mergefn(acc, access(i)); });
    ///         return acc;
    ///     },
    ///     std::forward<MergeFn>(mergefn));
    /// ```
    /// In TBB `init` is the identity seed supplied as `subinit` to each leaf range.
    /// rayon's `reduce` takes an identity closure `|| init.clone()` and the same
    /// `merge` (mergefn) for both leaf folding and inter-range combination, matching
    /// TBB's two uses of `mergefn`.
    fn reduce<T, MergeFn, AccessFn>(
        &self,
        from: usize,
        to: usize,
        init: T,
        merge: MergeFn,
        access: AccessFn,
        _granularity: usize,
    ) -> T
    where
        T: Clone + Send + Sync,
        MergeFn: Fn(T, T) -> T + Send + Sync,
        AccessFn: Fn(usize) -> T + Send + Sync,
    {
        (from..to)
            .into_par_iter()
            .map(|i| access(i))
            .reduce(|| init.clone(), |a, b| merge(a, b))
    }

    /// Parallel reduce over a slice.
    fn reduce_slice<T, MergeFn>(&self, items: &[T], init: T, merge: MergeFn) -> T
    where
        T: Clone + Send + Sync,
        MergeFn: Fn(T, T) -> T + Send + Sync,
    {
        items
            .par_iter()
            .cloned()
            .reduce(|| init.clone(), |a, b| merge(a, b))
    }

    /// Query the number of rayon threads available.
    ///
    /// ExecutionTBB.hpp:70-73:
    /// ```cpp
    /// static size_t max_concurrency(const ExecutionTBB &)
    /// {
    ///     return tbb::this_task_arena::max_concurrency();
    /// }
    /// ```
    fn max_concurrency(&self) -> usize {
        rayon::current_num_threads()
    }

    /// Parallel sort using rayon's par_sort_by.
    fn sort<T, F>(&self, items: &mut [T], compare: F)
    where
        T: Send,
        F: Fn(&T, &T) -> std::cmp::Ordering + Send + Sync,
    {
        items.par_sort_by(compare);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::{AtomicUsize, Ordering};

    #[test]
    fn test_parallel_for_each() {
        let policy = ParallelPolicy;
        let counter = AtomicUsize::new(0);
        policy.for_each(
            0,
            100,
            |_i| {
                counter.fetch_add(1, Ordering::Relaxed);
            },
            1,
        );
        assert_eq!(counter.load(Ordering::Relaxed), 100);
    }

    #[test]
    fn test_parallel_reduce() {
        let policy = ParallelPolicy;
        let sum = policy.reduce(0, 100, 0usize, |a, b| a + b, |i| i, 1);
        assert_eq!(sum, 4950); // 0+1+...+99
    }

    #[test]
    fn test_parallel_max_concurrency() {
        let policy = ParallelPolicy;
        assert!(policy.max_concurrency() >= 1);
    }

    #[test]
    fn test_parallel_sort() {
        let policy = ParallelPolicy;
        let mut items: Vec<i32> = (0..100).rev().collect();
        policy.sort(&mut items, |a, b| a.cmp(b));
        let expected: Vec<i32> = (0..100).collect();
        assert_eq!(items, expected);
    }

    #[test]
    fn test_parallel_reduce_slice() {
        let policy = ParallelPolicy;
        let items: Vec<usize> = (1..=10).collect();
        let sum = policy.reduce_slice(&items, 0, |a, b| a + b);
        assert_eq!(sum, 55);
    }

    #[test]
    fn test_ex_tbb_global() {
        assert!(EX_TBB.max_concurrency() >= 1);
    }
}
