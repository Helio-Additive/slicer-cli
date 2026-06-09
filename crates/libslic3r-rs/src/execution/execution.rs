//! Execution policy framework - generic dispatch functions.
//!
//! C++ Reference:
//! - Execution/Execution.hpp
//!
//! This module provides a trait-based execution policy system that dispatches
//! parallel or sequential operations. In C++, this uses template specialization
//! with SFINAE; in Rust, we use traits.
//!
//! Key concepts:
//! - `ExecutionPolicy` trait: Interface for execution strategies
//! - `for_each`: Parallel/sequential iteration
//! - `reduce`: Parallel/sequential reduction
//! - `accumulate`: Summation via reduce with addition
//! - `max_concurrency`: Query available parallelism

/// Trait representing an execution policy.
///
/// C++ equivalent: `execution::Traits<EP>` template specialization.
/// Each execution policy (sequential, parallel) implements this trait.
///
/// Execution/Execution.hpp:26
///
/// In C++ the namespace-level alias templates
/// `template<class EP> using SpinningMutex = typename Traits<EP>::SpinningMutex;`
/// and `template<class EP> using BlockingMutex = typename Traits<EP>::BlockingMutex;`
/// (Execution/Execution.hpp:30-33) forward to per-policy mutex types. In this Rust
/// port those mutex types are exposed as concrete `type` aliases inside each policy
/// module (see `execution_seq::{SpinningMutex, BlockingMutex}` and the TBB equivalent).
pub trait ExecutionPolicy: Send + Sync {
    /// Execute a function for each element in a range.
    ///
    /// Execution/Execution.hpp:44-48
    fn for_each<F>(&self, from: usize, to: usize, f: F, granularity: usize)
    where
        F: Fn(usize) + Send + Sync;

    /// Execute a function for each element in a slice (by reference).
    fn for_each_ref<T, F>(&self, items: &[T], f: F)
    where
        T: Send + Sync,
        F: Fn(&T) + Send + Sync;

    /// Execute a function for each element in a mutable slice.
    fn for_each_mut<T, F>(&self, items: &mut [T], f: F)
    where
        T: Send + Sync,
        F: Fn(&mut T) + Send + Sync;

    /// Reduce a range with a merge function and an access function.
    ///
    /// Execution/Execution.hpp:54-72
    fn reduce<T, MergeFn, AccessFn>(
        &self,
        from: usize,
        to: usize,
        init: T,
        merge: MergeFn,
        access: AccessFn,
        granularity: usize,
    ) -> T
    where
        T: Clone + Send + Sync,
        MergeFn: Fn(T, T) -> T + Send + Sync,
        AccessFn: Fn(usize) -> T + Send + Sync;

    /// Reduce a slice with a merge function.
    ///
    /// Execution/Execution.hpp:76-92
    fn reduce_slice<T, MergeFn>(&self, items: &[T], init: T, merge: MergeFn) -> T
    where
        T: Clone + Send + Sync,
        MergeFn: Fn(T, T) -> T + Send + Sync;

    /// Query available threads for concurrency.
    ///
    /// Execution/Execution.hpp:36-40
    fn max_concurrency(&self) -> usize;

    /// Sort a mutable slice.
    fn sort<T, F>(&self, items: &mut [T], compare: F)
    where
        T: Send,
        F: Fn(&T, &T) -> std::cmp::Ordering + Send + Sync;
}

/// Accumulate (sum) a range using an access function.
///
/// This is a convenience wrapper around reduce using addition as the merge function.
///
/// Execution/Execution.hpp:93-107
pub fn accumulate<EP, T, AccessFn>(
    ep: &EP,
    from: usize,
    to: usize,
    init: T,
    access: AccessFn,
    granularity: usize,
) -> T
where
    EP: ExecutionPolicy,
    T: Clone + Send + Sync + std::ops::Add<Output = T>,
    AccessFn: Fn(usize) -> T + Send + Sync,
{
    ep.reduce(from, to, init, |a, b| a + b, access, granularity)
}

/// Accumulate (sum) a slice of values.
///
/// Execution/Execution.hpp:110-123
pub fn accumulate_slice<EP, T>(ep: &EP, items: &[T], init: T) -> T
where
    EP: ExecutionPolicy,
    T: Clone + Send + Sync + std::ops::Add<Output = T>,
{
    ep.reduce_slice(items, init, |a, b| a + b)
}

/// Convenience wrapper: for_each with default granularity of 1.
///
/// Execution/Execution.hpp:44-48
pub fn for_each<EP, F>(ep: &EP, from: usize, to: usize, f: F)
where
    EP: ExecutionPolicy,
    F: Fn(usize) + Send + Sync,
{
    ep.for_each(from, to, f, 1);
}

/// Convenience wrapper: reduce with default granularity of 1.
///
/// Execution/Execution.hpp:54-72
pub fn reduce<EP, T, MergeFn, AccessFn>(
    ep: &EP,
    from: usize,
    to: usize,
    init: T,
    merge: MergeFn,
    access: AccessFn,
) -> T
where
    EP: ExecutionPolicy,
    T: Clone + Send + Sync,
    MergeFn: Fn(T, T) -> T + Send + Sync,
    AccessFn: Fn(usize) -> T + Send + Sync,
{
    ep.reduce(from, to, init, merge, access, 1)
}

/// Query the maximum concurrency for an execution policy.
///
/// Execution/Execution.hpp:36-40
pub fn max_concurrency<EP: ExecutionPolicy>(ep: &EP) -> usize {
    ep.max_concurrency()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::execution::execution_seq::SequentialPolicy;

    #[test]
    fn test_for_each() {
        let ep = SequentialPolicy;
        let data = std::sync::Mutex::new(vec![0usize; 10]);
        for_each(&ep, 0, 10, |i| {
            data.lock().unwrap()[i] = i * 2;
        });
        let result = data.lock().unwrap();
        for i in 0..10 {
            assert_eq!(result[i], i * 2);
        }
    }

    #[test]
    fn test_reduce() {
        let ep = SequentialPolicy;
        let sum = reduce(&ep, 0, 10, 0usize, |a, b| a + b, |i| i);
        assert_eq!(sum, 45); // 0+1+...+9
    }

    #[test]
    fn test_accumulate() {
        let ep = SequentialPolicy;
        let sum = accumulate(&ep, 0, 5, 0usize, |i| i * i, 1);
        assert_eq!(sum, 30); // 0+1+4+9+16
    }

    #[test]
    fn test_max_concurrency_seq() {
        let ep = SequentialPolicy;
        assert_eq!(max_concurrency(&ep), 1);
    }
}
