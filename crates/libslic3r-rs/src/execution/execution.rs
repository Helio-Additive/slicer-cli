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

/// Query the available threads for concurrency.
///
/// C++:
/// ```cpp
/// template<class EP, class = ExecutionPolicyOnly<EP> >
/// size_t max_concurrency(const EP &ep)
/// { return AsTraits<EP>::max_concurrency(ep); }
/// ```
///
/// Execution/Execution.hpp:36-40
pub fn max_concurrency<EP: ExecutionPolicy>(ep: &EP) -> usize {
    // Execution/Execution.hpp:39: return AsTraits<EP>::max_concurrency(ep);
    ep.max_concurrency()
}

/// foreach loop with the execution policy passed as argument. Granularity can
/// be specified explicitly. `max_concurrency()` can be used for optimal results.
///
/// C++:
/// ```cpp
/// template<class EP, class It, class Fn, class = ExecutionPolicyOnly<EP>>
/// void for_each(const EP &ep, It from, It to, Fn &&fn, size_t granularity = 1)
/// { AsTraits<EP>::for_each(ep, from, to, std::forward<Fn>(fn), granularity); }
/// ```
///
/// The C++ `granularity` default of `1` is reproduced by callers passing `1`.
///
/// Execution/Execution.hpp:44-48
pub fn for_each<EP, F>(ep: &EP, from: usize, to: usize, f: F, granularity: usize)
where
    EP: ExecutionPolicy,
    F: Fn(usize) + Send + Sync,
{
    // Execution/Execution.hpp:47: AsTraits<EP>::for_each(ep, from, to, std::forward<Fn>(fn), granularity);
    ep.for_each(from, to, f, granularity);
}

/// A reduce operation with the execution policy passed as argument.
/// `mergefn` has `T(const T&, const T&)` signature and `accessfn` has
/// `T(I)` signature (for integral `I`) / `T(const I::value_type&)` (for iterators).
///
/// C++:
/// ```cpp
/// template<class EP, class I, class MergeFn, class T, class AccessFn,
///          class = ExecutionPolicyOnly<EP> >
/// T reduce(const EP &ep, I from, I to, const T &init,
///          MergeFn &&mergefn, AccessFn &&accessfn, size_t granularity = 1)
/// { return AsTraits<EP>::reduce(ep, from, to, init,
///       std::forward<MergeFn>(mergefn), std::forward<AccessFn>(accessfn), granularity); }
/// ```
///
/// Execution/Execution.hpp:54-72
pub fn reduce<EP, T, MergeFn, AccessFn>(
    ep: &EP,
    from: usize,
    to: usize,
    init: T,
    merge: MergeFn,
    access: AccessFn,
    granularity: usize,
) -> T
where
    EP: ExecutionPolicy,
    T: Clone + Send + Sync,
    MergeFn: Fn(T, T) -> T + Send + Sync,
    AccessFn: Fn(usize) -> T + Send + Sync,
{
    // Execution/Execution.hpp:68-71: return AsTraits<EP>::reduce(ep, from, to, init,
    //     std::forward<MergeFn>(mergefn), std::forward<AccessFn>(accessfn), granularity);
    ep.reduce(from, to, init, merge, access, granularity)
}

/// An overload of `reduce` to be used with iterators as `from`/`to` arguments.
/// The access functor is omitted; the C++ overload forwards to the access-fn
/// overload with the identity functor `[](const auto &i) { return i; }`.
///
/// C++:
/// ```cpp
/// template<class EP, class I, class MergeFn, class T, class = ExecutionPolicyOnly<EP> >
/// T reduce(const EP &ep, I from, I to, const T &init, MergeFn &&mergefn, size_t granularity = 1)
/// { return reduce(ep, from, to, init, std::forward<MergeFn>(mergefn),
///       [](const auto &i) { return i; }, granularity); }
/// ```
///
/// In this port the iterator/`value_type` instantiation is modelled by the
/// per-policy `reduce_slice` trait method (the `granularity` argument is carried
/// for parity with the C++ overload even though the slice form does not split).
///
/// Execution/Execution.hpp:76-91
pub fn reduce_slice<EP, T, MergeFn>(
    ep: &EP,
    items: &[T],
    init: T,
    merge: MergeFn,
    _granularity: usize,
) -> T
where
    EP: ExecutionPolicy,
    T: Clone + Send + Sync,
    MergeFn: Fn(T, T) -> T + Send + Sync,
{
    // Execution/Execution.hpp:88-90: return reduce(ep, from, to, init,
    //     std::forward<MergeFn>(mergefn), [](const auto &i) { return i; }, granularity);
    ep.reduce_slice(items, init, merge)
}

/// Accumulate (sum) a range using an access function. Convenience wrapper around
/// `reduce` using `std::plus<T>` as the merge function.
///
/// C++:
/// ```cpp
/// template<class EP, class I, class T, class AccessFn, class = ExecutionPolicyOnly<EP>>
/// T accumulate(const EP &ep, I from, I to, const T &init, AccessFn &&accessfn, size_t granularity = 1)
/// { return reduce(ep, from, to, init, std::plus<T>{},
///       std::forward<AccessFn>(accessfn), granularity); }
/// ```
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
    // Execution/Execution.hpp:105-106: return reduce(ep, from, to, init, std::plus<T>{},
    //     std::forward<AccessFn>(accessfn), granularity);
    reduce(ep, from, to, init, |a, b| a + b, access, granularity)
}

/// Accumulate (sum) over an iterator range, with the identity access functor.
///
/// C++:
/// ```cpp
/// template<class EP, class I, class T, class = ExecutionPolicyOnly<EP> >
/// T accumulate(const EP &ep, I from, I to, const T &init, size_t granularity = 1)
/// { return reduce(ep, from, to, init, std::plus<T>{},
///       [](const auto &i) { return i; }, granularity); }
/// ```
///
/// Modelled over a slice via `reduce_slice` (the iterator/`value_type`
/// instantiation), mirroring the identity-access `std::plus<T>` reduction.
///
/// Execution/Execution.hpp:110-123
pub fn accumulate_slice<EP, T>(ep: &EP, items: &[T], init: T, granularity: usize) -> T
where
    EP: ExecutionPolicy,
    T: Clone + Send + Sync + std::ops::Add<Output = T>,
{
    // Execution/Execution.hpp:120-122: return reduce(ep, from, to, init, std::plus<T>{},
    //     [](const auto &i) { return i; }, granularity);
    reduce_slice(ep, items, init, |a, b| a + b, granularity)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::execution::execution_seq::SequentialPolicy;

    #[test]
    fn test_for_each() {
        let ep = SequentialPolicy;
        let data = std::sync::Mutex::new(vec![0usize; 10]);
        for_each(
            &ep,
            0,
            10,
            |i| {
                data.lock().unwrap()[i] = i * 2;
            },
            1,
        );
        let result = data.lock().unwrap();
        for i in 0..10 {
            assert_eq!(result[i], i * 2);
        }
    }

    #[test]
    fn test_reduce() {
        let ep = SequentialPolicy;
        let sum = reduce(&ep, 0, 10, 0usize, |a, b| a + b, |i| i, 1);
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
