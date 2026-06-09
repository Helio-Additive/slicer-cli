//! Sequential execution policy.
//!
//! C++ Reference:
//! - Execution/ExecutionSeq.hpp
//!
//! Implements the ExecutionPolicy trait using standard sequential iteration.
//! No parallelism is used; all operations run on the calling thread.
//! The mutex types are no-ops (zero-cost).

use super::execution::ExecutionPolicy;

/// Sequential execution policy.
///
/// All operations are performed sequentially on the calling thread.
/// Granularity parameters are ignored since there is no parallelism.
///
/// C++ equivalent: `ExecutionSeq` struct + `Traits<ExecutionSeq>` specialization.
///
/// ExecutionSeq.hpp:13
#[derive(Debug, Clone, Copy)]
pub struct SequentialPolicy;

/// Global instance of the sequential execution policy.
///
/// ExecutionSeq.hpp:17: `static constexpr ExecutionSeq ex_seq = {};`
pub static EX_SEQ: SequentialPolicy = SequentialPolicy;

/// No-op mutex for sequential policy (satisfies BasicLockable concept).
///
/// ExecutionSeq.hpp:36: `struct _Mtx { inline void lock() {} inline void unlock() {} };`
#[derive(Debug, Default)]
pub struct NoOpMutex;

impl NoOpMutex {
    // ExecutionSeq.hpp:36: `inline void lock() {}`
    pub fn lock(&self) {}
    // ExecutionSeq.hpp:36: `inline void unlock() {}`
    pub fn unlock(&self) {}
}

/// Spinning mutex type for sequential policy.
///
/// The sequential `Traits` specialization aliases both mutex types to the
/// no-op `_Mtx` since there is no concurrency.
///
/// ExecutionSeq.hpp:51: `using SpinningMutex = _Mtx;`
pub type SpinningMutex = NoOpMutex;

/// Blocking mutex type for sequential policy.
///
/// ExecutionSeq.hpp:52: `using BlockingMutex = _Mtx;`
pub type BlockingMutex = NoOpMutex;

impl ExecutionPolicy for SequentialPolicy {
    /// Sequential for_each: simple loop from `from` to `to`.
    ///
    /// ExecutionSeq.hpp:54-62
    fn for_each<F>(&self, from: usize, to: usize, f: F, _granularity: usize)
    where
        F: Fn(usize) + Send + Sync,
    {
        for i in from..to {
            f(i);
        }
    }

    /// Sequential for_each over a slice by reference.
    fn for_each_ref<T, F>(&self, items: &[T], f: F)
    where
        T: Send + Sync,
        F: Fn(&T) + Send + Sync,
    {
        for item in items {
            f(item);
        }
    }

    /// Sequential for_each over a mutable slice.
    fn for_each_mut<T, F>(&self, items: &mut [T], f: F)
    where
        T: Send + Sync,
        F: Fn(&mut T) + Send + Sync,
    {
        for item in items.iter_mut() {
            f(item);
        }
    }

    /// Sequential reduce with merge and access functions.
    ///
    /// ExecutionSeq.hpp:64-77
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
        // ExecutionSeq.hpp:74-76
        // C++: T acc = init;
        // C++: loop_(from, to, [&](auto &i) { acc = mergefn(acc, access(i)); });
        // C++: return acc;
        let mut acc = init;
        for i in from..to {
            acc = merge(acc, access(i));
        }
        acc
    }

    /// Sequential reduce over a slice.
    fn reduce_slice<T, MergeFn>(&self, items: &[T], init: T, merge: MergeFn) -> T
    where
        T: Clone + Send + Sync,
        MergeFn: Fn(T, T) -> T + Send + Sync,
    {
        let mut acc = init;
        for item in items {
            acc = merge(acc, item.clone());
        }
        acc
    }

    /// Sequential max_concurrency always returns 1.
    ///
    /// ExecutionSeq.hpp:79
    fn max_concurrency(&self) -> usize {
        1
    }

    /// Sequential sort using std sort_by.
    fn sort<T, F>(&self, items: &mut [T], compare: F)
    where
        T: Send,
        F: Fn(&T, &T) -> std::cmp::Ordering + Send + Sync,
    {
        items.sort_by(compare);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_sequential_for_each() {
        let policy = SequentialPolicy;
        let mut results = vec![0; 5];
        // Use index-based approach since we can't capture &mut with Fn
        let results_ptr = results.as_mut_ptr();
        policy.for_each(
            0,
            5,
            move |i| unsafe {
                *results_ptr.add(i) = i * 3;
            },
            1,
        );
        assert_eq!(results, vec![0, 3, 6, 9, 12]);
    }

    #[test]
    fn test_sequential_reduce() {
        let policy = SequentialPolicy;
        let sum = policy.reduce(0, 10, 0usize, |a, b| a + b, |i| i, 1);
        assert_eq!(sum, 45);
    }

    #[test]
    fn test_sequential_reduce_product() {
        let policy = SequentialPolicy;
        let product = policy.reduce(1, 6, 1usize, |a, b| a * b, |i| i, 1);
        assert_eq!(product, 120); // 1*2*3*4*5
    }

    #[test]
    fn test_sequential_max_concurrency() {
        let policy = SequentialPolicy;
        assert_eq!(policy.max_concurrency(), 1);
    }

    #[test]
    fn test_sequential_sort() {
        let policy = SequentialPolicy;
        let mut items = vec![5, 3, 1, 4, 2];
        policy.sort(&mut items, |a, b| a.cmp(b));
        assert_eq!(items, vec![1, 2, 3, 4, 5]);
    }

    #[test]
    fn test_sequential_reduce_slice() {
        let policy = SequentialPolicy;
        let items = vec![1, 2, 3, 4, 5];
        let sum = policy.reduce_slice(&items, 0, |a, b| a + b);
        assert_eq!(sum, 15);
    }

    #[test]
    fn test_noop_mutex() {
        let mtx = NoOpMutex;
        mtx.lock();
        mtx.unlock();
        // No-op: just verifying it doesn't panic
    }

    #[test]
    fn test_ex_seq_global() {
        assert_eq!(EX_SEQ.max_concurrency(), 1);
    }
}
