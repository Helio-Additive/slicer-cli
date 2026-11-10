//! Thread-safe message passing channel with blocking and non-blocking operations
//!
//! C++ Reference:
//! - Channel.hpp
//!
//! Faithful port of `template<class T> class Channel` (Channel.hpp:15-96).
//!
//! The C++ class owns a `std::deque<T>` guarded by a `std::mutex`, with a
//! `std::condition_variable` for blocking `pop()`. It is a non-copyable owning
//! container; sharing across threads is done by passing references/pointers to a
//! single instance (in Rust this is `Arc<Channel<T>>` at the call site).

use std::collections::VecDeque;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::{Condvar, Mutex, MutexGuard};

/// Thread-safe message passing channel
/// Channel.hpp:15-96
///
/// C++: template<class T> class Channel
pub struct Channel<T> {
    /// Internal queue protected by mutex
    /// Channel.hpp:93 — C++: Queue m_queue;  (using Queue = std::deque<T>;  Channel.hpp:34)
    m_queue: Mutex<VecDeque<T>>,

    /// Lock-free shadow of the queue length, kept in sync under `m_queue`'s lock.
    /// Used by `size_hint()` to mirror the C++ unlocked (thread-unsafe) read of
    /// `m_queue.size()`, which Rust's `Mutex` cannot do without locking.
    m_size: AtomicUsize,

    /// Condition variable for blocking operations
    /// Channel.hpp:95 — C++: std::condition_variable m_condition;
    m_condition: Condvar,
}

impl<T> Channel<T> {
    /// Channel.hpp:38 — C++: Channel() {}
    pub fn new() -> Self {
        Channel {
            m_queue: Mutex::new(VecDeque::new()),
            m_size: AtomicUsize::new(0),
            m_condition: Condvar::new(),
        }
    }

    // Channel.hpp:39 — C++: ~Channel() {}  (no-op; Rust drops members automatically)

    /// Push an item onto the channel.
    /// Channel.hpp:41-48 — C++: void push(const T& item, bool silent = false)
    /// Channel.hpp:50-57 — C++: void push(T &&item, bool silent = false)
    ///
    /// The two C++ overloads (copy / move) collapse into a single by-value
    /// signature in Rust, where `item` is always moved in.
    pub fn push(&self, item: T, silent: bool) {
        {
            // Channel.hpp:44 — C++: UniqueLock lock(m_mutex);
            let mut lock = self.m_queue.lock().unwrap();
            // Channel.hpp:45 — C++: m_queue.push_back(item);
            lock.push_back(item);
            self.m_size.store(lock.len(), Ordering::Relaxed);
        }
        // Channel.hpp:47 — C++: if (! silent) { m_condition.notify_one(); }
        if !silent {
            self.m_condition.notify_one();
        }
    }

    /// Pop an item from the channel, blocking until one is available.
    /// Channel.hpp:59-66 — C++: T pop()
    pub fn pop(&self) -> T {
        // Channel.hpp:61 — C++: UniqueLock lock(m_mutex);
        let mut lock = self.m_queue.lock().unwrap();
        // Channel.hpp:62 — C++: m_condition.wait(lock, [this]() { return !m_queue.empty(); });
        while lock.is_empty() {
            lock = self.m_condition.wait(lock).unwrap();
        }
        // Channel.hpp:63 — C++: auto item = std::move(m_queue.front());
        // Channel.hpp:64 — C++: m_queue.pop_front();
        let item = lock.pop_front().unwrap();
        self.m_size.store(lock.len(), Ordering::Relaxed);
        // Channel.hpp:65 — C++: return item;
        item
    }

    /// Try to pop an item without blocking; returns `None` if the queue is empty.
    /// Channel.hpp:68-78 — C++: boost::optional<T> try_pop()
    pub fn try_pop(&self) -> Option<T> {
        // Channel.hpp:70 — C++: UniqueLock lock(m_mutex);
        let mut lock = self.m_queue.lock().unwrap();
        // Channel.hpp:71-72 — C++: if (m_queue.empty()) { return boost::none; }
        if lock.is_empty() {
            None
        } else {
            // Channel.hpp:73-76 — C++: auto item = std::move(m_queue.front());
            //                          m_queue.pop();  (a deque has no pop(); intent is pop_front)
            //                          return item;
            let item = lock.pop_front();
            self.m_size.store(lock.len(), Ordering::Relaxed);
            item
        }
    }

    /// Unlocked observer/hint. Thread unsafe! Keep in mind you need to re-verify
    /// the result after locking.
    /// Channel.hpp:80-81 — C++: size_t size_hint() const noexcept { return m_queue.size(); }
    pub fn size_hint(&self) -> usize {
        // Mirrors C++'s deliberately unsynchronized read of m_queue.size(): we read
        // a lock-free shadow counter rather than locking the mutex.
        self.m_size.load(Ordering::Relaxed)
    }

    /// Lock the queue for read-only batch access.
    /// Channel.hpp:83-86 — C++: LockedConstPtr lock_read() const
    ///
    /// The C++ returns a `unique_ptr<const Queue, Unlocker>` whose deleter unlocks
    /// the mutex on destruction; the idiomatic Rust equivalent is a guard that
    /// unlocks on drop and only exposes shared (`&`) access.
    pub fn lock_read(&self) -> ChannelReadGuard<'_, T> {
        ChannelReadGuard {
            guard: self.m_queue.lock().unwrap(),
        }
    }

    /// Lock the queue for read-write batch access.
    /// Channel.hpp:88-91 — C++: LockedPtr lock_rw()
    pub fn lock_rw(&self) -> ChannelWriteGuard<'_, T> {
        let guard = self.m_queue.lock().unwrap();
        ChannelWriteGuard {
            guard,
            size: &self.m_size,
        }
    }
}

impl<T> Default for Channel<T> {
    fn default() -> Self {
        Self::new()
    }
}

/// RAII guard for read-only access to the channel queue.
/// Channel.hpp:35 — C++: using LockedConstPtr = std::unique_ptr<const Queue, Unlocker<const Queue>>;
///
/// Exposes the queue as `&VecDeque<T>` (read-only), unlocking on drop.
pub struct ChannelReadGuard<'a, T> {
    guard: MutexGuard<'a, VecDeque<T>>,
}

impl<T> std::ops::Deref for ChannelReadGuard<'_, T> {
    type Target = VecDeque<T>;

    fn deref(&self) -> &Self::Target {
        &self.guard
    }
}

/// RAII guard for read-write access to the channel queue.
/// Channel.hpp:36 — C++: using LockedPtr = std::unique_ptr<Queue, Unlocker<Queue>>;
///
/// Exposes the queue as `&mut VecDeque<T>`, unlocking on drop. The `size` shadow
/// counter is refreshed on drop so `size_hint()` stays consistent with any
/// mutations made through the guard.
pub struct ChannelWriteGuard<'a, T> {
    guard: MutexGuard<'a, VecDeque<T>>,
    size: &'a AtomicUsize,
}

impl<T> std::ops::Deref for ChannelWriteGuard<'_, T> {
    type Target = VecDeque<T>;

    fn deref(&self) -> &Self::Target {
        &self.guard
    }
}

impl<T> std::ops::DerefMut for ChannelWriteGuard<'_, T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.guard
    }
}

impl<T> Drop for ChannelWriteGuard<'_, T> {
    fn drop(&mut self) {
        self.size.store(self.guard.len(), Ordering::Relaxed);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Arc;
    use std::thread;
    use std::time::Duration;

    #[test]
    fn test_new() {
        let channel: Channel<i32> = Channel::new();
        assert!(channel.lock_read().is_empty());
        assert_eq!(channel.size_hint(), 0);
    }

    #[test]
    fn test_push_pop() {
        let channel = Channel::new();
        channel.push(42, false);
        assert_eq!(channel.size_hint(), 1);

        let val = channel.pop();
        assert_eq!(val, 42);
        assert!(channel.lock_read().is_empty());
    }

    #[test]
    fn test_try_pop() {
        let channel = Channel::new();

        // Empty channel should return None
        assert_eq!(channel.try_pop(), None);

        // After push, should return Some
        channel.push(42, false);
        assert_eq!(channel.try_pop(), Some(42));

        // Should be empty again
        assert_eq!(channel.try_pop(), None);
    }

    #[test]
    fn test_push_silent() {
        let channel = Channel::new();

        // Silent push should still add item
        channel.push(42, true);
        assert_eq!(channel.size_hint(), 1);
        assert_eq!(channel.pop(), 42);
    }

    #[test]
    fn test_size_hint() {
        let channel = Channel::new();
        assert_eq!(channel.size_hint(), 0);

        channel.push(1, false);
        channel.push(2, false);
        assert_eq!(channel.size_hint(), 2);
    }

    #[test]
    fn test_lock_read() {
        let channel = Channel::new();
        channel.push(1, false);
        channel.push(2, false);
        channel.push(3, false);

        let guard = channel.lock_read();
        assert_eq!(guard.len(), 3);
        assert_eq!(guard.get(0), Some(&1));
        assert_eq!(guard.get(1), Some(&2));
        assert_eq!(guard.get(2), Some(&3));
    }

    #[test]
    fn test_lock_rw() {
        let channel = Channel::new();
        channel.push(1, false);
        channel.push(2, false);

        let mut guard = channel.lock_rw();
        assert_eq!(guard.len(), 2);

        guard.push_back(3);
        assert_eq!(guard.len(), 3);

        assert_eq!(guard.pop_front(), Some(1));
        assert_eq!(guard.len(), 2);
    }

    #[test]
    fn test_lock_rw_clear() {
        let channel = Channel::new();
        channel.push(1, false);
        channel.push(2, false);

        {
            let mut guard = channel.lock_rw();
            guard.clear();
        }

        assert!(channel.lock_read().is_empty());
    }

    #[test]
    fn test_multi_threaded_producer_consumer() {
        let channel = Arc::new(Channel::new());
        let channel_clone = Arc::clone(&channel);

        // Producer thread
        let producer = thread::spawn(move || {
            for i in 0..10 {
                channel_clone.push(i, false);
                thread::sleep(Duration::from_millis(1));
            }
        });

        // Consumer thread
        let consumer = thread::spawn(move || {
            let mut sum = 0;
            for _ in 0..10 {
                let val = channel.pop();
                sum += val;
            }
            sum
        });

        producer.join().unwrap();
        let sum = consumer.join().unwrap();

        // Sum of 0..10 is 45
        assert_eq!(sum, 45);
    }

    #[test]
    fn test_multiple_producers() {
        let channel = Arc::new(Channel::new());
        let mut handles = vec![];

        // Spawn 5 producer threads, each pushing 10 items
        for thread_id in 0..5 {
            let channel_clone = Arc::clone(&channel);
            let handle = thread::spawn(move || {
                for i in 0..10 {
                    channel_clone.push(thread_id * 100 + i, false);
                }
            });
            handles.push(handle);
        }

        // Wait for all producers
        for handle in handles {
            handle.join().unwrap();
        }

        // Should have 50 items total
        assert_eq!(channel.size_hint(), 50);

        // Consume all items
        let mut count = 0;
        while channel.try_pop().is_some() {
            count += 1;
        }
        assert_eq!(count, 50);
    }

    #[test]
    fn test_shared_via_arc() {
        let channel = Arc::new(Channel::new());
        let channel2 = Arc::clone(&channel);

        // Push to one handle
        channel.push(42, false);

        // Should be visible through the other handle
        assert_eq!(channel2.pop(), 42);

        // Push to the other handle
        channel2.push(100, false);

        // Should be visible through the first handle
        assert_eq!(channel.pop(), 100);
    }

    #[test]
    fn test_blocking_pop_timeout() {
        let channel = Arc::new(Channel::new());
        let channel_clone = Arc::clone(&channel);

        let handle = thread::spawn(move || {
            // This will block until an item is available
            channel_clone.pop()
        });

        // Give the thread time to start waiting
        thread::sleep(Duration::from_millis(10));

        // Now push an item
        channel.push(42, false);

        // Thread should unblock and return the value
        let result = handle.join().unwrap();
        assert_eq!(result, 42);
    }

    #[test]
    fn test_iterator() {
        let channel = Channel::new();
        channel.push(1, false);
        channel.push(2, false);
        channel.push(3, false);

        let guard = channel.lock_read();
        let values: Vec<_> = guard.iter().copied().collect();
        assert_eq!(values, vec![1, 2, 3]);
    }

    #[test]
    fn test_mutable_iterator() {
        let channel = Channel::new();
        channel.push(1, false);
        channel.push(2, false);
        channel.push(3, false);

        {
            let mut guard = channel.lock_rw();
            for val in guard.iter_mut() {
                *val *= 2;
            }
        }

        assert_eq!(channel.pop(), 2);
        assert_eq!(channel.pop(), 4);
        assert_eq!(channel.pop(), 6);
    }
}
