//! Thread-safe message passing channel with blocking and non-blocking operations
//!
//! C++ Reference:
//! - Channel.hpp
//!
//! This module provides a thread-safe queue for passing messages between threads.
//! Similar to std::mpsc::channel but with additional features like locked access
//! and non-blocking try_pop.

use std::collections::VecDeque;
use std::sync::{Arc, Condvar, Mutex, MutexGuard};

/// Thread-safe message passing channel
/// Channel.hpp:15-105
///
/// A queue that allows multiple producers and consumers to safely exchange
/// messages. Provides both blocking (pop) and non-blocking (try_pop) operations,
/// as well as locked access to the entire queue for batch operations.
///
/// C++: template<class T> class Channel
pub struct Channel<T> {
    /// Internal queue protected by mutex
    /// Channel.hpp:99
    /// C++: Queue m_queue;
    queue: Arc<Mutex<VecDeque<T>>>,

    /// Condition variable for blocking operations
    /// Channel.hpp:101
    /// C++: std::condition_variable m_condition;
    condvar: Arc<Condvar>,
}

impl<T> Channel<T> {
    /// Create a new empty channel
    /// Channel.hpp:42-43
    /// C++: Channel() {}
    pub fn new() -> Self {
        Channel {
            queue: Arc::new(Mutex::new(VecDeque::new())),
            condvar: Arc::new(Condvar::new()),
        }
    }

    /// Push an item onto the channel
    /// Channel.hpp:45-51
    /// C++: void push(const T& item, bool silent = false)
    pub fn push(&self, item: T, silent: bool) {
        {
            /// Lock and push item
            /// Channel.hpp:47-49
            /// C++: UniqueLock lock(m_mutex);
            /// C++: m_queue.push_back(item);
            let mut queue = self.queue.lock().unwrap();
            queue.push_back(item);
        }

        /// Notify waiting threads unless silent
        /// Channel.hpp:50
        /// C++: if (! silent) { m_condition.notify_one(); }
        if !silent {
            self.condvar.notify_one();
        }
    }

    /// Pop an item from the channel (blocking)
    /// Waits until an item is available
    /// Channel.hpp:59-65
    /// C++: T pop()
    pub fn pop(&self) -> T {
        /// Lock the queue
        /// Channel.hpp:61
        /// C++: UniqueLock lock(m_mutex);
        let mut queue = self.queue.lock().unwrap();

        /// Wait for queue to have items
        /// Channel.hpp:62
        /// C++: m_condition.wait(lock, [this]() { return !m_queue.empty(); });
        while queue.is_empty() {
            queue = self.condvar.wait(queue).unwrap();
        }

        /// Remove and return front item
        /// Channel.hpp:63-64
        /// C++: auto item = std::move(m_queue.front());
        /// C++: m_queue.pop_front();
        queue.pop_front().unwrap()
    }

    /// Try to pop an item without blocking
    /// Returns None if queue is empty
    /// Channel.hpp:67-75
    /// C++: boost::optional<T> try_pop()
    pub fn try_pop(&self) -> Option<T> {
        /// Lock the queue
        /// Channel.hpp:69
        /// C++: UniqueLock lock(m_mutex);
        let mut queue = self.queue.lock().unwrap();

        /// Return None if empty, otherwise pop front
        /// Channel.hpp:70-74
        /// C++: if (m_queue.empty()) { return boost::none; }
        /// C++: else { auto item = std::move(m_queue.front()); m_queue.pop(); return item; }
        queue.pop_front()
    }

    /// Get approximate size (thread-unsafe hint)
    /// Channel.hpp:78
    /// C++: size_t size_hint() const noexcept { return m_queue.size(); }
    pub fn size_hint(&self) -> usize {
        // Note: This is deliberately lock-free and may return stale data
        // The C++ version warns: "Thread unsafe! Keep in mind you need to re-verify the result after locking."
        if let Ok(queue) = self.queue.try_lock() {
            queue.len()
        } else {
            0 // If we can't acquire lock, return 0 as a safe default
        }
    }

    /// Check if the channel is empty (thread-unsafe hint)
    pub fn is_empty_hint(&self) -> bool {
        self.size_hint() == 0
    }

    /// Lock the queue for read-only batch access
    /// Returns a guard that unlocks when dropped
    /// Channel.hpp:80-83
    /// C++: LockedConstPtr lock_read() const
    pub fn lock_read(&self) -> ChannelReadGuard<T> {
        ChannelReadGuard {
            guard: self.queue.lock().unwrap(),
        }
    }

    /// Lock the queue for read-write batch access
    /// Returns a guard that unlocks when dropped
    /// Channel.hpp:85-88
    /// C++: LockedPtr lock_rw()
    pub fn lock_rw(&self) -> ChannelWriteGuard<T> {
        ChannelWriteGuard {
            guard: self.queue.lock().unwrap(),
        }
    }

    /// Clear all items from the channel
    pub fn clear(&self) {
        let mut queue = self.queue.lock().unwrap();
        queue.clear();
    }

    /// Get the exact current size (thread-safe)
    pub fn len(&self) -> usize {
        let queue = self.queue.lock().unwrap();
        queue.len()
    }

    /// Check if the channel is empty (thread-safe)
    pub fn is_empty(&self) -> bool {
        let queue = self.queue.lock().unwrap();
        queue.is_empty()
    }
}

impl<T> Default for Channel<T> {
    fn default() -> Self {
        Self::new()
    }
}

impl<T> Clone for Channel<T> {
    /// Clone creates a new handle to the same channel
    fn clone(&self) -> Self {
        Channel {
            queue: Arc::clone(&self.queue),
            condvar: Arc::clone(&self.condvar),
        }
    }
}

/// RAII guard for read-only access to the channel queue
/// Channel.hpp:36
/// C++: using LockedConstPtr = std::unique_ptr<const Queue, Unlocker<const Queue>>;
pub struct ChannelReadGuard<'a, T> {
    guard: MutexGuard<'a, VecDeque<T>>,
}

impl<'a, T> ChannelReadGuard<'a, T> {
    /// Get the queue length
    pub fn len(&self) -> usize {
        self.guard.len()
    }

    /// Check if queue is empty
    pub fn is_empty(&self) -> bool {
        self.guard.is_empty()
    }

    /// Iterate over items
    pub fn iter(&self) -> std::collections::vec_deque::Iter<T> {
        self.guard.iter()
    }

    /// Get item at index
    pub fn get(&self, index: usize) -> Option<&T> {
        self.guard.get(index)
    }
}

impl<'a, T> std::ops::Deref for ChannelReadGuard<'a, T> {
    type Target = VecDeque<T>;

    fn deref(&self) -> &Self::Target {
        &self.guard
    }
}

/// RAII guard for read-write access to the channel queue
/// Channel.hpp:37
/// C++: using LockedPtr = std::unique_ptr<Queue, Unlocker<Queue>>;
pub struct ChannelWriteGuard<'a, T> {
    guard: MutexGuard<'a, VecDeque<T>>,
}

impl<'a, T> ChannelWriteGuard<'a, T> {
    /// Get the queue length
    pub fn len(&self) -> usize {
        self.guard.len()
    }

    /// Check if queue is empty
    pub fn is_empty(&self) -> bool {
        self.guard.is_empty()
    }

    /// Iterate over items
    pub fn iter(&self) -> std::collections::vec_deque::Iter<T> {
        self.guard.iter()
    }

    /// Iterate mutably over items
    pub fn iter_mut(&mut self) -> std::collections::vec_deque::IterMut<T> {
        self.guard.iter_mut()
    }

    /// Push an item to the back
    pub fn push_back(&mut self, item: T) {
        self.guard.push_back(item);
    }

    /// Pop an item from the front
    pub fn pop_front(&mut self) -> Option<T> {
        self.guard.pop_front()
    }

    /// Clear all items
    pub fn clear(&mut self) {
        self.guard.clear();
    }
}

impl<'a, T> std::ops::Deref for ChannelWriteGuard<'a, T> {
    type Target = VecDeque<T>;

    fn deref(&self) -> &Self::Target {
        &self.guard
    }
}

impl<'a, T> std::ops::DerefMut for ChannelWriteGuard<'a, T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.guard
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
        assert!(channel.is_empty());
        assert_eq!(channel.len(), 0);
    }

    #[test]
    fn test_push_pop() {
        let channel = Channel::new();
        channel.push(42, false);
        assert_eq!(channel.len(), 1);

        let val = channel.pop();
        assert_eq!(val, 42);
        assert!(channel.is_empty());
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
        assert_eq!(channel.len(), 1);
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
    fn test_clear() {
        let channel = Channel::new();
        channel.push(1, false);
        channel.push(2, false);
        channel.push(3, false);

        channel.clear();
        assert!(channel.is_empty());
        assert_eq!(channel.len(), 0);
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

        assert!(channel.is_empty());
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
        assert_eq!(channel.len(), 50);

        // Consume all items
        let mut count = 0;
        while channel.try_pop().is_some() {
            count += 1;
        }
        assert_eq!(count, 50);
    }

    #[test]
    fn test_clone_shares_queue() {
        let channel1 = Channel::new();
        let channel2 = channel1.clone();

        // Push to channel1
        channel1.push(42, false);

        // Should be visible in channel2
        assert_eq!(channel2.pop(), 42);

        // Push to channel2
        channel2.push(100, false);

        // Should be visible in channel1
        assert_eq!(channel1.pop(), 100);
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
