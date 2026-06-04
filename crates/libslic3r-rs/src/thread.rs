//! Thread utilities for naming and managing threads
//!
//! C++ Reference:
//! - Thread.hpp (lines 1-65)
//! - Thread.cpp (lines 1-200+)
//!
//! This module provides utilities for thread naming and management.
//! The C++ version has complex platform-specific code for Windows and POSIX
//! thread naming. Rust's std::thread provides simpler cross-platform naming.

use std::sync::{Mutex, OnceLock};
use std::thread::{self, ThreadId};

/// Cached main thread ID
///
/// Thread.cpp: static std::thread::id s_main_thread_id
static MAIN_THREAD_ID: OnceLock<ThreadId> = OnceLock::new();

/// Flag to track if main thread is active
///
/// Thread.cpp: static std::atomic<bool> s_is_main_thread_active(false)
static MAIN_THREAD_ACTIVE: Mutex<bool> = Mutex::new(false);

/// Save the current thread ID as the main (UI) thread ID
///
/// Thread.hpp:27
/// C++: void save_main_thread_id();
///
/// Thread.cpp: (implementation with std::thread::id storage)
pub fn save_main_thread_id() {
    let _ = MAIN_THREAD_ID.set(thread::current().id());
    *MAIN_THREAD_ACTIVE.lock().unwrap() = true;
}

/// Get the cached main (UI) thread ID
///
/// Thread.hpp:29
/// C++: boost::thread::id get_main_thread_id();
///
/// Returns the main thread ID if it has been saved, panics otherwise.
pub fn get_main_thread_id() -> ThreadId {
    *MAIN_THREAD_ID
        .get()
        .expect("Main thread ID not initialized - call save_main_thread_id() first")
}

/// Check if the main (UI) thread is active
///
/// Thread.hpp:31
/// C++: bool is_main_thread_active();
///
/// Thread.cpp: (implementation checking atomic flag)
pub fn is_main_thread_active() -> bool {
    *MAIN_THREAD_ACTIVE.lock().unwrap()
}

/// Check if the current thread is the main thread
///
/// Utility function (not in C++ API but useful)
pub fn is_current_thread_main() -> bool {
    if let Some(main_id) = MAIN_THREAD_ID.get() {
        thread::current().id() == *main_id
    } else {
        false
    }
}

/// Set the name of the current thread
///
/// Thread.hpp:25
/// C++: bool set_current_thread_name(const char *thread_name);
///
/// Thread.cpp: (complex platform-specific implementation)
///
/// Note: In Rust, thread names must be set when creating the thread via
/// thread::Builder. This function is a no-op stub for API compatibility.
/// To name a thread in Rust, use:
///   thread::Builder::new().name("thread_name".to_string()).spawn(|| { ... })
pub fn set_current_thread_name(_name: &str) -> bool {
    // Rust doesn't support setting thread names after creation.
    // Thread names must be set via thread::Builder::new().name()
    false
}

/// Create a named thread with appropriate stack size
///
/// Thread.hpp:49-56
/// C++: template<class Fn> inline boost::thread create_thread(Fn &&fn)
/// C++: {
/// C++:     boost::thread::attributes attrs;
/// C++:     return create_thread(attrs, std::forward<Fn>(fn));
/// C++: }
///
/// Thread.hpp:43-48
/// C++: template<class Fn>
/// C++: inline boost::thread create_thread(boost::thread::attributes &attrs, Fn &&fn)
/// C++: {
/// C++:     attrs.set_stack_size((sizeof(void*) == 4) ? (2048 * 1024) : (4096 * 1024));
/// C++:     return boost::thread{attrs, std::forward<Fn>(fn)};
/// C++: }
///
/// In Rust, we use thread::Builder with appropriate stack size.
pub fn create_thread<F, T>(name: &str, f: F) -> thread::JoinHandle<T>
where
    F: FnOnce() -> T + Send + 'static,
    T: Send + 'static,
{
    // Match C++ stack size: 4MB on 64-bit, 2MB on 32-bit
    let stack_size = if cfg!(target_pointer_width = "64") {
        4 * 1024 * 1024 // 4MB
    } else {
        2 * 1024 * 1024 // 2MB
    };

    thread::Builder::new()
        .name(name.to_string())
        .stack_size(stack_size)
        .spawn(f)
        .expect("Failed to spawn thread")
}

/// Initialize TBB thread pool thread names (no-op in Rust)
///
/// Thread.hpp:37
/// C++: void name_tbb_thread_pool_threads_set_locale();
///
/// Thread.cpp: (sets thread names and locale for TBB worker threads)
///
/// Note: Rust uses Rayon for parallelism, which manages its own thread pool.
/// This is a no-op for API compatibility.
pub fn name_tbb_thread_pool_threads_set_locale() {
    // No-op: Rust/Rayon handles thread pool internally
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::{AtomicBool, Ordering};
    use std::sync::Arc;

    #[test]
    fn test_save_and_get_main_thread_id() {
        save_main_thread_id();
        let main_id = get_main_thread_id();
        assert_eq!(main_id, thread::current().id());
    }

    #[test]
    fn test_is_main_thread_active() {
        save_main_thread_id();
        assert!(is_main_thread_active());
    }

    #[test]
    fn test_is_current_thread_main() {
        save_main_thread_id();
        assert!(is_current_thread_main());

        // Spawn a worker thread and check it's not main
        let handle = thread::spawn(|| !is_current_thread_main());

        assert!(handle.join().unwrap());
    }

    #[test]
    fn test_create_thread() {
        let flag = Arc::new(AtomicBool::new(false));
        let flag_clone = flag.clone();

        let handle = create_thread("test_thread", move || {
            flag_clone.store(true, Ordering::SeqCst);
        });

        handle.join().unwrap();
        assert!(flag.load(Ordering::SeqCst));
    }

    #[test]
    fn test_create_thread_with_return_value() {
        let handle = create_thread("test_return", || 42);

        let result = handle.join().unwrap();
        assert_eq!(result, 42);
    }

    #[test]
    fn test_create_thread_stack_size() {
        // Just verify it doesn't crash with the large stack
        let handle = create_thread("test_stack", || {
            // Allocate a reasonably large array on the stack
            let _large_array: [u8; 1024 * 1024] = [0; 1024 * 1024];
        });

        handle.join().unwrap();
    }

    #[test]
    fn test_set_current_thread_name_returns_false() {
        // This always returns false in Rust (not supported)
        assert!(!set_current_thread_name("test"));
    }

    #[test]
    fn test_name_tbb_thread_pool_threads_no_op() {
        // Should not panic
        name_tbb_thread_pool_threads_set_locale();
    }

    #[test]
    fn test_multiple_threads_have_different_ids() {
        save_main_thread_id();
        let main_id = get_main_thread_id();

        let handle1 = create_thread("thread1", || thread::current().id());
        let handle2 = create_thread("thread2", || thread::current().id());

        let id1 = handle1.join().unwrap();
        let id2 = handle2.join().unwrap();

        assert_ne!(id1, id2);
        assert_ne!(id1, main_id);
        assert_ne!(id2, main_id);
    }
}
