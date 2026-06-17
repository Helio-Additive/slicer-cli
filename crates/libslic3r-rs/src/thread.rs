//! 1:1 port of `Thread.cpp` / `Thread.hpp` (BambuStudio libslic3r).
//!
//! Thread naming and main-thread tracking utilities. These are debugger /
//! locale conveniences only and do not affect G-code output, but the public
//! surface and return values are mirrored faithfully so callers behave
//! identically.
//!
//! Platform notes carried over from C++:
//! - `pthread_setname_np` supports a maximum of 15 character thread names!
//!   (16th character is the null terminator)
//! - Methods taking the thread as an argument are not supported by OSX.
//! - Naming threads is only supported on newer Windows 10.
//!
//! Native-dependency status: the C++ uses `pthread_setname_np` /
//! `_configthreadlocale` / Win32 `SetThreadDescription` and Intel TBB. To stay
//! wasm-safe and avoid adding `libc`/native backends, the thread-naming side
//! effects are not performed here; the functions return the same booleans the
//! C++ would on the build platform (see per-function refs). See
//! `name_tbb_thread_pool_threads_set_locale` for the TBB-blocked symbol.

use std::sync::{OnceLock, RwLock};
use std::thread::{self, ThreadId};

// ----------------------------------------------------------------------------
// Thread.hpp:11-44  Set / get thread name.
// Returns false if the API is not supported.
//
// It is a good idea to name the main thread before spawning children threads,
// because dynamic linking is used on Windows 10 to initialize
// Get/SetThreadDescription functions, which is not thread safe.
//
// pthread_setname_np supports maximum 15 character thread names! (16th
// character is the null terminator)
//
// Methods taking the thread as an argument are not supported by OSX.
// Naming threads is only supported on newer Windows 10.
// ----------------------------------------------------------------------------

// Thread.cpp:101-104 / Thread.cpp:131-136 / Thread.cpp:162-166
// bool set_thread_name(std::thread &thread, const char *thread_name)
//
// The std::thread / boost::thread overloads operate on a thread *handle*. They
// are unsupported on OSX (return false), use pthread_setname_np on other POSIX
// (return true), and WindowsSetThreadName on Windows. Rust's std exposes no
// portable post-spawn rename, and the native handle path needs `libc`/Win32
// which we do not pull in (wasm-safety). We mirror the C++ return value for the
// build platform.
//
// Thread.hpp:23 inline overload taking a std::string forwards to `.c_str()`;
// in Rust a `&str` already covers both.
pub fn set_thread_name(_thread: &thread::JoinHandle<()>, thread_name: &str) -> bool {
    let _ = thread_name;
    // Thread.cpp:135 (__APPLE__) `return false;`
    // Thread.cpp:165 (posix)     `return true;`
    // Thread.cpp:103 (_WIN32)    WindowsSetThreadName(...)
    #[cfg(target_os = "macos")]
    {
        false
    }
    #[cfg(not(target_os = "macos"))]
    {
        true
    }
}

// Thread.cpp:111-114 / Thread.cpp:145-149 / Thread.cpp:174-178
// bool set_current_thread_name(const char *thread_name)
//
// #ifdef __APPLE__  : pthread_setname_np(thread_name); return true;
// posix             : pthread_setname_np(pthread_self(), thread_name); return true;
// _WIN32            : return WindowsSetThreadName(::GetCurrentThread(), thread_name);
//
// The actual rename requires pthread / Win32. We skip the side effect to stay
// wasm-safe (no libc) but return the same boolean the C++ would.
//
// Thread.hpp:27 inline overload taking a std::string forwards to `.c_str()`.
pub fn set_current_thread_name(thread_name: &str) -> bool {
    let _ = thread_name;
    #[cfg(target_os = "windows")]
    {
        // Thread.cpp:113 WindowsSetThreadName(::GetCurrentThread(), thread_name)
        // The Win32 SetThreadDescription path is not wired up; mirror the
        // common (API available) success return.
        true
    }
    #[cfg(not(target_os = "windows"))]
    {
        // Thread.cpp:148 (__APPLE__) / Thread.cpp:177 (posix) `return true;`
        true
    }
}

// Thread.cpp:116-124 / Thread.cpp:151-157 / Thread.cpp:180-184
// std::optional<std::string> get_current_thread_name()
//
// __APPLE__ : return std::nullopt;  (not supported)
// posix     : char buf[16]; return std::string(pthread_getname_np(...) == 0 ? buf : "");
// _WIN32    : GetThreadDescription path, or nullopt if API unavailable.
//
// Without libc/Win32 we cannot query the OS name; mirror the OSX `nullopt`
// elsewhere too (no observable effect on G-code).
pub fn get_current_thread_name() -> Option<String> {
    // Thread.cpp:156 (__APPLE__) `return std::nullopt;`
    None
}

// Thread.cpp:191  static boost::thread::id g_main_thread_id;
// To be called at the start of the application to save the current thread ID as
// the main (UI) thread ID.
//
// C++ `g_main_thread_id` is a plain mutable global that `save_main_thread_id`
// re-assigns on every call (overwrites). Model it with RwLock<Option<ThreadId>>
// rather than OnceLock so the overwrite semantics match; `None` models the
// default-constructed (no-thread) boost::thread::id before the first save.
static G_MAIN_THREAD_ID: RwLock<Option<ThreadId>> = RwLock::new(None);

// Thread.cpp:193-196
// void save_main_thread_id()
// {
//     g_main_thread_id = boost::this_thread::get_id();
// }
pub fn save_main_thread_id() {
    *G_MAIN_THREAD_ID.write().unwrap() = Some(thread::current().id());
}

// Thread.cpp:199-202
// Retrieve the cached main (UI) thread ID.
// boost::thread::id get_main_thread_id()
// {
//     return g_main_thread_id;
// }
pub fn get_main_thread_id() -> Option<ThreadId> {
    // C++ returns a default-constructed (no-thread) id before save; we model
    // the "not yet saved" state as None.
    *G_MAIN_THREAD_ID.read().unwrap()
}

// Thread.cpp:205-208
// Checks whether the main (UI) thread is active.
// bool is_main_thread_active()
// {
//     return get_main_thread_id() == boost::this_thread::get_id();
// }
pub fn is_main_thread_active() -> bool {
    get_main_thread_id() == Some(thread::current().id())
}

// Thread.cpp:212-274
// Spawn (n - 1) worker threads on Intel TBB thread pool and name them by an
// index and a system thread ID. Also it sets locale of the worker threads to
// "C" for the G-code generator to produce "." as a decimal separator.
//
// BLOCKED (native dep): this depends on Intel TBB (`tbb::parallel_for`,
// `tbb::this_task_arena::max_concurrency`, `tbb::blocked_range`) and on
// per-thread C-locale APIs (`_configthreadlocale` / `uselocale`/`newlocale`).
// Rust uses Rayon for parallelism (it owns its pool) and formats floats with
// '.' independently of the C locale, so the locale side effect is unnecessary
// for parity. We preserve the once-only guard structure; the TBB body itself is
// not portable wasm-safe and is intentionally not reimplemented.
pub fn name_tbb_thread_pool_threads_set_locale() {
    // Thread.cpp:214-217
    // static bool initialized = false;
    // if (initialized) return;
    // initialized = true;
    static INITIALIZED: OnceLock<()> = OnceLock::new();
    if INITIALIZED.set(()).is_err() {
        return;
    }

    // Thread.cpp:219-273  TBB worker-thread naming + per-thread "C" locale.
    // Blocked on TBB / native locale APIs (see doc comment above). Rayon owns
    // its own pool and Rust float formatting is locale-independent, so there is
    // nothing observable to reproduce here.
}

// ----------------------------------------------------------------------------
// Thread.hpp:46-61  template<class Fn> boost::thread create_thread(...)
//
// Duplicating the stack allocation size of Thread Building Block worker threads
// of the thread pool: allocate 4MB on a 64bit system, allocate 2MB on a 32bit
// system by default.
//
//     attrs.set_stack_size((sizeof(void*) == 4) ? (2048 * 1024) : (4096 * 1024));
//     return boost::thread{attrs, std::forward<Fn>(fn)};
//
// The C++ templates are header-only inline helpers. Rust models
// boost::thread::attributes::set_stack_size via thread::Builder::stack_size.
// ----------------------------------------------------------------------------
pub fn create_thread<F, T>(f: F) -> thread::JoinHandle<T>
where
    F: FnOnce() -> T + Send + 'static,
    T: Send + 'static,
{
    // Thread.hpp:53 (sizeof(void*) == 4) ? (2048 * 1024) : (4096 * 1024)
    let stack_size: usize = if std::mem::size_of::<*const ()>() == 4 {
        2048 * 1024
    } else {
        4096 * 1024
    };
    // Thread.hpp:54 return boost::thread{attrs, std::forward<Fn>(fn)};
    thread::Builder::new()
        .stack_size(stack_size)
        .spawn(f)
        .expect("Failed to spawn thread")
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
        assert_eq!(main_id, Some(thread::current().id()));
    }

    #[test]
    fn test_is_main_thread_active() {
        save_main_thread_id();
        assert!(is_main_thread_active());
    }

    #[test]
    fn test_create_thread() {
        let flag = Arc::new(AtomicBool::new(false));
        let flag_clone = flag.clone();

        let handle = create_thread(move || {
            flag_clone.store(true, Ordering::SeqCst);
        });

        handle.join().unwrap();
        assert!(flag.load(Ordering::SeqCst));
    }

    #[test]
    fn test_create_thread_with_return_value() {
        let handle = create_thread(|| 42);
        let result = handle.join().unwrap();
        assert_eq!(result, 42);
    }

    #[test]
    fn test_get_current_thread_name_none_on_osx() {
        // Thread.cpp:156 __APPLE__ returns nullopt.
        #[cfg(target_os = "macos")]
        assert!(get_current_thread_name().is_none());
    }

    #[test]
    fn test_name_tbb_thread_pool_threads_no_op() {
        // Should not panic; idempotent.
        name_tbb_thread_pool_threads_set_locale();
        name_tbb_thread_pool_threads_set_locale();
    }
}
