//! Signal handling utilities for catching crashes and exceptions
//!
//! C++ Reference:
//! - TryCatchSignal.hpp (lines 1-20)
//! - TryCatchSignal.cpp (lines 1-5)
//!
//! This module provides platform-specific signal handling to catch crashes
//! like segmentation faults, illegal instructions, and floating-point errors.
//! On Unix-like systems, it uses signal handlers; on Windows it delegates to
//! the SEH (Structured Exception Handling) implementation.
//!
//! **NOTE:** The Rust implementation is simplified compared to C++. True signal
//! handling requires unsafe FFI and platform-specific crates. This version
//! provides the API surface for compatibility but does not catch actual signals.

use crate::{Error, Result};

/// Signal type alias matching C++ SignalT
///
/// TryCatchSignal.hpp:9
/// C++: using SignalT = decltype (SIGSEGV);
pub type SignalT = i32;

#[cfg(windows)]
pub use crate::try_catch_signal_seh::{try_catch_signal, SIGFPE, SIGILL, SIGSEGV};

/// Execute a function with signal catching (Unix implementation)
///
/// TryCatchSignal.hpp:11-15
/// C++: template<class TryFn, class CatchFn, int N>
/// C++: void try_catch_signal(const SignalT (&/*sigs*/)[N], TryFn &&fn, CatchFn &&/*cfn*/)
/// C++: {
/// C++:     fn();
/// C++: }
///
/// Note: The C++ Unix implementation doesn't actually catch signals - it just
/// runs the function directly. This is a faithful port of that behavior.
#[cfg(not(windows))]
pub fn try_catch_signal<TryFn, CatchFn>(_signals: &[SignalT], try_fn: TryFn, _catch_fn: CatchFn)
where
    TryFn: FnOnce(),
    CatchFn: FnOnce(),
{
    // C++ implementation on Unix just calls fn() without any signal handling
    // TryCatchSignal.hpp:14
    try_fn();
}

// Common signal constants (Unix values)
#[cfg(not(windows))]
pub const SIGSEGV: SignalT = 11;
#[cfg(not(windows))]
pub const SIGILL: SignalT = 4;
#[cfg(not(windows))]
pub const SIGFPE: SignalT = 8;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_try_catch_signal_success() {
        let mut executed = false;
        let mut caught = false;

        let signals = [SIGSEGV, SIGILL, SIGFPE];

        try_catch_signal(
            &signals,
            || {
                executed = true;
            },
            || {
                caught = true;
            },
        );

        assert!(executed, "Try function should execute");
        // On Unix without handler, catch function is never called
        #[cfg(unix)]
        assert!(!caught, "Catch function should not be called on success");
    }

    #[test]
    fn test_try_catch_signal_empty_signals() {
        let mut executed = false;

        try_catch_signal(
            &[],
            || {
                executed = true;
            },
            || {},
        );

        assert!(executed);
    }

    #[cfg(not(windows))]
    #[test]
    fn test_signal_constants() {
        // Just verify the constants are defined and distinct
        assert_ne!(SIGSEGV, SIGILL);
        assert_ne!(SIGSEGV, SIGFPE);
        assert_ne!(SIGILL, SIGFPE);

        // Typical Unix values (may vary by platform)
        assert!(SIGSEGV > 0);
        assert!(SIGILL > 0);
        assert!(SIGFPE > 0);
    }
}
