//! Signal-catching dispatch header.
//!
//! C++ Reference:
//! - TryCatchSignal.hpp (lines 1-20)
//! - TryCatchSignal.cpp (lines 1-5)
//!
//! Faithful 1:1 port of `TryCatchSignal.{cpp,hpp}`.
//!
//! The C++ `.cpp` is a pure dispatch translation unit:
//! ```cpp
//! #include "TryCatchSignal.hpp"
//! #ifdef _MSC_VER
//! #include "TryCatchSignalSEH.cpp"
//! #endif
//! ```
//! and the `.hpp`, on every NON-MSVC compiler (Unix, MinGW, wasm, ...), defines
//! `try_catch_signal` as a template whose entire body is `fn();` — i.e. it does
//! NOT catch any signal. Only when compiled with MSVC (`_MSC_VER`) does it
//! delegate to the SEH implementation in `TryCatchSignalSEH.{cpp,hpp}` (ported
//! separately in `try_catch_signal_seh`).
//!
//! Rust has no `_MSC_VER` macro; the faithful equivalent of the MSVC gate is
//! `cfg(target_env = "msvc")`. The SEH body uses MSVC-only `__try`/`__except`,
//! so MinGW (a non-MSVC Windows env) takes the plain `fn()` path here, exactly
//! as in C++.

/// Signal type alias matching C++ `SignalT`.
///
/// TryCatchSignal.hpp:9
/// C++: using SignalT = decltype (SIGSEGV);
pub type SignalT = i32;

// On MSVC the `.cpp` pulls in the SEH translation unit, and the `.hpp` template
// forwards to `detail::try_catch_signal_seh`. Mirror that by re-exporting the
// SEH implementation (ported in `try_catch_signal_seh`).
//
// TryCatchSignal.cpp:3-5
// C++: #ifdef _MSC_VER
// C++: #include "TryCatchSignalSEH.cpp"
// C++: #endif
#[cfg(target_env = "msvc")]
pub use crate::try_catch_signal_seh::{try_catch_signal, SIGFPE, SIGILL, SIGSEGV};

/// Run `try_fn`, ignoring the requested signals and the catch handler.
///
/// TryCatchSignal.hpp:12-16
/// C++: template<class TryFn, class CatchFn, int N>
/// C++: void try_catch_signal(const SignalT (&/*sigs*/)[N], TryFn &&fn, CatchFn &&/*cfn*/)
/// C++: {
/// C++:     fn();
/// C++: }
///
/// This is the non-MSVC body verbatim: the signal array and the catch function
/// are unused (the C++ comments out their names `/*sigs*/` and `/*cfn*/`), and
/// the function simply invokes `fn()`. No signal is actually caught.
#[cfg(not(target_env = "msvc"))]
pub fn try_catch_signal<TryFn, CatchFn>(_sigs: &[SignalT], fn_: TryFn, _cfn: CatchFn)
where
    TryFn: FnOnce(),
    CatchFn: FnOnce(),
{
    // TryCatchSignal.hpp:15
    fn_();
}

// `SignalT` values come from `<csignal>` in C++ (the `.hpp` includes it via the
// `decltype(SIGSEGV)` alias). On non-MSVC targets, where this module owns the
// `try_catch_signal` definition, expose the same signal numbers used by the SEH
// port so callers have a single source of truth for the signal set.
//
// glibc/BSD/MSVC-CRT all agree on these values, so they are byte-identical to
// `<csignal>`'s `SIGILL`, `SIGFPE`, and `SIGSEGV`.
#[cfg(not(target_env = "msvc"))]
pub const SIGILL: SignalT = 4; // Illegal instruction
#[cfg(not(target_env = "msvc"))]
pub const SIGFPE: SignalT = 8; // Floating-point exception
#[cfg(not(target_env = "msvc"))]
pub const SIGSEGV: SignalT = 11; // Segmentation fault

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_try_catch_signal_success() {
        // The non-MSVC body just runs `fn()`; the catch fn is never invoked.
        let mut executed = false;
        let mut caught = false;

        let sigs = [SIGSEGV, SIGILL, SIGFPE];

        try_catch_signal(
            &sigs,
            || {
                executed = true;
            },
            || {
                caught = true;
            },
        );

        assert!(executed, "Try function should execute");
        #[cfg(not(target_env = "msvc"))]
        assert!(!caught, "Catch function is never called on non-MSVC targets");
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

    #[cfg(not(target_env = "msvc"))]
    #[test]
    fn test_signal_constants() {
        // The constants mirror `<csignal>` and must stay distinct and positive.
        assert_eq!(SIGILL, 4);
        assert_eq!(SIGFPE, 8);
        assert_eq!(SIGSEGV, 11);

        assert_ne!(SIGSEGV, SIGILL);
        assert_ne!(SIGSEGV, SIGFPE);
        assert_ne!(SIGILL, SIGFPE);
    }
}
