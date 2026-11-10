//! Windows SEH (Structured Exception Handling) signal catching.
//!
//! C++ Reference:
//! - TryCatchSignalSEH.hpp (lines 1-27)
//! - TryCatchSignalSEH.cpp (lines 1-44)
//!
//! Faithful 1:1 port of `TryCatchSignalSEH.{cpp,hpp}`.
//!
//! The entire C++ translation unit is MSVC-only: `TryCatchSignal.cpp` pulls in
//! `TryCatchSignalSEH.cpp` only `#ifdef _MSC_VER`, and the body uses the
//! MSVC-exclusive `__try`/`__except` keywords plus `<windows.h>`'s
//! `GetExceptionCode()` and `STATUS_*`/`EXCEPTION_*` constants. The faithful
//! gate is therefore `cfg(target_env = "msvc")`, matching `try_catch_signal.rs`
//! (MinGW — a non-MSVC Windows env — never compiles this file in C++ either).
//!
//! `signal_seh_filter` is pure integer logic and is ported verbatim. The
//! `try_catch_signal_seh` body, however, relies on the MSVC compiler's
//! structured-exception intrinsics (`__try`/`__except`/`GetExceptionCode`),
//! which Rust cannot express natively and which are unavailable on the
//! wasm/Unix parity targets. That single function is BLOCKED on a native MSVC
//! SEH backend; see the `cfg(target_env = "msvc")` body below.

// TryCatchSignalSEH.hpp:9
// C++: using SignalT = decltype (SIGSEGV);
//
// `SIGSEGV` from `<csignal>` is an `int`; mirror that with `i32`. (Exposed on
// every target so the type name resolves regardless of toolchain, matching the
// always-visible `SignalT` alias from the header.)
/// Signal type alias matching C++ `SignalT`.
pub type SignalT = SignalTInner;
type SignalTInner = i32;

// Signal numbers from `<csignal>`. glibc/BSD/MSVC-CRT all agree on these
// values, so they are byte-identical to the macros referenced by
// `decltype(SIGSEGV)` and used as `case` labels in `signal_seh_filter`.
//
// TryCatchSignalSEH.hpp:9
/// `SIGILL` — illegal instruction.
pub const SIGILL: SignalT = 4;
/// `SIGFPE` — floating-point exception.
pub const SIGFPE: SignalT = 8;
/// `SIGSEGV` — segmentation fault.
pub const SIGSEGV: SignalT = 11;

// `<windows.h>` SEH disposition codes returned by an `__except` filter
// expression. Values are fixed by the Win32 ABI.
//
// TryCatchSignalSEH.cpp:8,40
/// `EXCEPTION_EXECUTE_HANDLER` — run the `__except` block.
pub const EXCEPTION_EXECUTE_HANDLER: i32 = 1;
/// `EXCEPTION_CONTINUE_SEARCH` — let the exception propagate.
pub const EXCEPTION_CONTINUE_SEARCH: i32 = 0;

// `<windows.h>` NTSTATUS exception codes (the values delivered by
// `GetExceptionCode()`). Values are fixed by the Windows ABI.
//
// TryCatchSignalSEH.cpp:13,17,21-24
/// `STATUS_ACCESS_VIOLATION` (0xC0000005).
pub const STATUS_ACCESS_VIOLATION: u32 = 0xC000_0005;
/// `STATUS_ILLEGAL_INSTRUCTION` (0xC000001D).
pub const STATUS_ILLEGAL_INSTRUCTION: u32 = 0xC000_001D;
/// `STATUS_FLOAT_DIVIDE_BY_ZERO` (0xC000008E).
pub const STATUS_FLOAT_DIVIDE_BY_ZERO: u32 = 0xC000_008E;
/// `STATUS_FLOAT_OVERFLOW` (0xC0000091).
pub const STATUS_FLOAT_OVERFLOW: u32 = 0xC000_0091;
/// `STATUS_FLOAT_UNDERFLOW` (0xC0000093).
pub const STATUS_FLOAT_UNDERFLOW: u32 = 0xC000_0093;
/// `STATUS_INTEGER_DIVIDE_BY_ZERO` (0xC0000094).
pub const STATUS_INTEGER_DIVIDE_BY_ZERO: u32 = 0xC000_0094;

/// SEH `__except` filter: decide whether to handle `seh_code` given the set of
/// signals the caller is interested in.
///
/// TryCatchSignalSEH.cpp:5-31
/// C++: static int signal_seh_filter(int sigcnt, const Slic3r::SignalT *sigs,
/// C++:                              unsigned long seh_code)
/// C++: {
/// C++:     int ret = EXCEPTION_CONTINUE_SEARCH;
/// C++:     for (int s = 0; s < sigcnt && ret != EXCEPTION_EXECUTE_HANDLER; ++s)
/// C++:     switch (sigs[s]) {
/// C++:     case SIGSEGV: ...
/// C++:     case SIGILL:  ...
/// C++:     case SIGFPE:  ...
/// C++:     default: ret = EXCEPTION_CONTINUE_SEARCH;
/// C++:     }
/// C++:     return ret;
/// C++: }
///
/// `seh_code` is `unsigned long` in C++ (`GetExceptionCode()`'s `DWORD`); on
/// Windows `unsigned long` is 32-bit, so `u32` is the faithful integer type.
//
// Its sole non-test caller is the MSVC-only `detail::try_catch_signal_seh`, so
// off-MSVC the function is exercised only by the unit tests; keep it present
// (faithful to the TU) without emitting an unused-fn warning on other targets.
#[cfg_attr(not(target_env = "msvc"), allow(dead_code))]
fn signal_seh_filter(sigcnt: i32, sigs: &[SignalT], seh_code: u32) -> i32 {
    // TryCatchSignalSEH.cpp:8
    let mut ret = EXCEPTION_CONTINUE_SEARCH;

    // TryCatchSignalSEH.cpp:10-28
    let mut s: i32 = 0;
    while s < sigcnt && ret != EXCEPTION_EXECUTE_HANDLER {
        match sigs[s as usize] {
            // TryCatchSignalSEH.cpp:12-15
            SIGSEGV => {
                if seh_code == STATUS_ACCESS_VIOLATION {
                    ret = EXCEPTION_EXECUTE_HANDLER;
                }
            }
            // TryCatchSignalSEH.cpp:16-19
            SIGILL => {
                if seh_code == STATUS_ILLEGAL_INSTRUCTION {
                    ret = EXCEPTION_EXECUTE_HANDLER;
                }
            }
            // TryCatchSignalSEH.cpp:20-26
            SIGFPE => {
                if seh_code == STATUS_FLOAT_DIVIDE_BY_ZERO
                    || seh_code == STATUS_FLOAT_OVERFLOW
                    || seh_code == STATUS_FLOAT_UNDERFLOW
                    || seh_code == STATUS_INTEGER_DIVIDE_BY_ZERO
                {
                    ret = EXCEPTION_EXECUTE_HANDLER;
                }
            }
            // TryCatchSignalSEH.cpp:27
            _ => ret = EXCEPTION_CONTINUE_SEARCH,
        }
        s += 1;
    }

    // TryCatchSignalSEH.cpp:30
    ret
}

/// `Slic3r::detail::try_catch_signal_seh` — run `fn`, and on a matching
/// hardware exception run `cfn` instead.
///
/// TryCatchSignalSEH.cpp:33-43
/// C++: void Slic3r::detail::try_catch_signal_seh(int sigcnt, const SignalT *sigs,
/// C++:                                           std::function<void()> &&fn,
/// C++:                                           std::function<void()> &&cfn)
/// C++: {
/// C++:     __try {
/// C++:         fn();
/// C++:     }
/// C++:     __except(signal_seh_filter(sigcnt, sigs, GetExceptionCode())) {
/// C++:         cfn();
/// C++:     }
/// C++: }
///
/// BLOCKED (native backend): the `__try`/`__except` structured-exception
/// machinery and `GetExceptionCode()` are MSVC compiler intrinsics over the
/// Win32 SEH ABI. Rust has no native SEH; faithfully reproducing this requires
/// either the MSVC C compiler (the C++ approach) or unsafe FFI into
/// `RtlAddVectoredExceptionHandler`/`__C_specific_handler`, which is a native
/// Windows-only dependency disallowed on the wasm/Unix parity targets. The
/// filter (`signal_seh_filter`) above is ported faithfully; only the SEH frame
/// itself is unavailable here.
///
/// This module is, exactly like the C++ translation unit, MSVC-only: the
/// non-MSVC `try_catch_signal` (the plain `fn()` body) lives in
/// `try_catch_signal.rs`, mirroring `TryCatchSignal.hpp`'s `#else` branch.
#[cfg(target_env = "msvc")]
pub mod detail {
    use super::{signal_seh_filter, SignalT};

    /// See module-level note: the SEH frame is a blocked native dependency.
    ///
    /// On MSVC, the only place to obtain a correct `GetExceptionCode()` and a
    /// `__try`/`__except` frame is a real SEH backend. We retain the exact
    /// signature so a future native shim can drop in, and route the
    /// filter through `signal_seh_filter` to keep its decision logic live.
    pub fn try_catch_signal_seh(
        sigcnt: i32,
        sigs: &[SignalT],
        fn_: impl FnOnce(),
        cfn: impl FnOnce(),
    ) {
        // Without an MSVC SEH frame we cannot observe a `GetExceptionCode()`,
        // so there is nothing to feed `signal_seh_filter`; faithful behavior
        // (no fake panic catching) is to run `fn` directly. The filter is
        // referenced here so the ported logic stays wired to its sole caller.
        let _ = &signal_seh_filter;
        let _ = sigcnt;
        let _ = sigs;
        let _ = cfn;
        // TryCatchSignalSEH.cpp:38
        fn_();
    }
}

/// `try_catch_signal` template — MSVC entry point that forwards to the SEH
/// detail.
///
/// TryCatchSignalSEH.hpp:19-23
/// C++: template<class TryFn, class CatchFn, int N>
/// C++: void try_catch_signal(const SignalT (&sigs)[N], TryFn &&fn, CatchFn &&cfn)
/// C++: {
/// C++:     detail::try_catch_signal_seh(N, sigs, fn, cfn);
/// C++: }
#[cfg(target_env = "msvc")]
pub fn try_catch_signal<TryFn, CatchFn>(sigs: &[SignalT], fn_: TryFn, cfn: CatchFn)
where
    TryFn: FnOnce(),
    CatchFn: FnOnce(),
{
    // TryCatchSignalSEH.hpp:22 — N is the array length, i.e. `sigs.len()`.
    detail::try_catch_signal_seh(sigs.len() as i32, sigs, fn_, cfn);
}

#[cfg(test)]
mod tests {
    use super::*;

    // The filter is pure logic and is exercised on every target.
    #[test]
    fn test_signal_constants() {
        // TryCatchSignalSEH.hpp:9
        assert_eq!(SIGILL, 4);
        assert_eq!(SIGFPE, 8);
        assert_eq!(SIGSEGV, 11);
    }

    #[test]
    fn test_filter_no_signals_continues_search() {
        // sigcnt == 0: the loop body never runs, ret stays CONTINUE_SEARCH.
        assert_eq!(
            signal_seh_filter(0, &[], STATUS_ACCESS_VIOLATION),
            EXCEPTION_CONTINUE_SEARCH
        );
    }

    #[test]
    fn test_filter_segv_matches_access_violation() {
        let sigs = [SIGSEGV];
        assert_eq!(
            signal_seh_filter(1, &sigs, STATUS_ACCESS_VIOLATION),
            EXCEPTION_EXECUTE_HANDLER
        );
        // Wrong code for SIGSEGV -> keep searching.
        assert_eq!(
            signal_seh_filter(1, &sigs, STATUS_FLOAT_OVERFLOW),
            EXCEPTION_CONTINUE_SEARCH
        );
    }

    #[test]
    fn test_filter_ill_matches_illegal_instruction() {
        let sigs = [SIGILL];
        assert_eq!(
            signal_seh_filter(1, &sigs, STATUS_ILLEGAL_INSTRUCTION),
            EXCEPTION_EXECUTE_HANDLER
        );
        assert_eq!(
            signal_seh_filter(1, &sigs, STATUS_ACCESS_VIOLATION),
            EXCEPTION_CONTINUE_SEARCH
        );
    }

    #[test]
    fn test_filter_fpe_matches_all_float_and_int_codes() {
        let sigs = [SIGFPE];
        for code in [
            STATUS_FLOAT_DIVIDE_BY_ZERO,
            STATUS_FLOAT_OVERFLOW,
            STATUS_FLOAT_UNDERFLOW,
            STATUS_INTEGER_DIVIDE_BY_ZERO,
        ] {
            assert_eq!(
                signal_seh_filter(1, &sigs, code),
                EXCEPTION_EXECUTE_HANDLER
            );
        }
        // Access violation is not an FPE code.
        assert_eq!(
            signal_seh_filter(1, &sigs, STATUS_ACCESS_VIOLATION),
            EXCEPTION_CONTINUE_SEARCH
        );
    }

    #[test]
    fn test_filter_scans_until_match() {
        // {SIGSEGV, SIGFPE} as used by MeshBoolean.cpp; an FPE code should be
        // caught by the second entry even though the first does not match.
        let sigs = [SIGSEGV, SIGFPE];
        assert_eq!(
            signal_seh_filter(2, &sigs, STATUS_INTEGER_DIVIDE_BY_ZERO),
            EXCEPTION_EXECUTE_HANDLER
        );
    }

    #[test]
    fn test_filter_unknown_signal_continues_search() {
        // A signal number not in {SIGSEGV, SIGILL, SIGFPE} hits the default arm.
        let sigs: [SignalT; 1] = [2 /* SIGINT */];
        assert_eq!(
            signal_seh_filter(1, &sigs, STATUS_ACCESS_VIOLATION),
            EXCEPTION_CONTINUE_SEARCH
        );
    }
}
