//! Port of `libslic3r/PerfTrace.hpp` and `PerfTrace.cpp` of Bambu Studio at the pin.
//!
//! Lightweight performance tracing for startup / UI flows.

use std::time::Instant;

use crate::boost_log_trivial;
use crate::log::flush_logs;

/// PerfTrace.cpp:6-9
///
/// Log a one-shot marker for the current moment:
///   [perf] <label>
/// For timing a scope, use the PERF_TRACE macro instead.
pub fn perf_mark(label: &str) {
    boost_log_trivial!(Info, "[perf] ", label);
}

/// PerfTrace.hpp:15-32
///
/// RAII scope timer. Logs the elapsed time when the scope exits:
///   [perf] <name> cost <N> ms
/// Pass enabled == false to make the timer a no-op, so a call path that also runs
/// on a hot/periodic flow (e.g. the 10 s auto-backup save) does not spam the log.
/// Prefer the PERF_TRACE / PERF_TRACE_IF macros over constructing this directly.
///
/// The type is neither `Clone` nor `Copy`, as the C++ class has its copy constructor and
/// assignment deleted.
#[derive(Debug)]
pub struct PerfTimer {
    m_name: String,
    m_start: Instant,
    m_enabled: bool,
}

impl PerfTimer {
    /// PerfTrace.cpp:11-15 `PerfTimer::PerfTimer(const std::string &name, bool enabled = true)`
    pub fn new(name: &str, enabled: bool) -> PerfTimer {
        let timer = PerfTimer { m_name: name.to_string(), m_start: Instant::now(), m_enabled: enabled };
        if timer.m_enabled {
            boost_log_trivial!(Info, "[perf] ", timer.m_name, " begin");
        }
        timer
    }
}

impl Drop for PerfTimer {
    /// PerfTrace.cpp:17-24 `PerfTimer::~PerfTimer()`
    fn drop(&mut self) {
        if !self.m_enabled {
            return;
        }
        // duration_cast<milliseconds>(...).count(): a long long
        let ms = (Instant::now() - self.m_start).as_millis() as i64;
        boost_log_trivial!(Info, "[perf] ", self.m_name, " cost ", ms, " ms");
        flush_logs();
    }
}

/// PerfTrace.hpp:34-37 `PERF_TRACE(name)`
///
/// Convenience: time the current scope, logging "[perf] <name> cost <N> ms"
#[macro_export]
macro_rules! perf_trace {
    ($name:expr) => {
        let _perf_timer = $crate::perf_trace::PerfTimer::new(&$name, true);
    };
}

/// PerfTrace.hpp:39-41 `PERF_TRACE_IF(cond, name)`
///
/// Same as PERF_TRACE, but only logs when cond is true. Use it in code shared by an
/// interactive flow and a periodic one, to keep the trace to the interactive case.
#[macro_export]
macro_rules! perf_trace_if {
    ($cond:expr, $name:expr) => {
        let _perf_timer = $crate::perf_trace::PerfTimer::new(&$name, $cond);
    };
}
