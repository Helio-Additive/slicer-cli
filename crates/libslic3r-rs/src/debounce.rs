//! Port of `libslic3r/Debounce.hpp` of Bambu Studio at the pin.
//!
//! FIDELITY-NOTE: the time point. A `std::chrono::steady_clock::time_point` is a time since the
//! epoch of the clock and `time_point{}` is that epoch; `std::time::Instant` has no epoch one can
//! name. The time point here is an `Option<Instant>`, `None` standing for `time_point{}`. For
//! `None` the function accepts, which is what the header says of the very first call. The C++
//! compares there too (`now - time_point{} >= interval`), so with libc++, whose steady clock
//! counts from the start of the machine, it refuses when the machine has been up for less than
//! `interval`. A `steady_clock::duration` can be negative and a `Duration` cannot; a negative
//! interval always accepts, as a zero one does.

use std::time::{Duration, Instant};

/// Debounce.hpp:8-31
///
/// Rate-limit a recurring action.
///
/// Returns true and updates `last_tp` when at least `interval` has elapsed
/// since the last accepted call.  On the very first call (last_tp ==
/// time_point{}) the action is always accepted.
///
/// Usage:
///   static auto s_last = std::chrono::steady_clock::time_point{};
///   if (debounce_elapsed(s_last, std::chrono::seconds(5)))
///       do_expensive_thing();
///
/// The function is intentionally free of wx / GUI dependencies so it can be
/// exercised by the plain libslic3r unit-test suite.
pub fn debounce_elapsed(last_tp: &mut Option<Instant>, interval: Duration) -> bool {
    let now = Instant::now();
    // now - last_tp >= interval
    let elapsed = match *last_tp {
        None => true,
        Some(last_tp) => now.saturating_duration_since(last_tp) >= interval,
    };
    if elapsed {
        *last_tp = Some(now);
        return true;
    }
    false
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Debounce.hpp:10-12: the very first call is accepted and sets the time point.
    #[test]
    fn first_call_is_accepted() {
        let mut last: Option<Instant> = None;
        assert!(debounce_elapsed(&mut last, Duration::from_secs(3600)));
        assert!(last.is_some());
    }

    /// Debounce.hpp:25-30: a call within the interval is refused and leaves the time point; one
    /// after it is accepted and moves it.
    #[test]
    fn refused_within_the_interval_accepted_after_it() {
        let mut last: Option<Instant> = None;
        assert!(debounce_elapsed(&mut last, Duration::from_millis(40)));
        let first = last;
        assert!(!debounce_elapsed(&mut last, Duration::from_secs(3600)));
        assert_eq!(last, first);
        std::thread::sleep(Duration::from_millis(60));
        assert!(!debounce_elapsed(&mut last, Duration::from_secs(3600)));
        assert_eq!(last, first);
        assert!(debounce_elapsed(&mut last, Duration::from_millis(40)));
        assert!(last > first);
        // now - last_tp >= 0
        assert!(debounce_elapsed(&mut last, Duration::ZERO));
    }
}
