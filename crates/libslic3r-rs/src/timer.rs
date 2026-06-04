//! 1:1 port of `Timer.hpp` / `Timer.cpp` from BambuStudio libslic3r.
//!
//! C++ Reference:
//! - Timer.hpp (lines 1-93)
//! - Timer.cpp (lines 1-22)
//!
//! Faithful, line-by-line translation. `coord_t`/`coordf_t` do not appear here.
//!
//! Time sources:
//! - C++ `steady_clock` is a monotonic clock -> Rust `std::time::Instant`.
//! - C++ `high_resolution_clock::now().time_since_epoch()` is an absolute clock
//!   -> Rust `std::time::SystemTime` measured against `UNIX_EPOCH` (the same
//!   pattern used elsewhere in this crate, e.g. `time.rs`/`fuzzy_skin.rs`, and
//!   wasm-safe). This mirrors the C++ behaviour of returning an absolute
//!   epoch-relative nanosecond count that `Timing::Timer` subtracts.

// Timer.cpp:1
// C++: #include "Timer.hpp"
// Timer.cpp:2
// C++: #include <boost/log/trivial.hpp>
// Timer.cpp:4
// C++: using namespace std::chrono;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

/// Instance of this class is used for measure time consumtion
/// of block code until instance is alive and write result to debug output
///
/// Timer.hpp:13-28
/// C++: class Timer
/// C++: {
/// C++:     std::string m_name;
/// C++:     std::chrono::steady_clock::time_point m_start;
/// C++: public:
/// C++:     Timer(const std::string& name);
/// C++:     ~Timer();
/// C++: };
#[derive(Debug)]
pub struct Timer {
    /// Timer.hpp:15
    /// C++: std::string m_name;
    m_name: String,
    /// Timer.hpp:16
    /// C++: std::chrono::steady_clock::time_point m_start;
    m_start: Instant,
}

impl Timer {
    /// name describe timer
    ///
    /// Timer.hpp:22 / Timer.cpp:6
    /// C++: Slic3r::Timer::Timer(const std::string &name) : m_name(name), m_start(steady_clock::now()) {}
    pub fn new(name: &str) -> Self {
        // Timer.cpp:6
        Self {
            m_name: name.to_string(),
            m_start: Instant::now(),
        }
    }

    /// Convenience accessor for the elapsed duration since construction.
    pub fn elapsed(&self) -> Duration {
        self.m_start.elapsed()
    }

    /// Convenience accessor for the elapsed milliseconds since construction.
    ///
    /// Mirrors `duration_cast<milliseconds>(steady_clock::now() - m_start).count()`.
    pub fn elapsed_ms(&self) -> u128 {
        self.elapsed().as_millis()
    }
}

impl Drop for Timer {
    /// Timer.cpp:8-12
    /// C++: Slic3r::Timer::~Timer()
    /// C++: {
    /// C++:     BOOST_LOG_TRIVIAL(debug) << "Timer '" << m_name << "' spend " <<
    /// C++:         duration_cast<milliseconds>(steady_clock::now() - m_start).count() << "ms";
    /// C++: }
    fn drop(&mut self) {
        // Timer.cpp:10-11
        log::debug!(
            "Timer '{}' spend {}ms",
            self.m_name,
            self.m_start.elapsed().as_millis()
        );
    }
}

// Timer.cpp:15 / Timer.hpp:30
// C++: namespace Slic3r::Timing {
pub mod timing {
    use super::*;

    /// Timing code from Catch2 unit testing library
    ///
    /// Timer.hpp:33-35
    /// C++: static inline uint64_t nanoseconds_since_epoch() {
    /// C++:     return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    /// C++: }
    #[inline]
    pub fn nanoseconds_since_epoch() -> u64 {
        // Timer.hpp:34
        SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map(|d| d.as_nanos() as u64)
            .unwrap_or(0)
    }

    /// Timing code from Catch2 unit testing library
    ///
    /// Timer.hpp:38-57
    /// C++: class Timer {
    /// C++: public:
    /// C++:     void start() { m_nanoseconds = nanoseconds_since_epoch(); }
    /// C++:     uint64_t elapsed_nanoseconds() const { return nanoseconds_since_epoch() - m_nanoseconds; }
    /// C++:     uint64_t elapsed_microseconds() const { return elapsed_nanoseconds() / 1000; }
    /// C++:     unsigned int elapsed_milliseconds() const { return static_cast<unsigned int>(elapsed_microseconds()/1000); }
    /// C++:     double elapsed_seconds() const { return elapsed_microseconds() / 1000000.0; }
    /// C++: private:
    /// C++:     uint64_t m_nanoseconds = 0;
    /// C++: };
    #[derive(Debug, Clone)]
    pub struct Timer {
        /// Timer.hpp:56
        /// C++: uint64_t m_nanoseconds = 0;
        m_nanoseconds: u64,
    }

    impl Default for Timer {
        /// Timer.hpp:56
        /// C++: uint64_t m_nanoseconds = 0;
        fn default() -> Self {
            Self { m_nanoseconds: 0 }
        }
    }

    impl Timer {
        /// Timer.hpp:56
        /// C++: uint64_t m_nanoseconds = 0;
        pub fn new() -> Self {
            Self::default()
        }

        /// Timer.hpp:40-42
        /// C++: void start() {
        /// C++:     m_nanoseconds = nanoseconds_since_epoch();
        /// C++: }
        pub fn start(&mut self) {
            // Timer.hpp:41
            self.m_nanoseconds = nanoseconds_since_epoch();
        }

        /// Timer.hpp:43-45
        /// C++: uint64_t elapsed_nanoseconds() const {
        /// C++:     return nanoseconds_since_epoch() - m_nanoseconds;
        /// C++: }
        pub fn elapsed_nanoseconds(&self) -> u64 {
            // Timer.hpp:44
            nanoseconds_since_epoch().wrapping_sub(self.m_nanoseconds)
        }

        /// Timer.hpp:46-48
        /// C++: uint64_t elapsed_microseconds() const {
        /// C++:     return elapsed_nanoseconds() / 1000;
        /// C++: }
        pub fn elapsed_microseconds(&self) -> u64 {
            // Timer.hpp:47
            self.elapsed_nanoseconds() / 1000
        }

        /// Timer.hpp:49-51
        /// C++: unsigned int elapsed_milliseconds() const {
        /// C++:     return static_cast<unsigned int>(elapsed_microseconds()/1000);
        /// C++: }
        pub fn elapsed_milliseconds(&self) -> u32 {
            // Timer.hpp:50
            (self.elapsed_microseconds() / 1000) as u32
        }

        /// Timer.hpp:52-54
        /// C++: double elapsed_seconds() const {
        /// C++:     return elapsed_microseconds() / 1000000.0;
        /// C++: }
        pub fn elapsed_seconds(&self) -> f64 {
            // Timer.hpp:53
            self.elapsed_microseconds() as f64 / 1000000.0
        }
    }

    /// Emits a Boost::log error if the life time of this timing object exceeds a limit.
    ///
    /// Timer.hpp:60-86
    /// C++: class TimeLimitAlarm {
    /// C++: public:
    /// C++:     TimeLimitAlarm(uint64_t time_limit_nanoseconds, std::string_view limit_exceeded_message);
    /// C++:     ~TimeLimitAlarm();
    /// C++:     static TimeLimitAlarm new_nanos(...);
    /// C++:     static TimeLimitAlarm new_milis(...);
    /// C++:     static TimeLimitAlarm new_seconds(...);
    /// C++: private:
    /// C++:     void report_time_exceeded() const;
    /// C++:     Timer               m_timer;
    /// C++:     uint64_t            m_time_limit_nanoseconds;
    /// C++:     std::string_view    m_limit_exceeded_message;
    /// C++: };
    #[derive(Debug)]
    pub struct TimeLimitAlarm {
        /// Timer.hpp:83
        /// C++: Timer m_timer;
        m_timer: Timer,
        /// Timer.hpp:84
        /// C++: uint64_t m_time_limit_nanoseconds;
        m_time_limit_nanoseconds: u64,
        /// Timer.hpp:85
        /// C++: std::string_view m_limit_exceeded_message;
        m_limit_exceeded_message: String,
    }

    impl TimeLimitAlarm {
        /// Timer.hpp:62-65
        /// C++: TimeLimitAlarm(uint64_t time_limit_nanoseconds, std::string_view limit_exceeded_message) :
        /// C++:     m_time_limit_nanoseconds(time_limit_nanoseconds), m_limit_exceeded_message(limit_exceeded_message) {
        /// C++:     m_timer.start();
        /// C++: }
        pub fn new(time_limit_nanoseconds: u64, limit_exceeded_message: &str) -> Self {
            // Timer.hpp:63
            let mut alarm = Self {
                m_timer: Timer::new(),
                m_time_limit_nanoseconds: time_limit_nanoseconds,
                m_limit_exceeded_message: limit_exceeded_message.to_string(),
            };
            // Timer.hpp:64
            alarm.m_timer.start();
            alarm
        }

        /// Timer.hpp:71-73
        /// C++: static TimeLimitAlarm new_nanos(uint64_t time_limit_nanoseconds, std::string_view limit_exceeded_message) {
        /// C++:     return TimeLimitAlarm(time_limit_nanoseconds, limit_exceeded_message);
        /// C++: }
        pub fn new_nanos(time_limit_nanoseconds: u64, limit_exceeded_message: &str) -> Self {
            // Timer.hpp:72
            TimeLimitAlarm::new(time_limit_nanoseconds, limit_exceeded_message)
        }

        /// Timer.hpp:74-76
        /// C++: static TimeLimitAlarm new_milis(uint64_t time_limit_milis, std::string_view limit_exceeded_message) {
        /// C++:     return TimeLimitAlarm(uint64_t(time_limit_milis) * 1000000l, limit_exceeded_message);
        /// C++: }
        pub fn new_milis(time_limit_milis: u64, limit_exceeded_message: &str) -> Self {
            // Timer.hpp:75
            TimeLimitAlarm::new(time_limit_milis * 1000000, limit_exceeded_message)
        }

        /// Timer.hpp:77-79
        /// C++: static TimeLimitAlarm new_seconds(uint64_t time_limit_seconds, std::string_view limit_exceeded_message) {
        /// C++:     return TimeLimitAlarm(uint64_t(time_limit_seconds) * 1000000000l, limit_exceeded_message);
        /// C++: }
        pub fn new_seconds(time_limit_seconds: u64, limit_exceeded_message: &str) -> Self {
            // Timer.hpp:78
            TimeLimitAlarm::new(time_limit_seconds * 1000000000, limit_exceeded_message)
        }

        /// Timer.cpp:17-19
        /// C++: void TimeLimitAlarm::report_time_exceeded() const {
        /// C++:     BOOST_LOG_TRIVIAL(error) << "Time limit exceeded for " << m_limit_exceeded_message << ": " << m_timer.elapsed_seconds() << "s";
        /// C++: }
        fn report_time_exceeded(&self) {
            // Timer.cpp:18
            log::error!(
                "Time limit exceeded for {}: {}s",
                self.m_limit_exceeded_message,
                self.m_timer.elapsed_seconds()
            );
        }
    }

    impl Drop for TimeLimitAlarm {
        /// Timer.hpp:66-70
        /// C++: ~TimeLimitAlarm() {
        /// C++:     auto elapsed = m_timer.elapsed_nanoseconds();
        /// C++:     if (elapsed > m_time_limit_nanoseconds)
        /// C++:         this->report_time_exceeded();
        /// C++: }
        fn drop(&mut self) {
            // Timer.hpp:67
            let elapsed = self.m_timer.elapsed_nanoseconds();
            // Timer.hpp:68-69
            if elapsed > self.m_time_limit_nanoseconds {
                self.report_time_exceeded();
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::timing::{nanoseconds_since_epoch, TimeLimitAlarm, Timer as TimingTimer};
    use super::*;
    use std::thread;
    use std::time::Duration;

    #[test]
    fn test_timer_basic() {
        let timer = Timer::new("test");
        thread::sleep(Duration::from_millis(10));
        assert!(timer.elapsed_ms() >= 10);
    }

    #[test]
    fn test_timer_elapsed() {
        let timer = Timer::new("test");
        thread::sleep(Duration::from_millis(50));
        let elapsed = timer.elapsed_ms();
        assert!(elapsed >= 50);
        assert!(elapsed < 200); // Should be reasonably close
    }

    #[test]
    fn test_nanoseconds_since_epoch_monotonic_ish() {
        let a = nanoseconds_since_epoch();
        thread::sleep(Duration::from_millis(5));
        let b = nanoseconds_since_epoch();
        assert!(b > a);
    }

    #[test]
    fn test_timing_timer_start() {
        let mut timer = TimingTimer::new();
        // Before start, m_nanoseconds == 0, so elapsed == now since epoch (huge).
        timer.start();
        thread::sleep(Duration::from_millis(10));
        assert!(timer.elapsed_nanoseconds() > 0);
    }

    #[test]
    fn test_timing_timer_elapsed_units() {
        let mut timer = TimingTimer::new();
        timer.start();
        thread::sleep(Duration::from_millis(100));

        let nanos = timer.elapsed_nanoseconds();
        let micros = timer.elapsed_microseconds();
        let millis = timer.elapsed_milliseconds();
        let seconds = timer.elapsed_seconds();

        assert!(nanos >= 100_000_000); // At least 100ms in nanos
        assert!(micros >= 100_000); // At least 100ms in micros
        assert!(millis >= 100); // At least 100ms
        assert!(seconds >= 0.1); // At least 0.1s

        // Check conversions are approximately correct
        assert!((micros as f64 / 1000.0 - millis as f64).abs() < 10.0);
        assert!((micros as f64 / 1_000_000.0 - seconds).abs() < 0.01);
    }

    #[test]
    fn test_timing_timer_restart() {
        let mut timer = TimingTimer::new();
        timer.start();
        thread::sleep(Duration::from_millis(50));
        let elapsed1 = timer.elapsed_milliseconds();

        timer.start(); // Restart
        thread::sleep(Duration::from_millis(50));
        let elapsed2 = timer.elapsed_milliseconds();

        // After restart, elapsed should be less than before
        assert!(elapsed2 < elapsed1 + 20);
    }

    #[test]
    fn test_time_limit_alarm_not_exceeded() {
        let _alarm = TimeLimitAlarm::new_milis(1000, "test operation");
        thread::sleep(Duration::from_millis(10));
        // Should not log error
    }

    #[test]
    fn test_time_limit_alarm_new_seconds() {
        let _alarm = TimeLimitAlarm::new_seconds(1, "test operation");
        thread::sleep(Duration::from_millis(10));
        // Should not exceed 1 second
    }

    #[test]
    fn test_time_limit_alarm_new_nanos() {
        let _alarm = TimeLimitAlarm::new_nanos(1_000_000_000, "test operation");
        thread::sleep(Duration::from_millis(10));
        // Should not exceed 1 second (1B nanos)
    }

    #[test]
    fn test_timing_timer_default() {
        let timer = TimingTimer::default();
        assert_eq!(timer.elapsed_microseconds() > 0, true); // now - 0 is huge
    }

    #[test]
    fn test_timer_drop_logs() {
        // This test just ensures drop doesn't panic
        {
            let _timer = Timer::new("drop_test");
            thread::sleep(Duration::from_millis(5));
        } // Timer dropped here
    }
}
