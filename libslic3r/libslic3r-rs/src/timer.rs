//! Timer utilities for performance measurement and time-limited operations
//!
//! C++ Reference:
//! - Timer.hpp (lines 1-93)
//! - Timer.cpp (lines 1-19)
//!
//! This module provides utilities for measuring code execution time and
//! creating alarms that trigger when operations exceed time limits.

use std::time::{Duration, Instant};

/// Simple timer that logs elapsed time on drop
///
/// Timer.hpp:11-30
/// C++: class Timer
/// C++: {
/// C++:     std::string m_name;
/// C++:     std::chrono::steady_clock::time_point m_start;
/// C++: public:
/// C++:     Timer(const std::string& name);
/// C++:     ~Timer();
/// C++: };
///
/// Timer.cpp:5-6
/// C++: Slic3r::Timer::Timer(const std::string &name) : m_name(name), m_start(steady_clock::now()) {}
///
/// Timer.cpp:8-12
/// C++: Slic3r::Timer::~Timer()
/// C++: {
/// C++:     BOOST_LOG_TRIVIAL(debug) << "Timer '" << m_name << "' spend " <<
/// C++:         duration_cast<milliseconds>(steady_clock::now() - m_start).count() << "ms";
/// C++: }
#[derive(Debug)]
pub struct Timer {
    /// Name for logging
    /// Timer.hpp:14
    name: String,

    /// Start time
    /// Timer.hpp:15
    start: Instant,
}

impl Timer {
    /// Create a new timer with the given name
    ///
    /// Timer.hpp:22
    /// C++: Timer(const std::string& name);
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            start: Instant::now(),
        }
    }

    /// Get elapsed time since timer was created
    pub fn elapsed(&self) -> Duration {
        self.start.elapsed()
    }

    /// Get elapsed time in milliseconds
    pub fn elapsed_ms(&self) -> u128 {
        self.elapsed().as_millis()
    }

    /// Get elapsed time in seconds
    pub fn elapsed_secs(&self) -> f64 {
        self.elapsed().as_secs_f64()
    }
}

impl Drop for Timer {
    /// Log elapsed time when timer is dropped
    ///
    /// Timer.cpp:8-12
    /// C++: Slic3r::Timer::~Timer()
    /// C++: {
    /// C++:     BOOST_LOG_TRIVIAL(debug) << "Timer '" << m_name << "' spend " <<
    /// C++:         duration_cast<milliseconds>(steady_clock::now() - m_start).count() << "ms";
    /// C++: }
    fn drop(&mut self) {
        log::debug!("Timer '{}' spent {}ms", self.name, self.elapsed_ms());
    }
}

/// High-precision timer for performance measurement
///
/// Timer.hpp:38-59
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
pub struct PrecisionTimer {
    /// Start time
    /// Timer.hpp:58
    start: Option<Instant>,
}

impl Default for PrecisionTimer {
    fn default() -> Self {
        Self::new()
    }
}

impl PrecisionTimer {
    /// Create a new precision timer (not started)
    ///
    /// Timer.hpp:57
    /// C++: uint64_t m_nanoseconds = 0;
    pub fn new() -> Self {
        Self { start: None }
    }

    /// Start or restart the timer
    ///
    /// Timer.hpp:41
    /// C++: void start() { m_nanoseconds = nanoseconds_since_epoch(); }
    pub fn start(&mut self) {
        self.start = Some(Instant::now());
    }

    /// Get elapsed nanoseconds since start
    ///
    /// Timer.hpp:42-44
    /// C++: uint64_t elapsed_nanoseconds() const {
    /// C++:     return nanoseconds_since_epoch() - m_nanoseconds;
    /// C++: }
    pub fn elapsed_nanoseconds(&self) -> u64 {
        self.start
            .map(|s| s.elapsed().as_nanos() as u64)
            .unwrap_or(0)
    }

    /// Get elapsed microseconds since start
    ///
    /// Timer.hpp:45-47
    /// C++: uint64_t elapsed_microseconds() const {
    /// C++:     return elapsed_nanoseconds() / 1000;
    /// C++: }
    pub fn elapsed_microseconds(&self) -> u64 {
        self.elapsed_nanoseconds() / 1000
    }

    /// Get elapsed milliseconds since start
    ///
    /// Timer.hpp:48-50
    /// C++: unsigned int elapsed_milliseconds() const {
    /// C++:     return static_cast<unsigned int>(elapsed_microseconds()/1000);
    /// C++: }
    pub fn elapsed_milliseconds(&self) -> u32 {
        (self.elapsed_microseconds() / 1000) as u32
    }

    /// Get elapsed seconds since start
    ///
    /// Timer.hpp:51-53
    /// C++: double elapsed_seconds() const {
    /// C++:     return elapsed_microseconds() / 1000000.0;
    /// C++: }
    pub fn elapsed_seconds(&self) -> f64 {
        self.elapsed_microseconds() as f64 / 1_000_000.0
    }
}

/// Alarm that logs an error if a time limit is exceeded
///
/// Timer.hpp:62-89
/// C++: class TimeLimitAlarm {
/// C++: public:
/// C++:     TimeLimitAlarm(uint64_t time_limit_nanoseconds, std::string_view limit_exceeded_message);
/// C++:     ~TimeLimitAlarm() {
/// C++:         auto elapsed = m_timer.elapsed_nanoseconds();
/// C++:         if (elapsed > m_time_limit_nanoseconds)
/// C++:             this->report_time_exceeded();
/// C++:     }
/// C++:     static TimeLimitAlarm new_nanos(...);
/// C++:     static TimeLimitAlarm new_milis(...);
/// C++:     static TimeLimitAlarm new_seconds(...);
/// C++: private:
/// C++:     void report_time_exceeded() const;
/// C++:     Timer m_timer;
/// C++:     uint64_t m_time_limit_nanoseconds;
/// C++:     std::string_view m_limit_exceeded_message;
/// C++: };
#[derive(Debug)]
pub struct TimeLimitAlarm {
    /// Internal timer
    /// Timer.hpp:83
    timer: PrecisionTimer,

    /// Time limit in nanoseconds
    /// Timer.hpp:84
    time_limit_nanos: u64,

    /// Message to log if limit exceeded
    /// Timer.hpp:85
    message: String,
}

impl TimeLimitAlarm {
    /// Create a new time limit alarm with nanosecond precision
    ///
    /// Timer.hpp:75-78
    /// C++: static TimeLimitAlarm new_nanos(uint64_t time_limit_nanoseconds, std::string_view limit_exceeded_message) {
    /// C++:     return TimeLimitAlarm(time_limit_nanoseconds, limit_exceeded_message);
    /// C++: }
    pub fn new_nanos(time_limit_nanos: u64, message: impl Into<String>) -> Self {
        let mut timer = PrecisionTimer::new();
        timer.start();
        Self {
            timer,
            time_limit_nanos,
            message: message.into(),
        }
    }

    /// Create a new time limit alarm with millisecond precision
    ///
    /// Timer.hpp:79-81
    /// C++: static TimeLimitAlarm new_milis(uint64_t time_limit_milis, std::string_view limit_exceeded_message) {
    /// C++:     return TimeLimitAlarm(uint64_t(time_limit_milis) * 1000000l, limit_exceeded_message);
    /// C++: }
    pub fn new_millis(time_limit_millis: u64, message: impl Into<String>) -> Self {
        Self::new_nanos(time_limit_millis * 1_000_000, message)
    }

    /// Create a new time limit alarm with second precision
    ///
    /// Timer.hpp:82-84
    /// C++: static TimeLimitAlarm new_seconds(uint64_t time_limit_seconds, std::string_view limit_exceeded_message) {
    /// C++:     return TimeLimitAlarm(uint64_t(time_limit_seconds) * 1000000000l, limit_exceeded_message);
    /// C++: }
    pub fn new_seconds(time_limit_seconds: u64, message: impl Into<String>) -> Self {
        Self::new_nanos(time_limit_seconds * 1_000_000_000, message)
    }

    /// Report that time limit was exceeded
    ///
    /// Timer.cpp:15-17
    /// C++: void TimeLimitAlarm::report_time_exceeded() const {
    /// C++:     BOOST_LOG_TRIVIAL(error) << "Time limit exceeded for " << m_limit_exceeded_message << ": " << m_timer.elapsed_seconds() << "s";
    /// C++: }
    fn report_time_exceeded(&self) {
        log::error!(
            "Time limit exceeded for {}: {:.3}s",
            self.message,
            self.timer.elapsed_seconds()
        );
    }
}

impl Drop for TimeLimitAlarm {
    /// Check time limit when alarm is dropped
    ///
    /// Timer.hpp:68-72
    /// C++: ~TimeLimitAlarm() {
    /// C++:     auto elapsed = m_timer.elapsed_nanoseconds();
    /// C++:     if (elapsed > m_time_limit_nanoseconds)
    /// C++:         this->report_time_exceeded();
    /// C++: }
    fn drop(&mut self) {
        if self.timer.elapsed_nanoseconds() > self.time_limit_nanos {
            self.report_time_exceeded();
        }
    }
}

#[cfg(test)]
mod tests {
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
        assert!(elapsed < 100); // Should be reasonably close
    }

    #[test]
    fn test_precision_timer_start() {
        let mut timer = PrecisionTimer::new();
        assert_eq!(timer.elapsed_nanoseconds(), 0);

        timer.start();
        thread::sleep(Duration::from_millis(10));
        assert!(timer.elapsed_nanoseconds() > 0);
    }

    #[test]
    fn test_precision_timer_elapsed_units() {
        let mut timer = PrecisionTimer::new();
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
    fn test_precision_timer_restart() {
        let mut timer = PrecisionTimer::new();
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
        let _alarm = TimeLimitAlarm::new_millis(1000, "test operation");
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
    fn test_precision_timer_default() {
        let timer = PrecisionTimer::default();
        assert_eq!(timer.elapsed_nanoseconds(), 0);
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
