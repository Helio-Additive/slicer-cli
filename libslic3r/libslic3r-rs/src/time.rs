//! Time utilities for timestamps and time formatting
//!
//! C++ Reference:
//! - Time.hpp (lines 1-64)
//! - Time.cpp (lines 1-263)
//!
//! This module provides utilities for getting current time, formatting timestamps,
//! and parsing time strings in various formats (G-code format and ISO8601).

use crate::{Error, Result};
use std::time::{SystemTime, UNIX_EPOCH};

/// Time zone specification
///
/// Time.hpp:13
/// C++: enum class TimeZone { local, utc };
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TimeZone {
    /// Local time zone
    /// Time.hpp:13
    Local,

    /// UTC time zone
    /// Time.hpp:13
    Utc,
}

/// Time format specification
///
/// Time.hpp:14
/// C++: enum class TimeFormat { gcode, iso8601Z };
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TimeFormat {
    /// G-code time format: "YYYY-MM-DD at HH:MM:SS"
    /// Time.hpp:14
    /// Time.cpp:20
    GCode,

    /// ISO8601Z format: "YYYYMMDDTHHMMSSz"
    /// Time.hpp:14
    /// Time.cpp:23
    Iso8601Z,
}

impl TimeFormat {
    /// Get the strftime format string for this format
    ///
    /// Time.cpp:25-32
    /// C++: static const char * get_fmtstr(TimeFormat fmt)
    /// C++: {
    /// C++:     switch (fmt) {
    /// C++:     case TimeFormat::gcode: return SLICER_UTC_TIME_FMT;
    /// C++:     case TimeFormat::iso8601Z: return ISO8601Z_TIME_FMT;
    /// C++:     }
    /// C++:     return "";
    /// C++: }
    fn format_string(&self) -> &'static str {
        match self {
            TimeFormat::GCode => "%Y-%m-%d at %H:%M:%S",
            TimeFormat::Iso8601Z => "%Y%m%dT%H%M%SZ",
        }
    }
}

/// Get current UTC time as seconds since Unix epoch
///
/// Time.hpp:11
/// C++: time_t get_current_time_utc();
///
/// Time.cpp:172-176
/// C++: time_t get_current_time_utc()
/// C++: {
/// C++:     using clk = std::chrono::system_clock;
/// C++:     return clk::to_time_t(clk::now());
/// C++: }
pub fn get_current_time_utc() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .expect("System time before Unix epoch")
        .as_secs()
}

/// Get current UTC time as milliseconds since Unix epoch
///
/// Time.hpp:12
/// C++: time_t get_current_milliseconds_time_utc();
///
/// Time.cpp:178-184
/// C++: time_t get_current_milliseconds_time_utc()
/// C++: {
/// C++:     using clk = std::chrono::system_clock;
/// C++:     auto now = clk::now();
/// C++:     auto duration = now.time_since_epoch();
/// C++:     auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
/// C++:     return static_cast<time_t>(milliseconds);
/// C++: }
pub fn get_current_milliseconds_time_utc() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .expect("System time before Unix epoch")
        .as_millis() as u64
}

/// Convert time_t to formatted string
///
/// Time.hpp:18
/// C++: std::string time2str(const time_t &t, TimeZone zone, TimeFormat fmt);
///
/// Time.cpp:192-203
/// C++: std::string time2str(const time_t &t, TimeZone zone, TimeFormat fmt)
/// C++: {
/// C++:     std::string ret;
/// C++:     std::tm tms = {};
/// C++:     tms.tm_isdst = -1;
/// C++:     std::string fmtstr = process_format(get_fmtstr(fmt), zone);
/// C++:     switch (zone) {
/// C++:     case TimeZone::local:
/// C++:         ret = tm2str(_localtime_r(&t, &tms), fmtstr.c_str()); break;
/// C++:     case TimeZone::utc:
/// C++:         ret = tm2str(_gmtime_r(&t, &tms), fmtstr.c_str()); break;
/// C++:     }
/// C++:     return ret;
/// C++: }
pub fn time2str(t: u64, zone: TimeZone, fmt: TimeFormat) -> String {
    use chrono::{Local, TimeZone as ChronoTimeZone, Utc};

    let timestamp = t as i64;
    let mut format_str = fmt.format_string().to_string();

    // Add UTC suffix for G-code format with UTC timezone
    // Time.cpp:162-168
    if matches!(fmt, TimeFormat::GCode) && matches!(zone, TimeZone::Utc) {
        format_str.push_str(" UTC");
    }

    match zone {
        TimeZone::Local => {
            let dt = Local.timestamp_opt(timestamp, 0).unwrap();
            dt.format(&format_str).to_string()
        }
        TimeZone::Utc => {
            let dt = Utc.timestamp_opt(timestamp, 0).unwrap();
            dt.format(&format_str).to_string()
        }
    }
}

/// Convert current time to formatted string
///
/// Time.hpp:20-23
/// C++: inline std::string time2str(TimeZone zone, TimeFormat fmt)
/// C++: {
/// C++:     return time2str(get_current_time_utc(), zone, fmt);
/// C++: }
pub fn time2str_now(zone: TimeZone, fmt: TimeFormat) -> String {
    time2str(get_current_time_utc(), zone, fmt)
}

/// Convert time_t to UTC timestamp in G-code format
///
/// Time.hpp:25-28
/// C++: inline std::string utc_timestamp(time_t t)
/// C++: {
/// C++:     return time2str(t, TimeZone::utc, TimeFormat::gcode);
/// C++: }
pub fn utc_timestamp(t: u64) -> String {
    time2str(t, TimeZone::Utc, TimeFormat::GCode)
}

/// Get current UTC timestamp in G-code format
///
/// Time.hpp:30-33
/// C++: inline std::string utc_timestamp()
/// C++: {
/// C++:     return utc_timestamp(get_current_time_utc());
/// C++: }
pub fn utc_timestamp_now() -> String {
    utc_timestamp(get_current_time_utc())
}

/// Parse time string to time_t
///
/// Time.hpp:36
/// C++: time_t str2time(const std::string &str, TimeZone zone, TimeFormat fmt);
///
/// Time.cpp:220-228
/// C++: time_t str2time(const std::string &str, TimeZone zone, TimeFormat fmt)
/// C++: {
/// C++:     std::string fmtstr = process_format(get_fmtstr(fmt), zone).c_str();
/// C++:     std::stringstream ss(str);
/// C++:     ss.imbue(std::locale("C"));
/// C++:     return str2time(ss, zone, fmtstr.c_str());
/// C++: }
pub fn str2time(s: &str, zone: TimeZone, fmt: TimeFormat) -> Result<u64> {
    use chrono::{Local, NaiveDateTime, TimeZone as ChronoTimeZone, Utc};

    let mut input = s.trim();
    let mut format_str = fmt.format_string();

    // Handle UTC suffix for G-code format
    if matches!(fmt, TimeFormat::GCode) && input.ends_with(" UTC") {
        input = input.trim_end_matches(" UTC").trim();
        format_str = "%Y-%m-%d at %H:%M:%S";
    }

    // Parse the datetime
    let naive_dt = NaiveDateTime::parse_from_str(input, format_str)
        .map_err(|e| Error::ParseError(format!("Failed to parse time string '{}': {}", s, e)))?;

    // Convert to timestamp based on timezone
    let timestamp = match zone {
        TimeZone::Local => {
            let dt = Local.from_local_datetime(&naive_dt).unwrap();
            dt.timestamp()
        }
        TimeZone::Utc => {
            let dt = Utc.from_utc_datetime(&naive_dt);
            dt.timestamp()
        }
    };

    if timestamp < 0 {
        return Err(Error::ParseError(format!(
            "Invalid timestamp (negative): {}",
            timestamp
        )));
    }

    Ok(timestamp as u64)
}

/// Convert time_t to ISO8601 UTC timestamp
///
/// Time.hpp:47-50
/// C++: inline std::string iso_utc_timestamp(time_t t)
/// C++: {
/// C++:     return time2str(t, TimeZone::utc, TimeFormat::iso8601Z);
/// C++: }
pub fn iso_utc_timestamp(t: u64) -> String {
    time2str(t, TimeZone::Utc, TimeFormat::Iso8601Z)
}

/// Get current time as ISO8601 UTC timestamp
///
/// Time.hpp:52-55
/// C++: inline std::string iso_utc_timestamp()
/// C++: {
/// C++:     return iso_utc_timestamp(get_current_time_utc());
/// C++: }
pub fn iso_utc_timestamp_now() -> String {
    iso_utc_timestamp(get_current_time_utc())
}

/// Parse ISO8601 UTC timestamp string to time_t
///
/// Time.hpp:57-60
/// C++: inline time_t parse_iso_utc_timestamp(const std::string &str)
/// C++: {
/// C++:     return str2time(str, TimeZone::utc, TimeFormat::iso8601Z);
/// C++: }
pub fn parse_iso_utc_timestamp(s: &str) -> Result<u64> {
    str2time(s, TimeZone::Utc, TimeFormat::Iso8601Z)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_get_current_time() {
        let t = get_current_time_utc();
        // Should be a reasonable timestamp (after 2020)
        assert!(t > 1577836800); // Jan 1, 2020
    }

    #[test]
    fn test_get_current_milliseconds() {
        let ms = get_current_milliseconds_time_utc();
        let s = get_current_time_utc();
        // Milliseconds should be roughly 1000x seconds
        assert!(ms / 1000 >= s);
        assert!(ms / 1000 <= s + 1);
    }

    #[test]
    fn test_time2str_gcode_utc() {
        let t = 1609459200; // 2021-01-01 00:00:00 UTC
        let s = time2str(t, TimeZone::Utc, TimeFormat::GCode);
        assert_eq!(s, "2021-01-01 at 00:00:00 UTC");
    }

    #[test]
    fn test_time2str_iso8601z() {
        let t = 1609459200; // 2021-01-01 00:00:00 UTC
        let s = time2str(t, TimeZone::Utc, TimeFormat::Iso8601Z);
        assert_eq!(s, "20210101T000000Z");
    }

    #[test]
    fn test_utc_timestamp() {
        let t = 1609459200;
        let s = utc_timestamp(t);
        assert_eq!(s, "2021-01-01 at 00:00:00 UTC");
    }

    #[test]
    fn test_iso_utc_timestamp() {
        let t = 1609459200;
        let s = iso_utc_timestamp(t);
        assert_eq!(s, "20210101T000000Z");
    }

    #[test]
    fn test_str2time_gcode_utc() {
        let s = "2021-01-01 at 00:00:00 UTC";
        let t = str2time(s, TimeZone::Utc, TimeFormat::GCode).unwrap();
        assert_eq!(t, 1609459200);
    }

    #[test]
    fn test_str2time_gcode_no_utc() {
        let s = "2021-01-01 at 00:00:00";
        let t = str2time(s, TimeZone::Utc, TimeFormat::GCode).unwrap();
        assert_eq!(t, 1609459200);
    }

    #[test]
    fn test_str2time_iso8601z() {
        let s = "20210101T000000Z";
        let t = str2time(s, TimeZone::Utc, TimeFormat::Iso8601Z).unwrap();
        assert_eq!(t, 1609459200);
    }

    #[test]
    fn test_parse_iso_utc_timestamp() {
        let s = "20210101T000000Z";
        let t = parse_iso_utc_timestamp(s).unwrap();
        assert_eq!(t, 1609459200);
    }

    #[test]
    fn test_round_trip_gcode() {
        let t = 1609459200;
        let s = utc_timestamp(t);
        let t2 = str2time(&s, TimeZone::Utc, TimeFormat::GCode).unwrap();
        assert_eq!(t, t2);
    }

    #[test]
    fn test_round_trip_iso8601() {
        let t = 1609459200;
        let s = iso_utc_timestamp(t);
        let t2 = parse_iso_utc_timestamp(&s).unwrap();
        assert_eq!(t, t2);
    }

    #[test]
    fn test_format_string() {
        assert_eq!(TimeFormat::GCode.format_string(), "%Y-%m-%d at %H:%M:%S");
        assert_eq!(TimeFormat::Iso8601Z.format_string(), "%Y%m%dT%H%M%SZ");
    }
}
