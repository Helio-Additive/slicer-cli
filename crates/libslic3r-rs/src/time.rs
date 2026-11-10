//! Time utilities for timestamps and time formatting
//!
//! 1:1 port of BambuStudio `src/libslic3r/Time.{hpp,cpp}`.
//!
//! C++ Reference:
//! - Time.hpp (lines 1-68)
//! - Time.cpp (lines 1-244)
//!
//! Notes on type mapping:
//! - C++ `time_t` is a signed integer (64-bit on the target platforms). We map
//!   it to `i64` so that the `time_t(-1)` failure sentinel and the
//!   `ret < time_t(0)` negative-time checks translate faithfully.
//! - The C++ strftime/strptime emulation (`__get_put_time_emulation`) plus the
//!   `_gmtime_r`/`_localtime_r`/`_mktime`/`_timegm` platform wrappers are
//!   implemented here using the `chrono` crate, which is wasm-safe and provides
//!   the same broken-down-time conversions as `<ctime>`.

use crate::{Error, Result};
use std::time::{SystemTime, UNIX_EPOCH};

// "YYYY-MM-DD at HH:MM::SS [UTC]"
// If TimeZone::utc is used with the conversion functions, it will append the
// UTC letters to the end.
// Time.cpp:22
// C++: static const constexpr char *const SLICER_UTC_TIME_FMT = "%Y-%m-%d at %T";
// Note: strftime's "%T" is equivalent to "%H:%M:%S"; chrono does not support
// "%T", so we expand it to its canonical form for byte-exact output.
const SLICER_UTC_TIME_FMT: &str = "%Y-%m-%d at %H:%M:%S";

// ISO8601Z representation of time, without time zone info
// Time.cpp:25
// C++: static const constexpr char *const ISO8601Z_TIME_FMT = "%Y%m%dT%H%M%SZ";
const ISO8601Z_TIME_FMT: &str = "%Y%m%dT%H%M%SZ";

/// Time zone specification
///
/// Time.hpp:14
/// C++: enum class TimeZone { local, utc };
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TimeZone {
    /// Local time zone
    /// Time.hpp:14
    Local,

    /// UTC time zone
    /// Time.hpp:14
    Utc,
}

/// Time format specification
///
/// Time.hpp:15
/// C++: enum class TimeFormat { gcode, iso8601Z };
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TimeFormat {
    /// G-code time format: "YYYY-MM-DD at HH:MM:SS"
    /// Time.hpp:15
    /// Time.cpp:22
    GCode,

    /// ISO8601Z format: "YYYYMMDDTHHMMSSZ"
    /// Time.hpp:15
    /// Time.cpp:25
    Iso8601Z,
}

/// Time.cpp:27-35
/// C++: static const char * get_fmtstr(TimeFormat fmt)
/// C++: {
/// C++:     switch (fmt) {
/// C++:     case TimeFormat::gcode: return SLICER_UTC_TIME_FMT;
/// C++:     case TimeFormat::iso8601Z: return ISO8601Z_TIME_FMT;
/// C++:     }
/// C++:     return "";
/// C++: }
fn get_fmtstr(fmt: TimeFormat) -> &'static str {
    match fmt {
        TimeFormat::GCode => SLICER_UTC_TIME_FMT,
        TimeFormat::Iso8601Z => ISO8601Z_TIME_FMT,
    }
}

/// Time.cpp:163-171
/// C++: std::string process_format(const char *fmt, TimeZone zone)
/// C++: {
/// C++:     std::string fmtstr(fmt);
/// C++:     if (fmtstr == SLICER_UTC_TIME_FMT && zone == TimeZone::utc)
/// C++:         fmtstr += " UTC";
/// C++:     return fmtstr;
/// C++: }
fn process_format(fmt: &str, zone: TimeZone) -> String {
    let mut fmtstr = fmt.to_string();

    if fmtstr == SLICER_UTC_TIME_FMT && zone == TimeZone::Utc {
        fmtstr.push_str(" UTC");
    }

    fmtstr
}

/// Get current UTC time as seconds since Unix epoch.
///
/// Time.hpp:11
/// C++: time_t get_current_time_utc();
///
/// Time.cpp:175-179
/// C++: time_t get_current_time_utc()
/// C++: {
/// C++:     using clk = std::chrono::system_clock;
/// C++:     return clk::to_time_t(clk::now());
/// C++: }
pub fn get_current_time_utc() -> i64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .expect("System time before Unix epoch")
        .as_secs() as i64
}

/// Get current UTC time as milliseconds since Unix epoch.
///
/// Time.hpp:12
/// C++: time_t get_current_milliseconds_time_utc();
///
/// Time.cpp:181-188
/// C++: time_t get_current_milliseconds_time_utc()
/// C++: {
/// C++:     using clk = std::chrono::system_clock;
/// C++:     auto now = clk::now();
/// C++:     auto duration = now.time_since_epoch();
/// C++:     auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
/// C++:     return static_cast<time_t>(milliseconds);
/// C++: }
pub fn get_current_milliseconds_time_utc() -> i64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .expect("System time before Unix epoch")
        .as_millis() as i64
}

/// Format a broken-down time (`chrono::DateTime`) with the given strftime-style
/// format string, using the "C" locale (i.e. chrono's default, locale-agnostic
/// formatting).
///
/// Time.cpp:190-196
/// C++: static std::string tm2str(const std::tm *tms, const char *fmt)
/// C++: {
/// C++:     std::stringstream ss;
/// C++:     ss.imbue(std::locale("C"));
/// C++:     ss << __get_put_time_emulation::put_time(tms, fmt);
/// C++:     return ss.str();
/// C++: }
fn tm2str<Tz: chrono::TimeZone>(tms: &chrono::DateTime<Tz>, fmt: &str) -> String
where
    Tz::Offset: std::fmt::Display,
{
    tms.format(fmt).to_string()
}

/// Convert `time_t` to a formatted string.
///
/// Time.hpp:19
/// C++: std::string time2str(const time_t &t, TimeZone zone, TimeFormat fmt);
///
/// Time.cpp:198-213
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
pub fn time2str(t: i64, zone: TimeZone, fmt: TimeFormat) -> String {
    use chrono::{Local, TimeZone as ChronoTimeZone, Utc};

    // Time.cpp:203
    let fmtstr = process_format(get_fmtstr(fmt), zone);

    // Time.cpp:205-210
    match zone {
        // _localtime_r: time_t -> broken-down local time
        TimeZone::Local => {
            let tms = Local.timestamp_opt(t, 0).unwrap();
            tm2str(&tms, &fmtstr)
        }
        // _gmtime_r: time_t -> broken-down UTC time
        TimeZone::Utc => {
            let tms = Utc.timestamp_opt(t, 0).unwrap();
            tm2str(&tms, &fmtstr)
        }
    }
}

/// Convert the current time to a formatted string.
///
/// Time.hpp:21-24
/// C++: inline std::string time2str(TimeZone zone, TimeFormat fmt)
/// C++: {
/// C++:     return time2str(get_current_time_utc(), zone, fmt);
/// C++: }
pub fn time2str_now(zone: TimeZone, fmt: TimeFormat) -> String {
    time2str(get_current_time_utc(), zone, fmt)
}

/// Convert `time_t` to a UTC timestamp in G-code format.
///
/// Time.hpp:26-29
/// C++: inline std::string utc_timestamp(time_t t)
/// C++: {
/// C++:     return time2str(t, TimeZone::utc, TimeFormat::gcode);
/// C++: }
pub fn utc_timestamp(t: i64) -> String {
    time2str(t, TimeZone::Utc, TimeFormat::GCode)
}

/// Get the current UTC timestamp in G-code format.
///
/// Time.hpp:31-34
/// C++: inline std::string utc_timestamp()
/// C++: {
/// C++:     return utc_timestamp(get_current_time_utc());
/// C++: }
pub fn utc_timestamp_now() -> String {
    utc_timestamp(get_current_time_utc())
}

/// Inner string-to-time conversion. Parses the broken-down time from `stream`
/// using the strptime emulation, then converts it to a `time_t` according to
/// `zone`. Returns `time_t(-1)` on failure.
///
/// Time.cpp:215-231
/// C++: static time_t str2time(std::istream &stream, TimeZone zone, const char *fmt)
/// C++: {
/// C++:     std::tm tms = {};
/// C++:     tms.tm_isdst = -1;
/// C++:     stream >> __get_put_time_emulation::get_time(&tms, fmt);
/// C++:     time_t ret = time_t(-1);
/// C++:     switch (zone) {
/// C++:     case TimeZone::local: ret = _mktime(&tms); break;
/// C++:     case TimeZone::utc:   ret = _timegm(&tms); break;
/// C++:     }
/// C++:     if (stream.fail() || ret < time_t(0)) ret = time_t(-1);
/// C++:     return ret;
/// C++: }
fn str2time_stream(line: &str, zone: TimeZone, fmt: &str) -> i64 {
    use chrono::{Local, NaiveDateTime, TimeZone as ChronoTimeZone, Utc};

    // stream >> get_time(&tms, fmt): parse the broken-down time.
    // The emulation reads a single line (Time.cpp:97-106), so the entire input
    // (already a single line here) is matched against `fmt`. A parse failure
    // sets the stream failbit, which we model as the early return of
    // time_t(-1) below.
    let tms = match NaiveDateTime::parse_from_str(line, fmt) {
        Ok(tms) => tms,
        Err(_) => return -1, // stream.fail() => time_t(-1)
    };

    // time_t ret = time_t(-1);  followed by the zone switch.
    let ret = match zone {
        // _mktime: broken-down local time -> time_t
        TimeZone::Local => match Local.from_local_datetime(&tms).single() {
            Some(dt) => dt.timestamp(),
            None => -1, // mktime returns -1 on failure
        },
        // _timegm: broken-down UTC time -> time_t
        TimeZone::Utc => Utc.from_utc_datetime(&tms).timestamp(),
    };

    // if (stream.fail() || ret < time_t(0)) ret = time_t(-1);
    if ret < 0 {
        -1
    } else {
        ret
    }
}

/// Parse a time string to `time_t`. Returns `time_t(-1)` if parsing fails.
///
/// Returned as `Result` for ergonomic Rust error handling: `Ok(t)` mirrors a
/// non-negative `time_t`, while `Err(..)` mirrors the C++ `time_t(-1)` sentinel.
///
/// Time.hpp:37
/// C++: time_t str2time(const std::string &str, TimeZone zone, TimeFormat fmt);
///
/// Time.cpp:233-240
/// C++: time_t str2time(const std::string &str, TimeZone zone, TimeFormat fmt)
/// C++: {
/// C++:     std::string fmtstr = process_format(get_fmtstr(fmt), zone).c_str();
/// C++:     std::stringstream ss(str);
/// C++:     ss.imbue(std::locale("C"));
/// C++:     return str2time(ss, zone, fmtstr.c_str());
/// C++: }
pub fn str2time(s: &str, zone: TimeZone, fmt: TimeFormat) -> Result<i64> {
    // Time.cpp:235
    let fmtstr = process_format(get_fmtstr(fmt), zone);

    // The C++ reads the input via std::getline, consuming a single line. Mirror
    // that by taking the first line of the input.
    let line = s.lines().next().unwrap_or("");

    let ret = str2time_stream(line, zone, &fmtstr);

    if ret == -1 {
        Err(Error::ParseError(format!(
            "Failed to parse time string '{}'",
            s
        )))
    } else {
        Ok(ret)
    }
}

/// Convert `time_t` to an ISO8601 UTC timestamp.
///
/// Time.hpp:48-51
/// C++: inline std::string iso_utc_timestamp(time_t t)
/// C++: {
/// C++:     return time2str(t, TimeZone::utc, TimeFormat::iso8601Z);
/// C++: }
pub fn iso_utc_timestamp(t: i64) -> String {
    time2str(t, TimeZone::Utc, TimeFormat::Iso8601Z)
}

/// Get the current time as an ISO8601 UTC timestamp.
///
/// Time.hpp:53-56
/// C++: inline std::string iso_utc_timestamp()
/// C++: {
/// C++:     return iso_utc_timestamp(get_current_time_utc());
/// C++: }
pub fn iso_utc_timestamp_now() -> String {
    iso_utc_timestamp(get_current_time_utc())
}

/// Parse an ISO8601 UTC timestamp string to `time_t`.
///
/// Time.hpp:58-61
/// C++: inline time_t parse_iso_utc_timestamp(const std::string &str)
/// C++: {
/// C++:     return str2time(str, TimeZone::utc, TimeFormat::iso8601Z);
/// C++: }
pub fn parse_iso_utc_timestamp(s: &str) -> Result<i64> {
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
    fn test_get_fmtstr() {
        assert_eq!(get_fmtstr(TimeFormat::GCode), "%Y-%m-%d at %H:%M:%S");
        assert_eq!(get_fmtstr(TimeFormat::Iso8601Z), "%Y%m%dT%H%M%SZ");
    }

    #[test]
    fn test_process_format() {
        // gcode + utc => " UTC" appended
        assert_eq!(
            process_format(get_fmtstr(TimeFormat::GCode), TimeZone::Utc),
            "%Y-%m-%d at %H:%M:%S UTC"
        );
        // gcode + local => unchanged
        assert_eq!(
            process_format(get_fmtstr(TimeFormat::GCode), TimeZone::Local),
            "%Y-%m-%d at %H:%M:%S"
        );
        // iso8601Z + utc => unchanged (not SLICER_UTC_TIME_FMT)
        assert_eq!(
            process_format(get_fmtstr(TimeFormat::Iso8601Z), TimeZone::Utc),
            "%Y%m%dT%H%M%SZ"
        );
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
        // gcode + utc requires the " UTC" suffix (process_format appends it).
        let s = "2021-01-01 at 00:00:00 UTC";
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
    fn test_str2time_failure() {
        // Garbage input fails to parse => Err (time_t(-1)).
        assert!(str2time("not a time", TimeZone::Utc, TimeFormat::Iso8601Z).is_err());
    }
}
