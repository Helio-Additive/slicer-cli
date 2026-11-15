//! Port of `Log.hpp` / `Log.cpp`: the logging level, the `BOOST_LOG_TRIVIAL` record stream with its
//! source location, `KV` named values with masking and quoting, and the log file set-up.
//!
//! C++ Reference:
//! - src/libslic3r/Log.hpp
//! - src/libslic3r/Log.cpp
//!
//! FIDELITY-NOTE: Boost.Log is the backend in C++ (the core with its filter and sinks, the trivial
//! logger, the record attributes and the formatter). This file ports the unit's own logic over std
//! and keeps the three parts of Boost.Log the unit relies on as small local stand-ins:
//! - the core's sink list (`CORE_SINKS`; `boost::log::core::add_sink`),
//! - the record formatter that `set_log_path_and_level` installs (`format_record`),
//! - the default sink Boost.Log writes to while no sink is registered (`default_sink_consume`,
//!   Boost.Log `default_sink.cpp`): one line on stdout.
//! The file sink itself is `crate::log_sink`.
//!
//! FIDELITY-NOTE: nothing in the crate logs through this unit except the units ported with it
//! (`nozzle_inventory.rs`); the crate's other logging goes through the `log` crate as before, and
//! `utils.rs` keeps the level functions of an earlier `Utils.cpp`. The level here is this unit's own.

use std::sync::atomic::{AtomicI32, Ordering};
use std::sync::{Arc, Mutex};

use crate::log_sink::{LogSink, LogSinkBackend, LogSinkUtil};

// Log.hpp:18-31 `struct LogEncOptions` is defined in `log_sink.rs`, where an earlier port put it.
pub use crate::log_sink::{LogEncOptions, LogEncType};

/// Log.hpp:15-16 — log severity, ordered from the most verbose to the most severe. The values are
/// those of `boost::log::trivial::severity_level` (Log.cpp:35-41).
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[repr(i32)]
pub enum LogLevel {
    Trace = 0,
    Debug = 1,
    Info = 2,
    Warning = 3,
    Error = 4,
    Fatal = 5,
}

impl LogLevel {
    /// The level of a stored `logSeverity` value.
    fn from_i32(value: i32) -> LogLevel {
        match value {
            0 => LogLevel::Trace,
            1 => LogLevel::Debug,
            2 => LogLevel::Info,
            3 => LogLevel::Warning,
            4 => LogLevel::Error,
            _ => LogLevel::Fatal,
        }
    }

    /// The name Boost.Log writes for a severity (`boost::log::trivial::to_string`).
    pub fn to_str(self) -> &'static str {
        match self {
            LogLevel::Trace => "trace",
            LogLevel::Debug => "debug",
            LogLevel::Info => "info",
            LogLevel::Warning => "warning",
            LogLevel::Error => "error",
            LogLevel::Fatal => "fatal",
        }
    }
}

/// Log.cpp:43-44 — single source of the current level, read by the macros and the core filter from
/// any thread.
///
/// Log.cpp:108-119 `g_RunOnInit` installs the core filter and calls `set_logging_level(1)` when the
/// library is loaded; the initial value here is that level (error), and the filter is the
/// `log_enabled` check in `core_push_record`.
static LOG_SEVERITY: AtomicI32 = AtomicI32::new(LogLevel::Error as i32);

/// Log.cpp:46-62
fn level_to_boost(level: u32) -> LogLevel {
    match level {
        // Report fatal errors only.
        0 => LogLevel::Fatal,
        // Report fatal errors and errors.
        1 => LogLevel::Error,
        // Report fatal errors, errors and warnings.
        2 => LogLevel::Warning,
        // Report all errors, warnings and infos.
        3 => LogLevel::Info,
        // Report all errors, warnings, infos and debugging.
        4 => LogLevel::Debug,
        // Report everything including fine level tracing information.
        _ => LogLevel::Trace,
    }
}

/// Log.cpp:64-67 — set the logging level: 0 fatal, 1 error, 2 warning, 3 info, 4 debug, 5 and above
/// trace.
pub fn set_logging_level(level: u32) {
    LOG_SEVERITY.store(level_to_boost(level) as i32, Ordering::Relaxed);
}

/// Log.cpp:69-77 — convert a level name ("fatal" ... "trace") to its number; unknown names give 0.
pub fn level_string_to_boost(level: &str) -> u32 {
    const K_LEVELS: [(&str, u32); 6] =
        [("fatal", 0), ("error", 1), ("warning", 2), ("info", 3), ("debug", 4), ("trace", 5)];
    for (name, value) in K_LEVELS.iter() {
        if *name == level {
            return *value;
        }
    }
    0
}

/// Log.cpp:79-90 — convert a level number to its name; out of range gives "error".
pub fn get_string_logging_level(level: u32) -> String {
    match level {
        0 => "fatal".to_string(),
        1 => "error".to_string(),
        2 => "warning".to_string(),
        3 => "info".to_string(),
        4 => "debug".to_string(),
        5 => "trace".to_string(),
        _ => "error".to_string(),
    }
}

/// Log.cpp:92-103 — current logging level, numbered as in `set_logging_level`.
pub fn get_logging_level() -> u32 {
    match LogLevel::from_i32(LOG_SEVERITY.load(Ordering::Relaxed)) {
        LogLevel::Fatal => 0,
        LogLevel::Error => 1,
        LogLevel::Warning => 2,
        LogLevel::Info => 3,
        LogLevel::Debug => 4,
        LogLevel::Trace => 5,
    }
}

/// Log.cpp:121-126 — `g_log_sink`, `g_log_sink_backend` and `g_log_dir`, the folder given to
/// `set_log_path_and_level`; later files go to the same place.
struct LogGlobals {
    g_log_sink: Option<Arc<LogSink>>,
    g_log_sink_backend: Option<Arc<Mutex<LogSinkBackend>>>,
    g_log_dir: String,
}

static LOG_GLOBALS: Mutex<LogGlobals> =
    Mutex::new(LogGlobals { g_log_sink: None, g_log_sink_backend: None, g_log_dir: String::new() });

/// The sinks registered with `boost::log::core::get()->add_sink` (Log.cpp:153). Every call of
/// `set_log_path_and_level` adds one and none is ever removed, as in C++.
static CORE_SINKS: Mutex<Vec<Arc<LogSink>>> = Mutex::new(Vec::new());

/// Lock a mutex of this unit; a panic of another logging thread must not stop logging.
fn lock<T>(mutex: &Mutex<T>) -> std::sync::MutexGuard<'_, T> {
    mutex.lock().unwrap_or_else(|poisoned| poisoned.into_inner())
}

/// Log.cpp:128-131 — whether the log file has been set up.
pub fn is_log_trivial_valid() -> bool {
    lock(&LOG_GLOBALS).g_log_sink.is_some()
}

/// Log.cpp:138-159 — start writing logs to a new file in `log_dir` and set the logging level.
///
/// The file is named after the start time and process id; `log_dir` is created if missing.
pub fn set_log_path_and_level(log_dir: &str, level: u32, enc_options: &LogEncOptions) {
    let mut globals = lock(&LOG_GLOBALS);
    globals.g_log_dir = log_dir.to_string();
    // Also creates the log folder, so it runs before the early return below.
    let file = LogSinkUtil::get_log_filename_format(&globals.g_log_dir, enc_options);
    let backend = Arc::new(Mutex::new(LogSinkBackend::new(&file, enc_options)));
    globals.g_log_sink_backend = Some(backend.clone());
    // Log.cpp:145-152 `set_formatter`: the format is `format_record`.
    let sink = Arc::new(LogSink::new(backend));
    globals.g_log_sink = Some(sink.clone());
    lock(&CORE_SINKS).push(sink);

    // Log.cpp:155 `logging::add_common_attributes()`: the time stamp and the thread id are taken
    // when a record is made (`LogRecord::new`).

    set_logging_level(level);
}

/// Log.cpp:161-167 — apply new encryption options; logging continues in a new file in the same
/// folder.
pub fn update_log_sink(enc_options: &LogEncOptions) {
    let globals = lock(&LOG_GLOBALS);
    if let Some(backend) = globals.g_log_sink_backend.as_ref() {
        let file = LogSinkUtil::get_log_filename_format(&globals.g_log_dir, enc_options);
        lock(backend).update_enc_option(&file, enc_options);
    }
}

/// Log.cpp:169-181 — message of the exception being handled, for logging it in a catch block.
///
/// FIDELITY-NOTE: C++ reads the exception being handled (`std::current_exception()`); Rust has no
/// such state, so the caller passes what it caught: `None` outside a handler ("no exception"), the
/// payload of a caught panic otherwise. A payload that is a `String` or a `&str` (the message of a
/// panic, as `what()` is of a `std::exception`) gives that text; anything else gives "unknown
/// exception".
pub fn current_exception_message(exception: Option<&(dyn std::any::Any + Send)>) -> String {
    // A bare throw with no exception being handled would terminate the program.
    let Some(exception) = exception else {
        return "no exception".to_string();
    };
    if let Some(what) = exception.downcast_ref::<String>() {
        what.clone()
    } else if let Some(what) = exception.downcast_ref::<&'static str>() {
        (*what).to_string()
    } else {
        "unknown exception".to_string()
    }
}

/// Log.cpp:183-189 — write out buffered log records.
pub fn flush_logs() {
    let sink = lock(&LOG_GLOBALS).g_log_sink.clone();
    if let Some(sink) = sink {
        sink.flush();
    }
}

/// One record on its way to the sinks: what `boost::log::record_view` carries for this unit.
#[derive(Debug, Clone)]
pub struct LogRecord {
    /// The "TimeStamp" attribute: local time when the record was made.
    pub time_stamp: chrono::DateTime<chrono::Local>,
    /// The "ThreadID" attribute, as Boost.Log writes it.
    pub thread_id: String,
    /// The severity of the trivial logger.
    pub severity: LogLevel,
    /// The message.
    pub message: String,
}

impl LogRecord {
    /// Make a record of the current thread, stamped now.
    pub fn new(severity: LogLevel, message: String) -> LogRecord {
        LogRecord { time_stamp: chrono::Local::now(), thread_id: current_thread_id(), severity, message }
    }
}

/// The current thread's id in the form Boost.Log writes one (`thread_id.cpp` `format_id`): `0x` and
/// sixteen hexadecimal digits.
///
/// FIDELITY-NOTE: Boost.Log writes the native thread id (`pthread_self()`); std gives no native id,
/// so the number is the one of `std::thread::ThreadId`. Only the digits differ.
fn current_thread_id() -> String {
    let id = format!("{:?}", std::thread::current().id());
    let number: u64 = id.chars().filter(|c| c.is_ascii_digit()).collect::<String>().parse().unwrap_or(0);
    format!("0x{:016x}", number)
}

/// Log.cpp:145-152 — the formatter of the file sink: time stamp, thread id, the severity padded to
/// the longest name ("warning") so the columns line up, and the message.
pub fn format_record(record: &LogRecord) -> String {
    format!(
        "{} [{}] [{:<7}] {}",
        record.time_stamp.format("%Y-%m-%d %H:%M:%S%.6f"),
        record.thread_id,
        record.severity.to_str(),
        record.message
    )
}

/// The line Boost.Log's default sink prints for a record (`default_sink.cpp`, `message_printer`
/// and `severity_level_to_string`), without its line end.
pub fn default_sink_format(record: &LogRecord) -> String {
    let severity = match record.severity {
        LogLevel::Trace => "[trace]  ",
        LogLevel::Debug => "[debug]  ",
        LogLevel::Info => "[info]   ",
        LogLevel::Warning => "[warning]",
        LogLevel::Error => "[error]  ",
        LogLevel::Fatal => "[fatal]  ",
    };
    format!(
        "[{}] [{}] {} {}",
        record.time_stamp.format("%Y-%m-%d %H:%M:%S%.6f"),
        record.thread_id,
        severity,
        record.message
    )
}

/// What Boost.Log does with a record while no sink is registered (`default_sink::consume`): one
/// line on stdout, then a flush.
fn default_sink_consume(record: &LogRecord) {
    use std::io::Write;
    let stdout = std::io::stdout();
    let mut out = stdout.lock();
    let _ = writeln!(out, "{}", default_sink_format(record));
    let _ = out.flush();
}

/// `BOOST_LOG_STREAM_WITH_PARAMS(logger::get(), severity) << message` (Log.cpp:307-309): the core
/// filter of Log.cpp:112-115, then every registered sink, or the default sink if there is none.
fn core_push_record(level: LogLevel, message: String) {
    // Same check as the logging macros, which also covers records not made through them.
    if !log_detail::log_enabled(level) {
        return;
    }
    let record = LogRecord::new(level, message);
    let sinks: Vec<Arc<LogSink>> = lock(&CORE_SINKS).clone();
    if sinks.is_empty() {
        default_sink_consume(&record);
        return;
    }
    for sink in sinks.iter() {
        sink.consume(&record, &format_record(&record));
    }
}

/// Log.hpp:68-190, Log.cpp:193-315 — `namespace Slic3r::log_detail`.
pub mod log_detail {
    use super::{LogLevel, LOG_SEVERITY};
    use crate::log_sink::LogSinkBackend;
    use std::sync::atomic::Ordering;

    /// Log.hpp:82-89 — whether `T` holds a small number in a char-sized type: signed/unsigned char.
    ///
    /// `std::ostream` writes these as characters, so the number 0 would come out as a NUL byte.
    ///
    /// FIDELITY-NOTE: C++ also answers true for an enum based on a char-sized type; Rust cannot ask
    /// for the representation of an enum, and an enum is written through its own `WriteValue`.
    pub fn is_char_number<T: 'static>() -> bool {
        let t = std::any::TypeId::of::<T>();
        t == std::any::TypeId::of::<i8>() || t == std::any::TypeId::of::<u8>()
    }

    /// Log.hpp:116-134 `write_value` — write a value to a log stream.
    ///
    /// A null char pointer (`None` of `Option<&str>`) is written as "(null)", char-sized numbers are
    /// written as numbers, a json has its sensitive fields masked, and `char` is written as a
    /// character. Everything else is written as `std::ostream` writes it: bools as true/false
    /// (the streams of this unit set `boolalpha`), floats with six significant digits.
    ///
    /// FIDELITY-NOTE: the `ToUTF8()` branch (Log.hpp:91-100, 129-130) is for wxString, which the
    /// crate does not have; a Rust string is UTF-8 already.
    pub trait WriteValue {
        fn write_value(&self, os: &mut String);
    }

    impl<T: WriteValue + ?Sized> WriteValue for &T {
        fn write_value(&self, os: &mut String) {
            (**self).write_value(os)
        }
    }

    impl WriteValue for str {
        fn write_value(&self, os: &mut String) {
            os.push_str(self);
        }
    }

    impl WriteValue for String {
        fn write_value(&self, os: &mut String) {
            os.push_str(self);
        }
    }

    /// Log.hpp:119-124 — a char pointer; null is written as "(null)".
    impl WriteValue for Option<&str> {
        fn write_value(&self, os: &mut String) {
            match self {
                None => os.push_str("(null)"),
                Some(value) => os.push_str(value),
            }
        }
    }

    impl WriteValue for char {
        fn write_value(&self, os: &mut String) {
            os.push(*self);
        }
    }

    impl WriteValue for bool {
        fn write_value(&self, os: &mut String) {
            os.push_str(if *self { "true" } else { "false" });
        }
    }

    /// Log.hpp:125-126 — `os << static_cast<int>(value)` for the char-sized numbers.
    impl WriteValue for i8 {
        fn write_value(&self, os: &mut String) {
            os.push_str(&(*self as i32).to_string());
        }
    }

    impl WriteValue for u8 {
        fn write_value(&self, os: &mut String) {
            os.push_str(&(*self as i32).to_string());
        }
    }

    macro_rules! write_value_integer {
        ($($t:ty),*) => {$(
            impl WriteValue for $t {
                fn write_value(&self, os: &mut String) {
                    os.push_str(&self.to_string());
                }
            }
        )*};
    }
    write_value_integer!(i16, u16, i32, u32, i64, u64, isize, usize);

    impl WriteValue for f64 {
        fn write_value(&self, os: &mut String) {
            os.push_str(&ostream_double(*self));
        }
    }

    impl WriteValue for f32 {
        fn write_value(&self, os: &mut String) {
            os.push_str(&ostream_double(*self as f64));
        }
    }

    /// Log.hpp:127-128 — a json is written with its sensitive fields masked.
    impl WriteValue for serde_json::Value {
        fn write_value(&self, os: &mut String) {
            write_json(os, self);
        }
    }

    /// `os << value` for a floating point number on a stream with its default flags: `%g` with a
    /// precision of 6.
    fn ostream_double(value: f64) -> String {
        if value.is_nan() {
            return if value.is_sign_negative() { "-nan".to_string() } else { "nan".to_string() };
        }
        if value.is_infinite() {
            return if value < 0.0 { "-inf".to_string() } else { "inf".to_string() };
        }
        if value == 0.0 {
            return if value.is_sign_negative() { "-0".to_string() } else { "0".to_string() };
        }
        // The decimal exponent of the value rounded to six significant digits decides the style.
        let scientific = format!("{:.5e}", value);
        let (mantissa, exponent) = scientific.split_once('e').unwrap_or((scientific.as_str(), "0"));
        let exponent: i32 = exponent.parse().unwrap_or(0);
        let trim = |digits: &str| -> String {
            if digits.contains('.') {
                digits.trim_end_matches('0').trim_end_matches('.').to_string()
            } else {
                digits.to_string()
            }
        };
        if !(-4..6).contains(&exponent) {
            let sign = if exponent < 0 { '-' } else { '+' };
            format!("{}e{}{:02}", trim(mantissa), sign, exponent.abs())
        } else {
            trim(&format!("{:.*}", (5 - exponent) as usize, value))
        }
    }

    /// Log.cpp:196-204 — keeps only the file name of `__FILE__`; the build directory layout is not
    /// useful in the log.
    pub fn file_basename(path: &str) -> &str {
        let mut name = path;
        for (p, c) in path.char_indices() {
            if c == '/' || c == '\\' {
                name = &path[p + 1..];
            }
        }
        name
    }

    /// Log.cpp:206-210 — fields that are not registered for masking are written as is.
    fn masked(key: &str, value: &str) -> String {
        crate::desensitize::mask_by_key(key, value).unwrap_or_else(|| value.to_string())
    }

    /// Log.cpp:212-234 — quotes values that would otherwise be ambiguous in "key=value" form: empty,
    /// or containing a separator, a quote or a line break.
    pub fn quoted_if_needed(value: &str) -> String {
        if !value.is_empty() && !value.contains([' ', '=', '"', '\t', '\r', '\n']) {
            return value.to_string();
        }

        let mut out = String::with_capacity(value.len() + 2);
        out.push('"');
        for c in value.chars() {
            match c {
                '"' => out.push_str("\\\""),
                '\\' => out.push_str("\\\\"),
                '\n' => out.push_str("\\n"),
                '\r' => out.push_str("\\r"),
                '\t' => out.push_str("\\t"),
                _ => out.push(c),
            }
        }
        out.push('"');
        out
    }

    /// Log.cpp:237-244 — build "key=value " for `kv`.
    pub fn format_kv(key: &str, value: &str) -> String {
        let mut out = String::from(key);
        out.push('=');
        out.push_str(&quoted_if_needed(&masked(key, value)));
        out.push(' ');
        out
    }

    /// Log.cpp:246-254 — write a json with its sensitive fields masked, on one line. Never panics.
    pub fn write_json(os: &mut String, value: &serde_json::Value) {
        // json_log masks by field name in public builds; copying a large json can still run out of memory.
        match std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| crate::desensitize::json_log(value, -1))) {
            Ok(text) => os.push_str(&text),
            Err(_) => write_failed(os),
        }
    }

    /// Log.cpp:256-262 — write a placeholder for a value that could not be written; never panics.
    pub fn write_failed(os: &mut String) {
        os.push_str("<value not logged>");
    }

    /// Log.cpp:264-271 — "key=<value not logged> " for a value that could not be written.
    pub fn kv_failed(key: &str) -> String {
        format!("{}=<value not logged> ", key)
    }

    /// Log.cpp:273-279 — whether a record of the given level passes the current logging level.
    pub fn log_enabled(level: LogLevel) -> bool {
        // A record made while this thread is writing another one would deadlock on the sink's locks.
        if LogSinkBackend::is_consuming_on_this_thread() {
            return false;
        }
        level as i32 >= LOG_SEVERITY.load(Ordering::Relaxed)
    }

    /// Log.hpp:150-181, Log.cpp:281-313 — collects one log record and writes it when the statement
    /// ends. Created only by `boost_log_trivial!`.
    pub struct LogStream {
        m_level: LogLevel,
        m_buf: String,
    }

    impl LogStream {
        /// Log.cpp:281-292
        ///
        /// FIDELITY-NOTE: the location is that of the Rust statement (`file!()`, `line!()` and the
        /// enclosing function), so it names the Rust file and line, not the C++ ones.
        pub fn new(level: LogLevel, file: &str, line: u32, func: &str) -> LogStream {
            // The location leads the message; records from code outside our macro simply have none.
            let m_buf = format!("[{}:{} {}] ", file_basename(file), line, func);
            LogStream { m_level: level, m_buf }
        }

        /// Log.hpp:164-174 `operator<<`
        pub fn write<T: WriteValue + ?Sized>(&mut self, value: &T) -> &mut LogStream {
            // A value whose own formatting panics must not break the code that logs it.
            let buf = &mut self.m_buf;
            if std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| value.write_value(buf))).is_err() {
                write_failed(&mut self.m_buf);
            }
            self
        }
    }

    /// Log.cpp:300-313 `LogStream::~LogStream`
    impl Drop for LogStream {
        fn drop(&mut self) {
            let mut message = std::mem::take(&mut self.m_buf);
            // The sink ends every record with a newline; one written by the caller would leave an empty line.
            while message.ends_with('\n') || message.ends_with('\r') {
                message.pop();
            }
            let level = self.m_level;
            // A destructor must not panic, and a failure to log has nowhere else to be reported.
            let _ = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| super::core_push_record(level, message)));
        }
    }

    /// `__func__` for `boost_log_trivial!`: the last path segment of the enclosing function, from the
    /// type name of an item declared in it.
    pub fn function_name(type_name_of_inner_item: &'static str) -> &'static str {
        let mut name = type_name_of_inner_item;
        // The item the macro declares, then any closures the statement is in.
        name = name.strip_suffix("::slic3r_log_fn_").unwrap_or(name);
        while let Some(outer) = name.strip_suffix("::{{closure}}") {
            name = outer;
        }
        match name.rfind("::") {
            Some(pos) => &name[pos + 2..],
            None => name,
        }
    }

    /// The type name of a value, for `function_name`.
    pub fn type_name_of<T>(_: &T) -> &'static str {
        std::any::type_name::<T>()
    }
}

/// Log.hpp:194-215 `KV` — format a named value for a log record as "key=value ".
///
/// Values of sensitive keys are masked in public builds, and values containing spaces or quotes are
/// quoted. Pass the raw value, not an already masked one.
pub fn kv<T: log_detail::WriteValue + ?Sized>(key: &str, value: &T) -> String {
    // Evaluated inside log statements, so a failure must not reach the caller.
    let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        let mut s = String::new();
        value.write_value(&mut s);
        log_detail::format_kv(key, &s)
    }));
    match result {
        Ok(text) => text,
        Err(_) => log_detail::kv_failed(key),
    }
}

/// Log.hpp:219-225 `BOOST_LOG_TRIVIAL(lvl) << a << b;` is `boost_log_trivial!(Lvl, a, b);`.
///
/// The values are evaluated only when the level passes, so they cost nothing for filtered-out
/// records.
#[macro_export]
macro_rules! boost_log_trivial {
    ($lvl:ident, $($value:expr),+ $(,)?) => {{
        if $crate::log::log_detail::log_enabled($crate::log::LogLevel::$lvl) {
            fn slic3r_log_fn_() {}
            let mut slic3r_log_stream_ = $crate::log::log_detail::LogStream::new(
                $crate::log::LogLevel::$lvl,
                file!(),
                line!(),
                $crate::log::log_detail::function_name($crate::log::log_detail::type_name_of(&slic3r_log_fn_)),
            );
            $( slic3r_log_stream_.write(&$value); )+
        }
    }};
}

#[cfg(test)]
mod tests {
    use super::log_detail::*;
    use super::*;

    // Log.cpp:69-90
    #[test]
    fn level_names_and_numbers() {
        for (name, value) in [("fatal", 0), ("error", 1), ("warning", 2), ("info", 3), ("debug", 4), ("trace", 5)] {
            assert_eq!(level_string_to_boost(name), value);
            assert_eq!(get_string_logging_level(value), name);
        }
        assert_eq!(level_string_to_boost("Fatal"), 0);
        assert_eq!(level_string_to_boost(""), 0);
        assert_eq!(get_string_logging_level(6), "error");
    }

    // Log.cpp:46-62
    #[test]
    fn level_to_boost_table() {
        assert_eq!(level_to_boost(0), LogLevel::Fatal);
        assert_eq!(level_to_boost(1), LogLevel::Error);
        assert_eq!(level_to_boost(2), LogLevel::Warning);
        assert_eq!(level_to_boost(3), LogLevel::Info);
        assert_eq!(level_to_boost(4), LogLevel::Debug);
        assert_eq!(level_to_boost(5), LogLevel::Trace);
        assert_eq!(level_to_boost(77), LogLevel::Trace);
    }

    // Log.cpp:196-204
    #[test]
    fn file_basename_cases() {
        assert_eq!(file_basename("a/b/c.cpp"), "c.cpp");
        assert_eq!(file_basename("a\\b\\c.cpp"), "c.cpp");
        assert_eq!(file_basename("c.cpp"), "c.cpp");
        assert_eq!(file_basename("a/"), "");
        assert_eq!(file_basename(""), "");
    }

    // Log.cpp:169-181
    #[test]
    fn current_exception_message_cases() {
        assert_eq!(current_exception_message(None), "no exception");
        let caught = std::panic::catch_unwind(|| std::panic::panic_any("what".to_string())).unwrap_err();
        assert_eq!(current_exception_message(Some(caught.as_ref())), "what");
        let caught = std::panic::catch_unwind(|| std::panic::panic_any(7_i32)).unwrap_err();
        assert_eq!(current_exception_message(Some(caught.as_ref())), "unknown exception");
    }

    #[test]
    fn function_name_cases() {
        assert_eq!(function_name("slicer::a::b::slic3r_log_fn_"), "b");
        assert_eq!(function_name("slicer::a::<impl slicer::a::T>::m::{{closure}}::{{closure}}::slic3r_log_fn_"), "m");
        assert_eq!(function_name("f"), "f");
    }
}
