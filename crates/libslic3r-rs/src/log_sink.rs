//! 1:1 port of LogSink.cpp / LogSink.hpp
//!
//! C++ Reference:
//! - src/libslic3r/LogSink.hpp
//! - src/libslic3r/LogSink.cpp
//!
//! This module is the BambuStudio thread-safe log sink backend with optional
//! AES-256-CBC log-file encryption.
//!
//! PORTING NOTES / BLOCKED SYMBOLS (status: partial):
//! Upstream's encrypted-file path is built on two hard native dependencies that
//! the parity port forbids adding (neither is wasm-safe and neither touches the
//! G-code path):
//!   * OpenSSL `AES_cbc_encrypt` / `AES_set_*_key` (`<openssl/aes.h>`) used by
//!     `consume` and `DecodeAES256LogFile` via `BBL_Encrypt::AES256CBC_*`.
//!   * libcurl through `Slic3r::Http` (cloud key fetch in `get_aes_256_cbc`).
//! Consequently the following members of upstream's file are NOT ported and are
//! documented inline at their original line refs (`slicer_cli` does not compile
//! them either, see THE FILE SINK below):
//!   * the encrypting branch of `LogSinkBackend::consume`.
//!   * `LogSinkBackend::try_record_new_log_file` — the header of an encrypted file.
//!   * `LogSinkBackend::update_log_enc_key` — the key of an encrypted file.
//!   * `LogSinkBackend::get_aes_256_cbc` cloud branch — `Slic3r::Http`.
//!   * `LogSinkBackend::DecodeAES256LogFile` — OpenSSL AES decrypt.
//!   * `s_generate_uuid` — boost::uuids.
//! Everything tractable WITHOUT those backends is ported faithfully below.
//!
//! THE FILE SINK (`slicer_cli`'s `LogSink.cpp`): the Bambu Studio build of this repository compiles
//! the override-layer copy of this unit (`libslic3r/bambustudio/libslic3r/LogSink.cpp`), which is
//! upstream's without the encrypted-file path: every record is written as plain text. Its members
//! are ported from that file and cite it as `LogSink.cpp (override)`: `current_pid`,
//! `is_consuming_on_this_thread`, the constructor (`new`), `update_enc_option`, `consume` and
//! `LogSinkUtil::get_log_filename_format`, with `LogSink` (LogSink.hpp:79) as the front end.
//!
//! FIDELITY-NOTE: `text_file_backend` of Boost.Log is the base class in C++. `TextFileBackend`
//! below is the part of it this unit uses, over std: the `%N` file counter of the name pattern,
//! the rotation size, append mode, and `consume` / `rotate_file` / `flush` in Boost.Log's order
//! (`text_file_backend.cpp`). It has no file collector, no target file name and no time-based
//! rotation, which the unit never sets.

// LogSink.cpp:14 : #include <openssl/aes.h>
// AES_BLOCK_SIZE is defined by OpenSSL as 16. Mirror the constant so the
// pure-logic ported helpers compute identical values.
/// OpenSSL `AES_BLOCK_SIZE`
const AES_BLOCK_SIZE: usize = 16;

// LogSink.cpp:20
#[allow(dead_code)]
const HEADER_BEGIN_MARKER: &str = "BEGIN_HEADER\n";
// LogSink.cpp:21
#[allow(dead_code)]
const HEADER_END_MARKER: &str = "\nEND_HEADER\n";

// LogSink.cpp:26
// static std::string s_message_newline = "\n";
#[allow(dead_code)]
const S_MESSAGE_NEWLINE: &str = "\n";

// ===========================================================================
// LogEncOptions (Utils.hpp:366-378) — prerequisite data type for LogSink.
// ===========================================================================

/// Log encryption type.
/// Utils.hpp:368-373
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LogEncType {
    /// Utils.hpp:370 : LOG_ENC_NONE = 0,
    LogEncNone = 0,
    /// Utils.hpp:371 : LOG_ENC_AES_256_CBC = 1,
    LogEncAes256Cbc = 1,
    // Utils.hpp:372 : //ENC_RSA_2048 = 2, maybe supported in future
}

/// Log encryption options.
/// Utils.hpp:366-378
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct LogEncOptions {
    /// Utils.hpp:375 : LogEncType enc_type = LOG_ENC_AES_256_CBC;
    pub enc_type: LogEncType,
    /// Utils.hpp:376 : std::string enc_key_url;
    pub enc_key_url: String,
    /// Utils.hpp:377 : std::string enc_key_host_env;
    pub enc_key_host_env: String,
}

impl Default for LogEncOptions {
    fn default() -> Self {
        // Utils.hpp:375-377 : default member initializers
        Self {
            enc_type: LogEncType::LogEncAes256Cbc,
            enc_key_url: String::new(),
            enc_key_host_env: String::new(),
        }
    }
}

// ===========================================================================
// Default keys (LogSink.cpp:232-240)
// ===========================================================================

// LogSink.cpp:232
const DEFAULT_KEY_TAG_CN_1: &str = "68ba6f1721a2a225e9a499c1f73678931761106584";
// LogSink.cpp:233
const DEFAULT_KEY_STR_CN_1: &str = "OruMpXAHc7K8cgqLbJnRbAPOcQmFnH3J";
// LogSink.cpp:234
const DEFAULT_KEY_IV_CN_1: &str = "Ln2XZ0u6SLGfhftc";
// LogSink.cpp:235
const DEFAULT_KEY_TIME_CN_1: i64 = 202510221215;

// LogSink.cpp:237
const DEFAULT_KEY_TAG_US_1: &str = "7f79c976547b46fa4e1293f78b0e94541761106519";
// LogSink.cpp:238
const DEFAULT_KEY_STR_US_1: &str = "tzLvjYZy8QDFqVOirPxDxEDF0yFENgl0";
// LogSink.cpp:239
const DEFAULT_KEY_IV_US_1: &str = "YGUgmQ9mCs5N3yqJ";
// LogSink.cpp:240
const DEFAULT_KEY_TIME_US_1: i64 = 202510221215;

// ===========================================================================
// LogSinkBackend (LogSink.hpp:17-62)
// ===========================================================================

/// thread-safe log sink backend with encryption support
/// LogSink.hpp:16-17 : class LogSinkBackend : public boost::log::sinks::text_file_backend
///
/// NOTE: The boost.log `text_file_backend` base class is a native dependency
/// that cannot be ported (see module docs). Only the pure encryption-info
/// state and the pure-logic methods are mirrored here.
#[derive(Debug, Default)]
pub struct LogSinkBackend {
    /// The `boost::log::sinks::text_file_backend` base class (LogSink.hpp:18).
    pub m_text_file_backend: TextFileBackend,

    // LogSink.hpp:20 : std::mutex m_log_mutex; (provided by parking_lot in
    // the runtime backend; the pure-logic methods ported here are &self).

    // LogSink.hpp:23 : LogEncOptions m_log_enc_options;
    /// encryption options
    pub m_log_enc_options: LogEncOptions,

    // encryption info (LogSink.hpp:26-30)
    /// LogSink.hpp:26 : std::string m_log_enc_key;
    pub m_log_enc_key: String,
    /// LogSink.hpp:27 : std::string m_log_enc_key_iv_base; //the original iv from server
    pub m_log_enc_key_iv_base: String,
    /// LogSink.hpp:28 : std::string m_log_enc_key_iv;
    pub m_log_enc_key_iv: String,
    /// LogSink.hpp:29 : std::string m_log_enc_key_tag;
    pub m_log_enc_key_tag: String,
    /// LogSink.hpp:30 : time_t  m_log_enc_key_timestamp = 0;
    pub m_log_enc_key_timestamp: i64,

    // LogSink.hpp:33 : std::unordered_set<std::string> m_log_files;
    /// record of generated log files
    pub m_log_files: std::collections::HashSet<String>,
}

impl LogSinkBackend {
    // LogSink.cpp:37-44
    /// LogSink.hpp:61 : void reset_enc_key_info();
    pub fn reset_enc_key_info(&mut self) {
        self.m_log_enc_key.clear(); // LogSink.cpp:39
        self.m_log_enc_key_iv_base.clear(); // LogSink.cpp:40
        self.m_log_enc_key_iv.clear(); // LogSink.cpp:41
        self.m_log_enc_key_tag.clear(); // LogSink.cpp:42
        self.m_log_enc_key_timestamp = 0; // LogSink.cpp:43
    }

    // LogSink.cpp:243-255
    // warning: do not use BOOST_LOG_TRIVIAL in this function to avoid deadlock
    /// LogSink.hpp:60 : std::string get_enc_key_type(const std::string& key_tag);
    pub fn get_enc_key_type(&self, key_tag: &str) -> String {
        // LogSink.cpp:245-247
        if key_tag.is_empty() {
            return String::new();
        }

        // LogSink.cpp:249-252
        // static std::unordered_set<std::string> s_local_tags = { ... };
        let s_local_tags: [&str; 2] = [DEFAULT_KEY_TAG_CN_1, DEFAULT_KEY_TAG_US_1];

        // LogSink.cpp:254
        if s_local_tags.contains(&key_tag) {
            "local".to_string()
        } else {
            "cloud".to_string()
        }
    }

    // LogSink.cpp:258-297
    // warning: do not use BOOST_LOG_TRIVIAL in this function to avoid deadlock
    /// LogSink.hpp:54-58 : void get_aes_256_cbc(...) const;
    ///
    /// Only the default-key selection branch (LogSink.cpp:264-275) is ported.
    /// The cloud key fetch (LogSink.cpp:277-296) requires `Slic3r::Http`
    /// (libcurl, native) and is NOT ported; see module docs.
    ///
    /// NOTE on `key_time`: in C++ this parameter is `time_t key_time` passed
    /// BY VALUE (LogSink.cpp:262), so the assignments to it (LogSink.cpp:269,
    /// 274, 288) are dead writes that never reach the caller — the caller at
    /// LogSink.cpp:313 passes `m_log_enc_key_timestamp`, which is therefore
    /// never updated by this call. We mirror the by-value signature exactly so
    /// the (lack of) propagation matches; the writes are intentionally local.
    #[allow(unused_assignments, unused_mut, unused_variables)]
    pub fn get_aes_256_cbc(
        &self,
        enc_options: &LogEncOptions,
        key_str: &mut String,
        key_iv: &mut String,
        key_tag: &mut String,
        mut key_time: i64,
    ) {
        // the default key
        // LogSink.cpp:265-275
        if enc_options.enc_key_host_env == "cn" {
            *key_tag = DEFAULT_KEY_TAG_CN_1.to_string(); // LogSink.cpp:266
            *key_str = DEFAULT_KEY_STR_CN_1.to_string(); // LogSink.cpp:267
            *key_iv = DEFAULT_KEY_IV_CN_1.to_string(); // LogSink.cpp:268
            key_time = DEFAULT_KEY_TIME_CN_1; // LogSink.cpp:269 (by-value, dead write)
        } else {
            *key_tag = DEFAULT_KEY_TAG_US_1.to_string(); // LogSink.cpp:271
            *key_str = DEFAULT_KEY_STR_US_1.to_string(); // LogSink.cpp:272
            *key_iv = DEFAULT_KEY_IV_US_1.to_string(); // LogSink.cpp:273
            key_time = DEFAULT_KEY_TIME_US_1; // LogSink.cpp:274 (by-value, dead write)
        }

        // LogSink.cpp:277-296 : cloud key fetch via Slic3r::Http (libcurl).
        // BLOCKED — native HTTP backend not available / not wasm-safe.
        // The cloud branch also writes `key_time = time(nullptr)` (LogSink.cpp:288),
        // likewise a dead by-value write.
        let _ = enc_options.enc_key_url;
    }
}

// LogSink.cpp:69-79
// static int s_get_enc_block_size(LogEncOptions::LogEncType enc_type)
fn s_get_enc_block_size(enc_type: LogEncType) -> i32 {
    // LogSink.cpp:71-77
    match enc_type {
        // LogSink.cpp:72-73
        LogEncType::LogEncAes256Cbc => AES_BLOCK_SIZE as i32,
        // LogSink.cpp:74-76 : default: break;
        _ => AES_BLOCK_SIZE as i32, // LogSink.cpp:78 : return AES_BLOCK_SIZE;
    }
}

// ===========================================================================
// LogSinkUtil (LogSink.hpp:64-69)
// ===========================================================================

/// LogSink.hpp:64-69 : class LogSinkUtil
pub struct LogSinkUtil;

impl LogSinkUtil {
    // rules
    // studio_%a_%b_%d_%H_%M_%S_<pid>[_enc][_cn].log.%N
    // LogSink.cpp:410-446
    /// LogSink.hpp:68 : static std::string get_log_filaname_format(const LogEncOptions& enc_opts);
    pub fn get_log_filaname_format(enc_opts: &LogEncOptions) -> String {
        // LogSink.cpp:412-413
        // std::time_t t = std::time(0); std::tm* now_time = std::localtime(&t);
        let now_time = chrono::Local::now();

        // LogSink.cpp:414-416
        // std::stringstream buf;
        // buf << std::put_time(now_time, "studio_%a_%b_%d_%H_%M_%S_");
        // buf << get_current_pid();
        //
        // strftime conversion specifiers used:
        //   %a abbreviated weekday, %b abbreviated month, %d day of month,
        //   %H hour(24), %M minute, %S second.
        // chrono::format uses the same specifiers; %a/%b are locale-dependent
        // in C++ (default "C" locale -> English abbreviations), which chrono
        // emits unconditionally, matching the C-locale default.
        let mut buf = now_time
            .format("studio_%a_%b_%d_%H_%M_%S_")
            .to_string();
        // LogSink.cpp:416 : buf << get_current_pid();
        // Utils.cpp:1179-1186 : get_current_pid() => ::getpid() / GetCurrentProcessId()
        buf.push_str(&get_current_pid().to_string());

        // LogSink.cpp:417-431
        if enc_opts.enc_type == LogEncType::LogEncNone {
            // LogSink.cpp:418
            buf.push_str(".log.%N");
        } else {
            // default to us
            // LogSink.cpp:421
            buf.push_str("_enc");
            // LogSink.cpp:422-428
            if enc_opts.enc_key_host_env == "cn" {
                buf.push_str("_cn"); // LogSink.cpp:423
            } else if enc_opts.enc_key_host_env == "dc" {
                buf.push_str("_dc"); // LogSink.cpp:425 : dc means the user doesn't set the region
            } else {
                // LogSink.cpp:426-427 : no suffix for "us" region
            }

            // LogSink.cpp:430
            buf.push_str(".log.%N");
        }

        //BBS log file at C:\\Users\\[yourname]\\AppData\\Roaming\\BambuStudio\\log\\[log_filename].log
        // LogSink.cpp:434-443
        // try {
        //   auto log_folder = boost::filesystem::path(Slic3r::data_dir()) / "log";
        //   if (!boost::filesystem::exists(log_folder)) create_directories(log_folder);
        //   auto base_path = (log_folder / buf.str()).make_preferred();
        //   return base_path.string();
        // } catch (const std::exception& e) { printf(...); }
        //
        // NOTE: C++ does NOT special-case an empty data_dir() here. When
        // data_dir() is empty, boost::filesystem::path("") / "log" yields the
        // relative path "log", and the result is "log/<buf>" — NOT bare <buf>.
        // We mirror that exactly: Path::new("").join("log") == "log".
        // LogSink.cpp:435 : auto log_folder = path(data_dir()) / "log";
        let log_folder = std::path::Path::new(&data_dir()).join("log");
        // LogSink.cpp:436-438 : if (!exists(log_folder)) create_directories(log_folder);
        if !log_folder.exists() {
            // boost::filesystem::create_directories — best effort; the C++
            // catches any exception (LogSink.cpp:441-443) and falls through to
            // `return buf.str()` below.
            if std::fs::create_dir_all(&log_folder).is_err() {
                // LogSink.cpp:442 : printf error, then fall through.
                // LogSink.cpp:445 : return buf.str();
                return buf;
            }
        }
        // LogSink.cpp:439 : auto base_path = (log_folder / buf.str()).make_preferred();
        // .make_preferred() normalizes path separators for the platform;
        // PathBuf already uses the native separator.
        let base_path = log_folder.join(&buf);
        // LogSink.cpp:440 : return base_path.string();
        base_path.to_string_lossy().into_owned()
    }
}


// ===========================================================================
// The file sink: LogSink.cpp of the override layer, over `TextFileBackend`.
// ===========================================================================

thread_local! {
    /// LogSink.cpp (override):30-31 — true while this thread is inside `LogSinkBackend::consume()`.
    static T_CONSUMING: std::cell::Cell<bool> = const { std::cell::Cell::new(false) };
}

/// LogSink.cpp (override):33-42 — marks the current thread as consuming for the lifetime of the
/// scope, also when it exits by a panic.
struct ConsumingScope;

impl ConsumingScope {
    fn new() -> ConsumingScope {
        T_CONSUMING.with(|t| t.set(true));
        ConsumingScope
    }
}

impl Drop for ConsumingScope {
    fn drop(&mut self) {
        T_CONSUMING.with(|t| t.set(false));
    }
}

/// LogSink.cpp (override):44-51
fn current_pid() -> u32 {
    std::process::id()
}

impl LogSinkBackend {
    /// LogSink.cpp (override):54-57 — whether this thread is writing a record to the log file right
    /// now. Anything this thread logs meanwhile is dropped, since logging from inside the sink would
    /// deadlock.
    pub fn is_consuming_on_this_thread() -> bool {
        T_CONSUMING.with(|t| t.get())
    }

    /// LogSink.cpp (override):59-66 `LogSinkBackend::LogSinkBackend`
    pub fn new(base_path: &str, options: &LogEncOptions) -> LogSinkBackend {
        let mut backend = LogSinkBackend { m_log_enc_options: options.clone(), ..Default::default() };
        backend.m_text_file_backend.set_file_name_pattern(base_path);
        backend.m_text_file_backend.set_rotation_size(100 * 1024 * 1024);
        // set_auto_newline_mode(disabled_auto_newline): newline need to be added before encryption;
        // `TextFileBackend` never adds one.
        // set_open_mode(std::ios::binary | std::ios::app)
        backend.m_text_file_backend.set_open_mode_app(true);
        backend
    }

    /// LogSink.cpp (override):77-98
    ///
    /// `m_log_mutex` is the mutex the caller holds the backend in.
    pub fn update_enc_option(&mut self, base_path: &str, options: &LogEncOptions) -> bool {
        if self.m_log_enc_options.enc_type == options.enc_type
            && self.m_log_enc_options.enc_key_host_env == options.enc_key_host_env
            && self.m_log_enc_options.enc_key_url == options.enc_key_url
        {
            return false;
        }

        self.m_log_enc_options = options.clone();
        self.reset_enc_key_info();

        self.m_text_file_backend.set_file_name_pattern(base_path);
        self.m_text_file_backend.rotate_file();

        true
    }

    /// LogSink.cpp (override):100-110
    ///
    /// `rec` is not read: the text file backend writes the formatted message only.
    pub fn consume(&mut self, _rec: &crate::log::LogRecord, formatted_message: &str) {
        let _consuming = ConsumingScope::new();

        if formatted_message.is_empty() {
            return;
        }

        // Headless: no encryption key source, so every record is written as plain text.
        let mut message = String::with_capacity(formatted_message.len() + S_MESSAGE_NEWLINE.len());
        message.push_str(formatted_message);
        message.push_str(S_MESSAGE_NEWLINE);
        if let Err(error) = self.m_text_file_backend.consume(&message) {
            // Boost.Log's core swallows what a sink throws unless an exception handler is set.
            let _ = error;
        }
    }

    /// `text_file_backend::flush`, reached through `LogSink::flush` (Log.cpp:186).
    pub fn flush(&mut self) {
        self.m_text_file_backend.flush();
    }
}

impl LogSinkUtil {
    /// LogSink.cpp (override):112-149
    ///
    /// rules: `studio_%a_%b_%d_%H_%M_%S_<pid>[_enc][_cn].log.%N`
    pub fn get_log_filename_format(log_dir: &str, enc_opts: &LogEncOptions) -> String {
        let now_time = chrono::Local::now();
        let mut buf = now_time.format("studio_%a_%b_%d_%H_%M_%S_").to_string();
        buf.push_str(&current_pid().to_string());
        if enc_opts.enc_type == LogEncType::LogEncNone {
            buf.push_str(".log.%N");
        } else {
            // default to us
            buf.push_str("_enc");
            if enc_opts.enc_key_host_env == "cn" {
                buf.push_str("_cn");
            } else if enc_opts.enc_key_host_env == "dc" {
                buf.push_str("_dc"); // dc means the user doesn't set the region
            } else {
                // no suffix for "us" region
            }

            buf.push_str(".log.%N");
        }

        let log_folder = std::path::PathBuf::from(log_dir);
        if !log_folder.exists() {
            if let Err(e) = std::fs::create_dir_all(&log_folder) {
                println!("get_log_filename_format, error: {}", e);
                return buf;
            }
        }
        let base_path = log_folder.join(&buf).to_string_lossy().into_owned();
        // make_preferred()
        #[cfg(windows)]
        let base_path = base_path.replace('/', "\\");
        base_path
    }
}

/// LogSink.hpp:79 `typedef boost::log::sinks::synchronous_sink<LogSinkBackend> LogSink` — the front
/// end: it serialises the calls into the backend, which the mutex does here.
#[derive(Debug)]
pub struct LogSink {
    m_backend: std::sync::Arc<Mutex<LogSinkBackend>>,
}

impl LogSink {
    /// `boost::make_shared<LogSink>(g_log_sink_backend)` (Log.cpp:144)
    pub fn new(backend: std::sync::Arc<Mutex<LogSinkBackend>>) -> LogSink {
        LogSink { m_backend: backend }
    }

    /// `synchronous_sink::consume`: the formatted record goes to the backend.
    pub fn consume(&self, rec: &crate::log::LogRecord, formatted_message: &str) {
        self.m_backend.lock().unwrap_or_else(|p| p.into_inner()).consume(rec, formatted_message);
    }

    /// `synchronous_sink::flush`
    pub fn flush(&self) {
        self.m_backend.lock().unwrap_or_else(|p| p.into_inner()).flush();
    }
}

/// The part of Boost.Log's `sinks::text_file_backend` that `LogSinkBackend` uses (see the module
/// notes); the members carry the names of `text_file_backend::implementation`.
#[derive(Debug)]
pub struct TextFileBackend {
    /// The directory of the file name pattern, made absolute.
    m_storage_dir: std::path::PathBuf,
    /// The file name of the pattern, without its counter placeholder.
    m_file_name_pattern: String,
    /// Where the counter goes in `m_file_name_pattern`, if the pattern has a counter placeholder.
    m_file_counter_position: Option<usize>,
    /// The width the counter is zero-padded to.
    m_file_counter_width: usize,
    m_file_counter: u32,
    /// `std::ios_base::app` of the open mode; without it a file is truncated when opened.
    m_file_open_mode_app: bool,
    m_file: Option<std::fs::File>,
    /// The stream state: false after a failed write.
    m_file_good: bool,
    m_file_name: std::path::PathBuf,
    m_characters_written: u64,
    m_file_rotation_size: u64,
    /// `m_file_counter` is set to the last used counter value.
    m_file_counter_is_last_used: bool,
    m_is_first_file: bool,
}

impl Default for TextFileBackend {
    /// `text_file_backend::implementation::implementation` with the defaults of
    /// `text_file_backend::construct`: no rotation by size, the file truncated when opened.
    fn default() -> Self {
        let mut backend = TextFileBackend {
            m_storage_dir: std::path::PathBuf::new(),
            m_file_name_pattern: String::new(),
            m_file_counter_position: None,
            m_file_counter_width: 0,
            m_file_counter: 0,
            m_file_open_mode_app: false,
            m_file: None,
            m_file_good: true,
            m_file_name: std::path::PathBuf::new(),
            m_characters_written: 0,
            m_file_rotation_size: u64::MAX,
            m_file_counter_is_last_used: false,
            m_is_first_file: true,
        };
        // `set_file_name_pattern_internal` with an empty pattern: "%5N.log".
        backend.set_file_name_pattern("");
        backend
    }
}

impl TextFileBackend {
    /// `parse_counter_placeholder` (text_file_backend.cpp): the format placeholder of the file
    /// counter, after its `%`: an optional filler, a width, an optional precision, then `N`.
    /// Gives the position after the placeholder and the width.
    fn parse_counter_placeholder(pattern: &[u8], mut it: usize) -> Option<(usize, usize)> {
        let end = pattern.len();
        let mut width = 0usize;
        if it == end {
            return None;
        }

        let mut c = pattern[it];
        if c == b'0' || c == b' ' || c == b'+' || c == b'-' {
            // Skip filler and alignment specification
            it += 1;
            if it == end {
                return None;
            }
            c = pattern[it];
        }

        if c.is_ascii_digit() {
            // Parse width
            let mut value = 0usize;
            while it != end && pattern[it].is_ascii_digit() {
                value = value.checked_mul(10)?.checked_add((pattern[it] - b'0') as usize)?;
                it += 1;
            }
            width = value;
            if it == end {
                return None;
            }
            c = pattern[it];
        }

        if c == b'.' {
            // Skip precision
            it += 1;
            while it != end && pattern[it].is_ascii_digit() {
                it += 1;
            }
            if it == end {
                return None;
            }
            c = pattern[it];
        }

        if c == b'N' {
            it += 1;
            return Some((it, width));
        }

        None
    }

    /// `text_file_backend::set_file_name_pattern` with `parse_file_name_pattern`: the storage
    /// directory, and the first counter placeholder of the file name.
    ///
    /// FIDELITY-NOTE: Boost.Log also formats date and time placeholders of the pattern when a file
    /// is opened. The patterns of this unit have none (`get_log_filename_format` has already
    /// written the time into the name), so any other `%` sequence stays in the name as it is.
    pub fn set_file_name_pattern(&mut self, pattern: &str) {
        let pattern = if !pattern.is_empty() { pattern } else { "%5N.log" };
        let path = std::path::Path::new(pattern);
        let mut name_pattern: Vec<u8> =
            path.file_name().map(|name| name.to_string_lossy().into_owned()).unwrap_or_default().into_bytes();
        let parent = path.parent().unwrap_or_else(|| std::path::Path::new(""));
        self.m_storage_dir = std::path::absolute(parent).unwrap_or_else(|_| parent.to_path_buf());

        // Let's try to find the file counter placeholder
        self.m_file_counter_position = None;
        self.m_file_counter_width = 0;
        let mut it = 0usize;
        loop {
            match name_pattern[it..].iter().position(|&c| c == b'%') {
                Some(offset) => it += offset,
                None => break,
            }
            let placeholder_begin = it;
            it += 1;
            if it == name_pattern.len() {
                break;
            }
            if name_pattern[it] == b'%' {
                // An escaped percent detected
                it += 1;
                if it == name_pattern.len() {
                    break;
                }
                continue;
            }

            if let Some((placeholder_end, width)) = Self::parse_counter_placeholder(&name_pattern, it) {
                // We've found the file counter placeholder in the pattern
                self.m_file_counter_position = Some(placeholder_begin);
                self.m_file_counter_width = width;
                name_pattern.drain(placeholder_begin..placeholder_end);
                break;
            }
            if it == name_pattern.len() {
                break;
            }
        }
        self.m_file_name_pattern = String::from_utf8_lossy(&name_pattern).into_owned();
    }

    /// `file_counter_formatter::operator()`: the file name of a counter value, the counter
    /// zero-padded to its width.
    fn file_name_generator(&self, counter: u32) -> String {
        let mut file_name = self.m_file_name_pattern.clone();
        if let Some(position) = self.m_file_counter_position {
            file_name.insert_str(position, &format!("{:0width$}", counter, width = self.m_file_counter_width));
        }
        file_name
    }

    /// `text_file_backend::set_rotation_size`
    pub fn set_rotation_size(&mut self, size: u64) {
        self.m_file_rotation_size = size;
    }

    /// `text_file_backend::set_open_mode`, for the one flag that matters here: `std::ios_base::app`.
    pub fn set_open_mode_app(&mut self, app: bool) {
        self.m_file_open_mode_app = app;
    }

    /// The name of the file being written, empty while none is open.
    pub fn get_current_file_name(&self) -> &std::path::Path {
        &self.m_file_name
    }

    /// `text_file_backend::consume`: rotate if the message would pass the rotation size, open the
    /// next file if none is open, write the message.
    pub fn consume(&mut self, formatted_message: &str) -> std::io::Result<()> {
        let message_size = formatted_message.len() as u64;
        let mut prev_file_name = std::path::PathBuf::new();
        let mut use_prev_file_name = false;
        if !self.m_file_good {
            // The file stream is not operational. One possible reason is that there is no more free space
            // on the file system. In this case it is possible that this log record will fail to be written as well,
            // leaving the newly created file empty. Eventually this results in lots of empty log files.
            // We should take precautions to avoid this.
            prev_file_name = self.m_file_name.clone();
            self.close_file();

            match std::fs::metadata(&prev_file_name) {
                Ok(metadata) if metadata.len() != 0 => {}
                _ => {
                    // To reuse the empty file avoid re-generating the new file name later
                    use_prev_file_name = true;
                }
            }
        } else if self.m_file.is_some()
            && self.m_characters_written.saturating_add(message_size) >= self.m_file_rotation_size
        {
            self.rotate_file();
        }

        let last_file_counter = self.m_file_counter.wrapping_sub(1);
        while self.m_file.is_none() {
            let new_file_name;
            if !use_prev_file_name {
                let mut file_counter = self.m_file_counter;
                if self.m_file_counter_is_last_used {
                    // If the sink backend is configured to append to a previously written file, don't
                    // increment the file counter and try to open the existing file.
                    let mut increment_file_counter = true;
                    if self.m_is_first_file && self.m_file_open_mode_app {
                        let last_file_name = self.m_storage_dir.join(self.file_name_generator(file_counter));
                        increment_file_counter = !last_file_name.is_file();
                    }

                    if increment_file_counter {
                        file_counter = file_counter.wrapping_add(1);
                        self.m_file_counter = file_counter;
                    }
                } else {
                    self.m_file_counter_is_last_used = true;
                }

                new_file_name = self.m_storage_dir.join(self.file_name_generator(file_counter));
            } else {
                new_file_name = std::mem::take(&mut prev_file_name);
                use_prev_file_name = false;
            }

            if let Some(parent) = new_file_name.parent() {
                std::fs::create_dir_all(parent)?;
            }

            let mut options = std::fs::OpenOptions::new();
            if self.m_file_open_mode_app {
                options.append(true).create(true);
            } else {
                options.write(true).create(true).truncate(true);
            }
            let file = options.open(&new_file_name)?;
            // Check the file size, as the file may have been written before.
            // Only do this check if we haven't exhausted the file counter to avoid looping indefinitely.
            self.m_characters_written = file.metadata().map(|metadata| metadata.len()).unwrap_or(0);
            self.m_file = Some(file);
            self.m_file_good = true;
            self.m_file_name = new_file_name;
            self.m_is_first_file = false;

            if self.m_characters_written > 0
                && self.m_characters_written.saturating_add(message_size) >= self.m_file_rotation_size
                && self.m_file_counter != last_file_counter
            {
                self.rotate_file();
                continue;
            }

            break;
        }

        use std::io::Write;
        let file = self.m_file.as_mut().expect("the loop above leaves a file open");
        // FIDELITY-NOTE: C++ writes through a buffered `std::ofstream` that static destruction
        // flushes at exit. A static is not dropped in Rust, so the record goes to the file at once
        // and nothing is lost if `flush_logs()` is never called; the file content is the same.
        if let Err(error) = file.write_all(formatted_message.as_bytes()) {
            self.m_file_good = false;
            return Err(error);
        }
        self.m_characters_written += message_size;
        Ok(())
    }

    /// `text_file_backend::flush`
    pub fn flush(&mut self) {
        use std::io::Write;
        if let Some(file) = self.m_file.as_mut() {
            let _ = file.flush();
        }
    }

    /// `text_file_backend::close_file`
    fn close_file(&mut self) {
        self.m_file = None;
        self.m_file_good = true;
        self.m_characters_written = 0;
        self.m_file_name = std::path::PathBuf::new();
    }

    /// `text_file_backend::rotate_file`: close the file; the next record opens the next one.
    pub fn rotate_file(&mut self) {
        self.close_file();
    }
}

// ===========================================================================
// Local mirrors of not-yet-ported Utils.cpp globals used by this file.
// ===========================================================================

// Utils.cpp:264-273 : static std::string g_data_dir; data_dir() returns it.
// Utils.cpp itself is not yet ported to Rust; mirror the global with the same
// semantics (empty until set_data_dir is called). This is the faithful C++
// behavior — an unset data_dir yields an empty string, and the file-name
// format then falls through to the bare `buf`.
use std::sync::Mutex;
static G_DATA_DIR: Mutex<String> = Mutex::new(String::new());

/// Utils.cpp:266-269 : void set_data_dir(const std::string &dir)
pub fn set_data_dir(dir: &str) {
    // Utils.cpp:268 : g_data_dir = dir;
    *G_DATA_DIR.lock().unwrap() = dir.to_string();
}

/// Utils.cpp:271-273 : const std::string& data_dir()
pub fn data_dir() -> String {
    // Utils.cpp:273 : return g_data_dir;
    G_DATA_DIR.lock().unwrap().clone()
}

// Utils.cpp:1179-1186 : unsigned get_current_pid()
fn get_current_pid() -> u32 {
    // #ifdef WIN32 GetCurrentProcessId(); #else ::getpid();
    std::process::id()
}

#[cfg(test)]
mod tests {
    use super::*;

    // LogSink.cpp:69-79
    #[test]
    fn test_s_get_enc_block_size() {
        assert_eq!(s_get_enc_block_size(LogEncType::LogEncAes256Cbc), 16);
        assert_eq!(s_get_enc_block_size(LogEncType::LogEncNone), 16);
    }

    // LogSink.cpp:243-255
    #[test]
    fn test_get_enc_key_type() {
        let backend = LogSinkBackend::default();
        assert_eq!(backend.get_enc_key_type(""), "");
        assert_eq!(backend.get_enc_key_type(DEFAULT_KEY_TAG_CN_1), "local");
        assert_eq!(backend.get_enc_key_type(DEFAULT_KEY_TAG_US_1), "local");
        assert_eq!(backend.get_enc_key_type("some-cloud-tag"), "cloud");
    }

    // LogSink.cpp:258-297 (default-key branch)
    #[test]
    fn test_get_aes_256_cbc_defaults() {
        let backend = LogSinkBackend::default();
        let (mut k, mut iv, mut tag) = (String::new(), String::new(), String::new());
        // `key_time` is passed BY VALUE in C++ (LogSink.cpp:262); the writes to
        // it (LogSink.cpp:269/274) are dead and never reach the caller, so the
        // caller's timestamp is left unchanged. We assert that here.
        let t = 0i64;

        let cn = LogEncOptions {
            enc_type: LogEncType::LogEncAes256Cbc,
            enc_key_url: String::new(),
            enc_key_host_env: "cn".to_string(),
        };
        backend.get_aes_256_cbc(&cn, &mut k, &mut iv, &mut tag, t);
        assert_eq!(k, DEFAULT_KEY_STR_CN_1);
        assert_eq!(iv, DEFAULT_KEY_IV_CN_1);
        assert_eq!(tag, DEFAULT_KEY_TAG_CN_1);
        // by-value: caller's `t` is unchanged
        assert_eq!(t, 0i64);

        let us = LogEncOptions {
            enc_type: LogEncType::LogEncAes256Cbc,
            enc_key_url: String::new(),
            enc_key_host_env: "us".to_string(),
        };
        backend.get_aes_256_cbc(&us, &mut k, &mut iv, &mut tag, t);
        assert_eq!(k, DEFAULT_KEY_STR_US_1);
        assert_eq!(iv, DEFAULT_KEY_IV_US_1);
        assert_eq!(tag, DEFAULT_KEY_TAG_US_1);
        // by-value: caller's `t` is unchanged
        assert_eq!(t, 0i64);
    }

    // LogSink.cpp:37-44
    #[test]
    fn test_reset_enc_key_info() {
        let mut backend = LogSinkBackend::default();
        backend.m_log_enc_key = "k".to_string();
        backend.m_log_enc_key_iv_base = "ivb".to_string();
        backend.m_log_enc_key_iv = "iv".to_string();
        backend.m_log_enc_key_tag = "tag".to_string();
        backend.m_log_enc_key_timestamp = 123;
        backend.reset_enc_key_info();
        assert!(backend.m_log_enc_key.is_empty());
        assert!(backend.m_log_enc_key_iv_base.is_empty());
        assert!(backend.m_log_enc_key_iv.is_empty());
        assert!(backend.m_log_enc_key_tag.is_empty());
        assert_eq!(backend.m_log_enc_key_timestamp, 0);
    }

    // LogSink.cpp:410-446
    #[test]
    fn test_get_log_filaname_format_suffixes() {
        // ENC_NONE -> ends with ".log.%N", no _enc
        let none = LogEncOptions {
            enc_type: LogEncType::LogEncNone,
            enc_key_url: String::new(),
            enc_key_host_env: String::new(),
        };
        let s = LogSinkUtil::get_log_filaname_format(&none);
        assert!(s.contains("studio_"));
        assert!(s.ends_with(".log.%N"));
        assert!(!s.contains("_enc"));

        // cn -> _enc_cn
        let cn = LogEncOptions {
            enc_type: LogEncType::LogEncAes256Cbc,
            enc_key_url: String::new(),
            enc_key_host_env: "cn".to_string(),
        };
        let s = LogSinkUtil::get_log_filaname_format(&cn);
        assert!(s.contains("_enc_cn"));
        assert!(s.ends_with(".log.%N"));

        // dc -> _enc_dc
        let dc = LogEncOptions {
            enc_type: LogEncType::LogEncAes256Cbc,
            enc_key_url: String::new(),
            enc_key_host_env: "dc".to_string(),
        };
        let s = LogSinkUtil::get_log_filaname_format(&dc);
        assert!(s.contains("_enc_dc"));

        // us -> _enc, no region suffix
        let us = LogEncOptions {
            enc_type: LogEncType::LogEncAes256Cbc,
            enc_key_url: String::new(),
            enc_key_host_env: "us".to_string(),
        };
        let s = LogSinkUtil::get_log_filaname_format(&us);
        assert!(s.contains("_enc"));
        assert!(!s.contains("_enc_cn"));
        assert!(!s.contains("_enc_dc"));
    }

    // text_file_backend.cpp `parse_file_name_pattern` / `file_counter_formatter`
    #[test]
    fn test_file_name_pattern_counter() {
        let mut backend = TextFileBackend::default();
        assert_eq!(backend.file_name_generator(7), "00007.log");
        backend.set_file_name_pattern("dir/studio_x.log.%N");
        assert_eq!(backend.file_name_generator(0), "studio_x.log.0");
        assert_eq!(backend.file_name_generator(12), "studio_x.log.12");
        backend.set_file_name_pattern("a_%3N_b");
        assert_eq!(backend.file_name_generator(4), "a_004_b");
        // no counter placeholder: every file has the same name
        backend.set_file_name_pattern("plain.log");
        assert_eq!(backend.file_name_generator(3), "plain.log");
        // an escaped percent is not a placeholder; `%Nx` after it is the counter
        backend.set_file_name_pattern("p%%q%N");
        assert_eq!(backend.file_name_generator(1), "p%%q1");
    }

    // text_file_backend.cpp `consume` / `rotate_file`: a file is closed when the next message
    // would reach the rotation size, and the next file takes the next counter.
    #[test]
    fn test_rotation_by_size() {
        let dir = std::env::temp_dir().join(format!("slicer_log_sink_rotation_{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        let mut backend = TextFileBackend::default();
        backend.set_file_name_pattern(dir.join("r.log.%N").to_str().unwrap());
        backend.set_rotation_size(10);
        backend.set_open_mode_app(true);
        for message in ["aaaa\n", "bbbb\n", "cccc\n", "dddddddddddd\n", "e\n"] {
            backend.consume(message).unwrap();
        }
        backend.flush();
        let read = |n: u32| std::fs::read_to_string(dir.join(format!("r.log.{n}"))).unwrap_or_default();
        // 5 + 5 reaches 10, so the second message opens the second file
        assert_eq!(read(0), "aaaa\n");
        assert_eq!(read(1), "bbbb\n");
        assert_eq!(read(2), "cccc\n");
        // a message larger than the limit still gets a file of its own
        assert_eq!(read(3), "dddddddddddd\n");
        assert_eq!(read(4), "e\n");
        assert_eq!(backend.get_current_file_name(), dir.join("r.log.4"));

        // A new backend in append mode writes on in the first file while it has room, then goes on
        // with the next counter.
        let mut again = TextFileBackend::default();
        again.set_file_name_pattern(dir.join("r.log.%N").to_str().unwrap());
        again.set_rotation_size(10);
        again.set_open_mode_app(true);
        again.consume("zz\n").unwrap();
        again.consume("yyyyyy\n").unwrap();
        assert_eq!(read(0), "aaaa\nzz\n");
        assert_eq!(read(1), "bbbb\n");
        let _ = std::fs::remove_dir_all(&dir);
    }

    // LogSink.cpp (override):77-98
    #[test]
    fn test_update_enc_option_changes() {
        let none = LogEncOptions { enc_type: LogEncType::LogEncNone, enc_key_url: String::new(), enc_key_host_env: String::new() };
        let mut backend = LogSinkBackend::new("unused/x.log.%N", &none);
        backend.m_log_enc_key = "k".to_string();
        assert!(!backend.update_enc_option("unused/x.log.%N", &none));
        assert_eq!(backend.m_log_enc_key, "k");
        let cn = LogEncOptions { enc_type: LogEncType::LogEncAes256Cbc, enc_key_url: String::new(), enc_key_host_env: "cn".to_string() };
        assert!(backend.update_enc_option("unused/x_enc_cn.log.%N", &cn));
        assert!(backend.m_log_enc_key.is_empty());
        assert_eq!(backend.m_log_enc_options, cn);
        assert!(!LogSinkBackend::is_consuming_on_this_thread());
    }
}
