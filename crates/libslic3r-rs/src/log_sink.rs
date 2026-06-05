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
//! The runtime sink machinery is built on three hard native dependencies that
//! the parity port forbids adding (none are wasm-safe and none touch the
//! G-code path):
//!   * boost.log `text_file_backend` (base class of `LogSinkBackend`, the
//!     `record_view` argument, file rotation, file-name patterns).
//!   * OpenSSL `AES_cbc_encrypt` / `AES_set_*_key` (`<openssl/aes.h>`) used by
//!     `consume` and `DecodeAES256LogFile` via `BBL_Encrypt::AES256CBC_*`.
//!   * libcurl through `Slic3r::Http` (cloud key fetch in `get_aes_256_cbc`).
//! Consequently the following members are NOT ported and are documented inline
//! at their original line refs:
//!   * `LogSinkBackend::LogSinkBackend` (ctor) — sets boost.log backend options.
//!   * `LogSinkBackend::consume` — encrypts and forwards to text_file_backend.
//!   * `LogSinkBackend::try_record_new_log_file` — boost.log + nlohmann::json header.
//!   * `LogSinkBackend::update_enc_option` — calls boost.log `rotate_file`.
//!   * `LogSinkBackend::update_log_enc_key` — depends on boost.log file state.
//!   * `LogSinkBackend::get_aes_256_cbc` cloud branch — `Slic3r::Http`.
//!   * `LogSinkBackend::DecodeAES256LogFile` — OpenSSL AES decrypt.
//!   * `s_generate_uuid` — boost::uuids.
//! Everything tractable WITHOUT those backends is ported faithfully below.

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
#[derive(Debug, Clone, Default)]
pub struct LogSinkBackend {
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
    pub fn get_aes_256_cbc(
        &self,
        enc_options: &LogEncOptions,
        key_str: &mut String,
        key_iv: &mut String,
        key_tag: &mut String,
        key_time: &mut i64,
    ) {
        // the default key
        // LogSink.cpp:265-275
        if enc_options.enc_key_host_env == "cn" {
            *key_tag = DEFAULT_KEY_TAG_CN_1.to_string(); // LogSink.cpp:266
            *key_str = DEFAULT_KEY_STR_CN_1.to_string(); // LogSink.cpp:267
            *key_iv = DEFAULT_KEY_IV_CN_1.to_string(); // LogSink.cpp:268
            *key_time = DEFAULT_KEY_TIME_CN_1; // LogSink.cpp:269
        } else {
            *key_tag = DEFAULT_KEY_TAG_US_1.to_string(); // LogSink.cpp:271
            *key_str = DEFAULT_KEY_STR_US_1.to_string(); // LogSink.cpp:272
            *key_iv = DEFAULT_KEY_IV_US_1.to_string(); // LogSink.cpp:273
            *key_time = DEFAULT_KEY_TIME_US_1; // LogSink.cpp:274
        }

        // LogSink.cpp:277-296 : cloud key fetch via Slic3r::Http (libcurl).
        // BLOCKED — native HTTP backend not available / not wasm-safe.
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
        let data_dir = data_dir();
        if !data_dir.is_empty() {
            let log_folder = std::path::Path::new(&data_dir).join("log");
            if !log_folder.exists() {
                // boost::filesystem::create_directories — best effort; the C++
                // catches any exception and falls through to buf below.
                if std::fs::create_dir_all(&log_folder).is_err() {
                    // LogSink.cpp:442 : printf error, then fall through.
                    return buf;
                }
            }
            let base_path = log_folder.join(&buf);
            // .make_preferred() normalizes path separators for the platform;
            // PathBuf already uses the native separator.
            return base_path.to_string_lossy().into_owned();
        }

        // LogSink.cpp:445 : return buf.str();
        buf
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
        let (mut k, mut iv, mut tag, mut t) =
            (String::new(), String::new(), String::new(), 0i64);

        let cn = LogEncOptions {
            enc_type: LogEncType::LogEncAes256Cbc,
            enc_key_url: String::new(),
            enc_key_host_env: "cn".to_string(),
        };
        backend.get_aes_256_cbc(&cn, &mut k, &mut iv, &mut tag, &mut t);
        assert_eq!(k, DEFAULT_KEY_STR_CN_1);
        assert_eq!(iv, DEFAULT_KEY_IV_CN_1);
        assert_eq!(tag, DEFAULT_KEY_TAG_CN_1);
        assert_eq!(t, DEFAULT_KEY_TIME_CN_1);

        let us = LogEncOptions {
            enc_type: LogEncType::LogEncAes256Cbc,
            enc_key_url: String::new(),
            enc_key_host_env: "us".to_string(),
        };
        backend.get_aes_256_cbc(&us, &mut k, &mut iv, &mut tag, &mut t);
        assert_eq!(k, DEFAULT_KEY_STR_US_1);
        assert_eq!(iv, DEFAULT_KEY_IV_US_1);
        assert_eq!(tag, DEFAULT_KEY_TAG_US_1);
        assert_eq!(t, DEFAULT_KEY_TIME_US_1);
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
}
