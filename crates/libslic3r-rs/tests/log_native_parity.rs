//! `log.rs` and the file sink of `log_sink.rs` against Bambu Studio's `Log.cpp` at the pin, with
//! the `LogSink.cpp` that `slicer_cli` compiles and Boost.Log behind them.
//!
//! `tests/data/log/vectors.txt` is written by `gen.cpp` of that directory (how to build it is in
//! its header, with the format). The test replays its steps: the level functions, `kv` for every
//! registered key and value type, the line of Boost.Log's default sink, and the log files that
//! `set_log_path_and_level` and `update_log_sink` produce: their names, and every line in them.
//! Time stamps, thread ids, the process id and the source location differ by nature and are
//! compared by their shape.

use slicer::log::log_detail::{log_enabled, LogStream, WriteValue};
use slicer::log::*;

const VECTORS: &str = include_str!("data/log/vectors.txt");

/// A string token: `~` and the text, with `%XX` for the bytes gen.cpp encodes.
fn decode(token: &str) -> String {
    let bytes = token.strip_prefix('~').expect("string token").as_bytes();
    let mut out = Vec::new();
    let mut i = 0;
    while i < bytes.len() {
        if bytes[i] == b'%' {
            out.push(u8::from_str_radix(std::str::from_utf8(&bytes[i + 1..i + 3]).expect("hex"), 16).expect("hex"));
            i += 3;
        } else {
            out.push(bytes[i]);
            i += 1;
        }
    }
    String::from_utf8(out).expect("utf-8 text")
}

/// A value of gen.cpp's `Value`, as the Rust type that stands for the C++ one.
enum Value {
    Int(i32),
    Unsigned(u32),
    LongLong(i64),
    Bool(bool),
    Double(f64),
    Float(f32),
    Char(char),
    SignedChar(i8),
    UnsignedChar(u8),
    String(String),
    CharPointer(Option<String>),
    Json(serde_json::Value),
}

impl Value {
    fn parse(token: &str) -> Value {
        let (kind, payload) = token.split_once('=').expect("value token");
        match kind {
            "i" => Value::Int(payload.parse().expect("int")),
            "u" => Value::Unsigned(payload.parse().expect("unsigned")),
            "l" => Value::LongLong(payload.parse().expect("long long")),
            "b" => Value::Bool(payload == "1"),
            "d" => Value::Double(f64::from_bits(u64::from_str_radix(payload, 16).expect("bits"))),
            "f" => Value::Float(f32::from_bits(u32::from_str_radix(payload, 16).expect("bits"))),
            "c" => Value::Char(payload.chars().next().expect("char")),
            "sc" => Value::SignedChar(payload.parse().expect("signed char")),
            "uc" => Value::UnsignedChar(payload.parse().expect("unsigned char")),
            "s" => Value::String(decode(payload)),
            "p" => Value::CharPointer(if payload == "null" { None } else { Some(decode(payload)) }),
            "j" => Value::Json(serde_json::from_str(&decode(payload)).expect("json")),
            other => panic!("value type {other}"),
        }
    }

    /// Call `f` with the value as its own type, as `std::visit` does in gen.cpp.
    fn visit<R>(&self, f: &mut dyn FnMut(&dyn WriteValue) -> R) -> R {
        match self {
            Value::Int(v) => f(v),
            Value::Unsigned(v) => f(v),
            Value::LongLong(v) => f(v),
            Value::Bool(v) => f(v),
            Value::Double(v) => f(v),
            Value::Float(v) => f(v),
            Value::Char(v) => f(v),
            Value::SignedChar(v) => f(v),
            Value::UnsignedChar(v) => f(v),
            Value::String(v) => f(v),
            Value::CharPointer(v) => f(&v.as_deref()),
            Value::Json(v) => f(v),
        }
    }
}

fn values(tokens: &mut std::str::SplitWhitespace) -> Vec<Value> {
    let n: usize = tokens.next().expect("count").parse().expect("count");
    (0..n).map(|_| Value::parse(tokens.next().expect("value"))).collect()
}

fn level_of(name: &str) -> LogLevel {
    match name {
        "trace" => LogLevel::Trace,
        "debug" => LogLevel::Debug,
        "info" => LogLevel::Info,
        "warning" => LogLevel::Warning,
        "error" => LogLevel::Error,
        "fatal" => LogLevel::Fatal,
        other => panic!("level {other}"),
    }
}

fn enc_options(tokens: &mut std::str::SplitWhitespace) -> LogEncOptions {
    let enc_type = match tokens.next().expect("enc type") {
        "0" => LogEncType::LogEncNone,
        _ => LogEncType::LogEncAes256Cbc,
    };
    let enc_key_url = decode(tokens.next().expect("url"));
    let enc_key_host_env = decode(tokens.next().expect("env"));
    LogEncOptions { enc_type, enc_key_url, enc_key_host_env }
}

/// `normalized` of gen.cpp: the digits of the time stamp become D, those of the thread id H, and
/// the location becomes [L]; a line that does not start a record stays as it is.
fn normalized(line: &str, file_format: bool) -> String {
    let mut line = line.as_bytes().to_vec();
    let time_begin = if file_format { 0 } else { 1 };
    let thread_begin = time_begin + 26 + if file_format { 1 } else { 2 };
    if line.len() < thread_begin + 3
        || &line[thread_begin..thread_begin + 3] != b"[0x"
        || line[time_begin + 4] != b'-'
        || line[time_begin + 10] != b' '
    {
        return String::from_utf8(line).expect("utf-8");
    }
    for c in line[time_begin..time_begin + 26].iter_mut() {
        if c.is_ascii_digit() {
            *c = b'D';
        }
    }
    let find = |line: &[u8], c: u8, from: usize| line[from..].iter().position(|&x| x == c).map(|p| p + from);
    let Some(thread_end) = find(&line, b']', thread_begin) else {
        return String::from_utf8(line).expect("utf-8");
    };
    for c in line[thread_begin + 3..thread_end].iter_mut() {
        *c = b'H';
    }
    let loc = find(&line, b']', thread_end + 1)
        .and_then(|level_end| find(&line, b'[', level_end))
        .and_then(|loc_begin| find(&line, b']', loc_begin).map(|loc_end| (loc_begin, loc_end)));
    if let Some((loc_begin, loc_end)) = loc {
        line.splice(loc_begin..=loc_end, b"[L]".iter().copied());
    }
    String::from_utf8(line).expect("utf-8")
}

/// `normalized_file_name` of gen.cpp: `studio_%a_%b_%d_%H_%M_%S_<pid>` with those seven fields as `*`.
fn normalized_file_name(name: &str) -> String {
    let rest = name.strip_prefix("studio_").expect("log file name");
    let mut fields = rest.splitn(7, '_');
    let mut out = String::from("studio");
    for _ in 0..6 {
        fields.next().expect("date and time fields");
        out += "_*";
    }
    let tail = fields.next().expect("process id");
    let pid_end = tail.find(|c: char| !c.is_ascii_digit()).unwrap_or(tail.len());
    assert!(pid_end > 0, "process id in {name}");
    assert_eq!(&tail[..pid_end], std::process::id().to_string());
    out + "_*" + &tail[pid_end..]
}

/// `dump_dir` of gen.cpp: the files of a directory by normalized name, each with its lines.
fn dump_dir(label: &str, dir: &std::path::Path, out: &mut Vec<String>) {
    let mut files: Vec<(String, std::path::PathBuf)> = Vec::new();
    if let Ok(entries) = std::fs::read_dir(dir) {
        for entry in entries {
            let path = entry.expect("entry").path();
            if path.is_file() {
                files.push((normalized_file_name(path.file_name().expect("name").to_str().expect("utf-8")), path));
            }
        }
    }
    files.sort();
    for (name, path) in files {
        let text = std::fs::read(&path).expect("log file");
        // std::getline: the lines between '\n', without a last empty one
        let mut lines: Vec<&[u8]> = text.split(|&c| c == b'\n').collect();
        if lines.last().is_some_and(|line| line.is_empty()) {
            lines.pop();
        }
        out.push(format!("FILE {label} {} {}", tok(&name), lines.len()));
        for line in lines {
            out.push(format!("LINE {}", tok(&normalized(std::str::from_utf8(line).expect("utf-8"), true))));
        }
    }
}

/// `tok` of gen.cpp.
fn tok(s: &str) -> String {
    let mut out = String::from("~");
    for &c in s.as_bytes() {
        if c <= b' ' || c == b'%' || c >= 0x7f {
            out += &format!("%{:02X}", c);
        } else {
            out.push(c as char);
        }
    }
    out
}

fn set_level_text(n: u32) -> String {
    set_logging_level(n);
    let mut out = get_logging_level().to_string();
    for level in [LogLevel::Trace, LogLevel::Debug, LogLevel::Info, LogLevel::Warning, LogLevel::Error, LogLevel::Fatal] {
        out += if log_enabled(level) { " 1" } else { " 0" };
    }
    out
}

/// `record` of gen.cpp: one record, as `boost_log_trivial!` makes it.
fn record(level: LogLevel, values: &[Value]) {
    if !log_enabled(level) {
        return;
    }
    let mut stream = LogStream::new(level, file!(), line!(), "record");
    for value in values {
        value.visit(&mut |v| {
            stream.write(v);
        });
    }
}

#[test]
fn log_matches_cpp() {
    let scratch = tempfile::tempdir().expect("temp dir");
    let dir_of = |label: &str| match label {
        "a" => scratch.path().join("a"),
        _ => scratch.path().join("b/sub/dir"),
    };

    let mut counts = std::collections::BTreeMap::<&str, usize>::new();
    let mut expected_files: Vec<String> = Vec::new();
    for line in VECTORS.lines() {
        let (step, expected) = match line.split_once(" -> ") {
            Some((step, expected)) => (step, Some(expected)),
            None => (line, None),
        };
        let mut t = step.split_whitespace();
        let tag = t.next().expect("tag");
        *counts.entry(tag).or_insert(0) += 1;
        let got: Option<String> = match tag {
            "NAME" => Some(level_string_to_boost(&decode(t.next().expect("name"))).to_string()),
            "STR" => Some(tok(&get_string_logging_level(t.next().expect("n").parse().expect("n")))),
            "VALID" => Some((is_log_trivial_valid() as i32).to_string()),
            "GETLVL" => Some(get_logging_level().to_string()),
            "SETLVL" => Some(set_level_text(t.next().expect("n").parse().expect("n"))),
            "KV" => {
                let key = decode(t.next().expect("key"));
                let value = Value::parse(t.next().expect("value"));
                Some(tok(&value.visit(&mut |v| kv(&key, v))))
            }
            "EXC" => {
                let caught = match t.next().expect("kind") {
                    "none" => None,
                    "std" => Some(std::panic::catch_unwind(|| std::panic::panic_any("what it says".to_string())).unwrap_err()),
                    _ => Some(std::panic::catch_unwind(|| std::panic::panic_any(7_i32)).unwrap_err()),
                };
                Some(tok(&current_exception_message(caught.as_deref())))
            }
            "DEFREC" => {
                let level = level_of(t.next().expect("level"));
                let values = values(&mut t);
                if log_enabled(level) {
                    // The message as LogStream makes it (the records of the files below go through
                    // the stream itself): the location, the values, no line end at the end.
                    let mut message = String::from("[L] ");
                    for value in values.iter() {
                        value.visit(&mut |v| v.write_value(&mut message));
                    }
                    while message.ends_with('\n') || message.ends_with('\r') {
                        message.pop();
                    }
                    Some(tok(&normalized(&default_sink_format(&LogRecord::new(level, message)), false)))
                } else {
                    Some("-".to_string())
                }
            }
            "SETPATH" => {
                let dir = dir_of(t.next().expect("dir"));
                let level: u32 = t.next().expect("level").parse().expect("level");
                set_log_path_and_level(dir.to_str().expect("utf-8 path"), level, &enc_options(&mut t));
                None
            }
            "UPDATE" => {
                update_log_sink(&enc_options(&mut t));
                None
            }
            "REC" => {
                let level = level_of(t.next().expect("level"));
                record(level, &values(&mut t));
                None
            }
            "FILE" | "LINE" => {
                expected_files.push(line.to_string());
                continue;
            }
            other => panic!("tag {other}"),
        };
        assert!(t.next().is_none(), "tokens left in: {step}");
        assert_eq!(got.as_deref(), expected, "{step}");
    }

    flush_logs();
    let mut files: Vec<String> = Vec::new();
    dump_dir("a", &dir_of("a"), &mut files);
    dump_dir("b", &dir_of("b"), &mut files);
    for (i, (got, expected)) in files.iter().zip(expected_files.iter()).enumerate() {
        assert_eq!(got, expected, "line {i} of the log files");
    }
    assert_eq!(files.len(), expected_files.len());

    assert_eq!(counts["KV"], 702);
    assert_eq!(counts["DEFREC"], 40);
    assert_eq!(counts["REC"], 370);
    assert_eq!(counts["FILE"], 6);
    assert_eq!(counts["LINE"], 302);

    // The macro: the record carries the location of the statement.
    set_logging_level(3);
    let here = line!() + 1;
    slicer::boost_log_trivial!(Warning, kv(slicer::log_keys::log_key::PLATE, &3), "through the macro ", 1.5, ' ', true);
    slicer::boost_log_trivial!(Debug, "filtered out");
    flush_logs();
    let mut newest: Vec<String> = Vec::new();
    for entry in std::fs::read_dir(dir_of("b")).expect("dir") {
        let text = std::fs::read_to_string(entry.expect("entry").path()).expect("log file");
        newest.push(text.lines().last().expect("a line").to_string());
    }
    assert_eq!(newest.len(), 1);
    let expected_tail = format!("[warning] [log_native_parity.rs:{here} log_matches_cpp] plate=3 through the macro 1.5 true");
    assert!(newest[0].ends_with(&expected_tail), "{} / {}", newest[0], expected_tail);
    println!("log: {counts:?} equal to the C++");
}
