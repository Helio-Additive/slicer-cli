//! `perf_trace` against PerfTrace.cpp at the pin.
//!
//! `tests/data/perf_trace/vectors.txt` is the output of `gen.cpp` beside it, which compiles
//! PerfTrace.cpp itself with a Log.hpp that prints every record and every `flush_logs()` (the
//! header comment there says how). The test runs the same steps with the log going to a file and
//! compares, step by step, the level and the message of every record the step added. The number
//! of milliseconds in a "cost" line is compared as a number, not by its value. Where the C++
//! flushed the log after the last record of a step, the file is read without flushing it here;
//! the file sink of the port writes a record at once, though, so that the destructor calls
//! `flush_logs()` is read off the C++ (PerfTrace.cpp:23), not observed.

use slicer::log::{flush_logs, set_log_path_and_level, LogEncOptions, LogEncType};
use slicer::perf_trace::{perf_mark, PerfTimer};
use slicer::{perf_trace, perf_trace_if};

const VECTORS: &str = include_str!("data/perf_trace/vectors.txt");

/// `LOG <level> <message>` of every record in the log files of `dir`.
fn records(dir: &std::path::Path) -> Vec<String> {
    let mut files: Vec<std::path::PathBuf> = std::fs::read_dir(dir).expect("log dir").map(|e| e.expect("entry").path()).collect();
    files.sort();
    let mut out = Vec::new();
    for file in files {
        let text = std::fs::read_to_string(file).expect("log file");
        for line in text.split('\n') {
            if line.is_empty() {
                continue;
            }
            // time [thread] [level  ] [file:line function] message
            let rest = &line[line.find("] ").expect("the thread id") + 2..];
            let level = rest[1..rest.find(']').expect("the level")].trim();
            let rest = &rest[rest.find("] ").expect("the level") + 2..];
            let message = &rest[rest.find("] ").expect("the location") + 2..];
            out.push(format!("LOG {level} {message}"));
        }
    }
    out
}

/// A "cost" line with its number taken out, and the number.
fn without_cost(line: &str) -> (String, Option<i64>) {
    if let Some(at) = line.find(" cost ") {
        if let Some(number) = line[at + 6..].strip_suffix(" ms") {
            if let Ok(ms) = number.parse::<i64>() {
                return (format!("{} cost N ms", &line[..at]), Some(ms));
            }
        }
    }
    (line.to_string(), None)
}

fn step(name: &str) {
    match name {
        "mark" => perf_mark("startup"),
        "mark_empty" => perf_mark(""),
        "mark_text" => perf_mark("load project: a b [c] 100%"),
        "timer" => {
            let _timer = PerfTimer::new("load", true);
        }
        "timer_disabled" => {
            let _timer = PerfTimer::new("quiet", false);
        }
        "timer_enabled" => {
            let _timer = PerfTimer::new("loud", true);
        }
        "trace" => {
            perf_trace!("scope");
            perf_mark("inside the scope");
        }
        "trace_if_true" => {
            perf_trace_if!(true, String::from("3mf: ") + "add model");
            perf_mark("inside");
        }
        "trace_if_false" => {
            perf_trace_if!(false, "3mf: add thumbnails");
            perf_mark("inside");
        }
        "nested" => {
            perf_trace!("outer");
            {
                perf_trace!("inner");
            }
            perf_mark("between");
        }
        "sleep" => {
            perf_trace!("sleep");
            std::thread::sleep(std::time::Duration::from_millis(30));
        }
        other => panic!("unknown step {other}"),
    }
}

#[test]
fn perf_trace_logs_what_the_cpp_logs() {
    let dir = std::env::temp_dir().join(format!("perf_trace_{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    let options = LogEncOptions { enc_type: LogEncType::LogEncNone, enc_key_url: String::new(), enc_key_host_env: String::new() };
    set_log_path_and_level(dir.to_str().expect("utf-8 path"), 3, &options);

    // The steps and what each logged in C++.
    let mut steps: Vec<(&str, Vec<&str>)> = Vec::new();
    for line in VECTORS.lines() {
        match line.strip_prefix("STEP ") {
            Some(name) => steps.push((name, Vec::new())),
            None => steps.last_mut().expect("a step").1.push(line),
        }
    }
    assert_eq!(steps.len(), 11);

    let mut seen = 0;
    let mut compared = 0;
    for (name, want) in steps {
        step(name);
        let flushed = want.last() == Some(&"FLUSH");
        if !flushed {
            flush_logs();
        }
        let all = records(&dir);
        let got = &all[seen..];
        seen = all.len();
        let want: Vec<&str> = want.into_iter().filter(|line| *line != "FLUSH").collect();
        assert_eq!(got.len(), want.len(), "step {name}: {got:?}");
        for (got, want) in got.iter().zip(want) {
            let (got_text, got_ms) = without_cost(got);
            let (want_text, want_ms) = without_cost(want);
            assert_eq!(got_text, want_text, "step {name}");
            assert_eq!(got_ms.is_some(), want_ms.is_some(), "step {name}");
            if name == "sleep" {
                if let Some(ms) = got_ms {
                    assert!(ms >= 30, "step sleep cost {ms} ms");
                }
            }
            compared += 1;
        }
    }
    assert_eq!(compared, 21);
    let _ = std::fs::remove_dir_all(&dir);
}
