//! `slicer::print_base` (the port of libslic3r/PrintBase.hpp / PrintBase.cpp) against the C++ at the
//! pin.
//!
//! `tests/data/print_base/vectors.txt.gz` is the output of `gen.cpp` beside it (gzip -9n): random
//! operation sequences on a PrintState, on a PrintBaseWithState with a PrintObjectBaseWithState of
//! it (status callback records, cancel callback calls, timestamps relative to the start of the
//! sequence), and the output file name functions over small models. The header of gen.cpp
//! describes the lines. Every sequence is replayed here and has to give the same tokens.

use slicer::config::{ConfigOptionInt, ConfigOptionString, ConfigOptionStrings, DynamicConfig};
use slicer::print_base::*;
use std::io::Read;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::{Arc, Mutex};

fn read_vectors() -> String {
    let path = format!("{}/tests/data/print_base/vectors.txt.gz", env!("CARGO_MANIFEST_DIR"));
    let file = std::fs::File::open(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
    let mut text = String::new();
    flate2::read::GzDecoder::new(file).read_to_string(&mut text).expect("gzip");
    text
}

fn hex(s: &[u8]) -> String {
    if s.is_empty() {
        return "-".to_string();
    }
    s.iter().map(|c| format!("{:02x}", c)).collect()
}
fn unhex(t: &str) -> String {
    if t == "-" {
        return String::new();
    }
    let b: Vec<u8> = (0..t.len() / 2).map(|i| u8::from_str_radix(&t[2 * i..2 * i + 2], 16).expect("hex")).collect();
    String::from_utf8(b).expect("UTF-8")
}

struct Toks<'a> {
    t: Vec<&'a str>,
    i: usize,
}

impl<'a> Toks<'a> {
    fn new(s: &'a str) -> Self {
        Toks { t: s.split(' ').filter(|x| !x.is_empty()).collect(), i: 0 }
    }
    fn more(&self) -> bool {
        self.i < self.t.len()
    }
    fn s(&mut self) -> &'a str {
        let v = self.t[self.i];
        self.i += 1;
        v
    }
    fn i(&mut self) -> i64 {
        self.s().parse().expect("integer")
    }
    fn u(&mut self) -> usize {
        self.i() as usize
    }
    fn b(&mut self) -> bool {
        self.i() != 0
    }
    fn d(&mut self) -> f64 {
        f64::from_bits(u64::from_str_radix(self.s(), 16).expect("double"))
    }
    fn text(&mut self) -> String {
        unhex(self.s())
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum TestStep {
    A,
    B,
    C,
    D,
}

impl StepType for TestStep {
    fn index(self) -> usize {
        self as usize
    }
    fn from_index(idx: usize) -> Self {
        [TestStep::A, TestStep::B, TestStep::C, TestStep::D][idx]
    }
}

const MESSAGES: [&str; 5] = ["first warning", "second", "", "third one", "first warning"];

fn level(v: i64) -> WarningLevel {
    if v == 0 {
        WarningLevel::NonCritical
    } else {
        WarningLevel::Critical
    }
}

fn notification(v: i64) -> SlicingNotificationType {
    use SlicingNotificationType::*;
    [
        SlicingDefaultNotification,
        SlicingReplaceInitEmptyLayers,
        SlicingNeedSupportOn,
        SlicingEmptyGcodeLayers,
        SlicingGcodeOverlap,
        SlicingSupportIncomplete,
        SlicingSupportIncompleteOnBuildPlate,
    ][v as usize]
}

fn state_index(s: State) -> i64 {
    match s {
        State::Invalid => 0,
        State::Started => 1,
        State::Done => 2,
    }
}

struct Out {
    v: Vec<String>,
    base: usize,
}

impl Out {
    fn s(&mut self, x: impl Into<String>) {
        self.v.push(x.into());
    }
    fn i(&mut self, x: i64) {
        self.v.push(x.to_string());
    }
    fn text(&mut self, x: &str) {
        self.v.push(hex(x.as_bytes()));
    }
    fn stamp(&mut self, t: TimeStamp) {
        if t == 0 {
            self.s("z");
        } else {
            self.i((t - self.base) as i64);
        }
    }
    fn state(&mut self, st: &StateWithTimeStamp) {
        self.i(state_index(st.state));
        self.stamp(st.timestamp);
    }
    fn warnings(&mut self, st: &StateWithWarnings) {
        self.i(state_index(st.state));
        self.stamp(st.timestamp);
        self.i(st.warnings.len() as i64);
        for w in &st.warnings {
            self.i(if w.level == WarningLevel::NonCritical { 0 } else { 1 });
            self.i(w.current as i64);
            self.text(&w.message);
            self.i(w.message_id as i64);
        }
    }
}

fn run_print_state(inp: &str) -> String {
    let mut t = Toks::new(inp);
    t.s();
    let mut ps: PrintState<TestStep, 4> = PrintState::new();
    let mtx = Mutex::new(());
    let mut o = Out { v: Vec::new(), base: g_last_timestamp() };
    while t.more() {
        match t.s() {
            "S" => {
                let step = TestStep::from_index(t.u());
                let cancel = t.b();
                match ps.set_started(step, &mtx, || if cancel { Err(CanceledException) } else { Ok(()) }) {
                    Ok(r) => o.i(r as i64),
                    Err(_) => o.s("!C"),
                }
            }
            "D" => {
                let step = TestStep::from_index(t.u());
                let cancel = t.b();
                match ps.set_done(step, &mtx, || if cancel { Err(CanceledException) } else { Ok(()) }) {
                    Ok((ts, ui)) => {
                        o.stamp(ts);
                        o.i(ui as i64);
                    }
                    Err(_) => o.s("!C"),
                }
            }
            "I" => {
                let step = TestStep::from_index(t.u());
                let mut calls = 0;
                let r = ps.invalidate(step, || calls += 1);
                o.i(r as i64);
                o.i(calls);
            }
            "M" => {
                let n = t.u();
                let steps: Vec<TestStep> = (0..n).map(|_| TestStep::from_index(t.u())).collect();
                let mut calls = 0;
                let r = ps.invalidate_multiple(steps.clone(), || calls += 1);
                o.i(r as i64);
                o.i(calls);
            }
            "A" => {
                let mut calls = 0;
                let r = ps.invalidate_all(|| calls += 1);
                o.i(r as i64);
                o.i(calls);
            }
            "W" => {
                let lv = level(t.i());
                let mi = t.u();
                let id = t.i() as i32;
                let (step, ui) = ps.active_step_add_warning(lv, MESSAGES[mi], id, &mtx);
                o.i(step.index() as i64);
                o.i(ui as i64);
            }
            "T" => {
                let step = TestStep::from_index(t.u());
                let a = ps.state_with_timestamp(step, &mtx);
                o.state(&a);
                let b = ps.state_with_timestamp_unguarded(step);
                o.state(&b);
            }
            "X" => {
                let step = TestStep::from_index(t.u());
                let w = ps.state_with_warnings(step, &mtx);
                o.warnings(&w);
            }
            "Q" => {
                let step = TestStep::from_index(t.u());
                o.i(ps.is_started(step, &mtx) as i64);
                o.i(ps.is_done(step, &mtx) as i64);
                o.i(ps.is_started_unguarded(step) as i64);
                o.i(ps.is_done_unguarded(step) as i64);
            }
            k => panic!("print_state op {k}"),
        }
    }
    o.v.join(" ")
}

type TestPrint = PrintBaseWithState<TestStep, 4>;
type TestObject = PrintObjectBaseWithState<TestPrint, TestStep, 4>;

fn run_print_with_state(inp: &str) -> String {
    let mut t = Toks::new(inp);
    t.s();
    let with_cb = t.b();
    let statuses: Arc<Mutex<Vec<SlicingStatus>>> = Arc::new(Mutex::new(Vec::new()));
    let mut print: Box<TestPrint> = Box::new(TestPrint::new());
    let mut obj = TestObject::new(&*print as *const TestPrint, None);
    if with_cb {
        let st = statuses.clone();
        print.base.set_status_callback(Arc::new(move |s: &SlicingStatus| st.lock().unwrap().push(s.clone())));
    }
    let cancel_calls = Arc::new(AtomicUsize::new(0));
    {
        let cc = cancel_calls.clone();
        print.base.set_cancel_callback(Arc::new(move || {
            cc.fetch_add(1, Ordering::SeqCst);
        }));
    }
    let mut o = Out { v: Vec::new(), base: g_last_timestamp() };
    while t.more() {
        match t.s() {
            "S" => {
                let on_obj = t.b();
                let step = TestStep::from_index(t.u());
                let r = if on_obj { obj.set_started(step) } else { print.set_started(step) };
                match r {
                    Ok(r) => o.i(r as i64),
                    Err(_) => o.s("!C"),
                }
            }
            "D" => {
                let on_obj = t.b();
                let step = TestStep::from_index(t.u());
                let r = if on_obj { obj.set_done(step) } else { print.set_done(step) };
                match r {
                    Ok(ts) => o.stamp(ts),
                    Err(_) => o.s("!C"),
                }
            }
            "I" => {
                let on_obj = t.b();
                let step = TestStep::from_index(t.u());
                let r = if on_obj { obj.invalidate_step(step) } else { print.invalidate_step(step) };
                o.i(r as i64);
            }
            "M" => {
                let on_obj = t.b();
                let n = t.u();
                let steps: Vec<TestStep> = (0..n).map(|_| TestStep::from_index(t.u())).collect();
                let r = if on_obj { obj.invalidate_steps(steps.clone()) } else { print.invalidate_steps(steps.clone()) };
                o.i(r as i64);
            }
            "A" => {
                let on_obj = t.b();
                let which = t.b();
                let r = if on_obj {
                    if which {
                        obj.invalidate_all_steps_without_cancel()
                    } else {
                        obj.invalidate_all_steps()
                    }
                } else {
                    print.invalidate_all_steps()
                };
                o.i(r as i64);
            }
            "W" => {
                let on_obj = t.b();
                let lv = level(t.i());
                let mi = t.u();
                let id = notification(t.i());
                if on_obj {
                    obj.active_step_add_warning(lv, MESSAGES[mi], id);
                } else {
                    print.active_step_add_warning(lv, MESSAGES[mi], id);
                }
            }
            "T" => {
                let on_obj = t.b();
                let step = TestStep::from_index(t.u());
                let a = if on_obj { obj.step_state_with_timestamp(step) } else { print.step_state_with_timestamp(step) };
                o.state(&a);
                let w = if on_obj { obj.step_state_with_warnings(step) } else { print.step_state_with_warnings(step) };
                o.warnings(&w);
            }
            "Q" => {
                let on_obj = t.b();
                let step = TestStep::from_index(t.u());
                if on_obj {
                    o.i(obj.is_step_done(step) as i64);
                    o.i(obj.is_step_started_unguarded(step) as i64);
                    o.i(obj.is_step_done_unguarded(step) as i64);
                } else {
                    o.i(print.is_step_done(step) as i64);
                    o.i(print.is_step_started_unguarded(step) as i64);
                    o.i(print.is_step_done_unguarded(step) as i64);
                }
            }
            "CU" => print.base.cancel(),
            "CI" => print.base.cancel_internal(),
            "R" => print.base.restart(),
            "CS" => {
                let cs = match print.base.cancel_status() {
                    CancelStatus::NotCanceled => 0,
                    CancelStatus::CanceledByUser => 1,
                    CancelStatus::CanceledInternal => 2,
                };
                o.i(cs);
                o.i(print.base.canceled() as i64);
                // the C++ draws on_obj for this op too, and throws the same either way
                match print.base.throw_if_canceled() {
                    Ok(()) => o.i(0),
                    Err(_) => o.s("!C"),
                }
                match obj.throw_if_canceled().and(print.base.make_try_cancel().call()) {
                    Ok(()) => o.i(0),
                    Err(_) => o.s("!C"),
                }
            }
            "ST" => {
                let percent = t.i() as i32;
                let mi = t.u();
                let flags = t.i() as u32;
                let wstep = t.i() as i32;
                print.base.set_status(percent, MESSAGES[mi], flags, wstep);
            }
            "PI" => {
                let idx = t.i() as i32;
                print.base.set_plate_index(idx);
                o.i(print.base.get_plate_index() as i64);
            }
            "NC" => {
                let nc = t.b();
                print.base.set_no_check_flag(nc);
                o.i(print.base.get_no_check_flag() as i64);
            }
            "PN" => {
                let mi = t.u();
                print.base.set_plate_name(MESSAGES[mi]);
                o.text(&print.base.get_plate_name());
            }
            "CC" => print.base.call_cancel_callback(),
            "UW" => {
                let on_obj = t.b();
                let lv = level(t.i());
                let mi = t.u();
                let wstep = t.i() as i32;
                let id = notification(t.i());
                if on_obj {
                    obj.base.status_update_warnings(&print.base, wstep, lv, MESSAGES[mi], id);
                } else {
                    print.base.status_update_warnings(wstep, lv, MESSAGES[mi], None, id);
                }
            }
            "SD" => {
                let silent = t.b();
                if silent {
                    print.base.set_status_silent();
                } else {
                    print.base.set_status_default();
                }
            }
            k => panic!("print_with_state op {k}"),
        }
        o.i(cancel_calls.load(Ordering::SeqCst) as i64);
        let mut st = statuses.lock().unwrap();
        o.i(st.len() as i64);
        for s in st.iter() {
            o.i(s.percent as i64);
            o.text(&s.text);
            o.i(s.flags as i64);
            o.s(if s.warning_object_id == print.base.id() {
                "P"
            } else if s.warning_object_id == obj.base.id() {
                "O"
            } else {
                "-"
            });
            o.i(s.warning_step as i64);
            o.i(s.message_type as i64);
            o.i(if s.warning_level == WarningLevel::NonCritical { 0 } else { 1 });
        }
        st.clear();
    }
    o.v.join(" ")
}

fn run_output_filename(inp: &str) -> String {
    let mut t = Toks::new(inp);
    t.s();
    let mut model = PrintBaseModel::default();
    let nobj = t.u();
    for _ in 0..nobj {
        let mut mo = PrintBaseModelObject::default();
        mo.name = t.text();
        mo.input_file = t.text();
        let ninst = t.u();
        for _ in 0..ninst {
            let is_printable = t.b();
            let scaling_factor = [t.d(), t.d(), t.d()];
            mo.instances.push(PrintBaseModelInstance { is_printable, scaling_factor });
        }
        model.objects.push(mo);
    }
    model.proposed_export_file_name_and_path = t.text();
    let format = t.text();
    let ext = t.text();
    let base = t.text();
    let ovr = t.b();
    let mut print = PrintBase::new();
    print.m_model = model;
    let mut override_cfg = DynamicConfig::new();
    override_cfg.set_key_value("input_filename_base", Box::new(ConfigOptionString::with_value("ovr")));
    override_cfg.set_key_value("num_objects", Box::new(ConfigOptionInt::with_value(77)));

    let mut o = Out { v: Vec::new(), base: 0 };
    let mut cfg = DynamicConfig::new();
    print.update_object_placeholders(&mut cfg, &ext);
    o.i(cfg.opt_int("num_objects") as i64);
    o.i(cfg.opt_int("num_instances") as i64);
    {
        use slicer::config::ConfigOptionResolver;
        let scale = cfg.option("scale").unwrap().as_any().downcast_ref::<ConfigOptionStrings>().unwrap();
        o.i(scale.values.len() as i64);
        for s in &scale.values {
            o.text(s);
        }
        for k in ["input_filename", "input_filename_base"] {
            match cfg.option(k).and_then(|x| x.as_any().downcast_ref::<ConfigOptionString>()) {
                Some(s) => o.text(&s.value),
                None => o.s("none"),
            }
        }
    }
    match print.output_filename(&format, &ext, &base, if ovr { Some(&override_cfg) } else { None }) {
        Ok(s) => o.text(&s),
        Err(e) => {
            o.s("!P");
            o.text(&e.message);
        }
    }
    let virtual_output_filename =
        |b: &str| print.output_filename("{input_filename_base}_{num_objects}.gcode", ".gcode", b, None);
    for path in ["", "/", "/nonexistent-dir/file.gcode"] {
        match print.output_filepath(path, &base, &virtual_output_filename) {
            Ok(s) => o.text(&s),
            Err(e) => {
                o.s("!P");
                o.text(&e.message);
            }
        }
    }
    o.v.join(" ")
}

#[test]
fn print_base_matches_the_pin() {
    // One test function: the sequences read the process-wide g_last_timestamp.
    let text = read_vectors();
    let mut counts: std::collections::BTreeMap<String, (usize, usize)> = std::collections::BTreeMap::new();
    let mut failures: Vec<String> = Vec::new();
    for line in text.lines() {
        let (inp, want) = line.split_once(" => ").unwrap_or((line, ""));
        let kind = inp.split(' ').next().unwrap().to_string();
        let got = std::panic::catch_unwind(|| match kind.as_str() {
            "print_state" => run_print_state(inp),
            "print_with_state" => run_print_with_state(inp),
            "output_filename" => run_output_filename(inp),
            k => panic!("kind {k}"),
        });
        let e = counts.entry(kind.clone()).or_insert((0, 0));
        e.0 += 1;
        let ok = matches!(&got, Ok(g) if g == want);
        if !ok {
            e.1 += 1;
            if failures.len() < 10 {
                failures.push(format!(
                    "{}\n  got:  {}\n  want: {}",
                    &inp[..inp.len().min(300)],
                    got.unwrap_or_else(|_| "PANIC".to_string()),
                    want
                ));
            }
        }
    }
    let mut report = String::new();
    let mut failed = 0;
    for (k, (n, bad)) in &counts {
        report.push_str(&format!("{k}: {n} cases, {bad} differ\n"));
        failed += bad;
    }
    let root = env!("CARGO_MANIFEST_DIR").trim_end_matches("/crates/libslic3r-rs").to_string();
    std::fs::write(format!("{root}/target/print_base_native_parity.txt"), format!("{report}\n{}", failures.join("\n\n"))).ok();
    assert!(failed == 0, "{report}\n{}", failures.join("\n\n"));
}
