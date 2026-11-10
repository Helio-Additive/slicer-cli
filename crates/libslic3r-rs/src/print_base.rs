//! Faithful 1:1 port of `PrintBase.hpp` / `PrintBase.cpp` from BambuStudio's libslic3r.
//!
//! PrintBase is the abstract base for the print orchestration pipeline
//! (slice -> convert to instructions -> send to printer). This module ports the
//! generic, technology-independent machinery: the per-step state machine
//! (`PrintState<StepType, COUNT>`), the warning accumulation types, the
//! cancellation primitives and the status-notification value types.
//!
//! coord_t -> i64, coordf_t -> f64 (none are used here).
//!
//! ## Blocked symbols (see NOTES at bottom of this file)
//!
//! The following `PrintBase.cpp` members depend on libslic3r subsystems that are
//! not yet ported as faithful Rust equivalents (they exist only as stubs in this
//! crate). They are intentionally NOT faked here:
//!   * `PrintBase::update_object_placeholders` - needs `DynamicConfig` +
//!     `ConfigOptionInt/String/Strings`, `ModelInstance::is_printable()` /
//!     `get_scaling_factor(axis)`, and `boost::filesystem` path handling.
//!   * `PrintBase::output_filename(format, default_ext, ...)` - needs
//!     `DynamicConfig`, `PlaceholderParser::process` (a 1500-line Boost.Spirit
//!     parser, stubbed in `placeholder_parser.rs`) and `boost::filesystem`.
//!   * `PrintBase::output_filepath` - needs `boost::filesystem` and
//!     `Model::propose_export_file_name_and_path`.
//! Everything tractable (the state machine, warnings, cancellation, status value
//! types and the `PrintObjectBase`/`PrintBase` delegation helpers) is ported
//! faithfully below.

use crate::object_id::ObjectID;
use std::sync::Mutex;

// PrintBase.hpp:13
// #define L(s) Slic3r::I18N::translate(s)
// I18N::translate is the identity at runtime for the parity port; the macro is
// reproduced as a passthrough so the call sites read identically to the C++.
#[allow(non_snake_case)]
#[allow(dead_code)] // used by the (blocked) output_filename port; see NOTES.
#[inline]
fn L(s: &str) -> &str {
    // I18N::translate returns the same string (PrintBase.cpp:11-13).
    s
}

// PrintBase.hpp:19-27
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum StringExceptionType {
    StringExceptNotDefined = 0,
    StringExceptFilamentNotMatchBedType = 1,
    StringExceptFilamentsDifferentTemp = 2,
    StringExceptObjectCollisionInSeqPrint = 3,
    StringExceptObjectCollisionInLayerPrint = 4,
    StringExceptLayerHeightExceedsLimit = 5,
    StringExceptCount,
}

// PrintBase.hpp:29-39
// BBS: error with object
#[derive(Debug, Clone)]
pub struct StringObjectException {
    // PrintBase.hpp:32
    pub string: String,
    // PrintBase.hpp:33 : ObjectBase const *object = nullptr;
    // The Rust port carries the ObjectID rather than a raw back-pointer.
    pub object: Option<ObjectID>,
    // PrintBase.hpp:34
    pub opt_key: String,
    // PrintBase.hpp:35 : warning type for tips
    pub r#type: StringExceptionType,
    // PrintBase.hpp:36
    pub is_warning: bool,
    // PrintBase.hpp:37 : warning params for tips
    pub params: Vec<String>,
    // PrintBase.hpp:38
    pub hypetext: String,
}

impl Default for StringObjectException {
    fn default() -> Self {
        // PrintBase.hpp:30-39 : aggregate default-initialization. The struct has
        // no explicit member initializer for `type`, matching the C++ `{}` used in
        // `PrintBase::validate` returning `StringObjectException{}`.
        StringObjectException {
            string: String::new(),
            object: None,
            opt_key: String::new(),
            r#type: StringExceptionType::StringExceptNotDefined,
            is_warning: false,
            params: Vec::new(),
            hypetext: String::new(),
        }
    }
}

// PrintBase.hpp:41-45
// CanceledException : public std::exception
//    const char* what() const throw() { return "Background processing has been canceled"; }
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CanceledException;

impl CanceledException {
    // PrintBase.hpp:44
    pub fn what(&self) -> &'static str {
        "Background processing has been canceled"
    }
}

impl std::fmt::Display for CanceledException {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.what())
    }
}

impl std::error::Error for CanceledException {}

// PrintBase.hpp:47-106
// PrintStateBase
pub struct PrintStateBase;

// PrintBase.hpp:49-53
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum State {
    Invalid,
    Started,
    Done,
}

// PrintBase.hpp:55-58
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WarningLevel {
    NonCritical,
    Critical,
}

// PrintBase.hpp:60-67
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum SlicingNotificationType {
    // normal status update, called by set_status
    SlicingDefaultNotification = 0,
    SlicingReplaceInitEmptyLayers,
    SlicingNeedSupportOn,
    SlicingEmptyGcodeLayers,
    SlicingGcodeOverlap,
}

// PrintBase.hpp:69 : typedef size_t TimeStamp;
pub type TimeStamp = usize;

// PrintBase.hpp:72-77
// A new unique timestamp is being assigned to the step every time the step changes its state.
#[derive(Debug, Clone, Copy)]
pub struct StateWithTimeStamp {
    // PrintBase.hpp:75
    pub state: State,
    // PrintBase.hpp:76
    pub timestamp: TimeStamp,
}

impl StateWithTimeStamp {
    // PrintBase.hpp:74 : StateWithTimeStamp() : state(INVALID), timestamp(0) {}
    pub fn new() -> Self {
        StateWithTimeStamp {
            state: State::Invalid,
            timestamp: 0,
        }
    }
}

impl Default for StateWithTimeStamp {
    fn default() -> Self {
        StateWithTimeStamp::new()
    }
}

// PrintBase.hpp:79-93
#[derive(Debug, Clone)]
pub struct Warning {
    // PrintBase.hpp:81-82
    // Critical warnings will be displayed on G-code export in a modal dialog, so that the user cannot miss them.
    pub level: WarningLevel,
    // PrintBase.hpp:83-86
    // If the warning is not current, then it is in an unknown state. It may or may not be valid.
    // A current warning will become non-current if its milestone gets invalidated.
    // A non-current warning will either become current or it will be removed at the end of a milestone.
    pub current: bool,
    // PrintBase.hpp:87-88
    // Message to be shown to the user, UTF8, localized.
    pub message: String,
    // PrintBase.hpp:89-92
    // If message_id == 0, then the message is expected to identify the warning uniquely.
    // Otherwise message_id identifies the message. For example, if the message contains a varying number, then
    // it cannot itself identify the message type.
    pub message_id: i32,
}

// PrintBase.hpp:95-99
// StateWithWarnings : public StateWithTimeStamp
#[derive(Debug, Clone)]
pub struct StateWithWarnings {
    // PrintBase.hpp:95 (base StateWithTimeStamp members, flattened)
    pub state: State,
    pub timestamp: TimeStamp,
    // PrintBase.hpp:98
    pub warnings: Vec<Warning>,
}

impl StateWithWarnings {
    // Default-construct mirrors StateWithTimeStamp() (state=INVALID, timestamp=0)
    // plus an empty warnings vector.
    pub fn new() -> Self {
        StateWithWarnings {
            state: State::Invalid,
            timestamp: 0,
            warnings: Vec::new(),
        }
    }

    // PrintBase.hpp:97 : void mark_warnings_non_current() { for (auto &w : warnings) w.current = false; }
    pub fn mark_warnings_non_current(&mut self) {
        for w in self.warnings.iter_mut() {
            w.current = false;
        }
    }
}

impl Default for StateWithWarnings {
    fn default() -> Self {
        StateWithWarnings::new()
    }
}

// Conversion used at the read-only accessors `state_with_timestamp`, which in C++
// slices a StateWithWarnings down to its StateWithTimeStamp base.
impl From<&StateWithWarnings> for StateWithTimeStamp {
    fn from(s: &StateWithWarnings) -> Self {
        StateWithTimeStamp {
            state: s.state,
            timestamp: s.timestamp,
        }
    }
}

// PrintBase.hpp:101-105
//FIXME last timestamp is shared between Print & SLAPrint,
// and if multiple Print or SLAPrint instances are executed in parallel, modification of g_last_timestamp
// is not synchronized!
// PrintBase.cpp:23 : size_t PrintStateBase::g_last_timestamp = 0;
//
// The C++ static is a plain (unsynchronized) `size_t`. The Rust port keeps the
// monotonically-increasing `++ g_last_timestamp` semantics. Modelled as an atomic
// purely to satisfy Rust's aliasing rules; the increment is still a simple +1.
static G_LAST_TIMESTAMP: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);

// Pre-increment helper reproducing `++ g_last_timestamp`.
#[inline]
fn next_timestamp() -> TimeStamp {
    G_LAST_TIMESTAMP.fetch_add(1, std::sync::atomic::Ordering::SeqCst) + 1
}

// Trait bridging a `StepType` enum onto the array index used throughout
// `PrintState`. C++ uses `m_state[step]`, `static_cast<int>(step)` and
// `static_cast<StepType>(int)`; the trait provides those two casts.
pub trait StepType: Copy {
    // static_cast<size_t>(step) - index into m_state.
    fn index(self) -> usize;
    // static_cast<StepType>(int) - the inverse cast used by active_step_add_warning.
    fn from_index(idx: usize) -> Self;
}

// PrintBase.hpp:108-331
// To be instantiated over PrintStep or PrintObjectStep enums.
// template <class StepType, size_t COUNT> class PrintState : public PrintStateBase
pub struct PrintState<S: StepType, const COUNT: usize> {
    // PrintBase.hpp:326 : StateWithWarnings m_state[COUNT];
    m_state: Vec<StateWithWarnings>,
    // PrintBase.hpp:327-330
    // Active class StepType or -1 if none is active.
    // If the background processing is canceled, m_step_active may not be resetted
    // to -1, see the comment in this->set_started().
    m_step_active: i32,
    _marker: std::marker::PhantomData<S>,
}

impl<S: StepType, const COUNT: usize> PrintState<S, COUNT> {
    // PrintBase.hpp:113 : PrintState() {}
    pub fn new() -> Self {
        PrintState {
            // m_state[COUNT] default-constructs each StateWithWarnings.
            m_state: (0..COUNT).map(|_| StateWithWarnings::new()).collect(),
            // PrintBase.hpp:330 : int m_step_active = -1;
            m_step_active: -1,
            _marker: std::marker::PhantomData,
        }
    }

    // PrintBase.hpp:115-119
    pub fn state_with_timestamp(&self, step: S, mtx: &Mutex<()>) -> StateWithTimeStamp {
        // std::scoped_lock<std::mutex> lock(mtx);
        let _lock = mtx.lock().unwrap();
        // StateWithTimeStamp state = m_state[step]; (slice base out of StateWithWarnings)
        let state = StateWithTimeStamp::from(&self.m_state[step.index()]);
        state
    }

    // PrintBase.hpp:121-125
    pub fn state_with_warnings(&self, step: S, mtx: &Mutex<()>) -> StateWithWarnings {
        // std::scoped_lock<std::mutex> lock(mtx);
        let _lock = mtx.lock().unwrap();
        // StateWithWarnings state = m_state[step];
        let state = self.m_state[step.index()].clone();
        state
    }

    // PrintBase.hpp:127-129
    pub fn is_started(&self, step: S, mtx: &Mutex<()>) -> bool {
        self.state_with_timestamp(step, mtx).state == State::Started
    }

    // PrintBase.hpp:131-133
    pub fn is_done(&self, step: S, mtx: &Mutex<()>) -> bool {
        self.state_with_timestamp(step, mtx).state == State::Done
    }

    // PrintBase.hpp:135-137
    pub fn state_with_timestamp_unguarded(&self, step: S) -> StateWithTimeStamp {
        StateWithTimeStamp::from(&self.m_state[step.index()])
    }

    // PrintBase.hpp:139-141
    pub fn is_started_unguarded(&self, step: S) -> bool {
        self.state_with_timestamp_unguarded(step).state == State::Started
    }

    // PrintBase.hpp:143-145
    pub fn is_done_unguarded(&self, step: S) -> bool {
        self.state_with_timestamp_unguarded(step).state == State::Done
    }

    // PrintBase.hpp:147-177
    // Set the step as started. Block on mutex while the Print / PrintObject / PrintRegion objects are being
    // modified by the UI thread.
    // This is necessary to block until the Print::apply() updates its state, which may
    // influence the processing step being entered.
    // template<typename ThrowIfCanceled>
    // bool set_started(StepType step, std::mutex &mtx, ThrowIfCanceled throw_if_canceled)
    pub fn set_started<F>(
        &mut self,
        step: S,
        mtx: &Mutex<()>,
        throw_if_canceled: F,
    ) -> Result<bool, CanceledException>
    where
        F: Fn() -> Result<(), CanceledException>,
    {
        // std::scoped_lock<std::mutex> lock(mtx);
        let _lock = mtx.lock().unwrap();
        // If canceled, throw before changing the step state. (PrintBase.hpp:154-155)
        throw_if_canceled()?;
        // PrintBase.hpp:156-168 : the NDEBUG-guarded asserts are commented out in C++.
        // PrintBase.hpp:169-170
        if self.m_state[step.index()].state == State::Done {
            return Ok(false);
        }
        // PrintBase.hpp:171 : PrintStateBase::StateWithWarnings &state = m_state[step];
        let state = &mut self.m_state[step.index()];
        // PrintBase.hpp:172
        state.state = State::Started;
        // PrintBase.hpp:173 : state.timestamp = ++ g_last_timestamp;
        state.timestamp = next_timestamp();
        // PrintBase.hpp:174
        state.mark_warnings_non_current();
        // PrintBase.hpp:175 : m_step_active = static_cast<int>(step);
        self.m_step_active = step.index() as i32;
        // PrintBase.hpp:176
        Ok(true)
    }

    // PrintBase.hpp:179-203
    // Set the step as done. Block on mutex while the Print / PrintObject / PrintRegion objects are being
    // modified by the UI thread.
    // Return value:
    //      Timestamp when this step entered the DONE state.
    //      bool indicates whether the UI has to update the slicing warnings of this step or not.
    // template<typename ThrowIfCanceled>
    // std::pair<TimeStamp, bool> set_done(StepType step, std::mutex &mtx, ThrowIfCanceled throw_if_canceled)
    pub fn set_done<F>(
        &mut self,
        step: S,
        mtx: &Mutex<()>,
        throw_if_canceled: F,
    ) -> Result<(TimeStamp, bool), CanceledException>
    where
        F: Fn() -> Result<(), CanceledException>,
    {
        // std::scoped_lock<std::mutex> lock(mtx);
        let _lock = mtx.lock().unwrap();
        // If canceled, throw before changing the step state. (PrintBase.hpp:187-188)
        throw_if_canceled()?;
        // PrintBase.hpp:189 : assert(m_state[step].state == STARTED);
        debug_assert!(self.m_state[step.index()].state == State::Started);
        // PrintBase.hpp:190 : assert(m_step_active == static_cast<int>(step));
        debug_assert!(self.m_step_active == step.index() as i32);
        // PrintBase.hpp:191 : PrintStateBase::StateWithWarnings &state = m_state[step];
        let state = &mut self.m_state[step.index()];
        // PrintBase.hpp:192
        state.state = State::Done;
        // PrintBase.hpp:193 : state.timestamp = ++ g_last_timestamp;
        state.timestamp = next_timestamp();
        // PrintBase.hpp:194
        self.m_step_active = -1;
        // PrintBase.hpp:195-201
        // Remove all non-current warnings.
        // auto it = std::remove_if(state.warnings.begin(), state.warnings.end(), [](const auto &w){ return ! w.current; });
        // bool update_warning_ui = false;
        // if (it != state.warnings.end()) { state.warnings.erase(it, end); update_warning_ui = true; }
        let state = &mut self.m_state[step.index()];
        let before = state.warnings.len();
        state.warnings.retain(|w| w.current);
        let mut update_warning_ui = false;
        if state.warnings.len() != before {
            update_warning_ui = true;
        }
        // PrintBase.hpp:202 : return std::make_pair(state.timestamp, update_warning_ui);
        Ok((state.timestamp, update_warning_ui))
    }

    // PrintBase.hpp:205-232
    // Make the step invalid.
    // PrintBase::m_state_mutex should be locked at this point, guarding access to m_state.
    // In case the step has already been entered or finished, cancel the background
    // processing by calling the cancel callback.
    // template<typename CancelationCallback>
    // bool invalidate(StepType step, CancelationCallback cancel)
    pub fn invalidate<C>(&mut self, step: S, cancel: C) -> bool
    where
        C: FnOnce(),
    {
        // PrintBase.hpp:211 : bool invalidated = m_state[step].state != INVALID;
        let invalidated = self.m_state[step.index()].state != State::Invalid;
        if invalidated {
            // PrintBase.hpp:218 : PrintStateBase::StateWithWarnings &state = m_state[step];
            let state = &mut self.m_state[step.index()];
            // PrintBase.hpp:219
            state.state = State::Invalid;
            // PrintBase.hpp:220 : state.timestamp = ++ g_last_timestamp;
            state.timestamp = next_timestamp();
            // PrintBase.hpp:221-225
            // Raise the mutex, so that the following cancel() callback could cancel
            // the background processing.
            // Internally the cancel() callback shall unlock the PrintBase::m_status_mutex to let
            // the working thread proceed.
            cancel();
            // PrintBase.hpp:226-228
            // Now the worker thread should be stopped, therefore it cannot write into the warnings field.
            // It is safe to modify it.
            self.m_state[step.index()].mark_warnings_non_current();
            // PrintBase.hpp:229
            self.m_step_active = -1;
        }
        // PrintBase.hpp:231
        invalidated
    }

    // PrintBase.hpp:234-263
    // template<typename CancelationCallback, typename StepTypeIterator>
    // bool invalidate_multiple(StepTypeIterator step_begin, StepTypeIterator step_end, CancelationCallback cancel)
    pub fn invalidate_multiple<C, I>(&mut self, steps: I, cancel: C) -> bool
    where
        C: FnOnce(),
        I: IntoIterator<Item = S> + Clone,
    {
        // PrintBase.hpp:236
        let mut invalidated = false;
        // PrintBase.hpp:237-244
        // for (StepTypeIterator it = step_begin; it != step_end; ++ it) {
        //     StateWithTimeStamp &state = m_state[*it];
        //     if (state.state != INVALID) { invalidated = true; state.state = INVALID; state.timestamp = ++ g_last_timestamp; }
        // }
        for it in steps.clone() {
            let state = &mut self.m_state[it.index()];
            if state.state != State::Invalid {
                invalidated = true;
                state.state = State::Invalid;
                state.timestamp = next_timestamp();
            }
        }
        // PrintBase.hpp:245
        if invalidated {
            // PrintBase.hpp:251-255
            // Raise the mutex, so that the following cancel() callback could cancel
            // the background processing.
            cancel();
            // PrintBase.hpp:256-259
            // Now the worker thread should be stopped, therefore it cannot write into the warnings field.
            // for (StepTypeIterator it = step_begin; it != step_end; ++ it) m_state[*it].mark_warnings_non_current();
            for it in steps {
                self.m_state[it.index()].mark_warnings_non_current();
            }
            // PrintBase.hpp:260
            self.m_step_active = -1;
        }
        // PrintBase.hpp:262
        invalidated
    }

    // PrintBase.hpp:265-289
    // Make all steps invalid.
    // PrintBase::m_state_mutex should be locked at this point, guarding access to m_state.
    // In case any step has already been entered or finished, cancel the background
    // processing by calling the cancel callback.
    // template<typename CancelationCallback>
    // bool invalidate_all(CancelationCallback cancel)
    pub fn invalidate_all<C>(&mut self, cancel: C) -> bool
    where
        C: FnOnce(),
    {
        // PrintBase.hpp:271
        let mut invalidated = false;
        // PrintBase.hpp:272-279
        for i in 0..COUNT {
            let state = &mut self.m_state[i];
            if state.state != State::Invalid {
                invalidated = true;
                state.state = State::Invalid;
                state.timestamp = next_timestamp();
            }
        }
        // PrintBase.hpp:280
        if invalidated {
            // PrintBase.hpp:281
            cancel();
            // PrintBase.hpp:282-285
            // Now the worker thread should be stopped, therefore it cannot write into the warnings field.
            for i in 0..COUNT {
                self.m_state[i].mark_warnings_non_current();
            }
            // PrintBase.hpp:286
            self.m_step_active = -1;
        }
        // PrintBase.hpp:288
        invalidated
    }

    // PrintBase.hpp:291-323
    // Update list of warnings of the current milestone with a new warning.
    // The warning may already exist in the list, marked as current or not current.
    // If it already exists, mark it as current.
    // Return value:
    //      Current milestone (StepType).
    //      bool indicates whether the UI has to be updated or not.
    // std::pair<StepType, bool> active_step_add_warning(WarningLevel warning_level, const std::string &message, int message_id, std::mutex &mtx)
    pub fn active_step_add_warning(
        &mut self,
        warning_level: WarningLevel,
        message: &str,
        message_id: i32,
        mtx: &Mutex<()>,
    ) -> (S, bool) {
        // std::scoped_lock<std::mutex> lock(mtx);
        let _lock = mtx.lock().unwrap();
        // PrintBase.hpp:300 : assert(m_step_active != -1);
        debug_assert!(self.m_step_active != -1);
        // PrintBase.hpp:301 : StateWithWarnings &state = m_state[m_step_active];
        let active_idx = self.m_step_active as usize;
        let state = &mut self.m_state[active_idx];
        // PrintBase.hpp:302 : assert(state.state == STARTED);
        debug_assert!(state.state == State::Started);
        // PrintBase.hpp:303 : std::pair<StepType, bool> retval(static_cast<StepType>(m_step_active), true);
        let mut retval: (S, bool) = (S::from_index(active_idx), true);
        // PrintBase.hpp:304-307
        // Does a warning of the same level and message or message_id exist already?
        // auto it = (message_id == 0) ?
        //     find_if(... w.message_id == 0 && w.message == message) :
        //     find_if(... w.message_id == message_id);
        let pos = if message_id == 0 {
            state
                .warnings
                .iter()
                .position(|w| w.message_id == 0 && w.message == message)
        } else {
            state
                .warnings
                .iter()
                .position(|w| w.message_id == message_id)
        };
        match pos {
            // PrintBase.hpp:308-310
            // No, create a new warning and update UI.
            None => {
                state.warnings.push(Warning {
                    level: warning_level,
                    current: true,
                    message: message.to_string(),
                    message_id,
                });
            }
            Some(i) => {
                let w = &mut state.warnings[i];
                // PrintBase.hpp:311-315
                // else if (it->message != message || it->level != warning_level) {
                //     // Yes, however it needs an update.
                //     it->message = message; it->level = warning_level; it->current = true;
                if w.message != message || w.level != warning_level {
                    w.message = message.to_string();
                    w.level = warning_level;
                    w.current = true;
                } else if w.current {
                    // PrintBase.hpp:316-318
                    // Yes, and it is current. Don't update UI.
                    retval.1 = false;
                } else {
                    // PrintBase.hpp:319-321
                    // Yes, but it is not current. Mark it as current.
                    w.current = true;
                }
            }
        }
        // PrintBase.hpp:322
        retval
    }
}

impl<S: StepType, const COUNT: usize> Default for PrintState<S, COUNT> {
    fn default() -> Self {
        PrintState::new()
    }
}

// PrintBase.cpp:18-21
// void PrintTryCancel::operator()() { m_print->throw_if_canceled(); }
//
// In C++, PrintTryCancel holds a `const PrintBase *m_print` and forwards to the
// private throw_if_canceled(). Modelled in Rust as a wrapper around a cloned
// cancel-check closure passed in by the owning PrintBase, since PrintBase here is
// a trait rather than a concrete base class with a back-pointer.
pub struct PrintTryCancel {
    // PrintBase.hpp:368 : const PrintBase *m_print;
    throw_if_canceled: std::sync::Arc<dyn Fn() -> Result<(), CanceledException> + Send + Sync>,
}

impl PrintTryCancel {
    // PrintBase.hpp:367 : PrintTryCancel(const PrintBase *print) : m_print(print) {}
    pub fn new(
        throw_if_canceled: std::sync::Arc<
            dyn Fn() -> Result<(), CanceledException> + Send + Sync,
        >,
    ) -> Self {
        PrintTryCancel { throw_if_canceled }
    }

    // PrintBase.cpp:18-21 : void PrintTryCancel::operator()() { m_print->throw_if_canceled(); }
    pub fn call(&self) -> Result<(), CanceledException> {
        (self.throw_if_canceled)()
    }
}

// PrintBase.hpp:435-474
// SlicingStatus value type. PrintBase.cpp uses this for set_status() and
// status_update_warnings(). FlagBits is mirrored as associated constants.
#[derive(Debug, Clone)]
pub struct SlicingStatus {
    // PrintBase.hpp:451 : int percent { -1 };
    pub percent: i32,
    // PrintBase.hpp:452 : bool is_helio{false};
    pub is_helio: bool,
    // PrintBase.hpp:453
    pub text: String,
    // PrintBase.hpp:464-465 : unsigned int flags;
    pub flags: u32,
    // PrintBase.hpp:466-468 : ObjectID warning_object_id;
    pub warning_object_id: ObjectID,
    // PrintBase.hpp:469-470 : int warning_step { -1 };
    pub warning_step: i32,
    // PrintBase.hpp:472
    pub message_type: SlicingNotificationType,
    // PrintBase.hpp:473
    pub warning_level: WarningLevel,
}

impl SlicingStatus {
    // PrintBase.hpp:455-463 : enum FlagBits
    pub const DEFAULT: u32 = 0;
    pub const RELOAD_SCENE: u32 = 1 << 1;
    pub const RELOAD_SLA_SUPPORT_POINTS: u32 = 1 << 2;
    pub const RELOAD_SLA_PREVIEW: u32 = 1 << 3;
    // UPDATE_PRINT_STEP_WARNINGS is mutually exclusive with UPDATE_PRINT_OBJECT_STEP_WARNINGS.
    pub const UPDATE_PRINT_STEP_WARNINGS: u32 = 1 << 4;
    pub const UPDATE_PRINT_OBJECT_STEP_WARNINGS: u32 = 1 << 5;

    // PrintBase.hpp:436-440
    // SlicingStatus(int percent, const std::string &text, unsigned int flags = 0, int warning_step = -1,
    //     SlicingNotificationType msg_type = SlicingDefaultNotification, WarningLevel warning_level = NON_CRITICAL)
    pub fn from_percent(
        percent: i32,
        text: String,
        flags: u32,
        warning_step: i32,
        msg_type: SlicingNotificationType,
        warning_level: WarningLevel,
    ) -> Self {
        SlicingStatus {
            // PrintBase.hpp:438
            percent,
            is_helio: false,
            text,
            flags,
            // warning_object_id default-constructs to an invalid ObjectID.
            warning_object_id: ObjectID::default_invalid(),
            warning_step,
            message_type: msg_type,
            warning_level,
        }
    }

    // PrintBase.hpp:441-445
    // SlicingStatus(const PrintBase &print, int warning_step, const std::string& text,
    //     SlicingNotificationType msg_type = SlicingDefaultNotification, WarningLevel warning_level = NON_CRITICAL) :
    //     flags(UPDATE_PRINT_STEP_WARNINGS), warning_object_id(print.id()), text(text), warning_step(warning_step), message_type(msg_type), warning_level(warning_level)
    //
    // The print object is reduced to its ObjectID for this port.
    pub fn from_print(
        print_id: ObjectID,
        warning_step: i32,
        text: String,
        msg_type: SlicingNotificationType,
        warning_level: WarningLevel,
    ) -> Self {
        SlicingStatus {
            // PrintBase.hpp:451 : int percent { -1 }; (not in the member-init list -> default)
            percent: -1,
            is_helio: false,
            text,
            // PrintBase.hpp:443 : flags(UPDATE_PRINT_STEP_WARNINGS)
            flags: SlicingStatus::UPDATE_PRINT_STEP_WARNINGS,
            // PrintBase.hpp:443 : warning_object_id(print.id())
            warning_object_id: print_id,
            warning_step,
            message_type: msg_type,
            warning_level,
        }
    }

    // PrintBase.hpp:446-450
    // SlicingStatus(const PrintObjectBase &print_object, int warning_step, const std::string& text,
    //     SlicingNotificationType msg_type = SlicingDefaultNotification, WarningLevel warning_level = NON_CRITICAL) :
    //     flags(UPDATE_PRINT_OBJECT_STEP_WARNINGS), warning_object_id(print_object.id()), ...
    pub fn from_print_object(
        print_object_id: ObjectID,
        warning_step: i32,
        text: String,
        msg_type: SlicingNotificationType,
        warning_level: WarningLevel,
    ) -> Self {
        SlicingStatus {
            percent: -1,
            is_helio: false,
            text,
            // PrintBase.hpp:448 : flags(UPDATE_PRINT_OBJECT_STEP_WARNINGS)
            flags: SlicingStatus::UPDATE_PRINT_OBJECT_STEP_WARNINGS,
            // PrintBase.hpp:448 : warning_object_id(print_object.id())
            warning_object_id: print_object_id,
            warning_step,
            message_type: msg_type,
            warning_level,
        }
    }
}

// PrintBase.hpp:402-410
// PrintBase::ApplyStatus
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ApplyStatus {
    // No change after the Print::apply() call.
    ApplyStatusUnchanged,
    // Some of the Print / PrintObject / PrintObjectInstance data was changed,
    // but no result was invalidated (only data influencing not yet calculated results were changed).
    ApplyStatusChanged,
    // Some data was changed, which in turn invalidated already calculated steps.
    ApplyStatusInvalidated,
}

// PrintBase.hpp:491-498
// PrintBase::CancelStatus
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum CancelStatus {
    // No cancelation, background processing should run.
    NotCanceled = 0,
    // Canceled by user from the user interface (user pressed the "Cancel" button or user closed the application).
    CanceledByUser = 1,
    // Canceled internally from Print::apply() through the Print/PrintObject::invalidate_step() or ::invalidate_all_steps().
    CanceledInternal = 2,
}

// PrintBase.hpp:414-424
// PrintBase::TaskParams
#[derive(Debug, Clone)]
pub struct TaskParams {
    // PrintBase.hpp:417
    // If non-empty, limit the processing to this ModelObject.
    pub single_model_object: ObjectID,
    // PrintBase.hpp:418-419
    // If set, only process single_model_object. Otherwise process everything, but single_model_object first.
    pub single_model_instance_only: bool,
    // PrintBase.hpp:420-421
    // If non-negative, stop processing at the successive object step.
    pub to_object_step: i32,
    // PrintBase.hpp:422-423
    // If non-negative, stop processing at the successive print step.
    pub to_print_step: i32,
}

impl TaskParams {
    // PrintBase.hpp:415 : TaskParams() : single_model_object(0), single_model_instance_only(false), to_object_step(-1), to_print_step(-1) {}
    pub fn new() -> Self {
        TaskParams {
            single_model_object: ObjectID::new(0),
            single_model_instance_only: false,
            to_object_step: -1,
            to_print_step: -1,
        }
    }
}

impl Default for TaskParams {
    fn default() -> Self {
        TaskParams::new()
    }
}

// status_callback_type (PrintBase.hpp:475) : std::function<void(const SlicingStatus&)>
pub type StatusCallbackType = std::sync::Arc<dyn Fn(&SlicingStatus) + Send + Sync>;
// cancel_callback_type (PrintBase.hpp:485) : std::function<void()>
pub type CancelCallbackType = std::sync::Arc<dyn Fn() + Send + Sync>;

// ---------------------------------------------------------------------------
// PrintBase.cpp:107-153 : status / warning notification free helpers.
//
// In C++ these are members of PrintBase / PrintObjectBase that read m_status_callback.
// They are ported as free functions taking the resolved status callback (an
// `Option<StatusCallbackType>`), so any concrete PrintBase implementation in this
// crate can delegate to them without inheriting from a base class.
// ---------------------------------------------------------------------------

// PrintBase.cpp:106-113
//BBS: move set_status from hpp to cpp
// void PrintBase::set_status(int percent, const std::string &message, unsigned int flags, int warning_step) const
pub fn set_status(
    status_callback: &Option<StatusCallbackType>,
    percent: i32,
    message: &str,
    flags: u32,
    warning_step: i32,
) {
    // PrintBase.cpp:109-110
    if let Some(cb) = status_callback {
        cb(&SlicingStatus::from_percent(
            percent,
            message.to_string(),
            flags,
            warning_step,
            // default arguments (PrintBase.hpp:437)
            SlicingNotificationType::SlicingDefaultNotification,
            WarningLevel::NonCritical,
        ));
    } else {
        // PrintBase.cpp:111-112
        // BOOST_LOG_TRIVIAL(debug) << boost::format("Percent %1%: %2%\n") % percent % message.c_str();
        log::debug!("Percent {}: {}\n", percent, message);
    }
}

// PrintBase.cpp:115-124
// void PrintBase::status_update_warnings(int step, WarningLevel warning_level, const std::string &message,
//     const PrintObjectBase* print_object, SlicingNotificationType message_id)
//
// `print_id` is the owning PrintBase id (used when print_object is None);
// `print_object_id` is Some when a PrintObjectBase is supplied.
pub fn status_update_warnings(
    status_callback: &Option<StatusCallbackType>,
    print_id: ObjectID,
    step: i32,
    warning_level: WarningLevel,
    message: &str,
    print_object_id: Option<ObjectID>,
    message_id: SlicingNotificationType,
) {
    // PrintBase.cpp:118-121
    if let Some(cb) = status_callback {
        // auto status = print_object ? SlicingStatus(*print_object, step, message, message_id, warning_level)
        //                            : SlicingStatus(*this, step, message, message_id, warning_level);
        let status = match print_object_id {
            Some(obj_id) => SlicingStatus::from_print_object(
                obj_id,
                step,
                message.to_string(),
                message_id,
                warning_level,
            ),
            None => SlicingStatus::from_print(
                print_id,
                step,
                message.to_string(),
                message_id,
                warning_level,
            ),
        };
        cb(&status);
    } else if !message.is_empty() {
        // PrintBase.cpp:122-123
        // BOOST_LOG_TRIVIAL(info) << __FUNCTION__ << boost::format(", Print warning: %1%\n") % message.c_str();
        log::info!("status_update_warnings, Print warning: {}\n", message);
    }
}

// PrintBase.cpp:126-136
//BBS: add PrintObject id into slicing status
// void PrintBase::status_update_warnings(int step, WarningLevel warning_level, const std::string& message,
//     PrintObjectBase &object, SlicingNotificationType message_id)
pub fn status_update_warnings_object(
    status_callback: &Option<StatusCallbackType>,
    step: i32,
    warning_level: WarningLevel,
    message: &str,
    object_id: ObjectID,
    message_id: SlicingNotificationType,
) {
    //BBS: add object it into slicing status (PrintBase.cpp:130-133)
    if let Some(cb) = status_callback {
        cb(&SlicingStatus::from_print_object(
            object_id,
            step,
            message.to_string(),
            message_id,
            warning_level,
        ));
    } else if !message.is_empty() {
        // PrintBase.cpp:134-135
        // BOOST_LOG_TRIVIAL(info) << __FUNCTION__ << boost::format(", PrintObject warning: %1%\n") % message.c_str();
        log::info!(
            "status_update_warnings, PrintObject warning: {}\n",
            message
        );
    }
}

// PrintBase.cpp:139-153 : PrintObjectBase delegation helpers.
//
// std::mutex& PrintObjectBase::state_mutex(PrintBase *print) { return print->state_mutex(); }
// std::function<void()> PrintObjectBase::cancel_callback(PrintBase *print) { return print->cancel_callback(); }
// void PrintObjectBase::status_update_warnings(PrintBase *print, int step, WarningLevel warning_level,
//     const std::string &message, SlicingNotificationType message_id)
//     { print->status_update_warnings(step, warning_level, message, this, message_id); }
//
// These are pure forwarders to the owning PrintBase. In this crate PrintBase is a
// trait, so the forwarders live as default methods on that trait (below) rather
// than as free functions with a raw PrintBase* back-pointer.

// PrintBase.hpp:382-582
// The PrintBase abstract base class. The concrete print orchestrator in this crate
// lives in `print.rs`; here we expose the technology-independent contract as a
// trait that mirrors the C++ virtuals plus the concrete helpers that only depend
// on a status callback + object id.
pub trait PrintBaseTrait {
    // PrintBase.hpp:388 : virtual PrinterTechnology technology() const noexcept = 0;
    // (PrinterTechnology lives in PrintConfig; left to the implementor.)

    // PrintBase.hpp:412 : const Model& model() const { return m_model; }
    // (Model accessor is implementor-specific.)

    // PrintBase.hpp:443 etc. : print.id()
    fn id(&self) -> ObjectID;

    // PrintBase.hpp:533 : std::mutex& state_mutex() const { return m_state_mutex; }
    fn state_mutex(&self) -> &Mutex<()>;

    // Resolved status callback (PrintBase.hpp:568 : m_status_callback).
    fn status_callback(&self) -> &Option<StatusCallbackType>;

    // PrintBase.hpp:483 / PrintBase.cpp:107 : set_status forwarder.
    fn set_status(&self, percent: i32, message: &str, flags: u32, warning_step: i32) {
        set_status(self.status_callback(), percent, message, flags, warning_step)
    }

    // PrintBase.hpp:539-540 / PrintBase.cpp:115 : status_update_warnings forwarder.
    fn status_update_warnings(
        &self,
        step: i32,
        warning_level: WarningLevel,
        message: &str,
        print_object_id: Option<ObjectID>,
        message_id: SlicingNotificationType,
    ) {
        status_update_warnings(
            self.status_callback(),
            self.id(),
            step,
            warning_level,
            message,
            print_object_id,
            message_id,
        )
    }

    // PrintBase.hpp:542-543 / PrintBase.cpp:127 : status_update_warnings (PrintObject) forwarder.
    fn status_update_warnings_object(
        &self,
        step: i32,
        warning_level: WarningLevel,
        message: &str,
        object_id: ObjectID,
        message_id: SlicingNotificationType,
    ) {
        status_update_warnings_object(
            self.status_callback(),
            step,
            warning_level,
            message,
            object_id,
            message_id,
        )
    }
}

// ---------------------------------------------------------------------------
// NOTES on blocked PrintBase.cpp members (NOT ported; would require fakes):
//
//  * update_object_placeholders (PrintBase.cpp:26-62): iterates m_model.objects,
//    tests ModelInstance::is_printable() and reads get_scaling_factor(X/Y/Z), then
//    writes ConfigOptionInt / ConfigOptionStrings / ConfigOptionString entries into
//    a DynamicConfig via set_key_value. The Rust crate's Model uses a simplified
//    Instance (bool printable, scale[3]) and has no DynamicConfig / ConfigOptionXxx
//    key-value layer. Faithful porting is blocked on those config primitives.
//
//  * output_filename(format, default_ext, filename_base, config_override)
//    (PrintBase.cpp:66-88): builds a DynamicConfig, sets "version"=SLIC3R_VERSION,
//    calls PlaceholderParser::update_timestamp + this->update_object_placeholders,
//    then PlaceholderParser::process(format, 0, &cfg) and boost::filesystem path /
//    extension handling. Blocked on DynamicConfig, the (stubbed) PlaceholderParser
//    expression engine, and a boost::filesystem equivalent.
//
//  * output_filepath (PrintBase.cpp:90-104): boost::filesystem path joins,
//    is_directory(), make_preferred(), and Model::propose_export_file_name_and_path
//    (not present in the Rust Model). Blocked on those.
//
// When DynamicConfig / ConfigOptionXxx and PlaceholderParser::process gain faithful
// Rust ports, these three functions should be added here, mirroring PrintBase.cpp
// lines 26-104 exactly.
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    // Minimal StepType for exercising the PrintState state machine.
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    enum TestStep {
        A,
        B,
        C,
    }
    impl StepType for TestStep {
        fn index(self) -> usize {
            self as usize
        }
        fn from_index(idx: usize) -> Self {
            match idx {
                0 => TestStep::A,
                1 => TestStep::B,
                _ => TestStep::C,
            }
        }
    }

    fn ok() -> Result<(), CanceledException> {
        Ok(())
    }

    #[test]
    fn set_started_then_done() {
        let mtx = Mutex::new(());
        let mut st: PrintState<TestStep, 3> = PrintState::new();
        assert!(!st.is_done(TestStep::A, &mtx));
        assert!(st.set_started(TestStep::A, &mtx, ok).unwrap());
        assert!(st.is_started(TestStep::A, &mtx));
        // Second set_started on a not-DONE step returns true again.
        let (_ts, _ui) = st.set_done(TestStep::A, &mtx, ok).unwrap();
        assert!(st.is_done(TestStep::A, &mtx));
        // set_started on a DONE step returns false (PrintBase.hpp:169-170).
        assert!(!st.set_started(TestStep::A, &mtx, ok).unwrap());
    }

    #[test]
    fn invalidate_resets_state() {
        let mtx = Mutex::new(());
        let mut st: PrintState<TestStep, 3> = PrintState::new();
        st.set_started(TestStep::B, &mtx, ok).unwrap();
        let mut canceled = false;
        let inv = st.invalidate(TestStep::B, || canceled = true);
        assert!(inv);
        assert!(canceled);
        assert!(!st.is_started(TestStep::B, &mtx));
        // Invalidating an already-invalid step does not call cancel and returns false.
        let inv2 = st.invalidate(TestStep::B, || panic!("should not cancel"));
        assert!(!inv2);
    }

    #[test]
    fn add_warning_dedup() {
        let mtx = Mutex::new(());
        let mut st: PrintState<TestStep, 3> = PrintState::new();
        st.set_started(TestStep::A, &mtx, ok).unwrap();
        let (step, ui) =
            st.active_step_add_warning(WarningLevel::NonCritical, "hello", 0, &mtx);
        assert_eq!(step, TestStep::A);
        assert!(ui);
        // Same warning again, already current -> no UI update (PrintBase.hpp:316-318).
        let (_step, ui2) =
            st.active_step_add_warning(WarningLevel::NonCritical, "hello", 0, &mtx);
        assert!(!ui2);
    }
}
