//! Faithful 1:1 port of BambuStudio `GCodeReader` (GCodeReader.cpp / GCodeReader.hpp).
//!
//! This is a streaming G-code parser. It reads files or buffers, parses individual
//! lines, extracts axis values and maintains current position state. The port mirrors
//! the C++ control flow, constants, rounding and edge cases line by line so that the
//! downstream G-code is byte-exact.
//!
//! C++ Reference:
//! - GCodeReader.hpp
//! - GCodeReader.cpp

use crate::print_config::GCodeConfig;

// ---------------------------------------------------------------------------
// Axis enum (libslic3r.h:114-128)
// ---------------------------------------------------------------------------

/// Axis identifiers for G-code commands.
/// libslic3r.h:114-128
///
/// `X=0, Y, Z, E, F, I, J, P, NUM_AXES, UNKNOWN_AXIS = NUM_AXES, NUM_AXES_WITH_UNKNOWN`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum Axis {
    X = 0,
    Y = 1,
    Z = 2,
    E = 3,
    F = 4,
    // BBS: add I, J, P axis
    I = 5,
    J = 6,
    P = 7,
    // libslic3r.h:124
    NumAxes = 8,
    // For the GCodeReader to mark a parsed axis, which is not in "XYZEF", it was parsed correctly.
    // libslic3r.h:126: UNKNOWN_AXIS = NUM_AXES,
    UnknownAxis = 9,
    // libslic3r.h:127
    NumAxesWithUnknown = 10,
}

/// Number of standard axes (libslic3r.h:124: NUM_AXES).
pub const NUM_AXES: usize = 8;
/// Axis index used to mark a parsed-but-unknown axis (libslic3r.h:126: UNKNOWN_AXIS = NUM_AXES).
pub const UNKNOWN_AXIS: usize = 8;

impl Axis {
    /// Return the integer index of this axis, matching `int(axis)` in C++.
    #[inline]
    pub fn index(self) -> usize {
        match self {
            Axis::X => 0,
            Axis::Y => 1,
            Axis::Z => 2,
            Axis::E => 3,
            Axis::F => 4,
            Axis::I => 5,
            Axis::J => 6,
            Axis::P => 7,
            Axis::NumAxes => 8,
            Axis::UnknownAxis => 9,
            Axis::NumAxesWithUnknown => 10,
        }
    }
}

// ---------------------------------------------------------------------------
// Static character classification helpers (GCodeReader.hpp:159-172)
// ---------------------------------------------------------------------------

/// GCodeReader.hpp:159
#[inline]
fn is_whitespace(c: u8) -> bool {
    c == b' ' || c == b'\t'
}

/// GCodeReader.hpp:160
#[inline]
fn is_end_of_line(c: u8) -> bool {
    c == b'\r' || c == b'\n' || c == 0
}

/// GCodeReader.hpp:161
#[inline]
fn is_end_of_gcode_line(c: u8) -> bool {
    c == b';' || is_end_of_line(c)
}

/// GCodeReader.hpp:162
#[inline]
fn is_end_of_word(c: u8) -> bool {
    is_whitespace(c) || is_end_of_gcode_line(c)
}

/// GCodeReader.hpp:163-167
///
/// Advances `i` over whitespace in `buf` and returns the new index.
#[inline]
fn skip_whitespaces(buf: &[u8], mut i: usize) -> usize {
    // for (; is_whitespace(*c); ++ c) ;
    while i < buf.len() && is_whitespace(buf[i]) {
        i += 1;
    }
    i
}

/// GCodeReader.hpp:168-172
///
/// Advances `i` to the end of the current word in `buf` and returns the new index.
#[inline]
fn skip_word(buf: &[u8], mut i: usize) -> usize {
    // for (; ! is_end_of_word(*c); ++ c) ;
    while i < buf.len() && !is_end_of_word(buf[i]) {
        i += 1;
    }
    i
}

/// Read a single byte from `buf` at `i`, returning the NUL terminator (0) when out of
/// range. C++ scans a NUL-terminated string, so reaching the end behaves like reading
/// `'\0'`, which `is_end_of_line` treats as a line terminator.
#[inline]
fn at(buf: &[u8], i: usize) -> u8 {
    if i < buf.len() {
        buf[i]
    } else {
        0
    }
}

// ---------------------------------------------------------------------------
// GCodeLine (GCodeReader.hpp:16-92)
// ---------------------------------------------------------------------------

/// A single parsed line of G-code.
/// GCodeReader.hpp:16-92
#[derive(Debug, Clone)]
pub struct GCodeLine {
    /// GCodeReader.hpp:88: std::string m_raw;
    m_raw: String,
    /// GCodeReader.hpp:89: float m_axis[NUM_AXES];
    m_axis: [f32; NUM_AXES],
    /// GCodeReader.hpp:90: uint32_t m_mask;
    m_mask: u32,
}

impl GCodeLine {
    /// GCodeReader.hpp:18: GCodeLine() { reset(); }
    pub fn new() -> Self {
        let mut g = Self {
            m_raw: String::new(),
            m_axis: [0.0; NUM_AXES],
            m_mask: 0,
        };
        g.reset();
        g
    }

    /// GCodeReader.hpp:19: void reset() { m_mask = 0; memset(m_axis, 0, sizeof(m_axis)); m_raw.clear(); }
    pub fn reset(&mut self) {
        self.m_mask = 0;
        self.m_axis = [0.0; NUM_AXES];
        self.m_raw.clear();
    }

    /// GCodeReader.hpp:21: const std::string& raw() const { return m_raw; }
    pub fn raw(&self) -> &str {
        &self.m_raw
    }

    /// GCodeReader.hpp:22-25
    /// const std::string_view cmd() const {
    ///     const char *cmd = GCodeReader::skip_whitespaces(m_raw.c_str());
    ///     return std::string_view(cmd, GCodeReader::skip_word(cmd) - cmd);
    /// }
    pub fn cmd(&self) -> &str {
        let buf = self.m_raw.as_bytes();
        let cmd = skip_whitespaces(buf, 0);
        let end = skip_word(buf, cmd);
        // m_raw is ASCII G-code; slicing on byte indices is valid here.
        &self.m_raw[cmd..end]
    }

    /// GCodeReader.hpp:26-27
    /// const std::string_view comment() const
    ///     { size_t pos = m_raw.find(';'); return (pos == npos) ? {} : m_raw.substr(pos + 1); }
    pub fn comment(&self) -> &str {
        match self.m_raw.find(';') {
            Some(pos) => &self.m_raw[pos + 1..],
            None => "",
        }
    }

    /// GCodeReader.hpp:29: void clear() { m_raw.clear(); }
    pub fn clear(&mut self) {
        self.m_raw.clear();
    }

    /// GCodeReader.hpp:30: bool has(Axis axis) const { return (m_mask & (1 << int(axis))) != 0; }
    pub fn has(&self, axis: Axis) -> bool {
        (self.m_mask & (1 << axis.index())) != 0
    }

    /// GCodeReader.hpp:31: float value(Axis axis) const { return m_axis[axis]; }
    pub fn value(&self, axis: Axis) -> f32 {
        self.m_axis[axis.index()]
    }

    /// GCodeReader.hpp:32: bool has(char axis) const;
    /// GCodeReader.cpp:225-245
    pub fn has_char(&self, axis: u8) -> bool {
        let buf = self.m_raw.as_bytes();
        // const char *c = m_raw.c_str();
        let mut c = 0usize;
        // Skip the whitespaces.
        c = skip_whitespaces(buf, c);
        // Skip the command.
        c = skip_word(buf, c);
        // Up to the end of line or comment.
        while !is_end_of_gcode_line(at(buf, c)) {
            // Skip whitespaces.
            c = skip_whitespaces(buf, c);
            if is_end_of_gcode_line(at(buf, c)) {
                break;
            }
            // Check the name of the axis.
            if at(buf, c) == axis {
                return true;
            }
            // Skip the rest of the word.
            c = skip_word(buf, c);
        }
        false
    }

    /// GCodeReader.hpp:33: bool has_value(char axis, float &value) const;
    /// GCodeReader.cpp:247-276
    pub fn has_value(&self, axis: u8) -> Option<f32> {
        // assert(is_decimal_separator_point());
        let buf = self.m_raw.as_bytes();
        // const char *c = m_raw.c_str();
        let mut c = 0usize;
        // Skip the whitespaces.
        c = skip_whitespaces(buf, c);
        // Skip the command.
        c = skip_word(buf, c);
        // Up to the end of line or comment.
        while !is_end_of_gcode_line(at(buf, c)) {
            // Skip whitespaces.
            c = skip_whitespaces(buf, c);
            if is_end_of_gcode_line(at(buf, c)) {
                break;
            }
            // Check the name of the axis.
            if at(buf, c) == axis {
                // Try to parse the numeric value.
                // double v = strtod(++ c, &pend);
                c += 1;
                let (v, pend) = strtod(buf, c);
                if pend != c && is_end_of_word(at(buf, pend)) {
                    // The axis value has been parsed correctly.
                    return Some(v as f32);
                }
                // strtod did not advance / not end-of-word; fall through to skip the word.
                // (Match C++: when the if-branch fails we still skip the rest of the word.)
                c = pend;
            }
            // Skip the rest of the word.
            c = skip_word(buf, c);
        }
        None
    }

    /// GCodeReader.hpp:34: float new_X(const GCodeReader &reader) const
    pub fn new_x(&self, reader: &GCodeReader) -> f32 {
        if self.has(Axis::X) {
            self.x()
        } else {
            reader.x()
        }
    }
    /// GCodeReader.hpp:35
    pub fn new_y(&self, reader: &GCodeReader) -> f32 {
        if self.has(Axis::Y) {
            self.y()
        } else {
            reader.y()
        }
    }
    /// GCodeReader.hpp:36
    pub fn new_z(&self, reader: &GCodeReader) -> f32 {
        if self.has(Axis::Z) {
            self.z()
        } else {
            reader.z()
        }
    }
    /// GCodeReader.hpp:37
    pub fn new_e(&self, reader: &GCodeReader) -> f32 {
        if self.has(Axis::E) {
            self.e()
        } else {
            reader.e()
        }
    }
    /// GCodeReader.hpp:38
    pub fn new_f(&self, reader: &GCodeReader) -> f32 {
        if self.has(Axis::F) {
            self.f()
        } else {
            reader.f()
        }
    }

    /// GCodeReader.hpp:39: float dist_X(const GCodeReader &reader) const
    pub fn dist_x(&self, reader: &GCodeReader) -> f32 {
        if self.has(Axis::X) {
            self.x() - reader.x()
        } else {
            0.0
        }
    }
    /// GCodeReader.hpp:40
    pub fn dist_y(&self, reader: &GCodeReader) -> f32 {
        if self.has(Axis::Y) {
            self.y() - reader.y()
        } else {
            0.0
        }
    }
    /// GCodeReader.hpp:41
    pub fn dist_z(&self, reader: &GCodeReader) -> f32 {
        if self.has(Axis::Z) {
            self.z() - reader.z()
        } else {
            0.0
        }
    }
    /// GCodeReader.hpp:42
    pub fn dist_e(&self, reader: &GCodeReader) -> f32 {
        if self.has(Axis::E) {
            self.e() - reader.e()
        } else {
            0.0
        }
    }

    /// GCodeReader.hpp:43-47
    /// float dist_XY(const GCodeReader &reader) const {
    ///     float x = this->has(X) ? (this->x() - reader.x()) : 0;
    ///     float y = this->has(Y) ? (this->y() - reader.y()) : 0;
    ///     return sqrt(x*x + y*y);
    /// }
    pub fn dist_xy(&self, reader: &GCodeReader) -> f32 {
        let x = if self.has(Axis::X) {
            self.x() - reader.x()
        } else {
            0.0
        };
        let y = if self.has(Axis::Y) {
            self.y() - reader.y()
        } else {
            0.0
        };
        (x * x + y * y).sqrt()
    }

    /// GCodeReader.hpp:48: bool cmd_is(const char *cmd_test) const { return cmd_is(m_raw, cmd_test); }
    pub fn cmd_is(&self, cmd_test: &str) -> bool {
        Self::cmd_is_str(&self.m_raw, cmd_test)
    }

    /// GCodeReader.hpp:50
    /// bool extruding(const GCodeReader &reader) const { return (cmd_is("G1")||cmd_is("G2")||cmd_is("G3")) && dist_E(reader) > 0; }
    // BBS: modify to support G2 and G3
    pub fn extruding(&self, reader: &GCodeReader) -> bool {
        (self.cmd_is("G1") || self.cmd_is("G2") || self.cmd_is("G3")) && self.dist_e(reader) > 0.0
    }

    /// GCodeReader.hpp:51
    pub fn retracting(&self, reader: &GCodeReader) -> bool {
        (self.cmd_is("G1") || self.cmd_is("G2") || self.cmd_is("G3")) && self.dist_e(reader) < 0.0
    }

    /// GCodeReader.hpp:52
    /// bool travel() const { return (cmd_is("G1")||cmd_is("G2")||cmd_is("G3")) && ! has(E); }
    pub fn travel(&self) -> bool {
        (self.cmd_is("G1") || self.cmd_is("G2") || self.cmd_is("G3")) && !self.has(Axis::E)
    }

    /// GCodeReader.hpp:53: void set(const GCodeReader &reader, const Axis axis, const float new_value, const int decimal_digits = 3);
    /// GCodeReader.cpp:278-311
    pub fn set(&mut self, _reader: &GCodeReader, axis: Axis, new_value: f32, decimal_digits: usize) {
        // std::ostringstream ss; ss << std::fixed << std::setprecision(decimal_digits) << new_value;
        let ss = format!("{:.prec$}", new_value, prec = decimal_digits);

        // char match[3] = " X";
        let mut match_buf: [u8; 2] = [b' ', b'X'];
        // if (int(axis) < 3) match[1] += int(axis);
        if axis.index() < 3 {
            match_buf[1] = b'X' + axis.index() as u8;
        } else if axis == Axis::F {
            match_buf[1] = b'F';
        }
        // BBS: handle I and J axis
        else if axis == Axis::I {
            match_buf[1] = b'I';
        } else if axis == Axis::J {
            match_buf[1] = b'J';
        } else {
            // assert(axis == E);
            debug_assert!(axis == Axis::E);
            match_buf[1] = b'E';
        }
        let match_str = std::str::from_utf8(&match_buf).unwrap();

        if self.has(axis) {
            // size_t pos = m_raw.find(match)+2;
            let pos = self.m_raw.find(match_str).unwrap() + 2;
            // size_t end = m_raw.find(' ', pos+1);
            // m_raw.find(' ', pos+1): find first space at or after index pos+1.
            let end = match self.m_raw[pos + 1..].find(' ') {
                Some(off) => pos + 1 + off,
                None => self.m_raw.len(), // std::string::npos behaviour for replace
            };
            // m_raw = m_raw.replace(pos, end-pos, ss.str());
            self.m_raw.replace_range(pos..end, &ss);
        } else {
            // size_t pos = m_raw.find(' ');
            match self.m_raw.find(' ') {
                None => {
                    // m_raw += std::string(match) + ss.str();
                    self.m_raw.push_str(match_str);
                    self.m_raw.push_str(&ss);
                }
                Some(pos) => {
                    // m_raw = m_raw.replace(pos, 0, std::string(match) + ss.str());
                    // replace(pos, 0, X) inserts X at pos.
                    let mut insert = String::with_capacity(match_str.len() + ss.len());
                    insert.push_str(match_str);
                    insert.push_str(&ss);
                    self.m_raw.insert_str(pos, &insert);
                }
            }
        }
        // m_axis[axis] = new_value;
        self.m_axis[axis.index()] = new_value;
        // m_mask |= 1 << int(axis);
        self.m_mask |= 1 << axis.index();
    }

    /// GCodeReader.hpp:55
    pub fn has_x(&self) -> bool {
        self.has(Axis::X)
    }
    /// GCodeReader.hpp:56
    pub fn has_y(&self) -> bool {
        self.has(Axis::Y)
    }
    /// GCodeReader.hpp:57
    pub fn has_z(&self) -> bool {
        self.has(Axis::Z)
    }
    /// GCodeReader.hpp:58
    pub fn has_e(&self) -> bool {
        self.has(Axis::E)
    }
    /// GCodeReader.hpp:59
    pub fn has_f(&self) -> bool {
        self.has(Axis::F)
    }
    // BBS: add I J P axis
    /// GCodeReader.hpp:61
    pub fn has_i(&self) -> bool {
        self.has(Axis::I)
    }
    /// GCodeReader.hpp:62
    pub fn has_j(&self) -> bool {
        self.has(Axis::J)
    }
    /// GCodeReader.hpp:63
    pub fn has_p(&self) -> bool {
        self.has(Axis::P)
    }

    /// GCodeReader.hpp:65: bool has_unknown_axis() const { return this->has(UNKNOWN_AXIS); }
    pub fn has_unknown_axis(&self) -> bool {
        (self.m_mask & (1 << UNKNOWN_AXIS)) != 0
    }

    /// GCodeReader.hpp:66
    pub fn x(&self) -> f32 {
        self.m_axis[Axis::X.index()]
    }
    /// GCodeReader.hpp:67
    pub fn y(&self) -> f32 {
        self.m_axis[Axis::Y.index()]
    }
    /// GCodeReader.hpp:68
    pub fn z(&self) -> f32 {
        self.m_axis[Axis::Z.index()]
    }
    /// GCodeReader.hpp:69
    pub fn e(&self) -> f32 {
        self.m_axis[Axis::E.index()]
    }
    /// GCodeReader.hpp:70
    pub fn f(&self) -> f32 {
        self.m_axis[Axis::F.index()]
    }
    // BBS: add I J P axis
    /// GCodeReader.hpp:72
    pub fn i(&self) -> f32 {
        self.m_axis[Axis::I.index()]
    }
    /// GCodeReader.hpp:73
    pub fn j(&self) -> f32 {
        self.m_axis[Axis::J.index()]
    }
    /// GCodeReader.hpp:74
    pub fn p(&self) -> f32 {
        self.m_axis[Axis::P.index()]
    }

    /// GCodeReader.hpp:76-80
    /// static bool cmd_is(const std::string &gcode_line, const char *cmd_test) {
    ///     const char *cmd = GCodeReader::skip_whitespaces(gcode_line.c_str());
    ///     size_t len = strlen(cmd_test);
    ///     return strncmp(cmd, cmd_test, len) == 0 && GCodeReader::is_end_of_word(cmd[len]);
    /// }
    pub fn cmd_is_str(gcode_line: &str, cmd_test: &str) -> bool {
        let buf = gcode_line.as_bytes();
        let cmd = skip_whitespaces(buf, 0);
        let len = cmd_test.len();
        // strncmp(cmd, cmd_test, len) == 0
        let test = cmd_test.as_bytes();
        for k in 0..len {
            if at(buf, cmd + k) != test[k] {
                return false;
            }
        }
        // && is_end_of_word(cmd[len])
        is_end_of_word(at(buf, cmd + len))
    }

    /// GCodeReader.hpp:82-85
    /// static bool cmd_start_with(const std::string& gcode_line, const char* cmd_test) {
    ///     const char* cmd = GCodeReader::skip_whitespaces(gcode_line.c_str());
    ///     return strncmp(cmd, cmd_test, strlen(cmd_test)) == 0;
    /// }
    pub fn cmd_start_with(gcode_line: &str, cmd_test: &str) -> bool {
        let buf = gcode_line.as_bytes();
        let cmd = skip_whitespaces(buf, 0);
        let test = cmd_test.as_bytes();
        for (k, &t) in test.iter().enumerate() {
            if at(buf, cmd + k) != t {
                return false;
            }
        }
        true
    }
}

impl Default for GCodeLine {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// Callback types (GCodeReader.hpp:94-95)
// ---------------------------------------------------------------------------

/// GCodeReader.hpp:94: typedef std::function<void(GCodeReader&, const GCodeLine&)> callback_t;
pub type CallbackT<'a> = dyn FnMut(&mut GCodeReader, &GCodeLine) + 'a;
/// GCodeReader.hpp:95: typedef std::function<void(GCodeReader&, const char*, const char*)> raw_line_callback_t;
pub type RawLineCallbackT<'a> = dyn FnMut(&mut GCodeReader, &[u8]) + 'a;

// ---------------------------------------------------------------------------
// GCodeReader (GCodeReader.hpp:14-192, GCodeReader.cpp)
// ---------------------------------------------------------------------------

/// G-code reader that parses files or buffers line by line.
/// GCodeReader.hpp:14-192
#[derive(Debug, Clone)]
pub struct GCodeReader {
    /// GCodeReader.hpp:187: GCodeConfig m_config;
    m_config: GCodeConfig,
    /// GCodeReader.hpp:188: float m_position[NUM_AXES];
    m_position: [f32; NUM_AXES],
    /// GCodeReader.hpp:189: bool m_verbose;
    m_verbose: bool,
    /// GCodeReader.hpp:191: bool m_parsing{ false };
    m_parsing: bool,
}

impl GCodeReader {
    /// GCodeReader.hpp:97: GCodeReader() : m_verbose(false) { this->reset(); }
    pub fn new() -> Self {
        let mut r = Self {
            m_config: GCodeConfig::default(),
            m_position: [0.0; NUM_AXES],
            m_verbose: false,
            m_parsing: false,
        };
        r.reset();
        r
    }

    /// GCodeReader.hpp:98: void reset() { memset(m_position, 0, sizeof(m_position)); }
    pub fn reset(&mut self) {
        self.m_position = [0.0; NUM_AXES];
    }

    /// GCodeReader.cpp:19-22: void GCodeReader::apply_config(const GCodeConfig &config)
    pub fn apply_config(&mut self, config: GCodeConfig) {
        self.m_config = config;
    }

    // GCodeReader.cpp:24-27: void GCodeReader::apply_config(const DynamicPrintConfig &config)
    //     { m_config.apply(config, true); }
    // BLOCKED: requires the canonical `DynamicPrintConfig` (ConfigBase::apply) which is not
    // yet ported in this crate (only file-private placeholders exist). Deliberately omitted
    // rather than stubbed; add this overload once DynamicPrintConfig + ConfigBase::apply land.

    /// Set verbose output (m_verbose has no public setter in C++; exposed here as a helper).
    pub fn set_verbose(&mut self, verbose: bool) {
        self.m_verbose = verbose;
    }

    /// GCodeReader.cpp:29-108
    /// const char* GCodeReader::parse_line_internal(const char *ptr, const char *end, GCodeLine &gline, std::pair<const char*, const char*> &command)
    ///
    /// `command` is returned as `(first, second)` byte indices into `buf`.
    fn parse_line_internal(
        &mut self,
        buf: &[u8],
        ptr: usize,
        end: usize,
        gline: &mut GCodeLine,
    ) -> (usize, (usize, usize)) {
        // PROFILE_FUNC();
        // assert(is_decimal_separator_point());

        let mut command: (usize, usize) = (0, 0);

        // command and args
        // const char *c = ptr;
        let mut c = ptr;
        {
            // PROFILE_BLOCK(command_and_args);
            // Skip the whitespaces.
            command.0 = skip_whitespaces(buf, c);
            // Skip the command.
            c = skip_word(buf, command.0);
            command.1 = c;
            // Up to the end of line or comment.
            while !is_end_of_gcode_line(at(buf, c)) {
                // Skip whitespaces.
                c = skip_whitespaces(buf, c);
                if is_end_of_gcode_line(at(buf, c)) {
                    break;
                }
                // Check the name of the axis.
                // Axis axis = NUM_AXES_WITH_UNKNOWN;
                let mut axis: usize = Axis::NumAxesWithUnknown.index();
                match at(buf, c) {
                    b'X' => axis = Axis::X.index(),
                    b'Y' => axis = Axis::Y.index(),
                    b'Z' => axis = Axis::Z.index(),
                    b'F' => axis = Axis::F.index(),
                    // BBS: add I and J axis
                    b'I' => axis = Axis::I.index(),
                    b'J' => axis = Axis::J.index(),
                    b'E' => axis = Axis::E.index(),
                    b'P' => axis = Axis::P.index(),
                    ch => {
                        if (b'A'..=b'Z').contains(&ch) {
                            // Unknown axis, but we still want to remember that such a axis was seen.
                            axis = UNKNOWN_AXIS;
                        }
                    }
                }
                if axis != Axis::NumAxesWithUnknown.index() {
                    // Try to parse the numeric value.
                    // auto [pend, ec] = fast_float::from_chars(++ c, end, v);
                    c += 1;
                    let (v, pend) = from_chars(buf, c, end);
                    if pend != c && is_end_of_word(at(buf, pend)) {
                        // The axis value has been parsed correctly.
                        if axis != UNKNOWN_AXIS {
                            // gline.m_axis[int(axis)] = float(v);
                            gline.m_axis[axis] = v as f32;
                        }
                        // gline.m_mask |= 1 << int(axis);
                        gline.m_mask |= 1 << axis;
                        c = pend;
                    } else {
                        // Skip the rest of the word.
                        c = skip_word(buf, c);
                    }
                } else {
                    // Skip the rest of the word.
                    c = skip_word(buf, c);
                }
            }
        }

        // if (gline.has(E) && m_config.use_relative_e_distances)
        //     m_position[E] = 0;
        if gline.has(Axis::E) && self.m_config.use_relative_e_distances {
            self.m_position[Axis::E.index()] = 0.0;
        }

        // Skip the rest of the line.
        // for (; ! is_end_of_line(*c); ++ c);
        while !is_end_of_line(at(buf, c)) {
            c += 1;
        }

        // Copy the raw string including the comment, without the trailing newlines.
        // if (c > ptr) gline.m_raw.assign(ptr, c);
        if c > ptr {
            // PROFILE_BLOCK(copy_raw_string);
            gline.m_raw = String::from_utf8_lossy(&buf[ptr..c]).into_owned();
        }

        // Skip the trailing newlines.
        // if (*c == '\r') ++ c;
        if at(buf, c) == b'\r' {
            c += 1;
        }
        // if (*c == '\n') ++ c;
        if at(buf, c) == b'\n' {
            c += 1;
        }

        if self.m_verbose {
            // std::cout << gline.m_raw << std::endl;
            println!("{}", gline.m_raw);
        }

        (c, command)
    }

    /// GCodeReader.cpp:110-123
    /// void GCodeReader::update_coordinates(GCodeLine &gline, std::pair<const char*, const char*> &command)
    fn update_coordinates(&mut self, gline: &GCodeLine, buf: &[u8], command: (usize, usize)) {
        // PROFILE_FUNC();
        // if (*command.first == 'G') {
        if at(buf, command.0) == b'G' {
            // int cmd_len = int(command.second - command.first);
            let cmd_len = command.1 - command.0;
            // BBS: add support of G2 and G3
            // if ((cmd_len == 2 && (command.first[1] == '0'||'1'||'2'||'3')) ||
            //     (cmd_len == 3 && command.first[1] == '9' && command.first[2] == '2'))
            if (cmd_len == 2
                && (at(buf, command.0 + 1) == b'0'
                    || at(buf, command.0 + 1) == b'1'
                    || at(buf, command.0 + 1) == b'2'
                    || at(buf, command.0 + 1) == b'3'))
                || (cmd_len == 3 && at(buf, command.0 + 1) == b'9' && at(buf, command.0 + 2) == b'2')
            {
                // for (size_t i = 0; i < NUM_AXES; ++ i)
                for i in 0..NUM_AXES {
                    let ax = axis_from_index(i);
                    // if (gline.has(Axis(i))) m_position[i] = gline.value(Axis(i));
                    if gline.has(ax) {
                        self.m_position[i] = gline.value(ax);
                    }
                }
            }
        }
    }

    /// GCodeReader.hpp:118-126
    /// template<typename Callback>
    /// const char* parse_line(const char *ptr, const char *end, GCodeLine &gline, Callback &callback)
    /// {
    ///     std::pair<const char*, const char*> cmd;
    ///     const char *line_end = parse_line_internal(ptr, end, gline, cmd);
    ///     callback(*this, gline);
    ///     update_coordinates(gline, cmd);
    ///     return line_end;
    /// }
    fn parse_line<F>(
        &mut self,
        buf: &[u8],
        ptr: usize,
        end: usize,
        gline: &mut GCodeLine,
        callback: &mut F,
    ) -> usize
    where
        F: FnMut(&mut GCodeReader, &GCodeLine),
    {
        let (line_end, cmd) = self.parse_line_internal(buf, ptr, end, gline);
        callback(self, gline);
        self.update_coordinates(gline, buf, cmd);
        line_end
    }

    /// GCodeReader.hpp:102-113
    /// template<typename Callback>
    /// void parse_buffer(const std::string &buffer, Callback callback)
    /// {
    ///     const char *ptr = buffer.c_str();
    ///     const char *end = ptr + buffer.size();
    ///     GCodeLine gline;
    ///     m_parsing = true;
    ///     while (m_parsing && *ptr != 0) {
    ///         gline.reset();
    ///         ptr = this->parse_line(ptr, end, gline, callback);
    ///     }
    /// }
    pub fn parse_buffer<F>(&mut self, buffer: &str, mut callback: F)
    where
        F: FnMut(&mut GCodeReader, &GCodeLine),
    {
        let buf = buffer.as_bytes();
        let mut ptr = 0usize;
        let end = buf.len();
        let mut gline = GCodeLine::new();
        self.m_parsing = true;
        // while (m_parsing && *ptr != 0)
        while self.m_parsing && at(buf, ptr) != 0 {
            gline.reset();
            ptr = self.parse_line(buf, ptr, end, &mut gline, &mut callback);
        }
    }

    /// GCodeReader.hpp:115-116
    /// void parse_buffer(const std::string &buffer)
    ///     { this->parse_buffer(buffer, [](GCodeReader&, const GCodeReader::GCodeLine&){}); }
    pub fn parse_buffer_noop(&mut self, buffer: &str) {
        self.parse_buffer(buffer, |_, _| {});
    }

    /// GCodeReader.hpp:128-130
    /// template<typename Callback>
    /// void parse_line(const std::string &line, Callback callback)
    ///     { GCodeLine gline; this->parse_line(line.c_str(), line.c_str() + line.size(), gline, callback); }
    pub fn parse_line_str<F>(&mut self, line: &str, mut callback: F)
    where
        F: FnMut(&mut GCodeReader, &GCodeLine),
    {
        let buf = line.as_bytes();
        let mut gline = GCodeLine::new();
        self.parse_line(buf, 0, buf.len(), &mut gline, &mut callback);
    }

    /// GCodeReader.cpp:125-179
    /// template<typename ParseLineCallback, typename LineEndCallback>
    /// bool GCodeReader::parse_file_raw_internal(const std::string &filename, ParseLineCallback parse_line_callback, LineEndCallback line_end_callback)
    ///
    /// Reads the input stream 64kB-at-a-time, extracts lines and processes them. `parse_line_callback`
    /// receives the byte slice of one logical line (without trailing EOL). `line_end_callback` receives
    /// the absolute byte offset just past each `\n`.
    fn parse_file_raw_internal<P, L>(
        &mut self,
        filename: &str,
        mut parse_line_callback: P,
        mut line_end_callback: L,
    ) -> std::io::Result<bool>
    where
        P: FnMut(&mut GCodeReader, &[u8]),
        L: FnMut(usize),
    {
        use std::io::Read;
        // FilePtr in{ boost::nowide::fopen(filename.c_str(), "rb") };
        let mut file = match std::fs::File::open(filename) {
            Ok(f) => f,
            // Original C++ does not check the FILE* before fread; a NULL FILE* would crash.
            // We treat open failure as a hard error returned to the caller below via Err.
            Err(e) => return Err(e),
        };

        // Read the input stream 64kB at a time, extract lines and process them.
        // std::vector<char> buffer(65536 * 10, 0);
        let mut buffer = vec![0u8; 65536 * 10];
        // Line buffer.
        // std::string gcode_line;
        let mut gcode_line: Vec<u8> = Vec::new();
        // size_t file_pos = 0;
        let mut file_pos: usize = 0;
        self.m_parsing = true;
        // for (;;)
        loop {
            // size_t cnt_read = ::fread(buffer.data(), 1, buffer.size(), in.f);
            let cnt_read = match file.read(&mut buffer) {
                Ok(n) => n,
                // if (::ferror(in.f)) return false;
                Err(_) => return Ok(false),
            };
            // bool eof = cnt_read == 0;
            let eof = cnt_read == 0;
            // auto it = buffer.begin();
            let mut it = 0usize;
            // auto it_bufend = buffer.begin() + cnt_read;
            let it_bufend = cnt_read;
            // while (it != it_bufend || (eof && ! gcode_line.empty()))
            while it != it_bufend || (eof && !gcode_line.is_empty()) {
                // Find end of line.
                // bool eol = false; auto it_end = it;
                let mut eol = false;
                let mut it_end = it;
                // for (; it_end != it_bufend && ! (eol = *it_end == '\r' || *it_end == '\n'); ++ it_end)
                //     if (*it_end == '\n')
                //         line_end_callback(file_pos + (it_end - buffer.begin()) + 1);
                while it_end != it_bufend && {
                    eol = buffer[it_end] == b'\r' || buffer[it_end] == b'\n';
                    !eol
                } {
                    if buffer[it_end] == b'\n' {
                        line_end_callback(file_pos + it_end + 1);
                    }
                    it_end += 1;
                }
                // End of line is indicated also if end of file was reached.
                // eol |= eof && it_end == it_bufend;
                eol |= eof && it_end == it_bufend;
                if eol {
                    // if (gcode_line.empty())
                    //     parse_line_callback(&(*it), &(*it_end));
                    if gcode_line.is_empty() {
                        let slice = buffer[it..it_end].to_vec();
                        parse_line_callback(self, &slice);
                    } else {
                        // gcode_line.insert(gcode_line.end(), it, it_end);
                        gcode_line.extend_from_slice(&buffer[it..it_end]);
                        // parse_line_callback(gcode_line.c_str(), gcode_line.c_str() + gcode_line.size());
                        let slice = gcode_line.clone();
                        parse_line_callback(self, &slice);
                        // gcode_line.clear();
                        gcode_line.clear();
                    }
                    // if (! m_parsing) return true;  // The callback wishes to exit.
                    if !self.m_parsing {
                        return Ok(true);
                    }
                } else {
                    // gcode_line.insert(gcode_line.end(), it, it_end);
                    gcode_line.extend_from_slice(&buffer[it..it_end]);
                }
                // Skip EOL.
                // it = it_end;
                it = it_end;
                // if (it != it_bufend && *it == '\r') ++ it;
                if it != it_bufend && buffer[it] == b'\r' {
                    it += 1;
                }
                // if (it != it_bufend && *it == '\n') { line_end_callback(file_pos + (it - buffer.begin()) + 1); ++ it; }
                if it != it_bufend && buffer[it] == b'\n' {
                    line_end_callback(file_pos + it + 1);
                    it += 1;
                }
            }
            // if (eof) break;
            if eof {
                break;
            }
            // file_pos += cnt_read;
            file_pos += cnt_read;
        }
        Ok(true)
    }

    /// GCodeReader.cpp:181-197
    /// template<typename ParseLineCallback, typename LineEndCallback>
    /// bool GCodeReader::parse_file_internal(const std::string &filename, ParseLineCallback parse_line_callback, LineEndCallback line_end_callback)
    fn parse_file_internal<F, L>(
        &mut self,
        filename: &str,
        mut parse_line_callback: F,
        line_end_callback: L,
    ) -> std::io::Result<bool>
    where
        F: FnMut(&mut GCodeReader, &GCodeLine),
        L: FnMut(usize),
    {
        // GCodeLine gline;
        let mut gline = GCodeLine::new();
        self.parse_file_raw_internal(
            filename,
            // [this, &gline, parse_line_callback](const char *begin, const char *end) { ... }
            |reader, line| {
                // gline.reset();
                gline.reset();
                // const char* begin_new = begin;
                let mut begin_new = 0usize;
                // begin_new = skip_whitespaces(begin_new);
                begin_new = skip_whitespaces(line, begin_new);
                // if (std::toupper(*begin_new) == 'N') begin_new = skip_word(begin_new);
                if at(line, begin_new).to_ascii_uppercase() == b'N' {
                    begin_new = skip_word(line, begin_new);
                }
                // begin_new = skip_whitespaces(begin_new);
                begin_new = skip_whitespaces(line, begin_new);
                // this->parse_line(begin_new, end, gline, parse_line_callback);
                reader.parse_line(line, begin_new, line.len(), &mut gline, &mut parse_line_callback);
            },
            line_end_callback,
        )
    }

    /// GCodeReader.cpp:199-206
    /// bool GCodeReader::parse_file(const std::string &file, callback_t callback)
    pub fn parse_file<F>(&mut self, file: &str, callback: F) -> std::io::Result<bool>
    where
        F: FnMut(&mut GCodeReader, &GCodeLine),
    {
        // BOOST_LOG_TRIVIAL(info) << ... before parse_file ...
        let ret = self.parse_file_internal(file, callback, |_| {});
        // BOOST_LOG_TRIVIAL(info) << ... finished parse_file ...
        ret
    }

    /// GCodeReader.cpp:208-216
    /// bool GCodeReader::parse_file(const std::string &file, callback_t callback, std::vector<size_t> &lines_ends)
    pub fn parse_file_with_line_ends<F>(
        &mut self,
        file: &str,
        callback: F,
        lines_ends: &mut Vec<usize>,
    ) -> std::io::Result<bool>
    where
        F: FnMut(&mut GCodeReader, &GCodeLine),
    {
        // lines_ends.clear();
        lines_ends.clear();
        // auto ret = parse_file_internal(file, callback, [&lines_ends](size_t file_pos){ lines_ends.emplace_back(file_pos); });
        let ret = self.parse_file_internal(file, callback, |file_pos| {
            lines_ends.push(file_pos);
        });
        ret
    }

    /// GCodeReader.cpp:218-223
    /// bool GCodeReader::parse_file_raw(const std::string &filename, raw_line_callback_t line_callback)
    pub fn parse_file_raw<F>(&mut self, filename: &str, mut line_callback: F) -> std::io::Result<bool>
    where
        F: FnMut(&mut GCodeReader, &[u8]),
    {
        self.parse_file_raw_internal(
            filename,
            // [this, line_callback](const char *begin, const char *end) { line_callback(*this, begin, end); }
            |reader, line| line_callback(reader, line),
            // [](size_t){}
            |_| {},
        )
    }

    /// GCodeReader.hpp:141: void quit_parsing() { m_parsing = false; }
    pub fn quit_parsing(&mut self) {
        self.m_parsing = false;
    }

    /// GCodeReader.hpp:143-144
    pub fn x(&self) -> f32 {
        self.m_position[Axis::X.index()]
    }
    /// GCodeReader.hpp:143
    pub fn x_mut(&mut self) -> &mut f32 {
        &mut self.m_position[Axis::X.index()]
    }
    /// GCodeReader.hpp:145-146
    pub fn y(&self) -> f32 {
        self.m_position[Axis::Y.index()]
    }
    /// GCodeReader.hpp:145
    pub fn y_mut(&mut self) -> &mut f32 {
        &mut self.m_position[Axis::Y.index()]
    }
    /// GCodeReader.hpp:147-148
    pub fn z(&self) -> f32 {
        self.m_position[Axis::Z.index()]
    }
    /// GCodeReader.hpp:147
    pub fn z_mut(&mut self) -> &mut f32 {
        &mut self.m_position[Axis::Z.index()]
    }
    /// GCodeReader.hpp:149-150
    pub fn e(&self) -> f32 {
        self.m_position[Axis::E.index()]
    }
    /// GCodeReader.hpp:149
    pub fn e_mut(&mut self) -> &mut f32 {
        &mut self.m_position[Axis::E.index()]
    }
    /// GCodeReader.hpp:151-152
    pub fn f(&self) -> f32 {
        self.m_position[Axis::F.index()]
    }
    /// GCodeReader.hpp:151
    pub fn f_mut(&mut self) -> &mut f32 {
        &mut self.m_position[Axis::F.index()]
    }
    // BBS: add I J axis
    /// GCodeReader.hpp:154-155
    pub fn i(&self) -> f32 {
        self.m_position[Axis::I.index()]
    }
    /// GCodeReader.hpp:154
    pub fn i_mut(&mut self) -> &mut f32 {
        &mut self.m_position[Axis::I.index()]
    }
    /// GCodeReader.hpp:156-157
    pub fn j(&self) -> f32 {
        self.m_position[Axis::J.index()]
    }
    /// GCodeReader.hpp:156
    pub fn j_mut(&mut self) -> &mut f32 {
        &mut self.m_position[Axis::J.index()]
    }

    /// GCodeReader.hpp:159: static bool is_whitespace(char c)
    pub fn is_whitespace(c: u8) -> bool {
        is_whitespace(c)
    }
    /// GCodeReader.hpp:160: static bool is_end_of_line(char c)
    pub fn is_end_of_line(c: u8) -> bool {
        is_end_of_line(c)
    }
    /// GCodeReader.hpp:161: static bool is_end_of_gcode_line(char c)
    pub fn is_end_of_gcode_line(c: u8) -> bool {
        is_end_of_gcode_line(c)
    }
    /// GCodeReader.hpp:162: static bool is_end_of_word(char c)
    pub fn is_end_of_word(c: u8) -> bool {
        is_end_of_word(c)
    }

    /// GCodeReader.hpp:174-177: GCodeConfig get_config() const { return m_config; }
    pub fn get_config(&self) -> GCodeConfig {
        self.m_config.clone()
    }
}

impl Default for GCodeReader {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// Numeric parsing helpers mirroring fast_float::from_chars / strtod
// ---------------------------------------------------------------------------

/// Index of `axis i` (0..NUM_AXES) as an `Axis`, matching C++ `Axis(i)`.
#[inline]
fn axis_from_index(i: usize) -> Axis {
    match i {
        0 => Axis::X,
        1 => Axis::Y,
        2 => Axis::Z,
        3 => Axis::E,
        4 => Axis::F,
        5 => Axis::I,
        6 => Axis::J,
        7 => Axis::P,
        _ => unreachable!(),
    }
}

/// fast_float::from_chars(begin, end, v) over `buf[begin..end]`.
///
/// Returns the parsed value and the index `pend` one past the last consumed character.
/// On failure `pend == begin` and the value is 0.0 (the C++ caller only consults `pend`
/// and `ec` indirectly via the `pend != c` test). Matches fast_float semantics: parses
/// an optional sign, integer/fraction with '.' decimal point, and exponent.
fn from_chars(buf: &[u8], begin: usize, end: usize) -> (f64, usize) {
    let mut i = begin;
    let lim = end.min(buf.len());
    let start = i;

    // optional sign
    if i < lim && (buf[i] == b'+' || buf[i] == b'-') {
        i += 1;
    }

    let mut any_digits = false;
    // integer part
    while i < lim && buf[i].is_ascii_digit() {
        i += 1;
        any_digits = true;
    }
    // fraction part
    if i < lim && buf[i] == b'.' {
        i += 1;
        while i < lim && buf[i].is_ascii_digit() {
            i += 1;
            any_digits = true;
        }
    }
    if !any_digits {
        // No mantissa digits: parse failure, pend == begin.
        return (0.0, begin);
    }
    // exponent part
    if i < lim && (buf[i] == b'e' || buf[i] == b'E') {
        let mut j = i + 1;
        if j < lim && (buf[j] == b'+' || buf[j] == b'-') {
            j += 1;
        }
        let mut exp_digits = false;
        while j < lim && buf[j].is_ascii_digit() {
            j += 1;
            exp_digits = true;
        }
        // Only consume the exponent if it has digits (fast_float requires digits after 'e').
        if exp_digits {
            i = j;
        }
    }

    let s = std::str::from_utf8(&buf[start..i]).unwrap_or("");
    match s.parse::<f64>() {
        Ok(v) => (v, i),
        Err(_) => (0.0, begin),
    }
}

/// strtod(begin, &pend) over `buf` starting at `begin`.
///
/// Returns the parsed value and `pend` (index one past the last consumed character).
/// Reads the (implicitly NUL-terminated) string until the longest valid floating literal
/// ends. On no conversion `pend == begin`.
fn strtod(buf: &[u8], begin: usize) -> (f64, usize) {
    let mut i = begin;
    let n = buf.len();
    // strtod skips leading whitespace.
    while i < n && (buf[i] == b' ' || buf[i] == b'\t' || buf[i] == b'\n' || buf[i] == b'\r') {
        i += 1;
    }
    let mant_start = i;

    // optional sign
    if i < n && (buf[i] == b'+' || buf[i] == b'-') {
        i += 1;
    }

    let mut any_digits = false;
    while i < n && buf[i].is_ascii_digit() {
        i += 1;
        any_digits = true;
    }
    if i < n && buf[i] == b'.' {
        i += 1;
        while i < n && buf[i].is_ascii_digit() {
            i += 1;
            any_digits = true;
        }
    }
    if !any_digits {
        // No conversion performed: pend == begin (strtod sets endptr to nptr).
        return (0.0, begin);
    }
    if i < n && (buf[i] == b'e' || buf[i] == b'E') {
        let mut j = i + 1;
        if j < n && (buf[j] == b'+' || buf[j] == b'-') {
            j += 1;
        }
        let mut exp_digits = false;
        while j < n && buf[j].is_ascii_digit() {
            j += 1;
            exp_digits = true;
        }
        if exp_digits {
            i = j;
        }
    }

    let s = std::str::from_utf8(&buf[mant_start..i]).unwrap_or("");
    match s.parse::<f64>() {
        Ok(v) => (v, i),
        Err(_) => (0.0, begin),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_axis_index() {
        assert_eq!(Axis::X.index(), 0);
        assert_eq!(Axis::P.index(), 7);
        assert_eq!(NUM_AXES, 8);
        assert_eq!(UNKNOWN_AXIS, 8);
    }

    #[test]
    fn test_cmd_extraction() {
        let mut line = GCodeLine::new();
        line.m_raw = "G1 X10 Y20".to_string();
        assert_eq!(line.cmd(), "G1");
        line.m_raw = "M104 S200".to_string();
        assert_eq!(line.cmd(), "M104");
        line.m_raw = "  G28  ".to_string();
        assert_eq!(line.cmd(), "G28");
    }

    #[test]
    fn test_comment() {
        let mut line = GCodeLine::new();
        line.m_raw = "G1 X10 ; move to X10".to_string();
        assert_eq!(line.comment(), " move to X10");
        line.m_raw = "G28".to_string();
        assert_eq!(line.comment(), "");
    }

    #[test]
    fn test_cmd_is_str() {
        assert!(GCodeLine::cmd_is_str("G1 X10", "G1"));
        assert!(GCodeLine::cmd_is_str("  G28  ", "G28"));
        assert!(!GCodeLine::cmd_is_str("G10 X5", "G1"));
        assert!(GCodeLine::cmd_is_str("M104", "M104"));
    }

    #[test]
    fn test_cmd_start_with() {
        assert!(GCodeLine::cmd_start_with("G1 X10", "G1"));
        assert!(GCodeLine::cmd_start_with("G10 X5", "G1"));
        assert!(!GCodeLine::cmd_start_with("M104", "G"));
    }

    #[test]
    fn test_parse_buffer_count() {
        let mut reader = GCodeReader::new();
        let gcode = "G1 X10 Y20\nG1 Z5 E2.5\nG28";
        let mut n = 0;
        reader.parse_buffer(gcode, |_, _| n += 1);
        assert_eq!(n, 3);
    }

    #[test]
    fn test_position_tracking() {
        let mut reader = GCodeReader::new();
        let gcode = "G1 X10 Y20\nG1 X15 Y25 Z5";
        reader.parse_buffer(gcode, |_, _| {});
        assert_eq!(reader.x(), 15.0);
        assert_eq!(reader.y(), 25.0);
        assert_eq!(reader.z(), 5.0);
    }

    #[test]
    fn test_parse_line_axes() {
        let mut reader = GCodeReader::new();
        let mut got: Option<(f32, f32, f32)> = None;
        reader.parse_line_str("G1 X100.5 Y-50.25 F3000", |_, g| {
            got = Some((g.x(), g.y(), g.f()));
            assert!(g.has(Axis::X));
            assert!(g.has(Axis::Y));
            assert!(g.has(Axis::F));
        });
        let (x, y, f) = got.unwrap();
        assert_eq!(x, 100.5);
        assert_eq!(y, -50.25);
        assert_eq!(f, 3000.0);
    }

    #[test]
    fn test_unknown_axis_marked() {
        let mut reader = GCodeReader::new();
        reader.parse_line_str("M104 S200", |_, g| {
            // S is an unknown axis: it must be flagged but not stored in m_axis.
            assert!(g.has_unknown_axis());
            assert!(!g.has(Axis::E));
        });
    }

    #[test]
    fn test_arc_commands() {
        let mut reader = GCodeReader::new();
        reader.parse_line_str("G2 X10 Y10 I5 J0 E2", |_, g| {
            assert_eq!(g.cmd(), "G2");
            assert!(g.has(Axis::I));
            assert!(g.has(Axis::J));
            assert_eq!(g.i(), 5.0);
            assert_eq!(g.j(), 0.0);
        });
    }

    #[test]
    fn test_quit_parsing() {
        let mut reader = GCodeReader::new();
        let gcode = "G1 X10\nG1 X20\nG1 X30";
        let mut count = 0;
        reader.parse_buffer(gcode, |r, _| {
            count += 1;
            if count >= 2 {
                r.quit_parsing();
            }
        });
        assert_eq!(count, 2);
    }

    #[test]
    fn test_set_replace_existing() {
        let mut reader = GCodeReader::new();
        let mut gline = GCodeLine::new();
        reader.parse_line_str("G1 X10 Y20", |_, _| {});
        gline.m_raw = "G1 X10 Y20".to_string();
        gline.m_mask = (1 << Axis::X.index()) | (1 << Axis::Y.index());
        gline.m_axis[Axis::X.index()] = 10.0;
        gline.m_axis[Axis::Y.index()] = 20.0;
        gline.set(&reader, Axis::X, 12.5, 3);
        assert_eq!(gline.raw(), "G1 X12.500 Y20");
        assert_eq!(gline.x(), 12.5);
    }

    #[test]
    fn test_set_add_new() {
        let reader = GCodeReader::new();
        let mut gline = GCodeLine::new();
        gline.m_raw = "G1 X10".to_string();
        gline.m_mask = 1 << Axis::X.index();
        gline.m_axis[Axis::X.index()] = 10.0;
        gline.set(&reader, Axis::E, 1.25, 3);
        // " E1.250" inserted at first space.
        assert_eq!(gline.raw(), "G1 E1.250 X10");
        assert!(gline.has(Axis::E));
    }

    #[test]
    fn test_has_value() {
        let mut gline = GCodeLine::new();
        gline.m_raw = "G1 X10 Y20.5".to_string();
        assert_eq!(gline.has_value(b'Y'), Some(20.5));
        assert_eq!(gline.has_value(b'Z'), None);
    }

    #[test]
    fn test_relative_e() {
        let mut reader = GCodeReader::new();
        let mut cfg = GCodeConfig::default();
        cfg.use_relative_e_distances = true;
        reader.apply_config(cfg);
        *reader.e_mut() = 10.0;
        reader.parse_line_str("G1 E2", |_, _| {});
        // Relative E: position reset to 0 inside parse_line_internal, then update_coordinates
        // sets it to the parsed value 2.0.
        assert_eq!(reader.e(), 2.0);
    }
}
