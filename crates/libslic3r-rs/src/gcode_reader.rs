//! G-code parser for reading and interpreting G-code files line by line.
//!
//! C++ Reference:
//! - GCodeReader.hpp
//! - GCodeReader.cpp
//!
//! This module provides a streaming G-code parser that can read files or buffers,
//! parse individual lines, extract axis values, and maintain current position state.

use crate::{Error, Result};
use std::collections::HashMap;
use std::fs::File;
use std::io::{BufRead, BufReader, Read};
use std::path::Path;

/// Axis identifiers for G-code commands
/// libslic3r.h:114-128
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum Axis {
    X = 0,
    Y = 1,
    Z = 2,
    E = 3,
    F = 4,
    // BBS: add I, J, P axis for arc commands
    I = 5,
    J = 6,
    P = 7,
}

impl Axis {
    /// Total number of standard axes
    /// libslic3r.h:123
    pub const NUM_AXES: usize = 8;

    /// Convert axis to character representation
    /// GCodeReader.cpp:279-297
    pub fn to_char(self) -> char {
        match self {
            Axis::X => 'X',
            Axis::Y => 'Y',
            Axis::Z => 'Z',
            Axis::E => 'E',
            Axis::F => 'F',
            Axis::I => 'I',
            Axis::J => 'J',
            Axis::P => 'P',
        }
    }

    /// Parse axis from character
    /// GCodeReader.cpp:53-66
    pub fn from_char(c: char) -> Option<Self> {
        match c {
            'X' => Some(Axis::X),
            'Y' => Some(Axis::Y),
            'Z' => Some(Axis::Z),
            'E' => Some(Axis::E),
            'F' => Some(Axis::F),
            'I' => Some(Axis::I),
            'J' => Some(Axis::J),
            'P' => Some(Axis::P),
            _ => None,
        }
    }
}

/// A single parsed line of G-code
/// GCodeReader.hpp:13-82
#[derive(Debug, Clone)]
pub struct GCodeLine {
    /// Raw G-code line text
    /// GCodeReader.hpp:80
    raw: String,
    /// Axis values present on this line
    /// GCodeReader.hpp:81
    axis: [f32; Axis::NUM_AXES],
    /// Bitmask indicating which axes are present (bit i set = axis i present)
    /// GCodeReader.hpp:82
    mask: u32,
}

impl GCodeLine {
    /// Create a new empty G-code line
    /// GCodeReader.hpp:15
    pub fn new() -> Self {
        Self {
            raw: String::new(),
            axis: [0.0; Axis::NUM_AXES],
            mask: 0,
        }
    }

    /// Reset line to initial empty state
    /// GCodeReader.hpp:16
    pub fn reset(&mut self) {
        self.mask = 0;
        self.axis = [0.0; Axis::NUM_AXES];
        self.raw.clear();
    }

    /// Get the raw G-code line text
    /// GCodeReader.hpp:18
    pub fn raw(&self) -> &str {
        &self.raw
    }

    /// Extract the command portion (e.g., "G1", "M104")
    /// GCodeReader.hpp:19-22
    pub fn cmd(&self) -> &str {
        let trimmed = self.raw.trim_start();
        let end = trimmed
            .find(|c: char| c.is_whitespace() || c == ';' || c == '\r' || c == '\n')
            .unwrap_or(trimmed.len());
        &trimmed[..end]
    }

    /// Extract the comment portion (after ';')
    /// GCodeReader.hpp:23-24
    pub fn comment(&self) -> &str {
        if let Some(pos) = self.raw.find(';') {
            &self.raw[pos + 1..]
        } else {
            ""
        }
    }

    /// Clear the raw line text
    /// GCodeReader.hpp:26
    pub fn clear(&mut self) {
        self.raw.clear();
    }

    /// Check if an axis value is present on this line
    /// GCodeReader.hpp:27
    pub fn has(&self, axis: Axis) -> bool {
        (self.mask & (1 << axis as u8)) != 0
    }

    /// Get the value for an axis (returns 0.0 if not present)
    /// GCodeReader.hpp:28
    pub fn value(&self, axis: Axis) -> f32 {
        self.axis[axis as usize]
    }

    /// Check if axis character is present by scanning raw string
    /// GCodeReader.cpp:231-253
    pub fn has_char(&self, axis_char: char) -> bool {
        let bytes = self.raw.as_bytes();
        let mut i = 0;

        // Skip whitespaces
        while i < bytes.len() && (bytes[i] == b' ' || bytes[i] == b'\t') {
            i += 1;
        }

        // Skip command
        while i < bytes.len() && !Self::is_end_of_word(bytes[i] as char) {
            i += 1;
        }

        // Scan for axis
        while i < bytes.len() && !Self::is_end_of_gcode_line(bytes[i] as char) {
            // Skip whitespaces
            while i < bytes.len() && (bytes[i] == b' ' || bytes[i] == b'\t') {
                i += 1;
            }
            if i >= bytes.len() || Self::is_end_of_gcode_line(bytes[i] as char) {
                break;
            }
            // Check axis name
            if bytes[i] as char == axis_char {
                return true;
            }
            // Skip rest of word
            while i < bytes.len() && !Self::is_end_of_word(bytes[i] as char) {
                i += 1;
            }
        }
        false
    }

    /// Get the new X value, using reader's current if not present
    /// GCodeReader.hpp:29
    pub fn new_x(&self, reader: &GCodeReader) -> f32 {
        if self.has(Axis::X) {
            self.x()
        } else {
            reader.x()
        }
    }

    /// Get the new Y value, using reader's current if not present
    /// GCodeReader.hpp:30
    pub fn new_y(&self, reader: &GCodeReader) -> f32 {
        if self.has(Axis::Y) {
            self.y()
        } else {
            reader.y()
        }
    }

    /// Get the new Z value, using reader's current if not present
    /// GCodeReader.hpp:31
    pub fn new_z(&self, reader: &GCodeReader) -> f32 {
        if self.has(Axis::Z) {
            self.z()
        } else {
            reader.z()
        }
    }

    /// Get the new E value, using reader's current if not present
    /// GCodeReader.hpp:32
    pub fn new_e(&self, reader: &GCodeReader) -> f32 {
        if self.has(Axis::E) {
            self.e()
        } else {
            reader.e()
        }
    }

    /// Get the new F value, using reader's current if not present
    /// GCodeReader.hpp:33
    pub fn new_f(&self, reader: &GCodeReader) -> f32 {
        if self.has(Axis::F) {
            self.f()
        } else {
            reader.f()
        }
    }

    /// Get the distance moved in X
    /// GCodeReader.hpp:34
    pub fn dist_x(&self, reader: &GCodeReader) -> f32 {
        if self.has(Axis::X) {
            self.x() - reader.x()
        } else {
            0.0
        }
    }

    /// Get the distance moved in Y
    /// GCodeReader.hpp:35
    pub fn dist_y(&self, reader: &GCodeReader) -> f32 {
        if self.has(Axis::Y) {
            self.y() - reader.y()
        } else {
            0.0
        }
    }

    /// Get the distance moved in Z
    /// GCodeReader.hpp:36
    pub fn dist_z(&self, reader: &GCodeReader) -> f32 {
        if self.has(Axis::Z) {
            self.z() - reader.z()
        } else {
            0.0
        }
    }

    /// Get the distance extruded in E
    /// GCodeReader.hpp:37
    pub fn dist_e(&self, reader: &GCodeReader) -> f32 {
        if self.has(Axis::E) {
            self.e() - reader.e()
        } else {
            0.0
        }
    }

    /// Get the distance moved in XY plane
    /// GCodeReader.hpp:38-42
    pub fn dist_xy(&self, reader: &GCodeReader) -> f32 {
        let x = self.dist_x(reader);
        let y = self.dist_y(reader);
        (x * x + y * y).sqrt()
    }

    /// Check if command matches given string
    /// GCodeReader.hpp:43
    pub fn cmd_is(&self, cmd_test: &str) -> bool {
        Self::cmd_is_str(&self.raw, cmd_test)
    }

    /// Check if this is an extrusion move (G1/G2/G3 with positive E)
    /// GCodeReader.hpp:45
    pub fn extruding(&self, reader: &GCodeReader) -> bool {
        (self.cmd_is("G1") || self.cmd_is("G2") || self.cmd_is("G3")) && self.dist_e(reader) > 0.0
    }

    /// Check if this is a retraction (G1/G2/G3 with negative E)
    /// GCodeReader.hpp:46
    pub fn retracting(&self, reader: &GCodeReader) -> bool {
        (self.cmd_is("G1") || self.cmd_is("G2") || self.cmd_is("G3")) && self.dist_e(reader) < 0.0
    }

    /// Check if this is a travel move (G1/G2/G3 without E)
    /// GCodeReader.hpp:47
    pub fn travel(&self) -> bool {
        (self.cmd_is("G1") || self.cmd_is("G2") || self.cmd_is("G3")) && !self.has(Axis::E)
    }

    /// Set an axis value and update the raw string
    /// GCodeReader.cpp:298-313
    pub fn set(
        &mut self,
        _reader: &GCodeReader,
        axis: Axis,
        new_value: f32,
        decimal_digits: usize,
    ) {
        let value_str = format!("{:.prec$}", new_value, prec = decimal_digits);
        let axis_char = axis.to_char();
        let match_str = format!(" {}", axis_char);

        if self.has(axis) {
            // Replace existing value
            if let Some(pos) = self.raw.find(&match_str) {
                let start = pos + 2;
                let end = self.raw[start..]
                    .find(' ')
                    .map(|p| start + p)
                    .unwrap_or(self.raw.len());
                self.raw.replace_range(start..end, &value_str);
            }
        } else {
            // Add new axis value
            if let Some(pos) = self.raw.find(' ') {
                self.raw
                    .insert_str(pos, &format!("{}{}", match_str, value_str));
            } else {
                self.raw.push_str(&format!("{}{}", match_str, value_str));
            }
        }

        self.axis[axis as usize] = new_value;
        self.mask |= 1 << axis as u8;
    }

    // Axis accessor methods
    /// Check if X is present
    /// GCodeReader.hpp:49
    pub fn has_x(&self) -> bool {
        self.has(Axis::X)
    }
    /// Check if Y is present
    /// GCodeReader.hpp:50
    pub fn has_y(&self) -> bool {
        self.has(Axis::Y)
    }
    /// Check if Z is present
    /// GCodeReader.hpp:51
    pub fn has_z(&self) -> bool {
        self.has(Axis::Z)
    }
    /// Check if E is present
    /// GCodeReader.hpp:52
    pub fn has_e(&self) -> bool {
        self.has(Axis::E)
    }
    /// Check if F is present
    /// GCodeReader.hpp:53
    pub fn has_f(&self) -> bool {
        self.has(Axis::F)
    }
    /// Check if I is present
    /// GCodeReader.hpp:55
    pub fn has_i(&self) -> bool {
        self.has(Axis::I)
    }
    /// Check if J is present
    /// GCodeReader.hpp:56
    pub fn has_j(&self) -> bool {
        self.has(Axis::J)
    }
    /// Check if P is present
    /// GCodeReader.hpp:57
    pub fn has_p(&self) -> bool {
        self.has(Axis::P)
    }

    /// Get X value
    /// GCodeReader.hpp:60
    pub fn x(&self) -> f32 {
        self.axis[Axis::X as usize]
    }
    /// Get Y value
    /// GCodeReader.hpp:61
    pub fn y(&self) -> f32 {
        self.axis[Axis::Y as usize]
    }
    /// Get Z value
    /// GCodeReader.hpp:62
    pub fn z(&self) -> f32 {
        self.axis[Axis::Z as usize]
    }
    /// Get E value
    /// GCodeReader.hpp:63
    pub fn e(&self) -> f32 {
        self.axis[Axis::E as usize]
    }
    /// Get F value
    /// GCodeReader.hpp:64
    pub fn f(&self) -> f32 {
        self.axis[Axis::F as usize]
    }
    /// Get I value
    /// GCodeReader.hpp:66
    pub fn i(&self) -> f32 {
        self.axis[Axis::I as usize]
    }
    /// Get J value
    /// GCodeReader.hpp:67
    pub fn j(&self) -> f32 {
        self.axis[Axis::J as usize]
    }
    /// Get P value
    /// GCodeReader.hpp:68
    pub fn p(&self) -> f32 {
        self.axis[Axis::P as usize]
    }

    /// Check if command in string matches test string
    /// GCodeReader.hpp:70-74
    pub fn cmd_is_str(gcode_line: &str, cmd_test: &str) -> bool {
        let trimmed = gcode_line.trim_start();
        trimmed.starts_with(cmd_test)
            && (trimmed.len() == cmd_test.len()
                || Self::is_end_of_word(trimmed.chars().nth(cmd_test.len()).unwrap_or('\0')))
    }

    /// Check if command starts with test string
    /// GCodeReader.hpp:76-79
    pub fn cmd_start_with(gcode_line: &str, cmd_test: &str) -> bool {
        let trimmed = gcode_line.trim_start();
        trimmed.starts_with(cmd_test)
    }

    // Helper character classification methods
    fn is_whitespace(c: char) -> bool {
        c == ' ' || c == '\t'
    }

    fn is_end_of_line(c: char) -> bool {
        c == '\r' || c == '\n' || c == '\0'
    }

    fn is_end_of_gcode_line(c: char) -> bool {
        c == ';' || Self::is_end_of_line(c)
    }

    fn is_end_of_word(c: char) -> bool {
        Self::is_whitespace(c) || Self::is_end_of_gcode_line(c)
    }
}

impl Default for GCodeLine {
    fn default() -> Self {
        Self::new()
    }
}

/// Callback type for processing parsed G-code lines
/// GCodeReader.hpp:85
pub type Callback = Box<dyn FnMut(&mut GCodeReader, &GCodeLine)>;

/// Callback type for processing raw G-code line strings
/// GCodeReader.hpp:86
pub type RawLineCallback = Box<dyn FnMut(&mut GCodeReader, &str)>;

/// G-code reader that parses files or buffers line by line
/// GCodeReader.hpp:88-165
#[derive(Debug)]
pub struct GCodeReader {
    /// Current position state
    /// GCodeReader.hpp:159
    position: [f32; Axis::NUM_AXES],
    /// Configuration settings
    /// GCodeReader.hpp:158
    config: GCodeConfig,
    /// Verbose output flag
    /// GCodeReader.hpp:160
    verbose: bool,
    /// Parsing control flag (set to false by callback to stop)
    /// GCodeReader.hpp:162
    parsing: bool,
}

/// G-code configuration settings
/// PrintConfig.hpp (partial - simplified for this port)
#[derive(Debug, Clone)]
pub struct GCodeConfig {
    /// Use relative E distances
    pub use_relative_e_distances: bool,
}

impl Default for GCodeConfig {
    fn default() -> Self {
        Self {
            use_relative_e_distances: false,
        }
    }
}

impl GCodeReader {
    /// Create a new G-code reader
    /// GCodeReader.hpp:88
    pub fn new() -> Self {
        Self {
            position: [0.0; Axis::NUM_AXES],
            config: GCodeConfig::default(),
            verbose: false,
            parsing: false,
        }
    }

    /// Reset position state to origin
    /// GCodeReader.hpp:89
    pub fn reset(&mut self) {
        self.position = [0.0; Axis::NUM_AXES];
    }

    /// Apply configuration
    /// GCodeReader.cpp:17-20
    pub fn apply_config(&mut self, config: GCodeConfig) {
        self.config = config;
    }

    /// Set verbose output mode
    pub fn set_verbose(&mut self, verbose: bool) {
        self.verbose = verbose;
    }

    /// Parse a buffer of G-code
    /// GCodeReader.hpp:93-101
    pub fn parse_buffer<F>(&mut self, buffer: &str, mut callback: F)
    where
        F: FnMut(&mut GCodeReader, &GCodeLine),
    {
        let mut gline = GCodeLine::new();
        self.parsing = true;

        for line in buffer.lines() {
            if !self.parsing {
                break;
            }
            gline.reset();
            self.parse_line_str(line, &mut gline, &mut callback);
        }
    }

    /// Parse a single line of G-code
    /// GCodeReader.cpp:27-103
    fn parse_line_str<F>(&mut self, line: &str, gline: &mut GCodeLine, callback: &mut F)
    where
        F: FnMut(&mut GCodeReader, &GCodeLine),
    {
        let bytes = line.as_bytes();
        let mut i = 0;

        // Skip leading whitespace
        while i < bytes.len() && (bytes[i] == b' ' || bytes[i] == b'\t') {
            i += 1;
        }

        let cmd_start = i;

        // Skip command word
        while i < bytes.len()
            && bytes[i] != b' '
            && bytes[i] != b'\t'
            && bytes[i] != b';'
            && bytes[i] != b'\r'
            && bytes[i] != b'\n'
        {
            i += 1;
        }

        let _cmd_end = i;

        // Parse axis values
        // GCodeReader.cpp:48-87
        while i < bytes.len() && bytes[i] != b';' && bytes[i] != b'\r' && bytes[i] != b'\n' {
            // Skip whitespace
            while i < bytes.len() && (bytes[i] == b' ' || bytes[i] == b'\t') {
                i += 1;
            }

            if i >= bytes.len() || bytes[i] == b';' || bytes[i] == b'\r' || bytes[i] == b'\n' {
                break;
            }

            // Check for axis identifier
            let axis_opt = if i < bytes.len() {
                Axis::from_char(bytes[i] as char)
            } else {
                None
            };

            if let Some(axis) = axis_opt {
                i += 1; // Skip axis character

                // Try to parse numeric value
                let value_start = i;
                let mut has_dot = false;
                let mut has_sign = false;

                // Parse number
                while i < bytes.len() {
                    let c = bytes[i] as char;
                    if c == '-' || c == '+' {
                        if has_sign || i != value_start {
                            break;
                        }
                        has_sign = true;
                    } else if c == '.' {
                        if has_dot {
                            break;
                        }
                        has_dot = true;
                    } else if !c.is_ascii_digit() {
                        break;
                    }
                    i += 1;
                }

                if i > value_start {
                    let value_str = &line[value_start..i];
                    if let Ok(value) = value_str.parse::<f32>() {
                        gline.axis[axis as usize] = value;
                        gline.mask |= 1 << axis as u8;
                    }
                }
            } else {
                // Skip unknown word
                while i < bytes.len()
                    && bytes[i] != b' '
                    && bytes[i] != b'\t'
                    && bytes[i] != b';'
                    && bytes[i] != b'\r'
                    && bytes[i] != b'\n'
                {
                    i += 1;
                }
            }
        }

        // Handle relative E distances
        // GCodeReader.cpp:90-91
        if gline.has(Axis::E) && self.config.use_relative_e_distances {
            self.position[Axis::E as usize] = 0.0;
        }

        // Store raw line (up to comment or EOL)
        // GCodeReader.cpp:96-99
        let end = line.find(|c| c == '\r' || c == '\n').unwrap_or(line.len());
        gline.raw = line[..end].to_string();

        if self.verbose {
            println!("{}", gline.raw);
        }

        // Call user callback
        callback(self, gline);

        // Update position coordinates
        // GCodeReader.cpp:108-119
        self.update_coordinates(gline);
    }

    /// Update internal position state from parsed line
    /// GCodeReader.cpp:108-119
    fn update_coordinates(&mut self, gline: &GCodeLine) {
        let cmd = gline.cmd();
        if cmd.starts_with('G') {
            let cmd_num = cmd[1..].trim();
            // G0, G1, G2, G3, G92
            if cmd_num == "0"
                || cmd_num == "1"
                || cmd_num == "2"
                || cmd_num == "3"
                || cmd_num == "92"
            {
                for axis in [
                    Axis::X,
                    Axis::Y,
                    Axis::Z,
                    Axis::E,
                    Axis::F,
                    Axis::I,
                    Axis::J,
                    Axis::P,
                ] {
                    if gline.has(axis) {
                        self.position[axis as usize] = gline.value(axis);
                    }
                }
            }
        }
    }

    /// Parse a G-code file
    /// GCodeReader.cpp:205-213
    pub fn parse_file<P: AsRef<Path>, F>(&mut self, path: P, mut callback: F) -> Result<()>
    where
        F: FnMut(&mut GCodeReader, &GCodeLine),
    {
        let file = File::open(path.as_ref()).map_err(|e| {
            Error::IO(format!(
                "Failed to open G-code file '{}': {}",
                path.as_ref().display(),
                e
            ))
        })?;

        let reader = BufReader::new(file);
        let mut gline = GCodeLine::new();
        self.parsing = true;

        for line in reader.lines() {
            if !self.parsing {
                break;
            }

            let line = line.map_err(|e| Error::IO(format!("Failed to read line: {}", e)))?;

            // Skip line number if present
            // GCodeReader.cpp:188-193
            let line = if line.trim_start().starts_with('N') || line.trim_start().starts_with('n') {
                let trimmed = line.trim_start();
                if let Some(space_pos) = trimmed[1..].find(char::is_whitespace) {
                    trimmed[space_pos + 1..].trim_start()
                } else {
                    &trimmed[1..]
                }
            } else {
                &line
            };

            gline.reset();
            self.parse_line_str(line, &mut gline, &mut callback);
        }

        Ok(())
    }

    /// Parse a file line by line with raw string callback
    /// GCodeReader.cpp:224-228
    pub fn parse_file_raw<P: AsRef<Path>, F>(&mut self, path: P, mut callback: F) -> Result<()>
    where
        F: FnMut(&mut GCodeReader, &str),
    {
        let file = File::open(path.as_ref()).map_err(|e| {
            Error::IO(format!(
                "Failed to open G-code file '{}': {}",
                path.as_ref().display(),
                e
            ))
        })?;

        let reader = BufReader::new(file);
        self.parsing = true;

        for line in reader.lines() {
            if !self.parsing {
                break;
            }

            let line = line.map_err(|e| Error::IO(format!("Failed to read line: {}", e)))?;
            callback(self, &line);
        }

        Ok(())
    }

    /// Stop parsing (to be called by callback)
    /// GCodeReader.hpp:130
    pub fn quit_parsing(&mut self) {
        self.parsing = false;
    }

    /// Get current X position
    /// GCodeReader.hpp:132-133
    pub fn x(&self) -> f32 {
        self.position[Axis::X as usize]
    }

    /// Get mutable reference to current X position
    /// GCodeReader.hpp:132
    pub fn x_mut(&mut self) -> &mut f32 {
        &mut self.position[Axis::X as usize]
    }

    /// Get current Y position
    /// GCodeReader.hpp:134-135
    pub fn y(&self) -> f32 {
        self.position[Axis::Y as usize]
    }

    /// Get mutable reference to current Y position
    /// GCodeReader.hpp:134
    pub fn y_mut(&mut self) -> &mut f32 {
        &mut self.position[Axis::Y as usize]
    }

    /// Get current Z position
    /// GCodeReader.hpp:136-137
    pub fn z(&self) -> f32 {
        self.position[Axis::Z as usize]
    }

    /// Get mutable reference to current Z position
    /// GCodeReader.hpp:136
    pub fn z_mut(&mut self) -> &mut f32 {
        &mut self.position[Axis::Z as usize]
    }

    /// Get current E position
    /// GCodeReader.hpp:138-139
    pub fn e(&self) -> f32 {
        self.position[Axis::E as usize]
    }

    /// Get mutable reference to current E position
    /// GCodeReader.hpp:138
    pub fn e_mut(&mut self) -> &mut f32 {
        &mut self.position[Axis::E as usize]
    }

    /// Get current F (feedrate) value
    /// GCodeReader.hpp:140-141
    pub fn f(&self) -> f32 {
        self.position[Axis::F as usize]
    }

    /// Get mutable reference to current F value
    /// GCodeReader.hpp:140
    pub fn f_mut(&mut self) -> &mut f32 {
        &mut self.position[Axis::F as usize]
    }

    /// Get current I position (arc center X offset)
    /// GCodeReader.hpp:143-144
    pub fn i(&self) -> f32 {
        self.position[Axis::I as usize]
    }

    /// Get mutable reference to current I position
    /// GCodeReader.hpp:143
    pub fn i_mut(&mut self) -> &mut f32 {
        &mut self.position[Axis::I as usize]
    }

    /// Get current J position (arc center Y offset)
    /// GCodeReader.hpp:145-146
    pub fn j(&self) -> f32 {
        self.position[Axis::J as usize]
    }

    /// Get mutable reference to current J position
    /// GCodeReader.hpp:145
    pub fn j_mut(&mut self) -> &mut f32 {
        &mut self.position[Axis::J as usize]
    }

    /// Get current configuration
    /// GCodeReader.hpp:156-159
    pub fn get_config(&self) -> &GCodeConfig {
        &self.config
    }
}

impl Default for GCodeReader {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_axis_enum() {
        // Test axis to char conversion
        assert_eq!(Axis::X.to_char(), 'X');
        assert_eq!(Axis::Y.to_char(), 'Y');
        assert_eq!(Axis::E.to_char(), 'E');
        assert_eq!(Axis::I.to_char(), 'I');

        // Test char to axis parsing
        assert_eq!(Axis::from_char('X'), Some(Axis::X));
        assert_eq!(Axis::from_char('Z'), Some(Axis::Z));
        assert_eq!(Axis::from_char('Q'), None);
    }

    #[test]
    fn test_gcode_line_basic() {
        let mut line = GCodeLine::new();
        assert_eq!(line.raw(), "");
        assert!(!line.has(Axis::X));

        line.axis[Axis::X as usize] = 10.5;
        line.mask |= 1 << Axis::X as u8;
        assert!(line.has(Axis::X));
        assert_eq!(line.value(Axis::X), 10.5);
    }

    #[test]
    fn test_gcode_line_cmd_extraction() {
        let mut line = GCodeLine::new();
        line.raw = "G1 X10 Y20".to_string();
        assert_eq!(line.cmd(), "G1");

        line.raw = "M104 S200".to_string();
        assert_eq!(line.cmd(), "M104");

        line.raw = "  G28  ".to_string();
        assert_eq!(line.cmd(), "G28");
    }

    #[test]
    fn test_gcode_line_comment() {
        let mut line = GCodeLine::new();
        line.raw = "G1 X10 ; move to X10".to_string();
        assert_eq!(line.comment(), " move to X10");

        line.raw = "G28".to_string();
        assert_eq!(line.comment(), "");
    }

    #[test]
    fn test_gcode_line_cmd_is() {
        assert!(GCodeLine::cmd_is_str("G1 X10", "G1"));
        assert!(GCodeLine::cmd_is_str("  G28  ", "G28"));
        assert!(!GCodeLine::cmd_is_str("G10 X5", "G1"));
        assert!(GCodeLine::cmd_is_str("M104", "M104"));
    }

    #[test]
    fn test_gcode_line_cmd_start_with() {
        assert!(GCodeLine::cmd_start_with("G1 X10", "G1"));
        assert!(GCodeLine::cmd_start_with("G10 X5", "G1"));
        assert!(!GCodeLine::cmd_start_with("M104", "G"));
    }

    #[test]
    fn test_gcode_reader_parse_buffer() {
        let mut reader = GCodeReader::new();
        let gcode = "G1 X10 Y20\nG1 Z5 E2.5\nG28";

        let mut lines_parsed = 0;
        reader.parse_buffer(gcode, |_, _| {
            lines_parsed += 1;
        });

        assert_eq!(lines_parsed, 3);
    }

    #[test]
    fn test_gcode_reader_position_tracking() {
        let mut reader = GCodeReader::new();
        let gcode = "G1 X10 Y20\nG1 X15 Y25 Z5";

        reader.parse_buffer(gcode, |_, _| {});

        assert_eq!(reader.x(), 15.0);
        assert_eq!(reader.y(), 25.0);
        assert_eq!(reader.z(), 5.0);
    }

    #[test]
    fn test_gcode_reader_parse_line() {
        let mut reader = GCodeReader::new();
        let mut gline = GCodeLine::new();

        reader.parse_line_str("G1 X100.5 Y-50.25 F3000", &mut gline, &mut |_, _| {});

        assert!(gline.has(Axis::X));
        assert!(gline.has(Axis::Y));
        assert!(gline.has(Axis::F));
        assert_eq!(gline.x(), 100.5);
        assert_eq!(gline.y(), -50.25);
        assert_eq!(gline.f(), 3000.0);
    }

    #[test]
    fn test_gcode_line_distances() {
        let mut reader = GCodeReader::new();
        *reader.x_mut() = 10.0;
        *reader.y_mut() = 20.0;

        let mut gline = GCodeLine::new();
        gline.axis[Axis::X as usize] = 15.0;
        gline.axis[Axis::Y as usize] = 25.0;
        gline.mask = (1 << Axis::X as u8) | (1 << Axis::Y as u8);

        assert_eq!(gline.dist_x(&reader), 5.0);
        assert_eq!(gline.dist_y(&reader), 5.0);
        assert!((gline.dist_xy(&reader) - 7.071).abs() < 0.01);
    }

    #[test]
    fn test_gcode_line_extrusion_detection() {
        let mut reader = GCodeReader::new();
        *reader.e_mut() = 0.0;

        let mut gline = GCodeLine::new();
        gline.raw = "G1 X10 E2.5".to_string();
        gline.axis[Axis::E as usize] = 2.5;
        gline.mask = 1 << Axis::E as u8;

        assert!(gline.extruding(&reader));
        assert!(!gline.retracting(&reader));

        gline.axis[Axis::E as usize] = -0.5;
        assert!(!gline.extruding(&reader));
        assert!(gline.retracting(&reader));
    }

    #[test]
    fn test_gcode_line_travel_detection() {
        let mut gline = GCodeLine::new();
        gline.raw = "G1 X10 Y20".to_string();
        gline.axis[Axis::X as usize] = 10.0;
        gline.axis[Axis::Y as usize] = 20.0;
        gline.mask = (1 << Axis::X as u8) | (1 << Axis::Y as u8);

        assert!(gline.travel());

        gline.axis[Axis::E as usize] = 1.0;
        gline.mask |= 1 << Axis::E as u8;
        assert!(!gline.travel());
    }

    #[test]
    fn test_gcode_reader_quit_parsing() {
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
    fn test_arc_commands() {
        let mut reader = GCodeReader::new();
        let mut gline = GCodeLine::new();

        reader.parse_line_str("G2 X10 Y10 I5 J0 E2", &mut gline, &mut |_, _| {});

        assert_eq!(gline.cmd(), "G2");
        assert!(gline.has(Axis::I));
        assert!(gline.has(Axis::J));
        assert_eq!(gline.i(), 5.0);
        assert_eq!(gline.j(), 0.0);
    }

    #[test]
    fn test_relative_e_mode() {
        let mut reader = GCodeReader::new();
        reader.config.use_relative_e_distances = true;
        *reader.e_mut() = 10.0;

        let mut gline = GCodeLine::new();
        reader.parse_line_str("G1 E2", &mut gline, &mut |_, _| {});

        // In relative mode, E position should reset to 0
        assert_eq!(reader.e(), 2.0); // Updated to new value after parsing
    }
}
