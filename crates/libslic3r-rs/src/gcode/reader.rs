//! G-code reader module.
//!
//! This module provides the `GCodeReader` type, which mirrors the C++ `GCodeReader`
//! functionality (currently implemented via `ParsedGCode` in the `compare` module).

use crate::gcode::compare::ParsedGCode;

/// G-code reader (alias for ParsedGCode).
///
/// In the C++ implementation, `GCodeReader` is a parser that calls callbacks.
/// Here we currently use a DOM-style parser that loads the full file.
/// Future iterations might implement a streaming parser if needed.
pub type GCodeReader = ParsedGCode;

/// Implementation block for GCodeReader compatibility methods
/// GCode.cpp:1-50
impl GCodeReader {
    // Add any compatibility methods if needed here
}
