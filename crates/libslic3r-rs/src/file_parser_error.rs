//! File parser error type
//!
//! C++ Reference: FileParserError.hpp
//!
//! This module provides a specialized error type for file parsing errors,
//! including filename and line number information.

use crate::exception::RuntimeError;
use std::error::Error as StdError;
use std::fmt;
use std::path::Path;

// Generic file parser error, mostly copied from boost::property_tree::file_parser_error
/// Generic file parser error with location information
/// FileParserError.hpp:13-48
#[derive(Debug, Clone)]
pub struct FileParserError {
    /// Error message (without line and file info)
    /// FileParserError.hpp:34
    message: String,

    /// Filename where error occurred
    /// FileParserError.hpp:35
    filename: String,

    /// Line number where error occurred (0 if unknown)
    /// FileParserError.hpp:36
    line: u64,

    /// Formatted full error message
    /// FileParserError.hpp:39-47
    what: String,
}

impl FileParserError {
    /// Create a new file parser error with message, filename, and optional line number
    /// FileParserError.hpp:16-18
    pub fn new(msg: impl Into<String>, file: impl Into<String>, line: u64) -> Self {
        let message = msg.into();
        let filename = file.into();
        let what = Self::format_what(&message, &filename, line);

        Self {
            message,
            filename,
            line,
            what,
        }
    }

    /// Create a new file parser error from a Path
    /// FileParserError.hpp:19-21
    pub fn from_path(msg: impl Into<String>, file: &Path, line: u64) -> Self {
        let filename = file.to_string_lossy().into_owned();
        Self::new(msg, filename, line)
    }

    /// Get error message (without line and file - use Display to get full message)
    /// FileParserError.hpp:27
    pub fn message(&self) -> &str {
        &self.message
    }

    /// Get error filename
    /// FileParserError.hpp:29
    pub fn filename(&self) -> &str {
        &self.filename
    }

    /// Get error line number
    /// FileParserError.hpp:31
    pub fn line(&self) -> u64 {
        self.line
    }

    /// Format error message with filename and line number
    /// FileParserError.hpp:39-47
    fn format_what(msg: &str, file: &str, line: u64) -> String {
        let file_display = if file.is_empty() {
            "<unspecified file>"
        } else {
            file
        };

        if line > 0 {
            format!("{}({}): {}", file_display, line, msg)
        } else {
            format!("{}: {}", file_display, msg)
        }
    }
}

impl fmt::Display for FileParserError {
    /// Display the full error message with location
    /// FileParserError.hpp:39-47
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.what)
    }
}

impl StdError for FileParserError {}

/// Convert to RuntimeError
impl From<FileParserError> for RuntimeError {
    fn from(err: FileParserError) -> Self {
        RuntimeError::new(err.to_string())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::PathBuf;

    #[test]
    fn test_file_parser_error_with_line() {
        let err = FileParserError::new("Invalid syntax", "test.gcode", 42);

        assert_eq!(err.message(), "Invalid syntax");
        assert_eq!(err.filename(), "test.gcode");
        assert_eq!(err.line(), 42);
        assert_eq!(err.to_string(), "test.gcode(42): Invalid syntax");
    }

    #[test]
    fn test_file_parser_error_without_line() {
        let err = FileParserError::new("File not found", "missing.stl", 0);

        assert_eq!(err.message(), "File not found");
        assert_eq!(err.filename(), "missing.stl");
        assert_eq!(err.line(), 0);
        assert_eq!(err.to_string(), "missing.stl: File not found");
    }

    #[test]
    fn test_file_parser_error_empty_filename() {
        let err = FileParserError::new("Parse error", "", 10);

        assert_eq!(err.message(), "Parse error");
        assert_eq!(err.filename(), "");
        assert_eq!(err.line(), 10);
        assert_eq!(err.to_string(), "<unspecified file>(10): Parse error");
    }

    #[test]
    fn test_file_parser_error_from_path() {
        let path = PathBuf::from("/path/to/file.obj");
        let err = FileParserError::from_path("Bad format", &path, 100);

        assert_eq!(err.message(), "Bad format");
        assert_eq!(err.filename(), "/path/to/file.obj");
        assert_eq!(err.line(), 100);
        assert!(err.to_string().contains("file.obj"));
        assert!(err.to_string().contains("100"));
    }

    #[test]
    fn test_file_parser_error_display() {
        let err = FileParserError::new("Unexpected token", "config.ini", 5);
        let display = format!("{}", err);

        assert!(display.contains("config.ini"));
        assert!(display.contains("5"));
        assert!(display.contains("Unexpected token"));
    }

    #[test]
    fn test_file_parser_error_to_runtime_error() {
        let file_err = FileParserError::new("Test error", "test.txt", 1);
        let runtime_err: RuntimeError = file_err.into();

        assert!(runtime_err.to_string().contains("Test error"));
    }

    #[test]
    fn test_format_what_with_line() {
        let err = FileParserError::new("Error message", "file.txt", 42);
        assert_eq!(err.to_string(), "file.txt(42): Error message");
    }

    #[test]
    fn test_format_what_without_line() {
        let err = FileParserError::new("Error message", "file.txt", 0);
        assert_eq!(err.to_string(), "file.txt: Error message");
    }

    #[test]
    fn test_format_what_empty_file_with_line() {
        let err = FileParserError::new("Error message", "", 42);
        assert_eq!(err.to_string(), "<unspecified file>(42): Error message");
    }

    #[test]
    fn test_format_what_empty_file_without_line() {
        let err = FileParserError::new("Error message", "", 0);
        assert_eq!(err.to_string(), "<unspecified file>: Error message");
    }
}
