//! Exception types for libslic3r
//!
//! Direct port of BambuStudio's Exception.hpp
//! C++ Reference: `reference/BambuStudio/src/libslic3r/Exception.hpp`

use std::error::Error as StdError;
use std::fmt;

/// Namespace declaration for Slic3r exceptions
/// Exception.hpp:8

/// Base exception type for all Slic3r exceptions, equivalent to std::runtime_error
/// Exception.hpp:9-11
#[derive(Debug, Clone)]
/// Base exception for Slic3r errors
/// Exception.hpp:9-11
pub struct Exception {
    pub message: String,
}

/// Implementation of Exception constructor and methods
/// Exception.hpp:9-11
impl Exception {
    // Create a new Exception with the given message
    // Exception.hpp:10
    pub fn new(message: String) -> Self {
        Self { message }
    }
}

/// Display implementation for Exception
/// Exception.hpp:9-11
impl fmt::Display for Exception {
    // Format the exception message for display
    // Exception.hpp:11
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.message)
    }
}

/// StdError trait implementation for Exception
/// Exception.hpp:9-11
impl StdError for Exception {}

/// Macro definition for deriving exception types from parent exceptions
/// Exception.hpp:12-13

/// Critical exception that should never reach the UI thread
/// Exception.hpp:14-16
#[derive(Debug, Clone)]
/// Critical exception type
/// Exception.hpp:14-16
pub struct CriticalException {
    pub message: String,
}

/// Implementation of CriticalException constructor and methods
/// Exception.hpp:14-16
impl CriticalException {
    // Create a new CriticalException with the given message
    // Exception.hpp:15
    pub fn new(message: String) -> Self {
        Self { message }
    }
}

/// Display implementation for CriticalException
/// Exception.hpp:14-16
impl fmt::Display for CriticalException {
    // Format the critical exception message for display
    // Exception.hpp:16
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.message)
    }
}

/// StdError trait implementation for CriticalException
/// Exception.hpp:14-16
impl StdError for CriticalException {}

/// Runtime error derived from CriticalException
/// Exception.hpp:17
#[derive(Debug, Clone)]
/// Runtime error exception type
/// Exception.hpp:17
pub struct RuntimeError {
    pub message: String,
}

/// Implementation of RuntimeError constructor and methods
/// Exception.hpp:17
impl RuntimeError {
    // Create a new RuntimeError with the given message
    // Exception.hpp:17
    pub fn new(message: String) -> Self {
        Self { message }
    }
}

/// Display implementation for RuntimeError
/// Exception.hpp:17
impl fmt::Display for RuntimeError {
    // Format the runtime error message for display
    // Exception.hpp:17
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.message)
    }
}

/// StdError trait implementation for RuntimeError
/// Exception.hpp:17
impl StdError for RuntimeError {}

/// Logic error for programming/assertion failures
/// Exception.hpp:18
#[derive(Debug, Clone)]
/// Logic error exception type
/// Exception.hpp:18
pub struct LogicError {
    pub message: String,
}

/// Implementation of LogicError constructor and methods
/// Exception.hpp:18
impl LogicError {
    // Create a new LogicError with the given message
    // Exception.hpp:18
    pub fn new(message: String) -> Self {
        Self { message }
    }
}

/// Display implementation for LogicError
/// Exception.hpp:18
impl fmt::Display for LogicError {
    // Format the logic error message for display
    // Exception.hpp:18
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.message)
    }
}

/// StdError trait implementation for LogicError
/// Exception.hpp:18
impl StdError for LogicError {}

/// Hard crash exception for unrecoverable errors
/// Exception.hpp:19
#[derive(Debug, Clone)]
/// Hard crash exception type
/// Exception.hpp:19
pub struct HardCrash {
    pub message: String,
}

/// Implementation of HardCrash constructor and methods
/// Exception.hpp:19
impl HardCrash {
    // Create a new HardCrash with the given message
    // Exception.hpp:19
    pub fn new(message: String) -> Self {
        Self { message }
    }
}

/// Display implementation for HardCrash
/// Exception.hpp:19
impl fmt::Display for HardCrash {
    // Format the hard crash message for display
    // Exception.hpp:19
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.message)
    }
}

/// StdError trait implementation for HardCrash
/// Exception.hpp:19
impl StdError for HardCrash {}

/// Invalid argument exception for bad function parameters
/// Exception.hpp:20
#[derive(Debug, Clone)]
/// Invalid argument exception type
/// Exception.hpp:20
pub struct InvalidArgument {
    pub message: String,
}

/// Implementation of InvalidArgument constructor and methods
/// Exception.hpp:20
impl InvalidArgument {
    // Create a new InvalidArgument with the given message
    // Exception.hpp:20
    pub fn new(message: String) -> Self {
        Self { message }
    }
}

/// Display implementation for InvalidArgument
/// Exception.hpp:20
impl fmt::Display for InvalidArgument {
    // Format the invalid argument message for display
    // Exception.hpp:20
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.message)
    }
}

/// StdError trait implementation for InvalidArgument
/// Exception.hpp:20
impl StdError for InvalidArgument {}

/// Out of range exception for index/bounds errors
/// Exception.hpp:21
#[derive(Debug, Clone)]
/// Out of range exception type
/// Exception.hpp:21
pub struct OutOfRange {
    pub message: String,
}

/// Implementation of OutOfRange constructor and methods
/// Exception.hpp:21
impl OutOfRange {
    // Create a new OutOfRange with the given message
    // Exception.hpp:21
    pub fn new(message: String) -> Self {
        Self { message }
    }
}

/// Display implementation for OutOfRange
/// Exception.hpp:21
impl fmt::Display for OutOfRange {
    // Format the out of range message for display
    // Exception.hpp:21
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.message)
    }
}

/// StdError trait implementation for OutOfRange
/// Exception.hpp:21
impl StdError for OutOfRange {}

/// I/O error for file and network operations
/// Exception.hpp:22
#[derive(Debug, Clone)]
/// I/O error exception type
/// Exception.hpp:22
pub struct IOError {
    pub message: String,
}

/// Implementation of IOError constructor and methods
/// Exception.hpp:22
impl IOError {
    // Create a new IOError with the given message
    // Exception.hpp:22
    pub fn new(message: String) -> Self {
        Self { message }
    }
}

/// Display implementation for IOError
/// Exception.hpp:22
impl fmt::Display for IOError {
    // Format the I/O error message for display
    // Exception.hpp:22
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.message)
    }
}

/// StdError trait implementation for IOError
/// Exception.hpp:22
impl StdError for IOError {}

/// File I/O specific error
/// Exception.hpp:23
#[derive(Debug, Clone)]
/// File I/O error exception type
/// Exception.hpp:23
pub struct FileIOError {
    pub message: String,
}

/// Implementation of FileIOError constructor and methods
/// Exception.hpp:23
impl FileIOError {
    // Create a new FileIOError with the given message
    // Exception.hpp:23
    pub fn new(message: String) -> Self {
        Self { message }
    }
}

/// Display implementation for FileIOError
/// Exception.hpp:23
impl fmt::Display for FileIOError {
    // Format the file I/O error message for display
    // Exception.hpp:23
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.message)
    }
}

/// StdError trait implementation for FileIOError
/// Exception.hpp:23
impl StdError for FileIOError {}

/// Network/host communication error
/// Exception.hpp:24
#[derive(Debug, Clone)]
/// Host network error exception type
/// Exception.hpp:24
pub struct HostNetworkError {
    pub message: String,
}

/// Implementation of HostNetworkError constructor and methods
/// Exception.hpp:24
impl HostNetworkError {
    // Create a new HostNetworkError with the given message
    // Exception.hpp:24
    pub fn new(message: String) -> Self {
        Self { message }
    }
}

/// Display implementation for HostNetworkError
/// Exception.hpp:24
impl fmt::Display for HostNetworkError {
    // Format the host network error message for display
    // Exception.hpp:24
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.message)
    }
}

/// StdError trait implementation for HostNetworkError
/// Exception.hpp:24
impl StdError for HostNetworkError {}

/// G-code or 3MF export error
/// Exception.hpp:25
#[derive(Debug, Clone)]
/// Export error exception type
/// Exception.hpp:25
pub struct ExportError {
    pub message: String,
}

/// Implementation of ExportError constructor and methods
/// Exception.hpp:25
impl ExportError {
    // Create a new ExportError with the given message
    // Exception.hpp:25
    pub fn new(message: String) -> Self {
        Self { message }
    }
}

/// Display implementation for ExportError
/// Exception.hpp:25
impl fmt::Display for ExportError {
    // Format the export error message for display
    // Exception.hpp:25
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.message)
    }
}

/// StdError trait implementation for ExportError
/// Exception.hpp:25
impl StdError for ExportError {}

/// Error during placeholder parsing in configuration
/// Exception.hpp:26
#[derive(Debug, Clone)]
/// Placeholder parser error exception type
/// Exception.hpp:26
pub struct PlaceholderParserError {
    pub message: String,
}

/// Implementation of PlaceholderParserError constructor and methods
/// Exception.hpp:26
impl PlaceholderParserError {
    // Create a new PlaceholderParserError with the given message
    // Exception.hpp:26
    pub fn new(message: String) -> Self {
        Self { message }
    }
}

/// Display implementation for PlaceholderParserError
/// Exception.hpp:26
impl fmt::Display for PlaceholderParserError {
    // Format the placeholder parser error message for display
    // Exception.hpp:26
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.message)
    }
}

/// StdError trait implementation for PlaceholderParserError
/// Exception.hpp:26
impl StdError for PlaceholderParserError {}

/// Slicing error with associated object ID for tracking which object failed
/// Exception.hpp:27-38
#[derive(Debug, Clone)]
/// Slicing error with object ID
/// Exception.hpp:27-38
pub struct SlicingError {
    pub message: String,
    object_id: usize,
}

/// Implementation of SlicingError constructor and methods
/// Exception.hpp:27-38
impl SlicingError {
    // Create a new SlicingError with the given message
    // Exception.hpp:31
    pub fn new(message: String) -> Self {
        Self {
            message,
            object_id: 0,
        }
    }

    /// Constructor with object ID parameter
    /// Exception.hpp:32
    pub fn new_with_object(message: String, object_id: usize) -> Self {
        Self { message, object_id }
    }

    /// Get the object ID associated with this error
    /// Exception.hpp:33
    pub fn object_id(&self) -> usize {
        self.object_id
    }
}

/// Display implementation for SlicingError
/// Exception.hpp:27-38
impl fmt::Display for SlicingError {
    // Format the slicing error message for display
    // Exception.hpp:36
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.message)
    }
}

/// StdError trait implementation for SlicingError
/// Exception.hpp:27-38
impl StdError for SlicingError {}

/// Collection of multiple slicing errors
/// Exception.hpp:40-47
#[derive(Debug, Clone)]
/// Collection of multiple slicing errors
/// Exception.hpp:40-47
pub struct SlicingErrors {
    pub message: String,
    pub errors: Vec<SlicingError>,
}

/// Implementation of SlicingErrors constructor and methods
/// Exception.hpp:40-47
impl SlicingErrors {
    // Constructor taking vector of SlicingError objects
    // Exception.hpp:44
    pub fn new(errors: Vec<SlicingError>) -> Self {
        Self {
            message: "Errors".to_string(),
            errors,
        }
    }
}

/// Display implementation for SlicingErrors
/// Exception.hpp:40-47
impl fmt::Display for SlicingErrors {
    // Format the slicing errors message for display
    // Exception.hpp:47
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.message)
    }
}

/// StdError trait implementation for SlicingErrors
/// Exception.hpp:40-47
impl StdError for SlicingErrors {}

// Namespace closing brace
// Exception.hpp:51
