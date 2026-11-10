//! Exception types for libslic3r
//!
//! Direct 1:1 port of BambuStudio's `Exception.hpp` (header-only; no `.cpp`).
//! C++ Reference: `reference/BambuStudio/src/libslic3r/Exception.hpp`
//!
//! C++ uses an exception hierarchy rooted at `std::runtime_error`:
//!
//! ```text
//! std::runtime_error
//!  └─ Exception
//!      ├─ CriticalException
//!      │   ├─ RuntimeError
//!      │   │   └─ PlaceholderParserError
//!      │   ├─ LogicError
//!      │   │   ├─ InvalidArgument
//!      │   │   └─ OutOfRange
//!      │   ├─ HardCrash
//!      │   ├─ IOError
//!      │   │   ├─ FileIOError
//!      │   │   └─ HostNetworkError
//!      │   └─ ExportError
//!      ├─ SlicingError
//!      └─ SlicingErrors
//! ```
//!
//! Rust has no implementation inheritance, so each exception is its own struct
//! carrying the `std::runtime_error` message. The `is-a` relationships (which in
//! C++ allow a derived exception to be `catch`-ed as any of its bases) are
//! modelled with `From` conversions that promote a derived exception to each of
//! its ancestors, preserving the message verbatim.

use std::error::Error as StdError;
use std::fmt;

// Exception.hpp:7
// namespace Slic3r {

// PrusaSlicer's own exception hierarchy is derived from std::runtime_error.
// Base for Slicer's own exceptions.
// Exception.hpp:11
// class Exception : public std::runtime_error { using std::runtime_error::runtime_error; };
#[derive(Debug, Clone)]
pub struct Exception {
    /// The `std::runtime_error` message (returned by `what()`).
    pub message: String,
}

impl Exception {
    /// `using std::runtime_error::runtime_error;` — inherits the
    /// `runtime_error(const std::string&)` constructor.
    /// Exception.hpp:11
    pub fn new(message: String) -> Self {
        Self { message }
    }
}

impl fmt::Display for Exception {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.message)
    }
}

impl StdError for Exception {}

// Exception.hpp:12-13
// #define SLIC3R_DERIVE_EXCEPTION(DERIVED_EXCEPTION, PARENT_EXCEPTION) \
//     class DERIVED_EXCEPTION : public PARENT_EXCEPTION { using PARENT_EXCEPTION::PARENT_EXCEPTION; }
//
// Rust analogue of the C++ macro: declares a derived exception that "inherits"
// the parent's `new(String)` constructor and, via `From`, can be promoted to
// the parent exception (the faithful equivalent of public inheritance for the
// purpose of `catch` by base type). The trailing `$($ancestor),*` lists the
// remaining transitive bases so the type can also be promoted to those.
macro_rules! slic3r_derive_exception {
    ($derived:ident, $parent:ident $(, $ancestor:ident)* $(,)?) => {
        #[derive(Debug, Clone)]
        pub struct $derived {
            /// The `std::runtime_error` message (returned by `what()`).
            pub message: String,
        }

        impl $derived {
            // using PARENT_EXCEPTION::PARENT_EXCEPTION;
            pub fn new(message: String) -> Self {
                Self { message }
            }
        }

        impl fmt::Display for $derived {
            fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
                write!(f, "{}", self.message)
            }
        }

        impl StdError for $derived {}

        // public inheritance: $derived "is-a" $parent (and each $ancestor).
        impl From<$derived> for $parent {
            fn from(e: $derived) -> Self {
                $parent::new(e.message)
            }
        }
        $(
            impl From<$derived> for $ancestor {
                fn from(e: $derived) -> Self {
                    $ancestor::new(e.message)
                }
            }
        )*
    };
}

// Critical exception produced by Slicer, such exception shall never propagate up to the UI thread.
// If that happens, an ugly fat message box with an ugly fat exclamation mark is displayed.
// Exception.hpp:16
// SLIC3R_DERIVE_EXCEPTION(CriticalException,  Exception);
slic3r_derive_exception!(CriticalException, Exception);

// Exception.hpp:17
// SLIC3R_DERIVE_EXCEPTION(RuntimeError,       CriticalException);
slic3r_derive_exception!(RuntimeError, CriticalException, Exception);

// Exception.hpp:18
// SLIC3R_DERIVE_EXCEPTION(LogicError,         CriticalException);
slic3r_derive_exception!(LogicError, CriticalException, Exception);

// Exception.hpp:19
// SLIC3R_DERIVE_EXCEPTION(HardCrash,          CriticalException);
slic3r_derive_exception!(HardCrash, CriticalException, Exception);

// Exception.hpp:20
// SLIC3R_DERIVE_EXCEPTION(InvalidArgument,    LogicError);
slic3r_derive_exception!(InvalidArgument, LogicError, CriticalException, Exception);

// Exception.hpp:21
// SLIC3R_DERIVE_EXCEPTION(OutOfRange,         LogicError);
slic3r_derive_exception!(OutOfRange, LogicError, CriticalException, Exception);

// Exception.hpp:22
// SLIC3R_DERIVE_EXCEPTION(IOError,            CriticalException);
slic3r_derive_exception!(IOError, CriticalException, Exception);

// Exception.hpp:23
// SLIC3R_DERIVE_EXCEPTION(FileIOError,        IOError);
slic3r_derive_exception!(FileIOError, IOError, CriticalException, Exception);

// Exception.hpp:24
// SLIC3R_DERIVE_EXCEPTION(HostNetworkError,   IOError);
slic3r_derive_exception!(HostNetworkError, IOError, CriticalException, Exception);

// Exception.hpp:25
// SLIC3R_DERIVE_EXCEPTION(ExportError,        CriticalException);
slic3r_derive_exception!(ExportError, CriticalException, Exception);

// Exception.hpp:26
// SLIC3R_DERIVE_EXCEPTION(PlaceholderParserError, RuntimeError);
slic3r_derive_exception!(
    PlaceholderParserError,
    RuntimeError,
    CriticalException,
    Exception
);

// Runtime exception produced by Slicer. Such exception cancels the slicing process and it shall be shown in notifications.
// Exception.hpp:28
//SLIC3R_DERIVE_EXCEPTION(SlicingError,       Exception);
// Exception.hpp:29-38
// class SlicingError : public Exception
// {
// public:
//     using Exception::Exception;
//     SlicingError(std::string const &msg, size_t objectId) : Exception(msg), objectId_(objectId) {}
//     size_t objectId() const { return objectId_; }
//
// private:
//     size_t objectId_ = 0;
// };
#[derive(Debug, Clone)]
pub struct SlicingError {
    /// The `std::runtime_error` message (returned by `what()`).
    pub message: String,
    // Exception.hpp:37
    // size_t objectId_ = 0;
    object_id: usize,
}

impl SlicingError {
    // Exception.hpp:32
    // using Exception::Exception;
    pub fn new(message: String) -> Self {
        Self {
            message,
            object_id: 0,
        }
    }

    // Exception.hpp:33
    // SlicingError(std::string const &msg, size_t objectId) : Exception(msg), objectId_(objectId) {}
    pub fn new_with_object(msg: String, object_id: usize) -> Self {
        Self {
            message: msg,
            object_id,
        }
    }

    // Exception.hpp:34
    // size_t objectId() const { return objectId_; }
    pub fn object_id(&self) -> usize {
        self.object_id
    }
}

impl fmt::Display for SlicingError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.message)
    }
}

impl StdError for SlicingError {}

// public inheritance: SlicingError "is-a" Exception. The object id is C++
// private state that does not survive the slice to the base type.
impl From<SlicingError> for Exception {
    fn from(e: SlicingError) -> Self {
        Exception::new(e.message)
    }
}

// Exception.hpp:40-47
// class SlicingErrors : public Exception
// {
// public:
//     using Exception::Exception;
//     SlicingErrors(const std::vector<SlicingError> &errors) : Exception("Errors"), errors_(errors) {}
//
//     std::vector<SlicingError> errors_;
// };
#[derive(Debug, Clone)]
pub struct SlicingErrors {
    /// The `std::runtime_error` message (returned by `what()`).
    pub message: String,
    // Exception.hpp:46
    // std::vector<SlicingError> errors_;
    pub errors_: Vec<SlicingError>,
}

impl SlicingErrors {
    // Exception.hpp:43
    // using Exception::Exception;
    pub fn new(message: String) -> Self {
        Self {
            message,
            errors_: Vec::new(),
        }
    }

    // Exception.hpp:44
    // SlicingErrors(const std::vector<SlicingError> &errors) : Exception("Errors"), errors_(errors) {}
    pub fn new_with_errors(errors: Vec<SlicingError>) -> Self {
        Self {
            message: "Errors".to_string(),
            errors_: errors,
        }
    }
}

impl fmt::Display for SlicingErrors {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.message)
    }
}

impl StdError for SlicingErrors {}

// public inheritance: SlicingErrors "is-a" Exception.
impl From<SlicingErrors> for Exception {
    fn from(e: SlicingErrors) -> Self {
        Exception::new(e.message)
    }
}

// Exception.hpp:49
// #undef SLIC3R_DERIVE_EXCEPTION

// Exception.hpp:51
// } // namespace Slic3r
