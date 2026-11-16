//! Port of `TextureToColor/Callbacks.hpp`: the progress and cancel callbacks of the tex2color
//! algorithms.
//!
//! C++ Reference:
//! - src/libslic3r/TextureToColor/Callbacks.hpp

use std::sync::Arc;

/// Callbacks.hpp:6-9. `message` is a `const char*` in C++; most are literals, the one a mesh
/// repair callback reports is not, so it is a `String` here.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct AlgoProgress {
    pub percent: i32,
    pub message: String,
}

impl AlgoProgress {
    /// `AlgoProgress{percent, message}`
    pub fn new(percent: i32, message: &str) -> Self {
        AlgoProgress { percent, message: message.to_string() }
    }
}

/// Callbacks.hpp:11, `std::function<void(AlgoProgress)>`. `None` is the empty function.
pub type AlgoProgressCallback = Option<Arc<dyn Fn(AlgoProgress) + Send + Sync>>;

/// Callbacks.hpp:12, `std::function<bool()>`. `None` is the empty function.
pub type AlgoCancelCallback = Option<Arc<dyn Fn() -> bool + Send + Sync>>;
