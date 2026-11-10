//! Port of SLA/JobController.hpp
//!
//! C++ Reference:
//! - SLA/JobController.hpp (header-only)

// JobController.hpp:13 using StatusFn = std::function<void(unsigned, const std::string&)>;
pub type StatusFn = Box<dyn Fn(u32, &str)>;
// JobController.hpp:14 using StopCond = std::function<bool(void)>;
pub type StopCond = Box<dyn Fn() -> bool>;
// JobController.hpp:15 using CancelFn = std::function<void(void)>;
// (Arc instead of Box: C++ `std::function` is copyable — SupportTreeBuildsteps
//  copies it into its `m_thr` member (SupportTreeBuildsteps.cpp:43) while the
//  builder stays mutably shared, and `sla::normals` shares it across worker
//  threads (IndexedMesh.cpp:346-347), hence the `Send + Sync` bounds.)
pub type CancelFn = std::sync::Arc<dyn Fn() + Send + Sync>;

/// A Control structure for the support calculation. Consists of the status
/// indicator callback and the stop condition predicate.
// JobController.hpp:11
pub struct JobController {
    // This will signal the status of the calculation to the front-end
    // JobController.hpp:18 StatusFn statuscb = [](unsigned, const std::string&){};
    pub statuscb: StatusFn,

    // Returns true if the calculation should be aborted.
    // JobController.hpp:21 StopCond stopcondition = [](){ return false; };
    pub stopcondition: StopCond,

    // Similar to cancel callback. This should check the stop condition and
    // if true, throw an appropriate exception. (TriangleMeshSlicer needs this)
    // consider it a hard abort. stopcondition is permits the algorithm to
    // terminate itself
    // JobController.hpp:27 CancelFn cancelfn = [](){};
    pub cancelfn: CancelFn,
}

// C++ default member initializers (JobController.hpp:18,21,27) — the struct is
// default-constructible with no-op callbacks.
impl Default for JobController {
    fn default() -> Self {
        Self {
            // JobController.hpp:18
            statuscb: Box::new(|_, _| {}),
            // JobController.hpp:21
            stopcondition: Box::new(|| false),
            // JobController.hpp:27
            cancelfn: std::sync::Arc::new(|| {}),
        }
    }
}
