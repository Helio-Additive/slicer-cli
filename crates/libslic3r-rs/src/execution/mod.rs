//! Execution policy framework.
//!
//! Provides trait-based execution policies for parallel and sequential operations.
//! C++ Reference: Execution/Execution.hpp, ExecutionSeq.hpp, ExecutionTBB.hpp

pub mod execution;
pub mod execution_seq;
pub mod execution_tbb;

// Re-export key types
pub use execution::ExecutionPolicy;
pub use execution_seq::{SequentialPolicy, EX_SEQ};
pub use execution_tbb::{ParallelPolicy, EX_TBB};
