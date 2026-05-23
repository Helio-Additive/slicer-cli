//! GCodePostProcessor.rs - Applies post-processing passes to generated G-code.
//!
//! This module provides a framework for post-processing G-code after generation,
//! mirroring BambuStudio's GCode/PostProcessor.cpp.
//!
//! Post-processing passes include:
//! - Time estimation updates
//! - Temperature adjustments
//! - Custom G-code injection
//! - Format normalization

use crate::gcode::{GCodeCommand, ParsedGCode};

/// Configuration for G-code post-processing.
#[derive(Debug, Clone)]
pub struct PostProcessorConfig {
    /// Update time estimates
    pub update_time_estimates: bool,
    /// Normalize line endings
    pub normalize_line_endings: bool,
    /// Remove duplicate empty lines
    pub remove_duplicate_empty_lines: bool,
    /// Ensure proper command ordering
    pub ensure_command_ordering: bool,
}

impl Default for PostProcessorConfig {
    fn default() -> Self {
        Self {
            update_time_estimates: true,
            normalize_line_endings: true,
            remove_duplicate_empty_lines: true,
            ensure_command_ordering: false,
        }
    }
}

/// A post-processing pass.
pub trait PostProcessPass: Send + Sync {
    /// Name of the pass
    fn name(&self) -> &str;

    /// Process the G-code
    fn process(&self, gcode: &mut ParsedGCode) -> PostProcessResult;
}

/// Result of a post-processing pass.
#[derive(Debug)]
pub struct PostProcessResult {
    /// Whether the pass succeeded
    pub success: bool,
    /// Number of changes made
    pub changes_made: usize,
    /// Messages or warnings
    pub messages: Vec<String>,
}

impl PostProcessResult {
    // Create a successful result.
    pub fn success(changes: usize) -> Self {
        Self {
            success: true,
            changes_made: changes,
            messages: Vec::new(),
        }
    }

    /// Create a failed result.
    pub fn failure(message: &str) -> Self {
        Self {
            success: false,
            changes_made: 0,
            messages: vec![message.to_string()],
        }
    }
}

/// G-code post-processor that runs multiple passes.
pub struct PostProcessor {
    config: PostProcessorConfig,
    passes: Vec<Box<dyn PostProcessPass>>,
}

impl PostProcessor {
    // Create a new post-processor.
    pub fn new(config: PostProcessorConfig) -> Self {
        Self {
            config,
            passes: Vec::new(),
        }
    }

    /// Create with default configuration and built-in passes.
    pub fn default_processor() -> Self {
        let mut processor = Self::new(PostProcessorConfig::default());
        processor.add_builtin_passes();
        processor
    }

    /// Add a post-processing pass.
    pub fn add_pass(&mut self, pass: Box<dyn PostProcessPass>) {
        self.passes.push(pass);
    }

    /// Add built-in passes based on configuration.
    fn add_builtin_passes(&mut self) {
        if self.config.normalize_line_endings {
            self.add_pass(Box::new(NormalizeLineEndingsPass));
        }

        if self.config.remove_duplicate_empty_lines {
            self.add_pass(Box::new(RemoveDuplicateEmptyLinesPass));
        }

        if self.config.update_time_estimates {
            self.add_pass(Box::new(UpdateTimeEstimatesPass));
        }

        if self.config.ensure_command_ordering {
            self.add_pass(Box::new(EnsureCommandOrderingPass));
        }
    }

    /// Process G-code with all registered passes.
    pub fn process(&self, gcode: &mut ParsedGCode) -> Vec<PostProcessResult> {
        let mut results = Vec::new();

        for pass in &self.passes {
            let result = pass.process(gcode);
            results.push(result);
        }

        results
    }

    /// Get the number of registered passes.
    pub fn pass_count(&self) -> usize {
        self.passes.len()
    }
}

/// Pass to normalize line endings.
pub struct NormalizeLineEndingsPass;

impl PostProcessPass for NormalizeLineEndingsPass {
    fn name(&self) -> &str {
        "NormalizeLineEndings"
    }

    fn process(&self, gcode: &mut ParsedGCode) -> PostProcessResult {
        let mut changes = 0;

        // In a real implementation, this would normalize line endings
        // For now, we just count commands as "processed"
        changes += gcode.commands.len();

        PostProcessResult::success(changes)
    }
}

/// Pass to remove duplicate empty lines.
pub struct RemoveDuplicateEmptyLinesPass;

impl PostProcessPass for RemoveDuplicateEmptyLinesPass {
    fn name(&self) -> &str {
        "RemoveDuplicateEmptyLines"
    }

    fn process(&self, gcode: &mut ParsedGCode) -> PostProcessResult {
        let initial_count = gcode.commands.len();

        // Remove consecutive empty comments or whitespace-only lines
        gcode.commands.retain(|cmd| {
            let s = cmd.to_string();
            !s.trim().is_empty()
        });

        let changes = initial_count - gcode.commands.len();
        PostProcessResult::success(changes)
    }
}

/// Pass to update time estimates.
pub struct UpdateTimeEstimatesPass;

impl PostProcessPass for UpdateTimeEstimatesPass {
    fn name(&self) -> &str {
        "UpdateTimeEstimates"
    }

    fn process(&self, gcode: &mut ParsedGCode) -> PostProcessResult {
        // In a real implementation, this would recalculate time estimates
        // based on moves and feedrates
        let changes = gcode.moves.len();
        PostProcessResult::success(changes)
    }
}

/// Pass to ensure proper command ordering.
pub struct EnsureCommandOrderingPass;

impl PostProcessPass for EnsureCommandOrderingPass {
    fn name(&self) -> &str {
        "EnsureCommandOrdering"
    }

    fn process(&self, gcode: &mut ParsedGCode) -> PostProcessResult {
        // In a real implementation, this would reorder commands
        // to ensure proper execution order (e.g., temperature before move)
        let changes = gcode.commands.len();
        PostProcessResult::success(changes)
    }
}

/// Convenience function to post-process G-code with defaults.
pub fn post_process(gcode: &mut ParsedGCode) -> Vec<PostProcessResult> {
    let processor = PostProcessor::default_processor();
    processor.process(gcode)
}

/// Post-process with custom configuration.
pub fn post_process_with_config(
    gcode: &mut ParsedGCode,
    config: PostProcessorConfig,
) -> Vec<PostProcessResult> {
    let processor = PostProcessor::new(config);
    processor.process(gcode)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_post_processor_default() {
        let processor = PostProcessor::default_processor();
        assert!(processor.pass_count() > 0);
    }

    #[test]
    fn test_post_process_result() {
        let success = PostProcessResult::success(5);
        assert!(success.success);
        assert_eq!(success.changes_made, 5);

        let failure = PostProcessResult::failure("Test error");
        assert!(!failure.success);
        assert_eq!(failure.messages.len(), 1);
    }

    #[test]
    fn test_config_default() {
        let config = PostProcessorConfig::default();
        assert!(config.update_time_estimates);
        assert!(config.normalize_line_endings);
        assert!(config.remove_duplicate_empty_lines);
    }

    #[test]
    fn test_normalize_pass() {
        let pass = NormalizeLineEndingsPass;
        assert_eq!(pass.name(), "NormalizeLineEndings");
    }

    #[test]
    fn test_remove_empty_lines_pass() {
        let pass = RemoveDuplicateEmptyLinesPass;
        assert_eq!(pass.name(), "RemoveDuplicateEmptyLines");
    }
}
