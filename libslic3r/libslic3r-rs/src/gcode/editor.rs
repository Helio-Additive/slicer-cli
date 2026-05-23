//! GCodeEditor.rs - In-place editing of G-code commands.
//!
//! This module provides utilities for editing and modifying G-code files,
//! mirroring BambuStudio's GCode/GCodeEditor.cpp.
//!
//! Features:
//! - Search and replace in G-code
//! - Insert custom commands at specific points
//! - Delete or modify existing commands
//! - Batch transformations

use crate::gcode::{GCodeCommand, ParsedGCode};

/// Configuration for G-code editing operations.
#[derive(Debug, Clone)]
pub struct GCodeEditConfig {
    /// Preserve original comments
    pub preserve_comments: bool,
    /// Preserve empty lines
    pub preserve_empty_lines: bool,
    /// Normalize whitespace
    pub normalize_whitespace: bool,
}

impl Default for GCodeEditConfig {
    fn default() -> Self {
        Self {
            preserve_comments: true,
            preserve_empty_lines: true,
            normalize_whitespace: false,
        }
    }
}

/// Represents a G-code edit operation.
#[derive(Debug, Clone)]
pub enum EditOperation {
    /// Replace a command with another
    Replace {
        index: usize,
        new_command: GCodeCommand,
    },
    /// Insert a command at an index
    Insert { index: usize, command: GCodeCommand },
    /// Delete a command at an index
    Delete { index: usize },
    /// Find and replace text in commands
    FindReplace {
        pattern: String,
        replacement: String,
    },
    /// Insert at a specific Z height
    InsertAtZ {
        z: f64,
        command: GCodeCommand,
        tolerance: f64,
    },
}

/// G-code editor for performing modifications.
pub struct GCodeEditor {
    config: GCodeEditConfig,
    operations: Vec<EditOperation>,
}

impl GCodeEditor {
    // Create a new G-code editor.
    pub fn new(config: GCodeEditConfig) -> Self {
        Self {
            config,
            operations: Vec::new(),
        }
    }

    /// Create with default configuration.
    pub fn default_editor() -> Self {
        Self::new(GCodeEditConfig::default())
    }

    /// Add an edit operation.
    pub fn add_operation(&mut self, op: EditOperation) {
        self.operations.push(op);
    }

    /// Replace a command at the specified index.
    pub fn replace(&mut self, index: usize, command: GCodeCommand) {
        self.add_operation(EditOperation::Replace {
            index,
            new_command: command,
        });
    }

    /// Insert a command at the specified index.
    pub fn insert(&mut self, index: usize, command: GCodeCommand) {
        self.add_operation(EditOperation::Insert { index, command });
    }

    /// Delete a command at the specified index.
    pub fn delete(&mut self, index: usize) {
        self.add_operation(EditOperation::Delete { index });
    }

    /// Find and replace text in all commands.
    pub fn find_replace(&mut self, pattern: &str, replacement: &str) {
        self.add_operation(EditOperation::FindReplace {
            pattern: pattern.to_string(),
            replacement: replacement.to_string(),
        });
    }

    /// Insert a command near a specific Z height.
    pub fn insert_at_z(&mut self, z: f64, command: GCodeCommand, tolerance: f64) {
        self.add_operation(EditOperation::InsertAtZ {
            z,
            command,
            tolerance,
        });
    }

    /// Apply all operations to parsed G-code.
    pub fn edit(&self, gcode: &mut ParsedGCode) {
        for op in &self.operations {
            match op {
                EditOperation::Replace { index, new_command } => {
                    if *index < gcode.commands.len() {
                        gcode.commands[*index] = new_command.clone();
                    }
                }
                EditOperation::Insert { index, command } => {
                    let idx = index.min(gcode.commands.len());
                    gcode.commands.insert(idx, command.clone());
                }
                EditOperation::Delete { index } => {
                    if *index < gcode.commands.len() {
                        gcode.commands.remove(*index);
                    }
                }
                EditOperation::FindReplace {
                    pattern,
                    replacement,
                } => {
                    for cmd in &mut gcode.commands {
                        let cmd_str = cmd.to_string();
                        let new_str = cmd_str.replace(pattern, replacement);
                        if new_str != cmd_str {
                            *cmd = GCodeCommand::from_line(&new_str).unwrap_or(cmd.clone());
                        }
                    }
                }
                EditOperation::InsertAtZ {
                    z,
                    command,
                    tolerance,
                } => {
                    let mut insert_idx = gcode.commands.len();
                    for (i, cmd) in gcode.commands.iter().enumerate() {
                        if let Some(cmd_z) = Self::extract_z_from_command(cmd) {
                            if (cmd_z - *z).abs() < *tolerance {
                                insert_idx = i;
                                break;
                            } else if cmd_z > *z {
                                insert_idx = i;
                                break;
                            }
                        }
                    }
                    gcode.commands.insert(insert_idx, command.clone());
                }
            }
        }
    }

    /// Extract Z coordinate from a G-code command if present.
    fn extract_z_from_command(cmd: &GCodeCommand) -> Option<f64> {
        let cmd_str = cmd.to_string();
        Self::extract_z_from_line(&cmd_str)
    }

    /// Extract Z value from G-code line.
    fn extract_z_from_line(line: &str) -> Option<f64> {
        let parts: Vec<&str> = line.split_whitespace().collect();
        for part in parts {
            if part.starts_with('Z') || part.starts_with("z:") {
                let z_str = &part[1..];
                return z_str.parse().ok();
            }
        }
        None
    }

    /// Clear all operations.
    pub fn clear_operations(&mut self) {
        self.operations.clear();
    }

    /// Get the number of pending operations.
    pub fn operation_count(&self) -> usize {
        self.operations.len()
    }
}

/// Convenience function to perform a single replace operation.
pub fn replace_command(gcode: &mut ParsedGCode, index: usize, command: GCodeCommand) {
    let editor = GCodeEditor::default_editor();
    let mut editor = editor;
    editor.replace(index, command);
    editor.edit(gcode);
}

/// Convenience function to insert a command at an index.
pub fn insert_command(gcode: &mut ParsedGCode, index: usize, command: GCodeCommand) {
    let mut editor = GCodeEditor::default_editor();
    editor.insert(index, command);
    editor.edit(gcode);
}

/// Convenience function to delete a command.
pub fn delete_command(gcode: &mut ParsedGCode, index: usize) {
    let mut editor = GCodeEditor::default_editor();
    editor.delete(index);
    editor.edit(gcode);
}

/// Batch find and replace in G-code.
pub fn find_replace_all(gcode: &mut ParsedGCode, pattern: &str, replacement: &str) {
    let mut editor = GCodeEditor::default_editor();
    editor.find_replace(pattern, replacement);
    editor.edit(gcode);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_editor_default() {
        let editor = GCodeEditor::default_editor();
        assert_eq!(editor.operation_count(), 0);
        assert!(editor.config.preserve_comments);
    }

    #[test]
    fn test_extract_z_from_line() {
        assert_eq!(
            GCodeEditor::extract_z_from_line("G1 X10 Y20 Z0.3 E1.5"),
            Some(0.3)
        );
        assert_eq!(GCodeEditor::extract_z_from_line("G28 ; Home"), None);
        assert_eq!(GCodeEditor::extract_z_from_line("M104 S200"), None);
    }

    #[test]
    fn test_edit_config_default() {
        let config = GCodeEditConfig::default();
        assert!(config.preserve_comments);
        assert!(config.preserve_empty_lines);
        assert!(!config.normalize_whitespace);
    }
}
