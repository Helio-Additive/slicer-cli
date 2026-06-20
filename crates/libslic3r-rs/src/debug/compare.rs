//! Semantic G-code comparison module.
//!
//! This module provides tools for comparing G-code files semantically,
//! focusing on toolpath equivalence rather than byte-for-byte matching.
//!
//! # Overview
//!
//! Two G-code files can be considered semantically equivalent if they produce
//! the same physical result when printed, even if the exact commands differ.
//! This module enables comparison with configurable tolerances for:
//! - Position (X/Y/Z coordinates)
//! - Extrusion amounts (E values)
//! - Feed rates (F values)
//! - Layer structure
//!
//! # Extrusion Tracking
//!
//! The module properly handles:
//! - Absolute extrusion mode (M82) - E values are absolute positions
//! - Relative extrusion mode (M83) - E values are incremental deltas
//! - E resets (G92 E0) - Resets the E position without physical movement
//!
//! # Example
//!
//! ```rust,ignore
//! use slicer::gcode::compare::{GCodeComparator, ComparisonConfig};
//!
//! let config = ComparisonConfig::default();
//! let comparator = GCodeComparator::new(config);
//!
//! let result = comparator.compare_files("reference.gcode", "generated.gcode")?;
//! println!("Match percentage: {:.1}%", result.match_percentage());
//! ```

use std::collections::HashMap;
use std::fs;
use std::io::{self, BufRead};
use std::path::Path;

/// Extrusion mode for G-code.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ExtrusionMode {
    /// Absolute extrusion (M82) - E values are absolute positions.
    #[default]
    Absolute,
    /// Relative extrusion (M83) - E values are deltas.
    Relative,
}

/// Tracks extrusion state across G-code parsing.
///
/// Handles the complexity of:
/// - Absolute vs relative extrusion modes
/// - E resets (G92 E0)
/// - Accumulating total extrusion correctly
#[derive(Debug, Clone)]
pub struct ExtrusionTracker {
    /// Current extrusion mode.
    mode: ExtrusionMode,
    /// Current E position (in absolute mode, this is the E coordinate;
    /// in relative mode, this is tracked for consistency).
    e_position: f64,
    /// Total extrusion accumulated (always positive filament used).
    total_extrusion: f64,
    /// The E offset applied by G92 resets.
    e_offset: f64,
}

impl Default for ExtrusionTracker {
    fn default() -> Self {
        Self::new()
    }
}

impl ExtrusionTracker {
    // Create a new extrusion tracker starting in absolute mode.
    pub fn new() -> Self {
        Self {
            mode: ExtrusionMode::Absolute,
            e_position: 0.0,
            total_extrusion: 0.0,
            e_offset: 0.0,
        }
    }

    /// Get the current extrusion mode.
    pub fn mode(&self) -> ExtrusionMode {
        self.mode
    }

    /// Set to absolute extrusion mode (M82).
    pub fn set_absolute(&mut self) {
        self.mode = ExtrusionMode::Absolute;
    }

    /// Set to relative extrusion mode (M83).
    pub fn set_relative(&mut self) {
        self.mode = ExtrusionMode::Relative;
    }

    /// Handle an E reset (G92 E<value>).
    pub fn reset_e(&mut self, new_value: f64) {
        // Calculate the offset so that future E values are interpreted correctly
        self.e_offset = self.e_position - new_value;
        self.e_position = new_value;
    }

    /// Process an E value from a move command.
    /// Returns the extrusion delta for this move (positive for extrusion, negative for retraction).
    pub fn process_e(&mut self, e_value: f64) -> f64 {
        let delta = match self.mode {
            ExtrusionMode::Absolute => {
                // In absolute mode, delta is the difference from current position
                let delta = e_value - self.e_position;
                self.e_position = e_value;
                delta
            }
            ExtrusionMode::Relative => {
                // In relative mode, the value IS the delta
                self.e_position += e_value;
                e_value
            }
        };

        // Only count positive extrusion (not retractions)
        if delta > 0.0 {
            self.total_extrusion += delta;
        }

        delta
    }

    /// Get the total extrusion accumulated so far.
    pub fn total_extrusion(&self) -> f64 {
        self.total_extrusion
    }

    /// Get the current E position.
    pub fn e_position(&self) -> f64 {
        self.e_position
    }

    /// Get the real E position accounting for any offsets.
    pub fn real_e_position(&self) -> f64 {
        self.e_position + self.e_offset
    }
}

/// Configuration for G-code comparison.
#[derive(Debug, Clone)]
pub struct ComparisonConfig {
    /// Position tolerance in mm (default: 0.01mm = 10 microns)
    pub position_tolerance: f64,

    /// Extrusion tolerance as a fraction (default: 0.05 = 5%)
    pub extrusion_tolerance: f64,

    /// Feed rate tolerance as a fraction (default: 0.1 = 10%)
    pub feed_rate_tolerance: f64,

    /// Z height tolerance in mm (default: 0.001mm = 1 micron)
    pub z_tolerance: f64,

    /// Whether to compare layer counts
    pub compare_layer_count: bool,

    /// Whether to compare total extrusion
    pub compare_total_extrusion: bool,

    /// Whether to ignore travel moves (G0)
    pub ignore_travel_moves: bool,

    /// Whether to ignore comments
    pub ignore_comments: bool,

    /// Whether to compare in order (false allows reordering within layers)
    pub strict_ordering: bool,
}

impl Default for ComparisonConfig {
    fn default() -> Self {
        Self {
            position_tolerance: 0.01,  // 10 microns
            extrusion_tolerance: 0.05, // 5%
            feed_rate_tolerance: 0.1,  // 10%
            z_tolerance: 0.001,        // 1 micron
            compare_layer_count: true,
            compare_total_extrusion: true,
            ignore_travel_moves: false,
            ignore_comments: true,
            strict_ordering: false,
        }
    }
}

impl ComparisonConfig {
    // Create a strict comparison config (small tolerances, strict ordering).
    pub fn strict() -> Self {
        Self {
            position_tolerance: 0.001,
            extrusion_tolerance: 0.01,
            feed_rate_tolerance: 0.05,
            z_tolerance: 0.0001,
            compare_layer_count: true,
            compare_total_extrusion: true,
            ignore_travel_moves: false,
            ignore_comments: true,
            strict_ordering: true,
        }
    }

    /// Create a relaxed comparison config (larger tolerances, flexible ordering).
    pub fn relaxed() -> Self {
        Self {
            position_tolerance: 0.1,
            extrusion_tolerance: 0.1,
            feed_rate_tolerance: 0.2,
            z_tolerance: 0.01,
            compare_layer_count: true,
            compare_total_extrusion: true,
            ignore_travel_moves: true,
            ignore_comments: true,
            strict_ordering: false,
        }
    }
}

/// A parsed G-code move command.
#[derive(Debug, Clone, PartialEq)]
pub struct GCodeMove {
    /// Command type (G0, G1, G2, G3)
    pub command: String,
    /// X coordinate (if present)
    pub x: Option<f64>,
    /// Y coordinate (if present)
    pub y: Option<f64>,
    /// Z coordinate (if present)
    pub z: Option<f64>,
    /// Extrusion amount (if present)
    pub e: Option<f64>,
    /// Feed rate (if present)
    pub f: Option<f64>,
    /// Arc I parameter (for G2/G3)
    pub i: Option<f64>,
    /// Arc J parameter (for G2/G3)
    pub j: Option<f64>,
    /// Original line number in the file
    pub line_number: usize,
}

impl GCodeMove {
    // Parse a G-code move from a line of text.
    pub fn parse(line: &str, line_number: usize) -> Option<Self> {
        let line = line.trim();

        // Skip comments and empty lines
        if line.is_empty() || line.starts_with(';') {
            return None;
        }

        // Check if it's a move command
        if !line.starts_with("G0")
            && !line.starts_with("G1")
            && !line.starts_with("G2")
            && !line.starts_with("G3")
        {
            return None;
        }

        let parts: Vec<&str> = line.split_whitespace().collect();
        if parts.is_empty() {
            return None;
        }

        let command = parts[0].to_string();
        let mut mov = GCodeMove {
            command,
            x: None,
            y: None,
            z: None,
            e: None,
            f: None,
            i: None,
            j: None,
            line_number,
        };

        for part in &parts[1..] {
            if part.starts_with(';') {
                break; // End of command, rest is comment
            }

            if part.len() < 2 {
                continue;
            }

            let (code, value) = part.split_at(1);
            if let Ok(v) = value.parse::<f64>() {
                match code {
                    "X" => mov.x = Some(v),
                    "Y" => mov.y = Some(v),
                    "Z" => mov.z = Some(v),
                    "E" => mov.e = Some(v),
                    "F" => mov.f = Some(v),
                    "I" => mov.i = Some(v),
                    "J" => mov.j = Some(v),
                    _ => {}
                }
            }
        }

        Some(mov)
    }

    /// Check if this is a travel move (no extrusion).
    pub fn is_travel(&self) -> bool {
        self.e.is_none() || self.command == "G0"
    }

    /// Check if this is an extrusion move.
    pub fn is_extrusion(&self) -> bool {
        self.e.is_some() && self.command != "G0"
    }

    /// Check if this move has position data.
    pub fn has_position(&self) -> bool {
        self.x.is_some() || self.y.is_some() || self.z.is_some()
    }

    /// Get the 2D distance to another move (X/Y only).
    pub fn distance_2d(&self, other: &GCodeMove) -> f64 {
        let dx = self.x.unwrap_or(0.0) - other.x.unwrap_or(0.0);
        let dy = self.y.unwrap_or(0.0) - other.y.unwrap_or(0.0);
        (dx * dx + dy * dy).sqrt()
    }

    /// Get the 3D distance to another move.
    pub fn distance_3d(&self, other: &GCodeMove) -> f64 {
        let dx = self.x.unwrap_or(0.0) - other.x.unwrap_or(0.0);
        let dy = self.y.unwrap_or(0.0) - other.y.unwrap_or(0.0);
        let dz = self.z.unwrap_or(0.0) - other.z.unwrap_or(0.0);
        (dx * dx + dy * dy + dz * dz).sqrt()
    }
}

/// Information about a layer in G-code.
#[derive(Debug, Clone)]
pub struct LayerInfo {
    /// Layer number (0-indexed)
    pub layer_num: usize,
    /// Z height of this layer
    pub z_height: f64,
    /// Line number where this layer starts
    pub line_start: usize,
    /// Line number where this layer ends
    pub line_end: usize,
    /// All moves in this layer
    pub moves: Vec<GCodeMove>,
    /// Total extrusion in this layer
    pub total_extrusion: f64,
    /// Total travel distance in this layer
    pub total_travel: f64,
}

impl LayerInfo {
    // Create a new layer info.
    pub fn new(layer_num: usize, z_height: f64, line_start: usize) -> Self {
        Self {
            layer_num,
            z_height,
            line_start,
            line_end: line_start,
            moves: Vec::new(),
            total_extrusion: 0.0,
            total_travel: 0.0,
        }
    }

    /// Add a move to this layer.
    pub fn add_move(&mut self, mov: GCodeMove) {
        if let Some(e) = mov.e {
            if e > 0.0 {
                self.total_extrusion += e;
            }
        }
        self.moves.push(mov);
    }

    /// Get extrusion moves only.
    pub fn extrusion_moves(&self) -> Vec<&GCodeMove> {
        self.moves.iter().filter(|m| m.is_extrusion()).collect()
    }

    /// Get travel moves only.
    pub fn travel_moves(&self) -> Vec<&GCodeMove> {
        self.moves.iter().filter(|m| m.is_travel()).collect()
    }
}

/// Parsed G-code file with layer information.
#[derive(Debug)]
pub struct ParsedGCode {
    /// Path to the file (if loaded from file)
    pub path: Option<String>,
    /// All layers in the G-code
    pub layers: Vec<LayerInfo>,
    /// Total number of moves
    pub total_moves: usize,
    /// Total extrusion amount
    pub total_extrusion: f64,
    /// All raw lines
    pub lines: Vec<String>,
    /// Settings extracted from comments
    pub settings: HashMap<String, String>,
}

impl ParsedGCode {
    // Parse G-code from a string.
    pub fn from_string(content: &str) -> Self {
        let lines: Vec<String> = content.lines().map(|s| s.to_string()).collect();
        Self::parse_lines(lines, None)
    }

    /// Parse G-code from a file.
    pub fn from_file<P: AsRef<Path>>(path: P) -> io::Result<Self> {
        let path_str = path.as_ref().to_string_lossy().to_string();
        let file = fs::File::open(path)?;
        let reader = io::BufReader::new(file);
        let lines: Vec<String> = reader.lines().collect::<Result<_, _>>()?;
        Ok(Self::parse_lines(lines, Some(path_str)))
    }

    fn parse_lines(lines: Vec<String>, path: Option<String>) -> Self {
        let mut layers = Vec::new();
        let mut current_layer: Option<LayerInfo> = None;
        let mut total_moves = 0;
        let mut settings = HashMap::new();
        let mut current_z = 0.0;

        // Use the new ExtrusionTracker for robust E handling
        let mut e_tracker = ExtrusionTracker::new();

        // First pass: detect if the file has explicit layer markers
        // If it does, we'll only use those for layer boundaries (not Z changes)
        let has_layer_markers = lines.iter().any(|line| {
            let trimmed = line.trim();
            Self::is_layer_marker(trimmed)
        });

        for (line_num, line) in lines.iter().enumerate() {
            let trimmed = line.trim();

            // Extract settings from comments
            if trimmed.starts_with(';') {
                if let Some((key, value)) = trimmed[1..].split_once('=') {
                    settings.insert(key.trim().to_string(), value.trim().to_string());
                } else if let Some((key, value)) = trimmed[1..].split_once(':') {
                    settings.insert(key.trim().to_string(), value.trim().to_string());
                }

                // Check for layer change markers
                if Self::is_layer_marker(trimmed) {
                    // Finalize current layer
                    if let Some(mut layer) = current_layer.take() {
                        layer.line_end = line_num.saturating_sub(1);
                        if !layer.moves.is_empty() {
                            layers.push(layer);
                        }
                    }
                    // Start new layer
                    let layer_num = layers.len();
                    current_layer = Some(LayerInfo::new(layer_num, current_z, line_num));
                }
                continue;
            }

            // Handle extrusion mode commands (strip comments first)
            let cmd_part = trimmed.split(';').next().unwrap_or("").trim();
            if cmd_part == "M82" {
                e_tracker.set_absolute();
                continue;
            }
            if cmd_part == "M83" {
                e_tracker.set_relative();
                continue;
            }

            // Handle E resets (G92 E<value>)
            if cmd_part.starts_with("G92") {
                if let Some(e_val) = Self::parse_g92_e(cmd_part) {
                    e_tracker.reset_e(e_val);
                }
                continue;
            }

            // Parse move commands
            if let Some(mov) = GCodeMove::parse(trimmed, line_num) {
                // Track Z changes
                if let Some(z) = mov.z {
                    if (z - current_z).abs() > 0.001 {
                        current_z = z;
                        // Only use Z-change layer detection if there are NO explicit layer markers
                        if !has_layer_markers {
                            if current_layer.is_none()
                                || current_layer.as_ref().map_or(true, |l| {
                                    (l.z_height - z).abs() > 0.001 && !l.moves.is_empty()
                                })
                            {
                                // Finalize current layer
                                if let Some(mut layer) = current_layer.take() {
                                    layer.line_end = line_num.saturating_sub(1);
                                    if !layer.moves.is_empty() {
                                        layers.push(layer);
                                    }
                                }
                                // Start new layer
                                let layer_num = layers.len();
                                current_layer = Some(LayerInfo::new(layer_num, z, line_num));
                            }
                        } else {
                            // Update current layer's z_height when we have markers
                            // This tracks the actual print height within the layer
                            if let Some(ref mut layer) = current_layer {
                                // Only update z_height if it's a print move (not a Z-hop)
                                // A print move typically has positive extrusion
                                if mov.e.map_or(false, |e| e > 0.0) {
                                    layer.z_height = z;
                                }
                            }
                        }
                    }
                }

                // Track extrusion using the robust tracker
                if let Some(e) = mov.e {
                    e_tracker.process_e(e);
                }

                total_moves += 1;

                // Add move to current layer
                if let Some(ref mut layer) = current_layer {
                    layer.add_move(mov);
                }
            }
        }

        // Finalize last layer
        if let Some(mut layer) = current_layer {
            layer.line_end = lines.len().saturating_sub(1);
            if !layer.moves.is_empty() {
                layers.push(layer);
            }
        }

        Self {
            path,
            layers,
            total_moves,
            total_extrusion: e_tracker.total_extrusion(),
            lines,
            settings,
        }
    }

    /// Check if a comment line is a layer marker
    fn is_layer_marker(trimmed: &str) -> bool {
        // Supported formats:
        //   - "; LAYER:N" or ";LAYER:N" (PrusaSlicer/Cura style)
        //   - "; Layer N, Z = X.XXX" (our slicer format)
        //   - "; CHANGE_LAYER" (BambuStudio format)
        //
        // Note: We must be careful not to match substrings like "First layer:" in
        // comment lines like "; Layer height: 0.20mm, First layer: 0.20mm"
        //
        // Note: BambuStudio uses both "; CHANGE_LAYER" and "; layer num/total_layer_count:"
        // for each layer. We only match "; CHANGE_LAYER" to avoid double-counting layers.

        // Check for PrusaSlicer/Cura style: "; LAYER:" or ";LAYER:" at start
        let after_semicolon = trimmed.strip_prefix(';').map(|s| s.trim_start());
        if let Some(content) = after_semicolon {
            if content.starts_with("LAYER:") || content.starts_with("layer:") {
                return true;
            }
        }

        // Check for our slicer format: "; Layer N, Z = X.XXX"
        if trimmed.starts_with("; Layer ") && trimmed.contains(", Z =") {
            return true;
        }

        // Check for BambuStudio format: "; CHANGE_LAYER"
        // Note: We intentionally do NOT match "; layer num/total_layer_count:" because
        // BambuStudio emits both markers for each layer, and we want to avoid double-counting.
        if trimmed == "; CHANGE_LAYER" {
            return true;
        }

        false
    }

    /// Parse E value from a G92 command (e.g., "G92 E0" or "G92 E-1.5")
    fn parse_g92_e(line: &str) -> Option<f64> {
        for part in line.split_whitespace() {
            if part.starts_with('E') || part.starts_with('e') {
                if let Ok(val) = part[1..].parse::<f64>() {
                    return Some(val);
                }
            }
        }
        None
    }

    /// Get the number of layers.
    pub fn layer_count(&self) -> usize {
        self.layers.len()
    }

    /// Get all Z heights.
    pub fn z_heights(&self) -> Vec<f64> {
        self.layers.iter().map(|l| l.z_height).collect()
    }

    /// Get a specific layer by index.
    pub fn layer(&self, index: usize) -> Option<&LayerInfo> {
        self.layers.get(index)
    }

    /// Get all extrusion moves across all layers.
    pub fn all_extrusion_moves(&self) -> Vec<&GCodeMove> {
        self.layers
            .iter()
            .flat_map(|l| l.extrusion_moves())
            .collect()
    }
}

/// Result of comparing two moves.
#[derive(Debug, Clone)]
pub struct MoveComparison {
    /// The reference move
    pub reference: GCodeMove,
    /// The generated move (if found)
    pub generated: Option<GCodeMove>,
    /// Position difference (2D distance)
    pub position_diff: f64,
    /// Extrusion difference
    pub extrusion_diff: f64,
    /// Whether the moves match within tolerances
    pub matches: bool,
    /// Reason for mismatch (if any)
    pub mismatch_reason: Option<String>,
}

/// Result of comparing two layers.
#[derive(Debug)]
pub struct LayerComparison {
    /// Layer number
    pub layer_num: usize,
    /// Z height difference
    pub z_diff: f64,
    /// Number of moves in reference
    pub ref_move_count: usize,
    /// Number of moves in generated
    pub gen_move_count: usize,
    /// Extrusion difference
    pub extrusion_diff: f64,
    /// Extrusion difference as percentage
    pub extrusion_diff_percent: f64,
    /// Individual move comparisons
    pub move_comparisons: Vec<MoveComparison>,
    /// Percentage of moves that match
    pub match_percentage: f64,
}

/// Overall comparison result.
#[derive(Debug)]
pub struct ComparisonResult {
    /// Configuration used for comparison
    pub config: ComparisonConfig,
    /// Reference file path
    pub reference_path: Option<String>,
    /// Generated file path
    pub generated_path: Option<String>,
    /// Layer-by-layer comparisons
    pub layer_comparisons: Vec<LayerComparison>,
    /// Total moves in reference
    pub ref_total_moves: usize,
    /// Total moves in generated
    pub gen_total_moves: usize,
    /// Total extrusion in reference
    pub ref_total_extrusion: f64,
    /// Total extrusion in generated
    pub gen_total_extrusion: f64,
    /// Layer count difference
    pub layer_count_diff: i32,
    /// Overall match percentage
    pub overall_match_percentage: f64,
    /// Summary of issues found
    pub issues: Vec<String>,
}

impl ComparisonResult {
    // Get the match percentage (0-100).
    pub fn match_percentage(&self) -> f64 {
        self.overall_match_percentage
    }

    /// Check if the comparison passed (above threshold).
    pub fn passed(&self, threshold: f64) -> bool {
        self.overall_match_percentage >= threshold
    }

    /// Get a summary string.
    pub fn summary(&self) -> String {
        let mut s = String::new();
        s.push_str(&format!("G-code Comparison Summary\n"));
        s.push_str(&format!("========================\n"));
        s.push_str(&format!(
            "Reference: {}\n",
            self.reference_path.as_deref().unwrap_or("(unknown)")
        ));
        s.push_str(&format!(
            "Generated: {}\n",
            self.generated_path.as_deref().unwrap_or("(unknown)")
        ));
        s.push_str(&format!("\n"));
        s.push_str(&format!(
            "Layer count: {} vs {} (diff: {})\n",
            self.layer_comparisons.len(),
            self.layer_comparisons.len(),
            self.layer_count_diff
        ));
        s.push_str(&format!(
            "Total moves: {} vs {}\n",
            self.ref_total_moves, self.gen_total_moves
        ));
        s.push_str(&format!(
            "Total extrusion: {:.2}mm vs {:.2}mm ({:.1}% diff)\n",
            self.ref_total_extrusion,
            self.gen_total_extrusion,
            if self.ref_total_extrusion > 0.0 {
                (self.gen_total_extrusion - self.ref_total_extrusion).abs()
                    / self.ref_total_extrusion
                    * 100.0
            } else {
                0.0
            }
        ));
        s.push_str(&format!("\n"));
        s.push_str(&format!(
            "Overall match: {:.1}%\n",
            self.overall_match_percentage
        ));

        if !self.issues.is_empty() {
            s.push_str(&format!("\nIssues:\n"));
            for issue in &self.issues {
                s.push_str(&format!("  - {}\n", issue));
            }
        }

        s
    }
}

/// G-code semantic comparator.
pub struct GCodeComparator {
    config: ComparisonConfig,
}

impl GCodeComparator {
    // Create a new comparator with the given configuration.
    pub fn new(config: ComparisonConfig) -> Self {
        Self { config }
    }

    /// Create a comparator with default configuration.
    pub fn with_defaults() -> Self {
        Self::new(ComparisonConfig::default())
    }

    /// Compare two G-code files.
    pub fn compare_files<P: AsRef<Path>>(
        &self,
        reference: P,
        generated: P,
    ) -> io::Result<ComparisonResult> {
        let ref_gcode = ParsedGCode::from_file(reference)?;
        let gen_gcode = ParsedGCode::from_file(generated)?;
        Ok(self.compare(&ref_gcode, &gen_gcode))
    }

    /// Compare two parsed G-code structures.
    pub fn compare(&self, reference: &ParsedGCode, generated: &ParsedGCode) -> ComparisonResult {
        let mut issues = Vec::new();

        // Compare layer counts
        let layer_count_diff = generated.layer_count() as i32 - reference.layer_count() as i32;
        if self.config.compare_layer_count && layer_count_diff != 0 {
            issues.push(format!(
                "Layer count differs: {} vs {} (diff: {})",
                reference.layer_count(),
                generated.layer_count(),
                layer_count_diff
            ));
        }

        // Compare total extrusion
        let extrusion_diff_percent = if reference.total_extrusion > 0.0 {
            (generated.total_extrusion - reference.total_extrusion).abs()
                / reference.total_extrusion
        } else {
            0.0
        };

        if self.config.compare_total_extrusion
            && extrusion_diff_percent > self.config.extrusion_tolerance
        {
            issues.push(format!(
                "Total extrusion differs by {:.1}% (tolerance: {:.1}%)",
                extrusion_diff_percent * 100.0,
                self.config.extrusion_tolerance * 100.0
            ));
        }

        // Compare layers
        let mut layer_comparisons = Vec::new();
        let mut total_matching_moves = 0;
        let mut total_ref_moves = 0;

        let max_layers = reference.layer_count().max(generated.layer_count());
        for i in 0..max_layers {
            let ref_layer = reference.layer(i);
            let gen_layer = generated.layer(i);

            let comparison = self.compare_layers(ref_layer, gen_layer, i);
            total_ref_moves += comparison.ref_move_count;
            total_matching_moves +=
                (comparison.match_percentage / 100.0 * comparison.ref_move_count as f64) as usize;
            layer_comparisons.push(comparison);
        }

        let overall_match_percentage = if total_ref_moves > 0 {
            (total_matching_moves as f64 / total_ref_moves as f64) * 100.0
        } else {
            100.0
        };

        ComparisonResult {
            config: self.config.clone(),
            reference_path: reference.path.clone(),
            generated_path: generated.path.clone(),
            layer_comparisons,
            ref_total_moves: reference.total_moves,
            gen_total_moves: generated.total_moves,
            ref_total_extrusion: reference.total_extrusion,
            gen_total_extrusion: generated.total_extrusion,
            layer_count_diff,
            overall_match_percentage,
            issues,
        }
    }

    /// Compare two layers.
    fn compare_layers(
        &self,
        ref_layer: Option<&LayerInfo>,
        gen_layer: Option<&LayerInfo>,
        layer_num: usize,
    ) -> LayerComparison {
        let (ref_layer, gen_layer) = match (ref_layer, gen_layer) {
            (Some(r), Some(g)) => (r, g),
            (Some(r), None) => {
                return LayerComparison {
                    layer_num,
                    z_diff: r.z_height,
                    ref_move_count: r.moves.len(),
                    gen_move_count: 0,
                    extrusion_diff: r.total_extrusion,
                    extrusion_diff_percent: 100.0,
                    move_comparisons: Vec::new(),
                    match_percentage: 0.0,
                };
            }
            (None, Some(g)) => {
                return LayerComparison {
                    layer_num,
                    z_diff: g.z_height,
                    ref_move_count: 0,
                    gen_move_count: g.moves.len(),
                    extrusion_diff: g.total_extrusion,
                    extrusion_diff_percent: 100.0,
                    move_comparisons: Vec::new(),
                    match_percentage: 0.0,
                };
            }
            (None, None) => {
                return LayerComparison {
                    layer_num,
                    z_diff: 0.0,
                    ref_move_count: 0,
                    gen_move_count: 0,
                    extrusion_diff: 0.0,
                    extrusion_diff_percent: 0.0,
                    move_comparisons: Vec::new(),
                    match_percentage: 100.0,
                };
            }
        };

        let z_diff = (gen_layer.z_height - ref_layer.z_height).abs();
        let extrusion_diff = (gen_layer.total_extrusion - ref_layer.total_extrusion).abs();
        let extrusion_diff_percent = if ref_layer.total_extrusion > 0.0 {
            extrusion_diff / ref_layer.total_extrusion * 100.0
        } else {
            0.0
        };

        // Get moves to compare (optionally filtering travel moves)
        let ref_moves: Vec<&GCodeMove> = if self.config.ignore_travel_moves {
            ref_layer.extrusion_moves()
        } else {
            ref_layer.moves.iter().collect()
        };

        let gen_moves: Vec<&GCodeMove> = if self.config.ignore_travel_moves {
            gen_layer.extrusion_moves()
        } else {
            gen_layer.moves.iter().collect()
        };

        let mut move_comparisons = Vec::new();
        let mut matching_count = 0;

        if self.config.strict_ordering {
            // Compare moves in order
            for (i, ref_move) in ref_moves.iter().enumerate() {
                let gen_move = gen_moves.get(i).copied();
                let comparison = self.compare_moves(ref_move, gen_move);
                if comparison.matches {
                    matching_count += 1;
                }
                move_comparisons.push(comparison);
            }
        } else {
            // Find best match for each reference move
            let mut used_gen_moves = vec![false; gen_moves.len()];

            for ref_move in &ref_moves {
                let mut best_match: Option<(usize, f64)> = None;

                for (j, gen_move) in gen_moves.iter().enumerate() {
                    if used_gen_moves[j] {
                        continue;
                    }

                    let dist = ref_move.distance_2d(gen_move);
                    if dist <= self.config.position_tolerance {
                        if best_match.is_none() || dist < best_match.unwrap().1 {
                            best_match = Some((j, dist));
                        }
                    }
                }

                let gen_move = best_match.map(|(idx, _)| {
                    used_gen_moves[idx] = true;
                    gen_moves[idx]
                });

                let comparison = self.compare_moves(ref_move, gen_move);
                if comparison.matches {
                    matching_count += 1;
                }
                move_comparisons.push(comparison);
            }
        }

        let match_percentage = if ref_moves.is_empty() {
            100.0
        } else {
            (matching_count as f64 / ref_moves.len() as f64) * 100.0
        };

        LayerComparison {
            layer_num,
            z_diff,
            ref_move_count: ref_moves.len(),
            gen_move_count: gen_moves.len(),
            extrusion_diff,
            extrusion_diff_percent,
            move_comparisons,
            match_percentage,
        }
    }

    /// Compare two moves.
    fn compare_moves(
        &self,
        reference: &GCodeMove,
        generated: Option<&GCodeMove>,
    ) -> MoveComparison {
        let generated = match generated {
            Some(g) => g,
            None => {
                return MoveComparison {
                    reference: reference.clone(),
                    generated: None,
                    position_diff: f64::MAX,
                    extrusion_diff: reference.e.unwrap_or(0.0),
                    matches: false,
                    mismatch_reason: Some("No matching move found".to_string()),
                };
            }
        };

        let position_diff = reference.distance_2d(generated);
        let extrusion_diff = (reference.e.unwrap_or(0.0) - generated.e.unwrap_or(0.0)).abs();

        let mut mismatches = Vec::new();

        // Check position
        if position_diff > self.config.position_tolerance {
            mismatches.push(format!(
                "Position diff {:.4}mm > {:.4}mm",
                position_diff, self.config.position_tolerance
            ));
        }

        // Check Z
        if let (Some(ref_z), Some(gen_z)) = (reference.z, generated.z) {
            let z_diff = (ref_z - gen_z).abs();
            if z_diff > self.config.z_tolerance {
                mismatches.push(format!(
                    "Z diff {:.4}mm > {:.4}mm",
                    z_diff, self.config.z_tolerance
                ));
            }
        }

        // Check extrusion
        if let Some(ref_e) = reference.e {
            if ref_e > 0.0 {
                let e_diff_percent = extrusion_diff / ref_e;
                if e_diff_percent > self.config.extrusion_tolerance {
                    mismatches.push(format!(
                        "Extrusion diff {:.1}% > {:.1}%",
                        e_diff_percent * 100.0,
                        self.config.extrusion_tolerance * 100.0
                    ));
                }
            }
        }

        let matches = mismatches.is_empty();
        let mismatch_reason = if mismatches.is_empty() {
            None
        } else {
            Some(mismatches.join("; "))
        };

        MoveComparison {
            reference: reference.clone(),
            generated: Some(generated.clone()),
            position_diff,
            extrusion_diff,
            matches,
            mismatch_reason,
        }
    }
}

impl Default for GCodeComparator {
    fn default() -> Self {
        Self::with_defaults()
    }
}

/// Quick comparison function for convenience.
pub fn compare_gcode(reference: &str, generated: &str) -> ComparisonResult {
    let ref_parsed = ParsedGCode::from_string(reference);
    let gen_parsed = ParsedGCode::from_string(generated);
    GCodeComparator::with_defaults().compare(&ref_parsed, &gen_parsed)
}

/// Quick comparison function for files.
pub fn compare_gcode_files<P: AsRef<Path>>(
    reference: P,
    generated: P,
) -> io::Result<ComparisonResult> {
    GCodeComparator::with_defaults().compare_files(reference, generated)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_gcode_move_parse() {
        let mov = GCodeMove::parse("G1 X10.5 Y20.3 E0.5 F1200", 0).unwrap();
        assert_eq!(mov.command, "G1");
        assert_eq!(mov.x, Some(10.5));
        assert_eq!(mov.y, Some(20.3));
        assert_eq!(mov.e, Some(0.5));
        assert_eq!(mov.f, Some(1200.0));
    }

    #[test]
    fn test_gcode_move_parse_travel() {
        let mov = GCodeMove::parse("G0 X100 Y100 F3000", 0).unwrap();
        assert_eq!(mov.command, "G0");
        assert!(mov.is_travel());
        assert!(!mov.is_extrusion());
    }

    #[test]
    fn test_gcode_move_parse_with_comment() {
        let mov = GCodeMove::parse("G1 X10 Y20 ; move to start", 0).unwrap();
        assert_eq!(mov.x, Some(10.0));
        assert_eq!(mov.y, Some(20.0));
    }

    #[test]
    fn test_gcode_move_distance() {
        let m1 = GCodeMove::parse("G1 X0 Y0", 0).unwrap();
        let m2 = GCodeMove::parse("G1 X3 Y4", 0).unwrap();
        assert!((m1.distance_2d(&m2) - 5.0).abs() < 0.001);
    }

    #[test]
    fn test_parsed_gcode_from_string() {
        let gcode = r#"
; LAYER:0
G1 Z0.2 F3000
G1 X10 Y10 E0.1 F1200
G1 X20 Y10 E0.2
; LAYER:1
G1 Z0.4 F3000
G1 X10 Y10 E0.3
"#;
        let parsed = ParsedGCode::from_string(gcode);
        assert_eq!(parsed.layer_count(), 2);
        assert!(parsed.total_extrusion > 0.0);
    }

    #[test]
    fn test_comparison_config_default() {
        let config = ComparisonConfig::default();
        assert_eq!(config.position_tolerance, 0.01);
        assert_eq!(config.extrusion_tolerance, 0.05);
    }

    #[test]
    fn test_comparison_identical() {
        let gcode = r#"
G1 Z0.2 F3000
G1 X10 Y10 E0.1 F1200
G1 X20 Y10 E0.2
"#;
        let result = compare_gcode(gcode, gcode);
        assert!(result.match_percentage() >= 99.0);
    }

    #[test]
    fn test_comparison_small_diff() {
        let ref_gcode = r#"
G1 Z0.2 F3000
G1 X10.000 Y10.000 E0.100 F1200
G1 X20.000 Y10.000 E0.200
"#;
        let gen_gcode = r#"
G1 Z0.2 F3000
G1 X10.005 Y10.005 E0.101 F1200
G1 X20.005 Y10.005 E0.201
"#;
        let result = compare_gcode(ref_gcode, gen_gcode);
        // Small differences should still match with default tolerances
        assert!(result.match_percentage() >= 90.0);
    }

    #[test]
    fn test_comparison_large_diff() {
        let ref_gcode = r#"
G1 Z0.2 F3000
G1 X10 Y10 E0.1 F1200
"#;
        let gen_gcode = r#"
G1 Z0.2 F3000
G1 X50 Y50 E0.5 F1200
"#;
        let config = ComparisonConfig::strict();
        let comparator = GCodeComparator::new(config);
        let ref_parsed = ParsedGCode::from_string(ref_gcode);
        let gen_parsed = ParsedGCode::from_string(gen_gcode);
        let result = comparator.compare(&ref_parsed, &gen_parsed);
        // Large differences should result in mismatches being reported
        // With strict mode, the position diff (56.5mm) is way above tolerance (0.001mm)
        assert!(result.issues.len() > 0 || result.match_percentage() < 100.0);
        // Check that layer comparisons show mismatches
        if let Some(layer) = result.layer_comparisons.first() {
            // At least some moves should not match due to large position difference
            assert!(layer.move_comparisons.iter().any(|m| !m.matches));
        }
    }

    #[test]
    fn test_layer_detection_by_z() {
        let gcode = r#"
G1 Z0.2 F3000
G1 X10 Y10 E0.1
G1 Z0.4 F3000
G1 X20 Y20 E0.2
G1 Z0.6 F3000
G1 X30 Y30 E0.3
"#;
        let parsed = ParsedGCode::from_string(gcode);
        assert_eq!(parsed.layer_count(), 3);
    }

    #[test]
    fn test_comparison_summary() {
        let gcode = "G1 X10 Y10 E0.1 F1200\n";
        let result = compare_gcode(gcode, gcode);
        let summary = result.summary();
        assert!(summary.contains("G-code Comparison Summary"));
        assert!(summary.contains("Overall match"));
    }

    // ExtrusionTracker tests

    #[test]
    fn test_extrusion_tracker_new() {
        let tracker = ExtrusionTracker::new();
        assert_eq!(tracker.mode(), ExtrusionMode::Absolute);
        assert_eq!(tracker.e_position(), 0.0);
        assert_eq!(tracker.total_extrusion(), 0.0);
    }

    #[test]
    fn test_extrusion_tracker_absolute_mode() {
        let mut tracker = ExtrusionTracker::new();
        tracker.set_absolute();

        // First extrusion: E goes from 0 to 0.5
        let delta = tracker.process_e(0.5);
        assert!((delta - 0.5).abs() < 1e-6);
        assert!((tracker.total_extrusion() - 0.5).abs() < 1e-6);

        // Second extrusion: E goes from 0.5 to 1.0
        let delta = tracker.process_e(1.0);
        assert!((delta - 0.5).abs() < 1e-6);
        assert!((tracker.total_extrusion() - 1.0).abs() < 1e-6);

        // Third extrusion: E goes from 1.0 to 1.8
        let delta = tracker.process_e(1.8);
        assert!((delta - 0.8).abs() < 1e-6);
        assert!((tracker.total_extrusion() - 1.8).abs() < 1e-6);
    }

    #[test]
    fn test_extrusion_tracker_relative_mode() {
        let mut tracker = ExtrusionTracker::new();
        tracker.set_relative();

        // In relative mode, E values ARE the deltas
        let delta = tracker.process_e(0.5);
        assert!((delta - 0.5).abs() < 1e-6);
        assert!((tracker.total_extrusion() - 0.5).abs() < 1e-6);

        let delta = tracker.process_e(0.3);
        assert!((delta - 0.3).abs() < 1e-6);
        assert!((tracker.total_extrusion() - 0.8).abs() < 1e-6);

        let delta = tracker.process_e(0.7);
        assert!((delta - 0.7).abs() < 1e-6);
        assert!((tracker.total_extrusion() - 1.5).abs() < 1e-6);
    }

    #[test]
    fn test_extrusion_tracker_retraction() {
        let mut tracker = ExtrusionTracker::new();

        // Extrude
        tracker.process_e(1.0);
        assert!((tracker.total_extrusion() - 1.0).abs() < 1e-6);

        // Retract (negative delta in absolute mode means E decreases)
        let delta = tracker.process_e(0.0); // Retract back to 0
        assert!((delta - (-1.0)).abs() < 1e-6);
        // Total extrusion should NOT decrease (retractions don't add filament)
        assert!((tracker.total_extrusion() - 1.0).abs() < 1e-6);

        // Extrude again
        tracker.process_e(0.5);
        assert!((tracker.total_extrusion() - 1.5).abs() < 1e-6);
    }

    #[test]
    fn test_extrusion_tracker_e_reset() {
        let mut tracker = ExtrusionTracker::new();

        // Extrude to E=5.0
        tracker.process_e(5.0);
        assert!((tracker.total_extrusion() - 5.0).abs() < 1e-6);

        // Reset E to 0 (G92 E0)
        tracker.reset_e(0.0);
        assert!((tracker.e_position() - 0.0).abs() < 1e-6);
        // Total extrusion should be preserved
        assert!((tracker.total_extrusion() - 5.0).abs() < 1e-6);

        // Continue extruding from the reset position
        tracker.process_e(2.0);
        assert!((tracker.total_extrusion() - 7.0).abs() < 1e-6);
    }

    #[test]
    fn test_extrusion_tracker_mode_switch() {
        let mut tracker = ExtrusionTracker::new();

        // Start in absolute mode
        tracker.process_e(1.0);
        assert!((tracker.total_extrusion() - 1.0).abs() < 1e-6);

        // Switch to relative mode
        tracker.set_relative();
        assert_eq!(tracker.mode(), ExtrusionMode::Relative);

        // Now E values are deltas
        tracker.process_e(0.5);
        assert!((tracker.total_extrusion() - 1.5).abs() < 1e-6);

        // Switch back to absolute mode
        tracker.set_absolute();
        assert_eq!(tracker.mode(), ExtrusionMode::Absolute);

        // E position should have been tracked
        assert!((tracker.e_position() - 1.5).abs() < 1e-6);
    }

    #[test]
    fn test_extrusion_tracker_negative_relative() {
        let mut tracker = ExtrusionTracker::new();
        tracker.set_relative();

        // Extrude
        tracker.process_e(1.0);
        assert!((tracker.total_extrusion() - 1.0).abs() < 1e-6);

        // Retract (negative value in relative mode)
        let delta = tracker.process_e(-0.5);
        assert!((delta - (-0.5)).abs() < 1e-6);
        // Total should not decrease
        assert!((tracker.total_extrusion() - 1.0).abs() < 1e-6);

        // Extrude again
        tracker.process_e(0.3);
        assert!((tracker.total_extrusion() - 1.3).abs() < 1e-6);
    }

    #[test]
    fn test_parsed_gcode_with_e_reset() {
        let gcode = r#"
G1 Z0.2 F3000
G1 X10 Y10 E1.0 F1200
G1 X20 Y20 E2.0
G92 E0
G1 X30 Y30 E1.5
"#;
        let parsed = ParsedGCode::from_string(gcode);
        // Total should be 2.0 + 1.5 = 3.5
        assert!((parsed.total_extrusion - 3.5).abs() < 0.01);
    }

    #[test]
    fn test_parsed_gcode_with_relative_mode() {
        let gcode = r#"
M83 ; relative extrusion
G1 Z0.2 F3000
G1 X10 Y10 E0.5 F1200
G1 X20 Y20 E0.5
G1 X30 Y30 E0.5
"#;
        let parsed = ParsedGCode::from_string(gcode);
        // Total should be 0.5 + 0.5 + 0.5 = 1.5
        assert!((parsed.total_extrusion - 1.5).abs() < 0.01);
    }

    #[test]
    fn test_parsed_gcode_mode_switch() {
        let gcode = r#"
M82 ; absolute extrusion
G1 X10 Y10 E1.0 F1200
G1 X20 Y20 E2.0
M83 ; switch to relative
G1 X30 Y30 E0.5
G1 X40 Y40 E0.5
"#;
        let parsed = ParsedGCode::from_string(gcode);
        // Absolute: 1.0 + 1.0 = 2.0, then relative: 0.5 + 0.5 = 1.0
        // Total = 3.0
        assert!((parsed.total_extrusion - 3.0).abs() < 0.01);
    }

    #[test]
    fn test_extrusion_mode_default() {
        assert_eq!(ExtrusionMode::default(), ExtrusionMode::Absolute);
    }
}

/// Result of an exact line-by-line comparison.
#[derive(Debug, Clone)]
pub struct ExactComparisonResult {
    /// Path to the reference file.
    pub reference_path: String,
    /// Path to the generated file.
    pub generated_path: String,
    /// Total number of lines in reference.
    pub reference_line_count: usize,
    /// Total number of lines in generated.
    pub generated_line_count: usize,
    /// First line number where files diverge (1-based), or None if identical.
    pub first_divergence: Option<usize>,
    /// Number of exactly matching lines from the start.
    pub matching_lines: usize,
    /// Lines before divergence (for context).
    pub context_before: Vec<(usize, String)>,
    /// The diverging line from reference.
    pub divergence_reference: Option<(usize, String)>,
    /// The diverging line from generated.
    pub divergence_generated: Option<(usize, String)>,
    /// Lines after divergence (for context).
    pub context_after_reference: Vec<(usize, String)>,
    /// Lines after divergence (for context).
    pub context_after_generated: Vec<(usize, String)>,
}

impl ExactComparisonResult {
    // Check if the files are exactly identical.
    pub fn is_identical(&self) -> bool {
        self.first_divergence.is_none() && self.reference_line_count == self.generated_line_count
    }

    /// Get a formatted summary with color coding.
    pub fn format_colored_summary(&self, context_lines: usize) -> String {
        let mut output = String::new();

        output.push_str("=== EXACT LINE-BY-LINE COMPARISON ===\n\n");
        output.push_str(&format!(
            "Reference: {} ({} lines)\n",
            self.reference_path, self.reference_line_count
        ));
        output.push_str(&format!(
            "Generated: {} ({} lines)\n\n",
            self.generated_path, self.generated_line_count
        ));

        if self.is_identical() {
            output.push_str("✅ FILES ARE EXACTLY IDENTICAL\n");
            return output;
        }

        output.push_str(&format!(
            "✅ First {} lines match exactly\n",
            self.matching_lines
        ));

        if let Some(line_num) = self.first_divergence {
            output.push_str(&format!("❌ First divergence at line {}\n\n", line_num));

            // Show context before (in green)
            if !self.context_before.is_empty() {
                output.push_str("--- Context (last matching lines) ---\n");
                for (num, line) in &self.context_before {
                    output.push_str(&format!("  {:5} ✅ {}\n", num, line));
                }
                output.push('\n');
            }

            // Show the diverging line (in yellow/red)
            output.push_str("--- DIVERGENCE POINT ---\n");
            if let Some((num, line)) = &self.divergence_reference {
                output.push_str(&format!("  {:5} 📖 REF: {}\n", num, line));
            }
            if let Some((num, line)) = &self.divergence_generated {
                output.push_str(&format!("  {:5} 🔧 GEN: {}\n", num, line));
            }
            output.push('\n');

            // Show context after
            if !self.context_after_reference.is_empty() || !self.context_after_generated.is_empty()
            {
                output.push_str("--- Context (next lines) ---\n");

                let max_after = std::cmp::max(
                    self.context_after_reference.len(),
                    self.context_after_generated.len(),
                );

                for i in 0..max_after {
                    if let Some((num, line)) = self.context_after_reference.get(i) {
                        output.push_str(&format!("  {:5} 📖 {}\n", num, line));
                    } else {
                        output.push_str("        📖 (end of file)\n");
                    }

                    if let Some((num, line)) = self.context_after_generated.get(i) {
                        output.push_str(&format!("  {:5} 🔧 {}\n", num, line));
                    } else {
                        output.push_str("        🔧 (end of file)\n");
                    }
                    output.push('\n');
                }
            }
        }

        output
    }
}

/// Check if a line should be skipped in exact comparison.
/// Returns true for lines that are expected to differ (like filament stats).
fn skip_line_in_comparison(line: &str) -> bool {
    let line = line.trim();

    // The accel-aware print-time header line
    //   `; estimated printing time (normal mode) = 43m 0s`
    // is now produced by the faithful GCodeProcessor and is expected to match
    // the native value byte-for-byte, so it is NOT skipped.

    // Skip filament stats that will differ
    if line.starts_with("; total filament length") {
        return true;
    }
    if line.starts_with("; total filament volume") {
        return true;
    }
    if line.starts_with("; total filament weight") {
        return true;
    }

    false
}

/// Perform an exact line-by-line comparison of two G-code files.
///
/// This is stricter than semantic comparison - it looks for byte-for-byte matches.
/// Useful for verifying header generation and exact output parity.
///
/// Skips comparison of filament stats lines that are expected to differ.
///
/// # Arguments
///
/// * `reference` - Path to the reference G-code file
/// * `generated` - Path to the generated G-code file
/// * `context_lines` - Number of lines to show before/after divergence point
///
/// # Returns
///
/// An `ExactComparisonResult` containing the comparison details.
pub fn compare_exact_lines<P: AsRef<Path>>(
    reference: P,
    generated: P,
    context_lines: usize,
) -> io::Result<ExactComparisonResult> {
    let reference_path = reference.as_ref().display().to_string();
    let generated_path = generated.as_ref().display().to_string();

    let reference_content = fs::read_to_string(&reference)?;
    let generated_content = fs::read_to_string(&generated)?;

    let reference_lines: Vec<&str> = reference_content.lines().collect();
    let generated_lines: Vec<&str> = generated_content.lines().collect();

    let reference_line_count = reference_lines.len();
    let generated_line_count = generated_lines.len();

    // Find first divergence
    let mut first_divergence = None;
    let mut matching_lines = 0;

    for i in 0..std::cmp::min(reference_line_count, generated_line_count) {
        // Skip lines that are expected to differ
        if skip_line_in_comparison(reference_lines[i])
            || skip_line_in_comparison(generated_lines[i])
        {
            matching_lines += 1;
            continue;
        }

        if reference_lines[i] != generated_lines[i] {
            first_divergence = Some(i + 1); // 1-based line number
            break;
        }
        matching_lines += 1;
    }

    // If all common lines match but lengths differ
    if first_divergence.is_none() && reference_line_count != generated_line_count {
        first_divergence = Some(std::cmp::min(reference_line_count, generated_line_count) + 1);
    }

    let mut context_before = Vec::new();
    let mut divergence_reference = None;
    let mut divergence_generated = None;
    let mut context_after_reference = Vec::new();
    let mut context_after_generated = Vec::new();

    if let Some(div_line) = first_divergence {
        let div_idx = div_line - 1; // Convert to 0-based

        // Collect context before
        let start = if div_idx >= context_lines {
            div_idx - context_lines
        } else {
            0
        };
        for i in start..div_idx {
            if i < reference_line_count {
                context_before.push((i + 1, reference_lines[i].to_string()));
            }
        }

        // Collect the diverging lines
        if div_idx < reference_line_count {
            divergence_reference = Some((div_idx + 1, reference_lines[div_idx].to_string()));
        }
        if div_idx < generated_line_count {
            divergence_generated = Some((div_idx + 1, generated_lines[div_idx].to_string()));
        }

        // Collect context after
        let end = std::cmp::min(div_idx + context_lines + 1, reference_line_count);
        for i in (div_idx + 1)..end {
            context_after_reference.push((i + 1, reference_lines[i].to_string()));
        }

        let end = std::cmp::min(div_idx + context_lines + 1, generated_line_count);
        for i in (div_idx + 1)..end {
            context_after_generated.push((i + 1, generated_lines[i].to_string()));
        }
    }

    Ok(ExactComparisonResult {
        reference_path,
        generated_path,
        reference_line_count,
        generated_line_count,
        first_divergence,
        matching_lines,
        context_before,
        divergence_reference,
        divergence_generated,
        context_after_reference,
        context_after_generated,
    })
}
