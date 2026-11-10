//! G-code validation and report generation module.
//!
//! This module provides comprehensive validation tools for comparing G-code generated
//! by the Rust slicer against reference G-code from BambuStudio. It builds upon the
//! base comparison infrastructure to provide detailed quality reports.
//!
//! # Overview
//!
//! The validation system analyzes G-code differences across multiple dimensions:
//! - Layer structure and Z heights
//! - Extrusion amounts and flow rates
//! - Travel distances and movements
//! - Feature classification (perimeters, infill, support, etc.)
//! - Timing estimates
//!
//! # Quality Score
//!
//! A composite quality score (0-100) is computed based on weighted factors:
//! - Layer count accuracy (20%)
//! - Total extrusion accuracy (30%)
//! - Per-layer extrusion consistency (25%)
//! - Toolpath coverage (15%)
//! - Feature presence (10%)
//!
//! # Example
//!
//! ```rust,ignore
//! use slicer::gcode::validation::{ValidationReport, ValidationConfig};
//! use slicer::gcode::compare::{ParsedGCode, GCodeComparator};
//!
//! let reference = ParsedGCode::from_file("reference.gcode")?;
//! let generated = ParsedGCode::from_file("generated.gcode")?;
//!
//! let report = ValidationReport::generate(&reference, &generated, ValidationConfig::default());
//! println!("{}", report.to_text());
//! println!("Quality Score: {:.1}/100", report.quality_score());
//! ```

use super::compare::ParsedGCode;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::fmt;
use std::io::{self, Write};
use std::path::Path;

/// Feature types that can be detected in G-code.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum FeatureType {
    /// External perimeter (outermost wall)
    ExternalPerimeter,
    /// Internal perimeter (inner walls)
    InternalPerimeter,
    /// Solid infill (top/bottom surfaces)
    SolidInfill,
    /// Sparse infill (interior fill)
    SparseInfill,
    /// Bridge infill
    BridgeInfill,
    /// Support material
    Support,
    /// Support interface
    SupportInterface,
    /// Skirt
    Skirt,
    /// Brim
    Brim,
    /// Raft
    Raft,
    /// Travel move (non-extruding)
    Travel,
    /// Wipe move
    Wipe,
    /// Prime/purge
    Prime,
    /// Unknown/unclassified
    Unknown,
}

impl fmt::Display for FeatureType {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let s = match self {
            FeatureType::ExternalPerimeter => "External Perimeter",
            FeatureType::InternalPerimeter => "Internal Perimeter",
            FeatureType::SolidInfill => "Solid Infill",
            FeatureType::SparseInfill => "Sparse Infill",
            FeatureType::BridgeInfill => "Bridge Infill",
            FeatureType::Support => "Support",
            FeatureType::SupportInterface => "Support Interface",
            FeatureType::Skirt => "Skirt",
            FeatureType::Brim => "Brim",
            FeatureType::Raft => "Raft",
            FeatureType::Travel => "Travel",
            FeatureType::Wipe => "Wipe",
            FeatureType::Prime => "Prime",
            FeatureType::Unknown => "Unknown",
        };
        write!(f, "{}", s)
    }
}

impl FeatureType {
    // Parse feature type from G-code comment.
    //
    // Supports multiple slicer formats:
    // - BambuStudio: `; FEATURE: Outer wall`, `; FEATURE: Inner wall`, `; FEATURE: Sparse infill`
    // - PrusaSlicer/Cura: `; TYPE:External perimeter`, `; TYPE:Perimeter`
    // - Generic: keywords like "external perimeter", "outer wall", etc.
    pub fn from_comment(comment: &str) -> Option<Self> {
        let lower = comment.to_lowercase();

        // First check for BambuStudio-style FEATURE comments (most specific)
        // Format: "; FEATURE: <type>"
        if lower.contains("feature:") {
            // Extract the feature type after "feature:"
            if let Some(idx) = lower.find("feature:") {
                let feature_part = lower[idx + 8..].trim();
                return Self::parse_feature_name(feature_part);
            }
        }

        // Check for PrusaSlicer-style TYPE comments
        // Format: ";TYPE:<type>"
        if lower.contains("type:") {
            if let Some(idx) = lower.find("type:") {
                let type_part = lower[idx + 5..].trim();
                return Self::parse_feature_name(type_part);
            }
        }

        // Check for WIPE_START/WIPE_END markers (BambuStudio)
        if lower.contains("wipe_start") || lower.contains("wipe_end") {
            return Some(FeatureType::Wipe);
        }

        // Fallback: look for keywords in the comment
        Self::parse_feature_name(&lower)
    }

    /// Parse a feature name string into a FeatureType.
    fn parse_feature_name(name: &str) -> Option<Self> {
        let name = name.trim();

        // External perimeter / outer wall (check before generic "perimeter"/"wall")
        if name.contains("external perimeter")
            || name.contains("outer wall")
            || name == "outer"
            || name.starts_with("outer ")
        {
            return Some(FeatureType::ExternalPerimeter);
        }

        // Internal perimeter / inner wall
        if name.contains("perimeter")
            || name.contains("inner wall")
            || name == "inner"
            || name.starts_with("inner ")
            || name.contains("wall")
        {
            return Some(FeatureType::InternalPerimeter);
        }

        // Solid infill (top/bottom surfaces) - check before generic "infill"
        if name.contains("solid infill")
            || name.contains("top solid")
            || name.contains("bottom solid")
            || name.contains("top surface")
            || name.contains("bottom surface")
            || name == "top"
            || name == "bottom"
        {
            return Some(FeatureType::SolidInfill);
        }

        // Bridge infill - check before generic "infill"
        if name.contains("bridge") {
            return Some(FeatureType::BridgeInfill);
        }

        // Sparse infill (internal fill)
        if name.contains("sparse infill")
            || name.contains("internal infill")
            || name.contains("infill")
            || name == "fill"
        {
            return Some(FeatureType::SparseInfill);
        }

        // Support interface - check before generic "support"
        if name.contains("support interface") || name.contains("support-interface") {
            return Some(FeatureType::SupportInterface);
        }

        // Support material
        if name.contains("support") {
            return Some(FeatureType::Support);
        }

        // Skirt
        if name.contains("skirt") {
            return Some(FeatureType::Skirt);
        }

        // Brim
        if name.contains("brim") {
            return Some(FeatureType::Brim);
        }

        // Raft
        if name.contains("raft") {
            return Some(FeatureType::Raft);
        }

        // Wipe move
        if name.contains("wipe") {
            return Some(FeatureType::Wipe);
        }

        // Prime/purge
        if name.contains("prime") || name.contains("purge") {
            return Some(FeatureType::Prime);
        }

        // Travel
        if name.contains("travel") || name.contains("move") {
            return Some(FeatureType::Travel);
        }

        // Gap fill (treat as solid infill)
        if name.contains("gap fill") || name.contains("gap-fill") {
            return Some(FeatureType::SolidInfill);
        }

        // Overhang (treat as external perimeter)
        if name.contains("overhang") {
            return Some(FeatureType::ExternalPerimeter);
        }

        None
    }

    /// Check if this is a structural feature (not travel/wipe/prime).
    pub fn is_structural(&self) -> bool {
        !matches!(
            self,
            FeatureType::Travel | FeatureType::Wipe | FeatureType::Prime | FeatureType::Unknown
        )
    }
}

/// Statistics for a specific feature type.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct FeatureStats {
    /// Total extrusion amount (mm of filament)
    pub extrusion_mm: f64,
    /// Total path length (mm)
    pub path_length_mm: f64,
    /// Number of moves
    pub move_count: usize,
    /// Number of layers where this feature appears
    pub layer_count: usize,
}

impl FeatureStats {
    // Check if this feature has any content.
    pub fn is_empty(&self) -> bool {
        self.move_count == 0
    }
}

/// Per-layer validation statistics.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LayerValidation {
    /// Layer number (0-indexed)
    pub layer_num: usize,
    /// Z height in mm
    pub z_height: f64,
    /// Reference extrusion for this layer
    pub ref_extrusion: f64,
    /// Generated extrusion for this layer
    pub gen_extrusion: f64,
    /// Extrusion difference as percentage
    pub extrusion_diff_percent: f64,
    /// Reference move count
    pub ref_moves: usize,
    /// Generated move count
    pub gen_moves: usize,
    /// Layer match score (0-100)
    pub match_score: f64,
    /// Issues found in this layer
    pub issues: Vec<String>,
}

impl LayerValidation {
    // Check if this layer passed validation.
    pub fn passed(&self, threshold: f64) -> bool {
        self.match_score >= threshold
    }
}

/// Configuration for validation.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ValidationConfig {
    /// Tolerance for layer count difference (as fraction, e.g., 0.05 = 5%)
    pub layer_count_tolerance: f64,
    /// Tolerance for total extrusion difference (as fraction)
    pub total_extrusion_tolerance: f64,
    /// Tolerance for per-layer extrusion difference (as fraction)
    pub layer_extrusion_tolerance: f64,
    /// Tolerance for Z height difference (mm)
    pub z_tolerance: f64,
    /// Weight for layer count in quality score
    pub weight_layer_count: f64,
    /// Weight for total extrusion in quality score
    pub weight_total_extrusion: f64,
    /// Weight for per-layer consistency in quality score
    pub weight_layer_consistency: f64,
    /// Weight for toolpath coverage in quality score
    pub weight_coverage: f64,
    /// Weight for feature presence in quality score
    pub weight_features: f64,
    /// Passing threshold for quality score (0-100)
    pub pass_threshold: f64,
    /// Whether to include detailed per-move analysis
    pub detailed_moves: bool,
    /// Whether to classify features from comments
    pub classify_features: bool,
}

impl Default for ValidationConfig {
    fn default() -> Self {
        Self {
            layer_count_tolerance: 0.05,     // 5% tolerance
            total_extrusion_tolerance: 0.10, // 10% tolerance
            layer_extrusion_tolerance: 0.20, // 20% per-layer tolerance
            z_tolerance: 0.01,               // 0.01mm Z tolerance
            weight_layer_count: 0.20,
            weight_total_extrusion: 0.30,
            weight_layer_consistency: 0.25,
            weight_coverage: 0.15,
            weight_features: 0.10,
            pass_threshold: 100.0,
            detailed_moves: false,
            classify_features: true,
        }
    }
}

impl ValidationConfig {
    // Create a strict validation configuration.
    pub fn strict() -> Self {
        Self {
            layer_count_tolerance: 0.01,
            total_extrusion_tolerance: 0.05,
            layer_extrusion_tolerance: 0.10,
            z_tolerance: 0.001,
            pass_threshold: 100.0,
            detailed_moves: true,
            ..Default::default()
        }
    }

    /// Create a relaxed validation configuration.
    pub fn relaxed() -> Self {
        Self {
            layer_count_tolerance: 0.10,
            total_extrusion_tolerance: 0.20,
            layer_extrusion_tolerance: 0.30,
            z_tolerance: 0.05,
            pass_threshold: 50.0,
            detailed_moves: false,
            ..Default::default()
        }
    }
}

/// Summary statistics for the validation.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ValidationSummary {
    /// Reference file path
    pub reference_path: Option<String>,
    /// Generated file path
    pub generated_path: Option<String>,
    /// Reference layer count
    pub ref_layer_count: usize,
    /// Generated layer count
    pub gen_layer_count: usize,
    /// Layer count difference
    pub layer_count_diff: i32,
    /// Layer count difference as percentage
    pub layer_count_diff_percent: f64,
    /// Reference total extrusion (mm)
    pub ref_total_extrusion: f64,
    /// Generated total extrusion (mm)
    pub gen_total_extrusion: f64,
    /// Total extrusion difference (mm)
    pub extrusion_diff: f64,
    /// Total extrusion difference as percentage
    pub extrusion_diff_percent: f64,
    /// Reference total moves
    pub ref_total_moves: usize,
    /// Generated total moves
    pub gen_total_moves: usize,
    /// Move count difference
    pub move_count_diff: i32,
    /// Reference total travel distance (mm)
    pub ref_travel_distance: f64,
    /// Generated total travel distance (mm)
    pub gen_travel_distance: f64,
    /// Travel distance difference as percentage
    pub travel_diff_percent: f64,
    /// Reference arc move count (G2/G3)
    pub ref_arc_count: usize,
    /// Generated arc move count (G2/G3)
    pub gen_arc_count: usize,
    /// Reference retraction count
    pub ref_retraction_count: usize,
    /// Generated retraction count
    pub gen_retraction_count: usize,
    /// Reference extrusion moves count (G1 with positive E)
    pub ref_extrusion_moves: usize,
    /// Generated extrusion moves count
    pub gen_extrusion_moves: usize,
    /// Minimum Z height
    pub min_z: f64,
    /// Maximum Z height
    pub max_z: f64,
}

/// Individual issue found during validation.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ValidationIssue {
    /// Severity level
    pub severity: IssueSeverity,
    /// Issue category
    pub category: IssueCategory,
    /// Human-readable message
    pub message: String,
    /// Layer number (if applicable)
    pub layer: Option<usize>,
    /// Numeric value associated with issue (e.g., difference percentage)
    pub value: Option<f64>,
}

/// Severity level for validation issues.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum IssueSeverity {
    /// Informational only
    Info,
    /// Minor issue, might affect print quality slightly
    Warning,
    /// Significant issue, likely affects print quality
    Error,
    /// Critical issue, print will likely fail
    Critical,
}

impl fmt::Display for IssueSeverity {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let s = match self {
            IssueSeverity::Info => "INFO",
            IssueSeverity::Warning => "WARNING",
            IssueSeverity::Error => "ERROR",
            IssueSeverity::Critical => "CRITICAL",
        };
        write!(f, "{}", s)
    }
}

/// Category of validation issue.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum IssueCategory {
    /// Layer structure issues
    LayerStructure,
    /// Extrusion amount issues
    Extrusion,
    /// Travel/positioning issues
    Travel,
    /// Feature missing or different
    Feature,
    /// Timing/speed issues
    Timing,
    /// Configuration mismatch
    Configuration,
    /// Other issues
    Other,
}

impl fmt::Display for IssueCategory {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let s = match self {
            IssueCategory::LayerStructure => "Layer Structure",
            IssueCategory::Extrusion => "Extrusion",
            IssueCategory::Travel => "Travel",
            IssueCategory::Feature => "Feature",
            IssueCategory::Timing => "Timing",
            IssueCategory::Configuration => "Configuration",
            IssueCategory::Other => "Other",
        };
        write!(f, "{}", s)
    }
}

/// Score breakdown for the quality score.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ScoreBreakdown {
    /// Layer count score (0-100)
    pub layer_count_score: f64,
    /// Total extrusion score (0-100)
    pub total_extrusion_score: f64,
    /// Per-layer consistency score (0-100)
    pub layer_consistency_score: f64,
    /// Toolpath coverage score (0-100)
    pub coverage_score: f64,
    /// Feature presence score (0-100)
    pub feature_score: f64,
    /// Weights used for each component
    pub weights: ValidationConfig,
}

impl ScoreBreakdown {
    // Calculate the weighted total score.
    pub fn total(&self) -> f64 {
        self.layer_count_score * self.weights.weight_layer_count
            + self.total_extrusion_score * self.weights.weight_total_extrusion
            + self.layer_consistency_score * self.weights.weight_layer_consistency
            + self.coverage_score * self.weights.weight_coverage
            + self.feature_score * self.weights.weight_features
    }
}

/// Complete validation report.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ValidationReport {
    /// Configuration used for validation
    pub config: ValidationConfig,
    /// Summary statistics
    pub summary: ValidationSummary,
    /// Per-layer validation results
    pub layers: Vec<LayerValidation>,
    /// Feature breakdown for reference
    pub ref_features: HashMap<FeatureType, FeatureStats>,
    /// Feature breakdown for generated
    pub gen_features: HashMap<FeatureType, FeatureStats>,
    /// All issues found
    pub issues: Vec<ValidationIssue>,
    /// Score breakdown
    pub score_breakdown: ScoreBreakdown,
    /// Overall quality score (0-100)
    pub quality_score: f64,
    /// Whether validation passed
    pub passed: bool,
}

impl ValidationReport {
    // Generate a validation report comparing two G-code files.
    pub fn generate(
        reference: &ParsedGCode,
        generated: &ParsedGCode,
        config: ValidationConfig,
    ) -> Self {
        let mut issues = Vec::new();

        // Calculate summary statistics
        let summary = Self::compute_summary(reference, generated);

        // Check for layer count issues
        if summary.layer_count_diff_percent.abs() > config.layer_count_tolerance * 100.0 {
            let severity = if summary.layer_count_diff_percent.abs() > 20.0 {
                IssueSeverity::Critical
            } else if summary.layer_count_diff_percent.abs() > 10.0 {
                IssueSeverity::Error
            } else {
                IssueSeverity::Warning
            };

            issues.push(ValidationIssue {
                severity,
                category: IssueCategory::LayerStructure,
                message: format!(
                    "Layer count differs: {} vs {} ({:+.1}%)",
                    summary.ref_layer_count,
                    summary.gen_layer_count,
                    summary.layer_count_diff_percent
                ),
                layer: None,
                value: Some(summary.layer_count_diff_percent),
            });
        }

        // Check for total extrusion issues
        if summary.extrusion_diff_percent.abs() > config.total_extrusion_tolerance * 100.0 {
            let severity = if summary.extrusion_diff_percent.abs() > 25.0 {
                IssueSeverity::Critical
            } else if summary.extrusion_diff_percent.abs() > 15.0 {
                IssueSeverity::Error
            } else {
                IssueSeverity::Warning
            };

            issues.push(ValidationIssue {
                severity,
                category: IssueCategory::Extrusion,
                message: format!(
                    "Total extrusion differs: {:.2}mm vs {:.2}mm ({:+.1}%)",
                    summary.ref_total_extrusion,
                    summary.gen_total_extrusion,
                    summary.extrusion_diff_percent
                ),
                layer: None,
                value: Some(summary.extrusion_diff_percent),
            });
        }

        // Per-layer analysis
        let layers = Self::analyze_layers(reference, generated, &config, &mut issues);

        // Feature analysis
        let ref_features = if config.classify_features {
            Self::extract_features(reference)
        } else {
            HashMap::new()
        };

        let gen_features = if config.classify_features {
            Self::extract_features(generated)
        } else {
            HashMap::new()
        };

        // Check for missing features
        if config.classify_features {
            for (feature, ref_stats) in &ref_features {
                if !ref_stats.is_empty() {
                    if let Some(gen_stats) = gen_features.get(feature) {
                        if gen_stats.is_empty() {
                            issues.push(ValidationIssue {
                                severity: IssueSeverity::Warning,
                                category: IssueCategory::Feature,
                                message: format!(
                                    "Feature '{}' present in reference but missing in generated",
                                    feature
                                ),
                                layer: None,
                                value: None,
                            });
                        }
                    } else {
                        issues.push(ValidationIssue {
                            severity: IssueSeverity::Warning,
                            category: IssueCategory::Feature,
                            message: format!(
                                "Feature '{}' present in reference but missing in generated",
                                feature
                            ),
                            layer: None,
                            value: None,
                        });
                    }
                }
            }
        }

        // Calculate scores
        let score_breakdown =
            Self::calculate_scores(&summary, &layers, &ref_features, &gen_features, &config);
        let quality_score = score_breakdown.total();
        let passed = quality_score >= config.pass_threshold;

        // Sort issues by severity
        issues.sort_by(|a, b| {
            let severity_order = |s: &IssueSeverity| match s {
                IssueSeverity::Critical => 0,
                IssueSeverity::Error => 1,
                IssueSeverity::Warning => 2,
                IssueSeverity::Info => 3,
            };
            severity_order(&a.severity).cmp(&severity_order(&b.severity))
        });

        Self {
            config,
            summary,
            layers,
            ref_features,
            gen_features,
            issues,
            score_breakdown,
            quality_score,
            passed,
        }
    }

    /// Generate report from file paths.
    pub fn from_files<P: AsRef<Path>>(
        reference_path: P,
        generated_path: P,
        config: ValidationConfig,
    ) -> io::Result<Self> {
        let reference = ParsedGCode::from_file(reference_path)?;
        let generated = ParsedGCode::from_file(generated_path)?;
        Ok(Self::generate(&reference, &generated, config))
    }

    fn compute_summary(reference: &ParsedGCode, generated: &ParsedGCode) -> ValidationSummary {
        let ref_layer_count = reference.layer_count();
        let gen_layer_count = generated.layer_count();
        let layer_count_diff = gen_layer_count as i32 - ref_layer_count as i32;
        let layer_count_diff_percent = if ref_layer_count > 0 {
            (layer_count_diff as f64 / ref_layer_count as f64) * 100.0
        } else {
            0.0
        };

        let ref_total_extrusion = reference.total_extrusion;
        let gen_total_extrusion = generated.total_extrusion;
        let extrusion_diff = gen_total_extrusion - ref_total_extrusion;
        let extrusion_diff_percent = if ref_total_extrusion > 0.0 {
            (extrusion_diff / ref_total_extrusion) * 100.0
        } else {
            0.0
        };

        let ref_total_moves = reference.total_moves;
        let gen_total_moves = generated.total_moves;
        let move_count_diff = gen_total_moves as i32 - ref_total_moves as i32;

        // Calculate travel distances
        let ref_travel_distance = Self::calculate_travel_distance(reference);
        let gen_travel_distance = Self::calculate_travel_distance(generated);
        let travel_diff_percent = if ref_travel_distance > 0.0 {
            ((gen_travel_distance - ref_travel_distance) / ref_travel_distance) * 100.0
        } else {
            0.0
        };

        // Calculate arc counts, retraction counts, and extrusion move counts
        let (ref_arc_count, ref_retraction_count, ref_extrusion_moves) =
            Self::calculate_move_stats(reference);
        let (gen_arc_count, gen_retraction_count, gen_extrusion_moves) =
            Self::calculate_move_stats(generated);

        // Z range
        let ref_z_heights = reference.z_heights();
        let gen_z_heights = generated.z_heights();
        let all_z: Vec<f64> = ref_z_heights
            .iter()
            .chain(gen_z_heights.iter())
            .copied()
            .collect();
        let min_z = all_z.iter().copied().fold(f64::INFINITY, f64::min);
        let max_z = all_z.iter().copied().fold(f64::NEG_INFINITY, f64::max);

        ValidationSummary {
            reference_path: reference.path.clone(),
            generated_path: generated.path.clone(),
            ref_layer_count,
            gen_layer_count,
            layer_count_diff,
            layer_count_diff_percent,
            ref_total_extrusion,
            gen_total_extrusion,
            extrusion_diff,
            extrusion_diff_percent,
            ref_total_moves,
            gen_total_moves,
            move_count_diff,
            ref_travel_distance,
            gen_travel_distance,
            travel_diff_percent,
            ref_arc_count,
            gen_arc_count,
            ref_retraction_count,
            gen_retraction_count,
            ref_extrusion_moves,
            gen_extrusion_moves,
            min_z: if min_z.is_finite() { min_z } else { 0.0 },
            max_z: if max_z.is_finite() { max_z } else { 0.0 },
        }
    }

    /// Calculate arc count, retraction count, and extrusion move count from G-code.
    fn calculate_move_stats(gcode: &ParsedGCode) -> (usize, usize, usize) {
        let mut arc_count = 0;
        let mut retraction_count = 0;
        let mut extrusion_moves = 0;

        for layer in &gcode.layers {
            for mov in &layer.moves {
                // Check for arc moves (G2/G3)
                if mov.command == "G2" || mov.command == "G3" {
                    arc_count += 1;
                }

                // Check for extrusion and retraction
                if let Some(e) = mov.e {
                    if e > 0.0 {
                        extrusion_moves += 1;
                    } else if e < 0.0 {
                        retraction_count += 1;
                    }
                }
            }
        }

        (arc_count, retraction_count, extrusion_moves)
    }

    fn calculate_travel_distance(gcode: &ParsedGCode) -> f64 {
        let mut total = 0.0;
        let mut last_x = 0.0f64;
        let mut last_y = 0.0f64;

        for layer in &gcode.layers {
            for mov in &layer.moves {
                if mov.is_travel() {
                    if let (Some(x), Some(y)) = (mov.x, mov.y) {
                        let dx = x - last_x;
                        let dy = y - last_y;
                        total += (dx * dx + dy * dy).sqrt();
                        last_x = x;
                        last_y = y;
                    }
                } else {
                    if let Some(x) = mov.x {
                        last_x = x;
                    }
                    if let Some(y) = mov.y {
                        last_y = y;
                    }
                }
            }
        }

        total
    }

    fn analyze_layers(
        reference: &ParsedGCode,
        generated: &ParsedGCode,
        config: &ValidationConfig,
        issues: &mut Vec<ValidationIssue>,
    ) -> Vec<LayerValidation> {
        let max_layers = reference.layer_count().max(generated.layer_count());
        let mut layers = Vec::with_capacity(max_layers);

        // First pass: detect systematic Z offset
        // This helps avoid spamming warnings when there's just a different first layer height
        let z_offsets: Vec<f64> = (0..max_layers
            .min(reference.layer_count())
            .min(generated.layer_count()))
            .filter_map(|i| {
                let ref_layer = reference.layer(i)?;
                let gen_layer = generated.layer(i)?;
                if ref_layer.z_height > 0.0 && gen_layer.z_height > 0.0 {
                    Some(ref_layer.z_height - gen_layer.z_height)
                } else {
                    None
                }
            })
            .collect();

        // Calculate median Z offset to detect systematic shift
        let systematic_z_offset = if z_offsets.len() >= 5 {
            let mut sorted = z_offsets.clone();
            sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
            let median = sorted[sorted.len() / 2];

            // Check if most layers have similar offset (within 0.05mm of median)
            let consistent_count = z_offsets
                .iter()
                .filter(|&&o| (o - median).abs() < 0.05)
                .count();

            if consistent_count as f64 / z_offsets.len() as f64 > 0.8 {
                // More than 80% of layers have consistent offset
                Some(median)
            } else {
                None
            }
        } else {
            None
        };

        // Report systematic Z offset once instead of per-layer
        if let Some(offset) = systematic_z_offset {
            if offset.abs() > config.z_tolerance {
                issues.push(ValidationIssue {
                    severity: IssueSeverity::Info,
                    category: IssueCategory::LayerStructure,
                    message: format!(
                        "Systematic Z height offset detected: {:.3}mm (likely different first layer height)",
                        offset
                    ),
                    layer: None,
                    value: Some(offset),
                });
            }
        }

        for i in 0..max_layers {
            let ref_layer = reference.layer(i);
            let gen_layer = generated.layer(i);

            let (ref_extrusion, ref_moves, ref_z) = ref_layer
                .map(|l| (l.total_extrusion, l.moves.len(), l.z_height))
                .unwrap_or((0.0, 0, 0.0));

            let (gen_extrusion, gen_moves, gen_z) = gen_layer
                .map(|l| (l.total_extrusion, l.moves.len(), l.z_height))
                .unwrap_or((0.0, 0, 0.0));

            let z_height = if ref_z > 0.0 { ref_z } else { gen_z };

            let extrusion_diff_percent = if ref_extrusion > 0.0 {
                ((gen_extrusion - ref_extrusion) / ref_extrusion) * 100.0
            } else if gen_extrusion > 0.0 {
                100.0
            } else {
                0.0
            };

            // Calculate match score based on extrusion similarity
            let match_score = if ref_extrusion > 0.0 {
                let diff_ratio = (extrusion_diff_percent.abs() / 100.0).min(1.0);
                (1.0 - diff_ratio) * 100.0
            } else if gen_extrusion == 0.0 {
                100.0
            } else {
                0.0
            };

            let mut layer_issues = Vec::new();

            // Check for significant layer differences
            if extrusion_diff_percent.abs() > config.layer_extrusion_tolerance * 100.0 {
                let msg = format!(
                    "Layer {} extrusion differs by {:.1}%",
                    i, extrusion_diff_percent
                );
                layer_issues.push(msg.clone());

                if extrusion_diff_percent.abs() > 50.0 {
                    issues.push(ValidationIssue {
                        severity: IssueSeverity::Error,
                        category: IssueCategory::Extrusion,
                        message: msg,
                        layer: Some(i),
                        value: Some(extrusion_diff_percent),
                    });
                }
            }

            // Check for missing layers
            if ref_layer.is_some() && gen_layer.is_none() {
                let msg = format!("Layer {} missing in generated G-code", i);
                layer_issues.push(msg.clone());
                issues.push(ValidationIssue {
                    severity: IssueSeverity::Error,
                    category: IssueCategory::LayerStructure,
                    message: msg,
                    layer: Some(i),
                    value: None,
                });
            } else if ref_layer.is_none() && gen_layer.is_some() {
                let msg = format!("Extra layer {} in generated G-code", i);
                layer_issues.push(msg.clone());
                issues.push(ValidationIssue {
                    severity: IssueSeverity::Warning,
                    category: IssueCategory::LayerStructure,
                    message: msg,
                    layer: Some(i),
                    value: None,
                });
            }

            // Check Z height mismatch (only if not explained by systematic offset)
            if ref_layer.is_some() && gen_layer.is_some() {
                let z_diff = ref_z - gen_z;
                let z_diff_abs = z_diff.abs();

                // If there's a systematic offset, only report if this layer deviates from it
                let report_z_issue = if let Some(offset) = systematic_z_offset {
                    // Report if this layer's offset differs significantly from the systematic offset
                    (z_diff - offset).abs() > config.z_tolerance
                } else {
                    // No systematic offset, report any significant difference
                    z_diff_abs > config.z_tolerance
                };

                if report_z_issue {
                    let msg = format!(
                        "Layer {} Z height differs: {:.3}mm vs {:.3}mm",
                        i, ref_z, gen_z
                    );
                    layer_issues.push(msg.clone());
                    issues.push(ValidationIssue {
                        severity: IssueSeverity::Warning,
                        category: IssueCategory::LayerStructure,
                        message: msg,
                        layer: Some(i),
                        value: Some(z_diff_abs),
                    });
                }
            }

            layers.push(LayerValidation {
                layer_num: i,
                z_height,
                ref_extrusion,
                gen_extrusion,
                extrusion_diff_percent,
                ref_moves,
                gen_moves,
                match_score,
                issues: layer_issues,
            });
        }

        layers
    }

    fn extract_features(gcode: &ParsedGCode) -> HashMap<FeatureType, FeatureStats> {
        let mut features: HashMap<FeatureType, FeatureStats> = HashMap::new();
        let mut layers_with_feature: HashMap<FeatureType, std::collections::HashSet<usize>> =
            HashMap::new();

        for (layer_idx, layer) in gcode.layers.iter().enumerate() {
            // Reset current feature at the start of each layer
            // This ensures features don't incorrectly persist from previous layers
            let mut current_feature = FeatureType::Unknown;

            // Track in-wipe state (for BambuStudio WIPE_START/WIPE_END blocks)
            let mut in_wipe = false;

            // Scan the raw lines for this layer's range to detect feature comments
            let line_start = layer.line_start;
            let line_end = layer.line_end.min(gcode.lines.len());

            // Build a map of line number -> feature type for this layer
            let mut line_features: Vec<FeatureType> =
                vec![FeatureType::Unknown; line_end - line_start];
            let mut line_in_wipe: Vec<bool> = vec![false; line_end - line_start];

            for (rel_idx, line) in gcode.lines[line_start..line_end].iter().enumerate() {
                let trimmed = line.trim();

                // Check for feature type comments
                if trimmed.starts_with(';') {
                    // Check for WIPE_START/WIPE_END markers
                    let lower = trimmed.to_lowercase();
                    if lower.contains("wipe_start") {
                        in_wipe = true;
                    } else if lower.contains("wipe_end") {
                        in_wipe = false;
                    } else if let Some(feature) = FeatureType::from_comment(trimmed) {
                        // Don't override wipe state with other features
                        if feature != FeatureType::Wipe {
                            current_feature = feature;
                        }
                    }
                }

                line_features[rel_idx] = current_feature;
                line_in_wipe[rel_idx] = in_wipe;
            }

            // Accumulate stats for moves in this layer
            let mut last_x = 0.0f64;
            let mut last_y = 0.0f64;
            let mut move_idx = 0;

            // We need to match moves to their line numbers to get correct features
            // Iterate through lines and process moves when we encounter them
            for (rel_idx, line) in gcode.lines[line_start..line_end].iter().enumerate() {
                let trimmed = line.trim();

                // Skip comments and non-move lines
                if trimmed.starts_with(';') || trimmed.is_empty() {
                    continue;
                }

                // Check if this is a G0/G1/G2/G3 command
                let is_move = trimmed.starts_with("G0")
                    || trimmed.starts_with("G1")
                    || trimmed.starts_with("G2")
                    || trimmed.starts_with("G3");

                if !is_move {
                    continue;
                }

                // Get the corresponding move if we haven't exhausted them
                if move_idx >= layer.moves.len() {
                    break;
                }

                let mov = &layer.moves[move_idx];
                move_idx += 1;

                // Determine feature type
                let feature = if line_in_wipe[rel_idx] && mov.e.map_or(false, |e| e < 0.0) {
                    // Negative extrusion during wipe block
                    FeatureType::Wipe
                } else if mov.is_travel() {
                    FeatureType::Travel
                } else if line_in_wipe[rel_idx] {
                    FeatureType::Wipe
                } else {
                    line_features[rel_idx]
                };

                let stats = features.entry(feature).or_default();
                stats.move_count += 1;

                if let Some(e) = mov.e {
                    if e > 0.0 {
                        stats.extrusion_mm += e;
                    }
                }

                if let (Some(x), Some(y)) = (mov.x, mov.y) {
                    let dx = x - last_x;
                    let dy = y - last_y;
                    stats.path_length_mm += (dx * dx + dy * dy).sqrt();
                    last_x = x;
                    last_y = y;
                }

                layers_with_feature
                    .entry(feature)
                    .or_default()
                    .insert(layer_idx);
            }

            // Process any remaining moves that weren't matched to lines
            // (fallback for simpler G-code formats)
            while move_idx < layer.moves.len() {
                let mov = &layer.moves[move_idx];
                move_idx += 1;

                let feature = if mov.is_travel() {
                    FeatureType::Travel
                } else {
                    FeatureType::Unknown
                };

                let stats = features.entry(feature).or_default();
                stats.move_count += 1;

                if let Some(e) = mov.e {
                    if e > 0.0 {
                        stats.extrusion_mm += e;
                    }
                }

                if let (Some(x), Some(y)) = (mov.x, mov.y) {
                    let dx = x - last_x;
                    let dy = y - last_y;
                    stats.path_length_mm += (dx * dx + dy * dy).sqrt();
                    last_x = x;
                    last_y = y;
                }

                layers_with_feature
                    .entry(feature)
                    .or_default()
                    .insert(layer_idx);
            }
        }

        // Set layer counts
        for (feature, layers) in layers_with_feature {
            if let Some(stats) = features.get_mut(&feature) {
                stats.layer_count = layers.len();
            }
        }

        features
    }

    fn calculate_scores(
        summary: &ValidationSummary,
        layers: &[LayerValidation],
        ref_features: &HashMap<FeatureType, FeatureStats>,
        gen_features: &HashMap<FeatureType, FeatureStats>,
        config: &ValidationConfig,
    ) -> ScoreBreakdown {
        // Layer count score - very important for structural correctness
        // Use exponential decay: small differences are tolerated well, large ones penalized heavily
        let layer_count_score = {
            let diff_percent = summary.layer_count_diff_percent.abs();
            // Score of 100 at 0%, ~90 at 5%, ~50 at 20%, ~10 at 50%
            100.0 * (-diff_percent / 30.0).exp()
        };

        // Total extrusion score - use logarithmic scaling for better handling of large differences
        // This is more forgiving than linear scaling for larger differences
        let total_extrusion_score = {
            let diff_percent = summary.extrusion_diff_percent.abs();
            if diff_percent < 1.0 {
                // Within 1%: perfect score
                100.0
            } else if diff_percent <= 10.0 {
                // 1-10%: linear decay from 100 to 80
                100.0 - (diff_percent - 1.0) * 2.22
            } else if diff_percent <= 50.0 {
                // 10-50%: slower decay from 80 to 40
                80.0 - (diff_percent - 10.0) * 1.0
            } else if diff_percent <= 100.0 {
                // 50-100%: decay from 40 to 20
                40.0 - (diff_percent - 50.0) * 0.4
            } else {
                // >100%: logarithmic decay, never quite reaches 0
                // At 200% diff, score ~15; at 500% diff, score ~10
                20.0 / (1.0 + (diff_percent / 100.0).ln())
            }
        };

        // Per-layer consistency score (average of layer scores, with outlier tolerance)
        let layer_consistency_score = if !layers.is_empty() {
            // Use median-like approach: exclude worst 10% of layers to be more robust
            let mut scores: Vec<f64> = layers.iter().map(|l| l.match_score).collect();
            scores.sort_by(|a, b| b.partial_cmp(a).unwrap_or(std::cmp::Ordering::Equal));

            // Take top 90% of layers (exclude worst 10%)
            let take_count = (scores.len() as f64 * 0.9).ceil() as usize;
            let filtered_scores = &scores[..take_count.min(scores.len())];

            if !filtered_scores.is_empty() {
                filtered_scores.iter().sum::<f64>() / filtered_scores.len() as f64
            } else {
                100.0
            }
        } else {
            100.0
        };

        // Coverage score - use logarithmic scaling for move count ratio
        // This handles cases where one slicer produces more segments (e.g., no arc fitting)
        let coverage_score = {
            if summary.ref_total_moves > 0 {
                let ratio = summary.gen_total_moves as f64 / summary.ref_total_moves as f64;
                // For ratio: 1.0 = 100 score, 2.0 = ~75, 4.0 = ~50, 10.0 = ~25
                // Use: 100 / (1 + ln(ratio)) for ratio >= 1
                // For ratio < 1, use the inverse
                if ratio >= 1.0 {
                    (100.0 / (1.0 + ratio.ln().abs())).max(10.0)
                } else if ratio > 0.0 {
                    (100.0 / (1.0 + (1.0 / ratio).ln().abs())).max(10.0)
                } else {
                    0.0
                }
            } else {
                100.0
            }
        };

        // Feature score - presence and extrusion similarity of features
        let feature_score = if !ref_features.is_empty() {
            let mut presence_score = 0.0;
            let mut extrusion_score = 0.0;
            let mut total = 0;

            for (feature, ref_stats) in ref_features {
                // Skip non-structural features for scoring
                if !feature.is_structural() || ref_stats.is_empty() {
                    continue;
                }

                total += 1;

                if let Some(gen_stats) = gen_features.get(feature) {
                    if !gen_stats.is_empty() {
                        // Feature is present
                        presence_score += 1.0;

                        // Also compare extrusion amounts
                        if ref_stats.extrusion_mm > 0.0 {
                            let ext_ratio = gen_stats.extrusion_mm / ref_stats.extrusion_mm;
                            // Score based on how close the extrusion ratio is to 1.0
                            // Use similar logarithmic scaling
                            let ext_score = if ext_ratio >= 1.0 {
                                1.0 / (1.0 + ext_ratio.ln().abs() * 2.0)
                            } else if ext_ratio > 0.0 {
                                1.0 / (1.0 + (1.0 / ext_ratio).ln().abs() * 2.0)
                            } else {
                                0.0
                            };
                            extrusion_score += ext_score;
                        } else {
                            extrusion_score += 1.0; // No extrusion to compare
                        }
                    }
                }
            }

            if total > 0 {
                // 60% weight on presence, 40% on extrusion similarity
                let presence_pct = (presence_score / total as f64) * 100.0;
                let extrusion_pct = (extrusion_score / total as f64) * 100.0;
                presence_pct * 0.6 + extrusion_pct * 0.4
            } else {
                100.0
            }
        } else {
            100.0
        };

        ScoreBreakdown {
            layer_count_score,
            total_extrusion_score,
            layer_consistency_score,
            coverage_score,
            feature_score,
            weights: config.clone(),
        }
    }

    /// Get the quality score (0-100).
    pub fn quality_score(&self) -> f64 {
        self.quality_score
    }

    /// Check if validation passed.
    pub fn passed(&self) -> bool {
        self.passed
    }

    /// Get the pass/fail status as a string.
    pub fn status(&self) -> &'static str {
        if self.passed {
            "PASSED"
        } else {
            "FAILED"
        }
    }

    /// Get issues filtered by severity.
    pub fn issues_by_severity(&self, severity: IssueSeverity) -> Vec<&ValidationIssue> {
        self.issues
            .iter()
            .filter(|i| i.severity == severity)
            .collect()
    }

    /// Count issues by severity.
    pub fn issue_counts(&self) -> (usize, usize, usize, usize) {
        let critical = self
            .issues
            .iter()
            .filter(|i| i.severity == IssueSeverity::Critical)
            .count();
        let error = self
            .issues
            .iter()
            .filter(|i| i.severity == IssueSeverity::Error)
            .count();
        let warning = self
            .issues
            .iter()
            .filter(|i| i.severity == IssueSeverity::Warning)
            .count();
        let info = self
            .issues
            .iter()
            .filter(|i| i.severity == IssueSeverity::Info)
            .count();
        (critical, error, warning, info)
    }

    /// Generate a text report.
    pub fn to_text(&self) -> String {
        let mut s = String::new();

        // Header
        s.push_str("═══════════════════════════════════════════════════════════════════\n");
        s.push_str("                    G-CODE VALIDATION REPORT\n");
        s.push_str("═══════════════════════════════════════════════════════════════════\n\n");

        // Status
        let status_line = if self.passed {
            format!("Status: ✓ PASSED (Score: {:.1}/100)\n", self.quality_score)
        } else {
            format!(
                "Status: ✗ FAILED (Score: {:.1}/100, Threshold: {:.1})\n",
                self.quality_score, self.config.pass_threshold
            )
        };
        s.push_str(&status_line);
        s.push('\n');

        // Files
        s.push_str("Files:\n");
        s.push_str(&format!(
            "  Reference: {}\n",
            self.summary
                .reference_path
                .as_deref()
                .unwrap_or("(unknown)")
        ));
        s.push_str(&format!(
            "  Generated: {}\n",
            self.summary
                .generated_path
                .as_deref()
                .unwrap_or("(unknown)")
        ));
        s.push('\n');

        // Summary statistics
        s.push_str("───────────────────────────────────────────────────────────────────\n");
        s.push_str("SUMMARY\n");
        s.push_str("───────────────────────────────────────────────────────────────────\n");
        s.push_str(&format!(
            "  Layer Count:      {:>8} vs {:>8}  ({:+.1}%)\n",
            self.summary.ref_layer_count,
            self.summary.gen_layer_count,
            self.summary.layer_count_diff_percent
        ));
        s.push_str(&format!(
            "  Total Extrusion:  {:>8.2}mm vs {:>8.2}mm  ({:+.1}%)\n",
            self.summary.ref_total_extrusion,
            self.summary.gen_total_extrusion,
            self.summary.extrusion_diff_percent
        ));
        s.push_str(&format!(
            "  Total Moves:      {:>8} vs {:>8}  ({:+})\n",
            self.summary.ref_total_moves,
            self.summary.gen_total_moves,
            self.summary.move_count_diff
        ));
        s.push_str(&format!(
            "  Travel Distance:  {:>8.2}mm vs {:>8.2}mm  ({:+.1}%)\n",
            self.summary.ref_travel_distance,
            self.summary.gen_travel_distance,
            self.summary.travel_diff_percent
        ));
        s.push_str(&format!(
            "  Extrusion Moves:  {:>8} vs {:>8}\n",
            self.summary.ref_extrusion_moves, self.summary.gen_extrusion_moves
        ));
        s.push_str(&format!(
            "  Arc Moves (G2/G3):{:>8} vs {:>8}\n",
            self.summary.ref_arc_count, self.summary.gen_arc_count
        ));
        s.push_str(&format!(
            "  Retractions:      {:>8} vs {:>8}\n",
            self.summary.ref_retraction_count, self.summary.gen_retraction_count
        ));
        s.push_str(&format!(
            "  Z Range:          {:.3}mm - {:.3}mm\n",
            self.summary.min_z, self.summary.max_z
        ));
        s.push('\n');

        // Score breakdown
        s.push_str("───────────────────────────────────────────────────────────────────\n");
        s.push_str("SCORE BREAKDOWN\n");
        s.push_str("───────────────────────────────────────────────────────────────────\n");
        s.push_str(&format!(
            "  Layer Count:       {:>6.1} × {:.0}% = {:>6.2}\n",
            self.score_breakdown.layer_count_score,
            self.score_breakdown.weights.weight_layer_count * 100.0,
            self.score_breakdown.layer_count_score
                * self.score_breakdown.weights.weight_layer_count
        ));
        s.push_str(&format!(
            "  Total Extrusion:   {:>6.1} × {:.0}% = {:>6.2}\n",
            self.score_breakdown.total_extrusion_score,
            self.score_breakdown.weights.weight_total_extrusion * 100.0,
            self.score_breakdown.total_extrusion_score
                * self.score_breakdown.weights.weight_total_extrusion
        ));
        s.push_str(&format!(
            "  Layer Consistency: {:>6.1} × {:.0}% = {:>6.2}\n",
            self.score_breakdown.layer_consistency_score,
            self.score_breakdown.weights.weight_layer_consistency * 100.0,
            self.score_breakdown.layer_consistency_score
                * self.score_breakdown.weights.weight_layer_consistency
        ));
        s.push_str(&format!(
            "  Coverage:          {:>6.1} × {:.0}% = {:>6.2}\n",
            self.score_breakdown.coverage_score,
            self.score_breakdown.weights.weight_coverage * 100.0,
            self.score_breakdown.coverage_score * self.score_breakdown.weights.weight_coverage
        ));
        s.push_str(&format!(
            "  Features:          {:>6.1} × {:.0}% = {:>6.2}\n",
            self.score_breakdown.feature_score,
            self.score_breakdown.weights.weight_features * 100.0,
            self.score_breakdown.feature_score * self.score_breakdown.weights.weight_features
        ));
        s.push_str(&format!("  ─────────────────────────────────────\n"));
        s.push_str(&format!(
            "  TOTAL:                         {:>6.1}/100\n",
            self.quality_score
        ));
        s.push('\n');

        // Issues
        let (critical, error, warning, info) = self.issue_counts();
        s.push_str("───────────────────────────────────────────────────────────────────\n");
        s.push_str(&format!(
            "ISSUES ({} critical, {} errors, {} warnings, {} info)\n",
            critical, error, warning, info
        ));
        s.push_str("───────────────────────────────────────────────────────────────────\n");

        if self.issues.is_empty() {
            s.push_str("  No issues found.\n");
        } else {
            for issue in &self.issues {
                let icon = match issue.severity {
                    IssueSeverity::Critical => "✗✗",
                    IssueSeverity::Error => "✗",
                    IssueSeverity::Warning => "⚠",
                    IssueSeverity::Info => "ℹ",
                };
                let layer_str = issue
                    .layer
                    .map(|l| format!(" [Layer {}]", l))
                    .unwrap_or_default();
                s.push_str(&format!(
                    "  {} [{}]{} {}\n",
                    icon, issue.severity, layer_str, issue.message
                ));
            }
        }
        s.push('\n');

        // Feature comparison (if available)
        if !self.ref_features.is_empty() || !self.gen_features.is_empty() {
            s.push_str("───────────────────────────────────────────────────────────────────\n");
            s.push_str("FEATURE COMPARISON\n");
            s.push_str("───────────────────────────────────────────────────────────────────\n");
            s.push_str("  Feature                    Reference        Generated\n");
            s.push_str("  ─────────────────────────────────────────────────────────────\n");

            let mut all_features: Vec<_> = self
                .ref_features
                .keys()
                .chain(self.gen_features.keys())
                .collect();
            all_features.sort_by_key(|f| format!("{}", f));
            all_features.dedup();

            for feature in all_features {
                if *feature == FeatureType::Unknown {
                    continue;
                }

                let ref_stats = self.ref_features.get(feature);
                let gen_stats = self.gen_features.get(feature);

                let ref_str = ref_stats
                    .filter(|s| !s.is_empty())
                    .map(|s| format!("{:>6} moves", s.move_count))
                    .unwrap_or_else(|| "      -     ".to_string());

                let gen_str = gen_stats
                    .filter(|s| !s.is_empty())
                    .map(|s| format!("{:>6} moves", s.move_count))
                    .unwrap_or_else(|| "      -     ".to_string());

                s.push_str(&format!(
                    "  {:24} {:>12}    {:>12}\n",
                    format!("{}", feature),
                    ref_str,
                    gen_str
                ));
            }
            s.push('\n');
        }

        // Footer
        s.push_str("═══════════════════════════════════════════════════════════════════\n");

        s
    }

    /// Generate a JSON report.
    pub fn to_json(&self) -> Result<String, serde_json::Error> {
        serde_json::to_string_pretty(self)
    }

    /// Generate a compact JSON report (single line).
    pub fn to_json_compact(&self) -> Result<String, serde_json::Error> {
        serde_json::to_string(self)
    }

    /// Write the report to a file in the specified format.
    pub fn write_to_file<P: AsRef<Path>>(&self, path: P, format: ReportFormat) -> io::Result<()> {
        let content = match format {
            ReportFormat::Text => self.to_text(),
            ReportFormat::Json => self
                .to_json()
                .map_err(|e| io::Error::new(io::ErrorKind::Other, e))?,
            ReportFormat::Html => self.to_html(),
        };

        let mut file = std::fs::File::create(path)?;
        file.write_all(content.as_bytes())?;
        Ok(())
    }

    /// Generate an HTML report.
    pub fn to_html(&self) -> String {
        let status_class = if self.passed { "passed" } else { "failed" };
        let status_text = if self.passed { "PASSED" } else { "FAILED" };

        let mut html = String::new();

        html.push_str(r#"<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>G-code Validation Report</title>
    <style>
        body {
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, Oxygen, Ubuntu, sans-serif;
            max-width: 1000px;
            margin: 0 auto;
            padding: 20px;
            background: #f5f5f5;
        }
        .report {
            background: white;
            border-radius: 8px;
            box-shadow: 0 2px 4px rgba(0,0,0,0.1);
            padding: 24px;
        }
        h1 { color: #333; margin-top: 0; }
        h2 { color: #555; border-bottom: 1px solid #eee; padding-bottom: 8px; }
        .status {
            display: inline-block;
            padding: 8px 16px;
            border-radius: 4px;
            font-weight: bold;
            font-size: 1.2em;
        }
        .status.passed { background: #d4edda; color: #155724; }
        .status.failed { background: #f8d7da; color: #721c24; }
        .score {
            font-size: 2em;
            font-weight: bold;
            color: #333;
        }
        table {
            width: 100%;
            border-collapse: collapse;
            margin: 16px 0;
        }
        th, td {
            padding: 12px;
            text-align: left;
            border-bottom: 1px solid #eee;
        }
        th { background: #f8f9fa; font-weight: 600; }
        .number { text-align: right; font-family: monospace; }
        .diff-positive { color: #28a745; }
        .diff-negative { color: #dc3545; }
        .issue { padding: 8px 12px; margin: 4px 0; border-radius: 4px; }
        .issue.critical { background: #f8d7da; border-left: 4px solid #dc3545; }
        .issue.error { background: #fff3cd; border-left: 4px solid #ffc107; }
        .issue.warning { background: #d1ecf1; border-left: 4px solid #17a2b8; }
        .issue.info { background: #e2e3e5; border-left: 4px solid #6c757d; }
        .severity { font-weight: bold; margin-right: 8px; }
        .score-bar {
            height: 8px;
            background: #e9ecef;
            border-radius: 4px;
            overflow: hidden;
        }
        .score-fill {
            height: 100%;
            background: linear-gradient(90deg, #dc3545 0%, #ffc107 50%, #28a745 100%);
            border-radius: 4px;
        }
    </style>
</head>
<body>
    <div class="report">
"#);

        // Header
        html.push_str(&format!(
            r#"
        <h1>G-code Validation Report</h1>
        <p class="status {}">Status: {}</p>
        <p class="score">Quality Score: {:.1}/100</p>
        <div class="score-bar"><div class="score-fill" style="width: {:.1}%"></div></div>
"#,
            status_class, status_text, self.quality_score, self.quality_score
        ));

        // Summary
        html.push_str(r#"
        <h2>Summary</h2>
        <table>
            <tr><th>Metric</th><th class="number">Reference</th><th class="number">Generated</th><th class="number">Difference</th></tr>
"#);

        let diff_class = |v: f64| {
            if v >= 0.0 {
                "diff-positive"
            } else {
                "diff-negative"
            }
        };

        html.push_str(&format!(
            r#"
            <tr>
                <td>Layer Count</td>
                <td class="number">{}</td>
                <td class="number">{}</td>
                <td class="number {}">{}%</td>
            </tr>
"#,
            self.summary.ref_layer_count,
            self.summary.gen_layer_count,
            diff_class(self.summary.layer_count_diff_percent),
            if self.summary.layer_count_diff_percent >= 0.0 {
                "+"
            } else {
                ""
            },
        ));

        html.push_str(&format!(
            r#"
            <tr>
                <td>Total Extrusion (mm)</td>
                <td class="number">{:.2}</td>
                <td class="number">{:.2}</td>
                <td class="number {}">{}{:.1}%</td>
            </tr>
"#,
            self.summary.ref_total_extrusion,
            self.summary.gen_total_extrusion,
            diff_class(self.summary.extrusion_diff_percent),
            if self.summary.extrusion_diff_percent >= 0.0 {
                "+"
            } else {
                ""
            },
            self.summary.extrusion_diff_percent,
        ));

        html.push_str(&format!(
            r#"
            <tr>
                <td>Total Moves</td>
                <td class="number">{}</td>
                <td class="number">{}</td>
                <td class="number">{}{}</td>
            </tr>
"#,
            self.summary.ref_total_moves,
            self.summary.gen_total_moves,
            if self.summary.move_count_diff >= 0 {
                "+"
            } else {
                ""
            },
            self.summary.move_count_diff,
        ));

        html.push_str("        </table>\n");

        // Score Breakdown
        html.push_str(r#"
        <h2>Score Breakdown</h2>
        <table>
            <tr><th>Component</th><th class="number">Score</th><th class="number">Weight</th><th class="number">Weighted</th></tr>
"#);

        let components = [
            (
                "Layer Count",
                self.score_breakdown.layer_count_score,
                self.score_breakdown.weights.weight_layer_count,
            ),
            (
                "Total Extrusion",
                self.score_breakdown.total_extrusion_score,
                self.score_breakdown.weights.weight_total_extrusion,
            ),
            (
                "Layer Consistency",
                self.score_breakdown.layer_consistency_score,
                self.score_breakdown.weights.weight_layer_consistency,
            ),
            (
                "Coverage",
                self.score_breakdown.coverage_score,
                self.score_breakdown.weights.weight_coverage,
            ),
            (
                "Features",
                self.score_breakdown.feature_score,
                self.score_breakdown.weights.weight_features,
            ),
        ];

        for (name, score, weight) in components {
            html.push_str(&format!(
                r#"
            <tr>
                <td>{}</td>
                <td class="number">{:.1}</td>
                <td class="number">{:.0}%</td>
                <td class="number">{:.2}</td>
            </tr>
"#,
                name,
                score,
                weight * 100.0,
                score * weight
            ));
        }

        html.push_str("        </table>\n");

        // Issues
        let (critical, error, warning, info) = self.issue_counts();
        html.push_str(&format!(
            r#"
        <h2>Issues ({} critical, {} errors, {} warnings, {} info)</h2>
"#,
            critical, error, warning, info
        ));

        if self.issues.is_empty() {
            html.push_str("        <p>No issues found.</p>\n");
        } else {
            for issue in &self.issues {
                let class = match issue.severity {
                    IssueSeverity::Critical => "critical",
                    IssueSeverity::Error => "error",
                    IssueSeverity::Warning => "warning",
                    IssueSeverity::Info => "info",
                };
                let layer_str = issue
                    .layer
                    .map(|l| format!(" [Layer {}]", l))
                    .unwrap_or_default();
                html.push_str(&format!(
                    r#"
        <div class="issue {}">
            <span class="severity">{}</span>{}{}
        </div>
"#,
                    class, issue.severity, layer_str, issue.message
                ));
            }
        }

        // Footer
        html.push_str(
            r#"
    </div>
</body>
</html>
"#,
        );

        html
    }
}

/// Output format for validation reports.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReportFormat {
    /// Plain text format
    Text,
    /// JSON format
    Json,
    /// HTML format
    Html,
}

impl ReportFormat {
    // Parse format from string.
    pub fn from_str(s: &str) -> Option<Self> {
        match s.to_lowercase().as_str() {
            "text" | "txt" => Some(ReportFormat::Text),
            "json" => Some(ReportFormat::Json),
            "html" | "htm" => Some(ReportFormat::Html),
            _ => None,
        }
    }

    /// Get file extension for this format.
    pub fn extension(&self) -> &'static str {
        match self {
            ReportFormat::Text => "txt",
            ReportFormat::Json => "json",
            ReportFormat::Html => "html",
        }
    }
}

/// Quick validation function for comparing two G-code files.
pub fn validate_gcode_files<P: AsRef<Path>>(
    reference: P,
    generated: P,
) -> io::Result<ValidationReport> {
    ValidationReport::from_files(reference, generated, ValidationConfig::default())
}

/// Quick validation function with custom config.
pub fn validate_gcode_files_with_config<P: AsRef<Path>>(
    reference: P,
    generated: P,
    config: ValidationConfig,
) -> io::Result<ValidationReport> {
    ValidationReport::from_files(reference, generated, config)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_simple_gcode(layers: usize, extrusion_per_layer: f64) -> String {
        let mut gcode = String::new();
        gcode.push_str("; Simple test G-code\n");
        gcode.push_str("G28 ; Home\n");
        gcode.push_str("M82 ; Absolute extrusion\n");

        let mut e = 0.0;
        for i in 0..layers {
            let z = (i + 1) as f64 * 0.2;
            gcode.push_str(&format!("; LAYER:{}\n", i));
            gcode.push_str(&format!("G1 Z{:.3} F1000\n", z));
            gcode.push_str(&format!(
                "G1 X10 Y10 E{:.4} F1500\n",
                e + extrusion_per_layer * 0.25
            ));
            gcode.push_str(&format!(
                "G1 X20 Y10 E{:.4}\n",
                e + extrusion_per_layer * 0.5
            ));
            gcode.push_str(&format!(
                "G1 X20 Y20 E{:.4}\n",
                e + extrusion_per_layer * 0.75
            ));
            gcode.push_str(&format!("G1 X10 Y20 E{:.4}\n", e + extrusion_per_layer));
            e += extrusion_per_layer;
        }

        gcode
    }

    #[test]
    fn test_feature_type_from_comment() {
        // PrusaSlicer/Cura style TYPE comments
        assert_eq!(
            FeatureType::from_comment("; TYPE:External perimeter"),
            Some(FeatureType::ExternalPerimeter)
        );
        assert_eq!(
            FeatureType::from_comment("; TYPE:Inner wall"),
            Some(FeatureType::InternalPerimeter)
        );
        assert_eq!(
            FeatureType::from_comment("; TYPE:Solid infill"),
            Some(FeatureType::SolidInfill)
        );
        assert_eq!(
            FeatureType::from_comment("; TYPE:Support"),
            Some(FeatureType::Support)
        );

        // BambuStudio FEATURE style comments
        assert_eq!(
            FeatureType::from_comment("; FEATURE: Outer wall"),
            Some(FeatureType::ExternalPerimeter)
        );
        assert_eq!(
            FeatureType::from_comment("; FEATURE: Inner wall"),
            Some(FeatureType::InternalPerimeter)
        );
        assert_eq!(
            FeatureType::from_comment("; FEATURE: Sparse infill"),
            Some(FeatureType::SparseInfill)
        );
        assert_eq!(
            FeatureType::from_comment("; FEATURE: Solid infill"),
            Some(FeatureType::SolidInfill)
        );
        assert_eq!(
            FeatureType::from_comment("; FEATURE: Bridge"),
            Some(FeatureType::BridgeInfill)
        );
        assert_eq!(
            FeatureType::from_comment("; FEATURE: Support"),
            Some(FeatureType::Support)
        );

        // WIPE markers
        assert_eq!(
            FeatureType::from_comment("; WIPE_START"),
            Some(FeatureType::Wipe)
        );
        assert_eq!(
            FeatureType::from_comment("; WIPE_END"),
            Some(FeatureType::Wipe)
        );

        // Random comment should not match
        assert_eq!(FeatureType::from_comment("; some random comment"), None);
        assert_eq!(
            FeatureType::from_comment("; layer num/total_layer_count: 1/240"),
            None
        );
    }

    #[test]
    fn test_validation_config_default() {
        let config = ValidationConfig::default();
        assert!((config.layer_count_tolerance - 0.05).abs() < 1e-6);
        assert!((config.total_extrusion_tolerance - 0.10).abs() < 1e-6);
        assert!((config.pass_threshold - 100.0).abs() < 1e-6);
    }

    #[test]
    fn test_validation_config_strict() {
        let config = ValidationConfig::strict();
        assert!(config.layer_count_tolerance < ValidationConfig::default().layer_count_tolerance);
        assert!(config.pass_threshold >= ValidationConfig::default().pass_threshold);
    }

    #[test]
    fn test_validation_identical_gcode() {
        let gcode = make_simple_gcode(10, 5.0);
        let reference = ParsedGCode::from_string(&gcode);
        let generated = ParsedGCode::from_string(&gcode);

        let report =
            ValidationReport::generate(&reference, &generated, ValidationConfig::default());

        assert!(report.passed());
        assert!((report.quality_score() - 100.0).abs() < 1.0);
        assert!(report.issues.is_empty());
    }

    #[test]
    fn test_validation_different_layer_count() {
        let ref_gcode = make_simple_gcode(10, 5.0);
        let gen_gcode = make_simple_gcode(8, 5.0);

        let reference = ParsedGCode::from_string(&ref_gcode);
        let generated = ParsedGCode::from_string(&gen_gcode);

        let report =
            ValidationReport::generate(&reference, &generated, ValidationConfig::default());

        assert_eq!(report.summary.ref_layer_count, 10);
        assert_eq!(report.summary.gen_layer_count, 8);
        assert!(report.summary.layer_count_diff_percent < 0.0);
        assert!(!report.issues.is_empty());
    }

    #[test]
    fn test_validation_different_extrusion() {
        let ref_gcode = make_simple_gcode(10, 5.0);
        let gen_gcode = make_simple_gcode(10, 4.0);

        let reference = ParsedGCode::from_string(&ref_gcode);
        let generated = ParsedGCode::from_string(&gen_gcode);

        let report =
            ValidationReport::generate(&reference, &generated, ValidationConfig::default());

        assert!(report.summary.extrusion_diff_percent < 0.0);
        assert!(report.quality_score() < 100.0);
    }

    #[test]
    fn test_validation_report_text_output() {
        let gcode = make_simple_gcode(5, 3.0);
        let reference = ParsedGCode::from_string(&gcode);
        let generated = ParsedGCode::from_string(&gcode);

        let report =
            ValidationReport::generate(&reference, &generated, ValidationConfig::default());
        let text = report.to_text();

        assert!(text.contains("VALIDATION REPORT"));
        assert!(text.contains("PASSED"));
        assert!(text.contains("Layer Count"));
        assert!(text.contains("Total Extrusion"));
    }

    #[test]
    fn test_validation_report_json_output() {
        let gcode = make_simple_gcode(5, 3.0);
        let reference = ParsedGCode::from_string(&gcode);
        let generated = ParsedGCode::from_string(&gcode);

        let report =
            ValidationReport::generate(&reference, &generated, ValidationConfig::default());
        let json = report.to_json().expect("JSON serialization should succeed");

        assert!(json.contains("quality_score"));
        assert!(json.contains("passed"));
        assert!(json.contains("summary"));
    }

    #[test]
    fn test_validation_report_html_output() {
        let gcode = make_simple_gcode(5, 3.0);
        let reference = ParsedGCode::from_string(&gcode);
        let generated = ParsedGCode::from_string(&gcode);

        let report =
            ValidationReport::generate(&reference, &generated, ValidationConfig::default());
        let html = report.to_html();

        assert!(html.contains("<!DOCTYPE html>"));
        assert!(html.contains("G-code Validation Report"));
        assert!(html.contains("Quality Score"));
    }

    #[test]
    fn test_issue_counts() {
        let ref_gcode = make_simple_gcode(10, 5.0);
        let gen_gcode = make_simple_gcode(5, 2.0); // Very different

        let reference = ParsedGCode::from_string(&ref_gcode);
        let generated = ParsedGCode::from_string(&gen_gcode);

        let report =
            ValidationReport::generate(&reference, &generated, ValidationConfig::default());
        let (critical, error, warning, _info) = report.issue_counts();

        // Should have issues due to large differences
        assert!(critical + error + warning > 0);
    }

    #[test]
    fn test_report_format_from_str() {
        assert_eq!(ReportFormat::from_str("text"), Some(ReportFormat::Text));
        assert_eq!(ReportFormat::from_str("txt"), Some(ReportFormat::Text));
        assert_eq!(ReportFormat::from_str("json"), Some(ReportFormat::Json));
        assert_eq!(ReportFormat::from_str("JSON"), Some(ReportFormat::Json));
        assert_eq!(ReportFormat::from_str("html"), Some(ReportFormat::Html));
        assert_eq!(ReportFormat::from_str("htm"), Some(ReportFormat::Html));
        assert_eq!(ReportFormat::from_str("unknown"), None);
    }

    #[test]
    fn test_report_format_extension() {
        assert_eq!(ReportFormat::Text.extension(), "txt");
        assert_eq!(ReportFormat::Json.extension(), "json");
        assert_eq!(ReportFormat::Html.extension(), "html");
    }

    #[test]
    fn test_layer_validation_passed() {
        let layer = LayerValidation {
            layer_num: 0,
            z_height: 0.2,
            ref_extrusion: 5.0,
            gen_extrusion: 5.0,
            extrusion_diff_percent: 0.0,
            ref_moves: 10,
            gen_moves: 10,
            match_score: 100.0,
            issues: vec![],
        };

        assert!(layer.passed(70.0));
        assert!(layer.passed(100.0));
    }

    #[test]
    fn test_score_breakdown_total() {
        let breakdown = ScoreBreakdown {
            layer_count_score: 100.0,
            total_extrusion_score: 100.0,
            layer_consistency_score: 100.0,
            coverage_score: 100.0,
            feature_score: 100.0,
            weights: ValidationConfig::default(),
        };

        // With default weights summing to 1.0, total should be 100.0
        assert!((breakdown.total() - 100.0).abs() < 1e-6);
    }

    #[test]
    fn test_feature_stats_is_empty() {
        let empty = FeatureStats::default();
        assert!(empty.is_empty());

        let non_empty = FeatureStats {
            move_count: 1,
            ..Default::default()
        };
        assert!(!non_empty.is_empty());
    }

    #[test]
    fn test_feature_type_is_structural() {
        // Structural features (contribute to the printed object)
        assert!(FeatureType::ExternalPerimeter.is_structural());
        assert!(FeatureType::InternalPerimeter.is_structural());
        assert!(FeatureType::SolidInfill.is_structural());
        assert!(FeatureType::SparseInfill.is_structural());
        assert!(FeatureType::BridgeInfill.is_structural());
        assert!(FeatureType::Support.is_structural());
        assert!(FeatureType::SupportInterface.is_structural());
        assert!(FeatureType::Skirt.is_structural());
        assert!(FeatureType::Brim.is_structural());
        assert!(FeatureType::Raft.is_structural());

        // Non-structural features (auxiliary moves)
        assert!(!FeatureType::Travel.is_structural());
        assert!(!FeatureType::Wipe.is_structural());
        assert!(!FeatureType::Prime.is_structural());
        assert!(!FeatureType::Unknown.is_structural());
    }
}
