//! Tool Ordering Module for Multi-Extruder Coordination
//!
//! This module implements tool ordering and scheduling for multi-material printing,
//! porting the functionality from BambuStudio's `GCode/ToolOrdering.cpp`.
//!
//! The tool ordering system determines the optimal sequence of extruder switches
//! across all layers to minimize:
//! - Total filament waste (flush/purge volume)
//! - Number of tool changes
//! - Print time
//!
//! ## Key Concepts
//!
//! - **LayerTools**: Per-layer information about required extruders and toolchanges
//! - **Extruder Order**: The sequence of extruders within a layer
//! - **Flush Volume**: Amount of filament needed when switching between two materials
//! - **Wipe Tower Partitions**: Number of wipe tower segments needed per layer
//!
//! ## Algorithm Overview
//!
//! 1. Collect all extruders needed per layer from objects and supports
//! 2. Reorder extruders within each layer to minimize flush volume
//! 3. Handle "don't care" extruders (regions that can be printed with any extruder)
//! 4. Calculate wipe tower partition requirements
//! 5. Assign custom G-codes (color changes, pauses) to appropriate layers
//!
//! ## Reference
//!
//! - `BambuStudio/src/libslic3r/GCode/ToolOrdering.hpp`
//! - `BambuStudio/src/libslic3r/GCode/ToolOrdering.cpp`

use std::collections::{HashMap, HashSet};

use crate::clipper_utils::{offset_expolygon, OffsetJoinType};
use crate::geometry::ExPolygon;

// ============================================================================
// Constants
// ============================================================================

/// Small epsilon for floating point layer height comparisons
const LAYER_HEIGHT_EPSILON: f64 = 1e-6;

/// Similar color threshold for automatic grouping (Delta E 2000)
const SIMILAR_COLOR_THRESHOLD_DE2000: f64 = 20.0;

/// Default flush volume when not specified (mm³)
const DEFAULT_FLUSH_VOLUME: f32 = 140.0;

// ============================================================================
// Types and Enums
// ============================================================================

/// Statistics about filament changes for a print
#[derive(Debug, Clone, Default, PartialEq)]
pub struct FilamentChangeStats {
    /// Total flush weight in grams
    pub filament_flush_weight: i32,
    /// Number of filament changes
    pub filament_change_count: i32,
    /// Number of extruder changes (for multi-nozzle systems)
    pub extruder_change_count: i32,
}

impl FilamentChangeStats {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn clear(&mut self) {
        self.filament_flush_weight = 0;
        self.filament_change_count = 0;
        self.extruder_change_count = 0;
    }
}

impl std::ops::Add for FilamentChangeStats {
    type Output = Self;

    fn add(self, other: Self) -> Self {
        Self {
            filament_flush_weight: self.filament_flush_weight + other.filament_flush_weight,
            filament_change_count: self.filament_change_count + other.filament_change_count,
            extruder_change_count: self.extruder_change_count + other.extruder_change_count,
        }
    }
}

impl std::ops::AddAssign for FilamentChangeStats {
    fn add_assign(&mut self, other: Self) {
        self.filament_flush_weight += other.filament_flush_weight;
        self.filament_change_count += other.filament_change_count;
        self.extruder_change_count += other.extruder_change_count;
    }
}

/// Mode for filament change statistics retrieval
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FilamentChangeMode {
    /// Single extruder statistics
    SingleExt,
    /// Multi-extruder with best grouping
    MultiExtBest,
    /// Multi-extruder with current grouping
    MultiExtCurr,
}

/// Mode for automatic filament mapping
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum FilamentMapMode {
    /// Automatic mapping for minimum flush
    #[default]
    AutoForFlush,
    /// Automatic mapping for color matching
    AutoForMatch,
    /// Manual mapping
    Manual,
    /// Manual nozzle-specific mapping
    NozzleManual,
}

/// Type of custom G-code event
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CustomGCodeType {
    /// Color change (M600)
    ColorChange,
    /// Tool/extruder change
    ToolChange,
    /// Pause print
    Pause,
    /// Custom G-code template
    Custom,
}

/// Custom G-code event at a specific layer
#[derive(Debug, Clone, PartialEq)]
pub struct CustomGCodeItem {
    /// Print Z height for this event
    pub print_z: f64,
    /// Type of G-code event
    pub gcode_type: CustomGCodeType,
    /// Extruder index (1-based, for color/tool changes)
    pub extruder: i32,
    /// New color (hex string, for color changes)
    pub color: String,
    /// Custom G-code content (for custom events)
    pub extra: String,
}

impl CustomGCodeItem {
    pub fn new(print_z: f64, gcode_type: CustomGCodeType) -> Self {
        Self {
            print_z,
            gcode_type,
            extruder: 0,
            color: String::new(),
            extra: String::new(),
        }
    }

    pub fn color_change(print_z: f64, extruder: i32, color: &str) -> Self {
        Self {
            print_z,
            gcode_type: CustomGCodeType::ColorChange,
            extruder,
            color: color.to_string(),
            extra: String::new(),
        }
    }

    pub fn tool_change(print_z: f64, extruder: i32) -> Self {
        Self {
            print_z,
            gcode_type: CustomGCodeType::ToolChange,
            extruder,
            color: String::new(),
            extra: String::new(),
        }
    }

    pub fn pause(print_z: f64) -> Self {
        Self {
            print_z,
            gcode_type: CustomGCodeType::Pause,
            extruder: 0,
            color: String::new(),
            extra: String::new(),
        }
    }

    pub fn custom(print_z: f64, gcode: &str) -> Self {
        Self {
            print_z,
            gcode_type: CustomGCodeType::Custom,
            extruder: 0,
            color: String::new(),
            extra: gcode.to_string(),
        }
    }
}

/// Extrusion role for determining filament assignment
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ExtrusionRoleType {
    None,
    Perimeter,
    ExternalPerimeter,
    OverhangPerimeter,
    InternalInfill,
    SolidInfill,
    TopSolidInfill,
    BridgeInfill,
    GapFill,
    Skirt,
    SupportMaterial,
    SupportMaterialInterface,
    SupportTransition,
    Mixed,
}

impl ExtrusionRoleType {
    // Check if role is a perimeter type
    pub fn is_perimeter(&self) -> bool {
        matches!(
            self,
            ExtrusionRoleType::Perimeter
                | ExtrusionRoleType::ExternalPerimeter
                | ExtrusionRoleType::OverhangPerimeter
        )
    }

    /// Check if role is a solid infill type
    pub fn is_solid_infill(&self) -> bool {
        matches!(
            self,
            ExtrusionRoleType::SolidInfill
                | ExtrusionRoleType::TopSolidInfill
                | ExtrusionRoleType::BridgeInfill
        )
    }

    /// Check if role is any infill type
    pub fn is_infill(&self) -> bool {
        matches!(
            self,
            ExtrusionRoleType::InternalInfill
                | ExtrusionRoleType::SolidInfill
                | ExtrusionRoleType::TopSolidInfill
                | ExtrusionRoleType::BridgeInfill
        )
    }

    /// Check if role is support material
    pub fn is_support(&self) -> bool {
        matches!(
            self,
            ExtrusionRoleType::SupportMaterial
                | ExtrusionRoleType::SupportMaterialInterface
                | ExtrusionRoleType::SupportTransition
        )
    }
}

// ============================================================================
// Flush Matrix
// ============================================================================

/// Matrix of flush volumes between filament pairs
#[derive(Debug, Clone)]
pub struct FlushMatrix {
    /// Number of filaments
    size: usize,
    /// Flush volumes [from][to] in mm³
    volumes: Vec<Vec<f32>>,
}

impl FlushMatrix {
    // Create a new flush matrix with given size and default volume
    pub fn new(size: usize, default_volume: f32) -> Self {
        let volumes = vec![vec![default_volume; size]; size];
        let mut matrix = Self { size, volumes };
        // Diagonal is always zero (same filament)
        for i in 0..size {
            matrix.volumes[i][i] = 0.0;
        }
        matrix
    }

    /// Create from a flat vector of values (row-major order)
    pub fn from_flat(size: usize, values: &[f32]) -> Self {
        assert!(values.len() >= size * size);
        let mut volumes = vec![vec![0.0; size]; size];
        for i in 0..size {
            for j in 0..size {
                volumes[i][j] = values[i * size + j];
            }
        }
        Self { size, volumes }
    }

    /// Get flush volume from one filament to another
    pub fn get(&self, from: usize, to: usize) -> f32 {
        if from < self.size && to < self.size {
            self.volumes[from][to]
        } else {
            DEFAULT_FLUSH_VOLUME
        }
    }

    /// Set flush volume from one filament to another
    pub fn set(&mut self, from: usize, to: usize, volume: f32) {
        if from < self.size && to < self.size {
            self.volumes[from][to] = volume;
        }
    }

    /// Get the matrix size (number of filaments)
    pub fn size(&self) -> usize {
        self.size
    }

    /// Apply a multiplier to all flush volumes
    pub fn apply_multiplier(&mut self, multiplier: f32) {
        for row in &mut self.volumes {
            for vol in row {
                *vol *= multiplier;
            }
        }
    }

    /// Calculate total flush volume for a sequence of filaments
    pub fn total_flush_for_sequence(&self, sequence: &[usize]) -> f32 {
        if sequence.len() < 2 {
            return 0.0;
        }
        let mut total = 0.0;
        for i in 0..sequence.len() - 1 {
            total += self.get(sequence[i], sequence[i + 1]);
        }
        total
    }
}

impl Default for FlushMatrix {
    fn default() -> Self {
        Self::new(0, DEFAULT_FLUSH_VOLUME)
    }
}

// ============================================================================
// Wiping Extrusions
// ============================================================================

/// Tracks which extrusions are used for wiping during tool changes
#[derive(Debug, Clone, Default)]
pub struct WipingExtrusions {
    /// Map of (entity_id, object_id) -> extruder overrides per copy
    entity_overrides: HashMap<(u64, u64), Vec<i32>>,
    /// Support extruder overrides per object
    support_overrides: HashMap<u64, i32>,
    /// Support interface extruder overrides per object
    support_interface_overrides: HashMap<u64, i32>,
    /// Whether any extrusion has been overridden
    something_overridden: bool,
    /// Whether any extrusion is overridable
    something_overridable: bool,
}

impl WipingExtrusions {
    pub fn new() -> Self {
        Self::default()
    }

    /// Check if any extrusions have been overridden
    pub fn is_anything_overridden(&self) -> bool {
        self.something_overridden
    }

    /// Check if any extrusions are overridable
    pub fn is_anything_overridable(&self) -> bool {
        self.something_overridable
    }

    /// Mark that something is overridable
    pub fn mark_overridable(&mut self) {
        self.something_overridable = true;
    }

    /// Set extruder override for an extrusion entity
    pub fn set_extruder_override(
        &mut self,
        entity_id: u64,
        object_id: u64,
        copy_id: usize,
        extruder: i32,
        num_copies: usize,
    ) {
        self.something_overridden = true;
        let key = (entity_id, object_id);
        let overrides = self
            .entity_overrides
            .entry(key)
            .or_insert_with(|| vec![-1; num_copies]);
        if overrides.len() < num_copies {
            overrides.resize(num_copies, -1);
        }
        if copy_id < overrides.len() {
            overrides[copy_id] = extruder;
        }
    }

    /// Set support extruder override for an object
    pub fn set_support_extruder_override(&mut self, object_id: u64, extruder: i32) {
        self.something_overridden = true;
        self.support_overrides.insert(object_id, extruder);
    }

    /// Set support interface extruder override for an object
    pub fn set_support_interface_extruder_override(&mut self, object_id: u64, extruder: i32) {
        self.something_overridden = true;
        self.support_interface_overrides.insert(object_id, extruder);
    }

    /// Check if an entity is overridden for a specific copy
    pub fn is_entity_overridden(&self, entity_id: u64, object_id: u64, copy_id: usize) -> bool {
        if let Some(overrides) = self.entity_overrides.get(&(entity_id, object_id)) {
            if copy_id < overrides.len() {
                return overrides[copy_id] != -1;
            }
        }
        false
    }

    /// Check if support is overridden for an object
    pub fn is_support_overridden(&self, object_id: u64) -> bool {
        self.support_overrides.contains_key(&object_id)
    }

    /// Check if support interface is overridden for an object
    pub fn is_support_interface_overridden(&self, object_id: u64) -> bool {
        self.support_interface_overrides.contains_key(&object_id)
    }

    /// Get extruder override for an entity
    /// Returns the override vector, or None if not overridden
    pub fn get_extruder_overrides(
        &mut self,
        entity_id: u64,
        object_id: u64,
        correct_extruder_id: i32,
        num_copies: usize,
    ) -> Option<&Vec<i32>> {
        let key = (entity_id, object_id);
        if let Some(overrides) = self.entity_overrides.get_mut(&key) {
            overrides.resize(num_copies, -1);
            // Replace -1 with encoded correct extruder (-correct_extruder_id - 1)
            for ext in overrides.iter_mut() {
                if *ext == -1 {
                    *ext = -correct_extruder_id - 1;
                }
            }
            return Some(overrides);
        }
        None
    }

    /// Get support extruder override for an object
    pub fn get_support_extruder_override(&self, object_id: u64) -> Option<i32> {
        self.support_overrides.get(&object_id).copied()
    }

    /// Get support interface extruder override for an object
    pub fn get_support_interface_extruder_override(&self, object_id: u64) -> Option<i32> {
        self.support_interface_overrides.get(&object_id).copied()
    }

    /// Clear all overrides
    pub fn clear(&mut self) {
        self.entity_overrides.clear();
        self.support_overrides.clear();
        self.support_interface_overrides.clear();
        self.something_overridden = false;
        self.something_overridable = false;
    }
}

// ============================================================================
// Flush-into-object eligibility (WipingExtrusions::is_*_overriddable)
//
// C++ ToolOrdering.cpp:2645-2690. These decide whether an object extrusion may
// absorb part of a tool change's purge instead of the wipe tower. Kept as free
// functions taking the config values directly so they can be used both from a
// full `WipingExtrusions` walk and from the Tier-1 wipe-tower pre-pass (which
// has no central `LayerTools`).
// ============================================================================

/// Port of `WipingExtrusions::is_overriddable` (ToolOrdering.cpp:2645).
///
/// `filament_soluble` is the soluble flag of the filament that would print this
/// collection; soluble filament is never used for wiping.
pub fn is_overriddable(
    role: crate::extrusion_entity::ExtrusionRole,
    filament_soluble: bool,
    flush_into_objects: bool,
    flush_into_infill: bool,
) -> bool {
    // cpp:2647-2648
    if filament_soluble {
        return false;
    }
    // cpp:2650-2651
    if flush_into_objects {
        return true;
    }
    // cpp:2653-2654 — otherwise only *internal* infill qualifies.
    if !flush_into_infill || role != crate::extrusion_entity::ExtrusionRole::InternalInfill {
        return false;
    }
    true
}

/// Port of `WipingExtrusions::is_obj_overriddable` (ToolOrdering.cpp:2658).
pub fn is_obj_overriddable(
    role: crate::extrusion_entity::ExtrusionRole,
    flush_into_objects: bool,
    flush_into_infill: bool,
) -> bool {
    if flush_into_objects {
        return true;
    }
    if flush_into_infill && role == crate::extrusion_entity::ExtrusionRole::InternalInfill {
        return true;
    }
    false
}

/// Port of `WipingExtrusions::is_support_overriddable` (ToolOrdering.cpp:2670).
///
/// `support_filament` / `support_interface_filament` are the object's config
/// values; 0 means "use whatever filament is current", which is what makes the
/// support overriddable.
pub fn is_support_overriddable(
    role: crate::extrusion_entity::ExtrusionRole,
    flush_into_support: bool,
    support_filament: u32,
    support_interface_filament: u32,
) -> bool {
    use crate::extrusion_entity::ExtrusionRole;
    if !flush_into_support {
        return false;
    }
    match role {
        ExtrusionRole::Mixed => support_filament == 0 || support_interface_filament == 0,
        // cpp:2679 groups erSupportMaterial with erSupportTransition.
        ExtrusionRole::SupportMaterial | ExtrusionRole::SupportTransition => support_filament == 0,
        ExtrusionRole::SupportMaterialInterface => support_interface_filament == 0,
        _ => false,
    }
}

// ============================================================================
// Layer Tools
// ============================================================================

/// Per-layer information about extruders and toolchanges
#[derive(Debug, Clone)]
pub struct LayerTools {
    /// Print Z height of this layer
    pub print_z: f64,
    /// Whether this layer has object extrusions
    pub has_object: bool,
    /// Whether this layer has support extrusions
    pub has_support: bool,
    /// Zero-based extruder IDs, ordered to minimize tool switches
    pub extruders: Vec<u32>,
    /// Extruder override for the whole layer (0 = no override, 1-based otherwise)
    pub extruder_override: u32,
    /// Whether a skirt should be printed at this layer
    pub has_skirt: bool,
    /// Whether wipe tower is active at this layer
    pub has_wipe_tower: bool,
    /// Number of wipe tower partitions at this layer
    pub wipe_tower_partitions: usize,
    /// Wipe tower layer height
    pub wipe_tower_layer_height: f64,
    /// Custom G-code to be performed before this layer
    pub custom_gcode: Option<CustomGCodeItem>,
    /// Wiping extrusions for this layer
    wiping_extrusions: WipingExtrusions,
}

impl LayerTools {
    // Create new layer tools for a given Z height
    pub fn new(print_z: f64) -> Self {
        Self {
            print_z,
            has_object: false,
            has_support: false,
            extruders: Vec::new(),
            extruder_override: 0,
            has_skirt: false,
            has_wipe_tower: false,
            wipe_tower_partitions: 0,
            wipe_tower_layer_height: 0.0,
            custom_gcode: None,
            wiping_extrusions: WipingExtrusions::new(),
        }
    }

    /// Check if extruder `a` comes before extruder `b` in the order
    pub fn is_extruder_order(&self, a: u32, b: u32) -> bool {
        if a == b {
            return false;
        }
        for &extruder in &self.extruders {
            if extruder == a {
                return true;
            }
            if extruder == b {
                return false;
            }
        }
        false
    }

    /// Check if this layer uses a specific extruder
    pub fn has_extruder(&self, extruder: u32) -> bool {
        self.extruders.contains(&extruder)
    }

    /// Get the wall filament for a region (considering override)
    pub fn wall_filament(&self, region_wall_filament: u32) -> u32 {
        if self.extruder_override == 0 {
            region_wall_filament.saturating_sub(1) // Convert 1-based to 0-based
        } else {
            self.extruder_override - 1
        }
    }

    /// Get the sparse infill filament for a region (considering override)
    pub fn sparse_infill_filament(&self, region_sparse_infill_filament: u32) -> u32 {
        if self.extruder_override == 0 {
            region_sparse_infill_filament.saturating_sub(1)
        } else {
            self.extruder_override - 1
        }
    }

    /// Get the solid infill filament for a region (considering override)
    pub fn solid_infill_filament(&self, region_solid_infill_filament: u32) -> u32 {
        if self.extruder_override == 0 {
            region_solid_infill_filament.saturating_sub(1)
        } else {
            self.extruder_override - 1
        }
    }

    /// Get the extruder for a given extrusion role and region config
    pub fn extruder_for_role(
        &self,
        role: ExtrusionRoleType,
        wall_filament: u32,
        sparse_infill_filament: u32,
        solid_infill_filament: u32,
    ) -> u32 {
        if self.extruder_override != 0 {
            return self.extruder_override - 1;
        }

        let extruder = match role {
            ExtrusionRoleType::Perimeter
            | ExtrusionRoleType::ExternalPerimeter
            | ExtrusionRoleType::OverhangPerimeter => wall_filament,
            ExtrusionRoleType::SolidInfill
            | ExtrusionRoleType::TopSolidInfill
            | ExtrusionRoleType::BridgeInfill => solid_infill_filament,
            ExtrusionRoleType::InternalInfill => sparse_infill_filament,
            _ => wall_filament, // Default to wall filament
        };

        if extruder == 0 {
            0
        } else {
            extruder - 1
        }
    }

    /// Get mutable reference to wiping extrusions
    pub fn wiping_extrusions_mut(&mut self) -> &mut WipingExtrusions {
        &mut self.wiping_extrusions
    }

    /// Get reference to wiping extrusions
    pub fn wiping_extrusions(&self) -> &WipingExtrusions {
        &self.wiping_extrusions
    }
}

impl PartialEq for LayerTools {
    fn eq(&self, other: &Self) -> bool {
        (self.print_z - other.print_z).abs() < LAYER_HEIGHT_EPSILON
    }
}

impl PartialOrd for LayerTools {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        self.print_z.partial_cmp(&other.print_z)
    }
}

// ============================================================================
// Tool Ordering Configuration
// ============================================================================

/// Configuration for tool ordering
#[derive(Debug, Clone)]
pub struct ToolOrderingConfig {
    /// Number of filaments/extruders
    pub num_filaments: usize,
    /// Nozzle diameters for each extruder
    pub nozzle_diameters: Vec<f64>,
    /// Filament densities (g/cm³)
    pub filament_densities: Vec<f64>,
    /// Filament solubility flags
    pub filament_soluble: Vec<bool>,
    /// Filament is support material flags
    pub filament_is_support: Vec<bool>,
    /// Filament types (PLA, ABS, PETG, etc.)
    pub filament_types: Vec<String>,
    /// Filament colors (hex strings)
    pub filament_colors: Vec<String>,
    /// Flush matrix between filament pairs
    pub flush_matrix: FlushMatrix,
    /// Flush volume multipliers per extruder
    pub flush_multipliers: Vec<f32>,
    /// Enable prime/wipe tower
    pub enable_prime_tower: bool,
    /// Enable wrapping detection layers
    pub wrapping_detection_layers: usize,
    /// Enable wrapping detection
    pub enable_wrapping_detection: bool,
    /// Print infill before perimeters
    pub infill_first: bool,
    /// Filament map mode
    pub filament_map_mode: FilamentMapMode,
    /// Manual filament to extruder mapping (1-based)
    pub filament_map: Vec<i32>,
    /// Timelapse mode (smooth requires wipe tower)
    pub timelapse_smooth: bool,
    /// First layer print sequence (1-based extruder IDs)
    pub first_layer_print_sequence: Vec<i32>,
    /// Custom layer print sequences: ((start_layer, end_layer), extruder_sequence)
    pub other_layers_print_sequences: Vec<((i32, i32), Vec<i32>)>,
    /// Maximum layer height
    pub max_layer_height: f64,
}

impl Default for ToolOrderingConfig {
    fn default() -> Self {
        Self {
            num_filaments: 1,
            nozzle_diameters: vec![0.4],
            filament_densities: vec![1.24],
            filament_soluble: vec![false],
            filament_is_support: vec![false],
            filament_types: vec!["PLA".to_string()],
            filament_colors: vec!["#FFFFFF".to_string()],
            flush_matrix: FlushMatrix::new(1, DEFAULT_FLUSH_VOLUME),
            flush_multipliers: vec![1.0],
            enable_prime_tower: false,
            wrapping_detection_layers: 0,
            enable_wrapping_detection: false,
            infill_first: false,
            filament_map_mode: FilamentMapMode::AutoForFlush,
            filament_map: vec![1],
            timelapse_smooth: false,
            first_layer_print_sequence: Vec::new(),
            other_layers_print_sequences: Vec::new(),
            max_layer_height: 0.3,
        }
    }
}

impl ToolOrderingConfig {
    // Create a new config with the given number of filaments
    pub fn new(num_filaments: usize) -> Self {
        Self {
            num_filaments,
            nozzle_diameters: vec![0.4; num_filaments],
            filament_densities: vec![1.24; num_filaments],
            filament_soluble: vec![false; num_filaments],
            filament_is_support: vec![false; num_filaments],
            filament_types: vec!["PLA".to_string(); num_filaments],
            filament_colors: (0..num_filaments)
                .map(|i| format!("#{:06X}", (i * 0x333333) % 0xFFFFFF))
                .collect(),
            flush_matrix: FlushMatrix::new(num_filaments, DEFAULT_FLUSH_VOLUME),
            flush_multipliers: vec![1.0; num_filaments],
            filament_map: (1..=num_filaments as i32).collect(),
            ..Default::default()
        }
    }

    /// Check if a filament is soluble
    pub fn is_filament_soluble(&self, filament_id: usize) -> bool {
        self.filament_soluble
            .get(filament_id)
            .copied()
            .unwrap_or(false)
    }

    /// Check if a filament is support material
    pub fn is_filament_support(&self, filament_id: usize) -> bool {
        self.filament_is_support
            .get(filament_id)
            .copied()
            .unwrap_or(false)
    }

    /// Get flush volume between two filaments
    pub fn get_flush_volume(&self, from: usize, to: usize) -> f32 {
        let base = self.flush_matrix.get(from, to);
        let multiplier = self.flush_multipliers.get(to).copied().unwrap_or(1.0);
        base * multiplier
    }
}

// ============================================================================
// Tool Ordering
// ============================================================================

/// Main tool ordering engine
#[derive(Debug, Clone)]
pub struct ToolOrdering {
    /// Per-layer tool information
    layer_tools: Vec<LayerTools>,
    /// First printing extruder (0-based)
    first_printing_extruder: Option<u32>,
    /// Last printing extruder (0-based)
    last_printing_extruder: Option<u32>,
    /// All extruders used in the print (0-based)
    all_printing_extruders: Vec<u32>,
    /// Configuration
    config: ToolOrderingConfig,
    /// Statistics for single extruder mode
    stats_single_ext: FilamentChangeStats,
    /// Statistics for multi-extruder best grouping
    stats_multi_ext_best: FilamentChangeStats,
    /// Statistics for multi-extruder current grouping
    stats_multi_ext_curr: FilamentChangeStats,
    /// Most frequently used extruder
    most_used_extruder: u32,
    /// Whether extruders have been sorted/reordered
    sorted: bool,
}

impl ToolOrdering {
    // Create a new tool ordering instance with the given configuration
    pub fn new(config: ToolOrderingConfig) -> Self {
        Self {
            layer_tools: Vec::new(),
            first_printing_extruder: None,
            last_printing_extruder: None,
            all_printing_extruders: Vec::new(),
            config,
            stats_single_ext: FilamentChangeStats::default(),
            stats_multi_ext_best: FilamentChangeStats::default(),
            stats_multi_ext_curr: FilamentChangeStats::default(),
            most_used_extruder: 0,
            sorted: false,
        }
    }

    /// Create tool ordering with default configuration
    pub fn with_default_config() -> Self {
        Self::new(ToolOrderingConfig::default())
    }

    /// Initialize layers from a list of Z heights
    pub fn initialize_layers(&mut self, mut z_heights: Vec<f64>) {
        // Sort and remove duplicates
        z_heights.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        z_heights.dedup_by(|a, b| (*a - *b).abs() < LAYER_HEIGHT_EPSILON);

        // Merge numerically very close Z values
        self.layer_tools.clear();
        let mut i = 0;
        while i < z_heights.len() {
            let z_start = z_heights[i];
            let z_max = z_start + LAYER_HEIGHT_EPSILON;
            let mut j = i + 1;
            while j < z_heights.len() && z_heights[j] <= z_max {
                j += 1;
            }
            // Use average Z for merged layers
            let avg_z = 0.5 * (z_heights[i] + z_heights[j - 1]);
            self.layer_tools.push(LayerTools::new(avg_z));
            i = j;
        }
    }

    /// Add an extruder requirement to a specific layer
    pub fn add_extruder_to_layer(&mut self, print_z: f64, extruder: u32, has_object: bool) {
        if let Some(layer) = self.layer_tools_for_layer_mut(print_z) {
            if !layer.extruders.contains(&extruder) {
                layer.extruders.push(extruder);
            }
            if has_object {
                layer.has_object = true;
            }
        }
    }

    /// Add support extruder requirement to a specific layer
    pub fn add_support_extruder_to_layer(
        &mut self,
        print_z: f64,
        extruder: u32,
        _is_interface: bool,
    ) {
        if let Some(layer) = self.layer_tools_for_layer_mut(print_z) {
            if !layer.extruders.contains(&extruder) {
                layer.extruders.push(extruder);
            }
            layer.has_support = true;
        }
    }

    /// Find the layer tools for a given Z height
    pub fn layer_tools_for_layer(&self, print_z: f64) -> Option<&LayerTools> {
        self.layer_tools
            .iter()
            .min_by(|a, b| {
                let dist_a = (a.print_z - print_z).abs();
                let dist_b = (b.print_z - print_z).abs();
                dist_a
                    .partial_cmp(&dist_b)
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
            .filter(|lt| (lt.print_z - print_z).abs() < LAYER_HEIGHT_EPSILON)
    }

    /// Find the layer tools for a given Z height (mutable)
    pub fn layer_tools_for_layer_mut(&mut self, print_z: f64) -> Option<&mut LayerTools> {
        self.layer_tools
            .iter_mut()
            .min_by(|a, b| {
                let dist_a = (a.print_z - print_z).abs();
                let dist_b = (b.print_z - print_z).abs();
                dist_a
                    .partial_cmp(&dist_b)
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
            .filter(|lt| (lt.print_z - print_z).abs() < LAYER_HEIGHT_EPSILON)
    }

    /// Handle "don't care" extruders (value 0) by assigning them based on context
    pub fn handle_dontcare_extruders(&mut self, first_extruder: Option<u32>) {
        if self.layer_tools.is_empty() {
            return;
        }

        // Determine the initial extruder
        let last_extruder = first_extruder.or_else(|| {
            // Find the first non-zero extruder
            for layer in &self.layer_tools {
                for &extruder in &layer.extruders {
                    if extruder > 0 {
                        return Some(extruder);
                    }
                }
            }
            None
        });

        // If still no extruder found, nothing to do
        let Some(mut last_ext) = last_extruder else {
            return;
        };

        // Process each layer
        for layer in &mut self.layer_tools {
            if layer.extruders.is_empty() {
                continue;
            }

            if layer.extruders.len() == 1 && layer.extruders[0] == 0 {
                // Single "don't care" extruder - use last extruder
                layer.extruders[0] = last_ext;
            } else {
                // Remove leading "don't care" extruder
                if layer.extruders.first() == Some(&0) {
                    layer.extruders.remove(0);
                }

                // Reorder to start with the last extruder if present
                if let Some(pos) = layer.extruders.iter().position(|&e| e == last_ext) {
                    if pos > 0 {
                        // Move last_ext to front
                        let ext = layer.extruders.remove(pos);
                        layer.extruders.insert(0, ext);
                    }
                }

                // On first layer with wipe tower, prefer soluble extruder at the beginning
                if self.config.enable_prime_tower && layer.print_z < LAYER_HEIGHT_EPSILON {
                    for i in 0..layer.extruders.len() {
                        if self.config.is_filament_soluble(layer.extruders[i] as usize) {
                            layer.extruders.swap(0, i);
                            break;
                        }
                    }
                }
            }

            if let Some(&ext) = layer.extruders.last() {
                last_ext = ext;
            }
        }
    }

    /// Handle "don't care" extruders with a specific first layer tool order
    pub fn handle_dontcare_extruders_with_first_layer_order(&mut self, first_layer_order: &[u32]) {
        if self.layer_tools.is_empty() || first_layer_order.is_empty() {
            return;
        }

        // Reorder first layer extruders according to the given order
        {
            let layer = &mut self.layer_tools[0];
            let mut original_extruders = layer.extruders.clone();
            layer.extruders.clear();

            // Add extruders in the specified order
            for &ext in first_layer_order {
                if let Some(pos) = original_extruders.iter().position(|&e| e == ext) {
                    layer.extruders.push(ext);
                    original_extruders[pos] = u32::MAX; // Mark as used
                }
            }

            // Add any remaining extruders
            for ext in original_extruders {
                if ext != 0 && ext != u32::MAX {
                    layer.extruders.push(ext);
                }
            }

            // If all extruders were zero, use the first from the order
            if layer.extruders.is_empty() {
                layer.extruders.push(first_layer_order[0]);
            }
        }

        // Process remaining layers
        let mut last_extruder = self.layer_tools[0].extruders.last().copied().unwrap_or(0);

        for i in 1..self.layer_tools.len() {
            let layer = &mut self.layer_tools[i];

            if layer.extruders.is_empty() {
                continue;
            }

            if layer.extruders.len() == 1 && layer.extruders[0] == 0 {
                layer.extruders[0] = last_extruder;
            } else {
                if layer.extruders.first() == Some(&0) {
                    layer.extruders.remove(0);
                }

                if let Some(pos) = layer.extruders.iter().position(|&e| e == last_extruder) {
                    if pos > 0 {
                        let ext = layer.extruders.remove(pos);
                        layer.extruders.insert(0, ext);
                    }
                }
            }

            if let Some(&ext) = layer.extruders.last() {
                last_extruder = ext;
            }
        }
    }

    /// Reorder extruders within each layer to minimize flush volume
    pub fn reorder_extruders_for_minimum_flush(&mut self) {
        self.reorder_extruders_for_minimum_flush_internal(true);
    }

    /// Internal implementation of flush minimization
    fn reorder_extruders_for_minimum_flush_internal(&mut self, reorder_first_layer: bool) {
        if self.layer_tools.is_empty() {
            return;
        }

        // Collect layer filaments for optimization
        let layer_filaments: Vec<Vec<u32>> = self
            .layer_tools
            .iter()
            .map(|lt| lt.extruders.clone())
            .collect();

        // Calculate optimal sequences
        let sequences = self.calculate_optimal_sequences(&layer_filaments, reorder_first_layer);

        // Apply optimized sequences
        for (i, seq) in sequences.into_iter().enumerate() {
            if i < self.layer_tools.len() {
                self.layer_tools[i].extruders = seq;
            }
        }

        self.sorted = true;
    }

    /// Calculate optimal extruder sequences for all layers
    fn calculate_optimal_sequences(
        &self,
        layer_filaments: &[Vec<u32>],
        reorder_first_layer: bool,
    ) -> Vec<Vec<u32>> {
        let mut sequences = Vec::with_capacity(layer_filaments.len());
        let mut last_extruder: Option<u32> = None;

        for (layer_idx, extruders) in layer_filaments.iter().enumerate() {
            if extruders.is_empty() {
                sequences.push(Vec::new());
                continue;
            }

            // Check for custom sequence override
            if let Some(custom_seq) = self.get_custom_sequence_for_layer(layer_idx) {
                // Filter custom sequence to only include extruders actually used
                let filtered: Vec<u32> = custom_seq
                    .iter()
                    .filter_map(|&e| {
                        let e0 = if e > 0 { e as u32 - 1 } else { e as u32 };
                        if extruders.contains(&e0) {
                            Some(e0)
                        } else {
                            None
                        }
                    })
                    .collect();
                if !filtered.is_empty() {
                    if let Some(&last) = filtered.last() {
                        last_extruder = Some(last);
                    }
                    sequences.push(filtered);
                    continue;
                }
            }

            // Skip reordering first layer if requested
            if layer_idx == 0 && !reorder_first_layer {
                if let Some(&last) = extruders.last() {
                    last_extruder = Some(last);
                }
                sequences.push(extruders.clone());
                continue;
            }

            // Find optimal order using greedy algorithm
            let optimized = self.optimize_extruder_order(extruders, last_extruder);
            if let Some(&last) = optimized.last() {
                last_extruder = Some(last);
            }
            sequences.push(optimized);
        }

        sequences
    }

    /// Get custom sequence for a specific layer (if defined)
    fn get_custom_sequence_for_layer(&self, layer_idx: usize) -> Option<&Vec<i32>> {
        let layer_num = layer_idx as i32 + 1; // Convert to 1-based

        // Check first layer sequence
        if layer_idx == 0 && !self.config.first_layer_print_sequence.is_empty() {
            return Some(&self.config.first_layer_print_sequence);
        }

        // Check other layer sequences (reverse order to get most specific match)
        for ((start, end), seq) in self.config.other_layers_print_sequences.iter().rev() {
            if layer_num >= *start && layer_num <= *end {
                return Some(seq);
            }
        }

        None
    }

    /// Optimize extruder order for a single layer using greedy algorithm
    fn optimize_extruder_order(&self, extruders: &[u32], last_extruder: Option<u32>) -> Vec<u32> {
        if extruders.len() <= 1 {
            return extruders.to_vec();
        }

        let mut remaining: HashSet<u32> = extruders.iter().copied().collect();
        let mut result = Vec::with_capacity(extruders.len());

        // Start with the last extruder if it's in this layer, otherwise pick the best starting point
        let first = if let Some(last) = last_extruder {
            if remaining.contains(&last) {
                last
            } else {
                // Pick the one with minimum flush from last
                *remaining
                    .iter()
                    .min_by(|&&a, &&b| {
                        let flush_a = self.config.get_flush_volume(last as usize, a as usize);
                        let flush_b = self.config.get_flush_volume(last as usize, b as usize);
                        flush_a
                            .partial_cmp(&flush_b)
                            .unwrap_or(std::cmp::Ordering::Equal)
                    })
                    .unwrap()
            }
        } else {
        // R744: native iterates `curr_layer_extruders` — a std::vector — with a
        // parallel `is_visited` (ToolOrderUtils.cpp, solve_extruder_order_with_greedy),
        // so BOTH the seed (`std::find_if(is_visited, ==0)`) and the tie-break
        // (strict `<`, first index wins) follow the INPUT ORDER. R99 replaced a
        // nondeterministic `HashSet::iter().next()` with the MIN EXTRUDER ID —
        // deterministic, but a different rule than native's, and the `min_by` over
        // a HashSet below still left TIES on RandomState order.
        // TOOLORDER_INPUT_ORDER=0 restores R99's min-id behaviour.
            if crate::faithful_gate("TOOLORDER_INPUT_ORDER") {
                *extruders.iter().find(|e| remaining.contains(e)).unwrap()
            } else {
                *remaining.iter().min().unwrap()
            }
        };

        result.push(first);
        remaining.remove(&first);

        // Greedily add remaining extruders
        while !remaining.is_empty() {
            let current = *result.last().unwrap();
            // R744: iterate the INPUT order so ties resolve to the first index,
            // as native's indexed loop with strict `<` does.
            let order: Vec<u32> = if crate::faithful_gate("TOOLORDER_INPUT_ORDER") {
                extruders.iter().copied().filter(|e| remaining.contains(e)).collect()
            } else {
                remaining.iter().copied().collect()
            };
            let next = *order
                .iter()
                .min_by(|&&a, &&b| {
                    let flush_a = self.config.get_flush_volume(current as usize, a as usize);
                    let flush_b = self.config.get_flush_volume(current as usize, b as usize);
                    flush_a
                        .partial_cmp(&flush_b)
                        .unwrap_or(std::cmp::Ordering::Equal)
                })
                .unwrap();
            result.push(next);
            remaining.remove(&next);
        }

        result
    }

    /// Collect statistics about all extruders used
    pub fn collect_extruder_statistics(&mut self, prime_multi_material: bool) {
        // Find first printing extruder
        self.first_printing_extruder = None;
        for layer in &self.layer_tools {
            if let Some(&ext) = layer.extruders.first() {
                self.first_printing_extruder = Some(ext);
                break;
            }
        }

        // Find last printing extruder
        self.last_printing_extruder = None;
        for layer in self.layer_tools.iter().rev() {
            if let Some(&ext) = layer.extruders.last() {
                self.last_printing_extruder = Some(ext);
                break;
            }
        }

        // Collect all unique extruders
        let mut all_extruders: HashSet<u32> = HashSet::new();
        for layer in &self.layer_tools {
            all_extruders.extend(&layer.extruders);
        }
        self.all_printing_extruders = all_extruders.into_iter().collect();
        self.all_printing_extruders.sort();

        // For multi-material priming, reorder so first_printing_extruder is last to be primed
        if prime_multi_material && !self.all_printing_extruders.is_empty() {
            if let Some(first) = self.first_printing_extruder {
                self.all_printing_extruders.retain(|&e| e != first);
                self.all_printing_extruders.push(first);
                self.first_printing_extruder = self.all_printing_extruders.first().copied();
            }
        }
    }

    /// Fill wipe tower partition requirements
    pub fn fill_wipe_tower_partitions(&mut self, object_bottom_z: f64) {
        if self.layer_tools.is_empty() {
            return;
        }

        // Count minimum tool changes per layer
        let mut last_extruder: Option<u32> = None;
        for layer in &mut self.layer_tools {
            layer.wipe_tower_partitions = layer.extruders.len();
            if !layer.extruders.is_empty() {
                if last_extruder.is_none() || last_extruder == Some(layer.extruders[0]) {
                    // First extruder matches last, no initial tool change needed
                    layer.wipe_tower_partitions = layer.wipe_tower_partitions.saturating_sub(1);
                }
                last_extruder = layer.extruders.last().copied();
            }
        }

        // Propagate partitions down (lower layers must support upper)
        for i in (0..self.layer_tools.len().saturating_sub(1)).rev() {
            let next_partitions = self.layer_tools[i + 1].wipe_tower_partitions;
            self.layer_tools[i].wipe_tower_partitions = self.layer_tools[i]
                .wipe_tower_partitions
                .max(next_partitions);
        }

        // Mark layers that need wipe tower
        let wrapping_layers = self.config.wrapping_detection_layers;
        for (i, layer) in self.layer_tools.iter_mut().enumerate() {
            if i < wrapping_layers {
                layer.has_wipe_tower = self.config.enable_wrapping_detection;
            }

            layer.has_wipe_tower |= (layer.has_object
                && (self.config.timelapse_smooth || layer.wipe_tower_partitions > 0))
                || layer.print_z < object_bottom_z + LAYER_HEIGHT_EPSILON;
        }

        // Calculate wipe tower layer heights
        let mut wipe_tower_print_z_last = 0.0;
        for layer in &mut self.layer_tools {
            if layer.has_wipe_tower {
                layer.wipe_tower_layer_height = layer.print_z - wipe_tower_print_z_last;
                wipe_tower_print_z_last = layer.print_z;
            }
        }
    }

    /// Mark layers that should have skirt printed
    pub fn mark_skirt_layers(&mut self) {
        if self.layer_tools.is_empty() {
            return;
        }

        if self.layer_tools[0].extruders.is_empty() {
            return;
        }

        let max_layer_height = self.config.max_layer_height;
        let mut i = 0;

        loop {
            self.layer_tools[i].has_skirt = true;

            // Find next layer with object
            let mut j = i + 1;
            while j < self.layer_tools.len() && !self.layer_tools[j].has_object {
                j += 1;
            }

            if j >= self.layer_tools.len() {
                break;
            }

            // Mark intermediate layers for skirt if needed
            let mut last_z = self.layer_tools[i].print_z;
            for k in (i + 1)..j {
                if self.layer_tools[k + 1].print_z - last_z
                    > max_layer_height + LAYER_HEIGHT_EPSILON
                {
                    // Find last non-empty layer before k+1
                    let mut mark_idx = k;
                    while self.layer_tools[mark_idx].extruders.is_empty() && mark_idx > i {
                        mark_idx -= 1;
                    }

                    if !self.layer_tools[mark_idx].has_skirt {
                        self.layer_tools[mark_idx].has_skirt = true;
                        last_z = self.layer_tools[mark_idx].print_z;
                    }
                }
            }

            i = j;
        }
    }

    /// Assign custom G-codes to appropriate layers
    pub fn assign_custom_gcodes(&mut self, custom_gcodes: &[CustomGCodeItem]) {
        if custom_gcodes.is_empty() || self.layer_tools.is_empty() {
            return;
        }

        // Track which extruders print above each layer
        let num_filaments = self.config.num_filaments;
        let mut extruder_printing_above: Vec<Vec<bool>> = Vec::new();
        let mut current_above = vec![false; num_filaments];

        for layer in self.layer_tools.iter().rev() {
            for &ext in &layer.extruders {
                if (ext as usize) < num_filaments {
                    current_above[ext as usize] = true;
                }
            }
            extruder_printing_above.push(current_above.clone());
        }
        extruder_printing_above.reverse();

        // Assign each custom G-code to the closest layer
        for gcode in custom_gcodes {
            if gcode.gcode_type == CustomGCodeType::ToolChange {
                continue; // Tool changes are handled separately
            }

            // Find the closest layer
            let mut best_idx = 0;
            let mut best_dist = f64::MAX;
            for (i, layer) in self.layer_tools.iter().enumerate() {
                let dist = (layer.print_z - gcode.print_z).abs();
                if dist < best_dist {
                    best_dist = dist;
                    best_idx = i;
                }
            }

            // Check if this G-code should be applied
            let should_apply = match gcode.gcode_type {
                CustomGCodeType::ColorChange => {
                    // Only apply if the extruder will print above this layer
                    let ext_idx = (gcode.extruder - 1) as usize;
                    ext_idx < num_filaments
                        && extruder_printing_above
                            .get(best_idx)
                            .map(|v| v.get(ext_idx).copied().unwrap_or(false))
                            .unwrap_or(false)
                }
                _ => true,
            };

            if should_apply {
                self.layer_tools[best_idx].custom_gcode = Some(gcode.clone());
            }
        }
    }

    /// Generate the first layer tool order based on object geometry
    pub fn generate_first_layer_tool_order(
        &self,
        layer_regions: &[(u32, Vec<ExPolygon>)], // (extruder_id, slices)
        initial_layer_line_width: f64,
    ) -> Vec<u32> {
        let mut min_areas_per_extruder: HashMap<u32, f64> = HashMap::new();

        // Calculate minimum contour area per extruder
        for (extruder_id, slices) in layer_regions {
            for expoly in slices {
                // Check if slice is large enough after shrinking
                let shrink_amount = -0.2 * initial_layer_line_width; // Negative for shrinking
                let shrunk = offset_expolygon(expoly, shrink_amount, OffsetJoinType::Miter);
                if !shrunk.is_empty() {
                    let area = expoly.contour.area().abs();
                    let entry = min_areas_per_extruder.entry(*extruder_id).or_insert(area);
                    if area < *entry {
                        *entry = area;
                    }
                }
            }
        }

        // Sort extruders by their minimum area (largest first for better adhesion)
        let mut tool_order: Vec<(u32, f64)> = min_areas_per_extruder.into_iter().collect();
        tool_order.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));

        let mut result: Vec<u32> = tool_order.into_iter().map(|(id, _)| id).collect();

        // Apply user-defined first layer sequence if specified
        if !self.config.first_layer_print_sequence.is_empty()
            && self.config.first_layer_print_sequence.len() >= result.len()
        {
            let seq = &self.config.first_layer_print_sequence;
            result.sort_by(|a, b| {
                let pos_a = seq.iter().position(|&x| x == *a as i32 + 1);
                let pos_b = seq.iter().position(|&x| x == *b as i32 + 1);
                match (pos_a, pos_b) {
                    (Some(pa), Some(pb)) => pa.cmp(&pb),
                    (Some(_), None) => std::cmp::Ordering::Less,
                    (None, Some(_)) => std::cmp::Ordering::Greater,
                    (None, None) => std::cmp::Ordering::Equal,
                }
            });
        }

        result
    }

    /// Calculate the most frequently used extruder
    pub fn calculate_most_used_extruder(&mut self) {
        let mut extruder_counts: HashMap<u32, usize> = HashMap::new();

        for layer in &self.layer_tools {
            let unique_extruders: HashSet<u32> = layer.extruders.iter().copied().collect();
            for ext in unique_extruders {
                *extruder_counts.entry(ext).or_insert(0) += 1;
            }
        }

        self.most_used_extruder = extruder_counts
            .into_iter()
            .max_by_key(|(_, count)| *count)
            .map(|(ext, _)| ext)
            .unwrap_or(0);
    }

    /// Calculate filament change statistics
    pub fn calculate_filament_change_stats(&mut self) {
        self.stats_single_ext = self.calculate_stats_for_sequence(&self.layer_tools);
    }

    /// Calculate statistics for a given sequence
    fn calculate_stats_for_sequence(&self, layers: &[LayerTools]) -> FilamentChangeStats {
        let mut stats = FilamentChangeStats::new();
        let mut last_filament: Option<u32> = None;

        for layer in layers {
            for &filament in &layer.extruders {
                if let Some(last) = last_filament {
                    if last != filament {
                        stats.filament_change_count += 1;
                        let flush_volume = self
                            .config
                            .get_flush_volume(last as usize, filament as usize);
                        let density = self
                            .config
                            .filament_densities
                            .get(filament as usize)
                            .copied()
                            .unwrap_or(1.24);
                        // Convert mm³ to grams: volume * density / 1000
                        stats.filament_flush_weight +=
                            (flush_volume * density as f32 / 1000.0) as i32;
                    }
                }
                last_filament = Some(filament);
            }
        }

        stats
    }

    /// Get filament change statistics for a given mode
    pub fn get_filament_change_stats(&self, mode: FilamentChangeMode) -> &FilamentChangeStats {
        match mode {
            FilamentChangeMode::SingleExt => &self.stats_single_ext,
            FilamentChangeMode::MultiExtBest => &self.stats_multi_ext_best,
            FilamentChangeMode::MultiExtCurr => &self.stats_multi_ext_curr,
        }
    }

    // ========================================================================
    // Accessors
    // ========================================================================

    /// Get the first printing extruder
    pub fn first_extruder(&self) -> Option<u32> {
        self.first_printing_extruder
    }

    /// Get the last printing extruder
    pub fn last_extruder(&self) -> Option<u32> {
        self.last_printing_extruder
    }

    /// Get all printing extruders
    pub fn all_extruders(&self) -> &[u32] {
        &self.all_printing_extruders
    }

    /// Get layer tools by index
    pub fn get_layer_tools(&self, index: usize) -> Option<&LayerTools> {
        self.layer_tools.get(index)
    }

    /// Get mutable layer tools by index
    pub fn get_layer_tools_mut(&mut self, index: usize) -> Option<&mut LayerTools> {
        self.layer_tools.get_mut(index)
    }

    /// Get all layer tools
    pub fn layer_tools(&self) -> &[LayerTools] {
        &self.layer_tools
    }

    /// Get mutable reference to all layer tools
    pub fn layer_tools_mut(&mut self) -> &mut Vec<LayerTools> {
        &mut self.layer_tools
    }

    /// Check if there's a wipe tower in the print
    pub fn has_wipe_tower(&self) -> bool {
        !self.layer_tools.is_empty()
            && self.first_printing_extruder.is_some()
            && self.layer_tools[0].has_wipe_tower
    }

    /// Get the most frequently used extruder
    pub fn get_most_used_extruder(&self) -> u32 {
        self.most_used_extruder
    }

    /// Check if extruders have been sorted/reordered
    pub fn is_sorted(&self) -> bool {
        self.sorted
    }

    /// Get number of layers
    pub fn len(&self) -> usize {
        self.layer_tools.len()
    }

    /// Check if there are no layers
    pub fn is_empty(&self) -> bool {
        self.layer_tools.is_empty()
    }

    /// Get the front (first) layer tools
    pub fn front(&self) -> Option<&LayerTools> {
        self.layer_tools.first()
    }

    /// Get the back (last) layer tools
    pub fn back(&self) -> Option<&LayerTools> {
        self.layer_tools.last()
    }

    /// Clear all data
    pub fn clear(&mut self) {
        self.layer_tools.clear();
        self.first_printing_extruder = None;
        self.last_printing_extruder = None;
        self.all_printing_extruders.clear();
        self.stats_single_ext.clear();
        self.stats_multi_ext_best.clear();
        self.stats_multi_ext_curr.clear();
        self.most_used_extruder = 0;
        self.sorted = false;
    }

    /// Iterator over layer tools
    pub fn iter(&self) -> impl Iterator<Item = &LayerTools> {
        self.layer_tools.iter()
    }

    /// Mutable iterator over layer tools
    pub fn iter_mut(&mut self) -> impl Iterator<Item = &mut LayerTools> {
        self.layer_tools.iter_mut()
    }
}

impl IntoIterator for ToolOrdering {
    type Item = LayerTools;
    type IntoIter = std::vec::IntoIter<LayerTools>;

    fn into_iter(self) -> Self::IntoIter {
        self.layer_tools.into_iter()
    }
}

impl<'a> IntoIterator for &'a ToolOrdering {
    type Item = &'a LayerTools;
    type IntoIter = std::slice::Iter<'a, LayerTools>;

    fn into_iter(self) -> Self::IntoIter {
        self.layer_tools.iter()
    }
}

// ============================================================================
// Helper Functions
// ============================================================================

/// Calculate total flush volume for a sequence of tool changes
pub fn calculate_flush_volume(sequence: &[u32], flush_matrix: &FlushMatrix) -> f32 {
    if sequence.len() < 2 {
        return 0.0;
    }
    sequence
        .windows(2)
        .map(|w| flush_matrix.get(w[0] as usize, w[1] as usize))
        .sum()
}

/// Find the optimal ordering of extruders to minimize total flush volume
/// Uses a simple greedy algorithm (nearest neighbor heuristic)
pub fn optimize_extruder_sequence(
    extruders: &[u32],
    start_extruder: Option<u32>,
    flush_matrix: &FlushMatrix,
) -> Vec<u32> {
    if extruders.len() <= 1 {
        return extruders.to_vec();
    }

    let mut remaining: HashSet<u32> = extruders.iter().copied().collect();
    let mut result = Vec::with_capacity(extruders.len());

    // Choose starting extruder.
    // R99 determinism: min extruder id (HashSet<u32>.iter().next() is
    // RandomState-ordered per run).
    let start = start_extruder
        .filter(|e| remaining.contains(e))
        .or_else(|| {
            if crate::faithful_gate("TOOLORDER_INPUT_ORDER") {
                extruders.iter().find(|e| remaining.contains(e)).copied()
            } else {
                remaining.iter().min().copied()
            }
        })
        .unwrap();

    result.push(start);
    remaining.remove(&start);

    // Greedy nearest neighbor
    while !remaining.is_empty() {
        let current = *result.last().unwrap();
        let order: Vec<u32> = if crate::faithful_gate("TOOLORDER_INPUT_ORDER") {
            extruders.iter().copied().filter(|e| remaining.contains(e)).collect()
        } else {
            remaining.iter().copied().collect()
        };
        let next = order
            .iter()
            .min_by(|&&a, &&b| {
                let flush_a = flush_matrix.get(current as usize, a as usize);
                let flush_b = flush_matrix.get(current as usize, b as usize);
                flush_a
                    .partial_cmp(&flush_b)
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
            .copied()
            .unwrap();
        result.push(next);
        remaining.remove(&next);
    }

    result
}

/// Generate all possible orderings of a set of extruders
/// Warning: This has O(n!) complexity, only use for small sets
pub fn generate_all_orderings(extruders: &[u32]) -> Vec<Vec<u32>> {
    if extruders.is_empty() {
        return vec![Vec::new()];
    }
    if extruders.len() == 1 {
        return vec![extruders.to_vec()];
    }

    let mut result = Vec::new();
    for (i, &first) in extruders.iter().enumerate() {
        let mut rest: Vec<u32> = extruders.to_vec();
        rest.remove(i);
        for mut perm in generate_all_orderings(&rest) {
            let mut ordering = vec![first];
            ordering.append(&mut perm);
            result.push(ordering);
        }
    }
    result
}

/// Find the optimal ordering by exhaustive search (brute force)
/// Only practical for small numbers of extruders (≤8)
pub fn find_optimal_ordering_exhaustive(
    extruders: &[u32],
    start_extruder: Option<u32>,
    flush_matrix: &FlushMatrix,
) -> Vec<u32> {
    if extruders.len() <= 1 {
        return extruders.to_vec();
    }
    if extruders.len() > 8 {
        // Fall back to greedy for large sets
        return optimize_extruder_sequence(extruders, start_extruder, flush_matrix);
    }

    let orderings = generate_all_orderings(extruders);
    let mut best_ordering = extruders.to_vec();
    let mut best_cost = f32::MAX;

    for ordering in orderings {
        // If we have a start extruder constraint, skip orderings that don't start with it
        if let Some(start) = start_extruder {
            if ordering[0] != start && !extruders.contains(&start) {
                // start_extruder not in this layer, calculate flush from start to first
                let cost = flush_matrix.get(start as usize, ordering[0] as usize)
                    + calculate_flush_volume(&ordering, flush_matrix);
                if cost < best_cost {
                    best_cost = cost;
                    best_ordering = ordering;
                }
            } else if ordering[0] == start {
                let cost = calculate_flush_volume(&ordering, flush_matrix);
                if cost < best_cost {
                    best_cost = cost;
                    best_ordering = ordering;
                }
            }
        } else {
            let cost = calculate_flush_volume(&ordering, flush_matrix);
            if cost < best_cost {
                best_cost = cost;
                best_ordering = ordering;
            }
        }
    }

    best_ordering
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_filament_change_stats() {
        let mut stats1 = FilamentChangeStats {
            filament_flush_weight: 10,
            filament_change_count: 5,
            extruder_change_count: 2,
        };

        let stats2 = FilamentChangeStats {
            filament_flush_weight: 5,
            filament_change_count: 3,
            extruder_change_count: 1,
        };

        stats1 += stats2.clone();
        assert_eq!(stats1.filament_flush_weight, 15);
        assert_eq!(stats1.filament_change_count, 8);
        assert_eq!(stats1.extruder_change_count, 3);

        stats1.clear();
        assert_eq!(stats1.filament_flush_weight, 0);
        assert_eq!(stats1.filament_change_count, 0);
        assert_eq!(stats1.extruder_change_count, 0);
    }

    #[test]
    fn test_flush_matrix() {
        let mut matrix = FlushMatrix::new(4, 100.0);
        assert_eq!(matrix.get(0, 1), 100.0);
        assert_eq!(matrix.get(0, 0), 0.0); // Diagonal is zero

        matrix.set(0, 1, 150.0);
        assert_eq!(matrix.get(0, 1), 150.0);

        matrix.apply_multiplier(2.0);
        assert_eq!(matrix.get(0, 1), 300.0);
        assert_eq!(matrix.get(0, 0), 0.0); // Diagonal stays zero
    }

    #[test]
    fn test_flush_matrix_from_flat() {
        let values = vec![0.0, 100.0, 150.0, 100.0, 0.0, 120.0, 150.0, 120.0, 0.0];
        let matrix = FlushMatrix::from_flat(3, &values);
        assert_eq!(matrix.get(0, 1), 100.0);
        assert_eq!(matrix.get(1, 2), 120.0);
        assert_eq!(matrix.get(0, 2), 150.0);
    }

    #[test]
    fn test_layer_tools() {
        let mut layer = LayerTools::new(0.2);
        layer.extruders = vec![0, 1, 2];
        layer.has_object = true;

        assert!(layer.has_extruder(0));
        assert!(layer.has_extruder(1));
        assert!(!layer.has_extruder(3));

        assert!(layer.is_extruder_order(0, 1));
        assert!(layer.is_extruder_order(0, 2));
        assert!(layer.is_extruder_order(1, 2));
        assert!(!layer.is_extruder_order(2, 0));
        assert!(!layer.is_extruder_order(1, 1)); // Same extruder
    }

    #[test]
    fn test_tool_ordering_config() {
        let config = ToolOrderingConfig::new(4);
        assert_eq!(config.num_filaments, 4);
        assert_eq!(config.nozzle_diameters.len(), 4);
        assert_eq!(config.filament_map, vec![1, 2, 3, 4]);
    }

    #[test]
    fn test_tool_ordering_initialize_layers() {
        let config = ToolOrderingConfig::new(2);
        let mut ordering = ToolOrdering::new(config);

        ordering.initialize_layers(vec![0.2, 0.4, 0.6, 0.4, 0.2]); // With duplicates
        assert_eq!(ordering.len(), 3);
        assert!((ordering.layer_tools()[0].print_z - 0.2).abs() < 0.001);
        assert!((ordering.layer_tools()[1].print_z - 0.4).abs() < 0.001);
        assert!((ordering.layer_tools()[2].print_z - 0.6).abs() < 0.001);
    }

    #[test]
    fn test_optimize_extruder_sequence() {
        let mut matrix = FlushMatrix::new(4, 100.0);
        // Make transition 0->1 cheap
        matrix.set(0, 1, 10.0);
        matrix.set(1, 0, 10.0);
        // Make transition 1->2 cheap
        matrix.set(1, 2, 20.0);
        matrix.set(2, 1, 20.0);
        // Make transition 2->3 cheap
        matrix.set(2, 3, 30.0);
        matrix.set(3, 2, 30.0);

        let extruders = vec![0, 1, 2, 3];
        let result = optimize_extruder_sequence(&extruders, Some(0), &matrix);

        // Should find the optimal path: 0 -> 1 -> 2 -> 3
        assert_eq!(result, vec![0, 1, 2, 3]);
    }

    #[test]
    fn test_calculate_flush_volume() {
        let matrix = FlushMatrix::new(4, 100.0);
        let sequence = vec![0, 1, 2, 3];
        let flush = calculate_flush_volume(&sequence, &matrix);
        assert!((flush - 300.0).abs() < 0.001); // 3 transitions * 100 each
    }

    #[test]
    fn test_generate_all_orderings() {
        let extruders = vec![0, 1, 2];
        let orderings = generate_all_orderings(&extruders);
        assert_eq!(orderings.len(), 6); // 3! = 6
    }

    #[test]
    fn test_wiping_extrusions() {
        let mut wiping = WipingExtrusions::new();
        assert!(!wiping.is_anything_overridden());

        wiping.set_extruder_override(1, 1, 0, 2, 3);
        assert!(wiping.is_anything_overridden());
        assert!(wiping.is_entity_overridden(1, 1, 0));
        assert!(!wiping.is_entity_overridden(1, 1, 1));

        wiping.set_support_extruder_override(1, 3);
        assert!(wiping.is_support_overridden(1));
        assert!(!wiping.is_support_overridden(2));
    }

    #[test]
    fn test_custom_gcode_item() {
        let color_change = CustomGCodeItem::color_change(10.0, 1, "#FF0000");
        assert_eq!(color_change.gcode_type, CustomGCodeType::ColorChange);
        assert_eq!(color_change.extruder, 1);
        assert_eq!(color_change.color, "#FF0000");

        let pause = CustomGCodeItem::pause(20.0);
        assert_eq!(pause.gcode_type, CustomGCodeType::Pause);
    }

    #[test]
    fn test_handle_dontcare_extruders() {
        let config = ToolOrderingConfig::new(3);
        let mut ordering = ToolOrdering::new(config);

        ordering.initialize_layers(vec![0.2, 0.4, 0.6]);
        ordering.layer_tools_mut()[0].extruders = vec![0, 1]; // Don't care (0) and extruder 1
        ordering.layer_tools_mut()[1].extruders = vec![2];
        ordering.layer_tools_mut()[2].extruders = vec![0]; // Don't care

        ordering.handle_dontcare_extruders(Some(1));

        // First layer should start with 1 (specified), 0 removed
        assert_eq!(ordering.layer_tools()[0].extruders, vec![1]);
        // Second layer unchanged
        assert_eq!(ordering.layer_tools()[1].extruders, vec![2]);
        // Third layer should use last extruder (2)
        assert_eq!(ordering.layer_tools()[2].extruders, vec![2]);
    }

    #[test]
    fn test_collect_extruder_statistics() {
        let config = ToolOrderingConfig::new(4);
        let mut ordering = ToolOrdering::new(config);

        ordering.initialize_layers(vec![0.2, 0.4, 0.6]);
        ordering.layer_tools_mut()[0].extruders = vec![0, 1];
        ordering.layer_tools_mut()[1].extruders = vec![1, 2];
        ordering.layer_tools_mut()[2].extruders = vec![2, 3];

        ordering.collect_extruder_statistics(false);

        assert_eq!(ordering.first_extruder(), Some(0));
        assert_eq!(ordering.last_extruder(), Some(3));

        let all = ordering.all_extruders();
        assert!(all.contains(&0));
        assert!(all.contains(&1));
        assert!(all.contains(&2));
        assert!(all.contains(&3));
    }

    #[test]
    fn test_extrusion_role_type() {
        assert!(ExtrusionRoleType::Perimeter.is_perimeter());
        assert!(ExtrusionRoleType::ExternalPerimeter.is_perimeter());
        assert!(!ExtrusionRoleType::InternalInfill.is_perimeter());

        assert!(ExtrusionRoleType::SolidInfill.is_solid_infill());
        assert!(ExtrusionRoleType::TopSolidInfill.is_solid_infill());
        assert!(!ExtrusionRoleType::InternalInfill.is_solid_infill());

        assert!(ExtrusionRoleType::InternalInfill.is_infill());
        assert!(ExtrusionRoleType::SolidInfill.is_infill());
        assert!(!ExtrusionRoleType::Perimeter.is_infill());

        assert!(ExtrusionRoleType::SupportMaterial.is_support());
        assert!(!ExtrusionRoleType::Perimeter.is_support());
    }

    #[test]
    fn test_layer_tools_wall_filament() {
        let mut layer = LayerTools::new(0.2);

        // No override
        assert_eq!(layer.wall_filament(1), 0); // 1-based to 0-based
        assert_eq!(layer.wall_filament(2), 1);

        // With override
        layer.extruder_override = 3;
        assert_eq!(layer.wall_filament(1), 2); // Override wins
    }

    #[test]
    fn test_tool_ordering_iteration() {
        let config = ToolOrderingConfig::new(2);
        let mut ordering = ToolOrdering::new(config);

        ordering.initialize_layers(vec![0.2, 0.4]);

        // Test iter
        let count = ordering.iter().count();
        assert_eq!(count, 2);

        // Test into_iter
        let layers: Vec<LayerTools> = ordering.into_iter().collect();
        assert_eq!(layers.len(), 2);
    }

    #[test]
    fn test_flush_matrix_sequence() {
        let matrix = FlushMatrix::new(3, 50.0);
        let sequence = vec![0, 1, 2, 0];
        let total = matrix.total_flush_for_sequence(&sequence);
        assert!((total - 150.0).abs() < 0.001); // 3 transitions
    }

    #[test]
    fn test_find_optimal_ordering_exhaustive() {
        let mut matrix = FlushMatrix::new(3, 100.0);
        matrix.set(0, 1, 10.0);
        matrix.set(1, 2, 10.0);
        matrix.set(0, 2, 200.0);

        let extruders = vec![0, 1, 2];
        let result = find_optimal_ordering_exhaustive(&extruders, Some(0), &matrix);

        // Optimal is 0 -> 1 -> 2 (cost 20) not 0 -> 2 -> 1 (cost 200 + something)
        assert_eq!(result, vec![0, 1, 2]);
    }

    #[test]
    fn test_mark_skirt_layers() {
        let config = ToolOrderingConfig::new(2);
        let mut ordering = ToolOrdering::new(config);

        ordering.initialize_layers(vec![0.2, 0.4, 0.6, 0.8]);
        ordering.layer_tools_mut()[0].extruders = vec![0];
        ordering.layer_tools_mut()[0].has_object = true;
        ordering.layer_tools_mut()[1].has_object = false;
        ordering.layer_tools_mut()[2].extruders = vec![0];
        ordering.layer_tools_mut()[2].has_object = true;
        ordering.layer_tools_mut()[3].extruders = vec![0];
        ordering.layer_tools_mut()[3].has_object = true;

        ordering.mark_skirt_layers();

        assert!(ordering.layer_tools()[0].has_skirt);
        assert!(ordering.layer_tools()[2].has_skirt);
    }

    #[test]
    fn test_fill_wipe_tower_partitions() {
        let mut config = ToolOrderingConfig::new(3);
        config.enable_prime_tower = true;
        let mut ordering = ToolOrdering::new(config);

        ordering.initialize_layers(vec![0.2, 0.4, 0.6]);
        ordering.layer_tools_mut()[0].extruders = vec![0, 1];
        ordering.layer_tools_mut()[0].has_object = true;
        ordering.layer_tools_mut()[1].extruders = vec![1, 2];
        ordering.layer_tools_mut()[1].has_object = true;
        ordering.layer_tools_mut()[2].extruders = vec![2];
        ordering.layer_tools_mut()[2].has_object = true;

        ordering.fill_wipe_tower_partitions(0.0);

        // Layer 0: 2 extruders, first layer so one tool change = 1 partition
        assert_eq!(ordering.layer_tools()[0].wipe_tower_partitions, 1);
        // Layer 1: 2 extruders, but starts with 1 (last from layer 0), one tool change = 1 partition
        // But propagation from above might increase this
        assert!(ordering.layer_tools()[1].wipe_tower_partitions >= 1);
        // Layer 2: 1 extruder, 0 tool changes if starts with 2
        // Partitions propagated from above
    }
}
