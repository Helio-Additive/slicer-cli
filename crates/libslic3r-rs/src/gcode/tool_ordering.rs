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

use std::collections::{BTreeMap, BTreeSet, HashMap, HashSet};

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
    /// ToolOrdering.hpp:114 `int flush_filament_change_count {0}`
    pub flush_filament_change_count: i32,
}

impl FilamentChangeStats {
    pub fn new() -> Self {
        Self::default()
    }

    /// ToolOrdering.hpp:117-121
    pub fn clear(&mut self) {
        self.filament_flush_weight = 0;
        self.filament_change_count = 0;
        self.flush_filament_change_count = 0;
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
            flush_filament_change_count: self.flush_filament_change_count + other.flush_filament_change_count,
        }
    }
}

impl std::ops::AddAssign for FilamentChangeStats {
    fn add_assign(&mut self, other: Self) {
        self.filament_flush_weight += other.filament_flush_weight;
        self.filament_change_count += other.filament_change_count;
        self.extruder_change_count += other.extruder_change_count;
        self.flush_filament_change_count += other.flush_filament_change_count;
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
// WipingExtrusions::mark_wiping_extrusions / ensure_perimeters_infills_order
// (ToolOrdering.cpp:3906-4117)
//
// The C++ walks `print.objects()`, their layer at `lt.print_z`, its regions and
// their fill and perimeter collections. The structs below are that view of one
// layer: what the two functions read, and nothing else.
// ============================================================================

/// One `ExtrusionEntityCollection` of `layerm->fills.entities` or
/// `layerm->perimeters.entities`.
#[derive(Debug, Clone)]
pub struct WipingEntity {
    /// Stands for the entity pointer in `entity_map`.
    pub id: u64,
    /// `eec.role()`
    pub role: crate::extrusion_entity::ExtrusionRole,
    /// `eec.total_volume()`
    pub total_volume: f64,
    /// `m_layer_tools->extruder(eec, region)`, zero based.
    pub extruder: u32,
}

/// One `LayerRegion` of the object's layer.
#[derive(Debug, Clone, Default)]
pub struct WipingRegion {
    /// `lt.wall_filament(region)`, zero based.
    pub wall_filament: u32,
    /// `lt.sparse_infill_filament(region)`, zero based.
    pub sparse_infill_filament: u32,
    pub fills: Vec<WipingEntity>,
    pub perimeters: Vec<WipingEntity>,
}

/// The support layer of the object at this print_z.
#[derive(Debug, Clone, Default)]
pub struct WipingSupportLayer {
    pub planned_filament_id: i32,
    pub planned_interface_filament_id: i32,
    /// (role, total_volume) of `support_fills.entities`.
    pub entities: Vec<(crate::extrusion_entity::ExtrusionRole, f64)>,
}

/// One `PrintObject` with its layer at `lt.print_z`.
#[derive(Debug, Clone, Default)]
pub struct WipingObject {
    /// `object->id()`, also the object half of the `entity_map` key.
    pub id: u64,
    /// `object->instances().size()`
    pub num_of_copies: usize,
    pub flush_into_objects: bool,
    pub flush_into_infill: bool,
    pub flush_into_support: bool,
    pub support_filament: i32,
    pub support_interface_filament: i32,
    pub support_interface_not_for_body: bool,
    /// `None` when the object has no layer at this print_z.
    pub regions: Option<Vec<WipingRegion>>,
    pub support_layer: Option<WipingSupportLayer>,
}

/// What the two functions read of `Print` and of `*m_layer_tools`.
pub struct WipingLayer<'a> {
    /// `print.objects()`, in that order.
    pub objects: &'a [WipingObject],
    /// `lt.extruders`
    pub extruders: &'a [u32],
    pub filament_soluble: &'a dyn Fn(u32) -> bool,
    pub filament_is_support: &'a dyn Fn(u32) -> bool,
    /// `print.config().is_infill_first`
    pub is_infill_first: bool,
}

impl WipingLayer<'_> {
    /// `LayerTools::is_extruder_order` (ToolOrdering.cpp:88-101)
    fn is_extruder_order(&self, a: u32, b: u32) -> bool {
        if a == b {
            return false;
        }
        for &extruder in self.extruders {
            if extruder == a {
                return true;
            }
            if extruder == b {
                return false;
            }
        }
        false
    }

    /// `LayerTools::has_extruder`
    fn has_extruder(&self, extruder: u32) -> bool {
        self.extruders.contains(&extruder)
    }
}

impl WipingExtrusions {
    /// `WipingExtrusions::is_overriddable` (ToolOrdering.cpp:3861-3872)
    fn is_overriddable_entity(lt: &WipingLayer, eec: &WipingEntity, object: &WipingObject) -> bool {
        if (lt.filament_soluble)(eec.extruder) {
            return false;
        }
        if object.flush_into_objects {
            return true;
        }
        if !object.flush_into_infill || eec.role != crate::extrusion_entity::ExtrusionRole::InternalInfill {
            return false;
        }
        true
    }

    /// `WipingExtrusions::first_nonsoluble_extruder_on_layer` (ToolOrdering.cpp:3839-3848)
    fn first_nonsoluble_extruder_on_layer(lt: &WipingLayer) -> i32 {
        for &e in lt.extruders.iter() {
            if !(lt.filament_soluble)(e) && !(lt.filament_is_support)(e) {
                return e as i32;
            }
        }
        -1
    }

    /// `WipingExtrusions::last_nonsoluble_extruder_on_layer` (ToolOrdering.cpp:3850-3859)
    fn last_nonsoluble_extruder_on_layer(lt: &WipingLayer) -> i32 {
        for &e in lt.extruders.iter().rev() {
            if !(lt.filament_soluble)(e) && !(lt.filament_is_support)(e) {
                return e as i32;
            }
        }
        -1
    }

    /// `WipingExtrusions::mark_wiping_extrusions` (ToolOrdering.cpp:3906-4047).
    ///
    /// Iterates through all extrusions on the layer, remembers those that could
    /// be used for wiping after the tool change and returns the volume that is
    /// left to be wiped on the wipe tower.
    pub fn mark_wiping_extrusions(
        &mut self,
        lt: &WipingLayer,
        old_extruder: u32,
        new_extruder: u32,
        mut volume_to_wipe: f32,
    ) -> f32 {
        let min_infill_volume: f32 = 0.0; // ignore infill with smaller volume than this

        if !self.something_overridable
            || volume_to_wipe <= 0.0
            || (lt.filament_soluble)(old_extruder)
            || (lt.filament_soluble)(new_extruder)
        {
            // Soluble filament cannot be wiped in a random infill, neither the filament after it
            return volume_to_wipe.max(0.0);
        }
        // BBS
        if (lt.filament_is_support)(old_extruder) || (lt.filament_is_support)(new_extruder) {
            return volume_to_wipe.max(0.0);
        }

        // we will sort objects so that dedicated for wiping are at the beginning:
        let mut object_list: Vec<&WipingObject> = lt.objects.iter().collect();
        object_list.sort_by(|a, b| {
            if a.flush_into_objects != b.flush_into_objects {
                // the one that flushes into objects comes first
                b.flush_into_objects.cmp(&a.flush_into_objects)
            } else {
                a.id.cmp(&b.id)
            }
        });

        // We will now iterate through
        //  - first the dedicated objects to mark perimeters or infills (depending on infill_first)
        //  - second through the dedicated ones again to mark infills or perimeters (depending on infill_first)
        //  - then all the others to mark infills (in case that !infill_first, we must also check that the
        //    perimeter is finished already
        // this is controlled by the following variable:
        let mut perimeters_done = false;

        let mut i: i32 = 0;
        while i < object_list.len() as i32 + if perimeters_done { 0 } else { 1 } {
            if !perimeters_done
                && (i == object_list.len() as i32 || !object_list[i as usize].flush_into_objects)
            {
                // we passed the last dedicated object in list
                perimeters_done = true;
                i = 0; // let's go from the start again (`i = -1; continue;` and the loop's `++i`)
                continue;
            }

            let object = object_list[i as usize];
            i += 1;

            // Finds this layer:
            let Some(regions) = object.regions.as_ref() else { continue };

            let num_of_copies = object.num_of_copies;

            // iterate through copies (aka PrintObject instances) first, so that we mark neighbouring
            // infills to minimize travel moves
            for copy in 0..num_of_copies {
                for layerm in regions {
                    if !object.flush_into_infill && !object.flush_into_objects && !object.flush_into_support {
                        continue;
                    }
                    let wipe_into_infill_only = !object.flush_into_objects && object.flush_into_infill;
                    let is_infill_first = lt.is_infill_first;
                    if is_infill_first != perimeters_done || wipe_into_infill_only {
                        // iterate through all infill Collections
                        for fill in &layerm.fills {
                            if !Self::is_overriddable_entity(lt, fill, object) {
                                continue;
                            }

                            if wipe_into_infill_only && !is_infill_first {
                                // In this case we must check that the original extruder is used on this
                                // layer before the one we are overridding (and the perimeters will be
                                // finished before the infill is printed):
                                if !lt.is_extruder_order(layerm.wall_filament, new_extruder) {
                                    continue;
                                }
                            }

                            if !self.is_entity_overridden(fill.id, object.id, copy)
                                && fill.total_volume > min_infill_volume as f64
                            {
                                // this infill will be used to wipe this extruder
                                self.set_extruder_override(fill.id, object.id, copy, new_extruder as i32, num_of_copies);
                                volume_to_wipe -= fill.total_volume as f32;
                                if volume_to_wipe <= 0.0 {
                                    // More material was purged already than asked for.
                                    return 0.0;
                                }
                            }
                        }
                    }

                    // Now the same for perimeters - see comments above for explanation:
                    if object.flush_into_objects && is_infill_first == perimeters_done {
                        for fill in &layerm.perimeters {
                            if Self::is_overriddable_entity(lt, fill, object)
                                && !self.is_entity_overridden(fill.id, object.id, copy)
                                && fill.total_volume > min_infill_volume as f64
                            {
                                self.set_extruder_override(fill.id, object.id, copy, new_extruder as i32, num_of_copies);
                                volume_to_wipe -= fill.total_volume as f32;
                                if volume_to_wipe <= 0.0 {
                                    return 0.0;
                                }
                            }
                        }
                    }
                }

                // BBS
                if object.flush_into_support {
                    // `do { … } while (0)` with its `break`s
                    'support: {
                        let Some(this_support_layer) = object.support_layer.as_ref() else { break 'support };

                        // BBS multi-diameter: a support slab thicker than the object layers is pinned to
                        // the filament it was planned for, so it must not be flushed into by an arbitrary
                        // other filament.
                        let support_overriddable =
                            object.support_filament == 0 && this_support_layer.planned_filament_id == 0;
                        let support_intf_overriddable = object.support_interface_filament == 0
                            && this_support_layer.planned_interface_filament_id == 0;
                        if !support_overriddable && !support_intf_overriddable {
                            break 'support;
                        }

                        let entities = &this_support_layer.entities;
                        if support_overriddable
                            && !self.is_support_overridden(object.id)
                            && !(object.support_interface_not_for_body
                                && !support_intf_overriddable
                                && (new_extruder as i32 == object.support_interface_filament - 1
                                    || old_extruder as i32 == object.support_interface_filament - 1))
                        {
                            self.set_support_extruder_override(object.id, new_extruder as i32);
                            for (role, volume) in entities {
                                if *role == crate::extrusion_entity::ExtrusionRole::SupportMaterial
                                    || *role == crate::extrusion_entity::ExtrusionRole::SupportTransition
                                {
                                    volume_to_wipe -= *volume as f32;
                                }
                                if volume_to_wipe <= 0.0 {
                                    return 0.0;
                                }
                            }
                        }

                        if support_intf_overriddable && !self.is_support_interface_overridden(object.id) {
                            self.set_support_interface_extruder_override(object.id, new_extruder as i32);
                            for (role, volume) in entities {
                                if *role == crate::extrusion_entity::ExtrusionRole::SupportMaterialInterface {
                                    volume_to_wipe -= *volume as f32;
                                }
                                if volume_to_wipe <= 0.0 {
                                    return 0.0;
                                }
                            }
                        }
                    }
                }
            }
        }
        // Some purge remains to be done on the Wipe Tower.
        volume_to_wipe
    }

    /// `WipingExtrusions::ensure_perimeters_infills_order` (ToolOrdering.cpp:4055-4117).
    ///
    /// Called after all toolchanges on a layer were marked. There might still be
    /// overridable entities that were not actually overridden; printing them with
    /// the extruder they were assigned to might violate the perimeter-infill order.
    pub fn ensure_perimeters_infills_order(&mut self, lt: &WipingLayer) {
        if !self.something_overridable {
            return;
        }

        let first_nonsoluble_extruder = Self::first_nonsoluble_extruder_on_layer(lt);
        let last_nonsoluble_extruder = Self::last_nonsoluble_extruder_on_layer(lt);

        for object in lt.objects {
            // Finds this layer:
            let Some(regions) = object.regions.as_ref() else { continue };
            let num_of_copies = object.num_of_copies;

            // iterate through copies first, so that we mark neighbouring infills to minimize travel moves
            for copy in 0..num_of_copies {
                for layerm in regions {
                    // BBS
                    if !object.flush_into_infill && !object.flush_into_objects {
                        continue;
                    }

                    let is_infill_first = lt.is_infill_first;
                    // iterate through all infill Collections
                    for fill in &layerm.fills {
                        if !Self::is_overriddable_entity(lt, fill, object)
                            || self.is_entity_overridden(fill.id, object.id, copy)
                        {
                            continue;
                        }

                        // This infill could have been overridden but was not - unless we do something, it
                        // could be printed before its perimeter, or not be printed at all (in case its
                        // original extruder has not been added to LayerTools). Either way, we will now
                        // force-override it with something suitable:
                        // WIPE_ORDER_PAREN_CPP (on): ToolOrdering.cpp:4093-4094 reads
                        //   lt.is_extruder_order(lt.wall_filament(region), last_nonsoluble_extruder
                        //       || ! lt.has_extruder(lt.sparse_infill_filament(region)))
                        // with the `||` inside the call: the second argument is the
                        // truth value, 1 or 0, of "the last extruder is not extruder 0,
                        // or the infill's extruder is not on the layer". =0 takes the
                        // two as separate conditions, as the comments beside them say.
                        let forced = if crate::faithful_gate("WIPE_ORDER_PAREN_CPP") {
                            let b = (last_nonsoluble_extruder as u32 != 0
                                || !lt.has_extruder(layerm.sparse_infill_filament))
                                as u32;
                            lt.is_extruder_order(layerm.wall_filament, b)
                        } else {
                            // !infill_first, but perimeter is already printed when last extruder prints
                            lt.is_extruder_order(layerm.wall_filament, last_nonsoluble_extruder as u32)
                                // we have to force override - this could violate infill_first (FIXME)
                                || !lt.has_extruder(layerm.sparse_infill_filament)
                        };
                        if is_infill_first || forced {
                            self.set_extruder_override(
                                fill.id,
                                object.id,
                                copy,
                                if is_infill_first { first_nonsoluble_extruder } else { last_nonsoluble_extruder },
                                num_of_copies,
                            );
                        } else {
                            // In this case we can (and should) leave it to be printed normally.
                            // Force overriding would mean it gets printed before its perimeter.
                        }
                    }

                    // Now the same for perimeters - see comments above for explanation:
                    for fill in &layerm.perimeters {
                        if Self::is_overriddable_entity(lt, fill, object)
                            && !self.is_entity_overridden(fill.id, object.id, copy)
                        {
                            self.set_extruder_override(
                                fill.id,
                                object.id,
                                copy,
                                if is_infill_first { last_nonsoluble_extruder } else { first_nonsoluble_extruder },
                                num_of_copies,
                            );
                        }
                    }
                }
            }
        }
    }

    /// The override of one entity for one copy, when there is one (the value
    /// `entity_map` holds before `get_extruder_overrides` fills in the rest).
    pub fn entity_override(&self, entity_id: u64, object_id: u64, copy_id: usize) -> Option<i32> {
        self.entity_overrides
            .get(&(entity_id, object_id))
            .and_then(|v| v.get(copy_id).copied())
            .filter(|&e| e != -1)
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
    /// ToolOrdering.hpp:210 `const CustomGCode::Item *custom_gcode`: the item itself, as
    /// `ToolOrderingCpp::assign_custom_gcodes` sets it (the field above is the earlier Rust API's).
    pub custom_gcode_item: Option<crate::custom_g_code::Item>,
    /// ToolOrdering.hpp:212-214: 0-based mixed filament slot → 0-based resolved physical filament
    /// for this layer. Populated by `ToolOrdering::resolve_mixed_filaments()`.
    pub mixed_filament_resolution: std::collections::BTreeMap<u32, u32>,
    /// ToolOrdering.hpp:258
    pub mixed_sub_layer_groups: Vec<MixedSubLayerGroup>,
    /// Wiping extrusions for this layer
    wiping_extrusions: WipingExtrusions,
}

/// ToolOrdering.hpp:229-235 `LayerTools::MixedSubLayerGroup::ObjectGradient` (also `VolumeGradient`).
#[derive(Debug, Clone, PartialEq)]
pub struct ObjectGradient {
    pub total_layers: usize,
    pub current_idx: usize,
    pub gradient_start: f64,
    pub gradient_end: f64,
    /// empty -> linear fallback (start, end); non-empty wins
    pub curve: crate::filament_mixer::GradientCurve,
}

/// ToolOrdering.hpp:244-254 `LayerTools::MixedSubLayerGroup::VolumeKey`.
///
/// FIDELITY-NOTE: `obj` is the `const PrintObject*` of the C++; the maps keyed by it iterate in
/// pointer order there. Here it is the index the caller gives the object (`ToolOrderingObject::key`),
/// which follows the order the objects were created in, as allocation addresses usually do.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct VolumeKey {
    pub obj: usize,
    pub volume_id: usize,
}

/// ToolOrdering.hpp:221-257 `LayerTools::MixedSubLayerGroup`
#[derive(Debug, Clone, Default, PartialEq)]
pub struct MixedSubLayerGroup {
    pub mixed_slot_0based: u32,
    pub components_0based: Vec<u32>,
    /// per-component, sum ≈ layer_height
    pub sub_heights: Vec<f64>,
    /// the actual lh used to compute sub_heights
    pub layer_height: f64,
    pub is_gradient: bool,
    /// index of "first" config component after sorting
    pub gradient_first_sorted_idx: i32,
    pub per_object_gradient: std::collections::BTreeMap<usize, ObjectGradient>,
    pub per_volume_gradient: std::collections::BTreeMap<VolumeKey, ObjectGradient>,
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
            custom_gcode_item: None,
            mixed_filament_resolution: std::collections::BTreeMap::new(),
            mixed_sub_layer_groups: Vec::new(),
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
    ///
    /// ToolOrdering.cpp:138-143; the argument is `region.config().wall_filament.value`. Gate
    /// `TOOL_ORDERING_CPP` (on): the C++ unsigned arithmetic and `resolve_mixed()`; =0 the earlier
    /// saturating form.
    pub fn wall_filament(&self, region_wall_filament: u32) -> u32 {
        if crate::faithful_gate("TOOL_ORDERING_CPP") {
            let result = (if self.extruder_override == 0 { region_wall_filament } else { self.extruder_override }).wrapping_sub(1);
            return self.resolve_mixed(result);
        }
        if self.extruder_override == 0 {
            region_wall_filament.saturating_sub(1) // Convert 1-based to 0-based
        } else {
            self.extruder_override - 1
        }
    }

    /// Get the sparse infill filament for a region (considering override)
    ///
    /// ToolOrdering.cpp:145-150; the argument is `region.config().sparse_infill_filament.value`.
    pub fn sparse_infill_filament(&self, region_sparse_infill_filament: u32) -> u32 {
        if crate::faithful_gate("TOOL_ORDERING_CPP") {
            let result = (if self.extruder_override == 0 { region_sparse_infill_filament } else { self.extruder_override })
                .wrapping_sub(1);
            return self.resolve_mixed(result);
        }
        if self.extruder_override == 0 {
            region_sparse_infill_filament.saturating_sub(1)
        } else {
            self.extruder_override - 1
        }
    }

    /// Get the solid infill filament for a region (considering override)
    ///
    /// ToolOrdering.cpp:152-157; the argument is `region.config().solid_infill_filament.value`.
    pub fn solid_infill_filament(&self, region_solid_infill_filament: u32) -> u32 {
        if crate::faithful_gate("TOOL_ORDERING_CPP") {
            let result = (if self.extruder_override == 0 { region_solid_infill_filament } else { self.extruder_override })
                .wrapping_sub(1);
            return self.resolve_mixed(result);
        }
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
    // ---- the members of the C++ class the earlier Rust API did not have (ToolOrdering.hpp:437-491) ----
    /// ToolOrdering.hpp:444 `const PrintConfig* m_print_config_ptr`: set by the `Print` constructor.
    pub print_config_ptr: Option<ToolOrderingConfigPtr>,
    /// ToolOrdering.hpp:448
    pub mixed_object_layers: BTreeMap<u32, BTreeMap<usize, Vec<usize>>>,
    /// ToolOrdering.hpp:452
    pub mixed_layer_heights: BTreeMap<u32, BTreeMap<usize, f64>>,
    /// ToolOrdering.hpp:455
    pub used_mixed_filaments: Vec<u32>,
    /// ToolOrdering.hpp:457
    pub directly_used_filaments: Vec<u32>,
    /// ToolOrdering.hpp:460 (print_z, 1-based slot)
    pub bed_contact_filaments: Vec<(f64, u32)>,
    /// ToolOrdering.hpp:465
    pub object_all_layer_indices: BTreeMap<usize, Vec<usize>>,
    /// ToolOrdering.hpp:470
    pub mixed_swallowed_layers: BTreeMap<u32, BTreeMap<usize, BTreeSet<usize>>>,
    /// ToolOrdering.hpp:471
    pub gradient_volume_swallowed_layers: BTreeMap<u32, BTreeMap<VolumeKey, BTreeSet<usize>>>,
    /// ToolOrdering.hpp:479
    pub gradient_volume_layers: BTreeMap<u32, BTreeMap<VolumeKey, Vec<usize>>>,
    /// ToolOrdering.hpp:487
    pub nozzle_group_result: crate::multi_nozzle_utils::LayeredNozzleGroupResult,
    /// ToolOrdering.hpp:488
    pub initial_nozzle_status: crate::multi_nozzle_utils::NozzleStatusRecorder,
    /// ToolOrdering.hpp:489
    pub nozzle_status: crate::multi_nozzle_utils::NozzleStatusRecorder,
    /// ToolOrdering.hpp:481 `const PrintObject* m_print_object_ptr = nullptr;`: whether it is set (by the
    /// PrintObject constructor, `from_object`). The object itself is passed to the functions reading it.
    pub print_object_ptr: bool,
}

/// `m_print_config_ptr` of the C++ (ToolOrdering.hpp:444): the configuration of the `Print` the
/// ordering was built from.
#[derive(Clone)]
pub struct ToolOrderingConfigPtr(pub std::sync::Arc<ToolOrderingPrintConfig>);

impl std::fmt::Debug for ToolOrderingConfigPtr {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str("ToolOrderingConfigPtr")
    }
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
            print_config_ptr: None,
            mixed_object_layers: BTreeMap::new(),
            mixed_layer_heights: BTreeMap::new(),
            used_mixed_filaments: Vec::new(),
            directly_used_filaments: Vec::new(),
            bed_contact_filaments: Vec::new(),
            object_all_layer_indices: BTreeMap::new(),
            mixed_swallowed_layers: BTreeMap::new(),
            gradient_volume_swallowed_layers: BTreeMap::new(),
            gradient_volume_layers: BTreeMap::new(),
            nozzle_group_result: crate::multi_nozzle_utils::LayeredNozzleGroupResult::default(),
            initial_nozzle_status: crate::multi_nozzle_utils::NozzleStatusRecorder::new(),
            nozzle_status: crate::multi_nozzle_utils::NozzleStatusRecorder::new(),
            print_object_ptr: false,
        }
    }

    /// ToolOrdering.hpp:289 `ToolOrdering() = default;`: no layers, no configuration pointer.
    pub fn cpp_default() -> Self {
        Self::new(ToolOrderingConfig::default())
    }

    /// Create tool ordering with default configuration
    pub fn with_default_config() -> Self {
        Self::new(ToolOrderingConfig::default())
    }

    /// Initialize layers from a list of Z heights
    ///
    /// ToolOrdering.cpp:814-827. Gate `TOOL_ORDERING_CPP` (on): the C++ — exact duplicates removed,
    /// print_z within EPSILON (1e-4) merged, appended to the layers there are; =0 the earlier form
    /// (1e-6, layers cleared first).
    pub fn initialize_layers(&mut self, mut z_heights: Vec<f64>) {
        if crate::faithful_gate("TOOL_ORDERING_CPP") {
            let zs = &mut z_heights;
            sort_remove_duplicates_f64(zs);
            // Merge numerically very close Z values.
            let mut i = 0usize;
            while i < zs.len() {
                // Find the last layer with roughly the same print_z.
                let mut j = i + 1;
                let zmax = zs[i] + EPSILON;
                while j < zs.len() && zs[j] <= zmax {
                    j += 1;
                }
                // Assign an average print_z to the set of layers with nearly equal print_z.
                self.layer_tools.push(LayerTools::new(0.5 * (zs[i] + zs[j - 1])));
                i = j;
            }
            return;
        }
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
    ///
    /// ToolOrdering.hpp:305-311. Gate `TOOL_ORDERING_CPP` (on): what the C++ clears (the layers, the
    /// three statistics and the bed contact filaments; the first / last / all extruders stay); =0
    /// the earlier form, which resets everything.
    pub fn clear(&mut self) {
        if crate::faithful_gate("TOOL_ORDERING_CPP") {
            self.layer_tools.clear();
            self.stats_single_ext.clear();
            self.stats_multi_ext_best.clear();
            self.stats_multi_ext_curr.clear();
            self.bed_contact_filaments.clear();
            return;
        }
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
// ToolOrdering.cpp / ToolOrdering.hpp at the pin, in the order of the C++ file.
//
// The configuration is a full `DynamicPrintConfig` (every option present), as for the WipeTower
// port (`crate::gcode::wipe_tower::WipeTowerPrintConfig`). What the C++ reads of `Print` /
// `PrintObject` comes through `ToolOrderingPrint` and the view structs below, named after the
// expressions they stand for (the pattern of `WipingLayer` above and of `SupportMergePlanObject`).
// The C++ members whose name the earlier, config-driven Rust API already uses with another
// signature are in trait `ToolOrderingCpp`; the two overloads of `handle_dontcare_extruder` in
// trait `HandleDontcareExtruder`.
// ============================================================================

use crate::libslic3r::EPSILON;

/// `const PrintConfig&` of the functions below: a full print configuration (every option present).
pub type ToolOrderingPrintConfig = crate::print_config::config_def::DynamicPrintConfig;

/// `config.<key>.values` of a float vector option (nullable or not).
fn cfg_floats(config: &ToolOrderingPrintConfig, key: &str) -> Vec<f64> {
    use crate::config::ConfigOptionResolver;
    let opt = config.option(key).unwrap_or_else(|| panic!("option {key}"));
    if let Some(o) = opt.as_any().downcast_ref::<crate::config::ConfigOptionFloats>() {
        return o.values.clone();
    }
    if let Some(o) = opt.as_any().downcast_ref::<crate::config::ConfigOptionFloatsNullable>() {
        return o.values.clone();
    }
    if let Some(o) = opt.as_any().downcast_ref::<crate::config::ConfigOptionPercents>() {
        return o.values.clone();
    }
    if let Some(o) = opt.as_any().downcast_ref::<crate::config::ConfigOptionPercentsNullable>() {
        return o.values.clone();
    }
    panic!("option {key} is not a float vector")
}

/// `config.<key>.values` of an int vector option (nullable or not, or an enum vector).
fn cfg_ints(config: &ToolOrderingPrintConfig, key: &str) -> Vec<i32> {
    use crate::config::ConfigOptionResolver;
    let opt = config.option(key).unwrap_or_else(|| panic!("option {key}"));
    if let Some(o) = opt.as_any().downcast_ref::<crate::config::ConfigOptionInts>() {
        return o.values.clone();
    }
    if let Some(o) = opt.as_any().downcast_ref::<crate::config::ConfigOptionIntsNullable>() {
        return o.values.clone();
    }
    if let Some(o) = opt.as_any().downcast_ref::<crate::config::ConfigOptionEnumsGeneric>() {
        return o.values.clone();
    }
    if let Some(o) = opt.as_any().downcast_ref::<crate::config::ConfigOptionEnumsGenericNullable>() {
        return o.values.clone();
    }
    panic!("option {key} is not an int vector")
}

/// `config.<key>.values` of a bool vector option, as the C++ `unsigned char`s.
fn cfg_bools(config: &ToolOrderingPrintConfig, key: &str) -> Vec<u8> {
    use crate::config::ConfigOptionResolver;
    let opt = config.option(key).unwrap_or_else(|| panic!("option {key}"));
    if let Some(o) = opt.as_any().downcast_ref::<crate::config::ConfigOptionBools>() {
        return o.values.iter().map(|&v| v as u8).collect();
    }
    if let Some(o) = opt.as_any().downcast_ref::<crate::config::ConfigOptionBoolsNullable>() {
        return o.values.iter().map(|&v| v as u8).collect();
    }
    panic!("option {key} is not a bool vector")
}

/// `config.<key>.values` of a string vector option.
fn cfg_strings(config: &ToolOrderingPrintConfig, key: &str) -> Vec<String> {
    use crate::config::ConfigOptionResolver;
    let opt = config.option(key).unwrap_or_else(|| panic!("option {key}"));
    opt.as_any()
        .downcast_ref::<crate::config::ConfigOptionStrings>()
        .unwrap_or_else(|| panic!("option {key} is not a string vector"))
        .values
        .clone()
}

/// `config.<key>.value` of a bool option.
fn cfg_bool(config: &ToolOrderingPrintConfig, key: &str) -> bool {
    config.opt_bool(key)
}

/// `config.<key>.value` of an int or enum option.
fn cfg_int(config: &ToolOrderingPrintConfig, key: &str) -> i32 {
    use crate::config::ConfigOptionResolver;
    config.option(key).and_then(|o| o.get_int().ok()).unwrap_or_else(|| panic!("option {key}"))
}

/// `config.<key>.value` of a float option.
fn cfg_float(config: &ToolOrderingPrintConfig, key: &str) -> f64 {
    *config.opt_float(key)
}

/// Config.hpp:681 `get_at(i)`: `values[i]`, the first value past the end.
fn get_at<T: Clone>(values: &[T], i: usize) -> T {
    if i < values.len() {
        values[i].clone()
    } else {
        values[0].clone()
    }
}

/// libslic3r.h `sort_remove_duplicates` of a `std::vector<unsigned int>`.
fn sort_remove_duplicates_u32(v: &mut Vec<u32>) {
    v.sort_unstable();
    v.dedup();
}

/// libslic3r.h `sort_remove_duplicates` of a `std::vector<coordf_t>`.
fn sort_remove_duplicates_f64(v: &mut Vec<f64>) {
    v.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    v.dedup_by(|a, b| *a == *b);
}

/// `ExtrusionEntity::role()` of one entity of a collection.
fn entity_role(entity: &crate::extrusion_entity::ExtrusionEntityType) -> crate::extrusion_entity::ExtrusionRole {
    use crate::extrusion_entity::{ExtrusionEntityType, ExtrusionRole};
    match entity {
        ExtrusionEntityType::Path(p) => p.role,
        ExtrusionEntityType::MultiPath(m) => m.role(),
        ExtrusionEntityType::Loop(l) => {
            if l.paths.is_empty() {
                ExtrusionRole::None
            } else {
                l.paths[0].role
            }
        }
        ExtrusionEntityType::Collection(c) => c.role(),
    }
}

/// ToolOrdering.cpp:59-73 `extrusion_touches_bed`: the same test as GCode::path_touches_bed(), the
/// extrusion height reaches down to the bed.
pub fn extrusion_touches_bed(entity: &crate::extrusion_entity::ExtrusionEntityType, print_z: f64) -> bool {
    use crate::extrusion_entity::ExtrusionEntityType;
    let path_touches_bed = |path: &crate::extrusion_entity::ExtrusionPath| print_z - path.height <= EPSILON;
    match entity {
        ExtrusionEntityType::Path(path) => path_touches_bed(path),
        ExtrusionEntityType::MultiPath(multipath) => multipath.paths.iter().any(path_touches_bed),
        ExtrusionEntityType::Loop(l) => l.paths.iter().any(path_touches_bed),
        ExtrusionEntityType::Collection(collection) => {
            collection.entities.iter().any(|child| extrusion_touches_bed(child, print_z))
        }
    }
}

/// ToolOrdering.cpp:75-84 `get_filament_by_type`
pub fn get_filament_by_type(used_filaments: &[u32], print_config: &ToolOrderingPrintConfig, type_: &str) -> BTreeSet<i32> {
    let mut target_filaments = BTreeSet::new();
    let filament_types = cfg_strings(print_config, "filament_type");
    for &filament_id in used_filaments {
        let filament_type = get_at(&filament_types, filament_id as usize);
        if filament_type == type_ {
            target_filaments.insert(filament_id as i32);
        }
    }
    target_filaments
}

/// ToolOrdering.cpp:103-135 `check_filament_printable_after_group`. A thrown
/// `Slic3r::RuntimeError` is the `Err`.
pub fn check_filament_printable_after_group(
    used_filaments: &[u32],
    filament_maps: &[i32],
    filament_variants: &mut HashMap<i32, Vec<String>>,
    _used_type: &mut Vec<crate::filament_group_utils::FilamentUsageType>,
    print_config: &ToolOrderingPrintConfig,
) -> Result<bool, crate::exception::RuntimeError> {
    use crate::i18n::translate as tr;
    use crate::config::ConfigEnum;
    // TODO(shancang): Generic filament of XP printers
    let diameters = cfg_floats(print_config, "nozzle_diameter");
    let mut nozzle_fils: HashMap<String, i32> = HashMap::new();
    let filament_printable = cfg_ints(print_config, "filament_printable");
    for &filament_id in used_filaments {
        let extruder_idx = filament_maps[filament_id as usize];
        let printable_status = get_at(&filament_printable, filament_id as usize);
        if printable_status.wrapping_shr(extruder_idx as u32) & 1 == 0 {
            let extruder_name = if extruder_idx == 0 { tr("left") } else { tr("right") };
            let error_msg = tr("Grouping error: filament")
                + &(filament_id + 1).to_string()
                + &tr(" can not be placed in the ")
                + &extruder_name
                + &tr(" nozzle");
            return Err(crate::exception::RuntimeError::new(error_msg));
        }

        if diameters.len() < 2 {
            continue;
        }

        let extruder_variant = cfg_strings(print_config, "printer_extruder_variant")[extruder_idx as usize].clone();
        let filament_variants_set: HashSet<String> =
            filament_variants.entry(filament_id as i32).or_default().iter().cloned().collect();
        if !filament_variants_set.contains(&extruder_variant) {
            let variant_name = crate::print_config::config_def::convert_to_nvt_type(&extruder_variant);
            let volume_names = crate::print_config::enums::NozzleVolumeType::get_enum_names();
            let volume = volume_names[variant_name as usize].clone();
            let error_msg = tr("Filament Grouping Error: Filament ")
                + &(filament_id + 1).to_string()
                + &tr(" is not supported by the ")
                + &tr(&volume)
                + &tr(" nozzle. ")
                + &tr("Please change the nozzle type or reassign the filament and try again.");
            return Err(crate::exception::RuntimeError::new(error_msg));
        }

        *nozzle_fils.entry(extruder_variant).or_insert(0) += 1;
    }
    Ok(true)
}

impl LayerTools {
    /// ToolOrdering.cpp:159-165: a zero based extruder this eec should be printed with, according to
    /// PrintRegion config or extruder_override if overriden.
    pub fn extruder(
        &self,
        extrusions: &crate::extrusion_entity::ExtrusionEntityCollection,
        region: &crate::print_region::PrintRegion,
    ) -> u32 {
        let mut result = self.filament_slot(extrusions, region);
        result = if result == 0 { 0 } else { result - 1 };
        self.resolve_mixed(result)
    }

    /// ToolOrdering.cpp:167-174
    pub fn filament_slot(
        &self,
        extrusions: &crate::extrusion_entity::ExtrusionEntityCollection,
        region: &crate::print_region::PrintRegion,
    ) -> u32 {
        // use separate filament for different features only if the potion was open
        if self.extruder_override == 0 {
            Self::filament_slot_by_role(extrusions, region.config())
        } else {
            self.extruder_override
        }
    }

    /// ToolOrdering.cpp:176-217
    pub fn filament_slot_by_role(
        extrusions: &crate::extrusion_entity::ExtrusionEntityCollection,
        config: &crate::region_config::PrintRegionConfig,
    ) -> u32 {
        use crate::extrusion_entity::{is_infill, is_perimeter, is_solid_infill, ExtrusionRole};
        // 1 based extruder ID.
        let mut extruder: u32 = 0;
        let extrusions_role = extrusions.role();
        if is_perimeter(extrusions_role) {
            extruder = config.wall_filament as u32;
        } else if is_solid_infill(extrusions_role) {
            extruder = config.solid_infill_filament as u32;
        } else if is_infill(extrusions_role) {
            extruder = config.sparse_infill_filament as u32;
        } else if extrusions_role == ExtrusionRole::Mixed {
            //this set of extrusions contain multi roles, follow a Priority to decide which to use
            let mut curr_priority = 0;
            for ee in &extrusions.entities {
                let role = entity_role(ee);
                if role == ExtrusionRole::TopSolidInfill {
                    if curr_priority < 100 {
                        // top surface 1st priority
                        extruder = config.solid_infill_filament as u32;
                        curr_priority = 100;
                    }
                } else if is_perimeter(role) {
                    if curr_priority < 90 {
                        // perimeter 2st priority
                        extruder = config.wall_filament as u32;
                        curr_priority = 90;
                    }
                } else if is_solid_infill(role) {
                    if curr_priority < 70 {
                        // solid infill 4st priority
                        extruder = config.solid_infill_filament as u32;
                        curr_priority = 70;
                    }
                } else if is_infill(role) {
                    if curr_priority < 60 {
                        // infill 5st priority
                        extruder = config.sparse_infill_filament as u32;
                        curr_priority = 60;
                    }
                }
            }
            if curr_priority == 0 {
                // default
                extruder = config.wall_filament as u32;
            }
        } else {
            extruder = config.wall_filament as u32;
        }
        extruder
    }

    /// ToolOrdering.hpp:216-219
    pub fn resolve_mixed(&self, filament_0based: u32) -> u32 {
        match self.mixed_filament_resolution.get(&filament_0based) {
            Some(&v) => v,
            None => filament_0based,
        }
    }

    /// ToolOrdering.hpp:260-265
    pub fn mixed_group_by_slot(&self, slot_id: u32) -> Option<&MixedSubLayerGroup> {
        for g in &self.mixed_sub_layer_groups {
            if g.mixed_slot_0based == slot_id {
                return Some(g);
            }
        }
        None
    }

    /// ToolOrdering.hpp:267-269
    pub fn is_mixed_slot(&self, slot_id: u32) -> bool {
        self.mixed_group_by_slot(slot_id).is_some()
    }
}

/// What `object_max_layer_height` reads of a `PrintObject` (ToolOrdering.cpp:223-227).
#[derive(Debug, Clone, Copy, Default)]
pub struct ObjectMaxLayerHeightInputs {
    /// `object.print()->single_diameter_layering_nozzle()`
    pub single_diameter_layering_nozzle: f64,
    /// `object.slicing_parameters().max_layer_height`
    pub slicing_max_layer_height: f64,
    /// `object.config().layer_height.value`
    pub layer_height: f64,
}

/// ToolOrdering.cpp:219-227 `object_max_layer_height`: the layer height the tower gaps are measured
/// against.
pub fn object_max_layer_height(object: &ObjectMaxLayerHeightInputs) -> f64 {
    if object.single_diameter_layering_nozzle > EPSILON {
        object.slicing_max_layer_height
    } else {
        object.layer_height
    }
}

/// ToolOrdering.cpp:229-241 `calc_max_layer_height`
pub fn calc_max_layer_height(config: &ToolOrderingPrintConfig, max_object_layer_height: f64) -> f64 {
    let mut max_layer_height = f64::MAX;
    let nozzle_diameter = cfg_floats(config, "nozzle_diameter");
    let max_layer_height_values = cfg_floats(config, "max_layer_height");
    for i in 0..nozzle_diameter.len() {
        let mut mlh = max_layer_height_values[i];
        if mlh == 0.0 {
            mlh = 0.75 * nozzle_diameter[i];
        }
        max_layer_height = max_layer_height.min(mlh);
    }
    // The Prusa3D Fast (0.35mm layer height) print profile sets a higher layer height than what is normally allowed
    // by the nozzle. This is a hack and it works by increasing extrusion width. See GH #3919.
    max_layer_height.max(max_object_layer_height)
}

/// ToolOrdering.cpp:243-292 `calc_filament_change_info_by_toolorder`: the flush weight and the
/// filament change counts of a tool order.
pub fn calc_filament_change_info_by_toolorder(
    config: &ToolOrderingPrintConfig,
    group_result: &crate::multi_nozzle_utils::LayeredNozzleGroupResult,
    flush_matrix: &[group_reorder::FlushMatrix],
    layer_sequences: &[Vec<u32>],
) -> FilamentChangeStats {
    let mut ret = FilamentChangeStats::default();
    // std::unordered_map<int, int>: summed below in libc++'s order of iteration.
    let mut flush_volume_per_filament: crate::texture_to_color::color_utils::LibcxxUnorderedMap<i32> =
        crate::texture_to_color::color_utils::LibcxxUnorderedMap::new();

    let mut recorder = crate::multi_nozzle_utils::NozzleStatusRecorder::new();
    let mut total_filament_change_count = 0;
    let mut total_flush_filament_change_count = 0;
    let mut total_filament_flush_weight: f32 = 0.0;

    let mut old_filament_id: i32 = -1;
    for (layer_idx, ls) in layer_sequences.iter().enumerate() {
        for &filament in ls {
            let Some(nozzle) = group_result.get_nozzle_for_filament(filament as i32, layer_idx as i32) else {
                continue;
            };

            let new_extruder_id = nozzle.extruder_id;
            let new_nozzle_id_in_extruder = nozzle.group_id;
            let new_filament_id_in_nozzle = filament as i32;
            let old_filament_id_in_nozzle = recorder.get_filament_in_nozzle(new_nozzle_id_in_extruder);

            let filament_in_nozzle_change =
                old_filament_id_in_nozzle != -1 && new_filament_id_in_nozzle != old_filament_id_in_nozzle;
            let filament_change = old_filament_id != -1 && old_filament_id != new_filament_id_in_nozzle;

            if filament_in_nozzle_change {
                total_flush_filament_change_count += 1;
                // `int flush_volume = flush_matrix[...]`: the float truncated toward zero.
                let flush_volume = flush_matrix[new_extruder_id as usize][old_filament_id_in_nozzle as usize]
                    [new_filament_id_in_nozzle as usize] as i32;
                *flush_volume_per_filament.entry(filament as usize) += flush_volume;
            }
            if filament_change {
                total_filament_change_count += 1;
            }
            old_filament_id = new_filament_id_in_nozzle;
            recorder.set_nozzle_status(new_nozzle_id_in_extruder, new_filament_id_in_nozzle, new_extruder_id);
        }
    }

    let filament_density = cfg_floats(config, "filament_density");
    for (first, second) in flush_volume_per_filament.items() {
        let weight: f32 = (get_at(&filament_density, first) * 0.001 * second as f64) as f32;
        total_filament_flush_weight += weight;
    }

    ret.filament_change_count = total_filament_change_count;
    ret.flush_filament_change_count = total_flush_filament_change_count;
    ret.filament_flush_weight = total_filament_flush_weight as i32;

    ret
}

/// The two overloads of `ToolOrdering::handle_dontcare_extruder` (ToolOrdering.hpp:299-300).
pub trait HandleDontcareExtruder<T> {
    fn handle_dontcare_extruder(&mut self, arg: T);
}

impl<'a> HandleDontcareExtruder<&'a [u32]> for ToolOrdering {
    /// ToolOrdering.cpp:294-381 `handle_dontcare_extruder(const std::vector<unsigned int>& tool_order_layer0)`
    fn handle_dontcare_extruder(&mut self, tool_order_layer0: &'a [u32]) {
        if self.layer_tools.is_empty() {
            return;
        }

        // Reorder the extruders of first layer
        {
            let lt = &mut self.layer_tools[0];
            let mut layer0_extruders = lt.extruders.clone();
            lt.extruders.clear();
            for &extruder_id in tool_order_layer0 {
                if let Some(iter) = layer0_extruders.iter().position(|&e| e == extruder_id) {
                    lt.extruders.push(extruder_id);
                    layer0_extruders[iter] = u32::MAX;
                }
            }

            let mut had_dontcare = false;
            for &extruder_id in &layer0_extruders {
                if extruder_id == 0 {
                    // Something is printed here, but its filament was left unspecified.
                    had_dontcare = true;
                    continue;
                }

                if extruder_id != u32::MAX {
                    lt.extruders.push(extruder_id);
                }
            }

            // A printed first layer whose filaments were all "don't care" still needs one concrete tool.
            if lt.extruders.is_empty() && had_dontcare && !tool_order_layer0.is_empty() {
                lt.extruders.push(tool_order_layer0[0]);
            }
        }

        let mut last_extruder_id: i32 = 0;
        // BBS: fist layer may be empty or only has defult extrude id
        if self.layer_tools[0].extruders.is_empty() {
            // search for first extrude filament id
            for layer_id in 1..self.layer_tools.len() {
                for &extrude in &self.layer_tools[layer_id].extruders {
                    if extrude != 0 {
                        last_extruder_id = extrude as i32;
                        break;
                    }
                }

                if last_extruder_id != 0 {
                    break;
                }
            }
        } else {
            //use first layer last extruder
            last_extruder_id = *self.layer_tools[0].extruders.last().unwrap() as i32;
        }
        for i in 1..self.layer_tools.len() {
            let lt = &mut self.layer_tools[i];

            if lt.extruders.is_empty() {
                continue;
            }
            if lt.extruders.len() == 1 && lt.extruders[0] == 0 {
                lt.extruders[0] = last_extruder_id as u32;
            } else {
                if lt.extruders[0] == 0 {
                    // Pop the "don't care" extruder, the "don't care" region will be merged with the next one.
                    lt.extruders.remove(0);
                }
                // Reorder the extruders to start with the last one.
                for i in 1..lt.extruders.len() {
                    if lt.extruders[i] == last_extruder_id as u32 {
                        // Move the last extruder to the front.
                        lt.extruders[..=i].rotate_right(1);
                        lt.extruders[0] = last_extruder_id as u32;
                        break;
                    }
                }
            }
            last_extruder_id = *lt.extruders.last().unwrap() as i32;
        }

        // Reindex the extruders, so they are zero based, not 1 based.
        for lt in self.layer_tools.iter_mut() {
            for extruder_id in lt.extruders.iter_mut() {
                *extruder_id = extruder_id.wrapping_sub(1);
            }
        }
    }
}

impl HandleDontcareExtruder<u32> for ToolOrdering {
    /// ToolOrdering.cpp:383-445 `handle_dontcare_extruder(unsigned int last_extruder_id)`
    fn handle_dontcare_extruder(&mut self, mut last_extruder_id: u32) {
        if self.layer_tools.is_empty() {
            return;
        }
        if last_extruder_id == u32::MAX {
            // The initial print extruder has not been decided yet.
            // Initialize the last_extruder_id with the first non-zero extruder id used for the print.
            last_extruder_id = 0;
            let mut i = 0usize;
            while i < self.layer_tools.len() && last_extruder_id == 0 {
                let lt = &self.layer_tools[i];
                for &extruder_id in &lt.extruders {
                    if extruder_id > 0 {
                        last_extruder_id = extruder_id;
                        break;
                    }
                }
                i += 1;
            }
            if last_extruder_id == 0 {
                // Nothing to extrude.
                return;
            }
        } else {
            // 1 based idx
            last_extruder_id = last_extruder_id.wrapping_add(1);
        }

        let first_print_z = self.layer_tools[0].print_z;
        let print_config = self.print_config_ptr.clone();
        for lt in self.layer_tools.iter_mut() {
            if lt.extruders.is_empty() {
                continue;
            }
            if lt.extruders.len() == 1 && lt.extruders[0] == 0 {
                lt.extruders[0] = last_extruder_id;
            } else {
                if lt.extruders[0] == 0 {
                    // Pop the "don't care" extruder, the "don't care" region will be merged with the next one.
                    lt.extruders.remove(0);
                }
                // Reorder the extruders to start with the last one.
                for i in 1..lt.extruders.len() {
                    if lt.extruders[i] == last_extruder_id {
                        // Move the last extruder to the front.
                        lt.extruders[..=i].rotate_right(1);
                        lt.extruders[0] = last_extruder_id;
                        break;
                    }
                }

                // On first layer with wipe tower, prefer a soluble extruder
                // at the beginning, so it is not wiped on the first layer.
                // (`lt == m_layer_tools[0]`: LayerTools::operator== compares print_z.)
                if let Some(cfg) = print_config.as_ref() {
                    if lt.print_z == first_print_z && cfg_bool(&cfg.0, "enable_prime_tower") {
                        let filament_soluble = cfg_bools(&cfg.0, "filament_soluble");
                        for i in 0..lt.extruders.len() {
                            if get_at(&filament_soluble, lt.extruders[i].wrapping_sub(1) as usize) != 0 {
                                // 1-based...
                                lt.extruders.swap(i, 0);
                                break;
                            }
                        }
                    }
                }
            }
            last_extruder_id = *lt.extruders.last().unwrap();
        }

        // Reindex the extruders, so they are zero based, not 1 based.
        for lt in self.layer_tools.iter_mut() {
            for extruder_id in lt.extruders.iter_mut() {
                *extruder_id = extruder_id.wrapping_sub(1);
            }
        }
    }
}

/// What the member functions of ToolOrdering read of `Print` through `m_print` or a
/// `const Print&` argument (ToolOrdering.cpp). The C++ stores `m_print` in the constructor; here
/// the functions that read it take the print as an argument.
pub trait ToolOrderingPrint {
    /// `print.config()`
    fn config(&self) -> &ToolOrderingPrintConfig;
    /// `print.full_print_config()`
    fn full_print_config(&self) -> &ToolOrderingPrintConfig;
    /// `print.uses_multiple_nozzle_diameters()`
    fn uses_multiple_nozzle_diameters(&self) -> bool;
    /// `print.object_extruders()`
    fn object_extruders(&self) -> Vec<u32>;
    /// `print.model().get_curr_plate_custom_gcodes()`
    fn curr_plate_custom_gcodes(&self) -> &crate::custom_g_code::Info;
    /// `print.get_layered_nozzle_group_result()` (Print.hpp:1085-1088): the shared pointer, `None`
    /// when null.
    fn get_layered_nozzle_group_result(&self) -> Option<std::sync::Arc<crate::multi_nozzle_utils::LayeredNozzleGroupResult>>;
    /// `print.get_geometric_unprintable_filaments()`
    fn get_geometric_unprintable_filaments(&self) -> Vec<BTreeSet<i32>>;
    /// `print.get_physical_unprintable_filaments(used_filaments)`
    fn get_physical_unprintable_filaments(&self, used_filaments: &[u32]) -> Vec<BTreeSet<i32>>;
    /// `print.get_filament_unprintable_flow(used_filaments)`
    fn get_filament_unprintable_flow(
        &self,
        used_filaments: &[u32],
    ) -> BTreeMap<i32, BTreeSet<crate::extruder::NozzleVolumeType>>;
    // ---- what the filament grouping of ToolOrdering.cpp:1710-3634 reads and writes of `Print` ----
    // FIDELITY-NOTE: `m_print` is a non-const `Print*` in C++; the members that change the print take
    // `&self` here and an implementor keeps that state behind interior mutability.
    /// `print->calib_params()` (Print.hpp:1169, calib.hpp:37-45)
    fn calib_params(&self) -> ToolOrderingCalibParams;
    /// `print->get_extruder_filament_info()` (Print.hpp:1071)
    fn get_extruder_filament_info(&self) -> &[Vec<ToolOrderingPrintConfig>];
    /// `print->get_filament_usage_type()` (Print.cpp:3474-3503)
    fn get_filament_usage_type(&self) -> Vec<crate::filament_group_utils::FilamentUsageType>;
    /// `print->get_filament_print_time()` (Print.hpp:1081): a copy of the map.
    fn get_filament_print_time(
        &self,
    ) -> crate::multi_nozzle_utils::UnorderedMapInt<crate::multi_nozzle_utils::UnorderedMapInt<f64>>;
    /// `print->get_nozzle_loaded_filament_info()` (Print.hpp:1074)
    fn get_nozzle_loaded_filament_info(&self) -> &[crate::filament_group_utils::NozzleLoadedFilamentInfo];
    /// `print->is_dynamic_group_reorder()` (Print.cpp:1306-1318)
    fn is_dynamic_group_reorder(&self) -> bool;
    /// `print->is_sequential_print()` (Print.hpp:1038-1040)
    fn is_sequential_print(&self) -> bool;
    /// `print->resolve_manual_nozzle_map(used_filaments)` (Print.cpp:3322-3345); throws.
    fn resolve_manual_nozzle_map(&self, used_filaments: &[u32]) -> Result<(), crate::exception::RuntimeError>;
    /// `print->set_nozzle_group_result(std::make_shared<LayeredNozzleGroupResult>(result))` (Print.hpp:1083)
    fn set_nozzle_group_result(&self, result: std::sync::Arc<crate::multi_nozzle_utils::LayeredNozzleGroupResult>);
    /// `print->update_filament_maps_to_config(f_maps, f_volume_maps, f_nozzle_maps)` (Print.cpp:3241-3320)
    fn update_filament_maps_to_config(&self, f_maps: Vec<i32>, f_volume_maps: Vec<i32>, f_nozzle_maps: Vec<i32>);
    /// `print->update_to_config_by_nozzle_group_result(result)` (Print.cpp:3347-3400)
    fn update_to_config_by_nozzle_group_result(&self, result: &crate::multi_nozzle_utils::LayeredNozzleGroupResult);
}

/// calib.hpp:37-45 `struct Calib_Params`: the members ToolOrdering.cpp reads (`build_filament_group_context`,
/// `get_recommended_filament_maps`).
///
/// FIDELITY-NOTE: `crate::calib::CalibParams` has no `has_bowden_extruder` (calib.rs); this holds the
/// three members read here.
#[derive(Debug, Clone, Copy)]
pub struct ToolOrderingCalibParams {
    /// calib.hpp:40 `int extruder_id = 0;`
    pub extruder_id: i32,
    /// calib.hpp:41 `bool has_bowden_extruder = false;`
    pub has_bowden_extruder: bool,
    /// calib.hpp:44 `CalibMode mode;` (Calib_None by the constructor)
    pub mode: crate::calib::CalibMode,
}

impl Default for ToolOrderingCalibParams {
    /// calib.hpp:39 `Calib_Params() : mode(CalibMode::Calib_None){}`
    fn default() -> Self {
        ToolOrderingCalibParams { extruder_id: 0, has_bowden_extruder: false, mode: crate::calib::CalibMode::None }
    }
}

/// The members of `ToolOrdering` whose name the earlier Rust API of this file uses with another
/// signature (`fill_wipe_tower_partitions(object_bottom_z)`, `mark_skirt_layers()`,
/// `assign_custom_gcodes(&[CustomGCodeItem])`), with the signature of the C++. Being trait
/// methods, they are called as `ToolOrderingCpp::fill_wipe_tower_partitions(&mut ordering, ...)`.
pub trait ToolOrderingCpp {
    fn fill_wipe_tower_partitions(&mut self, config: &ToolOrderingPrintConfig, object_bottom_z: f64, max_layer_height: f64);
    fn mark_skirt_layers(&mut self, config: &ToolOrderingPrintConfig, max_layer_height: f64, print: Option<&dyn ToolOrderingPrint>);
    fn assign_custom_gcodes(&mut self, print: &dyn ToolOrderingPrint);
}

impl ToolOrderingCpp for ToolOrdering {
    /// ToolOrdering.cpp:1208-1392 `ToolOrdering::fill_wipe_tower_partitions`
    fn fill_wipe_tower_partitions(&mut self, config: &ToolOrderingPrintConfig, object_bottom_z: f64, max_layer_height: f64) {
        if self.layer_tools.is_empty() {
            return;
        }

        // Count the minimum number of tool changes per layer.
        let mut last_extruder = usize::MAX;
        for lt in self.layer_tools.iter_mut() {
            lt.wipe_tower_partitions = lt.extruders.len();
            if !lt.extruders.is_empty() {
                if last_extruder == usize::MAX || last_extruder == lt.extruders[0] as usize {
                    // The first extruder on this layer is equal to the current one, no need to do an initial tool change.
                    lt.wipe_tower_partitions = lt.wipe_tower_partitions.wrapping_sub(1);
                }
                last_extruder = *lt.extruders.last().unwrap() as usize;
            }
        }

        // Propagate the wipe tower partitions down to support the upper partitions by the lower partitions.
        let mut i = self.layer_tools.len() as i32 - 2;
        while i >= 0 {
            let iu = i as usize;
            self.layer_tools[iu].wipe_tower_partitions =
                self.layer_tools[iu + 1].wipe_tower_partitions.max(self.layer_tools[iu].wipe_tower_partitions);
            i -= 1;
        }

        let wrapping_layer_nums = cfg_int(config, "wrapping_detection_layers");
        let enable_wrapping_detection = cfg_bool(config, "enable_wrapping_detection");
        let mut i = 0usize;
        // `size_t i < int`: the int converted to size_t.
        while i < wrapping_layer_nums as usize {
            if i >= self.layer_tools.len() {
                break;
            }
            let lt = &mut self.layer_tools[i];
            lt.has_wipe_tower = enable_wrapping_detection;
            i += 1;
        }

        // TimelapseType::tlSmooth
        let timelapse_smooth = cfg_int(config, "timelapse_type") == 1;
        for lt in self.layer_tools.iter_mut() {
            let tower_needed = timelapse_smooth || lt.wipe_tower_partitions > 0;
            // Below the object bottom sit raft layers and, with multi-diameter adaptive merging, empty grid layers
            // swallowed by the first band. They only carry the tower down to the bed when a tower is needed at all;
            // wipe_tower_partitions was propagated downwards above, so any tool change higher up reaches them.
            lt.has_wipe_tower |= (lt.has_object || lt.has_support || lt.print_z < object_bottom_z + EPSILON) && tower_needed;
        }

        // Test for a raft, insert additional wipe tower layer to fill in the raft separation gap.
        let mut i = 0usize;
        while i + 1 < self.layer_tools.len() {
            let lt_print_z = self.layer_tools[i].print_z;
            let lt_next_print_z = self.layer_tools[i + 1].print_z;
            if lt_print_z < object_bottom_z + EPSILON && lt_next_print_z >= object_bottom_z + EPSILON {
                // lt is the last raft layer. Find the 1st object layer.
                let mut j = i + 1;
                while j < self.layer_tools.len() && !self.layer_tools[j].has_wipe_tower {
                    j += 1;
                }
                if j < self.layer_tools.len() {
                    let lt_object_print_z = self.layer_tools[j].print_z;
                    let gap = lt_object_print_z - lt_print_z;
                    // assert(gap > 0.f): compiled out (-DNDEBUG).
                    if gap > max_layer_height + EPSILON {
                        // Insert one additional wipe tower layer between lh.print_z and lt_object.print_z.
                        // `LayerTools lt_new(0.5f * (lt.print_z + lt_object.print_z))`: a float factor.
                        let lt_new = LayerTools::new(0.5f32 as f64 * (lt_print_z + lt_object_print_z));
                        // Find the 1st layer above lt_new.
                        j = i + 1;
                        while j < self.layer_tools.len() && self.layer_tools[j].print_z < lt_new.print_z - EPSILON {
                            j += 1;
                        }
                        if (self.layer_tools[j].print_z - lt_new.print_z).abs() < EPSILON {
                            self.layer_tools[j].has_wipe_tower = true;
                        } else {
                            self.layer_tools.insert(j, lt_new);
                            let next_front = self.layer_tools[j + 1].extruders[0];
                            let next_partitions = self.layer_tools[j + 1].wipe_tower_partitions;
                            let lt_extra = &mut self.layer_tools[j];
                            lt_extra.has_wipe_tower = true;
                            lt_extra.extruders.push(next_front);
                            lt_extra.wipe_tower_partitions = next_partitions;
                        }
                    }
                }
                break;
            }
            i += 1;
        }

        // Ensure wipe tower vertical continuity (ToolOrdering.cpp:1284-1360).
        {
            let mut first_wt_idx: i32 = -1;
            let mut last_wt_idx: i32 = -1;
            for i in 0..self.layer_tools.len() as i32 {
                if self.layer_tools[i as usize].has_wipe_tower {
                    if first_wt_idx < 0 {
                        first_wt_idx = i;
                    }
                    last_wt_idx = i;
                }
            }
            let mut i = first_wt_idx + 1;
            while i < last_wt_idx {
                let iu = i as usize;
                self.layer_tools[iu].has_wipe_tower = true;
                // GCode::process_layer emits wipe-tower G-code inside `for (extruder_id : layer_tools.extruders)`.
                // An empty extruders vector here would silently skip wipe tower output, leaving the tower
                // physically floating. Seed from the nearest non-empty neighbor so the loop actually runs.
                if self.layer_tools[iu].extruders.is_empty() {
                    let mut seed_extruder: u32 = 0;
                    let mut found_seed = false;
                    let mut j = i - 1;
                    while j >= 0 {
                        if !self.layer_tools[j as usize].extruders.is_empty() {
                            seed_extruder = *self.layer_tools[j as usize].extruders.last().unwrap();
                            found_seed = true;
                            break;
                        }
                        j -= 1;
                    }
                    if !found_seed {
                        let mut j = i + 1;
                        while j < self.layer_tools.len() as i32 {
                            if !self.layer_tools[j as usize].extruders.is_empty() {
                                seed_extruder = self.layer_tools[j as usize].extruders[0];
                                found_seed = true;
                                break;
                            }
                            j += 1;
                        }
                    }
                    if found_seed {
                        self.layer_tools[iu].extruders.push(seed_extruder);
                    }
                }
                i += 1;
            }

            // Walk adjacent has_wipe_tower pairs and split oversized gaps. Re-evaluate the same i
            // after each insertion so very large gaps get split into multiple layers.
            let mut i = 0usize;
            while i + 1 < self.layer_tools.len() {
                let lt = &self.layer_tools[i];
                let lt_next = &self.layer_tools[i + 1];
                if !lt.has_wipe_tower || !lt_next.has_wipe_tower {
                    i += 1;
                    continue;
                }
                let gap = lt_next.print_z - lt.print_z;
                if gap <= max_layer_height + EPSILON {
                    i += 1;
                    continue;
                }
                let mut lt_new = LayerTools::new(0.5 * (lt.print_z + lt_next.print_z));
                lt_new.has_wipe_tower = true;
                if !lt_next.extruders.is_empty() {
                    lt_new.extruders.push(lt_next.extruders[0]);
                } else if !lt.extruders.is_empty() {
                    lt_new.extruders.push(*lt.extruders.last().unwrap());
                }
                lt_new.wipe_tower_partitions = lt_next.wipe_tower_partitions;
                self.layer_tools.insert(i + 1, lt_new);
            }
        }

        // If the model contains empty layers (such as https://github.com/prusa3d/Slic3r/issues/1266), there might be layers
        // that were not marked as has_wipe_tower, even when they should have been. This produces a crash with soluble supports
        // and maybe other problems. We will therefore go through layer_tools and detect and fix this.
        // So, if there is a non-object layer starting with different extruder than the last one ended with (or containing more than one extruder),
        // we'll mark it with has_wipe tower.
        let mut i = 0usize;
        while i + 1 < self.layer_tools.len() {
            if self.layer_tools[i].extruders.is_empty() || self.layer_tools[i + 1].extruders.is_empty() {
                break;
            }
            {
                let lt_back = *self.layer_tools[i].extruders.last().unwrap();
                let lt_next = &mut self.layer_tools[i + 1];
                if !lt_next.has_wipe_tower && (lt_next.extruders[0] != lt_back || lt_next.extruders.len() > 1) {
                    lt_next.has_wipe_tower = true;
                }
            }
            // We should also check that the next wipe tower layer is no further than max_layer_height:
            let mut j = i + 1;
            let mut last_wipe_tower_print_z = self.layer_tools[i + 1].print_z;
            loop {
                j += 1;
                if !(j < self.layer_tools.len().wrapping_sub(1) && !self.layer_tools[j].has_wipe_tower) {
                    break;
                }
                if self.layer_tools[j + 1].print_z - last_wipe_tower_print_z > max_layer_height + EPSILON {
                    if !enable_wrapping_detection {
                        self.layer_tools[j].has_wipe_tower = true;
                    }
                    last_wipe_tower_print_z = self.layer_tools[j].print_z;
                }
            }
            i += 1;
        }

        // Calculate the wipe_tower_layer_height values.
        let mut wipe_tower_print_z_last = 0.0f64;
        for lt in self.layer_tools.iter_mut() {
            if lt.has_wipe_tower {
                lt.wipe_tower_layer_height = lt.print_z - wipe_tower_print_z_last;
                wipe_tower_print_z_last = lt.print_z;
            }
        }
    }

    /// ToolOrdering.cpp:3635-3683 `ToolOrdering::mark_skirt_layers`: layers are marked for infinite
    /// skirt aka draft shield. Not all the layers have to be printed. `print` is `m_print` (`None`:
    /// a null pointer).
    fn mark_skirt_layers(&mut self, _config: &ToolOrderingPrintConfig, max_layer_height: f64, print: Option<&dyn ToolOrderingPrint>) {
        if self.layer_tools.is_empty() {
            return;
        }

        let mut i = 0usize;
        if self.layer_tools[0].extruders.is_empty() {
            // BBS multi-diameter: when every bed band is merged, the first grid layers print nothing and the skirt
            // starts with the first band top instead.
            if let Some(print) = print {
                if print.uses_multiple_nozzle_diameters() {
                    while i < self.layer_tools.len() && self.layer_tools[i].extruders.is_empty() {
                        i += 1;
                    }
                }
            }
            if i == 0 || i == self.layer_tools.len() {
                // Empty first layer, no skirt will be printed.
                //FIXME throw an exception?
                return;
            }
        }

        loop {
            self.layer_tools[i].has_skirt = true;
            let mut j = i + 1;
            while j < self.layer_tools.len() && !self.layer_tools[j].has_object {
                j += 1;
            }
            // i and j are two successive layers printing an object.
            if j == self.layer_tools.len() {
                // Don't print skirt above the last object layer.
                break;
            }
            // Mark some printing intermediate layers as having skirt.
            let mut last_z = self.layer_tools[i].print_z;
            let mut k = i + 1;
            while k < j {
                if self.layer_tools[k + 1].print_z - last_z > max_layer_height + EPSILON {
                    // Layer k is the last one not violating the maximum layer height.
                    // Don't extrude skirt on empty layers.
                    while self.layer_tools[k].extruders.is_empty() {
                        k -= 1;
                    }
                    if self.layer_tools[k].has_skirt {
                        // Skirt cannot be generated due to empty layers, there would be a missing layer in the skirt.
                        //FIXME throw an exception?
                        break;
                    }
                    self.layer_tools[k].has_skirt = true;
                    last_z = self.layer_tools[k].print_z;
                }
                k += 1;
            }
            i = j;
        }
    }

    /// ToolOrdering.cpp:3685-3774 `ToolOrdering::assign_custom_gcodes`: a pointer to a custom G-code
    /// to the respective LayerTools (`LayerTools::custom_gcode_item`). Color changes performed on a
    /// layer and for an extruder that will not print above that layer are ignored. If multiple
    /// events are planned over a span of a single layer, the last one is used.
    fn assign_custom_gcodes(&mut self, print: &dyn ToolOrderingPrint) {
        use crate::custom_g_code::{Mode, Type};
        // BBS: replace model custom gcode with current plate custom gcode
        let custom_gcode_per_print_z = print.curr_plate_custom_gcodes().clone();
        if custom_gcode_per_print_z.gcodes.is_empty() {
            return;
        }

        // BBS
        let num_filaments = cfg_floats(print.config(), "filament_diameter").len() as u32;
        let mode = if num_filaments == 1 {
            Mode::SingleExtruder
        } else if print.object_extruders().len() == 1 {
            Mode::MultiAsSingle
        } else {
            Mode::MultiExtruder
        };
        let model_mode = print.curr_plate_custom_gcodes().mode;
        // Tool changes and color changes will be ignored, if the model's tool/color changes were entered in mm mode and the print is in non mm mode
        // or vice versa.
        let ignore_tool_and_color_changes = (mode == Mode::MultiExtruder) != (model_mode == Mode::MultiExtruder);
        // If printing on a single extruder machine, make the tool changes trigger color change (M600) events.
        let tool_changes_as_color_changes = mode == Mode::SingleExtruder && model_mode == Mode::MultiAsSingle;

        let apply_custom_gcode_to_layer =
            |lt: &mut LayerTools, extruder_printing_above: &[u8], item: &crate::custom_g_code::Item| {
                let color_change = item.gcode_type == Type::ColorChange;
                let tool_change = item.gcode_type == Type::ToolChange;
                let pause_or_custom_gcode = !color_change && !tool_change;
                let apply_color_change = !ignore_tool_and_color_changes
                    // If it is color change, it will actually be useful as the exturder above will print.
                    // BBS
                    && (if color_change {
                        mode == Mode::SingleExtruder
                            || (item.extruder <= num_filaments as i32
                                && extruder_printing_above[(item.extruder - 1) as u32 as usize] != 0)
                    } else {
                        tool_change && tool_changes_as_color_changes
                    });
                if pause_or_custom_gcode || apply_color_change {
                    lt.custom_gcode_item = Some(item.clone());
                }
            };

        // std::unordered_map<int, std::vector<unsigned char>>: only looked up.
        let mut extruder_print_above_by_layer: HashMap<i32, Vec<u8>> = HashMap::new();
        {
            let mut extruder_printing_above = vec![0u8; num_filaments as usize];
            for (rev_idx, lt) in self.layer_tools.iter().rev().enumerate() {
                for &i in &lt.extruders {
                    extruder_printing_above[i as usize] = 1;
                }
                let layer_idx = (self.layer_tools.len() - rev_idx - 1) as i32;
                extruder_print_above_by_layer.entry(layer_idx).or_insert_with(|| extruder_printing_above.clone());
            }
        }

        for item in custom_gcode_per_print_z.gcodes.iter().rev() {
            if item.gcode_type == Type::ToolChange {
                continue;
            }

            // std::upper_bound(..., print_z, z < lt.print_z)
            let upper = self.layer_tools.partition_point(|lt| !(item.print_z < lt.print_z));

            let upper_layer_idx = upper as i32;
            if upper == 0 {
                let above = extruder_print_above_by_layer.entry(0).or_default().clone();
                apply_custom_gcode_to_layer(&mut self.layer_tools[0], &above, item);
            } else if upper == self.layer_tools.len() {
                let lower = upper - 1;
                let lower_layer_idx = lower as i32;
                // Custom G-code Z strictly above the top print layer: cannot be reached; skip (do not clamp to top).
                if item.print_z > self.layer_tools[lower].print_z + EPSILON {
                    continue;
                }
                let above = extruder_print_above_by_layer.entry(lower_layer_idx).or_default().clone();
                apply_custom_gcode_to_layer(&mut self.layer_tools[lower], &above, item);
            } else {
                let lower = upper - 1;
                let lower_layer_idx = lower as i32;
                let gap_to_lower = (item.print_z - self.layer_tools[lower].print_z).abs();
                let gap_to_upper = (item.print_z - self.layer_tools[upper].print_z).abs();
                if gap_to_lower < gap_to_upper {
                    let above = extruder_print_above_by_layer.entry(lower_layer_idx).or_default().clone();
                    apply_custom_gcode_to_layer(&mut self.layer_tools[lower], &above, item);
                } else {
                    let above = extruder_print_above_by_layer.entry(upper_layer_idx).or_default().clone();
                    apply_custom_gcode_to_layer(&mut self.layer_tools[upper], &above, item);
                }
            }
        }
    }
}

impl ToolOrdering {
    /// ToolOrdering.hpp:328-330
    pub fn used_mixed_filaments(&self) -> &[u32] {
        &self.used_mixed_filaments
    }

    /// ToolOrdering.hpp:332-334
    pub fn directly_used_filaments(&self) -> &[u32] {
        &self.directly_used_filaments
    }

    /// ToolOrdering.hpp:359
    pub fn get_layered_nozzle_group_result(&self) -> &crate::multi_nozzle_utils::LayeredNozzleGroupResult {
        &self.nozzle_group_result
    }

    /// ToolOrdering.hpp:419
    pub fn get_nozzle_status(&self) -> &crate::multi_nozzle_utils::NozzleStatusRecorder {
        &self.nozzle_status
    }

    /// ToolOrdering.hpp:420 (copy assignments)
    pub fn set_nozzle_status(&mut self, status: &crate::multi_nozzle_utils::NozzleStatusRecorder) {
        self.initial_nozzle_status.assign_from(status);
        self.nozzle_status.assign_from(status);
    }

    /// ToolOrdering.cpp:1394-1400 `ToolOrdering::seed_first_tower_layer`: give the first tower layer
    /// a filament when nothing else prints on it.
    pub fn seed_first_tower_layer(&mut self, filament: u32) {
        let Some(first) = self.layer_tools.iter_mut().find(|lt| lt.has_wipe_tower) else {
            return;
        };
        if !first.extruders.is_empty() {
            return;
        }
        first.extruders.push(filament);
    }

    /// ToolOrdering.cpp:1436-1470 `ToolOrdering::calc_most_used_extruder`. `print` is `m_print`.
    pub fn calc_most_used_extruder(&mut self, config: &ToolOrderingPrintConfig, print: &dyn ToolOrderingPrint) {
        // record
        let mut extruder_count: Vec<i32> = vec![0; cfg_floats(config, "nozzle_diameter").len()];
        let Some(group_result) = print.get_layered_nozzle_group_result() else {
            return;
        };
        let mut layer_idx = 0i32;
        for layer_tools in &self.layer_tools {
            let filaments = layer_tools.extruders.clone();
            let mut layer_extruder_count: BTreeSet<i32> = BTreeSet::new();
            //count once only
            for &filament in &filaments {
                layer_extruder_count.insert(group_result.get_extruder_id(filament as i32, layer_idx));
            }
            layer_idx += 1;

            //record
            for &extruder_id in &layer_extruder_count {
                extruder_count[extruder_id as usize] += 1;
            }
        }

        // set key for most used extruder
        // count most used extruder
        let mut most_used_extruder = 0usize;
        for extruder_id in 1..extruder_count.len() {
            if extruder_count[extruder_id] >= extruder_count[most_used_extruder] {
                most_used_extruder = extruder_id;
            }
        }

        self.most_used_extruder = most_used_extruder as u32;
    }

    /// ToolOrdering.cpp:1472-1486 `ToolOrdering::cal_max_additional_fan`
    pub fn cal_max_additional_fan(&mut self, config: &ToolOrderingPrintConfig) -> f32 {
        let additional = cfg_ints(config, "additional_cooling_fan_speed");
        let optional = cfg_ints(config, "optional_cooling_fan_speed");
        // record
        let mut max_fan: f32 = 0.0;
        for layer_tools in &self.layer_tools {
            let filaments = layer_tools.extruders.clone();
            // count once only
            for &filament in &filaments {
                max_fan = max_fan.max(get_at(&additional, filament as usize) as f32);
                max_fan = max_fan.max(get_at(&optional, filament as usize) as f32);
            }
        }
        max_fan
    }

    /// ToolOrdering.cpp:1489-1546 `ToolOrdering::cal_non_support_filaments`: BBS: find first non
    /// support filament.
    pub fn cal_non_support_filaments(
        &mut self,
        config: &ToolOrderingPrintConfig,
        first_non_support_filament: &mut u32,
        initial_non_support_filaments: &mut Vec<i32>,
        initial_filaments: &mut Vec<i32>,
    ) -> bool {
        let mut find_count = 0i32;
        let mut find_first_filaments_count = 0i32;
        let has_non_support = self.has_non_support_filament(config);

        // When dynamic filament map is enabled, config.filament_map is not updated by the grouping
        // result (update_to_config_by_nozzle_group_result does not write filament_map). Use
        // m_nozzle_group_result to resolve the extruder for each filament instead.
        let use_dynamic_map =
            self.nozzle_group_result.is_support_dynamic_nozzle_map() && self.nozzle_group_result.get_layer_count() > 0;

        let filament_map = cfg_ints(config, "filament_map");
        let filament_is_support = cfg_bools(config, "filament_is_support");
        // Returns the 0-based extruder id for a filament, or -1 when unavailable.
        let get_extruder_for_filament = |nozzle_group_result: &crate::multi_nozzle_utils::LayeredNozzleGroupResult,
                                         filament: u32,
                                         layer_id: i32|
         -> i32 {
            if use_dynamic_map {
                return nozzle_group_result.get_extruder_id(filament as i32, layer_id);
            }
            if !filament_map.is_empty() {
                return filament_map[filament as usize] - 1; // filament_map is 1-based
            }
            -1
        };

        let mut layer_idx = 0i32;
        for layer_tool in &self.layer_tools {
            for &filament in &layer_tool.extruders {
                let extruder_id =
                    get_extruder_for_filament(&self.nozzle_group_result, filament, if use_dynamic_map { layer_idx } else { -1 });

                // check first filament
                if extruder_id >= 0
                    && extruder_id < initial_filaments.len() as i32
                    && initial_filaments[extruder_id as usize] == -1
                {
                    initial_filaments[extruder_id as usize] = filament as i32;
                    find_first_filaments_count += 1;
                }

                if has_non_support {
                    // check first non support filaments
                    if get_at(&filament_is_support, filament as usize) != 0 {
                        continue;
                    }

                    if *first_non_support_filament == u32::MAX {
                        *first_non_support_filament = filament;
                    }

                    // params missing, add protection
                    // filament map missing means single nozzle, no need to set initial_non_support_filaments
                    if !use_dynamic_map && filament_map.is_empty() {
                        return true;
                    }

                    if extruder_id >= 0
                        && extruder_id < initial_non_support_filaments.len() as i32
                        && initial_non_support_filaments[extruder_id as usize] == -1
                    {
                        initial_non_support_filaments[extruder_id as usize] = filament as i32;
                        find_count += 1;
                    }

                    if find_count == initial_non_support_filaments.len() as i32 {
                        return true;
                    }
                } else if find_first_filaments_count == initial_filaments.len() as i32
                    || (!use_dynamic_map && filament_map.is_empty())
                {
                    return false;
                }
            }
            layer_idx += 1;
        }

        false
    }

    /// ToolOrdering.cpp:1548-1556 `ToolOrdering::has_non_support_filament`
    pub fn has_non_support_filament(&mut self, config: &ToolOrderingPrintConfig) -> bool {
        let filament_is_support = cfg_bools(config, "filament_is_support");
        for &filament in &self.all_printing_extruders {
            if get_at(&filament_is_support, filament as usize) == 0 {
                return true;
            }
        }

        false
    }
}

/// `namespace GroupReorder` (ToolOrdering.hpp:141-162, ToolOrdering.cpp:1559-2202).
pub mod group_reorder {
    use super::{cfg_bool, cfg_bools, cfg_float, cfg_floats, cfg_int, cfg_ints, cfg_strings, get_at, ToolOrderingPrint, ToolOrderingPrintConfig};
    use std::collections::{BTreeMap, BTreeSet};
    use crate::multi_nozzle_utils::{NozzleGroupInfo, NozzleInfo};

    /// ToolOrdering.hpp:143 `using FlushMatrix = std::vector<std::vector<float>>;`
    pub type FlushMatrix = Vec<Vec<f32>>;

    /// `std::function<bool(int, std::vector<int>&)>` of the custom sequence.
    pub type CustomSeqFunction = Box<dyn Fn(i32, &mut Vec<i32>) -> bool>;

    /// ToolOrdering.cpp:1560-1585 `create_custom_seq_function`
    pub fn create_custom_seq_function(
        print_config: &ToolOrderingPrintConfig,
        include_first_layer: bool,
        first_layer_filaments: &[u32],
    ) -> CustomSeqFunction {
        let other_layers_seqs = crate::parameter_utils::get_other_layers_print_sequence(
            cfg_int(print_config, "other_layers_print_sequence_nums"),
            &cfg_ints(print_config, "other_layers_print_sequence"),
        );
        let first_layer_filaments = first_layer_filaments.to_vec();

        Box::new(move |layer_idx: i32, out_seq: &mut Vec<i32>| -> bool {
            if include_first_layer && layer_idx == 0 && !first_layer_filaments.is_empty() {
                out_seq.resize(first_layer_filaments.len(), 0);
                for (o, &v) in out_seq.iter_mut().zip(first_layer_filaments.iter()) {
                    *o = v as i32 + 1;
                }
                return true;
            }

            let mut idx = other_layers_seqs.len().wrapping_sub(1);
            while idx != usize::MAX {
                let other_layers_seq = &other_layers_seqs[idx];
                if layer_idx + 1 >= other_layers_seq.0 .0 && layer_idx + 1 <= other_layers_seq.0 .1 {
                    *out_seq = other_layers_seq.1.clone();
                    return true;
                }
                idx = idx.wrapping_sub(1);
            }
            false
        })
    }

    /// ToolOrdering.cpp:1588-1613 `build_nozzle_groups`
    pub fn build_nozzle_groups(print_config: &ToolOrderingPrintConfig, extruder_nums: usize) -> Vec<NozzleGroupInfo> {
        use crate::extruder::NozzleVolumeType;
        let mut nozzle_groups: Vec<NozzleGroupInfo> = Vec::new();
        let extruder_nozzle_counts = crate::print_config::get_extruder_nozzle_stats(&cfg_strings(print_config, "extruder_nozzle_stats"));
        let nozzle_volume_types = cfg_ints(print_config, "nozzle_volume_type");
        let nozzle_diameter = cfg_floats(print_config, "nozzle_diameter");
        let extruder_max_nozzle_count = cfg_ints(print_config, "extruder_max_nozzle_count");

        for idx in 0..extruder_nums {
            if idx >= extruder_nozzle_counts.len() || extruder_nozzle_counts[idx].is_empty() {
                nozzle_groups.push(NozzleGroupInfo::new(
                    crate::utils::format_diameter_to_str_default(nozzle_diameter[idx]),
                    NozzleVolumeType::from_i32(nozzle_volume_types[idx]),
                    idx as i32,
                    extruder_max_nozzle_count[idx],
                ));
            } else {
                let type_ = NozzleVolumeType::from_i32(nozzle_volume_types[idx]);
                if type_ == NozzleVolumeType::NvtHybrid {
                    for (&volume_type, &count) in &extruder_nozzle_counts[idx] {
                        nozzle_groups.push(NozzleGroupInfo::new(
                            crate::utils::format_diameter_to_str_default(nozzle_diameter[idx]),
                            volume_type,
                            idx as i32,
                            count,
                        ));
                    }
                } else {
                    // `extruder_nozzle_counts[idx][type]`: operator[] gives 0 for a missing key.
                    let count = extruder_nozzle_counts[idx].get(&type_).copied().unwrap_or(0);
                    nozzle_groups.push(NozzleGroupInfo::new(
                        crate::utils::format_diameter_to_str_default(nozzle_diameter[idx]),
                        type_,
                        idx as i32,
                        count,
                    ));
                }
            }
        }
        nozzle_groups
    }

    /// ToolOrdering.cpp:1615-1692 `build_manual_nozzle_map`: N1X manual modes: AMS-group filaments use
    /// AMS; fixed-group filaments use compatible fixed nozzles or appended virtual nozzles.
    #[allow(clippy::too_many_arguments)]
    pub fn build_manual_nozzle_map(
        print_config: &ToolOrderingPrintConfig,
        nozzle_list: &mut Vec<NozzleInfo>,
        used_filaments: &[u32],
        mode: crate::print_config::enums::FilamentMapMode,
        ams_nozzle_idx: i32,
        ams_capacity: i32,
        filament_nums: usize,
    ) -> Vec<i32> {
        use crate::print_config::enums::FilamentMapMode;
        let mut fnmap: Vec<i32> = vec![if ams_nozzle_idx >= 0 { ams_nozzle_idx } else { 0 }; filament_nums];
        let ufnm = cfg_ints(print_config, "filament_nozzle_map");
        let fvmap = cfg_ints(print_config, "filament_volume_map");
        let selected_diameters = cfg_strings(print_config, "filament_diameter_map");

        // nvtHybrid == 2 (PrintConfig.hpp:385)
        let compatible = |nozzle_vt: i32, filament_vt: i32| -> bool { nozzle_vt == 2 || filament_vt == 2 || nozzle_vt == filament_vt };

        let in_ams_group = |f: u32| -> bool {
            if ams_nozzle_idx < 0 {
                return false;
            }
            if mode == FilamentMapMode::FmmManual {
                return (f as usize) < ufnm.len() && ufnm[f as usize] == ams_nozzle_idx;
            }
            (f as usize) < ufnm.len() && ufnm[f as usize] == ams_nozzle_idx
        };

        let mut used_fixed: BTreeSet<i32> = BTreeSet::new(); // fixed nozzle group_ids already taken
        let mut ams_group_count = 0;
        let nozzle_diameter = cfg_floats(print_config, "nozzle_diameter");
        let virtual_diameter: String = if nozzle_diameter.is_empty() {
            String::new()
        } else {
            crate::utils::format_diameter_to_str_default(nozzle_diameter[0])
        };

        for &f in used_filaments {
            let fu = f as usize;
            if in_ams_group(f) {
                if fu < selected_diameters.len()
                    && !selected_diameters[fu].is_empty()
                    && (ams_nozzle_idx < 0
                        || ams_nozzle_idx >= nozzle_list.len() as i32
                        || nozzle_list[ams_nozzle_idx as usize].diameter != selected_diameters[fu])
                {
                    return Vec::new();
                }
                fnmap[fu] = ams_nozzle_idx;
                ams_group_count += 1;
                continue;
            }

            // nvtStandard == 0, nvtCount == 6 (PrintConfig.hpp:383-389)
            let mut req_vol = if fu < fvmap.len() { fvmap[fu] } else { 0 };
            if req_vol < 0 || req_vol >= 6 {
                req_vol = 0;
            }
            let mut chosen = -1;
            for n in nozzle_list.iter() {
                if !n.installed || n.group_id == ams_nozzle_idx || used_fixed.contains(&n.group_id) {
                    continue;
                }
                if fu < selected_diameters.len() && !selected_diameters[fu].is_empty() && n.diameter != selected_diameters[fu] {
                    continue;
                }
                if compatible(n.volume_type as i32, req_vol) {
                    chosen = n.group_id;
                    break;
                }
            }
            if chosen >= 0 {
                used_fixed.insert(chosen);
                fnmap[fu] = chosen;
            } else {
                let mut virtual_nozzle = NozzleInfo::default();
                virtual_nozzle.diameter = if fu < selected_diameters.len() && !selected_diameters[fu].is_empty() {
                    selected_diameters[fu].clone()
                } else {
                    virtual_diameter.clone()
                };
                // NozzleVolumeType(req_vol): FIDELITY-NOTE (crate::extruder's enum of an earlier pin)
                virtual_nozzle.volume_type = crate::extruder::NozzleVolumeType::from_i32(req_vol);
                virtual_nozzle.extruder_id = 0;
                virtual_nozzle.group_id = nozzle_list.len() as i32;
                virtual_nozzle.installed = true;
                chosen = virtual_nozzle.group_id;
                nozzle_list.push(virtual_nozzle);
                used_fixed.insert(chosen);
                fnmap[fu] = chosen;
            }
        }

        if ams_nozzle_idx >= 0 && ams_capacity > 0 && ams_group_count > ams_capacity {
            return Vec::new(); // AMS group exceeds AMS slot count
        }

        fnmap
    }

    /// ToolOrdering.cpp:1694-1708 `build_default_nozzle_list`
    pub fn build_default_nozzle_list(print_config: &ToolOrderingPrintConfig, extruder_nums: usize) -> Vec<NozzleInfo> {
        let nozzle_diameter = cfg_floats(print_config, "nozzle_diameter");
        let nozzle_volume_type = cfg_ints(print_config, "nozzle_volume_type");
        let mut nozzle_list = Vec::new();
        for idx in 0..extruder_nums {
            let mut tmp = NozzleInfo::default();
            tmp.diameter = crate::utils::format_diameter_to_str_default(nozzle_diameter[idx]);
            tmp.group_id = idx as i32;
            tmp.extruder_id = idx as i32;
            tmp.volume_type = crate::extruder::NozzleVolumeType::from_i32(nozzle_volume_type[idx]);

            nozzle_list.push(tmp);
        }
        nozzle_list
    }

    /// ToolOrdering.cpp:1710-2173 `build_filament_group_context`
    #[allow(clippy::too_many_arguments)]
    pub fn build_filament_group_context(
        print: &dyn ToolOrderingPrint,
        layer_filaments: &[Vec<u32>],
        physical_unprintables: &[BTreeSet<i32>],
        geometric_unprintables: &[BTreeSet<i32>],
        unprintable_volumes: &BTreeMap<i32, BTreeSet<crate::extruder::NozzleVolumeType>>,
        mode: crate::print_config::enums::FilamentMapMode,
        nozzle_status: &crate::multi_nozzle_utils::UnorderedMapInt<i32>,
    ) -> Result<crate::filament_group::FilamentGroupContext, crate::filament_group::FilamentGroupError> {
        use crate::filament_group::{collect_sorted_used_filaments, FGMode, FGStrategy, FilamentGroupContext, FilamentGroupError};
        use crate::filament_group_utils::{build_extruder_nozzle_list, build_machine_filaments, calc_max_group_size, collect_unprintable_limits, Color, FilamentInfo, FilamentUsageType};
        use crate::print_config::config_def;
        use crate::print_config::enums::FilamentMapMode;
        use crate::utils::format_diameter_to_str_default as format_diameter_to_str;

        let runtime = |msg: &str| FilamentGroupError::Runtime(crate::exception::RuntimeError::new(msg.to_string()));

        let mut context = FilamentGroupContext::default();

        let print_config = print.config();
        let filament_nums = cfg_strings(print_config, "filament_colour").len();
        let extruder_nums = cfg_floats(print_config, "nozzle_diameter").len();
        let has_multiple_nozzle = cfg_ints(print_config, "extruder_max_nozzle_count").iter().any(|&v| v > 1);

        let nozzle_flush_mtx = prepare_flush_matrices(print_config);

        let ams_nozzle_idx_ctx = config_def::get_effective_ams_nozzle_idx(print_config);
        // Single-extruder multi-nozzle (N1X): one physical extruder driving several nozzles. AMS presence
        // is a separate runtime fact: ams_nozzle_idx_ctx >= 0 means the AMS-fed nozzle exists at that slot.
        let single_ext_multi_nozzle = config_def::is_single_ext_multi_nozzle(print_config);

        // 收集不可打印限制
        let mut ext_unprintable_filaments: Vec<BTreeSet<i32>> = vec![BTreeSet::new(); 2];
        collect_unprintable_limits(physical_unprintables, geometric_unprintables, &mut ext_unprintable_filaments);

        // PA calibration on machines with a Bowden extruder: force the calibration filament onto the
        // user-selected extruder by marking it as unprintable on every other extruder.
        {
            let calib = print.calib_params();
            if calib.has_bowden_extruder && calib.extruder_id >= 0 && calib.extruder_id < ext_unprintable_filaments.len() as i32 {
                let is_manual = cfg_int(print_config, "filament_map_mode") == FilamentMapMode::FmmManual as i32;
                if !is_manual {
                    let used_filaments_tmp = collect_sorted_used_filaments(layer_filaments);
                    for ext in 0..ext_unprintable_filaments.len() as i32 {
                        if ext == calib.extruder_id {
                            continue;
                        }
                        for &f in &used_filaments_tmp {
                            ext_unprintable_filaments[ext as usize].insert(f as i32);
                        }
                    }
                }
            }
        }

        // 生成机器信息
        let ignore_ext_filament = false;
        let extruder_ams_count_str = cfg_strings(print_config, "extruder_ams_count");
        let extruder_ams_counts = config_def::get_extruder_ams_count(&extruder_ams_count_str);
        let mut group_size = calc_max_group_size(&extruder_ams_counts, ignore_ext_filament);

        // Single real extruder model: AMS-head capacity = external AMS slots + the native Ext slot
        // (1 when no AMS). Captured here before group_size is collapsed to a single group.
        let mut ams_nozzle_capacity = 1;
        if single_ext_multi_nozzle {
            if ams_nozzle_idx_ctx >= 0 {
                let ams_slots = if group_size.is_empty() { 0 } else { group_size[0] };
                ams_nozzle_capacity = std::cmp::max(1, ams_slots + 1);
            }
            // One real extruder holding all filaments; the per-nozzle capacity limits each physical nozzle.
            group_size = vec![filament_nums as i32];
        }

        // When filament switcher is connected, disable AMS capacity limit for grouping
        if cfg_bool(print_config, "has_filament_switcher") {
            let total_filaments = filament_nums as i32;
            for s in group_size.iter_mut() {
                *s = std::cmp::max(*s, total_filaments);
            }
        }

        // Single real extruder for N1X: size by the real extruder count and skip the per-extruder Bowden preference.
        let mut prefer_non_model_filament: Vec<bool> = vec![false; extruder_nums];
        if !single_ext_multi_nozzle {
            let extruder_type = cfg_ints(print_config, "extruder_type");
            for idx in 0..extruder_nums {
                // ExtruderType::etBowden == 1 (PrintConfig.hpp:376-380)
                prefer_non_model_filament[idx] = extruder_type[idx] == 1;
            }
        }

        let machine_filament_info = build_machine_filaments(print.get_extruder_filament_info(), &extruder_ams_counts, ignore_ext_filament);

        let filament_types = cfg_strings(print_config, "filament_type");
        let filament_colours = cfg_strings(print_config, "filament_colour");
        let filament_is_support = cfg_bools(print_config, "filament_is_support");
        let filament_ids = cfg_strings(print_config, "filament_ids");
        let filament_usage_types: Vec<FilamentUsageType> = print.get_filament_usage_type();

        let fg_mode = if mode == FilamentMapMode::FmmAutoForMatch { FGMode::MatchMode } else { FGMode::FlushMode };
        // N1X uses a single real extruder with one real flush matrix.
        context.model_info.flush_matrix = nozzle_flush_mtx;
        context.model_info.unprintable_filaments = ext_unprintable_filaments.clone();
        context.model_info.layer_filaments = layer_filaments.to_vec();
        context.model_info.filament_ids = filament_ids;
        context.model_info.selected_nozzle_diameters = cfg_strings(print_config, "filament_diameter_map");
        context.model_info.selected_nozzle_diameters.resize(filament_nums, String::new());
        context.model_info.unprintable_volumes = unprintable_volumes.clone();

        let filament_diameter = cfg_floats(print_config, "filament_diameter");
        for idx in 0..filament_types.len() {
            let mut info = FilamentInfo::default();
            info.color = Color::from_hex_str(&filament_colours[idx]);
            info.type_ = filament_types[idx].clone();
            info.is_support = filament_is_support[idx] != 0;
            info.usage_type = filament_usage_types[idx];
            if idx < filament_diameter.len() {
                info.diameter = get_at(&filament_diameter, idx) as f32;
            }
            context.model_info.filament_info.push(info);
        }

        // = print->get_filament_print_time(): the move assignment of the returned copy.
        context.speed_info.filament_print_time = print.get_filament_print_time();
        context.speed_info.group_with_time = cfg_bool(print_config, "group_algo_with_time");
        context.speed_info.filament_change_time =
            cfg_float(print_config, "machine_load_filament_time") + cfg_float(print_config, "machine_unload_filament_time");
        context.speed_info.extruder_change_time = cfg_float(print_config, "machine_switch_extruder_time");
        {
            let load_time = cfg_float(print_config, "machine_load_filament_time");
            let unload_time = cfg_float(print_config, "machine_unload_filament_time");
            context.speed_info.change_time_params.standard_load_time = load_time as f32;
            context.speed_info.change_time_params.standard_unload_time = unload_time as f32;
            context.speed_info.change_time_params.selector_load_time = (load_time / 2.0) as f32;
            context.speed_info.change_time_params.selector_unload_time = (unload_time / 2.0) as f32;
        }

        context.machine_info.machine_filament_info = machine_filament_info;
        context.machine_info.nozzle_loaded_filaments = print.get_nozzle_loaded_filament_info().to_vec();
        // = std::move(group_size): the moved-from vector is left empty (libc++), and it is read below.
        context.machine_info.max_group_size = std::mem::take(&mut group_size);
        context.machine_info.master_extruder_id = cfg_int(print_config, "master_extruder_id") - 1;
        context.machine_info.prefer_non_model_filament = prefer_non_model_filament.clone();

        context.group_info.total_filament_num = filament_nums as i32;
        context.group_info.max_gap_threshold = 0.01;
        context.group_info.strategy = FGStrategy::BestCost;
        context.group_info.mode = fg_mode;
        context.group_info.ignore_ext_filament = ignore_ext_filament;
        context.group_info.has_filament_switcher = cfg_bool(print_config, "has_filament_switcher");
        let used_filaments = collect_sorted_used_filaments(layer_filaments);

        // On single-extruder multi-nozzle machines with multi-diameter mode enabled, filament_volume_map is
        // the flow the user picked (as in fmmManual) and acts as a hard constraint. Otherwise auto modes
        // stay on Hybrid.
        let user_selected_nozzle_flow = mode == FilamentMapMode::FmmManual
            || (single_ext_multi_nozzle && config_def::multi_nozzle_diameter_mode_enabled(print.full_print_config()));
        if user_selected_nozzle_flow {
            context.group_info.filament_volume_map = cfg_ints(print_config, "filament_volume_map");
            // Hybrid (2) is the right padding: it means "no request".
            context.group_info.filament_volume_map.resize(filament_nums, 2);
        } else {
            // Hybrid means no requested nozzle volume type in auto modes.
            context.group_info.filament_volume_map = vec![2; filament_nums];
        }

        // Build the global nozzle list. For ALL machine types, prefer the per-slot nozzle params.
        {
            let ordered = crate::multi_nozzle_utils::build_nozzle_list_from_extruder_nozzles(print_config);
            // For N1X, the AMS slot must actually carry a nozzle; otherwise the per-slot data is invalid.
            let ams_slot_ok = !single_ext_multi_nozzle
                || ams_nozzle_idx_ctx < 0
                || (ams_nozzle_idx_ctx < ordered.len() as i32 && ordered[ams_nozzle_idx_ctx as usize].installed);

            if !ordered.is_empty() && ams_slot_ok {
                context.nozzle_info.nozzle_list = ordered;
            } else if single_ext_multi_nozzle {
                // N1X must always have valid per-slot params; the raw inputs are logged in C++
                // (BOOST_LOG_TRIVIAL(error) << "no valid nozzle." ...).
                return Err(runtime("No valid nozzle found. Please check nozzle count."));
            } else {
                // Multi-extruder fallback: real per-extruder nozzle groups, no remap.
                context.nozzle_info.nozzle_list = crate::multi_nozzle_utils::build_nozzle_list(build_nozzle_groups(print_config, extruder_nums));
            }
        }

        // Keep every authoritative per-slot nozzle available in multi-diameter mode; with the project gate
        // off, only slots matching the active printer preset diameter remain available.
        {
            let extruder_diameters = cfg_floats(print_config, "nozzle_diameter");
            let use_per_slot_diameters = config_def::has_authoritative_per_slot_nozzle_diameters(
                cfg_bool(print_config, "handle_hotend_as_extruder"),
                &cfg_ints(print_config, "extruder_nozzle_count"),
                &cfg_floats(print_config, "extruder_nozzle_diameter"),
            );
            let nozzle_list = &mut context.nozzle_info.nozzle_list;
            if !config_def::multi_nozzle_diameter_mode_enabled(print.full_print_config()) {
                let mut active_diameter = config_def::primary_nozzle_diameter(print.full_print_config());
                if active_diameter <= crate::libslic3r::EPSILON && !extruder_diameters.is_empty() {
                    active_diameter = extruder_diameters[0];
                }
                let active_diameter_str = format_diameter_to_str(active_diameter);
                for nozzle in nozzle_list.iter_mut() {
                    if nozzle.diameter != active_diameter_str {
                        nozzle.installed = false;
                    }
                }
            } else if !use_per_slot_diameters {
                for nozzle in nozzle_list.iter_mut() {
                    if nozzle.extruder_id < 0
                        || nozzle.extruder_id >= extruder_diameters.len() as i32
                        || nozzle.diameter != format_diameter_to_str(extruder_diameters[nozzle.extruder_id as usize])
                    {
                        nozzle.installed = false; //wrong-diameter nozzle: not usable for this print
                    }
                }
            }

            let mut ams_logical_group_id = -1;
            if single_ext_multi_nozzle
                && ams_nozzle_idx_ctx >= 0
                && ams_nozzle_idx_ctx < nozzle_list.len() as i32
                && nozzle_list[ams_nozzle_idx_ctx as usize].installed
            {
                ams_logical_group_id = ams_nozzle_idx_ctx;
            }

            // N1X 非 match 自动模式需要在 ctx 阶段补足基础容量；fmmNozzleManual under multi-diameter mode
            // joins the auto modes here.
            let needs_virtual_nozzle_padding = single_ext_multi_nozzle
                && ((config_def::is_auto_filament_map_mode(mode) && mode != FilamentMapMode::FmmAutoForMatch)
                    || (mode == FilamentMapMode::FmmNozzleManual && config_def::multi_nozzle_diameter_mode_enabled(print.full_print_config())));
            if needs_virtual_nozzle_padding {
                let virtual_diameter: String =
                    if extruder_diameters.is_empty() { String::new() } else { format_diameter_to_str(extruder_diameters[0]) };

                let ams_capacity = std::cmp::max(1, ams_nozzle_capacity);

                let nozzle_capacity = |nozzle: &crate::multi_nozzle_utils::NozzleInfo| -> i32 {
                    if !nozzle.installed {
                        return 0;
                    }
                    let is_ams_nozzle = ams_logical_group_id >= 0 && nozzle.group_id == ams_logical_group_id;
                    if is_ams_nozzle {
                        ams_capacity
                    } else {
                        1
                    }
                };

                // 统计当前真实/有效入口数。
                let usable_slot_count = |nozzle_list: &Vec<crate::multi_nozzle_utils::NozzleInfo>| -> i32 {
                    let mut count = 0;
                    for nozzle in nozzle_list {
                        count += nozzle_capacity(nozzle);
                    }
                    count
                };

                // Clone a real nozzle so the virtual slot inherits its diameter / flow / extruder_id and only
                // takes a new group_id.
                let append_virtual_nozzle = |nozzle_list: &mut Vec<crate::multi_nozzle_utils::NozzleInfo>,
                                             mut virtual_nozzle: crate::multi_nozzle_utils::NozzleInfo| {
                    virtual_nozzle.group_id = nozzle_list.len() as i32;
                    virtual_nozzle.installed = true;
                    nozzle_list.push(virtual_nozzle);
                };

                // Per-spec padding: a filament that requested a concrete (diameter, flow) needs a capacity
                // unit of exactly that spec.
                {
                    let mut spec_demand: BTreeMap<(String, i32), i32> = BTreeMap::new();
                    for &fil in &used_filaments {
                        if fil as usize >= filament_nums {
                            continue;
                        }
                        let diameter = &context.model_info.selected_nozzle_diameters[fil as usize];
                        let volume = context.group_info.filament_volume_map[fil as usize];
                        if diameter.is_empty() || volume == 2 {
                            continue;
                        }
                        *spec_demand.entry((diameter.clone(), volume)).or_insert(0) += 1;
                    }

                    let mut spec_supply: BTreeMap<(String, i32), i32> = BTreeMap::new();
                    let mut spec_source: BTreeMap<(String, i32), crate::multi_nozzle_utils::NozzleInfo> = BTreeMap::new();
                    for nozzle in nozzle_list.iter() {
                        let capacity = nozzle_capacity(nozzle);
                        if capacity <= 0 {
                            continue;
                        }
                        let spec = (nozzle.diameter.clone(), nozzle.volume_type as i32);
                        *spec_supply.entry(spec.clone()).or_insert(0) += capacity;
                        spec_source.entry(spec).or_insert_with(|| nozzle.clone());
                    }

                    for (spec, &demand) in spec_demand.iter() {
                        // No installed nozzle of that spec: leave it to the nozzle assignment to report.
                        let Some(source) = spec_source.get(spec) else {
                            continue;
                        };
                        let mut supplied = *spec_supply.entry(spec.clone()).or_insert(0);
                        while supplied < demand {
                            append_virtual_nozzle(nozzle_list, source.clone());
                            supplied += 1;
                        }
                    }
                }

                // 简化兜底：非 match 自动模式沿用 Standard 虚拟喷嘴补齐。
                while usable_slot_count(nozzle_list) < used_filaments.len() as i32 {
                    let mut virtual_nozzle = crate::multi_nozzle_utils::NozzleInfo::default();
                    virtual_nozzle.diameter = virtual_diameter.clone();
                    virtual_nozzle.volume_type = crate::extruder::NozzleVolumeType::NvtStandard;
                    virtual_nozzle.extruder_id = 0;
                    virtual_nozzle.group_id = nozzle_list.len() as i32;
                    virtual_nozzle.installed = true;
                    nozzle_list.push(virtual_nozzle);
                }
            }

            // Per-nozzle filament capacity (indexed by nozzle group_id) for every machine.
            let mut caps: Vec<i32> = vec![1; nozzle_list.len()];
            for i in 0..nozzle_list.len() {
                let nz = &nozzle_list[i];
                if !nz.installed {
                    caps[i] = 0;
                } else if single_ext_multi_nozzle {
                    caps[i] = if ams_logical_group_id >= 0 && nz.group_id == ams_logical_group_id { std::cmp::max(1, ams_nozzle_capacity) } else { 1 };
                } else {
                    // group_size was moved into machine_info above: empty here.
                    caps[i] = if nz.extruder_id >= 0 && nz.extruder_id < group_size.len() as i32 {
                        std::cmp::max(1, group_size[nz.extruder_id as usize])
                    } else {
                        1
                    };
                }
            }
            context.nozzle_info.per_nozzle_capacity = caps;
        }

        context.nozzle_info.extruder_nozzle_list = build_extruder_nozzle_list(&context.nozzle_info.nozzle_list);

        if context.nozzle_info.nozzle_list.is_empty() {
            return Err(runtime("No valid nozzle found. Please check nozzle count."));
        }

        if !nozzle_status.is_empty() {
            context.nozzle_info.nozzle_status.assign_from(nozzle_status);
        }

        // add_volume_type_limits, only for o1d
        if !has_multiple_nozzle {
            let mut ext_unprintable_filaments_with_volume = ext_unprintable_filaments.clone();

            for nozzle in &context.nozzle_info.nozzle_list {
                for &fil_id in &used_filaments {
                    // context.model_info.unprintable_volumes[fil_id] (std::map operator[]: inserts)
                    let unprintable_volumes = context.model_info.unprintable_volumes.entry(fil_id as i32).or_default().clone();
                    if unprintable_volumes.contains(&nozzle.volume_type) {
                        ext_unprintable_filaments_with_volume[nozzle.extruder_id as usize].insert(fil_id as i32);
                    }
                }
            }

            for &fil_id in &used_filaments {
                let f = fil_id as i32;
                if ext_unprintable_filaments_with_volume[0].contains(&f) && ext_unprintable_filaments_with_volume[1].contains(&f) {
                    ext_unprintable_filaments_with_volume[0].remove(&f);
                    ext_unprintable_filaments_with_volume[1].remove(&f);
                }
            }
            context.model_info.unprintable_filaments = ext_unprintable_filaments_with_volume;
        }

        if cfg_int(print_config, "filament_map_mode") == FilamentMapMode::FmmAutoForQuality as i32 {
            let mut prefer_support_extruders: Vec<i32> = Vec::new();
            let mut non_prefer_support_extruders: Vec<i32> = Vec::new();
            for idx in 0..prefer_non_model_filament.len() {
                if prefer_non_model_filament[idx] {
                    prefer_support_extruders.push(idx as i32);
                } else {
                    non_prefer_support_extruders.push(idx as i32);
                }
            }

            if !prefer_support_extruders.is_empty() {
                let filament_info = &context.model_info.filament_info;
                let unprintable_filaments = &mut context.model_info.unprintable_filaments;
                for &fil_id in &used_filaments {
                    let f = fil_id as i32;
                    let is_support_filament = filament_info[fil_id as usize].usage_type == FilamentUsageType::SupportOnly;
                    if is_support_filament {
                        // 若存在可打印的偏好支撑喷嘴，则限制该支撑料只在偏好喷嘴上打印
                        let mut printable_preferred_extruders: Vec<i32> = Vec::new();
                        for &extruder_id in &prefer_support_extruders {
                            if !unprintable_filaments[extruder_id as usize].contains(&f) {
                                printable_preferred_extruders.push(extruder_id);
                            }
                        }
                        if !printable_preferred_extruders.is_empty() {
                            for &extruder_id in &non_prefer_support_extruders {
                                unprintable_filaments[extruder_id as usize].insert(f);
                            }
                        }
                    } else {
                        // 若存在可打印的非偏好喷嘴，则限制模型料尽量避开偏好支撑喷嘴
                        let mut printable_non_preferred_extruders: Vec<i32> = Vec::new();
                        for &extruder_id in &non_prefer_support_extruders {
                            if !unprintable_filaments[extruder_id as usize].contains(&f) {
                                printable_non_preferred_extruders.push(extruder_id);
                            }
                        }
                        if !printable_non_preferred_extruders.is_empty() {
                            for &extruder_id in &prefer_support_extruders {
                                unprintable_filaments[extruder_id as usize].insert(f);
                            }
                        }
                    }
                }

                if !has_multiple_nozzle && unprintable_filaments.len() >= 2 {
                    for &fil_id in &used_filaments {
                        let f = fil_id as i32;
                        if unprintable_filaments[0].contains(&f) && unprintable_filaments[1].contains(&f) {
                            unprintable_filaments[0].remove(&f);
                            unprintable_filaments[1].remove(&f);
                        }
                    }
                }
            }
        }

        // Multi-diameter restriction: keep each filament on the diameter explicitly selected before slicing.
        {
            let extruder_diameters = cfg_floats(print_config, "nozzle_diameter");
            let mut distinct_diameters: BTreeSet<String> = BTreeSet::new();
            for &d in &extruder_diameters {
                distinct_diameters.insert(format_diameter_to_str(d));
            }

            let selected = cfg_strings(print_config, "filament_diameter_map");
            let unprintable_filaments = &mut context.model_info.unprintable_filaments;
            let has_selected = selected.iter().any(|value| !value.is_empty());
            let use_per_slot_diameters = config_def::has_authoritative_per_slot_nozzle_diameters(
                cfg_bool(print_config, "handle_hotend_as_extruder"),
                &cfg_ints(print_config, "extruder_nozzle_count"),
                &cfg_floats(print_config, "extruder_nozzle_diameter"),
            );
            if !use_per_slot_diameters && (distinct_diameters.len() > 1 || has_selected) && extruder_nums >= 2 {
                for &fil_id in &used_filaments {
                    if fil_id as usize >= selected.len() || selected[fil_id as usize].is_empty() {
                        continue;
                    }
                    let allowed = format_diameter_to_str(crate::locales_utils::string_to_double_decimal_point(&selected[fil_id as usize]).0);

                    let mut incompatible_extruders: Vec<i32> = Vec::new();
                    let mut e = 0usize;
                    while e < extruder_nums && e < unprintable_filaments.len() {
                        if allowed != format_diameter_to_str(extruder_diameters[e]) {
                            incompatible_extruders.push(e as i32);
                        }
                        e += 1;
                    }
                    for &e in &incompatible_extruders {
                        unprintable_filaments[e as usize].insert(fil_id as i32);
                    }
                }
            }
        }

        Ok(context)
    }

    /// ToolOrdering.cpp:2176-2200 `prepare_flush_matrices`
    pub fn prepare_flush_matrices(print_config: &ToolOrderingPrintConfig) -> Vec<FlushMatrix> {
        let extruder_nums = cfg_floats(print_config, "nozzle_diameter").len();
        let filament_nums = cfg_strings(print_config, "filament_colour").len();
        let flush_volumes_matrix = cfg_floats(print_config, "flush_volumes_matrix");
        let mut nozzle_flush_mtx: Vec<FlushMatrix> = Vec::new();
        for nozzle_id in 0..extruder_nums {
            let flush_matrix: Vec<f32> = get_flush_volumes_matrix(&flush_volumes_matrix, nozzle_id, extruder_nums)
                .into_iter()
                .map(|v| v as f32)
                .collect();
            let mut wipe_volumes: Vec<Vec<f32>> = Vec::new();
            for i in 0..filament_nums {
                wipe_volumes.push(flush_matrix[i * filament_nums..(i + 1) * filament_nums].to_vec());
            }

            nozzle_flush_mtx.push(wipe_volumes);
        }

        // PrimeVolumeMode::pvmFast
        let mut flush_multiplies = if cfg_int(print_config, "prime_volume_mode") == PVM_FAST {
            cfg_floats(print_config, "flush_multiplier_fast")
        } else {
            cfg_floats(print_config, "flush_multiplier")
        };
        flush_multiplies.resize(extruder_nums, 1.0);
        for nozzle_id in 0..extruder_nums {
            for vec in nozzle_flush_mtx[nozzle_id].iter_mut() {
                for v in vec.iter_mut() {
                    // `float *= double`: the product in double, stored as float.
                    *v = (*v as f64 * flush_multiplies[nozzle_id]) as f32;
                }
            }
        }

        nozzle_flush_mtx
    }

    /// PrintConfig.hpp `PrimeVolumeMode::pvmFast`
    const PVM_FAST: i32 = 2;

    /// PrintConfig.hpp:2176-2184 `get_flush_volumes_matrix`
    fn get_flush_volumes_matrix<T: Clone>(fv_matrix: &[T], extruder_id: usize, nozzle_nums: usize) -> Vec<T> {
        if extruder_id != usize::MAX && nozzle_nums != 1 {
            let from = ((fv_matrix.len() / nozzle_nums * extruder_id) as f64 + crate::libslic3r::EPSILON) as usize;
            let to = ((fv_matrix.len() / nozzle_nums * (extruder_id + 1)) as f64 + crate::libslic3r::EPSILON) as usize;
            return fv_matrix[from..to].to_vec();
        }
        fv_matrix.to_vec()
    }
}

/// ToolOrdering.cpp:2204-2334 `MultiNozzleUtils::LayeredNozzleGroupResult refine_groups_by_Nozzle_State(const FilamentGroupContext& ctx,
/// const MultiNozzleUtils::LayeredNozzleGroupResult& group, const std::unordered_map<int, int> &nozzles_state)`
pub fn refine_groups_by_nozzle_state(
    ctx: &crate::filament_group::FilamentGroupContext,
    group: &crate::multi_nozzle_utils::LayeredNozzleGroupResult,
    nozzles_state: &crate::multi_nozzle_utils::UnorderedMapInt<i32>,
) -> crate::multi_nozzle_utils::LayeredNozzleGroupResult {
    use crate::gcode::tool_order_utils::{MinFlushFlowSolver, INVALID_ID};
    use crate::multi_nozzle_utils::{CreateFromNozzleMap, LayeredNozzleGroupResult};

    let mut nozzle_fils: Vec<Vec<u32>> = vec![Vec::new(); ctx.nozzle_info.nozzle_list.len()];
    let fils = group.get_used_filaments_layer(0);
    let fil_noz_map = group.get_layer_filament_nozzle_map(0).clone();

    for &fil in &fils {
        nozzle_fils[fil_noz_map[fil as usize] as usize].push(fil);
    }

    // 1、收集每个材料不可使用的喷嘴
    let mut fil_unplaceable_nozs: BTreeMap<u32, BTreeSet<i32>> = BTreeMap::new();
    for &fil in &fils {
        let mut unprintable_volumes: BTreeSet<crate::extruder::NozzleVolumeType> = BTreeSet::new();
        if let Some(v) = ctx.model_info.unprintable_volumes.get(&(fil as i32)) {
            unprintable_volumes = v.clone();
        }
        let expected_volume = ctx.group_info.filament_volume_map[fil as usize];

        for noz in 0..ctx.nozzle_info.nozzle_list.len() {
            let noz_info = &ctx.nozzle_info.nozzle_list[noz];
            let ext_id = noz_info.extruder_id;
            let ext_unprintable_fils = &ctx.model_info.unprintable_filaments[ext_id as usize];
            let diameter_mismatch = (fil as usize) < ctx.model_info.selected_nozzle_diameters.len()
                && !ctx.model_info.selected_nozzle_diameters[fil as usize].is_empty()
                && ctx.model_info.selected_nozzle_diameters[fil as usize] != noz_info.diameter;
            // expected_volume != nvtHybrid (2) && expected_volume != noz_info.volume_type
            if ext_unprintable_fils.contains(&(fil as i32))
                || diameter_mismatch
                || (expected_volume != 2 && expected_volume != noz_info.volume_type as i32)
                || unprintable_volumes.contains(&noz_info.volume_type)
            {
                fil_unplaceable_nozs.entry(fil).or_default().insert(noz as i32);
            }
        }
    }

    // 2、存储全局喷嘴匹配结果 (std::unordered_map<int, int>, only searched)
    let mut global_uv_match: HashMap<i32, i32> = HashMap::new();

    // 3、按挤出机分组处理，每个挤出机单独跑费用流
    for (&ext_id, ext_nozzles) in ctx.nozzle_info.extruder_nozzle_list.iter() {
        if ext_nozzles.is_empty() {
            continue;
        }

        // 3.1、构建该挤出机的 u_nodes 和 v_nodes
        let u_nodes: Vec<i32> = ext_nozzles.clone();
        let v_nodes: Vec<i32> = ext_nozzles.clone();

        // 3.2、构建局部索引映射 (全局喷嘴id -> 局部索引)
        let mut global_to_local: HashMap<i32, i32> = HashMap::new();
        for i in 0..ext_nozzles.len() {
            global_to_local.insert(ext_nozzles[i], i as i32);
        }

        // 3.3、构建该挤出机的费用矩阵
        let mut cost_matrix: Vec<Vec<f32>> = vec![vec![f32::MAX; v_nodes.len()]; u_nodes.len()];
        let mut uv_unlink_limits: HashMap<i32, Vec<i32>> = HashMap::new();

        for local_u in 0..u_nodes.len() {
            let u_node = u_nodes[local_u];
            let mut unlink_v_local: BTreeSet<i32> = BTreeSet::new();
            let u_fils = nozzle_fils[u_node as usize].clone();

            // 收集不可连接的 v_node (转换为局部索引); fil_unplaceable_nozs[fil] (operator[])
            for &fil in &u_fils {
                if let Some(nozs) = fil_unplaceable_nozs.get(&fil) {
                    for unplaceable_noz in nozs {
                        if let Some(&l) = global_to_local.get(unplaceable_noz) {
                            unlink_v_local.insert(l);
                        }
                    }
                }
            }
            *uv_unlink_limits.entry(local_u as i32).or_default() = unlink_v_local.iter().copied().collect();

            // 3.4、计算费用
            for local_v in 0..v_nodes.len() {
                let v_node = v_nodes[local_v];
                let mut cost: f32 = 0.0;
                if unlink_v_local.contains(&(local_v as i32)) {
                    continue;
                }

                // std::optional<unsigned int> v_fil_opt = nozzles_state.at(v_node)
                let v_fil_opt: Option<u32> = nozzles_state.get(v_node).map(|&v| v as u32);

                if v_fil_opt.is_none() || v_fil_opt.unwrap() as usize >= ctx.model_info.filament_info.len() {
                    cost = 0.0;
                } else {
                    let v_fil = v_fil_opt.unwrap() as i32;
                    if u_fils.contains(&(v_fil as u32)) {
                        cost = -1.0;
                    } else {
                        for &u_fil in &u_fils {
                            cost += ctx.model_info.flush_matrix[ext_id as usize][u_fil as usize][v_fil as usize];
                        }
                        if !u_fils.is_empty() {
                            // float /= size_t: the count converted to float
                            cost /= u_fils.len() as f32;
                        }
                    }
                }

                cost_matrix[local_u][local_v] = cost;
            }
        }

        // 3.5、使用费用流求解该挤出机的喷嘴匹配
        let local_u_nodes: Vec<i32> = (0..u_nodes.len() as i32).collect();
        let local_v_nodes: Vec<i32> = (0..v_nodes.len() as i32).collect();

        let mut solver = MinFlushFlowSolver::new(&cost_matrix, &local_u_nodes, &local_v_nodes, &HashMap::new(), &uv_unlink_limits, &[], &[], &[]);
        let local_match = solver.solve();

        // 3.6、将局部匹配结果转换为全局匹配
        for local_u in 0..u_nodes.len() {
            let global_u = u_nodes[local_u];
            let local_v = local_match[local_u];
            if local_v == INVALID_ID || local_v < 0 || local_v >= v_nodes.len() as i32 {
                continue;
            }
            let global_v = v_nodes[local_v as usize];
            global_uv_match.insert(global_u, global_v);
        }
    }

    // 4、构建新的group_result
    let mut new_default_filament_nozzle_maps: Vec<i32> = group.get_layer_filament_nozzle_map(-1).clone();

    for &fil in &fils {
        let ori_noz = new_default_filament_nozzle_maps[fil as usize];
        if let Some(&m) = global_uv_match.get(&ori_noz) {
            new_default_filament_nozzle_maps[fil as usize] = m;
        }
    }

    let new_group = <LayeredNozzleGroupResult as CreateFromNozzleMap>::create(
        &new_default_filament_nozzle_maps,
        &ctx.nozzle_info.nozzle_list,
        &fils,
        group.is_handle_nozzle_as_extruder(),
    );
    match new_group {
        Some(g) => g,
        None => group.clone(),
    }
}

// ToolOrdering.cpp:2336-2345 `struct VectorHash` (the hash of std::vector<unsigned int> keys of the
// combo ranges below): only membership of the map it hashes is read, so a HashMap serves.

/// ToolOrdering.cpp:2347-2463 `std::vector<FilamentPlanRes> plan_filament_mapping_and_order_by_combo_ranges(Print* print,
/// const FilamentGroupContext& ctx, const ToolOrdering::OrderingContext& order_ctx, const FilamentMapMode mode,
/// const std::vector<std::set<int>>& physical_unprintables, const std::vector<std::set<int>>& geometric_unprintables,
/// const std::map<int, std::set<NozzleVolumeType>>& unprintable_volumes, MultiNozzleUtils::NozzleStatusRecorder* io_nozzle_status = nullptr)`
#[allow(clippy::too_many_arguments)]
pub fn plan_filament_mapping_and_order_by_combo_ranges(
    print: &dyn ToolOrderingPrint,
    ctx: &crate::filament_group::FilamentGroupContext,
    order_ctx: &OrderingContext,
    mode: crate::print_config::enums::FilamentMapMode,
    physical_unprintables: &[BTreeSet<i32>],
    geometric_unprintables: &[BTreeSet<i32>],
    unprintable_volumes: &BTreeMap<i32, BTreeSet<crate::extruder::NozzleVolumeType>>,
    io_nozzle_status: Option<&mut crate::multi_nozzle_utils::NozzleStatusRecorder>,
) -> Result<Vec<crate::filament_group::FilamentPlanRes>, crate::filament_group::FilamentGroupError> {
    use crate::filament_group::FilamentPlanRes;
    let mut results: Vec<FilamentPlanRes> = Vec::new();

    let layer_fils = &ctx.model_info.layer_filaments;
    if layer_fils.is_empty() {
        return Ok(results);
    }

    results.resize(layer_fils.len(), FilamentPlanRes::default());

    // key: 某一层使用的所有材料id组合 (排序后的vector); value: 具有相同材料组合的连续层区间列表
    let mut filament_combo_ranges: HashMap<Vec<u32>, Vec<(i32, i32)>> = HashMap::new();
    for layer_idx in 0..layer_fils.len() as i32 {
        let mut cur_combo: Vec<u32> = layer_fils[layer_idx as usize].clone();
        cur_combo.sort_unstable();
        cur_combo.dedup();
        if cur_combo.is_empty() {
            continue;
        }

        let ranges = filament_combo_ranges.entry(cur_combo).or_default();
        if ranges.is_empty() || ranges.last().unwrap().1 != layer_idx - 1 {
            ranges.push((layer_idx, layer_idx));
        } else {
            ranges.last_mut().unwrap().1 = layer_idx;
        }
    }

    let mut range_filas_map: BTreeMap<(i32, i32), Vec<u32>> = BTreeMap::new();
    for (combo, ranges) in filament_combo_ranges.iter() {
        for range in ranges {
            range_filas_map.insert(*range, combo.clone());
        }
    }

    let mut used_filaments: BTreeSet<i32> = BTreeSet::new(); // 收集所有使用的材料set

    // 对每个材料组合的每个连续区间，构建新的 layer_filaments 并调用 get_recommended_filament_maps
    let mut tool_status = crate::multi_nozzle_utils::NozzleStatusRecorder::new();
    let io_nozzle_status = io_nozzle_status;
    if let Some(io) = io_nozzle_status.as_deref() {
        tool_status.assign_from(io);
    }

    let mut fil_noz_map: Vec<i32> = vec![-1; ctx.group_info.total_filament_num as usize]; //全局的材料到喷嘴的映射
    // 记录每个材料首次使用的喷嘴id (std::unordered_map<int, int>, only searched)
    let mut fil_first_nozzle_map: HashMap<i32, i32> = HashMap::new();
    for (&(start_layer, end_layer), combo) in range_filas_map.iter() {
        // 1、构建区间的 layer_filaments
        let mut range_layer_fils: Vec<Vec<u32>> = Vec::with_capacity((end_layer - start_layer + 1) as usize);
        for layer_idx in start_layer..=end_layer {
            range_layer_fils.push(layer_fils[layer_idx as usize].clone());
        }
        for &c in combo {
            used_filaments.insert(c as i32);
        }

        // 2、生成区间分组结果
        let nozzle_filament_map = tool_status.get_nozzle_filament_map().clone();
        let group_result = ToolOrdering::get_recommended_filament_maps(
            print,
            &range_layer_fils,
            mode,
            physical_unprintables,
            geometric_unprintables,
            unprintable_volumes,
            &nozzle_filament_map,
        )?;

        // 3、动态调整分组id
        let new_group_result = refine_groups_by_nozzle_state(ctx, &group_result, &nozzle_filament_map);
        let get_custom_seq = &order_ctx.get_custom_seq;
        let range_seq_function = |layer_idx: i32, out_seq: &mut Vec<i32>| -> bool {
            if layer_idx <= end_layer - start_layer {
                let global_idx = start_layer + layer_idx;
                return get_custom_seq(global_idx, out_seq);
            }
            false
        };

        // 4、区间分组排序
        let mut fils_sequences: Vec<Vec<u32>> = Vec::new();
        crate::gcode::tool_order_utils::reorder_filaments_for_multi_nozzle_extruder(
            &range_layer_fils[0],
            &new_group_result,
            &range_layer_fils,
            &ctx.model_info.flush_matrix,
            Some(&range_seq_function),
            Some(&mut fils_sequences),
            &tool_status,
        );

        // 5、存储区间结果
        for &fil_id in fils_sequences.last().expect("fils_sequences.back()") {
            if let Some(noz) = new_group_result.get_nozzle_for_filament(fil_id as i32, -1) {
                let noz_id = noz.group_id;
                let ext_id = noz.extruder_id;

                fil_noz_map[fil_id as usize] = noz_id;

                // 记录材料首次使用的喷嘴id（仅首次插入）
                fil_first_nozzle_map.entry(fil_id as i32).or_insert(noz_id);

                tool_status.set_current_extruder_id(ext_id);
                tool_status.set_nozzle_status(noz_id, fil_id as i32, ext_id);
            }
        }

        // assert(fils_sequences.size() == range_layer_fils.size());
        for layer_id in 0..fils_sequences.len() {
            let g_layer_id = start_layer as usize + layer_id;
            results[g_layer_id].fil_nozzle_match = fil_noz_map.clone();
            results[g_layer_id].fil_order = fils_sequences[layer_id].iter().map(|&f| f as i32).collect();
        }
    }

    for res in results.iter_mut() {
        for fil_id in 0..res.fil_nozzle_match.len() {
            let noz_id = &mut res.fil_nozzle_match[fil_id];
            if *noz_id == -1 {
                *noz_id = if used_filaments.contains(&(fil_id as i32)) && fil_first_nozzle_map.contains_key(&(fil_id as i32)) {
                    fil_first_nozzle_map[&(fil_id as i32)]
                } else {
                    0
                };
            }
        }
    }

    if let Some(io) = io_nozzle_status {
        io.assign_from(&tool_status);
    }

    Ok(results)
}

/// ToolOrdering.cpp:2467-2509 `validate_manual_diameter_or_throw`: multi-diameter manual safety net,
/// reject a manual filament -> extruder assignment that disagrees with the filament's completed
/// selected diameter.
pub fn validate_manual_diameter_or_throw(
    print_config: &ToolOrderingPrintConfig,
    used_filaments: &[u32],
    extruder_map_0based: &[i32],
    multi_nozzle_diameter_mode: bool,
) -> Result<(), crate::exception::RuntimeError> {
    if !multi_nozzle_diameter_mode {
        return Ok(());
    }

    // Per-slot configurations are validated against the selected physical nozzle by the nozzle-level
    // manual grouping path. Their per-extruder diameter is only a representative fallback.
    if crate::print_config::config_def::has_authoritative_per_slot_nozzle_diameters(
        cfg_bool(print_config, "handle_hotend_as_extruder"),
        &cfg_ints(print_config, "extruder_nozzle_count"),
        &cfg_floats(print_config, "extruder_nozzle_diameter"),
    ) {
        return Ok(());
    }

    let extruder_diameters = cfg_floats(print_config, "nozzle_diameter");
    let mut distinct_diameters: BTreeSet<String> = BTreeSet::new();
    for &d in &extruder_diameters {
        distinct_diameters.insert(crate::utils::format_diameter_to_str_default(d));
    }

    let selected = cfg_strings(print_config, "filament_diameter_map");
    let has_selected = selected.iter().any(|value| !value.is_empty());
    if distinct_diameters.len() <= 1 && !has_selected {
        return Ok(());
    }

    for &fid in used_filaments {
        if fid as usize >= selected.len() || selected[fid as usize].is_empty() {
            continue;
        }
        let allowed = crate::utils::format_diameter_to_str_default(
            crate::locales_utils::string_to_double_decimal_point(&selected[fid as usize]).0,
        );
        if fid as usize >= extruder_map_0based.len() {
            continue;
        }
        let e = extruder_map_0based[fid as usize];
        if e < 0 || e >= extruder_diameters.len() as i32 {
            continue;
        }
        if allowed != crate::utils::format_diameter_to_str_default(extruder_diameters[e as usize]) {
            return Err(crate::exception::RuntimeError::new(crate::i18n::translate(
                "Filament assigned to a nozzle of incompatible diameter. Please regroup so each \
                 filament stays on a nozzle whose diameter it supports.",
            )));
        }
    }
    Ok(())
}

impl ToolOrdering {
    /// ToolOrdering.cpp:2511-2690 `static MultiNozzleUtils::LayeredNozzleGroupResult ToolOrdering::get_recommended_filament_maps(Print *print,
    /// const std::vector<std::vector<unsigned int>> &layer_filaments, const FilamentMapMode mode,
    /// const std::vector<std::set<int>> &physical_unprintables, const std::vector<std::set<int>> &geometric_unprintables,
    /// const std::map<int, std::set<NozzleVolumeType>> &unprintable_volumes, const std::unordered_map<int, int>& nozzle_status = {})`.
    ///
    /// Called in single extruder mode, the value in map are all 0; in dual extruder mode 0 or 1 (0 based
    /// group id). The `!print` check of the C++ has no counterpart: `print` is a reference.
    #[allow(clippy::too_many_arguments)]
    pub fn get_recommended_filament_maps(
        print: &dyn ToolOrderingPrint,
        layer_filaments: &[Vec<u32>],
        mode: crate::print_config::enums::FilamentMapMode,
        physical_unprintables: &[BTreeSet<i32>],
        geometric_unprintables: &[BTreeSet<i32>],
        unprintable_volumes: &BTreeMap<i32, BTreeSet<crate::extruder::NozzleVolumeType>>,
        nozzle_status: &crate::multi_nozzle_utils::UnorderedMapInt<i32>,
    ) -> Result<crate::multi_nozzle_utils::LayeredNozzleGroupResult, crate::filament_group::FilamentGroupError> {
        use crate::filament_group::{
            calc_filament_group_for_manual_multi_nozzle, calc_filament_group_for_match_multi_nozzle, calc_filament_group_for_nozzle_match,
            collect_sorted_used_filaments, FilamentGroup, FilamentGroupContext, FilamentGroupError,
        };
        use crate::multi_nozzle_utils::{CreateFromNozzleMap, LayeredNozzleGroupResult};
        use crate::print_config::config_def;
        use crate::print_config::enums::FilamentMapMode;

        let runtime = |msg: &str| FilamentGroupError::Runtime(crate::exception::RuntimeError::new(msg.to_string()));

        if layer_filaments.is_empty() {
            return Ok(LayeredNozzleGroupResult::default());
        }

        let print_config = print.config();
        let multi_diameter_mode = config_def::multi_nozzle_diameter_mode_enabled(print.full_print_config());
        let filament_nums = cfg_strings(print_config, "filament_colour").len();
        let extruder_nums = cfg_floats(print_config, "nozzle_diameter").len();
        let used_filaments = collect_sorted_used_filaments(layer_filaments);
        let has_multiple_nozzle = cfg_ints(print_config, "extruder_max_nozzle_count").iter().any(|&v| v > 1);
        let has_multiple_extruder = extruder_nums > 1;
        let single_ext_multi_nozzle = config_def::is_single_ext_multi_nozzle(print_config);
        let handle_hotend_as_extruder = cfg_bool(print_config, "handle_hotend_as_extruder");

        // N1X fmmManual stores only AMS/fixed grouping; the backend assigns concrete compatible nozzles.
        // fmmNozzleManual remains an explicit filament -> logical nozzle map.
        let build_manual_map_or_throw = |context: &mut FilamentGroupContext| -> Result<Vec<i32>, FilamentGroupError> {
            let mut ams_idx = config_def::get_effective_ams_nozzle_idx(print_config);
            if ams_idx < 0
                || ams_idx >= context.nozzle_info.nozzle_list.len() as i32
                || !context.nozzle_info.nozzle_list[ams_idx as usize].installed
            {
                ams_idx = -1;
            }
            let ams_cap = if ams_idx >= 0 { used_filaments.len() as i32 } else { 0 };
            let map = group_reorder::build_manual_nozzle_map(
                print_config,
                &mut context.nozzle_info.nozzle_list,
                &used_filaments,
                mode,
                ams_idx,
                ams_cap,
                filament_nums,
            );
            if map.is_empty() {
                return Err(runtime(
                    "Nozzle assignment error: no compatible fixed nozzle is available for the fixed filament group. Please check the nozzle setup or regroup.",
                ));
            }
            Ok(map)
        };

        let nozzle_list = group_reorder::build_default_nozzle_list(print_config, extruder_nums);

        // 处理非H2C的手动模式，此时可以直接退出
        if mode == FilamentMapMode::FmmManual && !has_multiple_nozzle {
            let manual_filament_map: Vec<i32> = cfg_ints(print_config, "filament_map").iter().map(|v| v - 1).collect();
            validate_manual_diameter_or_throw(print_config, &used_filaments, &manual_filament_map, multi_diameter_mode).map_err(FilamentGroupError::Runtime)?;
            let result =
                <LayeredNozzleGroupResult as CreateFromNozzleMap>::create(&manual_filament_map, &nozzle_list, &used_filaments, handle_hotend_as_extruder);
            return Ok(result.unwrap_or_default());
        }

        if mode == FilamentMapMode::FmmNozzleManual && has_multiple_nozzle {
            let context = group_reorder::build_filament_group_context(
                print,
                layer_filaments,
                physical_unprintables,
                geometric_unprintables,
                unprintable_volumes,
                mode,
                nozzle_status,
            )?;

            let mut ret: Vec<i32>;
            // Auto flow calibration pins each filament to the nozzle it calibrates through filament_nozzle_map.
            if single_ext_multi_nozzle && multi_diameter_mode && print.calib_params().mode != crate::calib::CalibMode::AutoPALine {
                // Assign from the user's per-filament (diameter, flow): a plain queue assignment over the
                // padded capacity-1 slots, leaving one filament per slot.
                let nozzles = &context.nozzle_info.nozzle_list;
                let mut remaining: Vec<i32> = context.nozzle_info.per_nozzle_capacity.clone();
                remaining.resize(nozzles.len(), 0);
                let ams_idx = config_def::get_effective_ams_nozzle_idx(print_config);

                ret = vec![0; filament_nums];
                for &fil in &used_filaments {
                    let diameter: String = if (fil as usize) < context.model_info.selected_nozzle_diameters.len() {
                        context.model_info.selected_nozzle_diameters[fil as usize].clone()
                    } else {
                        String::new()
                    };
                    // nvtHybrid == 2
                    let volume: i32 = if (fil as usize) < context.group_info.filament_volume_map.len() {
                        context.group_info.filament_volume_map[fil as usize]
                    } else {
                        2
                    };

                    let fits = |noz: i32, remaining: &Vec<i32>| -> bool {
                        let nozzle = &nozzles[noz as usize];
                        if !nozzle.installed || remaining[noz as usize] <= 0 {
                            return false;
                        }
                        if multi_diameter_mode && !diameter.is_empty() && nozzle.diameter != diameter {
                            return false;
                        }
                        volume == 2 || nozzle.volume_type as i32 == volume
                    };

                    // Two passes so the AMS nozzle is only taken once the single-filament slots of that spec
                    // are used up.
                    let mut chosen = -1;
                    let mut noz = 0;
                    while noz < nozzles.len() as i32 && chosen < 0 {
                        if noz != ams_idx && fits(noz, &remaining) {
                            chosen = noz;
                        }
                        noz += 1;
                    }
                    if chosen < 0 && ams_idx >= 0 && ams_idx < nozzles.len() as i32 && fits(ams_idx, &remaining) {
                        chosen = ams_idx;
                    }
                    if chosen < 0 {
                        return Err(runtime(
                            "Nozzle assignment error: no nozzle matches the diameter and flow selected for one of the filaments. Please check the nozzle setup.",
                        ));
                    }

                    ret[fil as usize] = chosen;
                    remaining[chosen as usize] -= 1;
                }
            } else {
                let user_nozzle_map = cfg_ints(print_config, "filament_nozzle_map");
                let nozzle_cnt = context.nozzle_info.nozzle_list.len() as i32;
                ret = vec![0; filament_nums];
                for &fil in &used_filaments {
                    if fil as usize >= user_nozzle_map.len() || user_nozzle_map[fil as usize] < 0 || user_nozzle_map[fil as usize] >= nozzle_cnt {
                        return Err(runtime("Nozzle assignment error: invalid per-filament nozzle map. Please check the nozzle setup or regroup."));
                    }
                    let nozzle_id = user_nozzle_map[fil as usize];
                    if !context.nozzle_info.nozzle_list[nozzle_id as usize].installed {
                        return Err(runtime("Nozzle assignment error: per-filament nozzle map points to an empty or unavailable nozzle."));
                    }
                    ret[fil as usize] = nozzle_id;
                }
            }
            let result_opt =
                <LayeredNozzleGroupResult as CreateFromNozzleMap>::create(&ret, &context.nozzle_info.nozzle_list, &used_filaments, handle_hotend_as_extruder);
            return Ok(result_opt.unwrap_or_default());
        }

        let master_extruder_id = cfg_int(print_config, "master_extruder_id") - 1;
        let mut ret: Vec<i32> = vec![master_extruder_id; filament_nums];

        if has_multiple_extruder || has_multiple_nozzle {
            let mut context = group_reorder::build_filament_group_context(
                print,
                layer_filaments,
                physical_unprintables,
                geometric_unprintables,
                unprintable_volumes,
                mode,
                nozzle_status,
            )?;

            if has_multiple_nozzle {
                if single_ext_multi_nozzle {
                    if mode == FilamentMapMode::FmmManual {
                        ret = build_manual_map_or_throw(&mut context)?;
                    } else if mode == FilamentMapMode::FmmAutoForMatch {
                        ret = calc_filament_group_for_nozzle_match(&mut context)?;
                    } else {
                        // Single-extruder multi-nozzle auto: unified FilamentGroup with per-nozzle capacity.
                        let mut fg = FilamentGroup::new(&context);
                        ret = fg.calc_filament_group(None)?;
                    }
                } else if mode == FilamentMapMode::FmmManual {
                    let manual_filament_map: Vec<i32> = cfg_ints(print_config, "filament_map").iter().map(|v| v - 1).collect();
                    ret = calc_filament_group_for_manual_multi_nozzle(&manual_filament_map, &context)?;
                } else if mode == FilamentMapMode::FmmAutoForMatch {
                    ret = calc_filament_group_for_match_multi_nozzle(&context)?;
                } else {
                    let mut fg = FilamentGroup::new(&context);
                    ret = fg.calc_filament_group(None)?;
                }
                let result_opt =
                    <LayeredNozzleGroupResult as CreateFromNozzleMap>::create(&ret, &context.nozzle_info.nozzle_list, &used_filaments, handle_hotend_as_extruder);
                let Some(result) = result_opt else {
                    return Ok(LayeredNozzleGroupResult::default());
                };
                // Manual-mode extruder validation only applies to the virtual-extruder model.
                if mode == FilamentMapMode::FmmManual && !single_ext_multi_nozzle {
                    let result_map = result.get_extruder_map(true, -1);
                    let filament_map = cfg_ints(print_config, "filament_map");
                    for &fid in &used_filaments {
                        if result_map[fid as usize] != filament_map[fid as usize] - 1 {
                            return Err(runtime("Group error in manual mode. Please check nozzle count or regroup."));
                        }
                    }
                    validate_manual_diameter_or_throw(print_config, &used_filaments, &result_map, multi_diameter_mode).map_err(FilamentGroupError::Runtime)?;
                }
                return Ok(result);
            } else {
                let mut fg = FilamentGroup::new(&context);
                fg.get_custom_seq = Some(std::rc::Rc::from(group_reorder::create_custom_seq_function(print_config, false, &[])));
                ret = fg.calc_filament_group(None)?;
            }
        }

        let result_opt = <LayeredNozzleGroupResult as CreateFromNozzleMap>::create(&ret, &nozzle_list, &used_filaments, handle_hotend_as_extruder);
        Ok(result_opt.unwrap_or_default())
    }

    /// ToolOrdering.cpp:2709-2787 `void ToolOrdering::calculate_and_store_statistics(const PrintConfig &print_config,
    /// const MultiNozzleUtils::LayeredNozzleGroupResult &grouping_result, const LayerData &layer_data,
    /// const OrderingContext &ordering_context, const std::vector<FlushMatrix> &nozzle_flush_mtx,
    /// const std::vector<std::vector<unsigned int>> &filament_sequences)`. `print` is `m_print`.
    #[allow(clippy::too_many_arguments)]
    pub fn calculate_and_store_statistics(
        &mut self,
        print: &dyn ToolOrderingPrint,
        print_config: &ToolOrderingPrintConfig,
        grouping_result: &crate::multi_nozzle_utils::LayeredNozzleGroupResult,
        layer_data: &LayerData,
        ordering_context: &OrderingContext,
        nozzle_flush_mtx: &[group_reorder::FlushMatrix],
        filament_sequences: &[Vec<u32>],
    ) -> Result<(), crate::filament_group::FilamentGroupError> {
        use crate::multi_nozzle_utils::{CreateFromLayerMaps, CreateFromNozzleMap, LayeredNozzleGroupResult, NozzleInfo, NozzleStatusRecorder};
        use crate::print_config::enums::FilamentMapMode;

        let extruder_num = cfg_floats(print_config, "nozzle_diameter").len();

        let curr_flush_info = calc_filament_change_info_by_toolorder(print_config, grouping_result, nozzle_flush_mtx, filament_sequences);

        let fmm_auto_for_flush = cfg_int(print_config, "filament_map_mode") == FilamentMapMode::FmmAutoForFlush as i32;
        if extruder_num <= 1 {
            self.stats_single_ext = curr_flush_info;
        } else {
            self.stats_multi_ext_curr = curr_flush_info.clone();
            if fmm_auto_for_flush {
                self.stats_multi_ext_best = curr_flush_info;
            }
        }

        // 多喷嘴时，计算单挤出机与最优分组下（如果需要）的信息
        if extruder_num > 1 {
            {
                let mut single_extruder_sequences: Vec<Vec<u32>> = Vec::new();
                let maps_without_group: Vec<i32> = vec![0; cfg_strings(print_config, "filament_colour").len()];

                let mut tmp = NozzleInfo::default();
                // tmp.diameter = print_config.nozzle_diameter.get_at(0): a double assigned to std::string is
                // operator=(char), the one character (char)value.
                tmp.diameter = String::from((get_at(&cfg_floats(print_config, "nozzle_diameter"), 0) as i8 as u8) as char);
                tmp.group_id = 0;
                tmp.extruder_id = 0;
                tmp.volume_type = crate::extruder::NozzleVolumeType::NvtStandard;

                let result_opt =
                    <LayeredNozzleGroupResult as CreateFromNozzleMap>::create(&maps_without_group, &[tmp], &layer_data.used_filaments, false);
                let Some(result) = result_opt else {
                    return Ok(());
                };

                crate::gcode::tool_order_utils::reorder_filaments_for_multi_nozzle_extruder(
                    &ordering_context.filament_lists,
                    &result,
                    &layer_data.layer_filaments,
                    nozzle_flush_mtx,
                    Some(&*ordering_context.get_custom_seq),
                    Some(&mut single_extruder_sequences),
                    &NozzleStatusRecorder::new(),
                );
                self.stats_single_ext = calc_filament_change_info_by_toolorder(print_config, &result, nozzle_flush_mtx, &single_extruder_sequences);
            }

            if !fmm_auto_for_flush {
                let mut best_seq: Vec<Vec<u32>> = Vec::new();

                // 如果支持选料器
                if print.is_dynamic_group_reorder() {
                    let grouping_context = group_reorder::build_filament_group_context(
                        print,
                        &layer_data.layer_filaments,
                        &layer_data.physical_unprintables,
                        &layer_data.geometric_unprintables,
                        &layer_data.filament_unprintable_volumes,
                        FilamentMapMode::FmmAutoForFlush,
                        self.initial_nozzle_status.get_nozzle_filament_map(),
                    )?;

                    let mut best_nozzle_status = self.initial_nozzle_status.clone();
                    let dynamic_plan_res = plan_filament_mapping_and_order_by_combo_ranges(
                        print,
                        &grouping_context,
                        ordering_context,
                        FilamentMapMode::FmmAutoForFlush,
                        &layer_data.physical_unprintables,
                        &layer_data.geometric_unprintables,
                        &layer_data.filament_unprintable_volumes,
                        Some(&mut best_nozzle_status),
                    )?;

                    let mut nozzle_map_per_layer: Vec<Vec<i32>> = Vec::new();
                    for res in &dynamic_plan_res {
                        best_seq.push(res.fil_order.iter().map(|&f| f as u32).collect());
                        nozzle_map_per_layer.push(res.fil_nozzle_match.clone());
                    }

                    let result = <LayeredNozzleGroupResult as CreateFromLayerMaps>::create(
                        &nozzle_map_per_layer,
                        &grouping_context.nozzle_info.nozzle_list,
                        &layer_data.used_filaments,
                        filament_sequences,
                        cfg_bool(print_config, "handle_hotend_as_extruder"),
                    );

                    if let Some(result) = result {
                        self.stats_multi_ext_best = calc_filament_change_info_by_toolorder(print_config, &result, nozzle_flush_mtx, &best_seq);
                    }
                } else {
                    let group_result_auto = ToolOrdering::get_recommended_filament_maps(
                        print,
                        &layer_data.layer_filaments,
                        FilamentMapMode::FmmAutoForFlush,
                        &layer_data.physical_unprintables,
                        &layer_data.geometric_unprintables,
                        &layer_data.filament_unprintable_volumes,
                        &crate::multi_nozzle_utils::UnorderedMapInt::new(),
                    )?;

                    crate::gcode::tool_order_utils::reorder_filaments_for_multi_nozzle_extruder(
                        &ordering_context.filament_lists,
                        &group_result_auto,
                        &layer_data.layer_filaments,
                        nozzle_flush_mtx,
                        Some(&*ordering_context.get_custom_seq),
                        Some(&mut best_seq),
                        &NozzleStatusRecorder::new(),
                    );

                    self.stats_multi_ext_best = calc_filament_change_info_by_toolorder(print_config, &group_result_auto, nozzle_flush_mtx, &best_seq);
                }
            }
        }
        Ok(())
    }
}

/// ToolOrdering.hpp:374-380 `ToolOrdering::LayerData`
#[derive(Debug, Clone, Default)]
pub struct LayerData {
    pub layer_filaments: Vec<Vec<u32>>,
    pub used_filaments: Vec<u32>,
    pub physical_unprintables: Vec<BTreeSet<i32>>,
    pub geometric_unprintables: Vec<BTreeSet<i32>>,
    pub filament_unprintable_volumes: BTreeMap<i32, BTreeSet<crate::extruder::NozzleVolumeType>>,
}

/// ToolOrdering.hpp:382-387 `ToolOrdering::OrderingContext`
pub struct OrderingContext {
    pub filament_lists: Vec<u32>,
    pub get_custom_seq: group_reorder::CustomSeqFunction,
    pub support_multi_nozzle: bool,
    pub support_dynamic_map: bool,
}

impl ToolOrdering {
    /// ToolOrdering.cpp:2789-2811 `ToolOrdering::collect_layer_and_unprintable_data`. `print` is
    /// `m_print`.
    pub fn collect_layer_and_unprintable_data(&mut self, print: &dyn ToolOrderingPrint) -> LayerData {
        let mut data = LayerData::default();

        // 收集层信息
        for lt in &self.layer_tools {
            data.layer_filaments.push(lt.extruders.clone());
        }

        data.used_filaments = crate::filament_group::collect_sorted_used_filaments(&data.layer_filaments);

        data.geometric_unprintables = print.get_geometric_unprintable_filaments();

        let is_mixed = cfg_bools(print.config(), "filament_is_mixed");
        let comp_strs = cfg_strings(print.config(), "filament_mixed_components");
        if crate::filament_mixer::has_any_mixed_filament(&is_mixed) {
            crate::filament_mixer::expand_mixed_slots_in_unprintables(&mut data.geometric_unprintables, &is_mixed, &comp_strs);
        }

        data.physical_unprintables = print.get_physical_unprintable_filaments(&data.used_filaments);
        data.filament_unprintable_volumes = print.get_filament_unprintable_flow(&data.used_filaments);

        data
    }

    /// ToolOrdering.cpp:2815-2841 `ToolOrdering::prepare_ordering_context`
    pub fn prepare_ordering_context(&mut self, print_config: &ToolOrderingPrintConfig, reorder_first_layer: bool) -> OrderingContext {
        let filament_nums = cfg_strings(print_config, "filament_colour").len();

        let filament_lists: Vec<u32> = (0..filament_nums as u32).collect();

        let mut first_layer_filaments: Vec<u32> = Vec::new();
        if !self.layer_tools.is_empty() {
            first_layer_filaments = self.layer_tools[0].extruders.clone();
        }

        let get_custom_seq =
            group_reorder::create_custom_seq_function(print_config, !reorder_first_layer, &first_layer_filaments);

        let support_multi_nozzle = {
            let has_multi_nozzle = cfg_ints(print_config, "extruder_max_nozzle_count").iter().any(|&v| v > 1);
            has_multi_nozzle
                && (cfg_bool(print_config, "handle_hotend_as_extruder") || cfg_floats(print_config, "nozzle_diameter").len() > 1)
        };

        let support_dynamic_map = cfg_bool(print_config, "enable_filament_dynamic_map");

        OrderingContext { filament_lists, get_custom_seq, support_multi_nozzle, support_dynamic_map }
    }

    /// ToolOrdering.cpp:2844-2873 `ToolOrdering::execute_filament_ordering`
    pub fn execute_filament_ordering(
        &mut self,
        _print_config: &ToolOrderingPrintConfig,
        grouping_result: &crate::multi_nozzle_utils::LayeredNozzleGroupResult,
        layer_data: &LayerData,
        ordering_context: &OrderingContext,
        nozzle_flush_mtx: &[group_reorder::FlushMatrix],
    ) -> Vec<Vec<u32>> {
        let mut filament_sequences: Vec<Vec<u32>> = Vec::new();
        if ordering_context.support_multi_nozzle {
            crate::gcode::tool_order_utils::reorder_filaments_for_multi_nozzle_extruder(
                &ordering_context.filament_lists,
                grouping_result,
                &layer_data.layer_filaments,
                nozzle_flush_mtx,
                Some(&*ordering_context.get_custom_seq),
                Some(&mut filament_sequences),
                &crate::multi_nozzle_utils::NozzleStatusRecorder::new(),
            );
        } else {
            crate::gcode::tool_order_utils::reorder_filaments_for_minimum_flush_volume(
                &ordering_context.filament_lists,
                &grouping_result.get_extruder_map(true, -1),
                &layer_data.layer_filaments,
                nozzle_flush_mtx,
                Some(&*ordering_context.get_custom_seq),
                Some(&mut filament_sequences),
                &HashMap::new(),
            );
        }
        filament_sequences
    }
}

/// ToolOrdering.cpp:2875-2891 `snap_to_simple_fraction` (`max_denom` defaults to 10).
pub fn snap_to_simple_fraction(r: f64, max_denom: i32) -> f64 {
    let mut best_r = r;
    let mut best_err = 1.0f64;
    for q in 1..=max_denom {
        // std::round: half away from zero.
        let mut p = (r * q as f64).round() as i32;
        if p < 0 {
            p = 0;
        }
        if p > q {
            p = q;
        }
        let candidate = p as f64 / q as f64;
        let err = (candidate - r).abs();
        if err < best_err {
            best_err = err;
            best_r = candidate;
        }
    }
    best_r
}

impl ToolOrdering {
    /// ToolOrdering.cpp:2893-3422 `ToolOrdering::resolve_mixed_filaments`
    pub fn resolve_mixed_filaments(&mut self, config: &ToolOrderingPrintConfig) {
        use crate::filament_mixer::{
            has_any_mixed_filament, parse_gradient_curve, parse_mixed_components, parse_mixed_ratios, sample_gradient_curve,
            GradientCurve,
        };
        let is_mixed = cfg_bools(config, "filament_is_mixed");
        let comp_strs = cfg_strings(config, "filament_mixed_components");
        let ratio_strs = cfg_strings(config, "filament_mixed_sublayer_ratios");

        // Capture mixed slots that actually appear on layers before they are expanded to
        // physical components. Assigned-but-unused mixed slots never enter layer_tools.
        self.used_mixed_filaments.clear();
        self.directly_used_filaments.clear();
        if has_any_mixed_filament(&is_mixed) {
            let mut used: BTreeSet<u32> = BTreeSet::new();
            for lt in &self.layer_tools {
                for &ext in &lt.extruders {
                    if (ext as usize) < is_mixed.len() && is_mixed[ext as usize] != 0 {
                        used.insert(ext);
                    }
                }
            }
            self.used_mixed_filaments = used.into_iter().collect();
        }
        {
            let mut direct: BTreeSet<u32> = BTreeSet::new();
            for lt in &self.layer_tools {
                for &ext in &lt.extruders {
                    if ext as usize >= is_mixed.len() || is_mixed[ext as usize] == 0 {
                        direct.insert(ext);
                    }
                }
            }
            self.directly_used_filaments = direct.into_iter().collect();
        }

        if !has_any_mixed_filament(&is_mixed) {
            return;
        }

        let sublayer_enabled = cfg_bool(config, "enable_mixed_color_sublayer");

        #[derive(Default, Clone)]
        struct SlotInfo {
            components: Vec<u32>, // 1-based
            ratios: Vec<f64>,
            accum: Vec<i64>, // deficit accumulator (integer, unit: 1e-6 mm)
        }
        let mut slots: Vec<SlotInfo> = vec![SlotInfo::default(); is_mixed.len()];
        for i in 0..is_mixed.len() {
            if is_mixed[i] == 0 {
                continue;
            }
            slots[i].components = parse_mixed_components(if i < comp_strs.len() { &comp_strs[i] } else { "" });
            if slots[i].components.len() < 2 {
                slots[i].components.clear();
                continue;
            }
            for k in 0..slots[i].components.len() {
                let cid = slots[i].components[k];
                let idx0 = cid.wrapping_sub(1);
                if idx0 as usize >= is_mixed.len() || ((idx0 as usize) < is_mixed.len() && is_mixed[idx0 as usize] != 0) {
                    slots[i].components.clear();
                    break;
                }
            }
            if slots[i].components.is_empty() {
                continue;
            }
            slots[i].ratios =
                parse_mixed_ratios(if i < ratio_strs.len() { &ratio_strs[i] } else { "" }, slots[i].components.len());
            if !sublayer_enabled {
                for r in slots[i].ratios.iter_mut() {
                    *r = snap_to_simple_fraction(*r, 10);
                }
                let mut sum = 0.0f64;
                for &r in &slots[i].ratios {
                    sum += r;
                }
                if sum > 0.0 {
                    for r in slots[i].ratios.iter_mut() {
                        *r /= sum;
                    }
                }
            }
            slots[i].accum = vec![0i64; slots[i].components.len()];
        }

        // Parse gradient settings per slot
        let gradient_flags = cfg_bools(config, "filament_mixed_gradient");
        let gradient_range_strs = cfg_strings(config, "filament_mixed_gradient_range");
        let gradient_curve_strs = cfg_strings(config, "filament_mixed_gradient_curve");
        #[derive(Clone)]
        struct GradientInfo {
            start: f64,
            end_val: f64,
            curve: GradientCurve, // empty -> use linear (start, end_val); non-empty wins
        }
        let mut is_gradient: Vec<bool> = vec![false; is_mixed.len()];
        let mut gradient_info: Vec<GradientInfo> =
            vec![GradientInfo { start: 0.10, end_val: 0.90, curve: GradientCurve::default() }; is_mixed.len()];
        for i in 0..is_mixed.len() {
            if is_mixed[i] == 0 || slots[i].components.len() != 2 {
                continue;
            }
            if i >= gradient_flags.len() || gradient_flags[i] == 0 {
                continue;
            }
            is_gradient[i] = true;
            if i < gradient_range_strs.len() && !gradient_range_strs[i].is_empty() {
                // CNumericLocalesSetter; sscanf("%f,%f")
                if let Some((v0, v1)) = sscanf_two_floats(&gradient_range_strs[i]) {
                    if v0 > 0.0 && (v0 as f64) < 1.0 && v1 > 0.0 && (v1 as f64) < 1.0 {
                        gradient_info[i].start = v0 as f64;
                        gradient_info[i].end_val = v1 as f64;
                    }
                }
            }
            if i < gradient_curve_strs.len() && !gradient_curve_strs[i].is_empty() {
                gradient_info[i].curve = parse_gradient_curve(&gradient_curve_strs[i]);
            }
        }

        // Pass 1: identify continuous runs for each gradient slot (Per-Run).
        #[derive(Default, Clone)]
        struct GradientRunInfo {
            run_lengths: Vec<usize>,
            current_run: i32,
            current_idx: usize,
            prev_appeared: bool,
            last_absent_was_relevant: bool,
        }
        let mut gradient_runs: BTreeMap<u32, GradientRunInfo> = BTreeMap::new();
        for i in 0..is_mixed.len() {
            if is_gradient[i] {
                gradient_runs.insert(i as u32, GradientRunInfo { current_run: -1, ..Default::default() });
            }
        }

        fn find_swallowed<'m, K: Ord>(
            swallowed_map: &'m BTreeMap<u32, BTreeMap<K, BTreeSet<usize>>>,
            slot: u32,
            key: &K,
        ) -> Option<&'m BTreeSet<usize>> {
            swallowed_map.get(&slot)?.get(key)
        }
        fn relevant_layers(layers: &[usize], swallowed: Option<&BTreeSet<usize>>) -> BTreeSet<usize> {
            let mut out = BTreeSet::new();
            for &li in layers {
                if swallowed.map_or(true, |s| !s.contains(&li)) {
                    out.insert(li);
                }
            }
            out
        }
        let mut slot_relevant_layers: BTreeMap<u32, BTreeSet<usize>> = BTreeMap::new();
        for (&slot_idx, obj_map) in &self.mixed_object_layers {
            for obj in obj_map.keys() {
                if let Some(it) = self.object_all_layer_indices.get(obj) {
                    let relevant = relevant_layers(it, find_swallowed(&self.mixed_swallowed_layers, slot_idx, obj));
                    slot_relevant_layers.entry(slot_idx).or_default().extend(relevant);
                }
            }
        }

        if !gradient_runs.is_empty() {
            for li in 0..self.layer_tools.len() {
                if li == 0 {
                    continue;
                }
                let lt = &self.layer_tools[li];
                for (slot, run) in gradient_runs.iter_mut() {
                    let here = lt.extruders.contains(slot);
                    if here {
                        let mut real_gap = false;
                        if !run.prev_appeared && !run.run_lengths.is_empty() {
                            real_gap = run.last_absent_was_relevant;
                        }
                        if run.run_lengths.is_empty() || real_gap {
                            run.run_lengths.push(0);
                        }
                        *run.run_lengths.last_mut().unwrap() += 1;
                        run.last_absent_was_relevant = false;
                    } else if !run.run_lengths.is_empty() {
                        if let Some(rel) = slot_relevant_layers.get(slot) {
                            if rel.contains(&li) {
                                run.last_absent_was_relevant = true;
                            }
                        }
                    }
                    run.prev_appeared = here;
                }
            }
            for run in gradient_runs.values_mut() {
                run.current_run = -1;
                run.current_idx = 0;
                run.prev_appeared = false;
                run.last_absent_was_relevant = false;
            }
        }

        // Per-object gradient: pre-compute per-object runs (respecting Z gaps within each object).
        #[derive(Default, Clone)]
        struct PerObjRunState {
            run_start_offsets: Vec<usize>,
            run_lengths: Vec<usize>,
            current_run: i32,
            current_idx: usize,
        }

        let has_real_gap = |prev_idx: usize, cur_idx: usize, obj_set: &BTreeSet<usize>, slot_set: &BTreeSet<usize>| -> bool {
            for &it in obj_set.range(prev_idx + 1..) {
                if it >= cur_idx {
                    break;
                }
                if !slot_set.contains(&it) {
                    return true;
                }
            }
            false
        };

        let segment_runs = |layer_indices: &[usize], obj_set: &BTreeSet<usize>, slot_set: &BTreeSet<usize>| -> PerObjRunState {
            let mut st = PerObjRunState { current_run: -1, ..Default::default() };
            for i in 0..layer_indices.len() {
                let new_run = i == 0 || has_real_gap(layer_indices[i - 1], layer_indices[i], obj_set, slot_set);
                if new_run {
                    st.run_start_offsets.push(i);
                    st.run_lengths.push(0);
                }
                *st.run_lengths.last_mut().unwrap() += 1;
            }
            st
        };

        let mut per_obj_runs: BTreeMap<u32, BTreeMap<usize, PerObjRunState>> = BTreeMap::new();
        {
            let slots_keys: Vec<u32> = self.mixed_object_layers.keys().copied().collect();
            for slot in slots_keys {
                if slot as usize >= is_gradient.len() || !is_gradient[slot as usize] {
                    continue;
                }
                let objs: Vec<usize> = self.mixed_object_layers[&slot].keys().copied().collect();
                for obj in objs {
                    let layer_indices = self.mixed_object_layers.get_mut(&slot).unwrap().get_mut(&obj).unwrap();
                    sort_remove_duplicates_usize(layer_indices);
                    // Erase layer 0 — this mutation is also relied upon by the Pass 2 binary_search below.
                    if !layer_indices.is_empty() && layer_indices[0] == 0 {
                        layer_indices.remove(0);
                    }
                    let layer_indices = layer_indices.clone();

                    // `m_object_all_layer_indices[obj]`: operator[] inserts an empty list.
                    let all = self.object_all_layer_indices.entry(obj).or_default().clone();
                    let all_obj_set = relevant_layers(&all, find_swallowed(&self.mixed_swallowed_layers, slot, &obj));
                    let grad_set: BTreeSet<usize> = layer_indices.iter().copied().collect();

                    per_obj_runs.entry(slot).or_default().insert(obj, segment_runs(&layer_indices, &all_obj_set, &grad_set));
                }
            }
        }

        // Per-volume gradient: mirror the per-object run-segmentation logic above.
        let mut per_vol_runs: BTreeMap<u32, BTreeMap<VolumeKey, PerObjRunState>> = BTreeMap::new();
        {
            let slots_keys: Vec<u32> = self.gradient_volume_layers.keys().copied().collect();
            for slot in slots_keys {
                if slot as usize >= is_gradient.len() || !is_gradient[slot as usize] {
                    continue;
                }
                let vkeys: Vec<VolumeKey> = self.gradient_volume_layers[&slot].keys().copied().collect();
                for vkey in vkeys {
                    let layer_indices = self.gradient_volume_layers.get_mut(&slot).unwrap().get_mut(&vkey).unwrap();
                    sort_remove_duplicates_usize(layer_indices);
                    if !layer_indices.is_empty() && layer_indices[0] == 0 {
                        layer_indices.remove(0);
                    }
                    let layer_indices = layer_indices.clone();

                    let all = self.object_all_layer_indices.entry(vkey.obj).or_default().clone();
                    let all_obj_set = relevant_layers(&all, find_swallowed(&self.gradient_volume_swallowed_layers, slot, &vkey));
                    let vol_grad_set: BTreeSet<usize> = layer_indices.iter().copied().collect();

                    per_vol_runs.entry(slot).or_default().insert(vkey, segment_runs(&layer_indices, &all_obj_set, &vol_grad_set));
                }
            }
        }
        // Pass 2: resolve per layer
        let mut prev_print_z: f64 = 0.0;
        let mut prev_print_z_for_slot: BTreeMap<u32, f64> = BTreeMap::new();
        let mut prev_relevant_z_for_slot: BTreeMap<u32, f64> = BTreeMap::new();

        let calc_slot_lh = |mixed_layer_heights: &BTreeMap<u32, BTreeMap<usize, f64>>,
                            prev_print_z_for_slot: &BTreeMap<u32, f64>,
                            prev_relevant_z_for_slot: &BTreeMap<u32, f64>,
                            prev_print_z: f64,
                            ext: u32,
                            print_z: f64,
                            layer_idx: usize|
         -> f64 {
            if let Some(height_slot) = mixed_layer_heights.get(&ext) {
                if let Some(&h) = height_slot.get(&layer_idx) {
                    if h > 0.0 {
                        return h;
                    }
                }
            }

            let slot_pz = prev_print_z_for_slot.get(&ext);
            let rel_pz = prev_relevant_z_for_slot.get(&ext);
            let mut base_z = prev_print_z;
            if let Some(&spz) = slot_pz {
                base_z = spz;
                if let Some(&rpz) = rel_pz {
                    base_z = base_z.max(rpz);
                }
            }
            let lh = print_z - base_z;
            if lh > 0.0 {
                lh
            } else {
                0.2
            } // 0.2mm safety fallback; should not trigger in normal operation
        };

        for layer_idx in 0..self.layer_tools.len() {
            // Update gradient run state (skip first layer to match counting).
            if layer_idx > 0 {
                let lt = &self.layer_tools[layer_idx];
                for (slot, run) in gradient_runs.iter_mut() {
                    let here = lt.extruders.contains(slot);
                    if here {
                        if !run.prev_appeared {
                            if run.last_absent_was_relevant || run.current_run < 0 {
                                run.current_run += 1;
                                run.current_idx = 0;
                            }
                        }
                        run.last_absent_was_relevant = false;
                    } else if let Some(rel) = slot_relevant_layers.get(slot) {
                        if rel.contains(&layer_idx) {
                            run.last_absent_was_relevant = true;
                        }
                    }
                    run.prev_appeared = here;
                }
            }

            let lt_print_z = self.layer_tools[layer_idx].print_z;
            let lt_extruders = self.layer_tools[layer_idx].extruders.clone();
            let mut new_extruders: Vec<u32> = Vec::new();
            for &ext in &lt_extruders {
                if ext as usize >= slots.len() || slots[ext as usize].components.is_empty() {
                    new_extruders.push(ext);
                    continue;
                }

                // Skip sublayer splitting for the first layer to preserve bed adhesion.
                if sublayer_enabled && layer_idx > 0 {
                    let lh = calc_slot_lh(
                        &self.mixed_layer_heights,
                        &prev_print_z_for_slot,
                        &prev_relevant_z_for_slot,
                        prev_print_z,
                        ext,
                        lt_print_z,
                        layer_idx,
                    );
                    let s = &slots[ext as usize];
                    let n = s.components.len();

                    let mut sub_heights: Vec<f64> = Vec::new();
                    let mut gradient_last_no_split = false;
                    let mut gradient_last_dominant_0b: u32 = 0;
                    if is_gradient[ext as usize] && n == 2 {
                        let gr = gradient_runs.get_mut(&ext);
                        let usable = gr.as_ref().map_or(false, |run| {
                            run.current_run >= 0 && (run.current_run as usize) < run.run_lengths.len()
                        });
                        if usable {
                            let run = gr.unwrap();
                            let big_n = run.run_lengths[run.current_run as usize];
                            let idx = run.current_idx;
                            run.current_idx += 1;
                            // clang contracts `2.0 * idx + 1.0` and `start + (end - start) * t` into fmas
                            // (-ffp-contract=on).
                            let t = if big_n > 0 { 2.0f64.mul_add(idx as f64, 1.0) / (2.0 * big_n as f64) } else { 0.5 };
                            // Custom curve wins over linear range when present; OFF path stays bit-identical.
                            let gi = &gradient_info[ext as usize];
                            let r1 = if gi.curve.empty() {
                                (gi.end_val - gi.start).mul_add(t, gi.start)
                            } else {
                                sample_gradient_curve(&gi.curve, t)
                            };
                            let r2 = 1.0 - r1;
                            sub_heights.push(r1 * lh);
                            sub_heights.push(r2 * lh);
                            if idx == big_n.wrapping_sub(1) {
                                // When r1 == r2 (exactly 50/50), component[0] is treated as dominant.
                                let dominant = if r1 >= r2 { 0 } else { 1 };
                                let dom_0b = s.components[dominant].wrapping_sub(1);
                                let oth_0b = s.components[1 - dominant].wrapping_sub(1);
                                if dom_0b < oth_0b {
                                    gradient_last_no_split = true;
                                    gradient_last_dominant_0b = dom_0b;
                                }
                            }
                        } else {
                            for &r in &s.ratios {
                                sub_heights.push(r * lh);
                            }
                        }
                    } else {
                        for &r in &s.ratios {
                            sub_heights.push(r * lh);
                        }
                    }

                    // Per-part gradient: when this slot has any qualifying volume, the global
                    // no-split short-circuit must NOT bypass MixedSubLayerGroup creation.
                    let per_vol_active_for_slot = per_vol_runs.get(&ext).map_or(false, |m| !m.is_empty());

                    if gradient_last_no_split && !per_vol_active_for_slot {
                        self.layer_tools[layer_idx].mixed_filament_resolution.insert(ext, gradient_last_dominant_0b);
                        new_extruders.push(gradient_last_dominant_0b);
                        prev_print_z_for_slot.insert(ext, lt_print_z);
                        continue;
                    }

                    let mut grp = MixedSubLayerGroup::default();
                    grp.mixed_slot_0based = ext;
                    grp.layer_height = lh;
                    grp.is_gradient = is_gradient[ext as usize];
                    for k in 0..s.components.len() {
                        let comp_0based = s.components[k].wrapping_sub(1);
                        grp.components_0based.push(comp_0based);
                    }
                    grp.sub_heights = sub_heights;

                    if grp.is_gradient {
                        if let Some(vol_runs_slot) = per_vol_runs.get_mut(&ext) {
                            for (vkey, st) in vol_runs_slot.iter_mut() {
                                let layer_indices = self
                                    .gradient_volume_layers
                                    .get_mut(&ext)
                                    .map(|m| m.entry(*vkey).or_default().clone())
                                    .unwrap_or_default();
                                if layer_indices.binary_search(&layer_idx).is_err() {
                                    continue;
                                }
                                if st.current_run < 0 || st.current_idx >= st.run_lengths[st.current_run as usize] {
                                    st.current_run += 1;
                                    st.current_idx = 0;
                                }
                                let run_n = st.run_lengths[st.current_run as usize];
                                let run_idx = st.current_idx;
                                st.current_idx += 1;
                                let gi = &gradient_info[ext as usize];
                                grp.per_volume_gradient.insert(
                                    *vkey,
                                    ObjectGradient {
                                        total_layers: run_n,
                                        current_idx: run_idx,
                                        gradient_start: gi.start,
                                        gradient_end: gi.end_val,
                                        curve: gi.curve.clone(),
                                    },
                                );
                            }
                        }

                        if let Some(runs_slot) = per_obj_runs.get_mut(&ext) {
                            for (obj, st) in runs_slot.iter_mut() {
                                let layer_indices = self
                                    .mixed_object_layers
                                    .get_mut(&ext)
                                    .map(|m| m.entry(*obj).or_default().clone())
                                    .unwrap_or_default();
                                if layer_indices.binary_search(&layer_idx).is_err() {
                                    continue;
                                }
                                if st.current_run < 0 || st.current_idx >= st.run_lengths[st.current_run as usize] {
                                    st.current_run += 1;
                                    st.current_idx = 0;
                                }
                                let run_n = st.run_lengths[st.current_run as usize];
                                let run_idx = st.current_idx;
                                st.current_idx += 1;
                                let gi = &gradient_info[ext as usize];
                                grp.per_object_gradient.insert(
                                    *obj,
                                    ObjectGradient {
                                        total_layers: run_n,
                                        current_idx: run_idx,
                                        gradient_start: gi.start,
                                        gradient_end: gi.end_val,
                                        curve: gi.curve.clone(),
                                    },
                                );
                            }
                        }
                    }

                    if grp.components_0based.len() > 1 {
                        let first_comp_0based = s.components[0].wrapping_sub(1);
                        let mut idx: Vec<usize> = (0..grp.components_0based.len()).collect();
                        // std::sort (libc++): components of equal id keep libc++'s order.
                        let comps = grp.components_0based.clone();
                        crate::clipper::libcxx_sort(&mut idx, |&a, &b| comps[a] < comps[b]);
                        let mut sorted_comps: Vec<u32> = Vec::new();
                        let mut sorted_heights: Vec<f64> = Vec::new();
                        for &i in &idx {
                            sorted_comps.push(grp.components_0based[i]);
                            sorted_heights.push(grp.sub_heights[i]);
                        }
                        grp.components_0based = sorted_comps;
                        grp.sub_heights = sorted_heights;
                        if grp.is_gradient {
                            for i in 0..grp.components_0based.len() {
                                if grp.components_0based[i] == first_comp_0based {
                                    grp.gradient_first_sorted_idx = i as i32;
                                    break;
                                }
                            }
                        }
                    }

                    for &comp in &grp.components_0based {
                        new_extruders.push(comp);
                    }
                    self.layer_tools[layer_idx].mixed_sub_layer_groups.push(grp);
                    prev_print_z_for_slot.insert(ext, lt_print_z);
                } else {
                    // Deficit Round-Robin: pick one component per layer.
                    // Weight by layer height so volume ratios stay accurate
                    // even with adaptive layer heights.
                    let lh = calc_slot_lh(
                        &self.mixed_layer_heights,
                        &prev_print_z_for_slot,
                        &prev_relevant_z_for_slot,
                        prev_print_z,
                        ext,
                        lt_print_z,
                        layer_idx,
                    );
                    let lh_i: i64 = llround(lh * 1e6);

                    let s = &mut slots[ext as usize];
                    // For 2-component gradient on the first layer, use the gradient's
                    // starting ratio instead of the configured mixing ratio.
                    if layer_idx == 0 && is_gradient[ext as usize] && s.components.len() == 2 {
                        let r0 = gradient_info[ext as usize].start;
                        s.accum[0] += llround(r0 * lh_i as f64);
                        s.accum[1] += llround((1.0 - r0) * lh_i as f64);
                    } else {
                        for k in 0..s.ratios.len() {
                            s.accum[k] += llround(s.ratios[k] * lh_i as f64);
                        }
                    }
                    let mut sel = 0usize;
                    for k in 1..s.accum.len() {
                        if s.accum[k] > s.accum[sel] {
                            sel = k;
                        }
                    }
                    s.accum[sel] -= lh_i;
                    let resolved = s.components[sel].wrapping_sub(1);
                    self.layer_tools[layer_idx].mixed_filament_resolution.insert(ext, resolved);
                    new_extruders.push(resolved);
                    prev_print_z_for_slot.insert(ext, lt_print_z);
                }
            }
            let lt = &mut self.layer_tools[layer_idx];
            lt.extruders = new_extruders;
            sort_remove_duplicates_u32(&mut lt.extruders);

            // Update prev_relevant_z: for each slot that has relevant-layer tracking,
            // advance if the current layer belongs to a slot-owning object.
            for (&slot, rel_set) in &slot_relevant_layers {
                if rel_set.contains(&layer_idx) {
                    prev_relevant_z_for_slot.insert(slot, lt_print_z);
                }
            }

            prev_print_z = lt_print_z;
        }
    }

    /// ToolOrdering.cpp:3424-3508 `ToolOrdering::enforce_mixed_component_order`
    pub fn enforce_mixed_component_order(&mut self) {
        for lt in self.layer_tools.iter_mut() {
            if lt.mixed_sub_layer_groups.is_empty() {
                continue;
            }

            // Build a set of extruders present in lt.extruders for fast lookup.
            let ext_set: BTreeSet<u32> = lt.extruders.iter().copied().collect();

            // 1. Build DAG from mixed group constraints.
            let mut adj: BTreeMap<u32, Vec<u32>> = BTreeMap::new();
            let mut in_degree: BTreeMap<u32, i32> = BTreeMap::new();
            let mut edge_set: BTreeSet<(u32, u32)> = BTreeSet::new();

            for &ext in &lt.extruders {
                in_degree.insert(ext, 0);
            }

            for grp in &lt.mixed_sub_layer_groups {
                let mut i = 0usize;
                while i + 1 < grp.components_0based.len() {
                    let a = grp.components_0based[i];
                    let b = grp.components_0based[i + 1];
                    i += 1;
                    if !ext_set.contains(&a) || !ext_set.contains(&b) {
                        continue;
                    }
                    if edge_set.insert((a, b)) {
                        adj.entry(a).or_default().push(b);
                        *in_degree.entry(b).or_insert(0) += 1;
                    }
                }
            }

            // 2. Record original position (from flush optimizer) as priority.
            let mut orig_pos: BTreeMap<u32, usize> = BTreeMap::new();
            for (i, &e) in lt.extruders.iter().enumerate() {
                orig_pos.insert(e, i);
            }

            // 3. Kahn's topological sort with priority queue (prefer original position):
            //    a min-heap by orig_pos (`orig_pos[lhs] > orig_pos[rhs]`).
            let mut pq: std::collections::BinaryHeap<std::cmp::Reverse<(usize, u32)>> = std::collections::BinaryHeap::new();

            for &ext in &lt.extruders {
                if *in_degree.entry(ext).or_insert(0) == 0 {
                    let p = *orig_pos.entry(ext).or_insert(0);
                    pq.push(std::cmp::Reverse((p, ext)));
                }
            }

            let mut ordered: Vec<u32> = Vec::with_capacity(lt.extruders.len());
            while let Some(std::cmp::Reverse((_, ext))) = pq.pop() {
                ordered.push(ext);
                if let Some(it) = adj.get(&ext) {
                    for &next in it {
                        let d = in_degree.entry(next).or_insert(0);
                        *d -= 1;
                        if *d == 0 {
                            let p = *orig_pos.entry(next).or_insert(0);
                            pq.push(std::cmp::Reverse((p, next)));
                        }
                    }
                }
            }

            // Safety: if topological sort didn't produce all elements, keep original order.
            if ordered.len() != lt.extruders.len() {
                ordered = lt.extruders.clone();
            }

            // 4. Verify: every mixed group's component order is preserved as subsequence.
            for grp in &lt.mixed_sub_layer_groups {
                let mut prev_pos = 0usize;
                let mut valid = true;
                for &c in &grp.components_0based {
                    if !ext_set.contains(&c) {
                        continue;
                    }
                    match ordered[prev_pos..].iter().position(|&e| e == c) {
                        None => {
                            valid = false;
                            break;
                        }
                        Some(off) => prev_pos = prev_pos + off + 1,
                    }
                }
                // assert(valid && "..."): compiled out (-DNDEBUG).
                let _ = valid;
            }

            lt.extruders = ordered;
        }
    }

    /// ToolOrdering.cpp:3510-3634 `void ToolOrdering::reorder_extruders_for_minimum_flush_volume(bool reorder_first_layer)`.
    /// `print` is `m_print`; `m_print_config_ptr` is `print_config_ptr`, and the configuration of
    /// `m_print_object_ptr->print()` is `print.config()` when `print_object_ptr` is set.
    pub fn reorder_extruders_for_minimum_flush_volume(
        &mut self,
        print: &dyn ToolOrderingPrint,
        reorder_first_layer: bool,
    ) -> Result<(), crate::filament_group::FilamentGroupError> {
        use crate::multi_nozzle_utils::{CreateFromLayerMaps, LayeredNozzleGroupResult};
        use crate::print_config::enums::FilamentMapMode;

        let print_config_arc: Option<std::sync::Arc<ToolOrderingPrintConfig>> = match &self.print_config_ptr {
            Some(p) => Some(p.0.clone()),
            None => None,
        };
        let print_config: &ToolOrderingPrintConfig = match &print_config_arc {
            Some(p) => p,
            None if self.print_object_ptr => print.config(),
            None => return Ok(()),
        };

        if self.layer_tools.is_empty() {
            return Ok(());
        }

        let mut filament_sequences: Vec<Vec<u32>> = Vec::new();

        let mut grouping_result = LayeredNozzleGroupResult::default();

        // Stage 1: 准备冲刷矩阵
        let nozzle_flush_mtx = group_reorder::prepare_flush_matrices(print_config);
        // Stage 2: 准备层信息与不可打印限制
        let layer_data = self.collect_layer_and_unprintable_data(print);
        // Stage 3: 生成排序所需的context
        let ordering_context = self.prepare_ordering_context(print_config, reorder_first_layer);

        // 不支持选料器或手动模式
        if !print.is_dynamic_group_reorder() {
            // 逐层打印时，在生成toolodering时分组，统计相关信息并写回
            if !print.is_sequential_print() {
                // fmmNozzleManual(H2C)：在整盘全集上把旧分组 key 的 filament_nozzle_map 解析成下标
                print.resolve_manual_nozzle_map(&layer_data.used_filaments).map_err(crate::filament_group::FilamentGroupError::Runtime)?;
                // Stage 4: 计算分组
                grouping_result = ToolOrdering::get_recommended_filament_maps(
                    print,
                    &layer_data.layer_filaments,
                    FilamentMapMode::from_i32(cfg_int(print_config, "filament_map_mode")),
                    &layer_data.physical_unprintables,
                    &layer_data.geometric_unprintables,
                    &layer_data.filament_unprintable_volumes,
                    &crate::multi_nozzle_utils::UnorderedMapInt::new(),
                )?;
            } else {
                // 逐件打印时，分组已经被调用过了，直接从print获取结果
                grouping_result = (*print.get_layered_nozzle_group_result().expect("*(m_print->get_layered_nozzle_group_result()) of a null pointer")).clone();
            }
            // 计算排序
            filament_sequences = self.execute_filament_ordering(print_config, &grouping_result, &layer_data, &ordering_context, &nozzle_flush_mtx);
        }

        // 支持选料器且为自动模式
        if print.is_dynamic_group_reorder() {
            let mut grouping_context = group_reorder::build_filament_group_context(
                print,
                &layer_data.layer_filaments,
                &layer_data.physical_unprintables,
                &layer_data.geometric_unprintables,
                &layer_data.filament_unprintable_volumes,
                FilamentMapMode::from_i32(cfg_int(print.config(), "filament_map_mode")),
                self.initial_nozzle_status.get_nozzle_filament_map(),
            )?;

            // 时间预估器中存储的材料在不同挤出机的打印时间是全局的，不适用于当前分区间的分组方法
            grouping_context.speed_info.group_with_time = false;

            self.nozzle_status.assign_from(&self.initial_nozzle_status);
            let mut nozzle_status = std::mem::take(&mut self.nozzle_status);
            let dynamic_plan_res = plan_filament_mapping_and_order_by_combo_ranges(
                print,
                &grouping_context,
                &ordering_context,
                FilamentMapMode::FmmAutoForFlush,
                &layer_data.physical_unprintables,
                &layer_data.geometric_unprintables,
                &layer_data.filament_unprintable_volumes,
                Some(&mut nozzle_status),
            );
            self.nozzle_status = nozzle_status;
            let dynamic_plan_res = dynamic_plan_res?;
            //auto dynamic_plan_res = plan_filament_nozzle_mapping_and_order(grouping_context);

            let mut nozzle_map_per_layer: Vec<Vec<i32>> = Vec::new();
            for res in &dynamic_plan_res {
                filament_sequences.push(res.fil_order.iter().map(|&f| f as u32).collect());
                nozzle_map_per_layer.push(res.fil_nozzle_match.clone());
            }

            let result = <LayeredNozzleGroupResult as CreateFromLayerMaps>::create(
                &nozzle_map_per_layer,
                &grouping_context.nozzle_info.nozzle_list,
                &layer_data.used_filaments,
                &filament_sequences,
                cfg_bool(print_config, "handle_hotend_as_extruder"),
            );
            grouping_result = result.unwrap_or_default();
        }

        if !print.is_sequential_print() {
            print.set_nozzle_group_result(std::sync::Arc::new(grouping_result.clone()));
            if !grouping_result.is_support_dynamic_nozzle_map() {
                // fmmNozzleManual keeps explicit user nozzle-map semantics. fmmManual may write back the
                // computed concrete nozzle map because its custom grouping can be resolved by backend.
                let keep_user_nozzle_map = cfg_bool(print_config, "handle_hotend_as_extruder")
                    && cfg_int(print_config, "filament_map_mode") == FilamentMapMode::FmmNozzleManual as i32;
                print.update_filament_maps_to_config(
                    crate::filament_group_utils::update_used_filament_values(
                        &cfg_ints(print_config, "filament_map"),
                        &grouping_result.get_extruder_map(false, -1),
                        &layer_data.used_filaments,
                    ),
                    crate::filament_group_utils::update_used_filament_values(
                        &cfg_ints(print_config, "filament_volume_map"),
                        &grouping_result.get_volume_map(-1),
                        &layer_data.used_filaments,
                    ),
                    if keep_user_nozzle_map { cfg_ints(print_config, "filament_nozzle_map") } else { grouping_result.get_nozzle_map(-1) },
                );
            } else {
                print.update_to_config_by_nozzle_group_result(&grouping_result);
            }
        }

        self.nozzle_group_result = grouping_result.clone();

        // Stage 6: 统计其它模式的信息
        self.calculate_and_store_statistics(
            print,
            print_config,
            &grouping_result,
            &layer_data,
            &ordering_context,
            &nozzle_flush_mtx,
            &filament_sequences,
        )?;

        // Stage 7: 应用排序结果
        for (i, seq) in filament_sequences.into_iter().enumerate() {
            self.layer_tools[i].extruders = seq;
        }
        Ok(())
    }

    /// ToolOrdering.cpp:3776-3790 `ToolOrdering::tools_for_layer`: the LayerTools with the closest
    /// print_z, as an index into the layers.
    pub fn tools_for_layer_index(&self, print_z: f64) -> usize {
        // std::lower_bound(..., LayerTools(print_z - EPSILON)): the first with !(lt.print_z < print_z - EPSILON)
        let key = print_z - EPSILON;
        let mut it = self.layer_tools.partition_point(|lt| lt.print_z < key);
        // assert(it_layer_tools != m_layer_tools.end()): compiled out (-DNDEBUG).
        let mut dist_min = (self.layer_tools[it].print_z - print_z).abs();
        it += 1;
        while it != self.layer_tools.len() {
            let d = (self.layer_tools[it].print_z - print_z).abs();
            if d >= dist_min {
                break;
            }
            dist_min = d;
            it += 1;
        }
        it -= 1;
        // assert(dist_min < EPSILON): compiled out (-DNDEBUG).
        it
    }

    /// ToolOrdering.cpp:3776-3790 `const LayerTools& ToolOrdering::tools_for_layer(coordf_t print_z) const`
    pub fn tools_for_layer(&self, print_z: f64) -> &LayerTools {
        &self.layer_tools[self.tools_for_layer_index(print_z)]
    }

    /// ToolOrdering.hpp:343 `LayerTools& tools_for_layer(coordf_t print_z)`
    pub fn tools_for_layer_mut(&mut self, print_z: f64) -> &mut LayerTools {
        let i = self.tools_for_layer_index(print_z);
        &mut self.layer_tools[i]
    }

    /// ToolOrdering.cpp:3792-3808 `ToolOrdering::first_layer_extruders`: 0-based filaments that extrude
    /// onto the bed.
    pub fn first_layer_extruders(&self) -> Vec<u32> {
        let mut extruders: Vec<u32> = Vec::new();
        if self.layer_tools.is_empty() {
            return extruders;
        }
        extruders = self.layer_tools[0].extruders.clone();
        for &(print_z, filament_1based) in &self.bed_contact_filaments {
            let layer_tools = self.tools_for_layer(print_z);
            let filament = filament_1based.wrapping_sub(1);
            if let Some(group) = layer_tools.mixed_group_by_slot(filament) {
                extruders.extend_from_slice(&group.components_0based);
            } else {
                extruders.push(layer_tools.resolve_mixed(filament));
            }
        }
        sort_remove_duplicates_u32(&mut extruders);
        extruders
    }
}

/// libslic3r.h `sort_remove_duplicates` of a `std::vector<size_t>`.
fn sort_remove_duplicates_usize(v: &mut Vec<usize>) {
    v.sort_unstable();
    v.dedup();
}

/// `std::llround`: round half away from zero to a `long long`.
fn llround(v: f64) -> i64 {
    v.round() as i64
}

/// `std::sscanf(s, "%f,%f", &v0, &v1) == 2` (C locale): the two floats when both convert.
fn sscanf_two_floats(s: &str) -> Option<(f32, f32)> {
    let (v0, rest) = scan_float(s)?;
    let rest = rest.strip_prefix(',')?;
    let (v1, _) = scan_float(rest)?;
    Some((v0, v1))
}

/// `%f` of `sscanf`: leading white space skipped, then the longest prefix `strtof` converts.
fn scan_float(s: &str) -> Option<(f32, &str)> {
    let t = s.trim_start_matches(|c: char| matches!(c, ' ' | '\t' | '\n' | '\x0b' | '\x0c' | '\r'));
    let b = t.as_bytes();
    let mut i = 0usize;
    if i < b.len() && (b[i] == b'+' || b[i] == b'-') {
        i += 1;
    }
    let mut digits = 0;
    while i < b.len() && b[i].is_ascii_digit() {
        i += 1;
        digits += 1;
    }
    if i < b.len() && b[i] == b'.' {
        i += 1;
        while i < b.len() && b[i].is_ascii_digit() {
            i += 1;
            digits += 1;
        }
    }
    if digits == 0 {
        return None;
    }
    if i < b.len() && (b[i] == b'e' || b[i] == b'E') {
        let mut j = i + 1;
        if j < b.len() && (b[j] == b'+' || b[j] == b'-') {
            j += 1;
        }
        let exp_start = j;
        while j < b.len() && b[j].is_ascii_digit() {
            j += 1;
        }
        if j > exp_start {
            i = j;
        }
    }
    let v: f32 = t[..i].parse().ok()?;
    Some((v, &t[i..]))
}

impl WipingExtrusions {
    /// ToolOrdering.hpp:53-57 `WipingExtrusions::is_overriddable_and_mark`, over the values
    /// `is_overriddable()` reads (see the free `is_overriddable`).
    pub fn is_overriddable_and_mark(
        &mut self,
        role: crate::extrusion_entity::ExtrusionRole,
        filament_soluble: bool,
        flush_into_objects: bool,
        flush_into_infill: bool,
    ) -> bool {
        let out = is_overriddable(role, filament_soluble, flush_into_objects, flush_into_infill);
        self.something_overridable |= out;
        out
    }

    /// ToolOrdering.hpp:60-65 `WipingExtrusions::is_obj_overriddable_and_mark`
    pub fn is_obj_overriddable_and_mark(
        &mut self,
        role: crate::extrusion_entity::ExtrusionRole,
        flush_into_objects: bool,
        flush_into_infill: bool,
    ) -> bool {
        let out = is_obj_overriddable(role, flush_into_objects, flush_into_infill);
        self.something_overridable |= out;
        out
    }

    /// ToolOrdering.hpp:69-73 `WipingExtrusions::is_support_overriddable_and_mark`
    pub fn is_support_overriddable_and_mark(
        &mut self,
        role: crate::extrusion_entity::ExtrusionRole,
        flush_into_support: bool,
        support_filament: u32,
        support_interface_filament: u32,
    ) -> bool {
        let out = is_support_overriddable(role, flush_into_support, support_filament, support_interface_filament);
        self.something_overridable |= out;
        out
    }

    /// ToolOrdering.cpp:4133-4141 `WipingExtrusions::get_support_extruder_overrides`: -1 when there
    /// is none.
    pub fn get_support_extruder_overrides(&self, object_id: u64) -> i32 {
        match self.support_overrides.get(&object_id) {
            Some(&e) => e,
            None => -1,
        }
    }

    /// ToolOrdering.cpp:4143-4150 `WipingExtrusions::get_support_interface_extruder_overrides`
    pub fn get_support_interface_extruder_overrides(&self, object_id: u64) -> i32 {
        match self.support_interface_overrides.get(&object_id) {
            Some(&e) => e,
            None => -1,
        }
    }
}

// ============================================================================
// The members of ToolOrdering that read a PrintObject's layers (ToolOrdering.cpp:499-1205): the two
// constructors, generate_first_layer_tool_order and collect_extruders, over the view below.
// ============================================================================

/// What collect_extruders reads of one `AdaptiveMergePatch` of a LayerRegion.
#[derive(Debug, Clone, Default)]
pub struct ToolOrderingMergePatch {
    /// `patch.group_top`
    pub group_top: bool,
    /// `patch.bottom_idx`
    pub bottom_idx: usize,
    /// `patch.area().empty()`
    pub area_empty: bool,
    /// `patch.merged_height`
    pub merged_height: f64,
}

/// What ToolOrdering reads of one `LayerRegion`.
#[derive(Debug, Clone)]
pub struct ToolOrderingLayerRegion {
    /// `layerm->region()`
    pub region: crate::print_region::PrintRegion,
    /// `layerm->region().gradient_volume_id()`: `Some(id)` when valid.
    pub gradient_volume_id: Option<usize>,
    /// `layerm->perimeters`
    pub perimeters: crate::extrusion_entity::ExtrusionEntityCollection,
    /// `layerm->fills`
    pub fills: crate::extrusion_entity::ExtrusionEntityCollection,
    /// `layerm->raw_slices`
    pub raw_slices: Vec<crate::geometry::ExPolygon>,
    /// `layerm->slices.empty()`
    pub slices_empty: bool,
    /// `layerm->adaptive_merge_patches`
    pub adaptive_merge_patches: Vec<ToolOrderingMergePatch>,
}

impl ToolOrderingLayerRegion {
    /// R934 TOOL_ORDERING_PERIMETER_ISLANDS (faithful_gate): `layerm->perimeters` as the pin's
    /// ToolOrdering reads it. collect_extruders casts every entry to a collection
    /// (ToolOrdering.cpp:943-951 `dynamic_cast<const ExtrusionEntityCollection&>(*eec)`, a
    /// `std::bad_cast` on anything else) and so does mark_wiping_extrusions (:3985-3987): the pin's
    /// entries are the island collections PerimeterGenerator appends (PerimeterGenerator.cpp:1340-1342
    /// `this->loops->append(entities)`, :1837-1838 `this->loops->append(extrusion_coll)`), one per
    /// island, and the role (`eec.role()`, erMixed when walls and gap fill share an island), the
    /// filament (`LayerTools::extruder(eec, region)`, :160-205) and the override unit
    /// (`set_extruder_override(fill, ...)` with `fill->total_volume()`, :3988-3992) are the island's.
    /// This crate's LayerRegion keeps the walls flat with the island ranges beside them
    /// (`LayerRegion::perimeter_islands`), so the view takes `LayerRegion::perimeter_island_collections()`.
    /// With `TOOL_ORDERING_PERIMETER_ISLANDS=0` it is the flat list as stored (the earlier arm, on
    /// which collect_extruders' cast panics as the pin's throws).
    pub fn perimeters_from_layer_region(
        layerm: &crate::layer::LayerRegion,
    ) -> crate::extrusion_entity::ExtrusionEntityCollection {
        if crate::faithful_gate("TOOL_ORDERING_PERIMETER_ISLANDS") {
            layerm.perimeter_island_collections()
        } else {
            layerm.perimeters.clone()
        }
    }

    /// R934: the `WipingEntity` view (ToolOrdering.cpp:3985-3992) of `perimeters` taken by
    /// `perimeters_from_layer_region`: one entry per top-level entry, with its role, total volume and
    /// filament (`extruder`, zero based, as `m_layer_tools->extruder(eec, region)` gives it; the caller
    /// passes the region's wall filament). Ids are `id_base | index`, as print.rs' fills view keys them.
    pub fn wiping_perimeters(
        perimeters: &crate::extrusion_entity::ExtrusionEntityCollection,
        id_base: u64,
        extruder: u32,
    ) -> Vec<WipingEntity> {
        use crate::extrusion_entity::ExtrusionEntityType as T;
        perimeters
            .entities
            .iter()
            .enumerate()
            .map(|(i, e)| {
                let (role, total_volume) = match e {
                    T::Path(p) => (p.role, p.total_volume()),
                    T::MultiPath(m) => (m.role(), m.total_volume()),
                    T::Loop(l) => (l.role(), l.total_volume()),
                    T::Collection(c) => (c.role(), c.total_volume()),
                };
                WipingEntity { id: id_base | i as u64, role, total_volume, extruder }
            })
            .collect()
    }
}

/// What ToolOrdering reads of one `Layer` of an object.
#[derive(Debug, Clone)]
pub struct ToolOrderingLayer {
    /// `layer->id()`
    pub id: usize,
    /// `layer->print_z`
    pub print_z: f64,
    /// `layer->height`
    pub height: f64,
    /// `layer->has_extrusions()`
    pub has_extrusions: bool,
    /// `layer->regions()`
    pub regions: Vec<ToolOrderingLayerRegion>,
}

/// What ToolOrdering reads of one `SupportLayer` of an object.
#[derive(Debug, Clone)]
pub struct ToolOrderingSupportLayer {
    /// `support_layer->print_z`
    pub print_z: f64,
    /// `support_layer->support_fills`
    pub support_fills: crate::extrusion_entity::ExtrusionEntityCollection,
    /// `support_layer->planned_filament_id`
    pub planned_filament_id: i32,
    /// `support_layer->planned_interface_filament_id`
    pub planned_interface_filament_id: i32,
}

/// What ToolOrdering reads of a `PrintObject` (its print is the `ToolOrderingPrint` the functions
/// take beside it).
#[derive(Debug, Clone)]
pub struct ToolOrderingObject {
    /// Stands for the `const PrintObject*` keying the per-object maps (see `VolumeKey`).
    pub key: usize,
    /// `object.layers()`
    pub layers: Vec<ToolOrderingLayer>,
    /// `object.support_layers()`
    pub support_layers: Vec<ToolOrderingSupportLayer>,
    /// `object.slicing_parameters().raft_layers()`
    pub raft_layers: usize,
    /// What `object_max_layer_height(object)` reads.
    pub max_layer_height_inputs: ObjectMaxLayerHeightInputs,
    /// `object.printing_region(i).config().bridge_flow.value` for every printing region
    pub printing_region_bridge_flows: Vec<f64>,
    /// `object.config().support_filament.value`
    pub support_filament: u32,
    /// `object.config().support_interface_filament.value`
    pub support_interface_filament: u32,
    /// `object.config().support_interface_not_for_body`
    pub support_interface_not_for_body: bool,
    /// `object.config().flush_into_objects`
    pub flush_into_objects: bool,
    /// `object.config().flush_into_infill`
    pub flush_into_infill: bool,
    /// `object.config().flush_into_support`
    pub flush_into_support: bool,
    /// `object.config().line_width`
    pub line_width: f64,
    /// `object.has_brim()`
    pub has_brim: bool,
}

/// What collect_extruders writes into the PrintObject (`const_cast`, ToolOrdering.cpp:1108-1110).
#[derive(Debug, Clone, Default, PartialEq)]
pub struct ToolOrderingObjectOutputs {
    /// `object_first_layer_wall_extruders`
    pub object_first_layer_wall_extruders: Vec<i32>,
    /// `object_brim_filament`
    pub object_brim_filament: i32,
    /// `object_brim_print_z`
    pub object_brim_print_z: f64,
}

/// What the constructors and collect_extruders read of `Print` beyond `ToolOrderingPrint`.
pub trait ToolOrderingPrintObjects: ToolOrderingPrint {
    /// The print as a `ToolOrderingPrint` (`self`).
    fn as_tool_ordering_print(&self) -> &dyn ToolOrderingPrint;
    /// `print.objects()`
    fn objects(&self) -> Vec<&ToolOrderingObject>;
    /// `print.has_skirt()`
    fn has_skirt(&self) -> bool;
    /// `print.extruders()`
    fn extruders(&self) -> Vec<u32>;
    /// `nozzle_diameter_for_filament(print.config(), filament)`, 1 based filament id.
    fn nozzle_diameter_for_filament(&self, filament: u32) -> f32;
}

/// `ConfigOptionFloat` of a value, for `Flow::extrusion_width_for_nozzle`.
fn config_option_float(v: f64) -> crate::config::ConfigOptionFloat {
    crate::config::ConfigOptionFloat::with_value(v)
}

/// `!offset_ex(expoly, -0.2 * scale_(line_width)).empty()`: the island survives the shrink.
fn survives_first_layer_shrink(expoly: &crate::geometry::ExPolygon, line_width: f64) -> bool {
    // `offset_ex(const ExPolygon&, const float delta)`: the double delta converted to float.
    let delta = (-0.2 * (line_width / crate::libslic3r::SCALING_FACTOR)) as f32 as f64;
    !crate::clipper_utils::offset_expolygons_clib_scaled(std::slice::from_ref(expoly), delta, crate::clipper_utils::OffsetJoinType::Miter)
        .is_empty()
}

/// `print.full_print_config().option<ConfigOptionInts>("first_layer_print_sequence")`
fn first_layer_print_sequence(full_print_config: &ToolOrderingPrintConfig) -> Option<Vec<i32>> {
    use crate::config::{ConfigOptionResolver, ConfigOptionResolverTyped};
    let _ = full_print_config.option("first_layer_print_sequence")?;
    full_print_config.option_typed::<crate::config::ConfigOptionInts>("first_layer_print_sequence").map(|o| o.values.clone())
}

/// ToolOrdering.cpp:700-727, the tail both overloads share: the extruders by decreasing minimal
/// area, then sorted by the first layer print sequence when it names them all.
fn first_layer_tool_order_from_areas(min_areas_per_extruder: &BTreeMap<i32, f64>, full_print_config: &ToolOrderingPrintConfig) -> Vec<u32> {
    let mut tool_order: Vec<u32> = Vec::new();
    for (&ape_first, _) in min_areas_per_extruder {
        let mut iter = 0usize;
        while iter < tool_order.len() {
            if min_areas_per_extruder[&(tool_order[iter] as i32)] < min_areas_per_extruder[&ape_first] {
                break;
            }
            iter += 1;
        }

        tool_order.insert(iter, ape_first as u32);
    }

    if let Some(print_sequence_1st) = first_layer_print_sequence(full_print_config) {
        if print_sequence_1st.len() >= tool_order.len() {
            // std::sort with a comparator that is no strict weak ordering when an extruder is not in
            // the sequence: libc++'s algorithm decides the result.
            crate::clipper::libcxx_sort(&mut tool_order, |&lh, &rh| {
                let lh_it = print_sequence_1st.iter().position(|&v| v == lh as i32);
                let rh_it = print_sequence_1st.iter().position(|&v| v == rh as i32);

                match (lh_it, rh_it) {
                    (Some(l), Some(r)) => l < r,
                    _ => false,
                }
            });
        }
    }

    tool_order
}

/// The two overloads of `ToolOrdering::generate_first_layer_tool_order` (ToolOrdering.hpp:434-435).
pub trait GenerateFirstLayerToolOrder<T> {
    fn generate_first_layer_tool_order(&mut self, arg: T) -> Vec<u32>;
}

impl<'a> GenerateFirstLayerToolOrder<&'a dyn ToolOrderingPrintObjects> for ToolOrdering {
    /// ToolOrdering.cpp:641-728 `generate_first_layer_tool_order(const Print& print)`
    fn generate_first_layer_tool_order(&mut self, print: &'a dyn ToolOrderingPrintObjects) -> Vec<u32> {
        use crate::print_config::config_def::{multi_nozzle_diameter_mode_enabled, primary_nozzle_diameter};
        let tool_order: Vec<u32> = Vec::new();
        let mut min_areas_per_extruder: BTreeMap<i32, f64> = BTreeMap::new();

        // This global heuristic spans all objects, so use the largest configured nozzle conservatively.
        let nozzle_diameter = cfg_floats(print.config(), "nozzle_diameter");
        if nozzle_diameter.is_empty() {
            return tool_order;
        }
        let reference_nozzle = nozzle_diameter.iter().copied().fold(f64::NEG_INFINITY, |a, b| if b > a { b } else { a });
        let first_layer_line_width = crate::flow::Flow::extrusion_width_for_nozzle(
            &config_option_float(cfg_float(print.config(), "initial_layer_line_width")),
            primary_nozzle_diameter(print.full_print_config()) as f32,
            reference_nozzle as f32,
            multi_nozzle_diameter_mode_enabled(print.full_print_config()),
        );

        for object in print.objects() {
            let mut target_layer: Option<&ToolOrderingLayer> = None;
            'layers: for layer in &object.layers {
                for layerm in &layer.regions {
                    for expoly in &layerm.raw_slices {
                        if survives_first_layer_shrink(expoly, first_layer_line_width) {
                            target_layer = Some(layer);
                            break 'layers;
                        }
                    }
                }
            }

            let Some(target_layer) = target_layer else {
                return tool_order;
            };

            for layerm in &target_layer.regions {
                let extruder_id = layerm.region.config().wall_filament as i32;

                for expoly in &layerm.raw_slices {
                    if !survives_first_layer_shrink(expoly, first_layer_line_width) {
                        continue;
                    }

                    let contour_area = expoly.contour.area();
                    match min_areas_per_extruder.get_mut(&extruder_id) {
                        None => {
                            min_areas_per_extruder.insert(extruder_id, contour_area);
                        }
                        Some(v) => {
                            if contour_area < *v {
                                *v = contour_area;
                            }
                        }
                    }
                }
            }
        }

        first_layer_tool_order_from_areas(&min_areas_per_extruder, print.full_print_config())
    }
}

/// `(print, object)`: the object overload of `generate_first_layer_tool_order` with the print it
/// belongs to.
impl<'a> GenerateFirstLayerToolOrder<(&'a dyn ToolOrderingPrintObjects, &'a ToolOrderingObject)> for ToolOrdering {
    /// ToolOrdering.cpp:730-812 `generate_first_layer_tool_order(const PrintObject& object)`
    fn generate_first_layer_tool_order(&mut self, arg: (&'a dyn ToolOrderingPrintObjects, &'a ToolOrderingObject)) -> Vec<u32> {
        use crate::print_config::config_def::{multi_nozzle_diameter_mode_enabled, primary_nozzle_diameter};
        let (print, object) = arg;
        let tool_order: Vec<u32> = Vec::new();
        let mut min_areas_per_extruder: BTreeMap<i32, f64> = BTreeMap::new();
        let mut target_layer: Option<&ToolOrderingLayer> = None;
        'layers: for layer in &object.layers {
            for layerm in &layer.regions {
                let wall_filament = layerm.region.config().wall_filament as u32;
                let object_line_width = crate::flow::Flow::extrusion_width_for_nozzle(
                    &config_option_float(object.line_width),
                    primary_nozzle_diameter(print.full_print_config()) as f32,
                    print.nozzle_diameter_for_filament(wall_filament),
                    multi_nozzle_diameter_mode_enabled(print.full_print_config()),
                );
                for expoly in &layerm.raw_slices {
                    if survives_first_layer_shrink(expoly, object_line_width) {
                        target_layer = Some(layer);
                        break 'layers;
                    }
                }
            }
        }

        let Some(target_layer) = target_layer else {
            return tool_order;
        };

        for layerm in &target_layer.regions {
            let extruder_id = layerm.region.config().wall_filament as i32;
            let object_line_width = crate::flow::Flow::extrusion_width_for_nozzle(
                &config_option_float(object.line_width),
                primary_nozzle_diameter(print.full_print_config()) as f32,
                print.nozzle_diameter_for_filament(extruder_id as u32),
                multi_nozzle_diameter_mode_enabled(print.full_print_config()),
            );
            for expoly in &layerm.raw_slices {
                if !survives_first_layer_shrink(expoly, object_line_width) {
                    continue;
                }

                let contour_area = expoly.contour.area();
                match min_areas_per_extruder.get_mut(&extruder_id) {
                    None => {
                        min_areas_per_extruder.insert(extruder_id, contour_area);
                    }
                    Some(v) => {
                        if contour_area < *v {
                            *v = contour_area;
                        }
                    }
                }
            }
        }

        first_layer_tool_order_from_areas(&min_areas_per_extruder, print.full_print_config())
    }
}

/// The two overloads of `ToolOrdering::sort_and_build_data` (ToolOrdering.hpp:302-303): over a `Print`
/// (`&dyn ToolOrderingPrintObjects`) and over a `PrintObject` (`(object, object.print())`). Called as
/// `SortAndBuildData::<&dyn ToolOrderingPrintObjects>::sort_and_build_data(&mut ordering, print, ...)`.
pub trait SortAndBuildData<T> {
    fn sort_and_build_data(&mut self, arg: T, first_extruder: u32, prime_multi_material: bool) -> Result<(), crate::filament_group::FilamentGroupError>;
}

impl<'a> SortAndBuildData<&'a dyn ToolOrderingPrintObjects> for ToolOrdering {
    /// ToolOrdering.cpp:447-476 `void ToolOrdering::sort_and_build_data(const Print& print, unsigned int first_extruder,
    /// bool prime_multi_material)`. `print` is also `m_print`.
    fn sort_and_build_data(
        &mut self,
        print: &'a dyn ToolOrderingPrintObjects,
        first_extruder: u32,
        prime_multi_material: bool,
    ) -> Result<(), crate::filament_group::FilamentGroupError> {
        // if first extruder is -1, we can decide the first layer tool order before doing reorder function
        // so we shouldn't reorder first layer in reorder function
        let reorder_first_layer = first_extruder != u32::MAX;
        self.resolve_mixed_filaments(print.config());
        self.reorder_extruders_for_minimum_flush_volume(print.as_tool_ordering_print(), reorder_first_layer)?;
        self.enforce_mixed_component_order();
        self.sorted = true;

        let mut max_layer_height: f64 = 0.0;
        let mut object_bottom_z: f64 = 0.0;
        for object in print.objects() {
            for layer in &object.layers {
                if layer.has_extrusions {
                    object_bottom_z = layer.print_z - layer.height;
                    break;
                }
            }
            max_layer_height = max_layer_height.max(object_max_layer_height(&object.max_layer_height_inputs));
        }

        max_layer_height = calc_max_layer_height(print.config(), max_layer_height);

        self.collect_extruder_statistics(prime_multi_material);

        ToolOrderingCpp::fill_wipe_tower_partitions(self, print.config(), object_bottom_z, max_layer_height);

        self.calc_most_used_extruder(print.config(), print.as_tool_ordering_print());
        Ok(())
    }
}

impl<'a> SortAndBuildData<(&'a ToolOrderingObject, &'a dyn ToolOrderingPrintObjects)> for ToolOrdering {
    /// ToolOrdering.cpp:478-496 `void ToolOrdering::sort_and_build_data(const PrintObject& object, unsigned int first_extruder,
    /// bool prime_multi_material)`. `print` is `object.print()` (and `m_print`).
    fn sort_and_build_data(
        &mut self,
        arg: (&'a ToolOrderingObject, &'a dyn ToolOrderingPrintObjects),
        _first_extruder: u32,
        prime_multi_material: bool,
    ) -> Result<(), crate::filament_group::FilamentGroupError> {
        let (object, print) = arg;
        // if first extruder is -1, we can decide the first layer tool order before doing reorder function
        // so we shouldn't reorder first layer in reorder function
        //bool reorder_first_layer = (first_extruder != (unsigned int)(-1));
        let reorder_first_layer = false; // when by object , the first layer of each object should use the best order, regard less of the filament change for better bed contect
        self.resolve_mixed_filaments(print.config());
        self.reorder_extruders_for_minimum_flush_volume(print.as_tool_ordering_print(), reorder_first_layer)?;
        self.enforce_mixed_component_order();
        self.sorted = true;

        let max_layer_height = calc_max_layer_height(print.config(), object_max_layer_height(&object.max_layer_height_inputs));

        self.collect_extruder_statistics(prime_multi_material);

        let front = &object.layers[0];
        ToolOrderingCpp::fill_wipe_tower_partitions(self, print.config(), front.print_z - front.height, max_layer_height);

        self.calc_most_used_extruder(print.config(), print.as_tool_ordering_print());
        Ok(())
    }
}

impl ToolOrdering {
    /// ToolOrdering.cpp:829-1205 `ToolOrdering::collect_extruders`: the extruders required to print
    /// the layers of an object. Returns what the C++ writes into the object.
    pub fn collect_extruders(
        &mut self,
        object: &ToolOrderingObject,
        print: &dyn ToolOrderingPrintObjects,
        per_layer_extruder_switches: &[(f64, u32)],
    ) -> ToolOrderingObjectOutputs {
        use crate::extrusion_entity::ExtrusionRole;
        let print_config = print.config();
        // Extruder overrides are ordered by print_z.
        let mut it_per_layer_extruder_override = 0usize;
        let mut extruder_override: u32 = 0;

        // Pre-compute 1-based IDs of mixed filament slots for per-object tracking.
        let mut mixed_slots_1based: BTreeSet<u32> = BTreeSet::new();
        let mut gradient_slots_1based: BTreeSet<u32> = BTreeSet::new();
        let mut per_part_slots_1based: BTreeSet<u32> = BTreeSet::new();
        {
            let is_mixed = cfg_bools(print_config, "filament_is_mixed");
            let grad_flags = cfg_bools(print_config, "filament_mixed_gradient");
            let per_part_flags = cfg_bools(print_config, "filament_mixed_gradient_per_part");
            let comp_strs = cfg_strings(print_config, "filament_mixed_components");
            for i in 0..is_mixed.len() {
                if is_mixed[i] == 0 {
                    continue;
                }
                let comps = crate::filament_mixer::parse_mixed_components(if i < comp_strs.len() { &comp_strs[i] } else { "" });
                if comps.len() < 2 {
                    continue;
                }
                mixed_slots_1based.insert((i + 1) as u32);
                // Gradient/per-part are only defined for 2-component slots.
                if comps.len() != 2 {
                    continue;
                }
                if i >= grad_flags.len() || grad_flags[i] == 0 {
                    continue;
                }
                gradient_slots_1based.insert((i + 1) as u32);
                if i < per_part_flags.len() && per_part_flags[i] != 0 {
                    per_part_slots_1based.insert((i + 1) as u32);
                }
            }
        }

        // BBS: collect first layer extruders of an object's wall, which will be used by brim generator
        let mut layer_count = 0;
        let mut first_layer_extruders: Vec<i32> = Vec::new();
        // The lowest of them, the one whose print_z the brim is printed at.
        let mut brim_filament: i32 = 0;
        let mut brim_print_z: f64 = 0.0;
        // Only filaments actually emitted on the first print layer may schedule a brim-layer tower visit.
        let mut bed_contact_extruders: Vec<u32> = Vec::new();

        // Only adaptive merging moves bed contact above the physical first layer.
        let mut max_contact_z: f64 = -1.0;
        if print.uses_multiple_nozzle_diameters() {
            let mut max_nozzle = 0.0f64;
            for filament in 0..cfg_floats(print_config, "filament_diameter").len() {
                max_nozzle = max_nozzle.max(print.nozzle_diameter_for_filament((filament + 1) as u32) as f64);
            }
            let mut max_bridge_flow = 1.0f64;
            for &bridge_flow in &object.printing_region_bridge_flows {
                max_bridge_flow = max_bridge_flow.max(bridge_flow);
            }
            max_contact_z = max_nozzle * max_bridge_flow + EPSILON;
        }

        let filament_soluble = cfg_bools(print_config, "filament_soluble");
        let first_bottom_z = object.layers.first().map(|l| l.print_z - l.height);

        // Collect the object extruders.
        for layer in &object.layers {
            let layer_idx = self.tools_for_layer_index(layer.print_z);
            let may_touch_bed = layer.print_z <= max_contact_z;

            // Adaptive merging leaves the swallowed grid layers in object.layers(), but moves all
            // of their extrusion to the group top.
            if layer.has_extrusions {
                self.object_all_layer_indices.entry(object.key).or_default().push(layer_idx);
            }

            // Override extruder with the next
            while it_per_layer_extruder_override < per_layer_extruder_switches.len()
                && per_layer_extruder_switches[it_per_layer_extruder_override].0 < layer.print_z + EPSILON
            {
                extruder_override = per_layer_extruder_switches[it_per_layer_extruder_override].1;
                it_per_layer_extruder_override += 1;
            }

            // Store the current extruder override (set to zero if no overriden), so that
            // layer_tools.wiping_extrusions().is_overridable_and_mark() will use it.
            self.layer_tools[layer_idx].extruder_override = extruder_override;

            // Snapshot extruders before this object's regions to track new additions.
            let ext_snapshot = self.layer_tools[layer_idx].extruders.len();

            // What extruders are required to print this object layer?
            for layerm in &layer.regions {
                let region = &layerm.region;
                let starts_at_object_base =
                    layerm.adaptive_merge_patches.iter().any(|patch| patch.group_top && patch.bottom_idx == 0);

                // `record_mixed_height`
                let record_mixed_height = |this: &mut ToolOrdering, filament_1based: u32| {
                    if !mixed_slots_1based.contains(&filament_1based) {
                        return;
                    }
                    let mut height = layer.height;
                    for patch in &layerm.adaptive_merge_patches {
                        if patch.group_top && !patch.area_empty {
                            height = height.max(patch.merged_height);
                        }
                    }
                    if height <= 0.0 {
                        return;
                    }
                    let stored = this.mixed_layer_heights.entry(filament_1based - 1).or_default().entry(layer_idx).or_insert(0.0);
                    *stored = stored.max(height);
                };

                if !layerm.perimeters.entities.is_empty() {
                    let mut something_nonoverriddable = true;

                    if self.print_config_ptr.is_some() {
                        // in this case print->config().print_sequence != PrintSequence::ByObject (see ToolOrdering constructors)
                        something_nonoverriddable = false;
                        for eec in &layerm.perimeters.entities {
                            // let's check if there are nonoverriddable entities
                            let coll = expect_collection(eec);
                            let lt = &mut self.layer_tools[layer_idx];
                            let soluble = get_at(&filament_soluble, lt.extruder(coll, region) as usize) != 0;
                            if !lt.wiping_extrusions_mut().is_overriddable_and_mark(
                                coll.role(),
                                soluble,
                                object.flush_into_objects,
                                object.flush_into_infill,
                            ) {
                                something_nonoverriddable = true;
                            }
                        }
                    } else {
                        something_nonoverriddable = false;
                        for eec in &layerm.perimeters.entities {
                            // let's check if there are nonoverriddable entities
                            let coll = expect_collection(eec);
                            if !self.layer_tools[layer_idx].wiping_extrusions_mut().is_obj_overriddable_and_mark(
                                coll.role(),
                                object.flush_into_objects,
                                object.flush_into_infill,
                            ) {
                                something_nonoverriddable = true;
                            }
                        }
                    }

                    if something_nonoverriddable {
                        let wall_filament = if extruder_override == 0 { region.config().wall_filament as u32 } else { extruder_override };
                        self.layer_tools[layer_idx].extruders.push(wall_filament);
                        record_mixed_height(self, wall_filament);
                        if may_touch_bed
                            && wall_filament > 0
                            && layerm.perimeters.entities.iter().any(|e| extrusion_touches_bed(e, layer.print_z))
                        {
                            self.bed_contact_filaments.push((layer.print_z, wall_filament));
                        }
                        if layer_count == 0 || starts_at_object_base {
                            first_layer_extruders.push(wall_filament as i32);
                            // Only a band starting on the bed is as tall as its print_z, e.g. not the first layer above a raft.
                            if first_bottom_z.map_or(false, |z| z.abs() <= EPSILON)
                                && (brim_filament == 0
                                    || layer.print_z < brim_print_z - EPSILON
                                    || (layer.print_z < brim_print_z + EPSILON && (wall_filament as i32) < brim_filament))
                            {
                                brim_filament = wall_filament as i32;
                                brim_print_z = layer.print_z;
                            }
                        }
                        if (layer_count == 0 || starts_at_object_base)
                            && !object.layers.is_empty()
                            && first_bottom_z.map_or(false, |z| z.abs() <= EPSILON)
                            && !self.layer_tools.is_empty()
                            && (layer.print_z - self.layer_tools[0].print_z).abs() <= EPSILON
                        {
                            bed_contact_extruders.push(wall_filament);
                        }
                    }

                    self.layer_tools[layer_idx].has_object = true;
                }

                let mut something_nonoverriddable = false;
                // The filament every fill collection will actually be printed with, read back through the
                // very decision GCode::process_layer() makes.
                let mut fill_slots: Vec<u32> = Vec::new();
                let mut bed_fill_slots: Vec<u32> = Vec::new();
                for ee in &layerm.fills.entities {
                    // fill represents infill extrusions of a single island.
                    let fill = expect_collection(ee);
                    if !fill.entities.is_empty() {
                        fill_slots.push(self.layer_tools[layer_idx].filament_slot(fill, region));
                        if may_touch_bed
                            && *fill_slots.last().unwrap() > 0
                            && fill.entities.iter().any(|e| extrusion_touches_bed(e, layer.print_z))
                        {
                            bed_fill_slots.push(*fill_slots.last().unwrap());
                        }
                    }

                    if self.print_config_ptr.is_some() {
                        let lt = &mut self.layer_tools[layer_idx];
                        let soluble = get_at(&filament_soluble, lt.extruder(fill, region) as usize) != 0;
                        if !lt.wiping_extrusions_mut().is_overriddable_and_mark(
                            fill.role(),
                            soluble,
                            object.flush_into_objects,
                            object.flush_into_infill,
                        ) {
                            something_nonoverriddable = true;
                        }
                    } else if !self.layer_tools[layer_idx].wiping_extrusions_mut().is_obj_overriddable_and_mark(
                        fill.role(),
                        object.flush_into_objects,
                        object.flush_into_infill,
                    ) {
                        something_nonoverriddable = true;
                    }
                }

                if something_nonoverriddable {
                    for &slot in &fill_slots {
                        self.layer_tools[layer_idx].extruders.push(slot);
                        record_mixed_height(self, slot);
                    }
                    for &slot in &bed_fill_slots {
                        self.bed_contact_filaments.push((layer.print_z, slot));
                    }
                }
                if !fill_slots.is_empty() {
                    self.layer_tools[layer_idx].has_object = true;
                }
            }

            // Record mixed slot usage for this object at this layer.
            if !mixed_slots_1based.is_empty() {
                let mut seen: BTreeSet<u32> = BTreeSet::new();
                for ei in ext_snapshot..self.layer_tools[layer_idx].extruders.len() {
                    let ext_1based = self.layer_tools[layer_idx].extruders[ei];
                    if mixed_slots_1based.contains(&ext_1based) && seen.insert(ext_1based) {
                        self.mixed_object_layers.entry(ext_1based - 1).or_default().entry(object.key).or_default().push(layer_idx);
                    }
                }

                // A region swallowed by its own adaptive band emits nothing here, but its slot is still
                // printed through this step by the group top above.
                let object_layer_idx = layer.id.wrapping_sub(object.raft_layers);
                for layerm in &layer.regions {
                    let swallowed = layerm.adaptive_merge_patches.iter().any(|patch| {
                        !patch.group_top && !patch.area_empty && object_layer_idx >= patch.bottom_idx
                    });
                    if !swallowed {
                        continue;
                    }
                    let rcfg = layerm.region.config();
                    let role_slots: [u32; 3] = [
                        if extruder_override != 0 { extruder_override } else { rcfg.wall_filament as u32 },
                        if extruder_override != 0 { extruder_override } else { rcfg.sparse_infill_filament as u32 },
                        if extruder_override != 0 { extruder_override } else { rcfg.solid_infill_filament as u32 },
                    ];
                    let vol_id = layerm.gradient_volume_id;
                    for &ext_1based in &role_slots {
                        if !mixed_slots_1based.contains(&ext_1based) {
                            continue;
                        }
                        self.mixed_swallowed_layers.entry(ext_1based - 1).or_default().entry(object.key).or_default().insert(layer_idx);
                        if let Some(vol) = vol_id {
                            if per_part_slots_1based.contains(&ext_1based) {
                                self.gradient_volume_swallowed_layers
                                    .entry(ext_1based - 1)
                                    .or_default()
                                    .entry(VolumeKey { obj: object.key, volume_id: vol })
                                    .or_default()
                                    .insert(layer_idx);
                            }
                        }
                    }
                }
            }

            // Per-part gradient: walk LayerRegions and record which (slot, ModelVolume) pairs
            // contributed to this layer.
            if !per_part_slots_1based.is_empty() {
                let mut vol_seen: BTreeSet<(u32, usize)> = BTreeSet::new();
                for layerm in &layer.regions {
                    if layerm.slices_empty {
                        continue;
                    }
                    let Some(vol_id) = layerm.gradient_volume_id else {
                        continue;
                    };
                    let rcfg = layerm.region.config();
                    let role_slots: [u32; 3] =
                        [rcfg.wall_filament as u32, rcfg.sparse_infill_filament as u32, rcfg.solid_infill_filament as u32];
                    for &ext_1based in &role_slots {
                        if ext_1based >= 1 && per_part_slots_1based.contains(&ext_1based) && vol_seen.insert((ext_1based, vol_id)) {
                            self.gradient_volume_layers
                                .entry(ext_1based - 1)
                                .or_default()
                                .entry(VolumeKey { obj: object.key, volume_id: vol_id })
                                .or_default()
                                .push(layer_idx);
                        }
                    }
                }
            }

            layer_count += 1;
        }

        first_layer_extruders.sort_unstable();
        first_layer_extruders.dedup();
        sort_remove_duplicates_u32(&mut bed_contact_extruders);
        if !bed_contact_extruders.is_empty() && object.has_brim {
            let adhesion_extruders = [bed_contact_extruders[0]];
            let adhesion_layer = &mut self.layer_tools[0];
            adhesion_layer.extruders.extend_from_slice(&adhesion_extruders);
            sort_remove_duplicates_u32(&mut adhesion_layer.extruders);
        }
        let outputs = ToolOrderingObjectOutputs {
            object_first_layer_wall_extruders: first_layer_extruders,
            object_brim_filament: brim_filament,
            object_brim_print_z: brim_print_z,
        };

        // Collect the support extruders.
        let flush_volumes_matrix = cfg_floats(print_config, "flush_volumes_matrix");
        let nozzle_count = cfg_floats(print_config, "nozzle_diameter").len();
        for support_layer in &object.support_layers {
            let layer_idx = self.tools_for_layer_index(support_layer.print_z);
            let role = support_layer.support_fills.role();
            let mut has_support = false;
            let mut has_interface = false;
            let mut support_touches_bed = false;
            let mut interface_touches_bed = false;
            let may_touch_bed = support_layer.print_z <= max_contact_z;
            for ee in &support_layer.support_fills.entities {
                let er = entity_role(ee);
                if er == ExtrusionRole::SupportMaterial || er == ExtrusionRole::SupportTransition {
                    has_support = true;
                    support_touches_bed |= may_touch_bed && extrusion_touches_bed(ee, support_layer.print_z);
                }
                if er == ExtrusionRole::SupportMaterialInterface {
                    has_interface = true;
                    interface_touches_bed |= may_touch_bed && extrusion_touches_bed(ee, support_layer.print_z);
                }
                if has_support && has_interface && !may_touch_bed {
                    break;
                }
            }
            let mut extruder_support = object.support_filament;
            let mut extruder_interface = object.support_interface_filament;
            // support_interface_not_for_body keeps the base off a filament the user dedicated to the interface.
            let interface_not_for_body = object.support_interface_not_for_body && object.support_interface_filament > 0;
            // BBS multi-diameter: this support layer is thicker than the object layers, so only the filament it was
            // planned for can extrude it.
            if extruder_support == 0 && support_layer.planned_filament_id > 0 {
                extruder_support = support_layer.planned_filament_id as u32;
            }
            if extruder_interface == 0 && support_layer.planned_interface_filament_id > 0 {
                extruder_interface = support_layer.planned_interface_filament_id as u32;
            }
            if has_support {
                let has_reusable_layer_extruder = |lt: &LayerTools| -> bool {
                    for &extruder_id in &lt.extruders {
                        if extruder_id == 0 {
                            continue;
                        }
                        if interface_not_for_body && extruder_id == extruder_interface {
                            continue;
                        }
                        if get_at(&filament_soluble, (extruder_id - 1) as usize) != 0 {
                            continue;
                        }
                        return true;
                    }
                    false
                };
                if extruder_support > 0 || extruder_interface == 0 || has_reusable_layer_extruder(&self.layer_tools[layer_idx]) {
                    self.layer_tools[layer_idx].extruders.push(extruder_support);
                } else {
                    let all_extruders = print.extruders();
                    let get_next_extruder = |current_extruder: i32, extruders: &[u32]| -> i32 {
                        let flush_matrix: Vec<f32> = get_flush_volumes_matrix_all(&flush_volumes_matrix, 0, nozzle_count)
                            .into_iter()
                            .map(|v| v as f32)
                            .collect();
                        let number_of_extruders = ((flush_matrix.len() as f64).sqrt() + EPSILON) as u32;
                        // Extract purging volumes for each extruder pair:
                        let mut wipe_volumes: Vec<Vec<f32>> = Vec::new();
                        for i in 0..number_of_extruders as usize {
                            wipe_volumes.push(
                                flush_matrix[i * number_of_extruders as usize..(i + 1) * number_of_extruders as usize].to_vec(),
                            );
                        }
                        let mut next_extruder = current_extruder;
                        let mut min_flush = f32::MAX;
                        for &extruder_id in extruders {
                            if get_at(&filament_soluble, extruder_id as usize) != 0 || extruder_id as i32 == current_extruder {
                                continue;
                            }
                            let v = wipe_volumes[(extruder_interface - 1) as usize][extruder_id as usize];
                            if v < min_flush {
                                next_extruder = extruder_id as i32;
                                min_flush = v;
                            }
                        }
                        next_extruder
                    };
                    let next = get_next_extruder(
                        if interface_not_for_body { extruder_interface as i32 - 1 } else { -1 },
                        &all_extruders,
                    );
                    self.layer_tools[layer_idx].extruders.push((next + 1) as u32);
                }
                let back = *self.layer_tools[layer_idx].extruders.last().unwrap();
                if support_touches_bed && back > 0 {
                    self.bed_contact_filaments.push((support_layer.print_z, back));
                }
            }
            if has_interface {
                self.layer_tools[layer_idx].extruders.push(extruder_interface);
                if interface_touches_bed && extruder_interface > 0 {
                    self.bed_contact_filaments.push((support_layer.print_z, extruder_interface));
                }
            }
            if has_support || has_interface {
                let lt = &mut self.layer_tools[layer_idx];
                lt.has_support = true;
                lt.wiping_extrusions_mut().is_support_overriddable_and_mark(
                    role,
                    object.flush_into_support,
                    object.support_filament,
                    object.support_interface_filament,
                );
            }
        }

        for layer in self.layer_tools.iter_mut() {
            // Sort and remove duplicates
            sort_remove_duplicates_u32(&mut layer.extruders);

            // make sure that there are some tools for each object layer (e.g. tall wiping object will result in empty extruders vector)
            if layer.extruders.is_empty() && layer.has_object {
                layer.extruders.push(0); // 0="dontcare" extruder - it will be taken care of in reorder_extruders
            }
        }

        outputs
    }

    /// ToolOrdering.cpp:499-553 `ToolOrdering::ToolOrdering(const PrintObject &object, unsigned int
    /// first_extruder, bool prime_multi_material)`: for the use case when each object is printed
    /// separately. Returns the ordering and what collect_extruders writes into the object.
    pub fn from_object(
        object: &ToolOrderingObject,
        print: &dyn ToolOrderingPrintObjects,
        first_extruder: u32,
        prime_multi_material: bool,
    ) -> (ToolOrdering, ToolOrderingObjectOutputs) {
        let mut to = ToolOrdering::cpp_default();
        to.print_object_ptr = true;
        if object.layers.is_empty() {
            return (to, ToolOrderingObjectOutputs::default());
        }

        // Initialize the print layers for just a single object.
        {
            // construct layer tools by z height
            let mut zs: Vec<f64> = Vec::with_capacity(object.layers.len() + object.support_layers.len());
            for layer in &object.layers {
                zs.push(layer.print_z);
            }
            for layer in &object.support_layers {
                zs.push(layer.print_z);
            }
            to.initialize_layers(zs);
        }

        // Collect extruders reuqired to print the layers. Add dontcare extruders
        let outputs = to.collect_extruders(object, print, &[]);

        // BBS
        // Reorder the extruders to minimize tool switches.
        let first_layer_print_sequence_op = first_layer_print_sequence(print.full_print_config());
        // BBS multi-diameter: never force a filament into the first grid layer just for the skirt.
        let multi_diameter = print.uses_multiple_nozzle_diameters();
        let can_add_skirt_filament_to_first_layer =
            |to: &ToolOrdering, filament: u32| !multi_diameter || to.layer_tools[0].has_extruder(filament);
        if first_extruder == u32::MAX || first_layer_print_sequence_op.as_ref().map_or(false, |s| s.len() > 1) {
            let first_layer_tool_order =
                GenerateFirstLayerToolOrder::<(&dyn ToolOrderingPrintObjects, &ToolOrderingObject)>::generate_first_layer_tool_order(
                    &mut to,
                    (print, object),
                );
            if print.has_skirt()
                && !first_layer_tool_order.is_empty()
                && can_add_skirt_filament_to_first_layer(&to, first_layer_tool_order[0])
            {
                to.layer_tools[0].extruders.push(first_layer_tool_order[0]);
                sort_remove_duplicates_u32(&mut to.layer_tools[0].extruders);
            }
            HandleDontcareExtruder::handle_dontcare_extruder(&mut to, &first_layer_tool_order[..]);
        } else {
            if print.has_skirt() && can_add_skirt_filament_to_first_layer(&to, first_extruder) {
                to.layer_tools[0].extruders.push(first_extruder);
                sort_remove_duplicates_u32(&mut to.layer_tools[0].extruders);
            }
            HandleDontcareExtruder::handle_dontcare_extruder(&mut to, first_extruder);
        }

        to.collect_extruder_statistics(prime_multi_material);

        let max_layer_height = calc_max_layer_height(print.config(), object_max_layer_height(&object.max_layer_height_inputs));

        ToolOrderingCpp::mark_skirt_layers(&mut to, print.config(), max_layer_height, Some(print.as_tool_ordering_print()));
        (to, outputs)
    }

    /// ToolOrdering.cpp:555-639 `ToolOrdering::ToolOrdering(const Print &print, unsigned int
    /// first_extruder, bool prime_multi_material)`: for the use case when all objects are printed at
    /// once. Returns the ordering and, per object, what collect_extruders writes into it.
    pub fn from_print(
        print: &dyn ToolOrderingPrintObjects,
        print_config_ptr: ToolOrderingConfigPtr,
        first_extruder: u32,
        prime_multi_material: bool,
    ) -> (ToolOrdering, Vec<ToolOrderingObjectOutputs>) {
        let mut to = ToolOrdering::cpp_default();
        to.print_config_ptr = Some(print_config_ptr);

        // Initialize the print layers for all objects and all layers.
        let mut max_layer_height: f64 = 0.0;
        {
            let mut zs: Vec<f64> = Vec::new();
            for object in print.objects() {
                zs.reserve(object.layers.len() + object.support_layers.len());
                for layer in &object.layers {
                    zs.push(layer.print_z);
                }
                for layer in &object.support_layers {
                    zs.push(layer.print_z);
                }

                max_layer_height = max_layer_height.max(object_max_layer_height(&object.max_layer_height_inputs));
            }
            to.initialize_layers(zs);
        }
        max_layer_height = calc_max_layer_height(print.config(), max_layer_height);

        // Use the extruder switches from Model::custom_gcode_per_print_z to override the extruder to print the object.
        // Do it only if all the objects were configured to be printed with a single extruder.
        let mut per_layer_extruder_switches: Vec<(f64, u32)> = Vec::new();

        // BBS
        let num_filaments = cfg_floats(print.config(), "filament_diameter").len() as u32;
        if num_filaments > 1
            && print.object_extruders().len() == 1 // the current Print's configuration is CustomGCode::MultiAsSingle
            //BBS: replace model custom gcode with current plate custom gcode
            && print.curr_plate_custom_gcodes().mode == crate::custom_g_code::Mode::MultiAsSingle
        {
            // Printing a single extruder platter on a printer with more than 1 extruder (or single-extruder multi-material).
            // There may be custom per-layer tool changes available at the model.
            per_layer_extruder_switches =
                crate::custom_g_code::custom_tool_changes(print.curr_plate_custom_gcodes(), num_filaments as usize);
        }

        // Collect extruders reuqired to print the layers.
        let mut outputs = Vec::new();
        for object in print.objects() {
            outputs.push(to.collect_extruders(object, print, &per_layer_extruder_switches));
        }

        // Reorder the extruders to minimize tool switches.
        let mut first_layer_tool_order: Vec<u32> = Vec::new();
        if first_extruder == u32::MAX {
            first_layer_tool_order =
                GenerateFirstLayerToolOrder::<&dyn ToolOrderingPrintObjects>::generate_first_layer_tool_order(&mut to, print);
        }

        if print.has_skirt() {
            let mut skirt_extruders: Vec<u32> = Vec::new();
            if !first_layer_tool_order.is_empty() {
                // Print::min_skirt_length (Print.cpp:58) is 0.
                const MIN_SKIRT_LENGTH: f32 = 0.0;
                if MIN_SKIRT_LENGTH as f64 > EPSILON {
                    skirt_extruders = first_layer_tool_order.clone();
                } else {
                    skirt_extruders.push(first_layer_tool_order[0]);
                }
            } else if first_extruder != u32::MAX {
                skirt_extruders.push(first_extruder);
            }
            // BBS multi-diameter: keep the skirt on a filament the first layer already prints.
            if print.uses_multiple_nozzle_diameters() && !skirt_extruders.is_empty() {
                let first_layer = &to.layer_tools[0];
                let candidates = if first_layer_tool_order.is_empty() { &skirt_extruders } else { &first_layer_tool_order };
                let it = candidates.iter().find(|&&filament| first_layer.has_extruder(filament));
                let mut kept: Vec<u32> = Vec::new();
                if let Some(&f) = it {
                    kept.push(f);
                }
                skirt_extruders = kept;
            }
            to.layer_tools[0].extruders.extend_from_slice(&skirt_extruders);
            sort_remove_duplicates_u32(&mut to.layer_tools[0].extruders);
        }

        if !first_layer_tool_order.is_empty() {
            HandleDontcareExtruder::handle_dontcare_extruder(&mut to, &first_layer_tool_order[..]);
        } else {
            HandleDontcareExtruder::handle_dontcare_extruder(&mut to, first_extruder);
        }

        to.collect_extruder_statistics(prime_multi_material);

        ToolOrderingCpp::mark_skirt_layers(&mut to, print.config(), max_layer_height, Some(print.as_tool_ordering_print()));
        (to, outputs)
    }
}

/// `dynamic_cast<const ExtrusionEntityCollection&>(*eec)` of a perimeter / fill entity.
fn expect_collection(e: &crate::extrusion_entity::ExtrusionEntityType) -> &crate::extrusion_entity::ExtrusionEntityCollection {
    match e {
        crate::extrusion_entity::ExtrusionEntityType::Collection(c) => c,
        _ => panic!("std::bad_cast: a layer region entity is not an ExtrusionEntityCollection"),
    }
}

/// PrintConfig.hpp:2176-2184 `get_flush_volumes_matrix`
fn get_flush_volumes_matrix_all(fv_matrix: &[f64], extruder_id: usize, nozzle_nums: usize) -> Vec<f64> {
    if extruder_id != usize::MAX && nozzle_nums != 1 {
        let from = ((fv_matrix.len() / nozzle_nums * extruder_id) as f64 + EPSILON) as usize;
        let to = ((fv_matrix.len() / nozzle_nums * (extruder_id + 1)) as f64 + EPSILON) as usize;
        return fv_matrix[from..to].to_vec();
    }
    fv_matrix.to_vec()
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
            flush_filament_change_count: 0,
        };

        let stats2 = FilamentChangeStats {
            filament_flush_weight: 5,
            filament_change_count: 3,
            extruder_change_count: 1,
            flush_filament_change_count: 0,
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

/// The `Print` the live export hands the filament grouping of a printer with more than one
/// extruder (ToolOrdering.cpp:3510-3548 -> get_recommended_filament_maps :2511-2690): the full
/// configuration and the state a print sliced once by the command line has (no filament print
/// times, no AMS / loaded-filament information, no calibration, not sequential, no dynamic
/// regrouping). Every filament is `ModelOnly` (Print.cpp:3474-3503 without support filaments).
///
/// FIDELITY-NOTE: `get_geometric_unprintable_filaments` returns the empty list (Print::process fills
/// m_geometric_unprintable_filaments from the objects against extruder_printable_area; not carried
/// by the live print).
pub struct LiveGroupingPrint {
    config: ToolOrderingPrintConfig,
    ext_info: Vec<Vec<ToolOrderingPrintConfig>>,
    loaded: Vec<crate::filament_group_utils::NozzleLoadedFilamentInfo>,
    custom_gcodes: crate::custom_g_code::Info,
    group: std::cell::RefCell<Option<std::sync::Arc<crate::multi_nozzle_utils::LayeredNozzleGroupResult>>>,
}

impl LiveGroupingPrint {
    pub fn new(config: ToolOrderingPrintConfig) -> Self {
        LiveGroupingPrint {
            config,
            ext_info: Vec::new(),
            loaded: Vec::new(),
            custom_gcodes: crate::custom_g_code::Info::default(),
            group: std::cell::RefCell::new(None),
        }
    }
}

impl ToolOrderingPrint for LiveGroupingPrint {
    fn config(&self) -> &ToolOrderingPrintConfig {
        &self.config
    }
    fn full_print_config(&self) -> &ToolOrderingPrintConfig {
        &self.config
    }
    fn uses_multiple_nozzle_diameters(&self) -> bool {
        false
    }
    fn object_extruders(&self) -> Vec<u32> {
        Vec::new()
    }
    fn curr_plate_custom_gcodes(&self) -> &crate::custom_g_code::Info {
        &self.custom_gcodes
    }
    fn get_layered_nozzle_group_result(&self) -> Option<std::sync::Arc<crate::multi_nozzle_utils::LayeredNozzleGroupResult>> {
        self.group.borrow().clone()
    }
    fn get_geometric_unprintable_filaments(&self) -> Vec<BTreeSet<i32>> {
        Vec::new()
    }
    fn get_physical_unprintable_filaments(&self, used_filaments: &[u32]) -> Vec<BTreeSet<i32>> {
        // Print.cpp:3504-3535
        let extruder_num = cfg_floats(&self.config, "nozzle_diameter").len();
        let mut physical_unprintables: Vec<BTreeSet<i32>> = vec![BTreeSet::new(); extruder_num];
        if extruder_num < 2 {
            return physical_unprintables;
        }
        let printable = cfg_ints(&self.config, "filament_printable");
        for &f in used_filaments {
            let Some(&status) = printable.get(f as usize) else { continue };
            if let Some(i) = (0..extruder_num as i32).find(|&i| (status >> i) & 1 == 0) {
                physical_unprintables[i as usize].insert(f as i32);
            }
        }
        physical_unprintables
    }
    fn get_filament_unprintable_flow(&self, used_filaments: &[u32]) -> BTreeMap<i32, BTreeSet<crate::extruder::NozzleVolumeType>> {
        // Print.cpp:3537-3566
        use crate::extruder::NozzleVolumeType;
        use crate::print_config::convert_to_nvt_type;
        let c = &self.config;
        let extruder_variant_list = cfg_strings(c, "printer_extruder_variant");
        let filament_variant_list = cfg_strings(c, "filament_extruder_variant");
        let filament_self_index: Vec<i32> =
            if !crate::config::ConfigOptionResolver::has(c, "filament_self_index") { vec![1; filament_variant_list.len()] } else { cfg_ints(c, "filament_self_index") };
        let mut ret: BTreeMap<i32, BTreeSet<NozzleVolumeType>> = BTreeMap::new();
        let mut filament_variant_map: BTreeMap<i32, BTreeSet<NozzleVolumeType>> = BTreeMap::new();
        for i in 0..filament_variant_list.len() {
            let volume = convert_to_nvt_type(&filament_variant_list[i]);
            if volume != NozzleVolumeType::NvtHybrid {
                filament_variant_map.entry(filament_self_index.get(i).copied().unwrap_or(1)).or_default().insert(volume);
            }
        }
        for (first, volumes) in &filament_variant_map {
            let fil_idx = first - 1;
            if !used_filaments.iter().any(|&f| f as i32 == fil_idx) {
                continue;
            }
            for v in &extruder_variant_list {
                let exd_volume = convert_to_nvt_type(v);
                if !volumes.contains(&exd_volume) && exd_volume != NozzleVolumeType::NvtHybrid {
                    ret.entry(fil_idx).or_default().insert(exd_volume);
                }
            }
        }
        ret
    }
    fn calib_params(&self) -> ToolOrderingCalibParams {
        ToolOrderingCalibParams::default()
    }
    fn get_extruder_filament_info(&self) -> &[Vec<ToolOrderingPrintConfig>] {
        &self.ext_info
    }
    fn get_filament_usage_type(&self) -> Vec<crate::filament_group_utils::FilamentUsageType> {
        // Print.cpp:3474-3503: no object has a support filament.
        vec![crate::filament_group_utils::FilamentUsageType::ModelOnly; cfg_strings(&self.config, "filament_type").len()]
    }
    fn get_filament_print_time(
        &self,
    ) -> crate::multi_nozzle_utils::UnorderedMapInt<crate::multi_nozzle_utils::UnorderedMapInt<f64>> {
        Default::default()
    }
    fn get_nozzle_loaded_filament_info(&self) -> &[crate::filament_group_utils::NozzleLoadedFilamentInfo] {
        &self.loaded
    }
    fn is_dynamic_group_reorder(&self) -> bool {
        false
    }
    fn is_sequential_print(&self) -> bool {
        false
    }
    fn resolve_manual_nozzle_map(&self, _used_filaments: &[u32]) -> Result<(), crate::exception::RuntimeError> {
        // Print.cpp:3322-3345 returns at once unless the mode is fmmNozzleManual.
        Ok(())
    }
    fn set_nozzle_group_result(&self, result: std::sync::Arc<crate::multi_nozzle_utils::LayeredNozzleGroupResult>) {
        *self.group.borrow_mut() = Some(result);
    }
    fn update_filament_maps_to_config(&self, _f_maps: Vec<i32>, _f_volume_maps: Vec<i32>, _f_nozzle_maps: Vec<i32>) {}
    fn update_to_config_by_nozzle_group_result(&self, _result: &crate::multi_nozzle_utils::LayeredNozzleGroupResult) {}
}

/// ToolOrdering.cpp:3510-3635 `reorder_extruders_for_minimum_flush_volume` for a printer with more
/// than one extruder and one nozzle per extruder, without dynamic regrouping, printed layer by layer:
/// Stage 1 prepare_flush_matrices (:2176-2200), Stage 4 get_recommended_filament_maps (:3536-3543),
/// then execute_filament_ordering (:2844-2873, support_multi_nozzle false) =
/// reorder_filaments_for_minimum_flush_volume(filament_lists, grouping_result.get_extruder_map(true),
/// layer_filaments, nozzle_flush_mtx, get_custom_seq, &filament_sequences) with the default (empty)
/// nozzle_status. `layer_filaments` are the LayerTools' extruders as the ordering finds them (layer
/// 0 in the first-layer order); `reorder_first_layer` as prepare_ordering_context (:2815-2842).
/// Returns (sequences, zero-based extruder map) or None when the grouping fails.
pub fn reorder_extruders_for_minimum_flush_volume_multi_extruder(
    config: &ToolOrderingPrintConfig,
    layer_filaments: &[Vec<u32>],
    reorder_first_layer: bool,
) -> Option<(Vec<Vec<u32>>, Vec<i32>)> {
    use crate::print_config::enums::FilamentMapMode;
    let print = LiveGroupingPrint::new(config.clone());
    let nozzle_flush_mtx = group_reorder::prepare_flush_matrices(config);
    // :2789-2812 collect_layer_and_unprintable_data
    let used_filaments = crate::filament_group::collect_sorted_used_filaments(layer_filaments);
    let geometric_unprintables = print.get_geometric_unprintable_filaments();
    let physical_unprintables = print.get_physical_unprintable_filaments(&used_filaments);
    let filament_unprintable_volumes = print.get_filament_unprintable_flow(&used_filaments);
    // :2815-2842 prepare_ordering_context
    let filament_nums = cfg_strings(config, "filament_colour").len();
    let filament_lists: Vec<u32> = (0..filament_nums as u32).collect();
    let first_layer_filaments: Vec<u32> = layer_filaments.first().cloned().unwrap_or_default();
    let get_custom_seq = group_reorder::create_custom_seq_function(config, !reorder_first_layer, &first_layer_filaments);
    // :3536-3543
    let grouping_result = ToolOrdering::get_recommended_filament_maps(
        &print,
        layer_filaments,
        FilamentMapMode::from_i32(cfg_int(config, "filament_map_mode")),
        &physical_unprintables,
        &geometric_unprintables,
        &filament_unprintable_volumes,
        &Default::default(),
    )
    .ok()?;
    let filament_maps = grouping_result.get_extruder_map(true, -1);
    // :2861-2869
    let mut filament_sequences: Vec<Vec<u32>> = Vec::new();
    crate::gcode::tool_order_utils::reorder_filaments_for_minimum_flush_volume(
        &filament_lists,
        &filament_maps,
        layer_filaments,
        &nozzle_flush_mtx,
        Some(&*get_custom_seq),
        Some(&mut filament_sequences),
        &HashMap::new(),
    );
    Some((filament_sequences, filament_maps))
}
