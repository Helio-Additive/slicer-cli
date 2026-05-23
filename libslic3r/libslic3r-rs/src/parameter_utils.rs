//! Parameter utilities for print configuration and extruder management
//!
//! Provides utilities for managing layer print sequences and extruder
//! parameter indexing in multi-extruder configurations.
//!
//! C++ Reference: ParameterUtils.hpp, ParameterUtils.cpp

/// Layer print sequence: ((start_layer, end_layer), [extruder_ids])
/// ParameterUtils.hpp:9
pub type LayerPrintSequence = ((i32, i32), Vec<i32>);

/// Extruder type enumeration
/// ParameterUtils.hpp:13 (from PrintConfig.hpp)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ExtruderType {
    /// Standard FDM extruder
    Standard,
    /// Multi-material unit (MMU)
    MultiMaterial,
    /// Direct drive extruder
    DirectDrive,
    /// Bowden extruder
    Bowden,
}

/// Nozzle volume type for purge calculations
/// ParameterUtils.hpp:13 (from PrintConfig.hpp)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum NozzleVolumeType {
    /// Standard volume nozzle
    Standard,
    /// High flow nozzle
    HighFlow,
    /// Low volume nozzle
    LowVolume,
}

/// Get print sequences for other layers based on a reference sequence
/// ParameterUtils.hpp:10
pub fn get_other_layers_print_sequence(
    sequence_nums: i32,
    sequence: &[i32],
) -> Vec<LayerPrintSequence> {
    let mut result = Vec::new();

    if sequence.is_empty() || sequence_nums <= 0 {
        return result;
    }

    // Generate sequences by repeating the pattern
    for i in 0..sequence_nums {
        let start_layer = i * sequence.len() as i32;
        let end_layer = start_layer + sequence.len() as i32 - 1;
        result.push(((start_layer, end_layer), sequence.to_vec()));
    }

    result
}

/// Extract print sequences back into sequence_nums and sequence vector
/// ParameterUtils.hpp:11
pub fn extract_print_sequence(
    customize_sequences: &[LayerPrintSequence],
    sequence_nums: &mut i32,
    sequence: &mut Vec<i32>,
) {
    sequence.clear();
    *sequence_nums = 0;

    if customize_sequences.is_empty() {
        return;
    }

    // Use the first sequence as the pattern
    if let Some(first) = customize_sequences.first() {
        *sequence = first.1.clone();
        *sequence_nums = customize_sequences.len() as i32;
    }
}

/// Get the configuration index for a given extruder parameter
/// ParameterUtils.hpp:13
pub fn get_index_for_extruder_parameter(
    opt_key: &str,
    cur_extruder_id: i32,
    _extruder_type: ExtruderType,
    _nozzle_volume_type: NozzleVolumeType,
) -> i32 {
    // Default implementation: return the current extruder ID
    // More sophisticated logic would look up in DynamicPrintConfig
    // and handle per-extruder vs global parameters

    // Some parameters are global (not per-extruder)
    match opt_key {
        "layer_height" | "first_layer_height" | "support_material" => {
            // Global parameters: use index 0
            0
        }
        _ => {
            // Per-extruder parameters: use current extruder ID
            cur_extruder_id
        }
    }
}

/// Check if a parameter key is per-extruder or global
/// ParameterUtils.hpp (utility)
pub fn is_per_extruder_parameter(opt_key: &str) -> bool {
    matches!(
        opt_key,
        "nozzle_diameter"
            | "filament_diameter"
            | "extrusion_multiplier"
            | "retract_length"
            | "retract_speed"
            | "retract_before_travel"
            | "wipe"
            | "retract_layer_change"
            | "filament_color"
            | "filament_notes"
            | "filament_max_volumetric_speed"
            | "temperature"
            | "first_layer_temperature"
            | "bed_temperature"
            | "first_layer_bed_temperature"
    )
}

/// Get default extruder type
/// ParameterUtils.hpp (utility)
impl Default for ExtruderType {
    fn default() -> Self {
        ExtruderType::Standard
    }
}

/// Get default nozzle volume type
/// ParameterUtils.hpp (utility)
impl Default for NozzleVolumeType {
    fn default() -> Self {
        NozzleVolumeType::Standard
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_layer_print_sequence() {
        let sequence = vec![0, 1, 2];
        let sequences = get_other_layers_print_sequence(3, &sequence);

        assert_eq!(sequences.len(), 3);
        assert_eq!(sequences[0], ((0, 2), vec![0, 1, 2]));
        assert_eq!(sequences[1], ((3, 5), vec![0, 1, 2]));
        assert_eq!(sequences[2], ((6, 8), vec![0, 1, 2]));
    }

    #[test]
    fn test_extract_print_sequence() {
        let sequences = vec![((0, 2), vec![0, 1, 2]), ((3, 5), vec![0, 1, 2])];

        let mut sequence_nums = 0;
        let mut sequence = Vec::new();

        extract_print_sequence(&sequences, &mut sequence_nums, &mut sequence);

        assert_eq!(sequence_nums, 2);
        assert_eq!(sequence, vec![0, 1, 2]);
    }

    #[test]
    fn test_extruder_parameter_index() {
        let idx = get_index_for_extruder_parameter(
            "nozzle_diameter",
            2,
            ExtruderType::Standard,
            NozzleVolumeType::Standard,
        );
        assert_eq!(idx, 2); // Per-extruder parameter

        let idx = get_index_for_extruder_parameter(
            "layer_height",
            2,
            ExtruderType::Standard,
            NozzleVolumeType::Standard,
        );
        assert_eq!(idx, 0); // Global parameter
    }

    #[test]
    fn test_is_per_extruder_parameter() {
        assert!(is_per_extruder_parameter("nozzle_diameter"));
        assert!(is_per_extruder_parameter("temperature"));
        assert!(!is_per_extruder_parameter("layer_height"));
        assert!(!is_per_extruder_parameter("support_material"));
    }

    #[test]
    fn test_empty_sequence() {
        let sequences = get_other_layers_print_sequence(0, &[]);
        assert!(sequences.is_empty());

        let sequences = get_other_layers_print_sequence(5, &[]);
        assert!(sequences.is_empty());
    }

    #[test]
    fn test_extruder_type_default() {
        assert_eq!(ExtruderType::default(), ExtruderType::Standard);
    }

    #[test]
    fn test_nozzle_volume_type_default() {
        assert_eq!(NozzleVolumeType::default(), NozzleVolumeType::Standard);
    }
}
