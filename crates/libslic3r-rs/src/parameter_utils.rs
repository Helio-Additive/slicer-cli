//! Parameter utilities.
//!
//! 1:1 port of `ParameterUtils.cpp` / `ParameterUtils.hpp` from BambuStudio.
//!
//! C++ Reference: ParameterUtils.hpp, ParameterUtils.cpp

// ParameterUtils.hpp:9
// using LayerPrintSequence = std::pair<std::pair<int, int>, std::vector<int>>;
/// `LayerPrintSequence = ((int, int), Vec<int>)`
pub type LayerPrintSequence = ((i32, i32), Vec<i32>);

// ParameterUtils.cpp:7
// std::vector<LayerPrintSequence> get_other_layers_print_sequence(int sequence_nums, const std::vector<int> &sequence)
/// Decode a flat `sequence` buffer of `sequence_nums` equally-sized items into
/// `LayerPrintSequence` records. Each item's first two ints form the (first,
/// second) pair and the remainder is the int vector.
pub fn get_other_layers_print_sequence(
    sequence_nums: i32,
    sequence: &[i32],
) -> Vec<LayerPrintSequence> {
    // ParameterUtils.cpp:9
    let mut res: Vec<LayerPrintSequence> = Vec::new();
    // ParameterUtils.cpp:10-11
    if sequence_nums == 0 || sequence.is_empty() {
        return res;
    }

    // ParameterUtils.cpp:13
    debug_assert!(sequence.len() % sequence_nums as usize == 0);

    // ParameterUtils.cpp:15
    res.reserve(sequence_nums as usize);
    // ParameterUtils.cpp:16
    let item_nums = sequence.len() / sequence_nums as usize;

    // ParameterUtils.cpp:18
    for i in 0..sequence_nums {
        // ParameterUtils.cpp:19-20
        let item: Vec<i32> = sequence
            [(i as usize * item_nums)..((i as usize + 1) * item_nums)]
            .to_vec();

        // ParameterUtils.cpp:22
        debug_assert!(item.len() > 2);
        // ParameterUtils.cpp:23-26
        let mut res_item: LayerPrintSequence = ((0, 0), Vec::new());
        res_item.0 .0 = item[0];
        res_item.0 .1 = item[1];
        res_item.1 = item[2..].to_vec();
        // ParameterUtils.cpp:27
        res.push(res_item);
    }

    // ParameterUtils.cpp:30
    res
}

// ParameterUtils.cpp:33
// void get_other_layers_print_sequence(const std::vector<LayerPrintSequence> &customize_sequences, int &sequence_nums, std::vector<int> &sequence)
/// Encode `customize_sequences` back into the flat (`sequence_nums`,
/// `sequence`) representation. Overload of the function above; renamed to
/// `set_other_layers_print_sequence` because Rust lacks overloading.
pub fn set_other_layers_print_sequence(
    customize_sequences: &[LayerPrintSequence],
    sequence_nums: &mut i32,
    sequence: &mut Vec<i32>,
) {
    // ParameterUtils.cpp:35
    *sequence_nums = 0;
    // ParameterUtils.cpp:36
    sequence.clear();
    // ParameterUtils.cpp:37
    if customize_sequences.is_empty() {
        return;
    }

    // ParameterUtils.cpp:39
    *sequence_nums = customize_sequences.len() as i32;
    // ParameterUtils.cpp:40
    for customize_sequence in customize_sequences {
        // ParameterUtils.cpp:41
        sequence.push(customize_sequence.0 .0);
        // ParameterUtils.cpp:42
        sequence.push(customize_sequence.0 .1);
        // ParameterUtils.cpp:43
        sequence.extend_from_slice(&customize_sequence.1);
    }
}

// ParameterUtils.cpp:47
// int get_index_for_extruder_parameter(const DynamicPrintConfig &config, const std::string &opt_key, int cur_extruder_id, ExtruderType extruder_type, NozzleVolumeType nozzle_volume_type)
//
// BLOCKED: not ported. This function dispatches on the global variant option
// sets (`printer_options_with_variant_1`, `printer_options_with_variant_2`,
// `filament_options_with_variant`, `print_options_with_variant`) and then calls
// `DynamicPrintConfig::get_index_for_extruder(...)` (PrintConfig.cpp:7586).
// That method body is the real blocker: it does
//     auto variant_opt = dynamic_cast<const ConfigOptionStrings*>(this->option(variant_name));
//     const ConfigOptionInts* id_opt   = dynamic_cast<const ConfigOptionInts*>(this->option(id_name));
// i.e. a runtime STRING-KEYED lookup `DynamicPrintConfig::option(name)` returning
// a polymorphic `ConfigOption*` that is then down-cast to `ConfigOptionStrings`
// / `ConfigOptionInts`. The variant_name / id_name strings ("printer_extruder_id",
// "printer_extruder_variant", "filament_extruder_variant", "print_extruder_id",
// "print_extruder_variant") are selected at runtime from `opt_key`, so the access
// is inherently dynamic — there is no fixed field to read.
//
// This is the Config.hpp `ConfigOption` virtual hierarchy + the
// `ConfigBase`/`DynamicConfig`/`DynamicPrintConfig` runtime-typed dictionary,
// which `config.rs` (see its SCOPE NOTE) and `print_config.rs:5500-5513`
// deliberately do NOT mirror: the crate uses plain typed structs
// (`PrintConfig`) with no generic `option(name)` accessor, no `ConfigOption*`
// variants, and no `printer_extruder_id` / `*_extruder_variant`
// `ConfigOptionInts`/`ConfigOptionStrings` fields at all. Both Config.cpp and
// PrintConfig.cpp are status="partial" precisely for this reason.
//
// NOTE on the 2026-06-12 config-hierarchy hint: the wired hierarchy navigation
// (`layer.object().print().config()`) hands back the typed `PrintConfig`
// struct, NOT a `DynamicPrintConfig` with a string-keyed `option(name)`
// dictionary, so it does NOT unblock this symbol. (The leaf helpers this would
// otherwise need — `get_extruder_variant_string` / `get_config_index_base` /
// the `ExtruderType`/`NozzleVolumeType` enums — ARE already ported in
// `crate::extruder`; the sole remaining gap is the dynamic dictionary.)
//
// The variant option sets themselves are PrintConfig.cpp globals (PrintConfig.cpp:6986,
// 7030, 7088, 7116) and belong to that file's port, not here — adding them in
// this module would duplicate (rule 3). Implementing a hardcoded match here
// would be a fake and is forbidden, so the symbol is left unported until
// DynamicPrintConfig's dynamic `ConfigOption` dictionary is available.

#[cfg(test)]
mod tests {
    use super::*;

    // Round-trip: encode then decode reproduces the original records.
    #[test]
    fn test_get_other_layers_print_sequence_decode() {
        // Two items, each of size 4: pair (first, second) + 2 trailing ints.
        let sequence = vec![0, 5, 1, 2, /*item 2*/ 6, 9, 3, 4];
        let res = get_other_layers_print_sequence(2, &sequence);

        assert_eq!(res.len(), 2);
        assert_eq!(res[0], ((0, 5), vec![1, 2]));
        assert_eq!(res[1], ((6, 9), vec![3, 4]));
    }

    #[test]
    fn test_get_other_layers_print_sequence_empty() {
        let res = get_other_layers_print_sequence(0, &[]);
        assert!(res.is_empty());

        let res = get_other_layers_print_sequence(5, &[]);
        assert!(res.is_empty());
    }

    #[test]
    fn test_set_other_layers_print_sequence_encode() {
        let customize = vec![((0, 5), vec![1, 2]), ((6, 9), vec![3, 4])];
        let mut sequence_nums = 0;
        let mut sequence = Vec::new();
        set_other_layers_print_sequence(&customize, &mut sequence_nums, &mut sequence);

        assert_eq!(sequence_nums, 2);
        assert_eq!(sequence, vec![0, 5, 1, 2, 6, 9, 3, 4]);
    }

    #[test]
    fn test_set_other_layers_print_sequence_empty() {
        let mut sequence_nums = 7;
        let mut sequence = vec![1, 2, 3];
        set_other_layers_print_sequence(&[], &mut sequence_nums, &mut sequence);

        assert_eq!(sequence_nums, 0);
        assert!(sequence.is_empty());
    }

    #[test]
    fn test_round_trip() {
        let customize = vec![((1, 100), vec![2, 0, 1]), ((101, 200), vec![0, 1, 2])];
        let mut sequence_nums = 0;
        let mut sequence = Vec::new();
        set_other_layers_print_sequence(&customize, &mut sequence_nums, &mut sequence);

        let decoded = get_other_layers_print_sequence(sequence_nums, &sequence);
        assert_eq!(decoded, customize);
    }
}
