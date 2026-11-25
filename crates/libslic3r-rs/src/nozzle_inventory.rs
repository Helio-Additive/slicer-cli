//! Port of `NozzleInventory.hpp` / `NozzleInventory.cpp`: the nozzles installed in each extruder
//! slot of the selected printer, resolved from preset defaults, per-printer history, the open
//! project, machine sync and user edits.
//!
//! C++ Reference:
//! - src/libslic3r/NozzleInventory.hpp
//! - src/libslic3r/NozzleInventory.cpp
//!
//! Project, Machine and User are separate layers. Each write replaces one layer and stamps it. The
//! effective nozzles are computed on read: the newest layer of the selected printer model wins if it
//! holds the selected diameter, else the saved slots of the model, else the saved volume types of the
//! selected variant, else the preset defaults. On a printer with a nozzle rack the selected diameter
//! only filters the slots, so there any installed nozzle holds it. A printer switch writes no layer,
//! so the order of switch and sync does not matter.
//!
//! The result is the nozzles that the next slice assumes. It matches the real machine only after a
//! sync of the same printer model, and only until a newer project or user write shadows that sync.
//!
//! FIDELITY-NOTE (what the crate does not have, and what stands in for it here):
//! - `DynamicPrintConfig`: the crate has no key/value config (`config.rs`, scope note), so
//!   [`NozzlePrintConfig`] holds the fourteen keys this unit reads and writes, each `None` when the
//!   key is missing, with the value type of its `ConfigOption` class.
//! - `PresetCollection`: the crate has no preset collections (`preset_bundle.rs`, port status), so
//!   [`PrinterPresetSelection`] holds what `read_selected_printer` takes from one: the name of the
//!   selected preset, the name of its base preset and the config of the edited preset.
//! - [`NozzleVolumeType`] holds the integer because the unit casts config values
//!   to the enum unchecked (`NozzleVolumeType(types[flat])`).
//! - The inventory's PrintConfig and PresetBundle helpers follow the pin and carry
//!   their source references at the end of this file.
//! - The mirror config and the history are a reference and a pointer that must outlive the
//!   inventory in C++; here they are shared (`Arc<Mutex<_>>`).
//!
//! Nothing in the crate calls this unit yet. In C++ it is a member of `PresetBundle`
//! (`PresetBundle::nozzles`), which the crate has not ported.

use std::collections::BTreeMap;
use std::sync::{Arc, Mutex, MutexGuard};

use crate::app_config::AppConfig;
use crate::boost_log_trivial;
use crate::locales_utils::string_to_double_decimal_point;
use crate::log::kv;
use crate::log_keys::log_key;
use crate::utils::format_diameter_to_str_default as format_diameter_to_str;

/// PrintConfig.hpp:382-390 `enum NozzleVolumeType`, as its integer value (see the module notes).
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct NozzleVolumeType(pub i32);

/// PrintConfig.hpp:383
pub const NVT_STANDARD: NozzleVolumeType = NozzleVolumeType(0);
/// PrintConfig.hpp:384
pub const NVT_HIGH_FLOW: NozzleVolumeType = NozzleVolumeType(1);
/// PrintConfig.hpp:385
pub const NVT_HYBRID: NozzleVolumeType = NozzleVolumeType(2);
/// PrintConfig.hpp:386
pub const NVT_TPU_HIGH_FLOW: NozzleVolumeType = NozzleVolumeType(3);
/// PrintConfig.hpp:387
pub const NVT_TPU285: NozzleVolumeType = NozzleVolumeType(4);
/// PrintConfig.hpp:388
pub const NVT_E3D_HIGH_FLOW: NozzleVolumeType = NozzleVolumeType(5);
/// PrintConfig.hpp:389 — always the last one.
pub const NVT_COUNT: NozzleVolumeType = NozzleVolumeType(6);

/// The keys of `DynamicPrintConfig` this unit reads and writes (see the module notes). A key that
/// is `None` is missing from the config.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct NozzlePrintConfig {
    /// `ConfigOptionEnumsGeneric`; read from a printer preset.
    pub default_nozzle_volume_type: Option<Vec<i32>>,
    /// `ConfigOptionIntsNullable`; a nil value is `i32::MAX`. Read from a printer preset.
    pub extruder_max_nozzle_count: Option<Vec<i32>>,
    /// `ConfigOptionInts`; per extruder, the number of installed nozzles.
    pub extruder_nozzle_count: Option<Vec<i32>>,
    /// `ConfigOptionFloats`; per installed nozzle.
    pub extruder_nozzle_diameter: Option<Vec<f64>>,
    /// `ConfigOptionInts`; per installed nozzle, its slot id counted across all extruders.
    pub extruder_nozzle_slot: Option<Vec<i32>>,
    /// `ConfigOptionStrings`; the legacy counts per extruder.
    pub extruder_nozzle_stats: Option<Vec<String>>,
    /// `ConfigOptionStrings`; the legacy counts per extruder, with the real type names.
    pub extruder_nozzle_stats_new: Option<Vec<String>>,
    /// `ConfigOptionEnumsGeneric`; per installed nozzle.
    pub extruder_nozzle_volume_type: Option<Vec<i32>>,
    /// `ConfigOptionBool`; read from a printer preset.
    pub handle_hotend_as_extruder: Option<bool>,
    /// `ConfigOptionBool`; the project's multi-diameter mode.
    pub multi_nozzle_diameter_mode: Option<bool>,
    /// `ConfigOptionFloatsNullable`; a nil value is NaN. Read from a printer preset.
    pub nozzle_diameter: Option<Vec<f64>>,
    /// `ConfigOptionEnumsGeneric`; per extruder.
    pub nozzle_volume_type: Option<Vec<i32>>,
    /// `ConfigOptionString`; read from a printer preset.
    pub printer_model: Option<String>,
    /// `ConfigOptionString`; read from a printer preset.
    pub printer_variant: Option<String>,
}

impl NozzlePrintConfig {
    /// `ConfigOptionIntsNullable::is_nil(idx)` of `extruder_max_nozzle_count` (Config.hpp): the nil
    /// value is `std::numeric_limits<int>::max()`.
    fn ints_nullable_is_nil(values: &[i32], idx: usize) -> bool {
        values[idx] == i32::MAX
    }

    /// `DynamicConfig::opt_string(key)` (Config.hpp:2805-2806) of a key the unit reads without a
    /// check: C++ dereferences the missing option there, so a missing key is a programming error.
    fn opt_string<'a>(value: &'a Option<String>, key: &str) -> &'a str {
        value.as_deref().unwrap_or_else(|| panic!("config has no {key}"))
    }
}

/// What `read_selected_printer` takes from `PresetCollection` (see the module notes).
#[derive(Debug, Clone, Default, PartialEq)]
pub struct PrinterPresetSelection {
    /// `printers.get_selected_preset().name`
    pub selected_preset_name: String,
    /// `printers.get_preset_base(selected)->name`; `None` when the base preset is not found.
    pub preset_base_name: Option<String>,
    /// `printers.get_edited_preset().config`
    pub edited_preset_config: NozzlePrintConfig,
}

/// NozzleInventory.hpp:36-41 — state of a nozzle slot.
///
/// All states but Empty hold a nozzle, with its health as the machine reports it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NozzleSlotStatus {
    Empty,
    Normal,
    Error,
    Unknown,
}

/// NozzleInventory.hpp:43-58 — one physical nozzle slot of an extruder.
///
/// A default slot is empty. An empty slot has an empty diameter, so a diameter compare never matches it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct NozzleSlot {
    /// nozzle diameter such as "0.4"; empty for an empty slot
    pub diameter: String,
    /// flow type of the nozzle
    pub volume_type: NozzleVolumeType,
    /// Empty, or the health of the nozzle
    pub status: NozzleSlotStatus,
}

impl Default for NozzleSlot {
    /// NozzleInventory.hpp:50-52
    fn default() -> Self {
        NozzleSlot { diameter: String::new(), volume_type: NVT_STANDARD, status: NozzleSlotStatus::Empty }
    }
}

impl NozzleSlot {
    /// NozzleInventory.hpp:54-57 — check if the slot holds no nozzle.
    pub fn empty(&self) -> bool {
        self.status == NozzleSlotStatus::Empty
    }
}

/// NozzleInventory.hpp:60-63 — all nozzle slots of one extruder, in slot order.
pub type ExtruderSlots = Vec<NozzleSlot>;

/// NozzleInventory.cpp:21-24 — the layer that gives the effective nozzles of an extruder.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum NozzleSource {
    Preset,
    History,
    Project,
    Machine,
    User,
}

/// NozzleInventory.cpp:26-37 — one write of Project, Machine or User.
///
/// Stamp 0 means the layer is empty.
#[derive(Debug, Clone)]
struct NozzleLayer {
    source: NozzleSource,
    model: String,
    extruders: Vec<ExtruderSlots>,
    stamp: u64,
}

impl Default for NozzleLayer {
    /// NozzleInventory.cpp:33-36
    fn default() -> Self {
        NozzleLayer { source: NozzleSource::Project, model: String::new(), extruders: Vec::new(), stamp: 0 }
    }
}

/// NozzleInventory.cpp:39-53 — what resolve needs from the selected printer, copied when it is
/// selected.
#[derive(Debug, Clone, Default)]
struct SelectedPrinter {
    model: String,
    history_key: String,
    diameter: String,
    /// the multi-diameter mode applies
    supports_multi_diameter: bool,
    /// an extruder has more than one slot
    has_rack: bool,
    /// per extruder, at least 1
    slot_counts: Vec<i32>,
    /// per extruder, per slot
    default_types: Vec<Vec<NozzleVolumeType>>,
}

impl SelectedPrinter {
    /// NozzleInventory.cpp:52
    fn extruder_count(&self) -> i32 {
        self.slot_counts.len() as i32
    }
}

/// NozzleInventory.cpp:55-59
fn source_name(source: NozzleSource) -> &'static str {
    const NAMES: [&str; 5] = ["Preset", "History", "Project", "Machine", "User"];
    NAMES[source as usize]
}

/// NozzleInventory.cpp:61-64
fn installed_count(slots: &ExtruderSlots) -> i32 {
    slots.iter().filter(|slot| !slot.empty()).count() as i32
}

/// NozzleInventory.cpp:66-76 — check if the extruder holds a nozzle the selected variant can print
/// with.
fn holds_selected_diameter(slots: &ExtruderSlots, printer: &SelectedPrinter) -> bool {
    // On a rack the selected diameter only filters the slots; there any installed nozzle keeps the layer.
    // The multi-diameter mode applies only to a rack, so it never changes the result here.
    if printer.has_rack {
        return installed_count(slots) > 0;
    }
    slots.iter().any(|slot| slot.diameter == printer.diameter)
}

/// NozzleInventory.cpp:78-85 — cut or pad the slots to the slot count of the extruder; a padded slot
/// is empty.
fn fit_slot_count(mut slots: ExtruderSlots, slot_count: i32) -> ExtruderSlots {
    slots.resize(slot_count as usize, NozzleSlot::default());
    slots
}

/// NozzleInventory.cpp:87-97 — installed slots of one diameter, one slot per volume type in `types`.
fn uniform_slots(diameter: &str, types: &[NozzleVolumeType]) -> ExtruderSlots {
    let mut slots = ExtruderSlots::with_capacity(types.len());
    for &type_ in types {
        slots.push(NozzleSlot { diameter: diameter.to_string(), volume_type: type_, status: NozzleSlotStatus::Normal });
    }
    slots
}

/// NozzleInventory.cpp:99-108
fn slots_text(slots: &ExtruderSlots) -> String {
    let mut out = String::new();
    for slot in slots {
        if !out.is_empty() {
            out.push('|');
        }
        if slot.empty() {
            out += "-";
        } else {
            out += &(slot.diameter.clone() + " " + &get_nozzle_volume_type_str(slot.volume_type));
        }
    }
    out
}

/// NozzleInventory.cpp:110-126 — text of a layer for logs.
///
/// `name` is the layer name in the text; a `layer_or_null` of `None` reads as an empty layer.
fn layer_text(name: &str, layer_or_null: Option<&NozzleLayer>) -> String {
    let none = NozzleLayer::default();
    let layer = layer_or_null.unwrap_or(&none);
    if layer.stamp == 0 {
        return name.to_string() + "=none";
    }
    let mut out = name.to_string() + "#" + &layer.stamp.to_string() + "(" + &layer.model + ")=";
    for eid in 0..layer.extruders.len() {
        out += &((if eid != 0 { "," } else { "" }).to_string() + &slots_text(&layer.extruders[eid]));
    }
    out
}

/// NozzleInventory.cpp:128-154
fn read_selected_printer(printers: &PrinterPresetSelection) -> SelectedPrinter {
    let mut printer = SelectedPrinter::default();
    let config = &printers.edited_preset_config;
    printer.model = NozzlePrintConfig::opt_string(&config.printer_model, "printer_model").to_string();
    printer.history_key = match printers.preset_base_name.as_ref() {
        Some(base) => base.clone(),
        None => printers.selected_preset_name.clone(),
    };
    printer.diameter = format_diameter_to_str(
        string_to_double_decimal_point(NozzlePrintConfig::opt_string(&config.printer_variant, "printer_variant")).0,
    );
    printer.supports_multi_diameter = is_single_ext_multi_nozzle(config);

    let nozzle_diameters = config.nozzle_diameter.as_ref();
    let max_counts = config.extruder_max_nozzle_count.as_ref();
    let extruder_count = match nozzle_diameters {
        Some(values) => values.len(),
        None => 1,
    };
    for eid in 0..extruder_count {
        let mut slot_count = 1;
        if let Some(max_counts) = max_counts {
            if eid < max_counts.len() && !NozzlePrintConfig::ints_nullable_is_nil(max_counts, eid) && max_counts[eid] > 0 {
                slot_count = max_counts[eid];
            }
        }
        let mut types: Vec<NozzleVolumeType> = Vec::new();
        for slot in 0..slot_count {
            types.push(get_default_nozzle_volume_type(config, eid as i32, slot));
        }
        printer.slot_counts.push(slot_count);
        printer.has_rack = printer.has_rack || slot_count > 1;
        printer.default_types.push(types);
    }
    printer
}

/// NozzleInventory.cpp:156-187 — the project's per-slot keys as slots.
///
/// Gives nothing if the keys are missing or do not fit the printer.
fn read_per_slot_keys(project: &NozzlePrintConfig, printer: &SelectedPrinter) -> Option<Vec<ExtruderSlots>> {
    if !per_slot_nozzle_options_consistent(project) {
        return None;
    }
    // The four keys are present: the check above counts a missing one as empty, and wants a nozzle.
    let counts: &Vec<i32> = project.extruder_nozzle_count.as_ref()?;
    let types: &Vec<i32> = project.extruder_nozzle_volume_type.as_ref()?;
    let diams: &Vec<f64> = project.extruder_nozzle_diameter.as_ref()?;
    let ids: &Vec<i32> = project.extruder_nozzle_slot.as_ref()?;
    if counts.len() as i32 != printer.extruder_count() {
        return None;
    }

    let mut extruders: Vec<ExtruderSlots> = vec![ExtruderSlots::new(); counts.len()];
    let mut flat: usize = 0; // index into the flat per-nozzle arrays
    let mut global_base: i32 = 0; // slot ids count across all extruders
    for eid in 0..counts.len() {
        let slot_count = counts[eid].max(printer.slot_counts[eid]);
        extruders[eid].resize(slot_count as usize, NozzleSlot::default());
        for k in 0..counts[eid] {
            let mut local = ids[flat].wrapping_sub(global_base);
            if local < 0 || local >= slot_count {
                local = k;
            }
            extruders[eid][local as usize] = NozzleSlot {
                diameter: format_diameter_to_str(diams[flat]),
                volume_type: NozzleVolumeType(types[flat]),
                status: NozzleSlotStatus::Normal,
            };
            flat += 1;
        }
        global_base += slot_count;
    }
    Some(extruders)
}

/// NozzleInventory.cpp:189-199 — the legacy counts option, or nothing if it is missing, empty or has
/// an empty entry.
///
/// C++ takes the config and the key; the caller passes the option of that key.
fn valid_legacy_counts(opt: Option<&Vec<String>>) -> Option<&Vec<String>> {
    let opt = opt?;
    if opt.is_empty() {
        return None;
    }
    let has_empty = opt.iter().any(|value| value.is_empty());
    if has_empty {
        None
    } else {
        Some(opt)
    }
}

/// NozzleInventory.cpp:201-224 — slots from the legacy per-extruder counts.
///
/// The nozzles take the selected diameter.
fn read_legacy_counts(project: &NozzlePrintConfig, printer: &SelectedPrinter) -> Option<Vec<ExtruderSlots>> {
    // The new key keeps the real type names; the old one may be downgraded on export.
    let mut opt = valid_legacy_counts(project.extruder_nozzle_stats_new.as_ref());
    if opt.is_none() {
        opt = valid_legacy_counts(project.extruder_nozzle_stats.as_ref());
    }
    let opt = opt?;
    if opt.len() as i32 != printer.extruder_count() {
        return None;
    }

    let counts: Vec<BTreeMap<NozzleVolumeType, i32>> = get_extruder_nozzle_stats(opt);
    let mut extruders: Vec<ExtruderSlots> = vec![ExtruderSlots::new(); counts.len()];
    for eid in 0..counts.len() {
        for (&type_, &count) in counts[eid].iter() {
            let mut i = 0;
            while i < count && (extruders[eid].len() as i32) < printer.slot_counts[eid] {
                extruders[eid].push(NozzleSlot {
                    diameter: printer.diameter.clone(),
                    volume_type: type_,
                    status: NozzleSlotStatus::Normal,
                });
                i += 1;
            }
        }
        extruders[eid].resize(printer.slot_counts[eid] as usize, NozzleSlot::default());
    }
    Some(extruders)
}

/// NozzleInventory.cpp:226-245 — slots from the per-extruder volume type alone.
///
/// All slots are installed.
fn read_volume_types(project: &NozzlePrintConfig, printer: &SelectedPrinter) -> Option<Vec<ExtruderSlots>> {
    let opt = project.nozzle_volume_type.as_ref()?;
    if opt.len() as i32 != printer.extruder_count() {
        return None;
    }
    let mut extruders: Vec<ExtruderSlots> = Vec::new();
    for eid in 0..printer.extruder_count() as usize {
        let type_ = NozzleVolumeType(opt[eid]);
        // Hybrid only says the slots differ; the preset gives the mix.
        let types: Vec<NozzleVolumeType> = if type_ == NVT_HYBRID {
            printer.default_types[eid].clone()
        } else {
            vec![type_; printer.slot_counts[eid] as usize]
        };
        extruders.push(uniform_slots(&printer.diameter, &types));
    }
    Some(extruders)
}

/// NozzleInventory.cpp:247-265 — replace with Standard the volume types that no nozzle of the slot's
/// diameter is made with.
///
/// Printer presets still list such combinations, and a machine or an old project can report them,
/// so every source goes through `nozzle_variant_exists()`. `extruder_id` is for the log only.
fn drop_unmade_volume_types(slots: &mut ExtruderSlots, extruder_id: i32) {
    for slot in slots.iter_mut() {
        if slot.empty() || nozzle_variant_exists(&slot.diameter, slot.volume_type) {
            continue;
        }
        boost_log_trivial!(
            Warning,
            kv(log_key::EXTRUDER_ID, &extruder_id),
            slot.diameter,
            " nozzle ",
            get_nozzle_volume_type_str(slot.volume_type),
            " replaced by Standard"
        );
        slot.volume_type = NVT_STANDARD;
    }
}

/// NozzleInventory.cpp:267-270 — one extruder of a history value: a volume type per slot, nothing
/// for an empty slot.
type HistoryTypes = Vec<Option<NozzleVolumeType>>;

/// NozzleInventory.cpp:272-292 — encode slots as a history value of BambuStudio.conf.
///
/// Extruders are joined by ',' and slots by '|'. A slot is its volume type name, or nothing if it is
/// empty; an older version reads that empty token as Standard.
fn encode_history(extruders: &[ExtruderSlots]) -> String {
    let mut out = String::new();
    for eid in 0..extruders.len() {
        if eid > 0 {
            out.push(',');
        }
        for slot in 0..extruders[eid].len() {
            if slot > 0 {
                out.push('|');
            }
            if !extruders[eid][slot].empty() {
                out += &get_nozzle_volume_type_str(extruders[eid][slot].volume_type);
            }
        }
    }
    out
}

/// NozzleInventory.cpp:294-305 — the volume type named `token`.
///
/// Gives nothing for a name no volume type has; reading it as Standard would invent a nozzle.
fn parse_volume_type(token: &str) -> Option<NozzleVolumeType> {
    let type_ = nozzle_volume_type_from_string(token);
    if get_nozzle_volume_type_str(type_) != token {
        return None;
    }
    Some(type_)
}

/// `boost::trim` with the classic locale: the characters `std::isspace` accepts.
fn boost_trim(token: &str) -> &str {
    token.trim_matches(|c| matches!(c, ' ' | '\t' | '\n' | '\x0b' | '\x0c' | '\r'))
}

/// NozzleInventory.cpp:307-341 — decode a history value.
///
/// An old value has one token per extruder. Gives an empty list if a token names no volume type, so
/// the next source is used.
fn decode_history(value: &str) -> Vec<HistoryTypes> {
    let mut extruders: Vec<HistoryTypes> = Vec::new();
    if value.is_empty() {
        return extruders;
    }
    for extruder_token in value.split(',') {
        let mut types = HistoryTypes::new();
        for token in extruder_token.split('|') {
            let token = boost_trim(token);
            if token.is_empty() {
                types.push(None);
                continue;
            }
            let type_ = parse_volume_type(token);
            if type_.is_none() {
                boost_log_trivial!(Warning, "nozzle history ignored, unknown volume type ", token);
                return Vec::new();
            }
            types.push(type_);
        }
        extruders.push(types);
    }
    extruders
}

/// NozzleInventory.cpp:343-348 — BambuStudio.conf section with the full slots of each printer model,
/// diameters included.
///
/// A separate section, because older versions reject a nozzle_volume_types value with a diameter in it.
const SLOT_HISTORY_SECTION: &str = "nozzle_slots";

/// NozzleInventory.cpp:350-371 — encode slots as a value of the section nozzle_slots.
///
/// Extruders are joined by ',' and slots by '|'. A slot is "<diameter> <volume type>", or nothing if
/// it is empty. The status is not kept: a restored slot is Normal.
fn encode_slot_history(extruders: &[ExtruderSlots]) -> String {
    let mut out = String::new();
    for eid in 0..extruders.len() {
        if eid > 0 {
            out.push(',');
        }
        for slot in 0..extruders[eid].len() {
            if slot > 0 {
                out.push('|');
            }
            let nozzle = &extruders[eid][slot];
            if !nozzle.empty() {
                out += &(nozzle.diameter.clone() + " " + &get_nozzle_volume_type_str(nozzle.volume_type));
            }
        }
    }
    out
}

/// NozzleInventory.cpp:373-409 — decode a value of the section nozzle_slots.
///
/// A token without a diameter is an empty slot. Gives an empty list if a token names no volume type,
/// so the next source is used.
fn decode_slot_history(value: &str) -> Vec<ExtruderSlots> {
    let mut extruders: Vec<ExtruderSlots> = Vec::new();
    if value.is_empty() {
        return extruders;
    }
    for extruder_token in value.split(',') {
        let mut slots = ExtruderSlots::new();
        for token in extruder_token.split('|') {
            let token = boost_trim(token);
            let space = token.find(' ');
            let diameter = match space {
                None => 0.0,
                Some(space) => string_to_double_decimal_point(&token[..space]).0,
            };
            if diameter <= 0.0 {
                slots.push(NozzleSlot::default());
                continue;
            }
            // A diameter above zero was read, so the token has a space.
            let space = space.unwrap_or(0);
            let type_ = parse_volume_type(&token[space + 1..]);
            let Some(type_) = type_ else {
                boost_log_trivial!(Warning, "nozzle slot history ignored, unknown volume type ", token);
                return Vec::new();
            };
            slots.push(NozzleSlot {
                diameter: format_diameter_to_str(diameter),
                volume_type: type_,
                status: NozzleSlotStatus::Normal,
            });
        }
        extruders.push(slots);
    }
    extruders
}

/// NozzleInventory.cpp:411-431 — number of slots per volume type.
///
/// Counts the slots of the selected diameter, or all installed slots if none has it. With
/// `multi_diameter` the multi-diameter mode is on; then all installed slots count.
fn count_slots(slots: &ExtruderSlots, printer: &SelectedPrinter, multi_diameter: bool) -> BTreeMap<NozzleVolumeType, i32> {
    let mut counts: BTreeMap<NozzleVolumeType, i32> = BTreeMap::new();
    if !multi_diameter {
        for slot in slots {
            if slot.diameter == printer.diameter {
                *counts.entry(slot.volume_type).or_insert(0) += 1;
            }
        }
    }
    // An extruder that holds only other diameters is still installed; it must stay in grouping.
    if counts.is_empty() {
        for slot in slots {
            if !slot.empty() {
                *counts.entry(slot.volume_type).or_insert(0) += 1;
            }
        }
    }
    counts
}

/// NozzleInventory.cpp:433-441 — the single volume type of `counts`; Hybrid if they differ, Standard
/// if there is none.
fn summary_volume_type(counts: &BTreeMap<NozzleVolumeType, i32>) -> NozzleVolumeType {
    if counts.is_empty() {
        return NVT_STANDARD;
    }
    if counts.len() == 1 {
        *counts.keys().next().unwrap_or(&NVT_STANDARD)
    } else {
        NVT_HYBRID
    }
}

/// Lock the mirror or the history; a panic of another holder must not lose the inventory.
fn lock<T>(mutex: &Mutex<T>) -> MutexGuard<'_, T> {
    mutex.lock().unwrap_or_else(|poisoned| poisoned.into_inner())
}

/// NozzleInventory.cpp:445-620 `struct NozzleInventory::Impl`
struct Impl {
    mirror: Arc<Mutex<NozzlePrintConfig>>,
    /// BambuStudio.conf sections nozzle_volume_types and nozzle_slots; null in scratch copies
    history: Option<Arc<Mutex<AppConfig>>>,
    /// what resolve() reads from the selected printer preset
    printer: SelectedPrinter,
    project: NozzleLayer,
    machine: NozzleLayer,
    user: NozzleLayer,
    last_stamp: u64,

    result: Vec<ExtruderSlots>,
    sources: Vec<NozzleSource>,
}

impl Impl {
    /// NozzleInventory.cpp:456
    fn new(mirror: Arc<Mutex<NozzlePrintConfig>>) -> Impl {
        Impl {
            mirror,
            history: None,
            printer: SelectedPrinter::default(),
            project: NozzleLayer::default(),
            machine: NozzleLayer::default(),
            user: NozzleLayer::default(),
            last_stamp: 0,
            result: Vec::new(),
            sources: Vec::new(),
        }
    }

    /// NozzleInventory.cpp:458-471 — the newest layer of the selected model with its extruder count,
    /// or nothing if none.
    fn top_layer(&self) -> Option<&NozzleLayer> {
        let mut top: Option<&NozzleLayer> = None;
        for layer in [&self.project, &self.machine, &self.user] {
            if layer.stamp == 0
                || layer.model != self.printer.model
                || layer.extruders.len() as i32 != self.printer.extruder_count()
            {
                continue;
            }
            if top.is_none_or(|top| layer.stamp > top.stamp) {
                top = Some(layer);
            }
        }
        top
    }

    /// NozzleInventory.cpp:473-486 — the full slots last saved for the selected model.
    ///
    /// Empty if they do not fit its extruders.
    fn model_history(&self) -> Vec<ExtruderSlots> {
        let Some(history) = self.history.as_ref() else {
            return Vec::new();
        };
        if self.printer.model.is_empty() {
            return Vec::new();
        }
        let extruders = decode_slot_history(&lock(history).get(SLOT_HISTORY_SECTION, &self.printer.model));
        if extruders.len() as i32 != self.printer.extruder_count() {
            return Vec::new();
        }
        extruders
    }

    /// NozzleInventory.cpp:488-496 — the saved volume types of the selected variant; empty if there
    /// is none.
    fn history_types(&self) -> Vec<HistoryTypes> {
        let Some(history) = self.history.as_ref() else {
            return Vec::new();
        };
        if self.printer.history_key.is_empty() {
            return Vec::new();
        }
        decode_history(&lock(history).get_nozzle_volume_types_from_config(&self.printer.history_key))
    }

    /// NozzleInventory.cpp:498-516 — history of the selected variant for one extruder.
    ///
    /// `grid` is the saved volume types of the variant, `eid` the 0-based extruder. Gives nothing if
    /// the extruder has no entry.
    fn history_slots(&self, grid: &[HistoryTypes], eid: i32) -> Option<ExtruderSlots> {
        if eid >= grid.len() as i32 || grid[eid as usize].is_empty() {
            return None;
        }
        let entry = &grid[eid as usize];
        let mut slots = ExtruderSlots::new();
        // A short entry repeats its last type over the remaining slots.
        for slot in 0..self.printer.slot_counts[eid as usize] {
            let type_: &Option<NozzleVolumeType> =
                if slot < entry.len() as i32 { &entry[slot as usize] } else { &entry[entry.len() - 1] };
            slots.push(match type_ {
                Some(type_) => NozzleSlot {
                    diameter: self.printer.diameter.clone(),
                    volume_type: *type_,
                    status: NozzleSlotStatus::Normal,
                },
                None => NozzleSlot::default(),
            });
        }
        Some(slots)
    }

    /// NozzleInventory.cpp:518-543 — compute the effective slots and their source for every extruder.
    fn resolve(&mut self) {
        let top: Option<NozzleLayer> = self.top_layer().cloned();
        let saved = self.model_history();
        let grid = self.history_types();
        self.result = vec![ExtruderSlots::new(); self.printer.extruder_count() as usize];
        self.sources = vec![NozzleSource::Preset; self.printer.extruder_count() as usize];
        for eid in 0..self.printer.extruder_count() {
            let e = eid as usize;
            if top.as_ref().is_some_and(|top| holds_selected_diameter(&top.extruders[e], &self.printer)) {
                let top = top.as_ref().expect("checked by the condition");
                self.result[e] = fit_slot_count(top.extruders[e].clone(), self.printer.slot_counts[e]);
                self.sources[e] = top.source;
            } else if !saved.is_empty() && holds_selected_diameter(&saved[e], &self.printer) {
                self.result[e] = fit_slot_count(saved[e].clone(), self.printer.slot_counts[e]);
                self.sources[e] = NozzleSource::History;
            } else if let Some(slots) = self.history_slots(&grid, eid) {
                self.result[e] = slots;
                self.sources[e] = NozzleSource::History;
            } else {
                self.result[e] = uniform_slots(&self.printer.diameter, &self.printer.default_types[e]);
            }
            drop_unmade_volume_types(&mut self.result[e], eid);
        }
    }

    /// NozzleInventory.cpp:545-563 — save the result as history.
    ///
    /// History keeps the last state of the printer, so only machine and user results go there.
    fn save_history(&self) {
        let top = self.top_layer();
        let (Some(history), Some(top)) = (self.history.as_ref(), top) else {
            return;
        };
        if top.source == NozzleSource::Project || self.printer.history_key.is_empty() {
            return;
        }
        if !self.sources.contains(&top.source) {
            return;
        }
        let mut history = lock(history);
        let encoded = encode_history(&self.result);
        if encoded != history.get_nozzle_volume_types_from_config(&self.printer.history_key) {
            history.save_nozzle_volume_types_to_config(&self.printer.history_key, &encoded);
        }
        let slots = encode_slot_history(&self.result);
        if !self.printer.model.is_empty() && slots != history.get(SLOT_HISTORY_SECTION, &self.printer.model) {
            history.set(SLOT_HISTORY_SECTION, &self.printer.model, &slots);
        }
    }

    /// NozzleInventory.cpp:565-581 — replace one layer and stamp it as the newest.
    ///
    /// Snaps each diameter to the preset spelling, such as "0.4"; an empty slot keeps its empty
    /// diameter. `layer` picks the member the C++ passes by reference.
    fn write(
        &mut self,
        layer: fn(&mut Impl) -> &mut NozzleLayer,
        source: NozzleSource,
        model: &str,
        mut extruders: Vec<ExtruderSlots>,
    ) {
        // Diameters are compared as text; one spelling here frees the writers from matching the preset.
        for slots in extruders.iter_mut() {
            for slot in slots.iter_mut() {
                if !slot.diameter.is_empty() {
                    slot.diameter = format_diameter_to_str(string_to_double_decimal_point(&slot.diameter).0);
                }
            }
        }
        self.last_stamp += 1;
        let stamp = self.last_stamp;
        let layer = layer(self);
        layer.source = source;
        layer.model = model.to_string();
        layer.extruders = extruders;
        layer.stamp = stamp;
    }

    /// NozzleInventory.cpp:583-595 — recompute, mirror, persist and log after any intent.
    ///
    /// `intent` is the name of the change, for the log; `self_` the owner, whose `export_to()` fills
    /// the mirror. C++ is a member of `Impl` that takes the owner; the owner holds the `Impl` here,
    /// so it is the only argument.
    fn commit(intent: &str, self_: &mut NozzleInventory) {
        self_.m_impl.resolve();
        self_.export_to_mirror();
        self_.m_impl.save_history();
        boost_log_trivial!(Info, "nozzle inventory ", intent, ": ", self_.m_impl.describe());
    }

    /// NozzleInventory.cpp:597-609 — all layers and the result, for logs.
    fn describe(&self) -> String {
        let mut out = "printer=".to_string()
            + &self.printer.history_key
            + " diameter="
            + &self.printer.diameter
            + (if self.multi_diameter() { " multi-diameter" } else { "" })
            + (if self.printer.has_rack { " rack" } else { "" })
            + " "
            + &layer_text("project", Some(&self.project))
            + " "
            + &layer_text("machine", Some(&self.machine))
            + " "
            + &layer_text("user", Some(&self.user))
            + " result=";
        for eid in 0..self.result.len() {
            out += &((if eid != 0 { "," } else { "" }).to_string()
                + source_name(self.sources[eid])
                + ":"
                + &slots_text(&self.result[eid]));
        }
        out
    }

    /// NozzleInventory.cpp:611-617 — check if the multi-diameter mode is on for the selected printer.
    ///
    /// Reads the mode from the project on each call, so no caller has to report a mode change. A
    /// printer without multi-diameter support reads off, even while the project still has the mode on.
    fn multi_diameter(&self) -> bool {
        self.printer.supports_multi_diameter && multi_nozzle_diameter_mode_enabled(&lock(&self.mirror))
    }

    /// NozzleInventory.cpp:619
    fn valid_extruder(&self, extruder_id: i32) -> bool {
        extruder_id >= 0 && extruder_id < self.result.len() as i32
    }
}

/// NozzleInventory.hpp:65-275 — nozzle inventory of the selected printer.
///
/// Each method changes one input of the resolve, and only its own event may call it:
///
/// | Method         | Layer   | Effect                                 | Only called by                             |
/// |----------------|---------|----------------------------------------|--------------------------------------------|
/// | select_printer | none    | new model, diameter, history, defaults | a printer, variant or diameter mode change |
/// | open_project   | Project | the file's nozzles win                 | opening a project file                     |
/// | close_project  | Project | the file's nozzles stop winning        | a new or closed project                    |
/// | sync_machine   | Machine | machine nozzles win, saved as history  | a sync with a connected machine            |
/// | user_*         | User    | user nozzles win, saved as history     | a user action in a nozzle dialog or combo  |
///
/// Every write updates the effective nozzles, writes the nozzle keys into the mirror config, saves
/// the history and writes one log line. Callers never order these steps.
pub struct NozzleInventory {
    m_impl: Box<Impl>,
}

impl NozzleInventory {
    /// NozzleInventory.cpp:622 — create an inventory that writes every change into `mirror`, the
    /// project config; it receives nozzle_volume_type, the per-slot keys and the legacy counts.
    pub fn new(mirror: Arc<Mutex<NozzlePrintConfig>>) -> NozzleInventory {
        NozzleInventory { m_impl: Box::new(Impl::new(mirror)) }
    }

    /// NozzleInventory.cpp:626-638 `operator=` — copy the layers and the selected printer, for a
    /// scratch copy of PresetBundle.
    ///
    /// This inventory keeps its own mirror and history, so a scratch copy never saves history.
    pub fn assign(&mut self, rhs: &NozzleInventory) -> &mut NozzleInventory {
        if std::ptr::eq(self, rhs) {
            return self;
        }
        self.m_impl.printer = rhs.m_impl.printer.clone();
        self.m_impl.project = rhs.m_impl.project.clone();
        self.m_impl.machine = rhs.m_impl.machine.clone();
        self.m_impl.user = rhs.m_impl.user.clone();
        self.m_impl.last_stamp = rhs.m_impl.last_stamp;
        self.m_impl.result = rhs.m_impl.result.clone();
        self.m_impl.sources = rhs.m_impl.sources.clone();
        self
    }

    /// NozzleInventory.cpp:640 — set the store of the per-printer history.
    ///
    /// The history is two sections of BambuStudio.conf, and each write saves both: nozzle_slots,
    /// keyed by printer model, with the full slots and their diameters; nozzle_volume_types, keyed by
    /// base preset name, with the volume types only. `None` (the default) means nothing is read or
    /// saved.
    pub fn set_history(&mut self, history: Option<Arc<Mutex<AppConfig>>>) {
        self.m_impl.history = history;
    }

    // ---- No layer: changes only what the layers resolve against ----

    /// NozzleInventory.cpp:642-646 — the selected printer or its variant changed.
    ///
    /// Reads from the selected preset the inputs of the resolve: printer model, diameter, history key,
    /// slot counts and preset defaults. The layers stay as they are, so a later switch back gives the
    /// same nozzles.
    pub fn select_printer(&mut self, printers: &PrinterPresetSelection) {
        self.m_impl.printer = read_selected_printer(printers);
        Impl::commit("select printer", self);
    }

    // ---- Project layer: only project open, new and close ----

    /// NozzleInventory.cpp:648-663 — a project file was opened; its nozzle keys replace the Project
    /// layer.
    ///
    /// Also selects the printer, so the file's printer must already be selected in `printers`. A file
    /// with no nozzle keys clears the layer. `file_keys` is the config as read from the file, so a
    /// missing key is really missing.
    pub fn open_project(&mut self, printers: &PrinterPresetSelection, file_keys: &NozzlePrintConfig) {
        self.m_impl.printer = read_selected_printer(printers);
        let mut extruders = read_per_slot_keys(file_keys, &self.m_impl.printer);
        if extruders.is_none() {
            extruders = read_legacy_counts(file_keys, &self.m_impl.printer);
        }
        if extruders.is_none() && file_keys.nozzle_volume_type.is_some() {
            extruders = read_volume_types(file_keys, &self.m_impl.printer);
        }
        let has_extruders = extruders.is_some();
        if let Some(extruders) = extruders {
            let model = self.m_impl.printer.model.clone();
            self.m_impl.write(|i| &mut i.project, NozzleSource::Project, &model, extruders);
        } else {
            self.m_impl.project = NozzleLayer::default();
        }
        if !has_extruders {
            boost_log_trivial!(Warning, "project has no usable nozzle keys, nozzles come from the printer");
        }
        Impl::commit("open project", self);
    }

    /// NozzleInventory.cpp:665-669 — the project was closed or a new project started; clears the
    /// Project layer.
    pub fn close_project(&mut self) {
        self.m_impl.project = NozzleLayer::default();
        Impl::commit("close project", self);
    }

    // ---- Machine layer: only a sync with a connected machine ----

    /// NozzleInventory.cpp:671-675 — a machine sync replaces the Machine layer, even when nothing
    /// changed.
    ///
    /// `printer_model` is the printer_model of the synced machine; the layer applies only to that
    /// model. `layout` is the slots per extruder, as the machine reports them; a diameter in any
    /// spelling of a standard size, such as "0.40", reads as that size.
    pub fn sync_machine(&mut self, printer_model: &str, layout: Vec<ExtruderSlots>) {
        self.m_impl.write(|i| &mut i.machine, NozzleSource::Machine, printer_model, layout);
        Impl::commit("sync machine", self);
    }

    // ---- User layer: only a user action in a nozzle dialog or combo ----

    /// NozzleInventory.cpp:677-687 — the user set all slots of one extruder.
    pub fn user_set_slots(&mut self, extruder_id: i32, slots: ExtruderSlots) {
        if !self.m_impl.valid_extruder(extruder_id) {
            boost_log_trivial!(
                Warning,
                kv(log_key::EXTRUDER_ID, &extruder_id),
                "user set slots ignored, no such extruder"
            );
            return;
        }
        let mut extruders: Vec<ExtruderSlots> = self.m_impl.result.clone();
        extruders[extruder_id as usize] = fit_slot_count(slots, self.m_impl.printer.slot_counts[extruder_id as usize]);
        let model = self.m_impl.printer.model.clone();
        self.m_impl.write(|i| &mut i.user, NozzleSource::User, &model, extruders);
        Impl::commit("user set slots", self);
    }

    /// NozzleInventory.cpp:689-716 — the user set the nozzle count per volume type of the selected
    /// diameter.
    ///
    /// A missing type means no nozzle of that type.
    pub fn user_set_counts(&mut self, extruder_id: i32, counts: &BTreeMap<NozzleVolumeType, i32>) {
        if !self.m_impl.valid_extruder(extruder_id) {
            boost_log_trivial!(
                Warning,
                kv(log_key::EXTRUDER_ID, &extruder_id),
                "user set counts ignored, no such extruder"
            );
            return;
        }
        // The counts describe the nozzles of the selected diameter; other diameters keep their slots.
        let mut slots: ExtruderSlots = self.m_impl.result[extruder_id as usize].clone();
        for slot in slots.iter_mut() {
            if slot.diameter == self.m_impl.printer.diameter || self.m_impl.multi_diameter() {
                *slot = NozzleSlot::default();
            }
        }
        let mut next_free: usize = 0;
        let mut dropped: i32 = 0;
        for (&type_, &count) in counts.iter() {
            for i in 0..count {
                next_free = match slots[next_free..].iter().position(|slot| slot.empty()) {
                    Some(offset) => next_free + offset,
                    None => slots.len(),
                };
                if next_free == slots.len() {
                    dropped += count - i;
                    break;
                }
                slots[next_free] = NozzleSlot {
                    diameter: self.m_impl.printer.diameter.clone(),
                    volume_type: type_,
                    status: NozzleSlotStatus::Normal,
                };
            }
        }
        if dropped > 0 {
            boost_log_trivial!(
                Warning,
                kv(log_key::EXTRUDER_ID, &extruder_id),
                "user set counts: ",
                dropped,
                " nozzles dropped, no free slot"
            );
        }
        self.user_set_slots(extruder_id, slots);
    }

    /// NozzleInventory.cpp:718-725 — the user picked the volume type of an extruder with one nozzle
    /// slot.
    ///
    /// An extruder with a rack derives its volume type from its slots; there this logs a warning and
    /// does nothing.
    pub fn user_set_volume_type(&mut self, extruder_id: i32, type_: NozzleVolumeType) {
        if !self.m_impl.valid_extruder(extruder_id)
            || type_ == NVT_HYBRID
            || self.m_impl.printer.slot_counts[extruder_id as usize] != 1
        {
            boost_log_trivial!(
                Warning,
                kv(log_key::EXTRUDER_ID, &extruder_id),
                "user set volume type ignored, extruder has no single nozzle"
            );
            return;
        }
        let slot = NozzleSlot {
            diameter: self.m_impl.printer.diameter.clone(),
            volume_type: type_,
            status: NozzleSlotStatus::Normal,
        };
        self.user_set_slots(extruder_id, vec![slot]);
    }

    /// NozzleInventory.cpp:727-742 — the user picked one diameter for all extruders.
    ///
    /// Every installed nozzle of the newest layer takes `diameter` and keeps its volume type, so a
    /// nozzle of another diameter is kept as the user meant it, not replaced by the history. Without
    /// a layer for the selected printer this logs a warning and does nothing.
    pub fn user_unify_diameter(&mut self, diameter: &str) {
        // The newest layer, not the result: the result already replaced a nozzle of another diameter.
        let Some(top) = self.m_impl.top_layer() else {
            boost_log_trivial!(Warning, "user unify diameter ignored, no nozzles of the selected printer");
            return;
        };
        let mut extruders: Vec<ExtruderSlots> = top.extruders.clone();
        for slots in extruders.iter_mut() {
            for slot in slots.iter_mut() {
                if !slot.empty() {
                    slot.diameter = diameter.to_string();
                }
            }
        }
        let model = self.m_impl.printer.model.clone();
        self.m_impl.write(|i| &mut i.user, NozzleSource::User, &model, extruders);
        Impl::commit("user unify diameter", self);
    }

    // ---- Read ----

    /// NozzleInventory.cpp:744 — number of extruders of the selected printer.
    pub fn extruder_count(&self) -> i32 {
        self.m_impl.result.len() as i32
    }

    /// NozzleInventory.cpp:746-750 — effective slots of one extruder; empty for an unknown extruder.
    pub fn slots(&self, extruder_id: i32) -> &ExtruderSlots {
        static NONE: ExtruderSlots = Vec::new();
        if self.m_impl.valid_extruder(extruder_id) {
            &self.m_impl.result[extruder_id as usize]
        } else {
            &NONE
        }
    }

    /// NozzleInventory.cpp:752-763 — number of slots per volume type.
    ///
    /// Counts the slots of the selected diameter, or all installed slots if none has it. With
    /// `diameter`, counts only the slots of this diameter, without that fallback; use it to ask how
    /// many nozzles a print of one diameter can really use.
    pub fn counts(&self, extruder_id: i32, diameter: Option<&str>) -> BTreeMap<NozzleVolumeType, i32> {
        let Some(diameter) = diameter else {
            return count_slots(self.slots(extruder_id), &self.m_impl.printer, self.m_impl.multi_diameter());
        };

        let wanted = format_diameter_to_str(string_to_double_decimal_point(diameter).0);
        let mut counts: BTreeMap<NozzleVolumeType, i32> = BTreeMap::new();
        for slot in self.slots(extruder_id) {
            if !slot.empty() && slot.diameter == wanted {
                *counts.entry(slot.volume_type).or_insert(0) += 1;
            }
        }
        counts
    }

    /// NozzleInventory.cpp:765-776 — one entry of `counts()`; no type or Hybrid gives the total.
    pub fn count(&self, extruder_id: i32, type_: Option<NozzleVolumeType>, diameter: Option<&str>) -> i32 {
        let all = self.counts(extruder_id, diameter);
        let Some(type_) = type_.filter(|type_| *type_ != NVT_HYBRID) else {
            let mut total = 0;
            for entry in all.iter() {
                total += *entry.1;
            }
            return total;
        };
        match all.get(&type_) {
            None => 0,
            Some(count) => *count,
        }
    }

    /// NozzleInventory.cpp:778-781 — volume type of one extruder: the single type of `counts()`;
    /// Hybrid if they differ, Standard if there is none.
    pub fn volume_type(&self, extruder_id: i32) -> NozzleVolumeType {
        summary_volume_type(&self.counts(extruder_id, None))
    }

    /// NozzleInventory.cpp:783 — write the nozzle keys into the mirror again.
    ///
    /// Call it after a bulk copy into the mirror replaced the nozzle keys, or after the
    /// multi-diameter mode changed, because the counts follow the mode.
    pub fn refresh_mirror(&mut self) {
        self.export_to_mirror();
    }

    /// `export_to(m_impl->mirror)` of NozzleInventory.cpp:592 and 783.
    ///
    /// FIDELITY-NOTE: `export_to` reads the multi-diameter mode from the mirror (through `counts`)
    /// while it has the mirror to write to; with the mirror behind a lock that is one borrow too
    /// many, so the keys are written into a copy that then replaces the mirror. `export_to` does all
    /// its reads before its first write, so the result is the same.
    fn export_to_mirror(&self) {
        let mut config = lock(&self.m_impl.mirror).clone();
        self.export_to(&mut config);
        *lock(&self.m_impl.mirror) = config;
    }

    /// NozzleInventory.cpp:785-807 — write the nozzle keys that slicing and project files read.
    ///
    /// Writes nothing before a printer is selected, so the keys keep their values. Other keys of
    /// `config` stay as they are.
    pub fn export_to(&self, config: &mut NozzlePrintConfig) {
        // Before the first select_printer() there are no extruders; empty keys would break readers of values[0].
        if self.extruder_count() == 0 {
            return;
        }
        let mut volume_types: Vec<i32> = Vec::new();
        let mut all_counts: Vec<BTreeMap<NozzleVolumeType, i32>> = Vec::new();
        let mut full_stat = ExtruderFullStat::default();
        full_stat.resize(self.extruder_count() as usize);
        for eid in 0..self.extruder_count() {
            all_counts.push(self.counts(eid, None));
            volume_types.push(summary_volume_type(&all_counts[all_counts.len() - 1]).0);
            let mut stats: Vec<(String, String)> = Vec::new();
            for slot in self.slots(eid) {
                stats.push((
                    slot.diameter.clone(),
                    if slot.empty() { String::new() } else { get_nozzle_volume_type_str(slot.volume_type) },
                ));
            }
            full_stat.set_extruder_stats(eid as usize, &stats);
        }
        config.nozzle_volume_type = Some(volume_types);
        let legacy: Vec<String> = save_extruder_nozzle_stats_to_string(&all_counts);
        config.extruder_nozzle_stats = Some(legacy.clone());
        config.extruder_nozzle_stats_new = Some(legacy);
        sync_per_slot_nozzle_options_from_full_stat(config, &full_stat);
    }
}

// =====================================================================================================
// Functions of PrintConfig and PresetBundle at the pin that this unit calls and the crate does not have
// (see the module notes). Each is ported from the lines it cites.
// =====================================================================================================

/// PrintConfig.cpp:897-904 `s_keys_map_NozzleVolumeType`
const S_KEYS_MAP_NOZZLE_VOLUME_TYPE: [(&str, NozzleVolumeType); 6] = [
    ("Standard", NVT_STANDARD),
    ("High Flow", NVT_HIGH_FLOW),
    ("TPU High Flow", NVT_TPU_HIGH_FLOW),
    ("TPU285", NVT_TPU285),
    ("Hybrid", NVT_HYBRID),
    ("E3D High Flow", NVT_E3D_HIGH_FLOW),
];

/// PrintConfig.cpp:905 `s_keys_names_NozzleVolumeType`: the names by enum value.
const S_KEYS_NAMES_NOZZLE_VOLUME_TYPE: [&str; 6] =
    ["Standard", "High Flow", "Hybrid", "TPU High Flow", "TPU285", "E3D High Flow"];

/// PrintConfig.cpp:139-143
fn multi_nozzle_diameter_mode_enabled(config: &NozzlePrintConfig) -> bool {
    config.multi_nozzle_diameter_mode.is_some_and(|value| value)
}

/// PrintConfig.cpp:1023-1030
fn get_nozzle_volume_type_string(nozzle_volume_type: NozzleVolumeType) -> String {
    if nozzle_volume_type.0 < 0 || nozzle_volume_type >= NVT_COUNT {
        boost_log_trivial!(Error, "unsupported NozzleVolumeType=", nozzle_volume_type.0);
        return String::new();
    }
    S_KEYS_NAMES_NOZZLE_VOLUME_TYPE[nozzle_volume_type.0 as usize].to_string()
}

/// PrintConfig.cpp:1199-1210
fn is_single_ext_multi_nozzle(config: &NozzlePrintConfig) -> bool {
    if !config.handle_hotend_as_extruder.is_some_and(|value| value) {
        return false;
    }
    if !config.nozzle_diameter.as_ref().is_some_and(|values| values.len() == 1) {
        return false;
    }
    config.extruder_max_nozzle_count.as_ref().is_some_and(|values| values.iter().any(|&v| v > 1))
}

/// `std::atoi`: leading white space, an optional sign, then digits; 0 if there is no number.
fn atoi(text: &str) -> i32 {
    let bytes = text.as_bytes();
    let mut i = 0;
    while i < bytes.len() && matches!(bytes[i], b' ' | b'\t' | b'\n' | 0x0b | 0x0c | b'\r') {
        i += 1;
    }
    let mut negative = false;
    if i < bytes.len() && (bytes[i] == b'+' || bytes[i] == b'-') {
        negative = bytes[i] == b'-';
        i += 1;
    }
    let mut value: i64 = 0;
    while i < bytes.len() && bytes[i].is_ascii_digit() {
        // Out of the range of int the result of atoi is undefined; this keeps the low bits.
        value = value.wrapping_mul(10).wrapping_add((bytes[i] - b'0') as i64);
        i += 1;
    }
    (if negative { value.wrapping_neg() } else { value }) as i32
}

/// PrintConfig.cpp:1212-1242
fn get_extruder_nozzle_stats(strs: &[String]) -> Vec<BTreeMap<NozzleVolumeType, i32>> {
    let mut extruder_nozzle_counts: Vec<BTreeMap<NozzleVolumeType, i32>> = Vec::new();
    for str in strs {
        let mut nozzle_count_map: BTreeMap<NozzleVolumeType, i32> = BTreeMap::new();
        if str.is_empty() {
            extruder_nozzle_counts.push(nozzle_count_map);
            continue;
        }
        for nozzle_info in str.split('|') {
            let attr: Vec<&str> = nozzle_info.split('#').collect();
            if attr.len() < 2 {
                boost_log_trivial!(Error, "malformed nozzle stat entry \"", nozzle_info, "\", skipped");
                continue;
            }

            let mut volume_type = NVT_STANDARD;
            match S_KEYS_MAP_NOZZLE_VOLUME_TYPE.iter().find(|(name, _)| *name == attr[0]) {
                Some((_, type_)) => volume_type = *type_,
                None => {
                    boost_log_trivial!(Error, "unknown NozzleVolumeType \"", attr[0], "\", fall back to Standard")
                }
            }
            let nozzle_count = atoi(attr[1]);
            // Accumulate so that several unknown types collapsing onto Standard keep the total count.
            let entry = nozzle_count_map.entry(volume_type).or_insert(0);
            *entry = entry.wrapping_add(nozzle_count);
        }
        extruder_nozzle_counts.push(nozzle_count_map);
    }
    extruder_nozzle_counts
}

/// PrintConfig.cpp:1264-1270
fn nozzle_volume_type_from_string(name: &str) -> NozzleVolumeType {
    match S_KEYS_MAP_NOZZLE_VOLUME_TYPE.iter().find(|(key, _)| *key == name) {
        Some((_, type_)) => *type_,
        None => NVT_STANDARD,
    }
}

/// PrintConfig.cpp:1272-1310 — default volume types of one extruder's physical nozzle slots, in
/// physical order. Lists no longer than the extruder count hold one value per extruder (extruders
/// past the list reuse its last value); a list as long as all slots of all extruders holds one value
/// per slot; any other length warns and yields Standard.
fn default_nozzle_volume_slots(config: &NozzlePrintConfig, extruder_id: i32) -> Vec<NozzleVolumeType> {
    let diameters = config.nozzle_diameter.as_ref();
    let max_counts = config.extruder_max_nozzle_count.as_ref();
    let extruder_count: i32 = match diameters {
        Some(values) => values.len() as i32,
        None => 1,
    };
    if extruder_id < 0 || extruder_id >= extruder_count {
        return Vec::new();
    }

    let slot_count = |eid: i32| -> i32 {
        match max_counts {
            Some(values) if eid < values.len() as i32 && !NozzlePrintConfig::ints_nullable_is_nil(values, eid as usize) => {
                1.max(values[eid as usize])
            }
            _ => 1,
        }
    };

    let mut slots: Vec<NozzleVolumeType> = vec![NVT_STANDARD; slot_count(extruder_id) as usize];
    let opt = config.default_nozzle_volume_type.as_ref();
    let count: i32 = match opt {
        Some(values) => values.len() as i32,
        None => 0,
    };
    let Some(opt) = opt.filter(|_| count != 0) else {
        return slots;
    };

    if count <= extruder_count {
        slots.fill(NozzleVolumeType(opt[extruder_id.min(count - 1) as usize]));
        return slots;
    }

    let mut offset = 0;
    let mut total = 0;
    for eid in 0..extruder_count {
        if eid == extruder_id {
            offset = total;
        }
        total += slot_count(eid);
    }
    if count != total {
        let model = config.printer_model.as_ref();
        boost_log_trivial!(
            Warning,
            "default_nozzle_volume_type has ",
            count,
            " values, expected 1, ",
            extruder_count,
            " or ",
            total,
            ", machine: ",
            match model {
                Some(model) => model.clone(),
                None => "unknown".to_string(),
            }
        );
        return slots;
    }
    for slot in 0..slots.len() {
        slots[slot] = NozzleVolumeType(opt[offset as usize + slot]);
    }
    slots
}

/// PrintConfig.cpp:1323-1327
fn get_default_nozzle_volume_type(config: &NozzlePrintConfig, extruder_id: i32, nozzle_id: i32) -> NozzleVolumeType {
    let slots = default_nozzle_volume_slots(config, extruder_id);
    if nozzle_id >= 0 && nozzle_id < slots.len() as i32 {
        slots[nozzle_id as usize]
    } else {
        NVT_STANDARD
    }
}

/// PrintConfig.cpp:1443-1457
fn save_extruder_nozzle_stats_to_string(extruder_nozzle_stats: &[BTreeMap<NozzleVolumeType, i32>]) -> Vec<String> {
    let mut extruder_nozzle_count_str: Vec<String> = Vec::new();
    for idx in 0..extruder_nozzle_stats.len() {
        let mut oss = String::new();
        let item = &extruder_nozzle_stats[idx];
        let mut it = item.iter().peekable();
        while let Some((type_, count)) = it.next() {
            oss += &get_nozzle_volume_type_string(*type_);
            oss += "#";
            oss += &count.to_string();
            if it.peek().is_some() {
                oss += "|";
            }
        }
        extruder_nozzle_count_str.push(oss);
    }
    extruder_nozzle_count_str
}

/// PrintConfig.hpp:2222-2236 — whether a nozzle of this (diameter, flow) is a product that exists at
/// all: E3D high flow only comes in 0.4 and 0.6, and 0.2 comes as Standard only.
fn nozzle_variant_exists(diameter: &str, volume_type: NozzleVolumeType) -> bool {
    if volume_type == NVT_E3D_HIGH_FLOW && diameter != "0.4" && diameter != "0.6" {
        return false;
    }
    if diameter == "0.2" && volume_type != NVT_STANDARD {
        return false;
    }
    true
}

/// PresetBundle.hpp:113-148 `struct ExtruderFullStat`, the members this unit and
/// `sync_per_slot_nozzle_options_from_full_stat` use: per extruder, per nozzle slot, the pair
/// (diameter, volume type name).
#[derive(Debug, Clone, Default)]
struct ExtruderFullStat {
    extruder_nozzle_stats: Vec<Vec<(String, String)>>,
}

impl ExtruderFullStat {
    /// PresetBundle.hpp:118
    fn get_raw_stat(&self) -> &Vec<Vec<(String, String)>> {
        &self.extruder_nozzle_stats
    }

    /// PresetBundle.hpp:131
    fn resize(&mut self, extruder_count: usize) {
        self.extruder_nozzle_stats.resize(extruder_count, Vec::new());
    }

    /// PresetBundle.hpp:132-136
    fn set_extruder_stats(&mut self, extruder_id: usize, stats: &[(String, String)]) {
        if extruder_id >= self.extruder_nozzle_stats.len() {
            self.extruder_nozzle_stats.resize(extruder_id + 1, Vec::new());
        }
        self.extruder_nozzle_stats[extruder_id] = stats.to_vec();
    }
}

/// PresetBundle.cpp:1778-1790 `GetNozzleVolumeTypeStr` — untranslated name of a volume type, as
/// BambuStudio.conf and the per-slot snapshot store it ("High Flow").
fn get_nozzle_volume_type_str(type_: NozzleVolumeType) -> String {
    match type_ {
        NVT_STANDARD => "Standard".to_string(),
        NVT_HIGH_FLOW => "High Flow".to_string(),
        NVT_TPU_HIGH_FLOW => "TPU High Flow".to_string(),
        NVT_TPU285 => "TPU285".to_string(),
        NVT_E3D_HIGH_FLOW => "E3D High Flow".to_string(),
        NVT_HYBRID => "Hybrid".to_string(),
        _ => String::new(),
    }
}

/// PresetBundle.cpp:1822-1843
fn per_slot_nozzle_options_consistent(config: &NozzlePrintConfig) -> bool {
    let counts: &[i32] = config.extruder_nozzle_count.as_deref().unwrap_or(&[]);
    let vtypes: &[i32] = config.extruder_nozzle_volume_type.as_deref().unwrap_or(&[]);
    let diams: &[f64] = config.extruder_nozzle_diameter.as_deref().unwrap_or(&[]);
    let slots: &[i32] = config.extruder_nozzle_slot.as_deref().unwrap_or(&[]);

    let mut total: i32 = 0;
    for &c in counts {
        if c < 0 {
            return false;
        }
        total = total.wrapping_add(c);
    }
    total > 0 && vtypes.len() as i32 == total && diams.len() as i32 == total && slots.len() as i32 == total
}

/// PresetBundle.cpp:1845-1870
fn sync_per_slot_nozzle_options_from_full_stat(config: &mut NozzlePrintConfig, full_stat: &ExtruderFullStat) {
    let raw = full_stat.get_raw_stat();
    let mut nozzle_counts: Vec<i32> = Vec::with_capacity(raw.len());
    let mut nozzle_volume_types: Vec<i32> = Vec::new();
    let mut nozzle_diameters: Vec<f64> = Vec::new();
    let mut nozzle_slots: Vec<i32> = Vec::new();
    let mut global_slot = 0;
    for ext in raw {
        let mut installed = 0;
        for slot in ext {
            if !slot.1.is_empty() {
                nozzle_volume_types.push(nozzle_volume_type_from_string(&slot.1).0);
                nozzle_diameters.push(if slot.0.is_empty() { 0.0 } else { string_to_double_decimal_point(&slot.0).0 });
                nozzle_slots.push(global_slot);
                installed += 1;
            }
            global_slot += 1;
        }
        nozzle_counts.push(installed);
    }
    config.extruder_nozzle_count = Some(nozzle_counts);
    config.extruder_nozzle_volume_type = Some(nozzle_volume_types);
    config.extruder_nozzle_diameter = Some(nozzle_diameters);
    config.extruder_nozzle_slot = Some(nozzle_slots);
}

#[cfg(test)]
mod tests {
    use super::*;

    fn slot(diameter: &str, volume_type: NozzleVolumeType) -> NozzleSlot {
        NozzleSlot { diameter: diameter.to_string(), volume_type, status: NozzleSlotStatus::Normal }
    }

    // NozzleInventory.cpp:278-292, 314-341: the two directions of a history value.
    #[test]
    fn history_round_trip() {
        let extruders = vec![vec![slot("0.4", NVT_HIGH_FLOW), NozzleSlot::default()], vec![slot("0.4", NVT_TPU285)]];
        assert_eq!(encode_history(&extruders), "High Flow|,TPU285");
        assert_eq!(
            decode_history("High Flow|,TPU285"),
            vec![vec![Some(NVT_HIGH_FLOW), None], vec![Some(NVT_TPU285)]]
        );
        assert!(decode_history("High Flow,Bogus").is_empty());
        assert!(decode_history("").is_empty());
    }

    // NozzleInventory.cpp:356-409: the two directions of a nozzle_slots value.
    #[test]
    fn slot_history_round_trip() {
        let extruders = vec![vec![slot("0.4", NVT_STANDARD), NozzleSlot::default(), slot("0.6", NVT_E3D_HIGH_FLOW)]];
        let encoded = encode_slot_history(&extruders);
        assert_eq!(encoded, "0.4 Standard||0.6 E3D High Flow");
        assert_eq!(decode_slot_history(&encoded), extruders);
        // A token without a diameter is an empty slot.
        assert_eq!(decode_slot_history("Standard"), vec![vec![NozzleSlot::default()]]);
        assert!(decode_slot_history("0.4 Bogus").is_empty());
    }

    // NozzleInventory.cpp:55-59, 116-126
    #[test]
    fn texts() {
        assert_eq!(source_name(NozzleSource::History), "History");
        assert_eq!(layer_text("user", None), "user=none");
        let layer = NozzleLayer {
            source: NozzleSource::User,
            model: "M".to_string(),
            extruders: vec![vec![slot("0.4", NVT_STANDARD), NozzleSlot::default()], vec![]],
            stamp: 3,
        };
        assert_eq!(layer_text("user", Some(&layer)), "user#3(M)=0.4 Standard|-,");
    }
}
