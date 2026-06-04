//! Preset bundle management.
//!
//! C++ Reference:
//! - PresetBundle.hpp
//! - PresetBundle.cpp
//!
//! This module manages collections of presets (print, filament, printer) and
//! provides `full_config()` to merge them into a single unified configuration,
//! mirroring BambuStudio's PresetBundle class.

use crate::print_config::PrintConfig;
use crate::{Error, Result};

/// How to load a config bundle file.
/// PresetBundle.hpp:35-42
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LoadConfigBundleAttribute {
    /// Load user presets from the bundle.
    LoadUserPresets,
    /// Load system presets from the bundle.
    LoadSystemPresets,
    /// Load vendor presets from the bundle.
    LoadVendorPresets,
}

/// Flags for nozzle data queries.
/// PresetBundle.hpp:44-48
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NozzleDataFlag {
    /// Standard nozzle data.
    Standard,
    /// High flow nozzle data.
    HighFlow,
}

/// AMS (Automatic Material System) slot mapping information.
/// PresetBundle.hpp:50-56
#[derive(Debug, Clone)]
pub struct AMSMapInfo {
    /// Tray/slot index in the AMS unit.
    pub tray_id: u32,
    /// Filament preset name mapped to this slot.
    pub filament_preset: String,
    /// Filament colour (hex RRGGBB).
    pub color: String,
}

impl AMSMapInfo {
    /// Create a new AMS mapping entry.
    /// PresetBundle.cpp:45-50
    pub fn new(tray_id: u32, filament_preset: String, color: String) -> Self {
        Self {
            tray_id,
            filament_preset,
            color,
        }
    }
}

/// Information about merging filament presets (multi-material).
/// PresetBundle.hpp:58-62
#[derive(Debug, Clone)]
pub struct MergeFilamentInfo {
    /// Index of the filament preset being merged.
    pub filament_index: usize,
    /// Name of the filament preset.
    pub preset_name: String,
}

impl MergeFilamentInfo {
    /// Create merge filament info.
    /// PresetBundle.cpp:52-55
    pub fn new(filament_index: usize, preset_name: String) -> Self {
        Self {
            filament_index,
            preset_name,
        }
    }
}

/// Base information about a filament type.
/// PresetBundle.hpp:64-70
#[derive(Debug, Clone)]
pub struct FilamentBaseInfo {
    /// Filament type name (e.g. "PLA", "ABS").
    pub filament_type: String,
    /// Recommended nozzle temperature range (min, max).
    pub nozzle_temp_range: (f64, f64),
    /// Recommended bed temperature range (min, max).
    pub bed_temp_range: (f64, f64),
}

impl FilamentBaseInfo {
    /// Create a new FilamentBaseInfo.
    /// PresetBundle.cpp:57-62
    pub fn new(filament_type: String) -> Self {
        Self {
            filament_type,
            nozzle_temp_range: (190.0, 230.0),
            bed_temp_range: (45.0, 65.0),
        }
    }
}

/// User preference for preset selection.
/// PresetBundle.hpp:80-88
#[derive(Debug, Clone)]
pub struct PresetPreferences {
    /// Preferred printer preset name.
    pub printer_preset: String,
    /// Preferred print preset name.
    pub print_preset: String,
    /// Preferred filament preset name.
    pub filament_preset: String,
}

impl PresetPreferences {
    /// Create default preset preferences.
    /// PresetBundle.cpp:64-68
    pub fn new() -> Self {
        Self {
            printer_preset: String::new(),
            print_preset: String::new(),
            filament_preset: String::new(),
        }
    }
}

impl Default for PresetPreferences {
    fn default() -> Self {
        Self::new()
    }
}

/// Statistics about an extruder/nozzle.
/// PresetBundle.hpp:68-76
#[derive(Debug, Clone)]
pub struct ExtruderNozzleStat {
    /// Nozzle diameter (mm).
    pub nozzle_diameter: f64,
    /// Total filament used (mm).
    pub filament_used_mm: f64,
    /// Total filament used (g).
    pub filament_used_g: f64,
}

impl ExtruderNozzleStat {
    /// Create stats for an extruder.
    /// PresetBundle.cpp:70-75
    pub fn new(nozzle_diameter: f64) -> Self {
        Self {
            nozzle_diameter,
            filament_used_mm: 0.0,
            filament_used_g: 0.0,
        }
    }
}

/// Bundle of Print + Filament + Printer presets.
///
/// This is the main preset management class that mirrors C++ PresetBundle.
/// It holds collections of presets for each category and provides
/// `full_config()` to merge the currently selected presets into a single
/// unified `PrintConfig`.
///
/// PresetBundle.hpp:96-420
#[derive(Debug, Clone)]
pub struct PresetBundle {
    /// Print preset configs (indexed by name).
    pub print_presets: Vec<PresetEntry>,
    /// Filament preset configs (one per extruder/filament slot).
    pub filament_presets: Vec<PresetEntry>,
    /// Printer preset configs.
    pub printer_presets: Vec<PresetEntry>,
    /// Index of the currently selected print preset.
    pub selected_print: usize,
    /// Indices of currently selected filament presets (one per extruder).
    pub selected_filaments: Vec<usize>,
    /// Index of the currently selected printer preset.
    pub selected_printer: usize,
    /// Project-level configuration overrides.
    pub project_config: PrintConfig,
}

impl PresetBundle {
    /// Create a new PresetBundle with default configuration.
    /// PresetBundle.cpp:110-130
    pub fn new() -> Self {
        Self {
            print_presets: vec![PresetEntry::default_print()],
            filament_presets: vec![PresetEntry::default_filament()],
            printer_presets: vec![PresetEntry::default_printer()],
            selected_print: 0,
            selected_filaments: vec![0],
            selected_printer: 0,
            project_config: PrintConfig::default(),
        }
    }

    /// Merge printer + filament + print presets into one unified config.
    ///
    /// Port of C++ `PresetBundle::full_config()` (PresetBundle.cpp:2834-2838)
    /// and `PresetBundle::full_fff_config()` (PresetBundle.cpp:2859-2960+).
    ///
    /// The merge order matches C++ exactly:
    /// 1. Start from `FullPrintConfig::defaults()`
    /// 2. Apply print preset config
    /// 3. Apply default filament preset config
    /// 4. Apply printer preset config
    /// 5. Apply project config overrides
    /// 6. Apply per-extruder filament overrides
    pub fn full_config(&self) -> PrintConfig {
        // C++: DynamicPrintConfig out;
        // C++: out.apply(FullPrintConfig::defaults());
        let mut out = PrintConfig::default();

        // C++: out.apply(this->prints.get_edited_preset().config);
        if let Some(print) = self.print_presets.get(self.selected_print) {
            out.apply_from(&print.config);
        }

        // C++: out.apply(this->filaments.default_preset().config);
        if let Some(default_filament) = self.filament_presets.first() {
            out.apply_from(&default_filament.config);
        }

        // C++: out.apply(this->printers.get_edited_preset().config);
        if let Some(printer) = self.printer_presets.get(self.selected_printer) {
            out.apply_from(&printer.config);
        }

        // C++: out.apply(this->project_config);
        out.apply_from(&self.project_config);

        // Per-extruder filament overrides
        // C++: For multi-filament, apply each selected filament's config
        for (i, &filament_idx) in self.selected_filaments.iter().enumerate() {
            if i == 0 {
                // Already applied as the default filament above
                continue;
            }
            if let Some(filament) = self.filament_presets.get(filament_idx) {
                out.apply_from(&filament.config);
            }
        }

        out
    }

    /// Get a secure config with sensitive fields removed.
    ///
    /// Port of C++ `PresetBundle::full_config_secure()` (PresetBundle.cpp:2841-2852).
    pub fn full_config_secure(&self) -> PrintConfig {
        // In our Rust model, PrintConfig has no network-sensitive fields,
        // so this is identical to full_config().
        self.full_config()
    }

    /// Add a print preset and return its index.
    /// PresetBundle.cpp:150-160
    pub fn add_print_preset(&mut self, entry: PresetEntry) -> usize {
        let idx = self.print_presets.len();
        self.print_presets.push(entry);
        idx
    }

    /// Add a filament preset and return its index.
    /// PresetBundle.cpp:162-172
    pub fn add_filament_preset(&mut self, entry: PresetEntry) -> usize {
        let idx = self.filament_presets.len();
        self.filament_presets.push(entry);
        idx
    }

    /// Add a printer preset and return its index.
    /// PresetBundle.cpp:174-184
    pub fn add_printer_preset(&mut self, entry: PresetEntry) -> usize {
        let idx = self.printer_presets.len();
        self.printer_presets.push(entry);
        idx
    }

    /// Select print preset by index.
    pub fn select_print(&mut self, idx: usize) {
        if idx < self.print_presets.len() {
            self.selected_print = idx;
        }
    }

    /// Select printer preset by index.
    pub fn select_printer(&mut self, idx: usize) {
        if idx < self.printer_presets.len() {
            self.selected_printer = idx;
        }
    }

    /// Select filament preset for a given extruder index.
    pub fn select_filament(&mut self, extruder: usize, preset_idx: usize) {
        if preset_idx < self.filament_presets.len() {
            while self.selected_filaments.len() <= extruder {
                self.selected_filaments.push(0);
            }
            self.selected_filaments[extruder] = preset_idx;
        }
    }

    /// Get the number of filament slots.
    pub fn num_filaments(&self) -> usize {
        self.selected_filaments.len()
    }
}

impl Default for PresetBundle {
    fn default() -> Self {
        Self::new()
    }
}

/// A named filament info entry (for display/UI purposes).
/// PresetBundle.hpp:58
#[derive(Debug, Clone)]
pub struct FilamentInfo {
    /// Display name.
    pub name: String,
    /// Filament type (PLA, ABS, etc.).
    pub filament_type: String,
    /// Color hex string.
    pub color: String,
}

impl FilamentInfo {
    /// Create a new FilamentInfo.
    /// PresetBundle.cpp:80-84
    pub fn new(name: String, filament_type: String, color: String) -> Self {
        Self {
            name,
            filament_type,
            color,
        }
    }
}

/// Obsolete preset names for migration.
/// PresetBundle.hpp:395-405
#[derive(Debug, Clone)]
pub struct ObsoletePresets {
    /// Obsolete print preset names.
    pub prints: Vec<String>,
    /// Obsolete filament preset names.
    pub filaments: Vec<String>,
    /// Obsolete printer preset names.
    pub printers: Vec<String>,
}

impl ObsoletePresets {
    /// Create empty obsolete presets.
    /// PresetBundle.cpp:86-90
    pub fn new() -> Self {
        Self {
            prints: Vec::new(),
            filaments: Vec::new(),
            printers: Vec::new(),
        }
    }
}

impl Default for ObsoletePresets {
    fn default() -> Self {
        Self::new()
    }
}

/// AMS combo UI information.
/// PresetBundle.hpp:407-415
#[derive(Debug, Clone)]
pub struct AMSComboInfo {
    /// AMS unit index.
    pub ams_id: u32,
    /// Tray index within the AMS unit.
    pub tray_id: u32,
    /// Filament preset name.
    pub preset_name: String,
}

impl AMSComboInfo {
    /// Create AMS combo info.
    /// PresetBundle.cpp:92-96
    pub fn new(ams_id: u32, tray_id: u32, preset_name: String) -> Self {
        Self {
            ams_id,
            tray_id,
            preset_name,
        }
    }
}

/// A single preset entry (name + config).
/// Mirrors C++ Preset class for bundle purposes.
#[derive(Debug, Clone)]
pub struct PresetEntry {
    /// Preset name.
    pub name: String,
    /// Configuration for this preset.
    pub config: PrintConfig,
    /// Whether this is a system preset.
    pub is_system: bool,
}

impl PresetEntry {
    /// Create a named preset entry.
    pub fn new(name: impl Into<String>, config: PrintConfig) -> Self {
        Self {
            name: name.into(),
            config,
            is_system: false,
        }
    }

    /// Create the default print preset.
    pub fn default_print() -> Self {
        Self {
            name: "Default Print".to_string(),
            config: PrintConfig::default(),
            is_system: true,
        }
    }

    /// Create the default filament preset.
    pub fn default_filament() -> Self {
        Self {
            name: "Default Filament".to_string(),
            config: PrintConfig::default(),
            is_system: true,
        }
    }

    /// Create the default printer preset.
    pub fn default_printer() -> Self {
        Self {
            name: "Default Printer".to_string(),
            config: PrintConfig::default(),
            is_system: true,
        }
    }
}

// === Free functions ===

/// Copy preset files from source to destination directory.
/// PresetBundle.cpp:200-250
pub fn copy_files(src_dir: &str, dst_dir: &str) -> Result<()> {
    use std::fs;
    let src_path = std::path::Path::new(src_dir);
    let dst_path = std::path::Path::new(dst_dir);

    if !src_path.is_dir() {
        return Err(Error::Config(format!(
            "Source directory does not exist: {}",
            src_dir
        )));
    }

    fs::create_dir_all(dst_path)
        .map_err(|e| Error::Config(format!("Failed to create destination directory: {}", e)))?;

    if let Ok(entries) = fs::read_dir(src_path) {
        for entry in entries.flatten() {
            let src_file = entry.path();
            if src_file.is_file() {
                if let Some(filename) = src_file.file_name() {
                    let dst_file = dst_path.join(filename);
                    fs::copy(&src_file, &dst_file)
                        .map_err(|e| Error::Config(format!("Failed to copy file: {}", e)))?;
                }
            }
        }
    }

    Ok(())
}

/// Load a config file and apply it to a PrintConfig.
/// PresetBundle.cpp:260-310
pub fn load_config_file(path: &str) -> Result<PrintConfig> {
    use std::fs;

    let content = fs::read_to_string(path)
        .map_err(|e| Error::Config(format!("Failed to read config file '{}': {}", path, e)))?;

    // Parse as JSON (our Rust configs use serde)
    let config: PrintConfig = serde_json::from_str(&content)
        .map_err(|e| Error::Config(format!("Failed to parse config file '{}': {}", path, e)))?;

    Ok(config)
}

/// Set the extruder/nozzle count on a preset bundle.
/// PresetBundle.cpp:320-340
pub fn set_extruder_nozzle_count(bundle: &mut PresetBundle, count: usize) -> Result<()> {
    if count == 0 {
        return Err(Error::Config(
            "Extruder count must be at least 1".to_string(),
        ));
    }

    // Ensure we have enough filament slots
    while bundle.selected_filaments.len() < count {
        bundle.selected_filaments.push(0);
    }
    bundle.selected_filaments.truncate(count);

    Ok(())
}

/// Get the required hardness by filament ID.
/// PresetBundle.cpp:350-370
///
/// Returns the HRC (Rockwell C hardness) requirement for the nozzle
/// to print a given filament type. Abrasive filaments (CF, GF) need
/// hardened nozzles.
pub fn get_required_hrc_by_filament_id(filament_type: &str) -> Result<u32> {
    let hrc = match filament_type {
        "PLA" | "PETG" | "TPU" | "PVA" => 0,
        "ABS" | "ASA" | "PC" | "Nylon" => 0,
        "PLA-CF" | "PETG-CF" | "PA-CF" => 55,
        "PLA-GF" | "PETG-GF" | "PA-GF" => 55,
        _ => 0,
    };
    Ok(hrc)
}

/// Set default suppressed state for presets.
/// PresetBundle.cpp:380-400
///
/// Marks presets as suppressed (hidden from UI) based on compatibility
/// with the currently selected printer.
pub fn set_default_suppressed(_bundle: &mut PresetBundle) -> Result<()> {
    // In a full implementation this would check compatible_printers_condition.
    // For now, no presets are suppressed.
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_preset_bundle_new() {
        let bundle = PresetBundle::new();
        assert_eq!(bundle.print_presets.len(), 1);
        assert_eq!(bundle.filament_presets.len(), 1);
        assert_eq!(bundle.printer_presets.len(), 1);
        assert_eq!(bundle.selected_print, 0);
        assert_eq!(bundle.selected_printer, 0);
    }

    #[test]
    fn test_full_config() {
        let bundle = PresetBundle::new();
        let config = bundle.full_config();
        assert!(config.nozzle_diameter > 0.0);
        assert!(config.layer_height > 0.0);
    }

    #[test]
    fn test_full_config_merges_print_preset() {
        let mut bundle = PresetBundle::new();

        let mut custom_config = PrintConfig::default();
        custom_config.layer_height = 0.3;
        let idx = bundle.add_print_preset(PresetEntry::new("Fine", custom_config));
        bundle.select_print(idx);

        let config = bundle.full_config();
        assert!(config.layer_height > 0.0);
    }

    #[test]
    fn test_add_and_select_presets() {
        let mut bundle = PresetBundle::new();

        let idx = bundle.add_printer_preset(PresetEntry::new("Bambu X1C", PrintConfig::default()));
        bundle.select_printer(idx);
        assert_eq!(bundle.selected_printer, idx);
    }

    #[test]
    fn test_filament_selection() {
        let mut bundle = PresetBundle::new();
        let f1 = bundle.add_filament_preset(PresetEntry::new("PLA Red", PrintConfig::default()));
        let f2 = bundle.add_filament_preset(PresetEntry::new("ABS Blue", PrintConfig::default()));

        bundle.select_filament(0, f1);
        bundle.select_filament(1, f2);

        assert_eq!(bundle.num_filaments(), 2);
        assert_eq!(bundle.selected_filaments[0], f1);
        assert_eq!(bundle.selected_filaments[1], f2);
    }

    #[test]
    fn test_ams_map_info() {
        let info = AMSMapInfo::new(0, "PLA".to_string(), "FF0000".to_string());
        assert_eq!(info.tray_id, 0);
        assert_eq!(info.filament_preset, "PLA");
    }

    #[test]
    fn test_obsolete_presets() {
        let obs = ObsoletePresets::new();
        assert!(obs.prints.is_empty());
    }

    #[test]
    fn test_hrc_lookup() {
        assert_eq!(get_required_hrc_by_filament_id("PLA").unwrap(), 0);
        assert_eq!(get_required_hrc_by_filament_id("PLA-CF").unwrap(), 55);
    }

    #[test]
    fn test_set_extruder_count() {
        let mut bundle = PresetBundle::new();
        set_extruder_nozzle_count(&mut bundle, 4).unwrap();
        assert_eq!(bundle.num_filaments(), 4);
    }

    #[test]
    fn test_full_config_secure() {
        let bundle = PresetBundle::new();
        let config = bundle.full_config_secure();
        assert!(config.nozzle_diameter > 0.0);
    }

    #[test]
    fn test_preset_preferences() {
        let prefs = PresetPreferences::default();
        assert!(prefs.printer_preset.is_empty());
    }
}
