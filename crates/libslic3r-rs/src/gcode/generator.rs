//! G-code generator.
//!
//! This module provides the GCode type representing generated G-code output,
//! mirroring BambuStudio's GCode class.

use crate::print_config::PrintConfig;
use crate::{Error, Result};
use std::fmt;
use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::Path;

/// Represents generated G-code output.
///
/// This is the result of the slicing process - a complete G-code file
/// that can be written to disk or sent to a printer.
#[derive(Clone, Default)]
pub struct GCode {
    /// The G-code content as a string.
    content: String,

    /// Statistics about the generated G-code.
    pub stats: GCodeStats,
}

impl GCode {
    // Create a new empty GCode.
    pub fn new() -> Self {
        Self {
            content: String::new(),
            stats: GCodeStats::default(),
        }
    }

    /// Create a GCode from content and existing stats.
    pub fn from_content_and_stats(content: String, stats: GCodeStats) -> Self {
        Self { content, stats }
    }

    /// Create a GCode from a string.
    pub fn from_string(content: String) -> Self {
        Self {
            content,
            stats: GCodeStats::default(),
        }
    }

    /// Create a GCode from a string with existing stats.
    pub fn from_string_with_stats(content: String, stats: GCodeStats) -> Self {
        Self { content, stats }
    }

    /// Get the G-code content as a string.
    pub fn content(&self) -> &str {
        &self.content
    }

    /// Get the G-code content as bytes.
    pub fn as_bytes(&self) -> &[u8] {
        self.content.as_bytes()
    }

    /// Get the length of the G-code content in bytes.
    pub fn len(&self) -> usize {
        self.content.len()
    }

    /// Check if the G-code is empty.
    pub fn is_empty(&self) -> bool {
        self.content.is_empty()
    }

    /// Append a line to the G-code.
    pub fn append_line(&mut self, line: &str) {
        self.content.push_str(line);
        self.content.push('\n');
    }

    /// Append raw content to the G-code.
    pub fn append(&mut self, content: &str) {
        self.content.push_str(content);
    }

    /// Append a comment to the G-code.
    pub fn append_comment(&mut self, comment: &str) {
        self.content.push_str("; ");
        self.content.push_str(comment);
        self.content.push('\n');
    }

    /// Write the G-code to a file.
    pub fn write_to_file<P: AsRef<Path>>(&self, path: P) -> Result<()> {
        let file = File::create(path).map_err(|e| Error::Io(e))?;
        let mut writer = BufWriter::new(file);
        writer
            .write_all(self.content.as_bytes())
            .map_err(|e| Error::Io(e))?;
        writer.flush().map_err(|e| Error::Io(e))?;
        Ok(())
    }

    /// Read G-code from a file.
    pub fn read_from_file<P: AsRef<Path>>(path: P) -> Result<Self> {
        let content = std::fs::read_to_string(path).map_err(|e| Error::Io(e))?;
        Ok(Self::from_string(content))
    }

    /// Get the number of lines in the G-code.
    pub fn line_count(&self) -> usize {
        self.content.lines().count()
    }

    /// Iterate over the lines of the G-code.
    pub fn lines(&self) -> impl Iterator<Item = &str> {
        self.content.lines()
    }

    /// Clear the G-code content.
    pub fn clear(&mut self) {
        self.content.clear();
        self.stats = GCodeStats::default();
    }
}

impl fmt::Debug for GCode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "GCode({} bytes, {} lines)",
            self.len(),
            self.line_count()
        )
    }
}

impl fmt::Display for GCode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.content)
    }
}

impl From<String> for GCode {
    fn from(content: String) -> Self {
        Self::from_string(content)
    }
}

impl From<GCode> for String {
    fn from(gcode: GCode) -> Self {
        gcode.content
    }
}

/// Statistics about generated G-code.
#[derive(Clone, Debug, Default)]
pub struct GCodeStats {
    /// Total number of layers.
    pub layer_count: usize,

    /// Total estimated print time (seconds).
    pub print_time_seconds: f64,

    /// Total filament length used (mm).
    pub filament_length_mm: f64,

    /// Total filament volume (cm³).
    pub filament_volume_cm3: f64,

    /// Total filament weight (g).
    pub filament_weight_g: f64,

    /// Filament density (g/cm³) — from stats/filament info, defaults to 0.
    /// This is separate from PrintConfig.filament_density; the header block
    /// uses this value (BambuStudio outputs 0 when density is not set in stats).
    pub filament_density: f64,

    /// Maximum Z height (mm).
    pub max_z_height: f64,

    /// Total travel distance (mm).
    pub travel_distance_mm: f64,

    /// Total extrusion distance (mm).
    pub extrusion_distance_mm: f64,

    /// Number of retractions.
    pub retraction_count: usize,

    /// Number of tool changes.
    pub tool_change_count: usize,
}

impl GCodeStats {
    // Create new empty statistics.
    pub fn new() -> Self {
        Self::default()
    }

    /// Calculate filament stats from extrusion length.
    ///
    /// Faithful port of BambuStudio's GCodeProcessor header back-fill
    /// (GCode/GCodeProcessor.cpp:697-734). In C++ the canonical accumulated
    /// quantity is the extruded *volume* per filament, in mm³:
    ///   volume_extruded_filament = area_filament_cross_section * delta_pos[E]
    ///   (GCodeProcessor.cpp:4007, area = PI * filament_radius²).
    /// Length and weight are then derived from that volume:
    ///   length = volume / (PI * sqr(0.5 * filament_diameter))   (cpp:731)
    ///   weight = volume * filament_density * 0.001               (cpp:705)
    /// The header field is labelled "[cm^3]" but BambuStudio actually writes
    /// the mm³ volume, so we keep mm³ here for byte parity.
    pub fn calculate_filament_stats(&mut self, config: &PrintConfig) {
        // Volume (mm³) = extruded length × cross-sectional area of the filament.
        let filament_radius = config.filament_diameter / 2.0;
        let cross_section_mm2 = std::f64::consts::PI * filament_radius * filament_radius;
        self.filament_volume_cm3 = self.filament_length_mm * cross_section_mm2;

        // Weight (g) = volume(mm³) * density(g/cm³) * 0.001  (cm³ per mm³).
        // GCodeProcessor.cpp:705 uses the filament density from the config,
        // not the (unset) per-stats density.
        self.filament_weight_g = self.filament_volume_cm3 * config.filament_density * 0.001;
    }

    /// Get print time formatted as HH:MM:SS.
    pub fn print_time_formatted(&self) -> String {
        let total_seconds = self.print_time_seconds as u64;
        let hours = total_seconds / 3600;
        let minutes = (total_seconds % 3600) / 60;
        let seconds = total_seconds % 60;
        format!("{:02}:{:02}:{:02}", hours, minutes, seconds)
    }

    /// Get filament used in meters.
    pub fn filament_used_meters(&self) -> f64 {
        self.filament_length_mm / 1000.0
    }
}

impl fmt::Display for GCodeStats {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "GCodeStats(layers={}, time={}, filament={:.2}m)",
            self.layer_count,
            self.print_time_formatted(),
            self.filament_used_meters()
        )
    }
}

/// Header information for BambuStudio-compatible G-code.
#[derive(Clone, Debug)]
pub struct GCodeHeader {
    /// Software name and version.
    pub software_version: String,

    /// Accel-aware estimated printing time for the "normal" time mode,
    /// formatted via `utils::get_time_dhms` (e.g. "43m 0s"). Computed by running
    /// the faithful `GCodeProcessor` over the assembled body; mirrors the C++
    /// `; estimated printing time (normal mode) = ...` header line.
    pub estimated_print_time: String,

    /// Statistics from the slicing process.
    pub stats: GCodeStats,

    /// Print configuration.
    pub config: PrintConfig,

    /// Raw BambuStudio settings for full CONFIG_BLOCK output.
    pub raw_settings: Option<serde_json::Value>,
}

impl GCodeHeader {
    // Create a new header from stats and config.
    pub fn new(stats: GCodeStats, config: PrintConfig) -> Self {
        Self::with_raw_settings(stats, config, None)
    }

    pub fn with_raw_settings(
        stats: GCodeStats,
        config: PrintConfig,
        raw_settings: Option<serde_json::Value>,
    ) -> Self {
        // Fall back to the crude (acceleration-free) stats time when no
        // accel-aware estimate has been computed yet (e.g. early header builds).
        let est = stats.print_time_seconds as f32;
        Self::with_estimated_time(stats, config, raw_settings, est)
    }

    /// Build a header with an explicit accel-aware estimated print time (s),
    /// computed by running the faithful `GCodeProcessor` over the body.
    pub fn with_estimated_time(
        stats: GCodeStats,
        config: PrintConfig,
        raw_settings: Option<serde_json::Value>,
        estimated_print_time_seconds: f32,
    ) -> Self {
        // GCode.cpp / GCodeProcessor: `; estimated printing time (normal mode) = `
        // is formatted with Utils get_time_dhms.
        let estimated_print_time = crate::utils::get_time_dhms(estimated_print_time_seconds);

        Self {
            // Match the BambuStudio version whose output we replicate byte-for-byte
            // (golden 3DBenchy_H2D_PLA.gcode header: "; BambuStudio 02.06.00.51").
            software_version: "BambuStudio 02.06.00.51".to_string(),
            estimated_print_time,
            stats,
            config,
            raw_settings,
        }
    }

    /// Generate the HEADER_BLOCK section.
    /// Format matches BambuStudio C++ output exactly.
    pub fn generate_header_block(&self) -> String {
        let mut header = String::new();

        header.push_str("; HEADER_BLOCK_START\n");
        header.push_str(&format!("; {}\n", self.software_version));
        // Native: `; estimated printing time (normal mode) = 43m 0s`
        // (GCode.cpp emits the accel-aware normal-mode time via get_time_dhms).
        header.push_str(&format!(
            "; estimated printing time (normal mode) = {}\n",
            self.estimated_print_time
        ));
        header.push_str(&format!(
            "; total layer number: {}\n",
            self.stats.layer_count
        ));
        header.push_str(&format!(
            "; total filament length [mm] : {:.2}\n",
            self.stats.filament_length_mm
        ));
        header.push_str(&format!(
            "; total filament volume [cm^3] : {:.2}\n",
            self.stats.filament_volume_cm3
        ));
        header.push_str(&format!(
            "; total filament weight [g] : {:.2}\n",
            self.stats.filament_weight_g
        ));
        header.push_str(&format!(
            "; filament_density: {:.2}\n",
            self.config.filament_density
        ));
        header.push_str(&format!(
            "; filament_diameter: {:.2}\n",
            self.config.filament_diameter
        ));
        header.push_str(&format!("; max_z_height: {:.2}\n", self.stats.max_z_height));
        // Configured filament count (single-material configs ⇒ 1, preserving
        // the locked default-path bytes). C++ writes the used-filament list;
        // matching that per-use accounting is part of the multicolour chain.
        header.push_str(&format!(
            "; filament: {}\n",
            self.config.num_filaments()
        ));
        header.push_str("; HEADER_BLOCK_END\n");
        header.push_str("\n");

        header
    }

    /// Generate the CONFIG_BLOCK section with all print settings.
    /// If raw_settings (from BambuStudio project_settings.config) are available,
    /// emit the full config block matching the reference output format.
    pub fn generate_config_block(&self) -> String {
        if let Some(ref settings) = self.raw_settings {
            if let Some(obj) = settings.as_object() {
                let mut config = String::new();
                config.push_str("; CONFIG_BLOCK_START\n");
                // BambuStudio iterates DynamicPrintConfig::keys(), which returns
                // keys in sorted (alphabetical) order, so we sort here too.
                // (GCode.cpp: append_full_config / GCode::process_layers)
                // Under CONFIG_FAITHFUL, add schema-default keys absent from
                // raw_settings so the block matches native's full_print_config
                // (every registered option). Merge + sort to keep ordering.
                let cf = std::env::var("CONFIG_FAITHFUL").is_ok();
                // Number of USED extruders. Native serializes per-extruder config
                // arrays (nozzle_temperature, retraction_distances_when_ec, …) at
                // this length — a single-material print uses 1 even on a 2-nozzle
                // H2D, so native emits "220" where rust emits "220,220". Derived
                // from the distinct extruder assignments in filament_map.
                let used_extruders: usize = obj
                    .get("filament_map")
                    .map(|v| {
                        let s = match v {
                            serde_json::Value::String(s) => s.clone(),
                            serde_json::Value::Array(a) => a
                                .iter()
                                .map(|x| {
                                    x.as_str().map(str::to_string).unwrap_or_else(|| x.to_string())
                                })
                                .collect::<Vec<_>>()
                                .join(","),
                            other => other.to_string(),
                        };
                        s.split(',')
                            .map(str::trim)
                            .filter(|x| !x.is_empty())
                            .collect::<std::collections::HashSet<_>>()
                            .len()
                            .max(1)
                    })
                    .unwrap_or(1);
                let mut keys: Vec<String> = obj.keys().cloned().collect();
                if cf {
                    for (k, _) in CONFIG_SCHEMA_DEFAULTS {
                        if !obj.contains_key(*k) {
                            keys.push((*k).to_string());
                        }
                    }
                }
                keys.sort();
                for key in &keys {
                    if is_config_key_skipped(key) {
                        continue;
                    }
                    if let Some(value) = obj.get(key) {
                        let formatted = format_config_value(key, value, used_extruders);
                        config.push_str(&format!("; {} = {}\n", key, formatted));
                    } else if cf {
                        if let Some((_, dv)) =
                            CONFIG_SCHEMA_DEFAULTS.iter().find(|(k, _)| k == key)
                        {
                            config.push_str(&format!("; {} = {}\n", key, dv));
                        }
                    }
                }
                config.push_str("; CONFIG_BLOCK_END\n\n");
                return config;
            }
        }
        self.generate_minimal_config_block()
    }

    /// Generate a minimal CONFIG_BLOCK if template is not available.
    fn generate_minimal_config_block(&self) -> String {
        let mut config = String::new();
        config.push_str("; CONFIG_BLOCK_START\n");

        let c = &self.config;
        config.push_str(&format!("; bed_temperature = {}\n", c.bed_temperature));
        config.push_str(&format!(
            "; bed_temperature_initial_layer = {}\n",
            c.first_layer_bed_temperature
        ));
        config.push_str(&format!("; brim_width = {:.1}\n", c.brim_width));
        config.push_str(&format!("; filament_density = {:.2}\n", c.filament_density));
        config.push_str(&format!(
            "; filament_diameter = {:.2}\n",
            c.filament_diameter
        ));
        config.push_str(&format!("; layer_height = {:.2}\n", c.layer_height));
        config.push_str(&format!(
            "; first_layer_height = {:.2}\n",
            c.first_layer_height
        ));
        config.push_str(&format!(
            "; nozzle_diameter = {:.2},{:.2}\n",
            c.nozzle_diameter, c.nozzle_diameter
        ));
        config.push_str(&format!(
            "; nozzle_temperature = {}\n",
            c.extruder_temperature
        ));
        config.push_str(&format!(
            "; nozzle_temperature_initial_layer = {}\n",
            c.first_layer_extruder_temperature
        ));
        config.push_str(&format!(
            "; travel_speed = {:.0},{:.0}\n",
            c.travel_speed, c.travel_speed
        ));
        config.push_str(&format!(
            "; initial_layer_speed = {:.0}\n",
            c.first_layer_speed
        ));
        config.push_str(&format!(
            "; retraction_length = {:.1},{:.1}\n",
            c.retract_length, c.retract_length
        ));
        config.push_str(&format!(
            "; retraction_speed = {:.0},{:.0}\n",
            c.retract_speed, c.retract_speed
        ));
        config.push_str(&format!(
            "; z_hop = {:.2},{:.2}\n",
            c.retract_lift, c.retract_lift
        ));

        config.push_str("; CONFIG_BLOCK_END\n\n");
        config
    }

    /// Generate the EXECUTABLE_BLOCK_START marker.
    pub fn generate_executable_block_start(&self) -> String {
        "; EXECUTABLE_BLOCK_START\n".to_string()
    }

    /// Generate the executable block content between EXECUTABLE_BLOCK_START
    /// and the first CHANGE_LAYER.  Matches the BambuStudio C++ output for
    /// Bambu Lab H2D + PLA Basic.
    pub fn generate_executable_block_content(&self) -> String {
        let mut s = String::new();
        let minutes = (self.stats.print_time_seconds / 60.0).round() as u64;
        let c = &self.config;

        // Progress estimate
        s.push_str(&format!("M73 P0 R{}\n", minutes));

        // Machine motion parameters — read from raw_settings if available
        if let Some(ref settings) = self.raw_settings {
            let get_first = |key: &str, default: &str| -> String {
                settings
                    .get(key)
                    .and_then(|v| match v {
                        serde_json::Value::Array(arr) => {
                            arr.first().and_then(|x| x.as_str()).map(|s| s.to_string())
                        }
                        serde_json::Value::String(s) => Some(s.clone()),
                        _ => None,
                    })
                    .unwrap_or_else(|| default.to_string())
            };
            s.push_str(&format!(
                "M201 X{} Y{} Z{} E{}\n",
                get_first("machine_max_acceleration_x", "20000"),
                get_first("machine_max_acceleration_y", "20000"),
                get_first("machine_max_acceleration_z", "500"),
                get_first("machine_max_acceleration_e", "5000")
            ));
            s.push_str(&format!(
                "M203 X{} Y{} Z{} E{}\n",
                get_first("machine_max_speed_x", "500"),
                get_first("machine_max_speed_y", "500"),
                get_first("machine_max_speed_z", "20"),
                get_first("machine_max_speed_e", "30")
            ));
            // M204: P=print accel (same as X), R=retract accel, T=travel accel (same as P)
            let print_accel = get_first("machine_max_acceleration_x", "20000");
            s.push_str(&format!(
                "M204 P{} R{} T{}\n",
                print_accel,
                get_first("machine_max_acceleration_retracting", "5000"),
                print_accel
            ));
            s.push_str(&format!(
                "M205 X{:.2} Y{:.2} Z{:.2} E{:.2}\n",
                get_first("machine_max_jerk_x", "9")
                    .parse::<f64>()
                    .unwrap_or(9.0),
                get_first("machine_max_jerk_y", "9")
                    .parse::<f64>()
                    .unwrap_or(9.0),
                get_first("machine_max_jerk_z", "3")
                    .parse::<f64>()
                    .unwrap_or(3.0),
                get_first("machine_max_jerk_e", "2.5")
                    .parse::<f64>()
                    .unwrap_or(2.5)
            ));
            // Fan off before machine_start_gcode (matches BambuStudio GCode.cpp preamble)
            // Only emit for printers with auxiliary cooling fans (X1C, P1S, etc.)
            // A1 mini doesn't have P2 aux fan, so skip for those printers
            if c.auxiliary_fan {
                s.push_str("M106 S0\n");
                s.push_str("M106 P2 S0\n");
            }
        } else {
            // Generic defaults
            s.push_str("M201 X20000 Y20000 Z500 E5000\n");
            s.push_str("M203 X1000 Y1000 Z30 E50\n");
            s.push_str("M204 P20000 R5000 T20000\n");
            s.push_str("M205 X9.00 Y9.00 Z3.00 E2.50\n");
            s.push_str(&format!(
                "M190 S{} ; set bed temperature and wait for it to be reached\n",
                c.cool_plate_temp
            ));
            s.push_str(&format!(
                "M104 S{} ; set nozzle temperature\n",
                c.extruder_temperature
            ));
        }

        // Machine start G-code — process template from settings if available
        s.push_str("; FEATURE: Custom\n");
        if let Some(ref settings) = self.raw_settings {
            if let Some(start_gcode) = settings.get("machine_start_gcode").and_then(|v| v.as_str())
            {
                let processed = process_gcode_template(start_gcode, settings, &self.config);
                // Trim trailing newlines to avoid blank line before MACHINE_START_GCODE_END
                s.push_str(processed.trim_end_matches('\n'));
                s.push('\n');
            }
            // Post-machine-start transition (matches BambuStudio GCode.cpp)
            s.push_str("; MACHINE_START_GCODE_END\n");
            // Filament start gcode
            if let Some(filament_start) = settings.get("filament_start_gcode").and_then(|v| match v
            {
                serde_json::Value::Array(a) => a.first().and_then(|x| x.as_str()),
                serde_json::Value::String(s) => Some(s.as_str()),
                _ => None,
            }) {
                if !filament_start.is_empty() {
                    // Don't add "; filament start gcode" — the template itself contains it
                    let processed = process_gcode_template(filament_start, settings, &self.config);
                    s.push_str(&processed);
                    if !processed.ends_with('\n') {
                        s.push('\n');
                    }
                }
            }
            // Post-filament-start positioning (matches BambuStudio GCode.cpp)
            s.push_str(";VT0\n");
            s.push_str("G90\n");
            s.push_str("G21\n");
            s.push_str("M83 ; use relative distances for extrusion\n");
            // Spaghetti detector
            s.push_str("M981 S1 P20000 ;open spaghetti detector\n");
        } else {
            s.push_str("G28 ; home all axes\n");
            s.push_str("G1 Z5 F5000 ; lift nozzle\n");
            s.push_str("; MACHINE_START_GCODE_END\n");
        }

        if self.raw_settings.is_none() {
            // Generic post-start (no BambuStudio settings)
            s.push_str("M104 T0 S0 N0 ;Multi extruder pre cooling\n");
            s.push_str(";VT0\n");
            s.push_str(&format!(
                "M109 S{} ; set nozzle temperature and wait for it to be reached\n",
                c.extruder_temperature
            ));
            s.push_str("G90\n");
            s.push_str("G21\n");
            s.push_str("M83 ; use relative distances for extrusion\n");
        }
        // When using BambuStudio settings, machine_start_gcode handles
        // all temperature/positioning. No extra commands needed.

        s
    }

    /// Generate the complete header (all blocks).
    pub fn generate_complete_header(&self) -> String {
        let mut header = String::new();
        header.push_str(&self.generate_header_block());
        header.push_str(&self.generate_config_block());
        header.push_str(&self.generate_executable_block_start());
        header.push_str(&self.generate_executable_block_content());
        header
    }
}

/// Format a JSON value for the CONFIG_BLOCK section.
///
/// Returns true if a config key must NOT be emitted into the CONFIG_BLOCK.
///
/// Faithful to `GCode::append_full_config` (BambuStudio src/libslic3r/GCode.cpp),
/// which iterates `print.full_print_config().keys()` and emits every option
/// except (a) a hard-coded set of host/credential `banned_keys`, and
/// (b) options that are `is_nil()`.
///
/// In C++ the iteration source is `full_print_config()` — the fully resolved
/// config that contains ONLY currently-defined option keys. Our `raw_settings`
/// instead carries the on-disk config, which still includes deprecated/renamed
/// or developer-only keys that BambuStudio drops during config normalization
/// (they are not added via `this->add(...)` in PrintConfig.cpp, or are
/// `comDevelop`-only, so they never appear in `full_print_config()`). We mirror
/// that filtering here so the emitted key set matches the native output.
/// R329: BambuStudio config-schema keys absent from rust's config but present
/// in native's CONFIG_BLOCK, with their PrintConfig.cpp STATIC defaults
/// (extracted from source, cross-verified == native output). Emitted under
/// CONFIG_FAITHFUL when the key is missing from raw_settings. Profile-derived
/// keys (where the profile overrides the default) are intentionally excluded.
const CONFIG_SCHEMA_DEFAULTS: &[(&str, &str)] = &[
    ("accel_to_decel_enable", "0"),
    ("accel_to_decel_factor", "50%"),
    ("apply_scarf_seam_on_circles", "1"),
    ("bed_custom_model", ""),
    ("bed_custom_texture", ""),
    ("before_layer_change_gcode", ""),
    ("bridge_angle", "0"),
    ("brim_type", "auto_brim"),
    ("default_filament_colour", "\"\""),
    ("default_jerk", "0"),
    ("detect_narrow_internal_solid_infill", "1"),
    ("embedding_wall_into_infill", "0"),
    ("enable_filament_dynamic_map", "0"),
    ("enable_mixed_color_sublayer", "0"),
    ("enable_overhang_bridge_fan", "1"),
    ("enable_pressure_advance", "0"),
    ("enforce_support_layers", "0"),
    ("ensure_vertical_shell_thickness", "enabled"),
    ("exclude_object", "1"),
    ("extruder_ams_count", "\"\""),
    ("filament_change_length_nc", "10"),
    ("filament_colour", "#00AE42"),
    ("filament_ids", "\"\""),
    ("filament_is_mixed", "0"),
    ("filament_mixed_components", "\"\""),
    ("filament_mixed_gradient", "0"),
    ("filament_mixed_gradient_range", "\"\""),
    ("filament_mixed_sublayer_ratios", "\"\""),
    ("filament_notes", ""),
    ("filter_out_gap_fill", "0"),
    ("first_layer_print_sequence", "0"),
    ("first_x_layer_fan_speed", "0"),
    ("first_x_layer_part_fan_speed", "0"),
    ("flush_into_infill", "0"),
    ("flush_into_objects", "0"),
    ("flush_into_support", "1"),
    ("flush_multiplier", "1"),
    ("gcode_add_line_number", "0"),
    ("has_filament_switcher", "0"),
    ("has_scarf_joint_seam", "0"),
    ("independent_support_layer_height", "1"),
    ("infill_jerk", "9"),
    ("initial_layer_flow_ratio", "1"),
    ("initial_layer_jerk", "9"),
    ("inner_wall_jerk", "9"),
    ("interlocking_beam", "0"),
    ("interlocking_beam_layer_count", "2"),
    ("interlocking_beam_width", "0.8"),
    ("interlocking_boundary_avoidance", "2"),
    ("interlocking_depth", "2"),
    ("interlocking_orientation", "22.5"),
    ("internal_solid_infill_pattern", "zig-zag"),
    ("ironing_direction", "45"),
    ("ironing_fan_speed", "-1"),
    ("ironing_pattern", "zig-zag"),
    ("is_infill_first", "0"),
    ("min_bead_width", "85%"),
    ("min_feature_size", "25%"),
    ("mmu_segmented_region_interlocking_depth", "0"),
    ("mmu_segmented_region_max_width", "0"),
    ("only_one_wall_first_layer", "0"),
    ("ooze_prevention", "0"),
    ("other_layers_print_sequence", "0"),
    ("other_layers_print_sequence_nums", "0"),
    ("outer_wall_jerk", "9"),
    ("post_process", "\"\""),
    ("precise_outer_wall", "0"),
    ("precise_z_height", "0"),
    ("pressure_advance", "0.02"),
    ("prime_tower_extra_rib_length", "0"),
    ("prime_tower_fillet_wall", "1"),
    ("prime_tower_infill_gap", "150%"),
    ("prime_tower_rib_wall", "1"),
    ("prime_tower_rib_width", "8"),
    ("prime_tower_skip_points", "1"),
    ("prime_volume_mode", "Default"),
    ("print_flow_ratio", "1"),
    ("printer_model", ""),
    ("printer_notes", ""),
    ("printing_by_object_gcode", ""),
    ("process_notes", ""),
    ("raft_contact_distance", "0.1"),
    ("raft_expansion", "1.5"),
    ("raft_first_layer_density", "90%"),
    ("raft_first_layer_expansion", "-1"),
    ("role_base_wipe_speed", "1"),
    ("seam_gap", "15%"),
    ("seam_slope_conditional", "1"),
    ("seam_slope_entire_loop", "0"),
    ("seam_slope_inner_walls", "1"),
    ("seam_slope_steps", "10"),
    ("slice_closing_radius", "0.049"),
    ("slicing_mode", "regular"),
    ("smooth_speed_discontinuity_area", "1"),
    ("sparse_infill_anchor", "400%"),
    ("sparse_infill_anchor_max", "20"),
    ("spiral_mode_max_xy_smoothing", "200%"),
    ("spiral_mode_smooth", "0"),
    ("start_end_points", "30x-3,54x245"),
    ("support_angle", "0"),
    ("support_bottom_interface_spacing", "0.5"),
    ("support_critical_regions_only", "0"),
    ("support_interface_not_for_body", "1"),
    ("support_object_first_layer_gap", "0.2"),
    ("support_remove_small_overhang", "1"),
    ("template_custom_gcode", ""),
    ("thick_bridges", "0"),
    ("thumbnail_size", "50x50"),
    ("timelapse_type", "0"),
    ("top_area_threshold", "200%"),
    ("top_one_wall_type", "all top"),
    ("top_surface_jerk", "9"),
    ("top_z_overrides_xy_distance", "0"),
    ("travel_jerk", "9"),
    ("tree_support_branch_diameter_angle", "5"),
    ("tree_support_branch_distance", "5"),
    ("use_firmware_retraction", "0"),
    ("use_relative_e_distances", "1"),
    ("wall_distribution_count", "1"),
    ("wall_sequence", "inner wall/outer wall"),
    ("wall_transition_angle", "10"),
    ("wall_transition_filter_deviation", "25%"),
    ("wall_transition_length", "100%"),
    ("wipe_speed", "80%"),
    ("wipe_tower_rotation_angle", "0"),
    ("wipe_tower_x", "15"),
    ("wipe_tower_y", "220"),
    ("wrapping_detection_layers", "20"),
];

fn is_config_key_skipped(key: &str) -> bool {
    // GCode.cpp append_full_config banned_keys (host / credential settings).
    const BANNED_KEYS: &[&str] = &[
        "compatible_printers",
        "compatible_prints",
        "print_host",
        "print_host_webui",
        "printhost_apikey",
        "printhost_cafile",
        "printhost_user",
        "printhost_password",
        "printhost_port",
    ];
    // Deprecated / renamed / developer-only keys that are absent from
    // full_print_config() (not registered via this->add(...) in
    // PrintConfig.cpp, or commented out / comDevelop), and therefore never
    // emitted by native BambuStudio.
    const DROPPED_KEYS: &[&str] = &[
        "adaptive_layer_height",
        "deretract_speed_extruder_change",
        "extruder_clearance_radius",
        "extruder_height_gap",
        "filament_deretraction_speed",
        "filament_id",
        "filament_long_retractions_when_cut",
        "filament_long_retractions_when_ec",
        "filament_printable",
        "filament_retract_before_wipe",
        "filament_retract_restart_extra",
        "filament_retract_when_changing_layer",
        "filament_retraction_distances_when_cut",
        "filament_retraction_distances_when_ec",
        "filament_retraction_minimum_travel",
        "filament_retraction_speed",
        "filament_z_hop",
        // Runtime template variables injected into raw_settings by print.rs
        // (first-layer bounding box for the start-gcode G29 calls). They are
        // not config options in full_print_config(), so native BambuStudio
        // never emits them as CONFIG_BLOCK keys.
        "first_layer_print_min",
        "first_layer_print_size",
        "layer_time_smoothing",
        "layer_time_smoothing_threshold",
        "only_one_wall_top",
        "reduce_infill_retraction",
        "wall_infill_order",
    ];
    BANNED_KEYS.contains(&key) || DROPPED_KEYS.contains(&key)
}

/// BambuStudio's CONFIG_BLOCK uses different formatting depending on the
/// C++ config option type. Since we don't have that type info, we use
/// key-name heuristics:
///
/// - Per-extruder numeric arrays (most keys): first element only
/// - Geometry arrays (printable_area, bed_exclude_area, etc.): comma-joined
/// - machine_max_* arrays (4 elements): first 2 comma-joined
/// - String list arrays (gcode, compatible_printers, etc.): semicolon-joined with quotes
/// - Quoted string arrays (settings_id, vendor, etc.): quoted first element
/// - Scalar strings: literal with \n escaping
fn format_config_value(key: &str, value: &serde_json::Value, used_extruders: usize) -> String {
    // Under CONFIG_FAITHFUL, native coFloat drops a trailing ".0" (12.0->12,
    // 1.0->1) and coPercent appends '%'. Apply to scalar string values.
    let cf = std::env::var("CONFIG_FAITHFUL").is_ok();
    // coPercent keys: native ConfigOptionPercent::serialize always appends '%'.
    // rust stores the bare number for some profile-resolved values.
    const PERCENT_KEYS: &[&str] =
        &["bottom_surface_density", "top_surface_density", "monotonic_travel_into_wall"];
    let normalize_scalar = |raw: &str| -> String {
        let mut out = raw.replace('\n', "\\n");
        if cf {
            // coPoint (best_object_pos): native ConfigOptionPoint::serialize emits
            // "X,Y"; BBL's profile JSON stores the point as "XxY".
            if key == "best_object_pos" {
                out = out.replace('x', ",");
            }
            if out.ends_with(".0") && out[..out.len() - 2].chars().all(|c| c.is_ascii_digit() || c == '-') && !out[..out.len()-2].is_empty() {
                out.truncate(out.len() - 2);
            }
            if PERCENT_KEYS.contains(&key) && !out.ends_with('%') {
                out.push('%');
            }
        }
        out
    };
    match value {
        serde_json::Value::String(s) => normalize_scalar(s),
        serde_json::Value::Array(arr) if arr.is_empty() => String::new(),
        serde_json::Value::Array(arr) => {
            // coPointsGroups (PrintConfig.cpp: extruder_printable_area):
            // groups joined by '#', each group is its element string as-is
            // (already comma-joined "AxB,CxD,..."). Mirrors Config.hpp
            // ConfigOptionPointsGroups::serialize, which emits '#' between groups.
            if key == "extruder_printable_area" {
                return arr
                    .iter()
                    .map(|v| v.as_str().unwrap_or("").to_string())
                    .collect::<Vec<_>>()
                    .join("#");
            }

            // Comma-joined geometry/point arrays
            const COMMA_JOIN_KEYS: &[&str] = &[
                "bed_exclude_area",
                "printable_area",
                "start_end_points",
                "flush_volumes_vector",
                "flush_volumes_matrix",
                "filament_dev_ams_drying_temperature",
                "filament_dev_ams_drying_time",
                "filament_self_index",
                // extruder_printable_height is coFloats -> comma-joined list.
                "extruder_printable_height",
            ];
            if COMMA_JOIN_KEYS.contains(&key) {
                return arr
                    .iter()
                    .map(|v| v.as_str().unwrap_or("").to_string())
                    .collect::<Vec<_>>()
                    .join(",");
            }

            // machine_max_* arrays: legacy emits 2-value Marlin format; native
            // (H2D) emits the FULL resolved array. Under CONFIG_FAITHFUL emit full.
            if key.starts_with("machine_max_") && arr.len() >= 2 {
                let take_n = if std::env::var("CONFIG_FAITHFUL").is_ok() { arr.len() } else { 2 };
                return arr
                    .iter()
                    .take(take_n)
                    .map(|v| v.as_str().unwrap_or("").to_string())
                    .collect::<Vec<_>>()
                    .join(",");
            }

            // Semicolon-joined string list arrays (gcode, compatible printers, etc.)
            const SEMICOLON_KEYS: &[&str] = &[
                "filament_start_gcode",
                "filament_end_gcode",
                "print_compatible_printers",
                "upward_compatible_machine",
                "extruder_ams_count",
                "filament_dev_ams_drying_ams_limitations",
            ];
            if SEMICOLON_KEYS.contains(&key) {
                // Native coStrings serialize via escape_string_cstyle: simple
                // tokens (no space/quote/newline) are emitted UNQUOTED. rust
                // historically quoted every element (byte-locked default). Under
                // CONFIG_FAITHFUL, drop quotes for the numeric-token key that
                // native leaves bare (1;0), matching escape_string_cstyle.
                let unquoted = cf && key == "filament_dev_ams_drying_ams_limitations";
                return arr
                    .iter()
                    .map(|v| {
                        let s = v.as_str().unwrap_or("").replace('\n', "\\n");
                        if unquoted { s } else { format!("\"{}\"", s) }
                    })
                    .collect::<Vec<_>>()
                    .join(";");
            }

            // Quoted first-element arrays (settings IDs, vendor names, etc.)
            const QUOTED_KEYS: &[&str] = &[
                "default_filament_colour",
                "default_filament_profile",
                "extruder_variant_list",
                "filament_extruder_variant",
                "filament_settings_id",
                "filament_vendor",
                "print_extruder_variant",
                "printer_extruder_variant",
                "volumetric_speed_coefficients",
            ];
            if QUOTED_KEYS.contains(&key) {
                let s = arr.first().and_then(|v| v.as_str()).unwrap_or("");
                return format!("\"{}\"", s.replace('\n', "\\n"));
            }

            // Per-USED-extruder configs: native trims these to the used-extruder
            // count (1 for a single-material print) rather than emitting every
            // physical extruder. rust's resolved config carries all physical
            // extruders (e.g. "220,220" on a 2-nozzle H2D); take only the first
            // `used_extruders` to match native's "220". Gated CONFIG_FAITHFUL.
            const PER_USED_EXTRUDER_KEYS: &[&str] = &[
                "long_retractions_when_ec",
                "nozzle_temperature",
                "nozzle_temperature_initial_layer",
                "override_process_overhang_speed",
                "retraction_distances_when_ec",
                "slow_down_min_speed",
            ];
            if cf && arr.len() > used_extruders && PER_USED_EXTRUDER_KEYS.contains(&key) {
                return arr
                    .iter()
                    .take(used_extruders)
                    .map(|v| v.as_str().unwrap_or("").replace('\n', "\\n"))
                    .collect::<Vec<_>>()
                    .join(",");
            }

            // Default: native ConfigOptionFloats/Ints/Strings::serialize emit
            // the FULL vector comma-joined; the resolved config already carries
            // the full per-extruder/filament array. Rust historically emitted
            // first element only (byte-locked default). Under CONFIG_FAITHFUL
            // emit the full comma-joined array to match native's CONFIG_BLOCK.
            if std::env::var("CONFIG_FAITHFUL").is_ok()
                && arr.len() > 1
                && !key.starts_with("filament_")
            {
                // extruder_colour (coStrings) serializes with ';' (unquoted),
                // unlike the ',' used by coFloats/coInts.
                let sep = if key == "extruder_colour" { ";" } else { "," };
                return arr
                    .iter()
                    .map(|v| v.as_str().unwrap_or("").replace('\n', "\\n"))
                    .collect::<Vec<_>>()
                    .join(sep);
            }
            // Single-element (or first-only) fallback. Route through
            // normalize_scalar so the coFloat ".0"-strip applies to single-value
            // arrays too (filament_dev_chamber_drying_time: 12.0 -> 12).
            normalize_scalar(arr.first().and_then(|v| v.as_str()).unwrap_or(""))
        }
        _ => value.to_string(),
    }
}

/// Process BambuStudio gcode template, substituting variables.
/// Handles [var], {var}, {expr}, {if cond}...{endif} patterns.
pub fn process_gcode_template(
    template: &str,
    settings: &serde_json::Value,
    config: &PrintConfig,
) -> String {
    // Helper: resolve a variable name to a string value
    let resolve_var = |name: &str| -> Option<String> {
        // Handle indexed access like "nozzle_temperature[0]" or "var[initial_no_support_extruder]"
        if let Some(bracket_pos) = name.find('[') {
            let var_name = &name[..bracket_pos];
            let idx_str = name[bracket_pos + 1..].trim_end_matches(']');
            // Resolve index
            let idx: usize = if let Ok(n) = idx_str.parse::<usize>() {
                n
            } else {
                // Index is a variable name like "initial_no_support_extruder"
                settings
                    .get(idx_str)
                    .and_then(|v| v.as_str().or_else(|| v.as_u64().map(|_| "").and(None)))
                    .and_then(|s| s.parse::<usize>().ok())
                    .unwrap_or(0)
            };
            // Get array element — try settings first, then fall through to computed vars
            let from_settings = settings
                .get(var_name)
                .and_then(|v| v.as_array())
                .and_then(|arr| arr.get(idx))
                .and_then(|v| v.as_str())
                .map(|s| s.to_string());
            if from_settings.is_some() {
                return from_settings;
            }
            // Fall through to computed variables below (e.g. flush_volumetric_speeds[0])
        }
        // Simple variable lookup
        match settings.get(name) {
            Some(serde_json::Value::String(s)) => Some(s.clone()),
            Some(serde_json::Value::Array(arr)) => {
                arr.first().and_then(|v| v.as_str()).map(|s| s.to_string())
            }
            Some(serde_json::Value::Number(n)) => Some(n.to_string()),
            _ => {
                // Computed variables
                match name {
                    "initial_no_support_extruder" | "current_extruder" => Some("0".to_string()),
                    // R241 (gated): computed lists — single object, extruder 0
                    // job: first (non-support) filament = 0. Default path keeps
                    // these unresolved (byte-locked).
                    "first_non_support_filaments[0]"
                    | "first_non_support_filaments[1]"
                    | "first_filaments[0]"
                    | "first_filaments[1]"
                    | "first_non_support_filaments"
                    | "first_filaments"
                        if std::env::var("ZSMOOTH_FAITHFUL").is_ok() =>
                    {
                        Some("0".to_string())
                    }
                    // R241 (gated): overall_chamber_temperature = max over
                    // filaments' chamber temps (0 on this profile → M141 S0).
                    "overall_chamber_temperature"
                        if std::env::var("ZSMOOTH_FAITHFUL").is_ok() =>
                    {
                        let v = settings
                            .get("chamber_temperatures")
                            .or_else(|| settings.get("chamber_temperature"))
                            .and_then(|v| v.as_array())
                            .map(|arr| {
                                arr.iter()
                                    .filter_map(|x| {
                                        x.as_str().and_then(|s| s.parse::<i64>().ok())
                                    })
                                    .max()
                                    .unwrap_or(0)
                            })
                            .unwrap_or(0);
                        Some(v.to_string())
                    }
                    "bed_temperature_initial_layer_single" => {
                        // Resolve from curr_bed_type
                        let bed_type = settings
                            .get("curr_bed_type")
                            .and_then(|v| v.as_str())
                            .unwrap_or("");
                        let key = match bed_type {
                            "Cool Plate" => "cool_plate_temp_initial_layer",
                            "Engineering Plate" => "eng_plate_temp_initial_layer",
                            "Textured PEI Plate" => "textured_plate_temp_initial_layer",
                            _ => "hot_plate_temp_initial_layer",
                        };
                        settings
                            .get(key)
                            .and_then(|v| match v {
                                serde_json::Value::Array(a) => {
                                    a.first().and_then(|x| x.as_str()).map(|s| s.to_string())
                                }
                                serde_json::Value::String(s) => Some(s.clone()),
                                _ => None,
                            })
                            .or(Some("55".to_string()))
                    }
                    "outer_wall_volumetric_speed" => {
                        // BambuStudio computes volumetric speed using Flow's elliptical
                        // cross-section: mm3_per_mm = (w - h*(1-PI/4))*h + h²/4*PI
                        // Then: volumetric_speed = mm3_per_mm * speed * flow_ratio
                        // Clamped to filament_max_volumetric_speed
                        let speed = settings
                            .get("outer_wall_speed")
                            .and_then(|v| match v {
                                serde_json::Value::Array(a) => a.first().and_then(|x| x.as_str()),
                                serde_json::Value::String(s) => Some(s.as_str()),
                                _ => None,
                            })
                            .and_then(|s| s.parse::<f64>().ok())
                            .unwrap_or(200.0);
                        let width = settings
                            .get("outer_wall_line_width")
                            .and_then(|v| {
                                v.as_str().or_else(|| match v {
                                    serde_json::Value::Array(a) => {
                                        a.first().and_then(|x| x.as_str())
                                    }
                                    _ => None,
                                })
                            })
                            .and_then(|s| s.parse::<f64>().ok())
                            .unwrap_or(0.42);
                        let height = config.layer_height;
                        let _flow_ratio = settings
                            .get("filament_flow_ratio")
                            .and_then(|v| match v {
                                serde_json::Value::Array(a) => a.first().and_then(|x| x.as_str()),
                                serde_json::Value::String(s) => Some(s.as_str()),
                                _ => None,
                            })
                            .and_then(|s| s.parse::<f64>().ok())
                            .unwrap_or(1.0);
                        // Flow::mm3_per_mm from Flow.cpp:
                        // height * (width - height * (1 - PI/4))
                        let pi = std::f64::consts::PI;
                        let mm3_per_mm = height * (width - height * (1.0 - 0.25 * pi));
                        let vol_speed = mm3_per_mm * speed; // C++ doesn't apply flow_ratio here
                                                            // Clamp to filament_max_volumetric_speed
                        let max_vol = settings
                            .get("filament_max_volumetric_speed")
                            .and_then(|v| match v {
                                serde_json::Value::Array(a) => a.first().and_then(|x| x.as_str()),
                                serde_json::Value::String(s) => Some(s.as_str()),
                                _ => None,
                            })
                            .and_then(|s| s.parse::<f64>().ok())
                            .unwrap_or(f64::MAX);
                        let clamped = if max_vol > 0.0 {
                            vol_speed.min(max_vol)
                        } else {
                            vol_speed
                        };
                        Some(format!("{:.6}", clamped))
                    }
                    // flush_volumetric_speeds[N] = filament_max_volumetric_speed[N]
                    n if n.starts_with("flush_volumetric_speeds") => settings
                        .get("filament_max_volumetric_speed")
                        .and_then(|v| match v {
                            serde_json::Value::Array(a) => {
                                a.first().and_then(|x| x.as_str()).map(|s| s.to_string())
                            }
                            serde_json::Value::String(s) => Some(s.clone()),
                            _ => None,
                        })
                        .or(Some("21".to_string())),
                    // flush_temperatures[N] — from filament_flush_temp, fallback to
                    // nozzle_temperature_range_high (matches C++ GCode.cpp:856-866)
                    n if n.starts_with("flush_temperatures") => {
                        // Try filament_flush_temp first
                        let flush_temp = settings
                            .get("filament_flush_temp")
                            .and_then(|v| v.as_array())
                            .and_then(|a| a.first())
                            .and_then(|x| x.as_str())
                            .and_then(|s| s.parse::<u32>().ok())
                            .unwrap_or(0);
                        if flush_temp > 0 {
                            Some(format!("{}", flush_temp))
                        } else {
                            // Fallback to nozzle_temperature_range_high
                            settings
                                .get("nozzle_temperature_range_high")
                                .and_then(|v| match v {
                                    serde_json::Value::Array(a) => {
                                        a.first().and_then(|x| x.as_str()).map(|s| s.to_string())
                                    }
                                    serde_json::Value::String(s) => Some(s.clone()),
                                    _ => None,
                                })
                                .or_else(|| Some(format!("{}", config.extruder_temperature + 20)))
                        }
                    }
                    // bed_temperature[N] - resolve from plate type
                    n if n.starts_with("bed_temperature") && n.contains('[') => {
                        let bed_type = settings
                            .get("curr_bed_type")
                            .and_then(|v| v.as_str())
                            .unwrap_or("");
                        let key = match bed_type {
                            "Cool Plate" => "cool_plate_temp",
                            "Engineering Plate" => "eng_plate_temp",
                            "Textured PEI Plate" => "textured_plate_temp",
                            _ => "hot_plate_temp",
                        };
                        settings
                            .get(key)
                            .and_then(|v| match v {
                                serde_json::Value::Array(a) => {
                                    a.first().and_then(|x| x.as_str()).map(|s| s.to_string())
                                }
                                serde_json::Value::String(s) => Some(s.clone()),
                                _ => None,
                            })
                            .or(Some("55".to_string()))
                    }
                    // bed_temperature_initial_layer[N]
                    n if n.starts_with("bed_temperature_initial_layer") && n.contains('[') => {
                        let bed_type = settings
                            .get("curr_bed_type")
                            .and_then(|v| v.as_str())
                            .unwrap_or("");
                        let key = match bed_type {
                            "Cool Plate" => "cool_plate_temp_initial_layer",
                            "Engineering Plate" => "eng_plate_temp_initial_layer",
                            "Textured PEI Plate" => "textured_plate_temp_initial_layer",
                            _ => "hot_plate_temp_initial_layer",
                        };
                        settings
                            .get(key)
                            .and_then(|v| match v {
                                serde_json::Value::Array(a) => {
                                    a.first().and_then(|x| x.as_str()).map(|s| s.to_string())
                                }
                                serde_json::Value::String(s) => Some(s.clone()),
                                _ => None,
                            })
                            .or(Some("55".to_string()))
                    }
                    // first_layer_print_min/size — handled via settings JSON injection
                    // (injected by print.rs from mesh bounding box before template processing)
                    // Fall through to settings lookup which handles array indexing

                    // min_vitrification_temperature — min of temperature_vitrification across filaments
                    // C++: computed in GCode.cpp from filament vitrification temps
                    "min_vitrification_temperature" => settings
                        .get("temperature_vitrification")
                        .and_then(|v| v.as_array())
                        .map(|arr| {
                            arr.iter()
                                .filter_map(|x| x.as_str().and_then(|s| s.parse::<f64>().ok()))
                                .fold(f64::MAX, f64::min)
                        })
                        .map(|v| {
                            if v == f64::MAX {
                                "0".to_string()
                            } else {
                                format!("{}", v as i32)
                            }
                        }),

                    // is_all_bbl_filament — true if all filaments are from Bambu Lab
                    "is_all_bbl_filament" => {
                        let all_bbl = settings
                            .get("filament_vendor")
                            .and_then(|v| v.as_array())
                            .map(|arr| {
                                arr.iter()
                                    .all(|x| x.as_str().map_or(true, |s| s.contains("Bambu")))
                            })
                            .unwrap_or(true);
                        Some(if all_bbl {
                            "1".to_string()
                        } else {
                            "0".to_string()
                        })
                    }

                    _ => None,
                }
            }
        }
    };

    // Process template line by line, handling escape sequences
    let unescaped = template.replace("\\n", "\n");
    // R242 (gated): native PlaceholderParser is a char-stream — {if} conditions
    // may span MULTIPLE physical lines (the H2D start template's
    // `{if (filament_type[..] == "PLA") ||\n (filament_type0 == "PLA-CF") ...}`)
    // and `{if(` appears without a space. Preprocess: normalize `{if(`→`{if (`,
    // and join any `{if`-opening line whose braces don't balance with the
    // following lines until they do.
    static TMPL_FAITHFUL: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    let tmpl_faithful = *TMPL_FAITHFUL.get_or_init(|| std::env::var("ZSMOOTH_FAITHFUL").is_ok());
    let unescaped = if tmpl_faithful {
        let normalized = unescaped.replace("{if(", "{if (").replace("{elsif(", "{elsif (");
        let mut out = String::with_capacity(normalized.len());
        let mut lines = normalized.lines().peekable();
        while let Some(line) = lines.next() {
            let t = line.trim_start();
            if t.starts_with("{if ") || t.starts_with("{elsif") {
                let mut joined = line.to_string();
                let balance = |s: &str| {
                    s.bytes().fold(0i32, |a, b| match b {
                        b'{' => a + 1,
                        b'}' => a - 1,
                        _ => a,
                    })
                };
                while balance(&joined) > 0 {
                    match lines.next() {
                        Some(next) => {
                            joined.push(' ');
                            joined.push_str(next.trim_start());
                        }
                        None => break,
                    }
                }
                out.push_str(&joined);
            } else {
                out.push_str(line);
            }
            out.push('\n');
        }
        out
    } else {
        unescaped
    };
    let mut output = String::new();
    let mut skip_depth = 0u32; // for {if}/{endif} nesting
                               // Track whether current {if} group has had a true branch.
                               // Stack of (nesting_level, branch_taken) — branch_taken=true means skip all remaining elsif/else
    let mut branch_satisfied: Vec<bool> = Vec::new();

    for line in unescaped.lines() {
        let trimmed = line.trim();

        // Handle {if condition} — supports both line-level and inline patterns
        if trimmed.starts_with("{if ") {
            if let Some(close_pos) = trimmed[4..].find('}') {
                let cond = &trimmed[4..4 + close_pos];
                let after_cond = &trimmed[4 + close_pos + 1..];
                if skip_depth > 0 {
                    // Already skipping outer block — increase nesting
                    skip_depth += 1;
                    branch_satisfied.push(false);
                } else {
                    let result = eval_condition(cond, &resolve_var);
                    if result {
                        branch_satisfied.push(true); // branch taken
                                                     // Emit indent + text after {if cond} (C++ emits trailing content)
                        if let Some(pos) = line.find("{if ") {
                            output.push_str(&line[..pos]);
                        }
                        if !after_cond.is_empty() {
                            output.push_str(after_cond);
                        }
                        output.push('\n');
                    } else {
                        skip_depth += 1;
                        branch_satisfied.push(false); // no branch taken yet
                    }
                }
                continue;
            }
        }
        if trimmed.starts_with("{endif}") {
            let _was_satisfied = branch_satisfied.pop().unwrap_or(false);
            if skip_depth > 0 {
                skip_depth -= 1;
            }
            // BambuStudio emits the line with {endif} removed, preserving
            // surrounding text and indentation. A bare {endif} produces a blank line.
            if skip_depth == 0 {
                if let Some(pos) = line.find("{endif}") {
                    let before = &line[..pos];
                    let after = &line[pos + 7..];
                    output.push_str(before);
                    output.push_str(after);
                    output.push('\n');
                }
            }
            continue;
        }
        // Handle {elsif(condition)} or {elsif condition}
        if trimmed.starts_with("{elsif") {
            let satisfied = branch_satisfied.last().copied().unwrap_or(false);
            if skip_depth > 1 {
                // Nested inside a skipped outer block — don't change state
            } else if satisfied {
                // A previous branch in this {if} group already matched → skip
                if skip_depth == 0 {
                    skip_depth = 1;
                }
            } else if skip_depth == 1 {
                // No branch matched yet → evaluate this condition
                let cond_start = if trimmed.starts_with("{elsif(") { 7 } else { 7 };
                if let Some(close_pos) = trimmed[cond_start..].find('}') {
                    let cond = &trimmed[cond_start..cond_start + close_pos];
                    let cond = cond.trim_start_matches('(').trim_end_matches(')');
                    let result = eval_condition(cond, &resolve_var);
                    if result {
                        skip_depth = 0;
                        if let Some(last) = branch_satisfied.last_mut() {
                            *last = true;
                        }
                        // Emit trailing text after {elsif cond} (C++ emits it)
                        let after = &trimmed[cond_start + close_pos + 1..];
                        if !after.is_empty() {
                            output.push_str(after);
                            output.push('\n');
                        }
                    }
                }
            }
            continue;
        }
        if trimmed == "{else}" {
            let satisfied = branch_satisfied.last().copied().unwrap_or(false);
            if skip_depth > 1 {
                // Nested in outer skip — don't change
            } else if satisfied {
                // Previous branch matched → skip else
                if skip_depth == 0 {
                    skip_depth = 1;
                }
            } else if skip_depth == 1 {
                // No branch matched → take else
                skip_depth = 0;
                if let Some(last) = branch_satisfied.last_mut() {
                    *last = true;
                }
            }
            // Emit blank line for {else} (inline replacement)
            if skip_depth == 0 {
                output.push('\n');
            }
            continue;
        }
        // Note: {endif} already handled above (with possible trailing comment)

        if skip_depth > 0 {
            continue;
        }

        // Substitute [var] outside {expr} and {expr} patterns in one pass
        let mut result = String::new();
        let processed = line.to_string();
        let mut chars = processed.chars().peekable();
        while let Some(ch) = chars.next() {
            if ch == '{' {
                let mut expr = String::new();
                let mut depth = 1;
                for c in chars.by_ref() {
                    if c == '{' {
                        depth += 1;
                    }
                    if c == '}' {
                        depth -= 1;
                        if depth == 0 {
                            break;
                        }
                    }
                    expr.push(c);
                }
                // Evaluate the expression (handles [var] internally via resolve_var)
                let val = eval_expr(&expr, &resolve_var);
                result.push_str(&val);
            } else if ch == '[' {
                // Substitute [var] outside of {expr}
                let mut var_name = String::new();
                for c in chars.by_ref() {
                    if c == ']' {
                        break;
                    }
                    var_name.push(c);
                }
                let replacement = resolve_var(&var_name).unwrap_or_default();
                result.push_str(&replacement);
            } else {
                result.push(ch);
            }
        }

        output.push_str(&result);
        output.push('\n');
    }

    output
}

/// Evaluate a simple condition like 'filament_type[0]=="PLA"' or 'default_acceleration > 0'
fn eval_condition(cond: &str, resolve: &dyn Fn(&str) -> Option<String>) -> bool {
    let cond = cond.trim();
    // R242 gate (checked early so the paren strip can use it): faithful mode
    // strips outer parens ONLY when they wrap the whole expression —
    // `((A) || (B)) && (C)` must NOT lose its outer chars (the naive strip
    // mangled it into `(A) || (B)) && (C`, mis-splitting at the inner ||).
    static COND_FAITHFUL_EARLY: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    let faithful_early =
        *COND_FAITHFUL_EARLY.get_or_init(|| std::env::var("ZSMOOTH_FAITHFUL").is_ok());
    // Strip outer parens
    let cond = if cond.starts_with('(') && cond.ends_with(')') {
        if faithful_early {
            let inner = &cond[1..cond.len() - 1];
            let mut depth = 0i32;
            let mut wraps = true;
            for b in inner.bytes() {
                match b {
                    b'(' => depth += 1,
                    b')' => {
                        depth -= 1;
                        if depth < 0 {
                            wraps = false;
                            break;
                        }
                    }
                    _ => {}
                }
            }
            if wraps && depth == 0 {
                inner
            } else {
                cond
            }
        } else {
            &cond[1..cond.len() - 1]
        }
    } else {
        cond
    };

    // R224 gate: the corrected {if} evaluator (paren-aware &&/|| splits +
    // `!` negation) matches native PlaceholderParser but flips branches the
    // old evaluator got wrong, so the default path keeps the legacy behavior
    // to preserve the byte-lock.
    static COND_FAITHFUL: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    let faithful = *COND_FAITHFUL.get_or_init(|| std::env::var("ZSMOOTH_FAITHFUL").is_ok());

    // R224: connective splits must ignore separators nested inside parens —
    // `!(a && b) && c` splits at the SECOND &&, not the one inside the group
    // (the naive find() split mis-parsed the timelapse template's
    // `!(has_timelapse_safe_pos && timelapse_type == 0)` guard).
    let find_top_level = |needle: &str| -> Option<usize> {
        let bytes = cond.as_bytes();
        let nb = needle.as_bytes();
        let mut depth = 0i32;
        let mut i = 0;
        while i + nb.len() <= bytes.len() {
            match bytes[i] {
                b'(' => depth += 1,
                b')' => depth -= 1,
                _ => {
                    if depth == 0 && &bytes[i..i + nb.len()] == nb {
                        return Some(i);
                    }
                }
            }
            i += 1;
        }
        None
    };
    if faithful {
        // Handle OR (||) — lower precedence than &&
        if let Some(pos) = find_top_level("||") {
            let left = &cond[..pos];
            let right = &cond[pos + 2..];
            return eval_condition(left, resolve) || eval_condition(right, resolve);
        }
        // Handle AND (&&)
        if let Some(pos) = find_top_level("&&") {
            let left = &cond[..pos];
            let right = &cond[pos + 2..];
            return eval_condition(left, resolve) && eval_condition(right, resolve);
        }
    } else {
        // Legacy (default-path) splits — byte-locked behavior.
        if let Some(pos) = cond.find(")||(") {
            let left = &cond[..pos];
            let right = &cond[pos + 4..];
            return eval_condition(left, resolve) || eval_condition(right, resolve);
        }
        if let Some(pos) = cond.find("||") {
            let left = &cond[..pos];
            let right = &cond[pos + 2..];
            return eval_condition(left, resolve) || eval_condition(right, resolve);
        }
        if let Some(pos) = cond.find("&&") {
            let left = &cond[..pos];
            let right = &cond[pos + 2..];
            return eval_condition(left, resolve) && eval_condition(right, resolve);
        }
    }

    // R224: logical negation — `!var` / `!(expr)`. Must be checked AFTER the
    // &&/|| splits so `!` binds tighter than the connectives (`!a && b` is
    // `(!a) && b`, not `!(a && b)` — the timelapse template's
    // `!spiral_mode && !(has_timelapse_safe_pos && ...)` guards rely on this).
    if faithful && cond.starts_with('!') && !cond.starts_with("!=") {
        return !eval_condition(&cond[1..], resolve);
    }

    // R241 (gated): templates write `var == "PLA"` with spaces — normalize
    // `== "` / `!= "` to the adjacency forms the legacy parser expects.
    let normalized;
    let cond = if faithful && (cond.contains("== \"") || cond.contains("!= \"")) {
        normalized = cond.replace("== \"", "==\"").replace("!= \"", "!=\"");
        normalized.as_str()
    } else {
        cond
    };

    // String equality: var=="value"
    if let Some(pos) = cond.find("==\"") {
        let lhs = cond[..pos].trim().trim_start_matches('(').trim();
        let rhs = cond[pos + 3..]
            .trim()
            .trim_end_matches('"')
            .trim_end_matches(')');
        let lhs_val = resolve(lhs).unwrap_or_default();
        return lhs_val == rhs;
    }
    // String inequality: var!="value"
    if let Some(pos) = cond.find("!=\"") {
        let lhs = cond[..pos].trim().trim_start_matches('(').trim();
        let rhs = cond[pos + 3..]
            .trim()
            .trim_end_matches('"')
            .trim_end_matches(')');
        let lhs_val = resolve(lhs).unwrap_or_default();
        return lhs_val != rhs;
    }

    // Numeric comparisons — check multi-char operators before single-char
    // Order matters: <= before <, >= before >
    let comparisons: &[(&str, fn(f64, f64) -> bool)] = &[
        ("<=", |a, b| a <= b),
        (">=", |a, b| a >= b),
        ("!=", |a, b| (a - b).abs() > f64::EPSILON),
        ("==", |a, b| (a - b).abs() < f64::EPSILON),
        ("<", |a, b| a < b),
        (">", |a, b| a > b),
    ];
    for &(op, cmp_fn) in comparisons {
        if let Some(pos) = cond.find(op) {
            let lhs = cond[..pos].trim().trim_start_matches('(').trim();
            let rhs = cond[pos + op.len()..].trim().trim_end_matches(')').trim();
            if faithful {
                // R240: comparison operands can be ARITHMETIC EXPRESSIONS
                // (the H2D end template's `{if (100.0 - max_layer_z/2) > 0}`)
                // — resolve variables inside, then eval_math. Plain variables
                // and literals still work through the same path.
                let num = |txt: &str| -> f64 {
                    // Comparison splitting leaves unbalanced parens on the
                    // operands (`(100.0 - max_layer_z/2` / `... /2)`).
                    let t = txt.trim().trim_start_matches('(').trim_end_matches(')').trim();
                    if let Ok(v) = t.parse::<f64>() {
                        return v;
                    }
                    if let Some(v) = resolve(t).and_then(|s| s.parse().ok()) {
                        return v;
                    }
                    let resolved = resolve_all_vars_in_expr(t, resolve);
                    eval_math(&resolved).unwrap_or(0.0)
                };
                return cmp_fn(num(lhs), num(rhs));
            }
            let lhs_val: f64 = resolve(lhs).and_then(|s| s.parse().ok()).unwrap_or(0.0);
            let rhs_val: f64 = rhs.parse().unwrap_or(0.0);
            return cmp_fn(lhs_val, rhs_val);
        }
    }

    // Truthy check
    let val = resolve(cond.trim()).unwrap_or_default();
    val != "0" && !val.is_empty() && val != "false"
}

/// Evaluate a simple expression like 'outer_wall_volumetric_speed/(0.3*0.5)*60'
fn eval_expr(expr: &str, resolve: &dyn Fn(&str) -> Option<String>) -> String {
    let trimmed = expr.trim();

    // R241 (gated): C++ PlaceholderParser supports ternaries and arbitrary
    // index expressions — `var[(a[0] != -1 ? a[0] : b[0])]` (H2D start
    // template M620.17). Handle a top-level `?:` by evaluating the condition
    // and recursing into the chosen branch, and an outer `name[<expr>]` by
    // evaluating the index expression first.
    static EXPR_FAITHFUL: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    let faithful = *EXPR_FAITHFUL.get_or_init(|| std::env::var("ZSMOOTH_FAITHFUL").is_ok());
    if faithful {
        // Strip outer parens that wrap the ENTIRE expression (`{(a ? b : c)}`)
        // so the ternary below is at depth 0.
        if trimmed.starts_with('(') && trimmed.ends_with(')') {
            let inner = &trimmed[1..trimmed.len() - 1];
            let mut depth = 0i32;
            let mut wraps = true;
            for b in inner.bytes() {
                match b {
                    b'(' => depth += 1,
                    b')' => {
                        depth -= 1;
                        if depth < 0 {
                            wraps = false;
                            break;
                        }
                    }
                    _ => {}
                }
            }
            if wraps && depth == 0 {
                return eval_expr(inner, resolve);
            }
        }
        // Top-level ternary (depth-0 over () and []).
        let bytes = trimmed.as_bytes();
        let mut depth = 0i32;
        let mut qpos: Option<usize> = None;
        for (i, &b) in bytes.iter().enumerate() {
            match b {
                b'(' | b'[' => depth += 1,
                b')' | b']' => depth -= 1,
                b'?' => {
                    if depth == 0 {
                        qpos = Some(i);
                        break;
                    }
                }
                _ => {}
            }
        }
        if let Some(q) = qpos {
            // find matching ':' at depth 0 after q
            let mut depth = 0i32;
            for (j, &b) in bytes.iter().enumerate().skip(q + 1) {
                match b {
                    b'(' | b'[' => depth += 1,
                    b')' | b']' => depth -= 1,
                    b':' => {
                        if depth == 0 {
                            let cond = &trimmed[..q];
                            let taken = if eval_condition(cond, resolve) {
                                &trimmed[q + 1..j]
                            } else {
                                &trimmed[j + 1..]
                            };
                            return eval_expr(taken, resolve);
                        }
                    }
                    _ => {}
                }
            }
        }
        // Outer `name[<expr>]` where the index itself is an expression.
        if trimmed.ends_with(']') {
            if let Some(br) = trimmed.find('[') {
                let name = &trimmed[..br];
                let inner = &trimmed[br + 1..trimmed.len() - 1];
                if !name.is_empty()
                    && name.chars().all(|c| c.is_alphanumeric() || c == '_')
                    && (inner.contains('?') || inner.contains('(') || inner.contains('['))
                {
                    let idx = eval_expr(inner, resolve);
                    let flat = format!("{}[{}]", name, idx.trim());
                    if let Some(v) = resolve(&flat) {
                        return v;
                    }
                }
            }
        }
    }

    // Try as a simple variable first (only if no math operators present)
    let has_math = trimmed.contains('+')
        || trimmed.contains('-')
        || trimmed.contains('*')
        || trimmed.contains('/');
    if !has_math {
        if let Some(val) = resolve(trimmed) {
            return val;
        }
    }

    // Handle expressions with math: var op val
    // Simple approach: resolve all variable references, then evaluate
    // For now, handle the common patterns we see in the templates

    // Pattern: "var/expr" or "var*expr" or "var+expr" or "var-expr"
    // Try to evaluate as a math expression with variable substitution
    let resolved = resolve_all_vars_in_expr(trimmed, resolve);

    // Try to evaluate as a math expression
    match eval_math(&resolved) {
        Some(val) => {
            if val == val.floor() && val.abs() < 1e9 {
                format!("{:.0}", val)
            } else {
                // Match C++ std::ostringstream default: 6 significant digits (%g format)
                // Then trim trailing zeros
                let sig_digits = 6;
                let magnitude = if val.abs() > 0.0 {
                    val.abs().log10().floor() as i32 + 1
                } else {
                    1
                };
                let decimal_places = (sig_digits - magnitude).max(0) as usize;
                let s = format!("{:.prec$}", val, prec = decimal_places);
                s.trim_end_matches('0').trim_end_matches('.').to_string()
            }
        }
        None => {
            // Return as-is if can't evaluate
            format!("{{{}}}", expr)
        }
    }
}

/// Replace variable names in an expression with their values
fn resolve_all_vars_in_expr(expr: &str, resolve: &dyn Fn(&str) -> Option<String>) -> String {
    let mut result = expr.to_string();

    // Find variable-like tokens (word chars + brackets) and try to resolve them
    // This is a simplistic approach - iterate and replace known patterns
    let mut attempts = 0;
    loop {
        let old = result.clone();
        // Find alphanumeric tokens that could be variables
        let re_like: Vec<(usize, usize)> = {
            let mut spans = Vec::new();
            let bytes = result.as_bytes();
            let mut i = 0;
            while i < bytes.len() {
                if bytes[i].is_ascii_alphabetic() || bytes[i] == b'_' {
                    let start = i;
                    while i < bytes.len()
                        && (bytes[i].is_ascii_alphanumeric()
                            || bytes[i] == b'_'
                            || bytes[i] == b'['
                            || bytes[i] == b']')
                    {
                        i += 1;
                    }
                    spans.push((start, i));
                } else {
                    i += 1;
                }
            }
            spans
        };

        // Replace from right to left to preserve indices
        for &(start, end) in re_like.iter().rev() {
            let token = &result[start..end];
            if let Some(val) = resolve(token) {
                result = format!("{}{}{}", &result[..start], val, &result[end..]);
            }
        }

        attempts += 1;
        if result == old || attempts > 3 {
            break;
        }
    }

    result
}

/// Evaluate a simple math expression (numbers and +-*/)
fn eval_math(expr: &str) -> Option<f64> {
    // Simple recursive descent for +, -, *, /
    let trimmed = expr.trim();
    if trimmed.is_empty() {
        return None;
    }

    // Try as a plain number
    if let Ok(n) = trimmed.parse::<f64>() {
        return Some(n);
    }

    // Find the last + or - (lowest precedence) outside parentheses
    let mut depth = 0i32;
    let mut last_add_sub = None;
    let bytes = trimmed.as_bytes();
    for i in (0..bytes.len()).rev() {
        match bytes[i] {
            b')' => depth += 1,
            b'(' => depth -= 1,
            b'+' | b'-' if depth == 0 && i > 0 => {
                // Make sure it's not a unary minus or part of scientific notation
                if i > 0 && !matches!(bytes[i - 1], b'(' | b'*' | b'/' | b'+' | b'-') {
                    last_add_sub = Some(i);
                    break;
                }
            }
            _ => {}
        }
    }

    if let Some(pos) = last_add_sub {
        let left = eval_math(&trimmed[..pos])?;
        let right = eval_math(&trimmed[pos + 1..])?;
        return Some(if bytes[pos] == b'+' {
            left + right
        } else {
            left - right
        });
    }

    // Find last * or / outside parentheses
    let mut last_mul_div = None;
    depth = 0;
    for i in (0..bytes.len()).rev() {
        match bytes[i] {
            b')' => depth += 1,
            b'(' => depth -= 1,
            b'*' | b'/' if depth == 0 => {
                last_mul_div = Some(i);
                break;
            }
            _ => {}
        }
    }

    if let Some(pos) = last_mul_div {
        let left = eval_math(&trimmed[..pos])?;
        let right = eval_math(&trimmed[pos + 1..])?;
        return Some(if bytes[pos] == b'*' {
            left * right
        } else if right != 0.0 {
            left / right
        } else {
            0.0
        });
    }

    // Handle parentheses
    if trimmed.starts_with('(') && trimmed.ends_with(')') {
        return eval_math(&trimmed[1..trimmed.len() - 1]);
    }

    // Handle unary minus
    if trimmed.starts_with('-') {
        return eval_math(&trimmed[1..]).map(|v| -v);
    }
    if trimmed.starts_with('+') {
        return eval_math(&trimmed[1..]);
    }

    None
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_gcode_new() {
        let gcode = GCode::new();
        assert!(gcode.is_empty());
        assert_eq!(gcode.line_count(), 0);
    }

    #[test]
    fn test_gcode_append_line() {
        let mut gcode = GCode::new();
        gcode.append_line("G1 X10 Y20");
        gcode.append_line("G1 Z5");

        assert_eq!(gcode.line_count(), 2);
        assert_eq!(gcode.content(), "G1 X10 Y20\nG1 Z5\n");
    }

    #[test]
    fn test_gcode_append_comment() {
        let mut gcode = GCode::new();
        gcode.append_comment("Test comment");
        assert_eq!(gcode.content(), "; Test comment\n");
    }

    #[test]
    fn test_gcode_from_string() {
        let content = "G1 X10\nG1 Y20\n";
        let gcode = GCode::from_string(content.to_string());
        assert_eq!(gcode.line_count(), 2);
        assert_eq!(gcode.content(), content);
    }

    #[test]
    fn test_gcode_lines_iterator() {
        let mut gcode = GCode::new();
        gcode.append_line("G1 X10");
        gcode.append_line("G1 Y20");
        gcode.append_line("G1 Z5");

        let lines: Vec<&str> = gcode.lines().collect();
        assert_eq!(lines.len(), 3);
        assert_eq!(lines[0], "G1 X10");
        assert_eq!(lines[1], "G1 Y20");
        assert_eq!(lines[2], "G1 Z5");
    }

    #[test]
    fn test_gcode_clear() {
        let mut gcode = GCode::new();
        gcode.append_line("G1 X10");
        assert!(!gcode.is_empty());

        gcode.clear();
        assert!(gcode.is_empty());
        assert_eq!(gcode.line_count(), 0);
    }

    #[test]
    fn test_gcode_stats_print_time_formatted() {
        let mut stats = GCodeStats::new();
        stats.print_time_seconds = 3665.0; // 1h 1m 5s
        assert_eq!(stats.print_time_formatted(), "01:01:05");
    }

    #[test]
    fn test_gcode_stats_filament_meters() {
        let mut stats = GCodeStats::new();
        stats.filament_length_mm = 2500.0;
        assert_eq!(stats.filament_used_meters(), 2.5);
    }
}
