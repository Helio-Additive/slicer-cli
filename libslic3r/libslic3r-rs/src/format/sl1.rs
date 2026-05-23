//! SL1/SL1S (SLA resin printer) archive handling.
//!
//! C++ Reference:
//! - Format/SL1.hpp
//! - Format/SL1.cpp
//!
//! SL1 archives are ZIP files containing slice images (PNGs), a config.ini,
//! and optionally a prusaslicer.ini profile.  This module provides:
//! - Archive extraction and inspection
//! - Profile/config import
//! - Rasterisation parameter extraction
//! - Export of SLA print data to a ZIP archive (Zipper)

use crate::{Error, Result};
use std::collections::HashMap;
use std::path::Path;

// ---------------------------------------------------------------------------
// Error types  (SL1.hpp:60)
// ---------------------------------------------------------------------------

/// Error raised when the SL1 archive lacks a required print profile.
/// SL1.hpp:60
#[derive(Debug, Clone)]
pub struct MissingProfileError {
    pub message: String,
}

impl MissingProfileError {
    pub fn new(msg: impl Into<String>) -> Self {
        Self {
            message: msg.into(),
        }
    }
}

impl std::fmt::Display for MissingProfileError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "MissingProfileError: {}", self.message)
    }
}

impl std::error::Error for MissingProfileError {}

// ---------------------------------------------------------------------------
// Archive data structures  (SL1.cpp:53-61)
// ---------------------------------------------------------------------------

/// A PNG image buffer extracted from the archive.
#[derive(Debug, Clone)]
pub struct PngBuffer {
    pub buf: Vec<u8>,
    pub fname: String,
}

/// Data extracted from an SL1 archive.
#[derive(Debug, Clone)]
pub struct ArchiveData {
    /// Key-value pairs from `prusaslicer.ini`.
    pub profile: HashMap<String, String>,
    /// Key-value pairs from `config.ini`.
    pub config: HashMap<String, String>,
    /// Slice images sorted by filename.
    pub images: Vec<PngBuffer>,
}

impl ArchiveData {
    pub fn new() -> Self {
        Self {
            profile: HashMap::new(),
            config: HashMap::new(),
            images: Vec::new(),
        }
    }
}

/// Well-known filenames inside an SL1 archive.
const CONFIG_FNAME: &str = "config.ini";
const PROFILE_FNAME: &str = "prusaslicer.ini";

// ---------------------------------------------------------------------------
// Raster / Slice parameters  (SL1.cpp:188-237)
// ---------------------------------------------------------------------------

/// Raster transformation parameters derived from the print profile.
/// SL1.cpp:188-193
#[derive(Debug, Clone)]
pub struct RasterTrafo {
    pub mirror_x: bool,
    pub mirror_y: bool,
    pub flip_xy: bool,
    pub center_x: i64,
    pub center_y: i64,
}

impl RasterTrafo {
    pub fn new() -> Self {
        Self {
            mirror_x: false,
            mirror_y: false,
            flip_xy: false,
            center_x: 0,
            center_y: 0,
        }
    }
}

/// Parameters for rasterisation extracted from the profile.
/// SL1.cpp:188-193
#[derive(Debug, Clone)]
pub struct RasterParams {
    pub trafo: RasterTrafo,
    pub width: i64,
    pub height: i64,
    pub px_w: f64,
    pub px_h: f64,
    pub win_rows: i32,
    pub win_cols: i32,
}

/// Layer height parameters extracted from the profile.
/// SL1.cpp:225-226
#[derive(Debug, Clone, Copy)]
pub struct SliceParams {
    pub layer_h: f64,
    pub initial_layer_h: f64,
}

/// Extract raster parameters from a profile config map.
/// SL1.cpp:195-223
pub fn get_raster_params(
    cfg: &HashMap<String, String>,
) -> std::result::Result<RasterParams, MissingProfileError> {
    let get_int = |key: &str| -> std::result::Result<i64, MissingProfileError> {
        cfg.get(key)
            .ok_or_else(|| MissingProfileError::new(format!("Missing key: {}", key)))
            .and_then(|v| {
                v.trim().parse::<i64>().map_err(|_| {
                    MissingProfileError::new(format!("Invalid integer for key {}: {}", key, v))
                })
            })
    };
    let get_float = |key: &str| -> std::result::Result<f64, MissingProfileError> {
        cfg.get(key)
            .ok_or_else(|| MissingProfileError::new(format!("Missing key: {}", key)))
            .and_then(|v| {
                v.trim().parse::<f64>().map_err(|_| {
                    MissingProfileError::new(format!("Invalid float for key {}: {}", key, v))
                })
            })
    };
    let get_bool = |key: &str| -> std::result::Result<bool, MissingProfileError> {
        cfg.get(key)
            .ok_or_else(|| MissingProfileError::new(format!("Missing key: {}", key)))
            .map(|v| v.trim() == "1" || v.trim().eq_ignore_ascii_case("true"))
    };

    let disp_cols = get_int("display_pixels_x")?;
    let disp_rows = get_int("display_pixels_y")?;
    let disp_w = get_float("display_width")?;
    let disp_h = get_float("display_height")?;
    let mirror_x = get_bool("display_mirror_x")?;
    let mirror_y = get_bool("display_mirror_y")?;
    let orient = cfg
        .get("display_orientation")
        .map(|v| v.trim().to_string())
        .unwrap_or_default();

    let is_landscape = orient == "landscape" || orient == "1";

    let px_w = disp_w / (disp_cols - 1) as f64;
    let px_h = disp_h / (disp_rows - 1) as f64;

    let trafo = RasterTrafo {
        mirror_x,
        mirror_y,
        flip_xy: is_landscape,
        center_x: 0,
        center_y: 0,
    };

    let scale = 1_000_000.0; // scaled coordinates
    Ok(RasterParams {
        trafo,
        width: (disp_w * scale) as i64,
        height: (disp_h * scale) as i64,
        px_w,
        px_h,
        win_rows: 4,
        win_cols: 4,
    })
}

/// Extract layer height parameters from a profile config map.
/// SL1.cpp:227-236
pub fn get_slice_params(
    cfg: &HashMap<String, String>,
) -> std::result::Result<SliceParams, MissingProfileError> {
    let layer_h: f64 = cfg
        .get("layer_height")
        .ok_or_else(|| MissingProfileError::new("Missing layer_height"))?
        .trim()
        .parse()
        .map_err(|_| MissingProfileError::new("Invalid layer_height"))?;
    let initial_layer_h: f64 = cfg
        .get("initial_layer_height")
        .ok_or_else(|| MissingProfileError::new("Missing initial_layer_height"))?
        .trim()
        .parse()
        .map_err(|_| MissingProfileError::new("Invalid initial_layer_height"))?;
    Ok(SliceParams {
        layer_h,
        initial_layer_h,
    })
}

// ---------------------------------------------------------------------------
// Archive extraction  (SL1.cpp:90-134)
// ---------------------------------------------------------------------------

/// Parse an INI-style string into a key-value map.
fn parse_ini(text: &str) -> HashMap<String, String> {
    let mut map = HashMap::new();
    for line in text.lines() {
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') || line.starts_with(';') {
            continue;
        }
        if let Some(eq_pos) = line.find('=') {
            let key = line[..eq_pos].trim().to_string();
            let val = line[eq_pos + 1..].trim().to_string();
            map.insert(key, val);
        }
    }
    map
}

/// Extract an SL1 archive from a ZIP file, returning its data.
///
/// In the C++ code this uses miniz. Here we do a minimal uncompressed-ZIP
/// walk. A production build should use the `zip` crate.
///
/// SL1.cpp:90-134
pub fn extract_sla_archive(zip_path: &Path, exclude: &str) -> Result<ArchiveData> {
    let data = std::fs::read(zip_path)
        .map_err(|e| Error::IO(format!("Cannot read SL1 archive: {}", e)))?;

    let mut arch = ArchiveData::new();
    let mut pos = 0;

    while pos + 30 <= data.len() {
        if data[pos..pos + 4] != [0x50, 0x4b, 0x03, 0x04] {
            pos += 1;
            continue;
        }

        let compression = u16::from_le_bytes([data[pos + 8], data[pos + 9]]);
        let compressed_size = u32::from_le_bytes([
            data[pos + 18],
            data[pos + 19],
            data[pos + 20],
            data[pos + 21],
        ]) as usize;
        let uncompressed_size = u32::from_le_bytes([
            data[pos + 22],
            data[pos + 23],
            data[pos + 24],
            data[pos + 25],
        ]) as usize;
        let filename_len = u16::from_le_bytes([data[pos + 26], data[pos + 27]]) as usize;
        let extra_len = u16::from_le_bytes([data[pos + 28], data[pos + 29]]) as usize;

        let name_start = pos + 30;
        let name_end = name_start + filename_len;
        if name_end > data.len() {
            break;
        }
        let raw_name = String::from_utf8_lossy(&data[name_start..name_end]).to_string();
        let name = raw_name.to_lowercase();

        let data_start = name_end + extra_len;

        // Skip excluded entries
        if !exclude.is_empty() && name.contains(exclude) {
            pos = data_start + compressed_size.max(uncompressed_size);
            continue;
        }

        if compression == 0 && data_start + uncompressed_size <= data.len() {
            let entry_data = &data[data_start..data_start + uncompressed_size];

            if name == CONFIG_FNAME {
                let text = String::from_utf8_lossy(entry_data);
                arch.config = parse_ini(&text);
            } else if name == PROFILE_FNAME {
                let text = String::from_utf8_lossy(entry_data);
                arch.profile = parse_ini(&text);
            } else if name.ends_with(".png") {
                // Insert sorted by filename
                let buf = PngBuffer {
                    buf: entry_data.to_vec(),
                    fname: name.clone(),
                };
                let insert_pos = arch
                    .images
                    .binary_search_by(|b| b.fname.cmp(&name))
                    .unwrap_or_else(|e| e);
                arch.images.insert(insert_pos, buf);
            }
        }

        pos = data_start + compressed_size.max(uncompressed_size);
    }

    Ok(arch)
}

// ---------------------------------------------------------------------------
// Import  (SL1.cpp:291-352)
// ---------------------------------------------------------------------------

/// Import the print profile from an SL1 archive (without extracting images).
/// SL1.cpp:291-295
pub fn import_sla_archive_config(zip_path: &Path) -> Result<HashMap<String, String>> {
    let arch = extract_sla_archive(zip_path, "png")?;
    Ok(arch.profile)
}

/// Import an SL1 archive, extracting profile and slice geometry.
///
/// Returns the profile as a key-value map.  The actual image-to-mesh
/// conversion (marching squares + slices_to_mesh) is not implemented here
/// as it requires the full PNG decode + marching squares pipeline.
///
/// SL1.cpp:300-352
pub fn import_sla_archive(
    zip_path: &Path,
    _window_size: [i32; 2],
) -> Result<HashMap<String, String>> {
    let arch = extract_sla_archive(zip_path, "thumbnail")?;
    Ok(arch.profile)
}

// ---------------------------------------------------------------------------
// SL1Archive  (SL1.hpp:11-39)
// ---------------------------------------------------------------------------

/// SL1 archive writer for exporting SLA print data.
/// SL1.hpp:11-39
#[derive(Debug)]
pub struct SL1Archive {
    /// Printer configuration (display dimensions, etc.).
    pub config: HashMap<String, String>,
    /// Encoded raster layers (PNG data).
    pub layers: Vec<Vec<u8>>,
}

impl SL1Archive {
    /// Create a new SL1Archive with the given printer configuration.
    pub fn new(config: HashMap<String, String>) -> Self {
        Self {
            config,
            layers: Vec::new(),
        }
    }

    /// Create an SL1Archive with default (empty) configuration.
    pub fn default_archive() -> Self {
        Self {
            config: HashMap::new(),
            layers: Vec::new(),
        }
    }

    /// Apply a new printer configuration. Clears cached layers if it differs.
    /// SL1.hpp:31-38
    pub fn apply(&mut self, cfg: HashMap<String, String>) {
        if self.config != cfg {
            self.config = cfg;
            self.layers.clear();
        }
    }

    /// Export the print data to a Zipper (file writer).
    ///
    /// This writes `config.ini`, `prusaslicer.ini`, and all layer PNG images.
    /// The caller provides the actual `write_fn` that adds entries to the archive.
    ///
    /// SL1.cpp:481-516
    pub fn export_print<F>(
        &self,
        project_name: &str,
        ini_config: &HashMap<String, String>,
        slicer_config: &HashMap<String, String>,
        mut write_fn: F,
    ) -> Result<()>
    where
        F: FnMut(&str, &[u8]) -> Result<()>,
    {
        // Write config.ini
        let ini_text = to_ini(ini_config);
        write_fn("config.ini", ini_text.as_bytes())?;

        // Write prusaslicer.ini
        let slicer_text = to_ini(slicer_config);
        write_fn("prusaslicer.ini", slicer_text.as_bytes())?;

        // Write layer images
        for (i, layer_data) in self.layers.iter().enumerate() {
            let img_name = format!("{}{:05}.png", project_name, i);
            write_fn(&img_name, layer_data)?;
        }

        Ok(())
    }
}

/// Convert a key-value map to INI format string.
/// SL1.cpp:358-363
fn to_ini(m: &HashMap<String, String>) -> String {
    let mut result = String::new();
    let mut keys: Vec<&String> = m.keys().collect();
    keys.sort();
    for key in keys {
        result.push_str(key);
        result.push_str(" = ");
        result.push_str(&m[key]);
        result.push('\n');
    }
    result
}

/// Convenience wrapper: export to a file path.
/// SL1.hpp:25-29
pub fn export_sl1_to_file(
    archive: &SL1Archive,
    path: &Path,
    project_name: &str,
    ini_config: &HashMap<String, String>,
    slicer_config: &HashMap<String, String>,
) -> Result<()> {
    // For a real implementation, use a ZIP writer.
    // Here we write raw files to a directory as a placeholder.
    let dir = path.parent().unwrap_or(Path::new("."));

    archive.export_print(project_name, ini_config, slicer_config, |name, data| {
        let file_path = dir.join(name);
        if let Some(parent) = file_path.parent() {
            std::fs::create_dir_all(parent)
                .map_err(|e| Error::IO(format!("mkdir failed: {}", e)))?;
        }
        std::fs::write(&file_path, data).map_err(|e| Error::IO(format!("write failed: {}", e)))?;
        Ok(())
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_missing_profile_error() {
        let e = MissingProfileError::new("test error");
        assert_eq!(e.message, "test error");
        assert!(format!("{}", e).contains("test error"));
    }

    #[test]
    fn test_parse_ini() {
        let text = "key1 = value1\nkey2 = value2\n# comment\n";
        let map = parse_ini(text);
        assert_eq!(map.get("key1").unwrap(), "value1");
        assert_eq!(map.get("key2").unwrap(), "value2");
        assert_eq!(map.len(), 2);
    }

    #[test]
    fn test_to_ini() {
        let mut m = HashMap::new();
        m.insert("b".to_string(), "2".to_string());
        m.insert("a".to_string(), "1".to_string());
        let s = to_ini(&m);
        assert!(s.contains("a = 1"));
        assert!(s.contains("b = 2"));
    }

    #[test]
    fn test_get_slice_params() {
        let mut cfg = HashMap::new();
        cfg.insert("layer_height".to_string(), "0.05".to_string());
        cfg.insert("initial_layer_height".to_string(), "0.1".to_string());
        let params = get_slice_params(&cfg).unwrap();
        assert!((params.layer_h - 0.05).abs() < 1e-9);
        assert!((params.initial_layer_h - 0.1).abs() < 1e-9);
    }

    #[test]
    fn test_get_slice_params_missing() {
        let cfg = HashMap::new();
        assert!(get_slice_params(&cfg).is_err());
    }

    #[test]
    fn test_sl1_archive_apply() {
        let mut archive = SL1Archive::default_archive();
        archive.layers.push(vec![1, 2, 3]);
        let mut new_cfg = HashMap::new();
        new_cfg.insert("foo".to_string(), "bar".to_string());
        archive.apply(new_cfg);
        assert!(archive.layers.is_empty()); // layers cleared on config change
    }
}
