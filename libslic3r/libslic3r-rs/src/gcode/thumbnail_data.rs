//! Thumbnail data module for G-code embedded thumbnails.
//!
//! C++ Reference:
//! - GCode/ThumbnailData.hpp
//! - GCode/ThumbnailData.cpp
//!
//! This module provides types for managing thumbnail images and bounding box
//! data embedded in G-code files for preview purposes.

/// Default thumbnail sizes embedded in G-code files.
pub const DEFAULT_THUMBNAIL_SIZES: &[(u32, u32)] = &[(50, 50)];

/// Thumbnail image data (RGBA pixels).
/// Corresponds to C++ ThumbnailData.
#[derive(Debug, Clone)]
pub struct ThumbnailData {
    pub width: u32,
    pub height: u32,
    pub pixels: Vec<u8>,
}

impl ThumbnailData {
    pub fn new() -> Self {
        Self {
            width: 0,
            height: 0,
            pixels: Vec::new(),
        }
    }

    /// Set the thumbnail dimensions and allocate pixel buffer.
    pub fn set(&mut self, w: u32, h: u32) {
        self.width = w;
        self.height = h;
        // 4 bytes per pixel (RGBA)
        self.pixels = vec![0u8; (w * h * 4) as usize];
    }

    /// Reset the thumbnail data.
    pub fn reset(&mut self) {
        self.width = 0;
        self.height = 0;
        self.pixels.clear();
    }

    /// Check if this thumbnail contains valid data.
    pub fn is_valid(&self) -> bool {
        self.width > 0
            && self.height > 0
            && self.pixels.len() == (self.width * self.height * 4) as usize
    }

    /// Load data from another ThumbnailData.
    pub fn load_from(&mut self, other: &ThumbnailData) {
        self.set(other.width, other.height);
        self.pixels = other.pixels.clone();
    }
}

impl Default for ThumbnailData {
    fn default() -> Self {
        Self::new()
    }
}

/// Bounding box data for a single object.
/// Corresponds to C++ BBoxData.
#[derive(Debug, Clone)]
pub struct BBoxData {
    /// Object ID.
    pub id: i32,
    /// First layer bounding box: [min_x, min_y, max_x, max_y].
    pub bbox: Vec<f64>,
    /// First layer area.
    pub area: f32,
    /// Layer height.
    pub layer_height: f32,
    /// Object name.
    pub name: String,
}

impl BBoxData {
    pub fn new() -> Self {
        Self {
            id: 0,
            bbox: Vec::new(),
            area: 0.0,
            layer_height: 0.0,
            name: String::new(),
        }
    }

    /// Serialize to a simple key-value representation.
    pub fn to_json_string(&self) -> String {
        format!(
            r#"{{"id":{},"bbox":[{}],"area":{},"layer_height":{},"name":"{}"}}"#,
            self.id,
            self.bbox
                .iter()
                .map(|v| format!("{}", v))
                .collect::<Vec<_>>()
                .join(","),
            self.area,
            self.layer_height,
            self.name
        )
    }

    /// Deserialize from a simple string representation.
    /// Returns None if parsing fails.
    pub fn from_json_string(_s: &str) -> Option<Self> {
        // Simplified: a full implementation would parse JSON
        // For now, return None to indicate parse not implemented
        None
    }
}

impl Default for BBoxData {
    fn default() -> Self {
        Self::new()
    }
}

/// Parameters for thumbnail generation.
/// Corresponds to C++ ThumbnailsParams.
#[derive(Debug, Clone)]
pub struct ThumbnailsParams {
    /// Requested thumbnail sizes (width, height).
    pub sizes: Vec<(f64, f64)>,
    /// Only include printable objects.
    pub printable_only: bool,
    /// Only include object parts (no supports, etc.).
    pub parts_only: bool,
    /// Show the bed in the thumbnail.
    pub show_bed: bool,
    /// Use transparent background.
    pub transparent_background: bool,
    /// Plate ID for multi-plate setups.
    pub plate_id: i32,
    /// Use plate bounding box for framing.
    pub use_plate_box: bool,
    /// Enable post-processing on the thumbnail.
    pub post_processing_enabled: bool,
    /// Background color (RGBA).
    pub background_color: [f32; 4],
}

impl ThumbnailsParams {
    pub fn new() -> Self {
        Self {
            sizes: vec![(50.0, 50.0)],
            printable_only: false,
            parts_only: false,
            show_bed: false,
            transparent_background: false,
            plate_id: 0,
            use_plate_box: true,
            post_processing_enabled: false,
            background_color: [0.0, 0.0, 0.0, 0.0],
        }
    }
}

impl Default for ThumbnailsParams {
    fn default() -> Self {
        Self::new()
    }
}

/// Bounding box data for an entire plate.
/// Corresponds to C++ PlateBBoxData.
#[derive(Debug, Clone)]
pub struct PlateBBoxData {
    /// Total bounding box of all objects including brim.
    pub bbox_all: Vec<f64>,
    /// Bounding box data for individual objects.
    pub bbox_objs: Vec<BBoxData>,
    /// Filament IDs used on this plate.
    pub filament_ids: Vec<i32>,
    /// Filament colors.
    pub filament_colors: Vec<String>,
    /// Whether this plate uses sequential printing.
    pub is_seq_print: bool,
    /// First extruder ID.
    pub first_extruder: i32,
    /// Nozzle diameter.
    pub nozzle_diameter: f32,
    /// Bed type string.
    pub bed_type: String,
    /// First layer print time estimate.
    pub first_layer_time: f32,
    /// Data format version (1=ColorPrint view, 2=FilamentId view).
    pub version: i32,
}

impl PlateBBoxData {
    pub fn new() -> Self {
        Self {
            bbox_all: Vec::new(),
            bbox_objs: Vec::new(),
            filament_ids: Vec::new(),
            filament_colors: Vec::new(),
            is_seq_print: false,
            first_extruder: 0,
            nozzle_diameter: 0.4,
            bed_type: String::new(),
            first_layer_time: 0.0,
            version: 2,
        }
    }

    /// Serialize to a JSON-like string.
    pub fn to_json_string(&self) -> String {
        let bbox_str = self
            .bbox_all
            .iter()
            .map(|v| format!("{}", v))
            .collect::<Vec<_>>()
            .join(",");
        format!(
            r#"{{"bbox_all":[{}],"is_seq_print":{},"first_extruder":{},"nozzle_diameter":{},"version":{},"bed_type":"{}","first_layer_time":{}}}"#,
            bbox_str,
            self.is_seq_print,
            self.first_extruder,
            self.nozzle_diameter,
            self.version,
            self.bed_type,
            self.first_layer_time
        )
    }

    /// Parse from a JSON-like string. Returns None if parsing fails.
    pub fn from_json_string(_s: &str) -> Option<Self> {
        // Simplified: a full implementation would parse JSON
        None
    }
}

impl Default for PlateBBoxData {
    fn default() -> Self {
        Self::new()
    }
}

/// Reset thumbnail data.
pub fn reset(data: &mut ThumbnailData) {
    data.reset();
}

/// Parse BBoxData from a JSON string.
pub fn from_json(s: &str) -> crate::Result<Option<BBoxData>> {
    Ok(BBoxData::from_json_string(s))
}

/// Serialize BBoxData to a JSON string.
pub fn to_json(data: &BBoxData) -> crate::Result<String> {
    Ok(data.to_json_string())
}

/// Check if thumbnail data is valid.
pub fn is_valid(data: &ThumbnailData) -> bool {
    data.is_valid()
}

/// Set thumbnail dimensions.
pub fn set(data: &mut ThumbnailData, width: u32, height: u32) {
    data.set(width, height);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_thumbnail_data_new() {
        let td = ThumbnailData::new();
        assert_eq!(td.width, 0);
        assert_eq!(td.height, 0);
        assert!(!td.is_valid());
    }

    #[test]
    fn test_thumbnail_data_set() {
        let mut td = ThumbnailData::new();
        td.set(50, 50);
        assert_eq!(td.width, 50);
        assert_eq!(td.height, 50);
        assert_eq!(td.pixels.len(), 50 * 50 * 4);
        assert!(td.is_valid());
    }

    #[test]
    fn test_thumbnail_data_reset() {
        let mut td = ThumbnailData::new();
        td.set(50, 50);
        td.reset();
        assert_eq!(td.width, 0);
        assert!(!td.is_valid());
    }

    #[test]
    fn test_thumbnail_data_load_from() {
        let mut src = ThumbnailData::new();
        src.set(32, 32);
        src.pixels[0] = 255;

        let mut dst = ThumbnailData::new();
        dst.load_from(&src);
        assert_eq!(dst.width, 32);
        assert_eq!(dst.pixels[0], 255);
    }

    #[test]
    fn test_bbox_data() {
        let bbox = BBoxData {
            id: 1,
            bbox: vec![0.0, 0.0, 10.0, 10.0],
            area: 100.0,
            layer_height: 0.2,
            name: "test_obj".into(),
        };
        let json = bbox.to_json_string();
        assert!(json.contains("test_obj"));
    }

    #[test]
    fn test_thumbnails_params_default() {
        let params = ThumbnailsParams::new();
        assert_eq!(params.sizes.len(), 1);
        assert!(params.use_plate_box);
    }

    #[test]
    fn test_plate_bbox_data() {
        let plate = PlateBBoxData::new();
        assert_eq!(plate.version, 2);
        assert_eq!(plate.nozzle_diameter, 0.4);
    }

    #[test]
    fn test_convenience_functions() {
        let mut td = ThumbnailData::new();
        set(&mut td, 64, 64);
        assert!(is_valid(&td));
        reset(&mut td);
        assert!(!is_valid(&td));
    }
}
