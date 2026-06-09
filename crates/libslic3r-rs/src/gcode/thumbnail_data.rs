//! Thumbnail data module for G-code embedded thumbnails.
//!
//! 1:1 line-by-line port of:
//! - GCode/ThumbnailData.hpp
//! - GCode/ThumbnailData.cpp
//!
//! C++ uses `nlohmann::json`; here we mirror it with `serde_json::Value`.
//! nlohmann's default `json` object type is `std::map`, which orders keys
//! lexicographically; `serde_json::Map` without the `preserve_order` feature
//! is a `BTreeMap`, also lexicographic — so the emitted JSON key ordering
//! matches the C++ output.
//!
//! coord_t -> i64, coordf_t -> f64.

use serde_json::{json, Value};

use crate::geometry::Vec2d;

/// Vec2ds is `std::vector<Vec2d>` in Point.hpp.
/// ThumbnailData.hpp:35 (`const Vec2ds sizes;`)
pub type Vec2ds = Vec<Vec2d>;

// ThumbnailData.hpp:10-11
//BBS: thumbnail_size in gcode file
// `static std::vector<Vec2d> THUMBNAIL_SIZE = { Vec2d(50, 50) };`
//
// Provided as a constructor function rather than a mutable static so that it is
// wasm-safe and does not rely on a global mutable; callers that need the value
// should call `thumbnail_size()`.
pub fn thumbnail_size() -> Vec<Vec2d> {
    vec![Vec2d::new(50.0, 50.0)]
}

// ThumbnailData.hpp:13-28
#[derive(Debug, Clone)]
pub struct ThumbnailData {
    pub width: u32,
    pub height: u32,
    pub pixels: Vec<u8>,
}

impl ThumbnailData {
    // ThumbnailData.hpp:19
    // `ThumbnailData() { reset(); }`
    pub fn new() -> Self {
        let mut data = Self {
            width: 0,
            height: 0,
            pixels: Vec::new(),
        };
        data.reset();
        data
    }

    // ThumbnailData.cpp:5
    // void ThumbnailData::set(unsigned int w, unsigned int h)
    pub fn set(&mut self, w: u32, h: u32) {
        // ThumbnailData.cpp:7-8
        if (w == 0) || (h == 0) {
            return;
        }

        // ThumbnailData.cpp:10
        if (self.width != w) || (self.height != h) {
            // ThumbnailData.cpp:12-13
            self.width = w;
            self.height = h;
            // ThumbnailData.cpp:14
            // defaults to white texture
            // ThumbnailData.cpp:15-16
            self.pixels.clear();
            self.pixels = vec![255u8; (self.width * self.height * 4) as usize];
        }
    }

    // ThumbnailData.cpp:20
    // void ThumbnailData::reset()
    pub fn reset(&mut self) {
        // ThumbnailData.cpp:22-24
        self.width = 0;
        self.height = 0;
        self.pixels.clear();
    }

    // ThumbnailData.cpp:27
    // bool ThumbnailData::is_valid() const
    pub fn is_valid(&self) -> bool {
        // ThumbnailData.cpp:29
        (self.width != 0)
            && (self.height != 0)
            && (self.pixels.len() as u32 == 4 * self.width * self.height)
    }

    // ThumbnailData.hpp:24-27
    // void load_from(ThumbnailData &data)
    pub fn load_from(&mut self, data: &ThumbnailData) {
        // ThumbnailData.hpp:25-26
        self.set(data.width, data.height);
        self.pixels = data.pixels.clone();
    }
}

impl Default for ThumbnailData {
    fn default() -> Self {
        Self::new()
    }
}

// ThumbnailData.hpp:31
//BBS: add plate id into thumbnail render logic
// `using ThumbnailsList = std::vector<ThumbnailData>;`
pub type ThumbnailsList = Vec<ThumbnailData>;

// ThumbnailData.hpp:33-44
#[derive(Debug, Clone)]
pub struct ThumbnailsParams {
    // ThumbnailData.hpp:35
    pub sizes: Vec2ds,
    // ThumbnailData.hpp:36
    pub printable_only: bool,
    // ThumbnailData.hpp:37
    pub parts_only: bool,
    // ThumbnailData.hpp:38
    pub show_bed: bool,
    // ThumbnailData.hpp:39
    pub transparent_background: bool,
    // ThumbnailData.hpp:40
    pub plate_id: i32,
    // ThumbnailData.hpp:41 `bool use_plate_box{true};`
    pub use_plate_box: bool,
    // ThumbnailData.hpp:42 `bool post_processing_enabled{ false };`
    pub post_processing_enabled: bool,
    // ThumbnailData.hpp:43 `Vec4f background_color{ 0.0f, 0.0f, 0.0f, 0.0f };`
    pub background_color: [f32; 4],
}

impl Default for ThumbnailsParams {
    fn default() -> Self {
        Self {
            sizes: Vec2ds::new(),
            printable_only: false,
            parts_only: false,
            show_bed: false,
            transparent_background: false,
            plate_id: 0,
            // ThumbnailData.hpp:41
            use_plate_box: true,
            // ThumbnailData.hpp:42
            post_processing_enabled: false,
            // ThumbnailData.hpp:43
            background_color: [0.0f32, 0.0f32, 0.0f32, 0.0f32],
        }
    }
}

// ThumbnailData.hpp:48-71
#[derive(Debug, Clone)]
pub struct BBoxData {
    // ThumbnailData.hpp:50 `int id;  // object id`
    pub id: i32,
    // ThumbnailData.hpp:51 `std::vector<coordf_t> bbox;` first layer bounding box: min.{x,y}, max.{x,y}
    pub bbox: Vec<f64>,
    // ThumbnailData.hpp:52 `float area;` first layer area
    pub area: f32,
    // ThumbnailData.hpp:53 `float layer_height;`
    pub layer_height: f32,
    // ThumbnailData.hpp:54 `std::string name;`
    pub name: String,
}

impl Default for BBoxData {
    fn default() -> Self {
        Self {
            id: 0,
            bbox: Vec::new(),
            area: 0.0,
            layer_height: 0.0,
            name: String::new(),
        }
    }
}

impl BBoxData {
    // ThumbnailData.hpp:55
    // void to_json(nlohmann::json& j) const
    pub fn to_json(&self, j: &mut Value) {
        // ThumbnailData.hpp:56-62
        *j = json!({
            "id": self.id,
            "bbox": self.bbox,
            "area": self.area,
            "layer_height": self.layer_height,
            "name": self.name
        });
    }

    // ThumbnailData.hpp:64
    // void from_json(const nlohmann::json& j)
    pub fn from_json(&mut self, j: &Value) -> crate::Result<()> {
        // ThumbnailData.hpp:65
        self.id = json_get_i64(j, "id")? as i32;
        // ThumbnailData.hpp:66
        self.bbox = json_get_f64_vec(j, "bbox")?;
        // ThumbnailData.hpp:67
        self.area = json_get_f64(j, "area")? as f32;
        // ThumbnailData.hpp:68
        self.layer_height = json_get_f64(j, "layer_height")? as f32;
        // ThumbnailData.hpp:69
        self.name = json_get_str(j, "name")?;
        Ok(())
    }
}

// ThumbnailData.hpp:73-122
#[derive(Debug, Clone)]
pub struct PlateBBoxData {
    // ThumbnailData.hpp:75 total bounding box of all objects including brim
    pub bbox_all: Vec<f64>,
    // ThumbnailData.hpp:76 BBoxData of seperate object
    pub bbox_objs: Vec<BBoxData>,
    // ThumbnailData.hpp:77 filament id used in curr plate
    pub filament_ids: Vec<i32>,
    // ThumbnailData.hpp:78
    pub filament_colors: Vec<String>,
    // ThumbnailData.hpp:79 `bool is_seq_print = false;`
    pub is_seq_print: bool,
    // ThumbnailData.hpp:80 `int first_extruder = 0;`
    pub first_extruder: i32,
    // ThumbnailData.hpp:81 `float nozzle_diameter = 0.4;`
    pub nozzle_diameter: f32,
    // ThumbnailData.hpp:82 `std::string bed_type;`
    pub bed_type: String,
    // ThumbnailData.hpp:83 `float first_layer_time;`
    pub first_layer_time: f32,
    // ThumbnailData.hpp:84-86
    // version 1: use view type ColorPrint (filament color)
    // version 2: use view type FilamentId (filament id)
    // `int version = 2;`
    pub version: i32,
}

impl Default for PlateBBoxData {
    fn default() -> Self {
        Self {
            bbox_all: Vec::new(),
            bbox_objs: Vec::new(),
            filament_ids: Vec::new(),
            filament_colors: Vec::new(),
            // ThumbnailData.hpp:79
            is_seq_print: false,
            // ThumbnailData.hpp:80
            first_extruder: 0,
            // ThumbnailData.hpp:81
            nozzle_diameter: 0.4,
            bed_type: String::new(),
            // `first_layer_time` is left default-initialized in C++ (float); 0.0 here.
            first_layer_time: 0.0,
            // ThumbnailData.hpp:86
            version: 2,
        }
    }
}

impl PlateBBoxData {
    // ThumbnailData.hpp:88
    // void to_json(nlohmann::json& j) const
    pub fn to_json(&self, j: &mut Value) {
        // ThumbnailData.hpp:89
        *j = json!({ "bbox_all": self.bbox_all });
        // ThumbnailData.hpp:90
        j["filament_ids"] = json!(self.filament_ids);
        // ThumbnailData.hpp:91
        j["filament_colors"] = json!(self.filament_colors);
        // ThumbnailData.hpp:92
        j["is_seq_print"] = json!(self.is_seq_print);
        // ThumbnailData.hpp:93
        j["first_extruder"] = json!(self.first_extruder);
        // ThumbnailData.hpp:94
        j["nozzle_diameter"] = json!(self.nozzle_diameter);
        // ThumbnailData.hpp:95
        j["version"] = json!(self.version);
        // ThumbnailData.hpp:96
        j["bed_type"] = json!(self.bed_type);
        // ThumbnailData.hpp:97
        j["first_layer_time"] = json!(self.first_layer_time);
        // ThumbnailData.hpp:98-102
        for bbox in &self.bbox_objs {
            let mut j_bbox = Value::Null;
            bbox.to_json(&mut j_bbox);
            // `j["bbox_objects"].push_back(j_bbox);`
            // nlohmann auto-creates the array on first push_back into a null value.
            if !j["bbox_objects"].is_array() {
                j["bbox_objects"] = json!([]);
            }
            j["bbox_objects"].as_array_mut().unwrap().push(j_bbox);
        }
    }

    // ThumbnailData.hpp:104
    // void from_json(const nlohmann::json& j)
    pub fn from_json(&mut self, j: &Value) -> crate::Result<()> {
        // ThumbnailData.hpp:105
        self.bbox_all = json_get_f64_vec(j, "bbox_all")?;
        // ThumbnailData.hpp:106
        self.filament_ids = json_get_i32_vec(j, "filament_ids")?;
        // ThumbnailData.hpp:107
        self.filament_colors = json_get_str_vec(j, "filament_colors")?;
        // ThumbnailData.hpp:108
        self.is_seq_print = json_get_bool(j, "is_seq_print")?;
        // ThumbnailData.hpp:109
        self.first_extruder = json_get_i64(j, "first_extruder")? as i32;
        // ThumbnailData.hpp:110
        self.nozzle_diameter = json_get_f64(j, "nozzle_diameter")? as f32;
        // ThumbnailData.hpp:111
        self.version = json_get_i64(j, "version")? as i32;
        // ThumbnailData.hpp:112
        self.bed_type = json_get_str(j, "bed_type")?;
        // ThumbnailData.hpp:113-117
        for bbox_j in json_get_array(j, "bbox_objects")? {
            // ThumbnailData.hpp:114
            let mut bbox_data = BBoxData::default();
            // ThumbnailData.hpp:115
            bbox_data.from_json(bbox_j)?;
            // ThumbnailData.hpp:116
            self.bbox_objs.push(bbox_data);
        }
        Ok(())
    }

    // ThumbnailData.hpp:119
    // bool is_valid() const
    pub fn is_valid(&self) -> bool {
        // ThumbnailData.hpp:120 `return !bbox_objs.empty();`
        !self.bbox_objs.is_empty()
    }
}

// -- helpers mirroring nlohmann's `j.at(key).get_to(...)` accessors --
// `j.at(key)` throws if the key is missing; here we map a missing key or a
// type mismatch into `crate::Error::ParseError`.

fn json_at<'a>(j: &'a Value, key: &str) -> crate::Result<&'a Value> {
    j.get(key)
        .ok_or_else(|| crate::Error::ParseError(format!("missing key: {}", key)))
}

fn json_get_i64(j: &Value, key: &str) -> crate::Result<i64> {
    json_at(j, key)?
        .as_i64()
        .ok_or_else(|| crate::Error::ParseError(format!("key {} is not an integer", key)))
}

fn json_get_f64(j: &Value, key: &str) -> crate::Result<f64> {
    json_at(j, key)?
        .as_f64()
        .ok_or_else(|| crate::Error::ParseError(format!("key {} is not a number", key)))
}

fn json_get_bool(j: &Value, key: &str) -> crate::Result<bool> {
    json_at(j, key)?
        .as_bool()
        .ok_or_else(|| crate::Error::ParseError(format!("key {} is not a bool", key)))
}

fn json_get_str(j: &Value, key: &str) -> crate::Result<String> {
    json_at(j, key)?
        .as_str()
        .map(|s| s.to_string())
        .ok_or_else(|| crate::Error::ParseError(format!("key {} is not a string", key)))
}

fn json_get_array<'a>(j: &'a Value, key: &str) -> crate::Result<&'a Vec<Value>> {
    json_at(j, key)?
        .as_array()
        .ok_or_else(|| crate::Error::ParseError(format!("key {} is not an array", key)))
}

fn json_get_f64_vec(j: &Value, key: &str) -> crate::Result<Vec<f64>> {
    let mut out = Vec::new();
    for v in json_get_array(j, key)? {
        out.push(
            v.as_f64()
                .ok_or_else(|| crate::Error::ParseError(format!("key {} has non-number", key)))?,
        );
    }
    Ok(out)
}

fn json_get_i32_vec(j: &Value, key: &str) -> crate::Result<Vec<i32>> {
    let mut out = Vec::new();
    for v in json_get_array(j, key)? {
        out.push(
            v.as_i64()
                .ok_or_else(|| crate::Error::ParseError(format!("key {} has non-integer", key)))?
                as i32,
        );
    }
    Ok(out)
}

fn json_get_str_vec(j: &Value, key: &str) -> crate::Result<Vec<String>> {
    let mut out = Vec::new();
    for v in json_get_array(j, key)? {
        out.push(
            v.as_str()
                .map(|s| s.to_string())
                .ok_or_else(|| crate::Error::ParseError(format!("key {} has non-string", key)))?,
        );
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_thumbnail_data_new() {
        // ThumbnailData() { reset(); }
        let td = ThumbnailData::new();
        assert_eq!(td.width, 0);
        assert_eq!(td.height, 0);
        assert!(!td.is_valid());
    }

    #[test]
    fn test_thumbnail_data_set_white() {
        let mut td = ThumbnailData::new();
        td.set(50, 50);
        assert_eq!(td.width, 50);
        assert_eq!(td.height, 50);
        assert_eq!(td.pixels.len(), 4 * 50 * 50);
        // defaults to white texture (255)
        assert!(td.pixels.iter().all(|&b| b == 255));
        assert!(td.is_valid());
    }

    #[test]
    fn test_thumbnail_data_set_zero_dims_noop() {
        let mut td = ThumbnailData::new();
        td.set(0, 50);
        assert_eq!(td.width, 0);
        assert!(td.pixels.is_empty());
        td.set(50, 0);
        assert_eq!(td.height, 0);
        assert!(td.pixels.is_empty());
    }

    #[test]
    fn test_thumbnail_data_set_same_dims_no_realloc() {
        let mut td = ThumbnailData::new();
        td.set(16, 16);
        td.pixels[0] = 7;
        // same dims -> no reallocation, pixel left untouched
        td.set(16, 16);
        assert_eq!(td.pixels[0], 7);
    }

    #[test]
    fn test_thumbnail_data_reset() {
        let mut td = ThumbnailData::new();
        td.set(50, 50);
        td.reset();
        assert_eq!(td.width, 0);
        assert_eq!(td.height, 0);
        assert!(td.pixels.is_empty());
        assert!(!td.is_valid());
    }

    #[test]
    fn test_thumbnail_data_load_from() {
        let mut src = ThumbnailData::new();
        src.set(32, 32);
        src.pixels[0] = 200;

        let mut dst = ThumbnailData::new();
        dst.load_from(&src);
        assert_eq!(dst.width, 32);
        assert_eq!(dst.height, 32);
        assert_eq!(dst.pixels[0], 200);
    }

    #[test]
    fn test_bbox_data_roundtrip() {
        let bbox = BBoxData {
            id: 1,
            bbox: vec![0.0, 0.0, 10.0, 10.0],
            area: 100.0,
            layer_height: 0.2,
            name: "test_obj".into(),
        };
        let mut j = Value::Null;
        bbox.to_json(&mut j);
        assert_eq!(j["id"], json!(1));
        assert_eq!(j["name"], json!("test_obj"));

        let mut parsed = BBoxData::default();
        parsed.from_json(&j).unwrap();
        assert_eq!(parsed.id, 1);
        assert_eq!(parsed.bbox, vec![0.0, 0.0, 10.0, 10.0]);
        assert_eq!(parsed.name, "test_obj");
    }

    #[test]
    fn test_thumbnails_params_default() {
        let params = ThumbnailsParams::default();
        assert!(params.sizes.is_empty());
        assert!(params.use_plate_box);
        assert!(!params.post_processing_enabled);
        assert_eq!(params.background_color, [0.0, 0.0, 0.0, 0.0]);
    }

    #[test]
    fn test_thumbnail_size_default() {
        let sizes = thumbnail_size();
        assert_eq!(sizes.len(), 1);
        assert_eq!(sizes[0].x, 50.0);
        assert_eq!(sizes[0].y, 50.0);
    }

    #[test]
    fn test_plate_bbox_data_default() {
        let plate = PlateBBoxData::default();
        assert_eq!(plate.version, 2);
        assert_eq!(plate.nozzle_diameter, 0.4);
        assert!(!plate.is_valid());
    }

    #[test]
    fn test_plate_bbox_data_roundtrip() {
        let mut plate = PlateBBoxData::default();
        plate.bbox_all = vec![0.0, 0.0, 100.0, 100.0];
        plate.filament_ids = vec![1, 2];
        plate.filament_colors = vec!["#FF0000".into(), "#00FF00".into()];
        plate.is_seq_print = true;
        plate.first_extruder = 1;
        plate.nozzle_diameter = 0.4;
        plate.bed_type = "PEI".into();
        plate.first_layer_time = 12.5;
        plate.version = 2;
        plate.bbox_objs.push(BBoxData {
            id: 0,
            bbox: vec![1.0, 1.0, 9.0, 9.0],
            area: 64.0,
            layer_height: 0.2,
            name: "obj".into(),
        });

        let mut j = Value::Null;
        plate.to_json(&mut j);
        assert!(j["bbox_objects"].is_array());
        assert_eq!(j["bbox_objects"].as_array().unwrap().len(), 1);

        // from_json does NOT read first_layer_time back (matches C++ asymmetry).
        let mut parsed = PlateBBoxData::default();
        parsed.from_json(&j).unwrap();
        assert_eq!(parsed.bbox_all, vec![0.0, 0.0, 100.0, 100.0]);
        assert_eq!(parsed.filament_ids, vec![1, 2]);
        assert_eq!(parsed.is_seq_print, true);
        assert_eq!(parsed.first_extruder, 1);
        assert_eq!(parsed.bed_type, "PEI");
        assert_eq!(parsed.version, 2);
        assert_eq!(parsed.bbox_objs.len(), 1);
        assert_eq!(parsed.bbox_objs[0].name, "obj");
        assert!(parsed.is_valid());
    }
}
