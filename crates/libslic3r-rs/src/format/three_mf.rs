//! Faithful 1:1 port of `Format/3mf.cpp` + `Format/3mf.hpp` (BambuStudio).
//!
//! This is the legacy PrusaSlicer-style 3MF reader/writer kept by BambuStudio
//! for importing Prusa 3MF files (the BambuStudio-native format lives in
//! `Format/bbs_3mf.cpp`).
//!
//! Backend substitutions (wasm-safe, no native deps):
//! - expat (`XML_Parser`) -> `quick-xml` event reader driving the same
//!   start/end/characters handlers (see `format::amf` for the precedent).
//! - miniz (`mz_zip_archive`) -> the `zip` crate. `mz_zip_reader_locate_file`
//!   with flags=0 is case-INSENSITIVE; entry-name comparisons mirror that.
//! - `tdefl_write_image_to_png_file_in_memory_ex` -> `png_read_write::encode_png`
//!   (pure-Rust PNG encoder already in this crate).
//!
//! BLOCKED(model)/BLOCKED(config) notes: the crate's `Model`/`ModelObject` is a
//! simplified port (single merged `TriangleMesh` per object, no `ModelVolume`,
//! no `ModelInstance` full 3D transform, no `layer_height_profile` /
//! `layer_config_ranges` / `sla_support_points` fields) and the reflective
//! `DynamicPrintConfig` (`set_deserialize` / `keys()` / `opt_serialize`) is not
//! ported. Every side effect that cannot be expressed is kept as a
//! `BLOCKED(...)` comment at the exact C++ line, following the conventions
//! established in `format::amf`.

use std::collections::BTreeMap;
use std::io::{Read, Write as IoWrite};

use log::error;

use crate::calib::DynamicPrintConfig;
use crate::format::bbs_3mf::ConfigSubstitutionContext;
use crate::format::objparser;
use crate::gcode::thumbnail_data::ThumbnailData;
use crate::geometry::geometry::{transform3d_from_string, Transform3d, Transformation, Vec3d};
use crate::geometry::Point3F;
use crate::locales_utils::{general_format, is_decimal_separator_point, CNumericLocalesSetter};
use crate::model::{Model, ModelObject, ModelVolumeType, ObjectConfig};
use crate::normal_utils::{indexed_triangle_set, Vec3crd, Vec3f};
use crate::png_read_write::{encode_png, PNG_COLOR_TYPE_RGB_ALPHA};
use crate::semver::{Semver, SLIC3R_VERSION};
use crate::time::{get_current_time_utc, utc_timestamp};
use crate::triangle_mesh::{its_compactify_vertices, RepairedMeshErrors, Triangle, TriangleMesh};
use crate::utils::{header_slic3r_generated, xml_escape, SLIC3R_APP_NAME};
use crate::{Error, Result};

// libslic3r.h EPSILON (1e-4), used at 3mf.cpp:1234.
use crate::libslic3r::EPSILON;

// 3mf.cpp:43 — #define EXPORT_3MF_USE_SPIRIT_KARMA_FP 0
// (the karma fast-path is disabled in C++; only the sprintf("%.9g") path is
//  live and is what this port reproduces.)

// VERSION NUMBERS
// 0 : .3mf, files saved by older slic3r or other applications. No version definition in them.
// 1 : Introduction of 3mf versioning. No other change in data saved into 3mf files.
// 2 : Volumes' matrices and source data added to Metadata/Slic3r_PE_model.config file, meshes transformed back to their coordinate system on loading.
// WARNING !! -> the version number has been rolled back to 1
//               the next change should use 3
// 3mf.cpp:51
pub const VERSION_3MF: u32 = 1;
// Allow loading version 2 file as well.
// 3mf.cpp:53
pub const VERSION_3MF_COMPATIBLE: u32 = 2;
// definition of the metadata name saved into .model file
// 3mf.cpp:54
pub const SLIC3RPE_3MF_VERSION: &str = "slic3rpe:Version3mf";

// Painting gizmos data version numbers
// 0 : 3MF files saved by older PrusaSlicer or the painting gizmo wasn't used. No version definition in them.
// 1 : Introduction of painting gizmos data versioning. No other changes in painting gizmos data.
// 3mf.cpp:59
pub const FDM_SUPPORTS_PAINTING_VERSION: u32 = 1;
// 3mf.cpp:60
pub const SEAM_PAINTING_VERSION: u32 = 1;
// 3mf.cpp:61
pub const MM_PAINTING_VERSION: u32 = 1;

// 3mf.cpp:63
pub const SLIC3RPE_FDM_SUPPORTS_PAINTING_VERSION: &str = "slic3rpe:FdmSupportsPaintingVersion";
// 3mf.cpp:64
pub const SLIC3RPE_SEAM_PAINTING_VERSION: &str = "slic3rpe:SeamPaintingVersion";
// 3mf.cpp:65
pub const SLIC3RPE_MM_PAINTING_VERSION: &str = "slic3rpe:MmPaintingVersion";

// 3mf.cpp:67
const MODEL_FOLDER: &str = "3D/";
// 3mf.cpp:68
const MODEL_EXTENSION: &str = ".model";
// 3mf.cpp:69 — << this is the only format of the string which works with CURA
const MODEL_FILE: &str = "3D/3dmodel.model";
// 3mf.cpp:70
const CONTENT_TYPES_FILE: &str = "[Content_Types].xml";
// 3mf.cpp:71
const RELATIONSHIPS_FILE: &str = "_rels/.rels";
// 3mf.cpp:72
const THUMBNAIL_FILE: &str = "Metadata/thumbnail.png";
// 3mf.cpp:73
const PRINT_CONFIG_FILE: &str = "Metadata/Slic3r_PE.config";
// 3mf.cpp:74
const MODEL_CONFIG_FILE: &str = "Metadata/Slic3r_PE_model.config";
// 3mf.cpp:75
const LAYER_HEIGHTS_PROFILE_FILE: &str = "Metadata/Slic3r_PE_layer_heights_profile.txt";
// 3mf.cpp:76
const LAYER_CONFIG_RANGES_FILE: &str = "Metadata/Prusa_Slicer_layer_config_ranges.xml";
// 3mf.cpp:77
const SLA_SUPPORT_POINTS_FILE: &str = "Metadata/Slic3r_PE_sla_support_points.txt";
// 3mf.cpp:78
const SLA_DRAIN_HOLES_FILE: &str = "Metadata/Slic3r_PE_sla_drain_holes.txt";
// 3mf.cpp:79
#[allow(dead_code)]
const CUSTOM_GCODE_PER_PRINT_Z_FILE: &str = "Metadata/Prusa_Slicer_custom_gcode_per_print_z.xml";

// 3mf.cpp:81-93
const MODEL_TAG: &str = "model";
const RESOURCES_TAG: &str = "resources";
const OBJECT_TAG: &str = "object";
const MESH_TAG: &str = "mesh";
const VERTICES_TAG: &str = "vertices";
const VERTEX_TAG: &str = "vertex";
const TRIANGLES_TAG: &str = "triangles";
const TRIANGLE_TAG: &str = "triangle";
const COMPONENTS_TAG: &str = "components";
const COMPONENT_TAG: &str = "component";
const BUILD_TAG: &str = "build";
const ITEM_TAG: &str = "item";
const METADATA_TAG: &str = "metadata";

// 3mf.cpp:95-96
const CONFIG_TAG: &str = "config";
const VOLUME_TAG: &str = "volume";

// 3mf.cpp:98-114
const UNIT_ATTR: &str = "unit";
const NAME_ATTR: &str = "name";
const TYPE_ATTR: &str = "type";
const ID_ATTR: &str = "id";
const X_ATTR: &str = "x";
const Y_ATTR: &str = "y";
const Z_ATTR: &str = "z";
const V1_ATTR: &str = "v1";
const V2_ATTR: &str = "v2";
const V3_ATTR: &str = "v3";
const OBJECTID_ATTR: &str = "objectid";
const TRANSFORM_ATTR: &str = "transform";
const PRINTABLE_ATTR: &str = "printable";
const INSTANCESCOUNT_ATTR: &str = "instances_count";
const CUSTOM_SUPPORTS_ATTR: &str = "slic3rpe:custom_supports";
const CUSTOM_SEAM_ATTR: &str = "slic3rpe:custom_seam";
const MMU_SEGMENTATION_ATTR: &str = "slic3rpe:mmu_segmentation";

// 3mf.cpp:116-119
const KEY_ATTR: &str = "key";
const VALUE_ATTR: &str = "value";
const FIRST_TRIANGLE_ID_ATTR: &str = "firstid";
const LAST_TRIANGLE_ID_ATTR: &str = "lastid";

// 3mf.cpp:121-122
const OBJECT_TYPE: &str = "object";
const VOLUME_TYPE: &str = "volume";

// 3mf.cpp:124-135
const NAME_KEY: &str = "name";
const MODIFIER_KEY: &str = "modifier";
const VOLUME_TYPE_KEY: &str = "volume_type";
const MATRIX_KEY: &str = "matrix";
const SOURCE_FILE_KEY: &str = "source_file";
const SOURCE_OBJECT_ID_KEY: &str = "source_object_id";
const SOURCE_VOLUME_ID_KEY: &str = "source_volume_id";
const SOURCE_OFFSET_X_KEY: &str = "source_offset_x";
const SOURCE_OFFSET_Y_KEY: &str = "source_offset_y";
const SOURCE_OFFSET_Z_KEY: &str = "source_offset_z";
const SOURCE_IN_INCHES: &str = "source_in_inches";
const SOURCE_IN_METERS: &str = "source_in_meters";

// 3mf.cpp:137-141
const MESH_STAT_EDGES_FIXED: &str = "edges_fixed";
const MESH_STAT_DEGENERATED_FACETS: &str = "degenerate_facets";
const MESH_STAT_FACETS_REMOVED: &str = "facets_removed";
const MESH_STAT_FACETS_RESERVED: &str = "facets_reversed";
const MESH_STAT_BACKWARDS_EDGES: &str = "backwards_edges";

// 3mf.cpp:144
const VALID_OBJECT_TYPES_COUNT: usize = 1;
// 3mf.cpp:145-148
const VALID_OBJECT_TYPES: [&str; 1] = ["model"];

// 3mf.cpp:150-156 — defined in C++ but unused there as well.
#[allow(dead_code)]
const INVALID_OBJECT_TYPES: [&str; 4] = ["solidsupport", "support", "surface", "other"];

/// 3mf.cpp:158-163
/// C++: `class version_error : public Slic3r::FileIOError`
/// Thrown by `_handle_end_metadata` (3mf.cpp:1849/1858/1862/1866) and rethrown
/// as `Slic3r::FileIOError` at 3mf.cpp:1000-1004. Modelled as a Rust error
/// value carried out of the XML dispatch (exceptions -> `Result`).
#[derive(Debug, Clone)]
struct VersionError(String);

// ---------------------------------------------------------------------------
// C runtime helpers (atoi/atof used throughout 3mf.cpp)
// ---------------------------------------------------------------------------

/// C `atof(nptr)` == `strtod(nptr, NULL)` (same helper as `format::amf`).
fn atof(s: &str) -> f64 {
    objparser::strtod(s.as_bytes(), 0).0
}

/// C `atoi(nptr)`: skip leading C whitespace, optional sign, longest run of
/// decimal digits; `0` when no conversion is performed (same as `format::amf`).
fn atoi(s: &str) -> i32 {
    let bytes = s.as_bytes();
    let n = bytes.len();
    let mut i = 0;
    while i < n && matches!(bytes[i], b' ' | b'\t' | b'\n' | 0x0B | 0x0C | b'\r') {
        i += 1;
    }
    let mut negative = false;
    if i < n && (bytes[i] == b'+' || bytes[i] == b'-') {
        negative = bytes[i] == b'-';
        i += 1;
    }
    let mut value: i64 = 0;
    let mut any = false;
    while i < n && bytes[i].is_ascii_digit() {
        any = true;
        value = value
            .saturating_mul(10)
            .saturating_add((bytes[i] - b'0') as i64);
        i += 1;
    }
    if !any {
        return 0;
    }
    let value = if negative { -value } else { value };
    value.clamp(i32::MIN as i64, i32::MAX as i64) as i32
}

/// boost::istarts_with — case-insensitive ASCII prefix test (3mf.cpp:744).
fn istarts_with(haystack: &str, prefix: &str) -> bool {
    let h = haystack.as_bytes();
    let p = prefix.as_bytes();
    h.len() >= p.len()
        && h[..p.len()]
            .iter()
            .zip(p)
            .all(|(a, b)| a.eq_ignore_ascii_case(b))
}

/// boost::iends_with — case-insensitive ASCII suffix test (3mf.cpp:744).
fn iends_with(haystack: &str, suffix: &str) -> bool {
    let h = haystack.as_bytes();
    let s = suffix.as_bytes();
    h.len() >= s.len()
        && h[h.len() - s.len()..]
            .iter()
            .zip(s)
            .all(|(a, b)| a.eq_ignore_ascii_case(b))
}

/// boost::iequals — case-insensitive ASCII equality (3mf.cpp:797).
fn iequals(a: &str, b: &str) -> bool {
    a.len() == b.len() && istarts_with(a, b)
}

/// Eigen `Transform3d::isApprox(other, prec)`:
/// `(a - b).norm() <= prec * min(a.norm(), b.norm())` (Frobenius norm).
/// Used by the volume-transform check at 3mf.cpp:2093 (same helper as amf.rs).
fn transform_is_approx(a: &Transform3d, b: &Transform3d, prec: f64) -> bool {
    (a - b).norm() <= prec * a.norm().min(b.norm())
}

// ---------------------------------------------------------------------------
// XML attribute helpers (3mf.cpp:165-204)
// ---------------------------------------------------------------------------
// expat hands attributes as a flat `const char**` array of key/value pairs with
// `attributes_size == 2 * count`; the Rust drivers collect `(key, value)`
// pairs, so the `attributes_size % 2 != 0` guard is unrepresentable here.

/// 3mf.cpp:165-176
/// C++: `const char* get_attribute_value_charptr(const char** attributes, unsigned int attributes_size, const char* attribute_key)`
fn get_attribute_value_charptr<'a>(
    attributes: &'a [(String, String)],
    attribute_key: &str,
) -> Option<&'a str> {
    // 3mf.cpp:167-168
    if attributes.is_empty() {
        return None;
    }
    // 3mf.cpp:170-173
    for (key, value) in attributes {
        if key == attribute_key {
            return Some(value.as_str());
        }
    }
    // 3mf.cpp:175
    None
}

/// 3mf.cpp:178-182
/// C++: `std::string get_attribute_value_string(...)`
fn get_attribute_value_string(attributes: &[(String, String)], attribute_key: &str) -> String {
    // 3mf.cpp:180-181
    get_attribute_value_charptr(attributes, attribute_key)
        .unwrap_or("")
        .to_string()
}

/// 3mf.cpp:184-190
/// C++: `float get_attribute_value_float(...)` —
/// `fast_float::from_chars(text, text + strlen(text), value)`; on parse failure
/// the value stays 0.0f. (fast_float does not skip leading whitespace; the
/// strtod-based helper here does — XML attribute values in practice carry no
/// leading whitespace.)
fn get_attribute_value_float(attributes: &[(String, String)], attribute_key: &str) -> f32 {
    // 3mf.cpp:186
    let mut value = 0.0f32;
    // 3mf.cpp:187-188
    if let Some(text) = get_attribute_value_charptr(attributes, attribute_key) {
        value = atof(text) as f32;
    }
    // 3mf.cpp:189
    value
}

/// 3mf.cpp:192-198
/// C++: `int get_attribute_value_int(...)` —
/// `boost::spirit::qi::parse(text, text + strlen(text), qi::int_, value)`;
/// on parse failure the value stays 0.
fn get_attribute_value_int(attributes: &[(String, String)], attribute_key: &str) -> i32 {
    // 3mf.cpp:194
    let mut value = 0i32;
    // 3mf.cpp:195-196 — qi::int_ parses [+-]?digits from the start (no
    // whitespace skipping with plain qi::parse).
    if let Some(text) = get_attribute_value_charptr(attributes, attribute_key) {
        let bytes = text.as_bytes();
        let mut i = 0usize;
        let mut negative = false;
        if i < bytes.len() && (bytes[i] == b'+' || bytes[i] == b'-') {
            negative = bytes[i] == b'-';
            i += 1;
        }
        let mut parsed: i64 = 0;
        let mut any = false;
        while i < bytes.len() && bytes[i].is_ascii_digit() {
            any = true;
            parsed = parsed
                .saturating_mul(10)
                .saturating_add((bytes[i] - b'0') as i64);
            i += 1;
        }
        if any {
            let parsed = if negative { -parsed } else { parsed };
            value = parsed.clamp(i32::MIN as i64, i32::MAX as i64) as i32;
        }
    }
    // 3mf.cpp:197
    value
}

/// 3mf.cpp:200-204
/// C++: `bool get_attribute_value_bool(...)` —
/// `(text != nullptr) ? (bool)::atoi(text) : true`
fn get_attribute_value_bool(attributes: &[(String, String)], attribute_key: &str) -> bool {
    // 3mf.cpp:202-203
    match get_attribute_value_charptr(attributes, attribute_key) {
        Some(text) => atoi(text) != 0,
        None => true,
    }
}

/// 3mf.cpp:206-233
/// C++: `Slic3r::Transform3d get_transform_from_3mf_specs_string(const std::string& mat_str)`
pub fn get_transform_from_3mf_specs_string(mat_str: &str) -> Transform3d {
    // check: https://3mf.io/3d-manufacturing-format/ or https://github.com/3MFConsortium/spec_core/blob/master/3MF%20Core%20Specification.md
    // to see how matrices are stored inside 3mf according to specifications
    // 3mf.cpp:210
    let mut ret = Transform3d::identity();

    // 3mf.cpp:212-214 — empty string means default identity matrix
    if mat_str.is_empty() {
        return ret;
    }

    // 3mf.cpp:216-217 — boost::split(..., is_any_of(" "), token_compress_on)
    let mat_elements_str: Vec<&str> = mat_str.split(' ').filter(|s| !s.is_empty()).collect();

    // 3mf.cpp:219-222 — invalid data, return identity matrix
    let size = mat_elements_str.len();
    if size != 12 {
        return ret;
    }

    // 3mf.cpp:224-231 — matrices are stored into 3mf files as 4x3,
    // we need to transpose them
    let mut i = 0usize;
    for c in 0..4 {
        for r in 0..3 {
            ret[(r, c)] = atof(mat_elements_str[i]);
            i += 1;
        }
    }
    ret
}

/// 3mf.cpp:235-252
/// C++: `float get_unit_factor(const std::string& unit)`
pub fn get_unit_factor(unit: &str) -> f32 {
    // 3mf.cpp:237-251
    if unit == "micron" {
        0.001f32
    } else if unit == "centimeter" {
        10.0f32
    } else if unit == "inch" {
        25.4f32
    } else if unit == "foot" {
        304.8f32
    } else if unit == "meter" {
        1000.0f32
    } else {
        // default "millimeters" (see specification)
        1.0f32
    }
}

/// 3mf.cpp:254-266
/// C++: `bool is_valid_object_type(const std::string& type)`
pub fn is_valid_object_type(type_: &str) -> bool {
    // if the type is empty defaults to "model" (see specification)
    // 3mf.cpp:257-258
    if type_.is_empty() {
        return true;
    }

    // 3mf.cpp:260-263
    for i in 0..VALID_OBJECT_TYPES_COUNT {
        if type_ == VALID_OBJECT_TYPES[i] {
            return true;
        }
    }

    // 3mf.cpp:265
    false
}

// 3mf.cpp:270-273
//   //! macro used to mark string used at localization, return same string
//   #define L(s) (s)
//   #define _(s) Slic3r::I18N::translate(s)
// I18N::translate is an identity passthrough in this crate's context.

// ---------------------------------------------------------------------------
// PrusaFileParser (3mf.hpp:7-27, 3mf.cpp:274-358)
// ---------------------------------------------------------------------------

/// PrusaFileParser is used to check 3mf file is from Prusa
/// 3mf.hpp:7-27
pub struct PrusaFileParser {
    /// 3mf.hpp:24
    m_from_prusa: bool,
    /// 3mf.hpp:25
    m_is_application_key: bool,
    // 3mf.hpp:26 — XML_Parser m_parser: the expat handle has no Rust
    // counterpart; `parse_prusa_buffer` drives the handlers below.
}

impl PrusaFileParser {
    /// 3mf.hpp:10 — `PrusaFileParser() {}`
    pub fn new() -> Self {
        Self {
            m_from_prusa: false,
            m_is_application_key: false,
        }
    }

    // 3mf.cpp:274-284 — the static `start_element_handler` / `characters_handler`
    // expat trampolines (userData -> this) are realized by the dispatch loop in
    // `parse_prusa_buffer`.

    /// 3mf.cpp:286-323
    /// C++: `bool PrusaFileParser::check_3mf_from_prusa(const std::string filename)`
    pub fn check_3mf_from_prusa(&mut self, filename: &str) -> bool {
        // 3mf.cpp:288-293 — mz_zip_zero_struct / open_zip_reader
        let file = match std::fs::File::open(filename) {
            Ok(f) => f,
            // throw Slic3r::RuntimeError("Loading 3mf file failed.");
            Err(_) => return false,
        };
        let mut archive = match zip::ZipArchive::new(file) {
            Ok(a) => a,
            Err(_) => return false,
        };

        // 3mf.cpp:295-296 — mz_zip_reader_locate_file(..., 0) is case-insensitive
        let sub_relationship_file = "3D/_rels/3dmodel.model.rels";
        let sub_index = locate_file(&mut archive, sub_relationship_file);
        // 3mf.cpp:297-318
        if sub_index.is_none() {
            let model_file = "3D/3dmodel.model";
            let model_file_index = locate_file(&mut archive, model_file);
            if let Some(model_file_index) = model_file_index {
                // 3mf.cpp:301-305 — XML_ParserCreate / handler setup
                // 3mf.cpp:307-314 — stat + extract to parser buffer
                let mut data = Vec::new();
                let read_ok = match archive.by_index(model_file_index) {
                    Ok(mut f) => f.read_to_end(&mut data).is_ok(),
                    Err(_) => false,
                };
                if read_ok {
                    // 3mf.cpp:316 — XML_ParseBuffer(..., 1)
                    self.parse_prusa_buffer(&data);
                }
                // goto EXIT on any failure above (close + return m_from_prusa)
            }
        }

        // 3mf.cpp:320-322 — EXIT: close_zip_reader (RAII) + return
        self.m_from_prusa
    }

    /// 3mf.cpp:325-331
    /// C++: `void PrusaFileParser::_characters_handler(const XML_Char *s, int len)`
    pub fn _characters_handler(&mut self, s: &str) {
        if self.m_is_application_key {
            // 3mf.cpp:328-329
            let str_ = s.to_string();
            if !str_.is_empty() && str_.contains("PrusaSlicer") {
                self.m_from_prusa = true;
            }
        }
        // NOTE: C++ never resets m_is_application_key — replicated here.
    }

    /// 3mf.cpp:333-341
    /// C++: `void PrusaFileParser::_start_element_handler(const char *name, const char **attributes)`
    pub fn _start_element_handler(&mut self, name: &str, attributes: &[(String, String)]) {
        if name == "metadata" {
            // 3mf.cpp:336 — XML_GetSpecifiedAttributeCount
            // 3mf.cpp:338-339
            let str_name = Self::get_attribute_value_string(attributes, "name");
            if !str_name.is_empty() && str_name.contains("Application") {
                self.m_is_application_key = true;
            }
        }
    }

    /// 3mf.cpp:343-352
    /// C++: `const char *PrusaFileParser::get_attribute_value_charptr(...)` —
    /// identical to the free function (3mf.cpp:165-176).
    fn get_attribute_value_charptr<'a>(
        attributes: &'a [(String, String)],
        attribute_key: &str,
    ) -> Option<&'a str> {
        // 3mf.cpp:345
        if attributes.is_empty() {
            return None;
        }
        // 3mf.cpp:347-349
        for (key, value) in attributes {
            if key == attribute_key {
                return Some(value.as_str());
            }
        }
        // 3mf.cpp:351
        None
    }

    /// 3mf.cpp:354-358
    /// C++: `std::string PrusaFileParser::get_attribute_value_string(...)`
    fn get_attribute_value_string(attributes: &[(String, String)], attribute_key: &str) -> String {
        // 3mf.cpp:356-357
        Self::get_attribute_value_charptr(attributes, attribute_key)
            .unwrap_or("")
            .to_string()
    }

    /// expat driver for `check_3mf_from_prusa`: dispatches start-element and
    /// character-data events (3mf.cpp:302-305 registers only those handlers).
    fn parse_prusa_buffer(&mut self, data: &[u8]) {
        use quick_xml::events::Event;
        use quick_xml::reader::Reader;

        let text = String::from_utf8_lossy(data);
        let mut reader = Reader::from_str(&text);
        loop {
            match reader.read_event() {
                Ok(Event::Start(e)) | Ok(Event::Empty(e)) => {
                    let name = String::from_utf8_lossy(e.name().as_ref()).into_owned();
                    let atts = collect_attributes(&e);
                    self._start_element_handler(&name, &atts);
                }
                Ok(Event::Text(e)) => {
                    let s = e
                        .unescape()
                        .map(|v| v.into_owned())
                        .unwrap_or_else(|_| String::from_utf8_lossy(&e).into_owned());
                    self._characters_handler(&s);
                }
                Ok(Event::CData(e)) => {
                    let s = String::from_utf8_lossy(&e.into_inner()).into_owned();
                    self._characters_handler(&s);
                }
                Ok(Event::Eof) | Err(_) => break,
                _ => {}
            }
        }
    }
}

impl Default for PrusaFileParser {
    fn default() -> Self {
        Self::new()
    }
}

/// 3mf.cpp:360-372
/// C++: `ModelVolumeType type_from_string(const std::string &s)`
pub fn type_from_string(s: &str) -> ModelVolumeType {
    // Legacy support
    // 3mf.cpp:363
    if s == "1" {
        return ModelVolumeType::ParameterModifier;
    }
    // New type (supporting the support enforcers & blockers)
    // 3mf.cpp:365
    if s == "ModelPart" {
        return ModelVolumeType::ModelPart;
    }
    // 3mf.cpp:366
    if s == "NegativeVolume" {
        return ModelVolumeType::NegativeVolume;
    }
    // 3mf.cpp:367
    if s == "ParameterModifier" {
        return ModelVolumeType::ParameterModifier;
    }
    // 3mf.cpp:368
    if s == "SupportEnforcer" {
        return ModelVolumeType::SupportEnforcer;
    }
    // 3mf.cpp:369
    if s == "SupportBlocker" {
        return ModelVolumeType::SupportBlocker;
    }
    // Default value if invalud type string received.
    // 3mf.cpp:371
    ModelVolumeType::ModelPart
}

// ---------------------------------------------------------------------------
// zip / expat backend helpers
// ---------------------------------------------------------------------------

/// `mz_zip_reader_locate_file(&archive, name, nullptr, 0)` — flags=0 means
/// case-insensitive comparison in miniz; returns the entry index.
fn locate_file(archive: &mut zip::ZipArchive<std::fs::File>, name: &str) -> Option<usize> {
    for i in 0..archive.len() {
        if let Ok(f) = archive.by_index(i) {
            if iequals(f.name(), name) {
                return Some(i);
            }
        }
    }
    None
}

/// Collect a start tag's attributes as (name, value) pairs with entities
/// decoded — matching what expat hands to the start-element handlers.
fn collect_attributes(e: &quick_xml::events::BytesStart) -> Vec<(String, String)> {
    let mut atts = Vec::new();
    for attr in e.attributes().flatten() {
        let key = String::from_utf8_lossy(attr.key.as_ref()).into_owned();
        let value = attr
            .unescape_value()
            .map(|v| v.into_owned())
            .unwrap_or_else(|_| String::from_utf8_lossy(&attr.value).into_owned());
        atts.push((key, value));
    }
    atts
}

/// `XML_GetCurrentLineNumber` equivalent: 1-based line number at byte offset
/// `pos` (3mf.cpp:993, 3mf.cpp:1360).
fn line_at(text: &str, pos: usize) -> usize {
    let pos = pos.min(text.len());
    text.as_bytes()[..pos]
        .iter()
        .filter(|&&b| b == b'\n')
        .count()
        + 1
}

/// `mz_zip_writer_add_mem(&archive, name, data, len, MZ_DEFAULT_COMPRESSION)`
/// over the `zip` crate writer.
fn mz_zip_writer_add_mem(
    archive: &mut zip::ZipWriter<std::fs::File>,
    name: &str,
    data: &[u8],
) -> bool {
    let options =
        zip::write::FileOptions::default().compression_method(zip::CompressionMethod::Deflated);
    archive.start_file(name, options).is_ok() && archive.write_all(data).is_ok()
}

// ---------------------------------------------------------------------------
// Local faithful data carriers for the SLA archive records.
// The crate's `sla` module types (`sla::SupportPoint`, `sla::DrainHole`) are
// unported placeholders, so the fields the 3MF archive carries are mirrored
// here 1:1 until those land (SLA/SupportPoint.hpp:21-46, SLA/Hollowing.hpp:38-60).
// ---------------------------------------------------------------------------

/// `sla::SupportPoint` — `pos` (Vec3f), `head_front_radius`, `is_new_island`.
#[derive(Debug, Clone, PartialEq)]
pub struct SlaSupportPoint {
    pub pos: Vec3f,
    pub head_front_radius: f32,
    pub is_new_island: bool,
}

/// `sla::DrainHole` — `pos`, `normal` (Vec3f), `radius`, `height`.
#[derive(Debug, Clone, PartialEq)]
pub struct SlaDrainHole {
    pub pos: Vec3f,
    pub normal: Vec3f,
    pub radius: f32,
    pub height: f32,
}

/// `t_layer_config_ranges` == `std::map<std::pair<coordf_t, coordf_t>, ModelConfig>`
/// (Model.hpp). BLOCKED(config): `ModelConfig` / `DynamicPrintConfig::set_deserialize`
/// are unported; the raw `(opt_key, serialized value)` pairs read from the
/// archive are retained losslessly per (min_z, max_z) range key.
type TLayerConfigRanges = Vec<((f64, f64), Vec<(String, String)>)>;

// ---------------------------------------------------------------------------
// _3MF_Base (3mf.cpp:374-389)
// ---------------------------------------------------------------------------

/// Base class with error messages management
/// 3mf.cpp:375-389
#[allow(non_camel_case_types)]
struct _3MF_Base {
    /// 3mf.cpp:377
    m_errors: Vec<String>,
}

impl _3MF_Base {
    fn new() -> Self {
        Self {
            m_errors: Vec::new(),
        }
    }

    /// 3mf.cpp:380
    fn add_error(&mut self, error: impl Into<String>) {
        self.m_errors.push(error.into());
    }

    /// 3mf.cpp:381
    fn clear_errors(&mut self) {
        self.m_errors.clear();
    }

    /// 3mf.cpp:384-388
    fn log_errors(&self) {
        for error_msg in &self.m_errors {
            // BOOST_LOG_TRIVIAL(error) << error;
            error!("{}", error_msg);
        }
    }
}

// ---------------------------------------------------------------------------
// _3MF_Importer (3mf.cpp:391-658)
// ---------------------------------------------------------------------------

/// 3mf.cpp:393-409
#[derive(Debug, Clone)]
struct Component {
    /// 3mf.cpp:395
    object_id: i32,
    /// 3mf.cpp:396
    transform: Transform3d,
}

impl Component {
    /// 3mf.cpp:398-402 — `explicit Component(int object_id)`
    fn from_id(object_id: i32) -> Self {
        Self {
            object_id,
            transform: Transform3d::identity(),
        }
    }

    /// 3mf.cpp:404-408
    fn new(object_id: i32, transform: Transform3d) -> Self {
        Self {
            object_id,
            transform,
        }
    }
}

/// 3mf.cpp:411 — `typedef std::vector<Component> ComponentsList;`
type ComponentsList = Vec<Component>;

/// 3mf.cpp:413-430
#[derive(Debug, Clone, Default)]
struct Geometry {
    /// 3mf.cpp:415
    vertices: Vec<Vec3f>,
    /// 3mf.cpp:416
    triangles: Vec<Vec3crd>,
    /// 3mf.cpp:417
    custom_supports: Vec<String>,
    /// 3mf.cpp:418
    custom_seam: Vec<String>,
    /// 3mf.cpp:419
    mmu_segmentation: Vec<String>,
}

impl Geometry {
    /// 3mf.cpp:421
    fn empty(&self) -> bool {
        self.vertices.is_empty() || self.triangles.is_empty()
    }

    /// 3mf.cpp:423-429
    fn reset(&mut self) {
        self.vertices.clear();
        self.triangles.clear();
        self.custom_supports.clear();
        self.custom_seam.clear();
        self.mmu_segmentation.clear();
    }
}

/// 3mf.cpp:432-451
struct CurrentObject {
    // ID of the object inside the 3MF file, 1 based.
    /// 3mf.cpp:435
    id: i32,
    // Index of the ModelObject in its respective Model, zero based.
    /// 3mf.cpp:437
    model_object_idx: i32,
    /// 3mf.cpp:438
    geometry: Geometry,
    /// 3mf.cpp:439 — `ModelObject* object;` the Rust port stores the index
    /// into `model.objects` (None == nullptr).
    object: Option<usize>,
    /// 3mf.cpp:440
    components: ComponentsList,
}

impl CurrentObject {
    /// 3mf.cpp:442 — `CurrentObject() { reset(); }`
    fn new() -> Self {
        Self {
            id: -1,
            model_object_idx: -1,
            geometry: Geometry::default(),
            object: None,
            components: Vec::new(),
        }
    }

    /// 3mf.cpp:444-450
    fn reset(&mut self) {
        self.id = -1;
        self.model_object_idx = -1;
        self.geometry.reset();
        self.object = None;
        self.components.clear();
    }
}

/// 3mf.cpp:453-457
struct CurrentConfig {
    /// 3mf.cpp:455
    object_id: i32,
    /// 3mf.cpp:456
    volume_id: i32,
}

/// 3mf.cpp:459-469 — `struct Instance { ModelInstance* instance; Transform3d transform; }`
/// The Rust port stores `(object index, instance index)` in place of the
/// `ModelInstance*`; `_handle_end_model` keeps the indices pointer-stable
/// across `Model::delete_object` (see there).
struct Instance {
    /// 3mf.cpp:461 — ModelInstance* (object index part)
    object_idx: usize,
    /// 3mf.cpp:461 — ModelInstance* (instance index part)
    instance_idx: usize,
    /// 3mf.cpp:462
    transform: Transform3d,
}

impl Instance {
    /// 3mf.cpp:464-468
    fn new(object_idx: usize, instance_idx: usize, transform: Transform3d) -> Self {
        Self {
            object_idx,
            instance_idx,
            transform,
        }
    }
}

/// 3mf.cpp:471-481
#[derive(Debug, Clone)]
struct Metadata {
    /// 3mf.cpp:473
    key: String,
    /// 3mf.cpp:474
    value: String,
}

impl Metadata {
    /// 3mf.cpp:476-480
    fn new(key: String, value: String) -> Self {
        Self { key, value }
    }
}

/// 3mf.cpp:483 — `typedef std::vector<Metadata> MetadataList;`
type MetadataList = Vec<Metadata>;

/// 3mf.cpp:485-505
#[derive(Debug, Clone, Default)]
struct ObjectMetadata {
    /// 3mf.cpp:503
    metadata: MetadataList,
    /// 3mf.cpp:504
    volumes: VolumeMetadataList,
}

/// 3mf.cpp:487-499
#[derive(Debug, Clone)]
struct VolumeMetadata {
    /// 3mf.cpp:489
    first_triangle_id: u32,
    /// 3mf.cpp:490
    last_triangle_id: u32,
    /// 3mf.cpp:491
    metadata: MetadataList,
    /// 3mf.cpp:492
    mesh_stats: RepairedMeshErrors,
}

impl VolumeMetadata {
    /// 3mf.cpp:494-498
    fn new(first_triangle_id: u32, last_triangle_id: u32) -> Self {
        Self {
            first_triangle_id,
            last_triangle_id,
            metadata: Vec::new(),
            mesh_stats: RepairedMeshErrors::default(),
        }
    }
}

/// 3mf.cpp:501 — `typedef std::vector<VolumeMetadata> VolumeMetadataList;`
type VolumeMetadataList = Vec<VolumeMetadata>;

// Map from a 1 based 3MF object ID to a 0 based ModelObject index inside m_model->objects.
// 3mf.cpp:508 — typedef std::map<int, int> IdToModelObjectMap;        (BTreeMap == ordered std::map)
// 3mf.cpp:509 — typedef std::map<int, ComponentsList> IdToAliasesMap;
// 3mf.cpp:510 — typedef std::vector<Instance> InstancesList;
// 3mf.cpp:511 — typedef std::map<int, ObjectMetadata> IdToMetadataMap;
// 3mf.cpp:512 — typedef std::map<int, Geometry> IdToGeometryMap;
// 3mf.cpp:513 — typedef std::map<int, std::vector<coordf_t>> IdToLayerHeightsProfileMap;
// 3mf.cpp:514 — typedef std::map<int, t_layer_config_ranges> IdToLayerConfigRangesMap;
// 3mf.cpp:515 — typedef std::map<int, std::vector<sla::SupportPoint>> IdToSlaSupportPointsMap;
// 3mf.cpp:516 — typedef std::map<int, std::vector<sla::DrainHole>> IdToSlaDrainHolesMap;

/// 3mf.cpp:391-658 — `class _3MF_Importer : public _3MF_Base`
#[allow(non_camel_case_types)]
pub struct _3MF_Importer<'a> {
    /// _3MF_Base (3mf.cpp:391)
    base: _3MF_Base,

    // Version of the 3mf file
    /// 3mf.cpp:519
    m_version: u32,
    /// 3mf.cpp:520
    m_check_version: bool,

    // Semantic version of PrusaSlicer, that generated this 3MF.
    /// 3mf.cpp:523
    m_prusaslicer_generator_version: Option<Semver>,
    /// 3mf.cpp:524
    m_fdm_supports_painting_version: u32,
    /// 3mf.cpp:525
    m_seam_painting_version: u32,
    /// 3mf.cpp:526
    m_mm_painting_version: u32,

    /// 3mf.cpp:528 — `XML_Parser m_xml_parser;` modelled as "parser exists"
    /// (the C++ dispatchers bail out when the handle is null).
    m_xml_parser: bool,
    // Error code returned by the application side of the parser. In that case the expat may not reliably deliver the error state
    // after returning from XML_Parse() function, thus we keep the error state here.
    /// 3mf.cpp:531
    m_parse_error: bool,
    /// 3mf.cpp:532
    m_parse_error_message: String,
    /// 3mf.cpp:533
    m_model: &'a mut Model,
    /// 3mf.cpp:534
    m_unit_factor: f32,
    /// 3mf.cpp:535
    m_curr_object: CurrentObject,
    /// 3mf.cpp:536
    m_objects: BTreeMap<i32, i32>,
    /// 3mf.cpp:537
    m_objects_aliases: BTreeMap<i32, ComponentsList>,
    /// 3mf.cpp:538
    m_instances: Vec<Instance>,
    /// 3mf.cpp:539
    m_geometries: BTreeMap<i32, Geometry>,
    /// 3mf.cpp:540
    m_curr_config: CurrentConfig,
    /// 3mf.cpp:541
    m_objects_metadata: BTreeMap<i32, ObjectMetadata>,
    /// 3mf.cpp:542
    m_layer_heights_profiles: BTreeMap<i32, Vec<f64>>,
    /// 3mf.cpp:543
    m_layer_config_ranges: BTreeMap<i32, TLayerConfigRanges>,
    /// 3mf.cpp:544
    m_sla_support_points: BTreeMap<i32, Vec<SlaSupportPoint>>,
    /// 3mf.cpp:545
    m_sla_drain_holes: BTreeMap<i32, Vec<SlaDrainHole>>,
    /// 3mf.cpp:546
    m_curr_metadata_name: String,
    /// 3mf.cpp:547
    m_curr_characters: String,
    /// 3mf.cpp:548
    m_name: String,
}

impl<'a> _3MF_Importer<'a> {
    /// 3mf.cpp:660-670 — `_3MF_Importer::_3MF_Importer()`
    /// (C++ binds `m_model` inside `load_model_from_file`; Rust takes the
    /// `&mut Model` at construction for lifetime reasons.)
    pub fn new(model: &'a mut Model) -> Self {
        Self {
            base: _3MF_Base::new(),
            m_version: 0,
            m_check_version: false,
            m_prusaslicer_generator_version: None,
            m_fdm_supports_painting_version: 0,
            m_seam_painting_version: 0,
            m_mm_painting_version: 0,
            m_xml_parser: false,
            m_parse_error: false,
            m_parse_error_message: String::new(),
            m_model: model,
            m_unit_factor: 1.0,
            m_curr_object: CurrentObject::new(),
            m_objects: BTreeMap::new(),
            m_objects_aliases: BTreeMap::new(),
            m_instances: Vec::new(),
            m_geometries: BTreeMap::new(),
            m_curr_config: CurrentConfig {
                object_id: -1,
                volume_id: -1,
            },
            m_objects_metadata: BTreeMap::new(),
            m_layer_heights_profiles: BTreeMap::new(),
            m_layer_config_ranges: BTreeMap::new(),
            m_sla_support_points: BTreeMap::new(),
            m_sla_drain_holes: BTreeMap::new(),
            m_curr_metadata_name: String::new(),
            m_curr_characters: String::new(),
            m_name: String::new(),
        }
    }

    // 3mf.cpp:672-675 — `~_3MF_Importer() { _destroy_xml_parser(); }` (RAII)

    /// 3mf.cpp:555 — `unsigned int version() const`
    pub fn version(&self) -> u32 {
        self.m_version
    }

    /// 3mf.cpp:677-702
    /// C++: `bool load_model_from_file(const std::string& filename, Model& model, DynamicPrintConfig& config, ConfigSubstitutionContext& config_substitutions, bool check_version)`
    /// (the `Model&` lives in `self.m_model`.)
    pub fn load_model_from_file(
        &mut self,
        filename: &str,
        config: &mut DynamicPrintConfig,
        config_substitutions: &mut ConfigSubstitutionContext,
        check_version: bool,
    ) -> Result<bool> {
        // 3mf.cpp:679-699
        self.m_version = 0;
        self.m_fdm_supports_painting_version = 0;
        self.m_seam_painting_version = 0;
        self.m_mm_painting_version = 0;
        self.m_check_version = check_version;
        // m_model = &model; — bound at construction.
        self.m_unit_factor = 1.0;
        self.m_curr_object.reset();
        self.m_objects.clear();
        self.m_objects_aliases.clear();
        self.m_instances.clear();
        self.m_geometries.clear();
        self.m_curr_config.object_id = -1;
        self.m_curr_config.volume_id = -1;
        self.m_objects_metadata.clear();
        self.m_layer_heights_profiles.clear();
        self.m_layer_config_ranges.clear();
        self.m_sla_support_points.clear();
        self.m_curr_metadata_name.clear();
        self.m_curr_characters.clear();
        self.base.clear_errors();

        // 3mf.cpp:701
        self._load_model_from_file(filename, config, config_substitutions)
    }

    /// 3mf.cpp:704-710
    fn _destroy_xml_parser(&mut self) {
        if self.m_xml_parser {
            // XML_ParserFree(m_xml_parser);
            self.m_xml_parser = false;
        }
    }

    /// 3mf.cpp:712-720
    fn _stop_xml_parser(&mut self, msg: &str) {
        debug_assert!(!self.m_parse_error);
        debug_assert!(self.m_parse_error_message.is_empty());
        debug_assert!(self.m_xml_parser);
        self.m_parse_error = true;
        self.m_parse_error_message = msg.to_string();
        // XML_StopParser(m_xml_parser, false); — the parse drivers check
        // `m_parse_error` after every event and abort.
    }

    /// 3mf.cpp:561
    /// (The parse drivers read `m_parse_error` directly, like the C++ callback
    ///  lambda at 3mf.cpp:991 reads it through this accessor.)
    #[allow(dead_code)]
    fn parse_error(&self) -> bool {
        self.m_parse_error
    }

    /// 3mf.cpp:562-568
    /// (The expat branch — `XML_ErrorString(XML_GetErrorCode(...))` — is
    /// supplied by the quick-xml error at the call sites.)
    fn parse_error_message(&self) -> &str {
        if self.m_parse_error_message.is_empty() {
            // The error was signalled by the user code, not the expat parser.
            "Invalid 3MF format"
        } else {
            &self.m_parse_error_message
        }
    }

    /// 3mf.cpp:722-953
    /// C++: `bool _load_model_from_file(const std::string& filename, Model& model, DynamicPrintConfig& config, ConfigSubstitutionContext& config_substitutions)`
    fn _load_model_from_file(
        &mut self,
        filename: &str,
        _config: &mut DynamicPrintConfig,
        config_substitutions: &mut ConfigSubstitutionContext,
    ) -> Result<bool> {
        // 3mf.cpp:724-730 — mz_zip_zero_struct / open_zip_reader
        let file = match std::fs::File::open(filename) {
            Ok(f) => f,
            Err(_) => {
                self.base.add_error("Unable to open the file");
                return Ok(false);
            }
        };
        let mut archive = match zip::ZipArchive::new(file) {
            Ok(a) => a,
            Err(_) => {
                self.base.add_error("Unable to open the file");
                return Ok(false);
            }
        };

        // 3mf.cpp:732
        let num_entries = archive.len();

        // 3mf.cpp:736
        self.m_name = std::path::Path::new(filename)
            .file_stem()
            .map(|s| s.to_string_lossy().into_owned())
            .unwrap_or_default();

        // we first loop the entries to read from the archive the .model file only, in order to extract the version from it
        // 3mf.cpp:739-762
        for i in 0..num_entries {
            // 3mf.cpp:740 — mz_zip_reader_file_stat
            let name = match archive.by_index(i) {
                Ok(f) => f.name().to_string(),
                Err(_) => continue,
            };
            // 3mf.cpp:742 — std::replace(name.begin(), name.end(), '\\', '/');
            let name = name.replace('\\', "/");

            // 3mf.cpp:744
            if istarts_with(&name, MODEL_FOLDER) && iends_with(&name, MODEL_EXTENSION) {
                // 3mf.cpp:745-759
                match self._extract_model_from_archive(&mut archive, i) {
                    // valid model name -> extract model
                    Ok(true) => {}
                    Ok(false) => {
                        // 3mf.cpp:748-752 — close_zip_reader + add_error
                        self.base
                            .add_error("Archive does not contain a valid model");
                        return Ok(false);
                    }
                    Err(e) => {
                        // 3mf.cpp:754-759 — ensure the zip archive is closed and
                        // rethrow the exception: throw Slic3r::FileIOError(e.what());
                        return Err(e);
                    }
                }
            }
        }

        // we then loop again the entries to read other files stored in the archive
        // 3mf.cpp:765-806
        for i in 0..num_entries {
            let name = match archive.by_index(i) {
                Ok(f) => f.name().to_string(),
                Err(_) => continue,
            };
            // 3mf.cpp:768
            let name = name.replace('\\', "/");

            // 3mf.cpp:770-795 — the LAYER_HEIGHTS_PROFILE_FILE /
            // LAYER_CONFIG_RANGES_FILE / SLA_SUPPORT_POINTS_FILE /
            // SLA_DRAIN_HOLES_FILE / PRINT_CONFIG_FILE /
            // CUSTOM_GCODE_PER_PRINT_Z_FILE extraction branches are commented
            // out in BambuStudio's 3mf.cpp (kept disabled here as well).

            // only read the model config for Prusa 3mf
            // 3mf.cpp:797-804
            if iequals(&name, MODEL_CONFIG_FILE) {
                // extract slic3r model config file
                if !self._extract_model_config_from_archive(&mut archive, i) {
                    // close_zip_reader(&archive);
                    self.base
                        .add_error("Archive does not contain a valid model config");
                    return Ok(false);
                }
            }
        }

        // 3mf.cpp:808 — close_zip_reader(&archive);
        drop(archive);

        // 3mf.cpp:810-853
        if self.m_version == 0 {
            // if the 3mf was not produced by PrusaSlicer and there is more than one instance,
            // split the object in as many objects as instances
            // 3mf.cpp:813-815
            let curr_models_count = self.m_model.objects.len();
            let mut i = 0usize;
            while i < curr_models_count {
                // 3mf.cpp:816-817
                if self.m_model.objects[i].instances.len() > 1 {
                    // select the geometry associated with the original model object
                    // 3mf.cpp:818-830
                    let mut geometry_id: Option<i32> = None;
                    for (&object_id, &object_idx) in &self.m_objects {
                        if object_idx == i as i32 {
                            if !self.m_geometries.contains_key(&object_id) {
                                // 3mf.cpp:823-826
                                self.base.add_error("Unable to find object geometry");
                                return Ok(false);
                            }
                            geometry_id = Some(object_id);
                            break;
                        }
                    }

                    // 3mf.cpp:832-835
                    let geometry_id = match geometry_id {
                        Some(id) => id,
                        None => {
                            self.base.add_error("Unable to find object geometry");
                            return Ok(false);
                        }
                    };

                    // use the geometry to create the volumes in the new model objects
                    // 3mf.cpp:838 — `(unsigned int)geometry->triangles.size() - 1`
                    // (wrapping: C unsigned arithmetic; geometries are only
                    //  stored non-empty, see _handle_end_object).
                    let volumes = vec![VolumeMetadata::new(
                        0,
                        (self.m_geometries[&geometry_id].triangles.len() as u32).wrapping_sub(1),
                    )];

                    // for each instance after the 1st, create a new model object containing only that instance
                    // and copy into it the geometry
                    // 3mf.cpp:842-849
                    while self.m_model.objects[i].instances.len() > 1 {
                        // ModelObject* new_model_object = m_model->add_object(*model_object);
                        let mut new_model_object = self.m_model.objects[i].clone();
                        // new_model_object->clear_instances();
                        new_model_object.instances.clear();
                        // new_model_object->add_instance(*model_object->instances.back());
                        let last_instance = *self.m_model.objects[i].instances.last().unwrap();
                        new_model_object.instances.push(last_instance);
                        // model_object->delete_last_instance();
                        self.m_model.objects[i].instances.pop();
                        self.m_model.objects.push(new_model_object);
                        let new_idx = self.m_model.objects.len() - 1;
                        if !Self::_generate_volumes(
                            &mut self.base,
                            &self.m_prusaslicer_generator_version,
                            self.m_version,
                            &mut self.m_model.objects[new_idx],
                            &self.m_geometries[&geometry_id],
                            &volumes,
                            config_substitutions,
                        ) {
                            return Ok(false);
                        }
                    }
                }
                // 3mf.cpp:851
                i += 1;
            }
        }

        // 3mf.cpp:855-919
        for (&object_first, &object_second) in &self.m_objects {
            // 3mf.cpp:856-859
            if object_second >= self.m_model.objects.len() as i32 {
                self.base.add_error("Unable to find object");
                return Ok(false);
            }
            // 3mf.cpp:860
            let model_object_idx = object_second as usize;
            // 3mf.cpp:861-865
            let obj_geometry = match self.m_geometries.get(&object_first) {
                Some(g) => g,
                None => {
                    self.base.add_error("Unable to find object geometry");
                    return Ok(false);
                }
            };

            // m_layer_heights_profiles are indexed by a 1 based model object index.
            // 3mf.cpp:868-870
            let _obj_layer_heights_profile =
                self.m_layer_heights_profiles.get(&(object_second + 1));
            // BLOCKED(model): `model_object->layer_height_profile.set(...)` — the
            // simplified ModelObject has no layer_height_profile; the map is only
            // filled by `_extract_layer_heights_profile_config_from_archive`, whose
            // call site is commented out in C++ (3mf.cpp:771-774), so it is empty.

            // m_layer_config_ranges are indexed by a 1 based model object index.
            // 3mf.cpp:873-875
            let _obj_layer_config_ranges = self.m_layer_config_ranges.get(&(object_second + 1));
            // BLOCKED(model): `model_object->layer_config_ranges = ...` — same as above
            // (call site commented out at 3mf.cpp:775-778).

            // m_sla_support_points are indexed by a 1 based model object index.
            // 3mf.cpp:878-882
            let _obj_sla_support_points = self.m_sla_support_points.get(&(object_second + 1));
            // BLOCKED(model): `model_object->sla_support_points = ...;
            // model_object->sla_points_status = sla::PointsStatus::UserModified;`
            // (call site commented out at 3mf.cpp:779-782).

            // 3mf.cpp:884-887
            let _obj_drain_holes = self.m_sla_drain_holes.get(&(object_second + 1));
            // BLOCKED(model): `model_object->sla_drain_holes = ...` (call site
            // commented out at 3mf.cpp:783-786).

            // 3mf.cpp:889-890
            let mut volumes: VolumeMetadataList = Vec::new();
            let volumes_ptr: &VolumeMetadataList;

            // 3mf.cpp:892-915
            if let Some(obj_metadata) = self.m_objects_metadata.get(&object_first) {
                // config data has been found, this model was saved using slic3r pe

                // apply object's name and config data
                // 3mf.cpp:897-902
                for metadata in &obj_metadata.metadata {
                    if metadata.key == "name" {
                        self.m_model.objects[model_object_idx].name = metadata.value.clone();
                    }
                    // BLOCKED(config): `model_object->config.set_deserialize(key, value,
                    // config_substitutions)` — the reflective DynamicPrintConfig /
                    // ModelConfig layer is unported; non-"name" keys are dropped.
                }

                // select object's detected volumes
                // 3mf.cpp:905
                volumes_ptr = &obj_metadata.volumes;
            } else {
                // config data not found, this model was not saved using slic3r pe

                // add the entire geometry as the single volume to generate
                // 3mf.cpp:911 — volumes.emplace_back(0, (int)triangles.size() - 1);
                volumes.push(VolumeMetadata::new(
                    0,
                    (obj_geometry.triangles.len() as i32 - 1) as u32,
                ));

                // select as volumes
                // 3mf.cpp:914
                volumes_ptr = &volumes;
            }

            // 3mf.cpp:917-918
            if !Self::_generate_volumes(
                &mut self.base,
                &self.m_prusaslicer_generator_version,
                self.m_version,
                &mut self.m_model.objects[model_object_idx],
                obj_geometry,
                volumes_ptr,
                config_substitutions,
            ) {
                return Ok(false);
            }
        }

        // 3mf.cpp:921-935
        // for (ModelObject* o : model.objects)
        //     for (ModelVolume* v : o->volumes)
        //         if (v->source.input_file.empty() && v->type() == ModelVolumeType::MODEL_PART) {
        //             v->source.input_file = filename;
        //             if (v->source.volume_idx == -1) v->source.volume_idx = volume_idx;
        //             if (v->source.object_idx == -1) v->source.object_idx = object_idx;
        //         }
        // BLOCKED(model): the simplified Model has no ModelVolume::source.

        //BBS: copy object isteadof instance
        // 3mf.cpp:938-947
        let object_size = self.m_model.objects.len();
        for obj_index in 0..object_size {
            while self.m_model.objects[obj_index].instances.len() > 1 {
                // ModelObject* new_model_object = model.add_object(*object);
                let mut new_model_object = self.m_model.objects[obj_index].clone();
                // new_model_object->clear_instances();
                new_model_object.instances.clear();
                // new_model_object->add_instance(*object->instances.back());
                let last_instance = *self.m_model.objects[obj_index].instances.last().unwrap();
                new_model_object.instances.push(last_instance);
                // object->delete_last_instance();
                self.m_model.objects[obj_index].instances.pop();
                self.m_model.objects.push(new_model_object);
            }
        }

        // 3mf.cpp:949-950
        // //        // fixes the min z of the model if negative
        // //        model.adjust_min_z();

        // 3mf.cpp:952
        Ok(true)
    }

    /// 3mf.cpp:955-1017
    /// C++: `bool _extract_model_from_archive(mz_zip_archive& archive, const mz_zip_archive_file_stat& stat)`
    /// (the Rust port takes the entry index into the open archive; `Err(...)`
    /// carries the C++ `Slic3r::FileIOError` rethrown for `version_error`s at
    /// 3mf.cpp:1000-1004.)
    fn _extract_model_from_archive(
        &mut self,
        archive: &mut zip::ZipArchive<std::fs::File>,
        index: usize,
    ) -> Result<bool> {
        use quick_xml::events::Event;
        use quick_xml::reader::Reader;

        let mut data = Vec::new();
        let stat_filename;
        {
            let mut file = match archive.by_index(index) {
                Ok(f) => f,
                Err(_) => {
                    // res == 0 (3mf.cpp:1011-1014)
                    self.base
                        .add_error("Error while extracting model data from zip archive");
                    return Ok(false);
                }
            };
            stat_filename = file.name().to_string();
            // 3mf.cpp:957-960
            if file.size() == 0 {
                drop(file);
                self.base.add_error("Found invalid size");
                return Ok(false);
            }
            // 3mf.cpp:989-998 — mz_zip_reader_extract_file_to_callback feeds the
            // expat parser chunk by chunk; the whole entry is read here, then parsed.
            if file.read_to_end(&mut data).is_err() {
                drop(file);
                self.base
                    .add_error("Error while extracting model data from zip archive");
                return Ok(false);
            }
        }

        // 3mf.cpp:962-972 — XML_ParserCreate + handler registration
        self._destroy_xml_parser();
        self.m_xml_parser = true;
        // (XML_ParserCreate cannot fail to allocate here.)

        // 3mf.cpp:974-998 — CallbackData + parse loop. expat decodes to UTF-8.
        let text = String::from_utf8_lossy(&data).into_owned();
        let mut reader = Reader::from_str(&text);
        loop {
            let event = reader.read_event();
            match event {
                Ok(Event::Start(e)) => {
                    let name = String::from_utf8_lossy(e.name().as_ref()).into_owned();
                    let atts = collect_attributes(&e);
                    if let Err(VersionError(msg)) =
                        self._handle_start_model_xml_element(&name, &atts)
                    {
                        // 3mf.cpp:1000-1004 — catch (const version_error& e)
                        //   { throw Slic3r::FileIOError(e.what()); }
                        return Err(Error::IO(msg));
                    }
                }
                Ok(Event::Empty(e)) => {
                    // expat reports `<tag/>` as startElement followed by endElement.
                    let name = String::from_utf8_lossy(e.name().as_ref()).into_owned();
                    let atts = collect_attributes(&e);
                    if let Err(VersionError(msg)) =
                        self._handle_start_model_xml_element(&name, &atts)
                    {
                        return Err(Error::IO(msg));
                    }
                    if !self.m_parse_error {
                        if let Err(VersionError(msg)) = self._handle_end_model_xml_element(&name) {
                            return Err(Error::IO(msg));
                        }
                    }
                }
                Ok(Event::End(e)) => {
                    let name = String::from_utf8_lossy(e.name().as_ref()).into_owned();
                    if let Err(VersionError(msg)) = self._handle_end_model_xml_element(&name) {
                        return Err(Error::IO(msg));
                    }
                }
                Ok(Event::Text(e)) => {
                    let s = e
                        .unescape()
                        .map(|v| v.into_owned())
                        .unwrap_or_else(|_| String::from_utf8_lossy(&e).into_owned());
                    self._handle_model_xml_characters(&s);
                }
                Ok(Event::CData(e)) => {
                    let s = String::from_utf8_lossy(&e.into_inner()).into_owned();
                    self._handle_model_xml_characters(&s);
                }
                Ok(Event::Decl(_))
                | Ok(Event::Comment(_))
                | Ok(Event::PI(_))
                | Ok(Event::DocType(_)) => {}
                Ok(Event::Eof) => break,
                Err(err) => {
                    // 3mf.cpp:991-995 — the callback throws FileIOError
                    //   "Error (%s) while parsing '%s' at line %d"
                    // which is caught at 3mf.cpp:1005-1009 (catch std::exception)
                    // -> add_error(e.what()); return false;
                    let msg = format!(
                        "Error ({}) while parsing '{}' at line {}",
                        err,
                        stat_filename,
                        line_at(&text, reader.buffer_position())
                    );
                    self.base.add_error(msg);
                    return Ok(false);
                }
            }
            // 3mf.cpp:991 — `|| data->importer.parse_error()` checked after
            // every chunk; XML_StopParser aborts the parse.
            if self.m_parse_error {
                let msg = format!(
                    "Error ({}) while parsing '{}' at line {}",
                    self.parse_error_message(),
                    stat_filename,
                    line_at(&text, reader.buffer_position())
                );
                self.base.add_error(msg);
                return Ok(false);
            }
        }

        // 3mf.cpp:1016
        Ok(true)
    }

    /// 3mf.cpp:1019-1040
    /// C++: `void _extract_print_config_from_archive(...)`
    /// NOTE: the call site is commented out in BambuStudio (3mf.cpp:787-790).
    #[allow(dead_code)]
    fn _extract_print_config_from_archive(
        &mut self,
        archive: &mut zip::ZipArchive<std::fs::File>,
        index: usize,
        _config: &mut DynamicPrintConfig,
        _subs_context: &mut ConfigSubstitutionContext,
        _archive_filename: &str,
    ) {
        // 3mf.cpp:1024-1030
        let mut buffer = Vec::new();
        let ok = match archive.by_index(index) {
            Ok(mut f) => {
                if f.size() == 0 {
                    return; // stat.m_uncomp_size > 0 guard
                }
                f.read_to_end(&mut buffer).is_ok()
            }
            Err(_) => false,
        };
        if !ok {
            self.base
                .add_error("Error while reading config data to buffer");
            return;
        }
        // 3mf.cpp:1031-1038
        //FIXME Loading a "will be one day a legacy format" of configuration in a form of a G-code comment.
        // Each config line is prefixed with a semicolon (G-code comment), that is ugly.
        // ConfigBase::load_from_gcode_string_legacy(config, buffer.data(), config_substitutions);
        // BLOCKED(config): `ConfigBase::load_from_gcode_string_legacy` requires
        // the reflective DynamicPrintConfig layer, which is unported.
    }

    /// 3mf.cpp:1042-1102
    /// C++: `void _extract_layer_heights_profile_config_from_archive(...)`
    /// NOTE: the call site is commented out in BambuStudio (3mf.cpp:771-774).
    #[allow(dead_code)]
    fn _extract_layer_heights_profile_config_from_archive(
        &mut self,
        archive: &mut zip::ZipArchive<std::fs::File>,
        index: usize,
    ) {
        // 3mf.cpp:1044-1050
        let mut buffer_bytes = Vec::new();
        let ok = match archive.by_index(index) {
            Ok(mut f) => {
                if f.size() == 0 {
                    return;
                }
                f.read_to_end(&mut buffer_bytes).is_ok()
            }
            Err(_) => false,
        };
        if !ok {
            self.base
                .add_error("Error while reading layer heights profile data to buffer");
            return;
        }
        let mut buffer = String::from_utf8_lossy(&buffer_bytes).into_owned();

        // 3mf.cpp:1052-1053
        if buffer.ends_with('\n') {
            buffer.pop();
        }

        // 3mf.cpp:1055-1056 — boost::split(..., is_any_of("\n"), token_compress_off)
        let objects: Vec<&str> = buffer.split('\n').collect();

        // 3mf.cpp:1058-1100
        for object in objects {
            // 3mf.cpp:1059-1064
            let object_data: Vec<&str> = object.split('|').collect();
            if object_data.len() != 2 {
                self.base.add_error("Error while reading object data");
                continue;
            }

            // 3mf.cpp:1066-1071
            let object_data_id: Vec<&str> = object_data[0].split('=').collect();
            if object_data_id.len() != 2 {
                self.base.add_error("Error while reading object id");
                continue;
            }

            // 3mf.cpp:1073-1077
            let object_id = atoi(object_data_id[1]);
            if object_id == 0 {
                self.base.add_error("Found invalid object id");
                continue;
            }

            // 3mf.cpp:1079-1083
            if self.m_layer_heights_profiles.contains_key(&object_id) {
                self.base
                    .add_error("Found duplicated layer heights profile");
                continue;
            }

            // 3mf.cpp:1085-1090
            let object_data_profile: Vec<&str> = object_data[1].split(';').collect();
            if object_data_profile.len() <= 4 || object_data_profile.len() % 2 != 0 {
                self.base.add_error("Found invalid layer heights profile");
                continue;
            }

            // 3mf.cpp:1092-1097
            let mut profile: Vec<f64> = Vec::with_capacity(object_data_profile.len());
            for value in object_data_profile {
                profile.push(atof(value));
            }

            // 3mf.cpp:1099
            self.m_layer_heights_profiles.insert(object_id, profile);
        }
    }

    /// 3mf.cpp:1104-1159
    /// C++: `void _extract_layer_config_ranges_from_archive(...)`
    /// NOTE: the call site is commented out in BambuStudio (3mf.cpp:775-778).
    /// BLOCKED(config): `config.set_deserialize(opt_key, value, config_substitutions)`
    /// is unported; the raw `(opt_key, value)` pairs are retained losslessly.
    #[allow(dead_code)]
    fn _extract_layer_config_ranges_from_archive(
        &mut self,
        archive: &mut zip::ZipArchive<std::fs::File>,
        index: usize,
        _config_substitutions: &mut ConfigSubstitutionContext,
    ) {
        use quick_xml::events::Event;
        use quick_xml::reader::Reader;

        // 3mf.cpp:1106-1112
        let mut buffer_bytes = Vec::new();
        let ok = match archive.by_index(index) {
            Ok(mut f) => {
                if f.size() == 0 {
                    return;
                }
                f.read_to_end(&mut buffer_bytes).is_ok()
            }
            Err(_) => false,
        };
        if !ok {
            self.base
                .add_error("Error while reading layer config ranges data to buffer");
            return;
        }

        // 3mf.cpp:1114-1116 — wrap returned xml to istringstream + pt::read_xml.
        // The ptree traversal below ("objects" -> "object" -> "range" ->
        // "option") is reproduced with a streaming quick-xml reader.
        let text = String::from_utf8_lossy(&buffer_bytes).into_owned();
        let mut reader = Reader::from_str(&text);

        // 3mf.cpp:1118-1157 — for (const auto& object : objects_tree.get_child("objects"))
        let mut obj_idx: i32 = -1;
        let mut obj_valid = false;
        let mut config_ranges: TLayerConfigRanges = Vec::new();
        let mut in_range = false;
        let mut curr_range_key: (f64, f64) = (0.0, 0.0);
        let mut curr_range_options: Vec<(String, String)> = Vec::new();
        let mut curr_opt_key: Option<String> = None;
        let mut curr_opt_value = String::new();

        loop {
            match reader.read_event() {
                Ok(Event::Start(e)) | Ok(Event::Empty(e)) => {
                    let name = String::from_utf8_lossy(e.name().as_ref()).into_owned();
                    let atts = collect_attributes(&e);
                    match name.as_str() {
                        "object" => {
                            // 3mf.cpp:1120-1124 — int obj_idx = <xmlattr>.id (-1 default)
                            obj_idx = get_attribute_value_charptr(&atts, "id")
                                .map(atoi)
                                .unwrap_or(-1);
                            if obj_idx <= 0 {
                                self.base.add_error("Found invalid object id");
                                obj_valid = false;
                            } else if self.m_layer_config_ranges.contains_key(&obj_idx) {
                                // 3mf.cpp:1126-1130
                                self.base.add_error("Found duplicated layer config range");
                                obj_valid = false;
                            } else {
                                obj_valid = true;
                                config_ranges = Vec::new();
                            }
                        }
                        "range" if obj_valid => {
                            // 3mf.cpp:1134-1139
                            in_range = true;
                            let min_z = get_attribute_value_charptr(&atts, "min_z")
                                .map(atof)
                                .unwrap_or(0.0);
                            let max_z = get_attribute_value_charptr(&atts, "max_z")
                                .map(atof)
                                .unwrap_or(0.0);
                            curr_range_key = (min_z, max_z);
                            // 3mf.cpp:1142 — DynamicPrintConfig config;
                            curr_range_options = Vec::new();
                        }
                        "option" if in_range => {
                            // 3mf.cpp:1144-1147
                            curr_opt_key = Some(get_attribute_value_string(&atts, "opt_key"));
                            curr_opt_value.clear();
                        }
                        _ => {}
                    }
                }
                Ok(Event::Text(e)) => {
                    if curr_opt_key.is_some() {
                        // 3mf.cpp:1148 — std::string value = option.second.data();
                        let s = e
                            .unescape()
                            .map(|v| v.into_owned())
                            .unwrap_or_else(|_| String::from_utf8_lossy(&e).into_owned());
                        curr_opt_value.push_str(&s);
                    }
                }
                Ok(Event::End(e)) => {
                    let name = String::from_utf8_lossy(e.name().as_ref()).into_owned();
                    match name.as_str() {
                        "option" => {
                            if let Some(opt_key) = curr_opt_key.take() {
                                // 3mf.cpp:1149 — config.set_deserialize(opt_key, value, ...)
                                // BLOCKED(config): raw pair retained.
                                curr_range_options
                                    .push((opt_key, std::mem::take(&mut curr_opt_value)));
                            }
                        }
                        "range" => {
                            if in_range {
                                // 3mf.cpp:1152 — config_ranges[{min_z, max_z}].assign_config(...)
                                config_ranges.push((
                                    curr_range_key,
                                    std::mem::take(&mut curr_range_options),
                                ));
                                in_range = false;
                            }
                        }
                        "object" => {
                            // 3mf.cpp:1155-1156
                            if obj_valid && !config_ranges.is_empty() {
                                self.m_layer_config_ranges
                                    .insert(obj_idx, std::mem::take(&mut config_ranges));
                            }
                            obj_valid = false;
                        }
                        _ => {}
                    }
                }
                Ok(Event::Eof) | Err(_) => break,
                _ => {}
            }
        }
    }

    /// 3mf.cpp:1161-1241
    /// C++: `void _extract_sla_support_points_from_archive(...)`
    /// NOTE: the call site is commented out in BambuStudio (3mf.cpp:779-782).
    #[allow(dead_code)]
    fn _extract_sla_support_points_from_archive(
        &mut self,
        archive: &mut zip::ZipArchive<std::fs::File>,
        index: usize,
    ) {
        // 3mf.cpp:1163-1169
        let mut buffer_bytes = Vec::new();
        let ok = match archive.by_index(index) {
            Ok(mut f) => {
                if f.size() == 0 {
                    return;
                }
                f.read_to_end(&mut buffer_bytes).is_ok()
            }
            Err(_) => false,
        };
        if !ok {
            self.base
                .add_error("Error while reading sla support points data to buffer");
            return;
        }
        let mut buffer = String::from_utf8_lossy(&buffer_bytes).into_owned();

        // 3mf.cpp:1171-1172
        if buffer.ends_with('\n') {
            buffer.pop();
        }

        // 3mf.cpp:1174-1175
        let mut objects: Vec<String> = buffer.split('\n').map(|s| s.to_string()).collect();

        // Info on format versioning - see 3mf.hpp
        // 3mf.cpp:1177-1184
        let mut version = 0;
        let key = "support_points_format_version=";
        if !objects.is_empty() && objects[0].contains(key) {
            objects[0].drain(..key.len()); // removes the string
            version = atoi(&objects[0]); // std::stoi
            objects.remove(0); // pop the header
        }

        // 3mf.cpp:1186-1239
        for object in &objects {
            // 3mf.cpp:1187-1193
            let object_data: Vec<&str> = object.split('|').collect();
            if object_data.len() != 2 {
                self.base.add_error("Error while reading object data");
                continue;
            }

            // 3mf.cpp:1195-1199
            let object_data_id: Vec<&str> = object_data[0].split('=').collect();
            if object_data_id.len() != 2 {
                self.base.add_error("Error while reading object id");
                continue;
            }

            // 3mf.cpp:1202-1206
            let object_id = atoi(object_data_id[1]);
            if object_id == 0 {
                self.base.add_error("Found invalid object id");
                continue;
            }

            // 3mf.cpp:1208-1212
            if self.m_sla_support_points.contains_key(&object_id) {
                self.base.add_error("Found duplicated SLA support points");
                continue;
            }

            // 3mf.cpp:1214-1215
            let object_data_points: Vec<&str> = object_data[1].split(' ').collect();

            // 3mf.cpp:1217
            let mut sla_support_points: Vec<SlaSupportPoint> = Vec::new();

            // 3mf.cpp:1219-1226
            if version == 0 {
                let mut i = 0usize;
                while i < object_data_points.len() {
                    sla_support_points.push(SlaSupportPoint {
                        pos: Vec3f::new(
                            atof(object_data_points[i]) as f32,
                            atof(object_data_points[i + 1]) as f32,
                            atof(object_data_points[i + 2]) as f32,
                        ),
                        head_front_radius: 0.4,
                        is_new_island: false,
                    });
                    i += 3;
                }
            }
            // 3mf.cpp:1227-1235
            if version == 1 {
                let mut i = 0usize;
                while i < object_data_points.len() {
                    sla_support_points.push(SlaSupportPoint {
                        pos: Vec3f::new(
                            atof(object_data_points[i]) as f32,
                            atof(object_data_points[i + 1]) as f32,
                            atof(object_data_points[i + 2]) as f32,
                        ),
                        head_front_radius: atof(object_data_points[i + 3]) as f32,
                        //FIXME storing boolean as 0 / 1 and importing it as float.
                        is_new_island: (atof(object_data_points[i + 4]) - 1.0).abs() < EPSILON,
                    });
                    i += 5;
                }
            }

            // 3mf.cpp:1237-1238
            if !sla_support_points.is_empty() {
                self.m_sla_support_points
                    .insert(object_id, sla_support_points);
            }
        }
    }

    /// 3mf.cpp:1243-1326
    /// C++: `void _extract_sla_drain_holes_from_archive(...)`
    /// NOTE: the call site is commented out in BambuStudio (3mf.cpp:783-786).
    #[allow(dead_code)]
    fn _extract_sla_drain_holes_from_archive(
        &mut self,
        archive: &mut zip::ZipArchive<std::fs::File>,
        index: usize,
    ) {
        // 3mf.cpp:1245-1251
        let mut buffer_bytes = Vec::new();
        let ok = match archive.by_index(index) {
            Ok(mut f) => {
                if f.size() == 0 {
                    return;
                }
                f.read_to_end(&mut buffer_bytes).is_ok()
            }
            Err(_) => false,
        };
        if !ok {
            self.base
                .add_error("Error while reading sla support points data to buffer");
            return;
        }
        let mut buffer = String::from_utf8_lossy(&buffer_bytes).into_owned();

        // 3mf.cpp:1253-1254
        if buffer.ends_with('\n') {
            buffer.pop();
        }

        // 3mf.cpp:1256-1257
        let mut objects: Vec<String> = buffer.split('\n').map(|s| s.to_string()).collect();

        // Info on format versioning - see 3mf.hpp
        // 3mf.cpp:1259-1266
        let mut version = 0;
        let key = "drain_holes_format_version=";
        if !objects.is_empty() && objects[0].contains(key) {
            objects[0].drain(..key.len()); // removes the string
            version = atoi(&objects[0]); // std::stoi
            objects.remove(0); // pop the header
        }

        // 3mf.cpp:1268-1324
        for object in &objects {
            // 3mf.cpp:1269-1275
            let object_data: Vec<&str> = object.split('|').collect();
            if object_data.len() != 2 {
                self.base.add_error("Error while reading object data");
                continue;
            }

            // 3mf.cpp:1277-1282
            let object_data_id: Vec<&str> = object_data[0].split('=').collect();
            if object_data_id.len() != 2 {
                self.base.add_error("Error while reading object id");
                continue;
            }

            // 3mf.cpp:1284-1288
            let object_id = atoi(object_data_id[1]);
            if object_id == 0 {
                self.base.add_error("Found invalid object id");
                continue;
            }

            // 3mf.cpp:1290-1294
            if self.m_sla_drain_holes.contains_key(&object_id) {
                self.base.add_error("Found duplicated SLA drain holes");
                continue;
            }

            // 3mf.cpp:1296-1297
            let object_data_points: Vec<&str> = object_data[1].split(' ').collect();

            // 3mf.cpp:1299
            let mut sla_drain_holes: Vec<SlaDrainHole> = Vec::new();

            // 3mf.cpp:1301-1311
            if version == 1 {
                let mut i = 0usize;
                while i < object_data_points.len() {
                    sla_drain_holes.push(SlaDrainHole {
                        pos: Vec3f::new(
                            atof(object_data_points[i]) as f32,
                            atof(object_data_points[i + 1]) as f32,
                            atof(object_data_points[i + 2]) as f32,
                        ),
                        normal: Vec3f::new(
                            atof(object_data_points[i + 3]) as f32,
                            atof(object_data_points[i + 4]) as f32,
                            atof(object_data_points[i + 5]) as f32,
                        ),
                        radius: atof(object_data_points[i + 6]) as f32,
                        height: atof(object_data_points[i + 7]) as f32,
                    });
                    i += 8;
                }
            }

            // The holes are saved elevated above the mesh and deeper (bad idea indeed).
            // This is retained for compatibility.
            // Place the hole to the mesh and make it shallower to compensate.
            // The offset is 1 mm above the mesh.
            // 3mf.cpp:1317-1320
            for hole in &mut sla_drain_holes {
                hole.pos += hole.normal.normalize();
                hole.height -= 1.0;
            }

            // 3mf.cpp:1322-1323
            if !sla_drain_holes.is_empty() {
                self.m_sla_drain_holes.insert(object_id, sla_drain_holes);
            }
        }
    }

    /// 3mf.cpp:1328-1366
    /// C++: `bool _extract_model_config_from_archive(mz_zip_archive& archive, const mz_zip_archive_file_stat& stat, Model& model)`
    fn _extract_model_config_from_archive(
        &mut self,
        archive: &mut zip::ZipArchive<std::fs::File>,
        index: usize,
    ) -> bool {
        use quick_xml::events::Event;
        use quick_xml::reader::Reader;

        let mut data = Vec::new();
        {
            let mut file = match archive.by_index(index) {
                Ok(f) => f,
                Err(_) => {
                    self.base
                        .add_error("Error while reading config data to buffer");
                    return false;
                }
            };
            // 3mf.cpp:1330-1333
            if file.size() == 0 {
                drop(file);
                self.base.add_error("Found invalid size");
                return false;
            }
            // 3mf.cpp:1346-1356 — XML_GetBuffer + extract to mem
            if file.read_to_end(&mut data).is_err() {
                drop(file);
                self.base
                    .add_error("Error while reading config data to buffer");
                return false;
            }
        }

        // 3mf.cpp:1335-1344 — parser + element handlers (no characters handler)
        self._destroy_xml_parser();
        self.m_xml_parser = true;

        // 3mf.cpp:1358-1363 — XML_ParseBuffer
        let text = String::from_utf8_lossy(&data).into_owned();
        let mut reader = Reader::from_str(&text);
        loop {
            match reader.read_event() {
                Ok(Event::Start(e)) => {
                    let name = String::from_utf8_lossy(e.name().as_ref()).into_owned();
                    let atts = collect_attributes(&e);
                    self._handle_start_config_xml_element(&name, &atts);
                }
                Ok(Event::Empty(e)) => {
                    let name = String::from_utf8_lossy(e.name().as_ref()).into_owned();
                    let atts = collect_attributes(&e);
                    self._handle_start_config_xml_element(&name, &atts);
                    if !self.m_parse_error {
                        self._handle_end_config_xml_element(&name);
                    }
                }
                Ok(Event::End(e)) => {
                    let name = String::from_utf8_lossy(e.name().as_ref()).into_owned();
                    self._handle_end_config_xml_element(&name);
                }
                Ok(Event::Eof) => break,
                Err(err) => {
                    // 3mf.cpp:1359-1362 — "Error (%s) while parsing xml file at line %d"
                    let msg = format!(
                        "Error ({}) while parsing xml file at line {}",
                        err,
                        line_at(&text, reader.buffer_position())
                    );
                    self.base.add_error(msg);
                    return false;
                }
                _ => {}
            }
            if self.m_parse_error {
                // XML_StopParser -> XML_ERROR_ABORTED -> "parsing aborted"
                let msg = format!(
                    "Error ({}) while parsing xml file at line {}",
                    "parsing aborted",
                    line_at(&text, reader.buffer_position())
                );
                self.base.add_error(msg);
                return false;
            }
        }

        // 3mf.cpp:1365
        true
    }

    /// 3mf.cpp:1368-1425
    /// C++: `void _extract_custom_gcode_per_print_z_from_archive(...)`
    /// The entire body is commented out in BambuStudio (3mf.cpp:1370-1424):
    /// reading CUSTOM_GCODE_PER_PRINT_Z_FILE into
    /// `m_model->custom_gcode_per_print_z` is disabled.
    #[allow(dead_code)]
    fn _extract_custom_gcode_per_print_z_from_archive(
        &mut self,
        _archive: &mut zip::ZipArchive<std::fs::File>,
        _index: usize,
    ) {
        // (intentionally empty — mirrors the fully commented-out C++ body)
    }

    /// 3mf.cpp:1427-1464
    /// C++: `void _handle_start_model_xml_element(const char* name, const char** attributes)`
    /// (`Err` carries a `version_error` thrown by a handler.)
    fn _handle_start_model_xml_element(
        &mut self,
        name: &str,
        attributes: &[(String, String)],
    ) -> std::result::Result<(), VersionError> {
        // 3mf.cpp:1429-1430
        if !self.m_xml_parser {
            return Ok(());
        }

        // 3mf.cpp:1432-1433
        let mut res = true;
        // (num_attributes == XML_GetSpecifiedAttributeCount — implicit in the slice.)

        // 3mf.cpp:1435-1460
        if name == MODEL_TAG {
            res = self._handle_start_model(attributes);
        } else if name == RESOURCES_TAG {
            res = self._handle_start_resources(attributes);
        } else if name == OBJECT_TAG {
            res = self._handle_start_object(attributes);
        } else if name == MESH_TAG {
            res = self._handle_start_mesh(attributes);
        } else if name == VERTICES_TAG {
            res = self._handle_start_vertices(attributes);
        } else if name == VERTEX_TAG {
            res = self._handle_start_vertex(attributes);
        } else if name == TRIANGLES_TAG {
            res = self._handle_start_triangles(attributes);
        } else if name == TRIANGLE_TAG {
            res = self._handle_start_triangle(attributes);
        } else if name == COMPONENTS_TAG {
            res = self._handle_start_components(attributes);
        } else if name == COMPONENT_TAG {
            res = self._handle_start_component(attributes);
        } else if name == BUILD_TAG {
            res = self._handle_start_build(attributes);
        } else if name == ITEM_TAG {
            res = self._handle_start_item(attributes);
        } else if name == METADATA_TAG {
            res = self._handle_start_metadata(attributes);
        }

        // 3mf.cpp:1462-1463
        if !res {
            self._stop_xml_parser("");
        }
        Ok(())
    }

    /// 3mf.cpp:1466-1502
    /// C++: `void _handle_end_model_xml_element(const char* name)`
    fn _handle_end_model_xml_element(
        &mut self,
        name: &str,
    ) -> std::result::Result<(), VersionError> {
        // 3mf.cpp:1468-1469
        if !self.m_xml_parser {
            return Ok(());
        }

        // 3mf.cpp:1471
        let mut res = true;

        // 3mf.cpp:1473-1498
        if name == MODEL_TAG {
            res = self._handle_end_model();
        } else if name == RESOURCES_TAG {
            res = self._handle_end_resources();
        } else if name == OBJECT_TAG {
            res = self._handle_end_object();
        } else if name == MESH_TAG {
            res = self._handle_end_mesh();
        } else if name == VERTICES_TAG {
            res = self._handle_end_vertices();
        } else if name == VERTEX_TAG {
            res = self._handle_end_vertex();
        } else if name == TRIANGLES_TAG {
            res = self._handle_end_triangles();
        } else if name == TRIANGLE_TAG {
            res = self._handle_end_triangle();
        } else if name == COMPONENTS_TAG {
            res = self._handle_end_components();
        } else if name == COMPONENT_TAG {
            res = self._handle_end_component();
        } else if name == BUILD_TAG {
            res = self._handle_end_build();
        } else if name == ITEM_TAG {
            res = self._handle_end_item();
        } else if name == METADATA_TAG {
            res = self._handle_end_metadata()?;
        }

        // 3mf.cpp:1500-1501
        if !res {
            self._stop_xml_parser("");
        }
        Ok(())
    }

    /// 3mf.cpp:1504-1507
    /// C++: `void _handle_model_xml_characters(const XML_Char* s, int len)`
    fn _handle_model_xml_characters(&mut self, s: &str) {
        self.m_curr_characters.push_str(s);
    }

    /// 3mf.cpp:1509-1530
    /// C++: `void _handle_start_config_xml_element(const char* name, const char** attributes)`
    fn _handle_start_config_xml_element(&mut self, name: &str, attributes: &[(String, String)]) {
        // 3mf.cpp:1511-1512
        if !self.m_xml_parser {
            return;
        }

        // 3mf.cpp:1514-1515
        let mut res = true;

        // 3mf.cpp:1517-1526
        if name == CONFIG_TAG {
            res = self._handle_start_config(attributes);
        } else if name == OBJECT_TAG {
            res = self._handle_start_config_object(attributes);
        } else if name == VOLUME_TAG {
            res = self._handle_start_config_volume(attributes);
        } else if name == MESH_TAG {
            res = self._handle_start_config_volume_mesh(attributes);
        } else if name == METADATA_TAG {
            res = self._handle_start_config_metadata(attributes);
        }

        // 3mf.cpp:1528-1529
        if !res {
            self._stop_xml_parser("");
        }
    }

    /// 3mf.cpp:1532-1552
    /// C++: `void _handle_end_config_xml_element(const char* name)`
    fn _handle_end_config_xml_element(&mut self, name: &str) {
        // 3mf.cpp:1534-1535
        if !self.m_xml_parser {
            return;
        }

        // 3mf.cpp:1537
        let mut res = true;

        // 3mf.cpp:1539-1548
        if name == CONFIG_TAG {
            res = self._handle_end_config();
        } else if name == OBJECT_TAG {
            res = self._handle_end_config_object();
        } else if name == VOLUME_TAG {
            res = self._handle_end_config_volume();
        } else if name == MESH_TAG {
            res = self._handle_end_config_volume_mesh();
        } else if name == METADATA_TAG {
            res = self._handle_end_config_metadata();
        }

        // 3mf.cpp:1550-1551
        if !res {
            self._stop_xml_parser("");
        }
    }

    /// 3mf.cpp:1554-1558
    fn _handle_start_model(&mut self, attributes: &[(String, String)]) -> bool {
        // 3mf.cpp:1556
        self.m_unit_factor = get_unit_factor(&get_attribute_value_string(attributes, UNIT_ATTR));
        true
    }

    /// 3mf.cpp:1560-1588
    fn _handle_end_model(&mut self) -> bool {
        // deletes all non-built or non-instanced objects
        // 3mf.cpp:1563-1571
        // (std::map iteration order == BTreeMap; the index snapshot mirrors the
        //  C++ map values, which are NOT updated when objects are deleted.)
        let object_entries: Vec<i32> = self.m_objects.values().copied().collect();
        for object_second in object_entries {
            // 3mf.cpp:1564-1567
            if object_second >= self.m_model.objects.len() as i32 {
                self.base.add_error("Unable to find object");
                return false;
            }
            // 3mf.cpp:1568-1570 — delete_object(model_object) when it has no
            // instances. The C++ holds ModelInstance POINTERS in m_instances,
            // which stay valid across the vector erase; the Rust port stores
            // indices and decrements them past the deletion point to preserve
            // identity (deleted objects have no instances, so no entry refers
            // to them).
            let idx = object_second as usize;
            if self.m_model.objects[idx].instances.is_empty() {
                self.m_model.objects.remove(idx);
                for instance in &mut self.m_instances {
                    if instance.object_idx > idx {
                        instance.object_idx -= 1;
                    }
                }
            }
        }

        // 3mf.cpp:1573-1578
        if self.m_version == 0 {
            // if the 3mf was not produced by PrusaSlicer and there is only one object,
            // set the object name to match the filename
            if self.m_model.objects.len() == 1 {
                self.m_model.objects[0].name = self.m_name.clone();
            }
        }

        // applies instances' matrices
        // 3mf.cpp:1581-1585
        for i in 0..self.m_instances.len() {
            let object_idx = self.m_instances[i].object_idx;
            let instance_idx = self.m_instances[i].instance_idx;
            // if (instance.instance != nullptr && instance.instance->get_object() != nullptr)
            if object_idx < self.m_model.objects.len()
                && instance_idx < self.m_model.objects[object_idx].instances.len()
            {
                // apply the transform to the instance
                let transform = self.m_instances[i].transform;
                Self::_apply_transform(
                    &mut self.m_model.objects[object_idx].instances[instance_idx],
                    &transform,
                );
            }
        }

        // 3mf.cpp:1587
        true
    }

    /// 3mf.cpp:1590-1594
    fn _handle_start_resources(&mut self, _attributes: &[(String, String)]) -> bool {
        // do nothing
        true
    }

    /// 3mf.cpp:1596-1600
    fn _handle_end_resources(&mut self) -> bool {
        // do nothing
        true
    }

    /// 3mf.cpp:1602-1625
    fn _handle_start_object(&mut self, attributes: &[(String, String)]) -> bool {
        // reset current data
        // 3mf.cpp:1605
        self.m_curr_object.reset();

        // 3mf.cpp:1607
        if is_valid_object_type(&get_attribute_value_string(attributes, TYPE_ATTR)) {
            // create new object (it may be removed later if no instances are generated from it)
            // 3mf.cpp:1609-1614
            self.m_curr_object.model_object_idx = self.m_model.objects.len() as i32;
            // m_model->add_object() — new empty ModelObject (no instances, no volumes).
            self.m_model.objects.push(ModelObject {
                name: String::new(),
                mesh: TriangleMesh::new(),
                instances: Vec::new(),
                config: ObjectConfig::default(),
                printable: true,
            });
            self.m_curr_object.object = Some(self.m_model.objects.len() - 1);
            // (add_object cannot return nullptr here.)

            // set object data
            // 3mf.cpp:1617-1619
            let object_idx = self.m_curr_object.object.unwrap();
            self.m_model.objects[object_idx].name =
                get_attribute_value_string(attributes, NAME_ATTR);
            if self.m_model.objects[object_idx].name.is_empty() {
                self.m_model.objects[object_idx].name =
                    format!("{}_{}", self.m_name, self.m_model.objects.len());
            }

            // 3mf.cpp:1621
            self.m_curr_object.id = get_attribute_value_int(attributes, ID_ATTR);
        }

        // 3mf.cpp:1624
        true
    }

    /// 3mf.cpp:1627-1666
    fn _handle_end_object(&mut self) -> bool {
        // 3mf.cpp:1629
        if let Some(object_idx) = self.m_curr_object.object {
            // 3mf.cpp:1630
            if self.m_curr_object.geometry.empty() {
                // no geometry defined
                // remove the object from the model
                // 3mf.cpp:1633 — m_model->delete_object(m_curr_object.object):
                // the current object is the most recently added one; erase it.
                self.m_model.objects.remove(object_idx);

                // 3mf.cpp:1635-1644
                if self.m_curr_object.components.is_empty() {
                    // no components defined -> invalid object, delete it
                    // 3mf.cpp:1637-1639
                    self.m_objects.remove(&self.m_curr_object.id);
                    // 3mf.cpp:1641-1643
                    self.m_objects_aliases.remove(&self.m_curr_object.id);
                } else {
                    // adds components to aliases
                    // 3mf.cpp:1647
                    self.m_objects_aliases.insert(
                        self.m_curr_object.id,
                        std::mem::take(&mut self.m_curr_object.components),
                    );
                }
            } else {
                // geometry defined, store it for later use
                // 3mf.cpp:1651
                self.m_geometries.insert(
                    self.m_curr_object.id,
                    std::mem::take(&mut self.m_curr_object.geometry),
                );

                // stores the object for later use
                // 3mf.cpp:1654-1661
                if !self.m_objects.contains_key(&self.m_curr_object.id) {
                    self.m_objects
                        .insert(self.m_curr_object.id, self.m_curr_object.model_object_idx);
                    // aliases itself
                    self.m_objects_aliases.insert(
                        self.m_curr_object.id,
                        vec![Component::from_id(self.m_curr_object.id)],
                    );
                } else {
                    self.base.add_error("Found object with duplicate id");
                    return false;
                }
            }
        }

        // 3mf.cpp:1665
        true
    }

    /// 3mf.cpp:1668-1673
    fn _handle_start_mesh(&mut self, _attributes: &[(String, String)]) -> bool {
        // reset current geometry
        // 3mf.cpp:1671
        self.m_curr_object.geometry.reset();
        true
    }

    /// 3mf.cpp:1675-1679
    fn _handle_end_mesh(&mut self) -> bool {
        // do nothing
        true
    }

    /// 3mf.cpp:1681-1686
    fn _handle_start_vertices(&mut self, _attributes: &[(String, String)]) -> bool {
        // reset current vertices
        // 3mf.cpp:1684
        self.m_curr_object.geometry.vertices.clear();
        true
    }

    /// 3mf.cpp:1688-1692
    fn _handle_end_vertices(&mut self) -> bool {
        // do nothing
        true
    }

    /// 3mf.cpp:1694-1703
    fn _handle_start_vertex(&mut self, attributes: &[(String, String)]) -> bool {
        // appends the vertex coordinates
        // missing values are set equal to ZERO
        // 3mf.cpp:1698-1701
        self.m_curr_object.geometry.vertices.push(Vec3f::new(
            self.m_unit_factor * get_attribute_value_float(attributes, X_ATTR),
            self.m_unit_factor * get_attribute_value_float(attributes, Y_ATTR),
            self.m_unit_factor * get_attribute_value_float(attributes, Z_ATTR),
        ));
        true
    }

    /// 3mf.cpp:1705-1709
    fn _handle_end_vertex(&mut self) -> bool {
        // do nothing
        true
    }

    /// 3mf.cpp:1711-1716
    fn _handle_start_triangles(&mut self, _attributes: &[(String, String)]) -> bool {
        // reset current triangles
        // 3mf.cpp:1714
        self.m_curr_object.geometry.triangles.clear();
        true
    }

    /// 3mf.cpp:1718-1722
    fn _handle_end_triangles(&mut self) -> bool {
        // do nothing
        true
    }

    /// 3mf.cpp:1724-1744
    fn _handle_start_triangle(&mut self, attributes: &[(String, String)]) -> bool {
        // we are ignoring the following attributes:
        // p1
        // p2
        // p3
        // pid
        // see specifications

        // appends the triangle's vertices indices
        // missing values are set equal to ZERO
        // 3mf.cpp:1735-1738
        self.m_curr_object.geometry.triangles.push(Vec3crd::new(
            get_attribute_value_int(attributes, V1_ATTR),
            get_attribute_value_int(attributes, V2_ATTR),
            get_attribute_value_int(attributes, V3_ATTR),
        ));

        // 3mf.cpp:1740-1742
        self.m_curr_object
            .geometry
            .custom_supports
            .push(get_attribute_value_string(attributes, CUSTOM_SUPPORTS_ATTR));
        self.m_curr_object
            .geometry
            .custom_seam
            .push(get_attribute_value_string(attributes, CUSTOM_SEAM_ATTR));
        self.m_curr_object
            .geometry
            .mmu_segmentation
            .push(get_attribute_value_string(
                attributes,
                MMU_SEGMENTATION_ATTR,
            ));
        true
    }

    /// 3mf.cpp:1746-1750
    fn _handle_end_triangle(&mut self) -> bool {
        // do nothing
        true
    }

    /// 3mf.cpp:1752-1757
    fn _handle_start_components(&mut self, _attributes: &[(String, String)]) -> bool {
        // reset current components
        // 3mf.cpp:1755
        self.m_curr_object.components.clear();
        true
    }

    /// 3mf.cpp:1759-1763
    fn _handle_end_components(&mut self) -> bool {
        // do nothing
        true
    }

    /// 3mf.cpp:1765-1782
    fn _handle_start_component(&mut self, attributes: &[(String, String)]) -> bool {
        // 3mf.cpp:1767-1768
        let object_id = get_attribute_value_int(attributes, OBJECTID_ATTR);
        let transform = get_transform_from_3mf_specs_string(&get_attribute_value_string(
            attributes,
            TRANSFORM_ATTR,
        ));

        // 3mf.cpp:1770-1777
        if !self.m_objects.contains_key(&object_id) {
            if !self.m_objects_aliases.contains_key(&object_id) {
                self.base
                    .add_error("Found component with invalid object id");
                return false;
            }
        }

        // 3mf.cpp:1779
        self.m_curr_object
            .components
            .push(Component::new(object_id, transform));

        // 3mf.cpp:1781
        true
    }

    /// 3mf.cpp:1784-1788
    fn _handle_end_component(&mut self) -> bool {
        // do nothing
        true
    }

    /// 3mf.cpp:1790-1794
    fn _handle_start_build(&mut self, _attributes: &[(String, String)]) -> bool {
        // do nothing
        true
    }

    /// 3mf.cpp:1796-1800
    fn _handle_end_build(&mut self) -> bool {
        // do nothing
        true
    }

    /// 3mf.cpp:1802-1816
    fn _handle_start_item(&mut self, attributes: &[(String, String)]) -> bool {
        // we are ignoring the following attributes
        // thumbnail
        // partnumber
        // pid
        // pindex
        // see specifications

        // 3mf.cpp:1811-1813
        let object_id = get_attribute_value_int(attributes, OBJECTID_ATTR);
        let transform = get_transform_from_3mf_specs_string(&get_attribute_value_string(
            attributes,
            TRANSFORM_ATTR,
        ));
        let printable = get_attribute_value_bool(attributes, PRINTABLE_ATTR);

        // 3mf.cpp:1815
        self._create_object_instance(object_id, &transform, printable, 1)
    }

    /// 3mf.cpp:1818-1822
    fn _handle_end_item(&mut self) -> bool {
        // do nothing
        true
    }

    /// 3mf.cpp:1824-1833
    fn _handle_start_metadata(&mut self, attributes: &[(String, String)]) -> bool {
        // 3mf.cpp:1826
        self.m_curr_characters.clear();

        // 3mf.cpp:1828-1830
        let name = get_attribute_value_string(attributes, NAME_ATTR);
        if !name.is_empty() {
            self.m_curr_metadata_name = name;
        }

        true
    }

    /// 3mf.cpp:1841-1871
    /// C++: `bool _handle_end_metadata()` — throws `version_error`.
    fn _handle_end_metadata(&mut self) -> std::result::Result<bool, VersionError> {
        // 3mf.cpp:1843-1850
        if self.m_curr_metadata_name == SLIC3RPE_3MF_VERSION {
            self.m_version = atoi(&self.m_curr_characters) as u32;
            if self.m_check_version && (self.m_version > VERSION_3MF_COMPATIBLE) {
                // 3mf.cpp:1848-1849
                let msg = format!(
                    "The selected 3mf file has been saved with a newer version of {} and is not compatible.",
                    SLIC3R_APP_NAME
                );
                return Err(VersionError(msg));
            }
        } else if self.m_curr_metadata_name == "Application" {
            // Generator application of the 3MF.
            // SLIC3R_APP_KEY - SLIC3R_VERSION
            // 3mf.cpp:1854-1855
            if self.m_curr_characters.starts_with("PrusaSlicer-") {
                self.m_prusaslicer_generator_version = Semver::parse(&self.m_curr_characters[12..]);
            }
        } else if self.m_curr_metadata_name == SLIC3RPE_FDM_SUPPORTS_PAINTING_VERSION {
            // 3mf.cpp:1857-1859
            self.m_fdm_supports_painting_version = atoi(&self.m_curr_characters) as u32;
            check_painting_version(
                self.m_fdm_supports_painting_version,
                FDM_SUPPORTS_PAINTING_VERSION,
                "The selected 3MF contains FDM supports painted object using a newer version of PrusaSlicer and is not compatible.",
            )?;
        } else if self.m_curr_metadata_name == SLIC3RPE_SEAM_PAINTING_VERSION {
            // 3mf.cpp:1861-1863
            self.m_seam_painting_version = atoi(&self.m_curr_characters) as u32;
            check_painting_version(
                self.m_seam_painting_version,
                SEAM_PAINTING_VERSION,
                "The selected 3MF contains seam painted object using a newer version of PrusaSlicer and is not compatible.",
            )?;
        } else if self.m_curr_metadata_name == SLIC3RPE_MM_PAINTING_VERSION {
            // 3mf.cpp:1865-1867
            self.m_mm_painting_version = atoi(&self.m_curr_characters) as u32;
            check_painting_version(
                self.m_mm_painting_version,
                MM_PAINTING_VERSION,
                "The selected 3MF contains multi-material painted object using a newer version of PrusaSlicer and is not compatible.",
            )?;
        }

        // 3mf.cpp:1870
        Ok(true)
    }

    /// 3mf.cpp:1873-1917
    /// C++: `bool _create_object_instance(int object_id, const Transform3d& transform, const bool printable, unsigned int recur_counter)`
    fn _create_object_instance(
        &mut self,
        object_id: i32,
        transform: &Transform3d,
        printable: bool,
        recur_counter: u32,
    ) -> bool {
        // 3mf.cpp:1875
        const MAX_RECURSIONS: u32 = 10;

        // escape from circular aliasing
        // 3mf.cpp:1878-1881
        if recur_counter > MAX_RECURSIONS {
            self.base.add_error("Too many recursions");
            return false;
        }

        // 3mf.cpp:1883-1887
        let aliases = match self.m_objects_aliases.get(&object_id) {
            Some(a) => a.clone(),
            None => {
                self.base.add_error("Found item with invalid object id");
                return false;
            }
        };

        // 3mf.cpp:1889-1907
        if aliases.len() == 1 && aliases[0].object_id == object_id {
            // aliasing to itself

            // 3mf.cpp:1892-1896
            let object_item = self.m_objects.get(&object_id).copied();
            match object_item {
                None => {
                    self.base.add_error("Found invalid object");
                    return false;
                }
                Some(-1) => {
                    self.base.add_error("Found invalid object");
                    return false;
                }
                Some(model_object_idx) => {
                    // 3mf.cpp:1898-1902 — ModelInstance* instance = ...->add_instance();
                    let object_idx = model_object_idx as usize;
                    self.m_model.objects[object_idx]
                        .instances
                        .push(crate::model::Instance::new());
                    let instance_idx = self.m_model.objects[object_idx].instances.len() - 1;
                    // 3mf.cpp:1903
                    self.m_model.objects[object_idx].instances[instance_idx].printable = printable;

                    // 3mf.cpp:1905
                    self.m_instances
                        .push(Instance::new(object_idx, instance_idx, *transform));
                }
            }
        } else {
            // recursively process nested components
            // 3mf.cpp:1909-1913
            for component in &aliases {
                if !self._create_object_instance(
                    component.object_id,
                    &(transform * component.transform),
                    printable,
                    recur_counter + 1,
                ) {
                    return false;
                }
            }
        }

        // 3mf.cpp:1916
        true
    }

    /// 3mf.cpp:1919-1927
    /// C++: `void _apply_transform(ModelInstance& instance, const Transform3d& transform)`
    fn _apply_transform(instance: &mut crate::model::Instance, transform: &Transform3d) {
        // 3mf.cpp:1921
        let t = Transformation::from_transform(*transform);
        // invalid scale value, return
        // 3mf.cpp:1922-1924 — Eigen `.all()`: every coefficient != 0.
        let scaling_factor = t.get_scaling_factor();
        if !(scaling_factor.x != 0.0 && scaling_factor.y != 0.0 && scaling_factor.z != 0.0) {
            return;
        }

        // 3mf.cpp:1926 — instance.set_transformation(t);
        // BLOCKED(model): the simplified `model::Instance` stores only an
        // offset, a Z rotation (degrees) and per-axis scale; the X/Y rotation
        // and mirror components of the full Transformation are dropped
        // (same convention as format::amf, AMF.cpp:874-878 port).
        let offset = t.get_offset();
        let rotation: Vec3d = t.get_rotation();
        instance.position = Point3F::new(offset.x, offset.y, offset.z);
        instance.rotation_z = rotation.z.to_degrees();
        instance.scale = [scaling_factor.x, scaling_factor.y, scaling_factor.z];
    }

    /// 3mf.cpp:1929-1933
    fn _handle_start_config(&mut self, _attributes: &[(String, String)]) -> bool {
        // do nothing
        true
    }

    /// 3mf.cpp:1935-1939
    fn _handle_end_config(&mut self) -> bool {
        // do nothing
        true
    }

    /// 3mf.cpp:1941-1956
    fn _handle_start_config_object(&mut self, attributes: &[(String, String)]) -> bool {
        // 3mf.cpp:1943-1948
        let object_id = get_attribute_value_int(attributes, ID_ATTR);
        if self.m_objects_metadata.contains_key(&object_id) {
            self.base.add_error("Found duplicated object id");
            return false;
        }

        // Added because of github #3435, currently not used by PrusaSlicer
        // 3mf.cpp:1951 — int instances_count_id = get_attribute_value_int(attributes, num_attributes, INSTANCESCOUNT_ATTR);

        // 3mf.cpp:1953-1954
        self.m_objects_metadata
            .insert(object_id, ObjectMetadata::default());
        self.m_curr_config.object_id = object_id;
        true
    }

    /// 3mf.cpp:1958-1962
    fn _handle_end_config_object(&mut self) -> bool {
        // do nothing
        true
    }

    /// 3mf.cpp:1964-1979
    fn _handle_start_config_volume(&mut self, attributes: &[(String, String)]) -> bool {
        // 3mf.cpp:1966-1970
        let object = match self
            .m_objects_metadata
            .get_mut(&self.m_curr_config.object_id)
        {
            Some(o) => o,
            None => {
                self.base
                    .add_error("Cannot assign volume to a valid object");
                return false;
            }
        };

        // 3mf.cpp:1972
        self.m_curr_config.volume_id = object.volumes.len() as i32;

        // 3mf.cpp:1974-1975
        let first_triangle_id = get_attribute_value_int(attributes, FIRST_TRIANGLE_ID_ATTR) as u32;
        let last_triangle_id = get_attribute_value_int(attributes, LAST_TRIANGLE_ID_ATTR) as u32;

        // 3mf.cpp:1977
        object
            .volumes
            .push(VolumeMetadata::new(first_triangle_id, last_triangle_id));
        true
    }

    /// 3mf.cpp:1981-2004
    fn _handle_start_config_volume_mesh(&mut self, attributes: &[(String, String)]) -> bool {
        // 3mf.cpp:1983-1987
        let object = match self
            .m_objects_metadata
            .get_mut(&self.m_curr_config.object_id)
        {
            Some(o) => o,
            None => {
                self.base
                    .add_error("Cannot assign volume mesh to a valid object");
                return false;
            }
        };
        // 3mf.cpp:1988-1991
        if object.volumes.is_empty() {
            self.base.add_error("Cannot assign mesh to a valid olume");
            return false;
        }

        // 3mf.cpp:1993
        let volume = object.volumes.last_mut().unwrap();

        // 3mf.cpp:1995-1999
        let edges_fixed = get_attribute_value_int(attributes, MESH_STAT_EDGES_FIXED);
        let degenerate_facets = get_attribute_value_int(attributes, MESH_STAT_DEGENERATED_FACETS);
        let facets_removed = get_attribute_value_int(attributes, MESH_STAT_FACETS_REMOVED);
        let facets_reversed = get_attribute_value_int(attributes, MESH_STAT_FACETS_RESERVED);
        let backwards_edges = get_attribute_value_int(attributes, MESH_STAT_BACKWARDS_EDGES);

        // 3mf.cpp:2001
        volume.mesh_stats = RepairedMeshErrors {
            edges_fixed,
            degenerate_facets,
            facets_removed,
            facets_reversed,
            backwards_edges,
        };

        // 3mf.cpp:2003
        true
    }

    /// 3mf.cpp:2006-2010
    fn _handle_end_config_volume(&mut self) -> bool {
        // do nothing
        true
    }

    /// 3mf.cpp:2012-2016
    fn _handle_end_config_volume_mesh(&mut self) -> bool {
        // do nothing
        true
    }

    /// 3mf.cpp:2018-2063
    fn _handle_start_config_metadata(&mut self, attributes: &[(String, String)]) -> bool {
        // 3mf.cpp:2020-2024
        if !self
            .m_objects_metadata
            .contains_key(&self.m_curr_config.object_id)
        {
            self.base
                .add_error("Cannot assign metadata to valid object id");
            return false;
        }

        // 3mf.cpp:2026-2028
        let type_ = get_attribute_value_string(attributes, TYPE_ATTR);
        let key = get_attribute_value_string(attributes, KEY_ATTR);
        let value = get_attribute_value_string(attributes, VALUE_ATTR);

        // filter the prusa model config keys
        // 3mf.cpp:2031-2043
        let valid_keys = [
            "name",
            "volume_type",
            "matrix",
            "source_file",
            "source_object_id",
            "source_volume_id",
            "source_offset_x",
            "source_offset_y",
            "source_offset_z",
            "extruder",
            "modifier",
        ];

        // 3mf.cpp:2045-2049
        if !valid_keys.contains(&key.as_str()) {
            // do nothing if not valid keys
            return true;
        }

        let object = self
            .m_objects_metadata
            .get_mut(&self.m_curr_config.object_id)
            .unwrap();

        // 3mf.cpp:2051-2060
        if type_ == OBJECT_TYPE {
            object.metadata.push(Metadata::new(key, value));
        } else if type_ == VOLUME_TYPE {
            if (self.m_curr_config.volume_id as usize) < object.volumes.len() {
                object.volumes[self.m_curr_config.volume_id as usize]
                    .metadata
                    .push(Metadata::new(key, value));
            }
        } else {
            self.base.add_error("Found invalid metadata type");
            return false;
        }

        // 3mf.cpp:2062
        true
    }

    /// 3mf.cpp:2065-2069
    fn _handle_end_config_metadata(&mut self) -> bool {
        // do nothing
        true
    }

    /// 3mf.cpp:2071-2213
    /// C++: `bool _generate_volumes(ModelObject& object, const Geometry& geometry, const ObjectMetadata::VolumeMetadataList& volumes, ConfigSubstitutionContext& config_substitutions)`
    /// Implemented as an associated function with the needed members threaded
    /// explicitly so the disjoint-field borrows check (`&mut self` would alias
    /// `m_geometries` / `m_model`).
    fn _generate_volumes(
        base: &mut _3MF_Base,
        m_prusaslicer_generator_version: &Option<Semver>,
        m_version: u32,
        object: &mut ModelObject,
        geometry: &Geometry,
        volumes: &[VolumeMetadata],
        _config_substitutions: &mut ConfigSubstitutionContext,
    ) -> bool {
        // 3mf.cpp:2073-2076 — if (!object.volumes.empty())
        // BLOCKED(model): no ModelVolume list; the merged object mesh is empty
        // exactly when no volumes have been generated yet.
        if !object.mesh.is_empty() {
            base.add_error("Found invalid volumes count");
            return false;
        }

        // 3mf.cpp:2078-2079
        let geo_tri_count = geometry.triangles.len() as u32;
        let mut renamed_volumes_count: u32 = 0;

        // 3mf.cpp:2081-2210
        for volume_data in volumes {
            // 3mf.cpp:2082-2085
            if geo_tri_count <= volume_data.first_triangle_id
                || geo_tri_count <= volume_data.last_triangle_id
                || volume_data.last_triangle_id < volume_data.first_triangle_id
            {
                base.add_error("Found invalid triangle id");
                return false;
            }

            // 3mf.cpp:2087-2096 — extract the volume transformation from the
            // volume's metadata, if present
            let mut volume_matrix_to_object = Transform3d::identity();
            let mut has_transform = false;
            for metadata in &volume_data.metadata {
                if metadata.key == MATRIX_KEY {
                    volume_matrix_to_object = transform3d_from_string(&metadata.value);
                    has_transform = !transform_is_approx(
                        &volume_matrix_to_object,
                        &Transform3d::identity(),
                        1e-10,
                    );
                    break;
                }
            }

            // splits volume out of imported geometry
            // 3mf.cpp:2099-2105
            let mut its = indexed_triangle_set {
                indices: geometry.triangles[volume_data.first_triangle_id as usize
                    ..=volume_data.last_triangle_id as usize]
                    .to_vec(),
                vertices: Vec::new(),
            };
            let triangles_count = its.indices.len();
            if triangles_count == 0 {
                base.add_error("An empty triangle mesh found");
                return false;
            }

            // 3mf.cpp:2107-2126
            {
                // 3mf.cpp:2108-2119
                let mut min_id = its.indices[0][0];
                let mut max_id = min_id;
                for face in &its.indices {
                    for k in 0..3 {
                        let tri_id = face[k];
                        if tri_id < 0 || tri_id >= geometry.vertices.len() as i32 {
                            base.add_error("Found invalid vertex id");
                            return false;
                        }
                        min_id = min_id.min(tri_id);
                        max_id = max_id.max(tri_id);
                    }
                }
                // 3mf.cpp:2120
                its.vertices = geometry.vertices[min_id as usize..=max_id as usize].to_vec();

                // rebase indices to the current vertices list
                // 3mf.cpp:2122-2125
                for face in &mut its.indices {
                    for k in 0..3 {
                        face[k] -= min_id;
                    }
                }
            }

            // 3mf.cpp:2128-2133
            if let Some(generator_version) = m_prusaslicer_generator_version {
                if *generator_version >= Semver::parse("2.4.0-alpha1").unwrap()
                    && *generator_version < Semver::parse("2.4.0-alpha3").unwrap()
                {
                    // PrusaSlicer 2.4.0-alpha2 contained a bug, where all vertices of a single object were saved for each volume the object contained.
                    // Remove the vertices, that are not referenced by any face.
                    its_compactify_vertices(&mut its, true);
                }
            }

            // 3mf.cpp:2135 — TriangleMesh triangle_mesh(std::move(its), volume_data.mesh_stats);
            // BLOCKED(model): the simplified TriangleMesh carries no
            // RepairedMeshErrors; `volume_data.mesh_stats` is parsed but cannot
            // be attached to the mesh.
            let mut triangle_mesh = TriangleMesh::from_parts(
                its.vertices
                    .iter()
                    .map(|v| Point3F::new(v[0] as f64, v[1] as f64, v[2] as f64))
                    .collect(),
                its.indices
                    .iter()
                    .map(|f| Triangle::new(f[0] as u32, f[1] as u32, f[2] as u32))
                    .collect(),
            );

            // 3mf.cpp:2137-2146
            if m_version == 0 {
                // if the 3mf was not produced by PrusaSlicer and there is only one instance,
                // bake the transformation into the geometry to allow the reload from disk command
                // to work properly
                if object.instances.len() == 1 {
                    // 3mf.cpp:2142 — triangle_mesh.transform(get_transformation().get_matrix(), false);
                    // (the second argument `fix_left_handed=false` has no
                    //  counterpart in the simplified TriangleMesh::transform.)
                    let matrix = object.instances[0].transform();
                    triangle_mesh.transform(&matrix);
                    // 3mf.cpp:2143 — set_transformation(Slic3r::Geometry::Transformation());
                    object.instances[0].position = Point3F::new(0.0, 0.0, 0.0);
                    object.instances[0].rotation_z = 0.0;
                    object.instances[0].scale = [1.0, 1.0, 1.0];
                    //FIXME do the mesh fixing?
                }
            }
            // 3mf.cpp:2147-2148 — if (triangle_mesh.volume() < 0) flip_triangles();
            if triangle_mesh.volume() < 0.0 {
                // TriangleMesh::flip_triangles() == its_flip_triangles: swap face(1), face(2).
                for tri in triangle_mesh.indices_mut() {
                    tri.indices.swap(1, 2);
                }
            }

            // 3mf.cpp:2150 — ModelVolume* volume = object.add_volume(std::move(triangle_mesh));
            // BLOCKED(model): no ModelVolume; merge this volume's mesh into the
            // object's single TriangleMesh (same convention as format::amf).
            {
                let base_idx = object.mesh.vertices().len() as u32;
                let mut vertices = object.mesh.vertices().to_vec();
                let mut indices = object.mesh.indices().to_vec();
                vertices.extend(triangle_mesh.vertices().iter().copied());
                indices.extend(triangle_mesh.indices().iter().map(|t| {
                    Triangle::new(
                        base_idx + t.indices[0],
                        base_idx + t.indices[1],
                        base_idx + t.indices[2],
                    )
                }));
                object.mesh = TriangleMesh::from_parts(vertices, indices);
            }

            // stores the volume matrix taken from the metadata, if present
            // 3mf.cpp:2151-2153 — volume->source.transform = Transformation(volume_matrix_to_object);
            // BLOCKED(model): no ModelVolume::source. `has_transform` is
            // computed faithfully above but its target field does not exist.
            let _ = has_transform;

            // recreate custom supports, seam and mmu segmentation from previously loaded attribute
            // 3mf.cpp:2155-2173 —
            //   volume->supported_facets / seam_facets / mmu_segmentation_facets
            //   .set_triangle_from_string(i, geometry.custom_supports[first_triangle_id + i]) ...
            // BLOCKED(model): no per-volume FacetsAnnotation on the simplified
            // Model; the strings were parsed into `geometry.custom_supports` /
            // `custom_seam` / `mmu_segmentation` but cannot be applied.

            // apply the remaining volume's metadata
            // 3mf.cpp:2175-2201
            // BLOCKED(model): no per-volume name; track the name string locally
            // so the `volume->name.empty()` rename test below mirrors C++.
            let mut volume_name = String::new();
            for metadata in &volume_data.metadata {
                if metadata.key == NAME_KEY {
                    // 3mf.cpp:2177-2178 — volume->name = metadata.value;
                    volume_name = metadata.value.clone();
                } else if metadata.key == MODIFIER_KEY && metadata.value == "1" {
                    // 3mf.cpp:2179-2180 — volume->set_type(PARAMETER_MODIFIER);
                    // BLOCKED(model): no per-volume type.
                } else if metadata.key == VOLUME_TYPE_KEY {
                    // 3mf.cpp:2181-2182 — volume->set_type(type_from_string(metadata.value));
                    // BLOCKED(model): no per-volume type (the parser
                    // `type_from_string` is ported above).
                } else if metadata.key == SOURCE_FILE_KEY
                    || metadata.key == SOURCE_OBJECT_ID_KEY
                    || metadata.key == SOURCE_VOLUME_ID_KEY
                    || metadata.key == SOURCE_OFFSET_X_KEY
                    || metadata.key == SOURCE_OFFSET_Y_KEY
                    || metadata.key == SOURCE_OFFSET_Z_KEY
                    || metadata.key == SOURCE_IN_INCHES
                    || metadata.key == SOURCE_IN_METERS
                {
                    // 3mf.cpp:2183-2198 — volume->source.* assignments.
                    // BLOCKED(model): no ModelVolume::source.
                } else {
                    // 3mf.cpp:2199-2200 — volume->config.set_deserialize(key, value, config_substitutions);
                    // BLOCKED(config): reflective config unported.
                }
            }

            // this may happen for 3mf saved by 3rd part softwares
            // 3mf.cpp:2203-2209 — if (volume->name.empty()) { volume->name = object.name; ... }
            // BLOCKED(model): no per-volume name; the counter is incremented only
            // when the volume name is empty, mirroring the C++ control flow.
            if volume_name.is_empty() {
                // volume->name = object.name; (+ "_N" when renamed_volumes_count > 0)
                renamed_volumes_count += 1;
            }
            let _ = renamed_volumes_count;
        }

        // 3mf.cpp:2212
        true
    }

    // 3mf.cpp:2215-2248 — the five static XMLCALL trampolines
    // (_handle_start_model_xml_element / _handle_end_model_xml_element /
    //  _handle_model_xml_characters / _handle_start_config_xml_element /
    //  _handle_end_config_xml_element with `void* userData`) are realized by
    // the dispatch loops in `_extract_model_from_archive` and
    // `_extract_model_config_from_archive`.

    /// 3mf.cpp:384-388 — `_3MF_Base::log_errors()` (public on the importer).
    pub fn log_errors(&self) {
        self.base.log_errors();
    }
}

/// 3mf.cpp:1835-1839
/// C++: `inline static void check_painting_version(unsigned int loaded_version, unsigned int highest_supported_version, const std::string &error_msg)`
/// — throws `version_error`.
#[inline]
fn check_painting_version(
    loaded_version: u32,
    highest_supported_version: u32,
    error_msg: &str,
) -> std::result::Result<(), VersionError> {
    if loaded_version > highest_supported_version {
        return Err(VersionError(error_msg.to_string()));
    }
    Ok(())
}

// ---------------------------------------------------------------------------
// _3MF_Exporter (3mf.cpp:2250-3234)
// ---------------------------------------------------------------------------

/// 3mf.cpp:2252-2264
#[derive(Debug, Clone)]
struct BuildItem {
    /// 3mf.cpp:2254
    id: u32,
    /// 3mf.cpp:2255
    transform: Transform3d,
    /// 3mf.cpp:2256
    printable: bool,
}

impl BuildItem {
    /// 3mf.cpp:2258-2263
    fn new(id: u32, transform: Transform3d, printable: bool) -> Self {
        Self {
            id,
            transform,
            printable,
        }
    }
}

/// 3mf.cpp:2266-2278
#[derive(Debug, Clone)]
struct Offsets {
    /// 3mf.cpp:2268
    first_vertex_id: u32,
    /// 3mf.cpp:2269
    first_triangle_id: u32,
    /// 3mf.cpp:2270
    last_triangle_id: u32,
}

impl Offsets {
    /// 3mf.cpp:2272-2277 — first/last_triangle_id are initialized to
    /// `(unsigned int)-1`.
    fn new(first_vertex_id: u32) -> Self {
        Self {
            first_vertex_id,
            first_triangle_id: u32::MAX,
            last_triangle_id: u32::MAX,
        }
    }
}

// 3mf.cpp:2280 — `typedef std::map<const ModelVolume*, Offsets> VolumeToOffsetsMap;`
// BLOCKED(model): no ModelVolume; the simplified model exposes exactly one
// merged volume per object, keyed by its index (always 0).
type VolumeToOffsetsMap = BTreeMap<usize, Offsets>;

/// 3mf.cpp:2282-2291
struct ObjectData {
    /// 3mf.cpp:2284 — `ModelObject* object;` stored as the index into
    /// `model.objects`.
    object: usize,
    /// 3mf.cpp:2285
    volumes_offsets: VolumeToOffsetsMap,
}

impl ObjectData {
    /// 3mf.cpp:2287-2290
    fn new(object: usize) -> Self {
        Self {
            object,
            volumes_offsets: BTreeMap::new(),
        }
    }
}

// 3mf.cpp:2293 — typedef std::vector<BuildItem> BuildItemsList;
type BuildItemsList = Vec<BuildItem>;
// 3mf.cpp:2294 — typedef std::map<int, ObjectData> IdToObjectDataMap;
type IdToObjectDataMap = BTreeMap<i32, ObjectData>;

// SLIC3R_APP_KEY (libslic3r_version.h / version.inc) — "BambuStudio", used in
// the "Application" metadata at 3mf.cpp:2558.
const SLIC3R_APP_KEY: &str = "BambuStudio";

/// 3mf.cpp:2503-2512
/// C++: `static void reset_stream(std::stringstream &stream)` — clears the
/// stream and sets `std::setprecision(std::numeric_limits<float>::max_digits10)`.
/// The Rust port returns the stream precision (9 significant digits) used by
/// `general_format` at each float insertion.
// https://en.cppreference.com/w/cpp/types/numeric_limits/max_digits10
// Conversion of a floating-point value to text and back is exact as long as at least max_digits10 were used (9 for float, 17 for double).
// It is guaranteed to produce the same floating-point value, even though the intermediate text representation is not exact.
// The default value of std::stream precision is 6 digits only!
const FLOAT_MAX_DIGITS10: usize = 9;
const DOUBLE_MAX_DIGITS10: usize = 17;

fn reset_stream(stream: &mut String) {
    stream.clear();
}

/// 3mf.cpp:2250-2318 — `class _3MF_Exporter : public _3MF_Base`
#[allow(non_camel_case_types)]
pub struct _3MF_Exporter {
    /// _3MF_Base (3mf.cpp:2250)
    base: _3MF_Base,
    /// 3mf.cpp:2296
    m_fullpath_sources: bool,
    /// 3mf.cpp:2297
    m_zip64: bool,
}

impl _3MF_Exporter {
    pub fn new() -> Self {
        Self {
            base: _3MF_Base::new(),
            m_fullpath_sources: true,
            m_zip64: true,
        }
    }

    /// 3mf.cpp:384-388 — `_3MF_Base::log_errors()` (public on the exporter).
    pub fn log_errors(&self) {
        self.base.log_errors();
    }

    /// 3mf.cpp:2320-2326
    /// C++: `bool save_model_to_file(const std::string& filename, Model& model, const DynamicPrintConfig* config, bool fullpath_sources, const ThumbnailData* thumbnail_data, bool zip64)`
    pub fn save_model_to_file(
        &mut self,
        filename: &str,
        model: &Model,
        config: Option<&DynamicPrintConfig>,
        fullpath_sources: bool,
        thumbnail_data: Option<&ThumbnailData>,
        zip64: bool,
    ) -> bool {
        // 3mf.cpp:2322-2325
        self.base.clear_errors();
        self.m_fullpath_sources = fullpath_sources;
        self.m_zip64 = zip64;
        self._save_model_to_file(filename, model, config, thumbnail_data)
    }

    /// 3mf.cpp:2328-2445
    /// C++: `bool _save_model_to_file(const std::string& filename, Model& model, const DynamicPrintConfig* config, const ThumbnailData* thumbnail_data)`
    fn _save_model_to_file(
        &mut self,
        filename: &str,
        model: &Model,
        config: Option<&DynamicPrintConfig>,
        thumbnail_data: Option<&ThumbnailData>,
    ) -> bool {
        // 3mf.cpp:2330-2336 — mz_zip_zero_struct / open_zip_writer
        let file = match std::fs::File::create(filename) {
            Ok(f) => f,
            Err(_) => {
                self.base.add_error("Unable to open the file");
                return false;
            }
        };
        let mut archive = zip::ZipWriter::new(file);

        // close_zip_writer(&archive); boost::filesystem::remove(filename);
        // (each failure branch below mirrors 3mf.cpp's close + remove.)
        macro_rules! fail_and_remove {
            ($archive:ident) => {{
                drop($archive);
                let _ = std::fs::remove_file(filename);
                return false;
            }};
        }

        // Adds content types file ("[Content_Types].xml";).
        // The content of this file is the same for each PrusaSlicer 3mf.
        // 3mf.cpp:2340-2344
        if !self._add_content_types_file_to_archive(&mut archive) {
            fail_and_remove!(archive);
        }

        // 3mf.cpp:2346-2353
        if let Some(thumbnail_data) = thumbnail_data {
            if thumbnail_data.is_valid() {
                // Adds the file Metadata/thumbnail.png.
                if !self._add_thumbnail_file_to_archive(&mut archive, thumbnail_data) {
                    fail_and_remove!(archive);
                }
            }
        }

        // Adds relationships file ("_rels/.rels").
        // The content of this file is the same for each PrusaSlicer 3mf.
        // The relationshis file contains a reference to the geometry file "3D/3dmodel.model", the name was chosen to be compatible with CURA.
        // 3mf.cpp:2358-2362
        if !self._add_relationships_file_to_archive(&mut archive) {
            fail_and_remove!(archive);
        }

        // Adds model file ("3D/3dmodel.model").
        // This is the one and only file that contains all the geometry (vertices and triangles) of all ModelVolumes.
        // 3mf.cpp:2366-2371
        let mut objects_data = IdToObjectDataMap::new();
        if !self._add_model_file_to_archive(filename, &mut archive, model, &mut objects_data) {
            fail_and_remove!(archive);
        }

        // Adds layer height profile file ("Metadata/Slic3r_PE_layer_heights_profile.txt").
        // All layer height profiles of all ModelObjects are stored here, indexed by 1 based index of the ModelObject in Model.
        // The index differes from the index of an object ID of an object instance of a 3MF file!
        // 3mf.cpp:2376-2380
        if !self._add_layer_height_profile_file_to_archive(&mut archive, model) {
            fail_and_remove!(archive);
        }

        // Adds layer config ranges file ("Metadata/Slic3r_PE_layer_config_ranges.txt").
        // All layer height profiles of all ModelObjects are stored here, indexed by 1 based index of the ModelObject in Model.
        // The index differes from the index of an object ID of an object instance of a 3MF file!
        // 3mf.cpp:2385-2389
        if !self._add_layer_config_ranges_file_to_archive(&mut archive, model) {
            fail_and_remove!(archive);
        }

        // Adds sla support points file ("Metadata/Slic3r_PE_sla_support_points.txt").
        // All  sla support points of all ModelObjects are stored here, indexed by 1 based index of the ModelObject in Model.
        // The index differes from the index of an object ID of an object instance of a 3MF file!
        // 3mf.cpp:2394-2398
        if !self._add_sla_support_points_file_to_archive(&mut archive, model) {
            fail_and_remove!(archive);
        }

        // 3mf.cpp:2400-2404
        if !self._add_sla_drain_holes_file_to_archive(&mut archive, model) {
            fail_and_remove!(archive);
        }

        // Adds custom gcode per height file ("Metadata/Prusa_Slicer_custom_gcode_per_print_z.xml").
        // All custom gcode per height of whole Model are stored here
        // 3mf.cpp:2409-2413
        if !self._add_custom_gcode_per_print_z_file_to_archive(&mut archive, model, config) {
            fail_and_remove!(archive);
        }

        // Adds slic3r print config file ("Metadata/Slic3r_PE.config").
        // This file contains the content of FullPrintConfing / SLAFullPrintConfig.
        // 3mf.cpp:2417-2423
        if let Some(config) = config {
            if !self._add_print_config_file_to_archive(&mut archive, config) {
                fail_and_remove!(archive);
            }
        }

        // Adds slic3r model config file ("Metadata/Slic3r_PE_model.config").
        // This file contains all the attributes of all ModelObjects and their ModelVolumes (names, parameter overrides).
        // As there is just a single Indexed Triangle Set data stored per ModelObject, offsets of volumes into their respective Indexed Triangle Set data
        // is stored here as well.
        // 3mf.cpp:2429-2433
        if !self._add_model_config_file_to_archive(&mut archive, model, &objects_data) {
            fail_and_remove!(archive);
        }

        // 3mf.cpp:2435-2440 — mz_zip_writer_finalize_archive
        if archive.finish().is_err() {
            let _ = std::fs::remove_file(filename);
            self.base.add_error("Unable to finalize the archive");
            return false;
        }

        // 3mf.cpp:2442 — close_zip_writer (RAII via finish above)

        // 3mf.cpp:2444
        true
    }

    /// 3mf.cpp:2447-2465
    fn _add_content_types_file_to_archive(
        &mut self,
        archive: &mut zip::ZipWriter<std::fs::File>,
    ) -> bool {
        // 3mf.cpp:2449-2455
        let mut stream = String::new();
        stream.push_str("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
        stream.push_str(
            "<Types xmlns=\"http://schemas.openxmlformats.org/package/2006/content-types\">\n",
        );
        stream.push_str(" <Default Extension=\"rels\" ContentType=\"application/vnd.openxmlformats-package.relationships+xml\"/>\n");
        stream.push_str(" <Default Extension=\"model\" ContentType=\"application/vnd.ms-package.3dmanufacturing-3dmodel+xml\"/>\n");
        stream.push_str(" <Default Extension=\"png\" ContentType=\"image/png\"/>\n");
        stream.push_str("</Types>");

        // 3mf.cpp:2457-2462
        let out = stream;
        if !mz_zip_writer_add_mem(archive, CONTENT_TYPES_FILE, out.as_bytes()) {
            self.base
                .add_error("Unable to add content types file to archive");
            return false;
        }

        // 3mf.cpp:2464
        true
    }

    /// 3mf.cpp:2467-2482
    fn _add_thumbnail_file_to_archive(
        &mut self,
        archive: &mut zip::ZipWriter<std::fs::File>,
        thumbnail_data: &ThumbnailData,
    ) -> bool {
        // 3mf.cpp:2469
        let mut res = false;

        // 3mf.cpp:2471-2472 — tdefl_write_image_to_png_file_in_memory_ex(
        //   pixels, width, height, 4 /*RGBA*/, &png_size, MZ_DEFAULT_LEVEL, 1 /*flip*/)
        let width = thumbnail_data.width as usize;
        let height = thumbnail_data.height as usize;
        let row = width * 4;
        let mut flipped: Vec<u8> = Vec::with_capacity(row * height);
        for y in (0..height).rev() {
            flipped.extend_from_slice(&thumbnail_data.pixels[y * row..(y + 1) * row]);
        }
        if let Some(png_data) = encode_png(width, height, PNG_COLOR_TYPE_RGB_ALPHA, &flipped) {
            // 3mf.cpp:2473-2475
            res = mz_zip_writer_add_mem(archive, THUMBNAIL_FILE, &png_data);
        }

        // 3mf.cpp:2478-2479
        if !res {
            self.base
                .add_error("Unable to add thumbnail file to archive");
        }

        // 3mf.cpp:2481
        res
    }

    /// 3mf.cpp:2484-2501
    fn _add_relationships_file_to_archive(
        &mut self,
        archive: &mut zip::ZipWriter<std::fs::File>,
    ) -> bool {
        // 3mf.cpp:2486-2491
        let mut stream = String::new();
        stream.push_str("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
        stream.push_str(
            "<Relationships xmlns=\"http://schemas.openxmlformats.org/package/2006/relationships\">\n",
        );
        stream.push_str(&format!(
            " <Relationship Target=\"/{}\" Id=\"rel-1\" Type=\"http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel\"/>\n",
            MODEL_FILE
        ));
        stream.push_str(&format!(
            " <Relationship Target=\"/{}\" Id=\"rel-2\" Type=\"http://schemas.openxmlformats.org/package/2006/relationships/metadata/thumbnail\"/>\n",
            THUMBNAIL_FILE
        ));
        stream.push_str("</Relationships>");

        // 3mf.cpp:2493-2498
        let out = stream;
        if !mz_zip_writer_add_mem(archive, RELATIONSHIPS_FILE, out.as_bytes()) {
            self.base
                .add_error("Unable to add relationships file to archive");
            return false;
        }

        // 3mf.cpp:2500
        true
    }

    /// 3mf.cpp:2514-2615
    /// C++: `bool _add_model_file_to_archive(const std::string& filename, mz_zip_archive& archive, const Model& model, IdToObjectDataMap& objects_data)`
    /// The C++ streams the entry with `mz_zip_writer_add_staged_*`; the Rust
    /// port accumulates the same bytes and writes the entry in one shot (the
    /// staged API is a miniz streaming detail).
    fn _add_model_file_to_archive(
        &mut self,
        filename: &str,
        archive: &mut zip::ZipWriter<std::fs::File>,
        model: &Model,
        objects_data: &mut IdToObjectDataMap,
    ) -> bool {
        // 3mf.cpp:2516-2528 — mz_zip_writer_add_staged_open with the zip64
        // switch: 16GiB max (zip64) vs 4GB-1 (workaround for the Windows 10 3D
        // model fixing API, GH issue #6193).
        let mut context: Vec<u8> = Vec::new();

        {
            // 3mf.cpp:2531-2533
            let mut stream = String::new();
            reset_stream(&mut stream);
            stream.push_str("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
            stream.push_str(&format!(
                "<{} unit=\"millimeter\" xml:lang=\"en-US\" xmlns=\"http://schemas.microsoft.com/3dmanufacturing/core/2015/02\" xmlns:slic3rpe=\"http://schemas.slic3r.org/3mf/2017/06\">\n",
                MODEL_TAG
            ));
            // 3mf.cpp:2535
            stream.push_str(&format!(
                " <{} name=\"{}\">{}</{}>\n",
                METADATA_TAG, SLIC3RPE_3MF_VERSION, VERSION_3MF, METADATA_TAG
            ));

            // 3mf.cpp:2537-2544 — model.is_fdm_support_painted() /
            // is_seam_painted() / is_mm_painted() metadata lines.
            // BLOCKED(model): the simplified Model carries no painting
            // annotations, so the painted-version metadata is never emitted
            // (matches an unpainted model in C++).

            // 3mf.cpp:2546-2547
            let name = xml_escape(
                &std::path::Path::new(filename)
                    .file_stem()
                    .map(|s| s.to_string_lossy().into_owned())
                    .unwrap_or_default(),
                false,
            );
            stream.push_str(&format!(
                " <{} name=\"Title\">{}</{}>\n",
                METADATA_TAG, name, METADATA_TAG
            ));
            // 3mf.cpp:2548
            stream.push_str(&format!(
                " <{} name=\"Designer\"></{}>\n",
                METADATA_TAG, METADATA_TAG
            ));
            // 3mf.cpp:2549
            stream.push_str(&format!(
                " <{} name=\"Description\">{}</{}>\n",
                METADATA_TAG, name, METADATA_TAG
            ));
            // 3mf.cpp:2550-2552
            stream.push_str(&format!(
                " <{} name=\"Copyright\"></{}>\n",
                METADATA_TAG, METADATA_TAG
            ));
            stream.push_str(&format!(
                " <{} name=\"LicenseTerms\"></{}>\n",
                METADATA_TAG, METADATA_TAG
            ));
            stream.push_str(&format!(
                " <{} name=\"Rating\"></{}>\n",
                METADATA_TAG, METADATA_TAG
            ));
            // 3mf.cpp:2553-2557 — keep only the date part of the string
            let date = utc_timestamp(get_current_time_utc());
            let date = date.chars().take(10).collect::<String>();
            stream.push_str(&format!(
                " <{} name=\"CreationDate\">{}</{}>\n",
                METADATA_TAG, date, METADATA_TAG
            ));
            stream.push_str(&format!(
                " <{} name=\"ModificationDate\">{}</{}>\n",
                METADATA_TAG, date, METADATA_TAG
            ));
            // 3mf.cpp:2558
            stream.push_str(&format!(
                " <{} name=\"Application\">{}-{}</{}>\n",
                METADATA_TAG, SLIC3R_APP_KEY, SLIC3R_VERSION, METADATA_TAG
            ));
            // 3mf.cpp:2559
            stream.push_str(&format!(" <{}>\n", RESOURCES_TAG));

            // 3mf.cpp:2560-2564
            let buf = stream;
            if !buf.is_empty() {
                context.extend_from_slice(buf.as_bytes());
            }
        }

        // Instance transformations, indexed by the 3MF object ID (which is a linear serialization of all instances of all ModelObjects).
        // 3mf.cpp:2568
        let mut build_items: BuildItemsList = Vec::new();

        // The object_id here is a one based identifier of the first instance of a ModelObject in the 3MF file, where
        // all the object instances of all ModelObjects are stored and indexed in a 1 based linear fashion.
        // Therefore the list of object_ids here may not be continuous.
        // 3mf.cpp:2573-2589
        let mut object_id: u32 = 1;
        for (obj_idx, obj) in model.objects.iter().enumerate() {
            // (obj == nullptr is unrepresentable.)

            // Index of an object in the 3MF file corresponding to the 1st instance of a ModelObject.
            let curr_id = object_id;
            objects_data.insert(curr_id as i32, ObjectData::new(obj_idx));
            let object_data = objects_data.get_mut(&(curr_id as i32)).unwrap();
            // Store geometry of all ModelVolumes contained in a single ModelObject into a single 3MF indexed triangle set object.
            // object_it->second.volumes_offsets will contain the offsets of the ModelVolumes in that single indexed triangle set.
            // object_id will be increased to point to the 1st instance of the next ModelObject.
            if !Self::_add_object_to_model_stream(
                &mut self.base,
                &mut context,
                &mut object_id,
                obj,
                &mut build_items,
                &mut object_data.volumes_offsets,
            ) {
                self.base.add_error("Unable to add object to archive");
                // mz_zip_writer_add_staged_finish(&context);
                return false;
            }
        }

        {
            // 3mf.cpp:2592-2594
            let mut stream = String::new();
            reset_stream(&mut stream);
            stream.push_str(&format!(" </{}>\n", RESOURCES_TAG));

            // Store the transformations of all the ModelInstances of all ModelObjects, indexed in a linear fashion.
            // 3mf.cpp:2597-2601
            if !Self::_add_build_to_model_stream(&mut self.base, &mut stream, &build_items) {
                self.base.add_error("Unable to add build to archive");
                return false;
            }

            // 3mf.cpp:2603
            stream.push_str(&format!("</{}>\n", MODEL_TAG));

            // 3mf.cpp:2605-2611
            let buf = stream;
            if !buf.is_empty() {
                context.extend_from_slice(buf.as_bytes());
            }
        }

        // mz_zip_writer_add_staged_open/finish — write the accumulated entry,
        // honoring the zip64 switch (3mf.cpp:2517-2525).
        let options = zip::write::FileOptions::default()
            .compression_method(zip::CompressionMethod::Deflated)
            .large_file(self.m_zip64);
        if archive.start_file(MODEL_FILE, options).is_err() || archive.write_all(&context).is_err()
        {
            self.base.add_error("Unable to add model file to archive");
            return false;
        }

        // 3mf.cpp:2614
        true
    }

    /// 3mf.cpp:2617-2658
    /// C++: `bool _add_object_to_model_stream(mz_zip_writer_staged_context &context, unsigned int& object_id, ModelObject& object, BuildItemsList& build_items, VolumeToOffsetsMap& volumes_offsets)`
    fn _add_object_to_model_stream(
        base: &mut _3MF_Base,
        context: &mut Vec<u8>,
        object_id: &mut u32,
        object: &ModelObject,
        build_items: &mut BuildItemsList,
        volumes_offsets: &mut VolumeToOffsetsMap,
    ) -> bool {
        // 3mf.cpp:2619-2621
        let mut stream = String::new();
        reset_stream(&mut stream);
        let mut id: u32 = 0;
        // 3mf.cpp:2622-2653
        for instance in &object.instances {
            // 3mf.cpp:2623-2625 — assert(instance != nullptr); unrepresentable.

            // 3mf.cpp:2627-2628
            let instance_id = *object_id + id;
            stream.push_str(&format!(
                "  <{} id=\"{}\" type=\"model\">\n",
                OBJECT_TAG, instance_id
            ));

            // 3mf.cpp:2630-2643
            if id == 0 {
                // 3mf.cpp:2631-2637
                let buf = std::mem::take(&mut stream);
                if !buf.is_empty() {
                    context.extend_from_slice(buf.as_bytes());
                }
                if !Self::_add_mesh_to_object_stream(base, context, object, volumes_offsets) {
                    base.add_error("Unable to add mesh to archive");
                    return false;
                }
            } else {
                // 3mf.cpp:2640-2642
                stream.push_str(&format!("   <{}>\n", COMPONENTS_TAG));
                stream.push_str(&format!(
                    "    <{} objectid=\"{}\"/>\n",
                    COMPONENT_TAG, *object_id
                ));
                stream.push_str(&format!("   </{}>\n", COMPONENTS_TAG));
            }

            // 3mf.cpp:2645-2648 — Transform3d t = instance->get_matrix();
            // (the simplified Instance transform: offset + Z rotation + scale.)
            let m = instance.transform();
            let t = Transform3d::from_column_slice(&m.matrix);
            // instance_id is just a 1 indexed index in build_items.
            debug_assert!(instance_id as usize == build_items.len() + 1);
            build_items.push(BuildItem::new(instance_id, t, instance.printable));

            // 3mf.cpp:2650
            stream.push_str(&format!("  </{}>\n", OBJECT_TAG));

            // 3mf.cpp:2652
            id += 1;
        }

        // 3mf.cpp:2655-2657
        *object_id += id;
        let buf = stream;
        if buf.is_empty() {
            true
        } else {
            context.extend_from_slice(buf.as_bytes());
            true
        }
    }

    // 3mf.cpp:2660-2678 — EXPORT_3MF_USE_SPIRIT_KARMA_FP coordinate policies:
    // compiled out (the macro is 0).

    /// 3mf.cpp:2680-2844
    /// C++: `bool _add_mesh_to_object_stream(mz_zip_writer_staged_context &context, ModelObject& object, VolumeToOffsetsMap& volumes_offsets)`
    fn _add_mesh_to_object_stream(
        base: &mut _3MF_Base,
        context: &mut Vec<u8>,
        object: &ModelObject,
        volumes_offsets: &mut VolumeToOffsetsMap,
    ) -> bool {
        // 3mf.cpp:2682-2687
        let mut output_buffer = String::new();
        output_buffer.push_str("   <");
        output_buffer.push_str(MESH_TAG);
        output_buffer.push_str(">\n    <");
        output_buffer.push_str(VERTICES_TAG);
        output_buffer.push_str(">\n");

        // 3mf.cpp:2689-2698 — flush lambda: stage out when forced or when the
        // buffer exceeds 65536 * 16 bytes.
        fn flush(output_buffer: &mut String, context: &mut Vec<u8>, force: bool) -> bool {
            if (force && !output_buffer.is_empty()) || output_buffer.len() >= 65536 * 16 {
                context.extend_from_slice(output_buffer.as_bytes());
                output_buffer.clear();
            }
            true
        }

        // 3mf.cpp:2700-2728 — format_coordinate: sprintf(buf, "%.9g", f)
        // (round-trippable float, shortest possible; the karma fast path is
        //  compiled out).
        fn format_coordinate(f: f32) -> String {
            debug_assert!(is_decimal_separator_point());
            general_format(f as f64, FLOAT_MAX_DIGITS10)
        }

        // 3mf.cpp:2730-2763
        let mut vertices_count: u32 = 0;
        // BLOCKED(model): `for (ModelVolume* volume : object.volumes)` — the
        // simplified ModelObject exposes one merged volume (its mesh), keyed 0.
        {
            let volume_key = 0usize;
            // 3mf.cpp:2736
            volumes_offsets.insert(volume_key, Offsets::new(vertices_count));

            // 3mf.cpp:2738-2742
            let its_vertices = object.mesh.vertices();
            if its_vertices.is_empty() {
                base.add_error("Found invalid mesh");
                return false;
            }

            // 3mf.cpp:2744
            vertices_count += its_vertices.len() as u32;

            // 3mf.cpp:2746 — const Transform3d& matrix = volume->get_matrix();
            // BLOCKED(model): no per-volume matrix; identity (vertices are
            // stored in object coordinates already).

            // 3mf.cpp:2748-2762
            for v in its_vertices {
                // Vec3f v = (matrix * its.vertices[i].cast<double>()).cast<float>();
                output_buffer.push_str("     <");
                output_buffer.push_str(VERTEX_TAG);
                output_buffer.push_str(" x=\"");
                output_buffer.push_str(&format_coordinate(v.x as f32));
                output_buffer.push_str("\" y=\"");
                output_buffer.push_str(&format_coordinate(v.y as f32));
                output_buffer.push_str("\" z=\"");
                output_buffer.push_str(&format_coordinate(v.z as f32));
                output_buffer.push_str("\"/>\n");
                if !flush(&mut output_buffer, context, false) {
                    return false;
                }
            }
        }

        // 3mf.cpp:2765-2769
        output_buffer.push_str("    </");
        output_buffer.push_str(VERTICES_TAG);
        output_buffer.push_str(">\n    <");
        output_buffer.push_str(TRIANGLES_TAG);
        output_buffer.push_str(">\n");

        // 3mf.cpp:2771-2834
        let mut triangles_count: u32 = 0;
        {
            let volume_key = 0usize;
            // 3mf.cpp:2776 — bool is_left_handed = volume->is_left_handed();
            // BLOCKED(model): no per-volume mirrored transform on the
            // simplified model; always right-handed.
            let is_left_handed = false;
            // 3mf.cpp:2777-2778
            let volume_it = volumes_offsets.get_mut(&volume_key).unwrap();

            let its_indices = object.mesh.indices();

            // updates triangle offsets
            // 3mf.cpp:2782-2785
            volume_it.first_triangle_id = triangles_count;
            triangles_count += its_indices.len() as u32;
            volume_it.last_triangle_id = triangles_count.wrapping_sub(1);

            // 3mf.cpp:2787-2833
            for idx in its_indices {
                // 3mf.cpp:2788-2800
                output_buffer.push_str("     <");
                output_buffer.push_str(TRIANGLE_TAG);
                output_buffer.push_str(" v1=\"");
                output_buffer.push_str(
                    &(idx.indices[if is_left_handed { 2 } else { 0 }] + volume_it.first_vertex_id)
                        .to_string(),
                );
                output_buffer.push_str("\" v2=\"");
                output_buffer.push_str(&(idx.indices[1] + volume_it.first_vertex_id).to_string());
                output_buffer.push_str("\" v3=\"");
                output_buffer.push_str(
                    &(idx.indices[if is_left_handed { 0 } else { 2 }] + volume_it.first_vertex_id)
                        .to_string(),
                );
                output_buffer.push_str("\"");

                // 3mf.cpp:2802-2827 — volume->supported_facets / seam_facets /
                // mmu_segmentation_facets .get_triangle_as_string(i).
                // BLOCKED(model): no FacetsAnnotation on the simplified model;
                // the strings are always empty, so no attribute is emitted
                // (matches an unpainted model in C++).

                // 3mf.cpp:2829
                output_buffer.push_str("/>\n");

                // 3mf.cpp:2831-2832
                if !flush(&mut output_buffer, context, false) {
                    return false;
                }
            }
        }

        // 3mf.cpp:2836-2840
        output_buffer.push_str("    </");
        output_buffer.push_str(TRIANGLES_TAG);
        output_buffer.push_str(">\n   </");
        output_buffer.push_str(MESH_TAG);
        output_buffer.push_str(">\n");

        // Force flush.
        // 3mf.cpp:2843
        flush(&mut output_buffer, context, true)
    }

    /// 3mf.cpp:2846-2871
    /// C++: `bool _add_build_to_model_stream(std::stringstream& stream, const BuildItemsList& build_items)`
    fn _add_build_to_model_stream(
        base: &mut _3MF_Base,
        stream: &mut String,
        build_items: &BuildItemsList,
    ) -> bool {
        // This happens for empty projects
        // 3mf.cpp:2849-2852
        if build_items.is_empty() {
            base.add_error("No build item found");
            return true;
        }

        // 3mf.cpp:2854
        stream.push_str(&format!(" <{}>\n", BUILD_TAG));

        // 3mf.cpp:2856-2866 — the stream carries setprecision(9) from
        // reset_stream (float max_digits10).
        for item in build_items {
            stream.push_str(&format!(
                "  <{} {}=\"{}\" {}=\"",
                ITEM_TAG, OBJECTID_ATTR, item.id, TRANSFORM_ATTR
            ));
            for c in 0..4 {
                for r in 0..3 {
                    stream.push_str(&general_format(item.transform[(r, c)], FLOAT_MAX_DIGITS10));
                    if r != 2 || c != 3 {
                        stream.push(' ');
                    }
                }
            }
            // 3mf.cpp:2865 — `stream << item.printable` (bool -> 1/0)
            stream.push_str(&format!(
                "\" {}=\"{}\"/>\n",
                PRINTABLE_ATTR,
                if item.printable { 1 } else { 0 }
            ));
        }

        // 3mf.cpp:2868
        stream.push_str(&format!(" </{}>\n", BUILD_TAG));

        // 3mf.cpp:2870
        true
    }

    /// 3mf.cpp:2873-2905
    /// C++: `bool _add_layer_height_profile_file_to_archive(mz_zip_archive& archive, Model& model)`
    fn _add_layer_height_profile_file_to_archive(
        &mut self,
        archive: &mut zip::ZipWriter<std::fs::File>,
        model: &Model,
    ) -> bool {
        // 3mf.cpp:2875
        debug_assert!(is_decimal_separator_point());
        // 3mf.cpp:2876-2877
        let out = String::new();

        // 3mf.cpp:2879-2895
        let mut count: u32 = 0;
        for _object in &model.objects {
            count += 1;
            // 3mf.cpp:2882-2894 —
            //   const std::vector<double>& layer_height_profile = object->layer_height_profile.get();
            //   if (size >= 4 && size % 2 == 0) { out += "object_id=%d|" + ";%f"-joined profile + "\n"; }
            // BLOCKED(model): the simplified ModelObject has no
            // layer_height_profile; the profile is always absent, so nothing is
            // appended (matches a model without profiles in C++).
        }
        let _ = count;

        // 3mf.cpp:2897-2902
        if !out.is_empty() {
            if !mz_zip_writer_add_mem(archive, LAYER_HEIGHTS_PROFILE_FILE, out.as_bytes()) {
                self.base
                    .add_error("Unable to add layer heights profile file to archive");
                return false;
            }
        }

        // 3mf.cpp:2904
        true
    }

    /// 3mf.cpp:2907-2963
    /// C++: `bool _add_layer_config_ranges_file_to_archive(mz_zip_archive& archive, Model& model)`
    fn _add_layer_config_ranges_file_to_archive(
        &mut self,
        archive: &mut zip::ZipWriter<std::fs::File>,
        model: &Model,
    ) -> bool {
        // 3mf.cpp:2909-2910
        let out = String::new();

        // 3mf.cpp:2912-2938
        let mut object_cnt: u32 = 0;
        for _object in &model.objects {
            object_cnt += 1;
            // 3mf.cpp:2915-2937 —
            //   const t_layer_config_ranges& ranges = object->layer_config_ranges;
            //   builds a ptree: objects.object(<xmlattr>.id) / range(min_z,max_z) /
            //   option(<xmlattr>.opt_key, config.opt_serialize(opt_key))
            // BLOCKED(model)+BLOCKED(config): the simplified ModelObject has no
            // layer_config_ranges and ModelConfig::opt_serialize is unported;
            // ranges are always absent, so the tree stays empty.
        }
        let _ = object_cnt;

        // 3mf.cpp:2940-2953 — pt::write_xml + "beautification" replace_all
        // passes; the tree is always empty here, so `out` stays empty.

        // 3mf.cpp:2955-2960
        if !out.is_empty() {
            if !mz_zip_writer_add_mem(archive, LAYER_CONFIG_RANGES_FILE, out.as_bytes()) {
                self.base
                    .add_error("Unable to add layer heights profile file to archive");
                return false;
            }
        }

        // 3mf.cpp:2962
        true
    }

    /// 3mf.cpp:2965-2998
    /// C++: `bool _add_sla_support_points_file_to_archive(mz_zip_archive& archive, Model& model)`
    fn _add_sla_support_points_file_to_archive(
        &mut self,
        archive: &mut zip::ZipWriter<std::fs::File>,
        model: &Model,
    ) -> bool {
        // 3mf.cpp:2967
        debug_assert!(is_decimal_separator_point());
        // 3mf.cpp:2968-2969
        let mut out = String::new();

        // 3mf.cpp:2971-2986
        let mut count: u32 = 0;
        for _object in &model.objects {
            count += 1;
            // 3mf.cpp:2974-2985 —
            //   const std::vector<sla::SupportPoint>& sla_support_points = object->sla_support_points;
            //   out += "object_id=%d|" + "%f %f %f %f %f"-formatted points + "\n";
            // BLOCKED(model): the simplified ModelObject has no
            // sla_support_points; always empty.
        }
        let _ = count;

        // 3mf.cpp:2988-2996
        if !out.is_empty() {
            // Adds version header at the beginning:
            out = format!(
                "support_points_format_version={}\n{}",
                SUPPORT_POINTS_FORMAT_VERSION, out
            );

            if !mz_zip_writer_add_mem(archive, SLA_SUPPORT_POINTS_FILE, out.as_bytes()) {
                self.base
                    .add_error("Unable to add sla support points file to archive");
                return false;
            }
        }
        // 3mf.cpp:2997
        true
    }

    /// 3mf.cpp:3000-3049
    /// C++: `bool _add_sla_drain_holes_file_to_archive(mz_zip_archive& archive, Model& model)`
    fn _add_sla_drain_holes_file_to_archive(
        &mut self,
        archive: &mut zip::ZipWriter<std::fs::File>,
        model: &Model,
    ) -> bool {
        // 3mf.cpp:3002
        debug_assert!(is_decimal_separator_point());
        // 3mf.cpp:3003-3004 — const char *const fmt = "object_id=%d|";
        let mut out = String::new();

        // 3mf.cpp:3006-3037
        let mut count: u32 = 0;
        for _object in &model.objects {
            count += 1;
            // 3mf.cpp:3009-3036 —
            //   sla::DrainHoles drain_holes = object->sla_drain_holes;
            //   (holes re-elevated 1mm above the mesh for compatibility:
            //    hole.pos -= normal.normalized(); hole.height += 1.f;)
            //   out += "object_id=%d|" + "%f %f %f %f %f %f %f %f"-formatted holes + "\n";
            // BLOCKED(model): the simplified ModelObject has no
            // sla_drain_holes; always empty.
        }
        let _ = count;

        // 3mf.cpp:3039-3047
        if !out.is_empty() {
            // Adds version header at the beginning:
            out = format!(
                "drain_holes_format_version={}\n{}",
                DRAIN_HOLES_FORMAT_VERSION, out
            );

            if !mz_zip_writer_add_mem(archive, SLA_DRAIN_HOLES_FILE, out.as_bytes()) {
                self.base
                    .add_error("Unable to add sla support points file to archive");
                return false;
            }
        }
        // 3mf.cpp:3048
        true
    }

    /// 3mf.cpp:3051-3070
    /// C++: `bool _add_print_config_file_to_archive(mz_zip_archive& archive, const DynamicPrintConfig &config)`
    fn _add_print_config_file_to_archive(
        &mut self,
        archive: &mut zip::ZipWriter<std::fs::File>,
        _config: &DynamicPrintConfig,
    ) -> bool {
        // 3mf.cpp:3053
        debug_assert!(is_decimal_separator_point());
        // 3mf.cpp:3054-3056
        let out = format!("; {}\n\n", header_slic3r_generated());

        // 3mf.cpp:3058-3060 —
        //   for (const std::string &key : config.keys())
        //       if (key != "compatible_printers")
        //           out += "; " + key + " = " + config.opt_serialize(key) + "\n";
        // BLOCKED(config): the placeholder DynamicPrintConfig exposes no
        // keys()/opt_serialize; only the generator header is written.

        // 3mf.cpp:3062-3067
        if !out.is_empty() {
            if !mz_zip_writer_add_mem(archive, PRINT_CONFIG_FILE, out.as_bytes()) {
                self.base
                    .add_error("Unable to add print config file to archive");
                return false;
            }
        }

        // 3mf.cpp:3069
        true
    }

    /// 3mf.cpp:3072-3181
    /// C++: `bool _add_model_config_file_to_archive(mz_zip_archive& archive, const Model& model, const IdToObjectDataMap &objects_data)`
    fn _add_model_config_file_to_archive(
        &mut self,
        archive: &mut zip::ZipWriter<std::fs::File>,
        model: &Model,
        objects_data: &IdToObjectDataMap,
    ) -> bool {
        // Store mesh transformation in full precision, as the volumes are stored transformed and they need to be transformed back
        // when loaded as accurately as possible.
        // 3mf.cpp:3074-3079 — setprecision(double max_digits10) == 17.
        let mut stream = String::new();
        stream.push_str("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
        stream.push_str(&format!("<{}>\n", CONFIG_TAG));

        // 3mf.cpp:3081-3169
        for (obj_id, obj_metadata) in objects_data {
            let obj = &model.objects[obj_metadata.object];
            // (obj == nullptr is unrepresentable.)

            // Output of instances count added because of github #3435, currently not used by PrusaSlicer
            // 3mf.cpp:3085
            stream.push_str(&format!(
                " <{} {}=\"{}\" {}=\"{}\">\n",
                OBJECT_TAG,
                ID_ATTR,
                obj_id,
                INSTANCESCOUNT_ATTR,
                obj.instances.len()
            ));

            // stores object's name
            // 3mf.cpp:3088-3089
            if !obj.name.is_empty() {
                stream.push_str(&format!(
                    "  <{} {}=\"{}\" {}=\"name\" {}=\"{}\"/>\n",
                    METADATA_TAG,
                    TYPE_ATTR,
                    OBJECT_TYPE,
                    KEY_ATTR,
                    VALUE_ATTR,
                    xml_escape(&obj.name, false)
                ));
            }

            // stores object's config data
            // 3mf.cpp:3092-3094 —
            //   for (const std::string& key : obj->config.keys()) { ... opt_serialize ... }
            // BLOCKED(config): ModelConfig::keys()/opt_serialize unported; the
            // simplified ObjectConfig carries no serializable overrides.

            // 3mf.cpp:3096-3165 — for (const ModelVolume* volume : volumes)
            // BLOCKED(model): single merged volume per object (key 0).
            for (_volume_key, offsets) in &obj_metadata.volumes_offsets {
                // stores volume's offsets
                // 3mf.cpp:3102-3104
                stream.push_str(&format!("  <{} ", VOLUME_TAG));
                stream.push_str(&format!(
                    "{}=\"{}\" ",
                    FIRST_TRIANGLE_ID_ATTR, offsets.first_triangle_id
                ));
                stream.push_str(&format!(
                    "{}=\"{}\">\n",
                    LAST_TRIANGLE_ID_ATTR, offsets.last_triangle_id
                ));

                // stores volume's name
                // 3mf.cpp:3107-3108 — if (!volume->name.empty()) ...
                // BLOCKED(model): no per-volume name; omitted (matches an
                // unnamed volume in C++).

                // stores volume's modifier field (legacy, to support old slicers)
                // 3mf.cpp:3111-3112 — if (volume->is_modifier()) ...
                // BLOCKED(model): no per-volume type; never a modifier.
                // stores volume's type (overrides the modifier field above)
                // 3mf.cpp:3114-3115
                // BLOCKED(model): no per-volume type; always MODEL_PART.
                stream.push_str(&format!(
                    "   <{} {}=\"{}\" {}=\"{}\" {}=\"{}\"/>\n",
                    METADATA_TAG,
                    TYPE_ATTR,
                    VOLUME_TYPE,
                    KEY_ATTR,
                    VOLUME_TYPE_KEY,
                    VALUE_ATTR,
                    ModelVolumeType::type_to_string(ModelVolumeType::ModelPart)
                ));

                // stores volume's local matrix
                // 3mf.cpp:3118-3127 — matrix = volume->get_matrix() * volume->source.transform.get_matrix()
                // BLOCKED(model): both are identity on the simplified model.
                let matrix = Transform3d::identity();
                stream.push_str(&format!(
                    "   <{} {}=\"{}\" {}=\"{}\" {}=\"",
                    METADATA_TAG, TYPE_ATTR, VOLUME_TYPE, KEY_ATTR, MATRIX_KEY, VALUE_ATTR
                ));
                for r in 0..4 {
                    for c in 0..4 {
                        stream.push_str(&general_format(matrix[(r, c)], DOUBLE_MAX_DIGITS10));
                        if r != 3 || c != 3 {
                            stream.push(' ');
                        }
                    }
                }
                stream.push_str("\"/>\n");

                // stores volume's source data
                // 3mf.cpp:3129-3146 — source_file / source_object_id /
                // source_volume_id / source_offset_{x,y,z} / source_in_inches /
                // source_in_meters metadata.
                // BLOCKED(model): no ModelVolume::source; `input_file` is empty
                // and the conversion flags are false, so nothing is emitted
                // (matches a volume without source info in C++).

                // stores volume's config data
                // 3mf.cpp:3148-3151 — volume->config.keys()/opt_serialize.
                // BLOCKED(config): unported.

                // stores mesh's statistics
                // 3mf.cpp:3153-3160 — const RepairedMeshErrors& stats =
                //   volume->mesh().stats().repaired_errors;
                // BLOCKED(model): the simplified TriangleMesh carries no
                // repaired statistics; zeros match an unrepaired mesh.
                let stats = RepairedMeshErrors::default();
                stream.push_str(&format!("   <{} ", MESH_TAG));
                stream.push_str(&format!(
                    "{}=\"{}\" ",
                    MESH_STAT_EDGES_FIXED, stats.edges_fixed
                ));
                stream.push_str(&format!(
                    "{}=\"{}\" ",
                    MESH_STAT_DEGENERATED_FACETS, stats.degenerate_facets
                ));
                stream.push_str(&format!(
                    "{}=\"{}\" ",
                    MESH_STAT_FACETS_REMOVED, stats.facets_removed
                ));
                stream.push_str(&format!(
                    "{}=\"{}\" ",
                    MESH_STAT_FACETS_RESERVED, stats.facets_reversed
                ));
                stream.push_str(&format!(
                    "{}=\"{}\"/>\n",
                    MESH_STAT_BACKWARDS_EDGES, stats.backwards_edges
                ));

                // 3mf.cpp:3162
                stream.push_str(&format!("  </{}>\n", VOLUME_TAG));
            }

            // 3mf.cpp:3167
            stream.push_str(&format!(" </{}>\n", OBJECT_TAG));
        }

        // 3mf.cpp:3171
        stream.push_str(&format!("</{}>\n", CONFIG_TAG));

        // 3mf.cpp:3173-3178
        let out = stream;
        if !mz_zip_writer_add_mem(archive, MODEL_CONFIG_FILE, out.as_bytes()) {
            self.base
                .add_error("Unable to add model config file to archive");
            return false;
        }

        // 3mf.cpp:3180
        true
    }

    /// 3mf.cpp:3183-3234
    /// C++: `bool _add_custom_gcode_per_print_z_file_to_archive(mz_zip_archive& archive, Model& model, const DynamicPrintConfig* config)`
    /// The entire body after `return true;` is commented out in BambuStudio
    /// (3mf.cpp:3186-3233): the custom-gcode ptree export is disabled.
    fn _add_custom_gcode_per_print_z_file_to_archive(
        &mut self,
        _archive: &mut zip::ZipWriter<std::fs::File>,
        _model: &Model,
        _config: Option<&DynamicPrintConfig>,
    ) -> bool {
        // 3mf.cpp:3185
        true
    }
}

impl Default for _3MF_Exporter {
    fn default() -> Self {
        Self::new()
    }
}

/* The format for saving the SLA points was changing in the past. This enum holds the latest version that is being currently used.
 * Examples of the Slic3r_PE_sla_support_points.txt for historically used versions:

 *  version 0 : object_id=1|-12.055421 -2.658771 10.000000
                object_id=2|-14.051745 -3.570338 5.000000
    // no header and x,y,z positions of the points)

 * version 1 :  ThreeMF_support_points_version=1
                object_id=1|-12.055421 -2.658771 10.000000 0.4 0.0
                object_id=2|-14.051745 -3.570338 5.000000 0.6 1.0
    // introduced header with version number; x,y,z,head_size,is_new_island)
*/
// 3mf.hpp:42-44
pub const SUPPORT_POINTS_FORMAT_VERSION: u32 = 1;
// 3mf.hpp:46-48
pub const DRAIN_HOLES_FORMAT_VERSION: u32 = 1;

// Perform conversions based on the config values available.
//FIXME provide a version of PrusaSlicer that stored the project file (3MF).
/// 3mf.cpp:3236-3247
/// C++: `static void handle_legacy_project_loaded(unsigned int version_project_file, DynamicPrintConfig& config)`
fn handle_legacy_project_loaded(_version_project_file: u32, _config: &mut DynamicPrintConfig) {
    // 3mf.cpp:3240-3246 —
    //   if (! config.has("brim_object_gap"))
    //       if (auto *opt_elephant_foot = config.option<ConfigOptionFloat>("elefant_foot_compensation", false); opt_elephant_foot) {
    //           // Conversion from older PrusaSlicer which applied brim separation equal to elephant foot compensation.
    //           auto *opt_brim_separation = config.option<ConfigOptionFloat>("brim_object_gap", true);
    //           opt_brim_separation->value = opt_elephant_foot->value;
    //       }
    // BLOCKED(config): `ConfigBase::has` / typed `option<ConfigOptionFloat>`
    // require the reflective DynamicPrintConfig layer, which is unported (the
    // placeholder carries no options, so there is nothing to convert).
}

/// 3mf.cpp:3249-3261
/// C++: `bool load_3mf(const char* path, DynamicPrintConfig& config, ConfigSubstitutionContext& config_substitutions, Model* model, bool check_version)`
/// (`Err(...)` carries the `Slic3r::FileIOError` the C++ function lets
///  propagate for incompatible-version 3MFs.)
pub fn load_3mf(
    path: &str,
    config: &mut DynamicPrintConfig,
    config_substitutions: &mut ConfigSubstitutionContext,
    model: &mut Model,
    check_version: bool,
) -> Result<bool> {
    // 3mf.cpp:3251-3252 — (path == nullptr || model == nullptr): encoded in
    // the signature (references cannot be null).

    // All import should use "C" locales for number formatting.
    // 3mf.cpp:3255
    let _locales_setter = CNumericLocalesSetter::new();
    // 3mf.cpp:3256-3257
    let mut importer = _3MF_Importer::new(model);
    let res = importer.load_model_from_file(path, config, config_substitutions, check_version)?;
    // 3mf.cpp:3258
    importer.log_errors();
    // 3mf.cpp:3259
    handle_legacy_project_loaded(importer.version(), config);
    // 3mf.cpp:3260
    Ok(res)
}

/// 3mf.cpp:3263-3277
/// C++: `bool store_3mf(const char* path, Model* model, const DynamicPrintConfig* config, bool fullpath_sources, const ThumbnailData* thumbnail_data = nullptr, bool zip64 = true)`
pub fn store_3mf(
    path: &str,
    model: &Model,
    config: Option<&DynamicPrintConfig>,
    fullpath_sources: bool,
    thumbnail_data: Option<&ThumbnailData>,
    zip64: bool,
) -> bool {
    // All export should use "C" locales for number formatting.
    // 3mf.cpp:3266
    let _locales_setter = CNumericLocalesSetter::new();

    // 3mf.cpp:3268-3269 — (path == nullptr || model == nullptr): encoded in
    // the signature.

    // 3mf.cpp:3271-3274
    let mut exporter = _3MF_Exporter::new();
    let res =
        exporter.save_model_to_file(path, model, config, fullpath_sources, thumbnail_data, zip64);
    if !res {
        exporter.log_errors();
    }

    // 3mf.cpp:3276
    res
}
