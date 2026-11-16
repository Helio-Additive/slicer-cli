//! Port of `Format/AssemblyStepsJson.cpp` and `Format/AssemblyStepsJson.hpp` of Bambu Studio's
//! libslic3r at the pin: the assembly steps document (`assembly_steps.json`), the keyframes and
//! notes it carries, the tree the assembly view keeps of it, and the independent assembly model
//! document.
//!
//! The functions stand in the order of the .cpp; the types and the inline functions of the .hpp
//! come first, in the order of the .hpp.
//!
//! JSON. The C++ uses `nlohmann::json` (not `ordered_json`): an object is a `std::map`, its keys in
//! byte order. Here a document is a `serde_json::Value`, read by
//! [`crate::desensitize::nlohmann_parse`] and printed by [`crate::desensitize::nlohmann_dump`],
//! the port of nlohmann's serializer, which writes the keys in byte order whatever order the map
//! keeps them in. The small functions under "nlohmann::json members" below are the members of
//! `basic_json` this unit calls, with the meaning they have there.
//!
//! FIDELITY-NOTE: a parse error. `nlohmann::json::parse` throws `parse_error` with a text that
//! names the position and the token; `nlohmann_parse` only says that the text did not parse (and
//! carries the differences listed at its definition). Where the C++ hands `e.what()` to the caller
//! (`assembly_model_from_json_string`), the text here is [`PARSE_ERROR_WHAT`]. The `type_error`
//! texts are nlohmann's.
//!
//! FIDELITY-NOTE: `Model`, `ModelObject`, `ModelVolume` and `ModelInstance`. `crate::model` has no
//! object ids, volumes, part GUIDs or assemble transformations, so this file defines stand-ins
//! with exactly the members this unit reads and writes (section "Model stand-ins"), each method
//! citing the Model.hpp / Model.cpp lines it follows. What is missing from them: the mesh and
//! everything else `ModelVolume(ModelObject*, const ModelVolume&)` copies, the backup calls
//! (`save_object_mesh`, `delete_object_mesh`), the bounding box invalidation, and the config,
//! of which only the `extruder` option is kept (`Option<i32>`: `None` is "no such option").

use std::cell::RefCell;
use std::collections::{BTreeMap, BTreeSet, HashMap};
use std::path::PathBuf;
use std::sync::Arc;

use serde_json::{Map, Value};

use crate::boost_log_trivial;
use crate::desensitize::{nlohmann_dump, nlohmann_parse};
use crate::geometry::geometry::{Transform3d, Transformation, Vec2d, Vec3d};
use crate::object_id::{ObjectBase, ObjectID};

// ---------------------------------------------------------------------------------------------
// nlohmann::json members
// ---------------------------------------------------------------------------------------------

/// `std::exception::what()` of an exception `nlohmann::json` threw.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct JsonException(pub String);

impl JsonException {
    /// `e.what()`
    pub fn what(&self) -> &str {
        &self.0
    }
}

/// What `e.what()` is here for a `nlohmann::json::parse_error` (see the FIDELITY-NOTE at the top).
pub const PARSE_ERROR_WHAT: &str = "[json.exception.parse_error.101] parse error";

/// `nlohmann::json::parse(text)`
pub(crate) fn parse(text: &str) -> Result<Value, JsonException> {
    nlohmann_parse(text).ok_or_else(|| JsonException(PARSE_ERROR_WHAT.to_string()))
}

/// `stream >> json`: `parser(...).parse(false, j)`, not strict, so the text may go on after the
/// first value.
fn parse_stream(bytes: &[u8]) -> Result<Value, JsonException> {
    use serde::Deserialize;
    // nlohmann skips a byte order mark, and reads the token `-0` as the integer 0.
    let bytes = bytes.strip_prefix(&[0xEF, 0xBB, 0xBF]).unwrap_or(bytes);
    let bytes = crate::support_recommended_params::json_minus_zero_as_integer(bytes);
    let mut de = serde_json::Deserializer::from_slice(&bytes);
    Value::deserialize(&mut de).map_err(|_| JsonException(PARSE_ERROR_WHAT.to_string()))
}

/// `json.dump(indent)`: pretty printed for `indent >= 0`.
pub fn dump(json: &Value, indent: i32) -> String {
    let mut o = String::new();
    if indent >= 0 {
        nlohmann_dump(json, true, indent as usize, 0, &mut o);
    } else {
        nlohmann_dump(json, false, 0, 0, &mut o);
    }
    o
}

/// `basic_json::type_name()`
pub(crate) fn type_name(json: &Value) -> &'static str {
    match json {
        Value::Null => "null",
        Value::Object(_) => "object",
        Value::Array(_) => "array",
        Value::String(_) => "string",
        Value::Bool(_) => "boolean",
        Value::Number(_) => "number",
    }
}

/// `json.contains(key)`: false for anything that is not an object.
fn contains(json: &Value, key: &str) -> bool {
    json.as_object().map_or(false, |object| object.contains_key(key))
}

/// `json.is_number_integer()`: `number_integer` or `number_unsigned`.
fn is_number_integer(json: &Value) -> bool {
    json.is_i64() || json.is_u64()
}

/// `json.is_number_unsigned()`
fn is_number_unsigned(json: &Value) -> bool {
    json.is_u64()
}

/// `json.size()` of an array.
fn size(json: &Value) -> usize {
    json.as_array().map_or(0, |array| array.len())
}

/// The elements a range-for over an array visits.
fn elements(json: &Value) -> &[Value] {
    json.as_array().map_or(&[], |array| array.as_slice())
}

/// `json.get<int>()` of a value that `is_number_integer()`: a `static_cast<int>` of the stored
/// 64 bit integer.
fn get_int(json: &Value) -> i32 {
    match json.as_u64() {
        Some(v) => v as i32,
        None => json.as_i64().unwrap_or(0) as i32,
    }
}

/// `json.get<long long>()` of a value that `is_number_integer()`.
fn get_long_long(json: &Value) -> i64 {
    match json.as_u64() {
        Some(v) => v as i64,
        None => json.as_i64().unwrap_or(0),
    }
}

/// `json.get<double>()` of a value that `is_number()`.
fn get_double(json: &Value) -> f64 {
    if let Some(v) = json.as_u64() {
        v as f64
    } else if let Some(v) = json.as_i64() {
        v as f64
    } else {
        json.as_f64().unwrap_or(0.0)
    }
}

/// `json.get<double>()` of any value: `get_arithmetic_value` throws `type_error.302` for what is
/// not a number (a boolean included, `double` being the `number_float_t` of `basic_json`).
fn try_get_double(json: &Value) -> Result<f64, JsonException> {
    if json.is_number() {
        Ok(get_double(json))
    } else {
        Err(JsonException(format!("[json.exception.type_error.302] type must be number, but is {}", type_name(json))))
    }
}

/// A `double` stored in a json. nlohmann keeps a NaN or an infinity as a `number_float` and
/// prints it `null`; `serde_json` has no such number, so it is kept as the `null` it prints as.
fn json_float(value: f64) -> Value {
    serde_json::Number::from_f64(value).map(Value::Number).unwrap_or(Value::Null)
}

/// `{a, b, ...}` of doubles: an array.
fn json_floats(values: &[f64]) -> Value {
    Value::Array(values.iter().map(|v| json_float(*v)).collect())
}

/// `json[key] = value` where `json` is null or an object: `operator[]` turns a null into an
/// object. For another type it throws `type_error.305`.
fn try_set(json: &mut Value, key: &str, value: Value) -> Result<(), JsonException> {
    if json.is_null() {
        *json = Value::Object(Map::new());
    }
    match json {
        Value::Object(object) => {
            object.insert(key.to_string(), value);
            Ok(())
        }
        other => Err(JsonException(format!(
            "[json.exception.type_error.305] cannot use operator[] with a string argument with {}",
            type_name(other)
        ))),
    }
}

/// `json[key] = value` on a json this unit made itself: it is null or an object.
fn set(json: &mut Value, key: &str, value: Value) {
    try_set(json, key, value).expect("a json made by to_json is null or an object");
}

// ---------------------------------------------------------------------------------------------
// Model stand-ins (see the FIDELITY-NOTE at the top)
// ---------------------------------------------------------------------------------------------

/// Model.hpp:1070 `class ModelVolume`, the members this unit uses.
#[derive(Debug, Clone)]
pub struct ModelVolume {
    /// ObjectID.hpp:84 `ObjectBase::m_id`
    pub m_id: ObjectID,
    /// Model.hpp `std::string name`
    pub name: String,
    /// `m_type == ModelVolumeType::MODEL_PART`
    pub m_type: crate::model::ModelVolumeType,
    /// `config.option("extruder")`: `None` when the config has no such option.
    pub config_extruder: Option<i32>,
    /// Model.hpp:1235 `mutable std::string m_part_guid`
    pub m_part_guid: RefCell<String>,
    /// Model.hpp:1237
    pub m_assembly_src_guid: String,
    /// Model.hpp `Geometry::Transformation m_transformation`
    pub m_transformation: Transformation,
    /// Model.hpp `Geometry::Transformation m_assemble_transformation`
    pub m_assemble_transformation: Transformation,
    /// Model.hpp:1231
    pub m_assemble_initialized: bool,
}

impl Default for ModelVolume {
    fn default() -> Self {
        ModelVolume {
            m_id: ObjectID::default(),
            name: String::new(),
            m_type: crate::model::ModelVolumeType::ModelPart,
            config_extruder: None,
            m_part_guid: RefCell::new(String::new()),
            m_assembly_src_guid: String::new(),
            m_transformation: Transformation::default(),
            m_assemble_transformation: Transformation::default(),
            m_assemble_initialized: false,
        }
    }
}

impl ModelVolume {
    /// ObjectID.hpp:55 `ObjectID id() const`
    pub fn id(&self) -> ObjectID {
        self.m_id
    }

    /// Model.hpp `bool is_model_part() const { return m_type == ModelVolumeType::MODEL_PART; }`
    pub fn is_model_part(&self) -> bool {
        self.m_type == crate::model::ModelVolumeType::ModelPart
    }

    /// Model.hpp:1187
    pub fn part_guid(&self) -> String {
        self.m_part_guid.borrow().clone()
    }

    /// Model.hpp:1188
    pub fn set_part_guid(&mut self, guid: &str) {
        *self.m_part_guid.borrow_mut() = guid.to_string();
    }

    /// Model.cpp:2215-2221 `const std::string &ModelVolume::ensure_part_guid(bool force) const`
    pub fn ensure_part_guid(&self, force: bool) -> String {
        // Lazily assign a stable identity used for cross-model (prepare <-> assembly) part mapping.
        if self.m_part_guid.borrow().is_empty() || force {
            *self.m_part_guid.borrow_mut() = random_uuid_string();
        }
        self.m_part_guid.borrow().clone()
    }

    /// Model.hpp:1192
    pub fn assembly_src_guid(&self) -> &str {
        &self.m_assembly_src_guid
    }

    /// Model.hpp:1193
    pub fn set_assembly_src_guid(&mut self, guid: &str) {
        self.m_assembly_src_guid = guid.to_string();
    }

    /// Model.cpp:3627-3632 `ModelVolume::get_assemble_transformation() const`
    pub fn get_assemble_transformation(&self) -> &Transformation {
        if !self.m_assemble_initialized {
            return &self.m_transformation;
        }
        &self.m_assemble_transformation
    }

    /// Model.cpp:3640-3644 `ModelVolume::set_assemble_from_transform(const Transform3d &)`
    pub fn set_assemble_from_transform(&mut self, transform: &Transform3d) {
        self.m_assemble_initialized = true;
        self.m_assemble_transformation.set_from_transform(*transform);
    }

    /// Model.cpp:3338-3349 `int ModelVolume::extruder_id() const`. `object` is `this->object`, the
    /// object that holds the volume.
    pub fn extruder_id(&self, object: &ModelObject) -> i32 {
        let mut opt = self.config_extruder;
        if opt.is_none() || opt == Some(0) {
            opt = object.config_extruder;
        }
        match opt {
            None => 1,
            Some(v) => v,
        }
    }
}

/// `boost::uuids::to_string(boost::uuids::random_generator()())`: a version 4 UUID, lower case.
fn random_uuid_string() -> String {
    let mut bytes: [u8; 16] = rand::random();
    // random_generator.hpp: the variant is 10xxxxxx, the version 0100xxxx.
    bytes[8] = (bytes[8] & 0xBF) | 0x80;
    bytes[6] = (bytes[6] & 0x4F) | 0x40;
    let mut out = String::with_capacity(36);
    for (i, b) in bytes.iter().enumerate() {
        out.push_str(&format!("{:02x}", b));
        if i == 3 || i == 5 || i == 7 || i == 9 {
            out.push('-');
        }
    }
    out
}

/// Model.hpp:1440 `class ModelInstance`, the members this unit uses.
#[derive(Debug, Clone)]
pub struct ModelInstance {
    /// Model.hpp `Geometry::Transformation m_transformation`
    pub m_transformation: Transformation,
    /// Model.hpp `mutable Geometry::Transformation m_assemble_transformation`
    pub m_assemble_transformation: RefCell<Transformation>,
    /// Model.hpp:1461
    pub m_offset_to_assembly: Vec3d,
    /// Model.hpp:1462
    pub m_assemble_initialized: bool,
    /// Model.hpp:1463 `mutable bool m_assemble_scalling_factor_dirty{ true }`
    pub m_assemble_scalling_factor_dirty: RefCell<bool>,
}

impl Default for ModelInstance {
    /// Model.hpp:1577 `explicit ModelInstance(ModelObject* object)`
    fn default() -> Self {
        ModelInstance {
            m_transformation: Transformation::default(),
            m_assemble_transformation: RefCell::new(Transformation::default()),
            m_offset_to_assembly: Vec3d::new(0.0, 0.0, 0.0),
            m_assemble_initialized: false,
            m_assemble_scalling_factor_dirty: RefCell::new(true),
        }
    }
}

impl ModelInstance {
    /// Model.cpp:3978-3986 `ModelInstance::get_assemble_transformation() const`
    pub fn get_assemble_transformation(&self) -> Transformation {
        if *self.m_assemble_scalling_factor_dirty.borrow() {
            let scaling_factor = self.m_transformation.get_scaling_factor();
            self.m_assemble_transformation.borrow_mut().set_scaling_factor(&scaling_factor);
            *self.m_assemble_scalling_factor_dirty.borrow_mut() = false;
        }
        *self.m_assemble_transformation.borrow()
    }

    /// Model.cpp:3994-3998 `ModelInstance::set_assemble_from_transform(const Transform3d &)`
    pub fn set_assemble_from_transform(&mut self, transform: &Transform3d) {
        self.m_assemble_initialized = true;
        self.m_assemble_transformation.borrow_mut().set_from_transform(*transform);
    }

    /// Model.hpp:1499
    pub fn set_offset_to_assembly(&mut self, offset: Vec3d) {
        self.m_offset_to_assembly = offset;
    }

    /// Model.hpp:1500
    pub fn get_offset_to_assembly(&self) -> Vec3d {
        self.m_offset_to_assembly
    }
}

/// Model.hpp:344 `class ModelObject`, the members this unit uses. `volumes` and `instances` are
/// vectors of pointers in C++, and this unit checks them for null: a `None` is a null pointer.
#[derive(Debug, Clone, Default)]
pub struct ModelObject {
    /// ObjectID.hpp:84 `ObjectBase::m_id`
    pub m_id: ObjectID,
    /// Model.hpp:347
    pub name: String,
    /// `config.option("extruder")` of the object, which `ModelVolume::extruder_id()` falls back on.
    pub config_extruder: Option<i32>,
    /// Model.hpp:356 `ModelVolumePtrs volumes`
    pub volumes: Vec<Option<ModelVolume>>,
    /// Model.hpp:353 `ModelInstancePtrs instances`
    pub instances: Vec<Option<ModelInstance>>,
}

impl ModelObject {
    /// ObjectID.hpp:55 `ObjectID id() const`
    pub fn id(&self) -> ObjectID {
        self.m_id
    }

    /// Model.cpp:1476-1491 `ModelObject::add_volume(const ModelVolume &other, ModelVolumeType type
    /// = INVALID)`: the copy keeps the id of `other` (Model.hpp:1303).
    pub fn add_volume(&mut self, other: &ModelVolume) -> &mut ModelVolume {
        self.volumes.push(Some(other.clone()));
        self.volumes.last_mut().and_then(|v| v.as_mut()).expect("the volume just added")
    }

    /// Model.cpp:1603-1612 `ModelInstance* ModelObject::add_instance()`
    pub fn add_instance(&mut self) -> &mut ModelInstance {
        self.instances.push(Some(ModelInstance::default()));
        self.instances.last_mut().and_then(|i| i.as_mut()).expect("the instance just added")
    }
}

/// Model.hpp:1581 `class Model`, the members this unit uses. `objects` is a vector of pointers in
/// C++, and this unit checks them for null: a `None` is a null pointer.
#[derive(Debug, Clone, Default)]
pub struct Model {
    /// Model.hpp:1588 `ModelObjectPtrs objects`
    pub objects: Vec<Option<ModelObject>>,
}

impl Model {
    /// Model.cpp:719-732 `void Model::clear_objects()`
    pub fn clear_objects(&mut self) {
        self.objects.clear();
    }

    /// Model.cpp:581-585 `ModelObject* Model::add_object()`: a new object, with a new id.
    pub fn add_object(&mut self) -> &mut ModelObject {
        self.objects.push(Some(ModelObject { m_id: ObjectBase::new().id(), ..ModelObject::default() }));
        self.objects.last_mut().and_then(|o| o.as_mut()).expect("the object just added")
    }
}

// ---------------------------------------------------------------------------------------------
// AssemblyStepsJson.hpp
// ---------------------------------------------------------------------------------------------

/// AssemblyStepsJson.hpp:19-27
#[derive(Debug, Clone, PartialEq)]
pub struct ArrowSvgRay {
    pub bound_volumes: Vec<(i32, i32)>,
    /// offset from bound-volumes (or step) bbox screen center
    pub arrow_start_offset: Vec2d,
    /// offset from arrow start → shared SVG icon center
    pub arrow_end_offset: Vec2d,
}

impl Default for ArrowSvgRay {
    fn default() -> Self {
        ArrowSvgRay {
            bound_volumes: Vec::new(),
            arrow_start_offset: Vec2d::zeros(),
            arrow_end_offset: Vec2d::new(80.0, -60.0),
        }
    }
}

/// AssemblyStepsJson.hpp:29-38
#[derive(Debug, Clone, PartialEq)]
pub struct ArrowSvgNote {
    pub svg_name: String,
    pub label_size: Vec2d,
    pub color: [i32; 4],
    pub rays: Vec<ArrowSvgRay>,
}

impl Default for ArrowSvgNote {
    fn default() -> Self {
        ArrowSvgNote {
            svg_name: String::new(),
            label_size: Vec2d::new(56.0, 56.0),
            color: [63, 130, 240, 255],
            rays: Vec::new(),
        }
    }
}

/// AssemblyStepsJson.hpp:40-51
#[derive(Debug, Clone, PartialEq)]
pub struct TextLabelNote {
    pub text: String,
    pub bound_volumes: Vec<(i32, i32)>,
    pub pos_offset: Vec2d,
    pub size: Vec2d,
    pub color: [i32; 4],
    pub background_color: [i32; 4],
}

impl Default for TextLabelNote {
    fn default() -> Self {
        TextLabelNote {
            text: "Note".to_string(),
            bound_volumes: Vec::new(),
            pos_offset: Vec2d::new(60.0, -60.0),
            size: Vec2d::new(160.0, 80.0),
            color: [255, 255, 255, 255],
            background_color: [63, 130, 240, 255],
        }
    }
}

/// AssemblyStepsJson.hpp:53-62
#[derive(Debug, Clone, PartialEq)]
pub struct CircleNote {
    pub bound_volumes: Vec<(i32, i32)>,
    pub pos_offset: Vec2d,
    pub size: Vec2d,
    pub color: [i32; 4],
}

impl Default for CircleNote {
    fn default() -> Self {
        CircleNote {
            bound_volumes: Vec::new(),
            pos_offset: Vec2d::new(60.0, -60.0),
            size: Vec2d::new(80.0, 80.0),
            color: [63, 130, 240, 255],
        }
    }
}

/// AssemblyStepsJson.hpp:64-73
#[derive(Debug, Clone, PartialEq)]
pub struct RectangleNote {
    pub bound_volumes: Vec<(i32, i32)>,
    pub pos_offset: Vec2d,
    pub size: Vec2d,
    pub color: [i32; 4],
}

impl Default for RectangleNote {
    fn default() -> Self {
        RectangleNote {
            bound_volumes: Vec::new(),
            pos_offset: Vec2d::new(60.0, -60.0),
            size: Vec2d::new(80.0, 80.0),
            color: [63, 130, 240, 255],
        }
    }
}

/// AssemblyStepsJson.hpp:75-84
#[derive(Debug, Clone, PartialEq)]
pub struct PlainArrowNote {
    pub bound_volumes: Vec<(i32, i32)>,
    pub arrow_start_offset: Vec2d,
    pub arrow_end_offset: Vec2d,
    pub color: [i32; 4],
}

impl Default for PlainArrowNote {
    fn default() -> Self {
        PlainArrowNote {
            bound_volumes: Vec::new(),
            arrow_start_offset: Vec2d::zeros(),
            arrow_end_offset: Vec2d::new(80.0, -60.0),
            color: [63, 130, 240, 255],
        }
    }
}

/// AssemblyStepsJson.hpp:86-99
#[derive(Debug, Clone, PartialEq)]
pub struct PartNumberLabel {
    pub object_idx: i32,
    pub volume_idx: i32,
    pub part_name: String,
    pub part_guid: String,
    pub arrow_start_offset: Vec2d,
    pub arrow_end_offset: Vec2d,
    pub visible: bool,
    pub in_explosion_state: bool,
}

impl Default for PartNumberLabel {
    fn default() -> Self {
        PartNumberLabel {
            object_idx: -1,
            volume_idx: -1,
            part_name: String::new(),
            part_guid: String::new(),
            arrow_start_offset: Vec2d::zeros(),
            arrow_end_offset: Vec2d::new(60.0, -50.0),
            visible: true,
            in_explosion_state: false,
        }
    }
}

/// AssemblyStepsJson.hpp:101-113
#[derive(Debug, Clone, PartialEq)]
pub struct AssemblyNote {
    pub arrow_svgs: Vec<ArrowSvgNote>,
    pub text_labels: Vec<TextLabelNote>,
    pub circle_notes: Vec<CircleNote>,
    pub rectangle_notes: Vec<RectangleNote>,
    pub plain_arrows: Vec<PlainArrowNote>,
    pub part_number_labels: Vec<PartNumberLabel>,
    /// guide_show_part_numbers
    pub show_part_labels: bool,
}

impl Default for AssemblyNote {
    fn default() -> Self {
        AssemblyNote {
            arrow_svgs: Vec::new(),
            text_labels: Vec::new(),
            circle_notes: Vec::new(),
            rectangle_notes: Vec::new(),
            plain_arrows: Vec::new(),
            part_number_labels: Vec::new(),
            show_part_labels: true,
        }
    }
}

/// AssemblyStepsJson.hpp:114 `enum class LabelsShowType { AutoRecommend, OnlyModelObject,
/// OnlyModelVolume }`. `KeyFrame::from_json` casts whatever integer the file holds to it and
/// `to_json` writes it back, so it is kept as the integer it is.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LabelsShowType(pub i32);

impl LabelsShowType {
    pub const AUTO_RECOMMEND: LabelsShowType = LabelsShowType(0);
    pub const ONLY_MODEL_OBJECT: LabelsShowType = LabelsShowType(1);
    pub const ONLY_MODEL_VOLUME: LabelsShowType = LabelsShowType(2);
}

/// AssemblyStepsJson.hpp:115-165
#[derive(Debug, Clone, PartialEq)]
pub struct KeyFrame {
    pub id: i32,
    pub name: String,
    pub is_sub_assembly: bool,
    pub is_camera_define: bool,
    pub camera_user_defined: bool,
    pub view_matrix: Transform3d,
    pub projection_matrix: Transform3d,
    pub camera_target: Vec3d,
    pub camera_zoom: f64,
    pub camera_margin_factor: f64,
    pub object_transformations: BTreeMap<i32, Transformation>,
    pub volume_transformations: BTreeMap<(i32, i32), Transformation>,
    pub volume_names: BTreeMap<(i32, i32), String>,
    pub volume_guids: BTreeMap<(i32, i32), String>,
    pub assembly_note: AssemblyNote,
    pub labels_show_type: LabelsShowType,
    /// no need to save
    pub is_interpolation: bool,
    /// no need to save
    pub play_node_idx: i32,
    /// no need to save
    pub play_frame_idx: i32,
}

impl Default for KeyFrame {
    fn default() -> Self {
        KeyFrame {
            id: 0,
            name: String::new(),
            is_sub_assembly: false,
            is_camera_define: false,
            camera_user_defined: false,
            view_matrix: Transform3d::identity(),
            projection_matrix: Transform3d::identity(),
            camera_target: Vec3d::zeros(),
            camera_zoom: 1.0,
            camera_margin_factor: 1.4,
            object_transformations: BTreeMap::new(),
            volume_transformations: BTreeMap::new(),
            volume_names: BTreeMap::new(),
            volume_guids: BTreeMap::new(),
            assembly_note: AssemblyNote::default(),
            labels_show_type: LabelsShowType::AUTO_RECOMMEND,
            is_interpolation: false,
            play_node_idx: -1,
            play_frame_idx: -1,
        }
    }
}

impl KeyFrame {
    /// AssemblyStepsJson.hpp:137-139
    pub fn is_last(&self) -> bool {
        self.id == 0
    }

    /// AssemblyStepsJson.hpp:140-142
    pub fn is_start(&self) -> bool {
        self.id == 1
    }

    /// AssemblyStepsJson.hpp:143-145
    pub fn is_transition(&self) -> bool {
        self.id > 1
    }

    /// AssemblyStepsJson.hpp:146-163
    pub fn clone_from(&mut self, src: &KeyFrame) {
        self.id = src.id;
        self.name = src.name.clone();
        self.is_sub_assembly = src.is_sub_assembly;
        self.is_camera_define = src.is_camera_define;
        self.camera_user_defined = src.camera_user_defined;
        self.view_matrix = src.view_matrix;
        self.projection_matrix = src.projection_matrix;
        self.camera_target = src.camera_target;
        self.camera_zoom = src.camera_zoom;
        self.camera_margin_factor = src.camera_margin_factor;
        self.object_transformations = src.object_transformations.clone();
        self.volume_transformations = src.volume_transformations.clone();
        self.volume_names = src.volume_names.clone();
        self.volume_guids = src.volume_guids.clone();
        self.assembly_note = src.assembly_note.clone();
        self.labels_show_type = src.labels_show_type;
    }
}

/// AssemblyStepsJson.hpp:169
pub type KeyFrameVector = Vec<KeyFrame>;

/// AssemblyStepsJson.hpp:170-182 `struct AssembleBaseInfo`: the members of the base class. The
/// two classes derived from it hold one as `base`; a `std::shared_ptr<AssembleBaseInfo>` is an
/// `Arc<AssembleItem>`.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct AssembleBaseInfo {
    pub name: String,
    pub keyframes: KeyFrameVector,
}

/// AssemblyStepsJson.hpp:184-192
#[derive(Debug, Clone, PartialEq)]
pub struct AssembleSingleInfo {
    pub base: AssembleBaseInfo,
    pub object_idx: i32,
    pub object_id: usize,
}

impl Default for AssembleSingleInfo {
    fn default() -> Self {
        AssembleSingleInfo { base: AssembleBaseInfo::default(), object_idx: -1, object_id: 0 }
    }
}

impl AssembleSingleInfo {
    /// AssemblyStepsJson.hpp:189
    pub fn get_type(&self) -> String {
        "single".to_string()
    }
}

/// AssemblyStepsJson.hpp:194-198 `namespace AssemblyStepKind`
#[allow(non_snake_case)]
pub mod AssemblyStepKind {
    #![allow(non_upper_case_globals)]
    pub const Normal: i32 = 0;
    pub const FinalAssembly: i32 = 1;
    pub const OverallPreview: i32 = 2;
}

/// AssemblyStepsJson.hpp:200-213
#[derive(Debug, Clone, PartialEq)]
pub struct AssembleSub {
    pub base: AssembleBaseInfo,
    pub id: i32,
    pub step: i32,
    pub is_final_assembly: i32,
    pub inherited_from_step_id: i32,
    pub inherited_object_ids: Vec<usize>,
    pub children: Vec<Arc<AssembleItem>>,
    pub assembly_tree_checked: Option<HashMap<String, bool>>,
}

impl Default for AssembleSub {
    fn default() -> Self {
        AssembleSub {
            base: AssembleBaseInfo::default(),
            id: -1,
            step: 0,
            is_final_assembly: AssemblyStepKind::Normal,
            inherited_from_step_id: -1,
            inherited_object_ids: Vec::new(),
            children: Vec::new(),
            assembly_tree_checked: None,
        }
    }
}

impl AssembleSub {
    /// AssemblyStepsJson.hpp:210
    pub fn get_type(&self) -> String {
        "sub".to_string()
    }
}

/// What a `std::shared_ptr<AssembleBaseInfo>` points to: one of the two classes derived from
/// `AssembleBaseInfo`. The virtual functions of the base dispatch here.
#[derive(Debug, Clone, PartialEq)]
pub enum AssembleItem {
    Single(AssembleSingleInfo),
    Sub(AssembleSub),
}

impl AssembleItem {
    /// The `AssembleBaseInfo` part of the object.
    pub fn base(&self) -> &AssembleBaseInfo {
        match self {
            AssembleItem::Single(single) => &single.base,
            AssembleItem::Sub(sub) => &sub.base,
        }
    }

    /// AssemblyStepsJson.hpp:177 `virtual std::string get_type() const = 0`
    pub fn get_type(&self) -> String {
        match self {
            AssembleItem::Single(single) => single.get_type(),
            AssembleItem::Sub(sub) => sub.get_type(),
        }
    }

    /// AssemblyStepsJson.hpp:178 `virtual void to_json(nlohmann::json &j) const`
    pub fn to_json(&self, j: &mut Value) {
        match self {
            AssembleItem::Single(single) => single.to_json(j),
            AssembleItem::Sub(sub) => sub.to_json(j),
        }
    }

    /// AssemblyStepsJson.hpp:179 `virtual void from_json(const nlohmann::json &j)`
    pub fn from_json(&mut self, j: &Value) {
        match self {
            AssembleItem::Single(single) => single.from_json(j),
            AssembleItem::Sub(sub) => sub.from_json(j),
        }
    }

    /// `dynamic_cast<const AssembleSub*>(item)`
    pub fn as_sub(&self) -> Option<&AssembleSub> {
        match self {
            AssembleItem::Sub(sub) => Some(sub),
            AssembleItem::Single(_) => None,
        }
    }

    /// `dynamic_cast<const AssembleSingleInfo*>(item)`
    pub fn as_single(&self) -> Option<&AssembleSingleInfo> {
        match self {
            AssembleItem::Single(single) => Some(single),
            AssembleItem::Sub(_) => None,
        }
    }
}

/// AssemblyStepsJson.hpp:215-228
#[derive(Debug, Clone, PartialEq)]
pub struct KeyFrameEntry {
    pub need_save: bool,
    pub data: KeyFrame,
}

impl Default for KeyFrameEntry {
    fn default() -> Self {
        KeyFrameEntry { need_save: true, data: KeyFrame::default() }
    }
}

impl KeyFrameEntry {
    /// AssemblyStepsJson.hpp:220
    pub fn is_last(&self) -> bool {
        self.data.is_last()
    }

    /// AssemblyStepsJson.hpp:221
    pub fn is_start(&self) -> bool {
        self.data.is_start()
    }

    /// AssemblyStepsJson.hpp:222
    pub fn is_transition(&self) -> bool {
        self.data.is_transition()
    }

    /// AssemblyStepsJson.hpp:223-227
    pub fn clone_from(&mut self, src: &KeyFrameEntry) {
        self.data.clone_from(&src.data);
        self.need_save = true;
    }
}

/// AssemblyStepsJson.hpp:230
pub type KeyFrameEntryVector = Vec<KeyFrameEntry>;

/// AssemblyStepsJson.hpp:231-237
#[derive(Debug, Clone, PartialEq)]
pub struct KFNodeData {
    pub node_idx: i32,
    pub object_idx: i32,
    pub is_folder: bool,
    pub entries: KeyFrameEntryVector,
}

impl Default for KFNodeData {
    fn default() -> Self {
        KFNodeData { node_idx: -1, object_idx: -1, is_folder: false, entries: Vec::new() }
    }
}

/// AssemblyStepsJson.hpp:239-243
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PlayFrameRef {
    pub node_idx: i32,
    pub frame_idx: i32,
}

impl Default for PlayFrameRef {
    fn default() -> Self {
        PlayFrameRef { node_idx: -1, frame_idx: -1 }
    }
}

/// AssemblyStepsJson.hpp:245
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AssemblyNoteSelectionType {
    None,
    ArrowSvg,
    TextLabel,
    Circle,
    PlainArrow,
    Rectangle,
}

/// AssemblyStepsJson.hpp:246
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum KeyframeDisplayMode {
    OnlyCurrentStep,
    Highlight,
    All,
}

/// AssemblyStepsJson.hpp:247
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PlayStrategy {
    Sequential,
    SubAssemblyFirst,
}

/// AssemblyStepsJson.hpp:248
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AssemblyNoteCursorType {
    Standard,
    Hand,
    Move,
    ResizeNWSE,
    ResizeNESW,
}

/// AssemblyStepsJson.hpp:252 `AssemblyStepsTreeNode::Type`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AssemblyStepsTreeNodeType {
    Folder,
    Object,
    Volume,
}

/// AssemblyStepsJson.hpp:250-267
#[derive(Debug, Clone, PartialEq)]
pub struct AssemblyStepsTreeNode {
    pub r#type: AssemblyStepsTreeNodeType,
    /// valid when type == Folder
    pub id: i32,
    /// valid when type == Folder
    pub step: i32,
    pub name: String,
    /// valid when type == Object
    pub object_idx: i32,
    /// ModelObject id, stable across object index changes
    pub object_id: usize,
    /// per-object render visibility (drives GLVolume show/hide)
    pub visible: bool,
    pub is_final_assembly: i32,
    pub inherited_from_step_id: i32,
    pub inherited_object_ids: Vec<usize>,
    /// indices into nodes
    pub children: Vec<i32>,
    pub assembly_tree_checked: Option<HashMap<String, bool>>,
    pub kf_data: KFNodeData,
}

impl Default for AssemblyStepsTreeNode {
    fn default() -> Self {
        AssemblyStepsTreeNode {
            r#type: AssemblyStepsTreeNodeType::Folder,
            id: -1,
            step: 0,
            name: String::new(),
            object_idx: -1,
            object_id: 0,
            visible: true,
            is_final_assembly: AssemblyStepKind::Normal,
            inherited_from_step_id: -1,
            inherited_object_ids: Vec::new(),
            children: Vec::new(),
            assembly_tree_checked: None,
            kf_data: KFNodeData::default(),
        }
    }
}

/// AssemblyStepsJson.hpp:270-296
#[derive(Debug, Clone, PartialEq, Default)]
pub struct AssemblyStepsTreeData {
    pub nodes: Vec<AssemblyStepsTreeNode>,
    pub roots: Vec<i32>,
    pub camera_ref_viewport_w: i32,
    pub camera_ref_viewport_h: i32,

    pub has_loaded_recorded_baseline: bool,
    pub loaded_recorded_volumes: BTreeSet<String>,
}

impl AssemblyStepsTreeData {
    /// AssemblyStepsJson.hpp:280
    pub fn empty(&self) -> bool {
        self.nodes.is_empty()
    }

    /// AssemblyStepsJson.hpp:281-289
    pub fn clear(&mut self) {
        self.nodes.clear();
        self.roots.clear();
        self.camera_ref_viewport_w = 0;
        self.camera_ref_viewport_h = 0;
        self.has_loaded_recorded_baseline = false;
        self.loaded_recorded_volumes.clear();
    }
}

/// AssemblyStepsJson.hpp:301-303 `AssemblyStepJson::PdfExportParams`
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct PdfExportParams {
    pub title: String,
}

/// AssemblyStepsJson.hpp:298-334 `class AssemblyStepJson`
#[derive(Debug, Clone, PartialEq, Default)]
pub struct AssemblyStepJson {
    m_items: Vec<Arc<AssembleItem>>,
    m_pdf_export_params: PdfExportParams,
    m_assembly_part_number_label_font_size: f32,
    m_camera_ref_viewport_w: i32,
    m_camera_ref_viewport_h: i32,
}

impl AssemblyStepJson {
    /// AssemblyStepsJson.hpp:312
    pub fn get_items(&self) -> &Vec<Arc<AssembleItem>> {
        &self.m_items
    }

    /// AssemblyStepsJson.hpp:313-314 (the copying and the moving overload)
    pub fn set_items(&mut self, items: Vec<Arc<AssembleItem>>) {
        self.m_items = items;
    }

    /// AssemblyStepsJson.hpp:315
    pub fn get_pdf_export_params(&self) -> &PdfExportParams {
        &self.m_pdf_export_params
    }

    /// AssemblyStepsJson.hpp:316
    pub fn set_pdf_export_params(&mut self, params: &PdfExportParams) {
        self.m_pdf_export_params = params.clone();
    }

    /// AssemblyStepsJson.hpp:317
    pub fn get_assembly_part_number_label_font_size(&self) -> f32 {
        self.m_assembly_part_number_label_font_size
    }

    /// AssemblyStepsJson.hpp:318
    pub fn set_assembly_part_number_label_font_size(&mut self, font_size: f32) {
        self.m_assembly_part_number_label_font_size = font_size;
    }

    /// AssemblyStepsJson.hpp:319
    pub fn get_camera_ref_viewport_w(&self) -> i32 {
        self.m_camera_ref_viewport_w
    }

    /// AssemblyStepsJson.hpp:320
    pub fn get_camera_ref_viewport_h(&self) -> i32 {
        self.m_camera_ref_viewport_h
    }

    /// AssemblyStepsJson.hpp:321
    pub fn set_camera_ref_viewport(&mut self, w: i32, h: i32) {
        self.m_camera_ref_viewport_w = w;
        self.m_camera_ref_viewport_h = h;
    }
}

/// AssemblyStepsJson.hpp:336
pub const K_ASSEMBLY_STEPS_QUALIFIED_KEY: &str = "qualified_assembly";

/// AssemblyStepsJson.hpp:338
pub const K_QUALIFIED_ASSEMBLY_RESULT_KEY: &str = "qualified_assembly";
/// AssemblyStepsJson.hpp:339
pub const K_NEED_REWRITE_3MF_RESULT_KEY: &str = "need_rewrite_3mf";

/// AssemblyStepsJson.hpp:352
pub const K_ASSEMBLY_STEPS_RECOMPUTE_BELOW_VERSION: f64 = 1.2;

/// AssemblyStepsJson.hpp:354-358
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct AssemblyStepsFileMeta {
    pub version: f64,
    pub has_qualified: bool,
    pub qualified: bool,
}

/// AssemblyStepsJson.hpp:361-364
pub fn assembly_steps_version_needs_recompute(version: f64) -> bool {
    version < K_ASSEMBLY_STEPS_RECOMPUTE_BELOW_VERSION - 1e-3
}

// ---------------------------------------------------------------------------------------------
// AssemblyStepsJson.cpp
// ---------------------------------------------------------------------------------------------

/// AssemblyStepsJson.cpp:17-23
fn json_get_string(j: &Value, key: &str, value: &mut String) -> bool {
    if !contains(j, key) || !j[key].is_string() {
        return false;
    }
    *value = j[key].as_str().unwrap_or("").to_string();
    true
}

/// AssemblyStepsJson.cpp:25-31
fn json_get_int(j: &Value, key: &str, value: &mut i32) -> bool {
    if !contains(j, key) || !is_number_integer(&j[key]) {
        return false;
    }
    *value = get_int(&j[key]);
    true
}

/// AssemblyStepsJson.cpp:33-42
fn json_get_size_t(j: &Value, key: &str, value: &mut usize) -> bool {
    if !contains(j, key) || !is_number_integer(&j[key]) {
        return false;
    }
    let raw = get_long_long(&j[key]);
    if raw < 0 {
        return false;
    }
    *value = raw as usize;
    true
}

/// AssemblyStepsJson.cpp:44-50
fn json_get_bool(j: &Value, key: &str, value: &mut bool) -> bool {
    if !contains(j, key) || !j[key].is_boolean() {
        return false;
    }
    *value = j[key].as_bool().unwrap_or(false);
    true
}

/// AssemblyStepsJson.cpp:52-70
///
/// Read is_final_assembly as int.
/// Legacy bool true → FinalAssembly (1) so old Default/final-assembly data is kept.
/// OverallPreview (2) is runtime UI-only and must not appear in persisted JSON;
/// if an older mid-migration file still has 2, callers skip it on restore.
fn json_get_assembly_step_kind(j: &Value, key: &str, value: &mut i32) -> bool {
    if !contains(j, key) {
        return false;
    }
    let v = &j[key];
    if v.is_boolean() {
        *value = if v.as_bool().unwrap_or(false) { AssemblyStepKind::FinalAssembly } else { AssemblyStepKind::Normal };
        return true;
    }
    if is_number_integer(v) {
        *value = get_int(v);
        return true;
    }
    false
}

/// AssemblyStepsJson.cpp:72-78
fn json_get_double(j: &Value, key: &str, value: &mut f64) -> bool {
    if !contains(j, key) || !j[key].is_number() {
        return false;
    }
    *value = get_double(&j[key]);
    true
}

/// AssemblyStepsJson.cpp:80-87
fn json_get_vec2d(j: &Value, key: &str, value: &mut Vec2d) -> bool {
    if !contains(j, key) || !j[key].is_array() || size(&j[key]) != 2 || !j[key][0].is_number() || !j[key][1].is_number() {
        return false;
    }
    *value = Vec2d::new(get_double(&j[key][0]), get_double(&j[key][1]));
    true
}

/// AssemblyStepsJson.cpp:89-92
fn color_to_json(j: &mut Value, key: &str, color: &[i32; 4]) {
    set(j, key, Value::Array(vec![Value::from(color[0]), Value::from(color[1]), Value::from(color[2]), Value::from(color[3])]));
}

/// AssemblyStepsJson.cpp:94-101
fn bound_volumes_to_json(j: &mut Value, key: &str, bound_volumes: &[(i32, i32)]) {
    let mut bv: Vec<Value> = Vec::new();
    for p in bound_volumes {
        bv.push(Value::Array(vec![Value::from(p.0), Value::from(p.1)]));
    }
    set(j, key, Value::Array(bv));
}

/// AssemblyStepsJson.cpp:103-113
fn bound_volumes_from_json(j: &Value, key: &str, bound_volumes: &mut Vec<(i32, i32)>) {
    bound_volumes.clear();
    if contains(j, key) && j[key].is_array() {
        for item in elements(&j[key]) {
            if item.is_array() && size(item) == 2 && is_number_integer(&item[0]) && is_number_integer(&item[1]) {
                bound_volumes.push((get_int(&item[0]), get_int(&item[1])));
            }
        }
    }
}

/// AssemblyStepsJson.cpp:115-137
///
/// The `try` of the C++ has nothing to catch: `get<double>()` is only called on a number.
fn color_from_json(j: &Value, key: &str, color: &mut [i32; 4]) {
    if !contains(j, key) || !j[key].is_array() {
        return;
    }

    let arr = &j[key];
    if size(arr) != 3 && size(arr) != 4 {
        return;
    }

    for i in 0..size(arr) {
        if !arr[i].is_number() {
            return;
        }
        // static_cast<int>(std::round(double)): the reference build (arm64) saturates.
        let mut value = get_double(&arr[i]).round() as i32;
        value = if value < 0 {
            0
        } else if value > 255 {
            255
        } else {
            value
        };
        color[i] = value;
    }
    if size(arr) == 3 {
        color[3] = 255;
    }
}

// ---- Transform3d helpers (file-local: only used by this translation unit) ----

/// AssemblyStepsJson.cpp:142-150
fn transform3d_to_json(t: &Transform3d) -> Value {
    let mut arr: Vec<Value> = Vec::new();
    let m = t;
    for r in 0..4 {
        for c in 0..4 {
            arr.push(json_float(m[(r, c)]));
        }
    }
    Value::Array(arr)
}

/// AssemblyStepsJson.cpp:152-168
fn transform3d_from_json(j: &Value) -> Transform3d {
    let mut t = Transform3d::identity();
    if !j.is_array() || size(j) != 16 {
        return t;
    }
    let m = &mut t;
    let mut idx = 0;
    for r in 0..4 {
        for c in 0..4 {
            let cell = &j[idx];
            if !cell.is_number() {
                return Transform3d::identity();
            }
            m[(r, c)] = get_double(cell);
            idx += 1;
        }
    }
    t
}

// ---- ArrowSvgRay ----
impl ArrowSvgRay {
    /// AssemblyStepsJson.cpp:170-175
    pub fn to_json(&self, j: &mut Value) {
        bound_volumes_to_json(j, "bound_volumes", &self.bound_volumes);
        set(j, "arrow_start_offset", json_floats(&[self.arrow_start_offset.x, self.arrow_start_offset.y]));
        set(j, "arrow_end_offset", json_floats(&[self.arrow_end_offset.x, self.arrow_end_offset.y]));
    }

    /// AssemblyStepsJson.cpp:177-182
    pub fn from_json(&mut self, j: &Value) {
        bound_volumes_from_json(j, "bound_volumes", &mut self.bound_volumes);
        json_get_vec2d(j, "arrow_start_offset", &mut self.arrow_start_offset);
        json_get_vec2d(j, "arrow_end_offset", &mut self.arrow_end_offset);
    }
}

// ---- ArrowSvgNote ----
impl ArrowSvgNote {
    /// AssemblyStepsJson.cpp:185-206
    pub fn to_json(&self, j: &mut Value) {
        set(j, "svg_name", Value::from(self.svg_name.clone()));
        set(j, "label_size", json_floats(&[self.label_size.x, self.label_size.y]));
        color_to_json(j, "color", &self.color);

        let mut rays_arr: Vec<Value> = Vec::new();
        for ray in &self.rays {
            let mut rj = Value::Null;
            ray.to_json(&mut rj);
            rays_arr.push(rj);
        }
        set(j, "rays", Value::Array(rays_arr));

        // Mirror the first ray into the legacy flat fields so older readers that
        // only know bound_volumes / arrow_*_offset still get a usable single arrow.
        if !self.rays.is_empty() {
            let front = &self.rays[0];
            bound_volumes_to_json(j, "bound_volumes", &front.bound_volumes);
            set(j, "arrow_start_offset", json_floats(&[front.arrow_start_offset.x, front.arrow_start_offset.y]));
            set(j, "arrow_end_offset", json_floats(&[front.arrow_end_offset.x, front.arrow_end_offset.y]));
        }
    }

    /// AssemblyStepsJson.cpp:208-229
    pub fn from_json(&mut self, j: &Value) {
        json_get_string(j, "svg_name", &mut self.svg_name);
        json_get_vec2d(j, "label_size", &mut self.label_size);
        color_from_json(j, "color", &mut self.color);

        self.rays.clear();
        if contains(j, "rays") && j["rays"].is_array() && size(&j["rays"]) != 0 {
            for rj in elements(&j["rays"]) {
                let mut ray = ArrowSvgRay::default();
                ray.from_json(rj);
                self.rays.push(ray);
            }
        } else {
            // Legacy single-arrow JSON: one combined ray from flat fields.
            let mut ray = ArrowSvgRay::default();
            bound_volumes_from_json(j, "bound_volumes", &mut ray.bound_volumes);
            json_get_vec2d(j, "arrow_start_offset", &mut ray.arrow_start_offset);
            json_get_vec2d(j, "arrow_end_offset", &mut ray.arrow_end_offset);
            self.rays.push(ray);
        }
    }
}

// ---- TextLabelNote ----
impl TextLabelNote {
    /// AssemblyStepsJson.cpp:232-240
    pub fn to_json(&self, j: &mut Value) {
        set(j, "text", Value::from(self.text.clone()));
        bound_volumes_to_json(j, "bound_volumes", &self.bound_volumes);
        set(j, "pos_offset", json_floats(&[self.pos_offset.x, self.pos_offset.y]));
        set(j, "size", json_floats(&[self.size.x, self.size.y]));
        color_to_json(j, "color", &self.color);
        color_to_json(j, "background_color", &self.background_color);
    }

    /// AssemblyStepsJson.cpp:242-250
    pub fn from_json(&mut self, j: &Value) {
        json_get_string(j, "text", &mut self.text);
        bound_volumes_from_json(j, "bound_volumes", &mut self.bound_volumes);
        json_get_vec2d(j, "pos_offset", &mut self.pos_offset);
        json_get_vec2d(j, "size", &mut self.size);
        color_from_json(j, "color", &mut self.color);
        color_from_json(j, "background_color", &mut self.background_color);
    }
}

// ---- CircleNote ----
impl CircleNote {
    /// AssemblyStepsJson.cpp:253-259
    pub fn to_json(&self, j: &mut Value) {
        bound_volumes_to_json(j, "bound_volumes", &self.bound_volumes);
        set(j, "pos_offset", json_floats(&[self.pos_offset.x, self.pos_offset.y]));
        set(j, "size", json_floats(&[self.size.x, self.size.y]));
        color_to_json(j, "color", &self.color);
    }

    /// AssemblyStepsJson.cpp:261-267
    pub fn from_json(&mut self, j: &Value) {
        bound_volumes_from_json(j, "bound_volumes", &mut self.bound_volumes);
        json_get_vec2d(j, "pos_offset", &mut self.pos_offset);
        json_get_vec2d(j, "size", &mut self.size);
        color_from_json(j, "color", &mut self.color);
    }
}

// ---- RectangleNote ----
impl RectangleNote {
    /// AssemblyStepsJson.cpp:270-276
    pub fn to_json(&self, j: &mut Value) {
        bound_volumes_to_json(j, "bound_volumes", &self.bound_volumes);
        set(j, "pos_offset", json_floats(&[self.pos_offset.x, self.pos_offset.y]));
        set(j, "size", json_floats(&[self.size.x, self.size.y]));
        color_to_json(j, "color", &self.color);
    }

    /// AssemblyStepsJson.cpp:278-284
    pub fn from_json(&mut self, j: &Value) {
        bound_volumes_from_json(j, "bound_volumes", &mut self.bound_volumes);
        json_get_vec2d(j, "pos_offset", &mut self.pos_offset);
        json_get_vec2d(j, "size", &mut self.size);
        color_from_json(j, "color", &mut self.color);
    }
}

// ---- PlainArrowNote ----
impl PlainArrowNote {
    /// AssemblyStepsJson.cpp:287-293
    pub fn to_json(&self, j: &mut Value) {
        bound_volumes_to_json(j, "bound_volumes", &self.bound_volumes);
        set(j, "arrow_start_offset", json_floats(&[self.arrow_start_offset.x, self.arrow_start_offset.y]));
        set(j, "arrow_end_offset", json_floats(&[self.arrow_end_offset.x, self.arrow_end_offset.y]));
        color_to_json(j, "color", &self.color);
    }

    /// AssemblyStepsJson.cpp:295-301
    pub fn from_json(&mut self, j: &Value) {
        bound_volumes_from_json(j, "bound_volumes", &mut self.bound_volumes);
        json_get_vec2d(j, "arrow_start_offset", &mut self.arrow_start_offset);
        json_get_vec2d(j, "arrow_end_offset", &mut self.arrow_end_offset);
        color_from_json(j, "color", &mut self.color);
    }
}

// ---- PartNumberLabel ----
impl PartNumberLabel {
    /// AssemblyStepsJson.cpp:304-314
    pub fn to_json(&self, j: &mut Value) {
        set(j, "object_idx", Value::from(self.object_idx));
        set(j, "volume_idx", Value::from(self.volume_idx));
        set(j, "part_name", Value::from(self.part_name.clone()));
        set(j, "uuid", Value::from(self.part_guid.clone()));
        set(j, "arrow_start_offset", json_floats(&[self.arrow_start_offset.x, self.arrow_start_offset.y]));
        set(j, "arrow_end_offset", json_floats(&[self.arrow_end_offset.x, self.arrow_end_offset.y]));
        set(j, "visible", Value::from(self.visible));
        set(j, "in_explosion_state", Value::from(self.in_explosion_state));
    }

    /// AssemblyStepsJson.cpp:316-330
    pub fn from_json(&mut self, j: &Value) {
        json_get_int(j, "object_idx", &mut self.object_idx);
        json_get_int(j, "volume_idx", &mut self.volume_idx);
        json_get_string(j, "part_name", &mut self.part_name);
        json_get_string(j, "uuid", &mut self.part_guid);
        json_get_vec2d(j, "arrow_start_offset", &mut self.arrow_start_offset);
        json_get_vec2d(j, "arrow_end_offset", &mut self.arrow_end_offset);
        // Default true for legacy JSON that predates the per-label switch.
        if !json_get_bool(j, "visible", &mut self.visible) {
            self.visible = true;
        }
        // Default false for legacy JSON that predates the explosion column.
        if !json_get_bool(j, "in_explosion_state", &mut self.in_explosion_state) {
            self.in_explosion_state = false;
        }
    }
}

// ---- AssemblyNote ----
impl AssemblyNote {
    /// AssemblyStepsJson.cpp:333-383
    pub fn to_json(&self, j: &mut Value) {
        let mut arr: Vec<Value> = Vec::new();
        for a in &self.arrow_svgs {
            let mut aj = Value::Null;
            a.to_json(&mut aj);
            arr.push(aj);
        }
        set(j, "arrow_svgs", Value::Array(arr));

        let mut text_arr: Vec<Value> = Vec::new();
        for text_label in &self.text_labels {
            let mut tj = Value::Null;
            text_label.to_json(&mut tj);
            text_arr.push(tj);
        }
        set(j, "text_labels", Value::Array(text_arr));

        let mut circle_arr: Vec<Value> = Vec::new();
        for circle in &self.circle_notes {
            let mut cj = Value::Null;
            circle.to_json(&mut cj);
            circle_arr.push(cj);
        }
        set(j, "circle_notes", Value::Array(circle_arr));

        let mut rect_arr: Vec<Value> = Vec::new();
        for rect in &self.rectangle_notes {
            let mut rj = Value::Null;
            rect.to_json(&mut rj);
            rect_arr.push(rj);
        }
        set(j, "rectangle_notes", Value::Array(rect_arr));

        let mut arrow_arr: Vec<Value> = Vec::new();
        for arrow in &self.plain_arrows {
            let mut aj = Value::Null;
            arrow.to_json(&mut aj);
            arrow_arr.push(aj);
        }
        set(j, "plain_arrows", Value::Array(arrow_arr));

        let mut pn_arr: Vec<Value> = Vec::new();
        for pn in &self.part_number_labels {
            let mut pj = Value::Null;
            pn.to_json(&mut pj);
            pn_arr.push(pj);
        }
        set(j, "part_number_labels", Value::Array(pn_arr));
        set(j, "show_part_labels", Value::from(self.show_part_labels));
    }

    /// AssemblyStepsJson.cpp:385-451
    pub fn from_json(&mut self, j: &Value) {
        self.arrow_svgs.clear();
        if contains(j, "arrow_svgs") && j["arrow_svgs"].is_array() {
            for aj in elements(&j["arrow_svgs"]) {
                let mut a = ArrowSvgNote::default();
                a.from_json(aj);
                self.arrow_svgs.push(a);
            }
        }

        self.text_labels.clear();
        if contains(j, "text_labels") && j["text_labels"].is_array() {
            for tj in elements(&j["text_labels"]) {
                let mut text_label = TextLabelNote::default();
                text_label.from_json(tj);
                self.text_labels.push(text_label);
            }
        }

        self.circle_notes.clear();
        if contains(j, "circle_notes") && j["circle_notes"].is_array() {
            for cj in elements(&j["circle_notes"]) {
                let mut circle = CircleNote::default();
                circle.from_json(cj);
                self.circle_notes.push(circle);
            }
        }

        self.rectangle_notes.clear();
        if contains(j, "rectangle_notes") && j["rectangle_notes"].is_array() {
            for rj in elements(&j["rectangle_notes"]) {
                let mut rect = RectangleNote::default();
                rect.from_json(rj);
                self.rectangle_notes.push(rect);
            }
        }

        self.plain_arrows.clear();
        if contains(j, "plain_arrows") && j["plain_arrows"].is_array() {
            for aj in elements(&j["plain_arrows"]) {
                let mut arrow = PlainArrowNote::default();
                arrow.from_json(aj);
                self.plain_arrows.push(arrow);
            }
        }

        self.part_number_labels.clear();
        let mut has_legacy_label_show = false;
        let mut legacy_label_show = false;
        if contains(j, "part_number_labels") && j["part_number_labels"].is_array() {
            for pj in elements(&j["part_number_labels"]) {
                let mut label_show = false;
                if json_get_bool(pj, "is_show", &mut label_show) {
                    has_legacy_label_show = true;
                    legacy_label_show = legacy_label_show || label_show;
                }
                let mut pn = PartNumberLabel::default();
                pn.from_json(pj);
                self.part_number_labels.push(pn);
            }
        }

        if !json_get_bool(j, "show_part_labels", &mut self.show_part_labels)
            && !json_get_bool(j, "is_show", &mut self.show_part_labels)
            && has_legacy_label_show
        {
            self.show_part_labels = legacy_label_show;
        }
    }
}

// ---- KeyFrame ----
impl KeyFrame {
    /// AssemblyStepsJson.cpp:454-499
    pub fn to_json(&self, j: &mut Value) {
        set(j, "id", Value::from(self.id));
        set(j, "name", Value::from(self.name.clone()));
        set(j, "is_camera_define", Value::from(self.is_camera_define));
        set(j, "camera_user_defined", Value::from(self.camera_user_defined));
        set(j, "view_matrix", transform3d_to_json(&self.view_matrix));
        set(j, "projection_matrix", transform3d_to_json(&self.projection_matrix));
        set(j, "camera_target", json_floats(&[self.camera_target.x, self.camera_target.y, self.camera_target.z]));
        set(j, "camera_zoom", json_float(self.camera_zoom));
        set(j, "camera_margin_factor", json_float(self.camera_margin_factor));
        set(j, "labels_show_type", Value::from(self.labels_show_type.0));
        // Per-object instance assemble matrices. Captured separately so replay can
        let mut ot_arr: Vec<Value> = Vec::new();
        for p in &self.object_transformations {
            let mut item = Value::Null;
            set(&mut item, "object_id", Value::from(*p.0));
            set(&mut item, "transformation", transform3d_to_json(&p.1.matrix()));
            ot_arr.push(item);
        }
        set(j, "object_transformations", Value::Array(ot_arr));

        let mut vt_arr: Vec<Value> = Vec::new();
        for p in &self.volume_transformations {
            let mut item = Value::Null;
            set(&mut item, "object_id", Value::from(p.0 .0));
            set(&mut item, "volume_idx", Value::from(p.0 .1));
            if let Some(it) = self.volume_names.get(p.0) {
                set(&mut item, "volume_name", Value::from(it.clone()));
            }
            if let Some(git) = self.volume_guids.get(p.0) {
                if !git.is_empty() {
                    set(&mut item, "uuid", Value::from(git.clone()));
                }
            }
            set(&mut item, "transformation", transform3d_to_json(&p.1.matrix()));
            vt_arr.push(item);
        }
        set(j, "volume_transformations", Value::Array(vt_arr));

        if !self.assembly_note.arrow_svgs.is_empty()
            || !self.assembly_note.text_labels.is_empty()
            || !self.assembly_note.circle_notes.is_empty()
            || !self.assembly_note.rectangle_notes.is_empty()
            || !self.assembly_note.plain_arrows.is_empty()
            || !self.assembly_note.part_number_labels.is_empty()
        {
            let mut nj = Value::Null;
            self.assembly_note.to_json(&mut nj);
            set(j, "assembly_note", nj);
        }
    }

    /// AssemblyStepsJson.cpp:501-559
    pub fn from_json(&mut self, j: &Value) {
        json_get_int(j, "id", &mut self.id);
        json_get_string(j, "name", &mut self.name);
        json_get_bool(j, "is_camera_define", &mut self.is_camera_define);
        json_get_bool(j, "camera_user_defined", &mut self.camera_user_defined);
        if contains(j, "view_matrix") {
            self.view_matrix = transform3d_from_json(&j["view_matrix"]);
        }
        if contains(j, "projection_matrix") {
            self.projection_matrix = transform3d_from_json(&j["projection_matrix"]);
        }
        if contains(j, "camera_target")
            && j["camera_target"].is_array()
            && size(&j["camera_target"]) == 3
            && j["camera_target"][0].is_number()
            && j["camera_target"][1].is_number()
            && j["camera_target"][2].is_number()
        {
            self.camera_target =
                Vec3d::new(get_double(&j["camera_target"][0]), get_double(&j["camera_target"][1]), get_double(&j["camera_target"][2]));
        }
        json_get_double(j, "camera_zoom", &mut self.camera_zoom);
        json_get_double(j, "camera_margin_factor", &mut self.camera_margin_factor);
        let mut labels_show_type_value = LabelsShowType::AUTO_RECOMMEND.0;
        if json_get_int(j, "labels_show_type", &mut labels_show_type_value) {
            self.labels_show_type = LabelsShowType(labels_show_type_value);
        }

        self.object_transformations.clear();
        if contains(j, "object_transformations") && j["object_transformations"].is_array() {
            for item in elements(&j["object_transformations"]) {
                let mut obj_id = -1;
                if !json_get_int(item, "object_id", &mut obj_id) || !contains(item, "transformation") {
                    continue;
                }
                let mat = transform3d_from_json(&item["transformation"]);
                self.object_transformations.insert(obj_id, Transformation::from_transform(mat));
            }
        }

        self.volume_transformations.clear();
        self.volume_names.clear();
        self.volume_guids.clear();
        if contains(j, "volume_transformations") && j["volume_transformations"].is_array() {
            for item in elements(&j["volume_transformations"]) {
                let mut obj_id = -1;
                if !json_get_int(item, "object_id", &mut obj_id) || !contains(item, "transformation") {
                    continue;
                }
                // Older test fixtures might omit volume_idx; default to 0 so a
                // single-volume object behaves the same as before the rename.
                let mut volume_idx = 0;
                json_get_int(item, "volume_idx", &mut volume_idx);
                let mat = transform3d_from_json(&item["transformation"]);
                let key = (obj_id, volume_idx);
                self.volume_transformations.insert(key, Transformation::from_transform(mat));
                let mut volume_name = String::new();
                if json_get_string(item, "volume_name", &mut volume_name) {
                    self.volume_names.insert(key, volume_name);
                }
                let mut volume_guid = String::new();
                if json_get_string(item, "uuid", &mut volume_guid) && !volume_guid.is_empty() {
                    self.volume_guids.insert(key, volume_guid);
                }
            }
        }

        self.assembly_note = AssemblyNote::default();
        if contains(j, "assembly_note") && j["assembly_note"].is_object() {
            self.assembly_note.from_json(&j["assembly_note"]);
        }
    }
}

// ---- AssembleBaseInfo ----
impl AssembleBaseInfo {
    /// AssemblyStepsJson.cpp:561-573. `get_type` is what the virtual `get_type()` of the object
    /// returns, which the base class cannot ask here.
    pub fn to_json(&self, get_type: &str, j: &mut Value) {
        set(j, "type", Value::from(get_type.to_string()));
        set(j, "name", Value::from(self.name.clone()));

        let mut kf_arr: Vec<Value> = Vec::new();
        for kf in &self.keyframes {
            let mut kf_j = Value::Null;
            kf.to_json(&mut kf_j);
            kf_arr.push(kf_j);
        }
        set(j, "keyframes", Value::Array(kf_arr));
    }

    /// AssemblyStepsJson.cpp:575-593
    ///
    /// The `try` around a keyframe has nothing to catch: `KeyFrame::from_json` checks the type of
    /// everything before it reads it.
    pub fn from_json(&mut self, j: &Value) {
        json_get_string(j, "name", &mut self.name);

        self.keyframes.clear();
        if contains(j, "keyframes") && j["keyframes"].is_array() {
            self.keyframes.reserve(size(&j["keyframes"]));
            for kf_j in elements(&j["keyframes"]) {
                self.keyframes.push(KeyFrame::default());
                let last = self.keyframes.len() - 1;
                self.keyframes[last].from_json(kf_j);
            }
        }
    }

    /// AssemblyStepsJson.cpp:595-610
    pub fn create_from_json(j: &Value) -> Option<Arc<AssembleItem>> {
        let mut type_str = String::new();
        json_get_string(j, "type", &mut type_str);

        if type_str == "sub" {
            let mut ptr = AssembleSub::default();
            ptr.from_json(j);
            return Some(Arc::new(AssembleItem::Sub(ptr)));
        } else if type_str == "single" {
            let mut ptr = AssembleSingleInfo::default();
            ptr.from_json(j);
            return Some(Arc::new(AssembleItem::Single(ptr)));
        }
        None
    }
}

// ---- AssembleSingleInfo ----
impl AssembleSingleInfo {
    /// AssemblyStepsJson.cpp:613-620
    pub fn to_json(&self, j: &mut Value) {
        self.base.to_json(&self.get_type(), j);
        if self.object_idx >= 0 {
            set(j, "object_idx", Value::from(self.object_idx));
        }
        if self.object_id != 0 {
            set(j, "object_id", Value::from(self.object_id as u64));
        }
    }

    /// AssemblyStepsJson.cpp:622-627
    pub fn from_json(&mut self, j: &Value) {
        self.base.from_json(j);
        json_get_int(j, "object_idx", &mut self.object_idx);
        json_get_size_t(j, "object_id", &mut self.object_id);
    }
}

// ---- AssembleSub ----
impl AssembleSub {
    /// AssemblyStepsJson.cpp:630-659
    pub fn to_json(&self, j: &mut Value) {
        self.base.to_json(&self.get_type(), j);
        if self.id >= 0 {
            set(j, "id", Value::from(self.id));
        }
        set(j, "step", Value::from(self.step));
        set(j, "is_final_assembly", Value::from(self.is_final_assembly));
        if self.inherited_from_step_id >= 0 {
            set(j, "inherited_from_step_id", Value::from(self.inherited_from_step_id));
        }
        if !self.inherited_object_ids.is_empty() {
            let mut arr: Vec<Value> = Vec::new();
            for oid in &self.inherited_object_ids {
                arr.push(Value::from(*oid as u64));
            }
            set(j, "inherited_object_ids", Value::Array(arr));
        }
        if let Some(assembly_tree_checked) = &self.assembly_tree_checked {
            let mut checked = Value::Object(Map::new());
            for item in assembly_tree_checked {
                set(&mut checked, item.0, Value::from(*item.1));
            }
            set(j, "assembly_tree_checked", checked);
        }

        let mut ch_arr: Vec<Value> = Vec::new();
        for child in &self.children {
            let mut ch_j = Value::Null;
            child.to_json(&mut ch_j);
            ch_arr.push(ch_j);
        }
        set(j, "children", Value::Array(ch_arr));
    }

    /// AssemblyStepsJson.cpp:661-695
    pub fn from_json(&mut self, j: &Value) {
        self.base.from_json(j);
        json_get_int(j, "id", &mut self.id);
        json_get_int(j, "step", &mut self.step);
        json_get_assembly_step_kind(j, "is_final_assembly", &mut self.is_final_assembly);
        self.inherited_from_step_id = -1;
        json_get_int(j, "inherited_from_step_id", &mut self.inherited_from_step_id);
        self.inherited_object_ids.clear();
        if contains(j, "inherited_object_ids") && j["inherited_object_ids"].is_array() {
            for v in elements(&j["inherited_object_ids"]) {
                if is_number_unsigned(v) {
                    self.inherited_object_ids.push(v.as_u64().unwrap_or(0) as usize);
                } else if is_number_integer(v) {
                    self.inherited_object_ids.push(get_long_long(v) as usize);
                }
            }
        }
        self.assembly_tree_checked = None;
        if contains(j, "assembly_tree_checked") && j["assembly_tree_checked"].is_object() {
            let mut checked: HashMap<String, bool> = HashMap::new();
            if let Some(object) = j["assembly_tree_checked"].as_object() {
                for (key, value) in object {
                    if value.is_boolean() {
                        checked.insert(key.clone(), value.as_bool().unwrap_or(false));
                    }
                }
            }
            self.assembly_tree_checked = Some(checked);
        }

        self.children.clear();
        if contains(j, "children") && j["children"].is_array() {
            for ch_j in elements(&j["children"]) {
                let child = AssembleBaseInfo::create_from_json(ch_j);
                if let Some(child) = child {
                    self.children.push(child);
                }
            }
        }
    }
}

// ---- AssemblyStepJson ----
impl AssemblyStepJson {
    /// AssemblyStepsJson.cpp:698-700
    pub fn get_debug_file_path() -> String {
        let mut path = PathBuf::from(crate::log_sink::data_dir());
        path.push("cache");
        path.push("assembly_step.json");
        path.to_string_lossy().into_owned()
    }

    /// AssemblyStepsJson.cpp:702-711
    fn load_pdf_export_params(&mut self, root: &Value) {
        self.m_pdf_export_params = PdfExportParams::default();
        if !contains(root, "pdf_export") || !root["pdf_export"].is_object() {
            return;
        }

        let pdf_export = &root["pdf_export"];
        if contains(pdf_export, "title") && pdf_export["title"].is_string() {
            self.m_pdf_export_params.title = pdf_export["title"].as_str().unwrap_or("").to_string();
        }
    }

    /// AssemblyStepsJson.cpp:713-719
    fn load_assembly_part_number_label_font_size(&mut self, root: &Value) {
        self.m_assembly_part_number_label_font_size = 0.0;
        let mut font_size = 0.0;
        if json_get_double(root, "assembly_part_number_label_font_size", &mut font_size) && font_size.is_finite() && font_size > 0.0 {
            self.m_assembly_part_number_label_font_size = font_size as f32;
        }
    }

    /// AssemblyStepsJson.cpp:721-727
    fn load_camera_ref_viewport(&mut self, root: &Value) {
        self.m_camera_ref_viewport_w = 0;
        self.m_camera_ref_viewport_h = 0;
        json_get_int(root, "camera_ref_viewport_w", &mut self.m_camera_ref_viewport_w);
        json_get_int(root, "camera_ref_viewport_h", &mut self.m_camera_ref_viewport_h);
    }

    /// AssemblyStepsJson.cpp:729-757
    pub fn load(&mut self, path: &str) -> bool {
        let bytes = match std::fs::read(path) {
            Ok(bytes) => bytes,
            Err(_) => {
                boost_log_trivial!(Warning, "AssemblyStepJson: cannot open ", path);
                return false;
            }
        };

        // ifs >> root;
        let root = match parse_stream(&bytes) {
            Ok(root) => root,
            Err(e) => {
                boost_log_trivial!(Error, "AssemblyStepJson: parse error: ", e.what());
                return false;
            }
        };

        self.m_items.clear();
        self.load_pdf_export_params(&root);
        self.load_assembly_part_number_label_font_size(&root);
        self.load_camera_ref_viewport(&root);
        if contains(&root, "items") && root["items"].is_array() {
            for item_j in elements(&root["items"]) {
                let ptr = AssembleBaseInfo::create_from_json(item_j);
                if let Some(ptr) = ptr {
                    self.m_items.push(ptr);
                }
            }
        }
        true
    }

    /// AssemblyStepsJson.cpp:759-779
    pub fn load_from_string(&mut self, json_str: &str) -> bool {
        let root = match parse(json_str) {
            Ok(root) => root,
            Err(e) => {
                boost_log_trivial!(Error, "AssemblyStepJson: parse from string error: ", e.what());
                return false;
            }
        };
        self.m_items.clear();
        self.load_pdf_export_params(&root);
        self.load_assembly_part_number_label_font_size(&root);
        self.load_camera_ref_viewport(&root);
        if contains(&root, "items") && root["items"].is_array() {
            for item_j in elements(&root["items"]) {
                let ptr = AssembleBaseInfo::create_from_json(item_j);
                if let Some(ptr) = ptr {
                    self.m_items.push(ptr);
                }
            }
        }
        true
    }

    /// AssemblyStepsJson.cpp:781-806
    ///
    /// The `catch` of the C++ is for `dump()` meeting a string that is not UTF-8
    /// (`type_error.316`), which a Rust `String` cannot be.
    pub fn to_json_string(&self) -> String {
        let mut root = Value::Null;
        // Current steps-file version. A loaded version above this is kept by the caller.
        set(&mut root, "version", json_float(1.2)); //V1.0 20220610 //V1.1 20220715
        set(&mut root, "assembly_part_number_label_font_size", json_float(self.m_assembly_part_number_label_font_size as f64));
        set(&mut root, "camera_ref_viewport_w", Value::from(self.m_camera_ref_viewport_w));
        set(&mut root, "camera_ref_viewport_h", Value::from(self.m_camera_ref_viewport_h));
        let mut pdf_export = Value::Null;
        set(&mut pdf_export, "title", Value::from(self.m_pdf_export_params.title.clone()));
        set(&mut root, "pdf_export", pdf_export);

        let mut items_arr: Vec<Value> = Vec::new();
        for item in &self.m_items {
            let mut item_j = Value::Null;
            item.to_json(&mut item_j);
            items_arr.push(item_j);
        }
        set(&mut root, "items", Value::Array(items_arr));
        dump(&root, 2)
    }

    /// AssemblyStepsJson.cpp:808-827
    ///
    /// `Err` is the `boost::filesystem::filesystem_error` that `create_directories` throws out of
    /// the C++ function, which does not catch it: for a path without a directory part
    /// (`create_directories("")` is `invalid_argument`) and for a directory that cannot be made.
    /// A write that fails after the file was opened is not noticed, as in the C++, where the
    /// stream does not throw.
    pub fn save(&self, path: &str) -> Result<bool, std::io::Error> {
        // boost::filesystem::path(path).parent_path(): for a path that ends in a separator it is the
        // path without it ("a/b/" -> "a/b"), where std::path::Path::parent() gives "a".
        let dir: PathBuf = if path.len() > 1 && path.ends_with(std::path::is_separator) {
            PathBuf::from(path.trim_end_matches(std::path::is_separator))
        } else {
            std::path::Path::new(path).parent().map(|p| p.to_path_buf()).unwrap_or_default()
        };
        if !dir.exists() {
            if dir.as_os_str().is_empty() {
                return Err(std::io::Error::new(std::io::ErrorKind::InvalidInput, "boost::filesystem::create_directories: Invalid argument"));
            }
            std::fs::create_dir_all(&dir)?;
        }

        let mut ofs = match std::fs::File::create(path) {
            Ok(ofs) => ofs,
            Err(_) => {
                boost_log_trivial!(Error, "AssemblyStepJson: cannot write ", path);
                return Ok(false);
            }
        };

        use std::io::Write;
        let _ = ofs.write_all(self.to_json_string().as_bytes());
        Ok(true)
    }
}

/// AssemblyStepsJson.cpp:829-860
pub fn assembly_steps_json_last_step_is_final_assembly(json_str: &str) -> bool {
    if json_str.is_empty() {
        return false;
    }
    let run = || -> Result<bool, JsonException> {
        let root = parse(json_str)?;
        if !contains(&root, "items") || !root["items"].is_array() {
            return Ok(false);
        }

        let mut last_step: Option<&Value> = None;
        for item in elements(&root["items"]) {
            if !item.is_object() {
                continue;
            }
            let mut r#type = String::new();
            if !json_get_string(item, "type", &mut r#type) || r#type != "sub" {
                continue;
            }
            let mut kind = AssemblyStepKind::Normal;
            json_get_assembly_step_kind(item, "is_final_assembly", &mut kind);
            if kind == AssemblyStepKind::OverallPreview {
                continue;
            }
            last_step = Some(item);
        }
        let last_step = match last_step {
            None => return Ok(false),
            Some(last_step) => last_step,
        };
        let mut kind = AssemblyStepKind::Normal;
        json_get_assembly_step_kind(last_step, "is_final_assembly", &mut kind);
        Ok(kind == AssemblyStepKind::FinalAssembly)
    };
    match run() {
        Ok(result) => result,
        Err(e) => {
            boost_log_trivial!(Warning, "AssemblyStepJson: last-step check failed: ", e.what());
            false
        }
    }
}

/// AssemblyStepsJson.cpp:863-886
fn collect_step_objects(node: &Value, idxs: &mut BTreeSet<i32>, ids: &mut BTreeSet<usize>) {
    if !node.is_object() {
        return;
    }
    let mut idx = -1;
    if json_get_int(node, "object_idx", &mut idx) && idx >= 0 {
        idxs.insert(idx);
    }
    let mut id: usize = 0;
    if json_get_size_t(node, "object_id", &mut id) && id != 0 {
        ids.insert(id);
    }
    if contains(node, "inherited_object_ids") && node["inherited_object_ids"].is_array() {
        for v in elements(&node["inherited_object_ids"]) {
            if !is_number_integer(v) {
                continue;
            }
            let raw = get_long_long(v);
            if raw > 0 {
                ids.insert(raw as usize);
            }
        }
    }
    if !contains(node, "children") || !node["children"].is_array() {
        return;
    }
    for child in elements(&node["children"]) {
        collect_step_objects(child, idxs, ids);
    }
}

/// AssemblyStepsJson.cpp:888-901
fn find_step_by_id(items: &Value, step_id: i32) -> Option<&Value> {
    for item in elements(items) {
        if !item.is_object() {
            continue;
        }
        let mut r#type = String::new();
        if !json_get_string(item, "type", &mut r#type) || r#type != "sub" {
            continue;
        }
        let mut id = -1;
        if json_get_int(item, "id", &mut id) && id == step_id {
            return Some(item);
        }
    }
    None
}

/// AssemblyStepsJson.cpp:904-991
pub fn assembly_steps_json_last_step_covers_model(json_str: &str, model: &Model, assembly_model_json: &str) -> bool {
    if json_str.is_empty() || model.objects.is_empty() {
        return false;
    }
    let run = || -> Result<bool, JsonException> {
        let root = parse(json_str)?;
        if !contains(&root, "items") || !root["items"].is_array() {
            return Ok(false);
        }

        let mut last_step: Option<&Value> = None;
        for item in elements(&root["items"]) {
            if !item.is_object() {
                continue;
            }
            let mut r#type = String::new();
            if !json_get_string(item, "type", &mut r#type) || r#type != "sub" {
                continue;
            }
            let mut kind = AssemblyStepKind::Normal;
            json_get_assembly_step_kind(item, "is_final_assembly", &mut kind);
            if kind == AssemblyStepKind::OverallPreview {
                continue;
            }
            last_step = Some(item);
        }
        if last_step.is_none() {
            return Ok(false);
        }

        // Restore gives each assembly object a new id. The id written in assembly_model.json
        // is the one the steps file stored.
        let mut persisted_ids: Vec<usize> = Vec::new();
        if !assembly_model_json.is_empty() {
            let aroot = parse(assembly_model_json)?;
            if contains(&aroot, "nodes") && aroot["nodes"].is_array() {
                for node in elements(&aroot["nodes"]) {
                    let mut id: usize = 0;
                    if node.is_object() {
                        json_get_size_t(node, "id", &mut id);
                    }
                    persisted_ids.push(id);
                }
            }
        }

        let mut idxs: BTreeSet<i32> = BTreeSet::new();
        let mut ids: BTreeSet<usize> = BTreeSet::new();
        let mut seen_ids: BTreeSet<i32> = BTreeSet::new();
        let mut step: Option<&Value> = last_step;
        while let Some(current) = step {
            collect_step_objects(current, &mut idxs, &mut ids);
            let mut self_id = -1;
            json_get_int(current, "id", &mut self_id);
            if self_id >= 0 && !seen_ids.insert(self_id) {
                break;
            }
            let mut parent_id = -1;
            if !json_get_int(current, "inherited_from_step_id", &mut parent_id) || parent_id < 0 {
                break;
            }
            if !seen_ids.insert(parent_id) {
                break;
            }
            step = find_step_by_id(&root["items"], parent_id);
        }

        let mut expected = 0;
        let mut covered = 0;
        for oi in 0..model.objects.len() as i32 {
            let obj = match &model.objects[oi as usize] {
                None => continue,
                Some(obj) => obj,
            };
            let mut has_part = false;
            for mv in &obj.volumes {
                if let Some(mv) = mv {
                    if mv.is_model_part() {
                        has_part = true;
                        break;
                    }
                }
            }
            if !has_part {
                continue;
            }
            expected += 1;
            let persisted_id: usize = if (oi as usize) < persisted_ids.len() { persisted_ids[oi as usize] } else { 0 };
            let by_idx = idxs.contains(&oi);
            let by_id = (obj.id().id != 0 && ids.contains(&obj.id().id)) || (persisted_id != 0 && ids.contains(&persisted_id));
            if by_idx || by_id {
                covered += 1;
            }
        }
        Ok(expected > 0 && covered == expected)
    };
    match run() {
        Ok(result) => result,
        Err(e) => {
            boost_log_trivial!(Warning, "AssemblyStepJson: last-step model coverage check failed: ", e.what());
            false
        }
    }
}

/// AssemblyStepsJson.cpp:993-1028
pub fn assembly_steps_json_mark_last_step_final_assembly(json_str: &str) -> String {
    if json_str.is_empty() {
        return json_str.to_string();
    }
    let run = || -> Result<String, JsonException> {
        let mut root = parse(json_str)?;
        if !contains(&root, "items") || !root["items"].is_array() {
            return Ok(json_str.to_string());
        }

        // nlohmann::json *last_step: the place of the item in root["items"].
        let mut last_step: Option<usize> = None;
        for (item_idx, item) in elements(&root["items"]).iter().enumerate() {
            if !item.is_object() {
                continue;
            }
            let mut r#type = String::new();
            if !json_get_string(item, "type", &mut r#type) || r#type != "sub" {
                continue;
            }
            let mut kind = AssemblyStepKind::Normal;
            json_get_assembly_step_kind(item, "is_final_assembly", &mut kind);
            if kind == AssemblyStepKind::OverallPreview {
                continue;
            }
            last_step = Some(item_idx);
        }
        let last_step = match last_step {
            None => return Ok(json_str.to_string()),
            Some(last_step) => last_step,
        };

        let mut kind = AssemblyStepKind::Normal;
        json_get_assembly_step_kind(&root["items"][last_step], "is_final_assembly", &mut kind);
        if kind == AssemblyStepKind::FinalAssembly {
            return Ok(json_str.to_string());
        }
        try_set(&mut root["items"][last_step], "is_final_assembly", Value::from(AssemblyStepKind::FinalAssembly))?;
        Ok(dump(&root, 2))
    };
    match run() {
        Ok(result) => result,
        Err(e) => {
            boost_log_trivial!(Warning, "AssemblyStepJson: mark last step final assembly failed: ", e.what());
            json_str.to_string()
        }
    }
}

/// AssemblyStepsJson.cpp:1030-1051
pub fn assembly_steps_json_with_qualified_assembly(json_str: &str, qualified: bool) -> String {
    if json_str.is_empty() {
        return json_str.to_string();
    }
    let run = || -> Result<String, JsonException> {
        let mut root = parse(json_str)?;
        let has_flag = contains(&root, K_ASSEMBLY_STEPS_QUALIFIED_KEY);
        if qualified {
            if has_flag && root[K_ASSEMBLY_STEPS_QUALIFIED_KEY].is_boolean() && root[K_ASSEMBLY_STEPS_QUALIFIED_KEY].as_bool().unwrap_or(false) {
                return Ok(json_str.to_string());
            }
            try_set(&mut root, K_ASSEMBLY_STEPS_QUALIFIED_KEY, Value::from(true))?;
        } else {
            if !has_flag {
                return Ok(json_str.to_string());
            }
            if let Some(object) = root.as_object_mut() {
                object.remove(K_ASSEMBLY_STEPS_QUALIFIED_KEY);
            }
        }
        Ok(dump(&root, 2))
    };
    match run() {
        Ok(result) => result,
        Err(e) => {
            boost_log_trivial!(Warning, "AssemblyStepJson: qualified_assembly update failed: ", e.what());
            json_str.to_string()
        }
    }
}

/// AssemblyStepsJson.cpp:1053-1059
pub fn qualified_assembly_result_json(qualified: bool, need_rewrite_3mf: bool) -> String {
    let mut root = Value::Null;
    set(&mut root, K_QUALIFIED_ASSEMBLY_RESULT_KEY, Value::from(if qualified { "true" } else { "false" }));
    set(&mut root, K_NEED_REWRITE_3MF_RESULT_KEY, Value::from(if need_rewrite_3mf { "true" } else { "false" }));
    dump(&root, 2)
}

/// AssemblyStepsJson.cpp:1061-1082
pub fn assembly_steps_json_file_meta(json_str: &str) -> AssemblyStepsFileMeta {
    let mut meta = AssemblyStepsFileMeta::default();
    if json_str.is_empty() {
        return meta;
    }
    match parse(json_str) {
        Ok(root) => {
            if contains(&root, "version") && root["version"].is_number() {
                meta.version = get_double(&root["version"]);
            }
            if contains(&root, K_ASSEMBLY_STEPS_QUALIFIED_KEY) {
                meta.has_qualified = true;
                let value = &root[K_ASSEMBLY_STEPS_QUALIFIED_KEY];
                if value.is_boolean() {
                    meta.qualified = value.as_bool().unwrap_or(false);
                } else if value.is_string() {
                    meta.qualified = value.as_str().unwrap_or("") == "true";
                }
            }
        }
        Err(e) => {
            boost_log_trivial!(Warning, "AssemblyStepJson: file meta parse failed: ", e.what());
        }
    }
    meta
}

/// AssemblyStepsJson.cpp:1084-1099
pub fn assembly_steps_json_with_version(json_str: &str, version: f64) -> String {
    if json_str.is_empty() {
        return json_str.to_string();
    }
    let run = || -> Result<String, JsonException> {
        let mut root = parse(json_str)?;
        if contains(&root, "version") && root["version"].is_number() && (get_double(&root["version"]) - version).abs() < 1e-6 {
            return Ok(json_str.to_string());
        }
        try_set(&mut root, "version", json_float(version))?;
        Ok(dump(&root, 2))
    };
    match run() {
        Ok(result) => result,
        Err(e) => {
            boost_log_trivial!(Warning, "AssemblyStepJson: version update failed: ", e.what());
            json_str.to_string()
        }
    }
}

impl AssemblyStepsTreeData {
    /// AssemblyStepsJson.cpp:1101-1152
    pub fn to_json_string(&self) -> String {
        // AssemblyStepsJson.cpp:1103-1135, the lambda `build_item`
        fn build_item(nodes: &[AssemblyStepsTreeNode], node_idx: i32) -> Option<Arc<AssembleItem>> {
            if node_idx < 0 || node_idx >= nodes.len() as i32 {
                return None;
            }
            let node = &nodes[node_idx as usize];

            if node.r#type == AssemblyStepsTreeNodeType::Folder {
                let mut sub = AssembleSub::default();
                sub.base.name = node.name.clone();
                sub.is_final_assembly = node.is_final_assembly;
                sub.id = node.id;
                sub.step = node.step;
                sub.inherited_from_step_id = node.inherited_from_step_id;
                sub.inherited_object_ids = node.inherited_object_ids.clone();
                sub.assembly_tree_checked = node.assembly_tree_checked.clone();
                // Mirror legacy AssemblyStepsUtils::build_steps_json_string — all entries
                for kf in &node.kf_data.entries {
                    sub.base.keyframes.push(kf.data.clone());
                }
                for child_idx in &node.children {
                    if let Some(child_item) = build_item(nodes, *child_idx) {
                        sub.children.push(child_item);
                    }
                }
                return Some(Arc::new(AssembleItem::Sub(sub)));
            }

            let mut single = AssembleSingleInfo::default();
            single.base.name = node.name.clone();
            single.object_idx = node.object_idx;
            single.object_id = node.object_id;
            for kf in &node.kf_data.entries {
                single.base.keyframes.push(kf.data.clone());
            }
            Some(Arc::new(AssembleItem::Single(single)))
        }

        let mut doc = AssemblyStepJson::default();
        let mut items: Vec<Arc<AssembleItem>> = Vec::new();
        items.reserve(self.roots.len());
        for root_idx in &self.roots {
            let root_idx = *root_idx;
            // OverallPreview is a runtime-only UI sentinel and must not be persisted.
            if root_idx >= 0
                && root_idx < self.nodes.len() as i32
                && self.nodes[root_idx as usize].r#type == AssemblyStepsTreeNodeType::Folder
                && self.nodes[root_idx as usize].is_final_assembly == AssemblyStepKind::OverallPreview
            {
                continue;
            }
            if let Some(item) = build_item(&self.nodes, root_idx) {
                items.push(item);
            }
        }
        doc.set_items(items);
        doc.set_camera_ref_viewport(self.camera_ref_viewport_w, self.camera_ref_viewport_h);
        doc.to_json_string()
    }

    // ---- AssemblyStepsTreeData::from_json_string ----
    /// AssemblyStepsJson.cpp:1155-1405
    ///
    /// The `catch` of the C++ has nothing of this unit to catch (`load_from_string` catches its
    /// own); it is there for the standard library.
    pub fn from_json_string(
        json_str: &str,
        tree: &mut AssemblyStepsTreeData,
        model: &Model,
        error: Option<&mut String>,
        assembly_part_number_label_font_size: Option<&mut f32>,
    ) -> bool {
        if json_str.is_empty() {
            if let Some(error) = error {
                *error = "empty JSON string".to_string();
            }
            return false;
        }

        let mut json_doc = AssemblyStepJson::default();
        if !json_doc.load_from_string(json_str) {
            if let Some(error) = error {
                *error = "AssemblyStepJson::load_from_string failed".to_string();
            }
            return false;
        }
        if let Some(assembly_part_number_label_font_size) = assembly_part_number_label_font_size {
            *assembly_part_number_label_font_size = json_doc.get_assembly_part_number_label_font_size();
        }

        let mut parsed = AssemblyStepsTreeData::default();
        let object_count = model.objects.len() as i32;

        // AssemblyStepsJson.cpp:1180-1190, the lambda `count_item_nodes`
        fn count_item_nodes(item: &Arc<AssembleItem>) -> usize {
            let mut count: usize = 1;
            if let Some(sub) = item.as_sub() {
                for child in &sub.children {
                    count += count_item_nodes(child);
                }
            }
            count
        }
        let mut expected_nodes: usize = 0;
        for item in json_doc.get_items() {
            expected_nodes += count_item_nodes(item);
        }
        parsed.nodes.reserve(expected_nodes);
        parsed.roots.reserve(json_doc.get_items().len());

        // AssemblyStepsJson.cpp:1197-1218, the lambda `resolve_object_idx`
        // Resolve stable object_id -> current object_idx. The fallback chain mirrors
        fn resolve_object_idx(model: &Model, object_count: i32, hint_idx: i32, object_id: usize, name: &str) -> i32 {
            if object_id != 0 {
                for i in 0..object_count {
                    if let Some(obj) = &model.objects[i as usize] {
                        if obj.id().id == object_id {
                            return i;
                        }
                    }
                }
                // Intentional fall-through to hint_idx / name match below.
            }
            if hint_idx >= 0 && hint_idx < object_count {
                if let Some(obj) = &model.objects[hint_idx as usize] {
                    if obj.name == name {
                        return hint_idx;
                    }
                }
            }
            for i in 0..object_count {
                if let Some(obj) = &model.objects[i as usize] {
                    if obj.name == name {
                        return i;
                    }
                }
            }
            -1
        }

        // AssemblyStepsJson.cpp:1220-1226, the lambda `next_folder_id`
        fn next_folder_id(parsed: &AssemblyStepsTreeData) -> i32 {
            let mut max_id = 0;
            for n in &parsed.nodes {
                if n.r#type == AssemblyStepsTreeNodeType::Folder {
                    max_id = std::cmp::max(max_id, n.id);
                }
            }
            max_id + 1
        }

        // AssemblyStepsJson.cpp:1228-1247, the lambda `create_object_node_for_item`
        fn create_object_node_for_item(
            parsed: &mut AssemblyStepsTreeData,
            model: &Model,
            object_count: i32,
            hint_idx: i32,
            object_id: usize,
            name: &str,
        ) -> i32 {
            let resolved = resolve_object_idx(model, object_count, hint_idx, object_id, name);
            if resolved < 0 {
                return -1;
            }
            // Always create a new node — the same ModelObject may appear in
            let idx = parsed.nodes.len() as i32;
            parsed.nodes.push(AssemblyStepsTreeNode::default());
            let obj_node = parsed.nodes.last_mut().expect("the node just added");
            obj_node.r#type = AssemblyStepsTreeNodeType::Object;
            obj_node.object_idx = resolved;
            let mo = model.objects[resolved as usize].as_ref();
            obj_node.object_id = match mo {
                Some(mo) => mo.id().id,
                None => 0,
            };
            obj_node.name = match mo {
                Some(mo) if name.is_empty() => mo.name.clone(),
                _ => name.to_string(),
            };
            obj_node.visible = true;
            obj_node.kf_data.node_idx = idx;
            obj_node.kf_data.object_idx = obj_node.object_idx;
            obj_node.kf_data.is_folder = false;

            idx
        }

        // AssemblyStepsJson.cpp:1264-1277, the lambda `remap_object_idx_in_pair_map`
        fn remap_object_idx_in_pair_map<V>(need_remap: bool, from_obj: i32, to_obj: i32, pair_map: &mut BTreeMap<(i32, i32), V>) {
            if !need_remap {
                return;
            }
            let mut remapped: BTreeMap<(i32, i32), V> = BTreeMap::new();
            for (key, value) in std::mem::take(pair_map) {
                // emplace: the first one to take a key keeps it.
                if key.0 == from_obj {
                    remapped.entry((to_obj, key.1)).or_insert(value);
                } else {
                    remapped.entry(key).or_insert(value);
                }
            }
            *pair_map = remapped;
        }

        // AssemblyStepsJson.cpp:1278-1292, the lambda `remap_object_idx_in_int_map`
        // object_transformations is keyed by a single int (object_idx) so the pair-map helper above does not apply; do the equivalent remap here.
        fn remap_object_idx_in_int_map<V>(need_remap: bool, from_obj: i32, to_obj: i32, int_map: &mut BTreeMap<i32, V>) {
            if !need_remap {
                return;
            }
            let mut remapped: BTreeMap<i32, V> = BTreeMap::new();
            for (key, value) in std::mem::take(int_map) {
                if key == from_obj {
                    remapped.entry(to_obj).or_insert(value);
                } else {
                    remapped.entry(key).or_insert(value);
                }
            }
            *int_map = remapped;
        }

        // AssemblyStepsJson.cpp:1248-1304, the lambda `load_node_keyframes`
        // Copy JSON-side AssembleBaseInfo keyframes into a node's kf_data. For Object
        fn load_node_keyframes(parsed: &mut AssemblyStepsTreeData, node_idx: i32, info: &AssembleItem, is_folder: bool) {
            if node_idx < 0 || node_idx >= parsed.nodes.len() as i32 {
                return;
            }
            let node = &mut parsed.nodes[node_idx as usize];
            if info.base().keyframes.is_empty() {
                return;
            }

            let single: Option<&AssembleSingleInfo> = if is_folder { None } else { info.as_single() };
            let need_remap = match single {
                Some(single) => single.object_idx >= 0 && single.object_idx != node.object_idx,
                None => false,
            };

            let from_obj = match single {
                Some(single) => single.object_idx,
                None => -1,
            };
            let to_obj = node.object_idx;

            node.kf_data.entries.reserve(info.base().keyframes.len());
            for kf in &info.base().keyframes {
                node.kf_data.entries.push(KeyFrameEntry::default());
                let entry = node.kf_data.entries.last_mut().expect("the entry just added");
                entry.need_save = true;
                entry.data = kf.clone();
                remap_object_idx_in_int_map(need_remap, from_obj, to_obj, &mut entry.data.object_transformations);
                remap_object_idx_in_pair_map(need_remap, from_obj, to_obj, &mut entry.data.volume_transformations);
                remap_object_idx_in_pair_map(need_remap, from_obj, to_obj, &mut entry.data.volume_names);
            }
        }

        // AssemblyStepsJson.cpp:1306-1354, the lambda `restore_item`
        fn restore_item(parsed: &mut AssemblyStepsTreeData, model: &Model, object_count: i32, item: &Arc<AssembleItem>) -> i32 {
            if item.get_type() == "sub" {
                let sub = match item.as_sub() {
                    None => return -1,
                    Some(sub) => sub,
                };

                let folder_idx = parsed.nodes.len() as i32;
                parsed.nodes.push(AssemblyStepsTreeNode::default());
                parsed.nodes[folder_idx as usize].r#type = AssemblyStepsTreeNodeType::Folder;
                let id = if sub.id >= 0 { sub.id } else { next_folder_id(parsed) };
                let folder = &mut parsed.nodes[folder_idx as usize];
                folder.id = id;
                folder.step = sub.step;
                folder.name = sub.base.name.clone();
                folder.is_final_assembly = sub.is_final_assembly;
                folder.inherited_from_step_id = sub.inherited_from_step_id;
                folder.inherited_object_ids = sub.inherited_object_ids.clone();
                folder.assembly_tree_checked = sub.assembly_tree_checked.clone();
                folder.kf_data.node_idx = folder_idx;
                folder.kf_data.object_idx = folder.object_idx;
                folder.kf_data.is_folder = true;
                folder.children.reserve(sub.children.len());

                load_node_keyframes(parsed, folder_idx, item, /*is_folder=*/ true);

                for child in &sub.children {
                    let child_idx = restore_item(parsed, model, object_count, child);
                    if child_idx >= 0 {
                        parsed.nodes[folder_idx as usize].children.push(child_idx);
                    }
                }
                return folder_idx;
            }

            if item.get_type() == "single" {
                let single = match item.as_single() {
                    None => return -1,
                    Some(single) => single,
                };
                let node_idx = create_object_node_for_item(parsed, model, object_count, single.object_idx, single.object_id, &item.base().name);
                if node_idx >= 0 {
                    load_node_keyframes(parsed, node_idx, item, /*is_folder=*/ false);
                }
                return node_idx;
            }

            -1
        }

        for item in json_doc.get_items() {
            // OverallPreview is runtime UI-only; drop it if an older file still contains it.
            if let Some(sub) = item.as_sub() {
                if sub.is_final_assembly == AssemblyStepKind::OverallPreview {
                    continue;
                }
            }
            let idx = restore_item(&mut parsed, model, object_count, item);
            if idx >= 0 {
                parsed.roots.push(idx);
            }
        }

        // One-line summary so 3MF reopen empty-folder regressions show up in the
        let mut total_children: usize = 0;
        for root_idx in &parsed.roots {
            if *root_idx >= 0 && *root_idx < parsed.nodes.len() as i32 {
                total_children += parsed.nodes[*root_idx as usize].children.len();
            }
        }
        boost_log_trivial!(Info, "loaded nodes=", parsed.nodes.len(), " roots=", parsed.roots.len(), " total_root_children=", total_children);

        // Document-level shared reference viewport (single value for all keyframes).
        parsed.camera_ref_viewport_w = json_doc.get_camera_ref_viewport_w();
        parsed.camera_ref_viewport_h = json_doc.get_camera_ref_viewport_h();

        // Capture the load-time model snapshot as the baseline for the assembly view.
        // The 3mf was saved with model and steps data consistent, so this snapshot
        // represents "the model state the loaded steps data matches". Runtime-only.
        parsed.loaded_recorded_volumes.clear();
        for oi in 0..object_count {
            let obj = match &model.objects[oi as usize] {
                None => continue,
                Some(obj) => obj,
            };
            for vi in 0..obj.volumes.len() as i32 {
                if let Some(volume) = &obj.volumes[vi as usize] {
                    if volume.is_model_part() {
                        parsed.loaded_recorded_volumes.insert(volume.ensure_part_guid(false));
                    }
                }
            }
        }
        // As long as it's read from JSON, it's definitely a new project.
        parsed.has_loaded_recorded_baseline = true;

        *tree = parsed;
        true
    }
}

// ---- Independent assembly model (object graph) JSON ----

/// AssemblyStepsJson.cpp:1409-1416
fn matrix4_to_json(arr: &mut Value, t: &Transform3d) {
    let mut out: Vec<Value> = Vec::new();
    let m = t;
    for r in 0..4 {
        for c in 0..4 {
            out.push(json_float(m[(r, c)]));
        }
    }
    *arr = Value::Array(out);
}

/// AssemblyStepsJson.cpp:1417-1428. `get<double>()` throws for a cell that is not a number.
fn json_to_matrix4(j: &Value, key: &str, out: &mut Transform3d) -> Result<bool, JsonException> {
    if !contains(j, key) || !j[key].is_array() || size(&j[key]) != 16 {
        return Ok(false);
    }
    let mut t = Transform3d::identity();
    let mut idx = 0;
    for r in 0..4 {
        for c in 0..4 {
            t[(r, c)] = try_get_double(&j[key][idx])?;
            idx += 1;
        }
    }
    *out = t;
    Ok(true)
}

/// AssemblyStepsJson.cpp:1431-1483
pub fn assembly_model_to_json_string(assembly_model: &Model) -> String {
    let mut root = Value::Null;
    set(&mut root, "version", Value::from(1));
    let mut jobjects: Vec<Value> = Vec::new();
    for mo in &assembly_model.objects {
        let mo = match mo {
            None => continue,
            Some(mo) => mo,
        };
        let mut jobj = Value::Null;
        set(&mut jobj, "id", Value::from(mo.id().id as i32));
        set(&mut jobj, "type", Value::from("group"));
        set(&mut jobj, "name", Value::from(mo.name.clone()));
        let mut jvols: Vec<Value> = Vec::new();
        for mv in &mo.volumes {
            let mv = match mv {
                None => continue,
                Some(mv) => mv,
            };
            let mut jv = Value::Null;
            set(&mut jv, "id", Value::from(mv.id().id as i32));
            set(&mut jv, "type", Value::from("leaf"));
            set(&mut jv, "volume", Value::from(mv.assembly_src_guid().to_string()));
            set(&mut jv, "name", Value::from(mv.name.clone()));
            // keep the assemble_transform which defines the part's pose in the assembly coordinate system.
            let mut assemble_transform = Value::Null;
            matrix4_to_json(&mut assemble_transform, &mv.get_assemble_transformation().matrix());
            set(&mut jv, "assemble_transform", assemble_transform);
            // The local transform ("transform") and type/uuid are omitted because they are not required in the assembly view.
            jvols.push(jv);
        }
        set(&mut jobj, "nodes", Value::Array(jvols));

        if mo.instances.len() == 1 && mo.instances[0].is_some() {
            let mi = mo.instances[0].as_ref().expect("checked above");
            // Only the assemble_transform is needed for the assembly model.
            let mut assemble_transform = Value::Null;
            matrix4_to_json(&mut assemble_transform, &mi.get_assemble_transformation().matrix());
            set(&mut jobj, "assemble_transform", assemble_transform);
            let off = mi.get_offset_to_assembly();
            set(&mut jobj, "offset_to_assembly", json_floats(&[off.x, off.y, off.z]));
        } else {
            let mut jinsts: Vec<Value> = Vec::new();
            for mi in &mo.instances {
                let mi = match mi {
                    None => continue,
                    Some(mi) => mi,
                };
                let mut ji = Value::Null;
                // Only the assemble_transform is needed for the assembly model.
                let mut assemble_transform = Value::Null;
                matrix4_to_json(&mut assemble_transform, &mi.get_assemble_transformation().matrix());
                set(&mut ji, "assemble_transform", assemble_transform);
                let off = mi.get_offset_to_assembly();
                set(&mut ji, "offset_to_assembly", json_floats(&[off.x, off.y, off.z]));
                jinsts.push(ji);
            }
            set(&mut jobj, "instances", Value::Array(jinsts));
        }
        jobjects.push(jobj);
    }
    set(&mut root, "nodes", Value::Array(jobjects));
    dump(&root, 2)
}

/// AssemblyStepsJson.cpp:1485-1579
///
/// `Err` is the `nlohmann::json::type_error` the C++ function lets out: only the parse is inside
/// a `try`, and `get<double>()` on a cell of `assemble_transform` or `offset_to_assembly` that is
/// not a number throws. The objects restored before it stay in `out_assembly_model`, as in C++.
pub fn assembly_model_from_json_string(
    json_str: &str,
    out_assembly_model: &mut Model,
    mesh_source: &Model,
    error: Option<&mut String>,
) -> Result<bool, JsonException> {
    out_assembly_model.clear_objects();
    if json_str.is_empty() {
        if let Some(error) = error {
            *error = "empty assembly model json".to_string();
        }
        return Ok(false);
    }

    let root = match parse(json_str) {
        Ok(root) => root,
        Err(e) => {
            if let Some(error) = error {
                *error = e.what().to_string();
            }
            return Ok(false);
        }
    };
    let mut version = 0;
    json_get_int(&root, "version", &mut version);

    if !contains(&root, "nodes") || !root["nodes"].is_array() {
        if let Some(error) = error {
            *error = "missing objects array".to_string();
        }
        return Ok(false);
    }

    // Index prepare-side parts by part_guid so each assembly volume can rebind its mesh shared_ptr.
    // The object comes with the volume: ModelVolume::extruder_id() asks it.
    let mut src_by_guid: HashMap<String, (&ModelObject, &ModelVolume)> = HashMap::new();
    for mo in &mesh_source.objects {
        let mo = mo.as_ref().expect("the C++ dereferences every object of mesh_source");
        for mv in &mo.volumes {
            if let Some(mv) = mv {
                if !mv.part_guid().is_empty() {
                    src_by_guid.entry(mv.part_guid()).or_insert((mo, mv));
                }
            }
        }
    }

    for jobj in elements(&root["nodes"]) {
        let ao = out_assembly_model.add_object();
        if contains(jobj, "name") && jobj["name"].is_string() {
            ao.name = jobj["name"].as_str().unwrap_or("").to_string();
        }

        if contains(jobj, "nodes") && jobj["nodes"].is_array() {
            for jv in elements(&jobj["nodes"]) {
                let mut src_guid = String::new();
                // The source GUID is now stored under the alias "volume".
                json_get_string(jv, "volume", &mut src_guid);
                let it = if src_guid.is_empty() { None } else { src_by_guid.get(&src_guid) };
                let (src_object, src_volume) = match it {
                    None => continue, // prepare-side part gone -> drop this volume (delete already propagated)
                    Some(it) => *it,
                };

                // Clone shares the mesh shared_ptr and copies config/type; then override identity + poses.
                let av = ao.add_volume(src_volume);
                // Preserve extruder configuration as before.
                let vol_extruder_opt = av.config_extruder;
                if vol_extruder_opt.is_none() || vol_extruder_opt == Some(0) {
                    av.config_extruder = Some(src_volume.extruder_id(src_object));
                }
                // The part GUID is no longer stored separately; it will be derived from the source volume when needed.
                av.set_assembly_src_guid(&src_guid);
                // If the part GUID is missing (new format), fall back to the assembly source GUID.
                if av.part_guid().is_empty() {
                    av.set_part_guid(&src_guid);
                }
                if contains(jv, "name") && jv["name"].is_string() {
                    av.name = jv["name"].as_str().unwrap_or("").to_string();
                }
                // The type, local transform and explicit UUID are omitted in the new JSON format.
                let mut t = Transform3d::identity();
                if json_to_matrix4(jv, "assemble_transform", &mut t)? {
                    av.set_assemble_from_transform(&t);
                }
            }
        }

        if contains(jobj, "instances") && jobj["instances"].is_array() {
            for ji in elements(&jobj["instances"]) {
                let ai = ao.add_instance();
                let mut t = Transform3d::identity();
                // The local transform is not needed for the assembly model.
                if json_to_matrix4(ji, "assemble_transform", &mut t)? {
                    ai.set_assemble_from_transform(&t);
                }
                if contains(ji, "offset_to_assembly") && ji["offset_to_assembly"].is_array() && size(&ji["offset_to_assembly"]) == 3 {
                    let x = try_get_double(&ji["offset_to_assembly"][0])?;
                    let y = try_get_double(&ji["offset_to_assembly"][1])?;
                    let z = try_get_double(&ji["offset_to_assembly"][2])?;
                    ai.set_offset_to_assembly(Vec3d::new(x, y, z));
                }
            }
        } else if contains(jobj, "assemble_transform") || contains(jobj, "offset_to_assembly") {
            let ai = ao.add_instance();
            let mut t = Transform3d::identity();
            if json_to_matrix4(jobj, "assemble_transform", &mut t)? {
                ai.set_assemble_from_transform(&t);
            }
            if contains(jobj, "offset_to_assembly") && jobj["offset_to_assembly"].is_array() && size(&jobj["offset_to_assembly"]) == 3 {
                let x = try_get_double(&jobj["offset_to_assembly"][0])?;
                let y = try_get_double(&jobj["offset_to_assembly"][1])?;
                let z = try_get_double(&jobj["offset_to_assembly"][2])?;
                ai.set_offset_to_assembly(Vec3d::new(x, y, z));
            }
        }
        // Every renderable object needs at least one instance.
        if ao.instances.is_empty() {
            ao.add_instance();
        }
    }
    Ok(true)
}
