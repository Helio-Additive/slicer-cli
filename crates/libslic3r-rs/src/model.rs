//! Model representation for 3D printing.
//!
//! This module provides Model and related types for representing 3D models
//! with multiple objects, instances, and metadata. Mirrors BambuStudio's Model.cpp.
//!
//! PORTING STATUS (Model.cpp / Model.hpp):
//! ----------------------------------------
//! The full C++ `Model` / `ModelObject` / `ModelVolume` / `ModelInstance` class
//! hierarchy is deeply coupled to infrastructure that is not yet ported into this
//! crate (the `ObjectBase` unique-ID system, `ModelConfigObject` wrapping
//! `DynamicPrintConfig`, per-volume `Geometry::Transformation`, cereal
//! serialization, the native 3MF/STEP/AMF loaders via boost/Eigen/OpenCASCADE,
//! `MeshBoolean`, `TriangleSelector`, `BuildVolume`, and `ModelArrange`).
//!
//! In particular, C++ `ModelObject` owns a `ModelVolumePtrs volumes` collection,
//! whereas the existing Rust `ModelObject` (used by the format loaders in this
//! crate: `threemf.rs`, `format/{stl,obj,amf,svg,step}.rs`, `obj.rs`,
//! `slicing_adaptive.rs`) stores a single merged `mesh: TriangleMesh`. Reworking
//! that into the C++ `volumes` model is a coordinated cross-module change that
//! must not be done piecemeal without breaking those consumers.
//!
//! This file therefore ports faithfully (1:1, line-referenced) the standalone,
//! consumer-safe pieces of Model.hpp / Model.cpp — the enums, plain-old-data
//! structs, constants, the BBS speed/extruder tables, and the pure
//! string<->type conversion functions — and retains the pre-existing simplified
//! domain `Model`/`ModelObject`/`Instance` shim that the loaders depend on. See
//! the "blocked" list in PORT_LEDGER / report for the symbols that require the
//! infrastructure above.

use crate::geometry::{BoundingBox3F, Point3F, Transform3D};
use crate::triangle_mesh::TriangleMesh;
use crate::Polygon;
use std::path::PathBuf;

// BBS initialization of static variables / const filament table.
// Model.cpp:50-53
//   const std::vector<std::string> CONST_FILAMENTS = { ... };
pub const CONST_FILAMENTS: [&str; 33] = [
    // Model.cpp:51
    "", "4", "8", "0C", "1C", "2C", "3C", "4C", "5C", "6C", "7C", "8C", "9C", "AC", "BC", "CC",
    "DC", // 16
    // Model.cpp:52
    "EC", "0FC", "1FC", "2FC", "3FC", "4FC", "5FC", "6FC", "7FC", "8FC", "9FC", "AFC", "BFC",
    "CFC", "DFC", "EFC", // 32
]; //      1                         5                                 10                                 15    16

// Model.hpp:241-244
//   enum class CutMode : int { cutPlanar, cutTongueAndGroove };
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum CutMode {
    CutPlanar,
    CutTongueAndGroove,
}

// Model.hpp:246-252
//   enum class CutConnectorType : int { Plug, Dowel, Snap, Thread, Undef };
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
#[repr(i32)]
pub enum CutConnectorType {
    Plug,
    Dowel,
    Snap,
    Thread,
    Undef,
}

// Model.hpp:254-259
//   enum class CutConnectorStyle : int { Prizm, Frustum, Undef //,Claw };
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
#[repr(i32)]
pub enum CutConnectorStyle {
    Prizm,
    Frustum,
    Undef,
    //,Claw
}

// Model.hpp:261-268
//   enum class CutConnectorShape : int { Triangle, Square, Hexagon, Circle, Undef //,D-shape };
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
#[repr(i32)]
pub enum CutConnectorShape {
    Triangle,
    Square,
    Hexagon,
    Circle,
    Undef,
    //,D-shape
}

// Model.hpp:269-273
//   struct CutConnectorParas { float snap_space_proportion{0.3}; float snap_bulge_proportion{0.15}; };
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CutConnectorParas {
    /// Model.hpp:271
    pub snap_space_proportion: f32,
    /// Model.hpp:272
    pub snap_bulge_proportion: f32,
}

// Model.hpp:271-272 default member initializers.
impl Default for CutConnectorParas {
    fn default() -> Self {
        Self {
            // Model.hpp:271 — float snap_space_proportion{0.3};
            snap_space_proportion: 0.3,
            // Model.hpp:272 — float snap_bulge_proportion{0.15};
            snap_bulge_proportion: 0.15,
        }
    }
}

// Model.hpp:275-298
//   struct CutConnectorAttributes { CutConnectorType type; CutConnectorStyle style; CutConnectorShape shape; ... };
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CutConnectorAttributes {
    /// Model.hpp:277 — CutConnectorType type{CutConnectorType::Plug};
    pub type_: CutConnectorType,
    /// Model.hpp:278 — CutConnectorStyle style{CutConnectorStyle::Prizm};
    pub style: CutConnectorStyle,
    /// Model.hpp:279 — CutConnectorShape shape{CutConnectorShape::Circle};
    pub shape: CutConnectorShape,
}

impl CutConnectorAttributes {
    // Model.hpp:283 — CutConnectorAttributes(CutConnectorType t, CutConnectorStyle st, CutConnectorShape sh)
    pub fn with(type_: CutConnectorType, style: CutConnectorStyle, shape: CutConnectorShape) -> Self {
        Self { type_, style, shape }
    }
}

// Model.hpp:281 — default constructor uses default member initializers.
impl Default for CutConnectorAttributes {
    fn default() -> Self {
        Self {
            // Model.hpp:277
            type_: CutConnectorType::Plug,
            // Model.hpp:278
            style: CutConnectorStyle::Prizm,
            // Model.hpp:279
            shape: CutConnectorShape::Circle,
        }
    }
}

// Model.hpp:291-295
//   bool operator<(const CutConnectorAttributes &other) const
impl PartialOrd for CutConnectorAttributes {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for CutConnectorAttributes {
    // Model.hpp:293-294
    //   return this->type < other.type || (this->type == other.type && this->style < other.style) ||
    //          (this->type == other.type && this->style == other.style && this->shape < other.shape);
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.type_
            .cmp(&other.type_)
            .then(self.style.cmp(&other.style))
            .then(self.shape.cmp(&other.shape))
    }
}

// Declared outside of ModelVolume, so it could be forward declared.
// Model.hpp:328-335
//   enum class ModelVolumeType : int { INVALID = -1, MODEL_PART = 0, NEGATIVE_VOLUME,
//       PARAMETER_MODIFIER, SUPPORT_BLOCKER, SUPPORT_ENFORCER };
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum ModelVolumeType {
    /// Model.hpp:329
    Invalid = -1,
    /// Model.hpp:330
    ModelPart = 0,
    /// Model.hpp:331
    NegativeVolume,
    /// Model.hpp:332
    ParameterModifier,
    /// Model.hpp:333
    SupportBlocker,
    /// Model.hpp:334
    SupportEnforcer,
}

impl ModelVolumeType {
    //BBS: refine the model part names
    // Model.cpp:3313-3329
    //   ModelVolumeType ModelVolume::type_from_string(const std::string &s)
    pub fn type_from_string(s: &str) -> ModelVolumeType {
        // New type (supporting the support enforcers & blockers)
        // Model.cpp:3316-3317
        if s == "normal_part" {
            return ModelVolumeType::ModelPart;
        }
        // Model.cpp:3318-3319
        if s == "negative_part" {
            return ModelVolumeType::NegativeVolume;
        }
        // Model.cpp:3320-3321
        if s == "modifier_part" {
            return ModelVolumeType::ParameterModifier;
        }
        // Model.cpp:3322-3323
        if s == "support_enforcer" {
            return ModelVolumeType::SupportEnforcer;
        }
        // Model.cpp:3324-3325
        if s == "support_blocker" {
            return ModelVolumeType::SupportBlocker;
        }
        //assert(s == "0");
        // Default value if invalud type string received.
        // Model.cpp:3328
        ModelVolumeType::ModelPart
    }

    //BBS: refine the model part names
    // Model.cpp:3332-3344
    //   std::string ModelVolume::type_to_string(const ModelVolumeType t)
    pub fn type_to_string(t: ModelVolumeType) -> &'static str {
        // Model.cpp:3334-3343
        match t {
            // Model.cpp:3335
            ModelVolumeType::ModelPart => "normal_part",
            // Model.cpp:3336
            ModelVolumeType::NegativeVolume => "negative_part",
            // Model.cpp:3337
            ModelVolumeType::ParameterModifier => "modifier_part",
            // Model.cpp:3338
            ModelVolumeType::SupportEnforcer => "support_enforcer",
            // Model.cpp:3339
            ModelVolumeType::SupportBlocker => "support_blocker",
            // Model.cpp:3340-3342 — default: assert(false); return "normal_part";
            _ => "normal_part",
        }
    }
}

// Model.hpp:755-760
//   enum class ConversionType : int { CONV_TO_INCH, CONV_FROM_INCH, CONV_TO_METER, CONV_FROM_METER };
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum ConversionType {
    ConvToInch,
    ConvFromInch,
    ConvToMeter,
    ConvFromMeter,
}

// Model.hpp:762-766
//   enum class En3mfType : int { From_BBS, From_Prusa, From_Other };
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum En3mfType {
    FromBBS,
    FromPrusa,
    FromOther,
}

// Model.hpp:1328-1335
//   enum ModelInstanceEPrintVolumeState : unsigned char { ... };
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum ModelInstanceEPrintVolumeState {
    /// Model.hpp:1330
    ModelInstancePvsInside,
    /// Model.hpp:1331
    ModelInstancePvsLimited,
    /// Model.hpp:1332
    ModelInstancePvsPartlyOutside,
    /// Model.hpp:1333
    ModelInstancePvsFullyOutside,
    /// Model.hpp:1334
    ModelInstanceNumBedStates,
}

// BBS structure stores extruder parameters and speed map of all models
// Model.hpp:1513-1520
//   struct ExtruderParams { std::string materialName; int bedTemp; double heatEndTemp; };
#[derive(Debug, Clone, Default, PartialEq)]
pub struct ExtruderParams {
    /// Model.hpp:1516
    pub material_name: String,
    //std::array<double, BedType::btCount> bedTemp;
    /// Model.hpp:1518
    pub bed_temp: i32,
    /// Model.hpp:1519
    pub heat_end_temp: f64,
}

// Model.hpp:1522-1533
//   struct GlobalSpeedMap { double perimeterSpeed; ...; Polygon bed_poly; };
#[derive(Debug, Clone, Default, PartialEq)]
pub struct GlobalSpeedMap {
    /// Model.hpp:1524
    pub perimeter_speed: f64,
    /// Model.hpp:1525
    pub external_perimeter_speed: f64,
    /// Model.hpp:1526
    pub infill_speed: f64,
    /// Model.hpp:1527
    pub solid_infill_speed: f64,
    /// Model.hpp:1528
    pub top_solid_infill_speed: f64,
    /// Model.hpp:1529
    pub support_speed: f64,
    /// Model.hpp:1530
    pub small_perimeter_speed: f64,
    /// Model.hpp:1531
    pub max_speed: f64,
    /// Model.hpp:1532
    pub bed_poly: Polygon,
}

// Model.hpp:1839
//   static const float SINKING_Z_THRESHOLD = -0.001f;
pub const SINKING_Z_THRESHOLD: f32 = -0.001;
// Model.hpp:1840
//   static const double SINKING_MIN_Z_THRESHOLD = 0.05;
pub const SINKING_MIN_Z_THRESHOLD: f64 = 0.05;

#[derive(Debug, Clone, Default)]
/// Container for all objects in a 3D print job with associated metadata.
/// Model.hpp:1581
pub struct Model {
    /// Objects in this model - corresponds to ModelObjectPtrs collection.
    /// Model.hpp:1588
    pub objects: Vec<ModelObject>,
    /// Model metadata - contains title, author, and descriptive information.
    /// Model.hpp:1581
    pub metadata: ModelMetadata,
    /// File path if loaded from file - for reference to source file.
    /// Model.hpp:350
    pub file_path: Option<PathBuf>,
}

/// Implementation of Model methods.
/// Model.hpp:1581-1680
impl Model {
    // Initialize a new empty Model with default constructor.
    // Model.hpp:1637
    pub fn new() -> Self {
        // Return default-constructed Model
        // Model.hpp:1637
        Self::default()
    }

    /// Insert a ModelObject into the objects collection and return its index.
    /// Model.cpp:461-465
    pub fn add_object(&mut self, object: ModelObject) -> usize {
        // Calculate index before pushing
        // Model.cpp:462
        let id = self.objects.len();
        // Append object to collection
        // Model.cpp:463
        self.objects.push(object);
        // Return new object index
        // Model.cpp:464
        id
    }

    /// Retrieve a reference to an object at the given index.
    /// Model.hpp:1592-1594
    pub fn get_object(&self, id: usize) -> Option<&ModelObject> {
        // Bounds-checked access returning None if out of range
        // Model.hpp:1593
        self.objects.get(id)
    }

    /// Retrieve a mutable reference to an object at the given index.
    /// Model.hpp:1592-1594
    pub fn get_object_mut(&mut self, id: usize) -> Option<&mut ModelObject> {
        // Bounds-checked mutable access returning None if out of range
        // Model.hpp:1593
        self.objects.get_mut(id)
    }

    /// Calculate bounding box encompassing all objects and instances.
    /// Model.hpp:437
    pub fn bounding_box(&self) -> BoundingBox3F {
        // Initialize empty bounding box
        // Model.hpp:437
        let mut bbox = BoundingBox3F::new();
        // Merge each object's bounding box into the total
        // Model.hpp:437
        for obj in &self.objects {
            // Merge individual object bounding box
            // Model.hpp:437
            bbox.merge(&obj.bounding_box());
        }
        // Return merged bounding box
        // Model.hpp:437
        bbox
    }

    /// Count total number of instances across all ModelObjects.
    /// Model.hpp:1588
    pub fn total_instances(&self) -> usize {
        // Sum instance counts across all objects
        // Model.hpp:1588
        self.objects.iter().map(|o| o.instances.len()).sum()
    }

    /// Check whether the model contains no objects.
    /// Model.hpp:1588
    pub fn is_empty(&self) -> bool {
        // Check if objects vector is empty
        // Model.hpp:1588
        self.objects.is_empty()
    }

    /// Return the number of ModelObjects in this Model.
    /// Model.hpp:1588
    pub fn object_count(&self) -> usize {
        // Return objects vector length
        // Model.hpp:1588
        self.objects.len()
    }
}

#[derive(Debug, Clone)]
/// ModelObject with mesh geometry and instance placements.
/// Model.hpp:344
pub struct ModelObject {
    /// Object name - user-visible identifier for the object.
    /// Model.hpp:347
    pub name: String,
    /// The mesh geometry - C++ stores this as volumes (ModelVolumePtrs).
    /// Model.hpp:356
    pub mesh: TriangleMesh,
    /// Instances of this object - each is a placement on the print bed.
    /// Model.hpp:353
    pub instances: Vec<Instance>,
    /// Per-object configuration - layer height, infill, extruder overrides.
    /// Model.hpp:358
    pub config: ObjectConfig,
    /// Whether this object is included in the print.
    /// Model.hpp:365
    pub printable: bool,
}

/// ModelObject methods for geometry and instance management.
/// Model.hpp:344-460
impl ModelObject {
    // Create a new ModelObject from a name and triangle mesh.
    // Model.cpp:467-483
    pub fn new(name: impl Into<String>, mesh: TriangleMesh) -> Self {
        // Construct ModelObject with name, mesh, and default instance
        // Model.cpp:468-474
        Self {
            // Set object name
            // Model.cpp:469
            name: name.into(),
            // Set mesh geometry
            // Model.cpp:470
            mesh,
            // Initialize with one default instance at the origin
            // Model.cpp:474
            instances: vec![Instance::default()],
            // Default configuration
            // Model.hpp:358
            config: ObjectConfig::default(),
            // Printable by default
            // Model.hpp:365
            printable: true,
        }
    }

    /// Calculate axis-aligned bounding box of this object's mesh.
    /// Model.hpp:437
    pub fn bounding_box(&self) -> BoundingBox3F {
        // Compute bounding box from mesh geometry
        // Model.hpp:437
        self.mesh.compute_bounding_box()
    }

    /// Add a new instance at the specified position.
    /// Model.hpp:427-429
    pub fn add_instance(&mut self, position: Point3F) -> usize {
        // Calculate instance index
        // Model.hpp:428
        let id = self.instances.len();
        // Create and push new instance at position
        // Model.hpp:428
        self.instances.push(Instance::at(position));
        // Return new instance index
        // Model.hpp:429
        id
    }

    /// Return the total number of instances for this object.
    /// Model.hpp:353
    pub fn instance_count(&self) -> usize {
        // Return instances vector length
        // Model.hpp:353
        self.instances.len()
    }

    /// Set the printable flag for this object.
    /// Model.hpp:365
    pub fn set_printable(&mut self, printable: bool) {
        // Set printable field directly
        // Model.hpp:365
        self.printable = printable;
    }
}

#[derive(Debug, Clone, Copy)]
/// An instance of an object placement on the print bed.
/// Model.hpp:1336
pub struct Instance {
    /// Instance position on the print bed.
    /// Model.hpp:1382
    pub position: Point3F,
    /// Rotation around Z axis (in degrees).
    /// Model.hpp:1388
    pub rotation_z: f64,
    /// Scaling factors for X, Y, Z axes.
    /// Model.hpp:1396
    pub scale: [f64; 3],
    /// Whether this instance is printable.
    /// Model.hpp:1349
    pub printable: bool,
}

/// Instance methods for positioning and transformation.
/// Model.hpp:1336-1460
impl Instance {
    // Create an instance at the origin with default values.
    // Model.hpp:1454
    pub fn new() -> Self {
        // Default instance at origin with identity transform
        // Model.hpp:1454
        Self {
            // Position at origin
            // Model.hpp:1382
            position: Point3F::new(0.0, 0.0, 0.0),
            // No rotation
            // Model.hpp:1388
            rotation_z: 0.0,
            // Unit scale
            // Model.hpp:1396
            scale: [1.0, 1.0, 1.0],
            // Printable by default
            // Model.hpp:1349
            printable: true,
        }
    }

    /// Create an instance at a specific position.
    /// Model.hpp:1454
    pub fn at(position: Point3F) -> Self {
        // Instance with specified position and default transform
        // Model.hpp:1454
        Self {
            // Set position
            // Model.hpp:1382
            position,
            // No rotation
            // Model.hpp:1388
            rotation_z: 0.0,
            // Unit scale
            // Model.hpp:1396
            scale: [1.0, 1.0, 1.0],
            // Printable by default
            // Model.hpp:1349
            printable: true,
        }
    }

    /// Set the Z rotation and return self for chaining.
    /// Model.hpp:1391
    pub fn with_rotation(mut self, degrees: f64) -> Self {
        // Set rotation angle in degrees
        // Model.hpp:1391
        self.rotation_z = degrees;
        // Return self for builder pattern
        // Model.hpp:1391
        self
    }

    /// Set uniform scale and return self for chaining.
    /// Model.hpp:1399
    pub fn with_scale(mut self, scale: f64) -> Self {
        // Set uniform scale on all three axes
        // Model.hpp:1399
        self.scale = [scale, scale, scale];
        // Return self for builder pattern
        // Model.hpp:1399
        self
    }

    /// Get the transformation matrix for this instance.
    /// Model.hpp:1421
    pub fn transform(&self) -> Transform3D {
        // Build transformation matrix from position, rotation, and scale
        // Model.hpp:1421
        Transform3D::identity()
            .translate(self.position.x, self.position.y, self.position.z)
            .rotate_z(self.rotation_z.to_radians())
            .scale(self.scale[0], self.scale[1], self.scale[2])
    }
}

/// Default implementation for Instance - creates instance at origin.
/// Model.hpp:1454
impl Default for Instance {
    // Return default instance at origin.
    // Model.hpp:1454
    fn default() -> Self {
        // Delegate to Instance::new()
        // Model.hpp:1454
        Self::new()
    }
}

#[derive(Debug, Clone, Default)]
/// Model metadata for descriptive information.
/// Model.hpp:1544
pub struct ModelMetadata {
    /// Model name/title.
    /// Model.hpp:1544
    pub title: Option<String>,
    /// Author/creator.
    /// Model.hpp:1544
    pub author: Option<String>,
    /// Description text.
    /// Model.hpp:1544
    pub description: Option<String>,
    /// Creation timestamp.
    /// Model.hpp:1544
    pub created_at: Option<String>,
    /// Modification timestamp.
    /// Model.hpp:1544
    pub modified_at: Option<String>,
    /// Application that created the model.
    /// Model.hpp:1544
    pub application: Option<String>,
}

#[derive(Debug, Clone, Default)]
/// Per-object configuration overrides.
/// Model.hpp:70
pub struct ObjectConfig {
    /// Layer height override.
    /// Model.hpp:70
    pub layer_height: Option<f64>,
    /// Infill density override.
    /// Model.hpp:70
    pub infill_density: Option<f64>,
    /// Number of perimeters override.
    /// Model.hpp:70
    pub perimeters: Option<u32>,
    /// Extruder to use for this object.
    /// Model.hpp:70
    pub extruder: Option<u32>,
}

/// Load a model from a file (detects format by extension).
/// Model.cpp:244-459
pub fn load_model(path: &PathBuf) -> crate::Result<Model> {
    // Extract file extension and normalize to lowercase
    // Model.cpp:245-246
    let ext = path
        .extension()
        .and_then(|e| e.to_str())
        .map(|e| e.to_lowercase());

    // Dispatch to format-specific loader based on extension
    // Model.cpp:247-459
    match ext.as_deref() {
        Some("stl") => {
            // Load STL file and wrap in Model
            // Model.cpp:247-300
            let mesh = crate::stl::read_stl_file(path)?;
            // Create empty model
            // Model.cpp:248
            let mut model = Model::new();
            // Create ModelObject from loaded mesh
            // Model.cpp:249
            let object = ModelObject::new("object", mesh);
            // Add object to model
            // Model.cpp:250
            model.add_object(object);
            // Store source file path
            // Model.cpp:251
            model.file_path = Some(path.clone());
            // Return loaded model
            // Model.cpp:252
            Ok(model)
        }
        Some("obj") => {
            // Load OBJ file with multiple objects
            // Model.cpp:301-350
            let loaded = crate::obj::load_obj(path)?;
            // Create empty model
            // Model.cpp:302
            let mut model = Model::new();
            // Add each object from loaded file
            // Model.cpp:303-305
            for obj in loaded.objects.into_iter() {
                // Add individual object
                // Model.cpp:304
                model.add_object(obj);
            }
            // Store source file path
            // Model.cpp:306
            model.file_path = Some(path.clone());
            // Return loaded model
            // Model.cpp:307
            Ok(model)
        }
        Some("3mf") => {
            // Load 3MF file with multiple objects.
            // Mirrors Model::read_from_file's 3MF branch (Model.cpp:244-373).
            // Construct empty `Model` to fill in-place.
            // Model.cpp:258
            let mut model = Model::new();
            // Temporary config + substitution context (the C++ uses local
            // `temp_config` / `temp_config_substitutions_context` when the
            // caller passes null).
            // Model.cpp:260-265
            let mut config = crate::calib::DynamicPrintConfig::default();
            let mut config_substitutions =
                crate::format::bbs_3mf::ConfigSubstitutionContext::default();
            // Convert the path to the `&str` the faithful port expects.
            let input_file = path.to_string_lossy();
            // Call the faithful 1:1 port of `load_3mf` (Format/3mf.cpp:3249).
            // Mirrors the `.3mf` dispatch at Model.cpp:324-329.
            // `check_version` follows `LoadStrategy::CheckVersion` (Model.cpp:327).
            let result = crate::format::three_mf::load_3mf(
                &input_file,
                &mut config,
                &mut config_substitutions,
                &mut model,
                true,
            )?;
            // Loading failed -> error out (Model.cpp:350-355).
            if !result {
                return Err(crate::Error::Mesh(
                    "Loading of a model file failed.".to_string(),
                ));
            }
            // The supplied file couldn't be read because it's empty.
            // Model.cpp:357-358
            if model.objects.is_empty() {
                return Err(crate::Error::Mesh(
                    "The supplied file couldn't be read because it's empty".to_string(),
                ));
            }
            // Store source file path.
            // Model.cpp:360-361
            model.file_path = Some(path.clone());
            // Return loaded model.
            // Model.cpp:373
            Ok(model)
        }
        _ => {
            // Unsupported file format error
            // Model.cpp:458-459
            Err(crate::Error::Mesh(format!(
                "Unsupported file format: {:?}",
                ext
            )))
        }
    }
}
