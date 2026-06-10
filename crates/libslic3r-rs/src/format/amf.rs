//! AMF file loading.
//!
//! Faithful 1:1 port of `Format/AMF.{hpp,cpp}` from BambuStudio libslic3r.
//!
//! C++ Reference:
//! - src/libslic3r/Format/AMF.hpp (19 lines)
//! - src/libslic3r/Format/AMF.cpp (1397 lines)
//!
//! Fidelity notes:
//! - C++ parses with expat (SAX callbacks). The Rust port drives the very same
//!   `startElement` / `endElement` / `characters` handlers from `quick-xml`
//!   events (pure Rust, wasm-safe, already a crate dependency).
//! - C++ reads `.zip.amf` archives with miniz (`miniz_extension.hpp`); the Rust
//!   port uses the `zip` crate (pure Rust, wasm-safe, already a dependency).
//! - All numeric character data is parsed with C `atof` / `atoi` semantics
//!   (leading whitespace skipped, longest valid prefix, `0` when no conversion
//!   is performed) via `objparser::strtod` — NOT Rust's strict `str::parse`.
//! - BLOCKED(model): the crate's `crate::model::Model` is still the simplified
//!   model (one `TriangleMesh` per `ModelObject`; no `ModelVolume`,
//!   `ModelMaterial`, or full `ModelInstance`). The blocked side effects are
//!   marked `BLOCKED(model)` inline:
//!     * per-volume meshes are merged into the object's single mesh (the C++
//!       `center_geometry_after_creation()` net world-space effect is identity,
//!       so merged world-space geometry matches the C++ exactly);
//!     * `ModelVolume::source.*`, per-volume `name`, and
//!       `calculate_convex_hull()` cannot be stored;
//!     * material attributes are collected in the parser context (mirroring
//!       `ModelMaterial::attributes`) but cannot be attached to `Model`;
//!     * instance `rx`/`ry` rotations and `mirror{x,y,z}` cannot be represented
//!       by the simplified `Instance` (only Z rotation, scale, offset,
//!       printable).
//! - AMF export (`store_amf`) is removed in BambuStudio (AMF.hpp:12-15,
//!   AMF.cpp:1117-1395 are commented out) and is therefore not ported.

use crate::calib::DynamicPrintConfig;
use crate::format::bbs_3mf::ConfigSubstitutionContext;
use crate::format::objparser;
use crate::geometry::geometry::Transform3d;
use crate::geometry::Point3F;
use crate::locales_utils::{is_decimal_separator_point, CNumericLocalesSetter};
use crate::model::{Model, ModelObject};
use crate::normal_utils::{indexed_triangle_set, StlTriangleVertexIndices, StlVertex};
use crate::triangle_mesh::{
    its_compactify_vertices, its_flip_triangles, its_volume, Triangle, TriangleMesh,
};

use log::error;
use std::collections::BTreeMap;
use std::io::Read;

// AMF.cpp:41-55
// VERSION NUMBERS
// 0 : .amf, .amf.xml and .zip.amf files saved by older slic3r. No version definition in them.
// 1 : Introduction of amf versioning. No other change in data saved into amf files.
// 2 : Added z component of offset
//     Added x and y components of rotation
//     Added x, y and z components of scale
//     Added x, y and z components of mirror
// 3 : Added volumes' matrices and source data, meshes transformed back to their coordinate system on loading.
// WARNING !! -> the version number has been rolled back to 2
//               the next change should use 4
// (const unsigned int VERSION_AMF / VERSION_AMF_COMPATIBLE / SLIC3RPE_AMF_VERSION /
//  SLIC3R_CONFIG_TYPE are commented out in BambuStudio's AMF.cpp:51-55.)

// ---------------------------------------------------------------------------
// C number parsing helpers (AMF.cpp parses with ::atof / ::atoi)
// ---------------------------------------------------------------------------

/// C `atof(nptr)` == `strtod(nptr, NULL)`: skip leading C whitespace, parse the
/// longest valid floating-point prefix, `0.0` when no conversion is performed.
fn atof(s: &str) -> f64 {
    objparser::strtod(s.as_bytes(), 0).0
}

/// C `atoi(nptr)`: skip leading C whitespace, optional sign, longest run of
/// decimal digits; `0` when no conversion is performed. (C `atoi` is
/// `(int)strtol(nptr, NULL, 10)`; `strtol` saturates on overflow — AMF indices
/// never approach those bounds.)
fn atoi(s: &str) -> i32 {
    let bytes = s.as_bytes();
    let n = bytes.len();
    let mut i = 0;
    // isspace() in the C locale.
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

/// boost::iends_with — case-insensitive ASCII suffix test
/// (AMF.cpp:1050, AMF.cpp:1101).
fn iends_with(haystack: &str, suffix: &str) -> bool {
    let h = haystack.as_bytes();
    let s = suffix.as_bytes();
    if s.len() > h.len() {
        return false;
    }
    h[h.len() - s.len()..]
        .iter()
        .zip(s.iter())
        .all(|(a, b)| a.eq_ignore_ascii_case(b))
}

/// Eigen `Transform3d::isApprox(other, prec)`:
/// `(a - b).norm() <= prec * min(a.norm(), b.norm())` (Frobenius norm).
/// Used by the volume-transform check at AMF.cpp:658.
fn transform_is_approx(a: &Transform3d, b: &Transform3d, prec: f64) -> bool {
    (a - b).norm() <= prec * a.norm().min(b.norm())
}

// AMF.cpp:60-63
// "macro used to mark string used at localization" — `#define L(s) (s)` and
// `#define _(s) Slic3r::I18N::translate(s)` are only used in commented-out
// C++ code and are not ported.

// ---------------------------------------------------------------------------
// AMFParserContext (AMF.cpp:65-269)
// ---------------------------------------------------------------------------

/// AMF.cpp:129-174
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(dead_code)]
enum AMFNodeType {
    /// AMF.cpp:130
    Invalid,
    /// AMF.cpp:131
    Unknown,
    /// AMF.cpp:132 — amf
    Amf,
    // AMF.cpp:133 — amf/metadata (no dedicated node type)
    /// AMF.cpp:134 — amf/material
    Material,
    // AMF.cpp:135 — amf/material/metadata (no dedicated node type)
    /// AMF.cpp:136 — amf/object
    Object,
    // AMF.cpp:137 — amf/object/metadata (no dedicated node type)
    // AMF.cpp:138-140 — NODE_TYPE_LAYER_CONFIG / NODE_TYPE_RANGE are commented out in C++
    /// AMF.cpp:141 — amf/object/mesh
    Mesh,
    /// AMF.cpp:142 — amf/object/mesh/vertices
    Vertices,
    /// AMF.cpp:143 — amf/object/mesh/vertices/vertex
    Vertex,
    /// AMF.cpp:144 — amf/object/mesh/vertices/vertex/coordinates
    Coordinates,
    /// AMF.cpp:145 — amf/object/mesh/vertices/vertex/coordinates/x
    CoordinateX,
    /// AMF.cpp:146 — amf/object/mesh/vertices/vertex/coordinates/y
    CoordinateY,
    /// AMF.cpp:147 — amf/object/mesh/vertices/vertex/coordinates/z
    CoordinateZ,
    /// AMF.cpp:148 — amf/object/mesh/volume
    Volume,
    // AMF.cpp:149 — amf/object/mesh/volume/metadata (no dedicated node type)
    /// AMF.cpp:150 — amf/object/mesh/volume/triangle
    Triangle,
    /// AMF.cpp:151 — amf/object/mesh/volume/triangle/v1
    Vertex1,
    /// AMF.cpp:152 — amf/object/mesh/volume/triangle/v2
    Vertex2,
    /// AMF.cpp:153 — amf/object/mesh/volume/triangle/v3
    Vertex3,
    /// AMF.cpp:154 — amf/constellation
    Constellation,
    /// AMF.cpp:155 — amf/constellation/instance
    Instance,
    /// AMF.cpp:156 — amf/constellation/instance/deltax
    DeltaX,
    /// AMF.cpp:157 — amf/constellation/instance/deltay
    DeltaY,
    /// AMF.cpp:158 — amf/constellation/instance/deltaz
    DeltaZ,
    /// AMF.cpp:159 — amf/constellation/instance/rx
    Rx,
    /// AMF.cpp:160 — amf/constellation/instance/ry
    Ry,
    /// AMF.cpp:161 — amf/constellation/instance/rz
    Rz,
    /// AMF.cpp:162 — amf/constellation/instance/scale
    Scale,
    /// AMF.cpp:163 — amf/constellation/instance/scalex
    ScaleX,
    /// AMF.cpp:164 — amf/constellation/instance/scaley
    ScaleY,
    /// AMF.cpp:165 — amf/constellation/instance/scalez
    ScaleZ,
    /// AMF.cpp:166 — amf/constellation/instance/mirrorx
    MirrorX,
    /// AMF.cpp:167 — amf/constellation/instance/mirrory
    MirrorY,
    /// AMF.cpp:168 — amf/constellation/instance/mirrorz
    MirrorZ,
    /// AMF.cpp:169 — amf/constellation/instance/mirrorz [sic]
    Printable,
    // AMF.cpp:170-172 — NODE_TYPE_CUSTOM_GCODE / GCODE_PER_HEIGHT / CUSTOM_GCODE_MODE
    //                   are commented out in C++
    /// AMF.cpp:173 — anywhere under amf/*/metadata
    Metadata,
}

/// AMF.cpp:176-222
#[derive(Debug, Clone)]
struct Instance {
    // Shift in the X axis. (AMF.cpp:183-185)
    deltax: f32,
    deltax_set: bool,
    // Shift in the Y axis. (AMF.cpp:186-188)
    deltay: f32,
    deltay_set: bool,
    // Shift in the Z axis. (AMF.cpp:189-191)
    deltaz: f32,
    deltaz_set: bool,
    // Rotation around the X axis. (AMF.cpp:192-194)
    rx: f32,
    rx_set: bool,
    // Rotation around the Y axis. (AMF.cpp:195-197)
    ry: f32,
    ry_set: bool,
    // Rotation around the Z axis. (AMF.cpp:198-200)
    rz: f32,
    rz_set: bool,
    // Scaling factors (AMF.cpp:201-207)
    scalex: f32,
    scalex_set: bool,
    scaley: f32,
    scaley_set: bool,
    scalez: f32,
    scalez_set: bool,
    // Mirroring factors (AMF.cpp:208-214)
    mirrorx: f32,
    mirrorx_set: bool,
    mirrory: f32,
    mirrory_set: bool,
    mirrorz: f32,
    mirrorz_set: bool,
    // printable property (AMF.cpp:215-216)
    printable: bool,
}

impl Instance {
    /// AMF.cpp:177-182 — the C++ constructor initializes only the `*_set`
    /// flags and `printable`; the float members are left uninitialized (they
    /// are only ever read when the corresponding flag is set). Rust requires
    /// initialization; `0.0` is used as a defined placeholder.
    fn new() -> Self {
        Self {
            deltax: 0.0,
            deltax_set: false,
            deltay: 0.0,
            deltay_set: false,
            deltaz: 0.0,
            deltaz_set: false,
            rx: 0.0,
            rx_set: false,
            ry: 0.0,
            ry_set: false,
            rz: 0.0,
            rz_set: false,
            scalex: 0.0,
            scalex_set: false,
            scaley: 0.0,
            scaley_set: false,
            scalez: 0.0,
            scalez_set: false,
            mirrorx: 0.0,
            mirrorx_set: false,
            mirrory: 0.0,
            mirrory_set: false,
            mirrorz: 0.0,
            mirrorz_set: false,
            printable: true,
        }
    }

    /// AMF.cpp:218-221
    fn anything_set(&self) -> bool {
        self.deltax_set
            || self.deltay_set
            || self.deltaz_set
            || self.rx_set
            || self.ry_set
            || self.rz_set
            || self.scalex_set
            || self.scaley_set
            || self.scalez_set
            || self.mirrorx_set
            || self.mirrory_set
            || self.mirrorz_set
    }
}

/// AMF.cpp:224-228
#[derive(Debug, Clone)]
struct Object {
    /// AMF.cpp:226
    idx: i32,
    /// AMF.cpp:227
    instances: Vec<Instance>,
}

impl Object {
    /// AMF.cpp:225 — `Object() : idx(-1) {}`
    fn new() -> Self {
        Self {
            idx: -1,
            instances: Vec::new(),
        }
    }
}

/// AMF.cpp:65-269 — `struct AMFParserContext`
struct AMFParserContext<'a> {
    // AMF.cpp:230-233 — m_version (commented out) / m_parser: the expat parser
    // instance has no Rust counterpart; `parse_buffer` drives the handlers.
    /// Error code returned by the application side of the parser. In that case
    /// the expat may not reliably deliver the error state after returning from
    /// XML_Parse() function, thus we keep the error state here.
    /// AMF.cpp:234-236
    error: bool,
    /// AMF.cpp:237
    error_message: String,
    /// Model to receive objects extracted from an AMF file.
    /// AMF.cpp:238-239
    model: &'a mut Model,
    /// Current parsing path in the XML file.
    /// AMF.cpp:240-241
    path: Vec<AMFNodeType>,
    /// Current object allocated for an amf/object XML subtree.
    /// AMF.cpp:242-243 — C++ holds a `ModelObject*`; the Rust port stores the
    /// index into `model.objects`.
    object: Option<usize>,
    /// Map from obect name to object idx & instances. [sic]
    /// AMF.cpp:244-245 — C++ `std::map<std::string, Object>` iterates in key
    /// order; `BTreeMap` preserves that ordering (it matters for the order in
    /// which `endDocument` appends cloned objects).
    object_instances_map: BTreeMap<String, Object>,
    /// Vertices parsed for the current m_object.
    /// AMF.cpp:246-247 — `std::vector<Vec3f>`
    object_vertices: Vec<StlVertex>,
    /// Current volume allocated for an amf/object/mesh/volume subtree.
    /// AMF.cpp:248-249 — C++ holds a `ModelVolume*`. BLOCKED(model): the
    /// simplified `Model` has no volumes; this flag tracks whether a volume
    /// subtree is open, and the finished volume mesh is merged into the
    /// object's single mesh at `</volume>`.
    volume: bool,
    /// Faces collected for the current m_volume.
    /// AMF.cpp:250-251 — `std::vector<Vec3i>`
    volume_facets: Vec<StlTriangleVertexIndices>,
    /// Transformation matrix of a volume mesh from its coordinate system to
    /// Object's coordinate system.
    /// AMF.cpp:252-253 — only ever set to identity in BambuStudio (the
    /// `slic3r.matrix` metadata that assigned it, AMF.cpp:799-801, is
    /// commented out).
    volume_transform: Transform3d,
    /// Current material allocated for an amf/metadata subtree.
    /// AMF.cpp:254-255 — C++ holds a `ModelMaterial*`. BLOCKED(model): the
    /// simplified `Model` has no materials; the id keys into `materials`.
    material: Option<String>,
    /// Stand-in for `ModelMaterial::attributes` of all materials parsed so far
    /// (`m_model.add_material(...)` destinations). BLOCKED(model): cannot be
    /// attached to the simplified `Model`.
    #[allow(dead_code)]
    materials: BTreeMap<String, BTreeMap<String, String>>,
    /// Current instance allocated for an amf/constellation/instance subtree.
    /// AMF.cpp:256-257 — C++ holds an `Instance*` pointing into
    /// `m_object_instances_map`; the Rust port stores (objectid, index).
    instance: Option<(String, usize)>,
    /// Generic string buffer for vertices, face indices, metadata etc.
    /// AMF.cpp:258-259
    value: [String; 5],
    /// Pointer to config to update if config data are stored inside the amf file
    /// AMF.cpp:260-261 — never read in BambuStudio (the config-metadata code,
    /// AMF.cpp:717-827, is commented out); kept for signature parity.
    #[allow(dead_code)]
    config: Option<&'a mut DynamicPrintConfig>,
    /// Config substitution rules and collected config substitution log.
    /// AMF.cpp:262-263 — never read in BambuStudio (see above).
    #[allow(dead_code)]
    config_substitutions: Option<&'a mut ConfigSubstitutionContext>,
    /// BBS: add units logic
    /// AMF.cpp:264-265
    use_inches: bool,
}

impl<'a> AMFParserContext<'a> {
    /// AMF.cpp:67-74
    fn new(
        config: Option<&'a mut DynamicPrintConfig>,
        config_substitutions: Option<&'a mut ConfigSubstitutionContext>,
        model: &'a mut Model,
    ) -> Self {
        Self {
            error: false,
            error_message: String::new(),
            model,
            // AMF.cpp:73 — m_path.reserve(12);
            path: Vec::with_capacity(12),
            object: None,
            object_instances_map: BTreeMap::new(),
            object_vertices: Vec::new(),
            volume: false,
            volume_facets: Vec::new(),
            volume_transform: Transform3d::identity(),
            material: None,
            materials: BTreeMap::new(),
            instance: None,
            value: Default::default(),
            config,
            config_substitutions,
            use_inches: false,
        }
    }

    /// AMF.cpp:76-83
    fn stop(&mut self, msg: &str) {
        debug_assert!(!self.error);
        debug_assert!(self.error_message.is_empty());
        self.error = true;
        self.error_message = msg.to_string();
        // AMF.cpp:82 — XML_StopParser(m_parser, 0): the quick-xml driver in
        // `parse_buffer` checks `error()` after every event and stops.
    }

    /// AMF.cpp:85
    fn error(&self) -> bool {
        self.error
    }

    /// AMF.cpp:86-92 — the branch for "error signalled by the expat parser"
    /// (`XML_ErrorString`) lives in the quick-xml driver; this returns the
    /// user-code message.
    fn error_message(&self) -> &str {
        if self.error_message.is_empty() {
            // The error was signalled by the user code, not the expat parser.
            "Parse AMF file failed"
        } else {
            &self.error_message
        }
    }

    /// Helper resolving the C++ `m_instance` pointer (AMF.cpp:256-257).
    fn instance_mut(&mut self) -> Option<&mut Instance> {
        let (id, i) = self.instance.as_ref()?;
        self.object_instances_map
            .get_mut(id)?
            .instances
            .get_mut(*i)
    }

    /// AMF.cpp:271-462 — `void AMFParserContext::startElement(const char *name, const char **atts)`
    fn start_element(&mut self, name: &str, atts: &[(String, String)]) {
        // AMF.cpp:273
        let mut node_type_new = AMFNodeType::Unknown;
        // AMF.cpp:274
        match self.path.len() {
            0 => {
                // An AMF file must start with an <amf> tag. (AMF.cpp:275-278)
                node_type_new = AMFNodeType::Amf;
                if name != "amf" {
                    self.stop("");
                }
                // BBS: add units logic (AMF.cpp:279-281)
                if let Some(units) = get_attribute(atts, "unit") {
                    if units == "inch" {
                        self.use_inches = true;
                    }
                }
            }
            1 => {
                if name == "metadata" {
                    // AMF.cpp:285-290
                    if let Some(typ) = get_attribute(atts, "type") {
                        self.value[0] = typ.to_string();
                        node_type_new = AMFNodeType::Metadata;
                    }
                } else if name == "material" {
                    // AMF.cpp:291-294 — m_material = m_model.add_material(id or "_")
                    let material_id = get_attribute(atts, "id").unwrap_or("_").to_string();
                    // BLOCKED(model): no `Model::add_material`; mirror
                    // ModelMaterial creation in the context-local map.
                    self.materials.entry(material_id.clone()).or_default();
                    self.material = Some(material_id);
                    node_type_new = AMFNodeType::Material;
                } else if name == "object" {
                    // AMF.cpp:295-304
                    match get_attribute(atts, "id") {
                        None => self.stop(""),
                        Some(object_id) => {
                            // AMF.cpp:300
                            debug_assert!(self.object_vertices.is_empty());
                            // AMF.cpp:301 — m_object = m_model.add_object();
                            // C++ Model::add_object() creates a ModelObject with
                            // NO instances; the crate's ModelObject::new injects
                            // one default instance, so clear it.
                            let mut object = ModelObject::new("", TriangleMesh::new());
                            object.instances.clear();
                            self.model.add_object(object);
                            self.object = Some(self.model.objects.len() - 1);
                            // AMF.cpp:302 — m_object_instances_map[object_id].idx = int(m_model.objects.size())-1;
                            self.object_instances_map
                                .entry(object_id.to_string())
                                .or_insert_with(Object::new)
                                .idx = self.model.objects.len() as i32 - 1;
                            node_type_new = AMFNodeType::Object;
                        }
                    }
                } else if name == "constellation" {
                    // AMF.cpp:305-306
                    node_type_new = AMFNodeType::Constellation;
                }
                // AMF.cpp:307-309 — "custom_gcodes_per_height" handling is
                // commented out in C++.
            }
            2 => {
                if name == "metadata" {
                    // AMF.cpp:312-316
                    if self.path[1] == AMFNodeType::Material || self.path[1] == AMFNodeType::Object
                    {
                        // AMF.cpp:314 — m_value[0] = get_attribute(atts, "type");
                        // (C++ assigns the raw pointer to std::string; a missing
                        // "type" would be UB there — map it to "".)
                        self.value[0] = get_attribute(atts, "type").unwrap_or("").to_string();
                        node_type_new = AMFNodeType::Metadata;
                    }
                }
                // AMF.cpp:317-318 — "layer_config_ranges" handling is commented
                // out in C++.
                else if name == "mesh" {
                    // AMF.cpp:319-321
                    if self.path[1] == AMFNodeType::Object {
                        node_type_new = AMFNodeType::Mesh;
                    }
                } else if name == "instance" {
                    // AMF.cpp:322-334
                    if self.path[1] == AMFNodeType::Constellation {
                        match get_attribute(atts, "objectid") {
                            None => self.stop(""),
                            Some(object_id) => {
                                // AMF.cpp:328-329
                                let entry = self
                                    .object_instances_map
                                    .entry(object_id.to_string())
                                    .or_insert_with(Object::new);
                                entry.instances.push(Instance::new());
                                self.instance =
                                    Some((object_id.to_string(), entry.instances.len() - 1));
                                node_type_new = AMFNodeType::Instance;
                            }
                        }
                    } else {
                        // AMF.cpp:333-334
                        self.stop("");
                    }
                }
                // AMF.cpp:336-365 — custom gcode "code"/"mode" handling is
                // commented out in C++.
            }
            3 => {
                if self.path[2] == AMFNodeType::Mesh {
                    // AMF.cpp:369
                    debug_assert!(self.object.is_some());
                    if name == "vertices" {
                        // AMF.cpp:370-371
                        node_type_new = AMFNodeType::Vertices;
                    } else if name == "volume" {
                        // AMF.cpp:372-377
                        debug_assert!(!self.volume);
                        // AMF.cpp:374 — m_volume = m_object->add_volume(TriangleMesh());
                        // BLOCKED(model): no ModelVolume; mark a volume subtree
                        // open. Its mesh is merged into the object at </volume>.
                        self.volume = true;
                        // AMF.cpp:375 — m_volume_transform = Transform3d::Identity();
                        self.volume_transform = Transform3d::identity();
                        node_type_new = AMFNodeType::Volume;
                    }
                } else if self.path[2] == AMFNodeType::Instance {
                    // AMF.cpp:380
                    debug_assert!(self.instance.is_some());
                    // AMF.cpp:381-408
                    if name == "deltax" {
                        node_type_new = AMFNodeType::DeltaX;
                    } else if name == "deltay" {
                        node_type_new = AMFNodeType::DeltaY;
                    } else if name == "deltaz" {
                        node_type_new = AMFNodeType::DeltaZ;
                    } else if name == "rx" {
                        node_type_new = AMFNodeType::Rx;
                    } else if name == "ry" {
                        node_type_new = AMFNodeType::Ry;
                    } else if name == "rz" {
                        node_type_new = AMFNodeType::Rz;
                    } else if name == "scalex" {
                        node_type_new = AMFNodeType::ScaleX;
                    } else if name == "scaley" {
                        node_type_new = AMFNodeType::ScaleY;
                    } else if name == "scalez" {
                        node_type_new = AMFNodeType::ScaleZ;
                    } else if name == "scale" {
                        node_type_new = AMFNodeType::Scale;
                    } else if name == "mirrorx" {
                        node_type_new = AMFNodeType::MirrorX;
                    } else if name == "mirrory" {
                        node_type_new = AMFNodeType::MirrorY;
                    } else if name == "mirrorz" {
                        node_type_new = AMFNodeType::MirrorZ;
                    } else if name == "printable" {
                        node_type_new = AMFNodeType::Printable;
                    }
                }
                // AMF.cpp:410-413 — NODE_TYPE_RANGE handling is commented out
                // in C++.
            }
            4 => {
                if self.path[3] == AMFNodeType::Vertices {
                    // AMF.cpp:416-418
                    if name == "vertex" {
                        node_type_new = AMFNodeType::Vertex;
                    }
                } else if self.path[3] == AMFNodeType::Volume {
                    if name == "metadata" {
                        // AMF.cpp:420-427
                        match get_attribute(atts, "type") {
                            None => self.stop(""),
                            Some(typ) => {
                                self.value[0] = typ.to_string();
                                node_type_new = AMFNodeType::Metadata;
                            }
                        }
                    } else if name == "triangle" {
                        // AMF.cpp:428-429
                        node_type_new = AMFNodeType::Triangle;
                    }
                }
                // AMF.cpp:431-434 — NODE_TYPE_RANGE metadata handling is
                // commented out in C++.
            }
            5 => {
                let b = name.as_bytes();
                if name == "coordinates" {
                    // AMF.cpp:437-441
                    if self.path[4] == AMFNodeType::Vertex {
                        node_type_new = AMFNodeType::Coordinates;
                    } else {
                        self.stop("");
                    }
                } else if b.len() == 2 && b[0] == b'v' && (b'1'..=b'3').contains(&b[1]) {
                    // AMF.cpp:442-447 — name[0] == 'v' && name[1] in '1'..'3' && name[2] == 0
                    if self.path[4] == AMFNodeType::Triangle {
                        // AMF.cpp:444 — AMFNodeType(NODE_TYPE_VERTEX1 + name[1] - '1')
                        node_type_new = match b[1] {
                            b'1' => AMFNodeType::Vertex1,
                            b'2' => AMFNodeType::Vertex2,
                            _ => AMFNodeType::Vertex3,
                        };
                    } else {
                        self.stop("");
                    }
                }
            }
            6 => {
                let b = name.as_bytes();
                // AMF.cpp:450 — (name[0] == 'x' || 'y' || 'z') && name[1] == 0
                if b.len() == 1 && (b[0] == b'x' || b[0] == b'y' || b[0] == b'z') {
                    if self.path[5] == AMFNodeType::Coordinates {
                        // AMF.cpp:452 — AMFNodeType(NODE_TYPE_COORDINATE_X + name[0] - 'x')
                        node_type_new = match b[0] {
                            b'x' => AMFNodeType::CoordinateX,
                            b'y' => AMFNodeType::CoordinateY,
                            _ => AMFNodeType::CoordinateZ,
                        };
                    } else {
                        self.stop("");
                    }
                }
            }
            // AMF.cpp:457-458
            _ => {}
        }

        // AMF.cpp:461
        self.path.push(node_type_new);
    }

    /// AMF.cpp:464-507 — `void AMFParserContext::characters(const XML_Char *s, int len)`
    fn characters(&mut self, s: &str) {
        // expat only delivers character data inside the document root, so
        // `m_path` is never empty here; the quick-xml driver filters events
        // outside the root.
        let back = *self.path.last().unwrap();
        if back == AMFNodeType::Metadata {
            // AMF.cpp:466-468
            self.value[1].push_str(s);
        } else {
            // AMF.cpp:471
            match self.path.len() {
                4 => {
                    // AMF.cpp:472-488
                    if matches!(
                        back,
                        AMFNodeType::DeltaX
                            | AMFNodeType::DeltaY
                            | AMFNodeType::DeltaZ
                            | AMFNodeType::Rx
                            | AMFNodeType::Ry
                            | AMFNodeType::Rz
                            | AMFNodeType::ScaleX
                            | AMFNodeType::ScaleY
                            | AMFNodeType::ScaleZ
                            | AMFNodeType::Scale
                            | AMFNodeType::MirrorX
                            | AMFNodeType::MirrorY
                            | AMFNodeType::MirrorZ
                            | AMFNodeType::Printable
                    ) {
                        self.value[0].push_str(s);
                    }
                }
                6 => {
                    // AMF.cpp:489-495
                    match back {
                        AMFNodeType::Vertex1 => self.value[0].push_str(s),
                        AMFNodeType::Vertex2 => self.value[1].push_str(s),
                        AMFNodeType::Vertex3 => self.value[2].push_str(s),
                        _ => {}
                    }
                    // AMF.cpp:496 — C++ case 6 has no `break` and falls through
                    // into case 7. Harmless (depth-6 nodes are never
                    // COORDINATE_*), reproduced for fidelity.
                    match back {
                        AMFNodeType::CoordinateX => self.value[0].push_str(s),
                        AMFNodeType::CoordinateY => self.value[1].push_str(s),
                        AMFNodeType::CoordinateZ => self.value[2].push_str(s),
                        _ => {}
                    }
                }
                7 => {
                    // AMF.cpp:496-502
                    match back {
                        AMFNodeType::CoordinateX => self.value[0].push_str(s),
                        AMFNodeType::CoordinateY => self.value[1].push_str(s),
                        AMFNodeType::CoordinateZ => self.value[2].push_str(s),
                        _ => {}
                    }
                }
                // AMF.cpp:503-504
                _ => {}
            }
        }
    }

    /// AMF.cpp:509-851 — `void AMFParserContext::endElement(const char *)`
    fn end_element(&mut self) {
        // AMF.cpp:511
        debug_assert!(is_decimal_separator_point());
        // AMF.cpp:512
        let back = *self.path.last().unwrap();
        match back {
            // Constellation transformation:
            // AMF.cpp:515-520
            AMFNodeType::DeltaX => {
                debug_assert!(self.instance.is_some());
                let v = atof(&self.value[0]) as f32;
                if let Some(instance) = self.instance_mut() {
                    instance.deltax = v;
                    instance.deltax_set = true;
                }
                self.value[0].clear();
            }
            // AMF.cpp:521-526
            AMFNodeType::DeltaY => {
                debug_assert!(self.instance.is_some());
                let v = atof(&self.value[0]) as f32;
                if let Some(instance) = self.instance_mut() {
                    instance.deltay = v;
                    instance.deltay_set = true;
                }
                self.value[0].clear();
            }
            // AMF.cpp:527-532
            AMFNodeType::DeltaZ => {
                debug_assert!(self.instance.is_some());
                let v = atof(&self.value[0]) as f32;
                if let Some(instance) = self.instance_mut() {
                    instance.deltaz = v;
                    instance.deltaz_set = true;
                }
                self.value[0].clear();
            }
            // AMF.cpp:533-538
            AMFNodeType::Rx => {
                debug_assert!(self.instance.is_some());
                let v = atof(&self.value[0]) as f32;
                if let Some(instance) = self.instance_mut() {
                    instance.rx = v;
                    instance.rx_set = true;
                }
                self.value[0].clear();
            }
            // AMF.cpp:539-544
            AMFNodeType::Ry => {
                debug_assert!(self.instance.is_some());
                let v = atof(&self.value[0]) as f32;
                if let Some(instance) = self.instance_mut() {
                    instance.ry = v;
                    instance.ry_set = true;
                }
                self.value[0].clear();
            }
            // AMF.cpp:545-550
            AMFNodeType::Rz => {
                debug_assert!(self.instance.is_some());
                let v = atof(&self.value[0]) as f32;
                if let Some(instance) = self.instance_mut() {
                    instance.rz = v;
                    instance.rz_set = true;
                }
                self.value[0].clear();
            }
            // AMF.cpp:551-560 — <scale> sets all three factors from the same value.
            AMFNodeType::Scale => {
                debug_assert!(self.instance.is_some());
                let v = atof(&self.value[0]) as f32;
                if let Some(instance) = self.instance_mut() {
                    instance.scalex = v;
                    instance.scalex_set = true;
                    instance.scaley = v;
                    instance.scaley_set = true;
                    instance.scalez = v;
                    instance.scalez_set = true;
                }
                self.value[0].clear();
            }
            // AMF.cpp:561-566
            AMFNodeType::ScaleX => {
                debug_assert!(self.instance.is_some());
                let v = atof(&self.value[0]) as f32;
                if let Some(instance) = self.instance_mut() {
                    instance.scalex = v;
                    instance.scalex_set = true;
                }
                self.value[0].clear();
            }
            // AMF.cpp:567-572
            AMFNodeType::ScaleY => {
                debug_assert!(self.instance.is_some());
                let v = atof(&self.value[0]) as f32;
                if let Some(instance) = self.instance_mut() {
                    instance.scaley = v;
                    instance.scaley_set = true;
                }
                self.value[0].clear();
            }
            // AMF.cpp:573-578
            AMFNodeType::ScaleZ => {
                debug_assert!(self.instance.is_some());
                let v = atof(&self.value[0]) as f32;
                if let Some(instance) = self.instance_mut() {
                    instance.scalez = v;
                    instance.scalez_set = true;
                }
                self.value[0].clear();
            }
            // AMF.cpp:579-584
            AMFNodeType::MirrorX => {
                debug_assert!(self.instance.is_some());
                let v = atof(&self.value[0]) as f32;
                if let Some(instance) = self.instance_mut() {
                    instance.mirrorx = v;
                    instance.mirrorx_set = true;
                }
                self.value[0].clear();
            }
            // AMF.cpp:585-590
            AMFNodeType::MirrorY => {
                debug_assert!(self.instance.is_some());
                let v = atof(&self.value[0]) as f32;
                if let Some(instance) = self.instance_mut() {
                    instance.mirrory = v;
                    instance.mirrory_set = true;
                }
                self.value[0].clear();
            }
            // AMF.cpp:591-596
            AMFNodeType::MirrorZ => {
                debug_assert!(self.instance.is_some());
                let v = atof(&self.value[0]) as f32;
                if let Some(instance) = self.instance_mut() {
                    instance.mirrorz = v;
                    instance.mirrorz_set = true;
                }
                self.value[0].clear();
            }
            // AMF.cpp:597-601 — m_instance->printable = bool(atoi(...));
            AMFNodeType::Printable => {
                debug_assert!(self.instance.is_some());
                let v = atoi(&self.value[0]) != 0;
                if let Some(instance) = self.instance_mut() {
                    instance.printable = v;
                }
                self.value[0].clear();
            }

            // Object vertices:
            // AMF.cpp:603-611
            AMFNodeType::Vertex => {
                debug_assert!(self.object.is_some());
                // Parse the vertex data (AMF.cpp:606-607)
                self.object_vertices.push(StlVertex::new(
                    atof(&self.value[0]) as f32,
                    atof(&self.value[1]) as f32,
                    atof(&self.value[2]) as f32,
                ));
                self.value[0].clear();
                self.value[1].clear();
                self.value[2].clear();
            }

            // Faces of the current volume:
            // AMF.cpp:613-620
            AMFNodeType::Triangle => {
                debug_assert!(self.object.is_some() && self.volume);
                self.volume_facets.push(StlTriangleVertexIndices::new(
                    atoi(&self.value[0]),
                    atoi(&self.value[1]),
                    atoi(&self.value[2]),
                ));
                self.value[0].clear();
                self.value[1].clear();
                self.value[2].clear();
            }

            // Closing the current volume. Create an STL from m_volume_facets
            // pointing to m_object_vertices.
            // AMF.cpp:622-673
            AMFNodeType::Volume => {
                // AMF.cpp:625
                debug_assert!(self.object.is_some() && self.volume);
                // AMF.cpp:626-629
                if self.volume_facets.is_empty() {
                    self.stop("Found an empty triangle mesh");
                    // C++ returns here WITHOUT popping m_path.
                    return;
                }

                {
                    // Verify validity of face indices, find the vertex span.
                    // AMF.cpp:632-644
                    let mut min_id = self.volume_facets[0][0];
                    let mut max_id = min_id;
                    for face in &self.volume_facets {
                        for k in 0..3 {
                            let tri_id = face[k];
                            if tri_id < 0 || tri_id >= self.object_vertices.len() as i32 {
                                self.stop("Found a malformed triangle mesh");
                                // C++ returns here WITHOUT popping m_path.
                                return;
                            }
                            min_id = min_id.min(tri_id);
                            max_id = max_id.max(tri_id);
                        }
                    }

                    // rebase indices to the current vertices list (AMF.cpp:646-648)
                    for face in &mut self.volume_facets {
                        *face -= StlTriangleVertexIndices::new(min_id, min_id, min_id);
                    }

                    // AMF.cpp:650 — indexed_triangle_set its { std::move(m_volume_facets),
                    //               { begin() + min_id, begin() + max_id + 1 } };
                    let mut its = indexed_triangle_set {
                        indices: std::mem::take(&mut self.volume_facets),
                        vertices: self.object_vertices[min_id as usize..=max_id as usize]
                            .to_vec(),
                    };
                    // AMF.cpp:651 — its_compactify_vertices(its); (shrink_to_fit defaults to true)
                    its_compactify_vertices(&mut its, true);
                    // AMF.cpp:652-653
                    if its_volume(&its) < 0.0 {
                        its_flip_triangles(&mut its);
                    }
                    // AMF.cpp:654 — m_volume->set_mesh(std::move(its));
                    // BLOCKED(model): no ModelVolume; merge this volume's mesh
                    // into the object's single TriangleMesh. World-space
                    // geometry matches the C++ (see module notes).
                    if let Some(object_idx) = self.object {
                        let object = &mut self.model.objects[object_idx];
                        let base = object.mesh.vertices().len() as u32;
                        let mut vertices = object.mesh.vertices().to_vec();
                        let mut indices = object.mesh.indices().to_vec();
                        vertices.extend(
                            its.vertices
                                .iter()
                                .map(|v| Point3F::new(v[0] as f64, v[1] as f64, v[2] as f64)),
                        );
                        indices.extend(its.indices.iter().map(|f| {
                            Triangle::new(
                                base + f[0] as u32,
                                base + f[1] as u32,
                                base + f[2] as u32,
                            )
                        }));
                        object.mesh = TriangleMesh::from_parts(vertices, indices);
                    }
                }

                // stores the volume matrix taken from the metadata, if present
                // AMF.cpp:657-659 — always identity in BambuStudio: the
                // `slic3r.matrix` metadata that assigned m_volume_transform
                // (AMF.cpp:799-801) is commented out.
                let has_transform =
                    !transform_is_approx(&self.volume_transform, &Transform3d::identity(), 1e-10);
                if has_transform {
                    // AMF.cpp:659 — m_volume->source.transform = Transformation(m_volume_transform);
                    // BLOCKED(model): no ModelVolume::source. Unreachable in
                    // practice (see above).
                }

                // AMF.cpp:661-667 —
                //   if (m_volume->source.input_file.empty() && type() == MODEL_PART) {
                //       source.object_idx / source.volume_idx = ...;
                //       m_volume->center_geometry_after_creation();
                //   } else
                //       m_volume->center_geometry_after_creation(m_volume->source.input_file.empty());
                // BLOCKED(model): no ModelVolume. `center_geometry_after_creation`
                // translates the volume mesh to its bounding-box center and
                // stores the inverse offset in the volume matrix — the net
                // world-space geometry is unchanged, so skipping it preserves
                // world-space geometry exactly.
                // AMF.cpp:669 — m_volume->calculate_convex_hull();
                // BLOCKED(model): no per-volume convex hull on the simplified Model.

                // AMF.cpp:670-671
                self.volume_facets.clear();
                self.volume = false;
            }

            // AMF.cpp:675-679
            AMFNodeType::Object => {
                debug_assert!(self.object.is_some());
                self.object_vertices.clear();
                self.object = None;
            }

            // AMF.cpp:681-684
            AMFNodeType::Material => {
                debug_assert!(self.material.is_some());
                self.material = None;
            }

            // AMF.cpp:686-689
            AMFNodeType::Instance => {
                debug_assert!(self.instance.is_some());
                self.instance = None;
            }

            // AMF.cpp:691-715 — NODE_TYPE_GCODE_PER_HEIGHT and
            // NODE_TYPE_CUSTOM_GCODE_MODE handling is commented out in C++.

            // AMF.cpp:716-846
            AMFNodeType::Metadata => {
                // AMF.cpp:717-827 — the `slic3rpe_config` / `slic3r.`-prefixed
                // metadata handling (print config, layer height profiles, SLA
                // support points, layer ranges, volume modifier/type/matrix/
                // source_* fields) is entirely commented out in BambuStudio's
                // AMF.cpp; only the generic attribute/name handling below is live.
                if self.path.len() == 3 {
                    if self.path[1] == AMFNodeType::Material {
                        // AMF.cpp:829-831 — m_material->attributes[m_value[0]] = m_value[1];
                        if let Some(material_id) = self.material.clone() {
                            // BLOCKED(model): stored in the context-local map,
                            // not on Model (no ModelMaterial).
                            self.materials
                                .entry(material_id)
                                .or_default()
                                .insert(self.value[0].clone(), self.value[1].clone());
                        }
                    } else if self.path[1] == AMFNodeType::Object {
                        // AMF.cpp:832-835 — m_object->name = std::move(m_value[1]);
                        if let Some(object_idx) = self.object {
                            if self.value[0] == "name" {
                                self.model.objects[object_idx].name =
                                    std::mem::take(&mut self.value[1]);
                            }
                        }
                    }
                } else if self.path.len() == 5 && self.path[3] == AMFNodeType::Volume {
                    // AMF.cpp:836-839 — m_volume->name = std::move(m_value[1]);
                    // BLOCKED(model): the simplified Model has no ModelVolume,
                    // so the parsed volume name is dropped.
                    if self.volume && self.value[0] == "name" {
                        let _volume_name = std::mem::take(&mut self.value[1]);
                    }
                }
                // AMF.cpp:840-842 — SLIC3RPE_AMF_VERSION handling is commented
                // out in C++.

                // AMF.cpp:844-845
                self.value[0].clear();
                self.value[1].clear();
            }
            // AMF.cpp:847-848
            _ => {}
        }
        // AMF.cpp:850
        self.path.pop();
    }

    /// AMF.cpp:853-883 — `void AMFParserContext::endDocument()`
    fn end_document(&mut self) {
        // AMF.cpp:855 — std::map iteration: ordered by object id string
        // (BTreeMap matches).
        for (object_id, object) in &self.object_instances_map {
            // AMF.cpp:856-859
            if object.idx == -1 {
                error!(
                    "Undefined object {} referenced in constellation",
                    object_id
                );
                continue;
            }
            // AMF.cpp:860
            let mut index = 0;
            // AMF.cpp:861
            for instance in &object.instances {
                // AMF.cpp:862 — ModelObject *model_object = m_model.objects[object.second.idx];
                let mut model_object_idx = object.idx as usize;
                // AMF.cpp:863
                index += 1;
                if index > 1 {
                    // AMF.cpp:864-868 — clone the object for additional
                    // instances and clear the clone's instances.
                    let mut new_model_object = self.model.objects[model_object_idx].clone();
                    // new_model_object->clear_instances();
                    new_model_object.instances.clear();
                    self.model.add_object(new_model_object);
                    model_object_idx = self.model.objects.len() - 1;
                }

                // AMF.cpp:870-880
                if instance.anything_set() {
                    let model_object = &mut self.model.objects[model_object_idx];
                    // AMF.cpp:871-873 — mi->set_offset(Vec3d(...));
                    let mi_idx = model_object.add_instance(Point3F::new(
                        if instance.deltax_set {
                            instance.deltax as f64
                        } else {
                            0.0
                        },
                        if instance.deltay_set {
                            instance.deltay as f64
                        } else {
                            0.0
                        },
                        if instance.deltaz_set {
                            instance.deltaz as f64
                        } else {
                            0.0
                        },
                    ));
                    let mi = &mut model_object.instances[mi_idx];
                    // AMF.cpp:874 — mi->set_rotation(Vec3d(rx, ry, rz)) in
                    // radians. BLOCKED(model): the simplified Instance models
                    // only the Z rotation and stores it in degrees; rx/ry are
                    // dropped, rz is converted radians -> degrees so the
                    // resulting transform matches the C++.
                    mi.rotation_z = (if instance.rz_set {
                        instance.rz as f64
                    } else {
                        0.0
                    })
                    .to_degrees();
                    // AMF.cpp:875-876 — mi->set_scaling_factor(Vec3d(...));
                    mi.scale = [
                        if instance.scalex_set {
                            instance.scalex as f64
                        } else {
                            1.0
                        },
                        if instance.scaley_set {
                            instance.scaley as f64
                        } else {
                            1.0
                        },
                        if instance.scalez_set {
                            instance.scalez as f64
                        } else {
                            1.0
                        },
                    ];
                    // AMF.cpp:877-878 — mi->set_mirror(Vec3d(...));
                    // BLOCKED(model): the simplified Instance has no mirror
                    // factors; mirror{x,y,z} are dropped.
                    // AMF.cpp:879
                    mi.printable = instance.printable;
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------
// XML driver (expat replacement)
// ---------------------------------------------------------------------------

/// AMF.cpp:118-127 — `static const char* get_attribute(const char **atts, const char *id)`
fn get_attribute<'a>(atts: &'a [(String, String)], id: &str) -> Option<&'a str> {
    atts.iter()
        .find(|(key, _)| key == id)
        .map(|(_, value)| value.as_str())
}

/// Collect a start tag's attributes as (name, value) pairs, with entities
/// decoded — matching what expat hands to `startElement` (AMF.cpp:99-103).
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

/// `XML_GetCurrentLineNumber` equivalent: 1-based line number of byte offset
/// `pos` (AMF.cpp:919, AMF.cpp:991).
fn line_at(text: &str, pos: usize) -> usize {
    let pos = pos.min(text.len());
    text.as_bytes()[..pos].iter().filter(|&&b| b == b'\n').count() + 1
}

/// Feed the document to the `AMFParserContext` handlers, mirroring the expat
/// callback dispatch (AMF.cpp:99-116) and the parse loop error handling
/// (AMF.cpp:909-926). On failure returns `Err((line, message))` where
/// `message` is either the XML-parser error (`XML_ErrorString`) or the
/// user-code `ctx.error_message()`.
fn parse_buffer(ctx: &mut AMFParserContext, data: &[u8]) -> Result<(), (usize, String)> {
    use quick_xml::events::Event;
    use quick_xml::reader::Reader;

    // expat decodes the document to UTF-8 for the handlers.
    let text = String::from_utf8_lossy(data);
    let mut reader = Reader::from_str(&text);
    // expat does not trim character data: keep quick-xml's default (no trim).
    loop {
        let pos_before = reader.buffer_position();
        match reader.read_event() {
            Ok(Event::Start(e)) => {
                let name = String::from_utf8_lossy(e.name().as_ref()).into_owned();
                let atts = collect_attributes(&e);
                // AMF.cpp:99-103 — startElement callback
                ctx.start_element(&name, &atts);
            }
            Ok(Event::Empty(e)) => {
                // expat reports `<tag/>` as startElement followed by endElement.
                let name = String::from_utf8_lossy(e.name().as_ref()).into_owned();
                let atts = collect_attributes(&e);
                ctx.start_element(&name, &atts);
                if !ctx.error() {
                    ctx.end_element();
                }
            }
            Ok(Event::End(_)) => {
                // AMF.cpp:105-109 — endElement callback
                ctx.end_element();
            }
            Ok(Event::Text(e)) => {
                // AMF.cpp:111-116 — characters callback; expat delivers decoded
                // entities and only reports character data inside the root.
                if !ctx.path.is_empty() {
                    match e.unescape() {
                        Ok(t) => ctx.characters(&t),
                        Err(err) => {
                            return Err((line_at(&text, pos_before), err.to_string()));
                        }
                    }
                }
            }
            Ok(Event::CData(e)) => {
                // expat reports CDATA through the character data handler.
                if !ctx.path.is_empty() {
                    let t = e.into_inner();
                    ctx.characters(&String::from_utf8_lossy(&t));
                }
            }
            // XML declaration, comments, processing instructions and DOCTYPE
            // have no registered expat handlers.
            Ok(Event::Decl(_)) | Ok(Event::Comment(_)) | Ok(Event::PI(_))
            | Ok(Event::DocType(_)) => {}
            Ok(Event::Eof) => return Ok(()),
            Err(err) => {
                // The error was signalled by the XML parser
                // (XML_ErrorString(XML_GetErrorCode(...)), AMF.cpp:91).
                return Err((line_at(&text, reader.buffer_position()), err.to_string()));
            }
        }
        // AMF.cpp:918 — `... || ctx.error()`: XML_StopParser makes XML_Parse
        // fail; the user-code error state is checked after every callback.
        if ctx.error() {
            return Err((
                line_at(&text, reader.buffer_position()),
                ctx.error_message().to_string(),
            ));
        }
    }
}

// ---------------------------------------------------------------------------
// Loading entry points (AMF.cpp:885-1115)
// ---------------------------------------------------------------------------

/// Load an AMF file into a provided model.
/// BBS: add inches check logic
/// AMF.cpp:887-947 — `bool load_amf_file(const char *path, DynamicPrintConfig *config,
///     ConfigSubstitutionContext *config_substitutions, Model *model, bool *use_inches)`
pub fn load_amf_file(
    path: &str,
    config: Option<&mut DynamicPrintConfig>,
    config_substitutions: Option<&mut ConfigSubstitutionContext>,
    model: &mut Model,
    use_inches: Option<&mut bool>,
) -> bool {
    // AMF.cpp:889-890 — (path == nullptr || model == nullptr): encoded in the
    // signature (references cannot be null).

    // AMF.cpp:892-896 — XML_ParserCreate: the quick-xml reader is created in
    // `parse_buffer` and cannot fail to allocate.

    // AMF.cpp:898-902
    let data = match std::fs::read(path) {
        Ok(data) => data,
        Err(_) => {
            error!("Cannot open file {}", path);
            return false;
        }
    };

    // AMF.cpp:904-907
    let mut ctx = AMFParserContext::new(config, config_substitutions, model);

    // AMF.cpp:909-926 — C++ feeds expat 8192-byte chunks from the FILE*; the
    // whole buffer is parsed here, with identical error reporting.
    let result = match parse_buffer(&mut ctx, &data) {
        Ok(()) => true,
        Err((line, message)) => {
            // AMF.cpp:919
            error!("AMF parser: Parse error at line {}: {}", line, message);
            false
        }
    };

    // AMF.cpp:928-929 — XML_ParserFree / fclose: RAII.

    // AMF.cpp:931-932
    if result {
        ctx.end_document();
    }
    // AMF.cpp:933-935
    let ctx_use_inches = ctx.use_inches;
    if let Some(use_inches) = use_inches {
        *use_inches = ctx_use_inches;
    }

    // AMF.cpp:937-944 —
    //   for (ModelObject* o : model->objects)
    //       for (ModelVolume* v : o->volumes)
    //           if (v->source.input_file.empty() && v->type() == MODEL_PART)
    //               v->source.input_file = path;
    // BLOCKED(model): the simplified Model has no ModelVolume::source.

    // AMF.cpp:946
    result
}

/// BBS: add inches logic
/// AMF.cpp:950-1025 — `bool extract_model_from_archive(mz_zip_archive &archive,
///     const mz_zip_archive_file_stat &stat, ...)`
/// The C++ takes a `mz_zip_archive_file_stat`; the Rust port takes the entry
/// index into the open `zip::ZipArchive`.
pub fn extract_model_from_archive(
    archive: &mut zip::ZipArchive<std::fs::File>,
    index: usize,
    config: Option<&mut DynamicPrintConfig>,
    config_substitutions: Option<&mut ConfigSubstitutionContext>,
    model: &mut Model,
    use_inches: Option<&mut bool>,
) -> bool {
    let mut data = Vec::new();
    let filename;
    {
        // AMF.cpp:986-996 — mz_zip_reader_extract_file_to_callback feeds the
        // expat parser chunk by chunk; the whole entry is read here, then parsed.
        let mut file = match archive.by_index(index) {
            Ok(file) => file,
            Err(e) => {
                // AMF.cpp:998-1003
                error!("Error reading AMF file: {}", e);
                // (close_zip_reader: RAII on the caller's archive.)
                return false;
            }
        };
        filename = file.name().to_string();
        // AMF.cpp:952-957 — if (stat.m_uncomp_size == 0)
        if file.size() == 0 {
            error!("Found invalid size");
            return false;
        }

        // AMF.cpp:959-964 — XML_ParserCreate: see load_amf_file.
        // AMF.cpp:966-980 — parser context and CallbackData setup.

        if let Err(e) = file.read_to_end(&mut data) {
            // AMF.cpp:998-1003 — catch (std::exception &e)
            error!("Error reading AMF file: {}", e);
            return false;
        }
    }

    // AMF.cpp:966
    let mut ctx = AMFParserContext::new(config, config_substitutions, model);

    if let Err((line, message)) = parse_buffer(&mut ctx, &data) {
        // AMF.cpp:988-992 — the callback throws
        //   FileIOError("Parsing file %s error at line %d: {%s}")
        // which is caught at AMF.cpp:998-1003 and logged.
        error!(
            "Error reading AMF file: Parsing file {} error at line {}: {{{}}}",
            filename, line, message
        );
        return false;
    }

    // AMF.cpp:1005-1010 — res == 0: covered by the error paths above.

    // AMF.cpp:1012
    ctx.end_document();
    // AMF.cpp:1013-1015
    let ctx_use_inches = ctx.use_inches;
    if let Some(use_inches) = use_inches {
        *use_inches = ctx_use_inches;
    }
    // AMF.cpp:1016-1022 — version compatibility check is commented out in C++.

    // AMF.cpp:1024
    true
}

/// Load an AMF archive into a provided model.
/// AMF.cpp:1028-1092 — `bool load_amf_archive(const char *path, ...)`
pub fn load_amf_archive(
    path: &str,
    mut config: Option<&mut DynamicPrintConfig>,
    mut config_substitutions: Option<&mut ConfigSubstitutionContext>,
    model: &mut Model,
    mut use_inches: Option<&mut bool>,
) -> bool {
    // AMF.cpp:1030-1031 — null checks encoded in the signature.

    // AMF.cpp:1033-1040 — mz_zip_zero_struct / open_zip_reader
    let file = match std::fs::File::open(path) {
        Ok(file) => file,
        Err(_) => {
            error!("Unable to init zip reader");
            return false;
        }
    };
    let mut archive = match zip::ZipArchive::new(file) {
        Ok(archive) => archive,
        Err(_) => {
            error!("Unable to init zip reader");
            return false;
        }
    };

    // AMF.cpp:1042
    let num_entries = archive.len();

    // we first loop the entries to read from the archive the .amf file only,
    // in order to extract the version from it (AMF.cpp:1044-1071)
    for i in 0..num_entries {
        // AMF.cpp:1048 — mz_zip_reader_file_stat: failures skip the entry.
        let entry_name = match archive.by_index_raw(i) {
            Ok(entry) => entry.name().to_string(),
            Err(_) => continue,
        };
        // AMF.cpp:1050 — boost::iends_with(stat.m_filename, ".amf")
        if iends_with(&entry_name, ".amf") {
            // AMF.cpp:1052-1066 — the C++ rethrows exceptions escaping
            // extract_model_from_archive as FileIOError; that path is dead in
            // practice (extract catches std::exception internally) and maps to
            // the `false` return here.
            if !extract_model_from_archive(
                &mut archive,
                i,
                config.as_deref_mut(),
                config_substitutions.as_deref_mut(),
                model,
                use_inches.as_deref_mut(),
            ) {
                // AMF.cpp:1056-1058
                error!("Archive does not contain a valid model");
                return false;
            }
            // AMF.cpp:1068
            break;
        }
    }

    // AMF.cpp:1073-1082 — `#if 0` forward compatibility loop (disabled in C++):
    //   we then loop again the entries to read other files stored in the archive.

    // AMF.cpp:1084 — close_zip_reader: RAII.

    // AMF.cpp:1086-1089 —
    //   for (ModelObject *o : model->objects)
    //       for (ModelVolume *v : o->volumes)
    //           if (v->source.input_file.empty() && v->type() == MODEL_PART)
    //               v->source.input_file = path;
    // BLOCKED(model): the simplified Model has no ModelVolume::source.

    // AMF.cpp:1091
    true
}

/// Load an AMF file into a provided model.
/// If config is not a null pointer, updates it if the amf file/archive contains config data
/// BBS: refine the amf logic
/// AMF.hpp:10 / AMF.cpp:1097-1115 — `bool load_amf(const char *path, DynamicPrintConfig *config,
///     ConfigSubstitutionContext *config_substitutions, Model *model, bool *use_inches)`
pub fn load_amf(
    path: &str,
    config: Option<&mut DynamicPrintConfig>,
    config_substitutions: Option<&mut ConfigSubstitutionContext>,
    model: &mut Model,
    use_inches: Option<&mut bool>,
) -> bool {
    // AMF.cpp:1099 — use "C" locales and point as a decimal separator
    let _locales_setter = CNumericLocalesSetter::new();

    // AMF.cpp:1101
    if iends_with(path, ".amf") {
        // AMF.cpp:1103-1105
        let mut file = match std::fs::File::open(path) {
            Ok(file) => file,
            Err(_) => return false,
        };

        // AMF.cpp:1107-1109 — std::string zip_mask(2, '\0'); file.read(...);
        // a short read leaves the remaining bytes NUL, exactly as in C++.
        let mut zip_mask = [0u8; 2];
        let _ = file.read(&mut zip_mask);
        drop(file);

        // AMF.cpp:1111
        if &zip_mask == b"PK" {
            load_amf_archive(path, config, config_substitutions, model, use_inches)
        } else {
            load_amf_file(path, config, config_substitutions, model, use_inches)
        }
    } else {
        // AMF.cpp:1113-1114
        false
    }
}

// AMF.cpp:1117-1395 — `store_amf` is commented out in BambuStudio ("BBS:
// remove amf export", AMF.hpp:12-15) and is not ported.

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_atof_c_semantics() {
        // atof parses the longest valid prefix and tolerates surrounding text.
        assert_eq!(atof("1.5"), 1.5);
        assert_eq!(atof("  -2.25e1xyz"), -22.5);
        assert_eq!(atof("\n 3"), 3.0);
        assert_eq!(atof("abc"), 0.0);
        assert_eq!(atof(""), 0.0);
    }

    #[test]
    fn test_atoi_c_semantics() {
        assert_eq!(atoi("42"), 42);
        assert_eq!(atoi("  -7rest"), -7);
        assert_eq!(atoi("3.9"), 3);
        assert_eq!(atoi("x1"), 0);
        assert_eq!(atoi(""), 0);
    }

    #[test]
    fn test_iends_with() {
        assert!(iends_with("model.AMF", ".amf"));
        assert!(iends_with("a.zip.amf", ".amf"));
        assert!(!iends_with("model.stl", ".amf"));
        assert!(!iends_with("amf", ".amf"));
    }

    #[test]
    fn test_instance_defaults() {
        // AMF.cpp:177-182
        let instance = Instance::new();
        assert!(!instance.anything_set());
        assert!(instance.printable);
    }

    const MINIMAL_AMF: &str = r#"<?xml version="1.0" encoding="UTF-8"?>
<amf unit="millimeter">
  <object id="0">
    <metadata type="name">tri</metadata>
    <mesh>
      <vertices>
        <vertex><coordinates><x>0</x><y>0</y><z>0</z></coordinates></vertex>
        <vertex><coordinates><x>1</x><y>0</y><z>0</z></coordinates></vertex>
        <vertex><coordinates><x>0</x><y>1</y><z>0</z></coordinates></vertex>
      </vertices>
      <volume>
        <triangle><v1>0</v1><v2>1</v2><v3>2</v3></triangle>
      </volume>
    </mesh>
  </object>
  <constellation id="1">
    <instance objectid="0">
      <deltax>10</deltax>
      <deltay>20</deltay>
      <rz>0</rz>
      <printable>1</printable>
    </instance>
  </constellation>
</amf>"#;

    #[test]
    fn test_minimal_amf_parse() {
        let mut model = Model::new();
        let mut ctx = AMFParserContext::new(None, None, &mut model);
        assert!(parse_buffer(&mut ctx, MINIMAL_AMF.as_bytes()).is_ok());
        assert!(!ctx.error());
        ctx.end_document();
        assert!(!ctx.use_inches);
        assert_eq!(model.objects.len(), 1);
        assert_eq!(model.objects[0].name, "tri");
        assert_eq!(model.objects[0].mesh.vertices().len(), 3);
        assert_eq!(model.objects[0].mesh.indices().len(), 1);
        // One constellation instance with offset (10, 20, 0).
        assert_eq!(model.objects[0].instances.len(), 1);
        let instance = &model.objects[0].instances[0];
        assert_eq!(instance.position, Point3F::new(10.0, 20.0, 0.0));
        assert!(instance.printable);
    }

    #[test]
    fn test_load_amf_rejects_non_amf_extension() {
        // AMF.cpp:1113-1114
        let mut model = Model::new();
        assert!(!load_amf("/nonexistent/file.stl", None, None, &mut model, None));
    }

    #[test]
    fn test_empty_volume_stops_parser() {
        // AMF.cpp:626-629
        let xml = r#"<amf><object id="0"><mesh><vertices></vertices><volume></volume></mesh></object></amf>"#;
        let mut model = Model::new();
        let mut ctx = AMFParserContext::new(None, None, &mut model);
        let result = parse_buffer(&mut ctx, xml.as_bytes());
        assert!(result.is_err());
        assert_eq!(ctx.error_message(), "Found an empty triangle mesh");
    }

    #[test]
    fn test_inch_units_flag() {
        // AMF.cpp:279-281
        let xml = r#"<amf unit="inch"></amf>"#;
        let mut model = Model::new();
        let mut ctx = AMFParserContext::new(None, None, &mut model);
        assert!(parse_buffer(&mut ctx, xml.as_bytes()).is_ok());
        assert!(ctx.use_inches);
    }
}
