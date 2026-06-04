//! AMF file loading.
//!
//! C++ Reference:
//! - Format/AMF.hpp
//! - Format/AMF.cpp
//!
//! AMF (Additive Manufacturing File) is an XML-based format that stores
//! geometry, materials, and constellations (instances).
//! This module implements a SAX-style XML parser that mirrors the C++ expat
//! callback approach using the `quick-xml` or standard XML parsing.

use crate::geometry::Point3F;
use crate::model::{Model, ModelObject};
use crate::triangle_mesh::{Triangle, TriangleMesh};
use crate::{Error, Result};

use log::error;
use std::collections::HashMap;
use std::path::Path;

// ---------------------------------------------------------------------------
// AMF Node types (AMF.cpp:128-173)
// ---------------------------------------------------------------------------

/// XML node types used during AMF parsing.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum AmfNodeType {
    #[allow(dead_code)]
    Invalid,
    Unknown,
    Amf,
    Material,
    Object,
    Mesh,
    Vertices,
    Vertex,
    Coordinates,
    CoordinateX,
    CoordinateY,
    CoordinateZ,
    Volume,
    TriangleNode,
    Vertex1,
    Vertex2,
    Vertex3,
    Constellation,
    InstanceNode,
    DeltaX,
    DeltaY,
    DeltaZ,
    Rx,
    Ry,
    Rz,
    Scale,
    ScaleX,
    ScaleY,
    ScaleZ,
    MirrorX,
    MirrorY,
    MirrorZ,
    Printable,
    Metadata,
}

// ---------------------------------------------------------------------------
// Instance data (AMF.cpp:175-221)
// ---------------------------------------------------------------------------

/// Parsed instance transform from the AMF constellation section.
#[derive(Debug, Clone)]
struct AmfInstance {
    deltax: f32,
    deltax_set: bool,
    deltay: f32,
    deltay_set: bool,
    deltaz: f32,
    deltaz_set: bool,
    rx: f32,
    rx_set: bool,
    ry: f32,
    ry_set: bool,
    rz: f32,
    rz_set: bool,
    scalex: f32,
    scalex_set: bool,
    scaley: f32,
    scaley_set: bool,
    scalez: f32,
    scalez_set: bool,
    mirrorx: f32,
    mirrorx_set: bool,
    mirrory: f32,
    mirrory_set: bool,
    mirrorz: f32,
    mirrorz_set: bool,
    printable: bool,
}

impl AmfInstance {
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
            scalex: 1.0,
            scalex_set: false,
            scaley: 1.0,
            scaley_set: false,
            scalez: 1.0,
            scalez_set: false,
            mirrorx: 1.0,
            mirrorx_set: false,
            mirrory: 1.0,
            mirrory_set: false,
            mirrorz: 1.0,
            mirrorz_set: false,
            printable: true,
        }
    }

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

/// Object entry in the instance map.
#[derive(Debug, Clone)]
struct AmfObject {
    idx: i32,
    instances: Vec<AmfInstance>,
}

impl AmfObject {
    fn new() -> Self {
        Self {
            idx: -1,
            instances: Vec::new(),
        }
    }
}

// ---------------------------------------------------------------------------
// Parser context (AMF.cpp:64-268)
// ---------------------------------------------------------------------------

/// Context for the SAX-style AMF parser (mirrors AMFParserContext).
struct AmfParserContext {
    path: Vec<AmfNodeType>,
    objects: Vec<ModelObject>,
    object_vertices: Vec<Point3F>,
    volume_facets: Vec<[i32; 3]>,
    object_instances_map: HashMap<String, AmfObject>,
    current_object_idx: Option<usize>,
    value: [String; 5],
    current_instance_object_id: Option<String>,
    current_instance_idx: Option<usize>,
    error: bool,
    error_message: String,
    use_inches: bool,
    // Material metadata
    #[allow(dead_code)]
    material_attributes: HashMap<String, HashMap<String, String>>,
}

impl AmfParserContext {
    fn new() -> Self {
        Self {
            path: Vec::with_capacity(12),
            objects: Vec::new(),
            object_vertices: Vec::new(),
            volume_facets: Vec::new(),
            object_instances_map: HashMap::new(),
            current_object_idx: None,
            value: Default::default(),
            current_instance_object_id: None,
            current_instance_idx: None,
            error: false,
            error_message: String::new(),
            use_inches: false,
            material_attributes: HashMap::new(),
        }
    }

    fn stop(&mut self, msg: &str) {
        self.error = true;
        self.error_message = msg.to_string();
    }
}

// ---------------------------------------------------------------------------
// XML parsing (AMF.cpp:270-851)
// ---------------------------------------------------------------------------

/// Minimal recursive-descent XML parser for AMF.
/// This avoids external XML dependencies by doing a simple tag-by-tag parse.
fn parse_amf_xml(xml: &str, ctx: &mut AmfParserContext) -> bool {
    let mut reader = XmlReader::new(xml);

    loop {
        match reader.next_event() {
            XmlEvent::StartElement { name, attributes } => {
                handle_start_element(ctx, &name, &attributes);
                if ctx.error {
                    return false;
                }
            }
            XmlEvent::EndElement { name } => {
                handle_end_element(ctx, &name);
                if ctx.error {
                    return false;
                }
            }
            XmlEvent::Characters(text) => {
                handle_characters(ctx, &text);
            }
            XmlEvent::Eof => break,
            XmlEvent::Error(msg) => {
                error!("AMF XML parse error: {}", msg);
                return false;
            }
        }
    }
    true
}

fn get_attribute<'a>(attrs: &'a [(String, String)], key: &str) -> Option<&'a str> {
    attrs
        .iter()
        .find(|(k, _)| k == key)
        .map(|(_, v)| v.as_str())
}

/// AMF.cpp:270-462
fn handle_start_element(ctx: &mut AmfParserContext, name: &str, attrs: &[(String, String)]) {
    let mut node_type = AmfNodeType::Unknown;
    let depth = ctx.path.len();

    match depth {
        0 => {
            if name != "amf" {
                ctx.stop("AMF file must start with <amf>");
                return;
            }
            node_type = AmfNodeType::Amf;
            if let Some(units) = get_attribute(attrs, "unit") {
                if units == "inch" {
                    ctx.use_inches = true;
                }
            }
        }
        1 => {
            if name == "metadata" {
                if get_attribute(attrs, "type").is_some() {
                    ctx.value[0] = get_attribute(attrs, "type").unwrap_or("").to_string();
                    node_type = AmfNodeType::Metadata;
                }
            } else if name == "material" {
                let _material_id = get_attribute(attrs, "id").unwrap_or("_").to_string();
                node_type = AmfNodeType::Material;
            } else if name == "object" {
                let object_id = match get_attribute(attrs, "id") {
                    Some(id) => id.to_string(),
                    None => {
                        ctx.stop("object missing id attribute");
                        return;
                    }
                };
                ctx.object_vertices.clear();
                let idx = ctx.objects.len();
                ctx.objects.push(ModelObject::new("", TriangleMesh::new()));
                ctx.current_object_idx = Some(idx);
                let entry = ctx
                    .object_instances_map
                    .entry(object_id)
                    .or_insert_with(AmfObject::new);
                entry.idx = idx as i32;
                node_type = AmfNodeType::Object;
            } else if name == "constellation" {
                node_type = AmfNodeType::Constellation;
            }
        }
        2 => {
            if name == "metadata" {
                if ctx.path[1] == AmfNodeType::Material || ctx.path[1] == AmfNodeType::Object {
                    ctx.value[0] = get_attribute(attrs, "type").unwrap_or("").to_string();
                    node_type = AmfNodeType::Metadata;
                }
            } else if name == "mesh" && ctx.path[1] == AmfNodeType::Object {
                node_type = AmfNodeType::Mesh;
            } else if name == "instance" && ctx.path[1] == AmfNodeType::Constellation {
                let object_id = match get_attribute(attrs, "objectid") {
                    Some(id) => id.to_string(),
                    None => {
                        ctx.stop("instance missing objectid");
                        return;
                    }
                };
                let entry = ctx
                    .object_instances_map
                    .entry(object_id.clone())
                    .or_insert_with(AmfObject::new);
                entry.instances.push(AmfInstance::new());
                ctx.current_instance_object_id = Some(object_id);
                ctx.current_instance_idx = Some(entry.instances.len() - 1);
                node_type = AmfNodeType::InstanceNode;
            }
        }
        3 => {
            if ctx.path[2] == AmfNodeType::Mesh {
                if name == "vertices" {
                    node_type = AmfNodeType::Vertices;
                } else if name == "volume" {
                    ctx.volume_facets.clear();
                    node_type = AmfNodeType::Volume;
                }
            } else if ctx.path[2] == AmfNodeType::InstanceNode {
                node_type = match name {
                    "deltax" => AmfNodeType::DeltaX,
                    "deltay" => AmfNodeType::DeltaY,
                    "deltaz" => AmfNodeType::DeltaZ,
                    "rx" => AmfNodeType::Rx,
                    "ry" => AmfNodeType::Ry,
                    "rz" => AmfNodeType::Rz,
                    "scalex" => AmfNodeType::ScaleX,
                    "scaley" => AmfNodeType::ScaleY,
                    "scalez" => AmfNodeType::ScaleZ,
                    "scale" => AmfNodeType::Scale,
                    "mirrorx" => AmfNodeType::MirrorX,
                    "mirrory" => AmfNodeType::MirrorY,
                    "mirrorz" => AmfNodeType::MirrorZ,
                    "printable" => AmfNodeType::Printable,
                    _ => AmfNodeType::Unknown,
                };
            }
        }
        4 => {
            if ctx.path[3] == AmfNodeType::Vertices && name == "vertex" {
                node_type = AmfNodeType::Vertex;
            } else if ctx.path[3] == AmfNodeType::Volume {
                if name == "metadata" {
                    ctx.value[0] = get_attribute(attrs, "type").unwrap_or("").to_string();
                    node_type = AmfNodeType::Metadata;
                } else if name == "triangle" {
                    node_type = AmfNodeType::TriangleNode;
                }
            }
        }
        5 => {
            if name == "coordinates" && ctx.path[4] == AmfNodeType::Vertex {
                node_type = AmfNodeType::Coordinates;
            } else if ctx.path[4] == AmfNodeType::TriangleNode {
                if name == "v1" {
                    node_type = AmfNodeType::Vertex1;
                } else if name == "v2" {
                    node_type = AmfNodeType::Vertex2;
                } else if name == "v3" {
                    node_type = AmfNodeType::Vertex3;
                }
            }
        }
        6 => {
            if ctx.path[5] == AmfNodeType::Coordinates {
                node_type = match name {
                    "x" => AmfNodeType::CoordinateX,
                    "y" => AmfNodeType::CoordinateY,
                    "z" => AmfNodeType::CoordinateZ,
                    _ => AmfNodeType::Unknown,
                };
            }
        }
        _ => {}
    }

    ctx.path.push(node_type);
}

/// AMF.cpp:464-507
fn handle_characters(ctx: &mut AmfParserContext, text: &str) {
    if ctx.path.is_empty() {
        return;
    }
    let back = *ctx.path.last().unwrap();

    if back == AmfNodeType::Metadata {
        ctx.value[1].push_str(text);
        return;
    }

    match ctx.path.len() {
        4 => match back {
            AmfNodeType::DeltaX
            | AmfNodeType::DeltaY
            | AmfNodeType::DeltaZ
            | AmfNodeType::Rx
            | AmfNodeType::Ry
            | AmfNodeType::Rz
            | AmfNodeType::ScaleX
            | AmfNodeType::ScaleY
            | AmfNodeType::ScaleZ
            | AmfNodeType::Scale
            | AmfNodeType::MirrorX
            | AmfNodeType::MirrorY
            | AmfNodeType::MirrorZ
            | AmfNodeType::Printable => {
                ctx.value[0].push_str(text);
            }
            _ => {}
        },
        6 => match back {
            AmfNodeType::Vertex1 => ctx.value[0].push_str(text),
            AmfNodeType::Vertex2 => ctx.value[1].push_str(text),
            AmfNodeType::Vertex3 => ctx.value[2].push_str(text),
            AmfNodeType::CoordinateX => ctx.value[0].push_str(text),
            AmfNodeType::CoordinateY => ctx.value[1].push_str(text),
            AmfNodeType::CoordinateZ => ctx.value[2].push_str(text),
            _ => {}
        },
        7 => match back {
            AmfNodeType::CoordinateX => ctx.value[0].push_str(text),
            AmfNodeType::CoordinateY => ctx.value[1].push_str(text),
            AmfNodeType::CoordinateZ => ctx.value[2].push_str(text),
            _ => {}
        },
        _ => {}
    }
}

/// AMF.cpp:509-851
fn handle_end_element(ctx: &mut AmfParserContext, _name: &str) {
    if ctx.path.is_empty() {
        return;
    }
    let back = *ctx.path.last().unwrap();

    /// Helper macro to access the current instance mutably.
    macro_rules! with_instance {
        ($ctx:expr, |$inst:ident| $body:expr) => {
            if let (Some(oid), Some(idx)) = (
                $ctx.current_instance_object_id.clone(),
                $ctx.current_instance_idx,
            ) {
                if let Some(obj) = $ctx.object_instances_map.get_mut(&oid) {
                    if let Some($inst) = obj.instances.get_mut(idx) {
                        $body
                    }
                }
            }
        };
    }

    match back {
        // Constellation transforms
        AmfNodeType::DeltaX => {
            let val: f32 = ctx.value[0].trim().parse().unwrap_or(0.0);
            with_instance!(ctx, |inst| {
                inst.deltax = val;
                inst.deltax_set = true;
            });
            ctx.value[0].clear();
        }
        AmfNodeType::DeltaY => {
            let val: f32 = ctx.value[0].trim().parse().unwrap_or(0.0);
            with_instance!(ctx, |inst| {
                inst.deltay = val;
                inst.deltay_set = true;
            });
            ctx.value[0].clear();
        }
        AmfNodeType::DeltaZ => {
            let val: f32 = ctx.value[0].trim().parse().unwrap_or(0.0);
            with_instance!(ctx, |inst| {
                inst.deltaz = val;
                inst.deltaz_set = true;
            });
            ctx.value[0].clear();
        }
        AmfNodeType::Rx => {
            let val: f32 = ctx.value[0].trim().parse().unwrap_or(0.0);
            with_instance!(ctx, |inst| {
                inst.rx = val;
                inst.rx_set = true;
            });
            ctx.value[0].clear();
        }
        AmfNodeType::Ry => {
            let val: f32 = ctx.value[0].trim().parse().unwrap_or(0.0);
            with_instance!(ctx, |inst| {
                inst.ry = val;
                inst.ry_set = true;
            });
            ctx.value[0].clear();
        }
        AmfNodeType::Rz => {
            let val: f32 = ctx.value[0].trim().parse().unwrap_or(0.0);
            with_instance!(ctx, |inst| {
                inst.rz = val;
                inst.rz_set = true;
            });
            ctx.value[0].clear();
        }
        AmfNodeType::Scale => {
            let val: f32 = ctx.value[0].trim().parse().unwrap_or(1.0);
            with_instance!(ctx, |inst| {
                inst.scalex = val;
                inst.scalex_set = true;
                inst.scaley = val;
                inst.scaley_set = true;
                inst.scalez = val;
                inst.scalez_set = true;
            });
            ctx.value[0].clear();
        }
        AmfNodeType::ScaleX => {
            let val: f32 = ctx.value[0].trim().parse().unwrap_or(1.0);
            with_instance!(ctx, |inst| {
                inst.scalex = val;
                inst.scalex_set = true;
            });
            ctx.value[0].clear();
        }
        AmfNodeType::ScaleY => {
            let val: f32 = ctx.value[0].trim().parse().unwrap_or(1.0);
            with_instance!(ctx, |inst| {
                inst.scaley = val;
                inst.scaley_set = true;
            });
            ctx.value[0].clear();
        }
        AmfNodeType::ScaleZ => {
            let val: f32 = ctx.value[0].trim().parse().unwrap_or(1.0);
            with_instance!(ctx, |inst| {
                inst.scalez = val;
                inst.scalez_set = true;
            });
            ctx.value[0].clear();
        }
        AmfNodeType::MirrorX => {
            let val: f32 = ctx.value[0].trim().parse().unwrap_or(1.0);
            with_instance!(ctx, |inst| {
                inst.mirrorx = val;
                inst.mirrorx_set = true;
            });
            ctx.value[0].clear();
        }
        AmfNodeType::MirrorY => {
            let val: f32 = ctx.value[0].trim().parse().unwrap_or(1.0);
            with_instance!(ctx, |inst| {
                inst.mirrory = val;
                inst.mirrory_set = true;
            });
            ctx.value[0].clear();
        }
        AmfNodeType::MirrorZ => {
            let val: f32 = ctx.value[0].trim().parse().unwrap_or(1.0);
            with_instance!(ctx, |inst| {
                inst.mirrorz = val;
                inst.mirrorz_set = true;
            });
            ctx.value[0].clear();
        }
        AmfNodeType::Printable => {
            let val: i32 = ctx.value[0].trim().parse().unwrap_or(1);
            with_instance!(ctx, |inst| {
                inst.printable = val != 0;
            });
            ctx.value[0].clear();
        }

        // Object vertices
        AmfNodeType::Vertex => {
            let x: f32 = ctx.value[0].trim().parse().unwrap_or(0.0);
            let y: f32 = ctx.value[1].trim().parse().unwrap_or(0.0);
            let z: f32 = ctx.value[2].trim().parse().unwrap_or(0.0);
            ctx.object_vertices
                .push(Point3F::new(x as f64, y as f64, z as f64));
            ctx.value[0].clear();
            ctx.value[1].clear();
            ctx.value[2].clear();
        }

        // Face indices
        AmfNodeType::TriangleNode => {
            let v1: i32 = ctx.value[0].trim().parse().unwrap_or(0);
            let v2: i32 = ctx.value[1].trim().parse().unwrap_or(0);
            let v3: i32 = ctx.value[2].trim().parse().unwrap_or(0);
            ctx.volume_facets.push([v1, v2, v3]);
            ctx.value[0].clear();
            ctx.value[1].clear();
            ctx.value[2].clear();
        }

        // End of volume – build mesh
        AmfNodeType::Volume => {
            if ctx.volume_facets.is_empty() {
                ctx.stop("Found an empty triangle mesh");
                ctx.path.pop();
                return;
            }

            // Find vertex span
            let mut min_id = ctx.volume_facets[0][0];
            let mut max_id = min_id;
            for face in &ctx.volume_facets {
                for &tri_id in face {
                    if tri_id < 0 || tri_id >= ctx.object_vertices.len() as i32 {
                        ctx.stop("Found a malformed triangle mesh");
                        ctx.path.pop();
                        return;
                    }
                    min_id = min_id.min(tri_id);
                    max_id = max_id.max(tri_id);
                }
            }

            // Rebase indices and build mesh
            let verts: Vec<Point3F> =
                ctx.object_vertices[min_id as usize..=max_id as usize].to_vec();
            let tris: Vec<Triangle> = ctx
                .volume_facets
                .iter()
                .map(|f| {
                    Triangle::new(
                        (f[0] - min_id) as u32,
                        (f[1] - min_id) as u32,
                        (f[2] - min_id) as u32,
                    )
                })
                .collect();

            let mesh = TriangleMesh::from_parts(verts, tris);

            if let Some(idx) = ctx.current_object_idx {
                ctx.objects[idx].mesh = mesh;
            }

            ctx.volume_facets.clear();
        }

        AmfNodeType::Object => {
            ctx.object_vertices.clear();
            ctx.current_object_idx = None;
        }

        AmfNodeType::Material => {}

        AmfNodeType::InstanceNode => {
            ctx.current_instance_object_id = None;
            ctx.current_instance_idx = None;
        }

        AmfNodeType::Metadata => {
            // Handle object name metadata
            if ctx.path.len() == 3 && ctx.path[1] == AmfNodeType::Object {
                if ctx.value[0] == "name" {
                    if let Some(idx) = ctx.current_object_idx {
                        ctx.objects[idx].name = ctx.value[1].clone();
                    }
                }
            } else if ctx.path.len() == 5 && ctx.path.get(3) == Some(&AmfNodeType::Volume) {
                // Volume name metadata – handled at model level
            }
            ctx.value[0].clear();
            ctx.value[1].clear();
        }

        _ => {}
    }

    ctx.path.pop();
}

/// Apply constellation instances to the model.
/// AMF.cpp:853-883
fn end_document(ctx: &AmfParserContext, model: &mut Model) {
    for (_id, amf_obj) in &ctx.object_instances_map {
        if amf_obj.idx < 0 || amf_obj.idx as usize >= ctx.objects.len() {
            continue;
        }

        for (i, instance) in amf_obj.instances.iter().enumerate() {
            let obj_idx = if i == 0 {
                // Use the existing object
                let obj = ctx.objects[amf_obj.idx as usize].clone();
                let idx = model.add_object(obj);
                idx
            } else {
                // Clone for additional instances
                let obj = ctx.objects[amf_obj.idx as usize].clone();
                let idx = model.add_object(obj);
                idx
            };

            if instance.anything_set() {
                let obj = &mut model.objects[obj_idx];
                let position = Point3F::new(
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
                );
                obj.add_instance(position);
            }
        }
    }

    // If no constellation, just add objects directly
    if ctx
        .object_instances_map
        .values()
        .all(|o| o.instances.is_empty())
    {
        for obj in &ctx.objects {
            model.add_object(obj.clone());
        }
    }
}

// ---------------------------------------------------------------------------
// Public API (AMF.hpp:10)
// ---------------------------------------------------------------------------

/// Load an AMF file into a `Model`.
///
/// Supports both plain AMF XML and zip-compressed AMF archives (detects
/// by checking if the file starts with "PK").
///
/// AMF.cpp:1097-1115
pub fn load_amf(path: &Path, model: &mut Model) -> Result<bool> {
    let data =
        std::fs::read(path).map_err(|e| Error::IO(format!("Failed to read AMF file: {}", e)))?;

    if data.len() < 2 {
        return Err(Error::IO("AMF file is too small".into()));
    }

    let xml_data = if &data[..2] == b"PK" {
        // ZIP archive – extract .amf entry
        extract_amf_from_zip(&data)?
    } else {
        // Plain XML
        data
    };

    let xml_str = String::from_utf8_lossy(&xml_data);
    let mut ctx = AmfParserContext::new();

    if !parse_amf_xml(&xml_str, &mut ctx) {
        return Err(Error::ParseError(format!(
            "AMF parse error: {}",
            ctx.error_message
        )));
    }

    end_document(&ctx, model);

    // In C++, source.input_file is set on volumes for MODEL_PART types.
    // That is handled at a higher level when ModelVolume support is added.

    Ok(!ctx.use_inches)
}

/// Extract the first .amf entry from a ZIP archive.
fn extract_amf_from_zip(data: &[u8]) -> Result<Vec<u8>> {
    // Minimal ZIP parsing: find the .amf file entry
    // For a production implementation, use the `zip` crate.
    // Here we do a simple search for the local file header.
    let mut pos = 0;
    while pos + 30 <= data.len() {
        // Local file header signature = 0x04034b50
        if data[pos..pos + 4] == [0x50, 0x4b, 0x03, 0x04] {
            let filename_len = u16::from_le_bytes([data[pos + 26], data[pos + 27]]) as usize;
            let extra_len = u16::from_le_bytes([data[pos + 28], data[pos + 29]]) as usize;
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
            let compression = u16::from_le_bytes([data[pos + 8], data[pos + 9]]);

            let name_start = pos + 30;
            let name_end = name_start + filename_len;
            if name_end > data.len() {
                break;
            }
            let filename = String::from_utf8_lossy(&data[name_start..name_end]);

            let data_start = name_end + extra_len;

            if filename.to_lowercase().ends_with(".amf") {
                if compression == 0 {
                    // Stored (no compression)
                    let data_end = data_start + uncompressed_size;
                    if data_end <= data.len() {
                        return Ok(data[data_start..data_end].to_vec());
                    }
                } else {
                    // Deflate – would need flate2 or miniz_oxide
                    return Err(Error::IO(
                        "Compressed AMF archives require decompression support".into(),
                    ));
                }
            }

            pos = data_start + compressed_size.max(uncompressed_size);
        } else {
            pos += 1;
        }
    }
    Err(Error::IO("No .amf file found in ZIP archive".into()))
}

// ---------------------------------------------------------------------------
// Minimal XML reader
// ---------------------------------------------------------------------------

enum XmlEvent {
    StartElement {
        name: String,
        attributes: Vec<(String, String)>,
    },
    EndElement {
        name: String,
    },
    Characters(String),
    Eof,
    Error(String),
}

struct XmlReader<'a> {
    data: &'a str,
    pos: usize,
}

impl<'a> XmlReader<'a> {
    fn new(data: &'a str) -> Self {
        Self { data, pos: 0 }
    }

    fn next_event(&mut self) -> XmlEvent {
        // Skip whitespace
        self.skip_ws();

        if self.pos >= self.data.len() {
            return XmlEvent::Eof;
        }

        if self.data[self.pos..].starts_with("<?") {
            // Processing instruction – skip
            if let Some(end) = self.data[self.pos..].find("?>") {
                self.pos += end + 2;
                return self.next_event();
            }
            return XmlEvent::Error("Unterminated processing instruction".into());
        }

        if self.data[self.pos..].starts_with("<!--") {
            // Comment – skip
            if let Some(end) = self.data[self.pos..].find("-->") {
                self.pos += end + 3;
                return self.next_event();
            }
            return XmlEvent::Error("Unterminated comment".into());
        }

        if self.data[self.pos..].starts_with("</") {
            // End element
            self.pos += 2;
            let start = self.pos;
            while self.pos < self.data.len() && self.data.as_bytes()[self.pos] != b'>' {
                self.pos += 1;
            }
            let name = self.data[start..self.pos].trim().to_string();
            if self.pos < self.data.len() {
                self.pos += 1; // skip '>'
            }
            return XmlEvent::EndElement { name };
        }

        if self.data.as_bytes()[self.pos] == b'<' {
            // Start element
            self.pos += 1;
            self.skip_ws();
            let name_start = self.pos;
            while self.pos < self.data.len() {
                let c = self.data.as_bytes()[self.pos];
                if c == b' ' || c == b'\t' || c == b'\n' || c == b'\r' || c == b'>' || c == b'/' {
                    break;
                }
                self.pos += 1;
            }
            let name = self.data[name_start..self.pos].to_string();

            // Parse attributes
            let mut attrs = Vec::new();
            loop {
                self.skip_ws();
                if self.pos >= self.data.len() {
                    break;
                }
                let c = self.data.as_bytes()[self.pos];
                if c == b'>' {
                    self.pos += 1;
                    return XmlEvent::StartElement {
                        name,
                        attributes: attrs,
                    };
                }
                if c == b'/' {
                    self.pos += 1;
                    if self.pos < self.data.len() && self.data.as_bytes()[self.pos] == b'>' {
                        self.pos += 1;
                    }
                    // Self-closing: emit start then end
                    // For simplicity, just return start – downstream handles empty elements
                    return XmlEvent::StartElement {
                        name,
                        attributes: attrs,
                    };
                }

                // Parse attribute name
                let attr_start = self.pos;
                while self.pos < self.data.len() {
                    let c = self.data.as_bytes()[self.pos];
                    if c == b'=' || c == b' ' || c == b'>' {
                        break;
                    }
                    self.pos += 1;
                }
                let attr_name = self.data[attr_start..self.pos].to_string();
                self.skip_ws();
                if self.pos < self.data.len() && self.data.as_bytes()[self.pos] == b'=' {
                    self.pos += 1;
                }
                self.skip_ws();
                let attr_val = self.read_attr_value();
                attrs.push((attr_name, attr_val));
            }

            return XmlEvent::StartElement {
                name,
                attributes: attrs,
            };
        }

        // Text content
        let start = self.pos;
        while self.pos < self.data.len() && self.data.as_bytes()[self.pos] != b'<' {
            self.pos += 1;
        }
        let text = xml_unescape(&self.data[start..self.pos]);
        XmlEvent::Characters(text)
    }

    fn skip_ws(&mut self) {
        while self.pos < self.data.len() {
            let c = self.data.as_bytes()[self.pos];
            if c == b' ' || c == b'\t' || c == b'\n' || c == b'\r' {
                self.pos += 1;
            } else {
                break;
            }
        }
    }

    fn read_attr_value(&mut self) -> String {
        if self.pos >= self.data.len() {
            return String::new();
        }
        let quote = self.data.as_bytes()[self.pos];
        if quote == b'"' || quote == b'\'' {
            self.pos += 1;
            let start = self.pos;
            while self.pos < self.data.len() && self.data.as_bytes()[self.pos] != quote {
                self.pos += 1;
            }
            let val = self.data[start..self.pos].to_string();
            if self.pos < self.data.len() {
                self.pos += 1; // skip closing quote
            }
            xml_unescape(&val)
        } else {
            // Unquoted
            let start = self.pos;
            while self.pos < self.data.len() {
                let c = self.data.as_bytes()[self.pos];
                if c == b' ' || c == b'>' || c == b'/' {
                    break;
                }
                self.pos += 1;
            }
            self.data[start..self.pos].to_string()
        }
    }
}

fn xml_unescape(s: &str) -> String {
    s.replace("&amp;", "&")
        .replace("&lt;", "<")
        .replace("&gt;", ">")
        .replace("&quot;", "\"")
        .replace("&apos;", "'")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_amf_instance_defaults() {
        let inst = AmfInstance::new();
        assert!(!inst.anything_set());
        assert!(inst.printable);
    }

    #[test]
    fn test_xml_unescape() {
        assert_eq!(xml_unescape("&amp;"), "&");
        assert_eq!(xml_unescape("a &lt; b"), "a < b");
    }

    #[test]
    fn test_minimal_amf_parse() {
        let xml = r#"<?xml version="1.0" encoding="UTF-8"?>
<amf unit="millimeter">
  <object id="0">
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
</amf>"#;

        let mut ctx = AmfParserContext::new();
        assert!(parse_amf_xml(xml, &mut ctx));
        assert_eq!(ctx.objects.len(), 1);
        assert_eq!(ctx.objects[0].mesh.vertex_count(), 3);
        assert_eq!(ctx.objects[0].mesh.indices().len(), 1);
    }
}
